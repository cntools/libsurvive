#define SURVIVE_ENABLE_FULL_API

#include <libsurvive/survive_api.h>
#include <libsurvive/survive.h>
#include <os_generic.h>

#include "ros/ros.h"
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

double ros_offset = 0;
SurviveSimpleContext *actx = 0;
std::unique_ptr<ros::NodeHandle> n;

static ros::Publisher& rootJoyPublisher() {
	static std::unique_ptr<ros::Publisher> root;
	if(!root) {
		root = std::make_unique<ros::Publisher>(n->advertise<sensor_msgs::Joy>("/joy", 1));
	}
	return *root;
}

static ros::Time rostime_from_survivetime(FLT timecode) {
    return ros::Time().fromSec(timecode + ros_offset);
}

static std::string sanitize(const std::string& serial) {
    std::string r = serial;
    for(char & i : r) {
        if(i == '-') i = '_';
    }
    return r;
}
struct ObjectPublishers {
    ros::NodeHandle obj_n;
    ros::Publisher joyPublisher, imuPublisher, configPublisher;
    SurviveObject* so;

    ObjectPublishers(SurviveObject* so) : obj_n(*n, sanitize(so->serial_number)), so(so) {
        assert(so != 0);
        configPublisher = obj_n.advertise<std_msgs::String>("config", 1, true);
        joyPublisher = obj_n.advertise<sensor_msgs::Joy>("joy", 1);
        imuPublisher = obj_n.advertise<sensor_msgs::Imu>("imu", 1);
    }

    int joy_seq = 0, imu_seq = 0;
    std::string serial_number() const {
        return so->serial_number;
    }
    void publish(const struct SurviveSimpleButtonEvent *button_event) {
        sensor_msgs::Joy joyMsg;
        auto obj = button_event->object;
        joyMsg.header.frame_id = survive_simple_serial_number(button_event->object);
        joyMsg.header.seq = joy_seq++;
        joyMsg.header.stamp = rostime_from_survivetime(button_event->time);

        joyMsg.axes.resize(SURVIVE_MAX_AXIS_COUNT);
        joyMsg.buttons.resize(SURVIVE_BUTTON_MAX * 2);

        int64_t mask = survive_simple_object_get_button_mask(obj) | (survive_simple_object_get_touch_mask(obj) << SURVIVE_BUTTON_MAX);
        for(int i = 0;i < SURVIVE_MAX_AXIS_COUNT;i++) {
            joyMsg.axes[i] = (float)survive_simple_object_get_input_axis(obj, (enum SurviveAxis)i);
        }
        for(int i = 0;i < mask && i < joyMsg.buttons.size();i++) {
            joyMsg.buttons[i] = (mask >> i) & 1;
        }

        joyPublisher.publish(joyMsg);
		rootJoyPublisher().publish(joyMsg);
    }
    void publish(const struct SurviveSimpleConfigEvent *cfg_event) const {
        std_msgs::String cfgMsg;
        cfgMsg.data = cfg_event->cfg;
        configPublisher.publish(cfgMsg);
    }
    void publish(int mask, const FLT *accelgyromag, uint32_t timecode, int id) {
        sensor_msgs::Imu imu;
        imu.header.frame_id = std::string(serial_number()) + "_imu";
        imu.header.seq = imu_seq++;
        imu.header.stamp = rostime_from_survivetime(1e-6 * SurviveSensorActivations_runtime(&so->activations, so->activations.last_imu));

        imu.angular_velocity.x = accelgyromag[3];
        imu.angular_velocity.y = accelgyromag[4];
        imu.angular_velocity.z = accelgyromag[5];

        imu.linear_acceleration.x = accelgyromag[0] * 9.80665;
        imu.linear_acceleration.y = accelgyromag[1] * 9.80665;
        imu.linear_acceleration.z = accelgyromag[2] * 9.80665;

        imuPublisher.publish(imu);
    }
};

std::map<std::string, std::shared_ptr<ObjectPublishers>> objectPublishers;
std::shared_ptr<ObjectPublishers> getPublishers(SurviveObject* so) {
    if(so == 0) return 0;

    std::string serial = so->serial_number;
    if(objectPublishers[serial] == 0) {
        objectPublishers[serial] = std::make_shared<ObjectPublishers>(so);
    }
    return objectPublishers[serial];
}
std::shared_ptr<ObjectPublishers> getPublishers(SurviveSimpleObject * aso) {
    return getPublishers(survive_simple_get_survive_object(aso));
}

static geometry_msgs::Transform ros_from_pose(const SurvivePose* pose) {
    geometry_msgs::Transform tx;
    tx.translation.x = pose->Pos[0];
    tx.translation.y = pose->Pos[1];
    tx.translation.z = pose->Pos[2];
    tx.rotation.w = pose->Rot[0];
    tx.rotation.x = pose->Rot[1];
    tx.rotation.y = pose->Rot[2];
    tx.rotation.z = pose->Rot[3];
    return tx;
}

bool publish_pose(tf::TransformBroadcaster &broadcaster, uint32_t seq, const SurviveSimpleObject *it, FLT time = -1) {
	SurvivePose pose = {};
	geometry_msgs::TransformStamped pose_msg = {};
	auto timecode = survive_simple_object_get_latest_pose(it, &pose);
	if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
		timecode = survive_simple_run_time(actx);
	}
	if (timecode > 0) {
	    time = time == -1 ? timecode : time;
        std::string name = survive_simple_serial_number(it);

		pose_msg.header.seq = seq++;
		pose_msg.header.stamp = rostime_from_survivetime(time);
		pose_msg.header.frame_id = "libsurvive_world";
		pose_msg.child_frame_id = name;
		pose_msg.transform = ros_from_pose(&pose);

		broadcaster.sendTransform(pose_msg);

		SurvivePose imu2head;
        survive_simple_object_get_transform_to_imu(it, &imu2head);
        imu2head = InvertPoseRtn(&imu2head);

        pose_msg.header.seq = seq++;
        pose_msg.header.stamp = rostime_from_survivetime(time);
        pose_msg.header.frame_id = name;
        pose_msg.child_frame_id = name + "_imu";
        pose_msg.transform = ros_from_pose(&imu2head);

        broadcaster.sendTransform(pose_msg);

        return true;
	}
	return false;
}

static void imu_func(SurviveObject *so, int mask, const FLT *accelgyromag, uint32_t timecode, int id) {
    survive_default_imu_process(so, mask, accelgyromag, timecode, id);
    if(auto pub = getPublishers(so)) {
        pub->publish(mask, accelgyromag, timecode, id);
    }
}

SurviveSimpleContext * actx_from_ros(ros::NodeHandle& n) {
    std::vector<std::string> arg_names, arg_values;
    n.getParamNames(arg_names);
    std::vector<const char *> params;
    auto node_name = ros::this_node::getName();
    params.push_back(node_name.c_str());
    for (auto &k : arg_names) {
        if(k.find(node_name + "/") == 0) {
            std::string local_name = k.c_str() + node_name.size() + 1;
            arg_values.emplace_back("--" + local_name);

            double vf;
            std::string v;
            if(n.getParam(k, v)) {
                arg_values.emplace_back(v);
            } else if(n.getParam(k, vf)) {
                if(vf == std::round(vf)) {
                    arg_values.emplace_back(std::to_string((int)vf));
                } else {
                    arg_values.emplace_back(std::to_string(vf));
                }
            }
        }
    }
    for (auto &v : arg_values) {
        params.push_back(v.c_str());
    }
    return survive_simple_init(params.size(), (char **) &params[0]);
}

int main(int argc, char **argv) {

	std::map<std::string, ros::Publisher> publishers;

	std::string node_name = "libsurvive";
	ros::init(argc, argv, node_name);
    n = std::make_unique<ros::NodeHandle>("libsurvive");
    actx = actx_from_ros(*n);

	if (actx == nullptr) // implies -help or similiar
		return 0;

    auto ctx = survive_simple_get_ctx(actx);
    survive_install_imu_fn(ctx, imu_func);

    auto now = survive_simple_run_time(actx);
	auto ros_now = ros::Time::now().toSec();
	ros_offset = ros_now - now;

	survive_simple_start_thread(actx);

	tf::TransformBroadcaster broadcaster;
    uint32_t seq = 1;

	auto last_chirp = now;

    struct SurviveSimpleEvent event = { };
    while (survive_simple_wait_for_event(actx, &event) != SurviveSimpleEventType_Shutdown && ros::ok()) {
        switch (event.event_type) {
            case SurviveSimpleEventType_PoseUpdateEvent: {
                const struct SurviveSimplePoseUpdatedEvent *pose_event = survive_simple_get_pose_updated_event(&event);
                if(survive_simple_object_get_type(pose_event->object) != SurviveSimpleObject_LIGHTHOUSE) {
                    publish_pose(broadcaster, seq++, pose_event->object);
                }
                break;
            }
            case SurviveSimpleEventType_ButtonEvent: {
                const struct SurviveSimpleButtonEvent *button_event = survive_simple_get_button_event(&event);
                if(auto pub = getPublishers(button_event->object)) {
                    pub->publish(button_event);
                }
                break;
            }
            case SurviveSimpleEventType_ConfigEvent: {
                const struct SurviveSimpleConfigEvent *configEvent = survive_simple_get_config_event(&event);
                if(auto pub = getPublishers(configEvent->object)) {
                    pub->publish(configEvent);
                }
            }
            case SurviveSimpleEventType_None:
                break;
        }

		ros_now = ros::Time::now().toSec();
		if (ros_now > last_chirp + .25) {
			last_chirp = now;
			for (const SurviveSimpleObject *it = survive_simple_get_first_object(actx); it != 0;
				 it = survive_simple_get_next_object(actx, it)) {
				if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
					publish_pose(broadcaster, seq++, it, ros_now);
				}
			}
		}
	}

	survive_simple_close(actx);
	return 0;
}
