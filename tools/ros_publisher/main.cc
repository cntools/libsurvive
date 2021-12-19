#define SURVIVE_ENABLE_FULL_API

#include <libsurvive/survive_api.h>
#include <libsurvive/survive.h>
#include <os_generic.h>

#include "ros/ros.h"
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

double ros_offset = 0;
SurviveSimpleContext *actx = 0;

bool publish_pose(tf::TransformBroadcaster &broadcaster, uint32_t seq, const SurviveSimpleObject *it) {
	SurvivePose pose = {};
	geometry_msgs::TransformStamped pose_msg = {};
	auto timecode = survive_simple_object_get_latest_pose(it, &pose);
	if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
		timecode = survive_simple_run_time(actx);
	}
	if (timecode > 0) {
        std::string name = survive_simple_object_name(it);

		pose_msg.header.seq = seq++;
		pose_msg.header.stamp = ros::Time::now(); // ros::Time().fromSec(timecode + ros_offset);
		pose_msg.header.frame_id = "libsurvive_world";
		pose_msg.child_frame_id = name;
		pose_msg.transform.translation.x = pose.Pos[0];
		pose_msg.transform.translation.y = pose.Pos[1];
		pose_msg.transform.translation.z = pose.Pos[2];
		pose_msg.transform.rotation.w = pose.Rot[0];
		pose_msg.transform.rotation.x = pose.Rot[1];
		pose_msg.transform.rotation.y = pose.Rot[2];
		pose_msg.transform.rotation.z = pose.Rot[3];

		broadcaster.sendTransform(pose_msg);

		SurvivePose imu2head;
        survive_simple_object_get_transform_to_imu(it, &imu2head);
        imu2head = InvertPoseRtn(&imu2head);

        pose_msg.header.seq = seq++;
        pose_msg.header.stamp = ros::Time::now(); // ros::Time().fromSec(timecode + ros_offset);
        pose_msg.header.frame_id = name;
        pose_msg.child_frame_id = name + "_imu";
        pose_msg.transform.translation.x = imu2head.Pos[0];
        pose_msg.transform.translation.y = imu2head.Pos[1];
        pose_msg.transform.translation.z = imu2head.Pos[2];
        pose_msg.transform.rotation.w = imu2head.Rot[0];
        pose_msg.transform.rotation.x = imu2head.Rot[1];
        pose_msg.transform.rotation.y = imu2head.Rot[2];
        pose_msg.transform.rotation.z = imu2head.Rot[3];

        broadcaster.sendTransform(pose_msg);

        return true;
	}
	return false;
}


void publish_control_state(ros::Publisher &publisher, uint32_t seq, const SurviveSimpleObject *it) {
    sensor_msgs::Joy joyMsg;
    joyMsg.header.frame_id = survive_simple_object_name(it);
    joyMsg.header.seq = seq;
    joyMsg.header.stamp = ros::Time::now();

    joyMsg.axes.resize(SURVIVE_MAX_AXIS_COUNT);
    joyMsg.buttons.resize(SURVIVE_BUTTON_MAX * 2);

    int64_t mask = survive_simple_object_get_button_mask(it) | (survive_simple_object_get_touch_mask(it) << SURVIVE_BUTTON_MAX);
    for(int i = 0;i < SURVIVE_MAX_AXIS_COUNT;i++) {
        joyMsg.axes[i] = (float)survive_simple_object_get_input_axis(it, (enum SurviveAxis)i);
    }
    for(int i = 0;i < mask;i++) {
        joyMsg.buttons[i] = (mask >> i) & 1;
    }
}

ros::Publisher imuPublisher;

static void imu_func(SurviveObject *so, int mask, const FLT *accelgyromag, uint32_t timecode, int id) {
    survive_default_imu_process(so, mask, accelgyromag, timecode, id);
    if(imuPublisher) {
        static int imuSeq = 0;
        sensor_msgs::Imu imu;
        imu.header.frame_id = std::string(so->codename) + "_imu";
        imu.header.seq = imuSeq++;
        imu.header.stamp = ros::Time::now();

        imu.angular_velocity.x = accelgyromag[3];
        imu.angular_velocity.y = accelgyromag[4];
        imu.angular_velocity.z = accelgyromag[5];

        imu.linear_acceleration.x = accelgyromag[0] * 9.80665;
        imu.linear_acceleration.y = accelgyromag[1] * 9.80665;
        imu.linear_acceleration.z = accelgyromag[2] * 9.80665;

        imuPublisher.publish(imu);
    }
}

int main(int argc, char **argv) {

	std::map<std::string, ros::Publisher> publishers;

	ros::init(argc, argv, "libsurvive");
	ros::NodeHandle n;

	actx = survive_simple_init(argc, argv);

	if (actx == nullptr) // implies -help or similiar
		return 0;

    auto ctx = survive_simple_get_ctx(actx);
    survive_install_imu_fn(ctx, imu_func);

    auto now = survive_simple_run_time(actx);
	auto ros_now = ros::Time::now();
	ros_offset = ros_now.toSec() - now;

	survive_simple_start_thread(actx);

	tf::TransformBroadcaster broadcaster;
    auto joyPublisher = n.advertise<sensor_msgs::Joy>("joy", 1);
    imuPublisher = n.advertise<sensor_msgs::Imu>("imu", 1);
	uint32_t seq = 1;

	auto last_chirp = now;

    struct SurviveSimpleEvent event = { };
    while (survive_simple_wait_for_event(actx, &event) != SurviveSimpleEventType_Shutdown && ros::ok()) {
        switch (event.event_type) {
            case SurviveSimpleEventType_PoseUpdateEvent: {
                const struct SurviveSimplePoseUpdatedEvent *pose_event = survive_simple_get_pose_updated_event(&event);
                publish_pose(broadcaster, seq++, pose_event->object);
                break;
            }
            case SurviveSimpleEventType_ButtonEvent: {
                const struct SurviveSimpleButtonEvent *button_event = survive_simple_get_button_event(&event);
                publish_control_state(joyPublisher, seq++, button_event->object);
                break;
            }
            case SurviveSimpleEventType_None:
                break;
        }

		now = survive_simple_run_time(actx);
		if (now > last_chirp + 3) {
			last_chirp = now;
			for (const SurviveSimpleObject *it = survive_simple_get_first_object(actx); it != 0;
				 it = survive_simple_get_next_object(actx, it)) {
				if (survive_simple_object_get_type(it) == SurviveSimpleObject_LIGHTHOUSE) {
					publish_pose(broadcaster, seq++, it);
				}
			}
		}
	}

	survive_simple_close(actx);
	return 0;
}
