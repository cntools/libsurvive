#include <libsurvive/survive_api.h>
#include <os_generic.h>
#include <stdio.h>
#include <string.h>

#include "ros/ros.h"
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
		const char *name = survive_simple_object_name(it);

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
		return true;
	}
	return false;
}
int main(int argc, char **argv) {

	std::map<std::string, ros::Publisher> publishers;

	ros::init(argc, argv, "libsurvive");
	ros::NodeHandle n;

	actx = survive_simple_init(argc, argv);
	if (actx == 0) // implies -help or similiar
		return 0;

	auto now = survive_simple_run_time(actx);
	auto ros_now = ros::Time::now();
	ros_offset = ros_now.toSec() - now;

	survive_simple_start_thread(actx);

	tf::TransformBroadcaster broadcaster;

	uint32_t seq = 1;

	auto last_chirp = now;

	while (survive_simple_wait_for_update(actx) && ros::ok()) {
		for (const SurviveSimpleObject *it = survive_simple_get_next_updated(actx); it != 0;
			 it = survive_simple_get_next_updated(actx)) {

			publish_pose(broadcaster, seq++, it);
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
