#include <libsurvive/survive_api.h>
#include <os_generic.h>
#include <stdio.h>
#include <string.h>

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>

static double timestamp_in_us() {
	static double start_time_us = 0;
	if (start_time_us == 0.)
		start_time_us = OGGetAbsoluteTime();
	return OGGetAbsoluteTime() - start_time_us;
}

int main(int argc, char **argv) {

	std::map<std::string, ros::Publisher> publishers;

	ros::init(argc, argv, "libsurvive");
	ros::NodeHandle n;

	SurviveSimpleContext *actx = survive_simple_init(argc, argv);
	if (actx == 0) // implies -help or similiar
		return 0;

	survive_simple_start_thread(actx);

	uint32_t seq = 0;
	while (survive_simple_is_running(actx)) {
		SurvivePose pose;

		for (const SurviveSimpleObject *it = survive_simple_get_next_updated(actx); it != 0;
			 it = survive_simple_get_next_updated(actx)) {
			uint32_t timecode = survive_simple_object_get_latest_pose(it, &pose);
			const char *name = survive_simple_object_name(it);

			auto time_us = (int64_t)OGGetAbsoluteTime();
			geometry_msgs::PoseStamped pose_msg = {};
			pose_msg.header.seq = seq++;
			pose_msg.header.stamp = ros::Time::now();
			pose_msg.pose.position.x = pose.Pos[0];
			pose_msg.pose.position.y = pose.Pos[1];
			pose_msg.pose.position.z = pose.Pos[2];
			pose_msg.pose.orientation.w = pose.Rot[0];
			pose_msg.pose.orientation.x = pose.Rot[1];
			pose_msg.pose.orientation.y = pose.Rot[2];
			pose_msg.pose.orientation.z = pose.Rot[3];

			if (publishers.find(name) == publishers.end()) {
				publishers[name] = n.advertise<geometry_msgs::PoseStamped>(std::string(name) + "_pose", 1000);
			}
			publishers[name].publish(pose_msg);
		}
	}

	survive_simple_close(actx);
	return 0;
}
