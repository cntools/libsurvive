#include <stdio.h>
#include <string.h>
#include <survive_api.h>
#include <os_generic.h>

int main(int argc, char **argv) {
	SurviveSimpleContext *actx = survive_simple_init(argc, argv);
	if (actx == 0) // implies -help or similiar
		return 0;

	survive_simple_start_thread(actx);

	while (survive_simple_is_running(actx)) {
		OGUSleep(30000);

		SurvivePose pose;

		for (const SurviveSimpleObject *it = survive_simple_get_first_object(actx); it != 0;
			 it = survive_simple_get_next_object(actx, it)) {
			uint32_t timecode = survive_simple_object_get_latest_pose(it, &pose);
			printf("%s (%u): %f %f %f %f %f %f %f\n", survive_simple_object_name(it), timecode, pose.Pos[0],
				   pose.Pos[1], pose.Pos[2], pose.Rot[0], pose.Rot[1], pose.Rot[2], pose.Rot[3]);
		}

		OGUSleep(30000);
		for (const SurviveSimpleObject *it = survive_simple_get_next_updated(actx); it != 0;
			 it = survive_simple_get_next_updated(actx)) {
			printf("%s changed since last checked\n", survive_simple_object_name(it));
		}

	}

	survive_simple_close(actx);
	return 0;
}
