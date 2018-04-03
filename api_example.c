#include <stdio.h>
#include <string.h>
#include <survive_api.h>
#include <os_generic.h>

int main(int argc, char **argv) {
	SurviveAsyncContext *actx = survive_asyc_init(argc, argv);
	if (actx == 0) // implies -help or similiar
		return 0;

	survive_async_start_thread(actx);

	while (survive_async_is_running(actx)) {
		OGUSleep(30000);

		SurvivePose pose;

		for (const SurviveAsyncObject* it = survive_async_get_first_tracked(actx); it != 0; it = survive_async_get_next_tracked(actx, it)) {
			uint32_t timecode = survive_async_object_get_latest_pose(it, &pose);
			printf("%s (%u): %f %f %f %f %f %f %f\n", survive_async_object_name(it), timecode,
				pose.Pos[0], pose.Pos[1], pose.Pos[2], pose.Rot[0], pose.Rot[1], pose.Rot[2], pose.Rot[3]);
		}

		OGUSleep(30000);
		for (const SurviveAsyncObject* it = survive_async_get_next_updated(actx); it != 0; it = survive_async_get_next_updated(actx)) {
			printf("%s changed since last checked\n", survive_async_object_name(it));
		}

	}
	
	survive_async_close(actx);
	return 0;
}
