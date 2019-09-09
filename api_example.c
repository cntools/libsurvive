#include <stdio.h>
#include <string.h>
#include <survive_api.h>
#include <os_generic.h>

static void log_fn(SurviveSimpleContext *actx, SurviveLogLevel logLevel, const char *msg) {
	fprintf(stderr, "SimpleApi: %s\n", msg);
}

int main(int argc, char **argv) {
	SurviveSimpleContext *actx = survive_simple_init_with_logger(argc, argv, log_fn);
	if (actx == 0) // implies -help or similiar
		return 0;

	survive_simple_start_thread(actx);

	for (const SurviveSimpleObject *it = survive_simple_get_first_object(actx); it != 0;
		 it = survive_simple_get_next_object(actx, it)) {
		printf("Found '%s'\n", survive_simple_object_name(it));
	}

	while (survive_simple_is_running(actx)) {
		OGUSleep(10000);
		for (const SurviveSimpleObject *it = survive_simple_get_next_updated(actx); it != 0;
			 it = survive_simple_get_next_updated(actx)) {
			SurvivePose pose;
			uint32_t timecode = survive_simple_object_get_latest_pose(it, &pose);
			printf("%s %s (%u): %f %f %f %f %f %f %f\n", survive_simple_object_name(it),
				   survive_simple_serial_number(it), timecode, pose.Pos[0], pose.Pos[1], pose.Pos[2], pose.Rot[0],
				   pose.Rot[1], pose.Rot[2], pose.Rot[3]);
		}

		struct SurviveSimpleEvent event = {0};

		while (survive_simple_next_event(actx, &event) != SurviveSimpleEventType_None) {
			switch (event.event_type) {
			case SurviveSimpleEventType_ButtonEvent: {
				const struct SurviveSimpleButtonEvent *button_event = survive_simple_get_button_event(&event);
				printf("%s button event %d %d\n", survive_simple_object_name(button_event->object),
					   (int)button_event->event_type, button_event->button_id);
			}
			case SurviveSimpleEventType_None:
				break;
			}
		}
	}

	survive_simple_close(actx);
	return 0;
}
