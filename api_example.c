#include <stdio.h>
#include <string.h>
#include <survive_api.h>
#include <os_generic.h>

static volatile int keepRunning = 1;

#ifdef __linux__

#include <assert.h>
#include <signal.h>
#include <stdlib.h>

void intHandler(int dummy) {
	if (keepRunning == 0)
		exit(-1);
	keepRunning = 0;
}

#endif

static void log_fn(SurviveSimpleContext *actx, SurviveLogLevel logLevel, const char *msg) {
	fprintf(stderr, "SimpleApi: %s\n", msg);
}

int main(int argc, char **argv) {
#ifdef __linux__
	signal(SIGINT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGKILL, intHandler);
#endif

	SurviveSimpleContext *actx = survive_simple_init_with_logger(argc, argv, log_fn);
	if (actx == 0) // implies -help or similiar
		return 0;

	survive_simple_start_thread(actx);

	for (const SurviveSimpleObject *it = survive_simple_get_first_object(actx); it != 0;
		 it = survive_simple_get_next_object(actx, it)) {
		printf("Found '%s'\n", survive_simple_object_name(it));
	}

	while (survive_simple_wait_for_update(actx) && keepRunning) {
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
				SurviveObjectSubtype subtype = survive_simple_object_get_subtype(button_event->object);
				printf("%s input %s (%d) ", survive_simple_object_name(button_event->object),
					   SurviveInputEventStr(button_event->event_type), button_event->event_type);

				FLT v1 = survive_simple_object_get_input_axis(button_event->object, SURVIVE_AXIS_TRACKPAD_X) / 2. + .5;

				if (button_event->button_id != 255) {
					printf(" button %16s (%2d) ", SurviveButtonsStr(subtype, button_event->button_id),
						   button_event->button_id);

					if (button_event->button_id == SURVIVE_BUTTON_SYSTEM) {
						FLT v = 1 - survive_simple_object_get_input_axis(button_event->object, SURVIVE_AXIS_TRIGGER);
						survive_simple_object_haptic(button_event->object, 30, v, .5);
					}
				}
				for (int i = 0; i < button_event->axis_count; i++) {
					printf(" %20s (%2d) %+5.4f   ", SurviveAxisStr(subtype, button_event->axis_ids[i]),
						   button_event->axis_ids[i], button_event->axis_val[i]);
				}
				printf("\n");
			}
			case SurviveSimpleEventType_None:
				break;
			}
		}
	}
	printf("Cleaning up\n");
	survive_simple_close(actx);
	return 0;
}
