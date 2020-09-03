#include <survive.h>

static volatile int keepRunning = 1;

#ifdef __linux__

#include <assert.h>
#include <signal.h>
#include <stdlib.h>
#include <sys/ioctl.h>

void intHandler(int dummy) {
	if(keepRunning == 0)
		exit(-1);
	keepRunning = 0;
}

#endif


struct button_state {
	bool changed;
	bool pressed;
	bool touch;
};

struct ObjectButtonInfo {
	SurviveObject * so;
	struct button_state buttons[32];
	struct axis_state {
		bool changed;
		SurviveAxisVal_t v;
	} axes[16];
};

struct ObjectButtonInfo buttonInfos[32];

struct ObjectButtonInfo* ButtonInfoBySO(SurviveObject* so) {
	for(int i = 0;i < 32;i++) {
		if(buttonInfos[i].so == 0) {
			buttonInfos[i].so = so;
		}

		if(buttonInfos[i].so == so)
			return &buttonInfos[i];
	}
	return 0;
}

FLT last_redraw = 0;
bool needsRedraw = true;

static void button_process(SurviveObject *so, enum SurviveInputEvent eventType, enum SurviveButton buttonId,
						   const enum SurviveAxis *axisIds, const SurviveAxisVal_t *axisVals) {

	struct SurviveContext *ctx = so->ctx;
	survive_default_button_process(so, eventType, buttonId, axisIds, axisVals);
	struct ObjectButtonInfo* info = ButtonInfoBySO(so);

	switch (eventType) {
	case SURVIVE_INPUT_EVENT_BUTTON_DOWN: {
		info->buttons[buttonId].changed = true;
		info->buttons[buttonId].pressed = true;
		break;
	}
	case SURVIVE_INPUT_EVENT_BUTTON_UP: {
		info->buttons[buttonId].changed = true;
		info->buttons[buttonId].pressed = false;
		break;
	}
	case SURVIVE_INPUT_EVENT_TOUCH_DOWN: {
		info->buttons[buttonId].changed = true;
		info->buttons[buttonId].touch = true;
		break;
	}
	case SURVIVE_INPUT_EVENT_TOUCH_UP: {
		info->buttons[buttonId].changed = true;
		info->buttons[buttonId].touch = false;
		break;
	}
	case SURVIVE_INPUT_EVENT_AXIS_CHANGED: {
		assert(buttonId == SURVIVE_BUTTON_UNKNOWN);
		for (int i = 0; i < 16 && axisIds[i] != 255; i++) {
			int axisId = axisIds[i];
			info->axes[axisId].changed = true;
			info->axes[axisId].v = axisVals[i];
		}
		break;
	}
	}

	SV_VERBOSE(50, "%s button event type: %02d id: (%12s) %03u axis1id: %2u axis1val %5.3f axis2id: %2u axis2val %5.3f",
			   so->codename, eventType, SurviveButtonsStr(so->object_subtype, buttonId), buttonId, axisIds[0],
			   axisVals[0], axisIds[1], axisVals[1]);

	needsRedraw = true;
}

char *lines[20] = {0};
size_t lines_idx = 0;

int window_rows = -1, window_cols = -1;
#define gotoxy(x, y) printf("\033[%d;%dH", (y), (x))
static void redraw(SurviveContext *ctx) {
	printf("\033[;H");
	for (int i = 0; i < 32; i++) {
		if(buttonInfos[i].so == 0)
			break;

		SurviveObject *so = buttonInfos[i].so;
		printf("Object %s (%s %s) (pressed: %08x, touched: %08x) \r\n", so->codename,
			   SurviveObjectTypeStr(so->object_type), SurviveObjectSubtypeStr(so->object_subtype), so->buttonmask,
			   so->touchmask);

		for (int k = 0; k < 16; k++) {
			if (!buttonInfos[i].axes[k].changed)
				continue;

			printf("\33[2KAxis   %24s (%2d) %+5.4f\r\n", SurviveAxisStr(so->object_subtype, k), k,
				   buttonInfos[i].axes[k].v);
		}

		for(int j = 0;j < 32;j++) {
			if(buttonInfos[i].buttons[j].changed == false)
				continue;

			printf("\33[2KButton %24s (%2d) (%s%s) \r\n", SurviveButtonsStr(so->object_subtype, j), j,
				   buttonInfos[i].buttons[j].pressed ? "DOWN" : "UP",
				   buttonInfos[i].buttons[j].touch ? ", CONTACT" : "");
		}
		printf("\r\n");
	}

	if (window_cols != -1) {
		gotoxy(0, window_rows - 20 - 1);
		printf("=== Log ===\n");
		for (int i = 0; i < 20; i++) {
			char *line = lines[(lines_idx + i) % 20];
			if (line != 0)
				printf("\33[2K\r %s\n", line);
		}
	}

	needsRedraw = false;
}
char *new_str(const char *s) {
	char *rtn = calloc(strlen(s) + 1, sizeof(char));
	strcpy(rtn, s);
	return rtn;
}

int printf_fn(SurviveContext *ctx, const char *fault, ...) { return 0; }
void info_fn(SurviveContext *ctx, SurviveLogLevel logLevel, const char *fault) {
	free(lines[lines_idx % 20]);
	lines[lines_idx % 20] = new_str(fault);
	lines_idx++;
	needsRedraw = true;
}

int main(int argc, char **argv) {
#ifdef __linux__
	signal(SIGINT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGKILL, intHandler);

	struct winsize w;
	ioctl(STDOUT_FILENO, TIOCGWINSZ, &w);
	window_cols = w.ws_col;
	window_rows = w.ws_row;
#endif
	int retcode = system("clear");
	(void)retcode;

	SurviveContext *ctx = survive_init(argc, argv);
	if (ctx == 0) // implies -help or similiar
		return 0;


	survive_install_button_fn(ctx, &button_process);
	survive_install_printf_fn(ctx, printf_fn);
	survive_install_log_fn(ctx, info_fn);
	survive_startup(ctx);
	while (keepRunning && survive_poll(ctx) == 0) {
		FLT this_time = OGGetAbsoluteTime();
		if (this_time > last_redraw + .03) {
			needsRedraw = true;
			last_redraw = this_time;
			redraw(ctx);
		}
	}

	survive_close(ctx);
	return 0;
}
