#include <survive.h>

static volatile int keepRunning = 1;

#ifdef __linux__

#include <assert.h>
#include <signal.h>
#include <stdlib.h>

void intHandler(int dummy) {
	if(keepRunning == 0)
		exit(-1);
	keepRunning = 0;
}

#endif


struct button_state {
	bool changed;
	bool pressed;
	struct axis_state {
		bool changed;
		uint16_t v;
	} axes[8];
};

struct ObjectButtonInfo {
	SurviveObject * so;
	struct button_state buttons[32];
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


static void button_process(SurviveObject *so, uint8_t eventType, uint8_t buttonId, uint8_t axis1Id, uint16_t axis1Val,
						   uint8_t axis2Id, uint16_t axis2Val) {

	struct SurviveContext *ctx = so->ctx;
	survive_default_button_process(so, eventType, buttonId, axis1Id, axis1Val, axis2Id, axis2Val);
	struct ObjectButtonInfo* info = ButtonInfoBySO(so);

	info->buttons[buttonId].changed = true;
	if(eventType == BUTTON_EVENT_BUTTON_DOWN) {
		info->buttons[buttonId].pressed = true;
	} else if(eventType == BUTTON_EVENT_BUTTON_UP){
		info->buttons[buttonId].pressed = false;
	} else if(eventType == BUTTON_EVENT_AXIS_CHANGED){
		info->buttons[buttonId].axes[axis1Id].changed = true;
		info->buttons[buttonId].axes[axis1Id].v = axis1Val;
		info->buttons[buttonId].axes[axis2Id].changed = true;
		info->buttons[buttonId].axes[axis2Id].v = axis2Val;
	}

	//SV_INFO("%s button event type: %02d id: %02u axis1id: %02u axis1val %5u axis2id: %u02 axis2val %5u",
	//so->codename, eventType, buttonId, axis1Id, axis1Val, axis2Id, axis2Val);
}

FLT last_redraw = 0;
bool needsRedraw = true;


#define gotoxy(x, y) printf("\033[%d;%dH", (y), (x))
static void redraw(SurviveContext *ctx) {
	printf("\033[;H");
	for (int i = 0; i < 32; i++) {
		if(buttonInfos[i].so == 0)
			break;

		printf("Object %s\r\n", buttonInfos[i].so->codename);
		for(int j = 0;j < 32;j++) {
			if(buttonInfos[i].buttons[j].changed == false)
				continue;

			printf("Button %3d (%s) ", j, buttonInfos[i].buttons[j].pressed ? "DOWN" : "UP");
			for(int k = 0;k<8;k++) {
				if(!buttonInfos[i].buttons[j].axes[k].changed)
					continue;

				printf("Axis %2d %5u ", k, buttonInfos[i].buttons[j].axes[k].v);
			}
			printf("\r\n");
		}
	}

	needsRedraw = false;
}
int printf_fn(SurviveContext *ctx, const char *fault, ...) { return 0; }
void info_fn(SurviveContext *ctx, SurviveLogLevel logLevel, const char *fault) { }

int main(int argc, char **argv) {
#ifdef __linux__
	signal(SIGINT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGKILL, intHandler);
#endif
	system("clear");
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