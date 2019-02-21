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

static void button_process(SurviveObject *so, uint8_t eventType, uint8_t buttonId, uint8_t axis1Id, uint16_t axis1Val,
						   uint8_t axis2Id, uint16_t axis2Val) {
	struct SurviveContext *ctx = so->ctx;
	survive_default_button_process(so, eventType, buttonId, axis1Id, axis1Val, axis2Id, axis2Val);

	SV_INFO("%s button event type: %d id: %u axis1id: %u axis1val %u axis2id: %u axis2val %u", so->codename, eventType,
			buttonId, axis1Id, axis1Val, axis2Id, axis2Val);
}

int main(int argc, char **argv) {
#ifdef __linux__
	signal(SIGINT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGKILL, intHandler);
#endif

  SurviveContext *ctx = survive_init(argc, argv);
	if (ctx == 0) // implies -help or similiar
		return 0;

	survive_startup(ctx);
	survive_install_button_fn(ctx, &button_process);
	while (keepRunning && survive_poll(ctx) == 0) {
	}

	survive_close(ctx);
	return 0;
}
