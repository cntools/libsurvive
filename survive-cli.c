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

SURVIVE_EXPORT void button_process(SurviveObject *so, enum SurviveInputEvent eventType, enum SurviveButton buttonId,
								   const enum SurviveAxis *axisIds, const SurviveAxisVal_t *axisVals) {
	if (buttonId == SURVIVE_BUTTON_MENU) {
		survive_reset_lighthouse_positions(so->ctx);
	}
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

  survive_install_button_fn(ctx, button_process);
  while (keepRunning && survive_poll(ctx) == 0) {
  }

	survive_close(ctx);
	return 0;
}
