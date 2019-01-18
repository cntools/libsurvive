#include <survive.h>
#include <signal.h>

static volatile int keepRunning = 1;

void intHandler(int dummy) {
  if(keepRunning == 0)
    exit(-1);
  keepRunning = 0;
}

int main(int argc, char **argv) {
  signal(SIGINT, intHandler);
  signal(SIGTERM, intHandler);
  signal(SIGKILL, intHandler);  

  SurviveContext *ctx = survive_init(argc, argv);
	if (ctx == 0) // implies -help or similiar
		return 0;

	survive_startup(ctx);

	while (keepRunning && survive_poll(ctx) == 0) {
	}

	survive_close(ctx);
	return 0;
}
