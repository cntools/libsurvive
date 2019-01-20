#include <survive.h>

static volatile int keepRunning = 1;

#ifdef __linux__

#include <signal.h>
#include <stdlib.h>

void intHandler(int dummy) {
  if(keepRunning == 0)
    exit(-1);
  keepRunning = 0;
}

#endif

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

	while (keepRunning && survive_poll(ctx) == 0) {
	}

	survive_close(ctx);
	return 0;
}
