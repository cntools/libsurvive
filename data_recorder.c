// Data recorder mod with GUI showing light positions.

#ifdef __linux__
#include <unistd.h>
#endif

#include <os_generic.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <survive.h>

#include <time.h>

#ifndef _MSC_VER
#include <sys/time.h>
#endif

#include "redist/os_generic.h"

struct SurviveContext *ctx;

int main(int argc, char **argv) {
	ctx = survive_init(argc, argv);
	if (ctx == 0) // implies -help or similiar
		return 0;

	const char *outfile = survive_configs(ctx, "dataoutfile", SC_GET, "");

	if (strlen(outfile) == 0) {
		survive_configi(ctx, "datastdout", SC_SETCONFIG | SC_OVERRIDE, 1);
	}

	survive_startup(ctx);
	if (survive_configi(ctx, "calibrate", SC_GET, 1)) {
		fprintf(stderr, "Installing calibration\n");
		survive_cal_install(ctx);
	}

	if (!ctx) {
		fprintf(stderr, "Fatal. Could not start\n");
		exit(1);
	}

	while (survive_poll(ctx) == 0) {
	}

	return 0;
}
