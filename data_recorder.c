/**
 * Most of the data recorder functionality lives in the library now; this just enables
 * stdout by default if no record file is specified.
 */
#include <stdio.h>
#include <string.h>
#include <survive.h>

int main(int argc, char **argv) {
	SurviveContext *ctx = survive_init(argc, argv);
	if (ctx == 0) // implies -help or similiar
		return 0;

	const char *dataout_file = survive_configs(ctx, "record", SC_SETCONFIG, "");
	if (strlen(dataout_file) == 0) {
		survive_configi(ctx, "record-stdout", SC_SET | SC_OVERRIDE, 1);
	}

	survive_startup(ctx);

	while (survive_poll(ctx) == 0) {
	}

	survive_close(ctx);
	return 0;
}
