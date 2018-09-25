#include <survive.h>

int main(int argc, char **argv) {
	SurviveContext *ctx = survive_init(argc, argv);
	if (ctx == 0) // implies -help or similiar
		return 0;

	survive_startup(ctx);

	while (survive_poll(ctx) == 0) {
	}

	survive_close(ctx);
	return 0;
}
