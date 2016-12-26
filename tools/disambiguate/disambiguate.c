// (C) 2016 Julian Picht, MIT/x11 License.
//
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "disambiguator.h"

int main() {

	FILE * f = fopen( "new_lightcap_data.csv", "r" );
	if (f == NULL) {
		fprintf(stderr, "ERROR OPENING INPUT FILE\n");
		return -1;
	}

	uint32_t last = 0, lastl = 0;

	struct disambiguator d;
	disambiguator_init(&d);
	for (;;) {
		char controller[10];
		int sensor;
		int unknown;
		uint32_t length;
		uint32_t time;

		if (fscanf(f, "%s %d %d %d %d", controller, &sensor, &unknown, &length, &time) != 5) {
			break;
		}
		if (lastl > time) {
			//printf("BACKWARDS: %li %li\n", lastl, time);
		}
		lastl = time;
		if (strcmp(controller, "HMD") != 0) continue;

		char cc = (length - 2750) / 500;
		switch (disambiguator_step(&d, time, length)) {
			default:
			case P_UNKNOWN:
				//printf("UNKN  %s %2d %d %d\n", controller, sensor, time - last, length);
				continue;
			case P_MASTER:
				printf("MASTR %s %2d %10d %5d %c%d %10d\n", controller, sensor, time, length, (cc & 0x1) ? 'k' : 'j', (cc >> 1) & 0x3, time-last);
				last = time;
				continue;
			case P_SLAVE:
				printf("SLAVE %s %2d %10d %5d %c%d %10d\n", controller, sensor, time, length, (cc & 0x1) ? 'k' : 'j', (cc >> 1) & 0x3, time-last);
				continue;
			case P_SWEEP:
				printf("SWEEP %s %2d %10d %5d\n", controller, sensor, time - last, length);
				continue;
		}
	}
	fclose(f);
}
