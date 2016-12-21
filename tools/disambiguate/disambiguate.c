// (C) 2016 Julian Picht, MIT/x11 License.
//
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../../include/disambiguator.h"

int main() {

	FILE * f = fopen( "raw_light_data_from_watchman.sorted.csv", "r" );
	if (f == NULL) {
		fprintf(stderr, "ERROR OPENING INPUT FILE\n");
		return -1;
	}

	long last = 0, lastl = 0;

	disambiguator d;
	disambiguator_init(&d);
	for (;;) {
		char controller[10];
		int sensor;
		int unknown;
		int length;
		long time;

		if (fscanf(f, "%s %d %d %d %li", controller, &sensor, &unknown, &length, &time) != 5) {
			break;
		}
		if (lastl > time) {
			printf("BACKWARDS: %li %li\n", lastl, time);
		}
		lastl = time;

		switch (disambiguator_step(&d, time, length)) {
			default:
			case P_UNKNOWN:
				//printf("UNKN  %s %2d %li %d\n", controller, sensor, time - last, length);
				continue;
			case P_SYNC:
				{
					double l = length;
					char cc = round(l / 500) - 6;
					int ll = (length+125)/250;
					printf("SYNC  %s %2d %10li %5d %c%d %10li %d %d\n", controller, sensor, time, length, (cc & 0x1) ? 'k' : 'j', (cc >> 1) & 0x3, time-last, ll & 1, (ll >> 1) - 6);
					last = time;
				}
				continue;
			case P_SWEEP:
				printf("SWEEP %s %2d %10li %5d\n", controller, sensor, time - last, length);
				continue;
		}
	}
	fclose(f);
}

