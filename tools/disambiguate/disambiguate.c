#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

long times[64];
int scores[64];

void discard(long age) {
	for (int i = 0; i < sizeof(times)/sizeof(times[0]); ++i) {
		if (times[i] != 0 && times[i] < age) {
			times[i] = 0;
			scores[i] = 0;
		}
	}
}

int findNearest(long time) {
	int diff = 1000; // max allowed diff for a match
	int idx = -1;
	for (int i = 0; i < sizeof(times)/sizeof(times[0]); ++i) {
		if (times[i] == 0) continue;

		int a = abs(times[i] - time);
		if (a < diff) {
			idx = i;
			diff = a;
		}

	}

	//printf("DIFF: %d %d\n", diff, idx);

	return idx;
}

int main() {

	FILE * f = fopen( "raw_light_data_from_watchman.csv", "r" );
	if (f == NULL) {
		fprintf(stderr, "ERROR OPENING INPUT FILE\n");
		return -1;
	}
	memset(&times, 0, sizeof( times ));
	memset(&scores, 0, sizeof( scores ));

	long offset = 0, last = 0;

	for (;;) {
		char controller[10];
		int sensor;
		int unknown;
		int length;
		long time;
		if (fscanf(f, "%s %d %d %d %li", controller, &sensor, &unknown, &length, &time) != 5) {
			break;
		}
		/*if (time - last < 10000) {
			offset += 0x100000000;
		}*/
		time += offset;
		last = time;

		if (length < 2750) continue;

		discard(time - 10000000);
		int idx = findNearest(time - 800000);
		if (idx == -1) {
			for (int i = 0; i < sizeof(times)/sizeof(times[0]); ++i) {
				if (times[i] == 0) {
//					printf("ADD: %d %li\n", i, time);
					times[i] = time;
					break;
				}
			}
		} else {
			double l = length;
			char cc = round(l / 500) - 6;
			printf("MATCH: %li %d %d (0b%d%d%d)\n", time - times[idx], scores[idx], length, (cc >> 2) & 0x1, (cc >> 1) & 0x1, cc & 0x1);
			scores[idx]++;
			if (scores[idx] >= 30) {
				printf("MATCH: %li %d\n", time - times[idx], scores[idx]);
				return 0;
			}
			times[idx] = time;
		}
	}
	fclose(f);
}

