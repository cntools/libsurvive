#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define DIS_NUM_VALUES 64

typedef enum {
	D_SYNC_J0 = 0,
	D_SYNC_K0 = 1,
	D_SYNC_J1 = 2,
	D_SYNC_K1 = 3,
	D_SYNC_J2 = 4,
	D_SYNC_K2 = 5,
	D_SYNC_J3 = 6,
	D_SYNC_K3 = 7,
} sync_pulse;

typedef enum {
	D_STATE_INVALID = 0,
	D_STATE_LOCKED = 1,
	D_STATE_UNLOCKED = -1,
} dis_state;

typedef enum {
	P_UNKNOWN = 0,
	P_SYNC = 1,
	P_SWEEP = 2,
} pulse_type;

typedef struct disambiguator_ {
	long times[DIS_NUM_VALUES];
	int scores[DIS_NUM_VALUES];
	dis_state state;
	long last;
	int max_confidence;
} disambiguator;

typedef struct classified_pulse_ {
	pulse_type t;
	int length;
} classified_pulse;

void disambiguator_init(disambiguator * d) {
	memset(&(d->times), 0x0, sizeof(d->times));
	memset(&(d->scores), 0x0, sizeof(d->times));
	d->state = D_STATE_UNLOCKED;
	d->last = 0;
	d->max_confidence = 0;
}

void disambiguator_discard(disambiguator * d, long age) {
	int confidence = 0;
	for (unsigned int i = 0; i < DIS_NUM_VALUES; ++i) {
		if (d->times[i] != 0 && d->times[i] < age) {
			d->times[i] = 0;
			d->scores[i] = 0;
		} else {
			if (d->scores[i] > confidence) {
				confidence = d->scores[i];
			}
		}
	}
	d->max_confidence = confidence;
}

int disambiguator_find_nearest(disambiguator * d, long time, int max_diff) {
	int diff = max_diff; // max allowed diff for a match
	int idx = -1;
	for (unsigned int i = 0; i < DIS_NUM_VALUES; ++i) {
		if (d->times[i] == 0) continue;

		int a = abs(d->times[i] - time);
		if (a < diff) {
			idx = i;
			diff = a;
		}
	}

	return idx;
}

pulse_type disambiguator_step(disambiguator * d, long time, int length) {
	if (length < 2750) {
		return d->state == D_STATE_LOCKED ? P_SWEEP : P_UNKNOWN;
	}

	disambiguator_discard(d, time - 10000000);
	int idx = disambiguator_find_nearest(d, time - 400000, 100);

	if (time - d->last > 401000) {
		d->state = D_STATE_UNLOCKED;
	}

	if (idx == -1) {
		for (int i = 0; i < DIS_NUM_VALUES; ++i) {
			if (d->times[i] == 0) {
				d->times[i] = time;
				break;
			}
		}

		return d->state == D_STATE_LOCKED ? P_SWEEP : P_UNKNOWN;
	} else {
		// double l = length;
		// char cc = round(l / 500) - 6;
		// printf("MATCH: %li %d %d (0b%d%d%d)\n", time - d->times[idx], d->scores[idx], length, (cc >> 2) & 0x1, (cc >> 1) & 0x1, cc & 0x1);
		d->scores[idx]++;
		if (d->scores[idx] >= 30) {
			d->state = D_STATE_LOCKED;
			// printf("MATCH: %li %d\n", time - d->times[idx], scores[idx]);
		}
/*
		for (unsigned int i = 0; i < DIS_NUM_VALUES; ++i) {
			if (d->scores[i] > 0 && d->times[i] != 0)
				printf("TS %2d %li %d\n", i, d->times[i], d->scores[i]);
		}
*/
		d->times[idx] = time;
		d->last = time;
		return d->state == D_STATE_LOCKED ? (
			d->scores[idx] >= d->max_confidence ? P_SYNC : P_SWEEP
		) : P_UNKNOWN;
	}

	return d->state == D_STATE_LOCKED ? P_SWEEP : P_UNKNOWN;
}

sync_pulse length_to_pulse(int length) {
	double l = length - 3000;
	return round(l/500);
}

int main() {

	FILE * f = fopen( "raw_light_data_from_watchman.csv", "r" );
	if (f == NULL) {
		fprintf(stderr, "ERROR OPENING INPUT FILE\n");
		return -1;
	}

	long last = 0;

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

		switch (disambiguator_step(&d, time, length)) {
			default:
			case P_UNKNOWN:
				//printf("UNKN  %s %2d %li %d\n", controller, sensor, time - last, length);
				continue;
			case P_SYNC:
				{
					double l = length;
					char cc = round(l / 500) - 6;
					printf("SYNC  %s %2d %10li    %c%d %10li\n", controller, sensor, time, cc & 0x1 ? 'j' : 'k', (cc >> 1) & 0x3, time-last);
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

