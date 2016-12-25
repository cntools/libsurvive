#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

typedef uint8_t pulse_data;

#if 0
struct sync_pulse_lengths {
	uint32_t expected_offset;
	uint16_t min;
	uint16_t max;
	pulse_data simple_data;
	pulse_data data;
};

const struct sync_pulse_lengths sync_pulses[] = {
	{400000, 2880, 3024, 0x0, 0x0}, // SKIP 0 DATA 0 AXIS 0
	{400000, 3380, 3524, 0x1, 0x1}, // SKIP 0 DATA 0 AXIS 1
	{400000, 3880, 4024, 0x0, 0x2}, // SKIP 0 DATA 1 AXIS 0
	{400000, 4380, 4524, 0x1, 0x3}, // SKIP 0 DATA 1 AXIS 1
	{ 20000, 4880, 5024, 0x2, 0x4}, // SKIP 1 DATA 0 AXIS 0
	{ 20000, 5380, 5524, 0x3, 0x5}, // SKIP 1 DATA 0 AXIS 1
	{ 20000, 5880, 6024, 0x2, 0x6}, // SKIP 1 DATA 1 AXIS 0
	{ 20000, 6380, 6524, 0x3, 0x7}, // SKIP 1 DATA 1 AXIS 1
};
#endif

const uint32_t pulse_types[] = {
	0, 1, 0, 1,
	2, 3, 2, 3,
};

typedef enum {
	PT_UNKNOWN = 0,
	PT_SWEEP = 1,
	PT_SYNC = 2
} pulse_type;

typedef enum {
	AX_J = 0,
	AX_K = 1
} scan_axis;
typedef enum {
	SK_ON = 0x4,
	SK_OFF = 0
} scan_skip;


#define DEBUG(...) printf(__VA_ARGS__)

#define PULSE_BIT_AXIS 0x1
#define PULSE_BIT_DATA 0x2
#define PULSE_BIT_SKIP 0x4

#define PULSE_DATA(D) ((D >> 1)&0x1)
#define PULSE_AXIS(D) ((scan_axis)(D&0x01))
#define PULSE_SKIP(D) ((D >> 2)&0x1)

pulse_data get_pulse_data(uint32_t length) {
	uint16_t temp = length - 2880;

#if BETTER_SAFE_THAN_FAST
	if (temp < 0 || length > 6525) {
		return -1;
	}
#endif

	if ((temp % 500) < 150) {
		return temp / 500;
	}

	return -1;
}

struct disambiguator;
typedef pulse_type (*pulse_fn)(struct disambiguator * d, uint32_t timestamp, uint32_t length);
typedef void (*data_fn)(struct disambiguator * d, uint8_t bit);

void null_data_fn(struct disambiguator * d __attribute__((unused)), uint8_t bit __attribute__((unused))) {
	return;
}

struct disambiguator {
	pulse_fn pulse_fn;
	data_fn data_fn;
	uint32_t last_master_sync;
	bool locked;
};

void disambiguator_init(struct disambiguator * d);
pulse_type disambiguator_sync_start(struct disambiguator * d, uint32_t timestamp, uint32_t length);
pulse_type disambiguator_sync_A1(struct disambiguator * d, uint32_t timestamp, uint32_t length);
pulse_type disambiguator_sync_B0(struct disambiguator * d, uint32_t timestamp, uint32_t length);
pulse_type disambiguator_sync_B1(struct disambiguator * d, uint32_t timestamp, uint32_t length);

pulse_type disambiguator_sync_DONE(struct disambiguator * d, uint32_t timestamp, uint32_t length);

void disambiguator_init(struct disambiguator * d) {
	d->data_fn = &null_data_fn;
	d->pulse_fn = &disambiguator_sync_start;
	d->locked = false;
}

pulse_type disambiguator_sync_start(struct disambiguator * d, uint32_t timestamp, uint32_t length) {
	DEBUG("START %10d %6d %6d\n", timestamp, 0, length);
	pulse_data dd = get_pulse_data(length);
	if ((dd & (PULSE_BIT_AXIS | PULSE_BIT_SKIP)) == 0) {
		d->pulse_fn = &disambiguator_sync_A1;
		d->last_master_sync = timestamp;
	}
	return PT_UNKNOWN;
}

pulse_type disambiguator_sync_A1(struct disambiguator * d, uint32_t timestamp, uint32_t length) {
	pulse_data dd = get_pulse_data(length);
	uint32_t diff = timestamp - d->last_master_sync;
	DEBUG("A1    %10d %6d %6d\n", timestamp, diff, length);
	if (18720 > diff) {
		return PT_UNKNOWN;
	}
	if (diff > 21600) {
		d->pulse_fn = &disambiguator_sync_start;
		return PT_UNKNOWN;
	}

	if ((dd & (PULSE_BIT_AXIS | PULSE_BIT_SKIP)) == PULSE_BIT_SKIP) {
		d->pulse_fn = &disambiguator_sync_B0;
	}
	return PT_UNKNOWN;
}

pulse_type disambiguator_sync_B0(struct disambiguator * d, uint32_t timestamp, uint32_t length) {
	pulse_data dd = get_pulse_data(length);
	uint32_t diff = timestamp - d->last_master_sync;
	DEBUG("B0    %10d %6d %6d\n", timestamp, diff, length);
	if (398832 > diff) {
		return PT_UNKNOWN;
	}
	if (diff > 401136) {
		d->pulse_fn = &disambiguator_sync_start;
		return PT_UNKNOWN;
	}

	if ((dd & (PULSE_BIT_AXIS | PULSE_BIT_SKIP)) == PULSE_BIT_AXIS) {
		d->pulse_fn = &disambiguator_sync_B1;
		d->last_master_sync = timestamp;
	}
	return PT_UNKNOWN;
}

pulse_type disambiguator_sync_B1(struct disambiguator * d, uint32_t timestamp, uint32_t length) {
	pulse_data dd = get_pulse_data(length);
	uint32_t diff = timestamp - d->last_master_sync;
	DEBUG("B1    %10d %6d %6d\n", timestamp, diff, length);
	if (18720 > diff) {
		return PT_UNKNOWN;
	}
	if (diff > 21600) {
		d->pulse_fn = &disambiguator_sync_start;
		return PT_UNKNOWN;
	}

	if ((dd & (PULSE_BIT_AXIS | PULSE_BIT_SKIP)) == (PULSE_BIT_AXIS|PULSE_BIT_SKIP)) {
		d->pulse_fn = &disambiguator_sync_DONE;
		d->locked = true;
	}
	return PT_UNKNOWN;
}

pulse_type disambiguator_sync_DONE(struct disambiguator * d __attribute__((unused)), uint32_t timestamp, uint32_t length) {
	DEBUG("DONE  %10d %6d %6d\n", timestamp, 0, length);
	return PT_UNKNOWN;
}

uint32_t fake_pulse_length(pulse_data d) {
	return 3000 + (d*500);
}


int main() {
#ifdef TEST
	struct disambiguator d;
	d.data_fn = &null_data_fn;
	d.pulse_fn = &disambiguator_sync_start;
	d.pulse_fn(&d, 0, 3000);
	d.pulse_fn(&d, 20000, fake_pulse_length(PULSE_BIT_SKIP));
	d.pulse_fn(&d, 20000, 3500);
#else
	FILE * f = fopen( "raw_light_data_from_watchman.sorted.csv", "r" );
	if (f == NULL) {
		fprintf(stderr, "ERROR OPENING INPUT FILE\n");
		return -1;
	}

	//long last = 0, lastl = 0;
	long lastl = 0;

	struct disambiguator d;
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
		printf("%s: ", controller);
		if (lastl > time) {
			printf("BACKWARDS: %li %li\n", lastl, time);
		}
		lastl = time;

		d.pulse_fn(&d, time, length);
/*		switch (d.step_fn(&d, time, length)) {
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
		}*/
	}
	fclose(f);
#endif
}