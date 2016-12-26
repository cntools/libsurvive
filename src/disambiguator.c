// (C) 2016 Julian Picht, MIT/x11 License.
//
// All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

//
// The theory behind this disambiguator is, that if we just track all pulses and if one could be a sync pulse, we look back in time,
// if we saw as sync pulse X samples ago than it is probably a sync pulse.
//
// If the skip flag is set, X is 20000 else X is 400000
//

#include "disambiguator.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

typedef uint8_t pulse_data;

/**
 * Translate pulse length to pulse SKIP, DATA, AXIS
 * @param length Length of the pulse in (1/48000000) seconds
 * @return pulse data
 */
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

const uint32_t pulse_types[] = {
	0, 1, 0, 1,
	2, 3, 2, 3,
};

#define PULSE_BIT_AXIS 0x1
#define PULSE_BIT_DATA 0x2
#define PULSE_BIT_SKIP 0x4

#define PULSE_DATA(D) ((D >> 1)&0x1)
#define PULSE_AXIS(D) (D&0x01)
#define PULSE_SKIP(D) ((D >> 2)&0x1)

void disambiguator_init( struct disambiguator * d ) {
	memset(&(d->times), 0x0, sizeof(d->times));
	memset(&(d->scores), 0x0, sizeof(d->scores));
	d->state = D_STATE_UNLOCKED;
	d->last = 0;
	d->max_confidence = 0;
}

inline void disambiguator_discard( struct disambiguator * d );

/**
 * Drop all data that is outdated
 * @param d
 * @param age Maximum age of data we care to keep
 */
void disambiguator_discard( struct disambiguator * d )
{
	long age;
	if (d->state == D_STATE_LOCKED) {
		age = d->last - 400000;
	} else {
		age = 1000000;
	}
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

/**
 * Find the index that has the best likelyhood too match up with the timestamp given
 * @param t1 Rising edge time, where we expect to find the last sync pulse, if this is a master pulse
 * @param t2 Rising edge time, where we expect to find the last sync pulse, if this is a slave pulse
 * @param max_diff Maximum difference we are prepared to accept
 * @return index inside d->times, if we found something, -1 otherwise
 */
inline int disambiguator_find_nearest( struct disambiguator * d, uint32_t t1, uint32_t t2, int max_diff );

int disambiguator_find_nearest( struct disambiguator * d, uint32_t t1, uint32_t t2, int max_diff )
{
	int diff = max_diff; // max allowed diff for a match
	int idx = -1;
	for (unsigned int i = 0; i < DIS_NUM_VALUES; ++i) {
		if (d->times[i] == 0) continue;

		int a_1 = abs(d->times[i] - t1);
		int a_2 = abs(d->times[i] - t2);

//		printf("T            %d %d %d\n", time, i, a);
		if (a_1 < diff) {
			idx = i;
			diff = a_1;
		} else if (a_2 < diff) {
			idx = i;
			diff = a_2;
		}
	}

//	if (idx != -1) {
//		printf("R            %d %d %d\n", idx, d->scores[idx], diff);
//	}

	return idx;
}

pulse_type disambiguator_step( struct disambiguator * d, uint32_t time, int length)
{
	uint32_t diff = time - d->last;

	// all smaller pulses are most probably sweeps
	// TODO: check we are inside the time window of actual sweeps
	if (length < 2750) {
		return d->state == D_STATE_LOCKED ? P_SWEEP : P_UNKNOWN;
	}

	// we expected to see a sync pulse earlier ...
	if (time - d->last > 401000) {
		d->state = D_STATE_UNLOCKED;
	}

	// discard all data, that is so old, we don't care about it anymore
	disambiguator_discard(d);

	// find the best match for our timestamp and presumed offset
	int idx = disambiguator_find_nearest(d, time - 400000, time - 20000, 100);

	// We did not find a matching pulse, so try find a place to record the current
	// one's time of arrival.
	if (idx == -1) {
		for (int i = 0; i < DIS_NUM_VALUES; ++i) {
			if (d->times[i] == 0) {
				d->times[i] = time;
				break;
			}
		}

		return d->state == D_STATE_LOCKED ? P_SWEEP : P_UNKNOWN;
	} else {
		d->scores[idx]++;
		if (d->scores[idx] >= DIS_NUM_PULSES_BEFORE_LOCK) {
			d->state = D_STATE_LOCKED;
		}

		if (diff < 21000) {
			return d->state == D_STATE_LOCKED ? P_SLAVE : P_UNKNOWN;
		}

		d->times[idx] = time;
		d->last = time;

		return d->state == D_STATE_LOCKED ? (
			d->scores[idx] >= d->max_confidence ? P_MASTER : P_SWEEP
		) : P_UNKNOWN;
	}

	return d->state == D_STATE_LOCKED ? P_SWEEP : P_UNKNOWN;
}
