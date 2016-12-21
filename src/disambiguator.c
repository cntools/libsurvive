// (C) 2016 Julian Picht, MIT/x11 License.
//
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include "disambiguator.h"
#include <stdlib.h>
#include <string.h>

void disambiguator_init( struct disambiguator * d ) {
	memset(&(d->times), 0x0, sizeof(d->times));
	memset(&(d->scores), 0x0, sizeof(d->scores));
	d->state = D_STATE_UNLOCKED;
	d->last = 0;
	d->max_confidence = 0;
}

inline void disambiguator_discard( struct disambiguator * d, long age );

void disambiguator_discard( struct disambiguator * d, long age )
{
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

inline int disambiguator_find_nearest( struct disambiguator * d, long time, int max_diff );

int disambiguator_find_nearest( struct disambiguator * d, long time, int max_diff )
{
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

pulse_type disambiguator_step( struct disambiguator * d, long time, int length)
{
	if (length < 2750) {
		return d->state == D_STATE_LOCKED ? P_SWEEP : P_UNKNOWN;
	}
	//printf( "%d %d\n", time, length );
	//printf( "." );
	//time -= length/2;

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
		d->scores[idx]++;
		if (d->scores[idx] >= 30) {
			d->state = D_STATE_LOCKED;
		}

		d->times[idx] = time;
		d->last = time;
		return d->state == D_STATE_LOCKED ? (
			d->scores[idx] >= d->max_confidence ? P_SYNC : P_SWEEP
		) : P_UNKNOWN;
	}

	return d->state == D_STATE_LOCKED ? P_SWEEP : P_UNKNOWN;
}
