// (C) 2016 Julian Picht, MIT/x11 License.
//
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.
#ifndef DISAMBIGUATOR_H
#define DISAMBIGUATOR_H

#define DIS_NUM_VALUES 8

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

struct disambiguator {
	long times[DIS_NUM_VALUES];
	int scores[DIS_NUM_VALUES];
	dis_state state;
	long last;
	int max_confidence;
	char code;
};

struct classified_pulse_ {
	pulse_type t;
	int length;
};

void disambiguator_init( struct disambiguator * d);
pulse_type disambiguator_step( struct disambiguator * d, long time, int length);

#endif /* DISAMBIGUATOR_H */
