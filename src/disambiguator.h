// (C) 2016 Julian Picht, MIT/x11 License.
//
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.
#ifndef DISAMBIGUATOR_H
#define DISAMBIGUATOR_H

// Determines the number of samples stored in the disambiguator struct.
// Has to be higher than the maximum number of pulses expected between sync pulses.
#define DIS_NUM_VALUES 48
#define DIS_NUM_PULSES_BEFORE_LOCK 30
#include <stdint.h>

/**
 * internal disambiguator state
 */
typedef enum {
	D_STATE_INVALID = 0,
	D_STATE_LOCKED = 1,
	D_STATE_UNLOCKED = -1,
} dis_state;

/**
 * classification result
 */
typedef enum {
	P_UNKNOWN = 0,
	P_MASTER = 1,
	P_SWEEP = 2,
	P_SLAVE = 3,
} pulse_type;

/**
 * internal state of the disambiguator
 */
struct disambiguator {
	// the timestamps of the recorded pulses
	uint32_t times[DIS_NUM_VALUES];
	// countes how many sync pulses we have seen, that match the time offset at the same offset
	uint16_t scores[DIS_NUM_VALUES];
	// current state
	dis_state state;
	// last sync pulse time
	uint32_t last;
	// the absolute maximum counter value
	int max_confidence;
	// the last code type seen
	char code;
};


/**
 * Initialize a new disambiguator. calloc or memset with 0x00 will work just as well.
 *
 * @param d Pointer to the struct
 */
void disambiguator_init( struct disambiguator * d);

/**
 * Feed in one pulse to have if classified.
 *
 * @param d      Pointer to disambiguator state
 * @param time   Rising edge of the pulse
 * @param length Length of the pulse
 * @return       Classification result
 */
pulse_type disambiguator_step( struct disambiguator * d, uint32_t time, int length);

#endif /* DISAMBIGUATOR_H */