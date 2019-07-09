//<>< (C) 2016 C. N. Lohr, MOSTLY Under MIT/x11 License.
//
#include "survive_internal.h"
#include <assert.h>
#include <math.h> /* for sqrt */
#include <stdint.h>
#include <string.h>

#define PULSELENGTH_MIN_SYNC 2200
#define TIMECENTER_TICKS (48000000 / 240)
#define PULSEDIST_MAX_TICKS 500000
#define PULSE_IN_CLEAR_TIME 35000
#define PULSE_MAX_FOR_SWEEP 1800
#define PULSE_SYNCTIME_OFFSET 20000 // unused?
#define PULSE_SYNCTIME_SLACK 5000

static int32_t decode_acode(uint32_t length, int32_t main_divisor) {
	//+50 adds a small offset and seems to help always get it right.
	// Check the +50 in the future to see how well this works on a variety of hardware.
	if (!main_divisor)
		return -1;

	int32_t acode = (length + main_divisor + 50) / (main_divisor * 2);
	if (acode & 1)
		return -1;

	int32_t rtn = (acode >> 1) - 6;
	if (rtn > 7 || rtn < 0) {
		return -1;
	}
	return rtn;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////The charles disambiguator.  Don't use this, mostly here for
/// debugging.///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void HandleOOTX(SurviveContext *ctx, SurviveObject *so) {
	int32_t main_divisor = so->timebase_hz / 384000; // 125 @ 48 MHz.

	int32_t acode_array[2] = {decode_acode(so->last_sync_length[0], main_divisor),
							  decode_acode(so->last_sync_length[1], main_divisor)};

	int32_t delta1 = so->last_sync_time[0] - so->recent_sync_time;
	int32_t delta2 = so->last_sync_time[1] - so->last_sync_time[0];

	// printf( "%p %p %d %d %d  %p\n", ctx, so, so->last_sync_time[0], acode_array, so->last_sync_length[0],
	// ctx->lightproc );
	if (acode_array[0] >= 0)
		ctx->lightproc(so, -1, acode_array[0], delta1, so->last_sync_time[0], so->last_sync_length[0], 0);
	if (acode_array[1] >= 0)
		ctx->lightproc(so, -2, acode_array[1], delta2, so->last_sync_time[1], so->last_sync_length[1], 1);

	so->recent_sync_time = so->last_sync_time[1];

	so->did_handle_ootx = 1;
}

// This is the disambiguator function, for taking light timing and figuring out place-in-sweep for a given photodiode.
void DisambiguatorCharles(SurviveObject *so, LightcapElement *le) {
	SurviveContext *ctx = so->ctx;
	//	static int32_t last;
	//	printf( "%d %lu %d %d\n", le->timestamp-last, le->timestamp, le->length, le->sensor_id );
	//	last = le->timestamp;
	// printf( "LE%3d%6d%12d\n", le->sensor_id, le->length, le->timestamp );

	// int32_t deltat = (uint32_t)le->timestamp - (uint32_t)so->last_master_time;

	if (le->sensor_id > SENSORS_PER_OBJECT) {
		return;
	}

	so->tsl = le->timestamp;
	if (le->length < 20)
		return; /// Assuming 20 is an okay value for here.

	// The sync pulse finder is taking Charles's old disambiguator code and mixing it with a more linear
	// version of Julian Picht's disambiguator, available in 488c5e9.  Removed afterwards into this
	// unified driver.
	int ssn = so->sync_set_number; // lighthouse number
	if (ssn < 0)
		ssn = 0;
#ifdef DEBUG
	if (ssn >= NUM_GEN1_LIGHTHOUSES) {
		SV_INFO("ALGORITHMIC WARNING: ssn exceeds NUM_GEN1_LIGHTHOUSES");
	}
#endif
	int last_sync_time = so->last_sync_time[ssn];
	int last_sync_length = so->last_sync_length[ssn];
	int32_t delta = le->timestamp - last_sync_time; // Handle time wrapping (be sure to be int32)

	if (delta < -PULSEDIST_MAX_TICKS || delta > PULSEDIST_MAX_TICKS) {
		// Reset pulse, etc.
		so->sync_set_number = -1;
		delta = PULSEDIST_MAX_TICKS;
		//		return; //if we don't know what lighthouse this is we don't care to do much else
	}

	if (le->length > PULSELENGTH_MIN_SYNC) // Pulse longer indicates a sync pulse.
	{
		int is_new_pulse = delta > PULSELENGTH_MIN_SYNC /*1500*/ + last_sync_length;

		if (is_new_pulse) {
			int is_master_sync_pulse = delta > PULSE_IN_CLEAR_TIME /*40000*/;
			int is_pulse_from_same_lh_as_last_sweep;
			int tp = delta % (TIMECENTER_TICKS * 2);
			is_pulse_from_same_lh_as_last_sweep = tp < PULSE_SYNCTIME_SLACK && tp > -PULSE_SYNCTIME_SLACK;

			if (!so->did_handle_ootx) {
				HandleOOTX(ctx, so);
			}
			if (!is_master_sync_pulse) {
				so->did_handle_ootx = 0;
			}

			if (is_master_sync_pulse) // Could also be called by slave if no master was seen.
			{
				ssn = so->sync_set_number = is_pulse_from_same_lh_as_last_sweep
												? (so->sync_set_number)
												: 0; // If repeated lighthouse, just back off one.
				if (ssn < 0) {
					SV_INFO("SEVERE WARNING: Pulse codes for tracking not able to be backed out.\n");
					ssn = 0;
				}
				if (ssn != 0) {
					// If it's the slave that is repeated, be sure to zero out its sync info.
					so->last_sync_length[0] = 0;
				} else {
					so->last_sync_length[1] = 0;
				}
				so->last_sync_time[ssn] = le->timestamp;
				so->last_sync_length[ssn] = le->length;
			} else if (so->sync_set_number == -1) {
				// Do nothing.
			} else {
				ssn = ++so->sync_set_number;

				if (so->sync_set_number >= NUM_GEN1_LIGHTHOUSES) {
					SV_INFO("Warning.  Received an extra, unassociated sync pulse.");
					ssn = so->sync_set_number = -1;
				} else {
					so->last_sync_time[ssn] = le->timestamp;
					so->last_sync_length[ssn] = le->length;
				}
			}
		} else {
			// Find the longest pulse.
			if (le->length > last_sync_length) {
				if (so->last_sync_time[ssn] > le->timestamp) {
					so->last_sync_time[ssn] = le->timestamp;
					so->last_sync_length[ssn] = le->length;
				}
			}
		}

#if 0
		//Extra tidbit for storing length-of-sync-pulses, if you want to try to use this to determine AoI or distance to LH.
		//We don't actually use this anywhere, and I doubt we ever will?  Though, it could be useful at a later time to improve tracking.
		{
			int32_t main_divisor = so->timebase_hz / 384000; //125 @ 48 MHz.
			int base_station = is_new_pulse;
			printf( "%s %d %d %d\n", so->codename, le->sensor_id, so->sync_set_number, le->length ); //XXX sync_set_number is wrong here.
			ctx->lightproc( so, le->sensor_id, -3 - so->sync_set_number, 0, le->timestamp, le->length, base_station); //XXX sync_set_number is wrong here.
		}
#endif
	}

	// Any else- statements below here are

	// See if this is a valid actual pulse.
	else if (le->length < PULSE_MAX_FOR_SWEEP && delta > PULSE_IN_CLEAR_TIME && ssn >= 0) {
		int32_t tpco = so->last_sync_length[0];

#if NUM_GEN1_LIGHTHOUSES != 2
#error You are going to have to fix the code around here to allow for something other than two base stations.
#endif

		// Adding length
		// Long pulse-code from IR flood.
		// Make sure it fits nicely into a divisible-by-500 time.

		int32_t main_divisor = so->timebase_hz / 384000; // 125 @ 48 MHz.
		int acode = decode_acode(so->last_sync_length[0], main_divisor);

		// If acode isn't right; don't even think of emitting an event
		if (acode >= 0) {
			int whichlh = (acode >> 2);
			assert(whichlh <= 1);
			int32_t dl = so->last_sync_time[whichlh];

			if (!so->did_handle_ootx)
				HandleOOTX(ctx, so);

			int32_t offset_from = le->timestamp - dl + le->length / 2;

			// Make sure pulse is in valid window
			if (offset_from < TIMECENTER_TICKS * 2 - PULSE_IN_CLEAR_TIME && offset_from > PULSE_IN_CLEAR_TIME) {
				ctx->lightproc(so, le->sensor_id, acode, offset_from, le->timestamp, le->length, whichlh);
			}
		}
	} else {
		// printf( "FAIL %d   %d - %d = %d\n", le->length, so->last_photo_time, le->timestamp, so->last_photo_time -
		// le->timestamp );
		// Runt pulse, or no sync pulses available.
	}
}

REGISTER_LINKTIME(DisambiguatorCharles);
