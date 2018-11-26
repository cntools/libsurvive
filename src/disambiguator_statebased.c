//
#include "survive_internal.h"
#include <assert.h>
#include <math.h> /* for sqrt */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

//#define DEBUG_TB(...) SV_INFO(__VA_ARGS__)
#define DEBUG_TB(...)
/**
 * The lighthouses go in the following order:
 *
 *     Ticks  State
 *         0  ACode 0b1x0 (4) <--- B
 *    20 000  ACode 0b0x0 (0) <--- A/c
 *            LH A X Sweep
 *   400 000  ACode 0b1x1 (5) <--- B
 *   420 000  ACode 0b0x1 (1) <--- A/c
 *            LH A Y SWEEP
 *   800 000  ACode 0b0x0 (0) <--- B
 *   820 000  ACode 0b1x0 (4) <--- A/c
 *            LH B X Sweep
 * 1 200 000  ACode 0b0x1 (1) <--- B
 * 1 220 000  ACode 0b1x1 (5) <--- A/c
 *            LH B Y SWEEP
 * 1 600 000  < REPEAT >
 *
 * NOTE: Obviously you cut the data bit out for this
 *
 * This disambiguator works by finding where in that order it is, and tracking along with it.
 * It is able to maintain this tracking for extended periods of time without further data
 * by knowing the modulo of the start of the cycle and calculating appropriatly although this
 * will run into issues when the timestamp rolls over or we simply drift off in accuracy.
 *
 * Neither case is terminal though; it will just have to find the modulo again which only takes
 * a handful of pulses.
 *
 * The main advantage to this scheme is that its reasonably fast and is able to deal with being
 * close enough to the lighthouse that the lengths are in a valid sync pulse range.
 */

// Every pulse_window seems roughly 20k ticks long. That leaves ~360 to the capture window
#define PULSE_WINDOW 20000
#define CAPTURE_WINDOW 360000

enum LighthouseState {
	LS_UNKNOWN = 0,

	LS_WaitLHA_ACode4 = 1,
	LS_WaitLHA_ACode0,
	LS_SweepAX,
	LS_WaitLHA_ACode5,
	LS_WaitLHA_ACode1,
	LS_SweepAY,
	LS_WaitLHB_ACode0,
	LS_WaitLHB_ACode4,
	LS_SweepBX,
	LS_WaitLHB_ACode1,
	LS_WaitLHB_ACode5,
	LS_SweepBY,

	LS_END
};

typedef struct {
	int acode, lh, axis, window, offset;
	bool is_sweep;
} LighthouseStateParameters;

// clang-format off
const LighthouseStateParameters LS_Params[LS_END + 1] = {
	{.acode = -1, .lh = -1, .axis = -1, .window = -1},

    {.acode = 0, .lh = 0, .axis = 0, .window = PULSE_WINDOW,   .offset = 0 * PULSE_WINDOW + 0 * CAPTURE_WINDOW},                // 0
    {.acode = 4, .lh = 1, .axis = 0, .window = PULSE_WINDOW,   .offset = 1 * PULSE_WINDOW + 0 * CAPTURE_WINDOW},                // 20000
    {.acode = 0, .lh = 0, .axis = 0, .window = CAPTURE_WINDOW, .offset = 2 * PULSE_WINDOW + 0 * CAPTURE_WINDOW, .is_sweep = 1}, // 40000

    {.acode = 1, .lh = 0, .axis = 1, .window = PULSE_WINDOW,   .offset = 2 * PULSE_WINDOW + 1 * CAPTURE_WINDOW},                // 400000
    {.acode = 5, .lh = 1, .axis = 1, .window = PULSE_WINDOW,   .offset = 3 * PULSE_WINDOW + 1 * CAPTURE_WINDOW},                // 420000
    {.acode = 1, .lh = 0, .axis = 1, .window = CAPTURE_WINDOW, .offset = 4 * PULSE_WINDOW + 1 * CAPTURE_WINDOW, .is_sweep = 1}, // 440000

    {.acode = 4, .lh = 0, .axis = 0, .window = PULSE_WINDOW,   .offset = 4 * PULSE_WINDOW + 2 * CAPTURE_WINDOW},                // 800000
	{.acode = 0, .lh = 1, .axis = 0, .window = PULSE_WINDOW,   .offset = 5 * PULSE_WINDOW + 2 * CAPTURE_WINDOW},                // 820000
	{.acode = 4, .lh = 1, .axis = 0, .window = CAPTURE_WINDOW, .offset = 6 * PULSE_WINDOW + 2 * CAPTURE_WINDOW, .is_sweep = 1}, // 840000

	{.acode = 5, .lh = 0, .axis = 1, .window = PULSE_WINDOW,   .offset = 6 * PULSE_WINDOW + 3 * CAPTURE_WINDOW},                // 1200000
	{.acode = 1, .lh = 1, .axis = 1, .window = PULSE_WINDOW,   .offset = 7 * PULSE_WINDOW + 3 * CAPTURE_WINDOW},                // 1220000
	{.acode = 5, .lh = 1, .axis = 1, .window = CAPTURE_WINDOW, .offset = 8 * PULSE_WINDOW + 3 * CAPTURE_WINDOW, .is_sweep = 1}, // 1240000

	{.acode = -1, .lh = -1, .axis = -1, .window = -1, .offset = 8 * PULSE_WINDOW + 4 * CAPTURE_WINDOW}                          // 1600000
};
// clang-format on

enum LighthouseState LighthouseState_findByOffset(int offset) {
	for (int i = 2; i < LS_END + 1; i++) {
		if (LS_Params[i].offset > offset)
			return i - 1;
	}
	assert(false);
	return -1;
}

typedef struct {
	SurviveObject *so;
	/* We keep the last sync time per LH because lightproc expects numbers relative to it */
	uint32_t time_of_last_sync[NUM_LIGHTHOUSES];

	/* Keep running average of sync signals as they come in */
	uint64_t last_sync_timestamp;
	uint64_t last_sync_length;
	int last_sync_count;

	uint32_t first_sync_timestamp;
	uint32_t longest_sync_length;

	/**  This part of the structure is general use when we know our state */
	enum LighthouseState state;
	uint32_t mod_offset;
	int confidence;

	/** This rest of the structure is dedicated to finding a state when we are unknown */
	int encoded_acodes;

	int stabalize;
	bool lastWasSync;
	int single_60hz_confidence;
	bool single_60hz_mode;

	LightcapElement sweep_data[];
} Disambiguator_data_t;

static uint32_t timestamp_diff(uint32_t recent, uint32_t prior) {
	if (recent > prior)
		return recent - prior;
	return (0xFFFFFFFF - prior) + recent;
}

static int find_acode(uint32_t pulseLen) {
	const static int offset = 50;
	if (pulseLen < 2500 + offset)
		return -1;

	if (pulseLen < 3000 + offset)
		return 0;
	if (pulseLen < 3500 + offset)
		return 1;
	if (pulseLen < 4000 + offset)
		return 2;
	if (pulseLen < 4500 + offset)
		return 3;
	if (pulseLen < 5000 + offset)
		return 4;
	if (pulseLen < 5500 + offset)
		return 5;
	if (pulseLen < 6000 + offset)
		return 6;
	if (pulseLen < 6500 + offset)
		return 7;

	return -1;
}

static bool overlaps(const LightcapElement *a, const LightcapElement *b) {
	int overlap = 0;
	if (a->timestamp < b->timestamp && a->length + a->timestamp > b->timestamp)
		overlap = a->length + a->timestamp - b->timestamp;
	else if (b->timestamp < a->timestamp && b->length + b->timestamp > a->timestamp)
		overlap = b->length + b->timestamp - a->timestamp;

	return overlap > a->length / 2;
}

const int SKIP_BIT = 4;
const int DATA_BIT = 2;
const int AXIS_BIT = 1;

#define LOWER_SYNC_TIME 2250
#define UPPER_SYNC_TIME 6750

#define DIV_ROUND_CLOSEST(n, d) ((((n) < 0) ^ ((d) < 0)) ? (((n) - (d) / 2) / (d)) : (((n) + (d) / 2) / (d)))

LightcapElement get_last_sync(Disambiguator_data_t *d) {
	if (d->last_sync_count == 0) {
		return (LightcapElement){0};
	}
	LightcapElement lastSync = {
		.timestamp = d->first_sync_timestamp, .length = d->longest_sync_length, .sensor_id = -d->last_sync_count};
	return lastSync;
}

enum LightcapClassification { LCC_SWEEP, LCC_SYNC };
static enum LightcapClassification naive_classify(Disambiguator_data_t *d, const LightcapElement *le) {
	bool clearlyNotSync = le->length < LOWER_SYNC_TIME || le->length > UPPER_SYNC_TIME;

	if (clearlyNotSync) {
		return LCC_SWEEP;
	} else {
		return LCC_SYNC;
	}
}

#define ACODE_TIMING(acode)                                                                                            \
	((3000 + ((acode)&1) * 500 + (((acode) >> 1) & 1) * 1000 + (((acode) >> 2) & 1) * 2000) - 250)
#define ACODE(s, d, a) ((s << 2) | (d << 1) | a)
#define SWEEP 0xFF

static uint32_t SolveForMod_Offset(Disambiguator_data_t *d, enum LighthouseState state, const LightcapElement *le) {
	assert(LS_Params[state].is_sweep == 0); // Doesn't work for sweep data
	SurviveContext *ctx = d->so->ctx;
	DEBUG_TB("Solve for mod %d (%u - %u) = %u", state, le->timestamp, LS_Params[state].offset,
			 (le->timestamp - LS_Params[state].offset));

	return (le->timestamp - (le->length / 2) - LS_Params[state].offset);
}

static enum LighthouseState SetState(Disambiguator_data_t *d, const LightcapElement *le,
									 enum LighthouseState new_state);
static enum LighthouseState CheckEncodedAcode(Disambiguator_data_t *d, uint8_t newByte) {

	// We chain together acodes / sweep indicators to form an int we can just switch on.
	SurviveContext *ctx = d->so->ctx;
	d->encoded_acodes &= 0xFF;
	d->encoded_acodes = (d->encoded_acodes << 8) | newByte;

	LightcapElement lastSync = get_last_sync(d);

	// These combinations are checked for specificaly to allow for the case one lighthouse is either
	//  missing or completely occluded.
	switch (d->encoded_acodes) {
	case (ACODE(0, 1, 0) << 8) | SWEEP:
		d->mod_offset = SolveForMod_Offset(d, LS_SweepAX - 1, &lastSync);

		return (LS_SweepAX + 1);
	case (ACODE(0, 1, 1) << 8) | SWEEP:
		d->mod_offset = SolveForMod_Offset(d, LS_SweepAY - 1, &lastSync);

		return (LS_SweepAY + 1);
	case (SWEEP << 8) | (ACODE(0, 1, 1)):
		d->mod_offset = SolveForMod_Offset(d, LS_WaitLHB_ACode1, &lastSync);

		return (LS_WaitLHB_ACode1 + 1);
	case (SWEEP << 8) | (ACODE(1, 1, 0)):
		d->mod_offset = SolveForMod_Offset(d, LS_WaitLHA_ACode4, &lastSync);

		return (LS_WaitLHA_ACode4 + 1);
	}

	return LS_UNKNOWN;
}
static enum LighthouseState EndSweep(Disambiguator_data_t *d, const LightcapElement *le) {
	return CheckEncodedAcode(d, SWEEP);
}
static enum LighthouseState EndSync(Disambiguator_data_t *d, const LightcapElement *le) {
	LightcapElement lastSync = get_last_sync(d);
	int acode = find_acode(lastSync.length) > 0;
	if (acode > 0) {
		return CheckEncodedAcode(d, (acode | DATA_BIT));
	} else {
		// If we can't resolve an acode, just reset
		d->encoded_acodes = 0;
	}
	return LS_UNKNOWN;
}

static void RegisterSync(Disambiguator_data_t *d, const LightcapElement *le) {
	if (le->timestamp < d->first_sync_timestamp || d->longest_sync_length == 0)
		d->first_sync_timestamp = le->timestamp;

	if (le->length > d->longest_sync_length) {
		d->longest_sync_length = le->length;
	}

	d->last_sync_timestamp += le->timestamp;
	d->last_sync_length += le->length;
	d->last_sync_count++;
}

static void ResetSync(Disambiguator_data_t *d) {
	d->first_sync_timestamp = d->longest_sync_length = 0;
	d->last_sync_timestamp = d->last_sync_length = d->last_sync_count = 0;
}

static enum LighthouseState AttemptFindState(Disambiguator_data_t *d, const LightcapElement *le) {
	enum LightcapClassification classification = naive_classify(d, le);

	if (classification == LCC_SYNC) {
		LightcapElement lastSync = get_last_sync(d);

		// Handle the case that this is a new SYNC coming in
		if (d->lastWasSync == false || overlaps(&lastSync, le) == false) {

			if (d->lastWasSync && timestamp_diff(lastSync.timestamp, le->timestamp) > 30000) {
				// Missed a sweep window; clear encoded values.
				d->encoded_acodes = 0;
			}

			// Now that the previous two states are in, check to see if they tell us where we are
			enum LighthouseState new_state = d->lastWasSync ? EndSync(d, le) : EndSweep(d, le);
			if (new_state != LS_UNKNOWN)
				return new_state;

			// Otherwise, just reset the sync registers and do another

			ResetSync(d);
		}

		RegisterSync(d, le);
		d->lastWasSync = true;
	} else {
		// If this is the start of a new sweep, check to see if the end of the sync solves
		// the state
		if (d->lastWasSync) {
			enum LighthouseState new_state = EndSync(d, le);
			if (new_state != LS_UNKNOWN)
				return new_state;
		}
		d->lastWasSync = false;
	}

	return LS_UNKNOWN;
}

static enum LighthouseState SetState(Disambiguator_data_t *d, const LightcapElement *le,
									 enum LighthouseState new_state) {

	SurviveContext *ctx = d->so->ctx;
	if (new_state >= LS_END)
		new_state = 1;

	d->encoded_acodes = 0;
	d->state = new_state;

	if (new_state == LS_UNKNOWN) {
		d->single_60hz_mode = false;
		d->single_60hz_confidence = 0;
	}

	ResetSync(d);

	memset(d->sweep_data, 0, sizeof(LightcapElement) * d->so->sensor_ct);

	return new_state;
}

static void PropagateState(Disambiguator_data_t *d, const LightcapElement *le);
static void RunACodeCapture(int target_acode, Disambiguator_data_t *d, const LightcapElement *le) {
	// Just ignore small signals; this has a measurable impact on signal quality
	if (le->length < 100)
		return;

	// We know what state we are in, so we verify that state as opposed to
	// trying to suss out the acode.

	// Calculate what it would be with and without data
	uint32_t time_error_d0 = abs(ACODE_TIMING(target_acode) - le->length);
	uint32_t time_error_d1 = abs(ACODE_TIMING(target_acode | DATA_BIT) - le->length);

	// Take the least of the two erors
	uint32_t error = time_error_d0 > time_error_d1 ? time_error_d1 : time_error_d0;

	// Errors do happen; either reflections or some other noise. Our scheme here is to
	// keep a tally of hits and misses, and if we ever go into the negatives reset
	// the state machine to find the state again.
	if (error > 1250) {
		uint32_t time_error_ad0 = abs(ACODE_TIMING(0) - le->length);
		uint32_t time_error_ad1 = abs(ACODE_TIMING(0 | DATA_BIT) - le->length);

		uint32_t a_error = time_error_ad0 > time_error_ad1 ? time_error_ad1 : time_error_ad0;
		if (target_acode == 4 && a_error < 1250) {
			if (d->single_60hz_confidence++ > 3 && d->single_60hz_mode == false && d->confidence < 50) {
				d->single_60hz_mode = true;
				SurviveContext *ctx = d->so->ctx;
				SV_WARN("Disambiguator detected single mode A LH (60hz mode)");
			}
		} else {
			// Penalize semi-harshly -- if it's ever off track it will take this many syncs
			// to reset
			const int penalty = 3;
			if (d->confidence < penalty) {
				SurviveContext *ctx = d->so->ctx;
				SetState(d, le, LS_UNKNOWN);
				SV_INFO("WARNING: Disambiguator got lost; refinding state for %s", d->so->codename);
			}
			d->confidence -= penalty;
		}

		return;
	}

	if (d->confidence < 100)
		d->confidence++;

	// If its a real timestep, integrate it here and we can take the average later

	RegisterSync(d, le);
}

static void ProcessStateChange(Disambiguator_data_t *d, const LightcapElement *le, enum LighthouseState new_state) {
	SurviveContext *ctx = d->so->ctx;

	// Leaving a sync ...
	if (LS_Params[d->state].is_sweep == 0) {
		if (d->last_sync_count > 0) {
			LightcapElement lastSync = {.timestamp = d->first_sync_timestamp,
										.length = d->longest_sync_length,
										.sensor_id = -d->last_sync_count};
			// Use the average of the captured pulse to adjust where we are modulo against.
			// This lets us handle drift in any of the timing chararacteristics
			d->mod_offset = SolveForMod_Offset(d, d->state, &lastSync);

			// Figure out if it looks more like it has data or doesn't. We need this for OOX
			int lengthData = ACODE_TIMING(LS_Params[d->state].acode | DATA_BIT);
			int lengthNoData = ACODE_TIMING(LS_Params[d->state].acode);
			bool hasData = abs(lengthData - lastSync.length) < abs(lengthNoData - lastSync.length);
			int acode = LS_Params[d->state].acode;
			if (hasData) {
				acode |= DATA_BIT;
			}

			int next_state = d->state + 1;

			if (next_state == LS_END || (d->single_60hz_mode && next_state == LS_WaitLHB_ACode0))
				next_state = 0;

			int index_code = LS_Params[next_state].is_sweep ? -1 : -2;
			ctx->lightproc(d->so, index_code, acode, 0, lastSync.timestamp, lastSync.length, LS_Params[d->state].lh);

			// Store last sync time for sweep calculations
			d->time_of_last_sync[LS_Params[d->state].lh] = lastSync.timestamp;
		}
	} else {
		// Leaving a sweep ...
		size_t avg_length = 0;
		size_t cnt = 0;

		for (int i = 0; i < d->so->sensor_ct; i++) {
			LightcapElement le = d->sweep_data[i];
			// Only care if we actually have data AND we have a time of last sync. We won't have the latter
			// if we synced with the LH at cetain times.
			if (le.length > 0 && d->time_of_last_sync[LS_Params[d->state].lh] > 0) {
				avg_length += le.length;
				cnt++;
			}
		}
		if (cnt > 0) {
			double var = 1.5;
			size_t minl = (1 / var) * DIV_ROUND_CLOSEST(avg_length, cnt);
			size_t maxl = var * DIV_ROUND_CLOSEST(avg_length, cnt);

			for (int i = 0; i < d->so->sensor_ct; i++) {
				LightcapElement le = d->sweep_data[i];
				// Only care if we actually have data AND we have a time of last sync. We won't have the latter
				// if we synced with the LH at certain times.
				if (le.length > 0 && d->time_of_last_sync[LS_Params[d->state].lh] > 0 && le.length >= minl &&
					le.length <= maxl) {
					int32_t offset_from =
						timestamp_diff(le.timestamp + le.length / 2, d->time_of_last_sync[LS_Params[d->state].lh]);

					// Send the lightburst out.
					if (offset_from > 0)
						d->so->ctx->lightproc(d->so, i, LS_Params[d->state].acode, offset_from, le.timestamp, le.length,
											  LS_Params[d->state].lh);
				}
			}
		}
	}
	SetState(d, le, new_state);
}

static void PropagateState(Disambiguator_data_t *d, const LightcapElement *le) {
	int end_of_mod = d->single_60hz_mode ? LS_WaitLHB_ACode0 : LS_END;

	int le_offset = le->timestamp > d->mod_offset
						? (le->timestamp - d->mod_offset + 10000) % LS_Params[end_of_mod].offset
						: (0xFFFFFFFF - d->mod_offset + le->timestamp + 10000) % LS_Params[end_of_mod].offset;

	/** Find where this new element fits into our state machine. This can skip states if its been a while since
	 * its been able to process, or if a LH is missing. */
	enum LighthouseState new_state = LighthouseState_findByOffset(le_offset);

	if (d->state != new_state) {
		// This processes the change -- think setting buffers, and sending OOTX / lightproc calls
		ProcessStateChange(d, le, new_state);
	}

	const LighthouseStateParameters *param = &LS_Params[d->state];
	if (param->is_sweep == 0) {
		RunACodeCapture(param->acode, d, le);
	} else if (le->length > d->sweep_data[le->sensor_id].length &&
			   le->length < 7000 /*anything above 10k seems to be bullshit?*/) {
		// Note we only select the highest length one per sweep. Also, we bundle everything up and send it later all at
		// once.
		// so that we can do this filtering. Might not be necessary?
		d->sweep_data[le->sensor_id] = *le;
	}
}

void DisambiguatorStateBased(SurviveObject *so, const LightcapElement *le) {
	SurviveContext *ctx = so->ctx;

	// Note, this happens if we don't have config yet -- just bail
	if (so->sensor_ct == 0) {
		return;
	}

	if (so->disambiguator_data == NULL) {
		DEBUG_TB("Initializing Disambiguator Data for TB %d", so->sensor_ct);
		Disambiguator_data_t *d = calloc(1, sizeof(Disambiguator_data_t) + sizeof(LightcapElement) * so->sensor_ct);
		d->so = so;
		so->disambiguator_data = d;
	}

	Disambiguator_data_t *d = so->disambiguator_data;
	// It seems like the first few hundred lightcapelements are missing a ton of data; let it stabilize.
	if (d->stabalize < 200) {
		d->stabalize++;
		return;
	}

	if (d->state == LS_UNKNOWN) {
		enum LighthouseState new_state = AttemptFindState(d, le);
		if (new_state != LS_UNKNOWN) {
			d->confidence = 0;

			int le_offset = (le->timestamp - d->mod_offset) % LS_Params[LS_END].offset;
			enum LighthouseState new_state1 = LighthouseState_findByOffset(le_offset);
			SetState(d, le, new_state1);
			DEBUG_TB("Locked onto state %d(%d, %d) at %u", new_state, new_state1, le_offset, d->mod_offset);
		}
	} else {
		PropagateState(d, le);
	}
}

REGISTER_LINKTIME(DisambiguatorStateBased);
