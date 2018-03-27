//
#include "survive_internal.h"
#include <assert.h>
#include <math.h> /* for sqrt */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define NUM_HISTORY 3

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

	{.acode = 4, .lh = 0, .axis = 0, .window = PULSE_WINDOW, .offset = 0 * PULSE_WINDOW + 0 * CAPTURE_WINDOW}, // 0
	{.acode = 0, .lh = 1, .axis = 0, .window = PULSE_WINDOW, .offset = 1 * PULSE_WINDOW + 0 * CAPTURE_WINDOW}, // 20000
	{.acode = 0, .lh = 1, .axis = 0, .window = CAPTURE_WINDOW, .offset = 2 * PULSE_WINDOW + 0 * CAPTURE_WINDOW,.is_sweep = 1}, // 40000

	{.acode = 5, .lh = 0, .axis = 1, .window = PULSE_WINDOW, .offset = 2 * PULSE_WINDOW + 1 * CAPTURE_WINDOW}, // 400000
	{.acode = 1, .lh = 1, .axis = 1, .window = PULSE_WINDOW, .offset = 3 * PULSE_WINDOW + 1 * CAPTURE_WINDOW}, // 420000
	{.acode = 1, .lh = 1, .axis = 1, .window = CAPTURE_WINDOW, .offset = 4 * PULSE_WINDOW + 1 * CAPTURE_WINDOW,.is_sweep = 1}, // 440000

	{.acode = 0, .lh = 0, .axis = 0, .window = PULSE_WINDOW, .offset = 4 * PULSE_WINDOW + 2 * CAPTURE_WINDOW}, // 800000
	{.acode = 4, .lh = 1, .axis = 0, .window = PULSE_WINDOW, .offset = 5 * PULSE_WINDOW + 2 * CAPTURE_WINDOW}, // 820000
	{.acode = 0, .lh = 0, .axis = 0, .window = CAPTURE_WINDOW, .offset = 6 * PULSE_WINDOW + 2 * CAPTURE_WINDOW,.is_sweep = 1}, // 840000

	{.acode = 1, .lh = 0, .axis = 1, .window = PULSE_WINDOW, .offset = 6 * PULSE_WINDOW + 3 * CAPTURE_WINDOW}, // 1200000
	{.acode = 5, .lh = 1, .axis = 1, .window = PULSE_WINDOW, .offset = 7 * PULSE_WINDOW + 3 * CAPTURE_WINDOW}, // 1220000
	{.acode = 1, .lh = 0, .axis = 1, .window = CAPTURE_WINDOW, .offset = 8 * PULSE_WINDOW + 3 * CAPTURE_WINDOW,.is_sweep = 1}, // 1240000

	{.acode = -1, .lh = -1, .axis = -1, .window = -1, .offset = 8 * PULSE_WINDOW + 4 * CAPTURE_WINDOW} // 1600000
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

enum LightcapClassification { LCC_UNKNOWN = 0, LCC_SYNC = 1, LCC_SWEEP = 2 };

typedef struct {
	LightcapElement history[NUM_HISTORY];
	enum LightcapClassification classifications[NUM_HISTORY];
	int idx;
} SensorHistory_t;

typedef struct {
	SurviveObject *so;

	/**  This part of the structure is general use when we know our state */
	uint32_t mod_offset;
	enum LighthouseState state;
	uint32_t last_state_transition_time;
	int confidence;
	uint32_t last_seen_time;
	LightcapElement sweep_data[SENSORS_PER_OBJECT];

	/** This rest of the structure is dedicated to finding a state when we are unknown */
	int encoded_acodes;
	/* Keep running average of sync signals as they come in */
	uint64_t last_sync_timestamp;
	uint64_t last_sync_length;
	int last_sync_count;
	int stabalize;
	bool lastWasSync;
	SensorHistory_t histories[];

} Disambiguator_data_t;

Disambiguator_data_t *Disambiguator_data_t_ctor(SurviveObject *so) {
	Disambiguator_data_t *rtn = calloc(1, sizeof(Disambiguator_data_t) + sizeof(SensorHistory_t) * so->sensor_ct);
	rtn->so = so;

	return rtn;
}

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

#define LOWER_SYNC_TIME 2250
#define UPPER_SYNC_TIME 6750

static int circle_buffer_get(int idx, int offset) { return ((idx + offset) + NUM_HISTORY) % NUM_HISTORY; }

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

LightcapElement get_last_sync(Disambiguator_data_t *d) {
	if (d->last_sync_count == 0) {
		return (LightcapElement){0};
	}

	return (LightcapElement){.timestamp = (d->last_sync_timestamp + d->last_sync_count / 2) / d->last_sync_count,
							 .length = (d->last_sync_length + d->last_sync_count / 2) / d->last_sync_count,
							 .sensor_id = -d->last_sync_count};
}

static uint32_t next_sync_expected(Disambiguator_data_t *d) {
	int acode = find_acode(get_last_sync(d).length);
	if (acode & SKIP_BIT)
		return get_last_sync(d).timestamp + 20000;
	return get_last_sync(d).timestamp + 399840;
}

static enum LightcapClassification classify(Disambiguator_data_t *d, SensorHistory_t *history,
											const LightcapElement *le) {
	bool clearlyNotSync = le->length < LOWER_SYNC_TIME || le->length > UPPER_SYNC_TIME;

	if (clearlyNotSync) {
		return LCC_SWEEP;
	}

	uint32_t time_diff_last_sync = timestamp_diff(le->timestamp, get_last_sync(d).timestamp);
	uint32_t split_time = 399840; // 8.33ms in 48mhz
	uint32_t jitter_allowance = 20000;

	// If we are ~8.33ms ahead of the last sync; we are a sync
	if (get_last_sync(d).length > 0 && abs(timestamp_diff(le->timestamp, next_sync_expected(d))) < jitter_allowance) {
		return LCC_SYNC;
	}

	LightcapElement last_sync = get_last_sync(d);
	if (get_last_sync(d).length > 0 && overlaps(&last_sync, le)) {
		return LCC_SYNC;
	}

	if (le->length > 2000)
		return LCC_SYNC;

	if (get_last_sync(d).length > 0 && time_diff_last_sync < (split_time - jitter_allowance)) {
		return LCC_SWEEP;
	}

	int prevIdx = circle_buffer_get(history->idx, -1);
	uint32_t time_diff = timestamp_diff(le->timestamp, history->history[prevIdx].timestamp);

	// We don't have recent data; unclear
	if (time_diff > split_time) {
		fprintf(stderr, "Time diff too high %d\n", time_diff);
		return LCC_UNKNOWN;
	}

	switch (history->classifications[prevIdx]) {
	case LCC_SWEEP:
		return LCC_SYNC;
	}
	fprintf(stderr, "last not sweep\n");
	return LCC_UNKNOWN;
}

static enum LightcapClassification update_histories(Disambiguator_data_t *d, const LightcapElement *le) {
	SensorHistory_t *history = &d->histories[le->sensor_id];

	enum LightcapClassification classification = classify(d, history, le);
	history->classifications[history->idx] = classification;
	history->history[history->idx] = *le;
	history->idx = (history->idx + 1) % NUM_HISTORY;
	return classification;
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

	return (le->timestamp - LS_Params[state].offset);
}

static enum LighthouseState SetState(Disambiguator_data_t *d, const LightcapElement *le,
									 enum LighthouseState new_state);
static enum LighthouseState CheckEncodedAcode(Disambiguator_data_t *d, uint8_t newByte) {
	SurviveContext *ctx = d->so->ctx;
	d->encoded_acodes &= 0xFF;
	d->encoded_acodes = (d->encoded_acodes << 8) | newByte; //(acode & (SKIP_BIT | AXIS_BIT));
	DEBUG_TB("0x%x", d->encoded_acodes);
	LightcapElement lastSync = get_last_sync(d);

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
	SurviveContext *ctx = d->so->ctx;
	LightcapElement lastSync = get_last_sync(d);
	int acode = find_acode(lastSync.length);
	DEBUG_TB("!!%.03f(%d)\tacode: %d 0x%x a:%d d:%d s:%d (%d)",
			 timestamp_diff(le->timestamp, lastSync.timestamp) / 48000.,
			 timestamp_diff(le->timestamp, lastSync.timestamp), lastSync.length, acode, acode & 1, (bool)(acode & 2),
			 (bool)(acode & 4), acode & (SKIP_BIT | AXIS_BIT));

	if (acode > 0) {
		return CheckEncodedAcode(d, (acode | DATA_BIT));
	} else {
		d->encoded_acodes = 0;
	}
	return LS_UNKNOWN;
}

static enum LighthouseState AttemptFindState(Disambiguator_data_t *d, const LightcapElement *le) {
	enum LightcapClassification classification = update_histories(d, le);

	static uint32_t start = 0;
	if (start == 0)
		start = le->timestamp;
	SurviveContext *ctx = d->so->ctx;
	DEBUG_TB("%d(%.03f) %d Incoming %u %u", (le->timestamp - start), (le->timestamp - start) / 48000., classification,
			 le->timestamp, le->length);

	if (classification == LCC_SYNC) {
		LightcapElement lastSync = get_last_sync(d);

		if (d->lastWasSync == false || overlaps(&lastSync, le) == false) {

			if (d->lastWasSync && timestamp_diff(lastSync.timestamp, le->timestamp) > 30000) {
				// Missed a sweep window; clear encoded values.
				SurviveContext *ctx = d->so->ctx;
				// DEBUG_TB("Missed sweep window.");
				d->encoded_acodes = 0;
			}

			enum LighthouseState new_state = d->lastWasSync ? EndSync(d, le) : EndSweep(d, le);

			if (new_state != LS_UNKNOWN)
				return new_state;

			d->last_sync_timestamp = le->timestamp;
			d->last_sync_length = le->length;
			d->last_sync_count = 1;
		} else {
			d->last_sync_timestamp += le->timestamp;
			d->last_sync_length += le->length;
			d->last_sync_count++;
		}

		d->lastWasSync = true;
	} else {
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
	DEBUG_TB("State transition %d -> %d at %u(%.03f)", d->state, new_state, le->timestamp,
			 timestamp_diff(d->last_state_transition_time, le->timestamp) / 480000.);

	d->state = new_state;
	d->last_state_transition_time = le->timestamp;

	d->last_sync_timestamp = d->last_sync_length = d->last_sync_count = 0;
	memset(d->sweep_data, 0, sizeof(LightcapElement) * SENSORS_PER_OBJECT);

	return new_state;
}

static void RunLightDataCapture(Disambiguator_data_t *d, const LightcapElement *le) {
	if (le->length > d->sweep_data[le->sensor_id].length)
		d->sweep_data[le->sensor_id] = *le;
}

static void PropagateState(Disambiguator_data_t *d, const LightcapElement *le);
static void RunACodeCapture(int target_acode, Disambiguator_data_t *d, const LightcapElement *le) {
	if (le->length < 100)
		return;

	int acode = find_acode(le->length);
	SurviveContext *ctx = d->so->ctx;

	uint32_t time_error_d0 = abs(ACODE_TIMING(target_acode) - le->length);
	uint32_t time_error_d1 = abs(ACODE_TIMING(target_acode | DATA_BIT) - le->length);
	uint32_t error = time_error_d0 > time_error_d1 ? time_error_d1 : time_error_d0;

	DEBUG_TB("acode %d %d 0x%x (%d)", target_acode, le->length, acode, error);
	if (error > 1250) {
		if (d->confidence < 3) {
			SetState(d, le, LS_UNKNOWN);
			assert(false);
		}
		d->confidence -= 3;
		return;
	}

	if (d->confidence < 100)
		d->confidence++;
	d->last_sync_timestamp += le->timestamp;
	d->last_sync_length += le->length;
	d->last_sync_count++;
}

static void PropagateState(Disambiguator_data_t *d, const LightcapElement *le) {
	int le_offset = le->timestamp > d->mod_offset
						? (le->timestamp - d->mod_offset + 10000) % LS_Params[LS_END].offset
						: (0xFFFFFFFF - d->mod_offset + le->timestamp + 10000) % LS_Params[LS_END].offset;

	enum LighthouseState new_state = LighthouseState_findByOffset(le_offset);
	SurviveContext *ctx = d->so->ctx;
	DEBUG_TB("new %u %d %d %d %d", le->timestamp, le->length, le_offset, LS_Params[d->state].offset,
			 LS_Params[new_state].offset);

	if (d->state != new_state) {
		if (LS_Params[d->state].is_sweep == 0) {
			if (d->last_sync_count > 0) {
				LightcapElement lastSync = get_last_sync(d);
				uint32_t mo = SolveForMod_Offset(d, d->state, &lastSync);
				DEBUG_TB("New mod offset diff %d", (int)d->mod_offset - (int)mo);
				d->mod_offset = mo;
				int acode = find_acode(lastSync.length);
				assert((acode | DATA_BIT) == (LS_Params[d->state].acode | DATA_BIT));
				ctx->lightproc(d->so, -LS_Params[d->state].lh - 1, acode, 0, lastSync.timestamp, lastSync.length,
							   LS_Params[d->state].lh);
			}
		} else {
			for (int i = 0; i < SENSORS_PER_OBJECT; i++) {
				if (d->sweep_data[i].length > 0) {
					d->so->ctx->lightproc(
						d->so, i, LS_Params[d->state].acode,
						timestamp_diff(d->sweep_data[i].timestamp, d->mod_offset + LS_Params[d->state].offset),
						d->sweep_data[i].timestamp, d->sweep_data[i].length, LS_Params[d->state].lh);
				}
			}
		}

		SetState(d, le, new_state);
	}

	const LighthouseStateParameters *param = &LS_Params[d->state];
	DEBUG_TB("param %u %d %d %d %d %d", le->timestamp, param->acode, le->length, le_offset, new_state,
			 LS_Params[d->state].offset);

	if (param->is_sweep == 0) {
		RunACodeCapture(param->acode, d, le);
	} else {
		DEBUG_TB("Logic for sweep %d", le->length);
		// assert( le->length < 2200);
		RunLightDataCapture(d, le);
	}
}

void DisambiguatorTimeBased(SurviveObject *so, const LightcapElement *le) {
	SurviveContext *ctx = so->ctx;

	// Note, this happens if we don't have config yet -- just bail
	if (so->sensor_ct == 0) {
		return;
	}

	if (so->disambiguator_data == NULL) {
		DEBUG_TB("Initializing Disambiguator Data for TB %d", so->sensor_ct);
		so->disambiguator_data = Disambiguator_data_t_ctor(so);
	}

	Disambiguator_data_t *d = so->disambiguator_data;
	if (d->stabalize < 500) {
		d->stabalize++;
		return;
	}

	if (d->state == LS_UNKNOWN) {
		enum LighthouseState new_state = AttemptFindState(d, le);
		if (new_state != LS_UNKNOWN) {
			LightcapElement lastSync = get_last_sync(d);
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

REGISTER_LINKTIME(DisambiguatorTimeBased);
