//
#include "survive_internal.h"
#include <assert.h>
#include <math.h> /* for sqrt */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define NUM_HISTORY 3

/**
 * The lighthouses go in the following order:
 *
 *     Ticks  State
 *         0  ACode 0b1x0 (4)
 *    20 000  ACode 0b0x0 (0)
 *            LH A X Sweep
 *   400 000  ACode 0b1x1 (5)
 *   420 000  ACode 0b0x1 (1)
 *            LH A Y SWEEP
 *   800 000  ACode 0b0x0 (0)
 *   820 000  ACode 0b1x0 (4)
 *            LH B X Sweep
 * 1 200 000  ACode 0b0x1 (1)
 * 1 220 000  ACode 0b1x1 (5)
 *            LH B Y SWEEP
 * 1 600 000  < REPEAT >
 *
 * NOTE: Obviously you cut the data bit out for this
 */

enum LighthouseState {
	LS_UNKNOWN = 0

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

enum LightcapClassification { LCC_UNKNOWN = 0, LCC_SYNC = 1, LCC_SWEEP = 2 };

typedef struct {
	LightcapElement history[NUM_HISTORY];
	enum LightcapClassification classifications[NUM_HISTORY];
	int idx;
} SensorHistory_t;

typedef struct {
	uint64_t last_sync_timestamp;
	uint64_t last_sync_length;
	int last_sync_count;
	bool lastWasSync;

	uint32_t mod_offset;
	LighthouseState state;

	SensorHistory_t histories[];
} Disambiguator_data_t;

Disambiguator_data_t *Disambiguator_data_t_ctor(SurviveObject *so) {
	return calloc(1, sizeof(Disambiguator_data_t) + sizeof(SensorHistory_t) * so->sensor_ct);
}

static uint32_t timestamp_diff(uint32_t recent, uint32_t prior) {
	if (recent > prior)
		return recent - prior;
	return (0xFFFFFFFF - prior) + recent;
}

static int find_acode(uint32_t pulseLen) {
	const static int offset = 0;
	if (pulseLen < 2200 + offset)
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

void DisambiguatorTimeBased(SurviveObject *so, const LightcapElement *le) {
	SurviveContext *ctx = so->ctx;

	// Note, this happens if we don't have config yet -- just bail
	if (so->sensor_ct == 0) {
		return;
	}

	if (so->disambiguator_data == NULL) {
		SV_INFO("Initializing Disambiguator Data for TB %d", so->sensor_ct);
		so->disambiguator_data = Disambiguator_data_t_ctor(so);
	}

	Disambiguator_data_t *d = so->disambiguator_data;

	SensorHistory_t *history = &d->histories[le->sensor_id];
	int prevIdx = circle_buffer_get(history->idx, -1);
	uint32_t time_diff = timestamp_diff(le->timestamp, history->history[prevIdx].timestamp);
	enum LightcapClassification classification = update_histories(d, le);

	uint32_t time_diff_last_sync = timestamp_diff(le->timestamp, get_last_sync(d).timestamp);
	if (time_diff_last_sync > (0xFFFFFFFF / 2))
		time_diff_last_sync = 0xFFFFFFFFF - time_diff_last_sync;

	LightcapElement lastSync = get_last_sync(d);
	int acode = find_acode(lastSync.length);

	if (classification == LCC_SYNC) {
		LightcapElement lastSync = get_last_sync(d);
		if (d->lastWasSync == false || overlaps(&lastSync, le) == false) {

			if (lastSync.length) {
				int acode = find_acode(lastSync.length);
				SV_INFO("%.03f(%d)\tacode: %d 0x%x a:%d d:%d s:%d (%d)",
						timestamp_diff(le->timestamp, lastSync.timestamp) / 48000.,
						timestamp_diff(le->timestamp, lastSync.timestamp), lastSync.length, acode, acode & 1,
						(bool)(acode & 2), (bool)(acode & 4), ((acode >> 1) & 0x2) | (acode & 1));
				assert(acode != -1);
			}
			d->last_sync_timestamp = le->timestamp;
			d->last_sync_length = le->length;
			d->last_sync_count = 1;
		} else {
			d->last_sync_timestamp += le->timestamp;
			d->last_sync_length += le->length;
			d->last_sync_count++;
		}

		d->lastWasSync = true;

		lastSync = get_last_sync(d);
		// SV_INFO("acode building: %u %u %u", lastSync.length, lastSync.timestamp, lastSync.length +
		// lastSync.timestamp);
	} else {
		if (d->lastWasSync) {
			if (lastSync.length) {
				int acode = find_acode(lastSync.length);
				SV_INFO("start acode: %d 0x%x a:%d d:%d s:%d (%d)", lastSync.length, acode, acode & 1,
						(bool)(acode & 2), (bool)(acode & 4), ((acode >> 1) & 0x2) | (acode & 1));
				assert(acode != -1);
			}
		}
		d->lastWasSync = false;
	}

	if (classification == LCC_SWEEP)
		SV_INFO("%.02fms Classification %d\t%d\t%d\t%u\t%u\t(%.02fms)\ttime since last sync: %.02fms a:%d d:%d s:%d",
				(double)le->timestamp / 48000, classification, le->sensor_id, le->length, le->timestamp, time_diff,
				(double)time_diff / 48000.0, (double)time_diff_last_sync / 48000., acode & 1, (bool)(acode & 2),
				(bool)(acode & 4));
}

REGISTER_LINKTIME(DisambiguatorTimeBased);
