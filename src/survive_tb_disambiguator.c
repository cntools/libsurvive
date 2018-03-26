//
#include "survive_internal.h"
#include <assert.h>
#include <math.h> /* for sqrt */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#define NUM_HISTORY 3

enum LightcapClassification { LCC_UNKNOWN = 0, LCC_SYNC = 1, LCC_SWEEP = 2 };

typedef struct {
	LightcapElement history[NUM_HISTORY];
	enum LightcapClassification classifications[NUM_HISTORY];
	int idx;
} SensorHistory_t;

typedef struct {
	LightcapElement last_sync;
	SensorHistory_t histories[];
} Disambiguator_data_t;

Disambiguator_data_t *Disambiguator_data_t_ctor(SurviveObject *so) {
	return calloc(sizeof(Disambiguator_data_t) + sizeof(SensorHistory_t) * so->sensor_ct, 1);
}

static uint32_t timestamp_diff(uint32_t recent, uint32_t prior) {
	if (recent > prior)
		return recent - prior;
	return (0xFFFFFFFF - prior) + recent;
}

#define LOWER_SYNC_TIME 2250
#define UPPER_SYNC_TIME 6750

static int circle_buffer_get(int idx, int offset) { return ((idx + offset) + NUM_HISTORY) % NUM_HISTORY; }

static enum LightcapClassification classify(Disambiguator_data_t *d, SensorHistory_t *history,
											const LightcapElement *le) {
	bool clearlyNotSync = le->length < LOWER_SYNC_TIME || le->length > UPPER_SYNC_TIME;

	if (clearlyNotSync) {
		return LCC_SWEEP;
	}

	uint32_t time_diff_last_sync = timestamp_diff(le->timestamp, d->last_sync.timestamp);
	uint32_t split_time = 399840; // 8.33ms in 48mhz
	uint32_t jitter_allowance = 4000;

	// If we are ~8.33ms ahead of the last sync; we are a sync
	if (d->last_sync.length > 0 && time_diff_last_sync < (split_time + jitter_allowance) &&
		time_diff_last_sync > (split_time - jitter_allowance)) {
		return LCC_SYNC;
	}

	if (d->last_sync.length > 0 && time_diff_last_sync < (split_time - jitter_allowance)) {
		return LCC_SWEEP;
	}

	int prevIdx = circle_buffer_get(history->idx, -1);
	uint32_t time_diff = timestamp_diff(le->timestamp, history->history[prevIdx].timestamp);

	// We don't have recent data; unclear
	if (time_diff > split_time - jitter_allowance) {
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

static int find_acode(uint32_t pulseLen) {
	const static int offset = 50;
	if (pulseLen < 2500 - offset)
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

void DisambiguatorTimeBased(SurviveObject *so, const LightcapElement *le) {
	SurviveContext *ctx = so->ctx;
	if (so->disambiguator_data == NULL) {
		SV_INFO("Initializing Disambiguator Data");
		so->disambiguator_data = Disambiguator_data_t_ctor(so);
	}

	Disambiguator_data_t *d = so->disambiguator_data;

	SensorHistory_t *history = &d->histories[le->sensor_id];
	int prevIdx = circle_buffer_get(history->idx, -1);
	uint32_t time_diff = timestamp_diff(le->timestamp, history->history[prevIdx].timestamp);
	enum LightcapClassification classification = update_histories(d, le);

	SV_INFO("Classification %d\t%d\t%d\t%u\t%u(%.02fms)", classification, le->sensor_id, le->length, le->timestamp,
			time_diff, (double)time_diff / 48000.0);
	if (classification == LCC_SYNC) {
		uint32_t time_diff_last_sync = timestamp_diff(le->timestamp, d->last_sync.timestamp);
		if (time_diff_last_sync > (0xFFFFFFFF / 2))
			time_diff_last_sync = 0xFFFFFFFFF - time_diff_last_sync;
		d->last_sync = *le;
		int acode = find_acode(le->length);
		SV_INFO("acode: %d 0x%x a:%d d:%d s:%d (%.02fms)", le->length, acode, acode & 1, (bool)(acode & 2),
				(bool)(acode & 4), (double)time_diff_last_sync / 48000.);
		assert(acode != -1);
	}
}

REGISTER_LINKTIME(DisambiguatorTimeBased);
