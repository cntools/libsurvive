#include "persistent_scene.h"
#include "linmath.h"
#include <stdlib.h>
#include <string.h>
#include <survive.h>

bool PersistentScene_isStillValid(const PersistentScene *self, uint32_t timecode_now, uint32_t idx, int lh) {
	const uint32_t *data_timecode = self->timecode[idx][lh];
	return !(timecode_now - data_timecode[0] > self->tolerance || timecode_now - data_timecode[1] > self->tolerance);
}

void PersistentScene_add(PersistentScene *self, SurviveObject *so, PoserDataLight *lightData) {
	int axis = (lightData->acode & 1);
	uint32_t *data_timecode = &self->timecode[lightData->sensor_id][lightData->lh][axis];
	FLT *angle = &self->angles[lightData->sensor_id][lightData->lh][axis];

	*angle = lightData->angle;
	*data_timecode = lightData->timecode;
}

void PersistentScene_ForEachCorrespondence(PersistentScene *self, PersistentScene_ForEachCorrespondence_fn fn,
										   SurviveObject *so, uint32_t timecode_now, void *user) {
	for (int lh = 0; lh < NUM_LIGHTHOUSES; lh++) {
		for (size_t i = 0; i < so->sensor_ct; i++) {
			if (PersistentScene_isStillValid(self, timecode_now, i, lh)) {
				double *pts = self->angles[i][lh];
				fn(so, lh, i, pts, user);
			}
		}
	}
}

void PersistentScene_ctor(PersistentScene *self) {
	memset(self, 0, sizeof(PersistentScene));
	self->tolerance = 1500000;
}
