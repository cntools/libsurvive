#pragma once
#include "persistent_scene.h"
#include "stdbool.h"

#ifndef USE_DOUBLE
#define FLT double
#define USE_DOUBLE
#endif

#include "linmath.h"
#include <survive.h>

typedef struct {
	uint32_t tolerance;

	// If "lengths[...]" < 0, means not a valid piece of sweep information.
	FLT angles[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2]; // 2 Axes  (Angles in LH space)
	uint32_t timecode[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2];

	PoserDataIMU lastimu;

} PersistentScene;

typedef void (*PersistentScene_ForEachCorrespondence_fn)(SurviveObject *so, int lh, int sensor_idx, FLT *angles,
														 void *);
void PersistentScene_ForEachCorrespondence(PersistentScene *self, PersistentScene_ForEachCorrespondence_fn fn,
										   SurviveObject *so, uint32_t timecode_now, void *user);

void PersistentScene_add(PersistentScene *self, SurviveObject *so, PoserDataLight *lightData);

bool PersistentScene_isStillValid(const PersistentScene *self, uint32_t timecode_now, uint32_t idx, int lh);
void PersistentScene_ctor(PersistentScene *self);
