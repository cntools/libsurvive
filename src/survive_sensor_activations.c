#include <survive.h>

bool SurviveSensorActivations_isPairValid(const SurviveSensorActivations *self, uint32_t tolerance,
										  uint32_t timecode_now, uint32_t idx, int lh) {
	const uint32_t *data_timecode = self->timecode[idx][lh];
	return !(timecode_now - data_timecode[0] > tolerance || timecode_now - data_timecode[1] > tolerance);
}

void SurviveSensorActivations_add(SurviveSensorActivations *self, struct PoserDataLight *lightData) {
	int axis = (lightData->acode & 1);
	uint32_t *data_timecode = &self->timecode[lightData->sensor_id][lightData->lh][axis];
	FLT *angle = &self->angles[lightData->sensor_id][lightData->lh][axis];

	*angle = lightData->angle;
	*data_timecode = lightData->timecode;
}

// Roughly 31ms at a 48mhz clock rate
uint32_t SurviveSensorActivations_default_tolerance = 500000;