#include "string.h"
#include <assert.h>
#include <math.h>
#include <survive.h>

bool SurviveSensorActivations_isReadingValid(const SurviveSensorActivations *self, survive_timecode tolerance,
											 survive_timecode timecode_now, uint32_t idx, int lh, int axis) {
	const uint32_t *data_timecode = self->timecode[idx][lh];
	if (self->lh_gen != 1 && self->lengths[idx][lh][axis] == 0)
		return false;

	if (isnan(self->angles[idx][lh][axis]))
		return false;

	return survive_timecode_difference(timecode_now, data_timecode[axis]) <= tolerance;
}
bool SurviveSensorActivations_isPairValid(const SurviveSensorActivations *self, uint32_t tolerance,
										  uint32_t timecode_now, uint32_t idx, int lh) {
	const uint32_t *data_timecode = self->timecode[idx][lh];
	if (self->lh_gen != 1 && (self->lengths[idx][lh][0] == 0 || self->lengths[idx][lh][1] == 0))
		return false;

	if (isnan(self->angles[idx][lh][0]) || isnan(self->angles[idx][lh][1]))
		return false;

	return !(timecode_now - data_timecode[0] > tolerance || timecode_now - data_timecode[1] > tolerance);
}

survive_timecode SurviveSensorActivations_stationary_time(const SurviveSensorActivations *self) {
	survive_long_timecode last_imu = ((survive_long_timecode)self->rollover_count << 32u) | self->last_imu;
	survive_long_timecode last_move = self->last_movement;
	survive_long_timecode time_elapsed = last_imu - last_move;
	if (time_elapsed > 0xFFFFFFFF)
		return 0xFFFFFFFF;

	return time_elapsed;
}

void SurviveSensorActivations_add_imu(SurviveSensorActivations *self, struct PoserDataIMU *imuData) {
	if (self->last_imu > imuData->hdr.timecode) {
		self->rollover_count++;
	}
	self->last_imu = imuData->hdr.timecode;

	if (norm3d(imuData->gyro) > .05) {
		survive_long_timecode long_timecode =
			((survive_long_timecode)self->rollover_count << 32u) | imuData->hdr.timecode;
		self->last_movement = long_timecode;
	}

	for (int i = 0; i < 3; i++) {
		self->accel[i] = .98 * self->accel[i] + .02 * imuData->accel[i];
	}
	for (int i = 0; i < 3; i++) {
		self->gyro[i] = .98 * self->gyro[i] + .02 * imuData->gyro[i];
	}
	for (int i = 0; i < 3; i++) {
		self->mag[i] = .98 * self->mag[i] + .02 * imuData->mag[i];
	}
}
void SurviveSensorActivations_add_gen2(SurviveSensorActivations *self, struct PoserDataLightGen2 *lightData) {
	self->lh_gen = 1;
	int axis = lightData->plane;
	PoserDataLight *l = &lightData->common;
	uint32_t *data_timecode = &self->timecode[l->sensor_id][l->lh][axis];

	FLT *angle = &self->angles[l->sensor_id][l->lh][axis];

	*data_timecode = l->hdr.timecode;
	*angle = l->angle;
}

SURVIVE_EXPORT void SurviveSensorActivations_ctor(SurviveSensorActivations *self) {
	memset(self, 0, sizeof(SurviveSensorActivations));

	for (int i = 0; i < SENSORS_PER_OBJECT; i++) {
		for (int j = 0; j < NUM_GEN2_LIGHTHOUSES; j++) {
			for (int h = 0; h < 2; h++) {
				self->angles[i][j][h] = NAN;
			}
		}
	}
}

void SurviveSensorActivations_add(SurviveSensorActivations *self, struct PoserDataLightGen1 *_lightData) {
	int axis = (_lightData->acode & 1);
	PoserDataLight *lightData = &_lightData->common;
	uint32_t *data_timecode = &self->timecode[lightData->sensor_id][lightData->lh][axis];

	FLT *angle = &self->angles[lightData->sensor_id][lightData->lh][axis];
	uint32_t *length = &self->lengths[lightData->sensor_id][lightData->lh][axis];
	// assert(*length == 0 || fabs(*angle - lightData->angle) < 0.05);

	*angle = lightData->angle;
	*data_timecode = lightData->hdr.timecode;
	*length = (uint32_t)(_lightData->length * 48000000);
}

FLT SurviveSensorActivations_difference(const SurviveSensorActivations *rhs, const SurviveSensorActivations *lhs) {
	FLT rtn = 0;
	int cnt = 0;
	for(size_t i = 0;i < SENSORS_PER_OBJECT;i++) {
		for (size_t lh = 0; lh < NUM_GEN1_LIGHTHOUSES; lh++) {
			for(size_t axis = 0;axis < 2;axis++) {
				if(rhs->lengths[i][lh][axis] > 0 && lhs->lengths[i][lh][axis] > 0) {
					FLT diff = rhs->angles[i][lh][axis] - lhs->angles[i][lh][axis];
					rtn += diff * diff;
					cnt++;
				}
			}
		}
	}
	return rtn / (double)cnt;
}

SURVIVE_EXPORT uint32_t SurviveSensorActivations_default_tolerance =
	(uint32_t)(48000000 /*mhz*/ * (16.7 /*ms*/) / 1000) + 5000;
