#include "string.h"
#include <assert.h>
#include <math.h>
#include <survive.h>

STATIC_CONFIG_ITEM(MOVMENT_THRESHOLD_GYRO, "move-threshold-gyro", 'f', "Threshold to count gyro norms as moving", .05);
STATIC_CONFIG_ITEM(MOVMENT_THRESHOLD_ACC, "move-threshold-acc", 'f', "Threshold to count acc diff norms as moving",
				   .03);
STATIC_CONFIG_ITEM(MOVMENT_THRESHOLD_ANG, "move-threshold-ang", 'f', "Threshold to count light angle diffs as moving",
				   .015);

static FLT moveThresholdGyro = 0;
static FLT moveThresholdAcc = 0;
static FLT moveThresholdAng = 0;

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

survive_long_timecode survive_extend_time(const SurviveObject *so, survive_timecode time) {
	return SurviveSensorActivations_extend_time(&so->activations, time);
}
survive_long_timecode SurviveSensorActivations_extend_time(const SurviveSensorActivations *self,
														   survive_timecode time) {
	return ((survive_long_timecode)self->rollover_count << 32u) | time;
}
survive_timecode SurviveSensorActivations_stationary_time(const SurviveSensorActivations *self) {
	survive_long_timecode last_time = 0;
	if (self->imu_init_cnt > 0) {
		last_time = ((survive_long_timecode)self->rollover_count << 32u) | self->last_light;
	} else {
		last_time = ((survive_long_timecode)self->rollover_count << 32u) | self->last_imu;
	}
	survive_long_timecode last_move = self->last_movement;
	survive_long_timecode time_elapsed = last_time - last_move;
	if (time_elapsed > 0xFFFFFFFF)
		return 0xFFFFFFFF;

	return time_elapsed;
}

void SurviveSensorActivations_add_imu(SurviveSensorActivations *self, struct PoserDataIMU *imuData) {
	if (self->last_imu > imuData->hdr.timecode) {
		self->rollover_count++;
	}
	self->last_imu = imuData->hdr.timecode;

	if (self->imu_init_cnt > 0) {
		self->imu_init_cnt--;
		return;
	}

	if (isnan(self->accel[0])) {
		for (int i = 0; i < 3; i++) {
			self->accel[i] = imuData->accel[i];
			self->gyro[i] = imuData->gyro[i];
			self->mag[i] = imuData->mag[i];
		}
		self->last_movement = imuData->hdr.timecode;
	} else {
		for (int i = 0; i < 3; i++) {
			self->accel[i] = .98 * self->accel[i] + .02 * imuData->accel[i];
			self->gyro[i] = .98 * self->gyro[i] + .02 * imuData->gyro[i];
			self->mag[i] = .98 * self->mag[i] + .02 * imuData->mag[i];
		}
	}

	if (norm3d(imuData->gyro) > moveThresholdGyro || dist3d(self->accel, imuData->accel) > moveThresholdAcc) {
		survive_long_timecode long_timecode =
			((survive_long_timecode)self->rollover_count << 32u) | imuData->hdr.timecode;
		self->last_movement = long_timecode;
		// fprintf(stderr, "%f %f\n", norm3d(imuData->gyro), dist3d(self->accel, imuData->accel));
	}
}
void SurviveSensorActivations_add_gen2(SurviveSensorActivations *self, struct PoserDataLightGen2 *lightData) {
	self->lh_gen = 1;

	int axis = lightData->plane;
	PoserDataLight *l = &lightData->common;
	if (l->sensor_id >= SENSORS_PER_OBJECT)
		return;

	uint32_t *data_timecode = &self->timecode[l->sensor_id][l->lh][axis];

	FLT *angle = &self->angles[l->sensor_id][l->lh][axis];

	*data_timecode = l->hdr.timecode;
	*angle = l->angle;
	self->last_light = lightData->common.hdr.timecode;
}

SURVIVE_EXPORT void SurviveSensorActivations_ctor(SurviveObject *so, SurviveSensorActivations *self) {
	if (so) {
		moveThresholdAcc = survive_configf(so->ctx, MOVMENT_THRESHOLD_ACC_TAG, SC_GET, 0);
		moveThresholdGyro = survive_configf(so->ctx, MOVMENT_THRESHOLD_GYRO_TAG, SC_GET, 0);
		moveThresholdAng = survive_configf(so->ctx, MOVMENT_THRESHOLD_ANG_TAG, SC_GET, 0);
	}

	memset(self, 0, sizeof(SurviveSensorActivations));

	for (int i = 0; i < SENSORS_PER_OBJECT; i++) {
		for (int j = 0; j < NUM_GEN2_LIGHTHOUSES; j++) {
			for (int h = 0; h < 2; h++) {
				self->angles[i][j][h] = NAN;
			}
		}
	}

	for (int i = 0; i < 3; i++) {
		self->accel[i] = NAN;
	}

	self->imu_init_cnt = 30;
	self->lh_gen = -1;
}

void SurviveSensorActivations_add(SurviveSensorActivations *self, struct PoserDataLightGen1 *_lightData) {
	self->lh_gen = 0;
	if (self->imu_init_cnt > 0 && self->last_light > _lightData->common.hdr.timecode) {
		self->rollover_count++;
	}

	int axis = (_lightData->acode & 1);
	PoserDataLight *lightData = &_lightData->common;
	uint32_t *data_timecode = &self->timecode[lightData->sensor_id][lightData->lh][axis];

	FLT *angle = &self->angles[lightData->sensor_id][lightData->lh][axis];
	uint32_t *length = &self->lengths[lightData->sensor_id][lightData->lh][axis];
	// printf("error %10.7f\n", fabs(*angle - lightData->angle));
	if (*length == 0 || fabs(*angle - lightData->angle) > 0.05) {
		self->last_movement = 0;
		survive_long_timecode long_timecode =
			((survive_long_timecode)self->rollover_count << 32u) | lightData->hdr.timecode;
		self->last_movement = long_timecode;
	}

	if (*length == 0 || fabs(*angle - lightData->angle) > moveThresholdAcc) {
		survive_long_timecode long_timecode =
			((survive_long_timecode)self->rollover_count << 32u) | lightData->hdr.timecode;
		// assert(long_timecode > self->last_movement);
		self->last_movement = long_timecode;
	}

	*angle = lightData->angle;
	*data_timecode = lightData->hdr.timecode;
	*length = (uint32_t)(_lightData->length * 48000000);
	self->last_light = lightData->hdr.timecode;
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
