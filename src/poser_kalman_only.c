#include "survive_kalman_tracker.h"
#include <survive.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>

struct PoserIMUData_t {
	SurviveSensorActivations previous_sweep;
	bool inited;
};

int PoserKalmanOnly(SurviveObject *so, PoserData *pd) {
	PoserType pt = pd->pt;
	SurviveContext *ctx = so->ctx;
	struct PoserIMUData_t *dd = so->PoserFnData;

	if (!dd) {
		so->PoserFnData = dd = SV_CALLOC(1, sizeof(struct PoserIMUData_t));
	}

	switch (pt) {
	case POSERDATA_IMU: {
		PoserDataIMU *imu = (PoserDataIMU *)pd;

		if (so->tracker->model.t == 0) {
			so->tracker->model.t = imu->hdr.timecode / (FLT)so->timebase_hz;

			SurvivePose pose;
			LinmathVec3d up = {0, 0, 1};
			quatfrom2vectors(pose.Rot, imu->accel, up);

			so->tracker->stats.obs_count = 30;
			survive_kalman_tracker_integrate_observation(&imu->hdr, so->tracker, &pose, 0);
			return 0;
		}

		return 0;
	}
	case POSERDATA_DISASSOCIATE: {
		if (dd == so->PoserFnData)
			so->PoserFnData = 0;
		free(dd);
		return 0;
	}

	default:
		return -1;
	}
	return -1;
}

REGISTER_LINKTIME(PoserKalmanOnly)
