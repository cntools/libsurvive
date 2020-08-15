#include "survive_kalman_tracker.h"
#include <survive.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>

struct PoserIMUData_t {
	SurviveKalmanTracker tracker;
	SurviveSensorActivations previous_sweep;
	bool inited;
};

int PoserIMU(SurviveObject *so, PoserData *pd) {
	PoserType pt = pd->pt;
	SurviveContext *ctx = so->ctx;
	struct PoserIMUData_t *dd = so->PoserFnData;

	if (!dd) {
		so->PoserFnData = dd = SV_CALLOC(1, sizeof(struct PoserIMUData_t));
		survive_kalman_tracker_init(&dd->tracker, so);
	}

	switch (pt) {
	case POSERDATA_IMU: {
		PoserDataIMU *imu = (PoserDataIMU *)pd;

		if (dd->tracker.model.t == 0) {
			dd->tracker.model.t = imu->hdr.timecode / (FLT)so->timebase_hz;

			SurvivePose pose;
			LinmathVec3d up = {0, 0, 1};
			quatfrom2vectors(pose.Rot, imu->accel, up);

			survive_kalman_tracker_integrate_observation(&imu->hdr, &dd->tracker, &pose, 0);
			return 0;
		}

		survive_kalman_tracker_integrate_imu(&dd->tracker, imu);
		return 0;
	}
	case POSERDATA_LIGHT:
	case POSERDATA_LIGHT_GEN2: {
		if (dd->tracker.model.t == 0) {
			return 0;
		}

		PoserDataLight *pdl = (PoserDataLight *)pd;

		survive_kalman_tracker_integrate_light(&dd->tracker, pdl);
	} break;
	case POSERDATA_DISASSOCIATE: {

		survive_kalman_tracker_free(&dd->tracker);
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

REGISTER_LINKTIME(PoserIMU)
