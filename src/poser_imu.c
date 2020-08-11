#include "survive_imu.h"
#include <survive.h>

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <memory.h>

struct PoserIMUData_t {
	SurviveIMUTracker tracker;
	SurviveSensorActivations previous_sweep;
	bool inited;
};

int PoserIMU(SurviveObject *so, PoserData *pd) {
	PoserType pt = pd->pt;
	SurviveContext *ctx = so->ctx;
	struct PoserIMUData_t *dd = so->PoserFnData;

	if (!dd) {
		so->PoserFnData = dd = SV_CALLOC(1, sizeof(struct PoserIMUData_t));
		survive_imu_tracker_init(&dd->tracker, so);
	}

	switch (pt) {
	case POSERDATA_IMU: {
		PoserDataIMU *imu = (PoserDataIMU *)pd;

		if (dd->tracker.rot.t == 0) {
			dd->tracker.rot.t = dd->tracker.position.t = imu->hdr.timecode / (FLT)so->timebase_hz;

			SurvivePose pose;
			LinmathVec3d up = {0, 0, 1};
			quatfrom2vectors(pose.Rot, up, imu->accel);

			FLT R[7] = {0};
			// survive_imu_integrate_rotation(&dd->tracker, dd->tracker.rot.t, q, R);
			survive_imu_tracker_integrate_observation(imu->hdr.timecode, &dd->tracker, &pose, R);
			return 0;
		}

		survive_imu_tracker_integrate_imu(&dd->tracker, imu);
		survive_imu_tracker_report_state(pd, &dd->tracker);
		return 0;
	}
	case POSERDATA_LIGHT:
	case POSERDATA_LIGHT_GEN2: {
		if (dd->tracker.rot.t == 0) {
			return 0;
		}

		PoserDataLight *pdl = (PoserDataLight *)pd;

		survive_imu_tracker_integrate_light(&dd->tracker, pdl);
		survive_imu_tracker_report_state(pd, &dd->tracker);
	}

	default:
		return -1;
	}
	return -1;
}

REGISTER_LINKTIME(PoserIMU)
