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

			LinmathQuat q;
			LinmathVec3d up = {0, 0, 1};
			quatfrom2vectors(q, up, imu->accel);

			FLT R[4] = {0};
			survive_imu_integrate_rotation(&dd->tracker, dd->tracker.rot.t, q, R);
			return 0;
		}

		survive_imu_tracker_integrate_imu(&dd->tracker, imu);

		SurvivePose pose = { 0 };
		survive_imu_tracker_predict(&dd->tracker, imu->hdr.timecode, &pose);
		if (!quatiszero(pose.Rot)) {
			SurviveVelocity velocity = survive_imu_velocity(&dd->tracker);
			PoserData_poser_pose_func_with_velocity(pd, so, &pose, &velocity);
		}

		return 0;
	}
	default:
		return -1;
	}
	return -1;
}

REGISTER_LINKTIME(PoserIMU)
