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
	void **user = survive_object_plugin_data(so, PoserKalmanOnly);
	struct PoserIMUData_t *dd = *user;

	if (!dd) {
		*user = dd = SV_CALLOC(sizeof(struct PoserIMUData_t));
	}

	switch (pt) {
	case POSERDATA_IMU: {
		PoserDataIMU *imu = (PoserDataIMU *)pd;

		if (so->tracker->model.t == 0) {
			so->tracker->model.t = imu->hdr.timecode / (FLT)so->timebase_hz;
		}

		while (so->tracker->stats.obs_count < 30) {
			SurvivePose pose = {0};
			LinmathVec3d up = {0, 0, 1};

			LinmathQuat q;
			LinmathVec3d imuWorld;
			quatrotatevector(imuWorld, so->tracker->state.Pose.Rot, imu->accel);

			quatfrom2vectors(q, imuWorld, up);
			quatrotateabout(pose.Rot, q, so->tracker->state.Pose.Rot);

			CN_CREATE_STACK_MAT(R, 7, 7);
			cn_set_diag_val(&R, 1);
			survive_kalman_tracker_integrate_observation(&imu->hdr, so->tracker, &pose, &R);
			return 0;
		}

		return 0;
	}
	case POSERDATA_DISASSOCIATE: {
		*user = 0;
		free(dd);
		return 0;
	}

	default:
		return -1;
	}
	return -1;
}

REGISTER_POSER(PoserKalmanOnly)
