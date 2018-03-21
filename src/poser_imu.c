#include <survive.h>
#include <survive_imu.h>

#include <stdio.h>
#include <stdlib.h>

int PoserIMU(SurviveObject *so, PoserData *pd) {
	PoserType pt = pd->pt;
	SurviveContext *ctx = so->ctx;
	SurviveIMUTracker *dd = so->PoserData;

	if (!dd) {
		so->PoserData = dd = malloc(sizeof(SurviveIMUTracker));
		*dd = (SurviveIMUTracker){};
	}

	switch (pt) {
	case POSERDATA_IMU: {
		PoserDataIMU *imu = (PoserDataIMU *)pd;

		survive_imu_tracker_integrate(so, dd, imu);

		PoserData_poser_raw_pose_func(pd, so, -1, &dd->pose);

		return 0;
	}
	}
	return -1;
}

REGISTER_LINKTIME(PoserIMU);
