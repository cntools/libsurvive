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

		survive_imu_tracker_integrate(dd, imu);

		PoserData_poser_pose_func(pd, so, &dd->pose);

		// if(magnitude3d(dd->pose.Pos) > 1)
		// SV_ERROR("IMU drift");
		return 0;
	}
	}
	return -1;
}

REGISTER_LINKTIME(PoserIMU);
