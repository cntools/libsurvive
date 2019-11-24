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
	struct PoserIMUData_t *dd = so->PoserData;

	if (!dd) {
		so->PoserData = dd = SV_CALLOC(1, sizeof(struct PoserIMUData_t));
		survive_imu_tracker_init(&dd->tracker, so);
	}

	switch (pt) {
		case POSERDATA_SYNC: {
			const PoserDataLight* pdl = (const struct PoserDataLight*)pd;
			FLT difference = (SurviveSensorActivations_difference(&so->activations, &dd->previous_sweep));
			if (pdl->lh == 0) {
				if (pdl->sensor_id == -3 &&
                    memcmp(&so->activations, &dd->previous_sweep, sizeof(so->activations)) != 0 && !isnan(difference)) {
                    //SV_INFO("Light diff %.8f %u", difference * 10e5, pdl->timecode % 1600000);
                    if(difference * 10e5 < .01) {
                        const FLT R[] = {difference * 10e6, 1000000000000.};
                        SurviveVelocity v = {0};
						survive_imu_tracker_integrate_velocity(&dd->tracker, pdl->hdr.timecode, R, &v);
					}
                }
                dd->previous_sweep = so->activations;
            }
			break;
		}
	case POSERDATA_IMU: {
		PoserDataIMU *imu = (PoserDataIMU *)pd;

		if (!dd->inited) {
			dd->inited = true;
			const FLT R[] = { 1. , 1.};
            SurvivePose pose = LinmathPose_Identity;
            const LinmathVec3d up = {0, 0, 1.};
            quatfrom2vectors(pose.Rot, imu->accel, up);

            LinmathVec3d rAcc;
            quatrotatevector(rAcc, pose.Rot, imu->accel);

			survive_imu_tracker_integrate_observation(imu->hdr.timecode, &dd->tracker, &pose, R);
			return 0;
		}

		//SV_INFO("%f", fabs(norm3d(imu->accel) - 1.));
		if( fabs(norm3d(imu->accel) - 1.) < .01) {
            const FLT R[] = { 1. , 1000000000000.};
            SurviveVelocity v = { 0 };
            //survive_imu_tracker_integrate_velocity(&dd->tracker, imu->timecode, R, &v);
        }
		survive_imu_tracker_integrate_imu(&dd->tracker, imu);

		SurvivePose pose = { 0 };
		SurviveVelocity velocity = survive_imu_velocity(&dd->tracker);
		survive_imu_tracker_update(&dd->tracker, imu->hdr.timecode, &pose);

		if (!quatiszero(pose.Rot)) {
			PoserData_poser_pose_func_with_velocity(pd, so, &pose, &velocity);
		}
		// if(magnitude3d(&dd->tracker->pose.Pos) > 1)
		// SV_GENERAL_ERROR("IMU drift");
		return 0;
	}
	}
	return -1;
}

REGISTER_LINKTIME(PoserIMU);
