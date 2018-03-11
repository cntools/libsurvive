//<>< (C) 2016 C. N. Lohr, FULLY Under MIT/x11 License.
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include "survive_cal.h"
#include "survive_config.h"

//XXX TODO: Once data is avialble in the context, use the stuff here to handle converting from time codes to
//proper angles, then from there perform the rest of the solution. 

void survive_default_light_process( SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length, uint32_t lh)
{
	SurviveContext * ctx = so->ctx;
	int base_station = lh;
	int axis = acode & 1;
	if( ctx->calptr )
	{
		survive_cal_light( so, sensor_id, acode, timeinsweep, timecode, length, lh);
	}

	//We don't use sync times, yet.
	if( acode < -1 ) return;

	if( base_station > NUM_LIGHTHOUSES ) return;

	//No loner need sync information past this point.
	if( sensor_id < 0 ) return;
	FLT angle = (timeinsweep - so->timecenter_ticks) * (1./so->timecenter_ticks * 3.14159265359/2.0);

	//Need to now do angle correction.
#if 1
	BaseStationData * bsd = &ctx->bsd[base_station];

	//XXX TODO: This seriously needs to be worked on.  See: https://github.com/cnlohr/libsurvive/issues/18
	angle += bsd->fcalphase[axis];
//	angle += bsd->fcaltilt[axis] * predicted_angle(axis1);
	
	//TODO!!!
#endif

	FLT length_sec = length / (FLT)so->timebase_hz;
	ctx->angleproc( so, sensor_id, acode, timecode, length_sec, angle, lh);
}


void survive_default_angle_process( SurviveObject * so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle, uint32_t lh)
{
	SurviveContext * ctx = so->ctx;
	if( ctx->calptr )
	{
		survive_cal_angle( so, sensor_id, acode, timecode, length, angle, lh );
	}
	if( so->PoserFn )
	{
		PoserDataLight l = {
			.hdr =
				{
					.pt = POSERDATA_LIGHT,
				},
			.sensor_id = sensor_id,
			.acode = acode,
			.timecode = timecode,
			.length = length,
			.angle = angle,
			.lh = lh,
		};

		so->PoserFn( so, (PoserData *)&l );
	}
}	

void survive_default_button_process(SurviveObject * so, uint8_t eventType, uint8_t buttonId, uint8_t axis1Id, uint16_t axis1Val, uint8_t axis2Id, uint16_t axis2Val)
{
	// do nothing.
	//printf("ButtonEntry: eventType:%x, buttonId:%d, axis1:%d, axis1Val:%8.8x, axis2:%d, axis2Val:%8.8x\n",
	//	eventType,
	//	buttonId,
	//	axis1Id,
	//	axis1Val,
	//	axis2Id,
	//	axis2Val);
	//if (buttonId == 24 && eventType == 1) // trigger engage
	//{
	//	for (int j = 0; j < 6; j++)
	//	{
	//		for (int i = 0; i < 0x5; i++)
	//		{
	//			survive_haptic(so, 0, 0xf401, 0xb5a2, 0x0100);
	//			//survive_haptic(so, 0, 0xf401, 0xb5a2, 0x0100);
	//			OGUSleep(1000);
	//		}
	//		OGUSleep(20000);
	//	}
	//}
	//if (buttonId == 2 && eventType == 1) // trigger engage
	//{
	//	for (int j = 0; j < 6; j++)
	//	{
	//		for (int i = 0; i < 0x1; i++)
	//		{
	//			survive_haptic(so, 0, 0xf401, 0x05a2, 0xf100);
	//			//survive_haptic(so, 0, 0xf401, 0xb5a2, 0x0100);
	//			OGUSleep(5000);
	//		}
	//		OGUSleep(20000);
	//	}
	//}
}

void survive_default_raw_pose_process(SurviveObject *so, uint8_t lighthouse, SurvivePose *pose) {
	// print the pose;
	//printf("Pose: [%1.1x][%s][% 08.8f,% 08.8f,% 08.8f] [% 08.8f,% 08.8f,% 08.8f,% 08.8f]\n", lighthouse, so->codename, pos[0], pos[1], pos[2], quat[0], quat[1], quat[2], quat[3]);

}

void survive_default_lighthouse_pose_process(SurviveContext *ctx, uint8_t lighthouse, SurvivePose *pose) {
	if (pose) {
		ctx->bsd[lighthouse].Pose = *pose;
		ctx->bsd[lighthouse].PositionSet = 1;
	} else {
		ctx->bsd[lighthouse].PositionSet = 0;
	}

	config_set_lighthouse(ctx->lh_config, &ctx->bsd[lighthouse], lighthouse);
	config_save(ctx, "config.json");
}

void survive_default_imu_process( SurviveObject * so, int mask, FLT * accelgyromag, uint32_t timecode, int id )
{
	if( so->PoserFn )
	{
		PoserDataIMU imu = {
			.hdr =
				{
					.pt = POSERDATA_IMU,
				},
			.datamask = mask,
			.accel = {accelgyromag[0], accelgyromag[1], accelgyromag[2]},
			.gyro = {accelgyromag[3], accelgyromag[4], accelgyromag[5]},
			.mag = {accelgyromag[6], accelgyromag[7], accelgyromag[8]},
			.timecode = timecode,
		};
		so->PoserFn( so, (PoserData *)&imu );
	}
}

