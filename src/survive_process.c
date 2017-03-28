//<>< (C) 2016 C. N. Lohr, FULLY Under MIT/x11 License.
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include "survive_cal.h"

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
			.pt = POSERDATA_LIGHT,
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


void survive_default_imu_process( SurviveObject * so, int mask, FLT * accelgyromag, uint32_t timecode, int id )
{
	if( so->PoserFn )
	{
		PoserDataIMU imu = {
			.pt = POSERDATA_IMU,
			.datamask = mask,
			.accel = { accelgyromag[0], accelgyromag[1], accelgyromag[2] },
			.gyro = {  accelgyromag[3], accelgyromag[4], accelgyromag[5] },
			.mag = {   accelgyromag[6], accelgyromag[7], accelgyromag[8] },
			.timecode = timecode,
		};
		so->PoserFn( so, (PoserData *)&imu );
	}
}

