//<>< (C) 2016 C. N. Lohr, FULLY Under MIT/x11 License.
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include "survive_cal.h"

//XXX TODO: Once data is avialble in the context, use the stuff here to handle converting from time codes to
//proper angles, then from there perform the rest of the solution. 

void survive_default_light_process( SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length  )
{
	SurviveContext * ctx = so->ctx;
	int base_station = acode >> 2;
	int axis = acode & 1;

	if( ctx->calptr )
	{
		survive_cal_light( so, sensor_id, acode, timeinsweep, timecode, length );
	}

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
	ctx->angleproc( so, sensor_id, acode, timecode, length_sec, angle );
}


void survive_default_angle_process( SurviveObject * so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle )
{
	SurviveContext * ctx = so->ctx;
	if( ctx->calptr )
	{
		survive_cal_angle( so, sensor_id, acode, timecode, length, angle );
	}

	//TODO: Writeme!
}	


void survive_default_imu_process( SurviveObject * so, int16_t * accelgyro, uint32_t timecode, int id )
{
	//TODO: Writeme!
}

