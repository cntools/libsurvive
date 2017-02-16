//<>< (C) 2016 C. N. Lohr, FULLY Under MIT/x11 License.
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include "survive_cal.h"

//XXX TODO: Once data is avialble in the context, use the stuff here to handle converting from time codes to
//proper angles, then from there perform the rest of the solution. 

void survive_default_light_process( struct SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length  )
{
	struct SurviveContext * ctx = so->ctx;
	int base_station = acode >> 2;
	int axis = acode & 1;

	if( ctx->calptr )
	{
		survive_cal_light( so, sensor_id, acode, timeinsweep, timecode, length );
	}

	if( base_station > NUM_LIGHTHOUSES ) return;

	//No loner need sync information past this point.
	if( sensor_id < 0 ) return;

	float angle = (timeinsweep - 200000) * (1./200000. * 3.14159265359/2.0);

	//Need to now do angle correction.
#if 1
	struct BaseStationData * bsd = &ctx->bsd[base_station];

	//TODO!!!
#endif
	
}

void survive_default_imu_process( struct SurviveObject * so, int16_t * accelgyro, uint32_t timecode, int id )
{
	//TODO: Writeme!
}

