//<>< (C) 2016 C. N. Lohr, FULLY Under MIT/x11 License.
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include "survive_cal.h"

//XXX TODO: Once data is avialble in the context, use the stuff here to handle converting from time codes to
//proper angles, then from there perform the rest of the solution. 

void survive_default_light_process( struct SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length  )
{

	if( so->ctx->calptr )
	{
		survive_cal_light( so, sensor_id, acode, timeinsweep, timecode, length );
	}

	//TODO: Writeme!
}

void survive_default_imu_process( struct SurviveObject * so, int16_t * accelgyro, uint32_t timecode, int id )
{
	//TODO: Writeme!
}

