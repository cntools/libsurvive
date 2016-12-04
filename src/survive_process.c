//<>< (C) 2016 C. N. Lohr, FULLY Under MIT/x11 License.
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include "survive_internal.h"

void survive_light_process( struct SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode  )
{
	//timeinsweep = 200,000 1/48,000,000ths of a second is "center-of-image"
	//printf( "L %s %d %d %d %d\n", so->codename, timecode, sensor_id, acode, timeinsweep );
}

void survive_imu_process( struct SurviveObject * so, int16_t * accelgyro, uint32_t timecode, int id )
{
	if( so->codename[0] != 'H' )
	{
//		printf( "I %s %d %d %d %d %d %d %d %d\n", so->codename, timecode, accelgyro[0], accelgyro[1], accelgyro[2], accelgyro[3], accelgyro[4], accelgyro[5], id );
	}
}
