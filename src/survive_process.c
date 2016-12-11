//<>< (C) 2016 C. N. Lohr, FULLY Under MIT/x11 License.
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include "survive_internal.h"


int bufferpts[32*2];
char buffermts[32*128];
int buffertimeto[32];

void survive_light_process( struct SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode  )
{
	if( acode == -1 ) return;

	if( acode == 0 || acode == 2 ) //data = 0
	{
//		printf( "L, X, %s, %d, %d, %d, %d\n", so->codename, timecode, sensor_id, acode, timeinsweep );
		bufferpts[sensor_id*2+0] = (timeinsweep-170000)/100;
		buffertimeto[sensor_id] = 0;
		//printf( "X: %d\n",bufferpts[sensor_id*2+0] );
			//480-(timeinsweep)/1000; // Full scan
	}
	if( acode == 1 || acode == 3 ) //data = 1
	{
//		printf( "L, Y, %s, %d, %d, %d, %d\n", so->codename, timecode, sensor_id, acode, timeinsweep );
		bufferpts[sensor_id*2+1] = 480-(timeinsweep-140000)/100;
		//printf( "Y: %d\n",bufferpts[sensor_id*2+1] );
		buffertimeto[sensor_id] = 0;

			//480-(timeinsweep)/1000; //Full scan
	}


	//timeinsweep = 200,000 1/48,000,000ths of a second is "center-of-image"
}

void survive_imu_process( struct SurviveObject * so, int16_t * accelgyro, uint32_t timecode, int id )
{
	//if( so->codename[0] == 'H' )
	if( 0 )
	{
		printf( "I %s %d %d %d %d %d %d %d %d\n", so->codename, timecode, accelgyro[0], accelgyro[1], accelgyro[2], accelgyro[3], accelgyro[4], accelgyro[5], id );
	}
}
