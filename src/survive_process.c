//<>< (C) 2016 C. N. Lohr, FULLY Under MIT/x11 License.
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include "survive_internal.h"


int bufferpts[32*2];
char buffermts[32*128];
int buffertimeto[32];

void survive_default_light_process( struct SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length  )
{
	//TODO: Writeme!
}

void survive_default_imu_process( struct SurviveObject * so, int16_t * accelgyro, uint32_t timecode, int id )
{
	//TODO: Writeme!
}


