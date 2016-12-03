//<>< (C) 2016 C. N. Lohr
//
//Based off of https://github.com/collabora/OSVR-Vive-Libre
// Originally Copyright 2016 Philipp Zabel
// Originally Copyright 2016 Lubosz Sarnecki <lubosz.sarnecki@collabora.co.uk>
// Originally Copyright (C) 2013 Fredrik Hultin
// Originally Copyright (C) 2013 Jakob Bornecrantz
//
//But, re-written as best as I can to get it put under a /real/ open souce license.
//If there are portions of the code too similar to the original, I would like to know 
//so they can be re-written.
//
//Mostly vased off of vl_hid_reports (HTC Vive USB HID reports).

#include "survive_internal.h"

void survive_light_process( struct SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode  )
{
	//timeinsweep = 200,000 1/48,000,000ths of a second is "center-of-image"
	//printf( "L %s %d %d %d %d\n", so->codename, timecode, sensor_id, acode, timeinsweep );
}

void survive_imu_process( struct SurviveObject * so, int16_t * accelgyro, uint32_t timecode, int id )
{
//	if( so->codename[0] != 'H' )
//	{
//		printf( "I %s %d %d %d %d %d %d %d %d\n", so->codename, timecode, accelgyro[0], accelgyro[1], accelgyro[2], accelgyro[3], accelgyro[4], accelgyro[5], id );
//	}
}
