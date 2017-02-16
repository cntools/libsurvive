// (C) 2016, 2017 <>< C. N. Lohr, Under MIT/x11 License.

// All OOTX code was written by J. Allen. Rest of the code is probably mostly CNLohr.

#ifndef _SURVIVE_CAL_H
#define _SURVIVE_CAL_H

//This is a file that is intended for use with capturing vive data during the
//setup phase.  This and survive_cal.c/.h should not be included on embedded
//uses of libsurvive.

//This file handles the following:
//  1: Decoding the OOTX data from the lighthouses.
//  2: Setting OOTX props in the survive context.
//  3: Collect a bunch of data with the vive pointed up and the watchment to either side.
//  4: Running the code to find the lighthouses.
//  5: Setting the information needed to develop the worldspace model in the SurviveContext.


#include <stdint.h>
#include "ootx_decoder.h"
#include "survive_internal.h"

void survive_cal_install( struct SurviveContext * ctx );
int survive_cal_get_status( struct SurviveContext * ctx, char * description, int max_data );

//void survive_cal_teardown( struct SurviveContext * ctx );

//Called from survive_default_light_process
void survive_cal_light( struct SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length  );
void survive_cal_angle( struct SurviveObject * so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle );

struct SurviveCalData
{
	//Stage:
	// 0: Idle
	// 1: Collecting OOTX data.
	int stage;

	//OOTX Data is sync'd off of 
	ootx_decoder_context ootx_decoders[NUM_LIGHTHOUSES];
};

#endif

