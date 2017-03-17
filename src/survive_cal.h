// (C) 2016, 2017 <>< C. N. Lohr, Under MIT/x11 License.

// All OOTX code was written by J. Allen. Rest of the code is probably mostly CNLohr.

//XXX XXX XXX Warning:  This subsystem will likely be mostly re-written.

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

void survive_cal_install( SurviveContext * ctx );
int survive_cal_get_status( SurviveContext * ctx, char * description, int description_length );

//void survive_cal_teardown( struct SurviveContext * ctx );

//Called from survive_default_light_process
void survive_cal_light( SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length  );
void survive_cal_angle( SurviveObject * so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle );

#define MAX_SENSORS_TO_CAL 96

#define MIN_PTS_BEFORE_CAL 24
#define DRPTS 128

#define MAX_POSE_OBJECTS 10

#define MAX_CAL_PT_DAT (MAX_SENSORS_TO_CAL*NUM_LIGHTHOUSES*2)
struct SurviveCalData
{
	SurviveContext * ctx;
	//OOTX Data is sync'd off of the sync pulses coming from the lighthouses.
	ootx_decoder_context ootx_decoders[NUM_LIGHTHOUSES];

	//For statistics-gathering phase. (Stage 2/3)
	FLT all_lengths[MAX_SENSORS_TO_CAL][NUM_LIGHTHOUSES][2][DRPTS];
	FLT all_angles[MAX_SENSORS_TO_CAL][NUM_LIGHTHOUSES][2][DRPTS];
	int16_t all_counts[MAX_SENSORS_TO_CAL][NUM_LIGHTHOUSES][2];
	int16_t peak_counts;
	int8_t found_common;
	int8_t times_found_common;

	//For camfind (4+)
	//Index is calculated with:      int dataindex = sen*(2*NUM_LIGHTHOUSES)+lh*2+axis;
	FLT avgsweeps[MAX_CAL_PT_DAT];
	FLT avglens[MAX_CAL_PT_DAT];
	FLT stdsweeps[MAX_CAL_PT_DAT];
	FLT stdlens[MAX_CAL_PT_DAT];
	int ctsweeps[MAX_CAL_PT_DAT];

	int senid_of_checkpt; //This is a point on a watchman that can be used to check the lh solution.

	SurviveObject * poseobjects[MAX_POSE_OBJECTS];

	size_t numPoseObjects;

	PoserCB ConfigPoserFn;

	//Stage:
	// 0: Idle
	// 1: Collecting OOTX data.
	int8_t stage;
};


//The following function is not included in the core survive_cal and must be compiled from a camfind file.
//It should use data for stage 4 and report if it found the 
int survive_cal_lhfind( SurviveCalData * cd );


#endif

