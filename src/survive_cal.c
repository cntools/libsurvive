// (C) 2016, 2017 Joshua Allen, MIT/x11 License.
// (C) 2016, 2017 <>< C. N. Lohr, Under MIT/x11 License.

// All OOTX code was written by J. Allen. Rest of the code is probably mostly CNLohr.

#include "survive_cal.h"
#include "survive_internal.h"
#include <math.h>
#include <string.h>

#define PTS_BEFORE_COMMON 32
#define NEEDED_COMMON_POINTS 20

static void handle_calibration( struct SurviveCalData *cd );
static void reset_calibration( struct SurviveCalData * cd );

void ootx_packet_clbk_d(ootx_decoder_context *ct, ootx_packet* packet)
{
	struct SurviveContext * ctx = (struct SurviveContext*)(ct->user);
	struct SurviveCalData * cd = ctx->calptr;
	int id = ct->user1;

	SV_INFO( "Got OOTX packet %d %p\n", id, cd );

	lighthouse_info_v6 v6;
	init_lighthouse_info_v6(&v6, packet->data);

	struct BaseStationData * b = &ctx->bsd[id];
	//print_lighthouse_info_v6(&v6);

	b->BaseStationID = v6.id;
	b->fcalphase[0] = v6.fcal_0_phase;
	b->fcalphase[1] = v6.fcal_1_phase;
	b->fcaltilt[0] = tan(v6.fcal_0_tilt);
	b->fcaltilt[1] = tan(v6.fcal_1_tilt);  //XXX??? Is this right? See https://github.com/cnlohr/libsurvive/issues/18
	b->fcalcurve[0] = v6.fcal_0_curve;
	b->fcalcurve[1] = v6.fcal_1_curve;
	b->fcalgibpha[0] = v6.fcal_0_gibphase;
	b->fcalgibpha[1] = v6.fcal_1_gibphase;
	b->fcalgibmag[0] = v6.fcal_0_gibmag;
	b->fcalgibmag[1] = v6.fcal_1_gibmag;
	b->OOTXSet = 1;
}

int survive_cal_get_status( struct SurviveContext * ctx, char * description, int description_length )
{
	struct SurviveCalData * cd = ctx->calptr;

	switch( cd->stage )
	{
	case 0:
		return snprintf( description, description_length, "Not calibrating" );
	case 1:
		return snprintf( description, description_length, "Collecting OOTX Data (%d:%d)", cd->ootx_decoders[0].buf_offset, cd->ootx_decoders[1].buf_offset );
	case 2:
		if( cd->found_common )
		{
			return snprintf( description, description_length, "Collecting Sweep Data %d/%d", cd->peak_counts, DRPTS );
		}
		else
		{
			return snprintf( description, description_length, "Searching for common watchman cal %d/%d", cd->peak_counts, PTS_BEFORE_COMMON );
		}
	default:
		return snprintf( description, description_length, "Unkown calibration state" );
	}
}

void survive_cal_install( struct SurviveContext * ctx )
{
	int i;
	struct SurviveCalData * cd = ctx->calptr = calloc( 1, sizeof( struct SurviveCalData ) );

	for( i = 0; i < NUM_LIGHTHOUSES; i++ )
	{
		ootx_init_decoder_context(&cd->ootx_decoders[i]);
		cd->ootx_decoders[i].user = ctx;
		cd->ootx_decoders[i].user1 = i;
	}

	cd->stage = 1;

	ootx_packet_clbk = ootx_packet_clbk_d;

	ctx->calptr = cd;
}


void survive_cal_light( struct SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length  )
{
	struct SurviveContext * ctx = so->ctx;
	struct SurviveCalData * cd = ctx->calptr;

	if( !cd ) return;

	switch( cd->stage )
	{
	default:
	case 0: //Default, inactive.
		break;

	case 1:
		//Collecting OOTX data.
		if( sensor_id < 0 )
		{
			int lhid = -sensor_id-1;
			if( lhid < NUM_LIGHTHOUSES && so->codename[0] == 'H' )
			{
				uint8_t dbit = (acode & 2)>>1;
				ootx_pump_bit( &cd->ootx_decoders[lhid], dbit );
			}
			int i;
			for( i = 0; i < NUM_LIGHTHOUSES; i++ )
				if( ctx->bsd[i].OOTXSet == 0 ) break;
			if( i == NUM_LIGHTHOUSES ) cd->stage = 2;  //If all lighthouses have their OOTX set, move on.
		}
		break;
	case 2:      //Taking in angle data.
		break;
	}
}

void survive_cal_angle( struct SurviveObject * so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle )
{
	struct SurviveContext * ctx = so->ctx;
	struct SurviveCalData * cd = ctx->calptr;

	if( !cd ) return;

	switch( cd->stage )
	{
	default:
	case 1:	//Collecting OOTX data. (Don't do anything here, yet.)
	case 0: //Default, inactive.
		break;
	case 2:
	{
		int sensid = sensor_id;
		if( strcmp( so->codename, "WM0" ) == 0 )
			sensid += 32;
		if( strcmp( so->codename, "WM1" ) == 1 )
			sensid += 64;

		int lighthouse = acode>>2;
		int axis = acode & 1;
		int ct = cd->all_counts[sensid][lighthouse][axis]++;
		cd->all_lengths[sensid][lighthouse][axis][ct] = length;
		cd->all_angles[sensid][lighthouse][axis][ct] = angle;
		if( ct > cd->peak_counts )
		{
			cd->peak_counts = ct;
			if( ct >= DRPTS )
				handle_calibration( cd ); //This will also reset all cals.
		}

		//TODO: Determine if there is a sensor on a watchman visible from both lighthouses.
		if( sensid >= 32 && !cd->found_common )
		{
			int k;
			int ok = 1;
			for( k = 0; k < NUM_LIGHTHOUSES; k++ )
			{

				if( cd->all_counts[sensid][k][0] < NEEDED_COMMON_POINTS || cd->all_counts[sensid][k][1] < NEEDED_COMMON_POINTS )
				{
					ok = 0;
					break;
				}
			}
			if( ok ) cd->found_common = 1;
		}

		if( cd->peak_counts > PTS_BEFORE_COMMON && !cd->found_common )
		{
			reset_calibration( cd );
		}

		break;
	}
	}
}

static void reset_calibration( struct SurviveCalData * cd )
{
	memset( cd->all_counts, 0, sizeof( cd->all_counts ) );
	cd->peak_counts = 0;
	cd->found_common = 0;
}

static void handle_calibration( struct SurviveCalData *cd )
{
	//Do stuff.

	reset_calibration( cd );
}
