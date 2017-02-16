// (C) 2016, 2017 Joshua Allen, MIT/x11 License.
// (C) 2016, 2017 <>< C. N. Lohr, Under MIT/x11 License.

// All OOTX code was written by J. Allen. Rest of the code is probably mostly CNLohr.

#include "survive_cal.h"
#include "survive_internal.h"
#include <math.h>

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
		//if( sensor_id == 0 && so->codename[0] == 'H' ) printf( "%d %f %f\n", acode, length, angle );
		break;
	}
}



