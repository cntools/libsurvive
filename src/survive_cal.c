// (C) 2016, 2017 Joshua Allen, MIT/x11 License.
// (C) 2016, 2017 <>< C. N. Lohr, Under MIT/x11 License.

// All OOTX code was written by J. Allen. Rest of the code is probably mostly CNLohr.

#include "survive_cal.h"
#include "survive_internal.h"

void ootx_packet_clbk_d(ootx_decoder_context *ct, ootx_packet* packet)
{
	struct SurviveContext * ctx = (struct SurviveContext*)(ct->user);
	struct SurviveCalData * cd = ctx->calptr;
	int id = ct->user1;

	printf( "Got OOTX packet %d %p\n", id, cd );
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
			if( lhid >= NUM_LIGHTHOUSES-1 && so->codename[0] == 'H' )
			{
				uint8_t dbit = (acode & 2)?0xff:0x00;
				printf( "%s %d %d %d\n", so->codename, lhid, acode, dbit );
				ootx_process_bit( &cd->ootx_decoders[lhid], dbit );
			}
		}
			
		break;
	}
}

