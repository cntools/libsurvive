// (C) 2016, 2017 Joshua Allen, MIT/x11 License.
// (C) 2016, 2017 <>< C. N. Lohr, Under MIT/x11 License.

// All OOTX code was written by J. Allen. Rest of the code is probably mostly CNLohr.
//
// This file is primarily geared to the calibration phase, to produce the world cal information.
// Once world cal is produced, it's unlikely you will need this file at all.  The plan is
// to not include it at all on any stripped-down versions of libsurvive.
//

#include "survive_cal.h"
#include "survive_internal.h"
#include <math.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <linmath.h>

#include "survive_config.h"

#define PTS_BEFORE_COMMON 32
#define NEEDED_COMMON_POINTS 10
#define NEEDED_TIMES_OF_COMMON 5
#define DRPTS_NEEDED_FOR_AVG ((int)(DRPTS*3/4))




static void handle_calibration( struct SurviveCalData *cd );
static void reset_calibration( struct SurviveCalData * cd );

void ootx_packet_clbk_d(ootx_decoder_context *ct, ootx_packet* packet)
{
	SurviveContext * ctx = (SurviveContext*)(ct->user);
	SurviveCalData * cd = ctx->calptr;
	int id = ct->user1;

	SV_INFO( "Got OOTX packet %d %p", id, cd );

	lighthouse_info_v6 v6;
	init_lighthouse_info_v6(&v6, packet->data);

	BaseStationData * b = &ctx->bsd[id];
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

	config_set_lighthouse(ctx->lh_config,b,id);
//	config_save("config.json");
}

int survive_cal_get_status( struct SurviveContext * ctx, char * description, int description_length )
{
	struct SurviveCalData * cd = ctx->calptr;

	switch( cd->stage )
	{
	case 0:
		return snprintf( description, description_length, "0 Not calibrating" );
	case 1:
		return snprintf( description, description_length, "1 Collecting OOTX Data (%d:%d)", cd->ootx_decoders[0].buf_offset, cd->ootx_decoders[1].buf_offset );
	case 2:
	case 3:
		if( cd->found_common )
		{
			return snprintf( description, description_length, "%d Collecting Sweep Data %d/%d", cd->stage, cd->peak_counts, DRPTS );
		}
		else
		{
			return snprintf( description, description_length, "%d Searching for common watchman cal %d/%d (%d/%d)", cd->stage, cd->peak_counts, PTS_BEFORE_COMMON, cd->times_found_common, NEEDED_TIMES_OF_COMMON );
		}

	case 5:
		return snprintf( description, description_length, "%d LH Find complete.", cd->stage );

	case 4:
	default:
		return snprintf( description, description_length, "%d Unkown calibration state", cd->stage );
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
	cd->ctx = ctx;

	cd->hmd = survive_get_so_by_name( ctx, "HMD" );
	if( !cd->hmd )
	{
		SV_ERROR( "Error: cannot find any devices labeled HMD. Required for calibration" );
		free( cd );
		return;
	}



	const char * DriverName;
	const char * PreferredPoser = config_read_str( ctx->global_config_values, "ConfigPoser", "PoserDaveOrtho" );
	PoserCB PreferredPoserCB = 0;
	const char * FirstPoser = 0;
	printf( "Available posers:\n" );
	i = 0;	
	while( ( DriverName = GetDriverNameMatching( "Poser", i++ ) ) )
	{
		PoserCB p = GetDriver( DriverName );
		if( !PreferredPoserCB ) PreferredPoserCB = p;
		int ThisPoser = strcmp( DriverName, PreferredPoser ) == 0;
		if( ThisPoser ) PreferredPoserCB = p;
	}
	cd->ConfigPoserFn = PreferredPoserCB;
	printf( "Got config poser: %p\n", cd->ConfigPoserFn );
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

	int sensid = sensor_id;
	if( strcmp( so->codename, "WM0" ) == 0 )
		sensid += 32;
	if( strcmp( so->codename, "WM1" ) == 0 )
		sensid += 64;

	if( sensid >= MAX_SENSORS_TO_CAL || sensid < 0 ) return;

	int lighthouse = acode>>2;
	int axis = acode & 1;

	switch( cd->stage )
	{
	default:
	case 1:	//Collecting OOTX data. (Don't do anything here, yet.)
	case 0: //Default, inactive.
		break;
	case 2:
	{
		int ct = cd->all_counts[sensid][lighthouse][axis]++;
		cd->all_lengths[sensid][lighthouse][axis][ct] = length;
		cd->all_angles[sensid][lighthouse][axis][ct] = angle;
		if( ct > cd->peak_counts )
		{
			cd->peak_counts = ct;
		}

		//Determine if there is a sensor on a watchman visible from both lighthouses.
		if( sensid >= 32 )
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

		if( cd->peak_counts >= PTS_BEFORE_COMMON )
		{
/*			int tfc = cd->times_found_common;
			if( cd->found_common )
			{
				if( tfc >= NEEDED_TIMES_OF_COMMON )
				{
					SV_INFO( "Stage 2 moving to stage 3. %d %d %d", cd->peak_counts, cd->found_common, tfc );
					reset_calibration( cd );
					cd->stage = 3;
					cd->found_common = 1;
				}
				else
				{
					SV_INFO( "Stage 2 good - continuing. %d %d %d", cd->peak_counts, cd->found_common, tfc );
					reset_calibration( cd );
					cd->times_found_common = tfc+1;
				}
			}
			else
			{
				SV_INFO( "Stage 2 bad - redoing. %d %d %d", cd->peak_counts, cd->found_common, tfc );
				reset_calibration( cd );
				cd->times_found_common = 0;
			}*/

			SV_INFO( "Stage 2 moving to stage 3. %d %d", cd->peak_counts, cd->found_common );
			reset_calibration( cd );
			cd->stage = 3;
			cd->found_common = 1;
		}			

		break;
	}
	case 3:
	{
		int ct = cd->all_counts[sensid][lighthouse][axis]++;
		cd->all_lengths[sensid][lighthouse][axis][ct] = length;
		cd->all_angles[sensid][lighthouse][axis][ct] = angle;
		if( ct > cd->peak_counts )
		{
			cd->peak_counts = ct;
			if( ct >= DRPTS )
				handle_calibration( cd ); //This must also reset all cals.
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
	cd->times_found_common = 0;
	cd->stage = 2;
}

static void handle_calibration( struct SurviveCalData *cd )
{
	struct SurviveContext * ctx = cd->ctx;

	#define MAX_CAL_PT_DAT (MAX_SENSORS_TO_CAL*NUM_LIGHTHOUSES*2)

/*
	FLT avgsweeps[MAX_CAL_PT_DAT];
	FLT avglens[MAX_CAL_PT_DAT];
	FLT stdsweeps[MAX_CAL_PT_DAT];
	FLT stdlens[MAX_CAL_PT_DAT];
	int ctsweeps[MAX_CAL_PT_DAT];
*/

	memset( cd->ctsweeps, 0, sizeof( cd->ctsweeps ) );

	//Either advance to stage 4 or go resetting will go back to stage 2.
	//What is stage 4?  Are we done then?

	mkdir( "calinfo", 0755 );
	FILE * hists = fopen( "calinfo/histograms.csv", "w" );
	FILE * ptinfo = fopen( "calinfo/ptinfo.csv", "w" );
	int sen, axis, lh;
	for( sen = 0; sen < MAX_SENSORS_TO_CAL; sen++ )
	for( lh = 0; lh < NUM_LIGHTHOUSES; lh++ )
	for( axis = 0; axis < 2; axis++ )
	{
		int dpmax = cd->all_counts[sen][lh][axis];
		if( dpmax < MIN_PTS_BEFORE_CAL ) continue;
		int i;

		FLT sumsweepangle = 0;
		FLT sumlentime = 0;

		//Find initial guess at average
		for( i = 0; i < dpmax; i++ )
		{
			FLT sweepangle = cd->all_angles[sen][lh][axis][i];
			FLT datalen = cd->all_lengths[sen][lh][axis][i];
			sumsweepangle += sweepangle;
			sumlentime += datalen;
		}

		#define OUTLIER_ANGLE   0.001	//TODO: Tune
		#define OUTLIER_LENGTH	0.001	//TODO: Tune
		#define ANGLE_STDEV_TOO_HIGH 0.000001 //TODO: Tune

		FLT avgsweep = sumsweepangle / dpmax;
		FLT avglen = sumlentime / dpmax;
		int count = 0;

		FLT max_outlier_angle = 0;
		FLT max_outlier_length = 0;

		//Get rid of outliers
		for( i = 0; i < dpmax; i++ )
		{
			FLT sweepangle = cd->all_angles[sen][lh][axis][i];
			FLT datalen = cd->all_lengths[sen][lh][axis][i];
			FLT Sdiff = sweepangle - avgsweep;
			FLT Ldiff = datalen - avglen;
			FLT Sdiff2 = Sdiff * Sdiff;
			FLT Ldiff2 = Ldiff * Ldiff;

			if( Sdiff2 > max_outlier_angle ) max_outlier_angle = Sdiff2;
			if( Ldiff2 > max_outlier_length ) max_outlier_length = Ldiff2;

			if( Sdiff2 > OUTLIER_ANGLE || Ldiff2 > OUTLIER_LENGTH )
			{
				cd->all_lengths[sen][lh][axis][i] = -1;
			}
			else
			{
				count++;
			}
		}

		if( count < DRPTS_NEEDED_FOR_AVG )
		{
			printf( "DPAVG %d\n", count );
			//Not enough for this point to be considered.
			continue;
		}

		sumsweepangle = 0;
		sumlentime = 0;
		//Redo, finding new average:
		for( i = 0; i < dpmax; i++ )
		{
			FLT sweepangle = cd->all_angles[sen][lh][axis][i];
			FLT datalen = cd->all_lengths[sen][lh][axis][i];
			if( datalen < 0 ) continue;
			sumsweepangle += sweepangle;
			sumlentime += datalen;
		}

		avgsweep = sumsweepangle / count;
		avglen = sumlentime / count;

		FLT stddevang = 0;
		FLT stddevlen = 0;

		#define HISTOGRAMSIZE   31
		#define HISTOGRAMBINANG 0.00001  //TODO: Tune

		int histo[HISTOGRAMSIZE];
		memset( histo, 0, sizeof( histo ) );

		for( i = 0; i < dpmax; i++ )
		{
			FLT sweepangle = cd->all_angles[sen][lh][axis][i];
			FLT datalen = cd->all_lengths[sen][lh][axis][i];
			if( datalen < 0 ) continue;

			FLT Sdiff = sweepangle - avgsweep;
			FLT Ldiff = datalen - avglen;
			FLT Sdiff2 = Sdiff * Sdiff;
			FLT Ldiff2 = Ldiff * Ldiff;

			stddevang += Sdiff2;
			stddevlen += Ldiff2;

			int llm = Sdiff / HISTOGRAMBINANG + (HISTOGRAMSIZE/2.0);
			if( llm < 0 ) llm = 0;
			if( llm >= HISTOGRAMSIZE ) llm = HISTOGRAMSIZE-1;

			histo[llm]++;
		}

		stddevang /= count;
		stddevlen /= count;

		if( stddevang > ANGLE_STDEV_TOO_HIGH )
		{
			SV_INFO( "DROPPED: %02d:%d:%d dropped because stddev (%f) was too high.", sen, lh, axis, stddevang );
			continue;
		}

		fprintf( hists, "%02d_%d_%d, ", sen, lh, axis );

		for( i = 0; i < HISTOGRAMSIZE; i++ )
		{
			fprintf( hists, "%d ", histo[i] );
		}
		fprintf( hists, "\n" );

		fprintf( ptinfo, "%d %d %d %d %f %f %f %f %f %f\n", sen, lh, axis, count, avgsweep, avglen*1000000, stddevang*1000000000, stddevlen*1000000000, max_outlier_length*1000000000, max_outlier_angle*1000000000 );

		int dataindex = sen*(2*NUM_LIGHTHOUSES)+lh*2+axis;
		cd->avgsweeps[dataindex] = avgsweep;
		cd->avglens[dataindex] = avglen;
		cd->stdsweeps[dataindex] = stddevang;
		cd->stdlens[dataindex] = stddevlen;
		cd->ctsweeps[dataindex] = count;
	}
	fclose( hists );
	fclose( ptinfo );
/*
	//Comb through data and make sure we still have a sensor on a WM that 
	//We don't do this anymore.

	int bcp_senid = 0;
	int bcp_count = 0;
	for( sen = 0; sen < MAX_SENSORS_TO_CAL; sen++ )
	{
		int ct0 = cd->ctsweeps[sen*4+0];
		int ct1 = cd->ctsweeps[sen*4+0];
		int ct2 = cd->ctsweeps[sen*4+0];
		int ct3 = cd->ctsweeps[sen*4+0];

		if( ct0 > ct1 ) ct0 = ct1;
		if( ct0 > ct2 ) ct0 = ct2;
		if( ct0 > ct3 ) ct0 = ct3;

		if( ct0 > bcp_count ) { bcp_count = ct0; bcp_senid = sen; }
	}

	if( bcp_count < DRPTS_NEEDED_FOR_AVG )
	{
		SV_INFO( "Stage 3 could not find a suitable common point on a watchman" );
		reset_calibration( cd );
		return;
	}
	cd->senid_of_checkpt = bcp_senid;
*/
	
	int i, j;
	PoserDataFullScene fsd;
	fsd.pt = POSERDATA_FULL_SCENE;
	for( j = 0; j < NUM_LIGHTHOUSES; j++ )
	for( i = 0; i < SENSORS_PER_OBJECT; i++ )
	{
		int gotdata = 0;

		int dataindex = i*(2*NUM_LIGHTHOUSES)+j*2+0;

		if( cd->ctsweeps[dataindex+0] < DRPTS_NEEDED_FOR_AVG ||
			cd->ctsweeps[dataindex+1] < DRPTS_NEEDED_FOR_AVG )
		{
			fsd.lengths[i][j][0] = -1;
			fsd.lengths[i][j][1] = -1;
			continue;
		}
		fsd.lengths[i][j][0] = cd->avglens[dataindex+0];
		fsd.lengths[i][j][1] = cd->avglens[dataindex+1];
		fsd.angles[i][j][0] = cd->avgsweeps[dataindex+0];
		fsd.angles[i][j][1] = cd->avgsweeps[dataindex+1];
	}

	cd->ConfigPoserFn( cd->hmd, (PoserData*)&fsd );
	if( 1 )
	{
		static int notfirstcal = 0;
		SV_INFO( "Stage 4 succeeded. Inverting %d", notfirstcal );

		if( !notfirstcal )
		{
			//  XXX This part is /all/ wrong.
			//  XXX This part is /all/ wrong.
			//  XXX This part is /all/ wrong.

			//OK! We've arrived.  Now, we have to get the LH's pose from.
			int lh;
			for( lh = 0; lh < NUM_LIGHTHOUSES; lh++ )
			{
				SurvivePose * objfromlh = &cd->hmd->FromLHPose[lh];
				SurvivePose * lhp = &ctx->bsd[lh].Pose;


/*				lhp->Pos[0] = objfromlh->Pos[0];
				lhp->Pos[1] = objfromlh->Pos[1];
				lhp->Pos[2] = objfromlh->Pos[2];*/

				lhp->Rot[0] =-objfromlh->Rot[0];
				lhp->Rot[1] = objfromlh->Rot[1];
				lhp->Rot[2] = objfromlh->Rot[2];
				lhp->Rot[3] = objfromlh->Rot[3];

				quatrotatevector( lhp->Pos, lhp->Rot, objfromlh->Pos );

				//Write lhp from the inverse of objfromlh
				//quatrotatevector( lhp->Pos, lhp->Rot, lhp->Pos );


				fprintf( stderr, "%f, %f, %f\n", objfromlh->Pos[0], objfromlh->Pos[1], objfromlh->Pos[2] );
				fprintf( stderr, "%f, %f, %f, %f\n", objfromlh->Rot[0], objfromlh->Rot[1], objfromlh->Rot[2], objfromlh->Rot[3] );

				/*
					-0.204066 3.238746 -0.856369
					0.812203 -0.264897 0.505599 0.120520
					-0.204066 3.238746 -0.856369

					0.020036 3.162476 -0.117896
					-0.322354 0.450869 0.346281 0.756898
					0.020036 3.162476 -0.117896
				*/

				/* Facing up, moved -x 1m.
					====> 0.446818 -0.309120 -0.747630 ====> -0.222356 -0.701865 -0.558656
					====> -0.341064 0.099785 0.887015 ====> 0.619095 0.727263 0.029786 
						IN PLACE, but rotated 90 * up.
					====> 0.374516 -0.370583 -0.606996 ====> -0.120238 -0.670330 -0.426896
					====> -0.231758 0.070437 0.765982 ====> 0.497615 0.625761 0.078759 
				*/

				printf( "\n" );

			}
			notfirstcal = 1;
		}
		else
		{
			for( lh = 0; lh < NUM_LIGHTHOUSES; lh++ )
			{
				SurvivePose * objfromlh = &cd->hmd->FromLHPose[lh];
				SurvivePose * lhp = &ctx->bsd[lh].Pose;

				FLT pos[3];
				quatrotatevector( pos, lhp->Rot, objfromlh->Pos );

				pos[0] -= lhp->Pos[0];
				pos[1] -= lhp->Pos[1];
				pos[2] -= lhp->Pos[2];

				//FLT rot[4] = { 
				//	[0],
				//	lhp->Rot[1],
				//	lhp->Rot[2],
				//	lhp->Rot[3] };
				//quatrotatevector( pos, lhp->Rot, pos );

				fprintf( stderr, "%f, %f, %f\n", objfromlh->Pos[0], objfromlh->Pos[1], objfromlh->Pos[2] );
				fprintf( stderr, "%f, %f, %f, %f\n", objfromlh->Rot[0], objfromlh->Rot[1], objfromlh->Rot[2], objfromlh->Rot[3] );
				fprintf( stderr, "%f, %f, %f\n", lhp->Pos[0], lhp->Pos[1], lhp->Pos[2] );
				fprintf( stderr, "%f, %f, %f, %f\n", lhp->Rot[0], lhp->Rot[1], lhp->Rot[2], lhp->Rot[3] );

				fprintf( stderr, "====> %f %f %f\n",
					pos[0], pos[1], pos[2] );
					
			}
		}
		fprintf( stderr, "\n" );


		reset_calibration( cd );
//		cd->stage = 5;
	}
	else
	{
		SV_INFO( "Stage 4 failed." );
		reset_calibration( cd );
	}
}




