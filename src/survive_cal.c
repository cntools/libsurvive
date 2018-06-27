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
#include "survive_reproject.h"

#include <assert.h>
#include <linmath.h>
#include <math.h>
#include <poser.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>

#include "survive_config.h"

#ifdef WINDOWS
int mkdir(const char *);
#endif


STATIC_CONFIG_ITEM( REQ_TRACK_FOR_CAL, "requiredtrackersforcal", 's', "Which devices will be used, i.e. HMD,WM0,WM1", "" );
STATIC_CONFIG_ITEM( ALLOW_TRACK_FOR_CAL, "allowalltrackersforcal", 'i', "Allow use of additional connected devices for calibration", 0 );
STATIC_CONFIG_ITEM( CONFIG_POSER, "configposer", 's', "Poser used for calibration step", "SBA" );

#define PTS_BEFORE_COMMON 32
#define NEEDED_COMMON_POINTS 10
#define MIN_SENSORS_VISIBLE_PER_LH_FOR_CAL 4

#define NEEDED_TIMES_OF_COMMON 5
#define DRPTS_NEEDED_FOR_AVG ((int)(DRPTS*3/4))


	//at stage 1, is when it branches to stage two or stage 7
	//stage 2 checks for the presence of two watchmen and an HMD visible to both lighthouses.
	//Stage 3 collects a bunch of data for statistical averageing
	//stage 4 does the calculation for the poses (NOT DONE!)
	//Stage 5 = System Calibrate.d


static void handle_calibration( struct SurviveCalData *cd );
static void reset_calibration( struct SurviveCalData * cd );

void ootx_error_clbk_d(ootx_decoder_context *ct, const char *msg) {
	SurviveContext *ctx = (SurviveContext *)(ct->user);
	SV_INFO("%s", msg);
}

void ootx_packet_clbk_d(ootx_decoder_context *ct, ootx_packet* packet)
{
	static uint8_t lighthouses_completed = 0;

	SurviveContext * ctx = (SurviveContext*)(ct->user);
	SurviveCalData * cd = ctx->calptr;
	int id = ct->user1;

	SV_INFO( "Got OOTX packet %d %p", id, cd );

	lighthouse_info_v6 v6;
	init_lighthouse_info_v6(&v6, packet->data);

	BaseStationData * b = &ctx->bsd[id];
	//print_lighthouse_info_v6(&v6);

	b->BaseStationID = v6.id;
	b->fcal.phase[0] = v6.fcal_0_phase;
	b->fcal.phase[1] = v6.fcal_1_phase;
	b->fcal.tilt[0] = (v6.fcal_0_tilt);
	b->fcal.tilt[1] = (v6.fcal_1_tilt); // XXX??? Is this right? See https://github.com/cnlohr/libsurvive/issues/18
	b->fcal.curve[0] = v6.fcal_0_curve;
	b->fcal.curve[1] = v6.fcal_1_curve;
	b->fcal.gibpha[0] = v6.fcal_0_gibphase;
	b->fcal.gibpha[1] = v6.fcal_1_gibphase;
	b->fcal.gibmag[0] = v6.fcal_0_gibmag;
	b->fcal.gibmag[1] = v6.fcal_1_gibmag;
	b->accel[0] = v6.accel_dir_x;
	b->accel[1] = v6.accel_dir_y;
	b->accel[2] = v6.accel_dir_z;
	b->mode = v6.mode_current;
	b->OOTXSet = 1;

	config_set_lighthouse(ctx->lh_config,b,id);
	lighthouses_completed++;

	if (lighthouses_completed >= ctx->activeLighthouses) {
		config_save(ctx, survive_configs(ctx, "configfile", SC_GET, "config.json"));
	}
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
	if (ctx->calptr)
		return;

	int i;
	struct SurviveCalData * cd = ctx->calptr = calloc( 1, sizeof( struct SurviveCalData ) );

	if( ctx->state != SURVIVE_RUNNING )
	{
		SV_ERROR( "Error: You cannot install a calibrator until the system is running." );
	}

	for( i = 0; i < NUM_LIGHTHOUSES; i++ )
	{
		ootx_init_decoder_context(&cd->ootx_decoders[i]);
		cd->ootx_decoders[i].user = ctx;
		cd->ootx_decoders[i].user1 = i;
	}

	cd->stage = 1;
	cd->ctx = ctx;

	cd->numPoseObjects = 0;

	// setting the required trackers for calibration to be permissive to make it easier for a newbie to start-- 
	// basically, libsurvive will detect whatever they have plugged in and start using that.  
//	const char * RequiredTrackersForCal = config_read_str(ctx->global_config_values, "RequiredTrackersForCal", "HMD,WM0,WM1");
	const char *RequiredTrackersForCal = survive_configs(ctx, "requiredtrackersforcal", SC_SETCONFIG, "");
	const uint32_t AllowAllTrackersForCal = survive_configi(ctx, "allowalltrackersforcal", SC_SETCONFIG, 0);
	size_t requiredTrackersFound = 0;

	for (int j=0; j < ctx->objs_ct; j++)
	{
		// Add the tracker if we allow all trackers for calibration, or if it's in the list 
		// of required trackers.
		int isRequiredTracker = strstr(RequiredTrackersForCal, ctx->objs[j]->codename) != NULL;

		if (isRequiredTracker)
		{
			requiredTrackersFound++;
		}

		if (AllowAllTrackersForCal || isRequiredTracker)
		{
			if (MAX_DEVICES_TO_CAL > cd->numPoseObjects)
			{
				cd->poseobjects[j] = ctx->objs[j];
				cd->numPoseObjects++;

				SV_INFO("Calibration is using %s", cd->poseobjects[j]->codename);
			}
			else
			{
				SV_INFO("Calibration is NOT using %s; device count exceeds MAX_DEVICES_TO_CAL", ctx->objs[j]->codename);
			}
		}

	}

	// If we want to mandate that certain devices have been found

	if (strlen(RequiredTrackersForCal) > 0)
	{
		if (requiredTrackersFound != ((strlen(RequiredTrackersForCal) + 1) / 4))
		{
			SV_ERROR( "Error: Did not find all devices required for calibration." );
			free( cd );
			return;
		}
	}

	cd->ConfigPoserFn = GetDriverByConfig(ctx, "Poser", "configposer", "SBA", 0);

	ootx_packet_clbk = ootx_packet_clbk_d;
	ootx_error_clbk = ootx_error_clbk_d;
	ctx->calptr = cd;
}


void survive_cal_light( struct SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length, uint32_t lh)
{
	struct SurviveContext * ctx = so->ctx;
	struct SurviveCalData * cd = ctx->calptr;

	if( !cd ) return;

	switch( cd->stage )
	{
	default:
	case 2: //Taking in angle data. We don't care about light data anymore.
	case 0: //Default, inactive.
		break;

	case 1:
		//Collecting OOTX data.
		if( sensor_id < 0 )
		{
				//fprintf(stderr, "%s\n", so->codename);
			int lhid = -sensor_id-1;
			// Take the OOTX data from the first device.  (if using HMD, WM0, WM1 only, this will be HMD)

			if( lhid < NUM_LIGHTHOUSES && so == cd->poseobjects[0]  ) 
			{
				uint8_t dbit = (acode & 2)>>1;
				ootx_pump_bit( &cd->ootx_decoders[lhid], dbit );
			}
			int i;
			for( i = 0; i < ctx->activeLighthouses; i++ )
				if( ctx->bsd[i].OOTXSet == 0 ) break;
			if( i == ctx->activeLighthouses ) cd->stage = 2;  //TODO: Make this configuratble to allow single lighthouse.
		}
		break;
	case 3: //Look for light sync lengths.
	{
		if( acode >= -2 ) break;
		else if( acode < -4 ) break;
		int lh = (-acode) - 3;

		for (int i=0; i < cd->numPoseObjects; i++)
		{
			if( strcmp( so->codename, cd->poseobjects[i]->codename ) == 0 )
			{
				sensor_id += i*32;
			}
		}

		cd->all_sync_times[sensor_id][lh][cd->all_sync_counts[sensor_id][lh]++] = length;
		break;
	}

	}
	

}

void survive_cal_angle( struct SurviveObject * so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle, uint32_t lh )
{
	struct SurviveContext * ctx = so->ctx;
	struct SurviveCalData * cd = ctx->calptr;

	if( !cd ) return;

	int sensid = sensor_id;

	for (int i=0; i < cd->numPoseObjects; i++)
	{
		if( strcmp( so->codename, cd->poseobjects[i]->codename ) == 0 )
		{
			sensid += i*32;
		}
	}

	if( sensid >= MAX_SENSORS_TO_CAL || sensid < 0 ) return;

	int lighthouse = lh;
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

		int dev = sensid / 32;

		if( ct > cd->peak_counts )
		{
			cd->peak_counts = ct;
		}


		//Determine if there is a sensor on a watchman visible from both lighthouses.
/*		if( sensid >= 32 )
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
		}*/

		if( cd->peak_counts >= PTS_BEFORE_COMMON )
		{
			int min_peaks = PTS_BEFORE_COMMON;
			int i, j, k;
			cd->found_common = 1;
			for( i = 0; i < cd->numPoseObjects; i++ )
			{
				// for( i = 0; i < MAX_SENSORS_TO_CAL/SENSORS_PER_OBJECT; i++ )
				for (j = 0; j < ctx->activeLighthouses; j++) {
					int sensors_visible = 0;
					for (k = 0; k < SENSORS_PER_OBJECT; k++) {
						if (cd->all_counts[k + i * SENSORS_PER_OBJECT][j][0] > NEEDED_COMMON_POINTS &&
							cd->all_counts[k + i * SENSORS_PER_OBJECT][j][1] > NEEDED_COMMON_POINTS)
							sensors_visible++;
					}
					if (sensors_visible < MIN_SENSORS_VISIBLE_PER_LH_FOR_CAL) {
						// printf( "Dev %d, LH %d not enough visible points found.\n", i, j );
						reset_calibration(cd);
						cd->found_common = 0;
						return;
					}
				}
			}
			
			int tfc = cd->times_found_common;
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
			}

/*			SV_INFO( "Stage 2 moving to stage 3. %d %d", cd->peak_counts, cd->found_common );
			reset_calibration( cd );
			cd->stage = 3;
			cd->found_common = 1;*/
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

	memset( cd->all_sync_counts, 0, sizeof( cd->all_sync_counts ) );
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
#ifdef WINDOWS
	mkdir( "calinfo" );
#else
	mkdir( "calinfo", 0755 );
#endif
	int sen, axis, lh;

	//Just to get it out of the way early, we'll calculate the sync-pulse-lengths here.
	FILE * sync_time_info = fopen( "calinfo/synctime.csv", "w" );

	for( sen = 0; sen < MAX_SENSORS_TO_CAL; sen++ )
	for( lh = 0; lh < NUM_LIGHTHOUSES; lh++ )
	{
		int count = cd->all_sync_counts[sen][lh];
		int i;
		double totaltime;

		totaltime = 0;

		if( count < 20 ) continue;
		for( i = 0; i < count; i++ )
		{
			totaltime += cd->all_sync_times[sen][lh][i];
		}
		FLT avg = totaltime/count;

		double stddev = 0.0;
		for( i = 0; i < count; i++ )
		{
			stddev += (cd->all_sync_times[sen][lh][i] - avg)*(cd->all_sync_times[sen][lh][i] - avg);
		}
		stddev /= count;

		fprintf( sync_time_info, "%d %d %f %d %f\n", sen, lh, totaltime/count, count, stddev );
	}

	fclose( sync_time_info );




	FILE * hists = fopen( "calinfo/histograms.csv", "w" );
	FILE * ptinfo = fopen( "calinfo/ptinfo.csv", "w" );
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


			if( Sdiff2 > OUTLIER_ANGLE || Ldiff2 > OUTLIER_LENGTH )
			{
				cd->all_lengths[sen][lh][axis][i] = -1;
			}
			else
			{
				if( Sdiff2 > max_outlier_angle ) max_outlier_angle = Sdiff2;
				if( Ldiff2 > max_outlier_length ) max_outlier_length = Ldiff2;
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
		#define HISTOGRAMBINANG ((3.14159)/400000.0)  //TODO: Tune

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

			int llm = (int)( Sdiff / HISTOGRAMBINANG + (HISTOGRAMSIZE/2.0) );
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

	int obj;

	//Poses of lighthouses relative to objects.
	SurvivePose  objphl[MAX_POSE_OBJECTS][NUM_LIGHTHOUSES];

	FILE * fobjp = fopen( "calinfo/objposes.csv", "w" );

	for( obj = 0; obj < cd->numPoseObjects; obj++ )
	{
		int i, j;
		PoserDataFullScene fsd = {0};
		fsd.hdr.pt = POSERDATA_FULL_SCENE;
		for( j = 0; j < NUM_LIGHTHOUSES; j++ )
		for( i = 0; i < SENSORS_PER_OBJECT; i++ )
		{
			int gotdata = 0;

			int dataindex = (i+obj*32)*(2*NUM_LIGHTHOUSES)+j*2+0;

			if( cd->ctsweeps[dataindex+0] < DRPTS_NEEDED_FOR_AVG ||
				cd->ctsweeps[dataindex+1] < DRPTS_NEEDED_FOR_AVG )
			{
				fsd.lengths[i][j][0] = -1;
				fsd.lengths[i][j][1] = -1;
				continue;
			}
			fsd.lengths[i][j][0] = cd->avglens[dataindex+0];
			fsd.lengths[i][j][1] = cd->avglens[dataindex+1];
			fsd.angles[i][j][0] = cd->avgsweeps[dataindex + 0];
			fsd.angles[i][j][1] = cd->avgsweeps[dataindex + 1];
		}

		int r = cd->ConfigPoserFn( cd->poseobjects[obj], (PoserData*)&fsd );
		if( r )
		{
			SV_INFO( "Failed calibration on dev %d\n", obj );
			reset_calibration( cd );
			cd->stage = 2;
			fclose( fobjp );
			return;
		}

		int compute_reprojection_error = config_read_uint32(
			ctx->global_config_values, "ComputeReprojectError", 0);

		int lh;
		for( lh = 0; lh < NUM_LIGHTHOUSES; lh++ )
		{
			SurvivePose * objfromlh = &cd->poseobjects[obj]->FromLHPose[lh];  //The pose is here
			SurvivePose * lhp = &ctx->bsd[lh].Pose; //Need to somehow put pose here.

			memcpy( &objphl[obj][lh], objfromlh, sizeof( SurvivePose ) );

			fprintf( fobjp, "%f %f %f\n", objfromlh->Pos[0], objfromlh->Pos[1], objfromlh->Pos[2] );
			fprintf( fobjp, "%f %f %f %f\n", objfromlh->Rot[0], objfromlh->Rot[1], objfromlh->Rot[2], objfromlh->Rot[3] );

			if (ctx->bsd[lh].PositionSet) {
				if (compute_reprojection_error) {
					FLT reproj_err = 0;
					size_t cnt = 0;
					SurviveObject *so = cd->poseobjects[obj];
					for (size_t idx = 0; idx < so->sensor_ct; idx++) {
						FLT *lengths = fsd.lengths[idx][lh];
						FLT *pt = fsd.angles[idx][lh];
						if (lengths[0] < 0 || lengths[1] < 0)
							continue;

						cnt++;
						FLT reproj_pt[2];
						survive_reproject(ctx, lh, so->sensor_locations,
										  reproj_pt);

						FLT err = 0;
						for (int dim = 0; dim < 2; dim++) {
							err += (reproj_pt[dim] - pt[dim]) *
								   (reproj_pt[dim] - pt[dim]);
						}
						reproj_err += sqrt(err);
					}

					// This represents the average distance we were off in our
					// reprojection.
					// Different libraries have slightly different variations on
					// this theme,
					// but this one has an intuitive meaning
					reproj_err = (reproj_err / cnt);

					SV_INFO("Reproject error was %.13g for lighthouse %d",
							reproj_err, lh);
				}
			}
		}

	}
	fclose( fobjp );



	SV_INFO( "Stage 4 succeeded." );
	reset_calibration( cd );
	cd->stage = 5;
}




