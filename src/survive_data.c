//<>< (C) 2016 C. N. Lohr, MOSTLY Under MIT/x11 License.
//

#include "survive_internal.h"
#include <stdint.h>
#include <string.h>


int32_t decode_acode(uint32_t length, int32_t main_divisor) {
	//+50 adds a small offset and seems to help always get it right. 
	//Check the +50 in the future to see how well this works on a variety of hardware.

	int32_t acode = (length+main_divisor+50)/(main_divisor*2);
	if( acode & 1 ) return -1;

	return (acode>>1) - 6;
}

//This is the disambiguator function, for taking light timing and figuring out place-in-sweep for a given photodiode.
void handle_lightcap( SurviveObject * so, LightcapElement * le )
{
	SurviveContext * ctx = so->ctx;	
	//int32_t deltat = (uint32_t)le->timestamp - (uint32_t)so->last_master_time;

	if( le->sensor_id > SENSORS_PER_OBJECT )
	{
		return;
	}

#if 0
	if( so->codename[0] == 'H' )
	{
		static int lt;
		static int last;
		if( le->length > 1000 )
		{
			int dl = le->timestamp - lt;
			lt = le->timestamp;
			if( dl > 10000 || dl < -10000 )
				printf( "+++%s %3d %5d %9d  ", so->codename, le->sensor_id, le->length, dl );
			if( dl > 100000 ) printf(" \n" );
		}
		last=le->length;
	}
#endif

	so->tsl = le->timestamp;
	if( le->length < 20 ) return;  ///Assuming 20 is an okay value for here.

	//The sync pulse finder is taking Charles's old disambiguator code and mixing it with a more linear
	//version of Julian Picht's disambiguator, available in 488c5e9.  Removed afterwards into this
	//unified driver.
	int ssn = so->sync_set_number; //lighthouse number
	if( ssn < 0 ) ssn = 0;
	int last_sync_time  =  so->last_sync_time  [ssn];
	int last_sync_length = so->last_sync_length[ssn];
	int32_t delta = le->timestamp - last_sync_time;  //Handle time wrapping (be sure to be int32)

	if( delta < -so->pulsedist_max_ticks || delta > so->pulsedist_max_ticks )
	{
		//Reset pulse, etc.
		so->sync_set_number = -1;
		delta = so->pulsedist_max_ticks;
//		return; //if we don't know what lighthouse this is we don't care to do much else
	}


	if( le->length > so->pulselength_min_sync ) //Pulse longer indicates a sync pulse.
	{
		int is_new_pulse = delta > so->pulselength_min_sync /*1500*/ + last_sync_length;



		so->did_handle_ootx = 0;

		if( is_new_pulse )
		{
			int is_master_sync_pulse = delta > so->pulse_in_clear_time /*40000*/; 

			if( is_master_sync_pulse )
			{
				ssn = so->sync_set_number = 0;
				so->last_sync_time[ssn] = le->timestamp;
				so->last_sync_length[ssn] = le->length;
			}
			else if( so->sync_set_number == -1 )
			{
				//Do nothing.
			}
			else
			{
				ssn = ++so->sync_set_number;
				if( so->sync_set_number >= NUM_LIGHTHOUSES )
				{
					SV_INFO( "Warning.  Received an extra, unassociated sync pulse." );
					ssn = so->sync_set_number = -1;
				}
				else
				{
					so->last_sync_time[ssn] = le->timestamp;
					so->last_sync_length[ssn] = le->length;
				}
			}
		}
		else
		{
			//Find the longest pulse.
			if( le->length > last_sync_length )
			{
				if( so->last_sync_time[ssn] > le->timestamp )
				{
					so->last_sync_time[ssn] = le->timestamp;
					so->last_sync_length[ssn] = le->length;
				}
			}
		}

		//Extra tidbit for storing length-of-sync-pulses.
		{
			int32_t main_divisor = so->timebase_hz / 384000; //125 @ 48 MHz.
			int base_station = is_new_pulse;
			printf( "%s %d %d %d\n", so->codename, le->sensor_id, so->sync_set_number, le->length );
			ctx->lightproc( so, le->sensor_id, -3 - so->sync_set_number, 0, le->timestamp, le->length );
		}
	}



	//See if this is a valid actual pulse.
	else if( le->length < so->pulse_max_for_sweep && delta > so->pulse_in_clear_time && ssn >= 0 )
	{
		int32_t dl = so->last_sync_time[0];
		int32_t tpco = so->last_sync_length[0];


#if NUM_LIGHTHOUSES != 2
		#error You are going to have to fix the code around here to allow for something other than two base stations.
#endif

		//Adding length 
		//Long pulse-code from IR flood.
		//Make sure it fits nicely into a divisible-by-500 time.

		int32_t main_divisor = so->timebase_hz / 384000; //125 @ 48 MHz.

		int32_t acode_array[2] =
			{
				decode_acode(so->last_sync_length[0],main_divisor),
				decode_acode(so->last_sync_length[1],main_divisor)
			};

		//XXX: TODO: Capture error count here.
		if( acode_array[0] < 0 ) return;
		if( acode_array[1] < 0 ) return;

		int acode = acode_array[0];

		if( !so->did_handle_ootx )
		{
			int32_t delta1 = so->last_sync_time[0] - so->recent_sync_time;
			int32_t delta2 = so->last_sync_time[1] - so->last_sync_time[0];

			ctx->lightproc( so, -1, acode_array[0], delta1, so->last_sync_time[0], so->last_sync_length[0] );
			ctx->lightproc( so, -2, acode_array[1], delta2, so->last_sync_time[1], so->last_sync_length[1] );

			so->recent_sync_time = so->last_sync_time[1];

			//Throw out everything if our sync pulses look like they're bad.

			int32_t center_1 = so->timecenter_ticks*2 - so->pulse_synctime_offset;
			int32_t center_2 = so->pulse_synctime_offset;
			int32_t slack = so->pulse_synctime_slack;

			if( delta1 < center_1 - slack || delta1 > center_1 + slack )
			{
				//XXX: TODO: Count faults.
				so->sync_set_number = -1;
				return;
			}

			if( delta2 < center_2 - slack || delta2 > center_2 + slack )
			{
				//XXX: TODO: Count faults.
				so->sync_set_number = -1;
				return;
			}

			so->did_handle_ootx = 1;
		}


		if (acode > 3) {
			if( ssn == 0 )
			{
				//SV_INFO( "Warning: got a slave marker but only got a master sync." );
				//This happens too frequently.  Consider further examination.
			}
			dl = so->last_sync_time[1];
			tpco = so->last_sync_length[1];
		}

		int32_t offset_from = le->timestamp - dl + le->length/2;

		//Make sure pulse is in valid window
		if( offset_from < 380000 && offset_from > 70000 )
		{
			ctx->lightproc( so, le->sensor_id, acode, offset_from, le->timestamp, le->length );
		}
	}
	else
	{
		//printf( "FAIL %d   %d - %d = %d\n", le->length, so->last_photo_time, le->timestamp, so->last_photo_time - le->timestamp );
		//Runt pulse, or no sync pulses available.
	}
}


