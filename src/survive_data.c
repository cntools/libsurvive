//<>< (C) 2016 C. N. Lohr, MOSTLY Under MIT/x11 License.
//

#include "survive_internal.h"
#include <stdint.h>
#include <string.h>

int getAcodeFromSyncPulse(int pulseLen)
{
	if (pulseLen < 3125) return 0;
	if (pulseLen < 3625) return 1;
	if (pulseLen < 4125) return 2;
	if (pulseLen < 4625) return 3;
	if (pulseLen < 5125) return 4;
	if (pulseLen < 5625) return 5;
	if (pulseLen < 6125) return 6;
	return 7;
}
void handle_lightcap2_sync(SurviveObject * so, LightcapElement * le )
{
	fprintf(stderr, "%d\n", le->length);

	if (le->timestamp - so->recent_sync_time < 24000)
	{
		// I do believe we are lighthouse B		
		so->last_sync_time[1] = so->recent_sync_time = le->timestamp;
		so->last_sync_length[1] = le->length;
		if (le->length < 4625) // max non-skip pulse + 1/4 the difference in time between pulses.  
		{
			so->sync_set_number = 1;
			so->ctx->lightproc(so, -2, getAcodeFromSyncPulse(le->length), le->length, le->timestamp, le->length); // Don't think I got this quite right.
		}
	}
	else
	{
		// looks like this is the first sync pulse.  Cool!
		if (le->length < 4625) // max non-skip pulse + 1/4 the difference in time between pulses.  
		{
			so->sync_set_number = 1;
			so->ctx->lightproc(so, -1, getAcodeFromSyncPulse(le->length), le->length, le->timestamp, le->length); // Don't think I got this quite right.
		}		
	}


//			ctx->lightproc( so, -2, acode_array[1], delta2, so->last_sync_time[1], so->last_sync_length[1] );


//	typedef void (*light_process_func)( SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length );

}

void handle_lightcap2_sweep(SurviveObject * so, LightcapElement * le )
{

}

void handle_lightcap2( SurviveObject * so, LightcapElement * le )
{
	SurviveContext * ctx = so->ctx;

	if( le->sensor_id > SENSORS_PER_OBJECT )
	{
		return;
	}

	if (le->length > 6750)
	{
		// Should never get a reading so high.  Odd.
		return;
	}
	if (le->length >= 2750)
	{
		// Looks like a sync pulse, process it!
		handle_lightcap2_sync(so, le);
	}

	// must be a sweep pulse, process it!
	handle_lightcap2_sweep(so, le);

}


//This is the disambiguator function, for taking light timing and figuring out place-in-sweep for a given photodiode.
void handle_lightcap( SurviveObject * so, LightcapElement * le )
{
	handle_lightcap2(so,le);
	return;

	SurviveContext * ctx = so->ctx;
	//int32_t deltat = (uint32_t)le->timestamp - (uint32_t)so->last_master_time;

	//if( so->codename[0] != 'H' )


	if( le->sensor_id > SENSORS_PER_OBJECT )
	{
		return;
	}

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

		//printf("m sync %d %d %d %d\n", le->sensor_id, so->last_sync_time[ssn], le->timestamp, delta);

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
				(so->last_sync_length[0]+main_divisor+50)/(main_divisor*2),  //+50 adds a small offset and seems to help always get it right. 
				(so->last_sync_length[1]+main_divisor+50)/(main_divisor*2),	//Check the +50 in the future to see how well this works on a variety of hardware.
			};

		//XXX: TODO: Capture error count here.
		if( acode_array[0] & 1 ) return;
		if( acode_array[1] & 1 ) return;

		acode_array[0] = (acode_array[0]>>1) - 6;
		acode_array[1] = (acode_array[1]>>1) - 6;


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


