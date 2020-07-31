//<>< (C) 2016 C. N. Lohr, MOSTLY Under MIT/x11 License.
//

#include "survive_internal.h"
#include <math.h> /* for sqrt */
#include <stdint.h>
#include <string.h>

static const float tau_table[33] = {0,
									0,
									0,
									1.151140982,
									1.425,
									1.5712213707,
									1.656266074,
									1.7110275587,
									1.7490784054,
									1.7770229476,
									1.798410005,
									1.8153056661,
									1.8289916275,
									1.8403044103,
									1.8498129961,
									1.8579178211,
									1.864908883,
									1.8710013691,
									1.8763583296,
									1.881105575,
									1.885341741,
									1.8891452542,
									1.8925792599,
									1.8956951735,
									1.8985352854,
									1.9011347009,
									1.9035228046,
									1.9057243816,
									1.9077604832,
									1.9096491058,
									1.9114057255,
									1.9130437248,
									1.914574735};

typedef struct {
	unsigned int sweep_time[SENSORS_PER_OBJECT];
	uint16_t sweep_len[SENSORS_PER_OBJECT]; // might want to align this to cache lines, will be hot for frequent access
} lightcaps_sweep_data;

typedef struct {
	int recent_sync_time;
	int activeLighthouse;
	int activeSweepStartTime;
	int activeAcode;

	//	int lh_pulse_len[NUM_GEN1_LIGHTHOUSES];
	int lh_start_time[NUM_GEN1_LIGHTHOUSES];
	int lh_max_pulse_length[NUM_GEN1_LIGHTHOUSES];
	int8_t lh_acode[NUM_GEN1_LIGHTHOUSES];
	int current_lh; // used knowing which sync pulse we're looking at.

} lightcap2_per_sweep_data;

typedef struct {
	double acode_offset;
	int sent_out_ootx_bits;
} lightcap2_global_data;

typedef struct {
	lightcaps_sweep_data sweep;
	lightcap2_per_sweep_data per_sweep;
	lightcap2_global_data global;
} lightcap2_data;

// static lightcap2_global_data lcgd = { 0 };

static int handle_lightcap2_getAcodeFromSyncPulse(SurviveObject *so, int pulseLen) {
	double oldOffset = ((lightcap2_data *)so->disambiguator_data)->global.acode_offset;

	int modifiedPulseLen = pulseLen - (int)oldOffset;

	double newOffset = (((pulseLen) + 250) % 500) - 250;

	((lightcap2_data *)so->disambiguator_data)->global.acode_offset = oldOffset * 0.9 + newOffset * 0.1;

// fprintf(stderr, "    %f\n", oldOffset);
#define ACODE_OFFSET 0
	if (pulseLen < 3250 - ACODE_OFFSET)
		return 0;
	if (pulseLen < 3750 - ACODE_OFFSET)
		return 1;
	if (pulseLen < 4250 - ACODE_OFFSET)
		return 2;
	if (pulseLen < 4750 - ACODE_OFFSET)
		return 3;
	if (pulseLen < 5250 - ACODE_OFFSET)
		return 4;
	if (pulseLen < 5750 - ACODE_OFFSET)
		return 5;
	if (pulseLen < 6250 - ACODE_OFFSET)
		return 6;
	return 7;
}

static uint8_t remove_outliers(SurviveObject *so) {
	return 0; // disabling this for now because it seems remove almost all the points for wired watchman and wired
			  // tracker.
	lightcap2_data *lcd = so->disambiguator_data;

	uint32_t sum = 0;
	uint8_t non_zero_count = 0;
	uint32_t mean = 0;

	uint16_t *min = NULL;
	uint16_t *max = NULL;
	uint8_t found_first = 0;

	// future: https://gcc.gnu.org/projects/tree-ssa/vectorization.html#vectorizab

	for (uint8_t i = 0; i < SENSORS_PER_OBJECT; i++) {
		sum += lcd->sweep.sweep_len[i];
		if (lcd->sweep.sweep_len[i] > 0)
			++non_zero_count;
	}

	if (non_zero_count == 0)
		return 0;

	mean = sum / non_zero_count;

	float standard_deviation = 0.0f;
	sum = 0;
	for (uint8_t i = 0; i < SENSORS_PER_OBJECT; i++) {
		uint16_t len = lcd->sweep.sweep_len[i];
		if (len > 0) {
			sum += (len - mean) * (len - mean);

			if (found_first == 0) {
				max = min = lcd->sweep.sweep_len + i;
				found_first = 1;
			} else {
				if (lcd->sweep.sweep_len[i] < *min)
					min = lcd->sweep.sweep_len + i;
				if (lcd->sweep.sweep_len[i] > *max)
					max = lcd->sweep.sweep_len + i;
			}
		}
	}
	standard_deviation = sqrtf(((float)sum) / ((float)non_zero_count));

	//	printf("%f\n", standard_deviation);

	float tau_test = standard_deviation;

	if (non_zero_count > 2)
		tau_test = standard_deviation * tau_table[non_zero_count];

	//	uint8_t removed_outliers = 0;

	uint32_t d1 = mean - *min;
	uint32_t d2 = *max - mean;

	if (d1 > d2) {
		if (d1 > tau_test) {
			*min = 0;
			return 1;
		}
	} else if (d2 > tau_test) {
		*max = 0;
		return 1;
	}

	return 0;
	/*
	  for (uint8_t i = 0; i < SENSORS_PER_OBJECT; i++)
	  {
	  uint16_t len = lcd->sweep.sweep_len[i];
	  if (len == 0) continue;

	  if ( abs(len-mean) > tau_test )
	  {
	  //			fprintf(stderr, "removing %d\n", len);
	  lcd->sweep.sweep_len[i] = 0;
	  removed_outliers = 1;
	  }
	  }
	*/
	//	return removed_outliers;
}

static void handle_lightcap2_process_sweep_data(SurviveObject *so) {
	lightcap2_data *lcd = so->disambiguator_data;

	while (remove_outliers(so))
		;

	// look at all of the sensors we found, and process the ones that were hit.
	// TODO: find the sensor(s) with the longest pulse length, and assume
	// those are the "highest quality".  Then, reject any pulses that are sufficiently
	// different from those values, assuming that they are reflections.
	{
		unsigned int longest_pulse = 0;
		unsigned int timestamp_of_longest_pulse = 0;
		(void)timestamp_of_longest_pulse;
		for (int i = 0; i < SENSORS_PER_OBJECT; i++) {
			if (lcd->sweep.sweep_len[i] > longest_pulse) {
				longest_pulse = lcd->sweep.sweep_len[i];
				timestamp_of_longest_pulse = lcd->sweep.sweep_time[i];
			}
		}

		int allZero = 1;
		for (int q = 0; q < 32; q++)
			if (lcd->sweep.sweep_len[q] != 0)
				allZero = 0;
		// if (!allZero)
		//	printf("a[%d]l[%d] ", lcd->per_sweep.activeAcode & 5,  lcd->per_sweep.activeLighthouse);
		for (int i = 0; i < SENSORS_PER_OBJECT; i++) {

			{
				static int counts[SENSORS_PER_OBJECT][2] = {0};
				(void)counts;

				//				if (lcd->per_sweep.activeLighthouse == 0 && !allZero)
				if (lcd->per_sweep.activeLighthouse > -1 && !allZero) {
					if (lcd->sweep.sweep_len[i] != 0) {
						// printf("%d  ", i);
						// counts[i][lcd->per_sweep.activeAcode & 1] ++;
					} else {
						counts[i][lcd->per_sweep.activeAcode & 1] = 0;
					}

					// if (counts[i][0] > 10 && counts[i][1] > 10)
					//{
					// printf("%d(%d,%d), ", i, counts[i][0], counts[i][1]);
					//}
				}
			}

			if (lcd->sweep.sweep_len[i] != 0) // if the sensor was hit, process it
			{
				// printf("%4d\n", lcd->sweep.sweep_len[i]);
				int offset_from =
					lcd->sweep.sweep_time[i] - lcd->per_sweep.activeSweepStartTime + lcd->sweep.sweep_len[i] / 2;

				// first, send out the sync pulse data for the last round (for OOTX decoding
				if (!lcd->global.sent_out_ootx_bits) {
					if (lcd->per_sweep.lh_max_pulse_length[0] != 0) {
						so->ctx->lightproc(
							so, -1, handle_lightcap2_getAcodeFromSyncPulse(so, lcd->per_sweep.lh_max_pulse_length[0]),
							lcd->per_sweep.lh_max_pulse_length[0], lcd->per_sweep.lh_start_time[0], 0, 0);
					}
					if (lcd->per_sweep.lh_max_pulse_length[1] != 0) {
						so->ctx->lightproc(
							so, -2, handle_lightcap2_getAcodeFromSyncPulse(so, lcd->per_sweep.lh_max_pulse_length[1]),
							lcd->per_sweep.lh_max_pulse_length[1], lcd->per_sweep.lh_start_time[1], 0, 1);
					}
					lcd->global.sent_out_ootx_bits = 1;
				}

				//				if (offset_from < 380000 && offset_from > 70000)
				{
					// if (longest_pulse *10 / 8 < lcd->sweep.sweep_len[i])
					{
						so->ctx->lightproc(so, i, lcd->per_sweep.activeAcode, offset_from, lcd->sweep.sweep_time[i],
										   lcd->sweep.sweep_len[i], lcd->per_sweep.activeLighthouse);
					}
				}
			}
		}
		// if (!allZero)
		//	printf("   ..:..\n");

		//		if (!allZero) printf("\n");
	}

	// clear out sweep data (could probably limit this to only after a "first" sync.
	// this is slightly more robust, so doing it here for now.
	memset(&(((lightcap2_data *)so->disambiguator_data)->sweep), 0, sizeof(lightcaps_sweep_data));
}

static void handle_lightcap2_sync(SurviveObject *so, LightcapElement *le) {
	// fprintf(stderr, "%6.6d %4.4d \n", le->timestamp - so->recent_sync_time, le->length);
	lightcap2_data *lcd = so->disambiguator_data;

	// static unsigned int recent_sync_time = 0;
	// static unsigned int recent_sync_count = -1;
	// static unsigned int activeSweepStartTime;
	int acode = handle_lightcap2_getAcodeFromSyncPulse(so, le->length); // acode for this sensor reading

	// Process any sweep data we have
	handle_lightcap2_process_sweep_data(so);

	int time_since_last_sync = (le->timestamp - lcd->per_sweep.recent_sync_time);

	//	fprintf(stderr, "            %2d %8d %d\n", le->sensor_id, time_since_last_sync, le->length);
	// need to store up sync pulses, so we can take the earliest starting time for all sensors.
	if (time_since_last_sync < 2400) {
		lcd->per_sweep.recent_sync_time = le->timestamp;
		// it's the same sync pulse;
		//		so->sync_set_number = 1;
		so->recent_sync_time = le->timestamp;

		//		lcd->per_sweep.lh_pulse_len[lcd->per_sweep.current_lh] = le->length;
		//		lcd->per_sweep.lh_start_time[lcd->per_sweep.current_lh] = le->timestamp;

		if (le->length > lcd->per_sweep.lh_max_pulse_length[lcd->per_sweep.current_lh]) {
			lcd->per_sweep.lh_max_pulse_length[lcd->per_sweep.current_lh] = le->length;
			lcd->per_sweep.lh_start_time[lcd->per_sweep.current_lh] = le->timestamp;
			lcd->per_sweep.lh_acode[lcd->per_sweep.current_lh] = acode;
		}

		/*
		//this stuff should probably be happening on the sweep so that we can filter out erroneous a codes
		if (!(acode >> 2 & 1)) // if the skip bit is not set
		{
		lcd->per_sweep.activeLighthouse = lcd->per_sweep.current_lh;
		lcd->per_sweep.activeSweepStartTime = le->timestamp;
		lcd->per_sweep.activeAcode = acode;
		}
		else
		{
		//this causes the master lighthouse to be ignored from the HMD
		lcd->per_sweep.activeLighthouse = -1;
		lcd->per_sweep.activeSweepStartTime = 0;
		lcd->per_sweep.activeAcode = 0;
		}
		*/
	} else if (time_since_last_sync < 24000) {
		lcd->per_sweep.activeLighthouse = -1;

		lcd->per_sweep.recent_sync_time = le->timestamp;
		// I do believe we are lighthouse B
		lcd->per_sweep.current_lh = 1;
		//		lcd->per_sweep.lh_pulse_len[lcd->per_sweep.current_lh] = le->length;
		lcd->per_sweep.lh_start_time[lcd->per_sweep.current_lh] = le->timestamp;
		lcd->per_sweep.lh_max_pulse_length[lcd->per_sweep.current_lh] = le->length;
		lcd->per_sweep.lh_acode[lcd->per_sweep.current_lh] = acode;

		/*
	  if (!(acode >> 2 & 1)) // if the skip bit is not set
	  {
	  if (lcd->per_sweep.activeLighthouse != -1)
	  {
	  static int pulseWarningCount=0;

	  if (pulseWarningCount < 5)
	  {
	  pulseWarningCount++;
	  // hmm, it appears we got two non-skip pulses at the same time.  That should never happen
	  fprintf(stderr, "WARNING: Two non-skip pulses received on the same cycle!\n");
	  }
	  }

	  lcd->per_sweep.activeLighthouse = 1;
	  lcd->per_sweep.activeSweepStartTime = le->timestamp;
	  lcd->per_sweep.activeAcode = acode;
	  }
		*/

	} else if (time_since_last_sync > 370000) {
		// XXX CAUTION: if we lose sight of a lighthouse then, the remaining lighthouse will default to master
		// this should probably be fixed. Maybe some kind of timing based guess at which lighthouse.

		// looks like this is the first sync pulse.  Cool!
		lcd->global.sent_out_ootx_bits = 0;

		// fprintf(stderr, "************************************ Reinitializing Disambiguator!!!\n");
		// initialize here.
		memset(&lcd->per_sweep, 0, sizeof(lcd->per_sweep));
		lcd->per_sweep.activeLighthouse = -1;

		for (uint8_t i = 0; i < NUM_GEN1_LIGHTHOUSES; ++i) {
			lcd->per_sweep.lh_acode[i] = -1;
		}

		lcd->per_sweep.recent_sync_time = le->timestamp;
		// I do believe we are lighthouse A
		lcd->per_sweep.current_lh = 0;
		//		lcd->per_sweep.lh_pulse_len[lcd->per_sweep.current_lh] = le->length;
		lcd->per_sweep.lh_start_time[lcd->per_sweep.current_lh] = le->timestamp;
		lcd->per_sweep.lh_max_pulse_length[lcd->per_sweep.current_lh] = le->length;
		lcd->per_sweep.lh_acode[lcd->per_sweep.current_lh] = acode;

		//		int acode = handle_lightcap2_getAcodeFromSyncPulse(so, le->length);

		/*
	  if (!(acode >> 2 & 1)) // if the skip bit is not set
	  {
	  lcd->per_sweep.activeLighthouse = 0;
	  lcd->per_sweep.activeSweepStartTime = le->timestamp;
	  lcd->per_sweep.activeAcode = acode;
	  }
		*/
	}
	//	printf("%d %d\n", acode, lcd->per_sweep.activeLighthouse );
}

static void handle_lightcap2_sweep(SurviveObject *so, LightcapElement *le) {
	lightcap2_data *lcd = so->disambiguator_data;

	// If we see multiple "hits" on the sweep for a given sensor,
	// assume that the longest (i.e. strongest signal) is most likely
	// the non-reflected signal.

	// if (le->length < 80)
	//{
	//	// this is a low-quality read.  Better to throw it out than to use it.
	//	//fprintf(stderr, "%2d %d\n", le->sensor_id, le->length);
	//	return;
	//}
	// fprintf(stderr, "%2d %d\n", le->sensor_id, le->length);
	// fprintf(stderr, ".");

	lcd->per_sweep.activeLighthouse = -1;
	lcd->per_sweep.activeSweepStartTime = 0;
	lcd->per_sweep.activeAcode = 0;

	for (uint8_t i = 0; i < NUM_GEN1_LIGHTHOUSES; ++i) {
		int acode = lcd->per_sweep.lh_acode[i];
		if ((acode >= 0) && !(acode >> 2 & 1)) {
			lcd->per_sweep.activeLighthouse = i;
			lcd->per_sweep.activeSweepStartTime = lcd->per_sweep.lh_start_time[i];
			lcd->per_sweep.activeAcode = acode;
		}
	}

	if (lcd->per_sweep.activeLighthouse < 0) {
		// fprintf(stderr, "WARNING: No active lighthouse!\n");
		// fprintf(stderr, "            %2d %8d %d %d\n", le->sensor_id,
		// le->length,lcd->per_sweep.lh_acode[0],lcd->per_sweep.lh_acode[1]);
		return;
	}

	if (lcd->sweep.sweep_len[le->sensor_id] < le->length) {
		lcd->sweep.sweep_len[le->sensor_id] = le->length;
		lcd->sweep.sweep_time[le->sensor_id] = le->timestamp;
	}
}

void DisambiguatorTurvey(SurviveObject *so, LightcapElement *le) {
	SurviveContext *ctx = so->ctx;

	if (so->disambiguator_data == NULL) {
		fprintf(stderr, "Initializing Disambiguator Data\n");
		so->disambiguator_data = SV_MALLOC(sizeof(lightcap2_data));
		memset(so->disambiguator_data, 0, sizeof(lightcap2_data));
	}

	if (le->sensor_id > SENSORS_PER_OBJECT) {
		return;
	}

	if (le->length > 6750) {
		// Should never get a reading so high.  Odd.
		return;
	}
	//	if (le->length >= 2750)
	if (le->length >= 2500) {
		// Looks like a sync pulse, process it!
		handle_lightcap2_sync(so, le);
		return;
	}

	// must be a sweep pulse, process it!
	handle_lightcap2_sweep(so, le);
}

REGISTER_LINKTIME(DisambiguatorTurvey)
