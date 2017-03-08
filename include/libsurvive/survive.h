#ifndef _SURVIVE_H
#define _SURVIVE_H

#include <stdint.h>
#include "poser.h"

typedef struct SurviveContext SurviveContext;

//DANGER: This structure may be redefined.  Note that it is logically split into 64-bit chunks
//for optimization on 32- and 64-bit systems.

struct SurviveObject
{
	SurviveContext * ctx;

	char    codename[4];    //3 letters, null-terminated.  Currently HMD, WM0, WM1.
	char    drivername[4];  //3 letters for driver.  Currently "HTC"
	int16_t buttonmask;
	int16_t axis1;

	int16_t axis2;
	int16_t axis3;
	int8_t  charge;
	int8_t  charging:1;
	int8_t  ison:1;
	int8_t  additional_flags:6;

	//Pose Information, also "resolver" field.
	FLT    PoseConfidence; //0..1
	SurvivePose OutPose;
	SurvivePose FromLHPose[NUM_LIGHTHOUSES]; //Optionally filled out by poser, contains computed position from each lighthouse.
	void * PoserData;   //Initialized to zero, configured by poser, can be anything the poser wants.
	PoserCB * PoserFn;

	//Device-specific information about the location of the sensors.  This data will be used by the poser.
	int8_t nr_locations;
	FLT * sensor_locations;
	FLT * sensor_normals;

	//Timing sensitive data (mostly for disambiguation)
	int32_t timebase_hz;		//48,000,000 for normal vive hardware.  (checked)
	int32_t timecenter_ticks; 	//200,000 for normal vive hardware.     (checked)
	int32_t pulsedist_max_ticks; //500,000 for normal vive hardware.   (guessed)
	int32_t pulselength_min_sync; //2,200 for normal vive hardware.    (guessed)
	int32_t pulse_in_clear_time; //35,000 for normal vive hardware.    (guessed)
	int32_t pulse_max_for_sweep; //1,800 for normal vive hardware.     (guessed)
	int32_t pulse_synctime_offset; //20,000 for normal vive hardware.  (guessed)
	int32_t pulse_synctime_slack; //5,000 for normal vive hardware.    (guessed)

	//Flood info, for calculating which laser is currently sweeping.
	int8_t   oldcode;
	int8_t   sync_set_number; //0 = master, 1 = slave, -1 = fault. 
	int8_t   did_handle_ootx; //If unset, will send lightcap data for sync pulses next time a sensor is hit.
	uint32_t last_time[NUM_LIGHTHOUSES];
	uint32_t last_length[NUM_LIGHTHOUSES];
	uint32_t recent_sync_time;

	uint32_t last_lighttime;  //May be a 24- or 32- bit number depending on what device.


	//Debug
	int tsl;
};

struct SurviveContext * survive_init( int headless );

//For any of these, you may pass in 0 for the function pointer to use default behavior.
void survive_install_info_fn( struct SurviveContext * ctx,  text_feedback_func fbp );
void survive_install_error_fn( struct SurviveContext * ctx,  text_feedback_func fbp );
void survive_install_light_fn( struct SurviveContext * ctx,  light_process_func fbp );
void survive_install_imu_fn( struct SurviveContext * ctx,  imu_process_func fbp );
void survive_install_angle_fn( struct SurviveContext * ctx,  angle_process_func fbp );

void survive_close( struct SurviveContext * ctx );
int survive_poll();

struct SurviveObject * survive_get_so_by_name( struct SurviveContext * ctx, const char * name );

//Utilitiy functions.
int survive_simple_inflate( struct SurviveContext * ctx, const char * input, int inlen, char * output, int outlen );

int survive_send_magic( struct SurviveContext * ctx, int magic_code, void * data, int datalen );

//Install the calibrator.
void survive_cal_install( struct SurviveContext * ctx );

//Call these from your callback if overridden.  
//Accept higher-level data.
void survive_default_light_process( struct SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length );
void survive_default_imu_process( struct SurviveObject * so, int16_t * accelgyro, uint32_t timecode, int id );
void survive_default_angle_process( struct SurviveObject * so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle );


#endif

