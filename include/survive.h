#ifndef _SURVIVE_H
#define _SURVIVE_H

#include <stdint.h>

#define SV_FLOAT  		double

struct SurviveContext;

//DANGER: This structure may be redefined.  Note that it is logically split into 64-bit chunks
//for optimization on 32- and 64-bit systems.

struct SurviveObject
{
	struct SurviveContext * ctx;

	char    codename[4];  //3 letters, null-terminated.  Currently HMD, WM0, WM1.
	int16_t buttonmask;
	int16_t axis1;

	int16_t axis2;
	int16_t axis3;
	int8_t  charge;
	int8_t  charging:1;
	int8_t  ison:1;
	int8_t  additional_flags:6;

	SV_FLOAT * sensor_locations;
	SV_FLOAT * sensor_normals;
	int8_t nr_locations;

	//Flood info, for calculating which laser is currently sweeping.
	int8_t oldcode;

 #ifdef USE_OLD_DISAMBIGUATOR
	uint32_t last_master_time;
	uint32_t last_slave_time;
 	int16_t last_master_length;
 	int16_t last_slave_length;
	int8_t   is_on_slave;
 #else
	uint32_t last_master_time;
 	struct disambiguator * d;
 #endif

	uint32_t last_lighttime;  //May be a 24- or 32- bit number depending on what device.

	//Debug
	int tsl;
};

typedef void (*text_feedback_fnptr)( struct SurviveContext * ctx, const char * fault );
typedef void (*light_process_func)( struct SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length );
typedef void (*imu_process_func)( struct SurviveObject * so, int16_t * accelgyro, uint32_t timecode, int id );

struct SurviveContext * survive_init();

//For any of these, you may pass in 0 for the function pointer to use default behavior.
void survive_install_info_fn( struct SurviveContext * ctx,  text_feedback_fnptr fbp );
void survive_install_error_fn( struct SurviveContext * ctx,  text_feedback_fnptr fbp );
void survive_install_light_fn( struct SurviveContext * ctx,  light_process_func fbp );
void survive_install_imu_fn( struct SurviveContext * ctx,  imu_process_func fbp );

void survive_close( struct SurviveContext * ctx );
int survive_poll();

struct SurviveObject * survive_get_so_by_name( struct SurviveContext * ctx, const char * name );

//Utilitiy functions.
int survive_simple_inflate( struct SurviveContext * ctx, const char * input, int inlen, char * output, int outlen );

//TODO: Need to make this do haptic responses for hands.
int survive_usb_send_magic( struct SurviveContext * ctx, int on );

#endif

