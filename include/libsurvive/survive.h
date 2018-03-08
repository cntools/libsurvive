#ifndef _SURVIVE_H
#define _SURVIVE_H

#include <stdint.h>
#include "survive_types.h"
#include "poser.h"

#ifdef __cplusplus
extern "C" {
#endif

//DANGER: This structure may be redefined.  Note that it is logically split into 64-bit chunks
//for optimization on 32- and 64-bit systems.

struct SurviveObject
{
	SurviveContext * ctx;

	char    codename[4];    //3 letters, null-terminated.  Currently HMD, WM0, WM1.
	char    drivername[4];  //3 letters for driver.  Currently "HTC"
	void    *driver;
	int32_t buttonmask;
	int16_t axis1;

	int16_t axis2;
	int16_t axis3;
	int8_t  charge;
	int8_t  charging:1;
	int8_t  ison:1;
	int8_t  additional_flags:6;

	//Pose Information, also "poser" field.
	FLT    PoseConfidence; //0..1
	SurvivePose OutPose; //Final pose? (some day, one can dream!)
	SurvivePose FromLHPose[NUM_LIGHTHOUSES]; //Filled out by poser, contains computed position from each lighthouse. 
	void * PoserData;   //Initialized to zero, configured by poser, can be anything the poser wants.
	PoserCB PoserFn;

	//Device-specific information about the location of the sensors.  This data will be used by the poser.
	int8_t nr_locations; // sensor count
	FLT * sensor_locations; // size is nr_locations*3.  Contains x,y,z values for each sensor
	FLT * sensor_normals;// size is nrlocations*3.  cointains normal vector for each sensor

	//Timing sensitive data (mostly for disambiguation)
	int32_t timebase_hz;		//48,000,000 for normal vive hardware.  (checked)
	int32_t timecenter_ticks; 	//200,000 for normal vive hardware.     (checked)  (This doubles-up as 2x this = full sweep length)
	int32_t pulsedist_max_ticks; //500,000 for normal vive hardware.   (guessed)
	int32_t pulselength_min_sync; //2,200 for normal vive hardware.    (guessed)
	int32_t pulse_in_clear_time; //35,000 for normal vive hardware.    (guessed)
	int32_t pulse_max_for_sweep; //1,800 for normal vive hardware.     (guessed)
	int32_t pulse_synctime_offset; //20,000 for normal vive hardware.  (guessed)
	int32_t pulse_synctime_slack; //5,000 for normal vive hardware.    (guessed)

	//Flood info, for calculating which laser is currently sweeping.
	void * disambiguator_data;
	int8_t   oldcode;
	int8_t   sync_set_number; //0 = master, 1 = slave, -1 = fault. 
	int8_t   did_handle_ootx; //If unset, will send lightcap data for sync pulses next time a sensor is hit.
	uint32_t last_sync_time[NUM_LIGHTHOUSES];
	uint32_t last_sync_length[NUM_LIGHTHOUSES];
	uint32_t recent_sync_time;

	uint32_t last_lighttime;  //May be a 24- or 32- bit number depending on what device.


	FLT* acc_bias; // size is FLT*3. contains x,y,z
	FLT* acc_scale; // size is FLT*3. contains x,y,z
	FLT* gyro_bias; // size is FLT*3. contains x,y,z
	FLT* gyro_scale; // size is FLT*3. contains x,y,z

	haptic_func haptic;

	//Debug
	int tsl;
};


struct BaseStationData
{
	uint8_t PositionSet:1;

	SurvivePose Pose;

	uint8_t OOTXSet:1;
	uint32_t BaseStationID;
	FLT fcalphase[2];
	FLT fcaltilt[2];
	FLT fcalcurve[2];
	FLT fcalgibpha[2];
	FLT fcalgibmag[2];
};

struct config_group;

#define BUTTON_QUEUE_MAX_LEN 32



// note: buttonId and axisId are 1-indexed values.
// a value of 0 for an id means that no data is present in that value
// additionally, when x and y values are both present in axis data,
// axis1 will be x, axis2 will be y.
typedef struct
{
	uint8_t isPopulated;  //probably can remove this given the semaphore in the parent struct.   helps with debugging
	uint8_t eventType;
	uint8_t buttonId;
	uint8_t axis1Id;
	uint16_t axis1Val;
	uint8_t axis2Id;
	uint16_t axis2Val;
	SurviveObject *so;
} ButtonQueueEntry;

typedef struct
{
	uint8_t nextReadIndex; //init to 0
	uint8_t nextWriteIndex; // init to 0
	void* buttonservicesem;
	ButtonQueueEntry entry[BUTTON_QUEUE_MAX_LEN];
} ButtonQueue;

struct SurviveContext
{
	text_feedback_func faultfunction;
	text_feedback_func notefunction;
	light_process_func lightproc;
	imu_process_func imuproc;
	angle_process_func angleproc;
	button_process_func buttonproc;
	raw_pose_func rawposeproc;

	struct config_group* global_config_values;
	struct config_group* lh_config; //lighthouse configs

	//Calibration data:
	int activeLighthouses;
	BaseStationData bsd[NUM_LIGHTHOUSES];
	SurviveCalData * calptr; //If and only if the calibration subsystem is attached.

	SurviveObject ** objs;
	int objs_ct;

	void ** drivers;
	DeviceDriverCb * driverpolls;
	DeviceDriverCb * drivercloses;
	DeviceDriverMagicCb * drivermagics;
	int driver_ct;

	uint8_t isClosing; // flag to indicate if threads should terminate themselves

	void* buttonservicethread;
	ButtonQueue buttonQueue;

	void *user_ptr;

};

SurviveContext * survive_init_internal( int headless );

// Baked in size of FLT to verify users of the library have the correct setting. 
void survive_verify_FLT_size(uint32_t user_size);
  
static inline SurviveContext * survive_init( int headless ) {
  survive_verify_FLT_size(sizeof(FLT));
  return survive_init_internal( headless );
}

//For any of these, you may pass in 0 for the function pointer to use default behavior.
//In general unless you are doing wacky things like recording or playing back data, you won't need to use this.
void survive_install_info_fn( SurviveContext * ctx,  text_feedback_func fbp );
void survive_install_error_fn( SurviveContext * ctx,  text_feedback_func fbp );
void survive_install_light_fn( SurviveContext * ctx,  light_process_func fbp );
void survive_install_imu_fn( SurviveContext * ctx,  imu_process_func fbp );
void survive_install_angle_fn( SurviveContext * ctx,  angle_process_func fbp );
void survive_install_button_fn(SurviveContext * ctx, button_process_func fbp);
void survive_install_raw_pose_fn(SurviveContext * ctx, raw_pose_func fbp);

void survive_close( SurviveContext * ctx );
int survive_poll( SurviveContext * ctx );

SurviveObject * survive_get_so_by_name( SurviveContext * ctx, const char * name );

//Utilitiy functions.
int survive_simple_inflate( SurviveContext * ctx, const char * input, int inlen, char * output, int outlen );

int survive_send_magic( SurviveContext * ctx, int magic_code, void * data, int datalen );

//Install the calibrator.
void survive_cal_install( SurviveContext * ctx );  //XXX This will be removed if not already done so.

// Read back a human-readable string description of the calibration status
int survive_cal_get_status( struct SurviveContext * ctx, char * description, int description_length );

// Induce haptic feedback
int survive_haptic(SurviveObject * so, uint8_t reserved, uint16_t pulseHigh, uint16_t pulseLow, uint16_t repeatCount);

//Call these from your callback if overridden.  
//Accept higher-level data.
void survive_default_light_process( SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length , uint32_t lh);
void survive_default_imu_process( SurviveObject * so, int mode, FLT * accelgyro, uint32_t timecode, int id );
void survive_default_angle_process( SurviveObject * so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle, uint32_t lh );
void survive_default_button_process(SurviveObject * so, uint8_t eventType, uint8_t buttonId, uint8_t axis1Id, uint16_t axis1Val, uint8_t axis2Id, uint16_t axis2Val);
void survive_default_raw_pose_process(SurviveObject * so, uint8_t lighthouse, FLT *position, FLT *quaternion);


////////////////////// Survive Drivers ////////////////////////////

void   RegisterDriver(const char * name, void * data);

#ifdef _MSC_VER
#define REGISTER_LINKTIME( func ) \
	__pragma(comment(linker,"/export:REGISTER"#func));\
	void REGISTER##func() { RegisterDriver(#func, &func); }
#else
#define REGISTER_LINKTIME( func ) \
	void __attribute__((constructor)) REGISTER##func() { RegisterDriver(#func, &func); }
#endif



///////////////////////// General stuff for writing drivers ///////

//For device drivers to call.  This actually attaches them.
int survive_add_object( SurviveContext * ctx, SurviveObject * obj );
void survive_add_driver( SurviveContext * ctx, void * payload, DeviceDriverCb poll, DeviceDriverCb close, DeviceDriverMagicCb magic );

//For lightcap, etc.  Don't change this structure at all.  Regular vive is dependent on it being exactly as-is.
//When you write drivers, you can use this to send survive lightcap data.
typedef struct
{
	uint8_t sensor_id;
	uint16_t length;
	uint32_t timestamp;
} 
LightcapElement;


//This is the disambiguator function, for taking light timing and figuring out place-in-sweep for a given photodiode.
void handle_lightcap( SurviveObject * so, LightcapElement * le );

#define SV_INFO( ... ) { char stbuff[1024]; sprintf( stbuff, __VA_ARGS__ ); ctx->notefunction( ctx, stbuff ); }
#define SV_ERROR( ... ) { char stbuff[1024]; sprintf( stbuff, __VA_ARGS__ ); ctx->faultfunction( ctx, stbuff ); }
#define SV_KILL()		exit(0)  //XXX This should likely be re-defined.

#ifdef __cplusplus
};
#endif

#endif

