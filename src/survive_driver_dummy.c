// All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL
// or LGPL licenses.

#include <stdio.h>
#include <stdlib.h>
#include <survive.h>
#include <string.h>
#include "survive_config.h"
#include "os_generic.h"

struct SurviveDriverDummy {
	SurviveContext * ctx;
	SurviveObject * so;
};
typedef struct SurviveDriverDummy SurviveDriverDummy;

static int dummy_poll(struct SurviveContext *ctx, void *_driver) {
	SurviveDriverDummy *driver = _driver;

/* 
	To emit an IMU event, send this:
		driver->ctx->imuproc(so, mask, accelgyro, timecode, id);

	To emit light data, send this:
		LightcapElement le;
		le.sensor_id = X		//8 bits
		le.length = Z			//16 bits
		le.timestamp = Y		//32 bits
		handle_lightcap(so, &le);
*/

	return 0;
}

static int dummy_close(struct SurviveContext *ctx, void *_driver) {
	SurviveDriverDummy *driver = _driver;

/*
	If you need to handle any cleanup here, like closing handles, etc.
	you can perform it here.
*/

	return 0;
}

int dummy_haptic( SurviveObject * so, uint8_t reserved, uint16_t pulseHigh, uint16_t pulseLow, uint16_t repeatCount )
{
/*
	If your device has haptics, you can add the control for them here.
*/
	return 0;
}

int DriverRegDummy(SurviveContext *ctx)
{
	int enable_dummy_driver = survive_configi( ctx, "dummy_driver_enable", SC_GET, 0 );

	if( !enable_dummy_driver ) return 0;

	SurviveDriverDummy *sp = calloc(1, sizeof(SurviveDriverDummy));
	sp->ctx = ctx;

	SV_INFO("Setting up dummy driver.");

	//Create a new SurviveObject...
	SurviveObject *device = calloc(1, sizeof(SurviveObject));
	device->ctx = ctx;
	device->driver = sp;
	memcpy(device->codename, "DM0", 4);
	memcpy(device->drivername, "DUM", 4);

	device->timebase_hz = 48000000;
	device->pulsedist_max_ticks = 500000;
	device->pulselength_min_sync = 2200;
	device->pulse_in_clear_time = 35000;
	device->pulse_max_for_sweep = 1800;
	device->pulse_synctime_offset = 20000;
	device->pulse_synctime_slack = 5000;
	device->timecenter_ticks = device->timebase_hz / 240;
	device->imu_freq = 1000.0f;
	device->haptic = dummy_haptic;

	sp->so = device;
	survive_add_object( ctx, device );
	survive_add_driver( ctx, sp, dummy_poll, dummy_close, 0 );
	return 0;
}

REGISTER_LINKTIME(DriverRegDummy);

