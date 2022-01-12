// All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL
// or LGPL licenses.

#include "os_generic.h"
#include "survive_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <survive.h>

STATIC_CONFIG_ITEM(DUMMY_DRIVER_ENABLE, "dummy-driver-enable", 'b', "Load a dummy driver for testing.", 0)

struct SurviveDriverDummy {
	SurviveContext *ctx;
	SurviveObject *so;
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

int DriverRegDummy(SurviveContext *ctx) {
	SurviveDriverDummy *sp = SV_CALLOC(sizeof(SurviveDriverDummy));
	sp->ctx = ctx;

	SV_INFO("Setting up dummy driver.");

	// Create a new SurviveObject...
	SurviveObject *device = SV_CALLOC(sizeof(SurviveObject));
	device->ctx = ctx;
	device->driver = sp;
	memcpy(device->codename, "DM0", 4);
	memcpy(device->drivername, "DUM", 4);
	device->sensor_ct = 1;
	device->sensor_locations = SV_MALLOC(sizeof(FLT) * 3);
	device->sensor_normals = SV_MALLOC(sizeof(FLT) * 3);
	device->sensor_locations[0] = 0;
	device->sensor_locations[1] = 0;
	device->sensor_locations[2] = 0;
	device->sensor_normals[0] = 0;
	device->sensor_normals[1] = 0;
	device->sensor_normals[2] = 1;

	device->timebase_hz = 48000000;
	device->imu_freq = 1000.0f;

	sp->so = device;
	survive_add_object(ctx, device);
	survive_add_driver(ctx, sp, dummy_poll, dummy_close);
	return 0;
}

REGISTER_LINKTIME(DriverRegDummy)
