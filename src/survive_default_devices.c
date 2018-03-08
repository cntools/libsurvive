#include <stdlib.h>
#include <string.h>
#include "survive_default_devices.h"

static SurviveObject* survive_create_device(SurviveContext * ctx,
					    const char* driver_name,
					    void* driver,
					    const char* device_name,
					    haptic_func fn) {
  SurviveObject * device = calloc( 1, sizeof( SurviveObject ) );

  device->ctx = ctx;
  device->driver = driver;
  memcpy( device->codename, device_name, strlen(device_name) );
  memcpy( device->drivername, driver_name, strlen(driver_name) );

  device->timebase_hz = 48000000;
  device->pulsedist_max_ticks = 500000;
  device->pulselength_min_sync = 2200;
  device->pulse_in_clear_time = 35000;
  device->pulse_max_for_sweep = 1800;
  device->pulse_synctime_offset = 20000;
  device->pulse_synctime_slack = 5000;
  device->timecenter_ticks = device->timebase_hz / 240;

  device->haptic = fn;
  
  return device;
}

SurviveObject* survive_create_hmd(SurviveContext * ctx, const char* driver_name, void* driver) {
  return survive_create_device(ctx, driver_name, driver, "HMD", 0);
}

SurviveObject* survive_create_wm0(SurviveContext * ctx, const char* driver_name, void* driver, haptic_func fn) {
  return survive_create_device(ctx, driver_name, driver, "WM0", fn);
}
SurviveObject* survive_create_wm1(SurviveContext * ctx, const char* driver_name, void* driver, haptic_func fn) {
  return survive_create_device(ctx, driver_name, driver, "WM1", fn);
}
SurviveObject* survive_create_tr0(SurviveContext * ctx, const char* driver_name, void* driver) {
  return survive_create_device(ctx, driver_name, driver, "TR0", 0);
}
SurviveObject* survive_create_ww0(SurviveContext * ctx, const char* driver_name, void* driver) {
  return survive_create_device(ctx, driver_name, driver, "WW0", 0);
}
