#ifndef _SURVIVE_DEFAULT_DEVICES_H
#define _SURVIVE_DEFAULT_DEVICES_H

#include <survive.h>

SURVIVE_EXPORT SurviveObject *survive_create_device(SurviveContext *ctx, const char *driver_name, void *driver,
									 const char *device_name, haptic_func fn);

SURVIVE_EXPORT void survive_destroy_device(SurviveObject *so);

SURVIVE_EXPORT SurviveObject *survive_create_hmd(SurviveContext *ctx, const char *driver_name,
								  void *driver);
SURVIVE_EXPORT SurviveObject *survive_create_wm0(SurviveContext *ctx, const char *driver_name,
								  void *driver, haptic_func cb);
SURVIVE_EXPORT SurviveObject *survive_create_wm1(SurviveContext *ctx, const char *driver_name,
								  void *driver, haptic_func cb);
SURVIVE_EXPORT SurviveObject *survive_create_tr0(SurviveContext *ctx, const char *driver_name,
								  void *driver);
SURVIVE_EXPORT SurviveObject *survive_create_tr1(SurviveContext *ctx, const char *driver_name,
								  void *driver);
SURVIVE_EXPORT SurviveObject *survive_create_ww0(SurviveContext *ctx, const char *driver_name,
								  void *driver);

SURVIVE_EXPORT int survive_load_steamvr_lighthousedb(SurviveContext *so, char *ct0conf, int length);
SURVIVE_EXPORT int survive_load_steamvr_lighthousedb_from_file(SurviveContext *ctx, const char *filename);
SURVIVE_EXPORT int survive_load_htc_config_format(SurviveObject *so, char *ct0conf, int length);
SURVIVE_EXPORT int survive_load_htc_config_format_from_file(SurviveObject *so, const char *filename);
#endif
