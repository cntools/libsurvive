//<>< (C) 2016-2017 C. N. Lohr, MOSTLY Under MIT/x11 License.
//

#ifndef _SURVIVE_INTERNAL_H
#define _SURVIVE_INTERNAL_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <survive.h>


//Driver registration
#define MAX_DRIVERS 32

SURVIVE_EXPORT const char *survive_config_file_name(struct SurviveContext *ctx);
SURVIVE_EXPORT const char *survive_config_file_path(struct SurviveContext *ctx, char *path);
SURVIVE_EXPORT survive_driver_fn GetDriver(const char *name);
SURVIVE_EXPORT const char * GetDriverNameMatching( const char * prefix, int place );
SURVIVE_EXPORT survive_driver_fn GetDriverWithPrefix(const char *prefix, const char *name);
SURVIVE_EXPORT void   ListDrivers();
SURVIVE_EXPORT survive_driver_fn GetDriverByConfig(SurviveContext *ctx, const char *name, const char *configname,
												   const char *configdef);

void survive_load_plugins(const char *additional_plugin_dir);
typedef double (*survive_run_time_fn)(const SurviveContext *ctx, void *user);
SURVIVE_EXPORT void survive_install_run_time_fn(SurviveContext *ctx, survive_run_time_fn fn, void *user);

#endif


