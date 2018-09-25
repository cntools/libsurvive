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

SURVIVE_EXPORT void * GetDriver( const char * name );
SURVIVE_EXPORT const char * GetDriverNameMatching( const char * prefix, int place );
SURVIVE_EXPORT void *GetDriverWithPrefix(const char *prefix, const char *name);
SURVIVE_EXPORT void   ListDrivers();
SURVIVE_EXPORT void *GetDriverByConfig(SurviveContext *ctx, const char *name, const char *configname, const char *configdef);

void survive_load_plugins(const char *additional_plugin_dir);

#endif


