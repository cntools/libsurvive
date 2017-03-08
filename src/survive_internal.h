//<>< (C) 2016-2017 C. N. Lohr, MOSTLY Under MIT/x11 License.
//

#ifndef _SURVIVE_INTERNAL_H
#define _SURVIVE_INTERNAL_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "survive_driverman.h"
#include <zlib.h>
#include <survive.h>

#define SV_INFO( x... ) { char stbuff[1024]; sprintf( stbuff, x ); ctx->notefunction( ctx, stbuff ); }
#define SV_ERROR( x... ) { char stbuff[1024]; sprintf( stbuff, x ); ctx->faultfunction( ctx, stbuff ); }

//XXX TODO This one needs to be rewritten.
#define SV_KILL()		exit(0)


#endif


