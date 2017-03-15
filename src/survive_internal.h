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

void * GetDriver( const char * name );
const char * GetDriverNameMatching( const char * prefix, int place );
void   ListDrivers();

#endif


