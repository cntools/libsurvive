// (C) 2017 <>< C. N. Lohr, Under MIT/x11 License.
//
// See notice in survive_driverman.h
//

#include "survive_internal.h"
#include <string.h>
#include <stdio.h>

static void * Drivers[MAX_DRIVERS];
static const char * DriverNames[MAX_DRIVERS];
static int NrDrivers;

void   RegisterDriver( const char * element, void * data )
{
	Drivers[NrDrivers] = data;
	DriverNames[NrDrivers] = element;
	NrDrivers++;
}

void * GetDriver( const char * element )
{
	int i;
	for( i = 0; i < NrDrivers; i++ )
	{
		if( strcmp( element, DriverNames[i] ) == 0 ) return Drivers[i];
	}
	return 0;
}

const char * GetDriverNameMatching( const char * prefix, int place )
{
	int i;
	int prefixlen = strlen( prefix );

	for( i = 0; i < NrDrivers; i++ )
	{
		if( memcmp( prefix, DriverNames[i], prefixlen ) == 0 )
			if( 0 == (place--) )
				return DriverNames[i];
	}
	return 0;
}

void   ListDrivers()
{
	int i;
	printf( "Drivers (%d/%d):\n", NrDrivers, MAX_DRIVERS );
	for( i = 0; i < NrDrivers; i++ )
	{
		printf( " %s\n", DriverNames[i] );
	}
}

