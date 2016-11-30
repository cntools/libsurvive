#include <survive.h>
#include "survive_internal.h"
#include <stdio.h>
#include <stdlib.h>

struct SurviveContext * survive_init( void(*ff)( struct SurviveContext * ctx, const char * fault ) )
{
	int r = 0;
	struct SurviveContext * ret = calloc( 1, sizeof( struct SurviveContext  ) );
	ret->faultfunction = ff;
	if( r = survive_usb_init( ret ) )
	{
		return 0;
	}

	return ret;
}


void survive_close( struct SurviveContext * ctx )
{
	survive_usb_close( ctx );
}

int survive_poll( struct SurviveContext * ctx )
{
	survive_usb_poll( ctx );
}

