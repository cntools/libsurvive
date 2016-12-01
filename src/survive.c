#include <survive.h>
#include "survive_internal.h"
#include <stdio.h>
#include <stdlib.h>

struct SurviveContext * survive_init( void(*ff)( struct SurviveContext * ctx, const char * fault ), void(*notefunction)( struct SurviveContext * ctx, const char * note ) )
{
	int r = 0;
	struct SurviveContext * ret = calloc( 1, sizeof( struct SurviveContext  ) );
	ret->faultfunction = ff;
	ret->notefunction = notefunction;
	if( r = survive_usb_init( ret ) )
	{
		return 0;
	}

	ret->headset.sensors = 32;
	ret->headset.ctx = ret;
	//ret->headset->photos = malloc( ret->headset->sensors * sizeof(struct SurvivePhoto) );
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

