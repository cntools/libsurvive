//Copyright 2016 <>< C. N. Lohr, FULLY Under MIT/x11 License.
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include <survive.h>
#include "survive_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>


struct SurviveContext * survive_init( void(*ff)( struct SurviveContext * ctx, const char * fault ), void(*notefunction)( struct SurviveContext * ctx, const char * note ) )
{
	int r = 0;
	struct SurviveContext * ctx = calloc( 1, sizeof( struct SurviveContext  ) );

	ctx->faultfunction = ff;
	ctx->notefunction = notefunction;

	ctx->headset.sensors = 32;
	ctx->headset.ctx = ctx;
	memcpy( ctx->headset.codename, "HED", 4 );

	ctx->watchman[0].sensors = 16;
	ctx->watchman[0].ctx = ctx;
	memcpy( ctx->watchman[0].codename, "CT0", 4 );

	ctx->watchman[1].sensors = 16;
	ctx->watchman[1].ctx = ctx;
	memcpy( ctx->watchman[1].codename, "CT1", 4 );

	//USB must happen last.
	if( r = survive_usb_init( ctx ) )
	{
		return 0;
	}

	//Next, pull out the config stuff.
/*	char * ct0conf;
	int len = survive_get_config( &ct0conf, ctx, 1, 0 );
	printf( "%d\n", len );
	puts( ct0conf );
*/

	//ctx->headset->photos = malloc( ctx->headset->sensors * sizeof(struct SurvivePhoto) );
	return ctx;
}


void survive_close( struct SurviveContext * ctx )
{
	survive_usb_close( ctx );
}

int survive_poll( struct SurviveContext * ctx )
{
	survive_usb_poll( ctx );
}

int survive_simple_inflate( struct SurviveContext * ctx, const char * input, int inlen, char * output, int outlen )
{
	z_stream zs; //Zlib stream.  May only be used by configuration at beginning and by USB thread periodically.
	memset( &zs, 0, sizeof( zs ) );
	inflateInit( &zs ); ///Consider checking error

	//XXX: Todo: If we find that this is not useful past the beginning (nix this here and move into the configuration getter)
    zs.avail_in = inlen;
    zs.next_in = (z_const Bytef *)input;
    zs.avail_out = outlen;
	zs.next_out = output;

    if( inflate( &zs, Z_FINISH) != Z_STREAM_END )
	{
		printf( "Zissue\n" );
        SV_INFO("survive_simple_inflate could not inflate." );
        return -1;
	}
	int len = zs.total_out;
	inflateEnd( &zs );
	return len;
}

