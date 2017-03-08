//Copyright 2016 <>< C. N. Lohr, FULLY Under MIT/x11 License.
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include <survive.h>
#include "survive_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <zlib.h>

static void survivefault( struct SurviveContext * ctx, const char * fault )
{
	fprintf( stderr, "Error: %s\n", fault );
	exit( -1 );
}

static void survivenote( struct SurviveContext * ctx, const char * fault )
{
	fprintf( stderr, "Info: %s\n", fault );
}


struct SurviveContext * survive_init( int headless )
{
	int r = 0;
	int i = 0;
	struct SurviveContext * ctx = calloc( 1, sizeof( struct SurviveContext ) );

	ctx->faultfunction = survivefault;
	ctx->notefunction = survivenote;

	ctx->lightproc = survive_default_light_process;
	ctx->imuproc = survive_default_imu_process;
	ctx->angleproc = survive_default_angle_process;

	const char * DriverName;
	while( ( DriverName = GetDriverNameMatching( "DriverReg", i++ ) ) )

	{
		DeviceDriver dd = GetDriver( DriverName );
		printf( "Loading driver %s (%p) (%d)\n", DriverName, dd, i );
		r = dd( ctx );
		printf( "Driver %s reports status %d\n", DriverName, r );
	}

	return ctx;
}

void survive_install_info_fn( struct SurviveContext * ctx,  text_feedback_func fbp )
{
	if( fbp )
		ctx->notefunction = fbp;
	else
		ctx->notefunction = survivenote;
}

void survive_install_error_fn( struct SurviveContext * ctx,  text_feedback_func fbp )
{
	if( fbp )
		ctx->faultfunction = fbp;
	else
		ctx->faultfunction = survivefault;
}

void survive_install_light_fn( struct SurviveContext * ctx, light_process_func fbp )
{
	if( fbp )
		ctx->lightproc = fbp;
	else
		ctx->lightproc = survive_default_light_process;
}

void survive_install_imu_fn( struct SurviveContext * ctx,  imu_process_func fbp )
{
	if( fbp )
		ctx->imuproc = fbp;
	else
		ctx->imuproc = survive_default_imu_process;
}


void survive_install_angle_fn( struct SurviveContext * ctx,  angle_process_func fbp )
{
	if( fbp )
		ctx->angleproc = fbp;
	else
		ctx->angleproc = survive_default_angle_process;
}

int survive_add_object( struct SurviveContext * ctx, struct SurviveObject * obj )
{
	int oldct = ctx->objs_ct;
	ctx->objs = realloc( ctx->objs, sizeof( struct SurviveObject * ) * (oldct+1) );
	ctx->objs[oldct] = obj;
	ctx->objs_ct = oldct+1;
}

void survive_add_driver( struct SurviveContext * ctx, void * payload, DeviceDriverCb poll, DeviceDriverCb close, DeviceDriverMagicCb magic )
{
	int oldct = ctx->driver_ct;
	ctx->drivers = realloc( ctx->drivers, sizeof( void * ) * (oldct+1) );
	ctx->driverpolls = realloc( ctx->driverpolls, sizeof( DeviceDriverCb * ) * (oldct+1) );
	ctx->drivercloses = realloc( ctx->drivercloses, sizeof( DeviceDriverCb * ) * (oldct+1) );
	ctx->drivermagics = realloc( ctx->drivermagics, sizeof( DeviceDriverMagicCb * ) * (oldct+1) );
	ctx->drivers[oldct] = payload;
	ctx->driverpolls[oldct] = poll;
	ctx->drivercloses[oldct] = close;
	ctx->drivermagics[oldct] = magic;
	ctx->driver_ct = oldct+1;
}

int survive_send_magic( struct SurviveContext * ctx, int magic_code, void * data, int datalen )
{
	int oldct = ctx->driver_ct;
	int i;
	for( i = 0; i < oldct; i++ )
	{
		ctx->drivermagics[i]( ctx, ctx->drivers[i], magic_code, data, datalen );
	}	
}

void survive_close( struct SurviveContext * ctx )
{
	const char * DriverName;
	int r = 0;
	while( ( DriverName = GetDriverNameMatching( "DriverUnreg", r++ ) ) )
	{
		DeviceDriver dd = GetDriver( DriverName );
		SV_INFO( "De-registering driver %s (%p)", DriverName, dd );
		r = dd( ctx );
		SV_INFO( "Driver %s reports status %d", DriverName, r );
	}

	int oldct = ctx->driver_ct;
	int i;
	for( i = 0; i < oldct; i++ )
	{
		ctx->drivercloses[i]( ctx, ctx->drivers[i] );
	}

	free( ctx->objs );
	free( ctx->drivers );
	free( ctx->driverpolls );
	free( ctx->drivermagics );
	free( ctx->drivercloses );
	free( ctx );
}

int survive_poll( struct SurviveContext * ctx )
{
	int oldct = ctx->driver_ct;
	int i, r;

	for( i = 0; i < oldct; i++ )
	{
		r = ctx->driverpolls[i]( ctx, ctx->drivers[i] );
		if( r ) return r;
	}

	return 0;
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
        SV_INFO("survive_simple_inflate could not inflate." );
        return -1;
	}
	int len = zs.total_out;
	inflateEnd( &zs );
	return len;
}

struct SurviveObject * survive_get_so_by_name( struct SurviveContext * ctx, const char * name )
{
	int i;
	for( i = 0; i < ctx->objs_ct; i++ )
	{
		if( strcmp( ctx->objs[i]->codename, name ) == 0 )
			return ctx->objs[i];
	}
	return 0;
}

