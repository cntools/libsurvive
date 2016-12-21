//Copyright 2016 <>< C. N. Lohr, FULLY Under MIT/x11 License.
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include <survive.h>
#include "survive_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <jsmn.h>
#include <string.h>
#include <zlib.h>
#include "disambiguator.h"

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
 if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
    strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
		return 0;
	}
	return -1;
}


static void survivefault( struct SurviveContext * ctx, const char * fault )
{
	fprintf( stderr, "Error: %s\n", fault );
	exit( -1 );
}

static void survivenote( struct SurviveContext * ctx, const char * fault )
{
	fprintf( stderr, "Info: %s\n", fault );
}

static int ParsePoints( struct SurviveContext * ctx, struct SurviveObject * so, char * ct0conf, SV_FLOAT ** floats_out, jsmntok_t * t, int i )
{
	int k;
	int pts = t[i+1].size;
	jsmntok_t * tk;

	so->nr_locations = 0;
	*floats_out = malloc( sizeof( **floats_out ) * 32 * 3 );

	for( k = 0; k < pts; k++ )
	{
		tk = &t[i+2+k*4];

		float vals[3];
		int m;
		for( m = 0; m < 3; m++ )
		{
			char ctt[128];

			tk++;
			int elemlen = tk->end - tk->start;

			if( tk->type != 4 || elemlen > sizeof( ctt )-1 )
			{
				SV_ERROR( "Parse error in JSON\n" );
				return 1;
			}

			memcpy( ctt, ct0conf + tk->start, elemlen );
			ctt[elemlen] = 0;
			float f = atof( ctt );
			int id = so->nr_locations*3+m;
			(*floats_out)[id] = f;
		}
		so->nr_locations++;
	}
	return 0;
}

static int LoadConfig( struct SurviveContext * ctx, struct SurviveObject * so, int devno, int iface, int extra_magic )
{
	char * ct0conf = 0;
	int len = survive_get_config( &ct0conf, ctx, devno, iface, extra_magic );

#if 0
	char fname[100];
	sprintf( fname, "%s_config.json", so->codename );
	FILE * f = fopen( fname, "w" );
	fwrite( ct0conf, strlen(ct0conf), 1, f );
	fclose( f );
#endif

	if( len > 0 )
	{

		//From JSMN example.
		jsmn_parser p;
		jsmntok_t t[4096];
		jsmn_init(&p);
		int i;
		int r = jsmn_parse(&p, ct0conf, len, t, sizeof(t)/sizeof(t[0]));	
		if (r < 0) {
			SV_INFO("Failed to parse JSON in HMD configuration: %d\n", r);
			return -1;
		}
		if (r < 1 || t[0].type != JSMN_OBJECT) {
			SV_INFO("Object expected in HMD configuration\n");
			return -2;
		}

		for (i = 1; i < r; i++) {
			jsmntok_t * tk = &t[i];

			char ctxo[100];
			int ilen = tk->end - tk->start;
			if( ilen > 99 ) ilen = 99;
			memcpy(ctxo, ct0conf + tk->start, ilen);
			ctxo[ilen] = 0;

//				printf( "%d / %d / %d / %d %s %d\n", tk->type, tk->start, tk->end, tk->size, ctxo, jsoneq(ct0conf, &t[i], "modelPoints") );
//				printf( "%.*s\n", ilen, ct0conf + tk->start );

			if (jsoneq(ct0conf, tk, "modelPoints") == 0) {
				if( ParsePoints( ctx, so, ct0conf, &so->sensor_locations, t, i  ) )
				{
					break;
				}
			}
			if (jsoneq(ct0conf, tk, "modelNormals") == 0) {
				if( ParsePoints( ctx, so, ct0conf, &so->sensor_normals, t, i  ) )
				{
					break;
				}
			}
		}
	}
	else
	{
		//TODO: Cleanup any remaining USB stuff.
		return 1;
	}
	return 0;
}

struct SurviveContext * survive_init()
{
	int r = 0;
	struct SurviveContext * ctx = calloc( 1, sizeof( struct SurviveContext ) );

	ctx->faultfunction = survivefault;
	ctx->notefunction = survivenote;

	ctx->lightproc = survive_default_light_process;
	ctx->imuproc = survive_default_imu_process;

	ctx->headset.ctx = ctx;
	memcpy( ctx->headset.codename, "HMD", 4 );
	ctx->headset.d = calloc( 1, sizeof( struct disambiguator ) );

	ctx->watchman[0].ctx = ctx;
	memcpy( ctx->watchman[0].codename, "WM0", 4 );
	ctx->watchman[0].d = calloc( 1, sizeof( struct disambiguator ) );

	ctx->watchman[1].ctx = ctx;
	memcpy( ctx->watchman[1].codename, "WM1", 4 );
	ctx->watchman[1].d = calloc( 1, sizeof( struct disambiguator ) );

	//USB must happen last.
	if( r = survive_usb_init( ctx ) )
	{
		//TODO: Cleanup any libUSB stuff sitting around.
		goto fail_gracefully;
	}

	//Next, pull out the config stuff.
	if( LoadConfig( ctx, &ctx->headset, 1, 0, 0 ) ) goto fail_gracefully;
	if( LoadConfig( ctx, &ctx->watchman[0], 2, 0, 1 ) ) { SV_INFO( "Watchman 0 config issue." ); }
	if( LoadConfig( ctx, &ctx->watchman[1], 3, 0, 1 ) ) { SV_INFO( "Watchman 1 config issue." ); }

/*
	int i;
	int locs = ctx->headset.nr_locations;
	printf( "Locs: %d\n", locs );
	if (ctx->headset.sensor_locations )
	{
		printf( "POSITIONS:\n" );
		for( i = 0; i < locs*3; i+=3 )
		{
			printf( "%f %f %f\n", ctx->headset.sensor_locations[i+0], ctx->headset.sensor_locations[i+1], ctx->headset.sensor_locations[i+2] );
		}
	}
	if( ctx->headset.sensor_normals )
	{
		printf( "NORMALS:\n" );
		for( i = 0; i < locs*3; i+=3 )
		{
			printf( "%f %f %f\n", ctx->headset.sensor_normals[i+0], ctx->headset.sensor_normals[i+1], ctx->headset.sensor_normals[i+2] );
		}
	}
*/

	

	return ctx;
fail_gracefully:
	survive_usb_close( ctx );
	free( ctx );
	return 0;
}

void survive_install_info_fn( struct SurviveContext * ctx,  text_feedback_fnptr fbp )
{
	if( fbp )
		ctx->notefunction = fbp;
	else
		ctx->notefunction = survivenote;
}

void survive_install_error_fn( struct SurviveContext * ctx,  text_feedback_fnptr fbp )
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
        SV_INFO("survive_simple_inflate could not inflate." );
        return -1;
	}
	int len = zs.total_out;
	inflateEnd( &zs );
	return len;
}

struct SurviveObject * survive_get_so_by_name( struct SurviveContext * ctx, const char * name )
{
	if( strcmp( name, "HMD" ) == 0 ) return &ctx->headset;
	if( strcmp( name, "WM0" ) == 0 ) return &ctx->watchman[0];
	if( strcmp( name, "WM1" ) == 0 ) return &ctx->watchman[1];
	return 0;
}

