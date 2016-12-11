//Copyright 2016 <>< C. N. Lohr, FULLY Under MIT/x11 License.
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include <survive.h>
#include "survive_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <jsmn.h>
#include <string.h>
#include <zlib.h>

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
	if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
			strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
		return 0;
	}
	return -1;
}

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
		//TODO: Cleanup any libUSB stuff sitting around.
		return 0;
	}

#if 1
	//Next, pull out the config stuff.
	{
		char * ct0conf = 0;
		int len = survive_get_config( &ct0conf, ctx, 1, 0 );
		if( len > 0 )
		{
			//From JSMN example.
			jsmn_parser p;
			jsmntok_t t[4096];
			jsmn_init(&p);
			int i;
			int r = jsmn_parse(&p, ct0conf, len, t, sizeof(t)/sizeof(t[0]));	
			if (r < 0) {
				SV_ERROR("Failed to parse JSON in HMD configuration: %d\n", r);
				return 0;
			}
			if (r < 1 || t[0].type != JSMN_OBJECT) {
				SV_ERROR("Object expected in HMD configuration\n");
				return 0;
			}
			for (i = 1; i < r; i++) {
				if (jsoneq(ct0conf, &t[i], "modelPoints") == 0) {
					int k;
					jsmntok_t * tk = &t[i+1];
					//printf( "%d / %d / %d / %d\n", tk->type, tk->start, tk->end, tk->size );
					int pts = tk->size;

					ctx->headset.nr_locations = 0;
					ctx->headset.sensor_locations = malloc( sizeof( *ctx->headset.sensor_locations) * 32 * 3 );

					for( k = 0; k < pts; k++ )
					{
						tk = &t[i+2+k*4];
						//printf( "++%d / %d / %d / %d\n", tk->type, tk->start, tk->end, tk->size );
				
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
								break;
							}

							memcpy( ctt, ct0conf + tk->start, elemlen );
							ctt[elemlen] = 0;
							float f = atof( ctt );
							int id = ctx->headset.nr_locations*3+m;
							ctx->headset.sensor_locations[id] = f;
						}
						ctx->headset.nr_locations++;
					}
				}
			}
		}
		else
		{
			//TODO: Cleanup any remaining USB stuff.
			return 0;
		}

	}
#endif

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

