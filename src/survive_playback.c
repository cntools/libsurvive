//Unofficial driver for the official Valve/HTC Vive hardware.
//
//Based off of https://github.com/collabora/OSVR-Vive-Libre
// Originally Copyright 2016 Philipp Zabel
// Originally Copyright 2016 Lubosz Sarnecki <lubosz.sarnecki@collabora.co.uk>
// Originally Copyright (C) 2013 Fredrik Hultin
// Originally Copyright (C) 2013 Jakob Bornecrantz
//
//But, re-written as best as I can to get it put under an open souce license instead of a forced-source license.
//If there are portions of the code too similar to the original, I would like to know  so they can be re-written.
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include <survive.h>
#include <jsmn.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <os_generic.h>
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h> // for alloca
#endif
#include <sys/time.h>
#include "json_helpers.h"
#include "survive_config.h"
#include "survive_default_devices.h"

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
 if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
    strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
		return 0;
	}
	return -1;
}


static int ParsePoints( SurviveContext * ctx, SurviveObject * so, char * ct0conf, FLT ** floats_out, jsmntok_t * t, int i )
{
	int k;
	int pts = t[i+1].size;
	jsmntok_t * tk;

	so->nr_locations = 0;
	*floats_out = malloc( sizeof( **floats_out ) * 32 * 3 );

	for( k = 0; k < pts; k++ )
	{
		tk = &t[i+2+k*4];

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
			FLT f = atof( ctt );
			int id = so->nr_locations*3+m;
			(*floats_out)[id] = f;
		}
		so->nr_locations++;
	}
	return 0;
}

struct SurvivePlaybackData {
  SurviveContext * ctx;
  const char* playback_dir; 
  FILE* playback_file;
  int lineno;

  uint64_t next_time_us; 
};
typedef struct SurvivePlaybackData SurvivePlaybackData;


uint64_t timestamp_in_us() {
  static uint64_t start_time_us = 0;
  struct timeval tv;
  gettimeofday(&tv,NULL);
  uint64_t now = (uint64_t)tv.tv_sec * 1000000L + tv.tv_usec;
  if(start_time_us == 0)
    start_time_us = now;
  return now - start_time_us;
}

static int parse_and_run_imu(const char* line, SurvivePlaybackData* driver) {  
  char dev[10];
  int timecode = 0;
  FLT accelgyro[6];
  int mask;
  int id;

  int rr = sscanf(line,"I %s %d %d " FLT_format " " FLT_format " " FLT_format " " FLT_format " " FLT_format " " FLT_format "%d", dev,
		  &mask,
		  &timecode,
		  &accelgyro[0], &accelgyro[1], &accelgyro[2],
		  &accelgyro[3], &accelgyro[4], &accelgyro[5], &id );

  if( rr != 10 )
    {
      fprintf( stderr, "Warning:  On line %d, only %d values read: '%s'\n", driver->lineno, rr, line );
      return -1;
    }

  SurviveObject * so = survive_get_so_by_name( driver->ctx, dev);
  if(!so) {
    fprintf(stderr, "Could not find device named %s from lineno %d\n", dev, driver->lineno);	  
    return -1;
  }
	
  driver->ctx->imuproc( so, mask, accelgyro, timecode, id);
  return 0;
}


static int parse_and_run_lightcode(const char* line, SurvivePlaybackData* driver) {  
  char lhn[10];
  char axn[10];
  char dev[10];
  uint32_t timecode = 0;
  int sensor = 0;
  int acode = 0;
  int timeinsweep = 0;
  uint32_t pulselength = 0;
  uint32_t lh = 0;

  int rr = sscanf(line,"%8s %8s %8s %u %d %d %d %u %u\n",
		  lhn, axn, dev,
		  &timecode, &sensor, &acode,
		  &timeinsweep, &pulselength, &lh );

	if( rr != 9 )
	  {
	    fprintf( stderr, "Warning:  On line %d, only %d values read: '%s'\n", driver->lineno, rr, line );
	    return -1;
	  }

	SurviveObject * so = survive_get_so_by_name( driver->ctx, dev);
	if(!so) {
	  fprintf(stderr, "Could not find device named %s from lineno %d\n", dev, driver->lineno);	  
	  return -1;
	}
	
	driver->ctx->lightproc( so, sensor, acode, timeinsweep, timecode, pulselength, lh);
	return 0;
}

static int playback_poll( struct SurviveContext * ctx, void * _driver ) {
  SurvivePlaybackData* driver = _driver;
  FILE* f = driver->playback_file;

  if(f && !feof(f) && !ferror(f) )
    {
      int i;
      driver->lineno++;
      char  * line;

      if(driver->next_time_us == 0) {
	char * buffer;
	size_t n = 0;      
	ssize_t r = getdelim( &line, &n, ' ', f );
	if( r <= 0 ) return 0;

	uint64_t timestamp;
	if(sscanf(line, "%lu", &driver->next_time_us) != 1) {
	  free(line);
	  return 0;
	}
	free(line);
	line = 0;
      }

      if(driver->next_time_us > timestamp_in_us())
	return 0;
      driver->next_time_us = 0;
      
      char * buffer;
      size_t n = 0;      
      ssize_t r = getline( &line, &n, f );
      if( r <= 0 ) return 0;
      
      if((line[0] != 'R' && line[0] != 'L' && line[0] != 'I') || line[1] != ' ' )
	return 0;

      switch(line[0]) {
      case 'L':
      case 'R':
	parse_and_run_lightcode(line, driver);
	break;
      case 'I':
	parse_and_run_imu(line, driver);
	break;
      }

      free( line );
    } else {
    if(f) {
      fclose(driver->playback_file);
    }
    driver->playback_file = 0;
    return -1;
  }
  
  return 0;
}

int playback_close( struct SurviveContext * ctx, void * _driver ) {
    SurvivePlaybackData* driver = _driver;
    if(driver->playback_file)
      fclose(driver->playback_file);
    driver->playback_file = 0; 
  return 0;
}


static int LoadConfig( SurvivePlaybackData * sv, SurviveObject * so, int devno, int iface, int extra_magic )
{
	SurviveContext * ctx = sv->ctx;
	char * ct0conf = 0;
	
	char fname[100];
	sprintf( fname, "%s/%s_config.json", sv->playback_dir, so->codename );
	FILE * f = fopen( fname, "r" );

	if(f == 0 || feof(f) || ferror(f) )
	  return 1;
	    
	fseek(f, 0, SEEK_END);
	int len = ftell(f);
	fseek(f, 0, SEEK_SET);  //same as rewind(f);

	ct0conf = malloc(len+1);       
	fread( ct0conf, len, 1, f);
	fclose( f );
	ct0conf[len] = 0;

	printf( "Loading config: %d\n", len );

	if (len == 0)
	  return 1;

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


			if (jsoneq(ct0conf, tk, "acc_bias") == 0) {
				int32_t count = (tk+1)->size;
				FLT* values = NULL;
				if ( parse_float_array(ct0conf, tk+2, &values, count) >0 ) {
					so->acc_bias = values;
					so->acc_bias[0] *= .125; //XXX Wat?  Observed by CNL.  Biasing by more than this seems to hose things.
					so->acc_bias[1] *= .125;
					so->acc_bias[2] *= .125;
				}
			}
			if (jsoneq(ct0conf, tk, "acc_scale") == 0) {
				int32_t count = (tk+1)->size;
				FLT* values = NULL;
				if ( parse_float_array(ct0conf, tk+2, &values, count) >0 ) {
					so->acc_scale = values;
				}
			}

			if (jsoneq(ct0conf, tk, "gyro_bias") == 0) {
				int32_t count = (tk+1)->size;
				FLT* values = NULL;
				if ( parse_float_array(ct0conf, tk+2, &values, count) >0 ) {
					so->gyro_bias = values;
				}
			}
			if (jsoneq(ct0conf, tk, "gyro_scale") == 0) {
				int32_t count = (tk+1)->size;
				FLT* values = NULL;
				if ( parse_float_array(ct0conf, tk+2, &values, count) >0 ) {
					so->gyro_scale = values;
				}
			}
		}

	return 0;
}



int DriverRegPlayback( SurviveContext * ctx )
{
  const char* playback_dir = config_read_str(ctx->global_config_values,
					      "PlaybackDir", "");

  if(strlen(playback_dir) == 0) {
  return 0;
}

  SurvivePlaybackData * sp = calloc( 1, sizeof( SurvivePlaybackData ) );
  sp->ctx = ctx;
  sp->playback_dir = playback_dir;
  printf("%s\n", playback_dir);

  char playback_file[100];
  sprintf( playback_file, "%s/events", playback_dir );	
  sp->playback_file = fopen( playback_file, "r"); 
  if(sp->playback_file == 0) {
  fprintf(stderr, "Could not open playback events file %s", playback_file);
  return -1;
}
  SurviveObject * hmd = survive_create_hmd(ctx, "Playback", sp);
  SurviveObject * wm0 = survive_create_wm0(ctx, "Playback", sp, 0);
  SurviveObject * wm1 = survive_create_wm1(ctx, "Playback", sp, 0);
  SurviveObject * tr0 = survive_create_tr0(ctx, "Playback", sp);
  SurviveObject * ww0 = survive_create_ww0(ctx, "Playback", sp);
  
  if( !LoadConfig( sp, hmd, 1, 0, 0 )) { survive_add_object( ctx, hmd ); }
  if( !LoadConfig( sp, wm0, 2, 0, 1 )) { survive_add_object( ctx, wm0 ); }
  if( !LoadConfig( sp, wm1, 3, 0, 1 )) { survive_add_object( ctx, wm1 ); }
  if( !LoadConfig( sp, tr0, 4, 0, 0 )) { survive_add_object( ctx, tr0 ); }
  if( !LoadConfig( sp, ww0, 5, 0, 0 )) { survive_add_object( ctx, ww0 ); }

  
  survive_add_driver(ctx, sp, playback_poll, playback_close, 0); 
  return 0;
 fail_gracefully:
  return -1;
}

REGISTER_LINKTIME( DriverRegPlayback );

