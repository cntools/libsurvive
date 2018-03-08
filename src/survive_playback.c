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
#include <stdio.h>
#include <stdlib.h>

#include <string.h>
#include <sys/time.h>

#include "survive_config.h"
#include "survive_default_devices.h"

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

static int playback_close( struct SurviveContext * ctx, void * _driver ) {
    SurvivePlaybackData* driver = _driver;
    if(driver->playback_file)
      fclose(driver->playback_file);
    driver->playback_file = 0; 
  return 0;
}


static int LoadConfig( SurvivePlaybackData * sv, SurviveObject * so)
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
	int read = fread( ct0conf, len, 1, f);
	fclose( f );
	ct0conf[len] = 0;

	printf( "Loading config: %d\n", len );
	return survive_load_htc_config_format(ct0conf, len, so);
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
  
  if( !LoadConfig( sp, hmd )) { survive_add_object( ctx, hmd ); }
  if( !LoadConfig( sp, wm0 )) { survive_add_object( ctx, wm0 ); }
  if( !LoadConfig( sp, wm1 )) { survive_add_object( ctx, wm1 ); }
  if( !LoadConfig( sp, tr0 )) { survive_add_object( ctx, tr0 ); }
  if( !LoadConfig( sp, ww0 )) { survive_add_object( ctx, ww0 ); }

  
  survive_add_driver(ctx, sp, playback_poll, playback_close, 0); 
  return 0;
 fail_gracefully:
  return -1;
}

REGISTER_LINKTIME( DriverRegPlayback );

