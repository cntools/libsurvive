//Totally hacky "client" for absorbing data from a network source of the vive data.

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <survive.h>
#include <string.h>
#include <os_generic.h>
#include "src/survive_cal.h"
#include <DrawFunctions.h>

#include "src/survive_config.h"

struct SurviveContext * ctx;

void HandleKey( int keycode, int bDown )
{
	if( !bDown ) return;

	if( keycode == 'O' || keycode == 'o' )
	{
		survive_usb_send_magic(ctx,1);
	}
	if( keycode == 'F' || keycode == 'f' )
	{
		survive_usb_send_magic(ctx,0);
	}
}

void HandleButton( int x, int y, int button, int bDown )
{
}

void HandleMotion( int x, int y, int mask )
{
}

int bufferpts[32*2*3];
char buffermts[32*128*3];
int buffertimeto[32*3];

void my_light_process( struct SurviveObject * so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length  )
{
	survive_default_light_process( so, sensor_id, acode, timeinsweep, timecode, length );

	if( acode == -1 ) return;
//return;
	int jumpoffset = sensor_id;
	if( strcmp( so->codename, "WM0" ) == 0 ) jumpoffset += 32;
	else if( strcmp( so->codename, "WM1" ) == 0 ) jumpoffset += 64;


	if( acode == 0 || acode == 2 ) //data = 0
	{
		bufferpts[jumpoffset*2+0] = (timeinsweep-100000)/500;
		buffertimeto[jumpoffset] = 0;
	}
	if( acode == 1 || acode == 3 ) //data = 1
	{
		bufferpts[jumpoffset*2+1] = (timeinsweep-100000)/500;
		buffertimeto[jumpoffset] = 0;
	}


	if( acode == 4 || acode == 6 ) //data = 0
	{
		bufferpts[jumpoffset*2+0] = (timeinsweep-100000)/500;
		buffertimeto[jumpoffset] = 0;
	}
	if( acode == 5 || acode == 7 ) //data = 1
	{
		bufferpts[jumpoffset*2+1] = (timeinsweep-100000)/500;
		buffertimeto[jumpoffset] = 0;
	}
}

void my_imu_process( struct SurviveObject * so, int16_t * accelgyro, uint32_t timecode, int id )
{
	survive_default_imu_process( so, accelgyro, timecode, id );

return;
	//if( so->codename[0] == 'H' )
	if( 1 )
	{
		printf( "I %s %d %d %d %d %d %d %d %d\n", so->codename, timecode, accelgyro[0], accelgyro[1], accelgyro[2], accelgyro[3], accelgyro[4], accelgyro[5], id );
	}
}


void my_angle_process( struct SurviveObject * so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle )
{
	survive_default_angle_process( so, sensor_id, acode, timecode, length, angle );
}


void * GuiThread( void * v )
{
	short screenx, screeny;
	while(1)
	{
		CNFGHandleInput();
		CNFGClearFrame();
		CNFGColor( 0xFFFFFF );
		CNFGGetDimensions( &screenx, &screeny );

		int i;
		for( i = 0; i < 32*3; i++ )
		{
			if( buffertimeto[i] < 50 )
			{
				uint32_t color = i * 3231349;
				uint8_t r = color & 0xff;
				uint8_t g = (color>>8) & 0xff;
				uint8_t b = (color>>16) & 0xff;
				r = (r * (5-buffertimeto[i])) / 5 ;
				g = (g * (5-buffertimeto[i])) / 5 ;
				b = (b * (5-buffertimeto[i])) / 5 ;
				CNFGColor( (b<<16) | (g<<8) | r );
				CNFGTackRectangle( bufferpts[i*2+0], bufferpts[i*2+1], bufferpts[i*2+0] + 5, bufferpts[i*2+1] + 5 );
				CNFGPenX = bufferpts[i*2+0]; CNFGPenY = bufferpts[i*2+1];
				CNFGDrawText( buffermts, 2 );			
				buffertimeto[i]++;
			}
		}

		CNFGColor( 0xffffff );
		char caldesc[256];
		survive_cal_get_status( ctx, caldesc, sizeof( caldesc ) );
		CNFGPenX = 3;
		CNFGPenY = 3;
		CNFGDrawText( caldesc, 4 );


		CNFGSwapBuffers();
		OGUSleep( 10000 );
	}
}




int main()
{
	ctx = survive_init( 1 );

	survive_install_light_fn( ctx,  my_light_process );
	survive_install_imu_fn( ctx,  my_imu_process );
	survive_install_angle_fn( ctx, my_angle_process );

	survive_cal_install( ctx );

	CNFGBGColor = 0x000000;
	CNFGDialogColor = 0x444444;
	CNFGSetup( "Survive GUI Debug", 640, 480 );
	OGCreateThread( GuiThread, 0 );

	config_init();
	config_set_str(&global_config_values, "Hello","World!");
	const char *s = config_read_str(&global_config_values, "TestStr","This is a test.");
	printf("%s\n", s);
	config_save("config.json");
	

	if( !ctx )
	{
		fprintf( stderr, "Fatal. Could not start\n" );
		return 1;
	}

	while(survive_poll(ctx) == 0)
	{
		char * lineptr;
		size_t n;
		lineptr = 0;
		n = 0;
		ssize_t gl = getline( &lineptr, &n, stdin );
//		printf( "%d %s\n", gl, lineptr );

		switch( lineptr[0] )
		{
			case 'I': 
				//IMU data
				//I WM0 -695533550 -321 1357 -3928 -16 -2 -2 0
				break;
			case 'R': 
			case 'L': 
			{
				//Light data
				//R X HMD -878577652 -1 6 380498 6004
				char lhn[10];
				char beam[10];
				char dev[10];
				int timecode, sensor_id, acode, timeinsweep, length;

				sscanf( lineptr, "%9s %9s %9s %d %d %d %d %d\n", 
					lhn, beam, dev, &timecode, &sensor_id, &acode, &timeinsweep, &length );

				struct SurviveObject * so = 0;
				if( strcmp( dev, "HMD" ) == 0 )
					so = &ctx->headset;
				if( strcmp( dev, "WM0" ) == 0 )
					so = &ctx->watchman[0];
				if( strcmp( dev, "WM1" ) == 0 )
					so = &ctx->watchman[1];

				if( so )
					my_light_process( so, sensor_id, acode, timeinsweep, timecode, length );

				break;
			}
		}

		free( lineptr );

        //printf( "!!!\n" );
		//Do stuff.
	}
}

