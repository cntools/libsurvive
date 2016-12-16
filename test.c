#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <survive.h>

#include <os_generic.h>
#include <DrawFunctions.h>

struct SurviveContext * ctx;

void survivefault( struct SurviveContext * ctx, const char * fault )
{
	fprintf( stderr, "Error: %s\n", fault );
	exit( -1 );
}

void survivenote( struct SurviveContext * ctx, const char * fault )
{
	fprintf( stderr, "Info: %s\n", fault );
}


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

extern int bufferpts[32*2];
extern char buffermts[32*128];
extern int buffertimeto[32];

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
		for( i = 0; i < 32; i++ )
		{
			if( buffertimeto[i] < 5 )
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


		CNFGSwapBuffers();
		OGUSleep( 10000 );
	}
}



int main()
{
/*	int i;
	uint8_t input[] = {  0x7a, 0x01, 0x48, 0xc4, 0x1e, 0x1a, 0xfe, 0x6f, 0x6a, 0xf7, 0x25, 0x34 };
//	uint8_t input[] = {  0x1f, 0x8b, 0x08, 0x00, 0xc2, 0x45, 0x43, 0x58, 0x00, 0x03, 0xcb, 0xc8, 0xe4, 0x02, 0x00, 0x7a, 0x7a, 0x6f, 0xed, 0x03, 0x00, 0x00, 0x00 };
//	uint8_t input[] = { 0x78, 0xda, 0xcb, 0xc8, 0x04, 0x00, 0x01, 0x3b, 0x00, 0xd2 };

	uint8_t output[1024];

	int r = survive_simple_inflate( 0, input, sizeof( input ), output, sizeof(output) );

	printf( "%d: ", r );
	for( i = 0 ;i < r; i++ )
	{
		printf( "%02x ", output[i] );
	}
	return 0;*/
	ctx = survive_init( &survivefault, &survivenote );

	CNFGBGColor = 0x000000;
	CNFGDialogColor = 0x444444;
	CNFGSetup( "Survive GUI Debug", 640, 480 );
	OGCreateThread( GuiThread, 0 );
	

	if( !ctx )
	{
		fprintf( stderr, "Fatal. Could not start\n" );
		return 1;
	}

	while(survive_poll(ctx) == 0)
	{
		//Do stuff.
	}
}

