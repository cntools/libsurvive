#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <survive.h>

void survivefault( struct SurviveContext * ctx, const char * fault )
{
	fprintf( stderr, "Error: %s\n", fault );
	exit( -1 );
}

void survivenote( struct SurviveContext * ctx, const char * fault )
{
	fprintf( stderr, "Info: %s\n", fault );
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
	struct SurviveContext * ctx = survive_init( &survivefault, &survivenote );


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

