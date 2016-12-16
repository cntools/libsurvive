#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <survive.h>

#include <os_generic.h>
#include <DrawFunctions.h>

struct SurviveContext * ctx;

int main()
{
	int magicon = 0;
	double Start = OGGetAbsoluteTime();

	ctx = survive_init( );

	if( !ctx )
	{
		fprintf( stderr, "Fatal. Could not start\n" );
		return 1;
	}

	while(survive_poll(ctx) == 0)
	{
		double Now = OGGetAbsoluteTime();
		if( Now > (Start+1) && !magicon )
		{
			survive_usb_send_magic(ctx,1);
			magicon = 1;
		}
		//Do stuff.
	}
}

