#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <survive.h>

void survivefault( struct SurviveContext * ctx, const char * fault )
{
	fprintf( stderr, "Error: %s\n", fault );
	exit( -1 );
}

int main()
{
	struct SurviveContext * ctx = survive_init( &survivefault );

	if( !ctx )
	{
		fprintf( stderr, "Fatal.\n" );
		return 1;
	}

	while(survive_poll(ctx) == 0)
	{
		//Do stuff.
	}
}

