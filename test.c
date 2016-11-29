#include <unistd.h>
#include <stdio.h>


#include <survive.h>

int main()
{
	struct SurviveContext * ctx = survive_init( &ctx );

	if( !ctx )
	{
		fprintf( stderr, "Fatal.\n" );
		return 1;
	}

	while(survive_poll(ctx) == 0)
	{
		
	}
}

