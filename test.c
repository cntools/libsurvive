#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <survive.h>

#include <os_generic.h>
#include <CNFGFunctions.h>

struct SurviveContext * ctx;



static void dump_iface( struct SurviveObject * so, const char * prefix )
{
	int i;
	FILE * f;
	char fname[1024];

	if (!so) { return; }

	sprintf( fname, "%s_points.csv", prefix );
	f = fopen( fname, "w" );
	for( i = 0; i < so->nr_locations; i++ )
	{
		fprintf( f, "%g %g %g\n", so->sensor_locations[i*3+0], so->sensor_locations[i*3+1], so->sensor_locations[i*3+2] );
	}
	fclose( f );

	sprintf( fname, "%s_normals.csv", prefix );
	f = fopen( fname, "w" );
	for( i = 0; i < so->nr_locations; i++ )
	{
		fprintf( f, "%g %g %g\n", so->sensor_normals[i*3+0], so->sensor_normals[i*3+1], so->sensor_normals[i*3+2] );
	}
	fclose( f );

}



int main()
{
	int magicon = 0;
	double Start = OGGetAbsoluteTime();

	ctx = survive_init( 0 );

	if( !ctx )
	{
		fprintf( stderr, "Fatal. Could not start\n" );
		return 1;
	}

	dump_iface( survive_get_so_by_name( ctx, "HMD" ), "HMD" );
	dump_iface( survive_get_so_by_name( ctx, "WM0" ), "WM0" );
	dump_iface( survive_get_so_by_name( ctx, "WM1" ), "WM1" );
	dump_iface( survive_get_so_by_name( ctx, "TR0" ), "TR0" );

	while(survive_poll(ctx) == 0)
	{
		double Now = OGGetAbsoluteTime();
		if( Now > (Start+1) && !magicon )
		{
			survive_send_magic(ctx,1,0,0);
			magicon = 1;
		}
		//Do stuff.
	}
}

