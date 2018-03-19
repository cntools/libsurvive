#ifdef __linux__
#include <unistd.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <survive.h>

#include <os_generic.h>
#include <CNFGFunctions.h>

struct SurviveContext * ctx;

void HandleKey( int keycode, int bDown )
{
	if( !bDown ) return;

	if( keycode == 'O' || keycode == 'o' )
	{
		survive_send_magic(ctx,1,0,0);
	}
	if( keycode == 'F' || keycode == 'f' )
	{
		survive_send_magic(ctx,0,0,0);
	}
}

void HandleButton( int x, int y, int button, int bDown )
{
}

void HandleMotion( int x, int y, int mask )
{
}

void HandleDestroy()
{
}

void testprog_button_process(SurviveObject * so, uint8_t eventType, uint8_t buttonId, uint8_t axis1Id, uint16_t axis1Val, uint8_t axis2Id, uint16_t axis2Val)
{
	survive_default_button_process(so, eventType, buttonId, axis1Id, axis1Val, axis2Id, axis2Val);
	
	// do nothing.
	printf("ButtonEntry: eventType:%x, buttonId:%d, axis1:%d, axis1Val:%8.8x, axis2:%d, axis2Val:%8.8x\n",
		eventType,
		buttonId,
		axis1Id,
		axis1Val,
		axis2Id,
		axis2Val);

	// Note: the code for haptic feedback is not yet written to support wired USB connections to the controller.  
	// Work is still needed to reverse engineer that USB protocol.

	// let's do some haptic feedback if the trigger is pushed
	if (buttonId == 24 && eventType == 1) // trigger engage
	{
		for (int j = 0; j < 6; j++)
		{
			for (int i = 0; i < 0x5; i++)
			{
				survive_haptic(so, 0, 0xf401, 0xb5a2, 0x0100);
				//survive_haptic(so, 0, 0xf401, 0xb5a2, 0x0100);
				OGUSleep(1000);
			}
			OGUSleep(20000);
		}
	}

	// let's do some haptic feedback if the touchpad is pressed.
	if (buttonId == 2 && eventType == 1) // trigger engage
	{
		for (int j = 0; j < 6; j++)
		{
			for (int i = 0; i < 0x1; i++)
			{
				survive_haptic(so, 0, 0xf401, 0x05a2, 0xf100);
				//survive_haptic(so, 0, 0xf401, 0xb5a2, 0x0100);
				OGUSleep(5000);
			}
			OGUSleep(20000);
		}
	}
}

void testprog_lighthouse_process(SurviveContext *ctx, uint8_t lighthouse, SurvivePose *pose) {
	survive_default_lighthouse_pose_process(ctx, lighthouse, pose);

	printf("Lighthouse: [%1.1x][% 08.8f,% 08.8f,% 08.8f] [% 08.8f,% 08.8f,% 08.8f,% 08.8f]\n", lighthouse, pose->Pos[0],
		   pose->Pos[1], pose->Pos[2], pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]);
}

void testprog_raw_pose_process(SurviveObject *so, uint8_t lighthouse, SurvivePose *pose) {
	survive_default_raw_pose_process(so, lighthouse, pose);

	// print the pose;
	printf("Pose: [%1.1x][%s][% 08.8f,% 08.8f,% 08.8f] [% 08.8f,% 08.8f,% 08.8f,% 08.8f]\n", lighthouse, so->codename,
		   pose->Pos[0], pose->Pos[1], pose->Pos[2], pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]);
}

void testprog_imu_process(SurviveObject * so, int mask, FLT * accelgyromag, uint32_t timecode, int id)
{
	survive_default_imu_process(so, mask, accelgyromag, timecode, id);

	// so something here, like printing out the data...

}


static void dump_iface( struct SurviveObject * so, const char * prefix )
{
	int i;
	FILE * f;
	char fname[1024];

	if (!so) { return; }

	sprintf( fname, "%s_points.csv", prefix );
	f = fopen( fname, "w" );
	for( i = 0; i < so->sensor_ct; i++ )
	{
		fprintf( f, "%g %g %g\n", so->sensor_locations[i*3+0], so->sensor_locations[i*3+1], so->sensor_locations[i*3+2] );
	}
	fclose( f );

	sprintf( fname, "%s_normals.csv", prefix );
	f = fopen( fname, "w" );
	for( i = 0; i < so->sensor_ct; i++ )
	{
		fprintf( f, "%g %g %g\n", so->sensor_normals[i*3+0], so->sensor_normals[i*3+1], so->sensor_normals[i*3+2] );
	}
	fclose( f );

}



int main( int argc, char ** argv )
{
	int magicon = 0;
	double Start = OGGetAbsoluteTime();

	ctx = survive_init( argc, argv );

	survive_install_button_fn(ctx, testprog_button_process);
	survive_install_raw_pose_fn(ctx, testprog_raw_pose_process);

	survive_install_imu_fn(ctx, testprog_imu_process);
	survive_install_lighthouse_pose_fn(ctx, testprog_lighthouse_process);

	if( !ctx )
	{
		fprintf( stderr, "Fatal. Could not start\n" );
		return 1;
	}

	dump_iface( survive_get_so_by_name( ctx, "HMD" ), "HMD" );
	dump_iface( survive_get_so_by_name( ctx, "WM0" ), "WM0" );
	dump_iface( survive_get_so_by_name( ctx, "WM1" ), "WM1" );
	dump_iface( survive_get_so_by_name( ctx, "TR0" ), "TR0" );
	dump_iface( survive_get_so_by_name( ctx, "WW0" ), "WW0" );

	survive_cal_install(ctx);

	while(survive_poll(ctx) == 0)
	{
		//double Now = OGGetAbsoluteTime();
		//if( Now > (Start+1) && !magicon )
		//{
		//	survive_send_magic(ctx,1,0,0);
		//	magicon = 1;
		//}
		//Do stuff.
	}
}

