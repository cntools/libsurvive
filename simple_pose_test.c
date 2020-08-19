#ifdef __linux__
#include <unistd.h>
#endif
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <survive.h>
#include "src/survive_cal.h"
#include <CNFG3D.h>
#include <os_generic.h>
#include <CNFGFunctions.h>
#include <string.h>
#include <linmath.h>
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

float viewX = 0.7853975;
float viewY = 0.7853975;
int down, downx, downy;
void HandleButton( int x, int y, int button, int bDown )
{
	if (button == 1) {
		if (bDown) {
			down = 1;
			downx = x;
			downy = y;
		} else {
			down = 0;
		}
	}
}

void HandleMotion( int x, int y, int mask )
{
	if (down) {
		viewX += (x - downx) / 100.0;
		viewY -= (y - downy) / 100.0;
		if (viewY < 0.01)
			viewY = 0.01;
		if (viewY > 3.14)
			viewY = 3.14;
		downx = x;
		downy = y;
	}
}

void HandleDestroy()
{
}

FLT hpos[3];
FLT hpos2[3];
FLT hposx[3];

void testprog_raw_pose_process(SurviveObject *so, uint32_t timecode, SurvivePose *pose) {
	survive_default_pose_process(so, timecode, pose);

	//	if (strcmp(so->codename, "WW0") != 0)
	//		return;

	// print the pose;
/*	double qw = quat[0];
	double qx = quat[1];
	double qy = quat[2];
	double qz = quat[3];

	hra = 2 * acos(qw);
	hrx = qx / sqrt(1-qw*qw);
	hry = qy / sqrt(1-qw*qw);
	hrz = qz / sqrt(1-qw*qw);
	hra *=  180/3.14159;

	hx = pos[0];
	hy = pos[1];
	hz = pos[2];*/

	printf("Pose: [%u][%s][% 08.8f,% 08.8f,% 08.8f] [ang:%08.2f %08.2f %08.2f %08.2f]\n", timecode, so->codename,
		   pose->Pos[0], pose->Pos[1], pose->Pos[2], pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]);

	hpos[0] = pose->Pos[0];
	hpos[1] = pose->Pos[1];
	hpos[2] = pose->Pos[2];
	FLT hposin[3] = { 0, 0, 1 };
	ApplyPoseToPoint(hpos2, pose, hposin);
	FLT hposinx[3] = {.1, 0, 0};
	ApplyPoseToPoint(hposx, pose, hposinx);

	fflush(stdout);
}


void testprog_angle_process( SurviveObject * so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle, uint32_t lh )
{
//	printf( "%d: %d %d\n", sensor_id, acode, timecode );
}

void DrawLineSegment( float x, float y, float z, float x2, float y2, float z2 )
{
	//float pti[6] = { 1, sin(TimeSinceStart), cos(TimeSinceStart ), 1, 0, 0};
	float pa[3] = { x, y, z };
	float pb[3] = { x2, y2, z2 };

	tdFinalPoint( pa, pa );
	tdFinalPoint( pb, pb );

	if( pa[2] >= 1.0 ) return;
	if( pb[2] >= 1.0 ) return;
	if( pa[2] < 0 ) return;
	if( pb[2] < 0 ) return;

	int dx, dy;
	for( dx = 0; dx < 2; dx++ )
	for( dy = 0; dy < 2; dy++ )
	{
		CNFGTackPixel( pa[0]+dx, pa[1]+dy );
		CNFGTackSegment( pa[0]+dx, pa[1]+dy, pb[0]+dx, pb[1]+dy );
	}
}


void *GUIThread(void*v)
{
	CNFGSetup( "Orientation test", 640, 480 );
	double fLast = 0;
	short screenx, screeny;
	double TimeSinceStart = 0;
	while(1)
	{
		double Now = OGGetAbsoluteTime();
		double dt = Now - fLast;
		fLast = Now;
		TimeSinceStart += dt;

		CNFGClearFrame();
		CNFGHandleInput();

		CNFGColor( 0xFFFFFF );
		CNFGGetDimensions( &screenx, &screeny );

		int x, y;
		float eye[3] = {3 * sin(viewX) * sin(viewY), 3 * cos(viewX) * sin(viewY),
						3 * cos(viewY)}; // Create a 2-rotation with Z primarily up.
		// float up[3] = { 0, cos(viewY), sin(viewY)}; //Create a 2-rotation with Z primarily up.
		float at[3] = { 0,0, 0 };
		float up[3] = {0, 0, 1.0};
		float right[3];
		tdCross(up, eye, right);
		tdCross(eye, right, up); // Have to make sure we're making a coordinate frame for lookat.
		tdNormalizeSelf(right);
		tdSetViewport( -1, -1, 1, 1, screenx, screeny );

		tdMode( tdPROJECTION );
		tdIdentity( gSMatrix );
		tdPerspective( 40, ((float)screenx)/((float)screeny), .1, 200., gSMatrix );

		tdMode( tdMODELVIEW );
		tdIdentity( gSMatrix );
		tdLookAt( gSMatrix, eye, at, up );

		CNFGColor( 0x00ffff ); DrawLineSegment( hpos[0], hpos[1], hpos[2], hpos2[0], hpos2[1], hpos2[2] );
		CNFGColor( 0xff00ff ); DrawLineSegment( hpos[0], hpos[1], hpos[2], hpos[0], hpos[1], hpos[2] );
		CNFGColor(0xffff00);
		DrawLineSegment(hpos[0], hpos[1], hpos[2], hposx[0], hposx[1], hposx[2]);
		CNFGColor( 0x0000ff ); DrawLineSegment( 0, 0, 0, 1, 0, 0  );
		CNFGColor( 0xff0000 ); DrawLineSegment( 0, 0, 0, 0, 1, 0  );
		CNFGColor( 0x00ff00 ); DrawLineSegment( 0, 0, 0, 0, 0, 1  );
		CNFGColor( 0xffffff ); DrawLineSegment( 0, 0, 0, 0, 0, 0  );
		CNFGSwapBuffers();
		OGUSleep(10000);
	}
}


int main( int argc, char ** argv )
{
	int magicon = 0;
	double Start = OGGetAbsoluteTime();

	ctx = survive_init( argc, argv );
	if (ctx == 0) { // Implies --help or similiar
		return -1;
	}

	//survive_install_button_fn(ctx, testprog_button_process);
	survive_install_pose_fn(ctx, testprog_raw_pose_process);
	//survive_install_imu_fn(ctx, testprog_imu_process);
	//survive_install_angle_fn(ctx, testprog_angle_process );

#if 0 // Don't reset poses
	ctx->bsd[0].PositionSet = ctx->bsd[1].PositionSet = 1;
	int i;
	for( i = 0; i < 2; i++ )
	{
		SurvivePose * p = &ctx->bsd[i].Pose;
		p->Pos[0] = 0;
		p->Pos[1] = 0;
		p->Pos[2] = 0;
		p->Rot[0] = 1;
		p->Rot[1] = 0;
		p->Rot[2] = 0;
		p->Rot[3] = 0;
	}
#endif
	OGCreateThread(GUIThread, "GUI", 0);

	if( !ctx )
	{
		fprintf( stderr, "Fatal. Could not start\n" );
		return 1;
	}

	//survive_cal_install(ctx);
	

	while(survive_poll(ctx) == 0)
	{
		//Do Stuff
	}
}

