#include <libsurvive/survive.h>
#include <CNFGFunctions.h>
#include <os_generic.h>
#include <CNFG3D.h>
#include <stdio.h>
#include <string.h>

unsigned frames = 0;
unsigned long iframeno = 0;


#define HMX 40
#define HMY 40
short screenx, screeny;
float Heightmap[HMX*HMY];

void DrawFrame( int offsetx, int lscreenx, FLT * eye, FLT * at, FLT * up )
{
	float scale = .02;
	int x, y;
	float fdt = ((iframeno++)%(360*10))/10.0;
//	float eye[3] = { (float)sin(fdt*(3.14159/180.0))*1, (float)cos(fdt*(3.14159/180.0))*1, 1 };
//	float at[3] = { 0,0, 0 };
//	float up[3] = { 0,0, 1 };

	float leye[3] = { eye[0], eye[1], eye[2] };
	float lat[3] = { at[0], at[1], at[2] };
	float lup[3] = { up[0], up[1], up[2] };

	tdSetViewport( -1, -1, 1, 1, lscreenx, screeny );

	tdMode( tdPROJECTION );
	tdIdentity( gSMatrix );
	tdPerspective( 80, ((float)lscreenx)/((float)screeny), .1, 200., gSMatrix );

	tdMode( tdMODELVIEW );
	tdIdentity( gSMatrix );
	//tdTranslate( gSMatrix, 0, 0, -40 );

	

	tdLookAt( gSMatrix, leye, lat, lup );

	for( x = 0; x < HMX-1; x++ )
	for( y = 0; y < HMY-1; y++ )
	{
		float tx = x-HMX/2;
		float ty = y-HMY/2;
		float pta[3];
		float ptb[3];
		float ptc[3];
		float ptd[3];

		float normal[3];
		float lightdir[3] = { 1, -1, 1 };
		float tmp1[3];
		float tmp2[3];

		RDPoint pto[6];

		pta[0] = tx+0; pta[1] = ty+0; pta[2] = Heightmap[(x+0)+(y+0)*HMX];
		ptb[0] = tx+1; ptb[1] = ty+0; ptb[2] = Heightmap[(x+1)+(y+0)*HMX];
		ptc[0] = tx+0; ptc[1] = ty+1; ptc[2] = Heightmap[(x+0)+(y+1)*HMX];
		ptd[0] = tx+1; ptd[1] = ty+1; ptd[2] = Heightmap[(x+1)+(y+1)*HMX];

		tdPSub( pta, ptb, tmp2 );
		tdPSub( ptc, ptb, tmp1 );
		tdCross( tmp1, tmp2, normal );

		int i;
		for( i = 0 ; i < 3; i++ )
		{
			pta[i] *= scale;
			ptb[i] *= scale;
			ptc[i] *= scale;
			ptd[i] *= scale;
		}

		tdFinalPoint( pta, pta );
		tdFinalPoint( ptb, ptb );
		tdFinalPoint( ptc, ptc );
		tdFinalPoint( ptd, ptd );

		if( pta[2] >= 1.0 ) continue;
		if( ptb[2] >= 1.0 ) continue;
		if( ptc[2] >= 1.0 ) continue;
		if( ptd[2] >= 1.0 ) continue;

		if( pta[2] < 0 ) continue;
		if( ptb[2] < 0 ) continue;
		if( ptc[2] < 0 ) continue;
		if( ptd[2] < 0 ) continue;

/*		if( pta[3] < -1 ) continue;
		if( ptb[3] < -1 ) continue;
		if( ptc[3] < -1 ) continue;
		if( ptd[3] < -1 ) continue;
*/

//		if( pta[2] < 0 ) continue;
//		if( ptb[2] < 0 ) continue;
//		if( ptc[2] < 0 ) continue;
//		if( ptd[2] < 0 ) continue;


		pta[0] += offsetx;
		ptb[0] += offsetx;
		ptc[0] += offsetx;
		ptd[0] += offsetx;

		pto[0].x = pta[0]; pto[0].y = pta[1];
		pto[1].x = ptb[0]; pto[1].y = ptb[1];
		pto[2].x = ptd[0]; pto[2].y = ptd[1];

		pto[3].x = ptc[0]; pto[3].y = ptc[1];
		pto[4].x = ptd[0]; pto[4].y = ptd[1];
		pto[5].x = pta[0]; pto[5].y = pta[1];

//		CNFGColor(((x+y)&1)?0xFFFFFF:0x000000);

		float bright = tdDot( normal, lightdir );
		if( bright < 0 ) bright = 0;
		CNFGColor( (int)( bright * 50 ) );

		CNFGTackPoly( &pto[0], 3 );		CNFGTackPoly( &pto[3], 3 );
//		CNFGTackSegment( pta[0], pta[1], ptb[0], ptb[1] );
//		CNFGTackSegment( pta[0], pta[1], ptc[0], ptc[1] );
		
	}
 
/*
	for( f = 0; f <= 6.28; f+=0.01 )
	{
		tdPSet( pta, cos( f ), sin(f), cos( f * 10. + ThisTime) );
		tdPSet( ptb, cos( f - 0.01 ), sin(f - 0.01), cos( (f-0.01) * 10. + ThisTime) );
	//			printf( "(%f, %f, %f) -> ", pta[0], pta[1], pta[2] );
		tdFinalPoint( pta, pta );
		tdFinalPoint( ptb, ptb );
	//			printf( "%f, %f, %f\n", pta[0], pta[1], pta[2] );
		CNFGTackSegment( pta[0], pta[1], ptb[0], ptb[1] );
	}

*/


}

SurvivePose phmd;

void my_raw_pose_process(SurviveObject *so, uint8_t lighthouse, SurvivePose *pose) {
	survive_default_raw_pose_process(so, lighthouse, pose);
	memcpy( &phmd , pose, sizeof( phmd ) );
	//printf("%s POSE %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f %0.6f\n", so->codename, pose->Pos[0], pose->Pos[1],	pose->Pos[2], pose->Rot[0], pose->Rot[1], pose->Rot[2], pose->Rot[3]);
}


int gargc;
char ** gargv;

void * LibSurviveThread()
{	
	struct SurviveContext *ctx = survive_init(gargc,gargv);

	if (ctx == 0) // implies -help or similiar
		return 0;

	survive_install_raw_pose_fn(ctx, my_raw_pose_process);

	survive_startup(ctx);

	while (survive_poll(ctx) == 0) {
	}
}


int main( int argc, char ** argv )
{
	int i, x, y;
	double ThisTime;
	double LastFPSTime = OGGetAbsoluteTime();
	double LastFrameTime = OGGetAbsoluteTime();
	double SecToWait;
	int linesegs = 0;


	gargc = argc;
	gargv = argv;

	CNFGBGColor = 0x800000;
	CNFGDialogColor = 0x444444;
	CNFGSetup( "Test Bench", 2160, 1200 );
	// CNFGSetupFullscreen( "Test Bench", 0 );

	for( x = 0; x < HMX; x++ )
	for( y = 0; y < HMY; y++ )
	{
		Heightmap[x+y*HMX] = tdPerlin2D( x, y )*8.;
	}

	float diopter = 60;

	OGCreateThread( LibSurviveThread, 0 );


	while(1)
	{
		int i, pos;
		float f;
		iframeno++;
		RDPoint pto[3];

		CNFGHandleInput();

		CNFGClearFrame();
		CNFGColor( 0xFFFFFF );
		CNFGGetDimensions( &screenx, &screeny );

		double p[3] = { 1, 1, 1} ;
		double eye1[3] = { 1, 1, 1 };
		double eye2[3] = { 1, 1, 1 };
		double at1[3] = { 1, 1, 1 };
		double at2[3] = { 1, 1, 1 };
		double up[3] = { 1, 1, 1 };

		double pin[3] = {  0.0, 0., 0 }; //Left eye
		double pineye1[3] = {  0.030, 0., 0 }; //Left eye
		double pineye2[3] = { -0.030, 0., 0 };
		double pinat1[3] = {  0.030, 0., 1 }; //Left eye
		double pinat2[3] = { -0.030, 0., 1 };

		double pinup[3] = { 0, 1., 0 };

		ApplyPoseToPoint(p, &phmd, pin);
		ApplyPoseToPoint(eye1, &phmd, pineye1);
		ApplyPoseToPoint(eye2, &phmd, pineye2);

		ApplyPoseToPoint(at1, &phmd, pinat1);
		ApplyPoseToPoint(at2, &phmd, pinat1);

		ApplyPoseToPoint(up, &phmd, pinup);
		up[0] -= p[0];
		up[1] -= p[1];
		up[2] -= p[2];

		//printf( "Eye: %f %f %f At: %f %f %f Up: %f %f %f\n", eye1[0], eye1[1], eye1[2], at1[0], at1[1], at1[2], up[0], up[1], up[2] );

		DrawFrame(0+diopter,1080,eye1, at1, up);
		DrawFrame(1080-diopter,1080,eye2,at2, up);

		frames++;
		CNFGSwapBuffers();

		ThisTime = OGGetAbsoluteTime();
		if( ThisTime > LastFPSTime + 1 )
		{
			printf( "FPS: %d\n", frames );
			frames = 0;
			linesegs = 0;
			LastFPSTime+=1;
		}
	}
}

