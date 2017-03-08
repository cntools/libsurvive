#include <stdio.h>
#include <stdlib.h>
#include "linmath.h"
#include <string.h>
#include <stdint.h>
#include <math.h>

#define PTS 32
#define MAX_CHECKS 40000
#define MIN_HITS_FOR_VALID 10

FLT hmd_points[PTS*3];
FLT hmd_norms[PTS*3];
FLT hmd_point_angles[PTS*2];
int hmd_point_counts[PTS*2];
int best_hmd_target = 0;
int LoadData( char Camera, const char * FileData );

//Values used for RunTest()
FLT LighthousePos[3] = { 0, 0, 0 };
FLT LighthouseQuat[4] = { 1, 0, 0, 0 };

FLT RunTest( int print );
void PrintOpti();

typedef struct
{
	FLT x;
	FLT y;
	FLT z;
} Point;

typedef struct
{
	unsigned char index1;
	unsigned char index2;
	FLT KnownDistance;
} PointPair;

typedef struct
{
	FLT radius;
	FLT HorizAngle;
	FLT VertAngle;
} RadiusGuess;

#define SQUARED(x) ((x)*(x))

FLT calculateFitness(RadiusGuess *radii, PointPair *pairs, size_t numPairs)
{
	FLT fitness = 0;
	for (size_t i = 0; i < numPairs; i++)
	{
		fitness += SQUARED(radii[pairs[i].index1].radius)
			+ SQUARED(radii[pairs[i].index2].radius)
			- 2 * radii[pairs[i].index1].radius * radii[pairs[i].index2].radius
				* FLT_SIN( radii[pairs[i].index1].HorizAngle) * FLT_SIN(radii[pairs[i].index2].HorizAngle)
				* FLT_COS( radii[pairs[i].index1].VertAngle - radii[pairs[i].index2].VertAngle ) 
			+ FLT_COS(radii[pairs[i].index1].VertAngle) * FLT_COS(radii[pairs[i].index2].VertAngle);
	}

	return FLT_SQRT(fitness);
}

int main( int argc, char ** argv )
{

	if( argc != 3 )
	{
		fprintf( stderr, "Error: usage: camfind [camera (L or R)] [datafile]\n" );
		exit( -1 );
	}

	//Load either 'L' (LH1) or 'R' (LH2) data.
	if( LoadData( argv[1][0], argv[2] ) ) return 5;




	
}





int LoadData( char Camera, const char * datafile  )
{

	//First, read the positions of all the sensors on the HMD.
	FILE * f = fopen( "HMD_points.csv", "r" );
	int pt = 0;
	if( !f ) { fprintf( stderr, "error: can't open hmd points.\n" ); return -5; }
	while(!feof(f) && !ferror(f) && pt < PTS)
	{
		float fa, fb, fc;
		int r = fscanf( f,"%g %g %g\n", &fa, &fb, &fc );
		hmd_points[pt*3+0] = fa;
		hmd_points[pt*3+1] = fb;
		hmd_points[pt*3+2] = fc;
		pt++;
		if( r != 3 )
		{
			fprintf( stderr, "Not enough entries on line %d of points\n", pt );
			return -8;
		}
	}
	if( pt < PTS )
	{
		fprintf( stderr, "Not enough points.\n" );
		return -9;
	}
	fclose( f );
	printf( "Loaded %d points\n", pt );

	//Read all the normals on the HMD into hmd_norms.
	f = fopen( "HMD_normals.csv", "r" );
	int nrm = 0;
	if( !f ) { fprintf( stderr, "error: can't open hmd points.\n" ); return -5; }
	while(!feof(f) && !ferror(f) && nrm < PTS)
	{
		float fa, fb, fc;
		int r = fscanf( f,"%g %g %g\n", &fa, &fb, &fc );
		hmd_norms[nrm*3+0] = fa;
		hmd_norms[nrm*3+1] = fb;
		hmd_norms[nrm*3+2] = fc;
		nrm++;
		if( r != 3 )
		{
			fprintf( stderr, "Not enough entries on line %d of normals\n", nrm );
			return -8;
		}
	}
	if( nrm < PTS )
	{
		fprintf( stderr, "Not enough points.\n" );
		return -9;
	}
	if( nrm != pt )
	{
		fprintf( stderr, "point/normal counts disagree.\n" );
		return -9;
	}
	fclose( f );
	printf( "Loaded %d norms\n", nrm );

	//Actually load the processed data!
	int xck = 0;
	f = fopen( datafile, "r" );
	if( !f )
	{
		fprintf( stderr, "Error: cannot open %s\n", datafile );
		exit (-11);
	}
	int lineno = 0;
	while( !feof( f ) )
	{
		//Format:
		// HMD LX 0 3433 173656.227498 327.160210 36.342361 2.990936
		lineno++;
		char devn[10];
		char inn[10];
		int id;
		int pointct;
		FLT avgTime;
		FLT avgLen;
		FLT stddevTime;
		FLT stddevLen;
		int ct = fscanf( f, "%9s %9s %d %d %lf %lf %lf %lf\n", devn, inn, &id, &pointct, &avgTime, &avgLen, &stddevTime, &stddevLen );
		if( ct == 0 ) continue;
		if( ct != 8 )
		{
			fprintf( stderr, "Malformatted line, %d in processed_data.txt\n", lineno );
		}
		if( strcmp( devn, "HMD" ) != 0 ) continue;

		if( inn[0] != Camera ) continue;

		int isy = inn[1] == 'Y';

		hmd_point_angles[id*2+isy] = ( avgTime - 200000 ) / 200000 * 3.1415926535/2.0;
		hmd_point_counts[id*2+isy] = pointct;
	}
	fclose( f );


	int targpd;
	int maxhits = 0;

	for( targpd = 0; targpd < PTS; targpd++ )
	{
		int hits = hmd_point_counts[targpd*2+0];
		if( hits > hmd_point_counts[targpd*2+1] ) hits = hmd_point_counts[targpd*2+1];
		//Need an X and a Y lock.  

		if( hits > maxhits ) { maxhits = hits; best_hmd_target = targpd; }
	}
	if( maxhits < MIN_HITS_FOR_VALID )
	{
		fprintf( stderr, "Error: Not enough data for a primary fix.\n" );
	}

	return 0;
}

