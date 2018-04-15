#include "fileutil.h"
#include <stdio.h>
#include <stdlib.h>

#define PI 3.14159265358979323846264


og_mutex_t  read_mutex;
og_thread_t read_thread;
double read_hmdAngles[NUM_SWEEP][NUM_HMD];
int read_hmdAngleViewed[NUM_SWEEP][NUM_HMD];
int read_frameno=0;

static FILE *fopen_orDie(const char *path, const char *flag)
{
	FILE *f = fopen(path, flag);
	if (f == NULL) {
		printf("ERROR could not oepn %s for %s\n", path, flag);
		exit(1);
	}
	return f;
}

static void SeekToken(FILE *f, const char *keyword)
{
	char token[4096];
	do {
		fscanf(f, "%s", token);
	} while( strcmp(token,keyword)!=0 && !feof(f) );
}

void LoadLighthousePos(
        const char *path,
        float *x, float *y, float *z,
        float *qi, float *qj, float *qk, float *qreal)
{
	FILE *f = fopen_orDie(path,"r");
	SeekToken(f, "POS:");
	fscanf(f, "%f %f %f\n", x, y, z);
	SeekToken(f, "QUAT:");
	fscanf(f, "%f %f %f %f\n", qreal, qi, qj, qk);
	fclose (f);
}

void LoadHmdProcessedDataAngles(
	const char *path,
	double angles[NUM_SWEEP][NUM_HMD])
{
	int i,j;
	char   type[256];
	char   sweep[256];
	int    id;
	int    nSweep;
	double ang;
	double d1,d2,d3;  // revisit these numbers later

	// Initialize all of the angles to -9999
	for (i=0; i<NUM_SWEEP; i++) {
		for (j=0; j<NUM_HMD; j++) {
			angles[i][j] = -9999.0;   // Initially no value
		}
	}

	FILE *f = fopen_orDie(path, "r");

	while (!feof(f))
	{
		// Read the line from the file
		int rt=fscanf(f, "%s %s %d %d %lf %lf %lf %lf",
			&type, &sweep, &id, &nSweep, &ang,
			&d1,&d2,&d3);

		if (rt<8) { break; }

		// Only hmd points
		if ( strcmp(type,"HMD")!=0 ) { continue; }

		// Which sweep is it?
		int sweepId=-1;
		if      ( strcmp(sweep,"LX")==0 ) { sweepId=SWEEP_LX; }
		else if ( strcmp(sweep,"LY")==0 ) { sweepId=SWEEP_LY; }
		else if ( strcmp(sweep,"RX")==0 ) { sweepId=SWEEP_RX; }
		else if ( strcmp(sweep,"RY")==0 ) { sweepId=SWEEP_RY; }
		else { continue; }
		
		// Convert the angle from ticks to radians
		angles[sweepId][id] = (PI / 400000.0) * ( ang-200000.0 );
	}

	fclose(f);
}

void *ThreadReadHmtAngles(void *junk)
{
	char house[256];
	char xy[256];
	char hmd[256];
	double ts;
	int id;
	int syncid;
	int timeInSweep;
	int length;
	//lighthouse sweep hmdOrwmd timestamp id syncid timeinsweep length

	while(1) {
		int rt=scanf("%s %s %s %lf %d %d %d %d", house, xy, hmd, &ts, &id, &syncid, &timeInSweep, &length);
		if (rt==8)
		{
			//int rt=scanf("%s %s %s %lf %d %d %d %d", house, xy, hmd, &ts, &id, &syncid, &timeInSweep, &length);
			//printf( "%s %s %s %f %d %d %d %d\n", house, xy, hmd, ts,id, syncid, timeInSweep, length );

			if( id < 0 ) continue;
			int sweepId=0;
			if ( house[0]=='R' ) { sweepId+=2; }
			if ( xy[0]   =='Y' ) { sweepId++; }
			double angle = (PI / 400000.0) * ( (double)timeInSweep-200000.0 );
		
			if      ( strcmp(hmd,"HMD")==0 ) { id += 0;  } 
			else if ( strcmp(hmd,"WM0")==0 ) { id += 32; }
			else if ( strcmp(hmd,"WM1")==0 ) { id += 56; }
			else    { continue; }
		
			if ( id<0 || id >NUM_HMD) { continue; }

			OGLockMutex  (read_mutex);
			read_hmdAngles[sweepId][id] = angle;
			OGUnlockMutex(read_mutex);
			read_hmdAngleViewed[sweepId][id] = read_frameno;
		}
	}
}


