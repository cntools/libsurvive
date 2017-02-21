#include <stdio.h>
#include <assert.h>
#include <memory.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include "linmath.h"
#include "torus_localizer.h"

#define PTS 32
#define MAX_CHECKS 40000
#define MIN_HITS_FOR_VALID 10

FLT hmd_points[PTS * 3];
FLT hmd_norms[PTS * 3];

// index for a given sensor is (2*sensor + is_sensor_y ? 1 : 0)
FLT hmd_point_angles[PTS * 2];  
int hmd_point_counts[PTS * 2];
int best_hmd_target = 0;

static void printTrackedObject(TrackedObject *to)
{
	for (unsigned int i = 0; i < to->numSensors; i++)
	{
		printf("%2.2d: [%5.5f,%5.5f] (%5.5f,%5.5f,%5.5f)\n",
			i,
			to->sensor[i].theta,
			to->sensor[i].phi,
			to->sensor[i].point.x,
			to->sensor[i].point.y,
			to->sensor[i].point.z
			);
	}
}

static void runTheNumbers()
{
	TrackedObject *to;

	to = malloc(sizeof(TrackedObject)+(PTS * sizeof(TrackedSensor)));

	int sensorCount = 0;

	for (int i = 0; i < PTS; i++)
	{
		// if there are enough valid counts for both the x and y sweeps for sensor i
		if ((hmd_point_counts[2*i] > MIN_HITS_FOR_VALID) &&
			(hmd_point_counts[2*i + 1] > MIN_HITS_FOR_VALID))
		{
			to->sensor[sensorCount].point.x = hmd_points[i * 3 + 0];
			to->sensor[sensorCount].point.y = hmd_points[i * 3 + 1];
			to->sensor[sensorCount].point.z = hmd_points[i * 3 + 2];
			to->sensor[sensorCount].normal.x = hmd_norms[i * 3 + 0];
			to->sensor[sensorCount].normal.y = hmd_norms[i * 3 + 1];
			to->sensor[sensorCount].normal.z = hmd_norms[i * 3 + 2];
			to->sensor[sensorCount].theta = hmd_point_angles[i * 2 + 0] + LINMATHPI / 2;
			to->sensor[sensorCount].phi = hmd_point_angles[i * 2 + 1] + LINMATHPI / 2;
			sensorCount++;
		}
	}

	to->numSensors = sensorCount;

	printf("Using %d sensors to find lighthouse.\n", sensorCount);

	Point lh;
	//for (int i = 0; i < 200; i++)
	for (int i = 0; i < 1; i++)
	{
		lh = SolveForLighthouse(to, 0);
		//(0.156754, -2.403268, 2.280167)
		//assert(fabs((lh.x / 0.1419305302702402) - 1) < 0.00001);
		//assert(fabs((lh.y / 2.5574949720325431) - 1) < 0.00001);
		//assert(fabs((lh.z / 2.2451193935772080) - 1) < 0.00001);
		//assert(lh.x > 0);
		//assert(lh.y > 0);
		//assert(lh.z > 0);
	}

	printf("(%f, %f, %f)\n", lh.x, lh.y, lh.z);

	//printTrackedObject(to);
	free(to);
}


int LoadData(char Camera, const char * datafile)
{


	//First, read the positions of all the sensors on the HMD.
	FILE * f = fopen("HMD_points.csv", "r");
	int pt = 0;
	if (!f) 
	{ 
		fprintf(stderr, "error: can't open hmd points.\n"); 
		return -5; 
	}
	while (!feof(f) && !ferror(f) && pt < PTS)
	{
		float fx, fy, fz;
		int r = fscanf(f, "%g %g %g\n", &fx, &fy, &fz);
		hmd_points[pt * 3 + 0] = fx;
		hmd_points[pt * 3 + 1] = fy;
		hmd_points[pt * 3 + 2] = fz;
		pt++;
		if (r != 3)
		{
			fprintf(stderr, "Not enough entries on line %d of points\n", pt);
			return -8;
		}
	}
	if (pt < PTS)
	{
		fprintf(stderr, "Not enough points.\n");
		return -9;
	}
	fclose(f);
	printf("Loaded %d points\n", pt);

	//Read all the normals on the HMD into hmd_norms.
	f = fopen("HMD_normals.csv", "r");
	int nrm = 0;
	if (!f) 
	{ 
		fprintf(stderr, "error: can't open hmd points.\n"); 
		return -5; 
	}
	while (!feof(f) && !ferror(f) && nrm < PTS)
	{
		float fa, fb, fc;
		int r = fscanf(f, "%g %g %g\n", &fa, &fb, &fc);
		hmd_norms[nrm * 3 + 0] = fa;
		hmd_norms[nrm * 3 + 1] = fb;
		hmd_norms[nrm * 3 + 2] = fc;
		nrm++;
		if (r != 3)
		{
			fprintf(stderr, "Not enough entries on line %d of normals\n", nrm);
			return -8;
		}
	}
	if (nrm < PTS)
	{
		fprintf(stderr, "Not enough points.\n");
		return -9;
	}
	if (nrm != pt)
	{
		fprintf(stderr, "point/normal counts disagree.\n");
		return -9;
	}
	fclose(f);
	printf("Loaded %d norms\n", nrm);

	//Actually load the processed data!
	int xck = 0;
	f = fopen(datafile, "r");
	if (!f)
	{
		fprintf(stderr, "Error: cannot open %s\n", datafile);
		exit(-11);
	}
	int lineno = 0;
	while (!feof(f))
	{
		//Format:
		// HMD LX 0 3433 173656.227498 327.160210 36.342361 2.990936
		lineno++;
		char devn[10];
		char inn[10];
		int id;
		int pointct;
		double avgTime;
		double avgLen;
		double stddevTime;
		double stddevLen;
		int ct = fscanf(f, "%9s %9s %d %d %lf %lf %lf %lf\n", devn, inn, &id, &pointct, &avgTime, &avgLen, &stddevTime, &stddevLen);
		if (ct == 0) continue;
		if (ct != 8)
		{
			fprintf(stderr, "Malformatted line, %d in processed_data.txt\n", lineno);
		}
		if (strcmp(devn, "HMD") != 0) continue;

		if (inn[0] != Camera) continue;

		int isy = inn[1] == 'Y';

		hmd_point_angles[id * 2 + isy] = ((FLT)avgTime - 200000) * LINMATHPI / 200000 / 2;
		
		hmd_point_counts[id * 2 + isy] = pointct;
	}
	fclose(f);


	int targpd;
	int maxhits = 0;

	for (targpd = 0; targpd < PTS; targpd++)
	{
		int hits = hmd_point_counts[targpd * 2 + 0];
		if (hits > hmd_point_counts[targpd * 2 + 1]) hits = hmd_point_counts[targpd * 2 + 1];
		//Need an X and a Y lock.  

		if (hits > maxhits) { maxhits = hits; best_hmd_target = targpd; }
	}
	if (maxhits < MIN_HITS_FOR_VALID)
	{
		fprintf(stderr, "Error: Not enough data for a primary fix.\n");
	}

	return 0;
}


int main(int argc, char ** argv)
{
	if (argc != 3)
	{
		fprintf(stderr, "Error: usage: lighthousefind-torus [camera (L or R)] [datafile]\n");
		exit(-1);
	}

	//Load either 'L' (LH1) or 'R' (LH2) data.
	if (LoadData(argv[1][0], argv[2])) return 5;

	runTheNumbers();

	return 0;
}
