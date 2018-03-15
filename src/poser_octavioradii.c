#include <survive.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct
{
#define OLD_ANGLES_BUFF_LEN 3
	FLT oldAngles[SENSORS_PER_OBJECT][2][NUM_LIGHTHOUSES][OLD_ANGLES_BUFF_LEN]; // sensor, sweep axis, lighthouse, instance
	int angleIndex[NUM_LIGHTHOUSES][2]; // index into circular buffer ahead. separate index for each axis.
	int lastAxis[NUM_LIGHTHOUSES];

	int hitCount[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2];
} OctavioRadiiData;

#include <stdio.h>
#include <stdlib.h>
#include "linmath.h"
#include <string.h>
#include <stdint.h>
#include <math.h>

#define PTS 32
#define MAX_CHECKS 40000
#define MIN_HITS_FOR_VALID 10

FLT hmd_points[PTS * 3];
FLT hmd_norms[PTS * 3];
FLT hmd_point_angles[PTS * 2];
int hmd_point_counts[PTS * 2];
int best_hmd_target = 0;

//Values used for RunTest()
FLT LighthousePos[3] = { 0, 0, 0 };
FLT LighthouseQuat[4] = { 1, 0, 0, 0 };


#define MAX_POINT_PAIRS 100

typedef struct
{
	FLT x;
	FLT y;
	FLT z;
} Point;

typedef struct
{
	Point point; // location of the sensor on the tracked object;
	Point normal; // unit vector indicating the normal for the sensor
	double theta; // "horizontal" angular measurement from lighthouse radians
	double phi; // "vertical" angular measurement from lighthouse in radians.
	int id;
} TrackedSensor;

typedef struct
{
	size_t numSensors;
	TrackedSensor sensor[0];
} TrackedObject;

typedef struct
{
	unsigned char index1;
	unsigned char index2;
	FLT KnownDistance;
} PointPair;

static FLT distance(Point a, Point b)
{
	FLT x = a.x - b.x;
	FLT y = a.y - b.y;
	FLT z = a.z - b.z;
	return FLT_SQRT(x*x + y*y + z*z);
}

typedef struct
{
	FLT HorizAngle;
	FLT VertAngle;
} SensorAngles;

#define SQUARED(x) ((x)*(x))

static FLT calculateFitnessOld(SensorAngles *angles, FLT *radii, PointPair *pairs, size_t numPairs)
{
	FLT fitness = 0;
	for (size_t i = 0; i < numPairs; i++)
	{
		FLT estimatedDistanceBetweenPoints =
			SQUARED(radii[pairs[i].index1])
			+ SQUARED(radii[pairs[i].index2])
			- 2 * radii[pairs[i].index1] * radii[pairs[i].index2]
			* FLT_SIN(angles[pairs[i].index1].HorizAngle) * FLT_SIN(angles[pairs[i].index2].HorizAngle)
			* FLT_COS(angles[pairs[i].index1].VertAngle - angles[pairs[i].index2].VertAngle)
			+ FLT_COS(angles[pairs[i].index1].VertAngle) * FLT_COS(angles[pairs[i].index2].VertAngle);

		fitness += SQUARED(estimatedDistanceBetweenPoints - pairs[i].KnownDistance);
	}

	return FLT_SQRT(fitness);
}

// angles is an array of angles between a sensor pair
// pairs is an array of point pairs
// radii is the guess at the radii of those angles
static FLT calculateFitnessOld2(SensorAngles *angles, FLT *radii, PointPair *pairs, size_t numPairs)
{
	FLT fitness = 0;
	for (size_t i = 0; i < numPairs; i++)
	{
		// These are the vectors that represent the direction for the two points.  
		// TODO: optimize by precomputing the tangent.
		FLT v1[3], v2[3], diff[3];

		v1[0] = 1;
		v2[0] = 1;
		v1[1] = tan(angles[pairs[i].index1].HorizAngle); // can be precomputed
		v2[1] = tan(angles[pairs[i].index2].HorizAngle); // can be precomputed
		v1[2] = tan(angles[pairs[i].index1].VertAngle);  // can be precomputed
		v2[2] = tan(angles[pairs[i].index2].VertAngle);  // can be precomputed

														 // Now, normalize the vectors
		normalize3d(v1, v1); // can be precomputed
		normalize3d(v2, v2); // can be precomputed

							 // Now, given the specified radii, find where the new points are
		scale3d(v1, v1, radii[pairs[i].index1]);
		scale3d(v2, v2, radii[pairs[i].index2]);

		// Cool, now find the vector between these two points
		// TODO: optimize the following two funcs into one.
		sub3d(diff, v1, v2);

		FLT distance = magnitude3d(diff);

		FLT t1 = magnitude3d(v1);
		FLT t2 = magnitude3d(v2);



		FLT estimatedDistanceBetweenPoints =

			SQUARED(radii[pairs[i].index1])
			+ SQUARED(radii[pairs[i].index2])
			- 2 * radii[pairs[i].index1] * radii[pairs[i].index2]
			* FLT_SIN(angles[pairs[i].index1].HorizAngle) * FLT_SIN(angles[pairs[i].index2].HorizAngle)
			* FLT_COS(angles[pairs[i].index1].VertAngle - angles[pairs[i].index2].VertAngle)
			+ FLT_COS(angles[pairs[i].index1].VertAngle) * FLT_COS(angles[pairs[i].index2].VertAngle);


		//fitness += SQUARED(estimatedDistanceBetweenPoints - pairs[i].KnownDistance);
		fitness += SQUARED(distance - pairs[i].KnownDistance);
	}

	return FLT_SQRT(fitness);
}

static FLT angleBetweenSensors(SensorAngles *a, SensorAngles *b)
{
	FLT angle = FLT_ACOS(FLT_COS(a->VertAngle - b->VertAngle)*FLT_COS(a->HorizAngle - b->HorizAngle));
	//FLT angle2 = FLT_ACOS(FLT_COS(b->phi - a->phi)*FLT_COS(b->theta - a->theta));

	return angle;
}

// angles is an array of angles between a sensor pair
// pairs is an array of point pairs
// radii is the guess at the radii of those angles
static FLT calculateFitness(SensorAngles *angles, FLT *radii, PointPair *pairs, size_t numPairs)
{
	FLT fitness = 0;
	for (size_t i = 0; i < numPairs; i++)
	{

		FLT angle = angleBetweenSensors(&angles[pairs[i].index1], &angles[pairs[i].index2]);

		// now we have a side-angle-side triangle, and we need to find the third side.

		// The Law of Cosines says: a^2 = b^2 + c^2 ? 2bc * cosA, 
		// where A is the angle opposite side a.

		// Transform this to:
		// a = sqrt(b^2 + c^2 - 2bc * cosA) and we know the length of the missing side!

		FLT b2 = (SQUARED(radii[pairs[i].index1]));
		FLT c2 = (SQUARED(radii[pairs[i].index2]));
		FLT bc2 = (2 * radii[pairs[i].index1] * radii[pairs[i].index2]);
		FLT cosA = (FLT_COS(angle));

		FLT angleInDegrees = angle * 180 / LINMATHPI;

		FLT dist = sqrt( (SQUARED(radii[pairs[i].index1])) + 
			             (SQUARED(radii[pairs[i].index2])) - 
			             (	(2 * radii[pairs[i].index1] * radii[pairs[i].index2]) *
							(FLT_COS(angle))));


		FLT fitnessAdder = SQUARED(dist - pairs[i].KnownDistance);

		if (isnan(fitnessAdder))
		{
			int a = 0;
		}

		//printf("  %2d %f\n", i, fitnessAdder);

		//fitness += SQUARED(estimatedDistanceBetweenPoints - pairs[i].KnownDistance);
		fitness += SQUARED(dist - pairs[i].KnownDistance);
	}

	//fitness = 1 / fitness;
	return FLT_SQRT(fitness);
}

#define MAX_RADII 32

// note gradientOut will be of the same degree as numRadii
static void getGradient(FLT *gradientOut, SensorAngles *angles, FLT *radii, size_t numRadii, PointPair *pairs, size_t numPairs, const FLT precision)
{
	FLT baseline = calculateFitness(angles, radii, pairs, numPairs);

	for (size_t i = 0; i < numRadii; i++)
	{
		FLT tmpPlus[MAX_RADII];
		memcpy(tmpPlus, radii, sizeof(*radii) * numRadii);
		tmpPlus[i] += precision;
		gradientOut[i] = -(calculateFitness(angles, tmpPlus, pairs, numPairs) - baseline);
	}

	return;
}

static void normalizeAndMultiplyVector(FLT *vectorToNormalize, size_t count, FLT desiredMagnitude)
{
	FLT distanceIn = 0;

	for (size_t i = 0; i < count; i++)
	{
		distanceIn += SQUARED(vectorToNormalize[i]);
	}
	distanceIn = FLT_SQRT(distanceIn);


	FLT scale = desiredMagnitude / distanceIn;

	for (size_t i = 0; i < count; i++)
	{
		vectorToNormalize[i] *= scale;
	}

	return;
}


static void RefineEstimateUsingGradientDescentRadii(FLT *estimateOut, SensorAngles *angles, FLT *initialEstimate, size_t numRadii, PointPair *pairs, size_t numPairs, FILE *logFile)
{
	int i = 0;
	FLT lastMatchFitness = calculateFitness(angles, initialEstimate, pairs, numPairs);
	if (estimateOut != initialEstimate)
	{
		memcpy(estimateOut, initialEstimate, sizeof(*estimateOut) * numRadii);
	}


	// The values below are somewhat magic, and definitely tunable
	// The initial vlue of g will represent the biggest step that the gradient descent can take at first.
	//   bigger values may be faster, especially when the initial guess is wildly off.
	//   The downside to a bigger starting guess is that if we've picked a good guess at the local minima
	//   if there are other local minima, we may accidentally jump to such a local minima and get stuck there.
	//   That's fairly unlikely with the lighthouse problem, from expereince.
	//   The other downside is that if it's too big, we may have to spend a few iterations before it gets down
	//   to a size that doesn't jump us out of our minima.
	// The terminal value of g represents how close we want to get to the local minima before we're "done"
	// The change in value of g for each iteration is intentionally very close to 1.
	//   in fact, it probably could probably be 1 without any issue.  The main place where g is decremented
	//   is in the block below when we've made a jump that results in a worse fitness than we're starting at.
	//   In those cases, we don't take the jump, and instead lower the value of g and try again.
	for (FLT g = 0.4; g > 0.00001; g *= 0.9999)
	{
		i++;



		FLT point1[MAX_RADII];
		memcpy(point1, estimateOut, sizeof(*point1) * numRadii);

		// let's get 3 iterations of gradient descent here.
		FLT gradient1[MAX_RADII];
		getGradient(gradient1, angles, point1, numRadii, pairs, numPairs, g / 1000 /*somewhat arbitrary*/);
		normalizeAndMultiplyVector(gradient1, numRadii, g);

		FLT point2[MAX_RADII];
		for (size_t i = 0; i < numRadii; i++)
		{
			point2[i] = point1[i] + gradient1[i];
		}
		FLT gradient2[MAX_RADII];
		getGradient(gradient2, angles, point2, numRadii, pairs, numPairs, g / 1000 /*somewhat arbitrary*/);
		normalizeAndMultiplyVector(gradient2, numRadii, g);

		FLT point3[MAX_RADII];
		for (size_t i = 0; i < numRadii; i++)
		{
			point3[i] = point2[i] + gradient2[i];
		}

		// remember that gradient descent has a tendency to zig-zag when it encounters a narrow valley?
		// Well, solving the lighthouse problem presents a very narrow valley, and the zig-zag of a basic
		// gradient descent is kinda horrible here.  Instead, think about the shape that a zig-zagging 
		// converging gradient descent makes.  Instead of using the gradient as the best indicator of 
		// the direction we should follow, we're looking at one side of the zig-zag pattern, and specifically
		// following *that* vector.  As it turns out, this works *amazingly* well.  

		FLT specialGradient[MAX_RADII];
		for (size_t i = 0; i < numRadii; i++)
		{
			specialGradient[i] = point3[i] - point1[i];
		}

		// The second parameter to this function is very much a tunable parameter.  Different values will result
		// in a different number of iterations before we get to the minimum.  Numbers between 3-10 seem to work well
		// It's not clear what would be optimum here.
		normalizeAndMultiplyVector(specialGradient, numRadii, g / 4);


		FLT point4[MAX_RADII];
		for (size_t i = 0; i < numRadii; i++)
		{
			point4[i] = point3[i] + specialGradient[i];
		}


		FLT newMatchFitness = calculateFitness(angles, point4, pairs, numPairs);


		if (newMatchFitness < lastMatchFitness)
		{
			//if (logFile)
			//{
			//	writePoint(logFile, lastPoint.x, lastPoint.y, lastPoint.z, 0xFFFFFF);
			//}

			lastMatchFitness = newMatchFitness;
			memcpy(estimateOut, point4, sizeof(*estimateOut) * numRadii);

#ifdef RADII_DEBUG
			printf("+ %d %0.9f (%0.9f) \n", i, newMatchFitness, g);
#endif
			g = g * 1.05;
		}
		else
		{
//#ifdef RADII_DEBUG
			//			printf("-");
			//printf("- %d %0.9f (%0.9f) [%0.9f] \n", i, newMatchFitness, g, estimateOut[0]);
//#endif
			// if it wasn't a match, back off on the distance we jump
			g *= 0.7;

		}

#ifdef RADII_DEBUG
		FLT avg = 0;
		FLT diffFromAvg[MAX_RADII];

		for (size_t m = 0; m < numRadii; m++)
		{
			avg += estimateOut[m];
		}
		avg = avg / numRadii;

		for (size_t m = 0; m < numRadii; m++)
		{
			diffFromAvg[m] = estimateOut[m] - avg;;
		}
		printf("[avg:%f] ", avg);

		for (size_t x = 0; x < numRadii; x++)
		{
			printf("%f, ", diffFromAvg[x]);
			//printf("%f, ", estimateOut[x]);
		}
		printf("\n");


#endif


	}
	printf(" i=%d ", i);
}

static void SolveForLighthouseRadii(Point *objPosition, FLT *objOrientation, TrackedObject *obj)
{
	FLT estimate[MAX_RADII];

	for (size_t i = 0; i < MAX_RADII; i++)
	{
		estimate[i] = 2.38;
	}


	//for (int i=0; i < obj->numSensors; i++)
	//{
	//	printf("%d, ", obj->sensor[i].id);
	//}

	SensorAngles angles[MAX_RADII];
	PointPair pairs[MAX_POINT_PAIRS];

	size_t pairCount = 0;

	//obj->numSensors = 5; // TODO: HACK!!!!

	for (size_t i = 0; i < obj->numSensors; i++)
	{
		angles[i].HorizAngle = obj->sensor[i].theta;
		angles[i].VertAngle = obj->sensor[i].phi;
	}

	for (unsigned char i = 0; i < obj->numSensors - 1; i++)
	{
		for (unsigned char j = i + 1; j < obj->numSensors; j++)
		{
			pairs[pairCount].index1 = i;
			pairs[pairCount].index2 = j;
			pairs[pairCount].KnownDistance = distance(obj->sensor[i].point, obj->sensor[j].point);
			pairCount++;
		}
	}


	RefineEstimateUsingGradientDescentRadii(estimate, angles, estimate, obj->numSensors, pairs, pairCount, NULL);

	// we should now have an estimate of the radii.

	//for (int i = 0; i < obj->numSensors; i++)
	for (int i = 0; i < 1; i++)
	{
		printf("radius[%d]: %f\n", i, estimate[i]);
	}

	// (FLT *estimateOut, SensorAngles *angles, FLT *initialEstimate, size_t numRadii, PointPair *pairs, size_t numPairs, FILE *logFile)

	return;
}

static void QuickPose(SurviveObject *so)
{
	OctavioRadiiData * td = so->PoserData;


	//for (int i=0; i < so->sensor_ct; i++)
	//{
	//	FLT x0=td->oldAngles[i][0][0][td->angleIndex[0][0]];
	//	FLT y0=td->oldAngles[i][1][0][td->angleIndex[0][1]];
	//	//FLT x1=td->oldAngles[i][0][1][td->angleIndex[1][0]];
	//	//FLT y1=td->oldAngles[i][1][1][td->angleIndex[1][1]];
	//	//printf("%2d: %8.8f, %8.8f   %8.8f, %8.8f   \n", 
	//	//	i,
	//	//	x0,
	//	//	y0,
	//	//	x1,
	//	//	y1
	//	//	);
	//	printf("%2d: %8.8f, %8.8f   \n", 
	//		i,
	//		x0,
	//		y0
	//		);
	//}
	//printf("\n");

	TrackedObject *to;

	to = malloc(sizeof(TrackedObject) + (SENSORS_PER_OBJECT * sizeof(TrackedSensor)));

	{
		int sensorCount = 0;

		for (int i = 0; i < so->sensor_ct; i++)
		{
			int lh = 0;
			//printf("%d[%d], ",i,td->hitCount[i][lh][0]);

			int angleIndex0 = (td->angleIndex[lh][0] + 1 + OLD_ANGLES_BUFF_LEN) % OLD_ANGLES_BUFF_LEN;
			int angleIndex1 = (td->angleIndex[lh][1] + 1 + OLD_ANGLES_BUFF_LEN) % OLD_ANGLES_BUFF_LEN;
			if ((td->oldAngles[i][0][lh][angleIndex0] != 0 && td->oldAngles[i][1][lh][angleIndex1] != 0))
				
				
			{
				if (td->hitCount[i][lh][0] > 10 && td->hitCount[i][lh][1] > 10)
				{
					FLT norm[3] = { so->sensor_normals[i * 3 + 0] , so->sensor_normals[i * 3 + 1] , so->sensor_normals[i * 3 + 2] };
					FLT point[3] = { so->sensor_locations[i * 3 + 0] , so->sensor_locations[i * 3 + 1] , so->sensor_locations[i * 3 + 2] };

					to->sensor[sensorCount].normal.x = norm[0];
					to->sensor[sensorCount].normal.y = norm[1];
					to->sensor[sensorCount].normal.z = norm[2];
					to->sensor[sensorCount].point.x = point[0];
					to->sensor[sensorCount].point.y = point[1];
					to->sensor[sensorCount].point.z = point[2];
					to->sensor[sensorCount].theta = td->oldAngles[i][0][lh][angleIndex0] + LINMATHPI / 2; // lighthouse 0, angle 0 (horizontal)
					to->sensor[sensorCount].phi = td->oldAngles[i][1][lh][angleIndex1] + LINMATHPI / 2; // lighthouse 0, angle 1 (vertical)
					to->sensor[sensorCount].id=i;



					//printf("%2d: %8.8f, %8.8f   \n", 
					//	i,
					//	to->sensor[sensorCount].theta,
					//	to->sensor[sensorCount].phi
					//	);

					sensorCount++;
				}
			}
		}
		//printf("\n");
		to->numSensors = sensorCount;

		if (sensorCount > 4)
		{
			Point pos;
			FLT orient[4];
			SolveForLighthouseRadii(&pos, orient, to);
		}

		
	}


	free(to);

}


int PoserOctavioRadii( SurviveObject * so, PoserData * pd )
{
	PoserType pt = pd->pt;
	SurviveContext * ctx = so->ctx;
	OctavioRadiiData * dd = so->PoserData;

	if( !dd )
	{
		so->PoserData = dd = malloc( sizeof(OctavioRadiiData) );
		memset(dd, 0, sizeof(OctavioRadiiData));
	}


	switch( pt )
	{
	case POSERDATA_IMU:
	{
		PoserDataIMU * imu = (PoserDataIMU*)pd;
		//printf( "IMU:%s (%f %f %f) (%f %f %f)\n", so->codename, imu->accel[0], imu->accel[1], imu->accel[2], imu->gyro[0], imu->gyro[1], imu->gyro[2] );
		break;
	}
	case POSERDATA_LIGHT:
	{
		PoserDataLight * l = (PoserDataLight*)pd;

		if (l->lh >= NUM_LIGHTHOUSES || l->lh < 0)
		{
			// should never happen.  Famous last words...
			break;
		}
		int axis = l->acode & 0x1;

		//printf("%d ", l->sensor_id);


		//printf( "LIG:%s %d @ %f rad, %f s (AC %d) (TC %d)\n", so->codename, l->sensor_id, l->angle, l->length, l->acode, l->timecode );
		if ((dd->lastAxis[l->lh] != (l->acode & 0x1)) )
		{
			int lastAxis = dd->lastAxis[l->lh];
	//printf("\n");
			//if (0 == l->lh)
			//	printf("or[%d,%d] ", l->lh,lastAxis);

			for (int i=0; i < SENSORS_PER_OBJECT; i++)
			{
	//FLT oldAngles[SENSORS_PER_OBJECT][2][NUM_LIGHTHOUSES][OLD_ANGLES_BUFF_LEN]; // sensor, sweep axis, lighthouse, instance
				int index = dd->angleIndex[l->lh][axis];
				if (dd->oldAngles[i][axis][l->lh][dd->angleIndex[l->lh][axis]] != 0)
				{
					//if (0 == l->lh)
					//	printf("%d ", i);

					dd->hitCount[i][l->lh][axis]++;
				}
				else
				{
					dd->hitCount[i][l->lh][axis] = (int)((double)dd->hitCount[i][l->lh][axis] * 0.5);
				}
			}
			//if (0 == l->lh)
			//	printf("\n");
			//int foo = l->acode & 0x1;
			//printf("%d", foo);


			//if (axis)
			{
				if (0 == l->lh && axis) // only once per full cycle...
				{
					static unsigned int counter = 1;

					counter++;

					// let's just do this occasionally for now...
					if (counter % 4 == 0)
						QuickPose(so);
				}
				// axis changed, time to increment the circular buffer index.


				dd->angleIndex[l->lh][axis]++;
				dd->angleIndex[l->lh][axis] = dd->angleIndex[l->lh][axis] % OLD_ANGLES_BUFF_LEN;

				// and clear out the data.
				for (int i=0; i < SENSORS_PER_OBJECT; i++)
				{
					dd->oldAngles[i][axis][l->lh][dd->angleIndex[l->lh][axis]] = 0;
				}

			}
			dd->lastAxis[l->lh] = axis;
		}

		//if (0 == l->lh)
		//	printf("(%d) ", l->sensor_id);

	//FLT oldAngles[SENSORS_PER_OBJECT][2][NUM_LIGHTHOUSES][OLD_ANGLES_BUFF_LEN]; // sensor, sweep axis, lighthouse, instance
		dd->oldAngles[l->sensor_id][axis][l->lh][dd->angleIndex[l->lh][axis]] = l->angle;
		break;	}
	case POSERDATA_FULL_SCENE:
	{
		TrackedObject *to;

		PoserDataFullScene * fs = (PoserDataFullScene*)pd;

		to = malloc(sizeof(TrackedObject) + (SENSORS_PER_OBJECT * sizeof(TrackedSensor)));

		//FLT  lengths[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2];
		//FLT  angles[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2];  //2 Axes  (Angles in LH space)
		//FLT  synctimes[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES];

		//to->numSensors = so->sensor_ct;
		{
			int sensorCount = 0;

			for (int i = 0; i < so->sensor_ct; i++)
			{
				if (fs->lengths[i][0][0] != -1 && fs->lengths[i][0][1] != -1) //lh 0
				{
					to->sensor[sensorCount].normal.x = so->sensor_normals[i * 3 + 0];
					to->sensor[sensorCount].normal.y = so->sensor_normals[i * 3 + 1];
					to->sensor[sensorCount].normal.z = so->sensor_normals[i * 3 + 2];
					to->sensor[sensorCount].point.x = so->sensor_locations[i * 3 + 0];
					to->sensor[sensorCount].point.y = so->sensor_locations[i * 3 + 1];
					to->sensor[sensorCount].point.z = so->sensor_locations[i * 3 + 2];
					to->sensor[sensorCount].theta = fs->angles[i][0][0] + LINMATHPI / 2; // lighthouse 0, angle 0 (horizontal)
					to->sensor[sensorCount].phi = fs->angles[i][0][1] + LINMATHPI / 2; // lighthosue 0, angle 1 (vertical)
					to->sensor[sensorCount].id=i;
					sensorCount++;
				}
			}

			to->numSensors = sensorCount;

			Point position;
			FLT orientation[4];

			SolveForLighthouseRadii(&position, orientation, to);
		}
		{
			int sensorCount = 0;
			int lh = 1;

			for (int i = 0; i < so->sensor_ct; i++)
			{
				if (fs->lengths[i][lh][0] != -1 && fs->lengths[i][lh][1] != -1)
				{
					to->sensor[sensorCount].normal.x = so->sensor_normals[i * 3 + 0];
					to->sensor[sensorCount].normal.y = so->sensor_normals[i * 3 + 1];
					to->sensor[sensorCount].normal.z = so->sensor_normals[i * 3 + 2];
					to->sensor[sensorCount].point.x = so->sensor_locations[i * 3 + 0];
					to->sensor[sensorCount].point.y = so->sensor_locations[i * 3 + 1];
					to->sensor[sensorCount].point.z = so->sensor_locations[i * 3 + 2];
					to->sensor[sensorCount].theta = fs->angles[i][lh][0] + LINMATHPI / 2; // lighthouse 0, angle 0 (horizontal)
					to->sensor[sensorCount].phi = fs->angles[i][lh][1] + LINMATHPI / 2; // lighthosue 0, angle 1 (vertical)
					to->sensor[sensorCount].id=i;
					sensorCount++;
				}
			}

			to->numSensors = sensorCount;

			Point position;
			FLT orientation[4];

			SolveForLighthouseRadii(&position, orientation, to);
		}
		//printf( "Full scene data.\n" );
		break;
	}
	case POSERDATA_DISASSOCIATE:
	{
		free( dd );
		so->PoserData = 0;
		//printf( "Need to disassociate.\n" );
		break;
	}
	}
	return 0;
}


REGISTER_LINKTIME( PoserOctavioRadii );

