#include <memory.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include "linmath.h"
#include "tori_includes.h"
#include "visualization.h"


static double distance(Point a, Point b)
{
	double x = a.x - b.x;
	double y = a.y - b.y;
	double z = a.z - b.z;
	return sqrt(x*x + y*y + z*z);
}

Matrix3x3 GetRotationMatrixForTorus(Point a, Point b)
{
	Matrix3x3 result;
	FLT v1[3] = { 0, 0, 1 };
	FLT v2[3] = { a.x - b.x, a.y - b.y, a.z - b.z };

	normalize3d(v2,v2);

	rotation_between_vecs_to_m3(&result, v1, v2);

	// Useful for debugging...
	FLT v2b[3];
	rotate_vec(v2b, v1, result);

	return result;
}

//void TestGetRotationMatrixForTorus()
//{
//	// a={x=0.079967096447944641 y=0.045225199311971664 z=0.034787099808454514 }
//	// b={x=0.085181400179862976 y=0.017062099650502205 z=0.046403601765632629 }
//
//	// a={x=0.050822000950574875 y=0.052537899464368820 z=0.033285100013017654 }
//	// b={x=0.085181400179862976 y=0.017062099650502205 z=0.046403601765632629 }
//
//}

Point RotateAndTranslatePoint(Point p, Matrix3x3 rot, Point newOrigin)
{
	Point q;

	double pf[3] = { p.x, p.y, p.z };
	q.x = rot.val[0][0] * p.x + rot.val[1][0] * p.y + rot.val[2][0] * p.z + newOrigin.x;
	q.y = rot.val[0][1] * p.x + rot.val[1][1] * p.y + rot.val[2][1] * p.z + newOrigin.y;
	q.z = rot.val[0][2] * p.x + rot.val[1][2] * p.y + rot.val[2][2] * p.z + newOrigin.z;

	return q;
}

double angleFromPoints(Point p1, Point p2, Point center)
{
	Point v1, v2, v1norm, v2norm;
	v1.x = p1.x - center.x;
	v1.y = p1.y - center.y;
	v1.z = p1.z - center.z;

	v2.x = p2.x - center.x;
	v2.y = p2.y - center.y;
	v2.z = p2.z - center.z;

	double v1mag = sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
	v1norm.x = v1.x / v1mag;
	v1norm.y = v1.y / v1mag;
	v1norm.z = v1.z / v1mag;

	double v2mag = sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);
	v2norm.x = v2.x / v2mag;
	v2norm.y = v2.y / v2mag;
	v2norm.z = v2.z / v2mag;

	double res = v1norm.x * v2norm.x + v1norm.y * v2norm.y + v1norm.z * v2norm.z;

	double angle = acos(res);

	return angle;
}

Point midpoint(Point a, Point b)
{
	Point m;
	m.x = (a.x + b.x) / 2;
	m.y = (a.y + b.y) / 2;
	m.z = (a.z + b.z) / 2;

	return m;
}




// This torus generator creates a point cloud of the given torus, and attempts to keep the 
// density of the points fairly uniform across the surface of the torus.
void partialTorusGenerator(
	Point p1,
	Point p2,
	double toroidalStartAngle,
	double toroidalEndAngle,
	double poloidalStartAngle,
	double poloidalEndAngle,
	double lighthouseAngle,
	double toroidalPrecision,
	Point **pointCloud)
{
	double poloidalRadius = 0;
	double toroidalRadius = 0;

	Point m = midpoint(p1, p2);
	double distanceBetweenPoints = distance(p1, p2);

	// ideally should only need to be lighthouseAngle, but increasing it here keeps us from accidentally
	// thinking the tori converge at the location of the tracked object instead of at the lighthouse.
	double centralAngleToIgnore = lighthouseAngle * 3;

	Matrix3x3 rot = GetRotationMatrixForTorus(p1, p2);

	toroidalRadius = distanceBetweenPoints / (2 * tan(lighthouseAngle));

	poloidalRadius = sqrt(pow(toroidalRadius, 2) + pow(distanceBetweenPoints / 2, 2));

	double poloidalPrecision = M_PI * 2 / toroidalPrecision;


	unsigned int pointCount = 0;

	// This loop tries to (cheaply) figure out an upper bound on the number of points we'll have in our point cloud
	for (double poloidalStep = poloidalStartAngle; poloidalStep < poloidalEndAngle; poloidalStep += poloidalPrecision)
	{
		// here, we specify the number of steps that will occur on the toroidal circle for a given poloidal angle
		// We do this so our point cloud will have a more even distribution of points across the surface of the torus.
		double steps = (cos(poloidalStep) + 1) / 2 * toroidalPrecision;

		double step_distance = 2 * M_PI / steps;

		pointCount += (unsigned int)((toroidalEndAngle - toroidalStartAngle) / step_distance + 2);
	}

	*pointCloud = malloc(pointCount * sizeof(Point) );

	assert(0 != *pointCloud);

	(*pointCloud)[pointCount - 1].x = -1000;
	(*pointCloud)[pointCount - 1].y = -1000;
	(*pointCloud)[pointCount - 1].z = -1000; // need a better magic number or flag, but this'll do for now.

	size_t currentPoint = 0;

	for (double poloidalStep = poloidalStartAngle; poloidalStep < poloidalEndAngle; poloidalStep += poloidalPrecision)
	{
		// here, we specify the number of steps that will occur on the toroidal circle for a given poloidal angle
		// We do this so our point cloud will have a more even distribution of points across the surface of the torus.
		double steps = (cos(poloidalStep) + 1) / 2 * toroidalPrecision;

		double step_distance = 2 * M_PI / steps;

		//for (double toroidalStep = toroidalStartAngle; toroidalStep < toroidalEndAngle; toroidalStep += M_PI / 40)
		for (double toroidalStep = toroidalStartAngle; toroidalStep < toroidalEndAngle; toroidalStep += step_distance)
		{
			if (currentPoint >= pointCount - 1)
			{
				int a = 0;
			}
			assert(currentPoint < pointCount - 1);
			(*pointCloud)[currentPoint].x = (toroidalRadius + poloidalRadius*cos(poloidalStep))*cos(toroidalStep);
			(*pointCloud)[currentPoint].y = (toroidalRadius + poloidalRadius*cos(poloidalStep))*sin(toroidalStep);
			(*pointCloud)[currentPoint].z = poloidalRadius*sin(poloidalStep);
			(*pointCloud)[currentPoint] = RotateAndTranslatePoint((*pointCloud)[currentPoint], rot, m);

			// TODO: HACK!!! Instead of doing anything with normals, we're "assuming" that all sensors point directly up
			// and hence we know that nothing with a negative z value is a possible lightouse location.
			// Before this code can go live, we'll have to take the normals into account and remove this hack.
			if ((*pointCloud)[currentPoint].z > 0)
			{
				currentPoint++;
			}
		}
	}

#ifdef TORI_DEBUG
	printf("%d / %d\n", currentPoint, pointCount);
#endif
	(*pointCloud)[currentPoint].x = -1000;
	(*pointCloud)[currentPoint].y = -1000;
	(*pointCloud)[currentPoint].z = -1000;
}

void torusGenerator(Point p1, Point p2, double lighthouseAngle, Point **pointCloud)
{

	double centralAngleToIgnore = lighthouseAngle * 6;

	centralAngleToIgnore = 20.0 / 180.0 * M_PI;

	partialTorusGenerator(p1, p2, 0, M_PI * 2, centralAngleToIgnore + M_PI, M_PI * 3 - centralAngleToIgnore, lighthouseAngle, DefaultPointsPerOuterDiameter, pointCloud);

	return;
}


// What we're doing here is:
// * Given a point in space
// * And points and a lighthouse angle that implicitly define a torus
// * for that torus, what is the toroidal angle of the plane that will go through that point in space
// * and given that toroidal angle, what is the poloidal angle that will be directed toward that point in space?
//
// Given the toroidal and poloidal angles of a "good estimate" of a solution position, a caller of this function
// will be able to "draw" the point cloud of a torus in just the surface of the torus near the point in space.
// That way, the caller doesn't have to draw the entire torus in high resolution, just the part of the torus
// that is most likely to contain the best solution.
void estimateToroidalAndPoloidalAngleOfPoint(
	Point torusP1,
	Point torusP2,
	double lighthouseAngle,
	Point point,
	double *toroidalAngle,
	double *poloidalAngle)
{
	// this is the rotation matrix that shows how to rotate the torus from being in a simple "default" orientation
	// into the coordinate system of the tracked object
	Matrix3x3 rot = GetRotationMatrixForTorus(torusP1, torusP2);

	// We take the inverse of the rotation matrix, and this now defines a rotation matrix that will take us from
	// the tracked object coordinate system into the "easy" or "default" coordinate system of the torus.
	// Using this will allow us to derive angles much more simply by being in a "friendly" coordinate system.
	rot = inverseM33(rot);
	Point origin;
	origin.x = 0;
	origin.y = 0;
	origin.z = 0;

	Point m = midpoint(torusP1, torusP2);

	// in this new coordinate system, we'll rename all of the points we care about to have an "F" after them
	// This will be their representation in the "friendly" coordinate system
	Point pointF;

	// Okay, I lied a little above.  In addition to the rotation matrix that we care about, there was also
	// a translation that we did to move the origin.  If we're going to get to the "friendly" coordinate system
	// of the torus, we need to first undo the translation, then undo the rotation.  Below, we're undoing the translation.
	pointF.x = point.x - m.x;
	pointF.y = point.y - m.y;
	pointF.z = point.z - m.z;

	// now we'll undo the rotation part.
	pointF = RotateAndTranslatePoint(pointF, rot, origin);

	// hooray, now pointF is in our more-friendly coordinate system.  

	// Now, it's time to figure out the toroidal angle to that point.  This should be pretty easy. 
	// We will "flatten" the z dimension to only look at the x and y values.  Then, we just need to measure the 
	// angle between a vector to pointF and a vector along the x axis.  

	*toroidalAngle = atan(pointF.y / pointF.x);
	if (pointF.x < 0)
	{
		*toroidalAngle += M_PI;
	}

	// SCORE!! We've got the toroidal angle.  We're half done!

	// Okay, what next...?  Now, we will need to rotate the torus *again* to make it easy to
	// figure out the poloidal angle.  We should rotate the entire torus by the toroidal angle
	// so that the point we're focusin on will lie on the x/z plane.  We then should translate the
	// torus so that the center of the poloidal circle is at the origin.  At that point, it will
	// be trivial to determine the poloidal angle-- it will be the angle on the xz plane of a 
	// vector from the origin to the point.

	// okay, instead of rotating the torus & point by the toroidal angle to get the point on
	// the xz plane, we're going to take advantage of the radial symmetry of the torus
	// (i.e. it's symmetric about the point we'd want to rotate it, so the rotation wouldn't 
	// change the torus at all).  Therefore, we'll leave the torus as is, but we'll rotate the point
	// This will only impact the x and y coordinates, and we'll use "G" as the postfix to represent
	// this new coordinate system

	Point pointG;
	pointG.z = pointF.z;
	pointG.y = 0;
	pointG.x = sqrt(SQUARED(pointF.x) + SQUARED(pointF.y));

	// okay, that ended up being easier than I expected.  Now that we have the point on the xZ plane,
	// our next step will be to shift it down so that the center of the poloidal circle is at the origin.
	// As you may have noticed, y has now gone to zero, and from here on out, we can basically treat
	// this as a 2D problem.  I think we're getting close...

	// I stole these lines from the torus generator.  Gonna need the poloidal radius.
	double distanceBetweenPoints = distance(torusP1, torusP2); // we don't care about the coordinate system of these points because we're just getting distance.
	double toroidalRadius = distanceBetweenPoints / (2 * tan(lighthouseAngle));
	double poloidalRadius = sqrt(pow(toroidalRadius, 2) + pow(distanceBetweenPoints / 2, 2));

	// The center of the polidal circle already lies on the z axis at this point, so we won't shift z at all. 
	// The shift along the X axis will be the toroidal radius.  

	Point pointH;
	pointH.z = pointG.z;
	pointH.y = pointG.y;
	pointH.x = pointG.x - toroidalRadius;

	// Okay, almost there.  If we treat pointH as a vector on the XZ plane, if we get its angle,
	// that will be the poloidal angle we're looking for.  (crosses fingers)

	*poloidalAngle = atan(pointH.z / pointH.x);
	if (pointH.x < 0)
	{
		*poloidalAngle += M_PI;
	}

	// Wow, that ended up being not so much code, but a lot of interesting trig.
	// can't remember the last time I spent so much time working through each line of code.

	return;
}

double FindSmallestDistance(Point p, Point* cloud)
{
	Point *cp = cloud;
	double smallestDistance = 10000000000000.0;

	while (cp->x != -1000 || cp->y != -1000 || cp->z != -1000)
	{
		double distance = (SQUARED(cp->x - p.x) + SQUARED(cp->y - p.y) + SQUARED(cp->z - p.z));
		if (distance < smallestDistance)
		{
			smallestDistance = distance;
		}
		cp++;
	}
	smallestDistance = sqrt(smallestDistance);
	return smallestDistance;
}

// Given a cloud and a list of clouds, find the point on masterCloud that best matches clouds.
Point findBestPointMatch(Point *masterCloud, Point** clouds, int numClouds)
{

	Point bestMatch = { 0 };
	double bestDistance = 10000000000000.0;
	Point *cp = masterCloud;
	int point = 0;
	while (cp->x != -1000 || cp->y != -1000 || cp->z != -1000)
	{
		point++;
#ifdef TORI_DEBUG
		if (point % 100 == 0)
		{
			printf(".");
		}
#endif
		double currentDistance = 0;
		for (int i = 0; i < numClouds; i++)
		{
			if (clouds[i] == masterCloud)
			{
				continue;
			}
			Point* cloud = clouds[i];
			currentDistance += FindSmallestDistance(*cp, cloud);
		}

		if (currentDistance < bestDistance)
		{
			bestDistance = currentDistance;
			bestMatch = *cp;
		}
		cp++;
	}

	return bestMatch;
}


#define MAX_POINT_PAIRS 100

typedef struct
{
	Point a;
	Point b;
	double angle;
} PointsAndAngle;

double angleBetweenSensors(TrackedSensor *a, TrackedSensor *b)
{
	double angle = acos(cos(a->phi - b->phi)*cos(a->theta - b->theta));
	double angle2 = acos(cos(b->phi - a->phi)*cos(b->theta - a->theta));

	return angle;
}
double pythAngleBetweenSensors2(TrackedSensor *a, TrackedSensor *b)
{
	double p = (a->phi - b->phi);
	double d = (a->theta - b->theta);

	double adjd = sin((a->phi + b->phi) / 2);
	double adjP = sin((a->theta + b->theta) / 2);
	double pythAngle = sqrt(SQUARED(p*adjP) + SQUARED(d*adjd));
	return pythAngle;
}
Point SolveForLighthouse(TrackedObject *obj, char doLogOutput)
{
	PointsAndAngle pna[MAX_POINT_PAIRS];

	size_t pnaCount = 0;
	for (unsigned int i = 0; i < obj->numSensors; i++)
	{
		for (unsigned int j = 0; j < i; j++)
		{
			if (pnaCount < MAX_POINT_PAIRS)
			{
				pna[pnaCount].a = obj->sensor[i].point;
				pna[pnaCount].b = obj->sensor[j].point;

				pna[pnaCount].angle = pythAngleBetweenSensors2(&obj->sensor[i], &obj->sensor[j]);

				double pythAngle = sqrt(SQUARED(obj->sensor[i].phi - obj->sensor[j].phi) + SQUARED(obj->sensor[i].theta - obj->sensor[j].theta));

				pnaCount++;
			}
		}
	}

	Point **pointCloud = malloc(sizeof(void*)* pnaCount);

	FILE *f = NULL;
	if (doLogOutput)
	{
		f = fopen("pointcloud2.pcd", "wb");
		writePcdHeader(f);
		writeAxes(f);
	}

	for (unsigned int i = 0; i < pnaCount; i++)
	{
		torusGenerator(pna[i].a, pna[i].b, pna[i].angle, &(pointCloud[i]));
		if (doLogOutput)
		{
			writePointCloud(f, pointCloud[i], COLORS[i%MAX_COLORS]);
		}

	}

	Point bestMatchA = findBestPointMatch(pointCloud[0], pointCloud, pnaCount);

	for (unsigned int i = 0; i < pnaCount; i++)
	{
		free(pointCloud[i]);
		pointCloud[i] = NULL;
	}

	if (doLogOutput)
	{
		markPointWithStar(f, bestMatchA, 0xFF0000);
	}
#ifdef TORI_DEBUG
	printf("(%f,%f,%f)\n", bestMatchA.x, bestMatchA.y, bestMatchA.z);
#endif
	// Now, let's add an extra patch or torus near the point we just found.

	double toroidalAngle = 0;
	double poloidalAngle = 0;



	Point **pointCloud2 = malloc(sizeof(void*)* pnaCount);

	for (unsigned int i = 0; i < pnaCount; i++)
	{
		estimateToroidalAndPoloidalAngleOfPoint(
			pna[i].a,
			pna[i].b,
			pna[i].angle,
			bestMatchA,
			&toroidalAngle,
			&poloidalAngle);

		partialTorusGenerator(pna[i].a, pna[i].b, toroidalAngle - 0.1, toroidalAngle + 0.1, poloidalAngle - 0.2, poloidalAngle + 0.2, pna[i].angle, 800, &(pointCloud2[i]));

		if (doLogOutput)
		{
			writePointCloud(f, pointCloud2[i], COLORS[i%MAX_COLORS]);
		}

	}

	Point bestMatchB = findBestPointMatch(pointCloud2[0], pointCloud2, pnaCount);

	for (unsigned int i = 0; i < pnaCount; i++)
	{
		free(pointCloud2[i]);
		pointCloud2[i] = NULL;
	}

	if (doLogOutput)
	{
		markPointWithStar(f, bestMatchB, 0x00FF00);
	}
#ifdef TORI_DEBUG
	printf("(%f,%f,%f)\n", bestMatchB.x, bestMatchB.y, bestMatchB.z);
#endif

	Point **pointCloud3 = malloc(sizeof(void*)* pnaCount);

	for (unsigned int i = 0; i < pnaCount; i++)
	{
		estimateToroidalAndPoloidalAngleOfPoint(
			pna[i].a,
			pna[i].b,
			pna[i].angle,
			bestMatchB,
			&toroidalAngle,
			&poloidalAngle);

		partialTorusGenerator(pna[i].a, pna[i].b, toroidalAngle - 0.05, toroidalAngle + 0.05, poloidalAngle - 0.1, poloidalAngle + 0.1, pna[i].angle, 3000, &(pointCloud3[i]));

		if (doLogOutput)
		{
			writePointCloud(f, pointCloud3[i], COLORS[i%MAX_COLORS]);
		}

	}

	Point bestMatchC = findBestPointMatch(pointCloud3[0], pointCloud3, pnaCount);

	for (unsigned int i = 0; i < pnaCount; i++)
	{
		free(pointCloud3[i]);
		pointCloud3[i] = NULL;
	}

	if (doLogOutput)
	{
		markPointWithStar(f, bestMatchC, 0xFFFFFF);
	}
#ifdef TORI_DEBUG
	printf("(%f,%f,%f)\n", bestMatchC.x, bestMatchC.y, bestMatchC.z);
#endif


	if (doLogOutput)
	{
		updateHeader(f);
		fclose(f);
	}



	return bestMatchC;
}

static Point makeUnitPoint(Point *p)
{
	Point newP;
	double r = sqrt(p->x*p->x + p->y*p->y + p->z*p->z);
	newP.x = p->x / r;
	newP.y = p->y / r;
	newP.z = p->z / r;

	return newP;
}

static double getPhi(Point p)
{
	//	double phi = acos(p.z / (sqrt(p.x*p.x + p.y*p.y + p.z*p.z)));
	//	double phi = atan(sqrt(p.x*p.x + p.y*p.y)/p.z);
	double phi = atan(p.x / p.z);
	return phi;
}

static double getTheta(Point p)
{
	//double theta = atan(p.y / p.x);
	double theta = atan(p.x / p.y);
	return theta;
}

// subtraction
static Point PointSub(Point a, Point b)
{
	Point newPoint;

	newPoint.x = a.x - b.x;
	newPoint.y = a.y - b.y;
	newPoint.z = a.z - b.z;

	return newPoint;
}


