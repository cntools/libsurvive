#include <survive.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <memory.h>
#include <assert.h>
#include "linmath.h"
#include <stddef.h>
#include <math.h>
#include <stdint.h>


#define PointToFlts(x) ((FLT*)(x))

typedef struct
{
	FLT x;
	FLT y;
	FLT z;
} Point;

void writePoint(FILE *file, double x, double y, double z, unsigned int rgb) {}
void updateHeader(FILE * file) {}
void writeAxes(FILE * file) {}
void drawLineBetweenPoints(FILE *file, Point a, Point b, unsigned int color) {}
void writePcdHeader(FILE * file) {}
void writePointCloud(FILE *f, Point *pointCloud, unsigned int Color) {}
void markPointWithStar(FILE *file, Point point, unsigned int color) {}

typedef struct
{
	Point point; // location of the sensor on the tracked object;
	Point normal; // unit vector indicating the normal for the sensor
	double theta; // "horizontal" angular measurement from lighthouse radians
	double phi; // "vertical" angular measurement from lighthouse in radians.
} TrackedSensor;

typedef struct
{
	size_t numSensors;
	TrackedSensor sensor[0];
} TrackedObject;


#ifndef M_PI
#define M_PI 3.14159265358979323846264338327
#endif

#define SQUARED(x) ((x)*(x))

typedef union
{
	struct
	{
		unsigned char Blue;
		unsigned char Green;
		unsigned char Red;
		unsigned char Alpha;
	};
	uint32_t long_value;
} RGBValue;

static RGBValue RED = { .Red = 255,.Green = 0,.Blue = 0,.Alpha = 125 };
static RGBValue GREEN = { .Red = 0,.Green = 255,.Blue = 0,.Alpha = 125 };
static RGBValue BLUE = { .Red = 0,.Green = 0,.Blue = 255,.Alpha = 125 };

static const double WORLD_BOUNDS = 100;
#define MAX_TRACKED_POINTS 40

static const float DefaultPointsPerOuterDiameter = 60;

typedef struct
{
	int something;
	//Stuff
} ToriData;







static FLT distance(Point a, Point b)
{
	FLT x = a.x - b.x;
	FLT y = a.y - b.y;
	FLT z = a.z - b.z;
	return FLT_SQRT(x*x + y*y + z*z);
}

Matrix3x3 GetRotationMatrixForTorus(Point a, Point b)
{
	Matrix3x3 result;
	FLT v1[3] = { 0, 0, 1 };
	FLT v2[3] = { a.x - b.x, a.y - b.y, a.z - b.z };

	normalize3d(v2, v2);

	rotation_between_vecs_to_m3(&result, v1, v2);

	// Useful for debugging...
	//FLT v2b[3];
	//rotate_vec(v2b, v1, result);

	return result;
}

typedef struct
{
	Point a;
	Point b;
	FLT angle;
	FLT tanAngle; // tangent of angle
	Matrix3x3 rotation;
	Matrix3x3 invRotation; // inverse of rotation

} PointsAndAngle;


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

// What we're doing here is:
// * Given a point in space
// * And points and a lighthouse angle that implicitly define a torus
// * for that torus, what is the toroidal angle of the plane that will go through that point in space
// * and given that toroidal angle, what is the poloidal angle that will be directed toward that point in space?
void estimateToroidalAndPoloidalAngleOfPoint(
	PointsAndAngle *pna,
	Point point,
	double *toroidalSin,
	double *toroidalCos,
	double *poloidalAngle,
	double *poloidalSin)
{
	// We take the inverse of the rotation matrix, and this now defines a rotation matrix that will take us from
	// the tracked object coordinate system into the "easy" or "default" coordinate system of the torus.
	// Using this will allow us to derive angles much more simply by being in a "friendly" coordinate system.
	Matrix3x3 rot = pna->invRotation;
	Point origin;
	origin.x = 0;
	origin.y = 0;
	origin.z = 0;

	Point m = midpoint(pna->a, pna->b);

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

	FLT toroidalHyp = FLT_SQRT(SQUARED(pointF.y) + SQUARED(pointF.x));

	*toroidalSin = pointF.y / toroidalHyp;

	*toroidalCos = pointF.x / toroidalHyp;

	//*toroidalAngle = atan(pointF.y / pointF.x);
	//if (pointF.x < 0)
	//{
	//	*toroidalAngle += M_PI;
	//}

	//assert(*toroidalSin / FLT_SIN(*toroidalAngle) - 1 < 0.000001);
	//assert(*toroidalSin / FLT_SIN(*toroidalAngle) - 1 > -0.000001);

	//assert(*toroidalCos / FLT_COS(*toroidalAngle) - 1 < 0.000001);
	//assert(*toroidalCos / FLT_COS(*toroidalAngle) - 1 > -0.000001);

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
	double distanceBetweenPoints = distance(pna->a, pna->b); // we don't care about the coordinate system of these points because we're just getting distance.
	double toroidalRadius = distanceBetweenPoints / (2 * pna->tanAngle);
	double poloidalRadius = sqrt(SQUARED(toroidalRadius) + SQUARED(distanceBetweenPoints / 2));

	// The center of the polidal circle already lies on the z axis at this point, so we won't shift z at all. 
	// The shift along the X axis will be the toroidal radius.  

	Point pointH;
	pointH.z = pointG.z;
	pointH.y = pointG.y;
	pointH.x = pointG.x - toroidalRadius;

	// Okay, almost there.  If we treat pointH as a vector on the XZ plane, if we get its angle,
	// that will be the poloidal angle we're looking for.  (crosses fingers)

	FLT poloidalHyp = FLT_SQRT(SQUARED(pointH.z) + SQUARED(pointH.x));

	*poloidalSin = pointH.z / poloidalHyp;


	*poloidalAngle = atan(pointH.z / pointH.x);
	if (pointH.x < 0)
	{
		*poloidalAngle += M_PI;
	}

	//assert(*toroidalSin / FLT_SIN(*toroidalAngle) - 1 < 0.000001);
	//assert(*toroidalSin / FLT_SIN(*toroidalAngle) - 1 > -0.000001);



	// Wow, that ended up being not so much code, but a lot of interesting trig.
	// can't remember the last time I spent so much time working through each line of code.

	return;
}

#define MAX_POINT_PAIRS 100

FLT angleBetweenSensors(TrackedSensor *a, TrackedSensor *b)
{
	FLT angle = FLT_ACOS(FLT_COS(a->phi - b->phi)*FLT_COS(a->theta - b->theta));
	//FLT angle2 = FLT_ACOS(FLT_COS(b->phi - a->phi)*FLT_COS(b->theta - a->theta));

	return angle;
}

// This provides a pretty good estimate of the angle above, probably better
// the further away the lighthouse is.  But, it's not crazy-precise.
// It's main advantage is speed.
FLT pythAngleBetweenSensors2(TrackedSensor *a, TrackedSensor *b)
{
	FLT p = (a->phi - b->phi);
	FLT d = (a->theta - b->theta);

	FLT adjd = FLT_SIN((a->phi + b->phi) / 2);
	FLT adjP = FLT_SIN((a->theta + b->theta) / 2);
	FLT pythAngle = sqrt(SQUARED(p*adjP) + SQUARED(d*adjd));
	return pythAngle;
}

Point calculateTorusPointFromAngles(PointsAndAngle *pna, FLT toroidalSin, FLT toroidalCos, FLT poloidalAngle, FLT poloidalSin)
{
	Point result;

	FLT distanceBetweenPoints = distance(pna->a, pna->b);
	Point m = midpoint(pna->a, pna->b);
	Matrix3x3 rot = pna->rotation;

	FLT toroidalRadius = distanceBetweenPoints / (2 * pna->tanAngle);
	FLT poloidalRadius = FLT_SQRT(SQUARED(toroidalRadius) + SQUARED(distanceBetweenPoints / 2));

	result.x = (toroidalRadius + poloidalRadius*cos(poloidalAngle))*toroidalCos;
	result.y = (toroidalRadius + poloidalRadius*cos(poloidalAngle))*toroidalSin;
	result.z = poloidalRadius*poloidalSin;
	result = RotateAndTranslatePoint(result, rot, m);

	return result;
}

FLT getPointFitnessForPna(Point pointIn, PointsAndAngle *pna)
{

	double toroidalSin = 0;
	double toroidalCos = 0;
	double poloidalAngle = 0;
	double poloidalSin = 0;

	estimateToroidalAndPoloidalAngleOfPoint(
		pna,
		pointIn,
		&toroidalSin,
		&toroidalCos,
		&poloidalAngle,
		&poloidalSin);

	Point torusPoint = calculateTorusPointFromAngles(pna, toroidalSin, toroidalCos, poloidalAngle, poloidalSin);

	FLT dist = distance(pointIn, torusPoint);

	// This is some voodoo black magic.  This is here to solve the problem that the origin 
	// (which is near the center of all the tori) erroniously will rank as a good match.
	// through a lot of empiracle testing on how to compensate for this, the "fudge factor"
	// below ended up being the best fit.  As simple as it is, I have a strong suspicion
	// that there's some crazy complex thesis-level math that could be used to derive this
	// but it works so we'll run with it.
	// Note that this may be resulting in a skewing of the found location by several millimeters.
	// it is not clear if this is actually removing existing skew (to get a more accurate value)
	// or if it is introducing an undesirable skew.
	double fudge = FLT_SIN((poloidalAngle - M_PI) / 2);
	dist = dist / fudge;

	return dist;
}

FLT getPointFitness(Point pointIn, PointsAndAngle *pna, size_t pnaCount)
{
	FLT fitness;

	FLT resultSum = 0;

	for (size_t i = 0; i < pnaCount; i++)
	{
		fitness = getPointFitnessForPna(pointIn, &(pna[i]));
		resultSum += SQUARED(fitness);
	}

	return 1 / FLT_SQRT(resultSum);
}

Point getGradient(Point pointIn, PointsAndAngle *pna, size_t pnaCount, FLT precision)
{
	Point result;

	Point tmpXplus = pointIn;
	Point tmpXminus = pointIn;
	tmpXplus.x = pointIn.x + precision;
	tmpXminus.x = pointIn.x - precision;
	result.x = getPointFitness(tmpXplus, pna, pnaCount) - getPointFitness(tmpXminus, pna, pnaCount);

	Point tmpYplus = pointIn;
	Point tmpYminus = pointIn;
	tmpYplus.y = pointIn.y + precision;
	tmpYminus.y = pointIn.y - precision;
	result.y = getPointFitness(tmpYplus, pna, pnaCount) - getPointFitness(tmpYminus, pna, pnaCount);

	Point tmpZplus = pointIn;
	Point tmpZminus = pointIn;
	tmpZplus.z = pointIn.z + precision;
	tmpZminus.z = pointIn.z - precision;
	result.z = getPointFitness(tmpZplus, pna, pnaCount) - getPointFitness(tmpZminus, pna, pnaCount);

	return result;
}

Point getNormalizedAndScaledVector(Point vectorIn, FLT desiredMagnitude)
{
	FLT distanceIn = sqrt(SQUARED(vectorIn.x) + SQUARED(vectorIn.y) + SQUARED(vectorIn.z));

	FLT scale = desiredMagnitude / distanceIn;

	Point result = vectorIn;

	result.x *= scale;
	result.y *= scale;
	result.z *= scale;

	return result;
}

Point getAvgPoints(Point a, Point b)
{
	Point result;
	result.x = (a.x + b.x) / 2;
	result.y = (a.y + b.y) / 2;
	result.z = (a.z + b.z) / 2;
	return result;
}


// This is modifies the basic gradient descent algorithm to better handle the shallow valley case,
// which appears to be typical of this convergence.  
static Point RefineEstimateUsingModifiedGradientDescent1(Point initialEstimate, PointsAndAngle *pna, size_t pnaCount, FILE *logFile)
{
	int i = 0;
	FLT lastMatchFitness = getPointFitness(initialEstimate, pna, pnaCount);
	Point lastPoint = initialEstimate;

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
	for (FLT g = 0.2; g > 0.00001; g *= 0.99)
	{
		i++;
		Point point1 = lastPoint;
		// let's get 3 iterations of gradient descent here.
		Point gradient1 = getGradient(point1, pna, pnaCount, g / 1000 /*somewhat arbitrary*/);
		Point gradientN1 = getNormalizedAndScaledVector(gradient1, g);

		Point point2;
		point2.x = point1.x + gradientN1.x;
		point2.y = point1.y + gradientN1.y;
		point2.z = point1.z + gradientN1.z;

		Point gradient2 = getGradient(point2, pna, pnaCount, g / 1000 /*somewhat arbitrary*/);
		Point gradientN2 = getNormalizedAndScaledVector(gradient2, g);

		Point point3;
		point3.x = point2.x + gradientN2.x;
		point3.y = point2.y + gradientN2.y;
		point3.z = point2.z + gradientN2.z;

		// remember that gradient descent has a tendency to zig-zag when it encounters a narrow valley?
		// Well, solving the lighthouse problem presents a very narrow valley, and the zig-zag of a basic
		// gradient descent is kinda horrible here.  Instead, think about the shape that a zig-zagging 
		// converging gradient descent makes.  Instead of using the gradient as the best indicator of 
		// the direction we should follow, we're looking at one side of the zig-zag pattern, and specifically
		// following *that* vector.  As it turns out, this works *amazingly* well.  

		Point specialGradient = { .x = point3.x - point1.x,.y = point3.y - point1.y,.z = point3.y - point1.y };

		// The second parameter to this function is very much a tunable parameter.  Different values will result
		// in a different number of iterations before we get to the minimum.  Numbers between 3-10 seem to work well
		// It's not clear what would be optimum here.
		specialGradient = getNormalizedAndScaledVector(specialGradient, g / 4);

		Point point4;

		point4.x = point3.x + specialGradient.x;
		point4.y = point3.y + specialGradient.y;
		point4.z = point3.z + specialGradient.z;

		FLT newMatchFitness = getPointFitness(point4, pna, pnaCount);

		if (newMatchFitness > lastMatchFitness)
		{
			if (logFile)
			{
				writePoint(logFile, lastPoint.x, lastPoint.y, lastPoint.z, 0xFFFFFF);
			}

			lastMatchFitness = newMatchFitness;
			lastPoint = point4;
#ifdef TORI_DEBUG
			printf("+");
#endif
		}
		else
		{
#ifdef TORI_DEBUG
			printf("-");
#endif
			g *= 0.7;

		}


	}
	printf("\ni=%d\n", i);

	return lastPoint;
}


// interesting-- this is one place where we could use any sensors that are only hit by
// just an x or y axis to make our estimate better.  TODO: bring that data to this fn.
FLT RotationEstimateFitnessOld(Point lhPoint, FLT *quaternion, TrackedObject *obj)
{
	FLT fitness = 0;
	for (size_t i = 0; i < obj->numSensors; i++)
	{
		// first, get the normal of the plane for the horizonal sweep
		FLT theta = obj->sensor[i].theta;
		// make two vectors that lie on the plane
		FLT t1H[3] = { 1, tan(theta-LINMATHPI/2), 0 };
		FLT t2H[3] = { 1, tan(theta-LINMATHPI/2), 1 };

		FLT tNormH[3];

		// the normal is the cross of two vectors on the plane.
		cross3d(tNormH, t1H, t2H);

		normalize3d(tNormH, tNormH);

		// Now do the same for the vertical sweep

		// first, get the normal of the plane for the horizonal sweep
		FLT phi = obj->sensor[i].phi;
		// make two vectors that lie on the plane
		FLT t1V[3] = { 0, 1, tan(phi-LINMATHPI/2)};
		FLT t2V[3] = { 1, 1, tan(phi-LINMATHPI/2)};

		FLT tNormV[3];

		// the normal is the cross of two vectors on the plane.
		cross3d(tNormV, t1V, t2V);

		normalize3d(tNormV, tNormV);


		// First, where is the sensor in the object's reference frame?
		FLT sensor_in_obj_reference_frame[3] = {obj->sensor->point.x, obj->sensor->point.y, obj->sensor->point.z};
		// Where is the point, in the reference frame of the lighthouse?
		// This has two steps, first we translate from the object's location being the
		// origin to the lighthouse being the origin.
		// And second, we apply the quaternion to rotate into the proper reference frame for the lighthouse.

		FLT sensor_in_lh_reference_frame[3];
		sub3d(sensor_in_lh_reference_frame, sensor_in_obj_reference_frame, (FLT[3]){lhPoint.x, lhPoint.y, lhPoint.z});

		quatrotatevector(sensor_in_lh_reference_frame, quaternion, sensor_in_lh_reference_frame);

		// now the we've got the location of the sensor in the lighthouses's reference frame, given lhPoint and quaternion inputs.

		// We need an arbitrary vector from the plane to the point.  
		// Since the plane goes through the origin, this is trivial.
		// The sensor point itself is such a vector!

		// And go calculate the distances!
		// TODO: don't need to ABS these because we square them below.
		FLT dH = FLT_FABS(dot3d(sensor_in_lh_reference_frame, tNormH));
		FLT dV = FLT_FABS(dot3d(sensor_in_lh_reference_frame, tNormV));

		
		fitness += SQUARED(dH);
		fitness += SQUARED(dV);
	}

	fitness = FLT_SQRT(fitness);

	return fitness;
}

FLT RotationEstimateFitnessAxisAngle(Point lh, FLT *AxisAngle, TrackedObject *obj)
{
	// For this fitness calculator, we're going to use the rotation information to figure out where
	// we expect to see the tracked object sensors, and we'll do a sum of squares to grade
	// the quality of the guess formed by the AxisAngle;

	FLT fitness = 0;

	// for each point in the tracked object
	for (int i=0; i< obj->numSensors; i++)
	{


		
		// let's see... we need to figure out where this sensor should be in the LH reference frame.
		FLT sensorLocation[3] = {obj->sensor[i].point.x-lh.x, obj->sensor[i].point.y-lh.y, obj->sensor[i].point.z-lh.z};

		// And this puppy needs to be rotated...

		rotatearoundaxis(sensorLocation, sensorLocation, AxisAngle, AxisAngle[3]);

		// Now, the vector indicating the position of the sensor, as seen by the lighthouse is:
		FLT realVectFromLh[3] = {1, tan(obj->sensor[i].theta - LINMATHPI/2), tan(obj->sensor[i].phi - LINMATHPI/2)};

		// and the vector we're calculating given the rotation passed in is the same as the sensor location:
		FLT calcVectFromLh[3] = {sensorLocation[0], sensorLocation[1], sensorLocation[2]};

		FLT angleBetween = anglebetween3d( realVectFromLh, calcVectFromLh );

		fitness += SQUARED(angleBetween);
	}

	return 1/FLT_SQRT(fitness);
}

// This figures out how far away from the scanned planes each point is, then does a sum of squares
// for the fitness.
// 
// interesting-- this is one place where we could use any sensors that are only hit by
// just an x or y axis to make our estimate better.  TODO: bring that data to this fn.
FLT RotationEstimateFitnessAxisAngleOriginal(Point lhPoint, FLT *quaternion, TrackedObject *obj)
{
	FLT fitness = 0;
	for (size_t i = 0; i < obj->numSensors; i++)
	{
		// first, get the normal of the plane for the horizonal sweep
		FLT theta = obj->sensor[i].theta;
		// make two vectors that lie on the plane
		FLT t1H[3] = { 1, tan(theta-LINMATHPI/2), 0 };
		FLT t2H[3] = { 1, tan(theta-LINMATHPI/2), 1 };

		FLT tNormH[3];

		// the normal is the cross of two vectors on the plane.
		cross3d(tNormH, t1H, t2H);

		normalize3d(tNormH, tNormH);

		// Now do the same for the vertical sweep

		// first, get the normal of the plane for the horizonal sweep
		FLT phi = obj->sensor[i].phi;
		// make two vectors that lie on the plane
		FLT t1V[3] = { 0, 1, tan(phi-LINMATHPI/2)};
		FLT t2V[3] = { 1, 1, tan(phi-LINMATHPI/2)};

		FLT tNormV[3];

		// the normal is the cross of two vectors on the plane.
		cross3d(tNormV, t1V, t2V);

		normalize3d(tNormV, tNormV);


		// First, where is the sensor in the object's reference frame?
		FLT sensor_in_obj_reference_frame[3] = {obj->sensor->point.x, obj->sensor->point.y, obj->sensor->point.z};
		// Where is the point, in the reference frame of the lighthouse?
		// This has two steps, first we translate from the object's location being the
		// origin to the lighthouse being the origin.
		// And second, we apply the quaternion to rotate into the proper reference frame for the lighthouse.

		FLT sensor_in_lh_reference_frame[3];
		sub3d(sensor_in_lh_reference_frame, sensor_in_obj_reference_frame, (FLT[3]){lhPoint.x, lhPoint.y, lhPoint.z});

		//quatrotatevector(sensor_in_lh_reference_frame, quaternion, sensor_in_lh_reference_frame);
		rotatearoundaxis(sensor_in_lh_reference_frame, sensor_in_lh_reference_frame, quaternion, quaternion[3]);

		// now the we've got the location of the sensor in the lighthouses's reference frame, given lhPoint and quaternion inputs.

		// We need an arbitrary vector from the plane to the point.  
		// Since the plane goes through the origin, this is trivial.
		// The sensor point itself is such a vector!

		// And go calculate the distances!
		// TODO: don't need to ABS these because we square them below.
		FLT dH = FLT_FABS(dot3d(sensor_in_lh_reference_frame, tNormH));
		FLT dV = FLT_FABS(dot3d(sensor_in_lh_reference_frame, tNormV));

		
		fitness += SQUARED(dH);
		fitness += SQUARED(dV);
	}

	fitness = FLT_SQRT(fitness);

	return 1/fitness;
}

// interesting-- this is one place where we could use any sensors that are only hit by
// just an x or y axis to make our estimate better.  TODO: bring that data to this fn.
FLT RotationEstimateFitnessQuaternion(Point lhPoint, FLT *quaternion, TrackedObject *obj)
{
	FLT fitness = 0;
	for (size_t i = 0; i < obj->numSensors; i++)
	{
		// first, get the normal of the plane for the horizonal sweep
		FLT theta = obj->sensor[i].theta;
		// make two vectors that lie on the plane
		FLT t1H[3] = { 1, tan(theta-LINMATHPI/2), 0 };
		FLT t2H[3] = { 1, tan(theta-LINMATHPI/2), 1 };

		FLT tNormH[3];

		// the normal is the cross of two vectors on the plane.
		cross3d(tNormH, t1H, t2H);

		normalize3d(tNormH, tNormH);

		// Now do the same for the vertical sweep

		// first, get the normal of the plane for the horizonal sweep
		FLT phi = obj->sensor[i].phi;
		// make two vectors that lie on the plane
		FLT t1V[3] = { 0, 1, tan(phi-LINMATHPI/2)};
		FLT t2V[3] = { 1, 1, tan(phi-LINMATHPI/2)};

		FLT tNormV[3];

		// the normal is the cross of two vectors on the plane.
		cross3d(tNormV, t1V, t2V);

		normalize3d(tNormV, tNormV);


		// First, where is the sensor in the object's reference frame?
		FLT sensor_in_obj_reference_frame[3] = {obj->sensor->point.x, obj->sensor->point.y, obj->sensor->point.z};
		// Where is the point, in the reference frame of the lighthouse?
		// This has two steps, first we translate from the object's location being the
		// origin to the lighthouse being the origin.
		// And second, we apply the quaternion to rotate into the proper reference frame for the lighthouse.

		FLT sensor_in_lh_reference_frame[3];
		sub3d(sensor_in_lh_reference_frame, sensor_in_obj_reference_frame, (FLT[3]){lhPoint.x, lhPoint.y, lhPoint.z});

		quatrotatevector(sensor_in_lh_reference_frame, quaternion, sensor_in_lh_reference_frame);
		//rotatearoundaxis(sensor_in_lh_reference_frame, sensor_in_lh_reference_frame, quaternion, quaternion[3]);

		// now the we've got the location of the sensor in the lighthouses's reference frame, given lhPoint and quaternion inputs.

		// We need an arbitrary vector from the plane to the point.  
		// Since the plane goes through the origin, this is trivial.
		// The sensor point itself is such a vector!

		// And go calculate the distances!
		// TODO: don't need to ABS these because we square them below.
		FLT dH = FLT_FABS(dot3d(sensor_in_lh_reference_frame, tNormH));
		FLT dV = FLT_FABS(dot3d(sensor_in_lh_reference_frame, tNormV));

		
		fitness += SQUARED(dH);
		fitness += SQUARED(dV);
	}

	fitness = FLT_SQRT(fitness);

	return 1/fitness;
}


void getRotationGradientQuaternion(FLT *gradientOut, Point lhPoint, FLT *quaternion, TrackedObject *obj, FLT precision)
{

	FLT baseFitness = RotationEstimateFitnessQuaternion(lhPoint, quaternion, obj);

	FLT tmp0plus[4];
	quatadd(tmp0plus, quaternion, (FLT[4]){precision, 0, 0, 0});
	gradientOut[0] = RotationEstimateFitnessQuaternion(lhPoint, tmp0plus, obj) - baseFitness;

	FLT tmp1plus[4];
	quatadd(tmp1plus, quaternion, (FLT[4]){0, precision, 0, 0});
	gradientOut[1] = RotationEstimateFitnessQuaternion(lhPoint, tmp1plus, obj) - baseFitness;

	FLT tmp2plus[4];
	quatadd(tmp2plus, quaternion, (FLT[4]){0, 0, precision, 0});
	gradientOut[2] = RotationEstimateFitnessQuaternion(lhPoint, tmp2plus, obj) - baseFitness;

	FLT tmp3plus[4];
	quatadd(tmp3plus, quaternion, (FLT[4]){0, 0, 0, precision});
	gradientOut[3] = RotationEstimateFitnessQuaternion(lhPoint, tmp3plus, obj) - baseFitness;

	return;
}

void getRotationGradientAxisAngle(FLT *gradientOut, Point lhPoint, FLT *quaternion, TrackedObject *obj, FLT precision)
{

	FLT baseFitness = RotationEstimateFitnessAxisAngle(lhPoint, quaternion, obj);

	FLT tmp0plus[4];
	quatadd(tmp0plus, quaternion, (FLT[4]){precision, 0, 0, 0});
	gradientOut[0] = RotationEstimateFitnessAxisAngle(lhPoint, tmp0plus, obj) - baseFitness;

	FLT tmp1plus[4];
	quatadd(tmp1plus, quaternion, (FLT[4]){0, precision, 0, 0});
	gradientOut[1] = RotationEstimateFitnessAxisAngle(lhPoint, tmp1plus, obj) - baseFitness;

	FLT tmp2plus[4];
	quatadd(tmp2plus, quaternion, (FLT[4]){0, 0, precision, 0});
	gradientOut[2] = RotationEstimateFitnessAxisAngle(lhPoint, tmp2plus, obj) - baseFitness;

	FLT tmp3plus[4];
	quatadd(tmp3plus, quaternion, (FLT[4]){0, 0, 0, precision});
	gradientOut[3] = RotationEstimateFitnessAxisAngle(lhPoint, tmp3plus, obj) - baseFitness;

	return;
}

//void getNormalizedAndScaledRotationGradient(FLT *vectorToScale, FLT desiredMagnitude)
//{
//	quatnormalize(vectorToScale, vectorToScale);
//	quatscale(vectorToScale, vectorToScale, desiredMagnitude);
//	return;
//}
void getNormalizedAndScaledRotationGradient(FLT *vectorToScale, FLT desiredMagnitude)
{
	quatnormalize(vectorToScale, vectorToScale);
	quatscale(vectorToScale, vectorToScale, desiredMagnitude);
	//vectorToScale[3] = desiredMagnitude;

	return;
}

static void WhereIsTheTrackedObjectAxisAngle(FLT *rotation, Point lhPoint)
{
	FLT reverseRotation[4] = {rotation[0], rotation[1], rotation[2], -rotation[3]};
	FLT objPoint[3] = {lhPoint.x, lhPoint.y, lhPoint.z};
	
	rotatearoundaxis(objPoint, objPoint, reverseRotation, reverseRotation[3]);

	printf("The tracked object is at location (%f, %f, %f)\n", objPoint[0], objPoint[1], objPoint[2]);
}

static void RefineRotationEstimateAxisAngle(FLT *rotOut, Point lhPoint, FLT *initialEstimate, TrackedObject *obj)
{
	int i = 0;
	FLT lastMatchFitness = RotationEstimateFitnessAxisAngle(lhPoint, initialEstimate, obj);

	quatcopy(rotOut, initialEstimate);

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
	for (FLT g = 0.1; g > 0.000000001; g *= 0.99)
	{
		i++;
		FLT point1[4];
		quatcopy(point1, rotOut);
		// let's get 3 iterations of gradient descent here.
		FLT gradient1[4];
		
		normalize3d(point1, point1);

		getRotationGradientAxisAngle(gradient1, lhPoint, point1, obj, g/10000);
		getNormalizedAndScaledRotationGradient(gradient1,g);

		FLT point2[4];
		quatadd(point2, gradient1, point1);
		//quatnormalize(point2,point2);

		normalize3d(point1, point1);

		FLT gradient2[4];
		getRotationGradientAxisAngle(gradient2, lhPoint, point2, obj, g/10000);
		getNormalizedAndScaledRotationGradient(gradient2,g);

		FLT point3[4];
		quatadd(point3, gradient2, point2);

		normalize3d(point1, point1);

		//quatnormalize(point3,point3);

		// remember that gradient descent has a tendency to zig-zag when it encounters a narrow valley?
		// Well, solving the lighthouse problem presents a very narrow valley, and the zig-zag of a basic
		// gradient descent is kinda horrible here.  Instead, think about the shape that a zig-zagging 
		// converging gradient descent makes.  Instead of using the gradient as the best indicator of 
		// the direction we should follow, we're looking at one side of the zig-zag pattern, and specifically
		// following *that* vector.  As it turns out, this works *amazingly* well.  

		FLT specialGradient[4];
		quatsub(specialGradient,point3,point1);

		// The second parameter to this function is very much a tunable parameter.  Different values will result
		// in a different number of iterations before we get to the minimum.  Numbers between 3-10 seem to work well
		// It's not clear what would be optimum here.
		getNormalizedAndScaledRotationGradient(specialGradient,g/4);

		FLT point4[4];
		quatadd(point4, specialGradient, point3);
		//quatnormalize(point4,point4);
		normalize3d(point1, point1);

		FLT newMatchFitness = RotationEstimateFitnessAxisAngle(lhPoint, point4, obj);

		if (newMatchFitness > lastMatchFitness)
		{

			lastMatchFitness = newMatchFitness;
			quatcopy(rotOut, point4);
//#ifdef TORI_DEBUG
			//printf("+  %8.8f, (%8.8f, %8.8f, %8.8f) %f\n", newMatchFitness, point4[0], point4[1], point4[2], point4[3]);
//#endif
			g *= 1.02;

		}
		else
		{
//#ifdef TORI_DEBUG
			//printf("-         , %f\n", point4[3]);
//#endif
			g *= 0.7;

		}


	}
	printf("\nRi=%d\n", i);
}
static void WhereIsTheTrackedObjectQuaternion(FLT *rotation, Point lhPoint)
{
	FLT reverseRotation[4] = {rotation[0], rotation[1], rotation[2], -rotation[3]};
	FLT objPoint[3] = {lhPoint.x, lhPoint.y, lhPoint.z};
	
	//rotatearoundaxis(objPoint, objPoint, reverseRotation, reverseRotation[3]);
	quatrotatevector(objPoint, rotation, objPoint);
	printf("The tracked object is at location (%f, %f, %f)\n", objPoint[0], objPoint[1], objPoint[2]);
}



static void RefineRotationEstimateQuaternion(FLT *rotOut, Point lhPoint, FLT *initialEstimate, TrackedObject *obj)
{
	int i = 0;
	FLT lastMatchFitness = RotationEstimateFitnessQuaternion(lhPoint, initialEstimate, obj);

	quatcopy(rotOut, initialEstimate);

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
	for (FLT g = 0.1; g > 0.000000001; g *= 0.99)
	{
		i++;
		FLT point1[4];
		quatcopy(point1, rotOut);
		// let's get 3 iterations of gradient descent here.
		FLT gradient1[4];
		
		//normalize3d(point1, point1);

		getRotationGradientQuaternion(gradient1, lhPoint, point1, obj, g/10000);
		getNormalizedAndScaledRotationGradient(gradient1,g);

		FLT point2[4];
		quatadd(point2, gradient1, point1);
		quatnormalize(point2,point2);

		//normalize3d(point1, point1);

		FLT gradient2[4];
		getRotationGradientQuaternion(gradient2, lhPoint, point2, obj, g/10000);
		getNormalizedAndScaledRotationGradient(gradient2,g);

		FLT point3[4];
		quatadd(point3, gradient2, point2);

		//normalize3d(point1, point1);

		quatnormalize(point3,point3);

		// remember that gradient descent has a tendency to zig-zag when it encounters a narrow valley?
		// Well, solving the lighthouse problem presents a very narrow valley, and the zig-zag of a basic
		// gradient descent is kinda horrible here.  Instead, think about the shape that a zig-zagging 
		// converging gradient descent makes.  Instead of using the gradient as the best indicator of 
		// the direction we should follow, we're looking at one side of the zig-zag pattern, and specifically
		// following *that* vector.  As it turns out, this works *amazingly* well.  

		FLT specialGradient[4];
		quatsub(specialGradient,point3,point1);

		// The second parameter to this function is very much a tunable parameter.  Different values will result
		// in a different number of iterations before we get to the minimum.  Numbers between 3-10 seem to work well
		// It's not clear what would be optimum here.
		getNormalizedAndScaledRotationGradient(specialGradient,g/4);

		FLT point4[4];
		quatadd(point4, specialGradient, point3);
		quatnormalize(point4,point4);
		//normalize3d(point1, point1);

		FLT newMatchFitness = RotationEstimateFitnessQuaternion(lhPoint, point4, obj);

		if (newMatchFitness > lastMatchFitness)
		{

			lastMatchFitness = newMatchFitness;
			quatcopy(rotOut, point4);
//#ifdef TORI_DEBUG
			//printf("+  %8.8f, (%8.8f, %8.8f, %8.8f) %f\n", newMatchFitness, point4[0], point4[1], point4[2], point4[3]);
//#endif
			g *= 1.02;
			printf("+");
			WhereIsTheTrackedObjectQuaternion(rotOut, lhPoint);
		}
		else
		{
//#ifdef TORI_DEBUG
			//printf("-         , %f\n", point4[3]);
//#endif
			g *= 0.7;
			printf("-");
		}


	}
	printf("\nRi=%d  Fitness=%3f\n", i, lastMatchFitness);
}


void SolveForRotation(FLT rotOut[4], TrackedObject *obj, Point lh)
{

	// Step 1, create initial quaternion for guess.  
	// This should have the lighthouse directly facing the tracked object.
	Point trackedObjRelativeToLh = { .x = -lh.x,.y = -lh.y,.z = -lh.z };
	FLT theta = atan2(-lh.x, -lh.y);
	FLT zAxis[4] = { 0, 0, 1 , theta-LINMATHPI/2};
	FLT quat1[4];
	quatfromaxisangle(quat1, zAxis, theta);

	//quatfrom2vectors(0,0)
	// not correcting for phi, but that's less important.


	// Step 2, optimize the axis/ angle to match the data.
	RefineRotationEstimateAxisAngle(rotOut, lh, zAxis, obj);

	WhereIsTheTrackedObjectAxisAngle(rotOut, lh);

	//// Step 2, optimize the quaternion to match the data.
	//RefineRotationEstimateQuaternion(rotOut, lh, quat1, obj);

	//WhereIsTheTrackedObjectQuaternion(rotOut, lh);

}


Point SolveForLighthouse(TrackedObject *obj, char doLogOutput)
{
	PointsAndAngle pna[MAX_POINT_PAIRS];

	volatile size_t sizeNeeded = sizeof(pna);

	Point avgNorm = { 0 };

	size_t pnaCount = 0;
	for (unsigned int i = 0; i < obj->numSensors; i++)
	{
		for (unsigned int j = 0; j < i; j++)
		{
			if (pnaCount < MAX_POINT_PAIRS)
			{
				pna[pnaCount].a = obj->sensor[i].point;
				pna[pnaCount].b = obj->sensor[j].point;

				pna[pnaCount].angle = angleBetweenSensors(&obj->sensor[i], &obj->sensor[j]);
				//pna[pnaCount].angle = pythAngleBetweenSensors2(&obj->sensor[i], &obj->sensor[j]);
				pna[pnaCount].tanAngle = FLT_TAN(pna[pnaCount].angle);

				double pythAngle = sqrt(SQUARED(obj->sensor[i].phi - obj->sensor[j].phi) + SQUARED(obj->sensor[i].theta - obj->sensor[j].theta));

				pna[pnaCount].rotation = GetRotationMatrixForTorus(pna[pnaCount].a, pna[pnaCount].b);
				pna[pnaCount].invRotation = inverseM33(pna[pnaCount].rotation);


				pnaCount++;
			}
		}

		avgNorm.x += obj->sensor[i].normal.x;
		avgNorm.y += obj->sensor[i].normal.y;
		avgNorm.z += obj->sensor[i].normal.z;
	}
	avgNorm.x = avgNorm.x / obj->numSensors;
	avgNorm.y = avgNorm.y / obj->numSensors;
	avgNorm.z = avgNorm.z / obj->numSensors;

	FLT avgNormF[3] = { avgNorm.x, avgNorm.y, avgNorm.z };


	FILE *logFile = NULL;
	if (doLogOutput)
	{
		logFile = fopen("pointcloud2.pcd", "wb");
		writePcdHeader(logFile);
		writeAxes(logFile);
	}


	// Point refinedEstimageGd = RefineEstimateUsingModifiedGradientDescent1(initialEstimate, pna, pnaCount, logFile);


	// arbitrarily picking a value of 8 meters out to start from.
	// intentionally picking the direction of the average normal vector of the sensors that see the lighthouse
	// since this is least likely to pick the incorrect "mirror" point that would send us 
	// back into the search for the correct point (see "if (a1 > M_PI / 2)" below)
	Point p1 = getNormalizedAndScaledVector(avgNorm, 8);

	Point refinedEstimateGd = RefineEstimateUsingModifiedGradientDescent1(p1, pna, pnaCount, logFile);

	FLT pf1[3] = { refinedEstimateGd.x, refinedEstimateGd.y, refinedEstimateGd.z };

	FLT a1 = anglebetween3d(pf1, avgNormF);

	if (a1 > M_PI / 2)
	{
		Point p2 = { .x = -refinedEstimateGd.x,.y = -refinedEstimateGd.y,.z = -refinedEstimateGd.z };
		refinedEstimateGd = RefineEstimateUsingModifiedGradientDescent1(p2, pna, pnaCount, logFile);

		//FLT pf2[3] = { refinedEstimageGd2.x, refinedEstimageGd2.y, refinedEstimageGd2.z };

		//FLT a2 = anglebetween3d(pf2, avgNormF);

	}

	FLT fitGd = getPointFitness(refinedEstimateGd, pna, pnaCount);

	FLT distance = FLT_SQRT(SQUARED(refinedEstimateGd.x) + SQUARED(refinedEstimateGd.y) + SQUARED(refinedEstimateGd.z));
	printf("(%4.4f, %4.4f, %4.4f)\n", refinedEstimateGd.x, refinedEstimateGd.y, refinedEstimateGd.z);
	printf("Distance is %f,   Fitness is %f\n", distance, fitGd);

	FLT rot[4];
	SolveForRotation(rot, obj, refinedEstimateGd);

	if (logFile)
	{
		updateHeader(logFile);
		fclose(logFile);
	}
	//fgetc(stdin);
	return refinedEstimateGd;
}












int PoserTurveyTori( SurviveObject * so, PoserData * pd )
{
	PoserType pt = pd->pt;
	SurviveContext * ctx = so->ctx;
	ToriData * dd = so->PoserData;

	if (!dd)
	{
		so->PoserData = dd = malloc(sizeof(ToriData));
		memset(dd, 0, sizeof(ToriData));
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
		//printf( "LIG:%s %d @ %f rad, %f s (AC %d) (TC %d)\n", so->codename, l->sensor_id, l->angle, l->length, l->acode, l->timecode );
		break;
	}
	case POSERDATA_FULL_SCENE:
	{
		TrackedObject *to;

		PoserDataFullScene * fs = (PoserDataFullScene*)pd;

		to = malloc(sizeof(TrackedObject) + (SENSORS_PER_OBJECT * sizeof(TrackedSensor)));

		//FLT  lengths[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2];
		//FLT  angles[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2];  //2 Axes  (Angles in LH space)
		//FLT  synctimes[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES];

		//to->numSensors = so->nr_locations;
		{
			int sensorCount = 0;

			for (int i = 0; i < so->nr_locations; i++)
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
					sensorCount++;
				}
			}

			to->numSensors = sensorCount;

			SolveForLighthouse(to, 0);
		}
		{
			int sensorCount = 0;
			int lh = 1;

			for (int i = 0; i < so->nr_locations; i++)
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
					sensorCount++;
				}
			}

			to->numSensors = sensorCount;

			SolveForLighthouse(to, 0);
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


REGISTER_LINKTIME( PoserTurveyTori );

