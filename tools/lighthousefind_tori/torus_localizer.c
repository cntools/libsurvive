#include <memory.h>
#include <stdlib.h>
#include <assert.h>
#include <stdio.h>
#include "linmath.h"
#include "tori_includes.h"
#include "visualization.h"


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

	normalize3d(v2,v2);

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
	FLT angle2 = FLT_ACOS(FLT_COS(b->phi - a->phi)*FLT_COS(b->theta - a->theta));

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

	FLT resultSum=0;

	for (size_t i = 0; i < pnaCount; i++)
	{
		fitness = getPointFitnessForPna(pointIn, &(pna[i]));
		resultSum += SQUARED(fitness);
	}

	return 1/FLT_SQRT(resultSum);
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

Point getNormalizedVector(Point vectorIn, FLT desiredMagnitude)
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
		Point gradientN1 = getNormalizedVector(gradient1, g);

		Point point2;
		point2.x = point1.x + gradientN1.x;
		point2.y = point1.y + gradientN1.y;
		point2.z = point1.z + gradientN1.z;

		Point gradient2 = getGradient(point2, pna, pnaCount, g / 1000 /*somewhat arbitrary*/);
		Point gradientN2 = getNormalizedVector(gradient2, g);

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

		Point specialGradient = { .x = point3.x - point1.x, .y = point3.y - point1.y, .z = point3.y - point1.y };

		// The second parameter to this function is very much a tunable parameter.  Different values will result
		// in a different number of iterations before we get to the minimum.  Numbers between 3-10 seem to work well
		// It's not clear what would be optimum here.
		specialGradient = getNormalizedVector(specialGradient, g/4);

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
FLT RotationEstimateFitness(Point lhPoint, FLT *quaternion, TrackedObject *obj)
{
	for (size_t i = 0; i < obj->numSensors; i++)
	{
		// first, get the normal of the plane for the horizonal sweep
		FLT theta = obj->sensor[i].theta;
		// make two vectors that lie on the plane
		FLT t1[3] = { 1, tan(theta), 0 };
		FLT t2[3] = { 1, tan(theta), 1 };

		FLT tNorm[3];

		// the normal is the cross of two vectors on the plane.
		cross3d(tNorm, t1, t2);

		// distance for this plane is d= fabs(A*x + B*y)/sqrt(A^2+B^2) (z term goes away since this plane is "vertical")
		// where A is 
		//FLT d = 
	}
}

static Point RefineRotationEstimate(Point initialEstimate, PointsAndAngle *pna, size_t pnaCount, FILE *logFile)
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
		Point gradientN1 = getNormalizedVector(gradient1, g);

		Point point2;
		point2.x = point1.x + gradientN1.x;
		point2.y = point1.y + gradientN1.y;
		point2.z = point1.z + gradientN1.z;

		Point gradient2 = getGradient(point2, pna, pnaCount, g / 1000 /*somewhat arbitrary*/);
		Point gradientN2 = getNormalizedVector(gradient2, g);

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

		Point specialGradient = { .x = point3.x - point1.x, .y = point3.y - point1.y, .z = point3.y - point1.y };

		// The second parameter to this function is very much a tunable parameter.  Different values will result
		// in a different number of iterations before we get to the minimum.  Numbers between 3-10 seem to work well
		// It's not clear what would be optimum here.
		specialGradient = getNormalizedVector(specialGradient, g / 4);

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

void SolveForRotation(FLT rotOut[4], TrackedObject *obj, Point lh)
{

	// Step 1, create initial quaternion for guess.  
	// This should have the lighthouse directly facing the tracked object.
	Point trackedObjRelativeToLh = { .x = -lh.x, .y = -lh.y, .z = -lh.z };
	FLT theta = atan2(-lh.x, -lh.y);
	FLT zAxis[3] = { 0, 0, 1 };
	FLT quat1[4];
	quatfromaxisangle(quat1, zAxis, theta);
	// not correcting for phi, but that's less important.

	// Step 2, optimize the quaternion to match the data.

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
	Point p1 = getNormalizedVector(avgNorm, 8); 

	Point refinedEstimateGd = RefineEstimateUsingModifiedGradientDescent1(p1, pna, pnaCount, logFile);

	FLT pf1[3] = { refinedEstimateGd.x, refinedEstimateGd.y, refinedEstimateGd.z };

	FLT a1 = anglebetween3d(pf1, avgNormF);

	if (a1 > M_PI / 2)
	{
		Point p2 = { .x = -refinedEstimateGd.x, .y = -refinedEstimateGd.y, .z = -refinedEstimateGd.z };
		refinedEstimateGd = RefineEstimateUsingModifiedGradientDescent1(p2, pna, pnaCount, logFile);

		//FLT pf2[3] = { refinedEstimageGd2.x, refinedEstimageGd2.y, refinedEstimageGd2.z };

		//FLT a2 = anglebetween3d(pf2, avgNormF);

	}

	FLT fitGd = getPointFitness(refinedEstimateGd, pna, pnaCount);

	printf("Fitness is %f\n", fitGd);

	if (logFile)
	{
		updateHeader(logFile);
		fclose(logFile);
	}
	//fgetc(stdin);
	return refinedEstimateGd;
}

