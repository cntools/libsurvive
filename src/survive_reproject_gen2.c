#include "survive_reproject_gen2.h"
#include <math.h>
#include <survive_reproject.h>

void survive_reproject_xy_gen2(const BaseStationCal *bcal, LinmathVec3d const ptInLh, SurviveAngleReading out) {
	/***
	 Using plane equation:
	 A*x + B*y + C*z + D = 0;

	 If you are looking at the lighthouse, you have this:
			 ^
			 |
			 Y
			 |
	 <---X---o (objects in front of LH are -Z).
	 ---- Rotor Direction --->


	 The first plane is oriented like / and the second is \. When the sensor is on X=0 and Y=0, the colliding planes
	 then are X=-Y, X=Y. The normals are then [1, a, 0] and [1, -a, 0]. We define the point at which the sensor plane
	 sweeps X=Y=0 in both planes as t=0. The projected plane is roughly ~60 degrees off of horizontal. a should be
	 tan(90 - <plane angle>).

	 If the object is at X=epsilon, Y=0, Z=1, The rotation hits it slightly sooner; t is slightly negative. The normal
	 is then something like [~1-epsilon, a, ~+epsilon], [~1-epsilon, -a, ~+epsilon]

	 We know the planes are mostly centered in the lighthouse, and so we get:
	 [cos(t), a, -sin(t)], [cos(t), -a, -sin(t)]

	 For a given X, Y, Z solve for t:
	 X * cos(t) - Z * sin(t)                = +/-Y*a

	 Simplifies to this; given harmonic addition rules of sin/cos:
	 sqrt(X^2 + Z^2) * sin(t + atan2(X, -Z)) = +/-Y*a
	 sin(t + atan2(X, -Z))                   = +/-Y*a / sqrt(X^2 + Z^2)
	 t + atan2(X, -Z)                        = asin(+/-Y*a / sqrt(X^2 + Z^2))
	 t + atan2(X, -Z)                        = +/-asin(Y*a / sqrt(X^2 + Z^2))
	 t                                       = +/-asin(Y*a / sqrt(X^2 + Z^2)) - atan2(X, -Z)
	***/
	FLT X = ptInLh[0], Y = ptInLh[1], Z = ptInLh[2];
	FLT tan30 = 0.57735026919;
	FLT B = atan2(X, -Z);
	FLT A = asin(tan30 * Y / sqrt(X * X + Z * Z));

	out[0] = -A - B - bcal[0].phase;
	out[1] = A - B - bcal[1].phase;
}

FLT survive_reproject_axis_x_gen2(const BaseStationCal *bcal, LinmathVec3d const ptInLh) {
	SurviveAngleReading reading;
	survive_reproject_xy_gen2(bcal, ptInLh, reading);
	return reading[0];
}
FLT survive_reproject_axis_y_gen2(const BaseStationCal *bcal, LinmathVec3d const ptInLh) {
	SurviveAngleReading reading;
	survive_reproject_xy_gen2(bcal, ptInLh, reading);
	return reading[1];
}

void survive_reproject_from_pose_with_bcal_gen2(const BaseStationCal *bcal, const SurvivePose *world2lh,
												LinmathVec3d const ptInWorld, SurviveAngleReading out) {
	LinmathPoint3d ptInLh;
	ApplyPoseToPoint(ptInLh, world2lh, ptInWorld);
	survive_reproject_xy_gen2(bcal, ptInLh, out);
}

void survive_reproject_from_pose_gen2(const SurviveContext *ctx, int lighthouse, const SurvivePose *world2lh,
									  LinmathVec3d const pt, SurviveAngleReading out) {
	survive_reproject_from_pose_with_bcal_gen2(ctx->bsd[lighthouse].fcal, world2lh, pt, out);
}

void survive_reproject_gen2(const SurviveContext *ctx, int lighthouse, LinmathVec3d const ptInWorld,
							SurviveAngleReading out) {
	SurvivePose world2lh = InvertPoseRtn(&ctx->bsd[lighthouse].Pose);
	survive_reproject_from_pose_gen2(ctx, lighthouse, &world2lh, ptInWorld, out);
}

const survive_reproject_model_t survive_reproject_gen2_model = {
	.reprojectAxisFn = {survive_reproject_axis_x_gen2, survive_reproject_axis_y_gen2},
	.reprojectXY = survive_reproject_xy_gen2};