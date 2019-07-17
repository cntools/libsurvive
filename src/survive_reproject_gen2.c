#include "survive_reproject_gen2.h"
#include <math.h>
#include <survive_reproject.h>

void survive_reproject_xy_gen2(const BaseStationCal *bcal, LinmathVec3d const ptInLh, SurviveAngleReading out) {
	// Using plane equation:
	// A*x + B*y + C*z + D = 0;
	// We model it as a rotating plane at 45 degrees so the plane normals are:
	// [sin(t), +/-1, cos(t)]
	// For a given X, Y, Z solve for t:
	// X * sin(t) + Z * cos(t) = +/-Y
	// Simplifies to this; given harmonic addition rules of sin/cos:
	// sqrt(X^2 + Z^2) * sin(t + atan2(Z, X)) = +/-Y
	// sin(t + atan2(Z, X)) = +/-Y / sqrt(X^2 + Z^2)
	// t = +/-asin(Y / sqrt(X^2 + Z^2)) - atan2(Z, X)

	FLT X = ptInLh[0], Y = ptInLh[1], Z = ptInLh[2];

	FLT B = -atan2(Z, X);
	FLT A = asin(Y / sqrt(X * X + Z * Z));

	out[0] = A + B;
	out[1] = -A + B;
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