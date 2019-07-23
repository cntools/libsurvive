#include <assert.h>
#include <math.h>
#include <survive_reproject.h>
#include <survive_reproject_gen2.h>

#pragma GCC push_options
#pragma GCC optimize("Ofast")

#include "survive_reproject.generated.h"

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
	FLT A = asin(linmath_enforce_range(tan30 * Y / sqrt(X * X + Z * Z), -1, 1));

	out[0] = -A - B - bcal[0].phase;
	out[1] = A - B - bcal[1].phase;

	assert(!isnan(out[0]));
	assert(!isnan(out[1]));
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

void survive_reproject_full_x_jac_obj_pose_gen2(SurviveAngleReading out, const SurvivePose *obj_pose,
												const double *obj_pt, const SurvivePose *world2lh,
												const BaseStationCal *bcal) {
	FLT phase_0 = bcal[0].phase;
	FLT tilt_0 = bcal[0].tilt;
	FLT curve_0 = bcal[0].curve;
	FLT gibPhase_0 = bcal[0].gibpha;
	FLT gibMag_0 = bcal[0].gibmag;

	gen_reproject_axis_x_jac_obj_p_gen2(out, obj_pose->Pos, obj_pt, world2lh->Pos, phase_0, tilt_0, curve_0, gibPhase_0,
										gibMag_0);
}

void survive_reproject_full_y_jac_obj_pose_gen2(SurviveAngleReading out, const SurvivePose *obj_pose,
												const double *obj_pt, const SurvivePose *world2lh,
												const BaseStationCal *bcal) {
	FLT phase_1 = bcal[1].phase;
	FLT tilt_1 = bcal[1].tilt;
	FLT curve_1 = bcal[1].curve;
	FLT gibPhase_1 = bcal[1].gibpha;
	FLT gibMag_1 = bcal[1].gibmag;

	gen_reproject_axis_y_jac_obj_p_gen2(out, obj_pose->Pos, obj_pt, world2lh->Pos, phase_1, tilt_1, curve_1, gibPhase_1,
										gibMag_1);
}

void survive_reproject_full_jac_obj_pose_gen2(SurviveAngleReading out, const SurvivePose *obj_pose,
											  const LinmathVec3d obj_pt, const SurvivePose *world2lh,
											  const BaseStationCal *bcal) {
	FLT phase_0 = bcal[0].phase;
	FLT phase_1 = bcal[1].phase;

	FLT tilt_0 = bcal[0].tilt;
	FLT tilt_1 = bcal[1].tilt;

	FLT curve_0 = bcal[0].curve;
	FLT curve_1 = bcal[1].curve;

	FLT gibPhase_0 = bcal[0].gibpha;
	FLT gibPhase_1 = bcal[1].gibpha;
	FLT gibMag_0 = bcal[0].gibmag;
	FLT gibMag_1 = bcal[1].gibmag;

	gen_reproject_jac_obj_p_gen2(out, obj_pose->Pos, obj_pt, world2lh->Pos, phase_0, phase_1, tilt_0, tilt_1, curve_0,
								 curve_1, gibPhase_0, gibPhase_1, gibMag_0, gibMag_1);
}

void survive_reproject_full_gen2(const BaseStationCal *bcal, const SurvivePose *world2lh, const SurvivePose *obj2world,
								 const LinmathVec3d obj_pt, SurviveAngleReading out) {
	LinmathVec3d world_pt;
	ApplyPoseToPoint(world_pt, obj2world, obj_pt);

	LinmathPoint3d t_pt;
	ApplyPoseToPoint(t_pt, world2lh, world_pt);

	survive_reproject_xy_gen2(bcal, t_pt, out);
}

const survive_reproject_model_t survive_reproject_gen2_model = {
	.reprojectAxisFn = {survive_reproject_axis_x_gen2, survive_reproject_axis_y_gen2},
	.reprojectXY = survive_reproject_xy_gen2,
	.reprojectAxisJacobFn = {survive_reproject_full_x_jac_obj_pose_gen2, survive_reproject_full_y_jac_obj_pose_gen2},
	.reprojectFullJacObjPose = survive_reproject_full_jac_obj_pose_gen2};
