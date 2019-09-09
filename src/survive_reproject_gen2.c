#define _USE_MATH_DEFINES
#include <assert.h>
#include <math.h>
#include <survive_reproject.h>
#include <survive_reproject_gen2.h>

#pragma GCC push_options
#pragma GCC optimize("Ofast")

#include "survive_reproject.generated.h"

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

// a_i = a_(i-1) * s + m[i-1]; a[0] = 0
// m_i = m_(i-1) * s + f[i]; m[0] = f[0]
// This yields, approx:
// a_i = \sum f[i] * (N-i) * a^(N-i-1)
// m_i = \sum f[i] * a^(N-i-1)
// It's useful to see them as separate eqs but not ultimately useful.
static inline void calc_cal_series(double s, double *m, double *a) {
	const double f[6] = {-8.0108022e-06, 0.0028679863, 5.3685255000000001e-06, 0.0076069798000000001};

	*m = f[0], *a = 0;
	for (int i = 1; i < 6; i++) {
		*a = *a * s + *m;
		*m = *m * s + f[i];
	}
}

static inline FLT survive_reproject_axis_gen2(const BaseStationCal *bcal, FLT X, FLT Y, FLT Z, bool axis) {
	const FLT phase = bcal->phase;
	const FLT curve = bcal->curve;
	const FLT tilt = bcal->tilt;
	const FLT gibPhase = bcal->gibpha;
	const FLT gibMag = bcal->gibmag;
	const FLT ogeePhase = bcal->ogeephase;
	const FLT ogeeMag = bcal->ogeemag;

	double B = atan2(Z, X);

	double Ydeg = tilt + (axis ? -1 : 1) * M_PI / 6.;
	double tanA = tan(Ydeg);
	double normXZ = sqrt(X * X + Z * Z);

	double asinArg = linmath_enforce_range(tanA * Y / normXZ, -1, 1);

	double sinYdeg = sin(Ydeg);
	double cosYdeg = cos(Ydeg);

	double sinPart = sin(B - asin(asinArg) + ogeePhase) * ogeeMag;

	double normXYZ = sqrt(X * X + Y * Y + Z * Z);

	double modAsinArg = linmath_enforce_range(Y / normXYZ / cosYdeg, -1, 1);

	double asinOut = asin(modAsinArg);

	double mod, acc;
	calc_cal_series(asinOut, &mod, &acc);

	double BcalCurved = sinPart + curve;
	double asinArg2 = linmath_enforce_range(asinArg + mod * BcalCurved / (cosYdeg - acc * BcalCurved * sinYdeg), -1, 1);

	double asinOut2 = asin(asinArg2);
	double sinOut2 = sin(B - asinOut2 + gibPhase);

	double rtn = B - asinOut2 + sinOut2 * gibMag - phase - M_PI / 2.;
	assert(!isnan(rtn));
	return rtn;
}

static inline FLT survive_reproject_axis_x_gen2_inline(const BaseStationCal *bcal, LinmathVec3d const ptInLh) {
	return survive_reproject_axis_gen2(&bcal[0], ptInLh[0], ptInLh[1], -ptInLh[2], 0);
}

static inline FLT survive_reproject_axis_y_gen2_inline(const BaseStationCal *bcal, LinmathVec3d const ptInLh) {
	return survive_reproject_axis_gen2(&bcal[1], ptInLh[0], ptInLh[1], -ptInLh[2], 1);
}

FLT survive_reproject_axis_x_gen2(const BaseStationCal *bcal, LinmathVec3d const ptInLh) {
	return survive_reproject_axis_x_gen2_inline(bcal, ptInLh);
}

FLT survive_reproject_axis_y_gen2(const BaseStationCal *bcal, LinmathVec3d const ptInLh) {
	return survive_reproject_axis_y_gen2_inline(bcal, ptInLh);
}

void survive_reproject_xy_gen2(const BaseStationCal *bcal, LinmathVec3d const ptInLh, SurviveAngleReading out) {
	out[0] = survive_reproject_axis_x_gen2_inline(bcal, ptInLh);
	out[1] = survive_reproject_axis_y_gen2_inline(bcal, ptInLh);

	/*
	FLT X = ptInLh[0], Y = ptInLh[1], Z = ptInLh[2];
	FLT tan30 = 0.57735026919;
	FLT B = atan2(X, -Z);
	FLT A = asin(linmath_enforce_range(tan30 * Y / sqrt(X * X + Z * Z), -1, 1));

	volatile FLT u = -A - B;
	volatile FLT v = A - B;
*/
	assert(!isnan(out[0]));
	assert(!isnan(out[1]));
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
	FLT ogeePhase_0 = bcal[0].ogeephase;
	FLT ogeeMag_0 = bcal[0].ogeemag;

	gen_reproject_axis_x_jac_obj_p_gen2(out, obj_pose->Pos, obj_pt, world2lh->Pos, phase_0, tilt_0, curve_0, gibPhase_0,
										gibMag_0, ogeePhase_0, ogeeMag_0);
}

void survive_reproject_full_y_jac_obj_pose_gen2(SurviveAngleReading out, const SurvivePose *obj_pose,
												const double *obj_pt, const SurvivePose *world2lh,
												const BaseStationCal *bcal) {
	FLT phase_1 = bcal[1].phase;
	FLT tilt_1 = bcal[1].tilt;
	FLT curve_1 = bcal[1].curve;
	FLT gibPhase_1 = bcal[1].gibpha;
	FLT gibMag_1 = bcal[1].gibmag;
	FLT ogeePhase_1 = bcal[1].ogeephase;
	FLT ogeeMag_1 = bcal[1].ogeemag;

	gen_reproject_axis_y_jac_obj_p_gen2(out, obj_pose->Pos, obj_pt, world2lh->Pos, phase_1, tilt_1, curve_1, gibPhase_1,
										gibMag_1, ogeePhase_1, ogeeMag_1);
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

	FLT ogeePhase_0 = bcal[0].ogeephase;
	FLT ogeePhase_1 = bcal[1].ogeephase;
	FLT ogeeMag_0 = bcal[0].ogeemag;
	FLT ogeeMag_1 = bcal[1].ogeemag;

	gen_reproject_jac_obj_p_gen2(out, obj_pose->Pos, obj_pt, world2lh->Pos, phase_0, phase_1, tilt_0, tilt_1, curve_0,
								 curve_1, gibPhase_0, gibPhase_1, gibMag_0, gibMag_1, ogeePhase_0, ogeePhase_1,
								 ogeeMag_0, ogeeMag_1);
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
