#define _USE_MATH_DEFINES
#include <assert.h>
#include <math.h>
#include <survive_reproject.h>
#include <survive_reproject_gen2.h>

#include "force_O3.h"

#include "generated/survive_reproject.generated.h"

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
static inline void calc_cal_series(FLT s, FLT *m, FLT *a) {
	const FLT f[6] = {-8.0108022e-06, 0.0028679863, 5.3685255000000001e-06, 0.0076069798000000001};

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

	FLT B = atan2(Z, X);

	FLT Ydeg = tilt + (axis ? -1 : 1) * LINMATHPI / 6.;
	FLT tanA = FLT_TAN(Ydeg);
	FLT normXZ = FLT_SQRT(X * X + Z * Z);

	FLT asinArg = tanA * Y / normXZ;
	FLT asinArg_sanitized = linmath_enforce_range(asinArg, -1, 1);

	FLT sinYdeg = FLT_SIN(Ydeg);
	FLT cosYdeg = FLT_COS(Ydeg);

	FLT sinPart = FLT_SIN(B - FLT_ASIN(asinArg_sanitized) + ogeePhase) * ogeeMag;

	FLT normXYZ = FLT_SQRT(X * X + Y * Y + Z * Z);

	FLT modAsinArg = linmath_enforce_range(Y / normXYZ / cosYdeg, -1, 1);

	FLT asinOut = FLT_ASIN(modAsinArg);

	FLT mod, acc;
	calc_cal_series(asinOut, &mod, &acc);

	FLT BcalCurved = sinPart + curve;
	FLT asinArg2 = linmath_enforce_range(asinArg + mod * BcalCurved / (cosYdeg - acc * BcalCurved * sinYdeg), -1, 1);

	FLT asinOut2 = FLT_ASIN(asinArg2);
	FLT sinOut2 = sin(B - asinOut2 + gibPhase);

	FLT rtn = B - asinOut2 + sinOut2 * gibMag - phase - LINMATHPI_2;
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
	SurvivePose world2lh = InvertPoseRtn(survive_get_lighthouse_position(ctx, lighthouse));
	survive_reproject_from_pose_gen2(ctx, lighthouse, &world2lh, ptInWorld, out);
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
	.reprojectAxisFullFn = {gen_reproject_axis_x_gen2, gen_reproject_axis_y_gen2},

	.reprojectAxisJacobFn = {gen_reproject_axis_x_gen2_jac_obj_p, gen_reproject_axis_y_gen2_jac_obj_p},
	.reprojectFullJacObjPose = gen_reproject_gen2_jac_obj_p,
	.reprojectFullJacLhPose = gen_reproject_gen2_jac_lh_p,
	.reprojectAxisJacobLhPoseFn = {gen_reproject_axis_x_gen2_jac_lh_p, gen_reproject_axis_y_gen2_jac_lh_p},

	.reprojectAxisangleFullXyFn =
		{
			gen_reproject_axis_x_gen2_axis_angle,
			gen_reproject_axis_y_gen2_axis_angle,
		},
	.reprojectAxisAngleFullJacObjPose = gen_reproject_gen2_jac_obj_p_axis_angle,
	.reprojectAxisAngleAxisJacobFn = {gen_reproject_axis_x_gen2_jac_obj_p_axis_angle,
									  gen_reproject_axis_y_gen2_jac_obj_p_axis_angle},

	.reprojectAxisAngleFullJacLhPose = gen_reproject_gen2_jac_lh_p_axis_angle,
	.reprojectAxisAngleAxisJacobLhPoseFn = {gen_reproject_axis_x_gen2_jac_lh_p_axis_angle,
											gen_reproject_axis_y_gen2_jac_lh_p_axis_angle},
	.reprojectAxisJacobSensorPt =
		{
			gen_reproject_axis_x_gen2_jac_sensor_pt,
			gen_reproject_axis_y_gen2_jac_sensor_pt,
		},
	.reprojectAxisAngleAxisJacobSensorPt = {
		gen_reproject_axis_x_gen2_jac_sensor_pt_axis_angle,
		gen_reproject_axis_y_gen2_jac_sensor_pt_axis_angle,
	}};
