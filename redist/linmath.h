// Copyright 2013,2016 <>< C. N. Lohr.  This file licensed under the terms of the MIT/x11 license.

#ifndef _LINMATH_H
#define _LINMATH_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined(_WIN32) && !defined(TCC)
#define LINMATH_EXPORT __declspec(dllexport)
#else
#define LINMATH_EXPORT __attribute__((visibility("default")))
#endif

// Yes, I know it's kind of arbitrary.
#define DEFAULT_EPSILON 0.0000000001

// For printf
#define PFTHREE(x) (x)[0], (x)[1], (x)[2]
#define PFFOUR(x) (x)[0], (x)[1], (x)[2], (x)[3]

#define LINMATHPI ((FLT)3.141592653589)

// uncomment the following line to use double precision instead of single precision.
//#define USE_DOUBLE

#ifdef USE_FLOAT

#define FLT float
#define FLT_SQRT sqrtf
#define FLT_TAN tanf
#define FLT_SIN sinf
#define FLT_COS cosf
#define FLT_ACOS acosf
#define FLT_ASIN asinf
#define FLT_ATAN2 atan2f
#define FLT_FABS__ fabsf
#define FLT_STRTO strtof

#else

#define USE_DOUBLE 1
#define FLT double
#define FLT_SQRT sqrt
#define FLT_TAN tan
#define FLT_SIN sin
#define FLT_COS cos
#define FLT_ACOS acos
#define FLT_ASIN asin
#define FLT_ATAN2 atan2
#define FLT_FABS__ fabs
#define FLT_STRTO strtod

#endif

#ifdef TCC
#define FLT_FABS(x) (((x) < 0) ? (-(x)) : (x))
#else
#define FLT_FABS FLT_FABS__
#endif

typedef FLT LinmathQuat[4]; // This is the [wxyz] quaternion, in wxyz format.
typedef FLT LinmathPoint3d[3];
typedef FLT LinmathVec3d[3];
typedef FLT LinmathEulerAngle[3];

typedef struct LinmathEulerPose {
	LinmathPoint3d Pos;
	LinmathEulerAngle EulerRot;
} LinmathEulerPose;

typedef struct LinmathPose {
	LinmathPoint3d Pos;
	LinmathQuat Rot;
} LinmathPose;

extern LinmathQuat LinmathQuat_Identity;
extern LinmathPose LinmathPose_Identity;

// NOTE: Inputs may never be output with cross product.
LINMATH_EXPORT void cross3d(FLT *out, const FLT *a, const FLT *b);

LINMATH_EXPORT void sub3d(FLT *out, const FLT *a, const FLT *b);

LINMATH_EXPORT void add3d(FLT *out, const FLT *a, const FLT *b);

LINMATH_EXPORT void scale3d(FLT *out, const FLT *a, FLT scalar);

LINMATH_EXPORT FLT norm3d(const FLT *in);
LINMATH_EXPORT void normalize3d(FLT *out, const FLT *in);

// out_pts needs to be preallocated with 3 * num_pts FLT's; it doesn't need to be zeroed.
// out_mean can be null if you don't need the mean
LINMATH_EXPORT void center3d(FLT *out_pts, FLT *out_mean, const FLT *pts, int num_pts);
LINMATH_EXPORT void mean3d(LinmathVec3d out, const FLT *pts, int num_pts);

FLT dot3d(const FLT *a, const FLT *b);

// Returns 0 if equal.  If either argument is null, 0 will ALWAYS be returned.
int compare3d(const FLT *a, const FLT *b, FLT epsilon);

LINMATH_EXPORT void copy3d(FLT *out, const FLT *in);

FLT magnitude3d(const FLT *a);
FLT dist3d(const FLT *a, const FLT *b);
FLT anglebetween3d(FLT *a, FLT *b);

LINMATH_EXPORT void rotatearoundaxis(FLT *outvec3, FLT *invec3, FLT *axis, FLT angle);
LINMATH_EXPORT void angleaxisfrom2vect(FLT *angle, FLT *axis, FLT *src, FLT *dest);
LINMATH_EXPORT void axisanglefromquat(FLT *angle, FLT *axis, LinmathQuat quat);
// Quaternion things...

typedef FLT LinmathEulerAngle[3];

LINMATH_EXPORT void quatset(LinmathQuat q, FLT w, FLT x, FLT y, FLT z);
LINMATH_EXPORT void quatsetnone(LinmathQuat q);
LINMATH_EXPORT void quatcopy(LinmathQuat q, const LinmathQuat qin);
LINMATH_EXPORT void quatfromeuler(LinmathQuat q, const LinmathEulerAngle euler);
LINMATH_EXPORT void quattoeuler(LinmathEulerAngle euler, const LinmathQuat q);
LINMATH_EXPORT void quatfromaxisangle(LinmathQuat q, const FLT *axis, FLT radians);
FLT quatmagnitude(const LinmathQuat q);
FLT quatinvsqmagnitude(const LinmathQuat q);
LINMATH_EXPORT void quatnormalize(LinmathQuat qout, const LinmathQuat qin); // Safe for in to be same as out.
LINMATH_EXPORT void quattomatrix(FLT *matrix44, const LinmathQuat q);
LINMATH_EXPORT void quattomatrix33(FLT *matrix33, const LinmathQuat qin);
LINMATH_EXPORT void quatfrommatrix(LinmathQuat q, const FLT *matrix44);
LINMATH_EXPORT void quatfrommatrix33(LinmathQuat q, const FLT *matrix33);
LINMATH_EXPORT void quatgetconjugate(LinmathQuat qout, const LinmathQuat qin);
LINMATH_EXPORT void quatgetreciprocal(LinmathQuat qout, const LinmathQuat qin);
LINMATH_EXPORT void quatsub(LinmathQuat qout, const LinmathQuat a, const LinmathQuat b);
LINMATH_EXPORT void quatadd(LinmathQuat qout, const LinmathQuat a, const LinmathQuat b);
LINMATH_EXPORT void quatrotateabout(LinmathQuat qout, const LinmathQuat a,
									const LinmathQuat b); // same as quat multiply, not piecewise multiply.
LINMATH_EXPORT void quatscale(LinmathQuat qout, const LinmathQuat qin, FLT s);
LINMATH_EXPORT void quatdivs(LinmathQuat qout, const LinmathQuat qin, FLT s);
FLT quatinnerproduct(const LinmathQuat qa, const LinmathQuat qb);
LINMATH_EXPORT void quatouterproduct(FLT *outvec3, LinmathQuat qa, LinmathQuat qb);
LINMATH_EXPORT void quatevenproduct(LinmathQuat q, LinmathQuat qa, LinmathQuat qb);
LINMATH_EXPORT void quatoddproduct(FLT *outvec3, LinmathQuat qa, LinmathQuat qb);
LINMATH_EXPORT void quatslerp(LinmathQuat q, const LinmathQuat qa, const LinmathQuat qb, FLT t);
LINMATH_EXPORT void quatrotatevector(FLT *vec3out, const LinmathQuat quat, const FLT *vec3in);
LINMATH_EXPORT void eulerrotatevector(FLT *vec3out, const LinmathEulerAngle quat, const FLT *vec3in);
LINMATH_EXPORT void quatfrom2vectors(LinmathQuat q, const FLT *src, const FLT *dest);
LINMATH_EXPORT void eulerfrom2vectors(LinmathEulerAngle q, const FLT *src, const FLT *dest);

// This is the quat equivalent of 'pout = pose * pin' if pose were a 4x4 matrix in homogenous space
LINMATH_EXPORT void ApplyPoseToPoint(LinmathPoint3d pout, const LinmathPose *pose, const LinmathPoint3d pin);

// This is the quat equivalent of 'pout = lhs_pose * rhs_pose' if poses were a 4x4 matrix in homogenous space
LINMATH_EXPORT void ApplyPoseToPose(LinmathPose *pout, const LinmathPose *lhs_pose, const LinmathPose *rhs_pose);

// This is the quat equivlant of 'pose_in^-1'; so that ApplyPoseToPose(..., InvertPose(..., pose_in), pose_in) ==
// Identity ( [0, 0, 0], [1, 0, 0, 0] )
// by definition.
LINMATH_EXPORT void InvertPose(LinmathPose *poseout, const LinmathPose *pose_in);
static inline LinmathPose InvertPoseRtn(const LinmathPose *pose_in) {
	LinmathPose rtn;
	InvertPose(&rtn, pose_in);
	return rtn;
}

LINMATH_EXPORT void PoseToMatrix(FLT *mat44, const LinmathPose *pose_in);

// Given num_pts in coordinate system B, and in coordinate system A, find the optimal RHS rotation
// which transforms from B to A.
//
// This assumes that the space A and B share an origin.
LINMATH_EXPORT void KabschCentered(LinmathQuat B2Atx, const FLT *ptsA, const FLT *ptsB, int num_pts);

// Same as above except it solves for the center for you
LINMATH_EXPORT void Kabsch(LinmathPose *B2Atx, const FLT *ptsA, const FLT *ptsB, int num_pts);
// Matrix Stuff

typedef struct {
	FLT val[3][3]; // row, column
} Matrix3x3;

LINMATH_EXPORT void rotate_vec(FLT *out, const FLT *in, Matrix3x3 rot);
LINMATH_EXPORT void rotation_between_vecs_to_m3(Matrix3x3 *m, const FLT v1[3], const FLT v2[3]);
Matrix3x3 inverseM33(const Matrix3x3 mat);

LINMATH_EXPORT void matrix44copy(FLT *mout, const FLT *minm);
LINMATH_EXPORT void matrix44transpose(FLT *mout, const FLT *minm);

static inline FLT linmath_max(FLT x, FLT y) { return x > y ? x : y; }

static inline FLT linmath_min(FLT x, FLT y) { return x < y ? x : y; }

static inline LinmathEulerPose Pose2EulerPose(const LinmathPose *pose) {
	LinmathEulerPose p;
	copy3d(p.Pos, pose->Pos);
	quattoeuler(p.EulerRot, pose->Rot);
	return p;
}

static inline LinmathPose EulerPose2Pose(const LinmathEulerPose *pose) {
	LinmathPose p;
	copy3d(p.Pos, pose->Pos);
	quatfromeuler(p.Rot, pose->EulerRot);
	return p;
}

#ifdef __cplusplus
}
#endif

#endif
