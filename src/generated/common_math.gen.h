/// NOTE: This is a generated file; do not edit.
#pragma once
#include <cnkalman/generated_header.h>

// clang-format off
static inline void apply_pose_to_pose(SurvivePose* out, const SurvivePose* lhs, const SurvivePose* rhs) {
	const FLT x0 = (-1 * (*rhs).Pos[2] * (*lhs).Rot[1]) + ((*rhs).Pos[1] * (*lhs).Rot[0]) + ((*rhs).Pos[0] * (*lhs).Rot[3]);
	const FLT x1 = ((*rhs).Pos[1] * (*lhs).Rot[1]) + (-1 * (*rhs).Pos[0] * (*lhs).Rot[2]) + ((*rhs).Pos[2] * (*lhs).Rot[0]);
	const FLT x2 = (-1 * (*rhs).Pos[1] * (*lhs).Rot[3]) + ((*rhs).Pos[0] * (*lhs).Rot[0]) + ((*rhs).Pos[2] * (*lhs).Rot[2]);
	out->Pos[0]=(2 * ((x1 * (*lhs).Rot[2]) + (-1 * x0 * (*lhs).Rot[3]))) + (*lhs).Pos[0] + (*rhs).Pos[0];
	out->Pos[1]=(*lhs).Pos[1] + (2 * ((x2 * (*lhs).Rot[3]) + (-1 * x1 * (*lhs).Rot[1]))) + (*rhs).Pos[1];
	out->Pos[2]=(*lhs).Pos[2] + (2 * ((x0 * (*lhs).Rot[1]) + (-1 * x2 * (*lhs).Rot[2]))) + (*rhs).Pos[2];
	out->Rot[0]=(-1 * (*rhs).Rot[1] * (*lhs).Rot[1]) + ((*rhs).Rot[0] * (*lhs).Rot[0]) + (-1 * (*rhs).Rot[3] * (*lhs).Rot[3]) + (-1 * (*rhs).Rot[2] * (*lhs).Rot[2]);
	out->Rot[1]=((*rhs).Rot[0] * (*lhs).Rot[1]) + ((*rhs).Rot[3] * (*lhs).Rot[2]) + ((*rhs).Rot[1] * (*lhs).Rot[0]) + (-1 * (*rhs).Rot[2] * (*lhs).Rot[3]);
	out->Rot[2]=(-1 * (*rhs).Rot[3] * (*lhs).Rot[1]) + ((*rhs).Rot[1] * (*lhs).Rot[3]) + ((*rhs).Rot[0] * (*lhs).Rot[2]) + ((*rhs).Rot[2] * (*lhs).Rot[0]);
	out->Rot[3]=((*rhs).Rot[0] * (*lhs).Rot[3]) + ((*rhs).Rot[2] * (*lhs).Rot[1]) + ((*rhs).Rot[3] * (*lhs).Rot[0]) + (-1 * (*rhs).Rot[1] * (*lhs).Rot[2]);
}

// Jacobian of apply_pose_to_pose wrt [(*lhs).Pos[0], (*lhs).Pos[1], (*lhs).Pos[2], (*lhs).Rot[0], (*lhs).Rot[1], (*lhs).Rot[2], (*lhs).Rot[3]]
static inline void apply_pose_to_pose_jac_lhs(CnMat* Hx, const SurvivePose* lhs, const SurvivePose* rhs) {
	const FLT x0 = 2 * (*rhs).Pos[1];
	const FLT x1 = x0 * (*lhs).Rot[3];
	const FLT x2 = 2 * (*rhs).Pos[2];
	const FLT x3 = x2 * (*lhs).Rot[2];
	const FLT x4 = x2 * (*lhs).Rot[3];
	const FLT x5 = x0 * (*lhs).Rot[2];
	const FLT x6 = 2 * (*lhs).Rot[1];
	const FLT x7 = x6 * (*rhs).Pos[1];
	const FLT x8 = (*rhs).Pos[0] * (*lhs).Rot[2];
	const FLT x9 = x2 * (*lhs).Rot[0];
	const FLT x10 = x0 * (*lhs).Rot[0];
	const FLT x11 = x6 * (*rhs).Pos[2];
	const FLT x12 = 2 * (*rhs).Pos[0];
	const FLT x13 = x12 * (*lhs).Rot[3];
	const FLT x14 = 4 * (*rhs).Pos[1];
	const FLT x15 = 2 * x8;
	const FLT x16 = x6 * (*rhs).Pos[0];
	const FLT x17 = x12 * (*lhs).Rot[0];
	const FLT x18 = 4 * (*rhs).Pos[2];
	const FLT x19 = -1 * (*rhs).Rot[1];
	const FLT x20 = -1 * (*rhs).Rot[2];
	const FLT x21 = -1 * (*rhs).Rot[3];
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[0])/sizeof(FLT), offsetof(SurvivePose, Pos[0])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[0])/sizeof(FLT), offsetof(SurvivePose, Rot[0])/sizeof(FLT), x3 + (-1 * x1));
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[0])/sizeof(FLT), offsetof(SurvivePose, Rot[1])/sizeof(FLT), x5 + x4);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[0])/sizeof(FLT), offsetof(SurvivePose, Rot[2])/sizeof(FLT), x9 + x7 + (-4 * x8));
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[0])/sizeof(FLT), offsetof(SurvivePose, Rot[3])/sizeof(FLT), x11 + (-1 * x10) + (-4 * (*rhs).Pos[0] * (*lhs).Rot[3]));
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[1])/sizeof(FLT), offsetof(SurvivePose, Pos[1])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[1])/sizeof(FLT), offsetof(SurvivePose, Rot[0])/sizeof(FLT), x13 + (-1 * x11));
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[1])/sizeof(FLT), offsetof(SurvivePose, Rot[1])/sizeof(FLT), (-1 * x14 * (*lhs).Rot[1]) + (-1 * x9) + x15);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[1])/sizeof(FLT), offsetof(SurvivePose, Rot[2])/sizeof(FLT), x4 + x16);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[1])/sizeof(FLT), offsetof(SurvivePose, Rot[3])/sizeof(FLT), (-1 * x14 * (*lhs).Rot[3]) + x3 + x17);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[2])/sizeof(FLT), offsetof(SurvivePose, Pos[2])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[2])/sizeof(FLT), offsetof(SurvivePose, Rot[0])/sizeof(FLT), x7 + (-1 * x15));
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[2])/sizeof(FLT), offsetof(SurvivePose, Rot[1])/sizeof(FLT), (-1 * x18 * (*lhs).Rot[1]) + x10 + x13);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[2])/sizeof(FLT), offsetof(SurvivePose, Rot[2])/sizeof(FLT), (-1 * x18 * (*lhs).Rot[2]) + x1 + (-1 * x17));
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[2])/sizeof(FLT), offsetof(SurvivePose, Rot[3])/sizeof(FLT), x16 + x5);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[0])/sizeof(FLT), offsetof(SurvivePose, Rot[0])/sizeof(FLT), (*rhs).Rot[0]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[0])/sizeof(FLT), offsetof(SurvivePose, Rot[1])/sizeof(FLT), x19);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[0])/sizeof(FLT), offsetof(SurvivePose, Rot[2])/sizeof(FLT), x20);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[0])/sizeof(FLT), offsetof(SurvivePose, Rot[3])/sizeof(FLT), x21);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[1])/sizeof(FLT), offsetof(SurvivePose, Rot[0])/sizeof(FLT), (*rhs).Rot[1]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[1])/sizeof(FLT), offsetof(SurvivePose, Rot[1])/sizeof(FLT), (*rhs).Rot[0]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[1])/sizeof(FLT), offsetof(SurvivePose, Rot[2])/sizeof(FLT), (*rhs).Rot[3]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[1])/sizeof(FLT), offsetof(SurvivePose, Rot[3])/sizeof(FLT), x20);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[2])/sizeof(FLT), offsetof(SurvivePose, Rot[0])/sizeof(FLT), (*rhs).Rot[2]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[2])/sizeof(FLT), offsetof(SurvivePose, Rot[1])/sizeof(FLT), x21);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[2])/sizeof(FLT), offsetof(SurvivePose, Rot[2])/sizeof(FLT), (*rhs).Rot[0]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[2])/sizeof(FLT), offsetof(SurvivePose, Rot[3])/sizeof(FLT), (*rhs).Rot[1]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[3])/sizeof(FLT), offsetof(SurvivePose, Rot[0])/sizeof(FLT), (*rhs).Rot[3]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[3])/sizeof(FLT), offsetof(SurvivePose, Rot[1])/sizeof(FLT), (*rhs).Rot[2]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[3])/sizeof(FLT), offsetof(SurvivePose, Rot[2])/sizeof(FLT), x19);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[3])/sizeof(FLT), offsetof(SurvivePose, Rot[3])/sizeof(FLT), (*rhs).Rot[0]);
}

// Full version Jacobian of apply_pose_to_pose wrt [(*lhs).Pos[0], (*lhs).Pos[1], (*lhs).Pos[2], (*lhs).Rot[0], (*lhs).Rot[1], (*lhs).Rot[2], (*lhs).Rot[3]]
// Jacobian of apply_pose_to_pose wrt [(*rhs).Pos[0], (*rhs).Pos[1], (*rhs).Pos[2], (*rhs).Rot[0], (*rhs).Rot[1], (*rhs).Rot[2], (*rhs).Rot[3]]
static inline void apply_pose_to_pose_jac_rhs(CnMat* Hx, const SurvivePose* lhs, const SurvivePose* rhs) {
	const FLT x0 = -2 * ((*lhs).Rot[3] * (*lhs).Rot[3]);
	const FLT x1 = 1 + (-2 * ((*lhs).Rot[2] * (*lhs).Rot[2]));
	const FLT x2 = 2 * (*lhs).Rot[3];
	const FLT x3 = x2 * (*lhs).Rot[0];
	const FLT x4 = 2 * (*lhs).Rot[1];
	const FLT x5 = x4 * (*lhs).Rot[2];
	const FLT x6 = x4 * (*lhs).Rot[3];
	const FLT x7 = 2 * (*lhs).Rot[0] * (*lhs).Rot[2];
	const FLT x8 = -2 * ((*lhs).Rot[1] * (*lhs).Rot[1]);
	const FLT x9 = x4 * (*lhs).Rot[0];
	const FLT x10 = x2 * (*lhs).Rot[2];
	const FLT x11 = -1 * (*lhs).Rot[1];
	const FLT x12 = -1 * (*lhs).Rot[2];
	const FLT x13 = -1 * (*lhs).Rot[3];
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[0])/sizeof(FLT), offsetof(SurvivePose, Pos[0])/sizeof(FLT), x1 + x0);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[0])/sizeof(FLT), offsetof(SurvivePose, Pos[1])/sizeof(FLT), x5 + (-1 * x3));
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[0])/sizeof(FLT), offsetof(SurvivePose, Pos[2])/sizeof(FLT), x7 + x6);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[1])/sizeof(FLT), offsetof(SurvivePose, Pos[0])/sizeof(FLT), x3 + x5);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[1])/sizeof(FLT), offsetof(SurvivePose, Pos[1])/sizeof(FLT), 1 + x0 + x8);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[1])/sizeof(FLT), offsetof(SurvivePose, Pos[2])/sizeof(FLT), x10 + (-1 * x9));
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[2])/sizeof(FLT), offsetof(SurvivePose, Pos[0])/sizeof(FLT), x6 + (-1 * x7));
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[2])/sizeof(FLT), offsetof(SurvivePose, Pos[1])/sizeof(FLT), x9 + x10);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Pos[2])/sizeof(FLT), offsetof(SurvivePose, Pos[2])/sizeof(FLT), x1 + x8);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[0])/sizeof(FLT), offsetof(SurvivePose, Rot[0])/sizeof(FLT), (*lhs).Rot[0]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[0])/sizeof(FLT), offsetof(SurvivePose, Rot[1])/sizeof(FLT), x11);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[0])/sizeof(FLT), offsetof(SurvivePose, Rot[2])/sizeof(FLT), x12);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[0])/sizeof(FLT), offsetof(SurvivePose, Rot[3])/sizeof(FLT), x13);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[1])/sizeof(FLT), offsetof(SurvivePose, Rot[0])/sizeof(FLT), (*lhs).Rot[1]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[1])/sizeof(FLT), offsetof(SurvivePose, Rot[1])/sizeof(FLT), (*lhs).Rot[0]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[1])/sizeof(FLT), offsetof(SurvivePose, Rot[2])/sizeof(FLT), x13);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[1])/sizeof(FLT), offsetof(SurvivePose, Rot[3])/sizeof(FLT), (*lhs).Rot[2]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[2])/sizeof(FLT), offsetof(SurvivePose, Rot[0])/sizeof(FLT), (*lhs).Rot[2]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[2])/sizeof(FLT), offsetof(SurvivePose, Rot[1])/sizeof(FLT), (*lhs).Rot[3]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[2])/sizeof(FLT), offsetof(SurvivePose, Rot[2])/sizeof(FLT), (*lhs).Rot[0]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[2])/sizeof(FLT), offsetof(SurvivePose, Rot[3])/sizeof(FLT), x11);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[3])/sizeof(FLT), offsetof(SurvivePose, Rot[0])/sizeof(FLT), (*lhs).Rot[3]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[3])/sizeof(FLT), offsetof(SurvivePose, Rot[1])/sizeof(FLT), x12);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[3])/sizeof(FLT), offsetof(SurvivePose, Rot[2])/sizeof(FLT), (*lhs).Rot[1]);
	cnMatrixOptionalSet(Hx, offsetof(SurvivePose, Rot[3])/sizeof(FLT), offsetof(SurvivePose, Rot[3])/sizeof(FLT), (*lhs).Rot[0]);
}

// Full version Jacobian of apply_pose_to_pose wrt [(*rhs).Pos[0], (*rhs).Pos[1], (*rhs).Pos[2], (*rhs).Rot[0], (*rhs).Rot[1], (*rhs).Rot[2], (*rhs).Rot[3]]
static inline void axisangle2euler(CnMat* out, const FLT* axis_angle) {
	const FLT axis_angle0 = axis_angle[0];
	const FLT axis_angle1 = axis_angle[1];
	const FLT axis_angle2 = axis_angle[2];
	const FLT x0 = axis_angle2 * axis_angle2;
	const FLT x1 = axis_angle0 * axis_angle0;
	const FLT x2 = axis_angle1 * axis_angle1;
	const FLT x3 = 1e-10 + x2 + x0 + x1;
	const FLT x4 = sqrt(x3);
	const FLT x5 = 0.5 * x4;
	const FLT x6 = sin(x5);
	const FLT x7 = (1. / x3) * (x6 * x6);
	const FLT x8 = x0 * x7;
	const FLT x9 = x2 * x7;
	const FLT x10 = cos(x5);
	const FLT x11 = x1 * x7;
	const FLT x12 = 1. / (x11 + (x10 * x10) + x8 + x9);
	const FLT x13 = x12 * axis_angle2;
	const FLT x14 = x7 * x13;
	const FLT x15 = (1. / x4) * x6 * x10;
	const FLT x16 = x15 * x12;
	const FLT x17 = x9 * x12;
	cnMatrixOptionalSet(out, 0, 0, atan2(2 * ((x16 * axis_angle0) + (x14 * axis_angle1)), 1 + (-2 * ((x12 * x11) + x17))));
	cnMatrixOptionalSet(out, 1, 0, asin(2 * ((x16 * axis_angle1) + (-1 * x14 * axis_angle0))));
	cnMatrixOptionalSet(out, 2, 0, atan2(2 * ((x15 * x13) + (x7 * x12 * axis_angle0 * axis_angle1)), 1 + (-2 * (x17 + (x8 * x12)))));
}

// Jacobian of axisangle2euler wrt [axis_angle0, axis_angle1, axis_angle2]
static inline void axisangle2euler_jac_axis_angle(CnMat* Hx, const FLT* axis_angle) {
	const FLT axis_angle0 = axis_angle[0];
	const FLT axis_angle1 = axis_angle[1];
	const FLT axis_angle2 = axis_angle[2];
	const FLT x0 = axis_angle0 * axis_angle0;
	const FLT x1 = axis_angle2 * axis_angle2;
	const FLT x2 = axis_angle1 * axis_angle1;
	const FLT x3 = 1e-10 + x2 + x1 + x0;
	const FLT x4 = 1. / x3;
	const FLT x5 = sqrt(x3);
	const FLT x6 = 0.5 * x5;
	const FLT x7 = sin(x6);
	const FLT x8 = x7 * x7;
	const FLT x9 = x4 * x8;
	const FLT x10 = x1 * x9;
	const FLT x11 = x2 * x9;
	const FLT x12 = cos(x6);
	const FLT x13 = x12 * x12;
	const FLT x14 = x0 * x9;
	const FLT x15 = x14 + x13 + x10 + x11;
	const FLT x16 = 1. / x15;
	const FLT x17 = x0 * x16;
	const FLT x18 = 0.5 * x4 * x13;
	const FLT x19 = x7 * x12;
	const FLT x20 = (1. / (x3 * sqrt(x3))) * x19;
	const FLT x21 = x20 * x17;
	const FLT x22 = axis_angle1 * axis_angle2;
	const FLT x23 = x9 * x22;
	const FLT x24 = 1. / (x15 * x15);
	const FLT x25 = 2 * x9;
	const FLT x26 = axis_angle0 * axis_angle0 * axis_angle0;
	const FLT x27 = (1. / (x3 * x3)) * x8;
	const FLT x28 = 2 * x27;
	const FLT x29 = 1.0 * x20;
	const FLT x30 = x1 * x28;
	const FLT x31 = x30 * axis_angle0;
	const FLT x32 = x1 * axis_angle0;
	const FLT x33 = x32 * x29;
	const FLT x34 = x2 * axis_angle0;
	const FLT x35 = x34 * x29;
	const FLT x36 = x34 * x28;
	const FLT x37 = (1. / x5) * x19;
	const FLT x38 = 1.0 * x37;
	const FLT x39 = x24 * ((-1 * x38 * axis_angle0) + (-1 * x36) + (-1 * x28 * x26) + x35 + (x25 * axis_angle0) + (x29 * x26) + (-1 * x31) + x33);
	const FLT x40 = x14 * x16;
	const FLT x41 = x37 * axis_angle0;
	const FLT x42 = x37 * x16;
	const FLT x43 = x28 * axis_angle2;
	const FLT x44 = x16 * axis_angle0;
	const FLT x45 = x43 * x44 * axis_angle1;
	const FLT x46 = x44 * x22 * x29;
	const FLT x47 = x46 + x42 + (-1 * x45);
	const FLT x48 = x11 * x16;
	const FLT x49 = 1 + (-2 * (x40 + x48));
	const FLT x50 = 2 * (1. / x49);
	const FLT x51 = x26 * x16;
	const FLT x52 = 4 * x27;
	const FLT x53 = 2 * x39;
	const FLT x54 = x9 * x16;
	const FLT x55 = x54 * axis_angle0;
	const FLT x56 = x20 * x16;
	const FLT x57 = x2 * x56;
	const FLT x58 = 2.0 * axis_angle0;
	const FLT x59 = x52 * x16;
	const FLT x60 = (x59 * x34) + (x53 * x11) + (-1 * x58 * x57);
	const FLT x61 = x49 * x49;
	const FLT x62 = x54 * axis_angle2;
	const FLT x63 = x62 * axis_angle1;
	const FLT x64 = (x42 * axis_angle0) + x63;
	const FLT x65 = 2 * x64 * (1. / x61);
	const FLT x66 = x61 * (1. / (x61 + (4 * (x64 * x64))));
	const FLT x67 = x28 * axis_angle1;
	const FLT x68 = x29 * axis_angle1;
	const FLT x69 = x30 * axis_angle1;
	const FLT x70 = x1 * x68;
	const FLT x71 = axis_angle1 * axis_angle1 * axis_angle1;
	const FLT x72 = x24 * ((x25 * axis_angle1) + (x71 * x29) + (-1 * x71 * x28) + (x0 * x68) + (-1 * x69) + (-1 * x0 * x67) + (-1 * x38 * axis_angle1) + x70);
	const FLT x73 = x29 * axis_angle2;
	const FLT x74 = x2 * x73;
	const FLT x75 = x2 * x43;
	const FLT x76 = x72 * x37;
	const FLT x77 = x56 * axis_angle0;
	const FLT x78 = x18 * x16;
	const FLT x79 = x78 * axis_angle0;
	const FLT x80 = x55 * axis_angle1;
	const FLT x81 = (-0.5 * x80) + (-1 * x77 * axis_angle1) + (x79 * axis_angle1);
	const FLT x82 = 2.0 * axis_angle1;
	const FLT x83 = x0 * x59;
	const FLT x84 = 2 * x72;
	const FLT x85 = x54 * axis_angle1;
	const FLT x86 = 2.0 * x56;
	const FLT x87 = (-4 * x85) + (x84 * x11) + (-1 * x86 * x71) + (x71 * x59);
	const FLT x88 = axis_angle2 * axis_angle2 * axis_angle2;
	const FLT x89 = x24 * ((x88 * x29) + (-1 * x75) + (-1 * x0 * x43) + (-1 * x88 * x28) + (x0 * x73) + (x25 * axis_angle2) + x74 + (-1 * x38 * axis_angle2));
	const FLT x90 = x62 * axis_angle0;
	const FLT x91 = x85 + (x79 * axis_angle2) + (-1 * x77 * axis_angle2) + (-0.5 * x90);
	const FLT x92 = 2 * x89;
	const FLT x93 = 2.0 * axis_angle2;
	const FLT x94 = (x2 * x59 * axis_angle2) + (-1 * x57 * x93) + (x92 * x11);
	const FLT x95 = x37 * axis_angle1;
	const FLT x96 = x9 * axis_angle0;
	const FLT x97 = x96 * axis_angle2;
	const FLT x98 = 1.0 * x21;
	const FLT x99 = 2 * (1. / sqrt(1 + (-4 * (((x42 * axis_angle1) + (-1 * x90)) * ((x42 * axis_angle1) + (-1 * x90))))));
	const FLT x100 = (-1 * x56 * x22) + (x78 * x22) + (-0.5 * x63);
	const FLT x101 = x37 * axis_angle2;
	const FLT x102 = x96 * axis_angle1;
	const FLT x103 = x10 * x16;
	const FLT x104 = 1 + (-2 * (x48 + x103));
	const FLT x105 = 2 * (1. / x104);
	const FLT x106 = x1 * x56;
	const FLT x107 = x104 * x104;
	const FLT x108 = (x42 * axis_angle2) + x80;
	const FLT x109 = 2 * x108 * (1. / x107);
	const FLT x110 = x107 * (1. / (x107 + (4 * (x108 * x108))));
	cnMatrixOptionalSet(Hx, 0, 0, x66 * ((-1 * x65 * ((-2.0 * x51 * x20) + (x52 * x51) + x60 + (-4 * x55) + (x53 * x14))) + (x50 * ((x18 * x17) + (-1 * x21) + (-1 * x39 * x23) + x47 + (-1 * x41 * x39) + (-0.5 * x40)))));
	cnMatrixOptionalSet(Hx, 0, 1, x66 * ((-1 * x65 * (x87 + (x84 * x14) + (-1 * x82 * x21) + (x83 * axis_angle1))) + (x50 * (x81 + (x74 * x16) + (-1 * x75 * x16) + x62 + (-1 * x72 * x23) + (-1 * x76 * axis_angle0)))));
	cnMatrixOptionalSet(Hx, 0, 2, x66 * ((-1 * x65 * (x94 + (-1 * x93 * x21) + (x92 * x14) + (x83 * axis_angle2))) + (x50 * (x91 + (-1 * x69 * x16) + (-1 * x89 * x41) + (-1 * x89 * x23) + (x70 * x16)))));
	cnMatrixOptionalSet(Hx, 1, 0, x99 * (x81 + (-1 * x95 * x39) + (x97 * x39) + (-1 * x98 * axis_angle2) + (x43 * x17) + (-1 * x62)));
	cnMatrixOptionalSet(Hx, 1, 1, x99 * ((x72 * x97) + x45 + (-1 * x57) + (-0.5 * x48) + x42 + (-1 * x46) + (-1 * x76 * axis_angle1) + (x2 * x78)));
	cnMatrixOptionalSet(Hx, 1, 2, x99 * (x100 + (-1 * x55) + (x31 * x16) + (-1 * x89 * x95) + (x89 * x97) + (-1 * x33 * x16)));
	cnMatrixOptionalSet(Hx, 2, 0, x110 * ((-1 * x109 * (x60 + (-1 * x58 * x106) + (x53 * x10) + (x59 * x32))) + (x105 * ((-1 * x39 * x102) + x91 + (x98 * axis_angle1) + (-1 * x39 * x101) + (-1 * x67 * x17)))));
	cnMatrixOptionalSet(Hx, 2, 1, x110 * ((-1 * x109 * (x87 + (x1 * x59 * axis_angle1) + (-1 * x82 * x106) + (x84 * x10))) + (x105 * (x100 + (x35 * x16) + x55 + (-1 * x36 * x16) + (-1 * x72 * x102) + (-1 * x76 * axis_angle2)))));
	cnMatrixOptionalSet(Hx, 2, 2, x110 * ((-1 * x109 * (x94 + (-1 * x88 * x86) + (x92 * x10) + (-4 * x62) + (x88 * x59))) + (x105 * (x47 + (-1 * x106) + (x1 * x78) + (-0.5 * x103) + (-1 * x89 * x102) + (-1 * x89 * x101)))));
}

// Full version Jacobian of axisangle2euler wrt [axis_angle0, axis_angle1, axis_angle2]

static inline void axisangle2euler_jac_axis_angle_with_hx(CnMat* Hx, CnMat* hx, const FLT* axis_angle) {
    if(hx != 0) { 
        axisangle2euler(hx, axis_angle);
    }
    if(Hx != 0) { 
        axisangle2euler_jac_axis_angle(Hx, axis_angle);
    }
}
static inline void GenerateQuatErrorModelExact(CnMat* out, const FLT* _x1, const FLT* _x0) {
	const FLT _x10 = _x1[0];
	const FLT _x11 = _x1[1];
	const FLT _x12 = _x1[2];
	const FLT _x13 = _x1[3];
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT x0 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x1 = (-1 * _x01 * _x12) + (_x02 * _x11) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x2 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x3 = (-1 * _x01 * _x10) + (_x00 * _x11) + (_x03 * _x12) + (-1 * _x02 * _x13);
	const FLT x4 = x0 * x0;
	cnMatrixOptionalSet(out, 0, 0, atan2(2 * ((x2 * x3) + (x0 * x1)), 1 + (-2 * ((x3 * x3) + x4))));
	cnMatrixOptionalSet(out, 1, 0, asin(2 * ((x0 * x2) + (-1 * x1 * x3))));
	cnMatrixOptionalSet(out, 2, 0, atan2(2 * ((x2 * x1) + (x0 * x3)), 1 + (-2 * (x4 + (x1 * x1)))));
}

// Jacobian of GenerateQuatErrorModelExact wrt [_x10, _x11, _x12, _x13]
static inline void GenerateQuatErrorModelExact_jac_x1(CnMat* Hx, const FLT* _x1, const FLT* _x0) {
	const FLT _x10 = _x1[0];
	const FLT _x11 = _x1[1];
	const FLT _x12 = _x1[2];
	const FLT _x13 = _x1[3];
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT x0 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x1 = x0 * _x03;
	const FLT x2 = (-1 * _x01 * _x12) + (_x02 * _x11) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x3 = x2 * _x02;
	const FLT x4 = (-1 * _x01 * _x10) + (_x00 * _x11) + (_x03 * _x12) + (-1 * _x02 * _x13);
	const FLT x5 = x4 * _x00;
	const FLT x6 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x7 = x6 * _x01;
	const FLT x8 = (-1 * x7) + x5;
	const FLT x9 = x0 * x0;
	const FLT x10 = 1 + (-2 * ((x4 * x4) + x9));
	const FLT x11 = 2 * (1. / x10);
	const FLT x12 = x0 * _x02;
	const FLT x13 = 4 * x12;
	const FLT x14 = x4 * _x01;
	const FLT x15 = x10 * x10;
	const FLT x16 = (x4 * x6) + (x0 * x2);
	const FLT x17 = 2 * (1. / x15) * x16;
	const FLT x18 = x15 * (1. / (x15 + (4 * (x16 * x16))));
	const FLT x19 = x2 * _x03;
	const FLT x20 = (x6 * _x00) + x14;
	const FLT x21 = x20 + x12 + (-1 * x19);
	const FLT x22 = 4 * x1;
	const FLT x23 = x4 * _x02;
	const FLT x24 = x6 * _x03;
	const FLT x25 = x0 * _x01;
	const FLT x26 = x2 * _x00;
	const FLT x27 = x26 + (-1 * x25);
	const FLT x28 = x0 * _x00;
	const FLT x29 = -4 * x28;
	const FLT x30 = x4 * _x03;
	const FLT x31 = x6 * _x02;
	const FLT x32 = x2 * _x01;
	const FLT x33 = x32 + x28;
	const FLT x34 = (-1 * x31) + x33 + x30;
	const FLT x35 = -4 * x25;
	const FLT x36 = 2 * (1. / sqrt(1 + (-4 * (((x0 * x6) + (-1 * x2 * x4)) * ((x0 * x6) + (-1 * x2 * x4))))));
	const FLT x37 = (-1 * x23) + (-1 * x24);
	const FLT x38 = x3 + x1;
	const FLT x39 = 1 + (-2 * (x9 + (x2 * x2)));
	const FLT x40 = 2 * (1. / x39);
	const FLT x41 = x39 * x39;
	const FLT x42 = (x2 * x6) + (x0 * x4);
	const FLT x43 = 2 * (1. / x41) * x42;
	const FLT x44 = x41 * (1. / (x41 + (4 * (x42 * x42))));
	cnMatrixOptionalSet(Hx, 0, 0, x18 * ((-1 * ((4 * x14) + x13) * x17) + ((x8 + (-1 * x1) + (-1 * x3)) * x11)));
	cnMatrixOptionalSet(Hx, 0, 1, x18 * ((-1 * x17 * ((-4 * x5) + x22)) + (x21 * x11)));
	cnMatrixOptionalSet(Hx, 0, 2, x18 * ((-1 * ((-4 * x30) + x29) * x17) + (x11 * (x23 + x27 + x24))));
	cnMatrixOptionalSet(Hx, 0, 3, x18 * ((-1 * ((4 * x23) + x35) * x17) + (x34 * x11)));
	cnMatrixOptionalSet(Hx, 1, 0, x34 * x36);
	cnMatrixOptionalSet(Hx, 1, 1, x36 * (x37 + x25 + (-1 * x26)));
	cnMatrixOptionalSet(Hx, 1, 2, x36 * x21);
	cnMatrixOptionalSet(Hx, 1, 3, x36 * (x38 + x7 + (-1 * x5)));
	cnMatrixOptionalSet(Hx, 2, 0, ((-1 * (x13 + (4 * x19)) * x43) + ((x27 + x37) * x40)) * x44);
	cnMatrixOptionalSet(Hx, 2, 1, x44 * ((-1 * x43 * (x22 + (-4 * x3))) + (x40 * (x31 + x33 + (-1 * x30)))));
	cnMatrixOptionalSet(Hx, 2, 2, x44 * ((-1 * (x29 + (4 * x32)) * x43) + (x40 * (x38 + x8))));
	cnMatrixOptionalSet(Hx, 2, 3, x44 * ((-1 * (x35 + (-4 * x26)) * x43) + (x40 * (x20 + x19 + (-1 * x12)))));
}

// Full version Jacobian of GenerateQuatErrorModelExact wrt [_x10, _x11, _x12, _x13]

static inline void GenerateQuatErrorModelExact_jac_x1_with_hx(CnMat* Hx, CnMat* hx, const FLT* _x1, const FLT* _x0) {
    if(hx != 0) { 
        GenerateQuatErrorModelExact(hx, _x1, _x0);
    }
    if(Hx != 0) { 
        GenerateQuatErrorModelExact_jac_x1(Hx, _x1, _x0);
    }
}
// Jacobian of GenerateQuatErrorModelExact wrt [_x00, _x01, _x02, _x03]
static inline void GenerateQuatErrorModelExact_jac_x0(CnMat* Hx, const FLT* _x1, const FLT* _x0) {
	const FLT _x10 = _x1[0];
	const FLT _x11 = _x1[1];
	const FLT _x12 = _x1[2];
	const FLT _x13 = _x1[3];
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT x0 = (-1 * _x01 * _x10) + (_x00 * _x11) + (_x03 * _x12) + (-1 * _x02 * _x13);
	const FLT x1 = x0 * _x10;
	const FLT x2 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x3 = x2 * _x13;
	const FLT x4 = x3 + x1;
	const FLT x5 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x6 = x5 * _x11;
	const FLT x7 = (-1 * _x01 * _x12) + (_x02 * _x11) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x8 = x7 * _x12;
	const FLT x9 = x8 + x6;
	const FLT x10 = x2 * x2;
	const FLT x11 = 1 + (-2 * ((x0 * x0) + x10));
	const FLT x12 = 2 * (1. / x11);
	const FLT x13 = x2 * _x12;
	const FLT x14 = -4 * x13;
	const FLT x15 = x0 * _x11;
	const FLT x16 = x11 * x11;
	const FLT x17 = (x0 * x5) + (x2 * x7);
	const FLT x18 = 2 * x17 * (1. / x16);
	const FLT x19 = x16 * (1. / (x16 + (4 * (x17 * x17))));
	const FLT x20 = x7 * _x13;
	const FLT x21 = x20 + (-1 * x5 * _x10);
	const FLT x22 = -4 * x3;
	const FLT x23 = x5 * _x13;
	const FLT x24 = x7 * _x10;
	const FLT x25 = x2 * _x11;
	const FLT x26 = x0 * _x12;
	const FLT x27 = x26 + x25;
	const FLT x28 = x2 * _x10;
	const FLT x29 = 4 * x28;
	const FLT x30 = x0 * _x13;
	const FLT x31 = x30 + (-1 * x28);
	const FLT x32 = x5 * _x12;
	const FLT x33 = x7 * _x11;
	const FLT x34 = (-1 * x33) + x32;
	const FLT x35 = 4 * x25;
	const FLT x36 = 2 * (1. / sqrt(1 + (-4 * (((x2 * x5) + (-1 * x0 * x7)) * ((x2 * x5) + (-1 * x0 * x7))))));
	const FLT x37 = x23 + x27 + x24;
	const FLT x38 = x21 + x13 + (-1 * x15);
	const FLT x39 = 1 + (-2 * (x10 + (x7 * x7)));
	const FLT x40 = 2 * (1. / x39);
	const FLT x41 = x39 * x39;
	const FLT x42 = (x5 * x7) + (x0 * x2);
	const FLT x43 = 2 * (1. / x41) * x42;
	const FLT x44 = x41 * (1. / (x41 + (4 * (x42 * x42))));
	cnMatrixOptionalSet(Hx, 0, 0, ((-1 * ((-4 * x15) + x14) * x18) + ((x9 + x4) * x12)) * x19);
	cnMatrixOptionalSet(Hx, 0, 1, x19 * ((-1 * x18 * ((4 * x1) + x22)) + (x12 * (x21 + x15 + (-1 * x13)))));
	cnMatrixOptionalSet(Hx, 0, 2, x19 * ((-1 * ((4 * x30) + x29) * x18) + (x12 * ((-1 * x23) + x27 + (-1 * x24)))));
	cnMatrixOptionalSet(Hx, 0, 3, ((-1 * ((-4 * x26) + x35) * x18) + ((x34 + x31) * x12)) * x19);
	cnMatrixOptionalSet(Hx, 1, 0, x36 * (x34 + x28 + (-1 * x30)));
	cnMatrixOptionalSet(Hx, 1, 1, x36 * x37);
	cnMatrixOptionalSet(Hx, 1, 2, x36 * x38);
	cnMatrixOptionalSet(Hx, 1, 3, (x4 + (-1 * x6) + (-1 * x8)) * x36);
	cnMatrixOptionalSet(Hx, 2, 0, x44 * ((-1 * (x14 + (-4 * x20)) * x43) + (x40 * x37)));
	cnMatrixOptionalSet(Hx, 2, 1, x44 * ((-1 * x43 * (x22 + (4 * x8))) + (x40 * (x31 + x33 + (-1 * x32)))));
	cnMatrixOptionalSet(Hx, 2, 2, x44 * ((-1 * (x29 + (-4 * x33)) * x43) + ((x9 + (-1 * x3) + (-1 * x1)) * x40)));
	cnMatrixOptionalSet(Hx, 2, 3, x44 * ((-1 * (x35 + (4 * x24)) * x43) + (x40 * x38)));
}

// Full version Jacobian of GenerateQuatErrorModelExact wrt [_x00, _x01, _x02, _x03]

static inline void GenerateQuatErrorModelExact_jac_x0_with_hx(CnMat* Hx, CnMat* hx, const FLT* _x1, const FLT* _x0) {
    if(hx != 0) { 
        GenerateQuatErrorModelExact(hx, _x1, _x0);
    }
    if(Hx != 0) { 
        GenerateQuatErrorModelExact_jac_x0(Hx, _x1, _x0);
    }
}
static inline void GenerateQuatModelExact(CnMat* out, const FLT* _x0, const FLT* error_state) {
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT error_state0 = error_state[0];
	const FLT error_state1 = error_state[1];
	const FLT error_state2 = error_state[2];
	const FLT x0 = 0.5 * error_state0;
	const FLT x1 = sin(x0);
	const FLT x2 = 0.5 * error_state2;
	const FLT x3 = cos(x2);
	const FLT x4 = 0.5 * error_state1;
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = sin(x2);
	const FLT x8 = cos(x4);
	const FLT x9 = cos(x0);
	const FLT x10 = x8 * x9;
	const FLT x11 = (x7 * x10) + (-1 * x1 * x6);
	const FLT x12 = x1 * x8;
	const FLT x13 = (x6 * x9) + (x7 * x12);
	const FLT x14 = x5 * x7;
	const FLT x15 = (x3 * x10) + (x1 * x14);
	const FLT x16 = (x3 * x12) + (-1 * x9 * x14);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x15 * x15) + (x11 * x11) + (x13 * x13));
	const FLT x18 = x11 * x17;
	const FLT x19 = x17 * _x02;
	const FLT x20 = x15 * x17;
	const FLT x21 = x17 * _x01;
	const FLT x22 = x13 * x17;
	const FLT x23 = x17 * x16;
	cnMatrixOptionalSet(out, 0, 0, (-1 * x21 * x16) + (x20 * _x00) + (-1 * x18 * _x03) + (-1 * x13 * x19));
	cnMatrixOptionalSet(out, 1, 0, (x21 * x15) + (x23 * _x00) + (-1 * x22 * _x03) + (x11 * x19));
	cnMatrixOptionalSet(out, 2, 0, (x22 * _x00) + (x23 * _x03) + (-1 * x18 * _x01) + (x15 * x19));
	cnMatrixOptionalSet(out, 3, 0, (x22 * _x01) + (x20 * _x03) + (x18 * _x00) + (-1 * x19 * x16));
}

// Jacobian of GenerateQuatModelExact wrt [_x00, _x01, _x02, _x03]
static inline void GenerateQuatModelExact_jac_x0(CnMat* Hx, const FLT* _x0, const FLT* error_state) {
	const FLT error_state0 = error_state[0];
	const FLT error_state1 = error_state[1];
	const FLT error_state2 = error_state[2];
	const FLT x0 = 0.5 * error_state0;
	const FLT x1 = sin(x0);
	const FLT x2 = 0.5 * error_state1;
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * error_state2;
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = cos(x4);
	const FLT x8 = cos(x0);
	const FLT x9 = cos(x2);
	const FLT x10 = x8 * x9;
	const FLT x11 = (x7 * x10) + (x1 * x6);
	const FLT x12 = x3 * x7;
	const FLT x13 = (x5 * x10) + (-1 * x1 * x12);
	const FLT x14 = x1 * x9;
	const FLT x15 = (x8 * x12) + (x5 * x14);
	const FLT x16 = (x7 * x14) + (-1 * x6 * x8);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x11 * x11) + (x13 * x13) + (x15 * x15));
	const FLT x18 = x11 * x17;
	const FLT x19 = x17 * x16;
	const FLT x20 = -1 * x19;
	const FLT x21 = x15 * x17;
	const FLT x22 = -1 * x21;
	const FLT x23 = x13 * x17;
	const FLT x24 = -1 * x23;
	cnMatrixOptionalSet(Hx, 0, 0, x18);
	cnMatrixOptionalSet(Hx, 0, 1, x20);
	cnMatrixOptionalSet(Hx, 0, 2, x22);
	cnMatrixOptionalSet(Hx, 0, 3, x24);
	cnMatrixOptionalSet(Hx, 1, 0, x19);
	cnMatrixOptionalSet(Hx, 1, 1, x18);
	cnMatrixOptionalSet(Hx, 1, 2, x23);
	cnMatrixOptionalSet(Hx, 1, 3, x22);
	cnMatrixOptionalSet(Hx, 2, 0, x21);
	cnMatrixOptionalSet(Hx, 2, 1, x24);
	cnMatrixOptionalSet(Hx, 2, 2, x18);
	cnMatrixOptionalSet(Hx, 2, 3, x19);
	cnMatrixOptionalSet(Hx, 3, 0, x23);
	cnMatrixOptionalSet(Hx, 3, 1, x21);
	cnMatrixOptionalSet(Hx, 3, 2, x20);
	cnMatrixOptionalSet(Hx, 3, 3, x18);
}

// Full version Jacobian of GenerateQuatModelExact wrt [_x00, _x01, _x02, _x03]

static inline void GenerateQuatModelExact_jac_x0_with_hx(CnMat* Hx, CnMat* hx, const FLT* _x0, const FLT* error_state) {
    if(hx != 0) { 
        GenerateQuatModelExact(hx, _x0, error_state);
    }
    if(Hx != 0) { 
        GenerateQuatModelExact_jac_x0(Hx, _x0, error_state);
    }
}
// Jacobian of GenerateQuatModelExact wrt [error_state0, error_state1, error_state2]
static inline void GenerateQuatModelExact_jac_error_state(CnMat* Hx, const FLT* _x0, const FLT* error_state) {
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT error_state0 = error_state[0];
	const FLT error_state1 = error_state[1];
	const FLT error_state2 = error_state[2];
	const FLT x0 = 0.5 * error_state2;
	const FLT x1 = cos(x0);
	const FLT x2 = 0.5 * error_state1;
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * error_state0;
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = x1 * x6;
	const FLT x8 = cos(x2);
	const FLT x9 = sin(x0);
	const FLT x10 = cos(x4);
	const FLT x11 = x9 * x10;
	const FLT x12 = x8 * x11;
	const FLT x13 = x12 + (-1 * x7);
	const FLT x14 = x5 * x8;
	const FLT x15 = x9 * x14;
	const FLT x16 = x1 * x10;
	const FLT x17 = x3 * x16;
	const FLT x18 = x17 + x15;
	const FLT x19 = x6 * x9;
	const FLT x20 = x8 * x16;
	const FLT x21 = x20 + x19;
	const FLT x22 = x3 * x11;
	const FLT x23 = x1 * x14;
	const FLT x24 = x23 + (-1 * x22);
	const FLT x25 = (x24 * x24) + (x21 * x21) + (x13 * x13) + (x18 * x18);
	const FLT x26 = 1.0/2.0 * (1. / (x25 * sqrt(x25)));
	const FLT x27 = x26 * _x01;
	const FLT x28 = 0.5 * x20;
	const FLT x29 = 0.5 * x19;
	const FLT x30 = x29 + x28;
	const FLT x31 = 2 * x24;
	const FLT x32 = 0.5 * x23;
	const FLT x33 = -1 * x32;
	const FLT x34 = 0.5 * x22;
	const FLT x35 = x34 + x33;
	const FLT x36 = 2 * x21;
	const FLT x37 = 0.5 * x15;
	const FLT x38 = -0.5 * x17;
	const FLT x39 = x38 + (-1 * x37);
	const FLT x40 = 2 * x13;
	const FLT x41 = 0.5 * x7;
	const FLT x42 = -1 * x41;
	const FLT x43 = 0.5 * x12;
	const FLT x44 = x43 + x42;
	const FLT x45 = 2 * x18;
	const FLT x46 = (x44 * x45) + (x40 * x39) + (x30 * x31) + (x36 * x35);
	const FLT x47 = x46 * x24;
	const FLT x48 = 1. / sqrt(x25);
	const FLT x49 = x48 * _x00;
	const FLT x50 = x48 * _x01;
	const FLT x51 = -1 * x50 * x30;
	const FLT x52 = x26 * _x02;
	const FLT x53 = x46 * x18;
	const FLT x54 = x48 * _x02;
	const FLT x55 = x48 * x39;
	const FLT x56 = x55 * _x03;
	const FLT x57 = x46 * x13;
	const FLT x58 = x57 * x26;
	const FLT x59 = x26 * _x00;
	const FLT x60 = x59 * x21;
	const FLT x61 = -1 * x43;
	const FLT x62 = x61 + x42;
	const FLT x63 = x37 + x38;
	const FLT x64 = -1 * x34;
	const FLT x65 = x33 + x64;
	const FLT x66 = (-1 * x29) + x28;
	const FLT x67 = (x66 * x45) + (x65 * x40) + (x62 * x31) + (x63 * x36);
	const FLT x68 = x67 * x24;
	const FLT x69 = x63 * x48;
	const FLT x70 = x48 * _x03;
	const FLT x71 = x67 * x13;
	const FLT x72 = x71 * x26;
	const FLT x73 = x67 * x18;
	const FLT x74 = -1 * x55 * _x01;
	const FLT x75 = x41 + x61;
	const FLT x76 = x70 * x30;
	const FLT x77 = x26 * _x03;
	const FLT x78 = x32 + x64;
	const FLT x79 = (x78 * x45) + (x31 * x39) + (x75 * x36) + (x40 * x30);
	const FLT x80 = x79 * x13;
	const FLT x81 = x79 * x18;
	const FLT x82 = x79 * x24;
	const FLT x83 = x46 * x21;
	const FLT x84 = x55 * _x02;
	const FLT x85 = x49 * x30;
	const FLT x86 = x59 * x46;
	const FLT x87 = x67 * x21;
	const FLT x88 = x87 * x26;
	const FLT x89 = x67 * x59;
	const FLT x90 = x79 * x21;
	const FLT x91 = x55 * _x00;
	const FLT x92 = x79 * x52;
	const FLT x93 = x54 * x30;
	const FLT x94 = x52 * x24;
	cnMatrixOptionalSet(Hx, 0, 0, (x58 * _x03) + (-1 * x56) + (x49 * x35) + (x47 * x27) + (-1 * x60 * x46) + x51 + (x53 * x52) + (-1 * x54 * x44));
	cnMatrixOptionalSet(Hx, 0, 1, (-1 * x60 * x67) + (-1 * x66 * x54) + (x68 * x27) + (x72 * _x03) + (x69 * _x00) + (x73 * x52) + (-1 * x62 * x50) + (-1 * x70 * x65));
	cnMatrixOptionalSet(Hx, 0, 2, (-1 * x79 * x60) + (-1 * x78 * x54) + (x75 * x49) + (x82 * x27) + x74 + (-1 * x76) + (x80 * x77) + (x81 * x52));
	cnMatrixOptionalSet(Hx, 1, 0, (-1 * x86 * x24) + (-1 * x52 * x57) + (x50 * x35) + x85 + (-1 * x83 * x27) + (-1 * x70 * x44) + (x77 * x53) + x84);
	cnMatrixOptionalSet(Hx, 1, 1, (-1 * x71 * x52) + (x62 * x49) + (-1 * x88 * _x01) + (x69 * _x01) + (-1 * x70 * x66) + (x65 * x54) + (x73 * x77) + (-1 * x89 * x24));
	cnMatrixOptionalSet(Hx, 1, 2, (-1 * x82 * x59) + x93 + (-1 * x92 * x13) + (-1 * x90 * x27) + (x75 * x50) + x91 + (-1 * x70 * x78) + (x81 * x77));
	cnMatrixOptionalSet(Hx, 2, 0, (-1 * x86 * x18) + (-1 * x83 * x52) + (x54 * x35) + (x44 * x49) + (x58 * _x01) + x76 + x74 + (-1 * x77 * x47));
	cnMatrixOptionalSet(Hx, 2, 1, (-1 * x89 * x18) + (x69 * _x02) + (-1 * x87 * x52) + (-1 * x65 * x50) + (x70 * x62) + (x72 * _x01) + (-1 * x77 * x68) + (x66 * x49));
	cnMatrixOptionalSet(Hx, 2, 2, (-1 * x81 * x59) + (-1 * x92 * x21) + (x75 * x54) + (x78 * x49) + x51 + x56 + (x80 * x27) + (-1 * x82 * x77));
	cnMatrixOptionalSet(Hx, 3, 0, (x70 * x35) + x91 + (-1 * x53 * x27) + (x50 * x44) + (-1 * x83 * x77) + (-1 * x57 * x59) + (x94 * x46) + (-1 * x93));
	cnMatrixOptionalSet(Hx, 3, 1, (-1 * x71 * x59) + (-1 * x62 * x54) + (x67 * x94) + (x65 * x49) + (-1 * x73 * x27) + (x66 * x50) + (x69 * _x03) + (-1 * x88 * _x03));
	cnMatrixOptionalSet(Hx, 3, 2, (x82 * x52) + (-1 * x80 * x59) + (-1 * x84) + (-1 * x81 * x27) + (x78 * x50) + (x70 * x75) + x85 + (-1 * x77 * x90));
}

// Full version Jacobian of GenerateQuatModelExact wrt [error_state0, error_state1, error_state2]

static inline void GenerateQuatModelExact_jac_error_state_with_hx(CnMat* Hx, CnMat* hx, const FLT* _x0, const FLT* error_state) {
    if(hx != 0) { 
        GenerateQuatModelExact(hx, _x0, error_state);
    }
    if(Hx != 0) { 
        GenerateQuatModelExact_jac_error_state(Hx, _x0, error_state);
    }
}
static inline void GenerateQuatErrorModelApprox(CnMat* out, const FLT* _x1, const FLT* _x0) {
	const FLT _x10 = _x1[0];
	const FLT _x11 = _x1[1];
	const FLT _x12 = _x1[2];
	const FLT _x13 = _x1[3];
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT x0 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x1 = (-1 * _x01 * _x12) + (_x02 * _x11) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x2 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x3 = (-1 * _x01 * _x10) + (_x00 * _x11) + (_x03 * _x12) + (-1 * _x02 * _x13);
	const FLT x4 = x0 * x0;
	cnMatrixOptionalSet(out, 0, 0, atan2(2 * ((x2 * x3) + (x0 * x1)), 1 + (-2 * ((x3 * x3) + x4))));
	cnMatrixOptionalSet(out, 1, 0, asin(2 * ((x0 * x2) + (-1 * x1 * x3))));
	cnMatrixOptionalSet(out, 2, 0, atan2(2 * ((x2 * x1) + (x0 * x3)), 1 + (-2 * (x4 + (x1 * x1)))));
}

// Jacobian of GenerateQuatErrorModelApprox wrt [_x10, _x11, _x12, _x13]
static inline void GenerateQuatErrorModelApprox_jac_x1(CnMat* Hx, const FLT* _x1, const FLT* _x0) {
	const FLT _x10 = _x1[0];
	const FLT _x11 = _x1[1];
	const FLT _x12 = _x1[2];
	const FLT _x13 = _x1[3];
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT x0 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x1 = x0 * _x03;
	const FLT x2 = (-1 * _x01 * _x12) + (_x02 * _x11) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x3 = x2 * _x02;
	const FLT x4 = (-1 * _x01 * _x10) + (_x00 * _x11) + (_x03 * _x12) + (-1 * _x02 * _x13);
	const FLT x5 = x4 * _x00;
	const FLT x6 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x7 = x6 * _x01;
	const FLT x8 = (-1 * x7) + x5;
	const FLT x9 = x0 * x0;
	const FLT x10 = 1 + (-2 * ((x4 * x4) + x9));
	const FLT x11 = 2 * (1. / x10);
	const FLT x12 = x0 * _x02;
	const FLT x13 = 4 * x12;
	const FLT x14 = x4 * _x01;
	const FLT x15 = x10 * x10;
	const FLT x16 = (x4 * x6) + (x0 * x2);
	const FLT x17 = 2 * (1. / x15) * x16;
	const FLT x18 = x15 * (1. / (x15 + (4 * (x16 * x16))));
	const FLT x19 = x2 * _x03;
	const FLT x20 = (x6 * _x00) + x14;
	const FLT x21 = x20 + x12 + (-1 * x19);
	const FLT x22 = 4 * x1;
	const FLT x23 = x4 * _x02;
	const FLT x24 = x6 * _x03;
	const FLT x25 = x0 * _x01;
	const FLT x26 = x2 * _x00;
	const FLT x27 = x26 + (-1 * x25);
	const FLT x28 = x0 * _x00;
	const FLT x29 = -4 * x28;
	const FLT x30 = x4 * _x03;
	const FLT x31 = x6 * _x02;
	const FLT x32 = x2 * _x01;
	const FLT x33 = x32 + x28;
	const FLT x34 = (-1 * x31) + x33 + x30;
	const FLT x35 = -4 * x25;
	const FLT x36 = 2 * (1. / sqrt(1 + (-4 * (((x0 * x6) + (-1 * x2 * x4)) * ((x0 * x6) + (-1 * x2 * x4))))));
	const FLT x37 = (-1 * x23) + (-1 * x24);
	const FLT x38 = x3 + x1;
	const FLT x39 = 1 + (-2 * (x9 + (x2 * x2)));
	const FLT x40 = 2 * (1. / x39);
	const FLT x41 = x39 * x39;
	const FLT x42 = (x2 * x6) + (x0 * x4);
	const FLT x43 = 2 * (1. / x41) * x42;
	const FLT x44 = x41 * (1. / (x41 + (4 * (x42 * x42))));
	cnMatrixOptionalSet(Hx, 0, 0, x18 * ((-1 * ((4 * x14) + x13) * x17) + ((x8 + (-1 * x1) + (-1 * x3)) * x11)));
	cnMatrixOptionalSet(Hx, 0, 1, x18 * ((-1 * x17 * ((-4 * x5) + x22)) + (x21 * x11)));
	cnMatrixOptionalSet(Hx, 0, 2, x18 * ((-1 * ((-4 * x30) + x29) * x17) + (x11 * (x23 + x27 + x24))));
	cnMatrixOptionalSet(Hx, 0, 3, x18 * ((-1 * ((4 * x23) + x35) * x17) + (x34 * x11)));
	cnMatrixOptionalSet(Hx, 1, 0, x34 * x36);
	cnMatrixOptionalSet(Hx, 1, 1, x36 * (x37 + x25 + (-1 * x26)));
	cnMatrixOptionalSet(Hx, 1, 2, x36 * x21);
	cnMatrixOptionalSet(Hx, 1, 3, x36 * (x38 + x7 + (-1 * x5)));
	cnMatrixOptionalSet(Hx, 2, 0, ((-1 * (x13 + (4 * x19)) * x43) + ((x27 + x37) * x40)) * x44);
	cnMatrixOptionalSet(Hx, 2, 1, x44 * ((-1 * x43 * (x22 + (-4 * x3))) + (x40 * (x31 + x33 + (-1 * x30)))));
	cnMatrixOptionalSet(Hx, 2, 2, x44 * ((-1 * (x29 + (4 * x32)) * x43) + (x40 * (x38 + x8))));
	cnMatrixOptionalSet(Hx, 2, 3, x44 * ((-1 * (x35 + (-4 * x26)) * x43) + (x40 * (x20 + x19 + (-1 * x12)))));
}

// Full version Jacobian of GenerateQuatErrorModelApprox wrt [_x10, _x11, _x12, _x13]

static inline void GenerateQuatErrorModelApprox_jac_x1_with_hx(CnMat* Hx, CnMat* hx, const FLT* _x1, const FLT* _x0) {
    if(hx != 0) { 
        GenerateQuatErrorModelApprox(hx, _x1, _x0);
    }
    if(Hx != 0) { 
        GenerateQuatErrorModelApprox_jac_x1(Hx, _x1, _x0);
    }
}
// Jacobian of GenerateQuatErrorModelApprox wrt [_x00, _x01, _x02, _x03]
static inline void GenerateQuatErrorModelApprox_jac_x0(CnMat* Hx, const FLT* _x1, const FLT* _x0) {
	const FLT _x10 = _x1[0];
	const FLT _x11 = _x1[1];
	const FLT _x12 = _x1[2];
	const FLT _x13 = _x1[3];
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT x0 = (-1 * _x01 * _x10) + (_x00 * _x11) + (_x03 * _x12) + (-1 * _x02 * _x13);
	const FLT x1 = x0 * _x10;
	const FLT x2 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x3 = x2 * _x13;
	const FLT x4 = x3 + x1;
	const FLT x5 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x6 = x5 * _x11;
	const FLT x7 = (-1 * _x01 * _x12) + (_x02 * _x11) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x8 = x7 * _x12;
	const FLT x9 = x8 + x6;
	const FLT x10 = x2 * x2;
	const FLT x11 = 1 + (-2 * ((x0 * x0) + x10));
	const FLT x12 = 2 * (1. / x11);
	const FLT x13 = x2 * _x12;
	const FLT x14 = -4 * x13;
	const FLT x15 = x0 * _x11;
	const FLT x16 = x11 * x11;
	const FLT x17 = (x0 * x5) + (x2 * x7);
	const FLT x18 = 2 * x17 * (1. / x16);
	const FLT x19 = x16 * (1. / (x16 + (4 * (x17 * x17))));
	const FLT x20 = x7 * _x13;
	const FLT x21 = x20 + (-1 * x5 * _x10);
	const FLT x22 = -4 * x3;
	const FLT x23 = x5 * _x13;
	const FLT x24 = x7 * _x10;
	const FLT x25 = x2 * _x11;
	const FLT x26 = x0 * _x12;
	const FLT x27 = x26 + x25;
	const FLT x28 = x2 * _x10;
	const FLT x29 = 4 * x28;
	const FLT x30 = x0 * _x13;
	const FLT x31 = x30 + (-1 * x28);
	const FLT x32 = x5 * _x12;
	const FLT x33 = x7 * _x11;
	const FLT x34 = (-1 * x33) + x32;
	const FLT x35 = 4 * x25;
	const FLT x36 = 2 * (1. / sqrt(1 + (-4 * (((x2 * x5) + (-1 * x0 * x7)) * ((x2 * x5) + (-1 * x0 * x7))))));
	const FLT x37 = x23 + x27 + x24;
	const FLT x38 = x21 + x13 + (-1 * x15);
	const FLT x39 = 1 + (-2 * (x10 + (x7 * x7)));
	const FLT x40 = 2 * (1. / x39);
	const FLT x41 = x39 * x39;
	const FLT x42 = (x5 * x7) + (x0 * x2);
	const FLT x43 = 2 * (1. / x41) * x42;
	const FLT x44 = x41 * (1. / (x41 + (4 * (x42 * x42))));
	cnMatrixOptionalSet(Hx, 0, 0, ((-1 * ((-4 * x15) + x14) * x18) + ((x9 + x4) * x12)) * x19);
	cnMatrixOptionalSet(Hx, 0, 1, x19 * ((-1 * x18 * ((4 * x1) + x22)) + (x12 * (x21 + x15 + (-1 * x13)))));
	cnMatrixOptionalSet(Hx, 0, 2, x19 * ((-1 * ((4 * x30) + x29) * x18) + (x12 * ((-1 * x23) + x27 + (-1 * x24)))));
	cnMatrixOptionalSet(Hx, 0, 3, ((-1 * ((-4 * x26) + x35) * x18) + ((x34 + x31) * x12)) * x19);
	cnMatrixOptionalSet(Hx, 1, 0, x36 * (x34 + x28 + (-1 * x30)));
	cnMatrixOptionalSet(Hx, 1, 1, x36 * x37);
	cnMatrixOptionalSet(Hx, 1, 2, x36 * x38);
	cnMatrixOptionalSet(Hx, 1, 3, (x4 + (-1 * x6) + (-1 * x8)) * x36);
	cnMatrixOptionalSet(Hx, 2, 0, x44 * ((-1 * (x14 + (-4 * x20)) * x43) + (x40 * x37)));
	cnMatrixOptionalSet(Hx, 2, 1, x44 * ((-1 * x43 * (x22 + (4 * x8))) + (x40 * (x31 + x33 + (-1 * x32)))));
	cnMatrixOptionalSet(Hx, 2, 2, x44 * ((-1 * (x29 + (-4 * x33)) * x43) + ((x9 + (-1 * x3) + (-1 * x1)) * x40)));
	cnMatrixOptionalSet(Hx, 2, 3, x44 * ((-1 * (x35 + (4 * x24)) * x43) + (x40 * x38)));
}

// Full version Jacobian of GenerateQuatErrorModelApprox wrt [_x00, _x01, _x02, _x03]

static inline void GenerateQuatErrorModelApprox_jac_x0_with_hx(CnMat* Hx, CnMat* hx, const FLT* _x1, const FLT* _x0) {
    if(hx != 0) { 
        GenerateQuatErrorModelApprox(hx, _x1, _x0);
    }
    if(Hx != 0) { 
        GenerateQuatErrorModelApprox_jac_x0(Hx, _x1, _x0);
    }
}
static inline void GenerateQuatModelApprox(CnMat* out, const FLT* _x0, const FLT* error_state) {
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT error_state0 = error_state[0];
	const FLT error_state1 = error_state[1];
	const FLT error_state2 = error_state[2];
	const FLT x0 = 0.5 * _x00;
	const FLT x1 = 0.5 * _x03;
	const FLT x2 = 0.5 * _x01;
	const FLT x3 = (-1 * x2 * error_state2) + (x1 * error_state0) + _x02 + (x0 * error_state1);
	const FLT x4 = 0.5 * _x02;
	const FLT x5 = (x2 * error_state1) + (x0 * error_state2) + _x03 + (-1 * x4 * error_state0);
	const FLT x6 = (-1 * x2 * error_state0) + (-1 * x1 * error_state2) + (-1 * x4 * error_state1) + _x00;
	const FLT x7 = _x01 + (x0 * error_state0) + (-1 * x1 * error_state1) + (x4 * error_state2);
	const FLT x8 = 1. / sqrt((x7 * x7) + (x6 * x6) + (x3 * x3) + (x5 * x5));
	cnMatrixOptionalSet(out, 0, 0, x6 * x8);
	cnMatrixOptionalSet(out, 1, 0, x8 * x7);
	cnMatrixOptionalSet(out, 2, 0, x3 * x8);
	cnMatrixOptionalSet(out, 3, 0, x5 * x8);
}

// Jacobian of GenerateQuatModelApprox wrt [_x00, _x01, _x02, _x03]
static inline void GenerateQuatModelApprox_jac_x0(CnMat* Hx, const FLT* _x0, const FLT* error_state) {
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT error_state0 = error_state[0];
	const FLT error_state1 = error_state[1];
	const FLT error_state2 = error_state[2];
	const FLT x0 = 0.5 * _x00;
	const FLT x1 = 0.5 * _x03;
	const FLT x2 = 0.5 * _x01;
	const FLT x3 = (-1 * x2 * error_state2) + (x1 * error_state0) + _x02 + (x0 * error_state1);
	const FLT x4 = 0.5 * _x02;
	const FLT x5 = (x2 * error_state1) + (x0 * error_state2) + _x03 + (-1 * x4 * error_state0);
	const FLT x6 = (-1 * x2 * error_state0) + (-1 * x1 * error_state2) + (-1 * x4 * error_state1) + _x00;
	const FLT x7 = _x01 + (x0 * error_state0) + (-1 * x1 * error_state1) + (x4 * error_state2);
	const FLT x8 = (x7 * x7) + (x6 * x6) + (x3 * x3) + (x5 * x5);
	const FLT x9 = 1. / sqrt(x8);
	const FLT x10 = 1.0 * x7;
	const FLT x11 = 1.0 * x3;
	const FLT x12 = 1.0 * x5;
	const FLT x13 = 1.0/2.0 * (1. / (x8 * sqrt(x8)));
	const FLT x14 = x13 * ((x12 * error_state2) + (x11 * error_state1) + (x10 * error_state0) + (2 * x6));
	const FLT x15 = 0.5 * x9;
	const FLT x16 = x15 * error_state0;
	const FLT x17 = -1 * x16;
	const FLT x18 = 1.0 * x6;
	const FLT x19 = (x12 * error_state1) + (2 * x7) + (-1 * x11 * error_state2) + (-1 * x18 * error_state0);
	const FLT x20 = x6 * x13;
	const FLT x21 = x15 * error_state1;
	const FLT x22 = -1 * x21;
	const FLT x23 = (2 * x3) + (-1 * x18 * error_state1) + (-1 * x12 * error_state0) + (x10 * error_state2);
	const FLT x24 = x15 * error_state2;
	const FLT x25 = -1 * x24;
	const FLT x26 = (2 * x5) + (x11 * error_state0) + (-1 * x10 * error_state1) + (-1 * x18 * error_state2);
	const FLT x27 = x7 * x13;
	const FLT x28 = x3 * x13;
	const FLT x29 = x5 * x13;
	cnMatrixOptionalSet(Hx, 0, 0, (-1 * x6 * x14) + x9);
	cnMatrixOptionalSet(Hx, 0, 1, (-1 * x20 * x19) + x17);
	cnMatrixOptionalSet(Hx, 0, 2, (-1 * x23 * x20) + x22);
	cnMatrixOptionalSet(Hx, 0, 3, (-1 * x20 * x26) + x25);
	cnMatrixOptionalSet(Hx, 1, 0, (-1 * x7 * x14) + x16);
	cnMatrixOptionalSet(Hx, 1, 1, (-1 * x27 * x19) + x9);
	cnMatrixOptionalSet(Hx, 1, 2, (-1 * x23 * x27) + x24);
	cnMatrixOptionalSet(Hx, 1, 3, (-1 * x26 * x27) + x22);
	cnMatrixOptionalSet(Hx, 2, 0, (-1 * x3 * x14) + x21);
	cnMatrixOptionalSet(Hx, 2, 1, (-1 * x28 * x19) + x25);
	cnMatrixOptionalSet(Hx, 2, 2, (-1 * x23 * x28) + x9);
	cnMatrixOptionalSet(Hx, 2, 3, (-1 * x28 * x26) + x16);
	cnMatrixOptionalSet(Hx, 3, 0, (-1 * x5 * x14) + x24);
	cnMatrixOptionalSet(Hx, 3, 1, (-1 * x29 * x19) + x21);
	cnMatrixOptionalSet(Hx, 3, 2, (-1 * x23 * x29) + x17);
	cnMatrixOptionalSet(Hx, 3, 3, (-1 * x29 * x26) + x9);
}

// Full version Jacobian of GenerateQuatModelApprox wrt [_x00, _x01, _x02, _x03]

static inline void GenerateQuatModelApprox_jac_x0_with_hx(CnMat* Hx, CnMat* hx, const FLT* _x0, const FLT* error_state) {
    if(hx != 0) { 
        GenerateQuatModelApprox(hx, _x0, error_state);
    }
    if(Hx != 0) { 
        GenerateQuatModelApprox_jac_x0(Hx, _x0, error_state);
    }
}
// Jacobian of GenerateQuatModelApprox wrt [error_state0, error_state1, error_state2]
static inline void GenerateQuatModelApprox_jac_error_state(CnMat* Hx, const FLT* _x0, const FLT* error_state) {
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT error_state0 = error_state[0];
	const FLT error_state1 = error_state[1];
	const FLT error_state2 = error_state[2];
	const FLT x0 = 0.5 * _x00;
	const FLT x1 = 0.5 * _x03;
	const FLT x2 = 0.5 * _x01;
	const FLT x3 = (-1 * x2 * error_state2) + (x1 * error_state0) + _x02 + (x0 * error_state1);
	const FLT x4 = 0.5 * _x02;
	const FLT x5 = (x2 * error_state1) + (x0 * error_state2) + _x03 + (-1 * x4 * error_state0);
	const FLT x6 = (-1 * x2 * error_state0) + (-1 * x1 * error_state2) + (-1 * x4 * error_state1) + _x00;
	const FLT x7 = _x01 + (x0 * error_state0) + (-1 * x1 * error_state1) + (x4 * error_state2);
	const FLT x8 = (x7 * x7) + (x6 * x6) + (x3 * x3) + (x5 * x5);
	const FLT x9 = 1. / sqrt(x8);
	const FLT x10 = x2 * x9;
	const FLT x11 = -1 * x10;
	const FLT x12 = 1.0 * x7;
	const FLT x13 = 1.0 * x3;
	const FLT x14 = 1.0 * x6;
	const FLT x15 = 1.0 * x5;
	const FLT x16 = (-1 * x15 * _x02) + (-1 * x14 * _x01) + (x12 * _x00) + (x13 * _x03);
	const FLT x17 = 1.0/2.0 * (1. / (x8 * sqrt(x8)));
	const FLT x18 = x17 * x16;
	const FLT x19 = x4 * x9;
	const FLT x20 = -1 * x19;
	const FLT x21 = (x15 * _x01) + (x13 * _x00) + (-1 * x12 * _x03) + (-1 * x14 * _x02);
	const FLT x22 = x21 * x17;
	const FLT x23 = x1 * x9;
	const FLT x24 = -1 * x23;
	const FLT x25 = (x15 * _x00) + (x12 * _x02) + (-1 * x14 * _x03) + (-1 * x13 * _x01);
	const FLT x26 = x25 * x17;
	const FLT x27 = x0 * x9;
	const FLT x28 = x7 * x17;
	cnMatrixOptionalSet(Hx, 0, 0, (-1 * x6 * x18) + x11);
	cnMatrixOptionalSet(Hx, 0, 1, (-1 * x6 * x22) + x20);
	cnMatrixOptionalSet(Hx, 0, 2, (-1 * x6 * x26) + x24);
	cnMatrixOptionalSet(Hx, 1, 0, (-1 * x28 * x16) + x27);
	cnMatrixOptionalSet(Hx, 1, 1, (-1 * x21 * x28) + x24);
	cnMatrixOptionalSet(Hx, 1, 2, (-1 * x25 * x28) + x19);
	cnMatrixOptionalSet(Hx, 2, 0, (-1 * x3 * x18) + x23);
	cnMatrixOptionalSet(Hx, 2, 1, (-1 * x3 * x22) + x27);
	cnMatrixOptionalSet(Hx, 2, 2, (-1 * x3 * x26) + x11);
	cnMatrixOptionalSet(Hx, 3, 0, (-1 * x5 * x18) + x20);
	cnMatrixOptionalSet(Hx, 3, 1, (-1 * x5 * x22) + x10);
	cnMatrixOptionalSet(Hx, 3, 2, (-1 * x5 * x26) + x27);
}

// Full version Jacobian of GenerateQuatModelApprox wrt [error_state0, error_state1, error_state2]

static inline void GenerateQuatModelApprox_jac_error_state_with_hx(CnMat* Hx, CnMat* hx, const FLT* _x0, const FLT* error_state) {
    if(hx != 0) { 
        GenerateQuatModelApprox(hx, _x0, error_state);
    }
    if(Hx != 0) { 
        GenerateQuatModelApprox_jac_error_state(Hx, _x0, error_state);
    }
}
static inline void GenerateQuatErrorModel(CnMat* out, const FLT* _x1, const FLT* _x0) {
	const FLT _x10 = _x1[0];
	const FLT _x11 = _x1[1];
	const FLT _x12 = _x1[2];
	const FLT _x13 = _x1[3];
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT x0 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x1 = (-1 * _x01 * _x12) + (_x02 * _x11) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x2 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x3 = (-1 * _x01 * _x10) + (_x00 * _x11) + (_x03 * _x12) + (-1 * _x02 * _x13);
	const FLT x4 = x0 * x0;
	cnMatrixOptionalSet(out, 0, 0, atan2(2 * ((x2 * x3) + (x0 * x1)), 1 + (-2 * ((x3 * x3) + x4))));
	cnMatrixOptionalSet(out, 1, 0, asin(2 * ((x0 * x2) + (-1 * x1 * x3))));
	cnMatrixOptionalSet(out, 2, 0, atan2(2 * ((x2 * x1) + (x0 * x3)), 1 + (-2 * (x4 + (x1 * x1)))));
}

// Jacobian of GenerateQuatErrorModel wrt [_x10, _x11, _x12, _x13]
static inline void GenerateQuatErrorModel_jac_x1(CnMat* Hx, const FLT* _x1, const FLT* _x0) {
	const FLT _x10 = _x1[0];
	const FLT _x11 = _x1[1];
	const FLT _x12 = _x1[2];
	const FLT _x13 = _x1[3];
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT x0 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x1 = x0 * _x03;
	const FLT x2 = (-1 * _x01 * _x12) + (_x02 * _x11) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x3 = x2 * _x02;
	const FLT x4 = (-1 * _x01 * _x10) + (_x00 * _x11) + (_x03 * _x12) + (-1 * _x02 * _x13);
	const FLT x5 = x4 * _x00;
	const FLT x6 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x7 = x6 * _x01;
	const FLT x8 = (-1 * x7) + x5;
	const FLT x9 = x0 * x0;
	const FLT x10 = 1 + (-2 * ((x4 * x4) + x9));
	const FLT x11 = 2 * (1. / x10);
	const FLT x12 = x0 * _x02;
	const FLT x13 = 4 * x12;
	const FLT x14 = x4 * _x01;
	const FLT x15 = x10 * x10;
	const FLT x16 = (x4 * x6) + (x0 * x2);
	const FLT x17 = 2 * (1. / x15) * x16;
	const FLT x18 = x15 * (1. / (x15 + (4 * (x16 * x16))));
	const FLT x19 = x2 * _x03;
	const FLT x20 = (x6 * _x00) + x14;
	const FLT x21 = x20 + x12 + (-1 * x19);
	const FLT x22 = 4 * x1;
	const FLT x23 = x4 * _x02;
	const FLT x24 = x6 * _x03;
	const FLT x25 = x0 * _x01;
	const FLT x26 = x2 * _x00;
	const FLT x27 = x26 + (-1 * x25);
	const FLT x28 = x0 * _x00;
	const FLT x29 = -4 * x28;
	const FLT x30 = x4 * _x03;
	const FLT x31 = x6 * _x02;
	const FLT x32 = x2 * _x01;
	const FLT x33 = x32 + x28;
	const FLT x34 = (-1 * x31) + x33 + x30;
	const FLT x35 = -4 * x25;
	const FLT x36 = 2 * (1. / sqrt(1 + (-4 * (((x0 * x6) + (-1 * x2 * x4)) * ((x0 * x6) + (-1 * x2 * x4))))));
	const FLT x37 = (-1 * x23) + (-1 * x24);
	const FLT x38 = x3 + x1;
	const FLT x39 = 1 + (-2 * (x9 + (x2 * x2)));
	const FLT x40 = 2 * (1. / x39);
	const FLT x41 = x39 * x39;
	const FLT x42 = (x2 * x6) + (x0 * x4);
	const FLT x43 = 2 * (1. / x41) * x42;
	const FLT x44 = x41 * (1. / (x41 + (4 * (x42 * x42))));
	cnMatrixOptionalSet(Hx, 0, 0, x18 * ((-1 * ((4 * x14) + x13) * x17) + ((x8 + (-1 * x1) + (-1 * x3)) * x11)));
	cnMatrixOptionalSet(Hx, 0, 1, x18 * ((-1 * x17 * ((-4 * x5) + x22)) + (x21 * x11)));
	cnMatrixOptionalSet(Hx, 0, 2, x18 * ((-1 * ((-4 * x30) + x29) * x17) + (x11 * (x23 + x27 + x24))));
	cnMatrixOptionalSet(Hx, 0, 3, x18 * ((-1 * ((4 * x23) + x35) * x17) + (x34 * x11)));
	cnMatrixOptionalSet(Hx, 1, 0, x34 * x36);
	cnMatrixOptionalSet(Hx, 1, 1, x36 * (x37 + x25 + (-1 * x26)));
	cnMatrixOptionalSet(Hx, 1, 2, x36 * x21);
	cnMatrixOptionalSet(Hx, 1, 3, x36 * (x38 + x7 + (-1 * x5)));
	cnMatrixOptionalSet(Hx, 2, 0, ((-1 * (x13 + (4 * x19)) * x43) + ((x27 + x37) * x40)) * x44);
	cnMatrixOptionalSet(Hx, 2, 1, x44 * ((-1 * x43 * (x22 + (-4 * x3))) + (x40 * (x31 + x33 + (-1 * x30)))));
	cnMatrixOptionalSet(Hx, 2, 2, x44 * ((-1 * (x29 + (4 * x32)) * x43) + (x40 * (x38 + x8))));
	cnMatrixOptionalSet(Hx, 2, 3, x44 * ((-1 * (x35 + (-4 * x26)) * x43) + (x40 * (x20 + x19 + (-1 * x12)))));
}

// Full version Jacobian of GenerateQuatErrorModel wrt [_x10, _x11, _x12, _x13]

static inline void GenerateQuatErrorModel_jac_x1_with_hx(CnMat* Hx, CnMat* hx, const FLT* _x1, const FLT* _x0) {
    if(hx != 0) { 
        GenerateQuatErrorModel(hx, _x1, _x0);
    }
    if(Hx != 0) { 
        GenerateQuatErrorModel_jac_x1(Hx, _x1, _x0);
    }
}
// Jacobian of GenerateQuatErrorModel wrt [_x00, _x01, _x02, _x03]
static inline void GenerateQuatErrorModel_jac_x0(CnMat* Hx, const FLT* _x1, const FLT* _x0) {
	const FLT _x10 = _x1[0];
	const FLT _x11 = _x1[1];
	const FLT _x12 = _x1[2];
	const FLT _x13 = _x1[3];
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT x0 = (-1 * _x01 * _x10) + (_x00 * _x11) + (_x03 * _x12) + (-1 * _x02 * _x13);
	const FLT x1 = x0 * _x10;
	const FLT x2 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x3 = x2 * _x13;
	const FLT x4 = x3 + x1;
	const FLT x5 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x6 = x5 * _x11;
	const FLT x7 = (-1 * _x01 * _x12) + (_x02 * _x11) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x8 = x7 * _x12;
	const FLT x9 = x8 + x6;
	const FLT x10 = x2 * x2;
	const FLT x11 = 1 + (-2 * ((x0 * x0) + x10));
	const FLT x12 = 2 * (1. / x11);
	const FLT x13 = x2 * _x12;
	const FLT x14 = -4 * x13;
	const FLT x15 = x0 * _x11;
	const FLT x16 = x11 * x11;
	const FLT x17 = (x0 * x5) + (x2 * x7);
	const FLT x18 = 2 * x17 * (1. / x16);
	const FLT x19 = x16 * (1. / (x16 + (4 * (x17 * x17))));
	const FLT x20 = x7 * _x13;
	const FLT x21 = x20 + (-1 * x5 * _x10);
	const FLT x22 = -4 * x3;
	const FLT x23 = x5 * _x13;
	const FLT x24 = x7 * _x10;
	const FLT x25 = x2 * _x11;
	const FLT x26 = x0 * _x12;
	const FLT x27 = x26 + x25;
	const FLT x28 = x2 * _x10;
	const FLT x29 = 4 * x28;
	const FLT x30 = x0 * _x13;
	const FLT x31 = x30 + (-1 * x28);
	const FLT x32 = x5 * _x12;
	const FLT x33 = x7 * _x11;
	const FLT x34 = (-1 * x33) + x32;
	const FLT x35 = 4 * x25;
	const FLT x36 = 2 * (1. / sqrt(1 + (-4 * (((x2 * x5) + (-1 * x0 * x7)) * ((x2 * x5) + (-1 * x0 * x7))))));
	const FLT x37 = x23 + x27 + x24;
	const FLT x38 = x21 + x13 + (-1 * x15);
	const FLT x39 = 1 + (-2 * (x10 + (x7 * x7)));
	const FLT x40 = 2 * (1. / x39);
	const FLT x41 = x39 * x39;
	const FLT x42 = (x5 * x7) + (x0 * x2);
	const FLT x43 = 2 * (1. / x41) * x42;
	const FLT x44 = x41 * (1. / (x41 + (4 * (x42 * x42))));
	cnMatrixOptionalSet(Hx, 0, 0, ((-1 * ((-4 * x15) + x14) * x18) + ((x9 + x4) * x12)) * x19);
	cnMatrixOptionalSet(Hx, 0, 1, x19 * ((-1 * x18 * ((4 * x1) + x22)) + (x12 * (x21 + x15 + (-1 * x13)))));
	cnMatrixOptionalSet(Hx, 0, 2, x19 * ((-1 * ((4 * x30) + x29) * x18) + (x12 * ((-1 * x23) + x27 + (-1 * x24)))));
	cnMatrixOptionalSet(Hx, 0, 3, ((-1 * ((-4 * x26) + x35) * x18) + ((x34 + x31) * x12)) * x19);
	cnMatrixOptionalSet(Hx, 1, 0, x36 * (x34 + x28 + (-1 * x30)));
	cnMatrixOptionalSet(Hx, 1, 1, x36 * x37);
	cnMatrixOptionalSet(Hx, 1, 2, x36 * x38);
	cnMatrixOptionalSet(Hx, 1, 3, (x4 + (-1 * x6) + (-1 * x8)) * x36);
	cnMatrixOptionalSet(Hx, 2, 0, x44 * ((-1 * (x14 + (-4 * x20)) * x43) + (x40 * x37)));
	cnMatrixOptionalSet(Hx, 2, 1, x44 * ((-1 * x43 * (x22 + (4 * x8))) + (x40 * (x31 + x33 + (-1 * x32)))));
	cnMatrixOptionalSet(Hx, 2, 2, x44 * ((-1 * (x29 + (-4 * x33)) * x43) + ((x9 + (-1 * x3) + (-1 * x1)) * x40)));
	cnMatrixOptionalSet(Hx, 2, 3, x44 * ((-1 * (x35 + (4 * x24)) * x43) + (x40 * x38)));
}

// Full version Jacobian of GenerateQuatErrorModel wrt [_x00, _x01, _x02, _x03]

static inline void GenerateQuatErrorModel_jac_x0_with_hx(CnMat* Hx, CnMat* hx, const FLT* _x1, const FLT* _x0) {
    if(hx != 0) { 
        GenerateQuatErrorModel(hx, _x1, _x0);
    }
    if(Hx != 0) { 
        GenerateQuatErrorModel_jac_x0(Hx, _x1, _x0);
    }
}
static inline void GenerateQuatModel(CnMat* out, const FLT* _x0, const FLT* error_state) {
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT error_state0 = error_state[0];
	const FLT error_state1 = error_state[1];
	const FLT error_state2 = error_state[2];
	const FLT x0 = 0.5 * _x00;
	const FLT x1 = 0.5 * _x03;
	const FLT x2 = 0.5 * _x01;
	const FLT x3 = (-1 * x2 * error_state2) + (x1 * error_state0) + _x02 + (x0 * error_state1);
	const FLT x4 = 0.5 * _x02;
	const FLT x5 = (x2 * error_state1) + (x0 * error_state2) + _x03 + (-1 * x4 * error_state0);
	const FLT x6 = (-1 * x2 * error_state0) + (-1 * x1 * error_state2) + (-1 * x4 * error_state1) + _x00;
	const FLT x7 = _x01 + (x0 * error_state0) + (-1 * x1 * error_state1) + (x4 * error_state2);
	const FLT x8 = 1. / sqrt((x7 * x7) + (x6 * x6) + (x3 * x3) + (x5 * x5));
	cnMatrixOptionalSet(out, 0, 0, x6 * x8);
	cnMatrixOptionalSet(out, 1, 0, x8 * x7);
	cnMatrixOptionalSet(out, 2, 0, x3 * x8);
	cnMatrixOptionalSet(out, 3, 0, x5 * x8);
}

// Jacobian of GenerateQuatModel wrt [_x00, _x01, _x02, _x03]
static inline void GenerateQuatModel_jac_x0(CnMat* Hx, const FLT* _x0, const FLT* error_state) {
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT error_state0 = error_state[0];
	const FLT error_state1 = error_state[1];
	const FLT error_state2 = error_state[2];
	const FLT x0 = 0.5 * _x00;
	const FLT x1 = 0.5 * _x03;
	const FLT x2 = 0.5 * _x01;
	const FLT x3 = (-1 * x2 * error_state2) + (x1 * error_state0) + _x02 + (x0 * error_state1);
	const FLT x4 = 0.5 * _x02;
	const FLT x5 = (x2 * error_state1) + (x0 * error_state2) + _x03 + (-1 * x4 * error_state0);
	const FLT x6 = (-1 * x2 * error_state0) + (-1 * x1 * error_state2) + (-1 * x4 * error_state1) + _x00;
	const FLT x7 = _x01 + (x0 * error_state0) + (-1 * x1 * error_state1) + (x4 * error_state2);
	const FLT x8 = (x7 * x7) + (x6 * x6) + (x3 * x3) + (x5 * x5);
	const FLT x9 = 1. / sqrt(x8);
	const FLT x10 = 1.0 * x7;
	const FLT x11 = 1.0 * x3;
	const FLT x12 = 1.0 * x5;
	const FLT x13 = 1.0/2.0 * (1. / (x8 * sqrt(x8)));
	const FLT x14 = x13 * ((x12 * error_state2) + (x11 * error_state1) + (x10 * error_state0) + (2 * x6));
	const FLT x15 = 0.5 * x9;
	const FLT x16 = x15 * error_state0;
	const FLT x17 = -1 * x16;
	const FLT x18 = 1.0 * x6;
	const FLT x19 = (x12 * error_state1) + (2 * x7) + (-1 * x11 * error_state2) + (-1 * x18 * error_state0);
	const FLT x20 = x6 * x13;
	const FLT x21 = x15 * error_state1;
	const FLT x22 = -1 * x21;
	const FLT x23 = (2 * x3) + (-1 * x18 * error_state1) + (-1 * x12 * error_state0) + (x10 * error_state2);
	const FLT x24 = x15 * error_state2;
	const FLT x25 = -1 * x24;
	const FLT x26 = (2 * x5) + (x11 * error_state0) + (-1 * x10 * error_state1) + (-1 * x18 * error_state2);
	const FLT x27 = x7 * x13;
	const FLT x28 = x3 * x13;
	const FLT x29 = x5 * x13;
	cnMatrixOptionalSet(Hx, 0, 0, (-1 * x6 * x14) + x9);
	cnMatrixOptionalSet(Hx, 0, 1, (-1 * x20 * x19) + x17);
	cnMatrixOptionalSet(Hx, 0, 2, (-1 * x23 * x20) + x22);
	cnMatrixOptionalSet(Hx, 0, 3, (-1 * x20 * x26) + x25);
	cnMatrixOptionalSet(Hx, 1, 0, (-1 * x7 * x14) + x16);
	cnMatrixOptionalSet(Hx, 1, 1, (-1 * x27 * x19) + x9);
	cnMatrixOptionalSet(Hx, 1, 2, (-1 * x23 * x27) + x24);
	cnMatrixOptionalSet(Hx, 1, 3, (-1 * x26 * x27) + x22);
	cnMatrixOptionalSet(Hx, 2, 0, (-1 * x3 * x14) + x21);
	cnMatrixOptionalSet(Hx, 2, 1, (-1 * x28 * x19) + x25);
	cnMatrixOptionalSet(Hx, 2, 2, (-1 * x23 * x28) + x9);
	cnMatrixOptionalSet(Hx, 2, 3, (-1 * x28 * x26) + x16);
	cnMatrixOptionalSet(Hx, 3, 0, (-1 * x5 * x14) + x24);
	cnMatrixOptionalSet(Hx, 3, 1, (-1 * x29 * x19) + x21);
	cnMatrixOptionalSet(Hx, 3, 2, (-1 * x23 * x29) + x17);
	cnMatrixOptionalSet(Hx, 3, 3, (-1 * x29 * x26) + x9);
}

// Full version Jacobian of GenerateQuatModel wrt [_x00, _x01, _x02, _x03]

static inline void GenerateQuatModel_jac_x0_with_hx(CnMat* Hx, CnMat* hx, const FLT* _x0, const FLT* error_state) {
    if(hx != 0) { 
        GenerateQuatModel(hx, _x0, error_state);
    }
    if(Hx != 0) { 
        GenerateQuatModel_jac_x0(Hx, _x0, error_state);
    }
}
// Jacobian of GenerateQuatModel wrt [error_state0, error_state1, error_state2]
static inline void GenerateQuatModel_jac_error_state(CnMat* Hx, const FLT* _x0, const FLT* error_state) {
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT error_state0 = error_state[0];
	const FLT error_state1 = error_state[1];
	const FLT error_state2 = error_state[2];
	const FLT x0 = 0.5 * _x00;
	const FLT x1 = 0.5 * _x03;
	const FLT x2 = 0.5 * _x01;
	const FLT x3 = (-1 * x2 * error_state2) + (x1 * error_state0) + _x02 + (x0 * error_state1);
	const FLT x4 = 0.5 * _x02;
	const FLT x5 = (x2 * error_state1) + (x0 * error_state2) + _x03 + (-1 * x4 * error_state0);
	const FLT x6 = (-1 * x2 * error_state0) + (-1 * x1 * error_state2) + (-1 * x4 * error_state1) + _x00;
	const FLT x7 = _x01 + (x0 * error_state0) + (-1 * x1 * error_state1) + (x4 * error_state2);
	const FLT x8 = (x7 * x7) + (x6 * x6) + (x3 * x3) + (x5 * x5);
	const FLT x9 = 1. / sqrt(x8);
	const FLT x10 = x2 * x9;
	const FLT x11 = -1 * x10;
	const FLT x12 = 1.0 * x7;
	const FLT x13 = 1.0 * x3;
	const FLT x14 = 1.0 * x6;
	const FLT x15 = 1.0 * x5;
	const FLT x16 = (-1 * x15 * _x02) + (-1 * x14 * _x01) + (x12 * _x00) + (x13 * _x03);
	const FLT x17 = 1.0/2.0 * (1. / (x8 * sqrt(x8)));
	const FLT x18 = x17 * x16;
	const FLT x19 = x4 * x9;
	const FLT x20 = -1 * x19;
	const FLT x21 = (x15 * _x01) + (x13 * _x00) + (-1 * x12 * _x03) + (-1 * x14 * _x02);
	const FLT x22 = x21 * x17;
	const FLT x23 = x1 * x9;
	const FLT x24 = -1 * x23;
	const FLT x25 = (x15 * _x00) + (x12 * _x02) + (-1 * x14 * _x03) + (-1 * x13 * _x01);
	const FLT x26 = x25 * x17;
	const FLT x27 = x0 * x9;
	const FLT x28 = x7 * x17;
	cnMatrixOptionalSet(Hx, 0, 0, (-1 * x6 * x18) + x11);
	cnMatrixOptionalSet(Hx, 0, 1, (-1 * x6 * x22) + x20);
	cnMatrixOptionalSet(Hx, 0, 2, (-1 * x6 * x26) + x24);
	cnMatrixOptionalSet(Hx, 1, 0, (-1 * x28 * x16) + x27);
	cnMatrixOptionalSet(Hx, 1, 1, (-1 * x21 * x28) + x24);
	cnMatrixOptionalSet(Hx, 1, 2, (-1 * x25 * x28) + x19);
	cnMatrixOptionalSet(Hx, 2, 0, (-1 * x3 * x18) + x23);
	cnMatrixOptionalSet(Hx, 2, 1, (-1 * x3 * x22) + x27);
	cnMatrixOptionalSet(Hx, 2, 2, (-1 * x3 * x26) + x11);
	cnMatrixOptionalSet(Hx, 3, 0, (-1 * x5 * x18) + x20);
	cnMatrixOptionalSet(Hx, 3, 1, (-1 * x5 * x22) + x10);
	cnMatrixOptionalSet(Hx, 3, 2, (-1 * x5 * x26) + x27);
}

// Full version Jacobian of GenerateQuatModel wrt [error_state0, error_state1, error_state2]

static inline void GenerateQuatModel_jac_error_state_with_hx(CnMat* Hx, CnMat* hx, const FLT* _x0, const FLT* error_state) {
    if(hx != 0) { 
        GenerateQuatModel(hx, _x0, error_state);
    }
    if(Hx != 0) { 
        GenerateQuatModel_jac_error_state(Hx, _x0, error_state);
    }
}
