/// NOTE: This is a generated file; do not edit.
#pragma once
#include <cnkalman/generated_header.h>
// clang-format off
static inline void LighthouseIMUPrediction(CnMat *out, const SurvivePose *_x1) {
	const FLT x0 = (*_x1).Rot[2] * (*_x1).Rot[2];
	const FLT x1 = (*_x1).Rot[1] * (*_x1).Rot[1];
	const FLT x2 = 1. / (x1 + ((*_x1).Rot[0] * (*_x1).Rot[0]) + ((*_x1).Rot[3] * (*_x1).Rot[3]) + x0);
	const FLT x3 = x2 * (*_x1).Rot[1];
	const FLT x4 = x2 * (*_x1).Rot[2];
	cnMatrixOptionalSet(out, 0, 0, 2 * ((-1 * x4 * (*_x1).Rot[0]) + (x3 * (*_x1).Rot[3])));
	cnMatrixOptionalSet(out, 1, 0, 2 * ((x4 * (*_x1).Rot[3]) + (x3 * (*_x1).Rot[0])));
	cnMatrixOptionalSet(out, 2, 0, 1 + (2 * ((-1 * x2 * x1) + (-1 * x0 * x2))));
}

// Jacobian of LighthouseIMUPrediction wrt [(*_x1).Pos[0], (*_x1).Pos[1], (*_x1).Pos[2], (*_x1).Rot[0], (*_x1).Rot[1],
// (*_x1).Rot[2], (*_x1).Rot[3]]
static inline void LighthouseIMUPrediction_jac_x1(CnMat *Hx, const SurvivePose *_x1) {
	const FLT x0 = (*_x1).Rot[0] * (*_x1).Rot[0];
	const FLT x1 = (*_x1).Rot[3] * (*_x1).Rot[3];
	const FLT x2 = (*_x1).Rot[2] * (*_x1).Rot[2];
	const FLT x3 = (*_x1).Rot[1] * (*_x1).Rot[1];
	const FLT x4 = x3 + x0 + x1 + x2;
	const FLT x5 = 1. / (x4 * x4);
	const FLT x6 = 2 * x5;
	const FLT x7 = x6 * (*_x1).Rot[2];
	const FLT x8 = x6 * (*_x1).Rot[1];
	const FLT x9 = x8 * (*_x1).Rot[3];
	const FLT x10 = -1 * x9 * (*_x1).Rot[0];
	const FLT x11 = 1. / x4;
	const FLT x12 = x11 * (*_x1).Rot[2];
	const FLT x13 = x8 * (*_x1).Rot[2] * (*_x1).Rot[0];
	const FLT x14 = x11 * (*_x1).Rot[3];
	const FLT x15 = x3 * x6;
	const FLT x16 = x2 * x6;
	const FLT x17 = -1 * x9 * (*_x1).Rot[2];
	const FLT x18 = x11 * (*_x1).Rot[0];
	const FLT x19 = x7 * (*_x1).Rot[3] * (*_x1).Rot[0];
	const FLT x20 = x11 * (*_x1).Rot[1];
	const FLT x21 = 4 * x5;
	const FLT x22 = x21 * (*_x1).Rot[0];
	const FLT x23 = x2 * x21;
	const FLT x24 = x3 * x21;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[0]) / sizeof(FLT), 2 * ((-1 * x12) + (x0 * x7) + x10));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[1]) / sizeof(FLT),
						2 * ((-1 * x15 * (*_x1).Rot[3]) + x13 + x14));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[2]) / sizeof(FLT),
						2 * ((-1 * x18) + (x16 * (*_x1).Rot[0]) + x17));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[3]) / sizeof(FLT), 2 * ((-1 * x1 * x8) + x19 + x20));
	cnMatrixOptionalSet(Hx, 1, offsetof(SurvivePose, Rot[0]) / sizeof(FLT), 2 * ((-1 * x0 * x8) + (-1 * x19) + x20));
	cnMatrixOptionalSet(Hx, 1, offsetof(SurvivePose, Rot[1]) / sizeof(FLT),
						2 * (x17 + (-1 * x15 * (*_x1).Rot[0]) + x18));
	cnMatrixOptionalSet(Hx, 1, offsetof(SurvivePose, Rot[2]) / sizeof(FLT),
						2 * (x14 + (-1 * x16 * (*_x1).Rot[3]) + (-1 * x13)));
	cnMatrixOptionalSet(Hx, 1, offsetof(SurvivePose, Rot[3]) / sizeof(FLT), 2 * (x12 + (-1 * x1 * x7) + x10));
	cnMatrixOptionalSet(Hx, 2, offsetof(SurvivePose, Rot[0]) / sizeof(FLT), (x3 * x22) + (x2 * x22));
	cnMatrixOptionalSet(Hx, 2, offsetof(SurvivePose, Rot[1]) / sizeof(FLT),
						(x23 * (*_x1).Rot[1]) + (-4 * x20) + (x21 * ((*_x1).Rot[1] * (*_x1).Rot[1] * (*_x1).Rot[1])));
	cnMatrixOptionalSet(Hx, 2, offsetof(SurvivePose, Rot[2]) / sizeof(FLT),
						(-4 * x12) + (x21 * ((*_x1).Rot[2] * (*_x1).Rot[2] * (*_x1).Rot[2])) + (x24 * (*_x1).Rot[2]));
	cnMatrixOptionalSet(Hx, 2, offsetof(SurvivePose, Rot[3]) / sizeof(FLT),
						(x24 * (*_x1).Rot[3]) + (x23 * (*_x1).Rot[3]));
}

// Full version Jacobian of LighthouseIMUPrediction wrt [(*_x1).Pos[0], (*_x1).Pos[1], (*_x1).Pos[2], (*_x1).Rot[0],
// (*_x1).Rot[1], (*_x1).Rot[2], (*_x1).Rot[3]]

static inline void LighthouseIMUPrediction_jac_x1_with_hx(CnMat *Hx, CnMat *hx, const SurvivePose *_x1) {
	if (hx != 0) {
		LighthouseIMUPrediction(hx, _x1);
	}
	if (Hx != 0) {
		LighthouseIMUPrediction_jac_x1(Hx, _x1);
	}
}
static inline void LighthouseErrorIMUPrediction(CnMat *out, const SurvivePose *_x1,
												const SurviveAxisAnglePose *error_state) {
	const FLT x0 = 0.5 * (*error_state).AxisAngleRot[1];
	const FLT x1 = sin(x0);
	const FLT x2 = 0.5 * (*error_state).AxisAngleRot[2];
	const FLT x3 = cos(x2);
	const FLT x4 = 0.5 * (*error_state).AxisAngleRot[0];
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = cos(x0);
	const FLT x8 = cos(x4);
	const FLT x9 = sin(x2);
	const FLT x10 = x8 * x9;
	const FLT x11 = (x7 * x10) + (-1 * x1 * x6);
	const FLT x12 = x5 * x9;
	const FLT x13 = x3 * x8;
	const FLT x14 = (x1 * x13) + (x7 * x12);
	const FLT x15 = (x7 * x13) + (x1 * x12);
	const FLT x16 = (x6 * x7) + (-1 * x1 * x10);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x15 * x15) + (x11 * x11) + (x14 * x14));
	const FLT x18 = x15 * x17;
	const FLT x19 = x11 * x17;
	const FLT x20 = x17 * x16;
	const FLT x21 = x14 * x17;
	const FLT x22 = (-1 * x20 * (*_x1).Rot[2]) + (x18 * (*_x1).Rot[3]) + (x21 * (*_x1).Rot[1]) + (x19 * (*_x1).Rot[0]);
	const FLT x23 = (-1 * x19 * (*_x1).Rot[1]) + (x21 * (*_x1).Rot[0]) + (x20 * (*_x1).Rot[3]) + (x18 * (*_x1).Rot[2]);
	const FLT x24 = x23 * x23;
	const FLT x25 =
		(-1 * x20 * (*_x1).Rot[1]) + (-1 * x21 * (*_x1).Rot[2]) + (-1 * x19 * (*_x1).Rot[3]) + (x18 * (*_x1).Rot[0]);
	const FLT x26 = (x18 * (*_x1).Rot[1]) + (x20 * (*_x1).Rot[0]) + (x19 * (*_x1).Rot[2]) + (-1 * x21 * (*_x1).Rot[3]);
	const FLT x27 = x26 * x26;
	const FLT x28 = 1. / (x27 + (x25 * x25) + (x22 * x22) + x24);
	const FLT x29 = x28 * x26;
	const FLT x30 = x23 * x28;
	cnMatrixOptionalSet(out, 0, 0, 2 * ((-1 * x30 * x25) + (x22 * x29)));
	cnMatrixOptionalSet(out, 1, 0, 2 * ((x30 * x22) + (x25 * x29)));
	cnMatrixOptionalSet(out, 2, 0, 1 + (2 * ((-1 * x28 * x27) + (-1 * x24 * x28))));
}

// Jacobian of LighthouseErrorIMUPrediction wrt [(*_x1).Pos[0], (*_x1).Pos[1], (*_x1).Pos[2], (*_x1).Rot[0],
// (*_x1).Rot[1], (*_x1).Rot[2], (*_x1).Rot[3]]
static inline void LighthouseErrorIMUPrediction_jac_x1(CnMat *Hx, const SurvivePose *_x1,
													   const SurviveAxisAnglePose *error_state) {
	const FLT x0 = 0.5 * (*error_state).AxisAngleRot[0];
	const FLT x1 = sin(x0);
	const FLT x2 = 0.5 * (*error_state).AxisAngleRot[1];
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * (*error_state).AxisAngleRot[2];
	const FLT x5 = cos(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = sin(x4);
	const FLT x8 = cos(x0);
	const FLT x9 = cos(x2);
	const FLT x10 = x8 * x9;
	const FLT x11 = (x7 * x10) + (-1 * x1 * x6);
	const FLT x12 = x1 * x9;
	const FLT x13 = (x6 * x8) + (x7 * x12);
	const FLT x14 = x3 * x7;
	const FLT x15 = (x5 * x10) + (x1 * x14);
	const FLT x16 = (x5 * x12) + (-1 * x8 * x14);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x15 * x15) + (x11 * x11) + (x13 * x13));
	const FLT x18 = x11 * x17;
	const FLT x19 = x17 * (*_x1).Rot[3];
	const FLT x20 = x17 * x16;
	const FLT x21 = x15 * x17;
	const FLT x22 = (x21 * (*_x1).Rot[1]) + (x20 * (*_x1).Rot[0]) + (x18 * (*_x1).Rot[2]) + (-1 * x13 * x19);
	const FLT x23 = x13 * x17;
	const FLT x24 = (x23 * (*_x1).Rot[1]) + (-1 * x20 * (*_x1).Rot[2]) + (x15 * x19) + (x18 * (*_x1).Rot[0]);
	const FLT x25 = (-1 * x18 * (*_x1).Rot[1]) + (x23 * (*_x1).Rot[0]) + (x19 * x16) + (x21 * (*_x1).Rot[2]);
	const FLT x26 = x25 * x25;
	const FLT x27 = (-1 * x11 * x19) + (-1 * x20 * (*_x1).Rot[1]) + (-1 * x23 * (*_x1).Rot[2]) + (x21 * (*_x1).Rot[0]);
	const FLT x28 = x22 * x22;
	const FLT x29 = x28 + (x24 * x24) + (x27 * x27) + x26;
	const FLT x30 = 1. / x29;
	const FLT x31 = x30 * x18;
	const FLT x32 = x31 * x22;
	const FLT x33 = 2 * x20;
	const FLT x34 = 2 * x21;
	const FLT x35 = 2 * x18;
	const FLT x36 = 2 * x23;
	const FLT x37 = (x36 * x25) + (x35 * x24) + (x33 * x22) + (x34 * x27);
	const FLT x38 = 1. / (x29 * x29);
	const FLT x39 = x38 * x24;
	const FLT x40 = x39 * x22;
	const FLT x41 = x30 * x21;
	const FLT x42 = x41 * x25;
	const FLT x43 = x38 * x27;
	const FLT x44 = x43 * x25;
	const FLT x45 = x30 * x27;
	const FLT x46 = x30 * x20;
	const FLT x47 = (x46 * x24) + (-1 * x45 * x23);
	const FLT x48 = (-1 * x35 * x25) + (x36 * x24) + (x34 * x22) + (-1 * x33 * x27);
	const FLT x49 = x30 * x23;
	const FLT x50 = x49 * x22;
	const FLT x51 = x46 * x25;
	const FLT x52 = x43 * x48;
	const FLT x53 = (x41 * x24) + (x45 * x18);
	const FLT x54 = x45 * x21;
	const FLT x55 = (x34 * x25) + (-1 * x33 * x24) + (x35 * x22) + (-1 * x36 * x27);
	const FLT x56 = x31 * x24;
	const FLT x57 = x46 * x22;
	const FLT x58 = x49 * x25;
	const FLT x59 = x58 + (-1 * x57);
	const FLT x60 = (-1 * x35 * x27) + (-1 * x36 * x22) + (x33 * x25) + (x34 * x24);
	const FLT x61 = x49 * x24;
	const FLT x62 = x45 * x20;
	const FLT x63 = x41 * x22;
	const FLT x64 = x31 * x25;
	const FLT x65 = x64 + x63;
	const FLT x66 = x43 * x22;
	const FLT x67 = x39 * x25;
	const FLT x68 = 2 * x38;
	const FLT x69 = x68 * x26;
	const FLT x70 = x68 * x28;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[0]) / sizeof(FLT),
						2 * (x47 + (x44 * x37) + (-1 * x42) + x32 + (-1 * x40 * x37)));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[1]) / sizeof(FLT),
						2 * (x51 + (-1 * x40 * x48) + x53 + (x52 * x25) + x50));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[2]) / sizeof(FLT),
						2 * ((x55 * x44) + x59 + x56 + (-1 * x54) + (-1 * x55 * x40)));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[3]) / sizeof(FLT),
						2 * ((x60 * x44) + (-1 * x62) + (-1 * x60 * x40) + x65 + (-1 * x61)));
	cnMatrixOptionalSet(Hx, 1, offsetof(SurvivePose, Rot[0]) / sizeof(FLT),
						2 * (x65 + x61 + (-1 * x67 * x37) + x62 + (-1 * x66 * x37)));
	cnMatrixOptionalSet(Hx, 1, offsetof(SurvivePose, Rot[1]) / sizeof(FLT),
						2 * (x59 + (-1 * x56) + (-1 * x67 * x48) + (-1 * x52 * x22) + x54));
	cnMatrixOptionalSet(Hx, 1, offsetof(SurvivePose, Rot[2]) / sizeof(FLT),
						2 * ((-1 * x67 * x55) + (-1 * x51) + x53 + (-1 * x66 * x55) + (-1 * x50)));
	cnMatrixOptionalSet(Hx, 1, offsetof(SurvivePose, Rot[3]) / sizeof(FLT),
						2 * ((-1 * x60 * x67) + (-1 * x32) + x42 + x47 + (-1 * x60 * x66)));
	cnMatrixOptionalSet(Hx, 2, offsetof(SurvivePose, Rot[0]) / sizeof(FLT),
						(-4 * x57) + (x70 * x37) + (x69 * x37) + (-4 * x58));
	cnMatrixOptionalSet(Hx, 2, offsetof(SurvivePose, Rot[1]) / sizeof(FLT),
						(4 * x64) + (x70 * x48) + (-4 * x63) + (x69 * x48));
	cnMatrixOptionalSet(Hx, 2, offsetof(SurvivePose, Rot[2]) / sizeof(FLT),
						(x70 * x55) + (-4 * x32) + (x69 * x55) + (-4 * x42));
	cnMatrixOptionalSet(Hx, 2, offsetof(SurvivePose, Rot[3]) / sizeof(FLT),
						(4 * x50) + (x70 * x60) + (x60 * x69) + (-4 * x51));
}

// Full version Jacobian of LighthouseErrorIMUPrediction wrt [(*_x1).Pos[0], (*_x1).Pos[1], (*_x1).Pos[2],
// (*_x1).Rot[0], (*_x1).Rot[1], (*_x1).Rot[2], (*_x1).Rot[3]]

static inline void LighthouseErrorIMUPrediction_jac_x1_with_hx(CnMat *Hx, CnMat *hx, const SurvivePose *_x1,
															   const SurviveAxisAnglePose *error_state) {
	if (hx != 0) {
		LighthouseErrorIMUPrediction(hx, _x1, error_state);
	}
	if (Hx != 0) {
		LighthouseErrorIMUPrediction_jac_x1(Hx, _x1, error_state);
	}
}
// Jacobian of LighthouseErrorIMUPrediction wrt [(*error_state).AxisAngleRot[0], (*error_state).AxisAngleRot[1],
// (*error_state).AxisAngleRot[2], (*error_state).Pos[0], (*error_state).Pos[1], (*error_state).Pos[2]]
static inline void LighthouseErrorIMUPrediction_jac_error_state(CnMat *Hx, const SurvivePose *_x1,
																const SurviveAxisAnglePose *error_state) {
	const FLT x0 = 0.5 * (*error_state).AxisAngleRot[1];
	const FLT x1 = sin(x0);
	const FLT x2 = 0.5 * (*error_state).AxisAngleRot[2];
	const FLT x3 = cos(x2);
	const FLT x4 = 0.5 * (*error_state).AxisAngleRot[0];
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = x1 * x6;
	const FLT x8 = sin(x2);
	const FLT x9 = cos(x4);
	const FLT x10 = cos(x0);
	const FLT x11 = x9 * x10;
	const FLT x12 = x8 * x11;
	const FLT x13 = x12 + (-1 * x7);
	const FLT x14 = x3 * x11;
	const FLT x15 = 0.5 * x14;
	const FLT x16 = x5 * x8;
	const FLT x17 = x1 * x16;
	const FLT x18 = 0.5 * x17;
	const FLT x19 = x18 + x15;
	const FLT x20 = x1 * x9;
	const FLT x21 = x8 * x20;
	const FLT x22 = x6 * x10;
	const FLT x23 = x22 + (-1 * x21);
	const FLT x24 = 2 * x23;
	const FLT x25 = 0.5 * x22;
	const FLT x26 = -1 * x25;
	const FLT x27 = 0.5 * x21;
	const FLT x28 = x27 + x26;
	const FLT x29 = x14 + x17;
	const FLT x30 = 2 * x29;
	const FLT x31 = x10 * x16;
	const FLT x32 = 0.5 * x31;
	const FLT x33 = x3 * x20;
	const FLT x34 = -0.5 * x33;
	const FLT x35 = x34 + (-1 * x32);
	const FLT x36 = 2 * x13;
	const FLT x37 = 0.5 * x7;
	const FLT x38 = -1 * x37;
	const FLT x39 = 0.5 * x12;
	const FLT x40 = x39 + x38;
	const FLT x41 = x33 + x31;
	const FLT x42 = 2 * x41;
	const FLT x43 = (x40 * x42) + (x36 * x35) + (x24 * x19) + (x30 * x28);
	const FLT x44 = (x23 * x23) + (x29 * x29) + (x13 * x13) + (x41 * x41);
	const FLT x45 = 1.0 / 2.0 * (1. / (x44 * sqrt(x44)));
	const FLT x46 = x43 * x45;
	const FLT x47 = x46 * x13;
	const FLT x48 = x45 * (*_x1).Rot[0];
	const FLT x49 = x41 * x48;
	const FLT x50 = x46 * x23;
	const FLT x51 = 1. / sqrt(x44);
	const FLT x52 = x51 * (*_x1).Rot[1];
	const FLT x53 = -1 * x52 * x35;
	const FLT x54 = x46 * x29;
	const FLT x55 = x51 * (*_x1).Rot[3];
	const FLT x56 = x55 * x19;
	const FLT x57 = x51 * x28;
	const FLT x58 = x51 * (*_x1).Rot[0];
	const FLT x59 = (x58 * x40) + x56 + (x57 * (*_x1).Rot[2]) + (-1 * x43 * x49) + (x47 * (*_x1).Rot[1]) +
					(-1 * x54 * (*_x1).Rot[2]) + (-1 * x50 * (*_x1).Rot[3]) + x53;
	const FLT x60 = x51 * (*_x1).Rot[2];
	const FLT x61 = x51 * x41;
	const FLT x62 = (x61 * (*_x1).Rot[1]) + (-1 * x60 * x23) + (x55 * x29) + (x58 * x13);
	const FLT x63 = (x61 * (*_x1).Rot[0]) + (x55 * x23) + (-1 * x52 * x13) + (x60 * x29);
	const FLT x64 = x63 * x63;
	const FLT x65 = (-1 * x52 * x23) + (-1 * x55 * x13) + (-1 * x61 * (*_x1).Rot[2]) + (x58 * x29);
	const FLT x66 = (x58 * x23) + (x60 * x13) + (x52 * x29) + (-1 * x61 * (*_x1).Rot[3]);
	const FLT x67 = x66 * x66;
	const FLT x68 = x67 + (x65 * x65) + (x62 * x62) + x64;
	const FLT x69 = 1. / x68;
	const FLT x70 = x65 * x69;
	const FLT x71 = x48 * x23;
	const FLT x72 = x41 * (*_x1).Rot[3];
	const FLT x73 = x60 * x35;
	const FLT x74 = x58 * x19;
	const FLT x75 = x73 + (-1 * x55 * x40) + (x57 * (*_x1).Rot[1]) + x74 + (-1 * x54 * (*_x1).Rot[1]) +
					(-1 * x71 * x43) + (-1 * x47 * (*_x1).Rot[2]) + (x72 * x46);
	const FLT x76 = x62 * x69;
	const FLT x77 = 2 * x66;
	const FLT x78 = x41 * x46;
	const FLT x79 = x55 * x35;
	const FLT x80 = -1 * x52 * x19;
	const FLT x81 = x48 * x29;
	const FLT x82 = (-1 * x60 * x40) + (-1 * x81 * x43) + (x78 * (*_x1).Rot[2]) + (x50 * (*_x1).Rot[1]) +
					(x47 * (*_x1).Rot[3]) + (x57 * (*_x1).Rot[0]) + x80 + (-1 * x79);
	const FLT x83 = 2 * x65;
	const FLT x84 = x58 * x35;
	const FLT x85 = x48 * x13;
	const FLT x86 = x60 * x19;
	const FLT x87 = (-1 * x85 * x43) + x84 + (x52 * x40) + (-1 * x86) + (-1 * x78 * (*_x1).Rot[1]) +
					(x57 * (*_x1).Rot[3]) + (x50 * (*_x1).Rot[2]) + (-1 * x54 * (*_x1).Rot[3]);
	const FLT x88 = 2 * x62;
	const FLT x89 = 2 * x63;
	const FLT x90 = (x89 * x59) + (x88 * x87) + (x75 * x77) + (x82 * x83);
	const FLT x91 = 1. / (x68 * x68);
	const FLT x92 = x63 * x91;
	const FLT x93 = x65 * x92;
	const FLT x94 = x91 * x90;
	const FLT x95 = x66 * x94;
	const FLT x96 = x66 * x69;
	const FLT x97 = x63 * x69;
	const FLT x98 = -1 * x27;
	const FLT x99 = x26 + x98;
	const FLT x100 = -1 * x39;
	const FLT x101 = x100 + x38;
	const FLT x102 = x32 + x34;
	const FLT x103 = (-1 * x18) + x15;
	const FLT x104 = (x42 * x103) + (x24 * x101) + (x99 * x36) + (x30 * x102);
	const FLT x105 = x45 * x104;
	const FLT x106 = x13 * x105;
	const FLT x107 = x23 * x104;
	const FLT x108 = x45 * x107;
	const FLT x109 = x29 * x104;
	const FLT x110 = x45 * x109;
	const FLT x111 = (-1 * x110 * (*_x1).Rot[2]) + (x58 * x103) + (-1 * x108 * (*_x1).Rot[3]) + (-1 * x49 * x104) +
					 (-1 * x52 * x99) + (x55 * x101) + (x60 * x102) + (x106 * (*_x1).Rot[1]);
	const FLT x112 = (x72 * x105) + (-1 * x106 * (*_x1).Rot[2]) + (x60 * x99) + (-1 * x110 * (*_x1).Rot[1]) +
					 (-1 * x48 * x107) + (x58 * x101) + (x52 * x102) + (-1 * x55 * x103);
	const FLT x113 = x41 * x105;
	const FLT x114 = (x58 * x99) + (-1 * x85 * x104) + (x55 * x102) + (x108 * (*_x1).Rot[2]) + (x52 * x103) +
					 (-1 * x113 * (*_x1).Rot[1]) + (-1 * x60 * x101) + (-1 * x110 * (*_x1).Rot[3]);
	const FLT x115 = (-1 * x55 * x99) + (x58 * x102) + (x108 * (*_x1).Rot[1]) + (-1 * x60 * x103) +
					 (x106 * (*_x1).Rot[3]) + (-1 * x52 * x101) + (x113 * (*_x1).Rot[2]) + (-1 * x48 * x109);
	const FLT x116 = (x89 * x111) + (x88 * x114) + (x77 * x112) + (x83 * x115);
	const FLT x117 = x91 * x116;
	const FLT x118 = x66 * x117;
	const FLT x119 = x37 + x100;
	const FLT x120 = x25 + x98;
	const FLT x121 = (x42 * x120) + (x36 * x19) + (x35 * x24) + (x30 * x119);
	const FLT x122 = x45 * x121;
	const FLT x123 = x23 * x122;
	const FLT x124 = x122 * (*_x1).Rot[2];
	const FLT x125 = x13 * x122;
	const FLT x126 = x51 * x120;
	const FLT x127 = (-1 * x126 * (*_x1).Rot[2]) + (x58 * x119) + x53 + (x41 * x124) + (x125 * (*_x1).Rot[3]) +
					 (-1 * x81 * x121) + (x123 * (*_x1).Rot[1]) + (-1 * x56);
	const FLT x128 = x29 * x122;
	const FLT x129 = x84 + (x72 * x122) + (-1 * x71 * x121) + (-1 * x125 * (*_x1).Rot[2]) +
					 (-1 * x126 * (*_x1).Rot[3]) + (x52 * x119) + x86 + (-1 * x128 * (*_x1).Rot[1]);
	const FLT x130 = (-1 * x85 * x121) + (-1 * x73) + (x126 * (*_x1).Rot[1]) + (-1 * x41 * x122 * (*_x1).Rot[1]) +
					 (x123 * (*_x1).Rot[2]) + x74 + (x55 * x119) + (-1 * x128 * (*_x1).Rot[3]);
	const FLT x131 = (x126 * (*_x1).Rot[0]) + (x60 * x119) + (-1 * x49 * x121) + x80 + x79 + (x125 * (*_x1).Rot[1]) +
					 (-1 * x29 * x124) + (-1 * x123 * (*_x1).Rot[3]);
	const FLT x132 = (x89 * x131) + (x88 * x130) + (x83 * x127) + (x77 * x129);
	const FLT x133 = x91 * x132;
	const FLT x134 = x66 * x133;
	const FLT x135 = x62 * x92;
	const FLT x136 = 2 * x64;
	const FLT x137 = 4 * x97;
	const FLT x138 = 2 * x67;
	const FLT x139 = 4 * x96;
	const FLT x140 = 2 * x117;
	cnSetZero(Hx);
	cnMatrixOptionalSet(
		Hx, 0, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
		2 * ((-1 * x82 * x97) + (x75 * x76) + (-1 * x70 * x59) + (x93 * x90) + (-1 * x62 * x95) + (x87 * x96)));
	cnMatrixOptionalSet(
		Hx, 0, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
		2 * ((-1 * x97 * x115) + (x76 * x112) + (x96 * x114) + (x93 * x116) + (-1 * x70 * x111) + (-1 * x62 * x118)));
	cnMatrixOptionalSet(
		Hx, 0, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
		2 * ((x96 * x130) + (-1 * x62 * x134) + (-1 * x97 * x127) + (x93 * x132) + (x76 * x129) + (-1 * x70 * x131)));
	cnMatrixOptionalSet(
		Hx, 1, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
		2 * ((x82 * x96) + (x87 * x97) + (-1 * x65 * x95) + (x70 * x75) + (-1 * x90 * x135) + (x76 * x59)));
	cnMatrixOptionalSet(
		Hx, 1, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
		2 * ((-1 * x116 * x135) + (x70 * x112) + (-1 * x65 * x118) + (x97 * x114) + (x76 * x111) + (x96 * x115)));
	cnMatrixOptionalSet(
		Hx, 1, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
		2 * ((-1 * x132 * x135) + (-1 * x65 * x134) + (x70 * x129) + (x76 * x131) + (x97 * x130) + (x96 * x127)));
	cnMatrixOptionalSet(Hx, 2, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						(x94 * x138) + (-1 * x75 * x139) + (x94 * x136) + (-1 * x59 * x137));
	cnMatrixOptionalSet(Hx, 2, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						(-1 * x112 * x139) + (x67 * x140) + (x64 * x140) + (-1 * x111 * x137));
	cnMatrixOptionalSet(Hx, 2, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						(-1 * x131 * x137) + (-1 * x129 * x139) + (x133 * x136) + (x133 * x138));
}

// Full version Jacobian of LighthouseErrorIMUPrediction wrt [(*error_state).AxisAngleRot[0],
// (*error_state).AxisAngleRot[1], (*error_state).AxisAngleRot[2], (*error_state).Pos[0], (*error_state).Pos[1],
// (*error_state).Pos[2]]

static inline void LighthouseErrorIMUPrediction_jac_error_state_with_hx(CnMat *Hx, CnMat *hx, const SurvivePose *_x1,
																		const SurviveAxisAnglePose *error_state) {
	if (hx != 0) {
		LighthouseErrorIMUPrediction(hx, _x1, error_state);
	}
	if (Hx != 0) {
		LighthouseErrorIMUPrediction_jac_error_state(Hx, _x1, error_state);
	}
}
static inline void SurviveLighthouseKalmanModelToErrorModel(SurviveLighthouseKalmanErrorModel *out,
															const SurviveLighthouseKalmanModel *_x1,
															const SurviveLighthouseKalmanModel *_x0) {
	const FLT x0 = ((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[1]) +
				   (-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[2]) +
				   ((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[0]) +
				   (-1 * (*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[3]);
	const FLT x1 = (-1 * (*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[1]) +
				   ((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[0]) +
				   (-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[3]) +
				   ((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[2]);
	const FLT x2 =
		((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[1]) + ((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[2]) +
		((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[3]) + ((*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[0]);
	const FLT x3 = (-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[1]) +
				   ((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[3]) +
				   (-1 * (*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[2]) +
				   ((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[0]);
	const FLT x4 = x0 * x0;
	out->Lighthouse.Pos[0] = (*_x1).Lighthouse.Pos[0] + (-1 * (*_x0).Lighthouse.Pos[0]);
	out->Lighthouse.Pos[1] = (*_x1).Lighthouse.Pos[1] + (-1 * (*_x0).Lighthouse.Pos[1]);
	out->Lighthouse.Pos[2] = (*_x1).Lighthouse.Pos[2] + (-1 * (*_x0).Lighthouse.Pos[2]);
	out->Lighthouse.AxisAngleRot[0] = atan2(2 * ((x2 * x3) + (x0 * x1)), 1 + (-2 * ((x3 * x3) + x4)));
	out->Lighthouse.AxisAngleRot[1] = asin(2 * ((x0 * x2) + (-1 * x1 * x3)));
	out->Lighthouse.AxisAngleRot[2] = atan2(2 * ((x2 * x1) + (x0 * x3)), 1 + (-2 * (x4 + (x1 * x1))));
	out->BSD0.phase = (*_x1).BSD0.phase + (-1 * (*_x0).BSD0.phase);
	out->BSD0.tilt = (*_x1).BSD0.tilt + (-1 * (*_x0).BSD0.tilt);
	out->BSD0.curve = (*_x1).BSD0.curve + (-1 * (*_x0).BSD0.curve);
	out->BSD0.gibpha = (*_x1).BSD0.gibpha + (-1 * (*_x0).BSD0.gibpha);
	out->BSD0.gibmag = (*_x1).BSD0.gibmag + (-1 * (*_x0).BSD0.gibmag);
	out->BSD0.ogeephase = (*_x1).BSD0.ogeephase + (-1 * (*_x0).BSD0.ogeephase);
	out->BSD0.ogeemag = (*_x1).BSD0.ogeemag + (-1 * (*_x0).BSD0.ogeemag);
	out->BSD1.phase = (*_x1).BSD1.phase + (-1 * (*_x0).BSD1.phase);
	out->BSD1.tilt = (*_x1).BSD1.tilt + (-1 * (*_x0).BSD1.tilt);
	out->BSD1.curve = (*_x1).BSD1.curve + (-1 * (*_x0).BSD1.curve);
	out->BSD1.gibpha = (*_x1).BSD1.gibpha + (-1 * (*_x0).BSD1.gibpha);
	out->BSD1.gibmag = (*_x1).BSD1.gibmag + (-1 * (*_x0).BSD1.gibmag);
	out->BSD1.ogeephase = (*_x1).BSD1.ogeephase + (-1 * (*_x0).BSD1.ogeephase);
	out->BSD1.ogeemag = (*_x1).BSD1.ogeemag + (-1 * (*_x0).BSD1.ogeemag);
}

// Jacobian of SurviveLighthouseKalmanModelToErrorModel wrt [(*_x1).Lighthouse.Pos[0], (*_x1).Lighthouse.Pos[1],
// (*_x1).Lighthouse.Pos[2], (*_x1).Lighthouse.Rot[0], (*_x1).Lighthouse.Rot[1], (*_x1).Lighthouse.Rot[2],
// (*_x1).Lighthouse.Rot[3], <cnkalman.codegen.WrapMember object at 0x7f4d1c391760>, <cnkalman.codegen.WrapMember object
// at 0x7f4d1c391a30>, <cnkalman.codegen.WrapMember object at 0x7f4d1c391820>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c391af0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3917c0>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c391a90>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3918e0>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c391bb0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c391880>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c391b50>, <cnkalman.codegen.WrapMember object at 0x7f4d1c391550>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c391970>, <cnkalman.codegen.WrapMember object at 0x7f4d1c391700>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c3919d0>]
static inline void SurviveLighthouseKalmanModelToErrorModel_jac_x1(CnMat *Hx, const SurviveLighthouseKalmanModel *_x1,
																   const SurviveLighthouseKalmanModel *_x0) {
	const FLT x0 = ((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[1]) +
				   (-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[2]) +
				   ((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[0]) +
				   (-1 * (*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[3]);
	const FLT x1 = x0 * (*_x0).Lighthouse.Rot[3];
	const FLT x2 = (-1 * (*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[1]) +
				   ((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[0]) +
				   (-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[3]) +
				   ((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[2]);
	const FLT x3 = x2 * (*_x0).Lighthouse.Rot[2];
	const FLT x4 = (-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[1]) +
				   ((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[3]) +
				   (-1 * (*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[2]) +
				   ((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[0]);
	const FLT x5 = x4 * (*_x0).Lighthouse.Rot[0];
	const FLT x6 =
		((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[1]) + ((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[2]) +
		((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[3]) + ((*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[0]);
	const FLT x7 = x6 * (*_x0).Lighthouse.Rot[1];
	const FLT x8 = (-1 * x7) + x5;
	const FLT x9 = x0 * x0;
	const FLT x10 = 1 + (-2 * ((x4 * x4) + x9));
	const FLT x11 = 2 * (1. / x10);
	const FLT x12 = x0 * (*_x0).Lighthouse.Rot[2];
	const FLT x13 = 4 * x12;
	const FLT x14 = x4 * (*_x0).Lighthouse.Rot[1];
	const FLT x15 = x10 * x10;
	const FLT x16 = (x4 * x6) + (x0 * x2);
	const FLT x17 = 2 * (1. / x15) * x16;
	const FLT x18 = x15 * (1. / (x15 + (4 * (x16 * x16))));
	const FLT x19 = x2 * (*_x0).Lighthouse.Rot[3];
	const FLT x20 = x14 + (x6 * (*_x0).Lighthouse.Rot[0]);
	const FLT x21 = x20 + x12 + (-1 * x19);
	const FLT x22 = 4 * x1;
	const FLT x23 = x4 * (*_x0).Lighthouse.Rot[2];
	const FLT x24 = x6 * (*_x0).Lighthouse.Rot[3];
	const FLT x25 = x0 * (*_x0).Lighthouse.Rot[1];
	const FLT x26 = x2 * (*_x0).Lighthouse.Rot[0];
	const FLT x27 = x26 + (-1 * x25);
	const FLT x28 = x0 * (*_x0).Lighthouse.Rot[0];
	const FLT x29 = -4 * x28;
	const FLT x30 = x4 * (*_x0).Lighthouse.Rot[3];
	const FLT x31 = x6 * (*_x0).Lighthouse.Rot[2];
	const FLT x32 = x2 * (*_x0).Lighthouse.Rot[1];
	const FLT x33 = x28 + x32;
	const FLT x34 = x33 + x30 + (-1 * x31);
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
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.Pos[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.Pos[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.Pos[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						x18 * ((-1 * ((4 * x14) + x13) * x17) + ((x8 + (-1 * x1) + (-1 * x3)) * x11)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						x18 * ((-1 * x17 * ((-4 * x5) + x22)) + (x21 * x11)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						x18 * ((-1 * ((-4 * x30) + x29) * x17) + (x11 * (x23 + x27 + x24))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						x18 * ((-1 * ((4 * x23) + x35) * x17) + (x34 * x11)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT), x34 * x36);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						x36 * (x37 + x25 + (-1 * x26)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT), x36 * x21);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						x36 * (x38 + x7 + (-1 * x5)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						((-1 * (x13 + (4 * x19)) * x43) + ((x37 + x27) * x40)) * x44);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						x44 * ((-1 * x43 * (x22 + (-4 * x3))) + (x40 * (x31 + x33 + (-1 * x30)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						x44 * ((-1 * (x29 + (4 * x32)) * x43) + (x40 * (x38 + x8))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						x44 * ((-1 * (x35 + (-4 * x26)) * x43) + (x40 * (x20 + (-1 * x12) + x19))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD0.phase) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.phase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD0.tilt) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.tilt) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD0.curve) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.curve) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD0.gibpha) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.gibpha) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD0.gibmag) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.gibmag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD0.ogeephase) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.ogeephase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD0.ogeemag) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.ogeemag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD1.phase) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.phase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD1.tilt) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.tilt) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD1.curve) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.curve) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD1.gibpha) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.gibpha) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD1.gibmag) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.gibmag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD1.ogeephase) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.ogeephase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD1.ogeemag) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.ogeemag) / sizeof(FLT), 1);
}

// Full version Jacobian of SurviveLighthouseKalmanModelToErrorModel wrt [(*_x1).Lighthouse.Pos[0],
// (*_x1).Lighthouse.Pos[1], (*_x1).Lighthouse.Pos[2], (*_x1).Lighthouse.Rot[0], (*_x1).Lighthouse.Rot[1],
// (*_x1).Lighthouse.Rot[2], (*_x1).Lighthouse.Rot[3], <cnkalman.codegen.WrapMember object at 0x7f4d1c391760>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c391a30>, <cnkalman.codegen.WrapMember object at 0x7f4d1c391820>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c391af0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3917c0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c391a90>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3918e0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c391bb0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c391880>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c391b50>, <cnkalman.codegen.WrapMember object at 0x7f4d1c391550>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c391970>, <cnkalman.codegen.WrapMember object at 0x7f4d1c391700>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3919d0>] Jacobian of SurviveLighthouseKalmanModelToErrorModel wrt
// [(*_x0).Lighthouse.Pos[0], (*_x0).Lighthouse.Pos[1], (*_x0).Lighthouse.Pos[2], (*_x0).Lighthouse.Rot[0],
// (*_x0).Lighthouse.Rot[1], (*_x0).Lighthouse.Rot[2], (*_x0).Lighthouse.Rot[3], <cnkalman.codegen.WrapMember object at
// 0x7f4d1c38eee0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38ea90>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c38ebb0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38e9d0>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c38ef40>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38ea30>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c38ecd0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38e910>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c38ec40>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38e970>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c38ee20>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38ed60>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c38ee80>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38eaf0>]
static inline void SurviveLighthouseKalmanModelToErrorModel_jac_x0(CnMat *Hx, const SurviveLighthouseKalmanModel *_x1,
																   const SurviveLighthouseKalmanModel *_x0) {
	const FLT x0 = ((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[1]) +
				   (-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[2]) +
				   ((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[0]) +
				   (-1 * (*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[3]);
	const FLT x1 = x0 * (*_x1).Lighthouse.Rot[3];
	const FLT x2 = (-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[1]) +
				   ((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[3]) +
				   (-1 * (*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[2]) +
				   ((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[0]);
	const FLT x3 = x2 * (*_x1).Lighthouse.Rot[0];
	const FLT x4 = x3 + x1;
	const FLT x5 =
		((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[1]) + ((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[2]) +
		((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[3]) + ((*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[0]);
	const FLT x6 = x5 * (*_x1).Lighthouse.Rot[1];
	const FLT x7 = (-1 * (*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[1]) +
				   ((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[0]) +
				   (-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[3]) +
				   ((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[2]);
	const FLT x8 = x7 * (*_x1).Lighthouse.Rot[2];
	const FLT x9 = x8 + x6;
	const FLT x10 = x0 * x0;
	const FLT x11 = 1 + (-2 * ((x2 * x2) + x10));
	const FLT x12 = 2 * (1. / x11);
	const FLT x13 = x0 * (*_x1).Lighthouse.Rot[2];
	const FLT x14 = -4 * x13;
	const FLT x15 = x2 * (*_x1).Lighthouse.Rot[1];
	const FLT x16 = x11 * x11;
	const FLT x17 = (x2 * x5) + (x0 * x7);
	const FLT x18 = 2 * x17 * (1. / x16);
	const FLT x19 = x16 * (1. / (x16 + (4 * (x17 * x17))));
	const FLT x20 = x7 * (*_x1).Lighthouse.Rot[3];
	const FLT x21 = x20 + (-1 * x5 * (*_x1).Lighthouse.Rot[0]);
	const FLT x22 = -4 * x1;
	const FLT x23 = x5 * (*_x1).Lighthouse.Rot[3];
	const FLT x24 = x7 * (*_x1).Lighthouse.Rot[0];
	const FLT x25 = x2 * (*_x1).Lighthouse.Rot[2];
	const FLT x26 = x0 * (*_x1).Lighthouse.Rot[1];
	const FLT x27 = x26 + x25;
	const FLT x28 = x0 * (*_x1).Lighthouse.Rot[0];
	const FLT x29 = 4 * x28;
	const FLT x30 = x2 * (*_x1).Lighthouse.Rot[3];
	const FLT x31 = x30 + (-1 * x28);
	const FLT x32 = x5 * (*_x1).Lighthouse.Rot[2];
	const FLT x33 = x7 * (*_x1).Lighthouse.Rot[1];
	const FLT x34 = (-1 * x33) + x32;
	const FLT x35 = 4 * x26;
	const FLT x36 = 2 * (1. / sqrt(1 + (-4 * (((x0 * x5) + (-1 * x2 * x7)) * ((x0 * x5) + (-1 * x2 * x7))))));
	const FLT x37 = x23 + x27 + x24;
	const FLT x38 = x21 + x13 + (-1 * x15);
	const FLT x39 = 1 + (-2 * (x10 + (x7 * x7)));
	const FLT x40 = 2 * (1. / x39);
	const FLT x41 = x39 * x39;
	const FLT x42 = (x5 * x7) + (x0 * x2);
	const FLT x43 = 2 * (1. / x41) * x42;
	const FLT x44 = x41 * (1. / (x41 + (4 * (x42 * x42))));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.Pos[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Pos[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.Pos[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Pos[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.Pos[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Pos[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						((-1 * ((-4 * x15) + x14) * x18) + ((x9 + x4) * x12)) * x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						x19 * ((-1 * x18 * ((4 * x3) + x22)) + (x12 * (x21 + (-1 * x13) + x15))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						x19 * ((-1 * ((4 * x30) + x29) * x18) + (x12 * ((-1 * x23) + x27 + (-1 * x24)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						((-1 * ((-4 * x25) + x35) * x18) + ((x34 + x31) * x12)) * x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						x36 * (x34 + (-1 * x30) + x28));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT), x36 * x37);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT), x36 * x38);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						(x4 + (-1 * x6) + (-1 * x8)) * x36);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						x44 * ((-1 * (x14 + (-4 * x20)) * x43) + (x40 * x37)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						x44 * ((-1 * x43 * (x22 + (4 * x8))) + (x40 * (x31 + x33 + (-1 * x32)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						x44 * ((-1 * (x29 + (-4 * x33)) * x43) + ((x9 + (-1 * x1) + (-1 * x3)) * x40)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						x44 * ((-1 * (x35 + (4 * x24)) * x43) + (x40 * x38)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD0.phase) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.phase) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD0.tilt) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.tilt) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD0.curve) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.curve) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD0.gibpha) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.gibpha) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD0.gibmag) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.gibmag) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD0.ogeephase) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.ogeephase) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD0.ogeemag) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.ogeemag) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD1.phase) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.phase) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD1.tilt) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.tilt) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD1.curve) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.curve) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD1.gibpha) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.gibpha) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD1.gibmag) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.gibmag) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD1.ogeephase) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.ogeephase) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanErrorModel, BSD1.ogeemag) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.ogeemag) / sizeof(FLT), -1);
}

// Full version Jacobian of SurviveLighthouseKalmanModelToErrorModel wrt [(*_x0).Lighthouse.Pos[0],
// (*_x0).Lighthouse.Pos[1], (*_x0).Lighthouse.Pos[2], (*_x0).Lighthouse.Rot[0], (*_x0).Lighthouse.Rot[1],
// (*_x0).Lighthouse.Rot[2], (*_x0).Lighthouse.Rot[3], <cnkalman.codegen.WrapMember object at 0x7f4d1c38eee0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c38ea90>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38ebb0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c38e9d0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38ef40>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c38ea30>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38ecd0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c38e910>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38ec40>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c38e970>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38ee20>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c38ed60>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38ee80>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c38eaf0>]
static inline void SurviveLighthouseKalmanModelAddErrorModel(SurviveLighthouseKalmanModel *out,
															 const SurviveLighthouseKalmanModel *_x0,
															 const SurviveLighthouseKalmanErrorModel *error_state) {
	const FLT x0 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[2];
	const FLT x1 = cos(x0);
	const FLT x2 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[0];
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[1];
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = cos(x4);
	const FLT x8 = sin(x0);
	const FLT x9 = cos(x2);
	const FLT x10 = x8 * x9;
	const FLT x11 = (x7 * x10) + (-1 * x1 * x6);
	const FLT x12 = x3 * x7;
	const FLT x13 = x1 * x9;
	const FLT x14 = (x5 * x13) + (x8 * x12);
	const FLT x15 = (x7 * x13) + (x6 * x8);
	const FLT x16 = (x1 * x12) + (-1 * x5 * x10);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x15 * x15) + (x11 * x11) + (x14 * x14));
	const FLT x18 = x11 * x17;
	const FLT x19 = x17 * (*_x0).Lighthouse.Rot[2];
	const FLT x20 = x15 * x17;
	const FLT x21 = x17 * x16;
	const FLT x22 = x14 * x17;
	out->Lighthouse.Pos[0] = (*_x0).Lighthouse.Pos[0] + (*error_state).Lighthouse.Pos[0];
	out->Lighthouse.Pos[1] = (*_x0).Lighthouse.Pos[1] + (*error_state).Lighthouse.Pos[1];
	out->Lighthouse.Pos[2] = (*_x0).Lighthouse.Pos[2] + (*error_state).Lighthouse.Pos[2];
	out->Lighthouse.Rot[0] = (-1 * x21 * (*_x0).Lighthouse.Rot[1]) + (x20 * (*_x0).Lighthouse.Rot[0]) +
							 (-1 * x18 * (*_x0).Lighthouse.Rot[3]) + (-1 * x14 * x19);
	out->Lighthouse.Rot[1] = (x21 * (*_x0).Lighthouse.Rot[0]) + (x20 * (*_x0).Lighthouse.Rot[1]) +
							 (-1 * x22 * (*_x0).Lighthouse.Rot[3]) + (x11 * x19);
	out->Lighthouse.Rot[2] = (-1 * x18 * (*_x0).Lighthouse.Rot[1]) + (x22 * (*_x0).Lighthouse.Rot[0]) +
							 (x21 * (*_x0).Lighthouse.Rot[3]) + (x15 * x19);
	out->Lighthouse.Rot[3] = (x22 * (*_x0).Lighthouse.Rot[1]) + (x18 * (*_x0).Lighthouse.Rot[0]) + (-1 * x19 * x16) +
							 (x20 * (*_x0).Lighthouse.Rot[3]);
	out->BSD0.phase = (*error_state).BSD0.phase + (*_x0).BSD0.phase;
	out->BSD0.tilt = (*error_state).BSD0.tilt + (*_x0).BSD0.tilt;
	out->BSD0.curve = (*error_state).BSD0.curve + (*_x0).BSD0.curve;
	out->BSD0.gibpha = (*error_state).BSD0.gibpha + (*_x0).BSD0.gibpha;
	out->BSD0.gibmag = (*error_state).BSD0.gibmag + (*_x0).BSD0.gibmag;
	out->BSD0.ogeephase = (*error_state).BSD0.ogeephase + (*_x0).BSD0.ogeephase;
	out->BSD0.ogeemag = (*error_state).BSD0.ogeemag + (*_x0).BSD0.ogeemag;
	out->BSD1.phase = (*error_state).BSD1.phase + (*_x0).BSD1.phase;
	out->BSD1.tilt = (*error_state).BSD1.tilt + (*_x0).BSD1.tilt;
	out->BSD1.curve = (*error_state).BSD1.curve + (*_x0).BSD1.curve;
	out->BSD1.gibpha = (*error_state).BSD1.gibpha + (*_x0).BSD1.gibpha;
	out->BSD1.gibmag = (*error_state).BSD1.gibmag + (*_x0).BSD1.gibmag;
	out->BSD1.ogeephase = (*error_state).BSD1.ogeephase + (*_x0).BSD1.ogeephase;
	out->BSD1.ogeemag = (*error_state).BSD1.ogeemag + (*_x0).BSD1.ogeemag;
}

// Jacobian of SurviveLighthouseKalmanModelAddErrorModel wrt [(*_x0).Lighthouse.Pos[0], (*_x0).Lighthouse.Pos[1],
// (*_x0).Lighthouse.Pos[2], (*_x0).Lighthouse.Rot[0], (*_x0).Lighthouse.Rot[1], (*_x0).Lighthouse.Rot[2],
// (*_x0).Lighthouse.Rot[3], <cnkalman.codegen.WrapMember object at 0x7f4d1c3864f0>, <cnkalman.codegen.WrapMember object
// at 0x7f4d1c3867c0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3865b0>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c386880>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386550>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c386820>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386670>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c386940>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386610>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c3868e0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3862e0>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c386700>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386490>, <cnkalman.codegen.WrapMember object at
// 0x7f4d1c386760>]
static inline void
SurviveLighthouseKalmanModelAddErrorModel_jac_x0(CnMat *Hx, const SurviveLighthouseKalmanModel *_x0,
												 const SurviveLighthouseKalmanErrorModel *error_state) {
	const FLT x0 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[1];
	const FLT x1 = sin(x0);
	const FLT x2 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[0];
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[2];
	const FLT x5 = cos(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = cos(x0);
	const FLT x8 = sin(x4);
	const FLT x9 = cos(x2);
	const FLT x10 = x8 * x9;
	const FLT x11 = (x7 * x10) + (-1 * x1 * x6);
	const FLT x12 = x3 * x8;
	const FLT x13 = x5 * x9;
	const FLT x14 = (x1 * x13) + (x7 * x12);
	const FLT x15 = (x7 * x13) + (x1 * x12);
	const FLT x16 = (x6 * x7) + (-1 * x1 * x10);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x15 * x15) + (x11 * x11) + (x14 * x14));
	const FLT x18 = x15 * x17;
	const FLT x19 = x17 * x16;
	const FLT x20 = -1 * x19;
	const FLT x21 = x14 * x17;
	const FLT x22 = -1 * x21;
	const FLT x23 = x11 * x17;
	const FLT x24 = -1 * x23;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Pos[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Pos[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Pos[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT), x20);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT), x22);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT), x24);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT), x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT), x23);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT), x22);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT), x21);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT), x24);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT), x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT), x23);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT), x21);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT), x20);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD0.phase) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.phase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD0.tilt) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.tilt) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD0.curve) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.curve) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD0.gibpha) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.gibpha) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD0.gibmag) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.gibmag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD0.ogeephase) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.ogeephase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD0.ogeemag) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD0.ogeemag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD1.phase) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.phase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD1.tilt) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.tilt) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD1.curve) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.curve) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD1.gibpha) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.gibpha) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD1.gibmag) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.gibmag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD1.ogeephase) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.ogeephase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD1.ogeemag) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanModel, BSD1.ogeemag) / sizeof(FLT), 1);
}

// Full version Jacobian of SurviveLighthouseKalmanModelAddErrorModel wrt [(*_x0).Lighthouse.Pos[0],
// (*_x0).Lighthouse.Pos[1], (*_x0).Lighthouse.Pos[2], (*_x0).Lighthouse.Rot[0], (*_x0).Lighthouse.Rot[1],
// (*_x0).Lighthouse.Rot[2], (*_x0).Lighthouse.Rot[3], <cnkalman.codegen.WrapMember object at 0x7f4d1c3864f0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3867c0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3865b0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c386880>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386550>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c386820>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386670>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c386940>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386610>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3868e0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3862e0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c386700>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386490>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c386760>] Jacobian of SurviveLighthouseKalmanModelAddErrorModel wrt
// [(*error_state).Lighthouse.AxisAngleRot[0], (*error_state).Lighthouse.AxisAngleRot[1],
// (*error_state).Lighthouse.AxisAngleRot[2], (*error_state).Lighthouse.Pos[0], (*error_state).Lighthouse.Pos[1],
// (*error_state).Lighthouse.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f4d1c386df0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c392100>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386eb0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3921c0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386e50>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c392160>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386f70>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c392280>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386f10>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c392220>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386c40>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c392040>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386d90>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3920a0>]
static inline void
SurviveLighthouseKalmanModelAddErrorModel_jac_error_state(CnMat *Hx, const SurviveLighthouseKalmanModel *_x0,
														  const SurviveLighthouseKalmanErrorModel *error_state) {
	const FLT x0 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[2];
	const FLT x1 = cos(x0);
	const FLT x2 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[0];
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[1];
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = x1 * x6;
	const FLT x8 = cos(x4);
	const FLT x9 = sin(x0);
	const FLT x10 = cos(x2);
	const FLT x11 = x9 * x10;
	const FLT x12 = x8 * x11;
	const FLT x13 = x12 + (-1 * x7);
	const FLT x14 = x3 * x8;
	const FLT x15 = x9 * x14;
	const FLT x16 = x1 * x10;
	const FLT x17 = x5 * x16;
	const FLT x18 = x17 + x15;
	const FLT x19 = x6 * x9;
	const FLT x20 = x8 * x16;
	const FLT x21 = x20 + x19;
	const FLT x22 = x5 * x11;
	const FLT x23 = x1 * x14;
	const FLT x24 = x23 + (-1 * x22);
	const FLT x25 = (x24 * x24) + (x21 * x21) + (x13 * x13) + (x18 * x18);
	const FLT x26 = 1.0 / 2.0 * (1. / (x25 * sqrt(x25)));
	const FLT x27 = x26 * (*_x0).Lighthouse.Rot[1];
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
	const FLT x40 = 2 * x39;
	const FLT x41 = 0.5 * x7;
	const FLT x42 = -1 * x41;
	const FLT x43 = 0.5 * x12;
	const FLT x44 = x43 + x42;
	const FLT x45 = 2 * x18;
	const FLT x46 = (x44 * x45) + (x40 * x13) + (x30 * x31) + (x36 * x35);
	const FLT x47 = x46 * x24;
	const FLT x48 = 1. / sqrt(x25);
	const FLT x49 = x48 * (*_x0).Lighthouse.Rot[1];
	const FLT x50 = -1 * x49 * x30;
	const FLT x51 = x26 * (*_x0).Lighthouse.Rot[0];
	const FLT x52 = x51 * x46;
	const FLT x53 = x48 * (*_x0).Lighthouse.Rot[3];
	const FLT x54 = x53 * x39;
	const FLT x55 = x26 * (*_x0).Lighthouse.Rot[3];
	const FLT x56 = x46 * x13;
	const FLT x57 = x48 * (*_x0).Lighthouse.Rot[0];
	const FLT x58 = x44 * x48;
	const FLT x59 = x26 * (*_x0).Lighthouse.Rot[2];
	const FLT x60 = x46 * x18;
	const FLT x61 = -1 * x43;
	const FLT x62 = x61 + x42;
	const FLT x63 = x37 + x38;
	const FLT x64 = -1 * x34;
	const FLT x65 = x33 + x64;
	const FLT x66 = 2 * x13;
	const FLT x67 = (-1 * x29) + x28;
	const FLT x68 = (x67 * x45) + (x63 * x36) + (x62 * x31) + (x65 * x66);
	const FLT x69 = x68 * x21;
	const FLT x70 = x68 * x13;
	const FLT x71 = x68 * x18;
	const FLT x72 = x71 * x26;
	const FLT x73 = x68 * x24;
	const FLT x74 = x73 * x26;
	const FLT x75 = x63 * x48;
	const FLT x76 = x67 * x48;
	const FLT x77 = -1 * x49 * x39;
	const FLT x78 = x41 + x61;
	const FLT x79 = x32 + x64;
	const FLT x80 = (x40 * x24) + (x79 * x45) + (x78 * x36) + (x66 * x30);
	const FLT x81 = x80 * x21;
	const FLT x82 = x80 * x13;
	const FLT x83 = x53 * x30;
	const FLT x84 = x48 * (*_x0).Lighthouse.Rot[2];
	const FLT x85 = x80 * x18;
	const FLT x86 = x85 * x26;
	const FLT x87 = x80 * x27;
	const FLT x88 = x46 * x21;
	const FLT x89 = x84 * x39;
	const FLT x90 = x57 * x30;
	const FLT x91 = x69 * x26;
	const FLT x92 = x80 * x24;
	const FLT x93 = x81 * x26;
	const FLT x94 = x84 * x30;
	const FLT x95 = x57 * x39;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Pos[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Pos[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Pos[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						(-1 * x58 * (*_x0).Lighthouse.Rot[2]) + (x57 * x35) + (x47 * x27) + (x60 * x59) + x50 +
							(-1 * x52 * x21) + (x56 * x55) + (-1 * x54));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						(-1 * x76 * (*_x0).Lighthouse.Rot[2]) + (-1 * x69 * x51) + (-1 * x62 * x49) +
							(x75 * (*_x0).Lighthouse.Rot[0]) + (-1 * x65 * x53) + (x74 * (*_x0).Lighthouse.Rot[1]) +
							(x70 * x55) + (x72 * (*_x0).Lighthouse.Rot[2]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						(x87 * x24) + (x86 * (*_x0).Lighthouse.Rot[2]) + (-1 * x81 * x51) + x77 + (x82 * x55) +
							(x78 * x57) + (-1 * x83) + (-1 * x84 * x79));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						(-1 * x56 * x59) + x90 + (x49 * x35) + (x60 * x55) + x89 + (-1 * x88 * x27) + (-1 * x52 * x24) +
							(-1 * x58 * (*_x0).Lighthouse.Rot[3]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						(x84 * x65) + (x75 * (*_x0).Lighthouse.Rot[1]) + (-1 * x91 * (*_x0).Lighthouse.Rot[1]) +
							(-1 * x70 * x59) + (-1 * x73 * x51) + (x62 * x57) + (x71 * x55) +
							(-1 * x76 * (*_x0).Lighthouse.Rot[3]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						x95 + x94 + (x86 * (*_x0).Lighthouse.Rot[3]) + (-1 * x51 * x92) + (-1 * x82 * x59) +
							(-1 * x93 * (*_x0).Lighthouse.Rot[1]) + (-1 * x79 * x53) + (x78 * x49));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						(x56 * x27) + (x84 * x35) + (-1 * x88 * x59) + (-1 * x52 * x18) + x77 +
							(x58 * (*_x0).Lighthouse.Rot[0]) + (-1 * x55 * x47) + x83);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						(x76 * (*_x0).Lighthouse.Rot[0]) + (x70 * x27) + (x75 * (*_x0).Lighthouse.Rot[2]) +
							(-1 * x73 * x55) + (-1 * x65 * x49) + (-1 * x91 * (*_x0).Lighthouse.Rot[2]) + (x62 * x53) +
							(-1 * x71 * x51));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						(-1 * x93 * (*_x0).Lighthouse.Rot[2]) + (x79 * x57) + (x87 * x13) + x54 + x50 + (x84 * x78) +
							(-1 * x85 * x51) + (-1 * x55 * x92));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						(x59 * x47) + (-1 * x88 * x55) + (-1 * x52 * x13) + x95 + (-1 * x60 * x27) + (x53 * x35) +
							(-1 * x94) + (x58 * (*_x0).Lighthouse.Rot[1]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						(x65 * x57) + (-1 * x91 * (*_x0).Lighthouse.Rot[3]) + (-1 * x84 * x62) + (-1 * x70 * x51) +
							(x74 * (*_x0).Lighthouse.Rot[2]) + (-1 * x72 * (*_x0).Lighthouse.Rot[1]) +
							(x76 * (*_x0).Lighthouse.Rot[1]) + (x75 * (*_x0).Lighthouse.Rot[3]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						(x79 * x49) + (x78 * x53) + x90 + (-1 * x82 * x51) + (-1 * x93 * (*_x0).Lighthouse.Rot[3]) +
							(-1 * x89) + (-1 * x86 * (*_x0).Lighthouse.Rot[1]) + (x59 * x92));
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD0.phase) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, BSD0.phase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD0.tilt) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, BSD0.tilt) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD0.curve) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, BSD0.curve) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD0.gibpha) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, BSD0.gibpha) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD0.gibmag) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, BSD0.gibmag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD0.ogeephase) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, BSD0.ogeephase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD0.ogeemag) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, BSD0.ogeemag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD1.phase) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, BSD1.phase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD1.tilt) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, BSD1.tilt) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD1.curve) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, BSD1.curve) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD1.gibpha) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, BSD1.gibpha) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD1.gibmag) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, BSD1.gibmag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD1.ogeephase) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, BSD1.ogeephase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveLighthouseKalmanModel, BSD1.ogeemag) / sizeof(FLT),
						offsetof(SurviveLighthouseKalmanErrorModel, BSD1.ogeemag) / sizeof(FLT), 1);
}

// Full version Jacobian of SurviveLighthouseKalmanModelAddErrorModel wrt [(*error_state).Lighthouse.AxisAngleRot[0],
// (*error_state).Lighthouse.AxisAngleRot[1], (*error_state).Lighthouse.AxisAngleRot[2],
// (*error_state).Lighthouse.Pos[0], (*error_state).Lighthouse.Pos[1], (*error_state).Lighthouse.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f4d1c386df0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c392100>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c386eb0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3921c0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c386e50>, <cnkalman.codegen.WrapMember object at 0x7f4d1c392160>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c386f70>, <cnkalman.codegen.WrapMember object at 0x7f4d1c392280>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c386f10>, <cnkalman.codegen.WrapMember object at 0x7f4d1c392220>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c386c40>, <cnkalman.codegen.WrapMember object at 0x7f4d1c392040>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c386d90>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3920a0>]
static inline void SurviveJointKalmanModelToErrorModel(SurviveJointKalmanErrorModel *out,
													   const SurviveJointKalmanModel *_x1,
													   const SurviveJointKalmanModel *_x0) {
	const FLT x0 = ((*_x1).Object.Pose.Rot[3] * (*_x0).Object.Pose.Rot[1]) +
				   (-1 * (*_x1).Object.Pose.Rot[0] * (*_x0).Object.Pose.Rot[2]) +
				   ((*_x1).Object.Pose.Rot[2] * (*_x0).Object.Pose.Rot[0]) +
				   (-1 * (*_x1).Object.Pose.Rot[1] * (*_x0).Object.Pose.Rot[3]);
	const FLT x1 = (-1 * (*_x1).Object.Pose.Rot[2] * (*_x0).Object.Pose.Rot[1]) +
				   ((*_x1).Object.Pose.Rot[1] * (*_x0).Object.Pose.Rot[2]) +
				   ((*_x1).Object.Pose.Rot[3] * (*_x0).Object.Pose.Rot[0]) +
				   (-1 * (*_x1).Object.Pose.Rot[0] * (*_x0).Object.Pose.Rot[3]);
	const FLT x2 = (-1 * (*_x1).Object.Pose.Rot[0] * (*_x0).Object.Pose.Rot[1]) +
				   ((*_x1).Object.Pose.Rot[1] * (*_x0).Object.Pose.Rot[0]) +
				   ((*_x1).Object.Pose.Rot[2] * (*_x0).Object.Pose.Rot[3]) +
				   (-1 * (*_x1).Object.Pose.Rot[3] * (*_x0).Object.Pose.Rot[2]);
	const FLT x3 = ((*_x1).Object.Pose.Rot[1] * (*_x0).Object.Pose.Rot[1]) +
				   ((*_x1).Object.Pose.Rot[3] * (*_x0).Object.Pose.Rot[3]) +
				   ((*_x1).Object.Pose.Rot[0] * (*_x0).Object.Pose.Rot[0]) +
				   ((*_x1).Object.Pose.Rot[2] * (*_x0).Object.Pose.Rot[2]);
	const FLT x4 = x0 * x0;
	const FLT x5 = ((*_x1).Object.IMUBias.IMUCorrection[3] * (*_x0).Object.IMUBias.IMUCorrection[1]) +
				   ((*_x1).Object.IMUBias.IMUCorrection[2] * (*_x0).Object.IMUBias.IMUCorrection[0]) +
				   (-1 * (*_x1).Object.IMUBias.IMUCorrection[1] * (*_x0).Object.IMUBias.IMUCorrection[3]) +
				   (-1 * (*_x1).Object.IMUBias.IMUCorrection[0] * (*_x0).Object.IMUBias.IMUCorrection[2]);
	const FLT x6 = (-1 * (*_x1).Object.IMUBias.IMUCorrection[2] * (*_x0).Object.IMUBias.IMUCorrection[1]) +
				   (-1 * (*_x1).Object.IMUBias.IMUCorrection[0] * (*_x0).Object.IMUBias.IMUCorrection[3]) +
				   ((*_x1).Object.IMUBias.IMUCorrection[1] * (*_x0).Object.IMUBias.IMUCorrection[2]) +
				   ((*_x1).Object.IMUBias.IMUCorrection[3] * (*_x0).Object.IMUBias.IMUCorrection[0]);
	const FLT x7 = ((*_x1).Object.IMUBias.IMUCorrection[2] * (*_x0).Object.IMUBias.IMUCorrection[3]) +
				   (-1 * (*_x1).Object.IMUBias.IMUCorrection[0] * (*_x0).Object.IMUBias.IMUCorrection[1]) +
				   ((*_x1).Object.IMUBias.IMUCorrection[1] * (*_x0).Object.IMUBias.IMUCorrection[0]) +
				   (-1 * (*_x1).Object.IMUBias.IMUCorrection[3] * (*_x0).Object.IMUBias.IMUCorrection[2]);
	const FLT x8 = ((*_x1).Object.IMUBias.IMUCorrection[1] * (*_x0).Object.IMUBias.IMUCorrection[1]) +
				   ((*_x1).Object.IMUBias.IMUCorrection[2] * (*_x0).Object.IMUBias.IMUCorrection[2]) +
				   ((*_x1).Object.IMUBias.IMUCorrection[0] * (*_x0).Object.IMUBias.IMUCorrection[0]) +
				   ((*_x1).Object.IMUBias.IMUCorrection[3] * (*_x0).Object.IMUBias.IMUCorrection[3]);
	const FLT x9 = x5 * x5;
	const FLT x10 = ((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[1]) +
					(-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[2]) +
					((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[0]) +
					(-1 * (*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[3]);
	const FLT x11 = (-1 * (*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[1]) +
					((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[0]) +
					(-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[3]) +
					((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[2]);
	const FLT x12 =
		((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[1]) + ((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[2]) +
		((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[3]) + ((*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[0]);
	const FLT x13 = (-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[1]) +
					((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[3]) +
					(-1 * (*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[2]) +
					((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[0]);
	const FLT x14 = x10 * x10;
	out->Object.Pose.Pos[0] = (*_x1).Object.Pose.Pos[0] + (-1 * (*_x0).Object.Pose.Pos[0]);
	out->Object.Pose.Pos[1] = (*_x1).Object.Pose.Pos[1] + (-1 * (*_x0).Object.Pose.Pos[1]);
	out->Object.Pose.Pos[2] = (*_x1).Object.Pose.Pos[2] + (-1 * (*_x0).Object.Pose.Pos[2]);
	out->Object.Pose.AxisAngleRot[0] = atan2(2 * ((x2 * x3) + (x0 * x1)), 1 + (-2 * ((x2 * x2) + x4)));
	out->Object.Pose.AxisAngleRot[1] = asin(2 * ((x0 * x3) + (-1 * x2 * x1)));
	out->Object.Pose.AxisAngleRot[2] = atan2(2 * ((x1 * x3) + (x0 * x2)), 1 + (-2 * (x4 + (x1 * x1))));
	out->Object.Velocity.Pos[0] = (*_x1).Object.Velocity.Pos[0] + (-1 * (*_x0).Object.Velocity.Pos[0]);
	out->Object.Velocity.Pos[1] = (*_x1).Object.Velocity.Pos[1] + (-1 * (*_x0).Object.Velocity.Pos[1]);
	out->Object.Velocity.Pos[2] = (*_x1).Object.Velocity.Pos[2] + (-1 * (*_x0).Object.Velocity.Pos[2]);
	out->Object.Velocity.AxisAngleRot[0] =
		(*_x1).Object.Velocity.AxisAngleRot[0] + (-1 * (*_x0).Object.Velocity.AxisAngleRot[0]);
	out->Object.Velocity.AxisAngleRot[1] =
		(*_x1).Object.Velocity.AxisAngleRot[1] + (-1 * (*_x0).Object.Velocity.AxisAngleRot[1]);
	out->Object.Velocity.AxisAngleRot[2] =
		(*_x1).Object.Velocity.AxisAngleRot[2] + (-1 * (*_x0).Object.Velocity.AxisAngleRot[2]);
	out->Object.Acc[0] = (*_x1).Object.Acc[0] + (-1 * (*_x0).Object.Acc[0]);
	out->Object.Acc[1] = (*_x1).Object.Acc[1] + (-1 * (*_x0).Object.Acc[1]);
	out->Object.Acc[2] = (*_x1).Object.Acc[2] + (-1 * (*_x0).Object.Acc[2]);
	out->Object.IMUBias.AccScale[0] = (*_x1).Object.IMUBias.AccScale[0] + (-1 * (*_x0).Object.IMUBias.AccScale[0]);
	out->Object.IMUBias.AccScale[1] = (*_x1).Object.IMUBias.AccScale[1] + (-1 * (*_x0).Object.IMUBias.AccScale[1]);
	out->Object.IMUBias.AccScale[2] = (*_x1).Object.IMUBias.AccScale[2] + (-1 * (*_x0).Object.IMUBias.AccScale[2]);
	out->Object.IMUBias.IMUCorrection[0] = atan2(2 * ((x8 * x7) + (x6 * x5)), 1 + (-2 * ((x7 * x7) + x9)));
	out->Object.IMUBias.IMUCorrection[1] = asin(2 * ((x5 * x8) + (-1 * x6 * x7)));
	out->Object.IMUBias.IMUCorrection[2] = atan2(2 * ((x6 * x8) + (x5 * x7)), 1 + (-2 * (x9 + (x6 * x6))));
	out->Object.IMUBias.AccBias[0] = (*_x1).Object.IMUBias.AccBias[0] + (-1 * (*_x0).Object.IMUBias.AccBias[0]);
	out->Object.IMUBias.AccBias[1] = (*_x1).Object.IMUBias.AccBias[1] + (-1 * (*_x0).Object.IMUBias.AccBias[1]);
	out->Object.IMUBias.AccBias[2] = (*_x1).Object.IMUBias.AccBias[2] + (-1 * (*_x0).Object.IMUBias.AccBias[2]);
	out->Object.IMUBias.GyroBias[0] = (*_x1).Object.IMUBias.GyroBias[0] + (-1 * (*_x0).Object.IMUBias.GyroBias[0]);
	out->Object.IMUBias.GyroBias[1] = (*_x1).Object.IMUBias.GyroBias[1] + (-1 * (*_x0).Object.IMUBias.GyroBias[1]);
	out->Object.IMUBias.GyroBias[2] = (*_x1).Object.IMUBias.GyroBias[2] + (-1 * (*_x0).Object.IMUBias.GyroBias[2]);
	out->Lighthouse.Pos[0] = (*_x1).Lighthouse.Pos[0] + (-1 * (*_x0).Lighthouse.Pos[0]);
	out->Lighthouse.Pos[1] = (*_x1).Lighthouse.Pos[1] + (-1 * (*_x0).Lighthouse.Pos[1]);
	out->Lighthouse.Pos[2] = (*_x1).Lighthouse.Pos[2] + (-1 * (*_x0).Lighthouse.Pos[2]);
	out->Lighthouse.AxisAngleRot[0] = atan2(2 * ((x13 * x12) + (x11 * x10)), 1 + (-2 * ((x13 * x13) + x14)));
	out->Lighthouse.AxisAngleRot[1] = asin(2 * ((x12 * x10) + (-1 * x13 * x11)));
	out->Lighthouse.AxisAngleRot[2] = atan2(2 * ((x12 * x11) + (x13 * x10)), 1 + (-2 * (x14 + (x11 * x11))));
	out->BSD0.phase = (*_x1).BSD0.phase + (-1 * (*_x0).BSD0.phase);
	out->BSD0.tilt = (*_x1).BSD0.tilt + (-1 * (*_x0).BSD0.tilt);
	out->BSD0.curve = (*_x1).BSD0.curve + (-1 * (*_x0).BSD0.curve);
	out->BSD0.gibpha = (*_x1).BSD0.gibpha + (-1 * (*_x0).BSD0.gibpha);
	out->BSD0.gibmag = (*_x1).BSD0.gibmag + (-1 * (*_x0).BSD0.gibmag);
	out->BSD0.ogeephase = (*_x1).BSD0.ogeephase + (-1 * (*_x0).BSD0.ogeephase);
	out->BSD0.ogeemag = (*_x1).BSD0.ogeemag + (-1 * (*_x0).BSD0.ogeemag);
	out->BSD1.phase = (*_x1).BSD1.phase + (-1 * (*_x0).BSD1.phase);
	out->BSD1.tilt = (*_x1).BSD1.tilt + (-1 * (*_x0).BSD1.tilt);
	out->BSD1.curve = (*_x1).BSD1.curve + (-1 * (*_x0).BSD1.curve);
	out->BSD1.gibpha = (*_x1).BSD1.gibpha + (-1 * (*_x0).BSD1.gibpha);
	out->BSD1.gibmag = (*_x1).BSD1.gibmag + (-1 * (*_x0).BSD1.gibmag);
	out->BSD1.ogeephase = (*_x1).BSD1.ogeephase + (-1 * (*_x0).BSD1.ogeephase);
	out->BSD1.ogeemag = (*_x1).BSD1.ogeemag + (-1 * (*_x0).BSD1.ogeemag);
}

// Jacobian of SurviveJointKalmanModelToErrorModel wrt [(*_x1).Lighthouse.Pos[0], (*_x1).Lighthouse.Pos[1],
// (*_x1).Lighthouse.Pos[2], (*_x1).Lighthouse.Rot[0], (*_x1).Lighthouse.Rot[1], (*_x1).Lighthouse.Rot[2],
// (*_x1).Lighthouse.Rot[3], (*_x1).Object.Acc[0], (*_x1).Object.Acc[1], (*_x1).Object.Acc[2],
// (*_x1).Object.IMUBias.AccBias[0], (*_x1).Object.IMUBias.AccBias[1], (*_x1).Object.IMUBias.AccBias[2],
// (*_x1).Object.IMUBias.AccScale[0], (*_x1).Object.IMUBias.AccScale[1], (*_x1).Object.IMUBias.AccScale[2],
// (*_x1).Object.IMUBias.GyroBias[0], (*_x1).Object.IMUBias.GyroBias[1], (*_x1).Object.IMUBias.GyroBias[2],
// (*_x1).Object.IMUBias.IMUCorrection[0], (*_x1).Object.IMUBias.IMUCorrection[1],
// (*_x1).Object.IMUBias.IMUCorrection[2], (*_x1).Object.IMUBias.IMUCorrection[3], (*_x1).Object.Pose.Pos[0],
// (*_x1).Object.Pose.Pos[1], (*_x1).Object.Pose.Pos[2], (*_x1).Object.Pose.Rot[0], (*_x1).Object.Pose.Rot[1],
// (*_x1).Object.Pose.Rot[2], (*_x1).Object.Pose.Rot[3], (*_x1).Object.Velocity.AxisAngleRot[0],
// (*_x1).Object.Velocity.AxisAngleRot[1], (*_x1).Object.Velocity.AxisAngleRot[2], (*_x1).Object.Velocity.Pos[0],
// (*_x1).Object.Velocity.Pos[1], (*_x1).Object.Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f4d1c386f40>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c396250>, <cnkalman.codegen.WrapMember object at 0x7f4d1c396040>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c396310>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386fa0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3962b0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c396100>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3963d0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3960a0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c396370>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386d30>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c396190>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386ee0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3961f0>]
static inline void SurviveJointKalmanModelToErrorModel_jac_x1(CnMat *Hx, const SurviveJointKalmanModel *_x1,
															  const SurviveJointKalmanModel *_x0) {
	const FLT x0 = ((*_x1).Object.Pose.Rot[3] * (*_x0).Object.Pose.Rot[1]) +
				   (-1 * (*_x1).Object.Pose.Rot[0] * (*_x0).Object.Pose.Rot[2]) +
				   ((*_x1).Object.Pose.Rot[2] * (*_x0).Object.Pose.Rot[0]) +
				   (-1 * (*_x1).Object.Pose.Rot[1] * (*_x0).Object.Pose.Rot[3]);
	const FLT x1 = x0 * (*_x0).Object.Pose.Rot[3];
	const FLT x2 = (-1 * (*_x1).Object.Pose.Rot[2] * (*_x0).Object.Pose.Rot[1]) +
				   ((*_x1).Object.Pose.Rot[1] * (*_x0).Object.Pose.Rot[2]) +
				   ((*_x1).Object.Pose.Rot[3] * (*_x0).Object.Pose.Rot[0]) +
				   (-1 * (*_x1).Object.Pose.Rot[0] * (*_x0).Object.Pose.Rot[3]);
	const FLT x3 = x2 * (*_x0).Object.Pose.Rot[2];
	const FLT x4 = ((*_x1).Object.Pose.Rot[1] * (*_x0).Object.Pose.Rot[1]) +
				   ((*_x1).Object.Pose.Rot[3] * (*_x0).Object.Pose.Rot[3]) +
				   ((*_x1).Object.Pose.Rot[0] * (*_x0).Object.Pose.Rot[0]) +
				   ((*_x1).Object.Pose.Rot[2] * (*_x0).Object.Pose.Rot[2]);
	const FLT x5 = x4 * (*_x0).Object.Pose.Rot[1];
	const FLT x6 = (-1 * (*_x1).Object.Pose.Rot[0] * (*_x0).Object.Pose.Rot[1]) +
				   ((*_x1).Object.Pose.Rot[1] * (*_x0).Object.Pose.Rot[0]) +
				   ((*_x1).Object.Pose.Rot[2] * (*_x0).Object.Pose.Rot[3]) +
				   (-1 * (*_x1).Object.Pose.Rot[3] * (*_x0).Object.Pose.Rot[2]);
	const FLT x7 = x6 * (*_x0).Object.Pose.Rot[0];
	const FLT x8 = x7 + (-1 * x5);
	const FLT x9 = x0 * x0;
	const FLT x10 = 1 + (-2 * ((x6 * x6) + x9));
	const FLT x11 = 2 * (1. / x10);
	const FLT x12 = x0 * (*_x0).Object.Pose.Rot[2];
	const FLT x13 = 4 * x12;
	const FLT x14 = x6 * (*_x0).Object.Pose.Rot[1];
	const FLT x15 = x10 * x10;
	const FLT x16 = (x4 * x6) + (x0 * x2);
	const FLT x17 = 2 * (1. / x15) * x16;
	const FLT x18 = x15 * (1. / (x15 + (4 * (x16 * x16))));
	const FLT x19 = x2 * (*_x0).Object.Pose.Rot[3];
	const FLT x20 = x14 + (x4 * (*_x0).Object.Pose.Rot[0]);
	const FLT x21 = x20 + x12 + (-1 * x19);
	const FLT x22 = 4 * x1;
	const FLT x23 = x4 * (*_x0).Object.Pose.Rot[3];
	const FLT x24 = x6 * (*_x0).Object.Pose.Rot[2];
	const FLT x25 = x0 * (*_x0).Object.Pose.Rot[1];
	const FLT x26 = x2 * (*_x0).Object.Pose.Rot[0];
	const FLT x27 = x26 + (-1 * x25);
	const FLT x28 = x0 * (*_x0).Object.Pose.Rot[0];
	const FLT x29 = -4 * x28;
	const FLT x30 = x6 * (*_x0).Object.Pose.Rot[3];
	const FLT x31 = x4 * (*_x0).Object.Pose.Rot[2];
	const FLT x32 = x2 * (*_x0).Object.Pose.Rot[1];
	const FLT x33 = x28 + x32;
	const FLT x34 = (-1 * x31) + x33 + x30;
	const FLT x35 = -4 * x25;
	const FLT x36 = 2 * (1. / sqrt(1 + (-4 * (((x0 * x4) + (-1 * x2 * x6)) * ((x0 * x4) + (-1 * x2 * x6))))));
	const FLT x37 = (-1 * x24) + (-1 * x23);
	const FLT x38 = x1 + x3;
	const FLT x39 = 1 + (-2 * (x9 + (x2 * x2)));
	const FLT x40 = 2 * (1. / x39);
	const FLT x41 = x39 * x39;
	const FLT x42 = (x2 * x4) + (x0 * x6);
	const FLT x43 = 2 * (1. / x41) * x42;
	const FLT x44 = x41 * (1. / (x41 + (4 * (x42 * x42))));
	const FLT x45 = ((*_x1).Object.IMUBias.IMUCorrection[3] * (*_x0).Object.IMUBias.IMUCorrection[1]) +
					((*_x1).Object.IMUBias.IMUCorrection[2] * (*_x0).Object.IMUBias.IMUCorrection[0]) +
					(-1 * (*_x1).Object.IMUBias.IMUCorrection[1] * (*_x0).Object.IMUBias.IMUCorrection[3]) +
					(-1 * (*_x1).Object.IMUBias.IMUCorrection[0] * (*_x0).Object.IMUBias.IMUCorrection[2]);
	const FLT x46 = x45 * (*_x0).Object.IMUBias.IMUCorrection[3];
	const FLT x47 = (-1 * (*_x1).Object.IMUBias.IMUCorrection[2] * (*_x0).Object.IMUBias.IMUCorrection[1]) +
					(-1 * (*_x1).Object.IMUBias.IMUCorrection[0] * (*_x0).Object.IMUBias.IMUCorrection[3]) +
					((*_x1).Object.IMUBias.IMUCorrection[1] * (*_x0).Object.IMUBias.IMUCorrection[2]) +
					((*_x1).Object.IMUBias.IMUCorrection[3] * (*_x0).Object.IMUBias.IMUCorrection[0]);
	const FLT x48 = x47 * (*_x0).Object.IMUBias.IMUCorrection[2];
	const FLT x49 = ((*_x1).Object.IMUBias.IMUCorrection[1] * (*_x0).Object.IMUBias.IMUCorrection[1]) +
					((*_x1).Object.IMUBias.IMUCorrection[2] * (*_x0).Object.IMUBias.IMUCorrection[2]) +
					((*_x1).Object.IMUBias.IMUCorrection[0] * (*_x0).Object.IMUBias.IMUCorrection[0]) +
					((*_x1).Object.IMUBias.IMUCorrection[3] * (*_x0).Object.IMUBias.IMUCorrection[3]);
	const FLT x50 = x49 * (*_x0).Object.IMUBias.IMUCorrection[1];
	const FLT x51 = ((*_x1).Object.IMUBias.IMUCorrection[2] * (*_x0).Object.IMUBias.IMUCorrection[3]) +
					(-1 * (*_x1).Object.IMUBias.IMUCorrection[0] * (*_x0).Object.IMUBias.IMUCorrection[1]) +
					((*_x1).Object.IMUBias.IMUCorrection[1] * (*_x0).Object.IMUBias.IMUCorrection[0]) +
					(-1 * (*_x1).Object.IMUBias.IMUCorrection[3] * (*_x0).Object.IMUBias.IMUCorrection[2]);
	const FLT x52 = x51 * (*_x0).Object.IMUBias.IMUCorrection[0];
	const FLT x53 = x52 + (-1 * x50);
	const FLT x54 = x45 * x45;
	const FLT x55 = 1 + (-2 * ((x51 * x51) + x54));
	const FLT x56 = 2 * (1. / x55);
	const FLT x57 = x45 * (*_x0).Object.IMUBias.IMUCorrection[2];
	const FLT x58 = 4 * x57;
	const FLT x59 = x51 * (*_x0).Object.IMUBias.IMUCorrection[1];
	const FLT x60 = x55 * x55;
	const FLT x61 = (x51 * x49) + (x45 * x47);
	const FLT x62 = 2 * (1. / x60) * x61;
	const FLT x63 = x60 * (1. / (x60 + (4 * (x61 * x61))));
	const FLT x64 = x47 * (*_x0).Object.IMUBias.IMUCorrection[3];
	const FLT x65 = x59 + (x49 * (*_x0).Object.IMUBias.IMUCorrection[0]);
	const FLT x66 = x65 + x57 + (-1 * x64);
	const FLT x67 = 4 * x46;
	const FLT x68 = x51 * (*_x0).Object.IMUBias.IMUCorrection[2];
	const FLT x69 = x49 * (*_x0).Object.IMUBias.IMUCorrection[3];
	const FLT x70 = x45 * (*_x0).Object.IMUBias.IMUCorrection[1];
	const FLT x71 = x47 * (*_x0).Object.IMUBias.IMUCorrection[0];
	const FLT x72 = x71 + (-1 * x70);
	const FLT x73 = x45 * (*_x0).Object.IMUBias.IMUCorrection[0];
	const FLT x74 = -4 * x73;
	const FLT x75 = x51 * (*_x0).Object.IMUBias.IMUCorrection[3];
	const FLT x76 = x49 * (*_x0).Object.IMUBias.IMUCorrection[2];
	const FLT x77 = x47 * (*_x0).Object.IMUBias.IMUCorrection[1];
	const FLT x78 = x77 + x73;
	const FLT x79 = x78 + x75 + (-1 * x76);
	const FLT x80 = -4 * x70;
	const FLT x81 = 2 * (1. / sqrt(1 + (-4 * (((x45 * x49) + (-1 * x51 * x47)) * ((x45 * x49) + (-1 * x51 * x47))))));
	const FLT x82 = (-1 * x69) + (-1 * x68);
	const FLT x83 = x46 + x48;
	const FLT x84 = 1 + (-2 * (x54 + (x47 * x47)));
	const FLT x85 = 2 * (1. / x84);
	const FLT x86 = x84 * x84;
	const FLT x87 = (x47 * x49) + (x51 * x45);
	const FLT x88 = 2 * (1. / x86) * x87;
	const FLT x89 = x86 * (1. / (x86 + (4 * (x87 * x87))));
	const FLT x90 = ((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[1]) +
					(-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[2]) +
					((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[0]) +
					(-1 * (*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[3]);
	const FLT x91 = x90 * (*_x0).Lighthouse.Rot[3];
	const FLT x92 = (-1 * (*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[1]) +
					((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[0]) +
					(-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[3]) +
					((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[2]);
	const FLT x93 = x92 * (*_x0).Lighthouse.Rot[2];
	const FLT x94 = (-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[1]) +
					((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[3]) +
					(-1 * (*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[2]) +
					((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[0]);
	const FLT x95 = x94 * (*_x0).Lighthouse.Rot[0];
	const FLT x96 =
		((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[1]) + ((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[2]) +
		((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[3]) + ((*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[0]);
	const FLT x97 = x96 * (*_x0).Lighthouse.Rot[1];
	const FLT x98 = (-1 * x97) + x95;
	const FLT x99 = x90 * x90;
	const FLT x100 = 1 + (-2 * ((x94 * x94) + x99));
	const FLT x101 = 2 * (1. / x100);
	const FLT x102 = x90 * (*_x0).Lighthouse.Rot[2];
	const FLT x103 = 4 * x102;
	const FLT x104 = x94 * (*_x0).Lighthouse.Rot[1];
	const FLT x105 = x100 * x100;
	const FLT x106 = (x96 * x94) + (x92 * x90);
	const FLT x107 = 2 * (1. / x105) * x106;
	const FLT x108 = x105 * (1. / (x105 + (4 * (x106 * x106))));
	const FLT x109 = x92 * (*_x0).Lighthouse.Rot[3];
	const FLT x110 = x104 + (x96 * (*_x0).Lighthouse.Rot[0]);
	const FLT x111 = x102 + x110 + (-1 * x109);
	const FLT x112 = 4 * x91;
	const FLT x113 = x94 * (*_x0).Lighthouse.Rot[2];
	const FLT x114 = x96 * (*_x0).Lighthouse.Rot[3];
	const FLT x115 = x90 * (*_x0).Lighthouse.Rot[1];
	const FLT x116 = x92 * (*_x0).Lighthouse.Rot[0];
	const FLT x117 = x116 + (-1 * x115);
	const FLT x118 = x90 * (*_x0).Lighthouse.Rot[0];
	const FLT x119 = -4 * x118;
	const FLT x120 = x94 * (*_x0).Lighthouse.Rot[3];
	const FLT x121 = x96 * (*_x0).Lighthouse.Rot[2];
	const FLT x122 = x92 * (*_x0).Lighthouse.Rot[1];
	const FLT x123 = x118 + x122;
	const FLT x124 = x123 + x120 + (-1 * x121);
	const FLT x125 = -4 * x115;
	const FLT x126 = 2 * (1. / sqrt(1 + (-4 * (((x90 * x96) + (-1 * x92 * x94)) * ((x90 * x96) + (-1 * x92 * x94))))));
	const FLT x127 = (-1 * x113) + (-1 * x114);
	const FLT x128 = x91 + x93;
	const FLT x129 = 1 + (-2 * (x99 + (x92 * x92)));
	const FLT x130 = 2 * (1. / x129);
	const FLT x131 = x129 * x129;
	const FLT x132 = (x92 * x96) + (x90 * x94);
	const FLT x133 = 2 * (1. / x131) * x132;
	const FLT x134 = x131 * (1. / (x131 + (4 * (x132 * x132))));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT),
						x18 * ((-1 * ((4 * x14) + x13) * x17) + ((x8 + (-1 * x1) + (-1 * x3)) * x11)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT),
						x18 * ((-1 * x17 * ((-4 * x7) + x22)) + (x21 * x11)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT),
						x18 * ((-1 * ((-4 * x30) + x29) * x17) + (x11 * (x23 + x27 + x24))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT),
						x18 * ((-1 * ((4 * x24) + x35) * x17) + (x34 * x11)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT), x34 * x36);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT),
						x36 * (x37 + x25 + (-1 * x26)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT), x36 * x21);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT),
						x36 * (x38 + x5 + (-1 * x7)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT),
						((-1 * (x13 + (4 * x19)) * x43) + ((x37 + x27) * x40)) * x44);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT),
						x44 * ((-1 * x43 * (x22 + (-4 * x3))) + (x40 * (x31 + x33 + (-1 * x30)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT),
						x44 * ((-1 * (x29 + (4 * x32)) * x43) + (x40 * (x38 + x8))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT),
						x44 * ((-1 * (x35 + (-4 * x26)) * x43) + (x40 * (x20 + x19 + (-1 * x12)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Acc[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Acc[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Acc[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Acc[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Acc[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Acc[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccScale[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccScale[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccScale[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccScale[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccScale[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccScale[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						x63 * ((-1 * ((4 * x59) + x58) * x62) + (x56 * (x53 + (-1 * x46) + (-1 * x48)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						x63 * ((-1 * ((-4 * x52) + x67) * x62) + (x66 * x56)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						x63 * ((-1 * ((-4 * x75) + x74) * x62) + (x56 * (x72 + x68 + x69))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[3]) / sizeof(FLT),
						x63 * ((-1 * ((4 * x68) + x80) * x62) + (x79 * x56)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT), x81 * x79);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						x81 * (x82 + x70 + (-1 * x71)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT), x81 * x66);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[3]) / sizeof(FLT),
						x81 * (x50 + x83 + (-1 * x52)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						x89 * ((-1 * (x58 + (4 * x64)) * x88) + ((x72 + x82) * x85)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						x89 * ((-1 * (x67 + (-4 * x48)) * x88) + (x85 * (x78 + x76 + (-1 * x75)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						x89 * ((-1 * (x74 + (4 * x77)) * x88) + ((x83 + x53) * x85)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[3]) / sizeof(FLT),
						x89 * ((-1 * (x80 + (-4 * x71)) * x88) + (x85 * (x65 + x64 + (-1 * x57)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccBias[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccBias[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccBias[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccBias[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.GyroBias[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.GyroBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.GyroBias[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.GyroBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.GyroBias[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.GyroBias[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						x108 * ((-1 * ((4 * x104) + x103) * x107) + ((x98 + (-1 * x91) + (-1 * x93)) * x101)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						x108 * ((-1 * x107 * ((-4 * x95) + x112)) + (x101 * x111)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						x108 * ((-1 * ((-4 * x120) + x119) * x107) + (x101 * (x117 + x113 + x114))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						x108 * ((-1 * ((4 * x113) + x125) * x107) + (x101 * x124)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT), x124 * x126);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						x126 * (x127 + x115 + (-1 * x116)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT), x111 * x126);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						x126 * (x128 + x97 + (-1 * x95)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						((-1 * (x103 + (4 * x109)) * x133) + ((x117 + x127) * x130)) * x134);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						x134 * ((-1 * x133 * (x112 + (-4 * x93))) + (x130 * (x121 + x123 + (-1 * x120)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						x134 * ((-1 * (x119 + (4 * x122)) * x133) + (x130 * (x128 + x98))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						x134 * ((-1 * (x125 + (-4 * x116)) * x133) + (x130 * ((-1 * x102) + x110 + x109))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD0.phase) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.phase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD0.tilt) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.tilt) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD0.curve) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.curve) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD0.gibpha) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.gibpha) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD0.gibmag) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.gibmag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD0.ogeephase) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.ogeephase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD0.ogeemag) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.ogeemag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD1.phase) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.phase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD1.tilt) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.tilt) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD1.curve) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.curve) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD1.gibpha) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.gibpha) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD1.gibmag) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.gibmag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD1.ogeephase) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.ogeephase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD1.ogeemag) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.ogeemag) / sizeof(FLT), 1);
}

// Full version Jacobian of SurviveJointKalmanModelToErrorModel wrt [(*_x1).Lighthouse.Pos[0], (*_x1).Lighthouse.Pos[1],
// (*_x1).Lighthouse.Pos[2], (*_x1).Lighthouse.Rot[0], (*_x1).Lighthouse.Rot[1], (*_x1).Lighthouse.Rot[2],
// (*_x1).Lighthouse.Rot[3], (*_x1).Object.Acc[0], (*_x1).Object.Acc[1], (*_x1).Object.Acc[2],
// (*_x1).Object.IMUBias.AccBias[0], (*_x1).Object.IMUBias.AccBias[1], (*_x1).Object.IMUBias.AccBias[2],
// (*_x1).Object.IMUBias.AccScale[0], (*_x1).Object.IMUBias.AccScale[1], (*_x1).Object.IMUBias.AccScale[2],
// (*_x1).Object.IMUBias.GyroBias[0], (*_x1).Object.IMUBias.GyroBias[1], (*_x1).Object.IMUBias.GyroBias[2],
// (*_x1).Object.IMUBias.IMUCorrection[0], (*_x1).Object.IMUBias.IMUCorrection[1],
// (*_x1).Object.IMUBias.IMUCorrection[2], (*_x1).Object.IMUBias.IMUCorrection[3], (*_x1).Object.Pose.Pos[0],
// (*_x1).Object.Pose.Pos[1], (*_x1).Object.Pose.Pos[2], (*_x1).Object.Pose.Rot[0], (*_x1).Object.Pose.Rot[1],
// (*_x1).Object.Pose.Rot[2], (*_x1).Object.Pose.Rot[3], (*_x1).Object.Velocity.AxisAngleRot[0],
// (*_x1).Object.Velocity.AxisAngleRot[1], (*_x1).Object.Velocity.AxisAngleRot[2], (*_x1).Object.Velocity.Pos[0],
// (*_x1).Object.Velocity.Pos[1], (*_x1).Object.Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f4d1c386f40>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c396250>, <cnkalman.codegen.WrapMember object at 0x7f4d1c396040>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c396310>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386fa0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3962b0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c396100>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3963d0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3960a0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c396370>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386d30>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c396190>, <cnkalman.codegen.WrapMember object at 0x7f4d1c386ee0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3961f0>] Jacobian of SurviveJointKalmanModelToErrorModel wrt
// [(*_x0).Lighthouse.Pos[0], (*_x0).Lighthouse.Pos[1], (*_x0).Lighthouse.Pos[2], (*_x0).Lighthouse.Rot[0],
// (*_x0).Lighthouse.Rot[1], (*_x0).Lighthouse.Rot[2], (*_x0).Lighthouse.Rot[3], (*_x0).Object.Acc[0],
// (*_x0).Object.Acc[1], (*_x0).Object.Acc[2], (*_x0).Object.IMUBias.AccBias[0], (*_x0).Object.IMUBias.AccBias[1],
// (*_x0).Object.IMUBias.AccBias[2], (*_x0).Object.IMUBias.AccScale[0], (*_x0).Object.IMUBias.AccScale[1],
// (*_x0).Object.IMUBias.AccScale[2], (*_x0).Object.IMUBias.GyroBias[0], (*_x0).Object.IMUBias.GyroBias[1],
// (*_x0).Object.IMUBias.GyroBias[2], (*_x0).Object.IMUBias.IMUCorrection[0], (*_x0).Object.IMUBias.IMUCorrection[1],
// (*_x0).Object.IMUBias.IMUCorrection[2], (*_x0).Object.IMUBias.IMUCorrection[3], (*_x0).Object.Pose.Pos[0],
// (*_x0).Object.Pose.Pos[1], (*_x0).Object.Pose.Pos[2], (*_x0).Object.Pose.Rot[0], (*_x0).Object.Pose.Rot[1],
// (*_x0).Object.Pose.Rot[2], (*_x0).Object.Pose.Rot[3], (*_x0).Object.Velocity.AxisAngleRot[0],
// (*_x0).Object.Velocity.AxisAngleRot[1], (*_x0).Object.Velocity.AxisAngleRot[2], (*_x0).Object.Velocity.Pos[0],
// (*_x0).Object.Velocity.Pos[1], (*_x0).Object.Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f4d1c398820>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c398af0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3988e0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c398bb0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c398880>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c398b50>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3989a0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c398c70>, <cnkalman.codegen.WrapMember object at 0x7f4d1c398940>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c398c10>, <cnkalman.codegen.WrapMember object at 0x7f4d1c398610>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c398a30>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3987c0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c398a90>]
static inline void SurviveJointKalmanModelToErrorModel_jac_x0(CnMat *Hx, const SurviveJointKalmanModel *_x1,
															  const SurviveJointKalmanModel *_x0) {
	const FLT x0 = ((*_x1).Object.Pose.Rot[3] * (*_x0).Object.Pose.Rot[1]) +
				   (-1 * (*_x1).Object.Pose.Rot[0] * (*_x0).Object.Pose.Rot[2]) +
				   ((*_x1).Object.Pose.Rot[2] * (*_x0).Object.Pose.Rot[0]) +
				   (-1 * (*_x1).Object.Pose.Rot[1] * (*_x0).Object.Pose.Rot[3]);
	const FLT x1 = x0 * (*_x1).Object.Pose.Rot[3];
	const FLT x2 = (-1 * (*_x1).Object.Pose.Rot[0] * (*_x0).Object.Pose.Rot[1]) +
				   ((*_x1).Object.Pose.Rot[1] * (*_x0).Object.Pose.Rot[0]) +
				   ((*_x1).Object.Pose.Rot[2] * (*_x0).Object.Pose.Rot[3]) +
				   (-1 * (*_x1).Object.Pose.Rot[3] * (*_x0).Object.Pose.Rot[2]);
	const FLT x3 = x2 * (*_x1).Object.Pose.Rot[0];
	const FLT x4 = x3 + x1;
	const FLT x5 = ((*_x1).Object.Pose.Rot[1] * (*_x0).Object.Pose.Rot[1]) +
				   ((*_x1).Object.Pose.Rot[3] * (*_x0).Object.Pose.Rot[3]) +
				   ((*_x1).Object.Pose.Rot[0] * (*_x0).Object.Pose.Rot[0]) +
				   ((*_x1).Object.Pose.Rot[2] * (*_x0).Object.Pose.Rot[2]);
	const FLT x6 = x5 * (*_x1).Object.Pose.Rot[1];
	const FLT x7 = (-1 * (*_x1).Object.Pose.Rot[2] * (*_x0).Object.Pose.Rot[1]) +
				   ((*_x1).Object.Pose.Rot[1] * (*_x0).Object.Pose.Rot[2]) +
				   ((*_x1).Object.Pose.Rot[3] * (*_x0).Object.Pose.Rot[0]) +
				   (-1 * (*_x1).Object.Pose.Rot[0] * (*_x0).Object.Pose.Rot[3]);
	const FLT x8 = x7 * (*_x1).Object.Pose.Rot[2];
	const FLT x9 = x8 + x6;
	const FLT x10 = x0 * x0;
	const FLT x11 = 1 + (-2 * ((x2 * x2) + x10));
	const FLT x12 = 2 * (1. / x11);
	const FLT x13 = x0 * (*_x1).Object.Pose.Rot[2];
	const FLT x14 = -4 * x13;
	const FLT x15 = x2 * (*_x1).Object.Pose.Rot[1];
	const FLT x16 = x11 * x11;
	const FLT x17 = (x2 * x5) + (x0 * x7);
	const FLT x18 = 2 * x17 * (1. / x16);
	const FLT x19 = x16 * (1. / (x16 + (4 * (x17 * x17))));
	const FLT x20 = x7 * (*_x1).Object.Pose.Rot[3];
	const FLT x21 = x20 + (-1 * x5 * (*_x1).Object.Pose.Rot[0]);
	const FLT x22 = -4 * x1;
	const FLT x23 = x5 * (*_x1).Object.Pose.Rot[3];
	const FLT x24 = x7 * (*_x1).Object.Pose.Rot[0];
	const FLT x25 = x2 * (*_x1).Object.Pose.Rot[2];
	const FLT x26 = x0 * (*_x1).Object.Pose.Rot[1];
	const FLT x27 = x26 + x25;
	const FLT x28 = x0 * (*_x1).Object.Pose.Rot[0];
	const FLT x29 = 4 * x28;
	const FLT x30 = x2 * (*_x1).Object.Pose.Rot[3];
	const FLT x31 = x5 * (*_x1).Object.Pose.Rot[2];
	const FLT x32 = x7 * (*_x1).Object.Pose.Rot[1];
	const FLT x33 = (-1 * x32) + x31;
	const FLT x34 = (-1 * x28) + x30;
	const FLT x35 = 4 * x26;
	const FLT x36 = 2 * (1. / sqrt(1 + (-4 * (((x0 * x5) + (-1 * x2 * x7)) * ((x0 * x5) + (-1 * x2 * x7))))));
	const FLT x37 = x23 + x27 + x24;
	const FLT x38 = x21 + x13 + (-1 * x15);
	const FLT x39 = 1 + (-2 * (x10 + (x7 * x7)));
	const FLT x40 = 2 * (1. / x39);
	const FLT x41 = x39 * x39;
	const FLT x42 = (x5 * x7) + (x0 * x2);
	const FLT x43 = 2 * (1. / x41) * x42;
	const FLT x44 = x41 * (1. / (x41 + (4 * (x42 * x42))));
	const FLT x45 = (-1 * (*_x1).Object.IMUBias.IMUCorrection[2] * (*_x0).Object.IMUBias.IMUCorrection[1]) +
					(-1 * (*_x1).Object.IMUBias.IMUCorrection[0] * (*_x0).Object.IMUBias.IMUCorrection[3]) +
					((*_x1).Object.IMUBias.IMUCorrection[1] * (*_x0).Object.IMUBias.IMUCorrection[2]) +
					((*_x1).Object.IMUBias.IMUCorrection[3] * (*_x0).Object.IMUBias.IMUCorrection[0]);
	const FLT x46 = x45 * (*_x1).Object.IMUBias.IMUCorrection[2];
	const FLT x47 = ((*_x1).Object.IMUBias.IMUCorrection[1] * (*_x0).Object.IMUBias.IMUCorrection[1]) +
					((*_x1).Object.IMUBias.IMUCorrection[2] * (*_x0).Object.IMUBias.IMUCorrection[2]) +
					((*_x1).Object.IMUBias.IMUCorrection[0] * (*_x0).Object.IMUBias.IMUCorrection[0]) +
					((*_x1).Object.IMUBias.IMUCorrection[3] * (*_x0).Object.IMUBias.IMUCorrection[3]);
	const FLT x48 = x47 * (*_x1).Object.IMUBias.IMUCorrection[1];
	const FLT x49 = x48 + x46;
	const FLT x50 = ((*_x1).Object.IMUBias.IMUCorrection[3] * (*_x0).Object.IMUBias.IMUCorrection[1]) +
					((*_x1).Object.IMUBias.IMUCorrection[2] * (*_x0).Object.IMUBias.IMUCorrection[0]) +
					(-1 * (*_x1).Object.IMUBias.IMUCorrection[1] * (*_x0).Object.IMUBias.IMUCorrection[3]) +
					(-1 * (*_x1).Object.IMUBias.IMUCorrection[0] * (*_x0).Object.IMUBias.IMUCorrection[2]);
	const FLT x51 = x50 * (*_x1).Object.IMUBias.IMUCorrection[3];
	const FLT x52 = ((*_x1).Object.IMUBias.IMUCorrection[2] * (*_x0).Object.IMUBias.IMUCorrection[3]) +
					(-1 * (*_x1).Object.IMUBias.IMUCorrection[0] * (*_x0).Object.IMUBias.IMUCorrection[1]) +
					((*_x1).Object.IMUBias.IMUCorrection[1] * (*_x0).Object.IMUBias.IMUCorrection[0]) +
					(-1 * (*_x1).Object.IMUBias.IMUCorrection[3] * (*_x0).Object.IMUBias.IMUCorrection[2]);
	const FLT x53 = x52 * (*_x1).Object.IMUBias.IMUCorrection[0];
	const FLT x54 = x53 + x51;
	const FLT x55 = x50 * x50;
	const FLT x56 = 1 + (-2 * ((x52 * x52) + x55));
	const FLT x57 = 2 * (1. / x56);
	const FLT x58 = x50 * (*_x1).Object.IMUBias.IMUCorrection[2];
	const FLT x59 = -4 * x58;
	const FLT x60 = x52 * (*_x1).Object.IMUBias.IMUCorrection[1];
	const FLT x61 = x56 * x56;
	const FLT x62 = (x52 * x47) + (x50 * x45);
	const FLT x63 = 2 * (1. / x61) * x62;
	const FLT x64 = x61 * (1. / (x61 + (4 * (x62 * x62))));
	const FLT x65 = x45 * (*_x1).Object.IMUBias.IMUCorrection[3];
	const FLT x66 = x65 + (-1 * x47 * (*_x1).Object.IMUBias.IMUCorrection[0]);
	const FLT x67 = -4 * x51;
	const FLT x68 = x47 * (*_x1).Object.IMUBias.IMUCorrection[3];
	const FLT x69 = x45 * (*_x1).Object.IMUBias.IMUCorrection[0];
	const FLT x70 = x52 * (*_x1).Object.IMUBias.IMUCorrection[2];
	const FLT x71 = x50 * (*_x1).Object.IMUBias.IMUCorrection[1];
	const FLT x72 = x71 + x70;
	const FLT x73 = x50 * (*_x1).Object.IMUBias.IMUCorrection[0];
	const FLT x74 = 4 * x73;
	const FLT x75 = x52 * (*_x1).Object.IMUBias.IMUCorrection[3];
	const FLT x76 = x75 + (-1 * x73);
	const FLT x77 = x47 * (*_x1).Object.IMUBias.IMUCorrection[2];
	const FLT x78 = x45 * (*_x1).Object.IMUBias.IMUCorrection[1];
	const FLT x79 = (-1 * x78) + x77;
	const FLT x80 = 4 * x71;
	const FLT x81 = 2 * (1. / sqrt(1 + (-4 * (((x50 * x47) + (-1 * x52 * x45)) * ((x50 * x47) + (-1 * x52 * x45))))));
	const FLT x82 = x72 + x68 + x69;
	const FLT x83 = x66 + x58 + (-1 * x60);
	const FLT x84 = 1 + (-2 * (x55 + (x45 * x45)));
	const FLT x85 = 2 * (1. / x84);
	const FLT x86 = x84 * x84;
	const FLT x87 = (x45 * x47) + (x50 * x52);
	const FLT x88 = 2 * (1. / x86) * x87;
	const FLT x89 = x86 * (1. / (x86 + (4 * (x87 * x87))));
	const FLT x90 = ((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[1]) +
					(-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[2]) +
					((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[0]) +
					(-1 * (*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[3]);
	const FLT x91 = x90 * (*_x1).Lighthouse.Rot[3];
	const FLT x92 = (-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[1]) +
					((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[3]) +
					(-1 * (*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[2]) +
					((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[0]);
	const FLT x93 = x92 * (*_x1).Lighthouse.Rot[0];
	const FLT x94 = x93 + x91;
	const FLT x95 = (-1 * (*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[1]) +
					((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[0]) +
					(-1 * (*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[3]) +
					((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[2]);
	const FLT x96 = x95 * (*_x1).Lighthouse.Rot[2];
	const FLT x97 =
		((*_x1).Lighthouse.Rot[1] * (*_x0).Lighthouse.Rot[1]) + ((*_x1).Lighthouse.Rot[2] * (*_x0).Lighthouse.Rot[2]) +
		((*_x1).Lighthouse.Rot[3] * (*_x0).Lighthouse.Rot[3]) + ((*_x1).Lighthouse.Rot[0] * (*_x0).Lighthouse.Rot[0]);
	const FLT x98 = x97 * (*_x1).Lighthouse.Rot[1];
	const FLT x99 = x98 + x96;
	const FLT x100 = x90 * x90;
	const FLT x101 = 1 + (-2 * ((x92 * x92) + x100));
	const FLT x102 = 2 * (1. / x101);
	const FLT x103 = x90 * (*_x1).Lighthouse.Rot[2];
	const FLT x104 = -4 * x103;
	const FLT x105 = x92 * (*_x1).Lighthouse.Rot[1];
	const FLT x106 = x101 * x101;
	const FLT x107 = (x92 * x97) + (x90 * x95);
	const FLT x108 = 2 * x107 * (1. / x106);
	const FLT x109 = x106 * (1. / (x106 + (4 * (x107 * x107))));
	const FLT x110 = x95 * (*_x1).Lighthouse.Rot[3];
	const FLT x111 = x110 + (-1 * x97 * (*_x1).Lighthouse.Rot[0]);
	const FLT x112 = -4 * x91;
	const FLT x113 = x97 * (*_x1).Lighthouse.Rot[3];
	const FLT x114 = x95 * (*_x1).Lighthouse.Rot[0];
	const FLT x115 = x92 * (*_x1).Lighthouse.Rot[2];
	const FLT x116 = x90 * (*_x1).Lighthouse.Rot[1];
	const FLT x117 = x116 + x115;
	const FLT x118 = x90 * (*_x1).Lighthouse.Rot[0];
	const FLT x119 = 4 * x118;
	const FLT x120 = x92 * (*_x1).Lighthouse.Rot[3];
	const FLT x121 = x97 * (*_x1).Lighthouse.Rot[2];
	const FLT x122 = x95 * (*_x1).Lighthouse.Rot[1];
	const FLT x123 = (-1 * x122) + x121;
	const FLT x124 = (-1 * x118) + x120;
	const FLT x125 = 4 * x116;
	const FLT x126 = 2 * (1. / sqrt(1 + (-4 * (((x90 * x97) + (-1 * x92 * x95)) * ((x90 * x97) + (-1 * x92 * x95))))));
	const FLT x127 = x117 + x113 + x114;
	const FLT x128 = x111 + x103 + (-1 * x105);
	const FLT x129 = 1 + (-2 * (x100 + (x95 * x95)));
	const FLT x130 = 2 * (1. / x129);
	const FLT x131 = x129 * x129;
	const FLT x132 = (x97 * x95) + (x92 * x90);
	const FLT x133 = 2 * (1. / x131) * x132;
	const FLT x134 = x131 * (1. / (x131 + (4 * (x132 * x132))));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Pos[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Pos[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Pos[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT),
						((-1 * ((-4 * x15) + x14) * x18) + ((x9 + x4) * x12)) * x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT),
						x19 * ((-1 * x18 * ((4 * x3) + x22)) + (x12 * (x21 + x15 + (-1 * x13)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT),
						x19 * ((-1 * ((4 * x30) + x29) * x18) + (x12 * ((-1 * x23) + x27 + (-1 * x24)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT),
						((-1 * ((-4 * x25) + x35) * x18) + ((x34 + x33) * x12)) * x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT),
						x36 * (x28 + x33 + (-1 * x30)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT), x36 * x37);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT), x36 * x38);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT),
						(x4 + (-1 * x6) + (-1 * x8)) * x36);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT),
						x44 * ((-1 * (x14 + (-4 * x20)) * x43) + (x40 * x37)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT),
						x44 * ((-1 * x43 * (x22 + (4 * x8))) + (x40 * (x34 + x32 + (-1 * x31)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT),
						x44 * ((-1 * (x29 + (-4 * x32)) * x43) + ((x9 + (-1 * x3) + (-1 * x1)) * x40)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT),
						x44 * ((-1 * (x35 + (4 * x24)) * x43) + (x40 * x38)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Acc[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Acc[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Acc[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Acc[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.Acc[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Acc[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccScale[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccScale[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccScale[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccScale[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccScale[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccScale[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						x64 * ((-1 * ((-4 * x60) + x59) * x63) + ((x54 + x49) * x57)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						x64 * ((-1 * ((4 * x53) + x67) * x63) + (x57 * (x66 + x60 + (-1 * x58)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						x64 * ((-1 * ((4 * x75) + x74) * x63) + (x57 * (x72 + (-1 * x68) + (-1 * x69)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[3]) / sizeof(FLT),
						x64 * ((-1 * ((-4 * x70) + x80) * x63) + ((x79 + x76) * x57)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						x81 * (x79 + x73 + (-1 * x75)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT), x81 * x82);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT), x81 * x83);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[3]) / sizeof(FLT),
						x81 * (x54 + (-1 * x48) + (-1 * x46)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						x89 * ((-1 * (x59 + (-4 * x65)) * x88) + (x82 * x85)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						x89 * ((-1 * (x67 + (4 * x46)) * x88) + (x85 * (x76 + x78 + (-1 * x77)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						x89 * ((-1 * (x74 + (-4 * x78)) * x88) + (x85 * (x49 + (-1 * x51) + (-1 * x53)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[3]) / sizeof(FLT),
						x89 * ((-1 * (x80 + (4 * x69)) * x88) + (x83 * x85)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccBias[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccBias[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccBias[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccBias[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccBias[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccBias[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.GyroBias[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.GyroBias[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.GyroBias[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.GyroBias[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.GyroBias[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.GyroBias[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Pos[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Pos[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Pos[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						((-1 * ((-4 * x105) + x104) * x108) + ((x99 + x94) * x102)) * x109);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						x109 * ((-1 * x108 * ((4 * x93) + x112)) + (x102 * (x111 + (-1 * x103) + x105))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						x109 * ((-1 * ((4 * x120) + x119) * x108) + (x102 * (x117 + (-1 * x113) + (-1 * x114)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						((-1 * ((-4 * x115) + x125) * x108) + ((x124 + x123) * x102)) * x109);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						x126 * (x123 + (-1 * x120) + x118));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT), x127 * x126);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT), x128 * x126);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						(x94 + (-1 * x98) + (-1 * x96)) * x126);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						x134 * ((-1 * (x104 + (-4 * x110)) * x133) + (x127 * x130)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						x134 * ((-1 * x133 * (x112 + (4 * x96))) + (x130 * (x124 + x122 + (-1 * x121)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						x134 * ((-1 * (x119 + (-4 * x122)) * x133) + ((x99 + (-1 * x91) + (-1 * x93)) * x130)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						x134 * ((-1 * (x125 + (4 * x114)) * x133) + (x128 * x130)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD0.phase) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.phase) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD0.tilt) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.tilt) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD0.curve) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.curve) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD0.gibpha) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.gibpha) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD0.gibmag) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.gibmag) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD0.ogeephase) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.ogeephase) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD0.ogeemag) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.ogeemag) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD1.phase) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.phase) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD1.tilt) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.tilt) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD1.curve) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.curve) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD1.gibpha) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.gibpha) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD1.gibmag) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.gibmag) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD1.ogeephase) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.ogeephase) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanErrorModel, BSD1.ogeemag) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.ogeemag) / sizeof(FLT), -1);
}

// Full version Jacobian of SurviveJointKalmanModelToErrorModel wrt [(*_x0).Lighthouse.Pos[0], (*_x0).Lighthouse.Pos[1],
// (*_x0).Lighthouse.Pos[2], (*_x0).Lighthouse.Rot[0], (*_x0).Lighthouse.Rot[1], (*_x0).Lighthouse.Rot[2],
// (*_x0).Lighthouse.Rot[3], (*_x0).Object.Acc[0], (*_x0).Object.Acc[1], (*_x0).Object.Acc[2],
// (*_x0).Object.IMUBias.AccBias[0], (*_x0).Object.IMUBias.AccBias[1], (*_x0).Object.IMUBias.AccBias[2],
// (*_x0).Object.IMUBias.AccScale[0], (*_x0).Object.IMUBias.AccScale[1], (*_x0).Object.IMUBias.AccScale[2],
// (*_x0).Object.IMUBias.GyroBias[0], (*_x0).Object.IMUBias.GyroBias[1], (*_x0).Object.IMUBias.GyroBias[2],
// (*_x0).Object.IMUBias.IMUCorrection[0], (*_x0).Object.IMUBias.IMUCorrection[1],
// (*_x0).Object.IMUBias.IMUCorrection[2], (*_x0).Object.IMUBias.IMUCorrection[3], (*_x0).Object.Pose.Pos[0],
// (*_x0).Object.Pose.Pos[1], (*_x0).Object.Pose.Pos[2], (*_x0).Object.Pose.Rot[0], (*_x0).Object.Pose.Rot[1],
// (*_x0).Object.Pose.Rot[2], (*_x0).Object.Pose.Rot[3], (*_x0).Object.Velocity.AxisAngleRot[0],
// (*_x0).Object.Velocity.AxisAngleRot[1], (*_x0).Object.Velocity.AxisAngleRot[2], (*_x0).Object.Velocity.Pos[0],
// (*_x0).Object.Velocity.Pos[1], (*_x0).Object.Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f4d1c398820>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c398af0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3988e0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c398bb0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c398880>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c398b50>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3989a0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c398c70>, <cnkalman.codegen.WrapMember object at 0x7f4d1c398940>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c398c10>, <cnkalman.codegen.WrapMember object at 0x7f4d1c398610>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c398a30>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3987c0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c398a90>]
static inline void SurviveJointKalmanModelAddErrorModel(SurviveJointKalmanModel *out,
														const SurviveJointKalmanModel *_x0,
														const SurviveJointKalmanErrorModel *error_state) {
	const FLT x0 = 0.5 * (*error_state).Object.Pose.AxisAngleRot[1];
	const FLT x1 = cos(x0);
	const FLT x2 = 0.5 * (*error_state).Object.Pose.AxisAngleRot[0];
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * (*error_state).Object.Pose.AxisAngleRot[2];
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = sin(x0);
	const FLT x8 = cos(x4);
	const FLT x9 = cos(x2);
	const FLT x10 = x8 * x9;
	const FLT x11 = (x7 * x10) + (x1 * x6);
	const FLT x12 = (x1 * x10) + (x6 * x7);
	const FLT x13 = x3 * x8;
	const FLT x14 = x5 * x9;
	const FLT x15 = (x1 * x14) + (-1 * x7 * x13);
	const FLT x16 = (x1 * x13) + (-1 * x7 * x14);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x15 * x15) + (x11 * x11) + (x12 * x12));
	const FLT x18 = x15 * x17;
	const FLT x19 = x11 * x17;
	const FLT x20 = x12 * x17;
	const FLT x21 = x17 * x16;
	const FLT x22 = 0.5 * (*error_state).Object.IMUBias.IMUCorrection[1];
	const FLT x23 = sin(x22);
	const FLT x24 = 0.5 * (*error_state).Object.IMUBias.IMUCorrection[0];
	const FLT x25 = sin(x24);
	const FLT x26 = 0.5 * (*error_state).Object.IMUBias.IMUCorrection[2];
	const FLT x27 = cos(x26);
	const FLT x28 = x25 * x27;
	const FLT x29 = cos(x24);
	const FLT x30 = cos(x22);
	const FLT x31 = sin(x26);
	const FLT x32 = x30 * x31;
	const FLT x33 = (x32 * x29) + (-1 * x23 * x28);
	const FLT x34 = x29 * x27;
	const FLT x35 = (x34 * x23) + (x32 * x25);
	const FLT x36 = x31 * x23;
	const FLT x37 = (x30 * x34) + (x36 * x25);
	const FLT x38 = (x30 * x28) + (-1 * x36 * x29);
	const FLT x39 = 1. / sqrt((x38 * x38) + (x37 * x37) + (x33 * x33) + (x35 * x35));
	const FLT x40 = x35 * x39;
	const FLT x41 = x37 * x39;
	const FLT x42 = x33 * x39;
	const FLT x43 = x38 * x39;
	const FLT x44 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[1];
	const FLT x45 = sin(x44);
	const FLT x46 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[2];
	const FLT x47 = cos(x46);
	const FLT x48 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[0];
	const FLT x49 = sin(x48);
	const FLT x50 = x47 * x49;
	const FLT x51 = cos(x44);
	const FLT x52 = cos(x48);
	const FLT x53 = sin(x46);
	const FLT x54 = x53 * x52;
	const FLT x55 = (x54 * x51) + (-1 * x50 * x45);
	const FLT x56 = x53 * x49;
	const FLT x57 = x52 * x47;
	const FLT x58 = (x57 * x45) + (x51 * x56);
	const FLT x59 = (x51 * x57) + (x56 * x45);
	const FLT x60 = (x50 * x51) + (-1 * x54 * x45);
	const FLT x61 = 1. / sqrt((x60 * x60) + (x59 * x59) + (x55 * x55) + (x58 * x58));
	const FLT x62 = x61 * x55;
	const FLT x63 = x61 * x58;
	const FLT x64 = x61 * x59;
	const FLT x65 = x61 * (*_x0).Lighthouse.Rot[1];
	const FLT x66 = x60 * x61;
	const FLT x67 = x61 * (*_x0).Lighthouse.Rot[2];
	out->Object.Pose.Pos[0] = (*_x0).Object.Pose.Pos[0] + (*error_state).Object.Pose.Pos[0];
	out->Object.Pose.Pos[1] = (*_x0).Object.Pose.Pos[1] + (*error_state).Object.Pose.Pos[1];
	out->Object.Pose.Pos[2] = (*_x0).Object.Pose.Pos[2] + (*error_state).Object.Pose.Pos[2];
	out->Object.Pose.Rot[0] = (x20 * (*_x0).Object.Pose.Rot[0]) + (-1 * x21 * (*_x0).Object.Pose.Rot[1]) +
							  (-1 * x18 * (*_x0).Object.Pose.Rot[3]) + (-1 * x19 * (*_x0).Object.Pose.Rot[2]);
	out->Object.Pose.Rot[1] = (x18 * (*_x0).Object.Pose.Rot[2]) + (-1 * x19 * (*_x0).Object.Pose.Rot[3]) +
							  (x20 * (*_x0).Object.Pose.Rot[1]) + (x21 * (*_x0).Object.Pose.Rot[0]);
	out->Object.Pose.Rot[2] = (-1 * x18 * (*_x0).Object.Pose.Rot[1]) + (x20 * (*_x0).Object.Pose.Rot[2]) +
							  (x21 * (*_x0).Object.Pose.Rot[3]) + (x19 * (*_x0).Object.Pose.Rot[0]);
	out->Object.Pose.Rot[3] = (x19 * (*_x0).Object.Pose.Rot[1]) + (x18 * (*_x0).Object.Pose.Rot[0]) +
							  (-1 * x21 * (*_x0).Object.Pose.Rot[2]) + (x20 * (*_x0).Object.Pose.Rot[3]);
	out->Object.Velocity.Pos[0] = (*_x0).Object.Velocity.Pos[0] + (*error_state).Object.Velocity.Pos[0];
	out->Object.Velocity.Pos[1] = (*_x0).Object.Velocity.Pos[1] + (*error_state).Object.Velocity.Pos[1];
	out->Object.Velocity.Pos[2] = (*_x0).Object.Velocity.Pos[2] + (*error_state).Object.Velocity.Pos[2];
	out->Object.Velocity.AxisAngleRot[0] =
		(*_x0).Object.Velocity.AxisAngleRot[0] + (*error_state).Object.Velocity.AxisAngleRot[0];
	out->Object.Velocity.AxisAngleRot[1] =
		(*_x0).Object.Velocity.AxisAngleRot[1] + (*error_state).Object.Velocity.AxisAngleRot[1];
	out->Object.Velocity.AxisAngleRot[2] =
		(*_x0).Object.Velocity.AxisAngleRot[2] + (*error_state).Object.Velocity.AxisAngleRot[2];
	out->Object.Acc[0] = (*_x0).Object.Acc[0] + (*error_state).Object.Acc[0];
	out->Object.Acc[1] = (*_x0).Object.Acc[1] + (*error_state).Object.Acc[1];
	out->Object.Acc[2] = (*_x0).Object.Acc[2] + (*error_state).Object.Acc[2];
	out->Object.IMUBias.AccScale[0] = (*_x0).Object.IMUBias.AccScale[0] + (*error_state).Object.IMUBias.AccScale[0];
	out->Object.IMUBias.AccScale[1] = (*_x0).Object.IMUBias.AccScale[1] + (*error_state).Object.IMUBias.AccScale[1];
	out->Object.IMUBias.AccScale[2] = (*_x0).Object.IMUBias.AccScale[2] + (*error_state).Object.IMUBias.AccScale[2];
	out->Object.IMUBias.IMUCorrection[0] =
		(-1 * x43 * (*_x0).Object.IMUBias.IMUCorrection[1]) + (-1 * x42 * (*_x0).Object.IMUBias.IMUCorrection[3]) +
		(-1 * x40 * (*_x0).Object.IMUBias.IMUCorrection[2]) + (x41 * (*_x0).Object.IMUBias.IMUCorrection[0]);
	out->Object.IMUBias.IMUCorrection[1] =
		(x41 * (*_x0).Object.IMUBias.IMUCorrection[1]) + (x43 * (*_x0).Object.IMUBias.IMUCorrection[0]) +
		(-1 * x40 * (*_x0).Object.IMUBias.IMUCorrection[3]) + (x42 * (*_x0).Object.IMUBias.IMUCorrection[2]);
	out->Object.IMUBias.IMUCorrection[2] =
		(-1 * x42 * (*_x0).Object.IMUBias.IMUCorrection[1]) + (x40 * (*_x0).Object.IMUBias.IMUCorrection[0]) +
		(x43 * (*_x0).Object.IMUBias.IMUCorrection[3]) + (x41 * (*_x0).Object.IMUBias.IMUCorrection[2]);
	out->Object.IMUBias.IMUCorrection[3] =
		(x40 * (*_x0).Object.IMUBias.IMUCorrection[1]) + (x42 * (*_x0).Object.IMUBias.IMUCorrection[0]) +
		(x41 * (*_x0).Object.IMUBias.IMUCorrection[3]) + (-1 * x43 * (*_x0).Object.IMUBias.IMUCorrection[2]);
	out->Object.IMUBias.AccBias[0] = (*_x0).Object.IMUBias.AccBias[0] + (*error_state).Object.IMUBias.AccBias[0];
	out->Object.IMUBias.AccBias[1] = (*_x0).Object.IMUBias.AccBias[1] + (*error_state).Object.IMUBias.AccBias[1];
	out->Object.IMUBias.AccBias[2] = (*_x0).Object.IMUBias.AccBias[2] + (*error_state).Object.IMUBias.AccBias[2];
	out->Object.IMUBias.GyroBias[0] = (*_x0).Object.IMUBias.GyroBias[0] + (*error_state).Object.IMUBias.GyroBias[0];
	out->Object.IMUBias.GyroBias[1] = (*_x0).Object.IMUBias.GyroBias[1] + (*error_state).Object.IMUBias.GyroBias[1];
	out->Object.IMUBias.GyroBias[2] = (*_x0).Object.IMUBias.GyroBias[2] + (*error_state).Object.IMUBias.GyroBias[2];
	out->Lighthouse.Pos[0] = (*_x0).Lighthouse.Pos[0] + (*error_state).Lighthouse.Pos[0];
	out->Lighthouse.Pos[1] = (*_x0).Lighthouse.Pos[1] + (*error_state).Lighthouse.Pos[1];
	out->Lighthouse.Pos[2] = (*_x0).Lighthouse.Pos[2] + (*error_state).Lighthouse.Pos[2];
	out->Lighthouse.Rot[0] = (-1 * x60 * x65) + (x64 * (*_x0).Lighthouse.Rot[0]) +
							 (-1 * x62 * (*_x0).Lighthouse.Rot[3]) + (-1 * x63 * (*_x0).Lighthouse.Rot[2]);
	out->Lighthouse.Rot[1] = (x65 * x59) + (x66 * (*_x0).Lighthouse.Rot[0]) + (-1 * x63 * (*_x0).Lighthouse.Rot[3]) +
							 (x62 * (*_x0).Lighthouse.Rot[2]);
	out->Lighthouse.Rot[2] =
		(-1 * x65 * x55) + (x66 * (*_x0).Lighthouse.Rot[3]) + (x63 * (*_x0).Lighthouse.Rot[0]) + (x67 * x59);
	out->Lighthouse.Rot[3] =
		(x62 * (*_x0).Lighthouse.Rot[0]) + (x65 * x58) + (-1 * x60 * x67) + (x64 * (*_x0).Lighthouse.Rot[3]);
	out->BSD0.phase = (*error_state).BSD0.phase + (*_x0).BSD0.phase;
	out->BSD0.tilt = (*error_state).BSD0.tilt + (*_x0).BSD0.tilt;
	out->BSD0.curve = (*error_state).BSD0.curve + (*_x0).BSD0.curve;
	out->BSD0.gibpha = (*error_state).BSD0.gibpha + (*_x0).BSD0.gibpha;
	out->BSD0.gibmag = (*error_state).BSD0.gibmag + (*_x0).BSD0.gibmag;
	out->BSD0.ogeephase = (*error_state).BSD0.ogeephase + (*_x0).BSD0.ogeephase;
	out->BSD0.ogeemag = (*error_state).BSD0.ogeemag + (*_x0).BSD0.ogeemag;
	out->BSD1.phase = (*error_state).BSD1.phase + (*_x0).BSD1.phase;
	out->BSD1.tilt = (*error_state).BSD1.tilt + (*_x0).BSD1.tilt;
	out->BSD1.curve = (*error_state).BSD1.curve + (*_x0).BSD1.curve;
	out->BSD1.gibpha = (*error_state).BSD1.gibpha + (*_x0).BSD1.gibpha;
	out->BSD1.gibmag = (*error_state).BSD1.gibmag + (*_x0).BSD1.gibmag;
	out->BSD1.ogeephase = (*error_state).BSD1.ogeephase + (*_x0).BSD1.ogeephase;
	out->BSD1.ogeemag = (*error_state).BSD1.ogeemag + (*_x0).BSD1.ogeemag;
}

// Jacobian of SurviveJointKalmanModelAddErrorModel wrt [(*_x0).Lighthouse.Pos[0], (*_x0).Lighthouse.Pos[1],
// (*_x0).Lighthouse.Pos[2], (*_x0).Lighthouse.Rot[0], (*_x0).Lighthouse.Rot[1], (*_x0).Lighthouse.Rot[2],
// (*_x0).Lighthouse.Rot[3], (*_x0).Object.Acc[0], (*_x0).Object.Acc[1], (*_x0).Object.Acc[2],
// (*_x0).Object.IMUBias.AccBias[0], (*_x0).Object.IMUBias.AccBias[1], (*_x0).Object.IMUBias.AccBias[2],
// (*_x0).Object.IMUBias.AccScale[0], (*_x0).Object.IMUBias.AccScale[1], (*_x0).Object.IMUBias.AccScale[2],
// (*_x0).Object.IMUBias.GyroBias[0], (*_x0).Object.IMUBias.GyroBias[1], (*_x0).Object.IMUBias.GyroBias[2],
// (*_x0).Object.IMUBias.IMUCorrection[0], (*_x0).Object.IMUBias.IMUCorrection[1],
// (*_x0).Object.IMUBias.IMUCorrection[2], (*_x0).Object.IMUBias.IMUCorrection[3], (*_x0).Object.Pose.Pos[0],
// (*_x0).Object.Pose.Pos[1], (*_x0).Object.Pose.Pos[2], (*_x0).Object.Pose.Rot[0], (*_x0).Object.Pose.Rot[1],
// (*_x0).Object.Pose.Rot[2], (*_x0).Object.Pose.Rot[3], (*_x0).Object.Velocity.AxisAngleRot[0],
// (*_x0).Object.Velocity.AxisAngleRot[1], (*_x0).Object.Velocity.AxisAngleRot[2], (*_x0).Object.Velocity.Pos[0],
// (*_x0).Object.Velocity.Pos[1], (*_x0).Object.Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f4d1c38e2e0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c38e220>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38e0a0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c38e520>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38e400>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c38e8b0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38e670>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c387df0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38ed30>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c387dc0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38e490>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c38e430>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38e640>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c38e730>]
static inline void SurviveJointKalmanModelAddErrorModel_jac_x0(CnMat *Hx, const SurviveJointKalmanModel *_x0,
															   const SurviveJointKalmanErrorModel *error_state) {
	const FLT x0 = 0.5 * (*error_state).Object.Pose.AxisAngleRot[1];
	const FLT x1 = cos(x0);
	const FLT x2 = 0.5 * (*error_state).Object.Pose.AxisAngleRot[2];
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * (*error_state).Object.Pose.AxisAngleRot[0];
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = sin(x0);
	const FLT x8 = cos(x4);
	const FLT x9 = cos(x2);
	const FLT x10 = x8 * x9;
	const FLT x11 = (x7 * x10) + (x1 * x6);
	const FLT x12 = (x1 * x10) + (x6 * x7);
	const FLT x13 = x5 * x9;
	const FLT x14 = x3 * x8;
	const FLT x15 = (x1 * x14) + (-1 * x7 * x13);
	const FLT x16 = (x1 * x13) + (-1 * x7 * x14);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x15 * x15) + (x11 * x11) + (x12 * x12));
	const FLT x18 = x12 * x17;
	const FLT x19 = x17 * x16;
	const FLT x20 = -1 * x19;
	const FLT x21 = x11 * x17;
	const FLT x22 = -1 * x21;
	const FLT x23 = x15 * x17;
	const FLT x24 = -1 * x23;
	const FLT x25 = 0.5 * (*error_state).Object.IMUBias.IMUCorrection[2];
	const FLT x26 = cos(x25);
	const FLT x27 = 0.5 * (*error_state).Object.IMUBias.IMUCorrection[0];
	const FLT x28 = sin(x27);
	const FLT x29 = 0.5 * (*error_state).Object.IMUBias.IMUCorrection[1];
	const FLT x30 = sin(x29);
	const FLT x31 = x30 * x28;
	const FLT x32 = sin(x25);
	const FLT x33 = cos(x29);
	const FLT x34 = cos(x27);
	const FLT x35 = x34 * x33;
	const FLT x36 = (x32 * x35) + (-1 * x31 * x26);
	const FLT x37 = x33 * x28;
	const FLT x38 = x30 * x34;
	const FLT x39 = (x38 * x26) + (x32 * x37);
	const FLT x40 = (x35 * x26) + (x32 * x31);
	const FLT x41 = (x37 * x26) + (-1 * x32 * x38);
	const FLT x42 = 1. / sqrt((x41 * x41) + (x40 * x40) + (x36 * x36) + (x39 * x39));
	const FLT x43 = x40 * x42;
	const FLT x44 = x41 * x42;
	const FLT x45 = -1 * x44;
	const FLT x46 = x42 * x39;
	const FLT x47 = -1 * x46;
	const FLT x48 = x42 * x36;
	const FLT x49 = -1 * x48;
	const FLT x50 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[1];
	const FLT x51 = sin(x50);
	const FLT x52 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[2];
	const FLT x53 = cos(x52);
	const FLT x54 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[0];
	const FLT x55 = sin(x54);
	const FLT x56 = x53 * x55;
	const FLT x57 = cos(x50);
	const FLT x58 = cos(x54);
	const FLT x59 = sin(x52);
	const FLT x60 = x58 * x59;
	const FLT x61 = (x60 * x57) + (-1 * x51 * x56);
	const FLT x62 = x55 * x59;
	const FLT x63 = x53 * x58;
	const FLT x64 = (x63 * x51) + (x62 * x57);
	const FLT x65 = (x63 * x57) + (x62 * x51);
	const FLT x66 = (x57 * x56) + (-1 * x60 * x51);
	const FLT x67 = 1. / sqrt((x66 * x66) + (x65 * x65) + (x61 * x61) + (x64 * x64));
	const FLT x68 = x67 * x65;
	const FLT x69 = x67 * x66;
	const FLT x70 = -1 * x69;
	const FLT x71 = x64 * x67;
	const FLT x72 = -1 * x71;
	const FLT x73 = x61 * x67;
	const FLT x74 = -1 * x73;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT), x20);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT), x22);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT), x24);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT), x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT), x23);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT), x22);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT), x21);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT), x24);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT), x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT), x23);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT), x21);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT), x20);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Acc[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Acc[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Acc[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Acc[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Acc[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.Acc[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.AccScale[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccScale[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.AccScale[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccScale[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.AccScale[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccScale[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT), x43);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT), x45);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT), x47);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[3]) / sizeof(FLT), x49);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT), x44);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT), x43);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT), x48);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[3]) / sizeof(FLT), x47);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT), x46);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT), x49);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT), x43);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[3]) / sizeof(FLT), x44);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT), x48);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT), x46);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT), x45);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[3]) / sizeof(FLT), x43);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.AccBias[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.AccBias[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.AccBias[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.AccBias[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.GyroBias[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.GyroBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.GyroBias[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.GyroBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.GyroBias[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Object.IMUBias.GyroBias[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT), x68);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT), x70);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT), x72);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT), x74);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT), x69);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT), x68);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT), x73);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT), x72);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT), x71);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT), x74);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT), x68);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT), x69);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT), x73);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT), x71);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT), x70);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT), x68);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD0.phase) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.phase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD0.tilt) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.tilt) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD0.curve) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.curve) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD0.gibpha) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.gibpha) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD0.gibmag) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.gibmag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD0.ogeephase) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.ogeephase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD0.ogeemag) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD0.ogeemag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD1.phase) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.phase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD1.tilt) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.tilt) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD1.curve) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.curve) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD1.gibpha) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.gibpha) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD1.gibmag) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.gibmag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD1.ogeephase) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.ogeephase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD1.ogeemag) / sizeof(FLT),
						offsetof(SurviveJointKalmanModel, BSD1.ogeemag) / sizeof(FLT), 1);
}

// Full version Jacobian of SurviveJointKalmanModelAddErrorModel wrt [(*_x0).Lighthouse.Pos[0],
// (*_x0).Lighthouse.Pos[1], (*_x0).Lighthouse.Pos[2], (*_x0).Lighthouse.Rot[0], (*_x0).Lighthouse.Rot[1],
// (*_x0).Lighthouse.Rot[2], (*_x0).Lighthouse.Rot[3], (*_x0).Object.Acc[0], (*_x0).Object.Acc[1], (*_x0).Object.Acc[2],
// (*_x0).Object.IMUBias.AccBias[0], (*_x0).Object.IMUBias.AccBias[1], (*_x0).Object.IMUBias.AccBias[2],
// (*_x0).Object.IMUBias.AccScale[0], (*_x0).Object.IMUBias.AccScale[1], (*_x0).Object.IMUBias.AccScale[2],
// (*_x0).Object.IMUBias.GyroBias[0], (*_x0).Object.IMUBias.GyroBias[1], (*_x0).Object.IMUBias.GyroBias[2],
// (*_x0).Object.IMUBias.IMUCorrection[0], (*_x0).Object.IMUBias.IMUCorrection[1],
// (*_x0).Object.IMUBias.IMUCorrection[2], (*_x0).Object.IMUBias.IMUCorrection[3], (*_x0).Object.Pose.Pos[0],
// (*_x0).Object.Pose.Pos[1], (*_x0).Object.Pose.Pos[2], (*_x0).Object.Pose.Rot[0], (*_x0).Object.Pose.Rot[1],
// (*_x0).Object.Pose.Rot[2], (*_x0).Object.Pose.Rot[3], (*_x0).Object.Velocity.AxisAngleRot[0],
// (*_x0).Object.Velocity.AxisAngleRot[1], (*_x0).Object.Velocity.AxisAngleRot[2], (*_x0).Object.Velocity.Pos[0],
// (*_x0).Object.Velocity.Pos[1], (*_x0).Object.Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f4d1c38e2e0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c38e220>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38e0a0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c38e520>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38e400>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c38e8b0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38e670>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c387df0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38ed30>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c387dc0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38e490>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c38e430>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38e640>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c38e730>] Jacobian of SurviveJointKalmanModelAddErrorModel wrt
// [(*error_state).Lighthouse.AxisAngleRot[0], (*error_state).Lighthouse.AxisAngleRot[1],
// (*error_state).Lighthouse.AxisAngleRot[2], (*error_state).Lighthouse.Pos[0], (*error_state).Lighthouse.Pos[1],
// (*error_state).Lighthouse.Pos[2], (*error_state).Object.Acc[0], (*error_state).Object.Acc[1],
// (*error_state).Object.Acc[2], (*error_state).Object.IMUBias.AccBias[0], (*error_state).Object.IMUBias.AccBias[1],
// (*error_state).Object.IMUBias.AccBias[2], (*error_state).Object.IMUBias.AccScale[0],
// (*error_state).Object.IMUBias.AccScale[1], (*error_state).Object.IMUBias.AccScale[2],
// (*error_state).Object.IMUBias.GyroBias[0], (*error_state).Object.IMUBias.GyroBias[1],
// (*error_state).Object.IMUBias.GyroBias[2], (*error_state).Object.IMUBias.IMUCorrection[0],
// (*error_state).Object.IMUBias.IMUCorrection[1], (*error_state).Object.IMUBias.IMUCorrection[2],
// (*error_state).Object.Pose.AxisAngleRot[0], (*error_state).Object.Pose.AxisAngleRot[1],
// (*error_state).Object.Pose.AxisAngleRot[2], (*error_state).Object.Pose.Pos[0], (*error_state).Object.Pose.Pos[1],
// (*error_state).Object.Pose.Pos[2], (*error_state).Object.Velocity.AxisAngleRot[0],
// (*error_state).Object.Velocity.AxisAngleRot[1], (*error_state).Object.Velocity.AxisAngleRot[2],
// (*error_state).Object.Velocity.Pos[0], (*error_state).Object.Velocity.Pos[1], (*error_state).Object.Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f4d1c35f430>, <cnkalman.codegen.WrapMember object at 0x7f4d1c377a90>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c35f1f0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c377f40>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c35f2b0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c377fa0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c35f220>, <cnkalman.codegen.WrapMember object at 0x7f4d1c377d90>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c35f250>, <cnkalman.codegen.WrapMember object at 0x7f4d1c377e50>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c35f880>, <cnkalman.codegen.WrapMember object at 0x7f4d1c35f6d0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c35f730>, <cnkalman.codegen.WrapMember object at 0x7f4d1c377220>]
static inline void
SurviveJointKalmanModelAddErrorModel_jac_error_state(CnMat *Hx, const SurviveJointKalmanModel *_x0,
													 const SurviveJointKalmanErrorModel *error_state) {
	const FLT x0 = 0.5 * (*error_state).Object.Pose.AxisAngleRot[2];
	const FLT x1 = sin(x0);
	const FLT x2 = 0.5 * (*error_state).Object.Pose.AxisAngleRot[1];
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * (*error_state).Object.Pose.AxisAngleRot[0];
	const FLT x5 = cos(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = x1 * x6;
	const FLT x8 = sin(x4);
	const FLT x9 = cos(x0);
	const FLT x10 = cos(x2);
	const FLT x11 = x9 * x10;
	const FLT x12 = x8 * x11;
	const FLT x13 = x12 + (-1 * x7);
	const FLT x14 = x1 * x10;
	const FLT x15 = x8 * x14;
	const FLT x16 = 0.5 * x15;
	const FLT x17 = x6 * x9;
	const FLT x18 = -0.5 * x17;
	const FLT x19 = x18 + (-1 * x16);
	const FLT x20 = x3 * x8;
	const FLT x21 = x9 * x20;
	const FLT x22 = x5 * x14;
	const FLT x23 = x22 + (-1 * x21);
	const FLT x24 = 2 * x23;
	const FLT x25 = x5 * x11;
	const FLT x26 = 0.5 * x25;
	const FLT x27 = x1 * x20;
	const FLT x28 = 0.5 * x27;
	const FLT x29 = x28 + x26;
	const FLT x30 = 2 * x13;
	const FLT x31 = 0.5 * x21;
	const FLT x32 = -1 * x31;
	const FLT x33 = 0.5 * x22;
	const FLT x34 = x33 + x32;
	const FLT x35 = x17 + x15;
	const FLT x36 = 2 * x35;
	const FLT x37 = 0.5 * x12;
	const FLT x38 = -1 * x37;
	const FLT x39 = 0.5 * x7;
	const FLT x40 = x39 + x38;
	const FLT x41 = x25 + x27;
	const FLT x42 = 2 * x41;
	const FLT x43 = (x23 * x23) + (x35 * x35) + (x13 * x13) + (x41 * x41);
	const FLT x44 = 1.0 / 2.0 * (1. / (x43 * sqrt(x43)));
	const FLT x45 = ((x40 * x42) + (x24 * x19) + (x34 * x36) + (x30 * x29)) * x44;
	const FLT x46 = x45 * x13;
	const FLT x47 = x45 * x23;
	const FLT x48 = 1. / sqrt(x43);
	const FLT x49 = x48 * (*_x0).Object.Pose.Rot[3];
	const FLT x50 = x49 * x19;
	const FLT x51 = x45 * (*_x0).Object.Pose.Rot[0];
	const FLT x52 = x48 * (*_x0).Object.Pose.Rot[2];
	const FLT x53 = x45 * x35;
	const FLT x54 = x48 * x29;
	const FLT x55 = -1 * x54 * (*_x0).Object.Pose.Rot[1];
	const FLT x56 = x48 * (*_x0).Object.Pose.Rot[0];
	const FLT x57 = -1 * x33;
	const FLT x58 = x57 + x32;
	const FLT x59 = x58 * x48;
	const FLT x60 = (-1 * x28) + x26;
	const FLT x61 = -1 * x39;
	const FLT x62 = x38 + x61;
	const FLT x63 = x16 + x18;
	const FLT x64 = ((x62 * x24) + (x63 * x42) + (x58 * x30) + (x60 * x36)) * x44;
	const FLT x65 = x64 * (*_x0).Object.Pose.Rot[0];
	const FLT x66 = x64 * x23;
	const FLT x67 = x62 * x48;
	const FLT x68 = x64 * x13;
	const FLT x69 = x64 * x35;
	const FLT x70 = x48 * (*_x0).Object.Pose.Rot[1];
	const FLT x71 = -1 * x70 * x19;
	const FLT x72 = x37 + x61;
	const FLT x73 = x31 + x57;
	const FLT x74 = ((x73 * x42) + (x24 * x29) + (x30 * x19) + (x72 * x36)) * x44;
	const FLT x75 = x74 * x23;
	const FLT x76 = x54 * (*_x0).Object.Pose.Rot[3];
	const FLT x77 = x74 * (*_x0).Object.Pose.Rot[0];
	const FLT x78 = x74 * x13;
	const FLT x79 = x74 * x35;
	const FLT x80 = x41 * x45;
	const FLT x81 = x54 * (*_x0).Object.Pose.Rot[0];
	const FLT x82 = x52 * x19;
	const FLT x83 = x64 * x41;
	const FLT x84 = x74 * x41;
	const FLT x85 = x56 * x19;
	const FLT x86 = x54 * (*_x0).Object.Pose.Rot[2];
	const FLT x87 = 0.5 * (*error_state).Object.IMUBias.IMUCorrection[0];
	const FLT x88 = cos(x87);
	const FLT x89 = 0.5 * (*error_state).Object.IMUBias.IMUCorrection[2];
	const FLT x90 = cos(x89);
	const FLT x91 = 0.5 * (*error_state).Object.IMUBias.IMUCorrection[1];
	const FLT x92 = cos(x91);
	const FLT x93 = x92 * x90;
	const FLT x94 = x88 * x93;
	const FLT x95 = 0.5 * x94;
	const FLT x96 = sin(x91);
	const FLT x97 = sin(x87);
	const FLT x98 = sin(x89);
	const FLT x99 = x98 * x97;
	const FLT x100 = x99 * x96;
	const FLT x101 = 0.5 * x100;
	const FLT x102 = x101 + x95;
	const FLT x103 = x90 * x96;
	const FLT x104 = x97 * x103;
	const FLT x105 = x88 * x98;
	const FLT x106 = x92 * x105;
	const FLT x107 = x106 + (-1 * x104);
	const FLT x108 = x92 * x99;
	const FLT x109 = x88 * x103;
	const FLT x110 = x109 + x108;
	const FLT x111 = x94 + x100;
	const FLT x112 = x96 * x105;
	const FLT x113 = x93 * x97;
	const FLT x114 = x113 + (-1 * x112);
	const FLT x115 = (x114 * x114) + (x107 * x107) + (x111 * x111) + (x110 * x110);
	const FLT x116 = 1. / sqrt(x115);
	const FLT x117 = x116 * (*_x0).Object.IMUBias.IMUCorrection[1];
	const FLT x118 = -1 * x102 * x117;
	const FLT x119 = 2 * x114;
	const FLT x120 = 0.5 * x108;
	const FLT x121 = -0.5 * x109;
	const FLT x122 = x121 + (-1 * x120);
	const FLT x123 = 2 * x107;
	const FLT x124 = 0.5 * x113;
	const FLT x125 = -1 * x124;
	const FLT x126 = 0.5 * x112;
	const FLT x127 = x126 + x125;
	const FLT x128 = 2 * x111;
	const FLT x129 = 0.5 * x104;
	const FLT x130 = -1 * x129;
	const FLT x131 = 0.5 * x106;
	const FLT x132 = x131 + x130;
	const FLT x133 = 2 * x110;
	const FLT x134 = (x133 * x132) + (x128 * x127) + (x102 * x119) + (x123 * x122);
	const FLT x135 = 1.0 / 2.0 * (1. / (x115 * sqrt(x115)));
	const FLT x136 = x107 * x135;
	const FLT x137 = x134 * x136;
	const FLT x138 = x110 * x135;
	const FLT x139 = x134 * x138;
	const FLT x140 = x116 * x132;
	const FLT x141 = x116 * (*_x0).Object.IMUBias.IMUCorrection[0];
	const FLT x142 = x134 * (*_x0).Object.IMUBias.IMUCorrection[0];
	const FLT x143 = x111 * x135;
	const FLT x144 = x114 * x135;
	const FLT x145 = x134 * x144;
	const FLT x146 = x116 * x122;
	const FLT x147 = x146 * (*_x0).Object.IMUBias.IMUCorrection[3];
	const FLT x148 = -1 * x131;
	const FLT x149 = x148 + x130;
	const FLT x150 = x120 + x121;
	const FLT x151 = -1 * x126;
	const FLT x152 = x125 + x151;
	const FLT x153 = (-1 * x101) + x95;
	const FLT x154 = (x133 * x153) + (x123 * x152) + (x119 * x149) + (x128 * x150);
	const FLT x155 = x144 * x154;
	const FLT x156 = x135 * x154;
	const FLT x157 = x107 * x156;
	const FLT x158 = x138 * x154;
	const FLT x159 = x116 * x152;
	const FLT x160 = x116 * x153;
	const FLT x161 = x116 * x150;
	const FLT x162 = x111 * x156;
	const FLT x163 = x129 + x148;
	const FLT x164 = x124 + x151;
	const FLT x165 = ((x164 * x133) + (x119 * x122) + (x102 * x123) + (x128 * x163)) * x135;
	const FLT x166 = x114 * x165;
	const FLT x167 = x110 * x165;
	const FLT x168 = x116 * (*_x0).Object.IMUBias.IMUCorrection[2];
	const FLT x169 = x116 * x163;
	const FLT x170 = x107 * x165;
	const FLT x171 = x111 * x165;
	const FLT x172 = -1 * x146 * (*_x0).Object.IMUBias.IMUCorrection[1];
	const FLT x173 = x116 * (*_x0).Object.IMUBias.IMUCorrection[3];
	const FLT x174 = x102 * x173;
	const FLT x175 = x134 * x143;
	const FLT x176 = x146 * (*_x0).Object.IMUBias.IMUCorrection[2];
	const FLT x177 = x102 * x141;
	const FLT x178 = x102 * x168;
	const FLT x179 = x146 * (*_x0).Object.IMUBias.IMUCorrection[0];
	const FLT x180 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[1];
	const FLT x181 = cos(x180);
	const FLT x182 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[0];
	const FLT x183 = cos(x182);
	const FLT x184 = 0.5 * (*error_state).Lighthouse.AxisAngleRot[2];
	const FLT x185 = cos(x184);
	const FLT x186 = x183 * x185;
	const FLT x187 = x181 * x186;
	const FLT x188 = 0.5 * x187;
	const FLT x189 = sin(x182);
	const FLT x190 = sin(x184);
	const FLT x191 = sin(x180);
	const FLT x192 = x190 * x191;
	const FLT x193 = x189 * x192;
	const FLT x194 = 0.5 * x193;
	const FLT x195 = x194 + x188;
	const FLT x196 = x183 * x192;
	const FLT x197 = x189 * x185;
	const FLT x198 = x181 * x197;
	const FLT x199 = x198 + (-1 * x196);
	const FLT x200 = 2 * x199;
	const FLT x201 = 0.5 * x198;
	const FLT x202 = -1 * x201;
	const FLT x203 = 0.5 * x196;
	const FLT x204 = x203 + x202;
	const FLT x205 = x187 + x193;
	const FLT x206 = 2 * x205;
	const FLT x207 = x181 * x190;
	const FLT x208 = x207 * x189;
	const FLT x209 = 0.5 * x208;
	const FLT x210 = x186 * x191;
	const FLT x211 = -0.5 * x210;
	const FLT x212 = x211 + (-1 * x209);
	const FLT x213 = x191 * x197;
	const FLT x214 = x207 * x183;
	const FLT x215 = x214 + (-1 * x213);
	const FLT x216 = 2 * x215;
	const FLT x217 = 0.5 * x213;
	const FLT x218 = -1 * x217;
	const FLT x219 = 0.5 * x214;
	const FLT x220 = x219 + x218;
	const FLT x221 = x210 + x208;
	const FLT x222 = 2 * x221;
	const FLT x223 = (x220 * x222) + (x212 * x216) + (x200 * x195) + (x204 * x206);
	const FLT x224 = (x205 * x205) + (x199 * x199) + (x215 * x215) + (x221 * x221);
	const FLT x225 = 1.0 / 2.0 * (1. / (x224 * sqrt(x224)));
	const FLT x226 = x225 * x199;
	const FLT x227 = x223 * x226;
	const FLT x228 = 1. / sqrt(x224);
	const FLT x229 = x228 * (*_x0).Lighthouse.Rot[1];
	const FLT x230 = -1 * x229 * x195;
	const FLT x231 = x205 * x225;
	const FLT x232 = x231 * x223;
	const FLT x233 = x228 * (*_x0).Lighthouse.Rot[3];
	const FLT x234 = x212 * x233;
	const FLT x235 = x215 * x225;
	const FLT x236 = x235 * x223;
	const FLT x237 = x228 * (*_x0).Lighthouse.Rot[0];
	const FLT x238 = x228 * (*_x0).Lighthouse.Rot[2];
	const FLT x239 = x221 * x225;
	const FLT x240 = x239 * x223;
	const FLT x241 = -1 * x219;
	const FLT x242 = x241 + x218;
	const FLT x243 = x209 + x211;
	const FLT x244 = -1 * x203;
	const FLT x245 = x202 + x244;
	const FLT x246 = (-1 * x194) + x188;
	const FLT x247 = ((x222 * x246) + (x200 * x242) + (x206 * x243) + (x216 * x245)) * x225;
	const FLT x248 = x205 * x247;
	const FLT x249 = x215 * x247;
	const FLT x250 = x221 * x247;
	const FLT x251 = x247 * x199;
	const FLT x252 = x228 * x243;
	const FLT x253 = x228 * x246;
	const FLT x254 = -1 * x212 * x229;
	const FLT x255 = x217 + x241;
	const FLT x256 = x201 + x244;
	const FLT x257 = (x256 * x222) + (x212 * x200) + (x206 * x255) + (x216 * x195);
	const FLT x258 = x231 * x257;
	const FLT x259 = x257 * (*_x0).Lighthouse.Rot[3];
	const FLT x260 = x233 * x195;
	const FLT x261 = x239 * x257;
	const FLT x262 = x257 * x226;
	const FLT x263 = x212 * x238;
	const FLT x264 = x237 * x195;
	const FLT x265 = x235 * x257;
	const FLT x266 = x238 * x195;
	const FLT x267 = x212 * x237;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[0]) / sizeof(FLT),
						x55 + (x47 * (*_x0).Object.Pose.Rot[3]) + (x53 * (*_x0).Object.Pose.Rot[2]) +
							(x46 * (*_x0).Object.Pose.Rot[1]) + (x56 * x40) + (-1 * x51 * x41) + (-1 * x50) +
							(-1 * x52 * x34));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[1]) / sizeof(FLT),
						(x69 * (*_x0).Object.Pose.Rot[2]) + (x63 * x56) + (-1 * x65 * x41) +
							(x68 * (*_x0).Object.Pose.Rot[1]) + (-1 * x67 * (*_x0).Object.Pose.Rot[3]) +
							(-1 * x59 * (*_x0).Object.Pose.Rot[1]) + (x66 * (*_x0).Object.Pose.Rot[3]) +
							(-1 * x60 * x52));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[2]) / sizeof(FLT),
						(x78 * (*_x0).Object.Pose.Rot[1]) + (-1 * x72 * x52) + (x75 * (*_x0).Object.Pose.Rot[3]) +
							(-1 * x77 * x41) + (x79 * (*_x0).Object.Pose.Rot[2]) + x71 + (-1 * x76) + (x73 * x56));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[0]) / sizeof(FLT),
						x82 + (-1 * x51 * x13) + (x53 * (*_x0).Object.Pose.Rot[3]) + (-1 * x49 * x34) + (x70 * x40) +
							(-1 * x47 * (*_x0).Object.Pose.Rot[2]) + (-1 * x80 * (*_x0).Object.Pose.Rot[1]) + x81);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[1]) / sizeof(FLT),
						(x67 * (*_x0).Object.Pose.Rot[2]) + (x59 * (*_x0).Object.Pose.Rot[0]) + (x70 * x63) +
							(-1 * x65 * x13) + (-1 * x83 * (*_x0).Object.Pose.Rot[1]) +
							(-1 * x66 * (*_x0).Object.Pose.Rot[2]) + (x69 * (*_x0).Object.Pose.Rot[3]) +
							(-1 * x60 * x49));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[2]) / sizeof(FLT),
						(-1 * x75 * (*_x0).Object.Pose.Rot[2]) + (-1 * x77 * x13) + (x79 * (*_x0).Object.Pose.Rot[3]) +
							x86 + (-1 * x84 * (*_x0).Object.Pose.Rot[1]) + (x70 * x73) + (-1 * x72 * x49) + x85);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[0]) / sizeof(FLT),
						(-1 * x51 * x35) + (x52 * x40) + (x56 * x34) + (x47 * (*_x0).Object.Pose.Rot[1]) + x71 +
							(-1 * x80 * (*_x0).Object.Pose.Rot[2]) + x76 + (-1 * x46 * (*_x0).Object.Pose.Rot[3]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[1]) / sizeof(FLT),
						(x63 * x52) + (x60 * x56) + (x59 * (*_x0).Object.Pose.Rot[3]) +
							(-1 * x83 * (*_x0).Object.Pose.Rot[2]) + (x66 * (*_x0).Object.Pose.Rot[1]) +
							(-1 * x70 * x62) + (-1 * x68 * (*_x0).Object.Pose.Rot[3]) + (-1 * x65 * x35));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[2]) / sizeof(FLT),
						(x72 * x56) + (-1 * x78 * (*_x0).Object.Pose.Rot[3]) + (-1 * x84 * (*_x0).Object.Pose.Rot[2]) +
							x55 + (x73 * x52) + (-1 * x77 * x35) + (x75 * (*_x0).Object.Pose.Rot[1]) + x50);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[0]) / sizeof(FLT),
						(-1 * x86) + x85 + (x70 * x34) + (-1 * x51 * x23) + (x40 * x49) +
							(x46 * (*_x0).Object.Pose.Rot[2]) + (-1 * x53 * (*_x0).Object.Pose.Rot[1]) +
							(-1 * x80 * (*_x0).Object.Pose.Rot[3]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[1]) / sizeof(FLT),
						(x62 * x56) + (-1 * x69 * (*_x0).Object.Pose.Rot[1]) + (x68 * (*_x0).Object.Pose.Rot[2]) +
							(-1 * x65 * x23) + (x70 * x60) + (-1 * x59 * (*_x0).Object.Pose.Rot[2]) +
							(-1 * x83 * (*_x0).Object.Pose.Rot[3]) + (x63 * x49));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[2]) / sizeof(FLT),
						(-1 * x84 * (*_x0).Object.Pose.Rot[3]) + (x73 * x49) + x81 + (x70 * x72) + (-1 * x77 * x23) +
							(x78 * (*_x0).Object.Pose.Rot[2]) + (-1 * x79 * (*_x0).Object.Pose.Rot[1]) + (-1 * x82));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Acc[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Acc[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Acc[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Acc[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.Acc[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.Acc[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.AccScale[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccScale[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.AccScale[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccScale[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.AccScale[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccScale[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						(-1 * x147) + (x145 * (*_x0).Object.IMUBias.IMUCorrection[1]) + (-1 * x142 * x143) + x118 +
							(x139 * (*_x0).Object.IMUBias.IMUCorrection[2]) +
							(x137 * (*_x0).Object.IMUBias.IMUCorrection[3]) +
							(-1 * x140 * (*_x0).Object.IMUBias.IMUCorrection[2]) + (x127 * x141));
	cnMatrixOptionalSet(
		Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
		offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
		(x161 * (*_x0).Object.IMUBias.IMUCorrection[0]) + (-1 * x117 * x149) +
			(x157 * (*_x0).Object.IMUBias.IMUCorrection[3]) + (x155 * (*_x0).Object.IMUBias.IMUCorrection[1]) +
			(-1 * x162 * (*_x0).Object.IMUBias.IMUCorrection[0]) +
			(-1 * x159 * (*_x0).Object.IMUBias.IMUCorrection[3]) + (x158 * (*_x0).Object.IMUBias.IMUCorrection[2]) +
			(-1 * x160 * (*_x0).Object.IMUBias.IMUCorrection[2]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						(-1 * x174) + x172 + (-1 * x171 * (*_x0).Object.IMUBias.IMUCorrection[0]) +
							(x167 * (*_x0).Object.IMUBias.IMUCorrection[2]) +
							(x166 * (*_x0).Object.IMUBias.IMUCorrection[1]) +
							(x169 * (*_x0).Object.IMUBias.IMUCorrection[0]) + (-1 * x168 * x164) +
							(x170 * (*_x0).Object.IMUBias.IMUCorrection[3]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						x177 + (-1 * x137 * (*_x0).Object.IMUBias.IMUCorrection[2]) + x176 + (-1 * x142 * x144) +
							(-1 * x175 * (*_x0).Object.IMUBias.IMUCorrection[1]) + (x117 * x127) +
							(x139 * (*_x0).Object.IMUBias.IMUCorrection[3]) +
							(-1 * x140 * (*_x0).Object.IMUBias.IMUCorrection[3]));
	cnMatrixOptionalSet(
		Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
		offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
		(x159 * (*_x0).Object.IMUBias.IMUCorrection[2]) + (-1 * x162 * (*_x0).Object.IMUBias.IMUCorrection[1]) +
			(x161 * (*_x0).Object.IMUBias.IMUCorrection[1]) + (x158 * (*_x0).Object.IMUBias.IMUCorrection[3]) +
			(-1 * x155 * (*_x0).Object.IMUBias.IMUCorrection[0]) + (x141 * x149) +
			(-1 * x157 * (*_x0).Object.IMUBias.IMUCorrection[2]) +
			(-1 * x160 * (*_x0).Object.IMUBias.IMUCorrection[3]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						x179 + (-1 * x164 * x173) + x178 + (-1 * x166 * (*_x0).Object.IMUBias.IMUCorrection[0]) +
							(x169 * (*_x0).Object.IMUBias.IMUCorrection[1]) +
							(x167 * (*_x0).Object.IMUBias.IMUCorrection[3]) +
							(-1 * x170 * (*_x0).Object.IMUBias.IMUCorrection[2]) +
							(-1 * x171 * (*_x0).Object.IMUBias.IMUCorrection[1]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						(x127 * x168) + (x140 * (*_x0).Object.IMUBias.IMUCorrection[0]) +
							(-1 * x175 * (*_x0).Object.IMUBias.IMUCorrection[2]) + x172 +
							(-1 * x145 * (*_x0).Object.IMUBias.IMUCorrection[3]) + (-1 * x138 * x142) +
							(x137 * (*_x0).Object.IMUBias.IMUCorrection[1]) + x174);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
						(-1 * x162 * (*_x0).Object.IMUBias.IMUCorrection[2]) +
							(x160 * (*_x0).Object.IMUBias.IMUCorrection[0]) +
							(-1 * x158 * (*_x0).Object.IMUBias.IMUCorrection[0]) + (x168 * x150) +
							(-1 * x159 * (*_x0).Object.IMUBias.IMUCorrection[1]) +
							(-1 * x155 * (*_x0).Object.IMUBias.IMUCorrection[3]) +
							(x157 * (*_x0).Object.IMUBias.IMUCorrection[1]) + (x173 * x149));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						(x164 * x141) + (-1 * x171 * (*_x0).Object.IMUBias.IMUCorrection[2]) +
							(x169 * (*_x0).Object.IMUBias.IMUCorrection[2]) +
							(-1 * x167 * (*_x0).Object.IMUBias.IMUCorrection[0]) +
							(x170 * (*_x0).Object.IMUBias.IMUCorrection[1]) + x118 +
							(-1 * x166 * (*_x0).Object.IMUBias.IMUCorrection[3]) + x147);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[0]) / sizeof(FLT),
						(x145 * (*_x0).Object.IMUBias.IMUCorrection[2]) + (x127 * x173) +
							(x140 * (*_x0).Object.IMUBias.IMUCorrection[1]) + x179 +
							(-1 * x139 * (*_x0).Object.IMUBias.IMUCorrection[1]) +
							(-1 * x175 * (*_x0).Object.IMUBias.IMUCorrection[3]) + (-1 * x178) + (-1 * x136 * x142));
	cnMatrixOptionalSet(
		Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[3]) / sizeof(FLT),
		offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[1]) / sizeof(FLT),
		(x161 * (*_x0).Object.IMUBias.IMUCorrection[3]) + (-1 * x168 * x149) +
			(x159 * (*_x0).Object.IMUBias.IMUCorrection[0]) + (x160 * (*_x0).Object.IMUBias.IMUCorrection[1]) +
			(-1 * x162 * (*_x0).Object.IMUBias.IMUCorrection[3]) +
			(-1 * x158 * (*_x0).Object.IMUBias.IMUCorrection[1]) +
			(-1 * x157 * (*_x0).Object.IMUBias.IMUCorrection[0]) + (x155 * (*_x0).Object.IMUBias.IMUCorrection[2]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.IMUCorrection[2]) / sizeof(FLT),
						(x166 * (*_x0).Object.IMUBias.IMUCorrection[2]) + x177 + (x117 * x164) + (-1 * x176) +
							(-1 * x167 * (*_x0).Object.IMUBias.IMUCorrection[1]) +
							(-1 * x171 * (*_x0).Object.IMUBias.IMUCorrection[3]) +
							(-1 * x170 * (*_x0).Object.IMUBias.IMUCorrection[0]) +
							(x169 * (*_x0).Object.IMUBias.IMUCorrection[3]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.AccBias[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.AccBias[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.AccBias[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.AccBias[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.GyroBias[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.GyroBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.GyroBias[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.GyroBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Object.IMUBias.GyroBias[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Object.IMUBias.GyroBias[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						x230 + (-1 * x232 * (*_x0).Lighthouse.Rot[0]) + (x236 * (*_x0).Lighthouse.Rot[3]) +
							(x240 * (*_x0).Lighthouse.Rot[2]) + (x227 * (*_x0).Lighthouse.Rot[1]) + (-1 * x238 * x220) +
							(x237 * x204) + (-1 * x234));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						(-1 * x253 * (*_x0).Lighthouse.Rot[2]) + (x252 * (*_x0).Lighthouse.Rot[0]) +
							(x251 * (*_x0).Lighthouse.Rot[1]) + (-1 * x248 * (*_x0).Lighthouse.Rot[0]) +
							(-1 * x233 * x245) + (x250 * (*_x0).Lighthouse.Rot[2]) + (-1 * x229 * x242) +
							(x249 * (*_x0).Lighthouse.Rot[3]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						(x262 * (*_x0).Lighthouse.Rot[1]) + (x261 * (*_x0).Lighthouse.Rot[2]) +
							(-1 * x258 * (*_x0).Lighthouse.Rot[0]) + (x237 * x255) + (-1 * x260) + x254 +
							(x235 * x259) + (-1 * x238 * x256));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						(-1 * x236 * (*_x0).Lighthouse.Rot[2]) + (x204 * x229) +
							(-1 * x232 * (*_x0).Lighthouse.Rot[1]) + (-1 * x227 * (*_x0).Lighthouse.Rot[0]) + x263 +
							(-1 * x233 * x220) + x264 + (x240 * (*_x0).Lighthouse.Rot[3]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						(-1 * x249 * (*_x0).Lighthouse.Rot[2]) + (x237 * x242) + (x238 * x245) +
							(-1 * x248 * (*_x0).Lighthouse.Rot[1]) + (x252 * (*_x0).Lighthouse.Rot[1]) +
							(x250 * (*_x0).Lighthouse.Rot[3]) + (-1 * x251 * (*_x0).Lighthouse.Rot[0]) +
							(-1 * x233 * x246));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						x267 + (-1 * x258 * (*_x0).Lighthouse.Rot[1]) + (x239 * x259) +
							(-1 * x262 * (*_x0).Lighthouse.Rot[0]) + (x255 * x229) +
							(-1 * x265 * (*_x0).Lighthouse.Rot[2]) + x266 + (-1 * x233 * x256));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						(-1 * x232 * (*_x0).Lighthouse.Rot[2]) + (-1 * x240 * (*_x0).Lighthouse.Rot[0]) +
							(x236 * (*_x0).Lighthouse.Rot[1]) + x254 + (x237 * x220) +
							(-1 * x227 * (*_x0).Lighthouse.Rot[3]) + (x238 * x204) + x260);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						(x253 * (*_x0).Lighthouse.Rot[0]) + (x249 * (*_x0).Lighthouse.Rot[1]) +
							(x252 * (*_x0).Lighthouse.Rot[2]) + (-1 * x251 * (*_x0).Lighthouse.Rot[3]) +
							(-1 * x229 * x245) + (-1 * x250 * (*_x0).Lighthouse.Rot[0]) +
							(-1 * x248 * (*_x0).Lighthouse.Rot[2]) + (x233 * x242));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						(-1 * x258 * (*_x0).Lighthouse.Rot[2]) + (x237 * x256) + (x238 * x255) + (-1 * x259 * x226) +
							(x265 * (*_x0).Lighthouse.Rot[1]) + x230 + (-1 * x261 * (*_x0).Lighthouse.Rot[0]) + x234);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						x267 + (x227 * (*_x0).Lighthouse.Rot[2]) + (x220 * x229) +
							(-1 * x240 * (*_x0).Lighthouse.Rot[1]) + (-1 * x232 * (*_x0).Lighthouse.Rot[3]) +
							(-1 * x266) + (-1 * x236 * (*_x0).Lighthouse.Rot[0]) + (x233 * x204));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						(x237 * x245) + (-1 * x238 * x242) + (-1 * x248 * (*_x0).Lighthouse.Rot[3]) +
							(-1 * x250 * (*_x0).Lighthouse.Rot[1]) + (-1 * x249 * (*_x0).Lighthouse.Rot[0]) +
							(x253 * (*_x0).Lighthouse.Rot[1]) + (x251 * (*_x0).Lighthouse.Rot[2]) + (x233 * x243));
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						(-1 * x263) + (-1 * x231 * x259) + (x233 * x255) + (-1 * x261 * (*_x0).Lighthouse.Rot[1]) +
							(x262 * (*_x0).Lighthouse.Rot[2]) + (x256 * x229) + (-1 * x265 * (*_x0).Lighthouse.Rot[0]) +
							x264);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD0.phase) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, BSD0.phase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD0.tilt) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, BSD0.tilt) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD0.curve) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, BSD0.curve) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD0.gibpha) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, BSD0.gibpha) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD0.gibmag) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, BSD0.gibmag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD0.ogeephase) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, BSD0.ogeephase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD0.ogeemag) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, BSD0.ogeemag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD1.phase) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, BSD1.phase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD1.tilt) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, BSD1.tilt) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD1.curve) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, BSD1.curve) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD1.gibpha) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, BSD1.gibpha) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD1.gibmag) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, BSD1.gibmag) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD1.ogeephase) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, BSD1.ogeephase) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveJointKalmanModel, BSD1.ogeemag) / sizeof(FLT),
						offsetof(SurviveJointKalmanErrorModel, BSD1.ogeemag) / sizeof(FLT), 1);
}

// Full version Jacobian of SurviveJointKalmanModelAddErrorModel wrt [(*error_state).Lighthouse.AxisAngleRot[0],
// (*error_state).Lighthouse.AxisAngleRot[1], (*error_state).Lighthouse.AxisAngleRot[2],
// (*error_state).Lighthouse.Pos[0], (*error_state).Lighthouse.Pos[1], (*error_state).Lighthouse.Pos[2],
// (*error_state).Object.Acc[0], (*error_state).Object.Acc[1], (*error_state).Object.Acc[2],
// (*error_state).Object.IMUBias.AccBias[0], (*error_state).Object.IMUBias.AccBias[1],
// (*error_state).Object.IMUBias.AccBias[2], (*error_state).Object.IMUBias.AccScale[0],
// (*error_state).Object.IMUBias.AccScale[1], (*error_state).Object.IMUBias.AccScale[2],
// (*error_state).Object.IMUBias.GyroBias[0], (*error_state).Object.IMUBias.GyroBias[1],
// (*error_state).Object.IMUBias.GyroBias[2], (*error_state).Object.IMUBias.IMUCorrection[0],
// (*error_state).Object.IMUBias.IMUCorrection[1], (*error_state).Object.IMUBias.IMUCorrection[2],
// (*error_state).Object.Pose.AxisAngleRot[0], (*error_state).Object.Pose.AxisAngleRot[1],
// (*error_state).Object.Pose.AxisAngleRot[2], (*error_state).Object.Pose.Pos[0], (*error_state).Object.Pose.Pos[1],
// (*error_state).Object.Pose.Pos[2], (*error_state).Object.Velocity.AxisAngleRot[0],
// (*error_state).Object.Velocity.AxisAngleRot[1], (*error_state).Object.Velocity.AxisAngleRot[2],
// (*error_state).Object.Velocity.Pos[0], (*error_state).Object.Velocity.Pos[1], (*error_state).Object.Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f4d1c35f430>, <cnkalman.codegen.WrapMember object at 0x7f4d1c377a90>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c35f1f0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c377f40>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c35f2b0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c377fa0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c35f220>, <cnkalman.codegen.WrapMember object at 0x7f4d1c377d90>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c35f250>, <cnkalman.codegen.WrapMember object at 0x7f4d1c377e50>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c35f880>, <cnkalman.codegen.WrapMember object at 0x7f4d1c35f6d0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c35f730>, <cnkalman.codegen.WrapMember object at 0x7f4d1c377220>]
static inline FLT SurviveJointKalmanErrorModel_LightMeas_x_gen1(const FLT dt, const SurviveJointKalmanModel *_x0,
																const SurviveJointKalmanErrorModel *error_model,
																const FLT *sensor_pt) {
	const FLT x0 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[0];
	const FLT x1 = sin(x0);
	const FLT x2 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[2];
	const FLT x3 = cos(x2);
	const FLT x4 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[1];
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
	const FLT x18 = x17 * (*_x0).Lighthouse.Rot[2];
	const FLT x19 = x17 * x16;
	const FLT x20 = x13 * x17;
	const FLT x21 = x17 * (*_x0).Lighthouse.Rot[1];
	const FLT x22 =
		(x21 * x15) + (-1 * x20 * (*_x0).Lighthouse.Rot[3]) + (x11 * x18) + (x19 * (*_x0).Lighthouse.Rot[0]);
	const FLT x23 = x15 * x17;
	const FLT x24 = x11 * x17;
	const FLT x25 =
		(x23 * (*_x0).Lighthouse.Rot[3]) + (x21 * x13) + (x24 * (*_x0).Lighthouse.Rot[0]) + (-1 * x18 * x16);
	const FLT x26 =
		(-1 * x21 * x11) + (x19 * (*_x0).Lighthouse.Rot[3]) + (x20 * (*_x0).Lighthouse.Rot[0]) + (x15 * x18);
	const FLT x27 =
		(x23 * (*_x0).Lighthouse.Rot[0]) + (-1 * x21 * x16) + (-1 * x24 * (*_x0).Lighthouse.Rot[3]) + (-1 * x13 * x18);
	const FLT x28 = 1. / sqrt((x27 * x27) + (x22 * x22) + (x25 * x25) + (x26 * x26));
	const FLT x29 = (*_x0).Lighthouse.Pos[2] + (*error_model).Lighthouse.Pos[2];
	const FLT x30 = x28 * x27;
	const FLT x31 = (*_x0).Lighthouse.Pos[1] + (*error_model).Lighthouse.Pos[1];
	const FLT x32 = x22 * x28;
	const FLT x33 = (*_x0).Lighthouse.Pos[0] + (*error_model).Lighthouse.Pos[0];
	const FLT x34 = x28 * x26;
	const FLT x35 = x28 * ((x34 * x33) + (x30 * x29) + (-1 * x32 * x31));
	const FLT x36 = x25 * x28;
	const FLT x37 = (x31 * x36) + (-1 * x34 * x29) + (x30 * x33);
	const FLT x38 = (*_x0).Object.Velocity.AxisAngleRot[2] + (*error_model).Object.Velocity.AxisAngleRot[2];
	const FLT x39 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[0];
	const FLT x40 = sin(x39);
	const FLT x41 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[2];
	const FLT x42 = sin(x41);
	const FLT x43 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[1];
	const FLT x44 = cos(x43);
	const FLT x45 = x42 * x44;
	const FLT x46 = cos(x39);
	const FLT x47 = sin(x43);
	const FLT x48 = cos(x41);
	const FLT x49 = x47 * x48;
	const FLT x50 = (x46 * x49) + (x40 * x45);
	const FLT x51 = x42 * x47;
	const FLT x52 = x44 * x48;
	const FLT x53 = (x52 * x46) + (x51 * x40);
	const FLT x54 = (x45 * x46) + (-1 * x40 * x49);
	const FLT x55 = (x52 * x40) + (-1 * x51 * x46);
	const FLT x56 = 1. / sqrt((x55 * x55) + (x54 * x54) + (x50 * x50) + (x53 * x53));
	const FLT x57 = x53 * x56;
	const FLT x58 = x56 * x55;
	const FLT x59 = x54 * x56;
	const FLT x60 = x56 * (*_x0).Object.Pose.Rot[1];
	const FLT x61 = (x60 * x50) + (x59 * (*_x0).Object.Pose.Rot[0]) + (x57 * (*_x0).Object.Pose.Rot[3]) +
					(-1 * x58 * (*_x0).Object.Pose.Rot[2]);
	const FLT x62 = dt * dt;
	const FLT x63 = x62 * (x38 * x38);
	const FLT x64 = (*_x0).Object.Velocity.AxisAngleRot[0] + (*error_model).Object.Velocity.AxisAngleRot[0];
	const FLT x65 = (x64 * x64) * x62;
	const FLT x66 = (*_x0).Object.Velocity.AxisAngleRot[1] + (*error_model).Object.Velocity.AxisAngleRot[1];
	const FLT x67 = x62 * (x66 * x66);
	const FLT x68 = 1e-10 + x67 + x63 + x65;
	const FLT x69 = sqrt(x68);
	const FLT x70 = 0.5 * x69;
	const FLT x71 = sin(x70);
	const FLT x72 = (x71 * x71) * (1. / x68);
	const FLT x73 = cos(x70);
	const FLT x74 = 1. / sqrt((x73 * x73) + (x72 * x63) + (x72 * x65) + (x72 * x67));
	const FLT x75 = dt * x71 * x74 * (1. / x69);
	const FLT x76 = x75 * x61;
	const FLT x77 = x50 * x56;
	const FLT x78 = (-1 * x60 * x54) + (x77 * (*_x0).Object.Pose.Rot[0]) + (x57 * (*_x0).Object.Pose.Rot[2]) +
					(x58 * (*_x0).Object.Pose.Rot[3]);
	const FLT x79 = x75 * x66;
	const FLT x80 = (x57 * (*_x0).Object.Pose.Rot[0]) + (-1 * x60 * x55) + (-1 * x59 * (*_x0).Object.Pose.Rot[3]) +
					(-1 * x77 * (*_x0).Object.Pose.Rot[2]);
	const FLT x81 = x73 * x74;
	const FLT x82 = (x60 * x53) + (x58 * (*_x0).Object.Pose.Rot[0]) + (-1 * x77 * (*_x0).Object.Pose.Rot[3]) +
					(x59 * (*_x0).Object.Pose.Rot[2]);
	const FLT x83 = x75 * x64;
	const FLT x84 = (x80 * x81) + (-1 * x82 * x83) + (-1 * x76 * x38) + (-1 * x79 * x78);
	const FLT x85 = x75 * x38;
	const FLT x86 = (x80 * x83) + (x81 * x82) + (-1 * x85 * x78) + (x76 * x66);
	const FLT x87 = x80 * x75;
	const FLT x88 = (x82 * x85) + (-1 * x83 * x61) + (x87 * x66) + (x81 * x78);
	const FLT x89 = (x84 * sensor_pt[2]) + (-1 * x88 * sensor_pt[0]) + (x86 * sensor_pt[1]);
	const FLT x90 = (x83 * x78) + (-1 * x82 * x79) + (x87 * x38) + (x81 * x61);
	const FLT x91 = (-1 * x90 * sensor_pt[1]) + (x84 * sensor_pt[0]) + (x88 * sensor_pt[2]);
	const FLT x92 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x93 = (*_x0).Object.Pose.Pos[1] + (2 * ((x91 * x90) + (-1 * x89 * x86))) +
					(x92 * ((*_x0).Object.Acc[1] + (*error_model).Object.Acc[1])) + (*error_model).Object.Pose.Pos[1] +
					sensor_pt[1] + (dt * ((*_x0).Object.Velocity.Pos[1] + (*error_model).Object.Velocity.Pos[1]));
	const FLT x94 = (-1 * x86 * sensor_pt[2]) + (x84 * sensor_pt[1]) + (x90 * sensor_pt[0]);
	const FLT x95 = (*error_model).Object.Pose.Pos[2] + sensor_pt[2] +
					(dt * ((*_x0).Object.Velocity.Pos[2] + (*error_model).Object.Velocity.Pos[2])) +
					(2 * ((x86 * x94) + (-1 * x88 * x91))) +
					(x92 * ((*_x0).Object.Acc[2] + (*error_model).Object.Acc[2])) + (*_x0).Object.Pose.Pos[2];
	const FLT x96 = (*_x0).Object.Pose.Pos[0] + (2 * ((x88 * x89) + (-1 * x90 * x94))) +
					(*error_model).Object.Pose.Pos[0] + (x92 * ((*_x0).Object.Acc[0] + (*error_model).Object.Acc[0])) +
					sensor_pt[0] + (dt * ((*_x0).Object.Velocity.Pos[0] + (*error_model).Object.Velocity.Pos[0]));
	const FLT x97 = (x96 * x34) + (-1 * x93 * x32) + (x95 * x30);
	const FLT x98 = (x93 * x36) + (x96 * x30) + (-1 * x95 * x34);
	const FLT x99 =
		x93 + (-1 * (x31 + (2 * ((-1 * x36 * x37) + (x35 * x22))))) + (2 * ((-1 * x98 * x36) + (x97 * x32)));
	const FLT x100 = (x32 * x29) + (x30 * x31) + (-1 * x33 * x36);
	const FLT x101 = (x95 * x32) + (x93 * x30) + (-1 * x96 * x36);
	const FLT x102 =
		x95 + (-1 * (x29 + (2 * ((-1 * x32 * x100) + (x34 * x37))))) + (2 * ((-1 * x32 * x101) + (x98 * x34)));
	const FLT x103 = -1 * x102;
	const FLT x104 =
		x96 + (2 * ((-1 * x97 * x34) + (x36 * x101))) + (-1 * (x33 + (2 * ((-1 * x35 * x26) + (x36 * x100)))));
	const FLT x105 =
		(-1 * ((*error_model).BSD0.phase + (*_x0).BSD0.phase)) +
		(-1 * asin((1. / sqrt((x104 * x104) + (x102 * x102))) * x99 * ((*error_model).BSD0.tilt + (*_x0).BSD0.tilt))) +
		(-1 * atan2(x104, x103));
	return ((atan2(x99, x103) * atan2(x99, x103)) * ((*error_model).BSD0.curve + (*_x0).BSD0.curve)) + x105 +
		   (-1 * ((*error_model).BSD0.gibmag + (*_x0).BSD0.gibmag) *
			cos(1.5707963267949 + x105 + (*error_model).BSD0.gibpha + (*_x0).BSD0.gibpha));
}

// Jacobian of SurviveJointKalmanErrorModel_LightMeas_x_gen1 wrt [(*_x0).Lighthouse.Pos[0], (*_x0).Lighthouse.Pos[1],
// (*_x0).Lighthouse.Pos[2], (*_x0).Lighthouse.Rot[0], (*_x0).Lighthouse.Rot[1], (*_x0).Lighthouse.Rot[2],
// (*_x0).Lighthouse.Rot[3], (*_x0).Object.Acc[0], (*_x0).Object.Acc[1], (*_x0).Object.Acc[2],
// (*_x0).Object.IMUBias.AccBias[0], (*_x0).Object.IMUBias.AccBias[1], (*_x0).Object.IMUBias.AccBias[2],
// (*_x0).Object.IMUBias.AccScale[0], (*_x0).Object.IMUBias.AccScale[1], (*_x0).Object.IMUBias.AccScale[2],
// (*_x0).Object.IMUBias.GyroBias[0], (*_x0).Object.IMUBias.GyroBias[1], (*_x0).Object.IMUBias.GyroBias[2],
// (*_x0).Object.IMUBias.IMUCorrection[0], (*_x0).Object.IMUBias.IMUCorrection[1],
// (*_x0).Object.IMUBias.IMUCorrection[2], (*_x0).Object.IMUBias.IMUCorrection[3], (*_x0).Object.Pose.Pos[0],
// (*_x0).Object.Pose.Pos[1], (*_x0).Object.Pose.Pos[2], (*_x0).Object.Pose.Rot[0], (*_x0).Object.Pose.Rot[1],
// (*_x0).Object.Pose.Rot[2], (*_x0).Object.Pose.Rot[3], (*_x0).Object.Velocity.AxisAngleRot[0],
// (*_x0).Object.Velocity.AxisAngleRot[1], (*_x0).Object.Velocity.AxisAngleRot[2], (*_x0).Object.Velocity.Pos[0],
// (*_x0).Object.Velocity.Pos[1], (*_x0).Object.Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f4d1c32f2e0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32f5b0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32f3a0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32f670>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32f340>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32f610>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32f460>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32f730>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32f400>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32f6d0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c398dc0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32f4f0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32f280>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32f550>]
static inline void SurviveJointKalmanErrorModel_LightMeas_x_gen1_jac_x0(CnMat *Hx, const FLT dt,
																		const SurviveJointKalmanModel *_x0,
																		const SurviveJointKalmanErrorModel *error_model,
																		const FLT *sensor_pt) {
	const FLT x0 = (*_x0).Lighthouse.Pos[2] + (*error_model).Lighthouse.Pos[2];
	const FLT x1 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[2];
	const FLT x2 = cos(x1);
	const FLT x3 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[1];
	const FLT x4 = sin(x3);
	const FLT x5 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[0];
	const FLT x6 = sin(x5);
	const FLT x7 = x4 * x6;
	const FLT x8 = sin(x1);
	const FLT x9 = cos(x5);
	const FLT x10 = cos(x3);
	const FLT x11 = x9 * x10;
	const FLT x12 = (x8 * x11) + (-1 * x2 * x7);
	const FLT x13 = x6 * x10;
	const FLT x14 = x4 * x9;
	const FLT x15 = (x2 * x14) + (x8 * x13);
	const FLT x16 = (x2 * x11) + (x8 * x7);
	const FLT x17 = (x2 * x13) + (-1 * x8 * x14);
	const FLT x18 = 1. / sqrt((x16 * x16) + (x17 * x17) + (x12 * x12) + (x15 * x15));
	const FLT x19 = x18 * x16;
	const FLT x20 = x18 * x17;
	const FLT x21 = x12 * x18;
	const FLT x22 = x15 * x18;
	const FLT x23 = (x22 * (*_x0).Lighthouse.Rot[1]) + (x21 * (*_x0).Lighthouse.Rot[0]) +
					(x19 * (*_x0).Lighthouse.Rot[3]) + (-1 * x20 * (*_x0).Lighthouse.Rot[2]);
	const FLT x24 = x23 * x23;
	const FLT x25 = (x22 * (*_x0).Lighthouse.Rot[0]) + (x20 * (*_x0).Lighthouse.Rot[3]) +
					(-1 * x21 * (*_x0).Lighthouse.Rot[1]) + (x19 * (*_x0).Lighthouse.Rot[2]);
	const FLT x26 = x25 * x25;
	const FLT x27 = (x19 * (*_x0).Lighthouse.Rot[0]) + (-1 * x21 * (*_x0).Lighthouse.Rot[3]) +
					(-1 * x20 * (*_x0).Lighthouse.Rot[1]) + (-1 * x22 * (*_x0).Lighthouse.Rot[2]);
	const FLT x28 = (-1 * x22 * (*_x0).Lighthouse.Rot[3]) + (x19 * (*_x0).Lighthouse.Rot[1]) +
					(x21 * (*_x0).Lighthouse.Rot[2]) + (x20 * (*_x0).Lighthouse.Rot[0]);
	const FLT x29 = x28 * x28;
	const FLT x30 = x29 + x24 + (x27 * x27) + x26;
	const FLT x31 = 1. / sqrt(x30);
	const FLT x32 = x31 * x25;
	const FLT x33 = (*_x0).Lighthouse.Pos[0] + (*error_model).Lighthouse.Pos[0];
	const FLT x34 = x31 * x27;
	const FLT x35 = (*_x0).Lighthouse.Pos[1] + (*error_model).Lighthouse.Pos[1];
	const FLT x36 = x31 * x23;
	const FLT x37 = (x36 * x35) + (-1 * x0 * x32) + (x34 * x33);
	const FLT x38 = x31 * x28;
	const FLT x39 = (x0 * x38) + (x34 * x35) + (-1 * x33 * x36);
	const FLT x40 = (*_x0).Object.Velocity.AxisAngleRot[2] + (*error_model).Object.Velocity.AxisAngleRot[2];
	const FLT x41 = dt * dt;
	const FLT x42 = x40 * x40;
	const FLT x43 = x41 * x42;
	const FLT x44 = (*_x0).Object.Velocity.AxisAngleRot[0] + (*error_model).Object.Velocity.AxisAngleRot[0];
	const FLT x45 = x44 * x44;
	const FLT x46 = x41 * x45;
	const FLT x47 = (*_x0).Object.Velocity.AxisAngleRot[1] + (*error_model).Object.Velocity.AxisAngleRot[1];
	const FLT x48 = x47 * x47;
	const FLT x49 = x41 * x48;
	const FLT x50 = 1e-10 + x49 + x43 + x46;
	const FLT x51 = sqrt(x50);
	const FLT x52 = 0.5 * x51;
	const FLT x53 = sin(x52);
	const FLT x54 = x53 * x53;
	const FLT x55 = 1. / x50;
	const FLT x56 = x54 * x55;
	const FLT x57 = cos(x52);
	const FLT x58 = (x56 * x46) + (x57 * x57) + (x56 * x43) + (x56 * x49);
	const FLT x59 = 1. / sqrt(x58);
	const FLT x60 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[0];
	const FLT x61 = sin(x60);
	const FLT x62 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[1];
	const FLT x63 = cos(x62);
	const FLT x64 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[2];
	const FLT x65 = sin(x64);
	const FLT x66 = x63 * x65;
	const FLT x67 = cos(x64);
	const FLT x68 = cos(x60);
	const FLT x69 = sin(x62);
	const FLT x70 = x68 * x69;
	const FLT x71 = (x70 * x67) + (x61 * x66);
	const FLT x72 = (x63 * x67 * x68) + (x61 * x65 * x69);
	const FLT x73 = x61 * x67;
	const FLT x74 = (x68 * x66) + (-1 * x73 * x69);
	const FLT x75 = (x73 * x63) + (-1 * x70 * x65);
	const FLT x76 = 1. / sqrt((x75 * x75) + (x74 * x74) + (x71 * x71) + (x72 * x72));
	const FLT x77 = x72 * x76;
	const FLT x78 = x75 * x76;
	const FLT x79 = x74 * x76;
	const FLT x80 = x71 * x76;
	const FLT x81 = (x80 * (*_x0).Object.Pose.Rot[1]) + (x79 * (*_x0).Object.Pose.Rot[0]) +
					(x77 * (*_x0).Object.Pose.Rot[3]) + (-1 * x78 * (*_x0).Object.Pose.Rot[2]);
	const FLT x82 = x81 * x59;
	const FLT x83 = x53 * (1. / x51);
	const FLT x84 = dt * x83;
	const FLT x85 = x82 * x84;
	const FLT x86 = (-1 * x79 * (*_x0).Object.Pose.Rot[1]) + (x80 * (*_x0).Object.Pose.Rot[0]) +
					(x77 * (*_x0).Object.Pose.Rot[2]) + (x78 * (*_x0).Object.Pose.Rot[3]);
	const FLT x87 = x84 * x59;
	const FLT x88 = x86 * x87;
	const FLT x89 = (-1 * x78 * (*_x0).Object.Pose.Rot[1]) + (x77 * (*_x0).Object.Pose.Rot[0]) +
					(-1 * x79 * (*_x0).Object.Pose.Rot[3]) + (-1 * x80 * (*_x0).Object.Pose.Rot[2]);
	const FLT x90 = x57 * x59;
	const FLT x91 = x89 * x90;
	const FLT x92 = (x77 * (*_x0).Object.Pose.Rot[1]) + (x78 * (*_x0).Object.Pose.Rot[0]) +
					(-1 * x80 * (*_x0).Object.Pose.Rot[3]) + (x79 * (*_x0).Object.Pose.Rot[2]);
	const FLT x93 = x87 * x92;
	const FLT x94 = (-1 * x93 * x44) + (-1 * x85 * x40) + x91 + (-1 * x88 * x47);
	const FLT x95 = x89 * x87;
	const FLT x96 = x81 * x90;
	const FLT x97 = (x88 * x44) + (-1 * x93 * x47) + (x95 * x40) + x96;
	const FLT x98 = x92 * x90;
	const FLT x99 = (x95 * x44) + x98 + (-1 * x88 * x40) + (x85 * x47);
	const FLT x100 = (-1 * x99 * sensor_pt[2]) + (x94 * sensor_pt[1]) + (x97 * sensor_pt[0]);
	const FLT x101 = x86 * x90;
	const FLT x102 = (-1 * x85 * x44) + (x93 * x40) + (x95 * x47) + x101;
	const FLT x103 = (-1 * x102 * sensor_pt[0]) + (x94 * sensor_pt[2]) + (x99 * sensor_pt[1]);
	const FLT x104 = dt * fabs(dt);
	const FLT x105 = 1.0 / 2.0 * x104;
	const FLT x106 = (*_x0).Object.Pose.Pos[0] + (2 * ((x103 * x102) + (-1 * x97 * x100))) +
					 (*error_model).Object.Pose.Pos[0] +
					 (x105 * ((*_x0).Object.Acc[0] + (*error_model).Object.Acc[0])) + sensor_pt[0] +
					 (dt * ((*_x0).Object.Velocity.Pos[0] + (*error_model).Object.Velocity.Pos[0]));
	const FLT x107 = (-1 * x97 * sensor_pt[1]) + (x94 * sensor_pt[0]) + (x102 * sensor_pt[2]);
	const FLT x108 = (dt * ((*_x0).Object.Velocity.Pos[2] + (*error_model).Object.Velocity.Pos[2])) + sensor_pt[2] +
					 (*error_model).Object.Pose.Pos[2] + (2 * ((x99 * x100) + (-1 * x102 * x107))) +
					 (x105 * ((*_x0).Object.Acc[2] + (*error_model).Object.Acc[2])) + (*_x0).Object.Pose.Pos[2];
	const FLT x109 = (*_x0).Object.Pose.Pos[1] + (2 * ((x97 * x107) + (-1 * x99 * x103))) +
					 (x105 * ((*_x0).Object.Acc[1] + (*error_model).Object.Acc[1])) +
					 (*error_model).Object.Pose.Pos[1] + sensor_pt[1] +
					 (dt * ((*_x0).Object.Velocity.Pos[1] + (*error_model).Object.Velocity.Pos[1]));
	const FLT x110 = (x36 * x109) + (x34 * x106) + (-1 * x32 * x108);
	const FLT x111 = (x38 * x108) + (x34 * x109) + (-1 * x36 * x106);
	const FLT x112 =
		x108 + (-1 * (x0 + (2 * ((-1 * x38 * x39) + (x32 * x37))))) + (2 * ((-1 * x38 * x111) + (x32 * x110)));
	const FLT x113 = 1. / x112;
	const FLT x114 = 1. / x30;
	const FLT x115 = 2 * x114;
	const FLT x116 = x23 * x115;
	const FLT x117 = x27 * x116;
	const FLT x118 = x25 * x115;
	const FLT x119 = x28 * x118;
	const FLT x120 = -1 * x119;
	const FLT x121 = x120 + x117;
	const FLT x122 = x28 * x116;
	const FLT x123 = -1 * x122;
	const FLT x124 = x27 * x118;
	const FLT x125 = -1 * x124;
	const FLT x126 = x125 + x123;
	const FLT x127 = x112 * x112;
	const FLT x128 = 1. / x127;
	const FLT x129 = (x32 * x33) + (x0 * x34) + (-1 * x35 * x38);
	const FLT x130 = (-1 * x38 * x109) + (x32 * x106) + (x34 * x108);
	const FLT x131 =
		x109 + (-1 * (x35 + (2 * ((-1 * x36 * x37) + (x38 * x129))))) + (2 * ((-1 * x36 * x110) + (x38 * x130)));
	const FLT x132 = x128 * x131;
	const FLT x133 = x131 * x131;
	const FLT x134 = -1 * x112;
	const FLT x135 = atan2(x131, x134);
	const FLT x136 = 2 * (1. / (x127 + x133)) * x127 * x135 * ((*error_model).BSD0.curve + (*_x0).BSD0.curve);
	const FLT x137 =
		x106 + (2 * ((-1 * x32 * x130) + (x36 * x111))) + (-1 * (x33 + (2 * ((-1 * x32 * x129) + (x36 * x39)))));
	const FLT x138 = (x137 * x137) + x127;
	const FLT x139 = 1. / x138;
	const FLT x140 = (*error_model).BSD0.tilt + (*_x0).BSD0.tilt;
	const FLT x141 = 1. / sqrt(1 + (-1 * x133 * x139 * (x140 * x140)));
	const FLT x142 = x24 * x115;
	const FLT x143 = x26 * x115;
	const FLT x144 = -1 + x143;
	const FLT x145 = x144 + x142;
	const FLT x146 = 2 * x137;
	const FLT x147 = 2 * x112;
	const FLT x148 = 1.0 / 2.0 * x131 * (1. / (x138 * sqrt(x138))) * x140;
	const FLT x149 = 1. / sqrt(x138);
	const FLT x150 = x140 * x149;
	const FLT x151 = x128 * x137;
	const FLT x152 = x127 * x139;
	const FLT x153 = (-1 * ((-1 * x113 * x145) + (x126 * x151)) * x152) +
					 (-1 * x141 * ((x121 * x150) + (-1 * ((x126 * x147) + (x146 * x145)) * x148)));
	const FLT x154 = 1.5707963267949 + (*_x0).BSD0.gibpha + (*error_model).BSD0.gibpha + (-1 * asin(x131 * x150)) +
					 (-1 * ((*error_model).BSD0.phase + (*_x0).BSD0.phase)) + (-1 * atan2(x137, x134));
	const FLT x155 = sin(x154) * ((*error_model).BSD0.gibmag + (*_x0).BSD0.gibmag);
	const FLT x156 = -1 * x117;
	const FLT x157 = x156 + x120;
	const FLT x158 = x28 * x27;
	const FLT x159 = x115 * x158;
	const FLT x160 = x25 * x116;
	const FLT x161 = -1 * x160;
	const FLT x162 = x161 + x159;
	const FLT x163 = x29 * x115;
	const FLT x164 = -1 + x163 + x142;
	const FLT x165 = (-1 * ((-1 * x113 * x157) + (x162 * x151)) * x152) +
					 (-1 * x141 * ((x164 * x150) + (-1 * ((x162 * x147) + (x146 * x157)) * x148)));
	const FLT x166 = x123 + x124;
	const FLT x167 = x144 + x163;
	const FLT x168 = -1 * x159;
	const FLT x169 = x168 + x161;
	const FLT x170 = (-1 * ((-1 * x113 * x166) + (x167 * x151)) * x152) +
					 (-1 * x141 * ((x169 * x150) + (-1 * ((x167 * x147) + (x166 * x146)) * x148)));
	const FLT x171 = 2 * x20;
	const FLT x172 = 2 * x21;
	const FLT x173 = 2 * x19;
	const FLT x174 = 2 * x22;
	const FLT x175 = (x25 * x174) + (x27 * x173) + (x28 * x171) + (x23 * x172);
	const FLT x176 = 1.0 / 2.0 * x175;
	const FLT x177 = 1. / (x30 * sqrt(x30));
	const FLT x178 = x25 * x177;
	const FLT x179 = x33 * x178;
	const FLT x180 = x27 * x177;
	const FLT x181 = x0 * x176;
	const FLT x182 = x28 * x177;
	const FLT x183 = x35 * x176;
	const FLT x184 = x31 * x33;
	const FLT x185 = x22 * x184;
	const FLT x186 = x31 * x35;
	const FLT x187 = -1 * x20 * x186;
	const FLT x188 = x0 * x31;
	const FLT x189 = x19 * x188;
	const FLT x190 = x189 + x187;
	const FLT x191 = x190 + x185 + (x183 * x182) + (-1 * x179 * x176) + (-1 * x181 * x180);
	const FLT x192 = 2 * x32;
	const FLT x193 = x108 * x176;
	const FLT x194 = x109 * x180;
	const FLT x195 = x23 * x177;
	const FLT x196 = 1.0 / 2.0 * x106;
	const FLT x197 = x196 * x195;
	const FLT x198 = x31 * x108;
	const FLT x199 = x20 * x198;
	const FLT x200 = x31 * x106;
	const FLT x201 = -1 * x21 * x200;
	const FLT x202 = x31 * x109;
	const FLT x203 = x19 * x202;
	const FLT x204 = x203 + x201;
	const FLT x205 = x204 + x199 + (x175 * x197) + (-1 * x182 * x193) + (-1 * x176 * x194);
	const FLT x206 = 2 * x36;
	const FLT x207 = x175 * x130;
	const FLT x208 = x178 * x196;
	const FLT x209 = x19 * x198;
	const FLT x210 = -1 * x20 * x202;
	const FLT x211 = x22 * x200;
	const FLT x212 = x211 + (x109 * x176 * x182) + (-1 * x208 * x175) + x210 + x209 + (-1 * x180 * x193);
	const FLT x213 = x111 * x175;
	const FLT x214 = x31 * x111;
	const FLT x215 = x214 * x172;
	const FLT x216 = x129 * x175;
	const FLT x217 = x39 * x175;
	const FLT x218 = x31 * x39;
	const FLT x219 = x218 * x172;
	const FLT x220 = x176 * x195;
	const FLT x221 = x19 * x186;
	const FLT x222 = x20 * x188;
	const FLT x223 = -1 * x21 * x184;
	const FLT x224 = x223 + (-1 * x180 * x183) + (-1 * x181 * x182) + (x33 * x220) + x221 + x222;
	const FLT x225 = x31 * x174;
	const FLT x226 = (x225 * x129) + (-1 * x225 * x130);
	const FLT x227 = (-1 * x219) + (-1 * x216 * x178) + (x205 * x206) + (-1 * x206 * x224) + x215 + (x191 * x192) +
					 (x207 * x178) + x226 + (-1 * x213 * x195) + (x217 * x195) + (-1 * x212 * x192);
	const FLT x228 = x21 * x186;
	const FLT x229 = -1 * x22 * x188;
	const FLT x230 = x19 * x184;
	const FLT x231 = x33 * x180;
	const FLT x232 = (-1 * x231 * x176) + (x178 * x181) + x230 + (-1 * x35 * x220) + x228 + x229;
	const FLT x233 = x37 * x225;
	const FLT x234 = x37 * x175;
	const FLT x235 = x19 * x200;
	const FLT x236 = x180 * x196;
	const FLT x237 = -1 * x22 * x198;
	const FLT x238 = x21 * x202;
	const FLT x239 = x238 + (-1 * x220 * x109) + x235 + (x178 * x193) + (-1 * x236 * x175) + x237;
	const FLT x240 = 2 * x38;
	const FLT x241 = x31 * x110;
	const FLT x242 = x241 * x174;
	const FLT x243 = x110 * x178;
	const FLT x244 = x31 * x171;
	const FLT x245 = (-1 * x244 * x111) + (x39 * x244);
	const FLT x246 = (-1 * x205 * x240) + x245 + (-1 * x243 * x175) + x242 + (-1 * x217 * x182) + (x224 * x240) +
					 (-1 * x232 * x192) + (x239 * x192) + (-1 * x233) + (x213 * x182) + (x234 * x178);
	const FLT x247 = x110 * x195;
	const FLT x248 = x244 * x129;
	const FLT x249 = x244 * x130;
	const FLT x250 = x31 * x37;
	const FLT x251 = (-1 * x241 * x172) + (x250 * x172);
	const FLT x252 = x251 + (-1 * x240 * x191) + (x232 * x206) + (-1 * x207 * x182) + (x247 * x175) +
					 (-1 * x239 * x206) + (-1 * x234 * x195) + (-1 * x248) + x249 + (x212 * x240) + (x216 * x182);
	const FLT x253 = (-1 * ((-1 * x227 * x113) + (x246 * x151)) * x152) +
					 (-1 * x141 * ((x252 * x150) + (-1 * ((x246 * x147) + (x227 * x146)) * x148)));
	const FLT x254 = (x28 * x173) + (-1 * x25 * x172) + (x23 * x174) + (-1 * x27 * x171);
	const FLT x255 = x254 * x182;
	const FLT x256 = 1.0 / 2.0 * x108;
	const FLT x257 = 1.0 / 2.0 * x254;
	const FLT x258 = (-1 * x211) + x210;
	const FLT x259 = x258 + (x254 * x197) + (-1 * x257 * x194) + x209 + (-1 * x256 * x255);
	const FLT x260 = x254 * x130;
	const FLT x261 = x254 * x111;
	const FLT x262 = x257 * x180;
	const FLT x263 = 1.0 / 2.0 * x0;
	const FLT x264 = -1 * x185;
	const FLT x265 = x257 * x195;
	const FLT x266 = x190 + (x33 * x265) + x264 + (-1 * x35 * x262) + (-1 * x263 * x255);
	const FLT x267 = 1.0 / 2.0 * x109;
	const FLT x268 = -1 * x199;
	const FLT x269 = x201 + (-1 * x208 * x254) + (-1 * x203) + (x267 * x255) + x268 + (-1 * x262 * x108);
	const FLT x270 = x129 * x178;
	const FLT x271 = x223 + (-1 * x222);
	const FLT x272 = x271 + (-1 * x221) + (-1 * x257 * x179) + (1.0 / 2.0 * x35 * x255) + (-1 * x0 * x262);
	const FLT x273 = x39 * x195;
	const FLT x274 = x31 * x130;
	const FLT x275 = x31 * x129;
	const FLT x276 = (-1 * x275 * x172) + (x274 * x172);
	const FLT x277 = (x225 * x111) + (-1 * x39 * x225);
	const FLT x278 = x277 + x276 + (x273 * x254) + (x272 * x192) + (x260 * x178) + (x206 * x259) + (-1 * x269 * x192) +
					 (-1 * x270 * x254) + (-1 * x206 * x266) + (-1 * x261 * x195);
	const FLT x279 = x257 * x178;
	const FLT x280 = x21 * x188;
	const FLT x281 = x20 * x184;
	const FLT x282 = x22 * x186;
	const FLT x283 = (x0 * x279) + (-1 * x35 * x265) + (-1 * x33 * x262) + x282 + x280 + (-1 * x281);
	const FLT x284 = x218 * x173;
	const FLT x285 = x37 * x254;
	const FLT x286 = x214 * x173;
	const FLT x287 = x22 * x202;
	const FLT x288 = x21 * x198;
	const FLT x289 = x20 * x200;
	const FLT x290 = (x279 * x108) + (-1 * x236 * x254) + (-1 * x265 * x109) + (-1 * x289) + x287 + x288;
	const FLT x291 = x251 + (x290 * x192) + (-1 * x286) + (x285 * x178) + (x266 * x240) + (-1 * x39 * x255) +
					 (-1 * x283 * x192) + x284 + (-1 * x254 * x243) + (-1 * x259 * x240) + (x261 * x182);
	const FLT x292 = x275 * x173;
	const FLT x293 = x274 * x173;
	const FLT x294 = x293 + (-1 * x285 * x195) + x233 + (-1 * x260 * x182) + (x254 * x247) + (-1 * x292) +
					 (-1 * x272 * x240) + (-1 * x242) + (x206 * x283) + (x255 * x129) + (x269 * x240) +
					 (-1 * x206 * x290);
	const FLT x295 = (-1 * ((-1 * x278 * x113) + (x291 * x151)) * x152) +
					 (-1 * x141 * ((x294 * x150) + (-1 * ((x291 * x147) + (x278 * x146)) * x148)));
	const FLT x296 = (x25 * x173) + (x28 * x172) + (-1 * x23 * x171) + (-1 * x27 * x174);
	const FLT x297 = x296 * x178;
	const FLT x298 = x296 * x182;
	const FLT x299 = 1.0 / 2.0 * x298;
	const FLT x300 = 1.0 / 2.0 * x296;
	const FLT x301 = x300 * x180;
	const FLT x302 = x237 + (-1 * x238);
	const FLT x303 = (-1 * x301 * x108) + x302 + (x299 * x109) + (-1 * x297 * x196) + x235;
	const FLT x304 = x296 * x195;
	const FLT x305 = (-1 * x300 * x194) + (-1 * x287) + (-1 * x298 * x256) + x289 + (x304 * x196) + x288;
	const FLT x306 = 1.0 / 2.0 * x304;
	const FLT x307 = (-1 * x35 * x301) + x281 + (-1 * x298 * x263) + x280 + (-1 * x282) + (x33 * x306);
	const FLT x308 = x39 * x296;
	const FLT x309 = x229 + (-1 * x228);
	const FLT x310 = x309 + x230 + (x35 * x299) + (-1.0 / 2.0 * x33 * x297) + (-1 * x0 * x301);
	const FLT x311 = (-1 * x297 * x129) + x245 + x292 + (x308 * x195) + (-1 * x293) + (x297 * x130) + (x310 * x192) +
					 (-1 * x303 * x192) + (x206 * x305) + (-1 * x206 * x307) + (-1 * x304 * x111);
	const FLT x312 = x187 + (x297 * x263) + x264 + (-1 * x35 * x306) + (-1 * x231 * x300) + (-1 * x189);
	const FLT x313 = x258 + (-1 * x236 * x296) + (-1 * x267 * x304) + (x297 * x256) + (-1 * x209);
	const FLT x314 = x250 * x173;
	const FLT x315 = x241 * x173;
	const FLT x316 = (x298 * x111) + (x37 * x297) + (x313 * x192) + (-1 * x240 * x305) + x315 + (-1 * x297 * x110) +
					 (-1 * x308 * x182) + (-1 * x314) + (-1 * x312 * x192) + x219 + (x240 * x307) + (-1 * x215);
	const FLT x317 = (-1 * x37 * x244) + (x241 * x171);
	const FLT x318 = (x240 * x303) + x317 + (x206 * x312) + (-1 * x298 * x130) + (x298 * x129) + (-1 * x240 * x310) +
					 x276 + (-1 * x206 * x313) + (-1 * x37 * x304) + (x296 * x247);
	const FLT x319 = (-1 * ((-1 * x311 * x113) + (x316 * x151)) * x152) +
					 (-1 * x141 * ((x318 * x150) + (-1 * ((x316 * x147) + (x311 * x146)) * x148)));
	const FLT x320 = (x25 * x171) + (x23 * x173) + (-1 * x28 * x174) + (-1 * x27 * x172);
	const FLT x321 = x320 * x182;
	const FLT x322 = 1.0 / 2.0 * x320;
	const FLT x323 = x320 * x195;
	const FLT x324 = x302 + (x323 * x196) + (-1 * x235) + (-1 * x256 * x321) + (-1 * x322 * x194);
	const FLT x325 = 1.0 / 2.0 * x321;
	const FLT x326 = x322 * x180;
	const FLT x327 = 1.0 / 2.0 * x323;
	const FLT x328 = x309 + (-1 * x230) + (x33 * x327) + (-1 * x0 * x325) + (-1 * x35 * x326);
	const FLT x329 = x282 + (-1 * x0 * x326) + (-1 * x280) + (x35 * x325) + (-1 * x322 * x179) + x281;
	const FLT x330 = x320 * x196;
	const FLT x331 = (-1 * x256 * x320 * x180) + (x325 * x109) + (-1 * x330 * x178) + x289 + x287 + (-1 * x288);
	const FLT x332 = x320 * x130;
	const FLT x333 = x248 + (-1 * x331 * x192) + (-1 * x284) + (x206 * x324) + x286 + (-1 * x323 * x111) +
					 (x329 * x192) + (-1 * x270 * x320) + (-1 * x206 * x328) + (x332 * x178) + (x273 * x320) +
					 (-1 * x249);
	const FLT x334 = x37 * x320;
	const FLT x335 = x320 * x178;
	const FLT x336 = x271 + (x263 * x335) + (-1 * x231 * x322) + x221 + (-1 * x35 * x327);
	const FLT x337 = (x256 * x335) + x204 + (-1 * x327 * x109) + x268 + (-1 * x330 * x180);
	const FLT x338 = x320 * x110;
	const FLT x339 = x277 + (-1 * x338 * x178) + (x321 * x111) + (x337 * x192) + (-1 * x336 * x192) + (x334 * x178) +
					 x317 + (-1 * x39 * x321) + (-1 * x240 * x324) + (x240 * x328);
	const FLT x340 = (-1 * x240 * x329) + (x240 * x331) + (-1 * x206 * x337) + (-1 * x334 * x195) + x226 +
					 (x321 * x129) + (x338 * x195) + (-1 * x332 * x182) + (x206 * x336) + x314 + (-1 * x315);
	const FLT x341 = (-1 * ((-1 * x333 * x113) + (x339 * x151)) * x152) +
					 (-1 * x141 * ((x340 * x150) + (-1 * ((x339 * x147) + (x333 * x146)) * x148)));
	const FLT x342 = x104 * x114;
	const FLT x343 = -1 * x24 * x342;
	const FLT x344 = (-1 * x26 * x342) + x105;
	const FLT x345 = x344 + x343;
	const FLT x346 = x25 * x342;
	const FLT x347 = x27 * x346;
	const FLT x348 = x23 * x342;
	const FLT x349 = x28 * x348;
	const FLT x350 = x349 + x347;
	const FLT x351 = x28 * x346;
	const FLT x352 = x27 * x348;
	const FLT x353 = (-1 * x352) + x351;
	const FLT x354 = (-1 * ((-1 * x345 * x113) + (x350 * x151)) * x152) +
					 (-1 * x141 * ((x353 * x150) + (-1 * ((x350 * x147) + (x345 * x146)) * x148)));
	const FLT x355 = -1 * x29 * x342;
	const FLT x356 = x105 + x355 + x343;
	const FLT x357 = x23 * x346;
	const FLT x358 = x342 * x158;
	const FLT x359 = (-1 * x358) + x357;
	const FLT x360 = x351 + x352;
	const FLT x361 = (-1 * ((-1 * x360 * x113) + (x359 * x151)) * x152) +
					 (-1 * x141 * ((x356 * x150) + (-1 * ((x359 * x147) + (x360 * x146)) * x148)));
	const FLT x362 = (-1 * x347) + x349;
	const FLT x363 = x344 + x355;
	const FLT x364 = x357 + x358;
	const FLT x365 = (-1 * ((-1 * x362 * x113) + (x363 * x151)) * x152) +
					 (-1 * x141 * ((x364 * x150) + (-1 * ((x363 * x147) + (x362 * x146)) * x148)));
	const FLT x366 = x156 + x119;
	const FLT x367 = x122 + x124;
	const FLT x368 = -1 * x142;
	const FLT x369 = -1 * x143;
	const FLT x370 = 1 + x369 + x368;
	const FLT x371 = (-1 * ((-1 * x370 * x113) + (x367 * x151)) * x152) +
					 (-1 * x141 * ((x366 * x150) + (-1 * ((x367 * x147) + (x370 * x146)) * x148)));
	const FLT x372 = x119 + x117;
	const FLT x373 = x168 + x160;
	const FLT x374 = 1 + (-1 * x163);
	const FLT x375 = x374 + x368;
	const FLT x376 = (-1 * ((-1 * x372 * x113) + (x373 * x151)) * x152) +
					 (-1 * x141 * ((x375 * x150) + (-1 * ((x373 * x147) + (x372 * x146)) * x148)));
	const FLT x377 = x125 + x122;
	const FLT x378 = x374 + x369;
	const FLT x379 = x160 + x159;
	const FLT x380 = (-1 * ((-1 * x377 * x113) + (x378 * x151)) * x152) +
					 (-1 * x141 * ((x379 * x150) + (-1 * ((x378 * x147) + (x377 * x146)) * x148)));
	const FLT x381 = x87 * x79;
	const FLT x382 = x47 * x381;
	const FLT x383 = x87 * x77;
	const FLT x384 = x44 * x383;
	const FLT x385 = x87 * x40;
	const FLT x386 = -1 * x80 * x385;
	const FLT x387 = x78 * x90;
	const FLT x388 = x387 + x386;
	const FLT x389 = x388 + x382 + x384;
	const FLT x390 = x87 * x78;
	const FLT x391 = x44 * x390;
	const FLT x392 = -1 * x391;
	const FLT x393 = x80 * x87;
	const FLT x394 = x47 * x393;
	const FLT x395 = -1 * x394;
	const FLT x396 = x79 * x385;
	const FLT x397 = x77 * x90;
	const FLT x398 = x397 + (-1 * x396);
	const FLT x399 = x398 + x392 + x395;
	const FLT x400 = x44 * x393;
	const FLT x401 = x79 * x90;
	const FLT x402 = -1 * x47 * x390;
	const FLT x403 = x40 * x383;
	const FLT x404 = x403 + x402;
	const FLT x405 = x400 + x404 + x401;
	const FLT x406 = (x405 * sensor_pt[0]) + (-1 * x389 * sensor_pt[2]) + (x399 * sensor_pt[1]);
	const FLT x407 = 2 * x97;
	const FLT x408 = 2 * x100;
	const FLT x409 = x47 * x383;
	const FLT x410 = x80 * x90;
	const FLT x411 = -1 * x44 * x381;
	const FLT x412 = x78 * x385;
	const FLT x413 = x412 + x411;
	const FLT x414 = x409 + x413 + x410;
	const FLT x415 = (x389 * sensor_pt[1]) + (-1 * x414 * sensor_pt[0]) + (x399 * sensor_pt[2]);
	const FLT x416 = 2 * x102;
	const FLT x417 = 2 * x103;
	const FLT x418 = (x414 * x417) + (x415 * x416) + (-1 * x406 * x407) + (-1 * x405 * x408);
	const FLT x419 = 2 * x99;
	const FLT x420 = (x414 * sensor_pt[2]) + (-1 * x405 * sensor_pt[1]) + (x399 * sensor_pt[0]);
	const FLT x421 = 2 * x107;
	const FLT x422 = (x405 * x421) + (x407 * x420) + (-1 * x415 * x419) + (-1 * x417 * x389);
	const FLT x423 = (x406 * x419) + (-1 * x416 * x420) + (-1 * x414 * x421) + (x408 * x389);
	const FLT x424 = (x34 * x423) + (x32 * x418) + (-1 * x38 * x422);
	const FLT x425 = (-1 * x36 * x418) + (x38 * x423) + (x34 * x422);
	const FLT x426 = x418 + (-1 * x424 * x192) + (x425 * x206);
	const FLT x427 = (-1 * x32 * x423) + (x36 * x422) + (x34 * x418);
	const FLT x428 = x423 + (x427 * x192) + (-1 * x425 * x240);
	const FLT x429 = x422 + (x424 * x240) + (-1 * x427 * x206);
	const FLT x430 = (-1 * ((-1 * x426 * x113) + (x428 * x151)) * x152) +
					 (-1 * x141 * ((x429 * x150) + (-1 * ((x428 * x147) + (x426 * x146)) * x148)));
	const FLT x431 = -1 * x401;
	const FLT x432 = -1 * x400;
	const FLT x433 = x431 + x404 + x432;
	const FLT x434 = x396 + x397;
	const FLT x435 = x434 + x394 + x392;
	const FLT x436 = -1 * x384;
	const FLT x437 = x386 + (-1 * x387);
	const FLT x438 = x437 + x436 + x382;
	const FLT x439 = -1 * x409;
	const FLT x440 = (-1 * x412) + x411;
	const FLT x441 = x440 + x439 + x410;
	const FLT x442 = (x441 * sensor_pt[0]) + (-1 * x435 * sensor_pt[2]) + (x438 * sensor_pt[1]);
	const FLT x443 = (x433 * sensor_pt[2]) + (-1 * x441 * sensor_pt[1]) + (x438 * sensor_pt[0]);
	const FLT x444 = (-1 * x416 * x443) + (x408 * x435) + (-1 * x421 * x433) + (x419 * x442);
	const FLT x445 = (x435 * sensor_pt[1]) + (-1 * x433 * sensor_pt[0]) + (x438 * sensor_pt[2]);
	const FLT x446 = (x421 * x441) + (x407 * x443) + (-1 * x419 * x445) + (-1 * x417 * x435);
	const FLT x447 = (x416 * x445) + (-1 * x408 * x441) + (x417 * x433) + (-1 * x407 * x442);
	const FLT x448 = (-1 * x36 * x447) + (x38 * x444) + (x34 * x446);
	const FLT x449 = (x34 * x444) + (x32 * x447) + (-1 * x38 * x446);
	const FLT x450 = (x448 * x206) + x447 + (-1 * x449 * x192);
	const FLT x451 = (-1 * x32 * x444) + (x36 * x446) + (x34 * x447);
	const FLT x452 = x444 + (x451 * x192) + (-1 * x448 * x240);
	const FLT x453 = x446 + (x449 * x240) + (-1 * x451 * x206);
	const FLT x454 = (-1 * ((-1 * x450 * x113) + (x452 * x151)) * x152) +
					 (-1 * x141 * ((x453 * x150) + (-1 * ((x452 * x147) + (x450 * x146)) * x148)));
	const FLT x455 = x391 + x434 + x395;
	const FLT x456 = -1 * x410;
	const FLT x457 = x456 + x413 + x439;
	const FLT x458 = x402 + (-1 * x403);
	const FLT x459 = x458 + x432 + x401;
	const FLT x460 = (x459 * sensor_pt[1]) + (-1 * x455 * sensor_pt[0]) + (x457 * sensor_pt[2]);
	const FLT x461 = -1 * x382;
	const FLT x462 = x437 + x384 + x461;
	const FLT x463 = (x455 * sensor_pt[2]) + (-1 * x462 * sensor_pt[1]) + (x457 * sensor_pt[0]);
	const FLT x464 = (x462 * x421) + (-1 * x459 * x417) + (-1 * x460 * x419) + (x463 * x407);
	const FLT x465 = (x462 * sensor_pt[0]) + (-1 * x459 * sensor_pt[2]) + (x457 * sensor_pt[1]);
	const FLT x466 = (x459 * x408) + (x465 * x419) + (-1 * x455 * x421) + (-1 * x463 * x416);
	const FLT x467 = (x455 * x417) + (x460 * x416) + (-1 * x465 * x407) + (-1 * x462 * x408);
	const FLT x468 = (-1 * x36 * x467) + (x34 * x464) + (x38 * x466);
	const FLT x469 = (x34 * x466) + (x32 * x467) + (-1 * x38 * x464);
	const FLT x470 = x467 + (x468 * x206) + (-1 * x469 * x192);
	const FLT x471 = (-1 * x32 * x466) + (x34 * x467) + (x36 * x464);
	const FLT x472 = x466 + (x471 * x192) + (-1 * x468 * x240);
	const FLT x473 = x464 + (x469 * x240) + (-1 * x471 * x206);
	const FLT x474 = (-1 * ((-1 * x470 * x113) + (x472 * x151)) * x152) +
					 (-1 * x141 * ((x473 * x150) + (-1 * ((x472 * x147) + (x470 * x146)) * x148)));
	const FLT x475 = x388 + x436 + x461;
	const FLT x476 = x458 + x400 + x431;
	const FLT x477 = x440 + x456 + x409;
	const FLT x478 = x398 + x391 + x394;
	const FLT x479 = (x478 * sensor_pt[0]) + (x476 * sensor_pt[1]) + (-1 * x477 * sensor_pt[2]);
	const FLT x480 = (-1 * x478 * sensor_pt[1]) + (x475 * sensor_pt[2]) + (x476 * sensor_pt[0]);
	const FLT x481 = (x477 * x408) + (-1 * x480 * x416) + (-1 * x475 * x421) + (x479 * x419);
	const FLT x482 = (x477 * sensor_pt[1]) + (-1 * x475 * sensor_pt[0]) + (x476 * sensor_pt[2]);
	const FLT x483 = (x478 * x421) + (x480 * x407) + (-1 * x482 * x419) + (-1 * x477 * x417);
	const FLT x484 = (-1 * x478 * x408) + (x475 * x417) + (-1 * x479 * x407) + (x482 * x416);
	const FLT x485 = (-1 * x36 * x484) + (x38 * x481) + (x34 * x483);
	const FLT x486 = (x34 * x481) + (x32 * x484) + (-1 * x38 * x483);
	const FLT x487 = x484 + (x485 * x206) + (-1 * x486 * x192);
	const FLT x488 = (x36 * x483) + (-1 * x32 * x481) + (x34 * x484);
	const FLT x489 = x481 + (x488 * x192) + (-1 * x485 * x240);
	const FLT x490 = (x486 * x240) + x483 + (-1 * x488 * x206);
	const FLT x491 = (-1 * ((-1 * x487 * x113) + (x489 * x151)) * x152) +
					 (-1 * x141 * ((x490 * x150) + (-1 * ((x489 * x147) + (x487 * x146)) * x148)));
	const FLT x492 = -1 * x85;
	const FLT x493 = x59 * x92;
	const FLT x494 = dt * dt * dt;
	const FLT x495 = (1. / (x50 * sqrt(x50))) * x53;
	const FLT x496 = x494 * x495;
	const FLT x497 = x44 * x496;
	const FLT x498 = x40 * x497;
	const FLT x499 = x498 * x493;
	const FLT x500 = x44 * x44 * x44;
	const FLT x501 = dt * dt * dt * dt;
	const FLT x502 = (1. / (x50 * x50)) * x54;
	const FLT x503 = 2 * x502;
	const FLT x504 = x501 * x503;
	const FLT x505 = 1.0 * x57;
	const FLT x506 = x44 * x505;
	const FLT x507 = x495 * x501 * x506;
	const FLT x508 = x42 * x504;
	const FLT x509 = x56 * x41;
	const FLT x510 = 2 * x509;
	const FLT x511 = x495 * x505;
	const FLT x512 = x511 * x501;
	const FLT x513 = x83 * x41;
	const FLT x514 = (-1 * x513 * x506) + (x512 * x500) + (x48 * x507) + (x42 * x507) + (-1 * x500 * x504) +
					 (-1 * x44 * x48 * x504) + (x44 * x510) + (-1 * x44 * x508);
	const FLT x515 = 1.0 / 2.0 * (1. / (x58 * sqrt(x58)));
	const FLT x516 = x57 * x515;
	const FLT x517 = x86 * x516;
	const FLT x518 = x86 * x59;
	const FLT x519 = 0.5 * x513;
	const FLT x520 = x44 * x519;
	const FLT x521 = x84 * x515;
	const FLT x522 = x44 * x521;
	const FLT x523 = x81 * x514;
	const FLT x524 = 0.5 * x55 * x494;
	const FLT x525 = x40 * x524;
	const FLT x526 = x44 * x525;
	const FLT x527 = x98 * x526;
	const FLT x528 = x92 * x521;
	const FLT x529 = x40 * x528;
	const FLT x530 = x45 * x496;
	const FLT x531 = x47 * x514;
	const FLT x532 = x89 * x521;
	const FLT x533 = x45 * x524;
	const FLT x534 = x89 * x59;
	const FLT x535 = x47 * x497;
	const FLT x536 = x44 * x47 * x524;
	const FLT x537 = (x91 * x536) + (-1 * x535 * x534);
	const FLT x538 = (-1 * x96 * x533) + (x82 * x530) + (-1 * x514 * x529) + x527 + (-1 * x532 * x531) + (-1 * x499) +
					 x537 + x492 + (-1 * x518 * x520) + (-1 * x514 * x517) + (x522 * x523);
	const FLT x539 = x96 * x536;
	const FLT x540 = x498 * x518;
	const FLT x541 = x521 * x531;
	const FLT x542 = x82 * x535;
	const FLT x543 = x526 * x101;
	const FLT x544 = x92 * x516;
	const FLT x545 = x40 * x521;
	const FLT x546 = x86 * x514;
	const FLT x547 = x89 * x522;
	const FLT x548 = (-1 * x514 * x547) + (-1 * x81 * x541) + x540 + (-1 * x542) + (-1 * x514 * x544) + x539 +
					 (x91 * x533) + (x545 * x546) + (-1 * x543) + x95 + (-1 * x534 * x530) + (-1 * x493 * x520);
	const FLT x549 = -1 * x93;
	const FLT x550 = x47 * x518;
	const FLT x551 = x497 * x550;
	const FLT x552 = x536 * x101;
	const FLT x553 = x44 * x528;
	const FLT x554 = x89 * x516;
	const FLT x555 = (x82 * x498) + (-1 * x96 * x526);
	const FLT x556 = x555 + (-1 * x514 * x554) + (x514 * x553) + (x493 * x530) + (-1 * x520 * x534) + (-1 * x552) +
					 (-1 * x98 * x533) + x549 + (x523 * x545) + x551 + (x86 * x541);
	const FLT x557 = x81 * x516;
	const FLT x558 = x82 * x519;
	const FLT x559 = x47 * x493;
	const FLT x560 = (x497 * x559) + (-1 * x98 * x536);
	const FLT x561 = x91 * x525;
	const FLT x562 = (-1 * x498 * x534) + (x44 * x561);
	const FLT x563 = x560 + (-1 * x518 * x530) + (-1 * x522 * x546) + (x528 * x531) + x562 + (-1 * x514 * x557) +
					 (x533 * x101) + (-1 * x40 * x514 * x532) + x88 + (-1 * x44 * x558);
	const FLT x564 = (x563 * sensor_pt[0]) + (-1 * x548 * sensor_pt[2]) + (x556 * sensor_pt[1]);
	const FLT x565 = (x538 * sensor_pt[2]) + (-1 * x563 * sensor_pt[1]) + (x556 * sensor_pt[0]);
	const FLT x566 = (x419 * x564) + (-1 * x421 * x538) + (-1 * x416 * x565) + (x408 * x548);
	const FLT x567 = (x548 * sensor_pt[1]) + (-1 * x538 * sensor_pt[0]) + (x556 * sensor_pt[2]);
	const FLT x568 = (-1 * x417 * x548) + (x421 * x563) + (-1 * x419 * x567) + (x407 * x565);
	const FLT x569 = (x416 * x567) + (x417 * x538) + (-1 * x407 * x564) + (-1 * x408 * x563);
	const FLT x570 = (-1 * x36 * x569) + (x38 * x566) + (x34 * x568);
	const FLT x571 = (x34 * x566) + (x32 * x569) + (-1 * x38 * x568);
	const FLT x572 = x569 + (x570 * x206) + (-1 * x571 * x192);
	const FLT x573 = (-1 * x32 * x566) + (x36 * x568) + (x34 * x569);
	const FLT x574 = x566 + (-1 * x570 * x240) + (x573 * x192);
	const FLT x575 = x568 + (-1 * x573 * x206) + (x571 * x240);
	const FLT x576 = (-1 * ((-1 * x572 * x113) + (x574 * x151)) * x152) +
					 (-1 * x141 * ((x575 * x150) + (-1 * ((x574 * x147) + (x572 * x146)) * x148)));
	const FLT x577 = x45 * x47;
	const FLT x578 = x513 * x505;
	const FLT x579 = x47 * x47 * x47;
	const FLT x580 = (x47 * x510) + (-1 * x504 * x579) + (x512 * x577) + (x512 * x579) + (-1 * x47 * x578) +
					 (x42 * x47 * x512) + (-1 * x504 * x577) + (-1 * x47 * x508);
	const FLT x581 = x516 * x580;
	const FLT x582 = x48 * x496;
	const FLT x583 = x47 * x525;
	const FLT x584 = x98 * x583;
	const FLT x585 = x89 * x580;
	const FLT x586 = x47 * x521;
	const FLT x587 = x40 * x496;
	const FLT x588 = x587 * x559;
	const FLT x589 = x48 * x524;
	const FLT x590 = x40 * x580;
	const FLT x591 = x81 * x580;
	const FLT x592 = x95 + (-1 * x86 * x581) + x584 + (-1 * x534 * x582) + (-1 * x585 * x586) + (x591 * x522) +
					 (-1 * x539) + (-1 * x588) + (x91 * x589) + x542 + (-1 * x590 * x528) + (-1 * x519 * x550);
	const FLT x593 = x47 * x528;
	const FLT x594 = x590 * x521;
	const FLT x595 = x86 * x580;
	const FLT x596 = x47 * x534;
	const FLT x597 = (x47 * x561) + (-1 * x596 * x587);
	const FLT x598 = x597 + (-1 * x595 * x522) + (x493 * x582) + (-1 * x551) + (-1 * x98 * x589) + x549 + x552 +
					 (-1 * x81 * x581) + (-1 * x89 * x594) + (-1 * x47 * x558) + (x593 * x580);
	const FLT x599 = x96 * x583;
	const FLT x600 = x82 * x47 * x587;
	const FLT x601 = -1 * x88;
	const FLT x602 = (-1 * x580 * x554) + (-1 * x596 * x519) + (x580 * x553) + (x81 * x594) + (-1 * x589 * x101) +
					 (-1 * x599) + x600 + x560 + x601 + (x595 * x586) + (x518 * x582);
	const FLT x603 = (x592 * sensor_pt[2]) + (-1 * x598 * sensor_pt[1]) + (x602 * sensor_pt[0]);
	const FLT x604 = (x587 * x550) + (-1 * x583 * x101);
	const FLT x605 = x604 + x537 + (-1 * x591 * x586) + (-1 * x82 * x582) + (x86 * x594) + (x96 * x589) +
					 (-1 * x519 * x559) + (-1 * x92 * x581) + (-1 * x522 * x585) + x85;
	const FLT x606 = (x598 * sensor_pt[0]) + (-1 * x605 * sensor_pt[2]) + (x602 * sensor_pt[1]);
	const FLT x607 = (x419 * x606) + (x408 * x605) + (-1 * x421 * x592) + (-1 * x416 * x603);
	const FLT x608 = (x605 * sensor_pt[1]) + (-1 * x592 * sensor_pt[0]) + (x602 * sensor_pt[2]);
	const FLT x609 = (x421 * x598) + (x407 * x603) + (-1 * x419 * x608) + (-1 * x417 * x605);
	const FLT x610 = (x417 * x592) + (x416 * x608) + (-1 * x407 * x606) + (-1 * x408 * x598);
	const FLT x611 = (-1 * x36 * x610) + (x38 * x607) + (x34 * x609);
	const FLT x612 = (-1 * x38 * x609) + (x34 * x607) + (x32 * x610);
	const FLT x613 = x610 + (x611 * x206) + (-1 * x612 * x192);
	const FLT x614 = (-1 * x32 * x607) + (x36 * x609) + (x34 * x610);
	const FLT x615 = x607 + (x614 * x192) + (-1 * x611 * x240);
	const FLT x616 = (x612 * x240) + x609 + (-1 * x614 * x206);
	const FLT x617 = (-1 * ((-1 * x613 * x113) + (x615 * x151)) * x152) +
					 (-1 * x141 * ((x616 * x150) + (-1 * ((x615 * x147) + (x613 * x146)) * x148)));
	const FLT x618 = x40 * x512;
	const FLT x619 = 2 * x40;
	const FLT x620 = x619 * x501 * x502;
	const FLT x621 = (x40 * x40 * x40) * x501;
	const FLT x622 = (x621 * x511) + (-1 * x621 * x503) + (-1 * x45 * x620) + (-1 * x40 * x578) + (x45 * x618) +
					 (x48 * x618) + (-1 * x48 * x620) + (x619 * x509);
	const FLT x623 = x81 * x622;
	const FLT x624 = x40 * x519;
	const FLT x625 = x42 * x496;
	const FLT x626 = x86 * x622;
	const FLT x627 = x42 * x524;
	const FLT x628 = (x622 * x553) + x604 + (-1 * x96 * x627) + (x626 * x586) + (-1 * x527) + x492 + (x623 * x545) +
					 x499 + (-1 * x622 * x554) + (-1 * x624 * x534) + (x82 * x625);
	const FLT x629 = x622 * x532;
	const FLT x630 = x597 + (x623 * x522) + (x98 * x627) + x93 + (-1 * x47 * x629) + (-1 * x624 * x518) +
					 (-1 * x622 * x517) + (-1 * x622 * x529) + x555 + (-1 * x493 * x625);
	const FLT x631 = x562 + (-1 * x627 * x101) + (-1 * x623 * x586) + x599 + (x626 * x545) + (x625 * x518) + x601 +
					 (-1 * x622 * x544) + (-1 * x600) + (-1 * x622 * x547) + (-1 * x493 * x624);
	const FLT x632 = (x628 * sensor_pt[2]) + (x631 * sensor_pt[1]) + (-1 * x630 * sensor_pt[0]);
	const FLT x633 = x588 + (x622 * x593) + (-1 * x622 * x557) + x543 + (-1 * x82 * x624) + (-1 * x540) +
					 (-1 * x40 * x629) + (x91 * x627) + (-1 * x626 * x522) + (-1 * x625 * x534) + (-1 * x584) + x95;
	const FLT x634 = (x630 * sensor_pt[2]) + (-1 * x633 * sensor_pt[1]) + (x628 * sensor_pt[0]);
	const FLT x635 = (x407 * x634) + (x421 * x633) + (-1 * x419 * x632) + (-1 * x417 * x631);
	const FLT x636 = (x633 * sensor_pt[0]) + (-1 * x631 * sensor_pt[2]) + (x628 * sensor_pt[1]);
	const FLT x637 = (x419 * x636) + (x408 * x631) + (-1 * x421 * x630) + (-1 * x416 * x634);
	const FLT x638 = (-1 * x408 * x633) + (x417 * x630) + (x416 * x632) + (-1 * x407 * x636);
	const FLT x639 = (-1 * x36 * x638) + (x34 * x635) + (x38 * x637);
	const FLT x640 = (x34 * x637) + (x32 * x638) + (-1 * x38 * x635);
	const FLT x641 = (x639 * x206) + x638 + (-1 * x640 * x192);
	const FLT x642 = (-1 * x32 * x637) + (x34 * x638) + (x36 * x635);
	const FLT x643 = x637 + (x642 * x192) + (-1 * x639 * x240);
	const FLT x644 = (x640 * x240) + x635 + (-1 * x642 * x206);
	const FLT x645 = (-1 * ((-1 * x641 * x113) + (x643 * x151)) * x152) +
					 (-1 * x141 * ((x644 * x150) + (-1 * ((x643 * x147) + (x641 * x146)) * x148)));
	const FLT x646 = -1 * dt * x143;
	const FLT x647 = (-1 * dt * x142) + dt;
	const FLT x648 = x647 + x646;
	const FLT x649 = dt * x124;
	const FLT x650 = dt * x122;
	const FLT x651 = x650 + x649;
	const FLT x652 = dt * x119;
	const FLT x653 = dt * x117;
	const FLT x654 = (-1 * x653) + x652;
	const FLT x655 = (-1 * ((-1 * x648 * x113) + (x651 * x151)) * x152) +
					 (-1 * x141 * ((x654 * x150) + (-1 * ((x651 * x147) + (x648 * x146)) * x148)));
	const FLT x656 = x652 + x653;
	const FLT x657 = dt * x160;
	const FLT x658 = dt * x159;
	const FLT x659 = (-1 * x658) + x657;
	const FLT x660 = -1 * dt * x163;
	const FLT x661 = x647 + x660;
	const FLT x662 = (-1 * ((-1 * x656 * x113) + (x659 * x151)) * x152) +
					 (-1 * x141 * ((x661 * x150) + (-1 * ((x659 * x147) + (x656 * x146)) * x148)));
	const FLT x663 = (-1 * x649) + x650;
	const FLT x664 = x646 + dt + x660;
	const FLT x665 = x657 + x658;
	const FLT x666 = (-1 * ((-1 * x663 * x113) + (x664 * x151)) * x152) +
					 (-1 * x141 * ((x665 * x150) + (-1 * ((x664 * x147) + (x663 * x146)) * x148)));
	const FLT x667 = x131 * x141 * x149;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[0]) / sizeof(FLT),
						x153 + (((x126 * x132) + (-1 * x113 * x121)) * x136) + (x153 * x155));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[1]) / sizeof(FLT),
						x165 + (x165 * x155) + (((x162 * x132) + (-1 * x113 * x164)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[2]) / sizeof(FLT),
						x170 + (x170 * x155) + (((x167 * x132) + (-1 * x113 * x169)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						x253 + (x253 * x155) + (((x246 * x132) + (-1 * x252 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						x295 + (x295 * x155) + (((x291 * x132) + (-1 * x294 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						x319 + (x319 * x155) + (((x316 * x132) + (-1 * x318 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						x341 + (x341 * x155) + (((x339 * x132) + (-1 * x340 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Acc[0]) / sizeof(FLT),
						(x354 * x155) + x354 + (((x350 * x132) + (-1 * x353 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Acc[1]) / sizeof(FLT),
						x361 + (((x359 * x132) + (-1 * x356 * x113)) * x136) + (x361 * x155));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Acc[2]) / sizeof(FLT),
						x365 + (x365 * x155) + (((x363 * x132) + (-1 * x364 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[0]) / sizeof(FLT),
						x371 + (((x367 * x132) + (-1 * x366 * x113)) * x136) + (x371 * x155));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[1]) / sizeof(FLT),
						(x376 * x155) + x376 + (((x373 * x132) + (-1 * x375 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[2]) / sizeof(FLT),
						x380 + (x380 * x155) + (((x378 * x132) + (-1 * x379 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT),
						(x430 * x155) + x430 + (((x428 * x132) + (-1 * x429 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT),
						x454 + (x454 * x155) + (((x452 * x132) + (-1 * x453 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT),
						x474 + (x474 * x155) + (((x472 * x132) + (-1 * x473 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT),
						x491 + (x491 * x155) + (((x489 * x132) + (-1 * x490 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[0]) / sizeof(FLT),
						(x576 * x155) + x576 + (((x574 * x132) + (-1 * x575 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x617 + (x617 * x155) + (((x615 * x132) + (-1 * x616 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x645 + (x645 * x155) + (((x643 * x132) + (-1 * x644 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[0]) / sizeof(FLT),
						x655 + (x655 * x155) + (((x651 * x132) + (-1 * x654 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[1]) / sizeof(FLT),
						x662 + (x662 * x155) + (((x659 * x132) + (-1 * x661 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[2]) / sizeof(FLT),
						x666 + (x666 * x155) + (((x664 * x132) + (-1 * x665 * x113)) * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD0.curve) / sizeof(FLT), x135 * x135);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD0.gibmag) / sizeof(FLT), -1 * cos(x154));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD0.gibpha) / sizeof(FLT), x155);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD0.phase) / sizeof(FLT), -1 + (-1 * x155));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD0.tilt) / sizeof(FLT),
						(-1 * x667) + (-1 * x667 * x155));
}

// Full version Jacobian of SurviveJointKalmanErrorModel_LightMeas_x_gen1 wrt [(*_x0).Lighthouse.Pos[0],
// (*_x0).Lighthouse.Pos[1], (*_x0).Lighthouse.Pos[2], (*_x0).Lighthouse.Rot[0], (*_x0).Lighthouse.Rot[1],
// (*_x0).Lighthouse.Rot[2], (*_x0).Lighthouse.Rot[3], (*_x0).Object.Acc[0], (*_x0).Object.Acc[1], (*_x0).Object.Acc[2],
// (*_x0).Object.IMUBias.AccBias[0], (*_x0).Object.IMUBias.AccBias[1], (*_x0).Object.IMUBias.AccBias[2],
// (*_x0).Object.IMUBias.AccScale[0], (*_x0).Object.IMUBias.AccScale[1], (*_x0).Object.IMUBias.AccScale[2],
// (*_x0).Object.IMUBias.GyroBias[0], (*_x0).Object.IMUBias.GyroBias[1], (*_x0).Object.IMUBias.GyroBias[2],
// (*_x0).Object.IMUBias.IMUCorrection[0], (*_x0).Object.IMUBias.IMUCorrection[1],
// (*_x0).Object.IMUBias.IMUCorrection[2], (*_x0).Object.IMUBias.IMUCorrection[3], (*_x0).Object.Pose.Pos[0],
// (*_x0).Object.Pose.Pos[1], (*_x0).Object.Pose.Pos[2], (*_x0).Object.Pose.Rot[0], (*_x0).Object.Pose.Rot[1],
// (*_x0).Object.Pose.Rot[2], (*_x0).Object.Pose.Rot[3], (*_x0).Object.Velocity.AxisAngleRot[0],
// (*_x0).Object.Velocity.AxisAngleRot[1], (*_x0).Object.Velocity.AxisAngleRot[2], (*_x0).Object.Velocity.Pos[0],
// (*_x0).Object.Velocity.Pos[1], (*_x0).Object.Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f4d1c32f2e0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32f5b0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32f3a0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32f670>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32f340>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32f610>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32f460>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32f730>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32f400>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32f6d0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c398dc0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32f4f0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32f280>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32f550>]

static inline void SurviveJointKalmanErrorModel_LightMeas_x_gen1_jac_x0_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveJointKalmanModel *_x0,
	const SurviveJointKalmanErrorModel *error_model, const FLT *sensor_pt) {
	if (hx != 0) {
		hx->data[0] = SurviveJointKalmanErrorModel_LightMeas_x_gen1(dt, _x0, error_model, sensor_pt);
	}
	if (Hx != 0) {
		SurviveJointKalmanErrorModel_LightMeas_x_gen1_jac_x0(Hx, dt, _x0, error_model, sensor_pt);
	}
}
// Jacobian of SurviveJointKalmanErrorModel_LightMeas_x_gen1 wrt [(*error_model).Lighthouse.AxisAngleRot[0],
// (*error_model).Lighthouse.AxisAngleRot[1], (*error_model).Lighthouse.AxisAngleRot[2],
// (*error_model).Lighthouse.Pos[0], (*error_model).Lighthouse.Pos[1], (*error_model).Lighthouse.Pos[2],
// (*error_model).Object.Acc[0], (*error_model).Object.Acc[1], (*error_model).Object.Acc[2],
// (*error_model).Object.IMUBias.AccBias[0], (*error_model).Object.IMUBias.AccBias[1],
// (*error_model).Object.IMUBias.AccBias[2], (*error_model).Object.IMUBias.AccScale[0],
// (*error_model).Object.IMUBias.AccScale[1], (*error_model).Object.IMUBias.AccScale[2],
// (*error_model).Object.IMUBias.GyroBias[0], (*error_model).Object.IMUBias.GyroBias[1],
// (*error_model).Object.IMUBias.GyroBias[2], (*error_model).Object.IMUBias.IMUCorrection[0],
// (*error_model).Object.IMUBias.IMUCorrection[1], (*error_model).Object.IMUBias.IMUCorrection[2],
// (*error_model).Object.Pose.AxisAngleRot[0], (*error_model).Object.Pose.AxisAngleRot[1],
// (*error_model).Object.Pose.AxisAngleRot[2], (*error_model).Object.Pose.Pos[0], (*error_model).Object.Pose.Pos[1],
// (*error_model).Object.Pose.Pos[2], (*error_model).Object.Velocity.AxisAngleRot[0],
// (*error_model).Object.Velocity.AxisAngleRot[1], (*error_model).Object.Velocity.AxisAngleRot[2],
// (*error_model).Object.Velocity.Pos[0], (*error_model).Object.Velocity.Pos[1], (*error_model).Object.Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f4d1c328700>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3283d0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3284c0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c328160>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c328430>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3280d0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c328580>, <cnkalman.codegen.WrapMember object at 0x7f4d1c328250>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c328520>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3281f0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c328850>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3282e0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3286a0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c328370>]
static inline void SurviveJointKalmanErrorModel_LightMeas_x_gen1_jac_error_model(
	CnMat *Hx, const FLT dt, const SurviveJointKalmanModel *_x0, const SurviveJointKalmanErrorModel *error_model,
	const FLT *sensor_pt) {
	const FLT x0 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[1];
	const FLT x1 = sin(x0);
	const FLT x2 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[2];
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[0];
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = x1 * x6;
	const FLT x8 = cos(x4);
	const FLT x9 = cos(x0);
	const FLT x10 = cos(x2);
	const FLT x11 = x9 * x10;
	const FLT x12 = x8 * x11;
	const FLT x13 = x12 + x7;
	const FLT x14 = x1 * x10;
	const FLT x15 = x5 * x14;
	const FLT x16 = x3 * x8;
	const FLT x17 = x9 * x16;
	const FLT x18 = x17 + (-1 * x15);
	const FLT x19 = x6 * x9;
	const FLT x20 = x8 * x14;
	const FLT x21 = x20 + x19;
	const FLT x22 = x1 * x16;
	const FLT x23 = x5 * x11;
	const FLT x24 = x23 + (-1 * x22);
	const FLT x25 = (x24 * x24) + (x13 * x13) + (x18 * x18) + (x21 * x21);
	const FLT x26 = 1. / sqrt(x25);
	const FLT x27 = x26 * (*_x0).Lighthouse.Rot[3];
	const FLT x28 = x26 * (*_x0).Lighthouse.Rot[2];
	const FLT x29 = x26 * (*_x0).Lighthouse.Rot[0];
	const FLT x30 = x26 * (*_x0).Lighthouse.Rot[1];
	const FLT x31 = (x29 * x18) + (x27 * x13) + (x30 * x21) + (-1 * x24 * x28);
	const FLT x32 = x31 * x31;
	const FLT x33 = (-1 * x30 * x18) + (x21 * x29) + (x24 * x27) + (x28 * x13);
	const FLT x34 = x33 * x33;
	const FLT x35 = (x29 * x13) + (-1 * x30 * x24) + (-1 * x27 * x18) + (-1 * x21 * x28);
	const FLT x36 = (-1 * x21 * x27) + (x30 * x13) + (x28 * x18) + (x24 * x29);
	const FLT x37 = x36 * x36;
	const FLT x38 = x37 + (x35 * x35) + x32 + x34;
	const FLT x39 = 1. / sqrt(x38);
	const FLT x40 = 0.5 * x15;
	const FLT x41 = -1 * x40;
	const FLT x42 = 0.5 * x17;
	const FLT x43 = x42 + x41;
	const FLT x44 = 1.0 / 2.0 * (1. / (x25 * sqrt(x25)));
	const FLT x45 = x44 * (*_x0).Lighthouse.Rot[0];
	const FLT x46 = 0.5 * x12;
	const FLT x47 = 0.5 * x7;
	const FLT x48 = x47 + x46;
	const FLT x49 = 2 * x48;
	const FLT x50 = 0.5 * x23;
	const FLT x51 = -1 * x50;
	const FLT x52 = 0.5 * x22;
	const FLT x53 = x52 + x51;
	const FLT x54 = 2 * x13;
	const FLT x55 = 0.5 * x19;
	const FLT x56 = -0.5 * x20;
	const FLT x57 = x56 + (-1 * x55);
	const FLT x58 = 2 * x18;
	const FLT x59 = 2 * x21;
	const FLT x60 = (x59 * x43) + (x58 * x57) + (x49 * x24) + (x54 * x53);
	const FLT x61 = x60 * x18;
	const FLT x62 = x44 * x13;
	const FLT x63 = x62 * (*_x0).Lighthouse.Rot[3];
	const FLT x64 = x48 * x28;
	const FLT x65 = x60 * x24;
	const FLT x66 = x65 * x44;
	const FLT x67 = x60 * x21;
	const FLT x68 = x67 * x44;
	const FLT x69 = x57 * x29;
	const FLT x70 = (-1 * x68 * (*_x0).Lighthouse.Rot[1]) + (x66 * (*_x0).Lighthouse.Rot[2]) + (-1 * x61 * x45) + x69 +
					(x43 * x30) + (-1 * x60 * x63) + (x53 * x27) + (-1 * x64);
	const FLT x71 = x70 * x39;
	const FLT x72 = (*_x0).Object.Velocity.AxisAngleRot[2] + (*error_model).Object.Velocity.AxisAngleRot[2];
	const FLT x73 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[0];
	const FLT x74 = sin(x73);
	const FLT x75 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[2];
	const FLT x76 = sin(x75);
	const FLT x77 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[1];
	const FLT x78 = sin(x77);
	const FLT x79 = x78 * x76;
	const FLT x80 = x79 * x74;
	const FLT x81 = cos(x73);
	const FLT x82 = cos(x77);
	const FLT x83 = cos(x75);
	const FLT x84 = x82 * x83;
	const FLT x85 = x81 * x84;
	const FLT x86 = x85 + x80;
	const FLT x87 = x82 * x76;
	const FLT x88 = x87 * x74;
	const FLT x89 = x83 * x78;
	const FLT x90 = x81 * x89;
	const FLT x91 = x90 + x88;
	const FLT x92 = x89 * x74;
	const FLT x93 = x81 * x87;
	const FLT x94 = x93 + (-1 * x92);
	const FLT x95 = x81 * x79;
	const FLT x96 = x84 * x74;
	const FLT x97 = x96 + (-1 * x95);
	const FLT x98 = (x94 * x94) + (x91 * x91) + (x97 * x97) + (x86 * x86);
	const FLT x99 = 1. / sqrt(x98);
	const FLT x100 = x99 * (*_x0).Object.Pose.Rot[3];
	const FLT x101 = x99 * (*_x0).Object.Pose.Rot[2];
	const FLT x102 = x99 * (*_x0).Object.Pose.Rot[0];
	const FLT x103 = x91 * x99;
	const FLT x104 = (x103 * (*_x0).Object.Pose.Rot[1]) + (x94 * x102) + (x86 * x100) + (-1 * x97 * x101);
	const FLT x105 = dt * dt;
	const FLT x106 = x72 * x72;
	const FLT x107 = x105 * x106;
	const FLT x108 = (*_x0).Object.Velocity.AxisAngleRot[0] + (*error_model).Object.Velocity.AxisAngleRot[0];
	const FLT x109 = x108 * x108;
	const FLT x110 = x109 * x105;
	const FLT x111 = (*_x0).Object.Velocity.AxisAngleRot[1] + (*error_model).Object.Velocity.AxisAngleRot[1];
	const FLT x112 = x111 * x111;
	const FLT x113 = x105 * x112;
	const FLT x114 = 1e-10 + x113 + x107 + x110;
	const FLT x115 = sqrt(x114);
	const FLT x116 = 0.5 * x115;
	const FLT x117 = sin(x116);
	const FLT x118 = x117 * x117;
	const FLT x119 = 1. / x114;
	const FLT x120 = x118 * x119;
	const FLT x121 = cos(x116);
	const FLT x122 = (x110 * x120) + (x121 * x121) + (x107 * x120) + (x113 * x120);
	const FLT x123 = 1. / sqrt(x122);
	const FLT x124 = x117 * x123;
	const FLT x125 = 1. / x115;
	const FLT x126 = dt * x125;
	const FLT x127 = x124 * x126;
	const FLT x128 = x104 * x127;
	const FLT x129 = x99 * (*_x0).Object.Pose.Rot[1];
	const FLT x130 = (-1 * x94 * x129) + (x103 * (*_x0).Object.Pose.Rot[0]) + (x86 * x101) + (x97 * x100);
	const FLT x131 = x127 * x130;
	const FLT x132 = (-1 * x97 * x129) + (-1 * x94 * x100) + (x86 * x102) + (-1 * x103 * (*_x0).Object.Pose.Rot[2]);
	const FLT x133 = x123 * x121;
	const FLT x134 = x133 * x132;
	const FLT x135 = (x86 * x129) + (x97 * x102) + (-1 * x103 * (*_x0).Object.Pose.Rot[3]) + (x94 * x101);
	const FLT x136 = x124 * x135;
	const FLT x137 = x126 * x136;
	const FLT x138 = (-1 * x108 * x137) + x134 + (-1 * x72 * x128) + (-1 * x111 * x131);
	const FLT x139 = x133 * x135;
	const FLT x140 = x127 * x132;
	const FLT x141 = (x108 * x140) + x139 + (-1 * x72 * x131) + (x111 * x128);
	const FLT x142 = x130 * x133;
	const FLT x143 = (x111 * x140) + (x72 * x137) + (-1 * x108 * x128) + x142;
	const FLT x144 = (-1 * x143 * sensor_pt[0]) + (x138 * sensor_pt[2]) + (x141 * sensor_pt[1]);
	const FLT x145 = x104 * x133;
	const FLT x146 = (-1 * x111 * x137) + (x72 * x140) + (x108 * x131) + x145;
	const FLT x147 = (-1 * x146 * sensor_pt[1]) + (x138 * sensor_pt[0]) + (x143 * sensor_pt[2]);
	const FLT x148 = dt * fabs(dt);
	const FLT x149 = 1.0 / 2.0 * x148;
	const FLT x150 = (*_x0).Object.Pose.Pos[1] + (*error_model).Object.Pose.Pos[1] +
					 (2 * ((x146 * x147) + (-1 * x141 * x144))) + sensor_pt[1] +
					 (dt * ((*_x0).Object.Velocity.Pos[1] + (*error_model).Object.Velocity.Pos[1])) +
					 (x149 * ((*_x0).Object.Acc[1] + (*error_model).Object.Acc[1]));
	const FLT x151 = x35 * x39;
	const FLT x152 = (-1 * x141 * sensor_pt[2]) + (x138 * sensor_pt[1]) + (x146 * sensor_pt[0]);
	const FLT x153 = (*_x0).Object.Pose.Pos[0] + (2 * ((x144 * x143) + (-1 * x146 * x152))) +
					 (*error_model).Object.Pose.Pos[0] +
					 (dt * ((*_x0).Object.Velocity.Pos[0] + (*error_model).Object.Velocity.Pos[0])) +
					 (x149 * ((*_x0).Object.Acc[0] + (*error_model).Object.Acc[0])) + sensor_pt[0];
	const FLT x154 = x39 * x153;
	const FLT x155 = (dt * ((*_x0).Object.Velocity.Pos[2] + (*error_model).Object.Velocity.Pos[2])) +
					 (*error_model).Object.Pose.Pos[2] + (2 * ((x141 * x152) + (-1 * x143 * x147))) + sensor_pt[2] +
					 (x149 * ((*_x0).Object.Acc[2] + (*error_model).Object.Acc[2])) + (*_x0).Object.Pose.Pos[2];
	const FLT x156 = x39 * x155;
	const FLT x157 = (x36 * x156) + (x150 * x151) + (-1 * x31 * x154);
	const FLT x158 = 2 * x157;
	const FLT x159 = -1 * x48 * x30;
	const FLT x160 = x45 * x13;
	const FLT x161 = x44 * (*_x0).Lighthouse.Rot[3];
	const FLT x162 = x57 * x27;
	const FLT x163 = (x53 * x29) + (x68 * (*_x0).Lighthouse.Rot[2]) + (-1 * x43 * x28) + x159 +
					 (x66 * (*_x0).Lighthouse.Rot[1]) + (-1 * x60 * x160) + (x61 * x161) + (-1 * x162);
	const FLT x164 = 2 * x35;
	const FLT x165 = x60 * x62;
	const FLT x166 = x44 * (*_x0).Lighthouse.Rot[2];
	const FLT x167 = x57 * x28;
	const FLT x168 = x48 * x29;
	const FLT x169 = (-1 * x43 * x27) + x168 + (-1 * x65 * x45) + (-1 * x165 * (*_x0).Lighthouse.Rot[1]) +
					 (-1 * x61 * x166) + (x53 * x30) + (x68 * (*_x0).Lighthouse.Rot[3]) + x167;
	const FLT x170 = 2 * x36;
	const FLT x171 = 2 * x31;
	const FLT x172 = x44 * (*_x0).Lighthouse.Rot[1];
	const FLT x173 = -1 * x57 * x30;
	const FLT x174 = x48 * x27;
	const FLT x175 = (-1 * x66 * (*_x0).Lighthouse.Rot[3]) + x173 + (x61 * x172) + (x43 * x29) + (-1 * x67 * x45) +
					 (x53 * x28) + (-1 * x165 * (*_x0).Lighthouse.Rot[2]) + x174;
	const FLT x176 = 2 * x33;
	const FLT x177 = (x176 * x175) + (x70 * x171) + (x164 * x163) + (x169 * x170);
	const FLT x178 = x33 * x177;
	const FLT x179 = (*_x0).Lighthouse.Pos[2] + (*error_model).Lighthouse.Pos[2];
	const FLT x180 = (*_x0).Lighthouse.Pos[1] + (*error_model).Lighthouse.Pos[1];
	const FLT x181 = x36 * x39;
	const FLT x182 = (*_x0).Lighthouse.Pos[0] + (*error_model).Lighthouse.Pos[0];
	const FLT x183 = x33 * x39;
	const FLT x184 = (x183 * x182) + (x179 * x151) + (-1 * x181 * x180);
	const FLT x185 = 1. / (x38 * sqrt(x38));
	const FLT x186 = x185 * x184;
	const FLT x187 = x185 * x153;
	const FLT x188 = 1.0 / 2.0 * x178;
	const FLT x189 = x36 * x177;
	const FLT x190 = x189 * x185;
	const FLT x191 = 1.0 / 2.0 * x190;
	const FLT x192 = x39 * x150;
	const FLT x193 = 1.0 / 2.0 * x35;
	const FLT x194 = x177 * x193;
	const FLT x195 = x185 * x194;
	const FLT x196 =
		(-1 * x195 * x155) + (x191 * x150) + (-1 * x187 * x188) + (x163 * x156) + (-1 * x169 * x192) + (x175 * x154);
	const FLT x197 = x39 * x176;
	const FLT x198 = x188 * x185;
	const FLT x199 = x39 * x169;
	const FLT x200 = x39 * x163;
	const FLT x201 = x39 * x175;
	const FLT x202 =
		(x180 * x191) + (-1 * x179 * x195) + (-1 * x180 * x199) + (-1 * x182 * x198) + (x200 * x179) + (x201 * x182);
	const FLT x203 = x31 * x185;
	const FLT x204 = x203 * x177;
	const FLT x205 = 1.0 / 2.0 * x204;
	const FLT x206 =
		(x179 * x199) + (x200 * x180) + (x205 * x182) + (-1 * x180 * x195) + (-1 * x179 * x191) + (-1 * x71 * x182);
	const FLT x207 = x39 * x171;
	const FLT x208 = (x33 * x154) + (-1 * x36 * x192) + (x151 * x155);
	const FLT x209 = x39 * x208;
	const FLT x210 = 2 * x209;
	const FLT x211 = x208 * x185;
	const FLT x212 =
		(x205 * x153) + (-1 * x191 * x155) + (-1 * x70 * x154) + (x169 * x156) + (x163 * x192) + (-1 * x195 * x150);
	const FLT x213 = 2 * x184;
	const FLT x214 = x31 * x39;
	const FLT x215 = (x179 * x181) + (x180 * x151) + (-1 * x214 * x182);
	const FLT x216 = 2 * x215;
	const FLT x217 = (-1 * x71 * x216) + (-1 * x196 * x197) + (x202 * x197) + (-1 * x178 * x186) + (x213 * x201) +
					 (-1 * x206 * x207) + (x211 * x178) + (-1 * x204 * x157) + (x71 * x158) + (-1 * x210 * x175) +
					 (x212 * x207) + (x215 * x204);
	const FLT x218 = x39 * x157;
	const FLT x219 =
		x153 + (2 * ((-1 * x33 * x209) + (x31 * x218))) + (-1 * (x182 + (2 * ((-1 * x183 * x184) + (x215 * x214)))));
	const FLT x220 = 2 * x219;
	const FLT x221 = (x31 * x192) + (x151 * x153) + (-1 * x33 * x156);
	const FLT x222 = 2 * x221;
	const FLT x223 = x185 * x155;
	const FLT x224 =
		(x223 * x188) + (-1 * x201 * x155) + (x71 * x150) + (-1 * x187 * x194) + (x163 * x154) + (-1 * x205 * x150);
	const FLT x225 = x39 * x170;
	const FLT x226 = (x214 * x180) + (-1 * x179 * x183) + (x182 * x151);
	const FLT x227 = x226 * x185;
	const FLT x228 = 2 * x218;
	const FLT x229 = 2 * x199;
	const FLT x230 = x221 * x185;
	const FLT x231 =
		(x179 * x198) + (-1 * x205 * x180) + (x200 * x182) + (-1 * x201 * x179) + (x71 * x180) + (-1 * x182 * x195);
	const FLT x232 = 2 * x226;
	const FLT x233 = (x190 * x157) + (x206 * x225) + (-1 * x230 * x178) + (-1 * x232 * x201) + (x227 * x178) +
					 (-1 * x212 * x225) + (x201 * x222) + (x224 * x197) + (-1 * x215 * x190) + (-1 * x231 * x197) +
					 (-1 * x228 * x169) + (x215 * x229);
	const FLT x234 =
		x155 + (-1 * (x179 + (2 * ((-1 * x215 * x181) + (x226 * x183))))) + (2 * ((-1 * x36 * x218) + (x221 * x183)));
	const FLT x235 = 2 * x234;
	const FLT x236 =
		x150 + (-1 * (x180 + (2 * ((-1 * x214 * x226) + (x181 * x184))))) + (2 * ((-1 * x214 * x221) + (x36 * x209)));
	const FLT x237 = (*error_model).BSD0.tilt + (*_x0).BSD0.tilt;
	const FLT x238 = x234 * x234;
	const FLT x239 = (x219 * x219) + x238;
	const FLT x240 = 1.0 / 2.0 * x237 * x236 * (1. / (x239 * sqrt(x239)));
	const FLT x241 = x31 * x227;
	const FLT x242 = (-1 * x229 * x184) + (x210 * x169) + (-1 * x71 * x222) + (-1 * x241 * x177) + (x204 * x221) +
					 (x186 * x189) + (-1 * x207 * x224) + (x231 * x207) + (-1 * x208 * x190) + (-1 * x202 * x225) +
					 (x71 * x232) + (x225 * x196);
	const FLT x243 = 1. / sqrt(x239);
	const FLT x244 = x237 * x243;
	const FLT x245 = x236 * x236;
	const FLT x246 = 1. / x239;
	const FLT x247 = 1. / sqrt(1 + (-1 * (x237 * x237) * x245 * x246));
	const FLT x248 = 1. / x238;
	const FLT x249 = x219 * x248;
	const FLT x250 = 1. / x234;
	const FLT x251 = x238 * x246;
	const FLT x252 = (-1 * ((-1 * x217 * x250) + (x233 * x249)) * x251) +
					 (-1 * x247 * ((x242 * x244) + (-1 * ((x233 * x235) + (x217 * x220)) * x240)));
	const FLT x253 = -1 * x234;
	const FLT x254 = 1.5707963267949 + (*_x0).BSD0.gibpha + (-1 * asin(x236 * x244)) + (*error_model).BSD0.gibpha +
					 (-1 * ((*error_model).BSD0.phase + (*_x0).BSD0.phase)) + (-1 * atan2(x219, x253));
	const FLT x255 = sin(x254) * ((*error_model).BSD0.gibmag + (*_x0).BSD0.gibmag);
	const FLT x256 = x236 * x248;
	const FLT x257 = atan2(x236, x253);
	const FLT x258 = 2 * (1. / (x238 + x245)) * x238 * x257 * ((*error_model).BSD0.curve + (*_x0).BSD0.curve);
	const FLT x259 = (-1 * x47) + x46;
	const FLT x260 = -1 * x42;
	const FLT x261 = x260 + x41;
	const FLT x262 = 2 * x24;
	const FLT x263 = x55 + x56;
	const FLT x264 = -1 * x52;
	const FLT x265 = x51 + x264;
	const FLT x266 = (x59 * x259) + (x58 * x265) + (x262 * x261) + (x54 * x263);
	const FLT x267 = x21 * x266;
	const FLT x268 = x18 * x266;
	const FLT x269 = x24 * x266;
	const FLT x270 = (x269 * x166) + (-1 * x45 * x268) + (x27 * x263) + (-1 * x63 * x266) + (x30 * x259) +
					 (-1 * x28 * x261) + (-1 * x267 * x172) + (x29 * x265);
	const FLT x271 = x39 * x270;
	const FLT x272 = x44 * x267;
	const FLT x273 = x62 * x266;
	const FLT x274 = x26 * x259;
	const FLT x275 = (x272 * (*_x0).Lighthouse.Rot[3]) + (-1 * x274 * (*_x0).Lighthouse.Rot[3]) + (-1 * x268 * x166) +
					 (-1 * x45 * x269) + (x28 * x265) + (-1 * x273 * (*_x0).Lighthouse.Rot[1]) + (x30 * x263) +
					 (x29 * x261);
	const FLT x276 = (x29 * x263) + (x268 * x161) + (-1 * x266 * x160) + (-1 * x274 * (*_x0).Lighthouse.Rot[2]) +
					 (x269 * x172) + (-1 * x27 * x265) + (-1 * x30 * x261) + (x272 * (*_x0).Lighthouse.Rot[2]);
	const FLT x277 = (x28 * x263) + (-1 * x273 * (*_x0).Lighthouse.Rot[2]) + (-1 * x30 * x265) + (-1 * x269 * x161) +
					 (-1 * x45 * x267) + (x268 * x172) + (x27 * x261) + (x274 * (*_x0).Lighthouse.Rot[0]);
	const FLT x278 = (x277 * x176) + (x270 * x171) + (x275 * x170) + (x276 * x164);
	const FLT x279 = x33 * x278;
	const FLT x280 = x39 * x277;
	const FLT x281 = x278 * x185;
	const FLT x282 = x36 * x281;
	const FLT x283 = 1.0 / 2.0 * x282;
	const FLT x284 = x281 * x193;
	const FLT x285 = x203 * x278;
	const FLT x286 = 1.0 / 2.0 * x285;
	const FLT x287 =
		(x275 * x156) + (-1 * x284 * x150) + (x276 * x192) + (-1 * x283 * x155) + (-1 * x270 * x154) + (x286 * x153);
	const FLT x288 = 1.0 / 2.0 * x279;
	const FLT x289 = x39 * x275;
	const FLT x290 =
		(x283 * x150) + (-1 * x288 * x187) + (-1 * x289 * x150) + (x276 * x156) + (x277 * x154) + (-1 * x284 * x155);
	const FLT x291 = x39 * x276;
	const FLT x292 =
		(x289 * x179) + (-1 * x283 * x179) + (-1 * x271 * x182) + (-1 * x284 * x180) + (x291 * x180) + (x286 * x182);
	const FLT x293 = x288 * x185;
	const FLT x294 =
		(x280 * x182) + (-1 * x289 * x180) + (-1 * x293 * x182) + (-1 * x284 * x179) + (x283 * x180) + (x291 * x179);
	const FLT x295 = (-1 * x210 * x277) + (x271 * x158) + (x213 * x280) + (x207 * x287) + (x211 * x279) +
					 (-1 * x290 * x197) + (x294 * x197) + (-1 * x207 * x292) + (-1 * x285 * x157) + (x215 * x285) +
					 (-1 * x216 * x271) + (-1 * x279 * x186);
	const FLT x296 =
		(-1 * x284 * x182) + (x293 * x179) + (-1 * x286 * x180) + (x291 * x182) + (-1 * x280 * x179) + (x271 * x180);
	const FLT x297 = x36 * x215;
	const FLT x298 = 1.0 / 2.0 * x150;
	const FLT x299 =
		(x271 * x150) + (-1 * x284 * x153) + (-1 * x298 * x285) + (x288 * x223) + (x276 * x154) + (-1 * x277 * x156);
	const FLT x300 = x36 * x157;
	const FLT x301 = x33 * x227;
	const FLT x302 = (x280 * x222) + (-1 * x287 * x225) + (-1 * x289 * x158) + (-1 * x296 * x197) + (x299 * x197) +
					 (x216 * x289) + (x292 * x225) + (-1 * x297 * x281) + (x281 * x300) + (-1 * x230 * x279) +
					 (-1 * x232 * x280) + (x278 * x301);
	const FLT x303 = (x36 * x278 * x186) + (x285 * x221) + (x232 * x271) + (-1 * x271 * x222) + (-1 * x207 * x299) +
					 (x207 * x296) + (-1 * x294 * x225) + (x210 * x275) + (-1 * x213 * x289) + (-1 * x278 * x241) +
					 (-1 * x208 * x282) + (x290 * x225);
	const FLT x304 = (-1 * ((-1 * x295 * x250) + (x249 * x302)) * x251) +
					 (-1 * x247 * ((x244 * x303) + (-1 * ((x235 * x302) + (x295 * x220)) * x240)));
	const FLT x305 = x50 + x264;
	const FLT x306 = x40 + x260;
	const FLT x307 = (x49 * x18) + (x59 * x305) + (x57 * x262) + (x54 * x306);
	const FLT x308 = x18 * x307;
	const FLT x309 = x21 * x307;
	const FLT x310 = x24 * x307;
	const FLT x311 = x168 + (-1 * x167) + (x310 * x166) + (x30 * x305) + (x27 * x306) + (-1 * x63 * x307) +
					 (-1 * x45 * x308) + (-1 * x309 * x172);
	const FLT x312 = x44 * x308;
	const FLT x313 = (x309 * x166) + (x29 * x306) + (-1 * x28 * x305) + x173 + (-1 * x307 * x160) + (x310 * x172) +
					 (x312 * (*_x0).Lighthouse.Rot[3]) + (-1 * x174);
	const FLT x314 = x62 * x307;
	const FLT x315 = (-1 * x27 * x305) + (-1 * x45 * x310) + x69 + (-1 * x314 * (*_x0).Lighthouse.Rot[1]) +
					 (x30 * x306) + (-1 * x312 * (*_x0).Lighthouse.Rot[2]) + (x309 * x161) + x64;
	const FLT x316 = (x29 * x305) + (-1 * x314 * (*_x0).Lighthouse.Rot[2]) + (x28 * x306) + x159 +
					 (x312 * (*_x0).Lighthouse.Rot[1]) + (-1 * x310 * x161) + (-1 * x45 * x309) + x162;
	const FLT x317 = (x316 * x176) + (x311 * x171) + (x313 * x164) + (x315 * x170);
	const FLT x318 = x317 * x185;
	const FLT x319 = x33 * x318;
	const FLT x320 = x31 * x318;
	const FLT x321 = x39 * x316;
	const FLT x322 = x208 * x318;
	const FLT x323 = x36 * x318;
	const FLT x324 = 1.0 / 2.0 * x323;
	const FLT x325 = x318 * x193;
	const FLT x326 = 1.0 / 2.0 * x320;
	const FLT x327 =
		(x326 * x153) + (-1 * x325 * x150) + (x313 * x192) + (-1 * x324 * x155) + (x315 * x156) + (-1 * x311 * x154);
	const FLT x328 = x39 * x311;
	const FLT x329 = 1.0 / 2.0 * x319;
	const FLT x330 =
		(-1 * x325 * x155) + (x298 * x323) + (-1 * x315 * x192) + (x316 * x154) + (-1 * x329 * x153) + (x313 * x156);
	const FLT x331 = x39 * x313;
	const FLT x332 = x39 * x315;
	const FLT x333 =
		(-1 * x332 * x180) + (x324 * x180) + (-1 * x329 * x182) + (-1 * x325 * x179) + (x321 * x182) + (x331 * x179);
	const FLT x334 =
		((x326 * x182) + (x332 * x179) + (-1 * x328 * x182) + (-1 * x325 * x180) + (x331 * x180) + (-1 * x324 * x179)) *
		x39;
	const FLT x335 = (-1 * x334 * x171) + (x213 * x321) + (-1 * x319 * x184) + (-1 * x320 * x157) + (x228 * x311) +
					 (x33 * x322) + (x207 * x327) + (-1 * x210 * x316) + (-1 * x216 * x328) + (-1 * x330 * x197) +
					 (x215 * x320) + (x333 * x197);
	const FLT x336 =
		(x328 * x180) + (x329 * x179) + (-1 * x326 * x180) + (-1 * x321 * x179) + (x331 * x182) + (-1 * x325 * x182);
	const FLT x337 =
		(-1 * x326 * x150) + (x311 * x192) + (-1 * x316 * x156) + (-1 * x325 * x153) + (x313 * x154) + (x329 * x155);
	const FLT x338 = (x337 * x197) + (x334 * x170) + (-1 * x228 * x315) + (-1 * x225 * x327) + (-1 * x336 * x197) +
					 (x301 * x317) + (x216 * x332) + (x222 * x321) + (-1 * x221 * x319) + (-1 * x297 * x318) +
					 (x300 * x318) + (-1 * x232 * x321);
	const FLT x339 = (-1 * x225 * x333) + (-1 * x207 * x337) + (-1 * x213 * x332) + (x232 * x328) + (x323 * x184) +
					 (-1 * x222 * x328) + (-1 * x241 * x317) + (-1 * x36 * x322) + (x225 * x330) + (x210 * x315) +
					 (x221 * x320) + (x207 * x336);
	const FLT x340 = (-1 * ((-1 * x250 * x335) + (x249 * x338)) * x251) +
					 (-1 * x247 * ((x244 * x339) + (-1 * ((x235 * x338) + (x220 * x335)) * x240)));
	const FLT x341 = 1. / x38;
	const FLT x342 = x31 * x341;
	const FLT x343 = x342 * x164;
	const FLT x344 = x33 * x341;
	const FLT x345 = x344 * x170;
	const FLT x346 = -1 * x345;
	const FLT x347 = x346 + x343;
	const FLT x348 = x342 * x170;
	const FLT x349 = -1 * x348;
	const FLT x350 = x341 * x164;
	const FLT x351 = x33 * x350;
	const FLT x352 = -1 * x351;
	const FLT x353 = x352 + x349;
	const FLT x354 = 2 * x341;
	const FLT x355 = x34 * x354;
	const FLT x356 = x32 * x354;
	const FLT x357 = -1 + x356 + x355;
	const FLT x358 = (-1 * ((-1 * x250 * x357) + (x249 * x353)) * x251) +
					 (-1 * x247 * ((x244 * x347) + (-1 * ((x235 * x353) + (x220 * x357)) * x240)));
	const FLT x359 = -1 * x343;
	const FLT x360 = x359 + x346;
	const FLT x361 = x36 * x350;
	const FLT x362 = x344 * x171;
	const FLT x363 = -1 * x362;
	const FLT x364 = x363 + x361;
	const FLT x365 = x37 * x354;
	const FLT x366 = -1 + x365;
	const FLT x367 = x366 + x356;
	const FLT x368 = (-1 * ((-1 * x250 * x360) + (x249 * x364)) * x251) +
					 (-1 * x247 * ((x244 * x367) + (-1 * ((x235 * x364) + (x220 * x360)) * x240)));
	const FLT x369 = x349 + x351;
	const FLT x370 = x366 + x355;
	const FLT x371 = -1 * x361;
	const FLT x372 = x371 + x363;
	const FLT x373 = (-1 * ((-1 * x250 * x369) + (x249 * x370)) * x251) +
					 (-1 * x247 * ((x244 * x372) + (-1 * ((x235 * x370) + (x220 * x369)) * x240)));
	const FLT x374 = x341 * x148;
	const FLT x375 = -1 * x34 * x374;
	const FLT x376 = (-1 * x32 * x374) + x149;
	const FLT x377 = x376 + x375;
	const FLT x378 = x35 * x374;
	const FLT x379 = x33 * x378;
	const FLT x380 = x31 * x374;
	const FLT x381 = x36 * x380;
	const FLT x382 = x381 + x379;
	const FLT x383 = x33 * x36 * x374;
	const FLT x384 = x35 * x380;
	const FLT x385 = (-1 * x384) + x383;
	const FLT x386 = (-1 * ((-1 * x250 * x377) + (x249 * x382)) * x251) +
					 (-1 * x247 * ((x244 * x385) + (-1 * ((x235 * x382) + (x220 * x377)) * x240)));
	const FLT x387 = -1 * x37 * x374;
	const FLT x388 = x376 + x387;
	const FLT x389 = x33 * x380;
	const FLT x390 = x36 * x378;
	const FLT x391 = (-1 * x390) + x389;
	const FLT x392 = x383 + x384;
	const FLT x393 = (-1 * ((-1 * x250 * x392) + (x249 * x391)) * x251) +
					 (-1 * x247 * ((x244 * x388) + (-1 * ((x235 * x391) + (x220 * x392)) * x240)));
	const FLT x394 = (-1 * x379) + x381;
	const FLT x395 = x375 + x149 + x387;
	const FLT x396 = x389 + x390;
	const FLT x397 = (-1 * ((-1 * x250 * x394) + (x249 * x395)) * x251) +
					 (-1 * x247 * ((x244 * x396) + (-1 * ((x235 * x395) + (x220 * x394)) * x240)));
	const FLT x398 = 0.5 * x88;
	const FLT x399 = -0.5 * x90;
	const FLT x400 = x399 + (-1 * x398);
	const FLT x401 = -1 * x400 * x129;
	const FLT x402 = 0.5 * x85;
	const FLT x403 = 0.5 * x80;
	const FLT x404 = x403 + x402;
	const FLT x405 = 2 * x97;
	const FLT x406 = 0.5 * x92;
	const FLT x407 = -1 * x406;
	const FLT x408 = 0.5 * x93;
	const FLT x409 = x408 + x407;
	const FLT x410 = 2 * x91;
	const FLT x411 = 2 * x94;
	const FLT x412 = 0.5 * x96;
	const FLT x413 = -1 * x412;
	const FLT x414 = 0.5 * x95;
	const FLT x415 = x414 + x413;
	const FLT x416 = 2 * x86;
	const FLT x417 = (x415 * x416) + (x400 * x411) + (x405 * x404) + (x409 * x410);
	const FLT x418 = 1.0 / 2.0 * (1. / (x98 * sqrt(x98)));
	const FLT x419 = x91 * x418;
	const FLT x420 = x417 * x419;
	const FLT x421 = x418 * (*_x0).Object.Pose.Rot[1];
	const FLT x422 = x94 * x417;
	const FLT x423 = x86 * x417;
	const FLT x424 = x418 * (*_x0).Object.Pose.Rot[2];
	const FLT x425 = x404 * x100;
	const FLT x426 = x97 * x418;
	const FLT x427 = x417 * x426;
	const FLT x428 = (x409 * x102) + (-1 * x427 * (*_x0).Object.Pose.Rot[3]) + x425 +
					 (-1 * x420 * (*_x0).Object.Pose.Rot[0]) + x401 + (x422 * x421) + (-1 * x423 * x424) +
					 (x415 * x101);
	const FLT x429 = x108 * x127;
	const FLT x430 = -1 * x404 * x129;
	const FLT x431 = x418 * (*_x0).Object.Pose.Rot[3];
	const FLT x432 = x400 * x100;
	const FLT x433 = x418 * (*_x0).Object.Pose.Rot[0];
	const FLT x434 = (-1 * x423 * x433) + (x427 * (*_x0).Object.Pose.Rot[1]) + (-1 * x432) + x430 + (-1 * x409 * x101) +
					 (x415 * x102) + (x422 * x431) + (x420 * (*_x0).Object.Pose.Rot[2]);
	const FLT x435 = x72 * x127;
	const FLT x436 = x404 * x102;
	const FLT x437 = x400 * x101;
	const FLT x438 = (-1 * x422 * x424) + (-1 * x427 * (*_x0).Object.Pose.Rot[0]) + x437 + (x415 * x129) +
					 (-1 * x409 * x100) + x436 + (-1 * x423 * x421) + (x420 * (*_x0).Object.Pose.Rot[3]);
	const FLT x439 = x111 * x127;
	const FLT x440 = x404 * x101;
	const FLT x441 = x400 * x102;
	const FLT x442 = (-1 * x440) + (x415 * x100) + (-1 * x420 * (*_x0).Object.Pose.Rot[1]) + x441 + (-1 * x423 * x431) +
					 (x409 * x129) + (-1 * x422 * x433) + (x427 * (*_x0).Object.Pose.Rot[2]);
	const FLT x443 = (x442 * x133) + (-1 * x438 * x439) + (x429 * x428) + (x434 * x435);
	const FLT x444 = (-1 * x428 * x439) + (-1 * x429 * x438) + (x434 * x133) + (-1 * x435 * x442);
	const FLT x445 = (-1 * x429 * x442) + (x428 * x133) + (x435 * x438) + (x434 * x439);
	const FLT x446 = (x445 * sensor_pt[2]) + (-1 * x443 * sensor_pt[1]) + (x444 * sensor_pt[0]);
	const FLT x447 = 2 * x146;
	const FLT x448 = (-1 * x428 * x435) + (x439 * x442) + (x429 * x434) + (x438 * x133);
	const FLT x449 = (x448 * sensor_pt[1]) + (-1 * x445 * sensor_pt[0]) + (x444 * sensor_pt[2]);
	const FLT x450 = 2 * x141;
	const FLT x451 = 2 * x144;
	const FLT x452 = 2 * x147;
	const FLT x453 = (x452 * x443) + (-1 * x451 * x448) + (x447 * x446) + (-1 * x450 * x449);
	const FLT x454 = 2 * x445;
	const FLT x455 = 2 * x143;
	const FLT x456 = 2 * x152;
	const FLT x457 = (x443 * sensor_pt[0]) + (-1 * x448 * sensor_pt[2]) + (x444 * sensor_pt[1]);
	const FLT x458 = (x456 * x448) + (-1 * x454 * x147) + (x457 * x450) + (-1 * x455 * x446);
	const FLT x459 = x39 * x458;
	const FLT x460 = (x454 * x144) + (-1 * x457 * x447) + (x455 * x449) + (-1 * x456 * x443);
	const FLT x461 = x39 * x460;
	const FLT x462 = (x453 * x151) + (-1 * x31 * x461) + (x36 * x459);
	const FLT x463 = x39 * x453;
	const FLT x464 = x39 * ((x458 * x151) + (x33 * x461) + (-1 * x36 * x463));
	const FLT x465 = x460 + (x462 * x207) + (-1 * x464 * x176);
	const FLT x466 = (-1 * x33 * x459) + (x31 * x463) + (x460 * x151);
	const FLT x467 = x458 + (x466 * x197) + (-1 * x462 * x225);
	const FLT x468 = x453 + (x464 * x170) + (-1 * x466 * x207);
	const FLT x469 = (-1 * ((-1 * x465 * x250) + (x467 * x249)) * x251) +
					 (-1 * x247 * ((x468 * x244) + (-1 * ((x467 * x235) + (x465 * x220)) * x240)));
	const FLT x470 = -1 * x408;
	const FLT x471 = x470 + x407;
	const FLT x472 = -1 * x414;
	const FLT x473 = x413 + x472;
	const FLT x474 = (-1 * x403) + x402;
	const FLT x475 = x398 + x399;
	const FLT x476 = (x475 * x416) + (x474 * x410) + (x471 * x405) + (x473 * x411);
	const FLT x477 = x476 * x426;
	const FLT x478 = x94 * x476;
	const FLT x479 = x476 * x419;
	const FLT x480 = x86 * x476;
	const FLT x481 = (-1 * x480 * x433) + (-1 * x474 * x101) + (x477 * (*_x0).Object.Pose.Rot[1]) +
					 (x479 * (*_x0).Object.Pose.Rot[2]) + (-1 * x471 * x129) + (x475 * x102) + (x478 * x431) +
					 (-1 * x473 * x100);
	const FLT x482 = x478 * x418;
	const FLT x483 = (x471 * x102) + (x473 * x101) + (-1 * x482 * (*_x0).Object.Pose.Rot[2]) + (x475 * x129) +
					 (-1 * x480 * x421) + (-1 * x477 * (*_x0).Object.Pose.Rot[0]) + (-1 * x474 * x100) +
					 (x479 * (*_x0).Object.Pose.Rot[3]);
	const FLT x484 = (-1 * x477 * (*_x0).Object.Pose.Rot[3]) + (x474 * x102) + (x471 * x100) +
					 (-1 * x479 * (*_x0).Object.Pose.Rot[0]) + (-1 * x473 * x129) + (x482 * (*_x0).Object.Pose.Rot[1]) +
					 (-1 * x480 * x424) + (x475 * x101);
	const FLT x485 = (x477 * (*_x0).Object.Pose.Rot[2]) + (x474 * x129) + (-1 * x480 * x431) + (-1 * x478 * x433) +
					 (-1 * x471 * x101) + (x473 * x102) + (-1 * x479 * (*_x0).Object.Pose.Rot[1]) + (x475 * x100);
	const FLT x486 = (x485 * x439) + (-1 * x484 * x435) + (x481 * x429) + (x483 * x133);
	const FLT x487 = (-1 * x485 * x435) + (-1 * x484 * x439) + (-1 * x483 * x429) + (x481 * x133);
	const FLT x488 = (x485 * x133) + (x481 * x435) + (x484 * x429) + (-1 * x483 * x439);
	const FLT x489 = (x488 * sensor_pt[0]) + (-1 * x486 * sensor_pt[2]) + (x487 * sensor_pt[1]);
	const FLT x490 = (x484 * x133) + (x483 * x435) + (-1 * x485 * x429) + (x481 * x439);
	const FLT x491 = (x486 * sensor_pt[1]) + (-1 * x490 * sensor_pt[0]) + (x487 * sensor_pt[2]);
	const FLT x492 = (-1 * x456 * x488) + (-1 * x489 * x447) + (x491 * x455) + (x490 * x451);
	const FLT x493 = (x490 * sensor_pt[2]) + (x487 * sensor_pt[0]) + (-1 * x488 * sensor_pt[1]);
	const FLT x494 = (-1 * x451 * x486) + (x452 * x488) + (-1 * x491 * x450) + (x493 * x447);
	const FLT x495 = (x456 * x486) + (x450 * x489) + (-1 * x493 * x455) + (-1 * x490 * x452);
	const FLT x496 = (x495 * x151) + (x492 * x183) + (-1 * x494 * x181);
	const FLT x497 = (-1 * x492 * x214) + (x495 * x181) + (x494 * x151);
	const FLT x498 = x492 + (-1 * x496 * x197) + (x497 * x207);
	const FLT x499 = (-1 * x495 * x183) + (x494 * x214) + (x492 * x151);
	const FLT x500 = x495 + (x499 * x197) + (-1 * x497 * x225);
	const FLT x501 = x494 + (x496 * x225) + (-1 * x499 * x207);
	const FLT x502 = (-1 * ((-1 * x498 * x250) + (x500 * x249)) * x251) +
					 (-1 * x247 * ((x501 * x244) + (-1 * ((x500 * x235) + (x498 * x220)) * x240)));
	const FLT x503 = x412 + x472;
	const FLT x504 = x406 + x470;
	const FLT x505 = (x416 * x504) + (x404 * x411) + (x410 * x503) + (x400 * x405);
	const FLT x506 = x419 * x505;
	const FLT x507 = x94 * x505;
	const FLT x508 = x86 * x505;
	const FLT x509 = x426 * x505;
	const FLT x510 = (x509 * (*_x0).Object.Pose.Rot[2]) + (x503 * x129) + (-1 * x431 * x508) + x436 +
					 (-1 * x433 * x507) + (x504 * x100) + (-1 * x506 * (*_x0).Object.Pose.Rot[1]) + (-1 * x437);
	const FLT x511 = (x506 * (*_x0).Object.Pose.Rot[3]) + x440 + (-1 * x424 * x507) +
					 (-1 * x509 * (*_x0).Object.Pose.Rot[0]) + (x504 * x129) + (-1 * x503 * x100) + x441 +
					 (-1 * x421 * x508);
	const FLT x512 = (x506 * (*_x0).Object.Pose.Rot[2]) + x401 + (x431 * x507) + (-1 * x503 * x101) +
					 (-1 * x433 * x508) + (x504 * x102) + (x509 * (*_x0).Object.Pose.Rot[1]) + (-1 * x425);
	const FLT x513 = (x503 * x102) + (x421 * x507) + (-1 * x509 * (*_x0).Object.Pose.Rot[3]) + x430 +
					 (-1 * x424 * x508) + (x504 * x101) + (-1 * x506 * (*_x0).Object.Pose.Rot[0]) + x432;
	const FLT x514 = (x513 * x133) + (x439 * x512) + (-1 * x429 * x510) + (x435 * x511);
	const FLT x515 = (x510 * x133) + (-1 * x439 * x511) + (x429 * x513) + (x435 * x512);
	const FLT x516 = (-1 * x439 * x513) + (-1 * x435 * x510) + (-1 * x429 * x511) + (x512 * x133);
	const FLT x517 = (x514 * sensor_pt[2]) + (-1 * x515 * sensor_pt[1]) + (x516 * sensor_pt[0]);
	const FLT x518 = (x439 * x510) + (-1 * x435 * x513) + (x429 * x512) + (x511 * x133);
	const FLT x519 = (x515 * sensor_pt[0]) + (-1 * x518 * sensor_pt[2]) + (x516 * sensor_pt[1]);
	const FLT x520 = (x450 * x519) + (x456 * x518) + (-1 * x452 * x514) + (-1 * x455 * x517);
	const FLT x521 = x39 * x520;
	const FLT x522 = (x516 * sensor_pt[2]) + (x518 * sensor_pt[1]) + (-1 * x514 * sensor_pt[0]);
	const FLT x523 = (x447 * x517) + (-1 * x451 * x518) + (-1 * x450 * x522) + (x452 * x515);
	const FLT x524 = (x451 * x514) + (-1 * x447 * x519) + (x455 * x522) + (-1 * x456 * x515);
	const FLT x525 = x39 * x524;
	const FLT x526 = (x36 * x521) + (-1 * x31 * x525) + (x523 * x151);
	const FLT x527 = (x520 * x151) + (x33 * x525) + (-1 * x523 * x181);
	const FLT x528 = (x526 * x207) + x524 + (-1 * x527 * x197);
	const FLT x529 = (-1 * x33 * x521) + (x524 * x151) + (x523 * x214);
	const FLT x530 = x520 + (x529 * x197) + (-1 * x526 * x225);
	const FLT x531 = x523 + (x527 * x225) + (-1 * x529 * x207);
	const FLT x532 = (-1 * ((-1 * x528 * x250) + (x530 * x249)) * x251) +
					 (-1 * x247 * ((x531 * x244) + (-1 * ((x530 * x235) + (x528 * x220)) * x240)));
	const FLT x533 = x359 + x345;
	const FLT x534 = x348 + x351;
	const FLT x535 = -1 * x355;
	const FLT x536 = 1 + (-1 * x356);
	const FLT x537 = x536 + x535;
	const FLT x538 = (-1 * ((-1 * x537 * x250) + (x534 * x249)) * x251) +
					 (-1 * x247 * ((x533 * x244) + (-1 * ((x534 * x235) + (x537 * x220)) * x240)));
	const FLT x539 = x345 + x343;
	const FLT x540 = x371 + x362;
	const FLT x541 = -1 * x365;
	const FLT x542 = x536 + x541;
	const FLT x543 = (-1 * ((-1 * x539 * x250) + (x540 * x249)) * x251) +
					 (-1 * x247 * ((x542 * x244) + (-1 * ((x540 * x235) + (x539 * x220)) * x240)));
	const FLT x544 = x352 + x348;
	const FLT x545 = 1 + x541 + x535;
	const FLT x546 = x362 + x361;
	const FLT x547 = (-1 * ((-1 * x544 * x250) + (x545 * x249)) * x251) +
					 (-1 * x247 * ((x546 * x244) + (-1 * ((x545 * x235) + (x544 * x220)) * x240)));
	const FLT x548 = -1 * x128;
	const FLT x549 = 1. / (x114 * sqrt(x114));
	const FLT x550 = dt * dt * dt;
	const FLT x551 = x549 * x550;
	const FLT x552 = x72 * x551;
	const FLT x553 = x552 * x108;
	const FLT x554 = x553 * x136;
	const FLT x555 = x108 * x108 * x108;
	const FLT x556 = dt * dt * dt * dt;
	const FLT x557 = 2 * x118 * (1. / (x114 * x114));
	const FLT x558 = x557 * x556;
	const FLT x559 = 1.0 * x117 * x121;
	const FLT x560 = x549 * x559;
	const FLT x561 = x556 * x560;
	const FLT x562 = x561 * x108;
	const FLT x563 = x558 * x108;
	const FLT x564 = x108 * x105;
	const FLT x565 = 2 * x120;
	const FLT x566 = x559 * x125;
	const FLT x567 = (x555 * x561) + (x562 * x112) + (-1 * x558 * x555) + (-1 * x563 * x112) + (x562 * x106) +
					 (x564 * x565) + (-1 * x564 * x566) + (-1 * x563 * x106);
	const FLT x568 = 1.0 / 2.0 * (1. / (x122 * sqrt(x122)));
	const FLT x569 = x568 * x121;
	const FLT x570 = x569 * x567;
	const FLT x571 = x124 * x130;
	const FLT x572 = 0.5 * x125;
	const FLT x573 = x572 * x564;
	const FLT x574 = x568 * x117 * x126;
	const FLT x575 = x574 * x104;
	const FLT x576 = x567 * x108;
	const FLT x577 = 0.5 * x550 * x119;
	const FLT x578 = x577 * x139;
	const FLT x579 = x578 * x108;
	const FLT x580 = x72 * x579;
	const FLT x581 = x574 * x135;
	const FLT x582 = x72 * x567;
	const FLT x583 = x104 * x124;
	const FLT x584 = x551 * x109;
	const FLT x585 = x574 * x111;
	const FLT x586 = x585 * x132;
	const FLT x587 = x577 * x145;
	const FLT x588 = x577 * x134;
	const FLT x589 = x588 * x108;
	const FLT x590 = x124 * x132;
	const FLT x591 = x551 * x108;
	const FLT x592 = x591 * x111;
	const FLT x593 = (-1 * x592 * x590) + (x589 * x111);
	const FLT x594 = x593 + (x583 * x584) + (-1 * x587 * x109) + (-1 * x581 * x582) + x580 + (-1 * x571 * x573) +
					 (-1 * x554) + x548 + (-1 * x570 * x130) + (-1 * x586 * x567) + (x576 * x575);
	const FLT x595 = x587 * x108;
	const FLT x596 = x595 * x111;
	const FLT x597 = x571 * x553;
	const FLT x598 = x585 * x104;
	const FLT x599 = x583 * x111;
	const FLT x600 = x599 * x591;
	const FLT x601 = x577 * x142;
	const FLT x602 = x601 * x108;
	const FLT x603 = x72 * x602;
	const FLT x604 = x569 * x135;
	const FLT x605 = x574 * x130;
	const FLT x606 = x72 * x605;
	const FLT x607 = x574 * x132;
	const FLT x608 = (-1 * x603) + x597 + (-1 * x598 * x567) + (-1 * x604 * x567) + (-1 * x607 * x576) + (x606 * x567) +
					 x596 + (-1 * x600) + (x588 * x109) + (-1 * x590 * x584) + x140 + (-1 * x573 * x136);
	const FLT x609 = -1 * x137;
	const FLT x610 = x592 * x571;
	const FLT x611 = x585 * x130;
	const FLT x612 = x602 * x111;
	const FLT x613 = x581 * x108;
	const FLT x614 = (x583 * x553) + (-1 * x72 * x595);
	const FLT x615 = x614 + (x584 * x136) + (-1 * x570 * x132) + (x613 * x567) + (-1 * x612) + (-1 * x578 * x109) +
					 (x575 * x582) + x609 + (-1 * x590 * x573) + x610 + (x611 * x567);
	const FLT x616 = x585 * x135;
	const FLT x617 = x111 * x136;
	const FLT x618 = (x617 * x591) + (-1 * x579 * x111);
	const FLT x619 = (-1 * x590 * x553) + (x72 * x589);
	const FLT x620 = x618 + x619 + (-1 * x605 * x576) + (x616 * x567) + (-1 * x570 * x104) + x131 + (x601 * x109) +
					 (-1 * x573 * x583) + (-1 * x571 * x584) + (-1 * x607 * x582);
	const FLT x621 = (x620 * sensor_pt[0]) + (-1 * x608 * sensor_pt[2]) + (x615 * sensor_pt[1]);
	const FLT x622 = (x594 * sensor_pt[2]) + (-1 * x620 * sensor_pt[1]) + (x615 * sensor_pt[0]);
	const FLT x623 = (-1 * x455 * x622) + (x450 * x621) + (-1 * x452 * x594) + (x456 * x608);
	const FLT x624 = x39 * x623;
	const FLT x625 = (x608 * sensor_pt[1]) + (-1 * x594 * sensor_pt[0]) + (x615 * sensor_pt[2]);
	const FLT x626 = (-1 * x451 * x608) + (x452 * x620) + (-1 * x450 * x625) + (x447 * x622);
	const FLT x627 = (x451 * x594) + (x455 * x625) + (-1 * x447 * x621) + (-1 * x456 * x620);
	const FLT x628 = x39 * x627;
	const FLT x629 = (x36 * x624) + (-1 * x31 * x628) + (x626 * x151);
	const FLT x630 = x39 * x626;
	const FLT x631 = x39 * ((x623 * x151) + (x33 * x628) + (-1 * x36 * x630));
	const FLT x632 = x627 + (x629 * x207) + (-1 * x631 * x176);
	const FLT x633 = (-1 * x33 * x624) + (x31 * x630) + (x627 * x151);
	const FLT x634 = x623 + (-1 * x629 * x225) + (x633 * x197);
	const FLT x635 = x626 + (-1 * x633 * x207) + (x631 * x170);
	const FLT x636 = (-1 * ((-1 * x632 * x250) + (x634 * x249)) * x251) +
					 (-1 * x247 * ((x635 * x244) + (-1 * ((x634 * x235) + (x632 * x220)) * x240)));
	const FLT x637 = x558 * x111;
	const FLT x638 = x561 * x111;
	const FLT x639 = x566 * x105;
	const FLT x640 = x556 * (x111 * x111 * x111);
	const FLT x641 = x565 * x105;
	const FLT x642 = (-1 * x640 * x557) + (x638 * x109) + (x638 * x106) + (-1 * x637 * x109) + (x641 * x111) +
					 (x640 * x560) + (-1 * x637 * x106) + (-1 * x639 * x111);
	const FLT x643 = x642 * x569;
	const FLT x644 = x551 * x112;
	const FLT x645 = x72 * x111;
	const FLT x646 = x645 * x578;
	const FLT x647 = x617 * x552;
	const FLT x648 = x577 * x112;
	const FLT x649 = x72 * x642;
	const FLT x650 = x572 * x105;
	const FLT x651 = x650 * x571;
	const FLT x652 = x642 * x108;
	const FLT x653 = (x652 * x575) + x646 + (-1 * x651 * x111) + x140 + (-1 * x647) + (-1 * x643 * x130) +
					 (-1 * x642 * x586) + (x648 * x134) + x600 + (-1 * x644 * x590) + (-1 * x596) + (-1 * x649 * x581);
	const FLT x654 = x650 * x111;
	const FLT x655 = x552 * x111;
	const FLT x656 = (x645 * x588) + (-1 * x655 * x590);
	const FLT x657 = x656 + (x644 * x136) + (-1 * x648 * x139) + x609 + (-1 * x605 * x652) + (x616 * x642) +
					 (-1 * x607 * x649) + (-1 * x654 * x583) + x612 + (-1 * x643 * x104) + (-1 * x610);
	const FLT x658 = x645 * x587;
	const FLT x659 = x599 * x552;
	const FLT x660 = -1 * x131;
	const FLT x661 = (-1 * x643 * x132) + (-1 * x654 * x590) + (x613 * x642) + x660 + x618 + (x649 * x575) +
					 (-1 * x648 * x142) + (x644 * x571) + (x611 * x642) + (-1 * x658) + x659;
	const FLT x662 = (x653 * sensor_pt[2]) + (-1 * x657 * sensor_pt[1]) + (x661 * sensor_pt[0]);
	const FLT x663 = (x655 * x571) + (-1 * x601 * x645);
	const FLT x664 = x663 + (-1 * x617 * x650) + (-1 * x642 * x598) + x593 + (x605 * x649) + (x648 * x145) +
					 (-1 * x644 * x583) + (-1 * x604 * x642) + (-1 * x607 * x652) + x128;
	const FLT x665 = (x657 * sensor_pt[0]) + (-1 * x664 * sensor_pt[2]) + (x661 * sensor_pt[1]);
	const FLT x666 = (x456 * x664) + (-1 * x452 * x653) + (x450 * x665) + (-1 * x455 * x662);
	const FLT x667 = x39 * x666;
	const FLT x668 = (x664 * sensor_pt[1]) + (-1 * x653 * sensor_pt[0]) + (x661 * sensor_pt[2]);
	const FLT x669 = (x447 * x662) + (-1 * x450 * x668) + (x452 * x657) + (-1 * x451 * x664);
	const FLT x670 = (x451 * x653) + (x455 * x668) + (-1 * x447 * x665) + (-1 * x456 * x657);
	const FLT x671 = (x36 * x667) + (-1 * x670 * x214) + (x669 * x151);
	const FLT x672 = x39 * x669;
	const FLT x673 = (x666 * x151) + (-1 * x36 * x672) + (x670 * x183);
	const FLT x674 = x670 + (x671 * x207) + (-1 * x673 * x197);
	const FLT x675 = x39 * ((x31 * x672) + (-1 * x33 * x667) + (x670 * x151));
	const FLT x676 = x666 + (x675 * x176) + (-1 * x671 * x225);
	const FLT x677 = x669 + (x673 * x225) + (-1 * x675 * x171);
	const FLT x678 = (-1 * ((-1 * x674 * x250) + (x676 * x249)) * x251) +
					 (-1 * x247 * ((x677 * x244) + (-1 * ((x676 * x235) + (x674 * x220)) * x240)));
	const FLT x679 = x72 * x561;
	const FLT x680 = x72 * x558;
	const FLT x681 = x72 * x72 * x72;
	const FLT x682 = (x679 * x112) + (x681 * x561) + (x679 * x109) + (-1 * x72 * x639) + (-1 * x680 * x109) +
					 (-1 * x680 * x112) + (-1 * x681 * x558) + (x72 * x641);
	const FLT x683 = x72 * x682;
	const FLT x684 = x72 * x650;
	const FLT x685 = x551 * x106;
	const FLT x686 = x682 * x569;
	const FLT x687 = x682 * x108;
	const FLT x688 = (x687 * x581) + (-1 * x686 * x132) + x554 + x548 + x663 + (-1 * x580) + (x683 * x575) +
					 (x611 * x682) + (-1 * x684 * x590) + (-1 * x587 * x106) + (x685 * x583);
	const FLT x689 = x614 + x656 + (x687 * x575) + (x578 * x106) + x137 + (-1 * x682 * x586) + (-1 * x686 * x130) +
					 (-1 * x72 * x651) + (-1 * x683 * x581) + (-1 * x685 * x136);
	const FLT x690 = x685 * x124;
	const FLT x691 = x619 + (-1 * x601 * x106) + (-1 * x682 * x598) + (x606 * x682) + (-1 * x686 * x135) + x660 +
					 (-1 * x607 * x687) + x658 + (-1 * x659) + (x690 * x130) + (-1 * x684 * x136);
	const FLT x692 = (x691 * sensor_pt[1]) + (x688 * sensor_pt[2]) + (-1 * x689 * sensor_pt[0]);
	const FLT x693 = x647 + (x616 * x682) + x603 + (-1 * x686 * x104) + (-1 * x646) + (-1 * x607 * x683) + (-1 * x597) +
					 (-1 * x690 * x132) + (x588 * x106) + (-1 * x684 * x583) + x140 + (-1 * x605 * x687);
	const FLT x694 = (x689 * sensor_pt[2]) + (-1 * x693 * sensor_pt[1]) + (x688 * sensor_pt[0]);
	const FLT x695 = (x447 * x694) + (x452 * x693) + (-1 * x450 * x692) + (-1 * x451 * x691);
	const FLT x696 = (x693 * sensor_pt[0]) + (-1 * x691 * sensor_pt[2]) + (x688 * sensor_pt[1]);
	const FLT x697 = (x450 * x696) + (-1 * x452 * x689) + (x456 * x691) + (-1 * x455 * x694);
	const FLT x698 = x39 * x697;
	const FLT x699 = (x451 * x689) + (-1 * x456 * x693) + (x455 * x692) + (-1 * x447 * x696);
	const FLT x700 = (-1 * x699 * x214) + (x695 * x151) + (x36 * x698);
	const FLT x701 = (x699 * x183) + (x697 * x151) + (-1 * x695 * x181);
	const FLT x702 = x699 + (x700 * x207) + (-1 * x701 * x197);
	const FLT x703 = (-1 * x33 * x698) + (x699 * x151) + (x695 * x214);
	const FLT x704 = x697 + (x703 * x197) + (-1 * x700 * x225);
	const FLT x705 = x695 + (x701 * x225) + (-1 * x703 * x207);
	const FLT x706 = (-1 * ((-1 * x702 * x250) + (x704 * x249)) * x251) +
					 (-1 * x247 * ((x705 * x244) + (-1 * ((x704 * x235) + (x702 * x220)) * x240)));
	const FLT x707 = -1 * dt * x355;
	const FLT x708 = -1 * dt * x356;
	const FLT x709 = x708 + dt + x707;
	const FLT x710 = dt * x351;
	const FLT x711 = dt * x348;
	const FLT x712 = x711 + x710;
	const FLT x713 = dt * x345;
	const FLT x714 = dt * x343;
	const FLT x715 = (-1 * x714) + x713;
	const FLT x716 = (-1 * ((-1 * x709 * x250) + (x712 * x249)) * x251) +
					 (-1 * x247 * ((x715 * x244) + (-1 * ((x712 * x235) + (x709 * x220)) * x240)));
	const FLT x717 = x713 + x714;
	const FLT x718 = dt * x362;
	const FLT x719 = dt * x361;
	const FLT x720 = (-1 * x719) + x718;
	const FLT x721 = (-1 * dt * x365) + dt;
	const FLT x722 = x721 + x708;
	const FLT x723 = (-1 * ((-1 * x717 * x250) + (x720 * x249)) * x251) +
					 (-1 * x247 * ((x722 * x244) + (-1 * ((x720 * x235) + (x717 * x220)) * x240)));
	const FLT x724 = (-1 * x710) + x711;
	const FLT x725 = x721 + x707;
	const FLT x726 = x718 + x719;
	const FLT x727 = (-1 * ((-1 * x724 * x250) + (x725 * x249)) * x251) +
					 (-1 * x247 * ((x726 * x244) + (-1 * ((x725 * x235) + (x724 * x220)) * x240)));
	const FLT x728 = x236 * x243 * x247;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						x252 + (x252 * x255) + (((x233 * x256) + (-1 * x250 * x242)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						x304 + (x255 * x304) + (((x256 * x302) + (-1 * x250 * x303)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						x340 + (x255 * x340) + (((x256 * x338) + (-1 * x250 * x339)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[0]) / sizeof(FLT),
						(((x256 * x353) + (-1 * x250 * x347)) * x258) + x358 + (x255 * x358));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[1]) / sizeof(FLT),
						(x255 * x368) + x368 + (((x256 * x364) + (-1 * x250 * x367)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[2]) / sizeof(FLT),
						x373 + (x255 * x373) + (((x256 * x370) + (-1 * x250 * x372)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Acc[0]) / sizeof(FLT),
						x386 + (x255 * x386) + (((x256 * x382) + (-1 * x250 * x385)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Acc[1]) / sizeof(FLT),
						x393 + (((x256 * x391) + (-1 * x250 * x388)) * x258) + (x255 * x393));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Acc[2]) / sizeof(FLT),
						x397 + (x255 * x397) + (((x256 * x395) + (-1 * x250 * x396)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[0]) / sizeof(FLT),
						x469 + (x469 * x255) + (((x467 * x256) + (-1 * x468 * x250)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[1]) / sizeof(FLT),
						x502 + (x502 * x255) + (((x500 * x256) + (-1 * x501 * x250)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[2]) / sizeof(FLT),
						x532 + (x532 * x255) + (((x530 * x256) + (-1 * x531 * x250)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[0]) / sizeof(FLT),
						x538 + (((x534 * x256) + (-1 * x533 * x250)) * x258) + (x538 * x255));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[1]) / sizeof(FLT),
						(x543 * x255) + x543 + (((x540 * x256) + (-1 * x542 * x250)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[2]) / sizeof(FLT),
						x547 + (x547 * x255) + (((x545 * x256) + (-1 * x546 * x250)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[0]) / sizeof(FLT),
						(x636 * x255) + x636 + (((x634 * x256) + (-1 * x635 * x250)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x678 + (x678 * x255) + (((x676 * x256) + (-1 * x677 * x250)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x706 + (x706 * x255) + (((x704 * x256) + (-1 * x705 * x250)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[0]) / sizeof(FLT),
						x716 + (x716 * x255) + (((x712 * x256) + (-1 * x715 * x250)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[1]) / sizeof(FLT),
						x723 + (x723 * x255) + (((x720 * x256) + (-1 * x722 * x250)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[2]) / sizeof(FLT),
						x727 + (x727 * x255) + (((x725 * x256) + (-1 * x726 * x250)) * x258));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD0.curve) / sizeof(FLT), x257 * x257);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD0.gibmag) / sizeof(FLT), -1 * cos(x254));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD0.gibpha) / sizeof(FLT), x255);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD0.phase) / sizeof(FLT), -1 + (-1 * x255));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD0.tilt) / sizeof(FLT),
						(-1 * x728) + (-1 * x728 * x255));
}

// Full version Jacobian of SurviveJointKalmanErrorModel_LightMeas_x_gen1 wrt
// [(*error_model).Lighthouse.AxisAngleRot[0], (*error_model).Lighthouse.AxisAngleRot[1],
// (*error_model).Lighthouse.AxisAngleRot[2], (*error_model).Lighthouse.Pos[0], (*error_model).Lighthouse.Pos[1],
// (*error_model).Lighthouse.Pos[2], (*error_model).Object.Acc[0], (*error_model).Object.Acc[1],
// (*error_model).Object.Acc[2], (*error_model).Object.IMUBias.AccBias[0], (*error_model).Object.IMUBias.AccBias[1],
// (*error_model).Object.IMUBias.AccBias[2], (*error_model).Object.IMUBias.AccScale[0],
// (*error_model).Object.IMUBias.AccScale[1], (*error_model).Object.IMUBias.AccScale[2],
// (*error_model).Object.IMUBias.GyroBias[0], (*error_model).Object.IMUBias.GyroBias[1],
// (*error_model).Object.IMUBias.GyroBias[2], (*error_model).Object.IMUBias.IMUCorrection[0],
// (*error_model).Object.IMUBias.IMUCorrection[1], (*error_model).Object.IMUBias.IMUCorrection[2],
// (*error_model).Object.Pose.AxisAngleRot[0], (*error_model).Object.Pose.AxisAngleRot[1],
// (*error_model).Object.Pose.AxisAngleRot[2], (*error_model).Object.Pose.Pos[0], (*error_model).Object.Pose.Pos[1],
// (*error_model).Object.Pose.Pos[2], (*error_model).Object.Velocity.AxisAngleRot[0],
// (*error_model).Object.Velocity.AxisAngleRot[1], (*error_model).Object.Velocity.AxisAngleRot[2],
// (*error_model).Object.Velocity.Pos[0], (*error_model).Object.Velocity.Pos[1], (*error_model).Object.Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f4d1c328700>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3283d0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3284c0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c328160>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c328430>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3280d0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c328580>, <cnkalman.codegen.WrapMember object at 0x7f4d1c328250>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c328520>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3281f0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c328850>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3282e0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3286a0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c328370>]

static inline void SurviveJointKalmanErrorModel_LightMeas_x_gen1_jac_error_model_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveJointKalmanModel *_x0,
	const SurviveJointKalmanErrorModel *error_model, const FLT *sensor_pt) {
	if (hx != 0) {
		hx->data[0] = SurviveJointKalmanErrorModel_LightMeas_x_gen1(dt, _x0, error_model, sensor_pt);
	}
	if (Hx != 0) {
		SurviveJointKalmanErrorModel_LightMeas_x_gen1_jac_error_model(Hx, dt, _x0, error_model, sensor_pt);
	}
}
// Jacobian of SurviveJointKalmanErrorModel_LightMeas_x_gen1 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]
static inline void
SurviveJointKalmanErrorModel_LightMeas_x_gen1_jac_sensor_pt(CnMat *Hx, const FLT dt, const SurviveJointKalmanModel *_x0,
															const SurviveJointKalmanErrorModel *error_model,
															const FLT *sensor_pt) {
	const FLT x0 = (*_x0).Lighthouse.Pos[2] + (*error_model).Lighthouse.Pos[2];
	const FLT x1 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[2];
	const FLT x2 = cos(x1);
	const FLT x3 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[1];
	const FLT x4 = sin(x3);
	const FLT x5 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[0];
	const FLT x6 = sin(x5);
	const FLT x7 = x4 * x6;
	const FLT x8 = sin(x1);
	const FLT x9 = cos(x5);
	const FLT x10 = cos(x3);
	const FLT x11 = x9 * x10;
	const FLT x12 = (x8 * x11) + (-1 * x2 * x7);
	const FLT x13 = x6 * x10;
	const FLT x14 = x4 * x9;
	const FLT x15 = (x2 * x14) + (x8 * x13);
	const FLT x16 = (x2 * x11) + (x8 * x7);
	const FLT x17 = (x2 * x13) + (-1 * x8 * x14);
	const FLT x18 = 1. / sqrt((x16 * x16) + (x17 * x17) + (x12 * x12) + (x15 * x15));
	const FLT x19 = x18 * x16;
	const FLT x20 = x18 * x17;
	const FLT x21 = x12 * x18;
	const FLT x22 = x18 * (*_x0).Lighthouse.Rot[1];
	const FLT x23 = (x22 * x15) + (x21 * (*_x0).Lighthouse.Rot[0]) + (x19 * (*_x0).Lighthouse.Rot[3]) +
					(-1 * x20 * (*_x0).Lighthouse.Rot[2]);
	const FLT x24 = x15 * x18;
	const FLT x25 = (-1 * x22 * x12) + (x20 * (*_x0).Lighthouse.Rot[3]) + (x24 * (*_x0).Lighthouse.Rot[0]) +
					(x19 * (*_x0).Lighthouse.Rot[2]);
	const FLT x26 = (-1 * x22 * x17) + (x19 * (*_x0).Lighthouse.Rot[0]) + (-1 * x21 * (*_x0).Lighthouse.Rot[3]) +
					(-1 * x24 * (*_x0).Lighthouse.Rot[2]);
	const FLT x27 = (x22 * x16) + (x21 * (*_x0).Lighthouse.Rot[2]) + (-1 * x24 * (*_x0).Lighthouse.Rot[3]) +
					(x20 * (*_x0).Lighthouse.Rot[0]);
	const FLT x28 = 1. / sqrt((x27 * x27) + (x26 * x26) + (x23 * x23) + (x25 * x25));
	const FLT x29 = x28 * x26;
	const FLT x30 = (*_x0).Lighthouse.Pos[1] + (*error_model).Lighthouse.Pos[1];
	const FLT x31 = x28 * x27;
	const FLT x32 = (*_x0).Lighthouse.Pos[0] + (*error_model).Lighthouse.Pos[0];
	const FLT x33 = x25 * x28;
	const FLT x34 = (x32 * x33) + (x0 * x29) + (-1 * x30 * x31);
	const FLT x35 = x23 * x28;
	const FLT x36 = (x30 * x35) + (-1 * x0 * x33) + (x32 * x29);
	const FLT x37 = (*_x0).Object.Velocity.AxisAngleRot[2] + (*error_model).Object.Velocity.AxisAngleRot[2];
	const FLT x38 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[2];
	const FLT x39 = sin(x38);
	const FLT x40 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[1];
	const FLT x41 = cos(x40);
	const FLT x42 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[0];
	const FLT x43 = sin(x42);
	const FLT x44 = x41 * x43;
	const FLT x45 = sin(x40);
	const FLT x46 = cos(x42);
	const FLT x47 = cos(x38);
	const FLT x48 = x46 * x47;
	const FLT x49 = (x45 * x48) + (x44 * x39);
	const FLT x50 = x43 * x45;
	const FLT x51 = (x41 * x48) + (x50 * x39);
	const FLT x52 = x46 * x39;
	const FLT x53 = (x52 * x41) + (-1 * x50 * x47);
	const FLT x54 = (x44 * x47) + (-1 * x52 * x45);
	const FLT x55 = 1. / sqrt((x53 * x53) + (x54 * x54) + (x49 * x49) + (x51 * x51));
	const FLT x56 = x51 * x55;
	const FLT x57 = x55 * (*_x0).Object.Pose.Rot[2];
	const FLT x58 = x55 * (*_x0).Object.Pose.Rot[0];
	const FLT x59 = x55 * x49;
	const FLT x60 =
		(x59 * (*_x0).Object.Pose.Rot[1]) + (x56 * (*_x0).Object.Pose.Rot[3]) + (x53 * x58) + (-1 * x54 * x57);
	const FLT x61 = dt * dt;
	const FLT x62 = x61 * (x37 * x37);
	const FLT x63 = (*_x0).Object.Velocity.AxisAngleRot[0] + (*error_model).Object.Velocity.AxisAngleRot[0];
	const FLT x64 = (x63 * x63) * x61;
	const FLT x65 = (*_x0).Object.Velocity.AxisAngleRot[1] + (*error_model).Object.Velocity.AxisAngleRot[1];
	const FLT x66 = x61 * (x65 * x65);
	const FLT x67 = 1e-10 + x66 + x62 + x64;
	const FLT x68 = sqrt(x67);
	const FLT x69 = 0.5 * x68;
	const FLT x70 = sin(x69);
	const FLT x71 = (x70 * x70) * (1. / x67);
	const FLT x72 = cos(x69);
	const FLT x73 = 1. / sqrt((x71 * x64) + (x72 * x72) + (x71 * x62) + (x71 * x66));
	const FLT x74 = dt * x70 * x73 * (1. / x68);
	const FLT x75 = x74 * x60;
	const FLT x76 = x54 * x55;
	const FLT x77 = x53 * x55;
	const FLT x78 =
		(-1 * x77 * (*_x0).Object.Pose.Rot[1]) + (x58 * x49) + (x51 * x57) + (x76 * (*_x0).Object.Pose.Rot[3]);
	const FLT x79 = x78 * x74;
	const FLT x80 = (-1 * x76 * (*_x0).Object.Pose.Rot[1]) + (x51 * x58) + (-1 * x77 * (*_x0).Object.Pose.Rot[3]) +
					(-1 * x57 * x49);
	const FLT x81 = x73 * x72;
	const FLT x82 = (x56 * (*_x0).Object.Pose.Rot[1]) + (-1 * x59 * (*_x0).Object.Pose.Rot[3]) +
					(x76 * (*_x0).Object.Pose.Rot[0]) + (x53 * x57);
	const FLT x83 = x82 * x74;
	const FLT x84 = (-1 * x83 * x63) + (-1 * x75 * x37) + (x80 * x81) + (-1 * x79 * x65);
	const FLT x85 = x79 * x37;
	const FLT x86 = x75 * x65;
	const FLT x87 = x81 * x82;
	const FLT x88 = x80 * x74;
	const FLT x89 = x88 * x63;
	const FLT x90 = x87 + (-1 * x85) + x89 + x86;
	const FLT x91 = x83 * x37;
	const FLT x92 = x81 * x78;
	const FLT x93 = x88 * x65;
	const FLT x94 = x75 * x63;
	const FLT x95 = x93 + x91 + (-1 * x94) + x92;
	const FLT x96 = (x84 * sensor_pt[2]) + (-1 * x95 * sensor_pt[0]) + (x90 * sensor_pt[1]);
	const FLT x97 = x88 * x37;
	const FLT x98 = x81 * x60;
	const FLT x99 = x83 * x65;
	const FLT x100 = x79 * x63;
	const FLT x101 = (-1 * x99) + x97 + x100 + x98;
	const FLT x102 = (-1 * x101 * sensor_pt[1]) + (x84 * sensor_pt[0]) + (x95 * sensor_pt[2]);
	const FLT x103 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x104 = (*_x0).Object.Pose.Pos[1] + (2 * ((x101 * x102) + (-1 * x90 * x96))) +
					 (*error_model).Object.Pose.Pos[1] + sensor_pt[1] +
					 (dt * ((*_x0).Object.Velocity.Pos[1] + (*error_model).Object.Velocity.Pos[1])) +
					 (x103 * ((*_x0).Object.Acc[1] + (*error_model).Object.Acc[1]));
	const FLT x105 = (-1 * x90 * sensor_pt[2]) + (x84 * sensor_pt[1]) + (x101 * sensor_pt[0]);
	const FLT x106 = (*error_model).Object.Pose.Pos[2] + sensor_pt[2] + (2 * ((x90 * x105) + (-1 * x95 * x102))) +
					 (*_x0).Object.Pose.Pos[2] +
					 (dt * ((*_x0).Object.Velocity.Pos[2] + (*error_model).Object.Velocity.Pos[2])) +
					 (x103 * ((*_x0).Object.Acc[2] + (*error_model).Object.Acc[2]));
	const FLT x107 = (*_x0).Object.Pose.Pos[0] + (2 * ((x96 * x95) + (-1 * x101 * x105))) +
					 (*error_model).Object.Pose.Pos[0] +
					 (x103 * ((*_x0).Object.Acc[0] + (*error_model).Object.Acc[0])) +
					 (dt * ((*_x0).Object.Velocity.Pos[0] + (*error_model).Object.Velocity.Pos[0])) + sensor_pt[0];
	const FLT x108 = (x33 * x107) + (-1 * x31 * x104) + (x29 * x106);
	const FLT x109 = (x35 * x104) + (x29 * x107) + (-1 * x33 * x106);
	const FLT x110 =
		x104 + (-1 * (x30 + (2 * ((-1 * x36 * x35) + (x31 * x34))))) + (2 * ((-1 * x35 * x109) + (x31 * x108)));
	const FLT x111 = x110 * x110;
	const FLT x112 = (x0 * x31) + (x30 * x29) + (-1 * x32 * x35);
	const FLT x113 = (x31 * x106) + (x29 * x104) + (-1 * x35 * x107);
	const FLT x114 =
		(-1 * (x0 + (2 * ((-1 * x31 * x112) + (x33 * x36))))) + x106 + (2 * ((-1 * x31 * x113) + (x33 * x109)));
	const FLT x115 = x114 * x114;
	const FLT x116 =
		x107 + (2 * ((-1 * x33 * x108) + (x35 * x113))) + (-1 * (x32 + (2 * ((-1 * x34 * x33) + (x35 * x112)))));
	const FLT x117 = (x116 * x116) + x115;
	const FLT x118 = 1. / x117;
	const FLT x119 = (*error_model).BSD0.tilt + (*_x0).BSD0.tilt;
	const FLT x120 = 1. / sqrt(1 + (-1 * x111 * x118 * (x119 * x119)));
	const FLT x121 = (-1 * x93) + x94 + (-1 * x92) + (-1 * x91);
	const FLT x122 = 2 * x95;
	const FLT x123 = 1 + (x122 * x121) + (-2 * (x101 * x101));
	const FLT x124 = 2 * x90;
	const FLT x125 = 2 * x101;
	const FLT x126 = x84 * x125;
	const FLT x127 = x126 + (-1 * x124 * x121);
	const FLT x128 = x84 * x122;
	const FLT x129 = (x90 * x125) + (-1 * x128);
	const FLT x130 = 2 * ((x29 * x129) + (x33 * x123) + (-1 * x31 * x127));
	const FLT x131 = 2 * ((x31 * x129) + (-1 * x35 * x123) + (x29 * x127));
	const FLT x132 = x123 + (-1 * x33 * x130) + (x35 * x131);
	const FLT x133 = 2 * x116;
	const FLT x134 = 2 * ((-1 * x33 * x129) + (x35 * x127) + (x29 * x123));
	const FLT x135 = x129 + (-1 * x31 * x131) + (x33 * x134);
	const FLT x136 = 2 * x114;
	const FLT x137 = 1.0 / 2.0 * x110 * x119 * (1. / (x117 * sqrt(x117)));
	const FLT x138 = x127 + (x31 * x130) + (-1 * x35 * x134);
	const FLT x139 = x119 * (1. / sqrt(x117));
	const FLT x140 = 1. / x115;
	const FLT x141 = x135 * x140;
	const FLT x142 = 1. / x114;
	const FLT x143 = x118 * x115;
	const FLT x144 = (-1 * ((-1 * x132 * x142) + (x116 * x141)) * x143) +
					 (-1 * x120 * ((x138 * x139) + (-1 * ((x135 * x136) + (x133 * x132)) * x137)));
	const FLT x145 = -1 * x114;
	const FLT x146 = sin(1.5707963267949 + (*_x0).BSD0.gibpha + (-1 * asin(x110 * x139)) + (*error_model).BSD0.gibpha +
						 (-1 * ((*error_model).BSD0.phase + (*_x0).BSD0.phase)) + (-1 * atan2(x116, x145))) *
					 ((*error_model).BSD0.gibmag + (*_x0).BSD0.gibmag);
	const FLT x147 =
		2 * (1. / (x115 + x111)) * x115 * atan2(x110, x145) * ((*error_model).BSD0.curve + (*_x0).BSD0.curve);
	const FLT x148 = (x90 * x122) + (-1 * x126);
	const FLT x149 = x99 + (-1 * x100) + (-1 * x98) + (-1 * x97);
	const FLT x150 = 1 + (x125 * x149) + (-2 * (x90 * x90));
	const FLT x151 = x84 * x124;
	const FLT x152 = x151 + (-1 * x122 * x149);
	const FLT x153 = (x29 * x152) + (x33 * x148) + (-1 * x31 * x150);
	const FLT x154 = 2 * x33;
	const FLT x155 = 2 * ((-1 * x35 * x148) + (x29 * x150) + (x31 * x152));
	const FLT x156 = x148 + (-1 * x153 * x154) + (x35 * x155);
	const FLT x157 = 2 * ((-1 * x33 * x152) + (x35 * x150) + (x29 * x148));
	const FLT x158 = x152 + (-1 * x31 * x155) + (x33 * x157);
	const FLT x159 = 2 * x31;
	const FLT x160 = x150 + (x153 * x159) + (-1 * x35 * x157);
	const FLT x161 = x140 * x158;
	const FLT x162 = (-1 * ((-1 * x142 * x156) + (x116 * x161)) * x143) +
					 (-1 * x120 * ((x160 * x139) + (-1 * ((x136 * x158) + (x133 * x156)) * x137)));
	const FLT x163 = (-1 * x87) + (-1 * x89) + (-1 * x86) + x85;
	const FLT x164 = x128 + (-1 * x125 * x163);
	const FLT x165 = (x95 * x125) + (-1 * x151);
	const FLT x166 = 1 + (x124 * x163) + (-2 * (x95 * x95));
	const FLT x167 = (x29 * x166) + (x33 * x164) + (-1 * x31 * x165);
	const FLT x168 = (-1 * x35 * x164) + (x31 * x166) + (x29 * x165);
	const FLT x169 = 2 * x35;
	const FLT x170 = x164 + (-1 * x167 * x154) + (x169 * x168);
	const FLT x171 = (-1 * x33 * x166) + (x35 * x165) + (x29 * x164);
	const FLT x172 = x166 + (-1 * x168 * x159) + (x171 * x154);
	const FLT x173 = x165 + (-1 * x169 * x171) + (x167 * x159);
	const FLT x174 = x172 * x140;
	const FLT x175 = (-1 * ((-1 * x170 * x142) + (x116 * x174)) * x143) +
					 (-1 * x120 * ((x173 * x139) + (-1 * ((x172 * x136) + (x170 * x133)) * x137)));
	cnMatrixOptionalSet(Hx, 0, 0, x144 + (x144 * x146) + (((x110 * x141) + (-1 * x138 * x142)) * x147));
	cnMatrixOptionalSet(Hx, 0, 1, x162 + (x162 * x146) + (((x110 * x161) + (-1 * x160 * x142)) * x147));
	cnMatrixOptionalSet(Hx, 0, 2, x175 + (x175 * x146) + (((x110 * x174) + (-1 * x173 * x142)) * x147));
}

// Full version Jacobian of SurviveJointKalmanErrorModel_LightMeas_x_gen1 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]

static inline void SurviveJointKalmanErrorModel_LightMeas_x_gen1_jac_sensor_pt_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveJointKalmanModel *_x0,
	const SurviveJointKalmanErrorModel *error_model, const FLT *sensor_pt) {
	if (hx != 0) {
		hx->data[0] = SurviveJointKalmanErrorModel_LightMeas_x_gen1(dt, _x0, error_model, sensor_pt);
	}
	if (Hx != 0) {
		SurviveJointKalmanErrorModel_LightMeas_x_gen1_jac_sensor_pt(Hx, dt, _x0, error_model, sensor_pt);
	}
}
static inline FLT SurviveJointKalmanErrorModel_LightMeas_y_gen1(const FLT dt, const SurviveJointKalmanModel *_x0,
																const SurviveJointKalmanErrorModel *error_model,
																const FLT *sensor_pt) {
	const FLT x0 = (*_x0).Object.Velocity.AxisAngleRot[2] + (*error_model).Object.Velocity.AxisAngleRot[2];
	const FLT x1 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[0];
	const FLT x2 = sin(x1);
	const FLT x3 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[1];
	const FLT x4 = cos(x3);
	const FLT x5 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[2];
	const FLT x6 = sin(x5);
	const FLT x7 = x4 * x6;
	const FLT x8 = cos(x1);
	const FLT x9 = sin(x3);
	const FLT x10 = cos(x5);
	const FLT x11 = x9 * x10;
	const FLT x12 = (x8 * x11) + (x2 * x7);
	const FLT x13 = x6 * x9;
	const FLT x14 = x4 * x10;
	const FLT x15 = (x8 * x14) + (x2 * x13);
	const FLT x16 = (x8 * x7) + (-1 * x2 * x11);
	const FLT x17 = (x2 * x14) + (-1 * x8 * x13);
	const FLT x18 = 1. / sqrt((x16 * x16) + (x17 * x17) + (x12 * x12) + (x15 * x15));
	const FLT x19 = x15 * x18;
	const FLT x20 = x18 * (*_x0).Object.Pose.Rot[2];
	const FLT x21 = x18 * x16;
	const FLT x22 = x12 * x18;
	const FLT x23 = (x22 * (*_x0).Object.Pose.Rot[1]) + (x21 * (*_x0).Object.Pose.Rot[0]) +
					(x19 * (*_x0).Object.Pose.Rot[3]) + (-1 * x20 * x17);
	const FLT x24 = dt * dt;
	const FLT x25 = (x0 * x0) * x24;
	const FLT x26 = (*_x0).Object.Velocity.AxisAngleRot[0] + (*error_model).Object.Velocity.AxisAngleRot[0];
	const FLT x27 = x24 * (x26 * x26);
	const FLT x28 = (*_x0).Object.Velocity.AxisAngleRot[1] + (*error_model).Object.Velocity.AxisAngleRot[1];
	const FLT x29 = x24 * (x28 * x28);
	const FLT x30 = 1e-10 + x29 + x25 + x27;
	const FLT x31 = sqrt(x30);
	const FLT x32 = 0.5 * x31;
	const FLT x33 = sin(x32);
	const FLT x34 = (1. / x30) * (x33 * x33);
	const FLT x35 = cos(x32);
	const FLT x36 = 1. / sqrt((x34 * x27) + (x35 * x35) + (x34 * x25) + (x34 * x29));
	const FLT x37 = dt * (1. / x31) * x33 * x36;
	const FLT x38 = x37 * x23;
	const FLT x39 = x18 * x17;
	const FLT x40 = (x22 * (*_x0).Object.Pose.Rot[0]) + (-1 * x21 * (*_x0).Object.Pose.Rot[1]) + (x20 * x15) +
					(x39 * (*_x0).Object.Pose.Rot[3]);
	const FLT x41 = x40 * x37;
	const FLT x42 = (x19 * (*_x0).Object.Pose.Rot[0]) + (-1 * x21 * (*_x0).Object.Pose.Rot[3]) +
					(-1 * x39 * (*_x0).Object.Pose.Rot[1]) + (-1 * x20 * x12);
	const FLT x43 = x36 * x35;
	const FLT x44 = (x19 * (*_x0).Object.Pose.Rot[1]) + (-1 * x22 * (*_x0).Object.Pose.Rot[3]) +
					(x39 * (*_x0).Object.Pose.Rot[0]) + (x20 * x16);
	const FLT x45 = x44 * x37;
	const FLT x46 = (x42 * x43) + (-1 * x45 * x26) + (-1 * x0 * x38) + (-1 * x41 * x28);
	const FLT x47 = x37 * x28;
	const FLT x48 = x42 * x37;
	const FLT x49 = (x43 * x44) + (-1 * x0 * x41) + (x48 * x26) + (x47 * x23);
	const FLT x50 = (-1 * x38 * x26) + (x0 * x45) + (x42 * x47) + (x40 * x43);
	const FLT x51 = (-1 * x50 * sensor_pt[0]) + (x46 * sensor_pt[2]) + (x49 * sensor_pt[1]);
	const FLT x52 = (x41 * x26) + (-1 * x45 * x28) + (x0 * x48) + (x43 * x23);
	const FLT x53 = (-1 * x52 * sensor_pt[1]) + (x46 * sensor_pt[0]) + (x50 * sensor_pt[2]);
	const FLT x54 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x55 = (*error_model).Object.Pose.Pos[1] + sensor_pt[1] + (2 * ((x53 * x52) + (-1 * x51 * x49))) +
					(dt * ((*_x0).Object.Velocity.Pos[1] + (*error_model).Object.Velocity.Pos[1])) +
					(*_x0).Object.Pose.Pos[1] + (x54 * ((*_x0).Object.Acc[1] + (*error_model).Object.Acc[1]));
	const FLT x56 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[2];
	const FLT x57 = cos(x56);
	const FLT x58 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[1];
	const FLT x59 = sin(x58);
	const FLT x60 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[0];
	const FLT x61 = sin(x60);
	const FLT x62 = x61 * x59;
	const FLT x63 = cos(x58);
	const FLT x64 = cos(x60);
	const FLT x65 = sin(x56);
	const FLT x66 = x64 * x65;
	const FLT x67 = (x63 * x66) + (-1 * x62 * x57);
	const FLT x68 = x63 * x61;
	const FLT x69 = x64 * x57;
	const FLT x70 = (x69 * x59) + (x68 * x65);
	const FLT x71 = (x63 * x69) + (x62 * x65);
	const FLT x72 = (x68 * x57) + (-1 * x66 * x59);
	const FLT x73 = 1. / sqrt((x72 * x72) + (x71 * x71) + (x67 * x67) + (x70 * x70));
	const FLT x74 = x71 * x73;
	const FLT x75 = x73 * x72;
	const FLT x76 = x73 * x67;
	const FLT x77 = x70 * x73;
	const FLT x78 = (x77 * (*_x0).Lighthouse.Rot[1]) + (x76 * (*_x0).Lighthouse.Rot[0]) +
					(x74 * (*_x0).Lighthouse.Rot[3]) + (-1 * x75 * (*_x0).Lighthouse.Rot[2]);
	const FLT x79 = (-1 * x76 * (*_x0).Lighthouse.Rot[1]) + (x77 * (*_x0).Lighthouse.Rot[0]) +
					(x75 * (*_x0).Lighthouse.Rot[3]) + (x74 * (*_x0).Lighthouse.Rot[2]);
	const FLT x80 = (x74 * (*_x0).Lighthouse.Rot[0]) + (-1 * x76 * (*_x0).Lighthouse.Rot[3]) +
					(-1 * x75 * (*_x0).Lighthouse.Rot[1]) + (-1 * x77 * (*_x0).Lighthouse.Rot[2]);
	const FLT x81 = (x74 * (*_x0).Lighthouse.Rot[1]) + (-1 * x77 * (*_x0).Lighthouse.Rot[3]) +
					(x76 * (*_x0).Lighthouse.Rot[2]) + (x75 * (*_x0).Lighthouse.Rot[0]);
	const FLT x82 = 1. / sqrt((x81 * x81) + (x80 * x80) + (x78 * x78) + (x79 * x79));
	const FLT x83 = x80 * x82;
	const FLT x84 = (-1 * x49 * sensor_pt[2]) + (x46 * sensor_pt[1]) + (x52 * sensor_pt[0]);
	const FLT x85 = (*_x0).Object.Pose.Pos[0] + (2 * ((x50 * x51) + (-1 * x84 * x52))) +
					(*error_model).Object.Pose.Pos[0] + (x54 * ((*_x0).Object.Acc[0] + (*error_model).Object.Acc[0])) +
					sensor_pt[0] + (dt * ((*_x0).Object.Velocity.Pos[0] + (*error_model).Object.Velocity.Pos[0]));
	const FLT x86 = x82 * x78;
	const FLT x87 = (dt * ((*_x0).Object.Velocity.Pos[2] + (*error_model).Object.Velocity.Pos[2])) +
					(*error_model).Object.Pose.Pos[2] + sensor_pt[2] + (2 * ((x84 * x49) + (-1 * x50 * x53))) +
					(*_x0).Object.Pose.Pos[2] + (x54 * ((*_x0).Object.Acc[2] + (*error_model).Object.Acc[2]));
	const FLT x88 = x81 * x82;
	const FLT x89 = (x88 * x87) + (x83 * x55) + (-1 * x85 * x86);
	const FLT x90 = x82 * x79;
	const FLT x91 = (-1 * x88 * x55) + (x85 * x90) + (x83 * x87);
	const FLT x92 = (*_x0).Lighthouse.Pos[1] + (*error_model).Lighthouse.Pos[1];
	const FLT x93 = x82 * x92;
	const FLT x94 = (*_x0).Lighthouse.Pos[0] + (*error_model).Lighthouse.Pos[0];
	const FLT x95 = (*_x0).Lighthouse.Pos[2] + (*error_model).Lighthouse.Pos[2];
	const FLT x96 = (x88 * x95) + (x80 * x93) + (-1 * x86 * x94);
	const FLT x97 = x82 * ((x90 * x94) + (x83 * x95) + (-1 * x81 * x93));
	const FLT x98 =
		x85 + (2 * ((-1 * x91 * x90) + (x89 * x86))) + (-1 * (x94 + (2 * ((-1 * x79 * x97) + (x86 * x96)))));
	const FLT x99 = (x86 * x92) + (-1 * x90 * x95) + (x83 * x94);
	const FLT x100 = (x86 * x55) + (x83 * x85) + (-1 * x87 * x90);
	const FLT x101 =
		x87 + (-1 * (x95 + (2 * ((-1 * x88 * x96) + (x90 * x99))))) + (2 * ((-1 * x88 * x89) + (x90 * x100)));
	const FLT x102 = -1 * x101;
	const FLT x103 =
		x55 + (-1 * (x92 + (2 * ((-1 * x86 * x99) + (x81 * x97))))) + (2 * ((-1 * x86 * x100) + (x88 * x91)));
	const FLT x104 =
		(-1 * ((*error_model).BSD1.phase + (*_x0).BSD1.phase)) +
		(-1 * asin((1. / sqrt((x103 * x103) + (x101 * x101))) * x98 * ((*error_model).BSD1.tilt + (*_x0).BSD1.tilt))) +
		(-1 * atan2(-1 * x103, x102));
	return x104 + ((atan2(x98, x102) * atan2(x98, x102)) * ((*error_model).BSD1.curve + (*_x0).BSD1.curve)) +
		   (-1 * ((*error_model).BSD1.gibmag + (*_x0).BSD1.gibmag) *
			cos(1.5707963267949 + x104 + (*error_model).BSD1.gibpha + (*_x0).BSD1.gibpha));
}

// Jacobian of SurviveJointKalmanErrorModel_LightMeas_y_gen1 wrt [(*_x0).Lighthouse.Pos[0], (*_x0).Lighthouse.Pos[1],
// (*_x0).Lighthouse.Pos[2], (*_x0).Lighthouse.Rot[0], (*_x0).Lighthouse.Rot[1], (*_x0).Lighthouse.Rot[2],
// (*_x0).Lighthouse.Rot[3], (*_x0).Object.Acc[0], (*_x0).Object.Acc[1], (*_x0).Object.Acc[2],
// (*_x0).Object.IMUBias.AccBias[0], (*_x0).Object.IMUBias.AccBias[1], (*_x0).Object.IMUBias.AccBias[2],
// (*_x0).Object.IMUBias.AccScale[0], (*_x0).Object.IMUBias.AccScale[1], (*_x0).Object.IMUBias.AccScale[2],
// (*_x0).Object.IMUBias.GyroBias[0], (*_x0).Object.IMUBias.GyroBias[1], (*_x0).Object.IMUBias.GyroBias[2],
// (*_x0).Object.IMUBias.IMUCorrection[0], (*_x0).Object.IMUBias.IMUCorrection[1],
// (*_x0).Object.IMUBias.IMUCorrection[2], (*_x0).Object.IMUBias.IMUCorrection[3], (*_x0).Object.Pose.Pos[0],
// (*_x0).Object.Pose.Pos[1], (*_x0).Object.Pose.Pos[2], (*_x0).Object.Pose.Rot[0], (*_x0).Object.Pose.Rot[1],
// (*_x0).Object.Pose.Rot[2], (*_x0).Object.Pose.Rot[3], (*_x0).Object.Velocity.AxisAngleRot[0],
// (*_x0).Object.Velocity.AxisAngleRot[1], (*_x0).Object.Velocity.AxisAngleRot[2], (*_x0).Object.Velocity.Pos[0],
// (*_x0).Object.Velocity.Pos[1], (*_x0).Object.Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f4d1c3388b0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c338b80>, <cnkalman.codegen.WrapMember object at 0x7f4d1c338970>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c338c40>, <cnkalman.codegen.WrapMember object at 0x7f4d1c338910>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c338be0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c338a30>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c338d00>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3389d0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c338ca0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3237f0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c338ac0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c338850>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c338b20>]
static inline void SurviveJointKalmanErrorModel_LightMeas_y_gen1_jac_x0(CnMat *Hx, const FLT dt,
																		const SurviveJointKalmanModel *_x0,
																		const SurviveJointKalmanErrorModel *error_model,
																		const FLT *sensor_pt) {
	const FLT x0 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[0];
	const FLT x1 = sin(x0);
	const FLT x2 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[1];
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[2];
	const FLT x5 = cos(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = sin(x4);
	const FLT x8 = cos(x0);
	const FLT x9 = cos(x2);
	const FLT x10 = x8 * x9;
	const FLT x11 = (x7 * x10) + (-1 * x1 * x6);
	const FLT x12 = x1 * x9;
	const FLT x13 = (x6 * x8) + (x7 * x12);
	const FLT x14 = x3 * x7;
	const FLT x15 = (x5 * x10) + (x1 * x14);
	const FLT x16 = (x5 * x12) + (-1 * x8 * x14);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x15 * x15) + (x11 * x11) + (x13 * x13));
	const FLT x18 = x11 * x17;
	const FLT x19 = x17 * (*_x0).Lighthouse.Rot[0];
	const FLT x20 = x13 * x17;
	const FLT x21 = x17 * (*_x0).Lighthouse.Rot[1];
	const FLT x22 =
		(-1 * x20 * (*_x0).Lighthouse.Rot[3]) + (x18 * (*_x0).Lighthouse.Rot[2]) + (x21 * x15) + (x19 * x16);
	const FLT x23 = x15 * x17;
	const FLT x24 = x17 * x16;
	const FLT x25 =
		(x21 * x13) + (x11 * x19) + (x23 * (*_x0).Lighthouse.Rot[3]) + (-1 * x24 * (*_x0).Lighthouse.Rot[2]);
	const FLT x26 = x25 * x25;
	const FLT x27 =
		(x13 * x19) + (x24 * (*_x0).Lighthouse.Rot[3]) + (-1 * x21 * x11) + (x23 * (*_x0).Lighthouse.Rot[2]);
	const FLT x28 = x27 * x27;
	const FLT x29 =
		(-1 * x21 * x16) + (-1 * x18 * (*_x0).Lighthouse.Rot[3]) + (x15 * x19) + (-1 * x20 * (*_x0).Lighthouse.Rot[2]);
	const FLT x30 = x22 * x22;
	const FLT x31 = x30 + (x29 * x29) + x26 + x28;
	const FLT x32 = 1. / x31;
	const FLT x33 = 2 * x32;
	const FLT x34 = x33 * x25;
	const FLT x35 = x34 * x22;
	const FLT x36 = -1 * x35;
	const FLT x37 = x33 * x29;
	const FLT x38 = x37 * x27;
	const FLT x39 = -1 * x38;
	const FLT x40 = x39 + x36;
	const FLT x41 = 1. / sqrt(x31);
	const FLT x42 = (*_x0).Lighthouse.Pos[2] + (*error_model).Lighthouse.Pos[2];
	const FLT x43 = x41 * x27;
	const FLT x44 = (*_x0).Lighthouse.Pos[0] + (*error_model).Lighthouse.Pos[0];
	const FLT x45 = x41 * x29;
	const FLT x46 = (*_x0).Lighthouse.Pos[1] + (*error_model).Lighthouse.Pos[1];
	const FLT x47 = x41 * x25;
	const FLT x48 = (x46 * x47) + (-1 * x42 * x43) + (x44 * x45);
	const FLT x49 = x41 * x48;
	const FLT x50 = x41 * x22;
	const FLT x51 = (x50 * x42) + (x45 * x46) + (-1 * x44 * x47);
	const FLT x52 = x51 * x41;
	const FLT x53 = (*_x0).Object.Velocity.AxisAngleRot[2] + (*error_model).Object.Velocity.AxisAngleRot[2];
	const FLT x54 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[0];
	const FLT x55 = sin(x54);
	const FLT x56 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[2];
	const FLT x57 = sin(x56);
	const FLT x58 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[1];
	const FLT x59 = sin(x58);
	const FLT x60 = x57 * x59;
	const FLT x61 = cos(x54);
	const FLT x62 = cos(x58);
	const FLT x63 = cos(x56);
	const FLT x64 = x63 * x62;
	const FLT x65 = (x64 * x61) + (x60 * x55);
	const FLT x66 = x62 * x57;
	const FLT x67 = x63 * x59;
	const FLT x68 = (x61 * x67) + (x66 * x55);
	const FLT x69 = (x61 * x66) + (-1 * x67 * x55);
	const FLT x70 = (x64 * x55) + (-1 * x60 * x61);
	const FLT x71 = 1. / sqrt((x70 * x70) + (x69 * x69) + (x68 * x68) + (x65 * x65));
	const FLT x72 = x71 * (*_x0).Object.Pose.Rot[3];
	const FLT x73 = x71 * (*_x0).Object.Pose.Rot[2];
	const FLT x74 = x71 * x69;
	const FLT x75 = x71 * x68;
	const FLT x76 =
		(x74 * (*_x0).Object.Pose.Rot[0]) + (x75 * (*_x0).Object.Pose.Rot[1]) + (x72 * x65) + (-1 * x70 * x73);
	const FLT x77 = dt * dt;
	const FLT x78 = x53 * x53;
	const FLT x79 = x78 * x77;
	const FLT x80 = (*_x0).Object.Velocity.AxisAngleRot[0] + (*error_model).Object.Velocity.AxisAngleRot[0];
	const FLT x81 = x80 * x80;
	const FLT x82 = x81 * x77;
	const FLT x83 = (*_x0).Object.Velocity.AxisAngleRot[1] + (*error_model).Object.Velocity.AxisAngleRot[1];
	const FLT x84 = x83 * x83;
	const FLT x85 = x84 * x77;
	const FLT x86 = 1e-10 + x85 + x79 + x82;
	const FLT x87 = sqrt(x86);
	const FLT x88 = 0.5 * x87;
	const FLT x89 = sin(x88);
	const FLT x90 = x89 * x89;
	const FLT x91 = 1. / x86;
	const FLT x92 = x91 * x90;
	const FLT x93 = cos(x88);
	const FLT x94 = (x82 * x92) + (x79 * x92) + (x93 * x93) + (x85 * x92);
	const FLT x95 = 1. / sqrt(x94);
	const FLT x96 = x89 * (1. / x87);
	const FLT x97 = dt * x96;
	const FLT x98 = x97 * x95;
	const FLT x99 = x76 * x98;
	const FLT x100 =
		(-1 * x74 * (*_x0).Object.Pose.Rot[1]) + (x75 * (*_x0).Object.Pose.Rot[0]) + (x73 * x65) + (x70 * x72);
	const FLT x101 = x98 * x100;
	const FLT x102 = x71 * x65;
	const FLT x103 = x71 * x70;
	const FLT x104 = (-1 * x103 * (*_x0).Object.Pose.Rot[1]) + (x102 * (*_x0).Object.Pose.Rot[0]) + (-1 * x72 * x69) +
					 (-1 * x75 * (*_x0).Object.Pose.Rot[2]);
	const FLT x105 = x93 * x95;
	const FLT x106 = x105 * x104;
	const FLT x107 =
		(x102 * (*_x0).Object.Pose.Rot[1]) + (-1 * x72 * x68) + (x103 * (*_x0).Object.Pose.Rot[0]) + (x73 * x69);
	const FLT x108 = x95 * x107;
	const FLT x109 = x97 * x108;
	const FLT x110 = (-1 * x80 * x109) + x106 + (-1 * x53 * x99) + (-1 * x83 * x101);
	const FLT x111 = x98 * x104;
	const FLT x112 = x76 * x105;
	const FLT x113 = (x80 * x101) + (-1 * x83 * x109) + (x53 * x111) + x112;
	const FLT x114 = x105 * x107;
	const FLT x115 = (-1 * x53 * x101) + (x80 * x111) + x114 + (x83 * x99);
	const FLT x116 = (-1 * x115 * sensor_pt[2]) + (x110 * sensor_pt[1]) + (x113 * sensor_pt[0]);
	const FLT x117 = x100 * x105;
	const FLT x118 = (-1 * x80 * x99) + (x83 * x111) + (x53 * x109) + x117;
	const FLT x119 = (-1 * x118 * sensor_pt[0]) + (x110 * sensor_pt[2]) + (x115 * sensor_pt[1]);
	const FLT x120 = dt * fabs(dt);
	const FLT x121 = 1.0 / 2.0 * x120;
	const FLT x122 = (*_x0).Object.Pose.Pos[0] + (*error_model).Object.Pose.Pos[0] +
					 (2 * ((x118 * x119) + (-1 * x113 * x116))) +
					 (x121 * ((*_x0).Object.Acc[0] + (*error_model).Object.Acc[0])) + sensor_pt[0] +
					 (dt * ((*_x0).Object.Velocity.Pos[0] + (*error_model).Object.Velocity.Pos[0]));
	const FLT x123 = x41 * x122;
	const FLT x124 = (-1 * x113 * sensor_pt[1]) + (x110 * sensor_pt[0]) + (x118 * sensor_pt[2]);
	const FLT x125 = (dt * ((*_x0).Object.Velocity.Pos[2] + (*error_model).Object.Velocity.Pos[2])) + sensor_pt[2] +
					 (*error_model).Object.Pose.Pos[2] +
					 (x121 * ((*_x0).Object.Acc[2] + (*error_model).Object.Acc[2])) +
					 (2 * ((x115 * x116) + (-1 * x118 * x124))) + (*_x0).Object.Pose.Pos[2];
	const FLT x126 = (*_x0).Object.Pose.Pos[1] + (2 * ((x113 * x124) + (-1 * x119 * x115))) +
					 (x121 * ((*_x0).Object.Acc[1] + (*error_model).Object.Acc[1])) +
					 (*error_model).Object.Pose.Pos[1] + sensor_pt[1] +
					 (dt * ((*_x0).Object.Velocity.Pos[1] + (*error_model).Object.Velocity.Pos[1]));
	const FLT x127 = x41 * x126;
	const FLT x128 = (x25 * x127) + (x29 * x123) + (-1 * x43 * x125);
	const FLT x129 = (x50 * x125) + (x29 * x127) + (-1 * x47 * x122);
	const FLT x130 =
		x125 + (-1 * (x42 + (2 * ((-1 * x52 * x22) + (x49 * x27))))) + (2 * ((-1 * x50 * x129) + (x43 * x128)));
	const FLT x131 = x130 * x130;
	const FLT x132 = 1. / x131;
	const FLT x133 = (x27 * x123) + (-1 * x22 * x127) + (x45 * x125);
	const FLT x134 = x41 * x133;
	const FLT x135 = (x42 * x45) + (x43 * x44) + (-1 * x50 * x46);
	const FLT x136 =
		x122 + (2 * ((-1 * x27 * x134) + (x47 * x129))) + (-1 * (x44 + (2 * ((-1 * x43 * x135) + (x52 * x25)))));
	const FLT x137 = x132 * x136;
	const FLT x138 = x33 * x28;
	const FLT x139 = x33 * x26;
	const FLT x140 = -1 + x139 + x138;
	const FLT x141 = 1. / x130;
	const FLT x142 = -1 * x130;
	const FLT x143 = atan2(x136, x142);
	const FLT x144 = x136 * x136;
	const FLT x145 = 2 * (1. / (x131 + x144)) * x131 * x143 * ((*error_model).BSD1.curve + (*_x0).BSD1.curve);
	const FLT x146 =
		x126 + (-1 * (x46 + (2 * ((-1 * x49 * x25) + (x50 * x135))))) + (2 * ((-1 * x47 * x128) + (x22 * x134)));
	const FLT x147 = x131 + (x146 * x146);
	const FLT x148 = 1. / x147;
	const FLT x149 = (*error_model).BSD1.tilt + (*_x0).BSD1.tilt;
	const FLT x150 = 1. / sqrt(1 + (-1 * x144 * x148 * (x149 * x149)));
	const FLT x151 = x37 * x25;
	const FLT x152 = x22 * x27;
	const FLT x153 = x33 * x152;
	const FLT x154 = -1 * x153;
	const FLT x155 = x154 + x151;
	const FLT x156 = 2 * x146;
	const FLT x157 = 2 * x130;
	const FLT x158 = 1.0 / 2.0 * x136 * (1. / (x147 * sqrt(x147))) * x149;
	const FLT x159 = 1. / sqrt(x147);
	const FLT x160 = x149 * x159;
	const FLT x161 = x132 * x146;
	const FLT x162 = x131 * x148;
	const FLT x163 = (-1 * x162 * ((-1 * x40 * x161) + (x141 * x155))) +
					 (-1 * x150 * ((x160 * x140) + (-1 * x158 * ((x40 * x157) + (x155 * x156)))));
	const FLT x164 = 1.5707963267949 + (*_x0).BSD1.gibpha + (-1 * asin(x160 * x136)) + (*error_model).BSD1.gibpha +
					 (-1 * ((*error_model).BSD1.phase + (*_x0).BSD1.phase)) + (-1 * atan2(-1 * x146, x142));
	const FLT x165 = sin(x164) * ((*error_model).BSD1.gibmag + (*_x0).BSD1.gibmag);
	const FLT x166 = x37 * x22;
	const FLT x167 = x34 * x27;
	const FLT x168 = -1 * x167;
	const FLT x169 = x168 + x166;
	const FLT x170 = -1 * x151;
	const FLT x171 = x170 + x154;
	const FLT x172 = x30 * x33;
	const FLT x173 = -1 + x172;
	const FLT x174 = x173 + x139;
	const FLT x175 = (-1 * ((-1 * x161 * x169) + (x174 * x141)) * x162) +
					 (-1 * x150 * ((x160 * x171) + (-1 * ((x169 * x157) + (x174 * x156)) * x158)));
	const FLT x176 = x173 + x138;
	const FLT x177 = x36 + x38;
	const FLT x178 = -1 * x166;
	const FLT x179 = x178 + x168;
	const FLT x180 = (-1 * ((-1 * x161 * x176) + (x179 * x141)) * x162) +
					 (-1 * x150 * ((x160 * x177) + (-1 * ((x176 * x157) + (x179 * x156)) * x158)));
	const FLT x181 = 1. / (x31 * sqrt(x31));
	const FLT x182 = 2 * x22;
	const FLT x183 = 2 * x18;
	const FLT x184 = 2 * x23;
	const FLT x185 = 2 * x20;
	const FLT x186 = (x27 * x185) + (x29 * x184) + (x24 * x182) + (x25 * x183);
	const FLT x187 = x181 * x186;
	const FLT x188 = x22 * x187;
	const FLT x189 = x25 * x187;
	const FLT x190 = 1.0 / 2.0 * x189;
	const FLT x191 = 1.0 / 2.0 * x187;
	const FLT x192 = x27 * x191;
	const FLT x193 = x41 * x18;
	const FLT x194 = x46 * x193;
	const FLT x195 = x41 * x20;
	const FLT x196 = -1 * x42 * x195;
	const FLT x197 = x41 * x44;
	const FLT x198 = x23 * x197;
	const FLT x199 = x29 * x191;
	const FLT x200 = (-1 * x44 * x199) + x196 + (-1 * x46 * x190) + x194 + (x42 * x192) + x198;
	const FLT x201 = 2 * x43;
	const FLT x202 = x49 * x185;
	const FLT x203 = x48 * x27;
	const FLT x204 = 1.0 / 2.0 * x126;
	const FLT x205 = x23 * x123;
	const FLT x206 = x29 * x122;
	const FLT x207 = x27 * x125;
	const FLT x208 = 1.0 / 2.0 * x207;
	const FLT x209 = -1 * x125 * x195;
	const FLT x210 = x18 * x127;
	const FLT x211 = x210 + x205 + (-1 * x206 * x191) + (x208 * x187) + (-1 * x204 * x189) + x209;
	const FLT x212 = 1.0 / 2.0 * x188;
	const FLT x213 = x41 * x23;
	const FLT x214 = x46 * x213;
	const FLT x215 = x41 * x24;
	const FLT x216 = x42 * x215;
	const FLT x217 = -1 * x18 * x197;
	const FLT x218 = x217 + (-1 * x46 * x199) + (-1 * x42 * x212) + (x44 * x190) + x214 + x216;
	const FLT x219 = 2 * x50;
	const FLT x220 = 2 * x195;
	const FLT x221 = x220 * x128;
	const FLT x222 = x128 * x181;
	const FLT x223 = x27 * x222;
	const FLT x224 = x41 * x125;
	const FLT x225 = x24 * x224;
	const FLT x226 = -1 * x18 * x123;
	const FLT x227 = x23 * x127;
	const FLT x228 = x227 + x226;
	const FLT x229 = x225 + (x122 * x190) + (-1 * x212 * x125) + x228 + (-1 * x29 * x204 * x187);
	const FLT x230 = 2 * x24;
	const FLT x231 = x41 * x129;
	const FLT x232 = (x52 * x230) + (-1 * x230 * x231);
	const FLT x233 = (-1 * x223 * x186) + x221 + (-1 * x219 * x229) + (-1 * x51 * x188) + (-1 * x200 * x201) + x232 +
					 (x203 * x187) + (x219 * x218) + (x211 * x201) + (x129 * x188) + (-1 * x202);
	const FLT x234 = x42 * x213;
	const FLT x235 = -1 * x46 * x215;
	const FLT x236 = x44 * x195;
	const FLT x237 = (-1 * x42 * x199) + x235 + (-1 * x44 * x192) + x236 + x234 + (x46 * x212);
	const FLT x238 = 2 * x47;
	const FLT x239 = x27 * x133;
	const FLT x240 = x20 * x123;
	const FLT x241 = x23 * x224;
	const FLT x242 = -1 * x24 * x127;
	const FLT x243 = x242 + x241;
	const FLT x244 = x240 + (-1 * x125 * x199) + x243 + (-1 * x122 * x192) + (x204 * x188);
	const FLT x245 = x25 * x129;
	const FLT x246 = x231 * x183;
	const FLT x247 = x27 * x135;
	const FLT x248 = x51 * x25;
	const FLT x249 = x52 * x183;
	const FLT x250 = (x220 * x135) + (-1 * x185 * x134);
	const FLT x251 = x250 + (-1 * x247 * x187) + (x248 * x187) + x246 + (-1 * x249) + (x238 * x229) +
					 (-1 * x201 * x244) + (x237 * x201) + (x239 * x187) + (-1 * x218 * x238) + (-1 * x245 * x187);
	const FLT x252 = x25 * x222;
	const FLT x253 = x48 * x25;
	const FLT x254 = x41 * x135;
	const FLT x255 = x230 * x254;
	const FLT x256 = x22 * x135;
	const FLT x257 = x230 * x134;
	const FLT x258 = x41 * x128;
	const FLT x259 = (-1 * x258 * x183) + (x49 * x183);
	const FLT x260 = (x238 * x200) + x259 + (x219 * x244) + (-1 * x219 * x237) + (-1 * x188 * x133) + (-1 * x255) +
					 (x252 * x186) + (-1 * x211 * x238) + (-1 * x253 * x187) + x257 + (x256 * x187);
	const FLT x261 = (-1 * ((-1 * x233 * x161) + (x260 * x141)) * x162) +
					 (-1 * x150 * ((x251 * x160) + (-1 * ((x233 * x157) + (x260 * x156)) * x158)));
	const FLT x262 = (x25 * x185) + (-1 * x27 * x183) + (x22 * x184) + (-1 * x29 * x230);
	const FLT x263 = x262 * x181;
	const FLT x264 = x22 * x263;
	const FLT x265 = 1.0 / 2.0 * x264;
	const FLT x266 = -1 * x240;
	const FLT x267 = x204 * x263;
	const FLT x268 = 1.0 / 2.0 * x263;
	const FLT x269 = x25 * x268;
	const FLT x270 = (x269 * x122) + (-1 * x29 * x267) + (-1 * x265 * x125) + x243 + x266;
	const FLT x271 = x42 * x27;
	const FLT x272 = x29 * x268;
	const FLT x273 = x42 * x193;
	const FLT x274 = x24 * x197;
	const FLT x275 = x46 * x195;
	const FLT x276 = x275 + (x271 * x268) + (-1 * x46 * x269) + (-1 * x44 * x272) + x273 + (-1 * x274);
	const FLT x277 = x52 * x184;
	const FLT x278 = x235 + (-1 * x236);
	const FLT x279 = x278 + x234 + (x44 * x269) + (-1 * x46 * x272) + (-1 * x42 * x265);
	const FLT x280 = x231 * x184;
	const FLT x281 = x51 * x22;
	const FLT x282 = x20 * x127;
	const FLT x283 = x18 * x224;
	const FLT x284 = x24 * x123;
	const FLT x285 = (x207 * x268) + (-1 * x272 * x122) + (-1 * x25 * x267) + x282 + x283 + (-1 * x284);
	const FLT x286 = x259 + (-1 * x263 * x281) + (-1 * x280) + (x203 * x263) + (x219 * x279) + (-1 * x201 * x276) +
					 x277 + (x201 * x285) + (-1 * x262 * x223) + (-1 * x219 * x270) + (x264 * x129);
	const FLT x287 = x27 * x122;
	const FLT x288 = -1 * x225;
	const FLT x289 = x226 + (x204 * x264) + (-1 * x268 * x287) + (-1 * x227) + x288 + (-1 * x272 * x125);
	const FLT x290 = x44 * x27;
	const FLT x291 = x217 + (-1 * x216);
	const FLT x292 = x291 + (x46 * x265) + (-1 * x214) + (-1 * x290 * x268) + (-1 * x42 * x272);
	const FLT x293 = (x220 * x129) + (-1 * x52 * x185);
	const FLT x294 = (-1 * x254 * x183) + (x183 * x134);
	const FLT x295 = x294 + x293 + (x263 * x248) + (-1 * x263 * x247) + (x201 * x292) + (x239 * x263) + (x238 * x270) +
					 (-1 * x263 * x245) + (-1 * x238 * x279) + (-1 * x201 * x289);
	const FLT x296 = x254 * x184;
	const FLT x297 = x184 * x134;
	const FLT x298 = (-1 * x263 * x253) + x297 + (x262 * x252) + x202 + (-1 * x264 * x133) + (-1 * x296) +
					 (x238 * x276) + (-1 * x219 * x292) + (x219 * x289) + (-1 * x221) + (x263 * x256) +
					 (-1 * x238 * x285);
	const FLT x299 = (-1 * ((-1 * x286 * x161) + (x298 * x141)) * x162) +
					 (-1 * x150 * ((x295 * x160) + (-1 * ((x286 * x157) + (x298 * x156)) * x158)));
	const FLT x300 = x22 * x125;
	const FLT x301 = (x27 * x184) + (-1 * x25 * x230) + (x22 * x183) + (-1 * x29 * x185);
	const FLT x302 = x301 * x181;
	const FLT x303 = 1.0 / 2.0 * x302;
	const FLT x304 = x204 * x302;
	const FLT x305 = x25 * x122;
	const FLT x306 = (-1 * x29 * x304) + x284 + (-1 * x300 * x303) + (x305 * x303) + (-1 * x282) + x283;
	const FLT x307 = x25 * x303;
	const FLT x308 = x29 * x303;
	const FLT x309 = x278 + (-1 * x44 * x308) + (-1 * x234) + (-1 * x46 * x307) + (x271 * x303);
	const FLT x310 = (-1 * x241) + (-1 * x25 * x304) + (x208 * x302) + x242 + (-1 * x206 * x303) + x266;
	const FLT x311 = x49 * x184;
	const FLT x312 = x22 * x129;
	const FLT x313 = x258 * x184;
	const FLT x314 = x22 * x303;
	const FLT x315 = x273 + (-1 * x42 * x314) + (-1 * x275) + (-1 * x46 * x308) + x274 + (x44 * x307);
	const FLT x316 = (-1 * x223 * x301) + (x302 * x312) + (-1 * x219 * x306) + (x201 * x310) + (-1 * x201 * x309) +
					 (x203 * x302) + x313 + (-1 * x246) + (-1 * x281 * x302) + x249 + (-1 * x311) + (x219 * x315);
	const FLT x317 = x209 + (-1 * x210);
	const FLT x318 = x317 + (-1 * x308 * x125) + (x22 * x304) + (-1 * x287 * x303) + x205;
	const FLT x319 = x196 + (-1 * x194);
	const FLT x320 = x198 + x319 + (x46 * x314) + (-1 * x290 * x303) + (-1 * x42 * x308);
	const FLT x321 = x232 + x296 + (-1 * x247 * x302) + (x201 * x320) + (x239 * x302) + (-1 * x201 * x318) +
					 (x248 * x302) + (-1 * x297) + (-1 * x245 * x302) + (x238 * x306) + (-1 * x238 * x315);
	const FLT x322 = x22 * x133;
	const FLT x323 = (x230 * x258) + (-1 * x49 * x230);
	const FLT x324 = x323 + (x238 * x309) + (x256 * x302) + (x252 * x301) + (-1 * x219 * x320) + (-1 * x253 * x302) +
					 x294 + (x219 * x318) + (-1 * x302 * x322) + (-1 * x238 * x310);
	const FLT x325 = (-1 * ((-1 * x316 * x161) + (x324 * x141)) * x162) +
					 (-1 * x150 * ((x321 * x160) + (-1 * ((x316 * x157) + (x324 * x156)) * x158)));
	const FLT x326 = (x25 * x184) + (x27 * x230) + (-1 * x20 * x182) + (-1 * x29 * x183);
	const FLT x327 = x326 * x181;
	const FLT x328 = 1.0 / 2.0 * x327;
	const FLT x329 = x22 * x328;
	const FLT x330 = x29 * x328;
	const FLT x331 = x25 * x328;
	const FLT x332 = x319 + (-1 * x198) + (x44 * x331) + (-1 * x42 * x329) + (-1 * x46 * x330);
	const FLT x333 = x291 + (-1 * x44 * x330) + (x271 * x328) + x214 + (-1 * x46 * x331);
	const FLT x334 = x204 * x327;
	const FLT x335 = x317 + (-1 * x205) + (x305 * x328) + (-1 * x300 * x328) + (-1 * x29 * x334);
	const FLT x336 = x228 + (x208 * x327) + (-1 * x25 * x334) + x288 + (-1 * x206 * x328);
	const FLT x337 = x293 + (-1 * x223 * x326) + (x201 * x336) + x323 + (-1 * x281 * x327) + (-1 * x201 * x333) +
					 (x203 * x327) + (x219 * x332) + (x327 * x312) + (-1 * x219 * x335);
	const FLT x338 = 2 * (x275 + (-1 * x42 * x330) + (-1 * x290 * x328) + (-1 * x273) + (x46 * x329) + x274);
	const FLT x339 = 2 * ((x22 * x334) + (-1 * x283) + (-1 * x287 * x328) + (-1 * x330 * x125) + x282 + x284);
	const FLT x340 = (-1 * x43 * x339) + x255 + x280 + (x238 * x335) + (-1 * x257) + (-1 * x238 * x332) + (x43 * x338) +
					 (-1 * x277) + (x239 * x327) + (-1 * x245 * x327) + (-1 * x247 * x327) + (x248 * x327);
	const FLT x341 = (x50 * x339) + (-1 * x327 * x322) + (-1 * x50 * x338) + (-1 * x238 * x336) + x250 + (x256 * x327) +
					 (x252 * x326) + (x238 * x333) + (-1 * x253 * x327) + x311 + (-1 * x313);
	const FLT x342 = (-1 * ((-1 * x337 * x161) + (x341 * x141)) * x162) +
					 (-1 * x150 * ((x340 * x160) + (-1 * ((x337 * x157) + (x341 * x156)) * x158)));
	const FLT x343 = x32 * x120;
	const FLT x344 = x29 * x343;
	const FLT x345 = x27 * x344;
	const FLT x346 = x25 * x343;
	const FLT x347 = x22 * x346;
	const FLT x348 = x347 + x345;
	const FLT x349 = -1 * x28 * x343;
	const FLT x350 = -1 * x26 * x343;
	const FLT x351 = x121 + x350 + x349;
	const FLT x352 = x343 * x152;
	const FLT x353 = x29 * x346;
	const FLT x354 = (-1 * x353) + x352;
	const FLT x355 = (-1 * ((-1 * x348 * x161) + (x354 * x141)) * x162) +
					 (-1 * x150 * ((x351 * x160) + (-1 * ((x348 * x157) + (x354 * x156)) * x158)));
	const FLT x356 = x27 * x346;
	const FLT x357 = x22 * x344;
	const FLT x358 = (-1 * x357) + x356;
	const FLT x359 = x352 + x353;
	const FLT x360 = (-1 * x30 * x343) + x121;
	const FLT x361 = x360 + x350;
	const FLT x362 = (-1 * ((-1 * x358 * x161) + (x361 * x141)) * x162) +
					 (-1 * x150 * ((x359 * x160) + (-1 * ((x358 * x157) + (x361 * x156)) * x158)));
	const FLT x363 = x360 + x349;
	const FLT x364 = (-1 * x345) + x347;
	const FLT x365 = x356 + x357;
	const FLT x366 = (-1 * ((-1 * x363 * x161) + (x365 * x141)) * x162) +
					 (-1 * x150 * ((x364 * x160) + (-1 * ((x363 * x157) + (x365 * x156)) * x158)));
	const FLT x367 = x35 + x38;
	const FLT x368 = -1 * x138;
	const FLT x369 = 1 + (-1 * x139);
	const FLT x370 = x369 + x368;
	const FLT x371 = x170 + x153;
	const FLT x372 = (-1 * ((-1 * x367 * x161) + (x371 * x141)) * x162) +
					 (-1 * x150 * ((x370 * x160) + (-1 * ((x367 * x157) + (x371 * x156)) * x158)));
	const FLT x373 = x178 + x167;
	const FLT x374 = x153 + x151;
	const FLT x375 = -1 * x172;
	const FLT x376 = x369 + x375;
	const FLT x377 = (-1 * ((-1 * x373 * x161) + (x376 * x141)) * x162) +
					 (-1 * x150 * ((x374 * x160) + (-1 * ((x373 * x157) + (x376 * x156)) * x158)));
	const FLT x378 = 1 + x375 + x368;
	const FLT x379 = x39 + x35;
	const FLT x380 = x167 + x166;
	const FLT x381 = (-1 * ((-1 * x378 * x161) + (x380 * x141)) * x162) +
					 (-1 * x150 * ((x379 * x160) + (-1 * ((x378 * x157) + (x380 * x156)) * x158)));
	const FLT x382 = x98 * x102;
	const FLT x383 = x83 * x382;
	const FLT x384 = x75 * x105;
	const FLT x385 = x74 * x98;
	const FLT x386 = -1 * x80 * x385;
	const FLT x387 = x98 * x103;
	const FLT x388 = x53 * x387;
	const FLT x389 = x388 + x386;
	const FLT x390 = x389 + x383 + x384;
	const FLT x391 = x80 * x387;
	const FLT x392 = -1 * x391;
	const FLT x393 = x53 * x385;
	const FLT x394 = -1 * x393;
	const FLT x395 = x75 * x98;
	const FLT x396 = x83 * x395;
	const FLT x397 = x102 * x105;
	const FLT x398 = x397 + (-1 * x396);
	const FLT x399 = x398 + x392 + x394;
	const FLT x400 = x80 * x382;
	const FLT x401 = x103 * x105;
	const FLT x402 = -1 * x53 * x395;
	const FLT x403 = x83 * x385;
	const FLT x404 = x403 + x402;
	const FLT x405 = x400 + x404 + x401;
	const FLT x406 = (-1 * x390 * sensor_pt[0]) + (x405 * sensor_pt[1]) + (x399 * sensor_pt[2]);
	const FLT x407 = 2 * x115;
	const FLT x408 = 2 * x119;
	const FLT x409 = x53 * x382;
	const FLT x410 = x74 * x105;
	const FLT x411 = x80 * x395;
	const FLT x412 = -1 * x83 * x387;
	const FLT x413 = x412 + x411;
	const FLT x414 = x409 + x413 + x410;
	const FLT x415 = (x390 * sensor_pt[2]) + (-1 * x414 * sensor_pt[1]) + (x399 * sensor_pt[0]);
	const FLT x416 = 2 * x113;
	const FLT x417 = 2 * x124;
	const FLT x418 = (x414 * x417) + (x415 * x416) + (-1 * x406 * x407) + (-1 * x405 * x408);
	const FLT x419 = (x414 * sensor_pt[0]) + (-1 * x405 * sensor_pt[2]) + (x399 * sensor_pt[1]);
	const FLT x420 = 2 * x116;
	const FLT x421 = 2 * x118;
	const FLT x422 = (x408 * x390) + (x406 * x421) + (-1 * x416 * x419) + (-1 * x414 * x420);
	const FLT x423 = (-1 * x415 * x421) + (x407 * x419) + (-1 * x417 * x390) + (x405 * x420);
	const FLT x424 = (-1 * x43 * x423) + (x47 * x418) + (x45 * x422);
	const FLT x425 = (-1 * x47 * x422) + (x50 * x423) + (x45 * x418);
	const FLT x426 = x423 + (x424 * x201) + (-1 * x425 * x219);
	const FLT x427 = (x43 * x422) + (x45 * x423) + (-1 * x50 * x418);
	const FLT x428 = x422 + (-1 * x427 * x201) + (x425 * x238);
	const FLT x429 = x418 + (x427 * x219) + (-1 * x424 * x238);
	const FLT x430 = (-1 * ((-1 * x426 * x161) + (x429 * x141)) * x162) +
					 (-1 * x150 * ((x428 * x160) + (-1 * ((x426 * x157) + (x429 * x156)) * x158)));
	const FLT x431 = -1 * x410;
	const FLT x432 = (-1 * x411) + x412;
	const FLT x433 = x432 + x431 + x409;
	const FLT x434 = -1 * x400;
	const FLT x435 = -1 * x401;
	const FLT x436 = x404 + x434 + x435;
	const FLT x437 = x396 + x397;
	const FLT x438 = x437 + x393 + x392;
	const FLT x439 = (x438 * sensor_pt[1]) + (-1 * x433 * sensor_pt[0]) + (x436 * sensor_pt[2]);
	const FLT x440 = -1 * x383;
	const FLT x441 = (-1 * x388) + x386;
	const FLT x442 = x441 + x440 + x384;
	const FLT x443 = (x433 * sensor_pt[2]) + (-1 * x442 * sensor_pt[1]) + (x436 * sensor_pt[0]);
	const FLT x444 = (x417 * x442) + (x416 * x443) + (-1 * x407 * x439) + (-1 * x408 * x438);
	const FLT x445 = (x442 * sensor_pt[0]) + (-1 * x438 * sensor_pt[2]) + (x436 * sensor_pt[1]);
	const FLT x446 = (x408 * x433) + (x421 * x439) + (-1 * x420 * x442) + (-1 * x416 * x445);
	const FLT x447 = (-1 * x421 * x443) + (x420 * x438) + (-1 * x417 * x433) + (x407 * x445);
	const FLT x448 = (-1 * x43 * x447) + (x47 * x444) + (x45 * x446);
	const FLT x449 = (-1 * x47 * x446) + (x50 * x447) + (x45 * x444);
	const FLT x450 = x447 + (x448 * x201) + (-1 * x449 * x219);
	const FLT x451 = (x45 * x447) + (x43 * x446) + (-1 * x50 * x444);
	const FLT x452 = x446 + (x449 * x238) + (-1 * x451 * x201);
	const FLT x453 = (x451 * x219) + x444 + (-1 * x448 * x238);
	const FLT x454 = (-1 * ((-1 * x450 * x161) + (x453 * x141)) * x162) +
					 (-1 * x150 * ((x452 * x160) + (-1 * ((x450 * x157) + (x453 * x156)) * x158)));
	const FLT x455 = -1 * x409;
	const FLT x456 = x432 + x455 + x410;
	const FLT x457 = -1 * x384;
	const FLT x458 = x389 + x457 + x440;
	const FLT x459 = (-1 * x403) + x402;
	const FLT x460 = x459 + x435 + x400;
	const FLT x461 = (x460 * sensor_pt[0]) + (-1 * x456 * sensor_pt[2]) + (x458 * sensor_pt[1]);
	const FLT x462 = x398 + x391 + x393;
	const FLT x463 = (x456 * sensor_pt[1]) + (-1 * x462 * sensor_pt[0]) + (x458 * sensor_pt[2]);
	const FLT x464 = (x462 * x408) + (x463 * x421) + (-1 * x461 * x416) + (-1 * x460 * x420);
	const FLT x465 = (x462 * sensor_pt[2]) + (-1 * x460 * sensor_pt[1]) + (x458 * sensor_pt[0]);
	const FLT x466 = (x460 * x417) + (-1 * x456 * x408) + (-1 * x463 * x407) + (x465 * x416);
	const FLT x467 = (x461 * x407) + (x456 * x420) + (-1 * x462 * x417) + (-1 * x465 * x421);
	const FLT x468 = (-1 * x43 * x467) + (x45 * x464) + (x47 * x466);
	const FLT x469 = x41 * x466;
	const FLT x470 = (-1 * x47 * x464) + (x29 * x469) + (x50 * x467);
	const FLT x471 = x467 + (x468 * x201) + (-1 * x470 * x219);
	const FLT x472 = (x45 * x467) + (x43 * x464) + (-1 * x22 * x469);
	const FLT x473 = x464 + (x470 * x238) + (-1 * x472 * x201);
	const FLT x474 = x466 + (x472 * x219) + (-1 * x468 * x238);
	const FLT x475 = (-1 * ((-1 * x471 * x161) + (x474 * x141)) * x162) +
					 (-1 * x150 * ((x473 * x160) + (-1 * ((x471 * x157) + (x474 * x156)) * x158)));
	const FLT x476 = x459 + x434 + x401;
	const FLT x477 = x413 + x431 + x455;
	const FLT x478 = x441 + x457 + x383;
	const FLT x479 = (-1 * x476 * sensor_pt[0]) + (x478 * sensor_pt[1]) + (x477 * sensor_pt[2]);
	const FLT x480 = x437 + x391 + x394;
	const FLT x481 = (x476 * sensor_pt[2]) + (-1 * x480 * sensor_pt[1]) + (x477 * sensor_pt[0]);
	const FLT x482 = (x481 * x416) + (-1 * x479 * x407) + (x480 * x417) + (-1 * x478 * x408);
	const FLT x483 = (x480 * sensor_pt[0]) + (x477 * sensor_pt[1]) + (-1 * x478 * sensor_pt[2]);
	const FLT x484 = (-1 * x480 * x420) + (x476 * x408) + (-1 * x483 * x416) + (x479 * x421);
	const FLT x485 = (x478 * x420) + (-1 * x481 * x421) + (-1 * x476 * x417) + (x483 * x407);
	const FLT x486 = (-1 * x43 * x485) + (x47 * x482) + (x45 * x484);
	const FLT x487 = (-1 * x47 * x484) + (x50 * x485) + (x45 * x482);
	const FLT x488 = x485 + (x486 * x201) + (-1 * x487 * x219);
	const FLT x489 = (x43 * x484) + (x45 * x485) + (-1 * x50 * x482);
	const FLT x490 = x484 + (x487 * x238) + (-1 * x489 * x201);
	const FLT x491 = x482 + (x489 * x219) + (-1 * x486 * x238);
	const FLT x492 = (-1 * ((-1 * x488 * x161) + (x491 * x141)) * x162) +
					 (-1 * x150 * ((x490 * x160) + (-1 * ((x488 * x157) + (x491 * x156)) * x158)));
	const FLT x493 = -1 * x99;
	const FLT x494 = dt * dt * dt;
	const FLT x495 = x89 * (1. / (x86 * sqrt(x86)));
	const FLT x496 = x494 * x495;
	const FLT x497 = x80 * x53;
	const FLT x498 = x496 * x497;
	const FLT x499 = x498 * x108;
	const FLT x500 = x80 * x80 * x80;
	const FLT x501 = dt * dt * dt * dt;
	const FLT x502 = 2 * (1. / (x86 * x86)) * x90;
	const FLT x503 = x501 * x502;
	const FLT x504 = 1.0 * x93;
	const FLT x505 = x495 * x504;
	const FLT x506 = x501 * x505;
	const FLT x507 = x78 * x506;
	const FLT x508 = x80 * x84;
	const FLT x509 = x78 * x503;
	const FLT x510 = 2 * x92;
	const FLT x511 = x77 * x510;
	const FLT x512 = x96 * x504;
	const FLT x513 = x77 * x512;
	const FLT x514 = (x500 * x506) + (-1 * x80 * x513) + (x506 * x508) + (x80 * x507) + (-1 * x80 * x509) +
					 (-1 * x500 * x503) + (x80 * x511) + (-1 * x503 * x508);
	const FLT x515 = 1.0 / 2.0 * (1. / (x94 * sqrt(x94)));
	const FLT x516 = x93 * x515;
	const FLT x517 = x514 * x516;
	const FLT x518 = x95 * x100;
	const FLT x519 = 0.5 * x96;
	const FLT x520 = x519 * x518;
	const FLT x521 = x77 * x520;
	const FLT x522 = x97 * x515;
	const FLT x523 = x80 * x522;
	const FLT x524 = x514 * x523;
	const FLT x525 = 0.5 * x91 * x494;
	const FLT x526 = x525 * x114;
	const FLT x527 = x497 * x526;
	const FLT x528 = x53 * x514;
	const FLT x529 = x522 * x107;
	const FLT x530 = x81 * x496;
	const FLT x531 = x76 * x95;
	const FLT x532 = x522 * x104;
	const FLT x533 = x83 * x514;
	const FLT x534 = x525 * x112;
	const FLT x535 = x525 * x106;
	const FLT x536 = x83 * x535;
	const FLT x537 = x95 * x104;
	const FLT x538 = x83 * x496;
	const FLT x539 = x537 * x538;
	const FLT x540 = (-1 * x80 * x539) + (x80 * x536);
	const FLT x541 = x540 + (x531 * x530) + x527 + (-1 * x499) + x493 + (-1 * x81 * x534) + (-1 * x517 * x100) +
					 (-1 * x533 * x532) + (-1 * x80 * x521) + (-1 * x528 * x529) + (x76 * x524);
	const FLT x542 = x83 * x534;
	const FLT x543 = x80 * x542;
	const FLT x544 = x498 * x518;
	const FLT x545 = x76 * x522;
	const FLT x546 = x83 * x545;
	const FLT x547 = x531 * x538;
	const FLT x548 = x80 * x547;
	const FLT x549 = x525 * x117;
	const FLT x550 = x497 * x549;
	const FLT x551 = x522 * x100;
	const FLT x552 = x519 * x108;
	const FLT x553 = x77 * x552;
	const FLT x554 = (-1 * x548) + (-1 * x524 * x104) + (x81 * x535) + (-1 * x514 * x546) + x543 + (x528 * x551) +
					 x544 + (-1 * x550) + (-1 * x530 * x537) + (-1 * x517 * x107) + x111 + (-1 * x80 * x553);
	const FLT x555 = -1 * x109;
	const FLT x556 = x518 * x538;
	const FLT x557 = x80 * x556;
	const FLT x558 = x83 * x549;
	const FLT x559 = x80 * x558;
	const FLT x560 = x519 * x537;
	const FLT x561 = x77 * x560;
	const FLT x562 = (x498 * x531) + (-1 * x497 * x534);
	const FLT x563 = (-1 * x517 * x104) + (x524 * x107) + x562 + (-1 * x80 * x561) + (-1 * x81 * x526) + x557 +
					 (-1 * x559) + (x533 * x551) + (x530 * x108) + x555 + (x528 * x545);
	const FLT x564 = x519 * x531;
	const FLT x565 = x77 * x564;
	const FLT x566 = x83 * x526;
	const FLT x567 = x538 * x108;
	const FLT x568 = (x80 * x567) + (-1 * x80 * x566);
	const FLT x569 = (-1 * x498 * x537) + (x497 * x535);
	const FLT x570 = (-1 * x524 * x100) + (x529 * x533) + (-1 * x76 * x517) + x101 + (-1 * x518 * x530) + (x81 * x549) +
					 (-1 * x80 * x565) + x569 + x568 + (-1 * x528 * x532);
	const FLT x571 = (x570 * sensor_pt[0]) + (-1 * x554 * sensor_pt[2]) + (x563 * sensor_pt[1]);
	const FLT x572 = (x541 * sensor_pt[2]) + (-1 * x570 * sensor_pt[1]) + (x563 * sensor_pt[0]);
	const FLT x573 = (-1 * x421 * x572) + (-1 * x417 * x541) + (x407 * x571) + (x420 * x554);
	const FLT x574 = (-1 * x541 * sensor_pt[0]) + (x554 * sensor_pt[1]) + (x563 * sensor_pt[2]);
	const FLT x575 = (x417 * x570) + (-1 * x407 * x574) + (-1 * x408 * x554) + (x416 * x572);
	const FLT x576 = x41 * x575;
	const FLT x577 = (x408 * x541) + (-1 * x416 * x571) + (x421 * x574) + (-1 * x420 * x570);
	const FLT x578 = (-1 * x47 * x577) + (x50 * x573) + (x29 * x576);
	const FLT x579 = (-1 * x43 * x573) + (x47 * x575) + (x45 * x577);
	const FLT x580 = (-1 * x578 * x219) + x573 + (x579 * x201);
	const FLT x581 = (x43 * x577) + (x45 * x573) + (-1 * x22 * x576);
	const FLT x582 = x577 + (x578 * x238) + (-1 * x581 * x201);
	const FLT x583 = x575 + (-1 * x579 * x238) + (x581 * x219);
	const FLT x584 = (-1 * ((-1 * x580 * x161) + (x583 * x141)) * x162) +
					 (-1 * x150 * ((x582 * x160) + (-1 * ((x580 * x157) + (x583 * x156)) * x158)));
	const FLT x585 = x81 * x83;
	const FLT x586 = x83 * x83 * x83;
	const FLT x587 = (x83 * x511) + (x506 * x586) + (-1 * x503 * x586) + (x506 * x585) + (-1 * x503 * x585) +
					 (x83 * x507) + (-1 * x83 * x509) + (-1 * x83 * x513);
	const FLT x588 = x516 * x587;
	const FLT x589 = x84 * x496;
	const FLT x590 = x53 * x566;
	const FLT x591 = x83 * x587;
	const FLT x592 = x591 * x522;
	const FLT x593 = x53 * x567;
	const FLT x594 = x53 * x587;
	const FLT x595 = x523 * x587;
	const FLT x596 = x590 + x111 + (x76 * x595) + (-1 * x537 * x589) + (-1 * x543) + (-1 * x588 * x100) + (-1 * x593) +
					 (-1 * x594 * x529) + (-1 * x592 * x104) + (x84 * x535) + x548 + (-1 * x83 * x521);
	const FLT x597 = x53 * x542;
	const FLT x598 = x53 * x547;
	const FLT x599 = -1 * x101;
	const FLT x600 = x568 + (-1 * x588 * x104) + x599 + (x594 * x545) + (-1 * x84 * x549) + (x595 * x107) +
					 (x591 * x551) + (-1 * x597) + (x518 * x589) + (-1 * x83 * x561) + x598;
	const FLT x601 = (x53 * x556) + (-1 * x53 * x558);
	const FLT x602 = x540 + (-1 * x76 * x592) + (x594 * x551) + x601 + (-1 * x595 * x104) + (x84 * x534) + x99 +
					 (-1 * x83 * x553) + (-1 * x531 * x589) + (-1 * x588 * x107);
	const FLT x603 = (x602 * sensor_pt[1]) + (-1 * x596 * sensor_pt[0]) + (x600 * sensor_pt[2]);
	const FLT x604 = x523 * x100;
	const FLT x605 = (x53 * x536) + (-1 * x53 * x539);
	const FLT x606 = (x589 * x108) + (-1 * x84 * x526) + (-1 * x604 * x587) + (-1 * x557) + (-1 * x76 * x588) +
					 (-1 * x83 * x565) + x555 + (x592 * x107) + x605 + x559 + (-1 * x594 * x532);
	const FLT x607 = (x596 * sensor_pt[2]) + (-1 * x606 * sensor_pt[1]) + (x600 * sensor_pt[0]);
	const FLT x608 = (x417 * x606) + (-1 * x407 * x603) + (x416 * x607) + (-1 * x408 * x602);
	const FLT x609 = (x606 * sensor_pt[0]) + (-1 * x602 * sensor_pt[2]) + (x600 * sensor_pt[1]);
	const FLT x610 = (-1 * x416 * x609) + (x408 * x596) + (x421 * x603) + (-1 * x420 * x606);
	const FLT x611 = (x420 * x602) + (x407 * x609) + (-1 * x417 * x596) + (-1 * x421 * x607);
	const FLT x612 = (-1 * x43 * x611) + (x47 * x608) + (x45 * x610);
	const FLT x613 = (x50 * x611) + (-1 * x47 * x610) + (x45 * x608);
	const FLT x614 = x611 + (x612 * x201) + (-1 * x613 * x219);
	const FLT x615 = (x45 * x611) + (-1 * x50 * x608) + (x43 * x610);
	const FLT x616 = x610 + (x613 * x238) + (-1 * x615 * x201);
	const FLT x617 = (x615 * x219) + x608 + (-1 * x612 * x238);
	const FLT x618 = (-1 * ((-1 * x614 * x161) + (x617 * x141)) * x162) +
					 (-1 * x150 * ((x616 * x160) + (-1 * ((x614 * x157) + (x617 * x156)) * x158)));
	const FLT x619 = x53 * x506;
	const FLT x620 = x53 * x503;
	const FLT x621 = x77 * x53;
	const FLT x622 = (x53 * x53 * x53) * x501;
	const FLT x623 = (x84 * x619) + (-1 * x81 * x620) + (x81 * x619) + (-1 * x84 * x620) + (x622 * x505) +
					 (-1 * x622 * x502) + (-1 * x621 * x512) + (x621 * x510);
	const FLT x624 = x53 * x623;
	const FLT x625 = x78 * x496;
	const FLT x626 = x83 * x623;
	const FLT x627 = x623 * x516;
	const FLT x628 = x623 * x523;
	const FLT x629 = (x628 * x107) + x601 + x499 + (-1 * x78 * x534) + (-1 * x627 * x104) + (x626 * x551) +
					 (-1 * x527) + x493 + (-1 * x621 * x560) + (x624 * x545) + (x625 * x531);
	const FLT x630 = x562 + (x76 * x628) + (x78 * x526) + (-1 * x626 * x532) + x109 + (-1 * x624 * x529) + x605 +
					 (-1 * x621 * x520) + (-1 * x627 * x100) + (-1 * x625 * x108);
	const FLT x631 = (-1 * x628 * x104) + x597 + (x624 * x551) + (-1 * x623 * x546) + (-1 * x627 * x107) + x599 + x569 +
					 (-1 * x78 * x549) + (-1 * x621 * x552) + (x625 * x518) + (-1 * x598);
	const FLT x632 = (x629 * sensor_pt[2]) + (x631 * sensor_pt[1]) + (-1 * x630 * sensor_pt[0]);
	const FLT x633 = (x78 * x535) + (-1 * x624 * x532) + (-1 * x623 * x604) + (-1 * x76 * x627) + (-1 * x544) + x593 +
					 x550 + (-1 * x625 * x537) + x111 + (x626 * x529) + (-1 * x621 * x564) + (-1 * x590);
	const FLT x634 = (x633 * sensor_pt[0]) + (-1 * x631 * sensor_pt[2]) + (x629 * sensor_pt[1]);
	const FLT x635 = (-1 * x420 * x633) + (x408 * x630) + (x421 * x632) + (-1 * x416 * x634);
	const FLT x636 = x41 * x635;
	const FLT x637 = (x630 * sensor_pt[2]) + (-1 * x633 * sensor_pt[1]) + (x629 * sensor_pt[0]);
	const FLT x638 = (x417 * x633) + (-1 * x407 * x632) + (x416 * x637) + (-1 * x408 * x631);
	const FLT x639 = (x407 * x634) + (x420 * x631) + (-1 * x417 * x630) + (-1 * x421 * x637);
	const FLT x640 = x41 * x639;
	const FLT x641 = (-1 * x27 * x640) + (x29 * x636) + (x47 * x638);
	const FLT x642 = (-1 * x25 * x636) + (x45 * x638) + (x50 * x639);
	const FLT x643 = x639 + (x641 * x201) + (-1 * x642 * x219);
	const FLT x644 = 2 * ((x29 * x640) + (x27 * x636) + (-1 * x50 * x638));
	const FLT x645 = x635 + (x642 * x238) + (-1 * x43 * x644);
	const FLT x646 = x638 + (x50 * x644) + (-1 * x641 * x238);
	const FLT x647 = (-1 * ((-1 * x643 * x161) + (x646 * x141)) * x162) +
					 (-1 * x150 * ((x645 * x160) + (-1 * ((x643 * x157) + (x646 * x156)) * x158)));
	const FLT x648 = dt * x38;
	const FLT x649 = dt * x35;
	const FLT x650 = x649 + x648;
	const FLT x651 = -1 * dt * x138;
	const FLT x652 = (-1 * dt * x139) + dt;
	const FLT x653 = x652 + x651;
	const FLT x654 = dt * x153;
	const FLT x655 = dt * x151;
	const FLT x656 = (-1 * x655) + x654;
	const FLT x657 = (-1 * ((-1 * x650 * x161) + (x656 * x141)) * x162) +
					 (-1 * x150 * ((x653 * x160) + (-1 * ((x650 * x157) + (x656 * x156)) * x158)));
	const FLT x658 = dt * x167;
	const FLT x659 = dt * x166;
	const FLT x660 = (-1 * x659) + x658;
	const FLT x661 = x654 + x655;
	const FLT x662 = -1 * dt * x172;
	const FLT x663 = x652 + x662;
	const FLT x664 = (-1 * ((-1 * x660 * x161) + (x663 * x141)) * x162) +
					 (-1 * x150 * ((x661 * x160) + (-1 * ((x660 * x157) + (x663 * x156)) * x158)));
	const FLT x665 = x651 + dt + x662;
	const FLT x666 = (-1 * x648) + x649;
	const FLT x667 = x658 + x659;
	const FLT x668 = (-1 * ((-1 * x665 * x161) + (x667 * x141)) * x162) +
					 (-1 * x150 * ((x666 * x160) + (-1 * ((x665 * x157) + (x667 * x156)) * x158)));
	const FLT x669 = x136 * x150 * x159;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[0]) / sizeof(FLT),
						x163 + (x145 * ((-1 * x140 * x141) + (x40 * x137))) + (x165 * x163));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[1]) / sizeof(FLT),
						x175 + (((-1 * x171 * x141) + (x169 * x137)) * x145) + (x165 * x175));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[2]) / sizeof(FLT),
						x180 + (((-1 * x177 * x141) + (x176 * x137)) * x145) + (x165 * x180));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						x261 + (((-1 * x251 * x141) + (x233 * x137)) * x145) + (x261 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						x299 + (((-1 * x295 * x141) + (x286 * x137)) * x145) + (x299 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						(((-1 * x321 * x141) + (x316 * x137)) * x145) + x325 + (x325 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						(((-1 * x340 * x141) + (x337 * x137)) * x145) + x342 + (x342 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Acc[0]) / sizeof(FLT),
						x355 + (((-1 * x351 * x141) + (x348 * x137)) * x145) + (x355 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Acc[1]) / sizeof(FLT),
						x362 + (((-1 * x359 * x141) + (x358 * x137)) * x145) + (x362 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Acc[2]) / sizeof(FLT),
						x366 + (((-1 * x364 * x141) + (x363 * x137)) * x145) + (x366 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[0]) / sizeof(FLT),
						x372 + (((-1 * x370 * x141) + (x367 * x137)) * x145) + (x372 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[1]) / sizeof(FLT),
						x377 + (((-1 * x374 * x141) + (x373 * x137)) * x145) + (x377 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[2]) / sizeof(FLT),
						x381 + (((-1 * x379 * x141) + (x378 * x137)) * x145) + (x381 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT),
						x430 + (((-1 * x428 * x141) + (x426 * x137)) * x145) + (x430 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT),
						x454 + (((-1 * x452 * x141) + (x450 * x137)) * x145) + (x454 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT),
						x475 + (((-1 * x473 * x141) + (x471 * x137)) * x145) + (x475 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT),
						x492 + (((-1 * x490 * x141) + (x488 * x137)) * x145) + (x492 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x584 + (((-1 * x582 * x141) + (x580 * x137)) * x145) + (x584 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x618 + (((-1 * x616 * x141) + (x614 * x137)) * x145) + (x618 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x647 + (((-1 * x645 * x141) + (x643 * x137)) * x145) + (x647 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[0]) / sizeof(FLT),
						x657 + (((-1 * x653 * x141) + (x650 * x137)) * x145) + (x657 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[1]) / sizeof(FLT),
						x664 + (((-1 * x661 * x141) + (x660 * x137)) * x145) + (x664 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[2]) / sizeof(FLT),
						x668 + (((-1 * x666 * x141) + (x665 * x137)) * x145) + (x668 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD1.curve) / sizeof(FLT), x143 * x143);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD1.gibmag) / sizeof(FLT), -1 * cos(x164));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD1.gibpha) / sizeof(FLT), x165);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD1.phase) / sizeof(FLT), -1 + (-1 * x165));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD1.tilt) / sizeof(FLT),
						(-1 * x669 * x165) + (-1 * x669));
}

// Full version Jacobian of SurviveJointKalmanErrorModel_LightMeas_y_gen1 wrt [(*_x0).Lighthouse.Pos[0],
// (*_x0).Lighthouse.Pos[1], (*_x0).Lighthouse.Pos[2], (*_x0).Lighthouse.Rot[0], (*_x0).Lighthouse.Rot[1],
// (*_x0).Lighthouse.Rot[2], (*_x0).Lighthouse.Rot[3], (*_x0).Object.Acc[0], (*_x0).Object.Acc[1], (*_x0).Object.Acc[2],
// (*_x0).Object.IMUBias.AccBias[0], (*_x0).Object.IMUBias.AccBias[1], (*_x0).Object.IMUBias.AccBias[2],
// (*_x0).Object.IMUBias.AccScale[0], (*_x0).Object.IMUBias.AccScale[1], (*_x0).Object.IMUBias.AccScale[2],
// (*_x0).Object.IMUBias.GyroBias[0], (*_x0).Object.IMUBias.GyroBias[1], (*_x0).Object.IMUBias.GyroBias[2],
// (*_x0).Object.IMUBias.IMUCorrection[0], (*_x0).Object.IMUBias.IMUCorrection[1],
// (*_x0).Object.IMUBias.IMUCorrection[2], (*_x0).Object.IMUBias.IMUCorrection[3], (*_x0).Object.Pose.Pos[0],
// (*_x0).Object.Pose.Pos[1], (*_x0).Object.Pose.Pos[2], (*_x0).Object.Pose.Rot[0], (*_x0).Object.Pose.Rot[1],
// (*_x0).Object.Pose.Rot[2], (*_x0).Object.Pose.Rot[3], (*_x0).Object.Velocity.AxisAngleRot[0],
// (*_x0).Object.Velocity.AxisAngleRot[1], (*_x0).Object.Velocity.AxisAngleRot[2], (*_x0).Object.Velocity.Pos[0],
// (*_x0).Object.Velocity.Pos[1], (*_x0).Object.Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f4d1c3388b0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c338b80>, <cnkalman.codegen.WrapMember object at 0x7f4d1c338970>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c338c40>, <cnkalman.codegen.WrapMember object at 0x7f4d1c338910>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c338be0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c338a30>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c338d00>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3389d0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c338ca0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3237f0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c338ac0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c338850>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c338b20>]

static inline void SurviveJointKalmanErrorModel_LightMeas_y_gen1_jac_x0_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveJointKalmanModel *_x0,
	const SurviveJointKalmanErrorModel *error_model, const FLT *sensor_pt) {
	if (hx != 0) {
		hx->data[0] = SurviveJointKalmanErrorModel_LightMeas_y_gen1(dt, _x0, error_model, sensor_pt);
	}
	if (Hx != 0) {
		SurviveJointKalmanErrorModel_LightMeas_y_gen1_jac_x0(Hx, dt, _x0, error_model, sensor_pt);
	}
}
// Jacobian of SurviveJointKalmanErrorModel_LightMeas_y_gen1 wrt [(*error_model).Lighthouse.AxisAngleRot[0],
// (*error_model).Lighthouse.AxisAngleRot[1], (*error_model).Lighthouse.AxisAngleRot[2],
// (*error_model).Lighthouse.Pos[0], (*error_model).Lighthouse.Pos[1], (*error_model).Lighthouse.Pos[2],
// (*error_model).Object.Acc[0], (*error_model).Object.Acc[1], (*error_model).Object.Acc[2],
// (*error_model).Object.IMUBias.AccBias[0], (*error_model).Object.IMUBias.AccBias[1],
// (*error_model).Object.IMUBias.AccBias[2], (*error_model).Object.IMUBias.AccScale[0],
// (*error_model).Object.IMUBias.AccScale[1], (*error_model).Object.IMUBias.AccScale[2],
// (*error_model).Object.IMUBias.GyroBias[0], (*error_model).Object.IMUBias.GyroBias[1],
// (*error_model).Object.IMUBias.GyroBias[2], (*error_model).Object.IMUBias.IMUCorrection[0],
// (*error_model).Object.IMUBias.IMUCorrection[1], (*error_model).Object.IMUBias.IMUCorrection[2],
// (*error_model).Object.Pose.AxisAngleRot[0], (*error_model).Object.Pose.AxisAngleRot[1],
// (*error_model).Object.Pose.AxisAngleRot[2], (*error_model).Object.Pose.Pos[0], (*error_model).Object.Pose.Pos[1],
// (*error_model).Object.Pose.Pos[2], (*error_model).Object.Velocity.AxisAngleRot[0],
// (*error_model).Object.Velocity.AxisAngleRot[1], (*error_model).Object.Velocity.AxisAngleRot[2],
// (*error_model).Object.Velocity.Pos[0], (*error_model).Object.Velocity.Pos[1], (*error_model).Object.Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f4d1c33a070>, <cnkalman.codegen.WrapMember object at 0x7f4d1c33a340>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c33a130>, <cnkalman.codegen.WrapMember object at 0x7f4d1c33a400>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c33a0d0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c33a3a0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c33a1f0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c33a4c0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c33a190>, <cnkalman.codegen.WrapMember object at 0x7f4d1c33a460>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c339e80>, <cnkalman.codegen.WrapMember object at 0x7f4d1c33a280>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c339fd0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c33a2e0>]
static inline void SurviveJointKalmanErrorModel_LightMeas_y_gen1_jac_error_model(
	CnMat *Hx, const FLT dt, const SurviveJointKalmanModel *_x0, const SurviveJointKalmanErrorModel *error_model,
	const FLT *sensor_pt) {
	const FLT x0 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[0];
	const FLT x1 = sin(x0);
	const FLT x2 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[2];
	const FLT x3 = cos(x2);
	const FLT x4 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[1];
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = x1 * x6;
	const FLT x8 = cos(x4);
	const FLT x9 = cos(x0);
	const FLT x10 = sin(x2);
	const FLT x11 = x9 * x10;
	const FLT x12 = x8 * x11;
	const FLT x13 = x12 + (-1 * x7);
	const FLT x14 = x1 * x10;
	const FLT x15 = x8 * x14;
	const FLT x16 = x6 * x9;
	const FLT x17 = x16 + x15;
	const FLT x18 = x5 * x14;
	const FLT x19 = x3 * x8;
	const FLT x20 = x9 * x19;
	const FLT x21 = x20 + x18;
	const FLT x22 = x5 * x11;
	const FLT x23 = x1 * x19;
	const FLT x24 = x23 + (-1 * x22);
	const FLT x25 = (x24 * x24) + (x21 * x21) + (x13 * x13) + (x17 * x17);
	const FLT x26 = 1.0 / 2.0 * (1. / (x25 * sqrt(x25)));
	const FLT x27 = x26 * (*_x0).Lighthouse.Rot[1];
	const FLT x28 = 0.5 * x20;
	const FLT x29 = 0.5 * x18;
	const FLT x30 = x29 + x28;
	const FLT x31 = 2 * x24;
	const FLT x32 = 0.5 * x23;
	const FLT x33 = -1 * x32;
	const FLT x34 = 0.5 * x22;
	const FLT x35 = x34 + x33;
	const FLT x36 = 2 * x21;
	const FLT x37 = 0.5 * x15;
	const FLT x38 = -0.5 * x16;
	const FLT x39 = x38 + (-1 * x37);
	const FLT x40 = 2 * x13;
	const FLT x41 = 0.5 * x7;
	const FLT x42 = -1 * x41;
	const FLT x43 = 0.5 * x12;
	const FLT x44 = x43 + x42;
	const FLT x45 = 2 * x17;
	const FLT x46 = (x44 * x45) + (x40 * x39) + (x30 * x31) + (x36 * x35);
	const FLT x47 = x46 * x13;
	const FLT x48 = 1. / sqrt(x25);
	const FLT x49 = x48 * (*_x0).Lighthouse.Rot[1];
	const FLT x50 = -1 * x49 * x39;
	const FLT x51 = x26 * (*_x0).Lighthouse.Rot[0];
	const FLT x52 = x46 * x17;
	const FLT x53 = x26 * (*_x0).Lighthouse.Rot[2];
	const FLT x54 = x53 * x46;
	const FLT x55 = x48 * (*_x0).Lighthouse.Rot[3];
	const FLT x56 = x55 * x30;
	const FLT x57 = x48 * (*_x0).Lighthouse.Rot[2];
	const FLT x58 = x46 * x24;
	const FLT x59 = x58 * x26;
	const FLT x60 = x48 * (*_x0).Lighthouse.Rot[0];
	const FLT x61 = (x60 * x44) + (-1 * x59 * (*_x0).Lighthouse.Rot[3]) + (x57 * x35) + x50 + (-1 * x52 * x51) +
					(x47 * x27) + (-1 * x54 * x21) + x56;
	const FLT x62 = (-1 * x49 * x24) + (x60 * x21) + (-1 * x55 * x13) + (-1 * x57 * x17);
	const FLT x63 = (x49 * x17) + (x60 * x13) + (x55 * x21) + (-1 * x57 * x24);
	const FLT x64 = x63 * x63;
	const FLT x65 = (-1 * x49 * x13) + (x60 * x17) + (x55 * x24) + (x57 * x21);
	const FLT x66 = x65 * x65;
	const FLT x67 = (-1 * x55 * x17) + (x49 * x21) + (x57 * x13) + (x60 * x24);
	const FLT x68 = x67 * x67;
	const FLT x69 = x68 + (x62 * x62) + x64 + x66;
	const FLT x70 = 1. / sqrt(x69);
	const FLT x71 = (*_x0).Object.Velocity.AxisAngleRot[2] + (*error_model).Object.Velocity.AxisAngleRot[2];
	const FLT x72 = dt * dt;
	const FLT x73 = x71 * x71;
	const FLT x74 = x73 * x72;
	const FLT x75 = (*_x0).Object.Velocity.AxisAngleRot[0] + (*error_model).Object.Velocity.AxisAngleRot[0];
	const FLT x76 = x75 * x75;
	const FLT x77 = x72 * x76;
	const FLT x78 = (*_x0).Object.Velocity.AxisAngleRot[1] + (*error_model).Object.Velocity.AxisAngleRot[1];
	const FLT x79 = x78 * x78;
	const FLT x80 = x72 * x79;
	const FLT x81 = 1e-10 + x80 + x74 + x77;
	const FLT x82 = sqrt(x81);
	const FLT x83 = 0.5 * x82;
	const FLT x84 = sin(x83);
	const FLT x85 = x84 * x84;
	const FLT x86 = 1. / x81;
	const FLT x87 = x85 * x86;
	const FLT x88 = cos(x83);
	const FLT x89 = (x88 * x88) + (x87 * x74) + (x87 * x77) + (x80 * x87);
	const FLT x90 = 1. / sqrt(x89);
	const FLT x91 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[1];
	const FLT x92 = sin(x91);
	const FLT x93 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[2];
	const FLT x94 = sin(x93);
	const FLT x95 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[0];
	const FLT x96 = sin(x95);
	const FLT x97 = x96 * x94;
	const FLT x98 = x92 * x97;
	const FLT x99 = cos(x93);
	const FLT x100 = cos(x91);
	const FLT x101 = cos(x95);
	const FLT x102 = x101 * x100;
	const FLT x103 = x99 * x102;
	const FLT x104 = x103 + x98;
	const FLT x105 = x97 * x100;
	const FLT x106 = x92 * x101;
	const FLT x107 = x99 * x106;
	const FLT x108 = x107 + x105;
	const FLT x109 = x99 * x96;
	const FLT x110 = x92 * x109;
	const FLT x111 = x94 * x102;
	const FLT x112 = x111 + (-1 * x110);
	const FLT x113 = x94 * x106;
	const FLT x114 = x100 * x109;
	const FLT x115 = x114 + (-1 * x113);
	const FLT x116 = (x112 * x112) + (x108 * x108) + (x115 * x115) + (x104 * x104);
	const FLT x117 = 1. / sqrt(x116);
	const FLT x118 = x117 * (*_x0).Object.Pose.Rot[3];
	const FLT x119 = x117 * (*_x0).Object.Pose.Rot[2];
	const FLT x120 = x117 * (*_x0).Object.Pose.Rot[0];
	const FLT x121 = x108 * x117;
	const FLT x122 = (x121 * (*_x0).Object.Pose.Rot[1]) + (x112 * x120) + (x104 * x118) + (-1 * x119 * x115);
	const FLT x123 = x90 * x122;
	const FLT x124 = (1. / x82) * x84;
	const FLT x125 = dt * x124;
	const FLT x126 = x123 * x125;
	const FLT x127 = x117 * (*_x0).Object.Pose.Rot[1];
	const FLT x128 = (-1 * x112 * x127) + (x121 * (*_x0).Object.Pose.Rot[0]) + (x104 * x119) + (x118 * x115);
	const FLT x129 = x90 * x125;
	const FLT x130 = x128 * x129;
	const FLT x131 = x104 * x117;
	const FLT x132 = (x131 * (*_x0).Object.Pose.Rot[0]) + (-1 * x112 * x118) + (-1 * x115 * x127) + (-1 * x108 * x119);
	const FLT x133 = x88 * x90;
	const FLT x134 = x133 * x132;
	const FLT x135 = (x115 * x120) + (x131 * (*_x0).Object.Pose.Rot[1]) + (-1 * x108 * x118) + (x112 * x119);
	const FLT x136 = x129 * x135;
	const FLT x137 = (-1 * x75 * x136) + x134 + (-1 * x71 * x126) + (-1 * x78 * x130);
	const FLT x138 = x129 * x132;
	const FLT x139 = x122 * x133;
	const FLT x140 = (x75 * x130) + (-1 * x78 * x136) + (x71 * x138) + x139;
	const FLT x141 = x133 * x135;
	const FLT x142 = (-1 * x71 * x130) + (x75 * x138) + x141 + (x78 * x126);
	const FLT x143 = (-1 * x142 * sensor_pt[2]) + (x137 * sensor_pt[1]) + (x140 * sensor_pt[0]);
	const FLT x144 = x128 * x133;
	const FLT x145 = (-1 * x75 * x126) + (x78 * x138) + (x71 * x136) + x144;
	const FLT x146 = (-1 * x145 * sensor_pt[0]) + (x137 * sensor_pt[2]) + (x142 * sensor_pt[1]);
	const FLT x147 = dt * fabs(dt);
	const FLT x148 = 1.0 / 2.0 * x147;
	const FLT x149 = (*_x0).Object.Pose.Pos[0] + (2 * ((x146 * x145) + (-1 * x140 * x143))) +
					 (*error_model).Object.Pose.Pos[0] +
					 (x148 * ((*_x0).Object.Acc[0] + (*error_model).Object.Acc[0])) + sensor_pt[0] +
					 (dt * ((*_x0).Object.Velocity.Pos[0] + (*error_model).Object.Velocity.Pos[0]));
	const FLT x150 = x70 * x149;
	const FLT x151 = (-1 * x140 * sensor_pt[1]) + (x137 * sensor_pt[0]) + (x145 * sensor_pt[2]);
	const FLT x152 = (dt * ((*_x0).Object.Velocity.Pos[2] + (*error_model).Object.Velocity.Pos[2])) +
					 (*error_model).Object.Pose.Pos[2] + sensor_pt[2] + (2 * ((x142 * x143) + (-1 * x145 * x151))) +
					 (x148 * ((*_x0).Object.Acc[2] + (*error_model).Object.Acc[2])) + (*_x0).Object.Pose.Pos[2];
	const FLT x153 = x70 * x152;
	const FLT x154 = (*_x0).Object.Pose.Pos[1] + (2 * ((x140 * x151) + (-1 * x142 * x146))) +
					 (x148 * ((*_x0).Object.Acc[1] + (*error_model).Object.Acc[1])) +
					 (*error_model).Object.Pose.Pos[1] + sensor_pt[1] +
					 (dt * ((*_x0).Object.Velocity.Pos[1] + (*error_model).Object.Velocity.Pos[1]));
	const FLT x155 = x70 * x154;
	const FLT x156 = (x62 * x150) + (x63 * x155) + (-1 * x65 * x153);
	const FLT x157 = 2 * x70;
	const FLT x158 = x156 * x157;
	const FLT x159 = 1.0 / 2.0 * x149;
	const FLT x160 = -1 * x49 * x30;
	const FLT x161 = x46 * x21;
	const FLT x162 = x26 * (*_x0).Lighthouse.Rot[3];
	const FLT x163 = x55 * x39;
	const FLT x164 = x160 + (x59 * (*_x0).Lighthouse.Rot[1]) + (x60 * x35) + (-1 * x57 * x44) + (-1 * x163) +
					 (x53 * x52) + (-1 * x51 * x161) + (x47 * x162);
	const FLT x165 = 2 * x62;
	const FLT x166 = x52 * x26;
	const FLT x167 = x57 * x39;
	const FLT x168 = x60 * x30;
	const FLT x169 = (-1 * x55 * x44) + x168 + (-1 * x51 * x58) + (x166 * (*_x0).Lighthouse.Rot[3]) +
					 (-1 * x27 * x161) + (x49 * x35) + (-1 * x54 * x13) + x167;
	const FLT x170 = 2 * x67;
	const FLT x171 = x57 * x30;
	const FLT x172 = x60 * x39;
	const FLT x173 = (-1 * x166 * (*_x0).Lighthouse.Rot[1]) + x172 + (x53 * x58) + (-1 * x51 * x47) + (x44 * x49) +
					 (-1 * x161 * x162) + (x55 * x35) + (-1 * x171);
	const FLT x174 = 2 * x63;
	const FLT x175 = 2 * x65;
	const FLT x176 = (x174 * x173) + (x165 * x164) + (x61 * x175) + (x169 * x170);
	const FLT x177 = 1. / (x69 * sqrt(x69));
	const FLT x178 = x62 * x177;
	const FLT x179 = x178 * x176;
	const FLT x180 = x63 * x177;
	const FLT x181 = x176 * x180;
	const FLT x182 = 1.0 / 2.0 * x154;
	const FLT x183 = x65 * x177;
	const FLT x184 = x176 * x183;
	const FLT x185 = 1.0 / 2.0 * x152;
	const FLT x186 =
		(x185 * x184) + (-1 * x61 * x153) + (-1 * x179 * x159) + (x173 * x155) + (-1 * x181 * x182) + (x164 * x150);
	const FLT x187 = x70 * x175;
	const FLT x188 = (*_x0).Lighthouse.Pos[2] + (*error_model).Lighthouse.Pos[2];
	const FLT x189 = 1.0 / 2.0 * x188;
	const FLT x190 = x67 * x177;
	const FLT x191 = x176 * x190;
	const FLT x192 = (*_x0).Lighthouse.Pos[1] + (*error_model).Lighthouse.Pos[1];
	const FLT x193 = 1.0 / 2.0 * x179;
	const FLT x194 = x70 * x192;
	const FLT x195 = (*_x0).Lighthouse.Pos[0] + (*error_model).Lighthouse.Pos[0];
	const FLT x196 = 1.0 / 2.0 * x195;
	const FLT x197 = x70 * x195;
	const FLT x198 = x70 * x188;
	const FLT x199 =
		(-1 * x192 * x193) + (-1 * x189 * x191) + (x164 * x194) + (x181 * x196) + (x169 * x198) + (-1 * x173 * x197);
	const FLT x200 = x70 * x170;
	const FLT x201 = (x67 * x198) + (x62 * x194) + (-1 * x63 * x197);
	const FLT x202 = x201 * x176;
	const FLT x203 = (x63 * x194) + (-1 * x65 * x198) + (x62 * x197);
	const FLT x204 = x203 * x183;
	const FLT x205 = (x67 * x153) + (x62 * x155) + (-1 * x63 * x150);
	const FLT x206 = x70 * x205;
	const FLT x207 = 2 * x206;
	const FLT x208 = x201 * x157;
	const FLT x209 = x176 * x156;
	const FLT x210 = 1.0 / 2.0 * x192;
	const FLT x211 =
		(x189 * x184) + (-1 * x210 * x181) + (-1 * x179 * x196) + (x173 * x194) + (-1 * x61 * x198) + (x164 * x197);
	const FLT x212 = x203 * x157;
	const FLT x213 =
		(x181 * x159) + (x164 * x155) + (-1 * x179 * x182) + (-1 * x185 * x191) + (x169 * x153) + (-1 * x173 * x150);
	const FLT x214 = x205 * x190;
	const FLT x215 = (x214 * x176) + (x200 * x199) + (x208 * x169) + (-1 * x211 * x187) + (x61 * x158) +
					 (-1 * x209 * x183) + (x204 * x176) + (-1 * x202 * x190) + (x187 * x186) + (-1 * x61 * x212) +
					 (-1 * x207 * x169) + (-1 * x213 * x200);
	const FLT x216 = (-1 * x67 * x155) + (x65 * x150) + (x62 * x153);
	const FLT x217 = x70 * x65;
	const FLT x218 = x70 * x63;
	const FLT x219 = (x65 * x197) + (x62 * x198) + (-1 * x67 * x194);
	const FLT x220 =
		x149 + (2 * ((-1 * x217 * x216) + (x63 * x206))) + (-1 * (x195 + (2 * ((-1 * x219 * x217) + (x218 * x201)))));
	const FLT x221 = x70 * x67;
	const FLT x222 =
		x152 + (-1 * (x188 + (2 * ((-1 * x201 * x221) + (x217 * x203))))) + (2 * ((-1 * x67 * x206) + (x217 * x156)));
	const FLT x223 = x222 * x222;
	const FLT x224 = 1. / x223;
	const FLT x225 = x220 * x224;
	const FLT x226 = x219 * x176;
	const FLT x227 =
		(-1 * x179 * x185) + (x182 * x191) + (-1 * x184 * x159) + (x164 * x153) + (-1 * x169 * x155) + (x61 * x150);
	const FLT x228 =
		(-1 * x188 * x193) + (x210 * x191) + (-1 * x184 * x196) + (x61 * x197) + (-1 * x169 * x194) + (x164 * x198);
	const FLT x229 = x70 * x174;
	const FLT x230 = x216 * x157;
	const FLT x231 = x219 * x157;
	const FLT x232 = (-1 * x208 * x173) + (x228 * x187) + (x202 * x180) + (-1 * x227 * x187) + (x207 * x173) +
					 (-1 * x229 * x199) + (-1 * x226 * x183) + (x216 * x184) + (x213 * x229) + (-1 * x205 * x181) +
					 (-1 * x61 * x230) + (x61 * x231);
	const FLT x233 = 1. / x222;
	const FLT x234 = -1 * x222;
	const FLT x235 = atan2(x220, x234);
	const FLT x236 = x220 * x220;
	const FLT x237 = 2 * (1. / (x223 + x236)) * x235 * x223 * ((*error_model).BSD1.curve + (*_x0).BSD1.curve);
	const FLT x238 =
		x154 + (-1 * (x192 + (2 * ((-1 * x218 * x203) + (x219 * x221))))) + (2 * ((-1 * x218 * x156) + (x216 * x221)));
	const FLT x239 = x223 + (x238 * x238);
	const FLT x240 = 1. / x239;
	const FLT x241 = (*error_model).BSD1.tilt + (*_x0).BSD1.tilt;
	const FLT x242 = 1. / sqrt(1 + (-1 * x236 * x240 * (x241 * x241)));
	const FLT x243 = x216 * x190;
	const FLT x244 = (-1 * x229 * x186) + (-1 * x231 * x169) + (x209 * x180) + (x230 * x169) + (-1 * x173 * x158) +
					 (x226 * x190) + (-1 * x200 * x228) + (-1 * x203 * x181) + (x212 * x173) + (x200 * x227) +
					 (x211 * x229) + (-1 * x243 * x176);
	const FLT x245 = 2 * x238;
	const FLT x246 = 2 * x222;
	const FLT x247 = 1.0 / 2.0 * (1. / (x239 * sqrt(x239))) * x220 * x241;
	const FLT x248 = 1. / sqrt(x239);
	const FLT x249 = x241 * x248;
	const FLT x250 = x238 * x224;
	const FLT x251 = x223 * x240;
	const FLT x252 = (-1 * ((-1 * x215 * x250) + (x233 * x244)) * x251) +
					 (-1 * x242 * ((x232 * x249) + (-1 * ((x215 * x246) + (x244 * x245)) * x247)));
	const FLT x253 = 1.5707963267949 + (*error_model).BSD1.gibpha + (-1 * asin(x220 * x249)) +
					 (-1 * ((*error_model).BSD1.phase + (*_x0).BSD1.phase)) + (*_x0).BSD1.gibpha +
					 (-1 * atan2(-1 * x238, x234));
	const FLT x254 = sin(x253) * ((*error_model).BSD1.gibmag + (*_x0).BSD1.gibmag);
	const FLT x255 = x37 + x38;
	const FLT x256 = -1 * x43;
	const FLT x257 = x256 + x42;
	const FLT x258 = -1 * x34;
	const FLT x259 = x33 + x258;
	const FLT x260 = (-1 * x29) + x28;
	const FLT x261 = (x40 * x259) + (x45 * x260) + (x31 * x257) + (x36 * x255);
	const FLT x262 = x17 * x261;
	const FLT x263 = x13 * x261;
	const FLT x264 = x24 * x261;
	const FLT x265 = x21 * x261;
	const FLT x266 = x26 * x265;
	const FLT x267 = x48 * x260;
	const FLT x268 = (-1 * x267 * (*_x0).Lighthouse.Rot[3]) + (-1 * x266 * (*_x0).Lighthouse.Rot[1]) + (x262 * x162) +
					 (-1 * x51 * x264) + (x49 * x255) + (x57 * x259) + (-1 * x53 * x263) + (x60 * x257);
	const FLT x269 = x48 * x255;
	const FLT x270 = (x269 * (*_x0).Lighthouse.Rot[0]) + (-1 * x266 * (*_x0).Lighthouse.Rot[0]) + (x263 * x162) +
					 (-1 * x49 * x257) + (-1 * x267 * (*_x0).Lighthouse.Rot[2]) + (-1 * x55 * x259) + (x27 * x264) +
					 (x53 * x262);
	const FLT x271 = (x269 * (*_x0).Lighthouse.Rot[3]) + (-1 * x51 * x263) + (-1 * x266 * (*_x0).Lighthouse.Rot[3]) +
					 (x49 * x260) + (-1 * x57 * x257) + (x53 * x264) + (-1 * x27 * x262) + (x60 * x259);
	const FLT x272 = (-1 * x53 * x265) + (x269 * (*_x0).Lighthouse.Rot[2]) + (x55 * x257) + (-1 * x51 * x262) +
					 (-1 * x264 * x162) + (x27 * x263) + (-1 * x49 * x259) + (x267 * (*_x0).Lighthouse.Rot[0]);
	const FLT x273 = (x272 * x175) + (x268 * x170) + (x271 * x174) + (x270 * x165);
	const FLT x274 = x273 * x190;
	const FLT x275 = 1.0 / 2.0 * x274;
	const FLT x276 = x273 * x178;
	const FLT x277 = x273 * x180;
	const FLT x278 =
		(x268 * x153) + (-1 * x276 * x182) + (-1 * x275 * x152) + (x270 * x155) + (x277 * x159) + (-1 * x271 * x150);
	const FLT x279 = x273 * x183;
	const FLT x280 = x70 * x271;
	const FLT x281 =
		(-1 * x276 * x196) + (x279 * x189) + (x270 * x197) + (-1 * x210 * x277) + (-1 * x272 * x198) + (x280 * x192);
	const FLT x282 =
		(x280 * x154) + (-1 * x276 * x159) + (x270 * x150) + (x279 * x185) + (-1 * x277 * x182) + (-1 * x272 * x153);
	const FLT x283 = 1.0 / 2.0 * x276;
	const FLT x284 =
		(x268 * x198) + (x270 * x194) + (-1 * x275 * x188) + (-1 * x280 * x195) + (x277 * x196) + (-1 * x283 * x192);
	const FLT x285 = (-1 * x201 * x274) + (x282 * x187) + (x208 * x268) + (-1 * x281 * x187) + (x200 * x284) +
					 (x272 * x158) + (-1 * x200 * x278) + (-1 * x207 * x268) + (x214 * x273) + (x204 * x273) +
					 (-1 * x212 * x272) + (-1 * x279 * x156);
	const FLT x286 = 2 * x280;
	const FLT x287 =
		(x272 * x150) + (-1 * x268 * x155) + (x275 * x154) + (-1 * x279 * x159) + (x270 * x153) + (-1 * x283 * x152);
	const FLT x288 =
		(-1 * x279 * x196) + (x270 * x198) + (x275 * x192) + (x272 * x197) + (-1 * x268 * x194) + (-1 * x283 * x188);
	const FLT x289 = (x231 * x272) + (-1 * x219 * x279) + (-1 * x230 * x272) + (x278 * x229) + (x205 * x286) +
					 (x216 * x279) + (-1 * x201 * x286) + (-1 * x205 * x277) + (-1 * x284 * x229) + (x288 * x187) +
					 (-1 * x287 * x187) + (x201 * x277);
	const FLT x290 = (x230 * x268) + (x219 * x274) + (x277 * x156) + (-1 * x282 * x229) + (x203 * x286) +
					 (x281 * x229) + (x200 * x287) + (-1 * x231 * x268) + (-1 * x286 * x156) + (-1 * x273 * x243) +
					 (-1 * x200 * x288) + (-1 * x203 * x277);
	const FLT x291 = (-1 * ((-1 * x285 * x250) + (x233 * x290)) * x251) +
					 (-1 * x242 * ((x289 * x249) + (-1 * ((x285 * x246) + (x290 * x245)) * x247)));
	const FLT x292 = x41 + x256;
	const FLT x293 = x32 + x258;
	const FLT x294 = (x45 * x293) + (x40 * x30) + (x31 * x39) + (x36 * x292);
	const FLT x295 = x24 * x294;
	const FLT x296 = x21 * x294;
	const FLT x297 = x13 * x294;
	const FLT x298 = x53 * x294;
	const FLT x299 = (x60 * x292) + (x17 * x298) + (-1 * x57 * x293) + x50 + (-1 * x51 * x296) + (x297 * x162) +
					 (x27 * x295) + (-1 * x56);
	const FLT x300 = x17 * x294;
	const FLT x301 = (-1 * x55 * x293) + (-1 * x51 * x295) + x172 + (x49 * x292) + (-1 * x27 * x296) +
					 (-1 * x53 * x297) + (x300 * x162) + x171;
	const FLT x302 = (x24 * x298) + (-1 * x167) + (-1 * x51 * x297) + x168 + (x49 * x293) + (-1 * x296 * x162) +
					 (x55 * x292) + (-1 * x27 * x300);
	const FLT x303 = (x60 * x293) + (x57 * x292) + (-1 * x295 * x162) + (-1 * x53 * x296) + x163 + x160 + (x27 * x297) +
					 (-1 * x51 * x300);
	const FLT x304 = (x303 * x175) + (x302 * x174) + (x299 * x165) + (x301 * x170);
	const FLT x305 = x304 * x180;
	const FLT x306 = x304 * x183;
	const FLT x307 = x304 * x178;
	const FLT x308 =
		(-1 * x210 * x305) + (x302 * x194) + (-1 * x303 * x198) + (x306 * x189) + (x299 * x197) + (-1 * x307 * x196);
	const FLT x309 = x303 * x157;
	const FLT x310 = x304 * x190;
	const FLT x311 =
		(x301 * x198) + (x305 * x196) + (-1 * x310 * x189) + (-1 * x302 * x197) + (-1 * x210 * x307) + (x299 * x194);
	const FLT x312 =
		(x305 * x159) + (x301 * x153) + (-1 * x307 * x182) + (x299 * x155) + (-1 * x310 * x185) + (-1 * x302 * x150);
	const FLT x313 =
		(-1 * x307 * x159) + (-1 * x303 * x153) + (x302 * x155) + (-1 * x305 * x182) + (x299 * x150) + (x306 * x185);
	const FLT x314 = (x309 * x156) + (-1 * x200 * x312) + (x214 * x304) + (-1 * x308 * x187) + (x313 * x187) +
					 (x200 * x311) + (x203 * x306) + (x208 * x301) + (-1 * x207 * x301) + (-1 * x201 * x310) +
					 (-1 * x212 * x303) + (-1 * x306 * x156);
	const FLT x315 =
		(-1 * x307 * x185) + (-1 * x301 * x155) + (-1 * x306 * x159) + (x303 * x150) + (x310 * x182) + (x299 * x153);
	const FLT x316 =
		(-1 * x301 * x194) + (x210 * x310) + (-1 * x307 * x189) + (x303 * x197) + (-1 * x306 * x196) + (x299 * x198);
	const FLT x317 = (-1 * x229 * x311) + (-1 * x205 * x305) + (-1 * x216 * x309) + (x216 * x306) + (x201 * x305) +
					 (x207 * x302) + (-1 * x208 * x302) + (x229 * x312) + (-1 * x315 * x187) + (x219 * x309) +
					 (-1 * x219 * x306) + (x316 * x187);
	const FLT x318 = (-1 * x243 * x304) + (-1 * x231 * x301) + (-1 * x229 * x313) + (x230 * x301) + (-1 * x203 * x305) +
					 (-1 * x200 * x316) + (x200 * x315) + (x229 * x308) + (x305 * x156) + (x212 * x302) +
					 (-1 * x302 * x158) + (x219 * x310);
	const FLT x319 = (-1 * ((-1 * x250 * x314) + (x233 * x318)) * x251) +
					 (-1 * x242 * ((x249 * x317) + (-1 * ((x246 * x314) + (x245 * x318)) * x247)));
	const FLT x320 = 1. / x69;
	const FLT x321 = x63 * x320 * x170;
	const FLT x322 = -1 * x321;
	const FLT x323 = x320 * x165;
	const FLT x324 = x65 * x323;
	const FLT x325 = -1 * x324;
	const FLT x326 = x325 + x322;
	const FLT x327 = 2 * x320;
	const FLT x328 = x66 * x327;
	const FLT x329 = x64 * x327;
	const FLT x330 = -1 + x329 + x328;
	const FLT x331 = x63 * x323;
	const FLT x332 = x320 * x175;
	const FLT x333 = x67 * x332;
	const FLT x334 = -1 * x333;
	const FLT x335 = x334 + x331;
	const FLT x336 = (-1 * ((-1 * x250 * x326) + (x233 * x335)) * x251) +
					 (-1 * x242 * ((x249 * x330) + (-1 * ((x246 * x326) + (x245 * x335)) * x247)));
	const FLT x337 = x67 * x323;
	const FLT x338 = x63 * x332;
	const FLT x339 = -1 * x338;
	const FLT x340 = x339 + x337;
	const FLT x341 = -1 * x331;
	const FLT x342 = x341 + x334;
	const FLT x343 = x68 * x327;
	const FLT x344 = -1 + x343;
	const FLT x345 = x344 + x329;
	const FLT x346 = (-1 * ((-1 * x250 * x340) + (x233 * x345)) * x251) +
					 (-1 * x242 * ((x249 * x342) + (-1 * ((x246 * x340) + (x245 * x345)) * x247)));
	const FLT x347 = x344 + x328;
	const FLT x348 = x322 + x324;
	const FLT x349 = -1 * x337;
	const FLT x350 = x349 + x339;
	const FLT x351 = (-1 * ((-1 * x250 * x347) + (x233 * x350)) * x251) +
					 (-1 * x242 * ((x249 * x348) + (-1 * ((x246 * x347) + (x245 * x350)) * x247)));
	const FLT x352 = x320 * x147;
	const FLT x353 = x65 * x352;
	const FLT x354 = x62 * x353;
	const FLT x355 = x63 * x67 * x352;
	const FLT x356 = x355 + x354;
	const FLT x357 = -1 * x66 * x352;
	const FLT x358 = -1 * x64 * x352;
	const FLT x359 = x358 + x148 + x357;
	const FLT x360 = x67 * x353;
	const FLT x361 = x62 * x352;
	const FLT x362 = x63 * x361;
	const FLT x363 = (-1 * x362) + x360;
	const FLT x364 = (-1 * ((-1 * x250 * x356) + (x233 * x363)) * x251) +
					 (-1 * x242 * ((x249 * x359) + (-1 * ((x246 * x356) + (x245 * x363)) * x247)));
	const FLT x365 = x63 * x353;
	const FLT x366 = x67 * x361;
	const FLT x367 = (-1 * x366) + x365;
	const FLT x368 = x360 + x362;
	const FLT x369 = (-1 * x68 * x352) + x148;
	const FLT x370 = x369 + x358;
	const FLT x371 = (-1 * ((-1 * x250 * x367) + (x233 * x370)) * x251) +
					 (-1 * x242 * ((x249 * x368) + (-1 * ((x246 * x367) + (x245 * x370)) * x247)));
	const FLT x372 = x369 + x357;
	const FLT x373 = (-1 * x354) + x355;
	const FLT x374 = x365 + x366;
	const FLT x375 = (-1 * ((-1 * x250 * x372) + (x233 * x374)) * x251) +
					 (-1 * x242 * ((x249 * x373) + (-1 * ((x246 * x372) + (x245 * x374)) * x247)));
	const FLT x376 = 0.5 * x105;
	const FLT x377 = -0.5 * x107;
	const FLT x378 = x377 + (-1 * x376);
	const FLT x379 = -1 * x378 * x127;
	const FLT x380 = x108 * (*_x0).Object.Pose.Rot[0];
	const FLT x381 = 0.5 * x103;
	const FLT x382 = 0.5 * x98;
	const FLT x383 = x382 + x381;
	const FLT x384 = 2 * x383;
	const FLT x385 = 0.5 * x110;
	const FLT x386 = -1 * x385;
	const FLT x387 = 0.5 * x111;
	const FLT x388 = x387 + x386;
	const FLT x389 = 2 * x108;
	const FLT x390 = 2 * x112;
	const FLT x391 = 0.5 * x114;
	const FLT x392 = -1 * x391;
	const FLT x393 = 0.5 * x113;
	const FLT x394 = x393 + x392;
	const FLT x395 = 2 * x104;
	const FLT x396 = (x395 * x394) + (x378 * x390) + (x384 * x115) + (x389 * x388);
	const FLT x397 = 1.0 / 2.0 * (1. / (x116 * sqrt(x116)));
	const FLT x398 = x397 * x396;
	const FLT x399 = x398 * (*_x0).Object.Pose.Rot[1];
	const FLT x400 = x396 * (*_x0).Object.Pose.Rot[2];
	const FLT x401 = x397 * x104;
	const FLT x402 = x383 * x118;
	const FLT x403 = x397 * x115;
	const FLT x404 = x403 * x396;
	const FLT x405 = x388 * x117;
	const FLT x406 = (x405 * (*_x0).Object.Pose.Rot[0]) + (-1 * x404 * (*_x0).Object.Pose.Rot[3]) + (-1 * x398 * x380) +
					 x379 + (x399 * x112) + (-1 * x401 * x400) + x402 + (x394 * x119);
	const FLT x407 = x75 * x129;
	const FLT x408 = x383 * x117;
	const FLT x409 = -1 * x408 * (*_x0).Object.Pose.Rot[1];
	const FLT x410 = x398 * x112;
	const FLT x411 = x378 * x118;
	const FLT x412 = x400 * x397;
	const FLT x413 = x401 * x396;
	const FLT x414 = x394 * x117;
	const FLT x415 = (x414 * (*_x0).Object.Pose.Rot[0]) + (-1 * x413 * (*_x0).Object.Pose.Rot[0]) +
					 (x404 * (*_x0).Object.Pose.Rot[1]) + (x410 * (*_x0).Object.Pose.Rot[3]) + x409 + (-1 * x411) +
					 (-1 * x388 * x119) + (x412 * x108);
	const FLT x416 = x71 * x129;
	const FLT x417 = x403 * (*_x0).Object.Pose.Rot[0];
	const FLT x418 = x408 * (*_x0).Object.Pose.Rot[0];
	const FLT x419 = x378 * x119;
	const FLT x420 = x419 + (-1 * x412 * x112) + x418 + (-1 * x417 * x396) + (x414 * (*_x0).Object.Pose.Rot[1]) +
					 (-1 * x413 * (*_x0).Object.Pose.Rot[1]) + (x398 * x108 * (*_x0).Object.Pose.Rot[3]) +
					 (-1 * x388 * x118);
	const FLT x421 = x78 * x129;
	const FLT x422 = x383 * x119;
	const FLT x423 = x378 * x120;
	const FLT x424 = (-1 * x422) + (-1 * x413 * (*_x0).Object.Pose.Rot[3]) + (-1 * x399 * x108) + x423 +
					 (x405 * (*_x0).Object.Pose.Rot[1]) + (x394 * x118) + (-1 * x410 * (*_x0).Object.Pose.Rot[0]) +
					 (x400 * x403);
	const FLT x425 = (-1 * x421 * x420) + (x424 * x133) + (x406 * x407) + (x415 * x416);
	const FLT x426 = (-1 * x407 * x420) + (-1 * x406 * x421) + (x415 * x133) + (-1 * x416 * x424);
	const FLT x427 = (x416 * x420) + (-1 * x407 * x424) + (x406 * x133) + (x415 * x421);
	const FLT x428 = (x427 * sensor_pt[2]) + (-1 * x425 * sensor_pt[1]) + (x426 * sensor_pt[0]);
	const FLT x429 = 2 * x140;
	const FLT x430 = (-1 * x406 * x416) + (x424 * x421) + (x407 * x415) + (x420 * x133);
	const FLT x431 = (x430 * sensor_pt[1]) + (-1 * x427 * sensor_pt[0]) + (x426 * sensor_pt[2]);
	const FLT x432 = 2 * x142;
	const FLT x433 = 2 * x146;
	const FLT x434 = 2 * x151;
	const FLT x435 = (-1 * x433 * x430) + (x429 * x428) + (x425 * x434) + (-1 * x432 * x431);
	const FLT x436 = x70 * x435;
	const FLT x437 = (x425 * sensor_pt[0]) + (-1 * x430 * sensor_pt[2]) + (x426 * sensor_pt[1]);
	const FLT x438 = 2 * x143;
	const FLT x439 = 2 * x145;
	const FLT x440 = (x427 * x433) + (x431 * x439) + (-1 * x429 * x437) + (-1 * x425 * x438);
	const FLT x441 = x70 * x440;
	const FLT x442 = (x432 * x437) + (x430 * x438) + (-1 * x427 * x434) + (-1 * x428 * x439);
	const FLT x443 = (-1 * x442 * x217) + (x63 * x436) + (x62 * x441);
	const FLT x444 = x70 * x442;
	const FLT x445 = (-1 * x63 * x441) + (x62 * x436) + (x67 * x444);
	const FLT x446 = x442 + (x443 * x187) + (-1 * x445 * x200);
	const FLT x447 = (x62 * x444) + (x440 * x217) + (-1 * x67 * x436);
	const FLT x448 = x440 + (x445 * x229) + (-1 * x447 * x187);
	const FLT x449 = (x447 * x200) + x435 + (-1 * x443 * x229);
	const FLT x450 = (-1 * ((-1 * x446 * x250) + (x449 * x233)) * x251) +
					 (-1 * x242 * ((x448 * x249) + (-1 * ((x446 * x246) + (x449 * x245)) * x247)));
	const FLT x451 = -1 * x387;
	const FLT x452 = x451 + x386;
	const FLT x453 = 2 * x115;
	const FLT x454 = -1 * x393;
	const FLT x455 = x392 + x454;
	const FLT x456 = (-1 * x382) + x381;
	const FLT x457 = x376 + x377;
	const FLT x458 = (x457 * x395) + (x456 * x389) + (x452 * x453) + (x455 * x390);
	const FLT x459 = x458 * x397;
	const FLT x460 = x459 * x112;
	const FLT x461 = x459 * x104;
	const FLT x462 = x459 * x108;
	const FLT x463 = x456 * x117;
	const FLT x464 = x458 * x403;
	const FLT x465 = (x464 * (*_x0).Object.Pose.Rot[2]) + (x463 * (*_x0).Object.Pose.Rot[1]) + (-1 * x452 * x119) +
					 (-1 * x460 * (*_x0).Object.Pose.Rot[0]) + (-1 * x462 * (*_x0).Object.Pose.Rot[1]) + (x455 * x120) +
					 (-1 * x461 * (*_x0).Object.Pose.Rot[3]) + (x457 * x118);
	const FLT x466 = x459 * (*_x0).Object.Pose.Rot[2];
	const FLT x467 = x457 * x117;
	const FLT x468 = (-1 * x456 * x119) + (x464 * (*_x0).Object.Pose.Rot[1]) + (-1 * x452 * x127) +
					 (x467 * (*_x0).Object.Pose.Rot[0]) + (x466 * x108) + (-1 * x461 * (*_x0).Object.Pose.Rot[0]) +
					 (x460 * (*_x0).Object.Pose.Rot[3]) + (-1 * x455 * x118);
	const FLT x469 = (-1 * x466 * x112) + (x455 * x119) + (x467 * (*_x0).Object.Pose.Rot[1]) + (-1 * x456 * x118) +
					 (-1 * x461 * (*_x0).Object.Pose.Rot[1]) + (-1 * x458 * x417) + (x452 * x120) +
					 (x462 * (*_x0).Object.Pose.Rot[3]);
	const FLT x470 = (-1 * x464 * (*_x0).Object.Pose.Rot[3]) + (x463 * (*_x0).Object.Pose.Rot[0]) + (x452 * x118) +
					 (-1 * x459 * x380) + (-1 * x455 * x127) + (x457 * x119) + (-1 * x466 * x104) +
					 (x460 * (*_x0).Object.Pose.Rot[1]);
	const FLT x471 = (x469 * x416) + (x470 * x133) + (-1 * x465 * x407) + (x468 * x421);
	const FLT x472 = (-1 * x470 * x421) + (-1 * x469 * x407) + (-1 * x465 * x416) + (x468 * x133);
	const FLT x473 = (-1 * x470 * x416) + (x468 * x407) + (x465 * x421) + (x469 * x133);
	const FLT x474 = (-1 * x471 * sensor_pt[0]) + (x473 * sensor_pt[1]) + (x472 * sensor_pt[2]);
	const FLT x475 = (x468 * x416) + (x465 * x133) + (x470 * x407) + (-1 * x469 * x421);
	const FLT x476 = (x471 * sensor_pt[2]) + (x472 * sensor_pt[0]) + (-1 * x475 * sensor_pt[1]);
	const FLT x477 = (x475 * x434) + (-1 * x473 * x433) + (-1 * x474 * x432) + (x476 * x429);
	const FLT x478 = (x475 * sensor_pt[0]) + (-1 * x473 * sensor_pt[2]) + (x472 * sensor_pt[1]);
	const FLT x479 = (x474 * x439) + (-1 * x475 * x438) + (-1 * x478 * x429) + (x471 * x433);
	const FLT x480 = x70 * x62;
	const FLT x481 = (x473 * x438) + (-1 * x476 * x439) + (x478 * x432) + (-1 * x471 * x434);
	const FLT x482 = (x477 * x218) + (-1 * x481 * x217) + (x479 * x480);
	const FLT x483 = (x481 * x221) + (-1 * x479 * x218) + (x477 * x480);
	const FLT x484 = (x482 * x187) + x481 + (-1 * x483 * x200);
	const FLT x485 = (x481 * x480) + (x479 * x217) + (-1 * x477 * x221);
	const FLT x486 = x479 + (-1 * x485 * x187) + (x483 * x229);
	const FLT x487 = (x485 * x200) + x477 + (-1 * x482 * x229);
	const FLT x488 = (-1 * ((-1 * x484 * x250) + (x487 * x233)) * x251) +
					 (-1 * x242 * ((x486 * x249) + (-1 * ((x484 * x246) + (x487 * x245)) * x247)));
	const FLT x489 = x391 + x454;
	const FLT x490 = x385 + x451;
	const FLT x491 = (x489 * x389) + (x490 * x395) + (x384 * x112) + (x453 * x378);
	const FLT x492 = x491 * x397;
	const FLT x493 = x492 * x104;
	const FLT x494 = x492 * x112;
	const FLT x495 = x490 * x117;
	const FLT x496 = x491 * x403;
	const FLT x497 = x492 * x108;
	const FLT x498 = (x497 * (*_x0).Object.Pose.Rot[2]) + x379 + (-1 * x493 * (*_x0).Object.Pose.Rot[0]) +
					 (-1 * x489 * x119) + (x494 * (*_x0).Object.Pose.Rot[3]) + (x495 * (*_x0).Object.Pose.Rot[0]) +
					 (x496 * (*_x0).Object.Pose.Rot[1]) + (-1 * x402);
	const FLT x499 = (x497 * (*_x0).Object.Pose.Rot[3]) + (x495 * (*_x0).Object.Pose.Rot[1]) + x422 +
					 (-1 * x491 * x417) + (-1 * x489 * x118) + (-1 * x494 * (*_x0).Object.Pose.Rot[2]) + x423 +
					 (-1 * x493 * (*_x0).Object.Pose.Rot[1]);
	const FLT x500 = x489 * x117;
	const FLT x501 = (x500 * (*_x0).Object.Pose.Rot[0]) + (x494 * (*_x0).Object.Pose.Rot[1]) +
					 (-1 * x496 * (*_x0).Object.Pose.Rot[3]) + (-1 * x493 * (*_x0).Object.Pose.Rot[2]) + x409 +
					 (x490 * x119) + (-1 * x492 * x380) + x411;
	const FLT x502 = (x496 * (*_x0).Object.Pose.Rot[2]) + x418 + (x500 * (*_x0).Object.Pose.Rot[1]) +
					 (-1 * x493 * (*_x0).Object.Pose.Rot[3]) + (-1 * x497 * (*_x0).Object.Pose.Rot[1]) + (x490 * x118) +
					 (-1 * x419) + (-1 * x494 * (*_x0).Object.Pose.Rot[0]);
	const FLT x503 = (x421 * x502) + (-1 * x416 * x501) + (x498 * x407) + (x499 * x133);
	const FLT x504 = (-1 * x421 * x501) + (-1 * x499 * x407) + (-1 * x416 * x502) + (x498 * x133);
	const FLT x505 = (x502 * x133) + (-1 * x499 * x421) + (x407 * x501) + (x498 * x416);
	const FLT x506 = (-1 * x503 * sensor_pt[2]) + (x505 * sensor_pt[0]) + (x504 * sensor_pt[1]);
	const FLT x507 = (x501 * x133) + (x498 * x421) + (-1 * x407 * x502) + (x499 * x416);
	const FLT x508 = (x503 * sensor_pt[1]) + (x504 * sensor_pt[2]) + (-1 * x507 * sensor_pt[0]);
	const FLT x509 = (x439 * x508) + (-1 * x429 * x506) + (x433 * x507) + (-1 * x438 * x505);
	const FLT x510 = x70 * x509;
	const FLT x511 = (x507 * sensor_pt[2]) + (-1 * x505 * sensor_pt[1]) + (x504 * sensor_pt[0]);
	const FLT x512 = (-1 * x433 * x503) + (x429 * x511) + (-1 * x432 * x508) + (x434 * x505);
	const FLT x513 = x70 * x512;
	const FLT x514 = (x438 * x503) + (x432 * x506) + (-1 * x434 * x507) + (-1 * x439 * x511);
	const FLT x515 = x70 * x514;
	const FLT x516 = (-1 * x65 * x515) + (x62 * x510) + (x63 * x513);
	const FLT x517 = (-1 * x63 * x510) + (x67 * x515) + (x62 * x513);
	const FLT x518 = x514 + (x516 * x187) + (-1 * x517 * x200);
	const FLT x519 = (x62 * x515) + (x509 * x217) + (-1 * x67 * x513);
	const FLT x520 = x509 + (x517 * x229) + (-1 * x519 * x187);
	const FLT x521 = x512 + (x519 * x200) + (-1 * x516 * x229);
	const FLT x522 = (-1 * ((-1 * x518 * x250) + (x521 * x233)) * x251) +
					 (-1 * x242 * ((x520 * x249) + (-1 * ((x518 * x246) + (x521 * x245)) * x247)));
	const FLT x523 = x321 + x324;
	const FLT x524 = -1 * x329;
	const FLT x525 = 1 + (-1 * x328);
	const FLT x526 = x525 + x524;
	const FLT x527 = x341 + x333;
	const FLT x528 = (-1 * ((-1 * x523 * x250) + (x527 * x233)) * x251) +
					 (-1 * x242 * ((x526 * x249) + (-1 * ((x523 * x246) + (x527 * x245)) * x247)));
	const FLT x529 = x349 + x338;
	const FLT x530 = x333 + x331;
	const FLT x531 = -1 * x343;
	const FLT x532 = 1 + x524 + x531;
	const FLT x533 = (-1 * ((-1 * x529 * x250) + (x532 * x233)) * x251) +
					 (-1 * x242 * ((x530 * x249) + (-1 * ((x529 * x246) + (x532 * x245)) * x247)));
	const FLT x534 = x525 + x531;
	const FLT x535 = x325 + x321;
	const FLT x536 = x338 + x337;
	const FLT x537 = (-1 * ((-1 * x534 * x250) + (x536 * x233)) * x251) +
					 (-1 * x242 * ((x535 * x249) + (-1 * ((x534 * x246) + (x536 * x245)) * x247)));
	const FLT x538 = -1 * x126;
	const FLT x539 = x90 * x135;
	const FLT x540 = dt * dt * dt;
	const FLT x541 = (1. / (x81 * sqrt(x81))) * x84;
	const FLT x542 = x540 * x541;
	const FLT x543 = x75 * x542;
	const FLT x544 = x539 * x543;
	const FLT x545 = x71 * x544;
	const FLT x546 = x75 * x75 * x75;
	const FLT x547 = dt * dt * dt * dt;
	const FLT x548 = (1. / (x81 * x81)) * x85;
	const FLT x549 = 2 * x548;
	const FLT x550 = x547 * x549;
	const FLT x551 = 1.0 * x88;
	const FLT x552 = x541 * x551;
	const FLT x553 = x547 * x552;
	const FLT x554 = x73 * x553;
	const FLT x555 = x75 * x550;
	const FLT x556 = x87 * x72;
	const FLT x557 = 2 * x556;
	const FLT x558 = x79 * x553;
	const FLT x559 = x72 * x124;
	const FLT x560 = x559 * x551;
	const FLT x561 = (x75 * x554) + (-1 * x546 * x550) + (x546 * x553) + (-1 * x73 * x555) + (-1 * x75 * x560) +
					 (x75 * x558) + (-1 * x79 * x555) + (x75 * x557);
	const FLT x562 = 1.0 / 2.0 * (1. / (x89 * sqrt(x89)));
	const FLT x563 = x88 * x562;
	const FLT x564 = x563 * x128;
	const FLT x565 = x90 * x128;
	const FLT x566 = 0.5 * x559;
	const FLT x567 = x565 * x566;
	const FLT x568 = x562 * x125;
	const FLT x569 = x75 * x568;
	const FLT x570 = x561 * x122;
	const FLT x571 = 0.5 * x86 * x540;
	const FLT x572 = x71 * x75;
	const FLT x573 = x572 * x571;
	const FLT x574 = x573 * x141;
	const FLT x575 = x71 * x568;
	const FLT x576 = x575 * x135;
	const FLT x577 = x76 * x542;
	const FLT x578 = x78 * x568;
	const FLT x579 = x578 * x561;
	const FLT x580 = x76 * x571;
	const FLT x581 = x90 * x132;
	const FLT x582 = x78 * x543;
	const FLT x583 = x571 * x134;
	const FLT x584 = x78 * x583;
	const FLT x585 = (x75 * x584) + (-1 * x581 * x582);
	const FLT x586 = (-1 * x579 * x132) + (x577 * x123) + (-1 * x576 * x561) + x585 + x574 + x538 + (-1 * x564 * x561) +
					 (-1 * x580 * x139) + (-1 * x545) + (x570 * x569) + (-1 * x75 * x567);
	const FLT x587 = x78 * x571;
	const FLT x588 = x75 * x587;
	const FLT x589 = x588 * x139;
	const FLT x590 = x71 * x565;
	const FLT x591 = x590 * x543;
	const FLT x592 = x582 * x123;
	const FLT x593 = x573 * x144;
	const FLT x594 = x563 * x561;
	const FLT x595 = x539 * x566;
	const FLT x596 = x569 * x132;
	const FLT x597 = (-1 * x596 * x561) + (-1 * x592) + (x575 * x561 * x128) + (-1 * x577 * x581) + (-1 * x594 * x135) +
					 x591 + (-1 * x579 * x122) + x138 + (x76 * x583) + x589 + (-1 * x593) + (-1 * x75 * x595);
	const FLT x598 = -1 * x136;
	const FLT x599 = x582 * x565;
	const FLT x600 = x588 * x144;
	const FLT x601 = x581 * x566;
	const FLT x602 = x569 * x135;
	const FLT x603 = (x71 * x543 * x123) + (-1 * x573 * x139);
	const FLT x604 = (x539 * x577) + x603 + (x602 * x561) + (-1 * x75 * x601) + (-1 * x600) + (-1 * x580 * x141) +
					 x598 + (x570 * x575) + (-1 * x594 * x132) + x599 + (x579 * x128);
	const FLT x605 = x566 * x123;
	const FLT x606 = x575 * x132;
	const FLT x607 = x569 * x128;
	const FLT x608 = (x78 * x544) + (-1 * x588 * x141);
	const FLT x609 = x71 * x581;
	const FLT x610 = (-1 * x609 * x543) + (x572 * x583);
	const FLT x611 = (-1 * x577 * x565) + x610 + (x579 * x135) + (-1 * x594 * x122) + (-1 * x606 * x561) +
					 (-1 * x607 * x561) + x130 + x608 + (x580 * x144) + (-1 * x75 * x605);
	const FLT x612 = (x611 * sensor_pt[0]) + (-1 * x597 * sensor_pt[2]) + (x604 * sensor_pt[1]);
	const FLT x613 = (x586 * sensor_pt[2]) + (-1 * x611 * sensor_pt[1]) + (x604 * sensor_pt[0]);
	const FLT x614 = (-1 * x439 * x613) + (x432 * x612) + (-1 * x434 * x586) + (x438 * x597);
	const FLT x615 = x70 * x614;
	const FLT x616 = (x597 * sensor_pt[1]) + (-1 * x586 * sensor_pt[0]) + (x604 * sensor_pt[2]);
	const FLT x617 = (-1 * x433 * x597) + (x434 * x611) + (-1 * x432 * x616) + (x429 * x613);
	const FLT x618 = x70 * x617;
	const FLT x619 = (x433 * x586) + (-1 * x429 * x612) + (x439 * x616) + (-1 * x438 * x611);
	const FLT x620 = (-1 * x619 * x218) + (x67 * x615) + (x62 * x618);
	const FLT x621 = (-1 * x614 * x217) + (x63 * x618) + (x480 * x619);
	const FLT x622 = x614 + (-1 * x620 * x200) + (x621 * x187);
	const FLT x623 = (x62 * x615) + (x619 * x217) + (-1 * x67 * x618);
	const FLT x624 = x619 + (x620 * x229) + (-1 * x623 * x187);
	const FLT x625 = x617 + (-1 * x621 * x229) + (x623 * x200);
	const FLT x626 = (-1 * ((-1 * x622 * x250) + (x625 * x233)) * x251) +
					 (-1 * x242 * ((x624 * x249) + (-1 * ((x622 * x246) + (x625 * x245)) * x247)));
	const FLT x627 = x78 * x550;
	const FLT x628 = x76 * x553;
	const FLT x629 = x78 * x78 * x78;
	const FLT x630 = (x78 * x628) + (x629 * x553) + (x78 * x554) + (-1 * x629 * x550) + (-1 * x76 * x627) +
					 (x78 * x557) + (-1 * x78 * x560) + (-1 * x73 * x627);
	const FLT x631 = x79 * x542;
	const FLT x632 = x71 * x587;
	const FLT x633 = x632 * x141;
	const FLT x634 = x630 * x578;
	const FLT x635 = x78 * x542;
	const FLT x636 = x71 * x635;
	const FLT x637 = x636 * x539;
	const FLT x638 = x630 * x575;
	const FLT x639 = x630 * x122;
	const FLT x640 = (x639 * x569) + (-1 * x630 * x564) + x592 + x138 + (-1 * x637) + (-1 * x78 * x567) +
					 (-1 * x631 * x581) + (-1 * x589) + x633 + (-1 * x634 * x132) + (-1 * x638 * x135) + (x79 * x583);
	const FLT x641 = x632 * x139;
	const FLT x642 = x79 * x571;
	const FLT x643 = x636 * x123;
	const FLT x644 = -1 * x130;
	const FLT x645 = x630 * x563;
	const FLT x646 = x608 + (x602 * x630) + x644 + (x639 * x575) + (-1 * x642 * x144) + (-1 * x641) + (x634 * x128) +
					 (x631 * x565) + (-1 * x645 * x132) + (-1 * x78 * x601) + x643;
	const FLT x647 = (x636 * x565) + (-1 * x632 * x144);
	const FLT x648 = x647 + x585 + (-1 * x78 * x595) + (-1 * x639 * x578) + (x638 * x128) + (-1 * x631 * x123) +
					 (x642 * x139) + (-1 * x630 * x596) + x126 + (-1 * x645 * x135);
	const FLT x649 = (x648 * sensor_pt[1]) + (-1 * x640 * sensor_pt[0]) + (x646 * sensor_pt[2]);
	const FLT x650 = (x71 * x584) + (-1 * x609 * x635);
	const FLT x651 = x650 + (-1 * x642 * x141) + (-1 * x607 * x630) + (-1 * x599) + (-1 * x645 * x122) +
					 (-1 * x78 * x605) + (-1 * x606 * x630) + (x631 * x539) + (x634 * x135) + x598 + x600;
	const FLT x652 = (x640 * sensor_pt[2]) + (-1 * x651 * sensor_pt[1]) + (x646 * sensor_pt[0]);
	const FLT x653 = (x434 * x651) + (x429 * x652) + (-1 * x432 * x649) + (-1 * x433 * x648);
	const FLT x654 = x70 * x653;
	const FLT x655 = (-1 * x648 * sensor_pt[2]) + (x651 * sensor_pt[0]) + (x646 * sensor_pt[1]);
	const FLT x656 = (x433 * x640) + (-1 * x429 * x655) + (x439 * x649) + (-1 * x438 * x651);
	const FLT x657 = x70 * x656;
	const FLT x658 = (x438 * x648) + (x432 * x655) + (-1 * x434 * x640) + (-1 * x439 * x652);
	const FLT x659 = (-1 * x658 * x217) + (x63 * x654) + (x62 * x657);
	const FLT x660 = (-1 * x63 * x657) + (x658 * x221) + (x62 * x654);
	const FLT x661 = x658 + (x659 * x187) + (-1 * x660 * x200);
	const FLT x662 = (x480 * x658) + (-1 * x67 * x654) + (x656 * x217);
	const FLT x663 = x656 + (x660 * x229) + (-1 * x662 * x187);
	const FLT x664 = x653 + (x662 * x200) + (-1 * x659 * x229);
	const FLT x665 = (-1 * ((-1 * x661 * x250) + (x664 * x233)) * x251) +
					 (-1 * x242 * ((x663 * x249) + (-1 * ((x661 * x246) + (x664 * x245)) * x247)));
	const FLT x666 = 2 * x71;
	const FLT x667 = x666 * x547 * x548;
	const FLT x668 = (x71 * x71 * x71) * x547;
	const FLT x669 = (x71 * x558) + (-1 * x76 * x667) + (x71 * x628) + (-1 * x668 * x549) + (x666 * x556) +
					 (x668 * x552) + (-1 * x71 * x560) + (-1 * x79 * x667);
	const FLT x670 = x669 * x575;
	const FLT x671 = x73 * x542;
	const FLT x672 = x669 * x578;
	const FLT x673 = x73 * x571;
	const FLT x674 = x669 * x563;
	const FLT x675 = (x602 * x669) + (-1 * x673 * x139) + (x672 * x128) + x647 + (-1 * x574) + x538 + x545 +
					 (-1 * x609 * x566) + (-1 * x674 * x132) + (x670 * x122) + (x671 * x123);
	const FLT x676 = x603 + (-1 * x672 * x132) + (-1 * x669 * x576) + (x669 * x569 * x122) + (-1 * x590 * x566) +
					 (-1 * x671 * x539) + x650 + (x673 * x141) + x136 + (-1 * x669 * x564);
	const FLT x677 = x610 + (-1 * x673 * x144) + x641 + (x670 * x128) + (-1 * x674 * x135) + (-1 * x669 * x596) +
					 (-1 * x71 * x595) + x644 + (x671 * x565) + (-1 * x672 * x122) + (-1 * x643);
	const FLT x678 = (x677 * sensor_pt[1]) + (x675 * sensor_pt[2]) + (-1 * x676 * sensor_pt[0]);
	const FLT x679 = x637 + (-1 * x606 * x669) + (-1 * x71 * x605) + (x672 * x135) + (-1 * x674 * x122) + x593 + x138 +
					 (-1 * x591) + (-1 * x607 * x669) + (-1 * x671 * x581) + (x73 * x583) + (-1 * x633);
	const FLT x680 = (x679 * sensor_pt[0]) + (-1 * x677 * sensor_pt[2]) + (x675 * sensor_pt[1]);
	const FLT x681 = (x433 * x676) + (-1 * x438 * x679) + (x439 * x678) + (-1 * x429 * x680);
	const FLT x682 = (x676 * sensor_pt[2]) + (-1 * x679 * sensor_pt[1]) + (x675 * sensor_pt[0]);
	const FLT x683 = (x434 * x679) + (x429 * x682) + (-1 * x432 * x678) + (-1 * x433 * x677);
	const FLT x684 = (x432 * x680) + (x438 * x677) + (-1 * x434 * x676) + (-1 * x439 * x682);
	const FLT x685 = x70 * x684;
	const FLT x686 = (x480 * x681) + (-1 * x65 * x685) + (x683 * x218);
	const FLT x687 = (-1 * x681 * x218) + (x480 * x683) + (x67 * x685);
	const FLT x688 = x684 + (x686 * x187) + (-1 * x687 * x200);
	const FLT x689 = (x62 * x685) + (x681 * x217) + (-1 * x683 * x221);
	const FLT x690 = x681 + (x687 * x229) + (-1 * x689 * x187);
	const FLT x691 = (x689 * x200) + x683 + (-1 * x686 * x229);
	const FLT x692 = (-1 * ((-1 * x688 * x250) + (x691 * x233)) * x251) +
					 (-1 * x242 * ((x690 * x249) + (-1 * ((x688 * x246) + (x691 * x245)) * x247)));
	const FLT x693 = dt * x324;
	const FLT x694 = dt * x321;
	const FLT x695 = x694 + x693;
	const FLT x696 = -1 * dt * x328;
	const FLT x697 = -1 * dt * x329;
	const FLT x698 = x697 + dt + x696;
	const FLT x699 = dt * x333;
	const FLT x700 = dt * x331;
	const FLT x701 = (-1 * x700) + x699;
	const FLT x702 = (-1 * ((-1 * x695 * x250) + (x701 * x233)) * x251) +
					 (-1 * x242 * ((x698 * x249) + (-1 * ((x695 * x246) + (x701 * x245)) * x247)));
	const FLT x703 = dt * x338;
	const FLT x704 = dt * x337;
	const FLT x705 = (-1 * x704) + x703;
	const FLT x706 = x699 + x700;
	const FLT x707 = (-1 * dt * x343) + dt;
	const FLT x708 = x707 + x697;
	const FLT x709 = (-1 * ((-1 * x705 * x250) + (x708 * x233)) * x251) +
					 (-1 * x242 * ((x706 * x249) + (-1 * ((x705 * x246) + (x708 * x245)) * x247)));
	const FLT x710 = x707 + x696;
	const FLT x711 = (-1 * x693) + x694;
	const FLT x712 = x703 + x704;
	const FLT x713 = (-1 * ((-1 * x710 * x250) + (x712 * x233)) * x251) +
					 (-1 * x242 * ((x711 * x249) + (-1 * ((x710 * x246) + (x712 * x245)) * x247)));
	const FLT x714 = x220 * x242 * x248;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						x252 + (((-1 * x233 * x232) + (x215 * x225)) * x237) + (x252 * x254));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						x291 + (((-1 * x233 * x289) + (x285 * x225)) * x237) + (x291 * x254));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						x319 + (((-1 * x233 * x317) + (x225 * x314)) * x237) + (x254 * x319));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[0]) / sizeof(FLT),
						x336 + (((-1 * x233 * x330) + (x225 * x326)) * x237) + (x254 * x336));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[1]) / sizeof(FLT),
						x346 + (((-1 * x233 * x342) + (x225 * x340)) * x237) + (x254 * x346));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[2]) / sizeof(FLT),
						x351 + (((-1 * x233 * x348) + (x225 * x347)) * x237) + (x254 * x351));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Acc[0]) / sizeof(FLT),
						x364 + (((-1 * x233 * x359) + (x225 * x356)) * x237) + (x254 * x364));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Acc[1]) / sizeof(FLT),
						x371 + (((-1 * x233 * x368) + (x225 * x367)) * x237) + (x254 * x371));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Acc[2]) / sizeof(FLT),
						x375 + (((-1 * x233 * x373) + (x225 * x372)) * x237) + (x254 * x375));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[0]) / sizeof(FLT),
						x450 + (((-1 * x448 * x233) + (x446 * x225)) * x237) + (x450 * x254));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[1]) / sizeof(FLT),
						x488 + (((-1 * x486 * x233) + (x484 * x225)) * x237) + (x488 * x254));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[2]) / sizeof(FLT),
						x522 + (((-1 * x520 * x233) + (x518 * x225)) * x237) + (x522 * x254));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[0]) / sizeof(FLT),
						x528 + (((-1 * x526 * x233) + (x523 * x225)) * x237) + (x528 * x254));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[1]) / sizeof(FLT),
						x533 + (((-1 * x530 * x233) + (x529 * x225)) * x237) + (x533 * x254));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[2]) / sizeof(FLT),
						x537 + (((-1 * x535 * x233) + (x534 * x225)) * x237) + (x537 * x254));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x626 + (((-1 * x624 * x233) + (x622 * x225)) * x237) + (x626 * x254));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x665 + (((-1 * x663 * x233) + (x661 * x225)) * x237) + (x665 * x254));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x692 + (((-1 * x690 * x233) + (x688 * x225)) * x237) + (x692 * x254));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[0]) / sizeof(FLT),
						x702 + (((-1 * x698 * x233) + (x695 * x225)) * x237) + (x702 * x254));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[1]) / sizeof(FLT),
						x709 + (((-1 * x706 * x233) + (x705 * x225)) * x237) + (x709 * x254));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[2]) / sizeof(FLT),
						x713 + (((-1 * x711 * x233) + (x710 * x225)) * x237) + (x713 * x254));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD1.curve) / sizeof(FLT), x235 * x235);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD1.gibmag) / sizeof(FLT), -1 * cos(x253));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD1.gibpha) / sizeof(FLT), x254);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD1.phase) / sizeof(FLT), -1 + (-1 * x254));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD1.tilt) / sizeof(FLT),
						(-1 * x714 * x254) + (-1 * x714));
}

// Full version Jacobian of SurviveJointKalmanErrorModel_LightMeas_y_gen1 wrt
// [(*error_model).Lighthouse.AxisAngleRot[0], (*error_model).Lighthouse.AxisAngleRot[1],
// (*error_model).Lighthouse.AxisAngleRot[2], (*error_model).Lighthouse.Pos[0], (*error_model).Lighthouse.Pos[1],
// (*error_model).Lighthouse.Pos[2], (*error_model).Object.Acc[0], (*error_model).Object.Acc[1],
// (*error_model).Object.Acc[2], (*error_model).Object.IMUBias.AccBias[0], (*error_model).Object.IMUBias.AccBias[1],
// (*error_model).Object.IMUBias.AccBias[2], (*error_model).Object.IMUBias.AccScale[0],
// (*error_model).Object.IMUBias.AccScale[1], (*error_model).Object.IMUBias.AccScale[2],
// (*error_model).Object.IMUBias.GyroBias[0], (*error_model).Object.IMUBias.GyroBias[1],
// (*error_model).Object.IMUBias.GyroBias[2], (*error_model).Object.IMUBias.IMUCorrection[0],
// (*error_model).Object.IMUBias.IMUCorrection[1], (*error_model).Object.IMUBias.IMUCorrection[2],
// (*error_model).Object.Pose.AxisAngleRot[0], (*error_model).Object.Pose.AxisAngleRot[1],
// (*error_model).Object.Pose.AxisAngleRot[2], (*error_model).Object.Pose.Pos[0], (*error_model).Object.Pose.Pos[1],
// (*error_model).Object.Pose.Pos[2], (*error_model).Object.Velocity.AxisAngleRot[0],
// (*error_model).Object.Velocity.AxisAngleRot[1], (*error_model).Object.Velocity.AxisAngleRot[2],
// (*error_model).Object.Velocity.Pos[0], (*error_model).Object.Velocity.Pos[1], (*error_model).Object.Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f4d1c33a070>, <cnkalman.codegen.WrapMember object at 0x7f4d1c33a340>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c33a130>, <cnkalman.codegen.WrapMember object at 0x7f4d1c33a400>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c33a0d0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c33a3a0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c33a1f0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c33a4c0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c33a190>, <cnkalman.codegen.WrapMember object at 0x7f4d1c33a460>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c339e80>, <cnkalman.codegen.WrapMember object at 0x7f4d1c33a280>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c339fd0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c33a2e0>]

static inline void SurviveJointKalmanErrorModel_LightMeas_y_gen1_jac_error_model_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveJointKalmanModel *_x0,
	const SurviveJointKalmanErrorModel *error_model, const FLT *sensor_pt) {
	if (hx != 0) {
		hx->data[0] = SurviveJointKalmanErrorModel_LightMeas_y_gen1(dt, _x0, error_model, sensor_pt);
	}
	if (Hx != 0) {
		SurviveJointKalmanErrorModel_LightMeas_y_gen1_jac_error_model(Hx, dt, _x0, error_model, sensor_pt);
	}
}
// Jacobian of SurviveJointKalmanErrorModel_LightMeas_y_gen1 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]
static inline void
SurviveJointKalmanErrorModel_LightMeas_y_gen1_jac_sensor_pt(CnMat *Hx, const FLT dt, const SurviveJointKalmanModel *_x0,
															const SurviveJointKalmanErrorModel *error_model,
															const FLT *sensor_pt) {
	const FLT x0 = (*_x0).Object.Velocity.AxisAngleRot[2] + (*error_model).Object.Velocity.AxisAngleRot[2];
	const FLT x1 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[1];
	const FLT x2 = cos(x1);
	const FLT x3 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[0];
	const FLT x4 = sin(x3);
	const FLT x5 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[2];
	const FLT x6 = sin(x5);
	const FLT x7 = x4 * x6;
	const FLT x8 = cos(x5);
	const FLT x9 = sin(x1);
	const FLT x10 = cos(x3);
	const FLT x11 = x9 * x10;
	const FLT x12 = (x8 * x11) + (x2 * x7);
	const FLT x13 = x2 * x10;
	const FLT x14 = (x8 * x13) + (x7 * x9);
	const FLT x15 = x4 * x8;
	const FLT x16 = (x6 * x13) + (-1 * x9 * x15);
	const FLT x17 = (x2 * x15) + (-1 * x6 * x11);
	const FLT x18 = 1. / sqrt((x16 * x16) + (x17 * x17) + (x12 * x12) + (x14 * x14));
	const FLT x19 = x14 * x18;
	const FLT x20 = x18 * x17;
	const FLT x21 = x18 * x16;
	const FLT x22 = x12 * x18;
	const FLT x23 = (x22 * (*_x0).Object.Pose.Rot[1]) + (x21 * (*_x0).Object.Pose.Rot[0]) +
					(x19 * (*_x0).Object.Pose.Rot[3]) + (-1 * x20 * (*_x0).Object.Pose.Rot[2]);
	const FLT x24 = dt * dt;
	const FLT x25 = (x0 * x0) * x24;
	const FLT x26 = (*_x0).Object.Velocity.AxisAngleRot[0] + (*error_model).Object.Velocity.AxisAngleRot[0];
	const FLT x27 = x24 * (x26 * x26);
	const FLT x28 = (*_x0).Object.Velocity.AxisAngleRot[1] + (*error_model).Object.Velocity.AxisAngleRot[1];
	const FLT x29 = x24 * (x28 * x28);
	const FLT x30 = 1e-10 + x29 + x25 + x27;
	const FLT x31 = sqrt(x30);
	const FLT x32 = 0.5 * x31;
	const FLT x33 = sin(x32);
	const FLT x34 = (1. / x30) * (x33 * x33);
	const FLT x35 = cos(x32);
	const FLT x36 = 1. / sqrt((x34 * x27) + (x35 * x35) + (x34 * x25) + (x34 * x29));
	const FLT x37 = dt * (1. / x31) * x33 * x36;
	const FLT x38 = x37 * x23;
	const FLT x39 = (-1 * x21 * (*_x0).Object.Pose.Rot[1]) + (x19 * (*_x0).Object.Pose.Rot[2]) +
					(x22 * (*_x0).Object.Pose.Rot[0]) + (x20 * (*_x0).Object.Pose.Rot[3]);
	const FLT x40 = x37 * x39;
	const FLT x41 = (-1 * x20 * (*_x0).Object.Pose.Rot[1]) + (x19 * (*_x0).Object.Pose.Rot[0]) +
					(-1 * x21 * (*_x0).Object.Pose.Rot[3]) + (-1 * x22 * (*_x0).Object.Pose.Rot[2]);
	const FLT x42 = x36 * x35;
	const FLT x43 = (x19 * (*_x0).Object.Pose.Rot[1]) + (x20 * (*_x0).Object.Pose.Rot[0]) +
					(-1 * x22 * (*_x0).Object.Pose.Rot[3]) + (x21 * (*_x0).Object.Pose.Rot[2]);
	const FLT x44 = x43 * x37;
	const FLT x45 = (-1 * x44 * x26) + (x41 * x42) + (-1 * x0 * x38) + (-1 * x40 * x28);
	const FLT x46 = x0 * x40;
	const FLT x47 = x38 * x28;
	const FLT x48 = x42 * x43;
	const FLT x49 = x41 * x37;
	const FLT x50 = x49 * x26;
	const FLT x51 = x50 + x48 + (-1 * x46) + x47;
	const FLT x52 = x0 * x44;
	const FLT x53 = x42 * x39;
	const FLT x54 = x49 * x28;
	const FLT x55 = x38 * x26;
	const FLT x56 = (-1 * x55) + x54 + x52 + x53;
	const FLT x57 = (-1 * x56 * sensor_pt[0]) + (x45 * sensor_pt[2]) + (x51 * sensor_pt[1]);
	const FLT x58 = x0 * x49;
	const FLT x59 = x42 * x23;
	const FLT x60 = x44 * x28;
	const FLT x61 = x40 * x26;
	const FLT x62 = x61 + (-1 * x60) + x58 + x59;
	const FLT x63 = (-1 * x62 * sensor_pt[1]) + (x45 * sensor_pt[0]) + (x56 * sensor_pt[2]);
	const FLT x64 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x65 = (*_x0).Object.Pose.Pos[1] + (2 * ((x63 * x62) + (-1 * x51 * x57))) +
					(*error_model).Object.Pose.Pos[1] + (x64 * ((*_x0).Object.Acc[1] + (*error_model).Object.Acc[1])) +
					sensor_pt[1] + (dt * ((*_x0).Object.Velocity.Pos[1] + (*error_model).Object.Velocity.Pos[1]));
	const FLT x66 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[1];
	const FLT x67 = sin(x66);
	const FLT x68 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[0];
	const FLT x69 = sin(x68);
	const FLT x70 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[2];
	const FLT x71 = cos(x70);
	const FLT x72 = x71 * x69;
	const FLT x73 = cos(x68);
	const FLT x74 = sin(x70);
	const FLT x75 = cos(x66);
	const FLT x76 = x75 * x74;
	const FLT x77 = (x73 * x76) + (-1 * x72 * x67);
	const FLT x78 = x71 * x73;
	const FLT x79 = (x78 * x67) + (x76 * x69);
	const FLT x80 = x74 * x67;
	const FLT x81 = (x78 * x75) + (x80 * x69);
	const FLT x82 = (x72 * x75) + (-1 * x80 * x73);
	const FLT x83 = 1. / sqrt((x82 * x82) + (x81 * x81) + (x77 * x77) + (x79 * x79));
	const FLT x84 = x81 * x83;
	const FLT x85 = x82 * x83;
	const FLT x86 = x83 * (*_x0).Lighthouse.Rot[0];
	const FLT x87 = x83 * x79;
	const FLT x88 = (x87 * (*_x0).Lighthouse.Rot[1]) + (x84 * (*_x0).Lighthouse.Rot[3]) + (x86 * x77) +
					(-1 * x85 * (*_x0).Lighthouse.Rot[2]);
	const FLT x89 = x83 * x77;
	const FLT x90 = (-1 * x89 * (*_x0).Lighthouse.Rot[1]) + (x86 * x79) + (x85 * (*_x0).Lighthouse.Rot[3]) +
					(x84 * (*_x0).Lighthouse.Rot[2]);
	const FLT x91 = (-1 * x85 * (*_x0).Lighthouse.Rot[1]) + (x81 * x86) + (-1 * x89 * (*_x0).Lighthouse.Rot[3]) +
					(-1 * x87 * (*_x0).Lighthouse.Rot[2]);
	const FLT x92 = (x84 * (*_x0).Lighthouse.Rot[1]) + (-1 * x87 * (*_x0).Lighthouse.Rot[3]) +
					(x89 * (*_x0).Lighthouse.Rot[2]) + (x82 * x86);
	const FLT x93 = 1. / sqrt((x88 * x88) + (x92 * x92) + (x91 * x91) + (x90 * x90));
	const FLT x94 = x93 * x91;
	const FLT x95 = (-1 * x51 * sensor_pt[2]) + (x45 * sensor_pt[1]) + (x62 * sensor_pt[0]);
	const FLT x96 = (*_x0).Object.Pose.Pos[0] + (2 * ((x57 * x56) + (-1 * x62 * x95))) +
					(*error_model).Object.Pose.Pos[0] + (x64 * ((*_x0).Object.Acc[0] + (*error_model).Object.Acc[0])) +
					(dt * ((*_x0).Object.Velocity.Pos[0] + (*error_model).Object.Velocity.Pos[0])) + sensor_pt[0];
	const FLT x97 = x88 * x93;
	const FLT x98 = (dt * ((*_x0).Object.Velocity.Pos[2] + (*error_model).Object.Velocity.Pos[2])) +
					(*error_model).Object.Pose.Pos[2] + sensor_pt[2] + (2 * ((x51 * x95) + (-1 * x63 * x56))) +
					(x64 * ((*_x0).Object.Acc[2] + (*error_model).Object.Acc[2])) + (*_x0).Object.Pose.Pos[2];
	const FLT x99 = x93 * x92;
	const FLT x100 = (x65 * x94) + (x99 * x98) + (-1 * x97 * x96);
	const FLT x101 = x93 * x90;
	const FLT x102 = (x96 * x101) + (-1 * x65 * x99) + (x98 * x94);
	const FLT x103 = (*_x0).Lighthouse.Pos[1] + (*error_model).Lighthouse.Pos[1];
	const FLT x104 = (*_x0).Lighthouse.Pos[0] + (*error_model).Lighthouse.Pos[0];
	const FLT x105 = (*_x0).Lighthouse.Pos[2] + (*error_model).Lighthouse.Pos[2];
	const FLT x106 = (x99 * x105) + (x94 * x103) + (-1 * x97 * x104);
	const FLT x107 = (x94 * x105) + (x101 * x104) + (-1 * x99 * x103);
	const FLT x108 =
		(2 * ((-1 * x101 * x102) + (x97 * x100))) + x96 + (-1 * (x104 + (2 * ((-1 * x101 * x107) + (x97 * x106)))));
	const FLT x109 = (x97 * x103) + (-1 * x101 * x105) + (x94 * x104);
	const FLT x110 = (x65 * x97) + (x96 * x94) + (-1 * x98 * x101);
	const FLT x111 =
		x98 + (-1 * (x105 + (2 * ((-1 * x99 * x106) + (x101 * x109))))) + (2 * ((-1 * x99 * x100) + (x101 * x110)));
	const FLT x112 = x111 * x111;
	const FLT x113 = 1. / x112;
	const FLT x114 = 2 * x56;
	const FLT x115 = x45 * x114;
	const FLT x116 = 2 * x62;
	const FLT x117 = (x51 * x116) + (-1 * x115);
	const FLT x118 = (-1 * x54) + x55 + (-1 * x53) + (-1 * x52);
	const FLT x119 = 2 * x51;
	const FLT x120 = x45 * x116;
	const FLT x121 = x120 + (-1 * x118 * x119);
	const FLT x122 = 1 + (x118 * x114) + (-2 * (x62 * x62));
	const FLT x123 = (-1 * x97 * x122) + (x99 * x117) + (x94 * x121);
	const FLT x124 = 2 * x99;
	const FLT x125 = (x97 * x121) + (-1 * x101 * x117) + (x94 * x122);
	const FLT x126 = 2 * x101;
	const FLT x127 = x117 + (-1 * x124 * x123) + (x126 * x125);
	const FLT x128 = x113 * x127;
	const FLT x129 = (x94 * x117) + (x101 * x122) + (-1 * x99 * x121);
	const FLT x130 = 2 * x97;
	const FLT x131 = (-1 * x126 * x129) + x122 + (x123 * x130);
	const FLT x132 = 1. / x111;
	const FLT x133 = -1 * x111;
	const FLT x134 = x108 * x108;
	const FLT x135 =
		2 * (1. / (x112 + x134)) * x112 * atan2(x108, x133) * ((*error_model).BSD1.curve + (*_x0).BSD1.curve);
	const FLT x136 = x121 + (x124 * x129) + (-1 * x125 * x130);
	const FLT x137 =
		x65 + (-1 * (x103 + (2 * ((-1 * x97 * x109) + (x99 * x107))))) + (2 * ((-1 * x97 * x110) + (x99 * x102)));
	const FLT x138 = 2 * x137;
	const FLT x139 = 2 * x111;
	const FLT x140 = (*error_model).BSD1.tilt + (*_x0).BSD1.tilt;
	const FLT x141 = (x137 * x137) + x112;
	const FLT x142 = 1.0 / 2.0 * x108 * x140 * (1. / (x141 * sqrt(x141)));
	const FLT x143 = x140 * (1. / sqrt(x141));
	const FLT x144 = 1. / x141;
	const FLT x145 = 1. / sqrt(1 + (-1 * x134 * (x140 * x140) * x144));
	const FLT x146 = x112 * x144;
	const FLT x147 = (-1 * ((-1 * x128 * x137) + (x132 * x136)) * x146) +
					 (-1 * x145 * ((x131 * x143) + (-1 * ((x127 * x139) + (x138 * x136)) * x142)));
	const FLT x148 = sin(1.5707963267949 + (*_x0).BSD1.gibpha + (-1 * asin(x108 * x143)) +
						 (-1 * ((*error_model).BSD1.phase + (*_x0).BSD1.phase)) + (*error_model).BSD1.gibpha +
						 (-1 * atan2(-1 * x137, x133))) *
					 ((*error_model).BSD1.gibmag + (*_x0).BSD1.gibmag);
	const FLT x149 = x60 + (-1 * x61) + (-1 * x59) + (-1 * x58);
	const FLT x150 = 1 + (x116 * x149) + (-2 * (x51 * x51));
	const FLT x151 = x45 * x119;
	const FLT x152 = x151 + (-1 * x114 * x149);
	const FLT x153 = (x51 * x114) + (-1 * x120);
	const FLT x154 = (-1 * x97 * x153) + (x94 * x150) + (x99 * x152);
	const FLT x155 = (-1 * x101 * x152) + (x97 * x150) + (x94 * x153);
	const FLT x156 = x152 + (-1 * x124 * x154) + (x126 * x155);
	const FLT x157 = x113 * x156;
	const FLT x158 = (x101 * x153) + (x94 * x152) + (-1 * x99 * x150);
	const FLT x159 = x153 + (-1 * x126 * x158) + (x130 * x154);
	const FLT x160 = x150 + (x124 * x158) + (-1 * x130 * x155);
	const FLT x161 = (-1 * ((-1 * x137 * x157) + (x160 * x132)) * x146) +
					 (-1 * x145 * ((x143 * x159) + (-1 * ((x139 * x156) + (x160 * x138)) * x142)));
	const FLT x162 = (-1 * x48) + (-1 * x50) + (-1 * x47) + x46;
	const FLT x163 = 1 + (x119 * x162) + (-2 * (x56 * x56));
	const FLT x164 = (x56 * x116) + (-1 * x151);
	const FLT x165 = x115 + (-1 * x116 * x162);
	const FLT x166 = (x99 * x163) + (-1 * x97 * x165) + (x94 * x164);
	const FLT x167 = (-1 * x101 * x163) + (x97 * x164) + (x94 * x165);
	const FLT x168 = x163 + (-1 * x124 * x166) + (x126 * x167);
	const FLT x169 = x113 * x168;
	const FLT x170 = (x94 * x163) + (x101 * x165) + (-1 * x99 * x164);
	const FLT x171 = x165 + (-1 * x126 * x170) + (x166 * x130);
	const FLT x172 = x164 + (-1 * x167 * x130) + (x124 * x170);
	const FLT x173 = (-1 * ((-1 * x169 * x137) + (x172 * x132)) * x146) +
					 (-1 * x145 * ((x171 * x143) + (-1 * ((x168 * x139) + (x172 * x138)) * x142)));
	cnMatrixOptionalSet(Hx, 0, 0, x147 + (((-1 * x131 * x132) + (x108 * x128)) * x135) + (x148 * x147));
	cnMatrixOptionalSet(Hx, 0, 1, (((-1 * x132 * x159) + (x108 * x157)) * x135) + x161 + (x161 * x148));
	cnMatrixOptionalSet(Hx, 0, 2, x173 + (((-1 * x171 * x132) + (x108 * x169)) * x135) + (x173 * x148));
}

// Full version Jacobian of SurviveJointKalmanErrorModel_LightMeas_y_gen1 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]

static inline void SurviveJointKalmanErrorModel_LightMeas_y_gen1_jac_sensor_pt_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveJointKalmanModel *_x0,
	const SurviveJointKalmanErrorModel *error_model, const FLT *sensor_pt) {
	if (hx != 0) {
		hx->data[0] = SurviveJointKalmanErrorModel_LightMeas_y_gen1(dt, _x0, error_model, sensor_pt);
	}
	if (Hx != 0) {
		SurviveJointKalmanErrorModel_LightMeas_y_gen1_jac_sensor_pt(Hx, dt, _x0, error_model, sensor_pt);
	}
}
static inline FLT SurviveJointKalmanErrorModel_LightMeas_x_gen2(const FLT dt, const SurviveJointKalmanModel *_x0,
																const SurviveJointKalmanErrorModel *error_model,
																const FLT *sensor_pt) {
	const FLT x0 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[2];
	const FLT x1 = cos(x0);
	const FLT x2 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[0];
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[1];
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = cos(x2);
	const FLT x8 = cos(x4);
	const FLT x9 = sin(x0);
	const FLT x10 = x8 * x9;
	const FLT x11 = (x7 * x10) + (-1 * x1 * x6);
	const FLT x12 = x5 * x7;
	const FLT x13 = (x1 * x12) + (x3 * x10);
	const FLT x14 = x1 * x8;
	const FLT x15 = (x7 * x14) + (x6 * x9);
	const FLT x16 = (x3 * x14) + (-1 * x9 * x12);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x15 * x15) + (x11 * x11) + (x13 * x13));
	const FLT x18 = x17 * x16;
	const FLT x19 = x15 * x17;
	const FLT x20 = x13 * x17;
	const FLT x21 = x11 * x17;
	const FLT x22 = (x20 * (*_x0).Lighthouse.Rot[0]) + (x18 * (*_x0).Lighthouse.Rot[3]) +
					(-1 * x21 * (*_x0).Lighthouse.Rot[1]) + (x19 * (*_x0).Lighthouse.Rot[2]);
	const FLT x23 = x17 * (*_x0).Lighthouse.Rot[3];
	const FLT x24 = (x21 * (*_x0).Lighthouse.Rot[0]) + (x23 * x15) + (x20 * (*_x0).Lighthouse.Rot[1]) +
					(-1 * x18 * (*_x0).Lighthouse.Rot[2]);
	const FLT x25 = (-1 * x18 * (*_x0).Lighthouse.Rot[1]) + (x19 * (*_x0).Lighthouse.Rot[0]) + (-1 * x23 * x11) +
					(-1 * x20 * (*_x0).Lighthouse.Rot[2]);
	const FLT x26 = (-1 * x20 * (*_x0).Lighthouse.Rot[3]) + (x19 * (*_x0).Lighthouse.Rot[1]) +
					(x21 * (*_x0).Lighthouse.Rot[2]) + (x18 * (*_x0).Lighthouse.Rot[0]);
	const FLT x27 = 1. / sqrt((x26 * x26) + (x24 * x24) + (x25 * x25) + (x22 * x22));
	const FLT x28 = (*_x0).Lighthouse.Pos[2] + (*error_model).Lighthouse.Pos[2];
	const FLT x29 = x22 * x27;
	const FLT x30 = (*_x0).Lighthouse.Pos[0] + (*error_model).Lighthouse.Pos[0];
	const FLT x31 = x25 * x27;
	const FLT x32 = (*_x0).Lighthouse.Pos[1] + (*error_model).Lighthouse.Pos[1];
	const FLT x33 = x24 * x27;
	const FLT x34 = x27 * ((x32 * x33) + (-1 * x28 * x29) + (x30 * x31));
	const FLT x35 = x26 * x27;
	const FLT x36 = (x35 * x28) + (x32 * x31) + (-1 * x30 * x33);
	const FLT x37 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[1];
	const FLT x38 = cos(x37);
	const FLT x39 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[2];
	const FLT x40 = sin(x39);
	const FLT x41 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[0];
	const FLT x42 = sin(x41);
	const FLT x43 = x40 * x42;
	const FLT x44 = cos(x39);
	const FLT x45 = sin(x37);
	const FLT x46 = cos(x41);
	const FLT x47 = x45 * x46;
	const FLT x48 = (x44 * x47) + (x43 * x38);
	const FLT x49 = x44 * x38;
	const FLT x50 = (x46 * x49) + (x43 * x45);
	const FLT x51 = (x40 * x46 * x38) + (-1 * x42 * x44 * x45);
	const FLT x52 = (x42 * x49) + (-1 * x40 * x47);
	const FLT x53 = 1. / sqrt((x51 * x51) + (x48 * x48) + (x52 * x52) + (x50 * x50));
	const FLT x54 = x50 * x53;
	const FLT x55 = x53 * (*_x0).Object.Pose.Rot[2];
	const FLT x56 = x53 * x51;
	const FLT x57 = x53 * x48;
	const FLT x58 = (x57 * (*_x0).Object.Pose.Rot[1]) + (x56 * (*_x0).Object.Pose.Rot[0]) +
					(x54 * (*_x0).Object.Pose.Rot[3]) + (-1 * x52 * x55);
	const FLT x59 = (*_x0).Object.Velocity.AxisAngleRot[2] + (*error_model).Object.Velocity.AxisAngleRot[2];
	const FLT x60 = dt * dt;
	const FLT x61 = x60 * (x59 * x59);
	const FLT x62 = (*_x0).Object.Velocity.AxisAngleRot[0] + (*error_model).Object.Velocity.AxisAngleRot[0];
	const FLT x63 = x60 * (x62 * x62);
	const FLT x64 = (*_x0).Object.Velocity.AxisAngleRot[1] + (*error_model).Object.Velocity.AxisAngleRot[1];
	const FLT x65 = x60 * (x64 * x64);
	const FLT x66 = 1e-10 + x65 + x61 + x63;
	const FLT x67 = sqrt(x66);
	const FLT x68 = 0.5 * x67;
	const FLT x69 = sin(x68);
	const FLT x70 = (1. / x66) * (x69 * x69);
	const FLT x71 = cos(x68);
	const FLT x72 = 1. / sqrt((x71 * x71) + (x70 * x63) + (x70 * x61) + (x70 * x65));
	const FLT x73 = dt * x72 * (1. / x67) * x69;
	const FLT x74 = x73 * x59;
	const FLT x75 = x53 * x52;
	const FLT x76 = (x57 * (*_x0).Object.Pose.Rot[0]) + (-1 * x56 * (*_x0).Object.Pose.Rot[1]) + (x50 * x55) +
					(x75 * (*_x0).Object.Pose.Rot[3]);
	const FLT x77 = x73 * x64;
	const FLT x78 = (-1 * x75 * (*_x0).Object.Pose.Rot[1]) + (x54 * (*_x0).Object.Pose.Rot[0]) +
					(-1 * x56 * (*_x0).Object.Pose.Rot[3]) + (-1 * x55 * x48);
	const FLT x79 = x71 * x72;
	const FLT x80 = (x54 * (*_x0).Object.Pose.Rot[1]) + (x75 * (*_x0).Object.Pose.Rot[0]) +
					(-1 * x57 * (*_x0).Object.Pose.Rot[3]) + (x51 * x55);
	const FLT x81 = x73 * x62;
	const FLT x82 = (x79 * x78) + (-1 * x74 * x58) + (-1 * x80 * x81) + (-1 * x77 * x76);
	const FLT x83 = (x81 * x76) + (-1 * x80 * x77) + (x78 * x74) + (x79 * x58);
	const FLT x84 = (x80 * x79) + (-1 * x74 * x76) + (x81 * x78) + (x77 * x58);
	const FLT x85 = (-1 * x84 * sensor_pt[2]) + (x82 * sensor_pt[1]) + (x83 * sensor_pt[0]);
	const FLT x86 = (-1 * x81 * x58) + (x78 * x77) + (x80 * x74) + (x79 * x76);
	const FLT x87 = (-1 * x86 * sensor_pt[0]) + (x82 * sensor_pt[2]) + (x84 * sensor_pt[1]);
	const FLT x88 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x89 = (*_x0).Object.Pose.Pos[0] + (2 * ((x86 * x87) + (-1 * x83 * x85))) +
					(*error_model).Object.Pose.Pos[0] + (x88 * ((*_x0).Object.Acc[0] + (*error_model).Object.Acc[0])) +
					sensor_pt[0] + (dt * ((*_x0).Object.Velocity.Pos[0] + (*error_model).Object.Velocity.Pos[0]));
	const FLT x90 = (x82 * sensor_pt[0]) + (-1 * x83 * sensor_pt[1]) + (x86 * sensor_pt[2]);
	const FLT x91 = (dt * ((*_x0).Object.Velocity.Pos[2] + (*error_model).Object.Velocity.Pos[2])) +
					(*error_model).Object.Pose.Pos[2] + (2 * ((x84 * x85) + (-1 * x86 * x90))) + sensor_pt[2] +
					(x88 * ((*_x0).Object.Acc[2] + (*error_model).Object.Acc[2])) + (*_x0).Object.Pose.Pos[2];
	const FLT x92 = (*_x0).Object.Pose.Pos[1] + (2 * ((x83 * x90) + (-1 * x84 * x87))) + sensor_pt[1] +
					(dt * ((*_x0).Object.Velocity.Pos[1] + (*error_model).Object.Velocity.Pos[1])) +
					(*error_model).Object.Pose.Pos[1] + (x88 * ((*_x0).Object.Acc[1] + (*error_model).Object.Acc[1]));
	const FLT x93 = (x92 * x33) + (x89 * x31) + (-1 * x91 * x29);
	const FLT x94 = (x91 * x35) + (x92 * x31) + (-1 * x89 * x33);
	const FLT x95 =
		x91 + (-1 * (x28 + (2 * ((-1 * x36 * x35) + (x34 * x22))))) + (2 * ((-1 * x94 * x35) + (x93 * x29)));
	const FLT x96 = (x89 * x29) + (-1 * x92 * x35) + (x91 * x31);
	const FLT x97 = (x30 * x29) + (x31 * x28) + (-1 * x32 * x35);
	const FLT x98 =
		x89 + (2 * ((-1 * x96 * x29) + (x94 * x33))) + (-1 * (x30 + (2 * ((-1 * x97 * x29) + (x33 * x36)))));
	const FLT x99 = atan2(-1 * x95, x98);
	const FLT x100 =
		x92 + (-1 * (x32 + (2 * ((-1 * x34 * x24) + (x97 * x35))))) + (2 * ((-1 * x93 * x33) + (x96 * x35)));
	const FLT x101 = (x98 * x98) + (x95 * x95);
	const FLT x102 = 0.523598775598299 + (*error_model).BSD0.tilt + (*_x0).BSD0.tilt;
	const FLT x103 = cos(x102);
	const FLT x104 = asin(x100 * (1. / x103) * (1. / sqrt(x101 + (x100 * x100))));
	const FLT x105 = 0.0028679863 + (x104 * (-8.0108022e-06 + (-8.0108022e-06 * x104)));
	const FLT x106 = 5.3685255e-06 + (x105 * x104);
	const FLT x107 = 0.0076069798 + (x104 * x106);
	const FLT x108 = (1. / sqrt(x101)) * x100 * tan(x102);
	const FLT x109 = (*_x0).BSD0.curve + (*error_model).BSD0.curve +
					 (sin((-1 * asin(x108)) + x99 + (*error_model).BSD0.ogeephase + (*_x0).BSD0.ogeephase) *
					  ((*error_model).BSD0.ogeemag + (*_x0).BSD0.ogeemag));
	const FLT x110 =
		(-1 * asin(x108 +
				   (x109 * (x104 * x104) * x107 *
					(1. / (x103 +
						   (-1 * x109 * sin(x102) *
							((x104 * (x107 + (x104 * (x106 + (x104 * (x105 + (x104 * (-8.0108022e-06 +
																					  (-1.60216044e-05 * x104))))))))) +
							 (x104 * x107)))))))) +
		x99;
	return -1.5707963267949 + x110 +
		   (sin(x110 + (*error_model).BSD0.gibpha + (*_x0).BSD0.gibpha) *
			((*error_model).BSD0.gibmag + (*_x0).BSD0.gibmag)) +
		   (-1 * ((*error_model).BSD0.phase + (*_x0).BSD0.phase));
}

// Jacobian of SurviveJointKalmanErrorModel_LightMeas_x_gen2 wrt [(*_x0).Lighthouse.Pos[0], (*_x0).Lighthouse.Pos[1],
// (*_x0).Lighthouse.Pos[2], (*_x0).Lighthouse.Rot[0], (*_x0).Lighthouse.Rot[1], (*_x0).Lighthouse.Rot[2],
// (*_x0).Lighthouse.Rot[3], (*_x0).Object.Acc[0], (*_x0).Object.Acc[1], (*_x0).Object.Acc[2],
// (*_x0).Object.IMUBias.AccBias[0], (*_x0).Object.IMUBias.AccBias[1], (*_x0).Object.IMUBias.AccBias[2],
// (*_x0).Object.IMUBias.AccScale[0], (*_x0).Object.IMUBias.AccScale[1], (*_x0).Object.IMUBias.AccScale[2],
// (*_x0).Object.IMUBias.GyroBias[0], (*_x0).Object.IMUBias.GyroBias[1], (*_x0).Object.IMUBias.GyroBias[2],
// (*_x0).Object.IMUBias.IMUCorrection[0], (*_x0).Object.IMUBias.IMUCorrection[1],
// (*_x0).Object.IMUBias.IMUCorrection[2], (*_x0).Object.IMUBias.IMUCorrection[3], (*_x0).Object.Pose.Pos[0],
// (*_x0).Object.Pose.Pos[1], (*_x0).Object.Pose.Pos[2], (*_x0).Object.Pose.Rot[0], (*_x0).Object.Pose.Rot[1],
// (*_x0).Object.Pose.Rot[2], (*_x0).Object.Pose.Rot[3], (*_x0).Object.Velocity.AxisAngleRot[0],
// (*_x0).Object.Velocity.AxisAngleRot[1], (*_x0).Object.Velocity.AxisAngleRot[2], (*_x0).Object.Velocity.Pos[0],
// (*_x0).Object.Velocity.Pos[1], (*_x0).Object.Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f4d1c32c430>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32c6a0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32c730>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32c8e0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32c040>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32c610>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32c580>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32c820>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32c4c0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32c880>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32c1f0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32c2b0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32c3a0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32cee0>]
static inline void SurviveJointKalmanErrorModel_LightMeas_x_gen2_jac_x0(CnMat *Hx, const FLT dt,
																		const SurviveJointKalmanModel *_x0,
																		const SurviveJointKalmanErrorModel *error_model,
																		const FLT *sensor_pt) {
	const FLT x0 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[2];
	const FLT x1 = cos(x0);
	const FLT x2 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[1];
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[0];
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = sin(x0);
	const FLT x8 = cos(x4);
	const FLT x9 = cos(x2);
	const FLT x10 = x8 * x9;
	const FLT x11 = (x7 * x10) + (-1 * x1 * x6);
	const FLT x12 = x5 * x9;
	const FLT x13 = x3 * x8;
	const FLT x14 = (x1 * x13) + (x7 * x12);
	const FLT x15 = (x1 * x10) + (x6 * x7);
	const FLT x16 = (x1 * x12) + (-1 * x7 * x13);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x15 * x15) + (x11 * x11) + (x14 * x14));
	const FLT x18 = x15 * x17;
	const FLT x19 = x17 * x16;
	const FLT x20 = x17 * (*_x0).Lighthouse.Rot[0];
	const FLT x21 = x14 * x17;
	const FLT x22 = (x20 * x11) + (x18 * (*_x0).Lighthouse.Rot[3]) + (x21 * (*_x0).Lighthouse.Rot[1]) +
					(-1 * x19 * (*_x0).Lighthouse.Rot[2]);
	const FLT x23 = x22 * x22;
	const FLT x24 = x11 * x17;
	const FLT x25 = (x20 * x14) + (-1 * x24 * (*_x0).Lighthouse.Rot[1]) + (x19 * (*_x0).Lighthouse.Rot[3]) +
					(x18 * (*_x0).Lighthouse.Rot[2]);
	const FLT x26 = x25 * x25;
	const FLT x27 = (x20 * x15) + (-1 * x24 * (*_x0).Lighthouse.Rot[3]) + (-1 * x19 * (*_x0).Lighthouse.Rot[1]) +
					(-1 * x21 * (*_x0).Lighthouse.Rot[2]);
	const FLT x28 = (x18 * (*_x0).Lighthouse.Rot[1]) + (-1 * x21 * (*_x0).Lighthouse.Rot[3]) +
					(x24 * (*_x0).Lighthouse.Rot[2]) + (x20 * x16);
	const FLT x29 = x28 * x28;
	const FLT x30 = x29 + x23 + (x27 * x27) + x26;
	const FLT x31 = 1. / x30;
	const FLT x32 = 2 * x31;
	const FLT x33 = x32 * x23;
	const FLT x34 = x32 * x26;
	const FLT x35 = -1 + x34;
	const FLT x36 = x35 + x33;
	const FLT x37 = (*_x0).Object.Velocity.AxisAngleRot[2] + (*error_model).Object.Velocity.AxisAngleRot[2];
	const FLT x38 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[0];
	const FLT x39 = sin(x38);
	const FLT x40 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[2];
	const FLT x41 = sin(x40);
	const FLT x42 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[1];
	const FLT x43 = sin(x42);
	const FLT x44 = x41 * x43;
	const FLT x45 = cos(x38);
	const FLT x46 = cos(x42);
	const FLT x47 = cos(x40);
	const FLT x48 = x46 * x47;
	const FLT x49 = (x45 * x48) + (x44 * x39);
	const FLT x50 = x41 * x46;
	const FLT x51 = x43 * x47;
	const FLT x52 = (x51 * x45) + (x50 * x39);
	const FLT x53 = (x50 * x45) + (-1 * x51 * x39);
	const FLT x54 = (x48 * x39) + (-1 * x44 * x45);
	const FLT x55 = 1. / sqrt((x53 * x53) + (x52 * x52) + (x54 * x54) + (x49 * x49));
	const FLT x56 = x55 * (*_x0).Object.Pose.Rot[3];
	const FLT x57 = x54 * x55;
	const FLT x58 = x53 * x55;
	const FLT x59 = x52 * x55;
	const FLT x60 = (x56 * x49) + (x59 * (*_x0).Object.Pose.Rot[1]) + (x58 * (*_x0).Object.Pose.Rot[0]) +
					(-1 * x57 * (*_x0).Object.Pose.Rot[2]);
	const FLT x61 = dt * dt;
	const FLT x62 = x37 * x37;
	const FLT x63 = x61 * x62;
	const FLT x64 = (*_x0).Object.Velocity.AxisAngleRot[0] + (*error_model).Object.Velocity.AxisAngleRot[0];
	const FLT x65 = x64 * x64;
	const FLT x66 = x61 * x65;
	const FLT x67 = (*_x0).Object.Velocity.AxisAngleRot[1] + (*error_model).Object.Velocity.AxisAngleRot[1];
	const FLT x68 = x67 * x67;
	const FLT x69 = x61 * x68;
	const FLT x70 = 1e-10 + x69 + x63 + x66;
	const FLT x71 = sqrt(x70);
	const FLT x72 = 0.5 * x71;
	const FLT x73 = sin(x72);
	const FLT x74 = x73 * x73;
	const FLT x75 = 1. / x70;
	const FLT x76 = x75 * x74;
	const FLT x77 = cos(x72);
	const FLT x78 = (x76 * x66) + (x76 * x63) + (x77 * x77) + (x76 * x69);
	const FLT x79 = 1. / sqrt(x78);
	const FLT x80 = (1. / x71) * x73;
	const FLT x81 = dt * x80;
	const FLT x82 = x81 * x79;
	const FLT x83 = x82 * x60;
	const FLT x84 = x55 * x49;
	const FLT x85 = (x59 * (*_x0).Object.Pose.Rot[0]) + (x84 * (*_x0).Object.Pose.Rot[2]) +
					(-1 * x58 * (*_x0).Object.Pose.Rot[1]) + (x54 * x56);
	const FLT x86 = x85 * x79;
	const FLT x87 = x81 * x86;
	const FLT x88 = (x84 * (*_x0).Object.Pose.Rot[0]) + (-1 * x57 * (*_x0).Object.Pose.Rot[1]) + (-1 * x53 * x56) +
					(-1 * x59 * (*_x0).Object.Pose.Rot[2]);
	const FLT x89 = x79 * x77;
	const FLT x90 = x88 * x89;
	const FLT x91 = (x84 * (*_x0).Object.Pose.Rot[1]) + (-1 * x52 * x56) + (x57 * (*_x0).Object.Pose.Rot[0]) +
					(x58 * (*_x0).Object.Pose.Rot[2]);
	const FLT x92 = x82 * x91;
	const FLT x93 = (-1 * x64 * x92) + x90 + (-1 * x83 * x37) + (-1 * x87 * x67);
	const FLT x94 = x89 * x91;
	const FLT x95 = x82 * x88;
	const FLT x96 = (x64 * x95) + x94 + (-1 * x87 * x37) + (x83 * x67);
	const FLT x97 = x89 * x85;
	const FLT x98 = (-1 * x83 * x64) + (x67 * x95) + (x92 * x37) + x97;
	const FLT x99 = (-1 * x98 * sensor_pt[0]) + (x93 * sensor_pt[2]) + (x96 * sensor_pt[1]);
	const FLT x100 = x89 * x60;
	const FLT x101 = (-1 * x67 * x92) + (x95 * x37) + (x87 * x64) + x100;
	const FLT x102 = (-1 * x101 * sensor_pt[1]) + (x93 * sensor_pt[0]) + (x98 * sensor_pt[2]);
	const FLT x103 = dt * fabs(dt);
	const FLT x104 = 1.0 / 2.0 * x103;
	const FLT x105 = (*error_model).Object.Pose.Pos[1] + (2 * ((x101 * x102) + (-1 * x99 * x96))) + sensor_pt[1] +
					 (dt * ((*_x0).Object.Velocity.Pos[1] + (*error_model).Object.Velocity.Pos[1])) +
					 (*_x0).Object.Pose.Pos[1] + (x104 * ((*_x0).Object.Acc[1] + (*error_model).Object.Acc[1]));
	const FLT x106 = 1. / sqrt(x30);
	const FLT x107 = x27 * x106;
	const FLT x108 = (x93 * sensor_pt[1]) + (-1 * x96 * sensor_pt[2]) + (x101 * sensor_pt[0]);
	const FLT x109 = (*_x0).Object.Pose.Pos[0] + (*error_model).Object.Pose.Pos[0] +
					 (2 * ((x99 * x98) + (-1 * x101 * x108))) +
					 (x104 * ((*_x0).Object.Acc[0] + (*error_model).Object.Acc[0])) + sensor_pt[0] +
					 (dt * ((*_x0).Object.Velocity.Pos[0] + (*error_model).Object.Velocity.Pos[0]));
	const FLT x110 = x22 * x106;
	const FLT x111 = (dt * ((*_x0).Object.Velocity.Pos[2] + (*error_model).Object.Velocity.Pos[2])) +
					 (*error_model).Object.Pose.Pos[2] + sensor_pt[2] + (2 * ((x96 * x108) + (-1 * x98 * x102))) +
					 (x104 * ((*_x0).Object.Acc[2] + (*error_model).Object.Acc[2])) + (*_x0).Object.Pose.Pos[2];
	const FLT x112 = x28 * x106;
	const FLT x113 = (x111 * x112) + (x105 * x107) + (-1 * x109 * x110);
	const FLT x114 = x25 * x106;
	const FLT x115 = (x109 * x114) + (-1 * x105 * x112) + (x107 * x111);
	const FLT x116 = (*_x0).Lighthouse.Pos[1] + (*error_model).Lighthouse.Pos[1];
	const FLT x117 = (*_x0).Lighthouse.Pos[0] + (*error_model).Lighthouse.Pos[0];
	const FLT x118 = (*_x0).Lighthouse.Pos[2] + (*error_model).Lighthouse.Pos[2];
	const FLT x119 = (x112 * x118) + (x107 * x116) + (-1 * x110 * x117);
	const FLT x120 = (x114 * x117) + (x107 * x118) + (-1 * x112 * x116);
	const FLT x121 =
		x109 + (2 * ((-1 * x114 * x115) + (x110 * x113))) + (-1 * (x117 + (2 * ((-1 * x114 * x120) + (x110 * x119)))));
	const FLT x122 = 2 * x121;
	const FLT x123 = x22 * x28;
	const FLT x124 = x32 * x123;
	const FLT x125 = -1 * x124;
	const FLT x126 = x32 * x25;
	const FLT x127 = x27 * x126;
	const FLT x128 = -1 * x127;
	const FLT x129 = x128 + x125;
	const FLT x130 = (x110 * x116) + (-1 * x118 * x114) + (x107 * x117);
	const FLT x131 = (x105 * x110) + (x109 * x107) + (-1 * x111 * x114);
	const FLT x132 =
		x111 + (-1 * (x118 + (2 * ((-1 * x112 * x119) + (x114 * x130))))) + (2 * ((-1 * x112 * x113) + (x114 * x131)));
	const FLT x133 = 2 * x132;
	const FLT x134 = (x129 * x133) + (x36 * x122);
	const FLT x135 = 0.523598775598299 + (*error_model).BSD0.tilt + (*_x0).BSD0.tilt;
	const FLT x136 = tan(x135);
	const FLT x137 = x121 * x121;
	const FLT x138 = x137 + (x132 * x132);
	const FLT x139 =
		(-1 * (x116 + (2 * ((-1 * x110 * x130) + (x112 * x120))))) + x105 + (2 * ((-1 * x110 * x131) + (x112 * x115)));
	const FLT x140 = 1.0 / 2.0 * x139;
	const FLT x141 = (1. / (x138 * sqrt(x138))) * x136 * x140;
	const FLT x142 = x32 * x27;
	const FLT x143 = x22 * x142;
	const FLT x144 = x28 * x126;
	const FLT x145 = -1 * x144;
	const FLT x146 = x145 + x143;
	const FLT x147 = 1. / sqrt(x138);
	const FLT x148 = x136 * x147;
	const FLT x149 = (x146 * x148) + (-1 * x134 * x141);
	const FLT x150 = x139 * x139;
	const FLT x151 = 1. / x138;
	const FLT x152 = x136 * x136;
	const FLT x153 = 1. / sqrt(1 + (-1 * x150 * x151 * x152));
	const FLT x154 = 1. / x121;
	const FLT x155 = x132 * (1. / x137);
	const FLT x156 = x137 * x151;
	const FLT x157 = x156 * ((x36 * x155) + (-1 * x129 * x154));
	const FLT x158 = (-1 * x157) + (x149 * x153);
	const FLT x159 = x138 + x150;
	const FLT x160 = 1. / sqrt(x159);
	const FLT x161 = cos(x135);
	const FLT x162 = 1. / x161;
	const FLT x163 = x160 * x162;
	const FLT x164 = asin(x163 * x139);
	const FLT x165 = 8.0108022e-06 * x164;
	const FLT x166 = -8.0108022e-06 + (-1 * x165);
	const FLT x167 = 0.0028679863 + (x166 * x164);
	const FLT x168 = 5.3685255e-06 + (x167 * x164);
	const FLT x169 = 0.0076069798 + (x168 * x164);
	const FLT x170 = x169 * x164;
	const FLT x171 = -8.0108022e-06 + (-1.60216044e-05 * x164);
	const FLT x172 = x167 + (x164 * x171);
	const FLT x173 = x168 + (x164 * x172);
	const FLT x174 = x169 + (x164 * x173);
	const FLT x175 = (x164 * x174) + x170;
	const FLT x176 = sin(x135);
	const FLT x177 = atan2(-1 * x132, x121);
	const FLT x178 = x139 * x148;
	const FLT x179 = asin(x178) + (-1 * x177) + (-1 * (*error_model).BSD0.ogeephase) + (-1 * (*_x0).BSD0.ogeephase);
	const FLT x180 = sin(x179);
	const FLT x181 = (*error_model).BSD0.ogeemag + (*_x0).BSD0.ogeemag;
	const FLT x182 = (*_x0).BSD0.curve + (*error_model).BSD0.curve + (-1 * x181 * x180);
	const FLT x183 = x176 * x182;
	const FLT x184 = x175 * x183;
	const FLT x185 = x161 + (-1 * x184);
	const FLT x186 = 1. / x185;
	const FLT x187 = x164 * x164;
	const FLT x188 = x187 * x186;
	const FLT x189 = x169 * x188;
	const FLT x190 = x181 * cos(x179);
	const FLT x191 = x189 * x190;
	const FLT x192 = 2 * x139;
	const FLT x193 = x162 * x140 * (1. / (x159 * sqrt(x159)));
	const FLT x194 = (-1 * x193 * (x134 + (x192 * x146))) + (x163 * x146);
	const FLT x195 = 1. / (x161 * x161);
	const FLT x196 = 1. / sqrt(1 + (-1 * x195 * x150 * (1. / x159)));
	const FLT x197 = x173 * x196;
	const FLT x198 = x172 * x196;
	const FLT x199 = x166 * x196;
	const FLT x200 = x199 * x194;
	const FLT x201 = x171 * x196;
	const FLT x202 = 2.40324066e-05 * x164;
	const FLT x203 = x202 * x196;
	const FLT x204 = x165 * x196;
	const FLT x205 = x167 * x196;
	const FLT x206 = (x205 * x194) + (x164 * (x200 + (-1 * x204 * x194)));
	const FLT x207 = x168 * x196;
	const FLT x208 = (x207 * x194) + (x206 * x164);
	const FLT x209 = x169 * x196;
	const FLT x210 = x174 * x196;
	const FLT x211 = x176 * x175;
	const FLT x212 = x211 * x190;
	const FLT x213 = x169 * x187 * (1. / (x185 * x185));
	const FLT x214 = x213 * x182;
	const FLT x215 = x170 * x186 * x196;
	const FLT x216 = 2 * x182;
	const FLT x217 = x215 * x216;
	const FLT x218 = x188 * x182;
	const FLT x219 = x178 + (x189 * x182);
	const FLT x220 = 1. / sqrt(1 + (-1 * (x219 * x219)));
	const FLT x221 =
		x157 + (-1 * x220 *
				(x149 + (x218 * x208) + (-1 * x191 * x158) + (x217 * x194) +
				 (-1 * x214 *
				  ((x212 * x158) +
				   (-1 * x183 *
					((x208 * x164) +
					 (x164 * ((x197 * x194) + x208 +
							  (x164 * (x206 + (x198 * x194) + (x164 * (x200 + (-1 * x203 * x194) + (x201 * x194))))))) +
					 (x210 * x194) + (x209 * x194)))))));
	const FLT x222 = x177 + (*error_model).BSD0.gibpha + (-1 * asin(x219)) + (*_x0).BSD0.gibpha;
	const FLT x223 = cos(x222) * ((*error_model).BSD0.gibmag + (*_x0).BSD0.gibmag);
	const FLT x224 = x32 * x29;
	const FLT x225 = -1 + x224 + x33;
	const FLT x226 = -1 * x143;
	const FLT x227 = x226 + x145;
	const FLT x228 = x28 * x142;
	const FLT x229 = x22 * x126;
	const FLT x230 = -1 * x229;
	const FLT x231 = x230 + x228;
	const FLT x232 = (x231 * x133) + (x227 * x122);
	const FLT x233 = (-1 * x193 * (x232 + (x225 * x192))) + (x225 * x163);
	const FLT x234 = x233 * x199;
	const FLT x235 = (x233 * x205) + (x164 * (x234 + (-1 * x233 * x204)));
	const FLT x236 = (x233 * x207) + (x235 * x164);
	const FLT x237 = (x225 * x148) + (-1 * x232 * x141);
	const FLT x238 = ((x227 * x155) + (-1 * x231 * x154)) * x156;
	const FLT x239 = (-1 * x238) + (x237 * x153);
	const FLT x240 =
		x238 + (-1 * x220 *
				(x237 + (x218 * x236) + (-1 * x239 * x191) + (x217 * x233) +
				 (-1 * x214 *
				  ((x212 * x239) +
				   (-1 * x183 *
					((x233 * x209) +
					 (x164 * (x236 + (x233 * x197) +
							  (x164 * (x235 + (x233 * x198) + (x164 * ((-1 * x233 * x203) + (x233 * x201) + x234)))))) +
					 (x236 * x164) + (x210 * x233)))))));
	const FLT x241 = -1 * x228;
	const FLT x242 = x241 + x230;
	const FLT x243 = x125 + x127;
	const FLT x244 = x35 + x224;
	const FLT x245 = (x244 * x133) + (x243 * x122);
	const FLT x246 = (-1 * x193 * (x245 + (x242 * x192))) + (x242 * x163);
	const FLT x247 = (x242 * x148) + (-1 * x245 * x141);
	const FLT x248 = ((x243 * x155) + (-1 * x244 * x154)) * x156;
	const FLT x249 = (-1 * x248) + (x247 * x153);
	const FLT x250 = x246 * x199;
	const FLT x251 = (x205 * x246) + (x164 * (x250 + (-1 * x204 * x246)));
	const FLT x252 = (x207 * x246) + (x251 * x164);
	const FLT x253 =
		x248 + (-1 * x220 *
				(x247 + (x218 * x252) +
				 (-1 * x214 *
				  ((x212 * x249) +
				   (-1 * x183 *
					((x252 * x164) + (x209 * x246) +
					 (x164 * (x252 + (x246 * x197) +
							  (x164 * (x251 + (x246 * x198) + (x164 * (x250 + (-1 * x203 * x246) + (x201 * x246))))))) +
					 (x210 * x246))))) +
				 (x217 * x246) + (-1 * x249 * x191)));
	const FLT x254 = 2 * x19;
	const FLT x255 = 2 * x22;
	const FLT x256 = 2 * x18;
	const FLT x257 = 2 * x25;
	const FLT x258 = (x21 * x257) + (x28 * x254) + (x27 * x256) + (x24 * x255);
	const FLT x259 = 1. / (x30 * sqrt(x30));
	const FLT x260 = 1.0 / 2.0 * x259;
	const FLT x261 = x260 * x258;
	const FLT x262 = x22 * x261;
	const FLT x263 = x18 * x106;
	const FLT x264 = x263 * x109;
	const FLT x265 = x27 * x109;
	const FLT x266 = x261 * x111;
	const FLT x267 = x21 * x106;
	const FLT x268 = -1 * x267 * x111;
	const FLT x269 = x24 * x106;
	const FLT x270 = x269 * x105;
	const FLT x271 = x270 + x264 + (-1 * x265 * x261) + (-1 * x262 * x105) + (x25 * x266) + x268;
	const FLT x272 = 2 * x110;
	const FLT x273 = x259 * x131;
	const FLT x274 = x22 * x273;
	const FLT x275 = x259 * x130;
	const FLT x276 = x22 * x275;
	const FLT x277 = x254 * x106;
	const FLT x278 = x277 * x120;
	const FLT x279 = x259 * x120;
	const FLT x280 = x28 * x279;
	const FLT x281 = x259 * x115;
	const FLT x282 = x28 * x281;
	const FLT x283 = x25 * x261;
	const FLT x284 = x27 * x118;
	const FLT x285 = x263 * x118;
	const FLT x286 = x19 * x106;
	const FLT x287 = -1 * x286 * x116;
	const FLT x288 = x28 * x261;
	const FLT x289 = x267 * x117;
	const FLT x290 = x289 + (-1 * x283 * x117) + (x288 * x116) + x285 + (-1 * x261 * x284) + x287;
	const FLT x291 = 2 * x112;
	const FLT x292 = x25 * x109;
	const FLT x293 = x267 * x109;
	const FLT x294 = x106 * x111;
	const FLT x295 = x18 * x294;
	const FLT x296 = -1 * x286 * x105;
	const FLT x297 = x296 + x295;
	const FLT x298 = x293 + (-1 * x27 * x266) + (-1 * x292 * x261) + x297 + (x288 * x105);
	const FLT x299 = x269 * x116;
	const FLT x300 = x27 * x117;
	const FLT x301 = -1 * x267 * x118;
	const FLT x302 = x263 * x117;
	const FLT x303 = x302 + x301;
	const FLT x304 = x303 + (-1 * x261 * x300) + x299 + (-1 * x262 * x116) + (x283 * x118);
	const FLT x305 = x277 * x115;
	const FLT x306 = 2 * x269;
	const FLT x307 = 2 * x131;
	const FLT x308 = (-1 * x269 * x307) + (x306 * x130);
	const FLT x309 = x308 + x305 + (x298 * x291) + (-1 * x291 * x290) + (-1 * x282 * x258) + (x280 * x258) +
					 (x274 * x258) + (-1 * x272 * x271) + (-1 * x276 * x258) + (x272 * x304) + (-1 * x278);
	const FLT x310 = 2 * x114;
	const FLT x311 = x27 * x105;
	const FLT x312 = x19 * x294;
	const FLT x313 = -1 * x269 * x109;
	const FLT x314 = x263 * x105;
	const FLT x315 = x314 + x313;
	const FLT x316 = x315 + x312 + (x262 * x109) + (-1 * x28 * x266) + (-1 * x261 * x311);
	const FLT x317 = x25 * x281;
	const FLT x318 = x259 * x113;
	const FLT x319 = x22 * x318;
	const FLT x320 = 2 * x113;
	const FLT x321 = x269 * x320;
	const FLT x322 = x25 * x279;
	const FLT x323 = x259 * x119;
	const FLT x324 = x22 * x323;
	const FLT x325 = x306 * x119;
	const FLT x326 = x27 * x116;
	const FLT x327 = x263 * x116;
	const FLT x328 = x286 * x118;
	const FLT x329 = -1 * x269 * x117;
	const FLT x330 = x329 + (-1 * x288 * x118) + x328 + (x262 * x117) + (-1 * x261 * x326) + x327;
	const FLT x331 = 2 * x267;
	const FLT x332 = (x331 * x120) + (-1 * x331 * x115);
	const FLT x333 = (-1 * x325) + (-1 * x258 * x322) + x321 + (x258 * x324) + (x272 * x316) + (-1 * x272 * x330) +
					 (x290 * x310) + x332 + (x258 * x317) + (-1 * x298 * x310) + (-1 * x258 * x319);
	const FLT x334 = x28 * x318;
	const FLT x335 = x331 * x130;
	const FLT x336 = x25 * x275;
	const FLT x337 = x28 * x323;
	const FLT x338 = x267 * x307;
	const FLT x339 = x25 * x273;
	const FLT x340 = (x277 * x119) + (-1 * x277 * x113);
	const FLT x341 = x340 + (-1 * x258 * x339) + x338 + (x291 * x330) + (-1 * x304 * x310) + (-1 * x258 * x337) +
					 (-1 * x335) + (x258 * x336) + (-1 * x291 * x316) + (x258 * x334) + (x271 * x310);
	const FLT x342 = (x341 * x133) + (x333 * x122);
	const FLT x343 = (-1 * x193 * (x342 + (x309 * x192))) + (x309 * x163);
	const FLT x344 = (x309 * x148) + (-1 * x342 * x141);
	const FLT x345 = ((x333 * x155) + (-1 * x341 * x154)) * x156;
	const FLT x346 = (-1 * x345) + (x344 * x153);
	const FLT x347 = x343 * x199;
	const FLT x348 = (x205 * x343) + (x164 * (x347 + (-1 * x204 * x343)));
	const FLT x349 = (x207 * x343) + (x348 * x164);
	const FLT x350 =
		x345 +
		(-1 * x220 *
		 (x344 + (x218 * x349) +
		  (-1 * x214 *
		   ((x212 * x346) +
			(-1 * x183 *
			 ((x209 * x343) + (x349 * x164) + (x210 * x343) +
			  (x164 * (x349 + (x343 * x197) +
					   (x164 * ((x343 * x198) + x348 + (x164 * ((-1 * x203 * x343) + (x201 * x343) + x347)))))))))) +
		  (x217 * x343) + (-1 * x346 * x191)));
	const FLT x351 = 2 * x21;
	const FLT x352 = (x22 * x351) + (x28 * x256) + (-1 * x24 * x257) + (-1 * x27 * x254);
	const FLT x353 = x22 * x352;
	const FLT x354 = 2 * x120;
	const FLT x355 = x263 * x354;
	const FLT x356 = x260 * x352;
	const FLT x357 = x25 * x356;
	const FLT x358 = x28 * x356;
	const FLT x359 = x329 + (-1 * x328);
	const FLT x360 = (x358 * x116) + (-1 * x327) + x359 + (-1 * x357 * x117) + (-1 * x284 * x356);
	const FLT x361 = x260 * x353;
	const FLT x362 = x269 * x118;
	const FLT x363 = x286 * x117;
	const FLT x364 = x267 * x116;
	const FLT x365 = x364 + (x357 * x118) + (-1 * x363) + (-1 * x361 * x116) + x362 + (-1 * x356 * x300);
	const FLT x366 = x260 * x105;
	const FLT x367 = x27 * x356;
	const FLT x368 = x267 * x105;
	const FLT x369 = x24 * x294;
	const FLT x370 = x286 * x109;
	const FLT x371 = (x357 * x111) + (-1 * x367 * x109) + (-1 * x366 * x353) + x368 + x369 + (-1 * x370);
	const FLT x372 = -1 * x312;
	const FLT x373 = x313 + (-1 * x292 * x356) + (-1 * x314) + (x358 * x105) + x372 + (-1 * x367 * x111);
	const FLT x374 = 2 * x115;
	const FLT x375 = x263 * x374;
	const FLT x376 = x375 + (-1 * x275 * x353) + (-1 * x338) + x335 + (-1 * x355) + (-1 * x291 * x360) +
					 (-1 * x282 * x352) + (x273 * x353) + (x272 * x365) + (x280 * x352) + (-1 * x272 * x371) +
					 (x291 * x373);
	const FLT x377 = -1 * x293;
	const FLT x378 = x297 + (-1 * x367 * x105) + (x361 * x109) + x377 + (-1 * x358 * x111);
	const FLT x379 = x287 + (-1 * x289);
	const FLT x380 = x379 + (x361 * x117) + x285 + (-1 * x356 * x326) + (-1 * x358 * x118);
	const FLT x381 = (x331 * x113) + (-1 * x331 * x119);
	const FLT x382 = (-1 * x269 * x354) + (x269 * x374);
	const FLT x383 = x381 + x382 + (x353 * x323) + (x360 * x310) + (x352 * x317) + (-1 * x373 * x310) +
					 (-1 * x352 * x322) + (x272 * x378) + (-1 * x353 * x318) + (-1 * x272 * x380);
	const FLT x384 = 2 * x263;
	const FLT x385 = x384 * x119;
	const FLT x386 = x263 * x320;
	const FLT x387 = (x371 * x310) + (-1 * x352 * x337) + x308 + (-1 * x386) + (x352 * x336) + (x291 * x380) +
					 (-1 * x291 * x378) + (x352 * x334) + (-1 * x365 * x310) + x385 + (-1 * x352 * x339);
	const FLT x388 = (x387 * x133) + (x383 * x122);
	const FLT x389 = (-1 * x193 * (x388 + (x376 * x192))) + (x376 * x163);
	const FLT x390 = (x376 * x148) + (-1 * x388 * x141);
	const FLT x391 = ((x383 * x155) + (-1 * x387 * x154)) * x156;
	const FLT x392 = (-1 * x391) + (x390 * x153);
	const FLT x393 = x389 * x199;
	const FLT x394 = (x205 * x389) + (x164 * (x393 + (-1 * x204 * x389)));
	const FLT x395 = (x207 * x389) + (x394 * x164);
	const FLT x396 =
		x391 + (-1 * x220 *
				((x218 * x395) +
				 (-1 * x214 *
				  ((x212 * x392) +
				   (-1 * x183 *
					((x395 * x164) + (x209 * x389) +
					 (x164 * (x395 + (x389 * x197) +
							  (x164 * (x394 + (x389 * x198) + (x164 * ((-1 * x203 * x389) + x393 + (x201 * x389))))))) +
					 (x210 * x389))))) +
				 (x217 * x389) + x390 + (-1 * x392 * x191)));
	const FLT x397 = 2 * x24;
	const FLT x398 = (x18 * x257) + (-1 * x22 * x254) + (x28 * x397) + (-1 * x27 * x351);
	const FLT x399 = x260 * x398;
	const FLT x400 = x399 * x105;
	const FLT x401 = x27 * x111;
	const FLT x402 = x268 + (-1 * x270);
	const FLT x403 = x402 + (x28 * x400) + (-1 * x292 * x399) + (-1 * x401 * x399) + x264;
	const FLT x404 = x281 * x398;
	const FLT x405 = x28 * x399;
	const FLT x406 = x22 * x109;
	const FLT x407 = (x406 * x399) + (-1 * x27 * x400) + x370 + (-1 * x405 * x111) + (-1 * x368) + x369;
	const FLT x408 = x22 * x399;
	const FLT x409 = x362 + (-1 * x399 * x326) + (-1 * x405 * x118) + (-1 * x364) + x363 + (x408 * x117);
	const FLT x410 = x25 * x399;
	const FLT x411 = -1 * x299;
	const FLT x412 = x411 + (-1 * x410 * x117) + x303 + (x405 * x116) + (-1 * x284 * x399);
	const FLT x413 = (-1 * x398 * x322) + x355 + x340 + (-1 * x375) + (x25 * x404) + (-1 * x403 * x310) +
					 (x398 * x324) + (x407 * x272) + (-1 * x398 * x319) + (x412 * x310) + (-1 * x409 * x272);
	const FLT x414 = x379 + (-1 * x399 * x300) + (-1 * x285) + (-1 * x408 * x116) + (x410 * x118);
	const FLT x415 = (x410 * x111) + (-1 * x295) + x296 + (-1 * x22 * x400) + (-1 * x265 * x399) + x377;
	const FLT x416 = x384 * x130;
	const FLT x417 = x263 * x307;
	const FLT x418 = x275 * x398;
	const FLT x419 = (-1 * x398 * x339) + (x415 * x310) + (-1 * x416) + (x398 * x334) + (-1 * x407 * x291) +
					 (-1 * x414 * x310) + x417 + (x25 * x418) + (-1 * x398 * x337) + x325 + (-1 * x321) + (x409 * x291);
	const FLT x420 = (x419 * x133) + (x413 * x122);
	const FLT x421 = (x277 * x131) + (-1 * x277 * x130);
	const FLT x422 = x421 + (x403 * x291) + (x414 * x272) + (x280 * x398) + (-1 * x412 * x291) + x382 +
					 (-1 * x28 * x404) + (-1 * x22 * x418) + (x274 * x398) + (-1 * x415 * x272);
	const FLT x423 = (x422 * x148) + (-1 * x420 * x141);
	const FLT x424 = ((x413 * x155) + (-1 * x419 * x154)) * x156;
	const FLT x425 = (-1 * x424) + (x423 * x153);
	const FLT x426 = (-1 * x193 * (x420 + (x422 * x192))) + (x422 * x163);
	const FLT x427 = x426 * x199;
	const FLT x428 = (x426 * x205) + (x164 * (x427 + (-1 * x426 * x204)));
	const FLT x429 = (x426 * x207) + (x428 * x164);
	const FLT x430 =
		x424 +
		(-1 * x220 *
		 (x423 + (x426 * x217) + (x429 * x218) + (-1 * x425 * x191) +
		  (-1 * x214 *
		   ((x425 * x212) +
			(-1 * x183 *
			 ((x429 * x164) + (x426 * x210) + (x426 * x209) +
			  (x164 * (x429 + (x426 * x197) +
					   (x164 * (x428 + (x426 * x198) + (x164 * ((-1 * x426 * x203) + x427 + (x426 * x201)))))))))))));
	const FLT x431 = (x19 * x257) + (-1 * x28 * x351) + (x18 * x255) + (-1 * x27 * x397);
	const FLT x432 = x431 * x260;
	const FLT x433 = x22 * x432;
	const FLT x434 = x25 * x432;
	const FLT x435 = x359 + (x434 * x118) + (-1 * x432 * x300) + x327 + (-1 * x433 * x116);
	const FLT x436 = x315 + (x434 * x111) + (-1 * x432 * x265) + x372 + (-1 * x433 * x105);
	const FLT x437 = x28 * x431;
	const FLT x438 = x437 * x260;
	const FLT x439 = x364 + (-1 * x432 * x284) + (-1 * x434 * x117) + (x438 * x116) + x363 + (-1 * x362);
	const FLT x440 = (x437 * x366) + (-1 * x401 * x432) + (-1 * x432 * x292) + x368 + (-1 * x369) + x370;
	const FLT x441 = (x440 * x291) + (-1 * x437 * x281) + (x437 * x279) + (-1 * x439 * x291) + (-1 * x431 * x276) +
					 (x431 * x274) + (-1 * x436 * x272) + x416 + (x435 * x272) + x332 + (-1 * x417);
	const FLT x442 = x402 + (-1 * x264) + (x406 * x432) + (-1 * x438 * x111) + (-1 * x432 * x311);
	const FLT x443 = x301 + (-1 * x432 * x326) + x411 + (x433 * x117) + (-1 * x438 * x118) + (-1 * x302);
	const FLT x444 = x278 + (-1 * x443 * x272) + (-1 * x431 * x319) + x386 + (-1 * x305) + (-1 * x440 * x310) +
					 (x431 * x324) + (x439 * x310) + (x442 * x272) + (x431 * x317) + (-1 * x385) + (-1 * x431 * x322);
	const FLT x445 = x421 + (x437 * x318) + (x436 * x310) + (-1 * x431 * x339) + (x431 * x336) + (-1 * x437 * x323) +
					 (x443 * x291) + (-1 * x435 * x310) + x381 + (-1 * x442 * x291);
	const FLT x446 = (x445 * x133) + (x444 * x122);
	const FLT x447 = (-1 * x193 * (x446 + (x441 * x192))) + (x441 * x163);
	const FLT x448 = x447 * x199;
	const FLT x449 = (x447 * x205) + (x164 * (x448 + (-1 * x447 * x204)));
	const FLT x450 = (x447 * x207) + (x449 * x164);
	const FLT x451 = (x441 * x148) + (-1 * x446 * x141);
	const FLT x452 = ((x444 * x155) + (-1 * x445 * x154)) * x156;
	const FLT x453 = (-1 * x452) + (x451 * x153);
	const FLT x454 =
		x452 + (-1 * x220 *
				(x451 + (-1 * x453 * x191) + (x450 * x218) +
				 (-1 * x214 *
				  ((x453 * x212) +
				   (-1 * x183 *
					((x450 * x164) +
					 (x164 * (x450 + (x447 * x197) +
							  (x164 * (x449 + (x447 * x198) + (x164 * (x448 + (-1 * x447 * x203) + (x447 * x201))))))) +
					 (x447 * x209) + (x447 * x210))))) +
				 (x447 * x217)));
	const FLT x455 = x31 * x103;
	const FLT x456 = -1 * x23 * x455;
	const FLT x457 = (-1 * x26 * x455) + x104;
	const FLT x458 = x457 + x456;
	const FLT x459 = x25 * x455;
	const FLT x460 = x27 * x459;
	const FLT x461 = x455 * x123;
	const FLT x462 = x461 + x460;
	const FLT x463 = (x462 * x133) + (x458 * x122);
	const FLT x464 = x28 * x459;
	const FLT x465 = x22 * x455;
	const FLT x466 = x27 * x465;
	const FLT x467 = (-1 * x466) + x464;
	const FLT x468 = (x467 * x148) + (-1 * x463 * x141);
	const FLT x469 = ((x458 * x155) + (-1 * x462 * x154)) * x156;
	const FLT x470 = (-1 * x469) + (x468 * x153);
	const FLT x471 = (-1 * x193 * (x463 + (x467 * x192))) + (x467 * x163);
	const FLT x472 = x471 * x199;
	const FLT x473 = (x471 * x205) + (x164 * (x472 + (-1 * x471 * x204)));
	const FLT x474 = (x471 * x207) + (x473 * x164);
	const FLT x475 =
		x469 + (-1 * x220 *
				((x474 * x218) + x468 + (x471 * x217) + (-1 * x470 * x191) +
				 (-1 * x214 *
				  ((x470 * x212) +
				   (-1 * x183 *
					((x474 * x164) + (x471 * x209) +
					 (x164 * (x474 + (x471 * x197) +
							  (x164 * (x473 + (x471 * x198) + (x164 * ((-1 * x471 * x203) + (x471 * x201) + x472)))))) +
					 (x471 * x210)))))));
	const FLT x476 = x464 + x466;
	const FLT x477 = x25 * x465;
	const FLT x478 = x28 * x27 * x455;
	const FLT x479 = (-1 * x478) + x477;
	const FLT x480 = (x479 * x133) + (x476 * x122);
	const FLT x481 = -1 * x29 * x455;
	const FLT x482 = x104 + x481 + x456;
	const FLT x483 = (x482 * x148) + (-1 * x480 * x141);
	const FLT x484 = ((x476 * x155) + (-1 * x479 * x154)) * x156;
	const FLT x485 = (-1 * x484) + (x483 * x153);
	const FLT x486 = (-1 * x193 * (x480 + (x482 * x192))) + (x482 * x163);
	const FLT x487 = x486 * x199;
	const FLT x488 = (x486 * x205) + (x164 * (x487 + (-1 * x486 * x204)));
	const FLT x489 = (x486 * x207) + (x488 * x164);
	const FLT x490 =
		x484 + (-1 * x220 *
				(x483 + (x489 * x218) + (x486 * x217) + (-1 * x485 * x191) +
				 (-1 * x214 *
				  ((x485 * x212) +
				   (-1 * x183 *
					((x489 * x164) + (x486 * x209) +
					 (x164 * (x489 + (x486 * x197) +
							  (x164 * (x488 + (x486 * x198) + (x164 * ((-1 * x486 * x203) + x487 + (x486 * x201))))))) +
					 (x486 * x210)))))));
	const FLT x491 = x477 + x478;
	const FLT x492 = (-1 * x460) + x461;
	const FLT x493 = x457 + x481;
	const FLT x494 = (x493 * x133) + (x492 * x122);
	const FLT x495 = (-1 * x193 * (x494 + (x491 * x192))) + (x491 * x163);
	const FLT x496 = x495 * x199;
	const FLT x497 = (x495 * x205) + (x164 * (x496 + (-1 * x495 * x204)));
	const FLT x498 = (x495 * x207) + (x497 * x164);
	const FLT x499 = (x491 * x148) + (-1 * x494 * x141);
	const FLT x500 = ((x492 * x155) + (-1 * x493 * x154)) * x156;
	const FLT x501 = (-1 * x500) + (x499 * x153);
	const FLT x502 =
		x500 +
		(-1 * x220 *
		 (x499 + (x498 * x218) +
		  (-1 * x214 *
		   ((x501 * x212) +
			(-1 * x183 *
			 ((x498 * x164) + (x495 * x209) + (x495 * x210) +
			  (x164 * (x498 + (x495 * x197) +
					   (x164 * (x497 + (x495 * x198) + (x164 * ((-1 * x495 * x203) + x496 + (x495 * x201))))))))))) +
		  (-1 * x501 * x191) + (x495 * x217)));
	const FLT x503 = x226 + x144;
	const FLT x504 = -1 * x33;
	const FLT x505 = -1 * x34;
	const FLT x506 = 1 + x505 + x504;
	const FLT x507 = x124 + x127;
	const FLT x508 = (x507 * x133) + (x506 * x122);
	const FLT x509 = (-1 * x193 * (x508 + (x503 * x192))) + (x503 * x163);
	const FLT x510 = x509 * x199;
	const FLT x511 = (x509 * x205) + (x164 * (x510 + (-1 * x509 * x204)));
	const FLT x512 = (x509 * x207) + (x511 * x164);
	const FLT x513 = (x503 * x148) + (-1 * x508 * x141);
	const FLT x514 = ((x506 * x155) + (-1 * x507 * x154)) * x156;
	const FLT x515 = (-1 * x514) + (x513 * x153);
	const FLT x516 =
		x514 + (-1 * x220 *
				(x513 + (x509 * x217) +
				 (-1 * x214 *
				  ((x515 * x212) +
				   (-1 * x183 *
					((x512 * x164) + (x509 * x210) +
					 (x164 * (x512 + (x509 * x197) +
							  (x164 * ((x509 * x198) + x511 + (x164 * ((-1 * x509 * x203) + x510 + (x509 * x201))))))) +
					 (x509 * x209))))) +
				 (x512 * x218) + (-1 * x515 * x191)));
	const FLT x517 = x144 + x143;
	const FLT x518 = x241 + x229;
	const FLT x519 = (x518 * x133) + (x517 * x122);
	const FLT x520 = 1 + (-1 * x224);
	const FLT x521 = x520 + x504;
	const FLT x522 = (x521 * x148) + (-1 * x519 * x141);
	const FLT x523 = ((x517 * x155) + (-1 * x518 * x154)) * x156;
	const FLT x524 = x190 * ((-1 * x523) + (x522 * x153));
	const FLT x525 = (-1 * x193 * (x519 + (x521 * x192))) + (x521 * x163);
	const FLT x526 = x525 * x199;
	const FLT x527 = (x525 * x205) + (x164 * (x526 + (-1 * x525 * x204)));
	const FLT x528 = (x525 * x207) + (x527 * x164);
	const FLT x529 =
		x523 + (-1 * x220 *
				((x525 * x217) + x522 + (x528 * x218) + (-1 * x524 * x189) +
				 (-1 * x214 *
				  ((x524 * x211) +
				   (-1 * x183 *
					((x528 * x164) + (x525 * x210) +
					 (x164 * (x528 + (x525 * x197) +
							  (x164 * (x527 + (x525 * x198) + (x164 * ((-1 * x525 * x203) + x526 + (x525 * x201))))))) +
					 (x525 * x209)))))));
	const FLT x530 = x229 + x228;
	const FLT x531 = x128 + x124;
	const FLT x532 = x520 + x505;
	const FLT x533 = (x532 * x133) + (x531 * x122);
	const FLT x534 = (-1 * x193 * (x533 + (x530 * x192))) + (x530 * x163);
	const FLT x535 = (x530 * x148) + (-1 * x533 * x141);
	const FLT x536 = ((x531 * x155) + (-1 * x532 * x154)) * x156;
	const FLT x537 = (-1 * x536) + (x535 * x153);
	const FLT x538 = x534 * x199;
	const FLT x539 = (x534 * x205) + (x164 * (x538 + (-1 * x534 * x204)));
	const FLT x540 = (x534 * x207) + (x539 * x164);
	const FLT x541 =
		x536 + (-1 * x220 *
				((-1 * x214 *
				  ((x537 * x212) +
				   (-1 * x183 *
					((x540 * x164) +
					 (x164 * (x540 + (x534 * x197) +
							  (x164 * (x539 + (x534 * x198) + (x164 * ((-1 * x534 * x203) + (x534 * x201) + x538)))))) +
					 (x534 * x210) + (x534 * x209))))) +
				 x535 + (x540 * x218) + (x534 * x217) + (-1 * x537 * x191)));
	const FLT x542 = x82 * x84;
	const FLT x543 = x64 * x542;
	const FLT x544 = x89 * x57;
	const FLT x545 = x82 * x59;
	const FLT x546 = -1 * x37 * x545;
	const FLT x547 = x82 * x58;
	const FLT x548 = x67 * x547;
	const FLT x549 = x548 + x546;
	const FLT x550 = x549 + x543 + x544;
	const FLT x551 = x82 * x57;
	const FLT x552 = x64 * x551;
	const FLT x553 = -1 * x552;
	const FLT x554 = x67 * x545;
	const FLT x555 = -1 * x554;
	const FLT x556 = x37 * x547;
	const FLT x557 = x89 * x84;
	const FLT x558 = x557 + (-1 * x556);
	const FLT x559 = x558 + x553 + x555;
	const FLT x560 = x64 * x545;
	const FLT x561 = x89 * x58;
	const FLT x562 = -1 * x67 * x551;
	const FLT x563 = x37 * x542;
	const FLT x564 = x563 + x562;
	const FLT x565 = x564 + x560 + x561;
	const FLT x566 = (x565 * sensor_pt[0]) + (-1 * x550 * sensor_pt[2]) + (x559 * sensor_pt[1]);
	const FLT x567 = 2 * x101;
	const FLT x568 = 2 * x108;
	const FLT x569 = x67 * x542;
	const FLT x570 = x89 * x59;
	const FLT x571 = -1 * x64 * x547;
	const FLT x572 = x37 * x551;
	const FLT x573 = x572 + x571;
	const FLT x574 = x573 + x569 + x570;
	const FLT x575 = (x550 * sensor_pt[1]) + (-1 * x574 * sensor_pt[0]) + (x559 * sensor_pt[2]);
	const FLT x576 = 2 * x98;
	const FLT x577 = 2 * x99;
	const FLT x578 = (x574 * x577) + (x576 * x575) + (-1 * x567 * x566) + (-1 * x568 * x565);
	const FLT x579 = 2 * x96;
	const FLT x580 = (x574 * sensor_pt[2]) + (-1 * x565 * sensor_pt[1]) + (x559 * sensor_pt[0]);
	const FLT x581 = 2 * x102;
	const FLT x582 = (x581 * x565) + (x580 * x567) + (-1 * x579 * x575) + (-1 * x577 * x550);
	const FLT x583 = (x579 * x566) + (-1 * x576 * x580) + (-1 * x574 * x581) + (x550 * x568);
	const FLT x584 = (x583 * x107) + (x578 * x114) + (-1 * x582 * x112);
	const FLT x585 = (-1 * x583 * x114) + (x582 * x110) + (x578 * x107);
	const FLT x586 = x582 + (x584 * x291) + (-1 * x585 * x272);
	const FLT x587 = (-1 * x578 * x110) + (x583 * x112) + (x582 * x107);
	const FLT x588 = x578 + (-1 * x584 * x310) + (x587 * x272);
	const FLT x589 = (x585 * x310) + x583 + (-1 * x587 * x291);
	const FLT x590 = (x589 * x133) + (x588 * x122);
	const FLT x591 = (-1 * x193 * (x590 + (x586 * x192))) + (x586 * x163);
	const FLT x592 = x591 * x199;
	const FLT x593 = (x591 * x205) + (x164 * (x592 + (-1 * x591 * x204)));
	const FLT x594 = (x591 * x207) + (x593 * x164);
	const FLT x595 = (x586 * x148) + (-1 * x590 * x141);
	const FLT x596 = ((x588 * x155) + (-1 * x589 * x154)) * x156;
	const FLT x597 = (-1 * x596) + (x595 * x153);
	const FLT x598 =
		x596 + (-1 * x220 *
				((x594 * x218) + (-1 * x597 * x191) +
				 (-1 * x214 *
				  ((x597 * x212) +
				   (-1 * x183 *
					((x594 * x164) +
					 (x164 * (x594 + (x591 * x197) +
							  (x164 * (x593 + (x591 * x198) + (x164 * ((-1 * x591 * x203) + x592 + (x591 * x201))))))) +
					 (x591 * x209) + (x591 * x210))))) +
				 x595 + (x591 * x217)));
	const FLT x599 = -1 * x569;
	const FLT x600 = (-1 * x572) + x571;
	const FLT x601 = x600 + x599 + x570;
	const FLT x602 = x556 + x557;
	const FLT x603 = x602 + x554 + x553;
	const FLT x604 = -1 * x543;
	const FLT x605 = -1 * x544;
	const FLT x606 = x549 + x604 + x605;
	const FLT x607 = (x601 * sensor_pt[0]) + (-1 * x603 * sensor_pt[2]) + (x606 * sensor_pt[1]);
	const FLT x608 = -1 * x560;
	const FLT x609 = -1 * x561;
	const FLT x610 = x564 + x608 + x609;
	const FLT x611 = (-1 * x610 * sensor_pt[0]) + (x603 * sensor_pt[1]) + (x606 * sensor_pt[2]);
	const FLT x612 = (x611 * x576) + (x610 * x577) + (-1 * x601 * x568) + (-1 * x607 * x567);
	const FLT x613 = (x610 * sensor_pt[2]) + (-1 * x601 * sensor_pt[1]) + (x606 * sensor_pt[0]);
	const FLT x614 = (x601 * x581) + (-1 * x611 * x579) + (x613 * x567) + (-1 * x603 * x577);
	const FLT x615 = (-1 * x613 * x576) + (x603 * x568) + (-1 * x610 * x581) + (x607 * x579);
	const FLT x616 = (x615 * x107) + (x612 * x114) + (-1 * x614 * x112);
	const FLT x617 = (-1 * x615 * x114) + (x614 * x110) + (x612 * x107);
	const FLT x618 = x614 + (x616 * x291) + (-1 * x617 * x272);
	const FLT x619 = (-1 * x612 * x110) + (x615 * x112) + (x614 * x107);
	const FLT x620 = x612 + (x619 * x272) + (-1 * x616 * x310);
	const FLT x621 = x615 + (x617 * x310) + (-1 * x619 * x291);
	const FLT x622 = (x621 * x133) + (x620 * x122);
	const FLT x623 = (-1 * x193 * (x622 + (x618 * x192))) + (x618 * x163);
	const FLT x624 = (x618 * x148) + (-1 * x622 * x141);
	const FLT x625 = ((x620 * x155) + (-1 * x621 * x154)) * x156;
	const FLT x626 = (-1 * x625) + (x624 * x153);
	const FLT x627 = x623 * x199;
	const FLT x628 = (x623 * x205) + (x164 * (x627 + (-1 * x623 * x204)));
	const FLT x629 = (x623 * x207) + (x628 * x164);
	const FLT x630 =
		x625 + (-1 * x220 *
				(x624 +
				 (-1 * x214 *
				  ((x626 * x212) +
				   (-1 * x183 *
					((x164 * ((x623 * x197) + x629 +
							  (x164 * (x628 + (x623 * x198) + (x164 * ((-1 * x623 * x203) + x627 + (x623 * x201))))))) +
					 (x623 * x210) + (x629 * x164) + (x623 * x209))))) +
				 (x629 * x218) + (x623 * x217) + (-1 * x626 * x191)));
	const FLT x631 = (-1 * x563) + x562;
	const FLT x632 = x631 + x561 + x608;
	const FLT x633 = -1 * x570;
	const FLT x634 = x573 + x633 + x599;
	const FLT x635 = (-1 * x548) + x546;
	const FLT x636 = x635 + x605 + x543;
	const FLT x637 = (x636 * sensor_pt[0]) + (-1 * x632 * sensor_pt[2]) + (x634 * sensor_pt[1]);
	const FLT x638 = x602 + x552 + x555;
	const FLT x639 = (x632 * sensor_pt[1]) + (-1 * x638 * sensor_pt[0]) + (x634 * sensor_pt[2]);
	const FLT x640 = (x638 * x577) + (-1 * x637 * x567) + (x639 * x576) + (-1 * x636 * x568);
	const FLT x641 = (x638 * sensor_pt[2]) + (-1 * x636 * sensor_pt[1]) + (x634 * sensor_pt[0]);
	const FLT x642 = (x636 * x581) + (-1 * x639 * x579) + (-1 * x632 * x577) + (x641 * x567);
	const FLT x643 = (x632 * x568) + (-1 * x638 * x581) + (x637 * x579) + (-1 * x641 * x576);
	const FLT x644 = (x643 * x107) + (x640 * x114) + (-1 * x642 * x112);
	const FLT x645 = (-1 * x643 * x114) + (x640 * x107) + (x642 * x110);
	const FLT x646 = (x644 * x291) + x642 + (-1 * x645 * x272);
	const FLT x647 = 2 * ((-1 * x640 * x110) + (x642 * x107) + (x643 * x112));
	const FLT x648 = x640 + (x647 * x110) + (-1 * x644 * x310);
	const FLT x649 = x643 + (x645 * x310) + (-1 * x647 * x112);
	const FLT x650 = (x649 * x133) + (x648 * x122);
	const FLT x651 = (-1 * x193 * (x650 + (x646 * x192))) + (x646 * x163);
	const FLT x652 = x651 * x196;
	const FLT x653 = (x646 * x148) + (-1 * x650 * x141);
	const FLT x654 = ((x648 * x155) + (-1 * x649 * x154)) * x156;
	const FLT x655 = (-1 * x654) + (x653 * x153);
	const FLT x656 = x651 * x199;
	const FLT x657 = (x651 * x205) + (x164 * (x656 + (-1 * x652 * x165)));
	const FLT x658 = (x651 * x207) + (x657 * x164);
	const FLT x659 =
		x654 +
		(-1 * x220 *
		 (x653 +
		  (-1 * x214 *
		   ((x655 * x212) +
			(-1 * x183 *
			 ((x658 * x164) + (x651 * x209) + (x651 * x210) +
			  (x164 * (x658 + (x651 * x197) +
					   (x164 * ((x651 * x198) + x657 + (x164 * ((-1 * x652 * x202) + x656 + (x651 * x201))))))))))) +
		  (x658 * x218) + (x652 * x216 * x170 * x186) + (-1 * x655 * x191)));
	const FLT x660 = x631 + x560 + x609;
	const FLT x661 = x600 + x633 + x569;
	const FLT x662 = x558 + x552 + x554;
	const FLT x663 = (x660 * sensor_pt[1]) + (x662 * sensor_pt[0]) + (-1 * x661 * sensor_pt[2]);
	const FLT x664 = x635 + x604 + x544;
	const FLT x665 = (x661 * sensor_pt[1]) + (-1 * x664 * sensor_pt[0]) + (x660 * sensor_pt[2]);
	const FLT x666 = (-1 * x662 * x568) + (-1 * x663 * x567) + (x664 * x577) + (x665 * x576);
	const FLT x667 = (x664 * sensor_pt[2]) + (-1 * x662 * sensor_pt[1]) + (x660 * sensor_pt[0]);
	const FLT x668 = (x662 * x581) + (x667 * x567) + (-1 * x665 * x579) + (-1 * x661 * x577);
	const FLT x669 = (-1 * x664 * x581) + (-1 * x667 * x576) + (x661 * x568) + (x663 * x579);
	const FLT x670 = (x669 * x107) + (x666 * x114) + (-1 * x668 * x112);
	const FLT x671 = (-1 * x669 * x114) + (x668 * x110) + (x666 * x107);
	const FLT x672 = x668 + (x670 * x291) + (-1 * x671 * x272);
	const FLT x673 = (-1 * x666 * x110) + (x669 * x112) + (x668 * x107);
	const FLT x674 = x666 + (x673 * x272) + (-1 * x670 * x310);
	const FLT x675 = x669 + (x671 * x310) + (-1 * x673 * x291);
	const FLT x676 = (x675 * x133) + (x674 * x122);
	const FLT x677 = (-1 * x193 * (x676 + (x672 * x192))) + (x672 * x163);
	const FLT x678 = x677 * x199;
	const FLT x679 = (x677 * x205) + (x164 * (x678 + (-1 * x677 * x204)));
	const FLT x680 = (x677 * x207) + (x679 * x164);
	const FLT x681 = (x672 * x148) + (-1 * x676 * x141);
	const FLT x682 = ((x674 * x155) + (-1 * x675 * x154)) * x156;
	const FLT x683 = (-1 * x682) + (x681 * x153);
	const FLT x684 =
		x682 + (-1 * x220 *
				(x681 + (-1 * x683 * x191) + (x677 * x217) + (x680 * x218) +
				 (-1 * x214 *
				  ((x683 * x212) +
				   (-1 * x183 *
					((x680 * x164) + (x677 * x209) +
					 (x164 * ((x677 * x197) + x680 +
							  (x164 * (x679 + (x677 * x198) + (x164 * ((-1 * x677 * x203) + x678 + (x677 * x201))))))) +
					 (x677 * x210)))))));
	const FLT x685 = -1 * x83;
	const FLT x686 = x79 * x91;
	const FLT x687 = dt * dt * dt;
	const FLT x688 = (1. / (x70 * sqrt(x70))) * x73;
	const FLT x689 = x688 * x687;
	const FLT x690 = x64 * x689;
	const FLT x691 = x37 * x690;
	const FLT x692 = x691 * x686;
	const FLT x693 = dt * dt * dt * dt;
	const FLT x694 = (x64 * x64 * x64) * x693;
	const FLT x695 = 2 * (1. / (x70 * x70)) * x74;
	const FLT x696 = 1.0 * x77;
	const FLT x697 = x696 * x688;
	const FLT x698 = x697 * x693;
	const FLT x699 = x62 * x698;
	const FLT x700 = x695 * x693;
	const FLT x701 = x64 * x68;
	const FLT x702 = x62 * x700;
	const FLT x703 = 2 * x76;
	const FLT x704 = x61 * x703;
	const FLT x705 = x80 * x696;
	const FLT x706 = x61 * x705;
	const FLT x707 = (-1 * x64 * x706) + (x697 * x694) + (x698 * x701) + (-1 * x700 * x701) + (x64 * x699) +
					 (x64 * x704) + (-1 * x694 * x695) + (-1 * x64 * x702);
	const FLT x708 = 1.0 / 2.0 * (1. / (x78 * sqrt(x78)));
	const FLT x709 = x77 * x708;
	const FLT x710 = x707 * x709;
	const FLT x711 = 0.5 * x80;
	const FLT x712 = x61 * x711;
	const FLT x713 = x86 * x712;
	const FLT x714 = x81 * x708;
	const FLT x715 = x60 * x714;
	const FLT x716 = x64 * x707;
	const FLT x717 = 0.5 * x75 * x687;
	const FLT x718 = x94 * x717;
	const FLT x719 = x64 * x718;
	const FLT x720 = x37 * x719;
	const FLT x721 = x91 * x714;
	const FLT x722 = x707 * x721;
	const FLT x723 = x65 * x689;
	const FLT x724 = x79 * x60;
	const FLT x725 = x88 * x714;
	const FLT x726 = x67 * x707;
	const FLT x727 = x717 * x100;
	const FLT x728 = x90 * x717;
	const FLT x729 = x64 * x728;
	const FLT x730 = x88 * x79;
	const FLT x731 = x690 * x730;
	const FLT x732 = (-1 * x67 * x731) + (x67 * x729);
	const FLT x733 = x732 + (-1 * x65 * x727) + (x724 * x723) + (-1 * x37 * x722) + (-1 * x726 * x725) + x720 +
					 (-1 * x692) + x685 + (-1 * x85 * x710) + (-1 * x64 * x713) + (x715 * x716);
	const FLT x734 = -1 * x92;
	const FLT x735 = x37 * x707;
	const FLT x736 = x67 * x690;
	const FLT x737 = x86 * x736;
	const FLT x738 = x85 * x714;
	const FLT x739 = x97 * x717;
	const FLT x740 = x64 * x739;
	const FLT x741 = x67 * x740;
	const FLT x742 = x64 * x712;
	const FLT x743 = x64 * x727;
	const FLT x744 = x64 * x724;
	const FLT x745 = (x37 * x689 * x744) + (-1 * x37 * x743);
	const FLT x746 = (x686 * x723) + x745 + (-1 * x88 * x710) + (x716 * x721) + (-1 * x730 * x742) + (-1 * x741) +
					 (-1 * x65 * x718) + x734 + (x738 * x726) + (x715 * x735) + x737;
	const FLT x747 = x67 * x743;
	const FLT x748 = x86 * x691;
	const FLT x749 = x67 * x715;
	const FLT x750 = x67 * x689;
	const FLT x751 = x744 * x750;
	const FLT x752 = x37 * x740;
	const FLT x753 = (-1 * x751) + (-1 * x91 * x710) + (-1 * x716 * x725) + (-1 * x707 * x749) + (x65 * x728) +
					 (-1 * x752) + x747 + (-1 * x730 * x723) + (x735 * x738) + x748 + x95 + (-1 * x686 * x742);
	const FLT x754 = (x753 * sensor_pt[1]) + (-1 * x733 * sensor_pt[0]) + (x746 * sensor_pt[2]);
	const FLT x755 = (x686 * x736) + (-1 * x67 * x719);
	const FLT x756 = (-1 * x37 * x731) + (x37 * x729);
	const FLT x757 = x755 + (-1 * x86 * x723) + (x67 * x722) + (-1 * x60 * x710) + x756 + x87 + (-1 * x716 * x738) +
					 (-1 * x735 * x725) + (x65 * x739) + (-1 * x712 * x744);
	const FLT x758 = (x733 * sensor_pt[2]) + (-1 * x757 * sensor_pt[1]) + (x746 * sensor_pt[0]);
	const FLT x759 = (-1 * x753 * x577) + (x757 * x581) + (-1 * x754 * x579) + (x758 * x567);
	const FLT x760 = (-1 * x753 * sensor_pt[2]) + (x757 * sensor_pt[0]) + (x746 * sensor_pt[1]);
	const FLT x761 = (x733 * x577) + (x754 * x576) + (-1 * x760 * x567) + (-1 * x757 * x568);
	const FLT x762 = (-1 * x758 * x576) + (-1 * x733 * x581) + (x760 * x579) + (x753 * x568);
	const FLT x763 = x762 * x106;
	const FLT x764 = (x759 * x110) + (-1 * x25 * x763) + (x761 * x107);
	const FLT x765 = (x762 * x107) + (x761 * x114) + (-1 * x759 * x112);
	const FLT x766 = (-1 * x764 * x272) + x759 + (x765 * x291);
	const FLT x767 = 2 * ((-1 * x761 * x110) + (x28 * x763) + (x759 * x107));
	const FLT x768 = x761 + (x767 * x110) + (-1 * x765 * x310);
	const FLT x769 = x762 + (-1 * x767 * x112) + (x764 * x310);
	const FLT x770 = (x769 * x133) + (x768 * x122);
	const FLT x771 = (-1 * x193 * (x770 + (x766 * x192))) + (x766 * x163);
	const FLT x772 = (x766 * x148) + (-1 * x770 * x141);
	const FLT x773 = ((x768 * x155) + (-1 * x769 * x154)) * x156;
	const FLT x774 = (-1 * x773) + (x772 * x153);
	const FLT x775 = x771 * x199;
	const FLT x776 = (x771 * x205) + (x164 * (x775 + (-1 * x771 * x204)));
	const FLT x777 = (x771 * x207) + (x776 * x164);
	const FLT x778 =
		x773 + (-1 * x220 *
				((-1 * x214 *
				  ((x774 * x212) +
				   (-1 * x183 *
					((x777 * x164) + (x771 * x209) +
					 (x164 * (x777 + (x771 * x197) +
							  (x164 * (x776 + (x771 * x198) + (x164 * ((-1 * x771 * x203) + (x771 * x201) + x775)))))) +
					 (x771 * x210))))) +
				 x772 + (x777 * x218) + (x771 * x217) + (-1 * x774 * x191)));
	const FLT x779 = x67 * x65;
	const FLT x780 = x61 * x67;
	const FLT x781 = x67 * x67 * x67;
	const FLT x782 = (x703 * x780) + (x698 * x781) + (x698 * x779) + (-1 * x779 * x700) + (-1 * x700 * x781) +
					 (-1 * x705 * x780) + (x67 * x699) + (-1 * x67 * x702);
	const FLT x783 = x709 * x782;
	const FLT x784 = x68 * x689;
	const FLT x785 = x67 * x37;
	const FLT x786 = x718 * x785;
	const FLT x787 = x67 * x782;
	const FLT x788 = x37 * x750;
	const FLT x789 = x686 * x788;
	const FLT x790 = x721 * x782;
	const FLT x791 = x711 * x780;
	const FLT x792 = x64 * x782;
	const FLT x793 = (x715 * x792) + x95 + (-1 * x730 * x784) + (-1 * x85 * x783) + (-1 * x747) + x786 + (-1 * x789) +
					 (-1 * x725 * x787) + (x68 * x728) + x751 + (-1 * x37 * x790) + (-1 * x86 * x791);
	const FLT x794 = x727 * x785;
	const FLT x795 = x724 * x788;
	const FLT x796 = x37 * x782;
	const FLT x797 = -1 * x87;
	const FLT x798 = (-1 * x730 * x791) + x797 + (x715 * x796) + (-1 * x88 * x783) + (-1 * x68 * x739) + (x64 * x790) +
					 (-1 * x794) + x755 + (x738 * x787) + x795 + (x86 * x784);
	const FLT x799 = (x86 * x788) + (-1 * x739 * x785);
	const FLT x800 = x799 + x732 + (-1 * x686 * x791) + (-1 * x782 * x749) + (x738 * x796) + (-1 * x725 * x792) +
					 (x68 * x727) + (-1 * x724 * x784) + (-1 * x91 * x783) + x83;
	const FLT x801 = (x800 * sensor_pt[1]) + (-1 * x793 * sensor_pt[0]) + (x798 * sensor_pt[2]);
	const FLT x802 = (x728 * x785) + (-1 * x730 * x788);
	const FLT x803 = x802 + (x686 * x784) + (-1 * x68 * x718) + x734 + (-1 * x724 * x791) + (-1 * x738 * x792) +
					 (x67 * x790) + x741 + (-1 * x725 * x796) + (-1 * x60 * x783) + (-1 * x737);
	const FLT x804 = (x793 * sensor_pt[2]) + (-1 * x803 * sensor_pt[1]) + (x798 * sensor_pt[0]);
	const FLT x805 = (x581 * x803) + (x567 * x804) + (-1 * x579 * x801) + (-1 * x577 * x800);
	const FLT x806 = x805 * x106;
	const FLT x807 = (-1 * x800 * sensor_pt[2]) + (x803 * sensor_pt[0]) + (x798 * sensor_pt[1]);
	const FLT x808 = (x793 * x577) + (x576 * x801) + (-1 * x567 * x807) + (-1 * x568 * x803);
	const FLT x809 = (x579 * x807) + (x568 * x800) + (-1 * x793 * x581) + (-1 * x576 * x804);
	const FLT x810 = (x809 * x107) + (-1 * x28 * x806) + (x808 * x114);
	const FLT x811 = (-1 * x809 * x114) + (x22 * x806) + (x808 * x107);
	const FLT x812 = (x810 * x291) + x805 + (-1 * x811 * x272);
	const FLT x813 = (-1 * x808 * x110) + (x809 * x112) + (x805 * x107);
	const FLT x814 = x808 + (x813 * x272) + (-1 * x810 * x310);
	const FLT x815 = (x811 * x310) + x809 + (-1 * x813 * x291);
	const FLT x816 = (x815 * x133) + (x814 * x122);
	const FLT x817 = (-1 * x193 * (x816 + (x812 * x192))) + (x812 * x163);
	const FLT x818 = (x812 * x148) + (-1 * x816 * x141);
	const FLT x819 = ((x814 * x155) + (-1 * x815 * x154)) * x156;
	const FLT x820 = (-1 * x819) + (x818 * x153);
	const FLT x821 = x817 * x199;
	const FLT x822 = (x817 * x205) + (x164 * (x821 + (-1 * x817 * x204)));
	const FLT x823 = (x817 * x207) + (x822 * x164);
	const FLT x824 =
		x819 + (-1 * x220 *
				(x818 + (x823 * x218) + (x817 * x217) +
				 (-1 * x214 *
				  ((x820 * x212) +
				   (-1 * x183 *
					((x817 * x209) + (x823 * x164) +
					 (x164 * ((x817 * x197) + x823 +
							  (x164 * (x822 + (x817 * x198) + (x164 * ((-1 * x817 * x203) + x821 + (x817 * x201))))))) +
					 (x817 * x210))))) +
				 (-1 * x820 * x191)));
	const FLT x825 = x37 * x698;
	const FLT x826 = x37 * x700;
	const FLT x827 = x37 * x37 * x37;
	const FLT x828 = (x698 * x827) + (-1 * x700 * x827) + (-1 * x65 * x826) + (x65 * x825) + (-1 * x37 * x706) +
					 (x68 * x825) + (x37 * x704) + (-1 * x68 * x826);
	const FLT x829 = x37 * x828;
	const FLT x830 = x37 * x712;
	const FLT x831 = x62 * x689;
	const FLT x832 = x67 * x828;
	const FLT x833 = x709 * x828;
	const FLT x834 = x64 * x828;
	const FLT x835 = (-1 * x88 * x833) + (-1 * x62 * x727) + x799 + (x738 * x832) + (x721 * x834) + (-1 * x720) + x692 +
					 x685 + (x715 * x829) + (x724 * x831) + (-1 * x730 * x830);
	const FLT x836 = (x715 * x834) + (x62 * x718) + (-1 * x725 * x832) + x745 + x92 + (-1 * x37 * x713) + x802 +
					 (-1 * x85 * x833) + (-1 * x721 * x829) + (-1 * x686 * x831);
	const FLT x837 = x756 + x794 + (-1 * x749 * x828) + (x738 * x829) + x797 + (-1 * x62 * x739) + (-1 * x686 * x830) +
					 (-1 * x91 * x833) + (x86 * x831) + (-1 * x725 * x834) + (-1 * x795);
	const FLT x838 = (x835 * sensor_pt[2]) + (x837 * sensor_pt[1]) + (-1 * x836 * sensor_pt[0]);
	const FLT x839 = x789 + (-1 * x725 * x829) + (x721 * x832) + (-1 * x738 * x834) + (-1 * x60 * x833) + (-1 * x786) +
					 x752 + (-1 * x724 * x830) + (-1 * x730 * x831) + x95 + (-1 * x748) + (x62 * x728);
	const FLT x840 = (x839 * sensor_pt[0]) + (-1 * x837 * sensor_pt[2]) + (x835 * sensor_pt[1]);
	const FLT x841 = (x577 * x836) + (-1 * x568 * x839) + (x576 * x838) + (-1 * x567 * x840);
	const FLT x842 = (x836 * sensor_pt[2]) + (-1 * x839 * sensor_pt[1]) + (x835 * sensor_pt[0]);
	const FLT x843 = (x581 * x839) + (x567 * x842) + (-1 * x579 * x838) + (-1 * x577 * x837);
	const FLT x844 = (x579 * x840) + (-1 * x581 * x836) + (x568 * x837) + (-1 * x576 * x842);
	const FLT x845 = x844 * x106;
	const FLT x846 = (x841 * x114) + (x27 * x845) + (-1 * x843 * x112);
	const FLT x847 = (-1 * x25 * x845) + (x841 * x107) + (x843 * x110);
	const FLT x848 = x843 + (x846 * x291) + (-1 * x847 * x272);
	const FLT x849 = (-1 * x841 * x110) + (x843 * x107) + (x28 * x845);
	const FLT x850 = (x849 * x272) + x841 + (-1 * x846 * x310);
	const FLT x851 = (x847 * x310) + x844 + (-1 * x849 * x291);
	const FLT x852 = (x851 * x133) + (x850 * x122);
	const FLT x853 = (-1 * x193 * (x852 + (x848 * x192))) + (x848 * x163);
	const FLT x854 = (x848 * x148) + (-1 * x852 * x141);
	const FLT x855 = ((x850 * x155) + (-1 * x851 * x154)) * x156;
	const FLT x856 = (-1 * x855) + (x854 * x153);
	const FLT x857 = x853 * x199;
	const FLT x858 = (x853 * x205) + (x164 * (x857 + (-1 * x853 * x204)));
	const FLT x859 = (x853 * x207) + (x858 * x164);
	const FLT x860 =
		x855 + (-1 * x220 *
				((x859 * x218) +
				 (-1 * x214 *
				  ((x856 * x212) +
				   (-1 * x183 *
					((x859 * x164) +
					 (x164 * (x859 + (x853 * x197) +
							  (x164 * (x858 + (x853 * x198) + (x164 * ((-1 * x853 * x203) + x857 + (x853 * x201))))))) +
					 (x853 * x210) + (x853 * x209))))) +
				 (x853 * x217) + x854 + (-1 * x856 * x191)));
	const FLT x861 = -1 * dt * x34;
	const FLT x862 = -1 * dt * x33;
	const FLT x863 = x862 + dt + x861;
	const FLT x864 = dt * x127;
	const FLT x865 = dt * x124;
	const FLT x866 = x865 + x864;
	const FLT x867 = (x866 * x133) + (x863 * x122);
	const FLT x868 = dt * x144;
	const FLT x869 = dt * x143;
	const FLT x870 = (-1 * x869) + x868;
	const FLT x871 = (x870 * x148) + (-1 * x867 * x141);
	const FLT x872 = ((x863 * x155) + (-1 * x866 * x154)) * x156;
	const FLT x873 = (-1 * x872) + (x871 * x153);
	const FLT x874 = (-1 * x193 * (x867 + (x870 * x192))) + (x870 * x163);
	const FLT x875 = x874 * x199;
	const FLT x876 = (x874 * x205) + (x164 * (x875 + (-1 * x874 * x204)));
	const FLT x877 = (x874 * x207) + (x876 * x164);
	const FLT x878 =
		x872 + (-1 * x220 *
				(x871 + (x877 * x218) + (x874 * x217) + (-1 * x873 * x191) +
				 (-1 * x214 *
				  ((x873 * x212) +
				   (-1 * x183 *
					((x877 * x164) +
					 (x164 * (x877 + (x874 * x197) +
							  (x164 * (x876 + (x874 * x198) + (x164 * ((-1 * x874 * x203) + x875 + (x874 * x201))))))) +
					 (x874 * x210) + (x874 * x209)))))));
	const FLT x879 = (-1 * dt * x224) + dt;
	const FLT x880 = x879 + x862;
	const FLT x881 = x868 + x869;
	const FLT x882 = dt * x229;
	const FLT x883 = dt * x228;
	const FLT x884 = (-1 * x883) + x882;
	const FLT x885 = (x884 * x133) + (x881 * x122);
	const FLT x886 = (-1 * x193 * (x885 + (x880 * x192))) + (x880 * x163);
	const FLT x887 = (x880 * x148) + (-1 * x885 * x141);
	const FLT x888 = ((x881 * x155) + (-1 * x884 * x154)) * x156;
	const FLT x889 = (-1 * x888) + (x887 * x153);
	const FLT x890 = x886 * x199;
	const FLT x891 = (x886 * x205) + (x164 * (x890 + (-1 * x886 * x204)));
	const FLT x892 = (x886 * x207) + (x891 * x164);
	const FLT x893 =
		x888 + (-1 * x220 *
				(x887 +
				 (-1 * x214 *
				  ((x889 * x212) +
				   (-1 * x183 *
					((x886 * x210) + (x892 * x164) +
					 (x164 * ((x886 * x197) + x892 +
							  (x164 * (x891 + (x886 * x198) + (x164 * ((-1 * x886 * x203) + x890 + (x886 * x201))))))) +
					 (x886 * x209))))) +
				 (x892 * x218) + (x886 * x217) + (-1 * x889 * x191)));
	const FLT x894 = x882 + x883;
	const FLT x895 = (-1 * x864) + x865;
	const FLT x896 = x879 + x861;
	const FLT x897 = (x896 * x133) + (x895 * x122);
	const FLT x898 = (-1 * x193 * (x897 + (x894 * x192))) + (x894 * x163);
	const FLT x899 = x898 * x199;
	const FLT x900 = (x898 * x205) + (x164 * (x899 + (-1 * x898 * x204)));
	const FLT x901 = (x898 * x207) + (x900 * x164);
	const FLT x902 = (x894 * x148) + (-1 * x897 * x141);
	const FLT x903 = ((x895 * x155) + (-1 * x896 * x154)) * x156;
	const FLT x904 = (-1 * x903) + (x902 * x153);
	const FLT x905 =
		x903 +
		(-1 * x220 *
		 (x902 +
		  (-1 * x214 *
		   ((x904 * x212) +
			(-1 * x183 *
			 ((x901 * x164) + (x898 * x210) + (x898 * x209) +
			  (x164 * (x901 + (x898 * x197) +
					   (x164 * ((x898 * x198) + x900 + (x164 * ((-1 * x898 * x203) + x899 + (x898 * x201))))))))))) +
		  (x901 * x218) + (x898 * x217) + (-1 * x904 * x191)));
	const FLT x906 = x213 * x184;
	const FLT x907 = (x189 + x906) * x220;
	const FLT x908 = ((-1 * x180 * x189) + (-1 * x906 * x180)) * x220;
	const FLT x909 = x220 * (x191 + (x906 * x190));
	const FLT x910 = x139 * x147 * (1 + x152);
	const FLT x911 = x160 * x195;
	const FLT x912 = x911 * x176 * x139;
	const FLT x913 = x912 * x199;
	const FLT x914 = (x912 * x205) + (x164 * (x913 + (-1 * x912 * x204)));
	const FLT x915 = (x912 * x207) + (x914 * x164);
	const FLT x916 = x910 * x153;
	const FLT x917 =
		x220 * ((-1 * x214 *
				 ((-1 * x183 *
				   ((x915 * x164) + (x912 * x209) +
					(x164 * (x915 + (x912 * x197) +
							 (x164 * (x914 + (x912 * x198) + (x164 * ((-1 * x912 * x203) + x913 + (x912 * x201))))))) +
					(x912 * x210))) +
				  (x916 * x212) + (-1 * x176) + (-1 * x161 * x175 * x182))) +
				(-1 * x916 * x191) + x910 + (x911 * x215 * x183 * x192) + (x915 * x218));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[0]) / sizeof(FLT),
						x221 + (x223 * x221));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[1]) / sizeof(FLT),
						x240 + (x223 * x240));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[2]) / sizeof(FLT),
						x253 + (x253 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						x350 + (x223 * x350));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						x396 + (x223 * x396));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						x430 + (x430 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						x454 + (x454 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Acc[0]) / sizeof(FLT), x475 + (x475 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Acc[1]) / sizeof(FLT), x490 + (x490 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Acc[2]) / sizeof(FLT), x502 + (x502 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[0]) / sizeof(FLT),
						x516 + (x516 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[1]) / sizeof(FLT),
						x529 + (x529 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[2]) / sizeof(FLT),
						x541 + (x541 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT),
						x598 + (x598 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT),
						x630 + (x630 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT),
						x659 + (x659 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT),
						x684 + (x684 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x778 + (x778 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x824 + (x824 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x860 + (x860 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[0]) / sizeof(FLT),
						x878 + (x878 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[1]) / sizeof(FLT),
						x893 + (x893 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[2]) / sizeof(FLT),
						x905 + (x905 * x223));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD0.curve) / sizeof(FLT),
						(-1 * x907 * x223) + (-1 * x907));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD0.gibmag) / sizeof(FLT), sin(x222));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD0.gibpha) / sizeof(FLT), x223);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD0.ogeemag) / sizeof(FLT),
						(-1 * x908 * x223) + (-1 * x908));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD0.ogeephase) / sizeof(FLT),
						(-1 * x909 * x223) + (-1 * x909));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD0.phase) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD0.tilt) / sizeof(FLT),
						(-1 * x917 * x223) + (-1 * x917));
}

// Full version Jacobian of SurviveJointKalmanErrorModel_LightMeas_x_gen2 wrt [(*_x0).Lighthouse.Pos[0],
// (*_x0).Lighthouse.Pos[1], (*_x0).Lighthouse.Pos[2], (*_x0).Lighthouse.Rot[0], (*_x0).Lighthouse.Rot[1],
// (*_x0).Lighthouse.Rot[2], (*_x0).Lighthouse.Rot[3], (*_x0).Object.Acc[0], (*_x0).Object.Acc[1], (*_x0).Object.Acc[2],
// (*_x0).Object.IMUBias.AccBias[0], (*_x0).Object.IMUBias.AccBias[1], (*_x0).Object.IMUBias.AccBias[2],
// (*_x0).Object.IMUBias.AccScale[0], (*_x0).Object.IMUBias.AccScale[1], (*_x0).Object.IMUBias.AccScale[2],
// (*_x0).Object.IMUBias.GyroBias[0], (*_x0).Object.IMUBias.GyroBias[1], (*_x0).Object.IMUBias.GyroBias[2],
// (*_x0).Object.IMUBias.IMUCorrection[0], (*_x0).Object.IMUBias.IMUCorrection[1],
// (*_x0).Object.IMUBias.IMUCorrection[2], (*_x0).Object.IMUBias.IMUCorrection[3], (*_x0).Object.Pose.Pos[0],
// (*_x0).Object.Pose.Pos[1], (*_x0).Object.Pose.Pos[2], (*_x0).Object.Pose.Rot[0], (*_x0).Object.Pose.Rot[1],
// (*_x0).Object.Pose.Rot[2], (*_x0).Object.Pose.Rot[3], (*_x0).Object.Velocity.AxisAngleRot[0],
// (*_x0).Object.Velocity.AxisAngleRot[1], (*_x0).Object.Velocity.AxisAngleRot[2], (*_x0).Object.Velocity.Pos[0],
// (*_x0).Object.Velocity.Pos[1], (*_x0).Object.Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f4d1c32c430>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32c6a0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32c730>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32c8e0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32c040>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32c610>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32c580>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32c820>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32c4c0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32c880>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32c1f0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32c2b0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c32c3a0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c32cee0>]

static inline void SurviveJointKalmanErrorModel_LightMeas_x_gen2_jac_x0_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveJointKalmanModel *_x0,
	const SurviveJointKalmanErrorModel *error_model, const FLT *sensor_pt) {
	if (hx != 0) {
		hx->data[0] = SurviveJointKalmanErrorModel_LightMeas_x_gen2(dt, _x0, error_model, sensor_pt);
	}
	if (Hx != 0) {
		SurviveJointKalmanErrorModel_LightMeas_x_gen2_jac_x0(Hx, dt, _x0, error_model, sensor_pt);
	}
}
// Jacobian of SurviveJointKalmanErrorModel_LightMeas_x_gen2 wrt [(*error_model).Lighthouse.AxisAngleRot[0],
// (*error_model).Lighthouse.AxisAngleRot[1], (*error_model).Lighthouse.AxisAngleRot[2],
// (*error_model).Lighthouse.Pos[0], (*error_model).Lighthouse.Pos[1], (*error_model).Lighthouse.Pos[2],
// (*error_model).Object.Acc[0], (*error_model).Object.Acc[1], (*error_model).Object.Acc[2],
// (*error_model).Object.IMUBias.AccBias[0], (*error_model).Object.IMUBias.AccBias[1],
// (*error_model).Object.IMUBias.AccBias[2], (*error_model).Object.IMUBias.AccScale[0],
// (*error_model).Object.IMUBias.AccScale[1], (*error_model).Object.IMUBias.AccScale[2],
// (*error_model).Object.IMUBias.GyroBias[0], (*error_model).Object.IMUBias.GyroBias[1],
// (*error_model).Object.IMUBias.GyroBias[2], (*error_model).Object.IMUBias.IMUCorrection[0],
// (*error_model).Object.IMUBias.IMUCorrection[1], (*error_model).Object.IMUBias.IMUCorrection[2],
// (*error_model).Object.Pose.AxisAngleRot[0], (*error_model).Object.Pose.AxisAngleRot[1],
// (*error_model).Object.Pose.AxisAngleRot[2], (*error_model).Object.Pose.Pos[0], (*error_model).Object.Pose.Pos[1],
// (*error_model).Object.Pose.Pos[2], (*error_model).Object.Velocity.AxisAngleRot[0],
// (*error_model).Object.Velocity.AxisAngleRot[1], (*error_model).Object.Velocity.AxisAngleRot[2],
// (*error_model).Object.Velocity.Pos[0], (*error_model).Object.Velocity.Pos[1], (*error_model).Object.Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3346a0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3387c0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3349d0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3382e0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c334640>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3382b0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c338430>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3381c0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3383d0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c338100>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c334be0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3385b0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3348e0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c338700>]
static inline void SurviveJointKalmanErrorModel_LightMeas_x_gen2_jac_error_model(
	CnMat *Hx, const FLT dt, const SurviveJointKalmanModel *_x0, const SurviveJointKalmanErrorModel *error_model,
	const FLT *sensor_pt) {
	const FLT x0 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[2];
	const FLT x1 = cos(x0);
	const FLT x2 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[0];
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[1];
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = x1 * x6;
	const FLT x8 = sin(x0);
	const FLT x9 = cos(x4);
	const FLT x10 = cos(x2);
	const FLT x11 = x9 * x10;
	const FLT x12 = x8 * x11;
	const FLT x13 = x12 + (-1 * x7);
	const FLT x14 = x3 * x9;
	const FLT x15 = x8 * x14;
	const FLT x16 = x5 * x10;
	const FLT x17 = x1 * x16;
	const FLT x18 = x17 + x15;
	const FLT x19 = x6 * x8;
	const FLT x20 = x1 * x11;
	const FLT x21 = x20 + x19;
	const FLT x22 = x8 * x16;
	const FLT x23 = x1 * x14;
	const FLT x24 = x23 + (-1 * x22);
	const FLT x25 = (x24 * x24) + (x21 * x21) + (x13 * x13) + (x18 * x18);
	const FLT x26 = 1. / sqrt(x25);
	const FLT x27 = x26 * (*_x0).Lighthouse.Rot[2];
	const FLT x28 = x26 * (*_x0).Lighthouse.Rot[0];
	const FLT x29 = x26 * (*_x0).Lighthouse.Rot[3];
	const FLT x30 = x26 * (*_x0).Lighthouse.Rot[1];
	const FLT x31 = (-1 * x29 * x18) + (x27 * x13) + (x30 * x21) + (x24 * x28);
	const FLT x32 = (x30 * x18) + (x28 * x13) + (x21 * x29) + (-1 * x24 * x27);
	const FLT x33 = x32 * x32;
	const FLT x34 = (-1 * x30 * x13) + (x28 * x18) + (x24 * x29) + (x21 * x27);
	const FLT x35 = x34 * x34;
	const FLT x36 = (x21 * x28) + (-1 * x29 * x13) + (-1 * x30 * x24) + (-1 * x27 * x18);
	const FLT x37 = x31 * x31;
	const FLT x38 = (x36 * x36) + x37 + x33 + x35;
	const FLT x39 = 1. / sqrt(x38);
	const FLT x40 = (*_x0).Lighthouse.Pos[2] + (*error_model).Lighthouse.Pos[2];
	const FLT x41 = x40 * x39;
	const FLT x42 = (*_x0).Lighthouse.Pos[1] + (*error_model).Lighthouse.Pos[1];
	const FLT x43 = x42 * x39;
	const FLT x44 = (*_x0).Lighthouse.Pos[0] + (*error_model).Lighthouse.Pos[0];
	const FLT x45 = x44 * x39;
	const FLT x46 = (x41 * x36) + (x45 * x34) + (-1 * x43 * x31);
	const FLT x47 = x46 * x39;
	const FLT x48 = (-1 * x41 * x34) + (x43 * x32) + (x45 * x36);
	const FLT x49 = x48 * x39;
	const FLT x50 = (*_x0).Object.Velocity.AxisAngleRot[2] + (*error_model).Object.Velocity.AxisAngleRot[2];
	const FLT x51 = dt * dt;
	const FLT x52 = x50 * x50;
	const FLT x53 = x52 * x51;
	const FLT x54 = (*_x0).Object.Velocity.AxisAngleRot[0] + (*error_model).Object.Velocity.AxisAngleRot[0];
	const FLT x55 = x54 * x54;
	const FLT x56 = x51 * x55;
	const FLT x57 = (*_x0).Object.Velocity.AxisAngleRot[1] + (*error_model).Object.Velocity.AxisAngleRot[1];
	const FLT x58 = x57 * x57;
	const FLT x59 = x51 * x58;
	const FLT x60 = 1e-10 + x53 + x59 + x56;
	const FLT x61 = sqrt(x60);
	const FLT x62 = 0.5 * x61;
	const FLT x63 = sin(x62);
	const FLT x64 = x63 * x63;
	const FLT x65 = 1. / x60;
	const FLT x66 = x64 * x65;
	const FLT x67 = cos(x62);
	const FLT x68 = (x66 * x56) + (x67 * x67) + (x66 * x53) + (x66 * x59);
	const FLT x69 = 1. / sqrt(x68);
	const FLT x70 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[0];
	const FLT x71 = sin(x70);
	const FLT x72 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[1];
	const FLT x73 = sin(x72);
	const FLT x74 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[2];
	const FLT x75 = sin(x74);
	const FLT x76 = x73 * x75;
	const FLT x77 = x71 * x76;
	const FLT x78 = cos(x74);
	const FLT x79 = cos(x70);
	const FLT x80 = cos(x72);
	const FLT x81 = x80 * x79;
	const FLT x82 = x81 * x78;
	const FLT x83 = x82 + x77;
	const FLT x84 = x80 * x71;
	const FLT x85 = x84 * x75;
	const FLT x86 = x73 * x78;
	const FLT x87 = x86 * x79;
	const FLT x88 = x87 + x85;
	const FLT x89 = x86 * x71;
	const FLT x90 = x81 * x75;
	const FLT x91 = x90 + (-1 * x89);
	const FLT x92 = x79 * x76;
	const FLT x93 = x84 * x78;
	const FLT x94 = x93 + (-1 * x92);
	const FLT x95 = (x94 * x94) + (x91 * x91) + (x88 * x88) + (x83 * x83);
	const FLT x96 = 1. / sqrt(x95);
	const FLT x97 = x83 * x96;
	const FLT x98 = x96 * x94;
	const FLT x99 = x96 * (*_x0).Object.Pose.Rot[0];
	const FLT x100 = x96 * (*_x0).Object.Pose.Rot[1];
	const FLT x101 =
		(x91 * x99) + (x97 * (*_x0).Object.Pose.Rot[3]) + (x88 * x100) + (-1 * x98 * (*_x0).Object.Pose.Rot[2]);
	const FLT x102 = x69 * x101;
	const FLT x103 = x63 * (1. / x61);
	const FLT x104 = dt * x103;
	const FLT x105 = x102 * x104;
	const FLT x106 =
		(-1 * x91 * x100) + (x88 * x99) + (x97 * (*_x0).Object.Pose.Rot[2]) + (x98 * (*_x0).Object.Pose.Rot[3]);
	const FLT x107 = x69 * x104;
	const FLT x108 = x107 * x106;
	const FLT x109 = x96 * (*_x0).Object.Pose.Rot[3];
	const FLT x110 = x96 * (*_x0).Object.Pose.Rot[2];
	const FLT x111 = (-1 * x94 * x100) + (x97 * (*_x0).Object.Pose.Rot[0]) + (-1 * x91 * x109) + (-1 * x88 * x110);
	const FLT x112 = x67 * x69;
	const FLT x113 = x111 * x112;
	const FLT x114 =
		(x97 * (*_x0).Object.Pose.Rot[1]) + (-1 * x88 * x109) + (x98 * (*_x0).Object.Pose.Rot[0]) + (x91 * x110);
	const FLT x115 = x107 * x114;
	const FLT x116 = (-1 * x54 * x115) + (-1 * x50 * x105) + x113 + (-1 * x57 * x108);
	const FLT x117 = x112 * x114;
	const FLT x118 = x107 * x111;
	const FLT x119 = (x54 * x118) + x117 + (-1 * x50 * x108) + (x57 * x105);
	const FLT x120 = x106 * x112;
	const FLT x121 = (x57 * x118) + (-1 * x54 * x105) + (x50 * x115) + x120;
	const FLT x122 = (-1 * x121 * sensor_pt[0]) + (x116 * sensor_pt[2]) + (x119 * sensor_pt[1]);
	const FLT x123 = x101 * x112;
	const FLT x124 = (x54 * x108) + (-1 * x57 * x115) + (x50 * x118) + x123;
	const FLT x125 = (-1 * x124 * sensor_pt[1]) + (x116 * sensor_pt[0]) + (x121 * sensor_pt[2]);
	const FLT x126 = dt * fabs(dt);
	const FLT x127 = 1.0 / 2.0 * x126;
	const FLT x128 = (*_x0).Object.Pose.Pos[1] + (2 * ((x124 * x125) + (-1 * x119 * x122))) + sensor_pt[1] +
					 (dt * ((*_x0).Object.Velocity.Pos[1] + (*error_model).Object.Velocity.Pos[1])) +
					 (*error_model).Object.Pose.Pos[1] + (x127 * ((*_x0).Object.Acc[1] + (*error_model).Object.Acc[1]));
	const FLT x129 = x39 * x128;
	const FLT x130 = (-1 * x119 * sensor_pt[2]) + (x116 * sensor_pt[1]) + (x124 * sensor_pt[0]);
	const FLT x131 = (dt * ((*_x0).Object.Velocity.Pos[2] + (*error_model).Object.Velocity.Pos[2])) +
					 (*error_model).Object.Pose.Pos[2] + (2 * ((x119 * x130) + (-1 * x121 * x125))) + sensor_pt[2] +
					 (x127 * ((*_x0).Object.Acc[2] + (*error_model).Object.Acc[2])) + (*_x0).Object.Pose.Pos[2];
	const FLT x132 = x39 * x131;
	const FLT x133 = (*_x0).Object.Pose.Pos[0] + (2 * ((x122 * x121) + (-1 * x124 * x130))) +
					 (*error_model).Object.Pose.Pos[0] +
					 (x127 * ((*_x0).Object.Acc[0] + (*error_model).Object.Acc[0])) + sensor_pt[0] +
					 (dt * ((*_x0).Object.Velocity.Pos[0] + (*error_model).Object.Velocity.Pos[0]));
	const FLT x134 = x39 * x133;
	const FLT x135 = (x34 * x134) + (-1 * x31 * x129) + (x36 * x132);
	const FLT x136 = x39 * x135;
	const FLT x137 = (x32 * x129) + (x36 * x134) + (-1 * x34 * x132);
	const FLT x138 = x39 * x137;
	const FLT x139 =
		(-1 * (x42 + (2 * ((-1 * x49 * x32) + (x47 * x31))))) + x128 + (2 * ((-1 * x32 * x138) + (x31 * x136)));
	const FLT x140 = x139 * x139;
	const FLT x141 = (x41 * x31) + (x43 * x36) + (-1 * x45 * x32);
	const FLT x142 = x39 * x141;
	const FLT x143 = (x31 * x132) + (x36 * x129) + (-1 * x32 * x134);
	const FLT x144 = x39 * x143;
	const FLT x145 =
		(-1 * (x40 + (2 * ((-1 * x31 * x142) + (x49 * x34))))) + x131 + (2 * ((-1 * x31 * x144) + (x34 * x138)));
	const FLT x146 =
		x133 + (2 * ((-1 * x34 * x136) + (x32 * x144))) + (-1 * (x44 + (2 * ((-1 * x47 * x34) + (x32 * x142)))));
	const FLT x147 = x146 * x146;
	const FLT x148 = x147 + (x145 * x145);
	const FLT x149 = x148 + x140;
	const FLT x150 = 1. / sqrt(x149);
	const FLT x151 = 0.523598775598299 + (*error_model).BSD0.tilt + (*_x0).BSD0.tilt;
	const FLT x152 = cos(x151);
	const FLT x153 = 1. / x152;
	const FLT x154 = x150 * x153;
	const FLT x155 = asin(x139 * x154);
	const FLT x156 = 0.5 * x20;
	const FLT x157 = 0.5 * x19;
	const FLT x158 = x157 + x156;
	const FLT x159 = 2 * x158;
	const FLT x160 = 0.5 * x23;
	const FLT x161 = -1 * x160;
	const FLT x162 = 0.5 * x22;
	const FLT x163 = x162 + x161;
	const FLT x164 = 2 * x21;
	const FLT x165 = 0.5 * x15;
	const FLT x166 = -0.5 * x17;
	const FLT x167 = x166 + (-1 * x165);
	const FLT x168 = 2 * x13;
	const FLT x169 = 0.5 * x7;
	const FLT x170 = -1 * x169;
	const FLT x171 = 0.5 * x12;
	const FLT x172 = x171 + x170;
	const FLT x173 = 2 * x18;
	const FLT x174 = 1.0 / 2.0 * (1. / (x25 * sqrt(x25)));
	const FLT x175 = x174 * ((x168 * x167) + (x24 * x159) + (x172 * x173) + (x164 * x163));
	const FLT x176 = x24 * x175;
	const FLT x177 = x26 * x158;
	const FLT x178 = -1 * x177 * (*_x0).Lighthouse.Rot[1];
	const FLT x179 = x21 * x175;
	const FLT x180 = x13 * x175;
	const FLT x181 = x29 * x167;
	const FLT x182 = x18 * x175;
	const FLT x183 = (x28 * x163) + (-1 * x27 * x172) + x178 + (x176 * (*_x0).Lighthouse.Rot[1]) + (-1 * x181) +
					 (x182 * (*_x0).Lighthouse.Rot[2]) + (-1 * x179 * (*_x0).Lighthouse.Rot[0]) +
					 (x180 * (*_x0).Lighthouse.Rot[3]);
	const FLT x184 = 2 * x36;
	const FLT x185 = x13 * (*_x0).Lighthouse.Rot[2];
	const FLT x186 = x26 * x167;
	const FLT x187 = x186 * (*_x0).Lighthouse.Rot[2];
	const FLT x188 = x177 * (*_x0).Lighthouse.Rot[0];
	const FLT x189 = (-1 * x29 * x172) + (-1 * x176 * (*_x0).Lighthouse.Rot[0]) +
					 (-1 * x179 * (*_x0).Lighthouse.Rot[1]) + (-1 * x175 * x185) + x187 + x188 +
					 (x182 * (*_x0).Lighthouse.Rot[3]) + (x30 * x163);
	const FLT x190 = 2 * x31;
	const FLT x191 = x13 * (*_x0).Lighthouse.Rot[0];
	const FLT x192 = x177 * (*_x0).Lighthouse.Rot[2];
	const FLT x193 = x186 * (*_x0).Lighthouse.Rot[0];
	const FLT x194 = x193 + (-1 * x182 * (*_x0).Lighthouse.Rot[1]) + (-1 * x192) + (x176 * (*_x0).Lighthouse.Rot[2]) +
					 (-1 * x175 * x191) + (-1 * x179 * (*_x0).Lighthouse.Rot[3]) + (x30 * x172) + (x29 * x163);
	const FLT x195 = 2 * x32;
	const FLT x196 = -1 * x186 * (*_x0).Lighthouse.Rot[1];
	const FLT x197 = x29 * x158;
	const FLT x198 = (x28 * x172) + x196 + (x180 * (*_x0).Lighthouse.Rot[1]) + (x27 * x163) +
					 (-1 * x182 * (*_x0).Lighthouse.Rot[0]) + (-1 * x179 * (*_x0).Lighthouse.Rot[2]) +
					 (-1 * x176 * (*_x0).Lighthouse.Rot[3]) + x197;
	const FLT x199 = 2 * x34;
	const FLT x200 = (x198 * x199) + (x194 * x195) + (x183 * x184) + (x189 * x190);
	const FLT x201 = 1. / (x38 * sqrt(x38));
	const FLT x202 = x32 * x201;
	const FLT x203 = x200 * x202;
	const FLT x204 = x31 * x201;
	const FLT x205 = x200 * x204;
	const FLT x206 = 2 * x138;
	const FLT x207 = 2 * x136;
	const FLT x208 = 2 * x189;
	const FLT x209 = x34 * x201;
	const FLT x210 = x200 * x209;
	const FLT x211 = 1.0 / 2.0 * x44;
	const FLT x212 = x36 * x201;
	const FLT x213 = x212 * x200;
	const FLT x214 = 1.0 / 2.0 * x213;
	const FLT x215 = x39 * x198;
	const FLT x216 = 1.0 / 2.0 * x205;
	const FLT x217 =
		(x42 * x216) + (-1 * x40 * x214) + (-1 * x211 * x210) + (-1 * x43 * x189) + (x41 * x183) + (x44 * x215);
	const FLT x218 = x39 * x190;
	const FLT x219 = 2 * x49;
	const FLT x220 = 1.0 / 2.0 * x42;
	const FLT x221 = 1.0 / 2.0 * x210;
	const FLT x222 = x39 * x183;
	const FLT x223 =
		(x40 * x221) + (-1 * x41 * x198) + (-1 * x211 * x213) + (x43 * x194) + (-1 * x203 * x220) + (x44 * x222);
	const FLT x224 = x39 * x195;
	const FLT x225 =
		(x216 * x128) + (-1 * x221 * x133) + (x215 * x133) + (-1 * x214 * x131) + (x222 * x131) + (-1 * x129 * x189);
	const FLT x226 = 1.0 / 2.0 * x128;
	const FLT x227 =
		(x221 * x131) + (x222 * x133) + (-1 * x215 * x131) + (-1 * x214 * x133) + (x129 * x194) + (-1 * x203 * x226);
	const FLT x228 = (-1 * x224 * x227) + (x207 * x189) + (x223 * x224) + (-1 * x206 * x194) + (x203 * x137) +
					 (x46 * x205) + (x219 * x194) + (-1 * x47 * x208) + (-1 * x218 * x217) + (-1 * x48 * x203) +
					 (x218 * x225) + (-1 * x205 * x135);
	const FLT x229 = 2 * x139;
	const FLT x230 = 2 * x144;
	const FLT x231 = x39 * x199;
	const FLT x232 =
		(x41 * x189) + (-1 * x40 * x216) + (-1 * x42 * x214) + (x211 * x203) + (x42 * x222) + (-1 * x45 * x194);
	const FLT x233 = 2 * x215;
	const FLT x234 = 1.0 / 2.0 * x133;
	const FLT x235 =
		(x189 * x132) + (-1 * x216 * x131) + (x129 * x183) + (-1 * x213 * x226) + (x234 * x203) + (-1 * x194 * x134);
	const FLT x236 = 2 * x142;
	const FLT x237 = (x217 * x231) + (x230 * x194) + (-1 * x231 * x225) + (-1 * x233 * x135) + (-1 * x232 * x224) +
					 (-1 * x46 * x210) + (-1 * x236 * x194) + (x203 * x141) + (x210 * x135) + (x235 * x224) +
					 (x46 * x233) + (-1 * x203 * x143);
	const FLT x238 = 2 * x146;
	const FLT x239 = x204 * x141;
	const FLT x240 = x204 * x143;
	const FLT x241 = (x200 * x240) + (x48 * x210) + (-1 * x239 * x200) + (x218 * x232) + (-1 * x231 * x223) +
					 (-1 * x48 * x233) + (-1 * x218 * x235) + (x233 * x137) + (-1 * x230 * x189) + (x231 * x227) +
					 (x208 * x142) + (-1 * x210 * x137);
	const FLT x242 = 2 * x145;
	const FLT x243 = (x241 * x242) + (x238 * x237);
	const FLT x244 = 1.0 / 2.0 * x139;
	const FLT x245 = x244 * (1. / (x149 * sqrt(x149))) * x153;
	const FLT x246 = (-1 * x245 * (x243 + (x228 * x229))) + (x228 * x154);
	const FLT x247 = -8.0108022e-06 + (-1.60216044e-05 * x155);
	const FLT x248 = 8.0108022e-06 * x155;
	const FLT x249 = -8.0108022e-06 + (-1 * x248);
	const FLT x250 = 0.0028679863 + (x249 * x155);
	const FLT x251 = x250 + (x247 * x155);
	const FLT x252 = 5.3685255e-06 + (x250 * x155);
	const FLT x253 = x252 + (x251 * x155);
	const FLT x254 = 1. / (x152 * x152);
	const FLT x255 = 1. / sqrt(1 + (-1 * x254 * x140 * (1. / x149)));
	const FLT x256 = x253 * x255;
	const FLT x257 = x251 * x255;
	const FLT x258 = x255 * x249;
	const FLT x259 = x258 * x246;
	const FLT x260 = x255 * x247;
	const FLT x261 = 2.40324066e-05 * x155;
	const FLT x262 = x261 * x255;
	const FLT x263 = x255 * x248;
	const FLT x264 = x250 * x255;
	const FLT x265 = (x264 * x246) + (x155 * (x259 + (-1 * x263 * x246)));
	const FLT x266 = x252 * x255;
	const FLT x267 = (x266 * x246) + (x265 * x155);
	const FLT x268 = 0.0076069798 + (x252 * x155);
	const FLT x269 = x268 + (x253 * x155);
	const FLT x270 = x269 * x255;
	const FLT x271 = x268 * x255;
	const FLT x272 = sin(x151);
	const FLT x273 = atan2(-1 * x145, x146);
	const FLT x274 = 1. / sqrt(x148);
	const FLT x275 = tan(x151);
	const FLT x276 = x275 * x274;
	const FLT x277 = x276 * x139;
	const FLT x278 = asin(x277) + (-1 * x273) + (-1 * (*error_model).BSD0.ogeephase) + (-1 * (*_x0).BSD0.ogeephase);
	const FLT x279 = sin(x278);
	const FLT x280 = (*error_model).BSD0.ogeemag + (*_x0).BSD0.ogeemag;
	const FLT x281 = (*_x0).BSD0.curve + (*error_model).BSD0.curve + (-1 * x279 * x280);
	const FLT x282 = x272 * x281;
	const FLT x283 = 1. / x148;
	const FLT x284 = x275 * x275;
	const FLT x285 = 1. / sqrt(1 + (-1 * x283 * x284 * x140));
	const FLT x286 = x275 * x244 * (1. / (x148 * sqrt(x148)));
	const FLT x287 = (x276 * x228) + (-1 * x286 * x243);
	const FLT x288 = 1. / x146;
	const FLT x289 = x145 * (1. / x147);
	const FLT x290 = x283 * x147;
	const FLT x291 = ((x237 * x289) + (-1 * x288 * x241)) * x290;
	const FLT x292 = (-1 * x291) + (x287 * x285);
	const FLT x293 = x280 * cos(x278);
	const FLT x294 = x268 * x155;
	const FLT x295 = (x269 * x155) + x294;
	const FLT x296 = x272 * x295;
	const FLT x297 = x296 * x293;
	const FLT x298 = x155 * x155;
	const FLT x299 = x295 * x282;
	const FLT x300 = x152 + (-1 * x299);
	const FLT x301 = x298 * x268 * (1. / (x300 * x300));
	const FLT x302 = x281 * x301;
	const FLT x303 = 1. / x300;
	const FLT x304 = x298 * x303;
	const FLT x305 = x268 * x304;
	const FLT x306 = x293 * x305;
	const FLT x307 = x294 * x303;
	const FLT x308 = 2 * x281 * x307;
	const FLT x309 = x255 * x308;
	const FLT x310 = x281 * x304;
	const FLT x311 = x277 + (x281 * x305);
	const FLT x312 = 1. / sqrt(1 + (-1 * (x311 * x311)));
	const FLT x313 =
		x291 + (-1 * x312 *
				((x267 * x310) + x287 + (x246 * x309) +
				 (-1 * x302 *
				  ((x297 * x292) +
				   (-1 * x282 *
					((x267 * x155) + (x271 * x246) +
					 (x155 * (x267 + (x256 * x246) +
							  (x155 * (x265 + (x257 * x246) + (x155 * ((-1 * x262 * x246) + x259 + (x260 * x246))))))) +
					 (x270 * x246))))) +
				 (-1 * x292 * x306)));
	const FLT x314 = (-1 * asin(x311)) + (*error_model).BSD0.gibpha + x273 + (*_x0).BSD0.gibpha;
	const FLT x315 = cos(x314) * ((*error_model).BSD0.gibmag + (*_x0).BSD0.gibmag);
	const FLT x316 = x165 + x166;
	const FLT x317 = x26 * x316;
	const FLT x318 = -1 * x171;
	const FLT x319 = x318 + x170;
	const FLT x320 = 2 * x24;
	const FLT x321 = -1 * x162;
	const FLT x322 = x161 + x321;
	const FLT x323 = (-1 * x157) + x156;
	const FLT x324 = ((x323 * x173) + (x322 * x168) + (x320 * x319) + (x316 * x164)) * x174;
	const FLT x325 = x18 * x324;
	const FLT x326 = x26 * x322;
	const FLT x327 = x24 * x324;
	const FLT x328 = x21 * x324;
	const FLT x329 = (-1 * x328 * (*_x0).Lighthouse.Rot[1]) + (x325 * (*_x0).Lighthouse.Rot[3]) +
					 (x326 * (*_x0).Lighthouse.Rot[2]) + (-1 * x327 * (*_x0).Lighthouse.Rot[0]) +
					 (x317 * (*_x0).Lighthouse.Rot[1]) + (x28 * x319) + (-1 * x29 * x323) + (-1 * x324 * x185);
	const FLT x330 = x13 * x324;
	const FLT x331 = (-1 * x27 * x323) + (x330 * (*_x0).Lighthouse.Rot[3]) + (-1 * x30 * x319) +
					 (-1 * x328 * (*_x0).Lighthouse.Rot[0]) + (x327 * (*_x0).Lighthouse.Rot[1]) +
					 (x317 * (*_x0).Lighthouse.Rot[0]) + (-1 * x29 * x322) + (x325 * (*_x0).Lighthouse.Rot[2]);
	const FLT x332 = (-1 * x324 * x191) + (-1 * x328 * (*_x0).Lighthouse.Rot[3]) + (-1 * x27 * x319) +
					 (x327 * (*_x0).Lighthouse.Rot[2]) + (x29 * x316) + (-1 * x325 * (*_x0).Lighthouse.Rot[1]) +
					 (x30 * x323) + (x326 * (*_x0).Lighthouse.Rot[0]);
	const FLT x333 = (x317 * (*_x0).Lighthouse.Rot[2]) + (-1 * x328 * (*_x0).Lighthouse.Rot[2]) + (x29 * x319) +
					 (x330 * (*_x0).Lighthouse.Rot[1]) + (-1 * x326 * (*_x0).Lighthouse.Rot[1]) +
					 (-1 * x327 * (*_x0).Lighthouse.Rot[3]) + (-1 * x325 * (*_x0).Lighthouse.Rot[0]) + (x28 * x323);
	const FLT x334 = (x333 * x199) + (x332 * x195) + (x329 * x190) + (x331 * x184);
	const FLT x335 = x202 * x334;
	const FLT x336 = x212 * x334;
	const FLT x337 = x209 * x334;
	const FLT x338 = 1.0 / 2.0 * x131;
	const FLT x339 =
		(x332 * x129) + (x338 * x337) + (-1 * x226 * x335) + (-1 * x234 * x336) + (x331 * x134) + (-1 * x333 * x132);
	const FLT x340 = 2 * x47;
	const FLT x341 = x204 * x334;
	const FLT x342 = 1.0 / 2.0 * x40;
	const FLT x343 =
		(x337 * x342) + (x43 * x332) + (-1 * x211 * x336) + (-1 * x220 * x335) + (-1 * x41 * x333) + (x45 * x331);
	const FLT x344 = 1.0 / 2.0 * x336;
	const FLT x345 = 1.0 / 2.0 * x341;
	const FLT x346 =
		(x42 * x345) + (x45 * x333) + (-1 * x40 * x344) + (x41 * x331) + (-1 * x211 * x337) + (-1 * x43 * x329);
	const FLT x347 =
		(x333 * x134) + (x345 * x128) + (x331 * x132) + (-1 * x344 * x131) + (-1 * x234 * x337) + (-1 * x329 * x129);
	const FLT x348 = (x46 * x341) + (x219 * x332) + (-1 * x224 * x339) + (-1 * x218 * x346) + (x207 * x329) +
					 (x335 * x137) + (-1 * x329 * x340) + (-1 * x341 * x135) + (-1 * x48 * x335) + (x224 * x343) +
					 (x218 * x347) + (-1 * x206 * x332);
	const FLT x349 =
		(-1 * x226 * x336) + (x329 * x132) + (-1 * x345 * x131) + (x234 * x335) + (x331 * x129) + (-1 * x332 * x134);
	const FLT x350 =
		(x41 * x329) + (x43 * x331) + (-1 * x40 * x345) + (-1 * x42 * x344) + (-1 * x45 * x332) + (x211 * x335);
	const FLT x351 = (-1 * x46 * x337) + (x337 * x135) + (x231 * x346) + (x224 * x349) + (x230 * x332) + (x333 * x340) +
					 (-1 * x231 * x347) + (x335 * x141) + (-1 * x335 * x143) + (-1 * x236 * x332) + (-1 * x207 * x333) +
					 (-1 * x224 * x350);
	const FLT x352 = (x206 * x333) + (-1 * x239 * x334) + (-1 * x230 * x329) + (x240 * x334) + (-1 * x218 * x349) +
					 (-1 * x231 * x343) + (x231 * x339) + (x218 * x350) + (-1 * x219 * x333) + (-1 * x337 * x137) +
					 (x236 * x329) + (x48 * x337);
	const FLT x353 = (x242 * x352) + (x238 * x351);
	const FLT x354 = (-1 * x245 * (x353 + (x229 * x348))) + (x348 * x154);
	const FLT x355 = (x276 * x348) + (-1 * x286 * x353);
	const FLT x356 = ((x289 * x351) + (-1 * x288 * x352)) * x290;
	const FLT x357 = (-1 * x356) + (x285 * x355);
	const FLT x358 = x258 * x354;
	const FLT x359 = (x264 * x354) + (x155 * (x358 + (-1 * x263 * x354)));
	const FLT x360 = (x266 * x354) + (x359 * x155);
	const FLT x361 =
		x356 + (-1 * x312 *
				(x355 + (x360 * x310) +
				 (-1 * x302 *
				  ((x297 * x357) +
				   (-1 * x282 *
					((x155 * ((x256 * x354) + x360 +
							  (x155 * ((x257 * x354) + x359 + (x155 * ((-1 * x262 * x354) + x358 + (x260 * x354))))))) +
					 (x360 * x155) + (x270 * x354) + (x271 * x354))))) +
				 (x354 * x309) + (-1 * x357 * x306)));
	const FLT x362 = x169 + x318;
	const FLT x363 = x160 + x321;
	const FLT x364 = x174 * ((x13 * x159) + (x363 * x173) + (x320 * x167) + (x362 * x164));
	const FLT x365 = x21 * x364;
	const FLT x366 = x18 * x364;
	const FLT x367 = x24 * x364;
	const FLT x368 = (-1 * x29 * x363) + (-1 * x367 * (*_x0).Lighthouse.Rot[0]) + (x30 * x362) + x193 +
					 (-1 * x365 * (*_x0).Lighthouse.Rot[1]) + (x366 * (*_x0).Lighthouse.Rot[3]) + (-1 * x364 * x185) +
					 x192;
	const FLT x369 = 2 * x368;
	const FLT x370 = x13 * x364;
	const FLT x371 = (x28 * x362) + (-1 * x27 * x363) + x196 + (x367 * (*_x0).Lighthouse.Rot[1]) +
					 (x366 * (*_x0).Lighthouse.Rot[2]) + (-1 * x365 * (*_x0).Lighthouse.Rot[0]) +
					 (x370 * (*_x0).Lighthouse.Rot[3]) + (-1 * x197);
	const FLT x372 = (-1 * x187) + (x367 * (*_x0).Lighthouse.Rot[2]) + x188 + (-1 * x364 * x191) + (x30 * x363) +
					 (x29 * x362) + (-1 * x366 * (*_x0).Lighthouse.Rot[1]) + (-1 * x365 * (*_x0).Lighthouse.Rot[3]);
	const FLT x373 = (-1 * x365 * (*_x0).Lighthouse.Rot[2]) + (x27 * x362) + x178 + (x370 * (*_x0).Lighthouse.Rot[1]) +
					 (-1 * x366 * (*_x0).Lighthouse.Rot[0]) + (x28 * x363) + x181 +
					 (-1 * x367 * (*_x0).Lighthouse.Rot[3]);
	const FLT x374 = (x372 * x195) + (x373 * x199) + (x371 * x184) + (x368 * x190);
	const FLT x375 = x202 * x374;
	const FLT x376 = x212 * x374;
	const FLT x377 = x204 * x374;
	const FLT x378 = 1.0 / 2.0 * x377;
	const FLT x379 = x209 * x374;
	const FLT x380 =
		(-1 * x43 * x368) + (x42 * x378) + (-1 * x211 * x379) + (x45 * x373) + (-1 * x376 * x342) + (x41 * x371);
	const FLT x381 =
		(-1 * x376 * x338) + (x226 * x377) + (-1 * x234 * x379) + (x371 * x132) + (-1 * x368 * x129) + (x373 * x134);
	const FLT x382 = 1.0 / 2.0 * x379;
	const FLT x383 =
		(-1 * x234 * x376) + (-1 * x226 * x375) + (-1 * x373 * x132) + (x372 * x129) + (x371 * x134) + (x382 * x131);
	const FLT x384 = 2 * x372;
	const FLT x385 = x46 * x374;
	const FLT x386 =
		(x43 * x372) + (x40 * x382) + (x45 * x371) + (-1 * x220 * x375) + (-1 * x41 * x373) + (-1 * x211 * x376);
	const FLT x387 = (-1 * x377 * x135) + (-1 * x206 * x372) + (x375 * x137) + (x369 * x136) + (-1 * x48 * x375) +
					 (-1 * x218 * x380) + (-1 * x224 * x383) + (-1 * x47 * x369) + (x49 * x384) + (x218 * x381) +
					 (x224 * x386) + (x204 * x385);
	const FLT x388 = 2 * x373;
	const FLT x389 =
		(x234 * x375) + (-1 * x226 * x376) + (-1 * x378 * x131) + (x371 * x129) + (-1 * x372 * x134) + (x368 * x132);
	const FLT x390 =
		(x41 * x368) + (x211 * x375) + (-1 * x45 * x372) + (x43 * x371) + (-1 * x40 * x378) + (-1 * x220 * x376);
	const FLT x391 = (-1 * x224 * x390) + (-1 * x375 * x143) + (-1 * x207 * x373) + (x47 * x388) + (x384 * x144) +
					 (-1 * x384 * x142) + (x379 * x135) + (-1 * x209 * x385) + (x224 * x389) + (-1 * x231 * x381) +
					 (x375 * x141) + (x231 * x380);
	const FLT x392 = (x231 * x383) + (-1 * x218 * x389) + (-1 * x231 * x386) + (x48 * x379) + (-1 * x369 * x144) +
					 (-1 * x377 * x141) + (x369 * x142) + (x240 * x374) + (x206 * x373) + (-1 * x49 * x388) +
					 (x218 * x390) + (-1 * x379 * x137);
	const FLT x393 = (x242 * x392) + (x238 * x391);
	const FLT x394 = (-1 * x245 * (x393 + (x229 * x387))) + (x387 * x154);
	const FLT x395 = x258 * x394;
	const FLT x396 = (x264 * x394) + (x155 * (x395 + (-1 * x263 * x394)));
	const FLT x397 = (x266 * x394) + (x396 * x155);
	const FLT x398 = (x276 * x387) + (-1 * x286 * x393);
	const FLT x399 = ((x289 * x391) + (-1 * x288 * x392)) * x290;
	const FLT x400 = (-1 * x399) + (x285 * x398);
	const FLT x401 =
		x399 +
		(-1 * x312 *
		 ((x397 * x310) + x398 + (x394 * x309) +
		  (-1 * x302 *
		   ((x400 * x297) +
			(-1 * x282 *
			 ((x397 * x155) + (x271 * x394) + (x270 * x394) +
			  (x155 * (x397 + (x256 * x394) +
					   (x155 * (x396 + (x257 * x394) + (x155 * ((-1 * x262 * x394) + x395 + (x260 * x394))))))))))) +
		  (-1 * x400 * x306)));
	const FLT x402 = 1. / x38;
	const FLT x403 = 2 * x402;
	const FLT x404 = x33 * x403;
	const FLT x405 = x35 * x403;
	const FLT x406 = -1 + x405;
	const FLT x407 = x406 + x404;
	const FLT x408 = x32 * x402;
	const FLT x409 = x408 * x190;
	const FLT x410 = -1 * x409;
	const FLT x411 = x34 * x402;
	const FLT x412 = x411 * x184;
	const FLT x413 = -1 * x412;
	const FLT x414 = x413 + x410;
	const FLT x415 = (x414 * x242) + (x407 * x238);
	const FLT x416 = x408 * x184;
	const FLT x417 = x411 * x190;
	const FLT x418 = -1 * x417;
	const FLT x419 = x418 + x416;
	const FLT x420 = (x419 * x276) + (-1 * x415 * x286);
	const FLT x421 = ((x407 * x289) + (-1 * x414 * x288)) * x290;
	const FLT x422 = (-1 * x421) + (x420 * x285);
	const FLT x423 = (-1 * x245 * (x415 + (x419 * x229))) + (x419 * x154);
	const FLT x424 = x423 * x258;
	const FLT x425 = (x423 * x264) + (x155 * (x424 + (-1 * x423 * x263)));
	const FLT x426 = (x423 * x266) + (x425 * x155);
	const FLT x427 =
		x421 + (-1 * x312 *
				((x426 * x310) + x420 + (x423 * x309) + (-1 * x422 * x306) +
				 (-1 * x302 *
				  ((x422 * x297) +
				   (-1 * x282 *
					((x423 * x270) +
					 (x155 * (x426 + (x423 * x256) +
							  (x155 * (x425 + (x423 * x257) + (x155 * ((-1 * x423 * x262) + x424 + (x423 * x260))))))) +
					 (x426 * x155) + (x423 * x271)))))));
	const FLT x428 = x37 * x403;
	const FLT x429 = -1 + x428 + x404;
	const FLT x430 = -1 * x416;
	const FLT x431 = x430 + x418;
	const FLT x432 = x31 * x402 * x184;
	const FLT x433 = x411 * x195;
	const FLT x434 = -1 * x433;
	const FLT x435 = x434 + x432;
	const FLT x436 = (x435 * x242) + (x431 * x238);
	const FLT x437 = (-1 * x245 * (x436 + (x429 * x229))) + (x429 * x154);
	const FLT x438 = x437 * x258;
	const FLT x439 = (x437 * x264) + (x155 * (x438 + (-1 * x437 * x263)));
	const FLT x440 = (x437 * x266) + (x439 * x155);
	const FLT x441 = (x429 * x276) + (-1 * x436 * x286);
	const FLT x442 = ((x431 * x289) + (-1 * x435 * x288)) * x290;
	const FLT x443 = (-1 * x442) + (x441 * x285);
	const FLT x444 =
		x442 + (-1 * x312 *
				(x441 + (-1 * x443 * x306) + (x437 * x309) + (x440 * x310) +
				 (-1 * x302 *
				  ((x443 * x297) +
				   (-1 * x282 *
					((x440 * x155) + (x437 * x271) +
					 (x155 * (x440 + (x437 * x256) +
							  (x155 * (x439 + (x437 * x257) + (x155 * ((-1 * x437 * x262) + (x437 * x260) + x438)))))) +
					 (x437 * x270)))))));
	const FLT x445 = -1 * x432;
	const FLT x446 = x445 + x434;
	const FLT x447 = x410 + x412;
	const FLT x448 = x406 + x428;
	const FLT x449 = (x448 * x242) + (x447 * x238);
	const FLT x450 = (-1 * x245 * (x449 + (x446 * x229))) + (x446 * x154);
	const FLT x451 = (x446 * x276) + (-1 * x449 * x286);
	const FLT x452 = ((x447 * x289) + (-1 * x448 * x288)) * x290;
	const FLT x453 = (-1 * x452) + (x451 * x285);
	const FLT x454 = x450 * x258;
	const FLT x455 = (x450 * x264) + (x155 * (x454 + (-1 * x450 * x263)));
	const FLT x456 = (x450 * x266) + (x455 * x155);
	const FLT x457 =
		x452 + (-1 * x312 *
				(x451 + (x456 * x310) +
				 (-1 * x302 *
				  ((x453 * x297) +
				   (-1 * x282 *
					((x456 * x155) + (x450 * x271) +
					 (x155 * (x456 + (x450 * x256) +
							  (x155 * (x455 + (x450 * x257) + (x155 * ((-1 * x450 * x262) + x454 + (x450 * x260))))))) +
					 (x450 * x270))))) +
				 (x450 * x309) + (-1 * x453 * x306)));
	const FLT x458 = x402 * x126;
	const FLT x459 = -1 * x35 * x458;
	const FLT x460 = -1 * x33 * x458;
	const FLT x461 = x460 + x127 + x459;
	const FLT x462 = x411 * x126;
	const FLT x463 = x36 * x462;
	const FLT x464 = x408 * x126;
	const FLT x465 = x31 * x464;
	const FLT x466 = x465 + x463;
	const FLT x467 = (x466 * x242) + (x461 * x238);
	const FLT x468 = x31 * x462;
	const FLT x469 = x36 * x464;
	const FLT x470 = (-1 * x469) + x468;
	const FLT x471 = (x470 * x276) + (-1 * x467 * x286);
	const FLT x472 = ((x461 * x289) + (-1 * x466 * x288)) * x290;
	const FLT x473 = (-1 * x472) + (x471 * x285);
	const FLT x474 = (-1 * x245 * (x467 + (x470 * x229))) + (x470 * x154);
	const FLT x475 = x474 * x258;
	const FLT x476 = (x474 * x264) + (x155 * (x475 + (-1 * x474 * x263)));
	const FLT x477 = (x474 * x266) + (x476 * x155);
	const FLT x478 =
		x472 + (-1 * x312 *
				((x477 * x310) + (x474 * x309) + (-1 * x473 * x306) + x471 +
				 (-1 * x302 *
				  ((x473 * x297) +
				   (-1 * x282 *
					((x474 * x271) +
					 (x155 * (x477 + (x474 * x256) +
							  (x155 * (x476 + (x474 * x257) + (x155 * ((-1 * x474 * x262) + (x474 * x260) + x475)))))) +
					 (x477 * x155) + (x474 * x270)))))));
	const FLT x479 = x468 + x469;
	const FLT x480 = x32 * x462;
	const FLT x481 = x31 * x36 * x458;
	const FLT x482 = (-1 * x481) + x480;
	const FLT x483 = (x482 * x242) + (x479 * x238);
	const FLT x484 = (-1 * x37 * x458) + x127;
	const FLT x485 = x484 + x460;
	const FLT x486 = (x485 * x276) + (-1 * x483 * x286);
	const FLT x487 = ((x479 * x289) + (-1 * x482 * x288)) * x290;
	const FLT x488 = (-1 * x487) + (x486 * x285);
	const FLT x489 = (-1 * x245 * (x483 + (x485 * x229))) + (x485 * x154);
	const FLT x490 = x489 * x258;
	const FLT x491 = (x489 * x264) + (x155 * (x490 + (-1 * x489 * x263)));
	const FLT x492 = (x489 * x266) + (x491 * x155);
	const FLT x493 =
		x487 + (-1 * x312 *
				(x486 + (-1 * x488 * x306) + (x492 * x310) + (x489 * x309) +
				 (-1 * x302 *
				  ((x488 * x297) +
				   (-1 * x282 *
					((x492 * x155) +
					 (x155 * (x492 + (x489 * x256) +
							  (x155 * (x491 + (x489 * x257) + (x155 * ((-1 * x489 * x262) + x490 + (x489 * x260))))))) +
					 (x489 * x271) + (x489 * x270)))))));
	const FLT x494 = x480 + x481;
	const FLT x495 = (-1 * x463) + x465;
	const FLT x496 = x484 + x459;
	const FLT x497 = (x496 * x242) + (x495 * x238);
	const FLT x498 = (-1 * x245 * (x497 + (x494 * x229))) + (x494 * x154);
	const FLT x499 = x498 * x258;
	const FLT x500 = x498 * x255;
	const FLT x501 = (x498 * x264) + (x155 * (x499 + (-1 * x500 * x248)));
	const FLT x502 = (x498 * x266) + (x501 * x155);
	const FLT x503 = (x494 * x276) + (-1 * x497 * x286);
	const FLT x504 = ((x495 * x289) + (-1 * x496 * x288)) * x290;
	const FLT x505 = x293 * ((-1 * x504) + (x503 * x285));
	const FLT x506 =
		x504 +
		(-1 * x312 *
		 ((x502 * x310) + (-1 * x505 * x305) +
		  (-1 * x302 *
		   ((x505 * x296) +
			(-1 * x282 *
			 ((x502 * x155) + (x498 * x271) + (x498 * x270) +
			  (x155 * (x502 + (x498 * x256) +
					   (x155 * (x501 + (x498 * x257) + (x155 * ((-1 * x500 * x261) + x499 + (x498 * x260))))))))))) +
		  x503 + (x500 * x308)));
	const FLT x507 = 0.5 * x82;
	const FLT x508 = 0.5 * x77;
	const FLT x509 = x508 + x507;
	const FLT x510 = x96 * x509;
	const FLT x511 = -1 * x510 * (*_x0).Object.Pose.Rot[1];
	const FLT x512 = 1.0 / 2.0 * (1. / (x95 * sqrt(x95)));
	const FLT x513 = x512 * (*_x0).Object.Pose.Rot[3];
	const FLT x514 = 2 * x94;
	const FLT x515 = 0.5 * x89;
	const FLT x516 = -1 * x515;
	const FLT x517 = 0.5 * x90;
	const FLT x518 = x517 + x516;
	const FLT x519 = 2 * x88;
	const FLT x520 = 0.5 * x85;
	const FLT x521 = -0.5 * x87;
	const FLT x522 = x521 + (-1 * x520);
	const FLT x523 = 2 * x91;
	const FLT x524 = 0.5 * x93;
	const FLT x525 = -1 * x524;
	const FLT x526 = 0.5 * x92;
	const FLT x527 = x526 + x525;
	const FLT x528 = 2 * x83;
	const FLT x529 = (x522 * x523) + (x528 * x527) + (x514 * x509) + (x519 * x518);
	const FLT x530 = x91 * x529;
	const FLT x531 = x522 * x109;
	const FLT x532 = x512 * (*_x0).Object.Pose.Rot[2];
	const FLT x533 = x88 * x529;
	const FLT x534 = x512 * (*_x0).Object.Pose.Rot[1];
	const FLT x535 = x94 * x529;
	const FLT x536 = x512 * (*_x0).Object.Pose.Rot[0];
	const FLT x537 = x83 * x529;
	const FLT x538 = (x513 * x530) + (x533 * x532) + (-1 * x537 * x536) + x511 + (x535 * x534) + (-1 * x531) +
					 (x99 * x527) + (-1 * x518 * x110);
	const FLT x539 = x54 * x107;
	const FLT x540 = x510 * (*_x0).Object.Pose.Rot[0];
	const FLT x541 = x522 * x110;
	const FLT x542 = x541 + (-1 * x532 * x530) + (-1 * x535 * x536) + (x527 * x100) + (x513 * x533) + x540 +
					 (-1 * x518 * x109) + (-1 * x534 * x537);
	const FLT x543 = -1 * x522 * x100;
	const FLT x544 = x512 * x537;
	const FLT x545 = x510 * (*_x0).Object.Pose.Rot[3];
	const FLT x546 = (x99 * x518) + (-1 * x513 * x535) + x545 + (-1 * x533 * x536) + x543 +
					 (-1 * x544 * (*_x0).Object.Pose.Rot[2]) + (x534 * x530) + (x527 * x110);
	const FLT x547 = x50 * x107;
	const FLT x548 = x510 * (*_x0).Object.Pose.Rot[2];
	const FLT x549 = x99 * x522;
	const FLT x550 = (-1 * x544 * (*_x0).Object.Pose.Rot[3]) + (x527 * x109) + (-1 * x548) + (-1 * x530 * x536) +
					 (-1 * x533 * x534) + (x518 * x100) + x549 + (x532 * x535);
	const FLT x551 = x57 * x107;
	const FLT x552 = (x550 * x551) + (-1 * x546 * x547) + (x539 * x538) + (x542 * x112);
	const FLT x553 = (-1 * x546 * x551) + (-1 * x539 * x542) + (x538 * x112) + (-1 * x547 * x550);
	const FLT x554 = (x539 * x546) + (x550 * x112) + (-1 * x542 * x551) + (x538 * x547);
	const FLT x555 = (x554 * sensor_pt[0]) + (-1 * x552 * sensor_pt[2]) + (x553 * sensor_pt[1]);
	const FLT x556 = 2 * x124;
	const FLT x557 = 2 * x130;
	const FLT x558 = (x546 * x112) + (-1 * x539 * x550) + (x542 * x547) + (x538 * x551);
	const FLT x559 = (x552 * sensor_pt[1]) + (-1 * x558 * sensor_pt[0]) + (x553 * sensor_pt[2]);
	const FLT x560 = 2 * x121;
	const FLT x561 = 2 * x122;
	const FLT x562 = (x558 * x561) + (x559 * x560) + (-1 * x556 * x555) + (-1 * x554 * x557);
	const FLT x563 = x39 * x562;
	const FLT x564 = (x558 * sensor_pt[2]) + (-1 * x554 * sensor_pt[1]) + (x553 * sensor_pt[0]);
	const FLT x565 = 2 * x119;
	const FLT x566 = 2 * x125;
	const FLT x567 = (x554 * x566) + (-1 * x552 * x561) + (x556 * x564) + (-1 * x559 * x565);
	const FLT x568 = x39 * x567;
	const FLT x569 = (x555 * x565) + (x552 * x557) + (-1 * x558 * x566) + (-1 * x560 * x564);
	const FLT x570 = x39 * x569;
	const FLT x571 = (x36 * x570) + (x34 * x563) + (-1 * x31 * x568);
	const FLT x572 = (-1 * x34 * x570) + (x32 * x568) + (x36 * x563);
	const FLT x573 = x567 + (x571 * x218) + (-1 * x572 * x224);
	const FLT x574 = (x36 * x568) + (-1 * x32 * x563) + (x31 * x570);
	const FLT x575 = x562 + (x574 * x224) + (-1 * x571 * x231);
	const FLT x576 = (x572 * x231) + x569 + (-1 * x574 * x218);
	const FLT x577 = (x576 * x242) + (x575 * x238);
	const FLT x578 = (-1 * x245 * (x577 + (x573 * x229))) + (x573 * x154);
	const FLT x579 = (x573 * x276) + (-1 * x577 * x286);
	const FLT x580 = ((x575 * x289) + (-1 * x576 * x288)) * x290;
	const FLT x581 = (-1 * x580) + (x579 * x285);
	const FLT x582 = x578 * x258;
	const FLT x583 = (x578 * x264) + (x155 * (x582 + (-1 * x578 * x263)));
	const FLT x584 = (x578 * x266) + (x583 * x155);
	const FLT x585 =
		x580 + (-1 * x312 *
				((x584 * x310) + x579 + (x578 * x309) +
				 (-1 * x302 *
				  ((x581 * x297) +
				   (-1 * x282 *
					((x584 * x155) + (x578 * x270) +
					 (x155 * (x584 + (x578 * x256) +
							  (x155 * (x583 + (x578 * x257) + (x155 * ((-1 * x578 * x262) + x582 + (x578 * x260))))))) +
					 (x578 * x271))))) +
				 (-1 * x581 * x306)));
	const FLT x586 = -1 * x517;
	const FLT x587 = x586 + x516;
	const FLT x588 = -1 * x526;
	const FLT x589 = x525 + x588;
	const FLT x590 = (-1 * x508) + x507;
	const FLT x591 = x520 + x521;
	const FLT x592 = (x591 * x528) + (x590 * x519) + (x514 * x587) + (x523 * x589);
	const FLT x593 = x94 * x592;
	const FLT x594 = x91 * x592;
	const FLT x595 = x88 * x592;
	const FLT x596 = x96 * x589;
	const FLT x597 = x83 * x592;
	const FLT x598 = (-1 * x597 * x536) + (x593 * x534) + (x594 * x513) + (-1 * x587 * x100) + (x99 * x591) +
					 (-1 * x590 * x110) + (x595 * x532) + (-1 * x596 * (*_x0).Object.Pose.Rot[3]);
	const FLT x599 = (x99 * x587) + (x596 * (*_x0).Object.Pose.Rot[2]) + (x591 * x100) + (-1 * x594 * x532) +
					 (-1 * x593 * x536) + (-1 * x597 * x534) + (-1 * x590 * x109) + (x595 * x513);
	const FLT x600 = (-1 * x593 * x513) + (x99 * x590) + (-1 * x595 * x536) + (x587 * x109) + (-1 * x597 * x532) +
					 (-1 * x589 * x100) + (x594 * x534) + (x591 * x110);
	const FLT x601 = (x593 * x532) + (-1 * x587 * x110) + (-1 * x597 * x513) + (x590 * x100) + (-1 * x594 * x536) +
					 (-1 * x595 * x534) + (x596 * (*_x0).Object.Pose.Rot[0]) + (x591 * x109);
	const FLT x602 = (x601 * x551) + (-1 * x600 * x547) + (x598 * x539) + (x599 * x112);
	const FLT x603 = (-1 * x600 * x551) + (-1 * x599 * x539) + (-1 * x601 * x547) + (x598 * x112);
	const FLT x604 = (x601 * x112) + (x598 * x547) + (x600 * x539) + (-1 * x599 * x551);
	const FLT x605 = (x604 * sensor_pt[0]) + (-1 * x602 * sensor_pt[2]) + (x603 * sensor_pt[1]);
	const FLT x606 = (x600 * x112) + (x599 * x547) + (-1 * x601 * x539) + (x598 * x551);
	const FLT x607 = (x602 * sensor_pt[1]) + (-1 * x606 * sensor_pt[0]) + (x603 * sensor_pt[2]);
	const FLT x608 = (x607 * x560) + (-1 * x604 * x557) + (-1 * x605 * x556) + (x606 * x561);
	const FLT x609 = x39 * x608;
	const FLT x610 = (x606 * sensor_pt[2]) + (x603 * sensor_pt[0]) + (-1 * x604 * sensor_pt[1]);
	const FLT x611 = (-1 * x602 * x561) + (x604 * x566) + (-1 * x607 * x565) + (x610 * x556);
	const FLT x612 = x39 * x611;
	const FLT x613 = (x602 * x557) + (x605 * x565) + (-1 * x610 * x560) + (-1 * x606 * x566);
	const FLT x614 = x39 * x613;
	const FLT x615 = (x34 * x609) + (x36 * x614) + (-1 * x31 * x612);
	const FLT x616 = (x31 * x614) + (-1 * x32 * x609) + (x36 * x612);
	const FLT x617 = x608 + (-1 * x615 * x231) + (x616 * x224);
	const FLT x618 = (-1 * x34 * x614) + (x32 * x612) + (x36 * x609);
	const FLT x619 = x613 + (x618 * x231) + (-1 * x616 * x218);
	const FLT x620 = (x619 * x242) + (x617 * x238);
	const FLT x621 = x611 + (x615 * x218) + (-1 * x618 * x224);
	const FLT x622 = (x621 * x276) + (-1 * x620 * x286);
	const FLT x623 = ((x617 * x289) + (-1 * x619 * x288)) * x290;
	const FLT x624 = (-1 * x623) + (x622 * x285);
	const FLT x625 = (-1 * x245 * (x620 + (x621 * x229))) + (x621 * x154);
	const FLT x626 = x625 * x258;
	const FLT x627 = (x625 * x264) + (x155 * (x626 + (-1 * x625 * x263)));
	const FLT x628 = (x625 * x266) + (x627 * x155);
	const FLT x629 =
		x623 + (-1 * x312 *
				((x628 * x310) + (x625 * x309) + x622 + (-1 * x624 * x306) +
				 (-1 * x302 *
				  ((x624 * x297) +
				   (-1 * x282 *
					((x628 * x155) +
					 (x155 * (x628 + (x625 * x256) +
							  (x155 * (x627 + (x625 * x257) + (x155 * ((-1 * x625 * x262) + x626 + (x625 * x260))))))) +
					 (x625 * x270) + (x625 * x271)))))));
	const FLT x630 = x524 + x588;
	const FLT x631 = x515 + x586;
	const FLT x632 = (x630 * x519) + (x523 * x509) + (x631 * x528) + (x514 * x522);
	const FLT x633 = x83 * x632;
	const FLT x634 = x91 * x632;
	const FLT x635 = x634 * x512;
	const FLT x636 = x94 * x632;
	const FLT x637 = x88 * x632;
	const FLT x638 = x543 + (x635 * (*_x0).Object.Pose.Rot[3]) + (x636 * x534) + (-1 * x633 * x536) + (x99 * x631) +
					 (-1 * x545) + (x637 * x532) + (-1 * x630 * x110);
	const FLT x639 = (-1 * x635 * (*_x0).Object.Pose.Rot[2]) + (x637 * x513) + x548 + (-1 * x636 * x536) +
					 (x631 * x100) + (-1 * x630 * x109) + x549 + (-1 * x633 * x534);
	const FLT x640 = (x99 * x630) + (x634 * x534) + (-1 * x636 * x513) + (-1 * x633 * x532) + x511 + x531 +
					 (-1 * x637 * x536) + (x631 * x110);
	const FLT x641 = (x636 * x532) + (-1 * x633 * x513) + x540 + (-1 * x634 * x536) + (x630 * x100) + (x631 * x109) +
					 (-1 * x637 * x534) + (-1 * x541);
	const FLT x642 = (-1 * x640 * x547) + (x638 * x539) + (x641 * x551) + (x639 * x112);
	const FLT x643 = (-1 * x641 * x547) + (-1 * x639 * x539) + (-1 * x640 * x551) + (x638 * x112);
	const FLT x644 = (-1 * x639 * x551) + (x640 * x539) + (x641 * x112) + (x638 * x547);
	const FLT x645 = (x644 * sensor_pt[0]) + (-1 * x642 * sensor_pt[2]) + (x643 * sensor_pt[1]);
	const FLT x646 = (x640 * x112) + (x638 * x551) + (-1 * x641 * x539) + (x639 * x547);
	const FLT x647 = (x642 * sensor_pt[1]) + (x643 * sensor_pt[2]) + (-1 * x646 * sensor_pt[0]);
	const FLT x648 = (x647 * x560) + (x646 * x561) + (-1 * x645 * x556) + (-1 * x644 * x557);
	const FLT x649 = x39 * x648;
	const FLT x650 = (-1 * x644 * sensor_pt[1]) + (x646 * sensor_pt[2]) + (x643 * sensor_pt[0]);
	const FLT x651 = (-1 * x642 * x561) + (x650 * x556) + (-1 * x647 * x565) + (x644 * x566);
	const FLT x652 = x39 * x651;
	const FLT x653 = (x645 * x565) + (x642 * x557) + (-1 * x646 * x566) + (-1 * x650 * x560);
	const FLT x654 = x39 * x653;
	const FLT x655 = (x36 * x654) + (x34 * x649) + (-1 * x31 * x652);
	const FLT x656 = (-1 * x34 * x654) + (x36 * x649) + (x32 * x652);
	const FLT x657 = x651 + (x655 * x218) + (-1 * x656 * x224);
	const FLT x658 = (-1 * x32 * x649) + (x31 * x654) + (x36 * x652);
	const FLT x659 = x648 + (x658 * x224) + (-1 * x655 * x231);
	const FLT x660 = x653 + (x656 * x231) + (-1 * x658 * x218);
	const FLT x661 = (x660 * x242) + (x659 * x238);
	const FLT x662 = (-1 * x245 * (x661 + (x657 * x229))) + (x657 * x154);
	const FLT x663 = (x657 * x276) + (-1 * x661 * x286);
	const FLT x664 = ((x659 * x289) + (-1 * x660 * x288)) * x290;
	const FLT x665 = (-1 * x664) + (x663 * x285);
	const FLT x666 = x662 * x258;
	const FLT x667 = (x662 * x264) + (x155 * (x666 + (-1 * x662 * x263)));
	const FLT x668 = (x662 * x266) + (x667 * x155);
	const FLT x669 =
		x664 + (-1 * x312 *
				(x663 + (x668 * x310) +
				 (-1 * x302 *
				  ((x665 * x297) +
				   (-1 * x282 *
					((x668 * x155) +
					 (x155 * (x668 + (x662 * x256) +
							  (x155 * (x667 + (x662 * x257) + (x155 * ((-1 * x662 * x262) + (x662 * x260) + x666)))))) +
					 (x662 * x270) + (x662 * x271))))) +
				 (x662 * x309) + (-1 * x665 * x306)));
	const FLT x670 = x430 + x417;
	const FLT x671 = -1 * x404;
	const FLT x672 = 1 + (-1 * x405);
	const FLT x673 = x672 + x671;
	const FLT x674 = x409 + x412;
	const FLT x675 = (x674 * x242) + (x673 * x238);
	const FLT x676 = (-1 * x245 * (x675 + (x670 * x229))) + (x670 * x154);
	const FLT x677 = x676 * x258;
	const FLT x678 = (x676 * x264) + (x155 * (x677 + (-1 * x676 * x263)));
	const FLT x679 = (x676 * x266) + (x678 * x155);
	const FLT x680 = (x670 * x276) + (-1 * x675 * x286);
	const FLT x681 = ((x673 * x289) + (-1 * x674 * x288)) * x290;
	const FLT x682 = (-1 * x681) + (x680 * x285);
	const FLT x683 =
		x681 + (-1 * x312 *
				((x679 * x310) + (x676 * x309) +
				 (-1 * x302 *
				  ((x682 * x297) +
				   (-1 * x282 *
					((x679 * x155) +
					 (x155 * (x679 + (x676 * x256) +
							  (x155 * ((x676 * x257) + x678 + (x155 * ((-1 * x676 * x262) + x677 + (x676 * x260))))))) +
					 (x676 * x270) + (x676 * x271))))) +
				 x680 + (-1 * x682 * x306)));
	const FLT x684 = x417 + x416;
	const FLT x685 = x445 + x433;
	const FLT x686 = (x685 * x242) + (x684 * x238);
	const FLT x687 = -1 * x428;
	const FLT x688 = 1 + x671 + x687;
	const FLT x689 = (x688 * x276) + (-1 * x686 * x286);
	const FLT x690 = ((x684 * x289) + (-1 * x685 * x288)) * x290;
	const FLT x691 = (-1 * x690) + (x689 * x285);
	const FLT x692 = (-1 * x245 * (x686 + (x688 * x229))) + (x688 * x154);
	const FLT x693 = x692 * x258;
	const FLT x694 = (x692 * x264) + (x155 * (x693 + (-1 * x692 * x263)));
	const FLT x695 = (x692 * x266) + (x694 * x155);
	const FLT x696 =
		x690 + (-1 * x312 *
				((x692 * x309) + (x695 * x310) + x689 + (-1 * x691 * x306) +
				 (-1 * x302 *
				  ((x691 * x297) +
				   (-1 * x282 *
					((x695 * x155) + (x692 * x270) +
					 (x155 * (x695 + (x692 * x256) +
							  (x155 * (x694 + (x692 * x257) + (x155 * ((-1 * x692 * x262) + x693 + (x692 * x260))))))) +
					 (x692 * x271)))))));
	const FLT x697 = x433 + x432;
	const FLT x698 = x413 + x409;
	const FLT x699 = x672 + x687;
	const FLT x700 = (x699 * x242) + (x698 * x238);
	const FLT x701 = (-1 * x245 * (x700 + (x697 * x229))) + (x697 * x154);
	const FLT x702 = x701 * x255;
	const FLT x703 = (x697 * x276) + (-1 * x700 * x286);
	const FLT x704 = ((x698 * x289) + (-1 * x699 * x288)) * x290;
	const FLT x705 = (-1 * x704) + (x703 * x285);
	const FLT x706 = x701 * x258;
	const FLT x707 = (x702 * x250) + (x155 * (x706 + (-1 * x702 * x248)));
	const FLT x708 = (x702 * x252) + (x707 * x155);
	const FLT x709 =
		x704 + (-1 * x312 *
				((-1 * x302 *
				  ((x705 * x297) +
				   (-1 * x282 *
					((x708 * x155) +
					 (x155 * (x708 + (x702 * x253) +
							  (x155 * (x707 + (x702 * x251) + (x155 * ((-1 * x702 * x261) + (x702 * x247) + x706)))))) +
					 (x702 * x269) + (x702 * x268))))) +
				 x703 + (x708 * x310) + (x702 * x308) + (-1 * x705 * x306)));
	const FLT x710 = -1 * x105;
	const FLT x711 = x69 * x114;
	const FLT x712 = dt * dt * dt;
	const FLT x713 = (1. / (x60 * sqrt(x60))) * x63;
	const FLT x714 = x712 * x713;
	const FLT x715 = x711 * x714;
	const FLT x716 = x54 * x715;
	const FLT x717 = x50 * x716;
	const FLT x718 = dt * dt * dt * dt;
	const FLT x719 = (x54 * x54 * x54) * x718;
	const FLT x720 = 2 * (1. / (x60 * x60)) * x64;
	const FLT x721 = 1.0 * x67;
	const FLT x722 = x713 * x721;
	const FLT x723 = x718 * x722;
	const FLT x724 = x54 * x723;
	const FLT x725 = x718 * x720;
	const FLT x726 = x54 * x725;
	const FLT x727 = 2 * x66;
	const FLT x728 = x51 * x727;
	const FLT x729 = x51 * x103;
	const FLT x730 = x54 * x729;
	const FLT x731 = (x719 * x722) + (x58 * x724) + (-1 * x730 * x721) + (x52 * x724) + (-1 * x719 * x720) +
					 (x54 * x728) + (-1 * x58 * x726) + (-1 * x52 * x726);
	const FLT x732 = 1.0 / 2.0 * (1. / (x68 * sqrt(x68)));
	const FLT x733 = x67 * x732;
	const FLT x734 = x731 * x733;
	const FLT x735 = x69 * x106;
	const FLT x736 = 0.5 * x735;
	const FLT x737 = x732 * x104;
	const FLT x738 = x737 * x101;
	const FLT x739 = x54 * x738;
	const FLT x740 = x50 * x54;
	const FLT x741 = 0.5 * x65;
	const FLT x742 = x712 * x741;
	const FLT x743 = x742 * x117;
	const FLT x744 = x740 * x743;
	const FLT x745 = x731 * x737;
	const FLT x746 = x50 * x745;
	const FLT x747 = x55 * x712;
	const FLT x748 = x713 * x747;
	const FLT x749 = x57 * x745;
	const FLT x750 = x747 * x741;
	const FLT x751 = x54 * x57;
	const FLT x752 = x742 * x113;
	const FLT x753 = x69 * x111;
	const FLT x754 = x714 * x753;
	const FLT x755 = (-1 * x754 * x751) + (x751 * x752);
	const FLT x756 = (-1 * x746 * x114) + x744 + (-1 * x750 * x123) + (x748 * x102) + (-1 * x717) + x710 +
					 (-1 * x730 * x736) + (-1 * x734 * x106) + x755 + (-1 * x749 * x111) + (x731 * x739);
	const FLT x757 = -1 * x115;
	const FLT x758 = x714 * x735;
	const FLT x759 = x758 * x751;
	const FLT x760 = x742 * x120;
	const FLT x761 = x760 * x751;
	const FLT x762 = 0.5 * x753;
	const FLT x763 = x54 * x745;
	const FLT x764 = x742 * x123;
	const FLT x765 = x714 * x102;
	const FLT x766 = (x765 * x740) + (-1 * x764 * x740);
	const FLT x767 = x766 + (x763 * x114) + (-1 * x761) + (x711 * x748) + (-1 * x734 * x111) + x757 +
					 (-1 * x762 * x730) + (x50 * x731 * x738) + (-1 * x750 * x117) + (x749 * x106) + x759;
	const FLT x768 = x764 * x751;
	const FLT x769 = x740 * x758;
	const FLT x770 = x57 * x738;
	const FLT x771 = x765 * x751;
	const FLT x772 = x760 * x740;
	const FLT x773 = 0.5 * x711;
	const FLT x774 = (-1 * x763 * x111) + (-1 * x770 * x731) + x769 + (-1 * x734 * x114) + (x746 * x106) + x118 +
					 (-1 * x771) + (x750 * x113) + x768 + (-1 * x772) + (-1 * x748 * x753) + (-1 * x773 * x730);
	const FLT x775 = (x774 * sensor_pt[1]) + (-1 * x756 * sensor_pt[0]) + (x767 * sensor_pt[2]);
	const FLT x776 = 0.5 * x102;
	const FLT x777 = (x57 * x716) + (-1 * x743 * x751);
	const FLT x778 = (-1 * x740 * x754) + (x740 * x752);
	const FLT x779 = x777 + (-1 * x735 * x748) + x108 + x778 + (-1 * x763 * x106) + (x750 * x120) + (-1 * x776 * x730) +
					 (x749 * x114) + (-1 * x734 * x101) + (-1 * x746 * x111);
	const FLT x780 = (x756 * sensor_pt[2]) + (-1 * x779 * sensor_pt[1]) + (x767 * sensor_pt[0]);
	const FLT x781 = (x779 * x566) + (-1 * x775 * x565) + (-1 * x774 * x561) + (x780 * x556);
	const FLT x782 = x39 * x781;
	const FLT x783 = (-1 * x774 * sensor_pt[2]) + (x779 * sensor_pt[0]) + (x767 * sensor_pt[1]);
	const FLT x784 = (x775 * x560) + (x756 * x561) + (-1 * x783 * x556) + (-1 * x779 * x557);
	const FLT x785 = x39 * x784;
	const FLT x786 = (-1 * x780 * x560) + (x783 * x565) + (-1 * x756 * x566) + (x774 * x557);
	const FLT x787 = x39 * x786;
	const FLT x788 = (x32 * x782) + (-1 * x34 * x787) + (x36 * x785);
	const FLT x789 = (x36 * x787) + (x34 * x785) + (-1 * x31 * x782);
	const FLT x790 = x781 + (-1 * x788 * x224) + (x789 * x218);
	const FLT x791 = (-1 * x32 * x785) + (x31 * x787) + (x36 * x782);
	const FLT x792 = x784 + (x791 * x224) + (-1 * x789 * x231);
	const FLT x793 = x786 + (-1 * x791 * x218) + (x788 * x231);
	const FLT x794 = (x793 * x242) + (x792 * x238);
	const FLT x795 = (-1 * x245 * (x794 + (x790 * x229))) + (x790 * x154);
	const FLT x796 = (x790 * x276) + (-1 * x794 * x286);
	const FLT x797 = ((x792 * x289) + (-1 * x793 * x288)) * x290;
	const FLT x798 = (-1 * x797) + (x796 * x285);
	const FLT x799 = x795 * x258;
	const FLT x800 = (x795 * x264) + (x155 * (x799 + (-1 * x795 * x263)));
	const FLT x801 = (x795 * x266) + (x800 * x155);
	const FLT x802 =
		x797 + (-1 * x312 *
				(x796 + (x801 * x310) +
				 (-1 * x302 *
				  ((x798 * x297) +
				   (-1 * x282 *
					((x801 * x155) + (x795 * x271) +
					 (x155 * ((x795 * x256) + x801 +
							  (x155 * (x800 + (x795 * x257) + (x155 * ((-1 * x795 * x262) + (x795 * x260) + x799)))))) +
					 (x795 * x270))))) +
				 (x795 * x309) + (-1 * x798 * x306)));
	const FLT x803 = x55 * x725;
	const FLT x804 = x55 * x723;
	const FLT x805 = x57 * x729;
	const FLT x806 = x52 * x57;
	const FLT x807 = x57 * x57 * x57;
	const FLT x808 = (x57 * x728) + (x723 * x807) + (-1 * x725 * x807) + (x57 * x804) + (-1 * x57 * x803) +
					 (-1 * x721 * x805) + (x723 * x806) + (-1 * x725 * x806);
	const FLT x809 = x733 * x808;
	const FLT x810 = x50 * x57;
	const FLT x811 = x742 * x810;
	const FLT x812 = x811 * x117;
	const FLT x813 = x737 * x111;
	const FLT x814 = x808 * x813;
	const FLT x815 = x715 * x810;
	const FLT x816 = x58 * x742;
	const FLT x817 = x50 * x808;
	const FLT x818 = x737 * x114;
	const FLT x819 = (-1 * x58 * x754) + (-1 * x736 * x805) + (-1 * x768) + (-1 * x809 * x106) + x812 +
					 (-1 * x57 * x814) + (-1 * x815) + (x816 * x113) + (x739 * x808) + (-1 * x818 * x817) + x118 + x771;
	const FLT x820 = x811 * x123;
	const FLT x821 = x737 * x106;
	const FLT x822 = x808 * x821;
	const FLT x823 = x765 * x810;
	const FLT x824 = -1 * x108;
	const FLT x825 = x808 * x818;
	const FLT x826 = x777 + (-1 * x762 * x805) + (x54 * x825) + (x738 * x817) + (-1 * x816 * x120) + (-1 * x820) +
					 (x57 * x822) + (-1 * x809 * x111) + x824 + (x58 * x758) + x823;
	const FLT x827 = (x758 * x810) + (-1 * x811 * x120);
	const FLT x828 = (-1 * x770 * x808) + (x817 * x821) + (-1 * x54 * x814) + x827 + (-1 * x773 * x805) +
					 (x816 * x123) + (-1 * x809 * x114) + x755 + (-1 * x58 * x765) + x105;
	const FLT x829 = (x828 * sensor_pt[1]) + (-1 * x819 * sensor_pt[0]) + (x826 * sensor_pt[2]);
	const FLT x830 = (x752 * x810) + (-1 * x754 * x810);
	const FLT x831 = x830 + (-1 * x816 * x117) + (-1 * x54 * x822) + (-1 * x776 * x805) + (-1 * x759) +
					 (-1 * x809 * x101) + (x58 * x715) + x757 + (x57 * x825) + x761 + (-1 * x813 * x817);
	const FLT x832 = (x819 * sensor_pt[2]) + (-1 * x831 * sensor_pt[1]) + (x826 * sensor_pt[0]);
	const FLT x833 = (x566 * x831) + (x556 * x832) + (-1 * x565 * x829) + (-1 * x561 * x828);
	const FLT x834 = x39 * x833;
	const FLT x835 = (x831 * sensor_pt[0]) + (-1 * x828 * sensor_pt[2]) + (x826 * sensor_pt[1]);
	const FLT x836 = (x561 * x819) + (x560 * x829) + (-1 * x556 * x835) + (-1 * x557 * x831);
	const FLT x837 = x39 * x836;
	const FLT x838 = (x565 * x835) + (-1 * x566 * x819) + (x557 * x828) + (-1 * x560 * x832);
	const FLT x839 = x39 * x838;
	const FLT x840 = (-1 * x31 * x834) + (x36 * x839) + (x34 * x837);
	const FLT x841 = (-1 * x34 * x839) + (x32 * x834) + (x36 * x837);
	const FLT x842 = x833 + (x840 * x218) + (-1 * x841 * x224);
	const FLT x843 = (-1 * x32 * x837) + (x31 * x839) + (x36 * x834);
	const FLT x844 = x836 + (x843 * x224) + (-1 * x840 * x231);
	const FLT x845 = x838 + (x841 * x231) + (-1 * x843 * x218);
	const FLT x846 = (x845 * x242) + (x844 * x238);
	const FLT x847 = (-1 * x245 * (x846 + (x842 * x229))) + (x842 * x154);
	const FLT x848 = (x842 * x276) + (-1 * x846 * x286);
	const FLT x849 = ((x844 * x289) + (-1 * x845 * x288)) * x290;
	const FLT x850 = (-1 * x849) + (x848 * x285);
	const FLT x851 = x847 * x258;
	const FLT x852 = (x847 * x264) + (x155 * (x851 + (-1 * x847 * x263)));
	const FLT x853 = (x847 * x266) + (x852 * x155);
	const FLT x854 =
		x849 + (-1 * x312 *
				(x848 + (x853 * x310) +
				 (-1 * x302 *
				  ((x850 * x297) +
				   (-1 * x282 *
					((x853 * x155) + (x847 * x271) +
					 (x155 * (x853 + (x847 * x256) +
							  (x155 * (x852 + (x847 * x257) + (x155 * ((-1 * x847 * x262) + x851 + (x847 * x260))))))) +
					 (x847 * x270))))) +
				 (x847 * x309) + (-1 * x850 * x306)));
	const FLT x855 = x50 * x51;
	const FLT x856 = x855 * x103;
	const FLT x857 = x50 * x58;
	const FLT x858 = x50 * x50 * x50;
	const FLT x859 = (x723 * x857) + (-1 * x725 * x858) + (-1 * x50 * x803) + (-1 * x721 * x856) + (x723 * x858) +
					 (x50 * x804) + (-1 * x725 * x857) + (x727 * x855);
	const FLT x860 = x50 * x859;
	const FLT x861 = x57 * x859;
	const FLT x862 = x52 * x742;
	const FLT x863 = x733 * x859;
	const FLT x864 = x54 * x859;
	const FLT x865 = x827 + (-1 * x863 * x111) + (x821 * x861) + x710 + (-1 * x744) + x717 + (-1 * x862 * x123) +
					 (x738 * x860) + (x818 * x864) + (-1 * x762 * x856) + (x52 * x765);
	const FLT x866 = x766 + (x739 * x859) + (x862 * x117) + x830 + (-1 * x813 * x861) + x115 + (-1 * x52 * x715) +
					 (-1 * x736 * x856) + (-1 * x863 * x106) + (-1 * x818 * x860);
	const FLT x867 = x778 + (-1 * x862 * x120) + x820 + (x821 * x860) + (-1 * x863 * x114) + (-1 * x823) + x824 +
					 (-1 * x770 * x859) + (x52 * x758) + (-1 * x813 * x864) + (-1 * x773 * x856);
	const FLT x868 = (x867 * sensor_pt[1]) + (x865 * sensor_pt[2]) + (-1 * x866 * sensor_pt[0]);
	const FLT x869 = x815 + (-1 * x863 * x101) + (-1 * x52 * x754) + x118 + x772 + (-1 * x813 * x860) +
					 (-1 * x776 * x856) + (-1 * x769) + (x862 * x113) + (x818 * x861) + (-1 * x812) +
					 (-1 * x821 * x864);
	const FLT x870 = 2 * ((x869 * sensor_pt[0]) + (-1 * x867 * sensor_pt[2]) + (x865 * sensor_pt[1]));
	const FLT x871 = (x561 * x866) + (-1 * x557 * x869) + (x560 * x868) + (-1 * x870 * x124);
	const FLT x872 = x39 * x871;
	const FLT x873 = (x866 * sensor_pt[2]) + (-1 * x869 * sensor_pt[1]) + (x865 * sensor_pt[0]);
	const FLT x874 = (x566 * x869) + (-1 * x565 * x868) + (x556 * x873) + (-1 * x561 * x867);
	const FLT x875 = x39 * x874;
	const FLT x876 = (x870 * x119) + (-1 * x566 * x866) + (x557 * x867) + (-1 * x560 * x873);
	const FLT x877 = x39 * x876;
	const FLT x878 = (x36 * x877) + (x34 * x872) + (-1 * x31 * x875);
	const FLT x879 = (-1 * x34 * x877) + (x36 * x872) + (x32 * x875);
	const FLT x880 = x874 + (x878 * x218) + (-1 * x879 * x224);
	const FLT x881 = (x36 * x875) + (-1 * x32 * x872) + (x31 * x877);
	const FLT x882 = (x881 * x224) + x871 + (-1 * x878 * x231);
	const FLT x883 = x876 + (x879 * x231) + (-1 * x881 * x218);
	const FLT x884 = (x883 * x242) + (x882 * x238);
	const FLT x885 = (-1 * x245 * (x884 + (x880 * x229))) + (x880 * x154);
	const FLT x886 = (x880 * x276) + (-1 * x884 * x286);
	const FLT x887 = ((x882 * x289) + (-1 * x883 * x288)) * x290;
	const FLT x888 = (-1 * x887) + (x886 * x285);
	const FLT x889 = x885 * x258;
	const FLT x890 = (x885 * x264) + (x155 * (x889 + (-1 * x885 * x263)));
	const FLT x891 = (x885 * x266) + (x890 * x155);
	const FLT x892 =
		x887 + (-1 * x312 *
				(x886 + (x891 * x310) +
				 (-1 * x302 *
				  ((x888 * x297) +
				   (-1 * x282 *
					((x891 * x155) +
					 (x155 * (x891 + (x885 * x256) +
							  (x155 * ((x885 * x257) + x890 + (x155 * (x889 + (-1 * x885 * x262) + (x885 * x260))))))) +
					 (x885 * x270) + (x885 * x271))))) +
				 (x885 * x309) + (-1 * x888 * x306)));
	const FLT x893 = -1 * dt * x405;
	const FLT x894 = (-1 * dt * x404) + dt;
	const FLT x895 = x894 + x893;
	const FLT x896 = dt * x412;
	const FLT x897 = dt * x409;
	const FLT x898 = x897 + x896;
	const FLT x899 = (x898 * x242) + (x895 * x238);
	const FLT x900 = dt * x417;
	const FLT x901 = dt * x416;
	const FLT x902 = (-1 * x901) + x900;
	const FLT x903 = (x902 * x276) + (-1 * x899 * x286);
	const FLT x904 = ((x895 * x289) + (-1 * x898 * x288)) * x290;
	const FLT x905 = (-1 * x904) + (x903 * x285);
	const FLT x906 = (-1 * x245 * (x899 + (x902 * x229))) + (x902 * x154);
	const FLT x907 = x906 * x258;
	const FLT x908 = (x906 * x264) + (x155 * (x907 + (-1 * x906 * x263)));
	const FLT x909 = (x906 * x266) + (x908 * x155);
	const FLT x910 =
		x904 + (-1 * x312 *
				((x909 * x310) + (x906 * x309) + x903 + (-1 * x905 * x306) +
				 (-1 * x302 *
				  ((x905 * x297) +
				   (-1 * x282 *
					((x909 * x155) +
					 (x155 * (x909 + (x906 * x256) +
							  (x155 * ((x906 * x257) + x908 + (x155 * (x907 + (-1 * x906 * x262) + (x906 * x260))))))) +
					 (x906 * x270) + (x906 * x271)))))));
	const FLT x911 = -1 * dt * x428;
	const FLT x912 = x894 + x911;
	const FLT x913 = x900 + x901;
	const FLT x914 = dt * x433;
	const FLT x915 = dt * x432;
	const FLT x916 = (-1 * x915) + x914;
	const FLT x917 = (x916 * x242) + (x913 * x238);
	const FLT x918 = (-1 * x245 * (x917 + (x912 * x229))) + (x912 * x154);
	const FLT x919 = (x912 * x276) + (-1 * x917 * x286);
	const FLT x920 = ((x913 * x289) + (-1 * x916 * x288)) * x290;
	const FLT x921 = (-1 * x920) + (x919 * x285);
	const FLT x922 = x918 * x258;
	const FLT x923 = (x918 * x264) + (x155 * (x922 + (-1 * x918 * x263)));
	const FLT x924 = (x918 * x266) + (x923 * x155);
	const FLT x925 =
		x920 + (-1 * x312 *
				((x924 * x310) + x919 +
				 (-1 * x302 *
				  ((x921 * x297) +
				   (-1 * x282 *
					((x918 * x270) + (x924 * x155) +
					 (x155 * (x924 + (x918 * x256) +
							  (x155 * (x923 + (x918 * x257) + (x155 * ((-1 * x918 * x262) + x922 + (x918 * x260))))))) +
					 (x918 * x271))))) +
				 (x918 * x309) + (-1 * x921 * x306)));
	const FLT x926 = x914 + x915;
	const FLT x927 = (-1 * x896) + x897;
	const FLT x928 = x893 + dt + x911;
	const FLT x929 = (x928 * x242) + (x927 * x238);
	const FLT x930 = (-1 * x245 * (x929 + (x926 * x229))) + (x926 * x154);
	const FLT x931 = x930 * x258;
	const FLT x932 = (x930 * x264) + (x155 * (x931 + (-1 * x930 * x263)));
	const FLT x933 = (x930 * x266) + (x932 * x155);
	const FLT x934 = (x926 * x276) + (-1 * x929 * x286);
	const FLT x935 = ((x927 * x289) + (-1 * x928 * x288)) * x290;
	const FLT x936 = (-1 * x935) + (x934 * x285);
	const FLT x937 =
		x935 +
		(-1 * x312 *
		 ((x933 * x310) + (x930 * x309) +
		  (-1 * x302 *
		   ((x936 * x297) +
			(-1 * x282 *
			 ((x930 * x270) + (x933 * x155) + (x930 * x271) +
			  (x155 * (x933 + (x930 * x256) +
					   (x155 * ((x930 * x257) + x932 + (x155 * ((-1 * x930 * x262) + x931 + (x930 * x260))))))))))) +
		  x934 + (-1 * x936 * x306)));
	const FLT x938 = x299 * x301;
	const FLT x939 = (x305 + x938) * x312;
	const FLT x940 = ((-1 * x279 * x305) + (-1 * x938 * x279)) * x312;
	const FLT x941 = x312 * (x306 + (x938 * x293));
	const FLT x942 = x274 * x139 * (1 + x284);
	const FLT x943 = x254 * x150;
	const FLT x944 = x943 * x272 * x139;
	const FLT x945 = x944 * x258;
	const FLT x946 = (x944 * x264) + (x155 * (x945 + (-1 * x944 * x263)));
	const FLT x947 = (x944 * x266) + (x946 * x155);
	const FLT x948 = x942 * x285;
	const FLT x949 =
		x312 * ((-1 * x302 *
				 ((-1 * x282 *
				   ((x947 * x155) + (x944 * x271) +
					(x155 * ((x944 * x256) + x947 +
							 (x155 * (x946 + (x944 * x257) + (x155 * ((-1 * x944 * x262) + x945 + (x944 * x260))))))) +
					(x944 * x270))) +
				  (x948 * x297) + (-1 * x272) + (-1 * x295 * x281 * x152))) +
				(x943 * x282 * x255 * x229 * x307) + (-1 * x948 * x306) + x942 + (x947 * x310));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						x313 + (x313 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						x361 + (x361 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						x401 + (x401 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[0]) / sizeof(FLT),
						x427 + (x427 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[1]) / sizeof(FLT),
						x444 + (x444 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[2]) / sizeof(FLT),
						x457 + (x457 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Acc[0]) / sizeof(FLT),
						x478 + (x478 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Acc[1]) / sizeof(FLT),
						x493 + (x493 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Acc[2]) / sizeof(FLT),
						x506 + (x506 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[0]) / sizeof(FLT),
						x585 + (x585 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[1]) / sizeof(FLT),
						x629 + (x629 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[2]) / sizeof(FLT),
						x669 + (x669 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[0]) / sizeof(FLT),
						x683 + (x683 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[1]) / sizeof(FLT),
						x696 + (x696 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[2]) / sizeof(FLT),
						x709 + (x709 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x802 + (x802 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x854 + (x854 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x892 + (x892 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[0]) / sizeof(FLT),
						x910 + (x910 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[1]) / sizeof(FLT),
						x925 + (x925 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[2]) / sizeof(FLT),
						x937 + (x937 * x315));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD0.curve) / sizeof(FLT),
						(-1 * x939 * x315) + (-1 * x939));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD0.gibmag) / sizeof(FLT), sin(x314));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD0.gibpha) / sizeof(FLT), x315);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD0.ogeemag) / sizeof(FLT),
						(-1 * x940 * x315) + (-1 * x940));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD0.ogeephase) / sizeof(FLT),
						(-1 * x941 * x315) + (-1 * x941));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD0.phase) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD0.tilt) / sizeof(FLT),
						(-1 * x949 * x315) + (-1 * x949));
}

// Full version Jacobian of SurviveJointKalmanErrorModel_LightMeas_x_gen2 wrt
// [(*error_model).Lighthouse.AxisAngleRot[0], (*error_model).Lighthouse.AxisAngleRot[1],
// (*error_model).Lighthouse.AxisAngleRot[2], (*error_model).Lighthouse.Pos[0], (*error_model).Lighthouse.Pos[1],
// (*error_model).Lighthouse.Pos[2], (*error_model).Object.Acc[0], (*error_model).Object.Acc[1],
// (*error_model).Object.Acc[2], (*error_model).Object.IMUBias.AccBias[0], (*error_model).Object.IMUBias.AccBias[1],
// (*error_model).Object.IMUBias.AccBias[2], (*error_model).Object.IMUBias.AccScale[0],
// (*error_model).Object.IMUBias.AccScale[1], (*error_model).Object.IMUBias.AccScale[2],
// (*error_model).Object.IMUBias.GyroBias[0], (*error_model).Object.IMUBias.GyroBias[1],
// (*error_model).Object.IMUBias.GyroBias[2], (*error_model).Object.IMUBias.IMUCorrection[0],
// (*error_model).Object.IMUBias.IMUCorrection[1], (*error_model).Object.IMUBias.IMUCorrection[2],
// (*error_model).Object.Pose.AxisAngleRot[0], (*error_model).Object.Pose.AxisAngleRot[1],
// (*error_model).Object.Pose.AxisAngleRot[2], (*error_model).Object.Pose.Pos[0], (*error_model).Object.Pose.Pos[1],
// (*error_model).Object.Pose.Pos[2], (*error_model).Object.Velocity.AxisAngleRot[0],
// (*error_model).Object.Velocity.AxisAngleRot[1], (*error_model).Object.Velocity.AxisAngleRot[2],
// (*error_model).Object.Velocity.Pos[0], (*error_model).Object.Velocity.Pos[1], (*error_model).Object.Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3346a0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3387c0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3349d0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3382e0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c334640>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3382b0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c338430>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3381c0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3383d0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c338100>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c334be0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3385b0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3348e0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c338700>]

static inline void SurviveJointKalmanErrorModel_LightMeas_x_gen2_jac_error_model_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveJointKalmanModel *_x0,
	const SurviveJointKalmanErrorModel *error_model, const FLT *sensor_pt) {
	if (hx != 0) {
		hx->data[0] = SurviveJointKalmanErrorModel_LightMeas_x_gen2(dt, _x0, error_model, sensor_pt);
	}
	if (Hx != 0) {
		SurviveJointKalmanErrorModel_LightMeas_x_gen2_jac_error_model(Hx, dt, _x0, error_model, sensor_pt);
	}
}
// Jacobian of SurviveJointKalmanErrorModel_LightMeas_x_gen2 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]
static inline void
SurviveJointKalmanErrorModel_LightMeas_x_gen2_jac_sensor_pt(CnMat *Hx, const FLT dt, const SurviveJointKalmanModel *_x0,
															const SurviveJointKalmanErrorModel *error_model,
															const FLT *sensor_pt) {
	const FLT x0 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[1];
	const FLT x1 = cos(x0);
	const FLT x2 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[2];
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[0];
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = cos(x4);
	const FLT x8 = sin(x0);
	const FLT x9 = cos(x2);
	const FLT x10 = x8 * x9;
	const FLT x11 = (x7 * x10) + (x1 * x6);
	const FLT x12 = x1 * x9;
	const FLT x13 = (x7 * x12) + (x6 * x8);
	const FLT x14 = x3 * x7;
	const FLT x15 = (x1 * x14) + (-1 * x5 * x10);
	const FLT x16 = (x5 * x12) + (-1 * x8 * x14);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x15 * x15) + (x11 * x11) + (x13 * x13));
	const FLT x18 = x15 * x17;
	const FLT x19 = x11 * x17;
	const FLT x20 = x13 * x17;
	const FLT x21 = x17 * x16;
	const FLT x22 = (x20 * (*_x0).Object.Pose.Rot[0]) + (-1 * x21 * (*_x0).Object.Pose.Rot[1]) +
					(-1 * x18 * (*_x0).Object.Pose.Rot[3]) + (-1 * x19 * (*_x0).Object.Pose.Rot[2]);
	const FLT x23 = (*_x0).Object.Velocity.AxisAngleRot[2] + (*error_model).Object.Velocity.AxisAngleRot[2];
	const FLT x24 = dt * dt;
	const FLT x25 = x24 * (x23 * x23);
	const FLT x26 = (*_x0).Object.Velocity.AxisAngleRot[0] + (*error_model).Object.Velocity.AxisAngleRot[0];
	const FLT x27 = x24 * (x26 * x26);
	const FLT x28 = (*_x0).Object.Velocity.AxisAngleRot[1] + (*error_model).Object.Velocity.AxisAngleRot[1];
	const FLT x29 = x24 * (x28 * x28);
	const FLT x30 = 1e-10 + x29 + x25 + x27;
	const FLT x31 = sqrt(x30);
	const FLT x32 = 0.5 * x31;
	const FLT x33 = sin(x32);
	const FLT x34 = (1. / x30) * (x33 * x33);
	const FLT x35 = cos(x32);
	const FLT x36 = 1. / sqrt((x34 * x27) + (x35 * x35) + (x34 * x25) + (x34 * x29));
	const FLT x37 = dt * (1. / x31) * x33 * x36;
	const FLT x38 = x37 * x23;
	const FLT x39 = x38 * x22;
	const FLT x40 = (x19 * (*_x0).Object.Pose.Rot[1]) + (x18 * (*_x0).Object.Pose.Rot[0]) +
					(x20 * (*_x0).Object.Pose.Rot[3]) + (-1 * x21 * (*_x0).Object.Pose.Rot[2]);
	const FLT x41 = x36 * x35;
	const FLT x42 = x40 * x41;
	const FLT x43 = (x20 * (*_x0).Object.Pose.Rot[1]) + (x21 * (*_x0).Object.Pose.Rot[0]) +
					(-1 * x19 * (*_x0).Object.Pose.Rot[3]) + (x18 * (*_x0).Object.Pose.Rot[2]);
	const FLT x44 = x37 * x28;
	const FLT x45 = x43 * x44;
	const FLT x46 = (-1 * x18 * (*_x0).Object.Pose.Rot[1]) + (x19 * (*_x0).Object.Pose.Rot[0]) +
					(x20 * (*_x0).Object.Pose.Rot[2]) + (x21 * (*_x0).Object.Pose.Rot[3]);
	const FLT x47 = x37 * x26;
	const FLT x48 = x46 * x47;
	const FLT x49 = x48 + (-1 * x45) + x39 + x42;
	const FLT x50 = x41 * x46;
	const FLT x51 = x43 * x38;
	const FLT x52 = x40 * x47;
	const FLT x53 = x44 * x22;
	const FLT x54 = x52 + (-1 * x53) + (-1 * x50) + (-1 * x51);
	const FLT x55 = (-1 * x52) + x51 + x53 + x50;
	const FLT x56 = 2 * x55;
	const FLT x57 = 1 + (x54 * x56) + (-2 * (x49 * x49));
	const FLT x58 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[2];
	const FLT x59 = cos(x58);
	const FLT x60 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[0];
	const FLT x61 = sin(x60);
	const FLT x62 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[1];
	const FLT x63 = sin(x62);
	const FLT x64 = x63 * x61;
	const FLT x65 = cos(x60);
	const FLT x66 = cos(x62);
	const FLT x67 = sin(x58);
	const FLT x68 = x67 * x66;
	const FLT x69 = (x68 * x65) + (-1 * x64 * x59);
	const FLT x70 = x63 * x65;
	const FLT x71 = (x70 * x59) + (x61 * x68);
	const FLT x72 = x66 * x59;
	const FLT x73 = (x72 * x65) + (x64 * x67);
	const FLT x74 = (x72 * x61) + (-1 * x70 * x67);
	const FLT x75 = 1. / sqrt((x73 * x73) + (x69 * x69) + (x74 * x74) + (x71 * x71));
	const FLT x76 = x73 * x75;
	const FLT x77 = x75 * x74;
	const FLT x78 = x75 * (*_x0).Lighthouse.Rot[0];
	const FLT x79 = x71 * x75;
	const FLT x80 = (x79 * (*_x0).Lighthouse.Rot[1]) + (x76 * (*_x0).Lighthouse.Rot[3]) + (x78 * x69) +
					(-1 * x77 * (*_x0).Lighthouse.Rot[2]);
	const FLT x81 = x75 * x69;
	const FLT x82 = (-1 * x81 * (*_x0).Lighthouse.Rot[1]) + (x77 * (*_x0).Lighthouse.Rot[3]) + (x71 * x78) +
					(x76 * (*_x0).Lighthouse.Rot[2]);
	const FLT x83 = (-1 * x77 * (*_x0).Lighthouse.Rot[1]) + (x73 * x78) + (-1 * x81 * (*_x0).Lighthouse.Rot[3]) +
					(-1 * x79 * (*_x0).Lighthouse.Rot[2]);
	const FLT x84 = (x76 * (*_x0).Lighthouse.Rot[1]) + (-1 * x79 * (*_x0).Lighthouse.Rot[3]) +
					(x81 * (*_x0).Lighthouse.Rot[2]) + (x78 * x74);
	const FLT x85 = 1. / sqrt((x84 * x84) + (x83 * x83) + (x80 * x80) + (x82 * x82));
	const FLT x86 = x82 * x85;
	const FLT x87 = x46 * x38;
	const FLT x88 = x40 * x44;
	const FLT x89 = x41 * x43;
	const FLT x90 = x47 * x22;
	const FLT x91 = x89 + x90 + (-1 * x87) + x88;
	const FLT x92 = 2 * x91;
	const FLT x93 = (-1 * x43 * x47) + (-1 * x40 * x38) + (x41 * x22) + (-1 * x44 * x46);
	const FLT x94 = 2 * x49;
	const FLT x95 = x93 * x94;
	const FLT x96 = x95 + (-1 * x54 * x92);
	const FLT x97 = x84 * x85;
	const FLT x98 = x56 * x93;
	const FLT x99 = (x91 * x94) + (-1 * x98);
	const FLT x100 = x83 * x85;
	const FLT x101 = (x99 * x100) + (x86 * x57) + (-1 * x97 * x96);
	const FLT x102 = 2 * x86;
	const FLT x103 = x80 * x85;
	const FLT x104 = (x99 * x97) + (-1 * x57 * x103) + (x96 * x100);
	const FLT x105 = 2 * x103;
	const FLT x106 = x57 + (-1 * x101 * x102) + (x105 * x104);
	const FLT x107 = (-1 * x55 * sensor_pt[0]) + (x93 * sensor_pt[2]) + (x91 * sensor_pt[1]);
	const FLT x108 = (-1 * x49 * sensor_pt[1]) + (x93 * sensor_pt[0]) + (x55 * sensor_pt[2]);
	const FLT x109 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x110 = (*error_model).Object.Pose.Pos[1] + (*_x0).Object.Pose.Pos[1] +
					 (2 * ((x49 * x108) + (-1 * x91 * x107))) +
					 (x109 * ((*_x0).Object.Acc[1] + (*error_model).Object.Acc[1])) + sensor_pt[1] +
					 (dt * ((*_x0).Object.Velocity.Pos[1] + (*error_model).Object.Velocity.Pos[1]));
	const FLT x111 = (-1 * x91 * sensor_pt[2]) + (x93 * sensor_pt[1]) + (x49 * sensor_pt[0]);
	const FLT x112 = (*_x0).Object.Pose.Pos[0] + (2 * ((x55 * x107) + (-1 * x49 * x111))) +
					 (*error_model).Object.Pose.Pos[0] +
					 (dt * ((*_x0).Object.Velocity.Pos[0] + (*error_model).Object.Velocity.Pos[0])) +
					 (x109 * ((*_x0).Object.Acc[0] + (*error_model).Object.Acc[0])) + sensor_pt[0];
	const FLT x113 = (dt * ((*_x0).Object.Velocity.Pos[2] + (*error_model).Object.Velocity.Pos[2])) +
					 (*error_model).Object.Pose.Pos[2] + sensor_pt[2] + (2 * ((x91 * x111) + (-1 * x55 * x108))) +
					 (x109 * ((*_x0).Object.Acc[2] + (*error_model).Object.Acc[2])) + (*_x0).Object.Pose.Pos[2];
	const FLT x114 = (x97 * x113) + (x100 * x110) + (-1 * x103 * x112);
	const FLT x115 = (-1 * x97 * x110) + (x86 * x112) + (x100 * x113);
	const FLT x116 = (*_x0).Lighthouse.Pos[1] + (*error_model).Lighthouse.Pos[1];
	const FLT x117 = (*_x0).Lighthouse.Pos[0] + (*error_model).Lighthouse.Pos[0];
	const FLT x118 = (*_x0).Lighthouse.Pos[2] + (*error_model).Lighthouse.Pos[2];
	const FLT x119 = (x97 * x118) + (x100 * x116) + (-1 * x103 * x117);
	const FLT x120 = (x86 * x117) + (x100 * x118) + (-1 * x97 * x116);
	const FLT x121 =
		x112 + (2 * ((-1 * x86 * x115) + (x103 * x114))) + (-1 * (x117 + (2 * ((-1 * x86 * x120) + (x103 * x119)))));
	const FLT x122 = 2 * x121;
	const FLT x123 = 2 * x97;
	const FLT x124 = (-1 * x86 * x99) + (x96 * x103) + (x57 * x100);
	const FLT x125 = x99 + (-1 * x104 * x123) + (x102 * x124);
	const FLT x126 = (x103 * x116) + (-1 * x86 * x118) + (x100 * x117);
	const FLT x127 = (x103 * x110) + (x100 * x112) + (-1 * x86 * x113);
	const FLT x128 =
		(-1 * (x118 + (2 * ((-1 * x97 * x119) + (x86 * x126))))) + x113 + (2 * ((-1 * x97 * x114) + (x86 * x127)));
	const FLT x129 = 2 * x128;
	const FLT x130 = (x125 * x129) + (x106 * x122);
	const FLT x131 = 0.523598775598299 + (*error_model).BSD0.tilt + (*_x0).BSD0.tilt;
	const FLT x132 = tan(x131);
	const FLT x133 = x121 * x121;
	const FLT x134 = x133 + (x128 * x128);
	const FLT x135 =
		(-1 * (x116 + (2 * ((-1 * x103 * x126) + (x97 * x120))))) + x110 + (2 * ((-1 * x103 * x127) + (x97 * x115)));
	const FLT x136 = 1.0 / 2.0 * x135;
	const FLT x137 = (1. / (x134 * sqrt(x134))) * x132 * x136;
	const FLT x138 = x96 + (x101 * x123) + (-1 * x105 * x124);
	const FLT x139 = (1. / sqrt(x134)) * x132;
	const FLT x140 = (x138 * x139) + (-1 * x130 * x137);
	const FLT x141 = x135 * x135;
	const FLT x142 = 1. / x134;
	const FLT x143 = 1. / sqrt(1 + (-1 * (x132 * x132) * x142 * x141));
	const FLT x144 = 1. / x121;
	const FLT x145 = x128 * (1. / x133);
	const FLT x146 = x133 * x142;
	const FLT x147 = ((x106 * x145) + (-1 * x125 * x144)) * x146;
	const FLT x148 = (*error_model).BSD0.ogeemag + (*_x0).BSD0.ogeemag;
	const FLT x149 = atan2(-1 * x128, x121);
	const FLT x150 = x135 * x139;
	const FLT x151 = asin(x150) + (-1 * x149) + (-1 * (*error_model).BSD0.ogeephase) + (-1 * (*_x0).BSD0.ogeephase);
	const FLT x152 = x148 * cos(x151);
	const FLT x153 = x152 * ((-1 * x147) + (x140 * x143));
	const FLT x154 = x134 + x141;
	const FLT x155 = cos(x131);
	const FLT x156 = 1. / x155;
	const FLT x157 = (1. / sqrt(x154)) * x156;
	const FLT x158 = asin(x135 * x157);
	const FLT x159 = 8.0108022e-06 * x158;
	const FLT x160 = -8.0108022e-06 + (-1 * x159);
	const FLT x161 = 0.0028679863 + (x160 * x158);
	const FLT x162 = 5.3685255e-06 + (x161 * x158);
	const FLT x163 = 0.0076069798 + (x162 * x158);
	const FLT x164 = x163 * x158;
	const FLT x165 = -8.0108022e-06 + (-1.60216044e-05 * x158);
	const FLT x166 = x161 + (x165 * x158);
	const FLT x167 = x162 + (x166 * x158);
	const FLT x168 = x163 + (x167 * x158);
	const FLT x169 = (x168 * x158) + x164;
	const FLT x170 = sin(x131);
	const FLT x171 = (*_x0).BSD0.curve + (*error_model).BSD0.curve + (-1 * x148 * sin(x151));
	const FLT x172 = x170 * x171;
	const FLT x173 = x155 + (-1 * x169 * x172);
	const FLT x174 = 1. / x173;
	const FLT x175 = x158 * x158;
	const FLT x176 = x163 * x174 * x175;
	const FLT x177 = 1. / sqrt(1 + (-1 * x141 * (1. / x154) * (1. / (x155 * x155))));
	const FLT x178 = 2 * x135;
	const FLT x179 = x136 * (1. / (x154 * sqrt(x154))) * x156;
	const FLT x180 = x177 * ((-1 * x179 * (x130 + (x178 * x138))) + (x138 * x157));
	const FLT x181 = x160 * x180;
	const FLT x182 = 2.40324066e-05 * x158;
	const FLT x183 = (x161 * x180) + (x158 * (x181 + (-1 * x180 * x159)));
	const FLT x184 = (x162 * x180) + (x183 * x158);
	const FLT x185 = x169 * x170;
	const FLT x186 = x171 * x175;
	const FLT x187 = x163 * (1. / (x173 * x173)) * x186;
	const FLT x188 = x174 * x186;
	const FLT x189 = 2 * x164 * x171 * x174;
	const FLT x190 = x150 + (x163 * x188);
	const FLT x191 = 1. / sqrt(1 + (-1 * (x190 * x190)));
	const FLT x192 =
		x147 + (-1 * x191 *
				((x180 * x189) + (-1 * x176 * x153) + x140 + (x188 * x184) +
				 (-1 * x187 *
				  ((x185 * x153) +
				   (-1 * x172 *
					((x184 * x158) + (x163 * x180) +
					 (x158 * (x184 + (x167 * x180) +
							  (x158 * (x183 + (x166 * x180) + (x158 * ((-1 * x180 * x182) + x181 + (x165 * x180))))))) +
					 (x168 * x180)))))));
	const FLT x193 = ((*error_model).BSD0.gibmag + (*_x0).BSD0.gibmag) *
					 cos((-1 * asin(x190)) + x149 + (*error_model).BSD0.gibpha + (*_x0).BSD0.gibpha);
	const FLT x194 = (x56 * x91) + (-1 * x95);
	const FLT x195 = x45 + (-1 * x48) + (-1 * x42) + (-1 * x39);
	const FLT x196 = 1 + (x94 * x195) + (-2 * (x91 * x91));
	const FLT x197 = x85 * x196;
	const FLT x198 = x93 * x92;
	const FLT x199 = x198 + (-1 * x56 * x195);
	const FLT x200 = (x86 * x194) + (x100 * x199) + (-1 * x84 * x197);
	const FLT x201 = (-1 * x86 * x199) + (x103 * x196) + (x100 * x194);
	const FLT x202 = x196 + (x200 * x123) + (-1 * x201 * x105);
	const FLT x203 = (-1 * x103 * x194) + (x83 * x197) + (x97 * x199);
	const FLT x204 = x194 + (-1 * x200 * x102) + (x203 * x105);
	const FLT x205 = x199 + (-1 * x203 * x123) + (x201 * x102);
	const FLT x206 = (x205 * x129) + (x204 * x122);
	const FLT x207 = x177 * ((-1 * x179 * (x206 + (x202 * x178))) + (x202 * x157));
	const FLT x208 = x207 * x160;
	const FLT x209 = (x207 * x161) + (x158 * (x208 + (-1 * x207 * x159)));
	const FLT x210 = (x207 * x162) + (x209 * x158);
	const FLT x211 = (x202 * x139) + (-1 * x206 * x137);
	const FLT x212 = ((x204 * x145) + (-1 * x205 * x144)) * x146;
	const FLT x213 = (-1 * x212) + (x211 * x143);
	const FLT x214 = x185 * x152;
	const FLT x215 = x176 * x152;
	const FLT x216 =
		x212 + (-1 * x191 *
				(x211 + (x210 * x188) +
				 (-1 * x187 *
				  ((x213 * x214) +
				   (-1 * x172 *
					((x210 * x158) + (x207 * x163) +
					 (x158 * (x210 + (x207 * x167) +
							  (x158 * (x209 + (x207 * x166) + (x158 * ((-1 * x207 * x182) + (x207 * x165) + x208)))))) +
					 (x207 * x168))))) +
				 (x207 * x189) + (-1 * x213 * x215)));
	const FLT x217 = (x55 * x94) + (-1 * x198);
	const FLT x218 = (-1 * x89) + (-1 * x88) + (-1 * x90) + x87;
	const FLT x219 = x98 + (-1 * x94 * x218);
	const FLT x220 = 1 + (x92 * x218) + (-2 * (x55 * x55));
	const FLT x221 = x85 * x220;
	const FLT x222 = (-1 * x82 * x221) + (x217 * x103) + (x219 * x100);
	const FLT x223 = (x220 * x100) + (x86 * x219) + (-1 * x97 * x217);
	const FLT x224 = x217 + (-1 * x222 * x105) + (x223 * x123);
	const FLT x225 = (-1 * x219 * x103) + (x84 * x221) + (x217 * x100);
	const FLT x226 = x219 + (-1 * x223 * x102) + (x225 * x105);
	const FLT x227 = x220 + (-1 * x225 * x123) + (x222 * x102);
	const FLT x228 = (x227 * x129) + (x226 * x122);
	const FLT x229 = x177 * ((-1 * x179 * (x228 + (x224 * x178))) + (x224 * x157));
	const FLT x230 = (x224 * x139) + (-1 * x228 * x137);
	const FLT x231 = ((x226 * x145) + (-1 * x227 * x144)) * x146;
	const FLT x232 = (-1 * x231) + (x230 * x143);
	const FLT x233 = x229 * x160;
	const FLT x234 = (x229 * x161) + (x158 * (x233 + (-1 * x229 * x159)));
	const FLT x235 = (x229 * x162) + (x234 * x158);
	const FLT x236 =
		x231 + (-1 * x191 *
				(x230 +
				 (-1 * x187 *
				  ((x214 * x232) +
				   (-1 * x172 *
					((x235 * x158) + (x229 * x163) +
					 (x158 * ((x229 * x167) + x235 +
							  (x158 * (x234 + (x229 * x166) + (x158 * (x233 + (-1 * x229 * x182) + (x229 * x165))))))) +
					 (x229 * x168))))) +
				 (x235 * x188) + (x229 * x189) + (-1 * x215 * x232)));
	cnMatrixOptionalSet(Hx, 0, 0, x192 + (x192 * x193));
	cnMatrixOptionalSet(Hx, 0, 1, x216 + (x216 * x193));
	cnMatrixOptionalSet(Hx, 0, 2, x236 + (x236 * x193));
}

// Full version Jacobian of SurviveJointKalmanErrorModel_LightMeas_x_gen2 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]

static inline void SurviveJointKalmanErrorModel_LightMeas_x_gen2_jac_sensor_pt_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveJointKalmanModel *_x0,
	const SurviveJointKalmanErrorModel *error_model, const FLT *sensor_pt) {
	if (hx != 0) {
		hx->data[0] = SurviveJointKalmanErrorModel_LightMeas_x_gen2(dt, _x0, error_model, sensor_pt);
	}
	if (Hx != 0) {
		SurviveJointKalmanErrorModel_LightMeas_x_gen2_jac_sensor_pt(Hx, dt, _x0, error_model, sensor_pt);
	}
}
static inline FLT SurviveJointKalmanErrorModel_LightMeas_y_gen2(const FLT dt, const SurviveJointKalmanModel *_x0,
																const SurviveJointKalmanErrorModel *error_model,
																const FLT *sensor_pt) {
	const FLT x0 = (*_x0).Lighthouse.Pos[2] + (*error_model).Lighthouse.Pos[2];
	const FLT x1 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[2];
	const FLT x2 = cos(x1);
	const FLT x3 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[0];
	const FLT x4 = sin(x3);
	const FLT x5 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[1];
	const FLT x6 = sin(x5);
	const FLT x7 = x4 * x6;
	const FLT x8 = sin(x1);
	const FLT x9 = cos(x5);
	const FLT x10 = cos(x3);
	const FLT x11 = x9 * x10;
	const FLT x12 = (x8 * x11) + (-1 * x2 * x7);
	const FLT x13 = x4 * x9;
	const FLT x14 = x6 * x10;
	const FLT x15 = (x2 * x14) + (x8 * x13);
	const FLT x16 = (x2 * x11) + (x8 * x7);
	const FLT x17 = (x2 * x13) + (-1 * x8 * x14);
	const FLT x18 = 1. / sqrt((x16 * x16) + (x17 * x17) + (x12 * x12) + (x15 * x15));
	const FLT x19 = x18 * x16;
	const FLT x20 = x18 * x17;
	const FLT x21 = x18 * (*_x0).Lighthouse.Rot[0];
	const FLT x22 = x18 * (*_x0).Lighthouse.Rot[1];
	const FLT x23 =
		(x22 * x15) + (x21 * x12) + (x19 * (*_x0).Lighthouse.Rot[3]) + (-1 * x20 * (*_x0).Lighthouse.Rot[2]);
	const FLT x24 =
		(-1 * x22 * x12) + (x20 * (*_x0).Lighthouse.Rot[3]) + (x21 * x15) + (x19 * (*_x0).Lighthouse.Rot[2]);
	const FLT x25 = x12 * x18;
	const FLT x26 = x15 * x18;
	const FLT x27 =
		(-1 * x22 * x17) + (x21 * x16) + (-1 * x25 * (*_x0).Lighthouse.Rot[3]) + (-1 * x26 * (*_x0).Lighthouse.Rot[2]);
	const FLT x28 =
		(x22 * x16) + (-1 * x26 * (*_x0).Lighthouse.Rot[3]) + (x25 * (*_x0).Lighthouse.Rot[2]) + (x21 * x17);
	const FLT x29 = 1. / sqrt((x27 * x27) + (x28 * x28) + (x23 * x23) + (x24 * x24));
	const FLT x30 = x24 * x29;
	const FLT x31 = (*_x0).Lighthouse.Pos[0] + (*error_model).Lighthouse.Pos[0];
	const FLT x32 = x29 * x27;
	const FLT x33 = (*_x0).Lighthouse.Pos[1] + (*error_model).Lighthouse.Pos[1];
	const FLT x34 = x23 * x29;
	const FLT x35 = (-1 * x0 * x30) + (x34 * x33) + (x32 * x31);
	const FLT x36 = x28 * x29;
	const FLT x37 = (x0 * x36) + (x32 * x33) + (-1 * x31 * x34);
	const FLT x38 = (*_x0).Object.Velocity.AxisAngleRot[2] + (*error_model).Object.Velocity.AxisAngleRot[2];
	const FLT x39 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[1];
	const FLT x40 = cos(x39);
	const FLT x41 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[0];
	const FLT x42 = sin(x41);
	const FLT x43 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[2];
	const FLT x44 = sin(x43);
	const FLT x45 = x42 * x44;
	const FLT x46 = cos(x43);
	const FLT x47 = sin(x39);
	const FLT x48 = cos(x41);
	const FLT x49 = x47 * x48;
	const FLT x50 = (x46 * x49) + (x40 * x45);
	const FLT x51 = x40 * x48;
	const FLT x52 = (x51 * x46) + (x45 * x47);
	const FLT x53 = x42 * x46;
	const FLT x54 = (x51 * x44) + (-1 * x53 * x47);
	const FLT x55 = (x53 * x40) + (-1 * x44 * x49);
	const FLT x56 = 1. / sqrt((x55 * x55) + (x54 * x54) + (x50 * x50) + (x52 * x52));
	const FLT x57 = x52 * x56;
	const FLT x58 = x56 * x55;
	const FLT x59 = x54 * x56;
	const FLT x60 = x50 * x56;
	const FLT x61 = (x60 * (*_x0).Object.Pose.Rot[1]) + (x59 * (*_x0).Object.Pose.Rot[0]) +
					(x57 * (*_x0).Object.Pose.Rot[3]) + (-1 * x58 * (*_x0).Object.Pose.Rot[2]);
	const FLT x62 = dt * dt;
	const FLT x63 = x62 * (x38 * x38);
	const FLT x64 = (*_x0).Object.Velocity.AxisAngleRot[0] + (*error_model).Object.Velocity.AxisAngleRot[0];
	const FLT x65 = (x64 * x64) * x62;
	const FLT x66 = (*_x0).Object.Velocity.AxisAngleRot[1] + (*error_model).Object.Velocity.AxisAngleRot[1];
	const FLT x67 = x62 * (x66 * x66);
	const FLT x68 = 1e-10 + x67 + x63 + x65;
	const FLT x69 = sqrt(x68);
	const FLT x70 = 0.5 * x69;
	const FLT x71 = sin(x70);
	const FLT x72 = (x71 * x71) * (1. / x68);
	const FLT x73 = cos(x70);
	const FLT x74 = 1. / sqrt((x73 * x73) + (x72 * x63) + (x72 * x65) + (x72 * x67));
	const FLT x75 = dt * x71 * x74 * (1. / x69);
	const FLT x76 = x75 * x61;
	const FLT x77 = (x60 * (*_x0).Object.Pose.Rot[0]) + (-1 * x59 * (*_x0).Object.Pose.Rot[1]) +
					(x57 * (*_x0).Object.Pose.Rot[2]) + (x58 * (*_x0).Object.Pose.Rot[3]);
	const FLT x78 = x75 * x77;
	const FLT x79 = (x57 * (*_x0).Object.Pose.Rot[0]) + (-1 * x59 * (*_x0).Object.Pose.Rot[3]) +
					(-1 * x58 * (*_x0).Object.Pose.Rot[1]) + (-1 * x60 * (*_x0).Object.Pose.Rot[2]);
	const FLT x80 = x73 * x74;
	const FLT x81 = (x57 * (*_x0).Object.Pose.Rot[1]) + (x58 * (*_x0).Object.Pose.Rot[0]) +
					(-1 * x60 * (*_x0).Object.Pose.Rot[3]) + (x59 * (*_x0).Object.Pose.Rot[2]);
	const FLT x82 = x81 * x75;
	const FLT x83 = (-1 * x82 * x64) + (-1 * x76 * x38) + (x80 * x79) + (-1 * x78 * x66);
	const FLT x84 = x75 * x38;
	const FLT x85 = (-1 * x82 * x66) + (x84 * x79) + (x78 * x64) + (x80 * x61);
	const FLT x86 = x79 * x75;
	const FLT x87 = (x86 * x64) + (x80 * x81) + (-1 * x84 * x77) + (x76 * x66);
	const FLT x88 = (x83 * sensor_pt[1]) + (-1 * x87 * sensor_pt[2]) + (x85 * sensor_pt[0]);
	const FLT x89 = (-1 * x76 * x64) + (x86 * x66) + (x82 * x38) + (x80 * x77);
	const FLT x90 = (-1 * x89 * sensor_pt[0]) + (x83 * sensor_pt[2]) + (x87 * sensor_pt[1]);
	const FLT x91 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x92 = (*error_model).Object.Pose.Pos[0] + (*_x0).Object.Pose.Pos[0] +
					(x91 * ((*_x0).Object.Acc[0] + (*error_model).Object.Acc[0])) +
					(2 * ((x89 * x90) + (-1 * x88 * x85))) + sensor_pt[0] +
					(dt * ((*_x0).Object.Velocity.Pos[0] + (*error_model).Object.Velocity.Pos[0]));
	const FLT x93 = (-1 * x85 * sensor_pt[1]) + (x83 * sensor_pt[0]) + (x89 * sensor_pt[2]);
	const FLT x94 = (dt * ((*_x0).Object.Velocity.Pos[2] + (*error_model).Object.Velocity.Pos[2])) +
					(*error_model).Object.Pose.Pos[2] + sensor_pt[2] +
					(x91 * ((*_x0).Object.Acc[2] + (*error_model).Object.Acc[2])) +
					(2 * ((x88 * x87) + (-1 * x89 * x93))) + (*_x0).Object.Pose.Pos[2];
	const FLT x95 = (*_x0).Object.Pose.Pos[1] + (2 * ((x85 * x93) + (-1 * x87 * x90))) +
					(*error_model).Object.Pose.Pos[1] + sensor_pt[1] +
					(x91 * ((*_x0).Object.Acc[1] + (*error_model).Object.Acc[1])) +
					(dt * ((*_x0).Object.Velocity.Pos[1] + (*error_model).Object.Velocity.Pos[1]));
	const FLT x96 = x29 * ((x95 * x34) + (x92 * x32) + (-1 * x94 * x30));
	const FLT x97 = (x94 * x36) + (x95 * x32) + (-1 * x92 * x34);
	const FLT x98 = x94 + (-1 * (x0 + (2 * ((-1 * x36 * x37) + (x30 * x35))))) + (2 * ((-1 * x97 * x36) + (x96 * x24)));
	const FLT x99 = (x92 * x30) + (-1 * x95 * x36) + (x94 * x32);
	const FLT x100 = (x30 * x31) + (x0 * x32) + (-1 * x33 * x36);
	const FLT x101 =
		x92 + (2 * ((-1 * x99 * x30) + (x97 * x34))) + (-1 * (x31 + (2 * ((-1 * x30 * x100) + (x34 * x37)))));
	const FLT x102 = atan2(-1 * x98, x101);
	const FLT x103 =
		x95 + (-1 * (x33 + (2 * ((-1 * x34 * x35) + (x36 * x100))))) + (2 * ((-1 * x96 * x23) + (x99 * x36)));
	const FLT x104 = (x101 * x101) + (x98 * x98);
	const FLT x105 = 0.523598775598299 + (-1 * (*error_model).BSD1.tilt) + (-1 * (*_x0).BSD1.tilt);
	const FLT x106 = -1 * x103 * (1. / sqrt(x104)) * tan(x105);
	const FLT x107 = (*_x0).BSD1.curve +
					 (((*error_model).BSD1.ogeemag + (*_x0).BSD1.ogeemag) *
					  sin((-1 * asin(x106)) + (*error_model).BSD1.ogeephase + x102 + (*_x0).BSD1.ogeephase)) +
					 (*error_model).BSD1.curve;
	const FLT x108 = cos(x105);
	const FLT x109 = asin(x103 * (1. / x108) * (1. / sqrt(x104 + (x103 * x103))));
	const FLT x110 = 0.0028679863 + (x109 * (-8.0108022e-06 + (-8.0108022e-06 * x109)));
	const FLT x111 = 5.3685255e-06 + (x109 * x110);
	const FLT x112 = 0.0076069798 + (x109 * x111);
	const FLT x113 =
		(-1 * asin(x106 +
				   ((x109 * x109) * x107 * x112 *
					(1. / (x108 +
						   (x107 * sin(x105) *
							((x109 * (x112 + (x109 * (x111 + (x109 * (x110 + (x109 * (-8.0108022e-06 +
																					  (-1.60216044e-05 * x109))))))))) +
							 (x109 * x112)))))))) +
		x102;
	return -1.5707963267949 + x113 + (-1 * ((*error_model).BSD1.phase + (*_x0).BSD1.phase)) +
		   (sin(x113 + (*error_model).BSD1.gibpha + (*_x0).BSD1.gibpha) *
			((*error_model).BSD1.gibmag + (*_x0).BSD1.gibmag));
}

// Jacobian of SurviveJointKalmanErrorModel_LightMeas_y_gen2 wrt [(*_x0).Lighthouse.Pos[0], (*_x0).Lighthouse.Pos[1],
// (*_x0).Lighthouse.Pos[2], (*_x0).Lighthouse.Rot[0], (*_x0).Lighthouse.Rot[1], (*_x0).Lighthouse.Rot[2],
// (*_x0).Lighthouse.Rot[3], (*_x0).Object.Acc[0], (*_x0).Object.Acc[1], (*_x0).Object.Acc[2],
// (*_x0).Object.IMUBias.AccBias[0], (*_x0).Object.IMUBias.AccBias[1], (*_x0).Object.IMUBias.AccBias[2],
// (*_x0).Object.IMUBias.AccScale[0], (*_x0).Object.IMUBias.AccScale[1], (*_x0).Object.IMUBias.AccScale[2],
// (*_x0).Object.IMUBias.GyroBias[0], (*_x0).Object.IMUBias.GyroBias[1], (*_x0).Object.IMUBias.GyroBias[2],
// (*_x0).Object.IMUBias.IMUCorrection[0], (*_x0).Object.IMUBias.IMUCorrection[1],
// (*_x0).Object.IMUBias.IMUCorrection[2], (*_x0).Object.IMUBias.IMUCorrection[3], (*_x0).Object.Pose.Pos[0],
// (*_x0).Object.Pose.Pos[1], (*_x0).Object.Pose.Pos[2], (*_x0).Object.Pose.Rot[0], (*_x0).Object.Pose.Rot[1],
// (*_x0).Object.Pose.Rot[2], (*_x0).Object.Pose.Rot[3], (*_x0).Object.Velocity.AxisAngleRot[0],
// (*_x0).Object.Velocity.AxisAngleRot[1], (*_x0).Object.Velocity.AxisAngleRot[2], (*_x0).Object.Velocity.Pos[0],
// (*_x0).Object.Velocity.Pos[1], (*_x0).Object.Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f4d1c38e7c0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c394af0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c394460>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3943d0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3944c0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c394430>, <cnkalman.codegen.WrapMember object at 0x7f4d1c394130>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3942e0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c394160>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c394370>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38e0a0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c394250>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38e850>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c394070>]
static inline void SurviveJointKalmanErrorModel_LightMeas_y_gen2_jac_x0(CnMat *Hx, const FLT dt,
																		const SurviveJointKalmanModel *_x0,
																		const SurviveJointKalmanErrorModel *error_model,
																		const FLT *sensor_pt) {
	const FLT x0 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[1];
	const FLT x1 = sin(x0);
	const FLT x2 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[0];
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[2];
	const FLT x5 = cos(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = cos(x2);
	const FLT x8 = cos(x0);
	const FLT x9 = sin(x4);
	const FLT x10 = x8 * x9;
	const FLT x11 = (x7 * x10) + (-1 * x1 * x6);
	const FLT x12 = x5 * x7;
	const FLT x13 = (x1 * x12) + (x3 * x10);
	const FLT x14 = x1 * x9;
	const FLT x15 = (x8 * x12) + (x3 * x14);
	const FLT x16 = (x6 * x8) + (-1 * x7 * x14);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x15 * x15) + (x11 * x11) + (x13 * x13));
	const FLT x18 = x17 * x16;
	const FLT x19 = x15 * x17;
	const FLT x20 = x13 * x17;
	const FLT x21 = x11 * x17;
	const FLT x22 = (x20 * (*_x0).Lighthouse.Rot[0]) + (x18 * (*_x0).Lighthouse.Rot[3]) +
					(-1 * x21 * (*_x0).Lighthouse.Rot[1]) + (x19 * (*_x0).Lighthouse.Rot[2]);
	const FLT x23 = x22 * x22;
	const FLT x24 = (x21 * (*_x0).Lighthouse.Rot[0]) + (x20 * (*_x0).Lighthouse.Rot[1]) +
					(x19 * (*_x0).Lighthouse.Rot[3]) + (-1 * x18 * (*_x0).Lighthouse.Rot[2]);
	const FLT x25 = x24 * x24;
	const FLT x26 = (x19 * (*_x0).Lighthouse.Rot[0]) + (-1 * x18 * (*_x0).Lighthouse.Rot[1]) +
					(-1 * x21 * (*_x0).Lighthouse.Rot[3]) + (-1 * x20 * (*_x0).Lighthouse.Rot[2]);
	const FLT x27 = (-1 * x20 * (*_x0).Lighthouse.Rot[3]) + (x19 * (*_x0).Lighthouse.Rot[1]) +
					(x21 * (*_x0).Lighthouse.Rot[2]) + (x18 * (*_x0).Lighthouse.Rot[0]);
	const FLT x28 = x27 * x27;
	const FLT x29 = x28 + x25 + (x26 * x26) + x23;
	const FLT x30 = 1. / x29;
	const FLT x31 = 2 * x30;
	const FLT x32 = x31 * x23;
	const FLT x33 = x31 * x25;
	const FLT x34 = -1 + x33 + x32;
	const FLT x35 = (*_x0).Object.Velocity.AxisAngleRot[2] + (*error_model).Object.Velocity.AxisAngleRot[2];
	const FLT x36 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[0];
	const FLT x37 = sin(x36);
	const FLT x38 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[1];
	const FLT x39 = cos(x38);
	const FLT x40 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[2];
	const FLT x41 = sin(x40);
	const FLT x42 = x41 * x39;
	const FLT x43 = cos(x36);
	const FLT x44 = cos(x40);
	const FLT x45 = sin(x38);
	const FLT x46 = x44 * x45;
	const FLT x47 = (x43 * x46) + (x42 * x37);
	const FLT x48 = x41 * x45;
	const FLT x49 = x44 * x39;
	const FLT x50 = (x43 * x49) + (x48 * x37);
	const FLT x51 = (x42 * x43) + (-1 * x46 * x37);
	const FLT x52 = (x49 * x37) + (-1 * x43 * x48);
	const FLT x53 = 1. / sqrt((x51 * x51) + (x47 * x47) + (x52 * x52) + (x50 * x50));
	const FLT x54 = x50 * x53;
	const FLT x55 = x53 * x52;
	const FLT x56 = x53 * x51;
	const FLT x57 = x53 * x47;
	const FLT x58 = (x57 * (*_x0).Object.Pose.Rot[1]) + (x56 * (*_x0).Object.Pose.Rot[0]) +
					(x54 * (*_x0).Object.Pose.Rot[3]) + (-1 * x55 * (*_x0).Object.Pose.Rot[2]);
	const FLT x59 = dt * dt;
	const FLT x60 = x35 * x35;
	const FLT x61 = x60 * x59;
	const FLT x62 = (*_x0).Object.Velocity.AxisAngleRot[0] + (*error_model).Object.Velocity.AxisAngleRot[0];
	const FLT x63 = x62 * x62;
	const FLT x64 = x63 * x59;
	const FLT x65 = (*_x0).Object.Velocity.AxisAngleRot[1] + (*error_model).Object.Velocity.AxisAngleRot[1];
	const FLT x66 = x65 * x65;
	const FLT x67 = x66 * x59;
	const FLT x68 = 1e-10 + x67 + x61 + x64;
	const FLT x69 = sqrt(x68);
	const FLT x70 = 0.5 * x69;
	const FLT x71 = sin(x70);
	const FLT x72 = x71 * x71;
	const FLT x73 = 1. / x68;
	const FLT x74 = x73 * x72;
	const FLT x75 = cos(x70);
	const FLT x76 = (x74 * x64) + (x74 * x61) + (x75 * x75) + (x74 * x67);
	const FLT x77 = 1. / sqrt(x76);
	const FLT x78 = x71 * x77;
	const FLT x79 = x78 * x58;
	const FLT x80 = 1. / x69;
	const FLT x81 = dt * x80;
	const FLT x82 = x81 * x79;
	const FLT x83 = (x57 * (*_x0).Object.Pose.Rot[0]) + (x54 * (*_x0).Object.Pose.Rot[2]) +
					(-1 * x56 * (*_x0).Object.Pose.Rot[1]) + (x55 * (*_x0).Object.Pose.Rot[3]);
	const FLT x84 = x81 * x78;
	const FLT x85 = x83 * x84;
	const FLT x86 = (-1 * x55 * (*_x0).Object.Pose.Rot[1]) + (x54 * (*_x0).Object.Pose.Rot[0]) +
					(-1 * x56 * (*_x0).Object.Pose.Rot[3]) + (-1 * x57 * (*_x0).Object.Pose.Rot[2]);
	const FLT x87 = x75 * x77;
	const FLT x88 = x86 * x87;
	const FLT x89 = (x55 * (*_x0).Object.Pose.Rot[0]) + (x54 * (*_x0).Object.Pose.Rot[1]) +
					(-1 * x57 * (*_x0).Object.Pose.Rot[3]) + (x56 * (*_x0).Object.Pose.Rot[2]);
	const FLT x90 = x89 * x84;
	const FLT x91 = (-1 * x62 * x90) + x88 + (-1 * x82 * x35) + (-1 * x85 * x65);
	const FLT x92 = x89 * x87;
	const FLT x93 = x84 * x86;
	const FLT x94 = (x62 * x93) + x92 + (-1 * x85 * x35) + (x82 * x65);
	const FLT x95 = x83 * x87;
	const FLT x96 = (x65 * x93) + (x90 * x35) + (-1 * x82 * x62) + x95;
	const FLT x97 = (-1 * x96 * sensor_pt[0]) + (x91 * sensor_pt[2]) + (x94 * sensor_pt[1]);
	const FLT x98 = x87 * x58;
	const FLT x99 = (x85 * x62) + (x93 * x35) + (-1 * x65 * x90) + x98;
	const FLT x100 = (x91 * sensor_pt[0]) + (-1 * x99 * sensor_pt[1]) + (x96 * sensor_pt[2]);
	const FLT x101 = dt * fabs(dt);
	const FLT x102 = 1.0 / 2.0 * x101;
	const FLT x103 = (2 * ((x99 * x100) + (-1 * x97 * x94))) + (*error_model).Object.Pose.Pos[1] +
					 (*_x0).Object.Pose.Pos[1] + (x102 * ((*_x0).Object.Acc[1] + (*error_model).Object.Acc[1])) +
					 sensor_pt[1] + (dt * ((*_x0).Object.Velocity.Pos[1] + (*error_model).Object.Velocity.Pos[1]));
	const FLT x104 = 1. / sqrt(x29);
	const FLT x105 = x26 * x104;
	const FLT x106 = (-1 * x94 * sensor_pt[2]) + (x91 * sensor_pt[1]) + (x99 * sensor_pt[0]);
	const FLT x107 = (*_x0).Object.Pose.Pos[0] + (2 * ((x97 * x96) + (-1 * x99 * x106))) +
					 (*error_model).Object.Pose.Pos[0] +
					 (x102 * ((*_x0).Object.Acc[0] + (*error_model).Object.Acc[0])) +
					 (dt * ((*_x0).Object.Velocity.Pos[0] + (*error_model).Object.Velocity.Pos[0])) + sensor_pt[0];
	const FLT x108 = x24 * x104;
	const FLT x109 = (*error_model).Object.Pose.Pos[2] +
					 (dt * ((*_x0).Object.Velocity.Pos[2] + (*error_model).Object.Velocity.Pos[2])) +
					 (2 * ((x94 * x106) + (-1 * x96 * x100))) + sensor_pt[2] +
					 (x102 * ((*_x0).Object.Acc[2] + (*error_model).Object.Acc[2])) + (*_x0).Object.Pose.Pos[2];
	const FLT x110 = x27 * x104;
	const FLT x111 = (x109 * x110) + (x103 * x105) + (-1 * x108 * x107);
	const FLT x112 = x104 * x107;
	const FLT x113 = (x22 * x112) + (-1 * x103 * x110) + (x109 * x105);
	const FLT x114 = x104 * x113;
	const FLT x115 = (*_x0).Lighthouse.Pos[1] + (*error_model).Lighthouse.Pos[1];
	const FLT x116 = (*_x0).Lighthouse.Pos[0] + (*error_model).Lighthouse.Pos[0];
	const FLT x117 = (*_x0).Lighthouse.Pos[2] + (*error_model).Lighthouse.Pos[2];
	const FLT x118 = (x110 * x117) + (x105 * x115) + (-1 * x108 * x116);
	const FLT x119 = x22 * x104;
	const FLT x120 = (x119 * x116) + (x105 * x117) + (-1 * x110 * x115);
	const FLT x121 =
		x107 + (2 * ((-1 * x22 * x114) + (x108 * x111))) + (-1 * (x116 + (2 * ((-1 * x119 * x120) + (x108 * x118)))));
	const FLT x122 = 2 * x121;
	const FLT x123 = x31 * x24;
	const FLT x124 = x27 * x123;
	const FLT x125 = -1 * x124;
	const FLT x126 = x22 * x26;
	const FLT x127 = x31 * x126;
	const FLT x128 = -1 * x127;
	const FLT x129 = x128 + x125;
	const FLT x130 = (x108 * x115) + (-1 * x119 * x117) + (x105 * x116);
	const FLT x131 = (x103 * x108) + (x26 * x112) + (-1 * x109 * x119);
	const FLT x132 =
		x109 + (-1 * (x117 + (2 * ((-1 * x110 * x118) + (x119 * x130))))) + (2 * ((-1 * x110 * x111) + (x119 * x131)));
	const FLT x133 = 2 * x132;
	const FLT x134 = (x129 * x133) + (x34 * x122);
	const FLT x135 = 0.523598775598299 + (-1 * (*error_model).BSD1.tilt) + (-1 * (*_x0).BSD1.tilt);
	const FLT x136 = tan(x135);
	const FLT x137 = x121 * x121;
	const FLT x138 = x137 + (x132 * x132);
	const FLT x139 =
		x103 + (-1 * (x115 + (2 * ((-1 * x108 * x130) + (x110 * x120))))) + (2 * ((-1 * x108 * x131) + (x27 * x114)));
	const FLT x140 = 1.0 / 2.0 * x139;
	const FLT x141 = (1. / (x138 * sqrt(x138))) * x136 * x140;
	const FLT x142 = x26 * x123;
	const FLT x143 = x31 * x27;
	const FLT x144 = x22 * x143;
	const FLT x145 = -1 * x144;
	const FLT x146 = x145 + x142;
	const FLT x147 = 1. / sqrt(x138);
	const FLT x148 = x136 * x147;
	const FLT x149 = (-1 * x146 * x148) + (x134 * x141);
	const FLT x150 = x139 * x139;
	const FLT x151 = 1. / x138;
	const FLT x152 = x136 * x136;
	const FLT x153 = 1. / sqrt(1 + (-1 * x150 * x151 * x152));
	const FLT x154 = 1. / x121;
	const FLT x155 = x132 * (1. / x137);
	const FLT x156 = x137 * x151;
	const FLT x157 = x156 * ((x34 * x155) + (-1 * x129 * x154));
	const FLT x158 = (-1 * x157) + (x149 * x153);
	const FLT x159 = x138 + x150;
	const FLT x160 = 1. / sqrt(x159);
	const FLT x161 = cos(x135);
	const FLT x162 = 1. / x161;
	const FLT x163 = x160 * x162;
	const FLT x164 = asin(x163 * x139);
	const FLT x165 = 8.0108022e-06 * x164;
	const FLT x166 = -8.0108022e-06 + (-1 * x165);
	const FLT x167 = 0.0028679863 + (x166 * x164);
	const FLT x168 = 5.3685255e-06 + (x167 * x164);
	const FLT x169 = 0.0076069798 + (x168 * x164);
	const FLT x170 = x164 * x164;
	const FLT x171 = x169 * x164;
	const FLT x172 = -8.0108022e-06 + (-1.60216044e-05 * x164);
	const FLT x173 = x167 + (x164 * x172);
	const FLT x174 = x168 + (x164 * x173);
	const FLT x175 = x169 + (x164 * x174);
	const FLT x176 = (x164 * x175) + x171;
	const FLT x177 = (*error_model).BSD1.ogeemag + (*_x0).BSD1.ogeemag;
	const FLT x178 = atan2(-1 * x132, x121);
	const FLT x179 = -1 * x139 * x148;
	const FLT x180 = asin(x179) + (-1 * x178) + (-1 * (*error_model).BSD1.ogeephase) + (-1 * (*_x0).BSD1.ogeephase);
	const FLT x181 = sin(x180);
	const FLT x182 = (*_x0).BSD1.curve + (-1 * x177 * x181) + (*error_model).BSD1.curve;
	const FLT x183 = sin(x135);
	const FLT x184 = x183 * x182;
	const FLT x185 = x176 * x184;
	const FLT x186 = x161 + x185;
	const FLT x187 = 1. / x186;
	const FLT x188 = x170 * x187;
	const FLT x189 = x169 * x188;
	const FLT x190 = x177 * cos(x180);
	const FLT x191 = x189 * x190;
	const FLT x192 = x176 * x183;
	const FLT x193 = x190 * x192;
	const FLT x194 = 2 * x139;
	const FLT x195 = x162 * x140 * (1. / (x159 * sqrt(x159)));
	const FLT x196 = (-1 * x195 * (x134 + (x194 * x146))) + (x163 * x146);
	const FLT x197 = 1. / (x161 * x161);
	const FLT x198 = 1. / sqrt(1 + (-1 * x197 * x150 * (1. / x159)));
	const FLT x199 = x175 * x198;
	const FLT x200 = x169 * x198;
	const FLT x201 = x166 * x198;
	const FLT x202 = x201 * x196;
	const FLT x203 = x196 * x198;
	const FLT x204 = 2.40324066e-05 * x164;
	const FLT x205 = (x164 * (x202 + (-1 * x203 * x165))) + (x203 * x167);
	const FLT x206 = (x203 * x168) + (x205 * x164);
	const FLT x207 = x169 * x170 * (1. / (x186 * x186));
	const FLT x208 = x207 * x182;
	const FLT x209 = 2 * x182;
	const FLT x210 = x209 * x171 * x187;
	const FLT x211 = x188 * x182;
	const FLT x212 = x179 + (x189 * x182);
	const FLT x213 = 1. / sqrt(1 + (-1 * (x212 * x212)));
	const FLT x214 =
		x157 +
		(-1 * x213 *
		 (x149 + (x211 * x206) + (x210 * x203) + (-1 * x191 * x158) +
		  (-1 * x208 *
		   ((x184 *
			 ((x164 * (x206 + (x164 * (x205 + (x164 * ((-1 * x203 * x204) + x202 + (x203 * x172))) + (x203 * x173))) +
					   (x203 * x174))) +
			  (x196 * x199) + (x206 * x164) + (x200 * x196))) +
			(-1 * x193 * x158)))));
	const FLT x215 = (-1 * asin(x212)) + (*error_model).BSD1.gibpha + x178 + (*_x0).BSD1.gibpha;
	const FLT x216 = cos(x215) * ((*error_model).BSD1.gibmag + (*_x0).BSD1.gibmag);
	const FLT x217 = x31 * x28;
	const FLT x218 = -1 + x217;
	const FLT x219 = x218 + x33;
	const FLT x220 = -1 * x142;
	const FLT x221 = x220 + x145;
	const FLT x222 = x26 * x143;
	const FLT x223 = x22 * x123;
	const FLT x224 = -1 * x223;
	const FLT x225 = x224 + x222;
	const FLT x226 = (x225 * x133) + (x221 * x122);
	const FLT x227 = (-1 * x195 * (x226 + (x219 * x194))) + (x219 * x163);
	const FLT x228 = x227 * x198;
	const FLT x229 = x167 * x198;
	const FLT x230 = x165 * x198;
	const FLT x231 = x201 * x227;
	const FLT x232 = (x164 * (x231 + (-1 * x230 * x227))) + (x227 * x229);
	const FLT x233 = x168 * x198;
	const FLT x234 = (x233 * x227) + (x232 * x164);
	const FLT x235 = (-1 * x219 * x148) + (x226 * x141);
	const FLT x236 = ((x221 * x155) + (-1 * x225 * x154)) * x156;
	const FLT x237 = (-1 * x236) + (x235 * x153);
	const FLT x238 = x172 * x198;
	const FLT x239 = x173 * x198;
	const FLT x240 = x174 * x198;
	const FLT x241 =
		x236 +
		(-1 * x213 *
		 ((-1 * x237 * x191) + x235 + (x210 * x228) +
		  (-1 * x208 *
		   ((x184 *
			 ((x234 * x164) + (x200 * x227) +
			  (x164 * (x234 + (x164 * (x232 + (x164 * ((-1 * x204 * x228) + (x238 * x227) + x231)) + (x239 * x227))) +
					   (x227 * x240))) +
			  (x227 * x199))) +
			(-1 * x237 * x193))) +
		  (x211 * x234)));
	const FLT x242 = -1 * x222;
	const FLT x243 = x242 + x224;
	const FLT x244 = x125 + x127;
	const FLT x245 = x218 + x32;
	const FLT x246 = (x245 * x133) + (x244 * x122);
	const FLT x247 = (-1 * x195 * (x246 + (x243 * x194))) + (x243 * x163);
	const FLT x248 = x171 * x187 * x198;
	const FLT x249 = x209 * x248;
	const FLT x250 = (-1 * x243 * x148) + (x246 * x141);
	const FLT x251 = ((x244 * x155) + (-1 * x245 * x154)) * x156;
	const FLT x252 = (-1 * x251) + (x250 * x153);
	const FLT x253 = x201 * x247;
	const FLT x254 = x204 * x198;
	const FLT x255 = (x164 * (x253 + (-1 * x230 * x247))) + (x229 * x247);
	const FLT x256 = (x233 * x247) + (x255 * x164);
	const FLT x257 =
		x251 +
		(-1 * x213 *
		 (x250 + (x211 * x256) + (x247 * x249) + (-1 * x252 * x191) +
		  (-1 * x208 *
		   ((x184 * ((x256 * x164) + (x200 * x247) +
					 (x164 * ((x164 * (x255 + (x164 * ((-1 * x254 * x247) + x253 + (x238 * x247))) + (x239 * x247))) +
							  x256 + (x240 * x247))) +
					 (x247 * x199))) +
			(-1 * x252 * x193)))));
	const FLT x258 = 1. / (x29 * sqrt(x29));
	const FLT x259 = 2 * x27;
	const FLT x260 = 2 * x21;
	const FLT x261 = 2 * x19;
	const FLT x262 = 2 * x20;
	const FLT x263 = ((x26 * x261) + (x22 * x262) + (x18 * x259) + (x24 * x260)) * x258;
	const FLT x264 = x24 * x263;
	const FLT x265 = 1.0 / 2.0 * x103;
	const FLT x266 = 1.0 / 2.0 * x26;
	const FLT x267 = x266 * x263;
	const FLT x268 = x22 * x263;
	const FLT x269 = 1.0 / 2.0 * x109;
	const FLT x270 = x103 * x104;
	const FLT x271 = x21 * x270;
	const FLT x272 = x19 * x112;
	const FLT x273 = x109 * x104;
	const FLT x274 = -1 * x20 * x273;
	const FLT x275 = x274 + x272;
	const FLT x276 = x275 + x271 + (x268 * x269) + (-1 * x264 * x265) + (-1 * x267 * x107);
	const FLT x277 = 2 * x108;
	const FLT x278 = 2 * x18;
	const FLT x279 = x104 * x120;
	const FLT x280 = x279 * x278;
	const FLT x281 = x27 * x120;
	const FLT x282 = x27 * x113;
	const FLT x283 = 1.0 / 2.0 * x268;
	const FLT x284 = 1.0 / 2.0 * x27;
	const FLT x285 = x284 * x115;
	const FLT x286 = x104 * x116;
	const FLT x287 = x20 * x286;
	const FLT x288 = x104 * x115;
	const FLT x289 = -1 * x18 * x288;
	const FLT x290 = x104 * x117;
	const FLT x291 = x19 * x290;
	const FLT x292 = x291 + x289;
	const FLT x293 = x292 + x287 + (x263 * x285) + (-1 * x283 * x116) + (-1 * x267 * x117);
	const FLT x294 = 2 * x110;
	const FLT x295 = x27 * x265;
	const FLT x296 = x19 * x273;
	const FLT x297 = -1 * x18 * x270;
	const FLT x298 = x20 * x112;
	const FLT x299 = x298 + (x295 * x263) + x297 + (-1 * x283 * x107) + x296 + (-1 * x267 * x109);
	const FLT x300 = 1.0 / 2.0 * x264;
	const FLT x301 = x21 * x288;
	const FLT x302 = -1 * x20 * x290;
	const FLT x303 = x19 * x286;
	const FLT x304 = x303 + x302;
	const FLT x305 = x304 + x301 + (-1 * x267 * x116) + (-1 * x300 * x115) + (x283 * x117);
	const FLT x306 = x278 * x114;
	const FLT x307 = x260 * x104;
	const FLT x308 = (-1 * x307 * x131) + (x307 * x130);
	const FLT x309 = (x299 * x294) + (-1 * x263 * x282) + (x264 * x131) + x306 + (x277 * x305) + (-1 * x277 * x276) +
					 x308 + (-1 * x293 * x294) + (-1 * x264 * x130) + (-1 * x280) + (x263 * x281);
	const FLT x310 = 2 * x119;
	const FLT x311 = x27 * x269;
	const FLT x312 = x18 * x273;
	const FLT x313 = -1 * x21 * x112;
	const FLT x314 = x19 * x270;
	const FLT x315 = x314 + x313;
	const FLT x316 = x312 + x315 + (x300 * x107) + (-1 * x263 * x311) + (-1 * x267 * x103);
	const FLT x317 = x307 * x111;
	const FLT x318 = x307 * x118;
	const FLT x319 = x284 * x117;
	const FLT x320 = x19 * x288;
	const FLT x321 = x18 * x290;
	const FLT x322 = -1 * x21 * x286;
	const FLT x323 = x322 + (-1 * x267 * x115) + (-1 * x263 * x319) + x321 + (x300 * x116) + x320;
	const FLT x324 = (x279 * x262) + (-1 * x262 * x114);
	const FLT x325 = x324 + (-1 * x277 * x323) + (-1 * x318) + (x264 * x118) + (x293 * x310) + (x268 * x113) +
					 (-1 * x268 * x120) + x317 + (x277 * x316) + (-1 * x299 * x310) + (-1 * x264 * x111);
	const FLT x326 = x27 * x111;
	const FLT x327 = x262 * x104;
	const FLT x328 = x327 * x130;
	const FLT x329 = x27 * x118;
	const FLT x330 = x327 * x131;
	const FLT x331 = x104 * x118;
	const FLT x332 = x278 * x104;
	const FLT x333 = (-1 * x332 * x111) + (x278 * x331);
	const FLT x334 = x333 + (-1 * x294 * x316) + x330 + (-1 * x263 * x329) + (x294 * x323) + (-1 * x305 * x310) +
					 (x268 * x130) + (x263 * x326) + (-1 * x268 * x131) + (-1 * x328) + (x276 * x310);
	const FLT x335 = (x334 * x133) + (x325 * x122);
	const FLT x336 = (-1 * x195 * (x335 + (x309 * x194))) + (x309 * x163);
	const FLT x337 = (-1 * x309 * x148) + (x335 * x141);
	const FLT x338 = ((x325 * x155) + (-1 * x334 * x154)) * x156;
	const FLT x339 = (-1 * x338) + (x337 * x153);
	const FLT x340 = x201 * x336;
	const FLT x341 = (x164 * (x340 + (-1 * x230 * x336))) + (x229 * x336);
	const FLT x342 = (x233 * x336) + (x341 * x164);
	const FLT x343 =
		x338 +
		(-1 * x213 *
		 ((x211 * x342) + (x249 * x336) + x337 + (-1 * x339 * x191) +
		  (-1 * x208 *
		   ((x184 *
			 ((x342 * x164) + (x200 * x336) + (x336 * x199) +
			  (x164 * (x342 + (x164 * (x341 + (x164 * ((-1 * x254 * x336) + x340 + (x238 * x336))) + (x239 * x336))) +
					   (x240 * x336))))) +
			(-1 * x339 * x193)))));
	const FLT x344 = ((x24 * x262) + (-1 * x22 * x260) + (x19 * x259) + (-1 * x26 * x278)) * x258;
	const FLT x345 = x344 * x131;
	const FLT x346 = x261 * x104;
	const FLT x347 = x346 * x120;
	const FLT x348 = x24 * x130;
	const FLT x349 = 1.0 / 2.0 * x22;
	const FLT x350 = x349 * x344;
	const FLT x351 = x266 * x344;
	const FLT x352 = x284 * x344;
	const FLT x353 = x322 + (-1 * x321);
	const FLT x354 = x353 + (x352 * x115) + (-1 * x320) + (-1 * x350 * x116) + (-1 * x351 * x117);
	const FLT x355 = 1.0 / 2.0 * x24;
	const FLT x356 = x355 * x344;
	const FLT x357 = x21 * x290;
	const FLT x358 = x18 * x286;
	const FLT x359 = x20 * x288;
	const FLT x360 = x359 + x357 + (x350 * x117) + (-1 * x356 * x115) + (-1 * x358) + (-1 * x351 * x116);
	const FLT x361 = x344 * x103;
	const FLT x362 = x344 * x107;
	const FLT x363 = x20 * x270;
	const FLT x364 = x21 * x273;
	const FLT x365 = x18 * x112;
	const FLT x366 = (-1 * x266 * x362) + x363 + (x350 * x109) + (-1 * x361 * x355) + x364 + (-1 * x365);
	const FLT x367 = -1 * x312;
	const FLT x368 = x313 + (x284 * x361) + (-1 * x351 * x109) + (-1 * x362 * x349) + (-1 * x314) + x367;
	const FLT x369 = x261 * x114;
	const FLT x370 = x328 + x369 + (-1 * x282 * x344) + (-1 * x348 * x344) + (x24 * x345) + (-1 * x347) +
					 (-1 * x294 * x354) + (-1 * x277 * x366) + (x294 * x368) + (x277 * x360) + (-1 * x330) +
					 (x281 * x344);
	const FLT x371 = (-1 * x298) + x297;
	const FLT x372 = (-1 * x266 * x361) + x371 + (x356 * x107) + x296 + (-1 * x311 * x344);
	const FLT x373 = x22 * x113;
	const FLT x374 = x24 * x111;
	const FLT x375 = -1 * x287;
	const FLT x376 = x292 + x375 + (x356 * x116) + (-1 * x351 * x115) + (-1 * x352 * x117);
	const FLT x377 = x22 * x120;
	const FLT x378 = x24 * x118;
	const FLT x379 = (-1 * x307 * x120) + (x260 * x114);
	const FLT x380 = (x327 * x111) + (-1 * x262 * x331);
	const FLT x381 = (x354 * x310) + (-1 * x377 * x344) + (x378 * x344) + (x373 * x344) + x380 + (-1 * x374 * x344) +
					 (x277 * x372) + x379 + (-1 * x277 * x376) + (-1 * x368 * x310);
	const FLT x382 = x346 * x118;
	const FLT x383 = x22 * x130;
	const FLT x384 = x346 * x111;
	const FLT x385 = (-1 * x329 * x344) + (-1 * x384) + (x383 * x344) + x308 + (x366 * x310) + (-1 * x360 * x310) +
					 (-1 * x294 * x372) + x382 + (x294 * x376) + (x326 * x344) + (-1 * x22 * x345);
	const FLT x386 = (x385 * x133) + (x381 * x122);
	const FLT x387 = (-1 * x195 * (x386 + (x370 * x194))) + (x370 * x163);
	const FLT x388 = (-1 * x370 * x148) + (x386 * x141);
	const FLT x389 = ((x381 * x155) + (-1 * x385 * x154)) * x156;
	const FLT x390 = (-1 * x389) + (x388 * x153);
	const FLT x391 = x201 * x387;
	const FLT x392 = (x164 * (x391 + (-1 * x230 * x387))) + (x229 * x387);
	const FLT x393 = (x233 * x387) + (x392 * x164);
	const FLT x394 =
		x389 +
		(-1 * x213 *
		 ((x211 * x393) + x388 + (-1 * x390 * x191) + (x249 * x387) +
		  (-1 * x208 *
		   ((x184 *
			 ((x387 * x199) +
			  (x164 * (x393 + (x164 * (x392 + (x164 * ((-1 * x254 * x387) + x391 + (x238 * x387))) + (x239 * x387))) +
					   (x240 * x387))) +
			  (x393 * x164) + (x200 * x387))) +
			(-1 * x390 * x193)))));
	const FLT x395 = ((x22 * x261) + (-1 * x24 * x278) + (x21 * x259) + (-1 * x26 * x262)) * x258;
	const FLT x396 = x395 * x131;
	const FLT x397 = -1 * x301;
	const FLT x398 = x395 * x116;
	const FLT x399 = x266 * x395;
	const FLT x400 = x395 * x115;
	const FLT x401 = (-1 * x399 * x117) + (x400 * x284) + x397 + x304 + (-1 * x398 * x349);
	const FLT x402 = x395 * x355;
	const FLT x403 = x395 * x349;
	const FLT x404 = (x403 * x109) + (-1 * x402 * x103) + x371 + (-1 * x399 * x107) + (-1 * x296);
	const FLT x405 = x289 + (x403 * x117) + (-1 * x400 * x355) + (-1 * x266 * x398) + x375 + (-1 * x291);
	const FLT x406 = -1 * x271;
	const FLT x407 = (x295 * x395) + x275 + (-1 * x399 * x109) + x406 + (-1 * x403 * x107);
	const FLT x408 = (-1 * x332 * x130) + (x332 * x131);
	const FLT x409 = x408 + x379 + (x407 * x294) + (x281 * x395) + (x24 * x396) + (-1 * x282 * x395) + (x405 * x277) +
					 (-1 * x395 * x348) + (-1 * x401 * x294) + (-1 * x404 * x277);
	const FLT x410 = (x402 * x107) + (-1 * x399 * x103) + x364 + (-1 * x363) + (-1 * x395 * x311) + x365;
	const FLT x411 = (-1 * x399 * x115) + x357 + (-1 * x395 * x319) + (-1 * x359) + (x398 * x355) + x358;
	const FLT x412 = (x401 * x310) + (x378 * x395) + (-1 * x369) + x333 + (-1 * x377 * x395) + (-1 * x407 * x310) +
					 x347 + (x373 * x395) + (x410 * x277) + (-1 * x374 * x395) + (-1 * x411 * x277);
	const FLT x413 = x346 * x130;
	const FLT x414 = x346 * x131;
	const FLT x415 = (-1 * x22 * x396) + (x395 * x326) + x414 + (x395 * x383) + (-1 * x413) + (-1 * x410 * x294) +
					 (x404 * x310) + (-1 * x405 * x310) + (-1 * x395 * x329) + x318 + (x411 * x294) + (-1 * x317);
	const FLT x416 = (x415 * x133) + (x412 * x122);
	const FLT x417 = (-1 * x195 * (x416 + (x409 * x194))) + (x409 * x163);
	const FLT x418 = (-1 * x409 * x148) + (x416 * x141);
	const FLT x419 = ((x412 * x155) + (-1 * x415 * x154)) * x156;
	const FLT x420 = (-1 * x419) + (x418 * x153);
	const FLT x421 = x417 * x201;
	const FLT x422 = (x164 * (x421 + (-1 * x417 * x230))) + (x417 * x229);
	const FLT x423 = (x417 * x233) + (x422 * x164);
	const FLT x424 =
		x419 +
		(-1 * x213 *
		 (x418 +
		  (-1 * x208 *
		   ((x184 *
			 ((x423 * x164) + (x417 * x200) + (x417 * x199) +
			  (x164 * (x423 + (x164 * (x422 + (x164 * ((-1 * x417 * x254) + x421 + (x417 * x238))) + (x417 * x239))) +
					   (x417 * x240))))) +
			(-1 * x420 * x193))) +
		  (x423 * x211) + (x417 * x249) + (-1 * x420 * x191)));
	const FLT x425 = ((x22 * x278) + (x24 * x261) + (-1 * x20 * x259) + (-1 * x26 * x260)) * x258;
	const FLT x426 = x425 * x131;
	const FLT x427 = x425 * x355;
	const FLT x428 = x425 * x349;
	const FLT x429 = x425 * x266;
	const FLT x430 = (-1 * x429 * x116) + (x428 * x117) + x320 + x353 + (-1 * x427 * x115);
	const FLT x431 = x315 + (x428 * x109) + (-1 * x429 * x107) + x367 + (-1 * x427 * x103);
	const FLT x432 = x359 + (-1 * x429 * x117) + (-1 * x428 * x116) + x358 + (x425 * x285) + (-1 * x357);
	const FLT x433 = (x425 * x295) + (-1 * x428 * x107) + x365 + x363 + (-1 * x429 * x109) + (-1 * x364);
	const FLT x434 = (-1 * x432 * x294) + x324 + (-1 * x431 * x277) + (-1 * x425 * x348) + (-1 * x414) +
					 (-1 * x425 * x282) + (x425 * x281) + (x430 * x277) + (x433 * x294) + (x24 * x426) + x413;
	const FLT x435 = (-1 * x272) + (-1 * x429 * x103) + (-1 * x425 * x311) + (x427 * x107) + x406 + x274;
	const FLT x436 = x302 + (-1 * x429 * x115) + (-1 * x425 * x319) + x397 + (x427 * x116) + (-1 * x303);
	const FLT x437 = x280 + (-1 * x433 * x310) + (-1 * x382) + x384 + (x432 * x310) + (-1 * x425 * x377) +
					 (-1 * x436 * x277) + (x425 * x373) + (x435 * x277) + (-1 * x306) + (-1 * x425 * x374) +
					 (x425 * x378);
	const FLT x438 = x380 + (-1 * x22 * x426) + (x425 * x326) + x408 + (x431 * x310) + (x425 * x383) +
					 (-1 * x425 * x329) + (-1 * x435 * x294) + (x436 * x294) + (-1 * x430 * x310);
	const FLT x439 = (x438 * x133) + (x437 * x122);
	const FLT x440 = (-1 * x195 * (x439 + (x434 * x194))) + (x434 * x163);
	const FLT x441 = (-1 * x434 * x148) + (x439 * x141);
	const FLT x442 = ((x437 * x155) + (-1 * x438 * x154)) * x156;
	const FLT x443 = (-1 * x442) + (x441 * x153);
	const FLT x444 = x440 * x201;
	const FLT x445 = (x164 * (x444 + (-1 * x440 * x230))) + (x440 * x229);
	const FLT x446 = (x440 * x233) + (x445 * x164);
	const FLT x447 =
		x442 +
		(-1 * x213 *
		 (x441 + (-1 * x443 * x191) + (x446 * x211) + (x440 * x249) +
		  (-1 * x208 *
		   ((x184 *
			 ((x446 * x164) + (x440 * x200) +
			  (x164 * (x446 + (x164 * (x445 + (x164 * ((-1 * x440 * x254) + x444 + (x440 * x238))) + (x440 * x239))) +
					   (x440 * x240))) +
			  (x440 * x199))) +
			(-1 * x443 * x193)))));
	const FLT x448 = x30 * x101;
	const FLT x449 = x27 * x448;
	const FLT x450 = x22 * x449;
	const FLT x451 = x24 * x448;
	const FLT x452 = x26 * x451;
	const FLT x453 = (-1 * x452) + x450;
	const FLT x454 = -1 * x23 * x448;
	const FLT x455 = -1 * x25 * x448;
	const FLT x456 = x455 + x102 + x454;
	const FLT x457 = x448 * x126;
	const FLT x458 = x27 * x451;
	const FLT x459 = x458 + x457;
	const FLT x460 = (x459 * x133) + (x456 * x122);
	const FLT x461 = (-1 * x195 * (x460 + (x453 * x194))) + (x453 * x163);
	const FLT x462 = x461 * x198;
	const FLT x463 = (-1 * x453 * x148) + (x460 * x141);
	const FLT x464 = ((x456 * x155) + (-1 * x459 * x154)) * x156;
	const FLT x465 = (-1 * x464) + (x463 * x153);
	const FLT x466 = x461 * x201;
	const FLT x467 = (x164 * (x466 + (-1 * x462 * x165))) + (x462 * x167);
	const FLT x468 = (x462 * x168) + (x467 * x164);
	const FLT x469 =
		x464 +
		(-1 * x213 *
		 (x463 + (x468 * x211) + (x462 * x210) +
		  (-1 * x208 *
		   ((x184 *
			 ((x468 * x164) + (x462 * x175) +
			  (x164 * (x468 + (x164 * ((x164 * ((-1 * x462 * x204) + x466 + (x462 * x172))) + x467 + (x462 * x173))) +
					   (x462 * x174))) +
			  (x461 * x200))) +
			(-1 * x465 * x193))) +
		  (-1 * x465 * x191)));
	const FLT x470 = x450 + x452;
	const FLT x471 = x22 * x451;
	const FLT x472 = x26 * x449;
	const FLT x473 = (-1 * x472) + x471;
	const FLT x474 = (x473 * x133) + (x470 * x122);
	const FLT x475 = (-1 * x28 * x448) + x102;
	const FLT x476 = x475 + x455;
	const FLT x477 = (-1 * x476 * x148) + (x474 * x141);
	const FLT x478 = ((x470 * x155) + (-1 * x473 * x154)) * x156;
	const FLT x479 = (-1 * x478) + (x477 * x153);
	const FLT x480 = (-1 * x195 * (x474 + (x476 * x194))) + (x476 * x163);
	const FLT x481 = x480 * x201;
	const FLT x482 = (x164 * (x481 + (-1 * x480 * x230))) + (x480 * x229);
	const FLT x483 = (x480 * x233) + (x482 * x164);
	const FLT x484 =
		x478 +
		(-1 * x213 *
		 (x477 + (x483 * x211) + (x480 * x249) +
		  (-1 * x208 *
		   ((x184 *
			 ((x480 * x199) + (x483 * x164) + (x480 * x200) +
			  (x164 * (x483 + (x164 * ((x164 * ((-1 * x480 * x254) + x481 + (x480 * x238))) + x482 + (x480 * x239))) +
					   (x480 * x240))))) +
			(-1 * x479 * x193))) +
		  (-1 * x479 * x191)));
	const FLT x485 = x471 + x472;
	const FLT x486 = (-1 * x457) + x458;
	const FLT x487 = x475 + x454;
	const FLT x488 = (x487 * x133) + (x486 * x122);
	const FLT x489 = (-1 * x195 * (x488 + (x485 * x194))) + (x485 * x163);
	const FLT x490 = (-1 * x485 * x148) + (x488 * x141);
	const FLT x491 = ((x486 * x155) + (-1 * x487 * x154)) * x156;
	const FLT x492 = (-1 * x491) + (x490 * x153);
	const FLT x493 = x489 * x201;
	const FLT x494 = (x164 * (x493 + (-1 * x489 * x230))) + (x489 * x229);
	const FLT x495 = (x489 * x233) + (x494 * x164);
	const FLT x496 =
		x491 +
		(-1 * x213 *
		 (x490 + (x495 * x211) + (x489 * x249) + (-1 * x492 * x191) +
		  (-1 * x208 *
		   ((x184 *
			 ((x495 * x164) +
			  (x164 * (x495 + (x164 * ((x164 * ((-1 * x489 * x254) + x493 + (x489 * x238))) + x494 + (x489 * x239))) +
					   (x489 * x240))) +
			  (x489 * x200) + (x489 * x199))) +
			(-1 * x492 * x193)))));
	const FLT x497 = -1 * x33;
	const FLT x498 = -1 * x32;
	const FLT x499 = 1 + x498 + x497;
	const FLT x500 = x124 + x127;
	const FLT x501 = (x500 * x133) + (x499 * x122);
	const FLT x502 = x220 + x144;
	const FLT x503 = (-1 * x502 * x148) + (x501 * x141);
	const FLT x504 = ((x499 * x155) + (-1 * x500 * x154)) * x156;
	const FLT x505 = (-1 * x504) + (x503 * x153);
	const FLT x506 = (-1 * x195 * (x501 + (x502 * x194))) + (x502 * x163);
	const FLT x507 = x506 * x201;
	const FLT x508 = (x164 * (x507 + (-1 * x506 * x230))) + (x506 * x229);
	const FLT x509 = (x506 * x233) + (x508 * x164);
	const FLT x510 =
		x504 +
		(-1 * x213 *
		 ((x509 * x211) + (x506 * x249) + x503 +
		  (-1 * x208 *
		   ((x184 *
			 ((x509 * x164) + (x506 * x200) +
			  (x164 * (x509 + (x164 * (x508 + (x164 * (x507 + (-1 * x506 * x254) + (x506 * x238))) + (x506 * x239))) +
					   (x506 * x240))) +
			  (x506 * x199))) +
			(-1 * x505 * x193))) +
		  (-1 * x505 * x191)));
	const FLT x511 = 1 + (-1 * x217);
	const FLT x512 = x511 + x497;
	const FLT x513 = x144 + x142;
	const FLT x514 = x242 + x223;
	const FLT x515 = (x514 * x133) + (x513 * x122);
	const FLT x516 = (-1 * x195 * (x515 + (x512 * x194))) + (x512 * x163);
	const FLT x517 = (-1 * x512 * x148) + (x515 * x141);
	const FLT x518 = ((x513 * x155) + (-1 * x514 * x154)) * x156;
	const FLT x519 = (-1 * x518) + (x517 * x153);
	const FLT x520 = x516 * x201;
	const FLT x521 = (x164 * (x520 + (-1 * x516 * x230))) + (x516 * x229);
	const FLT x522 = (x516 * x233) + (x521 * x164);
	const FLT x523 =
		x518 +
		(-1 * x213 *
		 (x517 + (x522 * x211) + (-1 * x519 * x191) + (x516 * x249) +
		  (-1 * x208 *
		   ((x184 *
			 ((x516 * x200) +
			  (x164 * (x522 + (x164 * (x521 + (x164 * ((-1 * x516 * x254) + x520 + (x516 * x238))) + (x516 * x239))) +
					   (x516 * x240))) +
			  (x522 * x164) + (x516 * x199))) +
			(-1 * x519 * x193)))));
	const FLT x524 = x223 + x222;
	const FLT x525 = x128 + x124;
	const FLT x526 = x511 + x498;
	const FLT x527 = (x526 * x133) + (x525 * x122);
	const FLT x528 = (-1 * x195 * (x527 + (x524 * x194))) + (x524 * x163);
	const FLT x529 = (-1 * x524 * x148) + (x527 * x141);
	const FLT x530 = ((x525 * x155) + (-1 * x526 * x154)) * x156;
	const FLT x531 = (-1 * x530) + (x529 * x153);
	const FLT x532 = x528 * x201;
	const FLT x533 = (x164 * (x532 + (-1 * x528 * x230))) + (x528 * x229);
	const FLT x534 = (x528 * x233) + (x533 * x164);
	const FLT x535 =
		x530 +
		(-1 * x213 *
		 ((-1 * x531 * x191) + (x528 * x249) + x529 + (x534 * x211) +
		  (-1 * x208 *
		   ((x184 *
			 ((x528 * x199) + (x534 * x164) +
			  (x164 * (x534 + (x164 * (x533 + (x164 * ((-1 * x528 * x254) + x532 + (x528 * x238))) + (x528 * x239))) +
					   (x528 * x240))) +
			  (x528 * x200))) +
			(-1 * x531 * x193)))));
	const FLT x536 = x87 * x55;
	const FLT x537 = x84 * x54;
	const FLT x538 = x62 * x537;
	const FLT x539 = x84 * x57;
	const FLT x540 = -1 * x35 * x539;
	const FLT x541 = x84 * x56;
	const FLT x542 = x65 * x541;
	const FLT x543 = x542 + x540;
	const FLT x544 = x536 + x543 + x538;
	const FLT x545 = x65 * x539;
	const FLT x546 = -1 * x545;
	const FLT x547 = x84 * x55;
	const FLT x548 = x62 * x547;
	const FLT x549 = -1 * x548;
	const FLT x550 = x87 * x54;
	const FLT x551 = x35 * x541;
	const FLT x552 = (-1 * x551) + x550;
	const FLT x553 = x546 + x552 + x549;
	const FLT x554 = x62 * x539;
	const FLT x555 = x87 * x56;
	const FLT x556 = -1 * x65 * x547;
	const FLT x557 = x35 * x537;
	const FLT x558 = x557 + x556;
	const FLT x559 = x558 + x554 + x555;
	const FLT x560 = (x559 * sensor_pt[0]) + (-1 * x544 * sensor_pt[2]) + (x553 * sensor_pt[1]);
	const FLT x561 = 2 * x99;
	const FLT x562 = 2 * x106;
	const FLT x563 = x65 * x537;
	const FLT x564 = x87 * x57;
	const FLT x565 = -1 * x62 * x541;
	const FLT x566 = x35 * x547;
	const FLT x567 = x566 + x565;
	const FLT x568 = x567 + x563 + x564;
	const FLT x569 = (x544 * sensor_pt[1]) + (-1 * x568 * sensor_pt[0]) + (x553 * sensor_pt[2]);
	const FLT x570 = 2 * x96;
	const FLT x571 = 2 * x97;
	const FLT x572 = (x570 * x569) + (x571 * x568) + (-1 * x560 * x561) + (-1 * x559 * x562);
	const FLT x573 = 2 * x94;
	const FLT x574 = (-1 * x559 * sensor_pt[1]) + (x568 * sensor_pt[2]) + (x553 * sensor_pt[0]);
	const FLT x575 = 2 * x100;
	const FLT x576 = (x575 * x559) + (-1 * x573 * x569) + (x574 * x561) + (-1 * x544 * x571);
	const FLT x577 = (x573 * x560) + (-1 * x570 * x574) + (-1 * x575 * x568) + (x544 * x562);
	const FLT x578 = (x577 * x105) + (x572 * x119) + (-1 * x576 * x110);
	const FLT x579 = (-1 * x577 * x119) + (x576 * x108) + (x572 * x105);
	const FLT x580 = x576 + (x578 * x294) + (-1 * x579 * x277);
	const FLT x581 = (-1 * x572 * x108) + (x577 * x110) + (x576 * x105);
	const FLT x582 = x572 + (-1 * x578 * x310) + (x581 * x277);
	const FLT x583 = x577 + (x579 * x310) + (-1 * x581 * x294);
	const FLT x584 = (x583 * x133) + (x582 * x122);
	const FLT x585 = (-1 * x195 * (x584 + (x580 * x194))) + (x580 * x163);
	const FLT x586 = (-1 * x580 * x148) + (x584 * x141);
	const FLT x587 = ((x582 * x155) + (-1 * x583 * x154)) * x156;
	const FLT x588 = (-1 * x587) + (x586 * x153);
	const FLT x589 = x585 * x201;
	const FLT x590 = (x164 * (x589 + (-1 * x585 * x230))) + (x585 * x229);
	const FLT x591 = (x585 * x233) + (x590 * x164);
	const FLT x592 =
		x587 +
		(-1 * x213 *
		 ((x591 * x211) + x586 + (x585 * x249) +
		  (-1 * x208 *
		   ((x184 *
			 ((x164 * (x591 + (x164 * (x590 + (x164 * ((-1 * x585 * x254) + x589 + (x585 * x238))) + (x585 * x239))) +
					   (x585 * x240))) +
			  (x591 * x164) + (x585 * x200) + (x585 * x199))) +
			(-1 * x588 * x193))) +
		  (-1 * x588 * x191)));
	const FLT x593 = -1 * x563;
	const FLT x594 = (-1 * x566) + x565;
	const FLT x595 = x594 + x593 + x564;
	const FLT x596 = x550 + x551;
	const FLT x597 = x549 + x596 + x545;
	const FLT x598 = -1 * x538;
	const FLT x599 = -1 * x536;
	const FLT x600 = x543 + x598 + x599;
	const FLT x601 = (x595 * sensor_pt[0]) + (-1 * x597 * sensor_pt[2]) + (x600 * sensor_pt[1]);
	const FLT x602 = -1 * x554;
	const FLT x603 = -1 * x555;
	const FLT x604 = x558 + x602 + x603;
	const FLT x605 = (x597 * sensor_pt[1]) + (-1 * x604 * sensor_pt[0]) + (x600 * sensor_pt[2]);
	const FLT x606 = (x605 * x570) + (x604 * x571) + (-1 * x595 * x562) + (-1 * x601 * x561);
	const FLT x607 = (x604 * sensor_pt[2]) + (-1 * x595 * sensor_pt[1]) + (x600 * sensor_pt[0]);
	const FLT x608 = (x595 * x575) + (x607 * x561) + (-1 * x605 * x573) + (-1 * x597 * x571);
	const FLT x609 = (-1 * x607 * x570) + (x597 * x562) + (-1 * x604 * x575) + (x601 * x573);
	const FLT x610 = (x609 * x105) + (x606 * x119) + (-1 * x608 * x110);
	const FLT x611 = (-1 * x609 * x119) + (x608 * x108) + (x606 * x105);
	const FLT x612 = (x610 * x294) + x608 + (-1 * x611 * x277);
	const FLT x613 = (-1 * x606 * x108) + (x609 * x110) + (x608 * x105);
	const FLT x614 = x606 + (x613 * x277) + (-1 * x610 * x310);
	const FLT x615 = x609 + (x611 * x310) + (-1 * x613 * x294);
	const FLT x616 = (x615 * x133) + (x614 * x122);
	const FLT x617 = (-1 * x195 * (x616 + (x612 * x194))) + (x612 * x163);
	const FLT x618 = (-1 * x612 * x148) + (x616 * x141);
	const FLT x619 = ((x614 * x155) + (-1 * x615 * x154)) * x156;
	const FLT x620 = (-1 * x619) + (x618 * x153);
	const FLT x621 = x617 * x201;
	const FLT x622 = (x164 * (x621 + (-1 * x617 * x230))) + (x617 * x229);
	const FLT x623 = (x617 * x233) + (x622 * x164);
	const FLT x624 =
		x619 +
		(-1 * x213 *
		 (x618 + (x623 * x211) +
		  (-1 * x208 *
		   ((x184 *
			 ((x623 * x164) +
			  (x164 * (x623 + (x164 * (x622 + (x164 * ((-1 * x617 * x254) + x621 + (x617 * x238))) + (x617 * x239))) +
					   (x617 * x240))) +
			  (x617 * x199) + (x617 * x200))) +
			(-1 * x620 * x193))) +
		  (x617 * x249) + (-1 * x620 * x191)));
	const FLT x625 = (-1 * x557) + x556;
	const FLT x626 = x625 + x555 + x602;
	const FLT x627 = -1 * x564;
	const FLT x628 = x627 + x567 + x593;
	const FLT x629 = x540 + (-1 * x542);
	const FLT x630 = x629 + x538 + x599;
	const FLT x631 = (-1 * x626 * sensor_pt[2]) + (x630 * sensor_pt[0]) + (x628 * sensor_pt[1]);
	const FLT x632 = x596 + x548 + x546;
	const FLT x633 = (x626 * sensor_pt[1]) + (-1 * x632 * sensor_pt[0]) + (x628 * sensor_pt[2]);
	const FLT x634 = (x633 * x570) + (-1 * x631 * x561) + (x632 * x571) + (-1 * x630 * x562);
	const FLT x635 = (x632 * sensor_pt[2]) + (-1 * x630 * sensor_pt[1]) + (x628 * sensor_pt[0]);
	const FLT x636 = (-1 * x626 * x571) + (-1 * x633 * x573) + (x630 * x575) + (x635 * x561);
	const FLT x637 = x636 * x104;
	const FLT x638 = (x626 * x562) + (-1 * x632 * x575) + (x631 * x573) + (-1 * x635 * x570);
	const FLT x639 = x638 * x104;
	const FLT x640 = (x26 * x639) + (x634 * x119) + (-1 * x27 * x637);
	const FLT x641 = (-1 * x22 * x639) + (x634 * x105) + (x636 * x108);
	const FLT x642 = x636 + (x640 * x294) + (-1 * x641 * x277);
	const FLT x643 = (-1 * x634 * x108) + (x26 * x637) + (x27 * x639);
	const FLT x644 = x634 + (x643 * x277) + (-1 * x640 * x310);
	const FLT x645 = x638 + (x641 * x310) + (-1 * x643 * x294);
	const FLT x646 = (x645 * x133) + (x644 * x122);
	const FLT x647 = (-1 * x195 * (x646 + (x642 * x194))) + (x642 * x163);
	const FLT x648 = (-1 * x642 * x148) + (x646 * x141);
	const FLT x649 = ((x644 * x155) + (-1 * x645 * x154)) * x156;
	const FLT x650 = (-1 * x649) + (x648 * x153);
	const FLT x651 = x647 * x201;
	const FLT x652 = (x164 * (x651 + (-1 * x647 * x230))) + (x647 * x229);
	const FLT x653 = (x647 * x233) + (x652 * x164);
	const FLT x654 =
		x649 +
		(-1 * x213 *
		 (x648 + (-1 * x650 * x191) + (x653 * x211) + (x647 * x249) +
		  (-1 * x208 *
		   ((x184 * ((x647 * x199) + (x653 * x164) + (x647 * x200) +
					 (x164 * ((x164 * (x652 + (x164 * ((-1 * x647 * x254) + x651 + (x647 * x238))) + (x647 * x239))) +
							  x653 + (x647 * x240))))) +
			(-1 * x650 * x193)))));
	const FLT x655 = x629 + x598 + x536;
	const FLT x656 = x625 + x554 + x603;
	const FLT x657 = x594 + x627 + x563;
	const FLT x658 = x552 + x548 + x545;
	const FLT x659 = (x658 * sensor_pt[0]) + (x656 * sensor_pt[1]) + (-1 * x657 * sensor_pt[2]);
	const FLT x660 = (x655 * sensor_pt[2]) + (-1 * x658 * sensor_pt[1]) + (x656 * sensor_pt[0]);
	const FLT x661 = (-1 * x660 * x570) + (x657 * x562) + (-1 * x655 * x575) + (x659 * x573);
	const FLT x662 = (x657 * sensor_pt[1]) + (-1 * x655 * sensor_pt[0]) + (x656 * sensor_pt[2]);
	const FLT x663 = (x658 * x575) + (x660 * x561) + (-1 * x662 * x573) + (-1 * x657 * x571);
	const FLT x664 = (-1 * x659 * x561) + (-1 * x658 * x562) + (x655 * x571) + (x662 * x570);
	const FLT x665 = (-1 * x664 * x108) + (x661 * x110) + (x663 * x105);
	const FLT x666 = 2 * ((x661 * x105) + (x664 * x119) + (-1 * x663 * x110));
	const FLT x667 = x664 + (x665 * x277) + (-1 * x666 * x119);
	const FLT x668 = (-1 * x661 * x119) + (x663 * x108) + (x664 * x105);
	const FLT x669 = (x668 * x310) + x661 + (-1 * x665 * x294);
	const FLT x670 = (x669 * x133) + (x667 * x122);
	const FLT x671 = x663 + (x666 * x110) + (-1 * x668 * x277);
	const FLT x672 = (-1 * x671 * x148) + (x670 * x141);
	const FLT x673 = ((x667 * x155) + (-1 * x669 * x154)) * x156;
	const FLT x674 = (-1 * x673) + (x672 * x153);
	const FLT x675 = (-1 * x195 * (x670 + (x671 * x194))) + (x671 * x163);
	const FLT x676 = x675 * x201;
	const FLT x677 = (x164 * (x676 + (-1 * x675 * x230))) + (x675 * x229);
	const FLT x678 = (x675 * x233) + (x677 * x164);
	const FLT x679 =
		x673 +
		(-1 * x213 *
		 (x672 + (x675 * x249) +
		  (-1 * x208 *
		   ((x184 * ((x678 * x164) +
					 (x164 * ((x164 * (x677 + (x164 * ((-1 * x675 * x254) + x676 + (x675 * x238))) + (x675 * x239))) +
							  x678 + (x675 * x240))) +
					 (x675 * x200) + (x675 * x199))) +
			(-1 * x674 * x193))) +
		  (x678 * x211) + (-1 * x674 * x191)));
	const FLT x680 = -1 * x82;
	const FLT x681 = dt * dt * dt;
	const FLT x682 = 1. / (x68 * sqrt(x68));
	const FLT x683 = x682 * x681;
	const FLT x684 = x62 * x683;
	const FLT x685 = x89 * x78;
	const FLT x686 = x35 * x685;
	const FLT x687 = x684 * x686;
	const FLT x688 = dt * dt * dt * dt;
	const FLT x689 = (x62 * x62 * x62) * x688;
	const FLT x690 = 2 * x72 * (1. / (x68 * x68));
	const FLT x691 = 1.0 * x71 * x75;
	const FLT x692 = x691 * x682;
	const FLT x693 = x692 * x688;
	const FLT x694 = x60 * x62;
	const FLT x695 = x690 * x688;
	const FLT x696 = x66 * x695;
	const FLT x697 = x62 * x59;
	const FLT x698 = 2 * x74;
	const FLT x699 = x66 * x693;
	const FLT x700 = x80 * x691;
	const FLT x701 = (x692 * x689) + (x62 * x699) + (x694 * x693) + (-1 * x690 * x689) + (-1 * x694 * x695) +
					 (x698 * x697) + (-1 * x697 * x700) + (-1 * x62 * x696);
	const FLT x702 = 1.0 / 2.0 * (1. / (x76 * sqrt(x76)));
	const FLT x703 = x75 * x702;
	const FLT x704 = x701 * x703;
	const FLT x705 = x83 * x78;
	const FLT x706 = 0.5 * x80;
	const FLT x707 = x697 * x706;
	const FLT x708 = x81 * x71 * x702;
	const FLT x709 = x62 * x708;
	const FLT x710 = x58 * x701;
	const FLT x711 = 0.5 * x73 * x681;
	const FLT x712 = x62 * x35;
	const FLT x713 = x711 * x712;
	const FLT x714 = x92 * x713;
	const FLT x715 = x35 * x708;
	const FLT x716 = x715 * x701;
	const FLT x717 = x63 * x683;
	const FLT x718 = x65 * x708;
	const FLT x719 = x86 * x701;
	const FLT x720 = x63 * x711;
	const FLT x721 = x86 * x78;
	const FLT x722 = x684 * x721;
	const FLT x723 = x88 * x711;
	const FLT x724 = x62 * x65;
	const FLT x725 = (x724 * x723) + (-1 * x65 * x722);
	const FLT x726 = (-1 * x718 * x719) + (-1 * x89 * x716) + (-1 * x98 * x720) + (-1 * x687) + x725 +
					 (-1 * x707 * x705) + x680 + (x79 * x717) + x714 + (-1 * x83 * x704) + (x710 * x709);
	const FLT x727 = x65 * x711;
	const FLT x728 = x98 * x727;
	const FLT x729 = x62 * x728;
	const FLT x730 = x35 * x705;
	const FLT x731 = x684 * x730;
	const FLT x732 = x65 * x684;
	const FLT x733 = x79 * x732;
	const FLT x734 = x95 * x711;
	const FLT x735 = x712 * x734;
	const FLT x736 = (-1 * x709 * x719) + (-1 * x733) + (-1 * x710 * x718) + x731 + (x63 * x723) + x93 + x729 +
					 (-1 * x735) + (-1 * x89 * x704) + (x83 * x716) + (-1 * x717 * x721) + (-1 * x685 * x707);
	const FLT x737 = -1 * x90;
	const FLT x738 = x732 * x705;
	const FLT x739 = x83 * x701;
	const FLT x740 = x734 * x724;
	const FLT x741 = x89 * x701;
	const FLT x742 = x79 * x35;
	const FLT x743 = (x684 * x742) + (-1 * x98 * x713);
	const FLT x744 = x743 + (x685 * x717) + (-1 * x707 * x721) + (-1 * x740) + (-1 * x92 * x720) + (-1 * x86 * x704) +
					 (x709 * x741) + x738 + x737 + (x718 * x739) + (x58 * x716);
	const FLT x745 = x92 * x727;
	const FLT x746 = x65 * x685;
	const FLT x747 = (x684 * x746) + (-1 * x62 * x745);
	const FLT x748 = (-1 * x35 * x722) + (x712 * x723);
	const FLT x749 = x748 + (-1 * x709 * x739) + (-1 * x58 * x704) + (-1 * x717 * x705) + (x718 * x741) + x85 +
					 (-1 * x79 * x707) + x747 + (x63 * x734) + (-1 * x715 * x719);
	const FLT x750 = (-1 * x736 * sensor_pt[2]) + (x749 * sensor_pt[0]) + (x744 * sensor_pt[1]);
	const FLT x751 = (x726 * sensor_pt[2]) + (-1 * x749 * sensor_pt[1]) + (x744 * sensor_pt[0]);
	const FLT x752 = (-1 * x751 * x570) + (x750 * x573) + (-1 * x726 * x575) + (x736 * x562);
	const FLT x753 = x752 * x104;
	const FLT x754 = (x736 * sensor_pt[1]) + (-1 * x726 * sensor_pt[0]) + (x744 * sensor_pt[2]);
	const FLT x755 = (x749 * x575) + (-1 * x754 * x573) + (-1 * x736 * x571) + (x751 * x561);
	const FLT x756 = (x726 * x571) + (-1 * x750 * x561) + (x754 * x570) + (-1 * x749 * x562);
	const FLT x757 = (-1 * x756 * x108) + (x27 * x753) + (x755 * x105);
	const FLT x758 = (x26 * x753) + (x756 * x119) + (-1 * x755 * x110);
	const FLT x759 = x756 + (x757 * x277) + (-1 * x758 * x310);
	const FLT x760 = (-1 * x22 * x753) + (x755 * x108) + (x756 * x105);
	const FLT x761 = x752 + (-1 * x757 * x294) + (x760 * x310);
	const FLT x762 = (x761 * x133) + (x759 * x122);
	const FLT x763 = x755 + (-1 * x760 * x277) + (x758 * x294);
	const FLT x764 = (-1 * x763 * x148) + (x762 * x141);
	const FLT x765 = ((x759 * x155) + (-1 * x761 * x154)) * x156;
	const FLT x766 = (-1 * x765) + (x764 * x153);
	const FLT x767 = (-1 * x195 * (x762 + (x763 * x194))) + (x763 * x163);
	const FLT x768 = x767 * x201;
	const FLT x769 = (x164 * (x768 + (-1 * x767 * x230))) + (x767 * x229);
	const FLT x770 = (x767 * x233) + (x769 * x164);
	const FLT x771 =
		x765 +
		(-1 * x213 *
		 ((x770 * x211) + (-1 * x766 * x191) + x764 +
		  (-1 * x208 *
		   ((x184 *
			 ((x770 * x164) + (x767 * x200) +
			  (x164 * (x770 + (x164 * ((x164 * (x768 + (-1 * x767 * x254) + (x767 * x238))) + x769 + (x767 * x239))) +
					   (x767 * x240))) +
			  (x767 * x199))) +
			(-1 * x766 * x193))) +
		  (x767 * x249)));
	const FLT x772 = x65 * x695;
	const FLT x773 = x65 * x693;
	const FLT x774 = x59 * x700;
	const FLT x775 = x65 * x65 * x65;
	const FLT x776 = x59 * x698;
	const FLT x777 = (x65 * x776) + (x693 * x775) + (-1 * x695 * x775) + (x63 * x773) + (x60 * x773) +
					 (-1 * x60 * x772) + (-1 * x63 * x772) + (-1 * x65 * x774);
	const FLT x778 = x777 * x703;
	const FLT x779 = x66 * x683;
	const FLT x780 = x35 * x745;
	const FLT x781 = x777 * x718;
	const FLT x782 = x35 * x683;
	const FLT x783 = x782 * x746;
	const FLT x784 = x777 * x715;
	const FLT x785 = x59 * x706;
	const FLT x786 = x65 * x785;
	const FLT x787 = x777 * x709;
	const FLT x788 = x93 + (-1 * x779 * x721) + x733 + x780 + (-1 * x729) + (-1 * x83 * x778) + (-1 * x86 * x781) +
					 (x58 * x787) + (-1 * x783) + (x66 * x723) + (-1 * x89 * x784) + (-1 * x705 * x786);
	const FLT x789 = x35 * x728;
	const FLT x790 = x65 * x782;
	const FLT x791 = x79 * x790;
	const FLT x792 = -1 * x85;
	const FLT x793 = x721 * x785;
	const FLT x794 = (-1 * x86 * x778) + x792 + (x58 * x784) + x747 + (-1 * x66 * x734) + (-1 * x65 * x793) +
					 (x89 * x787) + (-1 * x789) + (x779 * x705) + (x83 * x781) + x791;
	const FLT x795 = x66 * x711;
	const FLT x796 = x65 * x35;
	const FLT x797 = (x65 * x683 * x730) + (-1 * x734 * x796);
	const FLT x798 = x725 + x797 + (-1 * x58 * x781) + (-1 * x785 * x746) + (x83 * x784) + (-1 * x86 * x787) +
					 (x98 * x795) + (-1 * x89 * x778) + (-1 * x79 * x779) + x82;
	const FLT x799 = (x798 * sensor_pt[1]) + (-1 * x788 * sensor_pt[0]) + (x794 * sensor_pt[2]);
	const FLT x800 = (x723 * x796) + (-1 * x721 * x790);
	const FLT x801 = (-1 * x83 * x787) + (x685 * x779) + (-1 * x92 * x795) + (-1 * x79 * x786) + x737 + (x89 * x781) +
					 (-1 * x86 * x784) + x800 + (-1 * x738) + (-1 * x58 * x778) + x740;
	const FLT x802 = (x788 * sensor_pt[2]) + (-1 * x801 * sensor_pt[1]) + (x794 * sensor_pt[0]);
	const FLT x803 = (x575 * x801) + (-1 * x799 * x573) + (x561 * x802) + (-1 * x798 * x571);
	const FLT x804 = x803 * x104;
	const FLT x805 = (x801 * sensor_pt[0]) + (-1 * x798 * sensor_pt[2]) + (x794 * sensor_pt[1]);
	const FLT x806 = (x788 * x571) + (x799 * x570) + (-1 * x561 * x805) + (-1 * x562 * x801);
	const FLT x807 = x806 * x104;
	const FLT x808 = (x573 * x805) + (x798 * x562) + (-1 * x788 * x575) + (-1 * x570 * x802);
	const FLT x809 = (x808 * x105) + (-1 * x27 * x804) + (x22 * x807);
	const FLT x810 = (-1 * x808 * x119) + (x803 * x108) + (x26 * x807);
	const FLT x811 = x803 + (x809 * x294) + (-1 * x810 * x277);
	const FLT x812 = (-1 * x806 * x108) + (x808 * x110) + (x26 * x804);
	const FLT x813 = (x812 * x277) + x806 + (-1 * x809 * x310);
	const FLT x814 = (x810 * x310) + x808 + (-1 * x812 * x294);
	const FLT x815 = (x814 * x133) + (x813 * x122);
	const FLT x816 = (-1 * x195 * (x815 + (x811 * x194))) + (x811 * x163);
	const FLT x817 = x816 * x201;
	const FLT x818 = (x164 * (x817 + (-1 * x816 * x230))) + (x816 * x229);
	const FLT x819 = (x816 * x233) + (x818 * x164);
	const FLT x820 = (-1 * x811 * x148) + (x815 * x141);
	const FLT x821 = ((x813 * x155) + (-1 * x814 * x154)) * x156;
	const FLT x822 = x190 * ((-1 * x821) + (x820 * x153));
	const FLT x823 =
		x821 +
		(-1 * x213 *
		 ((-1 * x822 * x189) + (x816 * x249) + x820 +
		  (-1 * x208 *
		   ((x184 *
			 ((x819 * x164) + (x816 * x200) + (x816 * x199) +
			  (x164 * (x819 + (x164 * (x818 + (x164 * ((-1 * x816 * x254) + x817 + (x816 * x238))) + (x816 * x239))) +
					   (x816 * x240))))) +
			(-1 * x822 * x192))) +
		  (x819 * x211)));
	const FLT x824 = x63 * x35;
	const FLT x825 = x35 * x35 * x35;
	const FLT x826 = (x693 * x825) + (x693 * x824) + (-1 * x35 * x774) + (-1 * x695 * x825) + (-1 * x695 * x824) +
					 (x35 * x776) + (x35 * x699) + (-1 * x35 * x696);
	const FLT x827 = x715 * x826;
	const FLT x828 = x60 * x683;
	const FLT x829 = x718 * x826;
	const FLT x830 = x60 * x711;
	const FLT x831 = x703 * x826;
	const FLT x832 = x709 * x826;
	const FLT x833 = x687 + (-1 * x98 * x830) + (x83 * x829) + (-1 * x86 * x831) + x680 + (x89 * x832) + (-1 * x714) +
					 (x79 * x828) + x797 + (x58 * x827) + (-1 * x35 * x793);
	const FLT x834 = x86 * x826;
	const FLT x835 = (x58 * x832) + x800 + (x92 * x830) + (-1 * x718 * x834) + x90 + (-1 * x89 * x827) +
					 (-1 * x730 * x785) + x743 + (-1 * x83 * x831) + (-1 * x685 * x828);
	const FLT x836 = x748 + x789 + (-1 * x58 * x829) + (-1 * x95 * x830) + (x83 * x827) + (-1 * x89 * x831) +
					 (-1 * x686 * x785) + (-1 * x709 * x834) + x792 + (x705 * x828) + (-1 * x791);
	const FLT x837 = (x836 * sensor_pt[1]) + (x833 * sensor_pt[2]) + (-1 * x835 * sensor_pt[0]);
	const FLT x838 = (-1 * x780) + (x89 * x829) + (x88 * x830) + x735 + (-1 * x715 * x834) + (-1 * x58 * x831) + x783 +
					 (-1 * x785 * x742) + (-1 * x731) + (-1 * x721 * x828) + x93 + (-1 * x83 * x832);
	const FLT x839 = (x838 * sensor_pt[0]) + (-1 * x836 * sensor_pt[2]) + (x833 * sensor_pt[1]);
	const FLT x840 = (x571 * x835) + (-1 * x562 * x838) + (x570 * x837) + (-1 * x561 * x839);
	const FLT x841 = x840 * x104;
	const FLT x842 = (x835 * sensor_pt[2]) + (-1 * x838 * sensor_pt[1]) + (x833 * sensor_pt[0]);
	const FLT x843 = (x575 * x838) + (x561 * x842) + (-1 * x573 * x837) + (-1 * x571 * x836);
	const FLT x844 = (x562 * x836) + (x573 * x839) + (-1 * x575 * x835) + (-1 * x570 * x842);
	const FLT x845 = x844 * x104;
	const FLT x846 = (x26 * x845) + (x22 * x841) + (-1 * x843 * x110);
	const FLT x847 = (-1 * x22 * x845) + (x26 * x841) + (x843 * x108);
	const FLT x848 = x843 + (x846 * x294) + (-1 * x847 * x277);
	const FLT x849 = (-1 * x840 * x108) + (x843 * x105) + (x27 * x845);
	const FLT x850 = x840 + (x849 * x277) + (-1 * x846 * x310);
	const FLT x851 = x844 + (x847 * x310) + (-1 * x849 * x294);
	const FLT x852 = (x851 * x133) + (x850 * x122);
	const FLT x853 = (-1 * x195 * (x852 + (x848 * x194))) + (x848 * x163);
	const FLT x854 = (-1 * x848 * x148) + (x852 * x141);
	const FLT x855 = ((x850 * x155) + (-1 * x851 * x154)) * x156;
	const FLT x856 = (-1 * x855) + (x854 * x153);
	const FLT x857 = x853 * x201;
	const FLT x858 = (x164 * (x857 + (-1 * x853 * x230))) + (x853 * x229);
	const FLT x859 = (x853 * x233) + (x858 * x164);
	const FLT x860 =
		x855 +
		(-1 * x213 *
		 ((x859 * x211) + x854 + (-1 * x856 * x191) + (x853 * x249) +
		  (-1 * x208 *
		   ((x184 * ((x164 * ((x164 * (x858 + (x164 * ((-1 * x853 * x254) + x857 + (x853 * x238))) + (x853 * x239))) +
							  x859 + (x853 * x240))) +
					 (x853 * x199) + (x859 * x164) + (x853 * x200))) +
			(-1 * x856 * x193)))));
	const FLT x861 = dt * x144;
	const FLT x862 = dt * x142;
	const FLT x863 = (-1 * x862) + x861;
	const FLT x864 = -1 * dt * x32;
	const FLT x865 = -1 * dt * x33;
	const FLT x866 = x865 + dt + x864;
	const FLT x867 = dt * x127;
	const FLT x868 = dt * x124;
	const FLT x869 = x868 + x867;
	const FLT x870 = (x869 * x133) + (x866 * x122);
	const FLT x871 = (-1 * x195 * (x870 + (x863 * x194))) + (x863 * x163);
	const FLT x872 = (-1 * x863 * x148) + (x870 * x141);
	const FLT x873 = ((x866 * x155) + (-1 * x869 * x154)) * x156;
	const FLT x874 = (-1 * x873) + (x872 * x153);
	const FLT x875 = x871 * x201;
	const FLT x876 = (x164 * (x875 + (-1 * x871 * x230))) + (x871 * x229);
	const FLT x877 = (x871 * x233) + (x876 * x164);
	const FLT x878 =
		x873 +
		(-1 * x213 *
		 (x872 + (x877 * x211) + (-1 * x874 * x191) + (x871 * x249) +
		  (-1 * x208 *
		   ((x184 * ((x877 * x164) + (x871 * x200) +
					 (x164 * ((x164 * (x876 + (x164 * ((-1 * x871 * x254) + x875 + (x871 * x238))) + (x871 * x239))) +
							  x877 + (x871 * x240))) +
					 (x871 * x199))) +
			(-1 * x874 * x193)))));
	const FLT x879 = x861 + x862;
	const FLT x880 = dt * x223;
	const FLT x881 = dt * x222;
	const FLT x882 = (-1 * x881) + x880;
	const FLT x883 = (x882 * x133) + (x879 * x122);
	const FLT x884 = (-1 * dt * x217) + dt;
	const FLT x885 = x884 + x865;
	const FLT x886 = (-1 * x885 * x148) + (x883 * x141);
	const FLT x887 = ((x879 * x155) + (-1 * x882 * x154)) * x156;
	const FLT x888 = (-1 * x887) + (x886 * x153);
	const FLT x889 = (-1 * x195 * (x883 + (x885 * x194))) + (x885 * x163);
	const FLT x890 = x889 * x201;
	const FLT x891 = (x164 * (x890 + (-1 * x889 * x230))) + (x889 * x229);
	const FLT x892 = (x889 * x233) + (x891 * x164);
	const FLT x893 =
		x887 +
		(-1 * x213 *
		 ((-1 * x888 * x191) + x886 + (x892 * x211) +
		  (-1 * x208 *
		   ((x184 *
			 ((x892 * x164) + (x889 * x200) +
			  (x164 * (x892 + (x164 * (x891 + (x164 * ((-1 * x889 * x254) + x890 + (x889 * x238))) + (x889 * x239))) +
					   (x889 * x240))) +
			  (x889 * x199))) +
			(-1 * x888 * x193))) +
		  (x889 * x249)));
	const FLT x894 = x880 + x881;
	const FLT x895 = (-1 * x867) + x868;
	const FLT x896 = x884 + x864;
	const FLT x897 = (x896 * x133) + (x895 * x122);
	const FLT x898 = (-1 * x195 * (x897 + (x894 * x194))) + (x894 * x163);
	const FLT x899 = (-1 * x894 * x148) + (x897 * x141);
	const FLT x900 = ((x895 * x155) + (-1 * x896 * x154)) * x156;
	const FLT x901 = (-1 * x900) + (x899 * x153);
	const FLT x902 = x898 * x201;
	const FLT x903 = (x164 * (x902 + (-1 * x898 * x230))) + (x898 * x229);
	const FLT x904 = (x898 * x233) + (x903 * x164);
	const FLT x905 =
		x900 +
		(-1 * x213 *
		 (x899 + (x904 * x211) +
		  (-1 * x208 *
		   ((x184 *
			 ((x904 * x164) +
			  (x164 * (x904 + (x164 * (x903 + (x164 * ((-1 * x898 * x254) + x902 + (x898 * x238))) + (x898 * x239))) +
					   (x898 * x240))) +
			  (x898 * x199) + (x898 * x200))) +
			(-1 * x901 * x193))) +
		  (x898 * x249) + (-1 * x901 * x191)));
	const FLT x906 = x207 * x185;
	const FLT x907 = ((-1 * x906) + x189) * x213;
	const FLT x908 = ((x906 * x181) + (-1 * x181 * x189)) * x213;
	const FLT x909 = x213 * ((-1 * x906 * x190) + x191);
	const FLT x910 = x160 * x197;
	const FLT x911 = x910 * x183 * x139;
	const FLT x912 = -1 * x911 * x201;
	const FLT x913 = (x164 * (x912 + (x911 * x230))) + (-1 * x911 * x229);
	const FLT x914 = (-1 * x911 * x233) + (x913 * x164);
	const FLT x915 = x139 * x147 * (1 + x152);
	const FLT x916 = x915 * x153;
	const FLT x917 =
		x213 *
		((-1 * x208 *
		  ((-1 * x916 * x193) +
		   (x184 *
			((x914 * x164) + (-1 * x911 * x200) +
			 (x164 *
			  (x914 + (x164 * (x913 + (x164 * ((x911 * x254) + x912 + (-1 * x911 * x238))) + (-1 * x911 * x239))) +
			   (-1 * x911 * x240))) +
			 (-1 * x911 * x199))) +
		   x183 + (-1 * x161 * x176 * x182))) +
		 x915 + (-1 * x910 * x248 * x184 * x194) + (x914 * x211) + (-1 * x916 * x191));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[0]) / sizeof(FLT),
						x214 + (x214 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[1]) / sizeof(FLT),
						x241 + (x216 * x241));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Pos[2]) / sizeof(FLT),
						x257 + (x216 * x257));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[0]) / sizeof(FLT),
						x343 + (x216 * x343));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[1]) / sizeof(FLT),
						x394 + (x216 * x394));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[2]) / sizeof(FLT),
						x424 + (x424 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Lighthouse.Rot[3]) / sizeof(FLT),
						x447 + (x447 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Acc[0]) / sizeof(FLT), x469 + (x469 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Acc[1]) / sizeof(FLT), x484 + (x484 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Acc[2]) / sizeof(FLT), x496 + (x496 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[0]) / sizeof(FLT),
						x510 + (x510 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[1]) / sizeof(FLT),
						x523 + (x523 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Pos[2]) / sizeof(FLT),
						x535 + (x535 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[0]) / sizeof(FLT),
						x592 + (x592 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[1]) / sizeof(FLT),
						x624 + (x624 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[2]) / sizeof(FLT),
						x654 + (x654 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Pose.Rot[3]) / sizeof(FLT),
						x679 + (x679 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x771 + (x771 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x823 + (x823 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x860 + (x860 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[0]) / sizeof(FLT),
						x878 + (x878 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[1]) / sizeof(FLT),
						x893 + (x893 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, Object.Velocity.Pos[2]) / sizeof(FLT),
						x905 + (x905 * x216));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD1.curve) / sizeof(FLT),
						(-1 * x907 * x216) + (-1 * x907));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD1.gibmag) / sizeof(FLT), sin(x215));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD1.gibpha) / sizeof(FLT), x216);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD1.ogeemag) / sizeof(FLT),
						(-1 * x908 * x216) + (-1 * x908));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD1.ogeephase) / sizeof(FLT),
						(-1 * x909 * x216) + (-1 * x909));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD1.phase) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanModel, BSD1.tilt) / sizeof(FLT),
						(-1 * x917 * x216) + (-1 * x917));
}

// Full version Jacobian of SurviveJointKalmanErrorModel_LightMeas_y_gen2 wrt [(*_x0).Lighthouse.Pos[0],
// (*_x0).Lighthouse.Pos[1], (*_x0).Lighthouse.Pos[2], (*_x0).Lighthouse.Rot[0], (*_x0).Lighthouse.Rot[1],
// (*_x0).Lighthouse.Rot[2], (*_x0).Lighthouse.Rot[3], (*_x0).Object.Acc[0], (*_x0).Object.Acc[1], (*_x0).Object.Acc[2],
// (*_x0).Object.IMUBias.AccBias[0], (*_x0).Object.IMUBias.AccBias[1], (*_x0).Object.IMUBias.AccBias[2],
// (*_x0).Object.IMUBias.AccScale[0], (*_x0).Object.IMUBias.AccScale[1], (*_x0).Object.IMUBias.AccScale[2],
// (*_x0).Object.IMUBias.GyroBias[0], (*_x0).Object.IMUBias.GyroBias[1], (*_x0).Object.IMUBias.GyroBias[2],
// (*_x0).Object.IMUBias.IMUCorrection[0], (*_x0).Object.IMUBias.IMUCorrection[1],
// (*_x0).Object.IMUBias.IMUCorrection[2], (*_x0).Object.IMUBias.IMUCorrection[3], (*_x0).Object.Pose.Pos[0],
// (*_x0).Object.Pose.Pos[1], (*_x0).Object.Pose.Pos[2], (*_x0).Object.Pose.Rot[0], (*_x0).Object.Pose.Rot[1],
// (*_x0).Object.Pose.Rot[2], (*_x0).Object.Pose.Rot[3], (*_x0).Object.Velocity.AxisAngleRot[0],
// (*_x0).Object.Velocity.AxisAngleRot[1], (*_x0).Object.Velocity.AxisAngleRot[2], (*_x0).Object.Velocity.Pos[0],
// (*_x0).Object.Velocity.Pos[1], (*_x0).Object.Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f4d1c38e7c0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c394af0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c394460>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3943d0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3944c0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c394430>, <cnkalman.codegen.WrapMember object at 0x7f4d1c394130>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3942e0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c394160>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c394370>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38e0a0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c394250>, <cnkalman.codegen.WrapMember object at 0x7f4d1c38e850>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c394070>]

static inline void SurviveJointKalmanErrorModel_LightMeas_y_gen2_jac_x0_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveJointKalmanModel *_x0,
	const SurviveJointKalmanErrorModel *error_model, const FLT *sensor_pt) {
	if (hx != 0) {
		hx->data[0] = SurviveJointKalmanErrorModel_LightMeas_y_gen2(dt, _x0, error_model, sensor_pt);
	}
	if (Hx != 0) {
		SurviveJointKalmanErrorModel_LightMeas_y_gen2_jac_x0(Hx, dt, _x0, error_model, sensor_pt);
	}
}
// Jacobian of SurviveJointKalmanErrorModel_LightMeas_y_gen2 wrt [(*error_model).Lighthouse.AxisAngleRot[0],
// (*error_model).Lighthouse.AxisAngleRot[1], (*error_model).Lighthouse.AxisAngleRot[2],
// (*error_model).Lighthouse.Pos[0], (*error_model).Lighthouse.Pos[1], (*error_model).Lighthouse.Pos[2],
// (*error_model).Object.Acc[0], (*error_model).Object.Acc[1], (*error_model).Object.Acc[2],
// (*error_model).Object.IMUBias.AccBias[0], (*error_model).Object.IMUBias.AccBias[1],
// (*error_model).Object.IMUBias.AccBias[2], (*error_model).Object.IMUBias.AccScale[0],
// (*error_model).Object.IMUBias.AccScale[1], (*error_model).Object.IMUBias.AccScale[2],
// (*error_model).Object.IMUBias.GyroBias[0], (*error_model).Object.IMUBias.GyroBias[1],
// (*error_model).Object.IMUBias.GyroBias[2], (*error_model).Object.IMUBias.IMUCorrection[0],
// (*error_model).Object.IMUBias.IMUCorrection[1], (*error_model).Object.IMUBias.IMUCorrection[2],
// (*error_model).Object.Pose.AxisAngleRot[0], (*error_model).Object.Pose.AxisAngleRot[1],
// (*error_model).Object.Pose.AxisAngleRot[2], (*error_model).Object.Pose.Pos[0], (*error_model).Object.Pose.Pos[1],
// (*error_model).Object.Pose.Pos[2], (*error_model).Object.Velocity.AxisAngleRot[0],
// (*error_model).Object.Velocity.AxisAngleRot[1], (*error_model).Object.Velocity.AxisAngleRot[2],
// (*error_model).Object.Velocity.Pos[0], (*error_model).Object.Velocity.Pos[1], (*error_model).Object.Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f4d1c396340>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3966d0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c396280>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3967f0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3962e0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c396850>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3965e0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c396550>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c396760>, <cnkalman.codegen.WrapMember object at 0x7f4d1c396490>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3961f0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3966a0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c396370>, <cnkalman.codegen.WrapMember object at 0x7f4d1c396400>]
static inline void SurviveJointKalmanErrorModel_LightMeas_y_gen2_jac_error_model(
	CnMat *Hx, const FLT dt, const SurviveJointKalmanModel *_x0, const SurviveJointKalmanErrorModel *error_model,
	const FLT *sensor_pt) {
	const FLT x0 = (*_x0).Lighthouse.Pos[2] + (*error_model).Lighthouse.Pos[2];
	const FLT x1 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[2];
	const FLT x2 = sin(x1);
	const FLT x3 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[0];
	const FLT x4 = sin(x3);
	const FLT x5 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[1];
	const FLT x6 = sin(x5);
	const FLT x7 = x4 * x6;
	const FLT x8 = x2 * x7;
	const FLT x9 = cos(x3);
	const FLT x10 = cos(x5);
	const FLT x11 = cos(x1);
	const FLT x12 = x11 * x10;
	const FLT x13 = x9 * x12;
	const FLT x14 = x13 + x8;
	const FLT x15 = x7 * x11;
	const FLT x16 = x2 * x10;
	const FLT x17 = x9 * x16;
	const FLT x18 = x17 + (-1 * x15);
	const FLT x19 = x4 * x16;
	const FLT x20 = x6 * x9;
	const FLT x21 = x20 * x11;
	const FLT x22 = x21 + x19;
	const FLT x23 = x2 * x20;
	const FLT x24 = x4 * x12;
	const FLT x25 = x24 + (-1 * x23);
	const FLT x26 = (x14 * x14) + (x18 * x18) + (x25 * x25) + (x22 * x22);
	const FLT x27 = 1. / sqrt(x26);
	const FLT x28 = x27 * (*_x0).Lighthouse.Rot[3];
	const FLT x29 = x27 * (*_x0).Lighthouse.Rot[2];
	const FLT x30 = x27 * (*_x0).Lighthouse.Rot[0];
	const FLT x31 = x27 * (*_x0).Lighthouse.Rot[1];
	const FLT x32 = (x31 * x22) + (x30 * x18) + (x28 * x14) + (-1 * x25 * x29);
	const FLT x33 = x32 * x32;
	const FLT x34 = (-1 * x31 * x18) + (x30 * x22) + (x25 * x28) + (x29 * x14);
	const FLT x35 = x34 * x34;
	const FLT x36 = (-1 * x31 * x25) + (x30 * x14) + (-1 * x28 * x18) + (-1 * x22 * x29);
	const FLT x37 = (x31 * x14) + (x29 * x18) + (-1 * x22 * x28) + (x30 * x25);
	const FLT x38 = x37 * x37;
	const FLT x39 = x38 + (x36 * x36) + x33 + x35;
	const FLT x40 = 1. / sqrt(x39);
	const FLT x41 = x40 * x36;
	const FLT x42 = (*_x0).Lighthouse.Pos[1] + (*error_model).Lighthouse.Pos[1];
	const FLT x43 = x40 * x37;
	const FLT x44 = (*_x0).Lighthouse.Pos[0] + (*error_model).Lighthouse.Pos[0];
	const FLT x45 = x40 * x34;
	const FLT x46 = (x44 * x45) + (x0 * x41) + (-1 * x42 * x43);
	const FLT x47 = x40 * x32;
	const FLT x48 = (x42 * x47) + (-1 * x0 * x45) + (x41 * x44);
	const FLT x49 = (*_x0).Object.Velocity.AxisAngleRot[2] + (*error_model).Object.Velocity.AxisAngleRot[2];
	const FLT x50 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[0];
	const FLT x51 = sin(x50);
	const FLT x52 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[2];
	const FLT x53 = sin(x52);
	const FLT x54 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[1];
	const FLT x55 = sin(x54);
	const FLT x56 = x53 * x55;
	const FLT x57 = x51 * x56;
	const FLT x58 = cos(x50);
	const FLT x59 = cos(x54);
	const FLT x60 = cos(x52);
	const FLT x61 = x60 * x59;
	const FLT x62 = x61 * x58;
	const FLT x63 = x62 + x57;
	const FLT x64 = x53 * x59;
	const FLT x65 = x64 * x51;
	const FLT x66 = x60 * x55;
	const FLT x67 = x66 * x58;
	const FLT x68 = x67 + x65;
	const FLT x69 = x66 * x51;
	const FLT x70 = x64 * x58;
	const FLT x71 = x70 + (-1 * x69);
	const FLT x72 = x58 * x56;
	const FLT x73 = x61 * x51;
	const FLT x74 = x73 + (-1 * x72);
	const FLT x75 = (x74 * x74) + (x71 * x71) + (x68 * x68) + (x63 * x63);
	const FLT x76 = 1. / sqrt(x75);
	const FLT x77 = x76 * (*_x0).Object.Pose.Rot[3];
	const FLT x78 = x76 * (*_x0).Object.Pose.Rot[2];
	const FLT x79 = x76 * (*_x0).Object.Pose.Rot[0];
	const FLT x80 = x76 * (*_x0).Object.Pose.Rot[1];
	const FLT x81 = (x71 * x79) + (x80 * x68) + (x77 * x63) + (-1 * x78 * x74);
	const FLT x82 = dt * dt;
	const FLT x83 = x49 * x49;
	const FLT x84 = x82 * x83;
	const FLT x85 = (*_x0).Object.Velocity.AxisAngleRot[0] + (*error_model).Object.Velocity.AxisAngleRot[0];
	const FLT x86 = x85 * x85;
	const FLT x87 = x82 * x86;
	const FLT x88 = (*_x0).Object.Velocity.AxisAngleRot[1] + (*error_model).Object.Velocity.AxisAngleRot[1];
	const FLT x89 = x88 * x88;
	const FLT x90 = x82 * x89;
	const FLT x91 = 1e-10 + x84 + x90 + x87;
	const FLT x92 = sqrt(x91);
	const FLT x93 = 0.5 * x92;
	const FLT x94 = sin(x93);
	const FLT x95 = x94 * x94;
	const FLT x96 = 1. / x91;
	const FLT x97 = x96 * x95;
	const FLT x98 = cos(x93);
	const FLT x99 = (x98 * x98) + (x87 * x97) + (x84 * x97) + (x90 * x97);
	const FLT x100 = 1. / sqrt(x99);
	const FLT x101 = x94 * x100;
	const FLT x102 = 1. / x92;
	const FLT x103 = dt * x102;
	const FLT x104 = x101 * x103;
	const FLT x105 = x81 * x104;
	const FLT x106 = (x78 * x63) + (-1 * x80 * x71) + (x79 * x68) + (x74 * x77);
	const FLT x107 = x104 * x106;
	const FLT x108 = (-1 * x80 * x74) + (x79 * x63) + (-1 * x71 * x77) + (-1 * x78 * x68);
	const FLT x109 = x98 * x100;
	const FLT x110 = x109 * x108;
	const FLT x111 = (x79 * x74) + (-1 * x77 * x68) + (x80 * x63) + (x71 * x78);
	const FLT x112 = x101 * x111;
	const FLT x113 = x103 * x112;
	const FLT x114 = (-1 * x85 * x113) + x110 + (-1 * x49 * x105) + (-1 * x88 * x107);
	const FLT x115 = x109 * x111;
	const FLT x116 = x108 * x104;
	const FLT x117 = (x85 * x116) + (-1 * x49 * x107) + x115 + (x88 * x105);
	const FLT x118 = x109 * x106;
	const FLT x119 = (-1 * x85 * x105) + (x88 * x116) + (x49 * x113) + x118;
	const FLT x120 = (-1 * x119 * sensor_pt[0]) + (x114 * sensor_pt[2]) + (x117 * sensor_pt[1]);
	const FLT x121 = x81 * x109;
	const FLT x122 = (x85 * x107) + (-1 * x88 * x113) + (x49 * x116) + x121;
	const FLT x123 = (x114 * sensor_pt[0]) + (-1 * x122 * sensor_pt[1]) + (x119 * sensor_pt[2]);
	const FLT x124 = dt * fabs(dt);
	const FLT x125 = 1.0 / 2.0 * x124;
	const FLT x126 = (*_x0).Object.Pose.Pos[1] + (2 * ((x123 * x122) + (-1 * x117 * x120))) + sensor_pt[1] +
					 (dt * ((*_x0).Object.Velocity.Pos[1] + (*error_model).Object.Velocity.Pos[1])) +
					 (*error_model).Object.Pose.Pos[1] + (x125 * ((*_x0).Object.Acc[1] + (*error_model).Object.Acc[1]));
	const FLT x127 = x40 * x126;
	const FLT x128 = (-1 * x117 * sensor_pt[2]) + (x114 * sensor_pt[1]) + (x122 * sensor_pt[0]);
	const FLT x129 = (dt * ((*_x0).Object.Velocity.Pos[2] + (*error_model).Object.Velocity.Pos[2])) +
					 (*error_model).Object.Pose.Pos[2] + (2 * ((x117 * x128) + (-1 * x119 * x123))) + sensor_pt[2] +
					 (x125 * ((*_x0).Object.Acc[2] + (*error_model).Object.Acc[2])) + (*_x0).Object.Pose.Pos[2];
	const FLT x130 = x40 * x129;
	const FLT x131 = (*_x0).Object.Pose.Pos[0] + (2 * ((x119 * x120) + (-1 * x122 * x128))) +
					 (*error_model).Object.Pose.Pos[0] +
					 (x125 * ((*_x0).Object.Acc[0] + (*error_model).Object.Acc[0])) + sensor_pt[0] +
					 (dt * ((*_x0).Object.Velocity.Pos[0] + (*error_model).Object.Velocity.Pos[0]));
	const FLT x132 = x40 * x131;
	const FLT x133 = (x34 * x132) + (-1 * x37 * x127) + (x36 * x130);
	const FLT x134 = x40 * x133;
	const FLT x135 = (x36 * x132) + (x32 * x127) + (-1 * x34 * x130);
	const FLT x136 =
		(-1 * (x42 + (2 * ((-1 * x47 * x48) + (x43 * x46))))) + x126 + (2 * ((-1 * x47 * x135) + (x37 * x134)));
	const FLT x137 = x136 * x136;
	const FLT x138 = (x0 * x43) + (x41 * x42) + (-1 * x44 * x47);
	const FLT x139 = (x37 * x130) + (x36 * x127) + (-1 * x32 * x132);
	const FLT x140 =
		x129 + (-1 * (x0 + (2 * ((-1 * x43 * x138) + (x45 * x48))))) + (2 * ((-1 * x43 * x139) + (x45 * x135)));
	const FLT x141 =
		(2 * ((-1 * x34 * x134) + (x47 * x139))) + x131 + (-1 * (x44 + (2 * ((-1 * x45 * x46) + (x47 * x138)))));
	const FLT x142 = x141 * x141;
	const FLT x143 = x142 + (x140 * x140);
	const FLT x144 = 1. / x143;
	const FLT x145 = 0.523598775598299 + (-1 * (*error_model).BSD1.tilt) + (-1 * (*_x0).BSD1.tilt);
	const FLT x146 = tan(x145);
	const FLT x147 = x146 * x146;
	const FLT x148 = 1. / sqrt(1 + (-1 * x137 * x144 * x147));
	const FLT x149 = 0.5 * x15;
	const FLT x150 = -1 * x149;
	const FLT x151 = 0.5 * x17;
	const FLT x152 = x151 + x150;
	const FLT x153 = 1.0 / 2.0 * (1. / (x26 * sqrt(x26)));
	const FLT x154 = x153 * (*_x0).Lighthouse.Rot[0];
	const FLT x155 = 0.5 * x13;
	const FLT x156 = 0.5 * x8;
	const FLT x157 = x156 + x155;
	const FLT x158 = 2 * x25;
	const FLT x159 = 0.5 * x24;
	const FLT x160 = -1 * x159;
	const FLT x161 = 0.5 * x23;
	const FLT x162 = x161 + x160;
	const FLT x163 = 2 * x14;
	const FLT x164 = 0.5 * x19;
	const FLT x165 = -0.5 * x21;
	const FLT x166 = x165 + (-1 * x164);
	const FLT x167 = 2 * x166;
	const FLT x168 = 2 * x22;
	const FLT x169 = (x18 * x167) + (x157 * x158) + (x168 * x152) + (x163 * x162);
	const FLT x170 = x18 * x169;
	const FLT x171 = x153 * (*_x0).Lighthouse.Rot[3];
	const FLT x172 = x14 * x169;
	const FLT x173 = x29 * x157;
	const FLT x174 = x25 * x169;
	const FLT x175 = x153 * (*_x0).Lighthouse.Rot[2];
	const FLT x176 = x153 * (*_x0).Lighthouse.Rot[1];
	const FLT x177 = x22 * x169;
	const FLT x178 = x30 * x166;
	const FLT x179 = x178 + (-1 * x176 * x177) + (x174 * x175) + (-1 * x170 * x154) + (x31 * x152) + (x28 * x162) +
					 (-1 * x171 * x172) + (-1 * x173);
	const FLT x180 = x40 * x179;
	const FLT x181 = 2 * x139;
	const FLT x182 = 1. / (x39 * sqrt(x39));
	const FLT x183 = -1 * x31 * x157;
	const FLT x184 = x28 * x166;
	const FLT x185 = x177 * x153;
	const FLT x186 = (x185 * (*_x0).Lighthouse.Rot[2]) + (-1 * x29 * x152) + (x30 * x162) + x183 + (x170 * x171) +
					 (-1 * x172 * x154) + (x176 * x174) + (-1 * x184);
	const FLT x187 = 2 * x36;
	const FLT x188 = x29 * x166;
	const FLT x189 = x30 * x157;
	const FLT x190 = (-1 * x28 * x152) + x189 + (-1 * x174 * x154) + (-1 * x176 * x172) + (x171 * x177) + (x31 * x162) +
					 (-1 * x170 * x175) + x188;
	const FLT x191 = 2 * x37;
	const FLT x192 = 2 * x32;
	const FLT x193 = -1 * x31 * x166;
	const FLT x194 = x28 * x157;
	const FLT x195 = (x29 * x162) + x193 + (x30 * x152) + (x170 * x176) + (-1 * x171 * x174) +
					 (-1 * x185 * (*_x0).Lighthouse.Rot[0]) + (-1 * x175 * x172) + x194;
	const FLT x196 = 2 * x34;
	const FLT x197 = (x196 * x195) + (x179 * x192) + (x187 * x186) + (x190 * x191);
	const FLT x198 = x182 * x197;
	const FLT x199 = x34 * x198;
	const FLT x200 = 1.0 / 2.0 * x131;
	const FLT x201 = x37 * x198;
	const FLT x202 = 1.0 / 2.0 * x126;
	const FLT x203 = 1.0 / 2.0 * x36;
	const FLT x204 = x203 * x198;
	const FLT x205 =
		(-1 * x204 * x129) + (x201 * x202) + (-1 * x127 * x190) + (x195 * x132) + (-1 * x200 * x199) + (x186 * x130);
	const FLT x206 = x40 * x196;
	const FLT x207 = 1.0 / 2.0 * x199;
	const FLT x208 = x40 * x42;
	const FLT x209 = x40 * x186;
	const FLT x210 = x40 * x195;
	const FLT x211 = 1.0 / 2.0 * x201;
	const FLT x212 =
		(x42 * x211) + (-1 * x0 * x204) + (x44 * x210) + (-1 * x208 * x190) + (-1 * x44 * x207) + (x0 * x209);
	const FLT x213 = x32 * x197;
	const FLT x214 = x213 * x182;
	const FLT x215 = 1.0 / 2.0 * x44;
	const FLT x216 = x40 * x190;
	const FLT x217 =
		(x0 * x216) + (-1 * x42 * x204) + (-1 * x0 * x211) + (x208 * x186) + (x215 * x214) + (-1 * x44 * x180);
	const FLT x218 = x40 * x192;
	const FLT x219 = 2 * x134;
	const FLT x220 =
		(x214 * x200) + (x127 * x186) + (-1 * x179 * x132) + (-1 * x211 * x129) + (x190 * x130) + (-1 * x204 * x126);
	const FLT x221 = x182 * x139;
	const FLT x222 = 2 * x46;
	const FLT x223 = 2 * x138;
	const FLT x224 = (x212 * x206) + (-1 * x218 * x217) + (-1 * x205 * x206) + (x181 * x180) + (-1 * x46 * x199) +
					 (x210 * x222) + (x214 * x138) + (-1 * x213 * x221) + (-1 * x219 * x195) + (x199 * x133) +
					 (-1 * x223 * x180) + (x218 * x220);
	const FLT x225 = 2 * x141;
	const FLT x226 = 2 * x135;
	const FLT x227 = x36 * x200;
	const FLT x228 =
		(x207 * x129) + (-1 * x214 * x202) + (-1 * x195 * x130) + (-1 * x227 * x198) + (x126 * x180) + (x186 * x132);
	const FLT x229 = x40 * x191;
	const FLT x230 = x34 * x135;
	const FLT x231 = 1.0 / 2.0 * x42;
	const FLT x232 =
		(x0 * x207) + (-1 * x214 * x231) + (-1 * x0 * x210) + (x42 * x180) + (x44 * x209) + (-1 * x44 * x204);
	const FLT x233 = 2 * x48;
	const FLT x234 = (x37 * x221 * x197) + (-1 * x201 * x138) + (x210 * x226) + (-1 * x216 * x181) + (x48 * x199) +
					 (x217 * x229) + (x206 * x228) + (-1 * x210 * x233) + (x216 * x223) + (-1 * x220 * x229) +
					 (-1 * x230 * x198) + (-1 * x232 * x206);
	const FLT x235 = 2 * x140;
	const FLT x236 = (x234 * x235) + (x224 * x225);
	const FLT x237 = 1.0 / 2.0 * x136;
	const FLT x238 = x237 * (1. / (x143 * sqrt(x143))) * x146;
	const FLT x239 = (-1 * x218 * x228) + (-1 * x226 * x180) + (x219 * x190) + (-1 * x216 * x222) + (-1 * x201 * x133) +
					 (x214 * x135) + (x46 * x201) + (x233 * x180) + (x218 * x232) + (-1 * x48 * x214) +
					 (-1 * x212 * x229) + (x205 * x229);
	const FLT x240 = 1. / sqrt(x143);
	const FLT x241 = x240 * x146;
	const FLT x242 = (-1 * x239 * x241) + (x238 * x236);
	const FLT x243 = 1. / x141;
	const FLT x244 = x140 * (1. / x142);
	const FLT x245 = x142 * x144;
	const FLT x246 = ((x224 * x244) + (-1 * x234 * x243)) * x245;
	const FLT x247 = (-1 * x246) + (x242 * x148);
	const FLT x248 = sin(x145);
	const FLT x249 = x143 + x137;
	const FLT x250 = 1. / sqrt(x249);
	const FLT x251 = cos(x145);
	const FLT x252 = 1. / x251;
	const FLT x253 = x250 * x252;
	const FLT x254 = asin(x253 * x136);
	const FLT x255 = 8.0108022e-06 * x254;
	const FLT x256 = -8.0108022e-06 + (-1 * x255);
	const FLT x257 = 0.0028679863 + (x254 * x256);
	const FLT x258 = 5.3685255e-06 + (x254 * x257);
	const FLT x259 = 0.0076069798 + (x254 * x258);
	const FLT x260 = x254 * x259;
	const FLT x261 = -8.0108022e-06 + (-1.60216044e-05 * x254);
	const FLT x262 = x257 + (x261 * x254);
	const FLT x263 = x258 + (x262 * x254);
	const FLT x264 = x259 + (x263 * x254);
	const FLT x265 = (x264 * x254) + x260;
	const FLT x266 = (*error_model).BSD1.ogeemag + (*_x0).BSD1.ogeemag;
	const FLT x267 = atan2(-1 * x140, x141);
	const FLT x268 = -1 * x241 * x136;
	const FLT x269 = asin(x268) + (-1 * x267) + (-1 * (*error_model).BSD1.ogeephase) + (-1 * (*_x0).BSD1.ogeephase);
	const FLT x270 = x266 * cos(x269);
	const FLT x271 = x270 * x265 * x248;
	const FLT x272 = 2 * x136;
	const FLT x273 = x237 * x252 * (1. / (x249 * sqrt(x249)));
	const FLT x274 = (-1 * x273 * (x236 + (x239 * x272))) + (x239 * x253);
	const FLT x275 = 1. / (x251 * x251);
	const FLT x276 = 1. / sqrt(1 + (-1 * x275 * (1. / x249) * x137));
	const FLT x277 = x276 * x261;
	const FLT x278 = x276 * x256;
	const FLT x279 = x278 * x274;
	const FLT x280 = 2.40324066e-05 * x254;
	const FLT x281 = x276 * x280;
	const FLT x282 = x276 * x262;
	const FLT x283 = x276 * x257;
	const FLT x284 = x276 * x255;
	const FLT x285 = (x254 * (x279 + (-1 * x274 * x284))) + (x274 * x283);
	const FLT x286 = x276 * x263;
	const FLT x287 = x276 * x258;
	const FLT x288 = (x274 * x287) + (x285 * x254);
	const FLT x289 = x276 * x264;
	const FLT x290 = x276 * x259;
	const FLT x291 = sin(x269);
	const FLT x292 = (*_x0).BSD1.curve + (-1 * x291 * x266) + (*error_model).BSD1.curve;
	const FLT x293 = x292 * x248;
	const FLT x294 = x254 * x254;
	const FLT x295 = x293 * x265;
	const FLT x296 = x251 + x295;
	const FLT x297 = (1. / (x296 * x296)) * x294 * x259;
	const FLT x298 = x297 * x292;
	const FLT x299 = 1. / x296;
	const FLT x300 = x299 * x294;
	const FLT x301 = x259 * x300;
	const FLT x302 = x270 * x301;
	const FLT x303 = x276 * x299 * x260;
	const FLT x304 = 2 * x292;
	const FLT x305 = x303 * x304;
	const FLT x306 = x292 * x300;
	const FLT x307 = x268 + (x292 * x301);
	const FLT x308 = 1. / sqrt(1 + (-1 * (x307 * x307)));
	const FLT x309 =
		x246 +
		(-1 * x308 *
		 ((x288 * x306) + x242 + (x274 * x305) +
		  (-1 * x298 *
		   ((x293 *
			 ((x288 * x254) + (x274 * x290) +
			  (x254 * (x288 + (x254 * ((x254 * ((-1 * x274 * x281) + (x277 * x274) + x279)) + x285 + (x274 * x282))) +
					   (x274 * x286))) +
			  (x274 * x289))) +
			(-1 * x271 * x247))) +
		  (-1 * x247 * x302)));
	const FLT x310 = (-1 * asin(x307)) + x267 + (*error_model).BSD1.gibpha + (*_x0).BSD1.gibpha;
	const FLT x311 = cos(x310) * ((*error_model).BSD1.gibmag + (*_x0).BSD1.gibmag);
	const FLT x312 = (-1 * x156) + x155;
	const FLT x313 = -1 * x151;
	const FLT x314 = x313 + x150;
	const FLT x315 = x164 + x165;
	const FLT x316 = -1 * x161;
	const FLT x317 = x160 + x316;
	const FLT x318 = 2 * x18;
	const FLT x319 = ((x312 * x168) + (x317 * x318) + (x314 * x158) + (x315 * x163)) * x153;
	const FLT x320 = x14 * x319;
	const FLT x321 = x319 * (*_x0).Lighthouse.Rot[1];
	const FLT x322 = x18 * x319;
	const FLT x323 = x25 * x319;
	const FLT x324 = (x323 * (*_x0).Lighthouse.Rot[2]) + (-1 * x320 * (*_x0).Lighthouse.Rot[3]) + (x30 * x317) +
					 (x31 * x312) + (-1 * x322 * (*_x0).Lighthouse.Rot[0]) + (-1 * x22 * x321) + (x28 * x315) +
					 (-1 * x29 * x314);
	const FLT x325 = x40 * x324;
	const FLT x326 = x22 * x319;
	const FLT x327 = (-1 * x28 * x312) + (-1 * x14 * x321) + (-1 * x323 * (*_x0).Lighthouse.Rot[0]) +
					 (x326 * (*_x0).Lighthouse.Rot[3]) + (x30 * x314) + (-1 * x322 * (*_x0).Lighthouse.Rot[2]) +
					 (x31 * x315) + (x29 * x317);
	const FLT x328 = (x30 * x315) + (x322 * (*_x0).Lighthouse.Rot[3]) + (-1 * x320 * (*_x0).Lighthouse.Rot[0]) +
					 (-1 * x29 * x312) + (-1 * x31 * x314) + (x323 * (*_x0).Lighthouse.Rot[1]) + (-1 * x28 * x317) +
					 (x326 * (*_x0).Lighthouse.Rot[2]);
	const FLT x329 = (x29 * x315) + (-1 * x320 * (*_x0).Lighthouse.Rot[2]) + (x28 * x314) +
					 (-1 * x326 * (*_x0).Lighthouse.Rot[0]) + (-1 * x31 * x317) +
					 (-1 * x323 * (*_x0).Lighthouse.Rot[3]) + (x30 * x312) + (x18 * x321);
	const FLT x330 = (x329 * x196) + (x324 * x192) + (x327 * x191) + (x328 * x187);
	const FLT x331 = x330 * x182;
	const FLT x332 = x34 * x331;
	const FLT x333 = x40 * x329;
	const FLT x334 = x37 * x330;
	const FLT x335 = x334 * x182;
	const FLT x336 = 1.0 / 2.0 * x129;
	const FLT x337 = x203 * x331;
	const FLT x338 = x32 * x331;
	const FLT x339 = x40 * x327;
	const FLT x340 =
		(x339 * x129) + (-1 * x337 * x126) + (-1 * x336 * x335) + (-1 * x325 * x131) + (x328 * x127) + (x200 * x338);
	const FLT x341 = 2 * x133;
	const FLT x342 =
		(x202 * x335) + (-1 * x200 * x332) + (x328 * x130) + (x333 * x131) + (-1 * x337 * x129) + (-1 * x339 * x126);
	const FLT x343 = 1.0 / 2.0 * x0;
	const FLT x344 = x40 * x328;
	const FLT x345 =
		(x42 * x344) + (x215 * x338) + (x0 * x339) + (-1 * x335 * x343) + (-1 * x42 * x337) + (-1 * x44 * x325);
	const FLT x346 = 1.0 / 2.0 * x332;
	const FLT x347 =
		(-1 * x44 * x346) + (-1 * x42 * x339) + (x0 * x344) + (x231 * x335) + (x44 * x333) + (-1 * x0 * x337);
	const FLT x348 = (x206 * x347) + (x325 * x181) + (x222 * x333) + (-1 * x46 * x332) + (x218 * x340) + (x332 * x133) +
					 (-1 * x32 * x221 * x330) + (-1 * x206 * x342) + (-1 * x223 * x325) + (-1 * x333 * x341) +
					 (x338 * x138) + (-1 * x218 * x345);
	const FLT x349 =
		(-1 * x44 * x337) + (x0 * x346) + (-1 * x0 * x333) + (-1 * x231 * x338) + (x44 * x344) + (x42 * x325);
	const FLT x350 =
		(x346 * x129) + (x325 * x126) + (-1 * x202 * x338) + (x328 * x132) + (-1 * x227 * x331) + (-1 * x333 * x129);
	const FLT x351 = x48 * x34;
	const FLT x352 = (x226 * x333) + (-1 * x206 * x349) + (-1 * x233 * x333) + (-1 * x335 * x138) + (x223 * x339) +
					 (-1 * x229 * x340) + (x206 * x350) + (x351 * x331) + (-1 * x339 * x181) + (x229 * x345) +
					 (x221 * x334) + (-1 * x230 * x331);
	const FLT x353 = (x235 * x352) + (x225 * x348);
	const FLT x354 = x48 * x32;
	const FLT x355 = (x46 * x335) + (x338 * x135) + (-1 * x218 * x350) + (-1 * x354 * x331) + (-1 * x222 * x339) +
					 (-1 * x229 * x347) + (x218 * x349) + (-1 * x335 * x133) + (x339 * x341) + (x229 * x342) +
					 (x233 * x325) + (-1 * x226 * x325);
	const FLT x356 = (-1 * x241 * x355) + (x238 * x353);
	const FLT x357 = ((x244 * x348) + (-1 * x243 * x352)) * x245;
	const FLT x358 = (-1 * x357) + (x356 * x148);
	const FLT x359 = (-1 * x273 * (x353 + (x272 * x355))) + (x253 * x355);
	const FLT x360 = x278 * x359;
	const FLT x361 = (x254 * (x360 + (-1 * x284 * x359))) + (x283 * x359);
	const FLT x362 = (x287 * x359) + (x254 * x361);
	const FLT x363 =
		x357 +
		(-1 * x308 *
		 (x356 + (-1 * x358 * x302) + (x359 * x305) +
		  (-1 * x298 *
		   ((x293 * ((x254 * x362) +
					 (x254 * ((x254 * (x361 + (x254 * ((-1 * x281 * x359) + x360 + (x277 * x359))) + (x282 * x359))) +
							  x362 + (x286 * x359))) +
					 (x289 * x359) + (x290 * x359))) +
			(-1 * x271 * x358))) +
		  (x362 * x306)));
	const FLT x364 = x149 + x313;
	const FLT x365 = x159 + x316;
	const FLT x366 = (x365 * x168) + (x25 * x167) + (x318 * x157) + (x364 * x163);
	const FLT x367 = x14 * x366;
	const FLT x368 = x22 * x366;
	const FLT x369 = x18 * x366;
	const FLT x370 = x369 * x153;
	const FLT x371 = x25 * x366;
	const FLT x372 = x178 + (x31 * x364) + (-1 * x367 * x176) + (-1 * x371 * x154) + (x368 * x171) + (-1 * x28 * x365) +
					 (-1 * x370 * (*_x0).Lighthouse.Rot[2]) + x173;
	const FLT x373 = x40 * x372;
	const FLT x374 = (-1 * x29 * x365) + x193 + (x371 * x176) + (-1 * x367 * x154) + (x30 * x364) +
					 (x370 * (*_x0).Lighthouse.Rot[3]) + (x368 * x175) + (-1 * x194);
	const FLT x375 = x189 + (x371 * x175) + (-1 * x370 * (*_x0).Lighthouse.Rot[0]) + (x28 * x364) + (x31 * x365) +
					 (-1 * x188) + (-1 * x368 * x176) + (-1 * x367 * x171);
	const FLT x376 = (-1 * x367 * x175) + (x30 * x365) + (x29 * x364) + (-1 * x368 * x154) + x183 + (x369 * x176) +
					 (-1 * x371 * x171) + x184;
	const FLT x377 = ((x376 * x196) + (x375 * x192) + (x374 * x187) + (x372 * x191)) * x182;
	const FLT x378 = x40 * x375;
	const FLT x379 = x203 * x377;
	const FLT x380 = x37 * x377;
	const FLT x381 = x34 * x377;
	const FLT x382 = x40 * x376;
	const FLT x383 = x40 * x374;
	const FLT x384 =
		(-1 * x42 * x373) + (x231 * x380) + (x0 * x383) + (-1 * x0 * x379) + (-1 * x215 * x381) + (x44 * x382);
	const FLT x385 =
		(-1 * x379 * x129) + (-1 * x200 * x381) + (x374 * x130) + (-1 * x372 * x127) + (x202 * x380) + (x376 * x132);
	const FLT x386 = x32 * x377;
	const FLT x387 =
		(x375 * x127) + (-1 * x227 * x377) + (x381 * x336) + (-1 * x202 * x386) + (-1 * x376 * x130) + (x374 * x132);
	const FLT x388 = x46 * x377;
	const FLT x389 = 1.0 / 2.0 * x386;
	const FLT x390 =
		(x44 * x383) + (x381 * x343) + (-1 * x42 * x389) + (-1 * x0 * x382) + (x42 * x378) + (-1 * x44 * x379);
	const FLT x391 = x377 * x133;
	const FLT x392 = (-1 * x229 * x384) + (-1 * x218 * x387) + (-1 * x226 * x378) + (x373 * x341) + (x233 * x378) +
					 (-1 * x222 * x373) + (-1 * x37 * x391) + (-1 * x377 * x354) + (x229 * x385) + (x386 * x135) +
					 (x218 * x390) + (x37 * x388);
	const FLT x393 = x377 * x139;
	const FLT x394 =
		(x200 * x386) + (-1 * x379 * x126) + (-1 * x380 * x336) + (x374 * x127) + (x372 * x130) + (-1 * x375 * x132);
	const FLT x395 = x377 * x138;
	const FLT x396 =
		(x0 * x373) + (-1 * x44 * x378) + (-1 * x380 * x343) + (x42 * x383) + (x44 * x389) + (-1 * x42 * x379);
	const FLT x397 = (-1 * x218 * x396) + (x34 * x391) + (-1 * x32 * x393) + (-1 * x34 * x388) + (x32 * x395) +
					 (x378 * x181) + (x222 * x382) + (x206 * x384) + (x218 * x394) + (-1 * x382 * x341) +
					 (-1 * x206 * x385) + (-1 * x223 * x378);
	const FLT x398 = (x206 * x387) + (-1 * x229 * x394) + (x377 * x351) + (x229 * x396) + (-1 * x206 * x390) +
					 (-1 * x37 * x395) + (x226 * x382) + (x37 * x393) + (x223 * x373) + (-1 * x233 * x382) +
					 (-1 * x373 * x181) + (-1 * x230 * x377);
	const FLT x399 = (x235 * x398) + (x225 * x397);
	const FLT x400 = (-1 * x273 * (x399 + (x272 * x392))) + (x253 * x392);
	const FLT x401 = (-1 * x241 * x392) + (x238 * x399);
	const FLT x402 = ((x244 * x397) + (-1 * x243 * x398)) * x245;
	const FLT x403 = (-1 * x402) + (x401 * x148);
	const FLT x404 = x400 * x278;
	const FLT x405 = (x254 * (x404 + (-1 * x400 * x284))) + (x400 * x283);
	const FLT x406 = (x400 * x287) + (x405 * x254);
	const FLT x407 =
		x402 +
		(-1 * x308 *
		 ((x406 * x306) + (x400 * x305) + x401 + (-1 * x403 * x302) +
		  (-1 * x298 *
		   ((x293 *
			 ((x406 * x254) +
			  (x254 * (x406 + (x254 * (x405 + (x254 * ((-1 * x400 * x281) + (x400 * x277) + x404)) + (x400 * x282))) +
					   (x400 * x286))) +
			  (x400 * x290) + (x400 * x289))) +
			(-1 * x403 * x271)))));
	const FLT x408 = 1. / x39;
	const FLT x409 = 2 * x408;
	const FLT x410 = x33 * x409;
	const FLT x411 = x35 * x409;
	const FLT x412 = -1 + x411;
	const FLT x413 = x412 + x410;
	const FLT x414 = x32 * x408;
	const FLT x415 = x414 * x191;
	const FLT x416 = -1 * x415;
	const FLT x417 = x34 * x408;
	const FLT x418 = x417 * x187;
	const FLT x419 = -1 * x418;
	const FLT x420 = x419 + x416;
	const FLT x421 = (x420 * x235) + (x413 * x225);
	const FLT x422 = x408 * x187;
	const FLT x423 = x32 * x422;
	const FLT x424 = x417 * x191;
	const FLT x425 = -1 * x424;
	const FLT x426 = x425 + x423;
	const FLT x427 = (-1 * x426 * x241) + (x421 * x238);
	const FLT x428 = ((x413 * x244) + (-1 * x420 * x243)) * x245;
	const FLT x429 = (-1 * x428) + (x427 * x148);
	const FLT x430 = (-1 * x273 * (x421 + (x426 * x272))) + (x426 * x253);
	const FLT x431 = x430 * x278;
	const FLT x432 = (x254 * (x431 + (-1 * x430 * x284))) + (x430 * x283);
	const FLT x433 = (x430 * x287) + (x432 * x254);
	const FLT x434 =
		x428 +
		(-1 * x308 *
		 (x427 + (x433 * x306) + (x430 * x305) + (-1 * x429 * x302) +
		  (-1 * x298 *
		   ((x293 *
			 ((x254 * (x433 + (x254 * (x432 + (x254 * (x431 + (-1 * x430 * x281) + (x430 * x277))) + (x430 * x282))) +
					   (x430 * x286))) +
			  (x430 * x289) + (x433 * x254) + (x430 * x290))) +
			(-1 * x429 * x271)))));
	const FLT x435 = x38 * x409;
	const FLT x436 = -1 + x435 + x410;
	const FLT x437 = -1 * x423;
	const FLT x438 = x437 + x425;
	const FLT x439 = x37 * x422;
	const FLT x440 = x417 * x192;
	const FLT x441 = -1 * x440;
	const FLT x442 = x441 + x439;
	const FLT x443 = (x442 * x235) + (x438 * x225);
	const FLT x444 = (-1 * x273 * (x443 + (x436 * x272))) + (x436 * x253);
	const FLT x445 = x444 * x278;
	const FLT x446 = (x254 * (x445 + (-1 * x444 * x284))) + (x444 * x283);
	const FLT x447 = (x444 * x287) + (x446 * x254);
	const FLT x448 = (-1 * x436 * x241) + (x443 * x238);
	const FLT x449 = ((x438 * x244) + (-1 * x442 * x243)) * x245;
	const FLT x450 = (-1 * x449) + (x448 * x148);
	const FLT x451 =
		x449 +
		(-1 * x308 *
		 (x448 +
		  (-1 * x298 *
		   ((x293 *
			 ((x444 * x290) + (x447 * x254) +
			  (x254 * (x447 + (x254 * ((x254 * ((-1 * x444 * x281) + (x444 * x277) + x445)) + x446 + (x444 * x282))) +
					   (x444 * x286))) +
			  (x444 * x289))) +
			(-1 * x450 * x271))) +
		  (-1 * x450 * x302) + (x444 * x305) + (x447 * x306)));
	const FLT x452 = -1 * x439;
	const FLT x453 = x452 + x441;
	const FLT x454 = x416 + x418;
	const FLT x455 = x412 + x435;
	const FLT x456 = (x455 * x235) + (x454 * x225);
	const FLT x457 = (-1 * x273 * (x456 + (x453 * x272))) + (x453 * x253);
	const FLT x458 = (-1 * x453 * x241) + (x456 * x238);
	const FLT x459 = ((x454 * x244) + (-1 * x455 * x243)) * x245;
	const FLT x460 = (-1 * x459) + (x458 * x148);
	const FLT x461 = x457 * x278;
	const FLT x462 = (x254 * (x461 + (-1 * x457 * x284))) + (x457 * x283);
	const FLT x463 = (x457 * x287) + (x462 * x254);
	const FLT x464 =
		x459 +
		(-1 * x308 *
		 ((-1 * x460 * x302) + x458 + (x463 * x306) + (x457 * x305) +
		  (-1 * x298 *
		   ((x293 *
			 ((x463 * x254) + (x457 * x290) +
			  (x254 * (x463 + (x254 * ((x254 * ((-1 * x457 * x281) + x461 + (x457 * x277))) + x462 + (x457 * x282))) +
					   (x457 * x286))) +
			  (x457 * x289))) +
			(-1 * x460 * x271)))));
	const FLT x465 = x37 * x124;
	const FLT x466 = x465 * x417;
	const FLT x467 = x36 * x124;
	const FLT x468 = x467 * x414;
	const FLT x469 = (-1 * x468) + x466;
	const FLT x470 = x408 * x124;
	const FLT x471 = -1 * x35 * x470;
	const FLT x472 = -1 * x33 * x470;
	const FLT x473 = x472 + x125 + x471;
	const FLT x474 = x467 * x417;
	const FLT x475 = x465 * x414;
	const FLT x476 = x475 + x474;
	const FLT x477 = (x476 * x235) + (x473 * x225);
	const FLT x478 = (-1 * x273 * (x477 + (x469 * x272))) + (x469 * x253);
	const FLT x479 = (-1 * x469 * x241) + (x477 * x238);
	const FLT x480 = ((x473 * x244) + (-1 * x476 * x243)) * x245;
	const FLT x481 = (-1 * x480) + (x479 * x148);
	const FLT x482 = x478 * x278;
	const FLT x483 = (x254 * (x482 + (-1 * x478 * x284))) + (x478 * x283);
	const FLT x484 = (x478 * x287) + (x483 * x254);
	const FLT x485 =
		x480 +
		(-1 * x308 *
		 ((-1 * x298 *
		   ((x293 *
			 ((x484 * x254) +
			  (x254 * (x484 + (x254 * (x483 + (x254 * ((-1 * x478 * x281) + x482 + (x478 * x277))) + (x478 * x282))) +
					   (x478 * x286))) +
			  (x478 * x289) + (x478 * x290))) +
			(-1 * x481 * x271))) +
		  (x484 * x306) + x479 + (x478 * x305) + (-1 * x481 * x302)));
	const FLT x486 = x466 + x468;
	const FLT x487 = x32 * x417 * x124;
	const FLT x488 = x36 * x37 * x470;
	const FLT x489 = (-1 * x488) + x487;
	const FLT x490 = (x489 * x235) + (x486 * x225);
	const FLT x491 = (-1 * x38 * x470) + x125;
	const FLT x492 = x491 + x472;
	const FLT x493 = (-1 * x492 * x241) + (x490 * x238);
	const FLT x494 = ((x486 * x244) + (-1 * x489 * x243)) * x245;
	const FLT x495 = (-1 * x494) + (x493 * x148);
	const FLT x496 = (-1 * x273 * (x490 + (x492 * x272))) + (x492 * x253);
	const FLT x497 = x496 * x278;
	const FLT x498 = (x254 * (x497 + (-1 * x496 * x284))) + (x496 * x283);
	const FLT x499 = (x496 * x287) + (x498 * x254);
	const FLT x500 =
		x494 +
		(-1 * x308 *
		 ((x499 * x306) + (x496 * x305) +
		  (-1 * x298 *
		   ((x293 *
			 ((x499 * x254) + (x496 * x289) + (x496 * x290) +
			  (x254 * (x499 + (x254 * (x498 + (x254 * ((-1 * x496 * x281) + x497 + (x496 * x277))) + (x496 * x282))) +
					   (x496 * x286))))) +
			(-1 * x495 * x271))) +
		  x493 + (-1 * x495 * x302)));
	const FLT x501 = x487 + x488;
	const FLT x502 = (-1 * x474) + x475;
	const FLT x503 = x491 + x471;
	const FLT x504 = (x503 * x235) + (x502 * x225);
	const FLT x505 = (-1 * x273 * (x504 + (x501 * x272))) + (x501 * x253);
	const FLT x506 = (-1 * x501 * x241) + (x504 * x238);
	const FLT x507 = ((x502 * x244) + (-1 * x503 * x243)) * x245;
	const FLT x508 = (-1 * x507) + (x506 * x148);
	const FLT x509 = x505 * x278;
	const FLT x510 = (x254 * (x509 + (-1 * x505 * x284))) + (x505 * x283);
	const FLT x511 = (x505 * x287) + (x510 * x254);
	const FLT x512 =
		x507 +
		(-1 * x308 *
		 (x506 + (-1 * x508 * x302) + (x505 * x305) + (x511 * x306) +
		  (-1 * x298 *
		   ((x293 *
			 ((x511 * x254) + (x505 * x290) +
			  (x254 * (x511 + (x254 * ((x254 * (x509 + (-1 * x505 * x281) + (x505 * x277))) + x510 + (x505 * x282))) +
					   (x505 * x286))) +
			  (x505 * x289))) +
			(-1 * x508 * x271)))));
	const FLT x513 = 0.5 * x65;
	const FLT x514 = -0.5 * x67;
	const FLT x515 = x514 + (-1 * x513);
	const FLT x516 = -1 * x80 * x515;
	const FLT x517 = 1.0 / 2.0 * (1. / (x75 * sqrt(x75)));
	const FLT x518 = x517 * (*_x0).Object.Pose.Rot[0];
	const FLT x519 = 0.5 * x62;
	const FLT x520 = 0.5 * x57;
	const FLT x521 = x520 + x519;
	const FLT x522 = 2 * x521;
	const FLT x523 = 0.5 * x69;
	const FLT x524 = -1 * x523;
	const FLT x525 = 0.5 * x70;
	const FLT x526 = x525 + x524;
	const FLT x527 = 2 * x68;
	const FLT x528 = 2 * x71;
	const FLT x529 = 0.5 * x73;
	const FLT x530 = -1 * x529;
	const FLT x531 = 0.5 * x72;
	const FLT x532 = x531 + x530;
	const FLT x533 = 2 * x63;
	const FLT x534 = (x533 * x532) + (x74 * x522) + (x515 * x528) + (x526 * x527);
	const FLT x535 = x68 * x534;
	const FLT x536 = x517 * (*_x0).Object.Pose.Rot[1];
	const FLT x537 = x71 * x534;
	const FLT x538 = x517 * (*_x0).Object.Pose.Rot[2];
	const FLT x539 = x63 * x534;
	const FLT x540 = x76 * x532;
	const FLT x541 = x77 * x521;
	const FLT x542 = x517 * (*_x0).Object.Pose.Rot[3];
	const FLT x543 = x74 * x534;
	const FLT x544 = (x79 * x526) + x541 + (-1 * x542 * x543) + (-1 * x518 * x535) + (-1 * x539 * x538) + x516 +
					 (x537 * x536) + (x540 * (*_x0).Object.Pose.Rot[2]);
	const FLT x545 = x85 * x104;
	const FLT x546 = -1 * x80 * x521;
	const FLT x547 = x77 * x515;
	const FLT x548 = x74 * x536;
	const FLT x549 = (x540 * (*_x0).Object.Pose.Rot[0]) + (-1 * x518 * x539) + (x537 * x542) + (x534 * x548) + x546 +
					 (-1 * x78 * x526) + (-1 * x547) + (x535 * x538);
	const FLT x550 = x49 * x104;
	const FLT x551 = x518 * x534;
	const FLT x552 = x79 * x521;
	const FLT x553 = x78 * x515;
	const FLT x554 = (-1 * x537 * x538) + x553 + x552 + (-1 * x74 * x551) + (x540 * (*_x0).Object.Pose.Rot[1]) +
					 (-1 * x77 * x526) + (-1 * x536 * x539) + (x535 * x542);
	const FLT x555 = x88 * x104;
	const FLT x556 = x68 * x536;
	const FLT x557 = x78 * x521;
	const FLT x558 = x79 * x515;
	const FLT x559 = x558 + (x540 * (*_x0).Object.Pose.Rot[3]) + (-1 * x534 * x556) + (-1 * x539 * x542) +
					 (x538 * x543) + (-1 * x557) + (x80 * x526) + (-1 * x71 * x551);
	const FLT x560 = (x559 * x109) + (-1 * x554 * x555) + (x544 * x545) + (x549 * x550);
	const FLT x561 = (-1 * x544 * x555) + (-1 * x545 * x554) + (x549 * x109) + (-1 * x550 * x559);
	const FLT x562 = (x544 * x109) + (-1 * x545 * x559) + (x550 * x554) + (x549 * x555);
	const FLT x563 = (-1 * x560 * sensor_pt[1]) + (x562 * sensor_pt[2]) + (x561 * sensor_pt[0]);
	const FLT x564 = 2 * x122;
	const FLT x565 = (-1 * x544 * x550) + (x545 * x549) + (x559 * x555) + (x554 * x109);
	const FLT x566 = (x565 * sensor_pt[1]) + (-1 * x562 * sensor_pt[0]) + (x561 * sensor_pt[2]);
	const FLT x567 = 2 * x117;
	const FLT x568 = 2 * x120;
	const FLT x569 = 2 * x123;
	const FLT x570 = (-1 * x568 * x565) + (x560 * x569) + (x563 * x564) + (-1 * x567 * x566);
	const FLT x571 = x40 * x570;
	const FLT x572 = 2 * x119;
	const FLT x573 = 2 * x128;
	const FLT x574 = (x560 * sensor_pt[0]) + (-1 * x565 * sensor_pt[2]) + (x561 * sensor_pt[1]);
	const FLT x575 = (x574 * x567) + (x573 * x565) + (-1 * x569 * x562) + (-1 * x572 * x563);
	const FLT x576 = x40 * x575;
	const FLT x577 = (x572 * x566) + (-1 * x574 * x564) + (x562 * x568) + (-1 * x573 * x560);
	const FLT x578 = x40 * x577;
	const FLT x579 = (-1 * x32 * x578) + (x36 * x571) + (x37 * x576);
	const FLT x580 = (x36 * x576) + (x34 * x578) + (-1 * x37 * x571);
	const FLT x581 = x577 + (x579 * x218) + (-1 * x580 * x206);
	const FLT x582 = (-1 * x34 * x576) + (x47 * x570) + (x36 * x578);
	const FLT x583 = x575 + (x582 * x206) + (-1 * x579 * x229);
	const FLT x584 = (x583 * x235) + (x581 * x225);
	const FLT x585 = (x580 * x229) + x570 + (-1 * x582 * x218);
	const FLT x586 = (-1 * x585 * x241) + (x584 * x238);
	const FLT x587 = ((x581 * x244) + (-1 * x583 * x243)) * x245;
	const FLT x588 = (-1 * x587) + (x586 * x148);
	const FLT x589 = (-1 * x273 * (x584 + (x585 * x272))) + (x585 * x253);
	const FLT x590 = x589 * x278;
	const FLT x591 = (x254 * (x590 + (-1 * x589 * x284))) + (x589 * x283);
	const FLT x592 = (x589 * x287) + (x591 * x254);
	const FLT x593 =
		x587 +
		(-1 * x308 *
		 (x586 + (x592 * x306) +
		  (-1 * x298 *
		   ((x293 *
			 ((x592 * x254) + (x589 * x290) +
			  (x254 * (x592 + (x254 * (x591 + (x254 * ((-1 * x589 * x281) + x590 + (x589 * x277))) + (x589 * x282))) +
					   (x589 * x286))) +
			  (x589 * x289))) +
			(-1 * x588 * x271))) +
		  (-1 * x588 * x302) + (x589 * x305)));
	const FLT x594 = -1 * x525;
	const FLT x595 = x594 + x524;
	const FLT x596 = 2 * x74;
	const FLT x597 = -1 * x531;
	const FLT x598 = x530 + x597;
	const FLT x599 = (-1 * x520) + x519;
	const FLT x600 = x513 + x514;
	const FLT x601 = (x600 * x533) + (x599 * x527) + (x596 * x595) + (x598 * x528);
	const FLT x602 = x71 * x601;
	const FLT x603 = x68 * x601;
	const FLT x604 = x63 * x518;
	const FLT x605 = x76 * x599;
	const FLT x606 = x76 * x600;
	const FLT x607 = (x606 * (*_x0).Object.Pose.Rot[0]) + (-1 * x601 * x604) + (-1 * x605 * (*_x0).Object.Pose.Rot[2]) +
					 (x601 * x548) + (-1 * x77 * x598) + (-1 * x80 * x595) + (x602 * x542) + (x603 * x538);
	const FLT x608 = x63 * x601;
	const FLT x609 = x74 * x601;
	const FLT x610 = (x79 * x595) + (-1 * x602 * x538) + (x78 * x598) + (x606 * (*_x0).Object.Pose.Rot[1]) +
					 (-1 * x605 * (*_x0).Object.Pose.Rot[3]) + (-1 * x608 * x536) + (x603 * x542) + (-1 * x609 * x518);
	const FLT x611 = (x605 * (*_x0).Object.Pose.Rot[0]) + (x77 * x595) + (-1 * x603 * x518) + (-1 * x80 * x598) +
					 (-1 * x608 * x538) + (-1 * x609 * x542) + (x602 * x536) + (x606 * (*_x0).Object.Pose.Rot[2]);
	const FLT x612 = x611 * x104;
	const FLT x613 = (x609 * x538) + (x605 * (*_x0).Object.Pose.Rot[1]) + (-1 * x78 * x595) + (-1 * x608 * x542) +
					 (-1 * x602 * x518) + (-1 * x601 * x556) + (x79 * x598) + (x606 * (*_x0).Object.Pose.Rot[3]);
	const FLT x614 = (x613 * x555) + (-1 * x49 * x612) + (x607 * x545) + (x610 * x109);
	const FLT x615 = (-1 * x88 * x612) + (-1 * x613 * x550) + (-1 * x610 * x545) + (x607 * x109);
	const FLT x616 = (x613 * x109) + (x607 * x550) + (x85 * x612) + (-1 * x610 * x555);
	const FLT x617 = (x616 * sensor_pt[0]) + (-1 * x614 * sensor_pt[2]) + (x615 * sensor_pt[1]);
	const FLT x618 = (x611 * x109) + (x610 * x550) + (-1 * x613 * x545) + (x607 * x555);
	const FLT x619 = (x614 * sensor_pt[1]) + (-1 * x618 * sensor_pt[0]) + (x615 * sensor_pt[2]);
	const FLT x620 = (-1 * x616 * x573) + (-1 * x617 * x564) + (x619 * x572) + (x618 * x568);
	const FLT x621 = (x618 * sensor_pt[2]) + (x615 * sensor_pt[0]) + (-1 * x616 * sensor_pt[1]);
	const FLT x622 = (-1 * x614 * x568) + (x616 * x569) + (-1 * x619 * x567) + (x621 * x564);
	const FLT x623 = (x614 * x573) + (x617 * x567) + (-1 * x621 * x572) + (-1 * x618 * x569);
	const FLT x624 = (x41 * x623) + (x45 * x620) + (-1 * x43 * x622);
	const FLT x625 = (x47 * x622) + (-1 * x45 * x623) + (x41 * x620);
	const FLT x626 = x622 + (x624 * x229) + (-1 * x625 * x218);
	const FLT x627 = (-1 * x47 * x620) + (x43 * x623) + (x41 * x622);
	const FLT x628 = x620 + (-1 * x624 * x206) + (x627 * x218);
	const FLT x629 = (x625 * x206) + x623 + (-1 * x627 * x229);
	const FLT x630 = (x629 * x235) + (x628 * x225);
	const FLT x631 = (-1 * x273 * (x630 + (x626 * x272))) + (x626 * x253);
	const FLT x632 = x631 * x276;
	const FLT x633 = x299 * x260 * x304;
	const FLT x634 = (-1 * x626 * x241) + (x630 * x238);
	const FLT x635 = ((x628 * x244) + (-1 * x629 * x243)) * x245;
	const FLT x636 = (-1 * x635) + (x634 * x148);
	const FLT x637 = x632 * x256;
	const FLT x638 = (x254 * (x637 + (-1 * x632 * x255))) + (x631 * x283);
	const FLT x639 = (x632 * x258) + (x638 * x254);
	const FLT x640 =
		x635 +
		(-1 * x308 *
		 (x634 + (x639 * x306) + (-1 * x636 * x302) + (x632 * x633) +
		  (-1 * x298 *
		   ((x293 *
			 ((x631 * x290) + (x639 * x254) +
			  (x254 * (x639 + (x254 * (x638 + (x254 * ((-1 * x632 * x280) + x637 + (x632 * x261))) + (x632 * x262))) +
					   (x632 * x263))) +
			  (x632 * x264))) +
			(-1 * x636 * x271)))));
	const FLT x641 = x529 + x597;
	const FLT x642 = x523 + x594;
	const FLT x643 = (x641 * x527) + (x642 * x533) + (x71 * x522) + (x596 * x515);
	const FLT x644 = x71 * x643;
	const FLT x645 = x76 * x641;
	const FLT x646 = x68 * x643;
	const FLT x647 = (x646 * x538) + x516 + (x643 * x548) + (x644 * x542) + (-1 * x541) + (x79 * x642) +
					 (-1 * x604 * x643) + (-1 * x645 * (*_x0).Object.Pose.Rot[2]);
	const FLT x648 = x74 * x643;
	const FLT x649 = x63 * x643;
	const FLT x650 = (x646 * x542) + x557 + (-1 * x648 * x518) + (x80 * x642) + (-1 * x644 * x538) + x558 +
					 (-1 * x645 * (*_x0).Object.Pose.Rot[3]) + (-1 * x649 * x536);
	const FLT x651 = (x645 * (*_x0).Object.Pose.Rot[0]) + (x644 * x536) + (-1 * x648 * x542) + x546 + (x78 * x642) +
					 (-1 * x649 * x538) + (-1 * x646 * x518) + x547;
	const FLT x652 = (x648 * x538) + (-1 * x649 * x542) + x552 + (x645 * (*_x0).Object.Pose.Rot[1]) + (x77 * x642) +
					 (-1 * x643 * x556) + (-1 * x553) + (-1 * x644 * x518);
	const FLT x653 = (x652 * x555) + (x647 * x545) + (-1 * x651 * x550) + (x650 * x109);
	const FLT x654 = (-1 * x652 * x550) + (-1 * x650 * x545) + (-1 * x651 * x555) + (x647 * x109);
	const FLT x655 = (x652 * x109) + (-1 * x650 * x555) + (x651 * x545) + (x647 * x550);
	const FLT x656 = (x655 * sensor_pt[0]) + (-1 * x653 * sensor_pt[2]) + (x654 * sensor_pt[1]);
	const FLT x657 = (x647 * x555) + (-1 * x652 * x545) + (x651 * x109) + (x650 * x550);
	const FLT x658 = (x653 * sensor_pt[1]) + (x654 * sensor_pt[2]) + (-1 * x657 * sensor_pt[0]);
	const FLT x659 = (x657 * x568) + (x658 * x572) + (-1 * x656 * x564) + (-1 * x655 * x573);
	const FLT x660 = x40 * x659;
	const FLT x661 = (x657 * sensor_pt[2]) + (-1 * x655 * sensor_pt[1]) + (x654 * sensor_pt[0]);
	const FLT x662 = (-1 * x653 * x568) + (x661 * x564) + (-1 * x658 * x567) + (x655 * x569);
	const FLT x663 = (x656 * x567) + (x653 * x573) + (-1 * x657 * x569) + (-1 * x661 * x572);
	const FLT x664 = x40 * x663;
	const FLT x665 = (x34 * x660) + (x36 * x664) + (-1 * x43 * x662);
	const FLT x666 = (-1 * x34 * x664) + (x36 * x660) + (x47 * x662);
	const FLT x667 = x662 + (x665 * x229) + (-1 * x666 * x218);
	const FLT x668 = (-1 * x47 * x659) + (x37 * x664) + (x41 * x662);
	const FLT x669 = x659 + (x668 * x218) + (-1 * x665 * x206);
	const FLT x670 = x663 + (x666 * x206) + (-1 * x668 * x229);
	const FLT x671 = (x670 * x235) + (x669 * x225);
	const FLT x672 = (-1 * x273 * (x671 + (x667 * x272))) + (x667 * x253);
	const FLT x673 = (-1 * x667 * x241) + (x671 * x238);
	const FLT x674 = ((x669 * x244) + (-1 * x670 * x243)) * x245;
	const FLT x675 = (-1 * x674) + (x673 * x148);
	const FLT x676 = x672 * x278;
	const FLT x677 = (x254 * (x676 + (-1 * x672 * x284))) + (x672 * x283);
	const FLT x678 = (x672 * x287) + (x677 * x254);
	const FLT x679 =
		x674 +
		(-1 * x308 *
		 ((x678 * x306) + x673 +
		  (-1 * x298 *
		   ((x293 *
			 ((x678 * x254) + (x672 * x290) +
			  (x254 * (x678 + (x254 * (x677 + (x254 * ((-1 * x672 * x281) + x676 + (x672 * x277))) + (x672 * x282))) +
					   (x672 * x286))) +
			  (x672 * x289))) +
			(-1 * x675 * x271))) +
		  (x672 * x305) + (-1 * x675 * x302)));
	const FLT x680 = -1 * x411;
	const FLT x681 = 1 + (-1 * x410);
	const FLT x682 = x681 + x680;
	const FLT x683 = x415 + x418;
	const FLT x684 = (x683 * x235) + (x682 * x225);
	const FLT x685 = x437 + x424;
	const FLT x686 = (-1 * x685 * x241) + (x684 * x238);
	const FLT x687 = ((x682 * x244) + (-1 * x683 * x243)) * x245;
	const FLT x688 = (-1 * x687) + (x686 * x148);
	const FLT x689 = (-1 * x273 * (x684 + (x685 * x272))) + (x685 * x253);
	const FLT x690 = x689 * x278;
	const FLT x691 = (x254 * (x690 + (-1 * x689 * x284))) + (x689 * x283);
	const FLT x692 = (x689 * x287) + (x691 * x254);
	const FLT x693 =
		x687 +
		(-1 * x308 *
		 ((x689 * x305) +
		  (-1 * x298 *
		   ((x293 *
			 ((x692 * x254) + (x689 * x290) +
			  (x254 * (x692 + (x254 * ((x254 * ((-1 * x689 * x281) + x690 + (x689 * x277))) + x691 + (x689 * x282))) +
					   (x689 * x286))) +
			  (x689 * x289))) +
			(-1 * x688 * x271))) +
		  x686 + (x692 * x306) + (-1 * x688 * x302)));
	const FLT x694 = -1 * x435;
	const FLT x695 = x681 + x694;
	const FLT x696 = x424 + x423;
	const FLT x697 = x452 + x440;
	const FLT x698 = (x697 * x235) + (x696 * x225);
	const FLT x699 = (-1 * x273 * (x698 + (x695 * x272))) + (x695 * x253);
	const FLT x700 = (-1 * x695 * x241) + (x698 * x238);
	const FLT x701 = ((x696 * x244) + (-1 * x697 * x243)) * x245;
	const FLT x702 = (-1 * x701) + (x700 * x148);
	const FLT x703 = x699 * x278;
	const FLT x704 = (x254 * (x703 + (-1 * x699 * x284))) + (x699 * x283);
	const FLT x705 = (x699 * x287) + (x704 * x254);
	const FLT x706 =
		x701 +
		(-1 * x308 *
		 (x700 + (-1 * x702 * x302) + (x705 * x306) + (x699 * x305) +
		  (-1 * x298 *
		   ((x293 * ((x699 * x290) +
					 (x254 * ((x254 * ((x254 * ((-1 * x699 * x281) + x703 + (x699 * x277))) + x704 + (x699 * x282))) +
							  x705 + (x699 * x286))) +
					 (x705 * x254) + (x699 * x289))) +
			(-1 * x702 * x271)))));
	const FLT x707 = x440 + x439;
	const FLT x708 = x419 + x415;
	const FLT x709 = 1 + x694 + x680;
	const FLT x710 = (x709 * x235) + (x708 * x225);
	const FLT x711 = (-1 * x273 * (x710 + (x707 * x272))) + (x707 * x253);
	const FLT x712 = (-1 * x707 * x241) + (x710 * x238);
	const FLT x713 = ((x708 * x244) + (-1 * x709 * x243)) * x245;
	const FLT x714 = (-1 * x713) + (x712 * x148);
	const FLT x715 = x711 * x278;
	const FLT x716 = (x254 * (x715 + (-1 * x711 * x284))) + (x711 * x283);
	const FLT x717 = (x711 * x287) + (x716 * x254);
	const FLT x718 =
		x713 +
		(-1 * x308 *
		 (x712 + (x717 * x306) + (-1 * x714 * x302) + (x711 * x305) +
		  (-1 * x298 *
		   ((x293 *
			 ((x717 * x254) + (x711 * x289) +
			  (x254 * (x717 + (x254 * (x716 + (x254 * ((-1 * x711 * x281) + x715 + (x711 * x277))) + (x711 * x282))) +
					   (x711 * x286))) +
			  (x711 * x290))) +
			(-1 * x714 * x271)))));
	const FLT x719 = -1 * x105;
	const FLT x720 = x49 * x112;
	const FLT x721 = 1. / (x91 * sqrt(x91));
	const FLT x722 = dt * dt * dt;
	const FLT x723 = x722 * x721;
	const FLT x724 = x85 * x723;
	const FLT x725 = x720 * x724;
	const FLT x726 = dt * dt * dt * dt;
	const FLT x727 = (x85 * x85 * x85) * x726;
	const FLT x728 = 2 * (1. / (x91 * x91)) * x95;
	const FLT x729 = 1.0 * x98 * x94;
	const FLT x730 = x729 * x721;
	const FLT x731 = x730 * x726;
	const FLT x732 = x83 * x731;
	const FLT x733 = x728 * x726;
	const FLT x734 = x89 * x85;
	const FLT x735 = x83 * x733;
	const FLT x736 = 2 * x97;
	const FLT x737 = x82 * x736;
	const FLT x738 = x729 * x102;
	const FLT x739 = x82 * x738;
	const FLT x740 = (x730 * x727) + (x731 * x734) + (x85 * x732) + (-1 * x85 * x739) + (-1 * x728 * x727) +
					 (-1 * x733 * x734) + (x85 * x737) + (-1 * x85 * x735);
	const FLT x741 = 1.0 / 2.0 * (1. / (x99 * sqrt(x99)));
	const FLT x742 = x98 * x741;
	const FLT x743 = x740 * x742;
	const FLT x744 = x101 * x106;
	const FLT x745 = 0.5 * x102;
	const FLT x746 = x82 * x745;
	const FLT x747 = x85 * x746;
	const FLT x748 = x94 * x741 * x103;
	const FLT x749 = x85 * x748;
	const FLT x750 = x749 * x740;
	const FLT x751 = 0.5 * x96 * x722;
	const FLT x752 = x751 * x115;
	const FLT x753 = x49 * x752;
	const FLT x754 = x85 * x753;
	const FLT x755 = x748 * x740;
	const FLT x756 = x49 * x755;
	const FLT x757 = x81 * x101;
	const FLT x758 = x86 * x723;
	const FLT x759 = x88 * x755;
	const FLT x760 = x751 * x121;
	const FLT x761 = x751 * x110;
	const FLT x762 = x88 * x85;
	const FLT x763 = x101 * x108;
	const FLT x764 = x88 * x723;
	const FLT x765 = x85 * x764;
	const FLT x766 = (-1 * x765 * x763) + (x762 * x761);
	const FLT x767 = (x758 * x757) + x754 + (-1 * x759 * x108) + (x81 * x750) + (-1 * x725) + x719 +
					 (-1 * x743 * x106) + x766 + (-1 * x86 * x760) + (-1 * x756 * x111) + (-1 * x744 * x747);
	const FLT x768 = x762 * x760;
	const FLT x769 = x49 * x744;
	const FLT x770 = x769 * x724;
	const FLT x771 = x81 * x88;
	const FLT x772 = x765 * x757;
	const FLT x773 = x751 * x118;
	const FLT x774 = x49 * x773;
	const FLT x775 = x85 * x774;
	const FLT x776 = x742 * x111;
	const FLT x777 = x770 + (x86 * x761) + (-1 * x771 * x755) + (-1 * x772) + x768 + (-1 * x747 * x112) + (-1 * x775) +
					 (-1 * x750 * x108) + (-1 * x776 * x740) + (-1 * x763 * x758) + x116 + (x756 * x106);
	const FLT x778 = -1 * x113;
	const FLT x779 = x765 * x744;
	const FLT x780 = x762 * x773;
	const FLT x781 = x49 * x760;
	const FLT x782 = x49 * x757;
	const FLT x783 = (x724 * x782) + (-1 * x85 * x781);
	const FLT x784 = (x758 * x112) + (x750 * x111) + (-1 * x763 * x747) + (-1 * x743 * x108) + (-1 * x780) + x783 +
					 (-1 * x86 * x752) + x778 + (x81 * x756) + (x759 * x106) + x779;
	const FLT x785 = x81 * x742;
	const FLT x786 = x49 * x761;
	const FLT x787 = x49 * x763;
	const FLT x788 = (-1 * x724 * x787) + (x85 * x786);
	const FLT x789 = (x765 * x112) + (-1 * x762 * x752);
	const FLT x790 = (-1 * x744 * x758) + (x759 * x111) + x788 + (-1 * x785 * x740) + x789 + x107 + (-1 * x750 * x106) +
					 (x86 * x773) + (-1 * x747 * x757) + (-1 * x756 * x108);
	const FLT x791 = (x790 * sensor_pt[0]) + (-1 * x777 * sensor_pt[2]) + (x784 * sensor_pt[1]);
	const FLT x792 = (x767 * sensor_pt[2]) + (-1 * x790 * sensor_pt[1]) + (x784 * sensor_pt[0]);
	const FLT x793 = (x791 * x567) + (-1 * x792 * x572) + (-1 * x767 * x569) + (x777 * x573);
	const FLT x794 = x40 * x793;
	const FLT x795 = (x777 * sensor_pt[1]) + (-1 * x767 * sensor_pt[0]) + (x784 * sensor_pt[2]);
	const FLT x796 = (-1 * x777 * x568) + (x790 * x569) + (-1 * x795 * x567) + (x792 * x564);
	const FLT x797 = x40 * x796;
	const FLT x798 = (x767 * x568) + (x795 * x572) + (-1 * x791 * x564) + (-1 * x790 * x573);
	const FLT x799 = (x37 * x794) + (-1 * x47 * x798) + (x36 * x797);
	const FLT x800 = x40 * x798;
	const FLT x801 = (x36 * x794) + (x34 * x800) + (-1 * x37 * x797);
	const FLT x802 = x798 + (x799 * x218) + (-1 * x801 * x206);
	const FLT x803 = (-1 * x34 * x794) + (x32 * x797) + (x36 * x800);
	const FLT x804 = (-1 * x799 * x229) + x793 + (x803 * x206);
	const FLT x805 = (x804 * x235) + (x802 * x225);
	const FLT x806 = x796 + (-1 * x803 * x218) + (x801 * x229);
	const FLT x807 = (-1 * x806 * x241) + (x805 * x238);
	const FLT x808 = ((x802 * x244) + (-1 * x804 * x243)) * x245;
	const FLT x809 = (-1 * x808) + (x807 * x148);
	const FLT x810 = (-1 * x273 * (x805 + (x806 * x272))) + (x806 * x253);
	const FLT x811 = x810 * x278;
	const FLT x812 = (x254 * (x811 + (-1 * x810 * x284))) + (x810 * x283);
	const FLT x813 = (x810 * x287) + (x812 * x254);
	const FLT x814 =
		x808 +
		(-1 * x308 *
		 (x807 + (x813 * x306) +
		  (-1 * x298 *
		   ((x293 *
			 ((x813 * x254) + (x810 * x290) +
			  (x254 * (x813 + (x254 * (x812 + (x254 * ((-1 * x810 * x281) + x811 + (x810 * x277))) + (x810 * x282))) +
					   (x810 * x286))) +
			  (x810 * x289))) +
			(-1 * x809 * x271))) +
		  (-1 * x809 * x302) + (x810 * x305)));
	const FLT x815 = x88 * x86;
	const FLT x816 = x82 * x88;
	const FLT x817 = x88 * x88 * x88;
	const FLT x818 = (x731 * x817) + (-1 * x733 * x817) + (x731 * x815) + (x736 * x816) + (-1 * x738 * x816) +
					 (-1 * x733 * x815) + (x88 * x732) + (-1 * x88 * x735);
	const FLT x819 = x742 * x106;
	const FLT x820 = x89 * x723;
	const FLT x821 = x88 * x753;
	const FLT x822 = x818 * x108;
	const FLT x823 = x88 * x748;
	const FLT x824 = x764 * x720;
	const FLT x825 = x49 * x748;
	const FLT x826 = x825 * x111;
	const FLT x827 = x745 * x816;
	const FLT x828 = x81 * x818;
	const FLT x829 = (x749 * x828) + x821 + (-1 * x763 * x820) + x116 + (-1 * x824) + (-1 * x818 * x826) +
					 (-1 * x818 * x819) + (-1 * x744 * x827) + (-1 * x822 * x823) + (x89 * x761) + (-1 * x768) + x772;
	const FLT x830 = -1 * x107;
	const FLT x831 = x88 * x781;
	const FLT x832 = x818 * x106;
	const FLT x833 = x764 * x782;
	const FLT x834 = x749 * x111;
	const FLT x835 = x742 * x108;
	const FLT x836 = (-1 * x818 * x835) + (-1 * x763 * x827) + (x825 * x828) + x833 + (-1 * x831) + x830 +
					 (x818 * x834) + (-1 * x89 * x773) + x789 + (x823 * x832) + (x744 * x820);
	const FLT x837 = x749 * x108;
	const FLT x838 = x771 * x748;
	const FLT x839 = (x769 * x764) + (-1 * x88 * x774);
	const FLT x840 = x766 + (-1 * x827 * x112) + (-1 * x818 * x838) + (x825 * x832) + (-1 * x818 * x837) +
					 (x89 * x760) + (-1 * x776 * x818) + x839 + (-1 * x757 * x820) + x105;
	const FLT x841 = (x840 * sensor_pt[1]) + (-1 * x829 * sensor_pt[0]) + (x836 * sensor_pt[2]);
	const FLT x842 = x823 * x111;
	const FLT x843 = (x88 * x786) + (-1 * x764 * x787);
	const FLT x844 = (-1 * x89 * x752) + (x820 * x112) + (-1 * x749 * x832) + x778 + (-1 * x757 * x827) +
					 (-1 * x785 * x818) + (x818 * x842) + x780 + (-1 * x822 * x825) + x843 + (-1 * x779);
	const FLT x845 = (-1 * x844 * sensor_pt[1]) + (x829 * sensor_pt[2]) + (x836 * sensor_pt[0]);
	const FLT x846 = (x569 * x844) + (x564 * x845) + (-1 * x567 * x841) + (-1 * x568 * x840);
	const FLT x847 = x40 * x846;
	const FLT x848 = (x844 * sensor_pt[0]) + (-1 * x840 * sensor_pt[2]) + (x836 * sensor_pt[1]);
	const FLT x849 = (x568 * x829) + (x572 * x841) + (-1 * x564 * x848) + (-1 * x573 * x844);
	const FLT x850 = x40 * x849;
	const FLT x851 = (x567 * x848) + (x573 * x840) + (-1 * x569 * x829) + (-1 * x572 * x845);
	const FLT x852 = x40 * x851;
	const FLT x853 = (x36 * x852) + (-1 * x37 * x847) + (x34 * x850);
	const FLT x854 = (-1 * x34 * x852) + (x32 * x847) + (x36 * x850);
	const FLT x855 = x846 + (x853 * x229) + (-1 * x854 * x218);
	const FLT x856 = (x43 * x851) + (-1 * x47 * x849) + (x36 * x847);
	const FLT x857 = x849 + (x856 * x218) + (-1 * x853 * x206);
	const FLT x858 = x851 + (x854 * x206) + (-1 * x856 * x229);
	const FLT x859 = (x858 * x235) + (x857 * x225);
	const FLT x860 = (-1 * x273 * (x859 + (x855 * x272))) + (x855 * x253);
	const FLT x861 = x860 * x278;
	const FLT x862 = (x254 * (x861 + (-1 * x860 * x284))) + (x860 * x283);
	const FLT x863 = (x860 * x287) + (x862 * x254);
	const FLT x864 = (-1 * x855 * x241) + (x859 * x238);
	const FLT x865 = ((x857 * x244) + (-1 * x858 * x243)) * x245;
	const FLT x866 = (-1 * x865) + (x864 * x148);
	const FLT x867 =
		x865 +
		(-1 * x308 *
		 (x864 +
		  (-1 * x298 *
		   ((x293 *
			 ((x863 * x254) + (x860 * x289) + (x860 * x290) +
			  (x254 * (x863 + (x254 * (x862 + (x254 * ((-1 * x860 * x281) + x861 + (x860 * x277))) + (x860 * x282))) +
					   (x860 * x286))))) +
			(-1 * x866 * x271))) +
		  (x860 * x305) + (-1 * x866 * x302) + (x863 * x306)));
	const FLT x868 = x49 * x731;
	const FLT x869 = x49 * x733;
	const FLT x870 = x49 * x49 * x49;
	const FLT x871 = (x89 * x868) + (-1 * x733 * x870) + (-1 * x86 * x869) + (x86 * x868) + (x49 * x737) +
					 (-1 * x49 * x739) + (x731 * x870) + (-1 * x89 * x869);
	const FLT x872 = x81 * x871;
	const FLT x873 = x83 * x723;
	const FLT x874 = x871 * x106;
	const FLT x875 = (-1 * x83 * x760) + (x871 * x834) + (-1 * x871 * x835) + (x874 * x823) + x725 + x719 + x839 +
					 (x872 * x825) + (-1 * x787 * x746) + (-1 * x754) + (x757 * x873);
	const FLT x876 = x871 * x108;
	const FLT x877 = (x749 * x872) + (-1 * x876 * x823) + x113 + (-1 * x769 * x746) + x783 + (x83 * x752) +
					 (-1 * x871 * x819) + (-1 * x871 * x826) + x843 + (-1 * x873 * x112);
	const FLT x878 = x873 * x101;
	const FLT x879 = (-1 * x83 * x773) + (-1 * x871 * x837) + x831 + (-1 * x871 * x838) + (-1 * x833) +
					 (-1 * x776 * x871) + x788 + x830 + (x878 * x106) + (x874 * x825) + (-1 * x720 * x746);
	const FLT x880 = (x879 * sensor_pt[1]) + (x875 * sensor_pt[2]) + (-1 * x877 * sensor_pt[0]);
	const FLT x881 = x824 + (x871 * x842) + x775 + (-1 * x785 * x871) + (-1 * x876 * x825) + (-1 * x782 * x746) +
					 (-1 * x770) + (-1 * x878 * x108) + (x83 * x761) + x116 + (-1 * x821) + (-1 * x749 * x874);
	const FLT x882 = (x881 * sensor_pt[0]) + (-1 * x879 * sensor_pt[2]) + (x875 * sensor_pt[1]);
	const FLT x883 = (x568 * x877) + (-1 * x573 * x881) + (x572 * x880) + (-1 * x564 * x882);
	const FLT x884 = (x877 * sensor_pt[2]) + (-1 * x881 * sensor_pt[1]) + (x875 * sensor_pt[0]);
	const FLT x885 = (x569 * x881) + (x564 * x884) + (-1 * x567 * x880) + (-1 * x568 * x879);
	const FLT x886 = (x573 * x879) + (-1 * x569 * x877) + (x567 * x882) + (-1 * x572 * x884);
	const FLT x887 = x40 * x886;
	const FLT x888 = (x36 * x887) + (x45 * x883) + (-1 * x43 * x885);
	const FLT x889 = (-1 * x34 * x887) + (x41 * x883) + (x47 * x885);
	const FLT x890 = x885 + (x888 * x229) + (-1 * x889 * x218);
	const FLT x891 = (x41 * x885) + (-1 * x47 * x883) + (x37 * x887);
	const FLT x892 = x883 + (x891 * x218) + (-1 * x888 * x206);
	const FLT x893 = x886 + (x889 * x206) + (-1 * x891 * x229);
	const FLT x894 = (x893 * x235) + (x892 * x225);
	const FLT x895 = (-1 * x273 * (x894 + (x890 * x272))) + (x890 * x253);
	const FLT x896 = (-1 * x890 * x241) + (x894 * x238);
	const FLT x897 = ((x892 * x244) + (-1 * x893 * x243)) * x245;
	const FLT x898 = (-1 * x897) + (x896 * x148);
	const FLT x899 = x895 * x278;
	const FLT x900 = (x254 * (x899 + (-1 * x895 * x284))) + (x895 * x283);
	const FLT x901 = (x895 * x287) + (x900 * x254);
	const FLT x902 =
		x897 +
		(-1 * x308 *
		 ((-1 * x898 * x302) + (x895 * x305) + x896 + (x901 * x306) +
		  (-1 * x298 *
		   ((x293 *
			 ((x901 * x254) +
			  (x254 * (x901 + (x254 * (x900 + (x254 * ((-1 * x895 * x281) + x899 + (x895 * x277))) + (x895 * x282))) +
					   (x895 * x286))) +
			  (x895 * x289) + (x895 * x290))) +
			(-1 * x898 * x271)))));
	const FLT x903 = dt * x424;
	const FLT x904 = dt * x423;
	const FLT x905 = (-1 * x904) + x903;
	const FLT x906 = -1 * dt * x410;
	const FLT x907 = (-1 * dt * x411) + dt;
	const FLT x908 = x907 + x906;
	const FLT x909 = dt * x418;
	const FLT x910 = dt * x415;
	const FLT x911 = x910 + x909;
	const FLT x912 = (x911 * x235) + (x908 * x225);
	const FLT x913 = (-1 * x273 * (x912 + (x905 * x272))) + (x905 * x253);
	const FLT x914 = (-1 * x905 * x241) + (x912 * x238);
	const FLT x915 = ((x908 * x244) + (-1 * x911 * x243)) * x245;
	const FLT x916 = (-1 * x915) + (x914 * x148);
	const FLT x917 = x913 * x278;
	const FLT x918 = (x254 * (x917 + (-1 * x913 * x284))) + (x913 * x283);
	const FLT x919 = (x913 * x287) + (x918 * x254);
	const FLT x920 =
		x915 +
		(-1 * x308 *
		 ((x919 * x306) + (-1 * x916 * x302) + x914 + (x913 * x305) +
		  (-1 * x298 *
		   ((x293 *
			 ((x913 * x290) + (x919 * x254) +
			  (x254 * (x919 + (x254 * (x918 + (x254 * ((-1 * x913 * x281) + x917 + (x913 * x277))) + (x913 * x282))) +
					   (x913 * x286))) +
			  (x913 * x289))) +
			(-1 * x916 * x271)))));
	const FLT x921 = x903 + x904;
	const FLT x922 = dt * x440;
	const FLT x923 = dt * x439;
	const FLT x924 = (-1 * x923) + x922;
	const FLT x925 = (x924 * x235) + (x921 * x225);
	const FLT x926 = -1 * dt * x435;
	const FLT x927 = dt + x906 + x926;
	const FLT x928 = (-1 * x927 * x241) + (x925 * x238);
	const FLT x929 = ((x921 * x244) + (-1 * x924 * x243)) * x245;
	const FLT x930 = (-1 * x929) + (x928 * x148);
	const FLT x931 = (-1 * x273 * (x925 + (x927 * x272))) + (x927 * x253);
	const FLT x932 = x931 * x278;
	const FLT x933 = (x254 * (x932 + (-1 * x931 * x284))) + (x931 * x283);
	const FLT x934 = (x931 * x287) + (x933 * x254);
	const FLT x935 =
		x929 +
		(-1 * x308 *
		 (x928 + (x934 * x306) +
		  (-1 * x298 *
		   ((x293 * ((x934 * x254) + (x931 * x290) +
					 (x254 * ((x254 * (x933 + (x254 * ((-1 * x931 * x281) + x932 + (x931 * x277))) + (x931 * x282))) +
							  x934 + (x931 * x286))) +
					 (x931 * x289))) +
			(-1 * x930 * x271))) +
		  (-1 * x930 * x302) + (x931 * x305)));
	const FLT x936 = x922 + x923;
	const FLT x937 = (-1 * x909) + x910;
	const FLT x938 = x907 + x926;
	const FLT x939 = (x938 * x235) + (x937 * x225);
	const FLT x940 = (-1 * x273 * (x939 + (x936 * x272))) + (x936 * x253);
	const FLT x941 = x940 * x276;
	const FLT x942 = (-1 * x936 * x241) + (x939 * x238);
	const FLT x943 = ((x937 * x244) + (-1 * x938 * x243)) * x245;
	const FLT x944 = (-1 * x943) + (x942 * x148);
	const FLT x945 = x940 * x278;
	const FLT x946 = (x254 * (x945 + (-1 * x941 * x255))) + (x940 * x283);
	const FLT x947 = (x940 * x287) + (x946 * x254);
	const FLT x948 =
		x943 +
		(-1 * x308 *
		 (x942 + (x947 * x306) +
		  (-1 * x298 *
		   ((x293 *
			 ((x947 * x254) +
			  (x254 * (x947 + (x254 * (x946 + (x254 * ((-1 * x941 * x280) + x945 + (x940 * x277))) + (x940 * x282))) +
					   (x940 * x286))) +
			  (x940 * x289) + (x940 * x290))) +
			(-1 * x944 * x271))) +
		  (x941 * x633) + (-1 * x944 * x302)));
	const FLT x949 = x297 * x295;
	const FLT x950 = ((-1 * x949) + x301) * x308;
	const FLT x951 = ((x949 * x291) + (-1 * x291 * x301)) * x308;
	const FLT x952 = x308 * ((-1 * x949 * x270) + x302);
	const FLT x953 = x275 * x250;
	const FLT x954 = x953 * x248 * x136;
	const FLT x955 = -1 * x954 * x278;
	const FLT x956 = (x254 * (x955 + (x954 * x284))) + (-1 * x954 * x283);
	const FLT x957 = (-1 * x954 * x287) + (x956 * x254);
	const FLT x958 = x240 * x136 * (1 + x147);
	const FLT x959 = x958 * x148;
	const FLT x960 =
		x308 *
		((-1 * x298 *
		  ((-1 * x959 * x271) +
		   (x293 *
			((x957 * x254) + (-1 * x954 * x290) +
			 (x254 *
			  (x957 + (x254 * (x956 + (x254 * ((x954 * x281) + x955 + (-1 * x954 * x277))) + (-1 * x954 * x282))) +
			   (-1 * x954 * x286))) +
			 (-1 * x954 * x289))) +
		   x248 + (-1 * x292 * x265 * x251))) +
		 x958 + (-1 * x953 * x272 * x293 * x303) + (x957 * x306) + (-1 * x959 * x302));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[0]) / sizeof(FLT),
						x309 + (x309 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[1]) / sizeof(FLT),
						x363 + (x363 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.AxisAngleRot[2]) / sizeof(FLT),
						x407 + (x407 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[0]) / sizeof(FLT),
						x434 + (x434 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[1]) / sizeof(FLT),
						x451 + (x451 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Lighthouse.Pos[2]) / sizeof(FLT),
						x464 + (x464 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Acc[0]) / sizeof(FLT),
						x485 + (x485 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Acc[1]) / sizeof(FLT),
						x500 + (x500 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Acc[2]) / sizeof(FLT),
						x512 + (x512 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[0]) / sizeof(FLT),
						x593 + (x593 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[1]) / sizeof(FLT),
						x640 + (x640 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.AxisAngleRot[2]) / sizeof(FLT),
						x679 + (x679 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[0]) / sizeof(FLT),
						x693 + (x693 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[1]) / sizeof(FLT),
						x706 + (x706 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Pose.Pos[2]) / sizeof(FLT),
						x718 + (x718 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x814 + (x814 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x867 + (x867 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x902 + (x902 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[0]) / sizeof(FLT),
						x920 + (x920 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[1]) / sizeof(FLT),
						x935 + (x935 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, Object.Velocity.Pos[2]) / sizeof(FLT),
						x948 + (x948 * x311));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD1.curve) / sizeof(FLT),
						(-1 * x950 * x311) + (-1 * x950));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD1.gibmag) / sizeof(FLT), sin(x310));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD1.gibpha) / sizeof(FLT), x311);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD1.ogeemag) / sizeof(FLT),
						(-1 * x951 * x311) + (-1 * x951));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD1.ogeephase) / sizeof(FLT),
						(-1 * x952 * x311) + (-1 * x952));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD1.phase) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveJointKalmanErrorModel, BSD1.tilt) / sizeof(FLT),
						(-1 * x960 * x311) + (-1 * x960));
}

// Full version Jacobian of SurviveJointKalmanErrorModel_LightMeas_y_gen2 wrt
// [(*error_model).Lighthouse.AxisAngleRot[0], (*error_model).Lighthouse.AxisAngleRot[1],
// (*error_model).Lighthouse.AxisAngleRot[2], (*error_model).Lighthouse.Pos[0], (*error_model).Lighthouse.Pos[1],
// (*error_model).Lighthouse.Pos[2], (*error_model).Object.Acc[0], (*error_model).Object.Acc[1],
// (*error_model).Object.Acc[2], (*error_model).Object.IMUBias.AccBias[0], (*error_model).Object.IMUBias.AccBias[1],
// (*error_model).Object.IMUBias.AccBias[2], (*error_model).Object.IMUBias.AccScale[0],
// (*error_model).Object.IMUBias.AccScale[1], (*error_model).Object.IMUBias.AccScale[2],
// (*error_model).Object.IMUBias.GyroBias[0], (*error_model).Object.IMUBias.GyroBias[1],
// (*error_model).Object.IMUBias.GyroBias[2], (*error_model).Object.IMUBias.IMUCorrection[0],
// (*error_model).Object.IMUBias.IMUCorrection[1], (*error_model).Object.IMUBias.IMUCorrection[2],
// (*error_model).Object.Pose.AxisAngleRot[0], (*error_model).Object.Pose.AxisAngleRot[1],
// (*error_model).Object.Pose.AxisAngleRot[2], (*error_model).Object.Pose.Pos[0], (*error_model).Object.Pose.Pos[1],
// (*error_model).Object.Pose.Pos[2], (*error_model).Object.Velocity.AxisAngleRot[0],
// (*error_model).Object.Velocity.AxisAngleRot[1], (*error_model).Object.Velocity.AxisAngleRot[2],
// (*error_model).Object.Velocity.Pos[0], (*error_model).Object.Velocity.Pos[1], (*error_model).Object.Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f4d1c396340>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3966d0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c396280>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3967f0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3962e0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c396850>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3965e0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c396550>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c396760>, <cnkalman.codegen.WrapMember object at 0x7f4d1c396490>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c3961f0>, <cnkalman.codegen.WrapMember object at 0x7f4d1c3966a0>,
// <cnkalman.codegen.WrapMember object at 0x7f4d1c396370>, <cnkalman.codegen.WrapMember object at 0x7f4d1c396400>]

static inline void SurviveJointKalmanErrorModel_LightMeas_y_gen2_jac_error_model_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveJointKalmanModel *_x0,
	const SurviveJointKalmanErrorModel *error_model, const FLT *sensor_pt) {
	if (hx != 0) {
		hx->data[0] = SurviveJointKalmanErrorModel_LightMeas_y_gen2(dt, _x0, error_model, sensor_pt);
	}
	if (Hx != 0) {
		SurviveJointKalmanErrorModel_LightMeas_y_gen2_jac_error_model(Hx, dt, _x0, error_model, sensor_pt);
	}
}
// Jacobian of SurviveJointKalmanErrorModel_LightMeas_y_gen2 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]
static inline void
SurviveJointKalmanErrorModel_LightMeas_y_gen2_jac_sensor_pt(CnMat *Hx, const FLT dt, const SurviveJointKalmanModel *_x0,
															const SurviveJointKalmanErrorModel *error_model,
															const FLT *sensor_pt) {
	const FLT x0 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[0];
	const FLT x1 = sin(x0);
	const FLT x2 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[1];
	const FLT x3 = cos(x2);
	const FLT x4 = 0.5 * (*error_model).Object.Pose.AxisAngleRot[2];
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = cos(x0);
	const FLT x8 = cos(x4);
	const FLT x9 = sin(x2);
	const FLT x10 = x8 * x9;
	const FLT x11 = (x7 * x10) + (x1 * x6);
	const FLT x12 = x5 * x9;
	const FLT x13 = x3 * x8;
	const FLT x14 = (x7 * x13) + (x1 * x12);
	const FLT x15 = (x6 * x7) + (-1 * x1 * x10);
	const FLT x16 = (x1 * x13) + (-1 * x7 * x12);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x15 * x15) + (x11 * x11) + (x14 * x14));
	const FLT x18 = x15 * x17;
	const FLT x19 = x11 * x17;
	const FLT x20 = x14 * x17;
	const FLT x21 = x17 * x16;
	const FLT x22 = (x20 * (*_x0).Object.Pose.Rot[0]) + (-1 * x21 * (*_x0).Object.Pose.Rot[1]) +
					(-1 * x18 * (*_x0).Object.Pose.Rot[3]) + (-1 * x19 * (*_x0).Object.Pose.Rot[2]);
	const FLT x23 = (*_x0).Object.Velocity.AxisAngleRot[2] + (*error_model).Object.Velocity.AxisAngleRot[2];
	const FLT x24 = dt * dt;
	const FLT x25 = x24 * (x23 * x23);
	const FLT x26 = (*_x0).Object.Velocity.AxisAngleRot[0] + (*error_model).Object.Velocity.AxisAngleRot[0];
	const FLT x27 = x24 * (x26 * x26);
	const FLT x28 = (*_x0).Object.Velocity.AxisAngleRot[1] + (*error_model).Object.Velocity.AxisAngleRot[1];
	const FLT x29 = x24 * (x28 * x28);
	const FLT x30 = 1e-10 + x29 + x25 + x27;
	const FLT x31 = sqrt(x30);
	const FLT x32 = 0.5 * x31;
	const FLT x33 = sin(x32);
	const FLT x34 = (1. / x30) * (x33 * x33);
	const FLT x35 = cos(x32);
	const FLT x36 = 1. / sqrt((x34 * x27) + (x35 * x35) + (x34 * x25) + (x34 * x29));
	const FLT x37 = dt * (1. / x31) * x33 * x36;
	const FLT x38 = x37 * x23;
	const FLT x39 = x38 * x22;
	const FLT x40 = (x19 * (*_x0).Object.Pose.Rot[1]) + (x18 * (*_x0).Object.Pose.Rot[0]) +
					(x20 * (*_x0).Object.Pose.Rot[3]) + (-1 * x21 * (*_x0).Object.Pose.Rot[2]);
	const FLT x41 = x36 * x35;
	const FLT x42 = x40 * x41;
	const FLT x43 = (x20 * (*_x0).Object.Pose.Rot[1]) + (x21 * (*_x0).Object.Pose.Rot[0]) +
					(-1 * x19 * (*_x0).Object.Pose.Rot[3]) + (x18 * (*_x0).Object.Pose.Rot[2]);
	const FLT x44 = x37 * x28;
	const FLT x45 = x43 * x44;
	const FLT x46 = (-1 * x18 * (*_x0).Object.Pose.Rot[1]) + (x19 * (*_x0).Object.Pose.Rot[0]) +
					(x20 * (*_x0).Object.Pose.Rot[2]) + (x21 * (*_x0).Object.Pose.Rot[3]);
	const FLT x47 = x46 * x37;
	const FLT x48 = x47 * x26;
	const FLT x49 = x48 + (-1 * x45) + x39 + x42;
	const FLT x50 = x41 * x46;
	const FLT x51 = x43 * x38;
	const FLT x52 = x37 * x26;
	const FLT x53 = x52 * x40;
	const FLT x54 = x44 * x22;
	const FLT x55 = (-1 * x54) + x53 + (-1 * x50) + (-1 * x51);
	const FLT x56 = x54 + x51 + (-1 * x53) + x50;
	const FLT x57 = 2 * x56;
	const FLT x58 = 1 + (x57 * x55) + (-2 * (x49 * x49));
	const FLT x59 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[2];
	const FLT x60 = cos(x59);
	const FLT x61 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[1];
	const FLT x62 = sin(x61);
	const FLT x63 = 0.5 * (*error_model).Lighthouse.AxisAngleRot[0];
	const FLT x64 = sin(x63);
	const FLT x65 = x64 * x62;
	const FLT x66 = sin(x59);
	const FLT x67 = cos(x63);
	const FLT x68 = cos(x61);
	const FLT x69 = x67 * x68;
	const FLT x70 = (x66 * x69) + (-1 * x60 * x65);
	const FLT x71 = x64 * x68;
	const FLT x72 = x62 * x67;
	const FLT x73 = (x72 * x60) + (x71 * x66);
	const FLT x74 = (x60 * x69) + (x65 * x66);
	const FLT x75 = (x71 * x60) + (-1 * x72 * x66);
	const FLT x76 = 1. / sqrt((x75 * x75) + (x74 * x74) + (x70 * x70) + (x73 * x73));
	const FLT x77 = x74 * x76;
	const FLT x78 = x75 * x76;
	const FLT x79 = x70 * x76;
	const FLT x80 = x73 * x76;
	const FLT x81 = (x80 * (*_x0).Lighthouse.Rot[1]) + (x77 * (*_x0).Lighthouse.Rot[3]) +
					(x79 * (*_x0).Lighthouse.Rot[0]) + (-1 * x78 * (*_x0).Lighthouse.Rot[2]);
	const FLT x82 = (-1 * x79 * (*_x0).Lighthouse.Rot[1]) + (x80 * (*_x0).Lighthouse.Rot[0]) +
					(x78 * (*_x0).Lighthouse.Rot[3]) + (x77 * (*_x0).Lighthouse.Rot[2]);
	const FLT x83 = (x77 * (*_x0).Lighthouse.Rot[0]) + (-1 * x78 * (*_x0).Lighthouse.Rot[1]) +
					(-1 * x79 * (*_x0).Lighthouse.Rot[3]) + (-1 * x80 * (*_x0).Lighthouse.Rot[2]);
	const FLT x84 = (x77 * (*_x0).Lighthouse.Rot[1]) + (-1 * x80 * (*_x0).Lighthouse.Rot[3]) +
					(x79 * (*_x0).Lighthouse.Rot[2]) + (x78 * (*_x0).Lighthouse.Rot[0]);
	const FLT x85 = 1. / sqrt((x84 * x84) + (x81 * x81) + (x83 * x83) + (x82 * x82));
	const FLT x86 = x82 * x85;
	const FLT x87 = x47 * x23;
	const FLT x88 = x40 * x44;
	const FLT x89 = x41 * x43;
	const FLT x90 = x52 * x22;
	const FLT x91 = x89 + x90 + (-1 * x87) + x88;
	const FLT x92 = 2 * x91;
	const FLT x93 = (-1 * x52 * x43) + (x41 * x22) + (-1 * x40 * x38) + (-1 * x47 * x28);
	const FLT x94 = 2 * x49;
	const FLT x95 = x93 * x94;
	const FLT x96 = x95 + (-1 * x55 * x92);
	const FLT x97 = x85 * x96;
	const FLT x98 = x57 * x93;
	const FLT x99 = (x91 * x94) + (-1 * x98);
	const FLT x100 = x85 * x99;
	const FLT x101 = (x83 * x100) + (x86 * x58) + (-1 * x84 * x97);
	const FLT x102 = 2 * x86;
	const FLT x103 = x81 * x85;
	const FLT x104 = (x84 * x100) + (-1 * x58 * x103) + (x83 * x97);
	const FLT x105 = 2 * x103;
	const FLT x106 = (-1 * x101 * x102) + x58 + (x105 * x104);
	const FLT x107 = (-1 * x56 * sensor_pt[0]) + (x93 * sensor_pt[2]) + (x91 * sensor_pt[1]);
	const FLT x108 = (-1 * x49 * sensor_pt[1]) + (x93 * sensor_pt[0]) + (x56 * sensor_pt[2]);
	const FLT x109 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x110 = (*error_model).Object.Pose.Pos[1] + (*_x0).Object.Pose.Pos[1] +
					 (2 * ((x49 * x108) + (-1 * x91 * x107))) +
					 (x109 * ((*_x0).Object.Acc[1] + (*error_model).Object.Acc[1])) + sensor_pt[1] +
					 (dt * ((*_x0).Object.Velocity.Pos[1] + (*error_model).Object.Velocity.Pos[1]));
	const FLT x111 = x83 * x85;
	const FLT x112 = (-1 * x91 * sensor_pt[2]) + (x93 * sensor_pt[1]) + (x49 * sensor_pt[0]);
	const FLT x113 = (*_x0).Object.Pose.Pos[0] + (2 * ((x56 * x107) + (-1 * x49 * x112))) +
					 (*error_model).Object.Pose.Pos[0] +
					 (x109 * ((*_x0).Object.Acc[0] + (*error_model).Object.Acc[0])) + sensor_pt[0] +
					 (dt * ((*_x0).Object.Velocity.Pos[0] + (*error_model).Object.Velocity.Pos[0]));
	const FLT x114 = x85 * x113;
	const FLT x115 = (dt * ((*_x0).Object.Velocity.Pos[2] + (*error_model).Object.Velocity.Pos[2])) +
					 (*error_model).Object.Pose.Pos[2] + sensor_pt[2] + (2 * ((x91 * x112) + (-1 * x56 * x108))) +
					 (x109 * ((*_x0).Object.Acc[2] + (*error_model).Object.Acc[2])) + (*_x0).Object.Pose.Pos[2];
	const FLT x116 = x84 * x85;
	const FLT x117 = (x115 * x116) + (x110 * x111) + (-1 * x81 * x114);
	const FLT x118 = (x82 * x114) + (-1 * x110 * x116) + (x111 * x115);
	const FLT x119 = (*_x0).Lighthouse.Pos[1] + (*error_model).Lighthouse.Pos[1];
	const FLT x120 = (*_x0).Lighthouse.Pos[0] + (*error_model).Lighthouse.Pos[0];
	const FLT x121 = (*_x0).Lighthouse.Pos[2] + (*error_model).Lighthouse.Pos[2];
	const FLT x122 = (x116 * x121) + (x111 * x119) + (-1 * x103 * x120);
	const FLT x123 = (x86 * x120) + (x111 * x121) + (-1 * x119 * x116);
	const FLT x124 =
		(2 * ((-1 * x86 * x118) + (x103 * x117))) + x113 + (-1 * (x120 + (2 * ((-1 * x86 * x123) + (x103 * x122)))));
	const FLT x125 = 2 * x124;
	const FLT x126 = 2 * x116;
	const FLT x127 = (-1 * x82 * x100) + (x81 * x97) + (x58 * x111);
	const FLT x128 = (-1 * x104 * x126) + x99 + (x102 * x127);
	const FLT x129 = (x103 * x119) + (-1 * x86 * x121) + (x111 * x120);
	const FLT x130 = x85 * ((x103 * x110) + (x83 * x114) + (-1 * x86 * x115));
	const FLT x131 =
		x115 + (-1 * (x121 + (2 * ((-1 * x116 * x122) + (x86 * x129))))) + (2 * ((-1 * x116 * x117) + (x82 * x130)));
	const FLT x132 = 2 * x131;
	const FLT x133 = (x128 * x132) + (x106 * x125);
	const FLT x134 = 0.523598775598299 + (-1 * (*error_model).BSD1.tilt) + (-1 * (*_x0).BSD1.tilt);
	const FLT x135 = tan(x134);
	const FLT x136 = x124 * x124;
	const FLT x137 = x136 + (x131 * x131);
	const FLT x138 =
		x110 + (-1 * (x119 + (2 * ((-1 * x103 * x129) + (x116 * x123))))) + (2 * ((-1 * x81 * x130) + (x118 * x116)));
	const FLT x139 = 1.0 / 2.0 * x138;
	const FLT x140 = (1. / (x137 * sqrt(x137))) * x135 * x139;
	const FLT x141 = x96 + (x101 * x126) + (-1 * x105 * x127);
	const FLT x142 = (1. / sqrt(x137)) * x135;
	const FLT x143 = (-1 * x142 * x141) + (x133 * x140);
	const FLT x144 = x138 * x138;
	const FLT x145 = 1. / x137;
	const FLT x146 = 1. / sqrt(1 + (-1 * (x135 * x135) * x144 * x145));
	const FLT x147 = 1. / x124;
	const FLT x148 = x131 * (1. / x136);
	const FLT x149 = x136 * x145;
	const FLT x150 = ((x106 * x148) + (-1 * x128 * x147)) * x149;
	const FLT x151 = (-1 * x150) + (x143 * x146);
	const FLT x152 = (*error_model).BSD1.ogeemag + (*_x0).BSD1.ogeemag;
	const FLT x153 = atan2(-1 * x131, x124);
	const FLT x154 = -1 * x138 * x142;
	const FLT x155 = asin(x154) + (-1 * x153) + (-1 * (*error_model).BSD1.ogeephase) + (-1 * (*_x0).BSD1.ogeephase);
	const FLT x156 = x152 * cos(x155);
	const FLT x157 = x137 + x144;
	const FLT x158 = cos(x134);
	const FLT x159 = 1. / x158;
	const FLT x160 = (1. / sqrt(x157)) * x159;
	const FLT x161 = asin(x160 * x138);
	const FLT x162 = 8.0108022e-06 * x161;
	const FLT x163 = -8.0108022e-06 + (-1 * x162);
	const FLT x164 = 0.0028679863 + (x161 * x163);
	const FLT x165 = 5.3685255e-06 + (x161 * x164);
	const FLT x166 = 0.0076069798 + (x161 * x165);
	const FLT x167 = x161 * x161;
	const FLT x168 = x161 * x166;
	const FLT x169 = -8.0108022e-06 + (-1.60216044e-05 * x161);
	const FLT x170 = x164 + (x161 * x169);
	const FLT x171 = x165 + (x161 * x170);
	const FLT x172 = x166 + (x161 * x171);
	const FLT x173 = (x161 * x172) + x168;
	const FLT x174 = (*_x0).BSD1.curve + (-1 * x152 * sin(x155)) + (*error_model).BSD1.curve;
	const FLT x175 = sin(x134);
	const FLT x176 = x174 * x175;
	const FLT x177 = x158 + (x176 * x173);
	const FLT x178 = 1. / x177;
	const FLT x179 = x167 * x166 * x178;
	const FLT x180 = x179 * x156;
	const FLT x181 = x175 * x173;
	const FLT x182 = x181 * x156;
	const FLT x183 = 1. / sqrt(1 + (-1 * x144 * (1. / x157) * (1. / (x158 * x158))));
	const FLT x184 = 2 * x138;
	const FLT x185 = x139 * (1. / (x157 * sqrt(x157))) * x159;
	const FLT x186 = x183 * ((-1 * x185 * (x133 + (x184 * x141))) + (x160 * x141));
	const FLT x187 = x163 * x186;
	const FLT x188 = 2.40324066e-05 * x161;
	const FLT x189 = (x161 * (x187 + (-1 * x162 * x186))) + (x164 * x186);
	const FLT x190 = (x165 * x186) + (x161 * x189);
	const FLT x191 = x167 * x174;
	const FLT x192 = x166 * (1. / (x177 * x177)) * x191;
	const FLT x193 = 2 * x168 * x178 * x174;
	const FLT x194 = x178 * x191;
	const FLT x195 = x154 + (x166 * x194);
	const FLT x196 = 1. / sqrt(1 + (-1 * (x195 * x195)));
	const FLT x197 =
		x150 +
		(-1 * x196 *
		 (x143 + (x190 * x194) + (x186 * x193) + (-1 * x180 * x151) +
		  (-1 * x192 *
		   ((x176 * ((x166 * x186) +
					 (x161 * ((x161 * (x189 + (x161 * (x187 + (-1 * x186 * x188) + (x169 * x186))) + (x170 * x186))) +
							  x190 + (x171 * x186))) +
					 (x161 * x190) + (x172 * x186))) +
			(-1 * x182 * x151)))));
	const FLT x198 = ((*error_model).BSD1.gibmag + (*_x0).BSD1.gibmag) *
					 cos((-1 * asin(x195)) + (*error_model).BSD1.gibpha + x153 + (*_x0).BSD1.gibpha);
	const FLT x199 = (x57 * x91) + (-1 * x95);
	const FLT x200 = x85 * x199;
	const FLT x201 = x45 + (-1 * x48) + (-1 * x42) + (-1 * x39);
	const FLT x202 = 1 + (x94 * x201) + (-2 * (x91 * x91));
	const FLT x203 = x85 * x202;
	const FLT x204 = x93 * x92;
	const FLT x205 = x204 + (-1 * x57 * x201);
	const FLT x206 = (x205 * x111) + (x82 * x200) + (-1 * x84 * x203);
	const FLT x207 = (-1 * x86 * x205) + (x81 * x203) + (x83 * x200);
	const FLT x208 = x202 + (x206 * x126) + (-1 * x207 * x105);
	const FLT x209 = (-1 * x81 * x200) + (x83 * x203) + (x205 * x116);
	const FLT x210 = (-1 * x206 * x102) + x199 + (x209 * x105);
	const FLT x211 = x205 + (-1 * x209 * x126) + (x207 * x102);
	const FLT x212 = (x211 * x132) + (x210 * x125);
	const FLT x213 = x183 * ((-1 * x185 * (x212 + (x208 * x184))) + (x208 * x160));
	const FLT x214 = (-1 * x208 * x142) + (x212 * x140);
	const FLT x215 = ((x210 * x148) + (-1 * x211 * x147)) * x149;
	const FLT x216 = x156 * ((-1 * x215) + (x214 * x146));
	const FLT x217 = x213 * x163;
	const FLT x218 = (x161 * (x217 + (-1 * x213 * x162))) + (x213 * x164);
	const FLT x219 = (x213 * x165) + (x218 * x161);
	const FLT x220 =
		x215 +
		(-1 * x196 *
		 (x214 + (x219 * x194) + (-1 * x216 * x179) + (x213 * x193) +
		  (-1 * x192 *
		   ((x176 *
			 ((x219 * x161) + (x213 * x166) +
			  (x161 * (x219 + (x161 * (x218 + (x161 * ((-1 * x213 * x188) + x217 + (x213 * x169))) + (x213 * x170))) +
					   (x213 * x171))) +
			  (x213 * x172))) +
			(-1 * x216 * x181)))));
	const FLT x221 = (x56 * x94) + (-1 * x204);
	const FLT x222 = (-1 * x89) + (-1 * x88) + (-1 * x90) + x87;
	const FLT x223 = x98 + (-1 * x94 * x222);
	const FLT x224 = x85 * x223;
	const FLT x225 = 1 + (x92 * x222) + (-2 * (x56 * x56));
	const FLT x226 = x85 * x225;
	const FLT x227 = (-1 * x82 * x226) + (x221 * x103) + (x83 * x224);
	const FLT x228 = (x83 * x226) + (x82 * x224) + (-1 * x221 * x116);
	const FLT x229 = x221 + (-1 * x227 * x105) + (x228 * x126);
	const FLT x230 = (-1 * x81 * x224) + (x84 * x226) + (x221 * x111);
	const FLT x231 = x223 + (-1 * x228 * x102) + (x230 * x105);
	const FLT x232 = x225 + (-1 * x230 * x126) + (x227 * x102);
	const FLT x233 = (x232 * x132) + (x231 * x125);
	const FLT x234 = x183 * ((-1 * x185 * (x233 + (x229 * x184))) + (x229 * x160));
	const FLT x235 = (-1 * x229 * x142) + (x233 * x140);
	const FLT x236 = ((x231 * x148) + (-1 * x232 * x147)) * x149;
	const FLT x237 = (-1 * x236) + (x235 * x146);
	const FLT x238 = x234 * x163;
	const FLT x239 = (x161 * (x238 + (-1 * x234 * x162))) + (x234 * x164);
	const FLT x240 = (x234 * x165) + (x239 * x161);
	const FLT x241 =
		x236 +
		(-1 * x196 *
		 ((x240 * x194) +
		  (-1 * x192 *
		   ((x176 *
			 ((x234 * x166) + (x240 * x161) +
			  (x161 * (x240 + (x161 * (x239 + (x161 * ((-1 * x234 * x188) + x238 + (x234 * x169))) + (x234 * x170))) +
					   (x234 * x171))) +
			  (x234 * x172))) +
			(-1 * x237 * x182))) +
		  (x234 * x193) + x235 + (-1 * x237 * x180)));
	cnMatrixOptionalSet(Hx, 0, 0, x197 + (x197 * x198));
	cnMatrixOptionalSet(Hx, 0, 1, x220 + (x220 * x198));
	cnMatrixOptionalSet(Hx, 0, 2, x241 + (x241 * x198));
}

// Full version Jacobian of SurviveJointKalmanErrorModel_LightMeas_y_gen2 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]

static inline void SurviveJointKalmanErrorModel_LightMeas_y_gen2_jac_sensor_pt_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveJointKalmanModel *_x0,
	const SurviveJointKalmanErrorModel *error_model, const FLT *sensor_pt) {
	if (hx != 0) {
		hx->data[0] = SurviveJointKalmanErrorModel_LightMeas_y_gen2(dt, _x0, error_model, sensor_pt);
	}
	if (Hx != 0) {
		SurviveJointKalmanErrorModel_LightMeas_y_gen2_jac_sensor_pt(Hx, dt, _x0, error_model, sensor_pt);
	}
}
