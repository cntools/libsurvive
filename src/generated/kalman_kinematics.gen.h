/// NOTE: This is a generated file; do not edit.
#pragma once
#include <cnkalman/generated_header.h>
// clang-format off
static inline void axisangle2euler(CnMat *out, const FLT *axis_angle) {
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
	cnMatrixOptionalSet(out, 0, 0,
						atan2(2 * ((x16 * axis_angle0) + (x14 * axis_angle1)), 1 + (-2 * ((x12 * x11) + x17))));
	cnMatrixOptionalSet(out, 1, 0, asin(2 * ((x16 * axis_angle1) + (-1 * x14 * axis_angle0))));
	cnMatrixOptionalSet(
		out, 2, 0, atan2(2 * ((x15 * x13) + (x7 * x12 * axis_angle0 * axis_angle1)), 1 + (-2 * (x17 + (x8 * x12)))));
}

// Jacobian of axisangle2euler wrt [axis_angle0, axis_angle1, axis_angle2]
static inline void axisangle2euler_jac_axis_angle(CnMat *Hx, const FLT *axis_angle) {
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
	const FLT x39 = x24 * ((-1 * x38 * axis_angle0) + (-1 * x36) + (-1 * x28 * x26) + x35 + (x25 * axis_angle0) +
						   (x29 * x26) + (-1 * x31) + x33);
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
	const FLT x72 = x24 * ((x25 * axis_angle1) + (x71 * x29) + (-1 * x71 * x28) + (x0 * x68) + (-1 * x69) +
						   (-1 * x0 * x67) + (-1 * x38 * axis_angle1) + x70);
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
	const FLT x89 = x24 * ((x88 * x29) + (-1 * x75) + (-1 * x0 * x43) + (-1 * x88 * x28) + (x0 * x73) +
						   (x25 * axis_angle2) + x74 + (-1 * x38 * axis_angle2));
	const FLT x90 = x62 * axis_angle0;
	const FLT x91 = x85 + (x79 * axis_angle2) + (-1 * x77 * axis_angle2) + (-0.5 * x90);
	const FLT x92 = 2 * x89;
	const FLT x93 = 2.0 * axis_angle2;
	const FLT x94 = (x2 * x59 * axis_angle2) + (-1 * x57 * x93) + (x92 * x11);
	const FLT x95 = x37 * axis_angle1;
	const FLT x96 = x9 * axis_angle0;
	const FLT x97 = x96 * axis_angle2;
	const FLT x98 = 1.0 * x21;
	const FLT x99 =
		2 * (1. / sqrt(1 + (-4 * (((x42 * axis_angle1) + (-1 * x90)) * ((x42 * axis_angle1) + (-1 * x90))))));
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
	cnMatrixOptionalSet(
		Hx, 0, 0,
		x66 * ((-1 * x65 * ((-2.0 * x51 * x20) + (x52 * x51) + x60 + (-4 * x55) + (x53 * x14))) +
			   (x50 * ((x18 * x17) + (-1 * x21) + (-1 * x39 * x23) + x47 + (-1 * x41 * x39) + (-0.5 * x40)))));
	cnMatrixOptionalSet(
		Hx, 0, 1,
		x66 * ((-1 * x65 * (x87 + (x84 * x14) + (-1 * x82 * x21) + (x83 * axis_angle1))) +
			   (x50 * (x81 + (x74 * x16) + (-1 * x75 * x16) + x62 + (-1 * x72 * x23) + (-1 * x76 * axis_angle0)))));
	cnMatrixOptionalSet(Hx, 0, 2,
						x66 * ((-1 * x65 * (x94 + (-1 * x93 * x21) + (x92 * x14) + (x83 * axis_angle2))) +
							   (x50 * (x91 + (-1 * x69 * x16) + (-1 * x89 * x41) + (-1 * x89 * x23) + (x70 * x16)))));
	cnMatrixOptionalSet(
		Hx, 1, 0, x99 * (x81 + (-1 * x95 * x39) + (x97 * x39) + (-1 * x98 * axis_angle2) + (x43 * x17) + (-1 * x62)));
	cnMatrixOptionalSet(Hx, 1, 1,
						x99 * ((x72 * x97) + x45 + (-1 * x57) + (-0.5 * x48) + x42 + (-1 * x46) +
							   (-1 * x76 * axis_angle1) + (x2 * x78)));
	cnMatrixOptionalSet(Hx, 1, 2,
						x99 * (x100 + (-1 * x55) + (x31 * x16) + (-1 * x89 * x95) + (x89 * x97) + (-1 * x33 * x16)));
	cnMatrixOptionalSet(
		Hx, 2, 0,
		x110 * ((-1 * x109 * (x60 + (-1 * x58 * x106) + (x53 * x10) + (x59 * x32))) +
				(x105 * ((-1 * x39 * x102) + x91 + (x98 * axis_angle1) + (-1 * x39 * x101) + (-1 * x67 * x17)))));
	cnMatrixOptionalSet(
		Hx, 2, 1,
		x110 * ((-1 * x109 * (x87 + (x1 * x59 * axis_angle1) + (-1 * x82 * x106) + (x84 * x10))) +
				(x105 * (x100 + (x35 * x16) + x55 + (-1 * x36 * x16) + (-1 * x72 * x102) + (-1 * x76 * axis_angle2)))));
	cnMatrixOptionalSet(
		Hx, 2, 2,
		x110 * ((-1 * x109 * (x94 + (-1 * x88 * x86) + (x92 * x10) + (-4 * x62) + (x88 * x59))) +
				(x105 * (x47 + (-1 * x106) + (x1 * x78) + (-0.5 * x103) + (-1 * x89 * x102) + (-1 * x89 * x101)))));
}

// Full version Jacobian of axisangle2euler wrt [axis_angle0, axis_angle1, axis_angle2]

static inline void axisangle2euler_jac_axis_angle_with_hx(CnMat *Hx, CnMat *hx, const FLT *axis_angle) {
	if (hx != 0) {
		axisangle2euler(hx, axis_angle);
	}
	if (Hx != 0) {
		axisangle2euler_jac_axis_angle(Hx, axis_angle);
	}
}
static inline void GenerateQuatErrorModel(CnMat *out, const FLT *_x1, const FLT *_x0) {
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
static inline void GenerateQuatErrorModel_jac_x1(CnMat *Hx, const FLT *_x1, const FLT *_x0) {
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

static inline void GenerateQuatErrorModel_jac_x1_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x1, const FLT *_x0) {
	if (hx != 0) {
		GenerateQuatErrorModel(hx, _x1, _x0);
	}
	if (Hx != 0) {
		GenerateQuatErrorModel_jac_x1(Hx, _x1, _x0);
	}
}
// Jacobian of GenerateQuatErrorModel wrt [_x00, _x01, _x02, _x03]
static inline void GenerateQuatErrorModel_jac_x0(CnMat *Hx, const FLT *_x1, const FLT *_x0) {
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

static inline void GenerateQuatErrorModel_jac_x0_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x1, const FLT *_x0) {
	if (hx != 0) {
		GenerateQuatErrorModel(hx, _x1, _x0);
	}
	if (Hx != 0) {
		GenerateQuatErrorModel_jac_x0(Hx, _x1, _x0);
	}
}
static inline void GenerateQuatModel(CnMat *out, const FLT *_x0, const FLT *error_state) {
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

// Jacobian of GenerateQuatModel wrt [_x00, _x01, _x02, _x03]
static inline void GenerateQuatModel_jac_x0(CnMat *Hx, const FLT *_x0, const FLT *error_state) {
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

// Full version Jacobian of GenerateQuatModel wrt [_x00, _x01, _x02, _x03]

static inline void GenerateQuatModel_jac_x0_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x0, const FLT *error_state) {
	if (hx != 0) {
		GenerateQuatModel(hx, _x0, error_state);
	}
	if (Hx != 0) {
		GenerateQuatModel_jac_x0(Hx, _x0, error_state);
	}
}
// Jacobian of GenerateQuatModel wrt [error_state0, error_state1, error_state2]
static inline void GenerateQuatModel_jac_error_state(CnMat *Hx, const FLT *_x0, const FLT *error_state) {
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
	const FLT x26 = 1.0 / 2.0 * (1. / (x25 * sqrt(x25)));
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
	cnMatrixOptionalSet(Hx, 0, 0,
						(x58 * _x03) + (-1 * x56) + (x49 * x35) + (x47 * x27) + (-1 * x60 * x46) + x51 + (x53 * x52) +
							(-1 * x54 * x44));
	cnMatrixOptionalSet(Hx, 0, 1,
						(-1 * x60 * x67) + (-1 * x66 * x54) + (x68 * x27) + (x72 * _x03) + (x69 * _x00) + (x73 * x52) +
							(-1 * x62 * x50) + (-1 * x70 * x65));
	cnMatrixOptionalSet(Hx, 0, 2,
						(-1 * x79 * x60) + (-1 * x78 * x54) + (x75 * x49) + (x82 * x27) + x74 + (-1 * x76) +
							(x80 * x77) + (x81 * x52));
	cnMatrixOptionalSet(Hx, 1, 0,
						(-1 * x86 * x24) + (-1 * x52 * x57) + (x50 * x35) + x85 + (-1 * x83 * x27) + (-1 * x70 * x44) +
							(x77 * x53) + x84);
	cnMatrixOptionalSet(Hx, 1, 1,
						(-1 * x71 * x52) + (x62 * x49) + (-1 * x88 * _x01) + (x69 * _x01) + (-1 * x70 * x66) +
							(x65 * x54) + (x73 * x77) + (-1 * x89 * x24));
	cnMatrixOptionalSet(Hx, 1, 2,
						(-1 * x82 * x59) + x93 + (-1 * x92 * x13) + (-1 * x90 * x27) + (x75 * x50) + x91 +
							(-1 * x70 * x78) + (x81 * x77));
	cnMatrixOptionalSet(Hx, 2, 0,
						(-1 * x86 * x18) + (-1 * x83 * x52) + (x54 * x35) + (x44 * x49) + (x58 * _x01) + x76 + x74 +
							(-1 * x77 * x47));
	cnMatrixOptionalSet(Hx, 2, 1,
						(-1 * x89 * x18) + (x69 * _x02) + (-1 * x87 * x52) + (-1 * x65 * x50) + (x70 * x62) +
							(x72 * _x01) + (-1 * x77 * x68) + (x66 * x49));
	cnMatrixOptionalSet(Hx, 2, 2,
						(-1 * x81 * x59) + (-1 * x92 * x21) + (x75 * x54) + (x78 * x49) + x51 + x56 + (x80 * x27) +
							(-1 * x82 * x77));
	cnMatrixOptionalSet(Hx, 3, 0,
						(x70 * x35) + x91 + (-1 * x53 * x27) + (x50 * x44) + (-1 * x83 * x77) + (-1 * x57 * x59) +
							(x94 * x46) + (-1 * x93));
	cnMatrixOptionalSet(Hx, 3, 1,
						(-1 * x71 * x59) + (-1 * x62 * x54) + (x67 * x94) + (x65 * x49) + (-1 * x73 * x27) +
							(x66 * x50) + (x69 * _x03) + (-1 * x88 * _x03));
	cnMatrixOptionalSet(Hx, 3, 2,
						(x82 * x52) + (-1 * x80 * x59) + (-1 * x84) + (-1 * x81 * x27) + (x78 * x50) + (x70 * x75) +
							x85 + (-1 * x77 * x90));
}

// Full version Jacobian of GenerateQuatModel wrt [error_state0, error_state1, error_state2]

static inline void GenerateQuatModel_jac_error_state_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x0,
															 const FLT *error_state) {
	if (hx != 0) {
		GenerateQuatModel(hx, _x0, error_state);
	}
	if (Hx != 0) {
		GenerateQuatModel_jac_error_state(Hx, _x0, error_state);
	}
}
static inline void GenerateQuatErrorModelApprox(CnMat *out, const FLT *_x1, const FLT *_x0) {
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
static inline void GenerateQuatErrorModelApprox_jac_x1(CnMat *Hx, const FLT *_x1, const FLT *_x0) {
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

static inline void GenerateQuatErrorModelApprox_jac_x1_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x1, const FLT *_x0) {
	if (hx != 0) {
		GenerateQuatErrorModelApprox(hx, _x1, _x0);
	}
	if (Hx != 0) {
		GenerateQuatErrorModelApprox_jac_x1(Hx, _x1, _x0);
	}
}
// Jacobian of GenerateQuatErrorModelApprox wrt [_x00, _x01, _x02, _x03]
static inline void GenerateQuatErrorModelApprox_jac_x0(CnMat *Hx, const FLT *_x1, const FLT *_x0) {
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

static inline void GenerateQuatErrorModelApprox_jac_x0_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x1, const FLT *_x0) {
	if (hx != 0) {
		GenerateQuatErrorModelApprox(hx, _x1, _x0);
	}
	if (Hx != 0) {
		GenerateQuatErrorModelApprox_jac_x0(Hx, _x1, _x0);
	}
}
static inline void GenerateQuatModelApprox(CnMat *out, const FLT *_x0, const FLT *error_state) {
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT error_state0 = error_state[0];
	const FLT error_state1 = error_state[1];
	const FLT error_state2 = error_state[2];
	const FLT x0 = 0.5 * _x02;
	const FLT x1 = 0.5 * _x03;
	const FLT x2 = 0.5 * _x01;
	const FLT x3 = 0.5 * _x00;
	cnMatrixOptionalSet(out, 0, 0,
						(-1 * x2 * error_state0) + (-1 * x1 * error_state2) + (-1 * x0 * error_state1) + _x00);
	cnMatrixOptionalSet(out, 1, 0, _x01 + (x3 * error_state0) + (-1 * x1 * error_state1) + (x0 * error_state2));
	cnMatrixOptionalSet(out, 2, 0, (-1 * x2 * error_state2) + (x1 * error_state0) + _x02 + (x3 * error_state1));
	cnMatrixOptionalSet(out, 3, 0, (x3 * error_state2) + (x2 * error_state1) + _x03 + (-1 * x0 * error_state0));
}

// Jacobian of GenerateQuatModelApprox wrt [_x00, _x01, _x02, _x03]
static inline void GenerateQuatModelApprox_jac_x0(CnMat *Hx, const FLT *_x0, const FLT *error_state) {
	const FLT error_state0 = error_state[0];
	const FLT error_state1 = error_state[1];
	const FLT error_state2 = error_state[2];
	const FLT x0 = 0.5 * error_state0;
	const FLT x1 = -1 * x0;
	const FLT x2 = 0.5 * error_state1;
	const FLT x3 = -1 * x2;
	const FLT x4 = 0.5 * error_state2;
	const FLT x5 = -1 * x4;
	cnMatrixOptionalSet(Hx, 0, 0, 1);
	cnMatrixOptionalSet(Hx, 0, 1, x1);
	cnMatrixOptionalSet(Hx, 0, 2, x3);
	cnMatrixOptionalSet(Hx, 0, 3, x5);
	cnMatrixOptionalSet(Hx, 1, 0, x0);
	cnMatrixOptionalSet(Hx, 1, 1, 1);
	cnMatrixOptionalSet(Hx, 1, 2, x4);
	cnMatrixOptionalSet(Hx, 1, 3, x3);
	cnMatrixOptionalSet(Hx, 2, 0, x2);
	cnMatrixOptionalSet(Hx, 2, 1, x5);
	cnMatrixOptionalSet(Hx, 2, 2, 1);
	cnMatrixOptionalSet(Hx, 2, 3, x0);
	cnMatrixOptionalSet(Hx, 3, 0, x4);
	cnMatrixOptionalSet(Hx, 3, 1, x2);
	cnMatrixOptionalSet(Hx, 3, 2, x1);
	cnMatrixOptionalSet(Hx, 3, 3, 1);
}

// Full version Jacobian of GenerateQuatModelApprox wrt [_x00, _x01, _x02, _x03]

static inline void GenerateQuatModelApprox_jac_x0_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x0,
														  const FLT *error_state) {
	if (hx != 0) {
		GenerateQuatModelApprox(hx, _x0, error_state);
	}
	if (Hx != 0) {
		GenerateQuatModelApprox_jac_x0(Hx, _x0, error_state);
	}
}
// Jacobian of GenerateQuatModelApprox wrt [error_state0, error_state1, error_state2]
static inline void GenerateQuatModelApprox_jac_error_state(CnMat *Hx, const FLT *_x0, const FLT *error_state) {
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT x0 = 0.5 * _x01;
	const FLT x1 = -1 * x0;
	const FLT x2 = 0.5 * _x02;
	const FLT x3 = -1 * x2;
	const FLT x4 = 0.5 * _x03;
	const FLT x5 = -1 * x4;
	const FLT x6 = 0.5 * _x00;
	cnMatrixOptionalSet(Hx, 0, 0, x1);
	cnMatrixOptionalSet(Hx, 0, 1, x3);
	cnMatrixOptionalSet(Hx, 0, 2, x5);
	cnMatrixOptionalSet(Hx, 1, 0, x6);
	cnMatrixOptionalSet(Hx, 1, 1, x5);
	cnMatrixOptionalSet(Hx, 1, 2, x2);
	cnMatrixOptionalSet(Hx, 2, 0, x4);
	cnMatrixOptionalSet(Hx, 2, 1, x6);
	cnMatrixOptionalSet(Hx, 2, 2, x1);
	cnMatrixOptionalSet(Hx, 3, 0, x3);
	cnMatrixOptionalSet(Hx, 3, 1, x0);
	cnMatrixOptionalSet(Hx, 3, 2, x6);
}

// Full version Jacobian of GenerateQuatModelApprox wrt [error_state0, error_state1, error_state2]

static inline void GenerateQuatModelApprox_jac_error_state_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x0,
																   const FLT *error_state) {
	if (hx != 0) {
		GenerateQuatModelApprox(hx, _x0, error_state);
	}
	if (Hx != 0) {
		GenerateQuatModelApprox_jac_error_state(Hx, _x0, error_state);
	}
}
static inline void SurviveKalmanModelToErrorModel(SurviveKalmanErrorModel *out, const SurviveKalmanModel *_x1,
												  const SurviveKalmanModel *_x0) {
	const FLT x0 = ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[0]) +
				   (-1 * (*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[3]) + (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[2]);
	const FLT x1 = (-1 * (*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[2]) +
				   (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[3]) + ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[0]);
	const FLT x2 = (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[3]) +
				   ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[0]) + (-1 * (*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[2]);
	const FLT x3 = ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[3]) +
				   ((*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[0]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[2]);
	const FLT x4 = x0 * x0;
	out->Pose.Pos[0] = (*_x1).Pose.Pos[0] + (-1 * (*_x0).Pose.Pos[0]);
	out->Pose.Pos[1] = (*_x1).Pose.Pos[1] + (-1 * (*_x0).Pose.Pos[1]);
	out->Pose.Pos[2] = (*_x1).Pose.Pos[2] + (-1 * (*_x0).Pose.Pos[2]);
	out->Pose.AxisAngleRot[0] = atan2(2 * ((x2 * x3) + (x0 * x1)), 1 + (-2 * ((x2 * x2) + x4)));
	out->Pose.AxisAngleRot[1] = asin(2 * ((x0 * x3) + (-1 * x2 * x1)));
	out->Pose.AxisAngleRot[2] = atan2(2 * ((x1 * x3) + (x0 * x2)), 1 + (-2 * (x4 + (x1 * x1))));
	out->Velocity.Pos[0] = (*_x1).Velocity.Pos[0] + (-1 * (*_x0).Velocity.Pos[0]);
	out->Velocity.Pos[1] = (*_x1).Velocity.Pos[1] + (-1 * (*_x0).Velocity.Pos[1]);
	out->Velocity.Pos[2] = (*_x1).Velocity.Pos[2] + (-1 * (*_x0).Velocity.Pos[2]);
	out->Velocity.AxisAngleRot[0] = (*_x1).Velocity.AxisAngleRot[0] + (-1 * (*_x0).Velocity.AxisAngleRot[0]);
	out->Velocity.AxisAngleRot[1] = (*_x1).Velocity.AxisAngleRot[1] + (-1 * (*_x0).Velocity.AxisAngleRot[1]);
	out->Velocity.AxisAngleRot[2] = (*_x1).Velocity.AxisAngleRot[2] + (-1 * (*_x0).Velocity.AxisAngleRot[2]);
	out->Acc[0] = (*_x1).Acc[0] + (-1 * (*_x0).Acc[0]);
	out->Acc[1] = (*_x1).Acc[1] + (-1 * (*_x0).Acc[1]);
	out->Acc[2] = (*_x1).Acc[2] + (-1 * (*_x0).Acc[2]);
	out->AccScale = (*_x1).AccScale + (-1 * (*_x0).AccScale);
	out->IMUCorrection[0] = (*_x1).IMUCorrection[0] + (-1 * (*_x0).IMUCorrection[0]);
	out->IMUCorrection[1] = (*_x1).IMUCorrection[1] + (-1 * (*_x0).IMUCorrection[1]);
	out->IMUCorrection[2] = (*_x1).IMUCorrection[2] + (-1 * (*_x0).IMUCorrection[2]);
	out->IMUCorrection[3] = (*_x1).IMUCorrection[3] + (-1 * (*_x0).IMUCorrection[3]);
	out->AccBias[0] = (*_x1).AccBias[0] + (-1 * (*_x0).AccBias[0]);
	out->AccBias[1] = (*_x1).AccBias[1] + (-1 * (*_x0).AccBias[1]);
	out->AccBias[2] = (*_x1).AccBias[2] + (-1 * (*_x0).AccBias[2]);
	out->GyroBias[0] = (*_x1).GyroBias[0] + (-1 * (*_x0).GyroBias[0]);
	out->GyroBias[1] = (*_x1).GyroBias[1] + (-1 * (*_x0).GyroBias[1]);
	out->GyroBias[2] = (*_x1).GyroBias[2] + (-1 * (*_x0).GyroBias[2]);
}

// Jacobian of SurviveKalmanModelToErrorModel wrt [(*_x1).AccBias[0], (*_x1).AccBias[1], (*_x1).AccBias[2],
// (*_x1).Acc[0], (*_x1).Acc[1], (*_x1).Acc[2], (*_x1).GyroBias[0], (*_x1).GyroBias[1], (*_x1).GyroBias[2],
// (*_x1).IMUCorrection[0], (*_x1).IMUCorrection[1], (*_x1).IMUCorrection[2], (*_x1).IMUCorrection[3],
// (*_x1).Pose.Pos[0], (*_x1).Pose.Pos[1], (*_x1).Pose.Pos[2], (*_x1).Pose.Rot[0], (*_x1).Pose.Rot[1],
// (*_x1).Pose.Rot[2], (*_x1).Pose.Rot[3], (*_x1).Velocity.AxisAngleRot[0], (*_x1).Velocity.AxisAngleRot[1],
// (*_x1).Velocity.AxisAngleRot[2], (*_x1).Velocity.Pos[0], (*_x1).Velocity.Pos[1], (*_x1).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4f28f40>]
static inline void SurviveKalmanModelToErrorModel_jac_x1(CnMat *Hx, const SurviveKalmanModel *_x1,
														 const SurviveKalmanModel *_x0) {
	const FLT x0 = ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[0]) +
				   (-1 * (*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[3]) + (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[2]);
	const FLT x1 = x0 * (*_x0).Pose.Rot[3];
	const FLT x2 = (-1 * (*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[2]) +
				   (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[3]) + ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[0]);
	const FLT x3 = x2 * (*_x0).Pose.Rot[2];
	const FLT x4 = ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[3]) +
				   ((*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[0]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[2]);
	const FLT x5 = x4 * (*_x0).Pose.Rot[1];
	const FLT x6 = (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[3]) +
				   ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[0]) + (-1 * (*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[2]);
	const FLT x7 = x6 * (*_x0).Pose.Rot[0];
	const FLT x8 = x7 + (-1 * x5);
	const FLT x9 = x0 * x0;
	const FLT x10 = 1 + (-2 * ((x6 * x6) + x9));
	const FLT x11 = 2 * (1. / x10);
	const FLT x12 = x0 * (*_x0).Pose.Rot[2];
	const FLT x13 = 4 * x12;
	const FLT x14 = x6 * (*_x0).Pose.Rot[1];
	const FLT x15 = x10 * x10;
	const FLT x16 = (x4 * x6) + (x0 * x2);
	const FLT x17 = 2 * (1. / x15) * x16;
	const FLT x18 = x15 * (1. / (x15 + (4 * (x16 * x16))));
	const FLT x19 = x2 * (*_x0).Pose.Rot[3];
	const FLT x20 = x14 + (x4 * (*_x0).Pose.Rot[0]);
	const FLT x21 = x20 + x12 + (-1 * x19);
	const FLT x22 = 4 * x1;
	const FLT x23 = x6 * (*_x0).Pose.Rot[2];
	const FLT x24 = x4 * (*_x0).Pose.Rot[3];
	const FLT x25 = x2 * (*_x0).Pose.Rot[0];
	const FLT x26 = x0 * (*_x0).Pose.Rot[1];
	const FLT x27 = (-1 * x26) + x25;
	const FLT x28 = x0 * (*_x0).Pose.Rot[0];
	const FLT x29 = -4 * x28;
	const FLT x30 = x6 * (*_x0).Pose.Rot[3];
	const FLT x31 = x4 * (*_x0).Pose.Rot[2];
	const FLT x32 = x2 * (*_x0).Pose.Rot[1];
	const FLT x33 = x28 + x32;
	const FLT x34 = (-1 * x31) + x33 + x30;
	const FLT x35 = -4 * x26;
	const FLT x36 = 2 * (1. / sqrt(1 + (-4 * (((x0 * x4) + (-1 * x2 * x6)) * ((x0 * x4) + (-1 * x2 * x6))))));
	const FLT x37 = (-1 * x23) + (-1 * x24);
	const FLT x38 = x3 + x1;
	const FLT x39 = 1 + (-2 * (x9 + (x2 * x2)));
	const FLT x40 = 2 * (1. / x39);
	const FLT x41 = x39 * x39;
	const FLT x42 = (x2 * x4) + (x0 * x6);
	const FLT x43 = 2 * (1. / x41) * x42;
	const FLT x44 = x41 * (1. / (x41 + (4 * (x42 * x42))));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						x18 * ((-1 * ((4 * x14) + x13) * x17) + ((x8 + (-1 * x1) + (-1 * x3)) * x11)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						x18 * ((-1 * x17 * ((-4 * x7) + x22)) + (x21 * x11)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x18 * ((-1 * ((-4 * x30) + x29) * x17) + (x11 * (x23 + x27 + x24))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						x18 * ((-1 * ((4 * x23) + x35) * x17) + (x34 * x11)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x34 * x36);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x36 * (x37 + x26 + (-1 * x25)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x36 * x21);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x36 * (x38 + x5 + (-1 * x7)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						((-1 * (x13 + (4 * x19)) * x43) + ((x37 + x27) * x40)) * x44);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						x44 * ((-1 * x43 * (x22 + (-4 * x3))) + (x40 * (x31 + x33 + (-1 * x30)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x44 * ((-1 * (x29 + (4 * x32)) * x43) + (x40 * (x8 + x38))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						x44 * ((-1 * (x35 + (-4 * x25)) * x43) + (x40 * (x20 + x19 + (-1 * x12)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Acc[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Acc[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Acc[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, AccScale) / sizeof(FLT),
						offsetof(SurviveKalmanModel, AccScale) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, IMUCorrection[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, IMUCorrection[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, IMUCorrection[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, IMUCorrection[3]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, AccBias[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, AccBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, AccBias[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, AccBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, AccBias[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, AccBias[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, GyroBias[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, GyroBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, GyroBias[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, GyroBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, GyroBias[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, GyroBias[2]) / sizeof(FLT), 1);
}

// Full version Jacobian of SurviveKalmanModelToErrorModel wrt [(*_x1).AccBias[0], (*_x1).AccBias[1], (*_x1).AccBias[2],
// (*_x1).Acc[0], (*_x1).Acc[1], (*_x1).Acc[2], (*_x1).GyroBias[0], (*_x1).GyroBias[1], (*_x1).GyroBias[2],
// (*_x1).IMUCorrection[0], (*_x1).IMUCorrection[1], (*_x1).IMUCorrection[2], (*_x1).IMUCorrection[3],
// (*_x1).Pose.Pos[0], (*_x1).Pose.Pos[1], (*_x1).Pose.Pos[2], (*_x1).Pose.Rot[0], (*_x1).Pose.Rot[1],
// (*_x1).Pose.Rot[2], (*_x1).Pose.Rot[3], (*_x1).Velocity.AxisAngleRot[0], (*_x1).Velocity.AxisAngleRot[1],
// (*_x1).Velocity.AxisAngleRot[2], (*_x1).Velocity.Pos[0], (*_x1).Velocity.Pos[1], (*_x1).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4f28f40>] Jacobian of SurviveKalmanModelToErrorModel wrt
// [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2], (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2],
// (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1],
// (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3], (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2],
// (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1], (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0],
// (*_x0).Velocity.AxisAngleRot[1], (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1],
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f2ad30>]
static inline void SurviveKalmanModelToErrorModel_jac_x0(CnMat *Hx, const SurviveKalmanModel *_x1,
														 const SurviveKalmanModel *_x0) {
	const FLT x0 = (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[3]) +
				   ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[0]) + (-1 * (*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[2]);
	const FLT x1 = x0 * (*_x1).Pose.Rot[0];
	const FLT x2 = ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[0]) +
				   (-1 * (*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[3]) + (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[2]);
	const FLT x3 = x2 * (*_x1).Pose.Rot[3];
	const FLT x4 = x3 + x1;
	const FLT x5 = ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[3]) +
				   ((*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[0]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[2]);
	const FLT x6 = x5 * (*_x1).Pose.Rot[1];
	const FLT x7 = (-1 * (*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[2]) +
				   (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[3]) + ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[0]);
	const FLT x8 = x7 * (*_x1).Pose.Rot[2];
	const FLT x9 = x8 + x6;
	const FLT x10 = x2 * x2;
	const FLT x11 = 1 + (-2 * ((x0 * x0) + x10));
	const FLT x12 = 2 * (1. / x11);
	const FLT x13 = x2 * (*_x1).Pose.Rot[2];
	const FLT x14 = -4 * x13;
	const FLT x15 = x0 * (*_x1).Pose.Rot[1];
	const FLT x16 = x11 * x11;
	const FLT x17 = (x0 * x5) + (x2 * x7);
	const FLT x18 = 2 * x17 * (1. / x16);
	const FLT x19 = x16 * (1. / (x16 + (4 * (x17 * x17))));
	const FLT x20 = x7 * (*_x1).Pose.Rot[3];
	const FLT x21 = x20 + (-1 * x5 * (*_x1).Pose.Rot[0]);
	const FLT x22 = -4 * x3;
	const FLT x23 = x5 * (*_x1).Pose.Rot[3];
	const FLT x24 = x7 * (*_x1).Pose.Rot[0];
	const FLT x25 = x2 * (*_x1).Pose.Rot[1];
	const FLT x26 = x0 * (*_x1).Pose.Rot[2];
	const FLT x27 = x26 + x25;
	const FLT x28 = x2 * (*_x1).Pose.Rot[0];
	const FLT x29 = 4 * x28;
	const FLT x30 = x0 * (*_x1).Pose.Rot[3];
	const FLT x31 = (-1 * x28) + x30;
	const FLT x32 = x5 * (*_x1).Pose.Rot[2];
	const FLT x33 = x7 * (*_x1).Pose.Rot[1];
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
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						((-1 * ((-4 * x15) + x14) * x18) + ((x9 + x4) * x12)) * x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						x19 * ((-1 * x18 * ((4 * x1) + x22)) + (x12 * (x21 + x15 + (-1 * x13)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x19 * ((-1 * ((4 * x30) + x29) * x18) + (x12 * ((-1 * x23) + x27 + (-1 * x24)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						((-1 * ((-4 * x26) + x35) * x18) + ((x34 + x31) * x12)) * x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x36 * (x34 + x28 + (-1 * x30)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x36 * x37);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x36 * x38);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), (x4 + (-1 * x6) + (-1 * x8)) * x36);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						x44 * ((-1 * (x14 + (-4 * x20)) * x43) + (x40 * x37)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						x44 * ((-1 * x43 * (x22 + (4 * x8))) + (x40 * (x31 + x33 + (-1 * x32)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x44 * ((-1 * (x29 + (-4 * x33)) * x43) + ((x9 + (-1 * x3) + (-1 * x1)) * x40)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						x44 * ((-1 * (x35 + (4 * x24)) * x43) + (x40 * x38)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Acc[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Acc[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Acc[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, AccScale) / sizeof(FLT),
						offsetof(SurviveKalmanModel, AccScale) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, IMUCorrection[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, IMUCorrection[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, IMUCorrection[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, IMUCorrection[3]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, AccBias[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, AccBias[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, AccBias[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, AccBias[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, AccBias[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, AccBias[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, GyroBias[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, GyroBias[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, GyroBias[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, GyroBias[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, GyroBias[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, GyroBias[2]) / sizeof(FLT), -1);
}

// Full version Jacobian of SurviveKalmanModelToErrorModel wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2],
// (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3],
// (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1],
// (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1],
// (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4f2ad30>]
static inline void SurviveKalmanModelAddErrorModel(SurviveKalmanModel *out, const SurviveKalmanModel *_x0,
												   const SurviveKalmanErrorModel *error_state) {
	const FLT x0 = 0.5 * (*error_state).Pose.AxisAngleRot[2];
	const FLT x1 = 0.5 * (*error_state).Pose.AxisAngleRot[1];
	const FLT x2 = 0.5 * (*error_state).Pose.AxisAngleRot[0];
	out->Pose.Pos[0] = (*_x0).Pose.Pos[0] + (*error_state).Pose.Pos[0];
	out->Pose.Pos[1] = (*_x0).Pose.Pos[1] + (*error_state).Pose.Pos[1];
	out->Pose.Pos[2] = (*_x0).Pose.Pos[2] + (*error_state).Pose.Pos[2];
	out->Pose.Rot[0] = (-1 * x0 * (*_x0).Pose.Rot[3]) + (-1 * x2 * (*_x0).Pose.Rot[1]) +
					   (-1 * x1 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0];
	out->Pose.Rot[1] =
		(*_x0).Pose.Rot[1] + (x2 * (*_x0).Pose.Rot[0]) + (-1 * x1 * (*_x0).Pose.Rot[3]) + (x0 * (*_x0).Pose.Rot[2]);
	out->Pose.Rot[2] =
		(-1 * x0 * (*_x0).Pose.Rot[1]) + (x1 * (*_x0).Pose.Rot[0]) + (x2 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	out->Pose.Rot[3] =
		(x1 * (*_x0).Pose.Rot[1]) + (x0 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[3] + (-1 * x2 * (*_x0).Pose.Rot[2]);
	out->Velocity.Pos[0] = (*_x0).Velocity.Pos[0] + (*error_state).Velocity.Pos[0];
	out->Velocity.Pos[1] = (*_x0).Velocity.Pos[1] + (*error_state).Velocity.Pos[1];
	out->Velocity.Pos[2] = (*_x0).Velocity.Pos[2] + (*error_state).Velocity.Pos[2];
	out->Velocity.AxisAngleRot[0] = (*_x0).Velocity.AxisAngleRot[0] + (*error_state).Velocity.AxisAngleRot[0];
	out->Velocity.AxisAngleRot[1] = (*_x0).Velocity.AxisAngleRot[1] + (*error_state).Velocity.AxisAngleRot[1];
	out->Velocity.AxisAngleRot[2] = (*_x0).Velocity.AxisAngleRot[2] + (*error_state).Velocity.AxisAngleRot[2];
	out->Acc[0] = (*_x0).Acc[0] + (*error_state).Acc[0];
	out->Acc[1] = (*_x0).Acc[1] + (*error_state).Acc[1];
	out->Acc[2] = (*_x0).Acc[2] + (*error_state).Acc[2];
	out->AccScale = (*_x0).AccScale + (*error_state).AccScale;
	out->IMUCorrection[0] = (*_x0).IMUCorrection[0] + (*error_state).IMUCorrection[0];
	out->IMUCorrection[1] = (*_x0).IMUCorrection[1] + (*error_state).IMUCorrection[1];
	out->IMUCorrection[2] = (*_x0).IMUCorrection[2] + (*error_state).IMUCorrection[2];
	out->IMUCorrection[3] = (*_x0).IMUCorrection[3] + (*error_state).IMUCorrection[3];
	out->AccBias[0] = (*_x0).AccBias[0] + (*error_state).AccBias[0];
	out->AccBias[1] = (*_x0).AccBias[1] + (*error_state).AccBias[1];
	out->AccBias[2] = (*_x0).AccBias[2] + (*error_state).AccBias[2];
	out->GyroBias[0] = (*_x0).GyroBias[0] + (*error_state).GyroBias[0];
	out->GyroBias[1] = (*_x0).GyroBias[1] + (*error_state).GyroBias[1];
	out->GyroBias[2] = (*_x0).GyroBias[2] + (*error_state).GyroBias[2];
}

// Jacobian of SurviveKalmanModelAddErrorModel wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2],
// (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3],
// (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1],
// (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1],
// (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4f28e80>]
static inline void SurviveKalmanModelAddErrorModel_jac_x0(CnMat *Hx, const SurviveKalmanModel *_x0,
														  const SurviveKalmanErrorModel *error_state) {
	const FLT x0 = 0.5 * (*error_state).Pose.AxisAngleRot[0];
	const FLT x1 = -1 * x0;
	const FLT x2 = 0.5 * (*error_state).Pose.AxisAngleRot[1];
	const FLT x3 = -1 * x2;
	const FLT x4 = 0.5 * (*error_state).Pose.AxisAngleRot[2];
	const FLT x5 = -1 * x4;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x3);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x5);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x0);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x4);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x3);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x2);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x5);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x0);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x4);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x2);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, AccScale) / sizeof(FLT),
						offsetof(SurviveKalmanModel, AccScale) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, IMUCorrection[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, IMUCorrection[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, IMUCorrection[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, IMUCorrection[3]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, AccBias[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, AccBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, AccBias[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, AccBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, AccBias[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, AccBias[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, GyroBias[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, GyroBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, GyroBias[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, GyroBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, GyroBias[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, GyroBias[2]) / sizeof(FLT), 1);
}

// Full version Jacobian of SurviveKalmanModelAddErrorModel wrt [(*_x0).AccBias[0], (*_x0).AccBias[1],
// (*_x0).AccBias[2], (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1],
// (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2],
// (*_x0).IMUCorrection[3], (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0],
// (*_x0).Pose.Rot[1], (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0],
// (*_x0).Velocity.AxisAngleRot[1], (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1],
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f28e80>] Jacobian of
// SurviveKalmanModelAddErrorModel wrt [(*error_state).AccBias[0], (*error_state).AccBias[1], (*error_state).AccBias[2],
// (*error_state).Acc[0], (*error_state).Acc[1], (*error_state).Acc[2], (*error_state).GyroBias[0],
// (*error_state).GyroBias[1], (*error_state).GyroBias[2], (*error_state).IMUCorrection[0],
// (*error_state).IMUCorrection[1], (*error_state).IMUCorrection[2], (*error_state).IMUCorrection[3],
// (*error_state).Pose.AxisAngleRot[0], (*error_state).Pose.AxisAngleRot[1], (*error_state).Pose.AxisAngleRot[2],
// (*error_state).Pose.Pos[0], (*error_state).Pose.Pos[1], (*error_state).Pose.Pos[2],
// (*error_state).Velocity.AxisAngleRot[0], (*error_state).Velocity.AxisAngleRot[1],
// (*error_state).Velocity.AxisAngleRot[2], (*error_state).Velocity.Pos[0], (*error_state).Velocity.Pos[1],
// (*error_state).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f2bc40>]
static inline void SurviveKalmanModelAddErrorModel_jac_error_state(CnMat *Hx, const SurviveKalmanModel *_x0,
																   const SurviveKalmanErrorModel *error_state) {
	const FLT x0 = 0.5 * (*_x0).Pose.Rot[1];
	const FLT x1 = -1 * x0;
	const FLT x2 = 0.5 * (*_x0).Pose.Rot[2];
	const FLT x3 = -1 * x2;
	const FLT x4 = 0.5 * (*_x0).Pose.Rot[3];
	const FLT x5 = -1 * x4;
	const FLT x6 = 0.5 * (*_x0).Pose.Rot[0];
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT), x1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT), x3);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT), x5);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT), x6);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT), x5);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT), x2);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT), x4);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT), x6);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT), x1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT), x3);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT), x0);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT), x6);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Acc[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Acc[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Acc[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, AccScale) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, AccScale) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, IMUCorrection[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, IMUCorrection[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, IMUCorrection[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, IMUCorrection[3]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, AccBias[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, AccBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, AccBias[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, AccBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, AccBias[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, AccBias[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, GyroBias[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, GyroBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, GyroBias[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, GyroBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, GyroBias[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, GyroBias[2]) / sizeof(FLT), 1);
}

// Full version Jacobian of SurviveKalmanModelAddErrorModel wrt [(*error_state).AccBias[0], (*error_state).AccBias[1],
// (*error_state).AccBias[2], (*error_state).Acc[0], (*error_state).Acc[1], (*error_state).Acc[2],
// (*error_state).GyroBias[0], (*error_state).GyroBias[1], (*error_state).GyroBias[2], (*error_state).IMUCorrection[0],
// (*error_state).IMUCorrection[1], (*error_state).IMUCorrection[2], (*error_state).IMUCorrection[3],
// (*error_state).Pose.AxisAngleRot[0], (*error_state).Pose.AxisAngleRot[1], (*error_state).Pose.AxisAngleRot[2],
// (*error_state).Pose.Pos[0], (*error_state).Pose.Pos[1], (*error_state).Pose.Pos[2],
// (*error_state).Velocity.AxisAngleRot[0], (*error_state).Velocity.AxisAngleRot[1],
// (*error_state).Velocity.AxisAngleRot[2], (*error_state).Velocity.Pos[0], (*error_state).Velocity.Pos[1],
// (*error_state).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f2bc40>]
static inline void SurviveKalmanModelPredict(SurviveKalmanModel *out, const FLT t,
											 const SurviveKalmanModel *kalman_model) {
	const FLT x0 = t * (*kalman_model).Acc[0];
	const FLT x1 = 1.0 / 2.0 * fabs(t);
	const FLT x2 = t * (*kalman_model).Acc[1];
	const FLT x3 = t * (*kalman_model).Acc[2];
	const FLT x4 = t * t;
	const FLT x5 = x4 * ((*kalman_model).Velocity.AxisAngleRot[2] * (*kalman_model).Velocity.AxisAngleRot[2]);
	const FLT x6 = x4 * ((*kalman_model).Velocity.AxisAngleRot[0] * (*kalman_model).Velocity.AxisAngleRot[0]);
	const FLT x7 = x4 * ((*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Velocity.AxisAngleRot[1]);
	const FLT x8 = 1e-10 + x7 + x5 + x6;
	const FLT x9 = sqrt(x8);
	const FLT x10 = 0.5 * x9;
	const FLT x11 = sin(x10);
	const FLT x12 = (1. / x8) * (x11 * x11);
	const FLT x13 = cos(x10);
	const FLT x14 = 1. / sqrt((x6 * x12) + (x13 * x13) + (x5 * x12) + (x7 * x12));
	const FLT x15 = t * (1. / x9) * x14 * x11;
	const FLT x16 = x15 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x17 = x15 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x18 = x14 * x13;
	const FLT x19 = x15 * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x20 = x15 * (*kalman_model).Pose.Rot[0];
	out->Pose.Pos[0] = (x0 * x1) + (t * (*kalman_model).Velocity.Pos[0]) + (*kalman_model).Pose.Pos[0];
	out->Pose.Pos[1] = (t * (*kalman_model).Velocity.Pos[1]) + (x2 * x1) + (*kalman_model).Pose.Pos[1];
	out->Pose.Pos[2] = (t * (*kalman_model).Velocity.Pos[2]) + (x1 * x3) + (*kalman_model).Pose.Pos[2];
	out->Pose.Rot[0] = (-1 * x19 * (*kalman_model).Pose.Rot[1]) + (x18 * (*kalman_model).Pose.Rot[0]) +
					   (-1 * x16 * (*kalman_model).Pose.Rot[3]) + (-1 * x17 * (*kalman_model).Pose.Rot[2]);
	out->Pose.Rot[1] = (x20 * (*kalman_model).Velocity.AxisAngleRot[0]) + (x18 * (*kalman_model).Pose.Rot[1]) +
					   (-1 * x16 * (*kalman_model).Pose.Rot[2]) + (x17 * (*kalman_model).Pose.Rot[3]);
	out->Pose.Rot[2] = (-1 * x19 * (*kalman_model).Pose.Rot[3]) + (x18 * (*kalman_model).Pose.Rot[2]) +
					   (x20 * (*kalman_model).Velocity.AxisAngleRot[1]) + (x16 * (*kalman_model).Pose.Rot[1]);
	out->Pose.Rot[3] = (x19 * (*kalman_model).Pose.Rot[2]) + (x18 * (*kalman_model).Pose.Rot[3]) +
					   (x20 * (*kalman_model).Velocity.AxisAngleRot[2]) + (-1 * x17 * (*kalman_model).Pose.Rot[1]);
	out->Velocity.Pos[0] = (*kalman_model).Velocity.Pos[0] + x0;
	out->Velocity.Pos[1] = (*kalman_model).Velocity.Pos[1] + x2;
	out->Velocity.Pos[2] = (*kalman_model).Velocity.Pos[2] + x3;
	out->Velocity.AxisAngleRot[0] = (*kalman_model).Velocity.AxisAngleRot[0];
	out->Velocity.AxisAngleRot[1] = (*kalman_model).Velocity.AxisAngleRot[1];
	out->Velocity.AxisAngleRot[2] = (*kalman_model).Velocity.AxisAngleRot[2];
	out->Acc[0] = (*kalman_model).Acc[0];
	out->Acc[1] = (*kalman_model).Acc[1];
	out->Acc[2] = (*kalman_model).Acc[2];
	out->AccScale = (*kalman_model).AccScale;
	out->IMUCorrection[0] = (*kalman_model).IMUCorrection[0];
	out->IMUCorrection[1] = (*kalman_model).IMUCorrection[1];
	out->IMUCorrection[2] = (*kalman_model).IMUCorrection[2];
	out->IMUCorrection[3] = (*kalman_model).IMUCorrection[3];
	out->AccBias[0] = (*kalman_model).AccBias[0];
	out->AccBias[1] = (*kalman_model).AccBias[1];
	out->AccBias[2] = (*kalman_model).AccBias[2];
	out->GyroBias[0] = (*kalman_model).GyroBias[0];
	out->GyroBias[1] = (*kalman_model).GyroBias[1];
	out->GyroBias[2] = (*kalman_model).GyroBias[2];
}

// Jacobian of SurviveKalmanModelPredict wrt [t]
static inline void SurviveKalmanModelPredict_jac_t(CnMat *Hx, const FLT t, const SurviveKalmanModel *kalman_model) {
	const FLT x0 = 1.0 / 2.0 * t;
	const FLT x1 = x0 * ((t) > 0 ? 1 : -1) /* Note: Maybe not valid for == 0 */;
	const FLT x2 = fabs(t);
	const FLT x3 = 1.0 / 2.0 * x2;
	const FLT x4 = t * t;
	const FLT x5 = (*kalman_model).Velocity.AxisAngleRot[2] * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x6 = x4 * x5;
	const FLT x7 = (*kalman_model).Velocity.AxisAngleRot[0] * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x8 = x4 * x7;
	const FLT x9 = (*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x10 = x4 * x9;
	const FLT x11 = 1e-10 + x10 + x6 + x8;
	const FLT x12 = 1. / x11;
	const FLT x13 = sqrt(x11);
	const FLT x14 = 0.5 * x13;
	const FLT x15 = sin(x14);
	const FLT x16 = x15 * x15;
	const FLT x17 = x12 * x16;
	const FLT x18 = cos(x14);
	const FLT x19 = (x8 * x17) + (x6 * x17) + (x18 * x18) + (x10 * x17);
	const FLT x20 = 1. / sqrt(x19);
	const FLT x21 = x15 * (1. / x13);
	const FLT x22 = x21 * (*kalman_model).Pose.Rot[1];
	const FLT x23 = x22 * x20;
	const FLT x24 = x18 * (*kalman_model).Pose.Rot[2];
	const FLT x25 = 2 * t;
	const FLT x26 = x9 * x25;
	const FLT x27 = x5 * x25;
	const FLT x28 = x7 * x25;
	const FLT x29 = x28 + x26 + x27;
	const FLT x30 = 0.25 * x29;
	const FLT x31 = t * x30 * x12;
	const FLT x32 = x31 * x24;
	const FLT x33 = x32 * x20;
	const FLT x34 = x15 * (1. / (x11 * sqrt(x11)));
	const FLT x35 = x0 * x34 * x29;
	const FLT x36 = x35 * x20;
	const FLT x37 = x36 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x38 = (1. / (x11 * x11)) * x16;
	const FLT x39 = x38 * x29;
	const FLT x40 = 0.5 * x18;
	const FLT x41 = x40 * x29;
	const FLT x42 = x41 * x34;
	const FLT x43 = x29 * x10;
	const FLT x44 =
		(1. / (x19 * sqrt(x19))) * ((x6 * x42) + (x26 * x17) + (-1 * x6 * x39) + (x40 * x43 * x34) + (x27 * x17) +
									(x28 * x17) + (-1 * x43 * x38) + (-1 * x8 * x39) + (x8 * x42) + (-1 * x41 * x21));
	const FLT x45 = x0 * x44;
	const FLT x46 = x45 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x47 = x46 * x21;
	const FLT x48 = x20 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x49 = x48 * (*kalman_model).Pose.Rot[3];
	const FLT x50 = x31 * x18;
	const FLT x51 = x45 * x21;
	const FLT x52 = x51 * (*kalman_model).Pose.Rot[3];
	const FLT x53 = x50 * x20;
	const FLT x54 = x53 * (*kalman_model).Pose.Rot[1];
	const FLT x55 = 1.0 / 2.0 * x44;
	const FLT x56 = x55 * x18;
	const FLT x57 = x48 * x21;
	const FLT x58 = x20 * x21;
	const FLT x59 = x58 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x60 = x58 * (*kalman_model).Pose.Rot[0];
	const FLT x61 = x45 * x22;
	const FLT x62 = x36 * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x63 = (*kalman_model).Velocity.AxisAngleRot[0] * (*kalman_model).Pose.Rot[0];
	const FLT x64 = (*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Pose.Rot[3];
	const FLT x65 = x51 * (*kalman_model).Pose.Rot[2];
	const FLT x66 = x48 * x35;
	const FLT x67 = x58 * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x68 = x50 * x48;
	const FLT x69 = x58 * x30;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT), 0,
						(x3 * (*kalman_model).Acc[0]) + (*kalman_model).Velocity.Pos[0] +
							(x1 * (*kalman_model).Acc[0]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT), 0,
						(x3 * (*kalman_model).Acc[1]) + (*kalman_model).Velocity.Pos[1] +
							(x1 * (*kalman_model).Acc[1]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT), 0,
						(x3 * (*kalman_model).Acc[2]) + (*kalman_model).Velocity.Pos[2] +
							(x1 * (*kalman_model).Acc[2]));
	cnMatrixOptionalSet(
		Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), 0,
		(-1 * x60 * x30) + (x47 * (*kalman_model).Pose.Rot[2]) + (-1 * x57 * (*kalman_model).Pose.Rot[3]) +
			(-1 * x50 * x49) + (x61 * (*kalman_model).Velocity.AxisAngleRot[0]) +
			(-1 * x59 * (*kalman_model).Pose.Rot[2]) + (-1 * x23 * (*kalman_model).Velocity.AxisAngleRot[0]) +
			(-1 * x33 * (*kalman_model).Velocity.AxisAngleRot[1]) + (x52 * (*kalman_model).Velocity.AxisAngleRot[2]) +
			(x62 * (*kalman_model).Pose.Rot[1]) + (-1 * x56 * (*kalman_model).Pose.Rot[0]) + (x49 * x35) +
			(x37 * (*kalman_model).Pose.Rot[2]) + (-1 * x54 * (*kalman_model).Velocity.AxisAngleRot[0]));
	cnMatrixOptionalSet(
		Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), 0,
		(x59 * (*kalman_model).Pose.Rot[3]) + (-1 * x47 * (*kalman_model).Pose.Rot[3]) +
			(-1 * x56 * (*kalman_model).Pose.Rot[1]) + (x66 * (*kalman_model).Pose.Rot[2]) + (x64 * x53) +
			(-1 * x48 * x32) + (x63 * x53) + (x60 * (*kalman_model).Velocity.AxisAngleRot[0]) +
			(-1 * x57 * (*kalman_model).Pose.Rot[2]) + (-1 * x62 * (*kalman_model).Pose.Rot[0]) +
			(x65 * (*kalman_model).Velocity.AxisAngleRot[2]) + (-1 * x30 * x23) + (-1 * x63 * x51) + (-1 * x64 * x36));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), 0,
						(-1 * x53 * (*kalman_model).Velocity.AxisAngleRot[0] * (*kalman_model).Pose.Rot[3]) +
							(-1 * x37 * (*kalman_model).Pose.Rot[0]) + (-1 * x47 * (*kalman_model).Pose.Rot[0]) +
							(x52 * (*kalman_model).Velocity.AxisAngleRot[0]) + (-1 * x55 * x24) +
							(x53 * (*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Pose.Rot[0]) +
							(x60 * (*kalman_model).Velocity.AxisAngleRot[1]) +
							(-1 * x61 * (*kalman_model).Velocity.AxisAngleRot[2]) +
							(x68 * (*kalman_model).Pose.Rot[1]) + (-1 * x69 * (*kalman_model).Pose.Rot[2]) +
							(x48 * x22) + (x62 * (*kalman_model).Pose.Rot[3]) +
							(-1 * x67 * (*kalman_model).Pose.Rot[3]) + (-1 * x66 * (*kalman_model).Pose.Rot[1]));
	cnMatrixOptionalSet(
		Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), 0,
		(x33 * (*kalman_model).Velocity.AxisAngleRot[0]) + (-1 * x56 * (*kalman_model).Pose.Rot[3]) +
			(-1 * x66 * (*kalman_model).Pose.Rot[0]) +
			(-1 * x51 * (*kalman_model).Velocity.AxisAngleRot[2] * (*kalman_model).Pose.Rot[0]) +
			(-1 * x65 * (*kalman_model).Velocity.AxisAngleRot[0]) + (x46 * x22) + (x67 * (*kalman_model).Pose.Rot[2]) +
			(-1 * x54 * (*kalman_model).Velocity.AxisAngleRot[1]) + (-1 * x62 * (*kalman_model).Pose.Rot[2]) +
			(-1 * x69 * (*kalman_model).Pose.Rot[3]) + (x37 * (*kalman_model).Pose.Rot[1]) +
			(x68 * (*kalman_model).Pose.Rot[0]) + (-1 * x23 * (*kalman_model).Velocity.AxisAngleRot[1]) +
			(x57 * (*kalman_model).Pose.Rot[0]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT), 0, (*kalman_model).Acc[0]);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT), 0, (*kalman_model).Acc[1]);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT), 0, (*kalman_model).Acc[2]);
}

// Full version Jacobian of SurviveKalmanModelPredict wrt [t]
// Jacobian of SurviveKalmanModelPredict wrt [(*kalman_model).AccBias[0], (*kalman_model).AccBias[1],
// (*kalman_model).AccBias[2], (*kalman_model).Acc[0], (*kalman_model).Acc[1], (*kalman_model).Acc[2],
// (*kalman_model).GyroBias[0], (*kalman_model).GyroBias[1], (*kalman_model).GyroBias[2],
// (*kalman_model).IMUCorrection[0], (*kalman_model).IMUCorrection[1], (*kalman_model).IMUCorrection[2],
// (*kalman_model).IMUCorrection[3], (*kalman_model).Pose.Pos[0], (*kalman_model).Pose.Pos[1],
// (*kalman_model).Pose.Pos[2], (*kalman_model).Pose.Rot[0], (*kalman_model).Pose.Rot[1], (*kalman_model).Pose.Rot[2],
// (*kalman_model).Pose.Rot[3], (*kalman_model).Velocity.AxisAngleRot[0], (*kalman_model).Velocity.AxisAngleRot[1],
// (*kalman_model).Velocity.AxisAngleRot[2], (*kalman_model).Velocity.Pos[0], (*kalman_model).Velocity.Pos[1],
// (*kalman_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f2a670>]
static inline void SurviveKalmanModelPredict_jac_kalman_model(CnMat *Hx, const FLT t,
															  const SurviveKalmanModel *kalman_model) {
	const FLT x0 = 1.0 / 2.0 * t;
	const FLT x1 = fabs(t) * x0;
	const FLT x2 = t * t;
	const FLT x3 = (*kalman_model).Velocity.AxisAngleRot[2] * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x4 = x2 * x3;
	const FLT x5 = (*kalman_model).Velocity.AxisAngleRot[0] * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x6 = x2 * x5;
	const FLT x7 = (*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x8 = x2 * x7;
	const FLT x9 = 1e-10 + x8 + x4 + x6;
	const FLT x10 = 1. / x9;
	const FLT x11 = sqrt(x9);
	const FLT x12 = 0.5 * x11;
	const FLT x13 = sin(x12);
	const FLT x14 = x13 * x13;
	const FLT x15 = x14 * x10;
	const FLT x16 = cos(x12);
	const FLT x17 = (x6 * x15) + (x16 * x16) + (x4 * x15) + (x8 * x15);
	const FLT x18 = 1. / sqrt(x17);
	const FLT x19 = x18 * x16;
	const FLT x20 = x13 * x18;
	const FLT x21 = 1. / x11;
	const FLT x22 = t * x21;
	const FLT x23 = x22 * x20;
	const FLT x24 = x23 * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x25 = -1 * x24;
	const FLT x26 = x23 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x27 = -1 * x26;
	const FLT x28 = x23 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x29 = -1 * x28;
	const FLT x30 = t * t * t;
	const FLT x31 = 0.5 * x30 * x10 * x19;
	const FLT x32 = x31 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x33 = x32 * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x34 = x33 * (*kalman_model).Pose.Rot[2];
	const FLT x35 = 1. / (x17 * sqrt(x17));
	const FLT x36 = 2 * x15;
	const FLT x37 = x2 * x36;
	const FLT x38 = t * t * t * t;
	const FLT x39 = x38 * ((*kalman_model).Velocity.AxisAngleRot[0] * (*kalman_model).Velocity.AxisAngleRot[0] *
						   (*kalman_model).Velocity.AxisAngleRot[0]);
	const FLT x40 = 2 * (1. / (x9 * x9)) * x14;
	const FLT x41 = 1. / (x9 * sqrt(x9));
	const FLT x42 = 1.0 * x13 * x16;
	const FLT x43 = x41 * x42;
	const FLT x44 = x43 * x38;
	const FLT x45 = x3 * x44;
	const FLT x46 = x40 * x38;
	const FLT x47 = x3 * x46;
	const FLT x48 = x7 * x46;
	const FLT x49 = x7 * x44;
	const FLT x50 = x42 * x21;
	const FLT x51 = x2 * x50;
	const FLT x52 =
		((-1 * x51 * (*kalman_model).Velocity.AxisAngleRot[0]) + (x49 * (*kalman_model).Velocity.AxisAngleRot[0]) +
		 (-1 * x48 * (*kalman_model).Velocity.AxisAngleRot[0]) + (-1 * x40 * x39) +
		 (x37 * (*kalman_model).Velocity.AxisAngleRot[0]) + (x43 * x39) +
		 (x45 * (*kalman_model).Velocity.AxisAngleRot[0]) + (-1 * x47 * (*kalman_model).Velocity.AxisAngleRot[0])) *
		x35;
	const FLT x53 = x0 * x21 * x13;
	const FLT x54 = x53 * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x55 = x54 * x52;
	const FLT x56 = x20 * (*kalman_model).Pose.Rot[0];
	const FLT x57 = 0.5 * x21;
	const FLT x58 = x2 * x57;
	const FLT x59 = x58 * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x60 = x53 * x52;
	const FLT x61 = x60 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x62 = x20 * (*kalman_model).Pose.Rot[2];
	const FLT x63 = x41 * x30;
	const FLT x64 = x63 * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x65 = x64 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x66 = x62 * x65;
	const FLT x67 = 1.0 / 2.0 * x16;
	const FLT x68 = x67 * x52;
	const FLT x69 = x5 * x31;
	const FLT x70 = x23 * (*kalman_model).Pose.Rot[1];
	const FLT x71 = -1 * x70;
	const FLT x72 = (*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Pose.Rot[2];
	const FLT x73 = x5 * x63;
	const FLT x74 = x20 * (*kalman_model).Pose.Rot[1];
	const FLT x75 = x20 * (*kalman_model).Pose.Rot[3];
	const FLT x76 = x64 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x77 = x31 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x78 = x77 * (*kalman_model).Pose.Rot[3];
	const FLT x79 = (-1 * x78 * (*kalman_model).Velocity.AxisAngleRot[0]) + (x75 * x76);
	const FLT x80 = x23 * (*kalman_model).Pose.Rot[2];
	const FLT x81 = -1 * x80;
	const FLT x82 = x7 * x63;
	const FLT x83 = x78 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x84 = x5 * x46;
	const FLT x85 = x5 * x44;
	const FLT x86 = (*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Velocity.AxisAngleRot[1] *
					(*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x87 =
		((x37 * (*kalman_model).Velocity.AxisAngleRot[1]) + (-1 * x86 * x46) +
		 (-1 * x51 * (*kalman_model).Velocity.AxisAngleRot[1]) + (-1 * x84 * (*kalman_model).Velocity.AxisAngleRot[1]) +
		 (x86 * x44) + (x85 * (*kalman_model).Velocity.AxisAngleRot[1]) +
		 (-1 * x47 * (*kalman_model).Velocity.AxisAngleRot[1]) + (x45 * (*kalman_model).Velocity.AxisAngleRot[1])) *
		x35;
	const FLT x88 = x87 * x53;
	const FLT x89 = x88 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x90 = x7 * x31;
	const FLT x91 = x58 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x92 = x75 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x93 = x63 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x94 = x93 * x92;
	const FLT x95 = x87 * x54;
	const FLT x96 = x87 * x67;
	const FLT x97 = x74 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x98 = (*kalman_model).Velocity.AxisAngleRot[0] * (*kalman_model).Pose.Rot[1];
	const FLT x99 = (-1 * x98 * x32) + (x64 * x97);
	const FLT x100 = x74 * x76;
	const FLT x101 = x2 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x102 = (*kalman_model).Velocity.AxisAngleRot[2] * (*kalman_model).Velocity.AxisAngleRot[2] *
					 (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x103 =
		((-1 * x48 * (*kalman_model).Velocity.AxisAngleRot[2]) + (x49 * (*kalman_model).Velocity.AxisAngleRot[2]) +
		 (-1 * x46 * x102) + (-1 * x50 * x101) + (x85 * (*kalman_model).Velocity.AxisAngleRot[2]) +
		 (-1 * x84 * (*kalman_model).Velocity.AxisAngleRot[2]) + (x36 * x101) + (x44 * x102)) *
		x35;
	const FLT x104 = x103 * (*kalman_model).Pose.Rot[3];
	const FLT x105 = x53 * x104;
	const FLT x106 = x53 * x103;
	const FLT x107 = x106 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x108 = x3 * x63;
	const FLT x109 = x23 * (*kalman_model).Pose.Rot[3];
	const FLT x110 = -1 * x109;
	const FLT x111 = x57 * x101;
	const FLT x112 = x67 * x103;
	const FLT x113 = x3 * x31;
	const FLT x114 = x77 * x98;
	const FLT x115 = (*kalman_model).Velocity.AxisAngleRot[2] * (*kalman_model).Pose.Rot[2];
	const FLT x116 = x93 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x117 = (x62 * x116) + (-1 * x32 * x115);
	const FLT x118 = (*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Pose.Rot[3];
	const FLT x119 = x33 * (*kalman_model).Pose.Rot[3];
	const FLT x120 = x76 * x62;
	const FLT x121 = x75 * x65;
	const FLT x122 = x31 * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x123 = x115 * x122;
	const FLT x124 = x56 * x22;
	const FLT x125 = (*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Pose.Rot[0];
	const FLT x126 = (x122 * x125) + (-1 * x65 * x56);
	const FLT x127 = (*kalman_model).Velocity.AxisAngleRot[0] * (*kalman_model).Pose.Rot[0];
	const FLT x128 = (x77 * x127) + (-1 * x76 * x56);
	const FLT x129 = x67 * (*kalman_model).Pose.Rot[2];
	const FLT x130 = (*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Pose.Rot[1];
	const FLT x131 = x77 * x130;
	const FLT x132 = x93 * x97;
	const FLT x133 = x106 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x134 = (x77 * x125) + (-1 * x56 * x116);
	const FLT x135 = x54 * (*kalman_model).Pose.Rot[2];
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT), x1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT), x1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT), x1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x25);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x27);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x29);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x79 + (x72 * x60) + x71 + (-1 * x69 * (*kalman_model).Pose.Rot[1]) +
							(-1 * x68 * (*kalman_model).Pose.Rot[0]) + (x55 * (*kalman_model).Pose.Rot[1]) +
							(-1 * x34) + (x73 * x74) + (-1 * x56 * x59) + (x61 * (*kalman_model).Pose.Rot[3]) + x66);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						(x95 * (*kalman_model).Pose.Rot[1]) + x94 + (x82 * x62) + (-1 * x83) + (x88 * x72) +
							(-1 * x90 * (*kalman_model).Pose.Rot[2]) + x99 + (-1 * x96 * (*kalman_model).Pose.Rot[0]) +
							(-1 * x56 * x91) + x81 + (x89 * (*kalman_model).Pose.Rot[3]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						(x98 * x106) + (-1 * x113 * (*kalman_model).Pose.Rot[3]) + (-1 * x56 * x111) + (-1 * x114) +
							(x105 * (*kalman_model).Velocity.AxisAngleRot[2]) +
							(-1 * x112 * (*kalman_model).Pose.Rot[0]) + (x75 * x108) + x100 + x117 +
							(x107 * (*kalman_model).Pose.Rot[2]) + x110);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x24);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x29);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x26);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x120 + (x69 * (*kalman_model).Pose.Rot[0]) + (-1 * x123) + (-1 * x73 * x56) + x119 +
							(-1 * x121) + (-1 * x55 * (*kalman_model).Pose.Rot[0]) + (-1 * x60 * x118) +
							(-1 * x74 * x59) + x124 + (-1 * x68 * (*kalman_model).Pose.Rot[1]) + (x60 * x115));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x126 + x117 + (-1 * x88 * x118) + x109 + (x88 * x115) +
							(-1 * x95 * (*kalman_model).Pose.Rot[0]) + (-1 * x96 * (*kalman_model).Pose.Rot[1]) +
							(x90 * (*kalman_model).Pose.Rot[3]) + (-1 * x58 * x97) + (-1 * x82 * x75));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x128 + (-1 * x105 * (*kalman_model).Velocity.AxisAngleRot[1]) +
							(-1 * x113 * (*kalman_model).Pose.Rot[2]) + (-1 * x112 * (*kalman_model).Pose.Rot[1]) +
							(-1 * x94) + (-1 * x74 * x111) + (x106 * x115) + x83 + (-1 * x106 * x127) + (x62 * x108) +
							x81);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x26);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x28);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x25);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x126 + (-1 * x62 * x59) + (-1 * x100) + (-1 * x69 * (*kalman_model).Pose.Rot[3]) +
							(-1 * x52 * x129) + x110 + (-1 * x60 * x125) + (x55 * (*kalman_model).Pose.Rot[3]) + x114 +
							(x73 * x75) + (-1 * x61 * (*kalman_model).Pose.Rot[1]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						(x95 * (*kalman_model).Pose.Rot[3]) + x131 + (-1 * x87 * x129) + (-1 * x88 * x125) +
							(-1 * x82 * x56) + (-1 * x119) + x121 + (-1 * x62 * x91) +
							(x90 * (*kalman_model).Pose.Rot[0]) + (-1 * x132) + x124 +
							(-1 * x89 * (*kalman_model).Pose.Rot[1]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x79 + x134 + (-1 * x62 * x111) + (-1 * x133 * (*kalman_model).Pose.Rot[1]) + (x54 * x104) +
							(-1 * x107 * (*kalman_model).Pose.Rot[0]) + (x113 * (*kalman_model).Pose.Rot[1]) + x70 +
							(-1 * x74 * x108) + (-1 * x103 * x129));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x28);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x27);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x24);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x128 + x80 + (x69 * (*kalman_model).Pose.Rot[2]) + (-1 * x68 * (*kalman_model).Pose.Rot[3]) +
							(-1 * x73 * x62) + x99 + (x60 * x130) + (-1 * x52 * x135) +
							(-1 * x61 * (*kalman_model).Pose.Rot[0]) + (-1 * x75 * x59));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x134 + x34 + (x82 * x74) + (-1 * x58 * x92) + x71 + (-1 * x87 * x135) +
							(-1 * x89 * (*kalman_model).Pose.Rot[0]) + (-1 * x96 * (*kalman_model).Pose.Rot[3]) +
							(-1 * x66) + (-1 * x90 * (*kalman_model).Pose.Rot[1]) + (x88 * x130));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						(x113 * (*kalman_model).Pose.Rot[0]) + (-1 * x120) + (-1 * x131) + x123 + (-1 * x67 * x104) +
							(-1 * x133 * (*kalman_model).Pose.Rot[0]) + x132 + (-1 * x75 * x111) +
							(-1 * x106 * (*kalman_model).Velocity.AxisAngleRot[0] * (*kalman_model).Pose.Rot[2]) +
							x124 + (x107 * (*kalman_model).Pose.Rot[1]) + (-1 * x56 * x108));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, AccScale) / sizeof(FLT),
						offsetof(SurviveKalmanModel, AccScale) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, IMUCorrection[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, IMUCorrection[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, IMUCorrection[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, IMUCorrection[3]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, AccBias[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, AccBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, AccBias[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, AccBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, AccBias[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, AccBias[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, GyroBias[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, GyroBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, GyroBias[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, GyroBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, GyroBias[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, GyroBias[2]) / sizeof(FLT), 1);
}

// Full version Jacobian of SurviveKalmanModelPredict wrt [(*kalman_model).AccBias[0], (*kalman_model).AccBias[1],
// (*kalman_model).AccBias[2], (*kalman_model).Acc[0], (*kalman_model).Acc[1], (*kalman_model).Acc[2],
// (*kalman_model).GyroBias[0], (*kalman_model).GyroBias[1], (*kalman_model).GyroBias[2],
// (*kalman_model).IMUCorrection[0], (*kalman_model).IMUCorrection[1], (*kalman_model).IMUCorrection[2],
// (*kalman_model).IMUCorrection[3], (*kalman_model).Pose.Pos[0], (*kalman_model).Pose.Pos[1],
// (*kalman_model).Pose.Pos[2], (*kalman_model).Pose.Rot[0], (*kalman_model).Pose.Rot[1], (*kalman_model).Pose.Rot[2],
// (*kalman_model).Pose.Rot[3], (*kalman_model).Velocity.AxisAngleRot[0], (*kalman_model).Velocity.AxisAngleRot[1],
// (*kalman_model).Velocity.AxisAngleRot[2], (*kalman_model).Velocity.Pos[0], (*kalman_model).Velocity.Pos[1],
// (*kalman_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f2a670>]
static inline void AxisAngleFlip(CnMat *out, const FLT *_x0) {
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT x0 = sqrt(1e-10 + (_x01 * _x01) + (_x02 * _x02) + (_x00 * _x00));
	const FLT x1 = (1. / x0) * (-6.28318530717959 + x0);
	cnMatrixOptionalSet(out, 0, 0, x1 * _x00);
	cnMatrixOptionalSet(out, 1, 0, x1 * _x01);
	cnMatrixOptionalSet(out, 2, 0, x1 * _x02);
}

// Jacobian of AxisAngleFlip wrt [_x00, _x01, _x02]
static inline void AxisAngleFlip_jac_x0(CnMat *Hx, const FLT *_x0) {
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT x0 = _x00 * _x00;
	const FLT x1 = 1e-10 + (_x01 * _x01) + (_x02 * _x02) + x0;
	const FLT x2 = 1. / x1;
	const FLT x3 = sqrt(x1);
	const FLT x4 = -6.28318530717959 + x3;
	const FLT x5 = (1. / (x1 * sqrt(x1))) * x4;
	const FLT x6 = x2 * _x00;
	const FLT x7 = x5 * _x00;
	cnMatrixOptionalSet(Hx, 0, 0, (-1 * x0 * x5) + (x0 * x2) + (x4 * (1. / x3)));
	cnMatrixOptionalSet(Hx, 0, 1, (-1 * x7 * _x01) + (x6 * _x01));
	cnMatrixOptionalSet(Hx, 0, 2, (-1 * x7 * _x02) + (x6 * _x02));
}

// Full version Jacobian of AxisAngleFlip wrt [_x00, _x01, _x02]

static inline void AxisAngleFlip_jac_x0_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x0) {
	if (hx != 0) {
		AxisAngleFlip(hx, _x0);
	}
	if (Hx != 0) {
		AxisAngleFlip_jac_x0(Hx, _x0);
	}
}
static inline void SurviveKalmanModelErrorPredict(SurviveKalmanErrorModel *out, const FLT t,
												  const SurviveKalmanModel *_x0,
												  const SurviveKalmanErrorModel *error_model) {
	const FLT x0 = t * ((*_x0).Acc[0] + (*error_model).Acc[0]);
	const FLT x1 = 1.0 / 2.0 * fabs(t);
	const FLT x2 = t * ((*_x0).Acc[1] + (*error_model).Acc[1]);
	const FLT x3 = t * ((*_x0).Acc[2] + (*error_model).Acc[2]);
	const FLT x4 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x5 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x6 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x7 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x8 =
		(x7 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x5 * (*_x0).Pose.Rot[2]) + (x6 * (*_x0).Pose.Rot[0]);
	const FLT x9 = t * t;
	const FLT x10 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x11 = x9 * (x10 * x10);
	const FLT x12 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x13 = x9 * (x12 * x12);
	const FLT x14 = (x4 * x4) * x9;
	const FLT x15 = 1e-10 + x14 + x11 + x13;
	const FLT x16 = sqrt(x15);
	const FLT x17 = 0.5 * x16;
	const FLT x18 = sin(x17);
	const FLT x19 = (1. / x15) * (x18 * x18);
	const FLT x20 = cos(x17);
	const FLT x21 = 1. / sqrt((x13 * x19) + (x20 * x20) + (x11 * x19) + (x14 * x19));
	const FLT x22 = t * x21 * x18 * (1. / x16);
	const FLT x23 = x8 * x22;
	const FLT x24 =
		(*_x0).Pose.Rot[1] + (x5 * (*_x0).Pose.Rot[0]) + (-1 * x7 * (*_x0).Pose.Rot[3]) + (x6 * (*_x0).Pose.Rot[2]);
	const FLT x25 = x20 * x21;
	const FLT x26 =
		(-1 * x6 * (*_x0).Pose.Rot[1]) + (x7 * (*_x0).Pose.Rot[0]) + (x5 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x27 = x22 * x10;
	const FLT x28 = (-1 * x7 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] + (-1 * x5 * (*_x0).Pose.Rot[1]) +
					(-1 * x6 * (*_x0).Pose.Rot[3]);
	const FLT x29 = x22 * x28;
	const FLT x30 = (-1 * x26 * x27) + (x4 * x23) + (x29 * x12) + (x24 * x25);
	const FLT x31 = x24 * x22;
	const FLT x32 = x22 * x26;
	const FLT x33 = (x32 * x12) + (-1 * x4 * x31) + (x28 * x27) + (x8 * x25);
	const FLT x34 = (-1 * x31 * x12) + (x25 * x28) + (-1 * x23 * x10) + (-1 * x4 * x32);
	const FLT x35 = (x25 * x26) + (-1 * x23 * x12) + (x31 * x10) + (x4 * x29);
	const FLT x36 = (-1 * x35 * (*_x0).Pose.Rot[1]) + (-1 * x34 * (*_x0).Pose.Rot[3]) + (x30 * (*_x0).Pose.Rot[2]) +
					(x33 * (*_x0).Pose.Rot[0]);
	const FLT x37 = (x33 * (*_x0).Pose.Rot[1]) + (x35 * (*_x0).Pose.Rot[0]) + (-1 * x30 * (*_x0).Pose.Rot[3]) +
					(-1 * x34 * (*_x0).Pose.Rot[2]);
	const FLT x38 = (x35 * (*_x0).Pose.Rot[3]) + (-1 * x34 * (*_x0).Pose.Rot[1]) + (x30 * (*_x0).Pose.Rot[0]) +
					(-1 * x33 * (*_x0).Pose.Rot[2]);
	const FLT x39 = (x34 * (*_x0).Pose.Rot[0]) + (x30 * (*_x0).Pose.Rot[1]) + (x33 * (*_x0).Pose.Rot[3]) +
					(x35 * (*_x0).Pose.Rot[2]);
	const FLT x40 = x37 * x37;
	out->Pose.Pos[0] =
		(t * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) + (*error_model).Pose.Pos[0] + (x0 * x1);
	out->Pose.Pos[1] =
		(t * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) + (*error_model).Pose.Pos[1] + (x2 * x1);
	out->Pose.Pos[2] =
		(t * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2])) + (*error_model).Pose.Pos[2] + (x1 * x3);
	out->Pose.AxisAngleRot[0] = atan2(2 * ((x38 * x39) + (x36 * x37)), 1 + (-2 * ((x38 * x38) + x40)));
	out->Pose.AxisAngleRot[1] = asin(2 * ((x37 * x39) + (-1 * x36 * x38)));
	out->Pose.AxisAngleRot[2] = atan2(2 * ((x36 * x39) + (x38 * x37)), 1 + (-2 * (x40 + (x36 * x36))));
	out->Velocity.Pos[0] = x0 + (*error_model).Velocity.Pos[0];
	out->Velocity.Pos[1] = (*error_model).Velocity.Pos[1] + x2;
	out->Velocity.Pos[2] = x3 + (*error_model).Velocity.Pos[2];
	out->Velocity.AxisAngleRot[0] = (*error_model).Velocity.AxisAngleRot[0];
	out->Velocity.AxisAngleRot[1] = (*error_model).Velocity.AxisAngleRot[1];
	out->Velocity.AxisAngleRot[2] = (*error_model).Velocity.AxisAngleRot[2];
	out->Acc[0] = (*error_model).Acc[0];
	out->Acc[1] = (*error_model).Acc[1];
	out->Acc[2] = (*error_model).Acc[2];
	out->AccScale = (*error_model).AccScale;
	out->IMUCorrection[0] = (*error_model).IMUCorrection[0];
	out->IMUCorrection[1] = (*error_model).IMUCorrection[1];
	out->IMUCorrection[2] = (*error_model).IMUCorrection[2];
	out->IMUCorrection[3] = (*error_model).IMUCorrection[3];
	out->AccBias[0] = (*error_model).AccBias[0];
	out->AccBias[1] = (*error_model).AccBias[1];
	out->AccBias[2] = (*error_model).AccBias[2];
	out->GyroBias[0] = (*error_model).GyroBias[0];
	out->GyroBias[1] = (*error_model).GyroBias[1];
	out->GyroBias[2] = (*error_model).GyroBias[2];
}

// Jacobian of SurviveKalmanModelErrorPredict wrt [t]
static inline void SurviveKalmanModelErrorPredict_jac_t(CnMat *Hx, const FLT t, const SurviveKalmanModel *_x0,
														const SurviveKalmanErrorModel *error_model) {
	const FLT x0 = (*_x0).Acc[0] + (*error_model).Acc[0];
	const FLT x1 = 1.0 / 2.0 * t;
	const FLT x2 = x1 * ((t) > 0 ? 1 : -1) /* Note: Maybe not valid for == 0 */;
	const FLT x3 = fabs(t);
	const FLT x4 = 1.0 / 2.0 * x3;
	const FLT x5 = (*_x0).Acc[1] + (*error_model).Acc[1];
	const FLT x6 = (*_x0).Acc[2] + (*error_model).Acc[2];
	const FLT x7 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x8 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x9 = 0.5 * (*_x0).Pose.Rot[1];
	const FLT x10 = (x9 * (*error_model).Pose.AxisAngleRot[1]) + (*_x0).Pose.Rot[3] + (-1 * x7 * (*_x0).Pose.Rot[2]) +
					(x8 * (*_x0).Pose.Rot[0]);
	const FLT x11 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x12 = t * t;
	const FLT x13 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x14 = x13 * x13;
	const FLT x15 = x14 * x12;
	const FLT x16 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x17 = x16 * x16;
	const FLT x18 = x12 * x17;
	const FLT x19 = x11 * x11;
	const FLT x20 = x12 * x19;
	const FLT x21 = 1e-10 + x20 + x15 + x18;
	const FLT x22 = sqrt(x21);
	const FLT x23 = 0.5 * x22;
	const FLT x24 = sin(x23);
	const FLT x25 = x24 * x24;
	const FLT x26 = 1. / x21;
	const FLT x27 = x25 * x26;
	const FLT x28 = cos(x23);
	const FLT x29 = (x27 * x18) + (x28 * x28) + (x27 * x15) + (x20 * x27);
	const FLT x30 = 1. / sqrt(x29);
	const FLT x31 = x24 * (1. / x22);
	const FLT x32 = x30 * x31;
	const FLT x33 = x32 * x11;
	const FLT x34 = x33 * x10;
	const FLT x35 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x36 =
		(*_x0).Pose.Rot[1] + (x7 * (*_x0).Pose.Rot[0]) + (-1 * x35 * (*_x0).Pose.Rot[3]) + (x8 * (*_x0).Pose.Rot[2]);
	const FLT x37 = x30 * x28;
	const FLT x38 = x36 * x37;
	const FLT x39 = (x35 * (*_x0).Pose.Rot[0]) + (x7 * (*_x0).Pose.Rot[3]) +
					(-1 * x9 * (*error_model).Pose.AxisAngleRot[2]) + (*_x0).Pose.Rot[2];
	const FLT x40 = x32 * x13;
	const FLT x41 = x40 * x39;
	const FLT x42 = (-1 * x9 * (*error_model).Pose.AxisAngleRot[0]) + (-1 * x35 * (*_x0).Pose.Rot[2]) +
					(*_x0).Pose.Rot[0] + (-1 * x8 * (*_x0).Pose.Rot[3]);
	const FLT x43 = x32 * x16;
	const FLT x44 = x42 * x43;
	const FLT x45 = (t * x44) + (-1 * t * x41) + (t * x34) + x38;
	const FLT x46 = x40 * x10;
	const FLT x47 = x33 * x39;
	const FLT x48 = x42 * x37;
	const FLT x49 = x43 * x36;
	const FLT x50 = x48 + (-1 * t * x49) + (-1 * t * x46) + (-1 * t * x47);
	const FLT x51 = x40 * x36;
	const FLT x52 = x42 * x33;
	const FLT x53 = x37 * x39;
	const FLT x54 = x43 * x10;
	const FLT x55 = (-1 * t * x54) + x53 + (t * x51) + (t * x52);
	const FLT x56 = x40 * x42;
	const FLT x57 = x37 * x10;
	const FLT x58 = x33 * x36;
	const FLT x59 = x43 * x39;
	const FLT x60 = (t * x59) + (-1 * t * x58) + (t * x56) + x57;
	const FLT x61 = (x60 * (*_x0).Pose.Rot[1]) + (x55 * (*_x0).Pose.Rot[0]) + (-1 * x45 * (*_x0).Pose.Rot[3]) +
					(-1 * x50 * (*_x0).Pose.Rot[2]);
	const FLT x62 = x61 * x61;
	const FLT x63 = (-1 * x50 * (*_x0).Pose.Rot[1]) + (x45 * (*_x0).Pose.Rot[0]) + (x55 * (*_x0).Pose.Rot[3]) +
					(-1 * x60 * (*_x0).Pose.Rot[2]);
	const FLT x64 = 1 + (-2 * ((x63 * x63) + x62));
	const FLT x65 = x64 * x64;
	const FLT x66 = (-1 * x55 * (*_x0).Pose.Rot[1]) + (-1 * x50 * (*_x0).Pose.Rot[3]) + (x45 * (*_x0).Pose.Rot[2]) +
					(x60 * (*_x0).Pose.Rot[0]);
	const FLT x67 = (x45 * (*_x0).Pose.Rot[1]) + (x50 * (*_x0).Pose.Rot[0]) + (x60 * (*_x0).Pose.Rot[3]) +
					(x55 * (*_x0).Pose.Rot[2]);
	const FLT x68 = (x63 * x67) + (x61 * x66);
	const FLT x69 = x1 * x13;
	const FLT x70 = 2 * t;
	const FLT x71 = x70 * x19;
	const FLT x72 = x70 * x14;
	const FLT x73 = x70 * x17;
	const FLT x74 = x73 + x71 + x72;
	const FLT x75 = x24 * (1. / (x21 * sqrt(x21)));
	const FLT x76 = x75 * x74 * x30;
	const FLT x77 = x76 * x10;
	const FLT x78 = 0.25 * x74;
	const FLT x79 = t * x78 * x26;
	const FLT x80 = x79 * x13;
	const FLT x81 = 0.5 * x74 * x28;
	const FLT x82 = x74 * x25 * (1. / (x21 * x21));
	const FLT x83 = x81 * x75;
	const FLT x84 = ((x83 * x15) + (-1 * x82 * x20) + (x83 * x18) + (x73 * x27) + (-1 * x82 * x15) + (x83 * x20) +
					 (x71 * x27) + (-1 * x81 * x31) + (-1 * x82 * x18) + (x72 * x27)) *
					(1. / (x29 * sqrt(x29)));
	const FLT x85 = x84 * x42;
	const FLT x86 = 1.0 / 2.0 * x28;
	const FLT x87 = x1 * x16;
	const FLT x88 = x87 * x36;
	const FLT x89 = x84 * x31;
	const FLT x90 = x89 * x10;
	const FLT x91 = x1 * x11;
	const FLT x92 = x89 * x39;
	const FLT x93 = x79 * x11;
	const FLT x94 = x78 * x32;
	const FLT x95 = x79 * x16;
	const FLT x96 = x76 * x39;
	const FLT x97 = (-1 * x85 * x86) + (-1 * x95 * x38) + (-1 * x46) + (-1 * x53 * x93) + (x91 * x96) +
					(-1 * x80 * x57) + (-1 * x49) + (-1 * x94 * x42) + (x88 * x76) + (x88 * x89) + (x77 * x69) +
					(x69 * x90) + (x92 * x91) + (-1 * x47);
	const FLT x98 = x84 * x86;
	const FLT x99 = x85 * x31;
	const FLT x100 = x79 * x57;
	const FLT x101 = x76 * x42;
	const FLT x102 = (-1 * x94 * x36) + (x69 * x92) + (-1 * x87 * x99) + (x95 * x48) + (-1 * x41) + (-1 * x91 * x90) +
					 (-1 * x98 * x36) + x44 + (x11 * x100) + (-1 * x80 * x53) + x34 + (x69 * x96) + (-1 * x87 * x101) +
					 (-1 * x77 * x91);
	const FLT x103 = x69 * x36;
	const FLT x104 = (x87 * x77) + (-1 * x98 * x39) + (-1 * x76 * x103) + x52 + (-1 * x94 * x39) + x51 +
					 (-1 * x16 * x100) + (-1 * x91 * x99) + (-1 * x91 * x101) + (x87 * x90) + (x80 * x38) + (-1 * x54) +
					 (x93 * x48) + (-1 * x89 * x103);
	const FLT x105 = x91 * x36;
	const FLT x106 = (-1 * x93 * x38) + (x76 * x105) + (-1 * x58) + (x80 * x48) + (x53 * x95) + (-1 * x94 * x10) +
					 (-1 * x87 * x92) + (x89 * x105) + (-1 * x69 * x99) + (-1 * x87 * x96) + (-1 * x98 * x10) + x59 +
					 x56 + (-1 * x69 * x101);
	const FLT x107 = (-1 * x106 * (*_x0).Pose.Rot[2]) + (x104 * (*_x0).Pose.Rot[3]) + (-1 * x97 * (*_x0).Pose.Rot[1]) +
					 (x102 * (*_x0).Pose.Rot[0]);
	const FLT x108 = (x97 * (*_x0).Pose.Rot[0]) + (x104 * (*_x0).Pose.Rot[2]) + (x102 * (*_x0).Pose.Rot[1]) +
					 (x106 * (*_x0).Pose.Rot[3]);
	const FLT x109 = (-1 * x97 * (*_x0).Pose.Rot[2]) + (-1 * x102 * (*_x0).Pose.Rot[3]) + (x106 * (*_x0).Pose.Rot[1]) +
					 (x104 * (*_x0).Pose.Rot[0]);
	const FLT x110 = (x106 * (*_x0).Pose.Rot[0]) + (x102 * (*_x0).Pose.Rot[2]) + (-1 * x104 * (*_x0).Pose.Rot[1]) +
					 (-1 * x97 * (*_x0).Pose.Rot[3]);
	const FLT x111 = -4 * x61 * x109;
	const FLT x112 = 1 + (-2 * (x62 + (x66 * x66)));
	const FLT x113 = x112 * x112;
	const FLT x114 = (x67 * x66) + (x63 * x61);
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT), 0,
						(x0 * x4) + (*error_model).Velocity.Pos[0] + (*_x0).Velocity.Pos[0] + (x0 * x2));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT), 0,
						(x4 * x5) + (x2 * x5) + (*error_model).Velocity.Pos[1] + (*_x0).Velocity.Pos[1]);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT), 0,
						(x2 * x6) + (*error_model).Velocity.Pos[2] + (x4 * x6) + (*_x0).Velocity.Pos[2]);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT), 0,
						x65 * (1. / (x65 + (4 * (x68 * x68)))) *
							((-2 * x68 * (1. / x65) * ((-4 * x63 * x107) + x111)) +
							 (2 * ((x67 * x107) + (x61 * x110) + (x66 * x109) + (x63 * x108)) * (1. / x64))));
	cnMatrixOptionalSet(
		Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT), 0,
		2 * ((-1 * x63 * x110) + (-1 * x66 * x107) + (x61 * x108) + (x67 * x109)) *
			(1. / sqrt(1 + (-4 * (((x61 * x67) + (-1 * x63 * x66)) * ((x61 * x67) + (-1 * x63 * x66)))))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT), 0,
						x113 * (1. / (x113 + (4 * (x114 * x114)))) *
							((-2 * (1. / x113) * x114 * (x111 + (-4 * x66 * x110))) +
							 (2 * ((x61 * x107) + (x63 * x109) + (x67 * x110) + (x66 * x108)) * (1. / x112))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[0]) / sizeof(FLT), 0, x0);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[1]) / sizeof(FLT), 0, x5);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[2]) / sizeof(FLT), 0, x6);
}

// Full version Jacobian of SurviveKalmanModelErrorPredict wrt [t]
// Jacobian of SurviveKalmanModelErrorPredict wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2],
// (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3],
// (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1],
// (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1],
// (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4f44220>]
static inline void SurviveKalmanModelErrorPredict_jac_x0(CnMat *Hx, const FLT t, const SurviveKalmanModel *_x0,
														 const SurviveKalmanErrorModel *error_model) {
	const FLT x0 = 1.0 / 2.0 * t;
	const FLT x1 = fabs(t) * x0;
	const FLT x2 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x3 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x4 = t * t;
	const FLT x5 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x6 = x5 * x5;
	const FLT x7 = x4 * x6;
	const FLT x8 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x9 = x8 * x8;
	const FLT x10 = x4 * x9;
	const FLT x11 = x3 * x3;
	const FLT x12 = x4 * x11;
	const FLT x13 = 1e-10 + x12 + x7 + x10;
	const FLT x14 = sqrt(x13);
	const FLT x15 = 1. / x14;
	const FLT x16 = x3 * x15;
	const FLT x17 = 0.5 * x14;
	const FLT x18 = sin(x17);
	const FLT x19 = x18 * x18;
	const FLT x20 = 1. / x13;
	const FLT x21 = x20 * x19;
	const FLT x22 = cos(x17);
	const FLT x23 = (x21 * x10) + (x22 * x22) + (x7 * x21) + (x21 * x12);
	const FLT x24 = 1. / sqrt(x23);
	const FLT x25 = x24 * x18;
	const FLT x26 = t * x25;
	const FLT x27 = x26 * x16;
	const FLT x28 = x2 * x27;
	const FLT x29 = x24 * x22;
	const FLT x30 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x31 = x30 * x29;
	const FLT x32 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x33 = x26 * x15;
	const FLT x34 = x5 * x33;
	const FLT x35 = -1 * x32 * x34;
	const FLT x36 = x8 * x33;
	const FLT x37 = x36 + x35;
	const FLT x38 = x37 + x28 + x31;
	const FLT x39 = x32 * x29;
	const FLT x40 = x30 * x34;
	const FLT x41 = -1 * x2 * x36;
	const FLT x42 = x27 + x41;
	const FLT x43 = x42 + x39 + x40;
	const FLT x44 = x2 * x34;
	const FLT x45 = -1 * x44;
	const FLT x46 = x32 * x27;
	const FLT x47 = -1 * x46;
	const FLT x48 = x30 * x36;
	const FLT x49 = x29 + (-1 * x48);
	const FLT x50 = x49 + x45 + x47;
	const FLT x51 = x32 * x36;
	const FLT x52 = x2 * x29;
	const FLT x53 = -1 * x30 * x27;
	const FLT x54 = x34 + x53;
	const FLT x55 = x54 + x51 + x52;
	const FLT x56 =
		(x32 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x30 * (*_x0).Pose.Rot[2]) + (x2 * (*_x0).Pose.Rot[0]);
	const FLT x57 = x56 * x34;
	const FLT x58 =
		(-1 * x2 * (*_x0).Pose.Rot[1]) + (x32 * (*_x0).Pose.Rot[0]) + (x30 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x59 = x58 * x27;
	const FLT x60 = (-1 * x30 * (*_x0).Pose.Rot[1]) + (-1 * x32 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x2 * (*_x0).Pose.Rot[3]);
	const FLT x61 = x60 * x29;
	const FLT x62 =
		(*_x0).Pose.Rot[1] + (x30 * (*_x0).Pose.Rot[0]) + (-1 * x32 * (*_x0).Pose.Rot[3]) + (x2 * (*_x0).Pose.Rot[2]);
	const FLT x63 = x62 * x36;
	const FLT x64 = x61 + (-1 * x63) + (-1 * x57) + (-1 * x59);
	const FLT x65 = (x55 * (*_x0).Pose.Rot[3]) + (x50 * (*_x0).Pose.Rot[0]) + x64 + (x38 * (*_x0).Pose.Rot[1]) +
					(x43 * (*_x0).Pose.Rot[2]);
	const FLT x66 = x62 * x34;
	const FLT x67 = x60 * x27;
	const FLT x68 = x58 * x29;
	const FLT x69 = x56 * x36;
	const FLT x70 = (-1 * x69) + x68 + x66 + x67;
	const FLT x71 = x60 * x34;
	const FLT x72 = x56 * x29;
	const FLT x73 = x62 * x27;
	const FLT x74 = x58 * x36;
	const FLT x75 = x74 + (-1 * x73) + x71 + x72;
	const FLT x76 = x56 * x27;
	const FLT x77 = x62 * x29;
	const FLT x78 = x58 * x34;
	const FLT x79 = x60 * x36;
	const FLT x80 = x79 + x76 + (-1 * x78) + x77;
	const FLT x81 = (-1 * x64 * (*_x0).Pose.Rot[1]) + (x80 * (*_x0).Pose.Rot[0]) + (x70 * (*_x0).Pose.Rot[3]) +
					(-1 * x75 * (*_x0).Pose.Rot[2]);
	const FLT x82 = (x80 * (*_x0).Pose.Rot[1]) + (x64 * (*_x0).Pose.Rot[0]) + (x75 * (*_x0).Pose.Rot[3]) +
					(x70 * (*_x0).Pose.Rot[2]);
	const FLT x83 = x80 + (x38 * (*_x0).Pose.Rot[0]) + (x43 * (*_x0).Pose.Rot[3]) + (-1 * x50 * (*_x0).Pose.Rot[1]) +
					(-1 * x55 * (*_x0).Pose.Rot[2]);
	const FLT x84 = (-1 * x38 * (*_x0).Pose.Rot[3]) + (x55 * (*_x0).Pose.Rot[1]) + x70 + (x43 * (*_x0).Pose.Rot[0]) +
					(-1 * x50 * (*_x0).Pose.Rot[2]);
	const FLT x85 = (-1 * x70 * (*_x0).Pose.Rot[1]) + (-1 * x64 * (*_x0).Pose.Rot[3]) + (x80 * (*_x0).Pose.Rot[2]) +
					(x75 * (*_x0).Pose.Rot[0]);
	const FLT x86 = (x70 * (*_x0).Pose.Rot[0]) + (-1 * x80 * (*_x0).Pose.Rot[3]) + (x75 * (*_x0).Pose.Rot[1]) +
					(-1 * x64 * (*_x0).Pose.Rot[2]);
	const FLT x87 = x75 + (-1 * x50 * (*_x0).Pose.Rot[3]) + (x55 * (*_x0).Pose.Rot[0]) +
					(-1 * x43 * (*_x0).Pose.Rot[1]) + (x38 * (*_x0).Pose.Rot[2]);
	const FLT x88 = x86 * x86;
	const FLT x89 = 1 + (-2 * ((x81 * x81) + x88));
	const FLT x90 = 2 * (1. / x89);
	const FLT x91 = 4 * x86;
	const FLT x92 = -1 * x84 * x91;
	const FLT x93 = 4 * x81;
	const FLT x94 = x89 * x89;
	const FLT x95 = (x81 * x82) + (x85 * x86);
	const FLT x96 = 2 * x95 * (1. / x94);
	const FLT x97 = x94 * (1. / (x94 + (4 * (x95 * x95))));
	const FLT x98 = -1 * x40;
	const FLT x99 = x41 + (-1 * x27);
	const FLT x100 = x99 + x98 + x39;
	const FLT x101 = -1 * x52;
	const FLT x102 = -1 * x51;
	const FLT x103 = x54 + x101 + x102;
	const FLT x104 = x49 + x44 + x46;
	const FLT x105 = -1 * x31;
	const FLT x106 = x35 + (-1 * x36);
	const FLT x107 = x106 + x105 + x28;
	const FLT x108 = (-1 * x61) + x59 + x63 + x57;
	const FLT x109 = x108 + (x104 * (*_x0).Pose.Rot[0]) + (-1 * x100 * (*_x0).Pose.Rot[2]) +
					 (-1 * x107 * (*_x0).Pose.Rot[1]) + (x103 * (*_x0).Pose.Rot[3]);
	const FLT x110 = (x107 * (*_x0).Pose.Rot[0]) + x80 + (x100 * (*_x0).Pose.Rot[3]) + (x104 * (*_x0).Pose.Rot[1]) +
					 (x103 * (*_x0).Pose.Rot[2]);
	const FLT x111 = x75 + (-1 * x107 * (*_x0).Pose.Rot[2]) + (x103 * (*_x0).Pose.Rot[0]) +
					 (x100 * (*_x0).Pose.Rot[1]) + (-1 * x104 * (*_x0).Pose.Rot[3]);
	const FLT x112 = x69 + (x104 * (*_x0).Pose.Rot[2]) + (x100 * (*_x0).Pose.Rot[0]) + (-1 * x67) +
					 (-1 * x103 * (*_x0).Pose.Rot[1]) + (-1 * x68) + (-1 * x66) + (-1 * x107 * (*_x0).Pose.Rot[3]);
	const FLT x113 = -1 * x91 * x111;
	const FLT x114 = x53 + (-1 * x34);
	const FLT x115 = x114 + x102 + x52;
	const FLT x116 = -1 * x39;
	const FLT x117 = x99 + x116 + x40;
	const FLT x118 = x48 + x29;
	const FLT x119 = x118 + x47 + x44;
	const FLT x120 = -1 * x28;
	const FLT x121 = x37 + x105 + x120;
	const FLT x122 = (-1 * x72) + (-1 * x121 * (*_x0).Pose.Rot[2]) + (-1 * x71) + (-1 * x117 * (*_x0).Pose.Rot[1]) +
					 (x115 * (*_x0).Pose.Rot[0]) + (x119 * (*_x0).Pose.Rot[3]) + (-1 * x74) + x73;
	const FLT x123 = x70 + (x119 * (*_x0).Pose.Rot[2]) + (x117 * (*_x0).Pose.Rot[0]) + (x115 * (*_x0).Pose.Rot[1]) +
					 (x121 * (*_x0).Pose.Rot[3]);
	const FLT x124 = (-1 * x115 * (*_x0).Pose.Rot[3]) + (x119 * (*_x0).Pose.Rot[0]) + (x121 * (*_x0).Pose.Rot[1]) +
					 x108 + (-1 * x117 * (*_x0).Pose.Rot[2]);
	const FLT x125 = x80 + (x115 * (*_x0).Pose.Rot[2]) + (-1 * x119 * (*_x0).Pose.Rot[1]) +
					 (-1 * x117 * (*_x0).Pose.Rot[3]) + (x121 * (*_x0).Pose.Rot[0]);
	const FLT x126 = -1 * x91 * x124;
	const FLT x127 = x98 + x42 + x116;
	const FLT x128 = x118 + x45 + x46;
	const FLT x129 = x106 + x31 + x120;
	const FLT x130 = x114 + x51 + x101;
	const FLT x131 = x70 + (-1 * x130 * (*_x0).Pose.Rot[1]) + (x127 * (*_x0).Pose.Rot[0]) +
					 (x129 * (*_x0).Pose.Rot[3]) + (-1 * x128 * (*_x0).Pose.Rot[2]);
	const FLT x132 = (-1 * x76) + (-1 * x77) + (-1 * x127 * (*_x0).Pose.Rot[3]) + (x128 * (*_x0).Pose.Rot[1]) +
					 (-1 * x79) + x78 + (-1 * x130 * (*_x0).Pose.Rot[2]) + (x129 * (*_x0).Pose.Rot[0]);
	const FLT x133 = (x127 * (*_x0).Pose.Rot[1]) + x75 + (x128 * (*_x0).Pose.Rot[3]) + (x130 * (*_x0).Pose.Rot[0]) +
					 (x129 * (*_x0).Pose.Rot[2]);
	const FLT x134 = x108 + (-1 * x130 * (*_x0).Pose.Rot[3]) + (x128 * (*_x0).Pose.Rot[0]) +
					 (-1 * x129 * (*_x0).Pose.Rot[1]) + (x127 * (*_x0).Pose.Rot[2]);
	const FLT x135 = -1 * x91 * x132;
	const FLT x136 = x62 * x33;
	const FLT x137 = -1 * x136;
	const FLT x138 = x8 * x8 * x8;
	const FLT x139 = t * t * t * t;
	const FLT x140 = 1. / (x13 * sqrt(x13));
	const FLT x141 = 1.0 * x22 * x18;
	const FLT x142 = x140 * x141;
	const FLT x143 = x139 * x142;
	const FLT x144 = (1. / (x13 * x13)) * x19;
	const FLT x145 = 2 * x144;
	const FLT x146 = x139 * x145;
	const FLT x147 = x6 * x139;
	const FLT x148 = x8 * x141;
	const FLT x149 = x140 * x148;
	const FLT x150 = 2 * x8;
	const FLT x151 = x4 * x21;
	const FLT x152 = x11 * x139;
	const FLT x153 = x144 * x150;
	const FLT x154 = x4 * x15;
	const FLT x155 = (-1 * x152 * x153) + (-1 * x147 * x153) + (-1 * x138 * x146) + (-1 * x148 * x154) + (x147 * x149) +
					 (x150 * x151) + (x138 * x143) + (x149 * x152);
	const FLT x156 = 1. / (x23 * sqrt(x23));
	const FLT x157 = x56 * x156;
	const FLT x158 = x155 * x157;
	const FLT x159 = x0 * x18;
	const FLT x160 = x15 * x159;
	const FLT x161 = x5 * x160;
	const FLT x162 = 1.0 / 2.0 * x22;
	const FLT x163 = x162 * x156;
	const FLT x164 = x60 * x163;
	const FLT x165 = 0.5 * x20;
	const FLT x166 = x68 * x165;
	const FLT x167 = t * t * t;
	const FLT x168 = x3 * x8;
	const FLT x169 = x168 * x167;
	const FLT x170 = x169 * x166;
	const FLT x171 = x58 * x25;
	const FLT x172 = x171 * x140;
	const FLT x173 = x167 * x172;
	const FLT x174 = x168 * x173;
	const FLT x175 = x16 * x159;
	const FLT x176 = x155 * x156;
	const FLT x177 = x58 * x176;
	const FLT x178 = 0.5 * x154;
	const FLT x179 = x8 * x178;
	const FLT x180 = x60 * x25;
	const FLT x181 = x9 * x167;
	const FLT x182 = x25 * x140;
	const FLT x183 = x62 * x182;
	const FLT x184 = x8 * x160;
	const FLT x185 = x62 * x184;
	const FLT x186 = x77 * x165;
	const FLT x187 = x56 * x182;
	const FLT x188 = x5 * x8;
	const FLT x189 = x167 * x188;
	const FLT x190 = x72 * x165;
	const FLT x191 = (-1 * x189 * x190) + (x187 * x189);
	const FLT x192 = (-1 * x181 * x186) + x191 + (x176 * x185) + (x181 * x183) + (-1 * x179 * x180) + x137 +
					 (x177 * x175) + (-1 * x164 * x155) + (x161 * x158) + (-1 * x170) + x174;
	const FLT x193 = x189 * x183;
	const FLT x194 = x62 * x161;
	const FLT x195 = x186 * x189;
	const FLT x196 = x56 * x33;
	const FLT x197 = -1 * x196;
	const FLT x198 = x60 * x176;
	const FLT x199 = x58 * x163;
	const FLT x200 = x171 * x178;
	const FLT x201 = x61 * x165;
	const FLT x202 = x60 * x182;
	const FLT x203 = (-1 * x202 * x169) + (x201 * x169);
	const FLT x204 = (x181 * x187) + (x184 * x158) + (-1 * x8 * x200) + (-1 * x199 * x155) + (-1 * x176 * x194) +
					 (-1 * x181 * x190) + (-1 * x193) + x195 + x203 + x197 + (-1 * x175 * x198);
	const FLT x205 = x62 * x25;
	const FLT x206 = x169 * x190;
	const FLT x207 = x62 * x163;
	const FLT x208 = x169 * x187;
	const FLT x209 = x60 * x33;
	const FLT x210 = x173 * x188;
	const FLT x211 = x166 * x189;
	const FLT x212 = (x201 * x181) + (-1 * x208) + (-1 * x202 * x181) + x210 + x206 + (x161 * x177) +
					 (-1 * x205 * x179) + x209 + (-1 * x175 * x158) + (-1 * x207 * x155) + (-1 * x184 * x198) +
					 (-1 * x211);
	const FLT x213 = x56 * x25;
	const FLT x214 = x58 * x33;
	const FLT x215 = x60 * x161;
	const FLT x216 = x62 * x175;
	const FLT x217 = x162 * x157;
	const FLT x218 = (-1 * x169 * x186) + (x169 * x183);
	const FLT x219 = (x201 * x189) + (-1 * x202 * x189);
	const FLT x220 = x218 + (x166 * x181) + (-1 * x9 * x173) + (-1 * x177 * x184) + x214 + (-1 * x213 * x179) +
					 (-1 * x215 * x176) + x219 + (-1 * x217 * x155) + (x216 * x176);
	const FLT x221 = (x212 * (*_x0).Pose.Rot[0]) + (-1 * x192 * (*_x0).Pose.Rot[1]) + (-1 * x220 * (*_x0).Pose.Rot[2]) +
					 (x204 * (*_x0).Pose.Rot[3]);
	const FLT x222 = (x204 * (*_x0).Pose.Rot[2]) + (x220 * (*_x0).Pose.Rot[3]) + (x212 * (*_x0).Pose.Rot[1]) +
					 (x192 * (*_x0).Pose.Rot[0]);
	const FLT x223 = (-1 * x192 * (*_x0).Pose.Rot[2]) + (x220 * (*_x0).Pose.Rot[1]) + (-1 * x212 * (*_x0).Pose.Rot[3]) +
					 (x204 * (*_x0).Pose.Rot[0]);
	const FLT x224 = (x220 * (*_x0).Pose.Rot[0]) + (-1 * x204 * (*_x0).Pose.Rot[1]) + (-1 * x192 * (*_x0).Pose.Rot[3]) +
					 (x212 * (*_x0).Pose.Rot[2]);
	const FLT x225 = -1 * x91 * x223;
	const FLT x226 = x4 * x16;
	const FLT x227 = x3 * x146;
	const FLT x228 = 2 * x151;
	const FLT x229 = (x3 * x3 * x3) * x139;
	const FLT x230 = (-1 * x229 * x145) + (-1 * x226 * x141) + (x229 * x142) + (x3 * x9 * x143) + (-1 * x9 * x227) +
					 (x3 * x142 * x147) + (-1 * x6 * x227) + (x3 * x228);
	const FLT x231 = x230 * x156;
	const FLT x232 = x60 * x231;
	const FLT x233 = x58 * x231;
	const FLT x234 = 0.5 * x226;
	const FLT x235 = x11 * x167;
	const FLT x236 = x230 * x157;
	const FLT x237 = x3 * x5;
	const FLT x238 = x237 * x167;
	const FLT x239 = (-1 * x238 * x166) + (x237 * x173);
	const FLT x240 = x239 + x203 + (-1 * x236 * x175) + (-1 * x235 * x187) + (x235 * x190) + (-1 * x230 * x207) +
					 (x233 * x161) + (-1 * x232 * x184) + x196 + (-1 * x234 * x205);
	const FLT x241 = (x238 * x201) + (-1 * x238 * x202);
	const FLT x242 = x241 + (x235 * x183) + (-1 * x174) + x170 + x137 + (-1 * x217 * x230) + (-1 * x215 * x231) +
					 (-1 * x235 * x186) + (x216 * x231) + (-1 * x233 * x184) + (-1 * x213 * x234);
	const FLT x243 = x238 * x187;
	const FLT x244 = x238 * x190;
	const FLT x245 = -1 * x214;
	const FLT x246 = x218 + (-1 * x234 * x180) + (-1 * x230 * x164) + x245 + (-1 * x235 * x166) + (x11 * x173) +
					 (-1 * x244) + (x231 * x185) + x243 + (x233 * x175) + (x236 * x161);
	const FLT x247 = x238 * x183;
	const FLT x248 = x238 * x186;
	const FLT x249 = (-1 * x247) + (-1 * x206) + (x235 * x201) + (x236 * x184) + x248 + (-1 * x231 * x194) +
					 (-1 * x235 * x202) + (-1 * x230 * x199) + (-1 * x232 * x175) + x208 + x209 + (-1 * x234 * x171);
	const FLT x250 = (x249 * (*_x0).Pose.Rot[2]) + (x240 * (*_x0).Pose.Rot[1]) + (x246 * (*_x0).Pose.Rot[0]) +
					 (x242 * (*_x0).Pose.Rot[3]);
	const FLT x251 = (-1 * x246 * (*_x0).Pose.Rot[2]) + (-1 * x240 * (*_x0).Pose.Rot[3]) + (x242 * (*_x0).Pose.Rot[1]) +
					 (x249 * (*_x0).Pose.Rot[0]);
	const FLT x252 = (-1 * x242 * (*_x0).Pose.Rot[2]) + (-1 * x246 * (*_x0).Pose.Rot[1]) + (x249 * (*_x0).Pose.Rot[3]) +
					 (x240 * (*_x0).Pose.Rot[0]);
	const FLT x253 = (x242 * (*_x0).Pose.Rot[0]) + (-1 * x249 * (*_x0).Pose.Rot[1]) + (x240 * (*_x0).Pose.Rot[2]) +
					 (-1 * x246 * (*_x0).Pose.Rot[3]);
	const FLT x254 = -1 * x91 * x251;
	const FLT x255 = x5 * x141;
	const FLT x256 = x255 * x140;
	const FLT x257 = x5 * x5 * x5;
	const FLT x258 = x5 * x146;
	const FLT x259 = (-1 * x11 * x258) + (x256 * x152) + (x5 * x228) + (-1 * x255 * x154) + (x9 * x256 * x139) +
					 (x257 * x143) + (-1 * x9 * x258) + (-1 * x257 * x146);
	const FLT x260 = x259 * x156;
	const FLT x261 = x58 * x260;
	const FLT x262 = x259 * x157;
	const FLT x263 = x6 * x167;
	const FLT x264 = x5 * x178;
	const FLT x265 = x62 * x260;
	const FLT x266 = x239 + (-1 * x264 * x180) + (-1 * x263 * x190) + (x263 * x187) + (x265 * x184) + (x261 * x175) +
					 (-1 * x259 * x164) + x193 + x197 + (x262 * x161) + (-1 * x195);
	const FLT x267 = x60 * x260;
	const FLT x268 = x191 + (-1 * x5 * x200) + x241 + (-1 * x267 * x175) + (x262 * x184) + (-1 * x263 * x183) + x136 +
					 (-1 * x259 * x199) + (x263 * x186) + (-1 * x265 * x161);
	const FLT x269 = (-1 * x263 * x166) + x244 + (-1 * x243) + x219 + (-1 * x207 * x259) + (-1 * x205 * x264) +
					 (-1 * x267 * x184) + (x261 * x161) + (-1 * x262 * x175) + x245 + (x263 * x172);
	const FLT x270 = x211 + (-1 * x267 * x161) + (-1 * x213 * x264) + (x201 * x263) + x209 + (-1 * x202 * x263) + x247 +
					 (-1 * x262 * x162) + (x265 * x175) + (-1 * x248) + (-1 * x261 * x184) + (-1 * x210);
	const FLT x271 = (-1 * x270 * (*_x0).Pose.Rot[2]) + (-1 * x266 * (*_x0).Pose.Rot[1]) + (x269 * (*_x0).Pose.Rot[0]) +
					 (x268 * (*_x0).Pose.Rot[3]);
	const FLT x272 = (x270 * (*_x0).Pose.Rot[3]) + (x266 * (*_x0).Pose.Rot[0]) + (x268 * (*_x0).Pose.Rot[2]) +
					 (x269 * (*_x0).Pose.Rot[1]);
	const FLT x273 = (-1 * x266 * (*_x0).Pose.Rot[2]) + (-1 * x269 * (*_x0).Pose.Rot[3]) + (x270 * (*_x0).Pose.Rot[1]) +
					 (x268 * (*_x0).Pose.Rot[0]);
	const FLT x274 = (x269 * (*_x0).Pose.Rot[2]) + (x270 * (*_x0).Pose.Rot[0]) + (-1 * x268 * (*_x0).Pose.Rot[1]) +
					 (-1 * x266 * (*_x0).Pose.Rot[3]);
	const FLT x275 = -1 * x91 * x273;
	const FLT x276 = 2 * (1. / sqrt(1 + (-4 * (((x82 * x86) + (-1 * x81 * x85)) * ((x82 * x86) + (-1 * x81 * x85))))));
	const FLT x277 = 1 + (-2 * (x88 + (x85 * x85)));
	const FLT x278 = 2 * (1. / x277);
	const FLT x279 = 4 * x85;
	const FLT x280 = x277 * x277;
	const FLT x281 = (x82 * x85) + (x81 * x86);
	const FLT x282 = 2 * (1. / x280) * x281;
	const FLT x283 = x280 * (1. / (x280 + (4 * (x281 * x281))));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT), x1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT), x1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT), x1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						x97 * ((-1 * x96 * ((-1 * x83 * x93) + x92)) +
							   (((x84 * x85) + (x86 * x87) + (x81 * x65) + (x82 * x83)) * x90)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						x97 * ((-1 * x96 * ((-1 * x93 * x109) + x113)) +
							   (((x85 * x111) + (x86 * x112) + (x82 * x109) + (x81 * x110)) * x90)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x97 * ((-1 * x96 * ((-1 * x93 * x122) + x126)) +
							   (((x86 * x125) + (x82 * x122) + (x85 * x124) + (x81 * x123)) * x90)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						x97 * ((-1 * x96 * ((-1 * x93 * x131) + x135)) +
							   (((x86 * x134) + (x82 * x131) + (x81 * x133) + (x85 * x132)) * x90)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x97 * ((-1 * x96 * ((-1 * x93 * x221) + x225)) +
							   (((x86 * x224) + (x85 * x223) + (x82 * x221) + (x81 * x222)) * x90)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x97 * ((-1 * x96 * ((-1 * x93 * x252) + x254)) +
							   (((x82 * x252) + (x86 * x253) + (x81 * x250) + (x85 * x251)) * x90)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x97 * ((-1 * x96 * ((-1 * x93 * x271) + x275)) +
							   (((x86 * x274) + (x85 * x273) + (x82 * x271) + (x81 * x272)) * x90)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						((-1 * x83 * x85) + (x82 * x84) + (-1 * x81 * x87) + (x86 * x65)) * x276);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						((-1 * x85 * x109) + (x82 * x111) + (x86 * x110) + (-1 * x81 * x112)) * x276);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						((-1 * x81 * x125) + (x82 * x124) + (-1 * x85 * x122) + (x86 * x123)) * x276);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						((-1 * x85 * x131) + (-1 * x81 * x134) + (x86 * x133) + (x82 * x132)) * x276);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						((-1 * x81 * x224) + (-1 * x85 * x221) + (x82 * x223) + (x86 * x222)) * x276);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						((-1 * x85 * x252) + (-1 * x81 * x253) + (x82 * x251) + (x86 * x250)) * x276);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						((-1 * x85 * x271) + (x86 * x272) + (-1 * x81 * x274) + (x82 * x273)) * x276);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						x283 * ((-1 * x282 * (x92 + (-1 * x87 * x279))) +
								(((x83 * x86) + (x81 * x84) + (x82 * x87) + (x85 * x65)) * x278)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						x283 * ((-1 * x282 * (x113 + (-1 * x279 * x112))) +
								(((x86 * x109) + (x81 * x111) + (x82 * x112) + (x85 * x110)) * x278)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x283 * ((-1 * x282 * (x126 + (-1 * x279 * x125))) +
								(((x86 * x122) + (x82 * x125) + (x81 * x124) + (x85 * x123)) * x278)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						x283 * ((-1 * x282 * (x135 + (-1 * x279 * x134))) +
								(((x86 * x131) + (x85 * x133) + (x82 * x134) + (x81 * x132)) * x278)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x283 * ((-1 * x282 * (x225 + (-1 * x279 * x224))) +
								(((x86 * x221) + (x81 * x223) + (x82 * x224) + (x85 * x222)) * x278)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x283 * ((-1 * x282 * (x254 + (-1 * x279 * x253))) +
								(((x86 * x252) + (x85 * x250) + (x81 * x251) + (x82 * x253)) * x278)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x283 * ((-1 * x282 * (x275 + (-1 * x279 * x274))) +
								(((x86 * x271) + (x82 * x274) + (x85 * x272) + (x81 * x273)) * x278)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT), t);
}

// Full version Jacobian of SurviveKalmanModelErrorPredict wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2],
// (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3],
// (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1],
// (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1],
// (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4f44220>] Jacobian of SurviveKalmanModelErrorPredict wrt
// [(*error_model).AccBias[0], (*error_model).AccBias[1], (*error_model).AccBias[2], (*error_model).Acc[0],
// (*error_model).Acc[1], (*error_model).Acc[2], (*error_model).GyroBias[0], (*error_model).GyroBias[1],
// (*error_model).GyroBias[2], (*error_model).IMUCorrection[0], (*error_model).IMUCorrection[1],
// (*error_model).IMUCorrection[2], (*error_model).IMUCorrection[3], (*error_model).Pose.AxisAngleRot[0],
// (*error_model).Pose.AxisAngleRot[1], (*error_model).Pose.AxisAngleRot[2], (*error_model).Pose.Pos[0],
// (*error_model).Pose.Pos[1], (*error_model).Pose.Pos[2], (*error_model).Velocity.AxisAngleRot[0],
// (*error_model).Velocity.AxisAngleRot[1], (*error_model).Velocity.AxisAngleRot[2], (*error_model).Velocity.Pos[0],
// (*error_model).Velocity.Pos[1], (*error_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at
// 0x7f88f4f44f70>]
static inline void SurviveKalmanModelErrorPredict_jac_error_model(CnMat *Hx, const FLT t, const SurviveKalmanModel *_x0,
																  const SurviveKalmanErrorModel *error_model) {
	const FLT x0 = 1.0 / 2.0 * t;
	const FLT x1 = fabs(t) * x0;
	const FLT x2 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x3 = 0.5 * (*_x0).Pose.Rot[3];
	const FLT x4 = 0.5 * (*_x0).Pose.Rot[2];
	const FLT x5 = 0.5 * (*_x0).Pose.Rot[0];
	const FLT x6 = (x5 * (*error_model).Pose.AxisAngleRot[0]) + (*_x0).Pose.Rot[1] +
				   (-1 * x3 * (*error_model).Pose.AxisAngleRot[1]) + (x4 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x7 = t * t;
	const FLT x8 = x2 * x2;
	const FLT x9 = x8 * x7;
	const FLT x10 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x11 = x10 * x10;
	const FLT x12 = x7 * x11;
	const FLT x13 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x14 = x13 * x13;
	const FLT x15 = x7 * x14;
	const FLT x16 = 1e-10 + x15 + x9 + x12;
	const FLT x17 = sqrt(x16);
	const FLT x18 = 0.5 * x17;
	const FLT x19 = sin(x18);
	const FLT x20 = x19 * x19;
	const FLT x21 = 1. / x16;
	const FLT x22 = x20 * x21;
	const FLT x23 = cos(x18);
	const FLT x24 = (x22 * x12) + (x23 * x23) + (x9 * x22) + (x22 * x15);
	const FLT x25 = 1. / sqrt(x24);
	const FLT x26 = x25 * x19;
	const FLT x27 = 1. / x17;
	const FLT x28 = t * x27;
	const FLT x29 = x28 * x26;
	const FLT x30 = x6 * x29;
	const FLT x31 = 0.5 * (*_x0).Pose.Rot[1];
	const FLT x32 = (*_x0).Pose.Rot[0] + (-1 * x31 * (*error_model).Pose.AxisAngleRot[0]) +
					(-1 * x4 * (*error_model).Pose.AxisAngleRot[1]) + (-1 * x3 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x33 = x32 * x29;
	const FLT x34 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x35 = (-1 * x34 * (*_x0).Pose.Rot[1]) + (x5 * (*error_model).Pose.AxisAngleRot[1]) +
					(x3 * (*error_model).Pose.AxisAngleRot[0]) + (*_x0).Pose.Rot[2];
	const FLT x36 = x25 * x23;
	const FLT x37 = x36 * x35;
	const FLT x38 = (x31 * (*error_model).Pose.AxisAngleRot[1]) + (*_x0).Pose.Rot[3] +
					(-1 * x4 * (*error_model).Pose.AxisAngleRot[0]) + (x34 * (*_x0).Pose.Rot[0]);
	const FLT x39 = x38 * x29;
	const FLT x40 = (-1 * x39 * x10) + (x2 * x30) + x37 + (x33 * x13);
	const FLT x41 = x36 * x38;
	const FLT x42 = x35 * x26;
	const FLT x43 = x42 * x28;
	const FLT x44 = (-1 * x30 * x13) + (x43 * x10) + (x2 * x33) + x41;
	const FLT x45 = x6 * x36;
	const FLT x46 = (x33 * x10) + (-1 * x2 * x43) + (x39 * x13) + x45;
	const FLT x47 = x32 * x36;
	const FLT x48 = (-1 * x30 * x10) + x47 + (-1 * x2 * x39) + (-1 * x43 * x13);
	const FLT x49 = (-1 * x48 * (*_x0).Pose.Rot[1]) + (x46 * (*_x0).Pose.Rot[0]) + (x40 * (*_x0).Pose.Rot[3]) +
					(-1 * x44 * (*_x0).Pose.Rot[2]);
	const FLT x50 = x29 * x10;
	const FLT x51 = x3 * x50;
	const FLT x52 = x29 * x13;
	const FLT x53 = x5 * x52;
	const FLT x54 = x2 * x29;
	const FLT x55 = x54 * x31;
	const FLT x56 = x4 * x36;
	const FLT x57 = (-1 * x56) + (-1 * x55) + x51 + (-1 * x53);
	const FLT x58 = x57 * (*_x0).Pose.Rot[3];
	const FLT x59 = x4 * x29;
	const FLT x60 = x59 * x10;
	const FLT x61 = x3 * x36;
	const FLT x62 = x5 * x54;
	const FLT x63 = x52 * x31;
	const FLT x64 = (-1 * x63) + x62 + x60 + x61;
	const FLT x65 = x31 * x36;
	const FLT x66 = x2 * x59;
	const FLT x67 = x5 * x50;
	const FLT x68 = x3 * x52;
	const FLT x69 = (-1 * x68) + (-1 * x67) + (-1 * x65) + x66;
	const FLT x70 = (x5 * x36) + (-1 * x50 * x31) + (-1 * x59 * x13) + (-1 * x3 * x54);
	const FLT x71 = x70 * (*_x0).Pose.Rot[1];
	const FLT x72 = x71 + (x69 * (*_x0).Pose.Rot[0]);
	const FLT x73 = x72 + x58 + (x64 * (*_x0).Pose.Rot[2]);
	const FLT x74 = (x46 * (*_x0).Pose.Rot[1]) + (x44 * (*_x0).Pose.Rot[3]) + (x48 * (*_x0).Pose.Rot[0]) +
					(x40 * (*_x0).Pose.Rot[2]);
	const FLT x75 = -1 * x69 * (*_x0).Pose.Rot[1];
	const FLT x76 = x70 * (*_x0).Pose.Rot[0];
	const FLT x77 = -1 * x57 * (*_x0).Pose.Rot[2];
	const FLT x78 = x77 + (x64 * (*_x0).Pose.Rot[3]) + x75 + x76;
	const FLT x79 = (-1 * x40 * (*_x0).Pose.Rot[1]) + (x46 * (*_x0).Pose.Rot[2]) + (-1 * x48 * (*_x0).Pose.Rot[3]) +
					(x44 * (*_x0).Pose.Rot[0]);
	const FLT x80 = x57 * (*_x0).Pose.Rot[1];
	const FLT x81 = x70 * (*_x0).Pose.Rot[3];
	const FLT x82 = x69 * (*_x0).Pose.Rot[2];
	const FLT x83 = (-1 * x82) + x80 + (x64 * (*_x0).Pose.Rot[0]) + (-1 * x81);
	const FLT x84 = (x44 * (*_x0).Pose.Rot[1]) + (x40 * (*_x0).Pose.Rot[0]) + (-1 * x46 * (*_x0).Pose.Rot[3]) +
					(-1 * x48 * (*_x0).Pose.Rot[2]);
	const FLT x85 = x69 * (*_x0).Pose.Rot[3];
	const FLT x86 = x70 * (*_x0).Pose.Rot[2];
	const FLT x87 = x86 + (x57 * (*_x0).Pose.Rot[0]);
	const FLT x88 = x87 + (-1 * x85) + (-1 * x64 * (*_x0).Pose.Rot[1]);
	const FLT x89 = x84 * x84;
	const FLT x90 = 1 + (-2 * ((x49 * x49) + x89));
	const FLT x91 = 2 * (1. / x90);
	const FLT x92 = 4 * x84;
	const FLT x93 = -1 * x83 * x92;
	const FLT x94 = 4 * x49;
	const FLT x95 = x90 * x90;
	const FLT x96 = (x74 * x49) + (x84 * x79);
	const FLT x97 = 2 * x96 * (1. / x95);
	const FLT x98 = x95 * (1. / (x95 + (4 * (x96 * x96))));
	const FLT x99 = x65 + (-1 * x66) + x67 + x68;
	const FLT x100 = x63 + (-1 * x62) + (-1 * x60) + (-1 * x61);
	const FLT x101 = (x100 * (*_x0).Pose.Rot[0]) + x81;
	const FLT x102 = x101 + (-1 * x80) + (-1 * x99 * (*_x0).Pose.Rot[2]);
	const FLT x103 = x100 * (*_x0).Pose.Rot[1];
	const FLT x104 = x87 + x103 + (x99 * (*_x0).Pose.Rot[3]);
	const FLT x105 = x76 + (-1 * x100 * (*_x0).Pose.Rot[3]);
	const FLT x106 = x105 + (x99 * (*_x0).Pose.Rot[1]) + x77;
	const FLT x107 = x100 * (*_x0).Pose.Rot[2];
	const FLT x108 = (-1 * x58) + (x99 * (*_x0).Pose.Rot[0]) + x107 + (-1 * x71);
	const FLT x109 = -1 * x92 * x106;
	const FLT x110 = x56 + x53 + (-1 * x51) + x55;
	const FLT x111 = x101 + (x110 * (*_x0).Pose.Rot[1]) + x82;
	const FLT x112 = x72 + (-1 * x110 * (*_x0).Pose.Rot[3]) + (-1 * x107);
	const FLT x113 = (-1 * x86) + x85 + (-1 * x103) + (x110 * (*_x0).Pose.Rot[0]);
	const FLT x114 = x105 + x75 + (x110 * (*_x0).Pose.Rot[2]);
	const FLT x115 = -1 * x92 * x112;
	const FLT x116 = x10 * x10 * x10;
	const FLT x117 = t * t * t * t;
	const FLT x118 = 1. / (x16 * sqrt(x16));
	const FLT x119 = 1.0 * x23 * x19;
	const FLT x120 = x118 * x119;
	const FLT x121 = x117 * x120;
	const FLT x122 = 2 * x20 * (1. / (x16 * x16));
	const FLT x123 = x117 * x122;
	const FLT x124 = x8 * x121;
	const FLT x125 = x7 * x10;
	const FLT x126 = 2 * x22;
	const FLT x127 = x14 * x10;
	const FLT x128 = x8 * x123;
	const FLT x129 = x27 * x125;
	const FLT x130 = (-1 * x119 * x129) + (-1 * x10 * x128) + (-1 * x116 * x123) + (x10 * x124) + (-1 * x123 * x127) +
					 (x126 * x125) + (x116 * x121) + (x121 * x127);
	const FLT x131 = 1. / (x24 * sqrt(x24));
	const FLT x132 = x38 * x131;
	const FLT x133 = x130 * x132;
	const FLT x134 = x0 * x27 * x19;
	const FLT x135 = x133 * x134;
	const FLT x136 = -1 * x30;
	const FLT x137 = 1.0 / 2.0 * x23;
	const FLT x138 = x131 * x137;
	const FLT x139 = x130 * x138;
	const FLT x140 = t * t * t;
	const FLT x141 = 0.5 * x21 * x140;
	const FLT x142 = x37 * x141;
	const FLT x143 = x13 * x10;
	const FLT x144 = x142 * x143;
	const FLT x145 = x118 * x140;
	const FLT x146 = x143 * x145;
	const FLT x147 = x42 * x146;
	const FLT x148 = x134 * x131;
	const FLT x149 = x13 * x148;
	const FLT x150 = x35 * x130;
	const FLT x151 = x32 * x26;
	const FLT x152 = 0.5 * x151;
	const FLT x153 = x6 * x26;
	const FLT x154 = x11 * x145;
	const FLT x155 = x6 * x130;
	const FLT x156 = x10 * x148;
	const FLT x157 = x11 * x141;
	const FLT x158 = x38 * x26;
	const FLT x159 = x2 * x145;
	const FLT x160 = x10 * x159;
	const FLT x161 = x2 * x141;
	const FLT x162 = x10 * x161;
	const FLT x163 = (-1 * x41 * x162) + (x160 * x158);
	const FLT x164 = x163 + (-1 * x45 * x157) + (-1 * x129 * x152) + (x153 * x154) + x147 + x136 + (-1 * x32 * x139) +
					 (x155 * x156) + (x149 * x150) + (x2 * x135) + (-1 * x144);
	const FLT x165 = -1 * x39;
	const FLT x166 = x160 * x153;
	const FLT x167 = x2 * x148;
	const FLT x168 = x45 * x162;
	const FLT x169 = 0.5 * x42;
	const FLT x170 = x141 * x143;
	const FLT x171 = (x47 * x170) + (-1 * x146 * x151);
	const FLT x172 = (x10 * x135) + (x154 * x158) + (-1 * x129 * x169) + x171 + (-1 * x166) + (-1 * x35 * x139) + x165 +
					 (-1 * x41 * x157) + (-1 * x167 * x155) + (-1 * x32 * x130 * x149) + x168;
	const FLT x173 = 0.5 * x153;
	const FLT x174 = x41 * x170;
	const FLT x175 = x6 * x138;
	const FLT x176 = x146 * x158;
	const FLT x177 = x42 * x159;
	const FLT x178 = x10 * x177;
	const FLT x179 = x32 * x148;
	const FLT x180 = x179 * x130;
	const FLT x181 = x2 * x142;
	const FLT x182 = x10 * x181;
	const FLT x183 = x174 + (-1 * x175 * x130) + (-1 * x176) + (x167 * x150) + (-1 * x151 * x154) + (x47 * x157) + x33 +
					 (-1 * x129 * x173) + (-1 * x13 * x135) + (-1 * x10 * x180) + x178 + (-1 * x182);
	const FLT x184 = 0.5 * x158;
	const FLT x185 = (x47 * x162) + (-1 * x160 * x151);
	const FLT x186 = (-1 * x45 * x170) + (x146 * x153);
	const FLT x187 = x185 + (x11 * x142) + (-1 * x150 * x156) + (-1 * x133 * x137) + (-1 * x129 * x184) + x43 +
					 (x149 * x155) + x186 + (-1 * x42 * x154) + (-1 * x2 * x180);
	const FLT x188 = (-1 * x187 * (*_x0).Pose.Rot[2]) + (-1 * x164 * (*_x0).Pose.Rot[1]) + (x183 * (*_x0).Pose.Rot[0]) +
					 (x172 * (*_x0).Pose.Rot[3]);
	const FLT x189 = (x172 * (*_x0).Pose.Rot[2]) + (x187 * (*_x0).Pose.Rot[3]) + (x183 * (*_x0).Pose.Rot[1]) +
					 (x164 * (*_x0).Pose.Rot[0]);
	const FLT x190 = (-1 * x183 * (*_x0).Pose.Rot[3]) + (-1 * x164 * (*_x0).Pose.Rot[2]) + (x187 * (*_x0).Pose.Rot[1]) +
					 (x172 * (*_x0).Pose.Rot[0]);
	const FLT x191 = (x187 * (*_x0).Pose.Rot[0]) + (-1 * x172 * (*_x0).Pose.Rot[1]) + (-1 * x164 * (*_x0).Pose.Rot[3]) +
					 (x183 * (*_x0).Pose.Rot[2]);
	const FLT x192 = -1 * x92 * x190;
	const FLT x193 = x13 * x11;
	const FLT x194 = x7 * x27;
	const FLT x195 = x13 * x194;
	const FLT x196 = x7 * x126;
	const FLT x197 = x13 * x13 * x13;
	const FLT x198 = (x121 * x197) + (-1 * x123 * x197) + (-1 * x123 * x193) + (-1 * x119 * x195) + (x13 * x124) +
					 (x121 * x193) + (-1 * x13 * x128) + (x13 * x196);
	const FLT x199 = x10 * x198;
	const FLT x200 = x35 * x167;
	const FLT x201 = x14 * x141;
	const FLT x202 = x14 * x145;
	const FLT x203 = x134 * x132;
	const FLT x204 = (x13 * x177) + (-1 * x13 * x181);
	const FLT x205 = (-1 * x13 * x203 * x198) + (-1 * x175 * x198) + (-1 * x173 * x195) + x204 + (-1 * x179 * x199) +
					 (-1 * x202 * x158) + (x41 * x201) + (x200 * x198) + x171 + x39;
	const FLT x206 = x35 * x148;
	const FLT x207 = x132 * x137;
	const FLT x208 = x2 * x198;
	const FLT x209 = x6 * x198;
	const FLT x210 = x13 * x161;
	const FLT x211 = x13 * x159;
	const FLT x212 = (-1 * x211 * x151) + (x47 * x210);
	const FLT x213 = (-1 * x45 * x201) + x212 + x144 + (x209 * x149) + (x202 * x153) + (-1 * x207 * x198) + x136 +
					 (-1 * x208 * x179) + (-1 * x147) + (-1 * x206 * x199) + (-1 * x184 * x195);
	const FLT x214 = -1 * x43;
	const FLT x215 = x211 * x158;
	const FLT x216 = x198 * x149;
	const FLT x217 = x41 * x210;
	const FLT x218 = x198 * x138;
	const FLT x219 = (-1 * x195 * x152) + (-1 * x32 * x218) + (-1 * x14 * x142) + (-1 * x217) + (x209 * x156) + x215 +
					 x214 + (x203 * x208) + (x42 * x202) + x186 + (x35 * x216);
	const FLT x220 = x211 * x153;
	const FLT x221 = x45 * x210;
	const FLT x222 = (-1 * x174) + (-1 * x202 * x151) + (x47 * x201) + (-1 * x35 * x218) + (x203 * x199) +
					 (-1 * x32 * x216) + (-1 * x220) + x176 + (-1 * x209 * x167) + x221 + x33 + (-1 * x169 * x195);
	const FLT x223 = (x222 * (*_x0).Pose.Rot[2]) + (x219 * (*_x0).Pose.Rot[0]) + (x205 * (*_x0).Pose.Rot[1]) +
					 (x213 * (*_x0).Pose.Rot[3]);
	const FLT x224 = (-1 * x205 * (*_x0).Pose.Rot[3]) + (-1 * x219 * (*_x0).Pose.Rot[2]) + (x213 * (*_x0).Pose.Rot[1]) +
					 (x222 * (*_x0).Pose.Rot[0]);
	const FLT x225 = (-1 * x213 * (*_x0).Pose.Rot[2]) + (-1 * x219 * (*_x0).Pose.Rot[1]) + (x222 * (*_x0).Pose.Rot[3]) +
					 (x205 * (*_x0).Pose.Rot[0]);
	const FLT x226 = (x205 * (*_x0).Pose.Rot[2]) + (-1 * x222 * (*_x0).Pose.Rot[1]) + (x213 * (*_x0).Pose.Rot[0]) +
					 (-1 * x219 * (*_x0).Pose.Rot[3]);
	const FLT x227 = -1 * x92 * x224;
	const FLT x228 = x2 * x121;
	const FLT x229 = x2 * x194;
	const FLT x230 = (x2 * x2 * x2) * x117;
	const FLT x231 = x2 * x123;
	const FLT x232 = (-1 * x14 * x231) + (x2 * x196) + (x14 * x228) + (x11 * x228) + (x230 * x120) + (-1 * x11 * x231) +
					 (-1 * x229 * x119) + (-1 * x230 * x122);
	const FLT x233 = x232 * x149;
	const FLT x234 = x232 * x203;
	const FLT x235 = x8 * x145;
	const FLT x236 = x232 * x138;
	const FLT x237 = x8 * x141;
	const FLT x238 = x10 * x232;
	const FLT x239 = x204 + (-1 * x41 * x237) + (-1 * x32 * x236) + (x35 * x233) + (-1 * x229 * x152) + x165 +
					 (x2 * x234) + (x235 * x158) + x166 + (x6 * x238 * x148) + (-1 * x168);
	const FLT x240 = x32 * x232;
	const FLT x241 = x163 + (-1 * x229 * x169) + (x45 * x237) + x30 + x212 + (x238 * x203) + (-1 * x235 * x153) +
					 (-1 * x35 * x236) + (-1 * x240 * x149) + (-1 * x6 * x232 * x167);
	const FLT x242 = x185 + (-1 * x229 * x173) + x217 + (-1 * x238 * x179) + x214 + (-1 * x13 * x234) + (x232 * x200) +
					 (-1 * x8 * x142) + (-1 * x215) + (-1 * x232 * x175) + (x42 * x235);
	const FLT x243 = (-1 * x229 * x184) + x182 + (-1 * x178) + x33 + (-1 * x240 * x167) + x220 + (-1 * x232 * x207) +
					 (-1 * x235 * x151) + (x6 * x233) + (x47 * x237) + (-1 * x238 * x206) + (-1 * x221);
	const FLT x244 = (x242 * (*_x0).Pose.Rot[0]) + (-1 * x243 * (*_x0).Pose.Rot[2]) + (-1 * x239 * (*_x0).Pose.Rot[1]) +
					 (x241 * (*_x0).Pose.Rot[3]);
	const FLT x245 = (x241 * (*_x0).Pose.Rot[2]) + (x239 * (*_x0).Pose.Rot[0]) + (x243 * (*_x0).Pose.Rot[3]) +
					 (x242 * (*_x0).Pose.Rot[1]);
	const FLT x246 = (-1 * x239 * (*_x0).Pose.Rot[2]) + (-1 * x242 * (*_x0).Pose.Rot[3]) + (x243 * (*_x0).Pose.Rot[1]) +
					 (x241 * (*_x0).Pose.Rot[0]);
	const FLT x247 = (x243 * (*_x0).Pose.Rot[0]) + (x242 * (*_x0).Pose.Rot[2]) + (-1 * x241 * (*_x0).Pose.Rot[1]) +
					 (-1 * x239 * (*_x0).Pose.Rot[3]);
	const FLT x248 = -1 * x92 * x246;
	const FLT x249 = 2 * (1. / sqrt(1 + (-4 * (((x84 * x74) + (-1 * x79 * x49)) * ((x84 * x74) + (-1 * x79 * x49))))));
	const FLT x250 = 1 + (-2 * (x89 + (x79 * x79)));
	const FLT x251 = 2 * (1. / x250);
	const FLT x252 = 4 * x79;
	const FLT x253 = x250 * x250;
	const FLT x254 = (x79 * x74) + (x84 * x49);
	const FLT x255 = 2 * x254 * (1. / x253);
	const FLT x256 = x253 * (1. / (x253 + (4 * (x254 * x254))));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Acc[0]) / sizeof(FLT), x1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.Pos[0]) / sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Acc[1]) / sizeof(FLT), x1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.Pos[1]) / sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Acc[2]) / sizeof(FLT), x1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.Pos[2]) / sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						x98 * ((-1 * x97 * ((-1 * x78 * x94) + x93)) +
							   (((x88 * x84) + (x83 * x79) + (x73 * x49) + (x78 * x74)) * x91)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						x98 * ((-1 * x97 * ((-1 * x94 * x102) + x109)) +
							   (((x79 * x106) + (x74 * x102) + (x84 * x108) + (x49 * x104)) * x91)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						x98 * ((-1 * x97 * ((-1 * x94 * x113) + x115)) +
							   (((x84 * x114) + (x74 * x113) + (x49 * x111) + (x79 * x112)) * x91)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x98 * ((-1 * x97 * ((-1 * x94 * x188) + x192)) +
							   (((x79 * x190) + (x74 * x188) + (x84 * x191) + (x49 * x189)) * x91)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x98 * ((-1 * x97 * ((-1 * x94 * x225) + x227)) +
							   (((x74 * x225) + (x84 * x226) + (x49 * x223) + (x79 * x224)) * x91)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x98 * ((-1 * x97 * ((-1 * x94 * x244) + x248)) +
							   (((x79 * x246) + (x84 * x247) + (x74 * x244) + (x49 * x245)) * x91)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						((-1 * x88 * x49) + (x84 * x73) + (-1 * x79 * x78) + (x83 * x74)) * x249);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						((-1 * x79 * x102) + (-1 * x49 * x108) + (x74 * x106) + (x84 * x104)) * x249);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						((-1 * x79 * x113) + (-1 * x49 * x114) + (x84 * x111) + (x74 * x112)) * x249);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						((-1 * x79 * x188) + (x74 * x190) + (-1 * x49 * x191) + (x84 * x189)) * x249);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						((-1 * x79 * x225) + (-1 * x49 * x226) + (x74 * x224) + (x84 * x223)) * x249);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						((x84 * x245) + (-1 * x79 * x244) + (-1 * x49 * x247) + (x74 * x246)) * x249);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						x256 * ((-1 * x255 * (x93 + (-1 * x88 * x252))) +
								(((x88 * x74) + (x84 * x78) + (x73 * x79) + (x83 * x49)) * x251)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						x256 * ((-1 * x255 * (x109 + (-1 * x252 * x108))) +
								(((x49 * x106) + (x79 * x104) + (x84 * x102) + (x74 * x108)) * x251)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						x256 * ((-1 * x255 * (x115 + (-1 * x252 * x114))) +
								(((x49 * x112) + (x84 * x113) + (x74 * x114) + (x79 * x111)) * x251)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x256 * ((-1 * x255 * (x192 + (-1 * x252 * x191))) +
								(((x49 * x190) + (x84 * x188) + (x74 * x191) + (x79 * x189)) * x251)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x256 * ((-1 * x255 * (x227 + (-1 * x252 * x226))) +
								(((x84 * x225) + (x49 * x224) + (x79 * x223) + (x74 * x226)) * x251)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x256 * ((-1 * x255 * (x248 + (-1 * x252 * x247))) +
								(((x84 * x244) + (x74 * x247) + (x79 * x245) + (x49 * x246)) * x251)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Acc[0]) / sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Acc[1]) / sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Acc[2]) / sizeof(FLT), t);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Acc[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Acc[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Acc[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Acc[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Acc[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Acc[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, AccScale) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, AccScale) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, IMUCorrection[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, IMUCorrection[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, IMUCorrection[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, IMUCorrection[3]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, AccBias[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, AccBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, AccBias[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, AccBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, AccBias[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, AccBias[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, GyroBias[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, GyroBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, GyroBias[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, GyroBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, GyroBias[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, GyroBias[2]) / sizeof(FLT), 1);
}

// Full version Jacobian of SurviveKalmanModelErrorPredict wrt [(*error_model).AccBias[0], (*error_model).AccBias[1],
// (*error_model).AccBias[2], (*error_model).Acc[0], (*error_model).Acc[1], (*error_model).Acc[2],
// (*error_model).GyroBias[0], (*error_model).GyroBias[1], (*error_model).GyroBias[2], (*error_model).IMUCorrection[0],
// (*error_model).IMUCorrection[1], (*error_model).IMUCorrection[2], (*error_model).IMUCorrection[3],
// (*error_model).Pose.AxisAngleRot[0], (*error_model).Pose.AxisAngleRot[1], (*error_model).Pose.AxisAngleRot[2],
// (*error_model).Pose.Pos[0], (*error_model).Pose.Pos[1], (*error_model).Pose.Pos[2],
// (*error_model).Velocity.AxisAngleRot[0], (*error_model).Velocity.AxisAngleRot[1],
// (*error_model).Velocity.AxisAngleRot[2], (*error_model).Velocity.Pos[0], (*error_model).Velocity.Pos[1],
// (*error_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f44f70>]
static inline void SurviveObsErrorModelNoFlip(SurviveAxisAnglePose *out, const SurviveKalmanModel *_x0,
											  const SurviveAxisAnglePose *Z) {
	const FLT x0 = sqrt(1e-10 + ((*_x0).Pose.Rot[2] * (*_x0).Pose.Rot[2]) + ((*_x0).Pose.Rot[3] * (*_x0).Pose.Rot[3]) +
						((*_x0).Pose.Rot[1] * (*_x0).Pose.Rot[1]));
	const FLT x1 = 2 * (1. / x0) * atan2(x0, (*_x0).Pose.Rot[0]);
	out->Pos[0] = (*_x0).Pose.Pos[0] + (-1 * (*Z).Pos[0]);
	out->Pos[1] = (*_x0).Pose.Pos[1] + (-1 * (*Z).Pos[1]);
	out->Pos[2] = (*_x0).Pose.Pos[2] + (-1 * (*Z).Pos[2]);
	out->AxisAngleRot[0] = (x1 * (*_x0).Pose.Rot[1]) + (-1 * (*Z).AxisAngleRot[0]);
	out->AxisAngleRot[1] = (x1 * (*_x0).Pose.Rot[2]) + (-1 * (*Z).AxisAngleRot[1]);
	out->AxisAngleRot[2] = (x1 * (*_x0).Pose.Rot[3]) + (-1 * (*Z).AxisAngleRot[2]);
}

// Jacobian of SurviveObsErrorModelNoFlip wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2], (*_x0).Acc[0],
// (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2], (*_x0).IMUCorrection[0],
// (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3], (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1],
// (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1], (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3],
// (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1], (*_x0).Velocity.AxisAngleRot[2],
// (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at
// 0x7f88f4f36520>]
static inline void SurviveObsErrorModelNoFlip_jac_x0(CnMat *Hx, const SurviveKalmanModel *_x0,
													 const SurviveAxisAnglePose *Z) {
	const FLT x0 = (*_x0).Pose.Rot[3] * (*_x0).Pose.Rot[3];
	const FLT x1 = (*_x0).Pose.Rot[1] * (*_x0).Pose.Rot[1];
	const FLT x2 = (*_x0).Pose.Rot[2] * (*_x0).Pose.Rot[2];
	const FLT x3 = 1e-10 + x2 + x0 + x1;
	const FLT x4 = 2 * (1. / (x3 + ((*_x0).Pose.Rot[0] * (*_x0).Pose.Rot[0])));
	const FLT x5 = (1. / x3) * (*_x0).Pose.Rot[0];
	const FLT x6 = x4 * x5;
	const FLT x7 = sqrt(x3);
	const FLT x8 = 2 * atan2(x7, (*_x0).Pose.Rot[0]);
	const FLT x9 = x8 * (1. / x7);
	const FLT x10 = (1. / (x3 * sqrt(x3))) * x8;
	const FLT x11 = x4 * (*_x0).Pose.Rot[2];
	const FLT x12 = x5 * (*_x0).Pose.Rot[1];
	const FLT x13 = (x12 * x11) + (-1 * x10 * (*_x0).Pose.Rot[2] * (*_x0).Pose.Rot[1]);
	const FLT x14 = x10 * (*_x0).Pose.Rot[3];
	const FLT x15 = x4 * (*_x0).Pose.Rot[3];
	const FLT x16 = (x15 * x12) + (-1 * x14 * (*_x0).Pose.Rot[1]);
	const FLT x17 = (x5 * x15 * (*_x0).Pose.Rot[2]) + (-1 * x14 * (*_x0).Pose.Rot[2]);
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), -1 * x4 * (*_x0).Pose.Rot[1]);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), (-1 * x1 * x10) + (x1 * x6) + x9);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x13);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x16);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), -1 * x11);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x13);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), (-1 * x2 * x10) + (x2 * x6) + x9);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x17);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), -1 * x15);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x16);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x17);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), (x0 * x6) + (-1 * x0 * x10) + x9);
}

// Full version Jacobian of SurviveObsErrorModelNoFlip wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2],
// (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3],
// (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1],
// (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1],
// (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4f36520>] Jacobian of SurviveObsErrorModelNoFlip wrt
// [(*Z).AxisAngleRot[0], (*Z).AxisAngleRot[1], (*Z).AxisAngleRot[2], (*Z).Pos[0], (*Z).Pos[1], (*Z).Pos[2]]
static inline void SurviveObsErrorModelNoFlip_jac_Z(CnMat *Hx, const SurviveKalmanModel *_x0,
													const SurviveAxisAnglePose *Z) {
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[0]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, Pos[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[1]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, Pos[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[2]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, Pos[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT), -1);
}

// Full version Jacobian of SurviveObsErrorModelNoFlip wrt [(*Z).AxisAngleRot[0], (*Z).AxisAngleRot[1],
// (*Z).AxisAngleRot[2], (*Z).Pos[0], (*Z).Pos[1], (*Z).Pos[2]]
static inline void SurviveObsErrorModelFlip(SurviveAxisAnglePose *out, const SurviveKalmanModel *_x0,
											const SurviveAxisAnglePose *Z) {
	const FLT x0 = sqrt(1e-10 + ((*_x0).Pose.Rot[2] * (*_x0).Pose.Rot[2]) + ((*_x0).Pose.Rot[3] * (*_x0).Pose.Rot[3]) +
						((*_x0).Pose.Rot[1] * (*_x0).Pose.Rot[1]));
	const FLT x1 = 2 * (1. / x0) * atan2(x0, (*_x0).Pose.Rot[0]);
	const FLT x2 = (x1 * (*_x0).Pose.Rot[1]) + (-1 * (*Z).AxisAngleRot[0]);
	const FLT x3 = (x1 * (*_x0).Pose.Rot[3]) + (-1 * (*Z).AxisAngleRot[2]);
	const FLT x4 = (x1 * (*_x0).Pose.Rot[2]) + (-1 * (*Z).AxisAngleRot[1]);
	const FLT x5 = sqrt(1e-10 + (x4 * x4) + (x3 * x3) + (x2 * x2));
	const FLT x6 = (1. / x5) * (-6.28318530717959 + x5);
	out->Pos[0] = (*_x0).Pose.Pos[0] + (-1 * (*Z).Pos[0]);
	out->Pos[1] = (*_x0).Pose.Pos[1] + (-1 * (*Z).Pos[1]);
	out->Pos[2] = (*_x0).Pose.Pos[2] + (-1 * (*Z).Pos[2]);
	out->AxisAngleRot[0] = x2 * x6;
	out->AxisAngleRot[1] = x4 * x6;
	out->AxisAngleRot[2] = x3 * x6;
}

// Jacobian of SurviveObsErrorModelFlip wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2], (*_x0).Acc[0],
// (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2], (*_x0).IMUCorrection[0],
// (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3], (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1],
// (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1], (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3],
// (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1], (*_x0).Velocity.AxisAngleRot[2],
// (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at
// 0x7f88f4f51070>]
static inline void SurviveObsErrorModelFlip_jac_x0(CnMat *Hx, const SurviveKalmanModel *_x0,
												   const SurviveAxisAnglePose *Z) {
	const FLT x0 = (*_x0).Pose.Rot[3] * (*_x0).Pose.Rot[3];
	const FLT x1 = (*_x0).Pose.Rot[1] * (*_x0).Pose.Rot[1];
	const FLT x2 = (*_x0).Pose.Rot[2] * (*_x0).Pose.Rot[2];
	const FLT x3 = 1e-10 + x2 + x0 + x1;
	const FLT x4 = sqrt(x3);
	const FLT x5 = 2 * atan2(x4, (*_x0).Pose.Rot[0]);
	const FLT x6 = (1. / x4) * x5;
	const FLT x7 = (x6 * (*_x0).Pose.Rot[2]) + (-1 * (*Z).AxisAngleRot[1]);
	const FLT x8 = 1. / (x3 + ((*_x0).Pose.Rot[0] * (*_x0).Pose.Rot[0]));
	const FLT x9 = 4 * x8;
	const FLT x10 = (x6 * (*_x0).Pose.Rot[3]) + (-1 * (*Z).AxisAngleRot[2]);
	const FLT x11 = (x6 * (*_x0).Pose.Rot[1]) + (-1 * (*Z).AxisAngleRot[0]);
	const FLT x12 = (-1 * x9 * x11 * (*_x0).Pose.Rot[1]) + (-1 * x7 * x9 * (*_x0).Pose.Rot[2]) +
					(-1 * x9 * x10 * (*_x0).Pose.Rot[3]);
	const FLT x13 = 1e-10 + (x10 * x10) + (x7 * x7) + (x11 * x11);
	const FLT x14 = 1.0 / 2.0 * (1. / x13);
	const FLT x15 = x14 * x11;
	const FLT x16 = sqrt(x13);
	const FLT x17 = -6.28318530717959 + x16;
	const FLT x18 = 1.0 / 2.0 * (1. / (x13 * sqrt(x13))) * x17;
	const FLT x19 = x11 * x18;
	const FLT x20 = x17 * (1. / x16);
	const FLT x21 = 2 * x8;
	const FLT x22 = x20 * x21;
	const FLT x23 = (1. / (x3 * sqrt(x3))) * x5;
	const FLT x24 = (*_x0).Pose.Rot[2] * (*_x0).Pose.Rot[1];
	const FLT x25 = (1. / x3) * x21 * (*_x0).Pose.Rot[0];
	const FLT x26 = (x24 * x25) + (-1 * x24 * x23);
	const FLT x27 = 2 * x7;
	const FLT x28 = x23 * (*_x0).Pose.Rot[3];
	const FLT x29 = x25 * (*_x0).Pose.Rot[3];
	const FLT x30 = (x29 * (*_x0).Pose.Rot[1]) + (-1 * x28 * (*_x0).Pose.Rot[1]);
	const FLT x31 = 2 * x10;
	const FLT x32 = (-1 * x1 * x23) + (x1 * x25) + x6;
	const FLT x33 = 2 * x11;
	const FLT x34 = (x32 * x33) + (x26 * x27) + (x30 * x31);
	const FLT x35 = (-1 * x2 * x23) + (x2 * x25) + x6;
	const FLT x36 = (x29 * (*_x0).Pose.Rot[2]) + (-1 * x28 * (*_x0).Pose.Rot[2]);
	const FLT x37 = (x33 * x26) + (x35 * x27) + (x31 * x36);
	const FLT x38 = x20 * x26;
	const FLT x39 = (-1 * x0 * x23) + (x0 * x25) + x6;
	const FLT x40 = (x30 * x33) + (x36 * x27) + (x31 * x39);
	const FLT x41 = x30 * x20;
	const FLT x42 = x7 * x14;
	const FLT x43 = x7 * x18;
	const FLT x44 = x36 * x20;
	const FLT x45 = x14 * x10;
	const FLT x46 = x10 * x18;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						(-1 * x22 * (*_x0).Pose.Rot[1]) + (x15 * x12) + (-1 * x12 * x19));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						(x32 * x20) + (x34 * x15) + (-1 * x34 * x19));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x38 + (x37 * x15) + (-1 * x37 * x19));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x41 + (x40 * x15) + (-1 * x40 * x19));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						(-1 * x22 * (*_x0).Pose.Rot[2]) + (x42 * x12) + (-1 * x43 * x12));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), (x42 * x34) + x38 + (-1 * x43 * x34));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						(x35 * x20) + (x42 * x37) + (-1 * x43 * x37));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x44 + (x40 * x42) + (-1 * x40 * x43));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						(-1 * x22 * (*_x0).Pose.Rot[3]) + (x45 * x12) + (-1 * x46 * x12));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x41 + (x45 * x34) + (-1 * x46 * x34));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), (x45 * x37) + x44 + (-1 * x46 * x37));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						(x40 * x45) + (x39 * x20) + (-1 * x40 * x46));
}

// Full version Jacobian of SurviveObsErrorModelFlip wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2],
// (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3],
// (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1],
// (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1],
// (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4f51070>] Jacobian of SurviveObsErrorModelFlip wrt
// [(*Z).AxisAngleRot[0], (*Z).AxisAngleRot[1], (*Z).AxisAngleRot[2], (*Z).Pos[0], (*Z).Pos[1], (*Z).Pos[2]]
static inline void SurviveObsErrorModelFlip_jac_Z(CnMat *Hx, const SurviveKalmanModel *_x0,
												  const SurviveAxisAnglePose *Z) {
	const FLT x0 = sqrt(1e-10 + ((*_x0).Pose.Rot[2] * (*_x0).Pose.Rot[2]) + ((*_x0).Pose.Rot[3] * (*_x0).Pose.Rot[3]) +
						((*_x0).Pose.Rot[1] * (*_x0).Pose.Rot[1]));
	const FLT x1 = 2 * (1. / x0) * atan2(x0, (*_x0).Pose.Rot[0]);
	const FLT x2 = (x1 * (*_x0).Pose.Rot[3]) + (-1 * (*Z).AxisAngleRot[2]);
	const FLT x3 = x2 * x2;
	const FLT x4 = (x1 * (*_x0).Pose.Rot[1]) + (-1 * (*Z).AxisAngleRot[0]);
	const FLT x5 = x4 * x4;
	const FLT x6 = (x1 * (*_x0).Pose.Rot[2]) + (-1 * (*Z).AxisAngleRot[1]);
	const FLT x7 = x6 * x6;
	const FLT x8 = 1e-10 + x7 + x3 + x5;
	const FLT x9 = 1. / x8;
	const FLT x10 = sqrt(x8);
	const FLT x11 = -6.28318530717959 + x10;
	const FLT x12 = (1. / (x8 * sqrt(x8))) * x11;
	const FLT x13 = -1 * x11 * (1. / x10);
	const FLT x14 = x4 * x6;
	const FLT x15 = (x14 * x12) + (-1 * x9 * x14);
	const FLT x16 = x2 * x9;
	const FLT x17 = x2 * x12;
	const FLT x18 = (x4 * x17) + (-1 * x4 * x16);
	const FLT x19 = (x6 * x17) + (-1 * x6 * x16);
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[0]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, Pos[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[1]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, Pos[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[2]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, Pos[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						(-1 * x5 * x9) + x13 + (x5 * x12));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT), x15);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT), x15);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						x13 + (-1 * x7 * x9) + (x7 * x12));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT), x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT), x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						(-1 * x3 * x9) + x13 + (x3 * x12));
}

// Full version Jacobian of SurviveObsErrorModelFlip wrt [(*Z).AxisAngleRot[0], (*Z).AxisAngleRot[1],
// (*Z).AxisAngleRot[2], (*Z).Pos[0], (*Z).Pos[1], (*Z).Pos[2]]
static inline void SurviveObsErrorStateErrorModelNoFlip(SurviveAxisAnglePose *out, const SurviveKalmanModel *_x0,
														const SurviveKalmanErrorModel *err,
														const SurviveAxisAnglePose *Z) {
	const FLT x0 = 0.5 * (*_x0).Pose.Rot[3];
	const FLT x1 = 0.5 * (*err).Pose.AxisAngleRot[2];
	const FLT x2 = 0.5 * (*err).Pose.AxisAngleRot[0];
	const FLT x3 = (*_x0).Pose.Rot[1] + (x2 * (*_x0).Pose.Rot[0]) + (-1 * x0 * (*err).Pose.AxisAngleRot[1]) +
				   (x1 * (*_x0).Pose.Rot[2]);
	const FLT x4 = 0.5 * (*err).Pose.AxisAngleRot[1];
	const FLT x5 =
		(x4 * (*_x0).Pose.Rot[1]) + (-1 * x2 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[3] + (x1 * (*_x0).Pose.Rot[0]);
	const FLT x6 = (-1 * x1 * (*_x0).Pose.Rot[1]) + (x4 * (*_x0).Pose.Rot[0]) + (x0 * (*err).Pose.AxisAngleRot[0]) +
				   (*_x0).Pose.Rot[2];
	const FLT x7 = sqrt(1e-10 + (x6 * x6) + (x5 * x5) + (x3 * x3));
	const FLT x8 = 2 * (1. / x7) *
				   atan2(x7, (-1 * x0 * (*err).Pose.AxisAngleRot[2]) + (-1 * x2 * (*_x0).Pose.Rot[1]) +
								 (*_x0).Pose.Rot[0] + (-1 * x4 * (*_x0).Pose.Rot[2]));
	out->Pos[0] = (*_x0).Pose.Pos[0] + (-1 * (*Z).Pos[0]) + (*err).Pose.Pos[0];
	out->Pos[1] = (*_x0).Pose.Pos[1] + (-1 * (*Z).Pos[1]) + (*err).Pose.Pos[1];
	out->Pos[2] = (*_x0).Pose.Pos[2] + (-1 * (*Z).Pos[2]) + (*err).Pose.Pos[2];
	out->AxisAngleRot[0] = (x3 * x8) + (-1 * (*Z).AxisAngleRot[0]);
	out->AxisAngleRot[1] = (x6 * x8) + (-1 * (*Z).AxisAngleRot[1]);
	out->AxisAngleRot[2] = (x5 * x8) + (-1 * (*Z).AxisAngleRot[2]);
}

// Jacobian of SurviveObsErrorStateErrorModelNoFlip wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2],
// (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3],
// (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1],
// (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1],
// (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4f28fd0>]
static inline void SurviveObsErrorStateErrorModelNoFlip_jac_x0(CnMat *Hx, const SurviveKalmanModel *_x0,
															   const SurviveKalmanErrorModel *err,
															   const SurviveAxisAnglePose *Z) {
	const FLT x0 = 0.5 * (*err).Pose.AxisAngleRot[1];
	const FLT x1 = 0.5 * (*_x0).Pose.Rot[2];
	const FLT x2 = 0.5 * (*err).Pose.AxisAngleRot[0];
	const FLT x3 = (*_x0).Pose.Rot[1] + (x2 * (*_x0).Pose.Rot[0]) + (-1 * x0 * (*_x0).Pose.Rot[3]) +
				   (x1 * (*err).Pose.AxisAngleRot[2]);
	const FLT x4 = 2 * x3;
	const FLT x5 = 0.5 * (*err).Pose.AxisAngleRot[2];
	const FLT x6 =
		(-1 * x5 * (*_x0).Pose.Rot[1]) + (x0 * (*_x0).Pose.Rot[0]) + (x2 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x7 = 1.0 * (*err).Pose.AxisAngleRot[1];
	const FLT x8 = (x0 * (*_x0).Pose.Rot[1]) + (-1 * x1 * (*err).Pose.AxisAngleRot[0]) + (*_x0).Pose.Rot[3] +
				   (x5 * (*_x0).Pose.Rot[0]);
	const FLT x9 = 1.0 * x8;
	const FLT x10 = 1.0 * x3;
	const FLT x11 = (x10 * (*err).Pose.AxisAngleRot[0]) + (x6 * x7) + (x9 * (*err).Pose.AxisAngleRot[2]);
	const FLT x12 = 1e-10 + (x6 * x6) + (x8 * x8) + (x3 * x3);
	const FLT x13 = sqrt(x12);
	const FLT x14 = 1. / x13;
	const FLT x15 = (-1 * x2 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[0] + (-1 * x5 * (*_x0).Pose.Rot[3]) +
					(-1 * x1 * (*err).Pose.AxisAngleRot[1]);
	const FLT x16 = 1.0 / 2.0 * (1. / x15) * x14;
	const FLT x17 = x15 * x15;
	const FLT x18 = x13 * (1. / x17);
	const FLT x19 = (1. / (x12 + x17)) * x14 * x17;
	const FLT x20 = x19 * ((-1 * x18) + (x11 * x16));
	const FLT x21 = atan2(x13, x15);
	const FLT x22 = x21 * (1. / (x12 * sqrt(x12)));
	const FLT x23 = x3 * x22;
	const FLT x24 = x21 * x14;
	const FLT x25 = 1.0 * x24;
	const FLT x26 = x25 * (*err).Pose.AxisAngleRot[0];
	const FLT x27 = 2 * x24;
	const FLT x28 = 1.0 * x6;
	const FLT x29 = x4 + (-1 * x28 * (*err).Pose.AxisAngleRot[2]) + (x8 * x7);
	const FLT x30 = x19 * ((x2 * x18) + (x29 * x16));
	const FLT x31 = 2 * x6;
	const FLT x32 = (x10 * (*err).Pose.AxisAngleRot[2]) + x31 + (-1 * x9 * (*err).Pose.AxisAngleRot[0]);
	const FLT x33 = x19 * ((x0 * x18) + (x32 * x16));
	const FLT x34 = x32 * x22;
	const FLT x35 = x25 * (*err).Pose.AxisAngleRot[2];
	const FLT x36 = 2 * x8;
	const FLT x37 = (-1 * x3 * x7) + (x28 * (*err).Pose.AxisAngleRot[0]) + x36;
	const FLT x38 = x19 * ((x5 * x18) + (x37 * x16));
	const FLT x39 = x7 * x24;
	const FLT x40 = x6 * x22;
	const FLT x41 = x8 * x22;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x26 + (x4 * x20) + (-1 * x23 * x11));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), (-1 * x23 * x29) + x27 + (x4 * x30));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), (x4 * x33) + x35 + (-1 * x3 * x34));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						(-1 * x39) + (x4 * x38) + (-1 * x37 * x23));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x39 + (x31 * x20) + (-1 * x40 * x11));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						(-1 * x35) + (x30 * x31) + (-1 * x40 * x29));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x27 + (x31 * x33) + (-1 * x6 * x34));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x26 + (x31 * x38) + (-1 * x40 * x37));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x35 + (-1 * x41 * x11) + (x36 * x20));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x39 + (-1 * x41 * x29) + (x30 * x36));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						(-1 * x26) + (x33 * x36) + (-1 * x8 * x34));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x27 + (x36 * x38) + (-1 * x41 * x37));
}

// Full version Jacobian of SurviveObsErrorStateErrorModelNoFlip wrt [(*_x0).AccBias[0], (*_x0).AccBias[1],
// (*_x0).AccBias[2], (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1],
// (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2],
// (*_x0).IMUCorrection[3], (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0],
// (*_x0).Pose.Rot[1], (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0],
// (*_x0).Velocity.AxisAngleRot[1], (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1],
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f28fd0>] Jacobian of
// SurviveObsErrorStateErrorModelNoFlip wrt [(*err).AccBias[0], (*err).AccBias[1], (*err).AccBias[2], (*err).Acc[0],
// (*err).Acc[1], (*err).Acc[2], (*err).GyroBias[0], (*err).GyroBias[1], (*err).GyroBias[2], (*err).IMUCorrection[0],
// (*err).IMUCorrection[1], (*err).IMUCorrection[2], (*err).IMUCorrection[3], (*err).Pose.AxisAngleRot[0],
// (*err).Pose.AxisAngleRot[1], (*err).Pose.AxisAngleRot[2], (*err).Pose.Pos[0], (*err).Pose.Pos[1], (*err).Pose.Pos[2],
// (*err).Velocity.AxisAngleRot[0], (*err).Velocity.AxisAngleRot[1], (*err).Velocity.AxisAngleRot[2],
// (*err).Velocity.Pos[0], (*err).Velocity.Pos[1], (*err).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at
// 0x7f88f4f3b3d0>]
static inline void SurviveObsErrorStateErrorModelNoFlip_jac_err(CnMat *Hx, const SurviveKalmanModel *_x0,
																const SurviveKalmanErrorModel *err,
																const SurviveAxisAnglePose *Z) {
	const FLT x0 = 0.5 * (*err).Pose.AxisAngleRot[2];
	const FLT x1 = 0.5 * (*_x0).Pose.Rot[2];
	const FLT x2 = 0.5 * (*err).Pose.AxisAngleRot[1];
	const FLT x3 = (-1 * x1 * (*err).Pose.AxisAngleRot[0]) + (x2 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] +
				   (x0 * (*_x0).Pose.Rot[0]);
	const FLT x4 = 1.0 * x3;
	const FLT x5 = 0.5 * (*err).Pose.AxisAngleRot[0];
	const FLT x6 =
		(-1 * x0 * (*_x0).Pose.Rot[1]) + (x5 * (*_x0).Pose.Rot[3]) + (x2 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2];
	const FLT x7 = 1.0 * x6;
	const FLT x8 = (*_x0).Pose.Rot[1] + (x5 * (*_x0).Pose.Rot[0]) + (-1 * x2 * (*_x0).Pose.Rot[3]) +
				   (x1 * (*err).Pose.AxisAngleRot[2]);
	const FLT x9 = 1.0 * x8;
	const FLT x10 = (x9 * (*_x0).Pose.Rot[0]) + (-1 * x4 * (*_x0).Pose.Rot[2]) + (x7 * (*_x0).Pose.Rot[3]);
	const FLT x11 = 1e-10 + (x3 * x3) + (x6 * x6) + (x8 * x8);
	const FLT x12 = sqrt(x11);
	const FLT x13 = 1. / x12;
	const FLT x14 = (*_x0).Pose.Rot[0] + (-1 * x0 * (*_x0).Pose.Rot[3]) + (-1 * x5 * (*_x0).Pose.Rot[1]) +
					(-1 * x1 * (*err).Pose.AxisAngleRot[1]);
	const FLT x15 = 1.0 / 2.0 * (1. / x14) * x13;
	const FLT x16 = x14 * x14;
	const FLT x17 = x12 * (1. / x16);
	const FLT x18 = 0.5 * x17;
	const FLT x19 = (x18 * (*_x0).Pose.Rot[1]) + (x15 * x10);
	const FLT x20 = 2 * (1. / (x11 + x16)) * x13 * x16;
	const FLT x21 = x8 * x20;
	const FLT x22 = atan2(x12, x14);
	const FLT x23 = x22 * (1. / (x11 * sqrt(x11)));
	const FLT x24 = x23 * x10;
	const FLT x25 = 1.0 * x22 * x13;
	const FLT x26 = x25 * (*_x0).Pose.Rot[0];
	const FLT x27 = (x7 * (*_x0).Pose.Rot[0]) + (-1 * x9 * (*_x0).Pose.Rot[3]) + (x4 * (*_x0).Pose.Rot[1]);
	const FLT x28 = (x1 * x17) + (x27 * x15);
	const FLT x29 = x23 * x27;
	const FLT x30 = x25 * (*_x0).Pose.Rot[3];
	const FLT x31 = (x9 * (*_x0).Pose.Rot[2]) + (-1 * x7 * (*_x0).Pose.Rot[1]) + (x4 * (*_x0).Pose.Rot[0]);
	const FLT x32 = (x18 * (*_x0).Pose.Rot[3]) + (x31 * x15);
	const FLT x33 = x31 * x23;
	const FLT x34 = x25 * (*_x0).Pose.Rot[2];
	const FLT x35 = x6 * x20;
	const FLT x36 = x25 * (*_x0).Pose.Rot[1];
	const FLT x37 = x3 * x20;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						x26 + (x21 * x19) + (-1 * x8 * x24));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						(-1 * x30) + (x21 * x28) + (-1 * x8 * x29));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						x34 + (x32 * x21) + (-1 * x8 * x33));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						(-1 * x6 * x24) + x30 + (x35 * x19));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						x26 + (x35 * x28) + (-1 * x6 * x29));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						(-1 * x36) + (x32 * x35) + (-1 * x6 * x33));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						(-1 * x34) + (-1 * x3 * x24) + (x37 * x19));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						x36 + (x37 * x28) + (-1 * x3 * x29));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						(x32 * x37) + x26 + (-1 * x3 * x33));
}

// Full version Jacobian of SurviveObsErrorStateErrorModelNoFlip wrt [(*err).AccBias[0], (*err).AccBias[1],
// (*err).AccBias[2], (*err).Acc[0], (*err).Acc[1], (*err).Acc[2], (*err).GyroBias[0], (*err).GyroBias[1],
// (*err).GyroBias[2], (*err).IMUCorrection[0], (*err).IMUCorrection[1], (*err).IMUCorrection[2],
// (*err).IMUCorrection[3], (*err).Pose.AxisAngleRot[0], (*err).Pose.AxisAngleRot[1], (*err).Pose.AxisAngleRot[2],
// (*err).Pose.Pos[0], (*err).Pose.Pos[1], (*err).Pose.Pos[2], (*err).Velocity.AxisAngleRot[0],
// (*err).Velocity.AxisAngleRot[1], (*err).Velocity.AxisAngleRot[2], (*err).Velocity.Pos[0], (*err).Velocity.Pos[1],
// (*err).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f3b3d0>] Jacobian of
// SurviveObsErrorStateErrorModelNoFlip wrt [(*Z).AxisAngleRot[0], (*Z).AxisAngleRot[1], (*Z).AxisAngleRot[2],
// (*Z).Pos[0], (*Z).Pos[1], (*Z).Pos[2]]
static inline void SurviveObsErrorStateErrorModelNoFlip_jac_Z(CnMat *Hx, const SurviveKalmanModel *_x0,
															  const SurviveKalmanErrorModel *err,
															  const SurviveAxisAnglePose *Z) {
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[0]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, Pos[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[1]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, Pos[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[2]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, Pos[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT), -1);
}

// Full version Jacobian of SurviveObsErrorStateErrorModelNoFlip wrt [(*Z).AxisAngleRot[0], (*Z).AxisAngleRot[1],
// (*Z).AxisAngleRot[2], (*Z).Pos[0], (*Z).Pos[1], (*Z).Pos[2]]
static inline void SurviveObsErrorStateErrorModelFlip(SurviveAxisAnglePose *out, const SurviveKalmanModel *_x0,
													  const SurviveKalmanErrorModel *err,
													  const SurviveAxisAnglePose *Z) {
	const FLT x0 = 0.5 * (*err).Pose.AxisAngleRot[1];
	const FLT x1 = 0.5 * (*_x0).Pose.Rot[2];
	const FLT x2 = 0.5 * (*err).Pose.AxisAngleRot[0];
	const FLT x3 = (*_x0).Pose.Rot[1] + (x2 * (*_x0).Pose.Rot[0]) + (-1 * x0 * (*_x0).Pose.Rot[3]) +
				   (x1 * (*err).Pose.AxisAngleRot[2]);
	const FLT x4 = 0.5 * (*err).Pose.AxisAngleRot[2];
	const FLT x5 = (*_x0).Pose.Rot[3] + (x0 * (*_x0).Pose.Rot[1]) + (-1 * x1 * (*err).Pose.AxisAngleRot[0]) +
				   (x4 * (*_x0).Pose.Rot[0]);
	const FLT x6 =
		(-1 * x4 * (*_x0).Pose.Rot[1]) + (x0 * (*_x0).Pose.Rot[0]) + (x2 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x7 = sqrt(1e-10 + (x6 * x6) + (x5 * x5) + (x3 * x3));
	const FLT x8 = 2 * (1. / x7) *
				   atan2(x7, (-1 * x2 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[0] + (-1 * x4 * (*_x0).Pose.Rot[3]) +
								 (-1 * x1 * (*err).Pose.AxisAngleRot[1]));
	const FLT x9 = (x3 * x8) + (-1 * (*Z).AxisAngleRot[0]);
	const FLT x10 = (x5 * x8) + (-1 * (*Z).AxisAngleRot[2]);
	const FLT x11 = (x6 * x8) + (-1 * (*Z).AxisAngleRot[1]);
	const FLT x12 = sqrt(1e-10 + (x11 * x11) + (x10 * x10) + (x9 * x9));
	const FLT x13 = (1. / x12) * (-6.28318530717959 + x12);
	out->Pos[0] = (*_x0).Pose.Pos[0] + (-1 * (*Z).Pos[0]) + (*err).Pose.Pos[0];
	out->Pos[1] = (*_x0).Pose.Pos[1] + (-1 * (*Z).Pos[1]) + (*err).Pose.Pos[1];
	out->Pos[2] = (*_x0).Pose.Pos[2] + (-1 * (*Z).Pos[2]) + (*err).Pose.Pos[2];
	out->AxisAngleRot[0] = x9 * x13;
	out->AxisAngleRot[1] = x13 * x11;
	out->AxisAngleRot[2] = x13 * x10;
}

// Jacobian of SurviveObsErrorStateErrorModelFlip wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2],
// (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3],
// (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1],
// (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1],
// (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4f2b3a0>]
static inline void SurviveObsErrorStateErrorModelFlip_jac_x0(CnMat *Hx, const SurviveKalmanModel *_x0,
															 const SurviveKalmanErrorModel *err,
															 const SurviveAxisAnglePose *Z) {
	const FLT x0 = 0.5 * (*err).Pose.AxisAngleRot[0];
	const FLT x1 = 0.5 * (*err).Pose.AxisAngleRot[1];
	const FLT x2 = 0.5 * (*err).Pose.AxisAngleRot[2];
	const FLT x3 =
		(-1 * x2 * (*_x0).Pose.Rot[1]) + (x0 * (*_x0).Pose.Rot[3]) + (x1 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2];
	const FLT x4 = 1.0 * x3;
	const FLT x5 =
		(x1 * (*_x0).Pose.Rot[1]) + (-1 * x0 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[3] + (x2 * (*_x0).Pose.Rot[0]);
	const FLT x6 = 1.0 * (*err).Pose.AxisAngleRot[2];
	const FLT x7 =
		(*_x0).Pose.Rot[1] + (x0 * (*_x0).Pose.Rot[0]) + (-1 * x1 * (*_x0).Pose.Rot[3]) + (x2 * (*_x0).Pose.Rot[2]);
	const FLT x8 = 1.0 * x7;
	const FLT x9 = (x8 * (*err).Pose.AxisAngleRot[0]) + (x4 * (*err).Pose.AxisAngleRot[1]) + (x6 * x5);
	const FLT x10 = 1e-10 + (x3 * x3) + (x5 * x5) + (x7 * x7);
	const FLT x11 = sqrt(x10);
	const FLT x12 = 1. / x11;
	const FLT x13 = (-1 * x0 * (*_x0).Pose.Rot[1]) + (-1 * x2 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[0] +
					(-1 * x1 * (*_x0).Pose.Rot[2]);
	const FLT x14 = 1.0 / 2.0 * (1. / x13) * x12;
	const FLT x15 = x13 * x13;
	const FLT x16 = (1. / x15) * x11;
	const FLT x17 = (-1 * x16) + (x9 * x14);
	const FLT x18 = 2 * x7;
	const FLT x19 = (1. / (x10 + x15)) * x15 * x12;
	const FLT x20 = x19 * x18;
	const FLT x21 = atan2(x11, x13);
	const FLT x22 = x21 * (1. / (x10 * sqrt(x10)));
	const FLT x23 = x7 * x22;
	const FLT x24 = x21 * x12;
	const FLT x25 = 1.0 * x24;
	const FLT x26 = x25 * (*err).Pose.AxisAngleRot[0];
	const FLT x27 = x26 + (x20 * x17) + (-1 * x9 * x23);
	const FLT x28 = 2 * x5;
	const FLT x29 = (x24 * x28) + (-1 * (*Z).AxisAngleRot[2]);
	const FLT x30 = (x24 * x18) + (-1 * (*Z).AxisAngleRot[0]);
	const FLT x31 = 2 * x3;
	const FLT x32 = (x31 * x24) + (-1 * (*Z).AxisAngleRot[1]);
	const FLT x33 = 1e-10 + (x32 * x32) + (x29 * x29) + (x30 * x30);
	const FLT x34 = sqrt(x33);
	const FLT x35 = -6.28318530717959 + x34;
	const FLT x36 = (1. / x34) * x35;
	const FLT x37 = x31 * x19;
	const FLT x38 = x3 * x22;
	const FLT x39 = x25 * (*err).Pose.AxisAngleRot[1];
	const FLT x40 = x39 + (x37 * x17) + (-1 * x9 * x38);
	const FLT x41 = 2 * x32;
	const FLT x42 = x5 * x22;
	const FLT x43 = x28 * x19;
	const FLT x44 = x6 * x24;
	const FLT x45 = (-1 * x9 * x42) + x44 + (x43 * x17);
	const FLT x46 = 2 * x29;
	const FLT x47 = 2 * x30;
	const FLT x48 = (x47 * x27) + (x40 * x41) + (x45 * x46);
	const FLT x49 = 1.0 / 2.0 * (1. / x33);
	const FLT x50 = x48 * x49;
	const FLT x51 = 1.0 / 2.0 * (1. / (x33 * sqrt(x33))) * x35;
	const FLT x52 = x51 * x48;
	const FLT x53 = 2 * x24;
	const FLT x54 = 1.0 * x5;
	const FLT x55 = x18 + (-1 * x3 * x6) + (x54 * (*err).Pose.AxisAngleRot[1]);
	const FLT x56 = (x0 * x16) + (x55 * x14);
	const FLT x57 = (-1 * x55 * x23) + x53 + (x56 * x20);
	const FLT x58 = (-1 * x44) + (x56 * x37) + (-1 * x55 * x38);
	const FLT x59 = (-1 * x55 * x42) + x39 + (x56 * x43);
	const FLT x60 = (x57 * x47) + (x58 * x41) + (x59 * x46);
	const FLT x61 = x60 * x49;
	const FLT x62 = x60 * x51;
	const FLT x63 = (x6 * x7) + x31 + (-1 * x54 * (*err).Pose.AxisAngleRot[0]);
	const FLT x64 = x19 * ((x1 * x16) + (x63 * x14));
	const FLT x65 = x44 + (x64 * x18) + (-1 * x63 * x23);
	const FLT x66 = (x64 * x31) + x53 + (-1 * x63 * x38);
	const FLT x67 = (-1 * x26) + (x64 * x28) + (-1 * x63 * x42);
	const FLT x68 = (x66 * x41) + (x65 * x47) + (x67 * x46);
	const FLT x69 = x68 * x49;
	const FLT x70 = x68 * x51;
	const FLT x71 = (x4 * (*err).Pose.AxisAngleRot[0]) + (-1 * x8 * (*err).Pose.AxisAngleRot[1]) + x28;
	const FLT x72 = x19 * ((x2 * x16) + (x71 * x14));
	const FLT x73 = (-1 * x39) + (x72 * x18) + (-1 * x71 * x23);
	const FLT x74 = x26 + (x72 * x31) + (-1 * x71 * x38);
	const FLT x75 = x53 + (x72 * x28) + (-1 * x71 * x42);
	const FLT x76 = (x73 * x47) + (x74 * x41) + (x75 * x46);
	const FLT x77 = x76 * x49;
	const FLT x78 = x76 * x51;
	const FLT x79 = x51 * x29;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						(-1 * x52 * x30) + (x36 * x27) + (x50 * x30));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						(x57 * x36) + (-1 * x62 * x30) + (x61 * x30));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						(-1 * x70 * x30) + (x65 * x36) + (x69 * x30));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						(-1 * x78 * x30) + (x73 * x36) + (x77 * x30));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						(-1 * x52 * x32) + (x40 * x36) + (x50 * x32));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						(-1 * x62 * x32) + (x58 * x36) + (x61 * x32));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						(-1 * x70 * x32) + (x66 * x36) + (x69 * x32));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						(x74 * x36) + (-1 * x78 * x32) + (x77 * x32));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						(-1 * x79 * x48) + (x45 * x36) + (x50 * x29));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						(-1 * x79 * x60) + (x59 * x36) + (x61 * x29));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						(x67 * x36) + (-1 * x79 * x68) + (x69 * x29));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						(-1 * x79 * x76) + (x77 * x29) + (x75 * x36));
}

// Full version Jacobian of SurviveObsErrorStateErrorModelFlip wrt [(*_x0).AccBias[0], (*_x0).AccBias[1],
// (*_x0).AccBias[2], (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1],
// (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2],
// (*_x0).IMUCorrection[3], (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0],
// (*_x0).Pose.Rot[1], (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0],
// (*_x0).Velocity.AxisAngleRot[1], (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1],
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f2b3a0>] Jacobian of
// SurviveObsErrorStateErrorModelFlip wrt [(*err).AccBias[0], (*err).AccBias[1], (*err).AccBias[2], (*err).Acc[0],
// (*err).Acc[1], (*err).Acc[2], (*err).GyroBias[0], (*err).GyroBias[1], (*err).GyroBias[2], (*err).IMUCorrection[0],
// (*err).IMUCorrection[1], (*err).IMUCorrection[2], (*err).IMUCorrection[3], (*err).Pose.AxisAngleRot[0],
// (*err).Pose.AxisAngleRot[1], (*err).Pose.AxisAngleRot[2], (*err).Pose.Pos[0], (*err).Pose.Pos[1], (*err).Pose.Pos[2],
// (*err).Velocity.AxisAngleRot[0], (*err).Velocity.AxisAngleRot[1], (*err).Velocity.AxisAngleRot[2],
// (*err).Velocity.Pos[0], (*err).Velocity.Pos[1], (*err).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at
// 0x7f88f4f31e20>]
static inline void SurviveObsErrorStateErrorModelFlip_jac_err(CnMat *Hx, const SurviveKalmanModel *_x0,
															  const SurviveKalmanErrorModel *err,
															  const SurviveAxisAnglePose *Z) {
	const FLT x0 = 0.5 * (*err).Pose.AxisAngleRot[1];
	const FLT x1 = 0.5 * (*err).Pose.AxisAngleRot[2];
	const FLT x2 = 0.5 * (*err).Pose.AxisAngleRot[0];
	const FLT x3 =
		(*_x0).Pose.Rot[1] + (x2 * (*_x0).Pose.Rot[0]) + (-1 * x0 * (*_x0).Pose.Rot[3]) + (x1 * (*_x0).Pose.Rot[2]);
	const FLT x4 =
		(x0 * (*_x0).Pose.Rot[1]) + (-1 * x2 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[3] + (x1 * (*_x0).Pose.Rot[0]);
	const FLT x5 = 1.0 * x4;
	const FLT x6 =
		(-1 * x1 * (*_x0).Pose.Rot[1]) + (x0 * (*_x0).Pose.Rot[0]) + (x2 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x7 = 1.0 * x6;
	const FLT x8 = 1.0 * x3;
	const FLT x9 = (x8 * (*_x0).Pose.Rot[0]) + (-1 * x5 * (*_x0).Pose.Rot[2]) + (x7 * (*_x0).Pose.Rot[3]);
	const FLT x10 = (-1 * x2 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[0] + (-1 * x1 * (*_x0).Pose.Rot[3]) +
					(-1 * x0 * (*_x0).Pose.Rot[2]);
	const FLT x11 = 1e-10 + (x6 * x6) + (x4 * x4) + (x3 * x3);
	const FLT x12 = sqrt(x11);
	const FLT x13 = 1. / x12;
	const FLT x14 = 1.0 / 2.0 * x13 * (1. / x10);
	const FLT x15 = x10 * x10;
	const FLT x16 = 0.5 * (1. / x15) * x12;
	const FLT x17 = 2 * (1. / (x11 + x15)) * x15 * x13;
	const FLT x18 = x17 * ((x16 * (*_x0).Pose.Rot[1]) + (x9 * x14));
	const FLT x19 = atan2(x12, x10);
	const FLT x20 = (1. / (x11 * sqrt(x11))) * x19;
	const FLT x21 = x9 * x20;
	const FLT x22 = x13 * x19;
	const FLT x23 = 1.0 * x22;
	const FLT x24 = x23 * (*_x0).Pose.Rot[0];
	const FLT x25 = x24 + (x3 * x18) + (-1 * x3 * x21);
	const FLT x26 = 2 * x22;
	const FLT x27 = (x4 * x26) + (-1 * (*Z).AxisAngleRot[2]);
	const FLT x28 = (x3 * x26) + (-1 * (*Z).AxisAngleRot[0]);
	const FLT x29 = (x6 * x26) + (-1 * (*Z).AxisAngleRot[1]);
	const FLT x30 = 1e-10 + (x29 * x29) + (x27 * x27) + (x28 * x28);
	const FLT x31 = sqrt(x30);
	const FLT x32 = -6.28318530717959 + x31;
	const FLT x33 = x32 * (1. / x31);
	const FLT x34 = x23 * (*_x0).Pose.Rot[3];
	const FLT x35 = (-1 * x6 * x21) + x34 + (x6 * x18);
	const FLT x36 = 2 * x29;
	const FLT x37 = x23 * (*_x0).Pose.Rot[2];
	const FLT x38 = (-1 * x37) + (-1 * x4 * x21) + (x4 * x18);
	const FLT x39 = 2 * x27;
	const FLT x40 = 2 * x28;
	const FLT x41 = (x36 * x35) + (x40 * x25) + (x38 * x39);
	const FLT x42 = 1.0 / 2.0 * (1. / x30);
	const FLT x43 = x42 * x28;
	const FLT x44 = 1.0 / 2.0 * (1. / (x30 * sqrt(x30))) * x32;
	const FLT x45 = x41 * x44;
	const FLT x46 = (x7 * (*_x0).Pose.Rot[0]) + (-1 * x8 * (*_x0).Pose.Rot[3]) + (x5 * (*_x0).Pose.Rot[1]);
	const FLT x47 = x17 * ((x16 * (*_x0).Pose.Rot[2]) + (x46 * x14));
	const FLT x48 = x46 * x20;
	const FLT x49 = (-1 * x34) + (x3 * x47) + (-1 * x3 * x48);
	const FLT x50 = (x6 * x47) + x24 + (-1 * x6 * x48);
	const FLT x51 = x23 * (*_x0).Pose.Rot[1];
	const FLT x52 = x51 + (x4 * x47) + (-1 * x4 * x48);
	const FLT x53 = (x40 * x49) + (x50 * x36) + (x52 * x39);
	const FLT x54 = x53 * x44;
	const FLT x55 = (-1 * x7 * (*_x0).Pose.Rot[1]) + (x8 * (*_x0).Pose.Rot[2]) + (x5 * (*_x0).Pose.Rot[0]);
	const FLT x56 = x17 * ((x16 * (*_x0).Pose.Rot[3]) + (x55 * x14));
	const FLT x57 = x55 * x20;
	const FLT x58 = x37 + (x3 * x56) + (-1 * x3 * x57);
	const FLT x59 = (-1 * x51) + (x6 * x56) + (-1 * x6 * x57);
	const FLT x60 = (x4 * x56) + x24 + (-1 * x4 * x57);
	const FLT x61 = (x58 * x40) + (x59 * x36) + (x60 * x39);
	const FLT x62 = x61 * x42;
	const FLT x63 = x61 * x44;
	const FLT x64 = x42 * x29;
	const FLT x65 = x42 * x27;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						(-1 * x45 * x28) + (x33 * x25) + (x41 * x43));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						(-1 * x54 * x28) + (x49 * x33) + (x53 * x43));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						(-1 * x63 * x28) + (x58 * x33) + (x62 * x28));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						(-1 * x45 * x29) + (x64 * x41) + (x33 * x35));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						(x50 * x33) + (-1 * x54 * x29) + (x64 * x53));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						(x59 * x33) + (-1 * x63 * x29) + (x62 * x29));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						(-1 * x45 * x27) + (x33 * x38) + (x65 * x41));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						(-1 * x54 * x27) + (x52 * x33) + (x65 * x53));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						(-1 * x63 * x27) + (x60 * x33) + (x62 * x27));
}

// Full version Jacobian of SurviveObsErrorStateErrorModelFlip wrt [(*err).AccBias[0], (*err).AccBias[1],
// (*err).AccBias[2], (*err).Acc[0], (*err).Acc[1], (*err).Acc[2], (*err).GyroBias[0], (*err).GyroBias[1],
// (*err).GyroBias[2], (*err).IMUCorrection[0], (*err).IMUCorrection[1], (*err).IMUCorrection[2],
// (*err).IMUCorrection[3], (*err).Pose.AxisAngleRot[0], (*err).Pose.AxisAngleRot[1], (*err).Pose.AxisAngleRot[2],
// (*err).Pose.Pos[0], (*err).Pose.Pos[1], (*err).Pose.Pos[2], (*err).Velocity.AxisAngleRot[0],
// (*err).Velocity.AxisAngleRot[1], (*err).Velocity.AxisAngleRot[2], (*err).Velocity.Pos[0], (*err).Velocity.Pos[1],
// (*err).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f31e20>] Jacobian of
// SurviveObsErrorStateErrorModelFlip wrt [(*Z).AxisAngleRot[0], (*Z).AxisAngleRot[1], (*Z).AxisAngleRot[2],
// (*Z).Pos[0], (*Z).Pos[1], (*Z).Pos[2]]
static inline void SurviveObsErrorStateErrorModelFlip_jac_Z(CnMat *Hx, const SurviveKalmanModel *_x0,
															const SurviveKalmanErrorModel *err,
															const SurviveAxisAnglePose *Z) {
	const FLT x0 = 0.5 * (*err).Pose.AxisAngleRot[2];
	const FLT x1 = 0.5 * (*err).Pose.AxisAngleRot[0];
	const FLT x2 = 0.5 * (*err).Pose.AxisAngleRot[1];
	const FLT x3 =
		(x2 * (*_x0).Pose.Rot[1]) + (-1 * x1 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[3] + (x0 * (*_x0).Pose.Rot[0]);
	const FLT x4 =
		(*_x0).Pose.Rot[1] + (x1 * (*_x0).Pose.Rot[0]) + (-1 * x2 * (*_x0).Pose.Rot[3]) + (x0 * (*_x0).Pose.Rot[2]);
	const FLT x5 =
		(-1 * x0 * (*_x0).Pose.Rot[1]) + (x2 * (*_x0).Pose.Rot[0]) + (x1 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x6 = sqrt(1e-10 + (x5 * x5) + (x3 * x3) + (x4 * x4));
	const FLT x7 = 2 * (1. / x6) *
				   atan2(x6, (-1 * x1 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[0] + (-1 * x0 * (*_x0).Pose.Rot[3]) +
								 (-1 * x2 * (*_x0).Pose.Rot[2]));
	const FLT x8 = (x3 * x7) + (-1 * (*Z).AxisAngleRot[2]);
	const FLT x9 = x8 * x8;
	const FLT x10 = (x4 * x7) + (-1 * (*Z).AxisAngleRot[0]);
	const FLT x11 = x10 * x10;
	const FLT x12 = (x5 * x7) + (-1 * (*Z).AxisAngleRot[1]);
	const FLT x13 = x12 * x12;
	const FLT x14 = 1e-10 + x13 + x9 + x11;
	const FLT x15 = sqrt(x14);
	const FLT x16 = -6.28318530717959 + x15;
	const FLT x17 = -1 * (1. / x15) * x16;
	const FLT x18 = 1. / x14;
	const FLT x19 = (1. / (x14 * sqrt(x14))) * x16;
	const FLT x20 = x12 * x10;
	const FLT x21 = (-1 * x20 * x18) + (x20 * x19);
	const FLT x22 = x8 * x19;
	const FLT x23 = x8 * x18;
	const FLT x24 = (-1 * x23 * x10) + (x22 * x10);
	const FLT x25 = (-1 * x23 * x12) + (x22 * x12);
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[0]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, Pos[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[1]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, Pos[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, Pos[2]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, Pos[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						(x11 * x19) + x17 + (-1 * x11 * x18));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT), x21);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT), x24);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT), x21);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						(x13 * x19) + x17 + (-1 * x13 * x18));
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT), x25);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[0]) / sizeof(FLT), x24);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[1]) / sizeof(FLT), x25);
	cnMatrixOptionalSet(Hx, offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveAxisAnglePose, AxisAngleRot[2]) / sizeof(FLT),
						(x9 * x19) + x17 + (-1 * x9 * x18));
}

// Full version Jacobian of SurviveObsErrorStateErrorModelFlip wrt [(*Z).AxisAngleRot[0], (*Z).AxisAngleRot[1],
// (*Z).AxisAngleRot[2], (*Z).Pos[0], (*Z).Pos[1], (*Z).Pos[2]]
static inline FLT SurviveKalmanModel_LightMeas_x_gen1(const FLT dt, const SurviveKalmanModel *_x0, const FLT *sensor_pt,
													  const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const FLT x0 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x1 = dt * dt;
	const FLT x2 = x1 * ((*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x3 = x1 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x4 = x1 * ((*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x5 = 1e-10 + x2 + x3 + x4;
	const FLT x6 = sqrt(x5);
	const FLT x7 = 0.5 * x6;
	const FLT x8 = sin(x7);
	const FLT x9 = (1. / x5) * (x8 * x8);
	const FLT x10 = cos(x7);
	const FLT x11 = 1. / sqrt((x4 * x9) + (x3 * x9) + (x2 * x9) + (x10 * x10));
	const FLT x12 = (1. / x6) * x8 * dt * x11;
	const FLT x13 = x12 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x14 = x11 * x10;
	const FLT x15 = x12 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x16 = x12 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x17 = (-1 * x16 * (*_x0).Pose.Rot[3]) + (x15 * (*_x0).Pose.Rot[1]) + (x13 * (*_x0).Pose.Rot[0]) +
					(x14 * (*_x0).Pose.Rot[2]);
	const FLT x18 = (-1 * x16 * (*_x0).Pose.Rot[1]) + (x14 * (*_x0).Pose.Rot[0]) + (-1 * x15 * (*_x0).Pose.Rot[3]) +
					(-1 * x13 * (*_x0).Pose.Rot[2]);
	const FLT x19 = (x14 * (*_x0).Pose.Rot[3]) + (x15 * (*_x0).Pose.Rot[0]) + (x16 * (*_x0).Pose.Rot[2]) +
					(-1 * x13 * (*_x0).Pose.Rot[1]);
	const FLT x20 = (-1 * x19 * sensor_pt[1]) + (x17 * sensor_pt[2]) + (x18 * sensor_pt[0]);
	const FLT x21 = (x16 * (*_x0).Pose.Rot[0]) + (x14 * (*_x0).Pose.Rot[1]) + (-1 * x15 * (*_x0).Pose.Rot[2]) +
					(x13 * (*_x0).Pose.Rot[3]);
	const FLT x22 = (x19 * sensor_pt[0]) + (-1 * x21 * sensor_pt[2]) + (x18 * sensor_pt[1]);
	const FLT x23 = (2 * ((x22 * x21) + (-1 * x20 * x17))) + sensor_pt[2] + (x0 * (*_x0).Acc[2]) +
					(dt * (*_x0).Velocity.Pos[2]) + (*_x0).Pose.Pos[2];
	const FLT x24 = (-1 * x17 * sensor_pt[0]) + (x18 * sensor_pt[2]) + (x21 * sensor_pt[1]);
	const FLT x25 = (2 * ((x20 * x19) + (-1 * x24 * x21))) + (dt * (*_x0).Velocity.Pos[1]) + sensor_pt[1] +
					(*_x0).Pose.Pos[1] + (x0 * (*_x0).Acc[1]);
	const FLT x26 = (*_x0).Pose.Pos[0] + (2 * ((x24 * x17) + (-1 * x22 * x19))) + sensor_pt[0] +
					(dt * (*_x0).Velocity.Pos[0]) + (x0 * (*_x0).Acc[0]);
	const FLT x27 = (-1 * x26 * (*lh_p).Rot[2]) + (x23 * (*lh_p).Rot[0]) + (x25 * (*lh_p).Rot[1]);
	const FLT x28 = (-1 * x25 * (*lh_p).Rot[3]) + (x23 * (*lh_p).Rot[2]) + (x26 * (*lh_p).Rot[0]);
	const FLT x29 = x25 + (*lh_p).Pos[1] + (2 * ((x28 * (*lh_p).Rot[3]) + (-1 * x27 * (*lh_p).Rot[1])));
	const FLT x30 = (-1 * x23 * (*lh_p).Rot[1]) + (x25 * (*lh_p).Rot[0]) + (x26 * (*lh_p).Rot[3]);
	const FLT x31 = x23 + (2 * ((x30 * (*lh_p).Rot[1]) + (-1 * x28 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x32 = x26 + (2 * ((x27 * (*lh_p).Rot[2]) + (-1 * x30 * (*lh_p).Rot[3]))) + (*lh_p).Pos[0];
	const FLT x33 = -1 * x31;
	const FLT x34 = (-1 * (*bsc0).phase) + (-1 * asin((1. / sqrt((x32 * x32) + (x31 * x31))) * x29 * (*bsc0).tilt)) +
					(-1 * atan2(x32, x33));
	return x34 + (-1 * cos(1.5707963267949 + x34 + (*bsc0).gibpha) * (*bsc0).gibmag) +
		   ((atan2(x29, x33) * atan2(x29, x33)) * (*bsc0).curve);
}

// Jacobian of SurviveKalmanModel_LightMeas_x_gen1 wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2],
// (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3],
// (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1],
// (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1],
// (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4f45a60>]
static inline void SurviveKalmanModel_LightMeas_x_gen1_jac_x0(CnMat *Hx, const FLT dt, const SurviveKalmanModel *_x0,
															  const FLT *sensor_pt, const SurvivePose *lh_p,
															  const BaseStationCal *bsc0) {
	const FLT x0 = dt * fabs(dt);
	const FLT x1 = x0 * (*lh_p).Rot[1];
	const FLT x2 = x1 * (*lh_p).Rot[2];
	const FLT x3 = x0 * (*lh_p).Rot[3];
	const FLT x4 = x3 * (*lh_p).Rot[0];
	const FLT x5 = x4 + x2;
	const FLT x6 = 1.0 / 2.0 * x0;
	const FLT x7 = dt * dt;
	const FLT x8 = (*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x9 = x8 * x7;
	const FLT x10 = (*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x11 = x7 * x10;
	const FLT x12 = (*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x13 = x7 * x12;
	const FLT x14 = 1e-10 + x9 + x11 + x13;
	const FLT x15 = sqrt(x14);
	const FLT x16 = 0.5 * x15;
	const FLT x17 = sin(x16);
	const FLT x18 = x17 * x17;
	const FLT x19 = 1. / x14;
	const FLT x20 = x19 * x18;
	const FLT x21 = cos(x16);
	const FLT x22 = (x20 * x13) + (x20 * x11) + (x9 * x20) + (x21 * x21);
	const FLT x23 = 1. / sqrt(x22);
	const FLT x24 = (1. / x15) * x17;
	const FLT x25 = dt * x24;
	const FLT x26 = x25 * x23;
	const FLT x27 = x26 * (*_x0).Pose.Rot[0];
	const FLT x28 = x23 * x21;
	const FLT x29 = x28 * (*_x0).Pose.Rot[2];
	const FLT x30 = x26 * (*_x0).Pose.Rot[1];
	const FLT x31 = x23 * (*_x0).Pose.Rot[3];
	const FLT x32 = x31 * x25;
	const FLT x33 = (-1 * x32 * (*_x0).Velocity.AxisAngleRot[0]) + (x30 * (*_x0).Velocity.AxisAngleRot[2]) +
					(x27 * (*_x0).Velocity.AxisAngleRot[1]) + x29;
	const FLT x34 = x26 * (*_x0).Pose.Rot[2];
	const FLT x35 = x28 * (*_x0).Pose.Rot[0];
	const FLT x36 = (-1 * x30 * (*_x0).Velocity.AxisAngleRot[0]) + x35 + (-1 * x32 * (*_x0).Velocity.AxisAngleRot[2]) +
					(-1 * x34 * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x37 = x28 * (*_x0).Pose.Rot[3];
	const FLT x38 = (x34 * (*_x0).Velocity.AxisAngleRot[0]) + x37 + (x27 * (*_x0).Velocity.AxisAngleRot[2]) +
					(-1 * x30 * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x39 = (-1 * x38 * sensor_pt[1]) + (x33 * sensor_pt[2]) + (x36 * sensor_pt[0]);
	const FLT x40 = x28 * (*_x0).Pose.Rot[1];
	const FLT x41 = (x27 * (*_x0).Velocity.AxisAngleRot[0]) + x40 + (-1 * x34 * (*_x0).Velocity.AxisAngleRot[2]) +
					(x32 * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x42 = (-1 * x41 * sensor_pt[2]) + (x38 * sensor_pt[0]) + (x36 * sensor_pt[1]);
	const FLT x43 = (2 * ((x41 * x42) + (-1 * x33 * x39))) + sensor_pt[2] + (x6 * (*_x0).Acc[2]) +
					(dt * (*_x0).Velocity.Pos[2]) + (*_x0).Pose.Pos[2];
	const FLT x44 = (-1 * x33 * sensor_pt[0]) + (x36 * sensor_pt[2]) + (x41 * sensor_pt[1]);
	const FLT x45 = (2 * ((x44 * x33) + (-1 * x42 * x38))) + (*_x0).Pose.Pos[0] + sensor_pt[0] +
					(dt * (*_x0).Velocity.Pos[0]) + (x6 * (*_x0).Acc[0]);
	const FLT x46 = sensor_pt[1] + (2 * ((x38 * x39) + (-1 * x41 * x44))) + (*_x0).Pose.Pos[1] +
					(dt * (*_x0).Velocity.Pos[1]) + (x6 * (*_x0).Acc[1]);
	const FLT x47 = (-1 * x46 * (*lh_p).Rot[3]) + (x43 * (*lh_p).Rot[2]) + (x45 * (*lh_p).Rot[0]);
	const FLT x48 = (x46 * (*lh_p).Rot[0]) + (-1 * x43 * (*lh_p).Rot[1]) + (x45 * (*lh_p).Rot[3]);
	const FLT x49 = (2 * ((x48 * (*lh_p).Rot[1]) + (-1 * x47 * (*lh_p).Rot[2]))) + x43 + (*lh_p).Pos[2];
	const FLT x50 = 1. / x49;
	const FLT x51 = x0 * (*lh_p).Rot[2] * (*lh_p).Rot[0];
	const FLT x52 = x1 * (*lh_p).Rot[3];
	const FLT x53 = x52 + (-1 * x51);
	const FLT x54 = x49 * x49;
	const FLT x55 = 1. / x54;
	const FLT x56 = (x43 * (*lh_p).Rot[0]) + (-1 * x45 * (*lh_p).Rot[2]) + (x46 * (*lh_p).Rot[1]);
	const FLT x57 = (*lh_p).Pos[1] + x46 + (2 * ((x47 * (*lh_p).Rot[3]) + (-1 * x56 * (*lh_p).Rot[1])));
	const FLT x58 = x57 * x55;
	const FLT x59 = x57 * x57;
	const FLT x60 = -1 * x49;
	const FLT x61 = 2 * (1. / (x54 + x59)) * x54 * atan2(x57, x60) * (*bsc0).curve;
	const FLT x62 = (*lh_p).Rot[2] * (*lh_p).Rot[2];
	const FLT x63 = -1 * x0 * x62;
	const FLT x64 = (*lh_p).Rot[3] * (*lh_p).Rot[3];
	const FLT x65 = -1 * x0 * x64;
	const FLT x66 = x65 + x6 + x63;
	const FLT x67 = x45 + (2 * ((x56 * (*lh_p).Rot[2]) + (-1 * x48 * (*lh_p).Rot[3]))) + (*lh_p).Pos[0];
	const FLT x68 = x67 * x55;
	const FLT x69 = x54 + (x67 * x67);
	const FLT x70 = 1. / x69;
	const FLT x71 = x70 * x54;
	const FLT x72 = 1. / sqrt(1 + (-1 * x70 * x59 * ((*bsc0).tilt * (*bsc0).tilt)));
	const FLT x73 = 2 * x67;
	const FLT x74 = 2 * x49;
	const FLT x75 = 1.0 / 2.0 * (1. / (x69 * sqrt(x69))) * x57 * (*bsc0).tilt;
	const FLT x76 = (1. / sqrt(x69)) * (*bsc0).tilt;
	const FLT x77 = (-1 * x72 * ((x5 * x76) + (-1 * ((x74 * x53) + (x73 * x66)) * x75))) +
					(-1 * ((x68 * x53) + (-1 * x66 * x50)) * x71);
	const FLT x78 = (*bsc0).gibmag * sin(1.5707963267949 + (-1 * asin(x76 * x57)) + (-1 * (*bsc0).phase) +
										 (*bsc0).gibpha + (-1 * atan2(x67, x60)));
	const FLT x79 = (*lh_p).Rot[1] * (*lh_p).Rot[1];
	const FLT x80 = (-1 * x0 * x79) + x6;
	const FLT x81 = x80 + x65;
	const FLT x82 = x3 * (*lh_p).Rot[2];
	const FLT x83 = x1 * (*lh_p).Rot[0];
	const FLT x84 = x83 + x82;
	const FLT x85 = x2 + (-1 * x4);
	const FLT x86 = (-1 * x72 * ((x81 * x76) + (-1 * ((x84 * x74) + (x85 * x73)) * x75))) +
					(-1 * ((x84 * x68) + (-1 * x85 * x50)) * x71);
	const FLT x87 = x82 + (-1 * x83);
	const FLT x88 = x80 + x63;
	const FLT x89 = x51 + x52;
	const FLT x90 = (-1 * x72 * ((x87 * x76) + (-1 * ((x88 * x74) + (x89 * x73)) * x75))) +
					(-1 * ((x88 * x68) + (-1 * x89 * x50)) * x71);
	const FLT x91 = 2 * (*lh_p).Rot[1];
	const FLT x92 = x91 * (*lh_p).Rot[2];
	const FLT x93 = 2 * (*lh_p).Rot[3];
	const FLT x94 = x93 * (*lh_p).Rot[0];
	const FLT x95 = x94 + x92;
	const FLT x96 = 2 * (*lh_p).Rot[2];
	const FLT x97 = x96 * (*lh_p).Rot[0];
	const FLT x98 = x91 * (*lh_p).Rot[3];
	const FLT x99 = x98 + (-1 * x97);
	const FLT x100 = 2 * x62;
	const FLT x101 = -1 * x100;
	const FLT x102 = 2 * x64;
	const FLT x103 = 1 + (-1 * x102);
	const FLT x104 = x103 + x101;
	const FLT x105 = (-1 * x72 * ((x76 * x95) + (-1 * x75 * ((x74 * x99) + (x73 * x104))))) +
					 (-1 * x71 * ((x68 * x99) + (-1 * x50 * x104)));
	const FLT x106 = 2 * x79;
	const FLT x107 = -1 * x106;
	const FLT x108 = x103 + x107;
	const FLT x109 = x93 * (*lh_p).Rot[2];
	const FLT x110 = x91 * (*lh_p).Rot[0];
	const FLT x111 = x110 + x109;
	const FLT x112 = x92 + (-1 * x94);
	const FLT x113 = (-1 * x72 * ((x76 * x108) + (-1 * ((x74 * x111) + (x73 * x112)) * x75))) +
					 (-1 * ((x68 * x111) + (-1 * x50 * x112)) * x71);
	const FLT x114 = x109 + (-1 * x110);
	const FLT x115 = 1 + x107 + x101;
	const FLT x116 = x97 + x98;
	const FLT x117 = (-1 * x72 * ((x76 * x114) + (-1 * ((x74 * x115) + (x73 * x116)) * x75))) +
					 (-1 * ((x68 * x115) + (-1 * x50 * x116)) * x71);
	const FLT x118 = x26 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x119 = -1 * x118 * sensor_pt[2];
	const FLT x120 = x26 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x121 = x120 * sensor_pt[0];
	const FLT x122 = x28 * sensor_pt[1];
	const FLT x123 = x122 + x119 + x121;
	const FLT x124 = 2 * x38;
	const FLT x125 = 2 * x42;
	const FLT x126 = -1 * x120 * x125;
	const FLT x127 = x118 * sensor_pt[1];
	const FLT x128 = x26 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x129 = -1 * x128 * sensor_pt[0];
	const FLT x130 = x28 * sensor_pt[2];
	const FLT x131 = x130 + x129;
	const FLT x132 = x131 + x127;
	const FLT x133 = 2 * x33;
	const FLT x134 = 2 * x44;
	const FLT x135 = x128 * x134;
	const FLT x136 = x135 + (-1 * x124 * x123) + (x133 * x132) + x126;
	const FLT x137 = x128 * sensor_pt[2];
	const FLT x138 = x28 * sensor_pt[0];
	const FLT x139 = -1 * x120 * sensor_pt[1];
	const FLT x140 = x139 + x138;
	const FLT x141 = x140 + x137;
	const FLT x142 = x118 * x125;
	const FLT x143 = 2 * x39;
	const FLT x144 = -1 * x128 * x143;
	const FLT x145 = 2 * x41;
	const FLT x146 = x144 + (-1 * x133 * x141) + (x123 * x145) + x142;
	const FLT x147 = x120 * x143;
	const FLT x148 = -1 * x118 * x134;
	const FLT x149 = x148 + x147 + (-1 * x132 * x145) + (x124 * x141);
	const FLT x150 = (x149 * (*lh_p).Rot[1]) + (-1 * x136 * (*lh_p).Rot[2]) + (x146 * (*lh_p).Rot[0]);
	const FLT x151 = (-1 * x149 * (*lh_p).Rot[3]) + (x136 * (*lh_p).Rot[0]) + (x146 * (*lh_p).Rot[2]);
	const FLT x152 = x149 + (-1 * x91 * x150) + (x93 * x151);
	const FLT x153 = (x136 * (*lh_p).Rot[3]) + (-1 * x146 * (*lh_p).Rot[1]) + (x149 * (*lh_p).Rot[0]);
	const FLT x154 = x146 + (-1 * x96 * x151) + (x91 * x153);
	const FLT x155 = x136 + (-1 * x93 * x153) + (x96 * x150);
	const FLT x156 = (-1 * x72 * ((x76 * x152) + (-1 * ((x74 * x154) + (x73 * x155)) * x75))) +
					 (-1 * ((x68 * x154) + (-1 * x50 * x155)) * x71);
	const FLT x157 = x120 * x134;
	const FLT x158 = -1 * x127;
	const FLT x159 = x158 + (-1 * x130) + x129;
	const FLT x160 = x128 * x125;
	const FLT x161 = (-1 * x121) + x119;
	const FLT x162 = x161 + x122;
	const FLT x163 = (x162 * x133) + x160 + x157 + (-1 * x124 * x159);
	const FLT x164 = x128 * sensor_pt[1];
	const FLT x165 = x120 * sensor_pt[2];
	const FLT x166 = x118 * sensor_pt[0];
	const FLT x167 = (-1 * x166) + x164 + x165;
	const FLT x168 = x28 * x125;
	const FLT x169 = (-1 * x147) + x168 + (-1 * x167 * x133) + (x145 * x159);
	const FLT x170 = x28 * x134;
	const FLT x171 = x144 + (-1 * x170) + (-1 * x162 * x145) + (x124 * x167);
	const FLT x172 = (x171 * (*lh_p).Rot[1]) + (-1 * x163 * (*lh_p).Rot[2]) + (x169 * (*lh_p).Rot[0]);
	const FLT x173 = 2 * ((-1 * x171 * (*lh_p).Rot[3]) + (x163 * (*lh_p).Rot[0]) + (x169 * (*lh_p).Rot[2]));
	const FLT x174 = x171 + (-1 * x91 * x172) + (x173 * (*lh_p).Rot[3]);
	const FLT x175 = (x163 * (*lh_p).Rot[3]) + (-1 * x169 * (*lh_p).Rot[1]) + (x171 * (*lh_p).Rot[0]);
	const FLT x176 = x169 + (-1 * x173 * (*lh_p).Rot[2]) + (x91 * x175);
	const FLT x177 = x163 + (-1 * x93 * x175) + (x96 * x172);
	const FLT x178 = (-1 * x72 * ((x76 * x174) + (-1 * ((x74 * x176) + (x73 * x177)) * x75))) +
					 (-1 * ((x68 * x176) + (-1 * x50 * x177)) * x71);
	const FLT x179 = -1 * x137;
	const FLT x180 = x139 + (-1 * x138) + x179;
	const FLT x181 = x131 + x158;
	const FLT x182 = x118 * x143;
	const FLT x183 = x182 + (x124 * x181) + (-1 * x180 * x145) + x157;
	const FLT x184 = x28 * x143;
	const FLT x185 = 2 * ((-1 * x164) + x165 + x166);
	const FLT x186 = x126 + (x41 * x185) + (-1 * x181 * x133) + (-1 * x184);
	const FLT x187 = (x180 * x133) + x170 + (-1 * x142) + (-1 * x38 * x185);
	const FLT x188 = (x187 * (*lh_p).Rot[3]) + (x183 * (*lh_p).Rot[0]) + (-1 * x186 * (*lh_p).Rot[1]);
	const FLT x189 = (x183 * (*lh_p).Rot[1]) + (x186 * (*lh_p).Rot[0]) + (-1 * x187 * (*lh_p).Rot[2]);
	const FLT x190 = (-1 * x93 * x188) + x187 + (x96 * x189);
	const FLT x191 = (x187 * (*lh_p).Rot[0]) + (-1 * x183 * (*lh_p).Rot[3]) + (x186 * (*lh_p).Rot[2]);
	const FLT x192 = x186 + (-1 * x96 * x191) + (x91 * x188);
	const FLT x193 = (-1 * x91 * x189) + x183 + (x93 * x191);
	const FLT x194 = (-1 * x72 * ((x76 * x193) + (-1 * ((x74 * x192) + (x73 * x190)) * x75))) +
					 (-1 * ((x68 * x192) + (-1 * x50 * x190)) * x71);
	const FLT x195 = x161 + (-1 * x122);
	const FLT x196 = x140 + x179;
	const FLT x197 = x160 + (x196 * x145) + (-1 * x195 * x133) + x182;
	const FLT x198 = 2 * (x164 + x166 + (-1 * x165));
	const FLT x199 = (-1 * x135) + x184 + (-1 * x41 * x198) + (x124 * x195);
	const FLT x200 = (-1 * x168) + x148 + (x33 * x198) + (-1 * x124 * x196);
	const FLT x201 = (-1 * x197 * (*lh_p).Rot[1]) + (x200 * (*lh_p).Rot[3]) + (x199 * (*lh_p).Rot[0]);
	const FLT x202 = (x199 * (*lh_p).Rot[1]) + (-1 * x200 * (*lh_p).Rot[2]) + (x197 * (*lh_p).Rot[0]);
	const FLT x203 = (-1 * x93 * x201) + x200 + (x96 * x202);
	const FLT x204 = (x200 * (*lh_p).Rot[0]) + (-1 * x199 * (*lh_p).Rot[3]) + (x197 * (*lh_p).Rot[2]);
	const FLT x205 = x197 + (-1 * x96 * x204) + (x91 * x201);
	const FLT x206 = x199 + (x93 * x204) + (-1 * x91 * x202);
	const FLT x207 = (-1 * x72 * ((x76 * x206) + (-1 * ((x74 * x205) + (x73 * x203)) * x75))) +
					 (-1 * ((x68 * x205) + (-1 * x50 * x203)) * x71);
	const FLT x208 = -1 * x32;
	const FLT x209 = x23 * (*_x0).Pose.Rot[2];
	const FLT x210 = x7 * x24;
	const FLT x211 = 0.5 * x210;
	const FLT x212 = x211 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x213 = 0.5 * x19;
	const FLT x214 = dt * dt * dt;
	const FLT x215 = x12 * x214;
	const FLT x216 = x213 * x215;
	const FLT x217 = 1.0 / 2.0 * (1. / (x22 * sqrt(x22)));
	const FLT x218 = x25 * x217;
	const FLT x219 = x218 * (*_x0).Pose.Rot[3];
	const FLT x220 = dt * dt * dt * dt;
	const FLT x221 =
		x220 * ((*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x222 = (1. / (x14 * x14)) * x18;
	const FLT x223 = 2 * x222;
	const FLT x224 = 1.0 * x21;
	const FLT x225 = (1. / (x14 * sqrt(x14))) * x17;
	const FLT x226 = x224 * x225;
	const FLT x227 = x220 * x226;
	const FLT x228 = 2 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x229 = x222 * x228;
	const FLT x230 = x8 * x220;
	const FLT x231 = x210 * x224;
	const FLT x232 = x7 * x20;
	const FLT x233 = x10 * x220;
	const FLT x234 = x233 * x226;
	const FLT x235 = (-1 * x233 * x229) + (x234 * (*_x0).Velocity.AxisAngleRot[0]) + (-1 * x223 * x221) +
					 (x232 * x228) + (x8 * x227 * (*_x0).Velocity.AxisAngleRot[0]) + (-1 * x230 * x229) +
					 (x221 * x226) + (-1 * x231 * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x236 = x235 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x237 = x21 * x217;
	const FLT x238 = x237 * x235;
	const FLT x239 = x235 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x240 = x218 * x239;
	const FLT x241 = x23 * (*_x0).Pose.Rot[1];
	const FLT x242 = x214 * x225;
	const FLT x243 = x242 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x244 = x243 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x245 = x241 * x244;
	const FLT x246 = x214 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x247 = x213 * x246 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x248 = x40 * x247;
	const FLT x249 = x218 * (*_x0).Pose.Rot[1];
	const FLT x250 = x235 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x251 = x12 * x242;
	const FLT x252 = x214 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x253 = x35 * x213;
	const FLT x254 = x253 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x255 = x23 * (*_x0).Pose.Rot[0];
	const FLT x256 = x255 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x257 = x242 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x258 = (-1 * x256 * x257) + (x252 * x254);
	const FLT x259 = (x31 * x251) + x258 + (-1 * x250 * x249) + x248 + (-1 * x245) + (-1 * x212 * x209) +
					 (-1 * x240 * (*_x0).Pose.Rot[0]) + (-1 * x37 * x216) + (x219 * x236) + x208 +
					 (-1 * x238 * (*_x0).Pose.Rot[2]);
	const FLT x260 = x213 * x252;
	const FLT x261 = x260 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x262 = x37 * x261;
	const FLT x263 = x257 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x264 = x31 * x263;
	const FLT x265 = x29 * x247;
	const FLT x266 = x209 * x244;
	const FLT x267 = x218 * (*_x0).Pose.Rot[2];
	const FLT x268 = x218 * (*_x0).Pose.Rot[0];
	const FLT x269 = (-1 * x236 * x268) + x27 + (-1 * x264) + (x215 * x253) + (-1 * x238 * (*_x0).Pose.Rot[1]) + x266 +
					 (-1 * x219 * x239) + x262 + (-1 * x212 * x241) + (-1 * x265) + (x267 * x250) + (-1 * x251 * x255);
	const FLT x270 = x31 * x211;
	const FLT x271 = x255 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x272 = (x254 * x246) + (-1 * x271 * x243);
	const FLT x273 = (-1 * x40 * x261) + (x263 * x241);
	const FLT x274 = x272 + x273 + (-1 * x236 * x267) + (x29 * x216) + (-1 * x270 * (*_x0).Velocity.AxisAngleRot[0]) +
					 x34 + (-1 * x209 * x251) + (-1 * x238 * (*_x0).Pose.Rot[3]) + (-1 * x268 * x250) + (x239 * x249);
	const FLT x275 = -1 * x30;
	const FLT x276 = x29 * x260;
	const FLT x277 = x276 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x278 = x209 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x279 = x278 * x257;
	const FLT x280 = (x31 * x244) + (-1 * x37 * x247);
	const FLT x281 = x280 + (x251 * x241) + (-1 * x40 * x216) + (-1 * x211 * x271) + (x219 * x250) + (-1 * x277) +
					 (x240 * (*_x0).Pose.Rot[2]) + x275 + (-1 * x238 * (*_x0).Pose.Rot[0]) + (x236 * x249) + x279;
	const FLT x282 = (x281 * sensor_pt[1]) + (-1 * x269 * sensor_pt[2]) + (x274 * sensor_pt[0]);
	const FLT x283 = (x269 * sensor_pt[1]) + (-1 * x259 * sensor_pt[0]) + (x281 * sensor_pt[2]);
	const FLT x284 = (x283 * x133) + (-1 * x274 * x125) + (x259 * x134) + (-1 * x282 * x124);
	const FLT x285 = (-1 * x274 * sensor_pt[1]) + (x281 * sensor_pt[0]) + (x259 * sensor_pt[2]);
	const FLT x286 = 2 * x269;
	const FLT x287 = (x282 * x145) + (-1 * x285 * x133) + (x42 * x286) + (-1 * x259 * x143);
	const FLT x288 = (x285 * x124) + (-1 * x44 * x286) + (-1 * x283 * x145) + (x274 * x143);
	const FLT x289 = (x288 * (*lh_p).Rot[1]) + (-1 * x284 * (*lh_p).Rot[2]) + (x287 * (*lh_p).Rot[0]);
	const FLT x290 = (x284 * (*lh_p).Rot[0]) + (x287 * (*lh_p).Rot[2]) + (-1 * x288 * (*lh_p).Rot[3]);
	const FLT x291 = x288 + (-1 * x91 * x289) + (x93 * x290);
	const FLT x292 = (x284 * (*lh_p).Rot[3]) + (-1 * x287 * (*lh_p).Rot[1]) + (x288 * (*lh_p).Rot[0]);
	const FLT x293 = (-1 * x96 * x290) + x287 + (x91 * x292);
	const FLT x294 = x284 + (-1 * x93 * x292) + (x96 * x289);
	const FLT x295 = (-1 * x72 * ((x76 * x291) + (-1 * ((x74 * x293) + (x73 * x294)) * x75))) +
					 (-1 * ((x68 * x293) + (-1 * x50 * x294)) * x71);
	const FLT x296 = x220 * x223;
	const FLT x297 = x296 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x298 = 2 * x232;
	const FLT x299 =
		(*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x300 = (x299 * x227) + (-1 * x10 * x297) + (-1 * x231 * (*_x0).Velocity.AxisAngleRot[1]) +
					 (x234 * (*_x0).Velocity.AxisAngleRot[1]) + (-1 * x12 * x297) +
					 (x298 * (*_x0).Velocity.AxisAngleRot[1]) + (x12 * x227 * (*_x0).Velocity.AxisAngleRot[1]) +
					 (-1 * x296 * x299);
	const FLT x301 = x300 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x302 = x218 * x301;
	const FLT x303 = x8 * x242;
	const FLT x304 = x300 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x305 = x300 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x306 = x8 * x214;
	const FLT x307 = x213 * x306;
	const FLT x308 = x237 * x300;
	const FLT x309 = x211 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x310 = (x278 * x243) + (-1 * x276 * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x311 = (-1 * x241 * x309) + x32 + (-1 * x31 * x303) + (x37 * x307) + (-1 * x302 * (*_x0).Pose.Rot[0]) +
					 (-1 * x308 * (*_x0).Pose.Rot[1]) + (-1 * x219 * x305) + x258 + x310 + (x267 * x304);
	const FLT x312 = (x252 * x253 * (*_x0).Velocity.AxisAngleRot[2]) + (-1 * x256 * x243);
	const FLT x313 = x277 + (-1 * x270 * (*_x0).Velocity.AxisAngleRot[1]) + (-1 * x279) + (-1 * x268 * x304) +
					 (x249 * x305) + x312 + (-1 * x302 * (*_x0).Pose.Rot[2]) + x275 + (-1 * x40 * x307) +
					 (x241 * x303) + (-1 * x308 * (*_x0).Pose.Rot[3]);
	const FLT x314 = -1 * x34;
	const FLT x315 = x260 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x316 = x37 * x315;
	const FLT x317 = x243 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x318 = x31 * x317;
	const FLT x319 = x273 + x318 + (-1 * x308 * (*_x0).Pose.Rot[0]) + (-1 * x211 * x256) + (x219 * x304) +
					 (x267 * x305) + x314 + (-1 * x316) + (x249 * x301) + (x209 * x303) + (-1 * x29 * x307);
	const FLT x320 = (x319 * sensor_pt[1]) + (-1 * x311 * sensor_pt[2]) + (x313 * sensor_pt[0]);
	const FLT x321 = x241 * x317;
	const FLT x322 = x40 * x315;
	const FLT x323 = x264 + x27 + (-1 * x308 * (*_x0).Pose.Rot[2]) + (x302 * (*_x0).Pose.Rot[3]) + (-1 * x262) +
					 (-1 * x255 * x303) + (-1 * x249 * x304) + (x253 * x306) + (-1 * x268 * x305) + (-1 * x209 * x309) +
					 (-1 * x321) + x322;
	const FLT x324 = (x311 * sensor_pt[1]) + (-1 * x323 * sensor_pt[0]) + (x319 * sensor_pt[2]);
	const FLT x325 = (-1 * x313 * x125) + (x323 * x134) + (-1 * x320 * x124) + (x324 * x133);
	const FLT x326 = (-1 * x313 * sensor_pt[1]) + (x319 * sensor_pt[0]) + (x323 * sensor_pt[2]);
	const FLT x327 = (x320 * x145) + (-1 * x323 * x143) + (-1 * x326 * x133) + (x311 * x125);
	const FLT x328 = (x313 * x143) + (x326 * x124) + (-1 * x324 * x145) + (-1 * x311 * x134);
	const FLT x329 = (x328 * (*lh_p).Rot[1]) + (-1 * x325 * (*lh_p).Rot[2]) + (x327 * (*lh_p).Rot[0]);
	const FLT x330 = (-1 * x328 * (*lh_p).Rot[3]) + (x325 * (*lh_p).Rot[0]) + (x327 * (*lh_p).Rot[2]);
	const FLT x331 = x328 + (-1 * x91 * x329) + (x93 * x330);
	const FLT x332 = (x325 * (*lh_p).Rot[3]) + (-1 * x327 * (*lh_p).Rot[1]) + (x328 * (*lh_p).Rot[0]);
	const FLT x333 = x327 + (-1 * x96 * x330) + (x91 * x332);
	const FLT x334 = (-1 * x93 * x332) + x325 + (x96 * x329);
	const FLT x335 = (-1 * x72 * ((x76 * x331) + (-1 * ((x74 * x333) + (x73 * x334)) * x75))) +
					 (-1 * ((x68 * x333) + (-1 * x50 * x334)) * x71);
	const FLT x336 = x296 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x337 = x224 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x338 = x225 * x337;
	const FLT x339 =
		(*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x340 = (-1 * x296 * x339) + (x298 * (*_x0).Velocity.AxisAngleRot[2]) + (x12 * x220 * x338) +
					 (x230 * x338) + (-1 * x8 * x336) + (-1 * x12 * x336) + (x227 * x339) + (-1 * x210 * x337);
	const FLT x341 = x340 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x342 = x10 * x214;
	const FLT x343 = x225 * x342;
	const FLT x344 = x23 * x343;
	const FLT x345 = x340 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x346 = x340 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x347 = x237 * x340;
	const FLT x348 = x211 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x349 = x213 * x342;
	const FLT x350 = x272 + (-1 * x29 * x349) + (-1 * x241 * x348) + (-1 * x268 * x346) + (-1 * x219 * x345) +
					 (x267 * x341) + (-1 * x318) + (-1 * x347 * (*_x0).Pose.Rot[1]) + x316 + x314 +
					 (x344 * (*_x0).Pose.Rot[2]);
	const FLT x351 = x267 * x340;
	const FLT x352 = (-1 * x266) + x27 + (-1 * x351 * (*_x0).Velocity.AxisAngleRot[0]) +
					 (-1 * x347 * (*_x0).Pose.Rot[3]) + (-1 * x268 * x341) + (-1 * x322) + x321 + (x253 * x342) +
					 (x249 * x345) + x265 + (-1 * x344 * (*_x0).Pose.Rot[0]) +
					 (-1 * x270 * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x353 = (-1 * x255 * x348) + (-1 * x347 * (*_x0).Pose.Rot[0]) + x310 + (x219 * x341) + (-1 * x248) +
					 (-1 * x37 * x349) + (x351 * (*_x0).Velocity.AxisAngleRot[1]) + (x249 * x346) + x245 + x208 +
					 (x31 * x343);
	const FLT x354 = (x353 * sensor_pt[1]) + (-1 * x350 * sensor_pt[2]) + (x352 * sensor_pt[0]);
	const FLT x355 = x280 + (-1 * x249 * x341) + (-1 * x241 * x343) + x312 + (-1 * x209 * x348) + (x219 * x346) +
					 (-1 * x268 * x345) + (-1 * x347 * (*_x0).Pose.Rot[2]) + x30 + (x40 * x349);
	const FLT x356 = (x350 * sensor_pt[1]) + (-1 * x355 * sensor_pt[0]) + (x353 * sensor_pt[2]);
	const FLT x357 = (x356 * x133) + (-1 * x354 * x124) + (-1 * x352 * x125) + (x355 * x134);
	const FLT x358 = (x353 * sensor_pt[0]) + (-1 * x352 * sensor_pt[1]) + (x355 * sensor_pt[2]);
	const FLT x359 = 2 * x350;
	const FLT x360 = (x42 * x359) + (-1 * x358 * x133) + (-1 * x355 * x143) + (x354 * x145);
	const FLT x361 = (-1 * x44 * x359) + (x358 * x124) + (-1 * x356 * x145) + (x352 * x143);
	const FLT x362 = (-1 * x357 * (*lh_p).Rot[2]) + (x361 * (*lh_p).Rot[1]) + (x360 * (*lh_p).Rot[0]);
	const FLT x363 = (-1 * x361 * (*lh_p).Rot[3]) + (x357 * (*lh_p).Rot[0]) + (x360 * (*lh_p).Rot[2]);
	const FLT x364 = x361 + (-1 * x91 * x362) + (x93 * x363);
	const FLT x365 = (x357 * (*lh_p).Rot[3]) + (-1 * x360 * (*lh_p).Rot[1]) + (x361 * (*lh_p).Rot[0]);
	const FLT x366 = x360 + (x91 * x365) + (-1 * x96 * x363);
	const FLT x367 = x357 + (-1 * x93 * x365) + (x96 * x362);
	const FLT x368 = (-1 * x72 * ((x76 * x364) + (-1 * ((x74 * x366) + (x73 * x367)) * x75))) +
					 (-1 * ((x68 * x366) + (-1 * x50 * x367)) * x71);
	const FLT x369 = dt * x92;
	const FLT x370 = dt * x94;
	const FLT x371 = x370 + x369;
	const FLT x372 = dt * x97;
	const FLT x373 = dt * x98;
	const FLT x374 = x373 + (-1 * x372);
	const FLT x375 = -1 * dt * x102;
	const FLT x376 = (-1 * dt * x100) + dt;
	const FLT x377 = x376 + x375;
	const FLT x378 = (-1 * x72 * ((x76 * x371) + (-1 * ((x74 * x374) + (x73 * x377)) * x75))) +
					 (-1 * ((x68 * x374) + (-1 * x50 * x377)) * x71);
	const FLT x379 = -1 * dt * x106;
	const FLT x380 = x379 + x375 + dt;
	const FLT x381 = dt * x109;
	const FLT x382 = dt * x110;
	const FLT x383 = x382 + x381;
	const FLT x384 = x369 + (-1 * x370);
	const FLT x385 = (-1 * x72 * ((x76 * x380) + (-1 * ((x74 * x383) + (x73 * x384)) * x75))) +
					 (-1 * ((x68 * x383) + (-1 * x50 * x384)) * x71);
	const FLT x386 = x381 + (-1 * x382);
	const FLT x387 = x376 + x379;
	const FLT x388 = x372 + x373;
	const FLT x389 = (-1 * x72 * ((x76 * x386) + (-1 * ((x74 * x387) + (x73 * x388)) * x75))) +
					 (-1 * ((x68 * x387) + (-1 * x50 * x388)) * x71);
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT),
						x77 + (x61 * ((x53 * x58) + (-1 * x5 * x50))) + (x78 * x77));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT),
						(((x84 * x58) + (-1 * x81 * x50)) * x61) + x86 + (x86 * x78));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT),
						(((x88 * x58) + (-1 * x87 * x50)) * x61) + x90 + (x78 * x90));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT),
						x105 + (((x58 * x99) + (-1 * x50 * x95)) * x61) + (x78 * x105));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT),
						x113 + (((x58 * x111) + (-1 * x50 * x108)) * x61) + (x78 * x113));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT),
						x117 + (((x58 * x115) + (-1 * x50 * x114)) * x61) + (x78 * x117));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						x156 + (((x58 * x154) + (-1 * x50 * x152)) * x61) + (x78 * x156));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						(((x58 * x176) + (-1 * x50 * x174)) * x61) + x178 + (x78 * x178));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x194 + (x78 * x194) + (((x58 * x192) + (-1 * x50 * x193)) * x61));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						x207 + (x78 * x207) + (((x58 * x205) + (-1 * x50 * x206)) * x61));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x295 + (((x58 * x293) + (-1 * x50 * x291)) * x61) + (x78 * x295));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x335 + (((x58 * x333) + (-1 * x50 * x331)) * x61) + (x78 * x335));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x368 + (((x58 * x366) + (-1 * x50 * x364)) * x61) + (x78 * x368));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT),
						x378 + (((x58 * x374) + (-1 * x50 * x371)) * x61) + (x78 * x378));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT),
						x385 + (((x58 * x383) + (-1 * x50 * x380)) * x61) + (x78 * x385));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT),
						(((x58 * x387) + (-1 * x50 * x386)) * x61) + x389 + (x78 * x389));
}

// Full version Jacobian of SurviveKalmanModel_LightMeas_x_gen1 wrt [(*_x0).AccBias[0], (*_x0).AccBias[1],
// (*_x0).AccBias[2], (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1],
// (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2],
// (*_x0).IMUCorrection[3], (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0],
// (*_x0).Pose.Rot[1], (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0],
// (*_x0).Velocity.AxisAngleRot[1], (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1],
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f45a60>]

static inline void SurviveKalmanModel_LightMeas_x_gen1_jac_x0_with_hx(CnMat *Hx, CnMat *hx, const FLT dt,
																	  const SurviveKalmanModel *_x0,
																	  const FLT *sensor_pt, const SurvivePose *lh_p,
																	  const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanModel_LightMeas_x_gen1(dt, _x0, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanModel_LightMeas_x_gen1_jac_x0(Hx, dt, _x0, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanModel_LightMeas_x_gen1 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]
static inline void SurviveKalmanModel_LightMeas_x_gen1_jac_sensor_pt(CnMat *Hx, const FLT dt,
																	 const SurviveKalmanModel *_x0,
																	 const FLT *sensor_pt, const SurvivePose *lh_p,
																	 const BaseStationCal *bsc0) {
	const FLT x0 = dt * dt;
	const FLT x1 = x0 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x2 = x0 * ((*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x3 = x0 * ((*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x4 = 1e-10 + x3 + x1 + x2;
	const FLT x5 = sqrt(x4);
	const FLT x6 = 0.5 * x5;
	const FLT x7 = cos(x6);
	const FLT x8 = sin(x6);
	const FLT x9 = (1. / x4) * (x8 * x8);
	const FLT x10 = 1. / sqrt((x2 * x9) + (x1 * x9) + (x3 * x9) + (x7 * x7));
	const FLT x11 = x7 * x10;
	const FLT x12 = x11 * (*_x0).Pose.Rot[2];
	const FLT x13 = (1. / x5) * x8 * dt * x10;
	const FLT x14 = x13 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x15 = x14 * (*_x0).Pose.Rot[0];
	const FLT x16 = x13 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x17 = x16 * (*_x0).Pose.Rot[3];
	const FLT x18 = x13 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x19 = x18 * (*_x0).Pose.Rot[1];
	const FLT x20 = (-1 * x19) + x17 + (-1 * x12) + (-1 * x15);
	const FLT x21 = x18 * (*_x0).Pose.Rot[2];
	const FLT x22 = x14 * (*_x0).Pose.Rot[3];
	const FLT x23 = x11 * (*_x0).Pose.Rot[1];
	const FLT x24 = x16 * (*_x0).Pose.Rot[0];
	const FLT x25 = x23 + x24 + (-1 * x21) + x22;
	const FLT x26 = 2 * x25;
	const FLT x27 = (-1 * x16 * (*_x0).Pose.Rot[1]) + (x11 * (*_x0).Pose.Rot[0]) + (-1 * x18 * (*_x0).Pose.Rot[3]) +
					(-1 * x14 * (*_x0).Pose.Rot[2]);
	const FLT x28 = x18 * (*_x0).Pose.Rot[0];
	const FLT x29 = x14 * (*_x0).Pose.Rot[1];
	const FLT x30 = x11 * (*_x0).Pose.Rot[3];
	const FLT x31 = x16 * (*_x0).Pose.Rot[2];
	const FLT x32 = x30 + x28 + x31 + (-1 * x29);
	const FLT x33 = 2 * x32;
	const FLT x34 = x33 * x27;
	const FLT x35 = x34 + (-1 * x20 * x26);
	const FLT x36 = (-1 * x17) + x19 + x15 + x12;
	const FLT x37 = 2 * x36;
	const FLT x38 = x37 * x27;
	const FLT x39 = (x33 * x25) + (-1 * x38);
	const FLT x40 = 1 + (x37 * x20) + (-2 * (x32 * x32));
	const FLT x41 = (x40 * (*lh_p).Rot[0]) + (-1 * x35 * (*lh_p).Rot[3]) + (x39 * (*lh_p).Rot[2]);
	const FLT x42 = 2 * (*lh_p).Rot[3];
	const FLT x43 = (x35 * (*lh_p).Rot[1]) + (-1 * x40 * (*lh_p).Rot[2]) + (x39 * (*lh_p).Rot[0]);
	const FLT x44 = 2 * (*lh_p).Rot[1];
	const FLT x45 = x35 + (x41 * x42) + (-1 * x43 * x44);
	const FLT x46 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x47 = (-1 * x32 * sensor_pt[1]) + (x36 * sensor_pt[2]) + (x27 * sensor_pt[0]);
	const FLT x48 = (-1 * x25 * sensor_pt[2]) + (x32 * sensor_pt[0]) + (x27 * sensor_pt[1]);
	const FLT x49 = (2 * ((x48 * x25) + (-1 * x47 * x36))) + sensor_pt[2] + (x46 * (*_x0).Acc[2]) +
					(dt * (*_x0).Velocity.Pos[2]) + (*_x0).Pose.Pos[2];
	const FLT x50 = (-1 * x36 * sensor_pt[0]) + (x27 * sensor_pt[2]) + (x25 * sensor_pt[1]);
	const FLT x51 = (2 * ((x50 * x36) + (-1 * x48 * x32))) + (*_x0).Pose.Pos[0] + sensor_pt[0] +
					(dt * (*_x0).Velocity.Pos[0]) + (x46 * (*_x0).Acc[0]);
	const FLT x52 = (2 * ((x47 * x32) + (-1 * x50 * x25))) + (dt * (*_x0).Velocity.Pos[1]) + (*_x0).Pose.Pos[1] +
					sensor_pt[1] + (x46 * (*_x0).Acc[1]);
	const FLT x53 = (-1 * x52 * (*lh_p).Rot[3]) + (x49 * (*lh_p).Rot[2]) + (x51 * (*lh_p).Rot[0]);
	const FLT x54 = (-1 * x49 * (*lh_p).Rot[1]) + (x52 * (*lh_p).Rot[0]) + (x51 * (*lh_p).Rot[3]);
	const FLT x55 = x49 + (2 * ((x54 * (*lh_p).Rot[1]) + (-1 * x53 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x56 = 1. / x55;
	const FLT x57 = (x40 * (*lh_p).Rot[3]) + (-1 * x39 * (*lh_p).Rot[1]) + (x35 * (*lh_p).Rot[0]);
	const FLT x58 = 2 * (*lh_p).Rot[2];
	const FLT x59 = x39 + (x57 * x44) + (-1 * x58 * x41);
	const FLT x60 = x55 * x55;
	const FLT x61 = 1. / x60;
	const FLT x62 = (-1 * x51 * (*lh_p).Rot[2]) + (x49 * (*lh_p).Rot[0]) + (x52 * (*lh_p).Rot[1]);
	const FLT x63 = x52 + (*lh_p).Pos[1] + (2 * ((x53 * (*lh_p).Rot[3]) + (-1 * x62 * (*lh_p).Rot[1])));
	const FLT x64 = x63 * x61;
	const FLT x65 = x63 * x63;
	const FLT x66 = -1 * x55;
	const FLT x67 = 2 * (1. / (x60 + x65)) * x60 * atan2(x63, x66) * (*bsc0).curve;
	const FLT x68 = (x58 * x43) + x40 + (-1 * x57 * x42);
	const FLT x69 = x51 + (2 * ((x62 * (*lh_p).Rot[2]) + (-1 * x54 * (*lh_p).Rot[3]))) + (*lh_p).Pos[0];
	const FLT x70 = x61 * x69;
	const FLT x71 = x60 + (x69 * x69);
	const FLT x72 = 1. / x71;
	const FLT x73 = x72 * x60;
	const FLT x74 = 1. / sqrt(1 + (-1 * x72 * x65 * ((*bsc0).tilt * (*bsc0).tilt)));
	const FLT x75 = 2 * x69;
	const FLT x76 = 2 * x55;
	const FLT x77 = 1.0 / 2.0 * (1. / (x71 * sqrt(x71))) * x63 * (*bsc0).tilt;
	const FLT x78 = (1. / sqrt(x71)) * (*bsc0).tilt;
	const FLT x79 = (-1 * x74 * ((x78 * x45) + (-1 * ((x76 * x59) + (x75 * x68)) * x77))) +
					(-1 * ((x70 * x59) + (-1 * x68 * x56)) * x73);
	const FLT x80 = (*bsc0).gibmag * sin(1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha +
										 (-1 * asin(x78 * x63)) + (-1 * atan2(x69, x66)));
	const FLT x81 = (-1 * x30) + (-1 * x31) + x29 + (-1 * x28);
	const FLT x82 = x26 * x27;
	const FLT x83 = x82 + (-1 * x81 * x37);
	const FLT x84 = 1 + (x81 * x33) + (-2 * (x25 * x25));
	const FLT x85 = (x37 * x25) + (-1 * x34);
	const FLT x86 = 2 * ((x85 * (*lh_p).Rot[0]) + (x83 * (*lh_p).Rot[2]) + (-1 * x84 * (*lh_p).Rot[3]));
	const FLT x87 = 2 * ((x84 * (*lh_p).Rot[1]) + (x83 * (*lh_p).Rot[0]) + (-1 * x85 * (*lh_p).Rot[2]));
	const FLT x88 = x84 + (x86 * (*lh_p).Rot[3]) + (-1 * x87 * (*lh_p).Rot[1]);
	const FLT x89 = 2 * ((x85 * (*lh_p).Rot[3]) + (-1 * x83 * (*lh_p).Rot[1]) + (x84 * (*lh_p).Rot[0]));
	const FLT x90 = x83 + (x89 * (*lh_p).Rot[1]) + (-1 * x86 * (*lh_p).Rot[2]);
	const FLT x91 = x85 + (x87 * (*lh_p).Rot[2]) + (-1 * x89 * (*lh_p).Rot[3]);
	const FLT x92 = (-1 * x74 * ((x88 * x78) + (-1 * ((x76 * x90) + (x75 * x91)) * x77))) +
					(-1 * ((x70 * x90) + (-1 * x56 * x91)) * x73);
	const FLT x93 = (x32 * x37) + (-1 * x82);
	const FLT x94 = x21 + (-1 * x24) + (-1 * x22) + (-1 * x23);
	const FLT x95 = 1 + (x94 * x26) + (-2 * (x36 * x36));
	const FLT x96 = x38 + (-1 * x94 * x33);
	const FLT x97 = (x96 * (*lh_p).Rot[0]) + (-1 * x93 * (*lh_p).Rot[3]) + (x95 * (*lh_p).Rot[2]);
	const FLT x98 = (x93 * (*lh_p).Rot[1]) + (-1 * x96 * (*lh_p).Rot[2]) + (x95 * (*lh_p).Rot[0]);
	const FLT x99 = x93 + (x97 * x42) + (-1 * x98 * x44);
	const FLT x100 = (x96 * (*lh_p).Rot[3]) + (-1 * x95 * (*lh_p).Rot[1]) + (x93 * (*lh_p).Rot[0]);
	const FLT x101 = x95 + (x44 * x100) + (-1 * x58 * x97);
	const FLT x102 = x96 + (x58 * x98) + (-1 * x42 * x100);
	const FLT x103 = (-1 * x74 * ((x78 * x99) + (-1 * ((x76 * x101) + (x75 * x102)) * x77))) +
					 (-1 * ((x70 * x101) + (-1 * x56 * x102)) * x73);
	cnMatrixOptionalSet(Hx, 0, 0, x79 + (((x64 * x59) + (-1 * x56 * x45)) * x67) + (x80 * x79));
	cnMatrixOptionalSet(Hx, 0, 1, x92 + (((x64 * x90) + (-1 * x88 * x56)) * x67) + (x80 * x92));
	cnMatrixOptionalSet(Hx, 0, 2, x103 + (x67 * ((x64 * x101) + (-1 * x56 * x99))) + (x80 * x103));
}

// Full version Jacobian of SurviveKalmanModel_LightMeas_x_gen1 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]

static inline void SurviveKalmanModel_LightMeas_x_gen1_jac_sensor_pt_with_hx(CnMat *Hx, CnMat *hx, const FLT dt,
																			 const SurviveKalmanModel *_x0,
																			 const FLT *sensor_pt,
																			 const SurvivePose *lh_p,
																			 const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanModel_LightMeas_x_gen1(dt, _x0, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanModel_LightMeas_x_gen1_jac_sensor_pt(Hx, dt, _x0, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanModel_LightMeas_x_gen1 wrt [(*lh_p).Pos[0], (*lh_p).Pos[1], (*lh_p).Pos[2], (*lh_p).Rot[0],
// (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]
static inline void SurviveKalmanModel_LightMeas_x_gen1_jac_lh_p(CnMat *Hx, const FLT dt, const SurviveKalmanModel *_x0,
																const FLT *sensor_pt, const SurvivePose *lh_p,
																const BaseStationCal *bsc0) {
	const FLT x0 = dt * (*_x0).Velocity.Pos[2];
	const FLT x1 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x2 = x1 * (*_x0).Acc[2];
	const FLT x3 = dt * dt;
	const FLT x4 = x3 * ((*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x5 = x3 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x6 = x3 * ((*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x7 = 1e-10 + x4 + x5 + x6;
	const FLT x8 = sqrt(x7);
	const FLT x9 = 0.5 * x8;
	const FLT x10 = sin(x9);
	const FLT x11 = (1. / x7) * (x10 * x10);
	const FLT x12 = cos(x9);
	const FLT x13 = 1. / sqrt((x6 * x11) + (x5 * x11) + (x4 * x11) + (x12 * x12));
	const FLT x14 = (1. / x8) * dt * x13 * x10;
	const FLT x15 = x14 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x16 = x13 * x12;
	const FLT x17 = x14 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x18 = x14 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x19 = (-1 * x18 * (*_x0).Pose.Rot[3]) + (x17 * (*_x0).Pose.Rot[1]) + (x15 * (*_x0).Pose.Rot[0]) +
					(x16 * (*_x0).Pose.Rot[2]);
	const FLT x20 = x14 * (*_x0).Pose.Rot[2];
	const FLT x21 = (x16 * (*_x0).Pose.Rot[0]) + (-1 * x18 * (*_x0).Pose.Rot[1]) + (-1 * x17 * (*_x0).Pose.Rot[3]) +
					(-1 * x20 * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x22 = (x20 * (*_x0).Velocity.AxisAngleRot[0]) + (x16 * (*_x0).Pose.Rot[3]) + (x17 * (*_x0).Pose.Rot[0]) +
					(-1 * x15 * (*_x0).Pose.Rot[1]);
	const FLT x23 = (x19 * sensor_pt[2]) + (-1 * x22 * sensor_pt[1]) + (x21 * sensor_pt[0]);
	const FLT x24 = (-1 * x20 * (*_x0).Velocity.AxisAngleRot[2]) + (x18 * (*_x0).Pose.Rot[0]) +
					(x16 * (*_x0).Pose.Rot[1]) + (x15 * (*_x0).Pose.Rot[3]);
	const FLT x25 = (x22 * sensor_pt[0]) + (-1 * x24 * sensor_pt[2]) + (x21 * sensor_pt[1]);
	const FLT x26 = 2 * ((x24 * x25) + (-1 * x23 * x19));
	const FLT x27 = sensor_pt[2] + x2 + x0 + x26 + (*_x0).Pose.Pos[2];
	const FLT x28 = x27 * (*lh_p).Rot[2];
	const FLT x29 = dt * (*_x0).Velocity.Pos[0];
	const FLT x30 = x1 * (*_x0).Acc[0];
	const FLT x31 = (-1 * x19 * sensor_pt[0]) + (x21 * sensor_pt[2]) + (x24 * sensor_pt[1]);
	const FLT x32 = 2 * ((x31 * x19) + (-1 * x25 * x22));
	const FLT x33 = x32 + (*_x0).Pose.Pos[0] + x29 + sensor_pt[0] + x30;
	const FLT x34 = x33 * (*lh_p).Rot[0];
	const FLT x35 = x1 * (*_x0).Acc[1];
	const FLT x36 = dt * (*_x0).Velocity.Pos[1];
	const FLT x37 = 2 * ((x22 * x23) + (-1 * x31 * x24));
	const FLT x38 = x37 + x36 + (*_x0).Pose.Pos[1] + sensor_pt[1] + x35;
	const FLT x39 = x38 * (*lh_p).Rot[3];
	const FLT x40 = (-1 * x39) + x28 + x34;
	const FLT x41 = x38 * (*lh_p).Rot[0];
	const FLT x42 = x33 * (*lh_p).Rot[3];
	const FLT x43 = x27 * (*lh_p).Rot[1];
	const FLT x44 = (-1 * x43) + x41 + x42;
	const FLT x45 = (2 * ((x44 * (*lh_p).Rot[1]) + (-1 * x40 * (*lh_p).Rot[2]))) + x27 + (*lh_p).Pos[2];
	const FLT x46 = x27 * (*lh_p).Rot[0];
	const FLT x47 = x38 * (*lh_p).Rot[1];
	const FLT x48 = x33 * (*lh_p).Rot[2];
	const FLT x49 = (-1 * x48) + x46 + x47;
	const FLT x50 = x33 + (2 * ((x49 * (*lh_p).Rot[2]) + (-1 * x44 * (*lh_p).Rot[3]))) + (*lh_p).Pos[0];
	const FLT x51 = x45 * x45;
	const FLT x52 = x51 + (x50 * x50);
	const FLT x53 = 1. / x52;
	const FLT x54 = (*lh_p).Pos[1] + x38 + (2 * ((x40 * (*lh_p).Rot[3]) + (-1 * x49 * (*lh_p).Rot[1])));
	const FLT x55 = x54 * x54;
	const FLT x56 = 1. / sqrt(1 + (-1 * x53 * x55 * ((*bsc0).tilt * (*bsc0).tilt)));
	const FLT x57 = x54 * (1. / (x52 * sqrt(x52))) * (*bsc0).tilt;
	const FLT x58 = x57 * x56;
	const FLT x59 = (x50 * x58) + (x53 * x45);
	const FLT x60 = (1. / sqrt(x52)) * (*bsc0).tilt;
	const FLT x61 = -1 * x45;
	const FLT x62 = (*bsc0).gibmag * sin(1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha +
										 (-1 * asin(x60 * x54)) + (-1 * atan2(x50, x61)));
	const FLT x63 = 2 * x45;
	const FLT x64 = (1. / (x51 + x55)) * atan2(x54, x61) * (*bsc0).curve;
	const FLT x65 = x60 * x56;
	const FLT x66 = 2 * x64;
	const FLT x67 = (x58 * x45) + (-1 * x50 * x53);
	const FLT x68 = 1. / x45;
	const FLT x69 = 2 * x43;
	const FLT x70 = (2 * x42) + (-1 * x69);
	const FLT x71 = 2 * x48;
	const FLT x72 = (2 * x47) + (-1 * x71);
	const FLT x73 = 1. / x51;
	const FLT x74 = x73 * x54;
	const FLT x75 = x66 * x51;
	const FLT x76 = 2 * x39;
	const FLT x77 = (2 * x28) + (-1 * x76);
	const FLT x78 = x73 * x50;
	const FLT x79 = x53 * x51;
	const FLT x80 = 2 * x50;
	const FLT x81 = 1.0 / 2.0 * x57;
	const FLT x82 = (-1 * x56 * ((x70 * x60) + (-1 * ((x72 * x63) + (x80 * x77)) * x81))) +
					(-1 * ((x72 * x78) + (-1 * x77 * x68)) * x79);
	const FLT x83 = 2 * x46;
	const FLT x84 = (-1 * x83) + (-4 * x47) + x71;
	const FLT x85 = (-1 * sensor_pt[2]) + (-1 * x2) + (-1 * x26) + (-1 * (*_x0).Pose.Pos[2]) + (-1 * x0);
	const FLT x86 = 2 * (*lh_p).Rot[1];
	const FLT x87 = 2 * x41;
	const FLT x88 = x70 + (x85 * x86) + x87;
	const FLT x89 = 2 * (*lh_p).Rot[3];
	const FLT x90 = 2 * (*lh_p).Rot[2];
	const FLT x91 = (x90 * x38) + (-1 * x89 * x85);
	const FLT x92 = (-1 * x56 * ((x84 * x60) + (-1 * ((x88 * x63) + (x80 * x91)) * x81))) +
					(-1 * ((x88 * x78) + (-1 * x68 * x91)) * x79);
	const FLT x93 = 2 * ((-1 * (*_x0).Pose.Pos[0]) + (-1 * x32) + (-1 * x29) + (-1 * x30) + (-1 * sensor_pt[0]));
	const FLT x94 = (x89 * x27) + (-1 * x93 * (*lh_p).Rot[1]);
	const FLT x95 = 2 * x34;
	const FLT x96 = (-1 * x95) + (-4 * x28) + x76;
	const FLT x97 = x72 + x83 + (x93 * (*lh_p).Rot[2]);
	const FLT x98 = (-1 * x56 * ((x60 * x94) + (-1 * ((x63 * x96) + (x80 * x97)) * x81))) +
					(-1 * ((x78 * x96) + (-1 * x68 * x97)) * x79);
	const FLT x99 = (-1 * x37) + (-1 * x36) + (-1 * (*_x0).Pose.Pos[1]) + (-1 * x35) + (-1 * sensor_pt[1]);
	const FLT x100 = x77 + (x89 * x99) + x95;
	const FLT x101 = (x86 * x33) + (-1 * x90 * x99);
	const FLT x102 = (-1 * x87) + x69 + (-4 * x42);
	const FLT x103 = (-1 * x56 * ((x60 * x100) + (-1 * ((x63 * x101) + (x80 * x102)) * x81))) +
					 (-1 * ((x78 * x101) + (-1 * x68 * x102)) * x79);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[0]) / sizeof(FLT), x59 + (x62 * x59));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[1]) / sizeof(FLT),
						(-1 * x65) + (-1 * x63 * x64) + (-1 * x62 * x65));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[2]) / sizeof(FLT), x67 + (x66 * x54) + (x62 * x67));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[0]) / sizeof(FLT),
						x82 + (((x72 * x74) + (-1 * x70 * x68)) * x75) + (x82 * x62));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[1]) / sizeof(FLT),
						x92 + (((x88 * x74) + (-1 * x84 * x68)) * x75) + (x62 * x92));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[2]) / sizeof(FLT),
						x98 + (((x74 * x96) + (-1 * x68 * x94)) * x75) + (x62 * x98));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[3]) / sizeof(FLT),
						x103 + (((x74 * x101) + (-1 * x68 * x100)) * x75) + (x62 * x103));
}

// Full version Jacobian of SurviveKalmanModel_LightMeas_x_gen1 wrt [(*lh_p).Pos[0], (*lh_p).Pos[1], (*lh_p).Pos[2],
// (*lh_p).Rot[0], (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]

static inline void SurviveKalmanModel_LightMeas_x_gen1_jac_lh_p_with_hx(CnMat *Hx, CnMat *hx, const FLT dt,
																		const SurviveKalmanModel *_x0,
																		const FLT *sensor_pt, const SurvivePose *lh_p,
																		const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanModel_LightMeas_x_gen1(dt, _x0, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanModel_LightMeas_x_gen1_jac_lh_p(Hx, dt, _x0, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanModel_LightMeas_x_gen1 wrt [<cnkalman.codegen.WrapMember object at 0x7f88f4f31ee0>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4f31c70>, <cnkalman.codegen.WrapMember object at 0x7f88f4f31f70>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4f7fa90>, <cnkalman.codegen.WrapMember object at 0x7f88f4f316d0>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4f318e0>, <cnkalman.codegen.WrapMember object at 0x7f88f4f31940>]
static inline void SurviveKalmanModel_LightMeas_x_gen1_jac_bsc0(CnMat *Hx, const FLT dt, const SurviveKalmanModel *_x0,
																const FLT *sensor_pt, const SurvivePose *lh_p,
																const BaseStationCal *bsc0) {
	const FLT x0 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x1 = dt * dt;
	const FLT x2 = x1 * ((*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x3 = x1 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x4 = x1 * ((*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x5 = 1e-10 + x2 + x3 + x4;
	const FLT x6 = sqrt(x5);
	const FLT x7 = 0.5 * x6;
	const FLT x8 = sin(x7);
	const FLT x9 = (1. / x5) * (x8 * x8);
	const FLT x10 = cos(x7);
	const FLT x11 = 1. / sqrt((x4 * x9) + (x3 * x9) + (x2 * x9) + (x10 * x10));
	const FLT x12 = (1. / x6) * x8 * dt * x11;
	const FLT x13 = x12 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x14 = x11 * x10;
	const FLT x15 = x12 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x16 = x12 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x17 = (-1 * x16 * (*_x0).Pose.Rot[3]) + (x15 * (*_x0).Pose.Rot[1]) + (x13 * (*_x0).Pose.Rot[0]) +
					(x14 * (*_x0).Pose.Rot[2]);
	const FLT x18 = (-1 * x16 * (*_x0).Pose.Rot[1]) + (x14 * (*_x0).Pose.Rot[0]) + (-1 * x15 * (*_x0).Pose.Rot[3]) +
					(-1 * x13 * (*_x0).Pose.Rot[2]);
	const FLT x19 = (x14 * (*_x0).Pose.Rot[3]) + (x15 * (*_x0).Pose.Rot[0]) + (x16 * (*_x0).Pose.Rot[2]) +
					(-1 * x13 * (*_x0).Pose.Rot[1]);
	const FLT x20 = (-1 * x19 * sensor_pt[1]) + (x17 * sensor_pt[2]) + (x18 * sensor_pt[0]);
	const FLT x21 = (x16 * (*_x0).Pose.Rot[0]) + (x14 * (*_x0).Pose.Rot[1]) + (-1 * x15 * (*_x0).Pose.Rot[2]) +
					(x13 * (*_x0).Pose.Rot[3]);
	const FLT x22 = (x19 * sensor_pt[0]) + (-1 * x21 * sensor_pt[2]) + (x18 * sensor_pt[1]);
	const FLT x23 = (2 * ((x22 * x21) + (-1 * x20 * x17))) + sensor_pt[2] + (x0 * (*_x0).Acc[2]) +
					(dt * (*_x0).Velocity.Pos[2]) + (*_x0).Pose.Pos[2];
	const FLT x24 = (-1 * x17 * sensor_pt[0]) + (x18 * sensor_pt[2]) + (x21 * sensor_pt[1]);
	const FLT x25 = (2 * ((x20 * x19) + (-1 * x24 * x21))) + (dt * (*_x0).Velocity.Pos[1]) + sensor_pt[1] +
					(*_x0).Pose.Pos[1] + (x0 * (*_x0).Acc[1]);
	const FLT x26 = (*_x0).Pose.Pos[0] + (2 * ((x24 * x17) + (-1 * x22 * x19))) + sensor_pt[0] +
					(dt * (*_x0).Velocity.Pos[0]) + (x0 * (*_x0).Acc[0]);
	const FLT x27 = (-1 * x26 * (*lh_p).Rot[2]) + (x23 * (*lh_p).Rot[0]) + (x25 * (*lh_p).Rot[1]);
	const FLT x28 = (-1 * x25 * (*lh_p).Rot[3]) + (x23 * (*lh_p).Rot[2]) + (x26 * (*lh_p).Rot[0]);
	const FLT x29 = x25 + (*lh_p).Pos[1] + (2 * ((x28 * (*lh_p).Rot[3]) + (-1 * x27 * (*lh_p).Rot[1])));
	const FLT x30 = (-1 * x23 * (*lh_p).Rot[1]) + (x25 * (*lh_p).Rot[0]) + (x26 * (*lh_p).Rot[3]);
	const FLT x31 = x23 + (2 * ((x30 * (*lh_p).Rot[1]) + (-1 * x28 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x32 = -1 * x31;
	const FLT x33 = x26 + (2 * ((x27 * (*lh_p).Rot[2]) + (-1 * x30 * (*lh_p).Rot[3]))) + (*lh_p).Pos[0];
	const FLT x34 = (x33 * x33) + (x31 * x31);
	const FLT x35 = (1. / sqrt(x34)) * x29;
	const FLT x36 = 1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha + (-1 * asin(x35 * (*bsc0).tilt)) +
					(-1 * atan2(x33, x32));
	const FLT x37 = sin(x36) * (*bsc0).gibmag;
	const FLT x38 = x35 * (1. / sqrt(1 + (-1 * (1. / x34) * (x29 * x29) * ((*bsc0).tilt * (*bsc0).tilt))));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, curve) / sizeof(FLT), atan2(x29, x32) * atan2(x29, x32));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibmag) / sizeof(FLT), -1 * cos(x36));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibpha) / sizeof(FLT), x37);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, phase) / sizeof(FLT), -1 + (-1 * x37));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, tilt) / sizeof(FLT), (-1 * x38 * x37) + (-1 * x38));
}

// Full version Jacobian of SurviveKalmanModel_LightMeas_x_gen1 wrt [<cnkalman.codegen.WrapMember object at
// 0x7f88f4f31ee0>, <cnkalman.codegen.WrapMember object at 0x7f88f4f31c70>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4f31f70>, <cnkalman.codegen.WrapMember object at 0x7f88f4f7fa90>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4f316d0>, <cnkalman.codegen.WrapMember object at 0x7f88f4f318e0>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4f31940>]

static inline void SurviveKalmanModel_LightMeas_x_gen1_jac_bsc0_with_hx(CnMat *Hx, CnMat *hx, const FLT dt,
																		const SurviveKalmanModel *_x0,
																		const FLT *sensor_pt, const SurvivePose *lh_p,
																		const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanModel_LightMeas_x_gen1(dt, _x0, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanModel_LightMeas_x_gen1_jac_bsc0(Hx, dt, _x0, sensor_pt, lh_p, bsc0);
	}
}
static inline FLT SurviveKalmanModel_LightMeas_y_gen1(const FLT dt, const SurviveKalmanModel *_x0, const FLT *sensor_pt,
													  const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const FLT x0 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x1 = dt * dt;
	const FLT x2 = x1 * ((*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x3 = x1 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x4 = x1 * ((*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x5 = 1e-10 + x2 + x3 + x4;
	const FLT x6 = sqrt(x5);
	const FLT x7 = 0.5 * x6;
	const FLT x8 = sin(x7);
	const FLT x9 = (1. / x5) * (x8 * x8);
	const FLT x10 = cos(x7);
	const FLT x11 = 1. / sqrt((x4 * x9) + (x3 * x9) + (x2 * x9) + (x10 * x10));
	const FLT x12 = (1. / x6) * x8 * dt * x11;
	const FLT x13 = x12 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x14 = x12 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x15 = x11 * x10;
	const FLT x16 = x12 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x17 = (-1 * x16 * (*_x0).Pose.Rot[1]) + (x15 * (*_x0).Pose.Rot[0]) + (-1 * x13 * (*_x0).Pose.Rot[3]) +
					(-1 * x14 * (*_x0).Pose.Rot[2]);
	const FLT x18 = (x15 * (*_x0).Pose.Rot[1]) + (x16 * (*_x0).Pose.Rot[0]) + (-1 * x13 * (*_x0).Pose.Rot[2]) +
					(x14 * (*_x0).Pose.Rot[3]);
	const FLT x19 = (x13 * (*_x0).Pose.Rot[1]) + (-1 * x16 * (*_x0).Pose.Rot[3]) + (x14 * (*_x0).Pose.Rot[0]) +
					(x15 * (*_x0).Pose.Rot[2]);
	const FLT x20 = (-1 * x19 * sensor_pt[0]) + (x17 * sensor_pt[2]) + (x18 * sensor_pt[1]);
	const FLT x21 = (x16 * (*_x0).Pose.Rot[2]) + (x15 * (*_x0).Pose.Rot[3]) + (x13 * (*_x0).Pose.Rot[0]) +
					(-1 * x14 * (*_x0).Pose.Rot[1]);
	const FLT x22 = (-1 * x21 * sensor_pt[1]) + (x19 * sensor_pt[2]) + (x17 * sensor_pt[0]);
	const FLT x23 = (dt * (*_x0).Velocity.Pos[1]) + (2 * ((x22 * x21) + (-1 * x20 * x18))) + sensor_pt[1] +
					(*_x0).Pose.Pos[1] + (x0 * (*_x0).Acc[1]);
	const FLT x24 = (-1 * x18 * sensor_pt[2]) + (x21 * sensor_pt[0]) + (x17 * sensor_pt[1]);
	const FLT x25 = (2 * ((x20 * x19) + (-1 * x24 * x21))) + (*_x0).Pose.Pos[0] + sensor_pt[0] +
					(dt * (*_x0).Velocity.Pos[0]) + (x0 * (*_x0).Acc[0]);
	const FLT x26 = sensor_pt[2] + (2 * ((x24 * x18) + (-1 * x22 * x19))) + (x0 * (*_x0).Acc[2]) +
					(dt * (*_x0).Velocity.Pos[2]) + (*_x0).Pose.Pos[2];
	const FLT x27 = (-1 * x26 * (*lh_p).Rot[1]) + (x23 * (*lh_p).Rot[0]) + (x25 * (*lh_p).Rot[3]);
	const FLT x28 = (-1 * x25 * (*lh_p).Rot[2]) + (x26 * (*lh_p).Rot[0]) + (x23 * (*lh_p).Rot[1]);
	const FLT x29 = (2 * ((x28 * (*lh_p).Rot[2]) + (-1 * x27 * (*lh_p).Rot[3]))) + x25 + (*lh_p).Pos[0];
	const FLT x30 = (-1 * x23 * (*lh_p).Rot[3]) + (x26 * (*lh_p).Rot[2]) + (x25 * (*lh_p).Rot[0]);
	const FLT x31 = x26 + (2 * ((x27 * (*lh_p).Rot[1]) + (-1 * x30 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x32 = -1 * x31;
	const FLT x33 = x23 + (*lh_p).Pos[1] + (2 * ((x30 * (*lh_p).Rot[3]) + (-1 * x28 * (*lh_p).Rot[1])));
	const FLT x34 = (-1 * (*bsc0).phase) + (-1 * asin((1. / sqrt((x33 * x33) + (x31 * x31))) * x29 * (*bsc0).tilt)) +
					(-1 * atan2(-1 * x33, x32));
	return ((atan2(x29, x32) * atan2(x29, x32)) * (*bsc0).curve) + x34 +
		   (-1 * cos(1.5707963267949 + x34 + (*bsc0).gibpha) * (*bsc0).gibmag);
}

// Jacobian of SurviveKalmanModel_LightMeas_y_gen1 wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2],
// (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3],
// (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1],
// (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1],
// (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4f502e0>]
static inline void SurviveKalmanModel_LightMeas_y_gen1_jac_x0(CnMat *Hx, const FLT dt, const SurviveKalmanModel *_x0,
															  const FLT *sensor_pt, const SurvivePose *lh_p,
															  const BaseStationCal *bsc0) {
	const FLT x0 = (*lh_p).Rot[3] * (*lh_p).Rot[3];
	const FLT x1 = dt * fabs(dt);
	const FLT x2 = -1 * x0 * x1;
	const FLT x3 = 1.0 / 2.0 * x1;
	const FLT x4 = (*lh_p).Rot[2] * (*lh_p).Rot[2];
	const FLT x5 = (-1 * x1 * x4) + x3;
	const FLT x6 = x5 + x2;
	const FLT x7 = dt * dt;
	const FLT x8 = (*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x9 = x8 * x7;
	const FLT x10 = (*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x11 = x7 * x10;
	const FLT x12 = (*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x13 = x7 * x12;
	const FLT x14 = 1e-10 + x9 + x11 + x13;
	const FLT x15 = sqrt(x14);
	const FLT x16 = 0.5 * x15;
	const FLT x17 = sin(x16);
	const FLT x18 = x17 * x17;
	const FLT x19 = 1. / x14;
	const FLT x20 = x19 * x18;
	const FLT x21 = cos(x16);
	const FLT x22 = (x20 * x13) + (x20 * x11) + (x9 * x20) + (x21 * x21);
	const FLT x23 = 1. / sqrt(x22);
	const FLT x24 = x23 * x17;
	const FLT x25 = 1. / x15;
	const FLT x26 = dt * x25;
	const FLT x27 = x24 * x26;
	const FLT x28 = x27 * (*_x0).Pose.Rot[0];
	const FLT x29 = x23 * x21;
	const FLT x30 = x29 * (*_x0).Pose.Rot[2];
	const FLT x31 = x27 * (*_x0).Pose.Rot[1];
	const FLT x32 = x24 * (*_x0).Pose.Rot[3];
	const FLT x33 = x32 * x26;
	const FLT x34 = (-1 * x33 * (*_x0).Velocity.AxisAngleRot[0]) + (x31 * (*_x0).Velocity.AxisAngleRot[2]) +
					(x28 * (*_x0).Velocity.AxisAngleRot[1]) + x30;
	const FLT x35 = x27 * (*_x0).Pose.Rot[2];
	const FLT x36 = x29 * (*_x0).Pose.Rot[0];
	const FLT x37 = (-1 * x31 * (*_x0).Velocity.AxisAngleRot[0]) + x36 + (-1 * x33 * (*_x0).Velocity.AxisAngleRot[2]) +
					(-1 * x35 * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x38 = x29 * (*_x0).Pose.Rot[3];
	const FLT x39 = (x35 * (*_x0).Velocity.AxisAngleRot[0]) + x38 + (x28 * (*_x0).Velocity.AxisAngleRot[2]) +
					(-1 * x31 * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x40 = (-1 * x39 * sensor_pt[1]) + (x34 * sensor_pt[2]) + (x37 * sensor_pt[0]);
	const FLT x41 = x29 * (*_x0).Pose.Rot[1];
	const FLT x42 = (x28 * (*_x0).Velocity.AxisAngleRot[0]) + x41 + (-1 * x35 * (*_x0).Velocity.AxisAngleRot[2]) +
					(x33 * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x43 = (-1 * x42 * sensor_pt[2]) + (x39 * sensor_pt[0]) + (x37 * sensor_pt[1]);
	const FLT x44 = sensor_pt[2] + (x3 * (*_x0).Acc[2]) + (dt * (*_x0).Velocity.Pos[2]) +
					(2 * ((x42 * x43) + (-1 * x40 * x34))) + (*_x0).Pose.Pos[2];
	const FLT x45 = (-1 * x34 * sensor_pt[0]) + (x37 * sensor_pt[2]) + (x42 * sensor_pt[1]);
	const FLT x46 = (*_x0).Pose.Pos[0] + sensor_pt[0] + (2 * ((x45 * x34) + (-1 * x43 * x39))) +
					(dt * (*_x0).Velocity.Pos[0]) + (x3 * (*_x0).Acc[0]);
	const FLT x47 = (dt * (*_x0).Velocity.Pos[1]) + sensor_pt[1] + (2 * ((x40 * x39) + (-1 * x42 * x45))) +
					(*_x0).Pose.Pos[1] + (x3 * (*_x0).Acc[1]);
	const FLT x48 = (x44 * (*lh_p).Rot[2]) + (-1 * x47 * (*lh_p).Rot[3]) + (x46 * (*lh_p).Rot[0]);
	const FLT x49 = (-1 * x44 * (*lh_p).Rot[1]) + (x47 * (*lh_p).Rot[0]) + (x46 * (*lh_p).Rot[3]);
	const FLT x50 = x44 + (2 * ((x49 * (*lh_p).Rot[1]) + (-1 * x48 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x51 = 1. / x50;
	const FLT x52 = x1 * (*lh_p).Rot[0];
	const FLT x53 = x52 * (*lh_p).Rot[2];
	const FLT x54 = x1 * (*lh_p).Rot[3];
	const FLT x55 = x54 * (*lh_p).Rot[1];
	const FLT x56 = x55 + (-1 * x53);
	const FLT x57 = x50 * x50;
	const FLT x58 = 1. / x57;
	const FLT x59 = (x44 * (*lh_p).Rot[0]) + (-1 * x46 * (*lh_p).Rot[2]) + (x47 * (*lh_p).Rot[1]);
	const FLT x60 = x46 + (2 * ((x59 * (*lh_p).Rot[2]) + (-1 * x49 * (*lh_p).Rot[3]))) + (*lh_p).Pos[0];
	const FLT x61 = x60 * x58;
	const FLT x62 = -1 * x50;
	const FLT x63 = x60 * x60;
	const FLT x64 = 2 * (1. / (x57 + x63)) * x57 * atan2(x60, x62) * (*bsc0).curve;
	const FLT x65 = x1 * (*lh_p).Rot[1] * (*lh_p).Rot[2];
	const FLT x66 = x54 * (*lh_p).Rot[0];
	const FLT x67 = x66 + x65;
	const FLT x68 = (*lh_p).Pos[1] + x47 + (2 * ((x48 * (*lh_p).Rot[3]) + (-1 * x59 * (*lh_p).Rot[1])));
	const FLT x69 = x68 * x58;
	const FLT x70 = (x68 * x68) + x57;
	const FLT x71 = 1. / x70;
	const FLT x72 = x71 * x57;
	const FLT x73 = 1. / sqrt(1 + (-1 * x71 * x63 * ((*bsc0).tilt * (*bsc0).tilt)));
	const FLT x74 = (1. / sqrt(x70)) * (*bsc0).tilt;
	const FLT x75 = 2 * x68;
	const FLT x76 = 2 * x50;
	const FLT x77 = 1.0 / 2.0 * (1. / (x70 * sqrt(x70))) * x60 * (*bsc0).tilt;
	const FLT x78 = (-1 * x73 * ((-1 * ((x76 * x56) + (x75 * x67)) * x77) + (x6 * x74))) +
					(-1 * ((-1 * x69 * x56) + (x67 * x51)) * x72);
	const FLT x79 = (*bsc0).gibmag * sin(1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha +
										 (-1 * asin(x74 * x60)) + (-1 * atan2(-1 * x68, x62)));
	const FLT x80 = x65 + (-1 * x66);
	const FLT x81 = x54 * (*lh_p).Rot[2];
	const FLT x82 = x52 * (*lh_p).Rot[1];
	const FLT x83 = x82 + x81;
	const FLT x84 = (*lh_p).Rot[1] * (*lh_p).Rot[1];
	const FLT x85 = -1 * x1 * x84;
	const FLT x86 = x85 + x2 + x3;
	const FLT x87 = (-1 * x73 * ((-1 * ((x83 * x76) + (x86 * x75)) * x77) + (x80 * x74))) +
					(-1 * ((-1 * x83 * x69) + (x86 * x51)) * x72);
	const FLT x88 = x53 + x55;
	const FLT x89 = x5 + x85;
	const FLT x90 = x81 + (-1 * x82);
	const FLT x91 = (-1 * x73 * ((-1 * ((x89 * x76) + (x75 * x90)) * x77) + (x88 * x74))) +
					(-1 * ((-1 * x89 * x69) + (x51 * x90)) * x72);
	const FLT x92 = 2 * x4;
	const FLT x93 = -1 * x92;
	const FLT x94 = 2 * x0;
	const FLT x95 = 1 + (-1 * x94);
	const FLT x96 = x95 + x93;
	const FLT x97 = 2 * (*lh_p).Rot[2];
	const FLT x98 = x97 * (*lh_p).Rot[0];
	const FLT x99 = 2 * (*lh_p).Rot[1];
	const FLT x100 = x99 * (*lh_p).Rot[3];
	const FLT x101 = x100 + (-1 * x98);
	const FLT x102 = x97 * (*lh_p).Rot[1];
	const FLT x103 = 2 * (*lh_p).Rot[3];
	const FLT x104 = x103 * (*lh_p).Rot[0];
	const FLT x105 = x104 + x102;
	const FLT x106 = (-1 * x73 * ((-1 * ((x76 * x101) + (x75 * x105)) * x77) + (x74 * x96))) +
					 (-1 * ((-1 * x69 * x101) + (x51 * x105)) * x72);
	const FLT x107 = x102 + (-1 * x104);
	const FLT x108 = x97 * (*lh_p).Rot[3];
	const FLT x109 = x99 * (*lh_p).Rot[0];
	const FLT x110 = x109 + x108;
	const FLT x111 = 2 * x84;
	const FLT x112 = -1 * x111;
	const FLT x113 = x95 + x112;
	const FLT x114 = (-1 * x73 * ((-1 * ((x76 * x110) + (x75 * x113)) * x77) + (x74 * x107))) +
					 (-1 * ((-1 * x69 * x110) + (x51 * x113)) * x72);
	const FLT x115 = x98 + x100;
	const FLT x116 = 1 + x112 + x93;
	const FLT x117 = x108 + (-1 * x109);
	const FLT x118 = (-1 * x73 * ((-1 * ((x76 * x116) + (x75 * x117)) * x77) + (x74 * x115))) +
					 (-1 * ((-1 * x69 * x116) + (x51 * x117)) * x72);
	const FLT x119 = x27 * sensor_pt[2];
	const FLT x120 = x119 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x121 = x29 * sensor_pt[0];
	const FLT x122 = x27 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x123 = -1 * x122 * sensor_pt[1];
	const FLT x124 = x123 + x121;
	const FLT x125 = x124 + x120;
	const FLT x126 = 2 * x34;
	const FLT x127 = 2 * x43;
	const FLT x128 = x27 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x129 = x128 * x127;
	const FLT x130 = 2 * x40;
	const FLT x131 = x27 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x132 = -1 * x130 * x131;
	const FLT x133 = x122 * sensor_pt[0];
	const FLT x134 = -1 * x119 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x135 = x29 * sensor_pt[1];
	const FLT x136 = x135 + x134;
	const FLT x137 = x136 + x133;
	const FLT x138 = 2 * x42;
	const FLT x139 = (x137 * x138) + x132 + (-1 * x126 * x125) + x129;
	const FLT x140 = -1 * x131 * sensor_pt[0];
	const FLT x141 = x29 * sensor_pt[2];
	const FLT x142 = x128 * sensor_pt[1];
	const FLT x143 = x142 + x140 + x141;
	const FLT x144 = 2 * x39;
	const FLT x145 = x122 * x130;
	const FLT x146 = 2 * x45;
	const FLT x147 = -1 * x128 * x146;
	const FLT x148 = x145 + (-1 * x138 * x143) + x147 + (x125 * x144);
	const FLT x149 = -1 * x122 * x127;
	const FLT x150 = x131 * x146;
	const FLT x151 = x150 + (-1 * x137 * x144) + (x126 * x143) + x149;
	const FLT x152 = (x151 * (*lh_p).Rot[3]) + (-1 * x139 * (*lh_p).Rot[1]) + (x148 * (*lh_p).Rot[0]);
	const FLT x153 = (x148 * (*lh_p).Rot[1]) + (-1 * x151 * (*lh_p).Rot[2]) + (x139 * (*lh_p).Rot[0]);
	const FLT x154 = x151 + (-1 * x103 * x152) + (x97 * x153);
	const FLT x155 = (x151 * (*lh_p).Rot[0]) + (-1 * x148 * (*lh_p).Rot[3]) + (x139 * (*lh_p).Rot[2]);
	const FLT x156 = x139 + (-1 * x97 * x155) + (x99 * x152);
	const FLT x157 = (-1 * x99 * x153) + x148 + (x103 * x155);
	const FLT x158 = (-1 * x73 * ((-1 * ((x76 * x156) + (x75 * x157)) * x77) + (x74 * x154))) +
					 (-1 * ((-1 * x69 * x156) + (x51 * x157)) * x72);
	const FLT x159 = x131 * sensor_pt[1];
	const FLT x160 = x119 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x161 = x128 * sensor_pt[0];
	const FLT x162 = (-1 * x161) + x159 + x160;
	const FLT x163 = (-1 * x142) + x140;
	const FLT x164 = x163 + (-1 * x141);
	const FLT x165 = x29 * x127;
	const FLT x166 = (-1 * x126 * x162) + (-1 * x145) + x165 + (x164 * x138);
	const FLT x167 = -1 * x133;
	const FLT x168 = x136 + x167;
	const FLT x169 = x29 * x146;
	const FLT x170 = (-1 * x169) + x132 + (-1 * x168 * x138) + (x162 * x144);
	const FLT x171 = x122 * x146;
	const FLT x172 = x127 * x131;
	const FLT x173 = (x126 * x168) + x172 + x171 + (-1 * x164 * x144);
	const FLT x174 = (x173 * (*lh_p).Rot[3]) + (-1 * x166 * (*lh_p).Rot[1]) + (x170 * (*lh_p).Rot[0]);
	const FLT x175 = 2 * ((-1 * x173 * (*lh_p).Rot[2]) + (x170 * (*lh_p).Rot[1]) + (x166 * (*lh_p).Rot[0]));
	const FLT x176 = x173 + (-1 * x103 * x174) + (x175 * (*lh_p).Rot[2]);
	const FLT x177 = 2 * ((x173 * (*lh_p).Rot[0]) + (-1 * x170 * (*lh_p).Rot[3]) + (x166 * (*lh_p).Rot[2]));
	const FLT x178 = x166 + (-1 * x177 * (*lh_p).Rot[2]) + (x99 * x174);
	const FLT x179 = x170 + (-1 * x175 * (*lh_p).Rot[1]) + (x177 * (*lh_p).Rot[3]);
	const FLT x180 = (-1 * x73 * ((-1 * ((x76 * x178) + (x75 * x179)) * x77) + (x74 * x176))) +
					 (-1 * ((-1 * x69 * x178) + (x51 * x179)) * x72);
	const FLT x181 = -1 * x120;
	const FLT x182 = x123 + (-1 * x121) + x181;
	const FLT x183 = x163 + x141;
	const FLT x184 = x128 * x130;
	const FLT x185 = (-1 * x182 * x138) + x184 + (x183 * x144) + x171;
	const FLT x186 = x29 * x130;
	const FLT x187 = (-1 * x159) + x160 + x161;
	const FLT x188 = x149 + (x187 * x138) + (-1 * x126 * x183) + (-1 * x186);
	const FLT x189 = (x126 * x182) + x169 + (-1 * x129) + (-1 * x187 * x144);
	const FLT x190 = (x189 * (*lh_p).Rot[3]) + (x185 * (*lh_p).Rot[0]) + (-1 * x188 * (*lh_p).Rot[1]);
	const FLT x191 = (x185 * (*lh_p).Rot[1]) + (x188 * (*lh_p).Rot[0]) + (-1 * x189 * (*lh_p).Rot[2]);
	const FLT x192 = x189 + (-1 * x103 * x190) + (x97 * x191);
	const FLT x193 = (-1 * x185 * (*lh_p).Rot[3]) + (x189 * (*lh_p).Rot[0]) + (x188 * (*lh_p).Rot[2]);
	const FLT x194 = x188 + (-1 * x97 * x193) + (x99 * x190);
	const FLT x195 = x185 + (-1 * x99 * x191) + (x103 * x193);
	const FLT x196 = (-1 * x73 * ((-1 * ((x76 * x194) + (x75 * x195)) * x77) + (x74 * x192))) +
					 (-1 * ((-1 * x69 * x194) + (x51 * x195)) * x72);
	const FLT x197 = x167 + (-1 * x135) + x134;
	const FLT x198 = x124 + x181;
	const FLT x199 = (x198 * x138) + (-1 * x126 * x197) + x172 + x184;
	const FLT x200 = x161 + x159 + (-1 * x160);
	const FLT x201 = (-1 * x150) + x186 + (-1 * x200 * x138) + (x197 * x144);
	const FLT x202 = x147 + (x200 * x126) + (-1 * x165) + (-1 * x198 * x144);
	const FLT x203 = (x202 * (*lh_p).Rot[3]) + (-1 * x199 * (*lh_p).Rot[1]) + (x201 * (*lh_p).Rot[0]);
	const FLT x204 = (x201 * (*lh_p).Rot[1]) + (-1 * x202 * (*lh_p).Rot[2]) + (x199 * (*lh_p).Rot[0]);
	const FLT x205 = x202 + (-1 * x203 * x103) + (x97 * x204);
	const FLT x206 = (x202 * (*lh_p).Rot[0]) + (-1 * x201 * (*lh_p).Rot[3]) + (x199 * (*lh_p).Rot[2]);
	const FLT x207 = (-1 * x97 * x206) + x199 + (x99 * x203);
	const FLT x208 = x201 + (x206 * x103) + (-1 * x99 * x204);
	const FLT x209 = (-1 * x73 * ((-1 * ((x76 * x207) + (x75 * x208)) * x77) + (x74 * x205))) +
					 (-1 * ((-1 * x69 * x207) + (x51 * x208)) * x72);
	const FLT x210 = dt * dt * dt;
	const FLT x211 = 0.5 * x19 * x210;
	const FLT x212 = x12 * x211;
	const FLT x213 = 1.0 / 2.0 * (1. / (x22 * sqrt(x22)));
	const FLT x214 = x26 * x17 * x213;
	const FLT x215 = x214 * (*_x0).Pose.Rot[0];
	const FLT x216 =
		(*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x217 = dt * dt * dt * dt;
	const FLT x218 = 2 * (1. / (x14 * x14)) * x18;
	const FLT x219 = x218 * x217;
	const FLT x220 = x8 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x221 = 1. / (x14 * sqrt(x14));
	const FLT x222 = 1.0 * x21 * x17;
	const FLT x223 = x221 * x222;
	const FLT x224 = x217 * x223;
	const FLT x225 = x25 * x222;
	const FLT x226 = x7 * x225;
	const FLT x227 = 2 * x20;
	const FLT x228 = x7 * x227;
	const FLT x229 = x10 * x224;
	const FLT x230 = x10 * x219;
	const FLT x231 = (-1 * x230 * (*_x0).Velocity.AxisAngleRot[0]) + (x229 * (*_x0).Velocity.AxisAngleRot[0]) +
					 (x220 * x224) + (-1 * x219 * x216) + (-1 * x219 * x220) + (x216 * x224) +
					 (x228 * (*_x0).Velocity.AxisAngleRot[0]) + (-1 * x226 * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x232 = x231 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x233 = x210 * x221;
	const FLT x234 = x12 * x233;
	const FLT x235 = x24 * (*_x0).Pose.Rot[2];
	const FLT x236 = x21 * x213;
	const FLT x237 = x231 * x236;
	const FLT x238 = x231 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x239 = x214 * x238;
	const FLT x240 = 0.5 * x25;
	const FLT x241 = x7 * x240;
	const FLT x242 = x241 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x243 = x214 * (*_x0).Pose.Rot[2];
	const FLT x244 = x231 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x245 = x211 * (*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x246 = x24 * (*_x0).Pose.Rot[0];
	const FLT x247 = x233 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x248 = x247 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x249 = (-1 * x246 * x248) + (x36 * x245);
	const FLT x250 = x24 * (*_x0).Pose.Rot[1];
	const FLT x251 = x247 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x252 = x211 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x253 = x252 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x254 = (-1 * x41 * x253) + (x250 * x251);
	const FLT x255 = x249 + (-1 * x243 * x244) + x254 + (-1 * x32 * x242) + x35 + (-1 * x215 * x232) + (x30 * x212) +
					 (-1 * x234 * x235) + (-1 * x237 * (*_x0).Pose.Rot[3]) + (x239 * (*_x0).Pose.Rot[1]);
	const FLT x256 = -1 * x33;
	const FLT x257 = x214 * (*_x0).Pose.Rot[3];
	const FLT x258 = x250 * x248;
	const FLT x259 = x41 * x245;
	const FLT x260 = x214 * (*_x0).Pose.Rot[1];
	const FLT x261 = (-1 * x251 * x246) + (x36 * x253);
	const FLT x262 = x261 + (-1 * x232 * x260) + (-1 * x258) + (-1 * x239 * (*_x0).Pose.Rot[0]) + x256 +
					 (-1 * x235 * x242) + (-1 * x237 * (*_x0).Pose.Rot[2]) + (-1 * x38 * x212) + (x32 * x234) + x259 +
					 (x257 * x244);
	const FLT x263 = x30 * x253;
	const FLT x264 = x235 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x265 = x264 * x247;
	const FLT x266 = -1 * x31;
	const FLT x267 = (-1 * x38 * x245) + (x32 * x248);
	const FLT x268 = x267 + (x234 * x250) + (-1 * x41 * x212) + (-1 * x237 * (*_x0).Pose.Rot[0]) + x266 +
					 (x239 * (*_x0).Pose.Rot[2]) + (-1 * x263) + (x232 * x257) + (x260 * x244) + (-1 * x242 * x246) +
					 x265;
	const FLT x269 = (x268 * sensor_pt[0]) + (-1 * x255 * sensor_pt[1]) + (x262 * sensor_pt[2]);
	const FLT x270 = x38 * x253;
	const FLT x271 = x32 * x251;
	const FLT x272 = x30 * x245;
	const FLT x273 = x235 * x248;
	const FLT x274 = (-1 * x215 * x244) + x28 + (-1 * x234 * x246) + (-1 * x272) + (x232 * x243) + x270 + (-1 * x271) +
					 (-1 * x250 * x242) + x273 + (-1 * x237 * (*_x0).Pose.Rot[1]) + (-1 * x238 * x257) + (x36 * x212);
	const FLT x275 = (x268 * sensor_pt[1]) + (-1 * x274 * sensor_pt[2]) + (x255 * sensor_pt[0]);
	const FLT x276 = 2 * x274;
	const FLT x277 = (x43 * x276) + (x275 * x138) + (-1 * x269 * x126) + (-1 * x262 * x130);
	const FLT x278 = (x274 * sensor_pt[1]) + (-1 * x262 * sensor_pt[0]) + (x268 * sensor_pt[2]);
	const FLT x279 = (x269 * x144) + (-1 * x278 * x138) + (-1 * x45 * x276) + (x255 * x130);
	const FLT x280 = (x278 * x126) + (x262 * x146) + (-1 * x255 * x127) + (-1 * x275 * x144);
	const FLT x281 = 2 * ((x280 * (*lh_p).Rot[3]) + (-1 * x277 * (*lh_p).Rot[1]) + (x279 * (*lh_p).Rot[0]));
	const FLT x282 = 2 * ((x279 * (*lh_p).Rot[1]) + (-1 * x280 * (*lh_p).Rot[2]) + (x277 * (*lh_p).Rot[0]));
	const FLT x283 = (-1 * x281 * (*lh_p).Rot[3]) + x280 + (x282 * (*lh_p).Rot[2]);
	const FLT x284 = 2 * ((x280 * (*lh_p).Rot[0]) + (x277 * (*lh_p).Rot[2]) + (-1 * x279 * (*lh_p).Rot[3]));
	const FLT x285 = x277 + (-1 * x284 * (*lh_p).Rot[2]) + (x281 * (*lh_p).Rot[1]);
	const FLT x286 = x279 + (-1 * x282 * (*lh_p).Rot[1]) + (x284 * (*lh_p).Rot[3]);
	const FLT x287 = (-1 * x73 * ((-1 * ((x76 * x285) + (x75 * x286)) * x77) + (x74 * x283))) +
					 (-1 * ((-1 * x69 * x285) + (x51 * x286)) * x72);
	const FLT x288 = x12 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x289 =
		(*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x290 = (-1 * x230 * (*_x0).Velocity.AxisAngleRot[1]) + (x229 * (*_x0).Velocity.AxisAngleRot[1]) +
					 (x289 * x224) + (-1 * x226 * (*_x0).Velocity.AxisAngleRot[1]) + (-1 * x219 * x288) +
					 (x288 * x224) + (x228 * (*_x0).Velocity.AxisAngleRot[1]) + (-1 * x219 * x289);
	const FLT x291 = x214 * x290;
	const FLT x292 = x291 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x293 = x291 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x294 = x8 * x233;
	const FLT x295 = x236 * x290;
	const FLT x296 = x291 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x297 = x8 * x211;
	const FLT x298 = x241 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x299 = x246 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x300 = x233 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x301 = x252 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x302 = (x36 * x301) + (-1 * x299 * x300);
	const FLT x303 = x263 + (-1 * x32 * x298) + x302 + (-1 * x265) + (-1 * x296 * (*_x0).Pose.Rot[0]) +
					 (-1 * x292 * (*_x0).Pose.Rot[2]) + (x294 * x250) + (x293 * (*_x0).Pose.Rot[1]) +
					 (-1 * x41 * x297) + x266 + (-1 * x295 * (*_x0).Pose.Rot[3]);
	const FLT x304 = x300 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x305 = x250 * x304;
	const FLT x306 = x41 * x301;
	const FLT x307 = (-1 * x293 * (*_x0).Pose.Rot[0]) + (-1 * x264 * x241) + (-1 * x294 * x246) + (-1 * x270) +
					 (x292 * (*_x0).Pose.Rot[3]) + (-1 * x295 * (*_x0).Pose.Rot[2]) + x28 + (-1 * x305) +
					 (-1 * x296 * (*_x0).Pose.Rot[1]) + x271 + (x36 * x297) + x306;
	const FLT x308 = -1 * x35;
	const FLT x309 = (*_x0).Velocity.AxisAngleRot[1] * (*_x0).Pose.Rot[2];
	const FLT x310 = x38 * x301;
	const FLT x311 = x32 * x304;
	const FLT x312 = x254 + (x235 * x294) + (-1 * x299 * x241) + x311 + (x291 * x309) +
					 (-1 * x295 * (*_x0).Pose.Rot[0]) + x308 + (-1 * x30 * x297) + (-1 * x310) +
					 (x296 * (*_x0).Pose.Rot[3]) + (x292 * (*_x0).Pose.Rot[1]);
	const FLT x313 = (x312 * sensor_pt[0]) + (-1 * x303 * sensor_pt[1]) + (x307 * sensor_pt[2]);
	const FLT x314 = (-1 * x30 * x301) + (x264 * x300);
	const FLT x315 = (-1 * x295 * (*_x0).Pose.Rot[1]) + (x38 * x297) + x33 + x314 + (-1 * x293 * (*_x0).Pose.Rot[3]) +
					 x261 + (-1 * x298 * x250) + (x296 * (*_x0).Pose.Rot[2]) + (-1 * x292 * (*_x0).Pose.Rot[0]) +
					 (-1 * x32 * x294);
	const FLT x316 = (x312 * sensor_pt[1]) + (-1 * x315 * sensor_pt[2]) + (x303 * sensor_pt[0]);
	const FLT x317 = (x316 * x138) + (-1 * x307 * x130) + (-1 * x313 * x126) + (x315 * x127);
	const FLT x318 = (x315 * sensor_pt[1]) + (-1 * x307 * sensor_pt[0]) + (x312 * sensor_pt[2]);
	const FLT x319 = (x303 * x130) + (x313 * x144) + (-1 * x318 * x138) + (-1 * x315 * x146);
	const FLT x320 = (-1 * x316 * x144) + (-1 * x303 * x127) + (x307 * x146) + (x318 * x126);
	const FLT x321 = (-1 * x317 * (*lh_p).Rot[1]) + (x320 * (*lh_p).Rot[3]) + (x319 * (*lh_p).Rot[0]);
	const FLT x322 = (x319 * (*lh_p).Rot[1]) + (-1 * x320 * (*lh_p).Rot[2]) + (x317 * (*lh_p).Rot[0]);
	const FLT x323 = x320 + (-1 * x321 * x103) + (x97 * x322);
	const FLT x324 = (-1 * x319 * (*lh_p).Rot[3]) + (x320 * (*lh_p).Rot[0]) + (x317 * (*lh_p).Rot[2]);
	const FLT x325 = (-1 * x97 * x324) + x317 + (x99 * x321);
	const FLT x326 = x319 + (-1 * x99 * x322) + (x324 * x103);
	const FLT x327 = (-1 * x73 * ((-1 * ((x76 * x325) + (x75 * x326)) * x77) + (x74 * x323))) +
					 (-1 * ((-1 * x69 * x325) + (x51 * x326)) * x72);
	const FLT x328 = x10 * x211;
	const FLT x329 = x219 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x330 = x224 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x331 =
		x217 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x332 = x7 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x333 = (-1 * x218 * x331) + (x227 * x332) + (x12 * x330) + (-1 * x8 * x329) + (-1 * x12 * x329) +
					 (x223 * x331) + (x8 * x330) + (-1 * x225 * x332);
	const FLT x334 = x333 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x335 = x333 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x336 = x214 * x335;
	const FLT x337 = x10 * x233;
	const FLT x338 = x236 * x333;
	const FLT x339 = x333 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x340 = x240 * x332;
	const FLT x341 = (-1 * x273) + (-1 * x336 * (*_x0).Pose.Rot[2]) + (-1 * x215 * x339) + (x260 * x334) + (-1 * x306) +
					 x305 + x28 + (-1 * x338 * (*_x0).Pose.Rot[3]) + (-1 * x246 * x337) + (x36 * x328) +
					 (-1 * x32 * x340) + x272;
	const FLT x342 = x267 + x31 + (-1 * x250 * x337) + (x257 * x335) + (-1 * x260 * x339) + (-1 * x215 * x334) +
					 (-1 * x338 * (*_x0).Pose.Rot[2]) + x302 + (-1 * x235 * x340) + (x41 * x328);
	const FLT x343 = x214 * x333;
	const FLT x344 = x314 + (-1 * x338 * (*_x0).Pose.Rot[0]) + (x257 * x339) + (-1 * x246 * x340) + (x32 * x337) +
					 (x260 * x335) + (-1 * x38 * x328) + x256 + x258 + (-1 * x259) + (x309 * x343);
	const FLT x345 = (x344 * sensor_pt[0]) + (-1 * x341 * sensor_pt[1]) + (x342 * sensor_pt[2]);
	const FLT x346 = x249 + (-1 * x250 * x340) + (-1 * x338 * (*_x0).Pose.Rot[1]) + (-1 * x336 * (*_x0).Pose.Rot[0]) +
					 (-1 * x311) + (-1 * x30 * x328) + x310 + (-1 * x257 * x334) + (x235 * x337) +
					 (x343 * (*_x0).Velocity.AxisAngleRot[2] * (*_x0).Pose.Rot[2]) + x308;
	const FLT x347 = (-1 * x346 * sensor_pt[2]) + (x344 * sensor_pt[1]) + (x341 * sensor_pt[0]);
	const FLT x348 = 2 * x342;
	const FLT x349 = 2 * x346;
	const FLT x350 = (x43 * x349) + (-1 * x40 * x348) + (-1 * x345 * x126) + (x347 * x138);
	const FLT x351 = (x346 * sensor_pt[1]) + (-1 * x342 * sensor_pt[0]) + (x344 * sensor_pt[2]);
	const FLT x352 = 2 * x341;
	const FLT x353 = (-1 * x45 * x349) + (x345 * x144) + (-1 * x351 * x138) + (x40 * x352);
	const FLT x354 = (x351 * x126) + (-1 * x43 * x352) + (-1 * x347 * x144) + (x45 * x348);
	const FLT x355 = (x354 * (*lh_p).Rot[3]) + (-1 * x350 * (*lh_p).Rot[1]) + (x353 * (*lh_p).Rot[0]);
	const FLT x356 = (-1 * x354 * (*lh_p).Rot[2]) + (x353 * (*lh_p).Rot[1]) + (x350 * (*lh_p).Rot[0]);
	const FLT x357 = x354 + (-1 * x355 * x103) + (x97 * x356);
	const FLT x358 = (x354 * (*lh_p).Rot[0]) + (-1 * x353 * (*lh_p).Rot[3]) + (x350 * (*lh_p).Rot[2]);
	const FLT x359 = x350 + (x99 * x355) + (-1 * x97 * x358);
	const FLT x360 = x353 + (-1 * x99 * x356) + (x358 * x103);
	const FLT x361 = (-1 * x73 * ((-1 * ((x76 * x359) + (x75 * x360)) * x77) + (x74 * x357))) +
					 (-1 * ((-1 * x69 * x359) + (x51 * x360)) * x72);
	const FLT x362 = -1 * dt * x92;
	const FLT x363 = (-1 * dt * x94) + dt;
	const FLT x364 = x363 + x362;
	const FLT x365 = dt * x98;
	const FLT x366 = dt * x100;
	const FLT x367 = x366 + (-1 * x365);
	const FLT x368 = dt * x102;
	const FLT x369 = dt * x104;
	const FLT x370 = x369 + x368;
	const FLT x371 = (-1 * x73 * ((-1 * ((x76 * x367) + (x75 * x370)) * x77) + (x74 * x364))) +
					 (-1 * ((-1 * x69 * x367) + (x51 * x370)) * x72);
	const FLT x372 = x368 + (-1 * x369);
	const FLT x373 = dt * x108;
	const FLT x374 = dt * x109;
	const FLT x375 = x374 + x373;
	const FLT x376 = -1 * dt * x111;
	const FLT x377 = x363 + x376;
	const FLT x378 = (-1 * x73 * ((-1 * ((x76 * x375) + (x75 * x377)) * x77) + (x74 * x372))) +
					 (-1 * ((-1 * x69 * x375) + (x51 * x377)) * x72);
	const FLT x379 = x365 + x366;
	const FLT x380 = x362 + dt + x376;
	const FLT x381 = x373 + (-1 * x374);
	const FLT x382 = (-1 * x73 * ((-1 * ((x76 * x380) + (x75 * x381)) * x77) + (x74 * x379))) +
					 (-1 * ((-1 * x69 * x380) + (x51 * x381)) * x72);
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT),
						x78 + (x64 * ((x61 * x56) + (-1 * x6 * x51))) + (x79 * x78));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT),
						x87 + (((x83 * x61) + (-1 * x80 * x51)) * x64) + (x87 * x79));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT),
						x91 + (((x89 * x61) + (-1 * x88 * x51)) * x64) + (x79 * x91));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT),
						x106 + (x64 * ((x61 * x101) + (-1 * x51 * x96))) + (x79 * x106));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT),
						x114 + (((x61 * x110) + (-1 * x51 * x107)) * x64) + (x79 * x114));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT),
						x118 + (((x61 * x116) + (-1 * x51 * x115)) * x64) + (x79 * x118));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						x158 + (((x61 * x156) + (-1 * x51 * x154)) * x64) + (x79 * x158));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						x180 + (((x61 * x178) + (-1 * x51 * x176)) * x64) + (x79 * x180));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						(((x61 * x194) + (-1 * x51 * x192)) * x64) + x196 + (x79 * x196));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						x209 + (((x61 * x207) + (-1 * x51 * x205)) * x64) + (x79 * x209));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x287 + (((x61 * x285) + (-1 * x51 * x283)) * x64) + (x79 * x287));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x327 + (((x61 * x325) + (-1 * x51 * x323)) * x64) + (x79 * x327));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x361 + (((x61 * x359) + (-1 * x51 * x357)) * x64) + (x79 * x361));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT),
						x371 + (((x61 * x367) + (-1 * x51 * x364)) * x64) + (x79 * x371));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT),
						x378 + (((x61 * x375) + (-1 * x51 * x372)) * x64) + (x79 * x378));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT),
						x382 + (((x61 * x380) + (-1 * x51 * x379)) * x64) + (x79 * x382));
}

// Full version Jacobian of SurviveKalmanModel_LightMeas_y_gen1 wrt [(*_x0).AccBias[0], (*_x0).AccBias[1],
// (*_x0).AccBias[2], (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1],
// (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2],
// (*_x0).IMUCorrection[3], (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0],
// (*_x0).Pose.Rot[1], (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0],
// (*_x0).Velocity.AxisAngleRot[1], (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1],
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f502e0>]

static inline void SurviveKalmanModel_LightMeas_y_gen1_jac_x0_with_hx(CnMat *Hx, CnMat *hx, const FLT dt,
																	  const SurviveKalmanModel *_x0,
																	  const FLT *sensor_pt, const SurvivePose *lh_p,
																	  const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanModel_LightMeas_y_gen1(dt, _x0, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanModel_LightMeas_y_gen1_jac_x0(Hx, dt, _x0, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanModel_LightMeas_y_gen1 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]
static inline void SurviveKalmanModel_LightMeas_y_gen1_jac_sensor_pt(CnMat *Hx, const FLT dt,
																	 const SurviveKalmanModel *_x0,
																	 const FLT *sensor_pt, const SurvivePose *lh_p,
																	 const BaseStationCal *bsc0) {
	const FLT x0 = dt * dt;
	const FLT x1 = x0 * ((*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x2 = x0 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x3 = x0 * ((*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x4 = 1e-10 + x1 + x2 + x3;
	const FLT x5 = sqrt(x4);
	const FLT x6 = 0.5 * x5;
	const FLT x7 = sin(x6);
	const FLT x8 = (1. / x4) * (x7 * x7);
	const FLT x9 = cos(x6);
	const FLT x10 = 1. / sqrt((x3 * x8) + (x1 * x8) + (x2 * x8) + (x9 * x9));
	const FLT x11 = (1. / x5) * x7 * dt * x10;
	const FLT x12 = x11 * (*_x0).Pose.Rot[0];
	const FLT x13 = x12 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x14 = x11 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x15 = x14 * (*_x0).Pose.Rot[1];
	const FLT x16 = x9 * x10;
	const FLT x17 = x16 * (*_x0).Pose.Rot[3];
	const FLT x18 = x11 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x19 = x18 * (*_x0).Pose.Rot[2];
	const FLT x20 = x17 + x19 + x13 + (-1 * x15);
	const FLT x21 = x16 * (*_x0).Pose.Rot[2];
	const FLT x22 = x12 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x23 = x18 * (*_x0).Pose.Rot[3];
	const FLT x24 = x11 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x25 = x24 * (*_x0).Pose.Rot[1];
	const FLT x26 = (-1 * x25) + x23 + (-1 * x21) + (-1 * x22);
	const FLT x27 = (-1 * x23) + x25 + x22 + x21;
	const FLT x28 = 2 * x27;
	const FLT x29 = 1 + (x28 * x26) + (-2 * (x20 * x20));
	const FLT x30 = (x16 * (*_x0).Pose.Rot[0]) + (-1 * x24 * (*_x0).Pose.Rot[3]) + (-1 * x18 * (*_x0).Pose.Rot[1]) +
					(-1 * x14 * (*_x0).Pose.Rot[2]);
	const FLT x31 = 2 * x30;
	const FLT x32 = x31 * x27;
	const FLT x33 = x24 * (*_x0).Pose.Rot[2];
	const FLT x34 = x14 * (*_x0).Pose.Rot[3];
	const FLT x35 = x16 * (*_x0).Pose.Rot[1];
	const FLT x36 = x12 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x37 = x36 + x35 + (-1 * x33) + x34;
	const FLT x38 = 2 * x20;
	const FLT x39 = (x38 * x37) + (-1 * x32);
	const FLT x40 = 2 * x37;
	const FLT x41 = x31 * x20;
	const FLT x42 = x41 + (-1 * x40 * x26);
	const FLT x43 = (x42 * (*lh_p).Rot[1]) + (-1 * x29 * (*lh_p).Rot[2]) + (x39 * (*lh_p).Rot[0]);
	const FLT x44 = 2 * (*lh_p).Rot[2];
	const FLT x45 = (x29 * (*lh_p).Rot[3]) + (-1 * x39 * (*lh_p).Rot[1]) + (x42 * (*lh_p).Rot[0]);
	const FLT x46 = 2 * (*lh_p).Rot[3];
	const FLT x47 = x29 + (x43 * x44) + (-1 * x45 * x46);
	const FLT x48 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x49 = (-1 * x20 * sensor_pt[1]) + (x27 * sensor_pt[2]) + (x30 * sensor_pt[0]);
	const FLT x50 = (-1 * x37 * sensor_pt[2]) + (x20 * sensor_pt[0]) + (x30 * sensor_pt[1]);
	const FLT x51 = sensor_pt[2] + (x48 * (*_x0).Acc[2]) + (2 * ((x50 * x37) + (-1 * x49 * x27))) +
					(dt * (*_x0).Velocity.Pos[2]) + (*_x0).Pose.Pos[2];
	const FLT x52 = (-1 * x27 * sensor_pt[0]) + (x30 * sensor_pt[2]) + (x37 * sensor_pt[1]);
	const FLT x53 = (*_x0).Pose.Pos[0] + sensor_pt[0] + (2 * ((x52 * x27) + (-1 * x50 * x20))) +
					(dt * (*_x0).Velocity.Pos[0]) + (x48 * (*_x0).Acc[0]);
	const FLT x54 = (2 * ((x49 * x20) + (-1 * x52 * x37))) + sensor_pt[1] + (*_x0).Pose.Pos[1] +
					(dt * (*_x0).Velocity.Pos[1]) + (x48 * (*_x0).Acc[1]);
	const FLT x55 = (-1 * x54 * (*lh_p).Rot[3]) + (x51 * (*lh_p).Rot[2]) + (x53 * (*lh_p).Rot[0]);
	const FLT x56 = (-1 * x51 * (*lh_p).Rot[1]) + (x54 * (*lh_p).Rot[0]) + (x53 * (*lh_p).Rot[3]);
	const FLT x57 = (2 * ((x56 * (*lh_p).Rot[1]) + (-1 * x55 * (*lh_p).Rot[2]))) + x51 + (*lh_p).Pos[2];
	const FLT x58 = 1. / x57;
	const FLT x59 = 2 * (*lh_p).Rot[1];
	const FLT x60 = (x29 * (*lh_p).Rot[0]) + (-1 * x42 * (*lh_p).Rot[3]) + (x39 * (*lh_p).Rot[2]);
	const FLT x61 = x39 + (x59 * x45) + (-1 * x60 * x44);
	const FLT x62 = x57 * x57;
	const FLT x63 = 1. / x62;
	const FLT x64 = (-1 * x53 * (*lh_p).Rot[2]) + (x51 * (*lh_p).Rot[0]) + (x54 * (*lh_p).Rot[1]);
	const FLT x65 = x53 + (2 * ((x64 * (*lh_p).Rot[2]) + (-1 * x56 * (*lh_p).Rot[3]))) + (*lh_p).Pos[0];
	const FLT x66 = x63 * x65;
	const FLT x67 = -1 * x57;
	const FLT x68 = x65 * x65;
	const FLT x69 = 2 * (1. / (x62 + x68)) * x62 * atan2(x65, x67) * (*bsc0).curve;
	const FLT x70 = x42 + (x60 * x46) + (-1 * x59 * x43);
	const FLT x71 = x54 + (*lh_p).Pos[1] + (2 * ((x55 * (*lh_p).Rot[3]) + (-1 * x64 * (*lh_p).Rot[1])));
	const FLT x72 = x71 * x63;
	const FLT x73 = x62 + (x71 * x71);
	const FLT x74 = 1. / x73;
	const FLT x75 = x74 * x62;
	const FLT x76 = 1. / sqrt(1 + (-1 * x74 * x68 * ((*bsc0).tilt * (*bsc0).tilt)));
	const FLT x77 = (1. / sqrt(x73)) * (*bsc0).tilt;
	const FLT x78 = 2 * x71;
	const FLT x79 = 2 * x57;
	const FLT x80 = 1.0 / 2.0 * (1. / (x73 * sqrt(x73))) * x65 * (*bsc0).tilt;
	const FLT x81 = (-1 * x76 * ((-1 * ((x79 * x61) + (x70 * x78)) * x80) + (x77 * x47))) +
					(-1 * ((-1 * x72 * x61) + (x70 * x58)) * x75);
	const FLT x82 = (*bsc0).gibmag * sin(1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha +
										 (-1 * asin(x77 * x65)) + (-1 * atan2(-1 * x71, x67)));
	const FLT x83 = (-1 * x17) + x15 + (-1 * x19) + (-1 * x13);
	const FLT x84 = x31 * x37;
	const FLT x85 = x84 + (-1 * x83 * x28);
	const FLT x86 = (x40 * x27) + (-1 * x41);
	const FLT x87 = 1 + (x83 * x38) + (-2 * (x37 * x37));
	const FLT x88 = 2 * ((x87 * (*lh_p).Rot[1]) + (x85 * (*lh_p).Rot[0]) + (-1 * x86 * (*lh_p).Rot[2]));
	const FLT x89 = 2 * ((x86 * (*lh_p).Rot[3]) + (-1 * x85 * (*lh_p).Rot[1]) + (x87 * (*lh_p).Rot[0]));
	const FLT x90 = x86 + (x88 * (*lh_p).Rot[2]) + (-1 * x89 * (*lh_p).Rot[3]);
	const FLT x91 = 2 * ((x86 * (*lh_p).Rot[0]) + (x85 * (*lh_p).Rot[2]) + (-1 * x87 * (*lh_p).Rot[3]));
	const FLT x92 = x85 + (x89 * (*lh_p).Rot[1]) + (-1 * x91 * (*lh_p).Rot[2]);
	const FLT x93 = (x91 * (*lh_p).Rot[3]) + x87 + (-1 * x88 * (*lh_p).Rot[1]);
	const FLT x94 = (-1 * x76 * ((-1 * ((x79 * x92) + (x78 * x93)) * x80) + (x77 * x90))) +
					(-1 * ((-1 * x72 * x92) + (x58 * x93)) * x75);
	const FLT x95 = x33 + (-1 * x36) + (-1 * x34) + (-1 * x35);
	const FLT x96 = x32 + (-1 * x95 * x38);
	const FLT x97 = 1 + (x95 * x40) + (-2 * (x27 * x27));
	const FLT x98 = (x38 * x27) + (-1 * x84);
	const FLT x99 = (-1 * x96 * (*lh_p).Rot[2]) + (x98 * (*lh_p).Rot[1]) + (x97 * (*lh_p).Rot[0]);
	const FLT x100 = (x96 * (*lh_p).Rot[3]) + (-1 * x97 * (*lh_p).Rot[1]) + (x98 * (*lh_p).Rot[0]);
	const FLT x101 = x96 + (x99 * x44) + (-1 * x46 * x100);
	const FLT x102 = (x96 * (*lh_p).Rot[0]) + (-1 * x98 * (*lh_p).Rot[3]) + (x97 * (*lh_p).Rot[2]);
	const FLT x103 = x97 + (x59 * x100) + (-1 * x44 * x102);
	const FLT x104 = (x46 * x102) + x98 + (-1 * x59 * x99);
	const FLT x105 = (-1 * x76 * ((-1 * ((x79 * x103) + (x78 * x104)) * x80) + (x77 * x101))) +
					 (-1 * ((-1 * x72 * x103) + (x58 * x104)) * x75);
	cnMatrixOptionalSet(Hx, 0, 0, x81 + (((x61 * x66) + (-1 * x58 * x47)) * x69) + (x81 * x82));
	cnMatrixOptionalSet(Hx, 0, 1, (((x66 * x92) + (-1 * x58 * x90)) * x69) + x94 + (x82 * x94));
	cnMatrixOptionalSet(Hx, 0, 2, x105 + (((x66 * x103) + (-1 * x58 * x101)) * x69) + (x82 * x105));
}

// Full version Jacobian of SurviveKalmanModel_LightMeas_y_gen1 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]

static inline void SurviveKalmanModel_LightMeas_y_gen1_jac_sensor_pt_with_hx(CnMat *Hx, CnMat *hx, const FLT dt,
																			 const SurviveKalmanModel *_x0,
																			 const FLT *sensor_pt,
																			 const SurvivePose *lh_p,
																			 const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanModel_LightMeas_y_gen1(dt, _x0, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanModel_LightMeas_y_gen1_jac_sensor_pt(Hx, dt, _x0, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanModel_LightMeas_y_gen1 wrt [(*lh_p).Pos[0], (*lh_p).Pos[1], (*lh_p).Pos[2], (*lh_p).Rot[0],
// (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]
static inline void SurviveKalmanModel_LightMeas_y_gen1_jac_lh_p(CnMat *Hx, const FLT dt, const SurviveKalmanModel *_x0,
																const FLT *sensor_pt, const SurvivePose *lh_p,
																const BaseStationCal *bsc0) {
	const FLT x0 = dt * (*_x0).Velocity.Pos[2];
	const FLT x1 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x2 = x1 * (*_x0).Acc[2];
	const FLT x3 = dt * dt;
	const FLT x4 = x3 * ((*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x5 = x3 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x6 = x3 * ((*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x7 = 1e-10 + x4 + x5 + x6;
	const FLT x8 = sqrt(x7);
	const FLT x9 = 0.5 * x8;
	const FLT x10 = sin(x9);
	const FLT x11 = (1. / x7) * (x10 * x10);
	const FLT x12 = cos(x9);
	const FLT x13 = 1. / sqrt((x6 * x11) + (x5 * x11) + (x4 * x11) + (x12 * x12));
	const FLT x14 = (1. / x8) * dt * x13 * x10;
	const FLT x15 = x14 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x16 = x13 * x12;
	const FLT x17 = x14 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x18 = x14 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x19 = (-1 * x18 * (*_x0).Pose.Rot[3]) + (x17 * (*_x0).Pose.Rot[1]) + (x15 * (*_x0).Pose.Rot[0]) +
					(x16 * (*_x0).Pose.Rot[2]);
	const FLT x20 = (x16 * (*_x0).Pose.Rot[0]) + (-1 * x18 * (*_x0).Pose.Rot[1]) + (-1 * x17 * (*_x0).Pose.Rot[3]) +
					(-1 * x15 * (*_x0).Pose.Rot[2]);
	const FLT x21 = (x18 * (*_x0).Pose.Rot[2]) + (x16 * (*_x0).Pose.Rot[3]) + (x17 * (*_x0).Pose.Rot[0]) +
					(-1 * x15 * (*_x0).Pose.Rot[1]);
	const FLT x22 = (-1 * x21 * sensor_pt[1]) + (x19 * sensor_pt[2]) + (x20 * sensor_pt[0]);
	const FLT x23 = (-1 * x17 * (*_x0).Pose.Rot[2]) + (x18 * (*_x0).Pose.Rot[0]) + (x16 * (*_x0).Pose.Rot[1]) +
					(x15 * (*_x0).Pose.Rot[3]);
	const FLT x24 = (x21 * sensor_pt[0]) + (-1 * x23 * sensor_pt[2]) + (x20 * sensor_pt[1]);
	const FLT x25 = 2 * ((x24 * x23) + (-1 * x22 * x19));
	const FLT x26 = x25 + sensor_pt[2] + x2 + x0 + (*_x0).Pose.Pos[2];
	const FLT x27 = x26 * (*lh_p).Rot[2];
	const FLT x28 = dt * (*_x0).Velocity.Pos[0];
	const FLT x29 = x1 * (*_x0).Acc[0];
	const FLT x30 = (x20 * sensor_pt[2]) + (-1 * x19 * sensor_pt[0]) + (x23 * sensor_pt[1]);
	const FLT x31 = 2 * ((x30 * x19) + (-1 * x24 * x21));
	const FLT x32 = (*_x0).Pose.Pos[0] + sensor_pt[0] + x28 + x31 + x29;
	const FLT x33 = x32 * (*lh_p).Rot[0];
	const FLT x34 = x1 * (*_x0).Acc[1];
	const FLT x35 = dt * (*_x0).Velocity.Pos[1];
	const FLT x36 = 2 * ((x22 * x21) + (-1 * x30 * x23));
	const FLT x37 = x35 + sensor_pt[1] + x36 + (*_x0).Pose.Pos[1] + x34;
	const FLT x38 = x37 * (*lh_p).Rot[3];
	const FLT x39 = (-1 * x38) + x27 + x33;
	const FLT x40 = x37 * (*lh_p).Rot[0];
	const FLT x41 = x32 * (*lh_p).Rot[3];
	const FLT x42 = x26 * (*lh_p).Rot[1];
	const FLT x43 = (-1 * x42) + x40 + x41;
	const FLT x44 = x26 + (2 * ((x43 * (*lh_p).Rot[1]) + (-1 * x39 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x45 = x44 * x44;
	const FLT x46 = x26 * (*lh_p).Rot[0];
	const FLT x47 = x37 * (*lh_p).Rot[1];
	const FLT x48 = x32 * (*lh_p).Rot[2];
	const FLT x49 = (-1 * x48) + x46 + x47;
	const FLT x50 = x37 + (*lh_p).Pos[1] + (2 * ((x39 * (*lh_p).Rot[3]) + (-1 * x49 * (*lh_p).Rot[1])));
	const FLT x51 = (x50 * x50) + x45;
	const FLT x52 = 1. / x51;
	const FLT x53 = x32 + (2 * ((x49 * (*lh_p).Rot[2]) + (-1 * x43 * (*lh_p).Rot[3]))) + (*lh_p).Pos[0];
	const FLT x54 = x53 * x53;
	const FLT x55 = 1. / sqrt(1 + (-1 * x54 * x52 * ((*bsc0).tilt * (*bsc0).tilt)));
	const FLT x56 = (1. / sqrt(x51)) * (*bsc0).tilt;
	const FLT x57 = x56 * x55;
	const FLT x58 = 2 * x44;
	const FLT x59 = -1 * x44;
	const FLT x60 = (1. / (x45 + x54)) * atan2(x53, x59) * (*bsc0).curve;
	const FLT x61 = (*bsc0).gibmag * sin(1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha +
										 (-1 * asin(x53 * x56)) + (-1 * atan2(-1 * x50, x59)));
	const FLT x62 = x53 * (1. / (x51 * sqrt(x51))) * (*bsc0).tilt;
	const FLT x63 = x62 * x55;
	const FLT x64 = (x63 * x50) + (-1 * x52 * x44);
	const FLT x65 = 2 * x60;
	const FLT x66 = (x63 * x44) + (x50 * x52);
	const FLT x67 = 2 * x38;
	const FLT x68 = (2 * x27) + (-1 * x67);
	const FLT x69 = 1. / x44;
	const FLT x70 = 2 * x48;
	const FLT x71 = (2 * x47) + (-1 * x70);
	const FLT x72 = 1. / x45;
	const FLT x73 = x72 * x53;
	const FLT x74 = x65 * x45;
	const FLT x75 = 2 * x42;
	const FLT x76 = (2 * x41) + (-1 * x75);
	const FLT x77 = x72 * x50;
	const FLT x78 = x52 * x45;
	const FLT x79 = 2 * x50;
	const FLT x80 = 1.0 / 2.0 * x62;
	const FLT x81 = (-1 * x55 * ((-1 * ((x71 * x58) + (x79 * x76)) * x80) + (x68 * x56))) +
					(-1 * ((-1 * x71 * x77) + (x76 * x69)) * x78);
	const FLT x82 = (-1 * sensor_pt[2]) + (-1 * x2) + (-1 * x25) + (-1 * (*_x0).Pose.Pos[2]) + (-1 * x0);
	const FLT x83 = 2 * (*lh_p).Rot[3];
	const FLT x84 = 2 * (*lh_p).Rot[2];
	const FLT x85 = (x84 * x37) + (-1 * x82 * x83);
	const FLT x86 = 2 * (*lh_p).Rot[1];
	const FLT x87 = 2 * x40;
	const FLT x88 = x76 + (x82 * x86) + x87;
	const FLT x89 = 2 * x46;
	const FLT x90 = (-4 * x47) + (-1 * x89) + x70;
	const FLT x91 = (-1 * x55 * ((-1 * ((x88 * x58) + (x79 * x90)) * x80) + (x85 * x56))) +
					(-1 * ((-1 * x88 * x77) + (x69 * x90)) * x78);
	const FLT x92 = (-1 * (*_x0).Pose.Pos[0]) + (-1 * x28) + (-1 * x31) + (-1 * x29) + (-1 * sensor_pt[0]);
	const FLT x93 = x71 + x89 + (x84 * x92);
	const FLT x94 = 2 * x33;
	const FLT x95 = (-1 * x94) + (-4 * x27) + x67;
	const FLT x96 = (x83 * x26) + (-1 * x86 * x92);
	const FLT x97 = (-1 * x55 * ((-1 * ((x58 * x95) + (x79 * x96)) * x80) + (x56 * x93))) +
					(-1 * ((-1 * x77 * x95) + (x69 * x96)) * x78);
	const FLT x98 = (-1 * x87) + x75 + (-4 * x41);
	const FLT x99 = (-1 * x36) + (-1 * (*_x0).Pose.Pos[1]) + (-1 * x34) + (-1 * x35) + (-1 * sensor_pt[1]);
	const FLT x100 = (x86 * x32) + (-1 * x84 * x99);
	const FLT x101 = x68 + (x83 * x99) + x94;
	const FLT x102 = (-1 * x55 * ((-1 * ((x58 * x100) + (x79 * x101)) * x80) + (x56 * x98))) +
					 (-1 * ((-1 * x77 * x100) + (x69 * x101)) * x78);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[0]) / sizeof(FLT),
						(-1 * x61 * x57) + (-1 * x57) + (-1 * x60 * x58));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[1]) / sizeof(FLT), x64 + (x64 * x61));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[2]) / sizeof(FLT), x66 + (x65 * x53) + (x61 * x66));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[0]) / sizeof(FLT),
						x81 + (((x71 * x73) + (-1 * x68 * x69)) * x74) + (x81 * x61));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[1]) / sizeof(FLT),
						(((x88 * x73) + (-1 * x85 * x69)) * x74) + x91 + (x61 * x91));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[2]) / sizeof(FLT),
						x97 + (((x73 * x95) + (-1 * x69 * x93)) * x74) + (x61 * x97));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[3]) / sizeof(FLT),
						(x74 * ((x73 * x100) + (-1 * x69 * x98))) + x102 + (x61 * x102));
}

// Full version Jacobian of SurviveKalmanModel_LightMeas_y_gen1 wrt [(*lh_p).Pos[0], (*lh_p).Pos[1], (*lh_p).Pos[2],
// (*lh_p).Rot[0], (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]

static inline void SurviveKalmanModel_LightMeas_y_gen1_jac_lh_p_with_hx(CnMat *Hx, CnMat *hx, const FLT dt,
																		const SurviveKalmanModel *_x0,
																		const FLT *sensor_pt, const SurvivePose *lh_p,
																		const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanModel_LightMeas_y_gen1(dt, _x0, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanModel_LightMeas_y_gen1_jac_lh_p(Hx, dt, _x0, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanModel_LightMeas_y_gen1 wrt [<cnkalman.codegen.WrapMember object at 0x7f88f4f502b0>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4f501f0>, <cnkalman.codegen.WrapMember object at 0x7f88f4f50640>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4ed3c70>, <cnkalman.codegen.WrapMember object at 0x7f88f4ed3c10>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4f50100>, <cnkalman.codegen.WrapMember object at 0x7f88f4f50250>]
static inline void SurviveKalmanModel_LightMeas_y_gen1_jac_bsc0(CnMat *Hx, const FLT dt, const SurviveKalmanModel *_x0,
																const FLT *sensor_pt, const SurvivePose *lh_p,
																const BaseStationCal *bsc0) {
	const FLT x0 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x1 = dt * dt;
	const FLT x2 = x1 * ((*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x3 = x1 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x4 = x1 * ((*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x5 = 1e-10 + x2 + x3 + x4;
	const FLT x6 = sqrt(x5);
	const FLT x7 = 0.5 * x6;
	const FLT x8 = sin(x7);
	const FLT x9 = (1. / x5) * (x8 * x8);
	const FLT x10 = cos(x7);
	const FLT x11 = 1. / sqrt((x4 * x9) + (x3 * x9) + (x2 * x9) + (x10 * x10));
	const FLT x12 = (1. / x6) * x8 * dt * x11;
	const FLT x13 = x12 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x14 = x12 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x15 = x11 * x10;
	const FLT x16 = x12 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x17 = (-1 * x16 * (*_x0).Pose.Rot[1]) + (x15 * (*_x0).Pose.Rot[0]) + (-1 * x13 * (*_x0).Pose.Rot[3]) +
					(-1 * x14 * (*_x0).Pose.Rot[2]);
	const FLT x18 = (x15 * (*_x0).Pose.Rot[1]) + (x16 * (*_x0).Pose.Rot[0]) + (-1 * x13 * (*_x0).Pose.Rot[2]) +
					(x14 * (*_x0).Pose.Rot[3]);
	const FLT x19 = (x13 * (*_x0).Pose.Rot[1]) + (-1 * x16 * (*_x0).Pose.Rot[3]) + (x14 * (*_x0).Pose.Rot[0]) +
					(x15 * (*_x0).Pose.Rot[2]);
	const FLT x20 = (-1 * x19 * sensor_pt[0]) + (x17 * sensor_pt[2]) + (x18 * sensor_pt[1]);
	const FLT x21 = (x16 * (*_x0).Pose.Rot[2]) + (x15 * (*_x0).Pose.Rot[3]) + (x13 * (*_x0).Pose.Rot[0]) +
					(-1 * x14 * (*_x0).Pose.Rot[1]);
	const FLT x22 = (-1 * x21 * sensor_pt[1]) + (x19 * sensor_pt[2]) + (x17 * sensor_pt[0]);
	const FLT x23 = (dt * (*_x0).Velocity.Pos[1]) + (2 * ((x22 * x21) + (-1 * x20 * x18))) + sensor_pt[1] +
					(*_x0).Pose.Pos[1] + (x0 * (*_x0).Acc[1]);
	const FLT x24 = (-1 * x18 * sensor_pt[2]) + (x21 * sensor_pt[0]) + (x17 * sensor_pt[1]);
	const FLT x25 = (2 * ((x20 * x19) + (-1 * x24 * x21))) + (*_x0).Pose.Pos[0] + sensor_pt[0] +
					(dt * (*_x0).Velocity.Pos[0]) + (x0 * (*_x0).Acc[0]);
	const FLT x26 = sensor_pt[2] + (2 * ((x24 * x18) + (-1 * x22 * x19))) + (x0 * (*_x0).Acc[2]) +
					(dt * (*_x0).Velocity.Pos[2]) + (*_x0).Pose.Pos[2];
	const FLT x27 = (-1 * x26 * (*lh_p).Rot[1]) + (x23 * (*lh_p).Rot[0]) + (x25 * (*lh_p).Rot[3]);
	const FLT x28 = (-1 * x25 * (*lh_p).Rot[2]) + (x26 * (*lh_p).Rot[0]) + (x23 * (*lh_p).Rot[1]);
	const FLT x29 = (2 * ((x28 * (*lh_p).Rot[2]) + (-1 * x27 * (*lh_p).Rot[3]))) + x25 + (*lh_p).Pos[0];
	const FLT x30 = (-1 * x23 * (*lh_p).Rot[3]) + (x26 * (*lh_p).Rot[2]) + (x25 * (*lh_p).Rot[0]);
	const FLT x31 = x26 + (2 * ((x27 * (*lh_p).Rot[1]) + (-1 * x30 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x32 = -1 * x31;
	const FLT x33 = x23 + (*lh_p).Pos[1] + (2 * ((x30 * (*lh_p).Rot[3]) + (-1 * x28 * (*lh_p).Rot[1])));
	const FLT x34 = (x33 * x33) + (x31 * x31);
	const FLT x35 = (1. / sqrt(x34)) * x29;
	const FLT x36 = 1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha + (-1 * asin(x35 * (*bsc0).tilt)) +
					(-1 * atan2(-1 * x33, x32));
	const FLT x37 = sin(x36) * (*bsc0).gibmag;
	const FLT x38 = x35 * (1. / sqrt(1 + (-1 * (1. / x34) * (x29 * x29) * ((*bsc0).tilt * (*bsc0).tilt))));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, curve) / sizeof(FLT), atan2(x29, x32) * atan2(x29, x32));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibmag) / sizeof(FLT), -1 * cos(x36));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibpha) / sizeof(FLT), x37);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, phase) / sizeof(FLT), -1 + (-1 * x37));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, tilt) / sizeof(FLT), (-1 * x38 * x37) + (-1 * x38));
}

// Full version Jacobian of SurviveKalmanModel_LightMeas_y_gen1 wrt [<cnkalman.codegen.WrapMember object at
// 0x7f88f4f502b0>, <cnkalman.codegen.WrapMember object at 0x7f88f4f501f0>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4f50640>, <cnkalman.codegen.WrapMember object at 0x7f88f4ed3c70>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4ed3c10>, <cnkalman.codegen.WrapMember object at 0x7f88f4f50100>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4f50250>]

static inline void SurviveKalmanModel_LightMeas_y_gen1_jac_bsc0_with_hx(CnMat *Hx, CnMat *hx, const FLT dt,
																		const SurviveKalmanModel *_x0,
																		const FLT *sensor_pt, const SurvivePose *lh_p,
																		const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanModel_LightMeas_y_gen1(dt, _x0, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanModel_LightMeas_y_gen1_jac_bsc0(Hx, dt, _x0, sensor_pt, lh_p, bsc0);
	}
}
static inline FLT SurviveKalmanModel_LightMeas_x_gen2(const FLT dt, const SurviveKalmanModel *_x0, const FLT *sensor_pt,
													  const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const FLT x0 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x1 = dt * dt;
	const FLT x2 = x1 * ((*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x3 = x1 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x4 = x1 * ((*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x5 = 1e-10 + x2 + x3 + x4;
	const FLT x6 = sqrt(x5);
	const FLT x7 = 0.5 * x6;
	const FLT x8 = sin(x7);
	const FLT x9 = (1. / x5) * (x8 * x8);
	const FLT x10 = cos(x7);
	const FLT x11 = 1. / sqrt((x4 * x9) + (x3 * x9) + (x2 * x9) + (x10 * x10));
	const FLT x12 = (1. / x6) * x8 * dt * x11;
	const FLT x13 = x12 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x14 = x11 * x10;
	const FLT x15 = x12 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x16 = x12 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x17 = (-1 * x16 * (*_x0).Pose.Rot[3]) + (x15 * (*_x0).Pose.Rot[1]) + (x13 * (*_x0).Pose.Rot[0]) +
					(x14 * (*_x0).Pose.Rot[2]);
	const FLT x18 = (-1 * x16 * (*_x0).Pose.Rot[1]) + (x14 * (*_x0).Pose.Rot[0]) + (-1 * x15 * (*_x0).Pose.Rot[3]) +
					(-1 * x13 * (*_x0).Pose.Rot[2]);
	const FLT x19 = (x14 * (*_x0).Pose.Rot[3]) + (x15 * (*_x0).Pose.Rot[0]) + (x16 * (*_x0).Pose.Rot[2]) +
					(-1 * x13 * (*_x0).Pose.Rot[1]);
	const FLT x20 = (-1 * x19 * sensor_pt[1]) + (x17 * sensor_pt[2]) + (x18 * sensor_pt[0]);
	const FLT x21 = (x16 * (*_x0).Pose.Rot[0]) + (x14 * (*_x0).Pose.Rot[1]) + (-1 * x15 * (*_x0).Pose.Rot[2]) +
					(x13 * (*_x0).Pose.Rot[3]);
	const FLT x22 = (x19 * sensor_pt[0]) + (-1 * x21 * sensor_pt[2]) + (x18 * sensor_pt[1]);
	const FLT x23 = (2 * ((x22 * x21) + (-1 * x20 * x17))) + sensor_pt[2] + (x0 * (*_x0).Acc[2]) +
					(dt * (*_x0).Velocity.Pos[2]) + (*_x0).Pose.Pos[2];
	const FLT x24 = (-1 * x17 * sensor_pt[0]) + (x18 * sensor_pt[2]) + (x21 * sensor_pt[1]);
	const FLT x25 = (*_x0).Pose.Pos[0] + (2 * ((x24 * x17) + (-1 * x22 * x19))) + sensor_pt[0] +
					(dt * (*_x0).Velocity.Pos[0]) + (x0 * (*_x0).Acc[0]);
	const FLT x26 = (2 * ((x20 * x19) + (-1 * x24 * x21))) + (dt * (*_x0).Velocity.Pos[1]) + sensor_pt[1] +
					(*_x0).Pose.Pos[1] + (x0 * (*_x0).Acc[1]);
	const FLT x27 = (-1 * x26 * (*lh_p).Rot[3]) + (x23 * (*lh_p).Rot[2]) + (x25 * (*lh_p).Rot[0]);
	const FLT x28 = (-1 * x23 * (*lh_p).Rot[1]) + (x26 * (*lh_p).Rot[0]) + (x25 * (*lh_p).Rot[3]);
	const FLT x29 = x23 + (2 * ((x28 * (*lh_p).Rot[1]) + (-1 * x27 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x30 = (-1 * x25 * (*lh_p).Rot[2]) + (x23 * (*lh_p).Rot[0]) + (x26 * (*lh_p).Rot[1]);
	const FLT x31 = (2 * ((x30 * (*lh_p).Rot[2]) + (-1 * x28 * (*lh_p).Rot[3]))) + x25 + (*lh_p).Pos[0];
	const FLT x32 = atan2(-1 * x29, x31);
	const FLT x33 = (*lh_p).Pos[1] + x26 + (2 * ((x27 * (*lh_p).Rot[3]) + (-1 * x30 * (*lh_p).Rot[1])));
	const FLT x34 = (x31 * x31) + (x29 * x29);
	const FLT x35 = 0.523598775598299 + (*bsc0).tilt;
	const FLT x36 = cos(x35);
	const FLT x37 = asin(x33 * (1. / x36) * (1. / sqrt(x34 + (x33 * x33))));
	const FLT x38 = (1. / sqrt(x34)) * x33 * tan(x35);
	const FLT x39 = ((*bsc0).ogeemag * sin(x32 + (-1 * asin(x38)) + (*bsc0).ogeephase)) + (*bsc0).curve;
	const FLT x40 = 0.0028679863 + (x37 * (-8.0108022e-06 + (-8.0108022e-06 * x37)));
	const FLT x41 = 5.3685255e-06 + (x40 * x37);
	const FLT x42 = 0.0076069798 + (x41 * x37);
	const FLT x43 =
		(-1 *
		 asin(x38 + (x42 * (x37 * x37) * x39 *
					 (1. / (x36 + (-1 * x39 * sin(x35) *
								   ((x37 * (x42 + (x37 * (x41 + (x37 * (x40 + (x37 * (-8.0108022e-06 +
																					  (-1.60216044e-05 * x37))))))))) +
									(x42 * x37)))))))) +
		x32;
	return -1.5707963267949 + x43 + (-1 * (*bsc0).phase) + (sin(x43 + (*bsc0).gibpha) * (*bsc0).gibmag);
}

// Jacobian of SurviveKalmanModel_LightMeas_x_gen2 wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2],
// (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3],
// (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1],
// (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1],
// (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4edc580>]
static inline void SurviveKalmanModel_LightMeas_x_gen2_jac_x0(CnMat *Hx, const FLT dt, const SurviveKalmanModel *_x0,
															  const FLT *sensor_pt, const SurvivePose *lh_p,
															  const BaseStationCal *bsc0) {
	const FLT x0 = dt * fabs(dt);
	const FLT x1 = x0 * (*lh_p).Rot[1] * (*lh_p).Rot[2];
	const FLT x2 = x0 * (*lh_p).Rot[3];
	const FLT x3 = x2 * (*lh_p).Rot[0];
	const FLT x4 = x3 + x1;
	const FLT x5 = 1.0 / 2.0 * x0;
	const FLT x6 = dt * dt;
	const FLT x7 = (*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x8 = x6 * x7;
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x10 = x6 * x9;
	const FLT x11 = (*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x12 = x6 * x11;
	const FLT x13 = 1e-10 + x8 + x10 + x12;
	const FLT x14 = sqrt(x13);
	const FLT x15 = 0.5 * x14;
	const FLT x16 = sin(x15);
	const FLT x17 = x16 * x16;
	const FLT x18 = 1. / x13;
	const FLT x19 = x18 * x17;
	const FLT x20 = cos(x15);
	const FLT x21 = (x10 * x19) + (x8 * x19) + (x12 * x19) + (x20 * x20);
	const FLT x22 = 1. / sqrt(x21);
	const FLT x23 = x22 * x16;
	const FLT x24 = 1. / x14;
	const FLT x25 = dt * x24;
	const FLT x26 = x25 * x23;
	const FLT x27 = x26 * (*_x0).Pose.Rot[0];
	const FLT x28 = x22 * x20;
	const FLT x29 = x28 * (*_x0).Pose.Rot[2];
	const FLT x30 = x26 * (*_x0).Pose.Rot[1];
	const FLT x31 = x23 * (*_x0).Pose.Rot[3];
	const FLT x32 = x31 * x25;
	const FLT x33 = (-1 * x32 * (*_x0).Velocity.AxisAngleRot[0]) + (x30 * (*_x0).Velocity.AxisAngleRot[2]) +
					(x27 * (*_x0).Velocity.AxisAngleRot[1]) + x29;
	const FLT x34 = x26 * (*_x0).Pose.Rot[2];
	const FLT x35 = x28 * (*_x0).Pose.Rot[0];
	const FLT x36 = (-1 * x30 * (*_x0).Velocity.AxisAngleRot[0]) + x35 + (-1 * x32 * (*_x0).Velocity.AxisAngleRot[2]) +
					(-1 * x34 * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x37 = x28 * (*_x0).Pose.Rot[3];
	const FLT x38 = (x34 * (*_x0).Velocity.AxisAngleRot[0]) + x37 + (x27 * (*_x0).Velocity.AxisAngleRot[2]) +
					(-1 * x30 * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x39 = (-1 * x38 * sensor_pt[1]) + (x33 * sensor_pt[2]) + (x36 * sensor_pt[0]);
	const FLT x40 = x28 * (*_x0).Pose.Rot[1];
	const FLT x41 = (x27 * (*_x0).Velocity.AxisAngleRot[0]) + x40 + (-1 * x34 * (*_x0).Velocity.AxisAngleRot[2]) +
					(x32 * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x42 = (-1 * x41 * sensor_pt[2]) + (x38 * sensor_pt[0]) + (x36 * sensor_pt[1]);
	const FLT x43 = (2 * ((x41 * x42) + (-1 * x33 * x39))) + sensor_pt[2] + (x5 * (*_x0).Acc[2]) +
					(dt * (*_x0).Velocity.Pos[2]) + (*_x0).Pose.Pos[2];
	const FLT x44 = (-1 * x33 * sensor_pt[0]) + (x36 * sensor_pt[2]) + (x41 * sensor_pt[1]);
	const FLT x45 = (dt * (*_x0).Velocity.Pos[1]) + (2 * ((x38 * x39) + (-1 * x41 * x44))) + (*_x0).Pose.Pos[1] +
					sensor_pt[1] + (x5 * (*_x0).Acc[1]);
	const FLT x46 = (2 * ((x44 * x33) + (-1 * x42 * x38))) + (*_x0).Pose.Pos[0] + sensor_pt[0] +
					(dt * (*_x0).Velocity.Pos[0]) + (x5 * (*_x0).Acc[0]);
	const FLT x47 = (-1 * x46 * (*lh_p).Rot[2]) + (x43 * (*lh_p).Rot[0]) + (x45 * (*lh_p).Rot[1]);
	const FLT x48 = (-1 * x45 * (*lh_p).Rot[3]) + (x43 * (*lh_p).Rot[2]) + (x46 * (*lh_p).Rot[0]);
	const FLT x49 = x45 + (*lh_p).Pos[1] + (2 * ((x48 * (*lh_p).Rot[3]) + (-1 * x47 * (*lh_p).Rot[1])));
	const FLT x50 = x49 * x49;
	const FLT x51 = (x45 * (*lh_p).Rot[0]) + (-1 * x43 * (*lh_p).Rot[1]) + (x46 * (*lh_p).Rot[3]);
	const FLT x52 = (2 * ((x51 * (*lh_p).Rot[1]) + (-1 * x48 * (*lh_p).Rot[2]))) + x43 + (*lh_p).Pos[2];
	const FLT x53 = x46 + (2 * ((x47 * (*lh_p).Rot[2]) + (-1 * x51 * (*lh_p).Rot[3]))) + (*lh_p).Pos[0];
	const FLT x54 = x53 * x53;
	const FLT x55 = x54 + (x52 * x52);
	const FLT x56 = x55 + x50;
	const FLT x57 = 0.523598775598299 + (*bsc0).tilt;
	const FLT x58 = cos(x57);
	const FLT x59 = 1. / x58;
	const FLT x60 = (1. / sqrt(x56)) * x59;
	const FLT x61 = 2 * x49;
	const FLT x62 = (*lh_p).Rot[2] * (*lh_p).Rot[2];
	const FLT x63 = -1 * x0 * x62;
	const FLT x64 = (*lh_p).Rot[3] * (*lh_p).Rot[3];
	const FLT x65 = x5 + (-1 * x0 * x64);
	const FLT x66 = x65 + x63;
	const FLT x67 = 2 * x53;
	const FLT x68 = x0 * (*lh_p).Rot[0];
	const FLT x69 = x68 * (*lh_p).Rot[2];
	const FLT x70 = x2 * (*lh_p).Rot[1];
	const FLT x71 = x70 + (-1 * x69);
	const FLT x72 = 2 * x52;
	const FLT x73 = (x71 * x72) + (x67 * x66);
	const FLT x74 = 1.0 / 2.0 * x49;
	const FLT x75 = x74 * (1. / (x56 * sqrt(x56))) * x59;
	const FLT x76 = (-1 * x75 * (x73 + (x4 * x61))) + (x4 * x60);
	const FLT x77 = 1. / sqrt(1 + (-1 * x50 * (1. / (x58 * x58)) * (1. / x56)));
	const FLT x78 = atan2(-1 * x52, x53);
	const FLT x79 = tan(x57);
	const FLT x80 = x79 * (1. / sqrt(x55));
	const FLT x81 = x80 * x49;
	const FLT x82 = (-1 * asin(x81)) + x78 + (*bsc0).ogeephase;
	const FLT x83 = (sin(x82) * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x84 = asin(x60 * x49);
	const FLT x85 = 8.0108022e-06 * x84;
	const FLT x86 = -8.0108022e-06 + (-1 * x85);
	const FLT x87 = 0.0028679863 + (x84 * x86);
	const FLT x88 = 5.3685255e-06 + (x84 * x87);
	const FLT x89 = 0.0076069798 + (x88 * x84);
	const FLT x90 = x89 * x84;
	const FLT x91 = -8.0108022e-06 + (-1.60216044e-05 * x84);
	const FLT x92 = x87 + (x84 * x91);
	const FLT x93 = x88 + (x84 * x92);
	const FLT x94 = x89 + (x84 * x93);
	const FLT x95 = (x84 * x94) + x90;
	const FLT x96 = sin(x57);
	const FLT x97 = x83 * x96;
	const FLT x98 = x58 + (-1 * x97 * x95);
	const FLT x99 = 1. / x98;
	const FLT x100 = 2 * x83 * x90 * x99;
	const FLT x101 = x77 * x100;
	const FLT x102 = x87 * x77;
	const FLT x103 = x86 * x77;
	const FLT x104 = x76 * x103;
	const FLT x105 = x85 * x77;
	const FLT x106 = (x84 * ((-1 * x76 * x105) + x104)) + (x76 * x102);
	const FLT x107 = x88 * x77;
	const FLT x108 = (x76 * x107) + (x84 * x106);
	const FLT x109 = x84 * x84;
	const FLT x110 = x83 * x109;
	const FLT x111 = x99 * x110;
	const FLT x112 = 1. / x55;
	const FLT x113 = 1. / sqrt(1 + (-1 * (x79 * x79) * x50 * x112));
	const FLT x114 = x79 * x74 * (1. / (x55 * sqrt(x55)));
	const FLT x115 = (x4 * x80) + (-1 * x73 * x114);
	const FLT x116 = (1. / x54) * x52;
	const FLT x117 = 1. / x53;
	const FLT x118 = x54 * x112;
	const FLT x119 = ((-1 * x71 * x117) + (x66 * x116)) * x118;
	const FLT x120 = x119 + (-1 * x113 * x115);
	const FLT x121 = cos(x82) * (*bsc0).ogeemag;
	const FLT x122 = x96 * x95;
	const FLT x123 = x122 * x121;
	const FLT x124 = x77 * x94;
	const FLT x125 = 2.40324066e-05 * x84;
	const FLT x126 = x77 * x125;
	const FLT x127 = x77 * x91;
	const FLT x128 = x77 * x92;
	const FLT x129 = x77 * x93;
	const FLT x130 = x89 * x77;
	const FLT x131 = x89 * (1. / (x98 * x98)) * x110;
	const FLT x132 = x89 * x99 * x109;
	const FLT x133 = x121 * x132;
	const FLT x134 = x81 + (x89 * x111);
	const FLT x135 = 1. / sqrt(1 + (-1 * (x134 * x134)));
	const FLT x136 =
		x119 + (-1 * x135 *
				(x115 +
				 (-1 * x131 *
				  ((-1 * x97 *
					((x76 * x130) + (x84 * x108) + (x76 * x124) +
					 (x84 * (x108 + (x84 * (x106 + (x84 * ((x76 * x127) + x104 + (-1 * x76 * x126))) + (x76 * x128))) +
							 (x76 * x129))))) +
				   (-1 * x120 * x123))) +
				 (x76 * x101) + (x120 * x133) + (x108 * x111)));
	const FLT x137 = cos((-1 * asin(x134)) + (*bsc0).gibpha + x78) * (*bsc0).gibmag;
	const FLT x138 = (*lh_p).Rot[1] * (*lh_p).Rot[1];
	const FLT x139 = -1 * x0 * x138;
	const FLT x140 = x65 + x139;
	const FLT x141 = x1 + (-1 * x3);
	const FLT x142 = x2 * (*lh_p).Rot[2];
	const FLT x143 = x68 * (*lh_p).Rot[1];
	const FLT x144 = x143 + x142;
	const FLT x145 = (x72 * x144) + (x67 * x141);
	const FLT x146 = (-1 * x75 * (x145 + (x61 * x140))) + (x60 * x140);
	const FLT x147 = x103 * x146;
	const FLT x148 = (x84 * ((-1 * x105 * x146) + x147)) + (x102 * x146);
	const FLT x149 = (x107 * x146) + (x84 * x148);
	const FLT x150 = (x80 * x140) + (-1 * x114 * x145);
	const FLT x151 = ((-1 * x117 * x144) + (x116 * x141)) * x118;
	const FLT x152 = x121 * (x151 + (-1 * x113 * x150));
	const FLT x153 =
		x151 +
		(-1 * x135 *
		 ((x132 * x152) + (x101 * x146) + x150 + (x111 * x149) +
		  (-1 * x131 *
		   ((-1 * x97 *
			 ((x130 * x146) + (x84 * x149) + (x124 * x146) +
			  (x84 * (x149 + (x84 * (x148 + (x84 * ((x127 * x146) + x147 + (-1 * x126 * x146))) + (x128 * x146))) +
					  (x129 * x146))))) +
			(-1 * x122 * x152)))));
	const FLT x154 = x69 + x70;
	const FLT x155 = x63 + x5 + x139;
	const FLT x156 = (x72 * x155) + (x67 * x154);
	const FLT x157 = x142 + (-1 * x143);
	const FLT x158 = (x80 * x157) + (-1 * x114 * x156);
	const FLT x159 = ((-1 * x117 * x155) + (x116 * x154)) * x118;
	const FLT x160 = x159 + (-1 * x113 * x158);
	const FLT x161 = (-1 * x75 * (x156 + (x61 * x157))) + (x60 * x157);
	const FLT x162 = x103 * x161;
	const FLT x163 = (x84 * ((-1 * x105 * x161) + x162)) + (x102 * x161);
	const FLT x164 = (x107 * x161) + (x84 * x163);
	const FLT x165 =
		x159 +
		(-1 * x135 *
		 ((x160 * x133) + (x101 * x161) +
		  (-1 * x131 *
		   ((-1 * x97 *
			 ((x161 * x130) + (x124 * x161) +
			  (x84 * (x164 + (x84 * (x163 + (x84 * ((x127 * x161) + x162 + (-1 * x126 * x161))) + (x128 * x161))) +
					  (x129 * x161))) +
			  (x84 * x164))) +
			(-1 * x123 * x160))) +
		  x158 + (x111 * x164)));
	const FLT x166 = 2 * (*lh_p).Rot[1];
	const FLT x167 = x166 * (*lh_p).Rot[2];
	const FLT x168 = 2 * (*lh_p).Rot[3];
	const FLT x169 = x168 * (*lh_p).Rot[0];
	const FLT x170 = x169 + x167;
	const FLT x171 = 2 * x62;
	const FLT x172 = -1 * x171;
	const FLT x173 = 2 * x64;
	const FLT x174 = 1 + (-1 * x173);
	const FLT x175 = x174 + x172;
	const FLT x176 = 2 * (*lh_p).Rot[2];
	const FLT x177 = x176 * (*lh_p).Rot[0];
	const FLT x178 = x166 * (*lh_p).Rot[3];
	const FLT x179 = x178 + (-1 * x177);
	const FLT x180 = (x72 * x179) + (x67 * x175);
	const FLT x181 = (-1 * x75 * (x180 + (x61 * x170))) + (x60 * x170);
	const FLT x182 = x103 * x181;
	const FLT x183 = (x84 * ((-1 * x105 * x181) + x182)) + (x102 * x181);
	const FLT x184 = (x107 * x181) + (x84 * x183);
	const FLT x185 = (x80 * x170) + (-1 * x114 * x180);
	const FLT x186 = ((-1 * x117 * x179) + (x116 * x175)) * x118;
	const FLT x187 = x186 + (-1 * x113 * x185);
	const FLT x188 =
		x186 + (-1 * x135 *
				(x185 + (x187 * x133) +
				 (-1 * x131 *
				  ((-1 * x97 *
					((x181 * x130) + (x84 * x184) + (x124 * x181) +
					 (x84 * ((x84 * (x183 + (x84 * ((x127 * x181) + (-1 * x126 * x181) + x182)) + (x128 * x181))) +
							 x184 + (x129 * x181))))) +
				   (-1 * x123 * x187))) +
				 (x101 * x181) + (x111 * x184)));
	const FLT x189 = 2 * x138;
	const FLT x190 = -1 * x189;
	const FLT x191 = x174 + x190;
	const FLT x192 = x167 + (-1 * x169);
	const FLT x193 = x176 * (*lh_p).Rot[3];
	const FLT x194 = x166 * (*lh_p).Rot[0];
	const FLT x195 = x194 + x193;
	const FLT x196 = (x72 * x195) + (x67 * x192);
	const FLT x197 = (-1 * x75 * (x196 + (x61 * x191))) + (x60 * x191);
	const FLT x198 = (x80 * x191) + (-1 * x114 * x196);
	const FLT x199 = ((-1 * x117 * x195) + (x116 * x192)) * x118;
	const FLT x200 = x199 + (-1 * x113 * x198);
	const FLT x201 = x103 * x197;
	const FLT x202 = (x84 * ((-1 * x105 * x197) + x201)) + (x102 * x197);
	const FLT x203 = (x107 * x197) + (x84 * x202);
	const FLT x204 =
		x199 +
		(-1 * x135 *
		 (x198 + (x203 * x111) + (x200 * x133) + (x101 * x197) +
		  (-1 * x131 *
		   ((-1 * x97 *
			 ((x197 * x130) + (x124 * x197) +
			  (x84 * (x203 + (x84 * (x202 + (x84 * ((x127 * x197) + x201 + (-1 * x126 * x197))) + (x128 * x197))) +
					  (x129 * x197))) +
			  (x84 * x203))) +
			(-1 * x200 * x123)))));
	const FLT x205 = x193 + (-1 * x194);
	const FLT x206 = x177 + x178;
	const FLT x207 = 1 + x190 + x172;
	const FLT x208 = (x72 * x207) + (x67 * x206);
	const FLT x209 = (-1 * x75 * (x208 + (x61 * x205))) + (x60 * x205);
	const FLT x210 = x209 * x103;
	const FLT x211 = (x84 * ((-1 * x209 * x105) + x210)) + (x209 * x102);
	const FLT x212 = (x209 * x107) + (x84 * x211);
	const FLT x213 = (x80 * x205) + (-1 * x208 * x114);
	const FLT x214 = ((-1 * x207 * x117) + (x206 * x116)) * x118;
	const FLT x215 = x214 + (-1 * x213 * x113);
	const FLT x216 =
		x214 +
		(-1 * x135 *
		 (x213 + (x215 * x133) + (x209 * x101) + (x212 * x111) +
		  (-1 * x131 *
		   ((-1 * x97 *
			 ((x209 * x130) + (x209 * x124) +
			  (x84 * (x212 + (x84 * (x211 + (x84 * ((x209 * x127) + x210 + (-1 * x209 * x126))) + (x209 * x128))) +
					  (x209 * x129))) +
			  (x84 * x212))) +
			(-1 * x215 * x123)))));
	const FLT x217 = x26 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x218 = x217 * sensor_pt[2];
	const FLT x219 = x28 * sensor_pt[0];
	const FLT x220 = x26 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x221 = -1 * x220 * sensor_pt[1];
	const FLT x222 = x221 + x219;
	const FLT x223 = x222 + x218;
	const FLT x224 = 2 * x33;
	const FLT x225 = 2 * x42;
	const FLT x226 = x26 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x227 = x225 * x226;
	const FLT x228 = 2 * x39;
	const FLT x229 = -1 * x217 * x228;
	const FLT x230 = -1 * x226 * sensor_pt[2];
	const FLT x231 = x220 * sensor_pt[0];
	const FLT x232 = x28 * sensor_pt[1];
	const FLT x233 = x232 + x230 + x231;
	const FLT x234 = 2 * x41;
	const FLT x235 = x229 + (x234 * x233) + (-1 * x223 * x224) + x227;
	const FLT x236 = x226 * sensor_pt[1];
	const FLT x237 = -1 * x217 * sensor_pt[0];
	const FLT x238 = x28 * sensor_pt[2];
	const FLT x239 = x238 + x237;
	const FLT x240 = x239 + x236;
	const FLT x241 = 2 * x38;
	const FLT x242 = x220 * x228;
	const FLT x243 = 2 * x44;
	const FLT x244 = -1 * x226 * x243;
	const FLT x245 = x244 + x242 + (-1 * x234 * x240) + (x223 * x241);
	const FLT x246 = -1 * x220 * x225;
	const FLT x247 = x217 * x243;
	const FLT x248 = x247 + (x224 * x240) + (-1 * x233 * x241) + x246;
	const FLT x249 = (x248 * (*lh_p).Rot[3]) + (-1 * x235 * (*lh_p).Rot[1]) + (x245 * (*lh_p).Rot[0]);
	const FLT x250 = (x245 * (*lh_p).Rot[1]) + (-1 * x248 * (*lh_p).Rot[2]) + (x235 * (*lh_p).Rot[0]);
	const FLT x251 = x248 + (-1 * x249 * x168) + (x250 * x176);
	const FLT x252 = (x248 * (*lh_p).Rot[0]) + (-1 * x245 * (*lh_p).Rot[3]) + (x235 * (*lh_p).Rot[2]);
	const FLT x253 = (-1 * x252 * x176) + x235 + (x249 * x166);
	const FLT x254 = (x72 * x253) + (x67 * x251);
	const FLT x255 = x245 + (-1 * x250 * x166) + (x252 * x168);
	const FLT x256 = (x80 * x255) + (-1 * x254 * x114);
	const FLT x257 = ((-1 * x253 * x117) + (x251 * x116)) * x118;
	const FLT x258 = x257 + (-1 * x256 * x113);
	const FLT x259 = (-1 * x75 * (x254 + (x61 * x255))) + (x60 * x255);
	const FLT x260 = x259 * x103;
	const FLT x261 = (x84 * ((-1 * x259 * x105) + x260)) + (x259 * x102);
	const FLT x262 = (x259 * x107) + (x84 * x261);
	const FLT x263 =
		x257 +
		(-1 * x135 *
		 ((x258 * x133) + x256 + (x262 * x111) +
		  (-1 * x131 *
		   ((-1 * x97 *
			 ((x259 * x130) + (x84 * x262) + (x259 * x124) +
			  (x84 * (x262 + (x84 * ((x84 * ((x259 * x127) + (-1 * x259 * x126) + x260)) + x261 + (x259 * x128))) +
					  (x259 * x129))))) +
			(-1 * x258 * x123))) +
		  (x259 * x101)));
	const FLT x264 = x217 * sensor_pt[1];
	const FLT x265 = x220 * sensor_pt[2];
	const FLT x266 = x226 * sensor_pt[0];
	const FLT x267 = (-1 * x266) + x264 + x265;
	const FLT x268 = -1 * x236;
	const FLT x269 = x268 + (-1 * x238) + x237;
	const FLT x270 = x28 * x225;
	const FLT x271 = (-1 * x242) + x270 + (-1 * x267 * x224) + (x234 * x269);
	const FLT x272 = (-1 * x231) + x230;
	const FLT x273 = x272 + x232;
	const FLT x274 = x28 * x243;
	const FLT x275 = x229 + (-1 * x274) + (-1 * x234 * x273) + (x267 * x241);
	const FLT x276 = x220 * x243;
	const FLT x277 = x217 * x225;
	const FLT x278 = (x273 * x224) + x277 + x276 + (-1 * x269 * x241);
	const FLT x279 = (x278 * (*lh_p).Rot[3]) + (-1 * x271 * (*lh_p).Rot[1]) + (x275 * (*lh_p).Rot[0]);
	const FLT x280 = (-1 * x278 * (*lh_p).Rot[2]) + (x275 * (*lh_p).Rot[1]) + (x271 * (*lh_p).Rot[0]);
	const FLT x281 = x278 + (-1 * x279 * x168) + (x280 * x176);
	const FLT x282 = (-1 * x275 * (*lh_p).Rot[3]) + (x278 * (*lh_p).Rot[0]) + (x271 * (*lh_p).Rot[2]);
	const FLT x283 = x271 + (-1 * x282 * x176) + (x279 * x166);
	const FLT x284 = (x72 * x283) + (x67 * x281);
	const FLT x285 = x275 + (-1 * x280 * x166) + (x282 * x168);
	const FLT x286 = (x80 * x285) + (-1 * x284 * x114);
	const FLT x287 = ((-1 * x283 * x117) + (x281 * x116)) * x118;
	const FLT x288 = x287 + (-1 * x286 * x113);
	const FLT x289 = (-1 * x75 * (x284 + (x61 * x285))) + (x60 * x285);
	const FLT x290 = x289 * x103;
	const FLT x291 = (x84 * ((-1 * x289 * x105) + x290)) + (x289 * x102);
	const FLT x292 = (x289 * x107) + (x84 * x291);
	const FLT x293 = x77 * x289;
	const FLT x294 =
		x287 +
		(-1 * x135 *
		 (x286 + (x288 * x133) + (x293 * x100) +
		  (-1 * x131 *
		   ((-1 * x97 *
			 ((x289 * x130) +
			  (x84 * (x292 + (x84 * (x291 + (x84 * (x290 + (x289 * x127) + (-1 * x293 * x125))) + (x289 * x128))) +
					  (x289 * x129))) +
			  (x289 * x124) + (x84 * x292))) +
			(-1 * x288 * x123))) +
		  (x292 * x111)));
	const FLT x295 = x239 + x268;
	const FLT x296 = x28 * x228;
	const FLT x297 = (-1 * x264) + x265 + x266;
	const FLT x298 = x246 + (x234 * x297) + (-1 * x295 * x224) + (-1 * x296);
	const FLT x299 = -1 * x218;
	const FLT x300 = x221 + (-1 * x219) + x299;
	const FLT x301 = (x224 * x300) + x274 + (-1 * x227) + (-1 * x297 * x241);
	const FLT x302 = x228 * x226;
	const FLT x303 = x302 + (x295 * x241) + (-1 * x234 * x300) + x276;
	const FLT x304 = (x303 * (*lh_p).Rot[1]) + (x298 * (*lh_p).Rot[0]) + (-1 * x301 * (*lh_p).Rot[2]);
	const FLT x305 = (x301 * (*lh_p).Rot[0]) + (-1 * x303 * (*lh_p).Rot[3]) + (x298 * (*lh_p).Rot[2]);
	const FLT x306 = x303 + (-1 * x304 * x166) + (x305 * x168);
	const FLT x307 = (x301 * (*lh_p).Rot[3]) + (x303 * (*lh_p).Rot[0]) + (-1 * x298 * (*lh_p).Rot[1]);
	const FLT x308 = (-1 * x307 * x168) + x301 + (x304 * x176);
	const FLT x309 = x298 + (-1 * x305 * x176) + (x307 * x166);
	const FLT x310 = (x72 * x309) + (x67 * x308);
	const FLT x311 = x77 * ((-1 * x75 * (x310 + (x61 * x306))) + (x60 * x306));
	const FLT x312 = x86 * x311;
	const FLT x313 = (x84 * ((-1 * x85 * x311) + x312)) + (x87 * x311);
	const FLT x314 = (x88 * x311) + (x84 * x313);
	const FLT x315 = (x80 * x306) + (-1 * x310 * x114);
	const FLT x316 = ((-1 * x309 * x117) + (x308 * x116)) * x118;
	const FLT x317 = x316 + (-1 * x315 * x113);
	const FLT x318 =
		x316 + (-1 * x135 *
				(x315 +
				 (-1 * x131 *
				  ((-1 * x97 *
					((x89 * x311) + (x84 * x314) + (x94 * x311) +
					 (x84 * ((x84 * (x313 + (x84 * ((x91 * x311) + x312 + (-1 * x311 * x125))) + (x92 * x311))) + x314 +
							 (x93 * x311))))) +
				   (-1 * x317 * x123))) +
				 (x314 * x111) + (x317 * x133) + (x311 * x100)));
	const FLT x319 = x272 + (-1 * x232);
	const FLT x320 = x222 + x299;
	const FLT x321 = (x234 * x320) + x277 + (-1 * x224 * x319) + x302;
	const FLT x322 = x264 + x266 + (-1 * x265);
	const FLT x323 = (-1 * x247) + x296 + (-1 * x234 * x322) + (x241 * x319);
	const FLT x324 = x244 + (-1 * x270) + (x224 * x322) + (-1 * x241 * x320);
	const FLT x325 = (x324 * (*lh_p).Rot[3]) + (-1 * x321 * (*lh_p).Rot[1]) + (x323 * (*lh_p).Rot[0]);
	const FLT x326 = (x323 * (*lh_p).Rot[1]) + (-1 * x324 * (*lh_p).Rot[2]) + (x321 * (*lh_p).Rot[0]);
	const FLT x327 = x324 + (-1 * x325 * x168) + (x326 * x176);
	const FLT x328 = (x324 * (*lh_p).Rot[0]) + (-1 * x323 * (*lh_p).Rot[3]) + (x321 * (*lh_p).Rot[2]);
	const FLT x329 = x321 + (-1 * x328 * x176) + (x325 * x166);
	const FLT x330 = (x72 * x329) + (x67 * x327);
	const FLT x331 = (x328 * x168) + x323 + (-1 * x326 * x166);
	const FLT x332 = (x80 * x331) + (-1 * x330 * x114);
	const FLT x333 = ((-1 * x329 * x117) + (x327 * x116)) * x118;
	const FLT x334 = x333 + (-1 * x332 * x113);
	const FLT x335 = (-1 * x75 * (x330 + (x61 * x331))) + (x60 * x331);
	const FLT x336 = x335 * x103;
	const FLT x337 = (x84 * ((-1 * x335 * x105) + x336)) + (x335 * x102);
	const FLT x338 = (x335 * x107) + (x84 * x337);
	const FLT x339 =
		x333 +
		(-1 * x135 *
		 ((x334 * x133) + x332 + (x338 * x111) +
		  (-1 * x131 *
		   ((-1 * x97 *
			 ((x84 * (x338 + (x84 * (x337 + (x84 * ((x335 * x127) + x336 + (-1 * x335 * x126))) + (x335 * x128))) +
					  (x335 * x129))) +
			  (x335 * x130) + (x84 * x338) + (x335 * x124))) +
			(-1 * x334 * x123))) +
		  (x335 * x101)));
	const FLT x340 = -1 * x32;
	const FLT x341 = x23 * (*_x0).Pose.Rot[2];
	const FLT x342 = x6 * x24;
	const FLT x343 = x342 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x344 = 0.5 * x343;
	const FLT x345 = dt * dt * dt;
	const FLT x346 = 0.5 * x18 * x345;
	const FLT x347 = x11 * x346;
	const FLT x348 =
		(*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x349 = dt * dt * dt * dt;
	const FLT x350 = 2 * (1. / (x13 * x13)) * x17;
	const FLT x351 = x350 * x349;
	const FLT x352 = x7 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x353 = 1. / (x13 * sqrt(x13));
	const FLT x354 = 1.0 * x20 * x16;
	const FLT x355 = x354 * x353;
	const FLT x356 = x355 * x349;
	const FLT x357 = 2 * x19;
	const FLT x358 = x6 * x357;
	const FLT x359 = x9 * x351;
	const FLT x360 = (-1 * x359 * (*_x0).Velocity.AxisAngleRot[0]) + (x9 * x356 * (*_x0).Velocity.AxisAngleRot[0]) +
					 (x358 * (*_x0).Velocity.AxisAngleRot[0]) + (x352 * x356) + (-1 * x351 * x348) +
					 (-1 * x354 * x343) + (-1 * x352 * x351) + (x356 * x348);
	const FLT x361 = 1.0 / 2.0 * (1. / (x21 * sqrt(x21)));
	const FLT x362 = x25 * x16 * x361;
	const FLT x363 = x362 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x364 = x363 * x360;
	const FLT x365 = x20 * x361;
	const FLT x366 = x365 * x360;
	const FLT x367 = x362 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x368 = x367 * x360;
	const FLT x369 = x23 * (*_x0).Pose.Rot[1];
	const FLT x370 = x353 * x345;
	const FLT x371 = x370 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x372 = x371 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x373 = x372 * x369;
	const FLT x374 = x346 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x375 = x374 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x376 = x40 * x375;
	const FLT x377 = x362 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x378 = x377 * x360;
	const FLT x379 = x11 * x370;
	const FLT x380 = x374 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x381 = x23 * (*_x0).Pose.Rot[0];
	const FLT x382 = x381 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x383 = x370 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x384 = (-1 * x383 * x382) + (x35 * x380);
	const FLT x385 = x384 + (x31 * x379) + x376 + (-1 * x373) + (-1 * x344 * x341) + (-1 * x37 * x347) +
					 (-1 * x378 * (*_x0).Pose.Rot[1]) + (-1 * x368 * (*_x0).Pose.Rot[0]) + (x364 * (*_x0).Pose.Rot[3]) +
					 x340 + (-1 * x366 * (*_x0).Pose.Rot[2]);
	const FLT x386 = x37 * x380;
	const FLT x387 = x31 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x388 = x383 * x387;
	const FLT x389 = x29 * x375;
	const FLT x390 = x372 * x341;
	const FLT x391 = x377 * (*_x0).Pose.Rot[2];
	const FLT x392 = (-1 * x364 * (*_x0).Pose.Rot[0]) + (-1 * x366 * (*_x0).Pose.Rot[1]) + (-1 * x379 * x381) + x27 +
					 x386 + (x391 * x360) + (-1 * x389) + (-1 * x388) + x390 + (-1 * x369 * x344) +
					 (-1 * x368 * (*_x0).Pose.Rot[3]) + (x35 * x347);
	const FLT x393 = x363 * (*_x0).Pose.Rot[2];
	const FLT x394 = x369 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x395 = (-1 * x40 * x380) + (x394 * x383);
	const FLT x396 = (-1 * x372 * x381) + (x35 * x375);
	const FLT x397 = x396 + (-1 * x393 * x360) + x395 + (-1 * x31 * x344) + x34 + (-1 * x378 * (*_x0).Pose.Rot[0]) +
					 (-1 * x379 * x341) + (x29 * x347) + (x368 * (*_x0).Pose.Rot[1]) + (-1 * x366 * (*_x0).Pose.Rot[3]);
	const FLT x398 = x29 * x380;
	const FLT x399 = x341 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x400 = x399 * x383;
	const FLT x401 = -1 * x30;
	const FLT x402 = (-1 * x37 * x375) + (x31 * x372);
	const FLT x403 = (x379 * x369) + x401 + (-1 * x40 * x347) + (-1 * x366 * (*_x0).Pose.Rot[0]) + (-1 * x381 * x344) +
					 (x368 * (*_x0).Pose.Rot[2]) + x402 + (x378 * (*_x0).Pose.Rot[3]) + (-1 * x398) +
					 (x364 * (*_x0).Pose.Rot[1]) + x400;
	const FLT x404 = (-1 * x392 * sensor_pt[2]) + (x403 * sensor_pt[1]) + (x397 * sensor_pt[0]);
	const FLT x405 = (x392 * sensor_pt[1]) + (-1 * x385 * sensor_pt[0]) + (x403 * sensor_pt[2]);
	const FLT x406 = (x405 * x224) + (-1 * x225 * x397) + (x243 * x385) + (-1 * x404 * x241);
	const FLT x407 = 2 * ((x403 * sensor_pt[0]) + (-1 * x397 * sensor_pt[1]) + (x385 * sensor_pt[2]));
	const FLT x408 = (x404 * x234) + (-1 * x33 * x407) + (x225 * x392) + (-1 * x228 * x385);
	const FLT x409 = (x38 * x407) + (-1 * x243 * x392) + (-1 * x405 * x234) + (x228 * x397);
	const FLT x410 = (x409 * (*lh_p).Rot[1]) + (-1 * x406 * (*lh_p).Rot[2]) + (x408 * (*lh_p).Rot[0]);
	const FLT x411 = (x406 * (*lh_p).Rot[0]) + (x408 * (*lh_p).Rot[2]) + (-1 * x409 * (*lh_p).Rot[3]);
	const FLT x412 = x409 + (-1 * x410 * x166) + (x411 * x168);
	const FLT x413 = (x406 * (*lh_p).Rot[3]) + (-1 * x408 * (*lh_p).Rot[1]) + (x409 * (*lh_p).Rot[0]);
	const FLT x414 = x406 + (-1 * x413 * x168) + (x410 * x176);
	const FLT x415 = x408 + (-1 * x411 * x176) + (x413 * x166);
	const FLT x416 = (x72 * x415) + (x67 * x414);
	const FLT x417 = (-1 * x75 * (x416 + (x61 * x412))) + (x60 * x412);
	const FLT x418 = (x80 * x412) + (-1 * x416 * x114);
	const FLT x419 = ((-1 * x415 * x117) + (x414 * x116)) * x118;
	const FLT x420 = x419 + (-1 * x418 * x113);
	const FLT x421 = x417 * x103;
	const FLT x422 = (x84 * ((-1 * x417 * x105) + x421)) + (x417 * x102);
	const FLT x423 = (x417 * x107) + (x84 * x422);
	const FLT x424 =
		x419 +
		(-1 * x135 *
		 (x418 + (x420 * x133) + (x423 * x111) + (x417 * x101) +
		  (-1 * x131 *
		   ((-1 * x97 *
			 ((x84 * x423) + (x417 * x130) + (x417 * x124) +
			  (x84 * (x423 + (x84 * (x422 + (x84 * ((x417 * x127) + x421 + (-1 * x417 * x126))) + (x417 * x128))) +
					  (x417 * x129))))) +
			(-1 * x420 * x123)))));
	const FLT x425 = x11 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x426 = x354 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x427 =
		(*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x428 = (x427 * x356) + (-1 * x359 * (*_x0).Velocity.AxisAngleRot[1]) + (-1 * x426 * x342) +
					 (x9 * x426 * x353 * x349) + (-1 * x425 * x351) + (-1 * x427 * x351) + (x425 * x356) +
					 (x358 * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x429 = x363 * (*_x0).Pose.Rot[0];
	const FLT x430 = x7 * x370;
	const FLT x431 = x428 * (*_x0).Pose.Rot[3];
	const FLT x432 = x7 * x346;
	const FLT x433 = x428 * x365;
	const FLT x434 = 0.5 * x342;
	const FLT x435 = x346 * (*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x436 = (-1 * x29 * x435) + (x371 * x399);
	const FLT x437 = x384 + x436 + (-1 * x433 * (*_x0).Pose.Rot[1]) + (-1 * x434 * x394) + x32 + (-1 * x429 * x428) +
					 (x37 * x432) + (-1 * x31 * x430) + (-1 * x431 * x367) + (x428 * x391);
	const FLT x438 = x428 * x367;
	const FLT x439 = x23 * x430;
	const FLT x440 = x428 * x377;
	const FLT x441 = (x35 * x435) + (-1 * x371 * x382);
	const FLT x442 = (-1 * x434 * x387) + x441 + x398 + (-1 * x400) + (-1 * x440 * (*_x0).Pose.Rot[0]) +
					 (-1 * x40 * x432) + x401 + (-1 * x428 * x393) + (x439 * (*_x0).Pose.Rot[1]) +
					 (x438 * (*_x0).Pose.Rot[1]) + (-1 * x433 * (*_x0).Pose.Rot[3]);
	const FLT x443 = x37 * x435;
	const FLT x444 = x363 * (*_x0).Pose.Rot[1];
	const FLT x445 = x371 * x387;
	const FLT x446 = -1 * x34;
	const FLT x447 = x395 + (x439 * (*_x0).Pose.Rot[2]) + (-1 * x434 * x382) + (-1 * x433 * (*_x0).Pose.Rot[0]) +
					 (-1 * x443) + (-1 * x29 * x432) + x445 + (x438 * (*_x0).Pose.Rot[2]) + x446 + (x428 * x444) +
					 (x431 * x377);
	const FLT x448 = (x447 * sensor_pt[1]) + (-1 * x437 * sensor_pt[2]) + (x442 * sensor_pt[0]);
	const FLT x449 = x371 * x394;
	const FLT x450 = x40 * x435;
	const FLT x451 = x27 + (-1 * x438 * (*_x0).Pose.Rot[0]) + (x35 * x432) + (-1 * x430 * x381) + (x431 * x363) +
					 (-1 * x440 * (*_x0).Pose.Rot[1]) + x388 + (-1 * x434 * x399) + (-1 * x386) +
					 (-1 * x433 * (*_x0).Pose.Rot[2]) + (-1 * x449) + x450;
	const FLT x452 = (x437 * sensor_pt[1]) + (-1 * x451 * sensor_pt[0]) + (x447 * sensor_pt[2]);
	const FLT x453 = (-1 * x442 * x225) + (x451 * x243) + (-1 * x448 * x241) + (x452 * x224);
	const FLT x454 = 2 * ((-1 * x442 * sensor_pt[1]) + (x447 * sensor_pt[0]) + (x451 * sensor_pt[2]));
	const FLT x455 = (x448 * x234) + (-1 * x451 * x228) + (-1 * x33 * x454) + (x437 * x225);
	const FLT x456 = (x38 * x454) + (-1 * x452 * x234) + (x442 * x228) + (-1 * x437 * x243);
	const FLT x457 = (x456 * (*lh_p).Rot[1]) + (-1 * x453 * (*lh_p).Rot[2]) + (x455 * (*lh_p).Rot[0]);
	const FLT x458 = (x453 * (*lh_p).Rot[0]) + (-1 * x456 * (*lh_p).Rot[3]) + (x455 * (*lh_p).Rot[2]);
	const FLT x459 = x456 + (-1 * x457 * x166) + (x458 * x168);
	const FLT x460 = (x453 * (*lh_p).Rot[3]) + (-1 * x455 * (*lh_p).Rot[1]) + (x456 * (*lh_p).Rot[0]);
	const FLT x461 = x453 + (-1 * x460 * x168) + (x457 * x176);
	const FLT x462 = x455 + (-1 * x458 * x176) + (x460 * x166);
	const FLT x463 = (x72 * x462) + (x67 * x461);
	const FLT x464 = (-1 * x75 * (x463 + (x61 * x459))) + (x60 * x459);
	const FLT x465 = x464 * x103;
	const FLT x466 = (x84 * ((-1 * x464 * x105) + x465)) + (x464 * x102);
	const FLT x467 = (x464 * x107) + (x84 * x466);
	const FLT x468 = (x80 * x459) + (-1 * x463 * x114);
	const FLT x469 = ((-1 * x462 * x117) + (x461 * x116)) * x118;
	const FLT x470 = x469 + (-1 * x468 * x113);
	const FLT x471 =
		x469 +
		(-1 * x135 *
		 (x468 + (x464 * x101) + (x467 * x111) + (x470 * x133) +
		  (-1 * x131 *
		   ((-1 * x97 *
			 ((x84 * x467) + (x464 * x124) + (x464 * x130) +
			  (x84 * (x467 + (x84 * (x466 + (x84 * ((x464 * x127) + x465 + (-1 * x464 * x126))) + (x464 * x128))) +
					  (x464 * x129))))) +
			(-1 * x470 * x123)))));
	const FLT x472 = x351 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x473 = x356 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x474 =
		x349 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x475 = x6 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x476 = x24 * x475;
	const FLT x477 = (-1 * x7 * x472) + (x475 * x357) + (x7 * x473) + (-1 * x11 * x472) + (x11 * x473) + (x474 * x355) +
					 (-1 * x474 * x350) + (-1 * x476 * x354);
	const FLT x478 = x9 * x370;
	const FLT x479 = x477 * (*_x0).Pose.Rot[3];
	const FLT x480 = x477 * x365;
	const FLT x481 = 0.5 * x476;
	const FLT x482 = x9 * x346;
	const FLT x483 = x396 + (-1 * x29 * x482) + (-1 * x477 * x429) + (-1 * x479 * x367) + x446 + (x477 * x391) +
					 (-1 * x481 * x369) + x443 + (-1 * x480 * (*_x0).Pose.Rot[1]) + (-1 * x445) + (x478 * x341);
	const FLT x484 = x477 * x367;
	const FLT x485 = x477 * x377;
	const FLT x486 = (-1 * x390) + (-1 * x477 * x393) + (x35 * x482) + x449 + (x484 * (*_x0).Pose.Rot[1]) + x27 +
					 (-1 * x480 * (*_x0).Pose.Rot[3]) + x389 + (-1 * x478 * x381) + (-1 * x450) +
					 (-1 * x485 * (*_x0).Pose.Rot[0]) + (-1 * x31 * x481);
	const FLT x487 = x436 + (-1 * x480 * (*_x0).Pose.Rot[0]) + x340 + (-1 * x376) + (x485 * (*_x0).Pose.Rot[3]) +
					 (-1 * x481 * x381) + (-1 * x37 * x482) + (x31 * x478) + x373 + (x477 * x444) +
					 (x484 * (*_x0).Pose.Rot[2]);
	const FLT x488 = (x487 * sensor_pt[1]) + (-1 * x483 * sensor_pt[2]) + (x486 * sensor_pt[0]);
	const FLT x489 = x441 + x402 + (-1 * x485 * (*_x0).Pose.Rot[1]) + (-1 * x481 * x341) + (x479 * x363) +
					 (-1 * x484 * (*_x0).Pose.Rot[0]) + (-1 * x480 * (*_x0).Pose.Rot[2]) + x30 + (-1 * x478 * x369) +
					 (x40 * x482);
	const FLT x490 = (x483 * sensor_pt[1]) + (-1 * x489 * sensor_pt[0]) + (x487 * sensor_pt[2]);
	const FLT x491 = (-1 * x486 * x225) + (-1 * x488 * x241) + (x490 * x224) + (x489 * x243);
	const FLT x492 = (-1 * x486 * sensor_pt[1]) + (x487 * sensor_pt[0]) + (x489 * sensor_pt[2]);
	const FLT x493 = (x483 * x225) + (-1 * x489 * x228) + (-1 * x492 * x224) + (x488 * x234);
	const FLT x494 = (x492 * x241) + (-1 * x483 * x243) + (-1 * x490 * x234) + (x486 * x228);
	const FLT x495 = (x494 * (*lh_p).Rot[1]) + (-1 * x491 * (*lh_p).Rot[2]) + (x493 * (*lh_p).Rot[0]);
	const FLT x496 = (-1 * x494 * (*lh_p).Rot[3]) + (x491 * (*lh_p).Rot[0]) + (x493 * (*lh_p).Rot[2]);
	const FLT x497 = x494 + (-1 * x495 * x166) + (x496 * x168);
	const FLT x498 = (-1 * x493 * (*lh_p).Rot[1]) + (x491 * (*lh_p).Rot[3]) + (x494 * (*lh_p).Rot[0]);
	const FLT x499 = x491 + (-1 * x498 * x168) + (x495 * x176);
	const FLT x500 = (x498 * x166) + x493 + (-1 * x496 * x176);
	const FLT x501 = (x72 * x500) + (x67 * x499);
	const FLT x502 = (-1 * x75 * (x501 + (x61 * x497))) + (x60 * x497);
	const FLT x503 = x502 * x103;
	const FLT x504 = (x84 * ((-1 * x502 * x105) + x503)) + (x502 * x102);
	const FLT x505 = (x502 * x107) + (x84 * x504);
	const FLT x506 = (x80 * x497) + (-1 * x501 * x114);
	const FLT x507 = ((-1 * x500 * x117) + (x499 * x116)) * x118;
	const FLT x508 = x507 + (-1 * x506 * x113);
	const FLT x509 =
		x507 +
		(-1 * x135 *
		 (x506 +
		  (-1 * x131 *
		   ((-1 * x97 *
			 ((x502 * x130) + (x84 * x505) + (x502 * x124) +
			  (x84 * (x505 + (x84 * ((x84 * ((x502 * x127) + x503 + (-1 * x502 * x126))) + x504 + (x502 * x128))) +
					  (x502 * x129))))) +
			(-1 * x508 * x123))) +
		  (x508 * x133) + (x502 * x101) + (x505 * x111)));
	const FLT x510 = dt * x167;
	const FLT x511 = dt * x169;
	const FLT x512 = x511 + x510;
	const FLT x513 = -1 * dt * x171;
	const FLT x514 = -1 * dt * x173;
	const FLT x515 = x514 + dt + x513;
	const FLT x516 = dt * x177;
	const FLT x517 = dt * x178;
	const FLT x518 = x517 + (-1 * x516);
	const FLT x519 = (x72 * x518) + (x67 * x515);
	const FLT x520 = x77 * ((-1 * x75 * (x519 + (x61 * x512))) + (x60 * x512));
	const FLT x521 = x86 * x520;
	const FLT x522 = (x84 * ((-1 * x85 * x520) + x521)) + (x87 * x520);
	const FLT x523 = (x88 * x520) + (x84 * x522);
	const FLT x524 = (x80 * x512) + (-1 * x519 * x114);
	const FLT x525 = ((-1 * x518 * x117) + (x515 * x116)) * x118;
	const FLT x526 = x525 + (-1 * x524 * x113);
	const FLT x527 =
		x525 + (-1 * x135 *
				(x524 +
				 (-1 * x131 *
				  ((-1 * x97 *
					((x84 * (x523 + (x84 * (x522 + (x84 * ((x91 * x520) + x521 + (-1 * x520 * x125))) + (x92 * x520))) +
							 (x93 * x520))) +
					 (x89 * x520) + (x94 * x520) + (x84 * x523))) +
				   (-1 * x526 * x123))) +
				 (x526 * x133) + (x520 * x100) + (x523 * x111)));
	const FLT x528 = (-1 * dt * x189) + dt;
	const FLT x529 = x528 + x514;
	const FLT x530 = x510 + (-1 * x511);
	const FLT x531 = dt * x193;
	const FLT x532 = dt * x194;
	const FLT x533 = x532 + x531;
	const FLT x534 = (x72 * x533) + (x67 * x530);
	const FLT x535 = (-1 * x75 * (x534 + (x61 * x529))) + (x60 * x529);
	const FLT x536 = x77 * x535;
	const FLT x537 = x86 * x536;
	const FLT x538 = (x84 * ((-1 * x85 * x536) + x537)) + (x87 * x536);
	const FLT x539 = (x88 * x536) + (x84 * x538);
	const FLT x540 = (x80 * x529) + (-1 * x534 * x114);
	const FLT x541 = ((-1 * x533 * x117) + (x530 * x116)) * x118;
	const FLT x542 = x541 + (-1 * x540 * x113);
	const FLT x543 =
		x541 +
		(-1 * x135 *
		 ((x542 * x133) + (x539 * x111) + x540 + (x536 * x100) +
		  (-1 * x131 *
		   ((-1 * x97 *
			 ((x94 * x536) + (x535 * x130) + (x84 * x539) +
			  (x84 * (x539 + (x84 * (x538 + (x84 * ((x535 * x127) + x537 + (-1 * x536 * x125))) + (x92 * x536))) +
					  (x535 * x129))))) +
			(-1 * x542 * x123)))));
	const FLT x544 = x531 + (-1 * x532);
	const FLT x545 = x516 + x517;
	const FLT x546 = x528 + x513;
	const FLT x547 = (x72 * x546) + (x67 * x545);
	const FLT x548 = (-1 * x75 * (x547 + (x61 * x544))) + (x60 * x544);
	const FLT x549 = x548 * x103;
	const FLT x550 = (x84 * ((-1 * x548 * x105) + x549)) + (x548 * x102);
	const FLT x551 = (x548 * x107) + (x84 * x550);
	const FLT x552 = (x80 * x544) + (-1 * x547 * x114);
	const FLT x553 = ((-1 * x546 * x117) + (x545 * x116)) * x118;
	const FLT x554 = x553 + (-1 * x552 * x113);
	const FLT x555 =
		x553 +
		(-1 * x135 *
		 ((x554 * x133) +
		  (-1 * x131 *
		   ((-1 * x97 *
			 ((x84 * x551) +
			  (x84 * (x551 + (x84 * ((x84 * ((x548 * x127) + x549 + (-1 * x548 * x126))) + x550 + (x548 * x128))) +
					  (x548 * x129))) +
			  (x548 * x130) + (x548 * x124))) +
			(-1 * x554 * x123))) +
		  (x551 * x111) + x552 + (x548 * x101)));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT), x136 + (x137 * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT), x153 + (x137 * x153));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT), x165 + (x165 * x137));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT), x188 + (x188 * x137));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT), x204 + (x204 * x137));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT), x216 + (x216 * x137));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x263 + (x263 * x137));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x294 + (x294 * x137));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x318 + (x318 * x137));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x339 + (x339 * x137));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x424 + (x424 * x137));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x471 + (x471 * x137));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x509 + (x509 * x137));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT), x527 + (x527 * x137));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT), x543 + (x543 * x137));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT), x555 + (x555 * x137));
}

// Full version Jacobian of SurviveKalmanModel_LightMeas_x_gen2 wrt [(*_x0).AccBias[0], (*_x0).AccBias[1],
// (*_x0).AccBias[2], (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1],
// (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2],
// (*_x0).IMUCorrection[3], (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0],
// (*_x0).Pose.Rot[1], (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0],
// (*_x0).Velocity.AxisAngleRot[1], (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1],
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4edc580>]

static inline void SurviveKalmanModel_LightMeas_x_gen2_jac_x0_with_hx(CnMat *Hx, CnMat *hx, const FLT dt,
																	  const SurviveKalmanModel *_x0,
																	  const FLT *sensor_pt, const SurvivePose *lh_p,
																	  const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanModel_LightMeas_x_gen2(dt, _x0, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanModel_LightMeas_x_gen2_jac_x0(Hx, dt, _x0, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanModel_LightMeas_x_gen2 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]
static inline void SurviveKalmanModel_LightMeas_x_gen2_jac_sensor_pt(CnMat *Hx, const FLT dt,
																	 const SurviveKalmanModel *_x0,
																	 const FLT *sensor_pt, const SurvivePose *lh_p,
																	 const BaseStationCal *bsc0) {
	const FLT x0 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x1 = dt * dt;
	const FLT x2 = x1 * ((*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x3 = x1 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x4 = x1 * ((*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x5 = 1e-10 + x2 + x3 + x4;
	const FLT x6 = sqrt(x5);
	const FLT x7 = 0.5 * x6;
	const FLT x8 = sin(x7);
	const FLT x9 = (1. / x5) * (x8 * x8);
	const FLT x10 = cos(x7);
	const FLT x11 = 1. / sqrt((x4 * x9) + (x3 * x9) + (x2 * x9) + (x10 * x10));
	const FLT x12 = (1. / x6) * x8 * dt * x11;
	const FLT x13 = x12 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x14 = x13 * (*_x0).Pose.Rot[0];
	const FLT x15 = x11 * x10;
	const FLT x16 = x15 * (*_x0).Pose.Rot[2];
	const FLT x17 = x12 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x18 = x17 * (*_x0).Pose.Rot[1];
	const FLT x19 = x12 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x20 = x19 * (*_x0).Pose.Rot[3];
	const FLT x21 = x18 + x14 + (-1 * x20) + x16;
	const FLT x22 = (-1 * x19 * (*_x0).Pose.Rot[1]) + (x15 * (*_x0).Pose.Rot[0]) + (-1 * x17 * (*_x0).Pose.Rot[3]) +
					(-1 * x13 * (*_x0).Pose.Rot[2]);
	const FLT x23 = x17 * (*_x0).Pose.Rot[0];
	const FLT x24 = x13 * (*_x0).Pose.Rot[1];
	const FLT x25 = x15 * (*_x0).Pose.Rot[3];
	const FLT x26 = x19 * (*_x0).Pose.Rot[2];
	const FLT x27 = x26 + x25 + x23 + (-1 * x24);
	const FLT x28 = (x21 * sensor_pt[2]) + (-1 * x27 * sensor_pt[1]) + (x22 * sensor_pt[0]);
	const FLT x29 = x17 * (*_x0).Pose.Rot[2];
	const FLT x30 = x13 * (*_x0).Pose.Rot[3];
	const FLT x31 = x15 * (*_x0).Pose.Rot[1];
	const FLT x32 = x19 * (*_x0).Pose.Rot[0];
	const FLT x33 = x32 + x31 + (-1 * x29) + x30;
	const FLT x34 = (-1 * x33 * sensor_pt[2]) + (x27 * sensor_pt[0]) + (x22 * sensor_pt[1]);
	const FLT x35 = sensor_pt[2] + (x0 * (*_x0).Acc[2]) + (2 * ((x34 * x33) + (-1 * x21 * x28))) +
					(dt * (*_x0).Velocity.Pos[2]) + (*_x0).Pose.Pos[2];
	const FLT x36 = (-1 * x21 * sensor_pt[0]) + (x22 * sensor_pt[2]) + (x33 * sensor_pt[1]);
	const FLT x37 = (dt * (*_x0).Velocity.Pos[1]) + sensor_pt[1] + (2 * ((x28 * x27) + (-1 * x33 * x36))) +
					(*_x0).Pose.Pos[1] + (x0 * (*_x0).Acc[1]);
	const FLT x38 = (2 * ((x36 * x21) + (-1 * x34 * x27))) + (*_x0).Pose.Pos[0] + sensor_pt[0] +
					(dt * (*_x0).Velocity.Pos[0]) + (x0 * (*_x0).Acc[0]);
	const FLT x39 = (x35 * (*lh_p).Rot[0]) + (-1 * x38 * (*lh_p).Rot[2]) + (x37 * (*lh_p).Rot[1]);
	const FLT x40 = (-1 * x37 * (*lh_p).Rot[3]) + (x35 * (*lh_p).Rot[2]) + (x38 * (*lh_p).Rot[0]);
	const FLT x41 = x37 + (*lh_p).Pos[1] + (2 * ((x40 * (*lh_p).Rot[3]) + (-1 * x39 * (*lh_p).Rot[1])));
	const FLT x42 = x41 * x41;
	const FLT x43 = (-1 * x35 * (*lh_p).Rot[1]) + (x37 * (*lh_p).Rot[0]) + (x38 * (*lh_p).Rot[3]);
	const FLT x44 = x35 + (2 * ((x43 * (*lh_p).Rot[1]) + (-1 * x40 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x45 = x38 + (2 * ((x39 * (*lh_p).Rot[2]) + (-1 * x43 * (*lh_p).Rot[3]))) + (*lh_p).Pos[0];
	const FLT x46 = x45 * x45;
	const FLT x47 = x46 + (x44 * x44);
	const FLT x48 = x47 + x42;
	const FLT x49 = 0.523598775598299 + (*bsc0).tilt;
	const FLT x50 = cos(x49);
	const FLT x51 = 1. / x50;
	const FLT x52 = x51 * (1. / sqrt(x48));
	const FLT x53 = asin(x52 * x41);
	const FLT x54 = 8.0108022e-06 * x53;
	const FLT x55 = -8.0108022e-06 + (-1 * x54);
	const FLT x56 = 0.0028679863 + (x53 * x55);
	const FLT x57 = 1. / sqrt(1 + (-1 * (1. / (x50 * x50)) * x42 * (1. / x48)));
	const FLT x58 = (-1 * x18) + x20 + (-1 * x16) + (-1 * x14);
	const FLT x59 = 2 * x33;
	const FLT x60 = 2 * x22;
	const FLT x61 = x60 * x27;
	const FLT x62 = x61 + (-1 * x58 * x59);
	const FLT x63 = x60 * x21;
	const FLT x64 = (x59 * x27) + (-1 * x63);
	const FLT x65 = 2 * x21;
	const FLT x66 = 1 + (x65 * x58) + (-2 * (x27 * x27));
	const FLT x67 = (x66 * (*lh_p).Rot[0]) + (-1 * x62 * (*lh_p).Rot[3]) + (x64 * (*lh_p).Rot[2]);
	const FLT x68 = 2 * (*lh_p).Rot[3];
	const FLT x69 = (-1 * x66 * (*lh_p).Rot[2]) + (x62 * (*lh_p).Rot[1]) + (x64 * (*lh_p).Rot[0]);
	const FLT x70 = 2 * (*lh_p).Rot[1];
	const FLT x71 = x62 + (x67 * x68) + (-1 * x70 * x69);
	const FLT x72 = 2 * x41;
	const FLT x73 = 2 * (*lh_p).Rot[2];
	const FLT x74 = (x66 * (*lh_p).Rot[3]) + (-1 * x64 * (*lh_p).Rot[1]) + (x62 * (*lh_p).Rot[0]);
	const FLT x75 = x66 + (x73 * x69) + (-1 * x74 * x68);
	const FLT x76 = 2 * x45;
	const FLT x77 = x64 + (x70 * x74) + (-1 * x73 * x67);
	const FLT x78 = 2 * x44;
	const FLT x79 = (x78 * x77) + (x75 * x76);
	const FLT x80 = 1.0 / 2.0 * x41;
	const FLT x81 = x80 * x51 * (1. / (x48 * sqrt(x48)));
	const FLT x82 = x57 * ((-1 * x81 * (x79 + (x71 * x72))) + (x71 * x52));
	const FLT x83 = x82 * x55;
	const FLT x84 = (x53 * ((-1 * x82 * x54) + x83)) + (x82 * x56);
	const FLT x85 = 5.3685255e-06 + (x53 * x56);
	const FLT x86 = (x82 * x85) + (x84 * x53);
	const FLT x87 = 0.0076069798 + (x85 * x53);
	const FLT x88 = x87 * x53;
	const FLT x89 = -8.0108022e-06 + (-1.60216044e-05 * x53);
	const FLT x90 = x56 + (x89 * x53);
	const FLT x91 = x85 + (x53 * x90);
	const FLT x92 = x87 + (x53 * x91);
	const FLT x93 = (x53 * x92) + x88;
	const FLT x94 = atan2(-1 * x44, x45);
	const FLT x95 = tan(x49);
	const FLT x96 = x95 * (1. / sqrt(x47));
	const FLT x97 = x96 * x41;
	const FLT x98 = (-1 * asin(x97)) + x94 + (*bsc0).ogeephase;
	const FLT x99 = (sin(x98) * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x100 = sin(x49);
	const FLT x101 = x99 * x100;
	const FLT x102 = x50 + (-1 * x93 * x101);
	const FLT x103 = 1. / x102;
	const FLT x104 = x53 * x53;
	const FLT x105 = x99 * x104;
	const FLT x106 = x103 * x105;
	const FLT x107 = x80 * x95 * (1. / (x47 * sqrt(x47)));
	const FLT x108 = (x71 * x96) + (-1 * x79 * x107);
	const FLT x109 = 1. / x47;
	const FLT x110 = 1. / sqrt(1 + (-1 * (x95 * x95) * x42 * x109));
	const FLT x111 = x44 * (1. / x46);
	const FLT x112 = 1. / x45;
	const FLT x113 = x46 * x109;
	const FLT x114 = ((-1 * x77 * x112) + (x75 * x111)) * x113;
	const FLT x115 = cos(x98) * (*bsc0).ogeemag;
	const FLT x116 = x115 * (x114 + (-1 * x108 * x110));
	const FLT x117 = x93 * x100;
	const FLT x118 = 2.40324066e-05 * x53;
	const FLT x119 = x87 * (1. / (x102 * x102)) * x105;
	const FLT x120 = 2 * x88 * x99 * x103;
	const FLT x121 = x87 * x103 * x104;
	const FLT x122 = x97 + (x87 * x106);
	const FLT x123 = 1. / sqrt(1 + (-1 * (x122 * x122)));
	const FLT x124 =
		x114 + (-1 * x123 *
				(x108 + (x82 * x120) + (x86 * x106) + (x116 * x121) +
				 (-1 * x119 *
				  ((-1 * x101 *
					((x82 * x87) + (x82 * x92) + (x86 * x53) +
					 (x53 * (x86 + (x53 * (x84 + (x53 * ((x82 * x89) + x83 + (-1 * x82 * x118))) + (x82 * x90))) +
							 (x82 * x91))))) +
				   (-1 * x116 * x117)))));
	const FLT x125 = cos((*bsc0).gibpha + (-1 * asin(x122)) + x94) * (*bsc0).gibmag;
	const FLT x126 = 2 * ((-1 * x25) + (-1 * x26) + x24 + (-1 * x23));
	const FLT x127 = x60 * x33;
	const FLT x128 = x127 + (-1 * x21 * x126);
	const FLT x129 = 1 + (x27 * x126) + (-2 * (x33 * x33));
	const FLT x130 = (x65 * x33) + (-1 * x61);
	const FLT x131 = (x130 * (*lh_p).Rot[0]) + (x128 * (*lh_p).Rot[2]) + (-1 * x129 * (*lh_p).Rot[3]);
	const FLT x132 = (x129 * (*lh_p).Rot[1]) + (x128 * (*lh_p).Rot[0]) + (-1 * x130 * (*lh_p).Rot[2]);
	const FLT x133 = x129 + (x68 * x131) + (-1 * x70 * x132);
	const FLT x134 = 2 * ((x130 * (*lh_p).Rot[3]) + (-1 * x128 * (*lh_p).Rot[1]) + (x129 * (*lh_p).Rot[0]));
	const FLT x135 = x130 + (x73 * x132) + (-1 * x134 * (*lh_p).Rot[3]);
	const FLT x136 = x128 + (x134 * (*lh_p).Rot[1]) + (-1 * x73 * x131);
	const FLT x137 = (x78 * x136) + (x76 * x135);
	const FLT x138 = x57 * ((-1 * x81 * (x137 + (x72 * x133))) + (x52 * x133));
	const FLT x139 = x55 * x138;
	const FLT x140 = (x53 * ((-1 * x54 * x138) + x139)) + (x56 * x138);
	const FLT x141 = (x85 * x138) + (x53 * x140);
	const FLT x142 = (x96 * x133) + (-1 * x107 * x137);
	const FLT x143 = ((-1 * x112 * x136) + (x111 * x135)) * x113;
	const FLT x144 = x143 + (-1 * x110 * x142);
	const FLT x145 = x115 * x117;
	const FLT x146 = x115 * x121;
	const FLT x147 =
		x143 + (-1 * x123 *
				((x120 * x138) + (x106 * x141) + x142 + (x144 * x146) +
				 (-1 * x119 *
				  ((-1 * x101 *
					((x87 * x138) + (x53 * x141) + (x92 * x138) +
					 (x53 * ((x53 * (x140 + (x53 * (x139 + (x89 * x138) + (-1 * x118 * x138))) + (x90 * x138))) + x141 +
							 (x91 * x138))))) +
				   (-1 * x144 * x145)))));
	const FLT x148 = (x65 * x27) + (-1 * x127);
	const FLT x149 = 2 * ((-1 * x32) + (-1 * x30) + x29 + (-1 * x31));
	const FLT x150 = 1 + (x33 * x149) + (-2 * (x21 * x21));
	const FLT x151 = x63 + (-1 * x27 * x149);
	const FLT x152 = (x151 * (*lh_p).Rot[0]) + (-1 * x148 * (*lh_p).Rot[3]) + (x150 * (*lh_p).Rot[2]);
	const FLT x153 = (x148 * (*lh_p).Rot[1]) + (-1 * x151 * (*lh_p).Rot[2]) + (x150 * (*lh_p).Rot[0]);
	const FLT x154 = x148 + (x68 * x152) + (-1 * x70 * x153);
	const FLT x155 = (x151 * (*lh_p).Rot[3]) + (-1 * x150 * (*lh_p).Rot[1]) + (x148 * (*lh_p).Rot[0]);
	const FLT x156 = x151 + (x73 * x153) + (-1 * x68 * x155);
	const FLT x157 = x150 + (x70 * x155) + (-1 * x73 * x152);
	const FLT x158 = (x78 * x157) + (x76 * x156);
	const FLT x159 = x57 * ((-1 * x81 * (x158 + (x72 * x154))) + (x52 * x154));
	const FLT x160 = x55 * x159;
	const FLT x161 = (x53 * ((-1 * x54 * x159) + x160)) + (x56 * x159);
	const FLT x162 = (x85 * x159) + (x53 * x161);
	const FLT x163 = (x96 * x154) + (-1 * x107 * x158);
	const FLT x164 = ((-1 * x112 * x157) + (x111 * x156)) * x113;
	const FLT x165 = x164 + (-1 * x110 * x163);
	const FLT x166 =
		x164 + (-1 * x123 *
				(x163 + (x165 * x146) +
				 (-1 * x119 *
				  ((-1 * x101 *
					((x87 * x159) +
					 (x53 * ((x53 * (x161 + (x53 * ((x89 * x159) + x160 + (-1 * x118 * x159))) + (x90 * x159))) + x162 +
							 (x91 * x159))) +
					 (x53 * x162) + (x92 * x159))) +
				   (-1 * x165 * x145))) +
				 (x106 * x162) + (x120 * x159)));
	cnMatrixOptionalSet(Hx, 0, 0, x124 + (x124 * x125));
	cnMatrixOptionalSet(Hx, 0, 1, x147 + (x125 * x147));
	cnMatrixOptionalSet(Hx, 0, 2, x166 + (x125 * x166));
}

// Full version Jacobian of SurviveKalmanModel_LightMeas_x_gen2 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]

static inline void SurviveKalmanModel_LightMeas_x_gen2_jac_sensor_pt_with_hx(CnMat *Hx, CnMat *hx, const FLT dt,
																			 const SurviveKalmanModel *_x0,
																			 const FLT *sensor_pt,
																			 const SurvivePose *lh_p,
																			 const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanModel_LightMeas_x_gen2(dt, _x0, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanModel_LightMeas_x_gen2_jac_sensor_pt(Hx, dt, _x0, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanModel_LightMeas_x_gen2 wrt [(*lh_p).Pos[0], (*lh_p).Pos[1], (*lh_p).Pos[2], (*lh_p).Rot[0],
// (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]
static inline void SurviveKalmanModel_LightMeas_x_gen2_jac_lh_p(CnMat *Hx, const FLT dt, const SurviveKalmanModel *_x0,
																const FLT *sensor_pt, const SurvivePose *lh_p,
																const BaseStationCal *bsc0) {
	const FLT x0 = 0.523598775598299 + (*bsc0).tilt;
	const FLT x1 = tan(x0);
	const FLT x2 = dt * (*_x0).Velocity.Pos[2];
	const FLT x3 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x4 = x3 * (*_x0).Acc[2];
	const FLT x5 = dt * dt;
	const FLT x6 = x5 * ((*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x7 = x5 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x8 = x5 * ((*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x9 = 1e-10 + x6 + x7 + x8;
	const FLT x10 = sqrt(x9);
	const FLT x11 = 0.5 * x10;
	const FLT x12 = sin(x11);
	const FLT x13 = (1. / x9) * (x12 * x12);
	const FLT x14 = cos(x11);
	const FLT x15 = 1. / sqrt((x7 * x13) + (x6 * x13) + (x8 * x13) + (x14 * x14));
	const FLT x16 = dt * x15 * x12 * (1. / x10);
	const FLT x17 = x16 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x18 = x15 * x14;
	const FLT x19 = x16 * (*_x0).Pose.Rot[1];
	const FLT x20 = x16 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x21 = (x19 * (*_x0).Velocity.AxisAngleRot[2]) + (-1 * x20 * (*_x0).Pose.Rot[3]) +
					(x17 * (*_x0).Pose.Rot[0]) + (x18 * (*_x0).Pose.Rot[2]);
	const FLT x22 = x16 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x23 = (-1 * x20 * (*_x0).Pose.Rot[1]) + (x18 * (*_x0).Pose.Rot[0]) + (-1 * x22 * (*_x0).Pose.Rot[3]) +
					(-1 * x17 * (*_x0).Pose.Rot[2]);
	const FLT x24 = (x18 * (*_x0).Pose.Rot[3]) + (x20 * (*_x0).Pose.Rot[2]) + (x22 * (*_x0).Pose.Rot[0]) +
					(-1 * x19 * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x25 = (-1 * x24 * sensor_pt[1]) + (x21 * sensor_pt[2]) + (x23 * sensor_pt[0]);
	const FLT x26 = (x20 * (*_x0).Pose.Rot[0]) + (-1 * x22 * (*_x0).Pose.Rot[2]) + (x18 * (*_x0).Pose.Rot[1]) +
					(x17 * (*_x0).Pose.Rot[3]);
	const FLT x27 = (-1 * x26 * sensor_pt[2]) + (x24 * sensor_pt[0]) + (x23 * sensor_pt[1]);
	const FLT x28 = 2 * ((x26 * x27) + (-1 * x25 * x21));
	const FLT x29 = x28 + x4 + sensor_pt[2] + x2 + (*_x0).Pose.Pos[2];
	const FLT x30 = x29 * (*lh_p).Rot[0];
	const FLT x31 = x3 * (*_x0).Acc[1];
	const FLT x32 = dt * (*_x0).Velocity.Pos[1];
	const FLT x33 = (-1 * x21 * sensor_pt[0]) + (x23 * sensor_pt[2]) + (x26 * sensor_pt[1]);
	const FLT x34 = 2 * ((x24 * x25) + (-1 * x33 * x26));
	const FLT x35 = x34 + x32 + (*_x0).Pose.Pos[1] + sensor_pt[1] + x31;
	const FLT x36 = x35 * (*lh_p).Rot[1];
	const FLT x37 = dt * (*_x0).Velocity.Pos[0];
	const FLT x38 = x3 * (*_x0).Acc[0];
	const FLT x39 = 2 * ((x33 * x21) + (-1 * x24 * x27));
	const FLT x40 = x39 + sensor_pt[0] + x37 + (*_x0).Pose.Pos[0] + x38;
	const FLT x41 = x40 * (*lh_p).Rot[2];
	const FLT x42 = (-1 * x41) + x30 + x36;
	const FLT x43 = x29 * (*lh_p).Rot[2];
	const FLT x44 = x40 * (*lh_p).Rot[0];
	const FLT x45 = x35 * (*lh_p).Rot[3];
	const FLT x46 = (-1 * x45) + x43 + x44;
	const FLT x47 = x35 + (*lh_p).Pos[1] + (2 * ((x46 * (*lh_p).Rot[3]) + (-1 * x42 * (*lh_p).Rot[1])));
	const FLT x48 = x47 * x47;
	const FLT x49 = x35 * (*lh_p).Rot[0];
	const FLT x50 = x40 * (*lh_p).Rot[3];
	const FLT x51 = x29 * (*lh_p).Rot[1];
	const FLT x52 = (-1 * x51) + x49 + x50;
	const FLT x53 = x29 + (2 * ((x52 * (*lh_p).Rot[1]) + (-1 * x46 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x54 = x40 + (2 * ((x42 * (*lh_p).Rot[2]) + (-1 * x52 * (*lh_p).Rot[3]))) + (*lh_p).Pos[0];
	const FLT x55 = x54 * x54;
	const FLT x56 = x55 + (x53 * x53);
	const FLT x57 = 1. / x56;
	const FLT x58 = 1. / sqrt(1 + (-1 * (x1 * x1) * x57 * x48));
	const FLT x59 = x54 * x47;
	const FLT x60 = x1 * (1. / (x56 * sqrt(x56)));
	const FLT x61 = x60 * x59;
	const FLT x62 = x53 * x57;
	const FLT x63 = x62 + (x61 * x58);
	const FLT x64 = atan2(-1 * x53, x54);
	const FLT x65 = x1 * (1. / sqrt(x56));
	const FLT x66 = x65 * x47;
	const FLT x67 = (-1 * asin(x66)) + x64 + (*bsc0).ogeephase;
	const FLT x68 = cos(x67) * (*bsc0).ogeemag;
	const FLT x69 = x56 + x48;
	const FLT x70 = cos(x0);
	const FLT x71 = 1. / x70;
	const FLT x72 = x71 * (1. / sqrt(x69));
	const FLT x73 = asin(x72 * x47);
	const FLT x74 = 8.0108022e-06 * x73;
	const FLT x75 = -8.0108022e-06 + (-1 * x74);
	const FLT x76 = 0.0028679863 + (x73 * x75);
	const FLT x77 = 5.3685255e-06 + (x73 * x76);
	const FLT x78 = 0.0076069798 + (x73 * x77);
	const FLT x79 = x73 * x78;
	const FLT x80 = -8.0108022e-06 + (-1.60216044e-05 * x73);
	const FLT x81 = x76 + (x80 * x73);
	const FLT x82 = x77 + (x81 * x73);
	const FLT x83 = x78 + (x82 * x73);
	const FLT x84 = (x83 * x73) + x79;
	const FLT x85 = (sin(x67) * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x86 = sin(x0);
	const FLT x87 = x85 * x86;
	const FLT x88 = x70 + (-1 * x84 * x87);
	const FLT x89 = 1. / x88;
	const FLT x90 = x73 * x73;
	const FLT x91 = x78 * x90;
	const FLT x92 = x89 * x91;
	const FLT x93 = x68 * x92;
	const FLT x94 = 2 * x54;
	const FLT x95 = 1. / sqrt(1 + (-1 * (1. / (x70 * x70)) * (1. / x69) * x48));
	const FLT x96 = x71 * (1. / (x69 * sqrt(x69)));
	const FLT x97 = x96 * x47;
	const FLT x98 = x89 * x85;
	const FLT x99 = x79 * x98;
	const FLT x100 = x99 * x97 * x95;
	const FLT x101 = x84 * x86;
	const FLT x102 = x68 * x101;
	const FLT x103 = x83 * x95;
	const FLT x104 = x59 * x96;
	const FLT x105 = x75 * x95;
	const FLT x106 = -1 * x105 * x104;
	const FLT x107 = 2.40324066e-05 * x73;
	const FLT x108 = x95 * x107;
	const FLT x109 = x80 * x95;
	const FLT x110 = x81 * x95;
	const FLT x111 = x76 * x95;
	const FLT x112 = x74 * x95;
	const FLT x113 = (x73 * ((x104 * x112) + x106)) + (-1 * x104 * x111);
	const FLT x114 = x82 * x95;
	const FLT x115 = x77 * x95;
	const FLT x116 = (-1 * x104 * x115) + (x73 * x113);
	const FLT x117 = x78 * x95;
	const FLT x118 = (1. / (x88 * x88)) * x85 * x91;
	const FLT x119 = x90 * x98;
	const FLT x120 = x66 + (x78 * x119);
	const FLT x121 = 1. / sqrt(1 + (-1 * (x120 * x120)));
	const FLT x122 =
		x62 +
		(-1 * x121 *
		 ((-1 * x61) +
		  (-1 * x118 *
		   ((-1 * x87 *
			 ((x73 * x116) + (-1 * x103 * x104) + (-1 * x104 * x117) +
			  (x73 * (x116 + (x73 * (x113 + (x73 * (x106 + (-1 * x109 * x104) + (x108 * x104))) + (-1 * x104 * x110))) +
					  (-1 * x104 * x114))))) +
			(-1 * x63 * x102))) +
		  (x63 * x93) + (x119 * x116) + (-1 * x94 * x100)));
	const FLT x123 = cos((-1 * asin(x120)) + (*bsc0).gibpha + x64) * (*bsc0).gibmag;
	const FLT x124 = x65 * x58;
	const FLT x125 = (-1 * x96 * x48) + x72;
	const FLT x126 = x105 * x125;
	const FLT x127 = x95 * x125;
	const FLT x128 = (x73 * ((-1 * x74 * x127) + x126)) + (x111 * x125);
	const FLT x129 = (x115 * x125) + (x73 * x128);
	const FLT x130 = 2 * x99;
	const FLT x131 =
		x121 * ((x119 * x129) + (x127 * x130) +
				(-1 * x118 *
				 ((-1 * x87 *
				   ((x117 * x125) + (x73 * x129) + (x103 * x125) +
					(x73 * ((x73 * (x128 + (x73 * (x126 + (x109 * x125) + (-1 * x107 * x127))) + (x110 * x125))) +
							x129 + (x114 * x125))))) +
				  (x102 * x124))) +
				(-1 * x93 * x124) + x65);
	const FLT x132 = x60 * x47;
	const FLT x133 = x53 * x132;
	const FLT x134 = -1 * x54 * x57;
	const FLT x135 = x68 * (x134 + (x58 * x133));
	const FLT x136 = 2 * x53;
	const FLT x137 = x53 * x97;
	const FLT x138 = -1 * x105 * x137;
	const FLT x139 = (x73 * ((x112 * x137) + x138)) + (-1 * x111 * x137);
	const FLT x140 = (-1 * x115 * x137) + (x73 * x139);
	const FLT x141 =
		x134 +
		(-1 * x121 *
		 ((x119 * x140) +
		  (-1 * x118 *
		   ((-1 * x87 *
			 ((-1 * x103 * x137) +
			  (x73 * (x140 + (x73 * (x139 + (x73 * ((-1 * x109 * x137) + (x108 * x137) + x138)) + (-1 * x110 * x137))) +
					  (-1 * x114 * x137))) +
			  (-1 * x117 * x137) + (x73 * x140))) +
			(-1 * x101 * x135))) +
		  (-1 * x133) + (x92 * x135) + (-1 * x100 * x136)));
	const FLT x142 = 2 * x45;
	const FLT x143 = (2 * x43) + (-1 * x142);
	const FLT x144 = 2 * x41;
	const FLT x145 = (2 * x36) + (-1 * x144);
	const FLT x146 = (x136 * x145) + (x94 * x143);
	const FLT x147 = 1.0 / 2.0 * x132;
	const FLT x148 = 2 * x51;
	const FLT x149 = (2 * x50) + (-1 * x148);
	const FLT x150 = (x65 * x149) + (-1 * x146 * x147);
	const FLT x151 = x53 * (1. / x55);
	const FLT x152 = 1. / x54;
	const FLT x153 = x57 * x55;
	const FLT x154 = ((-1 * x145 * x152) + (x143 * x151)) * x153;
	const FLT x155 = x154 + (-1 * x58 * x150);
	const FLT x156 = 2 * x47;
	const FLT x157 = 1.0 / 2.0 * x97;
	const FLT x158 = (-1 * x157 * (x146 + (x149 * x156))) + (x72 * x149);
	const FLT x159 = x95 * x158;
	const FLT x160 = x75 * x159;
	const FLT x161 = (x73 * ((-1 * x74 * x159) + x160)) + (x76 * x159);
	const FLT x162 = (x77 * x159) + (x73 * x161);
	const FLT x163 =
		x154 +
		(-1 * x121 *
		 (x150 + (x93 * x155) + (x119 * x162) +
		  (-1 * x118 *
		   ((-1 * x87 *
			 ((x78 * x159) + (x73 * x162) +
			  (x73 * (x162 + (x73 * ((x73 * ((x109 * x158) + x160 + (-1 * x107 * x159))) + x161 + (x81 * x159))) +
					  (x82 * x159))) +
			  (x103 * x158))) +
			(-1 * x102 * x155))) +
		  (x130 * x159)));
	const FLT x164 = (-1 * x4) + (-1 * x28) + (-1 * (*_x0).Pose.Pos[2]) + (-1 * sensor_pt[2]) + (-1 * x2);
	const FLT x165 = 2 * (*lh_p).Rot[3];
	const FLT x166 = 2 * (*lh_p).Rot[2];
	const FLT x167 = (x35 * x166) + (-1 * x165 * x164);
	const FLT x168 = 2 * (*lh_p).Rot[1];
	const FLT x169 = 2 * x49;
	const FLT x170 = x149 + (x168 * x164) + x169;
	const FLT x171 = (x170 * x136) + (x94 * x167);
	const FLT x172 = 2 * x30;
	const FLT x173 = (-4 * x36) + (-1 * x172) + x144;
	const FLT x174 = (x65 * x173) + (-1 * x171 * x147);
	const FLT x175 = ((-1 * x170 * x152) + (x167 * x151)) * x153;
	const FLT x176 = x175 + (-1 * x58 * x174);
	const FLT x177 = (-1 * x157 * (x171 + (x173 * x156))) + (x72 * x173);
	const FLT x178 = x105 * x177;
	const FLT x179 = x95 * x177;
	const FLT x180 = (x73 * ((-1 * x74 * x179) + x178)) + (x111 * x177);
	const FLT x181 = (x115 * x177) + (x73 * x180);
	const FLT x182 =
		x175 +
		(-1 * x121 *
		 (x174 + (x93 * x176) + (x119 * x181) +
		  (-1 * x118 *
		   ((-1 * x87 *
			 ((x73 * x181) + (x103 * x177) + (x117 * x177) +
			  (x73 * (x181 + (x73 * (x180 + (x73 * (x178 + (x109 * x177) + (-1 * x107 * x179))) + (x110 * x177))) +
					  (x114 * x177))))) +
			(-1 * x102 * x176))) +
		  (x179 * x130)));
	const FLT x183 = (-1 * x39) + (-1 * (*_x0).Pose.Pos[0]) + (-1 * x38) + (-1 * x37) + (-1 * sensor_pt[0]);
	const FLT x184 = x145 + x172 + (x166 * x183);
	const FLT x185 = 2 * x44;
	const FLT x186 = (-1 * x185) + (-4 * x43) + x142;
	const FLT x187 = (x186 * x136) + (x94 * x184);
	const FLT x188 = (x29 * x165) + (-1 * x168 * x183);
	const FLT x189 = (x65 * x188) + (-1 * x187 * x147);
	const FLT x190 = ((-1 * x186 * x152) + (x184 * x151)) * x153;
	const FLT x191 = x190 + (-1 * x58 * x189);
	const FLT x192 = (-1 * x157 * (x187 + (x188 * x156))) + (x72 * x188);
	const FLT x193 = x105 * x192;
	const FLT x194 = (x73 * ((-1 * x112 * x192) + x193)) + (x111 * x192);
	const FLT x195 = (x115 * x192) + (x73 * x194);
	const FLT x196 = x95 * x130;
	const FLT x197 =
		x190 +
		(-1 * x121 *
		 ((x196 * x192) + (x93 * x191) +
		  (-1 * x118 *
		   ((-1 * x87 *
			 ((x117 * x192) + (x73 * x195) + (x103 * x192) +
			  (x73 * (x195 + (x73 * (x194 + (x73 * ((x109 * x192) + x193 + (-1 * x108 * x192))) + (x110 * x192))) +
					  (x114 * x192))))) +
			(-1 * x102 * x191))) +
		  x189 + (x119 * x195)));
	const FLT x198 = x148 + (-1 * x169) + (-4 * x50);
	const FLT x199 = (-1 * x34) + (-1 * x32) + (-1 * (*_x0).Pose.Pos[1]) + (-1 * x31) + (-1 * sensor_pt[1]);
	const FLT x200 = (x40 * x168) + (-1 * x166 * x199);
	const FLT x201 = (x200 * x136) + (x94 * x198);
	const FLT x202 = x143 + (x165 * x199) + x185;
	const FLT x203 = (x65 * x202) + (-1 * x201 * x147);
	const FLT x204 = ((-1 * x200 * x152) + (x198 * x151)) * x153;
	const FLT x205 = x204 + (-1 * x58 * x203);
	const FLT x206 = (-1 * x157 * (x201 + (x202 * x156))) + (x72 * x202);
	const FLT x207 = x206 * x105;
	const FLT x208 = (x73 * ((-1 * x206 * x112) + x207)) + (x206 * x111);
	const FLT x209 = (x206 * x115) + (x73 * x208);
	const FLT x210 =
		x204 +
		(-1 * x121 *
		 (x203 + (x93 * x205) + (x209 * x119) +
		  (-1 * x118 *
		   ((-1 * x87 *
			 ((x206 * x117) +
			  (x73 * (x209 + (x73 * (x208 + (x73 * ((x206 * x109) + x207 + (-1 * x206 * x108))) + (x206 * x110))) +
					  (x206 * x114))) +
			  (x206 * x103) + (x73 * x209))) +
			(-1 * x205 * x102))) +
		  (x206 * x196)));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[0]) / sizeof(FLT), x122 + (x123 * x122));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[1]) / sizeof(FLT), (-1 * x123 * x131) + (-1 * x131));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[2]) / sizeof(FLT), x141 + (x123 * x141));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[0]) / sizeof(FLT), x163 + (x123 * x163));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[1]) / sizeof(FLT), x182 + (x123 * x182));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[2]) / sizeof(FLT), x197 + (x123 * x197));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[3]) / sizeof(FLT), x210 + (x210 * x123));
}

// Full version Jacobian of SurviveKalmanModel_LightMeas_x_gen2 wrt [(*lh_p).Pos[0], (*lh_p).Pos[1], (*lh_p).Pos[2],
// (*lh_p).Rot[0], (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]

static inline void SurviveKalmanModel_LightMeas_x_gen2_jac_lh_p_with_hx(CnMat *Hx, CnMat *hx, const FLT dt,
																		const SurviveKalmanModel *_x0,
																		const FLT *sensor_pt, const SurvivePose *lh_p,
																		const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanModel_LightMeas_x_gen2(dt, _x0, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanModel_LightMeas_x_gen2_jac_lh_p(Hx, dt, _x0, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanModel_LightMeas_x_gen2 wrt [<cnkalman.codegen.WrapMember object at 0x7f88f4ee1730>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4ee17f0>, <cnkalman.codegen.WrapMember object at 0x7f88f4ee1790>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4ee18b0>, <cnkalman.codegen.WrapMember object at 0x7f88f4ee1850>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4ee1520>, <cnkalman.codegen.WrapMember object at 0x7f88f4ee16d0>]
static inline void SurviveKalmanModel_LightMeas_x_gen2_jac_bsc0(CnMat *Hx, const FLT dt, const SurviveKalmanModel *_x0,
																const FLT *sensor_pt, const SurvivePose *lh_p,
																const BaseStationCal *bsc0) {
	const FLT x0 = 0.523598775598299 + (*bsc0).tilt;
	const FLT x1 = cos(x0);
	const FLT x2 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x3 = dt * dt;
	const FLT x4 = x3 * ((*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x5 = x3 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x6 = x3 * ((*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x7 = 1e-10 + x4 + x5 + x6;
	const FLT x8 = sqrt(x7);
	const FLT x9 = 0.5 * x8;
	const FLT x10 = sin(x9);
	const FLT x11 = (1. / x7) * (x10 * x10);
	const FLT x12 = cos(x9);
	const FLT x13 = 1. / sqrt((x6 * x11) + (x5 * x11) + (x4 * x11) + (x12 * x12));
	const FLT x14 = (1. / x8) * dt * x13 * x10;
	const FLT x15 = x14 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x16 = x13 * x12;
	const FLT x17 = x14 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x18 = x14 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x19 = (-1 * x18 * (*_x0).Pose.Rot[3]) + (x17 * (*_x0).Pose.Rot[1]) + (x15 * (*_x0).Pose.Rot[0]) +
					(x16 * (*_x0).Pose.Rot[2]);
	const FLT x20 = (x16 * (*_x0).Pose.Rot[0]) + (-1 * x18 * (*_x0).Pose.Rot[1]) + (-1 * x17 * (*_x0).Pose.Rot[3]) +
					(-1 * x15 * (*_x0).Pose.Rot[2]);
	const FLT x21 = (x18 * (*_x0).Pose.Rot[2]) + (x16 * (*_x0).Pose.Rot[3]) + (x17 * (*_x0).Pose.Rot[0]) +
					(-1 * x15 * (*_x0).Pose.Rot[1]);
	const FLT x22 = (-1 * x21 * sensor_pt[1]) + (x19 * sensor_pt[2]) + (x20 * sensor_pt[0]);
	const FLT x23 = (-1 * x17 * (*_x0).Pose.Rot[2]) + (x18 * (*_x0).Pose.Rot[0]) + (x16 * (*_x0).Pose.Rot[1]) +
					(x15 * (*_x0).Pose.Rot[3]);
	const FLT x24 = (x21 * sensor_pt[0]) + (-1 * x23 * sensor_pt[2]) + (x20 * sensor_pt[1]);
	const FLT x25 = sensor_pt[2] + (x2 * (*_x0).Acc[2]) + (2 * ((x24 * x23) + (-1 * x22 * x19))) +
					(dt * (*_x0).Velocity.Pos[2]) + (*_x0).Pose.Pos[2];
	const FLT x26 = (x20 * sensor_pt[2]) + (-1 * x19 * sensor_pt[0]) + (x23 * sensor_pt[1]);
	const FLT x27 = (2 * ((x22 * x21) + (-1 * x23 * x26))) + sensor_pt[1] + (*_x0).Pose.Pos[1] +
					(dt * (*_x0).Velocity.Pos[1]) + (x2 * (*_x0).Acc[1]);
	const FLT x28 = (*_x0).Pose.Pos[0] + sensor_pt[0] + (2 * ((x26 * x19) + (-1 * x24 * x21))) +
					(dt * (*_x0).Velocity.Pos[0]) + (x2 * (*_x0).Acc[0]);
	const FLT x29 = (-1 * x28 * (*lh_p).Rot[2]) + (x25 * (*lh_p).Rot[0]) + (x27 * (*lh_p).Rot[1]);
	const FLT x30 = (-1 * x27 * (*lh_p).Rot[3]) + (x25 * (*lh_p).Rot[2]) + (x28 * (*lh_p).Rot[0]);
	const FLT x31 = x27 + (*lh_p).Pos[1] + (2 * ((x30 * (*lh_p).Rot[3]) + (-1 * x29 * (*lh_p).Rot[1])));
	const FLT x32 = x31 * x31;
	const FLT x33 = (-1 * x25 * (*lh_p).Rot[1]) + (x27 * (*lh_p).Rot[0]) + (x28 * (*lh_p).Rot[3]);
	const FLT x34 = x25 + (2 * ((x33 * (*lh_p).Rot[1]) + (-1 * x30 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x35 = x28 + (2 * ((x29 * (*lh_p).Rot[2]) + (-1 * x33 * (*lh_p).Rot[3]))) + (*lh_p).Pos[0];
	const FLT x36 = (x35 * x35) + (x34 * x34);
	const FLT x37 = x36 + x32;
	const FLT x38 = x31 * (1. / sqrt(x37));
	const FLT x39 = asin((1. / x1) * x38);
	const FLT x40 = 8.0108022e-06 * x39;
	const FLT x41 = -8.0108022e-06 + (-1 * x40);
	const FLT x42 = 0.0028679863 + (x41 * x39);
	const FLT x43 = 5.3685255e-06 + (x42 * x39);
	const FLT x44 = 0.0076069798 + (x43 * x39);
	const FLT x45 = x39 * x39;
	const FLT x46 = x44 * x39;
	const FLT x47 = -8.0108022e-06 + (-1.60216044e-05 * x39);
	const FLT x48 = x42 + (x47 * x39);
	const FLT x49 = x43 + (x48 * x39);
	const FLT x50 = x44 + (x49 * x39);
	const FLT x51 = (x50 * x39) + x46;
	const FLT x52 = atan2(-1 * x34, x35);
	const FLT x53 = tan(x0);
	const FLT x54 = x31 * (1. / sqrt(x36));
	const FLT x55 = x54 * x53;
	const FLT x56 = (-1 * asin(x55)) + x52 + (*bsc0).ogeephase;
	const FLT x57 = sin(x56);
	const FLT x58 = (x57 * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x59 = sin(x0);
	const FLT x60 = x58 * x59;
	const FLT x61 = x60 * x51;
	const FLT x62 = x1 + (-1 * x61);
	const FLT x63 = 1. / x62;
	const FLT x64 = x63 * x45;
	const FLT x65 = x64 * x44;
	const FLT x66 = (1. / (x62 * x62)) * x44 * x45;
	const FLT x67 = x61 * x66;
	const FLT x68 = x55 + (x65 * x58);
	const FLT x69 = 1. / sqrt(1 + (-1 * (x68 * x68)));
	const FLT x70 = (x67 + x65) * x69;
	const FLT x71 = (-1 * asin(x68)) + (*bsc0).gibpha + x52;
	const FLT x72 = cos(x71) * (*bsc0).gibmag;
	const FLT x73 = ((x67 * x57) + (x65 * x57)) * x69;
	const FLT x74 = cos(x56) * (*bsc0).ogeemag;
	const FLT x75 = x74 * x65;
	const FLT x76 = x69 * ((x74 * x67) + x75);
	const FLT x77 = x53 * x53;
	const FLT x78 = x54 * (1 + x77);
	const FLT x79 = x78 * (1. / sqrt(1 + (-1 * x77 * x32 * (1. / x36))));
	const FLT x80 = 1. / (x1 * x1);
	const FLT x81 = x80 * x38 * (1. / sqrt(1 + (-1 * x80 * x32 * (1. / x37))));
	const FLT x82 = x81 * x59;
	const FLT x83 = x82 * x41;
	const FLT x84 = (x39 * ((-1 * x82 * x40) + x83)) + (x82 * x42);
	const FLT x85 = (x82 * x43) + (x84 * x39);
	const FLT x86 =
		x69 *
		((-1 * x66 * x58 *
		  ((x79 * x74 * x51 * x59) + (-1 * x1 * x51 * x58) +
		   (-1 * x60 *
			((x82 * x44) + (x85 * x39) +
			 (x39 * (x85 + (x39 * (x84 + (x39 * (x83 + (x82 * x47) + (-2.40324066e-05 * x82 * x39))) + (x82 * x48))) +
					 (x82 * x49))) +
			 (x82 * x50))) +
		   (-1 * x59))) +
		 (x85 * x64 * x58) + x78 + (2 * x81 * x60 * x63 * x46) + (-1 * x79 * x75));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, curve) / sizeof(FLT), (-1 * x70 * x72) + (-1 * x70));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibmag) / sizeof(FLT), sin(x71));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibpha) / sizeof(FLT), x72);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, ogeemag) / sizeof(FLT), (-1 * x73 * x72) + (-1 * x73));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, ogeephase) / sizeof(FLT), (-1 * x72 * x76) + (-1 * x76));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, phase) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, tilt) / sizeof(FLT), (-1 * x86 * x72) + (-1 * x86));
}

// Full version Jacobian of SurviveKalmanModel_LightMeas_x_gen2 wrt [<cnkalman.codegen.WrapMember object at
// 0x7f88f4ee1730>, <cnkalman.codegen.WrapMember object at 0x7f88f4ee17f0>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4ee1790>, <cnkalman.codegen.WrapMember object at 0x7f88f4ee18b0>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4ee1850>, <cnkalman.codegen.WrapMember object at 0x7f88f4ee1520>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4ee16d0>]

static inline void SurviveKalmanModel_LightMeas_x_gen2_jac_bsc0_with_hx(CnMat *Hx, CnMat *hx, const FLT dt,
																		const SurviveKalmanModel *_x0,
																		const FLT *sensor_pt, const SurvivePose *lh_p,
																		const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanModel_LightMeas_x_gen2(dt, _x0, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanModel_LightMeas_x_gen2_jac_bsc0(Hx, dt, _x0, sensor_pt, lh_p, bsc0);
	}
}
static inline FLT SurviveKalmanModel_LightMeas_y_gen2(const FLT dt, const SurviveKalmanModel *_x0, const FLT *sensor_pt,
													  const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const FLT x0 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x1 = dt * dt;
	const FLT x2 = x1 * ((*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x3 = x1 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x4 = x1 * ((*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x5 = 1e-10 + x2 + x3 + x4;
	const FLT x6 = sqrt(x5);
	const FLT x7 = 0.5 * x6;
	const FLT x8 = sin(x7);
	const FLT x9 = (1. / x5) * (x8 * x8);
	const FLT x10 = cos(x7);
	const FLT x11 = 1. / sqrt((x4 * x9) + (x3 * x9) + (x2 * x9) + (x10 * x10));
	const FLT x12 = (1. / x6) * x8 * dt * x11;
	const FLT x13 = x12 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x14 = x11 * x10;
	const FLT x15 = x12 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x16 = x12 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x17 = (-1 * x16 * (*_x0).Pose.Rot[3]) + (x15 * (*_x0).Pose.Rot[1]) + (x13 * (*_x0).Pose.Rot[0]) +
					(x14 * (*_x0).Pose.Rot[2]);
	const FLT x18 = (-1 * x16 * (*_x0).Pose.Rot[1]) + (x14 * (*_x0).Pose.Rot[0]) + (-1 * x15 * (*_x0).Pose.Rot[3]) +
					(-1 * x13 * (*_x0).Pose.Rot[2]);
	const FLT x19 = (x14 * (*_x0).Pose.Rot[3]) + (x15 * (*_x0).Pose.Rot[0]) + (x16 * (*_x0).Pose.Rot[2]) +
					(-1 * x13 * (*_x0).Pose.Rot[1]);
	const FLT x20 = (-1 * x19 * sensor_pt[1]) + (x17 * sensor_pt[2]) + (x18 * sensor_pt[0]);
	const FLT x21 = (x16 * (*_x0).Pose.Rot[0]) + (x14 * (*_x0).Pose.Rot[1]) + (-1 * x15 * (*_x0).Pose.Rot[2]) +
					(x13 * (*_x0).Pose.Rot[3]);
	const FLT x22 = (x19 * sensor_pt[0]) + (-1 * x21 * sensor_pt[2]) + (x18 * sensor_pt[1]);
	const FLT x23 = (2 * ((x22 * x21) + (-1 * x20 * x17))) + sensor_pt[2] + (x0 * (*_x0).Acc[2]) +
					(dt * (*_x0).Velocity.Pos[2]) + (*_x0).Pose.Pos[2];
	const FLT x24 = (-1 * x17 * sensor_pt[0]) + (x18 * sensor_pt[2]) + (x21 * sensor_pt[1]);
	const FLT x25 = (*_x0).Pose.Pos[0] + (2 * ((x24 * x17) + (-1 * x22 * x19))) + sensor_pt[0] +
					(dt * (*_x0).Velocity.Pos[0]) + (x0 * (*_x0).Acc[0]);
	const FLT x26 = (2 * ((x20 * x19) + (-1 * x24 * x21))) + (dt * (*_x0).Velocity.Pos[1]) + sensor_pt[1] +
					(*_x0).Pose.Pos[1] + (x0 * (*_x0).Acc[1]);
	const FLT x27 = (-1 * x26 * (*lh_p).Rot[3]) + (x23 * (*lh_p).Rot[2]) + (x25 * (*lh_p).Rot[0]);
	const FLT x28 = (-1 * x23 * (*lh_p).Rot[1]) + (x26 * (*lh_p).Rot[0]) + (x25 * (*lh_p).Rot[3]);
	const FLT x29 = x23 + (2 * ((x28 * (*lh_p).Rot[1]) + (-1 * x27 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x30 = (-1 * x25 * (*lh_p).Rot[2]) + (x23 * (*lh_p).Rot[0]) + (x26 * (*lh_p).Rot[1]);
	const FLT x31 = (2 * ((x30 * (*lh_p).Rot[2]) + (-1 * x28 * (*lh_p).Rot[3]))) + x25 + (*lh_p).Pos[0];
	const FLT x32 = atan2(-1 * x29, x31);
	const FLT x33 = (*lh_p).Pos[1] + x26 + (2 * ((x27 * (*lh_p).Rot[3]) + (-1 * x30 * (*lh_p).Rot[1])));
	const FLT x34 = (x31 * x31) + (x29 * x29);
	const FLT x35 = 0.523598775598299 + (-1 * (*bsc0).tilt);
	const FLT x36 = cos(x35);
	const FLT x37 = asin(x33 * (1. / x36) * (1. / sqrt(x34 + (x33 * x33))));
	const FLT x38 = -1 * (1. / sqrt(x34)) * x33 * tan(x35);
	const FLT x39 = ((*bsc0).ogeemag * sin((-1 * asin(x38)) + (*bsc0).ogeephase + x32)) + (*bsc0).curve;
	const FLT x40 = 0.0028679863 + (x37 * (-8.0108022e-06 + (-8.0108022e-06 * x37)));
	const FLT x41 = 5.3685255e-06 + (x40 * x37);
	const FLT x42 = 0.0076069798 + (x41 * x37);
	const FLT x43 = asin(
		x38 +
		(x42 * (x37 * x37) * x39 *
		 (1. /
		  (x36 + (x39 * sin(x35) *
				  ((x37 * (x42 + (x37 * (x41 + (x37 * (x40 + (x37 * (-8.0108022e-06 + (-1.60216044e-05 * x37))))))))) +
				   (x42 * x37)))))));
	return -1.5707963267949 + x32 + (-1 * (*bsc0).phase) + (-1 * x43) +
		   (-1 * sin(x43 + (-1 * (*bsc0).gibpha) + (-1 * x32)) * (*bsc0).gibmag);
}

// Jacobian of SurviveKalmanModel_LightMeas_y_gen2 wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2],
// (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3],
// (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1],
// (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1],
// (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4eeb1c0>]
static inline void SurviveKalmanModel_LightMeas_y_gen2_jac_x0(CnMat *Hx, const FLT dt, const SurviveKalmanModel *_x0,
															  const FLT *sensor_pt, const SurvivePose *lh_p,
															  const BaseStationCal *bsc0) {
	const FLT x0 = dt * fabs(dt);
	const FLT x1 = 1.0 / 2.0 * x0;
	const FLT x2 = dt * dt;
	const FLT x3 = (*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x4 = x2 * x3;
	const FLT x5 = (*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x6 = x2 * x5;
	const FLT x7 = (*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x8 = x2 * x7;
	const FLT x9 = 1e-10 + x4 + x6 + x8;
	const FLT x10 = sqrt(x9);
	const FLT x11 = 0.5 * x10;
	const FLT x12 = sin(x11);
	const FLT x13 = x12 * x12;
	const FLT x14 = 1. / x9;
	const FLT x15 = x14 * x13;
	const FLT x16 = cos(x11);
	const FLT x17 = (x8 * x15) + (x6 * x15) + (x4 * x15) + (x16 * x16);
	const FLT x18 = 1. / sqrt(x17);
	const FLT x19 = x12 * x18;
	const FLT x20 = 1. / x10;
	const FLT x21 = dt * x20;
	const FLT x22 = x21 * x19;
	const FLT x23 = x22 * (*_x0).Pose.Rot[0];
	const FLT x24 = x18 * x16;
	const FLT x25 = x24 * (*_x0).Pose.Rot[2];
	const FLT x26 = x19 * (*_x0).Pose.Rot[1];
	const FLT x27 = x21 * x26;
	const FLT x28 = x22 * (*_x0).Pose.Rot[3];
	const FLT x29 = (-1 * x28 * (*_x0).Velocity.AxisAngleRot[0]) + (x27 * (*_x0).Velocity.AxisAngleRot[2]) +
					(x23 * (*_x0).Velocity.AxisAngleRot[1]) + x25;
	const FLT x30 = x22 * (*_x0).Pose.Rot[2];
	const FLT x31 = x24 * (*_x0).Pose.Rot[0];
	const FLT x32 = (-1 * x27 * (*_x0).Velocity.AxisAngleRot[0]) + x31 + (-1 * x28 * (*_x0).Velocity.AxisAngleRot[2]) +
					(-1 * x30 * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x33 = x24 * (*_x0).Pose.Rot[3];
	const FLT x34 = x33 + (x23 * (*_x0).Velocity.AxisAngleRot[2]) + (x30 * (*_x0).Velocity.AxisAngleRot[0]) +
					(-1 * x27 * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x35 = (-1 * x34 * sensor_pt[1]) + (x29 * sensor_pt[2]) + (x32 * sensor_pt[0]);
	const FLT x36 = x24 * (*_x0).Pose.Rot[1];
	const FLT x37 = (x23 * (*_x0).Velocity.AxisAngleRot[0]) + (-1 * x30 * (*_x0).Velocity.AxisAngleRot[2]) + x36 +
					(x28 * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x38 = (x34 * sensor_pt[0]) + (-1 * x37 * sensor_pt[2]) + (x32 * sensor_pt[1]);
	const FLT x39 = sensor_pt[2] + (x1 * (*_x0).Acc[2]) + (dt * (*_x0).Velocity.Pos[2]) +
					(2 * ((x38 * x37) + (-1 * x35 * x29))) + (*_x0).Pose.Pos[2];
	const FLT x40 = (-1 * x29 * sensor_pt[0]) + (x32 * sensor_pt[2]) + (x37 * sensor_pt[1]);
	const FLT x41 = (2 * ((x34 * x35) + (-1 * x40 * x37))) + (dt * (*_x0).Velocity.Pos[1]) + sensor_pt[1] +
					(*_x0).Pose.Pos[1] + (x1 * (*_x0).Acc[1]);
	const FLT x42 = (2 * ((x40 * x29) + (-1 * x34 * x38))) + (*_x0).Pose.Pos[0] + sensor_pt[0] +
					(dt * (*_x0).Velocity.Pos[0]) + (x1 * (*_x0).Acc[0]);
	const FLT x43 = (-1 * x42 * (*lh_p).Rot[2]) + (x39 * (*lh_p).Rot[0]) + (x41 * (*lh_p).Rot[1]);
	const FLT x44 = (x39 * (*lh_p).Rot[2]) + (-1 * x41 * (*lh_p).Rot[3]) + (x42 * (*lh_p).Rot[0]);
	const FLT x45 = x41 + (*lh_p).Pos[1] + (2 * ((x44 * (*lh_p).Rot[3]) + (-1 * x43 * (*lh_p).Rot[1])));
	const FLT x46 = x45 * x45;
	const FLT x47 = (-1 * x39 * (*lh_p).Rot[1]) + (x41 * (*lh_p).Rot[0]) + (x42 * (*lh_p).Rot[3]);
	const FLT x48 = x39 + (2 * ((x47 * (*lh_p).Rot[1]) + (-1 * x44 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x49 = x42 + (2 * ((x43 * (*lh_p).Rot[2]) + (-1 * x47 * (*lh_p).Rot[3]))) + (*lh_p).Pos[0];
	const FLT x50 = x49 * x49;
	const FLT x51 = x50 + (x48 * x48);
	const FLT x52 = x51 + x46;
	const FLT x53 = 0.523598775598299 + (-1 * (*bsc0).tilt);
	const FLT x54 = cos(x53);
	const FLT x55 = 1. / x54;
	const FLT x56 = (1. / sqrt(x52)) * x55;
	const FLT x57 = asin(x56 * x45);
	const FLT x58 = 8.0108022e-06 * x57;
	const FLT x59 = -8.0108022e-06 + (-1 * x58);
	const FLT x60 = 0.0028679863 + (x57 * x59);
	const FLT x61 = 5.3685255e-06 + (x60 * x57);
	const FLT x62 = 0.0076069798 + (x61 * x57);
	const FLT x63 = x62 * x57;
	const FLT x64 = -8.0108022e-06 + (-1.60216044e-05 * x57);
	const FLT x65 = x60 + (x64 * x57);
	const FLT x66 = x61 + (x65 * x57);
	const FLT x67 = x62 + (x66 * x57);
	const FLT x68 = (x67 * x57) + x63;
	const FLT x69 = sin(x53);
	const FLT x70 = atan2(-1 * x48, x49);
	const FLT x71 = tan(x53);
	const FLT x72 = x71 * (1. / sqrt(x51));
	const FLT x73 = -1 * x72 * x45;
	const FLT x74 = (-1 * asin(x73)) + (*bsc0).ogeephase + x70;
	const FLT x75 = (sin(x74) * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x76 = x75 * x69;
	const FLT x77 = x54 + (x76 * x68);
	const FLT x78 = 1. / x77;
	const FLT x79 = x57 * x57;
	const FLT x80 = x79 * x75;
	const FLT x81 = x80 * x78;
	const FLT x82 = x73 + (x81 * x62);
	const FLT x83 = 1. / sqrt(1 + (-1 * (x82 * x82)));
	const FLT x84 = 1. / x51;
	const FLT x85 = 1. / sqrt(1 + (-1 * x84 * (x71 * x71) * x46));
	const FLT x86 = (*lh_p).Rot[2] * (*lh_p).Rot[2];
	const FLT x87 = -1 * x0 * x86;
	const FLT x88 = (*lh_p).Rot[3] * (*lh_p).Rot[3];
	const FLT x89 = x1 + (-1 * x0 * x88);
	const FLT x90 = x89 + x87;
	const FLT x91 = 2 * x49;
	const FLT x92 = x0 * (*lh_p).Rot[2] * (*lh_p).Rot[0];
	const FLT x93 = x0 * (*lh_p).Rot[3];
	const FLT x94 = x93 * (*lh_p).Rot[1];
	const FLT x95 = x94 + (-1 * x92);
	const FLT x96 = 2 * x48;
	const FLT x97 = (x96 * x95) + (x91 * x90);
	const FLT x98 = 1.0 / 2.0 * x45;
	const FLT x99 = x71 * (1. / (x51 * sqrt(x51))) * x98;
	const FLT x100 = x0 * (*lh_p).Rot[1];
	const FLT x101 = x100 * (*lh_p).Rot[2];
	const FLT x102 = x93 * (*lh_p).Rot[0];
	const FLT x103 = x102 + x101;
	const FLT x104 = (-1 * x72 * x103) + (x99 * x97);
	const FLT x105 = (1. / x50) * x48;
	const FLT x106 = 1. / x49;
	const FLT x107 = x84 * x50;
	const FLT x108 = ((-1 * x95 * x106) + (x90 * x105)) * x107;
	const FLT x109 = x108 + (-1 * x85 * x104);
	const FLT x110 = cos(x74) * (*bsc0).ogeemag;
	const FLT x111 = x68 * x69;
	const FLT x112 = x110 * x111;
	const FLT x113 = 1. / sqrt(1 + (-1 * (1. / (x54 * x54)) * (1. / x52) * x46));
	const FLT x114 = 2 * x45;
	const FLT x115 = (1. / (x52 * sqrt(x52))) * x55 * x98;
	const FLT x116 = (-1 * x115 * (x97 + (x103 * x114))) + (x56 * x103);
	const FLT x117 = x113 * x116;
	const FLT x118 = x59 * x117;
	const FLT x119 = 2.40324066e-05 * x57;
	const FLT x120 = x64 * x113;
	const FLT x121 = x60 * x113;
	const FLT x122 = (x57 * ((-1 * x58 * x117) + x118)) + (x116 * x121);
	const FLT x123 = x66 * x113;
	const FLT x124 = (x57 * x122) + (x61 * x117);
	const FLT x125 = x67 * x113;
	const FLT x126 = x62 * x113;
	const FLT x127 = x80 * (1. / (x77 * x77)) * x62;
	const FLT x128 = 2 * x78 * x75 * x63;
	const FLT x129 = x79 * x78 * x62;
	const FLT x130 = x110 * x129;
	const FLT x131 =
		x83 *
		((x109 * x130) +
		 (-1 * x127 *
		  ((x76 * ((x116 * x126) +
				   (x57 * (x124 + (x57 * (x122 + (x57 * ((x116 * x120) + x118 + (-1 * x119 * x117))) + (x65 * x117))) +
						   (x116 * x123))) +
				   (x57 * x124) + (x116 * x125))) +
		   (x109 * x112))) +
		 x104 + (x117 * x128) + (x81 * x124));
	const FLT x132 = cos((-1 * asin(x82)) + (*bsc0).gibpha + x70) * (*bsc0).gibmag;
	const FLT x133 = x101 + (-1 * x102);
	const FLT x134 = x93 * (*lh_p).Rot[2];
	const FLT x135 = x100 * (*lh_p).Rot[0];
	const FLT x136 = x135 + x134;
	const FLT x137 = (x96 * x136) + (x91 * x133);
	const FLT x138 = (*lh_p).Rot[1] * (*lh_p).Rot[1];
	const FLT x139 = -1 * x0 * x138;
	const FLT x140 = x89 + x139;
	const FLT x141 = (-1 * x72 * x140) + (x99 * x137);
	const FLT x142 = ((-1 * x106 * x136) + (x105 * x133)) * x107;
	const FLT x143 = x142 + (-1 * x85 * x141);
	const FLT x144 = x113 * ((-1 * x115 * (x137 + (x114 * x140))) + (x56 * x140));
	const FLT x145 = x59 * x144;
	const FLT x146 = (x57 * ((-1 * x58 * x144) + x145)) + (x60 * x144);
	const FLT x147 = (x57 * x146) + (x61 * x144);
	const FLT x148 =
		x83 *
		((-1 * x127 *
		  ((x76 * ((x62 * x144) +
				   (x57 * (x147 + (x57 * (x146 + (x57 * ((x64 * x144) + x145 + (-1 * x119 * x144))) + (x65 * x144))) +
						   (x66 * x144))) +
				   (x57 * x147) + (x67 * x144))) +
		   (x112 * x143))) +
		 x141 + (x130 * x143) + (x81 * x147) + (x128 * x144));
	const FLT x149 = x134 + (-1 * x135);
	const FLT x150 = x92 + x94;
	const FLT x151 = x87 + x1 + x139;
	const FLT x152 = (x96 * x151) + (x91 * x150);
	const FLT x153 = (-1 * x115 * (x152 + (x114 * x149))) + (x56 * x149);
	const FLT x154 = x61 * x113;
	const FLT x155 = x59 * x113;
	const FLT x156 = x153 * x155;
	const FLT x157 = x58 * x113;
	const FLT x158 = (x57 * ((-1 * x153 * x157) + x156)) + (x121 * x153);
	const FLT x159 = (x57 * x158) + (x153 * x154);
	const FLT x160 = x113 * x128;
	const FLT x161 = (-1 * x72 * x149) + (x99 * x152);
	const FLT x162 = ((-1 * x106 * x151) + (x105 * x150)) * x107;
	const FLT x163 = x110 * (x162 + (-1 * x85 * x161));
	const FLT x164 = x113 * x119;
	const FLT x165 = x65 * x113;
	const FLT x166 =
		x83 *
		((x129 * x163) +
		 (-1 * x127 *
		  ((x76 * ((x126 * x153) + (x57 * x159) +
				   (x57 * (x159 + (x57 * (x158 + (x57 * (x156 + (x120 * x153) + (-1 * x164 * x153))) + (x165 * x153))) +
						   (x123 * x153))) +
				   (x125 * x153))) +
		   (x111 * x163))) +
		 x161 + (x81 * x159) + (x160 * x153));
	const FLT x167 = 2 * x86;
	const FLT x168 = -1 * x167;
	const FLT x169 = 2 * x88;
	const FLT x170 = 1 + (-1 * x169);
	const FLT x171 = x170 + x168;
	const FLT x172 = 2 * (*lh_p).Rot[2];
	const FLT x173 = x172 * (*lh_p).Rot[0];
	const FLT x174 = 2 * (*lh_p).Rot[1];
	const FLT x175 = x174 * (*lh_p).Rot[3];
	const FLT x176 = x175 + (-1 * x173);
	const FLT x177 = (x96 * x176) + (x91 * x171);
	const FLT x178 = x172 * (*lh_p).Rot[1];
	const FLT x179 = 2 * (*lh_p).Rot[3];
	const FLT x180 = x179 * (*lh_p).Rot[0];
	const FLT x181 = x180 + x178;
	const FLT x182 = (-1 * x72 * x181) + (x99 * x177);
	const FLT x183 = ((-1 * x106 * x176) + (x105 * x171)) * x107;
	const FLT x184 = x183 + (-1 * x85 * x182);
	const FLT x185 = (-1 * x115 * (x177 + (x114 * x181))) + (x56 * x181);
	const FLT x186 = x113 * x185;
	const FLT x187 = x59 * x186;
	const FLT x188 = (x57 * ((-1 * x58 * x186) + x187)) + (x121 * x185);
	const FLT x189 = (x57 * x188) + (x61 * x186);
	const FLT x190 =
		x83 *
		(x182 + (x184 * x130) + (x81 * x189) +
		 (-1 * x127 *
		  ((x76 * ((x126 * x185) + (x57 * x189) +
				   (x57 * (x189 + (x57 * (x188 + (x57 * (x187 + (x120 * x185) + (-1 * x119 * x186))) + (x65 * x186))) +
						   (x123 * x185))) +
				   (x67 * x186))) +
		   (x112 * x184))) +
		 (x128 * x186));
	const FLT x191 = x178 + (-1 * x180);
	const FLT x192 = x172 * (*lh_p).Rot[3];
	const FLT x193 = x174 * (*lh_p).Rot[0];
	const FLT x194 = x193 + x192;
	const FLT x195 = (x96 * x194) + (x91 * x191);
	const FLT x196 = 2 * x138;
	const FLT x197 = -1 * x196;
	const FLT x198 = x170 + x197;
	const FLT x199 = (-1 * x72 * x198) + (x99 * x195);
	const FLT x200 = ((-1 * x106 * x194) + (x105 * x191)) * x107;
	const FLT x201 = x200 + (-1 * x85 * x199);
	const FLT x202 = (-1 * x115 * (x195 + (x114 * x198))) + (x56 * x198);
	const FLT x203 = x202 * x155;
	const FLT x204 = (x57 * ((-1 * x202 * x157) + x203)) + (x202 * x121);
	const FLT x205 = (x57 * x204) + (x202 * x154);
	const FLT x206 =
		x83 *
		(x199 + (x201 * x130) + (x81 * x205) +
		 (-1 * x127 *
		  ((x76 * ((x57 * x205) +
				   (x57 * (x205 + (x57 * (x204 + (x57 * ((x202 * x120) + x203 + (-1 * x202 * x164))) + (x202 * x165))) +
						   (x202 * x123))) +
				   (x202 * x126) + (x202 * x125))) +
		   (x201 * x112))) +
		 (x202 * x160));
	const FLT x207 = x192 + (-1 * x193);
	const FLT x208 = x173 + x175;
	const FLT x209 = 1 + x197 + x168;
	const FLT x210 = (x96 * x209) + (x91 * x208);
	const FLT x211 = (-1 * x115 * (x210 + (x207 * x114))) + (x56 * x207);
	const FLT x212 = x211 * x155;
	const FLT x213 = (x57 * ((-1 * x211 * x157) + x212)) + (x211 * x121);
	const FLT x214 = (x57 * x213) + (x211 * x154);
	const FLT x215 = (-1 * x72 * x207) + (x99 * x210);
	const FLT x216 = ((-1 * x209 * x106) + (x208 * x105)) * x107;
	const FLT x217 = x216 + (-1 * x85 * x215);
	const FLT x218 =
		x83 *
		((x211 * x160) + x215 + (x81 * x214) + (x217 * x130) +
		 (-1 * x127 *
		  ((x76 * ((x211 * x126) + (x57 * x214) +
				   (x57 * (x214 + (x57 * ((x57 * ((x211 * x120) + x212 + (-1 * x211 * x164))) + x213 + (x211 * x165))) +
						   (x211 * x123))) +
				   (x211 * x125))) +
		   (x217 * x112))));
	const FLT x219 = x22 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x220 = -1 * x219 * sensor_pt[2];
	const FLT x221 = x22 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x222 = x221 * sensor_pt[0];
	const FLT x223 = x24 * sensor_pt[1];
	const FLT x224 = x223 + x220 + x222;
	const FLT x225 = 2 * x34;
	const FLT x226 = 2 * x38;
	const FLT x227 = -1 * x221 * x226;
	const FLT x228 = x22 * sensor_pt[1];
	const FLT x229 = x228 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x230 = x22 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x231 = -1 * x230 * sensor_pt[0];
	const FLT x232 = x24 * sensor_pt[2];
	const FLT x233 = x232 + x231;
	const FLT x234 = x233 + x229;
	const FLT x235 = 2 * x29;
	const FLT x236 = 2 * x40;
	const FLT x237 = x230 * x236;
	const FLT x238 = (x234 * x235) + x237 + (-1 * x224 * x225) + x227;
	const FLT x239 = x230 * sensor_pt[2];
	const FLT x240 = -1 * x228 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x241 = x24 * sensor_pt[0];
	const FLT x242 = x241 + x240;
	const FLT x243 = x242 + x239;
	const FLT x244 = x219 * x226;
	const FLT x245 = 2 * x35;
	const FLT x246 = -1 * x230 * x245;
	const FLT x247 = 2 * x37;
	const FLT x248 = (x224 * x247) + x246 + (-1 * x235 * x243) + x244;
	const FLT x249 = x221 * x245;
	const FLT x250 = -1 * x219 * x236;
	const FLT x251 = x250 + x249 + (-1 * x234 * x247) + (x225 * x243);
	const FLT x252 = (x251 * (*lh_p).Rot[1]) + (-1 * x238 * (*lh_p).Rot[2]) + (x248 * (*lh_p).Rot[0]);
	const FLT x253 = (x238 * (*lh_p).Rot[0]) + (-1 * x251 * (*lh_p).Rot[3]) + (x248 * (*lh_p).Rot[2]);
	const FLT x254 = x251 + (-1 * x252 * x174) + (x253 * x179);
	const FLT x255 = (x238 * (*lh_p).Rot[3]) + (-1 * x248 * (*lh_p).Rot[1]) + (x251 * (*lh_p).Rot[0]);
	const FLT x256 = x238 + (-1 * x255 * x179) + (x252 * x172);
	const FLT x257 = x248 + (-1 * x253 * x172) + (x255 * x174);
	const FLT x258 = (x96 * x257) + (x91 * x256);
	const FLT x259 = (-1 * x115 * (x258 + (x254 * x114))) + (x56 * x254);
	const FLT x260 = x259 * x113;
	const FLT x261 = x59 * x260;
	const FLT x262 = (x57 * ((-1 * x58 * x260) + x261)) + (x259 * x121);
	const FLT x263 = (x57 * x262) + (x61 * x260);
	const FLT x264 = (-1 * x72 * x254) + (x99 * x258);
	const FLT x265 = ((-1 * x257 * x106) + (x256 * x105)) * x107;
	const FLT x266 = x265 + (-1 * x85 * x264);
	const FLT x267 =
		x83 *
		(x264 + (x266 * x130) +
		 (-1 * x127 *
		  ((x76 * ((x259 * x126) + (x57 * x263) +
				   (x57 * (x263 + (x57 * (x262 + (x57 * ((x259 * x120) + x261 + (-1 * x260 * x119))) + (x259 * x165))) +
						   (x259 * x123))) +
				   (x259 * x125))) +
		   (x266 * x112))) +
		 (x260 * x128) + (x81 * x263));
	const FLT x268 = x230 * sensor_pt[1];
	const FLT x269 = x221 * sensor_pt[2];
	const FLT x270 = x219 * sensor_pt[0];
	const FLT x271 = (-1 * x270) + x268 + x269;
	const FLT x272 = -1 * x229;
	const FLT x273 = x272 + (-1 * x232) + x231;
	const FLT x274 = x24 * x226;
	const FLT x275 = (-1 * x249) + (-1 * x235 * x271) + x274 + (x273 * x247);
	const FLT x276 = (-1 * x222) + x220;
	const FLT x277 = x276 + x223;
	const FLT x278 = x24 * x236;
	const FLT x279 = x246 + (-1 * x277 * x247) + (-1 * x278) + (x271 * x225);
	const FLT x280 = x236 * x221;
	const FLT x281 = x230 * x226;
	const FLT x282 = (x235 * x277) + x280 + x281 + (-1 * x273 * x225);
	const FLT x283 = (x282 * (*lh_p).Rot[3]) + (-1 * x275 * (*lh_p).Rot[1]) + (x279 * (*lh_p).Rot[0]);
	const FLT x284 = (x279 * (*lh_p).Rot[1]) + (-1 * x282 * (*lh_p).Rot[2]) + (x275 * (*lh_p).Rot[0]);
	const FLT x285 = x282 + (-1 * x283 * x179) + (x284 * x172);
	const FLT x286 = (x282 * (*lh_p).Rot[0]) + (-1 * x279 * (*lh_p).Rot[3]) + (x275 * (*lh_p).Rot[2]);
	const FLT x287 = x275 + (-1 * x286 * x172) + (x283 * x174);
	const FLT x288 = (x96 * x287) + (x91 * x285);
	const FLT x289 = x279 + (-1 * x284 * x174) + (x286 * x179);
	const FLT x290 = (-1 * x72 * x289) + (x99 * x288);
	const FLT x291 = ((-1 * x287 * x106) + (x285 * x105)) * x107;
	const FLT x292 = x291 + (-1 * x85 * x290);
	const FLT x293 = (-1 * x115 * (x288 + (x289 * x114))) + (x56 * x289);
	const FLT x294 = x293 * x155;
	const FLT x295 = (x57 * ((-1 * x293 * x157) + x294)) + (x293 * x121);
	const FLT x296 = (x57 * x295) + (x293 * x154);
	const FLT x297 =
		x83 *
		((x292 * x130) + (x81 * x296) + x290 +
		 (-1 * x127 *
		  ((x76 * ((x293 * x126) + (x57 * x296) +
				   (x57 * (x296 + (x57 * ((x57 * ((x293 * x120) + x294 + (-1 * x293 * x164))) + x295 + (x293 * x165))) +
						   (x293 * x123))) +
				   (x293 * x125))) +
		   (x292 * x112))) +
		 (x293 * x160));
	const FLT x298 = -1 * x239;
	const FLT x299 = (-1 * x241) + x240 + x298;
	const FLT x300 = x233 + x272;
	const FLT x301 = x219 * x245;
	const FLT x302 = x301 + (x225 * x300) + (-1 * x299 * x247) + x280;
	const FLT x303 = x24 * x245;
	const FLT x304 = (-1 * x268) + x269 + x270;
	const FLT x305 = (x247 * x304) + (-1 * x235 * x300) + x227 + (-1 * x303);
	const FLT x306 = (x235 * x299) + x278 + (-1 * x244) + (-1 * x225 * x304);
	const FLT x307 = (x306 * (*lh_p).Rot[3]) + (x302 * (*lh_p).Rot[0]) + (-1 * x305 * (*lh_p).Rot[1]);
	const FLT x308 = (x302 * (*lh_p).Rot[1]) + (x305 * (*lh_p).Rot[0]) + (-1 * x306 * (*lh_p).Rot[2]);
	const FLT x309 = x306 + (-1 * x307 * x179) + (x308 * x172);
	const FLT x310 = (x306 * (*lh_p).Rot[0]) + (-1 * x302 * (*lh_p).Rot[3]) + (x305 * (*lh_p).Rot[2]);
	const FLT x311 = x305 + (-1 * x310 * x172) + (x307 * x174);
	const FLT x312 = (x96 * x311) + (x91 * x309);
	const FLT x313 = x302 + (-1 * x308 * x174) + (x310 * x179);
	const FLT x314 = (-1 * x72 * x313) + (x99 * x312);
	const FLT x315 = ((-1 * x311 * x106) + (x309 * x105)) * x107;
	const FLT x316 = x315 + (-1 * x85 * x314);
	const FLT x317 = (-1 * x115 * (x312 + (x313 * x114))) + (x56 * x313);
	const FLT x318 = x317 * x155;
	const FLT x319 = (x57 * ((-1 * x317 * x157) + x318)) + (x317 * x121);
	const FLT x320 = (x57 * x319) + (x317 * x154);
	const FLT x321 =
		x83 *
		((x316 * x130) + (x81 * x320) + x314 +
		 (-1 * x127 *
		  ((x76 * ((x57 * x320) +
				   (x57 * (x320 + (x57 * (x319 + (x57 * ((x317 * x120) + x318 + (-1 * x317 * x164))) + (x317 * x165))) +
						   (x317 * x123))) +
				   (x317 * x126) + (x317 * x125))) +
		   (x316 * x112))) +
		 (x317 * x160));
	const FLT x322 = 2 * (x268 + x270 + (-1 * x269));
	const FLT x323 = x276 + (-1 * x223);
	const FLT x324 = (-1 * x237) + x303 + (-1 * x37 * x322) + (x225 * x323);
	const FLT x325 = x242 + x298;
	const FLT x326 = (x247 * x325) + x281 + (-1 * x235 * x323) + x301;
	const FLT x327 = x250 + (-1 * x274) + (x29 * x322) + (-1 * x225 * x325);
	const FLT x328 = (x327 * (*lh_p).Rot[0]) + (-1 * x324 * (*lh_p).Rot[3]) + (x326 * (*lh_p).Rot[2]);
	const FLT x329 = (x324 * (*lh_p).Rot[1]) + (-1 * x327 * (*lh_p).Rot[2]) + (x326 * (*lh_p).Rot[0]);
	const FLT x330 = x324 + (x328 * x179) + (-1 * x329 * x174);
	const FLT x331 = (x327 * (*lh_p).Rot[3]) + (-1 * x326 * (*lh_p).Rot[1]) + (x324 * (*lh_p).Rot[0]);
	const FLT x332 = x327 + (-1 * x331 * x179) + (x329 * x172);
	const FLT x333 = x326 + (-1 * x328 * x172) + (x331 * x174);
	const FLT x334 = (x96 * x333) + (x91 * x332);
	const FLT x335 = (-1 * x115 * (x334 + (x330 * x114))) + (x56 * x330);
	const FLT x336 = x335 * x155;
	const FLT x337 = (x57 * ((-1 * x335 * x157) + x336)) + (x335 * x121);
	const FLT x338 = (x57 * x337) + (x335 * x154);
	const FLT x339 = (-1 * x72 * x330) + (x99 * x334);
	const FLT x340 = ((-1 * x333 * x106) + (x332 * x105)) * x107;
	const FLT x341 = x340 + (-1 * x85 * x339);
	const FLT x342 =
		x83 *
		(x339 +
		 (-1 * x127 *
		  ((x76 * ((x57 * x338) + (x335 * x126) +
				   (x57 * (x338 + (x57 * (x337 + (x57 * (x336 + (x335 * x120) + (-1 * x335 * x164))) + (x335 * x165))) +
						   (x335 * x123))) +
				   (x335 * x125))) +
		   (x341 * x112))) +
		 (x341 * x130) + (x335 * x160) + (x81 * x338));
	const FLT x343 = x19 * (*_x0).Pose.Rot[2];
	const FLT x344 = 0.5 * x20;
	const FLT x345 = x2 * x344;
	const FLT x346 = x345 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x347 = -1 * x28;
	const FLT x348 = dt * dt * dt;
	const FLT x349 = 0.5 * x14 * x348;
	const FLT x350 = x33 * x349;
	const FLT x351 = 1.0 / 2.0 * (1. / (x17 * sqrt(x17)));
	const FLT x352 = x21 * x12 * x351;
	const FLT x353 = x352 * (*_x0).Pose.Rot[3];
	const FLT x354 =
		(*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x355 = dt * dt * dt * dt;
	const FLT x356 = 2 * (1. / (x9 * x9)) * x13;
	const FLT x357 = x356 * x355;
	const FLT x358 = 1. / (x9 * sqrt(x9));
	const FLT x359 = 1.0 * x12 * x16;
	const FLT x360 = x359 * x358;
	const FLT x361 = x360 * x355;
	const FLT x362 = x3 * x361;
	const FLT x363 = x3 * x357;
	const FLT x364 = x20 * x359;
	const FLT x365 = x2 * x364;
	const FLT x366 = 2 * x15;
	const FLT x367 = x2 * x366;
	const FLT x368 = x5 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x369 = (-1 * x368 * x357) + (x361 * x368) + (x367 * (*_x0).Velocity.AxisAngleRot[0]) +
					 (x362 * (*_x0).Velocity.AxisAngleRot[0]) + (-1 * x354 * x357) +
					 (-1 * x363 * (*_x0).Velocity.AxisAngleRot[0]) + (x361 * x354) +
					 (-1 * x365 * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x370 = x369 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x371 = x16 * x351;
	const FLT x372 = x371 * x369;
	const FLT x373 = x352 * (*_x0).Pose.Rot[0];
	const FLT x374 = x369 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x375 = x358 * x348;
	const FLT x376 = x375 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x377 = x376 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x378 = x26 * x377;
	const FLT x379 = (*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x380 = x379 * x349;
	const FLT x381 = x36 * x380;
	const FLT x382 = x352 * (*_x0).Pose.Rot[1];
	const FLT x383 = x369 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x384 = x19 * (*_x0).Pose.Rot[3];
	const FLT x385 = x7 * x375;
	const FLT x386 = (*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x387 = x31 * x349;
	const FLT x388 = x19 * (*_x0).Pose.Rot[0];
	const FLT x389 = x388 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x390 = x375 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x391 = (-1 * x390 * x389) + (x387 * x386);
	const FLT x392 = x381 + (-1 * x378) + (-1 * x374 * x373) + x391 + (x384 * x385) + x347 + (-1 * x383 * x382) +
					 (-1 * x346 * x343) + (-1 * x7 * x350) + (x370 * x353) + (-1 * x372 * (*_x0).Pose.Rot[2]);
	const FLT x393 = x350 * x386;
	const FLT x394 = x390 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x395 = x394 * x384;
	const FLT x396 = x25 * x380;
	const FLT x397 = x377 * x343;
	const FLT x398 = x352 * (*_x0).Pose.Rot[2];
	const FLT x399 = x7 * x349;
	const FLT x400 = x19 * x385;
	const FLT x401 = (-1 * x373 * x370) + x23 + (-1 * x26 * x346) + x393 + (x31 * x399) +
					 (-1 * x372 * (*_x0).Pose.Rot[1]) + (-1 * x395) + x397 + (x398 * x383) + (-1 * x396) +
					 (-1 * x400 * (*_x0).Pose.Rot[0]) + (-1 * x374 * x353);
	const FLT x402 = (x31 * x380) + (-1 * x376 * x389);
	const FLT x403 = x36 * x349;
	const FLT x404 = (-1 * x403 * x386) + (x26 * x394);
	const FLT x405 = (-1 * x398 * x370) + (-1 * x384 * x346) + x404 + x30 + (-1 * x373 * x383) +
					 (-1 * x372 * (*_x0).Pose.Rot[3]) + (x25 * x399) + (-1 * x400 * (*_x0).Pose.Rot[2]) + x402 +
					 (x374 * x382);
	const FLT x406 = x25 * x349;
	const FLT x407 = x406 * x386;
	const FLT x408 = x343 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x409 = x408 * x375 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x410 = -1 * x27;
	const FLT x411 = (-1 * x379 * x350) + (x377 * x384);
	const FLT x412 = (x26 * x385) + x410 + (-1 * x389 * x345) + (x382 * x370) + x411 + (-1 * x36 * x399) +
					 (-1 * x372 * (*_x0).Pose.Rot[0]) + (-1 * x407) + x409 + (x374 * x398) + (x383 * x353);
	const FLT x413 = (x412 * sensor_pt[1]) + (-1 * x401 * sensor_pt[2]) + (x405 * sensor_pt[0]);
	const FLT x414 = (x401 * sensor_pt[1]) + (-1 * x392 * sensor_pt[0]) + (x412 * sensor_pt[2]);
	const FLT x415 = (x414 * x235) + (-1 * x405 * x226) + (x236 * x392) + (-1 * x413 * x225);
	const FLT x416 = (x412 * sensor_pt[0]) + (-1 * x405 * sensor_pt[1]) + (x392 * sensor_pt[2]);
	const FLT x417 = (-1 * x416 * x235) + (x401 * x226) + (x413 * x247) + (-1 * x245 * x392);
	const FLT x418 = (-1 * x401 * x236) + (x416 * x225) + (-1 * x414 * x247) + (x405 * x245);
	const FLT x419 = (-1 * x415 * (*lh_p).Rot[2]) + (x418 * (*lh_p).Rot[1]) + (x417 * (*lh_p).Rot[0]);
	const FLT x420 = (x415 * (*lh_p).Rot[0]) + (x417 * (*lh_p).Rot[2]) + (-1 * x418 * (*lh_p).Rot[3]);
	const FLT x421 = x418 + (-1 * x419 * x174) + (x420 * x179);
	const FLT x422 = (-1 * x417 * (*lh_p).Rot[1]) + (x415 * (*lh_p).Rot[3]) + (x418 * (*lh_p).Rot[0]);
	const FLT x423 = x415 + (-1 * x422 * x179) + (x419 * x172);
	const FLT x424 = x417 + (-1 * x420 * x172) + (x422 * x174);
	const FLT x425 = (x96 * x424) + (x91 * x423);
	const FLT x426 = x113 * ((-1 * x115 * (x425 + (x421 * x114))) + (x56 * x421));
	const FLT x427 = x59 * x426;
	const FLT x428 = (x57 * ((-1 * x58 * x426) + x427)) + (x60 * x426);
	const FLT x429 = (x57 * x428) + (x61 * x426);
	const FLT x430 = (-1 * x72 * x421) + (x99 * x425);
	const FLT x431 = ((-1 * x424 * x106) + (x423 * x105)) * x107;
	const FLT x432 = x431 + (-1 * x85 * x430);
	const FLT x433 =
		x83 *
		(x430 + (x432 * x130) + (x426 * x128) + (x81 * x429) +
		 (-1 * x127 *
		  ((x76 * ((x57 * x429) + (x62 * x426) +
				   (x57 * (x429 + (x57 * (x428 + (x57 * ((x64 * x426) + (-1 * x426 * x119) + x427)) + (x65 * x426))) +
						   (x66 * x426))) +
				   (x67 * x426))) +
		   (x432 * x112))));
	const FLT x434 = x357 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x435 = x361 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x436 =
		(*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x437 = (x436 * x361) + (-1 * x5 * x434) + (-1 * x365 * (*_x0).Velocity.AxisAngleRot[1]) + (x5 * x435) +
					 (-1 * x7 * x434) + (-1 * x436 * x357) + (x7 * x435) + (x367 * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x438 = x437 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x439 = x3 * x375;
	const FLT x440 = x437 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x441 = x437 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x442 = x3 * x349;
	const FLT x443 = x437 * x371;
	const FLT x444 = x345 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x445 = x376 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x446 = (*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x447 = (-1 * x406 * x446) + (x445 * x343);
	const FLT x448 = x447 + x391 + (-1 * x443 * (*_x0).Pose.Rot[1]) + (-1 * x26 * x444) + (x33 * x442) + x28 +
					 (-1 * x438 * x373) + (-1 * x439 * x384) + (x440 * x398) + (-1 * x441 * x353);
	const FLT x449 = (x446 * x387) + (-1 * x445 * x388);
	const FLT x450 = (-1 * x444 * x384) + (-1 * x409) + (-1 * x36 * x442) + (-1 * x438 * x398) + x407 + (x26 * x439) +
					 (-1 * x443 * (*_x0).Pose.Rot[3]) + x449 + x410 + (-1 * x440 * x373) + (x441 * x382);
	const FLT x451 = -1 * x30;
	const FLT x452 = x446 * x350;
	const FLT x453 = x445 * x384;
	const FLT x454 = x404 + (-1 * x444 * x388) + x453 + (-1 * x443 * (*_x0).Pose.Rot[0]) + (x440 * x353) + (-1 * x452) +
					 (x439 * x343) + (x441 * x398) + x451 + (x438 * x382) + (-1 * x25 * x442);
	const FLT x455 = (x454 * sensor_pt[1]) + (-1 * x448 * sensor_pt[2]) + (x450 * sensor_pt[0]);
	const FLT x456 = x26 * x445;
	const FLT x457 = x403 * x446;
	const FLT x458 = (-1 * x393) + (-1 * x441 * x373) + (-1 * x439 * x388) + x457 + (x438 * x353) + x395 + (-1 * x456) +
					 (-1 * x443 * (*_x0).Pose.Rot[2]) + (-1 * x440 * x382) + x23 + (-1 * x408 * x345) + (x31 * x442);
	const FLT x459 = (x448 * sensor_pt[1]) + (-1 * x458 * sensor_pt[0]) + (x454 * sensor_pt[2]);
	const FLT x460 = (-1 * x450 * x226) + (x458 * x236) + (-1 * x455 * x225) + (x459 * x235);
	const FLT x461 = 2 * ((-1 * x450 * sensor_pt[1]) + (x454 * sensor_pt[0]) + (x458 * sensor_pt[2]));
	const FLT x462 = (x455 * x247) + (-1 * x29 * x461) + (-1 * x458 * x245) + (x448 * x226);
	const FLT x463 = (x450 * x245) + (x34 * x461) + (-1 * x459 * x247) + (-1 * x448 * x236);
	const FLT x464 = (-1 * x460 * (*lh_p).Rot[2]) + (x463 * (*lh_p).Rot[1]) + (x462 * (*lh_p).Rot[0]);
	const FLT x465 = (x460 * (*lh_p).Rot[0]) + (-1 * x463 * (*lh_p).Rot[3]) + (x462 * (*lh_p).Rot[2]);
	const FLT x466 = x463 + (-1 * x464 * x174) + (x465 * x179);
	const FLT x467 = (x460 * (*lh_p).Rot[3]) + (-1 * x462 * (*lh_p).Rot[1]) + (x463 * (*lh_p).Rot[0]);
	const FLT x468 = x460 + (-1 * x467 * x179) + (x464 * x172);
	const FLT x469 = x462 + (-1 * x465 * x172) + (x467 * x174);
	const FLT x470 = (x96 * x469) + (x91 * x468);
	const FLT x471 = x113 * ((-1 * x115 * (x470 + (x466 * x114))) + (x56 * x466));
	const FLT x472 = x59 * x471;
	const FLT x473 = (x57 * ((-1 * x58 * x471) + x472)) + (x60 * x471);
	const FLT x474 = (x57 * x473) + (x61 * x471);
	const FLT x475 = (-1 * x72 * x466) + (x99 * x470);
	const FLT x476 = ((-1 * x469 * x106) + (x468 * x105)) * x107;
	const FLT x477 = x476 + (-1 * x85 * x475);
	const FLT x478 =
		x83 *
		(x475 + (x471 * x128) + (x477 * x130) + (x81 * x474) +
		 (-1 * x127 *
		  ((x76 * ((x62 * x471) + (x57 * x474) +
				   (x57 * (x474 + (x57 * ((x57 * ((x64 * x471) + x472 + (-1 * x471 * x119))) + x473 + (x65 * x471))) +
						   (x66 * x471))) +
				   (x67 * x471))) +
		   (x477 * x112))));
	const FLT x479 = x7 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x480 =
		x355 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x481 = x2 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x482 = (-1 * x480 * x356) + (-1 * x363 * (*_x0).Velocity.AxisAngleRot[2]) + (x481 * x366) +
					 (-1 * x479 * x357) + (-1 * x481 * x364) + (x479 * x361) + (x480 * x360) +
					 (x362 * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x483 = x482 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x484 = x5 * x375;
	const FLT x485 = x482 * x353;
	const FLT x486 = x482 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x487 = x482 * x371;
	const FLT x488 = x481 * x344;
	const FLT x489 = x5 * x349;
	const FLT x490 = (-1 * x25 * x489) + (-1 * x26 * x488) + (-1 * x487 * (*_x0).Pose.Rot[1]) +
					 (-1 * x485 * (*_x0).Velocity.AxisAngleRot[1]) + (x484 * x343) + (x483 * x398) + (-1 * x453) +
					 x452 + x402 + (-1 * x486 * x373) + x451;
	const FLT x491 = x482 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x492 = x491 * x352;
	const FLT x493 = (-1 * x397) + x23 + (-1 * x486 * x398) + x396 + (x492 * (*_x0).Pose.Rot[1]) + (-1 * x484 * x388) +
					 x456 + (-1 * x483 * x373) + (x31 * x489) + (-1 * x488 * x384) + (-1 * x487 * (*_x0).Pose.Rot[3]) +
					 (-1 * x457);
	const FLT x494 = (-1 * x488 * x388) + (x486 * x382) + (-1 * x487 * (*_x0).Pose.Rot[0]) + x347 + x447 +
					 (-1 * x5 * x350) + (-1 * x381) + (x485 * (*_x0).Velocity.AxisAngleRot[2]) + (x484 * x384) + x378 +
					 (x491 * x398);
	const FLT x495 = (x494 * sensor_pt[1]) + (-1 * x490 * sensor_pt[2]) + (x493 * sensor_pt[0]);
	const FLT x496 = x411 + (-1 * x483 * x382) + (-1 * x26 * x484) + x27 + (-1 * x488 * x343) + x449 + (x36 * x489) +
					 (x485 * (*_x0).Velocity.AxisAngleRot[0]) + (-1 * x492 * (*_x0).Pose.Rot[0]) +
					 (-1 * x487 * (*_x0).Pose.Rot[2]);
	const FLT x497 = 2 * x496;
	const FLT x498 = 2 * ((x490 * sensor_pt[1]) + (-1 * x496 * sensor_pt[0]) + (x494 * sensor_pt[2]));
	const FLT x499 = (x29 * x498) + (-1 * x493 * x226) + (-1 * x495 * x225) + (x40 * x497);
	const FLT x500 = (x494 * sensor_pt[0]) + (-1 * x493 * sensor_pt[1]) + (x496 * sensor_pt[2]);
	const FLT x501 = (x490 * x226) + (-1 * x35 * x497) + (-1 * x500 * x235) + (x495 * x247);
	const FLT x502 = (-1 * x490 * x236) + (x500 * x225) + (-1 * x37 * x498) + (x493 * x245);
	const FLT x503 = (-1 * x499 * (*lh_p).Rot[2]) + (x502 * (*lh_p).Rot[1]) + (x501 * (*lh_p).Rot[0]);
	const FLT x504 = 2 * ((x499 * (*lh_p).Rot[0]) + (-1 * x502 * (*lh_p).Rot[3]) + (x501 * (*lh_p).Rot[2]));
	const FLT x505 = (-1 * x503 * x174) + x502 + (x504 * (*lh_p).Rot[3]);
	const FLT x506 = 2 * ((x499 * (*lh_p).Rot[3]) + (-1 * x501 * (*lh_p).Rot[1]) + (x502 * (*lh_p).Rot[0]));
	const FLT x507 = x499 + (-1 * x506 * (*lh_p).Rot[3]) + (x503 * x172);
	const FLT x508 = x501 + (x506 * (*lh_p).Rot[1]) + (-1 * x504 * (*lh_p).Rot[2]);
	const FLT x509 = (x96 * x508) + (x91 * x507);
	const FLT x510 = (-1 * x115 * (x509 + (x505 * x114))) + (x56 * x505);
	const FLT x511 = x510 * x155;
	const FLT x512 = (x57 * ((-1 * x510 * x157) + x511)) + (x510 * x121);
	const FLT x513 = (x57 * x512) + (x510 * x154);
	const FLT x514 = (-1 * x72 * x505) + (x99 * x509);
	const FLT x515 = ((-1 * x508 * x106) + (x507 * x105)) * x107;
	const FLT x516 = x515 + (-1 * x85 * x514);
	const FLT x517 =
		x83 *
		((x516 * x130) +
		 (-1 * x127 *
		  ((x76 * ((x510 * x126) +
				   (x57 * (x513 + (x57 * (x512 + (x57 * ((x510 * x120) + x511 + (-1 * x510 * x164))) + (x510 * x165))) +
						   (x510 * x123))) +
				   (x57 * x513) + (x510 * x125))) +
		   (x516 * x112))) +
		 x514 + (x81 * x513) + (x510 * x160));
	const FLT x518 = dt * x178;
	const FLT x519 = dt * x180;
	const FLT x520 = x519 + x518;
	const FLT x521 = -1 * dt * x169;
	const FLT x522 = (-1 * dt * x167) + dt;
	const FLT x523 = x522 + x521;
	const FLT x524 = dt * x173;
	const FLT x525 = dt * x175;
	const FLT x526 = x525 + (-1 * x524);
	const FLT x527 = (x96 * x526) + (x91 * x523);
	const FLT x528 = (-1 * x115 * (x527 + (x520 * x114))) + (x56 * x520);
	const FLT x529 = x528 * x155;
	const FLT x530 = (x57 * ((-1 * x528 * x157) + x529)) + (x528 * x121);
	const FLT x531 = (x57 * x530) + (x528 * x154);
	const FLT x532 = (-1 * x72 * x520) + (x99 * x527);
	const FLT x533 = ((-1 * x526 * x106) + (x523 * x105)) * x107;
	const FLT x534 = x533 + (-1 * x85 * x532);
	const FLT x535 =
		x83 *
		(x532 + (x534 * x130) +
		 (-1 * x127 *
		  ((x76 * ((x57 * (x531 + (x57 * (x530 + (x57 * ((x528 * x120) + x529 + (-1 * x528 * x164))) + (x528 * x165))) +
						   (x528 * x123))) +
				   (x528 * x126) + (x57 * x531) + (x528 * x125))) +
		   (x534 * x112))) +
		 (x528 * x160) + (x81 * x531));
	const FLT x536 = -1 * dt * x196;
	const FLT x537 = x536 + x521 + dt;
	const FLT x538 = x518 + (-1 * x519);
	const FLT x539 = dt * x192;
	const FLT x540 = dt * x193;
	const FLT x541 = x540 + x539;
	const FLT x542 = (x96 * x541) + (x91 * x538);
	const FLT x543 = x113 * ((-1 * x115 * (x542 + (x537 * x114))) + (x56 * x537));
	const FLT x544 = x59 * x543;
	const FLT x545 = (x57 * ((-1 * x58 * x543) + x544)) + (x60 * x543);
	const FLT x546 = (x57 * x545) + (x61 * x543);
	const FLT x547 = (-1 * x72 * x537) + (x99 * x542);
	const FLT x548 = ((-1 * x541 * x106) + (x538 * x105)) * x107;
	const FLT x549 = x548 + (-1 * x85 * x547);
	const FLT x550 =
		x83 * (x547 + (x543 * x128) + (x549 * x130) +
			   (-1 * x127 *
				((x76 * ((x62 * x543) + (x57 * x546) +
						 (x57 * ((x57 * (x545 + (x57 * ((x64 * x543) + x544 + (-1 * x543 * x119))) + (x65 * x543))) +
								 x546 + (x66 * x543))) +
						 (x67 * x543))) +
				 (x549 * x112))) +
			   (x81 * x546));
	const FLT x551 = x524 + x525;
	const FLT x552 = x522 + x536;
	const FLT x553 = (x96 * x552) + (x91 * x551);
	const FLT x554 = x539 + (-1 * x540);
	const FLT x555 = (-1 * x72 * x554) + (x99 * x553);
	const FLT x556 = ((-1 * x552 * x106) + (x551 * x105)) * x107;
	const FLT x557 = x556 + (-1 * x85 * x555);
	const FLT x558 = (-1 * x115 * (x553 + (x554 * x114))) + (x56 * x554);
	const FLT x559 = x558 * x155;
	const FLT x560 = x558 * x113;
	const FLT x561 = (x57 * ((-1 * x58 * x560) + x559)) + (x558 * x121);
	const FLT x562 = (x57 * x561) + (x558 * x154);
	const FLT x563 =
		x83 *
		((x557 * x130) + (x560 * x128) + x555 +
		 (-1 * x127 *
		  ((x76 * ((x558 * x125) +
				   (x57 * (x562 + (x57 * (x561 + (x57 * (x559 + (x558 * x120) + (-1 * x560 * x119))) + (x558 * x165))) +
						   (x558 * x123))) +
				   (x558 * x126) + (x57 * x562))) +
		   (x557 * x112))) +
		 (x81 * x562));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT),
						x108 + (-1 * x131) + (-1 * ((-1 * x108) + x131) * x132));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT),
						x142 + (-1 * x148) + (-1 * ((-1 * x142) + x148) * x132));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT),
						x162 + (-1 * x166) + (-1 * ((-1 * x162) + x166) * x132));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT),
						x183 + (-1 * x190) + (-1 * ((-1 * x183) + x190) * x132));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT),
						x200 + (-1 * x206) + (-1 * ((-1 * x200) + x206) * x132));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT),
						x216 + (-1 * ((-1 * x216) + x218) * x132) + (-1 * x218));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						x265 + (-1 * x267) + (-1 * ((-1 * x265) + x267) * x132));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						(-1 * x297) + x291 + (-1 * ((-1 * x291) + x297) * x132));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x315 + (-1 * x321) + (-1 * ((-1 * x315) + x321) * x132));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						(-1 * x342) + x340 + (-1 * ((-1 * x340) + x342) * x132));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						(-1 * x433) + x431 + (-1 * ((-1 * x431) + x433) * x132));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x476 + (-1 * x478) + (-1 * ((-1 * x476) + x478) * x132));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x515 + (-1 * ((-1 * x515) + x517) * x132) + (-1 * x517));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT),
						x533 + (-1 * x535) + (-1 * ((-1 * x533) + x535) * x132));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT),
						x548 + (-1 * x550) + (-1 * ((-1 * x548) + x550) * x132));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT),
						x556 + (-1 * x563) + (-1 * ((-1 * x556) + x563) * x132));
}

// Full version Jacobian of SurviveKalmanModel_LightMeas_y_gen2 wrt [(*_x0).AccBias[0], (*_x0).AccBias[1],
// (*_x0).AccBias[2], (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1],
// (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2],
// (*_x0).IMUCorrection[3], (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0],
// (*_x0).Pose.Rot[1], (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0],
// (*_x0).Velocity.AxisAngleRot[1], (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1],
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4eeb1c0>]

static inline void SurviveKalmanModel_LightMeas_y_gen2_jac_x0_with_hx(CnMat *Hx, CnMat *hx, const FLT dt,
																	  const SurviveKalmanModel *_x0,
																	  const FLT *sensor_pt, const SurvivePose *lh_p,
																	  const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanModel_LightMeas_y_gen2(dt, _x0, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanModel_LightMeas_y_gen2_jac_x0(Hx, dt, _x0, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanModel_LightMeas_y_gen2 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]
static inline void SurviveKalmanModel_LightMeas_y_gen2_jac_sensor_pt(CnMat *Hx, const FLT dt,
																	 const SurviveKalmanModel *_x0,
																	 const FLT *sensor_pt, const SurvivePose *lh_p,
																	 const BaseStationCal *bsc0) {
	const FLT x0 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x1 = dt * dt;
	const FLT x2 = x1 * ((*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x3 = x1 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x4 = x1 * ((*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x5 = 1e-10 + x2 + x3 + x4;
	const FLT x6 = sqrt(x5);
	const FLT x7 = 0.5 * x6;
	const FLT x8 = sin(x7);
	const FLT x9 = (1. / x5) * (x8 * x8);
	const FLT x10 = cos(x7);
	const FLT x11 = 1. / sqrt((x4 * x9) + (x3 * x9) + (x2 * x9) + (x10 * x10));
	const FLT x12 = (1. / x6) * x8 * dt * x11;
	const FLT x13 = x12 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x14 = x13 * (*_x0).Pose.Rot[0];
	const FLT x15 = x11 * x10;
	const FLT x16 = x15 * (*_x0).Pose.Rot[2];
	const FLT x17 = x12 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x18 = x17 * (*_x0).Pose.Rot[1];
	const FLT x19 = x12 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x20 = x19 * (*_x0).Pose.Rot[3];
	const FLT x21 = x18 + x14 + (-1 * x20) + x16;
	const FLT x22 = (-1 * x19 * (*_x0).Pose.Rot[1]) + (x15 * (*_x0).Pose.Rot[0]) + (-1 * x17 * (*_x0).Pose.Rot[3]) +
					(-1 * x13 * (*_x0).Pose.Rot[2]);
	const FLT x23 = x17 * (*_x0).Pose.Rot[0];
	const FLT x24 = x13 * (*_x0).Pose.Rot[1];
	const FLT x25 = x15 * (*_x0).Pose.Rot[3];
	const FLT x26 = x19 * (*_x0).Pose.Rot[2];
	const FLT x27 = x26 + x25 + x23 + (-1 * x24);
	const FLT x28 = (x21 * sensor_pt[2]) + (-1 * x27 * sensor_pt[1]) + (x22 * sensor_pt[0]);
	const FLT x29 = x17 * (*_x0).Pose.Rot[2];
	const FLT x30 = x13 * (*_x0).Pose.Rot[3];
	const FLT x31 = x15 * (*_x0).Pose.Rot[1];
	const FLT x32 = x19 * (*_x0).Pose.Rot[0];
	const FLT x33 = x32 + x31 + (-1 * x29) + x30;
	const FLT x34 = (-1 * x33 * sensor_pt[2]) + (x27 * sensor_pt[0]) + (x22 * sensor_pt[1]);
	const FLT x35 = sensor_pt[2] + (x0 * (*_x0).Acc[2]) + (2 * ((x34 * x33) + (-1 * x21 * x28))) +
					(dt * (*_x0).Velocity.Pos[2]) + (*_x0).Pose.Pos[2];
	const FLT x36 = (-1 * x21 * sensor_pt[0]) + (x22 * sensor_pt[2]) + (x33 * sensor_pt[1]);
	const FLT x37 = (dt * (*_x0).Velocity.Pos[1]) + sensor_pt[1] + (2 * ((x28 * x27) + (-1 * x33 * x36))) +
					(*_x0).Pose.Pos[1] + (x0 * (*_x0).Acc[1]);
	const FLT x38 = (2 * ((x36 * x21) + (-1 * x34 * x27))) + (*_x0).Pose.Pos[0] + sensor_pt[0] +
					(dt * (*_x0).Velocity.Pos[0]) + (x0 * (*_x0).Acc[0]);
	const FLT x39 = (x35 * (*lh_p).Rot[0]) + (-1 * x38 * (*lh_p).Rot[2]) + (x37 * (*lh_p).Rot[1]);
	const FLT x40 = (-1 * x37 * (*lh_p).Rot[3]) + (x35 * (*lh_p).Rot[2]) + (x38 * (*lh_p).Rot[0]);
	const FLT x41 = x37 + (*lh_p).Pos[1] + (2 * ((x40 * (*lh_p).Rot[3]) + (-1 * x39 * (*lh_p).Rot[1])));
	const FLT x42 = x41 * x41;
	const FLT x43 = (-1 * x35 * (*lh_p).Rot[1]) + (x37 * (*lh_p).Rot[0]) + (x38 * (*lh_p).Rot[3]);
	const FLT x44 = x35 + (2 * ((x43 * (*lh_p).Rot[1]) + (-1 * x40 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x45 = x38 + (2 * ((x39 * (*lh_p).Rot[2]) + (-1 * x43 * (*lh_p).Rot[3]))) + (*lh_p).Pos[0];
	const FLT x46 = x45 * x45;
	const FLT x47 = x46 + (x44 * x44);
	const FLT x48 = x47 + x42;
	const FLT x49 = 0.523598775598299 + (-1 * (*bsc0).tilt);
	const FLT x50 = cos(x49);
	const FLT x51 = 1. / x50;
	const FLT x52 = x51 * (1. / sqrt(x48));
	const FLT x53 = asin(x52 * x41);
	const FLT x54 = 8.0108022e-06 * x53;
	const FLT x55 = -8.0108022e-06 + (-1 * x54);
	const FLT x56 = 0.0028679863 + (x53 * x55);
	const FLT x57 = 5.3685255e-06 + (x53 * x56);
	const FLT x58 = 0.0076069798 + (x53 * x57);
	const FLT x59 = x53 * x58;
	const FLT x60 = -8.0108022e-06 + (-1.60216044e-05 * x53);
	const FLT x61 = x56 + (x60 * x53);
	const FLT x62 = x57 + (x61 * x53);
	const FLT x63 = x58 + (x62 * x53);
	const FLT x64 = (x63 * x53) + x59;
	const FLT x65 = sin(x49);
	const FLT x66 = atan2(-1 * x44, x45);
	const FLT x67 = tan(x49);
	const FLT x68 = x67 * (1. / sqrt(x47));
	const FLT x69 = -1 * x68 * x41;
	const FLT x70 = (-1 * asin(x69)) + (*bsc0).ogeephase + x66;
	const FLT x71 = (sin(x70) * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x72 = x71 * x65;
	const FLT x73 = x50 + (x72 * x64);
	const FLT x74 = 1. / x73;
	const FLT x75 = x53 * x53;
	const FLT x76 = x71 * x75;
	const FLT x77 = x74 * x76;
	const FLT x78 = x69 + (x77 * x58);
	const FLT x79 = 1. / sqrt(1 + (-1 * (x78 * x78)));
	const FLT x80 = (-1 * x18) + x20 + (-1 * x16) + (-1 * x14);
	const FLT x81 = 2 * x21;
	const FLT x82 = 1 + (x80 * x81) + (-2 * (x27 * x27));
	const FLT x83 = 2 * x22;
	const FLT x84 = x83 * x21;
	const FLT x85 = 2 * x33;
	const FLT x86 = (x85 * x27) + (-1 * x84);
	const FLT x87 = x83 * x27;
	const FLT x88 = x87 + (-1 * x80 * x85);
	const FLT x89 = (x88 * (*lh_p).Rot[1]) + (-1 * x82 * (*lh_p).Rot[2]) + (x86 * (*lh_p).Rot[0]);
	const FLT x90 = 2 * (*lh_p).Rot[2];
	const FLT x91 = (x82 * (*lh_p).Rot[3]) + (-1 * x86 * (*lh_p).Rot[1]) + (x88 * (*lh_p).Rot[0]);
	const FLT x92 = 2 * (*lh_p).Rot[3];
	const FLT x93 = x82 + (x89 * x90) + (-1 * x92 * x91);
	const FLT x94 = 2 * x45;
	const FLT x95 = 2 * (*lh_p).Rot[1];
	const FLT x96 = (x82 * (*lh_p).Rot[0]) + (-1 * x88 * (*lh_p).Rot[3]) + (x86 * (*lh_p).Rot[2]);
	const FLT x97 = x86 + (x91 * x95) + (-1 * x90 * x96);
	const FLT x98 = 2 * x44;
	const FLT x99 = (x98 * x97) + (x93 * x94);
	const FLT x100 = 1.0 / 2.0 * x41;
	const FLT x101 = x67 * (1. / (x47 * sqrt(x47))) * x100;
	const FLT x102 = x88 + (x92 * x96) + (-1 * x89 * x95);
	const FLT x103 = (-1 * x68 * x102) + (x99 * x101);
	const FLT x104 = 1. / x47;
	const FLT x105 = 1. / sqrt(1 + (-1 * (x67 * x67) * x42 * x104));
	const FLT x106 = x44 * (1. / x46);
	const FLT x107 = 1. / x45;
	const FLT x108 = x46 * x104;
	const FLT x109 = ((-1 * x97 * x107) + (x93 * x106)) * x108;
	const FLT x110 = cos(x70) * (*bsc0).ogeemag;
	const FLT x111 = x110 * (x109 + (-1 * x103 * x105));
	const FLT x112 = x64 * x65;
	const FLT x113 = 1. / sqrt(1 + (-1 * (1. / (x50 * x50)) * x42 * (1. / x48)));
	const FLT x114 = 2 * x41;
	const FLT x115 = x51 * (1. / (x48 * sqrt(x48))) * x100;
	const FLT x116 = x113 * ((-1 * x115 * (x99 + (x102 * x114))) + (x52 * x102));
	const FLT x117 = x55 * x116;
	const FLT x118 = 2.40324066e-05 * x53;
	const FLT x119 = (x53 * ((-1 * x54 * x116) + x117)) + (x56 * x116);
	const FLT x120 = (x53 * x119) + (x57 * x116);
	const FLT x121 = (1. / (x73 * x73)) * x76 * x58;
	const FLT x122 = x75 * x74 * x58;
	const FLT x123 = 2 * x71 * x74 * x59;
	const FLT x124 =
		x79 *
		(x103 + (x111 * x122) + (x116 * x123) +
		 (-1 * x121 *
		  ((x72 * ((x53 * x120) +
				   (x53 * (x120 + (x53 * (x119 + (x53 * ((x60 * x116) + x117 + (-1 * x118 * x116))) + (x61 * x116))) +
						   (x62 * x116))) +
				   (x58 * x116) + (x63 * x116))) +
		   (x111 * x112))) +
		 (x77 * x120));
	const FLT x125 = cos((-1 * asin(x78)) + (*bsc0).gibpha + x66) * (*bsc0).gibmag;
	const FLT x126 = 2 * ((-1 * x25) + (-1 * x26) + x24 + (-1 * x23));
	const FLT x127 = x83 * x33;
	const FLT x128 = x127 + (-1 * x21 * x126);
	const FLT x129 = 1 + (x27 * x126) + (-2 * (x33 * x33));
	const FLT x130 = (x85 * x21) + (-1 * x87);
	const FLT x131 = (x130 * (*lh_p).Rot[0]) + (x128 * (*lh_p).Rot[2]) + (-1 * x129 * (*lh_p).Rot[3]);
	const FLT x132 = (x129 * (*lh_p).Rot[1]) + (x128 * (*lh_p).Rot[0]) + (-1 * x130 * (*lh_p).Rot[2]);
	const FLT x133 = x129 + (x92 * x131) + (-1 * x95 * x132);
	const FLT x134 = 2 * ((x130 * (*lh_p).Rot[3]) + (-1 * x128 * (*lh_p).Rot[1]) + (x129 * (*lh_p).Rot[0]));
	const FLT x135 = x130 + (x90 * x132) + (-1 * x134 * (*lh_p).Rot[3]);
	const FLT x136 = (x134 * (*lh_p).Rot[1]) + x128 + (-1 * x90 * x131);
	const FLT x137 = (x98 * x136) + (x94 * x135);
	const FLT x138 = (-1 * x115 * (x137 + (x114 * x133))) + (x52 * x133);
	const FLT x139 = x113 * x138;
	const FLT x140 = x57 * x113;
	const FLT x141 = x55 * x139;
	const FLT x142 = (x53 * ((-1 * x54 * x139) + x141)) + (x56 * x139);
	const FLT x143 = (x53 * x142) + (x138 * x140);
	const FLT x144 = (-1 * x68 * x133) + (x101 * x137);
	const FLT x145 = ((-1 * x107 * x136) + (x106 * x135)) * x108;
	const FLT x146 = x145 + (-1 * x105 * x144);
	const FLT x147 = x110 * x122;
	const FLT x148 = x110 * x112;
	const FLT x149 =
		x79 *
		(x144 +
		 (-1 * x121 *
		  ((x72 * ((x58 * x139) + (x63 * x139) +
				   (x53 * (x143 + (x53 * ((x53 * (x141 + (x60 * x139) + (-1 * x118 * x139))) + x142 + (x61 * x139))) +
						   (x62 * x139))) +
				   (x53 * x143))) +
		   (x146 * x148))) +
		 (x146 * x147) + (x123 * x139) + (x77 * x143));
	const FLT x150 = (x81 * x27) + (-1 * x127);
	const FLT x151 = (-1 * x32) + (-1 * x30) + x29 + (-1 * x31);
	const FLT x152 = 1 + (x85 * x151) + (-2 * (x21 * x21));
	const FLT x153 = x84 + (-2 * x27 * x151);
	const FLT x154 = (x153 * (*lh_p).Rot[0]) + (-1 * x150 * (*lh_p).Rot[3]) + (x152 * (*lh_p).Rot[2]);
	const FLT x155 = (x150 * (*lh_p).Rot[1]) + (-1 * x153 * (*lh_p).Rot[2]) + (x152 * (*lh_p).Rot[0]);
	const FLT x156 = x150 + (x92 * x154) + (-1 * x95 * x155);
	const FLT x157 = (x153 * (*lh_p).Rot[3]) + (-1 * x152 * (*lh_p).Rot[1]) + (x150 * (*lh_p).Rot[0]);
	const FLT x158 = x153 + (x90 * x155) + (-1 * x92 * x157);
	const FLT x159 = x152 + (x95 * x157) + (-1 * x90 * x154);
	const FLT x160 = (x98 * x159) + (x94 * x158);
	const FLT x161 = (-1 * x115 * (x160 + (x114 * x156))) + (x52 * x156);
	const FLT x162 = x113 * x161;
	const FLT x163 = x55 * x162;
	const FLT x164 = (x53 * ((-1 * x54 * x162) + x163)) + (x56 * x162);
	const FLT x165 = (x53 * x164) + (x161 * x140);
	const FLT x166 = (-1 * x68 * x156) + (x101 * x160);
	const FLT x167 = ((-1 * x107 * x159) + (x106 * x158)) * x108;
	const FLT x168 = x167 + (-1 * x105 * x166);
	const FLT x169 =
		x79 *
		((-1 * x121 *
		  ((x72 * ((x58 * x162) + (x53 * x165) +
				   (x53 * (x165 + (x53 * (x164 + (x53 * ((x60 * x162) + x163 + (-1 * x118 * x162))) + (x61 * x162))) +
						   (x62 * x162))) +
				   (x63 * x162))) +
		   (x168 * x148))) +
		 (x168 * x147) + (x77 * x165) + x166 + (x123 * x162));
	cnMatrixOptionalSet(Hx, 0, 0, x109 + (-1 * x124) + (-1 * ((-1 * x109) + x124) * x125));
	cnMatrixOptionalSet(Hx, 0, 1, x145 + (-1 * x149) + (-1 * ((-1 * x145) + x149) * x125));
	cnMatrixOptionalSet(Hx, 0, 2, x167 + (-1 * x169) + (-1 * ((-1 * x167) + x169) * x125));
}

// Full version Jacobian of SurviveKalmanModel_LightMeas_y_gen2 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]

static inline void SurviveKalmanModel_LightMeas_y_gen2_jac_sensor_pt_with_hx(CnMat *Hx, CnMat *hx, const FLT dt,
																			 const SurviveKalmanModel *_x0,
																			 const FLT *sensor_pt,
																			 const SurvivePose *lh_p,
																			 const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanModel_LightMeas_y_gen2(dt, _x0, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanModel_LightMeas_y_gen2_jac_sensor_pt(Hx, dt, _x0, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanModel_LightMeas_y_gen2 wrt [(*lh_p).Pos[0], (*lh_p).Pos[1], (*lh_p).Pos[2], (*lh_p).Rot[0],
// (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]
static inline void SurviveKalmanModel_LightMeas_y_gen2_jac_lh_p(CnMat *Hx, const FLT dt, const SurviveKalmanModel *_x0,
																const FLT *sensor_pt, const SurvivePose *lh_p,
																const BaseStationCal *bsc0) {
	const FLT x0 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x1 = x0 * (*_x0).Acc[1];
	const FLT x2 = dt * (*_x0).Velocity.Pos[1];
	const FLT x3 = dt * dt;
	const FLT x4 = x3 * ((*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x5 = x3 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x6 = x3 * ((*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x7 = 1e-10 + x4 + x5 + x6;
	const FLT x8 = sqrt(x7);
	const FLT x9 = 0.5 * x8;
	const FLT x10 = sin(x9);
	const FLT x11 = (1. / x7) * (x10 * x10);
	const FLT x12 = cos(x9);
	const FLT x13 = 1. / sqrt((x6 * x11) + (x5 * x11) + (x4 * x11) + (x12 * x12));
	const FLT x14 = (1. / x8) * dt * x13 * x10;
	const FLT x15 = x14 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x16 = x14 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x17 = x13 * x12;
	const FLT x18 = x14 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x19 = (-1 * x18 * (*_x0).Pose.Rot[1]) + (x17 * (*_x0).Pose.Rot[0]) + (-1 * x15 * (*_x0).Pose.Rot[3]) +
					(-1 * x16 * (*_x0).Pose.Rot[2]);
	const FLT x20 = x14 * (*_x0).Pose.Rot[0];
	const FLT x21 = (x20 * (*_x0).Velocity.AxisAngleRot[0]) + (-1 * x15 * (*_x0).Pose.Rot[2]) +
					(x17 * (*_x0).Pose.Rot[1]) + (x16 * (*_x0).Pose.Rot[3]);
	const FLT x22 = (x20 * (*_x0).Velocity.AxisAngleRot[1]) + (-1 * x18 * (*_x0).Pose.Rot[3]) +
					(x15 * (*_x0).Pose.Rot[1]) + (x17 * (*_x0).Pose.Rot[2]);
	const FLT x23 = (-1 * x22 * sensor_pt[0]) + (x19 * sensor_pt[2]) + (x21 * sensor_pt[1]);
	const FLT x24 = (x18 * (*_x0).Pose.Rot[2]) + (x17 * (*_x0).Pose.Rot[3]) + (x20 * (*_x0).Velocity.AxisAngleRot[2]) +
					(-1 * x16 * (*_x0).Pose.Rot[1]);
	const FLT x25 = (-1 * x24 * sensor_pt[1]) + (x22 * sensor_pt[2]) + (x19 * sensor_pt[0]);
	const FLT x26 = 2 * ((x24 * x25) + (-1 * x23 * x21));
	const FLT x27 = x26 + x2 + sensor_pt[1] + (*_x0).Pose.Pos[1] + x1;
	const FLT x28 = x27 * (*lh_p).Rot[0];
	const FLT x29 = dt * (*_x0).Velocity.Pos[0];
	const FLT x30 = x0 * (*_x0).Acc[0];
	const FLT x31 = (-1 * x21 * sensor_pt[2]) + (x24 * sensor_pt[0]) + (x19 * sensor_pt[1]);
	const FLT x32 = 2 * ((x22 * x23) + (-1 * x31 * x24));
	const FLT x33 = x32 + (*_x0).Pose.Pos[0] + x29 + sensor_pt[0] + x30;
	const FLT x34 = x33 * (*lh_p).Rot[3];
	const FLT x35 = dt * (*_x0).Velocity.Pos[2];
	const FLT x36 = x0 * (*_x0).Acc[2];
	const FLT x37 = 2 * ((x31 * x21) + (-1 * x25 * x22));
	const FLT x38 = x37 + sensor_pt[2] + x36 + x35 + (*_x0).Pose.Pos[2];
	const FLT x39 = x38 * (*lh_p).Rot[1];
	const FLT x40 = (-1 * x39) + x28 + x34;
	const FLT x41 = x38 * (*lh_p).Rot[0];
	const FLT x42 = x27 * (*lh_p).Rot[1];
	const FLT x43 = x33 * (*lh_p).Rot[2];
	const FLT x44 = (-1 * x43) + x41 + x42;
	const FLT x45 = x33 + (2 * ((x44 * (*lh_p).Rot[2]) + (-1 * x40 * (*lh_p).Rot[3]))) + (*lh_p).Pos[0];
	const FLT x46 = x38 * (*lh_p).Rot[2];
	const FLT x47 = x33 * (*lh_p).Rot[0];
	const FLT x48 = x27 * (*lh_p).Rot[3];
	const FLT x49 = (-1 * x48) + x46 + x47;
	const FLT x50 = x27 + (*lh_p).Pos[1] + (2 * ((x49 * (*lh_p).Rot[3]) + (-1 * x44 * (*lh_p).Rot[1])));
	const FLT x51 = x50 * x45;
	const FLT x52 = 0.523598775598299 + (-1 * (*bsc0).tilt);
	const FLT x53 = tan(x52);
	const FLT x54 = x38 + (2 * ((x40 * (*lh_p).Rot[1]) + (-1 * x49 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x55 = x45 * x45;
	const FLT x56 = x55 + (x54 * x54);
	const FLT x57 = x53 * (1. / (x56 * sqrt(x56)));
	const FLT x58 = x51 * x57;
	const FLT x59 = x50 * x50;
	const FLT x60 = 1. / x56;
	const FLT x61 = 1. / sqrt(1 + (-1 * x60 * (x53 * x53) * x59));
	const FLT x62 = x60 * x54;
	const FLT x63 = atan2(-1 * x54, x45);
	const FLT x64 = x53 * (1. / sqrt(x56));
	const FLT x65 = -1 * x64 * x50;
	const FLT x66 = (-1 * asin(x65)) + (*bsc0).ogeephase + x63;
	const FLT x67 = cos(x66) * (*bsc0).ogeemag;
	const FLT x68 = x67 * (x62 + (-1 * x61 * x58));
	const FLT x69 = x56 + x59;
	const FLT x70 = cos(x52);
	const FLT x71 = 1. / x70;
	const FLT x72 = x71 * (1. / sqrt(x69));
	const FLT x73 = asin(x72 * x50);
	const FLT x74 = 8.0108022e-06 * x73;
	const FLT x75 = -8.0108022e-06 + (-1 * x74);
	const FLT x76 = 0.0028679863 + (x73 * x75);
	const FLT x77 = 5.3685255e-06 + (x73 * x76);
	const FLT x78 = 0.0076069798 + (x73 * x77);
	const FLT x79 = x73 * x73;
	const FLT x80 = x73 * x78;
	const FLT x81 = -8.0108022e-06 + (-1.60216044e-05 * x73);
	const FLT x82 = x76 + (x81 * x73);
	const FLT x83 = x77 + (x82 * x73);
	const FLT x84 = x78 + (x83 * x73);
	const FLT x85 = (x84 * x73) + x80;
	const FLT x86 = sin(x52);
	const FLT x87 = (sin(x66) * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x88 = x86 * x87;
	const FLT x89 = x70 + (x88 * x85);
	const FLT x90 = 1. / x89;
	const FLT x91 = x79 * x78 * x90;
	const FLT x92 = 2 * x45;
	const FLT x93 = 1. / sqrt(1 + (-1 * (1. / (x70 * x70)) * (1. / x69) * x59));
	const FLT x94 = x71 * (1. / (x69 * sqrt(x69)));
	const FLT x95 = x50 * x94;
	const FLT x96 = x80 * x87 * x90;
	const FLT x97 = x93 * x96 * x95;
	const FLT x98 = x77 * x93;
	const FLT x99 = x51 * x94;
	const FLT x100 = x93 * x99;
	const FLT x101 = -1 * x75 * x100;
	const FLT x102 = (x73 * ((x74 * x100) + x101)) + (-1 * x76 * x100);
	const FLT x103 = (x73 * x102) + (-1 * x99 * x98);
	const FLT x104 = x87 * x79;
	const FLT x105 = x90 * x104;
	const FLT x106 = x85 * x86;
	const FLT x107 = 2.40324066e-05 * x73;
	const FLT x108 = x81 * x93;
	const FLT x109 = x84 * x93;
	const FLT x110 = (1. / (x89 * x89)) * x78 * x104;
	const FLT x111 = x65 + (x78 * x105);
	const FLT x112 = 1. / sqrt(1 + (-1 * (x111 * x111)));
	const FLT x113 =
		x112 *
		((-1 * x110 *
		  ((x88 * ((-1 * x78 * x100) + (x73 * x103) +
				   (x73 * ((x73 * (x102 + (x73 * ((-1 * x99 * x108) + x101 + (x100 * x107))) + (-1 * x82 * x100))) +
						   x103 + (-1 * x83 * x100))) +
				   (-1 * x99 * x109))) +
		   (x68 * x106))) +
		 (x103 * x105) + (-1 * x92 * x97) + x58 + (x68 * x91));
	const FLT x114 = cos((-1 * asin(x111)) + (*bsc0).gibpha + x63) * (*bsc0).gibmag;
	const FLT x115 = x67 * x91;
	const FLT x116 = x64 * x61;
	const FLT x117 = (-1 * x59 * x94) + x72;
	const FLT x118 = x93 * x117;
	const FLT x119 = x75 * x118;
	const FLT x120 = (x73 * ((-1 * x74 * x118) + x119)) + (x76 * x118);
	const FLT x121 = (x73 * x120) + (x77 * x118);
	const FLT x122 = x67 * x106;
	const FLT x123 = 2 * x96;
	const FLT x124 =
		x112 *
		((x118 * x123) +
		 (-1 * x110 *
		  ((x88 * ((x78 * x118) + (x73 * x121) +
				   (x73 * (x121 + (x73 * (x120 + (x73 * ((x108 * x117) + x119 + (-1 * x107 * x118))) + (x82 * x118))) +
						   (x83 * x118))) +
				   (x84 * x118))) +
		   (x116 * x122))) +
		 (x105 * x121) + (-1 * x64) + (x115 * x116));
	const FLT x125 = x50 * x57;
	const FLT x126 = x54 * x125;
	const FLT x127 = x60 * x45;
	const FLT x128 = -1 * x127;
	const FLT x129 = x128 + (-1 * x61 * x126);
	const FLT x130 = x54 * x95;
	const FLT x131 = x93 * x130;
	const FLT x132 = -1 * x75 * x131;
	const FLT x133 = (x73 * ((x74 * x131) + x132)) + (-1 * x76 * x131);
	const FLT x134 = (x73 * x133) + (-1 * x98 * x130);
	const FLT x135 = 2 * x54;
	const FLT x136 =
		x112 *
		((-1 * x97 * x135) + (x105 * x134) +
		 (-1 * x110 *
		  ((x88 *
			((-1 * x109 * x130) +
			 (x73 * (x134 + (x73 * (x133 + (x73 * ((-1 * x108 * x130) + x132 + (x107 * x131))) + (-1 * x82 * x131))) +
					 (-1 * x83 * x131))) +
			 (-1 * x78 * x131) + (x73 * x134))) +
		   (x122 * x129))) +
		 x126 + (x115 * x129));
	const FLT x137 = 2 * x48;
	const FLT x138 = (2 * x46) + (-1 * x137);
	const FLT x139 = 2 * x43;
	const FLT x140 = (2 * x42) + (-1 * x139);
	const FLT x141 = (x135 * x140) + (x92 * x138);
	const FLT x142 = 1.0 / 2.0 * x125;
	const FLT x143 = 2 * x39;
	const FLT x144 = (2 * x34) + (-1 * x143);
	const FLT x145 = (-1 * x64 * x144) + (x142 * x141);
	const FLT x146 = x54 * (1. / x55);
	const FLT x147 = 1. / x45;
	const FLT x148 = x60 * x55;
	const FLT x149 = ((-1 * x140 * x147) + (x138 * x146)) * x148;
	const FLT x150 = x149 + (-1 * x61 * x145);
	const FLT x151 = 2 * x50;
	const FLT x152 = 1.0 / 2.0 * x95;
	const FLT x153 = (-1 * x152 * (x141 + (x144 * x151))) + (x72 * x144);
	const FLT x154 = x93 * x153;
	const FLT x155 = x75 * x154;
	const FLT x156 = (x73 * ((-1 * x74 * x154) + x155)) + (x76 * x154);
	const FLT x157 = (x73 * x156) + (x77 * x154);
	const FLT x158 =
		x112 *
		(x145 + (x115 * x150) + (x105 * x157) +
		 (-1 * x110 *
		  ((x88 * ((x78 * x154) +
				   (x73 * (x157 + (x73 * (x156 + (x73 * ((x108 * x153) + x155 + (-1 * x107 * x154))) + (x82 * x154))) +
						   (x83 * x154))) +
				   (x84 * x154) + (x73 * x157))) +
		   (x122 * x150))) +
		 (x123 * x154));
	const FLT x159 = 2 * x41;
	const FLT x160 = (-4 * x42) + (-1 * x159) + x139;
	const FLT x161 = (-1 * x36) + (-1 * sensor_pt[2]) + (-1 * x37) + (-1 * (*_x0).Pose.Pos[2]) + (-1 * x35);
	const FLT x162 = 2 * (*lh_p).Rot[3];
	const FLT x163 = 2 * (*lh_p).Rot[2];
	const FLT x164 = (x27 * x163) + (-1 * x161 * x162);
	const FLT x165 = 2 * (*lh_p).Rot[1];
	const FLT x166 = 2 * x28;
	const FLT x167 = x144 + (x161 * x165) + x166;
	const FLT x168 = (x167 * x135) + (x92 * x164);
	const FLT x169 = (-1 * x152 * (x168 + (x160 * x151))) + (x72 * x160);
	const FLT x170 = x93 * x169;
	const FLT x171 = (-1 * x64 * x160) + (x168 * x142);
	const FLT x172 = ((-1 * x167 * x147) + (x164 * x146)) * x148;
	const FLT x173 = x172 + (-1 * x61 * x171);
	const FLT x174 = x75 * x170;
	const FLT x175 = (x73 * ((-1 * x74 * x170) + x174)) + (x76 * x170);
	const FLT x176 = (x73 * x175) + (x77 * x170);
	const FLT x177 =
		x112 *
		((x115 * x173) + x171 + (x105 * x176) + (x123 * x170) +
		 (-1 * x110 *
		  ((x88 * ((x78 * x170) +
				   (x73 * (x176 + (x73 * (x175 + (x73 * ((x108 * x169) + x174 + (-1 * x107 * x170))) + (x82 * x170))) +
						   (x83 * x170))) +
				   (x73 * x176) + (x84 * x170))) +
		   (x122 * x173))));
	const FLT x178 = (-1 * (*_x0).Pose.Pos[0]) + (-1 * x32) + (-1 * x29) + (-1 * x30) + (-1 * sensor_pt[0]);
	const FLT x179 = x140 + x159 + (x163 * x178);
	const FLT x180 = 2 * x47;
	const FLT x181 = (-1 * x180) + (-4 * x46) + x137;
	const FLT x182 = (x181 * x135) + (x92 * x179);
	const FLT x183 = (x38 * x162) + (-1 * x165 * x178);
	const FLT x184 = (-1 * x64 * x183) + (x182 * x142);
	const FLT x185 = ((-1 * x181 * x147) + (x179 * x146)) * x148;
	const FLT x186 = x185 + (-1 * x61 * x184);
	const FLT x187 = (-1 * x152 * (x182 + (x183 * x151))) + (x72 * x183);
	const FLT x188 = x93 * x187;
	const FLT x189 = x75 * x188;
	const FLT x190 = (x73 * ((-1 * x74 * x188) + x189)) + (x76 * x188);
	const FLT x191 = (x73 * x190) + (x77 * x188);
	const FLT x192 =
		x112 *
		(x184 + (x115 * x186) + (x123 * x188) +
		 (-1 * x110 *
		  ((x88 * ((x73 * x191) + (x78 * x188) +
				   (x73 * (x191 + (x73 * (x190 + (x73 * ((x108 * x187) + x189 + (-1 * x107 * x188))) + (x82 * x188))) +
						   (x83 * x188))) +
				   (x84 * x188))) +
		   (x122 * x186))) +
		 (x105 * x191));
	const FLT x193 = (-1 * x2) + (-1 * x26) + (-1 * (*_x0).Pose.Pos[1]) + (-1 * x1) + (-1 * sensor_pt[1]);
	const FLT x194 = x138 + (x162 * x193) + x180;
	const FLT x195 = x143 + (-1 * x166) + (-4 * x34);
	const FLT x196 = (x33 * x165) + (-1 * x163 * x193);
	const FLT x197 = (x196 * x135) + (x92 * x195);
	const FLT x198 = (-1 * x152 * (x197 + (x194 * x151))) + (x72 * x194);
	const FLT x199 = x93 * x198;
	const FLT x200 = x75 * x199;
	const FLT x201 = (x73 * ((-1 * x74 * x199) + x200)) + (x76 * x199);
	const FLT x202 = (x73 * x201) + (x77 * x199);
	const FLT x203 = (-1 * x64 * x194) + (x197 * x142);
	const FLT x204 = ((-1 * x196 * x147) + (x195 * x146)) * x148;
	const FLT x205 = x204 + (-1 * x61 * x203);
	const FLT x206 =
		x112 *
		((x205 * x115) +
		 (-1 * x110 *
		  ((x88 * ((x73 * (x202 + (x73 * (x201 + (x73 * ((x108 * x198) + (-1 * x107 * x199) + x200)) + (x82 * x199))) +
						   (x83 * x199))) +
				   (x84 * x199) + (x78 * x199) + (x73 * x202))) +
		   (x205 * x122))) +
		 (x202 * x105) + x203 + (x123 * x199));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[0]) / sizeof(FLT),
						x62 + (-1 * x113) + (-1 * x114 * ((-1 * x62) + x113)));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[1]) / sizeof(FLT), (-1 * x114 * x124) + (-1 * x124));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[2]) / sizeof(FLT),
						x128 + (-1 * x136) + (-1 * (x127 + x136) * x114));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[0]) / sizeof(FLT),
						x149 + (-1 * x158) + (-1 * ((-1 * x149) + x158) * x114));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[1]) / sizeof(FLT),
						x172 + (-1 * x177) + (-1 * ((-1 * x172) + x177) * x114));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[2]) / sizeof(FLT),
						x185 + (-1 * x192) + (-1 * ((-1 * x185) + x192) * x114));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[3]) / sizeof(FLT),
						x204 + (-1 * x206) + (-1 * ((-1 * x204) + x206) * x114));
}

// Full version Jacobian of SurviveKalmanModel_LightMeas_y_gen2 wrt [(*lh_p).Pos[0], (*lh_p).Pos[1], (*lh_p).Pos[2],
// (*lh_p).Rot[0], (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]

static inline void SurviveKalmanModel_LightMeas_y_gen2_jac_lh_p_with_hx(CnMat *Hx, CnMat *hx, const FLT dt,
																		const SurviveKalmanModel *_x0,
																		const FLT *sensor_pt, const SurvivePose *lh_p,
																		const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanModel_LightMeas_y_gen2(dt, _x0, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanModel_LightMeas_y_gen2_jac_lh_p(Hx, dt, _x0, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanModel_LightMeas_y_gen2 wrt [<cnkalman.codegen.WrapMember object at 0x7f88f4eebf70>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4ed7070>, <cnkalman.codegen.WrapMember object at 0x7f88f4eebfd0>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4ed7130>, <cnkalman.codegen.WrapMember object at 0x7f88f4ed70d0>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4eebd60>, <cnkalman.codegen.WrapMember object at 0x7f88f4eebf10>]
static inline void SurviveKalmanModel_LightMeas_y_gen2_jac_bsc0(CnMat *Hx, const FLT dt, const SurviveKalmanModel *_x0,
																const FLT *sensor_pt, const SurvivePose *lh_p,
																const BaseStationCal *bsc0) {
	const FLT x0 = 0.523598775598299 + (-1 * (*bsc0).tilt);
	const FLT x1 = cos(x0);
	const FLT x2 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x3 = dt * dt;
	const FLT x4 = x3 * ((*_x0).Velocity.AxisAngleRot[1] * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x5 = x3 * ((*_x0).Velocity.AxisAngleRot[2] * (*_x0).Velocity.AxisAngleRot[2]);
	const FLT x6 = x3 * ((*_x0).Velocity.AxisAngleRot[0] * (*_x0).Velocity.AxisAngleRot[0]);
	const FLT x7 = 1e-10 + x4 + x5 + x6;
	const FLT x8 = sqrt(x7);
	const FLT x9 = 0.5 * x8;
	const FLT x10 = sin(x9);
	const FLT x11 = (1. / x7) * (x10 * x10);
	const FLT x12 = cos(x9);
	const FLT x13 = 1. / sqrt((x6 * x11) + (x5 * x11) + (x4 * x11) + (x12 * x12));
	const FLT x14 = (1. / x8) * dt * x13 * x10;
	const FLT x15 = x14 * (*_x0).Velocity.AxisAngleRot[1];
	const FLT x16 = x13 * x12;
	const FLT x17 = x14 * (*_x0).Velocity.AxisAngleRot[2];
	const FLT x18 = x14 * (*_x0).Velocity.AxisAngleRot[0];
	const FLT x19 = (-1 * x18 * (*_x0).Pose.Rot[3]) + (x17 * (*_x0).Pose.Rot[1]) + (x15 * (*_x0).Pose.Rot[0]) +
					(x16 * (*_x0).Pose.Rot[2]);
	const FLT x20 = x14 * (*_x0).Pose.Rot[2];
	const FLT x21 = (x16 * (*_x0).Pose.Rot[0]) + (-1 * x18 * (*_x0).Pose.Rot[1]) + (-1 * x17 * (*_x0).Pose.Rot[3]) +
					(-1 * x20 * (*_x0).Velocity.AxisAngleRot[1]);
	const FLT x22 = (x20 * (*_x0).Velocity.AxisAngleRot[0]) + (x16 * (*_x0).Pose.Rot[3]) + (x17 * (*_x0).Pose.Rot[0]) +
					(-1 * x15 * (*_x0).Pose.Rot[1]);
	const FLT x23 = (x19 * sensor_pt[2]) + (-1 * x22 * sensor_pt[1]) + (x21 * sensor_pt[0]);
	const FLT x24 = (-1 * x20 * (*_x0).Velocity.AxisAngleRot[2]) + (x18 * (*_x0).Pose.Rot[0]) +
					(x16 * (*_x0).Pose.Rot[1]) + (x15 * (*_x0).Pose.Rot[3]);
	const FLT x25 = (x22 * sensor_pt[0]) + (-1 * x24 * sensor_pt[2]) + (x21 * sensor_pt[1]);
	const FLT x26 = sensor_pt[2] + (x2 * (*_x0).Acc[2]) + (dt * (*_x0).Velocity.Pos[2]) +
					(2 * ((x24 * x25) + (-1 * x23 * x19))) + (*_x0).Pose.Pos[2];
	const FLT x27 = (-1 * x19 * sensor_pt[0]) + (x21 * sensor_pt[2]) + (x24 * sensor_pt[1]);
	const FLT x28 = (2 * ((x22 * x23) + (-1 * x24 * x27))) + sensor_pt[1] + (*_x0).Pose.Pos[1] +
					(dt * (*_x0).Velocity.Pos[1]) + (x2 * (*_x0).Acc[1]);
	const FLT x29 = (2 * ((x27 * x19) + (-1 * x25 * x22))) + (*_x0).Pose.Pos[0] + sensor_pt[0] +
					(dt * (*_x0).Velocity.Pos[0]) + (x2 * (*_x0).Acc[0]);
	const FLT x30 = (-1 * x29 * (*lh_p).Rot[2]) + (x26 * (*lh_p).Rot[0]) + (x28 * (*lh_p).Rot[1]);
	const FLT x31 = (-1 * x28 * (*lh_p).Rot[3]) + (x26 * (*lh_p).Rot[2]) + (x29 * (*lh_p).Rot[0]);
	const FLT x32 = x28 + (*lh_p).Pos[1] + (2 * ((x31 * (*lh_p).Rot[3]) + (-1 * x30 * (*lh_p).Rot[1])));
	const FLT x33 = x32 * x32;
	const FLT x34 = (-1 * x26 * (*lh_p).Rot[1]) + (x28 * (*lh_p).Rot[0]) + (x29 * (*lh_p).Rot[3]);
	const FLT x35 = x26 + (2 * ((x34 * (*lh_p).Rot[1]) + (-1 * x31 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x36 = x29 + (2 * ((x30 * (*lh_p).Rot[2]) + (-1 * x34 * (*lh_p).Rot[3]))) + (*lh_p).Pos[0];
	const FLT x37 = (x36 * x36) + (x35 * x35);
	const FLT x38 = x37 + x33;
	const FLT x39 = x32 * (1. / sqrt(x38));
	const FLT x40 = asin((1. / x1) * x39);
	const FLT x41 = 8.0108022e-06 * x40;
	const FLT x42 = -8.0108022e-06 + (-1 * x41);
	const FLT x43 = 0.0028679863 + (x40 * x42);
	const FLT x44 = 5.3685255e-06 + (x40 * x43);
	const FLT x45 = 0.0076069798 + (x40 * x44);
	const FLT x46 = x40 * x40;
	const FLT x47 = sin(x0);
	const FLT x48 = atan2(-1 * x35, x36);
	const FLT x49 = tan(x0);
	const FLT x50 = x32 * (1. / sqrt(x37));
	const FLT x51 = -1 * x50 * x49;
	const FLT x52 = (-1 * asin(x51)) + (*bsc0).ogeephase + x48;
	const FLT x53 = sin(x52);
	const FLT x54 = (x53 * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x55 = x40 * x45;
	const FLT x56 = -8.0108022e-06 + (-1.60216044e-05 * x40);
	const FLT x57 = x43 + (x56 * x40);
	const FLT x58 = x44 + (x57 * x40);
	const FLT x59 = x45 + (x58 * x40);
	const FLT x60 = (x59 * x40) + x55;
	const FLT x61 = x60 * x54;
	const FLT x62 = x61 * x47;
	const FLT x63 = x1 + x62;
	const FLT x64 = 1. / x63;
	const FLT x65 = x64 * x46;
	const FLT x66 = x65 * x45;
	const FLT x67 = (1. / (x63 * x63)) * x45 * x46;
	const FLT x68 = x62 * x67;
	const FLT x69 = x51 + (x66 * x54);
	const FLT x70 = 1. / sqrt(1 + (-1 * (x69 * x69)));
	const FLT x71 = ((-1 * x68) + x66) * x70;
	const FLT x72 = asin(x69) + (-1 * (*bsc0).gibpha) + (-1 * x48);
	const FLT x73 = cos(x72) * (*bsc0).gibmag;
	const FLT x74 = ((-1 * x68 * x53) + (x66 * x53)) * x70;
	const FLT x75 = cos(x52) * (*bsc0).ogeemag;
	const FLT x76 = x75 * x66;
	const FLT x77 = x70 * ((-1 * x75 * x68) + x76);
	const FLT x78 = x49 * x49;
	const FLT x79 = x50 * (1 + x78);
	const FLT x80 = x79 * (1. / sqrt(1 + (-1 * x78 * x33 * (1. / x37))));
	const FLT x81 = x54 * x47;
	const FLT x82 = 1. / (x1 * x1);
	const FLT x83 = x82 * x39 * (1. / sqrt(1 + (-1 * x82 * x33 * (1. / x38))));
	const FLT x84 = x83 * x47;
	const FLT x85 = -1 * x84 * x42;
	const FLT x86 = (x40 * ((x84 * x41) + x85)) + (-1 * x84 * x43);
	const FLT x87 = (x86 * x40) + (-1 * x84 * x44);
	const FLT x88 =
		x70 * ((-1 * x67 * x54 *
				((x81 * ((-1 * x84 * x45) + (-1 * x84 * x59) +
						 (x40 * ((x40 * (x86 + (x40 * ((-1 * x84 * x56) + x85 + (2.40324066e-05 * x84 * x40))) +
										 (-1 * x84 * x57))) +
								 x87 + (-1 * x84 * x58))) +
						 (x87 * x40))) +
				 (-1 * x80 * x75 * x60 * x47) + x47 + (-1 * x1 * x61))) +
			   (x87 * x65 * x54) + x79 + (-1 * x80 * x76) + (-2 * x81 * x83 * x64 * x55));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, curve) / sizeof(FLT), (-1 * x71 * x73) + (-1 * x71));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibmag) / sizeof(FLT), -1 * sin(x72));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibpha) / sizeof(FLT), x73);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, ogeemag) / sizeof(FLT), (-1 * x73 * x74) + (-1 * x74));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, ogeephase) / sizeof(FLT), (-1 * x73 * x77) + (-1 * x77));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, phase) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, tilt) / sizeof(FLT), (-1 * x88 * x73) + (-1 * x88));
}

// Full version Jacobian of SurviveKalmanModel_LightMeas_y_gen2 wrt [<cnkalman.codegen.WrapMember object at
// 0x7f88f4eebf70>, <cnkalman.codegen.WrapMember object at 0x7f88f4ed7070>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4eebfd0>, <cnkalman.codegen.WrapMember object at 0x7f88f4ed7130>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4ed70d0>, <cnkalman.codegen.WrapMember object at 0x7f88f4eebd60>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4eebf10>]

static inline void SurviveKalmanModel_LightMeas_y_gen2_jac_bsc0_with_hx(CnMat *Hx, CnMat *hx, const FLT dt,
																		const SurviveKalmanModel *_x0,
																		const FLT *sensor_pt, const SurvivePose *lh_p,
																		const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanModel_LightMeas_y_gen2(dt, _x0, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanModel_LightMeas_y_gen2_jac_bsc0(Hx, dt, _x0, sensor_pt, lh_p, bsc0);
	}
}
static inline FLT SurviveKalmanErrorModel_LightMeas_x_gen1(const FLT dt, const SurviveKalmanModel *_x0,
														   const SurviveKalmanErrorModel *error_model,
														   const FLT *sensor_pt, const SurvivePose *lh_p,
														   const BaseStationCal *bsc0) {
	const FLT x0 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x1 = 0.5 * (*_x0).Pose.Rot[3];
	const FLT x2 = 0.5 * (*_x0).Pose.Rot[0];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x4 = (-1 * x3 * (*_x0).Pose.Rot[1]) + (x2 * (*error_model).Pose.AxisAngleRot[1]) +
				   (x1 * (*error_model).Pose.AxisAngleRot[0]) + (*_x0).Pose.Rot[2];
	const FLT x5 = dt * dt;
	const FLT x6 = (x0 * x0) * x5;
	const FLT x7 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x8 = x5 * (x7 * x7);
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x10 = x5 * (x9 * x9);
	const FLT x11 = 1e-10 + x10 + x6 + x8;
	const FLT x12 = sqrt(x11);
	const FLT x13 = 0.5 * x12;
	const FLT x14 = sin(x13);
	const FLT x15 = (x14 * x14) * (1. / x11);
	const FLT x16 = cos(x13);
	const FLT x17 = 1. / sqrt((x8 * x15) + (x16 * x16) + (x6 * x15) + (x15 * x10));
	const FLT x18 = dt * x14 * (1. / x12) * x17;
	const FLT x19 = x4 * x18;
	const FLT x20 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x21 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x22 = (x21 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x20 * (*_x0).Pose.Rot[2]) +
					(x2 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x23 = x22 * x18;
	const FLT x24 = (*_x0).Pose.Rot[1] + (x2 * (*error_model).Pose.AxisAngleRot[0]) +
					(-1 * x1 * (*error_model).Pose.AxisAngleRot[1]) + (x3 * (*_x0).Pose.Rot[2]);
	const FLT x25 = x17 * x16;
	const FLT x26 = (-1 * x20 * (*_x0).Pose.Rot[1]) + (-1 * x21 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x1 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x27 = x26 * x18;
	const FLT x28 = (x7 * x27) + (-1 * x0 * x19) + (x24 * x25) + (x9 * x23);
	const FLT x29 = x24 * x18;
	const FLT x30 = (-1 * x7 * x29) + (-1 * x9 * x19) + (-1 * x0 * x23) + (x25 * x26);
	const FLT x31 = (-1 * x7 * x23) + (x4 * x25) + (x0 * x29) + (x9 * x27);
	const FLT x32 = (-1 * x31 * sensor_pt[0]) + (x28 * sensor_pt[1]) + (x30 * sensor_pt[2]);
	const FLT x33 = (x7 * x19) + (x25 * x22) + (-1 * x9 * x29) + (x0 * x27);
	const FLT x34 = (x31 * sensor_pt[2]) + (-1 * x33 * sensor_pt[1]) + (x30 * sensor_pt[0]);
	const FLT x35 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x36 = (*_x0).Pose.Pos[1] + (2 * ((x34 * x33) + (-1 * x32 * x28))) + (*error_model).Pose.Pos[1] +
					sensor_pt[1] + (dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) +
					(x35 * ((*_x0).Acc[1] + (*error_model).Acc[1]));
	const FLT x37 = (x30 * sensor_pt[1]) + (-1 * x28 * sensor_pt[2]) + (x33 * sensor_pt[0]);
	const FLT x38 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (x35 * ((*_x0).Acc[2] + (*error_model).Acc[2])) +
					(2 * ((x37 * x28) + (-1 * x31 * x34))) +
					(dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2])) + (*error_model).Pose.Pos[2];
	const FLT x39 = (x35 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + (*error_model).Pose.Pos[0] +
					(2 * ((x32 * x31) + (-1 * x33 * x37))) + sensor_pt[0] +
					(dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) + (*_x0).Pose.Pos[0];
	const FLT x40 = (-1 * x39 * (*lh_p).Rot[2]) + (x36 * (*lh_p).Rot[1]) + (x38 * (*lh_p).Rot[0]);
	const FLT x41 = (x39 * (*lh_p).Rot[0]) + (-1 * x36 * (*lh_p).Rot[3]) + (x38 * (*lh_p).Rot[2]);
	const FLT x42 = x36 + (*lh_p).Pos[1] + (2 * ((x41 * (*lh_p).Rot[3]) + (-1 * x40 * (*lh_p).Rot[1])));
	const FLT x43 = (x36 * (*lh_p).Rot[0]) + (-1 * x38 * (*lh_p).Rot[1]) + (x39 * (*lh_p).Rot[3]);
	const FLT x44 = x38 + (2 * ((x43 * (*lh_p).Rot[1]) + (-1 * x41 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x45 = -1 * x44;
	const FLT x46 = x39 + (*lh_p).Pos[0] + (2 * ((x40 * (*lh_p).Rot[2]) + (-1 * x43 * (*lh_p).Rot[3])));
	const FLT x47 = (-1 * (*bsc0).phase) + (-1 * asin((1. / sqrt((x46 * x46) + (x44 * x44))) * x42 * (*bsc0).tilt)) +
					(-1 * atan2(x46, x45));
	return x47 + ((atan2(x42, x45) * atan2(x42, x45)) * (*bsc0).curve) +
		   (-1 * cos(1.5707963267949 + x47 + (*bsc0).gibpha) * (*bsc0).gibmag);
}

// Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen1 wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2],
// (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3],
// (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1],
// (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1],
// (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4eea520>]
static inline void SurviveKalmanErrorModel_LightMeas_x_gen1_jac_x0(CnMat *Hx, const FLT dt,
																   const SurviveKalmanModel *_x0,
																   const SurviveKalmanErrorModel *error_model,
																   const FLT *sensor_pt, const SurvivePose *lh_p,
																   const BaseStationCal *bsc0) {
	const FLT x0 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x1 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x2 = 0.5 * (*_x0).Pose.Rot[2];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x4 = (*_x0).Pose.Rot[1] + (x3 * (*_x0).Pose.Rot[0]) + (-1 * x1 * (*_x0).Pose.Rot[3]) +
				   (x2 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x5 = dt * dt;
	const FLT x6 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x7 = x6 * x6;
	const FLT x8 = x5 * x7;
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x10 = x9 * x9;
	const FLT x11 = x5 * x10;
	const FLT x12 = x0 * x0;
	const FLT x13 = x5 * x12;
	const FLT x14 = 1e-10 + x13 + x8 + x11;
	const FLT x15 = sqrt(x14);
	const FLT x16 = 0.5 * x15;
	const FLT x17 = sin(x16);
	const FLT x18 = x17 * x17;
	const FLT x19 = 1. / x14;
	const FLT x20 = x19 * x18;
	const FLT x21 = cos(x16);
	const FLT x22 = (x20 * x11) + (x21 * x21) + (x8 * x20) + (x20 * x13);
	const FLT x23 = 1. / sqrt(x22);
	const FLT x24 = x23 * x17;
	const FLT x25 = x4 * x24;
	const FLT x26 = 1. / x15;
	const FLT x27 = dt * x26;
	const FLT x28 = x25 * x27;
	const FLT x29 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x30 = (-1 * x2 * (*error_model).Pose.AxisAngleRot[1]) + (*_x0).Pose.Rot[0] +
					(-1 * x3 * (*_x0).Pose.Rot[1]) + (-1 * x29 * (*_x0).Pose.Rot[3]);
	const FLT x31 = x24 * x27;
	const FLT x32 = x30 * x31;
	const FLT x33 = (x1 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x2 * (*error_model).Pose.AxisAngleRot[0]) +
					(x29 * (*_x0).Pose.Rot[0]);
	const FLT x34 = x23 * x21;
	const FLT x35 = x34 * x33;
	const FLT x36 =
		(x3 * (*_x0).Pose.Rot[3]) + (-1 * x29 * (*_x0).Pose.Rot[1]) + (x1 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2];
	const FLT x37 = x9 * x31;
	const FLT x38 = x35 + (x36 * x37) + (-1 * x0 * x28) + (x6 * x32);
	const FLT x39 = x0 * x31;
	const FLT x40 = x30 * x34;
	const FLT x41 = x6 * x31;
	const FLT x42 = (-1 * x41 * x33) + (-1 * x36 * x39) + (-1 * x9 * x28) + x40;
	const FLT x43 = x4 * x34;
	const FLT x44 = x43 + (-1 * x41 * x36) + (x9 * x32) + (x33 * x39);
	const FLT x45 = (-1 * x44 * sensor_pt[2]) + (x42 * sensor_pt[1]) + (x38 * sensor_pt[0]);
	const FLT x46 = x34 * x36;
	const FLT x47 = (-1 * x33 * x37) + x46 + (x6 * x28) + (x0 * x32);
	const FLT x48 = (-1 * x47 * sensor_pt[0]) + (x44 * sensor_pt[1]) + (x42 * sensor_pt[2]);
	const FLT x49 = dt * fabs(dt);
	const FLT x50 = 1.0 / 2.0 * x49;
	const FLT x51 = (*_x0).Pose.Pos[0] + (*error_model).Pose.Pos[0] + (2 * ((x47 * x48) + (-1 * x45 * x38))) +
					sensor_pt[0] + (x50 * ((*_x0).Acc[0] + (*error_model).Acc[0])) +
					(dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0]));
	const FLT x52 = (-1 * x38 * sensor_pt[1]) + (x47 * sensor_pt[2]) + (x42 * sensor_pt[0]);
	const FLT x53 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (*error_model).Pose.Pos[2] +
					(2 * ((x44 * x45) + (-1 * x52 * x47))) + (x50 * ((*_x0).Acc[2] + (*error_model).Acc[2])) +
					(dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2]));
	const FLT x54 = (*_x0).Pose.Pos[1] + (2 * ((x52 * x38) + (-1 * x44 * x48))) + (*error_model).Pose.Pos[1] +
					sensor_pt[1] + (dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) +
					(x50 * ((*_x0).Acc[1] + (*error_model).Acc[1]));
	const FLT x55 = (-1 * x54 * (*lh_p).Rot[3]) + (x51 * (*lh_p).Rot[0]) + (x53 * (*lh_p).Rot[2]);
	const FLT x56 = (-1 * x53 * (*lh_p).Rot[1]) + (x54 * (*lh_p).Rot[0]) + (x51 * (*lh_p).Rot[3]);
	const FLT x57 = x53 + (2 * ((x56 * (*lh_p).Rot[1]) + (-1 * x55 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x58 = 1. / x57;
	const FLT x59 = (*lh_p).Rot[3] * (*lh_p).Rot[3];
	const FLT x60 = -1 * x59 * x49;
	const FLT x61 = (*lh_p).Rot[2] * (*lh_p).Rot[2];
	const FLT x62 = (-1 * x61 * x49) + x50;
	const FLT x63 = x62 + x60;
	const FLT x64 = x49 * (*lh_p).Rot[0];
	const FLT x65 = x64 * (*lh_p).Rot[2];
	const FLT x66 = x49 * (*lh_p).Rot[3];
	const FLT x67 = x66 * (*lh_p).Rot[1];
	const FLT x68 = x67 + (-1 * x65);
	const FLT x69 = x57 * x57;
	const FLT x70 = 1. / x69;
	const FLT x71 = (-1 * x51 * (*lh_p).Rot[2]) + (x54 * (*lh_p).Rot[1]) + (x53 * (*lh_p).Rot[0]);
	const FLT x72 = x51 + (*lh_p).Pos[0] + (2 * ((x71 * (*lh_p).Rot[2]) + (-1 * x56 * (*lh_p).Rot[3])));
	const FLT x73 = x70 * x72;
	const FLT x74 = x69 + (x72 * x72);
	const FLT x75 = 1. / x74;
	const FLT x76 = x75 * x69;
	const FLT x77 = x54 + (*lh_p).Pos[1] + (2 * ((x55 * (*lh_p).Rot[3]) + (-1 * x71 * (*lh_p).Rot[1])));
	const FLT x78 = x77 * x77;
	const FLT x79 = 1. / sqrt(1 + (-1 * x78 * x75 * ((*bsc0).tilt * (*bsc0).tilt)));
	const FLT x80 = 2 * x72;
	const FLT x81 = 2 * x57;
	const FLT x82 = 1.0 / 2.0 * (1. / (x74 * sqrt(x74))) * x77 * (*bsc0).tilt;
	const FLT x83 = x49 * (*lh_p).Rot[1] * (*lh_p).Rot[2];
	const FLT x84 = x66 * (*lh_p).Rot[0];
	const FLT x85 = x84 + x83;
	const FLT x86 = (1. / sqrt(x74)) * (*bsc0).tilt;
	const FLT x87 = (-1 * x79 * ((x85 * x86) + (-1 * ((x81 * x68) + (x80 * x63)) * x82))) +
					(-1 * ((x73 * x68) + (-1 * x63 * x58)) * x76);
	const FLT x88 = -1 * x57;
	const FLT x89 = (*bsc0).gibmag * sin(1.5707963267949 + (-1 * asin(x86 * x77)) + (-1 * (*bsc0).phase) +
										 (*bsc0).gibpha + (-1 * atan2(x72, x88)));
	const FLT x90 = x70 * x77;
	const FLT x91 = 2 * (1. / (x69 + x78)) * x69 * atan2(x77, x88) * (*bsc0).curve;
	const FLT x92 = x83 + (-1 * x84);
	const FLT x93 = x66 * (*lh_p).Rot[2];
	const FLT x94 = x64 * (*lh_p).Rot[1];
	const FLT x95 = x94 + x93;
	const FLT x96 = (*lh_p).Rot[1] * (*lh_p).Rot[1];
	const FLT x97 = -1 * x96 * x49;
	const FLT x98 = x97 + x60 + x50;
	const FLT x99 = (-1 * x79 * ((x86 * x98) + (-1 * ((x81 * x95) + (x80 * x92)) * x82))) +
					(-1 * ((x73 * x95) + (-1 * x58 * x92)) * x76);
	const FLT x100 = x65 + x67;
	const FLT x101 = x62 + x97;
	const FLT x102 = x93 + (-1 * x94);
	const FLT x103 = (-1 * x79 * ((x86 * x102) + (-1 * ((x81 * x101) + (x80 * x100)) * x82))) +
					 (-1 * ((x73 * x101) + (-1 * x58 * x100)) * x76);
	const FLT x104 = 2 * x61;
	const FLT x105 = -1 * x104;
	const FLT x106 = 2 * x59;
	const FLT x107 = 1 + (-1 * x106);
	const FLT x108 = x107 + x105;
	const FLT x109 = 2 * (*lh_p).Rot[0];
	const FLT x110 = x109 * (*lh_p).Rot[2];
	const FLT x111 = 2 * (*lh_p).Rot[3];
	const FLT x112 = x111 * (*lh_p).Rot[1];
	const FLT x113 = x112 + (-1 * x110);
	const FLT x114 = 2 * (*lh_p).Rot[2];
	const FLT x115 = x114 * (*lh_p).Rot[1];
	const FLT x116 = x111 * (*lh_p).Rot[0];
	const FLT x117 = x116 + x115;
	const FLT x118 = (-1 * x79 * ((x86 * x117) + (-1 * ((x81 * x113) + (x80 * x108)) * x82))) +
					 (-1 * ((x73 * x113) + (-1 * x58 * x108)) * x76);
	const FLT x119 = x115 + (-1 * x116);
	const FLT x120 = x111 * (*lh_p).Rot[2];
	const FLT x121 = x109 * (*lh_p).Rot[1];
	const FLT x122 = x121 + x120;
	const FLT x123 = 2 * x96;
	const FLT x124 = -1 * x123;
	const FLT x125 = x107 + x124;
	const FLT x126 = (-1 * x79 * ((x86 * x125) + (-1 * ((x81 * x122) + (x80 * x119)) * x82))) +
					 (-1 * ((x73 * x122) + (-1 * x58 * x119)) * x76);
	const FLT x127 = x110 + x112;
	const FLT x128 = 1 + x124 + x105;
	const FLT x129 = x120 + (-1 * x121);
	const FLT x130 = (-1 * x79 * ((x86 * x129) + (-1 * ((x81 * x128) + (x80 * x127)) * x82))) +
					 (-1 * ((x73 * x128) + (-1 * x58 * x127)) * x76);
	const FLT x131 = x1 * x34;
	const FLT x132 = x3 * x41;
	const FLT x133 = -1 * x37 * x29;
	const FLT x134 = x39 + x133;
	const FLT x135 = x134 + x131 + x132;
	const FLT x136 = 2 * x52;
	const FLT x137 = x3 * x37;
	const FLT x138 = -1 * x137;
	const FLT x139 = x1 * x39;
	const FLT x140 = -1 * x139;
	const FLT x141 = x41 * x29;
	const FLT x142 = (-1 * x141) + x34;
	const FLT x143 = x142 + x138 + x140;
	const FLT x144 = x39 * x29;
	const FLT x145 = x3 * x34;
	const FLT x146 = -1 * x1 * x41;
	const FLT x147 = x37 + x146;
	const FLT x148 = x147 + x144 + x145;
	const FLT x149 = x34 * x29;
	const FLT x150 = x1 * x37;
	const FLT x151 = -1 * x3 * x39;
	const FLT x152 = x151 + x41;
	const FLT x153 = x152 + x149 + x150;
	const FLT x154 = (x153 * sensor_pt[0]) + (x143 * sensor_pt[1]) + (-1 * x148 * sensor_pt[2]);
	const FLT x155 = 2 * x44;
	const FLT x156 = (x143 * sensor_pt[0]) + (-1 * x153 * sensor_pt[1]) + (x135 * sensor_pt[2]);
	const FLT x157 = 2 * x47;
	const FLT x158 = 2 * x45;
	const FLT x159 = (x148 * x158) + (-1 * x156 * x157) + (-1 * x135 * x136) + (x154 * x155);
	const FLT x160 = (-1 * x135 * sensor_pt[0]) + (x143 * sensor_pt[2]) + (x148 * sensor_pt[1]);
	const FLT x161 = 2 * x48;
	const FLT x162 = 2 * x38;
	const FLT x163 = (x136 * x153) + (x162 * x156) + (-1 * x160 * x155) + (-1 * x161 * x148);
	const FLT x164 = (x160 * x157) + (x161 * x135) + (-1 * x153 * x158) + (-1 * x162 * x154);
	const FLT x165 = 2 * ((-1 * x159 * (*lh_p).Rot[1]) + (x164 * (*lh_p).Rot[3]) + (x163 * (*lh_p).Rot[0]));
	const FLT x166 = 2 * ((x159 * (*lh_p).Rot[0]) + (-1 * x164 * (*lh_p).Rot[2]) + (x163 * (*lh_p).Rot[1]));
	const FLT x167 = (-1 * x165 * (*lh_p).Rot[3]) + x164 + (x166 * (*lh_p).Rot[2]);
	const FLT x168 = 2 * ((x159 * (*lh_p).Rot[2]) + (-1 * x163 * (*lh_p).Rot[3]) + (x164 * (*lh_p).Rot[0]));
	const FLT x169 = x159 + (x165 * (*lh_p).Rot[1]) + (-1 * x168 * (*lh_p).Rot[2]);
	const FLT x170 = x163 + (-1 * x166 * (*lh_p).Rot[1]) + (x168 * (*lh_p).Rot[3]);
	const FLT x171 = (-1 * x79 * ((x86 * x170) + (-1 * ((x81 * x169) + (x80 * x167)) * x82))) +
					 (-1 * ((x73 * x169) + (-1 * x58 * x167)) * x76);
	const FLT x172 = -1 * x149;
	const FLT x173 = -1 * x150;
	const FLT x174 = x152 + x172 + x173;
	const FLT x175 = -1 * x132;
	const FLT x176 = (-1 * x39) + x133;
	const FLT x177 = x175 + x176 + x131;
	const FLT x178 = -1 * x145;
	const FLT x179 = x146 + (-1 * x37);
	const FLT x180 = x179 + x144 + x178;
	const FLT x181 = (x180 * sensor_pt[0]) + (-1 * x177 * sensor_pt[1]) + (x174 * sensor_pt[2]);
	const FLT x182 = x141 + x34;
	const FLT x183 = x182 + x138 + x139;
	const FLT x184 = (-1 * x183 * sensor_pt[2]) + (x177 * sensor_pt[0]) + (x180 * sensor_pt[1]);
	const FLT x185 = (x184 * x155) + (x183 * x158) + (-1 * x174 * x136) + (-1 * x181 * x157);
	const FLT x186 = (x180 * sensor_pt[2]) + (-1 * x174 * sensor_pt[0]) + (x183 * sensor_pt[1]);
	const FLT x187 = (x162 * x181) + (-1 * x186 * x155) + (-1 * x161 * x183) + (x177 * x136);
	const FLT x188 = (-1 * x162 * x184) + (x186 * x157) + (-1 * x177 * x158) + (x161 * x174);
	const FLT x189 = (x188 * (*lh_p).Rot[3]) + (-1 * x185 * (*lh_p).Rot[1]) + (x187 * (*lh_p).Rot[0]);
	const FLT x190 = (-1 * x188 * (*lh_p).Rot[2]) + (x185 * (*lh_p).Rot[0]) + (x187 * (*lh_p).Rot[1]);
	const FLT x191 = x188 + (-1 * x111 * x189) + (x114 * x190);
	const FLT x192 = (-1 * x187 * (*lh_p).Rot[3]) + (x185 * (*lh_p).Rot[2]) + (x188 * (*lh_p).Rot[0]);
	const FLT x193 = 2 * (*lh_p).Rot[1];
	const FLT x194 = (-1 * x114 * x192) + x185 + (x189 * x193);
	const FLT x195 = x187 + (-1 * x190 * x193) + (x111 * x192);
	const FLT x196 = (-1 * x79 * ((x86 * x195) + (-1 * ((x81 * x194) + (x80 * x191)) * x82))) +
					 (-1 * ((x73 * x194) + (-1 * x58 * x191)) * x76);
	const FLT x197 = x182 + x137 + x140;
	const FLT x198 = -1 * x144;
	const FLT x199 = x147 + x178 + x198;
	const FLT x200 = -1 * x131;
	const FLT x201 = x176 + x200 + x132;
	const FLT x202 = (x201 * sensor_pt[0]) + (x197 * sensor_pt[2]) + (-1 * x199 * sensor_pt[1]);
	const FLT x203 = (-1 * x41) + x151;
	const FLT x204 = x203 + x173 + x149;
	const FLT x205 = (x199 * sensor_pt[0]) + (x201 * sensor_pt[1]) + (-1 * x204 * sensor_pt[2]);
	const FLT x206 = (x204 * x158) + (-1 * x197 * x136) + (x205 * x155) + (-1 * x202 * x157);
	const FLT x207 = (-1 * x197 * sensor_pt[0]) + (x201 * sensor_pt[2]) + (x204 * sensor_pt[1]);
	const FLT x208 = (-1 * x207 * x155) + (x202 * x162) + (-1 * x204 * x161) + (x199 * x136);
	const FLT x209 = (x207 * x157) + (x161 * x197) + (-1 * x199 * x158) + (-1 * x205 * x162);
	const FLT x210 = (x209 * (*lh_p).Rot[3]) + (-1 * x206 * (*lh_p).Rot[1]) + (x208 * (*lh_p).Rot[0]);
	const FLT x211 = (x206 * (*lh_p).Rot[0]) + (-1 * x209 * (*lh_p).Rot[2]) + (x208 * (*lh_p).Rot[1]);
	const FLT x212 = (-1 * x210 * x111) + x209 + (x211 * x114);
	const FLT x213 = (-1 * x208 * (*lh_p).Rot[3]) + (x206 * (*lh_p).Rot[2]) + (x209 * (*lh_p).Rot[0]);
	const FLT x214 = x206 + (-1 * x213 * x114) + (x210 * x193);
	const FLT x215 = x208 + (-1 * x211 * x193) + (x213 * x111);
	const FLT x216 = (-1 * x79 * ((x86 * x215) + (-1 * ((x81 * x214) + (x80 * x212)) * x82))) +
					 (-1 * ((x73 * x214) + (-1 * x58 * x212)) * x76);
	const FLT x217 = x179 + x145 + x198;
	const FLT x218 = x142 + x139 + x137;
	const FLT x219 = x172 + x203 + x150;
	const FLT x220 = (x219 * sensor_pt[0]) + (-1 * x218 * sensor_pt[1]) + (x217 * sensor_pt[2]);
	const FLT x221 = x134 + x200 + x175;
	const FLT x222 = (-1 * x221 * sensor_pt[2]) + (x218 * sensor_pt[0]) + (x219 * sensor_pt[1]);
	const FLT x223 = (x222 * x155) + (-1 * x217 * x136) + (x221 * x158) + (-1 * x220 * x157);
	const FLT x224 = (x219 * sensor_pt[2]) + (-1 * x217 * sensor_pt[0]) + (x221 * sensor_pt[1]);
	const FLT x225 = (x220 * x162) + (x218 * x136) + (-1 * x221 * x161) + (-1 * x224 * x155);
	const FLT x226 = (x217 * x161) + (-1 * x222 * x162) + (-1 * x218 * x158) + (x224 * x157);
	const FLT x227 = (-1 * x223 * (*lh_p).Rot[1]) + (x226 * (*lh_p).Rot[3]) + (x225 * (*lh_p).Rot[0]);
	const FLT x228 = (x223 * (*lh_p).Rot[0]) + (x225 * (*lh_p).Rot[1]) + (-1 * x226 * (*lh_p).Rot[2]);
	const FLT x229 = x226 + (-1 * x227 * x111) + (x228 * x114);
	const FLT x230 = (x223 * (*lh_p).Rot[2]) + (-1 * x225 * (*lh_p).Rot[3]) + (x226 * (*lh_p).Rot[0]);
	const FLT x231 = (-1 * x230 * x114) + x223 + (x227 * x193);
	const FLT x232 = x225 + (x230 * x111) + (-1 * x228 * x193);
	const FLT x233 = (-1 * x79 * ((x86 * x232) + (-1 * ((x81 * x231) + (x80 * x229)) * x82))) +
					 (-1 * ((x73 * x231) + (-1 * x58 * x229)) * x76);
	const FLT x234 = x31 * x33;
	const FLT x235 = -1 * x234;
	const FLT x236 = dt * dt * dt;
	const FLT x237 = 0.5 * x19 * x236;
	const FLT x238 = x10 * x237;
	const FLT x239 = x9 * x9 * x9;
	const FLT x240 = dt * dt * dt * dt;
	const FLT x241 = 1. / (x14 * sqrt(x14));
	const FLT x242 = 1.0 * x21 * x17;
	const FLT x243 = x241 * x242;
	const FLT x244 = x240 * x243;
	const FLT x245 = x5 * x9;
	const FLT x246 = 2 * x20;
	const FLT x247 = 2 * (1. / (x14 * x14)) * x18;
	const FLT x248 = x240 * x247;
	const FLT x249 = x9 * x248;
	const FLT x250 = x9 * x244;
	const FLT x251 = x26 * x242;
	const FLT x252 = (-1 * x251 * x245) + (x12 * x250) + (x245 * x246) + (x7 * x250) + (x239 * x244) +
					 (-1 * x12 * x249) + (-1 * x239 * x248) + (-1 * x7 * x249);
	const FLT x253 = 1.0 / 2.0 * (1. / (x22 * sqrt(x22)));
	const FLT x254 = x27 * x17 * x253;
	const FLT x255 = x252 * x254;
	const FLT x256 = x0 * x30;
	const FLT x257 = x9 * x33;
	const FLT x258 = x236 * x241;
	const FLT x259 = x25 * x258;
	const FLT x260 = x6 * x9;
	const FLT x261 = x260 * x259;
	const FLT x262 = x21 * x253;
	const FLT x263 = x262 * x252;
	const FLT x264 = x43 * x237;
	const FLT x265 = x264 * x260;
	const FLT x266 = x4 * x255;
	const FLT x267 = x36 * x24;
	const FLT x268 = 0.5 * x26;
	const FLT x269 = x268 * x267;
	const FLT x270 = x10 * x258;
	const FLT x271 = x33 * x24;
	const FLT x272 = x0 * x9;
	const FLT x273 = x40 * x237;
	const FLT x274 = x30 * x24;
	const FLT x275 = x274 * x258;
	const FLT x276 = (-1 * x275 * x272) + (x273 * x272);
	const FLT x277 = x276 + (x271 * x270) + (-1 * x6 * x266) + x265 + (-1 * x35 * x238) + (-1 * x256 * x255) + x235 +
					 (-1 * x269 * x245) + (-1 * x36 * x263) + (x255 * x257) + (-1 * x261);
	const FLT x278 = x0 * x33;
	const FLT x279 = x25 * x268;
	const FLT x280 = x267 * x258;
	const FLT x281 = x260 * x280;
	const FLT x282 = x36 * x254;
	const FLT x283 = x6 * x282;
	const FLT x284 = x271 * x258;
	const FLT x285 = x272 * x284;
	const FLT x286 = x35 * x237;
	const FLT x287 = x272 * x286;
	const FLT x288 = x46 * x237;
	const FLT x289 = x260 * x288;
	const FLT x290 = x30 * x255;
	const FLT x291 = (x40 * x238) + (-1 * x278 * x255) + x281 + (x283 * x252) + (-1 * x279 * x245) + (-1 * x9 * x290) +
					 (-1 * x289) + x287 + (-1 * x274 * x270) + (-1 * x4 * x263) + (-1 * x285) + x32;
	const FLT x292 = -1 * x28;
	const FLT x293 = x272 * x280;
	const FLT x294 = x0 * x282;
	const FLT x295 = x272 * x288;
	const FLT x296 = x274 * x268;
	const FLT x297 = (x260 * x284) + (-1 * x260 * x286);
	const FLT x298 = x297 + (x25 * x270) + (x6 * x33 * x255) + (-1 * x43 * x238) + (x9 * x266) + (-1 * x295) +
					 (-1 * x30 * x263) + x293 + x292 + (-1 * x296 * x245) + (x294 * x252);
	const FLT x299 = x31 * x36;
	const FLT x300 = x271 * x268;
	const FLT x301 = x9 * x282;
	const FLT x302 = (x272 * x259) + (-1 * x272 * x264);
	const FLT x303 = (x273 * x260) + (-1 * x275 * x260);
	const FLT x304 = x302 + (-1 * x6 * x290) + x303 + (-1 * x270 * x267) + (-1 * x252 * x301) + x299 +
					 (-1 * x245 * x300) + (x0 * x266) + (x46 * x238) + (-1 * x33 * x263);
	const FLT x305 = (x304 * sensor_pt[0]) + (-1 * x291 * sensor_pt[2]) + (x298 * sensor_pt[1]);
	const FLT x306 = (x298 * sensor_pt[0]) + (-1 * x304 * sensor_pt[1]) + (x277 * sensor_pt[2]);
	const FLT x307 = (-1 * x306 * x157) + (x305 * x155) + (-1 * x277 * x136) + (x291 * x158);
	const FLT x308 = (x298 * sensor_pt[2]) + (-1 * x277 * sensor_pt[0]) + (x291 * sensor_pt[1]);
	const FLT x309 = (-1 * x308 * x155) + (-1 * x291 * x161) + (x304 * x136) + (x306 * x162);
	const FLT x310 = (x277 * x161) + (-1 * x305 * x162) + (-1 * x304 * x158) + (x308 * x157);
	const FLT x311 = 2 * ((x310 * (*lh_p).Rot[3]) + (-1 * x307 * (*lh_p).Rot[1]) + (x309 * (*lh_p).Rot[0]));
	const FLT x312 = 2 * ((x307 * (*lh_p).Rot[0]) + (-1 * x310 * (*lh_p).Rot[2]) + (x309 * (*lh_p).Rot[1]));
	const FLT x313 = x310 + (-1 * x311 * (*lh_p).Rot[3]) + (x312 * (*lh_p).Rot[2]);
	const FLT x314 = 2 * ((x307 * (*lh_p).Rot[2]) + (-1 * x309 * (*lh_p).Rot[3]) + (x310 * (*lh_p).Rot[0]));
	const FLT x315 = x307 + (-1 * x314 * (*lh_p).Rot[2]) + (x311 * (*lh_p).Rot[1]);
	const FLT x316 = x309 + (-1 * x312 * (*lh_p).Rot[1]) + (x314 * (*lh_p).Rot[3]);
	const FLT x317 = (-1 * x79 * ((x86 * x316) + (-1 * ((x81 * x315) + (x80 * x313)) * x82))) +
					 (-1 * ((x73 * x315) + (-1 * x58 * x313)) * x76);
	const FLT x318 = x12 * x258;
	const FLT x319 = x10 * x244;
	const FLT x320 = x10 * x248;
	const FLT x321 = x5 * x251;
	const FLT x322 = x0 * x7;
	const FLT x323 = x0 * x0 * x0;
	const FLT x324 = x5 * x246;
	const FLT x325 = (x0 * x324) + (x244 * x323) + (-1 * x0 * x320) + (x0 * x319) + (-1 * x248 * x323) +
					 (-1 * x0 * x321) + (-1 * x248 * x322) + (x244 * x322);
	const FLT x326 = x262 * x325;
	const FLT x327 = x254 * x325;
	const FLT x328 = x5 * x279;
	const FLT x329 = x9 * x30;
	const FLT x330 = x0 * x6;
	const FLT x331 = (-1 * x288 * x330) + (x280 * x330);
	const FLT x332 = (-1 * x327 * x329) + x276 + (-1 * x0 * x328) + (x283 * x325) + x331 + (-1 * x4 * x326) + x234 +
					 (-1 * x271 * x318) + (-1 * x278 * x327) + (x12 * x286);
	const FLT x333 = x6 * x327;
	const FLT x334 = x264 * x330;
	const FLT x335 = x259 * x330;
	const FLT x336 = x0 * x5;
	const FLT x337 = (-1 * x4 * x333) + (-1 * x274 * x318) + (-1 * x256 * x327) + (-1 * x36 * x326) +
					 (-1 * x269 * x336) + (-1 * x287) + (-1 * x335) + (x12 * x273) + (x257 * x327) + x32 + x334 + x285;
	const FLT x338 = x284 * x330;
	const FLT x339 = x286 * x330;
	const FLT x340 = -1 * x299;
	const FLT x341 = x4 * x327;
	const FLT x342 = x302 + (x9 * x341) + (-1 * x30 * x326) + (-1 * x296 * x336) + (x267 * x318) + x338 + (x33 * x333) +
					 (-1 * x12 * x288) + (x294 * x325) + x340 + (-1 * x339);
	const FLT x343 = (x342 * sensor_pt[2]) + (-1 * x337 * sensor_pt[0]) + (x332 * sensor_pt[1]);
	const FLT x344 = x5 * x300;
	const FLT x345 = (x273 * x330) + (-1 * x275 * x330);
	const FLT x346 = (-1 * x293) + (-1 * x301 * x325) + (-1 * x0 * x344) + (-1 * x33 * x326) + (-1 * x12 * x264) +
					 x295 + x345 + (-1 * x30 * x333) + x292 + (x0 * x341) + (x12 * x259);
	const FLT x347 = (x342 * sensor_pt[0]) + (x337 * sensor_pt[2]) + (-1 * x346 * sensor_pt[1]);
	const FLT x348 = (x347 * x162) + (x346 * x136) + (-1 * x332 * x161) + (-1 * x343 * x155);
	const FLT x349 = (x346 * sensor_pt[0]) + (-1 * x332 * sensor_pt[2]) + (x342 * sensor_pt[1]);
	const FLT x350 = (x349 * x155) + (x332 * x158) + (-1 * x337 * x136) + (-1 * x347 * x157);
	const FLT x351 = (x343 * x157) + (-1 * x349 * x162) + (-1 * x346 * x158) + (x337 * x161);
	const FLT x352 = (x351 * (*lh_p).Rot[3]) + (x348 * (*lh_p).Rot[0]) + (-1 * x350 * (*lh_p).Rot[1]);
	const FLT x353 = (-1 * x351 * (*lh_p).Rot[2]) + (x350 * (*lh_p).Rot[0]) + (x348 * (*lh_p).Rot[1]);
	const FLT x354 = x351 + (-1 * x352 * x111) + (x353 * x114);
	const FLT x355 = (-1 * x348 * (*lh_p).Rot[3]) + (x350 * (*lh_p).Rot[2]) + (x351 * (*lh_p).Rot[0]);
	const FLT x356 = (-1 * x355 * x114) + x350 + (x352 * x193);
	const FLT x357 = x348 + (x355 * x111) + (-1 * x353 * x193);
	const FLT x358 = (-1 * x79 * ((x86 * x357) + (-1 * ((x81 * x356) + (x80 * x354)) * x82))) +
					 (-1 * ((x73 * x356) + (-1 * x58 * x354)) * x76);
	const FLT x359 = (x6 * x6 * x6) * x240;
	const FLT x360 = x6 * x12;
	const FLT x361 = (-1 * x247 * x359) + (x243 * x359) + (-1 * x6 * x321) + (x6 * x324) + (x6 * x319) +
					 (-1 * x248 * x360) + (x244 * x360) + (-1 * x6 * x320);
	const FLT x362 = x254 * x361;
	const FLT x363 = x33 * x362;
	const FLT x364 = x7 * x258;
	const FLT x365 = x262 * x361;
	const FLT x366 = x303 + (-1 * x4 * x365) + (x267 * x364) + (-1 * x0 * x363) + x339 + x340 + (-1 * x362 * x329) +
					 (-1 * x7 * x288) + (-1 * x338) + (-1 * x6 * x328) + (x283 * x361);
	const FLT x367 = x4 * x362;
	const FLT x368 = x6 * x362;
	const FLT x369 = (-1 * x6 * x344) + (-1 * x33 * x365) + (x7 * x273) + (-1 * x274 * x364) + x289 +
					 (-1 * x30 * x368) + (x0 * x367) + (-1 * x281) + x32 + (-1 * x334) + (-1 * x361 * x301) + x335;
	const FLT x370 = x6 * x5;
	const FLT x371 = (x257 * x362) + (-1 * x36 * x365) + (-1 * x269 * x370) + (-1 * x4 * x368) + x345 +
					 (-1 * x256 * x362) + x28 + x297 + (x7 * x264) + (-1 * x7 * x259);
	const FLT x372 = (x9 * x367) + x261 + (x271 * x364) + (-1 * x7 * x286) + x331 + x235 + (-1 * x296 * x370) +
					 (x294 * x361) + (x6 * x363) + (-1 * x265) + (-1 * x30 * x365);
	const FLT x373 = (-1 * x369 * sensor_pt[1]) + (x372 * sensor_pt[0]) + (x371 * sensor_pt[2]);
	const FLT x374 = (x372 * sensor_pt[2]) + (-1 * x371 * sensor_pt[0]) + (x366 * sensor_pt[1]);
	const FLT x375 = (-1 * x374 * x155) + (x373 * x162) + (-1 * x366 * x161) + (x369 * x136);
	const FLT x376 = (x369 * sensor_pt[0]) + (-1 * x366 * sensor_pt[2]) + (x372 * sensor_pt[1]);
	const FLT x377 = (x376 * x155) + (x366 * x158) + (-1 * x371 * x136) + (-1 * x373 * x157);
	const FLT x378 = (-1 * x376 * x162) + (x371 * x161) + (-1 * x369 * x158) + (x374 * x157);
	const FLT x379 = (x378 * (*lh_p).Rot[3]) + (x375 * (*lh_p).Rot[0]) + (-1 * x377 * (*lh_p).Rot[1]);
	const FLT x380 = (x377 * (*lh_p).Rot[2]) + (-1 * x375 * (*lh_p).Rot[3]) + (x378 * (*lh_p).Rot[0]);
	const FLT x381 = x377 + (x379 * x193) + (-1 * x380 * x114);
	const FLT x382 = (x377 * (*lh_p).Rot[0]) + (-1 * x378 * (*lh_p).Rot[2]) + (x375 * (*lh_p).Rot[1]);
	const FLT x383 = x375 + (-1 * x382 * x193) + (x380 * x111);
	const FLT x384 = x378 + (-1 * x379 * x111) + (x382 * x114);
	const FLT x385 = (-1 * x79 * ((x86 * x383) + (-1 * ((x81 * x381) + (x80 * x384)) * x82))) +
					 (-1 * ((x73 * x381) + (-1 * x58 * x384)) * x76);
	const FLT x386 = dt * x110;
	const FLT x387 = dt * x112;
	const FLT x388 = x387 + (-1 * x386);
	const FLT x389 = dt * x115;
	const FLT x390 = dt * x116;
	const FLT x391 = x390 + x389;
	const FLT x392 = -1 * dt * x104;
	const FLT x393 = dt + (-1 * dt * x106);
	const FLT x394 = x393 + x392;
	const FLT x395 = (-1 * x79 * ((x86 * x391) + (-1 * ((x81 * x388) + (x80 * x394)) * x82))) +
					 (-1 * ((x73 * x388) + (-1 * x58 * x394)) * x76);
	const FLT x396 = x389 + (-1 * x390);
	const FLT x397 = dt * x120;
	const FLT x398 = dt * x121;
	const FLT x399 = x398 + x397;
	const FLT x400 = x70 * x399;
	const FLT x401 = -1 * dt * x123;
	const FLT x402 = x393 + x401;
	const FLT x403 = (-1 * x79 * ((x86 * x402) + (-1 * ((x81 * x399) + (x80 * x396)) * x82))) +
					 (-1 * ((x72 * x400) + (-1 * x58 * x396)) * x76);
	const FLT x404 = x386 + x387;
	const FLT x405 = dt + x392 + x401;
	const FLT x406 = x70 * x405;
	const FLT x407 = x397 + (-1 * x398);
	const FLT x408 = (-1 * x79 * ((x86 * x407) + (-1 * ((x81 * x405) + (x80 * x404)) * x82))) +
					 (-1 * ((x72 * x406) + (-1 * x58 * x404)) * x76);
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT),
						x87 + (x89 * x87) + (((-1 * x85 * x58) + (x68 * x90)) * x91));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT),
						x99 + (x89 * x99) + (((-1 * x58 * x98) + (x90 * x95)) * x91));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT),
						x103 + (x89 * x103) + (((-1 * x58 * x102) + (x90 * x101)) * x91));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT),
						(x89 * x118) + x118 + (((-1 * x58 * x117) + (x90 * x113)) * x91));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT),
						x126 + (x89 * x126) + (((-1 * x58 * x125) + (x90 * x122)) * x91));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT),
						x130 + (x89 * x130) + (((-1 * x58 * x129) + (x90 * x128)) * x91));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						x171 + (x89 * x171) + (((-1 * x58 * x170) + (x90 * x169)) * x91));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						x196 + (x89 * x196) + (((-1 * x58 * x195) + (x90 * x194)) * x91));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x216 + (x89 * x216) + (((-1 * x58 * x215) + (x90 * x214)) * x91));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						x233 + (x89 * x233) + (((-1 * x58 * x232) + (x90 * x231)) * x91));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x317 + (x89 * x317) + (((-1 * x58 * x316) + (x90 * x315)) * x91));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x358 + (x89 * x358) + (((-1 * x58 * x357) + (x90 * x356)) * x91));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x385 + (((-1 * x58 * x383) + (x90 * x381)) * x91) + (x89 * x385));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT),
						x395 + (((-1 * x58 * x391) + (x90 * x388)) * x91) + (x89 * x395));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT),
						(x89 * x403) + x403 + (((-1 * x58 * x402) + (x77 * x400)) * x91));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT),
						x408 + (x89 * x408) + (((-1 * x58 * x407) + (x77 * x406)) * x91));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen1 wrt [(*_x0).AccBias[0], (*_x0).AccBias[1],
// (*_x0).AccBias[2], (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1],
// (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2],
// (*_x0).IMUCorrection[3], (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0],
// (*_x0).Pose.Rot[1], (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0],
// (*_x0).Velocity.AxisAngleRot[1], (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1],
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4eea520>]

static inline void SurviveKalmanErrorModel_LightMeas_x_gen1_jac_x0_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_x_gen1(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_x_gen1_jac_x0(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen1 wrt [(*error_model).AccBias[0], (*error_model).AccBias[1],
// (*error_model).AccBias[2], (*error_model).Acc[0], (*error_model).Acc[1], (*error_model).Acc[2],
// (*error_model).GyroBias[0], (*error_model).GyroBias[1], (*error_model).GyroBias[2], (*error_model).IMUCorrection[0],
// (*error_model).IMUCorrection[1], (*error_model).IMUCorrection[2], (*error_model).IMUCorrection[3],
// (*error_model).Pose.AxisAngleRot[0], (*error_model).Pose.AxisAngleRot[1], (*error_model).Pose.AxisAngleRot[2],
// (*error_model).Pose.Pos[0], (*error_model).Pose.Pos[1], (*error_model).Pose.Pos[2],
// (*error_model).Velocity.AxisAngleRot[0], (*error_model).Velocity.AxisAngleRot[1],
// (*error_model).Velocity.AxisAngleRot[2], (*error_model).Velocity.Pos[0], (*error_model).Velocity.Pos[1],
// (*error_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4edf2b0>]
static inline void SurviveKalmanErrorModel_LightMeas_x_gen1_jac_error_model(
	CnMat *Hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const FLT x0 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x1 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x2 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x4 =
		(*_x0).Pose.Rot[1] + (x3 * (*_x0).Pose.Rot[0]) + (-1 * x1 * (*_x0).Pose.Rot[3]) + (x2 * (*_x0).Pose.Rot[2]);
	const FLT x5 = dt * dt;
	const FLT x6 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x7 = x6 * x6;
	const FLT x8 = x5 * x7;
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x10 = x9 * x9;
	const FLT x11 = x5 * x10;
	const FLT x12 = x0 * x0;
	const FLT x13 = x5 * x12;
	const FLT x14 = 1e-10 + x13 + x8 + x11;
	const FLT x15 = sqrt(x14);
	const FLT x16 = 0.5 * x15;
	const FLT x17 = sin(x16);
	const FLT x18 = x17 * x17;
	const FLT x19 = 1. / x14;
	const FLT x20 = x19 * x18;
	const FLT x21 = cos(x16);
	const FLT x22 = (x20 * x11) + (x21 * x21) + (x8 * x20) + (x20 * x13);
	const FLT x23 = 1. / sqrt(x22);
	const FLT x24 = (1. / x15) * x17;
	const FLT x25 = dt * x24;
	const FLT x26 = x25 * x23;
	const FLT x27 = x4 * x26;
	const FLT x28 = (-1 * x3 * (*_x0).Pose.Rot[1]) + (-1 * x1 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x2 * (*_x0).Pose.Rot[3]);
	const FLT x29 = x28 * x26;
	const FLT x30 =
		(*_x0).Pose.Rot[3] + (x1 * (*_x0).Pose.Rot[1]) + (-1 * x3 * (*_x0).Pose.Rot[2]) + (x2 * (*_x0).Pose.Rot[0]);
	const FLT x31 = x23 * x21;
	const FLT x32 = x30 * x31;
	const FLT x33 =
		(-1 * x2 * (*_x0).Pose.Rot[1]) + (x3 * (*_x0).Pose.Rot[3]) + (x1 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2];
	const FLT x34 = x33 * x23;
	const FLT x35 = x34 * x25;
	const FLT x36 = (x9 * x35) + x32 + (-1 * x0 * x27) + (x6 * x29);
	const FLT x37 = x31 * x28;
	const FLT x38 = x30 * x26;
	const FLT x39 = (-1 * x6 * x38) + (-1 * x9 * x27) + (-1 * x0 * x35) + x37;
	const FLT x40 = x4 * x31;
	const FLT x41 = (x9 * x29) + x40 + (-1 * x6 * x35) + (x0 * x38);
	const FLT x42 = (x39 * sensor_pt[1]) + (-1 * x41 * sensor_pt[2]) + (x36 * sensor_pt[0]);
	const FLT x43 = x31 * x33;
	const FLT x44 = x43 + (x6 * x27) + (-1 * x9 * x38) + (x0 * x29);
	const FLT x45 = (-1 * x44 * sensor_pt[0]) + (x41 * sensor_pt[1]) + (x39 * sensor_pt[2]);
	const FLT x46 = dt * fabs(dt);
	const FLT x47 = 1.0 / 2.0 * x46;
	const FLT x48 = (x47 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + (2 * ((x44 * x45) + (-1 * x42 * x36))) +
					(*error_model).Pose.Pos[0] + sensor_pt[0] +
					(dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) + (*_x0).Pose.Pos[0];
	const FLT x49 = (-1 * x36 * sensor_pt[1]) + (x44 * sensor_pt[2]) + (x39 * sensor_pt[0]);
	const FLT x50 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (*error_model).Pose.Pos[2] +
					(x47 * ((*_x0).Acc[2] + (*error_model).Acc[2])) + (2 * ((x41 * x42) + (-1 * x44 * x49))) +
					(dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2]));
	const FLT x51 = (*_x0).Pose.Pos[1] + (*error_model).Pose.Pos[1] + sensor_pt[1] +
					(dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) +
					(2 * ((x49 * x36) + (-1 * x41 * x45))) + (x47 * ((*_x0).Acc[1] + (*error_model).Acc[1]));
	const FLT x52 = (-1 * x51 * (*lh_p).Rot[3]) + (x48 * (*lh_p).Rot[0]) + (x50 * (*lh_p).Rot[2]);
	const FLT x53 = (-1 * x50 * (*lh_p).Rot[1]) + (x51 * (*lh_p).Rot[0]) + (x48 * (*lh_p).Rot[3]);
	const FLT x54 = x50 + (2 * ((x53 * (*lh_p).Rot[1]) + (-1 * x52 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x55 = 1. / x54;
	const FLT x56 = (*lh_p).Rot[2] * (*lh_p).Rot[2];
	const FLT x57 = -1 * x56 * x46;
	const FLT x58 = (*lh_p).Rot[3] * (*lh_p).Rot[3];
	const FLT x59 = x47 + (-1 * x58 * x46);
	const FLT x60 = x59 + x57;
	const FLT x61 = x46 * (*lh_p).Rot[2] * (*lh_p).Rot[0];
	const FLT x62 = x46 * (*lh_p).Rot[1];
	const FLT x63 = x62 * (*lh_p).Rot[3];
	const FLT x64 = x63 + (-1 * x61);
	const FLT x65 = x54 * x54;
	const FLT x66 = 1. / x65;
	const FLT x67 = (-1 * x48 * (*lh_p).Rot[2]) + (x51 * (*lh_p).Rot[1]) + (x50 * (*lh_p).Rot[0]);
	const FLT x68 = x48 + (*lh_p).Pos[0] + (2 * ((x67 * (*lh_p).Rot[2]) + (-1 * x53 * (*lh_p).Rot[3])));
	const FLT x69 = x68 * x66;
	const FLT x70 = x65 + (x68 * x68);
	const FLT x71 = 1. / x70;
	const FLT x72 = x71 * x65;
	const FLT x73 = x51 + (*lh_p).Pos[1] + (2 * ((x52 * (*lh_p).Rot[3]) + (-1 * x67 * (*lh_p).Rot[1])));
	const FLT x74 = x73 * x73;
	const FLT x75 = 1. / sqrt(1 + (-1 * x71 * x74 * ((*bsc0).tilt * (*bsc0).tilt)));
	const FLT x76 = 2 * x68;
	const FLT x77 = 2 * x54;
	const FLT x78 = 1.0 / 2.0 * (1. / (x70 * sqrt(x70))) * x73 * (*bsc0).tilt;
	const FLT x79 = x62 * (*lh_p).Rot[2];
	const FLT x80 = x46 * (*lh_p).Rot[3];
	const FLT x81 = x80 * (*lh_p).Rot[0];
	const FLT x82 = x81 + x79;
	const FLT x83 = (1. / sqrt(x70)) * (*bsc0).tilt;
	const FLT x84 = (-1 * x75 * ((x82 * x83) + (-1 * ((x77 * x64) + (x76 * x60)) * x78))) +
					(-1 * ((x64 * x69) + (-1 * x60 * x55)) * x72);
	const FLT x85 = -1 * x54;
	const FLT x86 = (*bsc0).gibmag * sin(1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha +
										 (-1 * asin(x83 * x73)) + (-1 * atan2(x68, x85)));
	const FLT x87 = x73 * x66;
	const FLT x88 = 2 * (1. / (x65 + x74)) * x65 * atan2(x73, x85) * (*bsc0).curve;
	const FLT x89 = x79 + (-1 * x81);
	const FLT x90 = x80 * (*lh_p).Rot[2];
	const FLT x91 = x62 * (*lh_p).Rot[0];
	const FLT x92 = x91 + x90;
	const FLT x93 = (*lh_p).Rot[1] * (*lh_p).Rot[1];
	const FLT x94 = -1 * x93 * x46;
	const FLT x95 = x59 + x94;
	const FLT x96 = (-1 * x75 * ((x83 * x95) + (-1 * ((x77 * x92) + (x89 * x76)) * x78))) +
					(-1 * ((x69 * x92) + (-1 * x89 * x55)) * x72);
	const FLT x97 = x61 + x63;
	const FLT x98 = x57 + x47 + x94;
	const FLT x99 = x90 + (-1 * x91);
	const FLT x100 = (-1 * x75 * ((x83 * x99) + (-1 * ((x77 * x98) + (x76 * x97)) * x78))) +
					 (-1 * ((x69 * x98) + (-1 * x55 * x97)) * x72);
	const FLT x101 = 0.5 * x26;
	const FLT x102 = x9 * x101;
	const FLT x103 = 0.5 * x31;
	const FLT x104 = x6 * x101;
	const FLT x105 = x0 * x101;
	const FLT x106 = (-1 * x105 * (*_x0).Pose.Rot[2]) + (-1 * x104 * (*_x0).Pose.Rot[3]) +
					 (-1 * x102 * (*_x0).Pose.Rot[1]) + (x103 * (*_x0).Pose.Rot[0]);
	const FLT x107 = 2 * x42;
	const FLT x108 = x107 * x106;
	const FLT x109 = x102 * (*_x0).Pose.Rot[2];
	const FLT x110 = x103 * (*_x0).Pose.Rot[3];
	const FLT x111 = x104 * (*_x0).Pose.Rot[0];
	const FLT x112 = x105 * (*_x0).Pose.Rot[1];
	const FLT x113 = (-1 * x112) + x111 + x109 + x110;
	const FLT x114 = 2 * x49;
	const FLT x115 = x103 * (*_x0).Pose.Rot[2];
	const FLT x116 = x105 * (*_x0).Pose.Rot[0];
	const FLT x117 = x102 * (*_x0).Pose.Rot[3];
	const FLT x118 = x104 * (*_x0).Pose.Rot[1];
	const FLT x119 = x117 + (-1 * x118) + (-1 * x115) + (-1 * x116);
	const FLT x120 = x119 * sensor_pt[1];
	const FLT x121 = x102 * (*_x0).Pose.Rot[0];
	const FLT x122 = x104 * (*_x0).Pose.Rot[2];
	const FLT x123 = x105 * (*_x0).Pose.Rot[3];
	const FLT x124 = x103 * (*_x0).Pose.Rot[1];
	const FLT x125 = (-1 * x124) + (-1 * x123) + (-1 * x121) + x122;
	const FLT x126 = x125 * sensor_pt[0];
	const FLT x127 = x126 + (x113 * sensor_pt[2]) + (-1 * x120);
	const FLT x128 = 2 * x44;
	const FLT x129 = x106 * sensor_pt[2];
	const FLT x130 = x119 * sensor_pt[0];
	const FLT x131 = x130 + (-1 * x129) + (x125 * sensor_pt[1]);
	const FLT x132 = 2 * x41;
	const FLT x133 = (x131 * x132) + (-1 * x128 * x127) + x108 + (-1 * x113 * x114);
	const FLT x134 = 2 * x45;
	const FLT x135 = x106 * x134;
	const FLT x136 = 2 * x36;
	const FLT x137 = x106 * sensor_pt[1];
	const FLT x138 = x125 * sensor_pt[2];
	const FLT x139 = x138 + (-1 * x113 * sensor_pt[0]) + x137;
	const FLT x140 = (-1 * x132 * x139) + (x127 * x136) + (-1 * x135) + (x119 * x114);
	const FLT x141 = (-1 * x107 * x119) + (x128 * x139) + (x113 * x134) + (-1 * x131 * x136);
	const FLT x142 = (x141 * (*lh_p).Rot[3]) + (-1 * x133 * (*lh_p).Rot[1]) + (x140 * (*lh_p).Rot[0]);
	const FLT x143 = 2 * (*lh_p).Rot[3];
	const FLT x144 = (x133 * (*lh_p).Rot[0]) + (-1 * x141 * (*lh_p).Rot[2]) + (x140 * (*lh_p).Rot[1]);
	const FLT x145 = 2 * (*lh_p).Rot[2];
	const FLT x146 = x141 + (-1 * x142 * x143) + (x144 * x145);
	const FLT x147 = (x133 * (*lh_p).Rot[2]) + (-1 * x140 * (*lh_p).Rot[3]) + (x141 * (*lh_p).Rot[0]);
	const FLT x148 = 2 * (*lh_p).Rot[1];
	const FLT x149 = (-1 * x145 * x147) + x133 + (x142 * x148);
	const FLT x150 = (-1 * x144 * x148) + x140 + (x143 * x147);
	const FLT x151 = (-1 * x75 * ((x83 * x150) + (-1 * ((x77 * x149) + (x76 * x146)) * x78))) +
					 (-1 * ((x69 * x149) + (-1 * x55 * x146)) * x72);
	const FLT x152 = x112 + (-1 * x111) + (-1 * x109) + (-1 * x110);
	const FLT x153 = (-1 * x122) + x123 + x121 + x124;
	const FLT x154 = x130 + (-1 * x153 * sensor_pt[1]) + x129;
	const FLT x155 = x106 * sensor_pt[0];
	const FLT x156 = x152 * sensor_pt[1];
	const FLT x157 = (-1 * x155) + (x119 * sensor_pt[2]) + x156;
	const FLT x158 = (-1 * x132 * x157) + (-1 * x134 * x152) + (x136 * x154) + (x114 * x153);
	const FLT x159 = x152 * sensor_pt[2];
	const FLT x160 = (x153 * sensor_pt[0]) + (-1 * x159) + x120;
	const FLT x161 = (x128 * x157) + x135 + (-1 * x107 * x153) + (-1 * x160 * x136);
	const FLT x162 = x106 * x114;
	const FLT x163 = (-1 * x128 * x154) + (x160 * x132) + (-1 * x162) + (x107 * x152);
	const FLT x164 = (x163 * (*lh_p).Rot[2]) + (-1 * x158 * (*lh_p).Rot[3]) + (x161 * (*lh_p).Rot[0]);
	const FLT x165 = (-1 * x163 * (*lh_p).Rot[1]) + (x161 * (*lh_p).Rot[3]) + (x158 * (*lh_p).Rot[0]);
	const FLT x166 = x163 + (-1 * x164 * x145) + (x165 * x148);
	const FLT x167 = (x163 * (*lh_p).Rot[0]) + (-1 * x161 * (*lh_p).Rot[2]) + (x158 * (*lh_p).Rot[1]);
	const FLT x168 = x158 + (-1 * x167 * x148) + (x164 * x143);
	const FLT x169 = x161 + (-1 * x165 * x143) + (x167 * x145);
	const FLT x170 = (-1 * x75 * ((x83 * x168) + (-1 * ((x77 * x166) + (x76 * x169)) * x78))) +
					 (-1 * ((x69 * x166) + (-1 * x55 * x169)) * x72);
	const FLT x171 = (x152 * sensor_pt[0]) + (-1 * x137) + x138;
	const FLT x172 = x116 + (-1 * x117) + x115 + x118;
	const FLT x173 = x155 + (-1 * x172 * sensor_pt[2]) + x156;
	const FLT x174 = (x173 * x132) + (x107 * x172) + (-1 * x114 * x125) + (-1 * x128 * x171);
	const FLT x175 = x159 + (-1 * x126) + (x172 * sensor_pt[1]);
	const FLT x176 = (-1 * x175 * x132) + x162 + (-1 * x172 * x134) + (x171 * x136);
	const FLT x177 = (x128 * x175) + (x125 * x134) + (-1 * x108) + (-1 * x173 * x136);
	const FLT x178 = (x177 * (*lh_p).Rot[3]) + (-1 * x174 * (*lh_p).Rot[1]) + (x176 * (*lh_p).Rot[0]);
	const FLT x179 = (-1 * x177 * (*lh_p).Rot[2]) + (x174 * (*lh_p).Rot[0]) + (x176 * (*lh_p).Rot[1]);
	const FLT x180 = x177 + (-1 * x178 * x143) + (x179 * x145);
	const FLT x181 = (x174 * (*lh_p).Rot[2]) + (-1 * x176 * (*lh_p).Rot[3]) + (x177 * (*lh_p).Rot[0]);
	const FLT x182 = x174 + (-1 * x181 * x145) + (x178 * x148);
	const FLT x183 = x176 + (-1 * x179 * x148) + (x181 * x143);
	const FLT x184 = (-1 * x75 * ((x83 * x183) + (-1 * ((x77 * x182) + (x76 * x180)) * x78))) +
					 (-1 * ((x69 * x182) + (-1 * x55 * x180)) * x72);
	const FLT x185 = 2 * x58;
	const FLT x186 = -1 * x185;
	const FLT x187 = 2 * x56;
	const FLT x188 = 1 + (-1 * x187);
	const FLT x189 = x188 + x186;
	const FLT x190 = x145 * (*lh_p).Rot[0];
	const FLT x191 = x148 * (*lh_p).Rot[3];
	const FLT x192 = x191 + (-1 * x190);
	const FLT x193 = x145 * (*lh_p).Rot[1];
	const FLT x194 = x143 * (*lh_p).Rot[0];
	const FLT x195 = x194 + x193;
	const FLT x196 = (-1 * x75 * ((x83 * x195) + (-1 * ((x77 * x192) + (x76 * x189)) * x78))) +
					 (-1 * ((x69 * x192) + (-1 * x55 * x189)) * x72);
	const FLT x197 = x193 + (-1 * x194);
	const FLT x198 = x145 * (*lh_p).Rot[3];
	const FLT x199 = x148 * (*lh_p).Rot[0];
	const FLT x200 = x199 + x198;
	const FLT x201 = 2 * x93;
	const FLT x202 = -1 * x201;
	const FLT x203 = 1 + x186 + x202;
	const FLT x204 = (-1 * x75 * ((x83 * x203) + (-1 * ((x77 * x200) + (x76 * x197)) * x78))) +
					 (-1 * ((x69 * x200) + (-1 * x55 * x197)) * x72);
	const FLT x205 = x190 + x191;
	const FLT x206 = x188 + x202;
	const FLT x207 = x198 + (-1 * x199);
	const FLT x208 = (-1 * x75 * ((x83 * x207) + (-1 * ((x77 * x206) + (x76 * x205)) * x78))) +
					 (-1 * ((x69 * x206) + (-1 * x55 * x205)) * x72);
	const FLT x209 = -1 * x38;
	const FLT x210 = dt * dt * dt;
	const FLT x211 = 0.5 * x19;
	const FLT x212 = x211 * x210;
	const FLT x213 = x32 * x212;
	const FLT x214 = dt * dt * dt * dt;
	const FLT x215 = (x9 * x9 * x9) * x214;
	const FLT x216 = 1.0 * x21;
	const FLT x217 = (1. / (x14 * sqrt(x14))) * x17;
	const FLT x218 = x217 * x216;
	const FLT x219 = 2 * x20;
	const FLT x220 = x5 * x219;
	const FLT x221 = 2 * (1. / (x14 * x14)) * x18;
	const FLT x222 = x214 * x221;
	const FLT x223 = x9 * x222;
	const FLT x224 = x9 * x216;
	const FLT x225 = x214 * x217 * x224;
	const FLT x226 = x5 * x24;
	const FLT x227 = (x12 * x225) + (x9 * x220) + (x218 * x215) + (-1 * x12 * x223) + (-1 * x215 * x221) +
					 (-1 * x7 * x223) + (-1 * x224 * x226) + (x7 * x225);
	const FLT x228 = 1.0 / 2.0 * (1. / (x22 * sqrt(x22)));
	const FLT x229 = x25 * x228;
	const FLT x230 = x227 * x229;
	const FLT x231 = x0 * x28;
	const FLT x232 = x9 * x30;
	const FLT x233 = x4 * x23;
	const FLT x234 = x210 * x217;
	const FLT x235 = x234 * x233;
	const FLT x236 = x6 * x9;
	const FLT x237 = x236 * x235;
	const FLT x238 = x21 * x228;
	const FLT x239 = x238 * x227;
	const FLT x240 = x40 * x212;
	const FLT x241 = x236 * x240;
	const FLT x242 = x4 * x229;
	const FLT x243 = x227 * x242;
	const FLT x244 = x9 * x226;
	const FLT x245 = 0.5 * x34;
	const FLT x246 = x23 * x234;
	const FLT x247 = x30 * x246;
	const FLT x248 = x37 * x212;
	const FLT x249 = x0 * x9;
	const FLT x250 = x28 * x246;
	const FLT x251 = x0 * x250;
	const FLT x252 = (-1 * x9 * x251) + (x248 * x249);
	const FLT x253 = (-1 * x244 * x245) + (-1 * x6 * x243) + (-1 * x33 * x239) + x241 + (-1 * x10 * x213) + x252 +
					 (x10 * x247) + x209 + (-1 * x230 * x231) + (x230 * x232) + (-1 * x237);
	const FLT x254 = x30 * x230;
	const FLT x255 = 0.5 * x233;
	const FLT x256 = x34 * x234;
	const FLT x257 = x236 * x256;
	const FLT x258 = x6 * x33;
	const FLT x259 = x0 * x247;
	const FLT x260 = x9 * x259;
	const FLT x261 = x0 * x213;
	const FLT x262 = x9 * x261;
	const FLT x263 = x43 * x212;
	const FLT x264 = x236 * x263;
	const FLT x265 = x9 * x28;
	const FLT x266 = (-1 * x230 * x265) + x257 + (x10 * x248) + (-1 * x264) + (-1 * x0 * x254) + (-1 * x255 * x244) +
					 (-1 * x260) + (x230 * x258) + x29 + x262 + (-1 * x10 * x250) + (-1 * x4 * x239);
	const FLT x267 = -1 * x27;
	const FLT x268 = x0 * x256;
	const FLT x269 = x9 * x268;
	const FLT x270 = x0 * x33;
	const FLT x271 = x263 * x249;
	const FLT x272 = 0.5 * x23;
	const FLT x273 = x272 * x226;
	const FLT x274 = (x236 * x247) + (-1 * x213 * x236);
	const FLT x275 = x274 + (x9 * x243) + (x10 * x235) + (-1 * x273 * x265) + (x6 * x254) + (-1 * x271) +
					 (-1 * x10 * x240) + x267 + (-1 * x28 * x239) + x269 + (x230 * x270);
	const FLT x276 = x6 * x28;
	const FLT x277 = x9 * x33;
	const FLT x278 = x0 * x240;
	const FLT x279 = x0 * x235;
	const FLT x280 = (x9 * x279) + (-1 * x9 * x278);
	const FLT x281 = (-1 * x236 * x250) + (x236 * x248);
	const FLT x282 = x281 + x280 + (-1 * x10 * x256) + x35 + (x0 * x243) + (x10 * x263) + (-1 * x230 * x277) +
					 (-1 * x230 * x276) + (-1 * x232 * x273) + (-1 * x30 * x239);
	const FLT x283 = (x282 * sensor_pt[0]) + (-1 * x266 * sensor_pt[2]) + (x275 * sensor_pt[1]);
	const FLT x284 = (x275 * sensor_pt[0]) + (-1 * x282 * sensor_pt[1]) + (x253 * sensor_pt[2]);
	const FLT x285 = (x283 * x132) + (-1 * x253 * x114) + (-1 * x284 * x128) + (x266 * x107);
	const FLT x286 = (-1 * x253 * sensor_pt[0]) + (x275 * sensor_pt[2]) + (x266 * sensor_pt[1]);
	const FLT x287 = (x282 * x114) + (-1 * x286 * x132) + (-1 * x266 * x134) + (x284 * x136);
	const FLT x288 = (-1 * x282 * x107) + (x253 * x134) + (-1 * x283 * x136) + (x286 * x128);
	const FLT x289 = (x288 * (*lh_p).Rot[3]) + (-1 * x285 * (*lh_p).Rot[1]) + (x287 * (*lh_p).Rot[0]);
	const FLT x290 = (x285 * (*lh_p).Rot[0]) + (-1 * x288 * (*lh_p).Rot[2]) + (x287 * (*lh_p).Rot[1]);
	const FLT x291 = x288 + (-1 * x289 * x143) + (x290 * x145);
	const FLT x292 = (x285 * (*lh_p).Rot[2]) + (-1 * x287 * (*lh_p).Rot[3]) + (x288 * (*lh_p).Rot[0]);
	const FLT x293 = x285 + (-1 * x292 * x145) + (x289 * x148);
	const FLT x294 = (-1 * x290 * x148) + x287 + (x292 * x143);
	const FLT x295 = (-1 * x75 * ((x83 * x294) + (-1 * ((x77 * x293) + (x76 * x291)) * x78))) +
					 (-1 * ((x69 * x293) + (-1 * x55 * x291)) * x72);
	const FLT x296 = x218 * x214;
	const FLT x297 = x0 * x296;
	const FLT x298 = x0 * x222;
	const FLT x299 = x0 * x5;
	const FLT x300 = x24 * x299;
	const FLT x301 = x0 * x0 * x0;
	const FLT x302 = (-1 * x10 * x298) + (x296 * x301) + (x10 * x297) + (-1 * x216 * x300) + (x219 * x299) +
					 (-1 * x7 * x298) + (-1 * x222 * x301) + (x7 * x297);
	const FLT x303 = x229 * x302;
	const FLT x304 = x238 * x302;
	const FLT x305 = x30 * x303;
	const FLT x306 = x0 * x6;
	const FLT x307 = (-1 * x263 * x306) + (x6 * x268);
	const FLT x308 = x252 + x38 + (x258 * x303) + (-1 * x12 * x247) + (-1 * x255 * x300) + (x12 * x213) + x307 +
					 (-1 * x4 * x304) + (-1 * x265 * x303) + (-1 * x0 * x305);
	const FLT x309 = x242 * x302;
	const FLT x310 = x6 * x278;
	const FLT x311 = x6 * x279;
	const FLT x312 = (-1 * x245 * x300) + x260 + (-1 * x12 * x250) + (-1 * x262) + (x12 * x248) + (-1 * x33 * x304) +
					 x310 + (-1 * x231 * x303) + (-1 * x311) + x29 + (-1 * x6 * x309) + (x232 * x303);
	const FLT x313 = x6 * x259;
	const FLT x314 = x6 * x261;
	const FLT x315 = x28 * x272;
	const FLT x316 = -1 * x35;
	const FLT x317 = (x9 * x309) + (x12 * x256) + x316 + (-1 * x300 * x315) + x313 + (x270 * x303) + (-1 * x28 * x304) +
					 (-1 * x12 * x263) + (x6 * x305) + x280 + (-1 * x314);
	const FLT x318 = (x317 * sensor_pt[2]) + (-1 * x312 * sensor_pt[0]) + (x308 * sensor_pt[1]);
	const FLT x319 = x30 * x272;
	const FLT x320 = (x248 * x306) + (-1 * x6 * x251);
	const FLT x321 = (-1 * x300 * x319) + x320 + (-1 * x269) + (-1 * x30 * x304) + (-1 * x277 * x303) + x267 +
					 (-1 * x12 * x240) + (x0 * x309) + x271 + (-1 * x276 * x303) + (x12 * x235);
	const FLT x322 = (x317 * sensor_pt[0]) + (x312 * sensor_pt[2]) + (-1 * x321 * sensor_pt[1]);
	const FLT x323 = (x321 * x114) + (-1 * x308 * x134) + (x322 * x136) + (-1 * x318 * x132);
	const FLT x324 = (x321 * sensor_pt[0]) + (-1 * x308 * sensor_pt[2]) + (x317 * sensor_pt[1]);
	const FLT x325 = (x324 * x132) + (x308 * x107) + (-1 * x312 * x114) + (-1 * x322 * x128);
	const FLT x326 = (x318 * x128) + (-1 * x324 * x136) + (-1 * x321 * x107) + (x312 * x134);
	const FLT x327 = (x326 * (*lh_p).Rot[3]) + (x323 * (*lh_p).Rot[0]) + (-1 * x325 * (*lh_p).Rot[1]);
	const FLT x328 = (x325 * (*lh_p).Rot[0]) + (-1 * x326 * (*lh_p).Rot[2]) + (x323 * (*lh_p).Rot[1]);
	const FLT x329 = x326 + (-1 * x327 * x143) + (x328 * x145);
	const FLT x330 = (x325 * (*lh_p).Rot[2]) + (-1 * x323 * (*lh_p).Rot[3]) + (x326 * (*lh_p).Rot[0]);
	const FLT x331 = (-1 * x330 * x145) + x325 + (x327 * x148);
	const FLT x332 = x323 + (x330 * x143) + (-1 * x328 * x148);
	const FLT x333 = (-1 * x75 * ((x83 * x332) + (-1 * ((x77 * x331) + (x76 * x329)) * x78))) +
					 (-1 * ((x69 * x331) + (-1 * x55 * x329)) * x72);
	const FLT x334 = x6 * x296;
	const FLT x335 = x6 * x6 * x6;
	const FLT x336 = x6 * x222;
	const FLT x337 = x6 * x226;
	const FLT x338 = (x12 * x334) + (-1 * x216 * x337) + (x296 * x335) + (x10 * x334) + (-1 * x222 * x335) +
					 (x6 * x220) + (-1 * x12 * x336) + (-1 * x10 * x336);
	const FLT x339 = x229 * x338;
	const FLT x340 = x7 * x210;
	const FLT x341 = x211 * x340;
	const FLT x342 = x30 * x339;
	const FLT x343 = x217 * x340;
	const FLT x344 = x238 * x338;
	const FLT x345 = x281 + x314 + (-1 * x313) + (x34 * x343) + (-1 * x0 * x342) + (-1 * x43 * x341) + (x258 * x339) +
					 (-1 * x265 * x339) + (-1 * x4 * x344) + x316 + (-1 * x255 * x337);
	const FLT x346 = x242 * x338;
	const FLT x347 = x23 * x343;
	const FLT x348 = (-1 * x277 * x339) + (-1 * x310) + (x0 * x346) + (-1 * x257) + (-1 * x337 * x319) + (x37 * x341) +
					 x311 + x29 + x264 + (-1 * x30 * x344) + (-1 * x276 * x339) + (-1 * x28 * x347);
	const FLT x349 = x320 + (x232 * x339) + x274 + (-1 * x33 * x344) + (-1 * x245 * x337) + (x40 * x341) +
					 (-1 * x231 * x339) + (-1 * x6 * x346) + x27 + (-1 * x233 * x343);
	const FLT x350 = (x9 * x346) + x307 + (x30 * x347) + (x6 * x342) + (-1 * x241) + (-1 * x32 * x341) +
					 (-1 * x337 * x315) + x209 + (-1 * x28 * x344) + x237 + (x270 * x339);
	const FLT x351 = (x350 * sensor_pt[0]) + (-1 * x348 * sensor_pt[1]) + (x349 * sensor_pt[2]);
	const FLT x352 = (x350 * sensor_pt[2]) + (-1 * x349 * sensor_pt[0]) + (x345 * sensor_pt[1]);
	const FLT x353 = (-1 * x352 * x132) + (-1 * x345 * x134) + (x351 * x136) + (x348 * x114);
	const FLT x354 = (x348 * sensor_pt[0]) + (-1 * x345 * sensor_pt[2]) + (x350 * sensor_pt[1]);
	const FLT x355 = (x354 * x132) + (x345 * x107) + (-1 * x349 * x114) + (-1 * x351 * x128);
	const FLT x356 = (-1 * x354 * x136) + (x349 * x134) + (-1 * x348 * x107) + (x352 * x128);
	const FLT x357 = (x353 * (*lh_p).Rot[0]) + (x356 * (*lh_p).Rot[3]) + (-1 * x355 * (*lh_p).Rot[1]);
	const FLT x358 = (x355 * (*lh_p).Rot[2]) + (-1 * x353 * (*lh_p).Rot[3]) + (x356 * (*lh_p).Rot[0]);
	const FLT x359 = x355 + (x357 * x148) + (-1 * x358 * x145);
	const FLT x360 = (x355 * (*lh_p).Rot[0]) + (-1 * x356 * (*lh_p).Rot[2]) + (x353 * (*lh_p).Rot[1]);
	const FLT x361 = (-1 * x360 * x148) + x353 + (x358 * x143);
	const FLT x362 = x356 + (-1 * x357 * x143) + (x360 * x145);
	const FLT x363 = (-1 * x75 * ((x83 * x361) + (-1 * ((x77 * x359) + (x76 * x362)) * x78))) +
					 (-1 * ((x69 * x359) + (-1 * x55 * x362)) * x72);
	const FLT x364 = dt * x190;
	const FLT x365 = dt * x191;
	const FLT x366 = x365 + (-1 * x364);
	const FLT x367 = dt * x193;
	const FLT x368 = dt * x194;
	const FLT x369 = x368 + x367;
	const FLT x370 = -1 * dt * x185;
	const FLT x371 = -1 * dt * x187;
	const FLT x372 = x371 + x370 + dt;
	const FLT x373 = (-1 * x75 * ((x83 * x369) + (-1 * ((x77 * x366) + (x76 * x372)) * x78))) +
					 (-1 * ((x69 * x366) + (-1 * x55 * x372)) * x72);
	const FLT x374 = x367 + (-1 * x368);
	const FLT x375 = dt * x198;
	const FLT x376 = dt * x199;
	const FLT x377 = x376 + x375;
	const FLT x378 = (-1 * dt * x201) + dt;
	const FLT x379 = x378 + x370;
	const FLT x380 = (-1 * x75 * ((x83 * x379) + (-1 * ((x77 * x377) + (x76 * x374)) * x78))) +
					 (-1 * ((x69 * x377) + (-1 * x55 * x374)) * x72);
	const FLT x381 = x364 + x365;
	const FLT x382 = x378 + x371;
	const FLT x383 = x375 + (-1 * x376);
	const FLT x384 = (-1 * x75 * ((x83 * x383) + (-1 * ((x77 * x382) + (x76 * x381)) * x78))) +
					 (-1 * ((x69 * x382) + (-1 * x55 * x381)) * x72);
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Acc[0]) / sizeof(FLT),
						x84 + (x84 * x86) + (((-1 * x82 * x55) + (x87 * x64)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Acc[1]) / sizeof(FLT),
						x96 + (x86 * x96) + (((-1 * x55 * x95) + (x87 * x92)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Acc[2]) / sizeof(FLT),
						x100 + (x86 * x100) + (((-1 * x55 * x99) + (x87 * x98)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						x151 + (x86 * x151) + (((-1 * x55 * x150) + (x87 * x149)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						x170 + (((-1 * x55 * x168) + (x87 * x166)) * x88) + (x86 * x170));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						x184 + (x86 * x184) + (((-1 * x55 * x183) + (x87 * x182)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT),
						x196 + (x86 * x196) + (((-1 * x55 * x195) + (x87 * x192)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT),
						x204 + (x86 * x204) + (((-1 * x55 * x203) + (x87 * x200)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT),
						x208 + (x86 * x208) + (((-1 * x55 * x207) + (x87 * x206)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x295 + (x86 * x295) + (((-1 * x55 * x294) + (x87 * x293)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x333 + (x86 * x333) + (((-1 * x55 * x332) + (x87 * x331)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x363 + (((-1 * x55 * x361) + (x87 * x359)) * x88) + (x86 * x363));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.Pos[0]) / sizeof(FLT),
						x373 + (((-1 * x55 * x369) + (x87 * x366)) * x88) + (x86 * x373));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.Pos[1]) / sizeof(FLT),
						x380 + (x86 * x380) + (((-1 * x55 * x379) + (x87 * x377)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.Pos[2]) / sizeof(FLT),
						x384 + (x86 * x384) + (((-1 * x55 * x383) + (x87 * x382)) * x88));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen1 wrt [(*error_model).AccBias[0],
// (*error_model).AccBias[1], (*error_model).AccBias[2], (*error_model).Acc[0], (*error_model).Acc[1],
// (*error_model).Acc[2], (*error_model).GyroBias[0], (*error_model).GyroBias[1], (*error_model).GyroBias[2],
// (*error_model).IMUCorrection[0], (*error_model).IMUCorrection[1], (*error_model).IMUCorrection[2],
// (*error_model).IMUCorrection[3], (*error_model).Pose.AxisAngleRot[0], (*error_model).Pose.AxisAngleRot[1],
// (*error_model).Pose.AxisAngleRot[2], (*error_model).Pose.Pos[0], (*error_model).Pose.Pos[1],
// (*error_model).Pose.Pos[2], (*error_model).Velocity.AxisAngleRot[0], (*error_model).Velocity.AxisAngleRot[1],
// (*error_model).Velocity.AxisAngleRot[2], (*error_model).Velocity.Pos[0], (*error_model).Velocity.Pos[1],
// (*error_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4edf2b0>]

static inline void SurviveKalmanErrorModel_LightMeas_x_gen1_jac_error_model_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_x_gen1(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_x_gen1_jac_error_model(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen1 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]
static inline void SurviveKalmanErrorModel_LightMeas_x_gen1_jac_sensor_pt(CnMat *Hx, const FLT dt,
																		  const SurviveKalmanModel *_x0,
																		  const SurviveKalmanErrorModel *error_model,
																		  const FLT *sensor_pt, const SurvivePose *lh_p,
																		  const BaseStationCal *bsc0) {
	const FLT x0 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x1 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x2 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x4 =
		(-1 * x3 * (*_x0).Pose.Rot[1]) + (x2 * (*_x0).Pose.Rot[0]) + (x1 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x5 = dt * dt;
	const FLT x6 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x7 = (x6 * x6) * x5;
	const FLT x8 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x9 = x5 * (x8 * x8);
	const FLT x10 = (x0 * x0) * x5;
	const FLT x11 = 1e-10 + x7 + x10 + x9;
	const FLT x12 = sqrt(x11);
	const FLT x13 = 0.5 * x12;
	const FLT x14 = sin(x13);
	const FLT x15 = (x14 * x14) * (1. / x11);
	const FLT x16 = cos(x13);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x9 * x15) + (x7 * x15) + (x15 * x10));
	const FLT x18 = dt * x14 * (1. / x12) * x17;
	const FLT x19 = x4 * x18;
	const FLT x20 = (-1 * x1 * (*_x0).Pose.Rot[1]) + (-1 * x2 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x3 * (*_x0).Pose.Rot[3]);
	const FLT x21 = x17 * x16;
	const FLT x22 =
		(x2 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x1 * (*_x0).Pose.Rot[2]) + (x3 * (*_x0).Pose.Rot[0]);
	const FLT x23 = x22 * x18;
	const FLT x24 =
		(*_x0).Pose.Rot[1] + (x1 * (*_x0).Pose.Rot[0]) + (-1 * x2 * (*_x0).Pose.Rot[3]) + (x3 * (*_x0).Pose.Rot[2]);
	const FLT x25 = x24 * x18;
	const FLT x26 = (-1 * x8 * x25) + (-1 * x6 * x23) + (-1 * x0 * x19) + (x20 * x21);
	const FLT x27 = x6 * x25;
	const FLT x28 = x20 * x18;
	const FLT x29 = x0 * x28;
	const FLT x30 = x4 * x21;
	const FLT x31 = x8 * x23;
	const FLT x32 = x30 + x27 + (-1 * x31) + x29;
	const FLT x33 = 2 * x32;
	const FLT x34 = x33 * x26;
	const FLT x35 = x0 * x25;
	const FLT x36 = x6 * x28;
	const FLT x37 = x22 * x21;
	const FLT x38 = x8 * x19;
	const FLT x39 = x38 + x37 + (-1 * x35) + x36;
	const FLT x40 = x6 * x19;
	const FLT x41 = x0 * x23;
	const FLT x42 = x24 * x21;
	const FLT x43 = x8 * x28;
	const FLT x44 = x43 + x42 + (-1 * x40) + x41;
	const FLT x45 = 2 * x44;
	const FLT x46 = (x45 * x39) + (-1 * x34);
	const FLT x47 = x31 + (-1 * x29) + (-1 * x30) + (-1 * x27);
	const FLT x48 = 2 * x39;
	const FLT x49 = x48 * x26;
	const FLT x50 = x49 + (-1 * x45 * x47);
	const FLT x51 = 1 + (x47 * x33) + (-2 * (x39 * x39));
	const FLT x52 = 2 * ((x51 * (*lh_p).Rot[3]) + (-1 * x46 * (*lh_p).Rot[1]) + (x50 * (*lh_p).Rot[0]));
	const FLT x53 = 2 * ((x46 * (*lh_p).Rot[2]) + (-1 * x50 * (*lh_p).Rot[3]) + (x51 * (*lh_p).Rot[0]));
	const FLT x54 = x46 + (x52 * (*lh_p).Rot[1]) + (-1 * x53 * (*lh_p).Rot[2]);
	const FLT x55 = (x26 * sensor_pt[1]) + (-1 * x44 * sensor_pt[2]) + (x39 * sensor_pt[0]);
	const FLT x56 = (-1 * x32 * sensor_pt[0]) + (x44 * sensor_pt[1]) + (x26 * sensor_pt[2]);
	const FLT x57 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x58 = (x57 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + (2 * ((x56 * x32) + (-1 * x55 * x39))) +
					(*error_model).Pose.Pos[0] + sensor_pt[0] +
					(dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) + (*_x0).Pose.Pos[0];
	const FLT x59 = (-1 * x39 * sensor_pt[1]) + (x32 * sensor_pt[2]) + (x26 * sensor_pt[0]);
	const FLT x60 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (*error_model).Pose.Pos[2] +
					(x57 * ((*_x0).Acc[2] + (*error_model).Acc[2])) + (2 * ((x55 * x44) + (-1 * x59 * x32))) +
					(dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2]));
	const FLT x61 = (*_x0).Pose.Pos[1] + (*error_model).Pose.Pos[1] + (2 * ((x59 * x39) + (-1 * x56 * x44))) +
					(x57 * ((*_x0).Acc[1] + (*error_model).Acc[1])) + sensor_pt[1] +
					(dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1]));
	const FLT x62 = (-1 * x61 * (*lh_p).Rot[3]) + (x58 * (*lh_p).Rot[0]) + (x60 * (*lh_p).Rot[2]);
	const FLT x63 = (-1 * x60 * (*lh_p).Rot[1]) + (x61 * (*lh_p).Rot[0]) + (x58 * (*lh_p).Rot[3]);
	const FLT x64 = (2 * ((x63 * (*lh_p).Rot[1]) + (-1 * x62 * (*lh_p).Rot[2]))) + x60 + (*lh_p).Pos[2];
	const FLT x65 = x64 * x64;
	const FLT x66 = 1. / x65;
	const FLT x67 = (-1 * x58 * (*lh_p).Rot[2]) + (x61 * (*lh_p).Rot[1]) + (x60 * (*lh_p).Rot[0]);
	const FLT x68 = x61 + (*lh_p).Pos[1] + (2 * ((x62 * (*lh_p).Rot[3]) + (-1 * x67 * (*lh_p).Rot[1])));
	const FLT x69 = x68 * x66;
	const FLT x70 = 1. / x64;
	const FLT x71 = (x46 * (*lh_p).Rot[0]) + (-1 * x51 * (*lh_p).Rot[2]) + (x50 * (*lh_p).Rot[1]);
	const FLT x72 = 2 * (*lh_p).Rot[1];
	const FLT x73 = x50 + (x53 * (*lh_p).Rot[3]) + (-1 * x71 * x72);
	const FLT x74 = -1 * x64;
	const FLT x75 = x68 * x68;
	const FLT x76 = 2 * (1. / (x65 + x75)) * x65 * atan2(x68, x74) * (*bsc0).curve;
	const FLT x77 = 2 * (*lh_p).Rot[2];
	const FLT x78 = x51 + (x71 * x77) + (-1 * x52 * (*lh_p).Rot[3]);
	const FLT x79 = (*lh_p).Pos[0] + x58 + (2 * ((x67 * (*lh_p).Rot[2]) + (-1 * x63 * (*lh_p).Rot[3])));
	const FLT x80 = x79 * x66;
	const FLT x81 = x65 + (x79 * x79);
	const FLT x82 = 1. / x81;
	const FLT x83 = x82 * x65;
	const FLT x84 = 1. / sqrt(1 + (-1 * x82 * x75 * ((*bsc0).tilt * (*bsc0).tilt)));
	const FLT x85 = 2 * x79;
	const FLT x86 = 2 * x64;
	const FLT x87 = 1.0 / 2.0 * (1. / (x81 * sqrt(x81))) * x68 * (*bsc0).tilt;
	const FLT x88 = (1. / sqrt(x81)) * (*bsc0).tilt;
	const FLT x89 = (-1 * x84 * ((x88 * x73) + (-1 * ((x86 * x54) + (x85 * x78)) * x87))) +
					(-1 * ((x80 * x54) + (-1 * x70 * x78)) * x83);
	const FLT x90 = (*bsc0).gibmag * sin(1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha +
										 (-1 * asin(x88 * x68)) + (-1 * atan2(x79, x74)));
	const FLT x91 = x35 + (-1 * x38) + (-1 * x36) + (-1 * x37);
	const FLT x92 = 1 + (x91 * x48) + (-2 * (x44 * x44));
	const FLT x93 = x45 * x26;
	const FLT x94 = x93 + (-1 * x91 * x33);
	const FLT x95 = (x45 * x32) + (-1 * x49);
	const FLT x96 = (x95 * (*lh_p).Rot[3]) + (x92 * (*lh_p).Rot[0]) + (-1 * x94 * (*lh_p).Rot[1]);
	const FLT x97 = 2 * (*lh_p).Rot[3];
	const FLT x98 = (x94 * (*lh_p).Rot[0]) + (-1 * x95 * (*lh_p).Rot[2]) + (x92 * (*lh_p).Rot[1]);
	const FLT x99 = x95 + (-1 * x97 * x96) + (x77 * x98);
	const FLT x100 = (x94 * (*lh_p).Rot[2]) + (-1 * x92 * (*lh_p).Rot[3]) + (x95 * (*lh_p).Rot[0]);
	const FLT x101 = x94 + (x72 * x96) + (-1 * x77 * x100);
	const FLT x102 = (x97 * x100) + x92 + (-1 * x72 * x98);
	const FLT x103 = (-1 * x84 * ((x88 * x102) + (-1 * x87 * ((x86 * x101) + (x85 * x99))))) +
					 (-1 * x83 * ((x80 * x101) + (-1 * x70 * x99)));
	const FLT x104 = (x33 * x39) + (-1 * x93);
	const FLT x105 = (-1 * x42) + (-1 * x43) + (-1 * x41) + x40;
	const FLT x106 = 1 + (x45 * x105) + (-2 * (x32 * x32));
	const FLT x107 = x34 + (-1 * x48 * x105);
	const FLT x108 = 2 * ((x107 * (*lh_p).Rot[3]) + (x104 * (*lh_p).Rot[0]) + (-1 * x106 * (*lh_p).Rot[1]));
	const FLT x109 = (x106 * (*lh_p).Rot[0]) + (x104 * (*lh_p).Rot[1]) + (-1 * x107 * (*lh_p).Rot[2]);
	const FLT x110 = x107 + (-1 * x108 * (*lh_p).Rot[3]) + (x77 * x109);
	const FLT x111 = 2 * ((x106 * (*lh_p).Rot[2]) + (-1 * x104 * (*lh_p).Rot[3]) + (x107 * (*lh_p).Rot[0]));
	const FLT x112 = x106 + (x108 * (*lh_p).Rot[1]) + (-1 * x111 * (*lh_p).Rot[2]);
	const FLT x113 = x104 + (x111 * (*lh_p).Rot[3]) + (-1 * x72 * x109);
	const FLT x114 = (-1 * x84 * ((x88 * x113) + (-1 * ((x86 * x112) + (x85 * x110)) * x87))) +
					 (-1 * ((x80 * x112) + (-1 * x70 * x110)) * x83);
	cnMatrixOptionalSet(Hx, 0, 0, x89 + (((-1 * x70 * x73) + (x69 * x54)) * x76) + (x89 * x90));
	cnMatrixOptionalSet(Hx, 0, 1, x103 + (x90 * x103) + (((-1 * x70 * x102) + (x69 * x101)) * x76));
	cnMatrixOptionalSet(Hx, 0, 2, x114 + (x90 * x114) + (((-1 * x70 * x113) + (x69 * x112)) * x76));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen1 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]

static inline void SurviveKalmanErrorModel_LightMeas_x_gen1_jac_sensor_pt_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_x_gen1(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_x_gen1_jac_sensor_pt(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen1 wrt [(*lh_p).Pos[0], (*lh_p).Pos[1], (*lh_p).Pos[2],
// (*lh_p).Rot[0], (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]
static inline void SurviveKalmanErrorModel_LightMeas_x_gen1_jac_lh_p(CnMat *Hx, const FLT dt,
																	 const SurviveKalmanModel *_x0,
																	 const SurviveKalmanErrorModel *error_model,
																	 const FLT *sensor_pt, const SurvivePose *lh_p,
																	 const BaseStationCal *bsc0) {
	const FLT x0 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x1 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x2 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x4 =
		(*_x0).Pose.Rot[1] + (x3 * (*_x0).Pose.Rot[0]) + (-1 * x1 * (*_x0).Pose.Rot[3]) + (x2 * (*_x0).Pose.Rot[2]);
	const FLT x5 = dt * dt;
	const FLT x6 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x7 = (x6 * x6) * x5;
	const FLT x8 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x9 = x5 * (x8 * x8);
	const FLT x10 = (x0 * x0) * x5;
	const FLT x11 = 1e-10 + x7 + x10 + x9;
	const FLT x12 = sqrt(x11);
	const FLT x13 = 0.5 * x12;
	const FLT x14 = sin(x13);
	const FLT x15 = (x14 * x14) * (1. / x11);
	const FLT x16 = cos(x13);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x9 * x15) + (x7 * x15) + (x15 * x10));
	const FLT x18 = dt * x14 * (1. / x12) * x17;
	const FLT x19 = x4 * x18;
	const FLT x20 = (-1 * x3 * (*_x0).Pose.Rot[1]) + (-1 * x1 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x2 * (*_x0).Pose.Rot[3]);
	const FLT x21 = x20 * x18;
	const FLT x22 =
		(*_x0).Pose.Rot[3] + (x1 * (*_x0).Pose.Rot[1]) + (-1 * x3 * (*_x0).Pose.Rot[2]) + (x2 * (*_x0).Pose.Rot[0]);
	const FLT x23 = x17 * x16;
	const FLT x24 =
		(-1 * x2 * (*_x0).Pose.Rot[1]) + (x3 * (*_x0).Pose.Rot[3]) + (x1 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2];
	const FLT x25 = x24 * x18;
	const FLT x26 = (x22 * x23) + (-1 * x0 * x19) + (x8 * x25) + (x6 * x21);
	const FLT x27 = x22 * x18;
	const FLT x28 = (-1 * x8 * x19) + (-1 * x0 * x25) + (-1 * x6 * x27) + (x23 * x20);
	const FLT x29 = (x8 * x21) + (x4 * x23) + (-1 * x6 * x25) + (x0 * x27);
	const FLT x30 = (-1 * x29 * sensor_pt[2]) + (x28 * sensor_pt[1]) + (x26 * sensor_pt[0]);
	const FLT x31 = (-1 * x8 * x27) + (x24 * x23) + (x6 * x19) + (x0 * x21);
	const FLT x32 = (-1 * x31 * sensor_pt[0]) + (x29 * sensor_pt[1]) + (x28 * sensor_pt[2]);
	const FLT x33 = 2 * ((x32 * x31) + (-1 * x30 * x26));
	const FLT x34 = dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0]);
	const FLT x35 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x36 = x35 * ((*_x0).Acc[0] + (*error_model).Acc[0]);
	const FLT x37 = x33 + x36 + sensor_pt[0] + (*error_model).Pose.Pos[0] + x34 + (*_x0).Pose.Pos[0];
	const FLT x38 = x37 * (*lh_p).Rot[0];
	const FLT x39 = (-1 * x26 * sensor_pt[1]) + (x31 * sensor_pt[2]) + (x28 * sensor_pt[0]);
	const FLT x40 = 2 * ((x30 * x29) + (-1 * x31 * x39));
	const FLT x41 = x35 * ((*_x0).Acc[2] + (*error_model).Acc[2]);
	const FLT x42 = dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2]);
	const FLT x43 = sensor_pt[2] + x40 + (*error_model).Pose.Pos[2] + (*_x0).Pose.Pos[2] + x41 + x42;
	const FLT x44 = x43 * (*lh_p).Rot[2];
	const FLT x45 = 2 * ((x39 * x26) + (-1 * x32 * x29));
	const FLT x46 = dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1]);
	const FLT x47 = x35 * ((*_x0).Acc[1] + (*error_model).Acc[1]);
	const FLT x48 = (*_x0).Pose.Pos[1] + x45 + (*error_model).Pose.Pos[1] + sensor_pt[1] + x46 + x47;
	const FLT x49 = x48 * (*lh_p).Rot[3];
	const FLT x50 = (-1 * x49) + x38 + x44;
	const FLT x51 = x48 * (*lh_p).Rot[0];
	const FLT x52 = x37 * (*lh_p).Rot[3];
	const FLT x53 = x43 * (*lh_p).Rot[1];
	const FLT x54 = (-1 * x53) + x51 + x52;
	const FLT x55 = (2 * ((x54 * (*lh_p).Rot[1]) + (-1 * x50 * (*lh_p).Rot[2]))) + x43 + (*lh_p).Pos[2];
	const FLT x56 = x55 * x55;
	const FLT x57 = x48 * (*lh_p).Rot[1];
	const FLT x58 = x43 * (*lh_p).Rot[0];
	const FLT x59 = x37 * (*lh_p).Rot[2];
	const FLT x60 = (-1 * x59) + x57 + x58;
	const FLT x61 = x37 + (*lh_p).Pos[0] + (2 * ((x60 * (*lh_p).Rot[2]) + (-1 * x54 * (*lh_p).Rot[3])));
	const FLT x62 = (x61 * x61) + x56;
	const FLT x63 = 1. / x62;
	const FLT x64 = x48 + (*lh_p).Pos[1] + (2 * ((x50 * (*lh_p).Rot[3]) + (-1 * x60 * (*lh_p).Rot[1])));
	const FLT x65 = x64 * x64;
	const FLT x66 = 1. / sqrt(1 + (-1 * x63 * x65 * ((*bsc0).tilt * (*bsc0).tilt)));
	const FLT x67 = x64 * (1. / (x62 * sqrt(x62))) * (*bsc0).tilt;
	const FLT x68 = x67 * x66;
	const FLT x69 = (x61 * x68) + (x63 * x55);
	const FLT x70 = (1. / sqrt(x62)) * (*bsc0).tilt;
	const FLT x71 = -1 * x55;
	const FLT x72 = (*bsc0).gibmag * sin(1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha +
										 (-1 * asin(x70 * x64)) + (-1 * atan2(x61, x71)));
	const FLT x73 = x70 * x66;
	const FLT x74 = 2 * x55;
	const FLT x75 = (1. / (x56 + x65)) * atan2(x64, x71) * (*bsc0).curve;
	const FLT x76 = (x68 * x55) + (-1 * x63 * x61);
	const FLT x77 = 2 * x75;
	const FLT x78 = 1. / x55;
	const FLT x79 = 2 * x49;
	const FLT x80 = (2 * x44) + (-1 * x79);
	const FLT x81 = 2 * x59;
	const FLT x82 = (2 * x57) + (-1 * x81);
	const FLT x83 = 1. / x56;
	const FLT x84 = x83 * x61;
	const FLT x85 = x63 * x56;
	const FLT x86 = 2 * x61;
	const FLT x87 = 1.0 / 2.0 * x67;
	const FLT x88 = 2 * x53;
	const FLT x89 = (2 * x52) + (-1 * x88);
	const FLT x90 = (-1 * x66 * ((x89 * x70) + (-1 * ((x82 * x74) + (x80 * x86)) * x87))) +
					(-1 * ((x82 * x84) + (-1 * x80 * x78)) * x85);
	const FLT x91 = x83 * x64;
	const FLT x92 = x77 * x56;
	const FLT x93 = (-1 * sensor_pt[2]) + (-1 * x42) + (-1 * x41) + (-1 * (*_x0).Pose.Pos[2]) + (-1 * x40) +
					(-1 * (*error_model).Pose.Pos[2]);
	const FLT x94 = 2 * (*lh_p).Rot[3];
	const FLT x95 = 2 * (*lh_p).Rot[2];
	const FLT x96 = (x95 * x48) + (-1 * x93 * x94);
	const FLT x97 = 2 * (*lh_p).Rot[1];
	const FLT x98 = 2 * x51;
	const FLT x99 = (x93 * x97) + x89 + x98;
	const FLT x100 = 2 * x58;
	const FLT x101 = (-1 * x100) + (-4 * x57) + x81;
	const FLT x102 = (-1 * x66 * ((x70 * x101) + (-1 * ((x74 * x99) + (x86 * x96)) * x87))) +
					 (-1 * ((x84 * x99) + (-1 * x78 * x96)) * x85);
	const FLT x103 = (-1 * (*error_model).Pose.Pos[0]) + (-1 * x33) + (-1 * (*_x0).Pose.Pos[0]) + (-1 * x34) +
					 (-1 * sensor_pt[0]) + (-1 * x36);
	const FLT x104 = x82 + (x95 * x103) + x100;
	const FLT x105 = 2 * x38;
	const FLT x106 = (-4 * x44) + (-1 * x105) + x79;
	const FLT x107 = (x94 * x43) + (-1 * x97 * x103);
	const FLT x108 = (-1 * x66 * ((x70 * x107) + (-1 * ((x74 * x106) + (x86 * x104)) * x87))) +
					 (-1 * ((x84 * x106) + (-1 * x78 * x104)) * x85);
	const FLT x109 = (-1 * sensor_pt[1]) + (-1 * x45) + (-1 * (*error_model).Pose.Pos[1]) + (-1 * x47) +
					 (-1 * (*_x0).Pose.Pos[1]) + (-1 * x46);
	const FLT x110 = (x97 * x37) + (-1 * x95 * x109);
	const FLT x111 = x80 + x105 + (x94 * x109);
	const FLT x112 = x88 + (-1 * x98) + (-4 * x52);
	const FLT x113 = (-1 * x66 * ((x70 * x111) + (-1 * ((x74 * x110) + (x86 * x112)) * x87))) +
					 (-1 * ((x84 * x110) + (-1 * x78 * x112)) * x85);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[0]) / sizeof(FLT), x69 + (x72 * x69));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[1]) / sizeof(FLT),
						(-1 * x73) + (-1 * x73 * x72) + (-1 * x75 * x74));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[2]) / sizeof(FLT), x76 + (x72 * x76) + (x77 * x64));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[0]) / sizeof(FLT),
						x90 + (x72 * x90) + (((-1 * x89 * x78) + (x82 * x91)) * x92));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[1]) / sizeof(FLT),
						x102 + (x72 * x102) + (x92 * ((-1 * x78 * x101) + (x91 * x99))));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[2]) / sizeof(FLT),
						x108 + (x72 * x108) + (((-1 * x78 * x107) + (x91 * x106)) * x92));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[3]) / sizeof(FLT),
						(((-1 * x78 * x111) + (x91 * x110)) * x92) + x113 + (x72 * x113));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen1 wrt [(*lh_p).Pos[0], (*lh_p).Pos[1],
// (*lh_p).Pos[2], (*lh_p).Rot[0], (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]

static inline void SurviveKalmanErrorModel_LightMeas_x_gen1_jac_lh_p_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_x_gen1(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_x_gen1_jac_lh_p(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen1 wrt [<cnkalman.codegen.WrapMember object at 0x7f88f4eef0a0>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4eef160>, <cnkalman.codegen.WrapMember object at 0x7f88f4eef100>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4eef220>, <cnkalman.codegen.WrapMember object at 0x7f88f4eef1c0>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4edfe50>, <cnkalman.codegen.WrapMember object at 0x7f88f4eef040>]
static inline void SurviveKalmanErrorModel_LightMeas_x_gen1_jac_bsc0(CnMat *Hx, const FLT dt,
																	 const SurviveKalmanModel *_x0,
																	 const SurviveKalmanErrorModel *error_model,
																	 const FLT *sensor_pt, const SurvivePose *lh_p,
																	 const BaseStationCal *bsc0) {
	const FLT x0 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x1 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x2 = 0.5 * (*_x0).Pose.Rot[0];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x4 = (-1 * x3 * (*_x0).Pose.Rot[1]) + (x2 * (*error_model).Pose.AxisAngleRot[1]) +
				   (x1 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x5 = dt * dt;
	const FLT x6 = (x0 * x0) * x5;
	const FLT x7 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x8 = x5 * (x7 * x7);
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x10 = x5 * (x9 * x9);
	const FLT x11 = 1e-10 + x10 + x6 + x8;
	const FLT x12 = sqrt(x11);
	const FLT x13 = 0.5 * x12;
	const FLT x14 = sin(x13);
	const FLT x15 = (x14 * x14) * (1. / x11);
	const FLT x16 = cos(x13);
	const FLT x17 = 1. / sqrt((x8 * x15) + (x16 * x16) + (x6 * x15) + (x15 * x10));
	const FLT x18 = dt * x14 * (1. / x12) * x17;
	const FLT x19 = x4 * x18;
	const FLT x20 = 0.5 * (*_x0).Pose.Rot[2];
	const FLT x21 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x22 = (x21 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x20 * (*error_model).Pose.AxisAngleRot[0]) +
					(x2 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x23 = x22 * x18;
	const FLT x24 = (*_x0).Pose.Rot[1] + (x2 * (*error_model).Pose.AxisAngleRot[0]) + (-1 * x21 * (*_x0).Pose.Rot[3]) +
					(x20 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x25 = x17 * x16;
	const FLT x26 = (-1 * x1 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[0] +
					(-1 * x20 * (*error_model).Pose.AxisAngleRot[1]) + (-1 * x3 * (*_x0).Pose.Rot[3]);
	const FLT x27 = x26 * x18;
	const FLT x28 = (x7 * x27) + (-1 * x0 * x19) + (x24 * x25) + (x9 * x23);
	const FLT x29 = x24 * x18;
	const FLT x30 = (-1 * x7 * x29) + (-1 * x9 * x19) + (-1 * x0 * x23) + (x25 * x26);
	const FLT x31 = (-1 * x7 * x23) + (x4 * x25) + (x0 * x29) + (x9 * x27);
	const FLT x32 = (-1 * x31 * sensor_pt[0]) + (x28 * sensor_pt[1]) + (x30 * sensor_pt[2]);
	const FLT x33 = (x7 * x19) + (x25 * x22) + (-1 * x9 * x29) + (x0 * x27);
	const FLT x34 = (x31 * sensor_pt[2]) + (-1 * x33 * sensor_pt[1]) + (x30 * sensor_pt[0]);
	const FLT x35 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x36 = (*_x0).Pose.Pos[1] + (2 * ((x34 * x33) + (-1 * x32 * x28))) + (*error_model).Pose.Pos[1] +
					sensor_pt[1] + (dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) +
					(x35 * ((*_x0).Acc[1] + (*error_model).Acc[1]));
	const FLT x37 = (x30 * sensor_pt[1]) + (-1 * x28 * sensor_pt[2]) + (x33 * sensor_pt[0]);
	const FLT x38 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (x35 * ((*_x0).Acc[2] + (*error_model).Acc[2])) +
					(2 * ((x37 * x28) + (-1 * x31 * x34))) +
					(dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2])) + (*error_model).Pose.Pos[2];
	const FLT x39 = (x35 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + (*error_model).Pose.Pos[0] +
					(2 * ((x32 * x31) + (-1 * x33 * x37))) + sensor_pt[0] +
					(dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) + (*_x0).Pose.Pos[0];
	const FLT x40 = (-1 * x39 * (*lh_p).Rot[2]) + (x36 * (*lh_p).Rot[1]) + (x38 * (*lh_p).Rot[0]);
	const FLT x41 = (x39 * (*lh_p).Rot[0]) + (-1 * x36 * (*lh_p).Rot[3]) + (x38 * (*lh_p).Rot[2]);
	const FLT x42 = x36 + (*lh_p).Pos[1] + (2 * ((x41 * (*lh_p).Rot[3]) + (-1 * x40 * (*lh_p).Rot[1])));
	const FLT x43 = (x36 * (*lh_p).Rot[0]) + (-1 * x38 * (*lh_p).Rot[1]) + (x39 * (*lh_p).Rot[3]);
	const FLT x44 = x38 + (2 * ((x43 * (*lh_p).Rot[1]) + (-1 * x41 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x45 = -1 * x44;
	const FLT x46 = x39 + (*lh_p).Pos[0] + (2 * ((x40 * (*lh_p).Rot[2]) + (-1 * x43 * (*lh_p).Rot[3])));
	const FLT x47 = (x46 * x46) + (x44 * x44);
	const FLT x48 = x42 * (1. / sqrt(x47));
	const FLT x49 = 1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha + (-1 * asin(x48 * (*bsc0).tilt)) +
					(-1 * atan2(x46, x45));
	const FLT x50 = sin(x49) * (*bsc0).gibmag;
	const FLT x51 = x48 * (1. / sqrt(1 + (-1 * (x42 * x42) * (1. / x47) * ((*bsc0).tilt * (*bsc0).tilt))));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, curve) / sizeof(FLT), atan2(x42, x45) * atan2(x42, x45));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibmag) / sizeof(FLT), -1 * cos(x49));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibpha) / sizeof(FLT), x50);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, phase) / sizeof(FLT), -1 + (-1 * x50));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, tilt) / sizeof(FLT), (-1 * x51) + (-1 * x50 * x51));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen1 wrt [<cnkalman.codegen.WrapMember object at
// 0x7f88f4eef0a0>, <cnkalman.codegen.WrapMember object at 0x7f88f4eef160>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4eef100>, <cnkalman.codegen.WrapMember object at 0x7f88f4eef220>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4eef1c0>, <cnkalman.codegen.WrapMember object at 0x7f88f4edfe50>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4eef040>]

static inline void SurviveKalmanErrorModel_LightMeas_x_gen1_jac_bsc0_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_x_gen1(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_x_gen1_jac_bsc0(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
static inline FLT SurviveKalmanErrorModel_LightMeas_y_gen1(const FLT dt, const SurviveKalmanModel *_x0,
														   const SurviveKalmanErrorModel *error_model,
														   const FLT *sensor_pt, const SurvivePose *lh_p,
														   const BaseStationCal *bsc0) {
	const FLT x0 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x1 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x2 = 0.5 * (*_x0).Pose.Rot[0];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x4 = (-1 * x3 * (*_x0).Pose.Rot[1]) + (x2 * (*error_model).Pose.AxisAngleRot[1]) +
				   (x1 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x5 = dt * dt;
	const FLT x6 = (x0 * x0) * x5;
	const FLT x7 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x8 = x5 * (x7 * x7);
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x10 = x5 * (x9 * x9);
	const FLT x11 = 1e-10 + x10 + x6 + x8;
	const FLT x12 = sqrt(x11);
	const FLT x13 = 0.5 * x12;
	const FLT x14 = sin(x13);
	const FLT x15 = (x14 * x14) * (1. / x11);
	const FLT x16 = cos(x13);
	const FLT x17 = 1. / sqrt((x8 * x15) + (x16 * x16) + (x6 * x15) + (x15 * x10));
	const FLT x18 = dt * x14 * (1. / x12) * x17;
	const FLT x19 = x4 * x18;
	const FLT x20 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x21 = (x20 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x1 * (*_x0).Pose.Rot[2]) +
					(x2 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x22 = x21 * x18;
	const FLT x23 = (*_x0).Pose.Rot[1] + (x2 * (*error_model).Pose.AxisAngleRot[0]) + (-1 * x20 * (*_x0).Pose.Rot[3]) +
					(x3 * (*_x0).Pose.Rot[2]);
	const FLT x24 = x17 * x16;
	const FLT x25 = (-1 * x1 * (*_x0).Pose.Rot[1]) + (-1 * x20 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x3 * (*_x0).Pose.Rot[3]);
	const FLT x26 = x25 * x18;
	const FLT x27 = (x7 * x26) + (x24 * x23) + (-1 * x0 * x19) + (x9 * x22);
	const FLT x28 = x23 * x18;
	const FLT x29 = (-1 * x7 * x28) + (-1 * x0 * x22) + (-1 * x9 * x19) + (x24 * x25);
	const FLT x30 = (x4 * x24) + (-1 * x7 * x22) + (x0 * x28) + (x9 * x26);
	const FLT x31 = (-1 * x30 * sensor_pt[0]) + (x27 * sensor_pt[1]) + (x29 * sensor_pt[2]);
	const FLT x32 = (x24 * x21) + (x7 * x19) + (-1 * x9 * x28) + (x0 * x26);
	const FLT x33 = (-1 * x32 * sensor_pt[1]) + (x30 * sensor_pt[2]) + (x29 * sensor_pt[0]);
	const FLT x34 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x35 = (*_x0).Pose.Pos[1] + (2 * ((x32 * x33) + (-1 * x31 * x27))) + sensor_pt[1] +
					(dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) + (*error_model).Pose.Pos[1] +
					(x34 * ((*_x0).Acc[1] + (*error_model).Acc[1]));
	const FLT x36 = (-1 * x27 * sensor_pt[2]) + (x29 * sensor_pt[1]) + (x32 * sensor_pt[0]);
	const FLT x37 = (x34 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + (*error_model).Pose.Pos[0] + sensor_pt[0] +
					(dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) +
					(2 * ((x30 * x31) + (-1 * x32 * x36))) + (*_x0).Pose.Pos[0];
	const FLT x38 = sensor_pt[2] + (*_x0).Pose.Pos[2] + (2 * ((x36 * x27) + (-1 * x30 * x33))) +
					(*error_model).Pose.Pos[2] + (x34 * ((*_x0).Acc[2] + (*error_model).Acc[2])) +
					(dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2]));
	const FLT x39 = (x35 * (*lh_p).Rot[0]) + (-1 * x38 * (*lh_p).Rot[1]) + (x37 * (*lh_p).Rot[3]);
	const FLT x40 = (-1 * x37 * (*lh_p).Rot[2]) + (x35 * (*lh_p).Rot[1]) + (x38 * (*lh_p).Rot[0]);
	const FLT x41 = x37 + (*lh_p).Pos[0] + (2 * ((x40 * (*lh_p).Rot[2]) + (-1 * x39 * (*lh_p).Rot[3])));
	const FLT x42 = (-1 * x35 * (*lh_p).Rot[3]) + (x37 * (*lh_p).Rot[0]) + (x38 * (*lh_p).Rot[2]);
	const FLT x43 = x38 + (2 * ((x39 * (*lh_p).Rot[1]) + (-1 * x42 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x44 = -1 * x43;
	const FLT x45 = x35 + (*lh_p).Pos[1] + (2 * ((x42 * (*lh_p).Rot[3]) + (-1 * x40 * (*lh_p).Rot[1])));
	const FLT x46 = (-1 * (*bsc0).phase) + (-1 * asin((1. / sqrt((x45 * x45) + (x43 * x43))) * x41 * (*bsc0).tilt)) +
					(-1 * atan2(-1 * x45, x44));
	return x46 + ((atan2(x41, x44) * atan2(x41, x44)) * (*bsc0).curve) +
		   (-1 * cos(1.5707963267949 + x46 + (*bsc0).gibpha) * (*bsc0).gibmag);
}

// Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen1 wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2],
// (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3],
// (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1],
// (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1],
// (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4f3beb0>]
static inline void SurviveKalmanErrorModel_LightMeas_y_gen1_jac_x0(CnMat *Hx, const FLT dt,
																   const SurviveKalmanModel *_x0,
																   const SurviveKalmanErrorModel *error_model,
																   const FLT *sensor_pt, const SurvivePose *lh_p,
																   const BaseStationCal *bsc0) {
	const FLT x0 = dt * fabs(dt);
	const FLT x1 = x0 * (*lh_p).Rot[2] * (*lh_p).Rot[0];
	const FLT x2 = x0 * (*lh_p).Rot[1];
	const FLT x3 = x2 * (*lh_p).Rot[3];
	const FLT x4 = x3 + (-1 * x1);
	const FLT x5 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x6 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x7 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x8 =
		(x6 * (*_x0).Pose.Rot[0]) + (x5 * (*_x0).Pose.Rot[3]) + (-1 * x7 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[2];
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x10 = dt * dt;
	const FLT x11 = x9 * x9;
	const FLT x12 = x11 * x10;
	const FLT x13 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x14 = x13 * x13;
	const FLT x15 = x14 * x10;
	const FLT x16 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x17 = x16 * x16;
	const FLT x18 = x10 * x17;
	const FLT x19 = 1e-10 + x18 + x12 + x15;
	const FLT x20 = sqrt(x19);
	const FLT x21 = 1. / x20;
	const FLT x22 = 0.5 * x20;
	const FLT x23 = sin(x22);
	const FLT x24 = x23 * x23;
	const FLT x25 = 1. / x19;
	const FLT x26 = x24 * x25;
	const FLT x27 = cos(x22);
	const FLT x28 = (x26 * x15) + (x27 * x27) + (x26 * x12) + (x26 * x18);
	const FLT x29 = 1. / sqrt(x28);
	const FLT x30 = x23 * x29;
	const FLT x31 = dt * x30;
	const FLT x32 = x31 * x21;
	const FLT x33 = x9 * x32;
	const FLT x34 =
		(x6 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x5 * (*_x0).Pose.Rot[2]) + (x7 * (*_x0).Pose.Rot[0]);
	const FLT x35 = x32 * x16;
	const FLT x36 =
		(*_x0).Pose.Rot[1] + (x5 * (*_x0).Pose.Rot[0]) + (-1 * x6 * (*_x0).Pose.Rot[3]) + (x7 * (*_x0).Pose.Rot[2]);
	const FLT x37 = x29 * x27;
	const FLT x38 = x36 * x37;
	const FLT x39 = (-1 * x6 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] + (-1 * x5 * (*_x0).Pose.Rot[1]) +
					(-1 * x7 * (*_x0).Pose.Rot[3]);
	const FLT x40 = x21 * x13;
	const FLT x41 = x40 * x31;
	const FLT x42 = (-1 * x8 * x33) + (x41 * x39) + x38 + (x34 * x35);
	const FLT x43 = x37 * x39;
	const FLT x44 = (-1 * x41 * x36) + (-1 * x34 * x33) + (-1 * x8 * x35) + x43;
	const FLT x45 = x8 * x37;
	const FLT x46 = (-1 * x41 * x34) + x45 + (x33 * x36) + (x35 * x39);
	const FLT x47 = (-1 * x46 * sensor_pt[0]) + (x42 * sensor_pt[1]) + (x44 * sensor_pt[2]);
	const FLT x48 = x34 * x37;
	const FLT x49 = (x8 * x41) + x48 + (-1 * x36 * x35) + (x33 * x39);
	const FLT x50 = (-1 * x49 * sensor_pt[1]) + (x46 * sensor_pt[2]) + (x44 * sensor_pt[0]);
	const FLT x51 = 1.0 / 2.0 * x0;
	const FLT x52 = (*_x0).Pose.Pos[1] + (2 * ((x50 * x49) + (-1 * x42 * x47))) + (*error_model).Pose.Pos[1] +
					sensor_pt[1] + (dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) +
					(x51 * ((*_x0).Acc[1] + (*error_model).Acc[1]));
	const FLT x53 = (-1 * x42 * sensor_pt[2]) + (x44 * sensor_pt[1]) + (x49 * sensor_pt[0]);
	const FLT x54 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (*error_model).Pose.Pos[2] +
					(x51 * ((*_x0).Acc[2] + (*error_model).Acc[2])) + (2 * ((x53 * x42) + (-1 * x50 * x46))) +
					(dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2]));
	const FLT x55 = (2 * ((x46 * x47) + (-1 * x53 * x49))) + sensor_pt[0] +
					(dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) +
					(x51 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + (*error_model).Pose.Pos[0] + (*_x0).Pose.Pos[0];
	const FLT x56 = (-1 * x55 * (*lh_p).Rot[2]) + (x52 * (*lh_p).Rot[1]) + (x54 * (*lh_p).Rot[0]);
	const FLT x57 = (-1 * x52 * (*lh_p).Rot[3]) + (x55 * (*lh_p).Rot[0]) + (x54 * (*lh_p).Rot[2]);
	const FLT x58 = x52 + (*lh_p).Pos[1] + (2 * ((x57 * (*lh_p).Rot[3]) + (-1 * x56 * (*lh_p).Rot[1])));
	const FLT x59 = (-1 * x54 * (*lh_p).Rot[1]) + (x52 * (*lh_p).Rot[0]) + (x55 * (*lh_p).Rot[3]);
	const FLT x60 = x54 + (2 * ((x59 * (*lh_p).Rot[1]) + (-1 * x57 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x61 = x60 * x60;
	const FLT x62 = 1. / x61;
	const FLT x63 = x62 * x58;
	const FLT x64 = x2 * (*lh_p).Rot[2];
	const FLT x65 = x0 * (*lh_p).Rot[3];
	const FLT x66 = x65 * (*lh_p).Rot[0];
	const FLT x67 = x66 + x64;
	const FLT x68 = 1. / x60;
	const FLT x69 = x61 + (x58 * x58);
	const FLT x70 = 1. / x69;
	const FLT x71 = x70 * x61;
	const FLT x72 = 2 * x58;
	const FLT x73 = 2 * x60;
	const FLT x74 = x55 + (*lh_p).Pos[0] + (2 * ((x56 * (*lh_p).Rot[2]) + (-1 * x59 * (*lh_p).Rot[3])));
	const FLT x75 = 1.0 / 2.0 * x74 * (1. / (x69 * sqrt(x69))) * (*bsc0).tilt;
	const FLT x76 = (*lh_p).Rot[3] * (*lh_p).Rot[3];
	const FLT x77 = -1 * x0 * x76;
	const FLT x78 = (*lh_p).Rot[2] * (*lh_p).Rot[2];
	const FLT x79 = (-1 * x0 * x78) + x51;
	const FLT x80 = x79 + x77;
	const FLT x81 = (1. / sqrt(x69)) * (*bsc0).tilt;
	const FLT x82 = x74 * x74;
	const FLT x83 = 1. / sqrt(1 + (-1 * x82 * x70 * ((*bsc0).tilt * (*bsc0).tilt)));
	const FLT x84 = (-1 * x83 * ((x80 * x81) + (-1 * x75 * ((x4 * x73) + (x72 * x67))))) +
					(-1 * x71 * ((x67 * x68) + (-1 * x4 * x63)));
	const FLT x85 = -1 * x60;
	const FLT x86 = (*bsc0).gibmag * sin(1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha +
										 (-1 * asin(x81 * x74)) + (-1 * atan2(-1 * x58, x85)));
	const FLT x87 = x74 * x62;
	const FLT x88 = 2 * (1. / (x61 + x82)) * x61 * atan2(x74, x85) * (*bsc0).curve;
	const FLT x89 = x65 * (*lh_p).Rot[2];
	const FLT x90 = x2 * (*lh_p).Rot[0];
	const FLT x91 = x90 + x89;
	const FLT x92 = (*lh_p).Rot[1] * (*lh_p).Rot[1];
	const FLT x93 = -1 * x0 * x92;
	const FLT x94 = x93 + x77 + x51;
	const FLT x95 = x64 + (-1 * x66);
	const FLT x96 = (-1 * x83 * ((x81 * x95) + (-1 * ((x73 * x91) + (x72 * x94)) * x75))) +
					(-1 * ((x68 * x94) + (-1 * x63 * x91)) * x71);
	const FLT x97 = x79 + x93;
	const FLT x98 = x89 + (-1 * x90);
	const FLT x99 = x1 + x3;
	const FLT x100 = (-1 * x83 * ((x81 * x99) + (-1 * ((x73 * x97) + (x72 * x98)) * x75))) +
					 (-1 * ((x68 * x98) + (-1 * x63 * x97)) * x71);
	const FLT x101 = 2 * (*lh_p).Rot[2];
	const FLT x102 = x101 * (*lh_p).Rot[0];
	const FLT x103 = 2 * (*lh_p).Rot[1];
	const FLT x104 = x103 * (*lh_p).Rot[3];
	const FLT x105 = x104 + (-1 * x102);
	const FLT x106 = x103 * (*lh_p).Rot[2];
	const FLT x107 = 2 * (*lh_p).Rot[3];
	const FLT x108 = x107 * (*lh_p).Rot[0];
	const FLT x109 = x108 + x106;
	const FLT x110 = 2 * x76;
	const FLT x111 = -1 * x110;
	const FLT x112 = 2 * x78;
	const FLT x113 = -1 * x112;
	const FLT x114 = 1 + x113 + x111;
	const FLT x115 = (-1 * x83 * ((x81 * x114) + (-1 * ((x73 * x105) + (x72 * x109)) * x75))) +
					 (-1 * ((x68 * x109) + (-1 * x63 * x105)) * x71);
	const FLT x116 = x107 * (*lh_p).Rot[2];
	const FLT x117 = x103 * (*lh_p).Rot[0];
	const FLT x118 = x117 + x116;
	const FLT x119 = 2 * x92;
	const FLT x120 = 1 + (-1 * x119);
	const FLT x121 = x120 + x111;
	const FLT x122 = x106 + (-1 * x108);
	const FLT x123 = (-1 * x83 * ((x81 * x122) + (-1 * ((x73 * x118) + (x72 * x121)) * x75))) +
					 (-1 * ((x68 * x121) + (-1 * x63 * x118)) * x71);
	const FLT x124 = x120 + x113;
	const FLT x125 = x116 + (-1 * x117);
	const FLT x126 = x102 + x104;
	const FLT x127 = (-1 * x83 * ((x81 * x126) + (-1 * ((x73 * x124) + (x72 * x125)) * x75))) +
					 (-1 * ((x68 * x125) + (-1 * x63 * x124)) * x71);
	const FLT x128 = x6 * x37;
	const FLT x129 = x5 * x33;
	const FLT x130 = -1 * x7 * x41;
	const FLT x131 = x35 + x130;
	const FLT x132 = x131 + x128 + x129;
	const FLT x133 = 2 * x50;
	const FLT x134 = x7 * x33;
	const FLT x135 = -1 * x134;
	const FLT x136 = x6 * x35;
	const FLT x137 = -1 * x136;
	const FLT x138 = x5 * x41;
	const FLT x139 = (-1 * x138) + x37;
	const FLT x140 = x139 + x135 + x137;
	const FLT x141 = x5 * x37;
	const FLT x142 = x7 * x35;
	const FLT x143 = -1 * x6 * x33;
	const FLT x144 = x41 + x143;
	const FLT x145 = x144 + x141 + x142;
	const FLT x146 = x6 * x41;
	const FLT x147 = x7 * x37;
	const FLT x148 = -1 * x5 * x35;
	const FLT x149 = x33 + x148;
	const FLT x150 = x149 + x146 + x147;
	const FLT x151 = (x150 * sensor_pt[0]) + (x140 * sensor_pt[1]) + (-1 * x145 * sensor_pt[2]);
	const FLT x152 = 2 * x42;
	const FLT x153 = (x140 * sensor_pt[0]) + (-1 * x150 * sensor_pt[1]) + (x132 * sensor_pt[2]);
	const FLT x154 = 2 * x46;
	const FLT x155 = 2 * x53;
	const FLT x156 = (x145 * x155) + (-1 * x153 * x154) + (-1 * x133 * x132) + (x151 * x152);
	const FLT x157 = (x140 * sensor_pt[2]) + (-1 * x132 * sensor_pt[0]) + (x145 * sensor_pt[1]);
	const FLT x158 = 2 * x47;
	const FLT x159 = 2 * x49;
	const FLT x160 = (x133 * x150) + (x153 * x159) + (-1 * x152 * x157) + (-1 * x145 * x158);
	const FLT x161 = (x154 * x157) + (x132 * x158) + (-1 * x150 * x155) + (-1 * x151 * x159);
	const FLT x162 = 2 * ((x161 * (*lh_p).Rot[3]) + (-1 * x156 * (*lh_p).Rot[1]) + (x160 * (*lh_p).Rot[0]));
	const FLT x163 = 2 * ((x156 * (*lh_p).Rot[2]) + (-1 * x160 * (*lh_p).Rot[3]) + (x161 * (*lh_p).Rot[0]));
	const FLT x164 = (x162 * (*lh_p).Rot[1]) + x156 + (-1 * x163 * (*lh_p).Rot[2]);
	const FLT x165 = 2 * ((x156 * (*lh_p).Rot[0]) + (-1 * x161 * (*lh_p).Rot[2]) + (x160 * (*lh_p).Rot[1]));
	const FLT x166 = (-1 * x165 * (*lh_p).Rot[1]) + x160 + (x163 * (*lh_p).Rot[3]);
	const FLT x167 = x161 + (-1 * x162 * (*lh_p).Rot[3]) + (x165 * (*lh_p).Rot[2]);
	const FLT x168 = (-1 * x83 * ((x81 * x167) + (-1 * ((x73 * x164) + (x72 * x166)) * x75))) +
					 (-1 * ((x68 * x166) + (-1 * x63 * x164)) * x71);
	const FLT x169 = x139 + x134 + x136;
	const FLT x170 = -1 * x129;
	const FLT x171 = (-1 * x35) + x130;
	const FLT x172 = x171 + x128 + x170;
	const FLT x173 = -1 * x147;
	const FLT x174 = -1 * x146;
	const FLT x175 = x173 + x149 + x174;
	const FLT x176 = -1 * x141;
	const FLT x177 = x143 + (-1 * x41);
	const FLT x178 = x177 + x142 + x176;
	const FLT x179 = (x178 * sensor_pt[2]) + (-1 * x175 * sensor_pt[0]) + (x169 * sensor_pt[1]);
	const FLT x180 = (x178 * sensor_pt[0]) + (-1 * x172 * sensor_pt[1]) + (x175 * sensor_pt[2]);
	const FLT x181 = (x180 * x159) + (-1 * x179 * x152) + (-1 * x169 * x158) + (x172 * x133);
	const FLT x182 = (x172 * sensor_pt[0]) + (-1 * x169 * sensor_pt[2]) + (x178 * sensor_pt[1]);
	const FLT x183 = (-1 * x182 * x159) + (x179 * x154) + (-1 * x172 * x155) + (x175 * x158);
	const FLT x184 = (x182 * x152) + (x169 * x155) + (-1 * x175 * x133) + (-1 * x180 * x154);
	const FLT x185 = (x184 * (*lh_p).Rot[2]) + (-1 * x181 * (*lh_p).Rot[3]) + (x183 * (*lh_p).Rot[0]);
	const FLT x186 = (x183 * (*lh_p).Rot[3]) + (-1 * x184 * (*lh_p).Rot[1]) + (x181 * (*lh_p).Rot[0]);
	const FLT x187 = (-1 * x101 * x185) + x184 + (x103 * x186);
	const FLT x188 = (-1 * x183 * (*lh_p).Rot[2]) + (x184 * (*lh_p).Rot[0]) + (x181 * (*lh_p).Rot[1]);
	const FLT x189 = x181 + (-1 * x103 * x188) + (x107 * x185);
	const FLT x190 = x183 + (-1 * x107 * x186) + (x101 * x188);
	const FLT x191 = (-1 * x83 * ((x81 * x190) + (-1 * ((x73 * x187) + (x72 * x189)) * x75))) +
					 (-1 * ((x68 * x189) + (-1 * x63 * x187)) * x71);
	const FLT x192 = x148 + (-1 * x33);
	const FLT x193 = x192 + x147 + x174;
	const FLT x194 = -1 * x142;
	const FLT x195 = x144 + x194 + x176;
	const FLT x196 = x138 + x37;
	const FLT x197 = x196 + x134 + x137;
	const FLT x198 = -1 * x128;
	const FLT x199 = x171 + x129 + x198;
	const FLT x200 = (x197 * sensor_pt[2]) + (x199 * sensor_pt[0]) + (-1 * x195 * sensor_pt[1]);
	const FLT x201 = (x199 * sensor_pt[2]) + (-1 * x197 * sensor_pt[0]) + (x193 * sensor_pt[1]);
	const FLT x202 = (-1 * x193 * x158) + (-1 * x201 * x152) + (x200 * x159) + (x195 * x133);
	const FLT x203 = (x195 * sensor_pt[0]) + (x199 * sensor_pt[1]) + (-1 * x193 * sensor_pt[2]);
	const FLT x204 = (x201 * x154) + (x197 * x158) + (-1 * x195 * x155) + (-1 * x203 * x159);
	const FLT x205 = (x203 * x152) + (x193 * x155) + (-1 * x197 * x133) + (-1 * x200 * x154);
	const FLT x206 = (x205 * (*lh_p).Rot[2]) + (-1 * x202 * (*lh_p).Rot[3]) + (x204 * (*lh_p).Rot[0]);
	const FLT x207 = (-1 * x205 * (*lh_p).Rot[1]) + (x204 * (*lh_p).Rot[3]) + (x202 * (*lh_p).Rot[0]);
	const FLT x208 = x205 + (-1 * x206 * x101) + (x207 * x103);
	const FLT x209 = (x205 * (*lh_p).Rot[0]) + (-1 * x204 * (*lh_p).Rot[2]) + (x202 * (*lh_p).Rot[1]);
	const FLT x210 = x202 + (-1 * x209 * x103) + (x206 * x107);
	const FLT x211 = x204 + (-1 * x207 * x107) + (x209 * x101);
	const FLT x212 = (-1 * x83 * ((x81 * x211) + (-1 * ((x73 * x208) + (x72 * x210)) * x75))) +
					 (-1 * ((x68 * x210) + (-1 * x63 * x208)) * x71);
	const FLT x213 = x131 + x170 + x198;
	const FLT x214 = x177 + x141 + x194;
	const FLT x215 = x146 + x192 + x173;
	const FLT x216 = (x215 * sensor_pt[2]) + (-1 * x214 * sensor_pt[0]) + (x213 * sensor_pt[1]);
	const FLT x217 = x196 + x136 + x135;
	const FLT x218 = (-1 * x217 * sensor_pt[1]) + (x215 * sensor_pt[0]) + (x214 * sensor_pt[2]);
	const FLT x219 = (x217 * x133) + (-1 * x213 * x158) + (x218 * x159) + (-1 * x216 * x152);
	const FLT x220 = (x217 * sensor_pt[0]) + (-1 * x213 * sensor_pt[2]) + (x215 * sensor_pt[1]);
	const FLT x221 = (-1 * x220 * x159) + (-1 * x217 * x155) + (x214 * x158) + (x216 * x154);
	const FLT x222 = (x220 * x152) + (x213 * x155) + (-1 * x214 * x133) + (-1 * x218 * x154);
	const FLT x223 = (x222 * (*lh_p).Rot[2]) + (-1 * x219 * (*lh_p).Rot[3]) + (x221 * (*lh_p).Rot[0]);
	const FLT x224 = (x221 * (*lh_p).Rot[3]) + (-1 * x222 * (*lh_p).Rot[1]) + (x219 * (*lh_p).Rot[0]);
	const FLT x225 = x222 + (-1 * x223 * x101) + (x224 * x103);
	const FLT x226 = (x222 * (*lh_p).Rot[0]) + (x219 * (*lh_p).Rot[1]) + (-1 * x221 * (*lh_p).Rot[2]);
	const FLT x227 = x219 + (x223 * x107) + (-1 * x226 * x103);
	const FLT x228 = x221 + (-1 * x224 * x107) + (x226 * x101);
	const FLT x229 = (-1 * x83 * ((x81 * x228) + (-1 * ((x73 * x225) + (x72 * x227)) * x75))) +
					 (-1 * ((x68 * x227) + (-1 * x63 * x225)) * x71);
	const FLT x230 = dt * dt * dt * dt;
	const FLT x231 = (x13 * x13 * x13) * x230;
	const FLT x232 = 1. / (x19 * sqrt(x19));
	const FLT x233 = 1.0 * x23 * x27;
	const FLT x234 = x233 * x232;
	const FLT x235 = 2 * x26;
	const FLT x236 = x10 * x235;
	const FLT x237 = 2 * x24 * (1. / (x19 * x19));
	const FLT x238 = x230 * x237;
	const FLT x239 = x13 * x11;
	const FLT x240 = x230 * x234;
	const FLT x241 = x17 * x240;
	const FLT x242 = x10 * x233;
	const FLT x243 = x17 * x238;
	const FLT x244 = (x13 * x241) + (-1 * x40 * x242) + (x13 * x236) + (-1 * x13 * x243) + (x234 * x231) +
					 (-1 * x231 * x237) + (x239 * x240) + (-1 * x238 * x239);
	const FLT x245 = x34 * x244;
	const FLT x246 = 1.0 / 2.0 * (1. / (x28 * sqrt(x28)));
	const FLT x247 = dt * x23 * x246;
	const FLT x248 = x21 * x16;
	const FLT x249 = x247 * x248;
	const FLT x250 = x30 * x36;
	const FLT x251 = 0.5 * x250;
	const FLT x252 = x10 * x251;
	const FLT x253 = dt * dt * dt;
	const FLT x254 = x14 * x253;
	const FLT x255 = 0.5 * x25;
	const FLT x256 = x43 * x255;
	const FLT x257 = x9 * x13;
	const FLT x258 = x30 * x232;
	const FLT x259 = x8 * x258;
	const FLT x260 = x253 * x259;
	const FLT x261 = x260 * x257;
	const FLT x262 = x9 * x21;
	const FLT x263 = x262 * x244;
	const FLT x264 = x8 * x247;
	const FLT x265 = x34 * x258;
	const FLT x266 = x265 * x253;
	const FLT x267 = x16 * x266;
	const FLT x268 = x13 * x267;
	const FLT x269 = x48 * x255;
	const FLT x270 = x269 * x253;
	const FLT x271 = x16 * x270;
	const FLT x272 = x13 * x271;
	const FLT x273 = x39 * x258;
	const FLT x274 = x27 * x246;
	const FLT x275 = x274 * x244;
	const FLT x276 = x45 * x255;
	const FLT x277 = x276 * x253;
	const FLT x278 = x277 * x257;
	const FLT x279 = x32 * x39;
	const FLT x280 = x40 * x39;
	const FLT x281 = x280 * x247;
	const FLT x282 = (-1 * x281 * x244) + (x264 * x263) + x261 + (-1 * x268) + (-1 * x36 * x275) + (x254 * x256) +
					 (-1 * x245 * x249) + (-1 * x40 * x252) + x272 + (-1 * x273 * x254) + (-1 * x278) + x279;
	const FLT x283 = x8 * x32;
	const FLT x284 = 0.5 * x30;
	const FLT x285 = x10 * x284;
	const FLT x286 = x40 * x285;
	const FLT x287 = x36 * x244;
	const FLT x288 = x39 * x247;
	const FLT x289 = x40 * x264;
	const FLT x290 = x38 * x255;
	const FLT x291 = x290 * x253;
	const FLT x292 = x16 * x291;
	const FLT x293 = x232 * x250;
	const FLT x294 = x293 * x253;
	const FLT x295 = x16 * x294;
	const FLT x296 = (x13 * x295) + (-1 * x13 * x292);
	const FLT x297 = x253 * x256;
	const FLT x298 = x273 * x253;
	const FLT x299 = (-1 * x298 * x257) + (x297 * x257);
	const FLT x300 = (-1 * x289 * x244) + x299 + (-1 * x263 * x288) + x296 + (-1 * x254 * x259) + x283 + (x287 * x249) +
					 (x276 * x254) + (-1 * x34 * x286) + (-1 * x34 * x275);
	const FLT x301 = x40 * x247;
	const FLT x302 = x294 * x257;
	const FLT x303 = x291 * x257;
	const FLT x304 = x32 * x34;
	const FLT x305 = -1 * x304;
	const FLT x306 = x16 * x298;
	const FLT x307 = x16 * x297;
	const FLT x308 = (x13 * x307) + (-1 * x13 * x306);
	const FLT x309 = x308 + (x265 * x254) + x305 + (-1 * x8 * x286) + x303 + (-1 * x288 * x244 * x248) +
					 (-1 * x269 * x254) + (-1 * x302) + (x245 * x301) + (-1 * x36 * x263 * x247) + (-1 * x8 * x275);
	const FLT x310 = x39 * x274;
	const FLT x311 = x16 * x260;
	const FLT x312 = x13 * x311;
	const FLT x313 = x264 * x248;
	const FLT x314 = x16 * x277;
	const FLT x315 = x13 * x314;
	const FLT x316 = x34 * x247;
	const FLT x317 = x32 * x36;
	const FLT x318 = -1 * x317;
	const FLT x319 = (-1 * x270 * x257) + (x266 * x257);
	const FLT x320 = x318 + (-1 * x285 * x280) + (x14 * x294) + (x263 * x316) + x319 + (x244 * x313) + (x287 * x301) +
					 (-1 * x290 * x254) + (-1 * x244 * x310) + x312 + (-1 * x315);
	const FLT x321 = (x320 * sensor_pt[0]) + (-1 * x300 * sensor_pt[1]) + (x309 * sensor_pt[2]);
	const FLT x322 = (x320 * sensor_pt[2]) + (-1 * x309 * sensor_pt[0]) + (x282 * sensor_pt[1]);
	const FLT x323 = (x300 * x133) + (-1 * x322 * x152) + (-1 * x282 * x158) + (x321 * x159);
	const FLT x324 = (-1 * x282 * sensor_pt[2]) + (x300 * sensor_pt[0]) + (x320 * sensor_pt[1]);
	const FLT x325 = (x309 * x158) + (-1 * x324 * x159) + (-1 * x300 * x155) + (x322 * x154);
	const FLT x326 = (-1 * x321 * x154) + (x324 * x152) + (-1 * x309 * x133) + (x282 * x155);
	const FLT x327 = 2 * ((x326 * (*lh_p).Rot[2]) + (-1 * x323 * (*lh_p).Rot[3]) + (x325 * (*lh_p).Rot[0]));
	const FLT x328 = 2 * ((x325 * (*lh_p).Rot[3]) + (-1 * x326 * (*lh_p).Rot[1]) + (x323 * (*lh_p).Rot[0]));
	const FLT x329 = (-1 * x327 * (*lh_p).Rot[2]) + x326 + (x328 * (*lh_p).Rot[1]);
	const FLT x330 = 2 * ((x326 * (*lh_p).Rot[0]) + (-1 * x325 * (*lh_p).Rot[2]) + (x323 * (*lh_p).Rot[1]));
	const FLT x331 = x323 + (-1 * x330 * (*lh_p).Rot[1]) + (x327 * (*lh_p).Rot[3]);
	const FLT x332 = x325 + (-1 * x328 * (*lh_p).Rot[3]) + (x330 * (*lh_p).Rot[2]);
	const FLT x333 = (-1 * x83 * ((x81 * x332) + (-1 * ((x73 * x329) + (x72 * x331)) * x75))) +
					 (-1 * ((x68 * x331) + (-1 * x63 * x329)) * x71);
	const FLT x334 = x17 * x253;
	const FLT x335 = x16 * x240;
	const FLT x336 = x16 * x238;
	const FLT x337 = x10 * x16;
	const FLT x338 = x21 * x337;
	const FLT x339 = x16 * x16 * x16;
	const FLT x340 = (x240 * x339) + (x14 * x335) + (-1 * x233 * x338) + (-1 * x14 * x336) + (-1 * x11 * x336) +
					 (-1 * x238 * x339) + (x235 * x337) + (x11 * x335);
	const FLT x341 = x264 * x340;
	const FLT x342 = x36 * x274;
	const FLT x343 = x34 * x340;
	const FLT x344 = (-1 * x9 * x314) + (x9 * x311);
	const FLT x345 = x344 + (-1 * x251 * x338) + x304 + (-1 * x281 * x340) + (x262 * x341) + x308 + (-1 * x265 * x334) +
					 (-1 * x249 * x343) + (-1 * x342 * x340) + (x269 * x334);
	const FLT x346 = x288 * x340;
	const FLT x347 = x36 * x340;
	const FLT x348 = x8 * x274;
	const FLT x349 = x9 * x292;
	const FLT x350 = x9 * x295;
	const FLT x351 = x284 * x338;
	const FLT x352 = (-1 * x8 * x351) + x279 + (-1 * x262 * x247 * x347) + (-1 * x273 * x334) + (-1 * x272) +
					 (-1 * x248 * x346) + (x256 * x334) + x349 + (-1 * x350) + (x301 * x343) + (-1 * x348 * x340) +
					 x268;
	const FLT x353 = x9 * x267;
	const FLT x354 = x9 * x271;
	const FLT x355 = -1 * x283;
	const FLT x356 = x355 + x296 + (x259 * x334) + x353 + (-1 * x310 * x340) + (x248 * x341) + (-1 * x276 * x334) +
					 (x301 * x347) + (-1 * x39 * x351) + (x262 * x316 * x340) + (-1 * x354);
	const FLT x357 = (x356 * sensor_pt[2]) + (-1 * x352 * sensor_pt[0]) + (x345 * sensor_pt[1]);
	const FLT x358 = x34 * x274;
	const FLT x359 = (x9 * x307) + (-1 * x9 * x306);
	const FLT x360 = x359 + (-1 * x34 * x351) + (-1 * x40 * x341) + (-1 * x358 * x340) + x315 + (x249 * x347) + x318 +
					 (-1 * x290 * x334) + (-1 * x312) + (x17 * x294) + (-1 * x262 * x346);
	const FLT x361 = (x356 * sensor_pt[0]) + (x352 * sensor_pt[2]) + (-1 * x360 * sensor_pt[1]);
	const FLT x362 = (x361 * x159) + (x360 * x133) + (-1 * x345 * x158) + (-1 * x357 * x152);
	const FLT x363 = (x360 * sensor_pt[0]) + (-1 * x345 * sensor_pt[2]) + (x356 * sensor_pt[1]);
	const FLT x364 = (x357 * x154) + (-1 * x363 * x159) + (-1 * x360 * x155) + (x352 * x158);
	const FLT x365 = (x363 * x152) + (-1 * x352 * x133) + (x345 * x155) + (-1 * x361 * x154);
	const FLT x366 = (x365 * (*lh_p).Rot[2]) + (-1 * x362 * (*lh_p).Rot[3]) + (x364 * (*lh_p).Rot[0]);
	const FLT x367 = (x364 * (*lh_p).Rot[3]) + (x362 * (*lh_p).Rot[0]) + (-1 * x365 * (*lh_p).Rot[1]);
	const FLT x368 = x365 + (-1 * x366 * x101) + (x367 * x103);
	const FLT x369 = (x365 * (*lh_p).Rot[0]) + (-1 * x364 * (*lh_p).Rot[2]) + (x362 * (*lh_p).Rot[1]);
	const FLT x370 = (x366 * x107) + x362 + (-1 * x369 * x103);
	const FLT x371 = (-1 * x367 * x107) + x364 + (x369 * x101);
	const FLT x372 = (-1 * x83 * ((x81 * x371) + (-1 * ((x73 * x368) + (x72 * x370)) * x75))) +
					 (-1 * ((x68 * x370) + (-1 * x63 * x368)) * x71);
	const FLT x373 = x9 * x14;
	const FLT x374 = x9 * x9 * x9;
	const FLT x375 = (-1 * x262 * x242) + (-1 * x238 * x374) + (x240 * x373) + (x9 * x236) + (x9 * x241) +
					 (x240 * x374) + (-1 * x238 * x373) + (-1 * x9 * x243);
	const FLT x376 = x262 * x375;
	const FLT x377 = x11 * x253;
	const FLT x378 = x34 * x375;
	const FLT x379 = (-1 * x375 * x342) + (-1 * x249 * x378) + (-1 * x353) + (x264 * x376) + x299 + x355 + x354 +
					 (x259 * x377) + (-1 * x276 * x377) + (-1 * x281 * x375) + (-1 * x262 * x252);
	const FLT x380 = x262 * x285;
	const FLT x381 = x36 * x375;
	const FLT x382 = x247 * x381;
	const FLT x383 = x288 * x375;
	const FLT x384 = (-1 * x289 * x375) + (x256 * x377) + (-1 * x375 * x358) + (x248 * x382) + (-1 * x349) + x350 +
					 (-1 * x34 * x380) + x279 + x278 + (-1 * x262 * x383) + (-1 * x261) + (-1 * x273 * x377);
	const FLT x385 = x359 + x319 + (-1 * x375 * x348) + (-1 * x8 * x380) + (-1 * x262 * x382) + (-1 * x248 * x383) +
					 x317 + (x378 * x301) + (-1 * x293 * x377) + (x290 * x377);
	const FLT x386 = x344 + (x381 * x301) + x302 + (-1 * x269 * x377) + (x376 * x316) + (x265 * x377) + (-1 * x303) +
					 (-1 * x375 * x310) + x305 + (-1 * x39 * x380) + (x375 * x313);
	const FLT x387 = (x386 * sensor_pt[0]) + (-1 * x384 * sensor_pt[1]) + (x385 * sensor_pt[2]);
	const FLT x388 = (x386 * sensor_pt[2]) + (-1 * x385 * sensor_pt[0]) + (x379 * sensor_pt[1]);
	const FLT x389 = (-1 * x388 * x152) + (x387 * x159) + (-1 * x379 * x158) + (x384 * x133);
	const FLT x390 = (x384 * sensor_pt[0]) + (-1 * x379 * sensor_pt[2]) + (x386 * sensor_pt[1]);
	const FLT x391 = (x390 * x152) + (x379 * x155) + (-1 * x385 * x133) + (-1 * x387 * x154);
	const FLT x392 = (-1 * x390 * x159) + (-1 * x384 * x155) + (x385 * x158) + (x388 * x154);
	const FLT x393 = (x392 * (*lh_p).Rot[3]) + (x389 * (*lh_p).Rot[0]) + (-1 * x391 * (*lh_p).Rot[1]);
	const FLT x394 = (x391 * (*lh_p).Rot[2]) + (-1 * x389 * (*lh_p).Rot[3]) + (x392 * (*lh_p).Rot[0]);
	const FLT x395 = x391 + (x393 * x103) + (-1 * x394 * x101);
	const FLT x396 = (x391 * (*lh_p).Rot[0]) + (-1 * x392 * (*lh_p).Rot[2]) + (x389 * (*lh_p).Rot[1]);
	const FLT x397 = x389 + (-1 * x396 * x103) + (x394 * x107);
	const FLT x398 = x392 + (-1 * x393 * x107) + (x396 * x101);
	const FLT x399 = (-1 * x83 * ((x81 * x398) + (-1 * ((x73 * x395) + (x72 * x397)) * x75))) +
					 (-1 * ((x68 * x397) + (-1 * x63 * x395)) * x71);
	const FLT x400 = dt * x102;
	const FLT x401 = dt * x104;
	const FLT x402 = x401 + (-1 * x400);
	const FLT x403 = dt * x106;
	const FLT x404 = dt * x108;
	const FLT x405 = x404 + x403;
	const FLT x406 = -1 * dt * x110;
	const FLT x407 = (-1 * dt * x112) + dt;
	const FLT x408 = x407 + x406;
	const FLT x409 = (-1 * x83 * ((x81 * x408) + (-1 * ((x73 * x402) + (x72 * x405)) * x75))) +
					 (-1 * ((x68 * x405) + (-1 * x63 * x402)) * x71);
	const FLT x410 = dt * x116;
	const FLT x411 = dt * x117;
	const FLT x412 = x411 + x410;
	const FLT x413 = -1 * dt * x119;
	const FLT x414 = x413 + x406 + dt;
	const FLT x415 = x403 + (-1 * x404);
	const FLT x416 = (-1 * x83 * ((x81 * x415) + (-1 * ((x73 * x412) + (x72 * x414)) * x75))) +
					 (-1 * ((x68 * x414) + (-1 * x63 * x412)) * x71);
	const FLT x417 = x407 + x413;
	const FLT x418 = x410 + (-1 * x411);
	const FLT x419 = x400 + x401;
	const FLT x420 = (-1 * x83 * ((x81 * x419) + (-1 * ((x73 * x417) + (x72 * x418)) * x75))) +
					 (-1 * ((x68 * x418) + (-1 * x63 * x417)) * x71);
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT),
						x84 + (x84 * x86) + (x88 * ((x4 * x87) + (-1 * x80 * x68))));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT),
						x96 + (x86 * x96) + (((x87 * x91) + (-1 * x68 * x95)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT),
						(x86 * x100) + x100 + (((x87 * x97) + (-1 * x68 * x99)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT),
						(x86 * x115) + x115 + (((x87 * x105) + (-1 * x68 * x114)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT),
						(x86 * x123) + x123 + (((x87 * x118) + (-1 * x68 * x122)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT),
						x127 + (x86 * x127) + (((x87 * x124) + (-1 * x68 * x126)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						x168 + (x86 * x168) + (((x87 * x164) + (-1 * x68 * x167)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						(x86 * x191) + x191 + (((x87 * x187) + (-1 * x68 * x190)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x212 + (x86 * x212) + (((x87 * x208) + (-1 * x68 * x211)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						x229 + (x86 * x229) + (((x87 * x225) + (-1 * x68 * x228)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x333 + (x86 * x333) + (((x87 * x329) + (-1 * x68 * x332)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x372 + (x86 * x372) + (((x87 * x368) + (-1 * x68 * x371)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x399 + (x86 * x399) + (((x87 * x395) + (-1 * x68 * x398)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT),
						x409 + (x86 * x409) + (((x87 * x402) + (-1 * x68 * x408)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT),
						(x86 * x416) + x416 + (((x87 * x412) + (-1 * x68 * x415)) * x88));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT),
						x420 + (x86 * x420) + (((x87 * x417) + (-1 * x68 * x419)) * x88));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen1 wrt [(*_x0).AccBias[0], (*_x0).AccBias[1],
// (*_x0).AccBias[2], (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1],
// (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2],
// (*_x0).IMUCorrection[3], (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0],
// (*_x0).Pose.Rot[1], (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0],
// (*_x0).Velocity.AxisAngleRot[1], (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1],
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f3beb0>]

static inline void SurviveKalmanErrorModel_LightMeas_y_gen1_jac_x0_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_y_gen1(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_y_gen1_jac_x0(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen1 wrt [(*error_model).AccBias[0], (*error_model).AccBias[1],
// (*error_model).AccBias[2], (*error_model).Acc[0], (*error_model).Acc[1], (*error_model).Acc[2],
// (*error_model).GyroBias[0], (*error_model).GyroBias[1], (*error_model).GyroBias[2], (*error_model).IMUCorrection[0],
// (*error_model).IMUCorrection[1], (*error_model).IMUCorrection[2], (*error_model).IMUCorrection[3],
// (*error_model).Pose.AxisAngleRot[0], (*error_model).Pose.AxisAngleRot[1], (*error_model).Pose.AxisAngleRot[2],
// (*error_model).Pose.Pos[0], (*error_model).Pose.Pos[1], (*error_model).Pose.Pos[2],
// (*error_model).Velocity.AxisAngleRot[0], (*error_model).Velocity.AxisAngleRot[1],
// (*error_model).Velocity.AxisAngleRot[2], (*error_model).Velocity.Pos[0], (*error_model).Velocity.Pos[1],
// (*error_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f31220>]
static inline void SurviveKalmanErrorModel_LightMeas_y_gen1_jac_error_model(
	CnMat *Hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const FLT x0 = dt * fabs(dt);
	const FLT x1 = x0 * (*lh_p).Rot[0];
	const FLT x2 = x1 * (*lh_p).Rot[2];
	const FLT x3 = x0 * (*lh_p).Rot[3];
	const FLT x4 = x3 * (*lh_p).Rot[1];
	const FLT x5 = x4 + (-1 * x2);
	const FLT x6 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x7 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x8 = 0.5 * (*_x0).Pose.Rot[0];
	const FLT x9 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x10 = (-1 * x9 * (*_x0).Pose.Rot[1]) + (x8 * (*error_model).Pose.AxisAngleRot[1]) +
					(x7 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x11 = dt * dt;
	const FLT x12 = x6 * x6;
	const FLT x13 = x12 * x11;
	const FLT x14 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x15 = x14 * x14;
	const FLT x16 = x15 * x11;
	const FLT x17 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x18 = x17 * x17;
	const FLT x19 = x11 * x18;
	const FLT x20 = 1e-10 + x19 + x13 + x16;
	const FLT x21 = sqrt(x20);
	const FLT x22 = 0.5 * x21;
	const FLT x23 = sin(x22);
	const FLT x24 = x23 * x23;
	const FLT x25 = 1. / x20;
	const FLT x26 = x24 * x25;
	const FLT x27 = cos(x22);
	const FLT x28 = (x26 * x16) + (x27 * x27) + (x26 * x13) + (x26 * x19);
	const FLT x29 = 1. / sqrt(x28);
	const FLT x30 = x23 * (1. / x21);
	const FLT x31 = dt * x30;
	const FLT x32 = x31 * x29;
	const FLT x33 = x32 * x10;
	const FLT x34 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x35 = (x34 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x7 * (*_x0).Pose.Rot[2]) +
					(x8 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x36 = x32 * x35;
	const FLT x37 = (x8 * (*error_model).Pose.AxisAngleRot[0]) + (-1 * x34 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[1] +
					(x9 * (*_x0).Pose.Rot[2]);
	const FLT x38 = x29 * x27;
	const FLT x39 = x38 * x37;
	const FLT x40 = (-1 * x7 * (*_x0).Pose.Rot[1]) + (-1 * x34 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x9 * (*_x0).Pose.Rot[3]);
	const FLT x41 = x40 * x32;
	const FLT x42 = (x41 * x14) + x39 + (-1 * x6 * x33) + (x36 * x17);
	const FLT x43 = x40 * x38;
	const FLT x44 = x37 * x29;
	const FLT x45 = x44 * x31;
	const FLT x46 = (-1 * x45 * x14) + (-1 * x6 * x36) + (-1 * x33 * x17) + x43;
	const FLT x47 = x38 * x10;
	const FLT x48 = (-1 * x36 * x14) + x47 + (x6 * x45) + (x41 * x17);
	const FLT x49 = (x42 * sensor_pt[1]) + (-1 * x48 * sensor_pt[0]) + (x46 * sensor_pt[2]);
	const FLT x50 = x35 * x38;
	const FLT x51 = (x33 * x14) + x50 + (-1 * x45 * x17) + (x6 * x41);
	const FLT x52 = (-1 * x51 * sensor_pt[1]) + (x48 * sensor_pt[2]) + (x46 * sensor_pt[0]);
	const FLT x53 = 1.0 / 2.0 * x0;
	const FLT x54 = (*_x0).Pose.Pos[1] + (*error_model).Pose.Pos[1] + (2 * ((x52 * x51) + (-1 * x42 * x49))) +
					sensor_pt[1] + (dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) +
					(x53 * ((*_x0).Acc[1] + (*error_model).Acc[1]));
	const FLT x55 = (-1 * x42 * sensor_pt[2]) + (x46 * sensor_pt[1]) + (x51 * sensor_pt[0]);
	const FLT x56 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (2 * ((x55 * x42) + (-1 * x52 * x48))) +
					(x53 * ((*_x0).Acc[2] + (*error_model).Acc[2])) + (*error_model).Pose.Pos[2] +
					(dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2]));
	const FLT x57 = (x53 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + (2 * ((x48 * x49) + (-1 * x51 * x55))) +
					(*error_model).Pose.Pos[0] + sensor_pt[0] +
					(dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) + (*_x0).Pose.Pos[0];
	const FLT x58 = (-1 * x57 * (*lh_p).Rot[2]) + (x54 * (*lh_p).Rot[1]) + (x56 * (*lh_p).Rot[0]);
	const FLT x59 = (-1 * x54 * (*lh_p).Rot[3]) + (x57 * (*lh_p).Rot[0]) + (x56 * (*lh_p).Rot[2]);
	const FLT x60 = x54 + (*lh_p).Pos[1] + (2 * ((x59 * (*lh_p).Rot[3]) + (-1 * x58 * (*lh_p).Rot[1])));
	const FLT x61 = (x54 * (*lh_p).Rot[0]) + (-1 * x56 * (*lh_p).Rot[1]) + (x57 * (*lh_p).Rot[3]);
	const FLT x62 = x56 + (2 * ((x61 * (*lh_p).Rot[1]) + (-1 * x59 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x63 = x62 * x62;
	const FLT x64 = 1. / x63;
	const FLT x65 = x60 * x64;
	const FLT x66 = x0 * (*lh_p).Rot[1] * (*lh_p).Rot[2];
	const FLT x67 = x3 * (*lh_p).Rot[0];
	const FLT x68 = x67 + x66;
	const FLT x69 = 1. / x62;
	const FLT x70 = x63 + (x60 * x60);
	const FLT x71 = 1. / x70;
	const FLT x72 = x71 * x63;
	const FLT x73 = 2 * x60;
	const FLT x74 = 2 * x62;
	const FLT x75 = x57 + (*lh_p).Pos[0] + (2 * ((x58 * (*lh_p).Rot[2]) + (-1 * x61 * (*lh_p).Rot[3])));
	const FLT x76 = 1.0 / 2.0 * (1. / (x70 * sqrt(x70))) * x75 * (*bsc0).tilt;
	const FLT x77 = (*lh_p).Rot[3] * (*lh_p).Rot[3];
	const FLT x78 = -1 * x0 * x77;
	const FLT x79 = (*lh_p).Rot[2] * (*lh_p).Rot[2];
	const FLT x80 = (-1 * x0 * x79) + x53;
	const FLT x81 = x80 + x78;
	const FLT x82 = (1. / sqrt(x70)) * (*bsc0).tilt;
	const FLT x83 = x75 * x75;
	const FLT x84 = 1. / sqrt(1 + (-1 * x83 * x71 * ((*bsc0).tilt * (*bsc0).tilt)));
	const FLT x85 = (-1 * x84 * ((x81 * x82) + (-1 * x76 * ((x5 * x74) + (x73 * x68))))) +
					(-1 * x72 * ((x68 * x69) + (-1 * x5 * x65)));
	const FLT x86 = -1 * x62;
	const FLT x87 = (*bsc0).gibmag * sin(1.5707963267949 + (-1 * asin(x82 * x75)) + (-1 * (*bsc0).phase) +
										 (*bsc0).gibpha + (-1 * atan2(-1 * x60, x86)));
	const FLT x88 = x75 * x64;
	const FLT x89 = 2 * (1. / (x63 + x83)) * x63 * atan2(x75, x86) * (*bsc0).curve;
	const FLT x90 = x3 * (*lh_p).Rot[2];
	const FLT x91 = x1 * (*lh_p).Rot[1];
	const FLT x92 = x91 + x90;
	const FLT x93 = (*lh_p).Rot[1] * (*lh_p).Rot[1];
	const FLT x94 = -1 * x0 * x93;
	const FLT x95 = x94 + x78 + x53;
	const FLT x96 = x66 + (-1 * x67);
	const FLT x97 = (-1 * x84 * ((x82 * x96) + (-1 * ((x74 * x92) + (x73 * x95)) * x76))) +
					(-1 * ((x69 * x95) + (-1 * x65 * x92)) * x72);
	const FLT x98 = x80 + x94;
	const FLT x99 = x90 + (-1 * x91);
	const FLT x100 = x2 + x4;
	const FLT x101 = (-1 * x84 * ((x82 * x100) + (-1 * ((x74 * x98) + (x73 * x99)) * x76))) +
					 (-1 * ((x69 * x99) + (-1 * x65 * x98)) * x72);
	const FLT x102 = x32 * x14;
	const FLT x103 = 0.5 * x102;
	const FLT x104 = x6 * x32;
	const FLT x105 = 0.5 * x104;
	const FLT x106 = x32 * x17;
	const FLT x107 = 0.5 * x106;
	const FLT x108 = (-1 * x105 * (*_x0).Pose.Rot[3]) + (-1 * x103 * (*_x0).Pose.Rot[1]) +
					 (-1 * x107 * (*_x0).Pose.Rot[2]) + (x8 * x38);
	const FLT x109 = 2 * x49;
	const FLT x110 = x109 * x108;
	const FLT x111 = x103 * (*_x0).Pose.Rot[3];
	const FLT x112 = x105 * (*_x0).Pose.Rot[1];
	const FLT x113 = x8 * x106;
	const FLT x114 = 0.5 * x38;
	const FLT x115 = x114 * (*_x0).Pose.Rot[2];
	const FLT x116 = (-1 * x115) + (-1 * x113) + x111 + (-1 * x112);
	const FLT x117 = 2 * x52;
	const FLT x118 = x103 * (*_x0).Pose.Rot[2];
	const FLT x119 = x114 * (*_x0).Pose.Rot[3];
	const FLT x120 = x8 * x104;
	const FLT x121 = x107 * (*_x0).Pose.Rot[1];
	const FLT x122 = (-1 * x121) + x120 + x118 + x119;
	const FLT x123 = x116 * sensor_pt[1];
	const FLT x124 = x8 * x102;
	const FLT x125 = x105 * (*_x0).Pose.Rot[2];
	const FLT x126 = x107 * (*_x0).Pose.Rot[3];
	const FLT x127 = x114 * (*_x0).Pose.Rot[1];
	const FLT x128 = (-1 * x127) + (-1 * x126) + (-1 * x124) + x125;
	const FLT x129 = x128 * sensor_pt[0];
	const FLT x130 = x129 + (x122 * sensor_pt[2]) + (-1 * x123);
	const FLT x131 = 2 * x51;
	const FLT x132 = x108 * sensor_pt[1];
	const FLT x133 = x128 * sensor_pt[2];
	const FLT x134 = x133 + (-1 * x122 * sensor_pt[0]) + x132;
	const FLT x135 = 2 * x42;
	const FLT x136 = (x130 * x131) + (-1 * x134 * x135) + (-1 * x110) + (x116 * x117);
	const FLT x137 = 2 * x55;
	const FLT x138 = x108 * sensor_pt[2];
	const FLT x139 = x116 * sensor_pt[0];
	const FLT x140 = x139 + (-1 * x138) + (x128 * sensor_pt[1]);
	const FLT x141 = 2 * x48;
	const FLT x142 = (x134 * x141) + (x109 * x122) + (-1 * x116 * x137) + (-1 * x131 * x140);
	const FLT x143 = x108 * x137;
	const FLT x144 = (x135 * x140) + (-1 * x130 * x141) + x143 + (-1 * x117 * x122);
	const FLT x145 = (x144 * (*lh_p).Rot[2]) + (-1 * x136 * (*lh_p).Rot[3]) + (x142 * (*lh_p).Rot[0]);
	const FLT x146 = 2 * (*lh_p).Rot[2];
	const FLT x147 = (-1 * x144 * (*lh_p).Rot[1]) + (x142 * (*lh_p).Rot[3]) + (x136 * (*lh_p).Rot[0]);
	const FLT x148 = 2 * (*lh_p).Rot[1];
	const FLT x149 = x144 + (-1 * x146 * x145) + (x148 * x147);
	const FLT x150 = 2 * ((x144 * (*lh_p).Rot[0]) + (-1 * x142 * (*lh_p).Rot[2]) + (x136 * (*lh_p).Rot[1]));
	const FLT x151 = 2 * (*lh_p).Rot[3];
	const FLT x152 = (-1 * x150 * (*lh_p).Rot[1]) + x136 + (x145 * x151);
	const FLT x153 = x142 + (-1 * x147 * x151) + (x150 * (*lh_p).Rot[2]);
	const FLT x154 = (-1 * x84 * ((x82 * x153) + (-1 * ((x74 * x149) + (x73 * x152)) * x76))) +
					 (-1 * ((x69 * x152) + (-1 * x65 * x149)) * x72);
	const FLT x155 = (-1 * x119) + (-1 * x118) + (-1 * x120) + x121;
	const FLT x156 = (-1 * x125) + x126 + x124 + x127;
	const FLT x157 = (-1 * x156 * sensor_pt[1]) + x139 + x138;
	const FLT x158 = x108 * sensor_pt[0];
	const FLT x159 = x155 * sensor_pt[1];
	const FLT x160 = (-1 * x158) + (x116 * sensor_pt[2]) + x159;
	const FLT x161 = (-1 * x160 * x135) + (x131 * x157) + (-1 * x109 * x155) + (x117 * x156);
	const FLT x162 = x155 * sensor_pt[2];
	const FLT x163 = (x156 * sensor_pt[0]) + (-1 * x162) + x123;
	const FLT x164 = (x160 * x141) + x110 + (-1 * x137 * x156) + (-1 * x163 * x131);
	const FLT x165 = x108 * x117;
	const FLT x166 = (x163 * x135) + (-1 * x141 * x157) + (-1 * x165) + (x137 * x155);
	const FLT x167 = (-1 * x161 * (*lh_p).Rot[3]) + (x166 * (*lh_p).Rot[2]) + (x164 * (*lh_p).Rot[0]);
	const FLT x168 = (x164 * (*lh_p).Rot[3]) + (-1 * x166 * (*lh_p).Rot[1]) + (x161 * (*lh_p).Rot[0]);
	const FLT x169 = x166 + (-1 * x167 * x146) + (x168 * x148);
	const FLT x170 = (x166 * (*lh_p).Rot[0]) + (-1 * x164 * (*lh_p).Rot[2]) + (x161 * (*lh_p).Rot[1]);
	const FLT x171 = (-1 * x170 * x148) + x161 + (x167 * x151);
	const FLT x172 = x164 + (-1 * x168 * x151) + (x170 * x146);
	const FLT x173 = (-1 * x84 * ((x82 * x172) + (-1 * ((x74 * x169) + (x73 * x171)) * x76))) +
					 (-1 * ((x69 * x171) + (-1 * x65 * x169)) * x72);
	const FLT x174 = x113 + x115 + (-1 * x111) + x112;
	const FLT x175 = (-1 * x132) + (x155 * sensor_pt[0]) + x133;
	const FLT x176 = x162 + (-1 * x129) + (x174 * sensor_pt[1]);
	const FLT x177 = x165 + (-1 * x176 * x135) + (-1 * x109 * x174) + (x175 * x131);
	const FLT x178 = x158 + (-1 * x174 * sensor_pt[2]) + x159;
	const FLT x179 = (x176 * x141) + (x109 * x128) + (-1 * x143) + (-1 * x178 * x131);
	const FLT x180 = (x174 * x137) + (x178 * x135) + (-1 * x117 * x128) + (-1 * x175 * x141);
	const FLT x181 = (x180 * (*lh_p).Rot[2]) + (-1 * x177 * (*lh_p).Rot[3]) + (x179 * (*lh_p).Rot[0]);
	const FLT x182 = 2 * ((-1 * x180 * (*lh_p).Rot[1]) + (x179 * (*lh_p).Rot[3]) + (x177 * (*lh_p).Rot[0]));
	const FLT x183 = x180 + (-1 * x181 * x146) + (x182 * (*lh_p).Rot[1]);
	const FLT x184 = (-1 * x179 * (*lh_p).Rot[2]) + (x180 * (*lh_p).Rot[0]) + (x177 * (*lh_p).Rot[1]);
	const FLT x185 = (-1 * x184 * x148) + x177 + (x181 * x151);
	const FLT x186 = (-1 * x182 * (*lh_p).Rot[3]) + x179 + (x184 * x146);
	const FLT x187 = (-1 * x84 * ((x82 * x186) + (-1 * ((x74 * x183) + (x73 * x185)) * x76))) +
					 (-1 * ((x69 * x185) + (-1 * x65 * x183)) * x72);
	const FLT x188 = x146 * (*lh_p).Rot[0];
	const FLT x189 = x148 * (*lh_p).Rot[3];
	const FLT x190 = x189 + (-1 * x188);
	const FLT x191 = x146 * (*lh_p).Rot[1];
	const FLT x192 = x151 * (*lh_p).Rot[0];
	const FLT x193 = x192 + x191;
	const FLT x194 = 2 * x79;
	const FLT x195 = -1 * x194;
	const FLT x196 = 2 * x77;
	const FLT x197 = 1 + (-1 * x196);
	const FLT x198 = x197 + x195;
	const FLT x199 = (-1 * x84 * ((x82 * x198) + (-1 * ((x74 * x190) + (x73 * x193)) * x76))) +
					 (-1 * ((x69 * x193) + (-1 * x65 * x190)) * x72);
	const FLT x200 = x146 * (*lh_p).Rot[3];
	const FLT x201 = x148 * (*lh_p).Rot[0];
	const FLT x202 = x201 + x200;
	const FLT x203 = 2 * x93;
	const FLT x204 = -1 * x203;
	const FLT x205 = x197 + x204;
	const FLT x206 = x191 + (-1 * x192);
	const FLT x207 = (-1 * x84 * ((x82 * x206) + (-1 * ((x74 * x202) + (x73 * x205)) * x76))) +
					 (-1 * ((x69 * x205) + (-1 * x65 * x202)) * x72);
	const FLT x208 = 1 + x204 + x195;
	const FLT x209 = x200 + (-1 * x201);
	const FLT x210 = x188 + x189;
	const FLT x211 = (-1 * x84 * ((x82 * x210) + (-1 * ((x74 * x208) + (x73 * x209)) * x76))) +
					 (-1 * ((x69 * x209) + (-1 * x65 * x208)) * x72);
	const FLT x212 = x14 * x14 * x14;
	const FLT x213 = dt * dt * dt * dt;
	const FLT x214 = 1.0 * x27;
	const FLT x215 = x23 * (1. / (x20 * sqrt(x20)));
	const FLT x216 = x215 * x214;
	const FLT x217 = x213 * x216;
	const FLT x218 = 2 * x26;
	const FLT x219 = x11 * x218;
	const FLT x220 = 2 * x24 * (1. / (x20 * x20));
	const FLT x221 = x213 * x220;
	const FLT x222 = x12 * x221;
	const FLT x223 = x12 * x217;
	const FLT x224 = x18 * x217;
	const FLT x225 = x30 * x11;
	const FLT x226 = x214 * x225;
	const FLT x227 = x18 * x221;
	const FLT x228 = (-1 * x14 * x226) + (x14 * x224) + (x14 * x219) + (-1 * x14 * x227) + (x212 * x217) +
					 (-1 * x212 * x221) + (-1 * x14 * x222) + (x14 * x223);
	const FLT x229 = x35 * x228;
	const FLT x230 = 1.0 / 2.0 * (1. / (x28 * sqrt(x28)));
	const FLT x231 = x31 * x230;
	const FLT x232 = x17 * x231;
	const FLT x233 = 0.5 * x44;
	const FLT x234 = x233 * x225;
	const FLT x235 = dt * dt * dt;
	const FLT x236 = x15 * x235;
	const FLT x237 = 0.5 * x25;
	const FLT x238 = x43 * x237;
	const FLT x239 = x29 * x10;
	const FLT x240 = x215 * x235;
	const FLT x241 = x6 * x240;
	const FLT x242 = x14 * x241;
	const FLT x243 = x239 * x242;
	const FLT x244 = x6 * x231;
	const FLT x245 = x10 * x228;
	const FLT x246 = x14 * x240;
	const FLT x247 = x35 * x29;
	const FLT x248 = x17 * x247;
	const FLT x249 = x246 * x248;
	const FLT x250 = x50 * x237;
	const FLT x251 = x235 * x250;
	const FLT x252 = x17 * x251;
	const FLT x253 = x14 * x252;
	const FLT x254 = x40 * x29;
	const FLT x255 = x215 * x236;
	const FLT x256 = x27 * x230;
	const FLT x257 = x37 * x256;
	const FLT x258 = x6 * x14;
	const FLT x259 = x47 * x237;
	const FLT x260 = x235 * x259;
	const FLT x261 = x260 * x258;
	const FLT x262 = x14 * x231;
	const FLT x263 = x40 * x228;
	const FLT x264 = (x244 * x245) + (-1 * x14 * x234) + x253 + x243 + (-1 * x262 * x263) + (-1 * x249) +
					 (-1 * x232 * x229) + (-1 * x257 * x228) + (-1 * x254 * x255) + x41 + (x238 * x236) + (-1 * x261);
	const FLT x265 = 0.5 * x247;
	const FLT x266 = x265 * x225;
	const FLT x267 = x37 * x231;
	const FLT x268 = x17 * x267;
	const FLT x269 = x35 * x256;
	const FLT x270 = x39 * x237;
	const FLT x271 = x235 * x270;
	const FLT x272 = x17 * x271;
	const FLT x273 = x44 * x17;
	const FLT x274 = (x273 * x246) + (-1 * x14 * x272);
	const FLT x275 = x254 * x240;
	const FLT x276 = x238 * x235;
	const FLT x277 = (x276 * x258) + (-1 * x275 * x258);
	const FLT x278 = x277 + x274 + (-1 * x263 * x244) + x33 + (-1 * x14 * x266) + (-1 * x262 * x245) + (x236 * x259) +
					 (x268 * x228) + (-1 * x239 * x255) + (-1 * x269 * x228);
	const FLT x279 = -1 * x36;
	const FLT x280 = x44 * x242;
	const FLT x281 = x10 * x256;
	const FLT x282 = x271 * x258;
	const FLT x283 = x37 * x244;
	const FLT x284 = 0.5 * x239;
	const FLT x285 = x284 * x225;
	const FLT x286 = x17 * x276;
	const FLT x287 = x17 * x275;
	const FLT x288 = (-1 * x14 * x287) + (x14 * x286);
	const FLT x289 = (-1 * x281 * x228) + (-1 * x236 * x250) + (-1 * x14 * x285) + x279 + x288 + (x255 * x247) +
					 (-1 * x283 * x228) + (-1 * x232 * x263) + (x262 * x229) + x282 + (-1 * x280);
	const FLT x290 = x40 * x256;
	const FLT x291 = x17 * x239;
	const FLT x292 = x291 * x246;
	const FLT x293 = x17 * x260;
	const FLT x294 = x14 * x293;
	const FLT x295 = 0.5 * x254;
	const FLT x296 = x295 * x225;
	const FLT x297 = -1 * x45;
	const FLT x298 = x14 * x267;
	const FLT x299 = (x242 * x247) + (-1 * x251 * x258);
	const FLT x300 = x299 + (x298 * x228) + (x44 * x255) + x297 + (-1 * x14 * x296) + (x229 * x244) +
					 (-1 * x290 * x228) + (-1 * x236 * x270) + (x232 * x245) + x292 + (-1 * x294);
	const FLT x301 = (x300 * sensor_pt[0]) + (-1 * x278 * sensor_pt[1]) + (x289 * sensor_pt[2]);
	const FLT x302 = (x300 * sensor_pt[2]) + (-1 * x289 * sensor_pt[0]) + (x264 * sensor_pt[1]);
	const FLT x303 = (x278 * x117) + (-1 * x302 * x135) + (-1 * x264 * x109) + (x301 * x131);
	const FLT x304 = (x278 * sensor_pt[0]) + (-1 * x264 * sensor_pt[2]) + (x300 * sensor_pt[1]);
	const FLT x305 = (x289 * x109) + (-1 * x304 * x131) + (-1 * x278 * x137) + (x302 * x141);
	const FLT x306 = (-1 * x301 * x141) + (x304 * x135) + (-1 * x289 * x117) + (x264 * x137);
	const FLT x307 = (x306 * (*lh_p).Rot[2]) + (-1 * x303 * (*lh_p).Rot[3]) + (x305 * (*lh_p).Rot[0]);
	const FLT x308 = (x305 * (*lh_p).Rot[3]) + (-1 * x306 * (*lh_p).Rot[1]) + (x303 * (*lh_p).Rot[0]);
	const FLT x309 = x306 + (-1 * x307 * x146) + (x308 * x148);
	const FLT x310 = (-1 * x305 * (*lh_p).Rot[2]) + (x306 * (*lh_p).Rot[0]) + (x303 * (*lh_p).Rot[1]);
	const FLT x311 = x303 + (-1 * x310 * x148) + (x307 * x151);
	const FLT x312 = x305 + (-1 * x308 * x151) + (x310 * x146);
	const FLT x313 = (-1 * x84 * ((x82 * x312) + (-1 * ((x74 * x309) + (x73 * x311)) * x76))) +
					 (-1 * ((x69 * x311) + (-1 * x65 * x309)) * x72);
	const FLT x314 = x18 * x240;
	const FLT x315 = x15 * x217;
	const FLT x316 = x15 * x221;
	const FLT x317 = (x17 * x17 * x17) * x213;
	const FLT x318 = (x17 * x219) + (-1 * x220 * x317) + (x216 * x317) + (-1 * x17 * x316) + (-1 * x17 * x222) +
					 (x17 * x223) + (x17 * x315) + (-1 * x17 * x226);
	const FLT x319 = x244 * x318;
	const FLT x320 = x35 * x318;
	const FLT x321 = x18 * x235;
	const FLT x322 = x40 * x318;
	const FLT x323 = (-1 * x6 * x293) + (x291 * x241);
	const FLT x324 = x288 + (-1 * x262 * x322) + x323 + x36 + (x10 * x319) + (-1 * x247 * x314) + (-1 * x232 * x320) +
					 (-1 * x17 * x234) + (x250 * x321) + (-1 * x257 * x318);
	const FLT x325 = x6 * x272;
	const FLT x326 = x273 * x241;
	const FLT x327 = (-1 * x17 * x285) + (-1 * x253) + (-1 * x232 * x322) + x41 + (-1 * x326) + (-1 * x18 * x275) +
					 (-1 * x281 * x318) + (x262 * x320) + (-1 * x37 * x319) + x249 + x325 + (x238 * x321);
	const FLT x328 = x241 * x248;
	const FLT x329 = x10 * x318;
	const FLT x330 = x6 * x252;
	const FLT x331 = -1 * x33;
	const FLT x332 = x274 + (x298 * x318) + x331 + (-1 * x290 * x318) + (-1 * x17 * x296) + x328 + (x35 * x319) +
					 (x239 * x314) + (-1 * x259 * x321) + (x232 * x329) + (-1 * x330);
	const FLT x333 = (x332 * sensor_pt[2]) + (-1 * x327 * sensor_pt[0]) + (x324 * sensor_pt[1]);
	const FLT x334 = (x6 * x286) + (-1 * x6 * x287);
	const FLT x335 = x334 + (-1 * x17 * x266) + (-1 * x269 * x318) + (x44 * x314) + x294 + x297 + (-1 * x262 * x329) +
					 (-1 * x270 * x321) + (-1 * x292) + (-1 * x40 * x319) + (x268 * x318);
	const FLT x336 = (x332 * sensor_pt[0]) + (x327 * sensor_pt[2]) + (-1 * x335 * sensor_pt[1]);
	const FLT x337 = (x336 * x131) + (x335 * x117) + (-1 * x324 * x109) + (-1 * x333 * x135);
	const FLT x338 = (x335 * sensor_pt[0]) + (-1 * x324 * sensor_pt[2]) + (x332 * sensor_pt[1]);
	const FLT x339 = (x333 * x141) + (-1 * x338 * x131) + (-1 * x335 * x137) + (x327 * x109);
	const FLT x340 = (x338 * x135) + (-1 * x327 * x117) + (x324 * x137) + (-1 * x336 * x141);
	const FLT x341 = (x340 * (*lh_p).Rot[2]) + (-1 * x337 * (*lh_p).Rot[3]) + (x339 * (*lh_p).Rot[0]);
	const FLT x342 = (x339 * (*lh_p).Rot[3]) + (x337 * (*lh_p).Rot[0]) + (-1 * x340 * (*lh_p).Rot[1]);
	const FLT x343 = x340 + (-1 * x341 * x146) + (x342 * x148);
	const FLT x344 = (x340 * (*lh_p).Rot[0]) + (-1 * x339 * (*lh_p).Rot[2]) + (x337 * (*lh_p).Rot[1]);
	const FLT x345 = (x341 * x151) + x337 + (-1 * x344 * x148);
	const FLT x346 = x339 + (-1 * x342 * x151) + (x344 * x146);
	const FLT x347 = (-1 * x84 * ((x82 * x346) + (-1 * ((x74 * x343) + (x73 * x345)) * x76))) +
					 (-1 * ((x69 * x345) + (-1 * x65 * x343)) * x72);
	const FLT x348 = x6 * x6 * x6;
	const FLT x349 = x6 * x11;
	const FLT x350 = x30 * x349;
	const FLT x351 = (-1 * x214 * x350) + (-1 * x221 * x348) + (x6 * x224) + (x218 * x349) + (x217 * x348) +
					 (x6 * x315) + (-1 * x6 * x227) + (-1 * x6 * x316);
	const FLT x352 = x10 * x351;
	const FLT x353 = x12 * x235;
	const FLT x354 = x35 * x351;
	const FLT x355 = x12 * x240;
	const FLT x356 = x40 * x351;
	const FLT x357 = (-1 * x262 * x356) + (x239 * x355) + (-1 * x328) + x277 + (-1 * x257 * x351) + (-1 * x232 * x354) +
					 x331 + (x244 * x352) + x330 + (-1 * x259 * x353) + (-1 * x233 * x350);
	const FLT x358 = (-1 * x262 * x352) + (x268 * x351) + (-1 * x265 * x350) + x326 + (-1 * x243) + (x238 * x353) +
					 x41 + x261 + (-1 * x244 * x356) + (-1 * x325) + (-1 * x12 * x275) + (-1 * x269 * x351);
	const FLT x359 = x334 + x299 + (x262 * x354) + (-1 * x281 * x351) + (-1 * x284 * x350) + (-1 * x283 * x351) +
					 (-1 * x44 * x355) + x45 + (x270 * x353) + (-1 * x232 * x356);
	const FLT x360 = x323 + (x298 * x351) + (-1 * x250 * x353) + (x244 * x354) + x280 + (-1 * x282) + (x247 * x355) +
					 (x232 * x352) + (-1 * x295 * x350) + x279 + (-1 * x290 * x351);
	const FLT x361 = (x360 * sensor_pt[0]) + (-1 * x358 * sensor_pt[1]) + (x359 * sensor_pt[2]);
	const FLT x362 = (x360 * sensor_pt[2]) + (-1 * x359 * sensor_pt[0]) + (x357 * sensor_pt[1]);
	const FLT x363 = (-1 * x362 * x135) + (x361 * x131) + (-1 * x357 * x109) + (x358 * x117);
	const FLT x364 = (x358 * sensor_pt[0]) + (-1 * x357 * sensor_pt[2]) + (x360 * sensor_pt[1]);
	const FLT x365 = (x364 * x135) + (x357 * x137) + (-1 * x359 * x117) + (-1 * x361 * x141);
	const FLT x366 = (-1 * x364 * x131) + (x359 * x109) + (-1 * x358 * x137) + (x362 * x141);
	const FLT x367 = (x366 * (*lh_p).Rot[3]) + (x363 * (*lh_p).Rot[0]) + (-1 * x365 * (*lh_p).Rot[1]);
	const FLT x368 = (-1 * x363 * (*lh_p).Rot[3]) + (x365 * (*lh_p).Rot[2]) + (x366 * (*lh_p).Rot[0]);
	const FLT x369 = (x367 * x148) + x365 + (-1 * x368 * x146);
	const FLT x370 = (x365 * (*lh_p).Rot[0]) + (-1 * x366 * (*lh_p).Rot[2]) + (x363 * (*lh_p).Rot[1]);
	const FLT x371 = x363 + (-1 * x370 * x148) + (x368 * x151);
	const FLT x372 = x366 + (-1 * x367 * x151) + (x370 * x146);
	const FLT x373 = (-1 * x84 * ((x82 * x372) + (-1 * ((x74 * x369) + (x73 * x371)) * x76))) +
					 (-1 * ((x69 * x371) + (-1 * x65 * x369)) * x72);
	const FLT x374 = dt * x188;
	const FLT x375 = dt * x189;
	const FLT x376 = x375 + (-1 * x374);
	const FLT x377 = dt * x191;
	const FLT x378 = dt * x192;
	const FLT x379 = x378 + x377;
	const FLT x380 = -1 * dt * x194;
	const FLT x381 = dt + (-1 * dt * x196);
	const FLT x382 = x381 + x380;
	const FLT x383 = (-1 * x84 * ((x82 * x382) + (-1 * ((x74 * x376) + (x73 * x379)) * x76))) +
					 (-1 * ((x69 * x379) + (-1 * x65 * x376)) * x72);
	const FLT x384 = dt * x200;
	const FLT x385 = dt * x201;
	const FLT x386 = x385 + x384;
	const FLT x387 = -1 * dt * x203;
	const FLT x388 = x381 + x387;
	const FLT x389 = x377 + (-1 * x378);
	const FLT x390 = (-1 * x84 * ((x82 * x389) + (-1 * ((x74 * x386) + (x73 * x388)) * x76))) +
					 (-1 * ((x69 * x388) + (-1 * x65 * x386)) * x72);
	const FLT x391 = x380 + dt + x387;
	const FLT x392 = x384 + (-1 * x385);
	const FLT x393 = x374 + x375;
	const FLT x394 = (-1 * x84 * ((x82 * x393) + (-1 * ((x74 * x391) + (x73 * x392)) * x76))) +
					 (-1 * ((x69 * x392) + (-1 * x65 * x391)) * x72);
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Acc[0]) / sizeof(FLT),
						x85 + (x85 * x87) + (x89 * ((x5 * x88) + (-1 * x81 * x69))));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Acc[1]) / sizeof(FLT),
						x97 + (x87 * x97) + (((x88 * x92) + (-1 * x69 * x96)) * x89));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Acc[2]) / sizeof(FLT),
						x101 + (x87 * x101) + (x89 * ((x88 * x98) + (-1 * x69 * x100))));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						x154 + (x87 * x154) + (((x88 * x149) + (-1 * x69 * x153)) * x89));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						x173 + (x87 * x173) + (((x88 * x169) + (-1 * x69 * x172)) * x89));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						x187 + (x87 * x187) + (((x88 * x183) + (-1 * x69 * x186)) * x89));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT),
						x199 + (x87 * x199) + (((x88 * x190) + (-1 * x69 * x198)) * x89));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT),
						x207 + (x87 * x207) + (((x88 * x202) + (-1 * x69 * x206)) * x89));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT),
						x211 + (x87 * x211) + (((x88 * x208) + (-1 * x69 * x210)) * x89));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x313 + (x87 * x313) + (((x88 * x309) + (-1 * x69 * x312)) * x89));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x347 + (x87 * x347) + (((x88 * x343) + (-1 * x69 * x346)) * x89));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						(x87 * x373) + x373 + (((x88 * x369) + (-1 * x69 * x372)) * x89));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.Pos[0]) / sizeof(FLT),
						x383 + (x87 * x383) + (((x88 * x376) + (-1 * x69 * x382)) * x89));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.Pos[1]) / sizeof(FLT),
						x390 + (x87 * x390) + (((x88 * x386) + (-1 * x69 * x389)) * x89));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.Pos[2]) / sizeof(FLT),
						x394 + (x87 * x394) + (((x88 * x391) + (-1 * x69 * x393)) * x89));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen1 wrt [(*error_model).AccBias[0],
// (*error_model).AccBias[1], (*error_model).AccBias[2], (*error_model).Acc[0], (*error_model).Acc[1],
// (*error_model).Acc[2], (*error_model).GyroBias[0], (*error_model).GyroBias[1], (*error_model).GyroBias[2],
// (*error_model).IMUCorrection[0], (*error_model).IMUCorrection[1], (*error_model).IMUCorrection[2],
// (*error_model).IMUCorrection[3], (*error_model).Pose.AxisAngleRot[0], (*error_model).Pose.AxisAngleRot[1],
// (*error_model).Pose.AxisAngleRot[2], (*error_model).Pose.Pos[0], (*error_model).Pose.Pos[1],
// (*error_model).Pose.Pos[2], (*error_model).Velocity.AxisAngleRot[0], (*error_model).Velocity.AxisAngleRot[1],
// (*error_model).Velocity.AxisAngleRot[2], (*error_model).Velocity.Pos[0], (*error_model).Velocity.Pos[1],
// (*error_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f31220>]

static inline void SurviveKalmanErrorModel_LightMeas_y_gen1_jac_error_model_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_y_gen1(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_y_gen1_jac_error_model(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen1 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]
static inline void SurviveKalmanErrorModel_LightMeas_y_gen1_jac_sensor_pt(CnMat *Hx, const FLT dt,
																		  const SurviveKalmanModel *_x0,
																		  const SurviveKalmanErrorModel *error_model,
																		  const FLT *sensor_pt, const SurvivePose *lh_p,
																		  const BaseStationCal *bsc0) {
	const FLT x0 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x1 = 0.5 * (*_x0).Pose.Rot[3];
	const FLT x2 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x4 = (-1 * x3 * (*_x0).Pose.Rot[1]) + (x1 * (*error_model).Pose.AxisAngleRot[0]) +
				   (x2 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2];
	const FLT x5 = dt * dt;
	const FLT x6 = (x0 * x0) * x5;
	const FLT x7 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x8 = x5 * (x7 * x7);
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x10 = x5 * (x9 * x9);
	const FLT x11 = 1e-10 + x10 + x6 + x8;
	const FLT x12 = sqrt(x11);
	const FLT x13 = 0.5 * x12;
	const FLT x14 = sin(x13);
	const FLT x15 = (x14 * x14) * (1. / x11);
	const FLT x16 = cos(x13);
	const FLT x17 = 1. / sqrt((x8 * x15) + (x16 * x16) + (x6 * x15) + (x15 * x10));
	const FLT x18 = dt * x14 * (1. / x12) * x17;
	const FLT x19 = x4 * x18;
	const FLT x20 = x0 * x19;
	const FLT x21 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x22 =
		(x2 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x21 * (*_x0).Pose.Rot[2]) + (x3 * (*_x0).Pose.Rot[0]);
	const FLT x23 = x22 * x18;
	const FLT x24 = x9 * x23;
	const FLT x25 = (*_x0).Pose.Rot[1] + (-1 * x1 * (*error_model).Pose.AxisAngleRot[1]) + (x21 * (*_x0).Pose.Rot[0]) +
					(x3 * (*_x0).Pose.Rot[2]);
	const FLT x26 = x17 * x16;
	const FLT x27 = x25 * x26;
	const FLT x28 = (-1 * x21 * (*_x0).Pose.Rot[1]) + (-1 * x2 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x1 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x29 = x28 * x18;
	const FLT x30 = x7 * x29;
	const FLT x31 = (-1 * x20) + x30 + x27 + x24;
	const FLT x32 = x25 * x18;
	const FLT x33 = (-1 * x7 * x32) + (-1 * x0 * x23) + (-1 * x9 * x19) + (x28 * x26);
	const FLT x34 = x0 * x32;
	const FLT x35 = x9 * x29;
	const FLT x36 = x4 * x26;
	const FLT x37 = x7 * x23;
	const FLT x38 = (-1 * x37) + x36 + x34 + x35;
	const FLT x39 = (x31 * sensor_pt[1]) + (-1 * x38 * sensor_pt[0]) + (x33 * sensor_pt[2]);
	const FLT x40 = x9 * x32;
	const FLT x41 = x0 * x29;
	const FLT x42 = x22 * x26;
	const FLT x43 = x7 * x19;
	const FLT x44 = x43 + x42 + (-1 * x40) + x41;
	const FLT x45 = (-1 * x44 * sensor_pt[1]) + (x38 * sensor_pt[2]) + (x33 * sensor_pt[0]);
	const FLT x46 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x47 = (*_x0).Pose.Pos[1] + (2 * ((x44 * x45) + (-1 * x31 * x39))) + (*error_model).Pose.Pos[1] +
					(x46 * ((*_x0).Acc[1] + (*error_model).Acc[1])) + sensor_pt[1] +
					(dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1]));
	const FLT x48 = (-1 * x31 * sensor_pt[2]) + (x33 * sensor_pt[1]) + (x44 * sensor_pt[0]);
	const FLT x49 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (2 * ((x48 * x31) + (-1 * x45 * x38))) +
					(*error_model).Pose.Pos[2] + (dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2])) +
					(x46 * ((*_x0).Acc[2] + (*error_model).Acc[2]));
	const FLT x50 = (x46 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + (2 * ((x38 * x39) + (-1 * x44 * x48))) +
					sensor_pt[0] + (dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) +
					(*error_model).Pose.Pos[0] + (*_x0).Pose.Pos[0];
	const FLT x51 = (-1 * x50 * (*lh_p).Rot[2]) + (x47 * (*lh_p).Rot[1]) + (x49 * (*lh_p).Rot[0]);
	const FLT x52 = (-1 * x47 * (*lh_p).Rot[3]) + (x50 * (*lh_p).Rot[0]) + (x49 * (*lh_p).Rot[2]);
	const FLT x53 = x47 + (*lh_p).Pos[1] + (2 * ((x52 * (*lh_p).Rot[3]) + (-1 * x51 * (*lh_p).Rot[1])));
	const FLT x54 = 2 * x33;
	const FLT x55 = x54 * x38;
	const FLT x56 = 2 * x31;
	const FLT x57 = (x56 * x44) + (-1 * x55);
	const FLT x58 = (-1 * x36) + x37 + (-1 * x35) + (-1 * x34);
	const FLT x59 = x54 * x44;
	const FLT x60 = x59 + (-1 * x58 * x56);
	const FLT x61 = 2 * x38;
	const FLT x62 = 1 + (x61 * x58) + (-2 * (x44 * x44));
	const FLT x63 = (x62 * (*lh_p).Rot[3]) + (-1 * x57 * (*lh_p).Rot[1]) + (x60 * (*lh_p).Rot[0]);
	const FLT x64 = 2 * (*lh_p).Rot[1];
	const FLT x65 = 2 * ((x57 * (*lh_p).Rot[2]) + (-1 * x60 * (*lh_p).Rot[3]) + (x62 * (*lh_p).Rot[0]));
	const FLT x66 = x57 + (x63 * x64) + (-1 * x65 * (*lh_p).Rot[2]);
	const FLT x67 = (-1 * x49 * (*lh_p).Rot[1]) + (x47 * (*lh_p).Rot[0]) + (x50 * (*lh_p).Rot[3]);
	const FLT x68 = x49 + (2 * ((x67 * (*lh_p).Rot[1]) + (-1 * x52 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x69 = x68 * x68;
	const FLT x70 = 1. / x69;
	const FLT x71 = x70 * x66;
	const FLT x72 = 1. / x68;
	const FLT x73 = (-1 * x62 * (*lh_p).Rot[2]) + (x57 * (*lh_p).Rot[0]) + (x60 * (*lh_p).Rot[1]);
	const FLT x74 = x60 + (x65 * (*lh_p).Rot[3]) + (-1 * x73 * x64);
	const FLT x75 = x69 + (x53 * x53);
	const FLT x76 = 1. / x75;
	const FLT x77 = x76 * x69;
	const FLT x78 = x50 + (*lh_p).Pos[0] + (2 * ((x51 * (*lh_p).Rot[2]) + (-1 * x67 * (*lh_p).Rot[3])));
	const FLT x79 = x78 * x78;
	const FLT x80 = 1. / sqrt(1 + (-1 * x79 * x76 * ((*bsc0).tilt * (*bsc0).tilt)));
	const FLT x81 = 2 * x53;
	const FLT x82 = 2 * x68;
	const FLT x83 = 1.0 / 2.0 * x78 * (1. / (x75 * sqrt(x75))) * (*bsc0).tilt;
	const FLT x84 = 2 * (*lh_p).Rot[2];
	const FLT x85 = 2 * (*lh_p).Rot[3];
	const FLT x86 = x62 + (x84 * x73) + (-1 * x85 * x63);
	const FLT x87 = (1. / sqrt(x75)) * (*bsc0).tilt;
	const FLT x88 = (-1 * x80 * ((x86 * x87) + (-1 * ((x82 * x66) + (x81 * x74)) * x83))) +
					(-1 * ((x72 * x74) + (-1 * x71 * x53)) * x77);
	const FLT x89 = -1 * x68;
	const FLT x90 = (*bsc0).gibmag * sin(1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha +
										 (-1 * asin(x87 * x78)) + (-1 * atan2(-1 * x53, x89)));
	const FLT x91 = 2 * (1. / (x69 + x79)) * x69 * atan2(x78, x89) * (*bsc0).curve;
	const FLT x92 = x40 + (-1 * x43) + (-1 * x41) + (-1 * x42);
	const FLT x93 = 2 * x44;
	const FLT x94 = 1 + (x93 * x92) + (-2 * (x31 * x31));
	const FLT x95 = x54 * x31;
	const FLT x96 = x95 + (-1 * x61 * x92);
	const FLT x97 = (x61 * x31) + (-1 * x59);
	const FLT x98 = (x97 * (*lh_p).Rot[3]) + (x94 * (*lh_p).Rot[0]) + (-1 * x96 * (*lh_p).Rot[1]);
	const FLT x99 = 2 * ((x96 * (*lh_p).Rot[2]) + (-1 * x94 * (*lh_p).Rot[3]) + (x97 * (*lh_p).Rot[0]));
	const FLT x100 = (x64 * x98) + x96 + (-1 * x99 * (*lh_p).Rot[2]);
	const FLT x101 = x70 * x100;
	const FLT x102 = (x96 * (*lh_p).Rot[0]) + (-1 * x97 * (*lh_p).Rot[2]) + (x94 * (*lh_p).Rot[1]);
	const FLT x103 = x94 + (x99 * (*lh_p).Rot[3]) + (-1 * x64 * x102);
	const FLT x104 = x97 + (-1 * x85 * x98) + (x84 * x102);
	const FLT x105 = (-1 * x80 * ((x87 * x104) + (-1 * ((x82 * x100) + (x81 * x103)) * x83))) +
					 (-1 * ((x72 * x103) + (-1 * x53 * x101)) * x77);
	const FLT x106 = (x61 * x44) + (-1 * x95);
	const FLT x107 = (-1 * x27) + (-1 * x30) + (-1 * x24) + x20;
	const FLT x108 = 1 + (x56 * x107) + (-2 * (x38 * x38));
	const FLT x109 = x55 + (-1 * x93 * x107);
	const FLT x110 = 2 * ((x109 * (*lh_p).Rot[3]) + (x106 * (*lh_p).Rot[0]) + (-1 * x108 * (*lh_p).Rot[1]));
	const FLT x111 = 2 * ((x108 * (*lh_p).Rot[2]) + (-1 * x106 * (*lh_p).Rot[3]) + (x109 * (*lh_p).Rot[0]));
	const FLT x112 = x108 + (x110 * (*lh_p).Rot[1]) + (-1 * x111 * (*lh_p).Rot[2]);
	const FLT x113 = x70 * x112;
	const FLT x114 = 2 * ((x108 * (*lh_p).Rot[0]) + (x106 * (*lh_p).Rot[1]) + (-1 * x109 * (*lh_p).Rot[2]));
	const FLT x115 = x106 + (x111 * (*lh_p).Rot[3]) + (-1 * x114 * (*lh_p).Rot[1]);
	const FLT x116 = x109 + (-1 * x110 * (*lh_p).Rot[3]) + (x114 * (*lh_p).Rot[2]);
	const FLT x117 = (-1 * x80 * ((x87 * x116) + (-1 * ((x82 * x112) + (x81 * x115)) * x83))) +
					 (-1 * ((x72 * x115) + (-1 * x53 * x113)) * x77);
	cnMatrixOptionalSet(Hx, 0, 0, (x88 * x90) + x88 + (((x71 * x78) + (-1 * x86 * x72)) * x91));
	cnMatrixOptionalSet(Hx, 0, 1, x105 + (x90 * x105) + (((x78 * x101) + (-1 * x72 * x104)) * x91));
	cnMatrixOptionalSet(Hx, 0, 2, x117 + (x90 * x117) + (((x78 * x113) + (-1 * x72 * x116)) * x91));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen1 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]

static inline void SurviveKalmanErrorModel_LightMeas_y_gen1_jac_sensor_pt_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_y_gen1(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_y_gen1_jac_sensor_pt(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen1 wrt [(*lh_p).Pos[0], (*lh_p).Pos[1], (*lh_p).Pos[2],
// (*lh_p).Rot[0], (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]
static inline void SurviveKalmanErrorModel_LightMeas_y_gen1_jac_lh_p(CnMat *Hx, const FLT dt,
																	 const SurviveKalmanModel *_x0,
																	 const SurviveKalmanErrorModel *error_model,
																	 const FLT *sensor_pt, const SurvivePose *lh_p,
																	 const BaseStationCal *bsc0) {
	const FLT x0 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x1 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x2 = 0.5 * (*_x0).Pose.Rot[0];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x4 = (-1 * x3 * (*_x0).Pose.Rot[1]) + (x2 * (*error_model).Pose.AxisAngleRot[1]) +
				   (x1 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x5 = dt * dt;
	const FLT x6 = (x0 * x0) * x5;
	const FLT x7 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x8 = x5 * (x7 * x7);
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x10 = x5 * (x9 * x9);
	const FLT x11 = 1e-10 + x10 + x6 + x8;
	const FLT x12 = sqrt(x11);
	const FLT x13 = 0.5 * x12;
	const FLT x14 = sin(x13);
	const FLT x15 = (x14 * x14) * (1. / x11);
	const FLT x16 = cos(x13);
	const FLT x17 = 1. / sqrt((x8 * x15) + (x16 * x16) + (x6 * x15) + (x15 * x10));
	const FLT x18 = dt * x14 * (1. / x12) * x17;
	const FLT x19 = x4 * x18;
	const FLT x20 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x21 = (x20 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x1 * (*_x0).Pose.Rot[2]) +
					(x2 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x22 = x9 * x18;
	const FLT x23 = (*_x0).Pose.Rot[1] + (x2 * (*error_model).Pose.AxisAngleRot[0]) + (-1 * x20 * (*_x0).Pose.Rot[3]) +
					(x3 * (*_x0).Pose.Rot[2]);
	const FLT x24 = x17 * x16;
	const FLT x25 = (-1 * x1 * (*_x0).Pose.Rot[1]) + (-1 * x20 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x3 * (*_x0).Pose.Rot[3]);
	const FLT x26 = x7 * x18;
	const FLT x27 = (x24 * x23) + (x25 * x26) + (-1 * x0 * x19) + (x22 * x21);
	const FLT x28 = x0 * x18;
	const FLT x29 = (-1 * x23 * x26) + (-1 * x9 * x19) + (-1 * x21 * x28) + (x24 * x25);
	const FLT x30 = (-1 * x21 * x26) + (x23 * x28) + (x4 * x24) + (x25 * x22);
	const FLT x31 = (-1 * x30 * sensor_pt[0]) + (x27 * sensor_pt[1]) + (x29 * sensor_pt[2]);
	const FLT x32 = (x24 * x21) + (x4 * x26) + (-1 * x22 * x23) + (x25 * x28);
	const FLT x33 = (-1 * x32 * sensor_pt[1]) + (x30 * sensor_pt[2]) + (x29 * sensor_pt[0]);
	const FLT x34 = 2 * ((x32 * x33) + (-1 * x31 * x27));
	const FLT x35 = dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1]);
	const FLT x36 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x37 = x36 * ((*_x0).Acc[1] + (*error_model).Acc[1]);
	const FLT x38 = x34 + (*error_model).Pose.Pos[1] + (*_x0).Pose.Pos[1] + x35 + sensor_pt[1] + x37;
	const FLT x39 = x38 * (*lh_p).Rot[0];
	const FLT x40 = (-1 * x27 * sensor_pt[2]) + (x29 * sensor_pt[1]) + (x32 * sensor_pt[0]);
	const FLT x41 = 2 * ((x30 * x31) + (-1 * x40 * x32));
	const FLT x42 = dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0]);
	const FLT x43 = x36 * ((*_x0).Acc[0] + (*error_model).Acc[0]);
	const FLT x44 = x43 + x41 + (*error_model).Pose.Pos[0] + sensor_pt[0] + x42 + (*_x0).Pose.Pos[0];
	const FLT x45 = x44 * (*lh_p).Rot[3];
	const FLT x46 = 2 * ((x40 * x27) + (-1 * x30 * x33));
	const FLT x47 = x36 * ((*_x0).Acc[2] + (*error_model).Acc[2]);
	const FLT x48 = dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2]);
	const FLT x49 = sensor_pt[2] + (*_x0).Pose.Pos[2] + x47 + x46 + (*error_model).Pose.Pos[2] + x48;
	const FLT x50 = x49 * (*lh_p).Rot[1];
	const FLT x51 = x39 + (-1 * x50) + x45;
	const FLT x52 = x38 * (*lh_p).Rot[1];
	const FLT x53 = x49 * (*lh_p).Rot[0];
	const FLT x54 = x44 * (*lh_p).Rot[2];
	const FLT x55 = (-1 * x54) + x52 + x53;
	const FLT x56 = x44 + (*lh_p).Pos[0] + (2 * ((x55 * (*lh_p).Rot[2]) + (-1 * x51 * (*lh_p).Rot[3])));
	const FLT x57 = x44 * (*lh_p).Rot[0];
	const FLT x58 = x49 * (*lh_p).Rot[2];
	const FLT x59 = x38 * (*lh_p).Rot[3];
	const FLT x60 = (-1 * x59) + x57 + x58;
	const FLT x61 = x49 + (2 * ((x51 * (*lh_p).Rot[1]) + (-1 * x60 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x62 = x61 * x61;
	const FLT x63 = (*lh_p).Pos[1] + x38 + (2 * ((x60 * (*lh_p).Rot[3]) + (-1 * x55 * (*lh_p).Rot[1])));
	const FLT x64 = (x63 * x63) + x62;
	const FLT x65 = (1. / sqrt(x64)) * (*bsc0).tilt;
	const FLT x66 = -1 * x61;
	const FLT x67 = (*bsc0).gibmag * sin(1.5707963267949 + (-1 * asin(x65 * x56)) + (-1 * (*bsc0).phase) +
										 (*bsc0).gibpha + (-1 * atan2(-1 * x63, x66)));
	const FLT x68 = x56 * x56;
	const FLT x69 = 1. / x64;
	const FLT x70 = 1. / sqrt(1 + (-1 * x68 * x69 * ((*bsc0).tilt * (*bsc0).tilt)));
	const FLT x71 = x70 * x65;
	const FLT x72 = 2 * x61;
	const FLT x73 = (1. / (x62 + x68)) * atan2(x56, x66) * (*bsc0).curve;
	const FLT x74 = (1. / (x64 * sqrt(x64))) * x56 * (*bsc0).tilt;
	const FLT x75 = x70 * x74;
	const FLT x76 = (x75 * x63) + (-1 * x61 * x69);
	const FLT x77 = (x75 * x61) + (x63 * x69);
	const FLT x78 = 2 * x73;
	const FLT x79 = 2 * x54;
	const FLT x80 = (2 * x52) + (-1 * x79);
	const FLT x81 = 1. / x62;
	const FLT x82 = x81 * x63;
	const FLT x83 = 1. / x61;
	const FLT x84 = 2 * x50;
	const FLT x85 = (2 * x45) + (-1 * x84);
	const FLT x86 = x62 * x69;
	const FLT x87 = 2 * x63;
	const FLT x88 = 1.0 / 2.0 * x74;
	const FLT x89 = 2 * x59;
	const FLT x90 = (2 * x58) + (-1 * x89);
	const FLT x91 = (-1 * x70 * ((x65 * x90) + (-1 * ((x80 * x72) + (x85 * x87)) * x88))) +
					(-1 * ((x83 * x85) + (-1 * x80 * x82)) * x86);
	const FLT x92 = x81 * x56;
	const FLT x93 = x78 * x62;
	const FLT x94 = (-1 * sensor_pt[2]) + (-1 * x48) + (-1 * x46) + (-1 * x47) + (-1 * (*_x0).Pose.Pos[2]) +
					(-1 * (*error_model).Pose.Pos[2]);
	const FLT x95 = 2 * (*lh_p).Rot[1];
	const FLT x96 = 2 * x39;
	const FLT x97 = x85 + (x95 * x94) + x96;
	const FLT x98 = 2 * x53;
	const FLT x99 = (-1 * x98) + (-4 * x52) + x79;
	const FLT x100 = 2 * (*lh_p).Rot[3];
	const FLT x101 = 2 * (*lh_p).Rot[2];
	const FLT x102 = (x38 * x101) + (-1 * x94 * x100);
	const FLT x103 = (-1 * x70 * ((x65 * x102) + (-1 * ((x72 * x97) + (x87 * x99)) * x88))) +
					 (-1 * ((x83 * x99) + (-1 * x82 * x97)) * x86);
	const FLT x104 = 2 * x57;
	const FLT x105 = (-4 * x58) + (-1 * x104) + x89;
	const FLT x106 = (-1 * (*error_model).Pose.Pos[0]) + (-1 * x41) + (-1 * (*_x0).Pose.Pos[0]) + (-1 * x42) +
					 (-1 * sensor_pt[0]) + (-1 * x43);
	const FLT x107 = (x49 * x100) + (-1 * x95 * x106);
	const FLT x108 = (x101 * x106) + x80 + x98;
	const FLT x109 = (-1 * x70 * ((x65 * x108) + (-1 * ((x72 * x105) + (x87 * x107)) * x88))) +
					 (-1 * ((x83 * x107) + (-1 * x82 * x105)) * x86);
	const FLT x110 = (-1 * sensor_pt[1]) + (-1 * x34) + (-1 * x37) + (-1 * (*error_model).Pose.Pos[1]) +
					 (-1 * (*_x0).Pose.Pos[1]) + (-1 * x35);
	const FLT x111 = (x95 * x44) + (-1 * x101 * x110);
	const FLT x112 = x90 + x104 + (x100 * x110);
	const FLT x113 = x84 + (-1 * x96) + (-4 * x45);
	const FLT x114 = (-1 * x70 * ((x65 * x113) + (-1 * ((x72 * x111) + (x87 * x112)) * x88))) +
					 (-1 * ((x83 * x112) + (-1 * x82 * x111)) * x86);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[0]) / sizeof(FLT),
						(-1 * x73 * x72) + (-1 * x71 * x67) + (-1 * x71));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[1]) / sizeof(FLT), x76 + (x76 * x67));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[2]) / sizeof(FLT), (x77 * x67) + x77 + (x78 * x56));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[0]) / sizeof(FLT),
						x91 + (x67 * x91) + (((x80 * x92) + (-1 * x83 * x90)) * x93));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[1]) / sizeof(FLT),
						x103 + (x67 * x103) + (x93 * ((x92 * x97) + (-1 * x83 * x102))));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[2]) / sizeof(FLT),
						x109 + (x67 * x109) + (((x92 * x105) + (-1 * x83 * x108)) * x93));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[3]) / sizeof(FLT),
						x114 + (x67 * x114) + (((x92 * x111) + (-1 * x83 * x113)) * x93));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen1 wrt [(*lh_p).Pos[0], (*lh_p).Pos[1],
// (*lh_p).Pos[2], (*lh_p).Rot[0], (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]

static inline void SurviveKalmanErrorModel_LightMeas_y_gen1_jac_lh_p_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_y_gen1(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_y_gen1_jac_lh_p(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen1 wrt [<cnkalman.codegen.WrapMember object at 0x7f88f4f2a700>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4f2a430>, <cnkalman.codegen.WrapMember object at 0x7f88f4f2a760>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4f2a520>, <cnkalman.codegen.WrapMember object at 0x7f88f4f2a4c0>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4f2a7c0>, <cnkalman.codegen.WrapMember object at 0x7f88f4f2a6a0>]
static inline void SurviveKalmanErrorModel_LightMeas_y_gen1_jac_bsc0(CnMat *Hx, const FLT dt,
																	 const SurviveKalmanModel *_x0,
																	 const SurviveKalmanErrorModel *error_model,
																	 const FLT *sensor_pt, const SurvivePose *lh_p,
																	 const BaseStationCal *bsc0) {
	const FLT x0 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x1 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x2 = 0.5 * (*_x0).Pose.Rot[0];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x4 = (-1 * x3 * (*_x0).Pose.Rot[1]) + (x2 * (*error_model).Pose.AxisAngleRot[1]) +
				   (x1 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x5 = dt * dt;
	const FLT x6 = (x0 * x0) * x5;
	const FLT x7 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x8 = x5 * (x7 * x7);
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x10 = x5 * (x9 * x9);
	const FLT x11 = 1e-10 + x10 + x6 + x8;
	const FLT x12 = sqrt(x11);
	const FLT x13 = 0.5 * x12;
	const FLT x14 = sin(x13);
	const FLT x15 = (x14 * x14) * (1. / x11);
	const FLT x16 = cos(x13);
	const FLT x17 = 1. / sqrt((x8 * x15) + (x16 * x16) + (x6 * x15) + (x15 * x10));
	const FLT x18 = dt * x14 * (1. / x12) * x17;
	const FLT x19 = x4 * x18;
	const FLT x20 = 0.5 * (*_x0).Pose.Rot[2];
	const FLT x21 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x22 = (x21 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x20 * (*error_model).Pose.AxisAngleRot[0]) +
					(x2 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x23 = x22 * x18;
	const FLT x24 = (*_x0).Pose.Rot[1] + (x2 * (*error_model).Pose.AxisAngleRot[0]) + (-1 * x21 * (*_x0).Pose.Rot[3]) +
					(x20 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x25 = x17 * x16;
	const FLT x26 = (-1 * x1 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[0] +
					(-1 * x20 * (*error_model).Pose.AxisAngleRot[1]) + (-1 * x3 * (*_x0).Pose.Rot[3]);
	const FLT x27 = x26 * x18;
	const FLT x28 = (x7 * x27) + (-1 * x0 * x19) + (x24 * x25) + (x9 * x23);
	const FLT x29 = x24 * x18;
	const FLT x30 = (-1 * x7 * x29) + (-1 * x9 * x19) + (-1 * x0 * x23) + (x25 * x26);
	const FLT x31 = (-1 * x7 * x23) + (x4 * x25) + (x0 * x29) + (x9 * x27);
	const FLT x32 = (-1 * x31 * sensor_pt[0]) + (x28 * sensor_pt[1]) + (x30 * sensor_pt[2]);
	const FLT x33 = (x7 * x19) + (x25 * x22) + (-1 * x9 * x29) + (x0 * x27);
	const FLT x34 = (x31 * sensor_pt[2]) + (-1 * x33 * sensor_pt[1]) + (x30 * sensor_pt[0]);
	const FLT x35 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x36 = (*_x0).Pose.Pos[1] + (2 * ((x34 * x33) + (-1 * x32 * x28))) + (*error_model).Pose.Pos[1] +
					sensor_pt[1] + (dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) +
					(x35 * ((*_x0).Acc[1] + (*error_model).Acc[1]));
	const FLT x37 = (x30 * sensor_pt[1]) + (-1 * x28 * sensor_pt[2]) + (x33 * sensor_pt[0]);
	const FLT x38 = (x35 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + (*error_model).Pose.Pos[0] +
					(2 * ((x32 * x31) + (-1 * x33 * x37))) + sensor_pt[0] +
					(dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) + (*_x0).Pose.Pos[0];
	const FLT x39 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (x35 * ((*_x0).Acc[2] + (*error_model).Acc[2])) +
					(2 * ((x37 * x28) + (-1 * x31 * x34))) +
					(dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2])) + (*error_model).Pose.Pos[2];
	const FLT x40 = (-1 * x39 * (*lh_p).Rot[1]) + (x36 * (*lh_p).Rot[0]) + (x38 * (*lh_p).Rot[3]);
	const FLT x41 = (-1 * x38 * (*lh_p).Rot[2]) + (x36 * (*lh_p).Rot[1]) + (x39 * (*lh_p).Rot[0]);
	const FLT x42 = x38 + (*lh_p).Pos[0] + (2 * ((x41 * (*lh_p).Rot[2]) + (-1 * x40 * (*lh_p).Rot[3])));
	const FLT x43 = (-1 * x36 * (*lh_p).Rot[3]) + (x38 * (*lh_p).Rot[0]) + (x39 * (*lh_p).Rot[2]);
	const FLT x44 = x39 + (2 * ((x40 * (*lh_p).Rot[1]) + (-1 * x43 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x45 = -1 * x44;
	const FLT x46 = x36 + (*lh_p).Pos[1] + (2 * ((x43 * (*lh_p).Rot[3]) + (-1 * x41 * (*lh_p).Rot[1])));
	const FLT x47 = (x46 * x46) + (x44 * x44);
	const FLT x48 = x42 * (1. / sqrt(x47));
	const FLT x49 = 1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha + (-1 * asin(x48 * (*bsc0).tilt)) +
					(-1 * atan2(-1 * x46, x45));
	const FLT x50 = sin(x49) * (*bsc0).gibmag;
	const FLT x51 = x48 * (1. / sqrt(1 + (-1 * (x42 * x42) * (1. / x47) * ((*bsc0).tilt * (*bsc0).tilt))));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, curve) / sizeof(FLT), atan2(x42, x45) * atan2(x42, x45));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibmag) / sizeof(FLT), -1 * cos(x49));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibpha) / sizeof(FLT), x50);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, phase) / sizeof(FLT), -1 + (-1 * x50));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, tilt) / sizeof(FLT), (-1 * x51) + (-1 * x50 * x51));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen1 wrt [<cnkalman.codegen.WrapMember object at
// 0x7f88f4f2a700>, <cnkalman.codegen.WrapMember object at 0x7f88f4f2a430>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4f2a760>, <cnkalman.codegen.WrapMember object at 0x7f88f4f2a520>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4f2a4c0>, <cnkalman.codegen.WrapMember object at 0x7f88f4f2a7c0>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4f2a6a0>]

static inline void SurviveKalmanErrorModel_LightMeas_y_gen1_jac_bsc0_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_y_gen1(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_y_gen1_jac_bsc0(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
static inline FLT SurviveKalmanErrorModel_LightMeas_x_gen2(const FLT dt, const SurviveKalmanModel *_x0,
														   const SurviveKalmanErrorModel *error_model,
														   const FLT *sensor_pt, const SurvivePose *lh_p,
														   const BaseStationCal *bsc0) {
	const FLT x0 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x1 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x2 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x4 =
		(*_x0).Pose.Rot[1] + (x3 * (*_x0).Pose.Rot[0]) + (-1 * x1 * (*_x0).Pose.Rot[3]) + (x2 * (*_x0).Pose.Rot[2]);
	const FLT x5 = dt * dt;
	const FLT x6 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x7 = (x6 * x6) * x5;
	const FLT x8 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x9 = x5 * (x8 * x8);
	const FLT x10 = (x0 * x0) * x5;
	const FLT x11 = 1e-10 + x7 + x10 + x9;
	const FLT x12 = sqrt(x11);
	const FLT x13 = 0.5 * x12;
	const FLT x14 = sin(x13);
	const FLT x15 = (x14 * x14) * (1. / x11);
	const FLT x16 = cos(x13);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x9 * x15) + (x7 * x15) + (x15 * x10));
	const FLT x18 = dt * x14 * (1. / x12) * x17;
	const FLT x19 = x4 * x18;
	const FLT x20 = (-1 * x3 * (*_x0).Pose.Rot[1]) + (-1 * x1 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x2 * (*_x0).Pose.Rot[3]);
	const FLT x21 = x20 * x18;
	const FLT x22 =
		(*_x0).Pose.Rot[3] + (x1 * (*_x0).Pose.Rot[1]) + (-1 * x3 * (*_x0).Pose.Rot[2]) + (x2 * (*_x0).Pose.Rot[0]);
	const FLT x23 = x17 * x16;
	const FLT x24 =
		(-1 * x2 * (*_x0).Pose.Rot[1]) + (x3 * (*_x0).Pose.Rot[3]) + (x1 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2];
	const FLT x25 = x24 * x18;
	const FLT x26 = (x22 * x23) + (-1 * x0 * x19) + (x8 * x25) + (x6 * x21);
	const FLT x27 = x22 * x18;
	const FLT x28 = (-1 * x8 * x19) + (-1 * x0 * x25) + (-1 * x6 * x27) + (x23 * x20);
	const FLT x29 = (x8 * x21) + (x4 * x23) + (-1 * x6 * x25) + (x0 * x27);
	const FLT x30 = (-1 * x29 * sensor_pt[2]) + (x28 * sensor_pt[1]) + (x26 * sensor_pt[0]);
	const FLT x31 = (-1 * x8 * x27) + (x24 * x23) + (x6 * x19) + (x0 * x21);
	const FLT x32 = (-1 * x31 * sensor_pt[0]) + (x29 * sensor_pt[1]) + (x28 * sensor_pt[2]);
	const FLT x33 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x34 = (x33 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + (2 * ((x32 * x31) + (-1 * x30 * x26))) +
					(*error_model).Pose.Pos[0] + sensor_pt[0] +
					(dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) + (*_x0).Pose.Pos[0];
	const FLT x35 = (-1 * x26 * sensor_pt[1]) + (x31 * sensor_pt[2]) + (x28 * sensor_pt[0]);
	const FLT x36 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (2 * ((x30 * x29) + (-1 * x31 * x35))) +
					(*error_model).Pose.Pos[2] + (dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2])) +
					(x33 * ((*_x0).Acc[2] + (*error_model).Acc[2]));
	const FLT x37 = (*_x0).Pose.Pos[1] + (2 * ((x35 * x26) + (-1 * x32 * x29))) + (*error_model).Pose.Pos[1] +
					(dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) + sensor_pt[1] +
					(x33 * ((*_x0).Acc[1] + (*error_model).Acc[1]));
	const FLT x38 = (-1 * x37 * (*lh_p).Rot[3]) + (x34 * (*lh_p).Rot[0]) + (x36 * (*lh_p).Rot[2]);
	const FLT x39 = (-1 * x36 * (*lh_p).Rot[1]) + (x37 * (*lh_p).Rot[0]) + (x34 * (*lh_p).Rot[3]);
	const FLT x40 = x36 + (2 * ((x39 * (*lh_p).Rot[1]) + (-1 * x38 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x41 = (-1 * x34 * (*lh_p).Rot[2]) + (x37 * (*lh_p).Rot[1]) + (x36 * (*lh_p).Rot[0]);
	const FLT x42 = x34 + (*lh_p).Pos[0] + (2 * ((x41 * (*lh_p).Rot[2]) + (-1 * x39 * (*lh_p).Rot[3])));
	const FLT x43 = atan2(-1 * x40, x42);
	const FLT x44 = x37 + (*lh_p).Pos[1] + (2 * ((x38 * (*lh_p).Rot[3]) + (-1 * x41 * (*lh_p).Rot[1])));
	const FLT x45 = 0.523598775598299 + (*bsc0).tilt;
	const FLT x46 = (x42 * x42) + (x40 * x40);
	const FLT x47 = x44 * (1. / sqrt(x46)) * tan(x45);
	const FLT x48 = ((*bsc0).ogeemag * sin((-1 * asin(x47)) + (*bsc0).ogeephase + x43)) + (*bsc0).curve;
	const FLT x49 = cos(x45);
	const FLT x50 = asin(x44 * (1. / x49) * (1. / sqrt(x46 + (x44 * x44))));
	const FLT x51 = 0.0028679863 + (x50 * (-8.0108022e-06 + (-8.0108022e-06 * x50)));
	const FLT x52 = 5.3685255e-06 + (x50 * x51);
	const FLT x53 = 0.0076069798 + (x50 * x52);
	const FLT x54 =
		(-1 *
		 asin(x47 + ((x50 * x50) * x53 * x48 *
					 (1. / (x49 + (-1 * x48 * sin(x45) *
								   ((x50 * (x53 + (x50 * (x52 + (x50 * (x51 + (x50 * (-8.0108022e-06 +
																					  (-1.60216044e-05 * x50))))))))) +
									(x50 * x53)))))))) +
		x43;
	return -1.5707963267949 + x54 + (-1 * (*bsc0).phase) + (sin(x54 + (*bsc0).gibpha) * (*bsc0).gibmag);
}

// Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen2 wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2],
// (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3],
// (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1],
// (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1],
// (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4f36d90>]
static inline void SurviveKalmanErrorModel_LightMeas_x_gen2_jac_x0(CnMat *Hx, const FLT dt,
																   const SurviveKalmanModel *_x0,
																   const SurviveKalmanErrorModel *error_model,
																   const FLT *sensor_pt, const SurvivePose *lh_p,
																   const BaseStationCal *bsc0) {
	const FLT x0 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x1 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x2 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x3 =
		(-1 * x2 * (*_x0).Pose.Rot[1]) + (x0 * (*_x0).Pose.Rot[3]) + (x1 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2];
	const FLT x4 = dt * dt;
	const FLT x5 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x6 = x5 * x5;
	const FLT x7 = x4 * x6;
	const FLT x8 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x9 = x8 * x8;
	const FLT x10 = x4 * x9;
	const FLT x11 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x12 = x11 * x11;
	const FLT x13 = x4 * x12;
	const FLT x14 = 1e-10 + x13 + x7 + x10;
	const FLT x15 = sqrt(x14);
	const FLT x16 = 1. / x15;
	const FLT x17 = x5 * x16;
	const FLT x18 = 0.5 * x15;
	const FLT x19 = sin(x18);
	const FLT x20 = x19 * x19;
	const FLT x21 = 1. / x14;
	const FLT x22 = x20 * x21;
	const FLT x23 = cos(x18);
	const FLT x24 = (x23 * x23) + (x22 * x10) + (x7 * x22) + (x22 * x13);
	const FLT x25 = 1. / sqrt(x24);
	const FLT x26 = x25 * x19;
	const FLT x27 = dt * x26;
	const FLT x28 = x27 * x17;
	const FLT x29 = 0.5 * (*_x0).Pose.Rot[2];
	const FLT x30 = (x1 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x29 * (*error_model).Pose.AxisAngleRot[0]) +
					(x2 * (*_x0).Pose.Rot[0]);
	const FLT x31 = x27 * x16;
	const FLT x32 = x31 * x11;
	const FLT x33 =
		(*_x0).Pose.Rot[1] + (x0 * (*_x0).Pose.Rot[0]) + (-1 * x1 * (*_x0).Pose.Rot[3]) + (x2 * (*_x0).Pose.Rot[2]);
	const FLT x34 = x25 * x23;
	const FLT x35 = x34 * x33;
	const FLT x36 = (-1 * x0 * (*_x0).Pose.Rot[1]) + (-1 * x29 * (*error_model).Pose.AxisAngleRot[1]) +
					(*_x0).Pose.Rot[0] + (-1 * x2 * (*_x0).Pose.Rot[3]);
	const FLT x37 = x8 * x31;
	const FLT x38 = (x36 * x37) + x35 + (-1 * x3 * x28) + (x30 * x32);
	const FLT x39 = x34 * x36;
	const FLT x40 = (-1 * x30 * x28) + (-1 * x33 * x37) + (-1 * x3 * x32) + x39;
	const FLT x41 = x3 * x34;
	const FLT x42 = (-1 * x30 * x37) + (x33 * x28) + x41 + (x32 * x36);
	const FLT x43 = (-1 * x42 * sensor_pt[0]) + (x38 * sensor_pt[1]) + (x40 * sensor_pt[2]);
	const FLT x44 = x30 * x34;
	const FLT x45 = (x3 * x37) + x44 + (-1 * x32 * x33) + (x36 * x28);
	const FLT x46 = (-1 * x45 * sensor_pt[1]) + (x42 * sensor_pt[2]) + (x40 * sensor_pt[0]);
	const FLT x47 = dt * fabs(dt);
	const FLT x48 = 1.0 / 2.0 * x47;
	const FLT x49 = (2 * ((x45 * x46) + (-1 * x43 * x38))) + (*error_model).Pose.Pos[1] +
					(dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) + sensor_pt[1] +
					(*_x0).Pose.Pos[1] + (x48 * ((*_x0).Acc[1] + (*error_model).Acc[1]));
	const FLT x50 = (-1 * x38 * sensor_pt[2]) + (x40 * sensor_pt[1]) + (x45 * sensor_pt[0]);
	const FLT x51 = sensor_pt[2] + (*_x0).Pose.Pos[2] + (2 * ((x50 * x38) + (-1 * x42 * x46))) +
					(dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2])) + (*error_model).Pose.Pos[2] +
					(x48 * ((*_x0).Acc[2] + (*error_model).Acc[2]));
	const FLT x52 = (x48 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + sensor_pt[0] +
					(2 * ((x42 * x43) + (-1 * x50 * x45))) +
					(dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) + (*error_model).Pose.Pos[0] +
					(*_x0).Pose.Pos[0];
	const FLT x53 = (-1 * x52 * (*lh_p).Rot[2]) + (x49 * (*lh_p).Rot[1]) + (x51 * (*lh_p).Rot[0]);
	const FLT x54 = (-1 * x49 * (*lh_p).Rot[3]) + (x52 * (*lh_p).Rot[0]) + (x51 * (*lh_p).Rot[2]);
	const FLT x55 = x49 + (*lh_p).Pos[1] + (2 * ((x54 * (*lh_p).Rot[3]) + (-1 * x53 * (*lh_p).Rot[1])));
	const FLT x56 = x55 * x55;
	const FLT x57 = (-1 * x51 * (*lh_p).Rot[1]) + (x49 * (*lh_p).Rot[0]) + (x52 * (*lh_p).Rot[3]);
	const FLT x58 = (2 * ((x57 * (*lh_p).Rot[1]) + (-1 * x54 * (*lh_p).Rot[2]))) + x51 + (*lh_p).Pos[2];
	const FLT x59 = (*lh_p).Pos[0] + x52 + (2 * ((x53 * (*lh_p).Rot[2]) + (-1 * x57 * (*lh_p).Rot[3])));
	const FLT x60 = x59 * x59;
	const FLT x61 = x60 + (x58 * x58);
	const FLT x62 = x61 + x56;
	const FLT x63 = 0.523598775598299 + (*bsc0).tilt;
	const FLT x64 = cos(x63);
	const FLT x65 = 1. / x64;
	const FLT x66 = (1. / sqrt(x62)) * x65;
	const FLT x67 = asin(x66 * x55);
	const FLT x68 = 8.0108022e-06 * x67;
	const FLT x69 = -8.0108022e-06 + (-1 * x68);
	const FLT x70 = 0.0028679863 + (x67 * x69);
	const FLT x71 = 5.3685255e-06 + (x70 * x67);
	const FLT x72 = 0.0076069798 + (x71 * x67);
	const FLT x73 = x72 * x67;
	const FLT x74 = -8.0108022e-06 + (-1.60216044e-05 * x67);
	const FLT x75 = x70 + (x74 * x67);
	const FLT x76 = x71 + (x75 * x67);
	const FLT x77 = x72 + (x76 * x67);
	const FLT x78 = (x77 * x67) + x73;
	const FLT x79 = sin(x63);
	const FLT x80 = atan2(-1 * x58, x59);
	const FLT x81 = tan(x63);
	const FLT x82 = x81 * (1. / sqrt(x61));
	const FLT x83 = x82 * x55;
	const FLT x84 = asin(x83) + (-1 * (*bsc0).ogeephase) + (-1 * x80);
	const FLT x85 = (-1 * sin(x84) * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x86 = x85 * x79;
	const FLT x87 = x64 + (-1 * x86 * x78);
	const FLT x88 = 1. / x87;
	const FLT x89 = x67 * x67;
	const FLT x90 = x89 * x85;
	const FLT x91 = x88 * x90;
	const FLT x92 = x83 + (x72 * x91);
	const FLT x93 = 1. / sqrt(1 + (-1 * (x92 * x92)));
	const FLT x94 = x47 * (*lh_p).Rot[1] * (*lh_p).Rot[2];
	const FLT x95 = x47 * (*lh_p).Rot[0];
	const FLT x96 = x95 * (*lh_p).Rot[3];
	const FLT x97 = x96 + x94;
	const FLT x98 = 2 * x55;
	const FLT x99 = (*lh_p).Rot[2] * (*lh_p).Rot[2];
	const FLT x100 = -1 * x99 * x47;
	const FLT x101 = (*lh_p).Rot[3] * (*lh_p).Rot[3];
	const FLT x102 = -1 * x47 * x101;
	const FLT x103 = x102 + x48 + x100;
	const FLT x104 = 2 * x59;
	const FLT x105 = x95 * (*lh_p).Rot[2];
	const FLT x106 = x47 * (*lh_p).Rot[3];
	const FLT x107 = x106 * (*lh_p).Rot[1];
	const FLT x108 = x107 + (-1 * x105);
	const FLT x109 = 2 * x58;
	const FLT x110 = (x109 * x108) + (x103 * x104);
	const FLT x111 = 1.0 / 2.0 * x55;
	const FLT x112 = (1. / (x62 * sqrt(x62))) * x65 * x111;
	const FLT x113 = (x66 * x97) + (-1 * x112 * (x110 + (x98 * x97)));
	const FLT x114 = 1. / sqrt(1 + (-1 * (1. / (x64 * x64)) * (1. / x62) * x56));
	const FLT x115 = x76 * x114;
	const FLT x116 = x69 * x114;
	const FLT x117 = x113 * x116;
	const FLT x118 = 2.40324066e-05 * x67;
	const FLT x119 = x118 * x114;
	const FLT x120 = x74 * x114;
	const FLT x121 = x75 * x114;
	const FLT x122 = x68 * x114;
	const FLT x123 = x70 * x114;
	const FLT x124 = (x113 * x123) + (x67 * ((-1 * x113 * x122) + x117));
	const FLT x125 = x71 * x114;
	const FLT x126 = (x113 * x125) + (x67 * x124);
	const FLT x127 = x77 * x114;
	const FLT x128 = x72 * x114;
	const FLT x129 = x81 * (1. / (x61 * sqrt(x61))) * x111;
	const FLT x130 = (x82 * x97) + (-1 * x110 * x129);
	const FLT x131 = 1. / x61;
	const FLT x132 = 1. / sqrt(1 + (-1 * (x81 * x81) * x56 * x131));
	const FLT x133 = (1. / x60) * x58;
	const FLT x134 = 1. / x59;
	const FLT x135 = x60 * x131;
	const FLT x136 = ((-1 * x108 * x134) + (x103 * x133)) * x135;
	const FLT x137 = (-1 * x136) + (x130 * x132);
	const FLT x138 = cos(x84) * (*bsc0).ogeemag;
	const FLT x139 = x79 * x78;
	const FLT x140 = x138 * x139;
	const FLT x141 = (1. / (x87 * x87)) * x72 * x90;
	const FLT x142 = x88 * x89 * x72;
	const FLT x143 = x138 * x142;
	const FLT x144 = 2 * x88 * x85 * x73;
	const FLT x145 = x114 * x144;
	const FLT x146 =
		x136 + (-1 * x93 *
				(x130 + (x113 * x145) + (-1 * x137 * x143) +
				 (-1 * x141 *
				  ((x137 * x140) +
				   (-1 * x86 *
					((x67 * x126) +
					 (x67 * ((x113 * x115) + x126 +
							 (x67 * (x124 + (x67 * ((x113 * x120) + x117 + (-1 * x113 * x119))) + (x113 * x121))))) +
					 (x113 * x128) + (x113 * x127))))) +
				 (x91 * x126)));
	const FLT x147 = cos((-1 * asin(x92)) + (*bsc0).gibpha + x80) * (*bsc0).gibmag;
	const FLT x148 = (*lh_p).Rot[1] * (*lh_p).Rot[1];
	const FLT x149 = (-1 * x47 * x148) + x48;
	const FLT x150 = x149 + x102;
	const FLT x151 = x94 + (-1 * x96);
	const FLT x152 = x106 * (*lh_p).Rot[2];
	const FLT x153 = x95 * (*lh_p).Rot[1];
	const FLT x154 = x153 + x152;
	const FLT x155 = (x109 * x154) + (x104 * x151);
	const FLT x156 = (x66 * x150) + (-1 * x112 * (x155 + (x98 * x150)));
	const FLT x157 = x116 * x156;
	const FLT x158 = x114 * x156;
	const FLT x159 = (x70 * x158) + (x67 * ((-1 * x68 * x158) + x157));
	const FLT x160 = (x71 * x158) + (x67 * x159);
	const FLT x161 = (x82 * x150) + (-1 * x129 * x155);
	const FLT x162 = ((-1 * x134 * x154) + (x133 * x151)) * x135;
	const FLT x163 = (-1 * x162) + (x161 * x132);
	const FLT x164 =
		x162 + (-1 * x93 *
				(x161 +
				 (-1 * x141 *
				  ((x163 * x140) +
				   (-1 * x86 *
					((x72 * x158) + (x67 * x160) +
					 (x67 * ((x115 * x156) + x160 +
							 (x67 * (x159 + (x67 * (x157 + (x120 * x156) + (-1 * x118 * x158))) + (x75 * x158))))) +
					 (x77 * x158))))) +
				 (x91 * x160) + (x144 * x158) + (-1 * x163 * x143)));
	const FLT x165 = x105 + x107;
	const FLT x166 = x149 + x100;
	const FLT x167 = (x109 * x166) + (x104 * x165);
	const FLT x168 = x152 + (-1 * x153);
	const FLT x169 = (x82 * x168) + (-1 * x129 * x167);
	const FLT x170 = ((-1 * x166 * x134) + (x165 * x133)) * x135;
	const FLT x171 = (-1 * x170) + (x169 * x132);
	const FLT x172 = (x66 * x168) + (-1 * x112 * (x167 + (x98 * x168)));
	const FLT x173 = x116 * x172;
	const FLT x174 = x114 * x172;
	const FLT x175 = (x70 * x174) + (x67 * ((-1 * x68 * x174) + x173));
	const FLT x176 = (x71 * x174) + (x67 * x175);
	const FLT x177 =
		x170 + (-1 * x93 *
				((-1 * x141 *
				  ((x171 * x140) +
				   (-1 * x86 *
					((x77 * x174) + (x67 * x176) + (x72 * x174) +
					 (x67 * (x176 + (x115 * x172) +
							 (x67 * (x175 + (x67 * ((x120 * x172) + (-1 * x118 * x174) + x173)) + (x75 * x174))))))))) +
				 (x174 * x144) + (-1 * x171 * x143) + x169 + (x91 * x176)));
	const FLT x178 = 2 * (*lh_p).Rot[2];
	const FLT x179 = x178 * (*lh_p).Rot[1];
	const FLT x180 = 2 * (*lh_p).Rot[0];
	const FLT x181 = x180 * (*lh_p).Rot[3];
	const FLT x182 = x181 + x179;
	const FLT x183 = 2 * x99;
	const FLT x184 = -1 * x183;
	const FLT x185 = 2 * x101;
	const FLT x186 = 1 + (-1 * x185);
	const FLT x187 = x186 + x184;
	const FLT x188 = x180 * (*lh_p).Rot[2];
	const FLT x189 = 2 * (*lh_p).Rot[1];
	const FLT x190 = x189 * (*lh_p).Rot[3];
	const FLT x191 = x190 + (-1 * x188);
	const FLT x192 = (x109 * x191) + (x104 * x187);
	const FLT x193 = (x66 * x182) + (-1 * x112 * (x192 + (x98 * x182)));
	const FLT x194 = x116 * x193;
	const FLT x195 = (x123 * x193) + (x67 * ((-1 * x122 * x193) + x194));
	const FLT x196 = (x125 * x193) + (x67 * x195);
	const FLT x197 = (x82 * x182) + (-1 * x129 * x192);
	const FLT x198 = ((-1 * x191 * x134) + (x187 * x133)) * x135;
	const FLT x199 = (-1 * x198) + (x197 * x132);
	const FLT x200 =
		x198 + (-1 * x93 *
				((x193 * x145) +
				 (-1 * x141 *
				  ((x199 * x140) +
				   (-1 * x86 *
					((x128 * x193) + (x67 * x196) +
					 (x67 * (x196 + (x115 * x193) +
							 (x67 * (x195 + (x67 * ((x120 * x193) + x194 + (-1 * x119 * x193))) + (x121 * x193))))) +
					 (x127 * x193))))) +
				 (x91 * x196) + x197 + (-1 * x199 * x143)));
	const FLT x201 = x179 + (-1 * x181);
	const FLT x202 = x178 * (*lh_p).Rot[3];
	const FLT x203 = x180 * (*lh_p).Rot[1];
	const FLT x204 = x203 + x202;
	const FLT x205 = (x204 * x109) + (x201 * x104);
	const FLT x206 = 2 * x148;
	const FLT x207 = -1 * x206;
	const FLT x208 = x186 + x207;
	const FLT x209 = (x82 * x208) + (-1 * x205 * x129);
	const FLT x210 = ((-1 * x204 * x134) + (x201 * x133)) * x135;
	const FLT x211 = (-1 * x210) + (x209 * x132);
	const FLT x212 = (x66 * x208) + (-1 * x112 * (x205 + (x98 * x208)));
	const FLT x213 = x212 * x116;
	const FLT x214 = (x212 * x123) + (x67 * ((-1 * x212 * x122) + x213));
	const FLT x215 = (x212 * x125) + (x67 * x214);
	const FLT x216 =
		x210 + (-1 * x93 *
				((x212 * x145) +
				 (-1 * x141 *
				  ((x211 * x140) +
				   (-1 * x86 *
					((x212 * x128) + (x67 * x215) +
					 (x67 * ((x212 * x115) + x215 +
							 (x67 * (x214 + (x67 * ((x212 * x120) + x213 + (-1 * x212 * x119))) + (x212 * x121))))) +
					 (x212 * x127))))) +
				 x209 + (-1 * x211 * x143) + (x91 * x215)));
	const FLT x217 = x202 + (-1 * x203);
	const FLT x218 = x188 + x190;
	const FLT x219 = 1 + x207 + x184;
	const FLT x220 = (x219 * x109) + (x218 * x104);
	const FLT x221 = (x66 * x217) + (-1 * x112 * (x220 + (x98 * x217)));
	const FLT x222 = x221 * x116;
	const FLT x223 = (x221 * x123) + (x67 * ((-1 * x221 * x122) + x222));
	const FLT x224 = (x221 * x125) + (x67 * x223);
	const FLT x225 = (x82 * x217) + (-1 * x220 * x129);
	const FLT x226 = ((-1 * x219 * x134) + (x218 * x133)) * x135;
	const FLT x227 = (-1 * x226) + (x225 * x132);
	const FLT x228 =
		x226 + (-1 * x93 *
				((-1 * x141 *
				  ((x227 * x140) +
				   (-1 * x86 *
					((x67 * x224) + (x221 * x128) +
					 (x67 * (x224 + (x221 * x115) +
							 (x67 * (x223 + (x67 * ((x221 * x120) + x222 + (-1 * x221 * x119))) + (x221 * x121))))) +
					 (x221 * x127))))) +
				 x225 + (x221 * x145) + (x91 * x224) + (-1 * x227 * x143)));
	const FLT x229 = x2 * x34;
	const FLT x230 = x1 * x37;
	const FLT x231 = -1 * x0 * x32;
	const FLT x232 = x28 + x231;
	const FLT x233 = x232 + x229 + x230;
	const FLT x234 = 2 * x50;
	const FLT x235 = x0 * x37;
	const FLT x236 = -1 * x235;
	const FLT x237 = x1 * x32;
	const FLT x238 = -1 * x237;
	const FLT x239 = x2 * x28;
	const FLT x240 = x34 + (-1 * x239);
	const FLT x241 = x240 + x236 + x238;
	const FLT x242 = x2 * x32;
	const FLT x243 = x0 * x34;
	const FLT x244 = -1 * x1 * x28;
	const FLT x245 = x37 + x244;
	const FLT x246 = x245 + x242 + x243;
	const FLT x247 = (x241 * sensor_pt[1]) + (x233 * sensor_pt[0]) + (-1 * x246 * sensor_pt[2]);
	const FLT x248 = 2 * x45;
	const FLT x249 = x1 * x34;
	const FLT x250 = x0 * x28;
	const FLT x251 = -1 * x2 * x37;
	const FLT x252 = x32 + x251;
	const FLT x253 = x252 + x249 + x250;
	const FLT x254 = 2 * x43;
	const FLT x255 = (x241 * sensor_pt[2]) + (-1 * x253 * sensor_pt[0]) + (x246 * sensor_pt[1]);
	const FLT x256 = 2 * x42;
	const FLT x257 = (x256 * x255) + (x254 * x253) + (-1 * x234 * x233) + (-1 * x247 * x248);
	const FLT x258 = 2 * x38;
	const FLT x259 = (x241 * sensor_pt[0]) + (-1 * x233 * sensor_pt[1]) + (x253 * sensor_pt[2]);
	const FLT x260 = 2 * x46;
	const FLT x261 = (x233 * x260) + (x259 * x248) + (-1 * x255 * x258) + (-1 * x254 * x246);
	const FLT x262 = (x234 * x246) + (-1 * x256 * x259) + (-1 * x260 * x253) + (x258 * x247);
	const FLT x263 = (x262 * (*lh_p).Rot[0]) + (-1 * x257 * (*lh_p).Rot[2]) + (x261 * (*lh_p).Rot[1]);
	const FLT x264 = (x262 * (*lh_p).Rot[2]) + (-1 * x261 * (*lh_p).Rot[3]) + (x257 * (*lh_p).Rot[0]);
	const FLT x265 = 2 * (*lh_p).Rot[3];
	const FLT x266 = x261 + (-1 * x263 * x189) + (x264 * x265);
	const FLT x267 = (x257 * (*lh_p).Rot[3]) + (-1 * x262 * (*lh_p).Rot[1]) + (x261 * (*lh_p).Rot[0]);
	const FLT x268 = x257 + (-1 * x267 * x265) + (x263 * x178);
	const FLT x269 = (x267 * x189) + x262 + (-1 * x264 * x178);
	const FLT x270 = (x269 * x109) + (x268 * x104);
	const FLT x271 = (x66 * x266) + (-1 * x112 * (x270 + (x98 * x266)));
	const FLT x272 = x271 * x116;
	const FLT x273 = (x271 * x123) + (x67 * ((-1 * x271 * x122) + x272));
	const FLT x274 = (x271 * x125) + (x67 * x273);
	const FLT x275 = (x82 * x266) + (-1 * x270 * x129);
	const FLT x276 = ((-1 * x269 * x134) + (x268 * x133)) * x135;
	const FLT x277 = (-1 * x276) + (x275 * x132);
	const FLT x278 =
		x276 + (-1 * x93 *
				(x275 + (-1 * x277 * x143) +
				 (-1 * x141 *
				  ((x277 * x140) +
				   (-1 * x86 *
					((x67 * (x274 + (x271 * x115) +
							 (x67 * ((x67 * (x272 + (x271 * x120) + (-1 * x271 * x119))) + x273 + (x271 * x121))))) +
					 (x271 * x128) + (x67 * x274) + (x271 * x127))))) +
				 (x271 * x145) + (x91 * x274)));
	const FLT x279 = -1 * x230;
	const FLT x280 = -1 * x229;
	const FLT x281 = x232 + x279 + x280;
	const FLT x282 = -1 * x250;
	const FLT x283 = (-1 * x32) + x251;
	const FLT x284 = x283 + x249 + x282;
	const FLT x285 = -1 * x243;
	const FLT x286 = x244 + (-1 * x37);
	const FLT x287 = x286 + x242 + x285;
	const FLT x288 = (x287 * sensor_pt[0]) + (-1 * x284 * sensor_pt[1]) + (x281 * sensor_pt[2]);
	const FLT x289 = x239 + x34;
	const FLT x290 = x289 + x236 + x237;
	const FLT x291 = (x284 * sensor_pt[0]) + (-1 * x290 * sensor_pt[2]) + (x287 * sensor_pt[1]);
	const FLT x292 = (x291 * x258) + (x234 * x290) + (-1 * x260 * x281) + (-1 * x288 * x256);
	const FLT x293 = (x287 * sensor_pt[2]) + (-1 * x281 * sensor_pt[0]) + (x290 * sensor_pt[1]);
	const FLT x294 = (x288 * x248) + (-1 * x293 * x258) + (-1 * x290 * x254) + (x260 * x284);
	const FLT x295 = (-1 * x291 * x248) + (x293 * x256) + (-1 * x234 * x284) + (x281 * x254);
	const FLT x296 = (x295 * (*lh_p).Rot[3]) + (-1 * x292 * (*lh_p).Rot[1]) + (x294 * (*lh_p).Rot[0]);
	const FLT x297 = (x292 * (*lh_p).Rot[0]) + (-1 * x295 * (*lh_p).Rot[2]) + (x294 * (*lh_p).Rot[1]);
	const FLT x298 = x295 + (-1 * x296 * x265) + (x297 * x178);
	const FLT x299 = (x292 * (*lh_p).Rot[2]) + (-1 * x294 * (*lh_p).Rot[3]) + (x295 * (*lh_p).Rot[0]);
	const FLT x300 = x292 + (-1 * x299 * x178) + (x296 * x189);
	const FLT x301 = (x300 * x109) + (x298 * x104);
	const FLT x302 = x294 + (-1 * x297 * x189) + (x299 * x265);
	const FLT x303 = (x82 * x302) + (-1 * x301 * x129);
	const FLT x304 = ((-1 * x300 * x134) + (x298 * x133)) * x135;
	const FLT x305 = (-1 * x304) + (x303 * x132);
	const FLT x306 = (x66 * x302) + (-1 * x112 * (x301 + (x98 * x302)));
	const FLT x307 = x306 * x116;
	const FLT x308 = (x306 * x123) + (x67 * ((-1 * x306 * x122) + x307));
	const FLT x309 = (x306 * x125) + (x67 * x308);
	const FLT x310 =
		x304 + (-1 * x93 *
				(x303 + (x91 * x309) + (-1 * x305 * x143) + (x306 * x145) +
				 (-1 * x141 *
				  ((x305 * x140) +
				   (-1 * x86 *
					((x306 * x128) + (x67 * x309) +
					 (x67 * (x309 + (x306 * x115) +
							 (x67 * (x308 + (x67 * ((x306 * x120) + (-1 * x306 * x119) + x307)) + (x306 * x121))))) +
					 (x306 * x127)))))));
	const FLT x311 = -1 * x242;
	const FLT x312 = x245 + x285 + x311;
	const FLT x313 = -1 * x249;
	const FLT x314 = x283 + x250 + x313;
	const FLT x315 = x231 + (-1 * x28);
	const FLT x316 = x315 + x229 + x279;
	const FLT x317 = (x314 * sensor_pt[1]) + (x312 * sensor_pt[0]) + (-1 * x316 * sensor_pt[2]);
	const FLT x318 = x289 + x238 + x235;
	const FLT x319 = (x314 * sensor_pt[2]) + (-1 * x318 * sensor_pt[0]) + (x316 * sensor_pt[1]);
	const FLT x320 = (x254 * x318) + (x256 * x319) + (-1 * x234 * x312) + (-1 * x248 * x317);
	const FLT x321 = (x314 * sensor_pt[0]) + (x318 * sensor_pt[2]) + (-1 * x312 * sensor_pt[1]);
	const FLT x322 = (x248 * x321) + (-1 * x258 * x319) + (-1 * x254 * x316) + (x260 * x312);
	const FLT x323 = (x258 * x317) + (x234 * x316) + (-1 * x260 * x318) + (-1 * x256 * x321);
	const FLT x324 = (x323 * (*lh_p).Rot[0]) + (-1 * x320 * (*lh_p).Rot[2]) + (x322 * (*lh_p).Rot[1]);
	const FLT x325 = (x323 * (*lh_p).Rot[2]) + (-1 * x322 * (*lh_p).Rot[3]) + (x320 * (*lh_p).Rot[0]);
	const FLT x326 = x322 + (-1 * x324 * x189) + (x265 * x325);
	const FLT x327 = (x320 * (*lh_p).Rot[3]) + (-1 * x323 * (*lh_p).Rot[1]) + (x322 * (*lh_p).Rot[0]);
	const FLT x328 = x320 + (-1 * x265 * x327) + (x324 * x178);
	const FLT x329 = x323 + (-1 * x325 * x178) + (x327 * x189);
	const FLT x330 = (x329 * x109) + (x328 * x104);
	const FLT x331 = (x66 * x326) + (-1 * x112 * (x330 + (x98 * x326)));
	const FLT x332 = x331 * x116;
	const FLT x333 = (x331 * x123) + (x67 * ((-1 * x331 * x122) + x332));
	const FLT x334 = (x331 * x125) + (x67 * x333);
	const FLT x335 = (x82 * x326) + (-1 * x330 * x129);
	const FLT x336 = ((-1 * x329 * x134) + (x328 * x133)) * x135;
	const FLT x337 = (-1 * x336) + (x335 * x132);
	const FLT x338 =
		x336 +
		(-1 * x93 *
		 ((x331 * x145) + (-1 * x337 * x143) + x335 +
		  (-1 * x141 *
		   ((x337 * x140) +
			(-1 * x86 *
			 ((x331 * x128) + (x67 * x334) + (x331 * x127) +
			  (x67 * (x334 + (x331 * x115) +
					  (x67 * (x333 + (x67 * ((x331 * x120) + (-1 * x331 * x119) + x332)) + (x331 * x121))))))))) +
		  (x91 * x334)));
	const FLT x339 = x252 + x282 + x313;
	const FLT x340 = x243 + x286 + x311;
	const FLT x341 = x230 + x315 + x280;
	const FLT x342 = (x341 * sensor_pt[2]) + (-1 * x340 * sensor_pt[0]) + (x339 * sensor_pt[1]);
	const FLT x343 = x240 + x235 + x237;
	const FLT x344 = (x341 * sensor_pt[0]) + (-1 * x343 * sensor_pt[1]) + (x340 * sensor_pt[2]);
	const FLT x345 = (x260 * x343) + (x248 * x344) + (-1 * x254 * x339) + (-1 * x258 * x342);
	const FLT x346 = (x343 * sensor_pt[0]) + (-1 * x339 * sensor_pt[2]) + (x341 * sensor_pt[1]);
	const FLT x347 = (x254 * x340) + (-1 * x248 * x346) + (-1 * x234 * x343) + (x256 * x342);
	const FLT x348 = (x234 * x339) + (x258 * x346) + (-1 * x260 * x340) + (-1 * x256 * x344);
	const FLT x349 = (x348 * (*lh_p).Rot[2]) + (-1 * x345 * (*lh_p).Rot[3]) + (x347 * (*lh_p).Rot[0]);
	const FLT x350 = (x348 * (*lh_p).Rot[0]) + (x345 * (*lh_p).Rot[1]) + (-1 * x347 * (*lh_p).Rot[2]);
	const FLT x351 = x345 + (x265 * x349) + (-1 * x350 * x189);
	const FLT x352 = (x347 * (*lh_p).Rot[3]) + (-1 * x348 * (*lh_p).Rot[1]) + (x345 * (*lh_p).Rot[0]);
	const FLT x353 = (-1 * x265 * x352) + x347 + (x350 * x178);
	const FLT x354 = x348 + (-1 * x349 * x178) + (x352 * x189);
	const FLT x355 = (x354 * x109) + (x353 * x104);
	const FLT x356 = (x66 * x351) + (-1 * x112 * (x355 + (x98 * x351)));
	const FLT x357 = x356 * x116;
	const FLT x358 = (x356 * x123) + (x67 * ((-1 * x356 * x122) + x357));
	const FLT x359 = (x356 * x125) + (x67 * x358);
	const FLT x360 = (x82 * x351) + (-1 * x355 * x129);
	const FLT x361 = ((-1 * x354 * x134) + (x353 * x133)) * x135;
	const FLT x362 = x138 * ((-1 * x361) + (x360 * x132));
	const FLT x363 =
		x361 + (-1 * x93 *
				(x360 +
				 (-1 * x141 *
				  ((x362 * x139) +
				   (-1 * x86 *
					((x356 * x128) + (x67 * x359) +
					 (x67 * (x359 + (x356 * x115) +
							 (x67 * ((x67 * ((x356 * x120) + x357 + (-1 * x356 * x119))) + x358 + (x356 * x121))))) +
					 (x356 * x127))))) +
				 (x356 * x145) + (x91 * x359) + (-1 * x362 * x142)));
	const FLT x364 = dt * dt * dt;
	const FLT x365 = x9 * x364;
	const FLT x366 = 0.5 * x21;
	const FLT x367 = x41 * x366;
	const FLT x368 = x3 * x31;
	const FLT x369 = x4 * x8;
	const FLT x370 = x16 * x369;
	const FLT x371 = 0.5 * x26;
	const FLT x372 = x30 * x371;
	const FLT x373 = x11 * x16;
	const FLT x374 = dt * dt * dt * dt;
	const FLT x375 = (x8 * x8 * x8) * x374;
	const FLT x376 = 1. / (x14 * sqrt(x14));
	const FLT x377 = 1.0 * x23 * x19;
	const FLT x378 = x377 * x376;
	const FLT x379 = 2 * x22;
	const FLT x380 = x4 * x379;
	const FLT x381 = 2 * x20 * (1. / (x14 * x14));
	const FLT x382 = x374 * x381;
	const FLT x383 = x6 * x382;
	const FLT x384 = x374 * x378;
	const FLT x385 = x8 * x384;
	const FLT x386 = x16 * x377;
	const FLT x387 = x12 * x382;
	const FLT x388 = (-1 * x369 * x386) + (x6 * x385) + (x12 * x385) + (x378 * x375) + (-1 * x8 * x387) +
					 (-1 * x375 * x381) + (x8 * x380) + (-1 * x8 * x383);
	const FLT x389 = 1.0 / 2.0 * (1. / (x24 * sqrt(x24)));
	const FLT x390 = dt * x19 * x389;
	const FLT x391 = x33 * x390;
	const FLT x392 = x391 * x388;
	const FLT x393 = x23 * x389;
	const FLT x394 = x30 * x393;
	const FLT x395 = x26 * x376;
	const FLT x396 = x3 * x395;
	const FLT x397 = x36 * x390;
	const FLT x398 = x397 * x388;
	const FLT x399 = x8 * x16;
	const FLT x400 = x3 * x390;
	const FLT x401 = x400 * x388;
	const FLT x402 = x35 * x366;
	const FLT x403 = x8 * x364;
	const FLT x404 = x11 * x403;
	const FLT x405 = x33 * x26;
	const FLT x406 = x405 * x376;
	const FLT x407 = x406 * x403;
	const FLT x408 = (x11 * x407) + (-1 * x402 * x404);
	const FLT x409 = x36 * x395;
	const FLT x410 = x403 * x409;
	const FLT x411 = x39 * x366;
	const FLT x412 = x5 * x403;
	const FLT x413 = (x412 * x411) + (-1 * x5 * x410);
	const FLT x414 = (-1 * x401 * x399) + x413 + (-1 * x17 * x398) + x408 + (x365 * x367) + (-1 * x396 * x365) +
					 (-1 * x372 * x370) + x368 + (-1 * x394 * x388) + (x373 * x392);
	const FLT x415 = x30 * x31;
	const FLT x416 = -1 * x415;
	const FLT x417 = x44 * x366;
	const FLT x418 = x30 * x390;
	const FLT x419 = x418 * x388;
	const FLT x420 = x5 * x407;
	const FLT x421 = x3 * x393;
	const FLT x422 = x402 * x412;
	const FLT x423 = x3 * x371;
	const FLT x424 = x16 * x423;
	const FLT x425 = x30 * x395;
	const FLT x426 = (x404 * x411) + (-1 * x11 * x410);
	const FLT x427 = (-1 * x424 * x369) + (-1 * x17 * x392) + x422 + (x425 * x365) + (-1 * x417 * x365) + x416 + x426 +
					 (-1 * x421 * x388) + (x419 * x399) + (-1 * x373 * x398) + (-1 * x420);
	const FLT x428 = 0.5 * x405;
	const FLT x429 = x403 * x396;
	const FLT x430 = x5 * x429;
	const FLT x431 = x403 * x425;
	const FLT x432 = x11 * x431;
	const FLT x433 = x404 * x417;
	const FLT x434 = x33 * x393;
	const FLT x435 = x412 * x367;
	const FLT x436 = x31 * x36;
	const FLT x437 = (-1 * x399 * x398) + x430 + (-1 * x419 * x373) + (x411 * x365) + (-1 * x428 * x370) + (-1 * x432) +
					 (x17 * x401) + (-1 * x409 * x365) + x433 + (-1 * x434 * x388) + x436 + (-1 * x435);
	const FLT x438 = x31 * x33;
	const FLT x439 = -1 * x438;
	const FLT x440 = x36 * x393;
	const FLT x441 = x11 * x429;
	const FLT x442 = x404 * x367;
	const FLT x443 = x36 * x371;
	const FLT x444 = x16 * x443;
	const FLT x445 = (x5 * x431) + (-1 * x412 * x417);
	const FLT x446 = (-1 * x444 * x369) + x445 + (x17 * x419) + (x399 * x392) + (x406 * x365) + (-1 * x402 * x365) +
					 x439 + x441 + (-1 * x442) + (-1 * x440 * x388) + (x401 * x373);
	const FLT x447 = (x446 * sensor_pt[2]) + (-1 * x427 * sensor_pt[0]) + (x437 * sensor_pt[1]);
	const FLT x448 = (x414 * sensor_pt[0]) + (-1 * x437 * sensor_pt[2]) + (x446 * sensor_pt[1]);
	const FLT x449 = (x427 * x254) + (-1 * x448 * x248) + (-1 * x414 * x234) + (x447 * x256);
	const FLT x450 = (x446 * sensor_pt[0]) + (-1 * x414 * sensor_pt[1]) + (x427 * sensor_pt[2]);
	const FLT x451 = (x414 * x260) + (-1 * x437 * x254) + (-1 * x447 * x258) + (x450 * x248);
	const FLT x452 = (-1 * x450 * x256) + (x448 * x258) + (-1 * x427 * x260) + (x437 * x234);
	const FLT x453 = (x452 * (*lh_p).Rot[0]) + (-1 * x449 * (*lh_p).Rot[2]) + (x451 * (*lh_p).Rot[1]);
	const FLT x454 = (x452 * (*lh_p).Rot[2]) + (-1 * x451 * (*lh_p).Rot[3]) + (x449 * (*lh_p).Rot[0]);
	const FLT x455 = x451 + (-1 * x453 * x189) + (x454 * x265);
	const FLT x456 = (x449 * (*lh_p).Rot[3]) + (-1 * x452 * (*lh_p).Rot[1]) + (x451 * (*lh_p).Rot[0]);
	const FLT x457 = x449 + (-1 * x456 * x265) + (x453 * x178);
	const FLT x458 = x452 + (-1 * x454 * x178) + (x456 * x189);
	const FLT x459 = (x458 * x109) + (x457 * x104);
	const FLT x460 = (x66 * x455) + (-1 * x112 * (x459 + (x98 * x455)));
	const FLT x461 = x460 * x116;
	const FLT x462 = x460 * x114;
	const FLT x463 = (x460 * x123) + (x67 * ((-1 * x68 * x462) + x461));
	const FLT x464 = (x460 * x125) + (x67 * x463);
	const FLT x465 = (x82 * x455) + (-1 * x459 * x129);
	const FLT x466 = ((-1 * x458 * x134) + (x457 * x133)) * x135;
	const FLT x467 = (-1 * x466) + (x465 * x132);
	const FLT x468 =
		x466 + (-1 * x93 *
				((x462 * x144) + x465 +
				 (-1 * x141 *
				  ((x467 * x140) +
				   (-1 * x86 *
					((x67 * x464) + (x460 * x128) +
					 (x67 * (x464 + (x460 * x115) +
							 (x67 * (x463 + (x67 * ((x460 * x120) + x461 + (-1 * x462 * x118))) + (x460 * x121))))) +
					 (x460 * x127))))) +
				 (x91 * x464) + (-1 * x467 * x143)));
	const FLT x469 = x12 * x364;
	const FLT x470 = x9 * x384;
	const FLT x471 = x9 * x382;
	const FLT x472 = x4 * x11;
	const FLT x473 = x11 * x11 * x11;
	const FLT x474 = (-1 * x473 * x382) + (x472 * x379) + (-1 * x11 * x471) + (x11 * x470) + (x473 * x384) +
					 (-1 * x472 * x386) + (-1 * x11 * x383) + (x6 * x11 * x384);
	const FLT x475 = x17 * x400;
	const FLT x476 = x474 * x373;
	const FLT x477 = x16 * x472;
	const FLT x478 = x474 * x397;
	const FLT x479 = x5 * x11 * x364;
	const FLT x480 = (-1 * x479 * x367) + (x479 * x396);
	const FLT x481 = x426 + x480 + (-1 * x477 * x428) + (x474 * x475) + (-1 * x469 * x425) + (-1 * x478 * x399) + x415 +
					 (-1 * x474 * x434) + (-1 * x476 * x418) + (x469 * x417);
	const FLT x482 = x474 * x391;
	const FLT x483 = x479 * x402;
	const FLT x484 = x479 * x406;
	const FLT x485 = x418 * x399;
	const FLT x486 = (-1 * x472 * x424) + x436 + (-1 * x469 * x409) + (-1 * x17 * x482) + (-1 * x478 * x373) +
					 (-1 * x433) + (-1 * x484) + (x469 * x411) + (-1 * x474 * x421) + x483 + (x474 * x485) + x432;
	const FLT x487 = x479 * x425;
	const FLT x488 = x17 * x418;
	const FLT x489 = x479 * x417;
	const FLT x490 = -1 * x368;
	const FLT x491 = (x482 * x399) + (-1 * x474 * x440) + (x469 * x396) + (-1 * x472 * x444) + x490 + (-1 * x489) +
					 x408 + x487 + (-1 * x469 * x367) + (x474 * x488) + (x476 * x400);
	const FLT x492 = (x491 * sensor_pt[2]) + (-1 * x486 * sensor_pt[0]) + (x481 * sensor_pt[1]);
	const FLT x493 = x400 * x399;
	const FLT x494 = (x479 * x411) + (-1 * x479 * x409);
	const FLT x495 = (-1 * x493 * x474) + (-1 * x477 * x372) + x494 + (-1 * x474 * x394) + (-1 * x441) + x439 + x442 +
					 (-1 * x17 * x478) + (x476 * x391) + (-1 * x469 * x402) + (x469 * x406);
	const FLT x496 = (x491 * sensor_pt[0]) + (x486 * sensor_pt[2]) + (-1 * x495 * sensor_pt[1]);
	const FLT x497 = (x495 * x260) + (-1 * x481 * x254) + (x496 * x248) + (-1 * x492 * x258);
	const FLT x498 = (x495 * sensor_pt[0]) + (-1 * x481 * sensor_pt[2]) + (x491 * sensor_pt[1]);
	const FLT x499 = (-1 * x495 * x234) + (x492 * x256) + (-1 * x498 * x248) + (x486 * x254);
	const FLT x500 = (x498 * x258) + (x481 * x234) + (-1 * x486 * x260) + (-1 * x496 * x256);
	const FLT x501 = (x500 * (*lh_p).Rot[2]) + (-1 * x497 * (*lh_p).Rot[3]) + (x499 * (*lh_p).Rot[0]);
	const FLT x502 = (x500 * (*lh_p).Rot[0]) + (-1 * x499 * (*lh_p).Rot[2]) + (x497 * (*lh_p).Rot[1]);
	const FLT x503 = x497 + (x501 * x265) + (-1 * x502 * x189);
	const FLT x504 = (x497 * (*lh_p).Rot[0]) + (x499 * (*lh_p).Rot[3]) + (-1 * x500 * (*lh_p).Rot[1]);
	const FLT x505 = x499 + (-1 * x504 * x265) + (x502 * x178);
	const FLT x506 = x500 + (-1 * x501 * x178) + (x504 * x189);
	const FLT x507 = (x506 * x109) + (x505 * x104);
	const FLT x508 = (x66 * x503) + (-1 * x112 * (x507 + (x98 * x503)));
	const FLT x509 = x508 * x116;
	const FLT x510 = (x508 * x123) + (x67 * ((-1 * x508 * x122) + x509));
	const FLT x511 = (x508 * x125) + (x67 * x510);
	const FLT x512 = (x82 * x503) + (-1 * x507 * x129);
	const FLT x513 = ((-1 * x506 * x134) + (x505 * x133)) * x135;
	const FLT x514 = (-1 * x513) + (x512 * x132);
	const FLT x515 =
		x513 +
		(-1 * x93 *
		 (x512 + (x508 * x145) + (-1 * x514 * x143) + (x91 * x511) +
		  (-1 * x141 *
		   ((x514 * x140) +
			(-1 * x86 *
			 ((x508 * x128) + (x67 * x511) + (x508 * x127) +
			  (x67 * (x511 + (x508 * x115) +
					  (x67 * (x510 + (x67 * ((x508 * x120) + x509 + (-1 * x508 * x119))) + (x508 * x121)))))))))));
	const FLT x516 = x5 * x5 * x5;
	const FLT x517 = x4 * x17;
	const FLT x518 = (-1 * x517 * x377) + (x5 * x470) + (x5 * x12 * x384) + (x5 * x380) + (-1 * x516 * x382) +
					 (-1 * x5 * x387) + (x516 * x384) + (-1 * x5 * x471);
	const FLT x519 = x6 * x364;
	const FLT x520 = x518 * x373;
	const FLT x521 = x518 * x397;
	const FLT x522 = (-1 * x521 * x399) + (-1 * x434 * x518) + x413 + (x519 * x396) + (-1 * x487) + x489 +
					 (-1 * x418 * x520) + x490 + (x475 * x518) + (-1 * x519 * x367) + (-1 * x428 * x517);
	const FLT x523 = x518 * x391;
	const FLT x524 = (-1 * x493 * x518) + (x523 * x373) + (-1 * x517 * x372) + x435 + x436 + (-1 * x409 * x519) +
					 (-1 * x17 * x521) + (x411 * x519) + x484 + (-1 * x483) + (-1 * x430) + (-1 * x518 * x394);
	const FLT x525 = x445 + (-1 * x421 * x518) + x494 + (x485 * x518) + (-1 * x423 * x517) + (-1 * x17 * x523) +
					 (x402 * x519) + (-1 * x521 * x373) + (-1 * x406 * x519) + x438;
	const FLT x526 = x480 + (x425 * x519) + x420 + (-1 * x417 * x519) + (-1 * x443 * x517) + (x488 * x518) + x416 +
					 (x523 * x399) + (-1 * x422) + (x400 * x520) + (-1 * x440 * x518);
	const FLT x527 = (x526 * sensor_pt[0]) + (-1 * x524 * sensor_pt[1]) + (x525 * sensor_pt[2]);
	const FLT x528 = (x526 * sensor_pt[2]) + (-1 * x525 * sensor_pt[0]) + (x522 * sensor_pt[1]);
	const FLT x529 = (-1 * x528 * x258) + (-1 * x522 * x254) + (x527 * x248) + (x524 * x260);
	const FLT x530 = (x524 * sensor_pt[0]) + (-1 * x522 * sensor_pt[2]) + (x526 * sensor_pt[1]);
	const FLT x531 = (x530 * x258) + (x522 * x234) + (-1 * x525 * x260) + (-1 * x527 * x256);
	const FLT x532 = (-1 * x530 * x248) + (x525 * x254) + (-1 * x524 * x234) + (x528 * x256);
	const FLT x533 = (x529 * (*lh_p).Rot[0]) + (x532 * (*lh_p).Rot[3]) + (-1 * x531 * (*lh_p).Rot[1]);
	const FLT x534 = (x531 * (*lh_p).Rot[0]) + (-1 * x532 * (*lh_p).Rot[2]) + (x529 * (*lh_p).Rot[1]);
	const FLT x535 = (-1 * x533 * x265) + x532 + (x534 * x178);
	const FLT x536 = (-1 * x529 * (*lh_p).Rot[3]) + (x531 * (*lh_p).Rot[2]) + (x532 * (*lh_p).Rot[0]);
	const FLT x537 = x531 + (x533 * x189) + (-1 * x536 * x178);
	const FLT x538 = (x537 * x109) + (x535 * x104);
	const FLT x539 = x529 + (-1 * x534 * x189) + (x536 * x265);
	const FLT x540 = (x82 * x539) + (-1 * x538 * x129);
	const FLT x541 = ((-1 * x537 * x134) + (x535 * x133)) * x135;
	const FLT x542 = (-1 * x541) + (x540 * x132);
	const FLT x543 = (x66 * x539) + (-1 * x112 * (x538 + (x98 * x539)));
	const FLT x544 = x543 * x116;
	const FLT x545 = (x543 * x123) + (x67 * ((-1 * x543 * x122) + x544));
	const FLT x546 = (x543 * x125) + (x67 * x545);
	const FLT x547 =
		x541 +
		(-1 * x93 *
		 (x540 + (x543 * x145) + (x91 * x546) + (-1 * x542 * x143) +
		  (-1 * x141 *
		   ((x542 * x140) +
			(-1 * x86 *
			 ((x543 * x127) + (x67 * x546) + (x543 * x128) +
			  (x67 * (x546 + (x543 * x115) +
					  (x67 * (x545 + (x67 * ((x543 * x120) + x544 + (-1 * x543 * x119))) + (x543 * x121)))))))))));
	const FLT x548 = dt * x179;
	const FLT x549 = dt * x181;
	const FLT x550 = x549 + x548;
	const FLT x551 = -1 * dt * x183;
	const FLT x552 = dt + (-1 * dt * x185);
	const FLT x553 = x552 + x551;
	const FLT x554 = dt * x188;
	const FLT x555 = dt * x190;
	const FLT x556 = x555 + (-1 * x554);
	const FLT x557 = (x556 * x109) + (x553 * x104);
	const FLT x558 = (x66 * x550) + (-1 * x112 * (x557 + (x98 * x550)));
	const FLT x559 = x558 * x116;
	const FLT x560 = (x558 * x123) + (x67 * ((-1 * x558 * x122) + x559));
	const FLT x561 = (x558 * x125) + (x67 * x560);
	const FLT x562 = (x82 * x550) + (-1 * x557 * x129);
	const FLT x563 = ((-1 * x556 * x134) + (x553 * x133)) * x135;
	const FLT x564 = (-1 * x563) + (x562 * x132);
	const FLT x565 =
		x563 + (-1 * x93 *
				(x562 + (x91 * x561) +
				 (-1 * x141 *
				  ((x564 * x140) +
				   (-1 * x86 *
					((x558 * x128) +
					 (x67 * (x561 + (x558 * x115) +
							 (x67 * (x560 + (x67 * ((x558 * x120) + (-1 * x558 * x119) + x559)) + (x558 * x121))))) +
					 (x558 * x127) + (x67 * x561))))) +
				 (x558 * x145) + (-1 * x564 * x143)));
	const FLT x566 = -1 * dt * x206;
	const FLT x567 = x552 + x566;
	const FLT x568 = x548 + (-1 * x549);
	const FLT x569 = dt * x202;
	const FLT x570 = dt * x203;
	const FLT x571 = x570 + x569;
	const FLT x572 = (x571 * x109) + (x568 * x104);
	const FLT x573 = (x66 * x567) + (-1 * x112 * (x572 + (x98 * x567)));
	const FLT x574 = x573 * x114;
	const FLT x575 = x573 * x116;
	const FLT x576 = (x573 * x123) + (x67 * ((-1 * x68 * x574) + x575));
	const FLT x577 = (x71 * x574) + (x67 * x576);
	const FLT x578 = (x82 * x567) + (-1 * x572 * x129);
	const FLT x579 = ((-1 * x571 * x134) + (x568 * x133)) * x135;
	const FLT x580 = (-1 * x579) + (x578 * x132);
	const FLT x581 =
		x579 + (-1 * x93 *
				((x574 * x144) + (x91 * x577) + x578 +
				 (-1 * x141 *
				  ((x580 * x140) +
				   (-1 * x86 *
					((x573 * x128) + (x67 * x577) +
					 (x67 * (x577 + (x573 * x115) +
							 (x67 * (x576 + (x67 * ((-1 * x574 * x118) + (x573 * x120) + x575)) + (x573 * x121))))) +
					 (x77 * x574))))) +
				 (-1 * x580 * x143)));
	const FLT x582 = x569 + (-1 * x570);
	const FLT x583 = x554 + x555;
	const FLT x584 = x551 + dt + x566;
	const FLT x585 = (x584 * x109) + (x583 * x104);
	const FLT x586 = (x66 * x582) + (-1 * x112 * (x585 + (x98 * x582)));
	const FLT x587 = x586 * x116;
	const FLT x588 = (x586 * x123) + (x67 * ((-1 * x586 * x122) + x587));
	const FLT x589 = (x586 * x125) + (x67 * x588);
	const FLT x590 = (x82 * x582) + (-1 * x585 * x129);
	const FLT x591 = ((-1 * x584 * x134) + (x583 * x133)) * x135;
	const FLT x592 = (-1 * x591) + (x590 * x132);
	const FLT x593 =
		x591 + (-1 * x93 *
				(x590 + (x586 * x145) +
				 (-1 * x141 *
				  ((x592 * x140) +
				   (-1 * x86 *
					((x67 * x589) + (x586 * x128) +
					 (x67 * (x589 + (x586 * x115) +
							 (x67 * (x588 + (x67 * ((x586 * x120) + x587 + (-1 * x586 * x119))) + (x586 * x121))))) +
					 (x586 * x127))))) +
				 (x91 * x589) + (-1 * x592 * x143)));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT), x146 + (x146 * x147));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT), x164 + (x164 * x147));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT), x177 + (x177 * x147));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT), x200 + (x200 * x147));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT), x216 + (x216 * x147));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT), x228 + (x228 * x147));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x278 + (x278 * x147));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x310 + (x310 * x147));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x338 + (x338 * x147));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x363 + (x363 * x147));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x468 + (x468 * x147));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x515 + (x515 * x147));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x547 + (x547 * x147));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT), x565 + (x565 * x147));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT), x581 + (x581 * x147));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT), x593 + (x593 * x147));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen2 wrt [(*_x0).AccBias[0], (*_x0).AccBias[1],
// (*_x0).AccBias[2], (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1],
// (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2],
// (*_x0).IMUCorrection[3], (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0],
// (*_x0).Pose.Rot[1], (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0],
// (*_x0).Velocity.AxisAngleRot[1], (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1],
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f36d90>]

static inline void SurviveKalmanErrorModel_LightMeas_x_gen2_jac_x0_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_x_gen2(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_x_gen2_jac_x0(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen2 wrt [(*error_model).AccBias[0], (*error_model).AccBias[1],
// (*error_model).AccBias[2], (*error_model).Acc[0], (*error_model).Acc[1], (*error_model).Acc[2],
// (*error_model).GyroBias[0], (*error_model).GyroBias[1], (*error_model).GyroBias[2], (*error_model).IMUCorrection[0],
// (*error_model).IMUCorrection[1], (*error_model).IMUCorrection[2], (*error_model).IMUCorrection[3],
// (*error_model).Pose.AxisAngleRot[0], (*error_model).Pose.AxisAngleRot[1], (*error_model).Pose.AxisAngleRot[2],
// (*error_model).Pose.Pos[0], (*error_model).Pose.Pos[1], (*error_model).Pose.Pos[2],
// (*error_model).Velocity.AxisAngleRot[0], (*error_model).Velocity.AxisAngleRot[1],
// (*error_model).Velocity.AxisAngleRot[2], (*error_model).Velocity.Pos[0], (*error_model).Velocity.Pos[1],
// (*error_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f354c0>]
static inline void SurviveKalmanErrorModel_LightMeas_x_gen2_jac_error_model(
	CnMat *Hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const FLT x0 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x1 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x2 = 0.5 * (*_x0).Pose.Rot[0];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x4 = (-1 * x3 * (*_x0).Pose.Rot[1]) + (x2 * (*error_model).Pose.AxisAngleRot[1]) +
				   (x1 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x5 = dt * dt;
	const FLT x6 = x0 * x0;
	const FLT x7 = x6 * x5;
	const FLT x8 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x9 = x8 * x8;
	const FLT x10 = x5 * x9;
	const FLT x11 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x12 = x11 * x11;
	const FLT x13 = x5 * x12;
	const FLT x14 = 1e-10 + x13 + x7 + x10;
	const FLT x15 = sqrt(x14);
	const FLT x16 = 0.5 * x15;
	const FLT x17 = sin(x16);
	const FLT x18 = x17 * x17;
	const FLT x19 = 1. / x14;
	const FLT x20 = x19 * x18;
	const FLT x21 = cos(x16);
	const FLT x22 = (x20 * x10) + (x21 * x21) + (x7 * x20) + (x20 * x13);
	const FLT x23 = 1. / sqrt(x22);
	const FLT x24 = x23 * x17;
	const FLT x25 = 1. / x15;
	const FLT x26 = dt * x25;
	const FLT x27 = x24 * x26;
	const FLT x28 = x4 * x27;
	const FLT x29 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x30 = (*_x0).Pose.Rot[3] + (-1 * x1 * (*_x0).Pose.Rot[2]) + (x29 * (*_x0).Pose.Rot[1]) +
					(x2 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x31 = x30 * x27;
	const FLT x32 = (*_x0).Pose.Rot[1] + (x2 * (*error_model).Pose.AxisAngleRot[0]) + (-1 * x29 * (*_x0).Pose.Rot[3]) +
					(x3 * (*_x0).Pose.Rot[2]);
	const FLT x33 = x23 * x21;
	const FLT x34 = x32 * x33;
	const FLT x35 = (-1 * x1 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[0] + (-1 * x29 * (*_x0).Pose.Rot[2]) +
					(-1 * x3 * (*_x0).Pose.Rot[3]);
	const FLT x36 = x35 * x24;
	const FLT x37 = x36 * x26;
	const FLT x38 = x34 + (-1 * x0 * x28) + (x8 * x37) + (x31 * x11);
	const FLT x39 = x33 * x35;
	const FLT x40 = x32 * x27;
	const FLT x41 = (-1 * x8 * x40) + (-1 * x28 * x11) + (-1 * x0 * x31) + x39;
	const FLT x42 = x4 * x33;
	const FLT x43 = (-1 * x8 * x31) + x42 + (x0 * x40) + (x37 * x11);
	const FLT x44 = (-1 * x43 * sensor_pt[0]) + (x38 * sensor_pt[1]) + (x41 * sensor_pt[2]);
	const FLT x45 = x30 * x33;
	const FLT x46 = x45 + (-1 * x40 * x11) + (x8 * x28) + (x0 * x37);
	const FLT x47 = (-1 * x46 * sensor_pt[1]) + (x43 * sensor_pt[2]) + (x41 * sensor_pt[0]);
	const FLT x48 = dt * fabs(dt);
	const FLT x49 = 1.0 / 2.0 * x48;
	const FLT x50 = (*_x0).Pose.Pos[1] + (2 * ((x46 * x47) + (-1 * x44 * x38))) + (*error_model).Pose.Pos[1] +
					sensor_pt[1] + (dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) +
					(x49 * ((*_x0).Acc[1] + (*error_model).Acc[1]));
	const FLT x51 = (-1 * x38 * sensor_pt[2]) + (x41 * sensor_pt[1]) + (x46 * sensor_pt[0]);
	const FLT x52 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (2 * ((x51 * x38) + (-1 * x43 * x47))) +
					(*error_model).Pose.Pos[2] + (x49 * ((*_x0).Acc[2] + (*error_model).Acc[2])) +
					(dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2]));
	const FLT x53 = (x49 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + (2 * ((x43 * x44) + (-1 * x51 * x46))) +
					(*_x0).Pose.Pos[0] + (*error_model).Pose.Pos[0] + sensor_pt[0] +
					(dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0]));
	const FLT x54 = (-1 * x53 * (*lh_p).Rot[2]) + (x50 * (*lh_p).Rot[1]) + (x52 * (*lh_p).Rot[0]);
	const FLT x55 = (-1 * x50 * (*lh_p).Rot[3]) + (x53 * (*lh_p).Rot[0]) + (x52 * (*lh_p).Rot[2]);
	const FLT x56 = x50 + (*lh_p).Pos[1] + (2 * ((x55 * (*lh_p).Rot[3]) + (-1 * x54 * (*lh_p).Rot[1])));
	const FLT x57 = x56 * x56;
	const FLT x58 = (-1 * x52 * (*lh_p).Rot[1]) + (x50 * (*lh_p).Rot[0]) + (x53 * (*lh_p).Rot[3]);
	const FLT x59 = x52 + (2 * ((x58 * (*lh_p).Rot[1]) + (-1 * x55 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x60 = x53 + (*lh_p).Pos[0] + (2 * ((x54 * (*lh_p).Rot[2]) + (-1 * x58 * (*lh_p).Rot[3])));
	const FLT x61 = x60 * x60;
	const FLT x62 = x61 + (x59 * x59);
	const FLT x63 = x62 + x57;
	const FLT x64 = 0.523598775598299 + (*bsc0).tilt;
	const FLT x65 = cos(x64);
	const FLT x66 = 1. / x65;
	const FLT x67 = (1. / sqrt(x63)) * x66;
	const FLT x68 = asin(x67 * x56);
	const FLT x69 = 8.0108022e-06 * x68;
	const FLT x70 = -8.0108022e-06 + (-1 * x69);
	const FLT x71 = 0.0028679863 + (x70 * x68);
	const FLT x72 = 5.3685255e-06 + (x71 * x68);
	const FLT x73 = 0.0076069798 + (x72 * x68);
	const FLT x74 = x68 * x68;
	const FLT x75 = atan2(-1 * x59, x60);
	const FLT x76 = tan(x64);
	const FLT x77 = x76 * (1. / sqrt(x62));
	const FLT x78 = x77 * x56;
	const FLT x79 = (-1 * (*bsc0).ogeephase) + asin(x78) + (-1 * x75);
	const FLT x80 = (-1 * sin(x79) * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x81 = x73 * x68;
	const FLT x82 = -8.0108022e-06 + (-1.60216044e-05 * x68);
	const FLT x83 = x71 + (x82 * x68);
	const FLT x84 = x72 + (x83 * x68);
	const FLT x85 = x73 + (x84 * x68);
	const FLT x86 = (x85 * x68) + x81;
	const FLT x87 = sin(x64);
	const FLT x88 = x80 * x87;
	const FLT x89 = x65 + (-1 * x88 * x86);
	const FLT x90 = 1. / x89;
	const FLT x91 = x80 * x90;
	const FLT x92 = x74 * x91;
	const FLT x93 = x78 + (x73 * x92);
	const FLT x94 = 1. / sqrt(1 + (-1 * (x93 * x93)));
	const FLT x95 = x48 * (*lh_p).Rot[1];
	const FLT x96 = x95 * (*lh_p).Rot[2];
	const FLT x97 = x48 * (*lh_p).Rot[3] * (*lh_p).Rot[0];
	const FLT x98 = x97 + x96;
	const FLT x99 = 2 * x56;
	const FLT x100 = (*lh_p).Rot[3] * (*lh_p).Rot[3];
	const FLT x101 = -1 * x48 * x100;
	const FLT x102 = (*lh_p).Rot[2] * (*lh_p).Rot[2];
	const FLT x103 = (-1 * x48 * x102) + x49;
	const FLT x104 = x103 + x101;
	const FLT x105 = 2 * x60;
	const FLT x106 = x48 * (*lh_p).Rot[2];
	const FLT x107 = x106 * (*lh_p).Rot[0];
	const FLT x108 = x95 * (*lh_p).Rot[3];
	const FLT x109 = x108 + (-1 * x107);
	const FLT x110 = 2 * x59;
	const FLT x111 = (x109 * x110) + (x105 * x104);
	const FLT x112 = 1.0 / 2.0 * x56;
	const FLT x113 = (1. / (x63 * sqrt(x63))) * x66 * x112;
	const FLT x114 = (x67 * x98) + (-1 * x113 * (x111 + (x99 * x98)));
	const FLT x115 = 1. / sqrt(1 + (-1 * (1. / x63) * (1. / (x65 * x65)) * x57));
	const FLT x116 = x84 * x115;
	const FLT x117 = x70 * x115;
	const FLT x118 = x114 * x117;
	const FLT x119 = 2.40324066e-05 * x68;
	const FLT x120 = x119 * x115;
	const FLT x121 = x82 * x115;
	const FLT x122 = x83 * x115;
	const FLT x123 = x69 * x115;
	const FLT x124 = x71 * x115;
	const FLT x125 = (x114 * x124) + (x68 * ((-1 * x114 * x123) + x118));
	const FLT x126 = x72 * x115;
	const FLT x127 = (x114 * x126) + (x68 * x125);
	const FLT x128 = x85 * x115;
	const FLT x129 = x73 * x115;
	const FLT x130 = x76 * (1. / (x62 * sqrt(x62))) * x112;
	const FLT x131 = (x77 * x98) + (-1 * x111 * x130);
	const FLT x132 = 1. / x62;
	const FLT x133 = 1. / sqrt(1 + (-1 * (x76 * x76) * x57 * x132));
	const FLT x134 = (1. / x61) * x59;
	const FLT x135 = 1. / x60;
	const FLT x136 = x61 * x132;
	const FLT x137 = ((-1 * x109 * x135) + (x104 * x134)) * x136;
	const FLT x138 = (-1 * x137) + (x133 * x131);
	const FLT x139 = cos(x79) * (*bsc0).ogeemag;
	const FLT x140 = x86 * x87;
	const FLT x141 = x139 * x140;
	const FLT x142 = x73 * x74;
	const FLT x143 = x80 * (1. / (x89 * x89)) * x142;
	const FLT x144 = x90 * x142;
	const FLT x145 = x139 * x144;
	const FLT x146 = 2 * x81 * x91;
	const FLT x147 = x115 * x146;
	const FLT x148 =
		x137 + (-1 * x94 *
				(x131 + (x114 * x147) + (-1 * x138 * x145) +
				 (-1 * x143 *
				  ((x138 * x141) +
				   (-1 * x88 *
					((x68 * x127) +
					 (x68 * (x127 + (x114 * x116) +
							 (x68 * (x125 + (x68 * (x118 + (x114 * x121) + (-1 * x114 * x120))) + (x114 * x122))))) +
					 (x114 * x129) + (x114 * x128))))) +
				 (x92 * x127)));
	const FLT x149 = cos((-1 * asin(x93)) + (*bsc0).gibpha + x75) * (*bsc0).gibmag;
	const FLT x150 = (*lh_p).Rot[1] * (*lh_p).Rot[1];
	const FLT x151 = -1 * x48 * x150;
	const FLT x152 = x151 + x101 + x49;
	const FLT x153 = x96 + (-1 * x97);
	const FLT x154 = x106 * (*lh_p).Rot[3];
	const FLT x155 = x95 * (*lh_p).Rot[0];
	const FLT x156 = x155 + x154;
	const FLT x157 = (x110 * x156) + (x105 * x153);
	const FLT x158 = (x67 * x152) + (-1 * x113 * (x157 + (x99 * x152)));
	const FLT x159 = x117 * x158;
	const FLT x160 = (x124 * x158) + (x68 * ((-1 * x123 * x158) + x159));
	const FLT x161 = (x126 * x158) + (x68 * x160);
	const FLT x162 = (x77 * x152) + (-1 * x130 * x157);
	const FLT x163 = ((-1 * x135 * x156) + (x134 * x153)) * x136;
	const FLT x164 = (-1 * x163) + (x162 * x133);
	const FLT x165 =
		x163 + (-1 * x94 *
				((x147 * x158) +
				 (-1 * x143 *
				  ((x164 * x141) +
				   (-1 * x88 *
					((x129 * x158) + (x68 * x161) +
					 (x68 * (x161 + (x116 * x158) +
							 (x68 * (x160 + (x68 * ((x121 * x158) + x159 + (-1 * x120 * x158))) + (x122 * x158))))) +
					 (x128 * x158))))) +
				 x162 + (x92 * x161) + (-1 * x164 * x145)));
	const FLT x166 = x107 + x108;
	const FLT x167 = x103 + x151;
	const FLT x168 = (x110 * x167) + (x105 * x166);
	const FLT x169 = x154 + (-1 * x155);
	const FLT x170 = (x77 * x169) + (-1 * x168 * x130);
	const FLT x171 = ((-1 * x167 * x135) + (x166 * x134)) * x136;
	const FLT x172 = (-1 * x171) + (x170 * x133);
	const FLT x173 = (x67 * x169) + (-1 * x113 * (x168 + (x99 * x169)));
	const FLT x174 = x117 * x173;
	const FLT x175 = x115 * x173;
	const FLT x176 = (x124 * x173) + (x68 * ((-1 * x69 * x175) + x174));
	const FLT x177 = (x72 * x175) + (x68 * x176);
	const FLT x178 =
		x171 +
		(-1 * x94 *
		 ((x175 * x146) +
		  (-1 * x143 *
		   ((x172 * x141) +
			(-1 * x88 *
			 ((x129 * x173) + (x85 * x175) + (x68 * x177) +
			  (x68 * ((x84 * x175) + x177 +
					  (x68 * (x176 + (x68 * ((x121 * x173) + (-1 * x119 * x175) + x174)) + (x122 * x173))))))))) +
		  (-1 * x172 * x145) + x170 + (x92 * x177)));
	const FLT x179 = 0.5 * x33;
	const FLT x180 = x179 * (*_x0).Pose.Rot[2];
	const FLT x181 = x2 * x27;
	const FLT x182 = x11 * x181;
	const FLT x183 = 0.5 * x27;
	const FLT x184 = x8 * x183;
	const FLT x185 = x184 * (*_x0).Pose.Rot[3];
	const FLT x186 = x0 * x183;
	const FLT x187 = x186 * (*_x0).Pose.Rot[1];
	const FLT x188 = x185 + (-1 * x180) + (-1 * x187) + (-1 * x182);
	const FLT x189 = 2 * x51;
	const FLT x190 = x11 * x183;
	const FLT x191 = (-1 * x186 * (*_x0).Pose.Rot[3]) + (-1 * x184 * (*_x0).Pose.Rot[1]) +
					 (-1 * x190 * (*_x0).Pose.Rot[2]) + (x2 * x33);
	const FLT x192 = x191 * sensor_pt[2];
	const FLT x193 = x8 * x181;
	const FLT x194 = x186 * (*_x0).Pose.Rot[2];
	const FLT x195 = x190 * (*_x0).Pose.Rot[3];
	const FLT x196 = x179 * (*_x0).Pose.Rot[1];
	const FLT x197 = (-1 * x195) + (-1 * x196) + (-1 * x193) + x194;
	const FLT x198 = x188 * sensor_pt[0];
	const FLT x199 = (-1 * x192) + x198 + (x197 * sensor_pt[1]);
	const FLT x200 = 2 * x46;
	const FLT x201 = x184 * (*_x0).Pose.Rot[2];
	const FLT x202 = x179 * (*_x0).Pose.Rot[3];
	const FLT x203 = x0 * x181;
	const FLT x204 = x190 * (*_x0).Pose.Rot[1];
	const FLT x205 = (-1 * x204) + x203 + x201 + x202;
	const FLT x206 = 2 * x44;
	const FLT x207 = x191 * sensor_pt[1];
	const FLT x208 = x197 * sensor_pt[2];
	const FLT x209 = x208 + (-1 * x205 * sensor_pt[0]) + x207;
	const FLT x210 = 2 * x43;
	const FLT x211 = (x210 * x209) + (x205 * x206) + (-1 * x189 * x188) + (-1 * x200 * x199);
	const FLT x212 = x206 * x191;
	const FLT x213 = 2 * x47;
	const FLT x214 = x188 * sensor_pt[1];
	const FLT x215 = x197 * sensor_pt[0];
	const FLT x216 = x215 + (x205 * sensor_pt[2]) + (-1 * x214);
	const FLT x217 = 2 * x38;
	const FLT x218 = (-1 * x217 * x209) + (x216 * x200) + (-1 * x212) + (x213 * x188);
	const FLT x219 = x189 * x191;
	const FLT x220 = (-1 * x210 * x216) + (x217 * x199) + x219 + (-1 * x213 * x205);
	const FLT x221 = 2 * ((-1 * x211 * (*lh_p).Rot[2]) + (x220 * (*lh_p).Rot[0]) + (x218 * (*lh_p).Rot[1]));
	const FLT x222 = 2 * ((x220 * (*lh_p).Rot[2]) + (-1 * x218 * (*lh_p).Rot[3]) + (x211 * (*lh_p).Rot[0]));
	const FLT x223 = x218 + (-1 * x221 * (*lh_p).Rot[1]) + (x222 * (*lh_p).Rot[3]);
	const FLT x224 = 2 * ((x211 * (*lh_p).Rot[3]) + (-1 * x220 * (*lh_p).Rot[1]) + (x218 * (*lh_p).Rot[0]));
	const FLT x225 = (-1 * x224 * (*lh_p).Rot[3]) + x211 + (x221 * (*lh_p).Rot[2]);
	const FLT x226 = x220 + (-1 * x222 * (*lh_p).Rot[2]) + (x224 * (*lh_p).Rot[1]);
	const FLT x227 = (x226 * x110) + (x225 * x105);
	const FLT x228 = (x67 * x223) + (-1 * x113 * (x227 + (x99 * x223)));
	const FLT x229 = x228 * x117;
	const FLT x230 = (x228 * x124) + (x68 * ((-1 * x228 * x123) + x229));
	const FLT x231 = (x228 * x126) + (x68 * x230);
	const FLT x232 = (x77 * x223) + (-1 * x227 * x130);
	const FLT x233 = ((-1 * x226 * x135) + (x225 * x134)) * x136;
	const FLT x234 = (-1 * x233) + (x232 * x133);
	const FLT x235 =
		x233 + (-1 * x94 *
				((x228 * x147) + x232 + (-1 * x234 * x145) +
				 (-1 * x143 *
				  ((x234 * x141) +
				   (-1 * x88 *
					((x228 * x129) + (x68 * x231) +
					 (x68 * (x231 + (x228 * x116) +
							 (x68 * (x230 + (x68 * ((x228 * x121) + x229 + (-1 * x228 * x120))) + (x228 * x122))))) +
					 (x228 * x128))))) +
				 (x92 * x231)));
	const FLT x236 = (-1 * x194) + x195 + x193 + x196;
	const FLT x237 = x204 + (-1 * x203) + (-1 * x201) + (-1 * x202);
	const FLT x238 = x237 * sensor_pt[2];
	const FLT x239 = (x236 * sensor_pt[0]) + (-1 * x238) + x214;
	const FLT x240 = x191 * sensor_pt[0];
	const FLT x241 = x237 * sensor_pt[1];
	const FLT x242 = (-1 * x240) + (x188 * sensor_pt[2]) + x241;
	const FLT x243 = (x210 * x242) + (-1 * x236 * x189) + x212 + (-1 * x239 * x200);
	const FLT x244 = x198 + (-1 * x236 * sensor_pt[1]) + x192;
	const FLT x245 = (x200 * x244) + (-1 * x217 * x242) + (-1 * x237 * x206) + (x213 * x236);
	const FLT x246 = x213 * x191;
	const FLT x247 = (x217 * x239) + (-1 * x210 * x244) + (-1 * x246) + (x237 * x189);
	const FLT x248 = (x247 * (*lh_p).Rot[0]) + (-1 * x243 * (*lh_p).Rot[2]) + (x245 * (*lh_p).Rot[1]);
	const FLT x249 = 2 * (*lh_p).Rot[1];
	const FLT x250 = (-1 * x245 * (*lh_p).Rot[3]) + (x247 * (*lh_p).Rot[2]) + (x243 * (*lh_p).Rot[0]);
	const FLT x251 = 2 * (*lh_p).Rot[3];
	const FLT x252 = x245 + (-1 * x248 * x249) + (x250 * x251);
	const FLT x253 = (x243 * (*lh_p).Rot[3]) + (-1 * x247 * (*lh_p).Rot[1]) + (x245 * (*lh_p).Rot[0]);
	const FLT x254 = 2 * (*lh_p).Rot[2];
	const FLT x255 = x243 + (-1 * x251 * x253) + (x254 * x248);
	const FLT x256 = x247 + (-1 * x250 * x254) + (x253 * x249);
	const FLT x257 = (x256 * x110) + (x255 * x105);
	const FLT x258 = (x67 * x252) + (-1 * x113 * (x257 + (x99 * x252)));
	const FLT x259 = x258 * x117;
	const FLT x260 = (x258 * x124) + (x68 * ((-1 * x258 * x123) + x259));
	const FLT x261 = (x258 * x126) + (x68 * x260);
	const FLT x262 = (x77 * x252) + (-1 * x257 * x130);
	const FLT x263 = ((-1 * x256 * x135) + (x255 * x134)) * x136;
	const FLT x264 = (-1 * x263) + (x262 * x133);
	const FLT x265 =
		x263 + (-1 * x94 *
				((-1 * x264 * x145) + x262 + (x92 * x261) + (x258 * x147) +
				 (-1 * x143 *
				  ((x264 * x141) +
				   (-1 * x88 *
					((x258 * x129) + (x68 * x261) +
					 (x68 * (x261 + (x258 * x116) +
							 (x68 * ((x68 * ((x258 * x121) + (-1 * x258 * x120) + x259)) + x260 + (x258 * x122))))) +
					 (x258 * x128)))))));
	const FLT x266 = x180 + (-1 * x185) + x182 + x187;
	const FLT x267 = x240 + (-1 * x266 * sensor_pt[2]) + x241;
	const FLT x268 = x238 + (-1 * x215) + (x266 * sensor_pt[1]);
	const FLT x269 = (x210 * x268) + (x206 * x197) + (-1 * x219) + (-1 * x200 * x267);
	const FLT x270 = (x237 * sensor_pt[0]) + (-1 * x207) + x208;
	const FLT x271 = x246 + (-1 * x217 * x268) + (-1 * x206 * x266) + (x200 * x270);
	const FLT x272 = (x266 * x189) + (x217 * x267) + (-1 * x213 * x197) + (-1 * x210 * x270);
	const FLT x273 = 2 * ((-1 * x269 * (*lh_p).Rot[2]) + (x272 * (*lh_p).Rot[0]) + (x271 * (*lh_p).Rot[1]));
	const FLT x274 = 2 * ((-1 * x271 * (*lh_p).Rot[3]) + (x272 * (*lh_p).Rot[2]) + (x269 * (*lh_p).Rot[0]));
	const FLT x275 = x271 + (-1 * x273 * (*lh_p).Rot[1]) + (x274 * (*lh_p).Rot[3]);
	const FLT x276 = 2 * ((x269 * (*lh_p).Rot[3]) + (-1 * x272 * (*lh_p).Rot[1]) + (x271 * (*lh_p).Rot[0]));
	const FLT x277 = x269 + (-1 * x276 * (*lh_p).Rot[3]) + (x273 * (*lh_p).Rot[2]);
	const FLT x278 = x272 + (-1 * x274 * (*lh_p).Rot[2]) + (x276 * (*lh_p).Rot[1]);
	const FLT x279 = (x278 * x110) + (x277 * x105);
	const FLT x280 = (x67 * x275) + (-1 * x113 * (x279 + (x99 * x275)));
	const FLT x281 = x280 * x117;
	const FLT x282 = (x280 * x124) + (x68 * ((-1 * x280 * x123) + x281));
	const FLT x283 = (x280 * x126) + (x68 * x282);
	const FLT x284 = (x77 * x275) + (-1 * x279 * x130);
	const FLT x285 = ((-1 * x278 * x135) + (x277 * x134)) * x136;
	const FLT x286 = (-1 * x285) + (x284 * x133);
	const FLT x287 =
		x285 + (-1 * x94 *
				(x284 + (-1 * x286 * x145) + (x280 * x147) +
				 (-1 * x143 *
				  ((x286 * x141) +
				   (-1 * x88 *
					((x280 * x129) + (x68 * x283) +
					 (x68 * (x283 + (x280 * x116) +
							 (x68 * (x282 + (x68 * ((x280 * x121) + x281 + (-1 * x280 * x120))) + (x280 * x122))))) +
					 (x280 * x128))))) +
				 (x92 * x283)));
	const FLT x288 = x249 * (*lh_p).Rot[2];
	const FLT x289 = x251 * (*lh_p).Rot[0];
	const FLT x290 = x289 + x288;
	const FLT x291 = 2 * x102;
	const FLT x292 = -1 * x291;
	const FLT x293 = 2 * x100;
	const FLT x294 = 1 + (-1 * x293);
	const FLT x295 = x294 + x292;
	const FLT x296 = x254 * (*lh_p).Rot[0];
	const FLT x297 = x251 * (*lh_p).Rot[1];
	const FLT x298 = x297 + (-1 * x296);
	const FLT x299 = (x298 * x110) + (x295 * x105);
	const FLT x300 = (x67 * x290) + (-1 * x113 * (x299 + (x99 * x290)));
	const FLT x301 = x300 * x117;
	const FLT x302 = (x300 * x124) + (x68 * ((-1 * x300 * x123) + x301));
	const FLT x303 = (x300 * x126) + (x68 * x302);
	const FLT x304 = (x77 * x290) + (-1 * x299 * x130);
	const FLT x305 = ((-1 * x298 * x135) + (x295 * x134)) * x136;
	const FLT x306 = (-1 * x305) + (x304 * x133);
	const FLT x307 =
		x305 + (-1 * x94 *
				((-1 * x143 *
				  ((x306 * x141) +
				   (-1 * x88 *
					((x68 * x303) + (x300 * x129) +
					 (x68 * (x303 + (x300 * x116) +
							 (x68 * (x302 + (x68 * (x301 + (x300 * x121) + (-1 * x300 * x120))) + (x300 * x122))))) +
					 (x300 * x128))))) +
				 x304 + (x300 * x147) + (x92 * x303) + (-1 * x306 * x145)));
	const FLT x308 = x288 + (-1 * x289);
	const FLT x309 = x251 * (*lh_p).Rot[2];
	const FLT x310 = x249 * (*lh_p).Rot[0];
	const FLT x311 = x310 + x309;
	const FLT x312 = (x311 * x110) + (x308 * x105);
	const FLT x313 = 2 * x150;
	const FLT x314 = -1 * x313;
	const FLT x315 = x294 + x314;
	const FLT x316 = (x77 * x315) + (-1 * x312 * x130);
	const FLT x317 = ((-1 * x311 * x135) + (x308 * x134)) * x136;
	const FLT x318 = (-1 * x317) + (x316 * x133);
	const FLT x319 = (x67 * x315) + (-1 * x113 * (x312 + (x99 * x315)));
	const FLT x320 = x319 * x117;
	const FLT x321 = (x319 * x124) + (x68 * ((-1 * x319 * x123) + x320));
	const FLT x322 = (x319 * x126) + (x68 * x321);
	const FLT x323 =
		x317 + (-1 * x94 *
				((x319 * x147) + x316 +
				 (-1 * x143 *
				  ((x318 * x141) +
				   (-1 * x88 *
					((x319 * x129) +
					 (x68 * (x322 + (x319 * x116) +
							 (x68 * (x321 + (x68 * ((x319 * x121) + x320 + (-1 * x319 * x120))) + (x319 * x122))))) +
					 (x68 * x322) + (x319 * x128))))) +
				 (-1 * x318 * x145) + (x92 * x322)));
	const FLT x324 = x309 + (-1 * x310);
	const FLT x325 = x296 + x297;
	const FLT x326 = 1 + x314 + x292;
	const FLT x327 = (x326 * x110) + (x325 * x105);
	const FLT x328 = (x67 * x324) + (-1 * x113 * (x327 + (x99 * x324)));
	const FLT x329 = x328 * x117;
	const FLT x330 = (x328 * x124) + (x68 * ((-1 * x328 * x123) + x329));
	const FLT x331 = (x328 * x126) + (x68 * x330);
	const FLT x332 = (x77 * x324) + (-1 * x327 * x130);
	const FLT x333 = ((-1 * x326 * x135) + (x325 * x134)) * x136;
	const FLT x334 = (-1 * x333) + (x332 * x133);
	const FLT x335 =
		x333 + (-1 * x94 *
				(x332 +
				 (-1 * x143 *
				  ((x334 * x141) +
				   (-1 * x88 *
					((x328 * x129) + (x68 * x331) +
					 (x68 * (x331 + (x328 * x116) +
							 (x68 * (x330 + (x68 * ((x328 * x121) + x329 + (-1 * x328 * x120))) + (x328 * x122))))) +
					 (x328 * x128))))) +
				 (x328 * x147) + (x92 * x331) + (-1 * x334 * x145)));
	const FLT x336 = dt * dt * dt;
	const FLT x337 = 0.5 * x19 * x336;
	const FLT x338 = x9 * x337;
	const FLT x339 = x5 * x25;
	const FLT x340 = x30 * x24;
	const FLT x341 = 0.5 * x340;
	const FLT x342 = x339 * x341;
	const FLT x343 = 1.0 / 2.0 * (1. / (x22 * sqrt(x22)));
	const FLT x344 = x26 * x17 * x343;
	const FLT x345 = x8 * x8 * x8;
	const FLT x346 = dt * dt * dt * dt;
	const FLT x347 = 1. / (x14 * sqrt(x14));
	const FLT x348 = 1.0 * x21 * x17;
	const FLT x349 = x348 * x347;
	const FLT x350 = x349 * x346;
	const FLT x351 = 2 * x20;
	const FLT x352 = x5 * x351;
	const FLT x353 = 2 * (1. / (x14 * x14)) * x18;
	const FLT x354 = x353 * x346;
	const FLT x355 = x8 * x354;
	const FLT x356 = x8 * x350;
	const FLT x357 = x339 * x348;
	const FLT x358 = (-1 * x8 * x357) + (x12 * x356) + (-1 * x12 * x355) + (x350 * x345) + (-1 * x354 * x345) +
					 (x8 * x352) + (x6 * x356) + (-1 * x6 * x355);
	const FLT x359 = x32 * x358;
	const FLT x360 = x359 * x344;
	const FLT x361 = x21 * x343;
	const FLT x362 = x361 * x358;
	const FLT x363 = x4 * x24;
	const FLT x364 = x336 * x347;
	const FLT x365 = x9 * x364;
	const FLT x366 = x35 * x344;
	const FLT x367 = x366 * x358;
	const FLT x368 = x8 * x344;
	const FLT x369 = x4 * x368;
	const FLT x370 = x36 * x364;
	const FLT x371 = x0 * x8;
	const FLT x372 = x39 * x337;
	const FLT x373 = (x372 * x371) + (-1 * x371 * x370);
	const FLT x374 = x34 * x337;
	const FLT x375 = x11 * x374;
	const FLT x376 = x32 * x24;
	const FLT x377 = x376 * x364;
	const FLT x378 = x11 * x377;
	const FLT x379 = (x8 * x378) + (-1 * x8 * x375);
	const FLT x380 = x373 + (-1 * x369 * x358) + (-1 * x0 * x367) + (-1 * x363 * x365) + x28 + (-1 * x8 * x342) +
					 (x42 * x338) + x379 + (x11 * x360) + (-1 * x30 * x362);
	const FLT x381 = -1 * x31;
	const FLT x382 = x30 * x368;
	const FLT x383 = x371 * x377;
	const FLT x384 = x374 * x371;
	const FLT x385 = 0.5 * x339;
	const FLT x386 = x8 * x385;
	const FLT x387 = x11 * x372;
	const FLT x388 = x11 * x370;
	const FLT x389 = (-1 * x8 * x388) + (x8 * x387);
	const FLT x390 = (x365 * x340) + x384 + (-1 * x4 * x362) + x381 + (x382 * x358) + x389 + (-1 * x363 * x386) +
					 (-1 * x0 * x360) + (-1 * x11 * x367) + (-1 * x45 * x338) + (-1 * x383);
	const FLT x391 = x30 * x344;
	const FLT x392 = x11 * x391;
	const FLT x393 = x363 * x364;
	const FLT x394 = x371 * x393;
	const FLT x395 = x4 * x344;
	const FLT x396 = x0 * x358;
	const FLT x397 = x364 * x340;
	const FLT x398 = x11 * x397;
	const FLT x399 = x8 * x398;
	const FLT x400 = x45 * x337;
	const FLT x401 = x11 * x400;
	const FLT x402 = x8 * x401;
	const FLT x403 = x42 * x337;
	const FLT x404 = x403 * x371;
	const FLT x405 = (-1 * x8 * x367) + (x395 * x396) + (-1 * x404) + (x39 * x338) + x394 + (-1 * x376 * x386) +
					 (-1 * x32 * x362) + (-1 * x392 * x358) + (-1 * x399) + x402 + (-1 * x36 * x365) + x37;
	const FLT x406 = -1 * x40;
	const FLT x407 = x11 * x393;
	const FLT x408 = x8 * x407;
	const FLT x409 = x11 * x395;
	const FLT x410 = x11 * x403;
	const FLT x411 = x8 * x410;
	const FLT x412 = (x371 * x397) + (-1 * x400 * x371);
	const FLT x413 = (-1 * x36 * x386) + (x368 * x359) + (x376 * x365) + (x396 * x391) + (-1 * x411) +
					 (-1 * x34 * x338) + x412 + x406 + (-1 * x35 * x362) + x408 + (x409 * x358);
	const FLT x414 = (x413 * sensor_pt[2]) + (-1 * x390 * sensor_pt[0]) + (x405 * sensor_pt[1]);
	const FLT x415 = (x380 * sensor_pt[0]) + (-1 * x405 * sensor_pt[2]) + (x413 * sensor_pt[1]);
	const FLT x416 = (x206 * x390) + (-1 * x415 * x200) + (-1 * x380 * x189) + (x414 * x210);
	const FLT x417 = (x413 * sensor_pt[0]) + (-1 * x380 * sensor_pt[1]) + (x390 * sensor_pt[2]);
	const FLT x418 = (x213 * x380) + (-1 * x414 * x217) + (-1 * x405 * x206) + (x417 * x200);
	const FLT x419 = (-1 * x417 * x210) + (x415 * x217) + (-1 * x213 * x390) + (x405 * x189);
	const FLT x420 = 2 * ((x419 * (*lh_p).Rot[0]) + (-1 * x416 * (*lh_p).Rot[2]) + (x418 * (*lh_p).Rot[1]));
	const FLT x421 = 2 * ((x419 * (*lh_p).Rot[2]) + (-1 * x418 * (*lh_p).Rot[3]) + (x416 * (*lh_p).Rot[0]));
	const FLT x422 = x418 + (-1 * x420 * (*lh_p).Rot[1]) + (x421 * (*lh_p).Rot[3]);
	const FLT x423 = 2 * ((x416 * (*lh_p).Rot[3]) + (-1 * x419 * (*lh_p).Rot[1]) + (x418 * (*lh_p).Rot[0]));
	const FLT x424 = x416 + (-1 * x423 * (*lh_p).Rot[3]) + (x420 * (*lh_p).Rot[2]);
	const FLT x425 = (-1 * x421 * (*lh_p).Rot[2]) + x419 + (x423 * (*lh_p).Rot[1]);
	const FLT x426 = (x425 * x110) + (x424 * x105);
	const FLT x427 = (x67 * x422) + (-1 * x113 * (x426 + (x99 * x422)));
	const FLT x428 = x427 * x117;
	const FLT x429 = (x427 * x124) + (x68 * ((-1 * x427 * x123) + x428));
	const FLT x430 = (x427 * x126) + (x68 * x429);
	const FLT x431 = (x77 * x422) + (-1 * x426 * x130);
	const FLT x432 = ((-1 * x425 * x135) + (x424 * x134)) * x136;
	const FLT x433 = (-1 * x432) + (x431 * x133);
	const FLT x434 =
		x432 + (-1 * x94 *
				((x427 * x147) + x431 +
				 (-1 * x143 *
				  ((x433 * x141) +
				   (-1 * x88 *
					((x427 * x129) + (x68 * x430) +
					 (x68 * (x430 + (x427 * x116) +
							 (x68 * (x429 + (x68 * ((x427 * x121) + x428 + (-1 * x427 * x120))) + (x427 * x122))))) +
					 (x427 * x128))))) +
				 (x92 * x430) + (-1 * x433 * x145)));
	const FLT x435 = x9 * x350;
	const FLT x436 = x9 * x354;
	const FLT x437 = x6 * x11;
	const FLT x438 = (x11 * x11 * x11) * x346;
	const FLT x439 = (-1 * x438 * x353) + (x11 * x352) + (x438 * x349) + (x11 * x435) + (-1 * x11 * x436) +
					 (-1 * x11 * x357) + (-1 * x437 * x354) + (x437 * x350);
	const FLT x440 = x439 * x395;
	const FLT x441 = x439 * x361;
	const FLT x442 = x11 * x385;
	const FLT x443 = x8 * x366;
	const FLT x444 = (x0 * x407) + (-1 * x0 * x410);
	const FLT x445 = x389 + (-1 * x439 * x443) + x444 + (-1 * x442 * x376) + x31 + (x0 * x440) + (-1 * x12 * x397) +
					 (x12 * x400) + (-1 * x439 * x392) + (-1 * x32 * x441);
	const FLT x446 = x11 * x366;
	const FLT x447 = x0 * x439;
	const FLT x448 = x32 * x344;
	const FLT x449 = x0 * x375;
	const FLT x450 = x0 * x378;
	const FLT x451 = (-1 * x442 * x363) + x449 + x37 + (-1 * x4 * x441) + (x439 * x382) + (-1 * x402) +
					 (-1 * x12 * x370) + (x12 * x372) + (-1 * x448 * x447) + (-1 * x439 * x446) + (-1 * x450) + x399;
	const FLT x452 = -1 * x28;
	const FLT x453 = x0 * x398;
	const FLT x454 = x0 * x401;
	const FLT x455 = (x12 * x393) + (-1 * x36 * x442) + (-1 * x454) + (-1 * x35 * x441) + (-1 * x12 * x403) + x452 +
					 (x447 * x391) + x453 + x379 + (x32 * x439 * x368) + (x11 * x440);
	const FLT x456 = (x455 * sensor_pt[2]) + (-1 * x451 * sensor_pt[0]) + (x445 * sensor_pt[1]);
	const FLT x457 = x11 * x448;
	const FLT x458 = (x0 * x387) + (-1 * x0 * x388);
	const FLT x459 = (-1 * x408) + (-1 * x439 * x369) + x406 + (-1 * x12 * x374) + (-1 * x30 * x441) + (x12 * x377) +
					 (-1 * x11 * x342) + x411 + (-1 * x447 * x366) + x458 + (x457 * x439);
	const FLT x460 = (x455 * sensor_pt[0]) + (x451 * sensor_pt[2]) + (-1 * x459 * sensor_pt[1]);
	const FLT x461 = (x460 * x200) + (x459 * x213) + (-1 * x445 * x206) + (-1 * x456 * x217);
	const FLT x462 = (x459 * sensor_pt[0]) + (-1 * x445 * sensor_pt[2]) + (x455 * sensor_pt[1]);
	const FLT x463 = (x456 * x210) + (-1 * x462 * x200) + (-1 * x459 * x189) + (x451 * x206);
	const FLT x464 = (x462 * x217) + (x445 * x189) + (-1 * x451 * x213) + (-1 * x460 * x210);
	const FLT x465 = (-1 * x461 * (*lh_p).Rot[3]) + (x464 * (*lh_p).Rot[2]) + (x463 * (*lh_p).Rot[0]);
	const FLT x466 = (x464 * (*lh_p).Rot[0]) + (-1 * x463 * (*lh_p).Rot[2]) + (x461 * (*lh_p).Rot[1]);
	const FLT x467 = x461 + (x465 * x251) + (-1 * x466 * x249);
	const FLT x468 = (x463 * (*lh_p).Rot[3]) + (x461 * (*lh_p).Rot[0]) + (-1 * x464 * (*lh_p).Rot[1]);
	const FLT x469 = x463 + (-1 * x468 * x251) + (x466 * x254);
	const FLT x470 = x464 + (-1 * x465 * x254) + (x468 * x249);
	const FLT x471 = (x470 * x110) + (x469 * x105);
	const FLT x472 = (x67 * x467) + (-1 * x113 * (x471 + (x99 * x467)));
	const FLT x473 = x472 * x117;
	const FLT x474 = (x472 * x124) + (x68 * ((-1 * x472 * x123) + x473));
	const FLT x475 = (x472 * x126) + (x68 * x474);
	const FLT x476 = (x77 * x467) + (-1 * x471 * x130);
	const FLT x477 = ((-1 * x470 * x135) + (x469 * x134)) * x136;
	const FLT x478 = (-1 * x477) + (x476 * x133);
	const FLT x479 =
		x477 +
		(-1 * x94 *
		 (x476 + (x472 * x147) + (-1 * x478 * x145) + (x92 * x475) +
		  (-1 * x143 *
		   ((x478 * x141) +
			(-1 * x88 *
			 ((x68 * x475) + (x472 * x128) + (x472 * x129) +
			  (x68 * (x475 + (x472 * x116) +
					  (x68 * (x474 + (x68 * (x473 + (x472 * x121) + (-1 * x472 * x120))) + (x472 * x122)))))))))));
	const FLT x480 = x0 * x0 * x0;
	const FLT x481 = x0 * x5;
	const FLT x482 = x0 * x12;
	const FLT x483 = x25 * x481;
	const FLT x484 = (x482 * x350) + (-1 * x483 * x348) + (-1 * x480 * x354) + (x480 * x350) + (x0 * x435) +
					 (-1 * x0 * x436) + (x481 * x351) + (-1 * x482 * x354);
	const FLT x485 = x0 * x484;
	const FLT x486 = 0.5 * x483;
	const FLT x487 = x484 * x361;
	const FLT x488 = (-1 * x32 * x487) + x454 + (x485 * x395) + (-1 * x484 * x392) + (-1 * x453) + (-1 * x484 * x443) +
					 (x6 * x393) + x452 + (-1 * x6 * x403) + x373 + (-1 * x486 * x376);
	const FLT x489 = x484 * x368;
	const FLT x490 = (-1 * x4 * x489) + x450 + (-1 * x483 * x341) + (x6 * x372) + x404 + (-1 * x394) + x37 +
					 (-1 * x485 * x366) + (-1 * x449) + (x457 * x484) + (-1 * x6 * x370) + (-1 * x30 * x487);
	const FLT x491 = x412 + (-1 * x486 * x363) + (-1 * x485 * x448) + (-1 * x6 * x377) + x458 + (x6 * x374) +
					 (x30 * x489) + (-1 * x4 * x487) + (-1 * x484 * x446) + x40;
	const FLT x492 = x444 + (x32 * x489) + x383 + (x485 * x391) + (-1 * x6 * x400) + (-1 * x36 * x486) + (-1 * x384) +
					 (-1 * x35 * x487) + (x6 * x397) + x381 + (x484 * x409);
	const FLT x493 = (x492 * sensor_pt[0]) + (-1 * x490 * sensor_pt[1]) + (x491 * sensor_pt[2]);
	const FLT x494 = (x492 * sensor_pt[2]) + (-1 * x491 * sensor_pt[0]) + (x488 * sensor_pt[1]);
	const FLT x495 = (-1 * x494 * x217) + (x493 * x200) + (-1 * x488 * x206) + (x490 * x213);
	const FLT x496 = (x490 * sensor_pt[0]) + (-1 * x488 * sensor_pt[2]) + (x492 * sensor_pt[1]);
	const FLT x497 = (-1 * x491 * x213) + (x496 * x217) + (x488 * x189) + (-1 * x493 * x210);
	const FLT x498 = (x491 * x206) + (-1 * x496 * x200) + (-1 * x490 * x189) + (x494 * x210);
	const FLT x499 = (x498 * (*lh_p).Rot[3]) + (x495 * (*lh_p).Rot[0]) + (-1 * x497 * (*lh_p).Rot[1]);
	const FLT x500 = (x497 * (*lh_p).Rot[0]) + (-1 * x498 * (*lh_p).Rot[2]) + (x495 * (*lh_p).Rot[1]);
	const FLT x501 = x498 + (-1 * x499 * x251) + (x500 * x254);
	const FLT x502 = (x497 * (*lh_p).Rot[2]) + (-1 * x495 * (*lh_p).Rot[3]) + (x498 * (*lh_p).Rot[0]);
	const FLT x503 = (x499 * x249) + x497 + (-1 * x502 * x254);
	const FLT x504 = (x503 * x110) + (x501 * x105);
	const FLT x505 = x495 + (-1 * x500 * x249) + (x502 * x251);
	const FLT x506 = (x77 * x505) + (-1 * x504 * x130);
	const FLT x507 = ((-1 * x503 * x135) + (x501 * x134)) * x136;
	const FLT x508 = x139 * ((-1 * x507) + (x506 * x133));
	const FLT x509 = (x67 * x505) + (-1 * x113 * (x504 + (x99 * x505)));
	const FLT x510 = x509 * x117;
	const FLT x511 = (x509 * x124) + (x68 * ((-1 * x509 * x123) + x510));
	const FLT x512 = (x509 * x126) + (x68 * x511);
	const FLT x513 =
		x507 +
		(-1 * x94 *
		 (x506 + (-1 * x508 * x144) + (x509 * x147) + (x92 * x512) +
		  (-1 * x143 *
		   ((x508 * x140) +
			(-1 * x88 *
			 ((x509 * x128) + (x509 * x129) + (x68 * x512) +
			  (x68 * (x512 + (x509 * x116) +
					  (x68 * (x511 + (x68 * ((x509 * x121) + x510 + (-1 * x509 * x120))) + (x509 * x122)))))))))));
	const FLT x514 = dt * x288;
	const FLT x515 = dt * x289;
	const FLT x516 = x515 + x514;
	const FLT x517 = -1 * dt * x291;
	const FLT x518 = dt + (-1 * dt * x293);
	const FLT x519 = x518 + x517;
	const FLT x520 = dt * x296;
	const FLT x521 = dt * x297;
	const FLT x522 = x521 + (-1 * x520);
	const FLT x523 = (x522 * x110) + (x519 * x105);
	const FLT x524 = (x67 * x516) + (-1 * x113 * (x523 + (x99 * x516)));
	const FLT x525 = x524 * x117;
	const FLT x526 = (x524 * x124) + (x68 * ((-1 * x524 * x123) + x525));
	const FLT x527 = (x524 * x126) + (x68 * x526);
	const FLT x528 = x524 * x115;
	const FLT x529 = (x77 * x516) + (-1 * x523 * x130);
	const FLT x530 = ((-1 * x522 * x135) + (x519 * x134)) * x136;
	const FLT x531 = (-1 * x530) + (x529 * x133);
	const FLT x532 =
		x530 + (-1 * x94 *
				((x528 * x146) + (x92 * x527) + x529 +
				 (-1 * x143 *
				  ((x531 * x141) +
				   (-1 * x88 *
					((x68 * (x527 + (x524 * x116) +
							 (x68 * (x526 + (x68 * ((-1 * x528 * x119) + (x524 * x121) + x525)) + (x524 * x122))))) +
					 (x524 * x129) + (x524 * x128) + (x68 * x527))))) +
				 (-1 * x531 * x145)));
	const FLT x533 = -1 * dt * x313;
	const FLT x534 = x518 + x533;
	const FLT x535 = x514 + (-1 * x515);
	const FLT x536 = dt * x309;
	const FLT x537 = dt * x310;
	const FLT x538 = x537 + x536;
	const FLT x539 = (x538 * x110) + (x535 * x105);
	const FLT x540 = (x67 * x534) + (-1 * x113 * (x539 + (x99 * x534)));
	const FLT x541 = x540 * x115;
	const FLT x542 = x540 * x117;
	const FLT x543 = (x71 * x541) + (x68 * ((-1 * x69 * x541) + x542));
	const FLT x544 = (x72 * x541) + (x68 * x543);
	const FLT x545 = (x77 * x534) + (-1 * x539 * x130);
	const FLT x546 = ((-1 * x538 * x135) + (x535 * x134)) * x136;
	const FLT x547 = (-1 * x546) + (x545 * x133);
	const FLT x548 =
		x546 + (-1 * x94 *
				(x545 + (x541 * x146) + (x92 * x544) +
				 (-1 * x143 *
				  ((x547 * x141) +
				   (-1 * x88 *
					((x73 * x541) + (x68 * x544) +
					 (x68 * (x544 + (x84 * x541) +
							 (x68 * ((x68 * ((x540 * x121) + (-1 * x541 * x119) + x542)) + x543 + (x83 * x541))))) +
					 (x85 * x541))))) +
				 (-1 * x547 * x145)));
	const FLT x549 = x536 + (-1 * x537);
	const FLT x550 = x520 + x521;
	const FLT x551 = x517 + dt + x533;
	const FLT x552 = (x551 * x110) + (x550 * x105);
	const FLT x553 = (x67 * x549) + (-1 * x113 * (x552 + (x99 * x549)));
	const FLT x554 = x553 * x117;
	const FLT x555 = (x553 * x124) + (x68 * ((-1 * x553 * x123) + x554));
	const FLT x556 = (x553 * x126) + (x68 * x555);
	const FLT x557 = (x77 * x549) + (-1 * x552 * x130);
	const FLT x558 = ((-1 * x551 * x135) + (x550 * x134)) * x136;
	const FLT x559 = (-1 * x558) + (x557 * x133);
	const FLT x560 =
		x558 + (-1 * x94 *
				((x553 * x147) +
				 (-1 * x143 *
				  ((x559 * x141) +
				   (-1 * x88 *
					((x553 * x129) + (x68 * x556) +
					 (x68 * (x556 + (x553 * x116) +
							 (x68 * (x555 + (x68 * ((x553 * x121) + x554 + (-1 * x553 * x120))) + (x553 * x122))))) +
					 (x553 * x128))))) +
				 x557 + (x92 * x556) + (-1 * x559 * x145)));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Acc[0]) / sizeof(FLT), x148 + (x148 * x149));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Acc[1]) / sizeof(FLT), x165 + (x165 * x149));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Acc[2]) / sizeof(FLT), x178 + (x178 * x149));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						x235 + (x235 * x149));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						x265 + (x265 * x149));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						x287 + (x287 * x149));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT), x307 + (x307 * x149));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT), x323 + (x323 * x149));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT), x335 + (x335 * x149));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x434 + (x434 * x149));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x479 + (x479 * x149));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x513 + (x513 * x149));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.Pos[0]) / sizeof(FLT), x532 + (x532 * x149));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.Pos[1]) / sizeof(FLT), x548 + (x548 * x149));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.Pos[2]) / sizeof(FLT), x560 + (x560 * x149));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen2 wrt [(*error_model).AccBias[0],
// (*error_model).AccBias[1], (*error_model).AccBias[2], (*error_model).Acc[0], (*error_model).Acc[1],
// (*error_model).Acc[2], (*error_model).GyroBias[0], (*error_model).GyroBias[1], (*error_model).GyroBias[2],
// (*error_model).IMUCorrection[0], (*error_model).IMUCorrection[1], (*error_model).IMUCorrection[2],
// (*error_model).IMUCorrection[3], (*error_model).Pose.AxisAngleRot[0], (*error_model).Pose.AxisAngleRot[1],
// (*error_model).Pose.AxisAngleRot[2], (*error_model).Pose.Pos[0], (*error_model).Pose.Pos[1],
// (*error_model).Pose.Pos[2], (*error_model).Velocity.AxisAngleRot[0], (*error_model).Velocity.AxisAngleRot[1],
// (*error_model).Velocity.AxisAngleRot[2], (*error_model).Velocity.Pos[0], (*error_model).Velocity.Pos[1],
// (*error_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f354c0>]

static inline void SurviveKalmanErrorModel_LightMeas_x_gen2_jac_error_model_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_x_gen2(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_x_gen2_jac_error_model(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen2 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]
static inline void SurviveKalmanErrorModel_LightMeas_x_gen2_jac_sensor_pt(CnMat *Hx, const FLT dt,
																		  const SurviveKalmanModel *_x0,
																		  const SurviveKalmanErrorModel *error_model,
																		  const FLT *sensor_pt, const SurvivePose *lh_p,
																		  const BaseStationCal *bsc0) {
	const FLT x0 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x1 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x2 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x3 =
		(-1 * x2 * (*_x0).Pose.Rot[1]) + (x0 * (*_x0).Pose.Rot[3]) + (x1 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2];
	const FLT x4 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x5 = dt * dt;
	const FLT x6 = (x4 * x4) * x5;
	const FLT x7 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x8 = x5 * (x7 * x7);
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x10 = x5 * (x9 * x9);
	const FLT x11 = 1e-10 + x10 + x6 + x8;
	const FLT x12 = sqrt(x11);
	const FLT x13 = 0.5 * x12;
	const FLT x14 = sin(x13);
	const FLT x15 = (x14 * x14) * (1. / x11);
	const FLT x16 = cos(x13);
	const FLT x17 = 1. / sqrt((x8 * x15) + (x16 * x16) + (x6 * x15) + (x15 * x10));
	const FLT x18 = dt * x14 * (1. / x12) * x17;
	const FLT x19 = x4 * x18;
	const FLT x20 = x3 * x19;
	const FLT x21 =
		(*_x0).Pose.Rot[3] + (x1 * (*_x0).Pose.Rot[1]) + (-1 * x0 * (*_x0).Pose.Rot[2]) + (x2 * (*_x0).Pose.Rot[0]);
	const FLT x22 = x21 * x18;
	const FLT x23 = x9 * x22;
	const FLT x24 =
		(*_x0).Pose.Rot[1] + (x0 * (*_x0).Pose.Rot[0]) + (-1 * x1 * (*_x0).Pose.Rot[3]) + (x2 * (*_x0).Pose.Rot[2]);
	const FLT x25 = x17 * x16;
	const FLT x26 = x24 * x25;
	const FLT x27 = (-1 * x0 * (*_x0).Pose.Rot[1]) + (-1 * x1 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x2 * (*_x0).Pose.Rot[3]);
	const FLT x28 = x27 * x18;
	const FLT x29 = x7 * x28;
	const FLT x30 = x26 + (-1 * x20) + x29 + x23;
	const FLT x31 = x3 * x18;
	const FLT x32 = x24 * x18;
	const FLT x33 = (-1 * x7 * x32) + (-1 * x21 * x19) + (-1 * x9 * x31) + (x25 * x27);
	const FLT x34 = x4 * x32;
	const FLT x35 = x9 * x28;
	const FLT x36 = x3 * x25;
	const FLT x37 = x7 * x22;
	const FLT x38 = (-1 * x37) + x36 + x34 + x35;
	const FLT x39 = (x30 * sensor_pt[1]) + (-1 * x38 * sensor_pt[0]) + (x33 * sensor_pt[2]);
	const FLT x40 = x9 * x32;
	const FLT x41 = x4 * x28;
	const FLT x42 = x25 * x21;
	const FLT x43 = x7 * x31;
	const FLT x44 = x43 + x42 + (-1 * x40) + x41;
	const FLT x45 = (-1 * x44 * sensor_pt[1]) + (x38 * sensor_pt[2]) + (x33 * sensor_pt[0]);
	const FLT x46 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x47 = (*_x0).Pose.Pos[1] + (2 * ((x44 * x45) + (-1 * x30 * x39))) + (*error_model).Pose.Pos[1] +
					sensor_pt[1] + (dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) +
					(x46 * ((*_x0).Acc[1] + (*error_model).Acc[1]));
	const FLT x48 = (-1 * x30 * sensor_pt[2]) + (x33 * sensor_pt[1]) + (x44 * sensor_pt[0]);
	const FLT x49 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (2 * ((x48 * x30) + (-1 * x45 * x38))) +
					(*error_model).Pose.Pos[2] + (x46 * ((*_x0).Acc[2] + (*error_model).Acc[2])) +
					(dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2]));
	const FLT x50 = (x46 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + (2 * ((x38 * x39) + (-1 * x44 * x48))) +
					sensor_pt[0] + (dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) +
					(*error_model).Pose.Pos[0] + (*_x0).Pose.Pos[0];
	const FLT x51 = (-1 * x50 * (*lh_p).Rot[2]) + (x47 * (*lh_p).Rot[1]) + (x49 * (*lh_p).Rot[0]);
	const FLT x52 = (-1 * x47 * (*lh_p).Rot[3]) + (x50 * (*lh_p).Rot[0]) + (x49 * (*lh_p).Rot[2]);
	const FLT x53 = x47 + (*lh_p).Pos[1] + (2 * ((x52 * (*lh_p).Rot[3]) + (-1 * x51 * (*lh_p).Rot[1])));
	const FLT x54 = x53 * x53;
	const FLT x55 = (-1 * x49 * (*lh_p).Rot[1]) + (x47 * (*lh_p).Rot[0]) + (x50 * (*lh_p).Rot[3]);
	const FLT x56 = x49 + (2 * ((x55 * (*lh_p).Rot[1]) + (-1 * x52 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x57 = x50 + (*lh_p).Pos[0] + (2 * ((x51 * (*lh_p).Rot[2]) + (-1 * x55 * (*lh_p).Rot[3])));
	const FLT x58 = x57 * x57;
	const FLT x59 = x58 + (x56 * x56);
	const FLT x60 = x59 + x54;
	const FLT x61 = 0.523598775598299 + (*bsc0).tilt;
	const FLT x62 = cos(x61);
	const FLT x63 = 1. / x62;
	const FLT x64 = (1. / sqrt(x60)) * x63;
	const FLT x65 = asin(x64 * x53);
	const FLT x66 = -8.0108022e-06 + (-1.60216044e-05 * x65);
	const FLT x67 = 8.0108022e-06 * x65;
	const FLT x68 = -8.0108022e-06 + (-1 * x67);
	const FLT x69 = 0.0028679863 + (x68 * x65);
	const FLT x70 = x69 + (x65 * x66);
	const FLT x71 = 5.3685255e-06 + (x65 * x69);
	const FLT x72 = x71 + (x70 * x65);
	const FLT x73 = 0.0076069798 + (x71 * x65);
	const FLT x74 = x73 + (x72 * x65);
	const FLT x75 = 1. / sqrt(1 + (-1 * (1. / x60) * (1. / (x62 * x62)) * x54));
	const FLT x76 = (-1 * x36) + x37 + (-1 * x35) + (-1 * x34);
	const FLT x77 = 2 * x30;
	const FLT x78 = 2 * x33;
	const FLT x79 = x78 * x44;
	const FLT x80 = x79 + (-1 * x77 * x76);
	const FLT x81 = 2 * x38;
	const FLT x82 = 1 + (x81 * x76) + (-2 * (x44 * x44));
	const FLT x83 = x81 * x33;
	const FLT x84 = 2 * x44;
	const FLT x85 = (x84 * x30) + (-1 * x83);
	const FLT x86 = (x85 * (*lh_p).Rot[2]) + (-1 * x80 * (*lh_p).Rot[3]) + (x82 * (*lh_p).Rot[0]);
	const FLT x87 = 2 * (*lh_p).Rot[3];
	const FLT x88 = (-1 * x82 * (*lh_p).Rot[2]) + (x85 * (*lh_p).Rot[0]) + (x80 * (*lh_p).Rot[1]);
	const FLT x89 = 2 * (*lh_p).Rot[1];
	const FLT x90 = x80 + (x86 * x87) + (-1 * x88 * x89);
	const FLT x91 = 2 * x53;
	const FLT x92 = 2 * (*lh_p).Rot[2];
	const FLT x93 = (x82 * (*lh_p).Rot[3]) + (-1 * x85 * (*lh_p).Rot[1]) + (x80 * (*lh_p).Rot[0]);
	const FLT x94 = x82 + (x88 * x92) + (-1 * x87 * x93);
	const FLT x95 = 2 * x57;
	const FLT x96 = x85 + (x89 * x93) + (-1 * x86 * x92);
	const FLT x97 = 2 * x56;
	const FLT x98 = (x97 * x96) + (x95 * x94);
	const FLT x99 = 1.0 / 2.0 * x53;
	const FLT x100 = (1. / (x60 * sqrt(x60))) * x63 * x99;
	const FLT x101 = (x64 * x90) + (-1 * x100 * (x98 + (x91 * x90)));
	const FLT x102 = x75 * x101;
	const FLT x103 = x68 * x102;
	const FLT x104 = (x69 * x102) + (x65 * ((-1 * x67 * x102) + x103));
	const FLT x105 = (x71 * x102) + (x65 * x104);
	const FLT x106 = 2.40324066e-05 * x65;
	const FLT x107 = x70 * x75;
	const FLT x108 = x73 * x75;
	const FLT x109 = sin(x61);
	const FLT x110 = atan2(-1 * x56, x57);
	const FLT x111 = tan(x61);
	const FLT x112 = (1. / sqrt(x59)) * x111;
	const FLT x113 = x53 * x112;
	const FLT x114 = asin(x113) + (-1 * (*bsc0).ogeephase) + (-1 * x110);
	const FLT x115 = (-1 * sin(x114) * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x116 = x109 * x115;
	const FLT x117 = (1. / (x59 * sqrt(x59))) * x99 * x111;
	const FLT x118 = (x90 * x112) + (-1 * x98 * x117);
	const FLT x119 = 1. / x59;
	const FLT x120 = 1. / sqrt(1 + (-1 * x54 * (x111 * x111) * x119));
	const FLT x121 = (1. / x58) * x56;
	const FLT x122 = 1. / x57;
	const FLT x123 = x58 * x119;
	const FLT x124 = ((-1 * x96 * x122) + (x94 * x121)) * x123;
	const FLT x125 = (-1 * x124) + (x118 * x120);
	const FLT x126 = cos(x114) * (*bsc0).ogeemag;
	const FLT x127 = x73 * x65;
	const FLT x128 = (x74 * x65) + x127;
	const FLT x129 = x109 * x128;
	const FLT x130 = x126 * x129;
	const FLT x131 = x62 + (-1 * x116 * x128);
	const FLT x132 = x65 * x65;
	const FLT x133 = x115 * x132;
	const FLT x134 = x73 * x133 * (1. / (x131 * x131));
	const FLT x135 = 1. / x131;
	const FLT x136 = x133 * x135;
	const FLT x137 = x73 * x132 * x135;
	const FLT x138 = x126 * x137;
	const FLT x139 = 2 * x115 * x127 * x135;
	const FLT x140 = x113 + (x73 * x136);
	const FLT x141 = 1. / sqrt(1 + (-1 * (x140 * x140)));
	const FLT x142 =
		x124 + (-1 * x141 *
				(x118 + (x102 * x139) + (-1 * x125 * x138) +
				 (-1 * x134 *
				  ((x125 * x130) +
				   (-1 * x116 *
					((x101 * x108) +
					 (x65 * ((x72 * x102) + x105 +
							 (x65 * (x104 + (x65 * ((x66 * x102) + x103 + (-1 * x102 * x106))) + (x101 * x107))))) +
					 (x74 * x102) + (x65 * x105))))) +
				 (x105 * x136)));
	const FLT x143 = cos((*bsc0).gibpha + (-1 * asin(x140)) + x110) * (*bsc0).gibmag;
	const FLT x144 = x40 + (-1 * x43) + (-1 * x41) + (-1 * x42);
	const FLT x145 = 1 + (x84 * x144) + (-2 * (x30 * x30));
	const FLT x146 = x78 * x30;
	const FLT x147 = x146 + (-1 * x81 * x144);
	const FLT x148 = (x81 * x30) + (-1 * x79);
	const FLT x149 = (x148 * (*lh_p).Rot[3]) + (x145 * (*lh_p).Rot[0]) + (-1 * x147 * (*lh_p).Rot[1]);
	const FLT x150 = (x147 * (*lh_p).Rot[0]) + (-1 * x148 * (*lh_p).Rot[2]) + (x145 * (*lh_p).Rot[1]);
	const FLT x151 = x148 + (-1 * x87 * x149) + (x92 * x150);
	const FLT x152 = (-1 * x145 * (*lh_p).Rot[3]) + (x147 * (*lh_p).Rot[2]) + (x148 * (*lh_p).Rot[0]);
	const FLT x153 = x147 + (x89 * x149) + (-1 * x92 * x152);
	const FLT x154 = (x97 * x153) + (x95 * x151);
	const FLT x155 = x145 + (x87 * x152) + (-1 * x89 * x150);
	const FLT x156 = (x112 * x155) + (-1 * x117 * x154);
	const FLT x157 = ((-1 * x122 * x153) + (x121 * x151)) * x123;
	const FLT x158 = x126 * ((-1 * x157) + (x120 * x156));
	const FLT x159 = (x64 * x155) + (-1 * x100 * (x154 + (x91 * x155)));
	const FLT x160 = x75 * x159;
	const FLT x161 = x68 * x160;
	const FLT x162 = (x69 * x160) + (x65 * ((-1 * x67 * x160) + x161));
	const FLT x163 = (x71 * x160) + (x65 * x162);
	const FLT x164 =
		x157 + (-1 * x141 *
				(x156 + (x163 * x136) + (x160 * x139) + (-1 * x137 * x158) +
				 (-1 * x134 *
				  ((x129 * x158) +
				   (-1 * x116 *
					((x108 * x159) +
					 (x65 * ((x72 * x160) + x163 +
							 (x65 * ((x65 * ((x66 * x160) + x161 + (-1 * x106 * x160))) + x162 + (x107 * x159))))) +
					 (x74 * x160) + (x65 * x163)))))));
	const FLT x165 = (x81 * x44) + (-1 * x146);
	const FLT x166 = (-1 * x29) + (-1 * x23) + (-1 * x26) + x20;
	const FLT x167 = x83 + (-1 * x84 * x166);
	const FLT x168 = 1 + (x77 * x166) + (-2 * (x38 * x38));
	const FLT x169 = (x168 * (*lh_p).Rot[2]) + (-1 * x165 * (*lh_p).Rot[3]) + (x167 * (*lh_p).Rot[0]);
	const FLT x170 = (x165 * (*lh_p).Rot[1]) + (x168 * (*lh_p).Rot[0]) + (-1 * x167 * (*lh_p).Rot[2]);
	const FLT x171 = x165 + (x87 * x169) + (-1 * x89 * x170);
	const FLT x172 = (x167 * (*lh_p).Rot[3]) + (x165 * (*lh_p).Rot[0]) + (-1 * x168 * (*lh_p).Rot[1]);
	const FLT x173 = x167 + (-1 * x87 * x172) + (x92 * x170);
	const FLT x174 = x168 + (x89 * x172) + (-1 * x92 * x169);
	const FLT x175 = (x97 * x174) + (x95 * x173);
	const FLT x176 = (x64 * x171) + (-1 * x100 * (x175 + (x91 * x171)));
	const FLT x177 = x75 * x176;
	const FLT x178 = x68 * x177;
	const FLT x179 = (x69 * x177) + (x65 * ((-1 * x67 * x177) + x178));
	const FLT x180 = (x71 * x177) + (x65 * x179);
	const FLT x181 = (x112 * x171) + (-1 * x117 * x175);
	const FLT x182 = ((-1 * x122 * x174) + (x121 * x173)) * x123;
	const FLT x183 = (-1 * x182) + (x120 * x181);
	const FLT x184 =
		x182 + (-1 * x141 *
				(x181 + (-1 * x183 * x138) +
				 (-1 * x134 *
				  ((x183 * x130) +
				   (-1 * x116 *
					((x108 * x176) + (x65 * x180) +
					 (x65 * (x180 + (x72 * x177) +
							 (x65 * (x179 + (x65 * ((x66 * x177) + (-1 * x106 * x177) + x178)) + (x107 * x176))))) +
					 (x74 * x177))))) +
				 (x177 * x139) + (x180 * x136)));
	cnMatrixOptionalSet(Hx, 0, 0, x142 + (x142 * x143));
	cnMatrixOptionalSet(Hx, 0, 1, x164 + (x164 * x143));
	cnMatrixOptionalSet(Hx, 0, 2, x184 + (x184 * x143));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen2 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]

static inline void SurviveKalmanErrorModel_LightMeas_x_gen2_jac_sensor_pt_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_x_gen2(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_x_gen2_jac_sensor_pt(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen2 wrt [(*lh_p).Pos[0], (*lh_p).Pos[1], (*lh_p).Pos[2],
// (*lh_p).Rot[0], (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]
static inline void SurviveKalmanErrorModel_LightMeas_x_gen2_jac_lh_p(CnMat *Hx, const FLT dt,
																	 const SurviveKalmanModel *_x0,
																	 const SurviveKalmanErrorModel *error_model,
																	 const FLT *sensor_pt, const SurvivePose *lh_p,
																	 const BaseStationCal *bsc0) {
	const FLT x0 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x1 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x2 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x4 =
		(-1 * x3 * (*_x0).Pose.Rot[1]) + (x2 * (*_x0).Pose.Rot[0]) + (x1 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x5 = dt * dt;
	const FLT x6 = (x0 * x0) * x5;
	const FLT x7 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x8 = x5 * (x7 * x7);
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x10 = x5 * (x9 * x9);
	const FLT x11 = 1e-10 + x10 + x6 + x8;
	const FLT x12 = sqrt(x11);
	const FLT x13 = 0.5 * x12;
	const FLT x14 = sin(x13);
	const FLT x15 = (x14 * x14) * (1. / x11);
	const FLT x16 = cos(x13);
	const FLT x17 = 1. / sqrt((x8 * x15) + (x16 * x16) + (x6 * x15) + (x15 * x10));
	const FLT x18 = dt * x14 * (1. / x12) * x17;
	const FLT x19 = x4 * x18;
	const FLT x20 = 0.5 * (*_x0).Pose.Rot[0];
	const FLT x21 = (x2 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x1 * (*_x0).Pose.Rot[2]) +
					(x20 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x22 = x21 * x18;
	const FLT x23 = (*_x0).Pose.Rot[1] + (x20 * (*error_model).Pose.AxisAngleRot[0]) + (-1 * x2 * (*_x0).Pose.Rot[3]) +
					(x3 * (*_x0).Pose.Rot[2]);
	const FLT x24 = x17 * x16;
	const FLT x25 = (-1 * x1 * (*_x0).Pose.Rot[1]) + (-1 * x2 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x3 * (*_x0).Pose.Rot[3]);
	const FLT x26 = x25 * x18;
	const FLT x27 = (x7 * x26) + (x24 * x23) + (-1 * x0 * x19) + (x9 * x22);
	const FLT x28 = x23 * x18;
	const FLT x29 = (-1 * x7 * x28) + (-1 * x0 * x22) + (-1 * x9 * x19) + (x24 * x25);
	const FLT x30 = (x4 * x24) + (-1 * x7 * x22) + (x0 * x28) + (x9 * x26);
	const FLT x31 = (-1 * x30 * sensor_pt[0]) + (x27 * sensor_pt[1]) + (x29 * sensor_pt[2]);
	const FLT x32 = (x24 * x21) + (x7 * x19) + (-1 * x9 * x28) + (x0 * x26);
	const FLT x33 = (-1 * x32 * sensor_pt[1]) + (x30 * sensor_pt[2]) + (x29 * sensor_pt[0]);
	const FLT x34 = 2 * ((x32 * x33) + (-1 * x31 * x27));
	const FLT x35 = dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1]);
	const FLT x36 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x37 = x36 * ((*_x0).Acc[1] + (*error_model).Acc[1]);
	const FLT x38 = x34 + (*error_model).Pose.Pos[1] + (*_x0).Pose.Pos[1] + x35 + sensor_pt[1] + x37;
	const FLT x39 = x38 * (*lh_p).Rot[1];
	const FLT x40 = (-1 * x27 * sensor_pt[2]) + (x29 * sensor_pt[1]) + (x32 * sensor_pt[0]);
	const FLT x41 = 2 * ((x40 * x27) + (-1 * x30 * x33));
	const FLT x42 = x36 * ((*_x0).Acc[2] + (*error_model).Acc[2]);
	const FLT x43 = dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2]);
	const FLT x44 = sensor_pt[2] + (*_x0).Pose.Pos[2] + x41 + x43 + (*error_model).Pose.Pos[2] + x42;
	const FLT x45 = x44 * (*lh_p).Rot[0];
	const FLT x46 = 2 * ((x30 * x31) + (-1 * x40 * x32));
	const FLT x47 = dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0]);
	const FLT x48 = x36 * ((*_x0).Acc[0] + (*error_model).Acc[0]);
	const FLT x49 = x48 + x46 + (*_x0).Pose.Pos[0] + (*error_model).Pose.Pos[0] + sensor_pt[0] + x47;
	const FLT x50 = x49 * (*lh_p).Rot[2];
	const FLT x51 = x39 + (-1 * x50) + x45;
	const FLT x52 = x49 * (*lh_p).Rot[0];
	const FLT x53 = x44 * (*lh_p).Rot[2];
	const FLT x54 = x38 * (*lh_p).Rot[3];
	const FLT x55 = (-1 * x54) + x52 + x53;
	const FLT x56 = x38 + (*lh_p).Pos[1] + (2 * ((x55 * (*lh_p).Rot[3]) + (-1 * x51 * (*lh_p).Rot[1])));
	const FLT x57 = x56 * x56;
	const FLT x58 = x38 * (*lh_p).Rot[0];
	const FLT x59 = x49 * (*lh_p).Rot[3];
	const FLT x60 = x44 * (*lh_p).Rot[1];
	const FLT x61 = (-1 * x60) + x58 + x59;
	const FLT x62 = x44 + (2 * ((x61 * (*lh_p).Rot[1]) + (-1 * x55 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x63 = x49 + (*lh_p).Pos[0] + (2 * ((x51 * (*lh_p).Rot[2]) + (-1 * x61 * (*lh_p).Rot[3])));
	const FLT x64 = x63 * x63;
	const FLT x65 = x64 + (x62 * x62);
	const FLT x66 = x65 + x57;
	const FLT x67 = 0.523598775598299 + (*bsc0).tilt;
	const FLT x68 = cos(x67);
	const FLT x69 = 1. / x68;
	const FLT x70 = (1. / sqrt(x66)) * x69;
	const FLT x71 = asin(x70 * x56);
	const FLT x72 = 8.0108022e-06 * x71;
	const FLT x73 = -8.0108022e-06 + (-1 * x72);
	const FLT x74 = 0.0028679863 + (x71 * x73);
	const FLT x75 = 5.3685255e-06 + (x71 * x74);
	const FLT x76 = 0.0076069798 + (x71 * x75);
	const FLT x77 = x71 * x76;
	const FLT x78 = -8.0108022e-06 + (-1.60216044e-05 * x71);
	const FLT x79 = x74 + (x71 * x78);
	const FLT x80 = x75 + (x71 * x79);
	const FLT x81 = x76 + (x80 * x71);
	const FLT x82 = (x81 * x71) + x77;
	const FLT x83 = sin(x67);
	const FLT x84 = atan2(-1 * x62, x63);
	const FLT x85 = tan(x67);
	const FLT x86 = x85 * (1. / sqrt(x65));
	const FLT x87 = x86 * x56;
	const FLT x88 = asin(x87) + (-1 * (*bsc0).ogeephase) + (-1 * x84);
	const FLT x89 = (-1 * sin(x88) * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x90 = x83 * x89;
	const FLT x91 = x68 + (-1 * x82 * x90);
	const FLT x92 = 1. / x91;
	const FLT x93 = x71 * x71;
	const FLT x94 = x89 * x93;
	const FLT x95 = x92 * x94;
	const FLT x96 = x87 + (x76 * x95);
	const FLT x97 = 1. / sqrt(1 + (-1 * (x96 * x96)));
	const FLT x98 = x85 * (1. / (x65 * sqrt(x65)));
	const FLT x99 = x56 * x98;
	const FLT x100 = x63 * x99;
	const FLT x101 = 2 * x63;
	const FLT x102 = 1. / sqrt(1 + (-1 * (1. / (x68 * x68)) * (1. / x66) * x57));
	const FLT x103 = (1. / (x66 * sqrt(x66))) * x69;
	const FLT x104 = x56 * x103;
	const FLT x105 = x89 * x77 * x92;
	const FLT x106 = x102 * x105 * x104;
	const FLT x107 = 1. / x65;
	const FLT x108 = 1. / sqrt(1 + (-1 * (x85 * x85) * x57 * x107));
	const FLT x109 = x62 * x107;
	const FLT x110 = (-1 * x109) + (-1 * x100 * x108);
	const FLT x111 = cos(x88) * (*bsc0).ogeemag;
	const FLT x112 = x76 * x93 * x92;
	const FLT x113 = x111 * x112;
	const FLT x114 = x63 * x104;
	const FLT x115 = x102 * x114;
	const FLT x116 = -1 * x73 * x115;
	const FLT x117 = (-1 * x74 * x115) + (x71 * ((x72 * x115) + x116));
	const FLT x118 = (-1 * x75 * x115) + (x71 * x117);
	const FLT x119 = 2.40324066e-05 * x71;
	const FLT x120 = x78 * x102;
	const FLT x121 = x79 * x102;
	const FLT x122 = x81 * x102;
	const FLT x123 = x82 * x83;
	const FLT x124 = x111 * x123;
	const FLT x125 = x76 * (1. / (x91 * x91)) * x94;
	const FLT x126 =
		x109 +
		(-1 * x97 *
		 ((-1 * x125 *
		   ((x110 * x124) +
			(-1 * x90 *
			 ((-1 * x76 * x115) + (-1 * x114 * x122) +
			  (x71 * ((-1 * x80 * x115) + x118 +
					  (x71 * (x117 + (x71 * (x116 + (-1 * x114 * x120) + (x119 * x115))) + (-1 * x114 * x121))))) +
			  (x71 * x118))))) +
		  (-1 * x110 * x113) + (x95 * x118) + (-1 * x100) + (-1 * x101 * x106)));
	const FLT x127 = cos((-1 * asin(x96)) + (*bsc0).gibpha + x84) * (*bsc0).gibmag;
	const FLT x128 = x70 + (-1 * x57 * x103);
	const FLT x129 = x102 * x128;
	const FLT x130 = 2 * x105;
	const FLT x131 = x86 * x108;
	const FLT x132 = x73 * x129;
	const FLT x133 = (x74 * x129) + (x71 * ((-1 * x72 * x129) + x132));
	const FLT x134 = (x75 * x129) + (x71 * x133);
	const FLT x135 =
		x97 * ((-1 * x125 *
				((x124 * x131) +
				 (-1 * x90 *
				  ((x76 * x129) +
				   (x71 * (x134 + (x80 * x129) +
						   (x71 * (x133 + (x71 * ((x78 * x129) + x132 + (-1 * x119 * x129))) + (x121 * x128))))) +
				   (x122 * x128) + (x71 * x134))))) +
			   (-1 * x113 * x131) + x86 + (x95 * x134) + (x129 * x130));
	const FLT x136 = x62 * x56;
	const FLT x137 = x98 * x136;
	const FLT x138 = 2 * x62;
	const FLT x139 = x63 * x107;
	const FLT x140 = x139 + (-1 * x108 * x137);
	const FLT x141 = x103 * x136;
	const FLT x142 = x102 * x141;
	const FLT x143 = -1 * x73 * x142;
	const FLT x144 = (-1 * x74 * x142) + (x71 * ((x72 * x142) + x143));
	const FLT x145 = (-1 * x75 * x142) + (x71 * x144);
	const FLT x146 =
		(-1 * x139) +
		(-1 * x97 *
		 ((-1 * x125 *
		   ((x124 * x140) +
			(-1 * x90 *
			 ((x71 * x145) + (-1 * x76 * x142) +
			  (x71 * ((-1 * x80 * x142) + x145 +
					  (x71 * ((x71 * ((-1 * x120 * x141) + x143 + (x119 * x142))) + x144 + (-1 * x121 * x141))))) +
			  (-1 * x122 * x141))))) +
		  (x95 * x145) + (-1 * x113 * x140) + (-1 * x137) + (-1 * x106 * x138)));
	const FLT x147 = 2 * x60;
	const FLT x148 = (2 * x59) + (-1 * x147);
	const FLT x149 = 2 * x56;
	const FLT x150 = 2 * x54;
	const FLT x151 = (2 * x53) + (-1 * x150);
	const FLT x152 = 2 * x50;
	const FLT x153 = (2 * x39) + (-1 * x152);
	const FLT x154 = (x138 * x153) + (x101 * x151);
	const FLT x155 = 1.0 / 2.0 * x104;
	const FLT x156 = (x70 * x148) + (-1 * x155 * (x154 + (x148 * x149)));
	const FLT x157 = x102 * x156;
	const FLT x158 = x73 * x157;
	const FLT x159 = (x74 * x157) + (x71 * ((-1 * x72 * x157) + x158));
	const FLT x160 = (x75 * x157) + (x71 * x159);
	const FLT x161 = 1.0 / 2.0 * x99;
	const FLT x162 = (x86 * x148) + (-1 * x161 * x154);
	const FLT x163 = (1. / x64) * x62;
	const FLT x164 = 1. / x63;
	const FLT x165 = x64 * x107;
	const FLT x166 = ((-1 * x164 * x153) + (x163 * x151)) * x165;
	const FLT x167 = (-1 * x166) + (x108 * x162);
	const FLT x168 =
		x166 + (-1 * x97 *
				(x162 + (x95 * x160) + (x130 * x157) +
				 (-1 * x125 *
				  ((x124 * x167) +
				   (-1 * x90 *
					((x76 * x157) + (x71 * x160) +
					 (x71 * (x160 + (x80 * x157) +
							 (x71 * (x159 + (x71 * (x158 + (x78 * x157) + (-1 * x119 * x157))) + (x121 * x156))))) +
					 (x122 * x156))))) +
				 (-1 * x113 * x167)));
	const FLT x169 = 2 * x45;
	const FLT x170 = (-1 * x169) + (-4 * x39) + x152;
	const FLT x171 = 2 * ((-1 * x42) + (-1 * sensor_pt[2]) + (-1 * x41) + (-1 * (*_x0).Pose.Pos[2]) + (-1 * x43) +
						  (-1 * (*error_model).Pose.Pos[2]));
	const FLT x172 = 2 * (*lh_p).Rot[2];
	const FLT x173 = (x38 * x172) + (-1 * x171 * (*lh_p).Rot[3]);
	const FLT x174 = 2 * x58;
	const FLT x175 = x148 + (x171 * (*lh_p).Rot[1]) + x174;
	const FLT x176 = (x175 * x138) + (x101 * x173);
	const FLT x177 = (x70 * x170) + (-1 * x155 * (x176 + (x170 * x149)));
	const FLT x178 = x102 * x177;
	const FLT x179 = x73 * x178;
	const FLT x180 = (x74 * x178) + (x71 * ((-1 * x72 * x178) + x179));
	const FLT x181 = (x75 * x178) + (x71 * x180);
	const FLT x182 = (x86 * x170) + (-1 * x161 * x176);
	const FLT x183 = ((-1 * x164 * x175) + (x163 * x173)) * x165;
	const FLT x184 = (-1 * x183) + (x108 * x182);
	const FLT x185 =
		x183 + (-1 * x97 *
				((x178 * x130) + (-1 * x113 * x184) +
				 (-1 * x125 *
				  ((x124 * x184) +
				   (-1 * x90 *
					((x76 * x178) + (x71 * x181) +
					 (x71 * (x181 + (x80 * x178) +
							 (x71 * ((x71 * ((x78 * x178) + (-1 * x119 * x178) + x179)) + x180 + (x121 * x177))))) +
					 (x122 * x177))))) +
				 x182 + (x95 * x181)));
	const FLT x186 = (-1 * (*error_model).Pose.Pos[0]) + (-1 * x46) + (-1 * (*_x0).Pose.Pos[0]) + (-1 * sensor_pt[0]) +
					 (-1 * x47) + (-1 * x48);
	const FLT x187 = 2 * (*lh_p).Rot[1];
	const FLT x188 = 2 * (*lh_p).Rot[3];
	const FLT x189 = (x44 * x188) + (-1 * x187 * x186);
	const FLT x190 = x153 + (x172 * x186) + x169;
	const FLT x191 = 2 * x52;
	const FLT x192 = (-4 * x53) + (-1 * x191) + x150;
	const FLT x193 = (x192 * x138) + (x101 * x190);
	const FLT x194 = (x70 * x189) + (-1 * x155 * (x193 + (x189 * x149)));
	const FLT x195 = x102 * x194;
	const FLT x196 = x73 * x195;
	const FLT x197 = (x74 * x195) + (x71 * ((-1 * x72 * x195) + x196));
	const FLT x198 = (x75 * x195) + (x71 * x197);
	const FLT x199 = (x86 * x189) + (-1 * x161 * x193);
	const FLT x200 = ((-1 * x164 * x192) + (x163 * x190)) * x165;
	const FLT x201 = x111 * ((-1 * x200) + (x108 * x199));
	const FLT x202 =
		x200 + (-1 * x97 *
				(x199 + (x95 * x198) + (-1 * x201 * x112) + (x195 * x130) +
				 (-1 * x125 *
				  ((x201 * x123) +
				   (-1 * x90 *
					((x76 * x195) + (x71 * x198) +
					 (x71 * (x198 + (x80 * x195) +
							 (x71 * (x197 + (x71 * ((x78 * x195) + (-1 * x119 * x195) + x196)) + (x121 * x194))))) +
					 (x122 * x194)))))));
	const FLT x203 = (-1 * sensor_pt[1]) + (-1 * x34) + (-1 * x37) + (-1 * (*error_model).Pose.Pos[1]) +
					 (-1 * (*_x0).Pose.Pos[1]) + (-1 * x35);
	const FLT x204 = x191 + x151 + (x203 * x188);
	const FLT x205 = x147 + (-1 * x174) + (-4 * x59);
	const FLT x206 = (x49 * x187) + (-1 * x203 * x172);
	const FLT x207 = (x206 * x138) + (x205 * x101);
	const FLT x208 = (x70 * x204) + (-1 * x155 * (x207 + (x204 * x149)));
	const FLT x209 = x208 * x102;
	const FLT x210 = x73 * x209;
	const FLT x211 = (x74 * x209) + (x71 * ((-1 * x72 * x209) + x210));
	const FLT x212 = (x75 * x209) + (x71 * x211);
	const FLT x213 = (x86 * x204) + (-1 * x207 * x161);
	const FLT x214 = ((-1 * x206 * x164) + (x205 * x163)) * x165;
	const FLT x215 = (-1 * x214) + (x213 * x108);
	const FLT x216 =
		x214 + (-1 * x97 *
				(x213 + (x209 * x130) + (x95 * x212) +
				 (-1 * x125 *
				  ((x215 * x124) +
				   (-1 * x90 *
					((x76 * x209) +
					 (x71 * (x212 + (x80 * x209) +
							 (x71 * (x211 + (x71 * ((x78 * x209) + x210 + (-1 * x209 * x119))) + (x208 * x121))))) +
					 (x208 * x122) + (x71 * x212))))) +
				 (-1 * x215 * x113)));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[0]) / sizeof(FLT), x126 + (x127 * x126));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[1]) / sizeof(FLT), (-1 * x127 * x135) + (-1 * x135));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[2]) / sizeof(FLT), x146 + (x127 * x146));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[0]) / sizeof(FLT), x168 + (x127 * x168));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[1]) / sizeof(FLT), x185 + (x127 * x185));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[2]) / sizeof(FLT), x202 + (x202 * x127));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[3]) / sizeof(FLT), x216 + (x216 * x127));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen2 wrt [(*lh_p).Pos[0], (*lh_p).Pos[1],
// (*lh_p).Pos[2], (*lh_p).Rot[0], (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]

static inline void SurviveKalmanErrorModel_LightMeas_x_gen2_jac_lh_p_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_x_gen2(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_x_gen2_jac_lh_p(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen2 wrt [<cnkalman.codegen.WrapMember object at 0x7f88f4f337f0>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4f33ca0>, <cnkalman.codegen.WrapMember object at 0x7f88f4f33c10>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4f33d60>, <cnkalman.codegen.WrapMember object at 0x7f88f4f33d00>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4f33f70>, <cnkalman.codegen.WrapMember object at 0x7f88f4f33eb0>]
static inline void SurviveKalmanErrorModel_LightMeas_x_gen2_jac_bsc0(CnMat *Hx, const FLT dt,
																	 const SurviveKalmanModel *_x0,
																	 const SurviveKalmanErrorModel *error_model,
																	 const FLT *sensor_pt, const SurvivePose *lh_p,
																	 const BaseStationCal *bsc0) {
	const FLT x0 = 0.5 * (*_x0).Pose.Rot[3];
	const FLT x1 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x2 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x3 = (*_x0).Pose.Rot[1] + (x2 * (*_x0).Pose.Rot[0]) + (-1 * x0 * (*error_model).Pose.AxisAngleRot[1]) +
				   (x1 * (*_x0).Pose.Rot[2]);
	const FLT x4 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x5 = dt * dt;
	const FLT x6 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x7 = (x6 * x6) * x5;
	const FLT x8 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x9 = x5 * (x8 * x8);
	const FLT x10 = (x4 * x4) * x5;
	const FLT x11 = 1e-10 + x7 + x10 + x9;
	const FLT x12 = sqrt(x11);
	const FLT x13 = 0.5 * x12;
	const FLT x14 = sin(x13);
	const FLT x15 = (x14 * x14) * (1. / x11);
	const FLT x16 = cos(x13);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x9 * x15) + (x7 * x15) + (x15 * x10));
	const FLT x18 = dt * x14 * (1. / x12) * x17;
	const FLT x19 = x4 * x18;
	const FLT x20 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x21 = (-1 * x2 * (*_x0).Pose.Rot[1]) + (-1 * x20 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x0 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x22 = x6 * x18;
	const FLT x23 =
		(x20 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x2 * (*_x0).Pose.Rot[2]) + (x1 * (*_x0).Pose.Rot[0]);
	const FLT x24 = x17 * x16;
	const FLT x25 = (-1 * x1 * (*_x0).Pose.Rot[1]) + (x20 * (*_x0).Pose.Rot[0]) +
					(x0 * (*error_model).Pose.AxisAngleRot[0]) + (*_x0).Pose.Rot[2];
	const FLT x26 = x8 * x18;
	const FLT x27 = (x25 * x26) + (x24 * x23) + (-1 * x3 * x19) + (x22 * x21);
	const FLT x28 = (-1 * x3 * x26) + (-1 * x22 * x23) + (-1 * x25 * x19) + (x24 * x21);
	const FLT x29 = (x3 * x24) + (-1 * x25 * x22) + (x21 * x26) + (x23 * x19);
	const FLT x30 = (-1 * x29 * sensor_pt[2]) + (x28 * sensor_pt[1]) + (x27 * sensor_pt[0]);
	const FLT x31 = (-1 * x23 * x26) + (x24 * x25) + (x3 * x22) + (x21 * x19);
	const FLT x32 = (-1 * x31 * sensor_pt[0]) + (x29 * sensor_pt[1]) + (x28 * sensor_pt[2]);
	const FLT x33 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x34 = (x33 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + (2 * ((x32 * x31) + (-1 * x30 * x27))) +
					(*error_model).Pose.Pos[0] + sensor_pt[0] +
					(dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) + (*_x0).Pose.Pos[0];
	const FLT x35 = (x31 * sensor_pt[2]) + (-1 * x27 * sensor_pt[1]) + (x28 * sensor_pt[0]);
	const FLT x36 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (2 * ((x30 * x29) + (-1 * x31 * x35))) +
					(*error_model).Pose.Pos[2] + (dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2])) +
					(x33 * ((*_x0).Acc[2] + (*error_model).Acc[2]));
	const FLT x37 = (*_x0).Pose.Pos[1] + (2 * ((x35 * x27) + (-1 * x32 * x29))) + (*error_model).Pose.Pos[1] +
					(dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) + sensor_pt[1] +
					(x33 * ((*_x0).Acc[1] + (*error_model).Acc[1]));
	const FLT x38 = (-1 * x37 * (*lh_p).Rot[3]) + (x34 * (*lh_p).Rot[0]) + (x36 * (*lh_p).Rot[2]);
	const FLT x39 = (-1 * x36 * (*lh_p).Rot[1]) + (x37 * (*lh_p).Rot[0]) + (x34 * (*lh_p).Rot[3]);
	const FLT x40 = x36 + (2 * ((x39 * (*lh_p).Rot[1]) + (-1 * x38 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x41 = (-1 * x34 * (*lh_p).Rot[2]) + (x37 * (*lh_p).Rot[1]) + (x36 * (*lh_p).Rot[0]);
	const FLT x42 = x34 + (*lh_p).Pos[0] + (2 * ((x41 * (*lh_p).Rot[2]) + (-1 * x39 * (*lh_p).Rot[3])));
	const FLT x43 = atan2(-1 * x40, x42);
	const FLT x44 = 0.523598775598299 + (*bsc0).tilt;
	const FLT x45 = tan(x44);
	const FLT x46 = x37 + (*lh_p).Pos[1] + (2 * ((x38 * (*lh_p).Rot[3]) + (-1 * x41 * (*lh_p).Rot[1])));
	const FLT x47 = (x42 * x42) + (x40 * x40);
	const FLT x48 = x46 * (1. / sqrt(x47));
	const FLT x49 = x45 * x48;
	const FLT x50 = asin(x49) + (-1 * (*bsc0).ogeephase) + (-1 * x43);
	const FLT x51 = sin(x50);
	const FLT x52 = (-1 * x51 * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x53 = cos(x44);
	const FLT x54 = x46 * x46;
	const FLT x55 = x47 + x54;
	const FLT x56 = (1. / sqrt(x55)) * x46;
	const FLT x57 = asin((1. / x53) * x56);
	const FLT x58 = 8.0108022e-06 * x57;
	const FLT x59 = -8.0108022e-06 + (-1 * x58);
	const FLT x60 = 0.0028679863 + (x57 * x59);
	const FLT x61 = 5.3685255e-06 + (x60 * x57);
	const FLT x62 = 0.0076069798 + (x61 * x57);
	const FLT x63 = x57 * x57;
	const FLT x64 = sin(x44);
	const FLT x65 = x62 * x57;
	const FLT x66 = -8.0108022e-06 + (-1.60216044e-05 * x57);
	const FLT x67 = x60 + (x66 * x57);
	const FLT x68 = x61 + (x67 * x57);
	const FLT x69 = x62 + (x68 * x57);
	const FLT x70 = (x69 * x57) + x65;
	const FLT x71 = x70 * x52;
	const FLT x72 = x71 * x64;
	const FLT x73 = x53 + (-1 * x72);
	const FLT x74 = 1. / x73;
	const FLT x75 = x74 * x63;
	const FLT x76 = x75 * x62;
	const FLT x77 = x49 + (x76 * x52);
	const FLT x78 = 1. / sqrt(1 + (-1 * (x77 * x77)));
	const FLT x79 = (1. / (x73 * x73)) * x63 * x62;
	const FLT x80 = x72 * x79;
	const FLT x81 = (x80 + x76) * x78;
	const FLT x82 = (-1 * asin(x77)) + (*bsc0).gibpha + x43;
	const FLT x83 = cos(x82) * (*bsc0).gibmag;
	const FLT x84 = ((-1 * x80 * x51) + (-1 * x76 * x51)) * x78;
	const FLT x85 = cos(x50) * (*bsc0).ogeemag;
	const FLT x86 = x85 * x76;
	const FLT x87 = x78 * ((x80 * x85) + x86);
	const FLT x88 = x64 * x52;
	const FLT x89 = 1. / (x53 * x53);
	const FLT x90 = x89 * x56 * (1. / sqrt(1 + (-1 * x89 * x54 * (1. / x55))));
	const FLT x91 = x45 * x45;
	const FLT x92 = x48 * (1 + x91);
	const FLT x93 = x92 * (1. / sqrt(1 + (-1 * x54 * x91 * (1. / x47))));
	const FLT x94 = x64 * x90;
	const FLT x95 = x59 * x94;
	const FLT x96 = (x60 * x94) + (x57 * ((-1 * x58 * x94) + x95));
	const FLT x97 = (x61 * x94) + (x57 * x96);
	const FLT x98 =
		x78 * ((-1 * x79 * x52 *
				((-1 * x88 *
				  ((x57 * (x97 + (x68 * x94) +
						   (x57 * (x96 + (x57 * ((x66 * x94) + x95 + (-2.40324066e-05 * x57 * x94))) + (x67 * x94))))) +
				   (x62 * x94) + (x57 * x97) + (x69 * x94))) +
				 (-1 * x64) + (x85 * x70 * x64 * x93) + (-1 * x71 * x53))) +
			   x92 + (2 * x88 * x74 * x65 * x90) + (x75 * x52 * x97) + (-1 * x86 * x93));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, curve) / sizeof(FLT), (-1 * x81 * x83) + (-1 * x81));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibmag) / sizeof(FLT), sin(x82));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibpha) / sizeof(FLT), x83);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, ogeemag) / sizeof(FLT), (-1 * x83 * x84) + (-1 * x84));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, ogeephase) / sizeof(FLT), (-1 * x83 * x87) + (-1 * x87));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, phase) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, tilt) / sizeof(FLT), (-1 * x83 * x98) + (-1 * x98));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_x_gen2 wrt [<cnkalman.codegen.WrapMember object at
// 0x7f88f4f337f0>, <cnkalman.codegen.WrapMember object at 0x7f88f4f33ca0>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4f33c10>, <cnkalman.codegen.WrapMember object at 0x7f88f4f33d60>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4f33d00>, <cnkalman.codegen.WrapMember object at 0x7f88f4f33f70>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4f33eb0>]

static inline void SurviveKalmanErrorModel_LightMeas_x_gen2_jac_bsc0_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_x_gen2(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_x_gen2_jac_bsc0(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
static inline FLT SurviveKalmanErrorModel_LightMeas_y_gen2(const FLT dt, const SurviveKalmanModel *_x0,
														   const SurviveKalmanErrorModel *error_model,
														   const FLT *sensor_pt, const SurvivePose *lh_p,
														   const BaseStationCal *bsc0) {
	const FLT x0 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x1 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x2 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x3 = 0.5 * (*_x0).Pose.Rot[0];
	const FLT x4 = (*_x0).Pose.Rot[1] + (x3 * (*error_model).Pose.AxisAngleRot[0]) + (-1 * x1 * (*_x0).Pose.Rot[3]) +
				   (x2 * (*_x0).Pose.Rot[2]);
	const FLT x5 = dt * dt;
	const FLT x6 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x7 = (x6 * x6) * x5;
	const FLT x8 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x9 = x5 * (x8 * x8);
	const FLT x10 = (x0 * x0) * x5;
	const FLT x11 = 1e-10 + x7 + x10 + x9;
	const FLT x12 = sqrt(x11);
	const FLT x13 = 0.5 * x12;
	const FLT x14 = sin(x13);
	const FLT x15 = (x14 * x14) * (1. / x11);
	const FLT x16 = cos(x13);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x9 * x15) + (x7 * x15) + (x15 * x10));
	const FLT x18 = dt * x14 * (1. / x12) * x17;
	const FLT x19 = x4 * x18;
	const FLT x20 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x21 = (-1 * x20 * (*_x0).Pose.Rot[1]) + (-1 * x1 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x2 * (*_x0).Pose.Rot[3]);
	const FLT x22 = x21 * x18;
	const FLT x23 = (x1 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x20 * (*_x0).Pose.Rot[2]) +
					(x3 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x24 = x17 * x16;
	const FLT x25 = (-1 * x2 * (*_x0).Pose.Rot[1]) + (x3 * (*error_model).Pose.AxisAngleRot[1]) +
					(x20 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x26 = x25 * x18;
	const FLT x27 = (x8 * x26) + (-1 * x0 * x19) + (x24 * x23) + (x6 * x22);
	const FLT x28 = x23 * x18;
	const FLT x29 = x8 * x18;
	const FLT x30 = (-1 * x4 * x29) + (-1 * x6 * x28) + (-1 * x0 * x26) + (x24 * x21);
	const FLT x31 = (x21 * x29) + (x4 * x24) + (-1 * x6 * x26) + (x0 * x28);
	const FLT x32 = (-1 * x31 * sensor_pt[2]) + (x30 * sensor_pt[1]) + (x27 * sensor_pt[0]);
	const FLT x33 = (x24 * x25) + (x6 * x19) + (-1 * x23 * x29) + (x0 * x22);
	const FLT x34 = (-1 * x33 * sensor_pt[0]) + (x31 * sensor_pt[1]) + (x30 * sensor_pt[2]);
	const FLT x35 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x36 = (x35 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + (*error_model).Pose.Pos[0] + sensor_pt[0] +
					(dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) +
					(2 * ((x34 * x33) + (-1 * x32 * x27))) + (*_x0).Pose.Pos[0];
	const FLT x37 = (x33 * sensor_pt[2]) + (-1 * x27 * sensor_pt[1]) + (x30 * sensor_pt[0]);
	const FLT x38 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (x35 * ((*_x0).Acc[2] + (*error_model).Acc[2])) +
					(*error_model).Pose.Pos[2] + (2 * ((x32 * x31) + (-1 * x33 * x37))) +
					(dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2]));
	const FLT x39 = (*_x0).Pose.Pos[1] + (2 * ((x37 * x27) + (-1 * x31 * x34))) + (*error_model).Pose.Pos[1] +
					sensor_pt[1] + (dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) +
					(x35 * ((*_x0).Acc[1] + (*error_model).Acc[1]));
	const FLT x40 = (x36 * (*lh_p).Rot[0]) + (-1 * x39 * (*lh_p).Rot[3]) + (x38 * (*lh_p).Rot[2]);
	const FLT x41 = (x39 * (*lh_p).Rot[0]) + (-1 * x38 * (*lh_p).Rot[1]) + (x36 * (*lh_p).Rot[3]);
	const FLT x42 = x38 + (2 * ((x41 * (*lh_p).Rot[1]) + (-1 * x40 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x43 = (-1 * x36 * (*lh_p).Rot[2]) + (x39 * (*lh_p).Rot[1]) + (x38 * (*lh_p).Rot[0]);
	const FLT x44 = x36 + (*lh_p).Pos[0] + (2 * ((x43 * (*lh_p).Rot[2]) + (-1 * x41 * (*lh_p).Rot[3])));
	const FLT x45 = atan2(-1 * x42, x44);
	const FLT x46 = 0.523598775598299 + (-1 * (*bsc0).tilt);
	const FLT x47 = x39 + (*lh_p).Pos[1] + (2 * ((x40 * (*lh_p).Rot[3]) + (-1 * x43 * (*lh_p).Rot[1])));
	const FLT x48 = (x44 * x44) + (x42 * x42);
	const FLT x49 = -1 * x47 * (1. / sqrt(x48)) * tan(x46);
	const FLT x50 = ((*bsc0).ogeemag * sin((-1 * asin(x49)) + x45 + (*bsc0).ogeephase)) + (*bsc0).curve;
	const FLT x51 = cos(x46);
	const FLT x52 = asin((1. / x51) * x47 * (1. / sqrt(x48 + (x47 * x47))));
	const FLT x53 = 0.0028679863 + (x52 * (-8.0108022e-06 + (-8.0108022e-06 * x52)));
	const FLT x54 = 5.3685255e-06 + (x53 * x52);
	const FLT x55 = 0.0076069798 + (x54 * x52);
	const FLT x56 =
		(-1 *
		 asin(x49 + (x50 * (x52 * x52) * x55 *
					 (1. / (x51 + (x50 * sin(x46) *
								   ((x52 * (x55 + (x52 * (x54 + (x52 * (x53 + (x52 * (-8.0108022e-06 +
																					  (-1.60216044e-05 * x52))))))))) +
									(x52 * x55)))))))) +
		x45;
	return -1.5707963267949 + x56 + (-1 * (*bsc0).phase) + (sin(x56 + (*bsc0).gibpha) * (*bsc0).gibmag);
}

// Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen2 wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2],
// (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3],
// (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1],
// (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1],
// (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7f88f4f2b1c0>]
static inline void SurviveKalmanErrorModel_LightMeas_y_gen2_jac_x0(CnMat *Hx, const FLT dt,
																   const SurviveKalmanModel *_x0,
																   const SurviveKalmanErrorModel *error_model,
																   const FLT *sensor_pt, const SurvivePose *lh_p,
																   const BaseStationCal *bsc0) {
	const FLT x0 = dt * fabs(dt);
	const FLT x1 = x0 * (*lh_p).Rot[2];
	const FLT x2 = x1 * (*lh_p).Rot[1];
	const FLT x3 = x0 * (*lh_p).Rot[3] * (*lh_p).Rot[0];
	const FLT x4 = x3 + x2;
	const FLT x5 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x6 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x7 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x8 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x9 =
		(-1 * x8 * (*_x0).Pose.Rot[1]) + (x7 * (*_x0).Pose.Rot[0]) + (x6 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x10 = dt * dt;
	const FLT x11 = x5 * x5;
	const FLT x12 = x11 * x10;
	const FLT x13 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x14 = x13 * x13;
	const FLT x15 = x14 * x10;
	const FLT x16 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x17 = x16 * x16;
	const FLT x18 = x10 * x17;
	const FLT x19 = 1e-10 + x18 + x12 + x15;
	const FLT x20 = sqrt(x19);
	const FLT x21 = 0.5 * x20;
	const FLT x22 = sin(x21);
	const FLT x23 = x22 * x22;
	const FLT x24 = 1. / x19;
	const FLT x25 = x24 * x23;
	const FLT x26 = cos(x21);
	const FLT x27 = (x25 * x15) + (x26 * x26) + (x25 * x12) + (x25 * x18);
	const FLT x28 = 1. / sqrt(x27);
	const FLT x29 = x22 * x28;
	const FLT x30 = x9 * x29;
	const FLT x31 = 1. / x20;
	const FLT x32 = dt * x31;
	const FLT x33 = x30 * x32;
	const FLT x34 =
		(*_x0).Pose.Rot[3] + (x7 * (*_x0).Pose.Rot[1]) + (-1 * x6 * (*_x0).Pose.Rot[2]) + (x8 * (*_x0).Pose.Rot[0]);
	const FLT x35 = x32 * x29;
	const FLT x36 = x34 * x35;
	const FLT x37 =
		(x6 * (*_x0).Pose.Rot[0]) + (-1 * x7 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[1] + (x8 * (*_x0).Pose.Rot[2]);
	const FLT x38 = x28 * x26;
	const FLT x39 = x38 * x37;
	const FLT x40 = (-1 * x6 * (*_x0).Pose.Rot[1]) + (-1 * x7 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x8 * (*_x0).Pose.Rot[3]);
	const FLT x41 = x40 * x35;
	const FLT x42 = (x41 * x13) + x39 + (-1 * x5 * x33) + (x36 * x16);
	const FLT x43 = x40 * x38;
	const FLT x44 = x5 * x35;
	const FLT x45 = x35 * x37;
	const FLT x46 = (-1 * x45 * x13) + (-1 * x44 * x34) + (-1 * x33 * x16) + x43;
	const FLT x47 = x9 * x38;
	const FLT x48 = (-1 * x36 * x13) + x47 + (x44 * x37) + (x41 * x16);
	const FLT x49 = (x42 * sensor_pt[1]) + (-1 * x48 * sensor_pt[0]) + (x46 * sensor_pt[2]);
	const FLT x50 = x34 * x38;
	const FLT x51 = (x33 * x13) + x50 + (-1 * x45 * x16) + (x40 * x44);
	const FLT x52 = (-1 * x51 * sensor_pt[1]) + (x48 * sensor_pt[2]) + (x46 * sensor_pt[0]);
	const FLT x53 = 1.0 / 2.0 * x0;
	const FLT x54 = (*_x0).Pose.Pos[1] + (*error_model).Pose.Pos[1] + (2 * ((x52 * x51) + (-1 * x42 * x49))) +
					sensor_pt[1] + (dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) +
					(x53 * ((*_x0).Acc[1] + (*error_model).Acc[1]));
	const FLT x55 = (-1 * x42 * sensor_pt[2]) + (x46 * sensor_pt[1]) + (x51 * sensor_pt[0]);
	const FLT x56 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (2 * ((x55 * x42) + (-1 * x52 * x48))) +
					(x53 * ((*_x0).Acc[2] + (*error_model).Acc[2])) + (*error_model).Pose.Pos[2] +
					(dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2]));
	const FLT x57 = (x53 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + (2 * ((x48 * x49) + (-1 * x51 * x55))) +
					(*error_model).Pose.Pos[0] + sensor_pt[0] +
					(dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) + (*_x0).Pose.Pos[0];
	const FLT x58 = (-1 * x57 * (*lh_p).Rot[2]) + (x54 * (*lh_p).Rot[1]) + (x56 * (*lh_p).Rot[0]);
	const FLT x59 = (-1 * x54 * (*lh_p).Rot[3]) + (x57 * (*lh_p).Rot[0]) + (x56 * (*lh_p).Rot[2]);
	const FLT x60 = x54 + (*lh_p).Pos[1] + (2 * ((x59 * (*lh_p).Rot[3]) + (-1 * x58 * (*lh_p).Rot[1])));
	const FLT x61 = 2 * x60;
	const FLT x62 = (*lh_p).Rot[2] * (*lh_p).Rot[2];
	const FLT x63 = -1 * x0 * x62;
	const FLT x64 = (*lh_p).Rot[3] * (*lh_p).Rot[3];
	const FLT x65 = x53 + (-1 * x0 * x64);
	const FLT x66 = x65 + x63;
	const FLT x67 = (x54 * (*lh_p).Rot[0]) + (-1 * x56 * (*lh_p).Rot[1]) + (x57 * (*lh_p).Rot[3]);
	const FLT x68 = x57 + (*lh_p).Pos[0] + (2 * ((x58 * (*lh_p).Rot[2]) + (-1 * x67 * (*lh_p).Rot[3])));
	const FLT x69 = 2 * x68;
	const FLT x70 = x1 * (*lh_p).Rot[0];
	const FLT x71 = x0 * (*lh_p).Rot[1];
	const FLT x72 = x71 * (*lh_p).Rot[3];
	const FLT x73 = x72 + (-1 * x70);
	const FLT x74 = x56 + (2 * ((x67 * (*lh_p).Rot[1]) + (-1 * x59 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x75 = 2 * x74;
	const FLT x76 = (x73 * x75) + (x66 * x69);
	const FLT x77 = 0.523598775598299 + (-1 * (*bsc0).tilt);
	const FLT x78 = cos(x77);
	const FLT x79 = 1. / x78;
	const FLT x80 = x60 * x60;
	const FLT x81 = x68 * x68;
	const FLT x82 = x81 + (x74 * x74);
	const FLT x83 = x82 + x80;
	const FLT x84 = 1.0 / 2.0 * x60;
	const FLT x85 = (1. / (x83 * sqrt(x83))) * x84 * x79;
	const FLT x86 = (1. / sqrt(x83)) * x79;
	const FLT x87 = (x4 * x86) + (-1 * x85 * (x76 + (x4 * x61)));
	const FLT x88 = 1. / sqrt(1 + (-1 * x80 * (1. / x83) * (1. / (x78 * x78))));
	const FLT x89 = atan2(-1 * x74, x68);
	const FLT x90 = tan(x77);
	const FLT x91 = (1. / sqrt(x82)) * x90;
	const FLT x92 = -1 * x60 * x91;
	const FLT x93 = asin(x92) + (-1 * x89) + (-1 * (*bsc0).ogeephase);
	const FLT x94 = (-1 * sin(x93) * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x95 = asin(x86 * x60);
	const FLT x96 = 8.0108022e-06 * x95;
	const FLT x97 = -8.0108022e-06 + (-1 * x96);
	const FLT x98 = 0.0028679863 + (x97 * x95);
	const FLT x99 = 5.3685255e-06 + (x98 * x95);
	const FLT x100 = 0.0076069798 + (x99 * x95);
	const FLT x101 = x95 * x100;
	const FLT x102 = -8.0108022e-06 + (-1.60216044e-05 * x95);
	const FLT x103 = x98 + (x95 * x102);
	const FLT x104 = x99 + (x95 * x103);
	const FLT x105 = x100 + (x95 * x104);
	const FLT x106 = (x95 * x105) + x101;
	const FLT x107 = sin(x77);
	const FLT x108 = x94 * x107;
	const FLT x109 = x78 + (x108 * x106);
	const FLT x110 = 1. / x109;
	const FLT x111 = 2 * x94 * x101 * x110;
	const FLT x112 = x88 * x111;
	const FLT x113 = (1. / (x82 * sqrt(x82))) * x84 * x90;
	const FLT x114 = (-1 * x4 * x91) + (x76 * x113);
	const FLT x115 = 1. / x82;
	const FLT x116 = 1. / sqrt(1 + (-1 * x80 * (x90 * x90) * x115));
	const FLT x117 = (1. / x81) * x74;
	const FLT x118 = 1. / x68;
	const FLT x119 = x81 * x115;
	const FLT x120 = ((-1 * x73 * x118) + (x66 * x117)) * x119;
	const FLT x121 = (-1 * x120) + (x114 * x116);
	const FLT x122 = cos(x93) * (*bsc0).ogeemag;
	const FLT x123 = x95 * x95;
	const FLT x124 = x100 * x110 * x123;
	const FLT x125 = x124 * x122;
	const FLT x126 = x107 * x106;
	const FLT x127 = x122 * x126;
	const FLT x128 = x88 * x105;
	const FLT x129 = x88 * x104;
	const FLT x130 = 2.40324066e-05 * x95;
	const FLT x131 = x88 * x130;
	const FLT x132 = x88 * x97;
	const FLT x133 = x87 * x132;
	const FLT x134 = x88 * x102;
	const FLT x135 = x88 * x103;
	const FLT x136 = x88 * x96;
	const FLT x137 = x88 * x98;
	const FLT x138 = (x87 * x137) + (x95 * ((-1 * x87 * x136) + x133));
	const FLT x139 = x88 * x99;
	const FLT x140 = (x87 * x139) + (x95 * x138);
	const FLT x141 = x88 * x100;
	const FLT x142 = x94 * x123;
	const FLT x143 = x100 * (1. / (x109 * x109)) * x142;
	const FLT x144 = x110 * x142;
	const FLT x145 = x92 + (x100 * x144);
	const FLT x146 = 1. / sqrt(1 + (-1 * (x145 * x145)));
	const FLT x147 =
		x120 +
		(-1 * x146 *
		 (x114 +
		  (-1 * x143 *
		   ((x108 * ((x95 * x140) + (x87 * x128) + (x87 * x141) +
					 (x95 * (x140 + (x87 * x129) +
							 (x95 * ((x95 * ((x87 * x134) + (-1 * x87 * x131) + x133)) + x138 + (x87 * x135))))))) +
			(-1 * x121 * x127))) +
		  (x87 * x112) + (x140 * x144) + (-1 * x121 * x125)));
	const FLT x148 = cos((-1 * asin(x145)) + (*bsc0).gibpha + x89) * (*bsc0).gibmag;
	const FLT x149 = (*lh_p).Rot[1] * (*lh_p).Rot[1];
	const FLT x150 = -1 * x0 * x149;
	const FLT x151 = x65 + x150;
	const FLT x152 = x2 + (-1 * x3);
	const FLT x153 = x1 * (*lh_p).Rot[3];
	const FLT x154 = x71 * (*lh_p).Rot[0];
	const FLT x155 = x154 + x153;
	const FLT x156 = (x75 * x155) + (x69 * x152);
	const FLT x157 = (x86 * x151) + (-1 * x85 * (x156 + (x61 * x151)));
	const FLT x158 = (-1 * x91 * x151) + (x113 * x156);
	const FLT x159 = ((-1 * x118 * x155) + (x117 * x152)) * x119;
	const FLT x160 = (-1 * x159) + (x116 * x158);
	const FLT x161 = x132 * x157;
	const FLT x162 = (x137 * x157) + (x95 * ((-1 * x136 * x157) + x161));
	const FLT x163 = (x139 * x157) + (x95 * x162);
	const FLT x164 =
		x159 +
		(-1 * x146 *
		 ((x163 * x144) + x158 + (-1 * x125 * x160) + (x112 * x157) +
		  (-1 * x143 *
		   ((x108 * ((x95 * x163) +
					 (x95 * (x163 + (x129 * x157) +
							 (x95 * ((x95 * ((x134 * x157) + (-1 * x131 * x157) + x161)) + x162 + (x135 * x157))))) +
					 (x141 * x157) + (x128 * x157))) +
			(-1 * x127 * x160)))));
	const FLT x165 = x153 + (-1 * x154);
	const FLT x166 = x70 + x72;
	const FLT x167 = x63 + x53 + x150;
	const FLT x168 = (x75 * x167) + (x69 * x166);
	const FLT x169 = (x86 * x165) + (-1 * x85 * (x168 + (x61 * x165)));
	const FLT x170 = (-1 * x91 * x165) + (x113 * x168);
	const FLT x171 = ((-1 * x118 * x167) + (x117 * x166)) * x119;
	const FLT x172 = (-1 * x171) + (x116 * x170);
	const FLT x173 = x169 * x132;
	const FLT x174 = (x169 * x137) + (x95 * ((-1 * x169 * x136) + x173));
	const FLT x175 = (x169 * x139) + (x95 * x174);
	const FLT x176 =
		x171 +
		(-1 * x146 *
		 ((-1 * x125 * x172) + x170 + (x112 * x169) + (x175 * x144) +
		  (-1 * x143 *
		   ((x108 * ((x95 * (x175 + (x129 * x169) +
							 (x95 * (x174 + (x95 * ((x169 * x134) + x173 + (-1 * x169 * x131))) + (x169 * x135))))) +
					 (x95 * x175) + (x128 * x169) + (x169 * x141))) +
			(-1 * x127 * x172)))));
	const FLT x177 = 2 * x64;
	const FLT x178 = -1 * x177;
	const FLT x179 = 2 * x62;
	const FLT x180 = -1 * x179;
	const FLT x181 = 1 + x180 + x178;
	const FLT x182 = 2 * (*lh_p).Rot[0];
	const FLT x183 = x182 * (*lh_p).Rot[2];
	const FLT x184 = 2 * (*lh_p).Rot[3];
	const FLT x185 = x184 * (*lh_p).Rot[1];
	const FLT x186 = x185 + (-1 * x183);
	const FLT x187 = (x75 * x186) + (x69 * x181);
	const FLT x188 = 2 * (*lh_p).Rot[2];
	const FLT x189 = x188 * (*lh_p).Rot[1];
	const FLT x190 = x182 * (*lh_p).Rot[3];
	const FLT x191 = x190 + x189;
	const FLT x192 = (-1 * x91 * x191) + (x113 * x187);
	const FLT x193 = ((-1 * x118 * x186) + (x117 * x181)) * x119;
	const FLT x194 = (-1 * x193) + (x116 * x192);
	const FLT x195 = (x86 * x191) + (-1 * x85 * (x187 + (x61 * x191)));
	const FLT x196 = x195 * x132;
	const FLT x197 = (x195 * x137) + (x95 * ((-1 * x195 * x136) + x196));
	const FLT x198 = (x195 * x139) + (x95 * x197);
	const FLT x199 =
		x193 +
		(-1 * x146 *
		 (x192 +
		  (-1 * x143 *
		   ((x108 * ((x95 * x198) + (x195 * x141) + (x128 * x195) +
					 (x95 * (x198 + (x129 * x195) +
							 (x95 * (x197 + (x95 * ((x195 * x134) + x196 + (-1 * x195 * x131))) + (x195 * x135))))))) +
			(-1 * x127 * x194))) +
		  (x198 * x144) + (-1 * x125 * x194) + (x112 * x195)));
	const FLT x200 = x189 + (-1 * x190);
	const FLT x201 = x184 * (*lh_p).Rot[2];
	const FLT x202 = x182 * (*lh_p).Rot[1];
	const FLT x203 = x202 + x201;
	const FLT x204 = (x75 * x203) + (x69 * x200);
	const FLT x205 = 2 * x149;
	const FLT x206 = 1 + (-1 * x205);
	const FLT x207 = x206 + x178;
	const FLT x208 = (-1 * x91 * x207) + (x204 * x113);
	const FLT x209 = ((-1 * x203 * x118) + (x200 * x117)) * x119;
	const FLT x210 = x122 * ((-1 * x209) + (x208 * x116));
	const FLT x211 = (x86 * x207) + (-1 * x85 * (x204 + (x61 * x207)));
	const FLT x212 = x211 * x132;
	const FLT x213 = (x211 * x137) + (x95 * ((-1 * x211 * x136) + x212));
	const FLT x214 = (x211 * x139) + (x95 * x213);
	const FLT x215 =
		x209 +
		(-1 * x146 *
		 ((x211 * x112) + x208 + (x214 * x144) + (-1 * x210 * x124) +
		  (-1 * x143 *
		   ((x108 * ((x211 * x141) + (x95 * x214) + (x211 * x128) +
					 (x95 * (x214 + (x211 * x129) +
							 (x95 * (x213 + (x95 * ((x211 * x134) + x212 + (-1 * x211 * x131))) + (x211 * x135))))))) +
			(-1 * x210 * x126)))));
	const FLT x216 = x183 + x185;
	const FLT x217 = x206 + x180;
	const FLT x218 = (x75 * x217) + (x69 * x216);
	const FLT x219 = x201 + (-1 * x202);
	const FLT x220 = (-1 * x91 * x219) + (x218 * x113);
	const FLT x221 = ((-1 * x217 * x118) + (x216 * x117)) * x119;
	const FLT x222 = (-1 * x221) + (x220 * x116);
	const FLT x223 = (x86 * x219) + (-1 * x85 * (x218 + (x61 * x219)));
	const FLT x224 = x223 * x132;
	const FLT x225 = (x223 * x137) + (x95 * ((-1 * x223 * x136) + x224));
	const FLT x226 = (x223 * x139) + (x95 * x225);
	const FLT x227 =
		x221 +
		(-1 * x146 *
		 (x220 + (x226 * x144) + (x223 * x112) +
		  (-1 * x143 *
		   ((x108 * ((x95 * x226) + (x223 * x141) + (x223 * x128) +
					 (x95 * (x226 + (x223 * x129) +
							 (x95 * (x225 + (x95 * ((x223 * x134) + x224 + (-1 * x223 * x131))) + (x223 * x135))))))) +
			(-1 * x222 * x127))) +
		  (-1 * x222 * x125)));
	const FLT x228 = x6 * x44;
	const FLT x229 = x7 * x38;
	const FLT x230 = x35 * x13;
	const FLT x231 = -1 * x8 * x230;
	const FLT x232 = x35 * x16;
	const FLT x233 = x232 + x231;
	const FLT x234 = x233 + x228 + x229;
	const FLT x235 = 2 * x52;
	const FLT x236 = x6 * x230;
	const FLT x237 = -1 * x236;
	const FLT x238 = x7 * x232;
	const FLT x239 = -1 * x238;
	const FLT x240 = x8 * x44;
	const FLT x241 = (-1 * x240) + x38;
	const FLT x242 = x241 + x237 + x239;
	const FLT x243 = x6 * x38;
	const FLT x244 = x8 * x232;
	const FLT x245 = -1 * x7 * x44;
	const FLT x246 = x230 + x245;
	const FLT x247 = x246 + x243 + x244;
	const FLT x248 = x8 * x38;
	const FLT x249 = x7 * x230;
	const FLT x250 = -1 * x6 * x232;
	const FLT x251 = x44 + x250;
	const FLT x252 = x248 + x251 + x249;
	const FLT x253 = (x252 * sensor_pt[0]) + (x242 * sensor_pt[1]) + (-1 * x247 * sensor_pt[2]);
	const FLT x254 = 2 * x42;
	const FLT x255 = (x242 * sensor_pt[0]) + (-1 * x252 * sensor_pt[1]) + (x234 * sensor_pt[2]);
	const FLT x256 = 2 * x48;
	const FLT x257 = 2 * x55;
	const FLT x258 = (x257 * x247) + (-1 * x256 * x255) + (-1 * x234 * x235) + (x254 * x253);
	const FLT x259 = (x242 * sensor_pt[2]) + (-1 * x234 * sensor_pt[0]) + (x247 * sensor_pt[1]);
	const FLT x260 = 2 * x49;
	const FLT x261 = 2 * x51;
	const FLT x262 = (x235 * x252) + (-1 * x254 * x259) + (x261 * x255) + (-1 * x260 * x247);
	const FLT x263 = (x256 * x259) + (x234 * x260) + (-1 * x252 * x257) + (-1 * x261 * x253);
	const FLT x264 = 2 * ((x263 * (*lh_p).Rot[3]) + (-1 * x258 * (*lh_p).Rot[1]) + (x262 * (*lh_p).Rot[0]));
	const FLT x265 = 2 * ((x258 * (*lh_p).Rot[0]) + (-1 * x263 * (*lh_p).Rot[2]) + (x262 * (*lh_p).Rot[1]));
	const FLT x266 = (-1 * x264 * (*lh_p).Rot[3]) + x263 + (x265 * (*lh_p).Rot[2]);
	const FLT x267 = 2 * ((x258 * (*lh_p).Rot[2]) + (-1 * x262 * (*lh_p).Rot[3]) + (x263 * (*lh_p).Rot[0]));
	const FLT x268 = x258 + (x264 * (*lh_p).Rot[1]) + (-1 * x267 * (*lh_p).Rot[2]);
	const FLT x269 = (x75 * x268) + (x69 * x266);
	const FLT x270 = x262 + (-1 * x265 * (*lh_p).Rot[1]) + (x267 * (*lh_p).Rot[3]);
	const FLT x271 = (-1 * x91 * x270) + (x269 * x113);
	const FLT x272 = ((-1 * x268 * x118) + (x266 * x117)) * x119;
	const FLT x273 = (-1 * x272) + (x271 * x116);
	const FLT x274 = (x86 * x270) + (-1 * x85 * (x269 + (x61 * x270)));
	const FLT x275 = x274 * x132;
	const FLT x276 = (x274 * x137) + (x95 * ((-1 * x274 * x136) + x275));
	const FLT x277 = (x274 * x139) + (x95 * x276);
	const FLT x278 =
		x272 +
		(-1 * x146 *
		 (x271 + (x277 * x144) +
		  (-1 * x143 *
		   ((x108 * ((x95 * x277) + (x274 * x141) + (x274 * x128) +
					 (x95 * (x277 + (x274 * x129) +
							 (x95 * (x276 + (x95 * ((x274 * x134) + (-1 * x274 * x131) + x275)) + (x274 * x135))))))) +
			(-1 * x273 * x127))) +
		  (-1 * x273 * x125) + (x274 * x112)));
	const FLT x279 = -1 * x228;
	const FLT x280 = (-1 * x232) + x231;
	const FLT x281 = x280 + x229 + x279;
	const FLT x282 = -1 * x249;
	const FLT x283 = -1 * x248;
	const FLT x284 = x251 + x282 + x283;
	const FLT x285 = x240 + x38;
	const FLT x286 = x285 + x238 + x237;
	const FLT x287 = -1 * x243;
	const FLT x288 = x245 + (-1 * x230);
	const FLT x289 = x288 + x244 + x287;
	const FLT x290 = (x289 * sensor_pt[2]) + (-1 * x284 * sensor_pt[0]) + (x286 * sensor_pt[1]);
	const FLT x291 = (-1 * x286 * sensor_pt[2]) + (x281 * sensor_pt[0]) + (x289 * sensor_pt[1]);
	const FLT x292 = (x290 * x256) + (-1 * x281 * x257) + (-1 * x291 * x261) + (x260 * x284);
	const FLT x293 = (x289 * sensor_pt[0]) + (-1 * x281 * sensor_pt[1]) + (x284 * sensor_pt[2]);
	const FLT x294 = (x293 * x261) + (-1 * x290 * x254) + (-1 * x260 * x286) + (x235 * x281);
	const FLT x295 = (x291 * x254) + (x286 * x257) + (-1 * x235 * x284) + (-1 * x293 * x256);
	const FLT x296 = (-1 * x292 * (*lh_p).Rot[2]) + (x295 * (*lh_p).Rot[0]) + (x294 * (*lh_p).Rot[1]);
	const FLT x297 = 2 * (*lh_p).Rot[1];
	const FLT x298 = (x295 * (*lh_p).Rot[2]) + (-1 * x294 * (*lh_p).Rot[3]) + (x292 * (*lh_p).Rot[0]);
	const FLT x299 = x294 + (-1 * x297 * x296) + (x298 * x184);
	const FLT x300 = (x292 * (*lh_p).Rot[3]) + (-1 * x295 * (*lh_p).Rot[1]) + (x294 * (*lh_p).Rot[0]);
	const FLT x301 = (-1 * x300 * x184) + x292 + (x296 * x188);
	const FLT x302 = x295 + (-1 * x298 * x188) + (x297 * x300);
	const FLT x303 = (x75 * x302) + (x69 * x301);
	const FLT x304 = (x86 * x299) + (-1 * x85 * (x303 + (x61 * x299)));
	const FLT x305 = x88 * x304;
	const FLT x306 = (-1 * x91 * x299) + (x303 * x113);
	const FLT x307 = ((-1 * x302 * x118) + (x301 * x117)) * x119;
	const FLT x308 = (-1 * x307) + (x306 * x116);
	const FLT x309 = x97 * x305;
	const FLT x310 = (x98 * x305) + (x95 * ((-1 * x96 * x305) + x309));
	const FLT x311 = (x99 * x305) + (x95 * x310);
	const FLT x312 =
		x307 +
		(-1 * x146 *
		 (x306 +
		  (-1 * x143 *
		   ((x108 * ((x95 * x311) + (x304 * x141) + (x304 * x128) +
					 (x95 * ((x305 * x104) + x311 +
							 (x95 * ((x95 * ((x304 * x134) + x309 + (-1 * x305 * x130))) + x310 + (x304 * x135))))))) +
			(-1 * x308 * x127))) +
		  (x311 * x144) + (x305 * x111) + (-1 * x308 * x125)));
	const FLT x313 = x285 + x236 + x239;
	const FLT x314 = -1 * x244;
	const FLT x315 = x246 + x314 + x287;
	const FLT x316 = -1 * x229;
	const FLT x317 = x280 + x228 + x316;
	const FLT x318 = (x317 * sensor_pt[0]) + (x313 * sensor_pt[2]) + (-1 * x315 * sensor_pt[1]);
	const FLT x319 = x250 + (-1 * x44);
	const FLT x320 = x319 + x248 + x282;
	const FLT x321 = (x315 * sensor_pt[0]) + (x317 * sensor_pt[1]) + (-1 * x320 * sensor_pt[2]);
	const FLT x322 = (x257 * x320) + (x254 * x321) + (-1 * x235 * x313) + (-1 * x256 * x318);
	const FLT x323 = (x317 * sensor_pt[2]) + (-1 * x313 * sensor_pt[0]) + (x320 * sensor_pt[1]);
	const FLT x324 = (-1 * x254 * x323) + (x261 * x318) + (-1 * x260 * x320) + (x235 * x315);
	const FLT x325 = (x256 * x323) + (-1 * x257 * x315) + (x260 * x313) + (-1 * x261 * x321);
	const FLT x326 = (x325 * (*lh_p).Rot[3]) + (-1 * x322 * (*lh_p).Rot[1]) + (x324 * (*lh_p).Rot[0]);
	const FLT x327 = (-1 * x325 * (*lh_p).Rot[2]) + (x322 * (*lh_p).Rot[0]) + (x324 * (*lh_p).Rot[1]);
	const FLT x328 = (-1 * x326 * x184) + x325 + (x327 * x188);
	const FLT x329 = (x322 * (*lh_p).Rot[2]) + (-1 * x324 * (*lh_p).Rot[3]) + (x325 * (*lh_p).Rot[0]);
	const FLT x330 = x322 + (-1 * x329 * x188) + (x297 * x326);
	const FLT x331 = (x75 * x330) + (x69 * x328);
	const FLT x332 = x324 + (-1 * x297 * x327) + (x329 * x184);
	const FLT x333 = (-1 * x91 * x332) + (x331 * x113);
	const FLT x334 = ((-1 * x330 * x118) + (x328 * x117)) * x119;
	const FLT x335 = (-1 * x334) + (x333 * x116);
	const FLT x336 = (x86 * x332) + (-1 * x85 * (x331 + (x61 * x332)));
	const FLT x337 = x88 * x336;
	const FLT x338 = x97 * x337;
	const FLT x339 = (x98 * x337) + (x95 * ((-1 * x96 * x337) + x338));
	const FLT x340 = (x99 * x337) + (x95 * x339);
	const FLT x341 =
		x334 +
		(-1 * x146 *
		 (x333 + (-1 * x335 * x125) +
		  (-1 * x143 *
		   ((x108 * ((x95 * x340) + (x336 * x141) + (x337 * x105) +
					 (x95 * (x340 + (x337 * x104) +
							 (x95 * (x339 + (x95 * (x338 + (x336 * x134) + (-1 * x337 * x130))) + (x336 * x135))))))) +
			(-1 * x335 * x127))) +
		  (x340 * x144) + (x337 * x111)));
	const FLT x342 = x243 + x288 + x314;
	const FLT x343 = x241 + x236 + x238;
	const FLT x344 = x319 + x249 + x283;
	const FLT x345 = (x344 * sensor_pt[0]) + (-1 * x343 * sensor_pt[1]) + (x342 * sensor_pt[2]);
	const FLT x346 = x233 + x316 + x279;
	const FLT x347 = (x343 * sensor_pt[0]) + (-1 * x346 * sensor_pt[2]) + (x344 * sensor_pt[1]);
	const FLT x348 = (x257 * x346) + (-1 * x235 * x342) + (x254 * x347) + (-1 * x256 * x345);
	const FLT x349 = (x344 * sensor_pt[2]) + (-1 * x342 * sensor_pt[0]) + (x346 * sensor_pt[1]);
	const FLT x350 = (x235 * x343) + (x261 * x345) + (-1 * x260 * x346) + (-1 * x254 * x349);
	const FLT x351 = (-1 * x261 * x347) + (-1 * x257 * x343) + (x260 * x342) + (x256 * x349);
	const FLT x352 = (x351 * (*lh_p).Rot[3]) + (-1 * x348 * (*lh_p).Rot[1]) + (x350 * (*lh_p).Rot[0]);
	const FLT x353 = (x348 * (*lh_p).Rot[0]) + (x350 * (*lh_p).Rot[1]) + (-1 * x351 * (*lh_p).Rot[2]);
	const FLT x354 = x351 + (-1 * x352 * x184) + (x353 * x188);
	const FLT x355 = (-1 * x350 * (*lh_p).Rot[3]) + (x348 * (*lh_p).Rot[2]) + (x351 * (*lh_p).Rot[0]);
	const FLT x356 = x348 + (-1 * x355 * x188) + (x297 * x352);
	const FLT x357 = (x75 * x356) + (x69 * x354);
	const FLT x358 = (x355 * x184) + x350 + (-1 * x297 * x353);
	const FLT x359 = (-1 * x91 * x358) + (x357 * x113);
	const FLT x360 = ((-1 * x356 * x118) + (x354 * x117)) * x119;
	const FLT x361 = (-1 * x360) + (x359 * x116);
	const FLT x362 = (x86 * x358) + (-1 * x85 * (x357 + (x61 * x358)));
	const FLT x363 = x88 * x362;
	const FLT x364 = x362 * x132;
	const FLT x365 = (x362 * x137) + (x95 * ((-1 * x362 * x136) + x364));
	const FLT x366 = (x362 * x139) + (x95 * x365);
	const FLT x367 =
		x360 +
		(-1 * x146 *
		 (x359 + (x363 * x111) + (x366 * x144) +
		  (-1 * x143 *
		   ((x108 * ((x95 * x366) +
					 (x95 * ((x362 * x129) + x366 +
							 (x95 * (x365 + (x95 * ((x362 * x134) + (-1 * x363 * x130) + x364)) + (x362 * x135))))) +
					 (x362 * x128) + (x362 * x141))) +
			(-1 * x361 * x127))) +
		  (-1 * x361 * x125)));
	const FLT x368 = dt * dt * dt;
	const FLT x369 = 0.5 * x24 * x368;
	const FLT x370 = x50 * x369;
	const FLT x371 = x13 * x13 * x13;
	const FLT x372 = 1.0 * x26;
	const FLT x373 = x22 * x372;
	const FLT x374 = dt * dt * dt * dt;
	const FLT x375 = 1. / (x19 * sqrt(x19));
	const FLT x376 = x374 * x375;
	const FLT x377 = x373 * x376;
	const FLT x378 = x13 * x10;
	const FLT x379 = 2 * x25;
	const FLT x380 = 2 * x23 * (1. / (x19 * x19));
	const FLT x381 = x374 * x380;
	const FLT x382 = x11 * x381;
	const FLT x383 = x11 * x377;
	const FLT x384 = x13 * x17;
	const FLT x385 = x31 * x378;
	const FLT x386 = (x379 * x378) + (-1 * x371 * x381) + (x371 * x377) + (-1 * x373 * x385) + (-1 * x13 * x382) +
					 (-1 * x384 * x381) + (x377 * x384) + (x13 * x383);
	const FLT x387 = 1.0 / 2.0 * (1. / (x27 * sqrt(x27)));
	const FLT x388 = x32 * x387;
	const FLT x389 = x22 * x388;
	const FLT x390 = x16 * x389;
	const FLT x391 = x390 * x386;
	const FLT x392 = x13 * x389;
	const FLT x393 = x34 * x386;
	const FLT x394 = x375 * x368;
	const FLT x395 = x13 * x394;
	const FLT x396 = x37 * x29;
	const FLT x397 = x5 * x396;
	const FLT x398 = x395 * x397;
	const FLT x399 = x26 * x387;
	const FLT x400 = x399 * x386;
	const FLT x401 = x39 * x369;
	const FLT x402 = x5 * x13;
	const FLT x403 = x402 * x401;
	const FLT x404 = x5 * x22;
	const FLT x405 = x404 * x388;
	const FLT x406 = x37 * x386;
	const FLT x407 = 0.5 * x385;
	const FLT x408 = -1 * x36;
	const FLT x409 = x14 * x394;
	const FLT x410 = x34 * x29;
	const FLT x411 = x40 * x29;
	const FLT x412 = x16 * x394;
	const FLT x413 = x13 * x412;
	const FLT x414 = x43 * x369;
	const FLT x415 = x16 * x414;
	const FLT x416 = (x13 * x415) + (-1 * x413 * x411);
	const FLT x417 = x416 + (x409 * x410) + x408 + (-1 * x30 * x407) + (-1 * x406 * x405) + x403 + (-1 * x14 * x370) +
					 (-1 * x40 * x391) + (-1 * x398) + (x393 * x392) + (-1 * x9 * x400);
	const FLT x418 = x5 * x30;
	const FLT x419 = x418 * x395;
	const FLT x420 = x9 * x386;
	const FLT x421 = x413 * x410;
	const FLT x422 = x13 * x16;
	const FLT x423 = x422 * x370;
	const FLT x424 = x47 * x369;
	const FLT x425 = x402 * x424;
	const FLT x426 = x40 * x386;
	const FLT x427 = (-1 * x426 * x392) + (x14 * x414) + (-1 * x390 * x393) + (-1 * x421) + (x405 * x420) +
					 (-1 * x409 * x411) + x419 + (-1 * x407 * x396) + x423 + (-1 * x37 * x400) + (-1 * x425) + x41;
	const FLT x428 = -1 * x45;
	const FLT x429 = x30 * x413;
	const FLT x430 = x422 * x424;
	const FLT x431 = x5 * x370;
	const FLT x432 = x5 * x410;
	const FLT x433 = (x432 * x395) + (-1 * x13 * x431);
	const FLT x434 = x433 + (x406 * x392) + (-1 * x430) + (x405 * x393) + (-1 * x14 * x401) + (x409 * x396) + x428 +
					 (-1 * x40 * x400) + (-1 * x407 * x411) + x429 + (x420 * x390);
	const FLT x435 = 0.5 * x410;
	const FLT x436 = x5 * x411;
	const FLT x437 = (x402 * x414) + (-1 * x436 * x395);
	const FLT x438 = x16 * x401;
	const FLT x439 = (x413 * x396) + (-1 * x13 * x438);
	const FLT x440 = x439 + x437 + (-1 * x30 * x409) + (x37 * x391) + (-1 * x420 * x392) + (-1 * x405 * x426) +
					 (x14 * x424) + (-1 * x435 * x385) + x33 + (-1 * x34 * x400);
	const FLT x441 = (x440 * sensor_pt[0]) + (-1 * x427 * sensor_pt[2]) + (x434 * sensor_pt[1]);
	const FLT x442 = (x434 * sensor_pt[0]) + (-1 * x440 * sensor_pt[1]) + (x417 * sensor_pt[2]);
	const FLT x443 = (-1 * x442 * x256) + (-1 * x417 * x235) + (x441 * x254) + (x427 * x257);
	const FLT x444 = (x434 * sensor_pt[2]) + (-1 * x417 * sensor_pt[0]) + (x427 * sensor_pt[1]);
	const FLT x445 = (x440 * x235) + (-1 * x444 * x254) + (-1 * x427 * x260) + (x442 * x261);
	const FLT x446 = (x417 * x260) + (-1 * x441 * x261) + (-1 * x440 * x257) + (x444 * x256);
	const FLT x447 = 2 * ((x446 * (*lh_p).Rot[3]) + (-1 * x443 * (*lh_p).Rot[1]) + (x445 * (*lh_p).Rot[0]));
	const FLT x448 = 2 * ((-1 * x446 * (*lh_p).Rot[2]) + (x443 * (*lh_p).Rot[0]) + (x445 * (*lh_p).Rot[1]));
	const FLT x449 = x446 + (-1 * x447 * (*lh_p).Rot[3]) + (x448 * (*lh_p).Rot[2]);
	const FLT x450 = 2 * ((x443 * (*lh_p).Rot[2]) + (-1 * x445 * (*lh_p).Rot[3]) + (x446 * (*lh_p).Rot[0]));
	const FLT x451 = (-1 * x450 * (*lh_p).Rot[2]) + x443 + (x447 * (*lh_p).Rot[1]);
	const FLT x452 = (x75 * x451) + (x69 * x449);
	const FLT x453 = (-1 * x448 * (*lh_p).Rot[1]) + x445 + (x450 * (*lh_p).Rot[3]);
	const FLT x454 = (-1 * x91 * x453) + (x452 * x113);
	const FLT x455 = ((-1 * x451 * x118) + (x449 * x117)) * x119;
	const FLT x456 = (-1 * x455) + (x454 * x116);
	const FLT x457 = (x86 * x453) + (-1 * x85 * (x452 + (x61 * x453)));
	const FLT x458 = x457 * x132;
	const FLT x459 = (x457 * x137) + (x95 * ((-1 * x457 * x136) + x458));
	const FLT x460 = (x457 * x139) + (x95 * x459);
	const FLT x461 =
		x455 +
		(-1 * x146 *
		 ((x460 * x144) + x454 + (x457 * x112) +
		  (-1 * x143 *
		   ((x108 * ((x95 * x460) + (x457 * x141) + (x457 * x128) +
					 (x95 * (x460 + (x457 * x129) +
							 (x95 * (x459 + (x95 * ((x457 * x134) + (-1 * x457 * x131) + x458)) + (x457 * x135))))))) +
			(-1 * x456 * x127))) +
		  (-1 * x456 * x125)));
	const FLT x462 = x17 * x394;
	const FLT x463 = x14 * x16;
	const FLT x464 = x31 * x10;
	const FLT x465 = x16 * x464;
	const FLT x466 = (x16 * x16 * x16) * x374;
	const FLT x467 = x10 * x379;
	const FLT x468 = (-1 * x466 * x380) + (x16 * x467) + (-1 * x463 * x381) + (x16 * x383) + (x466 * x373 * x375) +
					 (x463 * x377) + (-1 * x465 * x373) + (-1 * x16 * x382);
	const FLT x469 = x9 * x405;
	const FLT x470 = x37 * x399;
	const FLT x471 = x34 * x468;
	const FLT x472 = x17 * x369;
	const FLT x473 = 0.5 * x465;
	const FLT x474 = x40 * x392;
	const FLT x475 = (-1 * x5 * x16 * x424) + (x412 * x418);
	const FLT x476 = x416 + (-1 * x473 * x396) + x36 + (-1 * x462 * x410) + (x469 * x468) + (-1 * x471 * x390) + x475 +
					 (x50 * x472) + (-1 * x468 * x474) + (-1 * x468 * x470);
	const FLT x477 = x40 * x390;
	const FLT x478 = x37 * x468;
	const FLT x479 = x9 * x399;
	const FLT x480 = x5 * x438;
	const FLT x481 = x412 * x397;
	const FLT x482 = (-1 * x30 * x473) + x41 + (x43 * x472) + (-1 * x462 * x411) + (-1 * x423) + (x471 * x392) +
					 (-1 * x481) + x421 + (-1 * x478 * x405) + (-1 * x468 * x479) + (-1 * x468 * x477) + x480;
	const FLT x483 = -1 * x33;
	const FLT x484 = x412 * x432;
	const FLT x485 = x9 * x390;
	const FLT x486 = x16 * x431;
	const FLT x487 = x40 * x399;
	const FLT x488 = (x478 * x392) + x439 + (-1 * x468 * x487) + (-1 * x473 * x411) + (-1 * x486) + (-1 * x47 * x472) +
					 (x471 * x405) + x483 + (x468 * x485) + (x30 * x462) + x484;
	const FLT x489 = (x488 * sensor_pt[2]) + (-1 * x482 * sensor_pt[0]) + (x476 * sensor_pt[1]);
	const FLT x490 = x40 * x405;
	const FLT x491 = x34 * x399;
	const FLT x492 = x9 * x392;
	const FLT x493 = (x5 * x415) + (-1 * x412 * x436);
	const FLT x494 = (-1 * x492 * x468) + (-1 * x465 * x435) + x493 + (-1 * x491 * x468) + x428 + (-1 * x39 * x472) +
					 x430 + (x462 * x396) + (-1 * x429) + (-1 * x490 * x468) + (x478 * x390);
	const FLT x495 = (x482 * sensor_pt[2]) + (x488 * sensor_pt[0]) + (-1 * x494 * sensor_pt[1]);
	const FLT x496 = (x494 * x235) + (x495 * x261) + (-1 * x476 * x260) + (-1 * x489 * x254);
	const FLT x497 = (x494 * sensor_pt[0]) + (-1 * x476 * sensor_pt[2]) + (x488 * sensor_pt[1]);
	const FLT x498 = (x476 * x257) + (-1 * x482 * x235) + (x497 * x254) + (-1 * x495 * x256);
	const FLT x499 = (x489 * x256) + (-1 * x497 * x261) + (-1 * x494 * x257) + (x482 * x260);
	const FLT x500 = (x499 * (*lh_p).Rot[3]) + (x496 * (*lh_p).Rot[0]) + (-1 * x498 * (*lh_p).Rot[1]);
	const FLT x501 = (x498 * (*lh_p).Rot[0]) + (-1 * x499 * (*lh_p).Rot[2]) + (x496 * (*lh_p).Rot[1]);
	const FLT x502 = x499 + (-1 * x500 * x184) + (x501 * x188);
	const FLT x503 = (-1 * x496 * (*lh_p).Rot[3]) + (x498 * (*lh_p).Rot[2]) + (x499 * (*lh_p).Rot[0]);
	const FLT x504 = (-1 * x503 * x188) + x498 + (x500 * x297);
	const FLT x505 = (x75 * x504) + (x69 * x502);
	const FLT x506 = x496 + (x503 * x184) + (-1 * x501 * x297);
	const FLT x507 = (-1 * x91 * x506) + (x505 * x113);
	const FLT x508 = ((-1 * x504 * x118) + (x502 * x117)) * x119;
	const FLT x509 = (-1 * x508) + (x507 * x116);
	const FLT x510 = (x86 * x506) + (-1 * x85 * (x505 + (x61 * x506)));
	const FLT x511 = x510 * x132;
	const FLT x512 = (x510 * x137) + (x95 * ((-1 * x510 * x136) + x511));
	const FLT x513 = (x510 * x139) + (x95 * x512);
	const FLT x514 =
		x508 +
		(-1 * x146 *
		 (x507 + (x513 * x144) + (-1 * x509 * x125) +
		  (-1 * x143 *
		   ((x108 * ((x95 * x513) + (x510 * x141) + (x510 * x128) +
					 (x95 * (x513 + (x510 * x129) +
							 (x95 * (x512 + (x95 * ((x510 * x134) + x511 + (-1 * x510 * x131))) + (x510 * x135))))))) +
			(-1 * x509 * x127))) +
		  (x510 * x112)));
	const FLT x515 = x5 * x464;
	const FLT x516 = x404 * x372;
	const FLT x517 = x516 * x376;
	const FLT x518 = x5 * x5 * x5;
	const FLT x519 = x5 * x381;
	const FLT x520 = (x17 * x517) + (x518 * x377) + (x5 * x467) + (x14 * x517) + (-1 * x17 * x519) +
					 (-1 * x464 * x516) + (-1 * x518 * x381) + (-1 * x14 * x519);
	const FLT x521 = x37 * x520;
	const FLT x522 = x11 * x394;
	const FLT x523 = (-1 * x492 * x520) + (-1 * x435 * x515) + (x521 * x390) + (-1 * x480) + (-1 * x490 * x520) + x425 +
					 x41 + (x11 * x414) + (-1 * x411 * x522) + x481 + (-1 * x419) + (-1 * x491 * x520);
	const FLT x524 = x34 * x520;
	const FLT x525 = x433 + x493 + (x524 * x392) + (-1 * x479 * x520) + (x11 * x401) + (-0.5 * x30 * x515) +
					 (-1 * x405 * x521) + (-1 * x522 * x396) + (-1 * x477 * x520) + x45;
	const FLT x526 = 0.5 * x464;
	const FLT x527 = x437 + (-1 * x474 * x520) + (-1 * x470 * x520) + x486 + (-1 * x524 * x390) + (x469 * x520) +
					 (x30 * x522) + (-1 * x484) + x483 + (-1 * x11 * x424) + (-1 * x526 * x397);
	const FLT x528 = (x521 * x392) + x475 + x398 + (x405 * x524) + (-1 * x403) + (x485 * x520) + (-1 * x436 * x526) +
					 (x410 * x522) + (-1 * x11 * x370) + x408 + (-1 * x487 * x520);
	const FLT x529 = (x528 * sensor_pt[2]) + (-1 * x525 * sensor_pt[0]) + (x527 * sensor_pt[1]);
	const FLT x530 = (x523 * sensor_pt[0]) + (-1 * x527 * sensor_pt[2]) + (x528 * sensor_pt[1]);
	const FLT x531 = (-1 * x530 * x261) + (x525 * x260) + (-1 * x523 * x257) + (x529 * x256);
	const FLT x532 = (x528 * sensor_pt[0]) + (-1 * x523 * sensor_pt[1]) + (x525 * sensor_pt[2]);
	const FLT x533 = (-1 * x529 * x254) + (x532 * x261) + (-1 * x527 * x260) + (x523 * x235);
	const FLT x534 = (x530 * x254) + (x527 * x257) + (-1 * x525 * x235) + (-1 * x532 * x256);
	const FLT x535 = (-1 * x531 * (*lh_p).Rot[2]) + (x534 * (*lh_p).Rot[0]) + (x533 * (*lh_p).Rot[1]);
	const FLT x536 = (x534 * (*lh_p).Rot[2]) + (-1 * x533 * (*lh_p).Rot[3]) + (x531 * (*lh_p).Rot[0]);
	const FLT x537 = x533 + (-1 * x535 * x297) + (x536 * x184);
	const FLT x538 = (x533 * (*lh_p).Rot[0]) + (x531 * (*lh_p).Rot[3]) + (-1 * x534 * (*lh_p).Rot[1]);
	const FLT x539 = x531 + (-1 * x538 * x184) + (x535 * x188);
	const FLT x540 = x534 + (x538 * x297) + (-1 * x536 * x188);
	const FLT x541 = (x75 * x540) + (x69 * x539);
	const FLT x542 = (x86 * x537) + (-1 * x85 * (x541 + (x61 * x537)));
	const FLT x543 = (-1 * x91 * x537) + (x541 * x113);
	const FLT x544 = ((-1 * x540 * x118) + (x539 * x117)) * x119;
	const FLT x545 = (-1 * x544) + (x543 * x116);
	const FLT x546 = x542 * x132;
	const FLT x547 = (x542 * x137) + (x95 * ((-1 * x542 * x136) + x546));
	const FLT x548 = (x542 * x139) + (x95 * x547);
	const FLT x549 =
		x544 +
		(-1 * x146 *
		 (x543 + (x548 * x144) +
		  (-1 * x143 *
		   ((x108 * ((x95 * x548) +
					 (x95 * (x548 + (x542 * x129) +
							 (x95 * (x547 + (x95 * ((x542 * x134) + (-1 * x542 * x131) + x546)) + (x542 * x135))))) +
					 (x542 * x141) + (x542 * x128))) +
			(-1 * x545 * x127))) +
		  (x542 * x112) + (-1 * x545 * x125)));
	const FLT x550 = -1 * dt * x177;
	const FLT x551 = (-1 * dt * x179) + dt;
	const FLT x552 = x551 + x550;
	const FLT x553 = dt * x183;
	const FLT x554 = dt * x185;
	const FLT x555 = x554 + (-1 * x553);
	const FLT x556 = (x75 * x555) + (x69 * x552);
	const FLT x557 = dt * x189;
	const FLT x558 = dt * x190;
	const FLT x559 = x558 + x557;
	const FLT x560 = (-1 * x91 * x559) + (x556 * x113);
	const FLT x561 = ((-1 * x555 * x118) + (x552 * x117)) * x119;
	const FLT x562 = (-1 * x561) + (x560 * x116);
	const FLT x563 = (x86 * x559) + (-1 * x85 * (x556 + (x61 * x559)));
	const FLT x564 = x563 * x132;
	const FLT x565 = (x563 * x137) + (x95 * ((-1 * x563 * x136) + x564));
	const FLT x566 = (x563 * x139) + (x95 * x565);
	const FLT x567 =
		x561 +
		(-1 * x146 *
		 ((x566 * x144) + (-1 * x562 * x125) + x560 + (x563 * x112) +
		  (-1 * x143 *
		   ((x108 * ((x95 * x566) + (x563 * x141) +
					 (x95 * ((x563 * x129) + x566 +
							 (x95 * ((x95 * ((x563 * x134) + (-1 * x563 * x131) + x564)) + x565 + (x563 * x135))))) +
					 (x563 * x128))) +
			(-1 * x562 * x127)))));
	const FLT x568 = -1 * dt * x205;
	const FLT x569 = x568 + x550 + dt;
	const FLT x570 = x557 + (-1 * x558);
	const FLT x571 = dt * x201;
	const FLT x572 = dt * x202;
	const FLT x573 = x572 + x571;
	const FLT x574 = (x75 * x573) + (x69 * x570);
	const FLT x575 = (x86 * x569) + (-1 * x85 * (x574 + (x61 * x569)));
	const FLT x576 = (-1 * x91 * x569) + (x574 * x113);
	const FLT x577 = ((-1 * x573 * x118) + (x570 * x117)) * x119;
	const FLT x578 = (-1 * x577) + (x576 * x116);
	const FLT x579 = x575 * x132;
	const FLT x580 = (x575 * x137) + (x95 * ((-1 * x575 * x136) + x579));
	const FLT x581 = (x575 * x139) + (x95 * x580);
	const FLT x582 =
		x577 +
		(-1 * x146 *
		 (x576 +
		  (-1 * x143 *
		   ((x108 * ((x95 * x581) + (x575 * x141) + (x575 * x128) +
					 (x95 * (x581 + (x575 * x129) +
							 (x95 * (x580 + (x95 * ((x575 * x134) + x579 + (-1 * x575 * x131))) + (x575 * x135))))))) +
			(-1 * x578 * x127))) +
		  (x581 * x144) + (x575 * x112) + (-1 * x578 * x125)));
	const FLT x583 = x553 + x554;
	const FLT x584 = x551 + x568;
	const FLT x585 = (x75 * x584) + (x69 * x583);
	const FLT x586 = x571 + (-1 * x572);
	const FLT x587 = (-1 * x91 * x586) + (x585 * x113);
	const FLT x588 = ((-1 * x584 * x118) + (x583 * x117)) * x119;
	const FLT x589 = (-1 * x588) + (x587 * x116);
	const FLT x590 = (x86 * x586) + (-1 * x85 * (x585 + (x61 * x586)));
	const FLT x591 = x590 * x132;
	const FLT x592 = (x590 * x137) + (x95 * ((-1 * x590 * x136) + x591));
	const FLT x593 = (x590 * x139) + (x95 * x592);
	const FLT x594 =
		x588 +
		(-1 * x146 *
		 (x587 + (x593 * x144) + (x590 * x112) + (-1 * x589 * x125) +
		  (-1 * x143 *
		   ((x108 * ((x95 * x593) + (x590 * x141) + (x590 * x128) +
					 (x95 * (x593 + (x590 * x129) +
							 (x95 * (x592 + (x95 * ((x590 * x134) + x591 + (-1 * x590 * x131))) + (x590 * x135))))))) +
			(-1 * x589 * x127)))));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[0]) / sizeof(FLT), x147 + (x148 * x147));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[1]) / sizeof(FLT), x164 + (x164 * x148));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Acc[2]) / sizeof(FLT), x176 + (x176 * x148));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT), x199 + (x199 * x148));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT), x215 + (x215 * x148));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT), x227 + (x227 * x148));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x278 + (x278 * x148));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x312 + (x312 * x148));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x341 + (x341 * x148));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x367 + (x367 * x148));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x461 + (x461 * x148));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x514 + (x514 * x148));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x549 + (x549 * x148));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[0]) / sizeof(FLT), x567 + (x567 * x148));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[1]) / sizeof(FLT), x582 + (x582 * x148));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanModel, Velocity.Pos[2]) / sizeof(FLT), x594 + (x594 * x148));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen2 wrt [(*_x0).AccBias[0], (*_x0).AccBias[1],
// (*_x0).AccBias[2], (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1],
// (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2],
// (*_x0).IMUCorrection[3], (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0],
// (*_x0).Pose.Rot[1], (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0],
// (*_x0).Velocity.AxisAngleRot[1], (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1],
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f2b1c0>]

static inline void SurviveKalmanErrorModel_LightMeas_y_gen2_jac_x0_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_y_gen2(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_y_gen2_jac_x0(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen2 wrt [(*error_model).AccBias[0], (*error_model).AccBias[1],
// (*error_model).AccBias[2], (*error_model).Acc[0], (*error_model).Acc[1], (*error_model).Acc[2],
// (*error_model).GyroBias[0], (*error_model).GyroBias[1], (*error_model).GyroBias[2], (*error_model).IMUCorrection[0],
// (*error_model).IMUCorrection[1], (*error_model).IMUCorrection[2], (*error_model).IMUCorrection[3],
// (*error_model).Pose.AxisAngleRot[0], (*error_model).Pose.AxisAngleRot[1], (*error_model).Pose.AxisAngleRot[2],
// (*error_model).Pose.Pos[0], (*error_model).Pose.Pos[1], (*error_model).Pose.Pos[2],
// (*error_model).Velocity.AxisAngleRot[0], (*error_model).Velocity.AxisAngleRot[1],
// (*error_model).Velocity.AxisAngleRot[2], (*error_model).Velocity.Pos[0], (*error_model).Velocity.Pos[1],
// (*error_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f162b0>]
static inline void SurviveKalmanErrorModel_LightMeas_y_gen2_jac_error_model(
	CnMat *Hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const FLT x0 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x1 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x2 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x4 =
		(-1 * x3 * (*_x0).Pose.Rot[1]) + (x2 * (*_x0).Pose.Rot[0]) + (x1 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x5 = dt * dt;
	const FLT x6 = x0 * x0;
	const FLT x7 = x6 * x5;
	const FLT x8 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x9 = x8 * x8;
	const FLT x10 = x5 * x9;
	const FLT x11 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x12 = x11 * x11;
	const FLT x13 = x5 * x12;
	const FLT x14 = 1e-10 + x13 + x7 + x10;
	const FLT x15 = sqrt(x14);
	const FLT x16 = 0.5 * x15;
	const FLT x17 = sin(x16);
	const FLT x18 = x17 * x17;
	const FLT x19 = 1. / x14;
	const FLT x20 = x19 * x18;
	const FLT x21 = cos(x16);
	const FLT x22 = (x20 * x10) + (x21 * x21) + (x7 * x20) + (x20 * x13);
	const FLT x23 = 1. / sqrt(x22);
	const FLT x24 = x23 * x17;
	const FLT x25 = 1. / x15;
	const FLT x26 = dt * x25;
	const FLT x27 = x24 * x26;
	const FLT x28 = x4 * x27;
	const FLT x29 =
		(x2 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x1 * (*_x0).Pose.Rot[2]) + (x3 * (*_x0).Pose.Rot[0]);
	const FLT x30 = x29 * x27;
	const FLT x31 =
		(*_x0).Pose.Rot[1] + (x1 * (*_x0).Pose.Rot[0]) + (-1 * x2 * (*_x0).Pose.Rot[3]) + (x3 * (*_x0).Pose.Rot[2]);
	const FLT x32 = x23 * x21;
	const FLT x33 = x32 * x31;
	const FLT x34 = (-1 * x1 * (*_x0).Pose.Rot[1]) + (-1 * x2 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x3 * (*_x0).Pose.Rot[3]);
	const FLT x35 = x34 * x24;
	const FLT x36 = x35 * x26;
	const FLT x37 = x33 + (x8 * x36) + (-1 * x0 * x28) + (x30 * x11);
	const FLT x38 = x32 * x34;
	const FLT x39 = x31 * x27;
	const FLT x40 = (-1 * x0 * x30) + (-1 * x28 * x11) + (-1 * x8 * x39) + x38;
	const FLT x41 = x4 * x32;
	const FLT x42 = x41 + (-1 * x8 * x30) + (x0 * x39) + (x36 * x11);
	const FLT x43 = (-1 * x42 * sensor_pt[0]) + (x37 * sensor_pt[1]) + (x40 * sensor_pt[2]);
	const FLT x44 = x32 * x29;
	const FLT x45 = (x8 * x28) + x44 + (-1 * x39 * x11) + (x0 * x36);
	const FLT x46 = (-1 * x45 * sensor_pt[1]) + (x42 * sensor_pt[2]) + (x40 * sensor_pt[0]);
	const FLT x47 = dt * fabs(dt);
	const FLT x48 = 1.0 / 2.0 * x47;
	const FLT x49 = (2 * ((x45 * x46) + (-1 * x43 * x37))) + (*error_model).Pose.Pos[1] + (*_x0).Pose.Pos[1] +
					(x48 * ((*_x0).Acc[1] + (*error_model).Acc[1])) + sensor_pt[1] +
					(dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1]));
	const FLT x50 = (-1 * x37 * sensor_pt[2]) + (x40 * sensor_pt[1]) + (x45 * sensor_pt[0]);
	const FLT x51 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (*error_model).Pose.Pos[2] +
					(x48 * ((*_x0).Acc[2] + (*error_model).Acc[2])) + (2 * ((x50 * x37) + (-1 * x42 * x46))) +
					(dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2]));
	const FLT x52 = (x48 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + sensor_pt[0] +
					(2 * ((x42 * x43) + (-1 * x50 * x45))) +
					(dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) + (*error_model).Pose.Pos[0] +
					(*_x0).Pose.Pos[0];
	const FLT x53 = (-1 * x52 * (*lh_p).Rot[2]) + (x49 * (*lh_p).Rot[1]) + (x51 * (*lh_p).Rot[0]);
	const FLT x54 = (-1 * x49 * (*lh_p).Rot[3]) + (x52 * (*lh_p).Rot[0]) + (x51 * (*lh_p).Rot[2]);
	const FLT x55 = x49 + (*lh_p).Pos[1] + (2 * ((x54 * (*lh_p).Rot[3]) + (-1 * x53 * (*lh_p).Rot[1])));
	const FLT x56 = x55 * x55;
	const FLT x57 = (-1 * x51 * (*lh_p).Rot[1]) + (x49 * (*lh_p).Rot[0]) + (x52 * (*lh_p).Rot[3]);
	const FLT x58 = (2 * ((x57 * (*lh_p).Rot[1]) + (-1 * x54 * (*lh_p).Rot[2]))) + x51 + (*lh_p).Pos[2];
	const FLT x59 = (*lh_p).Pos[0] + x52 + (2 * ((x53 * (*lh_p).Rot[2]) + (-1 * x57 * (*lh_p).Rot[3])));
	const FLT x60 = x59 * x59;
	const FLT x61 = x60 + (x58 * x58);
	const FLT x62 = x61 + x56;
	const FLT x63 = 0.523598775598299 + (-1 * (*bsc0).tilt);
	const FLT x64 = cos(x63);
	const FLT x65 = 1. / sqrt(1 + (-1 * (1. / (x64 * x64)) * (1. / x62) * x56));
	const FLT x66 = x47 * (*lh_p).Rot[1] * (*lh_p).Rot[2];
	const FLT x67 = x47 * (*lh_p).Rot[3];
	const FLT x68 = x67 * (*lh_p).Rot[0];
	const FLT x69 = x68 + x66;
	const FLT x70 = 2 * x55;
	const FLT x71 = (*lh_p).Rot[2] * (*lh_p).Rot[2];
	const FLT x72 = -1 * x71 * x47;
	const FLT x73 = (*lh_p).Rot[3] * (*lh_p).Rot[3];
	const FLT x74 = x48 + (-1 * x73 * x47);
	const FLT x75 = x74 + x72;
	const FLT x76 = 2 * x59;
	const FLT x77 = x47 * (*lh_p).Rot[0];
	const FLT x78 = x77 * (*lh_p).Rot[2];
	const FLT x79 = x67 * (*lh_p).Rot[1];
	const FLT x80 = x79 + (-1 * x78);
	const FLT x81 = 2 * x58;
	const FLT x82 = (x80 * x81) + (x75 * x76);
	const FLT x83 = 1. / x64;
	const FLT x84 = 1.0 / 2.0 * x55;
	const FLT x85 = x83 * x84 * (1. / (x62 * sqrt(x62)));
	const FLT x86 = x83 * (1. / sqrt(x62));
	const FLT x87 = (x86 * x69) + (-1 * x85 * (x82 + (x70 * x69)));
	const FLT x88 = x87 * x65;
	const FLT x89 = atan2(-1 * x58, x59);
	const FLT x90 = tan(x63);
	const FLT x91 = (1. / sqrt(x61)) * x90;
	const FLT x92 = -1 * x55 * x91;
	const FLT x93 = asin(x92) + (-1 * x89) + (-1 * (*bsc0).ogeephase);
	const FLT x94 = (-1 * sin(x93) * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x95 = asin(x86 * x55);
	const FLT x96 = 8.0108022e-06 * x95;
	const FLT x97 = -8.0108022e-06 + (-1 * x96);
	const FLT x98 = 0.0028679863 + (x97 * x95);
	const FLT x99 = 5.3685255e-06 + (x98 * x95);
	const FLT x100 = 0.0076069798 + (x99 * x95);
	const FLT x101 = x95 * x100;
	const FLT x102 = -8.0108022e-06 + (-1.60216044e-05 * x95);
	const FLT x103 = x98 + (x95 * x102);
	const FLT x104 = x99 + (x95 * x103);
	const FLT x105 = x100 + (x95 * x104);
	const FLT x106 = (x95 * x105) + x101;
	const FLT x107 = sin(x63);
	const FLT x108 = x94 * x107;
	const FLT x109 = x64 + (x108 * x106);
	const FLT x110 = 1. / x109;
	const FLT x111 = 2 * x94 * x101 * x110;
	const FLT x112 = x84 * (1. / (x61 * sqrt(x61))) * x90;
	const FLT x113 = (-1 * x69 * x91) + (x82 * x112);
	const FLT x114 = 1. / x61;
	const FLT x115 = 1. / sqrt(1 + (-1 * x56 * (x90 * x90) * x114));
	const FLT x116 = (1. / x60) * x58;
	const FLT x117 = 1. / x59;
	const FLT x118 = x60 * x114;
	const FLT x119 = ((-1 * x80 * x117) + (x75 * x116)) * x118;
	const FLT x120 = (-1 * x119) + (x113 * x115);
	const FLT x121 = cos(x93) * (*bsc0).ogeemag;
	const FLT x122 = x95 * x95;
	const FLT x123 = x100 * x110 * x122;
	const FLT x124 = x123 * x121;
	const FLT x125 = x107 * x106;
	const FLT x126 = x121 * x125;
	const FLT x127 = x65 * x105;
	const FLT x128 = 2.40324066e-05 * x95;
	const FLT x129 = x65 * x97;
	const FLT x130 = x87 * x129;
	const FLT x131 = x65 * x103;
	const FLT x132 = x65 * x98;
	const FLT x133 = (x87 * x132) + (x95 * ((-1 * x88 * x96) + x130));
	const FLT x134 = x65 * x99;
	const FLT x135 = (x87 * x134) + (x95 * x133);
	const FLT x136 = x94 * x122;
	const FLT x137 = x100 * (1. / (x109 * x109)) * x136;
	const FLT x138 = x110 * x136;
	const FLT x139 = x92 + (x100 * x138);
	const FLT x140 = 1. / sqrt(1 + (-1 * (x139 * x139)));
	const FLT x141 =
		x119 +
		(-1 * x140 *
		 ((x138 * x135) +
		  (-1 * x137 *
		   ((x108 * ((x88 * x100) + (x87 * x127) + (x95 * x135) +
					 (x95 * ((x88 * x104) + x135 +
							 (x95 * ((x95 * ((x88 * x102) + (-1 * x88 * x128) + x130)) + x133 + (x87 * x131))))))) +
			(-1 * x120 * x126))) +
		  (x88 * x111) + x113 + (-1 * x120 * x124)));
	const FLT x142 = cos((-1 * asin(x139)) + (*bsc0).gibpha + x89) * (*bsc0).gibmag;
	const FLT x143 = (*lh_p).Rot[1] * (*lh_p).Rot[1];
	const FLT x144 = -1 * x47 * x143;
	const FLT x145 = x74 + x144;
	const FLT x146 = x66 + (-1 * x68);
	const FLT x147 = x67 * (*lh_p).Rot[2];
	const FLT x148 = x77 * (*lh_p).Rot[1];
	const FLT x149 = x148 + x147;
	const FLT x150 = (x81 * x149) + (x76 * x146);
	const FLT x151 = (x86 * x145) + (-1 * x85 * (x150 + (x70 * x145)));
	const FLT x152 = x65 * x151;
	const FLT x153 = (-1 * x91 * x145) + (x112 * x150);
	const FLT x154 = ((-1 * x117 * x149) + (x116 * x146)) * x118;
	const FLT x155 = (-1 * x154) + (x115 * x153);
	const FLT x156 = x97 * x152;
	const FLT x157 = (x98 * x152) + (x95 * ((-1 * x96 * x152) + x156));
	const FLT x158 = (x99 * x152) + (x95 * x157);
	const FLT x159 =
		x154 +
		(-1 * x140 *
		 (x153 + (-1 * x124 * x155) + (x138 * x158) + (x111 * x152) +
		  (-1 * x137 *
		   ((x108 * ((x95 * x158) +
					 (x95 * (x158 + (x104 * x152) +
							 (x95 * (x157 + (x95 * ((x102 * x152) + (-1 * x128 * x152) + x156)) + (x131 * x151))))) +
					 (x100 * x152) + (x127 * x151))) +
			(-1 * x126 * x155)))));
	const FLT x160 = x147 + (-1 * x148);
	const FLT x161 = x78 + x79;
	const FLT x162 = x72 + x48 + x144;
	const FLT x163 = (x81 * x162) + (x76 * x161);
	const FLT x164 = (x86 * x160) + (-1 * x85 * (x163 + (x70 * x160)));
	const FLT x165 = x65 * x111;
	const FLT x166 = (-1 * x91 * x160) + (x112 * x163);
	const FLT x167 = ((-1 * x117 * x162) + (x116 * x161)) * x118;
	const FLT x168 = (-1 * x167) + (x115 * x166);
	const FLT x169 = x65 * x100;
	const FLT x170 = x65 * x104;
	const FLT x171 = x129 * x164;
	const FLT x172 = x65 * x128;
	const FLT x173 = x65 * x102;
	const FLT x174 = x65 * x96;
	const FLT x175 = (x164 * x132) + (x95 * ((-1 * x164 * x174) + x171));
	const FLT x176 = (x164 * x134) + (x95 * x175);
	const FLT x177 =
		x167 +
		(-1 * x140 *
		 (x166 + (x176 * x138) + (-1 * x124 * x168) + (x165 * x164) +
		  (-1 * x137 *
		   ((x108 * ((x95 * x176) + (x127 * x164) +
					 (x95 * (x176 + (x164 * x170) +
							 (x95 * (x175 + (x95 * ((x164 * x173) + x171 + (-1 * x164 * x172))) + (x164 * x131))))) +
					 (x169 * x164))) +
			(-1 * x126 * x168)))));
	const FLT x178 = 0.5 * x32;
	const FLT x179 = x178 * (*_x0).Pose.Rot[2];
	const FLT x180 = 0.5 * x27;
	const FLT x181 = x11 * x180;
	const FLT x182 = x181 * (*_x0).Pose.Rot[0];
	const FLT x183 = x8 * x180;
	const FLT x184 = x183 * (*_x0).Pose.Rot[3];
	const FLT x185 = x0 * x180;
	const FLT x186 = x185 * (*_x0).Pose.Rot[1];
	const FLT x187 = (-1 * x186) + x184 + (-1 * x179) + (-1 * x182);
	const FLT x188 = 2 * x50;
	const FLT x189 = (-1 * x181 * (*_x0).Pose.Rot[2]) + (-1 * x183 * (*_x0).Pose.Rot[1]) +
					 (-1 * x185 * (*_x0).Pose.Rot[3]) + (x178 * (*_x0).Pose.Rot[0]);
	const FLT x190 = x189 * sensor_pt[2];
	const FLT x191 = x183 * (*_x0).Pose.Rot[0];
	const FLT x192 = x185 * (*_x0).Pose.Rot[2];
	const FLT x193 = x181 * (*_x0).Pose.Rot[3];
	const FLT x194 = x178 * (*_x0).Pose.Rot[1];
	const FLT x195 = (-1 * x194) + (-1 * x193) + (-1 * x191) + x192;
	const FLT x196 = x187 * sensor_pt[0];
	const FLT x197 = (-1 * x190) + x196 + (x195 * sensor_pt[1]);
	const FLT x198 = 2 * x45;
	const FLT x199 = x183 * (*_x0).Pose.Rot[2];
	const FLT x200 = x178 * (*_x0).Pose.Rot[3];
	const FLT x201 = x185 * (*_x0).Pose.Rot[0];
	const FLT x202 = x181 * (*_x0).Pose.Rot[1];
	const FLT x203 = x201 + (-1 * x202) + x199 + x200;
	const FLT x204 = 2 * x43;
	const FLT x205 = x189 * sensor_pt[1];
	const FLT x206 = x195 * sensor_pt[2];
	const FLT x207 = x206 + (-1 * x203 * sensor_pt[0]) + x205;
	const FLT x208 = 2 * x42;
	const FLT x209 = (x203 * x204) + (-1 * x187 * x188) + (x208 * x207) + (-1 * x197 * x198);
	const FLT x210 = x204 * x189;
	const FLT x211 = 2 * x46;
	const FLT x212 = x187 * sensor_pt[1];
	const FLT x213 = x195 * sensor_pt[0];
	const FLT x214 = x213 + (x203 * sensor_pt[2]) + (-1 * x212);
	const FLT x215 = 2 * x37;
	const FLT x216 = (x214 * x198) + (-1 * x215 * x207) + (-1 * x210) + (x211 * x187);
	const FLT x217 = x189 * x188;
	const FLT x218 = (x215 * x197) + x217 + (-1 * x214 * x208) + (-1 * x211 * x203);
	const FLT x219 = (x218 * (*lh_p).Rot[0]) + (-1 * x209 * (*lh_p).Rot[2]) + (x216 * (*lh_p).Rot[1]);
	const FLT x220 = 2 * (*lh_p).Rot[1];
	const FLT x221 = 2 * ((x218 * (*lh_p).Rot[2]) + (-1 * x216 * (*lh_p).Rot[3]) + (x209 * (*lh_p).Rot[0]));
	const FLT x222 = x216 + (-1 * x219 * x220) + (x221 * (*lh_p).Rot[3]);
	const FLT x223 = 2 * ((-1 * x218 * (*lh_p).Rot[1]) + (x209 * (*lh_p).Rot[3]) + (x216 * (*lh_p).Rot[0]));
	const FLT x224 = 2 * (*lh_p).Rot[2];
	const FLT x225 = x209 + (-1 * x223 * (*lh_p).Rot[3]) + (x219 * x224);
	const FLT x226 = x218 + (-1 * x221 * (*lh_p).Rot[2]) + (x223 * (*lh_p).Rot[1]);
	const FLT x227 = (x81 * x226) + (x76 * x225);
	const FLT x228 = (x86 * x222) + (-1 * x85 * (x227 + (x70 * x222)));
	const FLT x229 = (-1 * x91 * x222) + (x227 * x112);
	const FLT x230 = ((-1 * x226 * x117) + (x225 * x116)) * x118;
	const FLT x231 = (-1 * x230) + (x229 * x115);
	const FLT x232 = x228 * x129;
	const FLT x233 = (x228 * x132) + (x95 * ((-1 * x228 * x174) + x232));
	const FLT x234 = (x228 * x134) + (x95 * x233);
	const FLT x235 =
		x230 +
		(-1 * x140 *
		 (x229 + (x234 * x138) + (-1 * x231 * x124) + (x228 * x165) +
		  (-1 * x137 *
		   ((x108 * ((x228 * x169) + (x95 * x234) + (x228 * x127) +
					 (x95 * (x234 + (x228 * x170) +
							 (x95 * (x233 + (x95 * (x232 + (x228 * x173) + (-1 * x228 * x172))) + (x228 * x131))))))) +
			(-1 * x231 * x126)))));
	const FLT x236 = x211 * x189;
	const FLT x237 = (-1 * x201) + x202 + (-1 * x199) + (-1 * x200);
	const FLT x238 = (-1 * x192) + x193 + x191 + x194;
	const FLT x239 = x196 + (-1 * x238 * sensor_pt[1]) + x190;
	const FLT x240 = x237 * sensor_pt[2];
	const FLT x241 = (-1 * x240) + (x238 * sensor_pt[0]) + x212;
	const FLT x242 = (x215 * x241) + (-1 * x239 * x208) + (-1 * x236) + (x237 * x188);
	const FLT x243 = x189 * sensor_pt[0];
	const FLT x244 = x237 * sensor_pt[1];
	const FLT x245 = (x187 * sensor_pt[2]) + (-1 * x243) + x244;
	const FLT x246 = (-1 * x215 * x245) + (x239 * x198) + (-1 * x237 * x204) + (x211 * x238);
	const FLT x247 = (x208 * x245) + x210 + (-1 * x238 * x188) + (-1 * x241 * x198);
	const FLT x248 = (x247 * (*lh_p).Rot[3]) + (-1 * x242 * (*lh_p).Rot[1]) + (x246 * (*lh_p).Rot[0]);
	const FLT x249 = 2 * (*lh_p).Rot[3];
	const FLT x250 = (x242 * (*lh_p).Rot[0]) + (-1 * x247 * (*lh_p).Rot[2]) + (x246 * (*lh_p).Rot[1]);
	const FLT x251 = x247 + (-1 * x248 * x249) + (x250 * x224);
	const FLT x252 = (x242 * (*lh_p).Rot[2]) + (-1 * x246 * (*lh_p).Rot[3]) + (x247 * (*lh_p).Rot[0]);
	const FLT x253 = (-1 * x252 * x224) + x242 + (x220 * x248);
	const FLT x254 = (x81 * x253) + (x76 * x251);
	const FLT x255 = x246 + (-1 * x250 * x220) + (x252 * x249);
	const FLT x256 = (-1 * x91 * x255) + (x254 * x112);
	const FLT x257 = ((-1 * x253 * x117) + (x251 * x116)) * x118;
	const FLT x258 = (-1 * x257) + (x256 * x115);
	const FLT x259 = (x86 * x255) + (-1 * x85 * (x254 + (x70 * x255)));
	const FLT x260 = x259 * x129;
	const FLT x261 = (x259 * x132) + (x95 * ((-1 * x259 * x174) + x260));
	const FLT x262 = (x259 * x134) + (x95 * x261);
	const FLT x263 =
		x257 +
		(-1 * x140 *
		 (x256 + (-1 * x258 * x124) + (x262 * x138) +
		  (-1 * x137 *
		   ((x108 * ((x259 * x169) + (x95 * x262) + (x259 * x127) +
					 (x95 * (x262 + (x259 * x170) +
							 (x95 * (x261 + (x95 * ((x259 * x173) + x260 + (-1 * x259 * x172))) + (x259 * x131))))))) +
			(-1 * x258 * x126))) +
		  (x259 * x165)));
	const FLT x264 = (x237 * sensor_pt[0]) + (-1 * x205) + x206;
	const FLT x265 = x182 + x179 + (-1 * x184) + x186;
	const FLT x266 = x243 + (-1 * x265 * sensor_pt[2]) + x244;
	const FLT x267 = (x215 * x266) + (x265 * x188) + (-1 * x211 * x195) + (-1 * x208 * x264);
	const FLT x268 = x240 + (-1 * x213) + (x265 * sensor_pt[1]);
	const FLT x269 = (-1 * x215 * x268) + x236 + (-1 * x204 * x265) + (x264 * x198);
	const FLT x270 = (x204 * x195) + (x208 * x268) + (-1 * x217) + (-1 * x266 * x198);
	const FLT x271 = (x270 * (*lh_p).Rot[3]) + (-1 * x267 * (*lh_p).Rot[1]) + (x269 * (*lh_p).Rot[0]);
	const FLT x272 = 2 * ((x267 * (*lh_p).Rot[0]) + (-1 * x270 * (*lh_p).Rot[2]) + (x269 * (*lh_p).Rot[1]));
	const FLT x273 = (-1 * x271 * x249) + x270 + (x272 * (*lh_p).Rot[2]);
	const FLT x274 = (x267 * (*lh_p).Rot[2]) + (-1 * x269 * (*lh_p).Rot[3]) + (x270 * (*lh_p).Rot[0]);
	const FLT x275 = (-1 * x274 * x224) + x267 + (x271 * x220);
	const FLT x276 = (x81 * x275) + (x76 * x273);
	const FLT x277 = (-1 * x272 * (*lh_p).Rot[1]) + x269 + (x274 * x249);
	const FLT x278 = (-1 * x91 * x277) + (x276 * x112);
	const FLT x279 = ((-1 * x275 * x117) + (x273 * x116)) * x118;
	const FLT x280 = (-1 * x279) + (x278 * x115);
	const FLT x281 = (x86 * x277) + (-1 * x85 * (x276 + (x70 * x277)));
	const FLT x282 = x281 * x129;
	const FLT x283 = (x281 * x132) + (x95 * ((-1 * x281 * x174) + x282));
	const FLT x284 = (x281 * x134) + (x95 * x283);
	const FLT x285 =
		x279 +
		(-1 * x140 *
		 ((-1 * x280 * x124) + x278 + (x284 * x138) +
		  (-1 * x137 *
		   ((x108 * ((x95 * x284) + (x281 * x169) +
					 (x95 * (x284 + (x281 * x170) +
							 (x95 * (x283 + (x95 * ((x281 * x173) + (-1 * x281 * x172) + x282)) + (x281 * x131))))) +
					 (x281 * x127))) +
			(-1 * x280 * x126))) +
		  (x281 * x165)));
	const FLT x286 = 2 * x71;
	const FLT x287 = -1 * x286;
	const FLT x288 = 2 * x73;
	const FLT x289 = 1 + (-1 * x288);
	const FLT x290 = x289 + x287;
	const FLT x291 = x224 * (*lh_p).Rot[0];
	const FLT x292 = x249 * (*lh_p).Rot[1];
	const FLT x293 = x292 + (-1 * x291);
	const FLT x294 = (x81 * x293) + (x76 * x290);
	const FLT x295 = x224 * (*lh_p).Rot[1];
	const FLT x296 = x249 * (*lh_p).Rot[0];
	const FLT x297 = x296 + x295;
	const FLT x298 = (-1 * x91 * x297) + (x294 * x112);
	const FLT x299 = ((-1 * x293 * x117) + (x290 * x116)) * x118;
	const FLT x300 = (-1 * x299) + (x298 * x115);
	const FLT x301 = (x86 * x297) + (-1 * x85 * (x294 + (x70 * x297)));
	const FLT x302 = x301 * x129;
	const FLT x303 = (x301 * x132) + (x95 * ((-1 * x301 * x174) + x302));
	const FLT x304 = (x301 * x134) + (x95 * x303);
	const FLT x305 =
		x299 +
		(-1 * x140 *
		 ((x304 * x138) +
		  (-1 * x137 *
		   ((x108 * ((x95 * x304) + (x301 * x169) + (x301 * x127) +
					 (x95 * (x304 + (x301 * x170) +
							 (x95 * ((x95 * ((x301 * x173) + x302 + (-1 * x301 * x172))) + x303 + (x301 * x131))))))) +
			(-1 * x300 * x126))) +
		  x298 + (-1 * x300 * x124) + (x301 * x165)));
	const FLT x306 = x295 + (-1 * x296);
	const FLT x307 = x249 * (*lh_p).Rot[2];
	const FLT x308 = x220 * (*lh_p).Rot[0];
	const FLT x309 = x308 + x307;
	const FLT x310 = (x81 * x309) + (x76 * x306);
	const FLT x311 = 2 * x143;
	const FLT x312 = -1 * x311;
	const FLT x313 = x289 + x312;
	const FLT x314 = (-1 * x91 * x313) + (x310 * x112);
	const FLT x315 = ((-1 * x309 * x117) + (x306 * x116)) * x118;
	const FLT x316 = (-1 * x315) + (x314 * x115);
	const FLT x317 = (x86 * x313) + (-1 * x85 * (x310 + (x70 * x313)));
	const FLT x318 = x317 * x129;
	const FLT x319 = (x317 * x132) + (x95 * ((-1 * x317 * x174) + x318));
	const FLT x320 = (x317 * x134) + (x95 * x319);
	const FLT x321 =
		x315 +
		(-1 * x140 *
		 (x314 + (x320 * x138) + (x317 * x165) + (-1 * x316 * x124) +
		  (-1 * x137 *
		   ((x108 * ((x95 * x320) + (x317 * x169) + (x317 * x127) +
					 (x95 * (x320 + (x317 * x170) +
							 (x95 * (x319 + (x95 * ((x317 * x173) + x318 + (-1 * x317 * x172))) + (x317 * x131))))))) +
			(-1 * x316 * x126)))));
	const FLT x322 = x291 + x292;
	const FLT x323 = 1 + x312 + x287;
	const FLT x324 = (x81 * x323) + (x76 * x322);
	const FLT x325 = x307 + (-1 * x308);
	const FLT x326 = (-1 * x91 * x325) + (x324 * x112);
	const FLT x327 = ((-1 * x323 * x117) + (x322 * x116)) * x118;
	const FLT x328 = (-1 * x327) + (x326 * x115);
	const FLT x329 = (x86 * x325) + (-1 * x85 * (x324 + (x70 * x325)));
	const FLT x330 = x65 * x329;
	const FLT x331 = x97 * x330;
	const FLT x332 = (x98 * x330) + (x95 * ((-1 * x96 * x330) + x331));
	const FLT x333 = (x99 * x330) + (x95 * x332);
	const FLT x334 =
		x327 +
		(-1 * x140 *
		 (x326 + (x330 * x111) + (x333 * x138) +
		  (-1 * x137 *
		   ((x108 * ((x95 * x333) + (x330 * x100) + (x329 * x127) +
					 (x95 * (x333 + (x330 * x104) +
							 (x95 * (x332 + (x95 * (x331 + (x330 * x102) + (-1 * x330 * x128))) + (x329 * x131))))))) +
			(-1 * x328 * x126))) +
		  (-1 * x328 * x124)));
	const FLT x335 = -1 * x30;
	const FLT x336 = dt * dt * dt;
	const FLT x337 = 0.5 * x19 * x336;
	const FLT x338 = x44 * x337;
	const FLT x339 = dt * dt * dt * dt;
	const FLT x340 = (x8 * x8 * x8) * x339;
	const FLT x341 = 1. / (x14 * sqrt(x14));
	const FLT x342 = 1.0 * x21 * x17;
	const FLT x343 = x342 * x341;
	const FLT x344 = x5 * x8;
	const FLT x345 = 2 * x20;
	const FLT x346 = 2 * (1. / (x14 * x14)) * x18;
	const FLT x347 = x339 * x346;
	const FLT x348 = x8 * x347;
	const FLT x349 = x339 * x343;
	const FLT x350 = x8 * x349;
	const FLT x351 = x25 * x342;
	const FLT x352 = (-1 * x12 * x348) + (x12 * x350) + (x344 * x345) + (-1 * x351 * x344) + (-1 * x346 * x340) +
					 (x6 * x350) + (x343 * x340) + (-1 * x6 * x348);
	const FLT x353 = 1.0 / 2.0 * (1. / (x22 * sqrt(x22)));
	const FLT x354 = x26 * x17 * x353;
	const FLT x355 = x352 * x354;
	const FLT x356 = x11 * x355;
	const FLT x357 = x29 * x355;
	const FLT x358 = x336 * x341;
	const FLT x359 = x31 * x24;
	const FLT x360 = x359 * x358;
	const FLT x361 = x0 * x8;
	const FLT x362 = x361 * x360;
	const FLT x363 = x21 * x353;
	const FLT x364 = x363 * x352;
	const FLT x365 = x33 * x337;
	const FLT x366 = x361 * x365;
	const FLT x367 = x0 * x355;
	const FLT x368 = x4 * x24;
	const FLT x369 = 0.5 * x25;
	const FLT x370 = x369 * x344;
	const FLT x371 = x24 * x29;
	const FLT x372 = x9 * x358;
	const FLT x373 = x11 * x358;
	const FLT x374 = x35 * x373;
	const FLT x375 = x38 * x337;
	const FLT x376 = x8 * x11;
	const FLT x377 = (x376 * x375) + (-1 * x8 * x374);
	const FLT x378 = (x372 * x371) + (-1 * x368 * x370) + (-1 * x31 * x367) + x366 + (-1 * x4 * x364) +
					 (-1 * x9 * x338) + (-1 * x34 * x356) + x377 + x335 + (x8 * x357) + (-1 * x362);
	const FLT x379 = x8 * x358;
	const FLT x380 = x0 * x368;
	const FLT x381 = x379 * x380;
	const FLT x382 = x8 * x373 * x371;
	const FLT x383 = x376 * x338;
	const FLT x384 = x41 * x337;
	const FLT x385 = x361 * x384;
	const FLT x386 = x8 * x34;
	const FLT x387 = (x9 * x375) + (-1 * x370 * x359) + (x4 * x367) + (-1 * x386 * x355) + (-1 * x29 * x356) + x381 +
					 (-1 * x382) + x383 + (-1 * x31 * x364) + (-1 * x35 * x372) + (-1 * x385) + x36;
	const FLT x388 = x373 * x368;
	const FLT x389 = x8 * x388;
	const FLT x390 = x376 * x384;
	const FLT x391 = -1 * x39;
	const FLT x392 = x8 * x31;
	const FLT x393 = x0 * x371;
	const FLT x394 = (-1 * x361 * x338) + (x379 * x393);
	const FLT x395 = x394 + x391 + (x0 * x357) + (x392 * x355) + (-1 * x34 * x364) + (x4 * x356) + (x9 * x360) +
					 (-1 * x35 * x370) + (-1 * x9 * x365) + x389 + (-1 * x390);
	const FLT x396 = x11 * x360;
	const FLT x397 = (x8 * x396) + (-1 * x376 * x365);
	const FLT x398 = x0 * x35;
	const FLT x399 = (x375 * x361) + (-1 * x379 * x398);
	const FLT x400 = x399 + x397 + (-1 * x4 * x8 * x355) + (-1 * x34 * x367) + (x9 * x384) + x28 + (-1 * x371 * x370) +
					 (x31 * x356) + (-1 * x372 * x368) + (-1 * x29 * x364);
	const FLT x401 = (x400 * sensor_pt[0]) + (-1 * x387 * sensor_pt[2]) + (x395 * sensor_pt[1]);
	const FLT x402 = (x395 * sensor_pt[0]) + (-1 * x400 * sensor_pt[1]) + (x378 * sensor_pt[2]);
	const FLT x403 = (-1 * x402 * x208) + (x401 * x215) + (-1 * x211 * x378) + (x387 * x188);
	const FLT x404 = (x395 * sensor_pt[2]) + (-1 * x378 * sensor_pt[0]) + (x387 * sensor_pt[1]);
	const FLT x405 = (-1 * x404 * x215) + (-1 * x204 * x387) + (x400 * x211) + (x402 * x198);
	const FLT x406 = (x204 * x378) + (-1 * x401 * x198) + (-1 * x400 * x188) + (x404 * x208);
	const FLT x407 = 2 * ((-1 * x403 * (*lh_p).Rot[1]) + (x406 * (*lh_p).Rot[3]) + (x405 * (*lh_p).Rot[0]));
	const FLT x408 = 2 * ((x403 * (*lh_p).Rot[0]) + (-1 * x406 * (*lh_p).Rot[2]) + (x405 * (*lh_p).Rot[1]));
	const FLT x409 = x406 + (-1 * x407 * (*lh_p).Rot[3]) + (x408 * (*lh_p).Rot[2]);
	const FLT x410 = 2 * ((x403 * (*lh_p).Rot[2]) + (-1 * x405 * (*lh_p).Rot[3]) + (x406 * (*lh_p).Rot[0]));
	const FLT x411 = x403 + (-1 * x410 * (*lh_p).Rot[2]) + (x407 * (*lh_p).Rot[1]);
	const FLT x412 = (x81 * x411) + (x76 * x409);
	const FLT x413 = x405 + (-1 * x408 * (*lh_p).Rot[1]) + (x410 * (*lh_p).Rot[3]);
	const FLT x414 = (-1 * x91 * x413) + (x412 * x112);
	const FLT x415 = ((-1 * x411 * x117) + (x409 * x116)) * x118;
	const FLT x416 = (-1 * x415) + (x414 * x115);
	const FLT x417 = (x86 * x413) + (-1 * x85 * (x412 + (x70 * x413)));
	const FLT x418 = x417 * x129;
	const FLT x419 = (x417 * x132) + (x95 * ((-1 * x417 * x174) + x418));
	const FLT x420 = (x417 * x134) + (x95 * x419);
	const FLT x421 =
		x415 +
		(-1 * x140 *
		 ((x417 * x165) +
		  (-1 * x137 *
		   ((x108 * ((x417 * x127) + (x95 * x420) + (x417 * x169) +
					 (x95 * (x420 + (x417 * x170) +
							 (x95 * (x419 + (x95 * ((-1 * x417 * x172) + (x417 * x173) + x418)) + (x417 * x131))))))) +
			(-1 * x416 * x126))) +
		  x414 + (x420 * x138) + (-1 * x416 * x124)));
	const FLT x422 = x12 * x358;
	const FLT x423 = x9 * x349;
	const FLT x424 = x9 * x347;
	const FLT x425 = x5 * x351;
	const FLT x426 = x6 * x11;
	const FLT x427 = x11 * x11 * x11;
	const FLT x428 = x5 * x345;
	const FLT x429 = (x11 * x428) + (-1 * x427 * x347) + (x427 * x349) + (-1 * x11 * x424) + (x11 * x423) +
					 (-1 * x11 * x425) + (x426 * x349) + (-1 * x426 * x347);
	const FLT x430 = x429 * x354;
	const FLT x431 = x4 * x430;
	const FLT x432 = x429 * x363;
	const FLT x433 = x29 * x354;
	const FLT x434 = x429 * x433;
	const FLT x435 = x5 * x369;
	const FLT x436 = x11 * x435;
	const FLT x437 = x0 * x11;
	const FLT x438 = (-1 * x437 * x384) + (x0 * x388);
	const FLT x439 = x438 + x377 + (-1 * x436 * x359) + (x0 * x431) + x30 + (-1 * x422 * x371) + (-1 * x31 * x432) +
					 (-1 * x430 * x386) + (-1 * x11 * x434) + (x12 * x338);
	const FLT x440 = x31 * x430;
	const FLT x441 = x437 * x365;
	const FLT x442 = x0 * x396;
	const FLT x443 = (-1 * x383) + (-1 * x34 * x11 * x430) + (-1 * x35 * x422) + (-1 * x4 * x432) + x36 + x441 +
					 (x8 * x434) + (-1 * x436 * x368) + (-1 * x442) + (x12 * x375) + (-1 * x0 * x440) + x382;
	const FLT x444 = x373 * x393;
	const FLT x445 = x437 * x338;
	const FLT x446 = -1 * x28;
	const FLT x447 = x397 + x446 + (-1 * x34 * x432) + (-1 * x35 * x436) + x444 + (x430 * x392) + (x422 * x368) +
					 (x11 * x431) + (-1 * x12 * x384) + (x0 * x434) + (-1 * x445);
	const FLT x448 = (x447 * sensor_pt[2]) + (-1 * x443 * sensor_pt[0]) + (x439 * sensor_pt[1]);
	const FLT x449 = x0 * x34;
	const FLT x450 = (x437 * x375) + (-1 * x0 * x374);
	const FLT x451 = (-1 * x389) + (-1 * x8 * x431) + (-1 * x29 * x432) + (x422 * x359) + x390 + x391 +
					 (-1 * x12 * x365) + x450 + (-1 * x436 * x371) + (x11 * x440) + (-1 * x430 * x449);
	const FLT x452 = (x447 * sensor_pt[0]) + (x443 * sensor_pt[2]) + (-1 * x451 * sensor_pt[1]);
	const FLT x453 = (x451 * x211) + (-1 * x439 * x204) + (x452 * x198) + (-1 * x448 * x215);
	const FLT x454 = (x451 * sensor_pt[0]) + (-1 * x439 * sensor_pt[2]) + (x447 * sensor_pt[1]);
	const FLT x455 = (x439 * x188) + (x454 * x215) + (-1 * x443 * x211) + (-1 * x452 * x208);
	const FLT x456 = (x448 * x208) + (-1 * x454 * x198) + (-1 * x451 * x188) + (x443 * x204);
	const FLT x457 = (x456 * (*lh_p).Rot[3]) + (x453 * (*lh_p).Rot[0]) + (-1 * x455 * (*lh_p).Rot[1]);
	const FLT x458 = (x455 * (*lh_p).Rot[0]) + (-1 * x456 * (*lh_p).Rot[2]) + (x453 * (*lh_p).Rot[1]);
	const FLT x459 = x456 + (-1 * x457 * x249) + (x458 * x224);
	const FLT x460 = (x455 * (*lh_p).Rot[2]) + (-1 * x453 * (*lh_p).Rot[3]) + (x456 * (*lh_p).Rot[0]);
	const FLT x461 = x455 + (-1 * x460 * x224) + (x457 * x220);
	const FLT x462 = (x81 * x461) + (x76 * x459);
	const FLT x463 = x453 + (x460 * x249) + (-1 * x458 * x220);
	const FLT x464 = (-1 * x91 * x463) + (x462 * x112);
	const FLT x465 = ((-1 * x461 * x117) + (x459 * x116)) * x118;
	const FLT x466 = (-1 * x465) + (x464 * x115);
	const FLT x467 = (x86 * x463) + (-1 * x85 * (x462 + (x70 * x463)));
	const FLT x468 = x467 * x129;
	const FLT x469 = (x467 * x132) + (x95 * ((-1 * x467 * x174) + x468));
	const FLT x470 = (x467 * x134) + (x95 * x469);
	const FLT x471 =
		x465 +
		(-1 * x140 *
		 (x464 + (x470 * x138) +
		  (-1 * x137 *
		   ((x108 * ((x467 * x169) + (x467 * x127) + (x95 * x470) +
					 (x95 * (x470 + (x467 * x170) +
							 (x95 * ((x95 * ((x467 * x173) + x468 + (-1 * x467 * x172))) + x469 + (x467 * x131))))))) +
			(-1 * x466 * x126))) +
		  (-1 * x466 * x124) + (x467 * x165)));
	const FLT x472 = x0 * x0 * x0;
	const FLT x473 = x0 * x12;
	const FLT x474 = (-1 * x472 * x347) + (x472 * x349) + (x0 * x423) + (x473 * x349) + (-1 * x0 * x424) +
					 (-1 * x0 * x425) + (x0 * x428) + (-1 * x473 * x347);
	const FLT x475 = x474 * x354;
	const FLT x476 = x11 * x475;
	const FLT x477 = x6 * x358;
	const FLT x478 = x474 * x363;
	const FLT x479 = x4 * x475;
	const FLT x480 = (-1 * x8 * x479) + (-1 * x435 * x393) + (x6 * x375) + (x31 * x476) + x385 + (-1 * x475 * x449) +
					 x36 + (-1 * x441) + (-1 * x381) + x442 + (-1 * x29 * x478) + (-1 * x35 * x477);
	const FLT x481 = x31 * x475;
	const FLT x482 = x474 * x433;
	const FLT x483 = x450 + x394 + (-1 * x435 * x380) + (-1 * x6 * x360) + (x8 * x482) + (-1 * x4 * x478) +
					 (x6 * x365) + (-1 * x0 * x481) + x39 + (-1 * x34 * x476);
	const FLT x484 = x399 + (-1 * x31 * x478) + x445 + (x477 * x368) + (-1 * x11 * x482) + x446 + (x0 * x479) +
					 (-1 * x475 * x386) + (-1 * x6 * x384) + (-1 * x444) + (-1 * x0 * x435 * x359);
	const FLT x485 = x362 + (x477 * x371) + x438 + (x8 * x481) + (x0 * x482) + (-1 * x366) + x335 + (-1 * x6 * x338) +
					 (x4 * x476) + (-1 * x435 * x398) + (-1 * x34 * x478);
	const FLT x486 = (-1 * x483 * sensor_pt[0]) + (x485 * sensor_pt[2]) + (x484 * sensor_pt[1]);
	const FLT x487 = (x480 * sensor_pt[0]) + (-1 * x484 * sensor_pt[2]) + (x485 * sensor_pt[1]);
	const FLT x488 = (-1 * x487 * x198) + (x483 * x204) + (-1 * x480 * x188) + (x486 * x208);
	const FLT x489 = (x485 * sensor_pt[0]) + (-1 * x480 * sensor_pt[1]) + (x483 * sensor_pt[2]);
	const FLT x490 = (-1 * x486 * x215) + (x489 * x198) + (-1 * x484 * x204) + (x480 * x211);
	const FLT x491 = (x487 * x215) + (x484 * x188) + (-1 * x483 * x211) + (-1 * x489 * x208);
	const FLT x492 = (-1 * x488 * (*lh_p).Rot[2]) + (x491 * (*lh_p).Rot[0]) + (x490 * (*lh_p).Rot[1]);
	const FLT x493 = (-1 * x490 * (*lh_p).Rot[3]) + (x491 * (*lh_p).Rot[2]) + (x488 * (*lh_p).Rot[0]);
	const FLT x494 = x490 + (-1 * x492 * x220) + (x493 * x249);
	const FLT x495 = (x490 * (*lh_p).Rot[0]) + (x488 * (*lh_p).Rot[3]) + (-1 * x491 * (*lh_p).Rot[1]);
	const FLT x496 = x488 + (-1 * x495 * x249) + (x492 * x224);
	const FLT x497 = x491 + (x495 * x220) + (-1 * x493 * x224);
	const FLT x498 = (x81 * x497) + (x76 * x496);
	const FLT x499 = (x86 * x494) + (-1 * x85 * (x498 + (x70 * x494)));
	const FLT x500 = (-1 * x91 * x494) + (x498 * x112);
	const FLT x501 = ((-1 * x497 * x117) + (x496 * x116)) * x118;
	const FLT x502 = x121 * ((-1 * x501) + (x500 * x115));
	const FLT x503 = x499 * x129;
	const FLT x504 = (x499 * x132) + (x95 * ((-1 * x499 * x174) + x503));
	const FLT x505 = (x499 * x134) + (x95 * x504);
	const FLT x506 =
		x501 +
		(-1 * x140 *
		 (x500 +
		  (-1 * x137 *
		   ((x108 * ((x499 * x169) +
					 (x95 * (x505 + (x499 * x170) +
							 (x95 * (x504 + (x95 * ((-1 * x499 * x172) + (x499 * x173) + x503)) + (x499 * x131))))) +
					 (x95 * x505) + (x499 * x127))) +
			(-1 * x502 * x125))) +
		  (x499 * x165) + (x505 * x138) + (-1 * x502 * x123)));
	const FLT x507 = -1 * dt * x288;
	const FLT x508 = -1 * dt * x286;
	const FLT x509 = x508 + x507 + dt;
	const FLT x510 = dt * x291;
	const FLT x511 = dt * x292;
	const FLT x512 = x511 + (-1 * x510);
	const FLT x513 = (x81 * x512) + (x76 * x509);
	const FLT x514 = dt * x295;
	const FLT x515 = dt * x296;
	const FLT x516 = x515 + x514;
	const FLT x517 = (-1 * x91 * x516) + (x513 * x112);
	const FLT x518 = ((-1 * x512 * x117) + (x509 * x116)) * x118;
	const FLT x519 = (-1 * x518) + (x517 * x115);
	const FLT x520 = (x86 * x516) + (-1 * x85 * (x513 + (x70 * x516)));
	const FLT x521 = x65 * x520;
	const FLT x522 = x520 * x129;
	const FLT x523 = (x520 * x132) + (x95 * ((-1 * x520 * x174) + x522));
	const FLT x524 = (x520 * x134) + (x95 * x523);
	const FLT x525 =
		x518 +
		(-1 * x140 *
		 (x517 + (x524 * x138) + (x521 * x111) + (-1 * x519 * x124) +
		  (-1 * x137 *
		   ((x108 * ((x95 * (x524 + (x520 * x170) +
							 (x95 * (x523 + (x95 * ((x520 * x173) + (-1 * x521 * x128) + x522)) + (x520 * x131))))) +
					 (x95 * x524) + (x520 * x169) + (x520 * x127))) +
			(-1 * x519 * x126)))));
	const FLT x526 = (-1 * dt * x311) + dt;
	const FLT x527 = x526 + x507;
	const FLT x528 = x514 + (-1 * x515);
	const FLT x529 = dt * x307;
	const FLT x530 = dt * x308;
	const FLT x531 = x530 + x529;
	const FLT x532 = (x81 * x531) + (x76 * x528);
	const FLT x533 = (x86 * x527) + (-1 * x85 * (x532 + (x70 * x527)));
	const FLT x534 = (-1 * x91 * x527) + (x532 * x112);
	const FLT x535 = ((-1 * x531 * x117) + (x528 * x116)) * x118;
	const FLT x536 = (-1 * x535) + (x534 * x115);
	const FLT x537 = x533 * x129;
	const FLT x538 = (x533 * x132) + (x95 * ((-1 * x533 * x174) + x537));
	const FLT x539 = (x533 * x134) + (x95 * x538);
	const FLT x540 =
		x535 +
		(-1 * x140 *
		 ((x539 * x138) +
		  (-1 * x137 *
		   ((x108 * ((x95 * x539) + (x533 * x169) + (x533 * x127) +
					 (x95 * (x539 + (x533 * x170) +
							 (x95 * (x538 + (x95 * ((x533 * x173) + x537 + (-1 * x533 * x172))) + (x533 * x131))))))) +
			(-1 * x536 * x126))) +
		  (x533 * x165) + x534 + (-1 * x536 * x124)));
	const FLT x541 = x510 + x511;
	const FLT x542 = x526 + x508;
	const FLT x543 = (x81 * x542) + (x76 * x541);
	const FLT x544 = x529 + (-1 * x530);
	const FLT x545 = (-1 * x91 * x544) + (x543 * x112);
	const FLT x546 = ((-1 * x542 * x117) + (x541 * x116)) * x118;
	const FLT x547 = (-1 * x546) + (x545 * x115);
	const FLT x548 = (x86 * x544) + (-1 * x85 * (x543 + (x70 * x544)));
	const FLT x549 = x65 * x548;
	const FLT x550 = x97 * x549;
	const FLT x551 = (x98 * x549) + (x95 * ((-1 * x96 * x549) + x550));
	const FLT x552 = (x99 * x549) + (x95 * x551);
	const FLT x553 =
		x546 +
		(-1 * x140 *
		 (x545 + (x552 * x138) + (x549 * x111) + (-1 * x547 * x124) +
		  (-1 * x137 *
		   ((x108 * ((x95 * x552) + (x549 * x100) + (x548 * x127) +
					 (x95 * ((x549 * x104) + x552 +
							 (x95 * ((x95 * ((x549 * x102) + x550 + (-1 * x549 * x128))) + x551 + (x548 * x131))))))) +
			(-1 * x547 * x126)))));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Acc[0]) / sizeof(FLT), x141 + (x142 * x141));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Acc[1]) / sizeof(FLT), x159 + (x142 * x159));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Acc[2]) / sizeof(FLT), x177 + (x177 * x142));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						x235 + (x235 * x142));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						x263 + (x263 * x142));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						x285 + (x285 * x142));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT), x305 + (x305 * x142));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT), x321 + (x321 * x142));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT), x334 + (x334 * x142));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x421 + (x421 * x142));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x471 + (x471 * x142));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x506 + (x506 * x142));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.Pos[0]) / sizeof(FLT), x525 + (x525 * x142));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.Pos[1]) / sizeof(FLT), x540 + (x540 * x142));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurviveKalmanErrorModel, Velocity.Pos[2]) / sizeof(FLT), x553 + (x553 * x142));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen2 wrt [(*error_model).AccBias[0],
// (*error_model).AccBias[1], (*error_model).AccBias[2], (*error_model).Acc[0], (*error_model).Acc[1],
// (*error_model).Acc[2], (*error_model).GyroBias[0], (*error_model).GyroBias[1], (*error_model).GyroBias[2],
// (*error_model).IMUCorrection[0], (*error_model).IMUCorrection[1], (*error_model).IMUCorrection[2],
// (*error_model).IMUCorrection[3], (*error_model).Pose.AxisAngleRot[0], (*error_model).Pose.AxisAngleRot[1],
// (*error_model).Pose.AxisAngleRot[2], (*error_model).Pose.Pos[0], (*error_model).Pose.Pos[1],
// (*error_model).Pose.Pos[2], (*error_model).Velocity.AxisAngleRot[0], (*error_model).Velocity.AxisAngleRot[1],
// (*error_model).Velocity.AxisAngleRot[2], (*error_model).Velocity.Pos[0], (*error_model).Velocity.Pos[1],
// (*error_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f88f4f162b0>]

static inline void SurviveKalmanErrorModel_LightMeas_y_gen2_jac_error_model_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_y_gen2(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_y_gen2_jac_error_model(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen2 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]
static inline void SurviveKalmanErrorModel_LightMeas_y_gen2_jac_sensor_pt(CnMat *Hx, const FLT dt,
																		  const SurviveKalmanModel *_x0,
																		  const SurviveKalmanErrorModel *error_model,
																		  const FLT *sensor_pt, const SurvivePose *lh_p,
																		  const BaseStationCal *bsc0) {
	const FLT x0 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x1 = 0.5 * (*_x0).Pose.Rot[3];
	const FLT x2 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x4 = (-1 * x3 * (*_x0).Pose.Rot[1]) + (x1 * (*error_model).Pose.AxisAngleRot[0]) +
				   (x2 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2];
	const FLT x5 = dt * dt;
	const FLT x6 = (x0 * x0) * x5;
	const FLT x7 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x8 = x5 * (x7 * x7);
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x10 = x5 * (x9 * x9);
	const FLT x11 = 1e-10 + x10 + x6 + x8;
	const FLT x12 = sqrt(x11);
	const FLT x13 = 0.5 * x12;
	const FLT x14 = sin(x13);
	const FLT x15 = (x14 * x14) * (1. / x11);
	const FLT x16 = cos(x13);
	const FLT x17 = 1. / sqrt((x8 * x15) + (x16 * x16) + (x6 * x15) + (x15 * x10));
	const FLT x18 = dt * x14 * (1. / x12) * x17;
	const FLT x19 = x4 * x18;
	const FLT x20 = x0 * x19;
	const FLT x21 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x22 =
		(x2 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x21 * (*_x0).Pose.Rot[2]) + (x3 * (*_x0).Pose.Rot[0]);
	const FLT x23 = x22 * x18;
	const FLT x24 = x9 * x23;
	const FLT x25 = (*_x0).Pose.Rot[1] + (-1 * x1 * (*error_model).Pose.AxisAngleRot[1]) + (x21 * (*_x0).Pose.Rot[0]) +
					(x3 * (*_x0).Pose.Rot[2]);
	const FLT x26 = x17 * x16;
	const FLT x27 = x25 * x26;
	const FLT x28 = (-1 * x21 * (*_x0).Pose.Rot[1]) + (-1 * x2 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x1 * (*error_model).Pose.AxisAngleRot[2]);
	const FLT x29 = x28 * x18;
	const FLT x30 = x7 * x29;
	const FLT x31 = (-1 * x20) + x30 + x27 + x24;
	const FLT x32 = x25 * x18;
	const FLT x33 = (-1 * x7 * x32) + (-1 * x0 * x23) + (-1 * x9 * x19) + (x28 * x26);
	const FLT x34 = x0 * x32;
	const FLT x35 = x9 * x29;
	const FLT x36 = x4 * x26;
	const FLT x37 = x7 * x23;
	const FLT x38 = (-1 * x37) + x36 + x34 + x35;
	const FLT x39 = (x31 * sensor_pt[1]) + (-1 * x38 * sensor_pt[0]) + (x33 * sensor_pt[2]);
	const FLT x40 = x9 * x32;
	const FLT x41 = x0 * x29;
	const FLT x42 = x22 * x26;
	const FLT x43 = x7 * x19;
	const FLT x44 = x43 + x42 + (-1 * x40) + x41;
	const FLT x45 = (-1 * x44 * sensor_pt[1]) + (x38 * sensor_pt[2]) + (x33 * sensor_pt[0]);
	const FLT x46 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x47 = (*_x0).Pose.Pos[1] + (2 * ((x44 * x45) + (-1 * x31 * x39))) + (*error_model).Pose.Pos[1] +
					(x46 * ((*_x0).Acc[1] + (*error_model).Acc[1])) + sensor_pt[1] +
					(dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1]));
	const FLT x48 = (-1 * x31 * sensor_pt[2]) + (x33 * sensor_pt[1]) + (x44 * sensor_pt[0]);
	const FLT x49 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (2 * ((x48 * x31) + (-1 * x45 * x38))) +
					(*error_model).Pose.Pos[2] + (dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2])) +
					(x46 * ((*_x0).Acc[2] + (*error_model).Acc[2]));
	const FLT x50 = (x46 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + (2 * ((x38 * x39) + (-1 * x44 * x48))) +
					sensor_pt[0] + (dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) +
					(*error_model).Pose.Pos[0] + (*_x0).Pose.Pos[0];
	const FLT x51 = (-1 * x50 * (*lh_p).Rot[2]) + (x47 * (*lh_p).Rot[1]) + (x49 * (*lh_p).Rot[0]);
	const FLT x52 = (-1 * x47 * (*lh_p).Rot[3]) + (x50 * (*lh_p).Rot[0]) + (x49 * (*lh_p).Rot[2]);
	const FLT x53 = x47 + (*lh_p).Pos[1] + (2 * ((x52 * (*lh_p).Rot[3]) + (-1 * x51 * (*lh_p).Rot[1])));
	const FLT x54 = x53 * x53;
	const FLT x55 = (-1 * x49 * (*lh_p).Rot[1]) + (x47 * (*lh_p).Rot[0]) + (x50 * (*lh_p).Rot[3]);
	const FLT x56 = x49 + (2 * ((x55 * (*lh_p).Rot[1]) + (-1 * x52 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x57 = x50 + (*lh_p).Pos[0] + (2 * ((x51 * (*lh_p).Rot[2]) + (-1 * x55 * (*lh_p).Rot[3])));
	const FLT x58 = x57 * x57;
	const FLT x59 = x58 + (x56 * x56);
	const FLT x60 = x59 + x54;
	const FLT x61 = 0.523598775598299 + (-1 * (*bsc0).tilt);
	const FLT x62 = cos(x61);
	const FLT x63 = 1. / sqrt(1 + (-1 * (1. / x60) * (1. / (x62 * x62)) * x54));
	const FLT x64 = (-1 * x36) + x37 + (-1 * x35) + (-1 * x34);
	const FLT x65 = 2 * x31;
	const FLT x66 = 2 * x33;
	const FLT x67 = x66 * x44;
	const FLT x68 = x67 + (-1 * x64 * x65);
	const FLT x69 = 2 * x38;
	const FLT x70 = 1 + (x64 * x69) + (-2 * (x44 * x44));
	const FLT x71 = x69 * x33;
	const FLT x72 = 2 * x44;
	const FLT x73 = (x72 * x31) + (-1 * x71);
	const FLT x74 = (x73 * (*lh_p).Rot[2]) + (-1 * x68 * (*lh_p).Rot[3]) + (x70 * (*lh_p).Rot[0]);
	const FLT x75 = 2 * (*lh_p).Rot[3];
	const FLT x76 = (-1 * x70 * (*lh_p).Rot[2]) + (x73 * (*lh_p).Rot[0]) + (x68 * (*lh_p).Rot[1]);
	const FLT x77 = 2 * (*lh_p).Rot[1];
	const FLT x78 = x68 + (x75 * x74) + (-1 * x77 * x76);
	const FLT x79 = 2 * x53;
	const FLT x80 = 2 * (*lh_p).Rot[2];
	const FLT x81 = (x70 * (*lh_p).Rot[3]) + (-1 * x73 * (*lh_p).Rot[1]) + (x68 * (*lh_p).Rot[0]);
	const FLT x82 = (x80 * x76) + x70 + (-1 * x81 * x75);
	const FLT x83 = 2 * x57;
	const FLT x84 = x73 + (x81 * x77) + (-1 * x80 * x74);
	const FLT x85 = 2 * x56;
	const FLT x86 = (x84 * x85) + (x82 * x83);
	const FLT x87 = 1. / x62;
	const FLT x88 = 1.0 / 2.0 * x53;
	const FLT x89 = x88 * x87 * (1. / (x60 * sqrt(x60)));
	const FLT x90 = x87 * (1. / sqrt(x60));
	const FLT x91 = x63 * ((x78 * x90) + (-1 * x89 * (x86 + (x79 * x78))));
	const FLT x92 = atan2(-1 * x56, x57);
	const FLT x93 = tan(x61);
	const FLT x94 = (1. / sqrt(x59)) * x93;
	const FLT x95 = -1 * x53 * x94;
	const FLT x96 = asin(x95) + (-1 * x92) + (-1 * (*bsc0).ogeephase);
	const FLT x97 = (-1 * sin(x96) * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x98 = asin(x53 * x90);
	const FLT x99 = 8.0108022e-06 * x98;
	const FLT x100 = -8.0108022e-06 + (-1 * x99);
	const FLT x101 = 0.0028679863 + (x98 * x100);
	const FLT x102 = 5.3685255e-06 + (x98 * x101);
	const FLT x103 = 0.0076069798 + (x98 * x102);
	const FLT x104 = x98 * x103;
	const FLT x105 = -8.0108022e-06 + (-1.60216044e-05 * x98);
	const FLT x106 = x101 + (x98 * x105);
	const FLT x107 = x102 + (x98 * x106);
	const FLT x108 = x103 + (x98 * x107);
	const FLT x109 = (x98 * x108) + x104;
	const FLT x110 = sin(x61);
	const FLT x111 = x97 * x110;
	const FLT x112 = x62 + (x109 * x111);
	const FLT x113 = 1. / x112;
	const FLT x114 = 2 * x97 * x104 * x113;
	const FLT x115 = x88 * (1. / (x59 * sqrt(x59))) * x93;
	const FLT x116 = (-1 * x78 * x94) + (x86 * x115);
	const FLT x117 = 1. / x59;
	const FLT x118 = 1. / sqrt(1 + (-1 * x54 * (x93 * x93) * x117));
	const FLT x119 = (1. / x58) * x56;
	const FLT x120 = 1. / x57;
	const FLT x121 = x58 * x117;
	const FLT x122 = ((-1 * x84 * x120) + (x82 * x119)) * x121;
	const FLT x123 = cos(x96) * (*bsc0).ogeemag;
	const FLT x124 = x123 * ((-1 * x122) + (x118 * x116));
	const FLT x125 = x109 * x110;
	const FLT x126 = x91 * x100;
	const FLT x127 = 2.40324066e-05 * x98;
	const FLT x128 = (x91 * x101) + (x98 * ((-1 * x91 * x99) + x126));
	const FLT x129 = (x91 * x102) + (x98 * x128);
	const FLT x130 = x98 * x98;
	const FLT x131 = x97 * x130;
	const FLT x132 = x103 * (1. / (x112 * x112)) * x131;
	const FLT x133 = x103 * x113 * x130;
	const FLT x134 = x113 * x131;
	const FLT x135 = x95 + (x103 * x134);
	const FLT x136 = 1. / sqrt(1 + (-1 * (x135 * x135)));
	const FLT x137 =
		x122 +
		(-1 * x136 *
		 (x116 + (-1 * x124 * x133) + (x91 * x114) + (x129 * x134) +
		  (-1 * x132 *
		   ((x111 * ((x91 * x103) + (x98 * x129) + (x91 * x108) +
					 (x98 * ((x91 * x107) + x129 +
							 (x98 * (x128 + (x98 * ((x91 * x105) + x126 + (-1 * x91 * x127))) + (x91 * x106))))))) +
			(-1 * x124 * x125)))));
	const FLT x138 = cos((-1 * asin(x135)) + (*bsc0).gibpha + x92) * (*bsc0).gibmag;
	const FLT x139 = x40 + (-1 * x43) + (-1 * x41) + (-1 * x42);
	const FLT x140 = 1 + (x72 * x139) + (-2 * (x31 * x31));
	const FLT x141 = (x69 * x31) + (-1 * x67);
	const FLT x142 = x66 * x31;
	const FLT x143 = x142 + (-1 * x69 * x139);
	const FLT x144 = (x143 * (*lh_p).Rot[2]) + (-1 * x140 * (*lh_p).Rot[3]) + (x141 * (*lh_p).Rot[0]);
	const FLT x145 = (x143 * (*lh_p).Rot[0]) + (-1 * x141 * (*lh_p).Rot[2]) + (x140 * (*lh_p).Rot[1]);
	const FLT x146 = x140 + (x75 * x144) + (-1 * x77 * x145);
	const FLT x147 = (x141 * (*lh_p).Rot[3]) + (x140 * (*lh_p).Rot[0]) + (-1 * x143 * (*lh_p).Rot[1]);
	const FLT x148 = x141 + (-1 * x75 * x147) + (x80 * x145);
	const FLT x149 = x143 + (x77 * x147) + (-1 * x80 * x144);
	const FLT x150 = (x85 * x149) + (x83 * x148);
	const FLT x151 = x63 * ((x90 * x146) + (-1 * x89 * (x150 + (x79 * x146))));
	const FLT x152 = (-1 * x94 * x146) + (x115 * x150);
	const FLT x153 = ((-1 * x120 * x149) + (x119 * x148)) * x121;
	const FLT x154 = (-1 * x153) + (x118 * x152);
	const FLT x155 = x123 * x133;
	const FLT x156 = x123 * x125;
	const FLT x157 = x100 * x151;
	const FLT x158 = (x101 * x151) + (x98 * ((-1 * x99 * x151) + x157));
	const FLT x159 = (x102 * x151) + (x98 * x158);
	const FLT x160 =
		x153 +
		(-1 * x136 *
		 ((x134 * x159) + x152 +
		  (-1 * x132 *
		   ((x111 * ((x98 * x159) +
					 (x98 * (x159 + (x107 * x151) +
							 (x98 * (x158 + (x98 * ((x105 * x151) + x157 + (-1 * x127 * x151))) + (x106 * x151))))) +
					 (x108 * x151) + (x103 * x151))) +
			(-1 * x154 * x156))) +
		  (x114 * x151) + (-1 * x154 * x155)));
	const FLT x161 = (x69 * x44) + (-1 * x142);
	const FLT x162 = (-1 * x27) + (-1 * x30) + (-1 * x24) + x20;
	const FLT x163 = x71 + (-1 * x72 * x162);
	const FLT x164 = 1 + (x65 * x162) + (-2 * (x38 * x38));
	const FLT x165 = (x164 * (*lh_p).Rot[2]) + (-1 * x161 * (*lh_p).Rot[3]) + (x163 * (*lh_p).Rot[0]);
	const FLT x166 = (x164 * (*lh_p).Rot[0]) + (x161 * (*lh_p).Rot[1]) + (-1 * x163 * (*lh_p).Rot[2]);
	const FLT x167 = (x75 * x165) + x161 + (-1 * x77 * x166);
	const FLT x168 = (x163 * (*lh_p).Rot[3]) + (x161 * (*lh_p).Rot[0]) + (-1 * x164 * (*lh_p).Rot[1]);
	const FLT x169 = x163 + (-1 * x75 * x168) + (x80 * x166);
	const FLT x170 = x164 + (x77 * x168) + (-1 * x80 * x165);
	const FLT x171 = (x85 * x170) + (x83 * x169);
	const FLT x172 = x63 * ((x90 * x167) + (-1 * x89 * (x171 + (x79 * x167))));
	const FLT x173 = (-1 * x94 * x167) + (x115 * x171);
	const FLT x174 = ((-1 * x120 * x170) + (x119 * x169)) * x121;
	const FLT x175 = (-1 * x174) + (x118 * x173);
	const FLT x176 = x100 * x172;
	const FLT x177 = (x101 * x172) + (x98 * ((-1 * x99 * x172) + x176));
	const FLT x178 = (x102 * x172) + (x98 * x177);
	const FLT x179 =
		x174 +
		(-1 * x136 *
		 ((x178 * x134) + (-1 * x175 * x155) + (x114 * x172) + x173 +
		  (-1 * x132 *
		   ((x111 * ((x98 * x178) + (x103 * x172) + (x108 * x172) +
					 (x98 * (x178 + (x107 * x172) +
							 (x98 * (x177 + (x98 * ((x105 * x172) + x176 + (-1 * x127 * x172))) + (x106 * x172))))))) +
			(-1 * x175 * x156)))));
	cnMatrixOptionalSet(Hx, 0, 0, x137 + (x137 * x138));
	cnMatrixOptionalSet(Hx, 0, 1, x160 + (x160 * x138));
	cnMatrixOptionalSet(Hx, 0, 2, x179 + (x179 * x138));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen2 wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]

static inline void SurviveKalmanErrorModel_LightMeas_y_gen2_jac_sensor_pt_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_y_gen2(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_y_gen2_jac_sensor_pt(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen2 wrt [(*lh_p).Pos[0], (*lh_p).Pos[1], (*lh_p).Pos[2],
// (*lh_p).Rot[0], (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]
static inline void SurviveKalmanErrorModel_LightMeas_y_gen2_jac_lh_p(CnMat *Hx, const FLT dt,
																	 const SurviveKalmanModel *_x0,
																	 const SurviveKalmanErrorModel *error_model,
																	 const FLT *sensor_pt, const SurvivePose *lh_p,
																	 const BaseStationCal *bsc0) {
	const FLT x0 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x1 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x2 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x4 =
		(-1 * x3 * (*_x0).Pose.Rot[1]) + (x2 * (*_x0).Pose.Rot[0]) + (x1 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x5 = dt * dt;
	const FLT x6 = (x0 * x0) * x5;
	const FLT x7 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x8 = x5 * (x7 * x7);
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x10 = x5 * (x9 * x9);
	const FLT x11 = 1e-10 + x10 + x6 + x8;
	const FLT x12 = sqrt(x11);
	const FLT x13 = 0.5 * x12;
	const FLT x14 = sin(x13);
	const FLT x15 = (x14 * x14) * (1. / x11);
	const FLT x16 = cos(x13);
	const FLT x17 = 1. / sqrt((x8 * x15) + (x16 * x16) + (x6 * x15) + (x15 * x10));
	const FLT x18 = dt * x14 * (1. / x12) * x17;
	const FLT x19 = x4 * x18;
	const FLT x20 =
		(x2 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x1 * (*_x0).Pose.Rot[2]) + (x3 * (*_x0).Pose.Rot[0]);
	const FLT x21 = x9 * x18;
	const FLT x22 =
		(*_x0).Pose.Rot[1] + (x1 * (*_x0).Pose.Rot[0]) + (-1 * x2 * (*_x0).Pose.Rot[3]) + (x3 * (*_x0).Pose.Rot[2]);
	const FLT x23 = x17 * x16;
	const FLT x24 = (-1 * x1 * (*_x0).Pose.Rot[1]) + (-1 * x2 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x3 * (*_x0).Pose.Rot[3]);
	const FLT x25 = x24 * x18;
	const FLT x26 = (x7 * x25) + (x22 * x23) + (-1 * x0 * x19) + (x20 * x21);
	const FLT x27 = x20 * x18;
	const FLT x28 = x22 * x18;
	const FLT x29 = (-1 * x0 * x27) + (-1 * x7 * x28) + (-1 * x4 * x21) + (x24 * x23);
	const FLT x30 = (-1 * x7 * x27) + (x4 * x23) + (x0 * x28) + (x24 * x21);
	const FLT x31 = (x26 * sensor_pt[1]) + (-1 * x30 * sensor_pt[0]) + (x29 * sensor_pt[2]);
	const FLT x32 = (x7 * x19) + (x23 * x20) + (-1 * x22 * x21) + (x0 * x25);
	const FLT x33 = (-1 * x32 * sensor_pt[1]) + (x30 * sensor_pt[2]) + (x29 * sensor_pt[0]);
	const FLT x34 = 2 * ((x32 * x33) + (-1 * x31 * x26));
	const FLT x35 = dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1]);
	const FLT x36 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x37 = x36 * ((*_x0).Acc[1] + (*error_model).Acc[1]);
	const FLT x38 = x34 + (*error_model).Pose.Pos[1] + (*_x0).Pose.Pos[1] + x35 + sensor_pt[1] + x37;
	const FLT x39 = x38 * (*lh_p).Rot[0];
	const FLT x40 = (-1 * x26 * sensor_pt[2]) + (x29 * sensor_pt[1]) + (x32 * sensor_pt[0]);
	const FLT x41 = 2 * ((x30 * x31) + (-1 * x40 * x32));
	const FLT x42 = dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0]);
	const FLT x43 = x36 * ((*_x0).Acc[0] + (*error_model).Acc[0]);
	const FLT x44 = x43 + x41 + (*error_model).Pose.Pos[0] + sensor_pt[0] + x42 + (*_x0).Pose.Pos[0];
	const FLT x45 = x44 * (*lh_p).Rot[3];
	const FLT x46 = 2 * ((x40 * x26) + (-1 * x30 * x33));
	const FLT x47 = x36 * ((*_x0).Acc[2] + (*error_model).Acc[2]);
	const FLT x48 = dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2]);
	const FLT x49 = sensor_pt[2] + (*_x0).Pose.Pos[2] + x47 + x46 + (*error_model).Pose.Pos[2] + x48;
	const FLT x50 = x49 * (*lh_p).Rot[1];
	const FLT x51 = x39 + (-1 * x50) + x45;
	const FLT x52 = x38 * (*lh_p).Rot[1];
	const FLT x53 = x49 * (*lh_p).Rot[0];
	const FLT x54 = x44 * (*lh_p).Rot[2];
	const FLT x55 = (-1 * x54) + x52 + x53;
	const FLT x56 = x44 + (*lh_p).Pos[0] + (2 * ((x55 * (*lh_p).Rot[2]) + (-1 * x51 * (*lh_p).Rot[3])));
	const FLT x57 = x44 * (*lh_p).Rot[0];
	const FLT x58 = x49 * (*lh_p).Rot[2];
	const FLT x59 = x38 * (*lh_p).Rot[3];
	const FLT x60 = (-1 * x59) + x57 + x58;
	const FLT x61 = (*lh_p).Pos[1] + x38 + (2 * ((x60 * (*lh_p).Rot[3]) + (-1 * x55 * (*lh_p).Rot[1])));
	const FLT x62 = 0.523598775598299 + (-1 * (*bsc0).tilt);
	const FLT x63 = tan(x62);
	const FLT x64 = x49 + (2 * ((x51 * (*lh_p).Rot[1]) + (-1 * x60 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x65 = x56 * x56;
	const FLT x66 = x65 + (x64 * x64);
	const FLT x67 = x63 * (1. / (x66 * sqrt(x66)));
	const FLT x68 = x61 * x67;
	const FLT x69 = x68 * x56;
	const FLT x70 = x61 * x61;
	const FLT x71 = x66 + x70;
	const FLT x72 = cos(x62);
	const FLT x73 = 1. / x72;
	const FLT x74 = (1. / sqrt(x71)) * x73;
	const FLT x75 = asin(x74 * x61);
	const FLT x76 = 8.0108022e-06 * x75;
	const FLT x77 = -8.0108022e-06 + (-1 * x76);
	const FLT x78 = 1. / sqrt(1 + (-1 * (1. / x71) * x70 * (1. / (x72 * x72))));
	const FLT x79 = (1. / (x71 * sqrt(x71))) * x73;
	const FLT x80 = x79 * x61;
	const FLT x81 = x80 * x56;
	const FLT x82 = x81 * x78;
	const FLT x83 = -1 * x82 * x77;
	const FLT x84 = 0.0028679863 + (x75 * x77);
	const FLT x85 = (-1 * x82 * x84) + (x75 * ((x82 * x76) + x83));
	const FLT x86 = 5.3685255e-06 + (x84 * x75);
	const FLT x87 = x86 * x78;
	const FLT x88 = (-1 * x81 * x87) + (x85 * x75);
	const FLT x89 = x75 * x75;
	const FLT x90 = atan2(-1 * x64, x56);
	const FLT x91 = x63 * (1. / sqrt(x66));
	const FLT x92 = -1 * x61 * x91;
	const FLT x93 = asin(x92) + (-1 * x90) + (-1 * (*bsc0).ogeephase);
	const FLT x94 = (-1 * sin(x93) * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x95 = 0.0076069798 + (x86 * x75);
	const FLT x96 = x75 * x95;
	const FLT x97 = -8.0108022e-06 + (-1.60216044e-05 * x75);
	const FLT x98 = x84 + (x75 * x97);
	const FLT x99 = x86 + (x75 * x98);
	const FLT x100 = x95 + (x75 * x99);
	const FLT x101 = (x75 * x100) + x96;
	const FLT x102 = sin(x62);
	const FLT x103 = x94 * x102;
	const FLT x104 = x72 + (x101 * x103);
	const FLT x105 = 1. / x104;
	const FLT x106 = x94 * x105;
	const FLT x107 = x89 * x106;
	const FLT x108 = 1. / x66;
	const FLT x109 = 1. / sqrt(1 + (-1 * x70 * (x63 * x63) * x108));
	const FLT x110 = x64 * x108;
	const FLT x111 = cos(x93) * (*bsc0).ogeemag;
	const FLT x112 = x111 * ((-1 * x110) + (x69 * x109));
	const FLT x113 = x101 * x102;
	const FLT x114 = x78 * x99;
	const FLT x115 = 2.40324066e-05 * x75;
	const FLT x116 = x78 * x95;
	const FLT x117 = x89 * x95;
	const FLT x118 = x94 * (1. / (x104 * x104)) * x117;
	const FLT x119 = x105 * x117;
	const FLT x120 = 2 * x56;
	const FLT x121 = x96 * x106;
	const FLT x122 = x80 * x78 * x121;
	const FLT x123 = x92 + (x95 * x107);
	const FLT x124 = 1. / sqrt(1 + (-1 * (x123 * x123)));
	const FLT x125 =
		x110 +
		(-1 * x124 *
		 ((-1 * x112 * x119) +
		  (-1 * x118 *
		   ((x103 * ((x88 * x75) + (-1 * x81 * x116) +
					 (x75 * (x88 + (-1 * x81 * x114) +
							 (x75 * (x85 + (x75 * (x83 + (-1 * x82 * x97) + (x82 * x115))) + (-1 * x82 * x98))))) +
					 (-1 * x82 * x100))) +
			(-1 * x112 * x113))) +
		  x69 + (-1 * x120 * x122) + (x88 * x107)));
	const FLT x126 = cos((-1 * asin(x123)) + (*bsc0).gibpha + x90) * (*bsc0).gibmag;
	const FLT x127 = x74 + (-1 * x70 * x79);
	const FLT x128 = x78 * x127;
	const FLT x129 = x77 * x128;
	const FLT x130 = (x84 * x128) + (x75 * ((-1 * x76 * x128) + x129));
	const FLT x131 = (x86 * x128) + (x75 * x130);
	const FLT x132 = 2 * x121;
	const FLT x133 = x111 * x119;
	const FLT x134 = x91 * x109;
	const FLT x135 = x111 * x113;
	const FLT x136 =
		x124 *
		((-1 * x118 *
		  ((x103 * ((x75 * x131) + (x116 * x127) + (x100 * x128) +
					(x75 * (x131 + (x99 * x128) +
							(x75 * (x130 + (x75 * ((x97 * x128) + (-1 * x115 * x128) + x129)) + (x98 * x128))))))) +
		   (x134 * x135))) +
		 (x133 * x134) + (x128 * x132) + (-1 * x91) + (x107 * x131));
	const FLT x137 = x64 * x61;
	const FLT x138 = x67 * x137;
	const FLT x139 = x79 * x137;
	const FLT x140 = x78 * x139;
	const FLT x141 = -1 * x77 * x140;
	const FLT x142 = (-1 * x84 * x140) + (x75 * ((x76 * x140) + x141));
	const FLT x143 = (-1 * x87 * x139) + (x75 * x142);
	const FLT x144 = 2 * x64;
	const FLT x145 = x56 * x108;
	const FLT x146 = x145 + (x109 * x138);
	const FLT x147 =
		(-1 * x145) +
		(-1 * x124 *
		 ((-1 * x118 *
		   ((x103 * ((x75 * x143) +
					 (x75 * (x143 + (-1 * x114 * x139) +
							 (x75 * (x142 + (x75 * ((-1 * x97 * x140) + x141 + (x115 * x140))) + (-1 * x98 * x140))))) +
					 (-1 * x100 * x140) + (-1 * x116 * x139))) +
			(-1 * x135 * x146))) +
		  (-1 * x122 * x144) + (-1 * x133 * x146) + x138 + (x107 * x143)));
	const FLT x148 = 2 * x50;
	const FLT x149 = (2 * x45) + (-1 * x148);
	const FLT x150 = 2 * x61;
	const FLT x151 = 2 * x59;
	const FLT x152 = (2 * x58) + (-1 * x151);
	const FLT x153 = 2 * x54;
	const FLT x154 = (2 * x52) + (-1 * x153);
	const FLT x155 = (x144 * x154) + (x120 * x152);
	const FLT x156 = 1.0 / 2.0 * x80;
	const FLT x157 = (x74 * x149) + (-1 * x156 * (x155 + (x149 * x150)));
	const FLT x158 = x78 * x157;
	const FLT x159 = 1.0 / 2.0 * x68;
	const FLT x160 = (-1 * x91 * x149) + (x155 * x159);
	const FLT x161 = x64 * (1. / x65);
	const FLT x162 = 1. / x56;
	const FLT x163 = x65 * x108;
	const FLT x164 = ((-1 * x162 * x154) + (x161 * x152)) * x163;
	const FLT x165 = (-1 * x164) + (x109 * x160);
	const FLT x166 = x77 * x158;
	const FLT x167 = (x84 * x158) + (x75 * ((-1 * x76 * x158) + x166));
	const FLT x168 = (x86 * x158) + (x75 * x167);
	const FLT x169 =
		x164 +
		(-1 * x124 *
		 (x160 + (x107 * x168) +
		  (-1 * x118 *
		   ((x103 * ((x75 * x168) + (x116 * x157) + (x100 * x158) +
					 (x75 * (x168 + (x99 * x158) +
							 (x75 * (x167 + (x75 * ((x97 * x158) + x166 + (-1 * x115 * x158))) + (x98 * x158))))))) +
			(-1 * x165 * x135))) +
		  (x132 * x158) + (-1 * x165 * x133)));
	const FLT x170 = (-1 * sensor_pt[2]) + (-1 * x48) + (-1 * x46) + (-1 * x47) + (-1 * (*_x0).Pose.Pos[2]) +
					 (-1 * (*error_model).Pose.Pos[2]);
	const FLT x171 = 2 * (*lh_p).Rot[3];
	const FLT x172 = 2 * (*lh_p).Rot[2];
	const FLT x173 = (x38 * x172) + (-1 * x170 * x171);
	const FLT x174 = 2 * (*lh_p).Rot[1];
	const FLT x175 = 2 * x39;
	const FLT x176 = x149 + (x170 * x174) + x175;
	const FLT x177 = (x176 * x144) + (x120 * x173);
	const FLT x178 = 2 * x53;
	const FLT x179 = (-4 * x52) + (-1 * x178) + x153;
	const FLT x180 = (-1 * x91 * x179) + (x177 * x159);
	const FLT x181 = ((-1 * x162 * x176) + (x161 * x173)) * x163;
	const FLT x182 = (-1 * x181) + (x109 * x180);
	const FLT x183 = (x74 * x179) + (-1 * x156 * (x177 + (x179 * x150)));
	const FLT x184 = x78 * x183;
	const FLT x185 = x77 * x184;
	const FLT x186 = (x84 * x184) + (x75 * ((-1 * x76 * x184) + x185));
	const FLT x187 = (x86 * x184) + (x75 * x186);
	const FLT x188 =
		x181 +
		(-1 * x124 *
		 ((-1 * x182 * x133) + x180 + (x107 * x187) + (x184 * x132) +
		  (-1 * x118 *
		   ((x103 * ((x75 * x187) + (x116 * x183) + (x100 * x184) +
					 (x75 * ((x99 * x184) + x187 +
							 (x75 * (x186 + (x75 * ((x97 * x184) + x185 + (-1 * x115 * x184))) + (x98 * x184))))))) +
			(-1 * x182 * x135)))));
	const FLT x189 = (-1 * (*error_model).Pose.Pos[0]) + (-1 * x41) + (-1 * (*_x0).Pose.Pos[0]) + (-1 * x42) +
					 (-1 * sensor_pt[0]) + (-1 * x43);
	const FLT x190 = x154 + (x172 * x189) + x178;
	const FLT x191 = 2 * x57;
	const FLT x192 = (-4 * x58) + (-1 * x191) + x151;
	const FLT x193 = (x192 * x144) + (x120 * x190);
	const FLT x194 = (x49 * x171) + (-1 * x174 * x189);
	const FLT x195 = (-1 * x91 * x194) + (x193 * x159);
	const FLT x196 = ((-1 * x162 * x192) + (x161 * x190)) * x163;
	const FLT x197 = (-1 * x196) + (x109 * x195);
	const FLT x198 = (x74 * x194) + (-1 * x156 * (x193 + (x194 * x150)));
	const FLT x199 = x78 * x198;
	const FLT x200 = x77 * x199;
	const FLT x201 = (x84 * x199) + (x75 * ((-1 * x76 * x199) + x200));
	const FLT x202 = (x86 * x199) + (x75 * x201);
	const FLT x203 =
		x196 +
		(-1 * x124 *
		 (x195 + (-1 * x197 * x133) + (x199 * x132) +
		  (-1 * x118 *
		   ((x103 * ((x75 * x202) + (x116 * x198) + (x100 * x199) +
					 (x75 * ((x99 * x199) + x202 +
							 (x75 * (x201 + (x75 * ((x97 * x199) + x200 + (-1 * x115 * x199))) + (x98 * x199))))))) +
			(-1 * x197 * x135))) +
		  (x202 * x107)));
	const FLT x204 = x148 + (-1 * x175) + (-4 * x45);
	const FLT x205 = (-1 * sensor_pt[1]) + (-1 * x34) + (-1 * x37) + (-1 * (*error_model).Pose.Pos[1]) +
					 (-1 * (*_x0).Pose.Pos[1]) + (-1 * x35);
	const FLT x206 = (x44 * x174) + (-1 * x205 * x172);
	const FLT x207 = (x206 * x144) + (x204 * x120);
	const FLT x208 = x191 + x152 + (x205 * x171);
	const FLT x209 = (-1 * x91 * x208) + (x207 * x159);
	const FLT x210 = ((-1 * x206 * x162) + (x204 * x161)) * x163;
	const FLT x211 = (-1 * x210) + (x209 * x109);
	const FLT x212 = (x74 * x208) + (-1 * x156 * (x207 + (x208 * x150)));
	const FLT x213 = x78 * x212;
	const FLT x214 = x77 * x213;
	const FLT x215 = (x84 * x213) + (x75 * ((-1 * x76 * x213) + x214));
	const FLT x216 = (x86 * x213) + (x75 * x215);
	const FLT x217 =
		x210 +
		(-1 * x124 *
		 (x209 + (x213 * x132) + (x216 * x107) +
		  (-1 * x118 *
		   ((x103 * ((x75 * x216) + (x212 * x116) + (x213 * x100) +
					 (x75 * (x216 + (x99 * x213) +
							 (x75 * (x215 + (x75 * ((x97 * x213) + x214 + (-1 * x213 * x115))) + (x98 * x213))))))) +
			(-1 * x211 * x135))) +
		  (-1 * x211 * x133)));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[0]) / sizeof(FLT), x125 + (x126 * x125));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[1]) / sizeof(FLT), (-1 * x126 * x136) + (-1 * x136));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[2]) / sizeof(FLT), x147 + (x126 * x147));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[0]) / sizeof(FLT), x169 + (x126 * x169));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[1]) / sizeof(FLT), x188 + (x126 * x188));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[2]) / sizeof(FLT), x203 + (x203 * x126));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[3]) / sizeof(FLT), x217 + (x217 * x126));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen2 wrt [(*lh_p).Pos[0], (*lh_p).Pos[1],
// (*lh_p).Pos[2], (*lh_p).Rot[0], (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]

static inline void SurviveKalmanErrorModel_LightMeas_y_gen2_jac_lh_p_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_y_gen2(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_y_gen2_jac_lh_p(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
// Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen2 wrt [<cnkalman.codegen.WrapMember object at 0x7f88f4f82a90>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4f828b0>, <cnkalman.codegen.WrapMember object at 0x7f88f4f82a60>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4f82730>, <cnkalman.codegen.WrapMember object at 0x7f88f4f82790>,
// <cnkalman.codegen.WrapMember object at 0x7f88f4f828e0>, <cnkalman.codegen.WrapMember object at 0x7f88f4f82a30>]
static inline void SurviveKalmanErrorModel_LightMeas_y_gen2_jac_bsc0(CnMat *Hx, const FLT dt,
																	 const SurviveKalmanModel *_x0,
																	 const SurviveKalmanErrorModel *error_model,
																	 const FLT *sensor_pt, const SurvivePose *lh_p,
																	 const BaseStationCal *bsc0) {
	const FLT x0 = 0.523598775598299 + (-1 * (*bsc0).tilt);
	const FLT x1 = cos(x0);
	const FLT x2 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x4 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x5 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x6 =
		(-1 * x5 * (*_x0).Pose.Rot[1]) + (x4 * (*_x0).Pose.Rot[0]) + (x3 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x7 = dt * dt;
	const FLT x8 = (x2 * x2) * x7;
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x10 = x7 * (x9 * x9);
	const FLT x11 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x12 = x7 * (x11 * x11);
	const FLT x13 = 1e-10 + x12 + x8 + x10;
	const FLT x14 = sqrt(x13);
	const FLT x15 = 0.5 * x14;
	const FLT x16 = sin(x15);
	const FLT x17 = (1. / x13) * (x16 * x16);
	const FLT x18 = cos(x15);
	const FLT x19 = 1. / sqrt((x10 * x17) + (x18 * x18) + (x8 * x17) + (x12 * x17));
	const FLT x20 = dt * (1. / x14) * x19 * x16;
	const FLT x21 = x6 * x20;
	const FLT x22 =
		(x4 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[3] + (-1 * x3 * (*_x0).Pose.Rot[2]) + (x5 * (*_x0).Pose.Rot[0]);
	const FLT x23 = x22 * x20;
	const FLT x24 =
		(*_x0).Pose.Rot[1] + (x3 * (*_x0).Pose.Rot[0]) + (-1 * x4 * (*_x0).Pose.Rot[3]) + (x5 * (*_x0).Pose.Rot[2]);
	const FLT x25 = x19 * x18;
	const FLT x26 = (-1 * x3 * (*_x0).Pose.Rot[1]) + (-1 * x4 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x5 * (*_x0).Pose.Rot[3]);
	const FLT x27 = x20 * x26;
	const FLT x28 = (x9 * x27) + (x24 * x25) + (-1 * x2 * x21) + (x23 * x11);
	const FLT x29 = x24 * x20;
	const FLT x30 = (-1 * x9 * x29) + (-1 * x2 * x23) + (-1 * x21 * x11) + (x25 * x26);
	const FLT x31 = (x6 * x25) + (x2 * x29) + (-1 * x9 * x23) + (x27 * x11);
	const FLT x32 = (-1 * x31 * sensor_pt[0]) + (x28 * sensor_pt[1]) + (x30 * sensor_pt[2]);
	const FLT x33 = (x9 * x21) + (x25 * x22) + (-1 * x29 * x11) + (x2 * x27);
	const FLT x34 = (x31 * sensor_pt[2]) + (-1 * x33 * sensor_pt[1]) + (x30 * sensor_pt[0]);
	const FLT x35 = 1.0 / 2.0 * dt * fabs(dt);
	const FLT x36 = (*_x0).Pose.Pos[1] + (2 * ((x34 * x33) + (-1 * x32 * x28))) + (*error_model).Pose.Pos[1] +
					sensor_pt[1] + (dt * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) +
					(x35 * ((*_x0).Acc[1] + (*error_model).Acc[1]));
	const FLT x37 = (x30 * sensor_pt[1]) + (-1 * x28 * sensor_pt[2]) + (x33 * sensor_pt[0]);
	const FLT x38 = (*_x0).Pose.Pos[2] + sensor_pt[2] + (x35 * ((*_x0).Acc[2] + (*error_model).Acc[2])) +
					(2 * ((x37 * x28) + (-1 * x31 * x34))) +
					(dt * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2])) + (*error_model).Pose.Pos[2];
	const FLT x39 = (x35 * ((*_x0).Acc[0] + (*error_model).Acc[0])) + (*error_model).Pose.Pos[0] +
					(2 * ((x32 * x31) + (-1 * x33 * x37))) + sensor_pt[0] +
					(dt * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) + (*_x0).Pose.Pos[0];
	const FLT x40 = (-1 * x39 * (*lh_p).Rot[2]) + (x36 * (*lh_p).Rot[1]) + (x38 * (*lh_p).Rot[0]);
	const FLT x41 = (x39 * (*lh_p).Rot[0]) + (-1 * x36 * (*lh_p).Rot[3]) + (x38 * (*lh_p).Rot[2]);
	const FLT x42 = x36 + (*lh_p).Pos[1] + (2 * ((x41 * (*lh_p).Rot[3]) + (-1 * x40 * (*lh_p).Rot[1])));
	const FLT x43 = x42 * x42;
	const FLT x44 = (x36 * (*lh_p).Rot[0]) + (-1 * x38 * (*lh_p).Rot[1]) + (x39 * (*lh_p).Rot[3]);
	const FLT x45 = x38 + (2 * ((x44 * (*lh_p).Rot[1]) + (-1 * x41 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x46 = x39 + (*lh_p).Pos[0] + (2 * ((x40 * (*lh_p).Rot[2]) + (-1 * x44 * (*lh_p).Rot[3])));
	const FLT x47 = (x46 * x46) + (x45 * x45);
	const FLT x48 = x47 + x43;
	const FLT x49 = x42 * (1. / sqrt(x48));
	const FLT x50 = asin((1. / x1) * x49);
	const FLT x51 = 8.0108022e-06 * x50;
	const FLT x52 = -8.0108022e-06 + (-1 * x51);
	const FLT x53 = 0.0028679863 + (x50 * x52);
	const FLT x54 = 5.3685255e-06 + (x50 * x53);
	const FLT x55 = 0.0076069798 + (x50 * x54);
	const FLT x56 = x50 * x55;
	const FLT x57 = -8.0108022e-06 + (-1.60216044e-05 * x50);
	const FLT x58 = x53 + (x50 * x57);
	const FLT x59 = x54 + (x50 * x58);
	const FLT x60 = x55 + (x50 * x59);
	const FLT x61 = (x60 * x50) + x56;
	const FLT x62 = sin(x0);
	const FLT x63 = atan2(-1 * x45, x46);
	const FLT x64 = tan(x0);
	const FLT x65 = x42 * (1. / sqrt(x47));
	const FLT x66 = -1 * x64 * x65;
	const FLT x67 = asin(x66) + (-1 * x63) + (-1 * (*bsc0).ogeephase);
	const FLT x68 = sin(x67);
	const FLT x69 = (-1 * x68 * (*bsc0).ogeemag) + (*bsc0).curve;
	const FLT x70 = x62 * x69;
	const FLT x71 = x70 * x61;
	const FLT x72 = x50 * x50;
	const FLT x73 = x1 + x71;
	const FLT x74 = (1. / (x73 * x73)) * x72 * x55;
	const FLT x75 = x71 * x74;
	const FLT x76 = 1. / x73;
	const FLT x77 = x72 * x76;
	const FLT x78 = x77 * x55;
	const FLT x79 = x66 + (x78 * x69);
	const FLT x80 = 1. / sqrt(1 + (-1 * (x79 * x79)));
	const FLT x81 = (x78 + (-1 * x75)) * x80;
	const FLT x82 = (-1 * asin(x79)) + (*bsc0).gibpha + x63;
	const FLT x83 = cos(x82) * (*bsc0).gibmag;
	const FLT x84 = ((-1 * x78 * x68) + (x75 * x68)) * x80;
	const FLT x85 = cos(x67) * (*bsc0).ogeemag;
	const FLT x86 = x85 * x78;
	const FLT x87 = x80 * (x86 + (-1 * x85 * x75));
	const FLT x88 = x64 * x64;
	const FLT x89 = x65 * (1 + x88);
	const FLT x90 = 1. / (x1 * x1);
	const FLT x91 = x90 * x49 * (1. / sqrt(1 + (-1 * x90 * x43 * (1. / x48))));
	const FLT x92 = x62 * x91;
	const FLT x93 = -1 * x52 * x92;
	const FLT x94 = (-1 * x53 * x92) + (x50 * ((x51 * x92) + x93));
	const FLT x95 = (-1 * x54 * x92) + (x50 * x94);
	const FLT x96 = x89 * (1. / sqrt(1 + (-1 * x88 * x43 * (1. / x47))));
	const FLT x97 =
		x80 * ((-1 * x74 * x69 *
				((-1 * x1 * x61 * x69) +
				 (x70 * ((x50 * x95) + (-1 * x60 * x92) + (-1 * x55 * x92) +
						 (x50 * (x95 + (-1 * x59 * x92) +
								 (x50 * (x94 + (x50 * (x93 + (-1 * x57 * x92) + (2.40324066e-05 * x50 * x92))) +
										 (-1 * x58 * x92))))))) +
				 x62 + (-1 * x85 * x61 * x62 * x96))) +
			   (x77 * x69 * x95) + x89 + (-1 * x86 * x96) + (-2 * x70 * x76 * x56 * x91));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, curve) / sizeof(FLT), (-1 * x81 * x83) + (-1 * x81));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibmag) / sizeof(FLT), sin(x82));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibpha) / sizeof(FLT), x83);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, ogeemag) / sizeof(FLT), (-1 * x83 * x84) + (-1 * x84));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, ogeephase) / sizeof(FLT), (-1 * x83 * x87) + (-1 * x87));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, phase) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, tilt) / sizeof(FLT), (-1 * x83 * x97) + (-1 * x97));
}

// Full version Jacobian of SurviveKalmanErrorModel_LightMeas_y_gen2 wrt [<cnkalman.codegen.WrapMember object at
// 0x7f88f4f82a90>, <cnkalman.codegen.WrapMember object at 0x7f88f4f828b0>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4f82a60>, <cnkalman.codegen.WrapMember object at 0x7f88f4f82730>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4f82790>, <cnkalman.codegen.WrapMember object at 0x7f88f4f828e0>, <cnkalman.codegen.WrapMember object at
// 0x7f88f4f82a30>]

static inline void SurviveKalmanErrorModel_LightMeas_y_gen2_jac_bsc0_with_hx(
	CnMat *Hx, CnMat *hx, const FLT dt, const SurviveKalmanModel *_x0, const SurviveKalmanErrorModel *error_model,
	const FLT *sensor_pt, const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	if (hx != 0) {
		hx->data[0] = SurviveKalmanErrorModel_LightMeas_y_gen2(dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
	if (Hx != 0) {
		SurviveKalmanErrorModel_LightMeas_y_gen2_jac_bsc0(Hx, dt, _x0, error_model, sensor_pt, lh_p, bsc0);
	}
}
