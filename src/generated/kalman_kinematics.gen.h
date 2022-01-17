/// NOTE: This is a generated file; do not edit.
#pragma once
#include <cnkalman/generated_header.h>
// clang-format off
static inline void gen_axisangle2euler(CnMat *out, const FLT *axis_angle) {
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
	const FLT x14 = 2 * x13;
	const FLT x15 = (1. / x4) * x6 * x10;
	const FLT x16 = 2 * x12;
	const FLT x17 = 1 + (-1 * x9 * x16);
	const FLT x18 = x7 * axis_angle0;
	const FLT x19 = x12 * axis_angle1;
	cnSetZero(out);
	cnMatrixOptionalSet(out, 0, 0, atan2((x15 * x16 * axis_angle0) + (x7 * x14 * axis_angle1), x17 + (-1 * x11 * x16)));
	cnMatrixOptionalSet(out, 1, 0, asin(2 * ((x15 * x19) + (-1 * x13 * x18))));
	cnMatrixOptionalSet(out, 2, 0, atan2((2 * x19 * x18) + (x15 * x14), x17 + (-1 * x8 * x16)));
}

// Jacobian of axisangle2euler wrt [axis_angle0, axis_angle1, axis_angle2]
static inline void gen_axisangle2euler_jac_axis_angle(CnMat *Hx, const FLT *axis_angle) {
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
	const FLT x10 = x0 * x9;
	const FLT x11 = x1 * x9;
	const FLT x12 = x2 * x9;
	const FLT x13 = cos(x6);
	const FLT x14 = x13 * x13;
	const FLT x15 = x10 + x14 + x11 + x12;
	const FLT x16 = 1. / x15;
	const FLT x17 = 2 * x16;
	const FLT x18 = 1 + (-1 * x12 * x17);
	const FLT x19 = x18 + (-1 * x10 * x17);
	const FLT x20 = 1. / x19;
	const FLT x21 = x4 * x14;
	const FLT x22 = 1.0 * x16;
	const FLT x23 = x22 * x21;
	const FLT x24 = 1. / (x3 * sqrt(x3));
	const FLT x25 = x7 * x13;
	const FLT x26 = x25 * x16;
	const FLT x27 = x24 * x26;
	const FLT x28 = 2 * x27;
	const FLT x29 = 2 * x9;
	const FLT x30 = x29 * axis_angle2;
	const FLT x31 = 1. / (x15 * x15);
	const FLT x32 = x29 * axis_angle0;
	const FLT x33 = axis_angle0 * axis_angle0 * axis_angle0;
	const FLT x34 = (1. / (x3 * x3)) * x8;
	const FLT x35 = 2 * x34;
	const FLT x36 = 1.0 * x25;
	const FLT x37 = x36 * x24;
	const FLT x38 = x1 * x35;
	const FLT x39 = x38 * axis_angle0;
	const FLT x40 = x37 * axis_angle0;
	const FLT x41 = x2 * x35;
	const FLT x42 = 1. / x5;
	const FLT x43 = x42 * x36;
	const FLT x44 = ((x2 * x40) + (-1 * x43 * axis_angle0) + (-1 * x33 * x35) + (-1 * x41 * axis_angle0) + x32 +
					 (x33 * x37) + (-1 * x39) + (x1 * x40)) *
					x31;
	const FLT x45 = x44 * axis_angle1;
	const FLT x46 = x42 * x25;
	const FLT x47 = 2 * x46;
	const FLT x48 = x47 * axis_angle0;
	const FLT x49 = x27 * axis_angle1;
	const FLT x50 = x49 * axis_angle2;
	const FLT x51 = x50 * axis_angle0;
	const FLT x52 = 4 * x34 * x16;
	const FLT x53 = axis_angle0 * axis_angle2;
	const FLT x54 = x53 * axis_angle1;
	const FLT x55 = x42 * x26;
	const FLT x56 = 2 * x55;
	const FLT x57 = x56 + (2.0 * x51) + (-1 * x54 * x52);
	const FLT x58 = x9 * x16;
	const FLT x59 = x58 * axis_angle0;
	const FLT x60 = 2.0 * x27;
	const FLT x61 = 2 * x10;
	const FLT x62 = 2 * x12;
	const FLT x63 = x2 * x27;
	const FLT x64 = 2.0 * x63;
	const FLT x65 = x64 * axis_angle0;
	const FLT x66 = x2 * x52;
	const FLT x67 = x66 * axis_angle0;
	const FLT x68 = x67 + (x62 * x44) + (-1 * x65);
	const FLT x69 = x19 * x19;
	const FLT x70 = x30 * x16;
	const FLT x71 = (x56 * axis_angle0) + (x70 * axis_angle1);
	const FLT x72 = x71 * (1. / x69);
	const FLT x73 = x69 * (1. / (x69 + (x71 * x71)));
	const FLT x74 = x49 * axis_angle0;
	const FLT x75 = x23 * axis_angle1;
	const FLT x76 = x0 * x35;
	const FLT x77 = x37 * axis_angle1;
	const FLT x78 = axis_angle1 * axis_angle1 * axis_angle1;
	const FLT x79 = x29 * axis_angle1;
	const FLT x80 = x31 * ((x78 * x37) + (-1 * x78 * x35) + (x0 * x77) + (-1 * x76 * axis_angle1) +
						   (-1 * x43 * axis_angle1) + (-1 * x38 * axis_angle1) + x79 + (x1 * x77));
	const FLT x81 = x80 * axis_angle1;
	const FLT x82 = x64 * axis_angle2;
	const FLT x83 = x66 * axis_angle2;
	const FLT x84 = x80 * x47;
	const FLT x85 = 1.0 * axis_angle1;
	const FLT x86 = x0 * x52;
	const FLT x87 = x86 * axis_angle1;
	const FLT x88 = 2.0 * x49;
	const FLT x89 = x0 * x88;
	const FLT x90 = (x80 * x62) + (-1 * x78 * x60) + (-4 * x58 * axis_angle1) + (x78 * x52);
	const FLT x91 = axis_angle2 * axis_angle2 * axis_angle2;
	const FLT x92 = x37 * axis_angle2;
	const FLT x93 = x76 * axis_angle2;
	const FLT x94 = ((-1 * x41 * axis_angle2) + (x2 * x92) + (-1 * x93) + (x91 * x37) + (-1 * x43 * axis_angle2) +
					 (x0 * x92) + (-1 * x91 * x35) + x30) *
					x31;
	const FLT x95 = x94 * axis_angle1;
	const FLT x96 = x1 * x88;
	const FLT x97 = x1 * x52;
	const FLT x98 = x97 * axis_angle1;
	const FLT x99 = x58 * axis_angle2;
	const FLT x100 = x99 * axis_angle0;
	const FLT x101 = (x53 * x23) + (-1 * x53 * x28) + (x79 * x16) + (-1.0 * x100);
	const FLT x102 = x0 * axis_angle2;
	const FLT x103 = (-1 * x82) + x83 + (x62 * x94);
	const FLT x104 = x46 * axis_angle1;
	const FLT x105 = 0.5 * axis_angle1;
	const FLT x106 = x9 * x53;
	const FLT x107 = 0.5 * x16;
	const FLT x108 = x21 * x107;
	const FLT x109 = x108 * axis_angle1;
	const FLT x110 = 1.0 * x27;
	const FLT x111 =
		2 * (1. / sqrt(1 + (-4 * (((x55 * axis_angle1) + (-1 * x100)) * ((x55 * axis_angle1) + (-1 * x100))))));
	const FLT x112 = x1 * axis_angle0;
	const FLT x113 = x18 + (-1 * x11 * x17);
	const FLT x114 = 1. / x113;
	const FLT x115 = x47 * axis_angle2;
	const FLT x116 = 2 * x11;
	const FLT x117 = x113 * x113;
	const FLT x118 = x32 * x16;
	const FLT x119 = (x118 * axis_angle1) + (x56 * axis_angle2);
	const FLT x120 = x119 * (1. / x117);
	const FLT x121 = x117 * (1. / (x117 + (x119 * x119)));
	cnSetZero(Hx);
	cnMatrixOptionalSet(
		Hx, 0, 0,
		x73 * ((-1 * x72 * ((x52 * x33) + (x61 * x44) + x68 + (-4 * x59) + (-1 * x60 * x33))) +
			   (x20 * (x57 + (-1 * x0 * x28) + (-1 * x22 * x10) + (x0 * x23) + (-1 * x45 * x30) + (-1 * x44 * x48)))));
	cnMatrixOptionalSet(Hx, 0, 1,
						x73 * ((-1 * x72 * ((-1 * x89) + (x80 * x61) + x90 + x87)) +
							   (((-1 * x85 * x59) + x70 + (x75 * axis_angle0) + (-2 * x74) + (-1 * x83) +
								 (-1 * x81 * x30) + (-1 * x84 * axis_angle0) + x82) *
								x20)));
	cnMatrixOptionalSet(Hx, 0, 2,
						x73 * ((-1 * x72 * (x103 + (x86 * axis_angle2) + (x61 * x94) + (-1 * x60 * x102))) +
							   (x20 * (x101 + (-1 * x94 * x48) + (-1 * x98) + (-1 * x95 * x30) + x96))));
	cnMatrixOptionalSet(Hx, 1, 0,
						x111 * ((-1 * x99) + (x93 * x16) + (-1 * x102 * x110) + (-1 * x44 * x104) +
								(x109 * axis_angle0) + (-1 * x74) + (-1 * x59 * x105) + (x44 * x106)));
	cnMatrixOptionalSet(Hx, 1, 1,
						x111 * ((x80 * x106) + (x54 * x35 * x16) + x55 + (-1.0 * x51) + (-1 * x81 * x46) +
								(-1 * x12 * x107) + (-1 * x63) + (x2 * x108)));
	cnMatrixOptionalSet(Hx, 1, 2,
						x111 * ((-1 * x110 * x112) + (-1 * x94 * x104) + (x39 * x16) + (-1 * x50) +
								(x109 * axis_angle2) + (x94 * x106) + (-1 * x59) + (-1 * x99 * x105)));
	cnMatrixOptionalSet(Hx, 2, 0,
						x121 * ((-1 * x120 * (x68 + (x97 * axis_angle0) + (x44 * x116) + (-1 * x60 * x112))) +
								(x114 * ((-1 * x44 * x115) + x89 + (-1 * x87) + x101 + (-1 * x45 * x32)))));
	cnMatrixOptionalSet(Hx, 2, 1,
						x121 * ((-1 * x120 * (x90 + x98 + (-1 * x96) + (x80 * x116))) +
								(x114 * (x65 + (-1 * x81 * x32) + (x75 * axis_angle2) + (-2 * x50) +
										 (-1 * x84 * axis_angle2) + (-1 * x67) + x118 + (-1 * x85 * x99)))));
	cnMatrixOptionalSet(Hx, 2, 2,
						x121 * ((-1 * x120 * (x103 + (-1 * x60 * x91) + (-4 * x99) + (x94 * x116) + (x52 * x91))) +
								(x114 * ((-1 * x94 * x115) + (-1 * x95 * x32) + x57 + (x1 * x23) + (-1 * x1 * x28) +
										 (-1 * x22 * x11)))));
}

// Full version Jacobian of axisangle2euler wrt [axis_angle0, axis_angle1, axis_angle2]

static inline void gen_axisangle2euler_jac_axis_angle_with_hx(CnMat *Hx, CnMat *hx, const FLT *axis_angle) {
	if (hx != 0) {
		gen_axisangle2euler(hx, axis_angle);
	}
	if (Hx != 0) {
		gen_axisangle2euler_jac_axis_angle(Hx, axis_angle);
	}
}
static inline void gen_GenerateQuatErrorModel(CnMat *out, const FLT *_x1, const FLT *_x0) {
	const FLT _x10 = _x1[0];
	const FLT _x11 = _x1[1];
	const FLT _x12 = _x1[2];
	const FLT _x13 = _x1[3];
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT x0 = (_x02 * _x11) + (-1 * _x01 * _x12) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x1 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x2 = 2 * x1;
	const FLT x3 = (_x03 * _x12) + (-1 * _x02 * _x13) + (-1 * _x01 * _x10) + (_x00 * _x11);
	const FLT x4 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x5 = 2 * x4;
	const FLT x6 = 1 + (-2 * (x1 * x1));
	cnSetZero(out);
	cnMatrixOptionalSet(out, 0, 0, atan2((x3 * x5) + (x0 * x2), x6 + (-2 * (x3 * x3))));
	cnMatrixOptionalSet(out, 1, 0, asin(2 * ((x1 * x4) + (-1 * x0 * x3))));
	cnMatrixOptionalSet(out, 2, 0, atan2((x0 * x5) + (x2 * x3), x6 + (-2 * (x0 * x0))));
}

// Jacobian of GenerateQuatErrorModel wrt [_x10, _x11, _x12, _x13]
static inline void gen_GenerateQuatErrorModel_jac_x1(CnMat *Hx, const FLT *_x1, const FLT *_x0) {
	const FLT _x10 = _x1[0];
	const FLT _x11 = _x1[1];
	const FLT _x12 = _x1[2];
	const FLT _x13 = _x1[3];
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT x0 = (_x03 * _x12) + (-1 * _x02 * _x13) + (-1 * _x01 * _x10) + (_x00 * _x11);
	const FLT x1 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x2 = 1 + (-2 * (x1 * x1));
	const FLT x3 = x2 + (-2 * (x0 * x0));
	const FLT x4 = 1. / x3;
	const FLT x5 = x1 * _x03;
	const FLT x6 = 2 * x5;
	const FLT x7 = (_x02 * _x11) + (-1 * _x01 * _x12) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x8 = x7 * _x02;
	const FLT x9 = 2 * x8;
	const FLT x10 = x0 * _x00;
	const FLT x11 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x12 = x11 * _x01;
	const FLT x13 = (-2 * x12) + (2 * x10);
	const FLT x14 = x0 * _x01;
	const FLT x15 = x1 * _x02;
	const FLT x16 = 4 * x15;
	const FLT x17 = x3 * x3;
	const FLT x18 = 2 * x1;
	const FLT x19 = 2 * x11;
	const FLT x20 = (x0 * x19) + (x7 * x18);
	const FLT x21 = x20 * (1. / x17);
	const FLT x22 = x17 * (1. / (x17 + (x20 * x20)));
	const FLT x23 = 2 * x15;
	const FLT x24 = x7 * _x03;
	const FLT x25 = 2 * x24;
	const FLT x26 = x11 * _x00;
	const FLT x27 = (2 * x14) + (2 * x26);
	const FLT x28 = 4 * x5;
	const FLT x29 = x0 * _x02;
	const FLT x30 = 2 * x29;
	const FLT x31 = x11 * _x03;
	const FLT x32 = 2 * x31;
	const FLT x33 = x7 * _x00;
	const FLT x34 = x1 * _x01;
	const FLT x35 = (-2 * x34) + (2 * x33);
	const FLT x36 = x0 * _x03;
	const FLT x37 = x1 * _x00;
	const FLT x38 = -4 * x37;
	const FLT x39 = x11 * _x02;
	const FLT x40 = 2 * x39;
	const FLT x41 = 2 * x36;
	const FLT x42 = x7 * _x01;
	const FLT x43 = (2 * x37) + (2 * x42);
	const FLT x44 = -4 * x34;
	const FLT x45 = 2 * (1. / sqrt(1 + (-4 * (((x1 * x11) + (-1 * x0 * x7)) * ((x1 * x11) + (-1 * x0 * x7))))));
	const FLT x46 = x2 + (-2 * (x7 * x7));
	const FLT x47 = 1. / x46;
	const FLT x48 = x46 * x46;
	const FLT x49 = (x7 * x19) + (x0 * x18);
	const FLT x50 = (1. / x48) * x49;
	const FLT x51 = x48 * (1. / (x48 + (x49 * x49)));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, 0, x22 * ((-1 * (x16 + (4 * x14)) * x21) + (x4 * (x13 + (-1 * x6) + (-1 * x9)))));
	cnMatrixOptionalSet(Hx, 0, 1, x22 * ((-1 * (x28 + (-4 * x10)) * x21) + (x4 * (x27 + x23 + (-1 * x25)))));
	cnMatrixOptionalSet(Hx, 0, 2, x22 * ((-1 * (x38 + (-4 * x36)) * x21) + (x4 * (x35 + x30 + x32))));
	cnMatrixOptionalSet(Hx, 0, 3, x22 * ((-1 * (x44 + (4 * x29)) * x21) + (x4 * (x43 + (-1 * x40) + x41))));
	cnMatrixOptionalSet(Hx, 1, 0, (x42 + x36 + (-1 * x39) + x37) * x45);
	cnMatrixOptionalSet(Hx, 1, 1, ((-1 * x33) + (-1 * x29) + (-1 * x31) + x34) * x45);
	cnMatrixOptionalSet(Hx, 1, 2, ((-1 * x24) + x14 + x26 + x15) * x45);
	cnMatrixOptionalSet(Hx, 1, 3, (x8 + x5 + x12 + (-1 * x10)) * x45);
	cnMatrixOptionalSet(Hx, 2, 0, x51 * ((-1 * ((4 * x24) + x16) * x50) + (x47 * (x35 + (-1 * x32) + (-1 * x30)))));
	cnMatrixOptionalSet(Hx, 2, 1, x51 * ((-1 * x50 * ((-4 * x8) + x28)) + (x47 * (x43 + x40 + (-1 * x41)))));
	cnMatrixOptionalSet(Hx, 2, 2, x51 * ((-1 * ((4 * x42) + x38) * x50) + (x47 * (x13 + x9 + x6))));
	cnMatrixOptionalSet(Hx, 2, 3, x51 * ((-1 * ((-4 * x33) + x44) * x50) + (x47 * (x27 + x25 + (-1 * x23)))));
}

// Full version Jacobian of GenerateQuatErrorModel wrt [_x10, _x11, _x12, _x13]

static inline void gen_GenerateQuatErrorModel_jac_x1_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x1, const FLT *_x0) {
	if (hx != 0) {
		gen_GenerateQuatErrorModel(hx, _x1, _x0);
	}
	if (Hx != 0) {
		gen_GenerateQuatErrorModel_jac_x1(Hx, _x1, _x0);
	}
}
// Jacobian of GenerateQuatErrorModel wrt [_x00, _x01, _x02, _x03]
static inline void gen_GenerateQuatErrorModel_jac_x0(CnMat *Hx, const FLT *_x1, const FLT *_x0) {
	const FLT _x10 = _x1[0];
	const FLT _x11 = _x1[1];
	const FLT _x12 = _x1[2];
	const FLT _x13 = _x1[3];
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT x0 = (_x03 * _x12) + (-1 * _x02 * _x13) + (-1 * _x01 * _x10) + (_x00 * _x11);
	const FLT x1 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x2 = 1 + (-2 * (x1 * x1));
	const FLT x3 = x2 + (-2 * (x0 * x0));
	const FLT x4 = 1. / x3;
	const FLT x5 = x0 * _x10;
	const FLT x6 = 2 * x5;
	const FLT x7 = x1 * _x13;
	const FLT x8 = 2 * x7;
	const FLT x9 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x10 = x9 * _x11;
	const FLT x11 = (_x02 * _x11) + (-1 * _x01 * _x12) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x12 = x11 * _x12;
	const FLT x13 = (2 * x12) + (2 * x10);
	const FLT x14 = x0 * _x11;
	const FLT x15 = x1 * _x12;
	const FLT x16 = -4 * x15;
	const FLT x17 = x3 * x3;
	const FLT x18 = 2 * x1;
	const FLT x19 = 2 * x9;
	const FLT x20 = (x0 * x19) + (x11 * x18);
	const FLT x21 = x20 * (1. / x17);
	const FLT x22 = x17 * (1. / (x17 + (x20 * x20)));
	const FLT x23 = 2 * x14;
	const FLT x24 = 2 * x15;
	const FLT x25 = x11 * _x13;
	const FLT x26 = x9 * _x10;
	const FLT x27 = (-2 * x26) + (2 * x25);
	const FLT x28 = -4 * x7;
	const FLT x29 = x9 * _x13;
	const FLT x30 = 2 * x29;
	const FLT x31 = x11 * _x10;
	const FLT x32 = 2 * x31;
	const FLT x33 = x1 * _x11;
	const FLT x34 = x0 * _x12;
	const FLT x35 = (2 * x34) + (2 * x33);
	const FLT x36 = x0 * _x13;
	const FLT x37 = x1 * _x10;
	const FLT x38 = 4 * x37;
	const FLT x39 = x9 * _x12;
	const FLT x40 = 2 * x39;
	const FLT x41 = x11 * _x11;
	const FLT x42 = 2 * x41;
	const FLT x43 = (-2 * x37) + (2 * x36);
	const FLT x44 = 4 * x33;
	const FLT x45 = 2 * (1. / sqrt(1 + (-4 * (((x1 * x9) + (-1 * x0 * x11)) * ((x1 * x9) + (-1 * x0 * x11))))));
	const FLT x46 = x2 + (-2 * (x11 * x11));
	const FLT x47 = 1. / x46;
	const FLT x48 = x46 * x46;
	const FLT x49 = (x11 * x19) + (x0 * x18);
	const FLT x50 = (1. / x48) * x49;
	const FLT x51 = x48 * (1. / (x48 + (x49 * x49)));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, 0, x22 * ((-1 * (x16 + (-4 * x14)) * x21) + (x4 * (x6 + x13 + x8))));
	cnMatrixOptionalSet(Hx, 0, 1, x22 * ((-1 * x21 * (x28 + (4 * x5))) + (x4 * (x23 + x27 + (-1 * x24)))));
	cnMatrixOptionalSet(Hx, 0, 2, x22 * ((-1 * (x38 + (4 * x36)) * x21) + (x4 * (x35 + (-1 * x30) + (-1 * x32)))));
	cnMatrixOptionalSet(Hx, 0, 3, x22 * ((-1 * (x44 + (-4 * x34)) * x21) + (x4 * (x43 + x40 + (-1 * x42)))));
	cnMatrixOptionalSet(Hx, 1, 0, ((-1 * x41) + (-1 * x36) + x39 + x37) * x45);
	cnMatrixOptionalSet(Hx, 1, 1, (x34 + x31 + x29 + x33) * x45);
	cnMatrixOptionalSet(Hx, 1, 2, (x25 + (-1 * x14) + (-1 * x26) + x15) * x45);
	cnMatrixOptionalSet(Hx, 1, 3, (x5 + (-1 * x10) + (-1 * x12) + x7) * x45);
	cnMatrixOptionalSet(Hx, 2, 0, x51 * ((-1 * ((-4 * x25) + x16) * x50) + (x47 * (x35 + x30 + x32))));
	cnMatrixOptionalSet(Hx, 2, 1, x51 * ((-1 * ((4 * x12) + x28) * x50) + (x47 * (x43 + x42 + (-1 * x40)))));
	cnMatrixOptionalSet(Hx, 2, 2, x51 * ((-1 * ((-4 * x41) + x38) * x50) + (x47 * (x13 + (-1 * x8) + (-1 * x6)))));
	cnMatrixOptionalSet(Hx, 2, 3, x51 * ((-1 * ((4 * x31) + x44) * x50) + (x47 * (x27 + x24 + (-1 * x23)))));
}

// Full version Jacobian of GenerateQuatErrorModel wrt [_x00, _x01, _x02, _x03]

static inline void gen_GenerateQuatErrorModel_jac_x0_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x1, const FLT *_x0) {
	if (hx != 0) {
		gen_GenerateQuatErrorModel(hx, _x1, _x0);
	}
	if (Hx != 0) {
		gen_GenerateQuatErrorModel_jac_x0(Hx, _x1, _x0);
	}
}
static inline void gen_GenerateQuatModel(CnMat *out, const FLT *_x0, const FLT *error_state) {
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
	const FLT x12 = x5 * x7;
	const FLT x13 = x1 * x8;
	const FLT x14 = (x3 * x13) + (-1 * x9 * x12);
	const FLT x15 = (x6 * x9) + (x7 * x13);
	const FLT x16 = (x3 * x10) + (x1 * x12);
	const FLT x17 = 1. / sqrt((x11 * x11) + (x16 * x16) + (x14 * x14) + (x15 * x15));
	const FLT x18 = x11 * x17;
	const FLT x19 = x17 * _x02;
	const FLT x20 = x17 * x16;
	const FLT x21 = x17 * _x01;
	const FLT x22 = x15 * x17;
	const FLT x23 = x14 * x17;
	cnSetZero(out);
	cnMatrixOptionalSet(out, 0, 0, (-1 * x21 * x14) + (x20 * _x00) + (-1 * x18 * _x03) + (-1 * x15 * x19));
	cnMatrixOptionalSet(out, 1, 0, (x23 * _x00) + (x21 * x16) + (-1 * x22 * _x03) + (x11 * x19));
	cnMatrixOptionalSet(out, 2, 0, (x19 * x16) + (-1 * x18 * _x01) + (x22 * _x00) + (x23 * _x03));
	cnMatrixOptionalSet(out, 3, 0, (x22 * _x01) + (x18 * _x00) + (-1 * x14 * x19) + (x20 * _x03));
}

// Jacobian of GenerateQuatModel wrt [_x00, _x01, _x02, _x03]
static inline void gen_GenerateQuatModel_jac_x0(CnMat *Hx, const FLT *_x0, const FLT *error_state) {
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
	const FLT x12 = x1 * x9;
	const FLT x13 = (x7 * x12) + (-1 * x6 * x8);
	const FLT x14 = x3 * x7;
	const FLT x15 = (x8 * x14) + (x5 * x12);
	const FLT x16 = (x5 * x10) + (-1 * x1 * x14);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x11 * x11) + (x13 * x13) + (x15 * x15));
	const FLT x18 = x11 * x17;
	const FLT x19 = x13 * x17;
	const FLT x20 = -1 * x19;
	const FLT x21 = x15 * x17;
	const FLT x22 = -1 * x21;
	const FLT x23 = x17 * x16;
	const FLT x24 = -1 * x23;
	cnSetZero(Hx);
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

static inline void gen_GenerateQuatModel_jac_x0_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x0, const FLT *error_state) {
	if (hx != 0) {
		gen_GenerateQuatModel(hx, _x0, error_state);
	}
	if (Hx != 0) {
		gen_GenerateQuatModel_jac_x0(Hx, _x0, error_state);
	}
}
// Jacobian of GenerateQuatModel wrt [error_state0, error_state1, error_state2]
static inline void gen_GenerateQuatModel_jac_error_state(CnMat *Hx, const FLT *_x0, const FLT *error_state) {
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT error_state0 = error_state[0];
	const FLT error_state1 = error_state[1];
	const FLT error_state2 = error_state[2];
	const FLT x0 = 0.5 * error_state1;
	const FLT x1 = sin(x0);
	const FLT x2 = 0.5 * error_state2;
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * error_state0;
	const FLT x5 = cos(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = x1 * x6;
	const FLT x8 = cos(x2);
	const FLT x9 = cos(x0);
	const FLT x10 = sin(x4);
	const FLT x11 = x9 * x10;
	const FLT x12 = x8 * x11;
	const FLT x13 = x12 + (-1 * x7);
	const FLT x14 = x3 * x11;
	const FLT x15 = x5 * x8;
	const FLT x16 = x1 * x15;
	const FLT x17 = x16 + x14;
	const FLT x18 = x1 * x10;
	const FLT x19 = x3 * x18;
	const FLT x20 = x9 * x15;
	const FLT x21 = x20 + x19;
	const FLT x22 = x8 * x18;
	const FLT x23 = x6 * x9;
	const FLT x24 = x23 + (-1 * x22);
	const FLT x25 = (x24 * x24) + (x21 * x21) + (x13 * x13) + (x17 * x17);
	const FLT x26 = 1.0 / 2.0 * (1. / (x25 * sqrt(x25)));
	const FLT x27 = x26 * _x01;
	const FLT x28 = 0.5 * x14;
	const FLT x29 = -0.5 * x16;
	const FLT x30 = x29 + (-1 * x28);
	const FLT x31 = 2 * x24;
	const FLT x32 = 0.5 * x12;
	const FLT x33 = -1 * x32;
	const FLT x34 = 0.5 * x7;
	const FLT x35 = x34 + x33;
	const FLT x36 = 2 * x21;
	const FLT x37 = 0.5 * x20;
	const FLT x38 = 0.5 * x19;
	const FLT x39 = x38 + x37;
	const FLT x40 = 2 * x13;
	const FLT x41 = 0.5 * x22;
	const FLT x42 = -1 * x41;
	const FLT x43 = 0.5 * x23;
	const FLT x44 = x43 + x42;
	const FLT x45 = 2 * x17;
	const FLT x46 = (x44 * x45) + (x40 * x39) + (x30 * x31) + (x36 * x35);
	const FLT x47 = x46 * x13;
	const FLT x48 = 1. / sqrt(x25);
	const FLT x49 = x48 * _x00;
	const FLT x50 = x48 * _x01;
	const FLT x51 = -1 * x50 * x39;
	const FLT x52 = x26 * _x02;
	const FLT x53 = x46 * x17;
	const FLT x54 = x48 * _x02;
	const FLT x55 = x48 * x30;
	const FLT x56 = x55 * _x03;
	const FLT x57 = x46 * x24;
	const FLT x58 = x57 * x26;
	const FLT x59 = x26 * _x00;
	const FLT x60 = x59 * x21;
	const FLT x61 = -1 * x43;
	const FLT x62 = x61 + x42;
	const FLT x63 = -1 * x34;
	const FLT x64 = x33 + x63;
	const FLT x65 = x28 + x29;
	const FLT x66 = (-1 * x38) + x37;
	const FLT x67 = (x66 * x45) + (x62 * x40) + (x64 * x31) + (x65 * x36);
	const FLT x68 = x67 * x13;
	const FLT x69 = x65 * x48;
	const FLT x70 = x48 * _x03;
	const FLT x71 = x67 * x24;
	const FLT x72 = x71 * x26;
	const FLT x73 = x67 * x17;
	const FLT x74 = -1 * x55 * _x01;
	const FLT x75 = x41 + x61;
	const FLT x76 = x70 * x39;
	const FLT x77 = x26 * _x03;
	const FLT x78 = x32 + x63;
	const FLT x79 = (x78 * x45) + (x40 * x30) + (x31 * x39) + (x75 * x36);
	const FLT x80 = x79 * x24;
	const FLT x81 = x79 * x17;
	const FLT x82 = x79 * x13;
	const FLT x83 = x46 * x21;
	const FLT x84 = x55 * _x02;
	const FLT x85 = x49 * x39;
	const FLT x86 = x59 * x46;
	const FLT x87 = x67 * x21;
	const FLT x88 = x87 * x26;
	const FLT x89 = x67 * x59;
	const FLT x90 = x79 * x21;
	const FLT x91 = x55 * _x00;
	const FLT x92 = x79 * x52;
	const FLT x93 = x54 * x39;
	const FLT x94 = x52 * x13;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, 0,
						(x58 * _x03) + (-1 * x56) + (x49 * x35) + (x47 * x27) + (-1 * x60 * x46) + x51 + (x53 * x52) +
							(-1 * x54 * x44));
	cnMatrixOptionalSet(Hx, 0, 1,
						(-1 * x66 * x54) + (-1 * x60 * x67) + (-1 * x70 * x64) + (x68 * x27) + (x72 * _x03) +
							(x73 * x52) + (-1 * x62 * x50) + (x69 * _x00));
	cnMatrixOptionalSet(Hx, 0, 2,
						(-1 * x79 * x60) + (-1 * x78 * x54) + (x75 * x49) + (x82 * x27) + x74 + (-1 * x76) +
							(x80 * x77) + (x81 * x52));
	cnMatrixOptionalSet(Hx, 1, 0,
						(-1 * x86 * x13) + (-1 * x52 * x57) + (x50 * x35) + x85 + (-1 * x83 * x27) + (-1 * x70 * x44) +
							(x77 * x53) + x84);
	cnMatrixOptionalSet(Hx, 1, 1,
						(x64 * x54) + (-1 * x71 * x52) + (x62 * x49) + (-1 * x88 * _x01) + (x69 * _x01) +
							(-1 * x89 * x13) + (-1 * x70 * x66) + (x73 * x77));
	cnMatrixOptionalSet(Hx, 1, 2,
						(-1 * x82 * x59) + x93 + (-1 * x92 * x24) + (-1 * x90 * x27) + (x75 * x50) + x91 +
							(-1 * x70 * x78) + (x81 * x77));
	cnMatrixOptionalSet(Hx, 2, 0,
						x76 + (-1 * x83 * x52) + (x44 * x49) + (x54 * x35) + (x58 * _x01) + (-1 * x86 * x17) + x74 +
							(-1 * x77 * x47));
	cnMatrixOptionalSet(Hx, 2, 1,
						(x69 * _x02) + (x70 * x62) + (x66 * x49) + (-1 * x87 * x52) + (-1 * x64 * x50) +
							(-1 * x89 * x17) + (-1 * x77 * x68) + (x72 * _x01));
	cnMatrixOptionalSet(Hx, 2, 2,
						(-1 * x81 * x59) + x56 + (-1 * x82 * x77) + (-1 * x92 * x21) + (x78 * x49) + x51 + (x75 * x54) +
							(x80 * x27));
	cnMatrixOptionalSet(Hx, 3, 0,
						(x70 * x35) + x91 + (x50 * x44) + (-1 * x53 * x27) + (-1 * x57 * x59) + (x94 * x46) +
							(-1 * x93) + (-1 * x83 * x77));
	cnMatrixOptionalSet(Hx, 3, 1,
						(x69 * _x03) + (-1 * x88 * _x03) + (-1 * x71 * x59) + (x64 * x49) + (-1 * x73 * x27) +
							(-1 * x62 * x54) + (x67 * x94) + (x66 * x50));
	cnMatrixOptionalSet(Hx, 3, 2,
						(x82 * x52) + (x70 * x75) + (-1 * x81 * x27) + (x78 * x50) + (-1 * x80 * x59) + (-1 * x84) +
							x85 + (-1 * x77 * x90));
}

// Full version Jacobian of GenerateQuatModel wrt [error_state0, error_state1, error_state2]

static inline void gen_GenerateQuatModel_jac_error_state_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x0,
																 const FLT *error_state) {
	if (hx != 0) {
		gen_GenerateQuatModel(hx, _x0, error_state);
	}
	if (Hx != 0) {
		gen_GenerateQuatModel_jac_error_state(Hx, _x0, error_state);
	}
}
static inline void gen_GenerateQuatErrorModelApprox(CnMat *out, const FLT *_x1, const FLT *_x0) {
	const FLT _x10 = _x1[0];
	const FLT _x11 = _x1[1];
	const FLT _x12 = _x1[2];
	const FLT _x13 = _x1[3];
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT x0 = (_x02 * _x11) + (-1 * _x01 * _x12) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x1 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x2 = 2 * x1;
	const FLT x3 = (_x03 * _x12) + (-1 * _x02 * _x13) + (-1 * _x01 * _x10) + (_x00 * _x11);
	const FLT x4 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x5 = 2 * x4;
	const FLT x6 = 1 + (-2 * (x1 * x1));
	cnSetZero(out);
	cnMatrixOptionalSet(out, 0, 0, atan2((x3 * x5) + (x0 * x2), x6 + (-2 * (x3 * x3))));
	cnMatrixOptionalSet(out, 1, 0, asin(2 * ((x1 * x4) + (-1 * x0 * x3))));
	cnMatrixOptionalSet(out, 2, 0, atan2((x0 * x5) + (x2 * x3), x6 + (-2 * (x0 * x0))));
}

// Jacobian of GenerateQuatErrorModelApprox wrt [_x10, _x11, _x12, _x13]
static inline void gen_GenerateQuatErrorModelApprox_jac_x1(CnMat *Hx, const FLT *_x1, const FLT *_x0) {
	const FLT _x10 = _x1[0];
	const FLT _x11 = _x1[1];
	const FLT _x12 = _x1[2];
	const FLT _x13 = _x1[3];
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT x0 = (_x03 * _x12) + (-1 * _x02 * _x13) + (-1 * _x01 * _x10) + (_x00 * _x11);
	const FLT x1 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x2 = 1 + (-2 * (x1 * x1));
	const FLT x3 = x2 + (-2 * (x0 * x0));
	const FLT x4 = 1. / x3;
	const FLT x5 = x1 * _x03;
	const FLT x6 = 2 * x5;
	const FLT x7 = (_x02 * _x11) + (-1 * _x01 * _x12) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x8 = x7 * _x02;
	const FLT x9 = 2 * x8;
	const FLT x10 = x0 * _x00;
	const FLT x11 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x12 = x11 * _x01;
	const FLT x13 = (-2 * x12) + (2 * x10);
	const FLT x14 = x0 * _x01;
	const FLT x15 = x1 * _x02;
	const FLT x16 = 4 * x15;
	const FLT x17 = x3 * x3;
	const FLT x18 = 2 * x1;
	const FLT x19 = 2 * x11;
	const FLT x20 = (x0 * x19) + (x7 * x18);
	const FLT x21 = x20 * (1. / x17);
	const FLT x22 = x17 * (1. / (x17 + (x20 * x20)));
	const FLT x23 = 2 * x15;
	const FLT x24 = x7 * _x03;
	const FLT x25 = 2 * x24;
	const FLT x26 = x11 * _x00;
	const FLT x27 = (2 * x14) + (2 * x26);
	const FLT x28 = 4 * x5;
	const FLT x29 = x0 * _x02;
	const FLT x30 = 2 * x29;
	const FLT x31 = x11 * _x03;
	const FLT x32 = 2 * x31;
	const FLT x33 = x7 * _x00;
	const FLT x34 = x1 * _x01;
	const FLT x35 = (-2 * x34) + (2 * x33);
	const FLT x36 = x0 * _x03;
	const FLT x37 = x1 * _x00;
	const FLT x38 = -4 * x37;
	const FLT x39 = x11 * _x02;
	const FLT x40 = 2 * x39;
	const FLT x41 = 2 * x36;
	const FLT x42 = x7 * _x01;
	const FLT x43 = (2 * x37) + (2 * x42);
	const FLT x44 = -4 * x34;
	const FLT x45 = 2 * (1. / sqrt(1 + (-4 * (((x1 * x11) + (-1 * x0 * x7)) * ((x1 * x11) + (-1 * x0 * x7))))));
	const FLT x46 = x2 + (-2 * (x7 * x7));
	const FLT x47 = 1. / x46;
	const FLT x48 = x46 * x46;
	const FLT x49 = (x7 * x19) + (x0 * x18);
	const FLT x50 = (1. / x48) * x49;
	const FLT x51 = x48 * (1. / (x48 + (x49 * x49)));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, 0, x22 * ((-1 * (x16 + (4 * x14)) * x21) + (x4 * (x13 + (-1 * x6) + (-1 * x9)))));
	cnMatrixOptionalSet(Hx, 0, 1, x22 * ((-1 * (x28 + (-4 * x10)) * x21) + (x4 * (x27 + x23 + (-1 * x25)))));
	cnMatrixOptionalSet(Hx, 0, 2, x22 * ((-1 * (x38 + (-4 * x36)) * x21) + (x4 * (x35 + x30 + x32))));
	cnMatrixOptionalSet(Hx, 0, 3, x22 * ((-1 * (x44 + (4 * x29)) * x21) + (x4 * (x43 + (-1 * x40) + x41))));
	cnMatrixOptionalSet(Hx, 1, 0, (x42 + x36 + (-1 * x39) + x37) * x45);
	cnMatrixOptionalSet(Hx, 1, 1, ((-1 * x33) + (-1 * x29) + (-1 * x31) + x34) * x45);
	cnMatrixOptionalSet(Hx, 1, 2, ((-1 * x24) + x14 + x26 + x15) * x45);
	cnMatrixOptionalSet(Hx, 1, 3, (x8 + x5 + x12 + (-1 * x10)) * x45);
	cnMatrixOptionalSet(Hx, 2, 0, x51 * ((-1 * ((4 * x24) + x16) * x50) + (x47 * (x35 + (-1 * x32) + (-1 * x30)))));
	cnMatrixOptionalSet(Hx, 2, 1, x51 * ((-1 * x50 * ((-4 * x8) + x28)) + (x47 * (x43 + x40 + (-1 * x41)))));
	cnMatrixOptionalSet(Hx, 2, 2, x51 * ((-1 * ((4 * x42) + x38) * x50) + (x47 * (x13 + x9 + x6))));
	cnMatrixOptionalSet(Hx, 2, 3, x51 * ((-1 * ((-4 * x33) + x44) * x50) + (x47 * (x27 + x25 + (-1 * x23)))));
}

// Full version Jacobian of GenerateQuatErrorModelApprox wrt [_x10, _x11, _x12, _x13]

static inline void gen_GenerateQuatErrorModelApprox_jac_x1_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x1,
																   const FLT *_x0) {
	if (hx != 0) {
		gen_GenerateQuatErrorModelApprox(hx, _x1, _x0);
	}
	if (Hx != 0) {
		gen_GenerateQuatErrorModelApprox_jac_x1(Hx, _x1, _x0);
	}
}
// Jacobian of GenerateQuatErrorModelApprox wrt [_x00, _x01, _x02, _x03]
static inline void gen_GenerateQuatErrorModelApprox_jac_x0(CnMat *Hx, const FLT *_x1, const FLT *_x0) {
	const FLT _x10 = _x1[0];
	const FLT _x11 = _x1[1];
	const FLT _x12 = _x1[2];
	const FLT _x13 = _x1[3];
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT x0 = (_x03 * _x12) + (-1 * _x02 * _x13) + (-1 * _x01 * _x10) + (_x00 * _x11);
	const FLT x1 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x2 = 1 + (-2 * (x1 * x1));
	const FLT x3 = x2 + (-2 * (x0 * x0));
	const FLT x4 = 1. / x3;
	const FLT x5 = x0 * _x10;
	const FLT x6 = 2 * x5;
	const FLT x7 = x1 * _x13;
	const FLT x8 = 2 * x7;
	const FLT x9 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x10 = x9 * _x11;
	const FLT x11 = (_x02 * _x11) + (-1 * _x01 * _x12) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x12 = x11 * _x12;
	const FLT x13 = (2 * x12) + (2 * x10);
	const FLT x14 = x0 * _x11;
	const FLT x15 = x1 * _x12;
	const FLT x16 = -4 * x15;
	const FLT x17 = x3 * x3;
	const FLT x18 = 2 * x1;
	const FLT x19 = 2 * x9;
	const FLT x20 = (x0 * x19) + (x11 * x18);
	const FLT x21 = x20 * (1. / x17);
	const FLT x22 = x17 * (1. / (x17 + (x20 * x20)));
	const FLT x23 = 2 * x14;
	const FLT x24 = 2 * x15;
	const FLT x25 = x11 * _x13;
	const FLT x26 = x9 * _x10;
	const FLT x27 = (-2 * x26) + (2 * x25);
	const FLT x28 = -4 * x7;
	const FLT x29 = x9 * _x13;
	const FLT x30 = 2 * x29;
	const FLT x31 = x11 * _x10;
	const FLT x32 = 2 * x31;
	const FLT x33 = x1 * _x11;
	const FLT x34 = x0 * _x12;
	const FLT x35 = (2 * x34) + (2 * x33);
	const FLT x36 = x0 * _x13;
	const FLT x37 = x1 * _x10;
	const FLT x38 = 4 * x37;
	const FLT x39 = x9 * _x12;
	const FLT x40 = 2 * x39;
	const FLT x41 = x11 * _x11;
	const FLT x42 = 2 * x41;
	const FLT x43 = (-2 * x37) + (2 * x36);
	const FLT x44 = 4 * x33;
	const FLT x45 = 2 * (1. / sqrt(1 + (-4 * (((x1 * x9) + (-1 * x0 * x11)) * ((x1 * x9) + (-1 * x0 * x11))))));
	const FLT x46 = x2 + (-2 * (x11 * x11));
	const FLT x47 = 1. / x46;
	const FLT x48 = x46 * x46;
	const FLT x49 = (x11 * x19) + (x0 * x18);
	const FLT x50 = (1. / x48) * x49;
	const FLT x51 = x48 * (1. / (x48 + (x49 * x49)));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, 0, x22 * ((-1 * (x16 + (-4 * x14)) * x21) + (x4 * (x6 + x13 + x8))));
	cnMatrixOptionalSet(Hx, 0, 1, x22 * ((-1 * x21 * (x28 + (4 * x5))) + (x4 * (x23 + x27 + (-1 * x24)))));
	cnMatrixOptionalSet(Hx, 0, 2, x22 * ((-1 * (x38 + (4 * x36)) * x21) + (x4 * (x35 + (-1 * x30) + (-1 * x32)))));
	cnMatrixOptionalSet(Hx, 0, 3, x22 * ((-1 * (x44 + (-4 * x34)) * x21) + (x4 * (x43 + x40 + (-1 * x42)))));
	cnMatrixOptionalSet(Hx, 1, 0, ((-1 * x41) + (-1 * x36) + x39 + x37) * x45);
	cnMatrixOptionalSet(Hx, 1, 1, (x34 + x31 + x29 + x33) * x45);
	cnMatrixOptionalSet(Hx, 1, 2, (x25 + (-1 * x14) + (-1 * x26) + x15) * x45);
	cnMatrixOptionalSet(Hx, 1, 3, (x5 + (-1 * x10) + (-1 * x12) + x7) * x45);
	cnMatrixOptionalSet(Hx, 2, 0, x51 * ((-1 * ((-4 * x25) + x16) * x50) + (x47 * (x35 + x30 + x32))));
	cnMatrixOptionalSet(Hx, 2, 1, x51 * ((-1 * ((4 * x12) + x28) * x50) + (x47 * (x43 + x42 + (-1 * x40)))));
	cnMatrixOptionalSet(Hx, 2, 2, x51 * ((-1 * ((-4 * x41) + x38) * x50) + (x47 * (x13 + (-1 * x8) + (-1 * x6)))));
	cnMatrixOptionalSet(Hx, 2, 3, x51 * ((-1 * ((4 * x31) + x44) * x50) + (x47 * (x27 + x24 + (-1 * x23)))));
}

// Full version Jacobian of GenerateQuatErrorModelApprox wrt [_x00, _x01, _x02, _x03]

static inline void gen_GenerateQuatErrorModelApprox_jac_x0_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x1,
																   const FLT *_x0) {
	if (hx != 0) {
		gen_GenerateQuatErrorModelApprox(hx, _x1, _x0);
	}
	if (Hx != 0) {
		gen_GenerateQuatErrorModelApprox_jac_x0(Hx, _x1, _x0);
	}
}
static inline void gen_GenerateQuatModelApprox(CnMat *out, const FLT *_x0, const FLT *error_state) {
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
	cnSetZero(out);
	cnMatrixOptionalSet(out, 0, 0,
						(-1 * x2 * error_state0) + (-1 * x1 * error_state2) + (-1 * x0 * error_state1) + _x00);
	cnMatrixOptionalSet(out, 1, 0, (x3 * error_state0) + _x01 + (-1 * x1 * error_state1) + (x0 * error_state2));
	cnMatrixOptionalSet(out, 2, 0, (x3 * error_state1) + _x02 + (-1 * x2 * error_state2) + (x1 * error_state0));
	cnMatrixOptionalSet(out, 3, 0, (x3 * error_state2) + (-1 * x0 * error_state0) + _x03 + (x2 * error_state1));
}

// Jacobian of GenerateQuatModelApprox wrt [_x00, _x01, _x02, _x03]
static inline void gen_GenerateQuatModelApprox_jac_x0(CnMat *Hx, const FLT *_x0, const FLT *error_state) {
	const FLT error_state0 = error_state[0];
	const FLT error_state1 = error_state[1];
	const FLT error_state2 = error_state[2];
	const FLT x0 = 0.5 * error_state0;
	const FLT x1 = -1 * x0;
	const FLT x2 = 0.5 * error_state1;
	const FLT x3 = -1 * x2;
	const FLT x4 = 0.5 * error_state2;
	const FLT x5 = -1 * x4;
	cnSetZero(Hx);
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

static inline void gen_GenerateQuatModelApprox_jac_x0_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x0,
															  const FLT *error_state) {
	if (hx != 0) {
		gen_GenerateQuatModelApprox(hx, _x0, error_state);
	}
	if (Hx != 0) {
		gen_GenerateQuatModelApprox_jac_x0(Hx, _x0, error_state);
	}
}
// Jacobian of GenerateQuatModelApprox wrt [error_state0, error_state1, error_state2]
static inline void gen_GenerateQuatModelApprox_jac_error_state(CnMat *Hx, const FLT *_x0, const FLT *error_state) {
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
	cnSetZero(Hx);
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

static inline void gen_GenerateQuatModelApprox_jac_error_state_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x0,
																	   const FLT *error_state) {
	if (hx != 0) {
		gen_GenerateQuatModelApprox(hx, _x0, error_state);
	}
	if (Hx != 0) {
		gen_GenerateQuatModelApprox_jac_error_state(Hx, _x0, error_state);
	}
}
static inline void gen_SurviveKalmanModelToErrorModel(SurviveKalmanErrorModel *out, const SurviveKalmanModel *_x1,
													  const SurviveKalmanModel *_x0) {
	const FLT x0 = ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[2]) + (-1 * (*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[1]) +
				   (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[3]) + ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[0]);
	const FLT x1 = ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[0]) +
				   (-1 * (*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[3]) + (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[2]);
	const FLT x2 = 2 * x1;
	const FLT x3 = ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[3]) + (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[1]) +
				   ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[0]) + (-1 * (*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[2]);
	const FLT x4 = ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[3]) +
				   ((*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[0]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[2]);
	const FLT x5 = 2 * x4;
	const FLT x6 = 1 + (-2 * (x1 * x1));
	out->Pose.Pos[0] = (*_x1).Pose.Pos[0] + (-1 * (*_x0).Pose.Pos[0]);
	out->Pose.Pos[1] = (*_x1).Pose.Pos[1] + (-1 * (*_x0).Pose.Pos[1]);
	out->Pose.Pos[2] = (*_x1).Pose.Pos[2] + (-1 * (*_x0).Pose.Pos[2]);
	out->Pose.AxisAngleRot[0] = atan2((x3 * x5) + (x0 * x2), x6 + (-2 * (x3 * x3)));
	out->Pose.AxisAngleRot[1] = asin(2 * ((x1 * x4) + (-1 * x0 * x3)));
	out->Pose.AxisAngleRot[2] = atan2((x0 * x5) + (x2 * x3), x6 + (-2 * (x0 * x0)));
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
// <cnkalman.codegen.WrapMember object at 0x7fe05a4916a0>]
static inline void gen_SurviveKalmanModelToErrorModel_jac_x1(CnMat *Hx, const SurviveKalmanModel *_x1,
															 const SurviveKalmanModel *_x0) {
	const FLT x0 = ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[3]) + (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[1]) +
				   ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[0]) + (-1 * (*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[2]);
	const FLT x1 = ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[0]) +
				   (-1 * (*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[3]) + (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[2]);
	const FLT x2 = 1 + (-2 * (x1 * x1));
	const FLT x3 = x2 + (-2 * (x0 * x0));
	const FLT x4 = 1. / x3;
	const FLT x5 = x1 * (*_x0).Pose.Rot[3];
	const FLT x6 = 2 * x5;
	const FLT x7 = ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[2]) + (-1 * (*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[1]) +
				   (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[3]) + ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[0]);
	const FLT x8 = x7 * (*_x0).Pose.Rot[2];
	const FLT x9 = 2 * x8;
	const FLT x10 = ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[3]) +
					((*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[0]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[2]);
	const FLT x11 = x10 * (*_x0).Pose.Rot[1];
	const FLT x12 = x0 * (*_x0).Pose.Rot[0];
	const FLT x13 = (2 * x12) + (-2 * x11);
	const FLT x14 = x0 * (*_x0).Pose.Rot[1];
	const FLT x15 = x1 * (*_x0).Pose.Rot[2];
	const FLT x16 = 4 * x15;
	const FLT x17 = x3 * x3;
	const FLT x18 = 2 * x1;
	const FLT x19 = 2 * x10;
	const FLT x20 = (x0 * x19) + (x7 * x18);
	const FLT x21 = x20 * (1. / x17);
	const FLT x22 = x17 * (1. / (x17 + (x20 * x20)));
	const FLT x23 = 2 * x15;
	const FLT x24 = x7 * (*_x0).Pose.Rot[3];
	const FLT x25 = 2 * x24;
	const FLT x26 = x10 * (*_x0).Pose.Rot[0];
	const FLT x27 = (2 * x14) + (2 * x26);
	const FLT x28 = 4 * x5;
	const FLT x29 = x0 * (*_x0).Pose.Rot[2];
	const FLT x30 = 2 * x29;
	const FLT x31 = x10 * (*_x0).Pose.Rot[3];
	const FLT x32 = 2 * x31;
	const FLT x33 = x1 * (*_x0).Pose.Rot[1];
	const FLT x34 = x7 * (*_x0).Pose.Rot[0];
	const FLT x35 = (2 * x34) + (-2 * x33);
	const FLT x36 = x0 * (*_x0).Pose.Rot[3];
	const FLT x37 = x1 * (*_x0).Pose.Rot[0];
	const FLT x38 = -4 * x37;
	const FLT x39 = x10 * (*_x0).Pose.Rot[2];
	const FLT x40 = 2 * x39;
	const FLT x41 = 2 * x36;
	const FLT x42 = x7 * (*_x0).Pose.Rot[1];
	const FLT x43 = (2 * x37) + (2 * x42);
	const FLT x44 = -4 * x33;
	const FLT x45 = 2 * (1. / sqrt(1 + (-4 * (((x1 * x10) + (-1 * x0 * x7)) * ((x1 * x10) + (-1 * x0 * x7))))));
	const FLT x46 = x2 + (-2 * (x7 * x7));
	const FLT x47 = 1. / x46;
	const FLT x48 = x46 * x46;
	const FLT x49 = (x7 * x19) + (x0 * x18);
	const FLT x50 = (1. / x48) * x49;
	const FLT x51 = x48 * (1. / (x48 + (x49 * x49)));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						x22 * ((-1 * (x16 + (4 * x14)) * x21) + (x4 * (x13 + (-1 * x6) + (-1 * x9)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						x22 * ((-1 * (x28 + (-4 * x12)) * x21) + (x4 * (x27 + x23 + (-1 * x25)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x22 * ((-1 * (x38 + (-4 * x36)) * x21) + (x4 * (x35 + x30 + x32))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						x22 * ((-1 * (x44 + (4 * x29)) * x21) + (x4 * (x43 + (-1 * x40) + x41))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), (x42 + x37 + (-1 * x39) + x36) * x45);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						((-1 * x34) + x33 + (-1 * x31) + (-1 * x29)) * x45);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), ((-1 * x24) + x14 + x26 + x15) * x45);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), (x8 + (-1 * x12) + x11 + x5) * x45);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						x51 * ((-1 * ((4 * x24) + x16) * x50) + (x47 * (x35 + (-1 * x32) + (-1 * x30)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						x51 * ((-1 * x50 * ((-4 * x8) + x28)) + (x47 * (x43 + x40 + (-1 * x41)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x51 * ((-1 * ((4 * x42) + x38) * x50) + (x47 * (x13 + x9 + x6))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						x51 * ((-1 * ((-4 * x34) + x44) * x50) + (x47 * (x27 + x25 + (-1 * x23)))));
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
// <cnkalman.codegen.WrapMember object at 0x7fe05a4916a0>] Jacobian of SurviveKalmanModelToErrorModel wrt
// [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2], (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2],
// (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1],
// (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3], (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2],
// (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1], (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0],
// (*_x0).Velocity.AxisAngleRot[1], (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1],
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7fe05a495490>]
static inline void gen_SurviveKalmanModelToErrorModel_jac_x0(CnMat *Hx, const SurviveKalmanModel *_x1,
															 const SurviveKalmanModel *_x0) {
	const FLT x0 = ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[3]) + (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[1]) +
				   ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[0]) + (-1 * (*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[2]);
	const FLT x1 = ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[0]) +
				   (-1 * (*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[3]) + (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[2]);
	const FLT x2 = 1 + (-2 * (x1 * x1));
	const FLT x3 = x2 + (-2 * (x0 * x0));
	const FLT x4 = 1. / x3;
	const FLT x5 = x0 * (*_x1).Pose.Rot[0];
	const FLT x6 = 2 * x5;
	const FLT x7 = x1 * (*_x1).Pose.Rot[3];
	const FLT x8 = 2 * x7;
	const FLT x9 = ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[3]) +
				   ((*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[0]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[2]);
	const FLT x10 = x9 * (*_x1).Pose.Rot[1];
	const FLT x11 = ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[2]) + (-1 * (*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[1]) +
					(-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[3]) + ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[0]);
	const FLT x12 = x11 * (*_x1).Pose.Rot[2];
	const FLT x13 = (2 * x12) + (2 * x10);
	const FLT x14 = x0 * (*_x1).Pose.Rot[1];
	const FLT x15 = x1 * (*_x1).Pose.Rot[2];
	const FLT x16 = -4 * x15;
	const FLT x17 = x3 * x3;
	const FLT x18 = 2 * x1;
	const FLT x19 = 2 * x9;
	const FLT x20 = (x0 * x19) + (x11 * x18);
	const FLT x21 = x20 * (1. / x17);
	const FLT x22 = x17 * (1. / (x17 + (x20 * x20)));
	const FLT x23 = 2 * x14;
	const FLT x24 = 2 * x15;
	const FLT x25 = x9 * (*_x1).Pose.Rot[0];
	const FLT x26 = x11 * (*_x1).Pose.Rot[3];
	const FLT x27 = (2 * x26) + (-2 * x25);
	const FLT x28 = -4 * x7;
	const FLT x29 = x9 * (*_x1).Pose.Rot[3];
	const FLT x30 = 2 * x29;
	const FLT x31 = x11 * (*_x1).Pose.Rot[0];
	const FLT x32 = 2 * x31;
	const FLT x33 = x0 * (*_x1).Pose.Rot[2];
	const FLT x34 = x1 * (*_x1).Pose.Rot[1];
	const FLT x35 = (2 * x34) + (2 * x33);
	const FLT x36 = x0 * (*_x1).Pose.Rot[3];
	const FLT x37 = x1 * (*_x1).Pose.Rot[0];
	const FLT x38 = 4 * x37;
	const FLT x39 = x9 * (*_x1).Pose.Rot[2];
	const FLT x40 = 2 * x39;
	const FLT x41 = x11 * (*_x1).Pose.Rot[1];
	const FLT x42 = 2 * x41;
	const FLT x43 = (-2 * x37) + (2 * x36);
	const FLT x44 = 4 * x34;
	const FLT x45 = 2 * (1. / sqrt(1 + (-4 * (((x1 * x9) + (-1 * x0 * x11)) * ((x1 * x9) + (-1 * x0 * x11))))));
	const FLT x46 = x2 + (-2 * (x11 * x11));
	const FLT x47 = 1. / x46;
	const FLT x48 = x46 * x46;
	const FLT x49 = (x11 * x19) + (x0 * x18);
	const FLT x50 = (1. / x48) * x49;
	const FLT x51 = x48 * (1. / (x48 + (x49 * x49)));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						x22 * ((-1 * (x16 + (-4 * x14)) * x21) + (x4 * (x6 + x13 + x8))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						x22 * ((-1 * x21 * (x28 + (4 * x5))) + (x4 * (x23 + x27 + (-1 * x24)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x22 * ((-1 * (x38 + (4 * x36)) * x21) + (x4 * (x35 + (-1 * x30) + (-1 * x32)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						x22 * ((-1 * (x44 + (-4 * x33)) * x21) + (x4 * (x43 + x40 + (-1 * x42)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						((-1 * x41) + (-1 * x36) + x39 + x37) * x45);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), (x33 + x34 + x31 + x29) * x45);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						((-1 * x25) + x26 + x15 + (-1 * x14)) * x45);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						(x5 + (-1 * x10) + (-1 * x12) + x7) * x45);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						x51 * ((-1 * ((-4 * x26) + x16) * x50) + (x47 * (x35 + x32 + x30))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						x51 * ((-1 * ((4 * x12) + x28) * x50) + (x47 * (x43 + x42 + (-1 * x40)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x51 * ((-1 * ((-4 * x41) + x38) * x50) + (x47 * (x13 + (-1 * x8) + (-1 * x6)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						x51 * ((-1 * ((4 * x31) + x44) * x50) + (x47 * (x27 + x24 + (-1 * x23)))));
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
// <cnkalman.codegen.WrapMember object at 0x7fe05a495490>]
static inline void gen_SurviveKalmanModelAddErrorModel(SurviveKalmanModel *out, const SurviveKalmanModel *_x0,
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
		(-1 * x0 * (*_x0).Pose.Rot[1]) + (x1 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2] + (x2 * (*_x0).Pose.Rot[3]);
	out->Pose.Rot[3] =
		(x0 * (*_x0).Pose.Rot[0]) + (x1 * (*_x0).Pose.Rot[1]) + (-1 * x2 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[3];
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
// <cnkalman.codegen.WrapMember object at 0x7fe05a42faf0>]
static inline void gen_SurviveKalmanModelAddErrorModel_jac_x0(CnMat *Hx, const SurviveKalmanModel *_x0,
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
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7fe05a42faf0>] Jacobian of
// SurviveKalmanModelAddErrorModel wrt [(*error_state).AccBias[0], (*error_state).AccBias[1], (*error_state).AccBias[2],
// (*error_state).Acc[0], (*error_state).Acc[1], (*error_state).Acc[2], (*error_state).GyroBias[0],
// (*error_state).GyroBias[1], (*error_state).GyroBias[2], (*error_state).IMUCorrection[0],
// (*error_state).IMUCorrection[1], (*error_state).IMUCorrection[2], (*error_state).IMUCorrection[3],
// (*error_state).Pose.AxisAngleRot[0], (*error_state).Pose.AxisAngleRot[1], (*error_state).Pose.AxisAngleRot[2],
// (*error_state).Pose.Pos[0], (*error_state).Pose.Pos[1], (*error_state).Pose.Pos[2],
// (*error_state).Velocity.AxisAngleRot[0], (*error_state).Velocity.AxisAngleRot[1],
// (*error_state).Velocity.AxisAngleRot[2], (*error_state).Velocity.Pos[0], (*error_state).Velocity.Pos[1],
// (*error_state).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7fe05a420880>]
static inline void gen_SurviveKalmanModelAddErrorModel_jac_error_state(CnMat *Hx, const SurviveKalmanModel *_x0,
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
// (*error_state).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7fe05a420880>]
static inline void gen_SurviveKalmanModelPredict(SurviveKalmanModel *out, const FLT t,
												 const SurviveKalmanModel *kalman_model) {
	const FLT x0 = t * t;
	const FLT x1 = 1.0 / 2.0 * x0;
	const FLT x2 = x0 * ((*kalman_model).Velocity.AxisAngleRot[2] * (*kalman_model).Velocity.AxisAngleRot[2]);
	const FLT x3 = x0 * ((*kalman_model).Velocity.AxisAngleRot[0] * (*kalman_model).Velocity.AxisAngleRot[0]);
	const FLT x4 = x0 * ((*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Velocity.AxisAngleRot[1]);
	const FLT x5 = 1e-10 + x4 + x2 + x3;
	const FLT x6 = sqrt(x5);
	const FLT x7 = 0.5 * x6;
	const FLT x8 = sin(x7);
	const FLT x9 = (1. / x5) * (x8 * x8);
	const FLT x10 = cos(x7);
	const FLT x11 = 1. / sqrt((x3 * x9) + (x10 * x10) + (x2 * x9) + (x4 * x9));
	const FLT x12 = t * (1. / x6) * x8 * x11;
	const FLT x13 = x12 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x14 = x12 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x15 = x11 * x10;
	const FLT x16 = x12 * (*kalman_model).Velocity.AxisAngleRot[0];
	out->Pose.Pos[0] =
		(x1 * (*kalman_model).Acc[0]) + (t * (*kalman_model).Velocity.Pos[0]) + (*kalman_model).Pose.Pos[0];
	out->Pose.Pos[1] =
		(t * (*kalman_model).Velocity.Pos[1]) + (x1 * (*kalman_model).Acc[1]) + (*kalman_model).Pose.Pos[1];
	out->Pose.Pos[2] =
		(t * (*kalman_model).Velocity.Pos[2]) + (x1 * (*kalman_model).Acc[2]) + (*kalman_model).Pose.Pos[2];
	out->Pose.Rot[0] = (-1 * x16 * (*kalman_model).Pose.Rot[1]) + (x15 * (*kalman_model).Pose.Rot[0]) +
					   (-1 * x13 * (*kalman_model).Pose.Rot[3]) + (-1 * x14 * (*kalman_model).Pose.Rot[2]);
	out->Pose.Rot[1] = (x15 * (*kalman_model).Pose.Rot[1]) + (-1 * x13 * (*kalman_model).Pose.Rot[2]) +
					   (x16 * (*kalman_model).Pose.Rot[0]) + (x14 * (*kalman_model).Pose.Rot[3]);
	out->Pose.Rot[2] = (x15 * (*kalman_model).Pose.Rot[2]) + (x14 * (*kalman_model).Pose.Rot[0]) +
					   (-1 * x16 * (*kalman_model).Pose.Rot[3]) + (x13 * (*kalman_model).Pose.Rot[1]);
	out->Pose.Rot[3] = (x16 * (*kalman_model).Pose.Rot[2]) + (x15 * (*kalman_model).Pose.Rot[3]) +
					   (x13 * (*kalman_model).Pose.Rot[0]) + (-1 * x14 * (*kalman_model).Pose.Rot[1]);
	out->Velocity.Pos[0] = (*kalman_model).Velocity.Pos[0] + (t * (*kalman_model).Acc[0]);
	out->Velocity.Pos[1] = (*kalman_model).Velocity.Pos[1] + (t * (*kalman_model).Acc[1]);
	out->Velocity.Pos[2] = (*kalman_model).Velocity.Pos[2] + (t * (*kalman_model).Acc[2]);
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
static inline void gen_SurviveKalmanModelPredict_jac_t(CnMat *Hx, const FLT t, const SurviveKalmanModel *kalman_model) {
	const FLT x0 = t * t;
	const FLT x1 = (*kalman_model).Velocity.AxisAngleRot[2] * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x2 = x0 * x1;
	const FLT x3 = (*kalman_model).Velocity.AxisAngleRot[0] * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x4 = x0 * x3;
	const FLT x5 = (*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x6 = x0 * x5;
	const FLT x7 = 1e-10 + x6 + x2 + x4;
	const FLT x8 = sqrt(x7);
	const FLT x9 = 0.5 * x8;
	const FLT x10 = sin(x9);
	const FLT x11 = (1. / x8) * x10;
	const FLT x12 = x11 * (*kalman_model).Pose.Rot[1];
	const FLT x13 = 1. / x7;
	const FLT x14 = x10 * x10;
	const FLT x15 = x14 * x13;
	const FLT x16 = cos(x9);
	const FLT x17 = (x4 * x15) + (x16 * x16) + (x2 * x15) + (x6 * x15);
	const FLT x18 = 1. / sqrt(x17);
	const FLT x19 = x18 * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x20 = 2 * t;
	const FLT x21 = x5 * x20;
	const FLT x22 = x1 * x20;
	const FLT x23 = x3 * x20;
	const FLT x24 = x23 + x21 + x22;
	const FLT x25 = 0.25 * x24;
	const FLT x26 = x25 * x13 * x16;
	const FLT x27 = x18 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x28 = t * x27;
	const FLT x29 = x28 * x26;
	const FLT x30 = 1.0 / 2.0 * t;
	const FLT x31 = (1. / (x7 * sqrt(x7))) * x10;
	const FLT x32 = x31 * x24;
	const FLT x33 = x30 * x32;
	const FLT x34 = x33 * x27;
	const FLT x35 = x11 * (*kalman_model).Pose.Rot[2];
	const FLT x36 = (1. / (x7 * x7)) * x14;
	const FLT x37 = x36 * x24;
	const FLT x38 = 0.5 * x16;
	const FLT x39 = x38 * x24;
	const FLT x40 = x31 * x39;
	const FLT x41 = x6 * x24;
	const FLT x42 =
		(1. / (x17 * sqrt(x17))) * ((-1 * x2 * x37) + (x2 * x40) + (x41 * x31 * x38) + (x23 * x15) + (x4 * x40) +
									(-1 * x4 * x37) + (x21 * x15) + (x22 * x15) + (-1 * x39 * x11) + (-1 * x41 * x36));
	const FLT x43 = x42 * x30;
	const FLT x44 = x43 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x45 = x18 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x46 = t * x45;
	const FLT x47 = x46 * x26;
	const FLT x48 = 1.0 / 2.0 * (*kalman_model).Pose.Rot[3];
	const FLT x49 = t * x42 * x48 * x11;
	const FLT x50 = x48 * x32;
	const FLT x51 = x19 * (*kalman_model).Pose.Rot[1];
	const FLT x52 = t * x26;
	const FLT x53 = x42 * x16;
	const FLT x54 = 1.0 / 2.0 * x53;
	const FLT x55 = x11 * (*kalman_model).Pose.Rot[3];
	const FLT x56 = x11 * (*kalman_model).Pose.Rot[0];
	const FLT x57 = x25 * x18;
	const FLT x58 = x43 * x12;
	const FLT x59 = x52 * x19;
	const FLT x60 = x43 * x35;
	const FLT x61 = x45 * x33;
	const FLT x62 = x56 * x43;
	const FLT x63 = x33 * x19;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[0]) / sizeof(FLT), 0,
						(t * (*kalman_model).Acc[0]) + (*kalman_model).Velocity.Pos[0]);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[1]) / sizeof(FLT), 0,
						(t * (*kalman_model).Acc[1]) + (*kalman_model).Velocity.Pos[1]);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Pos[2]) / sizeof(FLT), 0,
						(t * (*kalman_model).Acc[2]) + (*kalman_model).Velocity.Pos[2]);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), 0,
						(x51 * x33) + (x58 * (*kalman_model).Velocity.AxisAngleRot[0]) + (-1 * x57 * x56) +
							(x34 * (*kalman_model).Pose.Rot[2]) + (x44 * x35) +
							(-1 * x29 * (*kalman_model).Pose.Rot[2]) + (x50 * x46) + (-1 * x12 * x19) +
							(-1 * x47 * (*kalman_model).Pose.Rot[3]) + (-1 * x52 * x51) +
							(x49 * (*kalman_model).Velocity.AxisAngleRot[2]) +
							(-1 * x54 * (*kalman_model).Pose.Rot[0]) + (-1 * x55 * x45) + (-1 * x35 * x27));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), 0,
						(-1 * x54 * (*kalman_model).Pose.Rot[1]) + (x60 * (*kalman_model).Velocity.AxisAngleRot[2]) +
							(-1 * x47 * (*kalman_model).Pose.Rot[2]) +
							(-1 * x62 * (*kalman_model).Velocity.AxisAngleRot[0]) + (-1 * x50 * x28) + (x55 * x27) +
							(-1 * x63 * (*kalman_model).Pose.Rot[0]) + (x59 * (*kalman_model).Pose.Rot[0]) +
							(x61 * (*kalman_model).Pose.Rot[2]) + (x29 * (*kalman_model).Pose.Rot[3]) + (x56 * x19) +
							(-1 * x49 * (*kalman_model).Velocity.AxisAngleRot[1]) + (-1 * x45 * x35) +
							(-1 * x57 * x12));
	cnMatrixOptionalSet(
		Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), 0,
		(-1 * x59 * (*kalman_model).Pose.Rot[3]) + (-1 * x57 * x35) + (-1 * x34 * (*kalman_model).Pose.Rot[0]) +
			(-1 * x56 * x44) + (x49 * (*kalman_model).Velocity.AxisAngleRot[0]) + (x56 * x27) +
			(-1 * x54 * (*kalman_model).Pose.Rot[2]) + (t * x50 * x19) + (-1 * x61 * (*kalman_model).Pose.Rot[1]) +
			(x47 * (*kalman_model).Pose.Rot[1]) + (x29 * (*kalman_model).Pose.Rot[0]) + (x45 * x12) + (-1 * x55 * x19) +
			(-1 * x58 * (*kalman_model).Velocity.AxisAngleRot[2]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), 0,
						(-1 * x27 * x12) + (-1 * x60 * (*kalman_model).Velocity.AxisAngleRot[0]) +
							(x59 * (*kalman_model).Pose.Rot[2]) + (-1 * x61 * (*kalman_model).Pose.Rot[0]) +
							(-1 * x63 * (*kalman_model).Pose.Rot[2]) +
							(-1 * x62 * (*kalman_model).Velocity.AxisAngleRot[2]) + (x35 * x19) + (-1 * x53 * x48) +
							(x56 * x45) + (-1 * x29 * (*kalman_model).Pose.Rot[1]) + (x44 * x12) + (-1 * x57 * x55) +
							(x34 * (*kalman_model).Pose.Rot[1]) + (x47 * (*kalman_model).Pose.Rot[0]));
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
// (*kalman_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7fe05a43eb20>]
static inline void gen_SurviveKalmanModelPredict_jac_kalman_model(CnMat *Hx, const FLT t,
																  const SurviveKalmanModel *kalman_model) {
	const FLT x0 = t * t;
	const FLT x1 = 1.0 / 2.0 * x0;
	const FLT x2 = (*kalman_model).Velocity.AxisAngleRot[2] * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x3 = x0 * x2;
	const FLT x4 = (*kalman_model).Velocity.AxisAngleRot[0] * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x5 = x0 * x4;
	const FLT x6 = (*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x7 = x0 * x6;
	const FLT x8 = 1e-10 + x7 + x3 + x5;
	const FLT x9 = 1. / x8;
	const FLT x10 = sqrt(x8);
	const FLT x11 = 0.5 * x10;
	const FLT x12 = sin(x11);
	const FLT x13 = x12 * x12;
	const FLT x14 = x9 * x13;
	const FLT x15 = cos(x11);
	const FLT x16 = (x5 * x14) + (x15 * x15) + (x3 * x14) + (x7 * x14);
	const FLT x17 = 1. / sqrt(x16);
	const FLT x18 = x15 * x17;
	const FLT x19 = x12 * x17;
	const FLT x20 = 1. / x10;
	const FLT x21 = t * x20;
	const FLT x22 = x21 * x19;
	const FLT x23 = x22 * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x24 = -1 * x23;
	const FLT x25 = x22 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x26 = -1 * x25;
	const FLT x27 = x22 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x28 = -1 * x27;
	const FLT x29 = (*kalman_model).Velocity.AxisAngleRot[0] * (*kalman_model).Pose.Rot[2];
	const FLT x30 = t * t * t;
	const FLT x31 = 0.5 * x9 * x30 * x18;
	const FLT x32 = x31 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x33 = x32 * x29;
	const FLT x34 = x0 * x14;
	const FLT x35 = 2 * x34;
	const FLT x36 = (*kalman_model).Velocity.AxisAngleRot[0] * (*kalman_model).Velocity.AxisAngleRot[0] *
					(*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x37 = t * t * t * t;
	const FLT x38 = (1. / (x8 * x8)) * x13;
	const FLT x39 = 2 * x38;
	const FLT x40 = x37 * x39;
	const FLT x41 = 1.0 * x15;
	const FLT x42 = x41 * x12;
	const FLT x43 = 1. / (x8 * sqrt(x8));
	const FLT x44 = x43 * x37;
	const FLT x45 = x42 * x44;
	const FLT x46 = x2 * x37;
	const FLT x47 = x42 * x43;
	const FLT x48 = x46 * x47;
	const FLT x49 = x40 * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x50 = x0 * x20;
	const FLT x51 = x50 * x42;
	const FLT x52 = (-1 * x51 * (*kalman_model).Velocity.AxisAngleRot[0]) +
					(x6 * x45 * (*kalman_model).Velocity.AxisAngleRot[0]) + (-1 * x6 * x49) + (-1 * x40 * x36) +
					(x35 * (*kalman_model).Velocity.AxisAngleRot[0]) + (x45 * x36) +
					(x48 * (*kalman_model).Velocity.AxisAngleRot[0]) + (-1 * x2 * x49);
	const FLT x53 = 1.0 / 2.0 * (1. / (x16 * sqrt(x16)));
	const FLT x54 = x53 * x21;
	const FLT x55 = x54 * x12;
	const FLT x56 = x52 * x55;
	const FLT x57 = (*kalman_model).Velocity.AxisAngleRot[0] * (*kalman_model).Pose.Rot[1];
	const FLT x58 = x19 * (*kalman_model).Pose.Rot[0];
	const FLT x59 = 0.5 * x50;
	const FLT x60 = x12 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x61 = x60 * x54;
	const FLT x62 = x61 * x52;
	const FLT x63 = x19 * (*kalman_model).Pose.Rot[2];
	const FLT x64 = x43 * x30;
	const FLT x65 = x64 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x66 = x65 * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x67 = x63 * x66;
	const FLT x68 = x53 * x15;
	const FLT x69 = x68 * x52;
	const FLT x70 = x4 * x31;
	const FLT x71 = x19 * (*kalman_model).Pose.Rot[1];
	const FLT x72 = x71 * x21;
	const FLT x73 = -1 * x72;
	const FLT x74 = (*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Pose.Rot[2];
	const FLT x75 = x4 * x64;
	const FLT x76 = x19 * (*kalman_model).Pose.Rot[3];
	const FLT x77 = x76 * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x78 = x64 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x79 = x31 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x80 = (*kalman_model).Velocity.AxisAngleRot[0] * (*kalman_model).Pose.Rot[3];
	const FLT x81 = (-1 * x80 * x79) + (x78 * x77);
	const FLT x82 = x22 * (*kalman_model).Pose.Rot[2];
	const FLT x83 = -1 * x82;
	const FLT x84 = x6 * x64;
	const FLT x85 = x84 * x19;
	const FLT x86 = x79 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x87 = x86 * (*kalman_model).Pose.Rot[3];
	const FLT x88 = 2 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x89 = x88 * x38;
	const FLT x90 = x4 * x37;
	const FLT x91 = x37 * ((*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Velocity.AxisAngleRot[1] *
						   (*kalman_model).Velocity.AxisAngleRot[1]);
	const FLT x92 = (-1 * x91 * x39) + (x88 * x34) + (-1 * x51 * (*kalman_model).Velocity.AxisAngleRot[1]) +
					(x91 * x47) + (-1 * x89 * x90) + (x90 * x47 * (*kalman_model).Velocity.AxisAngleRot[1]) +
					(-1 * x89 * x46) + (x48 * (*kalman_model).Velocity.AxisAngleRot[1]);
	const FLT x93 = x61 * (*kalman_model).Pose.Rot[3];
	const FLT x94 = x6 * x31;
	const FLT x95 = x55 * x92;
	const FLT x96 = x59 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x97 = x65 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x98 = x76 * x97;
	const FLT x99 = x68 * x92;
	const FLT x100 = (-1 * x57 * x32) + (x71 * x66);
	const FLT x101 = x78 * (*kalman_model).Velocity.AxisAngleRot[0];
	const FLT x102 = x71 * x101;
	const FLT x103 = x60 * x41;
	const FLT x104 = x40 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x105 = (*kalman_model).Velocity.AxisAngleRot[2] * (*kalman_model).Velocity.AxisAngleRot[2] *
					 (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x106 = (-1 * x6 * x104) + (-1 * x50 * x103) + (x6 * x44 * x103) + (x90 * x43 * x103) + (-1 * x40 * x105) +
					 (x45 * x105) + (-1 * x4 * x104) + (x35 * (*kalman_model).Velocity.AxisAngleRot[2]);
	const FLT x107 = x55 * x106;
	const FLT x108 = x107 * (*kalman_model).Velocity.AxisAngleRot[1];
	const FLT x109 = x2 * x64;
	const FLT x110 = x22 * (*kalman_model).Pose.Rot[3];
	const FLT x111 = -1 * x110;
	const FLT x112 = x59 * (*kalman_model).Velocity.AxisAngleRot[2];
	const FLT x113 = x68 * x106;
	const FLT x114 = x2 * x31;
	const FLT x115 = x79 * x57;
	const FLT x116 = (x63 * x97) + (-1 * x86 * (*kalman_model).Pose.Rot[2]);
	const FLT x117 = (*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Pose.Rot[3];
	const FLT x118 = (*kalman_model).Velocity.AxisAngleRot[0] * (*kalman_model).Pose.Rot[0];
	const FLT x119 = x80 * x32;
	const FLT x120 = x63 * x101;
	const FLT x121 = x77 * x65;
	const FLT x122 = x79 * x29;
	const FLT x123 = x71 * x59;
	const FLT x124 = x22 * (*kalman_model).Pose.Rot[0];
	const FLT x125 = x61 * (*kalman_model).Pose.Rot[2];
	const FLT x126 = (*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Pose.Rot[0];
	const FLT x127 = (x31 * x126 * (*kalman_model).Velocity.AxisAngleRot[0]) + (-1 * x66 * x58);
	const FLT x128 = (x79 * x118) + (-1 * x58 * x101);
	const FLT x129 = x63 * x59;
	const FLT x130 = x86 * (*kalman_model).Pose.Rot[1];
	const FLT x131 = x71 * x97;
	const FLT x132 = x61 * x92;
	const FLT x133 = x61 * x106;
	const FLT x134 = (x79 * x126) + (-1 * x58 * x97);
	const FLT x135 = (*kalman_model).Velocity.AxisAngleRot[1] * (*kalman_model).Pose.Rot[1];
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
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x24);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x26);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x28);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x81 + x73 + (x71 * x75) + (x57 * x56) + (x74 * x56) + (-1 * x33) +
							(x62 * (*kalman_model).Pose.Rot[3]) +
							(-1 * x58 * x59 * (*kalman_model).Velocity.AxisAngleRot[0]) +
							(-1 * x70 * (*kalman_model).Pose.Rot[1]) + (-1 * x69 * (*kalman_model).Pose.Rot[0]) + x67);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						(x57 * x95) + x98 + (-1 * x58 * x96) + (-1 * x99 * (*kalman_model).Pose.Rot[0]) +
							(x85 * (*kalman_model).Pose.Rot[2]) + x100 + x83 +
							(-1 * x94 * (*kalman_model).Pose.Rot[2]) + (-1 * x87) + (x74 * x95) + (x93 * x92));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x116 + (x57 * x107) + (-1 * x114 * (*kalman_model).Pose.Rot[3]) + (-1 * x115) +
							(-1 * x58 * x112) + (x76 * x109) + (-1 * x113 * (*kalman_model).Pose.Rot[0]) + x102 +
							(x93 * x106) + (x108 * (*kalman_model).Pose.Rot[2]) + x111);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x23);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x28);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x25);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x124 + x120 + (x70 * (*kalman_model).Pose.Rot[0]) + (-1 * x56 * x117) + x119 + (-1 * x121) +
							(-1 * x56 * x118) + (-1 * x122) + (-1 * x123 * (*kalman_model).Velocity.AxisAngleRot[0]) +
							(x62 * (*kalman_model).Pose.Rot[2]) + (-1 * x75 * x58) +
							(-1 * x69 * (*kalman_model).Pose.Rot[1]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x127 + x116 + x110 + (-1 * x95 * x117) + (-1 * x99 * (*kalman_model).Pose.Rot[1]) +
							(x92 * x125) + (x94 * (*kalman_model).Pose.Rot[3]) + (-1 * x95 * x118) +
							(-1 * x85 * (*kalman_model).Pose.Rot[3]) +
							(-1 * x123 * (*kalman_model).Velocity.AxisAngleRot[1]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x128 + (-1 * x107 * x118) + (-1 * x113 * (*kalman_model).Pose.Rot[1]) +
							(-1 * x123 * (*kalman_model).Velocity.AxisAngleRot[2]) +
							(-1 * x114 * (*kalman_model).Pose.Rot[2]) + (x106 * x125) + x87 +
							(-1 * x108 * (*kalman_model).Pose.Rot[3]) + (-1 * x98) + (x63 * x109) + x83);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x25);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x27);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x24);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						(-1 * x129 * (*kalman_model).Velocity.AxisAngleRot[0]) + (-1 * x56 * x126) + (-1 * x102) +
							(-1 * x70 * (*kalman_model).Pose.Rot[3]) + (-1 * x69 * (*kalman_model).Pose.Rot[2]) + x115 +
							(-1 * x62 * (*kalman_model).Pose.Rot[1]) + x127 + (x80 * x56) + x111 + (x75 * x76));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						(x80 * x95) + (-1 * x131) + (-1 * x95 * x126) + (-1 * x99 * (*kalman_model).Pose.Rot[2]) +
							x130 + (-1 * x119) + x121 + (-1 * x129 * (*kalman_model).Velocity.AxisAngleRot[1]) +
							(x94 * (*kalman_model).Pose.Rot[0]) + x124 + (-1 * x85 * (*kalman_model).Pose.Rot[0]) +
							(-1 * x132 * (*kalman_model).Pose.Rot[1]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x134 + (-1 * x63 * x112) + (-1 * x133 * (*kalman_model).Pose.Rot[1]) + (x80 * x107) +
							(-1 * x107 * x126) + (-1 * x113 * (*kalman_model).Pose.Rot[2]) +
							(x114 * (*kalman_model).Pose.Rot[1]) + (-1 * x71 * x109) + x81 + x72);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT), x27);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT), x26);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT), x23);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x128 + (-1 * x69 * (*kalman_model).Pose.Rot[3]) + (-1 * x75 * x63) + x100 + (x56 * x135) +
							(-1 * x62 * (*kalman_model).Pose.Rot[0]) + (x70 * (*kalman_model).Pose.Rot[2]) +
							(-1 * x56 * x29) + x82 + (-1 * x77 * x59));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x134 + (-1 * x67) + (x84 * x71) + (-1 * x95 * x29) + (-1 * x76 * x96) +
							(-1 * x132 * (*kalman_model).Pose.Rot[0]) + (-1 * x99 * (*kalman_model).Pose.Rot[3]) +
							(x95 * x135) + x73 + x33 + (-1 * x94 * (*kalman_model).Pose.Rot[1]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						(-1 * x29 * x107) + (-1 * x120) + (-1 * x130) + (x114 * (*kalman_model).Pose.Rot[0]) +
							(-1 * x58 * x109) + x124 + (-1 * x133 * (*kalman_model).Pose.Rot[0]) + x122 + x131 +
							(-1 * x113 * (*kalman_model).Pose.Rot[3]) + (-1 * x76 * x112) +
							(x108 * (*kalman_model).Pose.Rot[1]));
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
// (*kalman_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7fe05a43eb20>]
static inline void gen_AxisAngleFlip(CnMat *out, const FLT *_x0) {
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT x0 = sqrt(1e-10 + (_x01 * _x01) + (_x02 * _x02) + (_x00 * _x00));
	const FLT x1 = (1. / x0) * (-6.28318530717959 + x0);
	cnSetZero(out);
	cnMatrixOptionalSet(out, 0, 0, x1 * _x00);
	cnMatrixOptionalSet(out, 1, 0, x1 * _x01);
	cnMatrixOptionalSet(out, 2, 0, x1 * _x02);
}

// Jacobian of AxisAngleFlip wrt [_x00, _x01, _x02]
static inline void gen_AxisAngleFlip_jac_x0(CnMat *Hx, const FLT *_x0) {
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT x0 = _x00 * _x00;
	const FLT x1 = _x02 * _x02;
	const FLT x2 = _x01 * _x01;
	const FLT x3 = 1e-10 + x2 + x1 + x0;
	const FLT x4 = sqrt(x3);
	const FLT x5 = -6.28318530717959 + x4;
	const FLT x6 = (1. / (x3 * sqrt(x3))) * x5;
	const FLT x7 = 1. / x3;
	const FLT x8 = (1. / x4) * x5;
	const FLT x9 = _x01 * _x00;
	const FLT x10 = (x7 * x9) + (-1 * x6 * x9);
	const FLT x11 = x6 * _x02;
	const FLT x12 = x7 * _x02;
	const FLT x13 = (x12 * _x00) + (-1 * x11 * _x00);
	const FLT x14 = (x12 * _x01) + (-1 * x11 * _x01);
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, 0, x8 + (-1 * x0 * x6) + (x0 * x7));
	cnMatrixOptionalSet(Hx, 0, 1, x10);
	cnMatrixOptionalSet(Hx, 0, 2, x13);
	cnMatrixOptionalSet(Hx, 1, 0, x10);
	cnMatrixOptionalSet(Hx, 1, 1, x8 + (-1 * x2 * x6) + (x2 * x7));
	cnMatrixOptionalSet(Hx, 1, 2, x14);
	cnMatrixOptionalSet(Hx, 2, 0, x13);
	cnMatrixOptionalSet(Hx, 2, 1, x14);
	cnMatrixOptionalSet(Hx, 2, 2, x8 + (x1 * x7) + (-1 * x1 * x6));
}

// Full version Jacobian of AxisAngleFlip wrt [_x00, _x01, _x02]

static inline void gen_AxisAngleFlip_jac_x0_with_hx(CnMat *Hx, CnMat *hx, const FLT *_x0) {
	if (hx != 0) {
		gen_AxisAngleFlip(hx, _x0);
	}
	if (Hx != 0) {
		gen_AxisAngleFlip_jac_x0(Hx, _x0);
	}
}
static inline void gen_SurviveKalmanModelErrorPredict(SurviveKalmanErrorModel *out, const FLT t,
													  const SurviveKalmanModel *_x0,
													  const SurviveKalmanErrorModel *error_model) {
	const FLT x0 = (*_x0).Acc[0] + (*error_model).Acc[0];
	const FLT x1 = t * t;
	const FLT x2 = 1.0 / 2.0 * x1;
	const FLT x3 = (*_x0).Acc[1] + (*error_model).Acc[1];
	const FLT x4 = (*_x0).Acc[2] + (*error_model).Acc[2];
	const FLT x5 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x6 = 0.5 * (*_x0).Pose.Rot[1];
	const FLT x7 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x8 = (x7 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[3] + (-1 * x5 * (*_x0).Pose.Rot[2]) +
				   (x6 * (*error_model).Pose.AxisAngleRot[1]);
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x10 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x11 = x1 * (x10 * x10);
	const FLT x12 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x13 = x1 * (x12 * x12);
	const FLT x14 = x1 * (x9 * x9);
	const FLT x15 = 1e-10 + x14 + x11 + x13;
	const FLT x16 = sqrt(x15);
	const FLT x17 = 0.5 * x16;
	const FLT x18 = sin(x17);
	const FLT x19 = (1. / x15) * (x18 * x18);
	const FLT x20 = cos(x17);
	const FLT x21 = 1. / sqrt((x13 * x19) + (x20 * x20) + (x11 * x19) + (x14 * x19));
	const FLT x22 = t * x21 * x18 * (1. / x16);
	const FLT x23 = x9 * x22;
	const FLT x24 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x25 =
		(x5 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[1] + (-1 * x24 * (*_x0).Pose.Rot[3]) + (x7 * (*_x0).Pose.Rot[2]);
	const FLT x26 = x20 * x21;
	const FLT x27 = (x24 * (*_x0).Pose.Rot[0]) + (-1 * x6 * (*error_model).Pose.AxisAngleRot[2]) +
					(x5 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x28 = x22 * x10;
	const FLT x29 = (-1 * x24 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] + (-1 * x5 * (*_x0).Pose.Rot[1]) +
					(-1 * x7 * (*_x0).Pose.Rot[3]);
	const FLT x30 = x22 * x12;
	const FLT x31 = (-1 * x28 * x27) + (x30 * x29) + (x8 * x23) + (x25 * x26);
	const FLT x32 = x8 * x22;
	const FLT x33 = (-1 * x30 * x25) + (x29 * x26) + (-1 * x32 * x10) + (-1 * x23 * x27);
	const FLT x34 = (-1 * x32 * x12) + (x23 * x29) + (x26 * x27) + (x25 * x28);
	const FLT x35 = (-1 * x25 * x23) + (x30 * x27) + (x28 * x29) + (x8 * x26);
	const FLT x36 = (x35 * (*_x0).Pose.Rot[1]) + (-1 * x31 * (*_x0).Pose.Rot[3]) + (x34 * (*_x0).Pose.Rot[0]) +
					(-1 * x33 * (*_x0).Pose.Rot[2]);
	const FLT x37 = (x31 * (*_x0).Pose.Rot[2]) + (x35 * (*_x0).Pose.Rot[0]) + (-1 * x33 * (*_x0).Pose.Rot[3]) +
					(-1 * x34 * (*_x0).Pose.Rot[1]);
	const FLT x38 = 2 * x37;
	const FLT x39 = (x33 * (*_x0).Pose.Rot[0]) + (x35 * (*_x0).Pose.Rot[3]) + (x31 * (*_x0).Pose.Rot[1]) +
					(x34 * (*_x0).Pose.Rot[2]);
	const FLT x40 = (x31 * (*_x0).Pose.Rot[0]) + (-1 * x35 * (*_x0).Pose.Rot[2]) + (x34 * (*_x0).Pose.Rot[3]) +
					(-1 * x33 * (*_x0).Pose.Rot[1]);
	const FLT x41 = 2 * x40;
	const FLT x42 = 1 + (-2 * (x36 * x36));
	out->Pose.Pos[0] =
		(t * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) + (*error_model).Pose.Pos[0] + (x0 * x2);
	out->Pose.Pos[1] =
		(x2 * x3) + (*error_model).Pose.Pos[1] + (t * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1]));
	out->Pose.Pos[2] =
		(x2 * x4) + (*error_model).Pose.Pos[2] + (t * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2]));
	out->Pose.AxisAngleRot[0] = atan2((x41 * x39) + (x36 * x38), x42 + (-2 * (x40 * x40)));
	out->Pose.AxisAngleRot[1] = asin(2 * ((x36 * x39) + (-1 * x40 * x37)));
	out->Pose.AxisAngleRot[2] = atan2((x38 * x39) + (x41 * x36), x42 + (-2 * (x37 * x37)));
	out->Velocity.Pos[0] = (t * x0) + (*error_model).Velocity.Pos[0];
	out->Velocity.Pos[1] = (*error_model).Velocity.Pos[1] + (t * x3);
	out->Velocity.Pos[2] = (t * x4) + (*error_model).Velocity.Pos[2];
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
static inline void gen_SurviveKalmanModelErrorPredict_jac_t(CnMat *Hx, const FLT t, const SurviveKalmanModel *_x0,
															const SurviveKalmanErrorModel *error_model) {
	const FLT x0 = (*_x0).Acc[0] + (*error_model).Acc[0];
	const FLT x1 = (*_x0).Acc[1] + (*error_model).Acc[1];
	const FLT x2 = (*_x0).Acc[2] + (*error_model).Acc[2];
	const FLT x3 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x4 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x5 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x6 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x7 = (-1 * x5 * (*_x0).Pose.Rot[2]) + (-1 * x6 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[0] +
				   (-1 * x4 * (*_x0).Pose.Rot[3]);
	const FLT x8 = t * t;
	const FLT x9 = x3 * x3;
	const FLT x10 = x8 * x9;
	const FLT x11 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x12 = x11 * x11;
	const FLT x13 = x8 * x12;
	const FLT x14 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x15 = x14 * x14;
	const FLT x16 = x8 * x15;
	const FLT x17 = 1e-10 + x16 + x10 + x13;
	const FLT x18 = sqrt(x17);
	const FLT x19 = 0.5 * x18;
	const FLT x20 = sin(x19);
	const FLT x21 = x20 * x20;
	const FLT x22 = 1. / x17;
	const FLT x23 = x22 * x21;
	const FLT x24 = cos(x19);
	const FLT x25 = (x23 * x13) + (x24 * x24) + (x23 * x10) + (x23 * x16);
	const FLT x26 = 1. / sqrt(x25);
	const FLT x27 = x20 * (1. / x18);
	const FLT x28 = x26 * x27;
	const FLT x29 = x7 * x28;
	const FLT x30 = x3 * x29;
	const FLT x31 =
		(x4 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[3] + (-1 * x6 * (*_x0).Pose.Rot[2]) + (x5 * (*_x0).Pose.Rot[1]);
	const FLT x32 = x24 * x26;
	const FLT x33 = x32 * x31;
	const FLT x34 =
		(*_x0).Pose.Rot[1] + (x6 * (*_x0).Pose.Rot[0]) + (-1 * x5 * (*_x0).Pose.Rot[3]) + (x4 * (*_x0).Pose.Rot[2]);
	const FLT x35 = x34 * x28;
	const FLT x36 = x35 * x14;
	const FLT x37 =
		(x5 * (*_x0).Pose.Rot[0]) + (-1 * x4 * (*_x0).Pose.Rot[1]) + (x6 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x38 = x37 * x28;
	const FLT x39 = x38 * x11;
	const FLT x40 = (t * x39) + (-1 * t * x36) + (t * x30) + x33;
	const FLT x41 = x3 * x28;
	const FLT x42 = x41 * x31;
	const FLT x43 = x38 * x14;
	const FLT x44 = x7 * x32;
	const FLT x45 = x35 * x11;
	const FLT x46 = (-1 * t * x45) + x44 + (-1 * t * x42) + (-1 * t * x43);
	const FLT x47 = x31 * x28;
	const FLT x48 = x47 * x14;
	const FLT x49 = x32 * x34;
	const FLT x50 = x41 * x37;
	const FLT x51 = x29 * x11;
	const FLT x52 = (t * x51) + (t * x48) + (-1 * t * x50) + x49;
	const FLT x53 = x47 * x11;
	const FLT x54 = x41 * x34;
	const FLT x55 = x32 * x37;
	const FLT x56 = x29 * x14;
	const FLT x57 = (t * x56) + x55 + (-1 * t * x53) + (t * x54);
	const FLT x58 = (x52 * (*_x0).Pose.Rot[0]) + (x57 * (*_x0).Pose.Rot[3]) + (-1 * x40 * (*_x0).Pose.Rot[2]) +
					(-1 * x46 * (*_x0).Pose.Rot[1]);
	const FLT x59 = (x40 * (*_x0).Pose.Rot[1]) + (x57 * (*_x0).Pose.Rot[0]) + (-1 * x52 * (*_x0).Pose.Rot[3]) +
					(-1 * x46 * (*_x0).Pose.Rot[2]);
	const FLT x60 = 1 + (-2 * (x59 * x59));
	const FLT x61 = x60 + (-2 * (x58 * x58));
	const FLT x62 = x61 * x61;
	const FLT x63 = (x46 * (*_x0).Pose.Rot[0]) + (x40 * (*_x0).Pose.Rot[3]) + (x52 * (*_x0).Pose.Rot[1]) +
					(x57 * (*_x0).Pose.Rot[2]);
	const FLT x64 = 2 * t;
	const FLT x65 = x64 * x15;
	const FLT x66 = x9 * x64;
	const FLT x67 = x64 * x12;
	const FLT x68 = x67 + x65 + x66;
	const FLT x69 = t * x11;
	const FLT x70 = x68 * x69;
	const FLT x71 = 0.25 * x22;
	const FLT x72 = x71 * x70;
	const FLT x73 = x68 * x27;
	const FLT x74 = 0.5 * x24;
	const FLT x75 = x21 * (1. / (x17 * x17));
	const FLT x76 = x75 * x68;
	const FLT x77 = 1. / (x17 * sqrt(x17));
	const FLT x78 = x74 * x77 * x20;
	const FLT x79 = x78 * x68;
	const FLT x80 = x68 * x10;
	const FLT x81 = ((x65 * x23) + (x80 * x78) + (-1 * x76 * x16) + (x79 * x13) + (x67 * x23) + (-1 * x80 * x75) +
					 (-1 * x73 * x74) + (x79 * x16) + (-1 * x76 * x13) + (x66 * x23)) *
					(1. / (x25 * sqrt(x25)));
	const FLT x82 = x81 * x24;
	const FLT x83 = 1.0 / 2.0 * x82;
	const FLT x84 = 0.25 * x73;
	const FLT x85 = x84 * x26;
	const FLT x86 = x81 * x27;
	const FLT x87 = 1.0 / 2.0 * x7;
	const FLT x88 = x86 * x87;
	const FLT x89 = t * x14;
	const FLT x90 = x89 * x68;
	const FLT x91 = x71 * x90;
	const FLT x92 = 1.0 / 2.0 * x31;
	const FLT x93 = x86 * x92;
	const FLT x94 = x77 * x20 * x26;
	const FLT x95 = t * x3;
	const FLT x96 = x68 * x95;
	const FLT x97 = 1.0 / 2.0 * x37;
	const FLT x98 = x97 * x94;
	const FLT x99 = x86 * x97;
	const FLT x100 = x7 * x26;
	const FLT x101 = 1.0 / 2.0 * x77 * x20 * x100;
	const FLT x102 = x71 * x96;
	const FLT x103 = (-1 * x70 * x101) + (-1 * x50) + (-1 * x88 * x69) + (-1 * x85 * x34) + (x98 * x96) +
					 (-1 * x83 * x34) + (x72 * x44) + x51 + (-1 * x89 * x93) + (x91 * x33) + x48 + (-1 * x55 * x102) +
					 (-1 * x92 * x90 * x94) + (x99 * x95);
	const FLT x104 = x89 * x86;
	const FLT x105 = x70 * x94;
	const FLT x106 = 1.0 / 2.0 * x34;
	const FLT x107 = x94 * x106;
	const FLT x108 = x86 * x106;
	const FLT x109 = (-1 * x85 * x37) + (-1 * x83 * x37) + (-1 * x53) + (-1 * x72 * x33) + (-1 * x95 * x108) +
					 (x49 * x102) + (x91 * x44) + x54 + x56 + (-1 * x90 * x101) + (x69 * x93) + (-1 * x87 * x104) +
					 (x92 * x105) + (-1 * x96 * x107);
	const FLT x110 = (-1 * x91 * x49) + (x90 * x107) + (-1 * x85 * x31) + (x104 * x106) + (x44 * x102) + x30 +
					 (-1 * x82 * x92) + (-1 * x36) + (-1 * x69 * x99) + (x72 * x55) + (-1 * x70 * x98) +
					 (-1 * x88 * x95) + x39 + (-1 * x96 * x101);
	const FLT x111 = x92 * x95;
	const FLT x112 = (x105 * x106) + (x90 * x98) + (x69 * x108) + (-1 * x72 * x49) + (x68 * x94 * x111) +
					 (-1 * x7 * x83) + (x86 * x111) + (-1 * x33 * x102) + (x97 * x104) + (-1 * x45) + (-1 * x42) +
					 (-1 * x84 * x100) + (-1 * x43) + (-1 * x55 * x91);
	const FLT x113 = (-1 * x112 * (*_x0).Pose.Rot[1]) + (-1 * x110 * (*_x0).Pose.Rot[2]) + (x103 * (*_x0).Pose.Rot[0]) +
					 (x109 * (*_x0).Pose.Rot[3]);
	const FLT x114 = 2 * x113;
	const FLT x115 = (x109 * (*_x0).Pose.Rot[2]) + (x112 * (*_x0).Pose.Rot[0]) + (x103 * (*_x0).Pose.Rot[1]) +
					 (x110 * (*_x0).Pose.Rot[3]);
	const FLT x116 = 2 * x58;
	const FLT x117 = (-1 * x112 * (*_x0).Pose.Rot[2]) + (-1 * x103 * (*_x0).Pose.Rot[3]) + (x110 * (*_x0).Pose.Rot[1]) +
					 (x109 * (*_x0).Pose.Rot[0]);
	const FLT x118 = (x40 * (*_x0).Pose.Rot[0]) + (x52 * (*_x0).Pose.Rot[2]) + (-1 * x46 * (*_x0).Pose.Rot[3]) +
					 (-1 * x57 * (*_x0).Pose.Rot[1]);
	const FLT x119 = 2 * x118;
	const FLT x120 = (-1 * x109 * (*_x0).Pose.Rot[1]) + (x103 * (*_x0).Pose.Rot[2]) + (-1 * x112 * (*_x0).Pose.Rot[3]) +
					 (x110 * (*_x0).Pose.Rot[0]);
	const FLT x121 = 2 * x120;
	const FLT x122 = (x63 * x116) + (x59 * x119);
	const FLT x123 = -4 * x59 * x117;
	const FLT x124 = x60 + (-2 * (x118 * x118));
	const FLT x125 = x124 * x124;
	const FLT x126 = (x63 * x119) + (x59 * x116);
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT), 0,
						(*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0] + (t * x0));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT), 0,
						(t * x1) + (*error_model).Velocity.Pos[1] + (*_x0).Velocity.Pos[1]);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT), 0,
						(*error_model).Velocity.Pos[2] + (t * x2) + (*_x0).Velocity.Pos[2]);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT), 0,
						x62 * (1. / (x62 + (x122 * x122))) *
							((-1 * (1. / x62) * x122 * (x123 + (-4 * x58 * x113))) +
							 (((x59 * x121) + (x119 * x117) + (x63 * x114) + (x115 * x116)) * (1. / x61))));
	cnMatrixOptionalSet(
		Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT), 0,
		2 * (1. / sqrt(1 + (-4 * (((x63 * x59) + (-1 * x58 * x118)) * ((x63 * x59) + (-1 * x58 * x118)))))) *
			((-1 * x113 * x118) + (-1 * x58 * x120) + (x59 * x115) + (x63 * x117)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT), 0,
						x125 * (1. / (x125 + (x126 * x126))) *
							((-1 * x126 * (1. / x125) * ((-4 * x118 * x120) + x123)) +
							 (((x59 * x114) + (x63 * x121) + (x116 * x117) + (x119 * x115)) * (1. / x124))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[0]) / sizeof(FLT), 0, x0);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[1]) / sizeof(FLT), 0, x1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Velocity.Pos[2]) / sizeof(FLT), 0, x2);
}

// Full version Jacobian of SurviveKalmanModelErrorPredict wrt [t]
// Jacobian of SurviveKalmanModelErrorPredict wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2],
// (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3],
// (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2], (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1],
// (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0], (*_x0).Velocity.AxisAngleRot[1],
// (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1], (*_x0).Velocity.Pos[2],
// <cnkalman.codegen.WrapMember object at 0x7fe05a41dd30>]
static inline void gen_SurviveKalmanModelErrorPredict_jac_x0(CnMat *Hx, const FLT t, const SurviveKalmanModel *_x0,
															 const SurviveKalmanErrorModel *error_model) {
	const FLT x0 = t * t;
	const FLT x1 = 1.0 / 2.0 * x0;
	const FLT x2 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x4 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x5 = 0.5 * (*_x0).Pose.Rot[1];
	const FLT x6 = (-1 * x5 * (*error_model).Pose.AxisAngleRot[0]) + (-1 * x4 * (*_x0).Pose.Rot[2]) +
				   (*_x0).Pose.Rot[0] + (-1 * x3 * (*_x0).Pose.Rot[3]);
	const FLT x7 = x2 * x2;
	const FLT x8 = x0 * x7;
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x10 = x9 * x9;
	const FLT x11 = x0 * x10;
	const FLT x12 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x13 = x12 * x12;
	const FLT x14 = x0 * x13;
	const FLT x15 = 1e-10 + x14 + x8 + x11;
	const FLT x16 = sqrt(x15);
	const FLT x17 = 0.5 * x16;
	const FLT x18 = sin(x17);
	const FLT x19 = x18 * x18;
	const FLT x20 = 1. / x15;
	const FLT x21 = x20 * x19;
	const FLT x22 = cos(x17);
	const FLT x23 = (x21 * x11) + (x22 * x22) + (x8 * x21) + (x21 * x14);
	const FLT x24 = 1. / sqrt(x23);
	const FLT x25 = x24 * x18;
	const FLT x26 = 1. / x16;
	const FLT x27 = t * x26;
	const FLT x28 = x25 * x27;
	const FLT x29 = x6 * x28;
	const FLT x30 = x2 * x29;
	const FLT x31 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x32 = (x3 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[3] + (-1 * x31 * (*_x0).Pose.Rot[2]) +
					(x5 * (*error_model).Pose.AxisAngleRot[1]);
	const FLT x33 = x24 * x22;
	const FLT x34 = x32 * x33;
	const FLT x35 =
		(x31 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[1] + (-1 * x4 * (*_x0).Pose.Rot[3]) + (x3 * (*_x0).Pose.Rot[2]);
	const FLT x36 = x35 * x25;
	const FLT x37 = x36 * x27;
	const FLT x38 = x37 * x12;
	const FLT x39 = (x4 * (*_x0).Pose.Rot[0]) + (-1 * x5 * (*error_model).Pose.AxisAngleRot[2]) +
					(x31 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x40 = x39 * x28;
	const FLT x41 = x9 * x40;
	const FLT x42 = x41 + x30 + (-1 * x38) + x34;
	const FLT x43 = x32 * x28;
	const FLT x44 = x2 * x43;
	const FLT x45 = x40 * x12;
	const FLT x46 = x6 * x33;
	const FLT x47 = x9 * x37;
	const FLT x48 = (-1 * x47) + x46 + (-1 * x44) + (-1 * x45);
	const FLT x49 = x43 * x12;
	const FLT x50 = x33 * x35;
	const FLT x51 = x2 * x40;
	const FLT x52 = x9 * x28;
	const FLT x53 = x6 * x52;
	const FLT x54 = (-1 * x51) + x49 + x53 + x50;
	const FLT x55 = x9 * x43;
	const FLT x56 = x2 * x37;
	const FLT x57 = x33 * x39;
	const FLT x58 = x29 * x12;
	const FLT x59 = x58 + x57 + (-1 * x55) + x56;
	const FLT x60 = (x59 * (*_x0).Pose.Rot[3]) + (-1 * x42 * (*_x0).Pose.Rot[2]) + (x54 * (*_x0).Pose.Rot[0]) +
					(-1 * x48 * (*_x0).Pose.Rot[1]);
	const FLT x61 = (x42 * (*_x0).Pose.Rot[1]) + (x59 * (*_x0).Pose.Rot[0]) + (-1 * x54 * (*_x0).Pose.Rot[3]) +
					(-1 * x48 * (*_x0).Pose.Rot[2]);
	const FLT x62 = 1 + (-2 * (x61 * x61));
	const FLT x63 = x62 + (-2 * (x60 * x60));
	const FLT x64 = 1. / x63;
	const FLT x65 = x31 * x33;
	const FLT x66 = x28 * x12;
	const FLT x67 = x3 * x66;
	const FLT x68 = x2 * x28;
	const FLT x69 = -1 * x4 * x68;
	const FLT x70 = x52 + x69;
	const FLT x71 = x70 + x65 + x67;
	const FLT x72 = x4 * x33;
	const FLT x73 = x68 * x31;
	const FLT x74 = -1 * x3 * x52;
	const FLT x75 = x66 + x74;
	const FLT x76 = x75 + x72 + x73;
	const FLT x77 = x52 * x31;
	const FLT x78 = -1 * x77;
	const FLT x79 = x4 * x66;
	const FLT x80 = -1 * x79;
	const FLT x81 = x3 * x68;
	const FLT x82 = (-1 * x81) + x33;
	const FLT x83 = x82 + x78 + x80;
	const FLT x84 = x3 * x33;
	const FLT x85 = x4 * x52;
	const FLT x86 = -1 * x66 * x31;
	const FLT x87 = x68 + x86;
	const FLT x88 = x87 + x84 + x85;
	const FLT x89 = x48 + (x88 * (*_x0).Pose.Rot[3]) + (x83 * (*_x0).Pose.Rot[0]) + (x71 * (*_x0).Pose.Rot[1]) +
					(x76 * (*_x0).Pose.Rot[2]);
	const FLT x90 = 2 * x60;
	const FLT x91 = x54 + (x71 * (*_x0).Pose.Rot[0]) + (-1 * x88 * (*_x0).Pose.Rot[2]) + (x76 * (*_x0).Pose.Rot[3]) +
					(-1 * x83 * (*_x0).Pose.Rot[1]);
	const FLT x92 = (x54 * (*_x0).Pose.Rot[1]) + (x48 * (*_x0).Pose.Rot[0]) + (x42 * (*_x0).Pose.Rot[3]) +
					(x59 * (*_x0).Pose.Rot[2]);
	const FLT x93 = 2 * x92;
	const FLT x94 = (x54 * (*_x0).Pose.Rot[2]) + (-1 * x48 * (*_x0).Pose.Rot[3]) + (x42 * (*_x0).Pose.Rot[0]) +
					(-1 * x59 * (*_x0).Pose.Rot[1]);
	const FLT x95 = x59 + (-1 * x71 * (*_x0).Pose.Rot[3]) + (x76 * (*_x0).Pose.Rot[0]) +
					(-1 * x83 * (*_x0).Pose.Rot[2]) + (x88 * (*_x0).Pose.Rot[1]);
	const FLT x96 = 2 * x95;
	const FLT x97 = x42 + (x88 * (*_x0).Pose.Rot[0]) + (-1 * x83 * (*_x0).Pose.Rot[3]) + (x71 * (*_x0).Pose.Rot[2]) +
					(-1 * x76 * (*_x0).Pose.Rot[1]);
	const FLT x98 = 2 * x61;
	const FLT x99 = 4 * x60;
	const FLT x100 = 4 * x61;
	const FLT x101 = -1 * x95 * x100;
	const FLT x102 = x63 * x63;
	const FLT x103 = (x60 * x93) + (x98 * x94);
	const FLT x104 = x103 * (1. / x102);
	const FLT x105 = x102 * (1. / (x102 + (x103 * x103)));
	const FLT x106 = -1 * x73;
	const FLT x107 = x74 + (-1 * x66);
	const FLT x108 = x107 + x106 + x72;
	const FLT x109 = -1 * x65;
	const FLT x110 = x69 + (-1 * x52);
	const FLT x111 = x110 + x109 + x67;
	const FLT x112 = -1 * x85;
	const FLT x113 = -1 * x84;
	const FLT x114 = x112 + x87 + x113;
	const FLT x115 = x81 + x33;
	const FLT x116 = x115 + x78 + x79;
	const FLT x117 = (-1 * x46) + x47 + x45 + x44;
	const FLT x118 = x117 + (x116 * (*_x0).Pose.Rot[0]) + (x114 * (*_x0).Pose.Rot[3]) +
					 (-1 * x108 * (*_x0).Pose.Rot[2]) + (-1 * x111 * (*_x0).Pose.Rot[1]);
	const FLT x119 = x54 + (x108 * (*_x0).Pose.Rot[3]) + (x116 * (*_x0).Pose.Rot[1]) + (x111 * (*_x0).Pose.Rot[0]) +
					 (x114 * (*_x0).Pose.Rot[2]);
	const FLT x120 = x42 + (-1 * x111 * (*_x0).Pose.Rot[2]) + (x114 * (*_x0).Pose.Rot[0]) +
					 (x108 * (*_x0).Pose.Rot[1]) + (-1 * x116 * (*_x0).Pose.Rot[3]);
	const FLT x121 = 2 * x94;
	const FLT x122 = x55 + (-1 * x57) + (-1 * x114 * (*_x0).Pose.Rot[1]) + (x108 * (*_x0).Pose.Rot[0]) + (-1 * x56) +
					 (x116 * (*_x0).Pose.Rot[2]) + (-1 * x111 * (*_x0).Pose.Rot[3]) + (-1 * x58);
	const FLT x123 = -1 * x100 * x120;
	const FLT x124 = (-1 * x68) + x86;
	const FLT x125 = x124 + x112 + x84;
	const FLT x126 = x115 + x80 + x77;
	const FLT x127 = -1 * x72;
	const FLT x128 = x107 + x127 + x73;
	const FLT x129 = -1 * x67;
	const FLT x130 = x70 + x129 + x109;
	const FLT x131 = (-1 * x30) + (-1 * x130 * (*_x0).Pose.Rot[2]) + (x126 * (*_x0).Pose.Rot[3]) +
					 (-1 * x128 * (*_x0).Pose.Rot[1]) + x38 + (-1 * x34) + (x125 * (*_x0).Pose.Rot[0]) + (-1 * x41);
	const FLT x132 = (x128 * (*_x0).Pose.Rot[0]) + x59 + (x126 * (*_x0).Pose.Rot[2]) + (x125 * (*_x0).Pose.Rot[1]) +
					 (x130 * (*_x0).Pose.Rot[3]);
	const FLT x133 = x117 + (x126 * (*_x0).Pose.Rot[0]) + (-1 * x125 * (*_x0).Pose.Rot[3]) +
					 (x130 * (*_x0).Pose.Rot[1]) + (-1 * x128 * (*_x0).Pose.Rot[2]);
	const FLT x134 = x54 + (x130 * (*_x0).Pose.Rot[0]) + (x125 * (*_x0).Pose.Rot[2]) +
					 (-1 * x128 * (*_x0).Pose.Rot[3]) + (-1 * x126 * (*_x0).Pose.Rot[1]);
	const FLT x135 = -1 * x100 * x133;
	const FLT x136 = x106 + x75 + x127;
	const FLT x137 = x85 + x124 + x113;
	const FLT x138 = x82 + x77 + x79;
	const FLT x139 = x110 + x129 + x65;
	const FLT x140 = x59 + (-1 * x138 * (*_x0).Pose.Rot[2]) + (x136 * (*_x0).Pose.Rot[0]) +
					 (x139 * (*_x0).Pose.Rot[3]) + (-1 * x137 * (*_x0).Pose.Rot[1]);
	const FLT x141 = (-1 * x136 * (*_x0).Pose.Rot[3]) + (-1 * x50) + (-1 * x137 * (*_x0).Pose.Rot[2]) + (-1 * x49) +
					 (x138 * (*_x0).Pose.Rot[1]) + x51 + (x139 * (*_x0).Pose.Rot[0]) + (-1 * x53);
	const FLT x142 = 2 * x141;
	const FLT x143 = x42 + (x136 * (*_x0).Pose.Rot[1]) + (x138 * (*_x0).Pose.Rot[3]) + (x137 * (*_x0).Pose.Rot[0]) +
					 (x139 * (*_x0).Pose.Rot[2]);
	const FLT x144 = x117 + (x138 * (*_x0).Pose.Rot[0]) + (-1 * x139 * (*_x0).Pose.Rot[1]) +
					 (-1 * x137 * (*_x0).Pose.Rot[3]) + (x136 * (*_x0).Pose.Rot[2]);
	const FLT x145 = -1 * x100 * x141;
	const FLT x146 = -1 * x43;
	const FLT x147 = x9 * x36;
	const FLT x148 = t * t * t;
	const FLT x149 = 1. / (x15 * sqrt(x15));
	const FLT x150 = x148 * x149;
	const FLT x151 = x2 * x150;
	const FLT x152 = x147 * x151;
	const FLT x153 = x10 * x150;
	const FLT x154 = x32 * x25;
	const FLT x155 = x9 * x9 * x9;
	const FLT x156 = t * t * t * t;
	const FLT x157 = 1.0 * x22 * x18;
	const FLT x158 = x149 * x157;
	const FLT x159 = x156 * x158;
	const FLT x160 = (1. / (x15 * x15)) * x19;
	const FLT x161 = 2 * x160;
	const FLT x162 = x161 * x156;
	const FLT x163 = x9 * x159;
	const FLT x164 = x0 * x21;
	const FLT x165 = 2 * x164;
	const FLT x166 = x9 * x162;
	const FLT x167 = x0 * x26;
	const FLT x168 = x167 * x157;
	const FLT x169 = (-1 * x13 * x166) + (-1 * x9 * x168) + (-1 * x162 * x155) + (x155 * x159) + (x7 * x163) +
					 (x9 * x165) + (-1 * x7 * x166) + (x13 * x163);
	const FLT x170 = 1.0 / 2.0 * (1. / (x23 * sqrt(x23)));
	const FLT x171 = x27 * x18 * x170;
	const FLT x172 = x9 * x171;
	const FLT x173 = x169 * x172;
	const FLT x174 = 0.5 * x167;
	const FLT x175 = x39 * x25;
	const FLT x176 = x9 * x175;
	const FLT x177 = 0.5 * x20 * x148;
	const FLT x178 = x34 * x177;
	const FLT x179 = x12 * x171;
	const FLT x180 = x6 * x179;
	const FLT x181 = x22 * x170;
	const FLT x182 = x169 * x181;
	const FLT x183 = x50 * x177;
	const FLT x184 = x9 * x183;
	const FLT x185 = x2 * x184;
	const FLT x186 = x35 * x169;
	const FLT x187 = x2 * x171;
	const FLT x188 = x12 * x150;
	const FLT x189 = x6 * x25;
	const FLT x190 = x9 * x189;
	const FLT x191 = x46 * x177;
	const FLT x192 = x9 * x191;
	const FLT x193 = (x12 * x192) + (-1 * x188 * x190);
	const FLT x194 = x193 + (-1 * x10 * x178) + (-1 * x152) + (-1 * x169 * x180) + x146 + (x153 * x154) + x185 +
					 (-1 * x39 * x182) + (x32 * x173) + (-1 * x187 * x186) + (-1 * x176 * x174);
	const FLT x195 = x9 * x178;
	const FLT x196 = x12 * x195;
	const FLT x197 = x9 * x154;
	const FLT x198 = x188 * x197;
	const FLT x199 = x176 * x151;
	const FLT x200 = x169 * x179;
	const FLT x201 = x57 * x177;
	const FLT x202 = x2 * x201;
	const FLT x203 = x9 * x202;
	const FLT x204 = x39 * x187;
	const FLT x205 = (-1 * x174 * x147) + (-1 * x35 * x182) + (x10 * x191) + (-1 * x198) + (-1 * x189 * x153) +
					 (-1 * x32 * x200) + x29 + x199 + x196 + (-1 * x6 * x173) + (x204 * x169) + (-1 * x203);
	const FLT x206 = x9 * x174;
	const FLT x207 = x169 * x187;
	const FLT x208 = (-1 * x12 * x184) + (x188 * x147);
	const FLT x209 = (x2 * x192) + (-1 * x190 * x151);
	const FLT x210 = x209 + x208 + (x10 * x201) + (-1 * x175 * x153) + (-1 * x39 * x173) + x40 + (-1 * x206 * x154) +
					 (x179 * x186) + (-1 * x32 * x182) + (-1 * x6 * x207);
	const FLT x211 = -1 * x37;
	const FLT x212 = x9 * x12 * x201;
	const FLT x213 = x176 * x188;
	const FLT x214 = (-1 * x2 * x195) + (x197 * x151);
	const FLT x215 = (x36 * x153) + (-1 * x10 * x183) + (-1 * x206 * x189) + x211 + (x39 * x200) + (x32 * x207) + x214 +
					 (-1 * x6 * x182) + (x172 * x186) + x213 + (-1 * x212);
	const FLT x216 = (-1 * x215 * (*_x0).Pose.Rot[1]) + (-1 * x210 * (*_x0).Pose.Rot[2]) + (x194 * (*_x0).Pose.Rot[3]) +
					 (x205 * (*_x0).Pose.Rot[0]);
	const FLT x217 = (x194 * (*_x0).Pose.Rot[2]) + (x210 * (*_x0).Pose.Rot[3]) + (x205 * (*_x0).Pose.Rot[1]) +
					 (x215 * (*_x0).Pose.Rot[0]);
	const FLT x218 = 2 * x217;
	const FLT x219 = (-1 * x205 * (*_x0).Pose.Rot[3]) + (-1 * x215 * (*_x0).Pose.Rot[2]) + (x210 * (*_x0).Pose.Rot[1]) +
					 (x194 * (*_x0).Pose.Rot[0]);
	const FLT x220 = (-1 * x194 * (*_x0).Pose.Rot[1]) + (x210 * (*_x0).Pose.Rot[0]) + (x205 * (*_x0).Pose.Rot[2]) +
					 (-1 * x215 * (*_x0).Pose.Rot[3]);
	const FLT x221 = -1 * x219 * x100;
	const FLT x222 = x10 * x156;
	const FLT x223 = x222 * x158;
	const FLT x224 = 2 * x12;
	const FLT x225 = x224 * x160;
	const FLT x226 = (x12 * x12 * x12) * x156;
	const FLT x227 = (x226 * x158) + (-1 * x226 * x161) + (-1 * x222 * x225) + (x12 * x223) + (x7 * x12 * x159) +
					 (-1 * x7 * x225 * x156) + (-1 * x12 * x168) + (x224 * x164);
	const FLT x228 = x227 * x172;
	const FLT x229 = x227 * x181;
	const FLT x230 = x227 * x187;
	const FLT x231 = x12 * x174;
	const FLT x232 = x13 * x150;
	const FLT x233 = x227 * x179;
	const FLT x234 = x2 * x188;
	const FLT x235 = (x234 * x175) + (-1 * x12 * x202);
	const FLT x236 = x193 + x235 + (-1 * x32 * x233) + (-1 * x232 * x154) + (x13 * x178) + (-1 * x35 * x229) +
					 (-1 * x6 * x228) + x43 + (-1 * x36 * x231) + (x39 * x230);
	const FLT x237 = x2 * x12;
	const FLT x238 = (-1 * x234 * x189) + (x237 * x191);
	const FLT x239 = x238 + (-1 * x213) + (x36 * x232) + x212 + (-1 * x13 * x183) + (x35 * x233) + (-1 * x39 * x228) +
					 (-1 * x231 * x154) + (-1 * x6 * x230) + x211 + (-1 * x32 * x229);
	const FLT x240 = x234 * x154;
	const FLT x241 = x237 * x178;
	const FLT x242 = -1 * x40;
	const FLT x243 = (-1 * x231 * x189) + (-1 * x6 * x229) + x242 + (-1 * x241) + (-1 * x13 * x201) + (x35 * x228) +
					 (x232 * x175) + x240 + (x39 * x233) + x208 + (x32 * x230);
	const FLT x244 = x237 * x183;
	const FLT x245 = x36 * x234;
	const FLT x246 = (-1 * x232 * x189) + (x13 * x191) + x29 + (-1 * x227 * x180) + x244 + (-1 * x196) + (x32 * x228) +
					 x198 + (-1 * x245) + (-1 * x35 * x230) + (-1 * x39 * x229) + (-1 * x231 * x175);
	const FLT x247 = (x246 * (*_x0).Pose.Rot[2]) + (x243 * (*_x0).Pose.Rot[0]) + (x236 * (*_x0).Pose.Rot[1]) +
					 (x239 * (*_x0).Pose.Rot[3]);
	const FLT x248 = (-1 * x236 * (*_x0).Pose.Rot[3]) + (-1 * x243 * (*_x0).Pose.Rot[2]) + (x239 * (*_x0).Pose.Rot[1]) +
					 (x246 * (*_x0).Pose.Rot[0]);
	const FLT x249 = (-1 * x243 * (*_x0).Pose.Rot[1]) + (x246 * (*_x0).Pose.Rot[3]) + (-1 * x239 * (*_x0).Pose.Rot[2]) +
					 (x236 * (*_x0).Pose.Rot[0]);
	const FLT x250 = (-1 * x243 * (*_x0).Pose.Rot[3]) + (x236 * (*_x0).Pose.Rot[2]) + (-1 * x246 * (*_x0).Pose.Rot[1]) +
					 (x239 * (*_x0).Pose.Rot[0]);
	const FLT x251 = -1 * x248 * x100;
	const FLT x252 = x2 * x2 * x2;
	const FLT x253 = x2 * x162;
	const FLT x254 = (-1 * x13 * x253) + (x2 * x13 * x159) + (x2 * x165) + (-1 * x2 * x168) + (-1 * x252 * x162) +
					 (x2 * x223) + (x252 * x159) + (-1 * x10 * x253);
	const FLT x255 = x6 * x254;
	const FLT x256 = x35 * x254;
	const FLT x257 = x7 * x150;
	const FLT x258 = x32 * x254;
	const FLT x259 = x2 * x174;
	const FLT x260 = x254 * x181;
	const FLT x261 = x214 + (-1 * x39 * x260) + x238 + (-1 * x256 * x187) + (x258 * x172) + (-1 * x36 * x257) + x37 +
					 (x7 * x183) + (-1 * x255 * x179) + (-1 * x259 * x175);
	const FLT x262 = x255 * x171;
	const FLT x263 = x209 + (-1 * x36 * x259) + x241 + (-1 * x7 * x201) + (-1 * x240) + (-1 * x35 * x260) +
					 (-1 * x258 * x179) + x242 + (-1 * x9 * x262) + (x204 * x254) + (x257 * x175);
	const FLT x264 = x39 * x254;
	const FLT x265 = x203 + (-1 * x259 * x154) + (-1 * x2 * x262) + (-1 * x257 * x189) + x245 + (-1 * x264 * x172) +
					 (x7 * x191) + (-1 * x32 * x260) + (x256 * x179) + (-1 * x199) + (-1 * x244) + x29;
	const FLT x266 = x235 + (-1 * x7 * x178) + (x257 * x154) + (x256 * x172) + (x264 * x179) + (-1 * x185) +
					 (-1 * x6 * x260) + x146 + x152 + (-1 * x259 * x189) + (x258 * x187);
	const FLT x267 = (x261 * (*_x0).Pose.Rot[3]) + (-1 * x266 * (*_x0).Pose.Rot[1]) + (-1 * x265 * (*_x0).Pose.Rot[2]) +
					 (x263 * (*_x0).Pose.Rot[0]);
	const FLT x268 = (x266 * (*_x0).Pose.Rot[0]) + (x261 * (*_x0).Pose.Rot[2]) + (x265 * (*_x0).Pose.Rot[3]) +
					 (x263 * (*_x0).Pose.Rot[1]);
	const FLT x269 = 2 * x268;
	const FLT x270 = (-1 * x263 * (*_x0).Pose.Rot[3]) + (-1 * x266 * (*_x0).Pose.Rot[2]) + (x265 * (*_x0).Pose.Rot[1]) +
					 (x261 * (*_x0).Pose.Rot[0]);
	const FLT x271 = (-1 * x261 * (*_x0).Pose.Rot[1]) + (-1 * x266 * (*_x0).Pose.Rot[3]) + (x265 * (*_x0).Pose.Rot[0]) +
					 (x263 * (*_x0).Pose.Rot[2]);
	const FLT x272 = -1 * x270 * x100;
	const FLT x273 = 2 * (1. / sqrt(1 + (-4 * (((x61 * x92) + (-1 * x60 * x94)) * ((x61 * x92) + (-1 * x60 * x94))))));
	const FLT x274 = x62 + (-2 * (x94 * x94));
	const FLT x275 = 1. / x274;
	const FLT x276 = 4 * x94;
	const FLT x277 = x274 * x274;
	const FLT x278 = (x93 * x94) + (x60 * x98);
	const FLT x279 = x278 * (1. / x277);
	const FLT x280 = x277 * (1. / (x277 + (x278 * x278)));
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
						x105 * ((-1 * x104 * (x101 + (-1 * x91 * x99))) +
								(((x98 * x97) + (x96 * x94) + (x89 * x90) + (x93 * x91)) * x64)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						x105 * ((-1 * x104 * (x123 + (-1 * x99 * x118))) +
								(x64 * ((x120 * x121) + (x93 * x118) + (x98 * x122) + (x90 * x119)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x105 * ((-1 * x104 * (x135 + (-1 * x99 * x131))) +
								(x64 * ((x98 * x134) + (x121 * x133) + (x93 * x131) + (x90 * x132)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						x105 * ((-1 * x104 * (x145 + (-1 * x99 * x140))) +
								(((x98 * x144) + (x90 * x143) + (x93 * x140) + (x94 * x142)) * x64)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x105 * ((-1 * x104 * (x221 + (-1 * x99 * x216))) +
								(x64 * ((x98 * x220) + (x219 * x121) + (x93 * x216) + (x60 * x218)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x105 * ((-1 * x104 * (x251 + (-1 * x99 * x249))) +
								(x64 * ((x98 * x250) + (x93 * x249) + (x90 * x247) + (x248 * x121)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x105 * ((-1 * x104 * (x272 + (-1 * x99 * x267))) +
								(x64 * ((x98 * x271) + (x93 * x267) + (x270 * x121) + (x60 * x269)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						((-1 * x91 * x94) + (-1 * x60 * x97) + (x92 * x95) + (x89 * x61)) * x273);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						((-1 * x94 * x118) + (x92 * x120) + (x61 * x119) + (-1 * x60 * x122)) * x273);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						((-1 * x94 * x131) + (x92 * x133) + (-1 * x60 * x134) + (x61 * x132)) * x273);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						((-1 * x60 * x144) + (x61 * x143) + (-1 * x94 * x140) + (x92 * x141)) * x273);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						((x92 * x219) + (-1 * x94 * x216) + (-1 * x60 * x220) + (x61 * x217)) * x273);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						((-1 * x60 * x250) + (-1 * x94 * x249) + (x92 * x248) + (x61 * x247)) * x273);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						((-1 * x94 * x267) + (x61 * x268) + (-1 * x60 * x271) + (x92 * x270)) * x273);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						x280 * ((-1 * x279 * ((-1 * x97 * x276) + x101)) +
								(x275 * ((x91 * x98) + (x60 * x96) + (x93 * x97) + (x89 * x121)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						x280 * ((-1 * x279 * ((-1 * x276 * x122) + x123)) +
								(x275 * ((x98 * x118) + (x90 * x120) + (x93 * x122) + (x119 * x121)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x280 * ((-1 * x279 * ((-1 * x276 * x134) + x135)) +
								(x275 * ((x98 * x131) + (x90 * x133) + (x93 * x134) + (x121 * x132)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						x280 * ((-1 * x279 * ((-1 * x276 * x144) + x145)) +
								(x275 * ((x98 * x140) + (x121 * x143) + (x93 * x144) + (x60 * x142)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x280 * ((-1 * x279 * ((-1 * x276 * x220) + x221)) +
								(((x98 * x216) + (x90 * x219) + (x93 * x220) + (x94 * x218)) * x275)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x280 * ((-1 * x279 * ((-1 * x276 * x250) + x251)) +
								(x275 * ((x98 * x249) + (x90 * x248) + (x247 * x121) + (x93 * x250)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x280 * ((-1 * x279 * ((-1 * x276 * x271) + x272)) +
								(((x98 * x267) + (x93 * x271) + (x94 * x269) + (x90 * x270)) * x275)));
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
// <cnkalman.codegen.WrapMember object at 0x7fe05a41dd30>] Jacobian of SurviveKalmanModelErrorPredict wrt
// [(*error_model).AccBias[0], (*error_model).AccBias[1], (*error_model).AccBias[2], (*error_model).Acc[0],
// (*error_model).Acc[1], (*error_model).Acc[2], (*error_model).GyroBias[0], (*error_model).GyroBias[1],
// (*error_model).GyroBias[2], (*error_model).IMUCorrection[0], (*error_model).IMUCorrection[1],
// (*error_model).IMUCorrection[2], (*error_model).IMUCorrection[3], (*error_model).Pose.AxisAngleRot[0],
// (*error_model).Pose.AxisAngleRot[1], (*error_model).Pose.AxisAngleRot[2], (*error_model).Pose.Pos[0],
// (*error_model).Pose.Pos[1], (*error_model).Pose.Pos[2], (*error_model).Velocity.AxisAngleRot[0],
// (*error_model).Velocity.AxisAngleRot[1], (*error_model).Velocity.AxisAngleRot[2], (*error_model).Velocity.Pos[0],
// (*error_model).Velocity.Pos[1], (*error_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at
// 0x7fe05a41cac0>]
static inline void gen_SurviveKalmanModelErrorPredict_jac_error_model(CnMat *Hx, const FLT t,
																	  const SurviveKalmanModel *_x0,
																	  const SurviveKalmanErrorModel *error_model) {
	const FLT x0 = t * t;
	const FLT x1 = 1.0 / 2.0 * x0;
	const FLT x2 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x3 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x4 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x5 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x6 = (-1 * x5 * (*_x0).Pose.Rot[1]) + (-1 * x4 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
				   (-1 * x3 * (*_x0).Pose.Rot[3]);
	const FLT x7 = x2 * x2;
	const FLT x8 = x0 * x7;
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x10 = x9 * x9;
	const FLT x11 = x0 * x10;
	const FLT x12 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x13 = x12 * x12;
	const FLT x14 = x0 * x13;
	const FLT x15 = 1e-10 + x14 + x8 + x11;
	const FLT x16 = sqrt(x15);
	const FLT x17 = 0.5 * x16;
	const FLT x18 = sin(x17);
	const FLT x19 = x18 * x18;
	const FLT x20 = 1. / x15;
	const FLT x21 = x20 * x19;
	const FLT x22 = cos(x17);
	const FLT x23 = (x21 * x11) + (x22 * x22) + (x8 * x21) + (x21 * x14);
	const FLT x24 = 1. / sqrt(x23);
	const FLT x25 = x24 * x18;
	const FLT x26 = 1. / x16;
	const FLT x27 = t * x26;
	const FLT x28 = x25 * x27;
	const FLT x29 = x6 * x28;
	const FLT x30 =
		(x3 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[3] + (-1 * x5 * (*_x0).Pose.Rot[2]) + (x4 * (*_x0).Pose.Rot[1]);
	const FLT x31 = x24 * x22;
	const FLT x32 = x30 * x31;
	const FLT x33 =
		(x5 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[1] + (-1 * x4 * (*_x0).Pose.Rot[3]) + (x3 * (*_x0).Pose.Rot[2]);
	const FLT x34 = x33 * x28;
	const FLT x35 =
		(x4 * (*_x0).Pose.Rot[0]) + (-1 * x3 * (*_x0).Pose.Rot[1]) + (x5 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x36 = x35 * x28;
	const FLT x37 = (x9 * x36) + (-1 * x34 * x12) + (x2 * x29) + x32;
	const FLT x38 = x30 * x25;
	const FLT x39 = x38 * x27;
	const FLT x40 = x6 * x31;
	const FLT x41 = (-1 * x9 * x34) + x40 + (-1 * x2 * x39) + (-1 * x36 * x12);
	const FLT x42 = x31 * x33;
	const FLT x43 = (x39 * x12) + (x9 * x29) + (-1 * x2 * x36) + x42;
	const FLT x44 = x31 * x35;
	const FLT x45 = (x29 * x12) + (-1 * x9 * x39) + x44 + (x2 * x34);
	const FLT x46 = (x45 * (*_x0).Pose.Rot[3]) + (-1 * x37 * (*_x0).Pose.Rot[2]) + (x43 * (*_x0).Pose.Rot[0]) +
					(-1 * x41 * (*_x0).Pose.Rot[1]);
	const FLT x47 = (x37 * (*_x0).Pose.Rot[1]) + (-1 * x43 * (*_x0).Pose.Rot[3]) + (x45 * (*_x0).Pose.Rot[0]) +
					(-1 * x41 * (*_x0).Pose.Rot[2]);
	const FLT x48 = 1 + (-2 * (x47 * x47));
	const FLT x49 = x48 + (-2 * (x46 * x46));
	const FLT x50 = 1. / x49;
	const FLT x51 = 0.5 * x28;
	const FLT x52 = x9 * x51;
	const FLT x53 = x52 * (*_x0).Pose.Rot[3];
	const FLT x54 = x51 * x12;
	const FLT x55 = x54 * (*_x0).Pose.Rot[0];
	const FLT x56 = x51 * (*_x0).Pose.Rot[1];
	const FLT x57 = x2 * x56;
	const FLT x58 = 0.5 * x31;
	const FLT x59 = x58 * (*_x0).Pose.Rot[2];
	const FLT x60 = (-1 * x59) + (-1 * x57) + x53 + (-1 * x55);
	const FLT x61 = x60 * (*_x0).Pose.Rot[3];
	const FLT x62 = x56 * x12;
	const FLT x63 = x58 * (*_x0).Pose.Rot[3];
	const FLT x64 = x51 * (*_x0).Pose.Rot[2];
	const FLT x65 = x9 * x64;
	const FLT x66 = x2 * x51;
	const FLT x67 = x66 * (*_x0).Pose.Rot[0];
	const FLT x68 = x65 + (-1 * x62) + x67 + x63;
	const FLT x69 = x54 * (*_x0).Pose.Rot[3];
	const FLT x70 = x52 * (*_x0).Pose.Rot[0];
	const FLT x71 = x58 * (*_x0).Pose.Rot[1];
	const FLT x72 = x2 * x64;
	const FLT x73 = x72 + (-1 * x71) + (-1 * x69) + (-1 * x70);
	const FLT x74 = (x58 * (*_x0).Pose.Rot[0]) + (-1 * x9 * x56) + (-1 * x64 * x12) + (-1 * x66 * (*_x0).Pose.Rot[3]);
	const FLT x75 = x74 * (*_x0).Pose.Rot[1];
	const FLT x76 = x75 + (x73 * (*_x0).Pose.Rot[0]);
	const FLT x77 = x76 + x61 + (x68 * (*_x0).Pose.Rot[2]);
	const FLT x78 = 2 * x46;
	const FLT x79 = x74 * (*_x0).Pose.Rot[0];
	const FLT x80 = -1 * x60 * (*_x0).Pose.Rot[2];
	const FLT x81 = -1 * x73 * (*_x0).Pose.Rot[1];
	const FLT x82 = x81 + x80 + (x68 * (*_x0).Pose.Rot[3]) + x79;
	const FLT x83 = (x43 * (*_x0).Pose.Rot[1]) + (x41 * (*_x0).Pose.Rot[0]) + (x37 * (*_x0).Pose.Rot[3]) +
					(x45 * (*_x0).Pose.Rot[2]);
	const FLT x84 = 2 * x83;
	const FLT x85 = x60 * (*_x0).Pose.Rot[1];
	const FLT x86 = x74 * (*_x0).Pose.Rot[3];
	const FLT x87 = x73 * (*_x0).Pose.Rot[2];
	const FLT x88 = (-1 * x87) + (x68 * (*_x0).Pose.Rot[0]) + x85 + (-1 * x86);
	const FLT x89 = (x43 * (*_x0).Pose.Rot[2]) + (-1 * x41 * (*_x0).Pose.Rot[3]) + (x37 * (*_x0).Pose.Rot[0]) +
					(-1 * x45 * (*_x0).Pose.Rot[1]);
	const FLT x90 = 2 * x89;
	const FLT x91 = x73 * (*_x0).Pose.Rot[3];
	const FLT x92 = x74 * (*_x0).Pose.Rot[2];
	const FLT x93 = (x60 * (*_x0).Pose.Rot[0]) + x92;
	const FLT x94 = x93 + (-1 * x91) + (-1 * x68 * (*_x0).Pose.Rot[1]);
	const FLT x95 = 2 * x47;
	const FLT x96 = 4 * x46;
	const FLT x97 = 4 * x47;
	const FLT x98 = -1 * x88 * x97;
	const FLT x99 = x49 * x49;
	const FLT x100 = (x84 * x46) + (x90 * x47);
	const FLT x101 = (1. / x99) * x100;
	const FLT x102 = x99 * (1. / (x99 + (x100 * x100)));
	const FLT x103 = x71 + (-1 * x72) + x70 + x69;
	const FLT x104 = x62 + (-1 * x65) + (-1 * x67) + (-1 * x63);
	const FLT x105 = (x104 * (*_x0).Pose.Rot[0]) + x86;
	const FLT x106 = x105 + (-1 * x103 * (*_x0).Pose.Rot[2]) + (-1 * x85);
	const FLT x107 = x104 * (*_x0).Pose.Rot[1];
	const FLT x108 = x93 + x107 + (x103 * (*_x0).Pose.Rot[3]);
	const FLT x109 = (-1 * x104 * (*_x0).Pose.Rot[3]) + x79;
	const FLT x110 = x109 + (x103 * (*_x0).Pose.Rot[1]) + x80;
	const FLT x111 = x104 * (*_x0).Pose.Rot[2];
	const FLT x112 = (-1 * x75) + (x103 * (*_x0).Pose.Rot[0]) + x111 + (-1 * x61);
	const FLT x113 = -1 * x97 * x110;
	const FLT x114 = x59 + x55 + (-1 * x53) + x57;
	const FLT x115 = x105 + (x114 * (*_x0).Pose.Rot[1]) + x87;
	const FLT x116 = x76 + (-1 * x114 * (*_x0).Pose.Rot[3]) + (-1 * x111);
	const FLT x117 = (x114 * (*_x0).Pose.Rot[0]) + x91 + (-1 * x107) + (-1 * x92);
	const FLT x118 = x109 + (x114 * (*_x0).Pose.Rot[2]) + x81;
	const FLT x119 = -1 * x97 * x116;
	const FLT x120 = x33 * x25;
	const FLT x121 = t * t * t;
	const FLT x122 = 1. / (x15 * sqrt(x15));
	const FLT x123 = x122 * x121;
	const FLT x124 = x9 * x123;
	const FLT x125 = x120 * x124;
	const FLT x126 = x2 * x125;
	const FLT x127 = x10 * x123;
	const FLT x128 = x9 * x9 * x9;
	const FLT x129 = t * t * t * t;
	const FLT x130 = 1.0 * x22 * x18;
	const FLT x131 = x122 * x130;
	const FLT x132 = x129 * x131;
	const FLT x133 = 2 * (1. / (x15 * x15)) * x19;
	const FLT x134 = x129 * x133;
	const FLT x135 = 2 * x21;
	const FLT x136 = x0 * x135;
	const FLT x137 = x13 * x132;
	const FLT x138 = x9 * x134;
	const FLT x139 = x26 * x130;
	const FLT x140 = x0 * x139;
	const FLT x141 = (-1 * x13 * x138) + (-1 * x128 * x134) + (-1 * x9 * x140) + (x128 * x132) + (-1 * x7 * x138) +
					 (x9 * x137) + (x7 * x9 * x132) + (x9 * x136);
	const FLT x142 = 1.0 / 2.0 * (1. / (x23 * sqrt(x23)));
	const FLT x143 = x27 * x18 * x142;
	const FLT x144 = x9 * x143;
	const FLT x145 = x141 * x144;
	const FLT x146 = x35 * x25;
	const FLT x147 = 0.5 * x26;
	const FLT x148 = x146 * x147;
	const FLT x149 = x0 * x148;
	const FLT x150 = 0.5 * x20 * x121;
	const FLT x151 = x32 * x150;
	const FLT x152 = x12 * x143;
	const FLT x153 = -1 * x39;
	const FLT x154 = x22 * x142;
	const FLT x155 = x141 * x154;
	const FLT x156 = x42 * x150;
	const FLT x157 = x2 * x9;
	const FLT x158 = x156 * x157;
	const FLT x159 = x2 * x143;
	const FLT x160 = x33 * x141;
	const FLT x161 = x6 * x25;
	const FLT x162 = x124 * x161;
	const FLT x163 = x40 * x150;
	const FLT x164 = x9 * x12;
	const FLT x165 = (x164 * x163) + (-1 * x12 * x162);
	const FLT x166 = x165 + (-1 * x160 * x159) + x153 + x158 + (-1 * x35 * x155) + (-1 * x6 * x141 * x152) +
					 (x38 * x127) + (-1 * x9 * x149) + (x30 * x145) + (-1 * x126) + (-1 * x10 * x151);
	const FLT x167 = x0 * x147;
	const FLT x168 = x9 * x167;
	const FLT x169 = x164 * x151;
	const FLT x170 = x33 * x154;
	const FLT x171 = x38 * x124;
	const FLT x172 = x12 * x171;
	const FLT x173 = x2 * x146;
	const FLT x174 = x124 * x173;
	const FLT x175 = x30 * x141;
	const FLT x176 = x44 * x150;
	const FLT x177 = x176 * x157;
	const FLT x178 = x35 * x141;
	const FLT x179 = (x10 * x163) + (-1 * x172) + x29 + (-1 * x170 * x141) + (-1 * x6 * x145) + x169 +
					 (-1 * x120 * x168) + x174 + (-1 * x175 * x152) + (-1 * x127 * x161) + (-1 * x177) + (x178 * x159);
	const FLT x180 = x6 * x159;
	const FLT x181 = (-1 * x164 * x156) + (x12 * x125);
	const FLT x182 = (x163 * x157) + (-1 * x2 * x162);
	const FLT x183 = x182 + x181 + (-1 * x127 * x146) + (x10 * x176) + (-1 * x35 * x145) + x36 + (-1 * x38 * x168) +
					 (-1 * x180 * x141) + (x160 * x152) + (-1 * x30 * x155);
	const FLT x184 = -1 * x34;
	const FLT x185 = x164 * x176;
	const FLT x186 = x12 * x124 * x146;
	const FLT x187 = x161 * x147;
	const FLT x188 = x0 * x187;
	const FLT x189 = (x2 * x171) + (-1 * x151 * x157);
	const FLT x190 = x189 + (x120 * x127) + (x175 * x159) + (x33 * x145) + (-1 * x6 * x155) + (-1 * x10 * x156) +
					 (x178 * x152) + x184 + (-1 * x9 * x188) + (-1 * x185) + x186;
	const FLT x191 = (-1 * x183 * (*_x0).Pose.Rot[2]) + (x166 * (*_x0).Pose.Rot[3]) + (-1 * x190 * (*_x0).Pose.Rot[1]) +
					 (x179 * (*_x0).Pose.Rot[0]);
	const FLT x192 = (x183 * (*_x0).Pose.Rot[3]) + (x179 * (*_x0).Pose.Rot[1]) + (x166 * (*_x0).Pose.Rot[2]) +
					 (x190 * (*_x0).Pose.Rot[0]);
	const FLT x193 = (-1 * x190 * (*_x0).Pose.Rot[2]) + (-1 * x179 * (*_x0).Pose.Rot[3]) + (x183 * (*_x0).Pose.Rot[1]) +
					 (x166 * (*_x0).Pose.Rot[0]);
	const FLT x194 = (-1 * x166 * (*_x0).Pose.Rot[1]) + (x183 * (*_x0).Pose.Rot[0]) + (x179 * (*_x0).Pose.Rot[2]) +
					 (-1 * x190 * (*_x0).Pose.Rot[3]);
	const FLT x195 = -1 * x97 * x193;
	const FLT x196 = x12 * x132;
	const FLT x197 = x12 * x134;
	const FLT x198 = (x12 * x12 * x12) * x129;
	const FLT x199 = (x198 * x131) + (-1 * x198 * x133) + (-1 * x10 * x197) + (-1 * x12 * x140) + (x7 * x196) +
					 (x10 * x196) + (-1 * x7 * x197) + (x12 * x136);
	const FLT x200 = x6 * x199;
	const FLT x201 = x35 * x199;
	const FLT x202 = x12 * x167;
	const FLT x203 = x13 * x123;
	const FLT x204 = x30 * x199;
	const FLT x205 = x12 * x123;
	const FLT x206 = x2 * x12;
	const FLT x207 = (-1 * x206 * x176) + (x205 * x173);
	const FLT x208 = x207 + x165 + (-1 * x204 * x152) + (-1 * x38 * x203) + (x13 * x151) + (x201 * x159) +
					 (-1 * x200 * x144) + x39 + (-1 * x170 * x199) + (-1 * x202 * x120);
	const FLT x209 = x199 * x154;
	const FLT x210 = x33 * x199;
	const FLT x211 = x2 * x205;
	const FLT x212 = (-1 * x211 * x161) + (x206 * x163);
	const FLT x213 = (-1 * x186) + (x203 * x120) + x184 + (-1 * x30 * x209) + x185 + (-1 * x13 * x156) +
					 (-1 * x201 * x144) + x212 + (-1 * x38 * x202) + (-1 * x180 * x199) + (x210 * x152);
	const FLT x214 = x38 * x211;
	const FLT x215 = x206 * x151;
	const FLT x216 = -1 * x36;
	const FLT x217 = x181 + (-1 * x6 * x209) + (-1 * x215) + (-1 * x13 * x176) + (x203 * x146) + x216 + (x201 * x152) +
					 (x204 * x159) + (-1 * x12 * x188) + (x210 * x144) + x214;
	const FLT x218 = x206 * x156;
	const FLT x219 = x211 * x120;
	const FLT x220 = (-1 * x203 * x161) + (x13 * x163) + (-1 * x200 * x152) + x218 + (-1 * x169) + (-1 * x219) + x29 +
					 (-1 * x12 * x149) + (x204 * x144) + (-1 * x210 * x159) + x172 + (-1 * x35 * x209);
	const FLT x221 = (x217 * (*_x0).Pose.Rot[0]) + (x208 * (*_x0).Pose.Rot[1]) + (x220 * (*_x0).Pose.Rot[2]) +
					 (x213 * (*_x0).Pose.Rot[3]);
	const FLT x222 = (-1 * x217 * (*_x0).Pose.Rot[2]) + (-1 * x208 * (*_x0).Pose.Rot[3]) + (x213 * (*_x0).Pose.Rot[1]) +
					 (x220 * (*_x0).Pose.Rot[0]);
	const FLT x223 = (-1 * x217 * (*_x0).Pose.Rot[1]) + (-1 * x213 * (*_x0).Pose.Rot[2]) + (x220 * (*_x0).Pose.Rot[3]) +
					 (x208 * (*_x0).Pose.Rot[0]);
	const FLT x224 = (-1 * x220 * (*_x0).Pose.Rot[1]) + (-1 * x217 * (*_x0).Pose.Rot[3]) + (x208 * (*_x0).Pose.Rot[2]) +
					 (x213 * (*_x0).Pose.Rot[0]);
	const FLT x225 = -1 * x97 * x222;
	const FLT x226 = x0 * x2;
	const FLT x227 = x2 * x2 * x2;
	const FLT x228 = x2 * x134;
	const FLT x229 = (-1 * x13 * x228) + (-1 * x226 * x139) + (x226 * x135) + (-1 * x227 * x134) + (x2 * x10 * x132) +
					 (x2 * x137) + (x227 * x132) + (-1 * x10 * x228);
	const FLT x230 = x6 * x229;
	const FLT x231 = x33 * x229;
	const FLT x232 = x7 * x123;
	const FLT x233 = x30 * x229;
	const FLT x234 = x229 * x154;
	const FLT x235 = x34 + (-1 * x35 * x234) + (-1 * x231 * x159) + (x7 * x156) + (-1 * x230 * x152) +
					 (-1 * x226 * x148) + x189 + (-1 * x232 * x120) + x212 + (x233 * x144);
	const FLT x236 = x35 * x229;
	const FLT x237 = x226 * x147;
	const FLT x238 = (-1 * x237 * x120) + (x236 * x159) + (-1 * x7 * x176) + x215 + (-1 * x214) + x182 +
					 (-1 * x229 * x170) + x216 + (-1 * x230 * x144) + (-1 * x233 * x152) + (x232 * x146);
	const FLT x239 = x177 + (-1 * x230 * x159) + (x7 * x163) + x219 + (-1 * x38 * x237) + (-1 * x30 * x234) +
					 (-1 * x232 * x161) + (x231 * x152) + (-1 * x236 * x144) + (-1 * x218) + x29 + (-1 * x174);
	const FLT x240 = x207 + (x231 * x144) + (-1 * x226 * x187) + (-1 * x6 * x234) + (x38 * x232) + (x236 * x152) +
					 x153 + (-1 * x7 * x151) + x126 + (x233 * x159) + (-1 * x158);
	const FLT x241 = (-1 * x240 * (*_x0).Pose.Rot[1]) + (-1 * x239 * (*_x0).Pose.Rot[2]) + (x235 * (*_x0).Pose.Rot[3]) +
					 (x238 * (*_x0).Pose.Rot[0]);
	const FLT x242 = (x235 * (*_x0).Pose.Rot[2]) + (x240 * (*_x0).Pose.Rot[0]) + (x239 * (*_x0).Pose.Rot[3]) +
					 (x238 * (*_x0).Pose.Rot[1]);
	const FLT x243 = (-1 * x240 * (*_x0).Pose.Rot[2]) + (-1 * x238 * (*_x0).Pose.Rot[3]) + (x239 * (*_x0).Pose.Rot[1]) +
					 (x235 * (*_x0).Pose.Rot[0]);
	const FLT x244 = (-1 * x240 * (*_x0).Pose.Rot[3]) + (x239 * (*_x0).Pose.Rot[0]) + (-1 * x235 * (*_x0).Pose.Rot[1]) +
					 (x238 * (*_x0).Pose.Rot[2]);
	const FLT x245 = -1 * x97 * x243;
	const FLT x246 = 2 * (1. / sqrt(1 + (-4 * (((x83 * x47) + (-1 * x89 * x46)) * ((x83 * x47) + (-1 * x89 * x46))))));
	const FLT x247 = x48 + (-2 * (x89 * x89));
	const FLT x248 = 1. / x247;
	const FLT x249 = 4 * x89;
	const FLT x250 = x247 * x247;
	const FLT x251 = (x89 * x84) + (x95 * x46);
	const FLT x252 = (1. / x250) * x251;
	const FLT x253 = x250 * (1. / (x250 + (x251 * x251)));
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
						x102 * ((-1 * x101 * (x98 + (-1 * x82 * x96))) +
								(((x95 * x94) + (x88 * x90) + (x78 * x77) + (x82 * x84)) * x50)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						x102 * ((-1 * x101 * (x113 + (-1 * x96 * x106))) +
								(((x95 * x112) + (x90 * x110) + (x84 * x106) + (x78 * x108)) * x50)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						x102 * ((-1 * x101 * (x119 + (-1 * x96 * x117))) +
								(((x84 * x117) + (x95 * x118) + (x78 * x115) + (x90 * x116)) * x50)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x102 * ((-1 * x101 * (x195 + (-1 * x96 * x191))) +
								(((x95 * x194) + (x90 * x193) + (x84 * x191) + (x78 * x192)) * x50)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x102 * ((-1 * x101 * (x225 + (-1 * x96 * x223))) +
								(((x95 * x224) + (x84 * x223) + (x78 * x221) + (x90 * x222)) * x50)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x102 * ((-1 * x101 * (x245 + (-1 * x96 * x241))) +
								(((x95 * x244) + (x90 * x243) + (x84 * x241) + (x78 * x242)) * x50)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						((-1 * x82 * x89) + (-1 * x94 * x46) + (x77 * x47) + (x83 * x88)) * x246);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						((-1 * x89 * x106) + (-1 * x46 * x112) + (x83 * x110) + (x47 * x108)) * x246);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						((-1 * x46 * x118) + (-1 * x89 * x117) + (x47 * x115) + (x83 * x116)) * x246);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						((-1 * x89 * x191) + (-1 * x46 * x194) + (x83 * x193) + (x47 * x192)) * x246);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						((-1 * x46 * x224) + (x83 * x222) + (-1 * x89 * x223) + (x47 * x221)) * x246);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						((-1 * x89 * x241) + (x47 * x242) + (-1 * x46 * x244) + (x83 * x243)) * x246);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						x253 * ((-1 * x252 * ((-1 * x94 * x249) + x98)) +
								(((x82 * x95) + (x77 * x90) + (x84 * x94) + (x88 * x78)) * x248)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						x253 * ((-1 * x252 * ((-1 * x249 * x112) + x113)) +
								(((x78 * x110) + (x90 * x108) + (x95 * x106) + (x84 * x112)) * x248)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						x253 * ((-1 * x252 * ((-1 * x249 * x118) + x119)) +
								(((x95 * x117) + (x78 * x116) + (x84 * x118) + (x90 * x115)) * x248)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x253 * ((-1 * x252 * ((-1 * x249 * x194) + x195)) +
								(((x78 * x193) + (x95 * x191) + (x84 * x194) + (x90 * x192)) * x248)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x253 * ((-1 * x252 * ((-1 * x224 * x249) + x225)) +
								(((x78 * x222) + (x90 * x221) + (x95 * x223) + (x84 * x224)) * x248)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x253 * ((-1 * x252 * ((-1 * x244 * x249) + x245)) +
								(((x95 * x241) + (x84 * x244) + (x90 * x242) + (x78 * x243)) * x248)));
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
// (*error_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7fe05a41cac0>]
static inline void gen_SurviveObsErrorModelNoFlip(SurviveAxisAnglePose *out, const SurviveKalmanModel *_x0,
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
// 0x7fe05a45d850>]
static inline void gen_SurviveObsErrorModelNoFlip_jac_x0(CnMat *Hx, const SurviveKalmanModel *_x0,
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
// <cnkalman.codegen.WrapMember object at 0x7fe05a45d850>] Jacobian of SurviveObsErrorModelNoFlip wrt
// [(*Z).AxisAngleRot[0], (*Z).AxisAngleRot[1], (*Z).AxisAngleRot[2], (*Z).Pos[0], (*Z).Pos[1], (*Z).Pos[2]]
static inline void gen_SurviveObsErrorModelNoFlip_jac_Z(CnMat *Hx, const SurviveKalmanModel *_x0,
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
static inline void gen_SurviveObsErrorModelFlip(SurviveAxisAnglePose *out, const SurviveKalmanModel *_x0,
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
// 0x7fe05a43ec70>]
static inline void gen_SurviveObsErrorModelFlip_jac_x0(CnMat *Hx, const SurviveKalmanModel *_x0,
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
// <cnkalman.codegen.WrapMember object at 0x7fe05a43ec70>] Jacobian of SurviveObsErrorModelFlip wrt
// [(*Z).AxisAngleRot[0], (*Z).AxisAngleRot[1], (*Z).AxisAngleRot[2], (*Z).Pos[0], (*Z).Pos[1], (*Z).Pos[2]]
static inline void gen_SurviveObsErrorModelFlip_jac_Z(CnMat *Hx, const SurviveKalmanModel *_x0,
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
static inline void gen_SurviveObsErrorStateErrorModelNoFlip(SurviveAxisAnglePose *out, const SurviveKalmanModel *_x0,
															const SurviveKalmanErrorModel *err,
															const SurviveAxisAnglePose *Z) {
	const FLT x0 = 0.5 * (*_x0).Pose.Rot[3];
	const FLT x1 = 0.5 * (*err).Pose.AxisAngleRot[2];
	const FLT x2 = 0.5 * (*err).Pose.AxisAngleRot[0];
	const FLT x3 = (x2 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[1] + (-1 * x0 * (*err).Pose.AxisAngleRot[1]) +
				   (x1 * (*_x0).Pose.Rot[2]);
	const FLT x4 = 0.5 * (*err).Pose.AxisAngleRot[1];
	const FLT x5 =
		(x1 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[3] + (-1 * x2 * (*_x0).Pose.Rot[2]) + (x4 * (*_x0).Pose.Rot[1]);
	const FLT x6 = (x4 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2] + (-1 * x1 * (*_x0).Pose.Rot[1]) +
				   (x0 * (*err).Pose.AxisAngleRot[0]);
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
// <cnkalman.codegen.WrapMember object at 0x7fe05a491e80>]
static inline void gen_SurviveObsErrorStateErrorModelNoFlip_jac_x0(CnMat *Hx, const SurviveKalmanModel *_x0,
																   const SurviveKalmanErrorModel *err,
																   const SurviveAxisAnglePose *Z) {
	const FLT x0 = 0.5 * (*err).Pose.AxisAngleRot[1];
	const FLT x1 = 0.5 * (*_x0).Pose.Rot[2];
	const FLT x2 = 0.5 * (*err).Pose.AxisAngleRot[0];
	const FLT x3 = (x2 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[1] + (-1 * x0 * (*_x0).Pose.Rot[3]) +
				   (x1 * (*err).Pose.AxisAngleRot[2]);
	const FLT x4 = 2 * x3;
	const FLT x5 = 0.5 * (*err).Pose.AxisAngleRot[2];
	const FLT x6 =
		(-1 * x5 * (*_x0).Pose.Rot[1]) + (x0 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2] + (x2 * (*_x0).Pose.Rot[3]);
	const FLT x7 = 1.0 * (*err).Pose.AxisAngleRot[1];
	const FLT x8 = (x5 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[3] + (-1 * x1 * (*err).Pose.AxisAngleRot[0]) +
				   (x0 * (*_x0).Pose.Rot[1]);
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
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7fe05a491e80>] Jacobian of
// SurviveObsErrorStateErrorModelNoFlip wrt [(*err).AccBias[0], (*err).AccBias[1], (*err).AccBias[2], (*err).Acc[0],
// (*err).Acc[1], (*err).Acc[2], (*err).GyroBias[0], (*err).GyroBias[1], (*err).GyroBias[2], (*err).IMUCorrection[0],
// (*err).IMUCorrection[1], (*err).IMUCorrection[2], (*err).IMUCorrection[3], (*err).Pose.AxisAngleRot[0],
// (*err).Pose.AxisAngleRot[1], (*err).Pose.AxisAngleRot[2], (*err).Pose.Pos[0], (*err).Pose.Pos[1], (*err).Pose.Pos[2],
// (*err).Velocity.AxisAngleRot[0], (*err).Velocity.AxisAngleRot[1], (*err).Velocity.AxisAngleRot[2],
// (*err).Velocity.Pos[0], (*err).Velocity.Pos[1], (*err).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at
// 0x7fe05a475a90>]
static inline void gen_SurviveObsErrorStateErrorModelNoFlip_jac_err(CnMat *Hx, const SurviveKalmanModel *_x0,
																	const SurviveKalmanErrorModel *err,
																	const SurviveAxisAnglePose *Z) {
	const FLT x0 = 0.5 * (*_x0).Pose.Rot[2];
	const FLT x1 = 0.5 * (*err).Pose.AxisAngleRot[1];
	const FLT x2 = 0.5 * (*err).Pose.AxisAngleRot[2];
	const FLT x3 = (x2 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[3] + (-1 * x0 * (*err).Pose.AxisAngleRot[0]) +
				   (x1 * (*_x0).Pose.Rot[1]);
	const FLT x4 = 1.0 * x3;
	const FLT x5 = 0.5 * (*err).Pose.AxisAngleRot[0];
	const FLT x6 =
		(x1 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2] + (-1 * x2 * (*_x0).Pose.Rot[1]) + (x5 * (*_x0).Pose.Rot[3]);
	const FLT x7 = 1.0 * x6;
	const FLT x8 = (-1 * x1 * (*_x0).Pose.Rot[3]) + (x5 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[1] +
				   (x0 * (*err).Pose.AxisAngleRot[2]);
	const FLT x9 = 1.0 * x8;
	const FLT x10 = (x9 * (*_x0).Pose.Rot[0]) + (-1 * x4 * (*_x0).Pose.Rot[2]) + (x7 * (*_x0).Pose.Rot[3]);
	const FLT x11 = 1e-10 + (x3 * x3) + (x6 * x6) + (x8 * x8);
	const FLT x12 = sqrt(x11);
	const FLT x13 = 1. / x12;
	const FLT x14 = (-1 * x5 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[0] + (-1 * x2 * (*_x0).Pose.Rot[3]) +
					(-1 * x0 * (*err).Pose.AxisAngleRot[1]);
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
	const FLT x28 = (x0 * x17) + (x27 * x15);
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
// (*err).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7fe05a475a90>] Jacobian of
// SurviveObsErrorStateErrorModelNoFlip wrt [(*Z).AxisAngleRot[0], (*Z).AxisAngleRot[1], (*Z).AxisAngleRot[2],
// (*Z).Pos[0], (*Z).Pos[1], (*Z).Pos[2]]
static inline void gen_SurviveObsErrorStateErrorModelNoFlip_jac_Z(CnMat *Hx, const SurviveKalmanModel *_x0,
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
static inline void gen_SurviveObsErrorStateErrorModelFlip(SurviveAxisAnglePose *out, const SurviveKalmanModel *_x0,
														  const SurviveKalmanErrorModel *err,
														  const SurviveAxisAnglePose *Z) {
	const FLT x0 = 0.5 * (*err).Pose.AxisAngleRot[1];
	const FLT x1 = 0.5 * (*_x0).Pose.Rot[2];
	const FLT x2 = 0.5 * (*err).Pose.AxisAngleRot[0];
	const FLT x3 = (x2 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[1] + (-1 * x0 * (*_x0).Pose.Rot[3]) +
				   (x1 * (*err).Pose.AxisAngleRot[2]);
	const FLT x4 = 0.5 * (*err).Pose.AxisAngleRot[2];
	const FLT x5 = (*_x0).Pose.Rot[3] + (x4 * (*_x0).Pose.Rot[0]) + (-1 * x1 * (*err).Pose.AxisAngleRot[0]) +
				   (x0 * (*_x0).Pose.Rot[1]);
	const FLT x6 =
		(-1 * x4 * (*_x0).Pose.Rot[1]) + (x0 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2] + (x2 * (*_x0).Pose.Rot[3]);
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
// <cnkalman.codegen.WrapMember object at 0x7fe05a3b4940>]
static inline void gen_SurviveObsErrorStateErrorModelFlip_jac_x0(CnMat *Hx, const SurviveKalmanModel *_x0,
																 const SurviveKalmanErrorModel *err,
																 const SurviveAxisAnglePose *Z) {
	const FLT x0 = 0.5 * (*err).Pose.AxisAngleRot[2];
	const FLT x1 = 0.5 * (*err).Pose.AxisAngleRot[0];
	const FLT x2 = 0.5 * (*err).Pose.AxisAngleRot[1];
	const FLT x3 =
		(-1 * x0 * (*_x0).Pose.Rot[1]) + (x2 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2] + (x1 * (*_x0).Pose.Rot[3]);
	const FLT x4 = 1.0 * x3;
	const FLT x5 =
		(x0 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[3] + (-1 * x1 * (*_x0).Pose.Rot[2]) + (x2 * (*_x0).Pose.Rot[1]);
	const FLT x6 = 1.0 * (*err).Pose.AxisAngleRot[2];
	const FLT x7 =
		(*_x0).Pose.Rot[1] + (x1 * (*_x0).Pose.Rot[0]) + (-1 * x2 * (*_x0).Pose.Rot[3]) + (x0 * (*_x0).Pose.Rot[2]);
	const FLT x8 = 1.0 * x7;
	const FLT x9 = (x8 * (*err).Pose.AxisAngleRot[0]) + (x4 * (*err).Pose.AxisAngleRot[1]) + (x6 * x5);
	const FLT x10 = 1e-10 + (x3 * x3) + (x5 * x5) + (x7 * x7);
	const FLT x11 = sqrt(x10);
	const FLT x12 = 1. / x11;
	const FLT x13 = (-1 * x1 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[0] + (-1 * x0 * (*_x0).Pose.Rot[3]) +
					(-1 * x2 * (*_x0).Pose.Rot[2]);
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
	const FLT x56 = (x1 * x16) + (x55 * x14);
	const FLT x57 = (-1 * x55 * x23) + x53 + (x56 * x20);
	const FLT x58 = (-1 * x44) + (x56 * x37) + (-1 * x55 * x38);
	const FLT x59 = (-1 * x55 * x42) + x39 + (x56 * x43);
	const FLT x60 = (x57 * x47) + (x58 * x41) + (x59 * x46);
	const FLT x61 = x60 * x49;
	const FLT x62 = x60 * x51;
	const FLT x63 = (x6 * x7) + x31 + (-1 * x54 * (*err).Pose.AxisAngleRot[0]);
	const FLT x64 = x19 * ((x2 * x16) + (x63 * x14));
	const FLT x65 = x44 + (x64 * x18) + (-1 * x63 * x23);
	const FLT x66 = (x64 * x31) + x53 + (-1 * x63 * x38);
	const FLT x67 = (-1 * x26) + (x64 * x28) + (-1 * x63 * x42);
	const FLT x68 = (x66 * x41) + (x65 * x47) + (x67 * x46);
	const FLT x69 = x68 * x49;
	const FLT x70 = x68 * x51;
	const FLT x71 = (x4 * (*err).Pose.AxisAngleRot[0]) + (-1 * x8 * (*err).Pose.AxisAngleRot[1]) + x28;
	const FLT x72 = x19 * ((x0 * x16) + (x71 * x14));
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
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7fe05a3b4940>] Jacobian of
// SurviveObsErrorStateErrorModelFlip wrt [(*err).AccBias[0], (*err).AccBias[1], (*err).AccBias[2], (*err).Acc[0],
// (*err).Acc[1], (*err).Acc[2], (*err).GyroBias[0], (*err).GyroBias[1], (*err).GyroBias[2], (*err).IMUCorrection[0],
// (*err).IMUCorrection[1], (*err).IMUCorrection[2], (*err).IMUCorrection[3], (*err).Pose.AxisAngleRot[0],
// (*err).Pose.AxisAngleRot[1], (*err).Pose.AxisAngleRot[2], (*err).Pose.Pos[0], (*err).Pose.Pos[1], (*err).Pose.Pos[2],
// (*err).Velocity.AxisAngleRot[0], (*err).Velocity.AxisAngleRot[1], (*err).Velocity.AxisAngleRot[2],
// (*err).Velocity.Pos[0], (*err).Velocity.Pos[1], (*err).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at
// 0x7fe05a3b66d0>]
static inline void gen_SurviveObsErrorStateErrorModelFlip_jac_err(CnMat *Hx, const SurviveKalmanModel *_x0,
																  const SurviveKalmanErrorModel *err,
																  const SurviveAxisAnglePose *Z) {
	const FLT x0 = 0.5 * (*err).Pose.AxisAngleRot[1];
	const FLT x1 = 0.5 * (*err).Pose.AxisAngleRot[2];
	const FLT x2 = 0.5 * (*err).Pose.AxisAngleRot[0];
	const FLT x3 =
		(x2 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[1] + (-1 * x0 * (*_x0).Pose.Rot[3]) + (x1 * (*_x0).Pose.Rot[2]);
	const FLT x4 =
		(x1 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[3] + (-1 * x2 * (*_x0).Pose.Rot[2]) + (x0 * (*_x0).Pose.Rot[1]);
	const FLT x5 = 1.0 * x4;
	const FLT x6 =
		(-1 * x1 * (*_x0).Pose.Rot[1]) + (x0 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2] + (x2 * (*_x0).Pose.Rot[3]);
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
// (*err).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7fe05a3b66d0>] Jacobian of
// SurviveObsErrorStateErrorModelFlip wrt [(*Z).AxisAngleRot[0], (*Z).AxisAngleRot[1], (*Z).AxisAngleRot[2],
// (*Z).Pos[0], (*Z).Pos[1], (*Z).Pos[2]]
static inline void gen_SurviveObsErrorStateErrorModelFlip_jac_Z(CnMat *Hx, const SurviveKalmanModel *_x0,
																const SurviveKalmanErrorModel *err,
																const SurviveAxisAnglePose *Z) {
	const FLT x0 = 0.5 * (*err).Pose.AxisAngleRot[0];
	const FLT x1 = 0.5 * (*err).Pose.AxisAngleRot[1];
	const FLT x2 = 0.5 * (*err).Pose.AxisAngleRot[2];
	const FLT x3 =
		(x2 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[3] + (-1 * x0 * (*_x0).Pose.Rot[2]) + (x1 * (*_x0).Pose.Rot[1]);
	const FLT x4 =
		(*_x0).Pose.Rot[1] + (x0 * (*_x0).Pose.Rot[0]) + (-1 * x1 * (*_x0).Pose.Rot[3]) + (x2 * (*_x0).Pose.Rot[2]);
	const FLT x5 =
		(x1 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[2] + (-1 * x2 * (*_x0).Pose.Rot[1]) + (x0 * (*_x0).Pose.Rot[3]);
	const FLT x6 = sqrt(1e-10 + (x5 * x5) + (x3 * x3) + (x4 * x4));
	const FLT x7 = 2 * (1. / x6) *
				   atan2(x6, (-1 * x0 * (*_x0).Pose.Rot[1]) + (-1 * x2 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[0] +
								 (-1 * x1 * (*_x0).Pose.Rot[2]));
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
