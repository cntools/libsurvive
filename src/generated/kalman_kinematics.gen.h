/// NOTE: This is a generated file; do not edit.
#pragma once
#include <cnkalman/generated_header.h>
// clang-format off
static inline void gen_GenerateQuatErrorModel(CnMat *out, const FLT *_x1, const FLT *_x0) {
	const FLT _x10 = _x1[0];
	const FLT _x11 = _x1[1];
	const FLT _x12 = _x1[2];
	const FLT _x13 = _x1[3];
	const FLT _x00 = _x0[0];
	const FLT _x01 = _x0[1];
	const FLT _x02 = _x0[2];
	const FLT _x03 = _x0[3];
	const FLT x0 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x1 = (_x02 * _x11) + (-1 * _x01 * _x12) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x2 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x3 = (_x03 * _x12) + (-1 * _x02 * _x13) + (-1 * _x01 * _x10) + (_x00 * _x11);
	const FLT x4 = x0 * x0;
	cnSetZero(out);
	cnMatrixOptionalSet(out, 0, 0, atan2(2 * ((x2 * x3) + (x0 * x1)), 1 + (-2 * ((x3 * x3) + x4))));
	cnMatrixOptionalSet(out, 1, 0, asin(2 * ((x0 * x2) + (-1 * x1 * x3))));
	cnMatrixOptionalSet(out, 2, 0, atan2(2 * ((x2 * x1) + (x0 * x3)), 1 + (-2 * (x4 + (x1 * x1)))));
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
	const FLT x0 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x1 = x0 * _x03;
	const FLT x2 = (_x02 * _x11) + (-1 * _x01 * _x12) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x3 = x2 * _x02;
	const FLT x4 = (_x03 * _x12) + (-1 * _x02 * _x13) + (-1 * _x01 * _x10) + (_x00 * _x11);
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
	cnSetZero(Hx);
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
	const FLT x1 = x0 * _x10;
	const FLT x2 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x3 = x2 * _x13;
	const FLT x4 = x3 + x1;
	const FLT x5 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x6 = x5 * _x11;
	const FLT x7 = (_x02 * _x11) + (-1 * _x01 * _x12) + (-1 * _x03 * _x10) + (_x00 * _x13);
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
	cnSetZero(Hx);
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
	const FLT x0 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x1 = (_x02 * _x11) + (-1 * _x01 * _x12) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x2 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x3 = (_x03 * _x12) + (-1 * _x02 * _x13) + (-1 * _x01 * _x10) + (_x00 * _x11);
	const FLT x4 = x0 * x0;
	cnSetZero(out);
	cnMatrixOptionalSet(out, 0, 0, atan2(2 * ((x2 * x3) + (x0 * x1)), 1 + (-2 * ((x3 * x3) + x4))));
	cnMatrixOptionalSet(out, 1, 0, asin(2 * ((x0 * x2) + (-1 * x1 * x3))));
	cnMatrixOptionalSet(out, 2, 0, atan2(2 * ((x2 * x1) + (x0 * x3)), 1 + (-2 * (x4 + (x1 * x1)))));
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
	const FLT x0 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x1 = x0 * _x03;
	const FLT x2 = (_x02 * _x11) + (-1 * _x01 * _x12) + (-1 * _x03 * _x10) + (_x00 * _x13);
	const FLT x3 = x2 * _x02;
	const FLT x4 = (_x03 * _x12) + (-1 * _x02 * _x13) + (-1 * _x01 * _x10) + (_x00 * _x11);
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
	cnSetZero(Hx);
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
	const FLT x1 = x0 * _x10;
	const FLT x2 = (_x01 * _x13) + (_x00 * _x12) + (-1 * _x03 * _x11) + (-1 * _x02 * _x10);
	const FLT x3 = x2 * _x13;
	const FLT x4 = x3 + x1;
	const FLT x5 = (_x01 * _x11) + (_x02 * _x12) + (_x00 * _x10) + (_x03 * _x13);
	const FLT x6 = x5 * _x11;
	const FLT x7 = (_x02 * _x11) + (-1 * _x01 * _x12) + (-1 * _x03 * _x10) + (_x00 * _x13);
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
	cnSetZero(Hx);
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
	const FLT x0 = ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[0]) +
				   (-1 * (*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[3]) + (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[2]);
	const FLT x1 = ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[2]) + (-1 * (*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[1]) +
				   (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[3]) + ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[0]);
	const FLT x2 = ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[3]) + (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[1]) +
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
// <cnkalman.codegen.WrapMember object at 0x7f784d529d00>]
static inline void gen_SurviveKalmanModelToErrorModel_jac_x1(CnMat *Hx, const SurviveKalmanModel *_x1,
															 const SurviveKalmanModel *_x0) {
	const FLT x0 = ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[0]) +
				   (-1 * (*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[3]) + (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[2]);
	const FLT x1 = x0 * (*_x0).Pose.Rot[3];
	const FLT x2 = ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[2]) + (-1 * (*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[1]) +
				   (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[3]) + ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[0]);
	const FLT x3 = x2 * (*_x0).Pose.Rot[2];
	const FLT x4 = ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[3]) +
				   ((*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[0]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[2]);
	const FLT x5 = x4 * (*_x0).Pose.Rot[1];
	const FLT x6 = ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[3]) + (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[1]) +
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
// <cnkalman.codegen.WrapMember object at 0x7f784d529d00>] Jacobian of SurviveKalmanModelToErrorModel wrt
// [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2], (*_x0).Acc[0], (*_x0).Acc[1], (*_x0).Acc[2],
// (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1],
// (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3], (*_x0).Pose.Pos[0], (*_x0).Pose.Pos[1], (*_x0).Pose.Pos[2],
// (*_x0).Pose.Rot[0], (*_x0).Pose.Rot[1], (*_x0).Pose.Rot[2], (*_x0).Pose.Rot[3], (*_x0).Velocity.AxisAngleRot[0],
// (*_x0).Velocity.AxisAngleRot[1], (*_x0).Velocity.AxisAngleRot[2], (*_x0).Velocity.Pos[0], (*_x0).Velocity.Pos[1],
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f784d4afaf0>]
static inline void gen_SurviveKalmanModelToErrorModel_jac_x0(CnMat *Hx, const SurviveKalmanModel *_x1,
															 const SurviveKalmanModel *_x0) {
	const FLT x0 = ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[3]) + (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[1]) +
				   ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[0]) + (-1 * (*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[2]);
	const FLT x1 = x0 * (*_x1).Pose.Rot[0];
	const FLT x2 = ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[0]) +
				   (-1 * (*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[3]) + (-1 * (*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[2]);
	const FLT x3 = x2 * (*_x1).Pose.Rot[3];
	const FLT x4 = x3 + x1;
	const FLT x5 = ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[1]) + ((*_x1).Pose.Rot[3] * (*_x0).Pose.Rot[3]) +
				   ((*_x1).Pose.Rot[0] * (*_x0).Pose.Rot[0]) + ((*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[2]);
	const FLT x6 = x5 * (*_x1).Pose.Rot[1];
	const FLT x7 = ((*_x1).Pose.Rot[1] * (*_x0).Pose.Rot[2]) + (-1 * (*_x1).Pose.Rot[2] * (*_x0).Pose.Rot[1]) +
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
// <cnkalman.codegen.WrapMember object at 0x7f784d4afaf0>]
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
// <cnkalman.codegen.WrapMember object at 0x7f784d4c5e20>]
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
// (*_x0).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f784d4c5e20>] Jacobian of
// SurviveKalmanModelAddErrorModel wrt [(*error_state).AccBias[0], (*error_state).AccBias[1], (*error_state).AccBias[2],
// (*error_state).Acc[0], (*error_state).Acc[1], (*error_state).Acc[2], (*error_state).GyroBias[0],
// (*error_state).GyroBias[1], (*error_state).GyroBias[2], (*error_state).IMUCorrection[0],
// (*error_state).IMUCorrection[1], (*error_state).IMUCorrection[2], (*error_state).IMUCorrection[3],
// (*error_state).Pose.AxisAngleRot[0], (*error_state).Pose.AxisAngleRot[1], (*error_state).Pose.AxisAngleRot[2],
// (*error_state).Pose.Pos[0], (*error_state).Pose.Pos[1], (*error_state).Pose.Pos[2],
// (*error_state).Velocity.AxisAngleRot[0], (*error_state).Velocity.AxisAngleRot[1],
// (*error_state).Velocity.AxisAngleRot[2], (*error_state).Velocity.Pos[0], (*error_state).Velocity.Pos[1],
// (*error_state).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f784d4c8bb0>]
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
// (*error_state).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f784d4c8bb0>]
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
// (*kalman_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f784d4d6f40>]
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
// (*kalman_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f784d4d6f40>]
static inline void gen_SurviveKalmanModelErrorPredict(SurviveKalmanErrorModel *out, const FLT t,
													  const SurviveKalmanModel *_x0,
													  const SurviveKalmanErrorModel *error_model) {
	const FLT x0 = (*_x0).Acc[0] + (*error_model).Acc[0];
	const FLT x1 = t * t;
	const FLT x2 = 1.0 / 2.0 * x1;
	const FLT x3 = (*_x0).Acc[1] + (*error_model).Acc[1];
	const FLT x4 = (*_x0).Acc[2] + (*error_model).Acc[2];
	const FLT x5 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x6 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x7 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x8 =
		(x7 * (*_x0).Pose.Rot[0]) + (-1 * x5 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[3] + (x6 * (*_x0).Pose.Rot[1]);
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x10 = x1 * (x9 * x9);
	const FLT x11 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x12 = x1 * (x11 * x11);
	const FLT x13 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x14 = x1 * (x13 * x13);
	const FLT x15 = 1e-10 + x14 + x10 + x12;
	const FLT x16 = sqrt(x15);
	const FLT x17 = 0.5 * x16;
	const FLT x18 = sin(x17);
	const FLT x19 = (1. / x15) * (x18 * x18);
	const FLT x20 = cos(x17);
	const FLT x21 = 1. / sqrt((x12 * x19) + (x20 * x20) + (x10 * x19) + (x14 * x19));
	const FLT x22 = t * x21 * x18 * (1. / x16);
	const FLT x23 = x9 * x22;
	const FLT x24 =
		(x6 * (*_x0).Pose.Rot[0]) + (x5 * (*_x0).Pose.Rot[3]) + (-1 * x7 * (*_x0).Pose.Rot[1]) + (*_x0).Pose.Rot[2];
	const FLT x25 = x24 * x22;
	const FLT x26 = (-1 * x6 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] + (-1 * x5 * (*_x0).Pose.Rot[1]) +
					(-1 * x7 * (*_x0).Pose.Rot[3]);
	const FLT x27 = x20 * x21;
	const FLT x28 =
		(x5 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[1] + (-1 * x6 * (*_x0).Pose.Rot[3]) + (x7 * (*_x0).Pose.Rot[2]);
	const FLT x29 = x22 * x28;
	const FLT x30 = (-1 * x29 * x11) + (-1 * x8 * x23) + (x26 * x27) + (-1 * x25 * x13);
	const FLT x31 = x8 * x22;
	const FLT x32 = x22 * x26;
	const FLT x33 = (-1 * x31 * x11) + (x32 * x13) + (x24 * x27) + (x23 * x28);
	const FLT x34 = (x25 * x11) + (-1 * x29 * x13) + (x9 * x32) + (x8 * x27);
	const FLT x35 = (x32 * x11) + (-1 * x9 * x25) + (x31 * x13) + (x28 * x27);
	const FLT x36 = (x35 * (*_x0).Pose.Rot[2]) + (x34 * (*_x0).Pose.Rot[0]) + (-1 * x30 * (*_x0).Pose.Rot[3]) +
					(-1 * x33 * (*_x0).Pose.Rot[1]);
	const FLT x37 = (x33 * (*_x0).Pose.Rot[0]) + (-1 * x35 * (*_x0).Pose.Rot[3]) + (x34 * (*_x0).Pose.Rot[1]) +
					(-1 * x30 * (*_x0).Pose.Rot[2]);
	const FLT x38 = (x35 * (*_x0).Pose.Rot[0]) + (-1 * x34 * (*_x0).Pose.Rot[2]) + (x33 * (*_x0).Pose.Rot[3]) +
					(-1 * x30 * (*_x0).Pose.Rot[1]);
	const FLT x39 = (x35 * (*_x0).Pose.Rot[1]) + (x34 * (*_x0).Pose.Rot[3]) + (x30 * (*_x0).Pose.Rot[0]) +
					(x33 * (*_x0).Pose.Rot[2]);
	const FLT x40 = x37 * x37;
	out->Pose.Pos[0] =
		(t * ((*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0])) + (*error_model).Pose.Pos[0] + (x0 * x2);
	out->Pose.Pos[1] =
		(t * ((*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1])) + (x2 * x3) + (*error_model).Pose.Pos[1];
	out->Pose.Pos[2] =
		(t * ((*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2])) + (x2 * x4) + (*error_model).Pose.Pos[2];
	out->Pose.AxisAngleRot[0] = atan2(2 * ((x38 * x39) + (x36 * x37)), 1 + (-2 * ((x38 * x38) + x40)));
	out->Pose.AxisAngleRot[1] = asin(2 * ((x37 * x39) + (-1 * x36 * x38)));
	out->Pose.AxisAngleRot[2] = atan2(2 * ((x36 * x39) + (x38 * x37)), 1 + (-2 * (x40 + (x36 * x36))));
	out->Velocity.Pos[0] = (*error_model).Velocity.Pos[0] + (t * x0);
	out->Velocity.Pos[1] = (*error_model).Velocity.Pos[1] + (t * x3);
	out->Velocity.Pos[2] = (*error_model).Velocity.Pos[2] + (t * x4);
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
	const FLT x3 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x4 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x5 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x6 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x7 =
		(*_x0).Pose.Rot[3] + (-1 * x4 * (*_x0).Pose.Rot[2]) + (x6 * (*_x0).Pose.Rot[0]) + (x5 * (*_x0).Pose.Rot[1]);
	const FLT x8 = t * t;
	const FLT x9 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x10 = x9 * x9;
	const FLT x11 = x8 * x10;
	const FLT x12 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x13 = x12 * x12;
	const FLT x14 = x8 * x13;
	const FLT x15 = x3 * x3;
	const FLT x16 = x8 * x15;
	const FLT x17 = 1e-10 + x16 + x11 + x14;
	const FLT x18 = sqrt(x17);
	const FLT x19 = 0.5 * x18;
	const FLT x20 = sin(x19);
	const FLT x21 = x20 * x20;
	const FLT x22 = 1. / x17;
	const FLT x23 = x22 * x21;
	const FLT x24 = cos(x19);
	const FLT x25 = (x23 * x14) + (x24 * x24) + (x23 * x11) + (x23 * x16);
	const FLT x26 = 1. / sqrt(x25);
	const FLT x27 = x20 * (1. / x18);
	const FLT x28 = x26 * x27;
	const FLT x29 = x7 * x28;
	const FLT x30 = x3 * x29;
	const FLT x31 =
		(x4 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[1] + (-1 * x5 * (*_x0).Pose.Rot[3]) + (x6 * (*_x0).Pose.Rot[2]);
	const FLT x32 = x24 * x26;
	const FLT x33 = x32 * x31;
	const FLT x34 =
		(x5 * (*_x0).Pose.Rot[0]) + (-1 * x6 * (*_x0).Pose.Rot[1]) + (x4 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x35 = x34 * x28;
	const FLT x36 = x9 * x35;
	const FLT x37 = (-1 * x4 * (*_x0).Pose.Rot[1]) + (-1 * x5 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] +
					(-1 * x6 * (*_x0).Pose.Rot[3]);
	const FLT x38 = x37 * x28;
	const FLT x39 = x38 * x12;
	const FLT x40 = (t * x39) + (-1 * t * x36) + (t * x30) + x33;
	const FLT x41 = x9 * x29;
	const FLT x42 = x3 * x35;
	const FLT x43 = x32 * x37;
	const FLT x44 = x31 * x28;
	const FLT x45 = x44 * x12;
	const FLT x46 = (-1 * t * x45) + x43 + (-1 * t * x41) + (-1 * t * x42);
	const FLT x47 = x28 * x12;
	const FLT x48 = x7 * x47;
	const FLT x49 = x9 * x44;
	const FLT x50 = x32 * x34;
	const FLT x51 = x3 * x38;
	const FLT x52 = (t * x51) + (-1 * t * x48) + x50 + (t * x49);
	const FLT x53 = x9 * x38;
	const FLT x54 = x7 * x32;
	const FLT x55 = x3 * x44;
	const FLT x56 = x47 * x34;
	const FLT x57 = (t * x56) + (-1 * t * x55) + (t * x53) + x54;
	const FLT x58 = (x57 * (*_x0).Pose.Rot[1]) + (x52 * (*_x0).Pose.Rot[0]) + (-1 * x40 * (*_x0).Pose.Rot[3]) +
					(-1 * x46 * (*_x0).Pose.Rot[2]);
	const FLT x59 = x58 * x58;
	const FLT x60 = (x52 * (*_x0).Pose.Rot[3]) + (x40 * (*_x0).Pose.Rot[0]) + (-1 * x57 * (*_x0).Pose.Rot[2]) +
					(-1 * x46 * (*_x0).Pose.Rot[1]);
	const FLT x61 = 1 + (-2 * ((x60 * x60) + x59));
	const FLT x62 = x61 * x61;
	const FLT x63 = (x40 * (*_x0).Pose.Rot[2]) + (x57 * (*_x0).Pose.Rot[0]) + (-1 * x46 * (*_x0).Pose.Rot[3]) +
					(-1 * x52 * (*_x0).Pose.Rot[1]);
	const FLT x64 = (x40 * (*_x0).Pose.Rot[1]) + (x46 * (*_x0).Pose.Rot[0]) + (x57 * (*_x0).Pose.Rot[3]) +
					(x52 * (*_x0).Pose.Rot[2]);
	const FLT x65 = (x60 * x64) + (x63 * x58);
	const FLT x66 = 2 * t;
	const FLT x67 = x66 * x15;
	const FLT x68 = x66 * x10;
	const FLT x69 = x66 * x13;
	const FLT x70 = x67 + x69 + x68;
	const FLT x71 = 0.25 * x70;
	const FLT x72 = x71 * x22;
	const FLT x73 = t * x12;
	const FLT x74 = x73 * x72;
	const FLT x75 = 0.5 * x70 * x24;
	const FLT x76 = x70 * x21 * (1. / (x17 * x17));
	const FLT x77 = x20 * (1. / (x17 * sqrt(x17)));
	const FLT x78 = x75 * x77;
	const FLT x79 = ((x78 * x11) + (x78 * x14) + (x69 * x23) + (x67 * x23) + (x68 * x23) + (-1 * x76 * x16) +
					 (-1 * x75 * x27) + (-1 * x76 * x11) + (-1 * x76 * x14) + (x78 * x16)) *
					(1. / (x25 * sqrt(x25)));
	const FLT x80 = x79 * x24;
	const FLT x81 = 1.0 / 2.0 * x80;
	const FLT x82 = 1.0 / 2.0 * x37;
	const FLT x83 = x82 * x73;
	const FLT x84 = x79 * x27;
	const FLT x85 = t * x3;
	const FLT x86 = x85 * x72;
	const FLT x87 = 1.0 / 2.0 * x84;
	const FLT x88 = x85 * x87;
	const FLT x89 = x70 * x77 * x26;
	const FLT x90 = 1.0 / 2.0 * x89;
	const FLT x91 = x85 * x90;
	const FLT x92 = t * x9;
	const FLT x93 = x92 * x90;
	const FLT x94 = x87 * x92;
	const FLT x95 = x72 * x92;
	const FLT x96 = (-1 * x83 * x89) + (-1 * x36) + (-1 * x83 * x84) + (-1 * x7 * x91) + (x94 * x34) + (x74 * x43) +
					(-1 * x50 * x95) + (x86 * x54) + x30 + (-1 * x71 * x44) + (-1 * x81 * x31) + x39 + (-1 * x7 * x88) +
					(x93 * x34);
	const FLT x97 = x82 * x84;
	const FLT x98 = x72 * x33;
	const FLT x99 = x73 * x90;
	const FLT x100 = x90 * x31;
	const FLT x101 = x82 * x89;
	const FLT x102 = x87 * x73;
	const FLT x103 = (-1 * x94 * x31) + (x92 * x98) + (-1 * x74 * x54) + (-1 * x85 * x97) + (x86 * x43) + (-1 * x48) +
					 (-1 * x81 * x34) + (x7 * x99) + (-1 * x71 * x35) + (x7 * x102) + x51 + x49 + (-1 * x92 * x100) +
					 (-1 * x85 * x101);
	const FLT x104 = x87 * x31;
	const FLT x105 = (-1 * x55) + (x74 * x50) + (-1 * x99 * x34) + (x85 * x104) + (x85 * x100) + (x95 * x43) +
					 (-1 * x92 * x97) + (-1 * x71 * x29) + (-1 * x34 * x102) + (-1 * x85 * x98) + (-1 * x92 * x101) +
					 x56 + (-1 * x7 * x81) + x53;
	const FLT x106 = (x73 * x100) + (x91 * x34) + (-1 * x73 * x98) + (-1 * x80 * x82) + (x7 * x94) + (x73 * x104) +
					 (-1 * x41) + (-1 * x45) + (x7 * x93) + (x88 * x34) + (-1 * x42) + (-1 * x86 * x50) +
					 (-1 * x54 * x95) + (-1 * x71 * x38);
	const FLT x107 = (-1 * x106 * (*_x0).Pose.Rot[1]) + (x96 * (*_x0).Pose.Rot[0]) + (-1 * x105 * (*_x0).Pose.Rot[2]) +
					 (x103 * (*_x0).Pose.Rot[3]);
	const FLT x108 = (x106 * (*_x0).Pose.Rot[0]) + (x103 * (*_x0).Pose.Rot[2]) + (x96 * (*_x0).Pose.Rot[1]) +
					 (x105 * (*_x0).Pose.Rot[3]);
	const FLT x109 = (-1 * x106 * (*_x0).Pose.Rot[2]) + (-1 * x96 * (*_x0).Pose.Rot[3]) + (x105 * (*_x0).Pose.Rot[1]) +
					 (x103 * (*_x0).Pose.Rot[0]);
	const FLT x110 = (-1 * x106 * (*_x0).Pose.Rot[3]) + (x96 * (*_x0).Pose.Rot[2]) + (-1 * x103 * (*_x0).Pose.Rot[1]) +
					 (x105 * (*_x0).Pose.Rot[0]);
	const FLT x111 = -4 * x58 * x109;
	const FLT x112 = 1 + (-2 * (x59 + (x63 * x63)));
	const FLT x113 = x112 * x112;
	const FLT x114 = (x63 * x64) + (x60 * x58);
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[0]) / sizeof(FLT), 0,
						(*_x0).Velocity.Pos[0] + (*error_model).Velocity.Pos[0] + (t * x0));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[1]) / sizeof(FLT), 0,
						(t * x1) + (*_x0).Velocity.Pos[1] + (*error_model).Velocity.Pos[1]);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.Pos[2]) / sizeof(FLT), 0,
						(*_x0).Velocity.Pos[2] + (*error_model).Velocity.Pos[2] + (t * x2));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT), 0,
						x62 * (1. / (x62 + (4 * (x65 * x65)))) *
							((-2 * (1. / x62) * x65 * ((-4 * x60 * x107) + x111)) +
							 (2 * ((x63 * x109) + (x64 * x107) + (x58 * x110) + (x60 * x108)) * (1. / x61))));
	cnMatrixOptionalSet(
		Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT), 0,
		2 * ((-1 * x63 * x107) + (x58 * x108) + (-1 * x60 * x110) + (x64 * x109)) *
			(1. / sqrt(1 + (-4 * (((x64 * x58) + (-1 * x60 * x63)) * ((x64 * x58) + (-1 * x60 * x63)))))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT), 0,
						x113 * (1. / (x113 + (4 * (x114 * x114)))) *
							((-2 * (1. / x113) * x114 * (x111 + (-4 * x63 * x110))) +
							 (2 * ((x60 * x109) + (x64 * x110) + (x58 * x107) + (x63 * x108)) * (1. / x112))));
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
// <cnkalman.codegen.WrapMember object at 0x7f784d48c0a0>]
static inline void gen_SurviveKalmanModelErrorPredict_jac_x0(CnMat *Hx, const FLT t, const SurviveKalmanModel *_x0,
															 const SurviveKalmanErrorModel *error_model) {
	const FLT x0 = t * t;
	const FLT x1 = 1.0 / 2.0 * x0;
	const FLT x2 = 0.5 * (*error_model).Pose.AxisAngleRot[2];
	const FLT x3 = (*_x0).Velocity.AxisAngleRot[1] + (*error_model).Velocity.AxisAngleRot[1];
	const FLT x4 = (*_x0).Velocity.AxisAngleRot[2] + (*error_model).Velocity.AxisAngleRot[2];
	const FLT x5 = x4 * x4;
	const FLT x6 = x0 * x5;
	const FLT x7 = (*_x0).Velocity.AxisAngleRot[0] + (*error_model).Velocity.AxisAngleRot[0];
	const FLT x8 = x7 * x7;
	const FLT x9 = x0 * x8;
	const FLT x10 = x3 * x3;
	const FLT x11 = x0 * x10;
	const FLT x12 = 1e-10 + x11 + x6 + x9;
	const FLT x13 = sqrt(x12);
	const FLT x14 = 0.5 * x13;
	const FLT x15 = sin(x14);
	const FLT x16 = x15 * x15;
	const FLT x17 = 1. / x12;
	const FLT x18 = x17 * x16;
	const FLT x19 = cos(x14);
	const FLT x20 = (x9 * x18) + (x19 * x19) + (x6 * x18) + (x11 * x18);
	const FLT x21 = 1. / sqrt(x20);
	const FLT x22 = x21 * x15;
	const FLT x23 = 1. / x13;
	const FLT x24 = t * x23;
	const FLT x25 = x24 * x22;
	const FLT x26 = x3 * x25;
	const FLT x27 = x2 * x26;
	const FLT x28 = x21 * x19;
	const FLT x29 = 0.5 * (*error_model).Pose.AxisAngleRot[0];
	const FLT x30 = x28 * x29;
	const FLT x31 = 0.5 * (*error_model).Pose.AxisAngleRot[1];
	const FLT x32 = x4 * x25;
	const FLT x33 = -1 * x32 * x31;
	const FLT x34 = x7 * x25;
	const FLT x35 = x34 + x33;
	const FLT x36 = x35 + x27 + x30;
	const FLT x37 = x31 * x28;
	const FLT x38 = x32 * x29;
	const FLT x39 = -1 * x2 * x34;
	const FLT x40 = x26 + x39;
	const FLT x41 = x40 + x37 + x38;
	const FLT x42 = x2 * x32;
	const FLT x43 = -1 * x42;
	const FLT x44 = x31 * x26;
	const FLT x45 = -1 * x44;
	const FLT x46 = x34 * x29;
	const FLT x47 = x28 + (-1 * x46);
	const FLT x48 = x47 + x43 + x45;
	const FLT x49 = x31 * x34;
	const FLT x50 = x2 * x28;
	const FLT x51 = -1 * x29 * x26;
	const FLT x52 = x32 + x51;
	const FLT x53 = x52 + x49 + x50;
	const FLT x54 =
		(x2 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[3] + (-1 * x29 * (*_x0).Pose.Rot[2]) + (x31 * (*_x0).Pose.Rot[1]);
	const FLT x55 = x54 * x22;
	const FLT x56 = x55 * x24;
	const FLT x57 = x4 * x56;
	const FLT x58 =
		(-1 * x2 * (*_x0).Pose.Rot[1]) + (x31 * (*_x0).Pose.Rot[0]) + (x29 * (*_x0).Pose.Rot[3]) + (*_x0).Pose.Rot[2];
	const FLT x59 = x58 * x26;
	const FLT x60 = (-1 * x31 * (*_x0).Pose.Rot[2]) + (*_x0).Pose.Rot[0] + (-1 * x29 * (*_x0).Pose.Rot[1]) +
					(-1 * x2 * (*_x0).Pose.Rot[3]);
	const FLT x61 = x60 * x28;
	const FLT x62 =
		(x29 * (*_x0).Pose.Rot[0]) + (*_x0).Pose.Rot[1] + (-1 * x31 * (*_x0).Pose.Rot[3]) + (x2 * (*_x0).Pose.Rot[2]);
	const FLT x63 = x62 * x25;
	const FLT x64 = x7 * x63;
	const FLT x65 = x61 + (-1 * x57) + (-1 * x64) + (-1 * x59);
	const FLT x66 = (x48 * (*_x0).Pose.Rot[0]) + (x53 * (*_x0).Pose.Rot[3]) + (x36 * (*_x0).Pose.Rot[1]) + x65 +
					(x41 * (*_x0).Pose.Rot[2]);
	const FLT x67 = x60 * x25;
	const FLT x68 = x4 * x67;
	const FLT x69 = x54 * x28;
	const FLT x70 = x62 * x26;
	const FLT x71 = x58 * x25;
	const FLT x72 = x7 * x71;
	const FLT x73 = x72 + (-1 * x70) + x68 + x69;
	const FLT x74 = x3 * x56;
	const FLT x75 = x62 * x28;
	const FLT x76 = x58 * x32;
	const FLT x77 = x7 * x67;
	const FLT x78 = x77 + (-1 * x76) + x74 + x75;
	const FLT x79 = x7 * x56;
	const FLT x80 = x62 * x32;
	const FLT x81 = x58 * x28;
	const FLT x82 = x3 * x67;
	const FLT x83 = x82 + x81 + (-1 * x79) + x80;
	const FLT x84 = (x83 * (*_x0).Pose.Rot[3]) + (x78 * (*_x0).Pose.Rot[0]) + (-1 * x73 * (*_x0).Pose.Rot[2]) +
					(-1 * x65 * (*_x0).Pose.Rot[1]);
	const FLT x85 = (x78 * (*_x0).Pose.Rot[1]) + (x65 * (*_x0).Pose.Rot[0]) + (x73 * (*_x0).Pose.Rot[3]) +
					(x83 * (*_x0).Pose.Rot[2]);
	const FLT x86 = x78 + (-1 * x53 * (*_x0).Pose.Rot[2]) + (x36 * (*_x0).Pose.Rot[0]) + (x41 * (*_x0).Pose.Rot[3]) +
					(-1 * x48 * (*_x0).Pose.Rot[1]);
	const FLT x87 = (x41 * (*_x0).Pose.Rot[0]) + x83 + (-1 * x36 * (*_x0).Pose.Rot[3]) +
					(-1 * x48 * (*_x0).Pose.Rot[2]) + (x53 * (*_x0).Pose.Rot[1]);
	const FLT x88 = (x78 * (*_x0).Pose.Rot[2]) + (x73 * (*_x0).Pose.Rot[0]) + (-1 * x65 * (*_x0).Pose.Rot[3]) +
					(-1 * x83 * (*_x0).Pose.Rot[1]);
	const FLT x89 = (x83 * (*_x0).Pose.Rot[0]) + (-1 * x78 * (*_x0).Pose.Rot[3]) + (x73 * (*_x0).Pose.Rot[1]) +
					(-1 * x65 * (*_x0).Pose.Rot[2]);
	const FLT x90 = x73 + (x53 * (*_x0).Pose.Rot[0]) + (-1 * x48 * (*_x0).Pose.Rot[3]) + (x36 * (*_x0).Pose.Rot[2]) +
					(-1 * x41 * (*_x0).Pose.Rot[1]);
	const FLT x91 = x89 * x89;
	const FLT x92 = 1 + (-2 * ((x84 * x84) + x91));
	const FLT x93 = 2 * (1. / x92);
	const FLT x94 = 4 * x89;
	const FLT x95 = -1 * x87 * x94;
	const FLT x96 = 4 * x84;
	const FLT x97 = x92 * x92;
	const FLT x98 = (x84 * x85) + (x88 * x89);
	const FLT x99 = 2 * x98 * (1. / x97);
	const FLT x100 = x97 * (1. / (x97 + (4 * (x98 * x98))));
	const FLT x101 = -1 * x38;
	const FLT x102 = x39 + (-1 * x26);
	const FLT x103 = x102 + x101 + x37;
	const FLT x104 = -1 * x30;
	const FLT x105 = x33 + (-1 * x34);
	const FLT x106 = x105 + x104 + x27;
	const FLT x107 = -1 * x50;
	const FLT x108 = -1 * x49;
	const FLT x109 = x52 + x107 + x108;
	const FLT x110 = x47 + x42 + x44;
	const FLT x111 = (-1 * x61) + x64 + x59 + x57;
	const FLT x112 = (x110 * (*_x0).Pose.Rot[0]) + x111 + (x109 * (*_x0).Pose.Rot[3]) +
					 (-1 * x103 * (*_x0).Pose.Rot[2]) + (-1 * x106 * (*_x0).Pose.Rot[1]);
	const FLT x113 = x78 + (x106 * (*_x0).Pose.Rot[0]) + (x103 * (*_x0).Pose.Rot[3]) + (x110 * (*_x0).Pose.Rot[1]) +
					 (x109 * (*_x0).Pose.Rot[2]);
	const FLT x114 = x73 + (-1 * x106 * (*_x0).Pose.Rot[2]) + (x109 * (*_x0).Pose.Rot[0]) +
					 (x103 * (*_x0).Pose.Rot[1]) + (-1 * x110 * (*_x0).Pose.Rot[3]);
	const FLT x115 = (x103 * (*_x0).Pose.Rot[0]) + (x110 * (*_x0).Pose.Rot[2]) + x79 +
					 (-1 * x109 * (*_x0).Pose.Rot[1]) + (-1 * x80) + (-1 * x82) + (-1 * x81) +
					 (-1 * x106 * (*_x0).Pose.Rot[3]);
	const FLT x116 = -1 * x94 * x114;
	const FLT x117 = x51 + (-1 * x32);
	const FLT x118 = x117 + x108 + x50;
	const FLT x119 = x46 + x28;
	const FLT x120 = x45 + x119 + x42;
	const FLT x121 = -1 * x37;
	const FLT x122 = x102 + x121 + x38;
	const FLT x123 = -1 * x27;
	const FLT x124 = x104 + x35 + x123;
	const FLT x125 = (-1 * x69) + (x120 * (*_x0).Pose.Rot[3]) + (-1 * x72) + (-1 * x124 * (*_x0).Pose.Rot[2]) +
					 (x118 * (*_x0).Pose.Rot[0]) + (-1 * x122 * (*_x0).Pose.Rot[1]) + (-1 * x68) + x70;
	const FLT x126 = x83 + (x120 * (*_x0).Pose.Rot[2]) + (x122 * (*_x0).Pose.Rot[0]) + (x118 * (*_x0).Pose.Rot[1]) +
					 (x124 * (*_x0).Pose.Rot[3]);
	const FLT x127 = (x120 * (*_x0).Pose.Rot[0]) + x111 + (-1 * x118 * (*_x0).Pose.Rot[3]) +
					 (x124 * (*_x0).Pose.Rot[1]) + (-1 * x122 * (*_x0).Pose.Rot[2]);
	const FLT x128 = x78 + (x124 * (*_x0).Pose.Rot[0]) + (-1 * x122 * (*_x0).Pose.Rot[3]) +
					 (x118 * (*_x0).Pose.Rot[2]) + (-1 * x120 * (*_x0).Pose.Rot[1]);
	const FLT x129 = -1 * x94 * x127;
	const FLT x130 = x40 + x101 + x121;
	const FLT x131 = x49 + x117 + x107;
	const FLT x132 = x119 + x43 + x44;
	const FLT x133 = x105 + x123 + x30;
	const FLT x134 = x83 + (x133 * (*_x0).Pose.Rot[3]) + (-1 * x132 * (*_x0).Pose.Rot[2]) +
					 (x130 * (*_x0).Pose.Rot[0]) + (-1 * x131 * (*_x0).Pose.Rot[1]);
	const FLT x135 = (-1 * x74) + (-1 * x130 * (*_x0).Pose.Rot[3]) + (-1 * x75) + (-1 * x131 * (*_x0).Pose.Rot[2]) +
					 x76 + (x133 * (*_x0).Pose.Rot[0]) + (x132 * (*_x0).Pose.Rot[1]) + (-1 * x77);
	const FLT x136 = x73 + (x130 * (*_x0).Pose.Rot[1]) + (x132 * (*_x0).Pose.Rot[3]) + (x131 * (*_x0).Pose.Rot[0]) +
					 (x133 * (*_x0).Pose.Rot[2]);
	const FLT x137 = x111 + (-1 * x133 * (*_x0).Pose.Rot[1]) + (x132 * (*_x0).Pose.Rot[0]) +
					 (-1 * x131 * (*_x0).Pose.Rot[3]) + (x130 * (*_x0).Pose.Rot[2]);
	const FLT x138 = -1 * x94 * x135;
	const FLT x139 = t * t * t;
	const FLT x140 = 1. / (x12 * sqrt(x12));
	const FLT x141 = x139 * x140;
	const FLT x142 = x22 * x141;
	const FLT x143 = x7 * x142;
	const FLT x144 = x4 * x62;
	const FLT x145 = x144 * x143;
	const FLT x146 = x55 * x141;
	const FLT x147 = x7 * x7 * x7;
	const FLT x148 = t * t * t * t;
	const FLT x149 = 1.0 * x19;
	const FLT x150 = x15 * x149;
	const FLT x151 = x140 * x150;
	const FLT x152 = x148 * x151;
	const FLT x153 = (1. / (x12 * x12)) * x16;
	const FLT x154 = 2 * x153;
	const FLT x155 = x148 * x154;
	const FLT x156 = x5 * x148;
	const FLT x157 = x0 * x18;
	const FLT x158 = 2 * x157;
	const FLT x159 = x10 * x152;
	const FLT x160 = x7 * x155;
	const FLT x161 = x0 * x23;
	const FLT x162 = x7 * x161;
	const FLT x163 = (-1 * x5 * x160) + (x147 * x152) + (-1 * x10 * x160) + (x7 * x159) + (-1 * x162 * x150) +
					 (-1 * x147 * x155) + (x7 * x151 * x156) + (x7 * x158);
	const FLT x164 = 1.0 / 2.0 * (1. / (x20 * sqrt(x20)));
	const FLT x165 = x24 * x164;
	const FLT x166 = x15 * x165;
	const FLT x167 = x166 * x163;
	const FLT x168 = x7 * x167;
	const FLT x169 = x58 * x22;
	const FLT x170 = 0.5 * x169;
	const FLT x171 = 0.5 * x17 * x139;
	const FLT x172 = x69 * x171;
	const FLT x173 = x3 * x15;
	const FLT x174 = x165 * x173;
	const FLT x175 = x163 * x174;
	const FLT x176 = -1 * x56;
	const FLT x177 = x19 * x164;
	const FLT x178 = x163 * x177;
	const FLT x179 = x75 * x171;
	const FLT x180 = x4 * x7;
	const FLT x181 = x179 * x180;
	const FLT x182 = x60 * x143;
	const FLT x183 = x61 * x171;
	const FLT x184 = x3 * x7;
	const FLT x185 = (x183 * x184) + (-1 * x3 * x182);
	const FLT x186 = (-1 * x58 * x178) + (-1 * x60 * x175) + (x8 * x146) + (-1 * x145) + x185 + (-1 * x167 * x144) +
					 x181 + (-1 * x162 * x170) + x176 + (x54 * x168) + (-1 * x8 * x172);
	const FLT x187 = x8 * x142;
	const FLT x188 = 0.5 * x162;
	const FLT x189 = x62 * x22;
	const FLT x190 = x172 * x184;
	const FLT x191 = x184 * x146;
	const FLT x192 = x169 * x141;
	const FLT x193 = x4 * x192;
	const FLT x194 = x7 * x193;
	const FLT x195 = x81 * x171;
	const FLT x196 = x180 * x195;
	const FLT x197 = x4 * x58;
	const FLT x198 = (-1 * x62 * x178) + (-1 * x60 * x168) + x190 + (x8 * x183) + x194 + (-1 * x60 * x187) +
					 (-1 * x196) + x67 + (-1 * x191) + (-1 * x189 * x188) + (-1 * x54 * x175) + (x167 * x197);
	const FLT x199 = x4 * x60;
	const FLT x200 = (-1 * x179 * x184) + (x3 * x62 * x143);
	const FLT x201 = (x180 * x183) + (-1 * x4 * x182);
	const FLT x202 = x201 + (-1 * x8 * x192) + (x8 * x195) + x71 + (x62 * x175) + (-1 * x55 * x188) +
					 (-1 * x167 * x199) + x200 + (-1 * x58 * x168) + (-1 * x54 * x178);
	const FLT x203 = -1 * x63;
	const FLT x204 = x4 * x54;
	const FLT x205 = x184 * x195;
	const FLT x206 = x184 * x192;
	const FLT x207 = x60 * x22;
	const FLT x208 = x4 * x146;
	const FLT x209 = (x7 * x208) + (-1 * x172 * x180);
	const FLT x210 = (x62 * x168) + (-1 * x8 * x179) + (x62 * x187) + (x58 * x175) + (x204 * x167) +
					 (-1 * x207 * x188) + (-1 * x205) + x209 + x203 + (-1 * x60 * x178) + x206;
	const FLT x211 = (-1 * x210 * (*_x0).Pose.Rot[1]) + (-1 * x202 * (*_x0).Pose.Rot[2]) + (x186 * (*_x0).Pose.Rot[3]) +
					 (x198 * (*_x0).Pose.Rot[0]);
	const FLT x212 = (x186 * (*_x0).Pose.Rot[2]) + (x198 * (*_x0).Pose.Rot[1]) + (x202 * (*_x0).Pose.Rot[3]) +
					 (x210 * (*_x0).Pose.Rot[0]);
	const FLT x213 = (-1 * x210 * (*_x0).Pose.Rot[2]) + (x202 * (*_x0).Pose.Rot[1]) + (-1 * x198 * (*_x0).Pose.Rot[3]) +
					 (x186 * (*_x0).Pose.Rot[0]);
	const FLT x214 = (-1 * x186 * (*_x0).Pose.Rot[1]) + (x202 * (*_x0).Pose.Rot[0]) + (x198 * (*_x0).Pose.Rot[2]) +
					 (-1 * x210 * (*_x0).Pose.Rot[3]);
	const FLT x215 = -1 * x94 * x213;
	const FLT x216 = x7 * x166;
	const FLT x217 = x8 * x148;
	const FLT x218 = x173 * x149;
	const FLT x219 = x218 * x140;
	const FLT x220 = 2 * x3;
	const FLT x221 = x220 * x153;
	const FLT x222 = (x3 * x3 * x3) * x148;
	const FLT x223 = (x222 * x151) + (-1 * x222 * x154) + (-1 * x217 * x221) + (-1 * x218 * x161) + (x219 * x217) +
					 (x219 * x156) + (-1 * x221 * x156) + (x220 * x157);
	const FLT x224 = x60 * x223;
	const FLT x225 = x223 * x177;
	const FLT x226 = x58 * x223;
	const FLT x227 = x226 * x166;
	const FLT x228 = x3 * x161;
	const FLT x229 = 0.5 * x228;
	const FLT x230 = x54 * x223;
	const FLT x231 = x4 * x3;
	const FLT x232 = (-1 * x231 * x195) + (x3 * x193);
	const FLT x233 = x232 + x185 + (-1 * x10 * x146) + (-1 * x229 * x189) + (-1 * x230 * x174) + (x10 * x172) +
					 (-1 * x216 * x224) + x56 + (-1 * x62 * x225) + (x4 * x227);
	const FLT x234 = x223 * x166;
	const FLT x235 = x62 * x223;
	const FLT x236 = x10 * x142;
	const FLT x237 = x3 * x142;
	const FLT x238 = (-1 * x237 * x199) + (x231 * x183);
	const FLT x239 = x238 + (x62 * x236) + x205 + x203 + (-1 * x10 * x179) + (-1 * x54 * x225) + (-1 * x206) +
					 (-1 * x55 * x229) + (-1 * x7 * x227) + (-1 * x234 * x199) + (x235 * x174);
	const FLT x240 = x3 * x208;
	const FLT x241 = x231 * x172;
	const FLT x242 = -1 * x71;
	const FLT x243 = (-1 * x207 * x229) + x200 + (-1 * x60 * x225) + (-1 * x10 * x195) + x242 + (x10 * x192) +
					 (x216 * x235) + x240 + (x226 * x174) + (-1 * x241) + (x234 * x204);
	const FLT x244 = x231 * x179;
	const FLT x245 = x237 * x144;
	const FLT x246 = (-1 * x60 * x236) + (x10 * x183) + (-1 * x224 * x174) + (-1 * x190) + x67 + (-1 * x245) + x244 +
					 (x216 * x230) + (-1 * x234 * x144) + x191 + (-1 * x228 * x170) + (-1 * x58 * x225);
	const FLT x247 = (x246 * (*_x0).Pose.Rot[2]) + (x243 * (*_x0).Pose.Rot[0]) + (x233 * (*_x0).Pose.Rot[1]) +
					 (x239 * (*_x0).Pose.Rot[3]);
	const FLT x248 = (-1 * x233 * (*_x0).Pose.Rot[3]) + (-1 * x243 * (*_x0).Pose.Rot[2]) + (x239 * (*_x0).Pose.Rot[1]) +
					 (x246 * (*_x0).Pose.Rot[0]);
	const FLT x249 = (-1 * x243 * (*_x0).Pose.Rot[1]) + (x246 * (*_x0).Pose.Rot[3]) + (-1 * x239 * (*_x0).Pose.Rot[2]) +
					 (x233 * (*_x0).Pose.Rot[0]);
	const FLT x250 = (-1 * x243 * (*_x0).Pose.Rot[3]) + (x233 * (*_x0).Pose.Rot[2]) + (-1 * x246 * (*_x0).Pose.Rot[1]) +
					 (x239 * (*_x0).Pose.Rot[0]);
	const FLT x251 = -1 * x94 * x248;
	const FLT x252 = x4 * x161;
	const FLT x253 = x4 * x4 * x4;
	const FLT x254 = x4 * x155;
	const FLT x255 = (x4 * x159) + (x4 * x158) + (x4 * x217 * x151) + (-1 * x10 * x254) + (-1 * x252 * x150) +
					 (-1 * x8 * x254) + (x253 * x152) + (-1 * x253 * x155);
	const FLT x256 = x60 * x255;
	const FLT x257 = x255 * x166;
	const FLT x258 = x5 * x141;
	const FLT x259 = x22 * x258;
	const FLT x260 = x216 * x255;
	const FLT x261 = x5 * x171;
	const FLT x262 = x255 * x177;
	const FLT x263 = x238 + x63 + (x75 * x261) + (-1 * x256 * x174) + (-1 * x58 * x262) + (-1 * x252 * x170) +
					 (-1 * x62 * x259) + x209 + (-1 * x257 * x144) + (x54 * x260);
	const FLT x264 = x255 * x174;
	const FLT x265 = 0.5 * x252;
	const FLT x266 = (x257 * x197) + (-1 * x5 * x195) + x241 + (-1 * x265 * x189) + (-1 * x240) + x201 +
					 (-1 * x216 * x256) + x242 + (-1 * x54 * x264) + (-1 * x62 * x262) + (x258 * x169);
	const FLT x267 = (-1 * x55 * x265) + x245 + x196 + x67 + (x62 * x264) + (x5 * x183) + (-1 * x58 * x260) +
					 (-1 * x257 * x199) + (-1 * x60 * x259) + (-1 * x244) + (-1 * x54 * x262) + (-1 * x194);
	const FLT x268 = (-1 * x207 * x265) + (x55 * x258) + (x62 * x260) + (x58 * x264) + (-1 * x60 * x262) +
					 (x204 * x257) + x232 + x176 + x145 + (-1 * x69 * x261) + (-1 * x181);
	const FLT x269 = (-1 * x268 * (*_x0).Pose.Rot[1]) + (-1 * x267 * (*_x0).Pose.Rot[2]) + (x263 * (*_x0).Pose.Rot[3]) +
					 (x266 * (*_x0).Pose.Rot[0]);
	const FLT x270 = (x263 * (*_x0).Pose.Rot[2]) + (x267 * (*_x0).Pose.Rot[3]) + (x268 * (*_x0).Pose.Rot[0]) +
					 (x266 * (*_x0).Pose.Rot[1]);
	const FLT x271 = (-1 * x268 * (*_x0).Pose.Rot[2]) + (-1 * x266 * (*_x0).Pose.Rot[3]) + (x267 * (*_x0).Pose.Rot[1]) +
					 (x263 * (*_x0).Pose.Rot[0]);
	const FLT x272 = (-1 * x263 * (*_x0).Pose.Rot[1]) + (-1 * x268 * (*_x0).Pose.Rot[3]) + (x267 * (*_x0).Pose.Rot[0]) +
					 (x266 * (*_x0).Pose.Rot[2]);
	const FLT x273 = -1 * x94 * x271;
	const FLT x274 = 2 * (1. / sqrt(1 + (-4 * (((x89 * x85) + (-1 * x88 * x84)) * ((x89 * x85) + (-1 * x88 * x84))))));
	const FLT x275 = 1 + (-2 * (x91 + (x88 * x88)));
	const FLT x276 = 2 * (1. / x275);
	const FLT x277 = 4 * x88;
	const FLT x278 = x275 * x275;
	const FLT x279 = (x88 * x85) + (x89 * x84);
	const FLT x280 = 2 * x279 * (1. / x278);
	const FLT x281 = x278 * (1. / (x278 + (4 * (x279 * x279))));
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
						x100 * ((-1 * x99 * ((-1 * x86 * x96) + x95)) +
								(((x89 * x90) + (x88 * x87) + (x84 * x66) + (x85 * x86)) * x93)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						x100 * ((-1 * x99 * ((-1 * x96 * x112) + x116)) +
								(((x85 * x112) + (x89 * x115) + (x88 * x114) + (x84 * x113)) * x93)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x100 * ((-1 * x99 * ((-1 * x96 * x125) + x129)) +
								(((x89 * x128) + (x88 * x127) + (x85 * x125) + (x84 * x126)) * x93)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						x100 * ((-1 * x99 * ((-1 * x96 * x134) + x138)) +
								(((x89 * x137) + (x85 * x134) + (x84 * x136) + (x88 * x135)) * x93)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x100 * ((-1 * x99 * ((-1 * x96 * x211) + x215)) +
								(((x88 * x213) + (x85 * x211) + (x89 * x214) + (x84 * x212)) * x93)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x100 * ((-1 * x99 * ((-1 * x96 * x249) + x251)) +
								(((x89 * x250) + (x85 * x249) + (x84 * x247) + (x88 * x248)) * x93)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x100 * ((-1 * x99 * ((-1 * x96 * x269) + x273)) +
								(((x89 * x272) + (x88 * x271) + (x85 * x269) + (x84 * x270)) * x93)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						((-1 * x88 * x86) + (-1 * x84 * x90) + (x85 * x87) + (x89 * x66)) * x274);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						((x85 * x114) + (x89 * x113) + (-1 * x88 * x112) + (-1 * x84 * x115)) * x274);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						((-1 * x88 * x125) + (-1 * x84 * x128) + (x85 * x127) + (x89 * x126)) * x274);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						((-1 * x88 * x134) + (-1 * x84 * x137) + (x89 * x136) + (x85 * x135)) * x274);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						((-1 * x88 * x211) + (-1 * x84 * x214) + (x85 * x213) + (x89 * x212)) * x274);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						((-1 * x88 * x249) + (-1 * x84 * x250) + (x85 * x248) + (x89 * x247)) * x274);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						((-1 * x88 * x269) + (x89 * x270) + (-1 * x84 * x272) + (x85 * x271)) * x274);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[0]) / sizeof(FLT),
						x281 * ((-1 * x280 * (x95 + (-1 * x90 * x277))) +
								(((x89 * x86) + (x84 * x87) + (x85 * x90) + (x88 * x66)) * x276)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[1]) / sizeof(FLT),
						x281 * ((-1 * x280 * (x116 + (-1 * x277 * x115))) +
								(((x89 * x112) + (x84 * x114) + (x85 * x115) + (x88 * x113)) * x276)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[2]) / sizeof(FLT),
						x281 * ((-1 * x280 * (x129 + (-1 * x277 * x128))) +
								(((x84 * x127) + (x85 * x128) + (x89 * x125) + (x88 * x126)) * x276)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Pose.Rot[3]) / sizeof(FLT),
						x281 * ((-1 * x280 * (x138 + (-1 * x277 * x137))) +
								(((x89 * x134) + (x88 * x136) + (x85 * x137) + (x84 * x135)) * x276)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x281 * ((-1 * x280 * (x215 + (-1 * x214 * x277))) +
								(((x89 * x211) + (x84 * x213) + (x85 * x214) + (x88 * x212)) * x276)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x281 * ((-1 * x280 * (x251 + (-1 * x277 * x250))) +
								(((x89 * x249) + (x84 * x248) + (x88 * x247) + (x85 * x250)) * x276)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x281 * ((-1 * x280 * (x273 + (-1 * x277 * x272))) +
								(((x89 * x269) + (x85 * x272) + (x88 * x270) + (x84 * x271)) * x276)));
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
// <cnkalman.codegen.WrapMember object at 0x7f784d48c0a0>] Jacobian of SurviveKalmanModelErrorPredict wrt
// [(*error_model).AccBias[0], (*error_model).AccBias[1], (*error_model).AccBias[2], (*error_model).Acc[0],
// (*error_model).Acc[1], (*error_model).Acc[2], (*error_model).GyroBias[0], (*error_model).GyroBias[1],
// (*error_model).GyroBias[2], (*error_model).IMUCorrection[0], (*error_model).IMUCorrection[1],
// (*error_model).IMUCorrection[2], (*error_model).IMUCorrection[3], (*error_model).Pose.AxisAngleRot[0],
// (*error_model).Pose.AxisAngleRot[1], (*error_model).Pose.AxisAngleRot[2], (*error_model).Pose.Pos[0],
// (*error_model).Pose.Pos[1], (*error_model).Pose.Pos[2], (*error_model).Velocity.AxisAngleRot[0],
// (*error_model).Velocity.AxisAngleRot[1], (*error_model).Velocity.AxisAngleRot[2], (*error_model).Velocity.Pos[0],
// (*error_model).Velocity.Pos[1], (*error_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at
// 0x7f784d48cdf0>]
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
	const FLT x36 = x35 * x25;
	const FLT x37 = x36 * x27;
	const FLT x38 = (-1 * x34 * x12) + (x2 * x29) + (x9 * x37) + x32;
	const FLT x39 = x30 * x28;
	const FLT x40 = x6 * x31;
	const FLT x41 = (-1 * x9 * x34) + x40 + (-1 * x2 * x39) + (-1 * x37 * x12);
	const FLT x42 = x31 * x33;
	const FLT x43 = (x39 * x12) + (x9 * x29) + (-1 * x2 * x37) + x42;
	const FLT x44 = x31 * x35;
	const FLT x45 = (x29 * x12) + (-1 * x9 * x39) + x44 + (x2 * x34);
	const FLT x46 = (x45 * (*_x0).Pose.Rot[3]) + (-1 * x38 * (*_x0).Pose.Rot[2]) + (x43 * (*_x0).Pose.Rot[0]) +
					(-1 * x41 * (*_x0).Pose.Rot[1]);
	const FLT x47 = 0.5 * x28;
	const FLT x48 = x9 * x47;
	const FLT x49 = x48 * (*_x0).Pose.Rot[3];
	const FLT x50 = x47 * x12;
	const FLT x51 = x50 * (*_x0).Pose.Rot[0];
	const FLT x52 = x2 * x47;
	const FLT x53 = x52 * (*_x0).Pose.Rot[1];
	const FLT x54 = 0.5 * x31;
	const FLT x55 = x54 * (*_x0).Pose.Rot[2];
	const FLT x56 = (-1 * x53) + x49 + (-1 * x55) + (-1 * x51);
	const FLT x57 = x56 * (*_x0).Pose.Rot[3];
	const FLT x58 = x50 * (*_x0).Pose.Rot[1];
	const FLT x59 = x54 * (*_x0).Pose.Rot[3];
	const FLT x60 = x48 * (*_x0).Pose.Rot[2];
	const FLT x61 = x52 * (*_x0).Pose.Rot[0];
	const FLT x62 = x61 + x60 + (-1 * x58) + x59;
	const FLT x63 = x50 * (*_x0).Pose.Rot[3];
	const FLT x64 = x48 * (*_x0).Pose.Rot[0];
	const FLT x65 = x54 * (*_x0).Pose.Rot[1];
	const FLT x66 = x52 * (*_x0).Pose.Rot[2];
	const FLT x67 = x66 + (-1 * x65) + (-1 * x63) + (-1 * x64);
	const FLT x68 = (x54 * (*_x0).Pose.Rot[0]) + (-1 * x48 * (*_x0).Pose.Rot[1]) + (-1 * x50 * (*_x0).Pose.Rot[2]) +
					(-1 * x52 * (*_x0).Pose.Rot[3]);
	const FLT x69 = x68 * (*_x0).Pose.Rot[1];
	const FLT x70 = x69 + (x67 * (*_x0).Pose.Rot[0]);
	const FLT x71 = x57 + x70 + (x62 * (*_x0).Pose.Rot[2]);
	const FLT x72 = (x41 * (*_x0).Pose.Rot[0]) + (x43 * (*_x0).Pose.Rot[1]) + (x38 * (*_x0).Pose.Rot[3]) +
					(x45 * (*_x0).Pose.Rot[2]);
	const FLT x73 = x68 * (*_x0).Pose.Rot[0];
	const FLT x74 = -1 * x56 * (*_x0).Pose.Rot[2];
	const FLT x75 = -1 * x67 * (*_x0).Pose.Rot[1];
	const FLT x76 = x75 + x74 + (x62 * (*_x0).Pose.Rot[3]) + x73;
	const FLT x77 = (x38 * (*_x0).Pose.Rot[0]) + (x43 * (*_x0).Pose.Rot[2]) + (-1 * x41 * (*_x0).Pose.Rot[3]) +
					(-1 * x45 * (*_x0).Pose.Rot[1]);
	const FLT x78 = x56 * (*_x0).Pose.Rot[1];
	const FLT x79 = x68 * (*_x0).Pose.Rot[3];
	const FLT x80 = x67 * (*_x0).Pose.Rot[2];
	const FLT x81 = (-1 * x80) + (x62 * (*_x0).Pose.Rot[0]) + x78 + (-1 * x79);
	const FLT x82 = (x38 * (*_x0).Pose.Rot[1]) + (-1 * x43 * (*_x0).Pose.Rot[3]) + (x45 * (*_x0).Pose.Rot[0]) +
					(-1 * x41 * (*_x0).Pose.Rot[2]);
	const FLT x83 = x67 * (*_x0).Pose.Rot[3];
	const FLT x84 = x68 * (*_x0).Pose.Rot[2];
	const FLT x85 = x84 + (x56 * (*_x0).Pose.Rot[0]);
	const FLT x86 = x85 + (-1 * x83) + (-1 * x62 * (*_x0).Pose.Rot[1]);
	const FLT x87 = x82 * x82;
	const FLT x88 = 1 + (-2 * ((x46 * x46) + x87));
	const FLT x89 = 2 * (1. / x88);
	const FLT x90 = 4 * x82;
	const FLT x91 = -1 * x81 * x90;
	const FLT x92 = 4 * x46;
	const FLT x93 = x88 * x88;
	const FLT x94 = (x72 * x46) + (x82 * x77);
	const FLT x95 = 2 * (1. / x93) * x94;
	const FLT x96 = x93 * (1. / (x93 + (4 * (x94 * x94))));
	const FLT x97 = x65 + (-1 * x66) + x64 + x63;
	const FLT x98 = x58 + (-1 * x61) + (-1 * x60) + (-1 * x59);
	const FLT x99 = (x98 * (*_x0).Pose.Rot[0]) + x79;
	const FLT x100 = x99 + (-1 * x97 * (*_x0).Pose.Rot[2]) + (-1 * x78);
	const FLT x101 = x98 * (*_x0).Pose.Rot[1];
	const FLT x102 = x85 + x101 + (x97 * (*_x0).Pose.Rot[3]);
	const FLT x103 = (-1 * x98 * (*_x0).Pose.Rot[3]) + x73;
	const FLT x104 = x103 + (x97 * (*_x0).Pose.Rot[1]) + x74;
	const FLT x105 = x98 * (*_x0).Pose.Rot[2];
	const FLT x106 = (-1 * x69) + x105 + (x97 * (*_x0).Pose.Rot[0]) + (-1 * x57);
	const FLT x107 = -1 * x90 * x104;
	const FLT x108 = x55 + x51 + (-1 * x49) + x53;
	const FLT x109 = x99 + (x108 * (*_x0).Pose.Rot[1]) + x80;
	const FLT x110 = x70 + (-1 * x108 * (*_x0).Pose.Rot[3]) + (-1 * x105);
	const FLT x111 = (-1 * x101) + (x108 * (*_x0).Pose.Rot[0]) + x83 + (-1 * x84);
	const FLT x112 = x103 + (x108 * (*_x0).Pose.Rot[2]) + x75;
	const FLT x113 = -1 * x90 * x110;
	const FLT x114 = -1 * x39;
	const FLT x115 = t * t * t;
	const FLT x116 = 1. / (x15 * sqrt(x15));
	const FLT x117 = x115 * x116;
	const FLT x118 = x2 * x117;
	const FLT x119 = x33 * x25;
	const FLT x120 = x9 * x119;
	const FLT x121 = x118 * x120;
	const FLT x122 = x30 * x25;
	const FLT x123 = x10 * x117;
	const FLT x124 = t * t * t * t;
	const FLT x125 = (x9 * x9 * x9) * x124;
	const FLT x126 = 1.0 * x22;
	const FLT x127 = x18 * x126;
	const FLT x128 = x116 * x127;
	const FLT x129 = 2 * (1. / (x15 * x15)) * x19;
	const FLT x130 = x124 * x128;
	const FLT x131 = 2 * x21;
	const FLT x132 = x0 * x131;
	const FLT x133 = x9 * x13;
	const FLT x134 = x124 * x129;
	const FLT x135 = x7 * x134;
	const FLT x136 = x0 * x26;
	const FLT x137 = x9 * x136;
	const FLT x138 = (-1 * x133 * x134) + (-1 * x127 * x137) + (-1 * x9 * x135) + (-1 * x125 * x129) + (x128 * x125) +
					 (x7 * x9 * x130) + (x9 * x132) + (x130 * x133);
	const FLT x139 = 1.0 / 2.0 * (1. / (x23 * sqrt(x23)));
	const FLT x140 = x27 * x139;
	const FLT x141 = x18 * x140;
	const FLT x142 = x138 * x141;
	const FLT x143 = x9 * x142;
	const FLT x144 = 0.5 * x36;
	const FLT x145 = 0.5 * x20 * x115;
	const FLT x146 = x32 * x145;
	const FLT x147 = x12 * x18;
	const FLT x148 = x6 * x140;
	const FLT x149 = x148 * x147;
	const FLT x150 = x22 * x139;
	const FLT x151 = x138 * x150;
	const FLT x152 = x42 * x145;
	const FLT x153 = x2 * x9;
	const FLT x154 = x152 * x153;
	const FLT x155 = x2 * x33;
	const FLT x156 = x6 * x25;
	const FLT x157 = x12 * x117;
	const FLT x158 = x40 * x145;
	const FLT x159 = x9 * x12;
	const FLT x160 = (x158 * x159) + (-1 * x9 * x156 * x157);
	const FLT x161 = (-1 * x142 * x155) + (-1 * x138 * x149) + (-1 * x35 * x151) + (-1 * x10 * x146) + x160 +
					 (-1 * x121) + (-1 * x137 * x144) + x114 + x154 + (x123 * x122) + (x30 * x143);
	const FLT x162 = 0.5 * x137;
	const FLT x163 = x146 * x159;
	const FLT x164 = x9 * x122;
	const FLT x165 = x164 * x157;
	const FLT x166 = x9 * x118;
	const FLT x167 = x36 * x166;
	const FLT x168 = x140 * x147;
	const FLT x169 = x168 * x138;
	const FLT x170 = x18 * x148;
	const FLT x171 = x9 * x170;
	const FLT x172 = x44 * x145;
	const FLT x173 = x172 * x153;
	const FLT x174 = x2 * x35;
	const FLT x175 = (-1 * x33 * x151) + x163 + (-1 * x165) + (-1 * x123 * x156) + (-1 * x119 * x162) + (x10 * x158) +
					 x29 + (-1 * x171 * x138) + (-1 * x30 * x169) + x167 + (-1 * x173) + (x174 * x142);
	const FLT x176 = x2 * x170;
	const FLT x177 = x33 * x168;
	const FLT x178 = (x153 * x158) + (-1 * x166 * x156);
	const FLT x179 = (-1 * x152 * x159) + (x120 * x157);
	const FLT x180 = x179 + x178 + (-1 * x36 * x123) + x37 + (-1 * x122 * x162) + (x10 * x172) + (-1 * x35 * x143) +
					 (x177 * x138) + (-1 * x176 * x138) + (-1 * x30 * x151);
	const FLT x181 = x2 * x30;
	const FLT x182 = -1 * x34;
	const FLT x183 = x172 * x159;
	const FLT x184 = x36 * x12;
	const FLT x185 = x9 * x117 * x184;
	const FLT x186 = x9 * x33;
	const FLT x187 = (-1 * x146 * x153) + (x118 * x164);
	const FLT x188 = x187 + (x119 * x123) + (x35 * x169) + x182 + (-1 * x6 * x151) + (x186 * x142) +
					 (-1 * x162 * x156) + (x181 * x142) + (-1 * x10 * x152) + x185 + (-1 * x183);
	const FLT x189 = (-1 * x188 * (*_x0).Pose.Rot[1]) + (-1 * x180 * (*_x0).Pose.Rot[2]) + (x161 * (*_x0).Pose.Rot[3]) +
					 (x175 * (*_x0).Pose.Rot[0]);
	const FLT x190 = (x161 * (*_x0).Pose.Rot[2]) + (x180 * (*_x0).Pose.Rot[3]) + (x175 * (*_x0).Pose.Rot[1]) +
					 (x188 * (*_x0).Pose.Rot[0]);
	const FLT x191 = (-1 * x188 * (*_x0).Pose.Rot[2]) + (-1 * x175 * (*_x0).Pose.Rot[3]) + (x180 * (*_x0).Pose.Rot[1]) +
					 (x161 * (*_x0).Pose.Rot[0]);
	const FLT x192 = (-1 * x161 * (*_x0).Pose.Rot[1]) + (x175 * (*_x0).Pose.Rot[2]) + (x180 * (*_x0).Pose.Rot[0]) +
					 (-1 * x188 * (*_x0).Pose.Rot[3]);
	const FLT x193 = -1 * x90 * x191;
	const FLT x194 = x126 * x147;
	const FLT x195 = x116 * x124 * x194;
	const FLT x196 = x12 * x12 * x12;
	const FLT x197 = (x10 * x195) + (-1 * x12 * x10 * x134) + (-1 * x194 * x136) + (-1 * x12 * x135) +
					 (-1 * x196 * x134) + (x7 * x195) + (x196 * x130) + (x12 * x132);
	const FLT x198 = x197 * x150;
	const FLT x199 = x35 * x197;
	const FLT x200 = x199 * x141;
	const FLT x201 = 0.5 * x136;
	const FLT x202 = x12 * x119;
	const FLT x203 = x13 * x117;
	const FLT x204 = x30 * x197;
	const FLT x205 = x2 * x12;
	const FLT x206 = (x118 * x184) + (-1 * x205 * x172);
	const FLT x207 = x160 + x206 + (-1 * x204 * x168) + (x13 * x146) + (-1 * x33 * x198) + (-1 * x171 * x197) +
					 (x2 * x200) + (-1 * x203 * x122) + x39 + (-1 * x201 * x202);
	const FLT x208 = x12 * x122;
	const FLT x209 = x12 * x156;
	const FLT x210 = (-1 * x209 * x118) + (x205 * x158);
	const FLT x211 = (-1 * x185) + (x203 * x119) + x183 + (-1 * x13 * x152) + (x177 * x197) + (-1 * x9 * x200) +
					 (-1 * x30 * x198) + x182 + (-1 * x201 * x208) + x210 + (-1 * x176 * x197);
	const FLT x212 = -1 * x37;
	const FLT x213 = x197 * x141;
	const FLT x214 = x208 * x118;
	const FLT x215 = x181 * x141;
	const FLT x216 = x205 * x146;
	const FLT x217 = (-1 * x6 * x198) + (-1 * x13 * x172) + (-1 * x201 * x209) + (-1 * x216) + (x215 * x197) +
					 (x213 * x186) + x212 + x179 + (x168 * x199) + (x36 * x203) + x214;
	const FLT x218 = x205 * x152;
	const FLT x219 = x9 * x141;
	const FLT x220 = x202 * x118;
	const FLT x221 = (-1 * x203 * x156) + (x219 * x204) + x29 + x165 + (x13 * x158) + (-1 * x201 * x184) +
					 (-1 * x35 * x198) + (-1 * x197 * x149) + (-1 * x163) + x218 + (-1 * x220) + (-1 * x213 * x155);
	const FLT x222 = (x221 * (*_x0).Pose.Rot[2]) + (x207 * (*_x0).Pose.Rot[1]) + (x217 * (*_x0).Pose.Rot[0]) +
					 (x211 * (*_x0).Pose.Rot[3]);
	const FLT x223 = (-1 * x217 * (*_x0).Pose.Rot[2]) + (-1 * x207 * (*_x0).Pose.Rot[3]) + (x211 * (*_x0).Pose.Rot[1]) +
					 (x221 * (*_x0).Pose.Rot[0]);
	const FLT x224 = (-1 * x217 * (*_x0).Pose.Rot[1]) + (-1 * x211 * (*_x0).Pose.Rot[2]) + (x221 * (*_x0).Pose.Rot[3]) +
					 (x207 * (*_x0).Pose.Rot[0]);
	const FLT x225 = (-1 * x221 * (*_x0).Pose.Rot[1]) + (-1 * x217 * (*_x0).Pose.Rot[3]) + (x207 * (*_x0).Pose.Rot[2]) +
					 (x211 * (*_x0).Pose.Rot[0]);
	const FLT x226 = -1 * x90 * x223;
	const FLT x227 = x2 * x130;
	const FLT x228 = x0 * x2;
	const FLT x229 = x26 * x228;
	const FLT x230 = x2 * x2 * x2;
	const FLT x231 = x2 * x134;
	const FLT x232 = (x13 * x227) + (x228 * x131) + (-1 * x229 * x127) + (-1 * x230 * x134) + (x10 * x227) +
					 (-1 * x13 * x231) + (x230 * x130) + (-1 * x10 * x231);
	const FLT x233 = x232 * x141;
	const FLT x234 = x7 * x117;
	const FLT x235 = x30 * x232;
	const FLT x236 = x232 * x150;
	const FLT x237 = x187 + x34 + (-1 * x35 * x236) + (x7 * x152) + (-1 * x233 * x155) + (-1 * x232 * x149) + x210 +
					 (-1 * x234 * x119) + (x219 * x235) + (-1 * x229 * x144);
	const FLT x238 = 0.5 * x229;
	const FLT x239 = x178 + x212 + (-1 * x7 * x172) + (-1 * x232 * x171) + (-1 * x214) + (-1 * x33 * x236) +
					 (-1 * x238 * x119) + (-1 * x235 * x168) + (x233 * x174) + x216 + (x36 * x234);
	const FLT x240 = x35 * x232;
	const FLT x241 = (-1 * x238 * x122) + x29 + x220 + (-1 * x218) + (x7 * x158) + (-1 * x234 * x156) +
					 (-1 * x30 * x236) + (-1 * x232 * x176) + (-1 * x219 * x240) + x173 + (x232 * x177) + (-1 * x167);
	const FLT x242 = (-1 * x238 * x156) + (-1 * x7 * x146) + (x240 * x168) + x121 + (x234 * x122) + x114 +
					 (x233 * x186) + (x215 * x232) + x206 + (-1 * x6 * x236) + (-1 * x154);
	const FLT x243 = (-1 * x242 * (*_x0).Pose.Rot[1]) + (x237 * (*_x0).Pose.Rot[3]) + (-1 * x241 * (*_x0).Pose.Rot[2]) +
					 (x239 * (*_x0).Pose.Rot[0]);
	const FLT x244 = (x242 * (*_x0).Pose.Rot[0]) + (x237 * (*_x0).Pose.Rot[2]) + (x241 * (*_x0).Pose.Rot[3]) +
					 (x239 * (*_x0).Pose.Rot[1]);
	const FLT x245 = (-1 * x242 * (*_x0).Pose.Rot[2]) + (-1 * x239 * (*_x0).Pose.Rot[3]) + (x241 * (*_x0).Pose.Rot[1]) +
					 (x237 * (*_x0).Pose.Rot[0]);
	const FLT x246 = (-1 * x242 * (*_x0).Pose.Rot[3]) + (-1 * x237 * (*_x0).Pose.Rot[1]) + (x241 * (*_x0).Pose.Rot[0]) +
					 (x239 * (*_x0).Pose.Rot[2]);
	const FLT x247 = -1 * x90 * x245;
	const FLT x248 = 2 * (1. / sqrt(1 + (-4 * (((x82 * x72) + (-1 * x77 * x46)) * ((x82 * x72) + (-1 * x77 * x46))))));
	const FLT x249 = 1 + (-2 * (x87 + (x77 * x77)));
	const FLT x250 = 2 * (1. / x249);
	const FLT x251 = 4 * x77;
	const FLT x252 = x249 * x249;
	const FLT x253 = (x72 * x77) + (x82 * x46);
	const FLT x254 = 2 * (1. / x252) * x253;
	const FLT x255 = x252 * (1. / (x252 + (4 * (x253 * x253))));
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
						x96 * ((-1 * x95 * ((-1 * x76 * x92) + x91)) +
							   (((x82 * x86) + (x71 * x46) + (x81 * x77) + (x72 * x76)) * x89)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						x96 * ((-1 * x95 * ((-1 * x92 * x100) + x107)) +
							   (((x82 * x106) + (x77 * x104) + (x72 * x100) + (x46 * x102)) * x89)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						x96 * ((-1 * x95 * ((-1 * x92 * x111) + x113)) +
							   (((x82 * x112) + (x46 * x109) + (x72 * x111) + (x77 * x110)) * x89)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x96 * ((-1 * x95 * ((-1 * x92 * x189) + x193)) +
							   (((x82 * x192) + (x77 * x191) + (x72 * x189) + (x46 * x190)) * x89)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x96 * ((-1 * x95 * ((-1 * x92 * x224) + x226)) +
							   (((x82 * x225) + (x72 * x224) + (x46 * x222) + (x77 * x223)) * x89)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x96 * ((-1 * x95 * ((-1 * x92 * x243) + x247)) +
							   (((x82 * x246) + (x77 * x245) + (x72 * x243) + (x46 * x244)) * x89)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						((-1 * x77 * x76) + (x82 * x71) + (-1 * x86 * x46) + (x81 * x72)) * x248);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						((-1 * x46 * x106) + (x72 * x104) + (-1 * x77 * x100) + (x82 * x102)) * x248);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						((-1 * x77 * x111) + (x82 * x109) + (-1 * x46 * x112) + (x72 * x110)) * x248);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						((-1 * x77 * x189) + (-1 * x46 * x192) + (x72 * x191) + (x82 * x190)) * x248);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						((-1 * x77 * x224) + (-1 * x46 * x225) + (x72 * x223) + (x82 * x222)) * x248);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						((-1 * x77 * x243) + (x82 * x244) + (-1 * x46 * x246) + (x72 * x245)) * x248);
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[0]) / sizeof(FLT),
						x255 * ((-1 * x254 * (x91 + (-1 * x86 * x251))) +
								(((x82 * x76) + (x86 * x72) + (x71 * x77) + (x81 * x46)) * x250)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[1]) / sizeof(FLT),
						x255 * ((-1 * x254 * (x107 + (-1 * x251 * x106))) +
								(((x82 * x100) + (x77 * x102) + (x46 * x104) + (x72 * x106)) * x250)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						x255 * ((-1 * x254 * (x113 + (-1 * x251 * x112))) +
								(((x82 * x111) + (x72 * x112) + (x46 * x110) + (x77 * x109)) * x250)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[0]) / sizeof(FLT),
						x255 * ((-1 * x254 * (x193 + (-1 * x251 * x192))) +
								(((x82 * x189) + (x46 * x191) + (x72 * x192) + (x77 * x190)) * x250)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[1]) / sizeof(FLT),
						x255 * ((-1 * x254 * (x226 + (-1 * x251 * x225))) +
								(((x82 * x224) + (x46 * x223) + (x77 * x222) + (x72 * x225)) * x250)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveKalmanErrorModel, Pose.AxisAngleRot[2]) / sizeof(FLT),
						offsetof(SurviveKalmanErrorModel, Velocity.AxisAngleRot[2]) / sizeof(FLT),
						x255 * ((-1 * x254 * (x247 + (-1 * x251 * x246))) +
								(((x72 * x246) + (x77 * x244) + (x82 * x243) + (x46 * x245)) * x250)));
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
// (*error_model).Velocity.Pos[2], <cnkalman.codegen.WrapMember object at 0x7f784d48cdf0>]
