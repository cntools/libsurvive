/// NOTE: This is a generated file; do not edit.
#pragma once
#include <cnkalman/generated_header.h>

// clang-format off
static inline void SurviveIMUBiasModelToErrorModel(SurviveIMUBiasErrorModel* out, const SurviveIMUBiasModel* _x1, const SurviveIMUBiasModel* _x0) {
	const FLT x0 = ((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[1]) + (-1 * (*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[3]) + (-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[2]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[0]);
	const FLT x1 = (-1 * (*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[0]) + (-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[3]) + ((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[2]);
	const FLT x2 = (-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[3]) + ((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[0]) + (-1 * (*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[2]);
	const FLT x3 = ((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[3]) + ((*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[0]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[2]);
	const FLT x4 = x0 * x0;
	out->AccScale[0]=(*_x1).AccScale[0] + (-1 * (*_x0).AccScale[0]);
	out->AccScale[1]=(*_x1).AccScale[1] + (-1 * (*_x0).AccScale[1]);
	out->AccScale[2]=(*_x1).AccScale[2] + (-1 * (*_x0).AccScale[2]);
	out->IMUCorrection[0]=atan2(2 * ((x2 * x3) + (x0 * x1)), 1 + (-2 * ((x2 * x2) + x4)));
	out->IMUCorrection[1]=asin(2 * ((x0 * x3) + (-1 * x2 * x1)));
	out->IMUCorrection[2]=atan2(2 * ((x1 * x3) + (x0 * x2)), 1 + (-2 * (x4 + (x1 * x1))));
	out->AccBias[0]=(*_x1).AccBias[0] + (-1 * (*_x0).AccBias[0]);
	out->AccBias[1]=(*_x1).AccBias[1] + (-1 * (*_x0).AccBias[1]);
	out->AccBias[2]=(*_x1).AccBias[2] + (-1 * (*_x0).AccBias[2]);
	out->GyroBias[0]=(*_x1).GyroBias[0] + (-1 * (*_x0).GyroBias[0]);
	out->GyroBias[1]=(*_x1).GyroBias[1] + (-1 * (*_x0).GyroBias[1]);
	out->GyroBias[2]=(*_x1).GyroBias[2] + (-1 * (*_x0).GyroBias[2]);
}

// Jacobian of SurviveIMUBiasModelToErrorModel wrt [(*_x1).AccBias[0], (*_x1).AccBias[1], (*_x1).AccBias[2], (*_x1).AccScale[0], (*_x1).AccScale[1], (*_x1).AccScale[2], (*_x1).GyroBias[0], (*_x1).GyroBias[1], (*_x1).GyroBias[2], (*_x1).IMUCorrection[0], (*_x1).IMUCorrection[1], (*_x1).IMUCorrection[2], (*_x1).IMUCorrection[3]]
static inline void SurviveIMUBiasModelToErrorModel_jac_x1(CnMat* Hx, const SurviveIMUBiasModel* _x1, const SurviveIMUBiasModel* _x0) {
	const FLT x0 = ((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[1]) + (-1 * (*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[3]) + (-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[2]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[0]);
	const FLT x1 = x0 * (*_x0).IMUCorrection[3];
	const FLT x2 = (-1 * (*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[0]) + (-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[3]) + ((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[2]);
	const FLT x3 = x2 * (*_x0).IMUCorrection[2];
	const FLT x4 = ((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[3]) + ((*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[0]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[2]);
	const FLT x5 = x4 * (*_x0).IMUCorrection[1];
	const FLT x6 = (-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[3]) + ((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[0]) + (-1 * (*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[2]);
	const FLT x7 = x6 * (*_x0).IMUCorrection[0];
	const FLT x8 = x7 + (-1 * x5);
	const FLT x9 = x0 * x0;
	const FLT x10 = 1 + (-2 * ((x6 * x6) + x9));
	const FLT x11 = 1. / x10;
	const FLT x12 = 2 * x11;
	const FLT x13 = x0 * (*_x0).IMUCorrection[2];
	const FLT x14 = 4 * x13;
	const FLT x15 = x6 * (*_x0).IMUCorrection[1];
	const FLT x16 = x10 * x10;
	const FLT x17 = (x4 * x6) + (x0 * x2);
	const FLT x18 = 2 * x17 * (1. / x16);
	const FLT x19 = x16 * (1. / (x16 + (4 * (x17 * x17))));
	const FLT x20 = x2 * (*_x0).IMUCorrection[3];
	const FLT x21 = x15 + (x4 * (*_x0).IMUCorrection[0]);
	const FLT x22 = 2 * (x21 + x13 + (-1 * x20));
	const FLT x23 = 4 * x1;
	const FLT x24 = x4 * (*_x0).IMUCorrection[3];
	const FLT x25 = x6 * (*_x0).IMUCorrection[2];
	const FLT x26 = x0 * (*_x0).IMUCorrection[1];
	const FLT x27 = x2 * (*_x0).IMUCorrection[0];
	const FLT x28 = x27 + (-1 * x26);
	const FLT x29 = x0 * (*_x0).IMUCorrection[0];
	const FLT x30 = -4 * x29;
	const FLT x31 = x6 * (*_x0).IMUCorrection[3];
	const FLT x32 = x4 * (*_x0).IMUCorrection[2];
	const FLT x33 = x2 * (*_x0).IMUCorrection[1];
	const FLT x34 = x29 + x33;
	const FLT x35 = x34 + (-1 * x32) + x31;
	const FLT x36 = -4 * x26;
	const FLT x37 = 1. / sqrt(1 + (-4 * (((x0 * x4) + (-1 * x2 * x6)) * ((x0 * x4) + (-1 * x2 * x6)))));
	const FLT x38 = 2 * x37;
	const FLT x39 = (-1 * x25) + (-1 * x24);
	const FLT x40 = x3 + x1;
	const FLT x41 = 1 + (-2 * (x9 + (x2 * x2)));
	const FLT x42 = 2 * (1. / x41);
	const FLT x43 = x41 * x41;
	const FLT x44 = (x2 * x4) + (x0 * x6);
	const FLT x45 = 2 * (1. / x43) * x44;
	const FLT x46 = x43 * (1. / (x43 + (4 * (x44 * x44))));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccScale[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccScale[0])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccScale[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccScale[1])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccScale[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccScale[2])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[0])/sizeof(FLT), x19 * ((-1 * ((4 * x15) + x14) * x18) + ((x8 + (-1 * x1) + (-1 * x3)) * x12)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[1])/sizeof(FLT), x19 * ((-1 * x18 * ((-4 * x7) + x23)) + (x22 * x11)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[2])/sizeof(FLT), x19 * ((-1 * ((-4 * x31) + x30) * x18) + (x12 * (x28 + x24 + x25))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[3])/sizeof(FLT), x19 * ((-1 * ((4 * x25) + x36) * x18) + (x35 * x12)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[0])/sizeof(FLT), x35 * x38);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[1])/sizeof(FLT), x38 * (x39 + x26 + (-1 * x27)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[2])/sizeof(FLT), x37 * x22);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[3])/sizeof(FLT), x38 * (x5 + x40 + (-1 * x7)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[0])/sizeof(FLT), ((-1 * (x14 + (4 * x20)) * x45) + ((x39 + x28) * x42)) * x46);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[1])/sizeof(FLT), x46 * ((-1 * x45 * (x23 + (-4 * x3))) + (x42 * (x34 + x32 + (-1 * x31)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[2])/sizeof(FLT), x46 * ((-1 * (x30 + (4 * x33)) * x45) + (x42 * (x8 + x40))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[3])/sizeof(FLT), x46 * ((-1 * (x36 + (-4 * x27)) * x45) + (x42 * (x21 + x20 + (-1 * x13)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccBias[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccBias[0])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccBias[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccBias[1])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccBias[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccBias[2])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, GyroBias[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, GyroBias[0])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, GyroBias[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, GyroBias[1])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, GyroBias[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, GyroBias[2])/sizeof(FLT), 1);
}

// Full version Jacobian of SurviveIMUBiasModelToErrorModel wrt [(*_x1).AccBias[0], (*_x1).AccBias[1], (*_x1).AccBias[2], (*_x1).AccScale[0], (*_x1).AccScale[1], (*_x1).AccScale[2], (*_x1).GyroBias[0], (*_x1).GyroBias[1], (*_x1).GyroBias[2], (*_x1).IMUCorrection[0], (*_x1).IMUCorrection[1], (*_x1).IMUCorrection[2], (*_x1).IMUCorrection[3]]
// Jacobian of SurviveIMUBiasModelToErrorModel wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2], (*_x0).AccScale[0], (*_x0).AccScale[1], (*_x0).AccScale[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3]]
static inline void SurviveIMUBiasModelToErrorModel_jac_x0(CnMat* Hx, const SurviveIMUBiasModel* _x1, const SurviveIMUBiasModel* _x0) {
	const FLT x0 = ((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[1]) + (-1 * (*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[3]) + (-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[2]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[0]);
	const FLT x1 = x0 * (*_x1).IMUCorrection[3];
	const FLT x2 = (-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[3]) + ((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[0]) + (-1 * (*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[2]);
	const FLT x3 = x2 * (*_x1).IMUCorrection[0];
	const FLT x4 = x3 + x1;
	const FLT x5 = ((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[3]) + ((*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[0]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[2]);
	const FLT x6 = x5 * (*_x1).IMUCorrection[1];
	const FLT x7 = (-1 * (*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[0]) + (-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[3]) + ((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[2]);
	const FLT x8 = x7 * (*_x1).IMUCorrection[2];
	const FLT x9 = x8 + x6;
	const FLT x10 = x0 * x0;
	const FLT x11 = 1 + (-2 * ((x2 * x2) + x10));
	const FLT x12 = 2 * (1. / x11);
	const FLT x13 = x0 * (*_x1).IMUCorrection[2];
	const FLT x14 = -4 * x13;
	const FLT x15 = x2 * (*_x1).IMUCorrection[1];
	const FLT x16 = x11 * x11;
	const FLT x17 = (x2 * x5) + (x0 * x7);
	const FLT x18 = 2 * x17 * (1. / x16);
	const FLT x19 = x16 * (1. / (x16 + (4 * (x17 * x17))));
	const FLT x20 = x7 * (*_x1).IMUCorrection[3];
	const FLT x21 = x20 + (-1 * x5 * (*_x1).IMUCorrection[0]);
	const FLT x22 = -4 * x1;
	const FLT x23 = x5 * (*_x1).IMUCorrection[3];
	const FLT x24 = x7 * (*_x1).IMUCorrection[0];
	const FLT x25 = x2 * (*_x1).IMUCorrection[2];
	const FLT x26 = x0 * (*_x1).IMUCorrection[1];
	const FLT x27 = x26 + x25;
	const FLT x28 = x0 * (*_x1).IMUCorrection[0];
	const FLT x29 = 4 * x28;
	const FLT x30 = x2 * (*_x1).IMUCorrection[3];
	const FLT x31 = (-1 * x28) + x30;
	const FLT x32 = x5 * (*_x1).IMUCorrection[2];
	const FLT x33 = x7 * (*_x1).IMUCorrection[1];
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
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccScale[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccScale[0])/sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccScale[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccScale[1])/sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccScale[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccScale[2])/sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[0])/sizeof(FLT), ((-1 * ((-4 * x15) + x14) * x18) + ((x9 + x4) * x12)) * x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[1])/sizeof(FLT), x19 * ((-1 * x18 * ((4 * x3) + x22)) + (x12 * (x21 + (-1 * x13) + x15))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[2])/sizeof(FLT), x19 * ((-1 * ((4 * x30) + x29) * x18) + (x12 * ((-1 * x23) + x27 + (-1 * x24)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[3])/sizeof(FLT), ((-1 * ((-4 * x25) + x35) * x18) + ((x34 + x31) * x12)) * x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[0])/sizeof(FLT), x36 * (x34 + x28 + (-1 * x30)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[1])/sizeof(FLT), x36 * x37);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[2])/sizeof(FLT), x36 * x38);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[3])/sizeof(FLT), (x4 + (-1 * x6) + (-1 * x8)) * x36);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[0])/sizeof(FLT), x44 * ((-1 * (x14 + (-4 * x20)) * x43) + (x40 * x37)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[1])/sizeof(FLT), x44 * ((-1 * x43 * (x22 + (4 * x8))) + (x40 * (x31 + (-1 * x32) + x33))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[2])/sizeof(FLT), x44 * ((-1 * (x29 + (-4 * x33)) * x43) + ((x9 + (-1 * x3) + (-1 * x1)) * x40)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[3])/sizeof(FLT), x44 * ((-1 * (x35 + (4 * x24)) * x43) + (x40 * x38)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccBias[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccBias[0])/sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccBias[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccBias[1])/sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccBias[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccBias[2])/sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, GyroBias[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, GyroBias[0])/sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, GyroBias[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, GyroBias[1])/sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, GyroBias[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, GyroBias[2])/sizeof(FLT), -1);
}

// Full version Jacobian of SurviveIMUBiasModelToErrorModel wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2], (*_x0).AccScale[0], (*_x0).AccScale[1], (*_x0).AccScale[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3]]
static inline void SurviveIMUBiasModelAddErrorModel(SurviveIMUBiasModel* out, const SurviveIMUBiasModel* _x0, const SurviveIMUBiasErrorModel* error_state) {
	const FLT x0 = 0.5 * (*error_state).IMUCorrection[0];
	const FLT x1 = 0.5 * (*error_state).IMUCorrection[2];
	const FLT x2 = 0.5 * (*error_state).IMUCorrection[1];
	const FLT x3 = (x2 * (*_x0).IMUCorrection[1]) + (x1 * (*_x0).IMUCorrection[0]) + (*_x0).IMUCorrection[3] + (-1 * x0 * (*_x0).IMUCorrection[2]);
	const FLT x4 = (x0 * (*_x0).IMUCorrection[3]) + (*_x0).IMUCorrection[2] + (-1 * x1 * (*_x0).IMUCorrection[1]) + (x2 * (*_x0).IMUCorrection[0]);
	const FLT x5 = (-1 * x0 * (*_x0).IMUCorrection[1]) + (*_x0).IMUCorrection[0] + (-1 * x1 * (*_x0).IMUCorrection[3]) + (-1 * x2 * (*_x0).IMUCorrection[2]);
	const FLT x6 = (*_x0).IMUCorrection[1] + (-1 * x2 * (*_x0).IMUCorrection[3]) + (x1 * (*_x0).IMUCorrection[2]) + (x0 * (*_x0).IMUCorrection[0]);
	const FLT x7 = 1. / sqrt((x6 * x6) + (x5 * x5) + (x3 * x3) + (x4 * x4));
	out->AccScale[0]=(*_x0).AccScale[0] + (*error_state).AccScale[0];
	out->AccScale[1]=(*_x0).AccScale[1] + (*error_state).AccScale[1];
	out->AccScale[2]=(*_x0).AccScale[2] + (*error_state).AccScale[2];
	out->IMUCorrection[0]=x5 * x7;
	out->IMUCorrection[1]=x6 * x7;
	out->IMUCorrection[2]=x4 * x7;
	out->IMUCorrection[3]=x3 * x7;
	out->AccBias[0]=(*_x0).AccBias[0] + (*error_state).AccBias[0];
	out->AccBias[1]=(*_x0).AccBias[1] + (*error_state).AccBias[1];
	out->AccBias[2]=(*_x0).AccBias[2] + (*error_state).AccBias[2];
	out->GyroBias[0]=(*_x0).GyroBias[0] + (*error_state).GyroBias[0];
	out->GyroBias[1]=(*_x0).GyroBias[1] + (*error_state).GyroBias[1];
	out->GyroBias[2]=(*_x0).GyroBias[2] + (*error_state).GyroBias[2];
}

// Jacobian of SurviveIMUBiasModelAddErrorModel wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2], (*_x0).AccScale[0], (*_x0).AccScale[1], (*_x0).AccScale[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3]]
static inline void SurviveIMUBiasModelAddErrorModel_jac_x0(CnMat* Hx, const SurviveIMUBiasModel* _x0, const SurviveIMUBiasErrorModel* error_state) {
	const FLT x0 = 0.5 * (*error_state).IMUCorrection[0];
	const FLT x1 = 0.5 * (*error_state).IMUCorrection[2];
	const FLT x2 = 0.5 * (*_x0).IMUCorrection[1];
	const FLT x3 = (x2 * (*error_state).IMUCorrection[1]) + (x1 * (*_x0).IMUCorrection[0]) + (*_x0).IMUCorrection[3] + (-1 * x0 * (*_x0).IMUCorrection[2]);
	const FLT x4 = 0.5 * (*error_state).IMUCorrection[1];
	const FLT x5 = (-1 * x2 * (*error_state).IMUCorrection[2]) + (x0 * (*_x0).IMUCorrection[3]) + (*_x0).IMUCorrection[2] + (x4 * (*_x0).IMUCorrection[0]);
	const FLT x6 = (*_x0).IMUCorrection[0] + (-1 * x1 * (*_x0).IMUCorrection[3]) + (-1 * x2 * (*error_state).IMUCorrection[0]) + (-1 * x4 * (*_x0).IMUCorrection[2]);
	const FLT x7 = (*_x0).IMUCorrection[1] + (-1 * x4 * (*_x0).IMUCorrection[3]) + (x1 * (*_x0).IMUCorrection[2]) + (x0 * (*_x0).IMUCorrection[0]);
	const FLT x8 = (x7 * x7) + (x6 * x6) + (x3 * x3) + (x5 * x5);
	const FLT x9 = 1. / sqrt(x8);
	const FLT x10 = 1.0 * x7;
	const FLT x11 = 1.0 * x3;
	const FLT x12 = 1.0 * (*error_state).IMUCorrection[1];
	const FLT x13 = 1.0/2.0 * (1. / (x8 * sqrt(x8)));
	const FLT x14 = x13 * ((x5 * x12) + (x11 * (*error_state).IMUCorrection[2]) + (x10 * (*error_state).IMUCorrection[0]) + (2 * x6));
	const FLT x15 = 0.5 * x9;
	const FLT x16 = x15 * (*error_state).IMUCorrection[0];
	const FLT x17 = -1 * x16;
	const FLT x18 = 1.0 * x6;
	const FLT x19 = 1.0 * x5;
	const FLT x20 = x13 * ((-1 * x19 * (*error_state).IMUCorrection[2]) + (2 * x7) + (x3 * x12) + (-1 * x18 * (*error_state).IMUCorrection[0]));
	const FLT x21 = x4 * x9;
	const FLT x22 = -1 * x21;
	const FLT x23 = x13 * ((2 * x5) + (-1 * x18 * (*error_state).IMUCorrection[1]) + (x10 * (*error_state).IMUCorrection[2]) + (-1 * x11 * (*error_state).IMUCorrection[0]));
	const FLT x24 = x15 * (*error_state).IMUCorrection[2];
	const FLT x25 = -1 * x24;
	const FLT x26 = x13 * ((x19 * (*error_state).IMUCorrection[0]) + (2 * x3) + (-1 * x7 * x12) + (-1 * x18 * (*error_state).IMUCorrection[2]));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccScale[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccScale[0])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccScale[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccScale[1])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccScale[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccScale[2])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[0])/sizeof(FLT), (-1 * x6 * x14) + x9);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[1])/sizeof(FLT), (-1 * x6 * x20) + x17);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[2])/sizeof(FLT), (-1 * x6 * x23) + x22);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[3])/sizeof(FLT), (-1 * x6 * x26) + x25);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[0])/sizeof(FLT), (-1 * x7 * x14) + x16);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[1])/sizeof(FLT), (-1 * x7 * x20) + x9);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[2])/sizeof(FLT), (-1 * x7 * x23) + x24);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[3])/sizeof(FLT), (-1 * x7 * x26) + x22);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[0])/sizeof(FLT), (-1 * x5 * x14) + x21);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[1])/sizeof(FLT), (-1 * x5 * x20) + x25);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[2])/sizeof(FLT), (-1 * x5 * x23) + x9);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[3])/sizeof(FLT), (-1 * x5 * x26) + x16);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[3])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[0])/sizeof(FLT), (-1 * x3 * x14) + x24);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[3])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[1])/sizeof(FLT), (-1 * x3 * x20) + x21);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[3])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[2])/sizeof(FLT), (-1 * x3 * x23) + x17);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[3])/sizeof(FLT), offsetof(SurviveIMUBiasModel, IMUCorrection[3])/sizeof(FLT), (-1 * x3 * x26) + x9);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccBias[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccBias[0])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccBias[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccBias[1])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccBias[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, AccBias[2])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, GyroBias[0])/sizeof(FLT), offsetof(SurviveIMUBiasModel, GyroBias[0])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, GyroBias[1])/sizeof(FLT), offsetof(SurviveIMUBiasModel, GyroBias[1])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, GyroBias[2])/sizeof(FLT), offsetof(SurviveIMUBiasModel, GyroBias[2])/sizeof(FLT), 1);
}

// Full version Jacobian of SurviveIMUBiasModelAddErrorModel wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2], (*_x0).AccScale[0], (*_x0).AccScale[1], (*_x0).AccScale[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1], (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2], (*_x0).IMUCorrection[3]]
// Jacobian of SurviveIMUBiasModelAddErrorModel wrt [(*error_state).AccBias[0], (*error_state).AccBias[1], (*error_state).AccBias[2], (*error_state).AccScale[0], (*error_state).AccScale[1], (*error_state).AccScale[2], (*error_state).GyroBias[0], (*error_state).GyroBias[1], (*error_state).GyroBias[2], (*error_state).IMUCorrection[0], (*error_state).IMUCorrection[1], (*error_state).IMUCorrection[2]]
static inline void SurviveIMUBiasModelAddErrorModel_jac_error_state(CnMat* Hx, const SurviveIMUBiasModel* _x0, const SurviveIMUBiasErrorModel* error_state) {
	const FLT x0 = 0.5 * (*error_state).IMUCorrection[0];
	const FLT x1 = 0.5 * (*error_state).IMUCorrection[2];
	const FLT x2 = 0.5 * (*error_state).IMUCorrection[1];
	const FLT x3 = (x2 * (*_x0).IMUCorrection[1]) + (x1 * (*_x0).IMUCorrection[0]) + (*_x0).IMUCorrection[3] + (-1 * x0 * (*_x0).IMUCorrection[2]);
	const FLT x4 = (x0 * (*_x0).IMUCorrection[3]) + (*_x0).IMUCorrection[2] + (-1 * x1 * (*_x0).IMUCorrection[1]) + (x2 * (*_x0).IMUCorrection[0]);
	const FLT x5 = (-1 * x0 * (*_x0).IMUCorrection[1]) + (*_x0).IMUCorrection[0] + (-1 * x1 * (*_x0).IMUCorrection[3]) + (-1 * x2 * (*_x0).IMUCorrection[2]);
	const FLT x6 = (*_x0).IMUCorrection[1] + (-1 * x2 * (*_x0).IMUCorrection[3]) + (x1 * (*_x0).IMUCorrection[2]) + (x0 * (*_x0).IMUCorrection[0]);
	const FLT x7 = (x6 * x6) + (x5 * x5) + (x3 * x3) + (x4 * x4);
	const FLT x8 = 0.5 * (1. / sqrt(x7));
	const FLT x9 = x8 * (*_x0).IMUCorrection[1];
	const FLT x10 = -1 * x9;
	const FLT x11 = 1.0 * x5;
	const FLT x12 = 1.0 * x3;
	const FLT x13 = 1.0 * (*_x0).IMUCorrection[0];
	const FLT x14 = 1.0 * x4;
	const FLT x15 = 1.0/2.0 * (1. / (x7 * sqrt(x7)));
	const FLT x16 = x15 * ((x6 * x13) + (x14 * (*_x0).IMUCorrection[3]) + (-1 * x11 * (*_x0).IMUCorrection[1]) + (-1 * x12 * (*_x0).IMUCorrection[2]));
	const FLT x17 = x8 * (*_x0).IMUCorrection[2];
	const FLT x18 = -1 * x17;
	const FLT x19 = 1.0 * x6;
	const FLT x20 = x15 * ((x4 * x13) + (x12 * (*_x0).IMUCorrection[1]) + (-1 * x19 * (*_x0).IMUCorrection[3]) + (-1 * x11 * (*_x0).IMUCorrection[2]));
	const FLT x21 = x8 * (*_x0).IMUCorrection[3];
	const FLT x22 = -1 * x21;
	const FLT x23 = x15 * ((x3 * x13) + (x19 * (*_x0).IMUCorrection[2]) + (-1 * x14 * (*_x0).IMUCorrection[1]) + (-1 * x11 * (*_x0).IMUCorrection[3]));
	const FLT x24 = x8 * (*_x0).IMUCorrection[0];
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccScale[0])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, AccScale[0])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccScale[1])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, AccScale[1])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccScale[2])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, AccScale[2])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[0])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0])/sizeof(FLT), (-1 * x5 * x16) + x10);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[0])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1])/sizeof(FLT), (-1 * x5 * x20) + x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[0])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2])/sizeof(FLT), (-1 * x5 * x23) + x22);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[1])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0])/sizeof(FLT), (-1 * x6 * x16) + x24);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[1])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1])/sizeof(FLT), (-1 * x6 * x20) + x22);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[1])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2])/sizeof(FLT), (-1 * x6 * x23) + x17);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[2])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0])/sizeof(FLT), (-1 * x4 * x16) + x21);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[2])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1])/sizeof(FLT), (-1 * x4 * x20) + x24);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[2])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2])/sizeof(FLT), (-1 * x4 * x23) + x10);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[3])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0])/sizeof(FLT), (-1 * x3 * x16) + x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[3])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1])/sizeof(FLT), (-1 * x3 * x20) + x9);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[3])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2])/sizeof(FLT), (-1 * x3 * x23) + x24);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccBias[0])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, AccBias[0])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccBias[1])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, AccBias[1])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccBias[2])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, AccBias[2])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, GyroBias[0])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, GyroBias[0])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, GyroBias[1])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, GyroBias[1])/sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, GyroBias[2])/sizeof(FLT), offsetof(SurviveIMUBiasErrorModel, GyroBias[2])/sizeof(FLT), 1);
}

// Full version Jacobian of SurviveIMUBiasModelAddErrorModel wrt [(*error_state).AccBias[0], (*error_state).AccBias[1], (*error_state).AccBias[2], (*error_state).AccScale[0], (*error_state).AccScale[1], (*error_state).AccScale[2], (*error_state).GyroBias[0], (*error_state).GyroBias[1], (*error_state).GyroBias[2], (*error_state).IMUCorrection[0], (*error_state).IMUCorrection[1], (*error_state).IMUCorrection[2]]
