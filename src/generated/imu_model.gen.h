/// NOTE: This is a generated file; do not edit.
#pragma once
#include <cnkalman/generated_header.h>
// clang-format off
static inline void SurviveIMUBiasModelToErrorModel(SurviveIMUBiasErrorModel *out, const SurviveIMUBiasModel *_x1,
												   const SurviveIMUBiasModel *_x0) {
	const FLT x0 =
		((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[1]) + (-1 * (*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[3]) +
		(-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[2]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[0]);
	const FLT x1 =
		(-1 * (*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[0]) +
		(-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[3]) + ((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[2]);
	const FLT x2 =
		(-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[3]) +
		((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[0]) + (-1 * (*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[2]);
	const FLT x3 =
		((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[3]) +
		((*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[0]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[2]);
	const FLT x4 = x0 * x0;
	out->AccScale[0] = (*_x1).AccScale[0] + (-1 * (*_x0).AccScale[0]);
	out->AccScale[1] = (*_x1).AccScale[1] + (-1 * (*_x0).AccScale[1]);
	out->AccScale[2] = (*_x1).AccScale[2] + (-1 * (*_x0).AccScale[2]);
	out->IMUCorrection[0] = atan2(2 * ((x2 * x3) + (x0 * x1)), 1 + (-2 * ((x2 * x2) + x4)));
	out->IMUCorrection[1] = asin(2 * ((x0 * x3) + (-1 * x2 * x1)));
	out->IMUCorrection[2] = atan2(2 * ((x1 * x3) + (x0 * x2)), 1 + (-2 * (x4 + (x1 * x1))));
	out->AccBias[0] = (*_x1).AccBias[0] + (-1 * (*_x0).AccBias[0]);
	out->AccBias[1] = (*_x1).AccBias[1] + (-1 * (*_x0).AccBias[1]);
	out->AccBias[2] = (*_x1).AccBias[2] + (-1 * (*_x0).AccBias[2]);
	out->GyroBias[0] = (*_x1).GyroBias[0] + (-1 * (*_x0).GyroBias[0]);
	out->GyroBias[1] = (*_x1).GyroBias[1] + (-1 * (*_x0).GyroBias[1]);
	out->GyroBias[2] = (*_x1).GyroBias[2] + (-1 * (*_x0).GyroBias[2]);
}

// Jacobian of SurviveIMUBiasModelToErrorModel wrt [(*_x1).AccBias[0], (*_x1).AccBias[1], (*_x1).AccBias[2],
// (*_x1).AccScale[0], (*_x1).AccScale[1], (*_x1).AccScale[2], (*_x1).GyroBias[0], (*_x1).GyroBias[1],
// (*_x1).GyroBias[2], (*_x1).IMUCorrection[0], (*_x1).IMUCorrection[1], (*_x1).IMUCorrection[2],
// (*_x1).IMUCorrection[3]]
static inline void SurviveIMUBiasModelToErrorModel_jac_x1(CnMat *Hx, const SurviveIMUBiasModel *_x1,
														  const SurviveIMUBiasModel *_x0) {
	const FLT x0 =
		((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[1]) + (-1 * (*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[3]) +
		(-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[2]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[0]);
	const FLT x1 = x0 * (*_x0).IMUCorrection[3];
	const FLT x2 =
		(-1 * (*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[0]) +
		(-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[3]) + ((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[2]);
	const FLT x3 = x2 * (*_x0).IMUCorrection[2];
	const FLT x4 =
		((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[3]) +
		((*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[0]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[2]);
	const FLT x5 = x4 * (*_x0).IMUCorrection[1];
	const FLT x6 =
		(-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[3]) +
		((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[0]) + (-1 * (*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[2]);
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
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccScale[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccScale[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccScale[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccScale[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccScale[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccScale[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[0]) / sizeof(FLT),
						x19 * ((-1 * ((4 * x15) + x14) * x18) + ((x8 + (-1 * x1) + (-1 * x3)) * x12)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[1]) / sizeof(FLT),
						x19 * ((-1 * x18 * ((-4 * x7) + x23)) + (x22 * x11)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[2]) / sizeof(FLT),
						x19 * ((-1 * ((-4 * x31) + x30) * x18) + (x12 * (x28 + x24 + x25))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[3]) / sizeof(FLT),
						x19 * ((-1 * ((4 * x25) + x36) * x18) + (x35 * x12)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[0]) / sizeof(FLT), x35 * x38);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[1]) / sizeof(FLT), x38 * (x39 + x26 + (-1 * x27)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[2]) / sizeof(FLT), x37 * x22);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[3]) / sizeof(FLT), x38 * (x5 + x40 + (-1 * x7)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[0]) / sizeof(FLT),
						((-1 * (x14 + (4 * x20)) * x45) + ((x39 + x28) * x42)) * x46);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[1]) / sizeof(FLT),
						x46 * ((-1 * x45 * (x23 + (-4 * x3))) + (x42 * (x34 + x32 + (-1 * x31)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[2]) / sizeof(FLT),
						x46 * ((-1 * (x30 + (4 * x33)) * x45) + (x42 * (x8 + x40))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[3]) / sizeof(FLT),
						x46 * ((-1 * (x36 + (-4 * x27)) * x45) + (x42 * (x21 + x20 + (-1 * x13)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccBias[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccBias[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccBias[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccBias[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, GyroBias[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, GyroBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, GyroBias[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, GyroBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, GyroBias[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, GyroBias[2]) / sizeof(FLT), 1);
}

// Full version Jacobian of SurviveIMUBiasModelToErrorModel wrt [(*_x1).AccBias[0], (*_x1).AccBias[1],
// (*_x1).AccBias[2], (*_x1).AccScale[0], (*_x1).AccScale[1], (*_x1).AccScale[2], (*_x1).GyroBias[0],
// (*_x1).GyroBias[1], (*_x1).GyroBias[2], (*_x1).IMUCorrection[0], (*_x1).IMUCorrection[1], (*_x1).IMUCorrection[2],
// (*_x1).IMUCorrection[3]] Jacobian of SurviveIMUBiasModelToErrorModel wrt [(*_x0).AccBias[0], (*_x0).AccBias[1],
// (*_x0).AccBias[2], (*_x0).AccScale[0], (*_x0).AccScale[1], (*_x0).AccScale[2], (*_x0).GyroBias[0],
// (*_x0).GyroBias[1], (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2],
// (*_x0).IMUCorrection[3]]
static inline void SurviveIMUBiasModelToErrorModel_jac_x0(CnMat *Hx, const SurviveIMUBiasModel *_x1,
														  const SurviveIMUBiasModel *_x0) {
	const FLT x0 =
		((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[1]) + (-1 * (*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[3]) +
		(-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[2]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[0]);
	const FLT x1 = x0 * (*_x1).IMUCorrection[3];
	const FLT x2 =
		(-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[3]) +
		((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[0]) + (-1 * (*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[2]);
	const FLT x3 = x2 * (*_x1).IMUCorrection[0];
	const FLT x4 = x3 + x1;
	const FLT x5 =
		((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[3]) +
		((*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[0]) + ((*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[2]);
	const FLT x6 = x5 * (*_x1).IMUCorrection[1];
	const FLT x7 =
		(-1 * (*_x1).IMUCorrection[2] * (*_x0).IMUCorrection[1]) + ((*_x1).IMUCorrection[3] * (*_x0).IMUCorrection[0]) +
		(-1 * (*_x1).IMUCorrection[0] * (*_x0).IMUCorrection[3]) + ((*_x1).IMUCorrection[1] * (*_x0).IMUCorrection[2]);
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
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccScale[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccScale[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccScale[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccScale[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccScale[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccScale[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[0]) / sizeof(FLT),
						((-1 * ((-4 * x15) + x14) * x18) + ((x9 + x4) * x12)) * x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[1]) / sizeof(FLT),
						x19 * ((-1 * x18 * ((4 * x3) + x22)) + (x12 * (x21 + (-1 * x13) + x15))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[2]) / sizeof(FLT),
						x19 * ((-1 * ((4 * x30) + x29) * x18) + (x12 * ((-1 * x23) + x27 + (-1 * x24)))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[3]) / sizeof(FLT),
						((-1 * ((-4 * x25) + x35) * x18) + ((x34 + x31) * x12)) * x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[0]) / sizeof(FLT), x36 * (x34 + x28 + (-1 * x30)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[1]) / sizeof(FLT), x36 * x37);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[2]) / sizeof(FLT), x36 * x38);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[3]) / sizeof(FLT),
						(x4 + (-1 * x6) + (-1 * x8)) * x36);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[0]) / sizeof(FLT),
						x44 * ((-1 * (x14 + (-4 * x20)) * x43) + (x40 * x37)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[1]) / sizeof(FLT),
						x44 * ((-1 * x43 * (x22 + (4 * x8))) + (x40 * (x31 + (-1 * x32) + x33))));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[2]) / sizeof(FLT),
						x44 * ((-1 * (x29 + (-4 * x33)) * x43) + ((x9 + (-1 * x3) + (-1 * x1)) * x40)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[3]) / sizeof(FLT),
						x44 * ((-1 * (x35 + (4 * x24)) * x43) + (x40 * x38)));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccBias[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccBias[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccBias[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccBias[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, AccBias[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccBias[2]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, GyroBias[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, GyroBias[0]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, GyroBias[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, GyroBias[1]) / sizeof(FLT), -1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasErrorModel, GyroBias[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, GyroBias[2]) / sizeof(FLT), -1);
}

// Full version Jacobian of SurviveIMUBiasModelToErrorModel wrt [(*_x0).AccBias[0], (*_x0).AccBias[1],
// (*_x0).AccBias[2], (*_x0).AccScale[0], (*_x0).AccScale[1], (*_x0).AccScale[2], (*_x0).GyroBias[0],
// (*_x0).GyroBias[1], (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2],
// (*_x0).IMUCorrection[3]]
static inline void SurviveIMUBiasModelAddErrorModel(SurviveIMUBiasModel *out, const SurviveIMUBiasModel *_x0,
													const SurviveIMUBiasErrorModel *error_state) {
	const FLT x0 = 0.5 * (*error_state).IMUCorrection[1];
	const FLT x1 = sin(x0);
	const FLT x2 = 0.5 * (*error_state).IMUCorrection[2];
	const FLT x3 = cos(x2);
	const FLT x4 = 0.5 * (*error_state).IMUCorrection[0];
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = cos(x0);
	const FLT x8 = cos(x4);
	const FLT x9 = sin(x2);
	const FLT x10 = x8 * x9;
	const FLT x11 = (x7 * x10) + (-1 * x1 * x6);
	const FLT x12 = x5 * x9;
	const FLT x13 = x3 * x8;
	const FLT x14 = (x7 * x13) + (x1 * x12);
	const FLT x15 = (x1 * x13) + (x7 * x12);
	const FLT x16 = (x6 * x7) + (-1 * x1 * x10);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x15 * x15) + (x11 * x11) + (x14 * x14));
	const FLT x18 = x11 * x17;
	const FLT x19 = x14 * x17;
	const FLT x20 = x15 * x17;
	const FLT x21 = x17 * x16;
	const FLT x22 = x17 * (*_x0).IMUCorrection[0];
	out->AccScale[0] = (*_x0).AccScale[0] + (*error_state).AccScale[0];
	out->AccScale[1] = (*_x0).AccScale[1] + (*error_state).AccScale[1];
	out->AccScale[2] = (*_x0).AccScale[2] + (*error_state).AccScale[2];
	out->IMUCorrection[0] = (-1 * x21 * (*_x0).IMUCorrection[1]) + (-1 * x20 * (*_x0).IMUCorrection[2]) +
							(-1 * x18 * (*_x0).IMUCorrection[3]) + (x19 * (*_x0).IMUCorrection[0]);
	out->IMUCorrection[1] = (x19 * (*_x0).IMUCorrection[1]) + (x22 * x16) + (-1 * x20 * (*_x0).IMUCorrection[3]) +
							(x18 * (*_x0).IMUCorrection[2]);
	out->IMUCorrection[2] = (-1 * x18 * (*_x0).IMUCorrection[1]) + (x22 * x15) + (x21 * (*_x0).IMUCorrection[3]) +
							(x19 * (*_x0).IMUCorrection[2]);
	out->IMUCorrection[3] = (x20 * (*_x0).IMUCorrection[1]) + (x22 * x11) + (x19 * (*_x0).IMUCorrection[3]) +
							(-1 * x21 * (*_x0).IMUCorrection[2]);
	out->AccBias[0] = (*_x0).AccBias[0] + (*error_state).AccBias[0];
	out->AccBias[1] = (*_x0).AccBias[1] + (*error_state).AccBias[1];
	out->AccBias[2] = (*_x0).AccBias[2] + (*error_state).AccBias[2];
	out->GyroBias[0] = (*_x0).GyroBias[0] + (*error_state).GyroBias[0];
	out->GyroBias[1] = (*_x0).GyroBias[1] + (*error_state).GyroBias[1];
	out->GyroBias[2] = (*_x0).GyroBias[2] + (*error_state).GyroBias[2];
}

// Jacobian of SurviveIMUBiasModelAddErrorModel wrt [(*_x0).AccBias[0], (*_x0).AccBias[1], (*_x0).AccBias[2],
// (*_x0).AccScale[0], (*_x0).AccScale[1], (*_x0).AccScale[2], (*_x0).GyroBias[0], (*_x0).GyroBias[1],
// (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2],
// (*_x0).IMUCorrection[3]]
static inline void SurviveIMUBiasModelAddErrorModel_jac_x0(CnMat *Hx, const SurviveIMUBiasModel *_x0,
														   const SurviveIMUBiasErrorModel *error_state) {
	const FLT x0 = 0.5 * (*error_state).IMUCorrection[1];
	const FLT x1 = sin(x0);
	const FLT x2 = 0.5 * (*error_state).IMUCorrection[0];
	const FLT x3 = sin(x2);
	const FLT x4 = 0.5 * (*error_state).IMUCorrection[2];
	const FLT x5 = cos(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = cos(x0);
	const FLT x8 = sin(x4);
	const FLT x9 = cos(x2);
	const FLT x10 = x8 * x9;
	const FLT x11 = (x7 * x10) + (-1 * x1 * x6);
	const FLT x12 = x3 * x8;
	const FLT x13 = x5 * x9;
	const FLT x14 = (x7 * x13) + (x1 * x12);
	const FLT x15 = (x1 * x13) + (x7 * x12);
	const FLT x16 = (x6 * x7) + (-1 * x1 * x10);
	const FLT x17 = 1. / sqrt((x16 * x16) + (x15 * x15) + (x11 * x11) + (x14 * x14));
	const FLT x18 = x14 * x17;
	const FLT x19 = x17 * x16;
	const FLT x20 = -1 * x19;
	const FLT x21 = x15 * x17;
	const FLT x22 = -1 * x21;
	const FLT x23 = x11 * x17;
	const FLT x24 = -1 * x23;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccScale[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccScale[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccScale[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccScale[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccScale[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccScale[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[0]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[1]) / sizeof(FLT), x20);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[2]) / sizeof(FLT), x22);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[3]) / sizeof(FLT), x24);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[0]) / sizeof(FLT), x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[1]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[2]) / sizeof(FLT), x23);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[3]) / sizeof(FLT), x22);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[0]) / sizeof(FLT), x21);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[1]) / sizeof(FLT), x24);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[2]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[3]) / sizeof(FLT), x19);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[0]) / sizeof(FLT), x23);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[1]) / sizeof(FLT), x21);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[2]) / sizeof(FLT), x20);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, IMUCorrection[3]) / sizeof(FLT), x18);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccBias[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccBias[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccBias[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, AccBias[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, GyroBias[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, GyroBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, GyroBias[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, GyroBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, GyroBias[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasModel, GyroBias[2]) / sizeof(FLT), 1);
}

// Full version Jacobian of SurviveIMUBiasModelAddErrorModel wrt [(*_x0).AccBias[0], (*_x0).AccBias[1],
// (*_x0).AccBias[2], (*_x0).AccScale[0], (*_x0).AccScale[1], (*_x0).AccScale[2], (*_x0).GyroBias[0],
// (*_x0).GyroBias[1], (*_x0).GyroBias[2], (*_x0).IMUCorrection[0], (*_x0).IMUCorrection[1], (*_x0).IMUCorrection[2],
// (*_x0).IMUCorrection[3]] Jacobian of SurviveIMUBiasModelAddErrorModel wrt [(*error_state).AccBias[0],
// (*error_state).AccBias[1], (*error_state).AccBias[2], (*error_state).AccScale[0], (*error_state).AccScale[1],
// (*error_state).AccScale[2], (*error_state).GyroBias[0], (*error_state).GyroBias[1], (*error_state).GyroBias[2],
// (*error_state).IMUCorrection[0], (*error_state).IMUCorrection[1], (*error_state).IMUCorrection[2]]
static inline void SurviveIMUBiasModelAddErrorModel_jac_error_state(CnMat *Hx, const SurviveIMUBiasModel *_x0,
																	const SurviveIMUBiasErrorModel *error_state) {
	const FLT x0 = 0.5 * (*error_state).IMUCorrection[1];
	const FLT x1 = sin(x0);
	const FLT x2 = 0.5 * (*error_state).IMUCorrection[0];
	const FLT x3 = cos(x2);
	const FLT x4 = 0.5 * (*error_state).IMUCorrection[2];
	const FLT x5 = sin(x4);
	const FLT x6 = x3 * x5;
	const FLT x7 = x1 * x6;
	const FLT x8 = cos(x0);
	const FLT x9 = cos(x4);
	const FLT x10 = sin(x2);
	const FLT x11 = x9 * x10;
	const FLT x12 = x8 * x11;
	const FLT x13 = x12 + (-1 * x7);
	const FLT x14 = x1 * x11;
	const FLT x15 = 0.5 * x14;
	const FLT x16 = -1 * x15;
	const FLT x17 = x6 * x8;
	const FLT x18 = 0.5 * x17;
	const FLT x19 = x18 + x16;
	const FLT x20 = x5 * x10;
	const FLT x21 = x8 * x20;
	const FLT x22 = x3 * x9;
	const FLT x23 = x1 * x22;
	const FLT x24 = x23 + x21;
	const FLT x25 = 2 * x24;
	const FLT x26 = x8 * x22;
	const FLT x27 = 0.5 * x26;
	const FLT x28 = x1 * x20;
	const FLT x29 = 0.5 * x28;
	const FLT x30 = x29 + x27;
	const FLT x31 = 2 * x13;
	const FLT x32 = 0.5 * x21;
	const FLT x33 = -0.5 * x23;
	const FLT x34 = x33 + (-1 * x32);
	const FLT x35 = x17 + (-1 * x14);
	const FLT x36 = 2 * x35;
	const FLT x37 = 0.5 * x12;
	const FLT x38 = -1 * x37;
	const FLT x39 = 0.5 * x7;
	const FLT x40 = x39 + x38;
	const FLT x41 = x26 + x28;
	const FLT x42 = 2 * x41;
	const FLT x43 = (x34 * x36) + (x25 * x19) + (x40 * x42) + (x30 * x31);
	const FLT x44 = (x24 * x24) + (x35 * x35) + (x13 * x13) + (x41 * x41);
	const FLT x45 = 1.0 / 2.0 * (1. / (x44 * sqrt(x44)));
	const FLT x46 = x45 * (*_x0).IMUCorrection[1];
	const FLT x47 = x43 * x46;
	const FLT x48 = 1. / sqrt(x44);
	const FLT x49 = x48 * (*_x0).IMUCorrection[1];
	const FLT x50 = -1 * x49 * x30;
	const FLT x51 = x43 * x45;
	const FLT x52 = x51 * x24;
	const FLT x53 = x51 * (*_x0).IMUCorrection[3];
	const FLT x54 = x40 * x48;
	const FLT x55 = x48 * x19;
	const FLT x56 = x48 * x34;
	const FLT x57 = x56 * (*_x0).IMUCorrection[3];
	const FLT x58 = x51 * x41;
	const FLT x59 = -1 * x18;
	const FLT x60 = x59 + x16;
	const FLT x61 = (-1 * x29) + x27;
	const FLT x62 = -1 * x39;
	const FLT x63 = x38 + x62;
	const FLT x64 = x32 + x33;
	const FLT x65 = ((x63 * x36) + (x60 * x31) + (x64 * x42) + (x61 * x25)) * x45;
	const FLT x66 = x65 * x13;
	const FLT x67 = x65 * x24;
	const FLT x68 = x65 * x35;
	const FLT x69 = x63 * x48;
	const FLT x70 = x65 * x41;
	const FLT x71 = x48 * (*_x0).IMUCorrection[0];
	const FLT x72 = x61 * x48;
	const FLT x73 = -1 * x49 * x34;
	const FLT x74 = x37 + x62;
	const FLT x75 = x15 + x59;
	const FLT x76 = (x75 * x42) + (x74 * x25) + (x31 * x34) + (x30 * x36);
	const FLT x77 = x76 * x45;
	const FLT x78 = x77 * x24;
	const FLT x79 = x76 * x13;
	const FLT x80 = x77 * x41;
	const FLT x81 = x76 * x35;
	const FLT x82 = x81 * x45;
	const FLT x83 = x48 * x30;
	const FLT x84 = x83 * (*_x0).IMUCorrection[3];
	const FLT x85 = x74 * x48;
	const FLT x86 = x51 * x13;
	const FLT x87 = x51 * x35;
	const FLT x88 = x56 * (*_x0).IMUCorrection[2];
	const FLT x89 = x41 * x46;
	const FLT x90 = x71 * x30;
	const FLT x91 = x79 * x45;
	const FLT x92 = x83 * (*_x0).IMUCorrection[2];
	const FLT x93 = x71 * x34;
	const FLT x94 = x60 * x48;
	const FLT x95 = x64 * x48;
	const FLT x96 = x75 * x48;
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccScale[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, AccScale[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccScale[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, AccScale[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccScale[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, AccScale[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0]) / sizeof(FLT),
						(-1 * x58 * (*_x0).IMUCorrection[0]) + (-1 * x57) + x50 + (-1 * x55 * (*_x0).IMUCorrection[2]) +
							(x53 * x35) + (x47 * x13) + (x52 * (*_x0).IMUCorrection[2]) +
							(x54 * (*_x0).IMUCorrection[0]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1]) / sizeof(FLT),
						(-1 * x70 * (*_x0).IMUCorrection[0]) + (x67 * (*_x0).IMUCorrection[2]) + (-1 * x60 * x49) +
							(-1 * x72 * (*_x0).IMUCorrection[2]) + (x66 * (*_x0).IMUCorrection[1]) +
							(-1 * x69 * (*_x0).IMUCorrection[3]) + (x71 * x64) + (x68 * (*_x0).IMUCorrection[3]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2]) / sizeof(FLT),
						(-1 * x85 * (*_x0).IMUCorrection[2]) + (-1 * x84) + (x82 * (*_x0).IMUCorrection[3]) +
							(x78 * (*_x0).IMUCorrection[2]) + x73 + (x71 * x75) + (-1 * x80 * (*_x0).IMUCorrection[0]) +
							(x79 * x46));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0]) / sizeof(FLT),
						x88 + x90 + (-1 * x87 * (*_x0).IMUCorrection[2]) + (-1 * x89 * x43) +
							(-1 * x86 * (*_x0).IMUCorrection[0]) + (x53 * x24) + (x54 * (*_x0).IMUCorrection[1]) +
							(-1 * x55 * (*_x0).IMUCorrection[3]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1]) / sizeof(FLT),
						(-1 * x68 * (*_x0).IMUCorrection[2]) + (x69 * (*_x0).IMUCorrection[2]) +
							(-1 * x72 * (*_x0).IMUCorrection[3]) + (x64 * x49) + (-1 * x70 * (*_x0).IMUCorrection[1]) +
							(-1 * x66 * (*_x0).IMUCorrection[0]) + (x71 * x60) + (x67 * (*_x0).IMUCorrection[3]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2]) / sizeof(FLT),
						(-1 * x82 * (*_x0).IMUCorrection[2]) + x93 + (-1 * x85 * (*_x0).IMUCorrection[3]) +
							(-1 * x91 * (*_x0).IMUCorrection[0]) + (x75 * x49) + (-1 * x89 * x76) +
							(x78 * (*_x0).IMUCorrection[3]) + x92);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0]) / sizeof(FLT),
						(x71 * x19) + (-1 * x58 * (*_x0).IMUCorrection[2]) + (x54 * (*_x0).IMUCorrection[2]) +
							(-1 * x53 * x13) + x73 + (x47 * x35) + x84 + (-1 * x52 * (*_x0).IMUCorrection[0]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1]) / sizeof(FLT),
						(x95 * (*_x0).IMUCorrection[2]) + (-1 * x63 * x49) + (x94 * (*_x0).IMUCorrection[3]) +
							(x68 * (*_x0).IMUCorrection[1]) + (-1 * x66 * (*_x0).IMUCorrection[3]) + (x71 * x61) +
							(-1 * x67 * (*_x0).IMUCorrection[0]) + (-1 * x70 * (*_x0).IMUCorrection[2]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2]) / sizeof(FLT),
						(-1 * x91 * (*_x0).IMUCorrection[3]) + (-1 * x80 * (*_x0).IMUCorrection[2]) + x50 +
							(-1 * x78 * (*_x0).IMUCorrection[0]) + x57 + (x81 * x46) + (x96 * (*_x0).IMUCorrection[2]) +
							(x85 * (*_x0).IMUCorrection[0]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, IMUCorrection[0]) / sizeof(FLT),
						(x86 * (*_x0).IMUCorrection[2]) + (-1 * x53 * x41) + (-1 * x92) + (-1 * x47 * x24) +
							(x49 * x19) + (-1 * x87 * (*_x0).IMUCorrection[0]) + x93 + (x54 * (*_x0).IMUCorrection[3]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, IMUCorrection[1]) / sizeof(FLT),
						(x66 * (*_x0).IMUCorrection[2]) + (x95 * (*_x0).IMUCorrection[3]) + (x71 * x63) +
							(-1 * x67 * (*_x0).IMUCorrection[1]) + (-1 * x70 * (*_x0).IMUCorrection[3]) + (x61 * x49) +
							(-1 * x68 * (*_x0).IMUCorrection[0]) + (-1 * x94 * (*_x0).IMUCorrection[2]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, IMUCorrection[3]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, IMUCorrection[2]) / sizeof(FLT),
						x90 + (-1 * x88) + (x96 * (*_x0).IMUCorrection[3]) + (x85 * (*_x0).IMUCorrection[1]) +
							(-1 * x76 * x46 * x24) + (x91 * (*_x0).IMUCorrection[2]) +
							(-1 * x80 * (*_x0).IMUCorrection[3]) + (-1 * x82 * (*_x0).IMUCorrection[0]));
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccBias[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, AccBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccBias[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, AccBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, AccBias[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, AccBias[2]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, GyroBias[0]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, GyroBias[0]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, GyroBias[1]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, GyroBias[1]) / sizeof(FLT), 1);
	cnMatrixOptionalSet(Hx, offsetof(SurviveIMUBiasModel, GyroBias[2]) / sizeof(FLT),
						offsetof(SurviveIMUBiasErrorModel, GyroBias[2]) / sizeof(FLT), 1);
}

// Full version Jacobian of SurviveIMUBiasModelAddErrorModel wrt [(*error_state).AccBias[0], (*error_state).AccBias[1],
// (*error_state).AccBias[2], (*error_state).AccScale[0], (*error_state).AccScale[1], (*error_state).AccScale[2],
// (*error_state).GyroBias[0], (*error_state).GyroBias[1], (*error_state).GyroBias[2], (*error_state).IMUCorrection[0],
// (*error_state).IMUCorrection[1], (*error_state).IMUCorrection[2]]
