/// NOTE: This is a generated file; do not edit.
#pragma once
#include <cnkalman/generated_header.h>
// clang-format off    
static inline void reproject_xy(CnMat* out, const BaseStationCal* cal0, const BaseStationCal* cal1, const FLT* sensor_pt) {
	const FLT x0 = sensor_pt[2] * sensor_pt[2];
	const FLT x1 = -1 * sensor_pt[2];
	const FLT x2 = atan2(sensor_pt[0], x1);
	const FLT x3 = (-1 * (*cal0).phase) + (-1 * asin((*cal0).tilt * sensor_pt[1] * (1. / sqrt((sensor_pt[0] * sensor_pt[0]) + x0)))) + (-1 * x2);
	const FLT x4 = (-1 * (*cal1).phase) + (-1 * asin((*cal1).tilt * sensor_pt[0] * (1. / sqrt((sensor_pt[1] * sensor_pt[1]) + x0)))) + (-1 * atan2(-1 * sensor_pt[1], x1));
	cnMatrixOptionalSet(out, 0, 0, x3 + (-1 * (*cal0).gibmag * cos(1.5707963267949 + x3 + (*cal0).gibpha)) + ((atan2(sensor_pt[1], x1) * atan2(sensor_pt[1], x1)) * (*cal0).curve));
	cnMatrixOptionalSet(out, 1, 0, x4 + ((x2 * x2) * (*cal1).curve) + (-1 * (*cal1).gibmag * cos(1.5707963267949 + x4 + (*cal1).gibpha)));
}

// Jacobian of reproject_xy wrt [<cnkalman.codegen.WrapMember object at 0x7fcf8b662640>, <cnkalman.codegen.WrapMember object at 0x7fcf8b662670>, <cnkalman.codegen.WrapMember object at 0x7fcf8b662ac0>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6d3340>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6d3850>, <cnkalman.codegen.WrapMember object at 0x7fd013068d30>, <cnkalman.codegen.WrapMember object at 0x7fcf8b662c40>]
static inline void reproject_xy_jac_cal0(CnMat* Hx, const BaseStationCal* cal0, const BaseStationCal* cal1, const FLT* sensor_pt) {
	const FLT x0 = -1 * sensor_pt[2];
	const FLT x1 = (sensor_pt[0] * sensor_pt[0]) + (sensor_pt[2] * sensor_pt[2]);
	const FLT x2 = (1. / sqrt(x1)) * sensor_pt[1];
	const FLT x3 = 1.5707963267949 + (-1 * (*cal0).phase) + (-1 * atan2(sensor_pt[0], x0)) + (*cal0).gibpha + (-1 * asin(x2 * (*cal0).tilt));
	const FLT x4 = sin(x3) * (*cal0).gibmag;
	const FLT x5 = x2 * (1. / sqrt(1 + (-1 * (1. / x1) * ((*cal0).tilt * (*cal0).tilt) * (sensor_pt[1] * sensor_pt[1]))));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, curve)/sizeof(FLT), atan2(sensor_pt[1], x0) * atan2(sensor_pt[1], x0));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibmag)/sizeof(FLT), -1 * cos(x3));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibpha)/sizeof(FLT), x4);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, phase)/sizeof(FLT), -1 + (-1 * x4));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, tilt)/sizeof(FLT), (-1 * x4 * x5) + (-1 * x5));
}

// Full version Jacobian of reproject_xy wrt [<cnkalman.codegen.WrapMember object at 0x7fcf8b662640>, <cnkalman.codegen.WrapMember object at 0x7fcf8b662670>, <cnkalman.codegen.WrapMember object at 0x7fcf8b662ac0>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6d3340>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6d3850>, <cnkalman.codegen.WrapMember object at 0x7fd013068d30>, <cnkalman.codegen.WrapMember object at 0x7fcf8b662c40>]

static inline void reproject_xy_jac_cal0_with_hx(CnMat* Hx, CnMat* hx, const BaseStationCal* cal0, const BaseStationCal* cal1, const FLT* sensor_pt) {
    if(hx != 0) { 
        reproject_xy(hx, cal0, cal1, sensor_pt);
    }
    if(Hx != 0) { 
        reproject_xy_jac_cal0(Hx, cal0, cal1, sensor_pt);
    }
}
// Jacobian of reproject_xy wrt [<cnkalman.codegen.WrapMember object at 0x7fcf8b6d3a00>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6d3d30>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6d36d0>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6d3730>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6d3d60>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6d3520>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6d3820>]
static inline void reproject_xy_jac_cal1(CnMat* Hx, const BaseStationCal* cal0, const BaseStationCal* cal1, const FLT* sensor_pt) {
	const FLT x0 = -1 * sensor_pt[2];
	const FLT x1 = (sensor_pt[1] * sensor_pt[1]) + (sensor_pt[2] * sensor_pt[2]);
	const FLT x2 = (1. / sqrt(x1)) * sensor_pt[0];
	const FLT x3 = 1.5707963267949 + (-1 * (*cal1).phase) + (*cal1).gibpha + (-1 * asin(x2 * (*cal1).tilt)) + (-1 * atan2(-1 * sensor_pt[1], x0));
	const FLT x4 = sin(x3) * (*cal1).gibmag;
	const FLT x5 = x2 * (1. / sqrt(1 + (-1 * (1. / x1) * ((*cal1).tilt * (*cal1).tilt) * (sensor_pt[0] * sensor_pt[0]))));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 1, offsetof(BaseStationCal, curve)/sizeof(FLT), atan2(sensor_pt[0], x0) * atan2(sensor_pt[0], x0));
	cnMatrixOptionalSet(Hx, 1, offsetof(BaseStationCal, gibmag)/sizeof(FLT), -1 * cos(x3));
	cnMatrixOptionalSet(Hx, 1, offsetof(BaseStationCal, gibpha)/sizeof(FLT), x4);
	cnMatrixOptionalSet(Hx, 1, offsetof(BaseStationCal, phase)/sizeof(FLT), -1 + (-1 * x4));
	cnMatrixOptionalSet(Hx, 1, offsetof(BaseStationCal, tilt)/sizeof(FLT), (-1 * x5) + (-1 * x4 * x5));
}

// Full version Jacobian of reproject_xy wrt [<cnkalman.codegen.WrapMember object at 0x7fcf8b6d3a00>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6d3d30>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6d36d0>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6d3730>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6d3d60>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6d3520>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6d3820>]

static inline void reproject_xy_jac_cal1_with_hx(CnMat* Hx, CnMat* hx, const BaseStationCal* cal0, const BaseStationCal* cal1, const FLT* sensor_pt) {
    if(hx != 0) { 
        reproject_xy(hx, cal0, cal1, sensor_pt);
    }
    if(Hx != 0) { 
        reproject_xy_jac_cal1(Hx, cal0, cal1, sensor_pt);
    }
}
// Jacobian of reproject_xy wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]
static inline void reproject_xy_jac_sensor_pt(CnMat* Hx, const BaseStationCal* cal0, const BaseStationCal* cal1, const FLT* sensor_pt) {
	const FLT x0 = sensor_pt[0] * sensor_pt[0];
	const FLT x1 = sensor_pt[2] * sensor_pt[2];
	const FLT x2 = x1 + x0;
	const FLT x3 = 1. / x2;
	const FLT x4 = x3 * sensor_pt[2];
	const FLT x5 = sensor_pt[1] * sensor_pt[0];
	const FLT x6 = sensor_pt[1] * sensor_pt[1];
	const FLT x7 = 1. / sqrt(1 + (-1 * x3 * x6 * ((*cal0).tilt * (*cal0).tilt)));
	const FLT x8 = (1. / (x2 * sqrt(x2))) * x7 * (*cal0).tilt;
	const FLT x9 = (x5 * x8) + x4;
	const FLT x10 = (1. / sqrt(x2)) * (*cal0).tilt;
	const FLT x11 = -1 * sensor_pt[2];
	const FLT x12 = atan2(sensor_pt[0], x11);
	const FLT x13 = (*cal0).gibmag * sin(1.5707963267949 + (-1 * (*cal0).phase) + (-1 * x12) + (*cal0).gibpha + (-1 * asin(x10 * sensor_pt[1])));
	const FLT x14 = x7 * x10;
	const FLT x15 = x1 + x6;
	const FLT x16 = 1. / x15;
	const FLT x17 = x16 * sensor_pt[2];
	const FLT x18 = 2 * atan2(sensor_pt[1], x11) * (*cal0).curve;
	const FLT x19 = x3 * sensor_pt[0];
	const FLT x20 = (x8 * sensor_pt[2] * sensor_pt[1]) + (-1 * x19);
	const FLT x21 = x16 * sensor_pt[1];
	const FLT x22 = (1. / sqrt(x15)) * (*cal1).tilt;
	const FLT x23 = sin(1.5707963267949 + (-1 * (*cal1).phase) + (*cal1).gibpha + (-1 * asin(x22 * sensor_pt[0])) + (-1 * atan2(-1 * sensor_pt[1], x11))) * (*cal1).gibmag;
	const FLT x24 = 1. / sqrt(1 + (-1 * x0 * x16 * ((*cal1).tilt * (*cal1).tilt)));
	const FLT x25 = x24 * x22;
	const FLT x26 = 2 * x12 * (*cal1).curve;
	const FLT x27 = x24 * (1. / (x15 * sqrt(x15))) * (*cal1).tilt;
	const FLT x28 = (x5 * x27) + (-1 * x17);
	const FLT x29 = (x27 * sensor_pt[2] * sensor_pt[0]) + x21;
	cnMatrixOptionalSet(Hx, 0, 0, x9 + (x9 * x13));
	cnMatrixOptionalSet(Hx, 0, 1, (-1 * x18 * x17) + (-1 * x14 * x13) + (-1 * x14));
	cnMatrixOptionalSet(Hx, 0, 2, x20 + (x20 * x13) + (x21 * x18));
	cnMatrixOptionalSet(Hx, 1, 0, (-1 * x25) + (-1 * x25 * x23) + (-1 * x4 * x26));
	cnMatrixOptionalSet(Hx, 1, 1, x28 + (x23 * x28));
	cnMatrixOptionalSet(Hx, 1, 2, x29 + (x23 * x29) + (x26 * x19));
}

// Full version Jacobian of reproject_xy wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]

static inline void reproject_xy_jac_sensor_pt_with_hx(CnMat* Hx, CnMat* hx, const BaseStationCal* cal0, const BaseStationCal* cal1, const FLT* sensor_pt) {
    if(hx != 0) { 
        reproject_xy(hx, cal0, cal1, sensor_pt);
    }
    if(Hx != 0) { 
        reproject_xy_jac_sensor_pt(Hx, cal0, cal1, sensor_pt);
    }
}
static inline FLT reproject_axis_x(const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc0) {
	const FLT x0 = (-1 * sensor_pt[1] * (*obj_p).Rot[3]) + (sensor_pt[0] * (*obj_p).Rot[0]) + (sensor_pt[2] * (*obj_p).Rot[2]);
	const FLT x1 = (sensor_pt[0] * (*obj_p).Rot[3]) + (-1 * sensor_pt[2] * (*obj_p).Rot[1]) + (sensor_pt[1] * (*obj_p).Rot[0]);
	const FLT x2 = sensor_pt[2] + (2 * ((x1 * (*obj_p).Rot[1]) + (-1 * x0 * (*obj_p).Rot[2]))) + (*obj_p).Pos[2];
	const FLT x3 = (sensor_pt[1] * (*obj_p).Rot[1]) + (-1 * sensor_pt[0] * (*obj_p).Rot[2]) + (sensor_pt[2] * (*obj_p).Rot[0]);
	const FLT x4 = (2 * ((x0 * (*obj_p).Rot[3]) + (-1 * x3 * (*obj_p).Rot[1]))) + (*obj_p).Pos[1] + sensor_pt[1];
	const FLT x5 = (2 * ((x3 * (*obj_p).Rot[2]) + (-1 * x1 * (*obj_p).Rot[3]))) + (*obj_p).Pos[0] + sensor_pt[0];
	const FLT x6 = (-1 * x5 * (*lh_p).Rot[2]) + (x2 * (*lh_p).Rot[0]) + (x4 * (*lh_p).Rot[1]);
	const FLT x7 = (-1 * x4 * (*lh_p).Rot[3]) + (x5 * (*lh_p).Rot[0]) + (x2 * (*lh_p).Rot[2]);
	const FLT x8 = (*lh_p).Pos[1] + x4 + (2 * ((x7 * (*lh_p).Rot[3]) + (-1 * x6 * (*lh_p).Rot[1])));
	const FLT x9 = (-1 * x2 * (*lh_p).Rot[1]) + (x5 * (*lh_p).Rot[3]) + (x4 * (*lh_p).Rot[0]);
	const FLT x10 = (2 * ((x9 * (*lh_p).Rot[1]) + (-1 * x7 * (*lh_p).Rot[2]))) + x2 + (*lh_p).Pos[2];
	const FLT x11 = -1 * x10;
	const FLT x12 = x5 + (*lh_p).Pos[0] + (2 * ((x6 * (*lh_p).Rot[2]) + (-1 * x9 * (*lh_p).Rot[3])));
	const FLT x13 = (-1 * (*bsc0).phase) + (-1 * asin((1. / sqrt((x12 * x12) + (x10 * x10))) * x8 * (*bsc0).tilt)) + (-1 * atan2(x12, x11));
	return x13 + ((atan2(x8, x11) * atan2(x8, x11)) * (*bsc0).curve) + (-1 * cos(1.5707963267949 + x13 + (*bsc0).gibpha) * (*bsc0).gibmag);
}

// Jacobian of reproject_axis_x wrt [(*obj_p).Pos[0], (*obj_p).Pos[1], (*obj_p).Pos[2], (*obj_p).Rot[0], (*obj_p).Rot[1], (*obj_p).Rot[2], (*obj_p).Rot[3]]
static inline void reproject_axis_x_jac_obj_p(CnMat* Hx, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc0) {
	const FLT x0 = 2 * (*lh_p).Rot[2];
	const FLT x1 = x0 * (*lh_p).Rot[0];
	const FLT x2 = 2 * (*lh_p).Rot[3];
	const FLT x3 = x2 * (*lh_p).Rot[1];
	const FLT x4 = x3 + (-1 * x1);
	const FLT x5 = sensor_pt[0] * (*obj_p).Rot[3];
	const FLT x6 = sensor_pt[1] * (*obj_p).Rot[0];
	const FLT x7 = sensor_pt[2] * (*obj_p).Rot[1];
	const FLT x8 = (-1 * x7) + x5 + x6;
	const FLT x9 = sensor_pt[1] * (*obj_p).Rot[1];
	const FLT x10 = sensor_pt[2] * (*obj_p).Rot[0];
	const FLT x11 = sensor_pt[0] * (*obj_p).Rot[2];
	const FLT x12 = (-1 * x11) + x9 + x10;
	const FLT x13 = (2 * ((x12 * (*obj_p).Rot[2]) + (-1 * x8 * (*obj_p).Rot[3]))) + (*obj_p).Pos[0] + sensor_pt[0];
	const FLT x14 = sensor_pt[0] * (*obj_p).Rot[0];
	const FLT x15 = sensor_pt[2] * (*obj_p).Rot[2];
	const FLT x16 = sensor_pt[1] * (*obj_p).Rot[3];
	const FLT x17 = x14 + (-1 * x16) + x15;
	const FLT x18 = (2 * ((x17 * (*obj_p).Rot[3]) + (-1 * x12 * (*obj_p).Rot[1]))) + (*obj_p).Pos[1] + sensor_pt[1];
	const FLT x19 = sensor_pt[2] + (2 * ((x8 * (*obj_p).Rot[1]) + (-1 * x17 * (*obj_p).Rot[2]))) + (*obj_p).Pos[2];
	const FLT x20 = (-1 * x19 * (*lh_p).Rot[1]) + (x13 * (*lh_p).Rot[3]) + (x18 * (*lh_p).Rot[0]);
	const FLT x21 = (-1 * x13 * (*lh_p).Rot[2]) + (x19 * (*lh_p).Rot[0]) + (x18 * (*lh_p).Rot[1]);
	const FLT x22 = x13 + (*lh_p).Pos[0] + (2 * ((x21 * (*lh_p).Rot[2]) + (-1 * x20 * (*lh_p).Rot[3])));
	const FLT x23 = (-1 * x18 * (*lh_p).Rot[3]) + (x13 * (*lh_p).Rot[0]) + (x19 * (*lh_p).Rot[2]);
	const FLT x24 = x19 + (2 * ((x20 * (*lh_p).Rot[1]) + (-1 * x23 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x25 = x24 * x24;
	const FLT x26 = 1. / x25;
	const FLT x27 = x22 * x26;
	const FLT x28 = -2 * ((*lh_p).Rot[2] * (*lh_p).Rot[2]);
	const FLT x29 = 1 + (-2 * ((*lh_p).Rot[3] * (*lh_p).Rot[3]));
	const FLT x30 = x29 + x28;
	const FLT x31 = 1. / x24;
	const FLT x32 = x25 + (x22 * x22);
	const FLT x33 = 1. / x32;
	const FLT x34 = x33 * x25;
	const FLT x35 = x18 + (*lh_p).Pos[1] + (2 * ((x23 * (*lh_p).Rot[3]) + (-1 * x21 * (*lh_p).Rot[1])));
	const FLT x36 = x35 * x35;
	const FLT x37 = 1. / sqrt(1 + (-1 * x33 * x36 * ((*bsc0).tilt * (*bsc0).tilt)));
	const FLT x38 = 2 * (*lh_p).Rot[1];
	const FLT x39 = x38 * (*lh_p).Rot[2];
	const FLT x40 = x2 * (*lh_p).Rot[0];
	const FLT x41 = x40 + x39;
	const FLT x42 = (1. / sqrt(x32)) * (*bsc0).tilt;
	const FLT x43 = 2 * x22;
	const FLT x44 = 2 * x24;
	const FLT x45 = 1.0/2.0 * (1. / (x32 * sqrt(x32))) * x35 * (*bsc0).tilt;
	const FLT x46 = (-1 * x37 * ((-1 * x45 * ((x4 * x44) + (x43 * x30))) + (x41 * x42))) + (-1 * x34 * ((-1 * x30 * x31) + (x4 * x27)));
	const FLT x47 = -1 * x24;
	const FLT x48 = (*bsc0).gibmag * sin(1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha + (-1 * asin(x42 * x35)) + (-1 * atan2(x22, x47)));
	const FLT x49 = x35 * x26;
	const FLT x50 = 2 * (1. / (x25 + x36)) * x25 * atan2(x35, x47) * (*bsc0).curve;
	const FLT x51 = -2 * ((*lh_p).Rot[1] * (*lh_p).Rot[1]);
	const FLT x52 = x29 + x51;
	const FLT x53 = x2 * (*lh_p).Rot[2];
	const FLT x54 = x38 * (*lh_p).Rot[0];
	const FLT x55 = x54 + x53;
	const FLT x56 = x39 + (-1 * x40);
	const FLT x57 = (-1 * x37 * ((-1 * ((x55 * x44) + (x56 * x43)) * x45) + (x52 * x42))) + (-1 * ((-1 * x56 * x31) + (x55 * x27)) * x34);
	const FLT x58 = 1 + x51 + x28;
	const FLT x59 = x1 + x3;
	const FLT x60 = x53 + (-1 * x54);
	const FLT x61 = (-1 * x37 * ((-1 * ((x58 * x44) + (x59 * x43)) * x45) + (x60 * x42))) + (-1 * ((-1 * x59 * x31) + (x58 * x27)) * x34);
	const FLT x62 = 2 * x11;
	const FLT x63 = 2 * x9;
	const FLT x64 = x63 + (-1 * x62);
	const FLT x65 = 2 * x16;
	const FLT x66 = 2 * x15;
	const FLT x67 = x66 + (-1 * x65);
	const FLT x68 = 2 * x7;
	const FLT x69 = 2 * x5;
	const FLT x70 = x69 + (-1 * x68);
	const FLT x71 = (x70 * (*lh_p).Rot[0]) + (-1 * x64 * (*lh_p).Rot[1]) + (x67 * (*lh_p).Rot[3]);
	const FLT x72 = (x64 * (*lh_p).Rot[2]) + (-1 * x70 * (*lh_p).Rot[3]) + (x67 * (*lh_p).Rot[0]);
	const FLT x73 = x64 + (x71 * x38) + (-1 * x0 * x72);
	const FLT x74 = (x70 * (*lh_p).Rot[1]) + (x64 * (*lh_p).Rot[0]) + (-1 * x67 * (*lh_p).Rot[2]);
	const FLT x75 = x67 + (x0 * x74) + (-1 * x2 * x71);
	const FLT x76 = x70 + (x2 * x72) + (-1 * x74 * x38);
	const FLT x77 = (-1 * x37 * ((-1 * ((x73 * x44) + (x75 * x43)) * x45) + (x76 * x42))) + (-1 * ((-1 * x75 * x31) + (x73 * x27)) * x34);
	const FLT x78 = 2 * sensor_pt[2] * (*obj_p).Rot[3];
	const FLT x79 = 2 * sensor_pt[1] * (*obj_p).Rot[2];
	const FLT x80 = x79 + x78;
	const FLT x81 = 2 * x6;
	const FLT x82 = x69 + x81 + (-4 * x7);
	const FLT x83 = 2 * x10;
	const FLT x84 = (-1 * x83) + (-4 * x9) + x62;
	const FLT x85 = (x84 * (*lh_p).Rot[0]) + (x80 * (*lh_p).Rot[3]) + (-1 * x82 * (*lh_p).Rot[1]);
	const FLT x86 = (x82 * (*lh_p).Rot[2]) + (-1 * x84 * (*lh_p).Rot[3]) + (x80 * (*lh_p).Rot[0]);
	const FLT x87 = (x85 * x38) + x82 + (-1 * x0 * x86);
	const FLT x88 = (x84 * (*lh_p).Rot[1]) + (x82 * (*lh_p).Rot[0]) + (-1 * x80 * (*lh_p).Rot[2]);
	const FLT x89 = x80 + (x0 * x88) + (-1 * x2 * x85);
	const FLT x90 = x84 + (x2 * x86) + (-1 * x88 * x38);
	const FLT x91 = (-1 * x37 * ((-1 * ((x87 * x44) + (x89 * x43)) * x45) + (x90 * x42))) + (-1 * ((-1 * x89 * x31) + (x87 * x27)) * x34);
	const FLT x92 = 2 * x14;
	const FLT x93 = x65 + (-1 * x92) + (-4 * x15);
	const FLT x94 = x83 + x63 + (-4 * x11);
	const FLT x95 = 2 * sensor_pt[0] * (*obj_p).Rot[1];
	const FLT x96 = x78 + x95;
	const FLT x97 = (x96 * (*lh_p).Rot[0]) + (-1 * x93 * (*lh_p).Rot[1]) + (x94 * (*lh_p).Rot[3]);
	const FLT x98 = (x93 * (*lh_p).Rot[2]) + (-1 * x96 * (*lh_p).Rot[3]) + (x94 * (*lh_p).Rot[0]);
	const FLT x99 = x93 + (x97 * x38) + (-1 * x0 * x98);
	const FLT x100 = (x96 * (*lh_p).Rot[1]) + (-1 * x94 * (*lh_p).Rot[2]) + (x93 * (*lh_p).Rot[0]);
	const FLT x101 = x94 + (x0 * x100) + (-1 * x2 * x97);
	const FLT x102 = x96 + (x2 * x98) + (-1 * x38 * x100);
	const FLT x103 = (-1 * x37 * ((-1 * x45 * ((x99 * x44) + (x43 * x101))) + (x42 * x102))) + (-1 * x34 * ((-1 * x31 * x101) + (x99 * x27)));
	const FLT x104 = x95 + x79;
	const FLT x105 = (-1 * x81) + (-4 * x5) + x68;
	const FLT x106 = (-4 * x16) + x92 + x66;
	const FLT x107 = (-1 * x104 * (*lh_p).Rot[1]) + (x106 * (*lh_p).Rot[0]) + (x105 * (*lh_p).Rot[3]);
	const FLT x108 = (x104 * (*lh_p).Rot[2]) + (-1 * x106 * (*lh_p).Rot[3]) + (x105 * (*lh_p).Rot[0]);
	const FLT x109 = x104 + (x38 * x107) + (-1 * x0 * x108);
	const FLT x110 = (x106 * (*lh_p).Rot[1]) + (-1 * x105 * (*lh_p).Rot[2]) + (x104 * (*lh_p).Rot[0]);
	const FLT x111 = x105 + (x0 * x110) + (-1 * x2 * x107);
	const FLT x112 = x106 + (x2 * x108) + (-1 * x38 * x110);
	const FLT x113 = (-1 * x37 * ((-1 * ((x44 * x109) + (x43 * x111)) * x45) + (x42 * x112))) + (-1 * ((-1 * x31 * x111) + (x27 * x109)) * x34);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[0])/sizeof(FLT), x46 + (x46 * x48) + (x50 * ((x4 * x49) + (-1 * x41 * x31))));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[1])/sizeof(FLT), x57 + (((x55 * x49) + (-1 * x52 * x31)) * x50) + (x57 * x48));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[2])/sizeof(FLT), x61 + (x61 * x48) + (((x58 * x49) + (-1 * x60 * x31)) * x50));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[0])/sizeof(FLT), x77 + (x77 * x48) + (((x73 * x49) + (-1 * x76 * x31)) * x50));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[1])/sizeof(FLT), (x91 * x48) + x91 + (((x87 * x49) + (-1 * x90 * x31)) * x50));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[2])/sizeof(FLT), x103 + (x48 * x103) + (x50 * ((x99 * x49) + (-1 * x31 * x102))));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[3])/sizeof(FLT), x113 + (x48 * x113) + (((x49 * x109) + (-1 * x31 * x112)) * x50));
}

// Full version Jacobian of reproject_axis_x wrt [(*obj_p).Pos[0], (*obj_p).Pos[1], (*obj_p).Pos[2], (*obj_p).Rot[0], (*obj_p).Rot[1], (*obj_p).Rot[2], (*obj_p).Rot[3]]

static inline void reproject_axis_x_jac_obj_p_with_hx(CnMat* Hx, CnMat* hx, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc0) {
    if(hx != 0) { 
        hx->data[0] = reproject_axis_x(obj_p, sensor_pt, lh_p, bsc0);
    }
    if(Hx != 0) { 
        reproject_axis_x_jac_obj_p(Hx, obj_p, sensor_pt, lh_p, bsc0);
    }
}
// Jacobian of reproject_axis_x wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]
static inline void reproject_axis_x_jac_sensor_pt(CnMat* Hx, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc0) {
	const FLT x0 = 2 * (*obj_p).Rot[2];
	const FLT x1 = x0 * (*obj_p).Rot[0];
	const FLT x2 = 2 * (*obj_p).Rot[3];
	const FLT x3 = x2 * (*obj_p).Rot[1];
	const FLT x4 = x3 + (-1 * x1);
	const FLT x5 = -2 * ((*obj_p).Rot[3] * (*obj_p).Rot[3]);
	const FLT x6 = -2 * ((*obj_p).Rot[2] * (*obj_p).Rot[2]);
	const FLT x7 = 1 + x6 + x5;
	const FLT x8 = x0 * (*obj_p).Rot[1];
	const FLT x9 = x2 * (*obj_p).Rot[0];
	const FLT x10 = x9 + x8;
	const FLT x11 = (x10 * (*lh_p).Rot[0]) + (-1 * x4 * (*lh_p).Rot[1]) + (x7 * (*lh_p).Rot[3]);
	const FLT x12 = 2 * (*lh_p).Rot[1];
	const FLT x13 = (x4 * (*lh_p).Rot[2]) + (-1 * x10 * (*lh_p).Rot[3]) + (x7 * (*lh_p).Rot[0]);
	const FLT x14 = 2 * (*lh_p).Rot[2];
	const FLT x15 = x4 + (x12 * x11) + (-1 * x14 * x13);
	const FLT x16 = (sensor_pt[0] * (*obj_p).Rot[3]) + (-1 * sensor_pt[2] * (*obj_p).Rot[1]) + (sensor_pt[1] * (*obj_p).Rot[0]);
	const FLT x17 = (sensor_pt[1] * (*obj_p).Rot[1]) + (-1 * sensor_pt[0] * (*obj_p).Rot[2]) + (sensor_pt[2] * (*obj_p).Rot[0]);
	const FLT x18 = (2 * ((x17 * (*obj_p).Rot[2]) + (-1 * x16 * (*obj_p).Rot[3]))) + (*obj_p).Pos[0] + sensor_pt[0];
	const FLT x19 = (-1 * sensor_pt[1] * (*obj_p).Rot[3]) + (sensor_pt[0] * (*obj_p).Rot[0]) + (sensor_pt[2] * (*obj_p).Rot[2]);
	const FLT x20 = sensor_pt[2] + (2 * ((x16 * (*obj_p).Rot[1]) + (-1 * x19 * (*obj_p).Rot[2]))) + (*obj_p).Pos[2];
	const FLT x21 = (2 * ((x19 * (*obj_p).Rot[3]) + (-1 * x17 * (*obj_p).Rot[1]))) + (*obj_p).Pos[1] + sensor_pt[1];
	const FLT x22 = (-1 * x21 * (*lh_p).Rot[3]) + (x18 * (*lh_p).Rot[0]) + (x20 * (*lh_p).Rot[2]);
	const FLT x23 = (-1 * x20 * (*lh_p).Rot[1]) + (x18 * (*lh_p).Rot[3]) + (x21 * (*lh_p).Rot[0]);
	const FLT x24 = x20 + (2 * ((x23 * (*lh_p).Rot[1]) + (-1 * x22 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x25 = x24 * x24;
	const FLT x26 = 1. / x25;
	const FLT x27 = (-1 * x18 * (*lh_p).Rot[2]) + (x20 * (*lh_p).Rot[0]) + (x21 * (*lh_p).Rot[1]);
	const FLT x28 = x18 + (*lh_p).Pos[0] + (2 * ((x27 * (*lh_p).Rot[2]) + (-1 * x23 * (*lh_p).Rot[3])));
	const FLT x29 = x28 * x26;
	const FLT x30 = (x10 * (*lh_p).Rot[1]) + (-1 * x7 * (*lh_p).Rot[2]) + (x4 * (*lh_p).Rot[0]);
	const FLT x31 = 2 * (*lh_p).Rot[3];
	const FLT x32 = x7 + (x30 * x14) + (-1 * x31 * x11);
	const FLT x33 = 1. / x24;
	const FLT x34 = x25 + (x28 * x28);
	const FLT x35 = 1. / x34;
	const FLT x36 = x35 * x25;
	const FLT x37 = x21 + (*lh_p).Pos[1] + (2 * ((x22 * (*lh_p).Rot[3]) + (-1 * x27 * (*lh_p).Rot[1])));
	const FLT x38 = x37 * x37;
	const FLT x39 = 1. / sqrt(1 + (-1 * x35 * x38 * ((*bsc0).tilt * (*bsc0).tilt)));
	const FLT x40 = x10 + (x31 * x13) + (-1 * x30 * x12);
	const FLT x41 = (1. / sqrt(x34)) * (*bsc0).tilt;
	const FLT x42 = 2 * x28;
	const FLT x43 = 2 * x24;
	const FLT x44 = 1.0/2.0 * (1. / (x34 * sqrt(x34))) * x37 * (*bsc0).tilt;
	const FLT x45 = (-1 * x39 * ((-1 * ((x43 * x15) + (x42 * x32)) * x44) + (x40 * x41))) + (-1 * ((-1 * x32 * x33) + (x29 * x15)) * x36);
	const FLT x46 = -1 * x24;
	const FLT x47 = (*bsc0).gibmag * sin(1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha + (-1 * asin(x41 * x37)) + (-1 * atan2(x28, x46)));
	const FLT x48 = x37 * x26;
	const FLT x49 = 2 * (1. / (x25 + x38)) * x25 * atan2(x37, x46) * (*bsc0).curve;
	const FLT x50 = 1 + (-2 * ((*obj_p).Rot[1] * (*obj_p).Rot[1]));
	const FLT x51 = x50 + x5;
	const FLT x52 = x8 + (-1 * x9);
	const FLT x53 = x0 * (*obj_p).Rot[3];
	const FLT x54 = 2 * (*obj_p).Rot[1] * (*obj_p).Rot[0];
	const FLT x55 = x54 + x53;
	const FLT x56 = (x55 * (*lh_p).Rot[2]) + (-1 * x51 * (*lh_p).Rot[3]) + (x52 * (*lh_p).Rot[0]);
	const FLT x57 = (x51 * (*lh_p).Rot[1]) + (-1 * x52 * (*lh_p).Rot[2]) + (x55 * (*lh_p).Rot[0]);
	const FLT x58 = x51 + (x56 * x31) + (-1 * x57 * x12);
	const FLT x59 = (x51 * (*lh_p).Rot[0]) + (x52 * (*lh_p).Rot[3]) + (-1 * x55 * (*lh_p).Rot[1]);
	const FLT x60 = x55 + (x59 * x12) + (-1 * x56 * x14);
	const FLT x61 = (x57 * x14) + x52 + (-1 * x59 * x31);
	const FLT x62 = (-1 * x39 * ((-1 * ((x60 * x43) + (x61 * x42)) * x44) + (x58 * x41))) + (-1 * ((-1 * x61 * x33) + (x60 * x29)) * x36);
	const FLT x63 = x50 + x6;
	const FLT x64 = x1 + x3;
	const FLT x65 = x53 + (-1 * x54);
	const FLT x66 = 2 * ((-1 * x63 * (*lh_p).Rot[1]) + (x65 * (*lh_p).Rot[0]) + (x64 * (*lh_p).Rot[3]));
	const FLT x67 = 2 * ((x63 * (*lh_p).Rot[2]) + (-1 * x65 * (*lh_p).Rot[3]) + (x64 * (*lh_p).Rot[0]));
	const FLT x68 = x63 + (x66 * (*lh_p).Rot[1]) + (-1 * x67 * (*lh_p).Rot[2]);
	const FLT x69 = (x65 * (*lh_p).Rot[1]) + (-1 * x64 * (*lh_p).Rot[2]) + (x63 * (*lh_p).Rot[0]);
	const FLT x70 = x64 + (x69 * x14) + (-1 * x66 * (*lh_p).Rot[3]);
	const FLT x71 = x65 + (x67 * (*lh_p).Rot[3]) + (-1 * x69 * x12);
	const FLT x72 = (-1 * x39 * ((-1 * ((x68 * x43) + (x70 * x42)) * x44) + (x71 * x41))) + (-1 * ((-1 * x70 * x33) + (x68 * x29)) * x36);
	cnMatrixOptionalSet(Hx, 0, 0, x45 + (x45 * x47) + (((x48 * x15) + (-1 * x40 * x33)) * x49));
	cnMatrixOptionalSet(Hx, 0, 1, (((x60 * x48) + (-1 * x58 * x33)) * x49) + x62 + (x62 * x47));
	cnMatrixOptionalSet(Hx, 0, 2, x72 + (x72 * x47) + (((x68 * x48) + (-1 * x71 * x33)) * x49));
}

// Full version Jacobian of reproject_axis_x wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]

static inline void reproject_axis_x_jac_sensor_pt_with_hx(CnMat* Hx, CnMat* hx, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc0) {
    if(hx != 0) { 
        hx->data[0] = reproject_axis_x(obj_p, sensor_pt, lh_p, bsc0);
    }
    if(Hx != 0) { 
        reproject_axis_x_jac_sensor_pt(Hx, obj_p, sensor_pt, lh_p, bsc0);
    }
}
// Jacobian of reproject_axis_x wrt [(*lh_p).Pos[0], (*lh_p).Pos[1], (*lh_p).Pos[2], (*lh_p).Rot[0], (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]
static inline void reproject_axis_x_jac_lh_p(CnMat* Hx, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc0) {
	const FLT x0 = (sensor_pt[0] * (*obj_p).Rot[3]) + (-1 * sensor_pt[2] * (*obj_p).Rot[1]) + (sensor_pt[1] * (*obj_p).Rot[0]);
	const FLT x1 = (sensor_pt[1] * (*obj_p).Rot[1]) + (-1 * sensor_pt[0] * (*obj_p).Rot[2]) + (sensor_pt[2] * (*obj_p).Rot[0]);
	const FLT x2 = 2 * ((x1 * (*obj_p).Rot[2]) + (-1 * x0 * (*obj_p).Rot[3]));
	const FLT x3 = x2 + (*obj_p).Pos[0] + sensor_pt[0];
	const FLT x4 = x3 * (*lh_p).Rot[3];
	const FLT x5 = (-1 * sensor_pt[1] * (*obj_p).Rot[3]) + (sensor_pt[0] * (*obj_p).Rot[0]) + (sensor_pt[2] * (*obj_p).Rot[2]);
	const FLT x6 = 2 * ((x5 * (*obj_p).Rot[3]) + (-1 * x1 * (*obj_p).Rot[1]));
	const FLT x7 = x6 + (*obj_p).Pos[1] + sensor_pt[1];
	const FLT x8 = x7 * (*lh_p).Rot[0];
	const FLT x9 = 2 * ((x0 * (*obj_p).Rot[1]) + (-1 * x5 * (*obj_p).Rot[2]));
	const FLT x10 = x9 + sensor_pt[2] + (*obj_p).Pos[2];
	const FLT x11 = x10 * (*lh_p).Rot[1];
	const FLT x12 = (-1 * x11) + x4 + x8;
	const FLT x13 = x10 * (*lh_p).Rot[0];
	const FLT x14 = x7 * (*lh_p).Rot[1];
	const FLT x15 = x3 * (*lh_p).Rot[2];
	const FLT x16 = (-1 * x15) + x13 + x14;
	const FLT x17 = x3 + (*lh_p).Pos[0] + (2 * ((x16 * (*lh_p).Rot[2]) + (-1 * x12 * (*lh_p).Rot[3])));
	const FLT x18 = x3 * (*lh_p).Rot[0];
	const FLT x19 = x10 * (*lh_p).Rot[2];
	const FLT x20 = x7 * (*lh_p).Rot[3];
	const FLT x21 = (-1 * x20) + x18 + x19;
	const FLT x22 = x10 + (2 * ((x12 * (*lh_p).Rot[1]) + (-1 * x21 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x23 = x22 * x22;
	const FLT x24 = x23 + (x17 * x17);
	const FLT x25 = 1. / x24;
	const FLT x26 = x7 + (*lh_p).Pos[1] + (2 * ((x21 * (*lh_p).Rot[3]) + (-1 * x16 * (*lh_p).Rot[1])));
	const FLT x27 = x26 * x26;
	const FLT x28 = 1. / sqrt(1 + (-1 * x25 * x27 * ((*bsc0).tilt * (*bsc0).tilt)));
	const FLT x29 = (1. / (x24 * sqrt(x24))) * x26 * (*bsc0).tilt;
	const FLT x30 = x28 * x29;
	const FLT x31 = (x30 * x17) + (x25 * x22);
	const FLT x32 = (1. / sqrt(x24)) * (*bsc0).tilt;
	const FLT x33 = -1 * x22;
	const FLT x34 = (*bsc0).gibmag * sin(1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha + (-1 * asin(x32 * x26)) + (-1 * atan2(x17, x33)));
	const FLT x35 = x32 * x28;
	const FLT x36 = 2 * x22;
	const FLT x37 = (1. / (x23 + x27)) * atan2(x26, x33) * (*bsc0).curve;
	const FLT x38 = (x30 * x22) + (-1 * x25 * x17);
	const FLT x39 = 2 * x37;
	const FLT x40 = 2 * x15;
	const FLT x41 = (2 * x14) + (-1 * x40);
	const FLT x42 = 1. / x23;
	const FLT x43 = x42 * x17;
	const FLT x44 = 1. / x22;
	const FLT x45 = 2 * x20;
	const FLT x46 = (2 * x19) + (-1 * x45);
	const FLT x47 = x25 * x23;
	const FLT x48 = 2 * x11;
	const FLT x49 = (2 * x4) + (-1 * x48);
	const FLT x50 = 2 * x17;
	const FLT x51 = 1.0/2.0 * x29;
	const FLT x52 = (-1 * x28 * ((-1 * ((x41 * x36) + (x50 * x46)) * x51) + (x49 * x32))) + (-1 * ((-1 * x44 * x46) + (x41 * x43)) * x47);
	const FLT x53 = x42 * x26;
	const FLT x54 = x39 * x23;
	const FLT x55 = (-1 * x9) + (-1 * sensor_pt[2]) + (-1 * (*obj_p).Pos[2]);
	const FLT x56 = 2 * (*lh_p).Rot[1];
	const FLT x57 = 2 * x8;
	const FLT x58 = x49 + (x56 * x55) + x57;
	const FLT x59 = x58 * x42;
	const FLT x60 = 2 * (*lh_p).Rot[3];
	const FLT x61 = 2 * (*lh_p).Rot[2];
	const FLT x62 = (x7 * x61) + (-1 * x60 * x55);
	const FLT x63 = 2 * x13;
	const FLT x64 = (-4 * x14) + (-1 * x63) + x40;
	const FLT x65 = (-1 * x28 * ((-1 * ((x58 * x36) + (x62 * x50)) * x51) + (x64 * x32))) + (-1 * ((-1 * x62 * x44) + (x59 * x17)) * x47);
	const FLT x66 = (-1 * (*obj_p).Pos[0]) + (-1 * sensor_pt[0]) + (-1 * x2);
	const FLT x67 = (x60 * x10) + (-1 * x66 * x56);
	const FLT x68 = 2 * x18;
	const FLT x69 = (-4 * x19) + x45 + (-1 * x68);
	const FLT x70 = x41 + x63 + (x61 * x66);
	const FLT x71 = (-1 * x28 * ((-1 * ((x69 * x36) + (x70 * x50)) * x51) + (x67 * x32))) + (-1 * ((-1 * x70 * x44) + (x69 * x43)) * x47);
	const FLT x72 = (-1 * (*obj_p).Pos[1]) + (-1 * sensor_pt[1]) + (-1 * x6);
	const FLT x73 = (x3 * x56) + (-1 * x72 * x61);
	const FLT x74 = (-1 * x57) + (-4 * x4) + x48;
	const FLT x75 = (x72 * x60) + x46 + x68;
	const FLT x76 = (-1 * x28 * ((-1 * ((x73 * x36) + (x74 * x50)) * x51) + (x75 * x32))) + (-1 * ((-1 * x74 * x44) + (x73 * x43)) * x47);
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[0])/sizeof(FLT), x31 + (x31 * x34));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[1])/sizeof(FLT), (-1 * x36 * x37) + (-1 * x34 * x35) + (-1 * x35));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[2])/sizeof(FLT), x38 + (x34 * x38) + (x39 * x26));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[0])/sizeof(FLT), x52 + (x52 * x34) + (((x53 * x41) + (-1 * x44 * x49)) * x54));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[1])/sizeof(FLT), x65 + (x65 * x34) + (((x59 * x26) + (-1 * x64 * x44)) * x54));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[2])/sizeof(FLT), x71 + (((x69 * x53) + (-1 * x67 * x44)) * x54) + (x71 * x34));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[3])/sizeof(FLT), x76 + (x76 * x34) + (((x73 * x53) + (-1 * x75 * x44)) * x54));
}

// Full version Jacobian of reproject_axis_x wrt [(*lh_p).Pos[0], (*lh_p).Pos[1], (*lh_p).Pos[2], (*lh_p).Rot[0], (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]

static inline void reproject_axis_x_jac_lh_p_with_hx(CnMat* Hx, CnMat* hx, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc0) {
    if(hx != 0) { 
        hx->data[0] = reproject_axis_x(obj_p, sensor_pt, lh_p, bsc0);
    }
    if(Hx != 0) { 
        reproject_axis_x_jac_lh_p(Hx, obj_p, sensor_pt, lh_p, bsc0);
    }
}
// Jacobian of reproject_axis_x wrt [<cnkalman.codegen.WrapMember object at 0x7fcf8b676040>, <cnkalman.codegen.WrapMember object at 0x7fcf8b676100>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6760a0>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6761c0>, <cnkalman.codegen.WrapMember object at 0x7fcf8b676160>, <cnkalman.codegen.WrapMember object at 0x7fcf8b671df0>, <cnkalman.codegen.WrapMember object at 0x7fcf8b671fa0>]
static inline void reproject_axis_x_jac_bsc0(CnMat* Hx, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc0) {
	const FLT x0 = (-1 * sensor_pt[1] * (*obj_p).Rot[3]) + (sensor_pt[0] * (*obj_p).Rot[0]) + (sensor_pt[2] * (*obj_p).Rot[2]);
	const FLT x1 = (sensor_pt[0] * (*obj_p).Rot[3]) + (-1 * sensor_pt[2] * (*obj_p).Rot[1]) + (sensor_pt[1] * (*obj_p).Rot[0]);
	const FLT x2 = sensor_pt[2] + (2 * ((x1 * (*obj_p).Rot[1]) + (-1 * x0 * (*obj_p).Rot[2]))) + (*obj_p).Pos[2];
	const FLT x3 = (sensor_pt[1] * (*obj_p).Rot[1]) + (-1 * sensor_pt[0] * (*obj_p).Rot[2]) + (sensor_pt[2] * (*obj_p).Rot[0]);
	const FLT x4 = (2 * ((x0 * (*obj_p).Rot[3]) + (-1 * x3 * (*obj_p).Rot[1]))) + (*obj_p).Pos[1] + sensor_pt[1];
	const FLT x5 = (2 * ((x3 * (*obj_p).Rot[2]) + (-1 * x1 * (*obj_p).Rot[3]))) + (*obj_p).Pos[0] + sensor_pt[0];
	const FLT x6 = (-1 * x5 * (*lh_p).Rot[2]) + (x2 * (*lh_p).Rot[0]) + (x4 * (*lh_p).Rot[1]);
	const FLT x7 = (-1 * x4 * (*lh_p).Rot[3]) + (x5 * (*lh_p).Rot[0]) + (x2 * (*lh_p).Rot[2]);
	const FLT x8 = (*lh_p).Pos[1] + x4 + (2 * ((x7 * (*lh_p).Rot[3]) + (-1 * x6 * (*lh_p).Rot[1])));
	const FLT x9 = (-1 * x2 * (*lh_p).Rot[1]) + (x5 * (*lh_p).Rot[3]) + (x4 * (*lh_p).Rot[0]);
	const FLT x10 = (2 * ((x9 * (*lh_p).Rot[1]) + (-1 * x7 * (*lh_p).Rot[2]))) + x2 + (*lh_p).Pos[2];
	const FLT x11 = -1 * x10;
	const FLT x12 = x5 + (*lh_p).Pos[0] + (2 * ((x6 * (*lh_p).Rot[2]) + (-1 * x9 * (*lh_p).Rot[3])));
	const FLT x13 = (x12 * x12) + (x10 * x10);
	const FLT x14 = x8 * (1. / sqrt(x13));
	const FLT x15 = 1.5707963267949 + (-1 * (*bsc0).phase) + (*bsc0).gibpha + (-1 * asin(x14 * (*bsc0).tilt)) + (-1 * atan2(x12, x11));
	const FLT x16 = sin(x15) * (*bsc0).gibmag;
	const FLT x17 = x14 * (1. / sqrt(1 + (-1 * (x8 * x8) * (1. / x13) * ((*bsc0).tilt * (*bsc0).tilt))));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, curve)/sizeof(FLT), atan2(x8, x11) * atan2(x8, x11));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibmag)/sizeof(FLT), -1 * cos(x15));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibpha)/sizeof(FLT), x16);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, phase)/sizeof(FLT), -1 + (-1 * x16));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, tilt)/sizeof(FLT), (-1 * x17) + (-1 * x17 * x16));
}

// Full version Jacobian of reproject_axis_x wrt [<cnkalman.codegen.WrapMember object at 0x7fcf8b676040>, <cnkalman.codegen.WrapMember object at 0x7fcf8b676100>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6760a0>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6761c0>, <cnkalman.codegen.WrapMember object at 0x7fcf8b676160>, <cnkalman.codegen.WrapMember object at 0x7fcf8b671df0>, <cnkalman.codegen.WrapMember object at 0x7fcf8b671fa0>]

static inline void reproject_axis_x_jac_bsc0_with_hx(CnMat* Hx, CnMat* hx, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc0) {
    if(hx != 0) { 
        hx->data[0] = reproject_axis_x(obj_p, sensor_pt, lh_p, bsc0);
    }
    if(Hx != 0) { 
        reproject_axis_x_jac_bsc0(Hx, obj_p, sensor_pt, lh_p, bsc0);
    }
}
static inline FLT reproject_axis_y(const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc1) {
	const FLT x0 = (sensor_pt[0] * (*obj_p).Rot[3]) + (-1 * sensor_pt[2] * (*obj_p).Rot[1]) + (sensor_pt[1] * (*obj_p).Rot[0]);
	const FLT x1 = (sensor_pt[1] * (*obj_p).Rot[1]) + (-1 * sensor_pt[0] * (*obj_p).Rot[2]) + (sensor_pt[2] * (*obj_p).Rot[0]);
	const FLT x2 = (2 * ((x1 * (*obj_p).Rot[2]) + (-1 * x0 * (*obj_p).Rot[3]))) + (*obj_p).Pos[0] + sensor_pt[0];
	const FLT x3 = (-1 * sensor_pt[1] * (*obj_p).Rot[3]) + (sensor_pt[0] * (*obj_p).Rot[0]) + (sensor_pt[2] * (*obj_p).Rot[2]);
	const FLT x4 = (2 * ((x3 * (*obj_p).Rot[3]) + (-1 * x1 * (*obj_p).Rot[1]))) + (*obj_p).Pos[1] + sensor_pt[1];
	const FLT x5 = (2 * ((x0 * (*obj_p).Rot[1]) + (-1 * x3 * (*obj_p).Rot[2]))) + sensor_pt[2] + (*obj_p).Pos[2];
	const FLT x6 = (x2 * (*lh_p).Rot[3]) + (-1 * x5 * (*lh_p).Rot[1]) + (x4 * (*lh_p).Rot[0]);
	const FLT x7 = (-1 * x2 * (*lh_p).Rot[2]) + (x5 * (*lh_p).Rot[0]) + (x4 * (*lh_p).Rot[1]);
	const FLT x8 = x2 + (*lh_p).Pos[0] + (2 * ((x7 * (*lh_p).Rot[2]) + (-1 * x6 * (*lh_p).Rot[3])));
	const FLT x9 = (-1 * x4 * (*lh_p).Rot[3]) + (x2 * (*lh_p).Rot[0]) + (x5 * (*lh_p).Rot[2]);
	const FLT x10 = (2 * ((x6 * (*lh_p).Rot[1]) + (-1 * x9 * (*lh_p).Rot[2]))) + x5 + (*lh_p).Pos[2];
	const FLT x11 = -1 * x10;
	const FLT x12 = x4 + (*lh_p).Pos[1] + (2 * ((x9 * (*lh_p).Rot[3]) + (-1 * x7 * (*lh_p).Rot[1])));
	const FLT x13 = (-1 * atan2(-1 * x12, x11)) + (-1 * (*bsc1).phase) + (-1 * asin((1. / sqrt((x12 * x12) + (x10 * x10))) * x8 * (*bsc1).tilt));
	return x13 + ((atan2(x8, x11) * atan2(x8, x11)) * (*bsc1).curve) + (-1 * cos(1.5707963267949 + x13 + (*bsc1).gibpha) * (*bsc1).gibmag);
}

// Jacobian of reproject_axis_y wrt [(*obj_p).Pos[0], (*obj_p).Pos[1], (*obj_p).Pos[2], (*obj_p).Rot[0], (*obj_p).Rot[1], (*obj_p).Rot[2], (*obj_p).Rot[3]]
static inline void reproject_axis_y_jac_obj_p(CnMat* Hx, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc1) {
	const FLT x0 = 2 * (*lh_p).Rot[2];
	const FLT x1 = x0 * (*lh_p).Rot[0];
	const FLT x2 = 2 * (*lh_p).Rot[3];
	const FLT x3 = x2 * (*lh_p).Rot[1];
	const FLT x4 = x3 + (-1 * x1);
	const FLT x5 = sensor_pt[0] * (*obj_p).Rot[3];
	const FLT x6 = sensor_pt[1] * (*obj_p).Rot[0];
	const FLT x7 = sensor_pt[2] * (*obj_p).Rot[1];
	const FLT x8 = (-1 * x7) + x5 + x6;
	const FLT x9 = sensor_pt[1] * (*obj_p).Rot[1];
	const FLT x10 = sensor_pt[2] * (*obj_p).Rot[0];
	const FLT x11 = sensor_pt[0] * (*obj_p).Rot[2];
	const FLT x12 = (-1 * x11) + x9 + x10;
	const FLT x13 = (2 * ((x12 * (*obj_p).Rot[2]) + (-1 * x8 * (*obj_p).Rot[3]))) + (*obj_p).Pos[0] + sensor_pt[0];
	const FLT x14 = sensor_pt[0] * (*obj_p).Rot[0];
	const FLT x15 = sensor_pt[2] * (*obj_p).Rot[2];
	const FLT x16 = sensor_pt[1] * (*obj_p).Rot[3];
	const FLT x17 = x14 + (-1 * x16) + x15;
	const FLT x18 = (2 * ((x17 * (*obj_p).Rot[3]) + (-1 * x12 * (*obj_p).Rot[1]))) + (*obj_p).Pos[1] + sensor_pt[1];
	const FLT x19 = sensor_pt[2] + (2 * ((x8 * (*obj_p).Rot[1]) + (-1 * x17 * (*obj_p).Rot[2]))) + (*obj_p).Pos[2];
	const FLT x20 = (-1 * x19 * (*lh_p).Rot[1]) + (x13 * (*lh_p).Rot[3]) + (x18 * (*lh_p).Rot[0]);
	const FLT x21 = (-1 * x13 * (*lh_p).Rot[2]) + (x19 * (*lh_p).Rot[0]) + (x18 * (*lh_p).Rot[1]);
	const FLT x22 = x13 + (*lh_p).Pos[0] + (2 * ((x21 * (*lh_p).Rot[2]) + (-1 * x20 * (*lh_p).Rot[3])));
	const FLT x23 = (-1 * x18 * (*lh_p).Rot[3]) + (x13 * (*lh_p).Rot[0]) + (x19 * (*lh_p).Rot[2]);
	const FLT x24 = x19 + (2 * ((x20 * (*lh_p).Rot[1]) + (-1 * x23 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x25 = x24 * x24;
	const FLT x26 = 1. / x25;
	const FLT x27 = x22 * x26;
	const FLT x28 = -2 * ((*lh_p).Rot[2] * (*lh_p).Rot[2]);
	const FLT x29 = 1 + (-2 * ((*lh_p).Rot[3] * (*lh_p).Rot[3]));
	const FLT x30 = x29 + x28;
	const FLT x31 = 1. / x24;
	const FLT x32 = -1 * x24;
	const FLT x33 = x22 * x22;
	const FLT x34 = 2 * (1. / (x25 + x33)) * x25 * atan2(x22, x32) * (*bsc1).curve;
	const FLT x35 = x18 + (*lh_p).Pos[1] + (2 * ((x23 * (*lh_p).Rot[3]) + (-1 * x21 * (*lh_p).Rot[1])));
	const FLT x36 = (x35 * x35) + x25;
	const FLT x37 = 1. / x36;
	const FLT x38 = 1. / sqrt(1 + (-1 * x33 * x37 * ((*bsc1).tilt * (*bsc1).tilt)));
	const FLT x39 = x0 * (*lh_p).Rot[1];
	const FLT x40 = x2 * (*lh_p).Rot[0];
	const FLT x41 = x40 + x39;
	const FLT x42 = 2 * x35;
	const FLT x43 = 2 * x24;
	const FLT x44 = 1.0/2.0 * (1. / (x36 * sqrt(x36))) * x22 * (*bsc1).tilt;
	const FLT x45 = (1. / sqrt(x36)) * (*bsc1).tilt;
	const FLT x46 = x35 * x26;
	const FLT x47 = x37 * x25;
	const FLT x48 = (-1 * x47 * ((-1 * x4 * x46) + (x41 * x31))) + (-1 * x38 * ((x45 * x30) + (-1 * x44 * ((x4 * x43) + (x41 * x42)))));
	const FLT x49 = sin(1.5707963267949 + (-1 * (*bsc1).phase) + (-1 * asin(x45 * x22)) + (*bsc1).gibpha + (-1 * atan2(-1 * x35, x32))) * (*bsc1).gibmag;
	const FLT x50 = x0 * (*lh_p).Rot[3];
	const FLT x51 = 2 * (*lh_p).Rot[1];
	const FLT x52 = x51 * (*lh_p).Rot[0];
	const FLT x53 = x52 + x50;
	const FLT x54 = x53 * x26;
	const FLT x55 = x39 + (-1 * x40);
	const FLT x56 = -2 * ((*lh_p).Rot[1] * (*lh_p).Rot[1]);
	const FLT x57 = x29 + x56;
	const FLT x58 = (-1 * ((-1 * x54 * x35) + (x57 * x31)) * x47) + (-1 * x38 * ((x55 * x45) + (-1 * ((x53 * x43) + (x57 * x42)) * x44)));
	const FLT x59 = 1 + x56 + x28;
	const FLT x60 = x1 + x3;
	const FLT x61 = x50 + (-1 * x52);
	const FLT x62 = (-1 * ((-1 * x59 * x46) + (x61 * x31)) * x47) + (-1 * x38 * ((x60 * x45) + (-1 * ((x59 * x43) + (x61 * x42)) * x44)));
	const FLT x63 = 2 * x11;
	const FLT x64 = 2 * x9;
	const FLT x65 = x64 + (-1 * x63);
	const FLT x66 = 2 * x16;
	const FLT x67 = 2 * x15;
	const FLT x68 = x67 + (-1 * x66);
	const FLT x69 = 2 * x7;
	const FLT x70 = 2 * x5;
	const FLT x71 = x70 + (-1 * x69);
	const FLT x72 = 2 * ((x71 * (*lh_p).Rot[0]) + (-1 * x65 * (*lh_p).Rot[1]) + (x68 * (*lh_p).Rot[3]));
	const FLT x73 = 2 * ((x65 * (*lh_p).Rot[2]) + (-1 * x71 * (*lh_p).Rot[3]) + (x68 * (*lh_p).Rot[0]));
	const FLT x74 = (x72 * (*lh_p).Rot[1]) + x65 + (-1 * x73 * (*lh_p).Rot[2]);
	const FLT x75 = 2 * ((x71 * (*lh_p).Rot[1]) + (x65 * (*lh_p).Rot[0]) + (-1 * x68 * (*lh_p).Rot[2]));
	const FLT x76 = (x75 * (*lh_p).Rot[2]) + x68 + (-1 * x72 * (*lh_p).Rot[3]);
	const FLT x77 = x71 + (x73 * (*lh_p).Rot[3]) + (-1 * x75 * (*lh_p).Rot[1]);
	const FLT x78 = (-1 * ((-1 * x74 * x46) + (x77 * x31)) * x47) + (-1 * x38 * ((x76 * x45) + (-1 * ((x74 * x43) + (x77 * x42)) * x44)));
	const FLT x79 = 2 * sensor_pt[2] * (*obj_p).Rot[3];
	const FLT x80 = 2 * sensor_pt[1] * (*obj_p).Rot[2];
	const FLT x81 = x80 + x79;
	const FLT x82 = 2 * x6;
	const FLT x83 = x82 + x70 + (-4 * x7);
	const FLT x84 = 2 * x10;
	const FLT x85 = (-1 * x84) + (-4 * x9) + x63;
	const FLT x86 = (x81 * (*lh_p).Rot[3]) + (x85 * (*lh_p).Rot[0]) + (-1 * x83 * (*lh_p).Rot[1]);
	const FLT x87 = (-1 * x85 * (*lh_p).Rot[3]) + (x83 * (*lh_p).Rot[2]) + (x81 * (*lh_p).Rot[0]);
	const FLT x88 = x83 + (x86 * x51) + (-1 * x0 * x87);
	const FLT x89 = (x85 * (*lh_p).Rot[1]) + (x83 * (*lh_p).Rot[0]) + (-1 * x81 * (*lh_p).Rot[2]);
	const FLT x90 = x81 + (x0 * x89) + (-1 * x2 * x86);
	const FLT x91 = x85 + (x2 * x87) + (-1 * x89 * x51);
	const FLT x92 = (-1 * ((-1 * x88 * x46) + (x91 * x31)) * x47) + (-1 * x38 * ((x90 * x45) + (-1 * ((x88 * x43) + (x91 * x42)) * x44)));
	const FLT x93 = 2 * x14;
	const FLT x94 = x66 + (-1 * x93) + (-4 * x15);
	const FLT x95 = x84 + x64 + (-4 * x11);
	const FLT x96 = 2 * sensor_pt[0] * (*obj_p).Rot[1];
	const FLT x97 = x79 + x96;
	const FLT x98 = (x97 * (*lh_p).Rot[0]) + (-1 * x94 * (*lh_p).Rot[1]) + (x95 * (*lh_p).Rot[3]);
	const FLT x99 = (x94 * (*lh_p).Rot[2]) + (-1 * x97 * (*lh_p).Rot[3]) + (x95 * (*lh_p).Rot[0]);
	const FLT x100 = (x51 * x98) + x94 + (-1 * x0 * x99);
	const FLT x101 = (x97 * (*lh_p).Rot[1]) + (-1 * x95 * (*lh_p).Rot[2]) + (x94 * (*lh_p).Rot[0]);
	const FLT x102 = x95 + (x0 * x101) + (-1 * x2 * x98);
	const FLT x103 = x97 + (x2 * x99) + (-1 * x51 * x101);
	const FLT x104 = (-1 * ((-1 * x46 * x100) + (x31 * x103)) * x47) + (-1 * x38 * ((x45 * x102) + (-1 * ((x43 * x100) + (x42 * x103)) * x44)));
	const FLT x105 = x96 + x80;
	const FLT x106 = (-1 * x82) + (-4 * x5) + x69;
	const FLT x107 = (-4 * x16) + x93 + x67;
	const FLT x108 = 2 * ((x107 * (*lh_p).Rot[0]) + (-1 * x105 * (*lh_p).Rot[1]) + (x106 * (*lh_p).Rot[3]));
	const FLT x109 = 2 * ((-1 * x107 * (*lh_p).Rot[3]) + (x105 * (*lh_p).Rot[2]) + (x106 * (*lh_p).Rot[0]));
	const FLT x110 = x105 + (x108 * (*lh_p).Rot[1]) + (-1 * x109 * (*lh_p).Rot[2]);
	const FLT x111 = 2 * ((x107 * (*lh_p).Rot[1]) + (-1 * x106 * (*lh_p).Rot[2]) + (x105 * (*lh_p).Rot[0]));
	const FLT x112 = (x111 * (*lh_p).Rot[2]) + x106 + (-1 * x108 * (*lh_p).Rot[3]);
	const FLT x113 = x107 + (x109 * (*lh_p).Rot[3]) + (-1 * x111 * (*lh_p).Rot[1]);
	const FLT x114 = (-1 * ((-1 * x46 * x110) + (x31 * x113)) * x47) + (-1 * x38 * ((x45 * x112) + (-1 * ((x43 * x110) + (x42 * x113)) * x44)));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[0])/sizeof(FLT), x48 + (x34 * ((-1 * x30 * x31) + (x4 * x27))) + (x48 * x49));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[1])/sizeof(FLT), x58 + (((-1 * x55 * x31) + (x54 * x22)) * x34) + (x58 * x49));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[2])/sizeof(FLT), x62 + (((-1 * x60 * x31) + (x59 * x27)) * x34) + (x62 * x49));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[0])/sizeof(FLT), (((-1 * x76 * x31) + (x74 * x27)) * x34) + x78 + (x78 * x49));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[1])/sizeof(FLT), x92 + (((-1 * x90 * x31) + (x88 * x27)) * x34) + (x92 * x49));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[2])/sizeof(FLT), x104 + (((-1 * x31 * x102) + (x27 * x100)) * x34) + (x49 * x104));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[3])/sizeof(FLT), x114 + (((-1 * x31 * x112) + (x27 * x110)) * x34) + (x49 * x114));
}

// Full version Jacobian of reproject_axis_y wrt [(*obj_p).Pos[0], (*obj_p).Pos[1], (*obj_p).Pos[2], (*obj_p).Rot[0], (*obj_p).Rot[1], (*obj_p).Rot[2], (*obj_p).Rot[3]]

static inline void reproject_axis_y_jac_obj_p_with_hx(CnMat* Hx, CnMat* hx, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc1) {
    if(hx != 0) { 
        hx->data[0] = reproject_axis_y(obj_p, sensor_pt, lh_p, bsc1);
    }
    if(Hx != 0) { 
        reproject_axis_y_jac_obj_p(Hx, obj_p, sensor_pt, lh_p, bsc1);
    }
}
// Jacobian of reproject_axis_y wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]
static inline void reproject_axis_y_jac_sensor_pt(CnMat* Hx, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc1) {
	const FLT x0 = 2 * (*obj_p).Rot[2];
	const FLT x1 = x0 * (*obj_p).Rot[0];
	const FLT x2 = 2 * (*obj_p).Rot[3];
	const FLT x3 = x2 * (*obj_p).Rot[1];
	const FLT x4 = x3 + (-1 * x1);
	const FLT x5 = -2 * ((*obj_p).Rot[3] * (*obj_p).Rot[3]);
	const FLT x6 = -2 * ((*obj_p).Rot[2] * (*obj_p).Rot[2]);
	const FLT x7 = 1 + x6 + x5;
	const FLT x8 = x0 * (*obj_p).Rot[1];
	const FLT x9 = x2 * (*obj_p).Rot[0];
	const FLT x10 = x9 + x8;
	const FLT x11 = (x10 * (*lh_p).Rot[0]) + (-1 * x4 * (*lh_p).Rot[1]) + (x7 * (*lh_p).Rot[3]);
	const FLT x12 = 2 * (*lh_p).Rot[1];
	const FLT x13 = (x4 * (*lh_p).Rot[2]) + (-1 * x10 * (*lh_p).Rot[3]) + (x7 * (*lh_p).Rot[0]);
	const FLT x14 = 2 * (*lh_p).Rot[2];
	const FLT x15 = x4 + (x12 * x11) + (-1 * x14 * x13);
	const FLT x16 = (sensor_pt[0] * (*obj_p).Rot[3]) + (-1 * sensor_pt[2] * (*obj_p).Rot[1]) + (sensor_pt[1] * (*obj_p).Rot[0]);
	const FLT x17 = (sensor_pt[1] * (*obj_p).Rot[1]) + (-1 * sensor_pt[0] * (*obj_p).Rot[2]) + (sensor_pt[2] * (*obj_p).Rot[0]);
	const FLT x18 = (2 * ((x17 * (*obj_p).Rot[2]) + (-1 * x16 * (*obj_p).Rot[3]))) + (*obj_p).Pos[0] + sensor_pt[0];
	const FLT x19 = (-1 * sensor_pt[1] * (*obj_p).Rot[3]) + (sensor_pt[0] * (*obj_p).Rot[0]) + (sensor_pt[2] * (*obj_p).Rot[2]);
	const FLT x20 = (2 * ((x19 * (*obj_p).Rot[3]) + (-1 * x17 * (*obj_p).Rot[1]))) + (*obj_p).Pos[1] + sensor_pt[1];
	const FLT x21 = sensor_pt[2] + (2 * ((x16 * (*obj_p).Rot[1]) + (-1 * x19 * (*obj_p).Rot[2]))) + (*obj_p).Pos[2];
	const FLT x22 = (-1 * x21 * (*lh_p).Rot[1]) + (x18 * (*lh_p).Rot[3]) + (x20 * (*lh_p).Rot[0]);
	const FLT x23 = (x21 * (*lh_p).Rot[0]) + (-1 * x18 * (*lh_p).Rot[2]) + (x20 * (*lh_p).Rot[1]);
	const FLT x24 = x18 + (*lh_p).Pos[0] + (2 * ((x23 * (*lh_p).Rot[2]) + (-1 * x22 * (*lh_p).Rot[3])));
	const FLT x25 = (-1 * x20 * (*lh_p).Rot[3]) + (x18 * (*lh_p).Rot[0]) + (x21 * (*lh_p).Rot[2]);
	const FLT x26 = (2 * ((x22 * (*lh_p).Rot[1]) + (-1 * x25 * (*lh_p).Rot[2]))) + x21 + (*lh_p).Pos[2];
	const FLT x27 = x26 * x26;
	const FLT x28 = 1. / x27;
	const FLT x29 = x24 * x28;
	const FLT x30 = (x10 * (*lh_p).Rot[1]) + (-1 * x7 * (*lh_p).Rot[2]) + (x4 * (*lh_p).Rot[0]);
	const FLT x31 = 2 * (*lh_p).Rot[3];
	const FLT x32 = x7 + (x30 * x14) + (-1 * x31 * x11);
	const FLT x33 = 1. / x26;
	const FLT x34 = -1 * x26;
	const FLT x35 = x24 * x24;
	const FLT x36 = 2 * (1. / (x27 + x35)) * x27 * atan2(x24, x34) * (*bsc1).curve;
	const FLT x37 = (*lh_p).Pos[1] + x20 + (2 * ((x25 * (*lh_p).Rot[3]) + (-1 * x23 * (*lh_p).Rot[1])));
	const FLT x38 = (x37 * x37) + x27;
	const FLT x39 = 1. / x38;
	const FLT x40 = 1. / sqrt(1 + (-1 * x35 * x39 * ((*bsc1).tilt * (*bsc1).tilt)));
	const FLT x41 = x10 + (x31 * x13) + (-1 * x30 * x12);
	const FLT x42 = 2 * x37;
	const FLT x43 = 2 * x26;
	const FLT x44 = 1.0/2.0 * (1. / (x38 * sqrt(x38))) * x24 * (*bsc1).tilt;
	const FLT x45 = (1. / sqrt(x38)) * (*bsc1).tilt;
	const FLT x46 = x37 * x28;
	const FLT x47 = x39 * x27;
	const FLT x48 = (-1 * ((-1 * x46 * x15) + (x41 * x33)) * x47) + (-1 * x40 * ((x45 * x32) + (-1 * ((x43 * x15) + (x41 * x42)) * x44)));
	const FLT x49 = sin(1.5707963267949 + (-1 * asin(x45 * x24)) + (*bsc1).gibpha + (-1 * (*bsc1).phase) + (-1 * atan2(-1 * x37, x34))) * (*bsc1).gibmag;
	const FLT x50 = x8 + (-1 * x9);
	const FLT x51 = x0 * (*obj_p).Rot[3];
	const FLT x52 = 2 * (*obj_p).Rot[1] * (*obj_p).Rot[0];
	const FLT x53 = x52 + x51;
	const FLT x54 = 1 + (-2 * ((*obj_p).Rot[1] * (*obj_p).Rot[1]));
	const FLT x55 = x54 + x5;
	const FLT x56 = (x50 * (*lh_p).Rot[3]) + (x55 * (*lh_p).Rot[0]) + (-1 * x53 * (*lh_p).Rot[1]);
	const FLT x57 = (x53 * (*lh_p).Rot[2]) + (-1 * x55 * (*lh_p).Rot[3]) + (x50 * (*lh_p).Rot[0]);
	const FLT x58 = x53 + (x56 * x12) + (-1 * x57 * x14);
	const FLT x59 = (x55 * (*lh_p).Rot[1]) + (-1 * x50 * (*lh_p).Rot[2]) + (x53 * (*lh_p).Rot[0]);
	const FLT x60 = x50 + (x59 * x14) + (-1 * x56 * x31);
	const FLT x61 = x55 + (x57 * x31) + (-1 * x59 * x12);
	const FLT x62 = (-1 * ((-1 * x58 * x46) + (x61 * x33)) * x47) + (-1 * x40 * ((x60 * x45) + (-1 * ((x58 * x43) + (x61 * x42)) * x44)));
	const FLT x63 = x54 + x6;
	const FLT x64 = x1 + x3;
	const FLT x65 = x51 + (-1 * x52);
	const FLT x66 = (-1 * x63 * (*lh_p).Rot[1]) + (x65 * (*lh_p).Rot[0]) + (x64 * (*lh_p).Rot[3]);
	const FLT x67 = (x63 * (*lh_p).Rot[2]) + (-1 * x65 * (*lh_p).Rot[3]) + (x64 * (*lh_p).Rot[0]);
	const FLT x68 = (x66 * x12) + x63 + (-1 * x67 * x14);
	const FLT x69 = x68 * x28;
	const FLT x70 = (x65 * (*lh_p).Rot[1]) + (-1 * x64 * (*lh_p).Rot[2]) + (x63 * (*lh_p).Rot[0]);
	const FLT x71 = x64 + (x70 * x14) + (-1 * x66 * x31);
	const FLT x72 = x65 + (x67 * x31) + (-1 * x70 * x12);
	const FLT x73 = (-1 * ((-1 * x69 * x37) + (x72 * x33)) * x47) + (-1 * x40 * ((x71 * x45) + (-1 * ((x68 * x43) + (x72 * x42)) * x44)));
	cnMatrixOptionalSet(Hx, 0, 0, x48 + (((-1 * x32 * x33) + (x29 * x15)) * x36) + (x48 * x49));
	cnMatrixOptionalSet(Hx, 0, 1, x62 + (((-1 * x60 * x33) + (x58 * x29)) * x36) + (x62 * x49));
	cnMatrixOptionalSet(Hx, 0, 2, x73 + (((-1 * x71 * x33) + (x69 * x24)) * x36) + (x73 * x49));
}

// Full version Jacobian of reproject_axis_y wrt [sensor_pt[0], sensor_pt[1], sensor_pt[2]]

static inline void reproject_axis_y_jac_sensor_pt_with_hx(CnMat* Hx, CnMat* hx, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc1) {
    if(hx != 0) { 
        hx->data[0] = reproject_axis_y(obj_p, sensor_pt, lh_p, bsc1);
    }
    if(Hx != 0) { 
        reproject_axis_y_jac_sensor_pt(Hx, obj_p, sensor_pt, lh_p, bsc1);
    }
}
// Jacobian of reproject_axis_y wrt [(*lh_p).Pos[0], (*lh_p).Pos[1], (*lh_p).Pos[2], (*lh_p).Rot[0], (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]
static inline void reproject_axis_y_jac_lh_p(CnMat* Hx, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc1) {
	const FLT x0 = (sensor_pt[0] * (*obj_p).Rot[3]) + (-1 * sensor_pt[2] * (*obj_p).Rot[1]) + (sensor_pt[1] * (*obj_p).Rot[0]);
	const FLT x1 = (sensor_pt[1] * (*obj_p).Rot[1]) + (-1 * sensor_pt[0] * (*obj_p).Rot[2]) + (sensor_pt[2] * (*obj_p).Rot[0]);
	const FLT x2 = 2 * ((x1 * (*obj_p).Rot[2]) + (-1 * x0 * (*obj_p).Rot[3]));
	const FLT x3 = x2 + (*obj_p).Pos[0] + sensor_pt[0];
	const FLT x4 = x3 * (*lh_p).Rot[3];
	const FLT x5 = (-1 * sensor_pt[1] * (*obj_p).Rot[3]) + (sensor_pt[0] * (*obj_p).Rot[0]) + (sensor_pt[2] * (*obj_p).Rot[2]);
	const FLT x6 = 2 * ((x5 * (*obj_p).Rot[3]) + (-1 * x1 * (*obj_p).Rot[1]));
	const FLT x7 = x6 + (*obj_p).Pos[1] + sensor_pt[1];
	const FLT x8 = x7 * (*lh_p).Rot[0];
	const FLT x9 = 2 * ((x0 * (*obj_p).Rot[1]) + (-1 * x5 * (*obj_p).Rot[2]));
	const FLT x10 = x9 + sensor_pt[2] + (*obj_p).Pos[2];
	const FLT x11 = x10 * (*lh_p).Rot[1];
	const FLT x12 = (-1 * x11) + x4 + x8;
	const FLT x13 = x10 * (*lh_p).Rot[0];
	const FLT x14 = x7 * (*lh_p).Rot[1];
	const FLT x15 = x3 * (*lh_p).Rot[2];
	const FLT x16 = (-1 * x15) + x13 + x14;
	const FLT x17 = x3 + (*lh_p).Pos[0] + (2 * ((x16 * (*lh_p).Rot[2]) + (-1 * x12 * (*lh_p).Rot[3])));
	const FLT x18 = x17 * x17;
	const FLT x19 = x3 * (*lh_p).Rot[0];
	const FLT x20 = x10 * (*lh_p).Rot[2];
	const FLT x21 = x7 * (*lh_p).Rot[3];
	const FLT x22 = (-1 * x21) + x19 + x20;
	const FLT x23 = x10 + (2 * ((x12 * (*lh_p).Rot[1]) + (-1 * x22 * (*lh_p).Rot[2]))) + (*lh_p).Pos[2];
	const FLT x24 = x23 * x23;
	const FLT x25 = x7 + (*lh_p).Pos[1] + (2 * ((x22 * (*lh_p).Rot[3]) + (-1 * x16 * (*lh_p).Rot[1])));
	const FLT x26 = (x25 * x25) + x24;
	const FLT x27 = 1. / x26;
	const FLT x28 = 1. / sqrt(1 + (-1 * x27 * x18 * ((*bsc1).tilt * (*bsc1).tilt)));
	const FLT x29 = (1. / sqrt(x26)) * (*bsc1).tilt;
	const FLT x30 = x28 * x29;
	const FLT x31 = 2 * x23;
	const FLT x32 = -1 * x23;
	const FLT x33 = (1. / (x24 + x18)) * atan2(x17, x32) * (*bsc1).curve;
	const FLT x34 = sin(1.5707963267949 + (-1 * asin(x29 * x17)) + (*bsc1).gibpha + (-1 * (*bsc1).phase) + (-1 * atan2(-1 * x25, x32))) * (*bsc1).gibmag;
	const FLT x35 = (1. / (x26 * sqrt(x26))) * x17 * (*bsc1).tilt;
	const FLT x36 = x35 * x28;
	const FLT x37 = (-1 * x23 * x27) + (x36 * x25);
	const FLT x38 = 2 * x33;
	const FLT x39 = (x25 * x27) + (x36 * x23);
	const FLT x40 = 1. / x24;
	const FLT x41 = 2 * x15;
	const FLT x42 = (2 * x14) + (-1 * x41);
	const FLT x43 = x40 * x42;
	const FLT x44 = 1. / x23;
	const FLT x45 = 2 * x21;
	const FLT x46 = (2 * x20) + (-1 * x45);
	const FLT x47 = x38 * x24;
	const FLT x48 = 2 * x11;
	const FLT x49 = (2 * x4) + (-1 * x48);
	const FLT x50 = 2 * x25;
	const FLT x51 = 1.0/2.0 * x35;
	const FLT x52 = x24 * x27;
	const FLT x53 = (-1 * ((-1 * x43 * x25) + (x44 * x49)) * x52) + (-1 * x28 * ((x46 * x29) + (-1 * ((x42 * x31) + (x50 * x49)) * x51)));
	const FLT x54 = (-1 * x9) + (-1 * sensor_pt[2]) + (-1 * (*obj_p).Pos[2]);
	const FLT x55 = 2 * (*lh_p).Rot[1];
	const FLT x56 = 2 * x8;
	const FLT x57 = x49 + (x54 * x55) + x56;
	const FLT x58 = x57 * x40;
	const FLT x59 = 2 * (*lh_p).Rot[3];
	const FLT x60 = 2 * (*lh_p).Rot[2];
	const FLT x61 = (x7 * x60) + (-1 * x54 * x59);
	const FLT x62 = 2 * x13;
	const FLT x63 = (-4 * x14) + (-1 * x62) + x41;
	const FLT x64 = (-1 * ((-1 * x58 * x25) + (x63 * x44)) * x52) + (-1 * x28 * ((x61 * x29) + (-1 * ((x57 * x31) + (x63 * x50)) * x51)));
	const FLT x65 = 2 * x19;
	const FLT x66 = (-4 * x20) + x45 + (-1 * x65);
	const FLT x67 = x66 * x40;
	const FLT x68 = (-1 * (*obj_p).Pos[0]) + (-1 * sensor_pt[0]) + (-1 * x2);
	const FLT x69 = x42 + x62 + (x60 * x68);
	const FLT x70 = (x59 * x10) + (-1 * x68 * x55);
	const FLT x71 = (-1 * ((-1 * x67 * x25) + (x70 * x44)) * x52) + (-1 * x28 * ((x69 * x29) + (-1 * ((x66 * x31) + (x70 * x50)) * x51)));
	const FLT x72 = (-1 * (*obj_p).Pos[1]) + (-1 * sensor_pt[1]) + (-1 * x6);
	const FLT x73 = (x3 * x55) + (-1 * x72 * x60);
	const FLT x74 = x73 * x40;
	const FLT x75 = (-1 * x56) + (-4 * x4) + x48;
	const FLT x76 = x46 + (x72 * x59) + x65;
	const FLT x77 = (-1 * ((-1 * x74 * x25) + (x76 * x44)) * x52) + (-1 * x28 * ((x75 * x29) + (-1 * ((x73 * x31) + (x76 * x50)) * x51)));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[0])/sizeof(FLT), (-1 * x30 * x34) + (-1 * x30) + (-1 * x31 * x33));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[1])/sizeof(FLT), x37 + (x34 * x37));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Pos[2])/sizeof(FLT), x39 + (x38 * x17) + (x34 * x39));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[0])/sizeof(FLT), x53 + (((-1 * x44 * x46) + (x43 * x17)) * x47) + (x53 * x34));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[1])/sizeof(FLT), x64 + (((-1 * x61 * x44) + (x58 * x17)) * x47) + (x64 * x34));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[2])/sizeof(FLT), x71 + (((-1 * x69 * x44) + (x67 * x17)) * x47) + (x71 * x34));
	cnMatrixOptionalSet(Hx, 0, offsetof(SurvivePose, Rot[3])/sizeof(FLT), x77 + (((-1 * x75 * x44) + (x74 * x17)) * x47) + (x77 * x34));
}

// Full version Jacobian of reproject_axis_y wrt [(*lh_p).Pos[0], (*lh_p).Pos[1], (*lh_p).Pos[2], (*lh_p).Rot[0], (*lh_p).Rot[1], (*lh_p).Rot[2], (*lh_p).Rot[3]]

static inline void reproject_axis_y_jac_lh_p_with_hx(CnMat* Hx, CnMat* hx, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc1) {
    if(hx != 0) { 
        hx->data[0] = reproject_axis_y(obj_p, sensor_pt, lh_p, bsc1);
    }
    if(Hx != 0) { 
        reproject_axis_y_jac_lh_p(Hx, obj_p, sensor_pt, lh_p, bsc1);
    }
}
// Jacobian of reproject_axis_y wrt [<cnkalman.codegen.WrapMember object at 0x7fcf8b6623a0>, <cnkalman.codegen.WrapMember object at 0x7fcf8b662f70>, <cnkalman.codegen.WrapMember object at 0x7fcf8b662e50>, <cnkalman.codegen.WrapMember object at 0x7fcf8b662af0>, <cnkalman.codegen.WrapMember object at 0x7fcf8b662d30>, <cnkalman.codegen.WrapMember object at 0x7fcf8b662310>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6621c0>]
static inline void reproject_axis_y_jac_bsc1(CnMat* Hx, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc1) {
	const FLT x0 = (sensor_pt[0] * (*obj_p).Rot[3]) + (-1 * sensor_pt[2] * (*obj_p).Rot[1]) + (sensor_pt[1] * (*obj_p).Rot[0]);
	const FLT x1 = (sensor_pt[1] * (*obj_p).Rot[1]) + (-1 * sensor_pt[0] * (*obj_p).Rot[2]) + (sensor_pt[2] * (*obj_p).Rot[0]);
	const FLT x2 = (2 * ((x1 * (*obj_p).Rot[2]) + (-1 * x0 * (*obj_p).Rot[3]))) + (*obj_p).Pos[0] + sensor_pt[0];
	const FLT x3 = (-1 * sensor_pt[1] * (*obj_p).Rot[3]) + (sensor_pt[0] * (*obj_p).Rot[0]) + (sensor_pt[2] * (*obj_p).Rot[2]);
	const FLT x4 = (2 * ((x3 * (*obj_p).Rot[3]) + (-1 * x1 * (*obj_p).Rot[1]))) + (*obj_p).Pos[1] + sensor_pt[1];
	const FLT x5 = (2 * ((x0 * (*obj_p).Rot[1]) + (-1 * x3 * (*obj_p).Rot[2]))) + sensor_pt[2] + (*obj_p).Pos[2];
	const FLT x6 = (x2 * (*lh_p).Rot[3]) + (-1 * x5 * (*lh_p).Rot[1]) + (x4 * (*lh_p).Rot[0]);
	const FLT x7 = (-1 * x2 * (*lh_p).Rot[2]) + (x5 * (*lh_p).Rot[0]) + (x4 * (*lh_p).Rot[1]);
	const FLT x8 = x2 + (*lh_p).Pos[0] + (2 * ((x7 * (*lh_p).Rot[2]) + (-1 * x6 * (*lh_p).Rot[3])));
	const FLT x9 = (-1 * x4 * (*lh_p).Rot[3]) + (x2 * (*lh_p).Rot[0]) + (x5 * (*lh_p).Rot[2]);
	const FLT x10 = (2 * ((x6 * (*lh_p).Rot[1]) + (-1 * x9 * (*lh_p).Rot[2]))) + x5 + (*lh_p).Pos[2];
	const FLT x11 = -1 * x10;
	const FLT x12 = x4 + (*lh_p).Pos[1] + (2 * ((x9 * (*lh_p).Rot[3]) + (-1 * x7 * (*lh_p).Rot[1])));
	const FLT x13 = (x12 * x12) + (x10 * x10);
	const FLT x14 = x8 * (1. / sqrt(x13));
	const FLT x15 = 1.5707963267949 + (-1 * (*bsc1).phase) + (*bsc1).gibpha + (-1 * asin(x14 * (*bsc1).tilt)) + (-1 * atan2(-1 * x12, x11));
	const FLT x16 = sin(x15) * (*bsc1).gibmag;
	const FLT x17 = x14 * (1. / sqrt(1 + (-1 * (x8 * x8) * (1. / x13) * ((*bsc1).tilt * (*bsc1).tilt))));
	cnSetZero(Hx);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, curve)/sizeof(FLT), atan2(x8, x11) * atan2(x8, x11));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibmag)/sizeof(FLT), -1 * cos(x15));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, gibpha)/sizeof(FLT), x16);
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, phase)/sizeof(FLT), -1 + (-1 * x16));
	cnMatrixOptionalSet(Hx, 0, offsetof(BaseStationCal, tilt)/sizeof(FLT), (-1 * x17 * x16) + (-1 * x17));
}

// Full version Jacobian of reproject_axis_y wrt [<cnkalman.codegen.WrapMember object at 0x7fcf8b6623a0>, <cnkalman.codegen.WrapMember object at 0x7fcf8b662f70>, <cnkalman.codegen.WrapMember object at 0x7fcf8b662e50>, <cnkalman.codegen.WrapMember object at 0x7fcf8b662af0>, <cnkalman.codegen.WrapMember object at 0x7fcf8b662d30>, <cnkalman.codegen.WrapMember object at 0x7fcf8b662310>, <cnkalman.codegen.WrapMember object at 0x7fcf8b6621c0>]

static inline void reproject_axis_y_jac_bsc1_with_hx(CnMat* Hx, CnMat* hx, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose* lh_p, const BaseStationCal* bsc1) {
    if(hx != 0) { 
        hx->data[0] = reproject_axis_y(obj_p, sensor_pt, lh_p, bsc1);
    }
    if(Hx != 0) { 
        reproject_axis_y_jac_bsc1(Hx, obj_p, sensor_pt, lh_p, bsc1);
    }
}
