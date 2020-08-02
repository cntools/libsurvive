#pragma once
#include "common.h"
/** Applying function <function reproject_gen2 at 0x7ffa3c1c7b00> */
static inline void gen_reproject_gen2_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p, const FLT *sensor_pt,
												 const LinmathAxisAnglePose *lh_p, const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = 0.523598775598299 + tilt_0;
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (1));
	const GEN_FLT x3 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x4 = sin(x3);
	const GEN_FLT x5 = x2 * x4;
	const GEN_FLT x6 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x7 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x8 = cos(x3);
	const GEN_FLT x9 = 1 - x8;
	const GEN_FLT x10 = x6 * x7 * x9;
	const GEN_FLT x11 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x12 = sin(x11);
	const GEN_FLT x13 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x14 = x13 * x12;
	const GEN_FLT x15 = cos(x11);
	const GEN_FLT x16 = 1 - x15;
	const GEN_FLT x17 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x18 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x19 = x18 * x17 * x16;
	const GEN_FLT x20 = x12 * x17;
	const GEN_FLT x21 = x13 * x16;
	const GEN_FLT x22 = x21 * x18;
	const GEN_FLT x23 =
		obj_pz + (x14 + x19) * sensor_y + (x15 + pow(x18, 2) * x16) * sensor_z + (-x20 + x22) * sensor_x;
	const GEN_FLT x24 = x12 * x18;
	const GEN_FLT x25 = x21 * x17;
	const GEN_FLT x26 =
		obj_py + (-x14 + x19) * sensor_z + (x15 + pow(x17, 2) * x16) * sensor_y + (x24 + x25) * sensor_x;
	const GEN_FLT x27 = x4 * x6;
	const GEN_FLT x28 = x2 * x9;
	const GEN_FLT x29 = x7 * x28;
	const GEN_FLT x30 =
		obj_px + (x15 + pow(x13, 2) * x16) * sensor_x + (x20 + x22) * sensor_z + (-x24 + x25) * sensor_y;
	const GEN_FLT x31 = lh_py + x23 * (x10 - x5) + x26 * (x8 + pow(x7, 2) * x9) + (x27 + x29) * x30;
	const GEN_FLT x32 = x4 * x7;
	const GEN_FLT x33 = x6 * x28;
	const GEN_FLT x34 = lh_pz + x23 * (x8 + pow(x6, 2) * x9) + x26 * (x10 + x5) + (-x32 + x33) * x30;
	const GEN_FLT x35 = lh_px + x30 * (x8 + pow(x2, 2) * x9) + (-x27 + x29) * x26 + (x32 + x33) * x23;
	const GEN_FLT x36 = pow(x34, 2) + pow(x35, 2);
	const GEN_FLT x37 = x31 / sqrt(x36 + pow(x31, 2));
	const GEN_FLT x38 = asin(x37 / x1);
	const GEN_FLT x39 = 0.0028679863 + x38 * (-8.0108022e-06 - 8.0108022e-06 * x38);
	const GEN_FLT x40 = 5.3685255e-06 + x38 * x39;
	const GEN_FLT x41 = 0.0076069798 + x40 * x38;
	const GEN_FLT x42 = x31 / sqrt(x36);
	const GEN_FLT x43 = tan(x0) * x42;
	const GEN_FLT x44 = atan2(-x34, x35);
	const GEN_FLT x45 = curve_0 + sin(ogeeMag_0 + x44 - asin(x43)) * ogeePhase_0;
	const GEN_FLT x46 = asin(
		x43 + x41 * x45 * pow(x38, 2) /
				  (x1 - sin(x0) * x45 *
							(x38 * (x41 + x38 * (x40 + x38 * (x39 + x38 * (-8.0108022e-06 - 1.60216044e-05 * x38)))) +
							 x41 * x38)));
	const GEN_FLT x47 = 0.523598775598299 - tilt_1;
	const GEN_FLT x48 = cos(x47);
	const GEN_FLT x49 = asin(x37 / x48);
	const GEN_FLT x50 = -x42 * tan(x47);
	const GEN_FLT x51 = curve_1 + sin(ogeeMag_1 + x44 - asin(x50)) * ogeePhase_1;
	const GEN_FLT x52 = 0.0028679863 + x49 * (-8.0108022e-06 - 8.0108022e-06 * x49);
	const GEN_FLT x53 = 5.3685255e-06 + x52 * x49;
	const GEN_FLT x54 = 0.0076069798 + x53 * x49;
	const GEN_FLT x55 =
		x44 -
		asin(x50 +
			 x54 * x51 * pow(x49, 2) /
				 (x48 + x51 * sin(x47) *
							(x49 * (x54 + x49 * (x53 + x49 * (x52 + x49 * (-8.0108022e-06 - 1.60216044e-05 * x49)))) +
							 x54 * x49)));
	*(out++) = -1.5707963267949 - phase_0 + x44 - x46 + sin(gibPhase_0 + x44 - x46) * gibMag_0;
	*(out++) = -1.5707963267949 - phase_1 + x55 + sin(gibPhase_1 + x55) * gibMag_1;
}

// Jacobian of reproject_gen2 wrt [obj_px, obj_py, obj_pz, obj_qi, obj_qj, obj_qk]
static inline void gen_reproject_gen2_jac_obj_p_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
														   const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
														   const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x5 = cos(x1);
	const GEN_FLT x6 = 1 - x5;
	const GEN_FLT x7 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (1));
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = pow(obj_qj, 2);
	const GEN_FLT x13 = pow(obj_qi, 2);
	const GEN_FLT x14 = x11 + x12 + x13;
	const GEN_FLT x15 = sqrt(x14);
	const GEN_FLT x16 = sin(x15);
	const GEN_FLT x17 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x20 = cos(x15);
	const GEN_FLT x21 = 1 - x20;
	const GEN_FLT x22 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = x23 * x19;
	const GEN_FLT x25 = pow(x19, 2);
	const GEN_FLT x26 = x22 * x16;
	const GEN_FLT x27 = x21 * x17;
	const GEN_FLT x28 = x27 * x19;
	const GEN_FLT x29 = obj_pz + (x20 + x25 * x21) * sensor_z + (x18 + x24) * sensor_y + (-x26 + x28) * sensor_x;
	const GEN_FLT x30 = x2 * x4;
	const GEN_FLT x31 = x0 * x8;
	const GEN_FLT x32 = -x30 + x31;
	const GEN_FLT x33 = pow(x22, 2);
	const GEN_FLT x34 = x19 * x16;
	const GEN_FLT x35 = x23 * x17;
	const GEN_FLT x36 = obj_py + (x20 + x33 * x21) * sensor_y + (-x18 + x24) * sensor_z + (x34 + x35) * sensor_x;
	const GEN_FLT x37 = pow(x17, 2);
	const GEN_FLT x38 = obj_px + (x20 + x37 * x21) * sensor_x + (x26 + x28) * sensor_z + (-x34 + x35) * sensor_y;
	const GEN_FLT x39 = x5 + x6 * pow(x7, 2);
	const GEN_FLT x40 = lh_px + x29 * x10 + x32 * x36 + x38 * x39;
	const GEN_FLT x41 = pow(x40, -1);
	const GEN_FLT x42 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x43 = x42 * x27;
	const GEN_FLT x44 = x42 * x16;
	const GEN_FLT x45 = -x44;
	const GEN_FLT x46 = x42 * x23;
	const GEN_FLT x47 = x43 + x46;
	const GEN_FLT x48 = x21 * x19;
	const GEN_FLT x49 = x42 * x48;
	const GEN_FLT x50 = x43 + x49;
	const GEN_FLT x51 = 2 * x43 * sensor_x + (x45 + x47) * sensor_y + (x44 + x50) * sensor_z;
	const GEN_FLT x52 = 1 + x51;
	const GEN_FLT x53 = -x3 + x9;
	const GEN_FLT x54 = x5 + pow(x4, 2) * x6;
	const GEN_FLT x55 = x46 + x49;
	const GEN_FLT x56 = 2 * x49 * sensor_z + (x45 + x50) * sensor_x + (x44 + x55) * sensor_y;
	const GEN_FLT x57 = x54 * x56;
	const GEN_FLT x58 = x2 * x7;
	const GEN_FLT x59 = x0 * x4 * x6;
	const GEN_FLT x60 = x58 + x59;
	const GEN_FLT x61 = 2 * x46 * sensor_y + (x44 + x47) * sensor_x + (x45 + x55) * sensor_z;
	const GEN_FLT x62 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x63 = x2 * x62;
	const GEN_FLT x64 = -x63;
	const GEN_FLT x65 = x6 * x62;
	const GEN_FLT x66 = x4 * x65;
	const GEN_FLT x67 = x7 * x65;
	const GEN_FLT x68 = x66 + x67;
	const GEN_FLT x69 = x0 * x65;
	const GEN_FLT x70 = x66 + x69;
	const GEN_FLT x71 = 2 * x66 * x29 + (x64 + x68) * x38 + (x63 + x70) * x36;
	const GEN_FLT x72 = x71 + x60 * x61;
	const GEN_FLT x73 = x57 + x72 + x53 * x52;
	const GEN_FLT x74 = x61 * x32;
	const GEN_FLT x75 = x67 + x69;
	const GEN_FLT x76 = 2 * x67 * x38 + (x63 + x68) * x29 + (x64 + x75) * x36;
	const GEN_FLT x77 = x76 + x56 * x10;
	const GEN_FLT x78 = x74 + x77 + x52 * x39;
	const GEN_FLT x79 = lh_pz + x53 * x38 + x54 * x29 + x60 * x36;
	const GEN_FLT x80 = pow(x40, 2);
	const GEN_FLT x81 = x79 / x80;
	const GEN_FLT x82 = x80 + pow(x79, 2);
	const GEN_FLT x83 = pow(x82, -1);
	const GEN_FLT x84 = x80 * x83;
	const GEN_FLT x85 = (-x73 * x41 + x81 * x78) * x84;
	const GEN_FLT x86 = -x58 + x59;
	const GEN_FLT x87 = x5 + pow(x0, 2) * x6;
	const GEN_FLT x88 = x30 + x31;
	const GEN_FLT x89 = lh_py + x86 * x29 + x87 * x36 + x88 * x38;
	const GEN_FLT x90 = pow(x89, 2);
	const GEN_FLT x91 = x82 + x90;
	const GEN_FLT x92 = pow(x91, -1.0 / 2.0);
	const GEN_FLT x93 = 0.523598775598299 + tilt_0;
	const GEN_FLT x94 = cos(x93);
	const GEN_FLT x95 = pow(x94, -1);
	const GEN_FLT x96 = x92 * x95;
	const GEN_FLT x97 = asin(x89 * x96);
	const GEN_FLT x98 = 8.0108022e-06 * x97;
	const GEN_FLT x99 = -8.0108022e-06 - x98;
	const GEN_FLT x100 = 0.0028679863 + x99 * x97;
	const GEN_FLT x101 = 5.3685255e-06 + x97 * x100;
	const GEN_FLT x102 = 0.0076069798 + x97 * x101;
	const GEN_FLT x103 = x97 * x102;
	const GEN_FLT x104 = -8.0108022e-06 - 1.60216044e-05 * x97;
	const GEN_FLT x105 = x100 + x97 * x104;
	const GEN_FLT x106 = x101 + x97 * x105;
	const GEN_FLT x107 = x102 + x97 * x106;
	const GEN_FLT x108 = x103 + x97 * x107;
	const GEN_FLT x109 = sin(x93);
	const GEN_FLT x110 = pow(x82, -1.0 / 2.0);
	const GEN_FLT x111 = tan(x93);
	const GEN_FLT x112 = x110 * x111;
	const GEN_FLT x113 = x89 * x112;
	const GEN_FLT x114 = atan2(-x79, x40);
	const GEN_FLT x115 = ogeeMag_0 + x114 - asin(x113);
	const GEN_FLT x116 = curve_0 + sin(x115) * ogeePhase_0;
	const GEN_FLT x117 = x109 * x116;
	const GEN_FLT x118 = x94 - x108 * x117;
	const GEN_FLT x119 = pow(x118, -1);
	const GEN_FLT x120 = pow(x97, 2);
	const GEN_FLT x121 = x116 * x120;
	const GEN_FLT x122 = x119 * x121;
	const GEN_FLT x123 = x113 + x102 * x122;
	const GEN_FLT x124 = pow(1 - pow(x123, 2), -1.0 / 2.0);
	const GEN_FLT x125 = x87 * x61;
	const GEN_FLT x126 = 2 * x69 * x36 + (x64 + x70) * x29 + (x63 + x75) * x38;
	const GEN_FLT x127 = x126 + x86 * x56;
	const GEN_FLT x128 = x125 + x127 + x88 * x52;
	const GEN_FLT x129 = 2 * x89;
	const GEN_FLT x130 = 2 * x40;
	const GEN_FLT x131 = 2 * x79;
	const GEN_FLT x132 = x73 * x131 + x78 * x130;
	const GEN_FLT x133 = x132 + x128 * x129;
	const GEN_FLT x134 = (1.0 / 2.0) * x89;
	const GEN_FLT x135 = x134 / pow(x91, 3.0 / 2.0);
	const GEN_FLT x136 = x95 * x135;
	const GEN_FLT x137 = -x133 * x136 + x96 * x128;
	const GEN_FLT x138 = x90 / x91;
	const GEN_FLT x139 = pow(1 - x138 / pow(x94, 2), -1.0 / 2.0);
	const GEN_FLT x140 = x107 * x139;
	const GEN_FLT x141 = x106 * x139;
	const GEN_FLT x142 = x137 * x139;
	const GEN_FLT x143 = x99 * x142;
	const GEN_FLT x144 = 2.40324066e-05 * x97;
	const GEN_FLT x145 = x100 * x142 + x97 * (x143 - x98 * x142);
	const GEN_FLT x146 = x101 * x139;
	const GEN_FLT x147 = x137 * x146 + x97 * x145;
	const GEN_FLT x148 = x134 / pow(x82, 3.0 / 2.0);
	const GEN_FLT x149 = x132 * x148;
	const GEN_FLT x150 = -x111 * x149 + x112 * x128;
	const GEN_FLT x151 = x83 * x90;
	const GEN_FLT x152 = pow(1 - pow(x111, 2) * x151, -1.0 / 2.0);
	const GEN_FLT x153 = cos(x115) * ogeePhase_0;
	const GEN_FLT x154 = x153 * (x85 - x150 * x152);
	const GEN_FLT x155 = x109 * x108;
	const GEN_FLT x156 = x102 * x121 / pow(x118, 2);
	const GEN_FLT x157 = 2 * x103 * x119 * x116;
	const GEN_FLT x158 = x102 * x119 * x120;
	const GEN_FLT x159 =
		x124 * (x150 + x122 * x147 + x142 * x157 + x154 * x158 -
				x156 * (-x117 * (x102 * x142 + x137 * x140 + x97 * x147 +
								 x97 * (x147 + x137 * x141 +
										x97 * (x145 + x105 * x142 + x97 * (x143 + x104 * x142 - x142 * x144)))) -
						x154 * x155));
	const GEN_FLT x160 = cos(gibPhase_0 + x114 - asin(x123)) * gibMag_0;
	const GEN_FLT x161 = x53 * x51;
	const GEN_FLT x162 = 1 + x61;
	const GEN_FLT x163 = x161 + x57 + x71 + x60 * x162;
	const GEN_FLT x164 = x51 * x39;
	const GEN_FLT x165 = x164 + x77 + x32 * x162;
	const GEN_FLT x166 = (-x41 * x163 + x81 * x165) * x84;
	const GEN_FLT x167 = x88 * x51;
	const GEN_FLT x168 = x127 + x167 + x87 * x162;
	const GEN_FLT x169 = x163 * x131 + x165 * x130;
	const GEN_FLT x170 = x169 + x129 * x168;
	const GEN_FLT x171 = -x170 * x136 + x96 * x168;
	const GEN_FLT x172 = x171 * x139;
	const GEN_FLT x173 = x99 * x172;
	const GEN_FLT x174 = x100 * x172 + x97 * (x173 - x98 * x172);
	const GEN_FLT x175 = x171 * x146 + x97 * x174;
	const GEN_FLT x176 = x169 * x148;
	const GEN_FLT x177 = -x111 * x176 + x112 * x168;
	const GEN_FLT x178 = x166 - x177 * x152;
	const GEN_FLT x179 = x153 * x155;
	const GEN_FLT x180 = x153 * x158;
	const GEN_FLT x181 =
		x124 * (x177 + x122 * x175 -
				x156 * (-x117 * (x102 * x172 + x171 * x140 + x97 * x175 +
								 x97 * (x175 + x171 * x141 +
										x97 * (x174 + x105 * x172 + x97 * (x173 + x104 * x172 - x172 * x144)))) -
						x178 * x179) +
				x172 * x157 + x178 * x180);
	const GEN_FLT x182 = 1 + x56;
	const GEN_FLT x183 = x161 + x72 + x54 * x182;
	const GEN_FLT x184 = x164 + x74 + x76 + x10 * x182;
	const GEN_FLT x185 = (-x41 * x183 + x81 * x184) * x84;
	const GEN_FLT x186 = x125 + x126 + x167 + x86 * x182;
	const GEN_FLT x187 = x183 * x131 + x184 * x130;
	const GEN_FLT x188 = x187 + x129 * x186;
	const GEN_FLT x189 = -x188 * x136 + x96 * x186;
	const GEN_FLT x190 = x189 * x139;
	const GEN_FLT x191 = x99 * x190;
	const GEN_FLT x192 = x100 * x190 + x97 * (x191 - x98 * x190);
	const GEN_FLT x193 = x189 * x146 + x97 * x192;
	const GEN_FLT x194 = x187 * x148;
	const GEN_FLT x195 = -x111 * x194 + x112 * x186;
	const GEN_FLT x196 = x185 - x195 * x152;
	const GEN_FLT x197 =
		x124 * (x195 + x122 * x193 -
				x156 * (-x117 * (x102 * x190 + x189 * x140 + x97 * x193 +
								 x97 * (x193 + x189 * x141 +
										x97 * (x192 + x105 * x190 + x97 * (x191 + x104 * x190 - x190 * x144)))) -
						x179 * x196) +
				x180 * x196 + x190 * x157);
	const GEN_FLT x198 = pow(x15, -1);
	const GEN_FLT x199 = x198 * obj_qi;
	const GEN_FLT x200 = x16 * x199;
	const GEN_FLT x201 = -x200;
	const GEN_FLT x202 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qi, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x203 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qk * obj_qi / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x204 = x16 * x203;
	const GEN_FLT x205 = -x204;
	const GEN_FLT x206 = x20 * x19;
	const GEN_FLT x207 = x206 * x199;
	const GEN_FLT x208 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qj * obj_qi / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x209 = x27 * x208;
	const GEN_FLT x210 = x26 * x17;
	const GEN_FLT x211 = x209 + x210 * x199 + x23 * x202;
	const GEN_FLT x212 = x16 * x208;
	const GEN_FLT x213 = x22 * x20;
	const GEN_FLT x214 = x213 * x199;
	const GEN_FLT x215 = x34 * x17;
	const GEN_FLT x216 = x27 * x203;
	const GEN_FLT x217 = x216 + x215 * x199 + x48 * x202;
	const GEN_FLT x218 = (x201 + 2 * x27 * x202 + x37 * x200) * sensor_x + (x205 - x207 + x211) * sensor_y +
						 (x212 + x214 + x217) * sensor_z;
	const GEN_FLT x219 = x23 * x208;
	const GEN_FLT x220 = x20 * x17;
	const GEN_FLT x221 = x220 * x199;
	const GEN_FLT x222 = x16 * x202;
	const GEN_FLT x223 = x48 * x208;
	const GEN_FLT x224 = x23 * x203;
	const GEN_FLT x225 = x34 * x22;
	const GEN_FLT x226 = x223 + x224 + x225 * x199;
	const GEN_FLT x227 =
		(x204 + x207 + x211) * sensor_x + (x201 + 2 * x219 + x33 * x200) * sensor_y + (-x221 - x222 + x226) * sensor_z;
	const GEN_FLT x228 = -x212;
	const GEN_FLT x229 = x48 * x203;
	const GEN_FLT x230 =
		(x201 + 2 * x229 + x25 * x200) * sensor_z + (-x214 + x217 + x228) * sensor_x + (x221 + x222 + x226) * sensor_y;
	const GEN_FLT x231 = x71 + x53 * x218 + x54 * x230 + x60 * x227;
	const GEN_FLT x232 = x76 + x10 * x230 + x32 * x227 + x39 * x218;
	const GEN_FLT x233 = (-x41 * x231 + x81 * x232) * x84;
	const GEN_FLT x234 = x126 + x86 * x230 + x87 * x227 + x88 * x218;
	const GEN_FLT x235 = x231 * x131 + x232 * x130;
	const GEN_FLT x236 = x235 + x234 * x129;
	const GEN_FLT x237 = -x236 * x136 + x96 * x234;
	const GEN_FLT x238 = x237 * x139;
	const GEN_FLT x239 = x99 * x238;
	const GEN_FLT x240 = x238 * x100 + x97 * (x239 - x98 * x238);
	const GEN_FLT x241 = x238 * x101 + x97 * x240;
	const GEN_FLT x242 = x235 * x148;
	const GEN_FLT x243 = x234 * x112 - x242 * x111;
	const GEN_FLT x244 = x233 - x243 * x152;
	const GEN_FLT x245 =
		x124 * (x243 -
				x156 * (-x117 * (x237 * x140 + x238 * x102 + x97 * x241 +
								 x97 * (x241 + x237 * x141 +
										x97 * (x240 + x238 * x105 + x97 * (x239 + x238 * x104 - x238 * x144)))) -
						x244 * x179) +
				x238 * x157 + x241 * x122 + x244 * x180);
	const GEN_FLT x246 = x198 * obj_qj;
	const GEN_FLT x247 = x206 * x246;
	const GEN_FLT x248 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qk * obj_qj / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x249 = x16 * x248;
	const GEN_FLT x250 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qj, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x251 = x219 + x210 * x246 + x27 * x250;
	const GEN_FLT x252 = x16 * x246;
	const GEN_FLT x253 = -x252;
	const GEN_FLT x254 = x220 * x246;
	const GEN_FLT x255 = x23 * x248;
	const GEN_FLT x256 = x255 + x225 * x246 + x48 * x250;
	const GEN_FLT x257 = (x247 + x249 + x251) * sensor_x + (x253 + 2 * x23 * x250 + x33 * x252) * sensor_y +
						 (x228 - x254 + x256) * sensor_z;
	const GEN_FLT x258 = -x249;
	const GEN_FLT x259 = x213 * x246;
	const GEN_FLT x260 = x16 * x250;
	const GEN_FLT x261 = x27 * x248;
	const GEN_FLT x262 = x223 + x261 + x215 * x246;
	const GEN_FLT x263 =
		(2 * x209 + x253 + x37 * x252) * sensor_x + (-x247 + x251 + x258) * sensor_y + (x259 + x260 + x262) * sensor_z;
	const GEN_FLT x264 = x48 * x248;
	const GEN_FLT x265 =
		(x212 + x254 + x256) * sensor_y + (-x259 - x260 + x262) * sensor_x + (x253 + 2 * x264 + x25 * x252) * sensor_z;
	const GEN_FLT x266 = x71 + x53 * x263 + x54 * x265 + x60 * x257;
	const GEN_FLT x267 = x76 + x10 * x265 + x32 * x257 + x39 * x263;
	const GEN_FLT x268 = (-x41 * x266 + x81 * x267) * x84;
	const GEN_FLT x269 = x126 + x86 * x265 + x87 * x257 + x88 * x263;
	const GEN_FLT x270 = x266 * x131 + x267 * x130;
	const GEN_FLT x271 = x270 + x269 * x129;
	const GEN_FLT x272 = -x271 * x136 + x96 * x269;
	const GEN_FLT x273 = x272 * x139;
	const GEN_FLT x274 = x99 * x273;
	const GEN_FLT x275 = x273 * x100 + x97 * (x274 - x98 * x273);
	const GEN_FLT x276 = x272 * x146 + x97 * x275;
	const GEN_FLT x277 = x270 * x148;
	const GEN_FLT x278 = x269 * x112 - x277 * x111;
	const GEN_FLT x279 = x268 - x278 * x152;
	const GEN_FLT x280 =
		x124 * (x278 -
				x156 * (-x117 * (x272 * x140 + x273 * x102 + x97 * x276 +
								 x97 * (x276 + x272 * x141 +
										x97 * (x275 + x273 * x105 + x97 * (x274 + x273 * x104 - x273 * x144)))) -
						x279 * x179) +
				x273 * x157 + x276 * x122 + x279 * x180);
	const GEN_FLT x281 = x198 * obj_qk;
	const GEN_FLT x282 = x206 * x281;
	const GEN_FLT x283 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qk, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x284 = x16 * x283;
	const GEN_FLT x285 = x224 + x261 + x210 * x281;
	const GEN_FLT x286 = x16 * x281;
	const GEN_FLT x287 = -x286;
	const GEN_FLT x288 = x281 * x220;
	const GEN_FLT x289 = x264 + x23 * x283 + x281 * x225;
	const GEN_FLT x290 =
		(x205 - x288 + x289) * sensor_z + (x282 + x284 + x285) * sensor_x + (2 * x255 + x287 + x33 * x286) * sensor_y;
	const GEN_FLT x291 = x213 * x281;
	const GEN_FLT x292 = x229 + x215 * x281 + x27 * x283;
	const GEN_FLT x293 =
		(2 * x216 + x287 + x37 * x286) * sensor_x + (-x282 - x284 + x285) * sensor_y + (x249 + x291 + x292) * sensor_z;
	const GEN_FLT x294 = (x204 + x288 + x289) * sensor_y + (x287 + x25 * x286 + 2 * x48 * x283) * sensor_z +
						 (x258 - x291 + x292) * sensor_x;
	const GEN_FLT x295 = x71 + x53 * x293 + x54 * x294 + x60 * x290;
	const GEN_FLT x296 = x76 + x10 * x294 + x32 * x290 + x39 * x293;
	const GEN_FLT x297 = (-x41 * x295 + x81 * x296) * x84;
	const GEN_FLT x298 = x126 + x86 * x294 + x87 * x290 + x88 * x293;
	const GEN_FLT x299 = x295 * x131 + x296 * x130;
	const GEN_FLT x300 = x299 + x298 * x129;
	const GEN_FLT x301 = -x300 * x136 + x96 * x298;
	const GEN_FLT x302 = x301 * x139;
	const GEN_FLT x303 = x99 * x302;
	const GEN_FLT x304 = x302 * x100 + x97 * (x303 - x98 * x302);
	const GEN_FLT x305 = x302 * x101 + x97 * x304;
	const GEN_FLT x306 = x299 * x148;
	const GEN_FLT x307 = x298 * x112 - x306 * x111;
	const GEN_FLT x308 = x297 - x307 * x152;
	const GEN_FLT x309 =
		x124 * (x307 -
				x156 * (-x117 * (x301 * x140 + x302 * x102 + x97 * x305 +
								 x97 * (x305 + x301 * x141 +
										x97 * (x304 + x302 * x105 + x97 * (x303 + x302 * x104 - x302 * x144)))) -
						x308 * x179) +
				x302 * x157 + x305 * x122 + x308 * x180);
	const GEN_FLT x310 = 0.523598775598299 - tilt_1;
	const GEN_FLT x311 = cos(x310);
	const GEN_FLT x312 = pow(1 - x138 / pow(x311, 2), -1.0 / 2.0);
	const GEN_FLT x313 = pow(x311, -1);
	const GEN_FLT x314 = x313 * x135;
	const GEN_FLT x315 = x92 * x313;
	const GEN_FLT x316 = (-x314 * x133 + x315 * x128) * x312;
	const GEN_FLT x317 = tan(x310);
	const GEN_FLT x318 = x317 * x110;
	const GEN_FLT x319 = -x89 * x318;
	const GEN_FLT x320 = ogeeMag_1 + x114 - asin(x319);
	const GEN_FLT x321 = curve_1 + sin(x320) * ogeePhase_1;
	const GEN_FLT x322 = asin(x89 * x315);
	const GEN_FLT x323 = 8.0108022e-06 * x322;
	const GEN_FLT x324 = -8.0108022e-06 - x323;
	const GEN_FLT x325 = 0.0028679863 + x322 * x324;
	const GEN_FLT x326 = 5.3685255e-06 + x322 * x325;
	const GEN_FLT x327 = 0.0076069798 + x326 * x322;
	const GEN_FLT x328 = x327 * x322;
	const GEN_FLT x329 = -8.0108022e-06 - 1.60216044e-05 * x322;
	const GEN_FLT x330 = x325 + x329 * x322;
	const GEN_FLT x331 = x326 + x322 * x330;
	const GEN_FLT x332 = x327 + x322 * x331;
	const GEN_FLT x333 = x328 + x322 * x332;
	const GEN_FLT x334 = sin(x310);
	const GEN_FLT x335 = x321 * x334;
	const GEN_FLT x336 = x311 + x335 * x333;
	const GEN_FLT x337 = pow(x336, -1);
	const GEN_FLT x338 = 2 * x337 * x328 * x321;
	const GEN_FLT x339 = x317 * x149 - x318 * x128;
	const GEN_FLT x340 = pow(1 - pow(x317, 2) * x151, -1.0 / 2.0);
	const GEN_FLT x341 = x85 - x339 * x340;
	const GEN_FLT x342 = cos(x320) * ogeePhase_1;
	const GEN_FLT x343 = pow(x322, 2);
	const GEN_FLT x344 = x337 * x327 * x343;
	const GEN_FLT x345 = x344 * x342;
	const GEN_FLT x346 = x334 * x333;
	const GEN_FLT x347 = x346 * x342;
	const GEN_FLT x348 = x324 * x316;
	const GEN_FLT x349 = 2.40324066e-05 * x322;
	const GEN_FLT x350 = x322 * (x348 - x323 * x316) + x325 * x316;
	const GEN_FLT x351 = x326 * x316 + x350 * x322;
	const GEN_FLT x352 = x321 * x343;
	const GEN_FLT x353 = x352 * x327 / pow(x336, 2);
	const GEN_FLT x354 = x352 * x337;
	const GEN_FLT x355 = x319 + x354 * x327;
	const GEN_FLT x356 = pow(1 - pow(x355, 2), -1.0 / 2.0);
	const GEN_FLT x357 =
		x85 -
		x356 * (x339 + x338 * x316 + x345 * x341 + x351 * x354 -
				x353 * (x335 * (x322 * (x351 + x322 * (x350 + x322 * (x348 + x329 * x316 - x349 * x316) + x330 * x316) +
										x331 * x316) +
								x327 * x316 + x332 * x316 + x351 * x322) +
						x347 * x341));
	const GEN_FLT x358 = cos(gibPhase_1 + x114 - asin(x355)) * gibMag_1;
	const GEN_FLT x359 = (-x314 * x170 + x315 * x168) * x312;
	const GEN_FLT x360 = x317 * x176 - x318 * x168;
	const GEN_FLT x361 = x166 - x360 * x340;
	const GEN_FLT x362 = x359 * x324;
	const GEN_FLT x363 = x322 * (x362 - x359 * x323) + x359 * x325;
	const GEN_FLT x364 = x359 * x326 + x363 * x322;
	const GEN_FLT x365 =
		x166 -
		x356 * (x360 -
				x353 * (x335 * (x322 * (x364 + x322 * (x363 + x322 * (x362 + x359 * x329 - x359 * x349) + x359 * x330) +
										x359 * x331) +
								x359 * x327 + x359 * x332 + x364 * x322) +
						x361 * x347) +
				x359 * x338 + x361 * x345 + x364 * x354);
	const GEN_FLT x366 = (-x314 * x188 + x315 * x186) * x312;
	const GEN_FLT x367 = x317 * x194 - x318 * x186;
	const GEN_FLT x368 = x185 - x367 * x340;
	const GEN_FLT x369 = x366 * x324;
	const GEN_FLT x370 = x322 * (x369 - x366 * x323) + x366 * x325;
	const GEN_FLT x371 = x366 * x326 + x370 * x322;
	const GEN_FLT x372 =
		x185 -
		x356 * (x367 -
				x353 * (x335 * (x322 * (x371 + x322 * (x370 + x322 * (x369 + x366 * x329 - x366 * x349) + x366 * x330) +
										x366 * x331) +
								x366 * x327 + x366 * x332 + x371 * x322) +
						x368 * x347) +
				x366 * x338 + x368 * x345 + x371 * x354);
	const GEN_FLT x373 = (x234 * x315 - x236 * x314) * x312;
	const GEN_FLT x374 = -x234 * x318 + x242 * x317;
	const GEN_FLT x375 = x233 - x374 * x340;
	const GEN_FLT x376 = x373 * x324;
	const GEN_FLT x377 = x322 * (x376 - x373 * x323) + x373 * x325;
	const GEN_FLT x378 = x373 * x326 + x377 * x322;
	const GEN_FLT x379 =
		x233 -
		x356 * (x374 -
				x353 * (x335 * (x322 * (x378 + x322 * (x377 + x322 * (x376 + x373 * x329 - x373 * x349) + x373 * x330) +
										x373 * x331) +
								x373 * x327 + x373 * x332 + x378 * x322) +
						x375 * x347) +
				x373 * x338 + x375 * x345 + x378 * x354);
	const GEN_FLT x380 = (x269 * x315 - x271 * x314) * x312;
	const GEN_FLT x381 = -x269 * x318 + x277 * x317;
	const GEN_FLT x382 = x342 * (x268 - x381 * x340);
	const GEN_FLT x383 = x380 * x324;
	const GEN_FLT x384 = x322 * (x383 - x380 * x323) + x380 * x325;
	const GEN_FLT x385 = x380 * x326 + x384 * x322;
	const GEN_FLT x386 =
		x268 -
		x356 * (x381 -
				x353 * (x335 * (x322 * (x385 + x322 * (x384 + x322 * (x383 + x380 * x329 - x380 * x349) + x380 * x330) +
										x380 * x331) +
								x380 * x327 + x380 * x332 + x385 * x322) +
						x382 * x346) +
				x380 * x338 + x382 * x344 + x385 * x354);
	const GEN_FLT x387 = (x298 * x315 - x300 * x314) * x312;
	const GEN_FLT x388 = -x298 * x318 + x306 * x317;
	const GEN_FLT x389 = x297 - x388 * x340;
	const GEN_FLT x390 = x387 * x324;
	const GEN_FLT x391 = x322 * (x390 - x387 * x323) + x387 * x325;
	const GEN_FLT x392 = x387 * x326 + x391 * x322;
	const GEN_FLT x393 =
		x297 -
		x356 * (x388 -
				x353 * (x335 * (x322 * (x392 + x322 * (x391 + x322 * (x390 + x387 * x329 - x387 * x349) + x387 * x330) +
										x387 * x331) +
								x387 * x327 + x387 * x332 + x392 * x322) +
						x389 * x347) +
				x387 * x338 + x389 * x345 + x392 * x354);
	*(out++) = -x159 + x85 - x160 * (x159 - x85);
	*(out++) = x166 - x181 - (-x166 + x181) * x160;
	*(out++) = x185 - x197 - (-x185 + x197) * x160;
	*(out++) = x233 - x245 - (-x233 + x245) * x160;
	*(out++) = x268 - x280 - (-x268 + x280) * x160;
	*(out++) = x297 - x309 - (-x297 + x309) * x160;
	*(out++) = x357 + x358 * x357;
	*(out++) = x365 + x365 * x358;
	*(out++) = x372 + x372 * x358;
	*(out++) = x379 + x379 * x358;
	*(out++) = x386 + x386 * x358;
	*(out++) = x393 + x393 * x358;
}

// Jacobian of reproject_gen2 wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_reproject_gen2_jac_sensor_pt_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
															   const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
															   const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x5 = cos(x1);
	const GEN_FLT x6 = 1 - x5;
	const GEN_FLT x7 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (1));
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = -x3 + x9;
	const GEN_FLT x11 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x12 = cos(x11);
	const GEN_FLT x13 = 1 - x12;
	const GEN_FLT x14 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x15 = x12 + pow(x14, 2) * x13;
	const GEN_FLT x16 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x17 = x13 * x16;
	const GEN_FLT x18 = x14 * x17;
	const GEN_FLT x19 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x20 = x19 * x17;
	const GEN_FLT x21 = sin(x11);
	const GEN_FLT x22 = x21 * x16;
	const GEN_FLT x23 = -x22;
	const GEN_FLT x24 = x18 + x23;
	const GEN_FLT x25 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x26 = x25 * x17;
	const GEN_FLT x27 = x18 + x22;
	const GEN_FLT x28 = 2 * x18 * sensor_x + (x20 + x24) * sensor_y + (x26 + x27) * sensor_z;
	const GEN_FLT x29 = x15 + x28;
	const GEN_FLT x30 = x25 * x21;
	const GEN_FLT x31 = x13 * x19;
	const GEN_FLT x32 = x31 * x14;
	const GEN_FLT x33 = x30 + x32;
	const GEN_FLT x34 = x20 + x26;
	const GEN_FLT x35 = 2 * x20 * sensor_y + (x20 + x27) * sensor_x + (x23 + x34) * sensor_z;
	const GEN_FLT x36 = x33 + x35;
	const GEN_FLT x37 = x2 * x7;
	const GEN_FLT x38 = x0 * x4 * x6;
	const GEN_FLT x39 = x37 + x38;
	const GEN_FLT x40 = x21 * x19;
	const GEN_FLT x41 = x25 * x14 * x13;
	const GEN_FLT x42 = -x40 + x41;
	const GEN_FLT x43 = 2 * x26 * sensor_z + (x24 + x26) * sensor_x + (x22 + x34) * sensor_y;
	const GEN_FLT x44 = x42 + x43;
	const GEN_FLT x45 = x5 + pow(x4, 2) * x6;
	const GEN_FLT x46 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x47 = x6 * x46;
	const GEN_FLT x48 = x7 * x47;
	const GEN_FLT x49 = x4 * x47;
	const GEN_FLT x50 = x2 * x46;
	const GEN_FLT x51 = -x50;
	const GEN_FLT x52 = x49 + x51;
	const GEN_FLT x53 = x40 + x41;
	const GEN_FLT x54 = -x30 + x32;
	const GEN_FLT x55 = obj_px + x15 * sensor_x + x53 * sensor_z + x54 * sensor_y;
	const GEN_FLT x56 = x21 * x14;
	const GEN_FLT x57 = x31 * x25;
	const GEN_FLT x58 = -x56 + x57;
	const GEN_FLT x59 = x12 + x13 * pow(x19, 2);
	const GEN_FLT x60 = obj_py + x33 * sensor_x + x58 * sensor_z + x59 * sensor_y;
	const GEN_FLT x61 = x0 * x47;
	const GEN_FLT x62 = x50 + x61;
	const GEN_FLT x63 = x56 + x57;
	const GEN_FLT x64 = x12 + pow(x25, 2) * x13;
	const GEN_FLT x65 = obj_pz + x42 * sensor_x + x63 * sensor_y + x64 * sensor_z;
	const GEN_FLT x66 = 2 * x65 * x49 + (x48 + x52) * x55 + (x49 + x62) * x60;
	const GEN_FLT x67 = x66 + x29 * x10 + x36 * x39 + x44 * x45;
	const GEN_FLT x68 = x3 + x9;
	const GEN_FLT x69 = x2 * x4;
	const GEN_FLT x70 = x0 * x8;
	const GEN_FLT x71 = -x69 + x70;
	const GEN_FLT x72 = x5 + x6 * pow(x7, 2);
	const GEN_FLT x73 = lh_px + x68 * x65 + x71 * x60 + x72 * x55;
	const GEN_FLT x74 = pow(x73, -1);
	const GEN_FLT x75 = 2 * x55 * x48 + x60 * (x48 + x51 + x61) + x65 * (x48 + x49 + x50);
	const GEN_FLT x76 = x75 + x68 * x44 + x71 * x36 + x72 * x29;
	const GEN_FLT x77 = lh_pz + x55 * x10 + x60 * x39 + x65 * x45;
	const GEN_FLT x78 = pow(x73, 2);
	const GEN_FLT x79 = x77 / x78;
	const GEN_FLT x80 = x78 + pow(x77, 2);
	const GEN_FLT x81 = pow(x80, -1);
	const GEN_FLT x82 = x81 * x78;
	const GEN_FLT x83 = (-x74 * x67 + x79 * x76) * x82;
	const GEN_FLT x84 = -x37 + x38;
	const GEN_FLT x85 = x5 + pow(x0, 2) * x6;
	const GEN_FLT x86 = x69 + x70;
	const GEN_FLT x87 = lh_py + x84 * x65 + x85 * x60 + x86 * x55;
	const GEN_FLT x88 = 0.523598775598299 + tilt_0;
	const GEN_FLT x89 = cos(x88);
	const GEN_FLT x90 = pow(x89, -1);
	const GEN_FLT x91 = pow(x87, 2);
	const GEN_FLT x92 = x80 + x91;
	const GEN_FLT x93 = pow(x92, -1.0 / 2.0);
	const GEN_FLT x94 = x93 * x90;
	const GEN_FLT x95 = asin(x87 * x94);
	const GEN_FLT x96 = -8.0108022e-06 - 1.60216044e-05 * x95;
	const GEN_FLT x97 = 8.0108022e-06 * x95;
	const GEN_FLT x98 = -8.0108022e-06 - x97;
	const GEN_FLT x99 = 0.0028679863 + x98 * x95;
	const GEN_FLT x100 = x99 + x96 * x95;
	const GEN_FLT x101 = 5.3685255e-06 + x99 * x95;
	const GEN_FLT x102 = x101 + x95 * x100;
	const GEN_FLT x103 = x91 / x92;
	const GEN_FLT x104 = pow(1 - x103 / pow(x89, 2), -1.0 / 2.0);
	const GEN_FLT x105 = 2 * x60 * x61 + (x52 + x61) * x65 + (x48 + x62) * x55;
	const GEN_FLT x106 = x105 + x84 * x44 + x85 * x36 + x86 * x29;
	const GEN_FLT x107 = 2 * x87;
	const GEN_FLT x108 = 2 * x73;
	const GEN_FLT x109 = 2 * x77;
	const GEN_FLT x110 = x67 * x109 + x76 * x108;
	const GEN_FLT x111 = x110 + x107 * x106;
	const GEN_FLT x112 = (1.0 / 2.0) * x87;
	const GEN_FLT x113 = x112 / pow(x92, 3.0 / 2.0);
	const GEN_FLT x114 = x90 * x113;
	const GEN_FLT x115 = x104 * (-x111 * x114 + x94 * x106);
	const GEN_FLT x116 = x98 * x115;
	const GEN_FLT x117 = 2.40324066e-05 * x95;
	const GEN_FLT x118 = x95 * (x116 - x97 * x115) + x99 * x115;
	const GEN_FLT x119 = x101 * x115 + x95 * x118;
	const GEN_FLT x120 = 0.0076069798 + x95 * x101;
	const GEN_FLT x121 = x120 + x95 * x102;
	const GEN_FLT x122 = sin(x88);
	const GEN_FLT x123 = tan(x88);
	const GEN_FLT x124 = pow(x80, -1.0 / 2.0);
	const GEN_FLT x125 = x87 * x124;
	const GEN_FLT x126 = x123 * x125;
	const GEN_FLT x127 = atan2(-x77, x73);
	const GEN_FLT x128 = ogeeMag_0 + x127 - asin(x126);
	const GEN_FLT x129 = curve_0 + sin(x128) * ogeePhase_0;
	const GEN_FLT x130 = x122 * x129;
	const GEN_FLT x131 = x106 * x124;
	const GEN_FLT x132 = x112 / pow(x80, 3.0 / 2.0);
	const GEN_FLT x133 = x123 * x132;
	const GEN_FLT x134 = -x110 * x133 + x123 * x131;
	const GEN_FLT x135 = x81 * x91;
	const GEN_FLT x136 = pow(1 - pow(x123, 2) * x135, -1.0 / 2.0);
	const GEN_FLT x137 = x83 - x134 * x136;
	const GEN_FLT x138 = cos(x128) * ogeePhase_0;
	const GEN_FLT x139 = x95 * x120;
	const GEN_FLT x140 = x139 + x95 * x121;
	const GEN_FLT x141 = x122 * x140;
	const GEN_FLT x142 = x138 * x141;
	const GEN_FLT x143 = x89 - x130 * x140;
	const GEN_FLT x144 = pow(x95, 2);
	const GEN_FLT x145 = x129 * x144;
	const GEN_FLT x146 = x120 * x145 / pow(x143, 2);
	const GEN_FLT x147 = pow(x143, -1);
	const GEN_FLT x148 = 2 * x129 * x139 * x147;
	const GEN_FLT x149 = x120 * x144 * x147;
	const GEN_FLT x150 = x138 * x149;
	const GEN_FLT x151 = x145 * x147;
	const GEN_FLT x152 = x126 + x120 * x151;
	const GEN_FLT x153 = pow(1 - pow(x152, 2), -1.0 / 2.0);
	const GEN_FLT x154 =
		x153 * (x134 + x115 * x148 + x119 * x151 + x137 * x150 -
				x146 * (-x130 * (x115 * x120 + x115 * x121 + x95 * x119 +
								 x95 * (x119 + x102 * x115 +
										x95 * (x118 + x100 * x115 + x95 * (x116 - x115 * x117 + x96 * x115)))) -
						x137 * x142));
	const GEN_FLT x155 = cos(gibPhase_0 + x127 - asin(x152)) * gibMag_0;
	const GEN_FLT x156 = x28 + x54;
	const GEN_FLT x157 = x35 + x59;
	const GEN_FLT x158 = x43 + x63;
	const GEN_FLT x159 = x66 + x10 * x156 + x39 * x157 + x45 * x158;
	const GEN_FLT x160 = x75 + x68 * x158 + x71 * x157 + x72 * x156;
	const GEN_FLT x161 = (-x74 * x159 + x79 * x160) * x82;
	const GEN_FLT x162 = x105 + x84 * x158 + x85 * x157 + x86 * x156;
	const GEN_FLT x163 = x108 * x160 + x109 * x159;
	const GEN_FLT x164 = x113 * (x163 + x107 * x162);
	const GEN_FLT x165 = (-x90 * x164 + x94 * x162) * x104;
	const GEN_FLT x166 = x98 * x165;
	const GEN_FLT x167 = x95 * (x166 - x97 * x165) + x99 * x165;
	const GEN_FLT x168 = x101 * x165 + x95 * x167;
	const GEN_FLT x169 = x124 * x162;
	const GEN_FLT x170 = x123 * x169 - x163 * x133;
	const GEN_FLT x171 = x138 * (x161 - x170 * x136);
	const GEN_FLT x172 =
		x153 * (x170 -
				x146 * (-x130 * (x120 * x165 + x121 * x165 + x95 * x168 +
								 x95 * (x168 + x102 * x165 +
										x95 * (x167 + x100 * x165 + x95 * (x166 - x117 * x165 + x96 * x165)))) -
						x171 * x141) +
				x165 * x148 + x168 * x151 + x171 * x149);
	const GEN_FLT x173 = x35 + x58;
	const GEN_FLT x174 = x28 + x53;
	const GEN_FLT x175 = x43 + x64;
	const GEN_FLT x176 = x66 + x10 * x174 + x39 * x173 + x45 * x175;
	const GEN_FLT x177 = x75 + x68 * x175 + x71 * x173 + x72 * x174;
	const GEN_FLT x178 = (-x74 * x176 + x79 * x177) * x82;
	const GEN_FLT x179 = x105 + x84 * x175 + x85 * x173 + x86 * x174;
	const GEN_FLT x180 = x108 * x177 + x109 * x176;
	const GEN_FLT x181 = x180 + x107 * x179;
	const GEN_FLT x182 = x104 * (-x114 * x181 + x94 * x179);
	const GEN_FLT x183 = x98 * x182;
	const GEN_FLT x184 = x95 * (x183 - x97 * x182) + x99 * x182;
	const GEN_FLT x185 = x101 * x182 + x95 * x184;
	const GEN_FLT x186 = x124 * x179;
	const GEN_FLT x187 = x123 * x186 - x180 * x133;
	const GEN_FLT x188 = x178 - x187 * x136;
	const GEN_FLT x189 =
		x153 * (x187 -
				x146 * (-x130 * (x120 * x182 + x121 * x182 + x95 * x185 +
								 x95 * (x185 + x102 * x182 +
										x95 * (x184 + x100 * x182 + x95 * (x183 - x117 * x182 + x96 * x182)))) -
						x188 * x142) +
				x182 * x148 + x185 * x151 + x188 * x150);
	const GEN_FLT x190 = 0.523598775598299 - tilt_1;
	const GEN_FLT x191 = cos(x190);
	const GEN_FLT x192 = pow(x191, -1);
	const GEN_FLT x193 = x93 * x192;
	const GEN_FLT x194 = asin(x87 * x193);
	const GEN_FLT x195 = 8.0108022e-06 * x194;
	const GEN_FLT x196 = -8.0108022e-06 - x195;
	const GEN_FLT x197 = 0.0028679863 + x196 * x194;
	const GEN_FLT x198 = 5.3685255e-06 + x197 * x194;
	const GEN_FLT x199 = 0.0076069798 + x198 * x194;
	const GEN_FLT x200 = x199 * x194;
	const GEN_FLT x201 = -8.0108022e-06 - 1.60216044e-05 * x194;
	const GEN_FLT x202 = x197 + x201 * x194;
	const GEN_FLT x203 = x198 + x202 * x194;
	const GEN_FLT x204 = x199 + x203 * x194;
	const GEN_FLT x205 = x200 + x204 * x194;
	const GEN_FLT x206 = sin(x190);
	const GEN_FLT x207 = tan(x190);
	const GEN_FLT x208 = -x207 * x125;
	const GEN_FLT x209 = ogeeMag_1 + x127 - asin(x208);
	const GEN_FLT x210 = curve_1 + sin(x209) * ogeePhase_1;
	const GEN_FLT x211 = x210 * x206;
	const GEN_FLT x212 = x191 + x211 * x205;
	const GEN_FLT x213 = pow(x212, -1);
	const GEN_FLT x214 = pow(x194, 2);
	const GEN_FLT x215 = x210 * x214;
	const GEN_FLT x216 = x213 * x215;
	const GEN_FLT x217 = x208 + x216 * x199;
	const GEN_FLT x218 = pow(1 - pow(x217, 2), -1.0 / 2.0);
	const GEN_FLT x219 = pow(1 - x103 / pow(x191, 2), -1.0 / 2.0);
	const GEN_FLT x220 = x113 * x192;
	const GEN_FLT x221 = (x106 * x193 - x220 * x111) * x219;
	const GEN_FLT x222 = 2 * x210 * x213 * x200;
	const GEN_FLT x223 = x207 * x132;
	const GEN_FLT x224 = -x207 * x131 + x223 * x110;
	const GEN_FLT x225 = pow(1 - pow(x207, 2) * x135, -1.0 / 2.0);
	const GEN_FLT x226 = cos(x209) * ogeePhase_1;
	const GEN_FLT x227 = x226 * (x83 - x224 * x225);
	const GEN_FLT x228 = x213 * x214 * x199;
	const GEN_FLT x229 = x205 * x206;
	const GEN_FLT x230 = 2.40324066e-05 * x194;
	const GEN_FLT x231 = x221 * x196;
	const GEN_FLT x232 = x194 * (x231 - x221 * x195) + x221 * x197;
	const GEN_FLT x233 = x221 * x198 + x232 * x194;
	const GEN_FLT x234 = x215 * x199 / pow(x212, 2);
	const GEN_FLT x235 =
		x83 -
		x218 * (x224 + x216 * x233 + x221 * x222 + x227 * x228 -
				x234 * (x211 * (x194 * (x233 + x194 * (x232 + x194 * (x231 + x201 * x221 - x230 * x221) + x202 * x221) +
										x203 * x221) +
								x204 * x221 + x221 * x199 + x233 * x194) +
						x227 * x229));
	const GEN_FLT x236 = cos(gibPhase_1 + x127 - asin(x217)) * gibMag_1;
	const GEN_FLT x237 = (x162 * x193 - x164 * x192) * x219;
	const GEN_FLT x238 = -x207 * x169 + x223 * x163;
	const GEN_FLT x239 = x161 - x238 * x225;
	const GEN_FLT x240 = x228 * x226;
	const GEN_FLT x241 = x226 * x229;
	const GEN_FLT x242 = x237 * x196;
	const GEN_FLT x243 = x194 * (x242 - x237 * x195) + x237 * x197;
	const GEN_FLT x244 = x237 * x198 + x243 * x194;
	const GEN_FLT x245 =
		x161 -
		x218 * (x238 + x216 * x244 -
				x234 * (x211 * (x194 * (x244 + x194 * (x243 + x194 * (x242 - x230 * x237 + x237 * x201) + x237 * x202) +
										x237 * x203) +
								x237 * x199 + x237 * x204 + x244 * x194) +
						x239 * x241) +
				x237 * x222 + x239 * x240);
	const GEN_FLT x246 = (x179 * x193 - x220 * x181) * x219;
	const GEN_FLT x247 = -x207 * x186 + x223 * x180;
	const GEN_FLT x248 = x178 - x225 * x247;
	const GEN_FLT x249 = x246 * x196;
	const GEN_FLT x250 = x194 * (x249 - x246 * x195) + x246 * x197;
	const GEN_FLT x251 = x246 * x198 + x250 * x194;
	const GEN_FLT x252 =
		x178 -
		x218 * (x247 + x216 * x251 + x222 * x246 -
				x234 * (x211 * (x194 * (x251 + x194 * (x250 + x194 * (x249 + x201 * x246 - x230 * x246) + x202 * x246) +
										x203 * x246) +
								x204 * x246 + x246 * x199 + x251 * x194) +
						x241 * x248) +
				x240 * x248);
	*(out++) = -x154 + x83 - x155 * (x154 - x83);
	*(out++) = x161 - x172 - (-x161 + x172) * x155;
	*(out++) = x178 - x189 - (-x178 + x189) * x155;
	*(out++) = x235 + x236 * x235;
	*(out++) = x245 + x236 * x245;
	*(out++) = x252 + x236 * x252;
}

// Jacobian of reproject_gen2 wrt [lh_px, lh_py, lh_pz, lh_qi, lh_qj, lh_qk]
static inline void gen_reproject_gen2_jac_lh_p_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
														  const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
														  const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = pow(lh_qk, 2);
	const GEN_FLT x2 = pow(lh_qj, 2);
	const GEN_FLT x3 = pow(lh_qi, 2);
	const GEN_FLT x4 = x1 + x2 + x3;
	const GEN_FLT x5 = sqrt(x4);
	const GEN_FLT x6 = sin(x5);
	const GEN_FLT x7 = x0 * x6;
	const GEN_FLT x8 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x9 = cos(x5);
	const GEN_FLT x10 = 1 - x9;
	const GEN_FLT x11 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (1));
	const GEN_FLT x12 = x11 * x10;
	const GEN_FLT x13 = x8 * x12;
	const GEN_FLT x14 = x13 + x7;
	const GEN_FLT x15 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x16 = sin(x15);
	const GEN_FLT x17 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x20 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x21 = cos(x15);
	const GEN_FLT x22 = 1 - x21;
	const GEN_FLT x23 = x22 * x20;
	const GEN_FLT x24 = x23 * x19;
	const GEN_FLT x25 = x20 * x16;
	const GEN_FLT x26 = x22 * x17;
	const GEN_FLT x27 = x26 * x19;
	const GEN_FLT x28 =
		obj_pz + (x21 + x22 * pow(x19, 2)) * sensor_z + (x18 + x24) * sensor_y + (-x25 + x27) * sensor_x;
	const GEN_FLT x29 = x6 * x8;
	const GEN_FLT x30 = x0 * x10;
	const GEN_FLT x31 = x30 * x11;
	const GEN_FLT x32 = -x29 + x31;
	const GEN_FLT x33 = x19 * x16;
	const GEN_FLT x34 = x23 * x17;
	const GEN_FLT x35 =
		obj_py + (x21 + x22 * pow(x20, 2)) * sensor_y + (-x18 + x24) * sensor_z + (x33 + x34) * sensor_x;
	const GEN_FLT x36 =
		obj_px + (x21 + x22 * pow(x17, 2)) * sensor_x + (x25 + x27) * sensor_z + (-x33 + x34) * sensor_y;
	const GEN_FLT x37 = pow(x11, 2);
	const GEN_FLT x38 = x9 + x37 * x10;
	const GEN_FLT x39 = lh_px + x28 * x14 + x32 * x35 + x36 * x38;
	const GEN_FLT x40 = pow(x39, -1);
	const GEN_FLT x41 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x42 = x6 * x41;
	const GEN_FLT x43 = -x42;
	const GEN_FLT x44 = x41 * x12;
	const GEN_FLT x45 = x8 * x10;
	const GEN_FLT x46 = x41 * x45;
	const GEN_FLT x47 = x44 + x46;
	const GEN_FLT x48 = x41 * x30;
	const GEN_FLT x49 = x42 + x48;
	const GEN_FLT x50 = x6 * x11;
	const GEN_FLT x51 = x8 * x30;
	const GEN_FLT x52 = x50 + x51;
	const GEN_FLT x53 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x54 = x53 * x16;
	const GEN_FLT x55 = x53 * x23;
	const GEN_FLT x56 = x53 * x26;
	const GEN_FLT x57 = x55 + x56;
	const GEN_FLT x58 = -x54;
	const GEN_FLT x59 = x53 * x22 * x19;
	const GEN_FLT x60 = x55 + x59;
	const GEN_FLT x61 = 2 * x55 * sensor_y + (x54 + x57) * sensor_x + (x58 + x60) * sensor_z;
	const GEN_FLT x62 = x56 + x59;
	const GEN_FLT x63 = 2 * x56 * sensor_x + (x57 + x58) * sensor_y + (x54 + x62) * sensor_z;
	const GEN_FLT x64 = x13 - x7;
	const GEN_FLT x65 = pow(x8, 2);
	const GEN_FLT x66 = x9 + x65 * x10;
	const GEN_FLT x67 = 2 * x59 * sensor_z + (x54 + x60) * sensor_y + (x58 + x62) * sensor_x;
	const GEN_FLT x68 = x61 * x52 + x63 * x64 + x67 * x66;
	const GEN_FLT x69 = x68 + 2 * x46 * x28 + (x43 + x47) * x36 + (x46 + x49) * x35;
	const GEN_FLT x70 = -x69 * x40;
	const GEN_FLT x71 = x43 + x48;
	const GEN_FLT x72 = x61 * x32 + x63 * x38 + x67 * x14;
	const GEN_FLT x73 = x72 + 2 * x44 * x36 + (x42 + x47) * x28 + (x44 + x71) * x35;
	const GEN_FLT x74 = 1 + x73;
	const GEN_FLT x75 = lh_pz + x52 * x35 + x64 * x36 + x66 * x28;
	const GEN_FLT x76 = pow(x39, 2);
	const GEN_FLT x77 = x75 / x76;
	const GEN_FLT x78 = x76 + pow(x75, 2);
	const GEN_FLT x79 = pow(x78, -1);
	const GEN_FLT x80 = x79 * x76;
	const GEN_FLT x81 = x80 * (x70 + x74 * x77);
	const GEN_FLT x82 = 0.523598775598299 + tilt_0;
	const GEN_FLT x83 = cos(x82);
	const GEN_FLT x84 = pow(x83, -1);
	const GEN_FLT x85 = -x50 + x51;
	const GEN_FLT x86 = pow(x0, 2);
	const GEN_FLT x87 = x9 + x86 * x10;
	const GEN_FLT x88 = x29 + x31;
	const GEN_FLT x89 = lh_py + x85 * x28 + x87 * x35 + x88 * x36;
	const GEN_FLT x90 = pow(x89, 2);
	const GEN_FLT x91 = x78 + x90;
	const GEN_FLT x92 = pow(x91, -1.0 / 2.0);
	const GEN_FLT x93 = x89 * x92;
	const GEN_FLT x94 = asin(x84 * x93);
	const GEN_FLT x95 = 8.0108022e-06 * x94;
	const GEN_FLT x96 = -8.0108022e-06 - x95;
	const GEN_FLT x97 = 0.0028679863 + x96 * x94;
	const GEN_FLT x98 = 5.3685255e-06 + x97 * x94;
	const GEN_FLT x99 = 0.0076069798 + x98 * x94;
	const GEN_FLT x100 = x99 * x94;
	const GEN_FLT x101 = -8.0108022e-06 - 1.60216044e-05 * x94;
	const GEN_FLT x102 = x97 + x94 * x101;
	const GEN_FLT x103 = x98 + x94 * x102;
	const GEN_FLT x104 = x99 + x94 * x103;
	const GEN_FLT x105 = x100 + x94 * x104;
	const GEN_FLT x106 = sin(x82);
	const GEN_FLT x107 = pow(x78, -1.0 / 2.0);
	const GEN_FLT x108 = tan(x82);
	const GEN_FLT x109 = x108 * x107;
	const GEN_FLT x110 = x89 * x109;
	const GEN_FLT x111 = atan2(-x75, x39);
	const GEN_FLT x112 = ogeeMag_0 + x111 - asin(x110);
	const GEN_FLT x113 = curve_0 + sin(x112) * ogeePhase_0;
	const GEN_FLT x114 = x106 * x113;
	const GEN_FLT x115 = x83 - x105 * x114;
	const GEN_FLT x116 = pow(x115, -1);
	const GEN_FLT x117 = pow(x94, 2);
	const GEN_FLT x118 = x113 * x117;
	const GEN_FLT x119 = x118 * x116;
	const GEN_FLT x120 = x110 + x99 * x119;
	const GEN_FLT x121 = pow(1 - pow(x120, 2), -1.0 / 2.0);
	const GEN_FLT x122 = x90 / x91;
	const GEN_FLT x123 = pow(1 - x122 / pow(x83, 2), -1.0 / 2.0);
	const GEN_FLT x124 = x85 * x67 + x87 * x61 + x88 * x63;
	const GEN_FLT x125 = x124 + 2 * x48 * x35 + (x44 + x49) * x36 + (x46 + x71) * x28;
	const GEN_FLT x126 = 2 * x89;
	const GEN_FLT x127 = x126 * x125;
	const GEN_FLT x128 = 2 * x39;
	const GEN_FLT x129 = 2 * x75;
	const GEN_FLT x130 = x69 * x129;
	const GEN_FLT x131 = x130 + x74 * x128;
	const GEN_FLT x132 = x127 + x131;
	const GEN_FLT x133 = (1.0 / 2.0) * x89;
	const GEN_FLT x134 = x133 / pow(x91, 3.0 / 2.0);
	const GEN_FLT x135 = x84 * x134;
	const GEN_FLT x136 = x84 * x92;
	const GEN_FLT x137 = x125 * x136;
	const GEN_FLT x138 = x123 * (x137 - x132 * x135);
	const GEN_FLT x139 = 2.40324066e-05 * x94;
	const GEN_FLT x140 = x96 * x138;
	const GEN_FLT x141 = x94 * (x140 - x95 * x138) + x97 * x138;
	const GEN_FLT x142 = x94 * x141 + x98 * x138;
	const GEN_FLT x143 = x109 * x125;
	const GEN_FLT x144 = x133 / pow(x78, 3.0 / 2.0);
	const GEN_FLT x145 = x131 * x144;
	const GEN_FLT x146 = x143 - x108 * x145;
	const GEN_FLT x147 = x79 * x90;
	const GEN_FLT x148 = pow(1 - pow(x108, 2) * x147, -1.0 / 2.0);
	const GEN_FLT x149 = x81 - x146 * x148;
	const GEN_FLT x150 = cos(x112) * ogeePhase_0;
	const GEN_FLT x151 = x105 * x106;
	const GEN_FLT x152 = x150 * x151;
	const GEN_FLT x153 = x99 * x118 / pow(x115, 2);
	const GEN_FLT x154 = 2 * x100 * x113 * x116;
	const GEN_FLT x155 = x99 * x116 * x117;
	const GEN_FLT x156 = x150 * x155;
	const GEN_FLT x157 =
		x121 * (x146 + x119 * x142 + x138 * x154 + x149 * x156 -
				x153 * (-x114 * (x104 * x138 + x94 * x142 +
								 x94 * (x142 + x103 * x138 +
										x94 * (x141 + x102 * x138 + x94 * (x140 + x101 * x138 - x138 * x139))) +
								 x99 * x138) -
						x149 * x152));
	const GEN_FLT x158 = cos(gibPhase_0 + x111 - asin(x120)) * gibMag_0;
	const GEN_FLT x159 = x73 * x77;
	const GEN_FLT x160 = x80 * (x159 + x70);
	const GEN_FLT x161 = 1 + x125;
	const GEN_FLT x162 = x73 * x128;
	const GEN_FLT x163 = x130 + x162;
	const GEN_FLT x164 = x134 * (x163 + x126 * x161);
	const GEN_FLT x165 = x123 * (x161 * x136 - x84 * x164);
	const GEN_FLT x166 = x96 * x165;
	const GEN_FLT x167 = x94 * (x166 - x95 * x165) + x97 * x165;
	const GEN_FLT x168 = x94 * x167 + x98 * x165;
	const GEN_FLT x169 = x163 * x144;
	const GEN_FLT x170 = -x108 * x169 + x109 * x161;
	const GEN_FLT x171 = x160 - x170 * x148;
	const GEN_FLT x172 =
		x121 * (x170 + x119 * x168 -
				x153 * (-x114 * (x104 * x165 + x94 * x168 +
								 x94 * (x168 + x103 * x165 +
										x94 * (x167 + x102 * x165 + x94 * (x166 + x101 * x165 - x165 * x139))) +
								 x99 * x165) -
						x171 * x152) +
				x165 * x154 + x171 * x156);
	const GEN_FLT x173 = 1 + x69;
	const GEN_FLT x174 = x80 * (x159 - x40 * x173);
	const GEN_FLT x175 = x162 + x129 * x173;
	const GEN_FLT x176 = x127 + x175;
	const GEN_FLT x177 = x123 * (x137 - x176 * x135);
	const GEN_FLT x178 = x96 * x177;
	const GEN_FLT x179 = x94 * (x178 - x95 * x177) + x97 * x177;
	const GEN_FLT x180 = x94 * x179 + x98 * x177;
	const GEN_FLT x181 = x175 * x144;
	const GEN_FLT x182 = x143 - x108 * x181;
	const GEN_FLT x183 = x174 - x182 * x148;
	const GEN_FLT x184 =
		x121 * (x182 + x119 * x180 -
				x153 * (-x114 * (x104 * x177 + x94 * x180 +
								 x94 * (x180 + x103 * x177 +
										x94 * (x179 + x102 * x177 + x94 * (x178 + x101 * x177 - x177 * x139))) +
								 x99 * x177) -
						x183 * x152) +
				x177 * x154 + x183 * x156);
	const GEN_FLT x185 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qj * lh_qi / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x186 = x6 * x185;
	const GEN_FLT x187 = -x186;
	const GEN_FLT x188 = pow(x5, -1);
	const GEN_FLT x189 = x188 * lh_qi;
	const GEN_FLT x190 = x0 * x9;
	const GEN_FLT x191 = x189 * x190;
	const GEN_FLT x192 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qi, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x193 = x8 * x50;
	const GEN_FLT x194 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qk * lh_qi / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x195 = x12 * x194;
	const GEN_FLT x196 = x195 + x189 * x193 + x45 * x192;
	const GEN_FLT x197 = x6 * x192;
	const GEN_FLT x198 = x9 * x11;
	const GEN_FLT x199 = x189 * x198;
	const GEN_FLT x200 = x45 * x185;
	const GEN_FLT x201 = x30 * x194;
	const GEN_FLT x202 = x0 * x29;
	const GEN_FLT x203 = x200 + x201 + x202 * x189;
	const GEN_FLT x204 = x6 * x189;
	const GEN_FLT x205 = -x204;
	const GEN_FLT x206 = x45 * x194;
	const GEN_FLT x207 =
		x68 + x28 * (x205 + 2 * x206 + x65 * x204) + x35 * (x197 + x199 + x203) + x36 * (x187 - x191 + x196);
	const GEN_FLT x208 = x6 * x194;
	const GEN_FLT x209 = -x208;
	const GEN_FLT x210 = x8 * x9;
	const GEN_FLT x211 = x210 * x189;
	const GEN_FLT x212 = x12 * x185;
	const GEN_FLT x213 = x0 * x50;
	const GEN_FLT x214 = x212 + x213 * x189 + x30 * x192;
	const GEN_FLT x215 =
		x72 + x28 * (x186 + x191 + x196) + x35 * (x209 - x211 + x214) + x36 * (x205 + 2 * x12 * x192 + x37 * x204);
	const GEN_FLT x216 = (-x40 * x207 + x77 * x215) * x80;
	const GEN_FLT x217 = x30 * x185;
	const GEN_FLT x218 =
		x124 + x28 * (-x197 - x199 + x203) + x35 * (x205 + 2 * x217 + x86 * x204) + x36 * (x208 + x211 + x214);
	const GEN_FLT x219 = x207 * x129 + x215 * x128;
	const GEN_FLT x220 = x134 * (x219 + x218 * x126);
	const GEN_FLT x221 = x123 * (x218 * x136 - x84 * x220);
	const GEN_FLT x222 = x96 * x221;
	const GEN_FLT x223 = x94 * (x222 - x95 * x221) + x97 * x221;
	const GEN_FLT x224 = x94 * x223 + x98 * x221;
	const GEN_FLT x225 = x218 * x107;
	const GEN_FLT x226 = x219 * x144;
	const GEN_FLT x227 = x225 * x108 - x226 * x108;
	const GEN_FLT x228 = x150 * (x216 - x227 * x148);
	const GEN_FLT x229 =
		x121 * (x227 -
				x153 * (-x114 * (x221 * x104 + x94 * x224 +
								 x94 * (x224 + x221 * x103 +
										x94 * (x223 + x221 * x102 + x94 * (x222 + x221 * x101 - x221 * x139))) +
								 x99 * x221) -
						x228 * x151) +
				x221 * x154 + x224 * x119 + x228 * x155);
	const GEN_FLT x230 = x188 * lh_qj;
	const GEN_FLT x231 = x230 * x198;
	const GEN_FLT x232 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qk * lh_qj / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x233 = x30 * x232;
	const GEN_FLT x234 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qj, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x235 = x233 + x230 * x202 + x45 * x234;
	const GEN_FLT x236 = x6 * x234;
	const GEN_FLT x237 = x230 * x190;
	const GEN_FLT x238 = x12 * x232;
	const GEN_FLT x239 = x200 + x238 + x230 * x193;
	const GEN_FLT x240 = x6 * x230;
	const GEN_FLT x241 = -x240;
	const GEN_FLT x242 = x45 * x232;
	const GEN_FLT x243 =
		x68 + x28 * (x241 + 2 * x242 + x65 * x240) + x35 * (x186 + x231 + x235) + x36 * (-x236 - x237 + x239);
	const GEN_FLT x244 = x6 * x232;
	const GEN_FLT x245 = -x244;
	const GEN_FLT x246 = x210 * x230;
	const GEN_FLT x247 = x217 + x12 * x234 + x213 * x230;
	const GEN_FLT x248 =
		x72 + x28 * (x236 + x237 + x239) + x35 * (x245 - x246 + x247) + x36 * (2 * x212 + x241 + x37 * x240);
	const GEN_FLT x249 = (-x40 * x243 + x77 * x248) * x80;
	const GEN_FLT x250 =
		x124 + x28 * (x187 - x231 + x235) + x35 * (x241 + 2 * x30 * x234 + x86 * x240) + x36 * (x244 + x246 + x247);
	const GEN_FLT x251 = x243 * x129 + x248 * x128;
	const GEN_FLT x252 = x251 + x250 * x126;
	const GEN_FLT x253 = x92 * x250;
	const GEN_FLT x254 = x123 * (-x252 * x135 + x84 * x253);
	const GEN_FLT x255 = x96 * x254;
	const GEN_FLT x256 = x94 * (x255 - x95 * x254) + x97 * x254;
	const GEN_FLT x257 = x94 * x256 + x98 * x254;
	const GEN_FLT x258 = x250 * x107;
	const GEN_FLT x259 = x251 * x144;
	const GEN_FLT x260 = x258 * x108 - x259 * x108;
	const GEN_FLT x261 = x249 - x260 * x148;
	const GEN_FLT x262 =
		x121 * (x260 -
				x153 * (-x114 * (x254 * x104 + x94 * x257 +
								 x94 * (x257 + x254 * x103 +
										x94 * (x256 + x254 * x102 + x94 * (x255 + x254 * x101 - x254 * x139))) +
								 x99 * x254) -
						x261 * x152) +
				x254 * x154 + x257 * x119 + x261 * x156);
	const GEN_FLT x263 = x188 * lh_qk;
	const GEN_FLT x264 = x263 * x190;
	const GEN_FLT x265 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qk, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x266 = x206 + x12 * x265 + x263 * x193;
	const GEN_FLT x267 = x263 * x198;
	const GEN_FLT x268 = x242 + x202 * x263 + x30 * x265;
	const GEN_FLT x269 = x6 * x263;
	const GEN_FLT x270 = -x269;
	const GEN_FLT x271 =
		x68 + x28 * (x270 + 2 * x45 * x265 + x65 * x269) + x35 * (x208 + x267 + x268) + x36 * (x245 - x264 + x266);
	const GEN_FLT x272 = x6 * x265;
	const GEN_FLT x273 = x210 * x263;
	const GEN_FLT x274 = x201 + x238 + x213 * x263;
	const GEN_FLT x275 =
		x72 + x28 * (x244 + x264 + x266) + x35 * (-x272 - x273 + x274) + x36 * (2 * x195 + x270 + x37 * x269);
	const GEN_FLT x276 = (-x40 * x271 + x77 * x275) * x80;
	const GEN_FLT x277 =
		x124 + x28 * (x209 - x267 + x268) + x35 * (2 * x233 + x270 + x86 * x269) + x36 * (x272 + x273 + x274);
	const GEN_FLT x278 = x271 * x129 + x275 * x128;
	const GEN_FLT x279 = x278 + x277 * x126;
	const GEN_FLT x280 = (x277 * x136 - x279 * x135) * x123;
	const GEN_FLT x281 = x96 * x280;
	const GEN_FLT x282 = x94 * (x281 - x95 * x280) + x97 * x280;
	const GEN_FLT x283 = x94 * x282 + x98 * x280;
	const GEN_FLT x284 = x278 * x144;
	const GEN_FLT x285 = x277 * x109 - x284 * x108;
	const GEN_FLT x286 = x276 - x285 * x148;
	const GEN_FLT x287 =
		x121 * (x285 -
				x153 * (-x114 * (x280 * x104 + x94 * x283 +
								 x94 * (x283 + x280 * x103 +
										x94 * (x282 + x280 * x102 + x94 * (x281 + x280 * x101 - x280 * x139))) +
								 x99 * x280) -
						x286 * x152) +
				x280 * x154 + x283 * x119 + x286 * x156);
	const GEN_FLT x288 = 0.523598775598299 - tilt_1;
	const GEN_FLT x289 = cos(x288);
	const GEN_FLT x290 = pow(x289, -1);
	const GEN_FLT x291 = asin(x93 * x290);
	const GEN_FLT x292 = 8.0108022e-06 * x291;
	const GEN_FLT x293 = -8.0108022e-06 - x292;
	const GEN_FLT x294 = 0.0028679863 + x293 * x291;
	const GEN_FLT x295 = 5.3685255e-06 + x294 * x291;
	const GEN_FLT x296 = 0.0076069798 + x295 * x291;
	const GEN_FLT x297 = x296 * x291;
	const GEN_FLT x298 = -8.0108022e-06 - 1.60216044e-05 * x291;
	const GEN_FLT x299 = x294 + x298 * x291;
	const GEN_FLT x300 = x295 + x299 * x291;
	const GEN_FLT x301 = x296 + x291 * x300;
	const GEN_FLT x302 = x297 + x291 * x301;
	const GEN_FLT x303 = sin(x288);
	const GEN_FLT x304 = tan(x288);
	const GEN_FLT x305 = x304 * x107;
	const GEN_FLT x306 = -x89 * x305;
	const GEN_FLT x307 = ogeeMag_1 + x111 - asin(x306);
	const GEN_FLT x308 = curve_1 + sin(x307) * ogeePhase_1;
	const GEN_FLT x309 = x308 * x303;
	const GEN_FLT x310 = x289 + x309 * x302;
	const GEN_FLT x311 = pow(x310, -1);
	const GEN_FLT x312 = pow(x291, 2);
	const GEN_FLT x313 = x308 * x312;
	const GEN_FLT x314 = x313 * x311;
	const GEN_FLT x315 = x306 + x296 * x314;
	const GEN_FLT x316 = pow(1 - pow(x315, 2), -1.0 / 2.0);
	const GEN_FLT x317 = pow(1 - x122 / pow(x289, 2), -1.0 / 2.0);
	const GEN_FLT x318 = x290 * x134;
	const GEN_FLT x319 = x92 * x290;
	const GEN_FLT x320 = x319 * x125;
	const GEN_FLT x321 = x320 - x318 * x132;
	const GEN_FLT x322 = x321 * x317;
	const GEN_FLT x323 = 2 * x297 * x308 * x311;
	const GEN_FLT x324 = -x305 * x125;
	const GEN_FLT x325 = x324 + x304 * x145;
	const GEN_FLT x326 = pow(1 - pow(x304, 2) * x147, -1.0 / 2.0);
	const GEN_FLT x327 = x81 - x326 * x325;
	const GEN_FLT x328 = cos(x307) * ogeePhase_1;
	const GEN_FLT x329 = x296 * x312 * x311;
	const GEN_FLT x330 = x328 * x329;
	const GEN_FLT x331 = x302 * x303;
	const GEN_FLT x332 = x328 * x331;
	const GEN_FLT x333 = x293 * x322;
	const GEN_FLT x334 = 2.40324066e-05 * x291;
	const GEN_FLT x335 = x294 * x317;
	const GEN_FLT x336 = x291 * (x333 - x292 * x322) + x321 * x335;
	const GEN_FLT x337 = x295 * x317;
	const GEN_FLT x338 = x291 * x336 + x337 * x321;
	const GEN_FLT x339 = x296 * x313 / pow(x310, 2);
	const GEN_FLT x340 =
		x81 -
		x316 * (x325 + x322 * x323 + x327 * x330 + x338 * x314 -
				x339 * (x309 * (x291 * x338 +
								x291 * (x338 + x291 * (x336 + x291 * (x333 + x298 * x322 - x322 * x334) + x299 * x322) +
										x300 * x322) +
								x296 * x322 + x301 * x322) +
						x327 * x332));
	const GEN_FLT x341 = cos(gibPhase_1 + x111 - asin(x315)) * gibMag_1;
	const GEN_FLT x342 = (-x290 * x164 + x319 * x161) * x317;
	const GEN_FLT x343 = x304 * x169 - x305 * x161;
	const GEN_FLT x344 = x160 - x326 * x343;
	const GEN_FLT x345 = x293 * x342;
	const GEN_FLT x346 = x291 * (x345 - x292 * x342) + x294 * x342;
	const GEN_FLT x347 = x291 * x346 + x295 * x342;
	const GEN_FLT x348 =
		x160 -
		x316 * (x343 + x314 * x347 + x323 * x342 + x330 * x344 -
				x339 * (x309 * (x291 * x347 +
								x291 * (x347 + x291 * (x346 + x291 * (x345 + x298 * x342 - x334 * x342) + x299 * x342) +
										x300 * x342) +
								x296 * x342 + x301 * x342) +
						x332 * x344));
	const GEN_FLT x349 = x320 - x318 * x176;
	const GEN_FLT x350 = x349 * x317;
	const GEN_FLT x351 = x324 + x304 * x181;
	const GEN_FLT x352 = x174 - x351 * x326;
	const GEN_FLT x353 = x293 * x350;
	const GEN_FLT x354 = x291 * (x353 - x292 * x350) + x349 * x335;
	const GEN_FLT x355 = x291 * x354 + x337 * x349;
	const GEN_FLT x356 =
		x174 -
		x316 * (x351 -
				x339 * (x309 * (x291 * x355 +
								x291 * (x355 + x291 * (x354 + x291 * (x353 + x298 * x350 - x350 * x334) + x299 * x350) +
										x350 * x300) +
								x296 * x350 + x350 * x301) +
						x352 * x332) +
				x350 * x323 + x352 * x330 + x355 * x314);
	const GEN_FLT x357 = x218 * x319 - x290 * x220;
	const GEN_FLT x358 = x357 * x317;
	const GEN_FLT x359 = -x225 * x304 + x226 * x304;
	const GEN_FLT x360 = x328 * (x216 - x359 * x326);
	const GEN_FLT x361 = x293 * x358;
	const GEN_FLT x362 = x291 * (x361 - x292 * x358) + x294 * x358;
	const GEN_FLT x363 = x291 * x362 + x357 * x337;
	const GEN_FLT x364 =
		x216 -
		x316 * (x359 -
				x339 * (x309 * (x291 * x363 +
								x291 * (x363 + x291 * (x362 + x291 * (x361 + x298 * x358 - x358 * x334) + x299 * x358) +
										x358 * x300) +
								x296 * x358 + x358 * x301) +
						x360 * x331) +
				x358 * x323 + x360 * x329 + x363 * x314);
	const GEN_FLT x365 = -x252 * x318 + x290 * x253;
	const GEN_FLT x366 = x365 * x317;
	const GEN_FLT x367 = -x258 * x304 + x259 * x304;
	const GEN_FLT x368 = x249 - x367 * x326;
	const GEN_FLT x369 = x293 * x366;
	const GEN_FLT x370 = x291 * (x369 - x292 * x366) + x365 * x335;
	const GEN_FLT x371 = x291 * x370 + x365 * x337;
	const GEN_FLT x372 =
		x249 -
		x316 * (x367 -
				x339 * (x309 * (x291 * x371 +
								x291 * (x371 + x291 * (x370 + x291 * (x369 + x298 * x366 - x366 * x334) + x299 * x366) +
										x366 * x300) +
								x296 * x366 + x366 * x301) +
						x368 * x332) +
				x366 * x323 + x368 * x330 + x371 * x314);
	const GEN_FLT x373 = x277 * x319 - x279 * x318;
	const GEN_FLT x374 = x373 * x317;
	const GEN_FLT x375 = -x277 * x305 + x284 * x304;
	const GEN_FLT x376 = x276 - x375 * x326;
	const GEN_FLT x377 = x293 * x374;
	const GEN_FLT x378 = x291 * (x377 - x292 * x374) + x373 * x335;
	const GEN_FLT x379 = x291 * x378 + x373 * x337;
	const GEN_FLT x380 =
		x276 -
		x316 * (x375 -
				x339 * (x309 * (x291 * x379 +
								x291 * (x379 + x291 * (x378 + x291 * (x377 + x298 * x374 - x374 * x334) + x299 * x374) +
										x374 * x300) +
								x296 * x374 + x374 * x301) +
						x376 * x332) +
				x374 * x323 + x376 * x330 + x379 * x314);
	*(out++) = -x157 + x81 - x158 * (x157 - x81);
	*(out++) = x160 - x172 - (-x160 + x172) * x158;
	*(out++) = x174 - x184 - (-x174 + x184) * x158;
	*(out++) = x216 - x229 - (-x216 + x229) * x158;
	*(out++) = x249 - x262 - (-x249 + x262) * x158;
	*(out++) = x276 - x287 - (-x276 + x287) * x158;
	*(out++) = x340 + x340 * x341;
	*(out++) = x348 + x348 * x341;
	*(out++) = x356 + x356 * x341;
	*(out++) = x364 + x364 * x341;
	*(out++) = x372 + x372 * x341;
	*(out++) = x380 + x380 * x341;
}

// Jacobian of reproject_gen2 wrt [phase_0, tilt_0, curve_0, gibPhase_0, gibMag_0, ogeeMag_0, ogeePhase_0, phase_1,
// tilt_1, curve_1, gibPhase_1, gibMag_1, ogeeMag_1, ogeePhase_1]
static inline void gen_reproject_gen2_jac_bsd_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
														 const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
														 const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = cos(x1);
	const GEN_FLT x3 = 1 - x2;
	const GEN_FLT x4 = x2 + pow(x0, 2) * x3;
	const GEN_FLT x5 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x6 = sin(x5);
	const GEN_FLT x7 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1));
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (0));
	const GEN_FLT x10 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x11 = cos(x5);
	const GEN_FLT x12 = 1 - x11;
	const GEN_FLT x13 = x12 * x10;
	const GEN_FLT x14 = x9 * x13;
	const GEN_FLT x15 = x6 * x9;
	const GEN_FLT x16 = x7 * x13;
	const GEN_FLT x17 = obj_pz + (x11 + x12 * pow(x10, 2)) * sensor_z + (-x15 + x16) * sensor_x + (x14 + x8) * sensor_y;
	const GEN_FLT x18 = x6 * x10;
	const GEN_FLT x19 = x7 * x9 * x12;
	const GEN_FLT x20 = obj_py + (x11 + pow(x9, 2) * x12) * sensor_y + (x18 + x19) * sensor_x + (x14 - x8) * sensor_z;
	const GEN_FLT x21 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (1));
	const GEN_FLT x22 = sin(x1);
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (0));
	const GEN_FLT x25 = x0 * x3 * x24;
	const GEN_FLT x26 = x23 + x25;
	const GEN_FLT x27 = obj_px + (x11 + pow(x7, 2) * x12) * sensor_x + (x15 + x16) * sensor_z + (-x18 + x19) * sensor_y;
	const GEN_FLT x28 = x24 * x22;
	const GEN_FLT x29 = x3 * x21;
	const GEN_FLT x30 = x0 * x29;
	const GEN_FLT x31 = -x28 + x30;
	const GEN_FLT x32 = lh_pz + x20 * x26 + x31 * x27 + x4 * x17;
	const GEN_FLT x33 = x28 + x30;
	const GEN_FLT x34 = x0 * x22;
	const GEN_FLT x35 = x24 * x29;
	const GEN_FLT x36 = -x34 + x35;
	const GEN_FLT x37 = x2 + x3 * pow(x21, 2);
	const GEN_FLT x38 = lh_px + x33 * x17 + x36 * x20 + x37 * x27;
	const GEN_FLT x39 = pow(x38, 2);
	const GEN_FLT x40 = x39 + pow(x32, 2);
	const GEN_FLT x41 = pow(x40, -1);
	const GEN_FLT x42 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x43 = x42 * x22;
	const GEN_FLT x44 = -x43;
	const GEN_FLT x45 = x3 * x42;
	const GEN_FLT x46 = x45 * x21;
	const GEN_FLT x47 = x0 * x45;
	const GEN_FLT x48 = x46 + x47;
	const GEN_FLT x49 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x50 = x6 * x49;
	const GEN_FLT x51 = x49 * x12;
	const GEN_FLT x52 = x9 * x51;
	const GEN_FLT x53 = x7 * x51;
	const GEN_FLT x54 = x52 + x53;
	const GEN_FLT x55 = -x50;
	const GEN_FLT x56 = x49 * x13;
	const GEN_FLT x57 = x52 + x56;
	const GEN_FLT x58 = 2 * x52 * sensor_y + (x50 + x54) * sensor_x + (x55 + x57) * sensor_z;
	const GEN_FLT x59 = x45 * x24;
	const GEN_FLT x60 = x43 + x59;
	const GEN_FLT x61 = x53 + x56;
	const GEN_FLT x62 = 2 * x53 * sensor_x + (x54 + x55) * sensor_y + (x50 + x61) * sensor_z;
	const GEN_FLT x63 = 2 * x56 * sensor_z + (x50 + x57) * sensor_y + (x55 + x61) * sensor_x;
	const GEN_FLT x64 = x4 * x63 + 2 * x47 * x17 + x58 * x26 + x62 * x31 + (x44 + x48) * x27 + (x47 + x60) * x20;
	const GEN_FLT x65 = x44 + x59;
	const GEN_FLT x66 = 2 * x46 * x27 + x58 * x36 + x62 * x37 + x63 * x33 + (x43 + x48) * x17 + (x46 + x65) * x20;
	const GEN_FLT x67 = x41 * x39 * (-x64 / x38 + x66 * x32 / x39);
	const GEN_FLT x68 = -x67;
	const GEN_FLT x69 = 0.523598775598299 + tilt_0;
	const GEN_FLT x70 = cos(x69);
	const GEN_FLT x71 = pow(x70, -1);
	const GEN_FLT x72 = -x23 + x25;
	const GEN_FLT x73 = x2 + x3 * pow(x24, 2);
	const GEN_FLT x74 = x34 + x35;
	const GEN_FLT x75 = lh_py + x72 * x17 + x73 * x20 + x74 * x27;
	const GEN_FLT x76 = pow(x75, 2);
	const GEN_FLT x77 = x40 + x76;
	const GEN_FLT x78 = pow(x77, -1.0 / 2.0);
	const GEN_FLT x79 = x78 * x75;
	const GEN_FLT x80 = asin(x71 * x79);
	const GEN_FLT x81 = -8.0108022e-06 - 1.60216044e-05 * x80;
	const GEN_FLT x82 = 8.0108022e-06 * x80;
	const GEN_FLT x83 = -8.0108022e-06 - x82;
	const GEN_FLT x84 = 0.0028679863 + x80 * x83;
	const GEN_FLT x85 = x84 + x80 * x81;
	const GEN_FLT x86 = 5.3685255e-06 + x80 * x84;
	const GEN_FLT x87 = x86 + x80 * x85;
	const GEN_FLT x88 = 2 * x59 * x20 + x72 * x63 + x73 * x58 + x74 * x62 + (x46 + x60) * x27 + (x47 + x65) * x17;
	const GEN_FLT x89 = 2 * x64 * x32 + 2 * x66 * x38;
	const GEN_FLT x90 = (1.0 / 2.0) * x75;
	const GEN_FLT x91 = x90 * (x89 + 2 * x88 * x75) / pow(x77, 3.0 / 2.0);
	const GEN_FLT x92 = x88 * x78;
	const GEN_FLT x93 = -x71 * x91 + x71 * x92;
	const GEN_FLT x94 = pow(x70, -2);
	const GEN_FLT x95 = x76 / x77;
	const GEN_FLT x96 = pow(1 - x95 * x94, -1.0 / 2.0);
	const GEN_FLT x97 = x93 * x96;
	const GEN_FLT x98 = x83 * x97;
	const GEN_FLT x99 = 2.40324066e-05 * x80;
	const GEN_FLT x100 = x80 * (x98 - x82 * x97) + x84 * x97;
	const GEN_FLT x101 = x80 * x100 + x86 * x97;
	const GEN_FLT x102 = 0.0076069798 + x80 * x86;
	const GEN_FLT x103 = x102 + x80 * x87;
	const GEN_FLT x104 = sin(x69);
	const GEN_FLT x105 = tan(x69);
	const GEN_FLT x106 = pow(x40, -1.0 / 2.0);
	const GEN_FLT x107 = x75 * x106;
	const GEN_FLT x108 = x105 * x107;
	const GEN_FLT x109 = atan2(-x32, x38);
	const GEN_FLT x110 = ogeeMag_0 + x109 - asin(x108);
	const GEN_FLT x111 = sin(x110);
	const GEN_FLT x112 = curve_0 + x111 * ogeePhase_0;
	const GEN_FLT x113 = x104 * x112;
	const GEN_FLT x114 =
		-x113 *
		(x80 * x101 + x80 * (x101 + x80 * (x100 + x80 * (x98 + x81 * x97 - x99 * x97) + x85 * x97) + x87 * x97) +
		 x97 * x102 + x97 * x103);
	const GEN_FLT x115 = x88 * x106;
	const GEN_FLT x116 = x89 * x90 / pow(x40, 3.0 / 2.0);
	const GEN_FLT x117 = x105 * x115 - x105 * x116;
	const GEN_FLT x118 = pow(x105, 2);
	const GEN_FLT x119 = x76 * x41;
	const GEN_FLT x120 = pow(1 - x118 * x119, -1.0 / 2.0);
	const GEN_FLT x121 = x67 - x117 * x120;
	const GEN_FLT x122 = cos(x110) * ogeePhase_0;
	const GEN_FLT x123 = x122 * x121;
	const GEN_FLT x124 = x80 * x102;
	const GEN_FLT x125 = x124 + x80 * x103;
	const GEN_FLT x126 = x104 * x125;
	const GEN_FLT x127 = pow(x80, 2);
	const GEN_FLT x128 = x70 - x113 * x125;
	const GEN_FLT x129 = x102 * x112 * x127 / pow(x128, 2);
	const GEN_FLT x130 = pow(x128, -1);
	const GEN_FLT x131 = x127 * x130;
	const GEN_FLT x132 = x102 * x131;
	const GEN_FLT x133 = 2 * x112 * x124 * x130;
	const GEN_FLT x134 = x112 * x131;
	const GEN_FLT x135 = x117 + x101 * x134 + x97 * x133;
	const GEN_FLT x136 = x108 + x112 * x132;
	const GEN_FLT x137 = pow(1 - pow(x136, 2), -1.0 / 2.0);
	const GEN_FLT x138 = x137 * (x135 + x123 * x132 - x129 * (x114 - x123 * x126));
	const GEN_FLT x139 = x138 + x68;
	const GEN_FLT x140 = -gibPhase_0 - x109 + asin(x136);
	const GEN_FLT x141 = cos(x140) * gibMag_0;
	const GEN_FLT x142 = -x138 + x67;
	const GEN_FLT x143 = x142 - x139 * x141;
	const GEN_FLT x144 = x96 * (x93 + x79 * x94 * x104);
	const GEN_FLT x145 = x83 * x144;
	const GEN_FLT x146 = x80 * (x145 - x82 * x144) + x84 * x144;
	const GEN_FLT x147 = x80 * x146 + x86 * x144;
	const GEN_FLT x148 = x117 + x107 * (1 + x118);
	const GEN_FLT x149 = x67 - x120 * x148;
	const GEN_FLT x150 = x122 * x126;
	const GEN_FLT x151 = x122 * x132;
	const GEN_FLT x152 =
		x137 *
		(x148 -
		 x129 *
			 (-x104 -
			  x113 * (x102 * x144 + x103 * x144 + x80 * x147 +
					  x80 * (x147 + x80 * (x146 + x80 * (x145 + x81 * x144 - x99 * x144) + x85 * x144) + x87 * x144)) -
			  x149 * x150 - x70 * x112 * x125) +
		 x133 * x144 + x134 * x147 + x149 * x151);
	const GEN_FLT x153 = 1 + x123;
	const GEN_FLT x154 = x137 * (x135 - x129 * (x114 - x126 * x153) + x132 * x153);
	const GEN_FLT x155 = 1 + x121;
	const GEN_FLT x156 = x137 * (x135 - x129 * (x114 - x150 * x155) + x151 * x155);
	const GEN_FLT x157 = x111 + x123;
	const GEN_FLT x158 = x137 * (x135 - x129 * (x114 - x126 * x157) + x132 * x157);
	const GEN_FLT x159 = 0.523598775598299 - tilt_1;
	const GEN_FLT x160 = tan(x159);
	const GEN_FLT x161 = -x115 * x160 + x116 * x160;
	const GEN_FLT x162 = pow(x160, 2);
	const GEN_FLT x163 = pow(1 - x119 * x162, -1.0 / 2.0);
	const GEN_FLT x164 = x67 - x161 * x163;
	const GEN_FLT x165 = -x107 * x160;
	const GEN_FLT x166 = ogeeMag_1 + x109 - asin(x165);
	const GEN_FLT x167 = cos(x166) * ogeePhase_1;
	const GEN_FLT x168 = x167 * x164;
	const GEN_FLT x169 = cos(x159);
	const GEN_FLT x170 = pow(x169, -1);
	const GEN_FLT x171 = asin(x79 * x170);
	const GEN_FLT x172 = 8.0108022e-06 * x171;
	const GEN_FLT x173 = -8.0108022e-06 - x172;
	const GEN_FLT x174 = 0.0028679863 + x171 * x173;
	const GEN_FLT x175 = 5.3685255e-06 + x171 * x174;
	const GEN_FLT x176 = 0.0076069798 + x171 * x175;
	const GEN_FLT x177 = pow(x171, 2);
	const GEN_FLT x178 = sin(x159);
	const GEN_FLT x179 = sin(x166);
	const GEN_FLT x180 = curve_1 + x179 * ogeePhase_1;
	const GEN_FLT x181 = x171 * x176;
	const GEN_FLT x182 = -8.0108022e-06 - 1.60216044e-05 * x171;
	const GEN_FLT x183 = x174 + x171 * x182;
	const GEN_FLT x184 = x175 + x171 * x183;
	const GEN_FLT x185 = x176 + x171 * x184;
	const GEN_FLT x186 = x181 + x171 * x185;
	const GEN_FLT x187 = x180 * x186;
	const GEN_FLT x188 = x169 + x178 * x187;
	const GEN_FLT x189 = pow(x188, -1);
	const GEN_FLT x190 = x177 * x189;
	const GEN_FLT x191 = x176 * x190;
	const GEN_FLT x192 = x178 * x186;
	const GEN_FLT x193 = pow(x169, -2);
	const GEN_FLT x194 = pow(1 - x95 * x193, -1.0 / 2.0);
	const GEN_FLT x195 = -x91 * x170 + x92 * x170;
	const GEN_FLT x196 = x194 * x195;
	const GEN_FLT x197 = x173 * x196;
	const GEN_FLT x198 = 2.40324066e-05 * x171;
	const GEN_FLT x199 = x171 * (x197 - x172 * x196) + x174 * x196;
	const GEN_FLT x200 = x171 * x199 + x175 * x196;
	const GEN_FLT x201 = x178 * x180;
	const GEN_FLT x202 =
		x201 * (x171 * (x200 + x171 * (x199 + x171 * (x197 + x182 * x196 - x196 * x198) + x183 * x196) + x184 * x196) +
				x176 * x196 + x185 * x196 + x200 * x171);
	const GEN_FLT x203 = x176 * x177 * x180 / pow(x188, 2);
	const GEN_FLT x204 = 2 * x181 * x180 * x189;
	const GEN_FLT x205 = x180 * x190;
	const GEN_FLT x206 = x161 + x200 * x205 + x204 * x196;
	const GEN_FLT x207 = x165 + x205 * x176;
	const GEN_FLT x208 = pow(1 - pow(x207, 2), -1.0 / 2.0);
	const GEN_FLT x209 = x67 - x208 * (x206 + x168 * x191 - x203 * (x202 + x168 * x192));
	const GEN_FLT x210 = gibPhase_1 + x109 - asin(x207);
	const GEN_FLT x211 = cos(x210) * gibMag_1;
	const GEN_FLT x212 = x209 + x211 * x209;
	const GEN_FLT x213 = x194 * (x195 - x79 * x178 * x193);
	const GEN_FLT x214 = x161 + x107 * (1 + x162);
	const GEN_FLT x215 = x67 - x214 * x163;
	const GEN_FLT x216 = x167 * x191;
	const GEN_FLT x217 = x167 * x192;
	const GEN_FLT x218 = x213 * x173;
	const GEN_FLT x219 = x171 * (x218 - x213 * x172) + x213 * x174;
	const GEN_FLT x220 = x213 * x175 + x219 * x171;
	const GEN_FLT x221 =
		x67 -
		x208 * (x214 -
				x203 * (x178 - x169 * x187 +
						x201 * (x171 * (x220 + x171 * (x219 + x171 * (x218 + x213 * x182 - x213 * x198) + x213 * x183) +
										x213 * x184) +
								x213 * x176 + x213 * x185 + x220 * x171) +
						x215 * x217) +
				x205 * x220 + x213 * x204 + x215 * x216);
	const GEN_FLT x222 = 1 + x168;
	const GEN_FLT x223 = x67 - x208 * (x206 - x203 * (x202 + x222 * x192) + x222 * x191);
	const GEN_FLT x224 = 1 + x164;
	const GEN_FLT x225 = x67 - x208 * (x206 - x203 * (x202 + x217 * x224) + x216 * x224);
	const GEN_FLT x226 = x168 + x179;
	const GEN_FLT x227 = x67 - x208 * (x206 - x203 * (x202 + x226 * x192) + x226 * x191);
	*(out++) = -1 + x143;
	*(out++) = -x152 + x67 - x141 * (x152 + x68);
	*(out++) = -x154 + x67 - x141 * (x154 + x68);
	*(out++) = x142 - x141 * (-1 + x139);
	*(out++) = x143 - sin(x140);
	*(out++) = -x156 + x67 - x141 * (x156 + x68);
	*(out++) = -x158 + x67 - x141 * (x158 + x68);
	*(out++) = x143;
	*(out++) = x143;
	*(out++) = x143;
	*(out++) = x143;
	*(out++) = x143;
	*(out++) = x143;
	*(out++) = x143;
	*(out++) = x212;
	*(out++) = x212;
	*(out++) = x212;
	*(out++) = x212;
	*(out++) = x212;
	*(out++) = x212;
	*(out++) = x212;
	*(out++) = -1 + x212;
	*(out++) = x221 + x211 * x221;
	*(out++) = x223 + x211 * x223;
	*(out++) = x209 + x211 * (1 + x209);
	*(out++) = x212 + sin(x210);
	*(out++) = x225 + x211 * x225;
	*(out++) = x227 + x211 * x227;
}

/** Applying function <function reproject_axis_x_gen2 at 0x7ffa3c1c79e0> */
static inline void gen_reproject_axis_x_gen2_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
														const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
														const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = 0.523598775598299 + tilt_0;
	const GEN_FLT x1 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (1));
	const GEN_FLT x2 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x3 = sin(x2);
	const GEN_FLT x4 = x1 * x3;
	const GEN_FLT x5 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x6 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x7 = cos(x2);
	const GEN_FLT x8 = 1 - x7;
	const GEN_FLT x9 = x6 * x5 * x8;
	const GEN_FLT x10 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x11 = sin(x10);
	const GEN_FLT x12 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x13 = x12 * x11;
	const GEN_FLT x14 = cos(x10);
	const GEN_FLT x15 = 1 - x14;
	const GEN_FLT x16 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x17 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x18 = x15 * x17 * x16;
	const GEN_FLT x19 = x11 * x16;
	const GEN_FLT x20 = x15 * x12;
	const GEN_FLT x21 = x20 * x17;
	const GEN_FLT x22 =
		obj_pz + (x13 + x18) * sensor_y + (x14 + x15 * pow(x17, 2)) * sensor_z + (-x19 + x21) * sensor_x;
	const GEN_FLT x23 = x11 * x17;
	const GEN_FLT x24 = x20 * x16;
	const GEN_FLT x25 =
		obj_py + (-x13 + x18) * sensor_z + (x14 + x15 * pow(x16, 2)) * sensor_y + (x23 + x24) * sensor_x;
	const GEN_FLT x26 = x3 * x5;
	const GEN_FLT x27 = x1 * x8;
	const GEN_FLT x28 = x6 * x27;
	const GEN_FLT x29 =
		obj_px + (x14 + x15 * pow(x12, 2)) * sensor_x + (x19 + x21) * sensor_z + (-x23 + x24) * sensor_y;
	const GEN_FLT x30 = lh_py + x25 * (x7 + pow(x6, 2) * x8) + (x26 + x28) * x29 + (-x4 + x9) * x22;
	const GEN_FLT x31 = cos(x0);
	const GEN_FLT x32 = x3 * x6;
	const GEN_FLT x33 = x5 * x27;
	const GEN_FLT x34 = lh_pz + x22 * (x7 + pow(x5, 2) * x8) + (-x32 + x33) * x29 + (x4 + x9) * x25;
	const GEN_FLT x35 = lh_px + x29 * (x7 + pow(x1, 2) * x8) + (-x26 + x28) * x25 + (x32 + x33) * x22;
	const GEN_FLT x36 = pow(x34, 2) + pow(x35, 2);
	const GEN_FLT x37 = asin(x30 / (x31 * sqrt(x36 + pow(x30, 2))));
	const GEN_FLT x38 = 0.0028679863 + x37 * (-8.0108022e-06 - 8.0108022e-06 * x37);
	const GEN_FLT x39 = 5.3685255e-06 + x38 * x37;
	const GEN_FLT x40 = 0.0076069798 + x37 * x39;
	const GEN_FLT x41 = tan(x0) * x30 / sqrt(x36);
	const GEN_FLT x42 = atan2(-x34, x35);
	const GEN_FLT x43 = curve_0 + sin(ogeeMag_0 + x42 - asin(x41)) * ogeePhase_0;
	const GEN_FLT x44 = asin(
		x41 + x40 * x43 * pow(x37, 2) /
				  (x31 - sin(x0) * x43 *
							 (x37 * (x40 + x37 * (x39 + x37 * (x38 + x37 * (-8.0108022e-06 - 1.60216044e-05 * x37)))) +
							  x40 * x37)));
	*(out++) = -1.5707963267949 - phase_0 + x42 - x44 + sin(gibPhase_0 + x42 - x44) * gibMag_0;
}

// Jacobian of reproject_axis_x_gen2 wrt [obj_px, obj_py, obj_pz, obj_qi, obj_qj, obj_qk]
static inline void gen_reproject_axis_x_gen2_jac_obj_p_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
																  const FLT *sensor_pt,
																  const LinmathAxisAnglePose *lh_p,
																  const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x5 = cos(x1);
	const GEN_FLT x6 = 1 - x5;
	const GEN_FLT x7 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (1));
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = pow(obj_qj, 2);
	const GEN_FLT x13 = pow(obj_qi, 2);
	const GEN_FLT x14 = x11 + x12 + x13;
	const GEN_FLT x15 = sqrt(x14);
	const GEN_FLT x16 = sin(x15);
	const GEN_FLT x17 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x20 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x21 = cos(x15);
	const GEN_FLT x22 = 1 - x21;
	const GEN_FLT x23 = x22 * x20;
	const GEN_FLT x24 = x23 * x19;
	const GEN_FLT x25 = pow(x20, 2);
	const GEN_FLT x26 = x19 * x16;
	const GEN_FLT x27 = x23 * x17;
	const GEN_FLT x28 = obj_pz + (x21 + x25 * x22) * sensor_z + (x18 + x24) * sensor_y + (-x26 + x27) * sensor_x;
	const GEN_FLT x29 = x2 * x4;
	const GEN_FLT x30 = x0 * x8;
	const GEN_FLT x31 = -x29 + x30;
	const GEN_FLT x32 = pow(x19, 2);
	const GEN_FLT x33 = x20 * x16;
	const GEN_FLT x34 = x22 * x19;
	const GEN_FLT x35 = x34 * x17;
	const GEN_FLT x36 = obj_py + (x21 + x32 * x22) * sensor_y + (-x18 + x24) * sensor_z + (x33 + x35) * sensor_x;
	const GEN_FLT x37 = pow(x17, 2);
	const GEN_FLT x38 = obj_px + (x21 + x37 * x22) * sensor_x + (x26 + x27) * sensor_z + (-x33 + x35) * sensor_y;
	const GEN_FLT x39 = x5 + x6 * pow(x7, 2);
	const GEN_FLT x40 = lh_px + x28 * x10 + x31 * x36 + x38 * x39;
	const GEN_FLT x41 = pow(x40, -1);
	const GEN_FLT x42 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x43 = x22 * x17;
	const GEN_FLT x44 = x42 * x43;
	const GEN_FLT x45 = x42 * x16;
	const GEN_FLT x46 = -x45;
	const GEN_FLT x47 = x42 * x34;
	const GEN_FLT x48 = x44 + x47;
	const GEN_FLT x49 = x42 * x23;
	const GEN_FLT x50 = x44 + x49;
	const GEN_FLT x51 = 2 * x44 * sensor_x + (x46 + x48) * sensor_y + (x45 + x50) * sensor_z;
	const GEN_FLT x52 = 1 + x51;
	const GEN_FLT x53 = -x3 + x9;
	const GEN_FLT x54 = x5 + pow(x4, 2) * x6;
	const GEN_FLT x55 = x47 + x49;
	const GEN_FLT x56 = 2 * x49 * sensor_z + (x46 + x50) * sensor_x + (x45 + x55) * sensor_y;
	const GEN_FLT x57 = x54 * x56;
	const GEN_FLT x58 = x2 * x7;
	const GEN_FLT x59 = x0 * x4 * x6;
	const GEN_FLT x60 = x58 + x59;
	const GEN_FLT x61 = 2 * x47 * sensor_y + (x45 + x48) * sensor_x + (x46 + x55) * sensor_z;
	const GEN_FLT x62 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x63 = x2 * x62;
	const GEN_FLT x64 = -x63;
	const GEN_FLT x65 = x6 * x62;
	const GEN_FLT x66 = x4 * x65;
	const GEN_FLT x67 = x7 * x65;
	const GEN_FLT x68 = x66 + x67;
	const GEN_FLT x69 = x0 * x65;
	const GEN_FLT x70 = x66 + x69;
	const GEN_FLT x71 = 2 * x66 * x28 + (x64 + x68) * x38 + (x63 + x70) * x36;
	const GEN_FLT x72 = x71 + x60 * x61;
	const GEN_FLT x73 = x57 + x72 + x53 * x52;
	const GEN_FLT x74 = x61 * x31;
	const GEN_FLT x75 = x56 * x10;
	const GEN_FLT x76 = x67 + x69;
	const GEN_FLT x77 = 2 * x67 * x38 + (x63 + x68) * x28 + (x64 + x76) * x36;
	const GEN_FLT x78 = x74 + x75 + x77 + x52 * x39;
	const GEN_FLT x79 = pow(x40, 2);
	const GEN_FLT x80 = lh_pz + x53 * x38 + x54 * x28 + x60 * x36;
	const GEN_FLT x81 = x80 / x79;
	const GEN_FLT x82 = x79 + pow(x80, 2);
	const GEN_FLT x83 = pow(x82, -1);
	const GEN_FLT x84 = x83 * x79;
	const GEN_FLT x85 = (-x73 * x41 + x81 * x78) * x84;
	const GEN_FLT x86 = -x58 + x59;
	const GEN_FLT x87 = x5 + pow(x0, 2) * x6;
	const GEN_FLT x88 = x29 + x30;
	const GEN_FLT x89 = lh_py + x86 * x28 + x87 * x36 + x88 * x38;
	const GEN_FLT x90 = 0.523598775598299 + tilt_0;
	const GEN_FLT x91 = cos(x90);
	const GEN_FLT x92 = pow(x91, -1);
	const GEN_FLT x93 = pow(x89, 2);
	const GEN_FLT x94 = x82 + x93;
	const GEN_FLT x95 = x92 / sqrt(x94);
	const GEN_FLT x96 = asin(x89 * x95);
	const GEN_FLT x97 = 8.0108022e-06 * x96;
	const GEN_FLT x98 = -8.0108022e-06 - x97;
	const GEN_FLT x99 = 0.0028679863 + x98 * x96;
	const GEN_FLT x100 = 5.3685255e-06 + x99 * x96;
	const GEN_FLT x101 = 0.0076069798 + x96 * x100;
	const GEN_FLT x102 = x96 * x101;
	const GEN_FLT x103 = -8.0108022e-06 - 1.60216044e-05 * x96;
	const GEN_FLT x104 = x99 + x96 * x103;
	const GEN_FLT x105 = x100 + x96 * x104;
	const GEN_FLT x106 = x101 + x96 * x105;
	const GEN_FLT x107 = x102 + x96 * x106;
	const GEN_FLT x108 = sin(x90);
	const GEN_FLT x109 = tan(x90);
	const GEN_FLT x110 = x109 / sqrt(x82);
	const GEN_FLT x111 = x89 * x110;
	const GEN_FLT x112 = atan2(-x80, x40);
	const GEN_FLT x113 = ogeeMag_0 + x112 - asin(x111);
	const GEN_FLT x114 = curve_0 + sin(x113) * ogeePhase_0;
	const GEN_FLT x115 = x108 * x114;
	const GEN_FLT x116 = x91 - x107 * x115;
	const GEN_FLT x117 = pow(x116, -1);
	const GEN_FLT x118 = pow(x96, 2);
	const GEN_FLT x119 = x118 * x114;
	const GEN_FLT x120 = x119 * x117;
	const GEN_FLT x121 = x111 + x101 * x120;
	const GEN_FLT x122 = pow(1 - pow(x121, 2), -1.0 / 2.0);
	const GEN_FLT x123 = pow(1 - x93 / (pow(x91, 2) * x94), -1.0 / 2.0);
	const GEN_FLT x124 = x87 * x61;
	const GEN_FLT x125 = 2 * x69 * x36 + (x64 + x70) * x28 + (x63 + x76) * x38;
	const GEN_FLT x126 = x125 + x86 * x56;
	const GEN_FLT x127 = x124 + x126 + x88 * x52;
	const GEN_FLT x128 = 2 * x89;
	const GEN_FLT x129 = 2 * x40;
	const GEN_FLT x130 = 2 * x80;
	const GEN_FLT x131 = x73 * x130 + x78 * x129;
	const GEN_FLT x132 = (1.0 / 2.0) * x89;
	const GEN_FLT x133 = x92 * x132 / pow(x94, 3.0 / 2.0);
	const GEN_FLT x134 = -x133 * (x131 + x128 * x127) + x95 * x127;
	const GEN_FLT x135 = x123 * x134;
	const GEN_FLT x136 = x98 * x123;
	const GEN_FLT x137 = x134 * x136;
	const GEN_FLT x138 = 2.40324066e-05 * x96;
	const GEN_FLT x139 = x96 * (x137 - x97 * x135) + x99 * x135;
	const GEN_FLT x140 = x100 * x135 + x96 * x139;
	const GEN_FLT x141 = x109 * x132 / pow(x82, 3.0 / 2.0);
	const GEN_FLT x142 = x110 * x127 - x131 * x141;
	const GEN_FLT x143 = pow(1 - x83 * x93 * pow(x109, 2), -1.0 / 2.0);
	const GEN_FLT x144 = x85 - x142 * x143;
	const GEN_FLT x145 = cos(x113) * ogeePhase_0;
	const GEN_FLT x146 = x108 * x107;
	const GEN_FLT x147 = x146 * x145;
	const GEN_FLT x148 = x101 * x119 / pow(x116, 2);
	const GEN_FLT x149 = 2 * x102 * x114 * x117;
	const GEN_FLT x150 = x101 * x118 * x117;
	const GEN_FLT x151 = x145 * x150;
	const GEN_FLT x152 =
		x122 * (x142 + x120 * x140 + x135 * x149 + x144 * x151 -
				x148 * (-x115 * (x101 * x135 + x106 * x135 + x96 * x140 +
								 x96 * (x140 + x105 * x135 +
										x96 * (x139 + x104 * x135 + x96 * (x137 + x103 * x135 - x138 * x135)))) -
						x144 * x147));
	const GEN_FLT x153 = cos(gibPhase_0 + x112 - asin(x121)) * gibMag_0;
	const GEN_FLT x154 = x53 * x51;
	const GEN_FLT x155 = 1 + x61;
	const GEN_FLT x156 = x154 + x57 + x71 + x60 * x155;
	const GEN_FLT x157 = x77 + x51 * x39;
	const GEN_FLT x158 = x157 + x75 + x31 * x155;
	const GEN_FLT x159 = (-x41 * x156 + x81 * x158) * x84;
	const GEN_FLT x160 = x88 * x51;
	const GEN_FLT x161 = x126 + x160 + x87 * x155;
	const GEN_FLT x162 = x129 * x158 + x130 * x156;
	const GEN_FLT x163 = -x133 * (x162 + x128 * x161) + x95 * x161;
	const GEN_FLT x164 = x123 * x163;
	const GEN_FLT x165 = x163 * x136;
	const GEN_FLT x166 = x103 * x123;
	const GEN_FLT x167 = x96 * (x165 - x97 * x164) + x99 * x164;
	const GEN_FLT x168 = x100 * x164 + x96 * x167;
	const GEN_FLT x169 = x106 * x123;
	const GEN_FLT x170 = x110 * x161 - x162 * x141;
	const GEN_FLT x171 = x159 - x170 * x143;
	const GEN_FLT x172 =
		x122 * (x170 + x120 * x168 -
				x148 * (-x115 * (x101 * x164 + x169 * x163 + x96 * x168 +
								 x96 * (x168 + x105 * x164 +
										x96 * (x167 + x104 * x164 + x96 * (x165 - x164 * x138 + x166 * x163)))) -
						x171 * x147) +
				x164 * x149 + x171 * x151);
	const GEN_FLT x173 = 1 + x56;
	const GEN_FLT x174 = x154 + x72 + x54 * x173;
	const GEN_FLT x175 = x157 + x74 + x10 * x173;
	const GEN_FLT x176 = (-x41 * x174 + x81 * x175) * x84;
	const GEN_FLT x177 = x124 + x125 + x160 + x86 * x173;
	const GEN_FLT x178 = x129 * x175 + x174 * x130;
	const GEN_FLT x179 = -x133 * (x178 + x128 * x177) + x95 * x177;
	const GEN_FLT x180 = x123 * x179;
	const GEN_FLT x181 = x179 * x136;
	const GEN_FLT x182 = x96 * (x181 - x97 * x180) + x99 * x180;
	const GEN_FLT x183 = x100 * x180 + x96 * x182;
	const GEN_FLT x184 = x110 * x177 - x178 * x141;
	const GEN_FLT x185 = x145 * (x176 - x184 * x143);
	const GEN_FLT x186 =
		x122 * (x184 + x120 * x183 -
				x148 * (-x115 * (x101 * x180 + x106 * x180 + x96 * x183 +
								 x96 * (x183 + x105 * x180 +
										x96 * (x182 + x104 * x180 + x96 * (x181 + x166 * x179 - x180 * x138)))) -
						x185 * x146) +
				x180 * x149 + x185 * x150);
	const GEN_FLT x187 = pow(x15, -1);
	const GEN_FLT x188 = x187 * obj_qi;
	const GEN_FLT x189 = x16 * x188;
	const GEN_FLT x190 = -x189;
	const GEN_FLT x191 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qi, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x192 = x20 * x21;
	const GEN_FLT x193 = x188 * x192;
	const GEN_FLT x194 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qk * obj_qi / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x195 = x16 * x194;
	const GEN_FLT x196 = -x195;
	const GEN_FLT x197 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qj * obj_qi / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x198 = x22 * x197;
	const GEN_FLT x199 = x17 * x198;
	const GEN_FLT x200 = x26 * x17;
	const GEN_FLT x201 = x199 + x200 * x188 + x34 * x191;
	const GEN_FLT x202 = x21 * x19;
	const GEN_FLT x203 = x202 * x188;
	const GEN_FLT x204 = x16 * x197;
	const GEN_FLT x205 = x33 * x17;
	const GEN_FLT x206 = x43 * x194;
	const GEN_FLT x207 = x206 + x205 * x188 + x23 * x191;
	const GEN_FLT x208 = (x190 + x37 * x189 + 2 * x43 * x191) * sensor_x + (-x193 + x196 + x201) * sensor_y +
						 (x203 + x204 + x207) * sensor_z;
	const GEN_FLT x209 = x19 * x198;
	const GEN_FLT x210 = x21 * x17;
	const GEN_FLT x211 = x210 * x188;
	const GEN_FLT x212 = x16 * x191;
	const GEN_FLT x213 = x34 * x194;
	const GEN_FLT x214 = x33 * x19;
	const GEN_FLT x215 = x20 * x198;
	const GEN_FLT x216 = x213 + x215 + x214 * x188;
	const GEN_FLT x217 =
		(x193 + x195 + x201) * sensor_x + (x190 + 2 * x209 + x32 * x189) * sensor_y + (-x211 - x212 + x216) * sensor_z;
	const GEN_FLT x218 = -x204;
	const GEN_FLT x219 = x23 * x194;
	const GEN_FLT x220 =
		(x211 + x212 + x216) * sensor_y + (-x203 + x207 + x218) * sensor_x + (x190 + 2 * x219 + x25 * x189) * sensor_z;
	const GEN_FLT x221 = x71 + x53 * x208 + x54 * x220 + x60 * x217;
	const GEN_FLT x222 = x77 + x10 * x220 + x31 * x217 + x39 * x208;
	const GEN_FLT x223 = (-x41 * x221 + x81 * x222) * x84;
	const GEN_FLT x224 = x125 + x86 * x220 + x87 * x217 + x88 * x208;
	const GEN_FLT x225 = x221 * x130 + x222 * x129;
	const GEN_FLT x226 = x123 * (-x133 * (x225 + x224 * x128) + x95 * x224);
	const GEN_FLT x227 = x98 * x226;
	const GEN_FLT x228 = x96 * (x227 - x97 * x226) + x99 * x226;
	const GEN_FLT x229 = x226 * x100 + x96 * x228;
	const GEN_FLT x230 = x224 * x110 - x225 * x141;
	const GEN_FLT x231 = x223 - x230 * x143;
	const GEN_FLT x232 =
		x122 * (x230 -
				x148 * (-x115 * (x226 * x101 + x226 * x106 + x96 * x229 +
								 x96 * (x229 + x226 * x105 +
										x96 * (x228 + x226 * x104 + x96 * (x227 + x226 * x103 - x226 * x138)))) -
						x231 * x147) +
				x226 * x149 + x229 * x120 + x231 * x151);
	const GEN_FLT x233 = x187 * obj_qj;
	const GEN_FLT x234 = x21 * x233;
	const GEN_FLT x235 = x20 * x234;
	const GEN_FLT x236 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qk * obj_qj / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x237 = x16 * x236;
	const GEN_FLT x238 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qj, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x239 = x209 + x233 * x200 + x43 * x238;
	const GEN_FLT x240 = x16 * x233;
	const GEN_FLT x241 = -x240;
	const GEN_FLT x242 = x17 * x234;
	const GEN_FLT x243 = x34 * x236;
	const GEN_FLT x244 = x243 + x214 * x233 + x23 * x238;
	const GEN_FLT x245 = (x218 - x242 + x244) * sensor_z + (x235 + x237 + x239) * sensor_x +
						 (x241 + x32 * x240 + 2 * x34 * x238) * sensor_y;
	const GEN_FLT x246 = -x237;
	const GEN_FLT x247 = x233 * x202;
	const GEN_FLT x248 = x16 * x238;
	const GEN_FLT x249 = x43 * x236;
	const GEN_FLT x250 = x215 + x249 + x233 * x205;
	const GEN_FLT x251 =
		(-x235 + x239 + x246) * sensor_y + (2 * x199 + x241 + x37 * x240) * sensor_x + (x247 + x248 + x250) * sensor_z;
	const GEN_FLT x252 = x23 * x236;
	const GEN_FLT x253 =
		(x204 + x242 + x244) * sensor_y + (-x247 - x248 + x250) * sensor_x + (x241 + 2 * x252 + x25 * x240) * sensor_z;
	const GEN_FLT x254 = x71 + x53 * x251 + x54 * x253 + x60 * x245;
	const GEN_FLT x255 = x77 + x10 * x253 + x31 * x245 + x39 * x251;
	const GEN_FLT x256 = (-x41 * x254 + x81 * x255) * x84;
	const GEN_FLT x257 = x125 + x86 * x253 + x87 * x245 + x88 * x251;
	const GEN_FLT x258 = x254 * x130 + x255 * x129;
	const GEN_FLT x259 = -x133 * (x258 + x257 * x128) + x95 * x257;
	const GEN_FLT x260 = x259 * x123;
	const GEN_FLT x261 = x259 * x136;
	const GEN_FLT x262 = x96 * (x261 - x97 * x260) + x99 * x260;
	const GEN_FLT x263 = x260 * x100 + x96 * x262;
	const GEN_FLT x264 = x257 * x110 - x258 * x141;
	const GEN_FLT x265 = x256 - x264 * x143;
	const GEN_FLT x266 =
		x122 * (x264 -
				x148 * (-x115 * (x259 * x169 + x260 * x101 + x96 * x263 +
								 x96 * (x263 + x260 * x105 +
										x96 * (x262 + x260 * x104 + x96 * (x261 + x259 * x166 - x260 * x138)))) -
						x265 * x147) +
				x260 * x149 + x263 * x120 + x265 * x151);
	const GEN_FLT x267 = x187 * obj_qk;
	const GEN_FLT x268 = x267 * x192;
	const GEN_FLT x269 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qk, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x270 = x16 * x269;
	const GEN_FLT x271 = x213 + x249 + x200 * x267;
	const GEN_FLT x272 = x16 * x267;
	const GEN_FLT x273 = -x272;
	const GEN_FLT x274 = x210 * x267;
	const GEN_FLT x275 = x22 * x269;
	const GEN_FLT x276 = x252 + x19 * x275 + x214 * x267;
	const GEN_FLT x277 =
		(x268 + x270 + x271) * sensor_x + (2 * x243 + x273 + x32 * x272) * sensor_y + (x196 - x274 + x276) * sensor_z;
	const GEN_FLT x278 = x202 * x267;
	const GEN_FLT x279 = x219 + x17 * x275 + x205 * x267;
	const GEN_FLT x280 =
		(2 * x206 + x273 + x37 * x272) * sensor_x + (x237 + x278 + x279) * sensor_z + (-x268 - x270 + x271) * sensor_y;
	const GEN_FLT x281 = (x273 + 2 * x20 * x275 + x25 * x272) * sensor_z + (x195 + x274 + x276) * sensor_y +
						 (x246 - x278 + x279) * sensor_x;
	const GEN_FLT x282 = x71 + x53 * x280 + x54 * x281 + x60 * x277;
	const GEN_FLT x283 = x77 + x10 * x281 + x31 * x277 + x39 * x280;
	const GEN_FLT x284 = (-x41 * x282 + x81 * x283) * x84;
	const GEN_FLT x285 = x125 + x86 * x281 + x87 * x277 + x88 * x280;
	const GEN_FLT x286 = x282 * x130 + x283 * x129;
	const GEN_FLT x287 = -x133 * (x286 + x285 * x128) + x95 * x285;
	const GEN_FLT x288 = x287 * x123;
	const GEN_FLT x289 = x287 * x136;
	const GEN_FLT x290 = x96 * (x289 - x97 * x288) + x99 * x288;
	const GEN_FLT x291 = x288 * x100 + x96 * x290;
	const GEN_FLT x292 = x285 * x110 - x286 * x141;
	const GEN_FLT x293 = x284 - x292 * x143;
	const GEN_FLT x294 =
		x122 * (x292 -
				x148 * (-x115 * (x288 * x101 + x288 * x106 + x96 * x291 +
								 x96 * (x291 + x288 * x105 +
										x96 * (x290 + x288 * x104 + x96 * (x289 + x287 * x166 - x288 * x138)))) -
						x293 * x147) +
				x288 * x149 + x291 * x120 + x293 * x151);
	*(out++) = -x152 + x85 - x153 * (x152 - x85);
	*(out++) = x159 - x172 - (-x159 + x172) * x153;
	*(out++) = x176 - x186 - (-x176 + x186) * x153;
	*(out++) = x223 - x232 - (-x223 + x232) * x153;
	*(out++) = x256 - x266 - (-x256 + x266) * x153;
	*(out++) = x284 - x294 - (-x284 + x294) * x153;
}

// Jacobian of reproject_axis_x_gen2 wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_reproject_axis_x_gen2_jac_sensor_pt_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
																	  const FLT *sensor_pt,
																	  const LinmathAxisAnglePose *lh_p,
																	  const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x5 = cos(x1);
	const GEN_FLT x6 = 1 - x5;
	const GEN_FLT x7 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (1));
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = -x3 + x9;
	const GEN_FLT x11 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x12 = cos(x11);
	const GEN_FLT x13 = 1 - x12;
	const GEN_FLT x14 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x15 = x12 + pow(x14, 2) * x13;
	const GEN_FLT x16 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x17 = x13 * x16;
	const GEN_FLT x18 = x14 * x17;
	const GEN_FLT x19 = sin(x11);
	const GEN_FLT x20 = x19 * x16;
	const GEN_FLT x21 = -x20;
	const GEN_FLT x22 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x23 = x22 * x17;
	const GEN_FLT x24 = x21 + x23;
	const GEN_FLT x25 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x26 = x25 * x17;
	const GEN_FLT x27 = x18 + x26;
	const GEN_FLT x28 = 2 * x18 * sensor_x + (x20 + x27) * sensor_z + (x18 + x24) * sensor_y;
	const GEN_FLT x29 = x15 + x28;
	const GEN_FLT x30 = x25 * x19;
	const GEN_FLT x31 = x14 * x13;
	const GEN_FLT x32 = x31 * x22;
	const GEN_FLT x33 = x30 + x32;
	const GEN_FLT x34 = x20 + x23;
	const GEN_FLT x35 = 2 * x23 * sensor_y + (x24 + x26) * sensor_z + (x18 + x34) * sensor_x;
	const GEN_FLT x36 = x33 + x35;
	const GEN_FLT x37 = x2 * x7;
	const GEN_FLT x38 = x0 * x4 * x6;
	const GEN_FLT x39 = x37 + x38;
	const GEN_FLT x40 = x22 * x19;
	const GEN_FLT x41 = x31 * x25;
	const GEN_FLT x42 = -x40 + x41;
	const GEN_FLT x43 = 2 * x26 * sensor_z + (x21 + x27) * sensor_x + (x26 + x34) * sensor_y;
	const GEN_FLT x44 = x42 + x43;
	const GEN_FLT x45 = x5 + pow(x4, 2) * x6;
	const GEN_FLT x46 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x47 = x6 * x46;
	const GEN_FLT x48 = x7 * x47;
	const GEN_FLT x49 = x4 * x47;
	const GEN_FLT x50 = x2 * x46;
	const GEN_FLT x51 = -x50;
	const GEN_FLT x52 = x49 + x51;
	const GEN_FLT x53 = x40 + x41;
	const GEN_FLT x54 = -x30 + x32;
	const GEN_FLT x55 = obj_px + x15 * sensor_x + x53 * sensor_z + x54 * sensor_y;
	const GEN_FLT x56 = x14 * x19;
	const GEN_FLT x57 = x25 * x22 * x13;
	const GEN_FLT x58 = -x56 + x57;
	const GEN_FLT x59 = x12 + pow(x22, 2) * x13;
	const GEN_FLT x60 = obj_py + x33 * sensor_x + x58 * sensor_z + x59 * sensor_y;
	const GEN_FLT x61 = x0 * x47;
	const GEN_FLT x62 = x50 + x61;
	const GEN_FLT x63 = x56 + x57;
	const GEN_FLT x64 = x12 + pow(x25, 2) * x13;
	const GEN_FLT x65 = obj_pz + x42 * sensor_x + x63 * sensor_y + x64 * sensor_z;
	const GEN_FLT x66 = 2 * x65 * x49 + (x48 + x52) * x55 + (x49 + x62) * x60;
	const GEN_FLT x67 = x66 + x29 * x10 + x36 * x39 + x44 * x45;
	const GEN_FLT x68 = x3 + x9;
	const GEN_FLT x69 = x2 * x4;
	const GEN_FLT x70 = x0 * x8;
	const GEN_FLT x71 = -x69 + x70;
	const GEN_FLT x72 = x5 + x6 * pow(x7, 2);
	const GEN_FLT x73 = lh_px + x68 * x65 + x71 * x60 + x72 * x55;
	const GEN_FLT x74 = pow(x73, -1);
	const GEN_FLT x75 = 2 * x55 * x48 + x60 * (x48 + x51 + x61) + x65 * (x48 + x49 + x50);
	const GEN_FLT x76 = x75 + x68 * x44 + x71 * x36 + x72 * x29;
	const GEN_FLT x77 = lh_pz + x55 * x10 + x60 * x39 + x65 * x45;
	const GEN_FLT x78 = pow(x73, 2);
	const GEN_FLT x79 = x77 / x78;
	const GEN_FLT x80 = x78 + pow(x77, 2);
	const GEN_FLT x81 = pow(x80, -1);
	const GEN_FLT x82 = x81 * x78;
	const GEN_FLT x83 = (-x74 * x67 + x79 * x76) * x82;
	const GEN_FLT x84 = -x37 + x38;
	const GEN_FLT x85 = x5 + pow(x0, 2) * x6;
	const GEN_FLT x86 = x69 + x70;
	const GEN_FLT x87 = lh_py + x84 * x65 + x85 * x60 + x86 * x55;
	const GEN_FLT x88 = 0.523598775598299 + tilt_0;
	const GEN_FLT x89 = cos(x88);
	const GEN_FLT x90 = pow(x89, -1);
	const GEN_FLT x91 = pow(x87, 2);
	const GEN_FLT x92 = x80 + x91;
	const GEN_FLT x93 = x90 / sqrt(x92);
	const GEN_FLT x94 = asin(x87 * x93);
	const GEN_FLT x95 = -8.0108022e-06 - 1.60216044e-05 * x94;
	const GEN_FLT x96 = 8.0108022e-06 * x94;
	const GEN_FLT x97 = -8.0108022e-06 - x96;
	const GEN_FLT x98 = 0.0028679863 + x97 * x94;
	const GEN_FLT x99 = x98 + x95 * x94;
	const GEN_FLT x100 = 5.3685255e-06 + x98 * x94;
	const GEN_FLT x101 = x100 + x99 * x94;
	const GEN_FLT x102 = pow(1 - x91 / (pow(x89, 2) * x92), -1.0 / 2.0);
	const GEN_FLT x103 = 2 * x60 * x61 + (x52 + x61) * x65 + (x48 + x62) * x55;
	const GEN_FLT x104 = x103 + x84 * x44 + x85 * x36 + x86 * x29;
	const GEN_FLT x105 = 2 * x87;
	const GEN_FLT x106 = 2 * x73;
	const GEN_FLT x107 = 2 * x77;
	const GEN_FLT x108 = x67 * x107 + x76 * x106;
	const GEN_FLT x109 = (1.0 / 2.0) * x87;
	const GEN_FLT x110 = x90 * x109 / pow(x92, 3.0 / 2.0);
	const GEN_FLT x111 = x102 * (-x110 * (x108 + x105 * x104) + x93 * x104);
	const GEN_FLT x112 = x97 * x111;
	const GEN_FLT x113 = 2.40324066e-05 * x94;
	const GEN_FLT x114 = x94 * (x112 - x96 * x111) + x98 * x111;
	const GEN_FLT x115 = x100 * x111 + x94 * x114;
	const GEN_FLT x116 = 0.0076069798 + x94 * x100;
	const GEN_FLT x117 = x116 + x94 * x101;
	const GEN_FLT x118 = sin(x88);
	const GEN_FLT x119 = tan(x88);
	const GEN_FLT x120 = x119 / sqrt(x80);
	const GEN_FLT x121 = x87 * x120;
	const GEN_FLT x122 = atan2(-x77, x73);
	const GEN_FLT x123 = ogeeMag_0 + x122 - asin(x121);
	const GEN_FLT x124 = curve_0 + sin(x123) * ogeePhase_0;
	const GEN_FLT x125 = x118 * x124;
	const GEN_FLT x126 = x109 * x119 / pow(x80, 3.0 / 2.0);
	const GEN_FLT x127 = x104 * x120 - x108 * x126;
	const GEN_FLT x128 = pow(1 - x81 * x91 * pow(x119, 2), -1.0 / 2.0);
	const GEN_FLT x129 = x83 - x128 * x127;
	const GEN_FLT x130 = cos(x123) * ogeePhase_0;
	const GEN_FLT x131 = x94 * x116;
	const GEN_FLT x132 = x131 + x94 * x117;
	const GEN_FLT x133 = x118 * x132;
	const GEN_FLT x134 = x130 * x133;
	const GEN_FLT x135 = x89 - x125 * x132;
	const GEN_FLT x136 = pow(x94, 2);
	const GEN_FLT x137 = x116 * x136;
	const GEN_FLT x138 = x124 * x137 / pow(x135, 2);
	const GEN_FLT x139 = pow(x135, -1);
	const GEN_FLT x140 = x124 * x139;
	const GEN_FLT x141 = 2 * x131 * x140;
	const GEN_FLT x142 = x137 * x139;
	const GEN_FLT x143 = x130 * x142;
	const GEN_FLT x144 = x136 * x140;
	const GEN_FLT x145 = x121 + x116 * x144;
	const GEN_FLT x146 = pow(1 - pow(x145, 2), -1.0 / 2.0);
	const GEN_FLT x147 =
		x146 * (x127 + x111 * x141 + x115 * x144 + x129 * x143 -
				x138 * (-x125 * (x111 * x116 + x111 * x117 + x94 * x115 +
								 x94 * (x115 + x101 * x111 +
										x94 * (x114 + x94 * (x112 - x111 * x113 + x95 * x111) + x99 * x111))) -
						x129 * x134));
	const GEN_FLT x148 = cos(gibPhase_0 + x122 - asin(x145)) * gibMag_0;
	const GEN_FLT x149 = x28 + x54;
	const GEN_FLT x150 = x35 + x59;
	const GEN_FLT x151 = x43 + x63;
	const GEN_FLT x152 = x66 + x10 * x149 + x39 * x150 + x45 * x151;
	const GEN_FLT x153 = x75 + x68 * x151 + x71 * x150 + x72 * x149;
	const GEN_FLT x154 = (-x74 * x152 + x79 * x153) * x82;
	const GEN_FLT x155 = x103 + x84 * x151 + x85 * x150 + x86 * x149;
	const GEN_FLT x156 = x106 * x153 + x107 * x152;
	const GEN_FLT x157 = x102 * (-x110 * (x156 + x105 * x155) + x93 * x155);
	const GEN_FLT x158 = x97 * x157;
	const GEN_FLT x159 = x94 * (x158 - x96 * x157) + x98 * x157;
	const GEN_FLT x160 = x100 * x157 + x94 * x159;
	const GEN_FLT x161 = x120 * x155 - x126 * x156;
	const GEN_FLT x162 = x130 * (x154 - x128 * x161);
	const GEN_FLT x163 =
		x146 * (x161 -
				x138 * (-x125 * (x116 * x157 + x117 * x157 + x94 * x160 +
								 x94 * (x160 + x101 * x157 +
										x94 * (x159 + x94 * (x158 - x113 * x157 + x95 * x157) + x99 * x157))) -
						x162 * x133) +
				x141 * x157 + x160 * x144 + x162 * x142);
	const GEN_FLT x164 = x35 + x58;
	const GEN_FLT x165 = x28 + x53;
	const GEN_FLT x166 = x43 + x64;
	const GEN_FLT x167 = x66 + x10 * x165 + x39 * x164 + x45 * x166;
	const GEN_FLT x168 = x75 + x68 * x166 + x71 * x164 + x72 * x165;
	const GEN_FLT x169 = (-x74 * x167 + x79 * x168) * x82;
	const GEN_FLT x170 = x103 + x84 * x166 + x85 * x164 + x86 * x165;
	const GEN_FLT x171 = x106 * x168 + x107 * x167;
	const GEN_FLT x172 = x102 * (-x110 * (x171 + x105 * x170) + x93 * x170);
	const GEN_FLT x173 = x97 * x172;
	const GEN_FLT x174 = x94 * (x173 - x96 * x172) + x98 * x172;
	const GEN_FLT x175 = x100 * x172 + x94 * x174;
	const GEN_FLT x176 = x120 * x170 - x126 * x171;
	const GEN_FLT x177 = x169 - x128 * x176;
	const GEN_FLT x178 =
		x146 * (x176 -
				x138 * (-x125 * (x116 * x172 + x117 * x172 + x94 * x175 +
								 x94 * (x175 + x101 * x172 +
										x94 * (x174 + x94 * (x173 - x113 * x172 + x95 * x172) + x99 * x172))) -
						x177 * x134) +
				x172 * x141 + x175 * x144 + x177 * x143);
	*(out++) = -x147 + x83 - x148 * (x147 - x83);
	*(out++) = x154 - x163 - (-x154 + x163) * x148;
	*(out++) = x169 - x178 - (-x169 + x178) * x148;
}

// Jacobian of reproject_axis_x_gen2 wrt [lh_px, lh_py, lh_pz, lh_qi, lh_qj, lh_qk]
static inline void gen_reproject_axis_x_gen2_jac_lh_p_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
																 const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
																 const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = pow(lh_qk, 2);
	const GEN_FLT x2 = pow(lh_qj, 2);
	const GEN_FLT x3 = pow(lh_qi, 2);
	const GEN_FLT x4 = x1 + x2 + x3;
	const GEN_FLT x5 = sqrt(x4);
	const GEN_FLT x6 = sin(x5);
	const GEN_FLT x7 = x0 * x6;
	const GEN_FLT x8 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x9 = cos(x5);
	const GEN_FLT x10 = 1 - x9;
	const GEN_FLT x11 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (1));
	const GEN_FLT x12 = x11 * x10;
	const GEN_FLT x13 = x8 * x12;
	const GEN_FLT x14 = x13 + x7;
	const GEN_FLT x15 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x16 = sin(x15);
	const GEN_FLT x17 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x20 = cos(x15);
	const GEN_FLT x21 = 1 - x20;
	const GEN_FLT x22 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = x23 * x19;
	const GEN_FLT x25 = x22 * x16;
	const GEN_FLT x26 = x21 * x19;
	const GEN_FLT x27 = x26 * x17;
	const GEN_FLT x28 =
		obj_pz + (x20 + x21 * pow(x19, 2)) * sensor_z + (x18 + x24) * sensor_y + (-x25 + x27) * sensor_x;
	const GEN_FLT x29 = x6 * x8;
	const GEN_FLT x30 = x0 * x12;
	const GEN_FLT x31 = -x29 + x30;
	const GEN_FLT x32 = x19 * x16;
	const GEN_FLT x33 = x23 * x17;
	const GEN_FLT x34 =
		obj_py + (x20 + pow(x22, 2) * x21) * sensor_y + (-x18 + x24) * sensor_z + (x32 + x33) * sensor_x;
	const GEN_FLT x35 =
		obj_px + (x20 + x21 * pow(x17, 2)) * sensor_x + (x25 + x27) * sensor_z + (-x32 + x33) * sensor_y;
	const GEN_FLT x36 = pow(x11, 2);
	const GEN_FLT x37 = x9 + x36 * x10;
	const GEN_FLT x38 = lh_px + x28 * x14 + x31 * x34 + x35 * x37;
	const GEN_FLT x39 = pow(x38, -1);
	const GEN_FLT x40 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x41 = x6 * x40;
	const GEN_FLT x42 = -x41;
	const GEN_FLT x43 = x8 * x10;
	const GEN_FLT x44 = x40 * x43;
	const GEN_FLT x45 = x40 * x12;
	const GEN_FLT x46 = x44 + x45;
	const GEN_FLT x47 = x0 * x10;
	const GEN_FLT x48 = x40 * x47;
	const GEN_FLT x49 = x44 + x48;
	const GEN_FLT x50 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x51 = x50 * x21 * x17;
	const GEN_FLT x52 = x50 * x16;
	const GEN_FLT x53 = -x52;
	const GEN_FLT x54 = x50 * x23;
	const GEN_FLT x55 = x51 + x54;
	const GEN_FLT x56 = x50 * x26;
	const GEN_FLT x57 = x51 + x56;
	const GEN_FLT x58 = 2 * x51 * sensor_x + (x52 + x57) * sensor_z + (x53 + x55) * sensor_y;
	const GEN_FLT x59 = x13 - x7;
	const GEN_FLT x60 = x6 * x11;
	const GEN_FLT x61 = x8 * x47;
	const GEN_FLT x62 = x60 + x61;
	const GEN_FLT x63 = x54 + x56;
	const GEN_FLT x64 = 2 * x54 * sensor_y + (x52 + x55) * sensor_x + (x53 + x63) * sensor_z;
	const GEN_FLT x65 = pow(x8, 2);
	const GEN_FLT x66 = x9 + x65 * x10;
	const GEN_FLT x67 = 2 * x56 * sensor_z + (x53 + x57) * sensor_x + (x52 + x63) * sensor_y;
	const GEN_FLT x68 = x58 * x59 + x64 * x62 + x67 * x66;
	const GEN_FLT x69 = x68 + 2 * x44 * x28 + (x41 + x49) * x34 + (x42 + x46) * x35;
	const GEN_FLT x70 = -x69 * x39;
	const GEN_FLT x71 = x45 + x48;
	const GEN_FLT x72 = x58 * x37 + x64 * x31 + x67 * x14;
	const GEN_FLT x73 = x72 + 2 * x45 * x35 + (x41 + x46) * x28 + (x42 + x71) * x34;
	const GEN_FLT x74 = 1 + x73;
	const GEN_FLT x75 = lh_pz + x59 * x35 + x62 * x34 + x66 * x28;
	const GEN_FLT x76 = pow(x38, 2);
	const GEN_FLT x77 = x75 / x76;
	const GEN_FLT x78 = x76 + pow(x75, 2);
	const GEN_FLT x79 = pow(x78, -1);
	const GEN_FLT x80 = x79 * x76;
	const GEN_FLT x81 = x80 * (x70 + x74 * x77);
	const GEN_FLT x82 = -x60 + x61;
	const GEN_FLT x83 = pow(x0, 2);
	const GEN_FLT x84 = x9 + x83 * x10;
	const GEN_FLT x85 = x29 + x30;
	const GEN_FLT x86 = lh_py + x82 * x28 + x84 * x34 + x85 * x35;
	const GEN_FLT x87 = 0.523598775598299 + tilt_0;
	const GEN_FLT x88 = cos(x87);
	const GEN_FLT x89 = pow(x88, -1);
	const GEN_FLT x90 = pow(x86, 2);
	const GEN_FLT x91 = x78 + x90;
	const GEN_FLT x92 = x89 / sqrt(x91);
	const GEN_FLT x93 = asin(x86 * x92);
	const GEN_FLT x94 = 8.0108022e-06 * x93;
	const GEN_FLT x95 = -8.0108022e-06 - x94;
	const GEN_FLT x96 = 0.0028679863 + x93 * x95;
	const GEN_FLT x97 = 5.3685255e-06 + x93 * x96;
	const GEN_FLT x98 = 0.0076069798 + x93 * x97;
	const GEN_FLT x99 = x93 * x98;
	const GEN_FLT x100 = -8.0108022e-06 - 1.60216044e-05 * x93;
	const GEN_FLT x101 = x96 + x93 * x100;
	const GEN_FLT x102 = x97 + x93 * x101;
	const GEN_FLT x103 = x98 + x93 * x102;
	const GEN_FLT x104 = x99 + x93 * x103;
	const GEN_FLT x105 = sin(x87);
	const GEN_FLT x106 = tan(x87);
	const GEN_FLT x107 = x106 / sqrt(x78);
	const GEN_FLT x108 = x86 * x107;
	const GEN_FLT x109 = atan2(-x75, x38);
	const GEN_FLT x110 = ogeeMag_0 + x109 - asin(x108);
	const GEN_FLT x111 = curve_0 + sin(x110) * ogeePhase_0;
	const GEN_FLT x112 = x105 * x111;
	const GEN_FLT x113 = x88 - x104 * x112;
	const GEN_FLT x114 = pow(x113, -1);
	const GEN_FLT x115 = pow(x93, 2);
	const GEN_FLT x116 = x111 * x115;
	const GEN_FLT x117 = x114 * x116;
	const GEN_FLT x118 = x108 + x98 * x117;
	const GEN_FLT x119 = pow(1 - pow(x118, 2), -1.0 / 2.0);
	const GEN_FLT x120 = pow(1 - x90 / (pow(x88, 2) * x91), -1.0 / 2.0);
	const GEN_FLT x121 = x82 * x67 + x84 * x64 + x85 * x58;
	const GEN_FLT x122 = x121 + 2 * x48 * x34 + (x42 + x49) * x28 + (x41 + x71) * x35;
	const GEN_FLT x123 = 2 * x86;
	const GEN_FLT x124 = x123 * x122;
	const GEN_FLT x125 = 2 * x38;
	const GEN_FLT x126 = 2 * x75;
	const GEN_FLT x127 = x69 * x126;
	const GEN_FLT x128 = x127 + x74 * x125;
	const GEN_FLT x129 = (1.0 / 2.0) * x86;
	const GEN_FLT x130 = x89 * x129 / pow(x91, 3.0 / 2.0);
	const GEN_FLT x131 = x92 * x122;
	const GEN_FLT x132 = x131 - (x124 + x128) * x130;
	const GEN_FLT x133 = x120 * x132;
	const GEN_FLT x134 = 2.40324066e-05 * x93;
	const GEN_FLT x135 = x95 * x133;
	const GEN_FLT x136 = x93 * (x135 - x94 * x133) + x96 * x133;
	const GEN_FLT x137 = x97 * x120;
	const GEN_FLT x138 = x132 * x137 + x93 * x136;
	const GEN_FLT x139 = x98 * x120;
	const GEN_FLT x140 = x107 * x122;
	const GEN_FLT x141 = x106 * x129 / pow(x78, 3.0 / 2.0);
	const GEN_FLT x142 = x140 - x128 * x141;
	const GEN_FLT x143 = pow(1 - x79 * x90 * pow(x106, 2), -1.0 / 2.0);
	const GEN_FLT x144 = x81 - x142 * x143;
	const GEN_FLT x145 = cos(x110) * ogeePhase_0;
	const GEN_FLT x146 = x105 * x104;
	const GEN_FLT x147 = x146 * x145;
	const GEN_FLT x148 = x98 * x116 / pow(x113, 2);
	const GEN_FLT x149 = 2 * x99 * x111 * x114;
	const GEN_FLT x150 = x98 * x114 * x115;
	const GEN_FLT x151 = x145 * x150;
	const GEN_FLT x152 =
		x119 * (x142 + x117 * x138 + x133 * x149 + x144 * x151 -
				x148 * (-x112 * (x103 * x133 + x132 * x139 + x93 * x138 +
								 x93 * (x138 + x102 * x133 +
										x93 * (x136 + x101 * x133 + x93 * (x135 + x100 * x133 - x133 * x134)))) -
						x144 * x147));
	const GEN_FLT x153 = cos(gibPhase_0 + x109 - asin(x118)) * gibMag_0;
	const GEN_FLT x154 = x73 * x77;
	const GEN_FLT x155 = x80 * (x154 + x70);
	const GEN_FLT x156 = 1 + x122;
	const GEN_FLT x157 = x73 * x125;
	const GEN_FLT x158 = x127 + x157;
	const GEN_FLT x159 = -x130 * (x158 + x123 * x156) + x92 * x156;
	const GEN_FLT x160 = x120 * x159;
	const GEN_FLT x161 = x95 * x160;
	const GEN_FLT x162 = x93 * (x161 - x94 * x160) + x96 * x160;
	const GEN_FLT x163 = x137 * x159 + x93 * x162;
	const GEN_FLT x164 = x107 * x156 - x141 * x158;
	const GEN_FLT x165 = x155 - x164 * x143;
	const GEN_FLT x166 =
		x119 * (x164 + x117 * x163 -
				x148 * (-x112 * (x103 * x160 + x139 * x159 + x93 * x163 +
								 x93 * (x163 + x102 * x160 +
										x93 * (x162 + x101 * x160 + x93 * (x161 + x100 * x160 - x160 * x134)))) -
						x165 * x147) +
				x160 * x149 + x165 * x151);
	const GEN_FLT x167 = 1 + x69;
	const GEN_FLT x168 = x80 * (x154 - x39 * x167);
	const GEN_FLT x169 = x157 + x126 * x167;
	const GEN_FLT x170 = x131 - (x124 + x169) * x130;
	const GEN_FLT x171 = x120 * x170;
	const GEN_FLT x172 = x95 * x171;
	const GEN_FLT x173 = x93 * (x172 - x94 * x171) + x96 * x171;
	const GEN_FLT x174 = x170 * x137 + x93 * x173;
	const GEN_FLT x175 = x140 - x169 * x141;
	const GEN_FLT x176 = x145 * (x168 - x175 * x143);
	const GEN_FLT x177 =
		x119 * (x175 + x117 * x174 -
				x148 * (-x112 * (x103 * x171 + x93 * x174 +
								 x93 * (x174 + x102 * x171 +
										x93 * (x173 + x101 * x171 + x93 * (x172 + x100 * x171 - x171 * x134))) +
								 x98 * x171) -
						x176 * x146) +
				x171 * x149 + x176 * x150);
	const GEN_FLT x178 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qj * lh_qi / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x179 = x6 * x178;
	const GEN_FLT x180 = -x179;
	const GEN_FLT x181 = pow(x5, -1);
	const GEN_FLT x182 = x181 * lh_qi;
	const GEN_FLT x183 = x9 * x182;
	const GEN_FLT x184 = x0 * x183;
	const GEN_FLT x185 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qi, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x186 = x8 * x60;
	const GEN_FLT x187 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qk * lh_qi / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x188 = x12 * x187;
	const GEN_FLT x189 = x188 + x186 * x182 + x43 * x185;
	const GEN_FLT x190 = x6 * x185;
	const GEN_FLT x191 = x11 * x183;
	const GEN_FLT x192 = x43 * x178;
	const GEN_FLT x193 = x47 * x187;
	const GEN_FLT x194 = x0 * x182;
	const GEN_FLT x195 = x192 + x193 + x29 * x194;
	const GEN_FLT x196 = x6 * x182;
	const GEN_FLT x197 = -x196;
	const GEN_FLT x198 = x43 * x187;
	const GEN_FLT x199 =
		x68 + x28 * (x197 + 2 * x198 + x65 * x196) + x34 * (x190 + x191 + x195) + x35 * (x180 - x184 + x189);
	const GEN_FLT x200 = x6 * x187;
	const GEN_FLT x201 = -x200;
	const GEN_FLT x202 = x8 * x183;
	const GEN_FLT x203 = x12 * x178;
	const GEN_FLT x204 = x203 + x47 * x185 + x60 * x194;
	const GEN_FLT x205 =
		x72 + x28 * (x179 + x184 + x189) + x34 * (x201 - x202 + x204) + x35 * (x197 + 2 * x12 * x185 + x36 * x196);
	const GEN_FLT x206 = (-x39 * x199 + x77 * x205) * x80;
	const GEN_FLT x207 = x47 * x178;
	const GEN_FLT x208 =
		x121 + x28 * (-x190 - x191 + x195) + x34 * (x197 + 2 * x207 + x83 * x196) + x35 * (x200 + x202 + x204);
	const GEN_FLT x209 = x126 * x199 + x205 * x125;
	const GEN_FLT x210 = x120 * (-x130 * (x209 + x208 * x123) + x92 * x208);
	const GEN_FLT x211 = x95 * x210;
	const GEN_FLT x212 = x93 * (x211 - x94 * x210) + x96 * x210;
	const GEN_FLT x213 = x93 * x212 + x97 * x210;
	const GEN_FLT x214 = x208 * x107 - x209 * x141;
	const GEN_FLT x215 = x206 - x214 * x143;
	const GEN_FLT x216 =
		x119 * (x214 -
				x148 * (-x112 * (x210 * x103 + x93 * x213 +
								 x93 * (x213 + x210 * x102 +
										x93 * (x212 + x210 * x101 + x93 * (x211 + x210 * x100 - x210 * x134))) +
								 x98 * x210) -
						x215 * x147) +
				x210 * x149 + x213 * x117 + x215 * x151);
	const GEN_FLT x217 = x181 * lh_qj;
	const GEN_FLT x218 = x9 * x217;
	const GEN_FLT x219 = x11 * x218;
	const GEN_FLT x220 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qk * lh_qj / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x221 = x47 * x220;
	const GEN_FLT x222 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qj, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x223 = x0 * x29;
	const GEN_FLT x224 = x221 + x217 * x223 + x43 * x222;
	const GEN_FLT x225 = x6 * x222;
	const GEN_FLT x226 = x0 * x218;
	const GEN_FLT x227 = x12 * x220;
	const GEN_FLT x228 = x60 * x217;
	const GEN_FLT x229 = x192 + x227 + x8 * x228;
	const GEN_FLT x230 = x6 * x217;
	const GEN_FLT x231 = -x230;
	const GEN_FLT x232 = x43 * x220;
	const GEN_FLT x233 =
		x68 + x28 * (x231 + 2 * x232 + x65 * x230) + x34 * (x179 + x219 + x224) + x35 * (-x225 - x226 + x229);
	const GEN_FLT x234 = x6 * x220;
	const GEN_FLT x235 = -x234;
	const GEN_FLT x236 = x8 * x218;
	const GEN_FLT x237 = x207 + x0 * x228 + x12 * x222;
	const GEN_FLT x238 =
		x72 + x28 * (x225 + x226 + x229) + x34 * (x235 - x236 + x237) + x35 * (2 * x203 + x231 + x36 * x230);
	const GEN_FLT x239 = (-x39 * x233 + x77 * x238) * x80;
	const GEN_FLT x240 =
		x121 + x28 * (x180 - x219 + x224) + x34 * (x231 + 2 * x47 * x222 + x83 * x230) + x35 * (x234 + x236 + x237);
	const GEN_FLT x241 = x233 * x126 + x238 * x125;
	const GEN_FLT x242 = -x130 * (x241 + x240 * x123) + x92 * x240;
	const GEN_FLT x243 = x242 * x120;
	const GEN_FLT x244 = x95 * x243;
	const GEN_FLT x245 = x93 * (x244 - x94 * x243) + x96 * x243;
	const GEN_FLT x246 = x242 * x137 + x93 * x245;
	const GEN_FLT x247 = x240 * x107 - x241 * x141;
	const GEN_FLT x248 = x239 - x247 * x143;
	const GEN_FLT x249 =
		x119 * (x247 -
				x148 * (-x112 * (x242 * x139 + x243 * x103 + x93 * x246 +
								 x93 * (x246 + x243 * x102 +
										x93 * (x245 + x243 * x101 + x93 * (x244 + x243 * x100 - x243 * x134)))) -
						x248 * x147) +
				x243 * x149 + x246 * x117 + x248 * x151);
	const GEN_FLT x250 = x181 * lh_qk;
	const GEN_FLT x251 = x9 * x250;
	const GEN_FLT x252 = x0 * x251;
	const GEN_FLT x253 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qk, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x254 = x198 + x12 * x253 + x250 * x186;
	const GEN_FLT x255 = x11 * x251;
	const GEN_FLT x256 = x232 + x250 * x223 + x47 * x253;
	const GEN_FLT x257 = x6 * x250;
	const GEN_FLT x258 = -x257;
	const GEN_FLT x259 =
		x68 + x28 * (x258 + 2 * x43 * x253 + x65 * x257) + x34 * (x200 + x255 + x256) + x35 * (x235 - x252 + x254);
	const GEN_FLT x260 = x6 * x253;
	const GEN_FLT x261 = x8 * x251;
	const GEN_FLT x262 = x193 + x227 + x0 * x60 * x250;
	const GEN_FLT x263 =
		x72 + x28 * (x234 + x252 + x254) + x34 * (-x260 - x261 + x262) + x35 * (2 * x188 + x258 + x36 * x257);
	const GEN_FLT x264 = (-x39 * x259 + x77 * x263) * x80;
	const GEN_FLT x265 =
		x121 + x28 * (x201 - x255 + x256) + x34 * (2 * x221 + x258 + x83 * x257) + x35 * (x260 + x261 + x262);
	const GEN_FLT x266 = x259 * x126 + x263 * x125;
	const GEN_FLT x267 = -x130 * (x266 + x265 * x123) + x92 * x265;
	const GEN_FLT x268 = x267 * x120;
	const GEN_FLT x269 = x95 * x268;
	const GEN_FLT x270 = x93 * (x269 - x94 * x268) + x96 * x268;
	const GEN_FLT x271 = x267 * x137 + x93 * x270;
	const GEN_FLT x272 = x265 * x107 - x266 * x141;
	const GEN_FLT x273 = x264 - x272 * x143;
	const GEN_FLT x274 =
		x119 * (x272 -
				x148 * (-x112 * (x268 * x103 + x93 * x271 +
								 x93 * (x271 + x268 * x102 +
										x93 * (x270 + x268 * x101 + x93 * (x269 + x268 * x100 - x268 * x134))) +
								 x98 * x268) -
						x273 * x147) +
				x268 * x149 + x271 * x117 + x273 * x151);
	*(out++) = -x152 + x81 - x153 * (x152 - x81);
	*(out++) = x155 - x166 - (-x155 + x166) * x153;
	*(out++) = x168 - x177 - (-x168 + x177) * x153;
	*(out++) = x206 - x216 - (-x206 + x216) * x153;
	*(out++) = x239 - x249 - (-x239 + x249) * x153;
	*(out++) = x264 - x274 - (-x264 + x274) * x153;
}

// Jacobian of reproject_axis_x_gen2 wrt [phase_0, tilt_0, curve_0, gibPhase_0, gibMag_0, ogeeMag_0, ogeePhase_0]
static inline void gen_reproject_axis_x_gen2_jac_bsc0_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
																 const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
																 const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = cos(x1);
	const GEN_FLT x3 = 1 - x2;
	const GEN_FLT x4 = x2 + pow(x0, 2) * x3;
	const GEN_FLT x5 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x6 = sin(x5);
	const GEN_FLT x7 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1));
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (0));
	const GEN_FLT x10 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x11 = cos(x5);
	const GEN_FLT x12 = 1 - x11;
	const GEN_FLT x13 = x12 * x10;
	const GEN_FLT x14 = x9 * x13;
	const GEN_FLT x15 = x6 * x10;
	const GEN_FLT x16 = x7 * x12;
	const GEN_FLT x17 = x9 * x16;
	const GEN_FLT x18 = obj_pz + (x11 + pow(x9, 2) * x12) * sensor_z + (-x15 + x17) * sensor_x + (x14 + x8) * sensor_y;
	const GEN_FLT x19 = x6 * x9;
	const GEN_FLT x20 = x7 * x13;
	const GEN_FLT x21 = obj_py + (x11 + x12 * pow(x10, 2)) * sensor_y + (x19 + x20) * sensor_x + (x14 - x8) * sensor_z;
	const GEN_FLT x22 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (1));
	const GEN_FLT x23 = sin(x1);
	const GEN_FLT x24 = x22 * x23;
	const GEN_FLT x25 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (0));
	const GEN_FLT x26 = x0 * x3 * x25;
	const GEN_FLT x27 = x24 + x26;
	const GEN_FLT x28 = obj_px + (x11 + pow(x7, 2) * x12) * sensor_x + (x15 + x17) * sensor_z + (-x19 + x20) * sensor_y;
	const GEN_FLT x29 = x25 * x23;
	const GEN_FLT x30 = x3 * x22;
	const GEN_FLT x31 = x0 * x30;
	const GEN_FLT x32 = -x29 + x31;
	const GEN_FLT x33 = lh_pz + x21 * x27 + x32 * x28 + x4 * x18;
	const GEN_FLT x34 = x29 + x31;
	const GEN_FLT x35 = x0 * x23;
	const GEN_FLT x36 = x30 * x25;
	const GEN_FLT x37 = -x35 + x36;
	const GEN_FLT x38 = x2 + x3 * pow(x22, 2);
	const GEN_FLT x39 = lh_px + x34 * x18 + x37 * x21 + x38 * x28;
	const GEN_FLT x40 = pow(x39, 2);
	const GEN_FLT x41 = x40 + pow(x33, 2);
	const GEN_FLT x42 = pow(x41, -1);
	const GEN_FLT x43 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x44 = x3 * x43;
	const GEN_FLT x45 = x0 * x44;
	const GEN_FLT x46 = x44 * x22;
	const GEN_FLT x47 = x43 * x23;
	const GEN_FLT x48 = -x47;
	const GEN_FLT x49 = x46 + x48;
	const GEN_FLT x50 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x51 = x6 * x50;
	const GEN_FLT x52 = x50 * x16;
	const GEN_FLT x53 = x50 * x13;
	const GEN_FLT x54 = x52 + x53;
	const GEN_FLT x55 = -x51;
	const GEN_FLT x56 = x9 * x50 * x12;
	const GEN_FLT x57 = x53 + x56;
	const GEN_FLT x58 = 2 * x53 * sensor_y + (x51 + x54) * sensor_x + (x55 + x57) * sensor_z;
	const GEN_FLT x59 = x44 * x25;
	const GEN_FLT x60 = x45 + x59;
	const GEN_FLT x61 = x52 + x56;
	const GEN_FLT x62 = 2 * x52 * sensor_x + (x54 + x55) * sensor_y + (x51 + x61) * sensor_z;
	const GEN_FLT x63 = 2 * x56 * sensor_z + (x51 + x57) * sensor_y + (x55 + x61) * sensor_x;
	const GEN_FLT x64 = x4 * x63 + 2 * x45 * x18 + x58 * x27 + x62 * x32 + (x45 + x49) * x28 + (x47 + x60) * x21;
	const GEN_FLT x65 = x46 + x47;
	const GEN_FLT x66 = 2 * x46 * x28 + x58 * x37 + x62 * x38 + x63 * x34 + (x49 + x59) * x21 + (x45 + x65) * x18;
	const GEN_FLT x67 = x40 * x42 * (-x64 / x39 + x66 * x33 / x40);
	const GEN_FLT x68 = -x67;
	const GEN_FLT x69 = -x24 + x26;
	const GEN_FLT x70 = x2 + x3 * pow(x25, 2);
	const GEN_FLT x71 = x35 + x36;
	const GEN_FLT x72 = lh_py + x69 * x18 + x70 * x21 + x71 * x28;
	const GEN_FLT x73 = 0.523598775598299 + tilt_0;
	const GEN_FLT x74 = cos(x73);
	const GEN_FLT x75 = pow(x74, -1);
	const GEN_FLT x76 = pow(x72, 2);
	const GEN_FLT x77 = x41 + x76;
	const GEN_FLT x78 = pow(x77, -1.0 / 2.0);
	const GEN_FLT x79 = x78 * x75;
	const GEN_FLT x80 = asin(x72 * x79);
	const GEN_FLT x81 = -8.0108022e-06 - 1.60216044e-05 * x80;
	const GEN_FLT x82 = 8.0108022e-06 * x80;
	const GEN_FLT x83 = -8.0108022e-06 - x82;
	const GEN_FLT x84 = 0.0028679863 + x80 * x83;
	const GEN_FLT x85 = x84 + x80 * x81;
	const GEN_FLT x86 = 5.3685255e-06 + x80 * x84;
	const GEN_FLT x87 = x86 + x80 * x85;
	const GEN_FLT x88 = pow(x74, -2);
	const GEN_FLT x89 = pow(1 - x88 * x76 / x77, -1.0 / 2.0);
	const GEN_FLT x90 = 2 * x59 * x21 + x63 * x69 + x70 * x58 + x71 * x62 + (x48 + x60) * x18 + (x59 + x65) * x28;
	const GEN_FLT x91 = 2 * x64 * x33 + 2 * x66 * x39;
	const GEN_FLT x92 = (1.0 / 2.0) * x72;
	const GEN_FLT x93 = x79 * x90 - x75 * x92 * (x91 + 2 * x72 * x90) / pow(x77, 3.0 / 2.0);
	const GEN_FLT x94 = x89 * x93;
	const GEN_FLT x95 = x83 * x94;
	const GEN_FLT x96 = 2.40324066e-05 * x80;
	const GEN_FLT x97 = x80 * (x95 - x82 * x94) + x84 * x94;
	const GEN_FLT x98 = x80 * x97 + x86 * x94;
	const GEN_FLT x99 = 0.0076069798 + x80 * x86;
	const GEN_FLT x100 = x99 + x80 * x87;
	const GEN_FLT x101 = sin(x73);
	const GEN_FLT x102 = pow(x41, -1.0 / 2.0);
	const GEN_FLT x103 = tan(x73);
	const GEN_FLT x104 = x103 * x102;
	const GEN_FLT x105 = x72 * x104;
	const GEN_FLT x106 = atan2(-x33, x39);
	const GEN_FLT x107 = ogeeMag_0 + x106 - asin(x105);
	const GEN_FLT x108 = sin(x107);
	const GEN_FLT x109 = curve_0 + x108 * ogeePhase_0;
	const GEN_FLT x110 = x101 * x109;
	const GEN_FLT x111 =
		-x110 * (x80 * x98 + x80 * (x98 + x80 * (x97 + x80 * (x95 + x81 * x94 - x96 * x94) + x85 * x94) + x87 * x94) +
				 x94 * x100 + x99 * x94);
	const GEN_FLT x112 = x90 * x104 - x92 * x91 * x103 / pow(x41, 3.0 / 2.0);
	const GEN_FLT x113 = pow(x103, 2);
	const GEN_FLT x114 = pow(1 - x76 * x42 * x113, -1.0 / 2.0);
	const GEN_FLT x115 = x67 - x112 * x114;
	const GEN_FLT x116 = cos(x107) * ogeePhase_0;
	const GEN_FLT x117 = x115 * x116;
	const GEN_FLT x118 = x80 * x99;
	const GEN_FLT x119 = x118 + x80 * x100;
	const GEN_FLT x120 = x101 * x119;
	const GEN_FLT x121 = x74 - x110 * x119;
	const GEN_FLT x122 = pow(x80, 2);
	const GEN_FLT x123 = x99 * x122;
	const GEN_FLT x124 = x109 * x123 / pow(x121, 2);
	const GEN_FLT x125 = pow(x121, -1);
	const GEN_FLT x126 = x123 * x125;
	const GEN_FLT x127 = x109 * x125;
	const GEN_FLT x128 = 2 * x118 * x127;
	const GEN_FLT x129 = x122 * x127;
	const GEN_FLT x130 = x112 + x94 * x128 + x98 * x129;
	const GEN_FLT x131 = x105 + x99 * x129;
	const GEN_FLT x132 = pow(1 - pow(x131, 2), -1.0 / 2.0);
	const GEN_FLT x133 = x132 * (x130 + x117 * x126 - x124 * (x111 - x117 * x120));
	const GEN_FLT x134 = x133 + x68;
	const GEN_FLT x135 = -gibPhase_0 - x106 + asin(x131);
	const GEN_FLT x136 = cos(x135) * gibMag_0;
	const GEN_FLT x137 = -x133 + x67;
	const GEN_FLT x138 = x137 - x134 * x136;
	const GEN_FLT x139 = x89 * (x93 + x88 * x72 * x78 * x101);
	const GEN_FLT x140 = x83 * x139;
	const GEN_FLT x141 = x80 * (x140 - x82 * x139) + x84 * x139;
	const GEN_FLT x142 = x80 * x141 + x86 * x139;
	const GEN_FLT x143 = x112 + x72 * x102 * (1 + x113);
	const GEN_FLT x144 = x67 - x114 * x143;
	const GEN_FLT x145 = x116 * x120;
	const GEN_FLT x146 = x116 * x126;
	const GEN_FLT x147 =
		x132 *
		(x143 -
		 x124 *
			 (-x101 -
			  x110 * (x100 * x139 + x80 * x142 +
					  x80 * (x142 + x80 * (x141 + x80 * (x140 + x81 * x139 - x96 * x139) + x85 * x139) + x87 * x139) +
					  x99 * x139) -
			  x144 * x145 - x74 * x109 * x119) +
		 x128 * x139 + x129 * x142 + x144 * x146);
	const GEN_FLT x148 = 1 + x117;
	const GEN_FLT x149 = x132 * (x130 - x124 * (x111 - x120 * x148) + x126 * x148);
	const GEN_FLT x150 = 1 + x115;
	const GEN_FLT x151 = x132 * (x130 - x124 * (x111 - x145 * x150) + x146 * x150);
	const GEN_FLT x152 = x108 + x117;
	const GEN_FLT x153 = x132 * (x130 - x124 * (x111 - x120 * x152) + x126 * x152);
	*(out++) = -1 + x138;
	*(out++) = -x147 + x67 - x136 * (x147 + x68);
	*(out++) = -x149 + x67 - x136 * (x149 + x68);
	*(out++) = x137 - x136 * (-1 + x134);
	*(out++) = x138 - sin(x135);
	*(out++) = -x151 + x67 - x136 * (x151 + x68);
	*(out++) = -x153 + x67 - x136 * (x153 + x68);
}

/** Applying function <function reproject_axis_y_gen2 at 0x7ffa3c1c7a70> */
static inline void gen_reproject_axis_y_gen2_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
														const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
														const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = cos(x1);
	const GEN_FLT x3 = 1 - x2;
	const GEN_FLT x4 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x5 = sin(x4);
	const GEN_FLT x6 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1));
	const GEN_FLT x7 = x6 * x5;
	const GEN_FLT x8 = cos(x4);
	const GEN_FLT x9 = 1 - x8;
	const GEN_FLT x10 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x11 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x12 = x9 * x11 * x10;
	const GEN_FLT x13 = x5 * x10;
	const GEN_FLT x14 = x6 * x9;
	const GEN_FLT x15 = x14 * x11;
	const GEN_FLT x16 = obj_pz + (-x13 + x15) * sensor_x + (x12 + x7) * sensor_y + (x8 + x9 * pow(x11, 2)) * sensor_z;
	const GEN_FLT x17 = x5 * x11;
	const GEN_FLT x18 = x14 * x10;
	const GEN_FLT x19 = obj_py + (x17 + x18) * sensor_x + (x12 - x7) * sensor_z + (x8 + x9 * pow(x10, 2)) * sensor_y;
	const GEN_FLT x20 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (1));
	const GEN_FLT x21 = sin(x1);
	const GEN_FLT x22 = x20 * x21;
	const GEN_FLT x23 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (0));
	const GEN_FLT x24 = x0 * x3 * x23;
	const GEN_FLT x25 = obj_px + (x13 + x15) * sensor_z + (-x17 + x18) * sensor_y + (x8 + pow(x6, 2) * x9) * sensor_x;
	const GEN_FLT x26 = x23 * x21;
	const GEN_FLT x27 = x3 * x20;
	const GEN_FLT x28 = x0 * x27;
	const GEN_FLT x29 = lh_pz + x16 * (x2 + pow(x0, 2) * x3) + (x22 + x24) * x19 + (-x26 + x28) * x25;
	const GEN_FLT x30 = x0 * x21;
	const GEN_FLT x31 = x23 * x27;
	const GEN_FLT x32 = lh_px + x25 * (x2 + x3 * pow(x20, 2)) + (x26 + x28) * x16 + (-x30 + x31) * x19;
	const GEN_FLT x33 = atan2(-x29, x32);
	const GEN_FLT x34 = lh_py + x19 * (x2 + x3 * pow(x23, 2)) + (-x22 + x24) * x16 + (x30 + x31) * x25;
	const GEN_FLT x35 = 0.523598775598299 - tilt_1;
	const GEN_FLT x36 = cos(x35);
	const GEN_FLT x37 = pow(x29, 2) + pow(x32, 2);
	const GEN_FLT x38 = asin(x34 / (x36 * sqrt(x37 + pow(x34, 2))));
	const GEN_FLT x39 = -x34 * tan(x35) / sqrt(x37);
	const GEN_FLT x40 = curve_1 + sin(ogeeMag_1 + x33 - asin(x39)) * ogeePhase_1;
	const GEN_FLT x41 = 0.0028679863 + x38 * (-8.0108022e-06 - 8.0108022e-06 * x38);
	const GEN_FLT x42 = 5.3685255e-06 + x41 * x38;
	const GEN_FLT x43 = 0.0076069798 + x42 * x38;
	const GEN_FLT x44 =
		x33 -
		asin(x39 +
			 x40 * x43 * pow(x38, 2) /
				 (x36 + x40 * sin(x35) *
							(x38 * (x43 + x38 * (x42 + x38 * (x41 + x38 * (-8.0108022e-06 - 1.60216044e-05 * x38)))) +
							 x43 * x38)));
	*(out++) = -1.5707963267949 - phase_1 + x44 + sin(gibPhase_1 + x44) * gibMag_1;
}

// Jacobian of reproject_axis_y_gen2 wrt [obj_px, obj_py, obj_pz, obj_qi, obj_qj, obj_qk]
static inline void gen_reproject_axis_y_gen2_jac_obj_p_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
																  const FLT *sensor_pt,
																  const LinmathAxisAnglePose *lh_p,
																  const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (1));
	const GEN_FLT x5 = cos(x1);
	const GEN_FLT x6 = 1 - x5;
	const GEN_FLT x7 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = pow(obj_qj, 2);
	const GEN_FLT x13 = pow(obj_qi, 2);
	const GEN_FLT x14 = x11 + x12 + x13;
	const GEN_FLT x15 = sqrt(x14);
	const GEN_FLT x16 = sin(x15);
	const GEN_FLT x17 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x20 = cos(x15);
	const GEN_FLT x21 = 1 - x20;
	const GEN_FLT x22 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = x23 * x19;
	const GEN_FLT x25 = pow(x19, 2);
	const GEN_FLT x26 = x22 * x16;
	const GEN_FLT x27 = x21 * x17;
	const GEN_FLT x28 = x27 * x19;
	const GEN_FLT x29 = obj_pz + (x20 + x25 * x21) * sensor_z + (x18 + x24) * sensor_y + (-x26 + x28) * sensor_x;
	const GEN_FLT x30 = x2 * x7;
	const GEN_FLT x31 = x0 * x4 * x6;
	const GEN_FLT x32 = -x30 + x31;
	const GEN_FLT x33 = pow(x22, 2);
	const GEN_FLT x34 = x19 * x16;
	const GEN_FLT x35 = x23 * x17;
	const GEN_FLT x36 = obj_py + (x20 + x33 * x21) * sensor_y + (-x18 + x24) * sensor_z + (x34 + x35) * sensor_x;
	const GEN_FLT x37 = pow(x17, 2);
	const GEN_FLT x38 = obj_px + (x20 + x37 * x21) * sensor_x + (x26 + x28) * sensor_z + (-x34 + x35) * sensor_y;
	const GEN_FLT x39 = x5 + pow(x4, 2) * x6;
	const GEN_FLT x40 = lh_px + x29 * x10 + x32 * x36 + x38 * x39;
	const GEN_FLT x41 = pow(x40, -1);
	const GEN_FLT x42 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x43 = x42 * x27;
	const GEN_FLT x44 = x42 * x23;
	const GEN_FLT x45 = x42 * x16;
	const GEN_FLT x46 = -x45;
	const GEN_FLT x47 = x44 + x46;
	const GEN_FLT x48 = x21 * x19;
	const GEN_FLT x49 = x42 * x48;
	const GEN_FLT x50 = x45 + x49;
	const GEN_FLT x51 = 2 * x43 * sensor_x + (x43 + x47) * sensor_y + (x43 + x50) * sensor_z;
	const GEN_FLT x52 = 1 + x51;
	const GEN_FLT x53 = -x3 + x9;
	const GEN_FLT x54 = x5 + x6 * pow(x7, 2);
	const GEN_FLT x55 = 2 * x49 * sensor_z + (x44 + x50) * sensor_y + (x43 + x46 + x49) * sensor_x;
	const GEN_FLT x56 = x54 * x55;
	const GEN_FLT x57 = x2 * x4;
	const GEN_FLT x58 = x0 * x8;
	const GEN_FLT x59 = x57 + x58;
	const GEN_FLT x60 = 2 * x44 * sensor_y + (x47 + x49) * sensor_z + (x43 + x44 + x45) * sensor_x;
	const GEN_FLT x61 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x62 = x6 * x61;
	const GEN_FLT x63 = x4 * x62;
	const GEN_FLT x64 = x7 * x62;
	const GEN_FLT x65 = x2 * x61;
	const GEN_FLT x66 = -x65;
	const GEN_FLT x67 = x64 + x66;
	const GEN_FLT x68 = x0 * x62;
	const GEN_FLT x69 = x65 + x68;
	const GEN_FLT x70 = 2 * x64 * x29 + (x63 + x67) * x38 + (x64 + x69) * x36;
	const GEN_FLT x71 = x70 + x60 * x59;
	const GEN_FLT x72 = x56 + x71 + x53 * x52;
	const GEN_FLT x73 = x60 * x32;
	const GEN_FLT x74 = x55 * x10;
	const GEN_FLT x75 = x29 * (x63 + x64 + x65) + x36 * (x63 + x66 + x68) + 2 * x63 * x38;
	const GEN_FLT x76 = x73 + x74 + x75 + x52 * x39;
	const GEN_FLT x77 = pow(x40, 2);
	const GEN_FLT x78 = lh_pz + x53 * x38 + x54 * x29 + x59 * x36;
	const GEN_FLT x79 = x78 / x77;
	const GEN_FLT x80 = x77 + pow(x78, 2);
	const GEN_FLT x81 = pow(x80, -1);
	const GEN_FLT x82 = x81 * x77;
	const GEN_FLT x83 = (-x72 * x41 + x79 * x76) * x82;
	const GEN_FLT x84 = x30 + x31;
	const GEN_FLT x85 = -x57 + x58;
	const GEN_FLT x86 = x85 * x55;
	const GEN_FLT x87 = x5 + pow(x0, 2) * x6;
	const GEN_FLT x88 = 2 * x68 * x36 + (x63 + x69) * x38 + (x67 + x68) * x29;
	const GEN_FLT x89 = x88 + x87 * x60;
	const GEN_FLT x90 = x86 + x89 + x84 * x52;
	const GEN_FLT x91 = lh_py + x84 * x38 + x85 * x29 + x87 * x36;
	const GEN_FLT x92 = 2 * x91;
	const GEN_FLT x93 = 2 * x40;
	const GEN_FLT x94 = 2 * x78;
	const GEN_FLT x95 = x72 * x94 + x76 * x93;
	const GEN_FLT x96 = 0.523598775598299 - tilt_1;
	const GEN_FLT x97 = cos(x96);
	const GEN_FLT x98 = pow(x97, -1);
	const GEN_FLT x99 = pow(x91, 2);
	const GEN_FLT x100 = x80 + x99;
	const GEN_FLT x101 = (1.0 / 2.0) * x91;
	const GEN_FLT x102 = x98 * x101 / pow(x100, 3.0 / 2.0);
	const GEN_FLT x103 = x98 / sqrt(x100);
	const GEN_FLT x104 = -x102 * (x95 + x92 * x90) + x90 * x103;
	const GEN_FLT x105 = pow(1 - x99 / (pow(x97, 2) * x100), -1.0 / 2.0);
	const GEN_FLT x106 = x105 * x104;
	const GEN_FLT x107 = tan(x96);
	const GEN_FLT x108 = x107 / sqrt(x80);
	const GEN_FLT x109 = -x91 * x108;
	const GEN_FLT x110 = atan2(-x78, x40);
	const GEN_FLT x111 = ogeeMag_1 + x110 - asin(x109);
	const GEN_FLT x112 = curve_1 + sin(x111) * ogeePhase_1;
	const GEN_FLT x113 = asin(x91 * x103);
	const GEN_FLT x114 = 8.0108022e-06 * x113;
	const GEN_FLT x115 = -8.0108022e-06 - x114;
	const GEN_FLT x116 = 0.0028679863 + x113 * x115;
	const GEN_FLT x117 = 5.3685255e-06 + x113 * x116;
	const GEN_FLT x118 = 0.0076069798 + x113 * x117;
	const GEN_FLT x119 = x113 * x118;
	const GEN_FLT x120 = -8.0108022e-06 - 1.60216044e-05 * x113;
	const GEN_FLT x121 = x116 + x113 * x120;
	const GEN_FLT x122 = x117 + x113 * x121;
	const GEN_FLT x123 = x118 + x113 * x122;
	const GEN_FLT x124 = x119 + x113 * x123;
	const GEN_FLT x125 = sin(x96);
	const GEN_FLT x126 = x112 * x125;
	const GEN_FLT x127 = x97 + x124 * x126;
	const GEN_FLT x128 = pow(x127, -1);
	const GEN_FLT x129 = 2 * x112 * x119 * x128;
	const GEN_FLT x130 = x101 * x107 / pow(x80, 3.0 / 2.0);
	const GEN_FLT x131 = -x90 * x108 + x95 * x130;
	const GEN_FLT x132 = pow(1 - x81 * x99 * pow(x107, 2), -1.0 / 2.0);
	const GEN_FLT x133 = x83 - x131 * x132;
	const GEN_FLT x134 = cos(x111) * ogeePhase_1;
	const GEN_FLT x135 = pow(x113, 2);
	const GEN_FLT x136 = x118 * x128 * x135;
	const GEN_FLT x137 = x134 * x136;
	const GEN_FLT x138 = x124 * x125;
	const GEN_FLT x139 = x134 * x138;
	const GEN_FLT x140 = x106 * x115;
	const GEN_FLT x141 = 2.40324066e-05 * x113;
	const GEN_FLT x142 = x105 * x116;
	const GEN_FLT x143 = x104 * x142 + x113 * (x140 - x106 * x114);
	const GEN_FLT x144 = x106 * x117 + x113 * x143;
	const GEN_FLT x145 = x105 * x123;
	const GEN_FLT x146 = x112 * x135;
	const GEN_FLT x147 = x118 * x146 / pow(x127, 2);
	const GEN_FLT x148 = x128 * x146;
	const GEN_FLT x149 = x109 + x118 * x148;
	const GEN_FLT x150 = pow(1 - pow(x149, 2), -1.0 / 2.0);
	const GEN_FLT x151 =
		x83 -
		x150 * (x131 + x106 * x129 + x133 * x137 + x144 * x148 -
				x147 * (x126 * (x104 * x145 + x106 * x118 + x113 * x144 +
								x113 * (x144 + x106 * x122 +
										x113 * (x143 + x106 * x121 + x113 * (x140 + x106 * x120 - x106 * x141)))) +
						x133 * x139));
	const GEN_FLT x152 = cos(gibPhase_1 + x110 - asin(x149)) * gibMag_1;
	const GEN_FLT x153 = x53 * x51;
	const GEN_FLT x154 = 1 + x60;
	const GEN_FLT x155 = x153 + x56 + x70 + x59 * x154;
	const GEN_FLT x156 = x75 + x51 * x39;
	const GEN_FLT x157 = x156 + x74 + x32 * x154;
	const GEN_FLT x158 = (-x41 * x155 + x79 * x157) * x82;
	const GEN_FLT x159 = x84 * x51;
	const GEN_FLT x160 = x159 + x86 + x88 + x87 * x154;
	const GEN_FLT x161 = x93 * x157 + x94 * x155;
	const GEN_FLT x162 = -x102 * (x161 + x92 * x160) + x103 * x160;
	const GEN_FLT x163 = x105 * x162;
	const GEN_FLT x164 = -x108 * x160 + x161 * x130;
	const GEN_FLT x165 = x134 * (x158 - x164 * x132);
	const GEN_FLT x166 = x115 * x163;
	const GEN_FLT x167 = x105 * x120;
	const GEN_FLT x168 = x113 * (x166 - x114 * x163) + x162 * x142;
	const GEN_FLT x169 = x113 * x168 + x117 * x163;
	const GEN_FLT x170 =
		x158 -
		x150 * (x164 + x129 * x163 -
				x147 * (x126 * (x113 * x169 +
								x113 * (x169 + x113 * (x168 + x113 * (x166 - x163 * x141 + x167 * x162) + x121 * x163) +
										x122 * x163) +
								x118 * x163 + x162 * x145) +
						x165 * x138) +
				x165 * x136 + x169 * x148);
	const GEN_FLT x171 = 1 + x55;
	const GEN_FLT x172 = x153 + x71 + x54 * x171;
	const GEN_FLT x173 = x156 + x73 + x10 * x171;
	const GEN_FLT x174 = (-x41 * x172 + x79 * x173) * x82;
	const GEN_FLT x175 = x159 + x89 + x85 * x171;
	const GEN_FLT x176 = x93 * x173 + x94 * x172;
	const GEN_FLT x177 = -x102 * (x176 + x92 * x175) + x103 * x175;
	const GEN_FLT x178 = x105 * x177;
	const GEN_FLT x179 = -x108 * x175 + x176 * x130;
	const GEN_FLT x180 = x174 - x179 * x132;
	const GEN_FLT x181 = x115 * x178;
	const GEN_FLT x182 = x113 * (x181 - x114 * x178) + x177 * x142;
	const GEN_FLT x183 = x113 * x182 + x117 * x178;
	const GEN_FLT x184 =
		x174 -
		x150 * (x179 + x129 * x178 -
				x147 * (x126 * (x113 * x183 +
								x113 * (x183 + x113 * (x182 + x113 * (x181 + x120 * x178 - x178 * x141) + x121 * x178) +
										x122 * x178) +
								x118 * x178 + x177 * x145) +
						x180 * x139) +
				x180 * x137 + x183 * x148);
	const GEN_FLT x185 = pow(x15, -1);
	const GEN_FLT x186 = x185 * obj_qi;
	const GEN_FLT x187 = x16 * x186;
	const GEN_FLT x188 = -x187;
	const GEN_FLT x189 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qi, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x190 = x20 * x19;
	const GEN_FLT x191 = x186 * x190;
	const GEN_FLT x192 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qk * obj_qi / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x193 = x16 * x192;
	const GEN_FLT x194 = -x193;
	const GEN_FLT x195 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qj * obj_qi / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x196 = x27 * x195;
	const GEN_FLT x197 = x17 * x186;
	const GEN_FLT x198 = x196 + x23 * x189 + x26 * x197;
	const GEN_FLT x199 = x16 * x195;
	const GEN_FLT x200 = x22 * x20;
	const GEN_FLT x201 = x200 * x186;
	const GEN_FLT x202 = x27 * x192;
	const GEN_FLT x203 = x202 + x34 * x197 + x48 * x189;
	const GEN_FLT x204 = (x188 + 2 * x27 * x189 + x37 * x187) * sensor_x + (-x191 + x194 + x198) * sensor_y +
						 (x199 + x201 + x203) * sensor_z;
	const GEN_FLT x205 = x23 * x195;
	const GEN_FLT x206 = x20 * x17;
	const GEN_FLT x207 = x206 * x186;
	const GEN_FLT x208 = x16 * x189;
	const GEN_FLT x209 = x48 * x195;
	const GEN_FLT x210 = x23 * x192;
	const GEN_FLT x211 = x26 * x19;
	const GEN_FLT x212 = x209 + x210 + x211 * x186;
	const GEN_FLT x213 =
		(x191 + x193 + x198) * sensor_x + (x188 + 2 * x205 + x33 * x187) * sensor_y + (-x207 - x208 + x212) * sensor_z;
	const GEN_FLT x214 = -x199;
	const GEN_FLT x215 = x48 * x192;
	const GEN_FLT x216 =
		(x207 + x208 + x212) * sensor_y + (-x201 + x203 + x214) * sensor_x + (x188 + 2 * x215 + x25 * x187) * sensor_z;
	const GEN_FLT x217 = x70 + x53 * x204 + x54 * x216 + x59 * x213;
	const GEN_FLT x218 = x75 + x10 * x216 + x32 * x213 + x39 * x204;
	const GEN_FLT x219 = (-x41 * x217 + x79 * x218) * x82;
	const GEN_FLT x220 = x88 + x84 * x204 + x85 * x216 + x87 * x213;
	const GEN_FLT x221 = x93 * x218 + x94 * x217;
	const GEN_FLT x222 = -x102 * (x221 + x92 * x220) + x220 * x103;
	const GEN_FLT x223 = x222 * x105;
	const GEN_FLT x224 = -x220 * x108 + x221 * x130;
	const GEN_FLT x225 = x219 - x224 * x132;
	const GEN_FLT x226 = x223 * x115;
	const GEN_FLT x227 = x113 * (x226 - x223 * x114) + x222 * x142;
	const GEN_FLT x228 = x223 * x117 + x227 * x113;
	const GEN_FLT x229 =
		x219 -
		x150 * (x224 -
				x147 * (x126 * (x113 * (x228 + x113 * (x227 + x113 * (x226 + x223 * x120 - x223 * x141) + x223 * x121) +
										x223 * x122) +
								x222 * x145 + x223 * x118 + x228 * x113) +
						x225 * x139) +
				x223 * x129 + x225 * x137 + x228 * x148);
	const GEN_FLT x230 = x185 * obj_qj;
	const GEN_FLT x231 = x230 * x190;
	const GEN_FLT x232 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qk * obj_qj / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x233 = x16 * x232;
	const GEN_FLT x234 = x17 * x230;
	const GEN_FLT x235 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qj, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x236 = x205 + x26 * x234 + x27 * x235;
	const GEN_FLT x237 = x16 * x230;
	const GEN_FLT x238 = -x237;
	const GEN_FLT x239 = x230 * x206;
	const GEN_FLT x240 = x23 * x232;
	const GEN_FLT x241 = x240 + x211 * x230 + x48 * x235;
	const GEN_FLT x242 = (x214 - x239 + x241) * sensor_z + (x231 + x233 + x236) * sensor_x +
						 (x238 + 2 * x23 * x235 + x33 * x237) * sensor_y;
	const GEN_FLT x243 = -x233;
	const GEN_FLT x244 = x230 * x200;
	const GEN_FLT x245 = x16 * x235;
	const GEN_FLT x246 = x27 * x232;
	const GEN_FLT x247 = x209 + x246 + x34 * x234;
	const GEN_FLT x248 =
		(-x231 + x236 + x243) * sensor_y + (2 * x196 + x238 + x37 * x237) * sensor_x + (x244 + x245 + x247) * sensor_z;
	const GEN_FLT x249 = x48 * x232;
	const GEN_FLT x250 =
		(x238 + 2 * x249 + x25 * x237) * sensor_z + (x199 + x239 + x241) * sensor_y + (-x244 - x245 + x247) * sensor_x;
	const GEN_FLT x251 = x70 + x53 * x248 + x54 * x250 + x59 * x242;
	const GEN_FLT x252 = x75 + x10 * x250 + x32 * x242 + x39 * x248;
	const GEN_FLT x253 = (-x41 * x251 + x79 * x252) * x82;
	const GEN_FLT x254 = x88 + x84 * x248 + x85 * x250 + x87 * x242;
	const GEN_FLT x255 = x93 * x252 + x94 * x251;
	const GEN_FLT x256 = -x102 * (x255 + x92 * x254) + x254 * x103;
	const GEN_FLT x257 = x256 * x105;
	const GEN_FLT x258 = -x254 * x108 + x255 * x130;
	const GEN_FLT x259 = x253 - x258 * x132;
	const GEN_FLT x260 = x257 * x115;
	const GEN_FLT x261 = x113 * (x260 - x257 * x114) + x256 * x142;
	const GEN_FLT x262 = x257 * x117 + x261 * x113;
	const GEN_FLT x263 =
		x253 -
		x150 * (x258 -
				x147 * (x126 * (x113 * (x262 + x113 * (x261 + x113 * (x260 + x256 * x167 - x257 * x141) + x257 * x121) +
										x257 * x122) +
								x256 * x145 + x257 * x118 + x262 * x113) +
						x259 * x139) +
				x257 * x129 + x259 * x137 + x262 * x148);
	const GEN_FLT x264 = x185 * obj_qk;
	const GEN_FLT x265 = x264 * x190;
	const GEN_FLT x266 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qk, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x267 = x16 * x266;
	const GEN_FLT x268 = x26 * x264;
	const GEN_FLT x269 = x210 + x246 + x17 * x268;
	const GEN_FLT x270 = x16 * x264;
	const GEN_FLT x271 = -x270;
	const GEN_FLT x272 = x206 * x264;
	const GEN_FLT x273 = x249 + x19 * x268 + x23 * x266;
	const GEN_FLT x274 =
		(x265 + x267 + x269) * sensor_x + (2 * x240 + x271 + x33 * x270) * sensor_y + (x194 - x272 + x273) * sensor_z;
	const GEN_FLT x275 = x200 * x264;
	const GEN_FLT x276 = x215 + x27 * x266 + x34 * x17 * x264;
	const GEN_FLT x277 =
		(2 * x202 + x271 + x37 * x270) * sensor_x + (x233 + x275 + x276) * sensor_z + (-x265 - x267 + x269) * sensor_y;
	const GEN_FLT x278 = (x271 + x25 * x270 + 2 * x48 * x266) * sensor_z + (x193 + x272 + x273) * sensor_y +
						 (x243 - x275 + x276) * sensor_x;
	const GEN_FLT x279 = x70 + x53 * x277 + x54 * x278 + x59 * x274;
	const GEN_FLT x280 = x75 + x10 * x278 + x32 * x274 + x39 * x277;
	const GEN_FLT x281 = (-x41 * x279 + x79 * x280) * x82;
	const GEN_FLT x282 = x88 + x84 * x277 + x85 * x278 + x87 * x274;
	const GEN_FLT x283 = x93 * x280 + x94 * x279;
	const GEN_FLT x284 = -x102 * (x283 + x92 * x282) + x282 * x103;
	const GEN_FLT x285 = x284 * x105;
	const GEN_FLT x286 = -x282 * x108 + x283 * x130;
	const GEN_FLT x287 = x281 - x286 * x132;
	const GEN_FLT x288 = x285 * x115;
	const GEN_FLT x289 = x113 * (x288 - x285 * x114) + x284 * x142;
	const GEN_FLT x290 = x285 * x117 + x289 * x113;
	const GEN_FLT x291 =
		x281 -
		x150 * (x286 -
				x147 * (x126 * (x113 * (x290 + x113 * (x289 + x113 * (x288 + x285 * x120 - x285 * x141) + x285 * x121) +
										x285 * x122) +
								x284 * x145 + x285 * x118 + x290 * x113) +
						x287 * x139) +
				x285 * x129 + x287 * x137 + x290 * x148);
	*(out++) = x151 + x151 * x152;
	*(out++) = x170 + x170 * x152;
	*(out++) = x184 + x184 * x152;
	*(out++) = x229 + x229 * x152;
	*(out++) = x263 + x263 * x152;
	*(out++) = x291 + x291 * x152;
}

// Jacobian of reproject_axis_y_gen2 wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_reproject_axis_y_gen2_jac_sensor_pt_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
																	  const FLT *sensor_pt,
																	  const LinmathAxisAnglePose *lh_p,
																	  const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x5 = cos(x1);
	const GEN_FLT x6 = 1 - x5;
	const GEN_FLT x7 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (1));
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = -x3 + x9;
	const GEN_FLT x11 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x12 = cos(x11);
	const GEN_FLT x13 = 1 - x12;
	const GEN_FLT x14 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x15 = x12 + pow(x14, 2) * x13;
	const GEN_FLT x16 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x17 = x13 * x16;
	const GEN_FLT x18 = x14 * x17;
	const GEN_FLT x19 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x20 = x19 * x17;
	const GEN_FLT x21 = sin(x11);
	const GEN_FLT x22 = x21 * x16;
	const GEN_FLT x23 = -x22;
	const GEN_FLT x24 = x18 + x23;
	const GEN_FLT x25 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x26 = x25 * x17;
	const GEN_FLT x27 = 2 * x18 * sensor_x + (x20 + x24) * sensor_y + (x18 + x22 + x26) * sensor_z;
	const GEN_FLT x28 = x15 + x27;
	const GEN_FLT x29 = x25 * x21;
	const GEN_FLT x30 = x14 * x13;
	const GEN_FLT x31 = x30 * x19;
	const GEN_FLT x32 = x29 + x31;
	const GEN_FLT x33 = x20 + x22;
	const GEN_FLT x34 = 2 * x20 * sensor_y + (x18 + x33) * sensor_x + (x20 + x23 + x26) * sensor_z;
	const GEN_FLT x35 = x32 + x34;
	const GEN_FLT x36 = x2 * x7;
	const GEN_FLT x37 = x0 * x4 * x6;
	const GEN_FLT x38 = x36 + x37;
	const GEN_FLT x39 = x21 * x19;
	const GEN_FLT x40 = x30 * x25;
	const GEN_FLT x41 = -x39 + x40;
	const GEN_FLT x42 = 2 * x26 * sensor_z + (x24 + x26) * sensor_x + (x26 + x33) * sensor_y;
	const GEN_FLT x43 = x41 + x42;
	const GEN_FLT x44 = x5 + pow(x4, 2) * x6;
	const GEN_FLT x45 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x46 = x6 * x45;
	const GEN_FLT x47 = x7 * x46;
	const GEN_FLT x48 = x4 * x46;
	const GEN_FLT x49 = x2 * x45;
	const GEN_FLT x50 = -x49;
	const GEN_FLT x51 = x48 + x50;
	const GEN_FLT x52 = x39 + x40;
	const GEN_FLT x53 = -x29 + x31;
	const GEN_FLT x54 = obj_px + x15 * sensor_x + x52 * sensor_z + x53 * sensor_y;
	const GEN_FLT x55 = x21 * x14;
	const GEN_FLT x56 = x25 * x13 * x19;
	const GEN_FLT x57 = -x55 + x56;
	const GEN_FLT x58 = x12 + x13 * pow(x19, 2);
	const GEN_FLT x59 = obj_py + x32 * sensor_x + x57 * sensor_z + x58 * sensor_y;
	const GEN_FLT x60 = x0 * x46;
	const GEN_FLT x61 = x49 + x60;
	const GEN_FLT x62 = x55 + x56;
	const GEN_FLT x63 = x12 + pow(x25, 2) * x13;
	const GEN_FLT x64 = obj_pz + x41 * sensor_x + x62 * sensor_y + x63 * sensor_z;
	const GEN_FLT x65 = 2 * x64 * x48 + (x47 + x51) * x54 + (x48 + x61) * x59;
	const GEN_FLT x66 = x65 + x28 * x10 + x35 * x38 + x43 * x44;
	const GEN_FLT x67 = x3 + x9;
	const GEN_FLT x68 = x2 * x4;
	const GEN_FLT x69 = x0 * x8;
	const GEN_FLT x70 = -x68 + x69;
	const GEN_FLT x71 = x5 + x6 * pow(x7, 2);
	const GEN_FLT x72 = lh_px + x64 * x67 + x70 * x59 + x71 * x54;
	const GEN_FLT x73 = pow(x72, -1);
	const GEN_FLT x74 = 2 * x54 * x47 + x59 * (x47 + x50 + x60) + x64 * (x47 + x48 + x49);
	const GEN_FLT x75 = x74 + x67 * x43 + x70 * x35 + x71 * x28;
	const GEN_FLT x76 = lh_pz + x54 * x10 + x59 * x38 + x64 * x44;
	const GEN_FLT x77 = pow(x72, 2);
	const GEN_FLT x78 = x76 / x77;
	const GEN_FLT x79 = x77 + pow(x76, 2);
	const GEN_FLT x80 = pow(x79, -1);
	const GEN_FLT x81 = x80 * x77;
	const GEN_FLT x82 = (-x73 * x66 + x78 * x75) * x81;
	const GEN_FLT x83 = -x36 + x37;
	const GEN_FLT x84 = x5 + pow(x0, 2) * x6;
	const GEN_FLT x85 = x68 + x69;
	const GEN_FLT x86 = lh_py + x83 * x64 + x84 * x59 + x85 * x54;
	const GEN_FLT x87 = 0.523598775598299 - tilt_1;
	const GEN_FLT x88 = cos(x87);
	const GEN_FLT x89 = pow(x88, -1);
	const GEN_FLT x90 = pow(x86, 2);
	const GEN_FLT x91 = x79 + x90;
	const GEN_FLT x92 = x89 / sqrt(x91);
	const GEN_FLT x93 = asin(x86 * x92);
	const GEN_FLT x94 = 8.0108022e-06 * x93;
	const GEN_FLT x95 = -8.0108022e-06 - x94;
	const GEN_FLT x96 = 0.0028679863 + x93 * x95;
	const GEN_FLT x97 = 5.3685255e-06 + x93 * x96;
	const GEN_FLT x98 = 0.0076069798 + x93 * x97;
	const GEN_FLT x99 = x93 * x98;
	const GEN_FLT x100 = -8.0108022e-06 - 1.60216044e-05 * x93;
	const GEN_FLT x101 = x96 + x93 * x100;
	const GEN_FLT x102 = x97 + x93 * x101;
	const GEN_FLT x103 = x98 + x93 * x102;
	const GEN_FLT x104 = x99 + x93 * x103;
	const GEN_FLT x105 = sin(x87);
	const GEN_FLT x106 = tan(x87);
	const GEN_FLT x107 = x106 / sqrt(x79);
	const GEN_FLT x108 = -x86 * x107;
	const GEN_FLT x109 = atan2(-x76, x72);
	const GEN_FLT x110 = ogeeMag_1 + x109 - asin(x108);
	const GEN_FLT x111 = curve_1 + sin(x110) * ogeePhase_1;
	const GEN_FLT x112 = x105 * x111;
	const GEN_FLT x113 = x88 + x104 * x112;
	const GEN_FLT x114 = pow(x113, -1);
	const GEN_FLT x115 = pow(x93, 2);
	const GEN_FLT x116 = x111 * x115;
	const GEN_FLT x117 = x114 * x116;
	const GEN_FLT x118 = x108 + x98 * x117;
	const GEN_FLT x119 = pow(1 - pow(x118, 2), -1.0 / 2.0);
	const GEN_FLT x120 = pow(1 - x90 / (pow(x88, 2) * x91), -1.0 / 2.0);
	const GEN_FLT x121 = 2 * x60 * x59 + (x51 + x60) * x64 + (x47 + x61) * x54;
	const GEN_FLT x122 = x121 + x83 * x43 + x84 * x35 + x85 * x28;
	const GEN_FLT x123 = 2 * x86;
	const GEN_FLT x124 = 2 * x72;
	const GEN_FLT x125 = 2 * x76;
	const GEN_FLT x126 = x66 * x125 + x75 * x124;
	const GEN_FLT x127 = (1.0 / 2.0) * x86;
	const GEN_FLT x128 = x89 * x127 / pow(x91, 3.0 / 2.0);
	const GEN_FLT x129 = -x128 * (x126 + x123 * x122) + x92 * x122;
	const GEN_FLT x130 = x120 * x129;
	const GEN_FLT x131 = 2 * x99 * x111 * x114;
	const GEN_FLT x132 = x106 * x127 / pow(x79, 3.0 / 2.0);
	const GEN_FLT x133 = -x107 * x122 + x126 * x132;
	const GEN_FLT x134 = pow(1 - x80 * x90 * pow(x106, 2), -1.0 / 2.0);
	const GEN_FLT x135 = cos(x110) * ogeePhase_1;
	const GEN_FLT x136 = x135 * (x82 - x133 * x134);
	const GEN_FLT x137 = x98 * x114 * x115;
	const GEN_FLT x138 = x105 * x104;
	const GEN_FLT x139 = 2.40324066e-05 * x93;
	const GEN_FLT x140 = x95 * x130;
	const GEN_FLT x141 = x96 * x120;
	const GEN_FLT x142 = x129 * x141 + x93 * (x140 - x94 * x130);
	const GEN_FLT x143 = x102 * x120;
	const GEN_FLT x144 = x93 * x142 + x97 * x130;
	const GEN_FLT x145 = x98 * x116 / pow(x113, 2);
	const GEN_FLT x146 =
		x82 - x119 * (x133 + x117 * x144 + x130 * x131 + x137 * x136 -
					  x145 * (x112 * (x103 * x130 + x93 * x144 +
									  x93 * (x144 + x129 * x143 +
											 x93 * (x142 + x101 * x130 + x93 * (x140 + x100 * x130 - x130 * x139))) +
									  x98 * x130) +
							  x138 * x136));
	const GEN_FLT x147 = cos(gibPhase_1 + x109 - asin(x118)) * gibMag_1;
	const GEN_FLT x148 = x27 + x53;
	const GEN_FLT x149 = x34 + x58;
	const GEN_FLT x150 = x42 + x62;
	const GEN_FLT x151 = x65 + x10 * x148 + x38 * x149 + x44 * x150;
	const GEN_FLT x152 = x74 + x67 * x150 + x70 * x149 + x71 * x148;
	const GEN_FLT x153 = (-x73 * x151 + x78 * x152) * x81;
	const GEN_FLT x154 = x121 + x83 * x150 + x84 * x149 + x85 * x148;
	const GEN_FLT x155 = x124 * x152 + x125 * x151;
	const GEN_FLT x156 = -x128 * (x155 + x123 * x154) + x92 * x154;
	const GEN_FLT x157 = x120 * x156;
	const GEN_FLT x158 = -x107 * x154 + x132 * x155;
	const GEN_FLT x159 = x153 - x134 * x158;
	const GEN_FLT x160 = x137 * x135;
	const GEN_FLT x161 = x138 * x135;
	const GEN_FLT x162 = x95 * x157;
	const GEN_FLT x163 = x141 * x156 + x93 * (x162 - x94 * x157);
	const GEN_FLT x164 = x93 * x163 + x97 * x157;
	const GEN_FLT x165 =
		x153 - x119 * (x158 + x117 * x164 + x131 * x157 -
					   x145 * (x112 * (x103 * x157 + x93 * x164 +
									   x93 * (x164 + x143 * x156 +
											  x93 * (x163 + x101 * x157 + x93 * (x162 + x100 * x157 - x139 * x157))) +
									   x98 * x157) +
							   x161 * x159) +
					   x160 * x159);
	const GEN_FLT x166 = x34 + x57;
	const GEN_FLT x167 = x27 + x52;
	const GEN_FLT x168 = x42 + x63;
	const GEN_FLT x169 = x65 + x10 * x167 + x38 * x166 + x44 * x168;
	const GEN_FLT x170 = x74 + x67 * x168 + x70 * x166 + x71 * x167;
	const GEN_FLT x171 = (-x73 * x169 + x78 * x170) * x81;
	const GEN_FLT x172 = x121 + x83 * x168 + x84 * x166 + x85 * x167;
	const GEN_FLT x173 = x124 * x170 + x125 * x169;
	const GEN_FLT x174 = -x128 * (x173 + x123 * x172) + x92 * x172;
	const GEN_FLT x175 = x120 * x174;
	const GEN_FLT x176 = -x107 * x172 + x173 * x132;
	const GEN_FLT x177 = x171 - x176 * x134;
	const GEN_FLT x178 = x95 * x175;
	const GEN_FLT x179 = x174 * x141 + x93 * (x178 - x94 * x175);
	const GEN_FLT x180 = x93 * x179 + x97 * x175;
	const GEN_FLT x181 =
		x171 - x119 * (x176 + x117 * x180 -
					   x145 * (x112 * (x103 * x175 + x93 * x180 +
									   x93 * (x180 + x174 * x143 +
											  x93 * (x179 + x101 * x175 + x93 * (x178 + x100 * x175 - x175 * x139))) +
									   x98 * x175) +
							   x161 * x177) +
					   x160 * x177 + x175 * x131);
	*(out++) = x146 + x146 * x147;
	*(out++) = x165 + x165 * x147;
	*(out++) = x181 + x181 * x147;
}

// Jacobian of reproject_axis_y_gen2 wrt [lh_px, lh_py, lh_pz, lh_qi, lh_qj, lh_qk]
static inline void gen_reproject_axis_y_gen2_jac_lh_p_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
																 const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
																 const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = pow(lh_qk, 2);
	const GEN_FLT x2 = pow(lh_qj, 2);
	const GEN_FLT x3 = pow(lh_qi, 2);
	const GEN_FLT x4 = x1 + x2 + x3;
	const GEN_FLT x5 = sqrt(x4);
	const GEN_FLT x6 = sin(x5);
	const GEN_FLT x7 = x0 * x6;
	const GEN_FLT x8 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x9 = cos(x5);
	const GEN_FLT x10 = 1 - x9;
	const GEN_FLT x11 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (1));
	const GEN_FLT x12 = x11 * x10;
	const GEN_FLT x13 = x8 * x12;
	const GEN_FLT x14 = x13 + x7;
	const GEN_FLT x15 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x16 = sin(x15);
	const GEN_FLT x17 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x20 = cos(x15);
	const GEN_FLT x21 = 1 - x20;
	const GEN_FLT x22 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = x23 * x19;
	const GEN_FLT x25 = x19 * x16;
	const GEN_FLT x26 = x23 * x17;
	const GEN_FLT x27 =
		obj_pz + (x20 + pow(x22, 2) * x21) * sensor_z + (x18 + x24) * sensor_y + (-x25 + x26) * sensor_x;
	const GEN_FLT x28 = x6 * x8;
	const GEN_FLT x29 = x0 * x10;
	const GEN_FLT x30 = x29 * x11;
	const GEN_FLT x31 = -x28 + x30;
	const GEN_FLT x32 = x22 * x16;
	const GEN_FLT x33 = x21 * x17;
	const GEN_FLT x34 = x33 * x19;
	const GEN_FLT x35 =
		obj_py + (x20 + x21 * pow(x19, 2)) * sensor_y + (-x18 + x24) * sensor_z + (x32 + x34) * sensor_x;
	const GEN_FLT x36 =
		obj_px + (x20 + x21 * pow(x17, 2)) * sensor_x + (x25 + x26) * sensor_z + (-x32 + x34) * sensor_y;
	const GEN_FLT x37 = pow(x11, 2);
	const GEN_FLT x38 = x9 + x37 * x10;
	const GEN_FLT x39 = lh_px + x27 * x14 + x31 * x35 + x36 * x38;
	const GEN_FLT x40 = pow(x39, -1);
	const GEN_FLT x41 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x42 = x8 * x10;
	const GEN_FLT x43 = x41 * x42;
	const GEN_FLT x44 = x41 * x12;
	const GEN_FLT x45 = x6 * x41;
	const GEN_FLT x46 = -x45;
	const GEN_FLT x47 = x44 + x46;
	const GEN_FLT x48 = x41 * x29;
	const GEN_FLT x49 = x43 + x48;
	const GEN_FLT x50 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x51 = x50 * x33;
	const GEN_FLT x52 = x50 * x16;
	const GEN_FLT x53 = -x52;
	const GEN_FLT x54 = x50 * x21 * x19;
	const GEN_FLT x55 = x51 + x54;
	const GEN_FLT x56 = x50 * x23;
	const GEN_FLT x57 = x51 + x56;
	const GEN_FLT x58 = 2 * x51 * sensor_x + (x52 + x57) * sensor_z + (x53 + x55) * sensor_y;
	const GEN_FLT x59 = x13 - x7;
	const GEN_FLT x60 = x6 * x11;
	const GEN_FLT x61 = x8 * x29;
	const GEN_FLT x62 = x60 + x61;
	const GEN_FLT x63 = x54 + x56;
	const GEN_FLT x64 = 2 * x54 * sensor_y + (x52 + x55) * sensor_x + (x53 + x63) * sensor_z;
	const GEN_FLT x65 = pow(x8, 2);
	const GEN_FLT x66 = x9 + x65 * x10;
	const GEN_FLT x67 = 2 * x56 * sensor_z + (x53 + x57) * sensor_x + (x52 + x63) * sensor_y;
	const GEN_FLT x68 = x58 * x59 + x64 * x62 + x67 * x66;
	const GEN_FLT x69 = x68 + 2 * x43 * x27 + (x43 + x47) * x36 + (x45 + x49) * x35;
	const GEN_FLT x70 = -x69 * x40;
	const GEN_FLT x71 = x44 + x45;
	const GEN_FLT x72 = x58 * x38 + x64 * x31 + x67 * x14;
	const GEN_FLT x73 = x72 + 2 * x44 * x36 + (x47 + x48) * x35 + (x43 + x71) * x27;
	const GEN_FLT x74 = 1 + x73;
	const GEN_FLT x75 = pow(x39, 2);
	const GEN_FLT x76 = lh_pz + x59 * x36 + x62 * x35 + x66 * x27;
	const GEN_FLT x77 = x76 / x75;
	const GEN_FLT x78 = x75 + pow(x76, 2);
	const GEN_FLT x79 = pow(x78, -1);
	const GEN_FLT x80 = x79 * x75;
	const GEN_FLT x81 = x80 * (x70 + x74 * x77);
	const GEN_FLT x82 = -x60 + x61;
	const GEN_FLT x83 = pow(x0, 2);
	const GEN_FLT x84 = x9 + x83 * x10;
	const GEN_FLT x85 = x28 + x30;
	const GEN_FLT x86 = lh_py + x82 * x27 + x84 * x35 + x85 * x36;
	const GEN_FLT x87 = 0.523598775598299 - tilt_1;
	const GEN_FLT x88 = cos(x87);
	const GEN_FLT x89 = pow(x88, -1);
	const GEN_FLT x90 = pow(x86, 2);
	const GEN_FLT x91 = x78 + x90;
	const GEN_FLT x92 = x89 / sqrt(x91);
	const GEN_FLT x93 = asin(x86 * x92);
	const GEN_FLT x94 = 8.0108022e-06 * x93;
	const GEN_FLT x95 = -8.0108022e-06 - x94;
	const GEN_FLT x96 = 0.0028679863 + x93 * x95;
	const GEN_FLT x97 = 5.3685255e-06 + x93 * x96;
	const GEN_FLT x98 = 0.0076069798 + x93 * x97;
	const GEN_FLT x99 = pow(x93, 2);
	const GEN_FLT x100 = tan(x87);
	const GEN_FLT x101 = x100 / sqrt(x78);
	const GEN_FLT x102 = -x86 * x101;
	const GEN_FLT x103 = atan2(-x76, x39);
	const GEN_FLT x104 = ogeeMag_1 + x103 - asin(x102);
	const GEN_FLT x105 = curve_1 + sin(x104) * ogeePhase_1;
	const GEN_FLT x106 = x93 * x98;
	const GEN_FLT x107 = -8.0108022e-06 - 1.60216044e-05 * x93;
	const GEN_FLT x108 = x96 + x93 * x107;
	const GEN_FLT x109 = x97 + x93 * x108;
	const GEN_FLT x110 = x98 + x93 * x109;
	const GEN_FLT x111 = x106 + x93 * x110;
	const GEN_FLT x112 = sin(x87);
	const GEN_FLT x113 = x105 * x112;
	const GEN_FLT x114 = x88 + x111 * x113;
	const GEN_FLT x115 = pow(x114, -1);
	const GEN_FLT x116 = x105 * x115;
	const GEN_FLT x117 = x99 * x116;
	const GEN_FLT x118 = x102 + x98 * x117;
	const GEN_FLT x119 = pow(1 - pow(x118, 2), -1.0 / 2.0);
	const GEN_FLT x120 = pow(1 - x90 / (pow(x88, 2) * x91), -1.0 / 2.0);
	const GEN_FLT x121 = x82 * x67 + x84 * x64 + x85 * x58;
	const GEN_FLT x122 = x121 + 2 * x48 * x35 + (x46 + x49) * x27 + (x48 + x71) * x36;
	const GEN_FLT x123 = 2 * x86;
	const GEN_FLT x124 = x123 * x122;
	const GEN_FLT x125 = 2 * x39;
	const GEN_FLT x126 = 2 * x76;
	const GEN_FLT x127 = x69 * x126;
	const GEN_FLT x128 = x127 + x74 * x125;
	const GEN_FLT x129 = (1.0 / 2.0) * x86;
	const GEN_FLT x130 = x89 * x129 / pow(x91, 3.0 / 2.0);
	const GEN_FLT x131 = x92 * x122;
	const GEN_FLT x132 = x120 * (x131 - (x124 + x128) * x130);
	const GEN_FLT x133 = 2 * x106 * x116;
	const GEN_FLT x134 = -x101 * x122;
	const GEN_FLT x135 = x100 * x129 / pow(x78, 3.0 / 2.0);
	const GEN_FLT x136 = x134 + x128 * x135;
	const GEN_FLT x137 = pow(1 - x79 * x90 * pow(x100, 2), -1.0 / 2.0);
	const GEN_FLT x138 = x81 - x137 * x136;
	const GEN_FLT x139 = cos(x104) * ogeePhase_1;
	const GEN_FLT x140 = x99 * x98;
	const GEN_FLT x141 = x115 * x140;
	const GEN_FLT x142 = x139 * x141;
	const GEN_FLT x143 = x111 * x112;
	const GEN_FLT x144 = x139 * x143;
	const GEN_FLT x145 = x95 * x132;
	const GEN_FLT x146 = 2.40324066e-05 * x93;
	const GEN_FLT x147 = x93 * (x145 - x94 * x132) + x96 * x132;
	const GEN_FLT x148 = x93 * x147 + x97 * x132;
	const GEN_FLT x149 = x105 * x140 / pow(x114, 2);
	const GEN_FLT x150 =
		x81 - x119 * (x136 + x117 * x148 + x133 * x132 + x138 * x142 -
					  x149 * (x113 * (x110 * x132 + x93 * x148 +
									  x93 * (x148 + x109 * x132 +
											 x93 * (x147 + x108 * x132 + x93 * (x145 + x107 * x132 - x132 * x146))) +
									  x98 * x132) +
							  x138 * x144));
	const GEN_FLT x151 = cos(gibPhase_1 + x103 - asin(x118)) * gibMag_1;
	const GEN_FLT x152 = x73 * x77;
	const GEN_FLT x153 = x80 * (x152 + x70);
	const GEN_FLT x154 = 1 + x122;
	const GEN_FLT x155 = x73 * x125;
	const GEN_FLT x156 = x127 + x155;
	const GEN_FLT x157 = -x130 * (x156 + x123 * x154) + x92 * x154;
	const GEN_FLT x158 = x120 * x157;
	const GEN_FLT x159 = -x101 * x154 + x135 * x156;
	const GEN_FLT x160 = x153 - x137 * x159;
	const GEN_FLT x161 = x95 * x120;
	const GEN_FLT x162 = x161 * x157;
	const GEN_FLT x163 = x107 * x120;
	const GEN_FLT x164 = x93 * (x162 - x94 * x158) + x96 * x158;
	const GEN_FLT x165 = x109 * x120;
	const GEN_FLT x166 = x97 * x120;
	const GEN_FLT x167 = x166 * x157 + x93 * x164;
	const GEN_FLT x168 = x98 * x120;
	const GEN_FLT x169 =
		x153 - x119 * (x159 + x117 * x167 + x133 * x158 -
					   x149 * (x113 * (x110 * x158 + x168 * x157 + x93 * x167 +
									   x93 * (x167 + x165 * x157 +
											  x93 * (x164 + x108 * x158 + x93 * (x162 - x146 * x158 + x163 * x157)))) +
							   x160 * x144) +
					   x160 * x142);
	const GEN_FLT x170 = 1 + x69;
	const GEN_FLT x171 = x80 * (x152 - x40 * x170);
	const GEN_FLT x172 = x155 + x126 * x170;
	const GEN_FLT x173 = x131 - (x124 + x172) * x130;
	const GEN_FLT x174 = x120 * x173;
	const GEN_FLT x175 = x134 + x172 * x135;
	const GEN_FLT x176 = x171 - x175 * x137;
	const GEN_FLT x177 = x161 * x173;
	const GEN_FLT x178 = x93 * (x177 - x94 * x174) + x96 * x174;
	const GEN_FLT x179 = x166 * x173 + x93 * x178;
	const GEN_FLT x180 =
		x171 - x119 * (x175 + x117 * x179 -
					   x149 * (x113 * (x110 * x174 + x168 * x173 + x93 * x179 +
									   x93 * (x179 + x165 * x173 +
											  x93 * (x178 + x108 * x174 + x93 * (x177 + x163 * x173 - x174 * x146)))) +
							   x176 * x144) +
					   x174 * x133 + x176 * x142);
	const GEN_FLT x181 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qj * lh_qi / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x182 = x6 * x181;
	const GEN_FLT x183 = -x182;
	const GEN_FLT x184 = pow(x5, -1);
	const GEN_FLT x185 = x184 * lh_qi;
	const GEN_FLT x186 = x0 * x9;
	const GEN_FLT x187 = x186 * x185;
	const GEN_FLT x188 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qi, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x189 = x28 * x11;
	const GEN_FLT x190 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qk * lh_qi / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x191 = x12 * x190;
	const GEN_FLT x192 = x191 + x189 * x185 + x42 * x188;
	const GEN_FLT x193 = x6 * x188;
	const GEN_FLT x194 = x9 * x11;
	const GEN_FLT x195 = x185 * x194;
	const GEN_FLT x196 = x42 * x181;
	const GEN_FLT x197 = x29 * x190;
	const GEN_FLT x198 = x8 * x7;
	const GEN_FLT x199 = x196 + x197 + x185 * x198;
	const GEN_FLT x200 = x6 * x185;
	const GEN_FLT x201 = -x200;
	const GEN_FLT x202 = x42 * x190;
	const GEN_FLT x203 =
		x68 + x27 * (x201 + 2 * x202 + x65 * x200) + x35 * (x193 + x195 + x199) + x36 * (x183 - x187 + x192);
	const GEN_FLT x204 = x6 * x190;
	const GEN_FLT x205 = -x204;
	const GEN_FLT x206 = x8 * x9;
	const GEN_FLT x207 = x206 * x185;
	const GEN_FLT x208 = x12 * x181;
	const GEN_FLT x209 = x7 * x11;
	const GEN_FLT x210 = x208 + x209 * x185 + x29 * x188;
	const GEN_FLT x211 =
		x72 + x27 * (x182 + x187 + x192) + x35 * (x205 - x207 + x210) + x36 * (x201 + 2 * x12 * x188 + x37 * x200);
	const GEN_FLT x212 = (-x40 * x203 + x77 * x211) * x80;
	const GEN_FLT x213 = x29 * x181;
	const GEN_FLT x214 =
		x121 + x27 * (-x193 - x195 + x199) + x35 * (x201 + 2 * x213 + x83 * x200) + x36 * (x204 + x207 + x210);
	const GEN_FLT x215 = x203 * x126 + x211 * x125;
	const GEN_FLT x216 = -x130 * (x215 + x214 * x123) + x92 * x214;
	const GEN_FLT x217 = x216 * x120;
	const GEN_FLT x218 = -x214 * x101 + x215 * x135;
	const GEN_FLT x219 = x139 * (x212 - x218 * x137);
	const GEN_FLT x220 = x95 * x217;
	const GEN_FLT x221 = x93 * (x220 - x94 * x217) + x96 * x217;
	const GEN_FLT x222 = x93 * x221 + x97 * x217;
	const GEN_FLT x223 =
		x212 - x119 * (x218 -
					   x149 * (x113 * (x216 * x168 + x217 * x110 + x93 * x222 +
									   x93 * (x222 + x217 * x109 +
											  x93 * (x221 + x217 * x108 + x93 * (x220 + x216 * x163 - x217 * x146)))) +
							   x219 * x143) +
					   x217 * x133 + x219 * x141 + x222 * x117);
	const GEN_FLT x224 = x184 * lh_qj;
	const GEN_FLT x225 = x224 * x194;
	const GEN_FLT x226 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qk * lh_qj / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x227 = x29 * x226;
	const GEN_FLT x228 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qj, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x229 = x227 + x224 * x198 + x42 * x228;
	const GEN_FLT x230 = x6 * x228;
	const GEN_FLT x231 = x224 * x186;
	const GEN_FLT x232 = x12 * x226;
	const GEN_FLT x233 = x11 * x224;
	const GEN_FLT x234 = x196 + x232 + x28 * x233;
	const GEN_FLT x235 = x6 * x224;
	const GEN_FLT x236 = -x235;
	const GEN_FLT x237 = x42 * x226;
	const GEN_FLT x238 =
		x68 + x27 * (x236 + 2 * x237 + x65 * x235) + x35 * (x182 + x225 + x229) + x36 * (-x230 - x231 + x234);
	const GEN_FLT x239 = x6 * x226;
	const GEN_FLT x240 = -x239;
	const GEN_FLT x241 = x206 * x224;
	const GEN_FLT x242 = x213 + x12 * x228 + x7 * x233;
	const GEN_FLT x243 =
		x72 + x27 * (x230 + x231 + x234) + x35 * (x240 - x241 + x242) + x36 * (2 * x208 + x236 + x37 * x235);
	const GEN_FLT x244 = (-x40 * x238 + x77 * x243) * x80;
	const GEN_FLT x245 =
		x121 + x27 * (x183 - x225 + x229) + x35 * (x236 + 2 * x29 * x228 + x83 * x235) + x36 * (x239 + x241 + x242);
	const GEN_FLT x246 = x238 * x126 + x243 * x125;
	const GEN_FLT x247 = -x130 * (x246 + x245 * x123) + x92 * x245;
	const GEN_FLT x248 = x247 * x120;
	const GEN_FLT x249 = -x245 * x101 + x246 * x135;
	const GEN_FLT x250 = x244 - x249 * x137;
	const GEN_FLT x251 = x247 * x161;
	const GEN_FLT x252 = x93 * (x251 - x94 * x248) + x96 * x248;
	const GEN_FLT x253 = x247 * x166 + x93 * x252;
	const GEN_FLT x254 =
		x244 - x119 * (x249 -
					   x149 * (x113 * (x247 * x168 + x248 * x110 + x93 * x253 +
									   x93 * (x253 + x247 * x165 +
											  x93 * (x252 + x248 * x108 + x93 * (x251 + x247 * x163 - x248 * x146)))) +
							   x250 * x144) +
					   x248 * x133 + x250 * x142 + x253 * x117);
	const GEN_FLT x255 = x184 * lh_qk;
	const GEN_FLT x256 = x255 * x186;
	const GEN_FLT x257 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qk, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x258 = x202 + x12 * x257 + x255 * x189;
	const GEN_FLT x259 = x255 * x194;
	const GEN_FLT x260 = x237 + x255 * x198 + x29 * x257;
	const GEN_FLT x261 = x6 * x255;
	const GEN_FLT x262 = -x261;
	const GEN_FLT x263 =
		x68 + x27 * (x262 + 2 * x42 * x257 + x65 * x261) + x35 * (x204 + x259 + x260) + x36 * (x240 - x256 + x258);
	const GEN_FLT x264 = x6 * x257;
	const GEN_FLT x265 = x206 * x255;
	const GEN_FLT x266 = x197 + x232 + x209 * x255;
	const GEN_FLT x267 =
		x72 + x27 * (x239 + x256 + x258) + x35 * (-x264 - x265 + x266) + x36 * (2 * x191 + x262 + x37 * x261);
	const GEN_FLT x268 = (-x40 * x263 + x77 * x267) * x80;
	const GEN_FLT x269 =
		x121 + x27 * (x205 - x259 + x260) + x35 * (2 * x227 + x262 + x83 * x261) + x36 * (x264 + x265 + x266);
	const GEN_FLT x270 = x263 * x126 + x267 * x125;
	const GEN_FLT x271 = x120 * (-x130 * (x270 + x269 * x123) + x92 * x269);
	const GEN_FLT x272 = -x269 * x101 + x270 * x135;
	const GEN_FLT x273 = x268 - x272 * x137;
	const GEN_FLT x274 = x95 * x271;
	const GEN_FLT x275 = x93 * (x274 - x94 * x271) + x96 * x271;
	const GEN_FLT x276 = x93 * x275 + x97 * x271;
	const GEN_FLT x277 =
		x268 - x119 * (x272 -
					   x149 * (x113 * (x271 * x110 + x93 * x276 +
									   x93 * (x276 + x271 * x109 +
											  x93 * (x275 + x271 * x108 + x93 * (x274 + x271 * x107 - x271 * x146))) +
									   x98 * x271) +
							   x273 * x144) +
					   x271 * x133 + x273 * x142 + x276 * x117);
	*(out++) = x150 + x150 * x151;
	*(out++) = x169 + x169 * x151;
	*(out++) = x180 + x180 * x151;
	*(out++) = x223 + x223 * x151;
	*(out++) = x254 + x254 * x151;
	*(out++) = x277 + x277 * x151;
}

// Jacobian of reproject_axis_y_gen2 wrt [phase_1, tilt_1, curve_1, gibPhase_1, gibMag_1, ogeeMag_1, ogeePhase_1]
static inline void gen_reproject_axis_y_gen2_jac_bsc1_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
																 const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
																 const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = cos(x1);
	const GEN_FLT x3 = 1 - x2;
	const GEN_FLT x4 = x2 + pow(x0, 2) * x3;
	const GEN_FLT x5 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x6 = sin(x5);
	const GEN_FLT x7 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1));
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (0));
	const GEN_FLT x10 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x11 = cos(x5);
	const GEN_FLT x12 = 1 - x11;
	const GEN_FLT x13 = x12 * x10;
	const GEN_FLT x14 = x9 * x13;
	const GEN_FLT x15 = x6 * x10;
	const GEN_FLT x16 = x7 * x12;
	const GEN_FLT x17 = x9 * x16;
	const GEN_FLT x18 = obj_pz + (x11 + pow(x9, 2) * x12) * sensor_z + (-x15 + x17) * sensor_x + (x14 + x8) * sensor_y;
	const GEN_FLT x19 = x6 * x9;
	const GEN_FLT x20 = x10 * x16;
	const GEN_FLT x21 = obj_py + (x11 + x12 * pow(x10, 2)) * sensor_y + (x19 + x20) * sensor_x + (x14 - x8) * sensor_z;
	const GEN_FLT x22 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (1));
	const GEN_FLT x23 = sin(x1);
	const GEN_FLT x24 = x22 * x23;
	const GEN_FLT x25 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (0));
	const GEN_FLT x26 = x0 * x3 * x25;
	const GEN_FLT x27 = x24 + x26;
	const GEN_FLT x28 = obj_px + (x11 + pow(x7, 2) * x12) * sensor_x + (x15 + x17) * sensor_z + (-x19 + x20) * sensor_y;
	const GEN_FLT x29 = x25 * x23;
	const GEN_FLT x30 = x3 * x22;
	const GEN_FLT x31 = x0 * x30;
	const GEN_FLT x32 = -x29 + x31;
	const GEN_FLT x33 = lh_pz + x21 * x27 + x32 * x28 + x4 * x18;
	const GEN_FLT x34 = x29 + x31;
	const GEN_FLT x35 = x0 * x23;
	const GEN_FLT x36 = x30 * x25;
	const GEN_FLT x37 = -x35 + x36;
	const GEN_FLT x38 = x2 + x3 * pow(x22, 2);
	const GEN_FLT x39 = lh_px + x34 * x18 + x37 * x21 + x38 * x28;
	const GEN_FLT x40 = pow(x39, 2);
	const GEN_FLT x41 = x40 + pow(x33, 2);
	const GEN_FLT x42 = pow(x41, -1);
	const GEN_FLT x43 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x44 = x3 * x43;
	const GEN_FLT x45 = x0 * x44;
	const GEN_FLT x46 = x44 * x22;
	const GEN_FLT x47 = x43 * x23;
	const GEN_FLT x48 = -x47;
	const GEN_FLT x49 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x50 = x6 * x49;
	const GEN_FLT x51 = x49 * x13;
	const GEN_FLT x52 = x49 * x16;
	const GEN_FLT x53 = x51 + x52;
	const GEN_FLT x54 = -x50;
	const GEN_FLT x55 = x9 * x49 * x12;
	const GEN_FLT x56 = x51 + x55;
	const GEN_FLT x57 = 2 * x51 * sensor_y + (x50 + x53) * sensor_x + (x54 + x56) * sensor_z;
	const GEN_FLT x58 = x44 * x25;
	const GEN_FLT x59 = x45 + x47;
	const GEN_FLT x60 = x52 + x55;
	const GEN_FLT x61 = 2 * x52 * sensor_x + (x53 + x54) * sensor_y + (x50 + x60) * sensor_z;
	const GEN_FLT x62 = 2 * x55 * sensor_z + (x50 + x56) * sensor_y + (x54 + x60) * sensor_x;
	const GEN_FLT x63 = x28 * (x45 + x46 + x48) + x4 * x62 + 2 * x45 * x18 + x57 * x27 + x61 * x32 + (x58 + x59) * x21;
	const GEN_FLT x64 = x48 + x58;
	const GEN_FLT x65 = 2 * x46 * x28 + x57 * x37 + x61 * x38 + x62 * x34 + (x46 + x59) * x18 + (x46 + x64) * x21;
	const GEN_FLT x66 = x40 * x42 * (-x63 / x39 + x65 * x33 / x40);
	const GEN_FLT x67 = pow(x41, -1.0 / 2.0);
	const GEN_FLT x68 = 0.523598775598299 - tilt_1;
	const GEN_FLT x69 = tan(x68);
	const GEN_FLT x70 = x35 + x36;
	const GEN_FLT x71 = x2 + x3 * pow(x25, 2);
	const GEN_FLT x72 = -x24 + x26;
	const GEN_FLT x73 = x28 * (x46 + x47 + x58) + 2 * x58 * x21 + x70 * x61 + x71 * x57 + x72 * x62 + (x45 + x64) * x18;
	const GEN_FLT x74 = 2 * x63 * x33 + 2 * x65 * x39;
	const GEN_FLT x75 = lh_py + x70 * x28 + x71 * x21 + x72 * x18;
	const GEN_FLT x76 = (1.0 / 2.0) * x75;
	const GEN_FLT x77 = -x73 * x67 * x69 + x74 * x76 * x69 / pow(x41, 3.0 / 2.0);
	const GEN_FLT x78 = pow(x75, 2);
	const GEN_FLT x79 = pow(x69, 2);
	const GEN_FLT x80 = pow(1 - x79 * x78 * x42, -1.0 / 2.0);
	const GEN_FLT x81 = x66 - x80 * x77;
	const GEN_FLT x82 = x75 * x67;
	const GEN_FLT x83 = -x82 * x69;
	const GEN_FLT x84 = atan2(-x33, x39);
	const GEN_FLT x85 = ogeeMag_1 + x84 - asin(x83);
	const GEN_FLT x86 = cos(x85) * ogeePhase_1;
	const GEN_FLT x87 = x81 * x86;
	const GEN_FLT x88 = cos(x68);
	const GEN_FLT x89 = pow(x88, -1);
	const GEN_FLT x90 = x41 + x78;
	const GEN_FLT x91 = pow(x90, -1.0 / 2.0);
	const GEN_FLT x92 = x89 * x91;
	const GEN_FLT x93 = asin(x75 * x92);
	const GEN_FLT x94 = 8.0108022e-06 * x93;
	const GEN_FLT x95 = -8.0108022e-06 - x94;
	const GEN_FLT x96 = 0.0028679863 + x93 * x95;
	const GEN_FLT x97 = 5.3685255e-06 + x93 * x96;
	const GEN_FLT x98 = 0.0076069798 + x93 * x97;
	const GEN_FLT x99 = pow(x93, 2);
	const GEN_FLT x100 = sin(x68);
	const GEN_FLT x101 = sin(x85);
	const GEN_FLT x102 = curve_1 + x101 * ogeePhase_1;
	const GEN_FLT x103 = x93 * x98;
	const GEN_FLT x104 = -8.0108022e-06 - 1.60216044e-05 * x93;
	const GEN_FLT x105 = x96 + x93 * x104;
	const GEN_FLT x106 = x97 + x93 * x105;
	const GEN_FLT x107 = x98 + x93 * x106;
	const GEN_FLT x108 = x103 + x93 * x107;
	const GEN_FLT x109 = x102 * x108;
	const GEN_FLT x110 = x88 + x100 * x109;
	const GEN_FLT x111 = pow(x110, -1);
	const GEN_FLT x112 = x99 * x111;
	const GEN_FLT x113 = x98 * x112;
	const GEN_FLT x114 = x100 * x108;
	const GEN_FLT x115 = x73 * x92 - x89 * x76 * (x74 + 2 * x73 * x75) / pow(x90, 3.0 / 2.0);
	const GEN_FLT x116 = pow(x88, -2);
	const GEN_FLT x117 = pow(1 - x78 * x116 / x90, -1.0 / 2.0);
	const GEN_FLT x118 = x115 * x117;
	const GEN_FLT x119 = x95 * x118;
	const GEN_FLT x120 = 2.40324066e-05 * x93;
	const GEN_FLT x121 = x93 * (x119 - x94 * x118) + x96 * x118;
	const GEN_FLT x122 = x93 * x121 + x97 * x118;
	const GEN_FLT x123 = x100 * x102;
	const GEN_FLT x124 =
		x123 * (x107 * x118 + x93 * x122 +
				x93 * (x122 + x106 * x118 + x93 * (x121 + x105 * x118 + x93 * (x119 + x104 * x118 - x118 * x120))) +
				x98 * x118);
	const GEN_FLT x125 = x99 * x98 * x102 / pow(x110, 2);
	const GEN_FLT x126 = 2 * x103 * x102 * x111;
	const GEN_FLT x127 = x102 * x112;
	const GEN_FLT x128 = x77 + x118 * x126 + x122 * x127;
	const GEN_FLT x129 = x83 + x102 * x113;
	const GEN_FLT x130 = pow(1 - pow(x129, 2), -1.0 / 2.0);
	const GEN_FLT x131 = x66 - x130 * (x128 - x125 * (x124 + x87 * x114) + x87 * x113);
	const GEN_FLT x132 = gibPhase_1 + x84 - asin(x129);
	const GEN_FLT x133 = cos(x132) * gibMag_1;
	const GEN_FLT x134 = x131 + x133 * x131;
	const GEN_FLT x135 = x117 * (x115 - x75 * x91 * x100 * x116);
	const GEN_FLT x136 = x77 + x82 * (1 + x79);
	const GEN_FLT x137 = x66 - x80 * x136;
	const GEN_FLT x138 = x86 * x113;
	const GEN_FLT x139 = x86 * x114;
	const GEN_FLT x140 = x95 * x135;
	const GEN_FLT x141 = x93 * (x140 - x94 * x135) + x96 * x135;
	const GEN_FLT x142 = x93 * x141 + x97 * x135;
	const GEN_FLT x143 =
		x66 - x130 * (x136 -
					  x125 * (x100 +
							  x123 * (x107 * x135 + x93 * x142 +
									  x93 * (x142 + x106 * x135 +
											 x93 * (x141 + x105 * x135 + x93 * (x140 + x104 * x135 - x120 * x135))) +
									  x98 * x135) +
							  x137 * x139 - x88 * x109) +
					  x126 * x135 + x127 * x142 + x137 * x138);
	const GEN_FLT x144 = 1 + x87;
	const GEN_FLT x145 = x66 - x130 * (x128 + x113 * x144 - x125 * (x124 + x114 * x144));
	const GEN_FLT x146 = 1 + x81;
	const GEN_FLT x147 = x66 - x130 * (x128 - x125 * (x124 + x139 * x146) + x138 * x146);
	const GEN_FLT x148 = x101 + x87;
	const GEN_FLT x149 = x66 - x130 * (x128 + x113 * x148 - x125 * (x124 + x114 * x148));
	*(out++) = -1 + x134;
	*(out++) = x143 + x133 * x143;
	*(out++) = x145 + x133 * x145;
	*(out++) = x131 + x133 * (1 + x131);
	*(out++) = x134 + sin(x132);
	*(out++) = x147 + x133 * x147;
	*(out++) = x149 + x133 * x149;
}

/** Applying function <function reproject at 0x7ffa3c1c7320> */
static inline void gen_reproject_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p, const FLT *sensor_pt,
											const LinmathAxisAnglePose *lh_p, const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (1));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x5 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x6 = cos(x1);
	const GEN_FLT x7 = 1 - x6;
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x11 = sin(x10);
	const GEN_FLT x12 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x13 = x12 * x11;
	const GEN_FLT x14 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x15 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x16 = cos(x10);
	const GEN_FLT x17 = 1 - x16;
	const GEN_FLT x18 = x15 * x17;
	const GEN_FLT x19 = x14 * x18;
	const GEN_FLT x20 = x15 * x11;
	const GEN_FLT x21 = x14 * x12 * x17;
	const GEN_FLT x22 =
		obj_pz + (x13 + x19) * sensor_y + (x16 + pow(x14, 2) * x17) * sensor_z + (-x20 + x21) * sensor_x;
	const GEN_FLT x23 = x14 * x11;
	const GEN_FLT x24 = x12 * x18;
	const GEN_FLT x25 =
		obj_py + (-x13 + x19) * sensor_z + (x16 + pow(x15, 2) * x17) * sensor_y + (x23 + x24) * sensor_x;
	const GEN_FLT x26 = x2 * x4;
	const GEN_FLT x27 = x0 * x8;
	const GEN_FLT x28 =
		obj_px + (x16 + pow(x12, 2) * x17) * sensor_x + (x20 + x21) * sensor_z + (-x23 + x24) * sensor_y;
	const GEN_FLT x29 = lh_py + x25 * (x6 + pow(x5, 2) * x7) + (x26 + x27) * x28 + (-x3 + x9) * x22;
	const GEN_FLT x30 = x2 * x5;
	const GEN_FLT x31 = x0 * x4 * x7;
	const GEN_FLT x32 = lh_pz + x22 * (x6 + pow(x4, 2) * x7) + (x3 + x9) * x25 + (-x30 + x31) * x28;
	const GEN_FLT x33 = -x32;
	const GEN_FLT x34 = pow(x32, 2);
	const GEN_FLT x35 = lh_px + x28 * (x6 + pow(x0, 2) * x7) + (-x26 + x27) * x25 + (x30 + x31) * x22;
	const GEN_FLT x36 = atan2(x35, x33);
	const GEN_FLT x37 = -phase_0 - x36 - asin(x29 * tilt_0 / sqrt(x34 + pow(x35, 2)));
	const GEN_FLT x38 = -phase_1 - asin(x35 * tilt_1 / sqrt(x34 + pow(x29, 2))) - atan2(-x29, x33);
	*(out++) = x37 - cos(1.5707963267949 + gibPhase_0 + x37) * gibMag_0 + pow(atan2(x29, x33), 2) * curve_0;
	*(out++) = x38 + pow(x36, 2) * curve_1 - cos(1.5707963267949 + gibPhase_1 + x38) * gibMag_1;
}

// Jacobian of reproject wrt [obj_px, obj_py, obj_pz, obj_qi, obj_qj, obj_qk]
static inline void gen_reproject_jac_obj_p_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p, const FLT *sensor_pt,
													  const LinmathAxisAnglePose *lh_p, const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x1 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1));
	const GEN_FLT x2 = pow(obj_qk, 2);
	const GEN_FLT x3 = pow(obj_qj, 2);
	const GEN_FLT x4 = pow(obj_qi, 2);
	const GEN_FLT x5 = x2 + x3 + x4;
	const GEN_FLT x6 = sqrt(x5);
	const GEN_FLT x7 = cos(x6);
	const GEN_FLT x8 = 1 - x7;
	const GEN_FLT x9 = x1 * x8;
	const GEN_FLT x10 = x0 * x9;
	const GEN_FLT x11 = sin(x6);
	const GEN_FLT x12 = x0 * x11;
	const GEN_FLT x13 = -x12;
	const GEN_FLT x14 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x15 = x8 * x14;
	const GEN_FLT x16 = x0 * x15;
	const GEN_FLT x17 = x10 + x16;
	const GEN_FLT x18 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x19 = x8 * x18;
	const GEN_FLT x20 = x0 * x19;
	const GEN_FLT x21 = x10 + x20;
	const GEN_FLT x22 = 2 * x10 * sensor_x + (x13 + x17) * sensor_y + (x12 + x21) * sensor_z;
	const GEN_FLT x23 = 1 + x22;
	const GEN_FLT x24 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (0));
	const GEN_FLT x25 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x26 = sin(x25);
	const GEN_FLT x27 = x24 * x26;
	const GEN_FLT x28 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (1));
	const GEN_FLT x29 = cos(x25);
	const GEN_FLT x30 = 1 - x29;
	const GEN_FLT x31 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (0));
	const GEN_FLT x32 = x30 * x31;
	const GEN_FLT x33 = x32 * x28;
	const GEN_FLT x34 = -x27 + x33;
	const GEN_FLT x35 = x29 + x30 * pow(x31, 2);
	const GEN_FLT x36 = x16 + x20;
	const GEN_FLT x37 = 2 * x20 * sensor_z + (x13 + x21) * sensor_x + (x12 + x36) * sensor_y;
	const GEN_FLT x38 = x35 * x37;
	const GEN_FLT x39 = x28 * x26;
	const GEN_FLT x40 = x32 * x24;
	const GEN_FLT x41 = x39 + x40;
	const GEN_FLT x42 = 2 * x16 * sensor_y + (x12 + x17) * sensor_x + (x13 + x36) * sensor_z;
	const GEN_FLT x43 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x44 = x43 * x26;
	const GEN_FLT x45 = -x44;
	const GEN_FLT x46 = x43 * x32;
	const GEN_FLT x47 = x30 * x28;
	const GEN_FLT x48 = x43 * x47;
	const GEN_FLT x49 = x46 + x48;
	const GEN_FLT x50 = x14 * x11;
	const GEN_FLT x51 = x1 * x19;
	const GEN_FLT x52 = x11 * x18;
	const GEN_FLT x53 = x1 * x15;
	const GEN_FLT x54 = pow(x1, 2);
	const GEN_FLT x55 = obj_px + (x50 + x51) * sensor_z + (-x52 + x53) * sensor_y + (x7 + x8 * x54) * sensor_x;
	const GEN_FLT x56 = x1 * x11;
	const GEN_FLT x57 = x14 * x19;
	const GEN_FLT x58 = pow(x14, 2);
	const GEN_FLT x59 = obj_py + (x52 + x53) * sensor_x + (-x56 + x57) * sensor_z + (x7 + x8 * x58) * sensor_y;
	const GEN_FLT x60 = x43 * x30 * x24;
	const GEN_FLT x61 = x46 + x60;
	const GEN_FLT x62 = pow(x18, 2);
	const GEN_FLT x63 = obj_pz + (-x50 + x51) * sensor_x + (x56 + x57) * sensor_y + (x7 + x8 * x62) * sensor_z;
	const GEN_FLT x64 = 2 * x63 * x46 + (x45 + x49) * x55 + (x44 + x61) * x59;
	const GEN_FLT x65 = x64 + x41 * x42;
	const GEN_FLT x66 = x38 + x65 + x34 * x23;
	const GEN_FLT x67 = lh_pz + x55 * x34 + x59 * x41 + x63 * x35;
	const GEN_FLT x68 = pow(x67, 2);
	const GEN_FLT x69 = pow(x68, -1);
	const GEN_FLT x70 = x27 + x33;
	const GEN_FLT x71 = x31 * x26;
	const GEN_FLT x72 = x47 * x24;
	const GEN_FLT x73 = -x71 + x72;
	const GEN_FLT x74 = x29 + x30 * pow(x28, 2);
	const GEN_FLT x75 = lh_px + x70 * x63 + x73 * x59 + x74 * x55;
	const GEN_FLT x76 = x75 * x69;
	const GEN_FLT x77 = x73 * x42;
	const GEN_FLT x78 = x48 + x60;
	const GEN_FLT x79 = 2 * x55 * x48 + (x44 + x49) * x63 + (x45 + x78) * x59;
	const GEN_FLT x80 = x79 + x70 * x37;
	const GEN_FLT x81 = x77 + x80 + x74 * x23;
	const GEN_FLT x82 = pow(x67, -1);
	const GEN_FLT x83 = pow(x75, 2);
	const GEN_FLT x84 = x68 + x83;
	const GEN_FLT x85 = pow(x84, -1);
	const GEN_FLT x86 = x85 * x68;
	const GEN_FLT x87 = (x76 * x66 - x81 * x82) * x86;
	const GEN_FLT x88 = -x39 + x40;
	const GEN_FLT x89 = x29 + x30 * pow(x24, 2);
	const GEN_FLT x90 = x71 + x72;
	const GEN_FLT x91 = lh_py + x55 * x90 + x88 * x63 + x89 * x59;
	const GEN_FLT x92 = pow(x91, 2);
	const GEN_FLT x93 = pow(1 - x85 * x92 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x94 = x89 * x42;
	const GEN_FLT x95 = 2 * x60 * x59 + (x45 + x61) * x63 + (x44 + x78) * x55;
	const GEN_FLT x96 = x95 + x88 * x37;
	const GEN_FLT x97 = x94 + x96 + x90 * x23;
	const GEN_FLT x98 = tilt_0 / sqrt(x84);
	const GEN_FLT x99 = 2 * x75;
	const GEN_FLT x100 = 2 * x67;
	const GEN_FLT x101 = x66 * x100;
	const GEN_FLT x102 = (1.0 / 2.0) * x91 * tilt_0 / pow(x84, 3.0 / 2.0);
	const GEN_FLT x103 = -x87 - x93 * (-x102 * (x101 + x81 * x99) + x98 * x97);
	const GEN_FLT x104 = -x67;
	const GEN_FLT x105 = atan2(x75, x104);
	const GEN_FLT x106 = sin(1.5707963267949 + gibPhase_0 - phase_0 - x105 - asin(x91 * x98)) * gibMag_0;
	const GEN_FLT x107 = x69 * x91;
	const GEN_FLT x108 = x66 * x107;
	const GEN_FLT x109 = x82 * x97;
	const GEN_FLT x110 = x68 + x92;
	const GEN_FLT x111 = pow(x110, -1);
	const GEN_FLT x112 = x68 * x111;
	const GEN_FLT x113 = 2 * x112 * atan2(x91, x104) * curve_0;
	const GEN_FLT x114 = x34 * x22;
	const GEN_FLT x115 = 1 + x42;
	const GEN_FLT x116 = x114 + x38 + x64 + x41 * x115;
	const GEN_FLT x117 = x74 * x22;
	const GEN_FLT x118 = x117 + x80 + x73 * x115;
	const GEN_FLT x119 = (x76 * x116 - x82 * x118) * x86;
	const GEN_FLT x120 = x90 * x22;
	const GEN_FLT x121 = x120 + x96 + x89 * x115;
	const GEN_FLT x122 = x100 * x116;
	const GEN_FLT x123 = -x119 - x93 * (-x102 * (x122 + x99 * x118) + x98 * x121);
	const GEN_FLT x124 = x107 * x116;
	const GEN_FLT x125 = x82 * x121;
	const GEN_FLT x126 = 1 + x37;
	const GEN_FLT x127 = x114 + x65 + x35 * x126;
	const GEN_FLT x128 = x117 + x77 + x79 + x70 * x126;
	const GEN_FLT x129 = (x76 * x127 - x82 * x128) * x86;
	const GEN_FLT x130 = x120 + x94 + x95 + x88 * x126;
	const GEN_FLT x131 = x100 * x127;
	const GEN_FLT x132 = -x129 - x93 * (-x102 * (x131 + x99 * x128) + x98 * x130);
	const GEN_FLT x133 = x107 * x127;
	const GEN_FLT x134 = x82 * x130;
	const GEN_FLT x135 = pow(x6, -1);
	const GEN_FLT x136 = x135 * obj_qi;
	const GEN_FLT x137 = x11 * x136;
	const GEN_FLT x138 = -x137;
	const GEN_FLT x139 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qi, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x140 = x7 * x136;
	const GEN_FLT x141 = x18 * x140;
	const GEN_FLT x142 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qk * obj_qi / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x143 = x11 * x142;
	const GEN_FLT x144 = -x143;
	const GEN_FLT x145 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qj * obj_qi / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x146 = x9 * x145;
	const GEN_FLT x147 = x56 * x136;
	const GEN_FLT x148 = x146 + x14 * x147 + x15 * x139;
	const GEN_FLT x149 = x14 * x140;
	const GEN_FLT x150 = x11 * x145;
	const GEN_FLT x151 = x9 * x142;
	const GEN_FLT x152 = x151 + x18 * x147 + x19 * x139;
	const GEN_FLT x153 = (x138 + x54 * x137 + 2 * x9 * x139) * sensor_x + (-x141 + x144 + x148) * sensor_y +
						 (x149 + x150 + x152) * sensor_z;
	const GEN_FLT x154 = x15 * x145;
	const GEN_FLT x155 = x1 * x7;
	const GEN_FLT x156 = x136 * x155;
	const GEN_FLT x157 = x11 * x139;
	const GEN_FLT x158 = x50 * x18;
	const GEN_FLT x159 = x19 * x145;
	const GEN_FLT x160 = x15 * x142;
	const GEN_FLT x161 = x159 + x160 + x136 * x158;
	const GEN_FLT x162 =
		(x138 + 2 * x154 + x58 * x137) * sensor_y + (x141 + x143 + x148) * sensor_x + (-x156 - x157 + x161) * sensor_z;
	const GEN_FLT x163 = -x150;
	const GEN_FLT x164 = x19 * x142;
	const GEN_FLT x165 =
		(x156 + x157 + x161) * sensor_y + (-x149 + x152 + x163) * sensor_x + (x138 + 2 * x164 + x62 * x137) * sensor_z;
	const GEN_FLT x166 = x64 + x34 * x153 + x35 * x165 + x41 * x162;
	const GEN_FLT x167 = x79 + x70 * x165 + x73 * x162 + x74 * x153;
	const GEN_FLT x168 = (x76 * x166 - x82 * x167) * x86;
	const GEN_FLT x169 = x95 + x88 * x165 + x89 * x162 + x90 * x153;
	const GEN_FLT x170 = x100 * x166;
	const GEN_FLT x171 = -x168 - x93 * (-x102 * (x170 + x99 * x167) + x98 * x169);
	const GEN_FLT x172 = x107 * x166;
	const GEN_FLT x173 = x82 * x169;
	const GEN_FLT x174 = x135 * obj_qj;
	const GEN_FLT x175 = x7 * x18;
	const GEN_FLT x176 = x174 * x175;
	const GEN_FLT x177 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qk * obj_qj / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x178 = x11 * x177;
	const GEN_FLT x179 = x56 * x14;
	const GEN_FLT x180 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qj, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x181 = x154 + x179 * x174 + x9 * x180;
	const GEN_FLT x182 = x11 * x174;
	const GEN_FLT x183 = -x182;
	const GEN_FLT x184 = x174 * x155;
	const GEN_FLT x185 = x15 * x177;
	const GEN_FLT x186 = x185 + x174 * x158 + x19 * x180;
	const GEN_FLT x187 = (x163 - x184 + x186) * sensor_z + (x176 + x178 + x181) * sensor_x +
						 (x183 + 2 * x15 * x180 + x58 * x182) * sensor_y;
	const GEN_FLT x188 = -x178;
	const GEN_FLT x189 = x7 * x14;
	const GEN_FLT x190 = x174 * x189;
	const GEN_FLT x191 = x11 * x180;
	const GEN_FLT x192 = x56 * x18;
	const GEN_FLT x193 = x9 * x177;
	const GEN_FLT x194 = x159 + x193 + x174 * x192;
	const GEN_FLT x195 =
		(-x176 + x181 + x188) * sensor_y + (2 * x146 + x183 + x54 * x182) * sensor_x + (x190 + x191 + x194) * sensor_z;
	const GEN_FLT x196 = x19 * x177;
	const GEN_FLT x197 =
		(x183 + 2 * x196 + x62 * x182) * sensor_z + (x150 + x184 + x186) * sensor_y + (-x190 - x191 + x194) * sensor_x;
	const GEN_FLT x198 = x64 + x34 * x195 + x35 * x197 + x41 * x187;
	const GEN_FLT x199 = x79 + x70 * x197 + x73 * x187 + x74 * x195;
	const GEN_FLT x200 = (x76 * x198 - x82 * x199) * x86;
	const GEN_FLT x201 = x95 + x88 * x197 + x89 * x187 + x90 * x195;
	const GEN_FLT x202 = x100 * x198;
	const GEN_FLT x203 = -x200 - x93 * (-x102 * (x202 + x99 * x199) + x98 * x201);
	const GEN_FLT x204 = x107 * x198;
	const GEN_FLT x205 = x82 * x201;
	const GEN_FLT x206 = x135 * obj_qk;
	const GEN_FLT x207 = x206 * x175;
	const GEN_FLT x208 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qk, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x209 = x11 * x208;
	const GEN_FLT x210 = x160 + x193 + x206 * x179;
	const GEN_FLT x211 = x11 * x206;
	const GEN_FLT x212 = -x211;
	const GEN_FLT x213 = x206 * x155;
	const GEN_FLT x214 = x8 * x208;
	const GEN_FLT x215 = x196 + x14 * x214 + x206 * x158;
	const GEN_FLT x216 =
		(x207 + x209 + x210) * sensor_x + (2 * x185 + x212 + x58 * x211) * sensor_y + (x144 - x213 + x215) * sensor_z;
	const GEN_FLT x217 = x206 * x189;
	const GEN_FLT x218 = x164 + x1 * x214 + x206 * x192;
	const GEN_FLT x219 =
		(-x207 - x209 + x210) * sensor_y + (2 * x151 + x212 + x54 * x211) * sensor_x + (x178 + x217 + x218) * sensor_z;
	const GEN_FLT x220 = (x212 + 2 * x19 * x208 + x62 * x211) * sensor_z + (x143 + x213 + x215) * sensor_y +
						 (x188 - x217 + x218) * sensor_x;
	const GEN_FLT x221 = x64 + x34 * x219 + x35 * x220 + x41 * x216;
	const GEN_FLT x222 = x79 + x70 * x220 + x73 * x216 + x74 * x219;
	const GEN_FLT x223 = (x76 * x221 - x82 * x222) * x86;
	const GEN_FLT x224 = x95 + x88 * x220 + x89 * x216 + x90 * x219;
	const GEN_FLT x225 = x221 * x100;
	const GEN_FLT x226 = -x223 - x93 * (-x102 * (x225 + x99 * x222) + x98 * x224);
	const GEN_FLT x227 = x221 * x107;
	const GEN_FLT x228 = x82 * x224;
	const GEN_FLT x229 = pow(1 - x83 * x111 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x230 = tilt_1 / sqrt(x110);
	const GEN_FLT x231 = 2 * x91;
	const GEN_FLT x232 = (1.0 / 2.0) * x75 * tilt_1 / pow(x110, 3.0 / 2.0);
	const GEN_FLT x233 = -x229 * (-x232 * (x101 + x97 * x231) + x81 * x230) - (-x108 + x109) * x112;
	const GEN_FLT x234 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x75 * x230) - atan2(-x91, x104)) * gibMag_1;
	const GEN_FLT x235 = 2 * x105 * curve_1;
	const GEN_FLT x236 = -x229 * (x230 * x118 - x232 * (x122 + x231 * x121)) - (-x124 + x125) * x112;
	const GEN_FLT x237 = -x229 * (x230 * x128 - x232 * (x131 + x231 * x130)) - (-x133 + x134) * x112;
	const GEN_FLT x238 = -x229 * (x230 * x167 - x232 * (x170 + x231 * x169)) - (-x172 + x173) * x112;
	const GEN_FLT x239 = -x229 * (x230 * x199 - x232 * (x202 + x231 * x201)) - (-x204 + x205) * x112;
	const GEN_FLT x240 = -x229 * (x230 * x222 - x232 * (x225 + x231 * x224)) - (-x227 + x228) * x112;
	*(out++) = x103 + x103 * x106 + (x108 - x109) * x113;
	*(out++) = x123 + x106 * x123 + (x124 - x125) * x113;
	*(out++) = x132 + x106 * x132 + (x133 - x134) * x113;
	*(out++) = x171 + x106 * x171 + (x172 - x173) * x113;
	*(out++) = x203 + x203 * x106 + (x204 - x205) * x113;
	*(out++) = x226 + x226 * x106 + (x227 - x228) * x113;
	*(out++) = x233 + x234 * x233 + x87 * x235;
	*(out++) = x236 + x234 * x236 + x235 * x119;
	*(out++) = x237 + x234 * x237 + x235 * x129;
	*(out++) = x238 + x234 * x238 + x235 * x168;
	*(out++) = x239 + x234 * x239 + x235 * x200;
	*(out++) = x240 + x234 * x240 + x235 * x223;
}

// Jacobian of reproject wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_reproject_jac_sensor_pt_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
														  const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
														  const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (1));
	const GEN_FLT x5 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x6 = cos(x1);
	const GEN_FLT x7 = 1 - x6;
	const GEN_FLT x8 = x4 * x5 * x7;
	const GEN_FLT x9 = -x3 + x8;
	const GEN_FLT x10 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x11 = cos(x10);
	const GEN_FLT x12 = 1 - x11;
	const GEN_FLT x13 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x14 = x11 + pow(x13, 2) * x12;
	const GEN_FLT x15 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x16 = x15 * x12;
	const GEN_FLT x17 = x13 * x16;
	const GEN_FLT x18 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x19 = x18 * x16;
	const GEN_FLT x20 = sin(x10);
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = -x21;
	const GEN_FLT x23 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x24 = x23 * x16;
	const GEN_FLT x25 = 2 * x17 * sensor_x + (x17 + x19 + x22) * sensor_y + (x17 + x21 + x24) * sensor_z;
	const GEN_FLT x26 = x14 + x25;
	const GEN_FLT x27 = x23 * x20;
	const GEN_FLT x28 = x12 * x18;
	const GEN_FLT x29 = x28 * x13;
	const GEN_FLT x30 = x27 + x29;
	const GEN_FLT x31 = x19 + x21;
	const GEN_FLT x32 = x22 + x24;
	const GEN_FLT x33 = 2 * x19 * sensor_y + (x17 + x31) * sensor_x + (x19 + x32) * sensor_z;
	const GEN_FLT x34 = x30 + x33;
	const GEN_FLT x35 = x2 * x4;
	const GEN_FLT x36 = x0 * x7;
	const GEN_FLT x37 = x5 * x36;
	const GEN_FLT x38 = x35 + x37;
	const GEN_FLT x39 = x20 * x18;
	const GEN_FLT x40 = x23 * x13 * x12;
	const GEN_FLT x41 = -x39 + x40;
	const GEN_FLT x42 = 2 * x24 * sensor_z + (x24 + x31) * sensor_y + (x17 + x32) * sensor_x;
	const GEN_FLT x43 = x41 + x42;
	const GEN_FLT x44 = x6 + pow(x5, 2) * x7;
	const GEN_FLT x45 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x46 = x2 * x45;
	const GEN_FLT x47 = -x46;
	const GEN_FLT x48 = x7 * x45;
	const GEN_FLT x49 = x4 * x48;
	const GEN_FLT x50 = x5 * x48;
	const GEN_FLT x51 = x49 + x50;
	const GEN_FLT x52 = x39 + x40;
	const GEN_FLT x53 = -x27 + x29;
	const GEN_FLT x54 = obj_px + x14 * sensor_x + x52 * sensor_z + x53 * sensor_y;
	const GEN_FLT x55 = x20 * x13;
	const GEN_FLT x56 = x23 * x28;
	const GEN_FLT x57 = -x55 + x56;
	const GEN_FLT x58 = x11 + x12 * pow(x18, 2);
	const GEN_FLT x59 = obj_py + x30 * sensor_x + x57 * sensor_z + x58 * sensor_y;
	const GEN_FLT x60 = x0 * x48;
	const GEN_FLT x61 = x46 + x60;
	const GEN_FLT x62 = x55 + x56;
	const GEN_FLT x63 = x11 + pow(x23, 2) * x12;
	const GEN_FLT x64 = obj_pz + x41 * sensor_x + x62 * sensor_y + x63 * sensor_z;
	const GEN_FLT x65 = 2 * x64 * x50 + (x47 + x51) * x54 + (x50 + x61) * x59;
	const GEN_FLT x66 = x65 + x34 * x38 + x43 * x44 + x9 * x26;
	const GEN_FLT x67 = x3 + x8;
	const GEN_FLT x68 = x2 * x5;
	const GEN_FLT x69 = x4 * x36;
	const GEN_FLT x70 = -x68 + x69;
	const GEN_FLT x71 = x6 + pow(x4, 2) * x7;
	const GEN_FLT x72 = lh_px + x64 * x67 + x70 * x59 + x71 * x54;
	const GEN_FLT x73 = lh_pz + x59 * x38 + x64 * x44 + x9 * x54;
	const GEN_FLT x74 = pow(x73, 2);
	const GEN_FLT x75 = pow(x74, -1);
	const GEN_FLT x76 = x72 * x75;
	const GEN_FLT x77 = pow(x73, -1);
	const GEN_FLT x78 = x47 + x60;
	const GEN_FLT x79 = 2 * x54 * x49 + (x46 + x51) * x64 + (x49 + x78) * x59;
	const GEN_FLT x80 = x79 + x67 * x43 + x70 * x34 + x71 * x26;
	const GEN_FLT x81 = pow(x72, 2);
	const GEN_FLT x82 = x74 + x81;
	const GEN_FLT x83 = pow(x82, -1);
	const GEN_FLT x84 = x83 * x74;
	const GEN_FLT x85 = (x76 * x66 - x80 * x77) * x84;
	const GEN_FLT x86 = x68 + x69;
	const GEN_FLT x87 = x6 + pow(x0, 2) * x7;
	const GEN_FLT x88 = -x35 + x37;
	const GEN_FLT x89 = 2 * x60 * x59 + (x49 + x61) * x54 + (x50 + x78) * x64;
	const GEN_FLT x90 = x89 + x86 * x26 + x87 * x34 + x88 * x43;
	const GEN_FLT x91 = tilt_0 / sqrt(x82);
	const GEN_FLT x92 = 2 * x72;
	const GEN_FLT x93 = 2 * x73;
	const GEN_FLT x94 = x66 * x93;
	const GEN_FLT x95 = lh_py + x86 * x54 + x87 * x59 + x88 * x64;
	const GEN_FLT x96 = (1.0 / 2.0) * x95 * tilt_0 / pow(x82, 3.0 / 2.0);
	const GEN_FLT x97 = pow(x95, 2);
	const GEN_FLT x98 = pow(1 - x83 * x97 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x99 = -x85 - x98 * (x91 * x90 - x96 * (x94 + x80 * x92));
	const GEN_FLT x100 = -x73;
	const GEN_FLT x101 = atan2(x72, x100);
	const GEN_FLT x102 = sin(1.5707963267949 + gibPhase_0 - phase_0 - x101 - asin(x91 * x95)) * gibMag_0;
	const GEN_FLT x103 = x75 * x95;
	const GEN_FLT x104 = x66 * x103;
	const GEN_FLT x105 = x77 * x90;
	const GEN_FLT x106 = x74 + x97;
	const GEN_FLT x107 = pow(x106, -1);
	const GEN_FLT x108 = x74 * x107;
	const GEN_FLT x109 = 2 * x108 * atan2(x95, x100) * curve_0;
	const GEN_FLT x110 = x25 + x53;
	const GEN_FLT x111 = x33 + x58;
	const GEN_FLT x112 = x42 + x62;
	const GEN_FLT x113 = x65 + x38 * x111 + x44 * x112 + x9 * x110;
	const GEN_FLT x114 = x79 + x67 * x112 + x70 * x111 + x71 * x110;
	const GEN_FLT x115 = (x76 * x113 - x77 * x114) * x84;
	const GEN_FLT x116 = x89 + x86 * x110 + x87 * x111 + x88 * x112;
	const GEN_FLT x117 = x93 * x113;
	const GEN_FLT x118 = -x115 - x98 * (x91 * x116 - x96 * (x117 + x92 * x114));
	const GEN_FLT x119 = x103 * x113;
	const GEN_FLT x120 = x77 * x116;
	const GEN_FLT x121 = x33 + x57;
	const GEN_FLT x122 = x25 + x52;
	const GEN_FLT x123 = x42 + x63;
	const GEN_FLT x124 = x65 + x38 * x121 + x44 * x123 + x9 * x122;
	const GEN_FLT x125 = x79 + x67 * x123 + x70 * x121 + x71 * x122;
	const GEN_FLT x126 = (x76 * x124 - x77 * x125) * x84;
	const GEN_FLT x127 = x89 + x86 * x122 + x87 * x121 + x88 * x123;
	const GEN_FLT x128 = x93 * x124;
	const GEN_FLT x129 = -x126 - x98 * (x91 * x127 - x96 * (x128 + x92 * x125));
	const GEN_FLT x130 = x103 * x124;
	const GEN_FLT x131 = x77 * x127;
	const GEN_FLT x132 = pow(1 - x81 * x107 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x133 = tilt_1 / sqrt(x106);
	const GEN_FLT x134 = 2 * x95;
	const GEN_FLT x135 = (1.0 / 2.0) * x72 * tilt_1 / pow(x106, 3.0 / 2.0);
	const GEN_FLT x136 = -x132 * (-x135 * (x94 + x90 * x134) + x80 * x133) - (-x104 + x105) * x108;
	const GEN_FLT x137 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x72 * x133) - atan2(-x95, x100)) * gibMag_1;
	const GEN_FLT x138 = 2 * x101 * curve_1;
	const GEN_FLT x139 = -x132 * (x114 * x133 - x135 * (x117 + x116 * x134)) - (-x119 + x120) * x108;
	const GEN_FLT x140 = -x132 * (x125 * x133 - x135 * (x128 + x127 * x134)) - (-x130 + x131) * x108;
	*(out++) = x99 + x99 * x102 + (x104 - x105) * x109;
	*(out++) = x118 + x102 * x118 + (x119 - x120) * x109;
	*(out++) = x129 + x102 * x129 + (x130 - x131) * x109;
	*(out++) = x136 + x137 * x136 + x85 * x138;
	*(out++) = x139 + x115 * x138 + x137 * x139;
	*(out++) = x140 + x126 * x138 + x137 * x140;
}

// Jacobian of reproject wrt [lh_px, lh_py, lh_pz, lh_qi, lh_qj, lh_qk]
static inline void gen_reproject_jac_lh_p_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p, const FLT *sensor_pt,
													 const LinmathAxisAnglePose *lh_p, const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x1 = pow(lh_qk, 2);
	const GEN_FLT x2 = pow(lh_qj, 2);
	const GEN_FLT x3 = pow(lh_qi, 2);
	const GEN_FLT x4 = x1 + x2 + x3;
	const GEN_FLT x5 = sqrt(x4);
	const GEN_FLT x6 = sin(x5);
	const GEN_FLT x7 = x0 * x6;
	const GEN_FLT x8 = -x7;
	const GEN_FLT x9 = cos(x5);
	const GEN_FLT x10 = 1 - x9;
	const GEN_FLT x11 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (1));
	const GEN_FLT x12 = x11 * x10;
	const GEN_FLT x13 = x0 * x12;
	const GEN_FLT x14 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (0));
	const GEN_FLT x15 = x14 * x10;
	const GEN_FLT x16 = x0 * x15;
	const GEN_FLT x17 = x13 + x16;
	const GEN_FLT x18 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x19 = sin(x18);
	const GEN_FLT x20 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x23 = cos(x18);
	const GEN_FLT x24 = 1 - x23;
	const GEN_FLT x25 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x26 = x24 * x25;
	const GEN_FLT x27 = x22 * x26;
	const GEN_FLT x28 = x22 * x19;
	const GEN_FLT x29 = x20 * x26;
	const GEN_FLT x30 =
		obj_px + (x21 + x27) * sensor_z + (x23 + x24 * pow(x25, 2)) * sensor_x + (-x28 + x29) * sensor_y;
	const GEN_FLT x31 = x25 * x19;
	const GEN_FLT x32 = x24 * x22;
	const GEN_FLT x33 = x32 * x20;
	const GEN_FLT x34 =
		obj_py + (x23 + x24 * pow(x20, 2)) * sensor_y + (x28 + x29) * sensor_x + (-x31 + x33) * sensor_z;
	const GEN_FLT x35 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (0));
	const GEN_FLT x36 = x35 * x10;
	const GEN_FLT x37 = x0 * x36;
	const GEN_FLT x38 = x37 + x7;
	const GEN_FLT x39 =
		obj_pz + (-x21 + x27) * sensor_x + (x23 + x24 * pow(x22, 2)) * sensor_z + (x31 + x33) * sensor_y;
	const GEN_FLT x40 = x6 * x11;
	const GEN_FLT x41 = x36 * x14;
	const GEN_FLT x42 = x40 + x41;
	const GEN_FLT x43 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x44 = x43 * x19;
	const GEN_FLT x45 = x43 * x26;
	const GEN_FLT x46 = x43 * x24 * x20;
	const GEN_FLT x47 = x45 + x46;
	const GEN_FLT x48 = -x44;
	const GEN_FLT x49 = x43 * x32;
	const GEN_FLT x50 = x46 + x49;
	const GEN_FLT x51 = 2 * x46 * sensor_y + (x44 + x47) * sensor_x + (x48 + x50) * sensor_z;
	const GEN_FLT x52 = x45 + x49;
	const GEN_FLT x53 = 2 * x45 * sensor_x + (x47 + x48) * sensor_y + (x44 + x52) * sensor_z;
	const GEN_FLT x54 = x6 * x35;
	const GEN_FLT x55 = x14 * x12;
	const GEN_FLT x56 = -x54 + x55;
	const GEN_FLT x57 = pow(x14, 2);
	const GEN_FLT x58 = x9 + x57 * x10;
	const GEN_FLT x59 = 2 * x49 * sensor_z + (x44 + x50) * sensor_y + (x48 + x52) * sensor_x;
	const GEN_FLT x60 = x51 * x42 + x53 * x56 + x58 * x59;
	const GEN_FLT x61 = x60 + x30 * (x17 + x8) + 2 * x39 * x16 + (x16 + x38) * x34;
	const GEN_FLT x62 = x54 + x55;
	const GEN_FLT x63 = x6 * x14;
	const GEN_FLT x64 = x36 * x11;
	const GEN_FLT x65 = -x63 + x64;
	const GEN_FLT x66 = pow(x11, 2);
	const GEN_FLT x67 = x9 + x66 * x10;
	const GEN_FLT x68 = lh_px + x62 * x39 + x65 * x34 + x67 * x30;
	const GEN_FLT x69 = lh_pz + x42 * x34 + x56 * x30 + x58 * x39;
	const GEN_FLT x70 = pow(x69, 2);
	const GEN_FLT x71 = pow(x70, -1);
	const GEN_FLT x72 = x71 * x68;
	const GEN_FLT x73 = x72 * x61;
	const GEN_FLT x74 = x37 + x8;
	const GEN_FLT x75 = x62 * x59 + x65 * x51 + x67 * x53;
	const GEN_FLT x76 = x75 + 2 * x30 * x13 + x39 * (x17 + x7) + (x13 + x74) * x34;
	const GEN_FLT x77 = 1 + x76;
	const GEN_FLT x78 = pow(x69, -1);
	const GEN_FLT x79 = pow(x68, 2);
	const GEN_FLT x80 = x70 + x79;
	const GEN_FLT x81 = pow(x80, -1);
	const GEN_FLT x82 = x81 * x70;
	const GEN_FLT x83 = x82 * (x73 - x78 * x77);
	const GEN_FLT x84 = -x40 + x41;
	const GEN_FLT x85 = pow(x35, 2);
	const GEN_FLT x86 = x9 + x85 * x10;
	const GEN_FLT x87 = x63 + x64;
	const GEN_FLT x88 = lh_py + x84 * x39 + x86 * x34 + x87 * x30;
	const GEN_FLT x89 = pow(x88, 2);
	const GEN_FLT x90 = pow(1 - x81 * x89 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x91 = x84 * x59 + x86 * x51 + x87 * x53;
	const GEN_FLT x92 = x91 + 2 * x34 * x37 + (x13 + x38) * x30 + (x16 + x74) * x39;
	const GEN_FLT x93 = tilt_0 / sqrt(x80);
	const GEN_FLT x94 = x93 * x92;
	const GEN_FLT x95 = 2 * x68;
	const GEN_FLT x96 = 2 * x69;
	const GEN_FLT x97 = x61 * x96;
	const GEN_FLT x98 = (1.0 / 2.0) * x88 * tilt_0 / pow(x80, 3.0 / 2.0);
	const GEN_FLT x99 = -x83 - x90 * (x94 - x98 * (x97 + x77 * x95));
	const GEN_FLT x100 = -x69;
	const GEN_FLT x101 = atan2(x68, x100);
	const GEN_FLT x102 = sin(1.5707963267949 + gibPhase_0 - phase_0 - x101 - asin(x88 * x93)) * gibMag_0;
	const GEN_FLT x103 = x88 * x71;
	const GEN_FLT x104 = x61 * x103;
	const GEN_FLT x105 = x78 * x92;
	const GEN_FLT x106 = -x105;
	const GEN_FLT x107 = x70 + x89;
	const GEN_FLT x108 = pow(x107, -1);
	const GEN_FLT x109 = x70 * x108;
	const GEN_FLT x110 = 2 * x109 * atan2(x88, x100) * curve_0;
	const GEN_FLT x111 = -x78 * x76;
	const GEN_FLT x112 = x82 * (x111 + x73);
	const GEN_FLT x113 = 1 + x92;
	const GEN_FLT x114 = x76 * x95;
	const GEN_FLT x115 = -x112 - x90 * (x93 * x113 - x98 * (x114 + x97));
	const GEN_FLT x116 = x78 * x113;
	const GEN_FLT x117 = 1 + x61;
	const GEN_FLT x118 = x82 * (x111 + x72 * x117);
	const GEN_FLT x119 = x96 * x117;
	const GEN_FLT x120 = -x118 - x90 * (x94 - (x114 + x119) * x98);
	const GEN_FLT x121 = x103 * x117;
	const GEN_FLT x122 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qj * lh_qi / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x123 = x6 * x122;
	const GEN_FLT x124 = -x123;
	const GEN_FLT x125 = pow(x5, -1);
	const GEN_FLT x126 = x125 * lh_qi;
	const GEN_FLT x127 = x9 * x35;
	const GEN_FLT x128 = x127 * x126;
	const GEN_FLT x129 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qi, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x130 = x63 * x11;
	const GEN_FLT x131 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qk * lh_qi / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x132 = x12 * x131;
	const GEN_FLT x133 = x132 + x126 * x130 + x15 * x129;
	const GEN_FLT x134 = x6 * x129;
	const GEN_FLT x135 = x9 * x11;
	const GEN_FLT x136 = x126 * x135;
	const GEN_FLT x137 = x15 * x122;
	const GEN_FLT x138 = x36 * x131;
	const GEN_FLT x139 = x54 * x126;
	const GEN_FLT x140 = x137 + x138 + x14 * x139;
	const GEN_FLT x141 = x6 * x126;
	const GEN_FLT x142 = -x141;
	const GEN_FLT x143 = x15 * x131;
	const GEN_FLT x144 =
		x60 + x30 * (x124 - x128 + x133) + x34 * (x134 + x136 + x140) + x39 * (x142 + 2 * x143 + x57 * x141);
	const GEN_FLT x145 = x71 * x144;
	const GEN_FLT x146 = x6 * x131;
	const GEN_FLT x147 = -x146;
	const GEN_FLT x148 = x9 * x14;
	const GEN_FLT x149 = x126 * x148;
	const GEN_FLT x150 = x12 * x122;
	const GEN_FLT x151 = x150 + x11 * x139 + x36 * x129;
	const GEN_FLT x152 =
		x75 + x30 * (x142 + 2 * x12 * x129 + x66 * x141) + x34 * (x147 - x149 + x151) + x39 * (x123 + x128 + x133);
	const GEN_FLT x153 = (x68 * x145 - x78 * x152) * x82;
	const GEN_FLT x154 = x36 * x122;
	const GEN_FLT x155 =
		x91 + x30 * (x146 + x149 + x151) + x34 * (x142 + 2 * x154 + x85 * x141) + x39 * (-x134 - x136 + x140);
	const GEN_FLT x156 = x96 * x144;
	const GEN_FLT x157 = -x153 - x90 * (x93 * x155 - x98 * (x156 + x95 * x152));
	const GEN_FLT x158 = x88 * x145;
	const GEN_FLT x159 = x78 * x155;
	const GEN_FLT x160 = x125 * lh_qj;
	const GEN_FLT x161 = x160 * x135;
	const GEN_FLT x162 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qk * lh_qj / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x163 = x36 * x162;
	const GEN_FLT x164 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qj, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x165 = x54 * x160;
	const GEN_FLT x166 = x163 + x14 * x165 + x15 * x164;
	const GEN_FLT x167 = x6 * x164;
	const GEN_FLT x168 = x127 * x160;
	const GEN_FLT x169 = x12 * x162;
	const GEN_FLT x170 = x137 + x169 + x160 * x130;
	const GEN_FLT x171 = x6 * x160;
	const GEN_FLT x172 = -x171;
	const GEN_FLT x173 = x15 * x162;
	const GEN_FLT x174 =
		x60 + x30 * (-x167 - x168 + x170) + x34 * (x123 + x161 + x166) + x39 * (x172 + 2 * x173 + x57 * x171);
	const GEN_FLT x175 = x6 * x162;
	const GEN_FLT x176 = -x175;
	const GEN_FLT x177 = x160 * x148;
	const GEN_FLT x178 = x154 + x11 * x165 + x12 * x164;
	const GEN_FLT x179 =
		x75 + x30 * (2 * x150 + x172 + x66 * x171) + x34 * (x176 - x177 + x178) + x39 * (x167 + x168 + x170);
	const GEN_FLT x180 = (x72 * x174 - x78 * x179) * x82;
	const GEN_FLT x181 =
		x91 + x30 * (x175 + x177 + x178) + x34 * (x172 + 2 * x36 * x164 + x85 * x171) + x39 * (x124 - x161 + x166);
	const GEN_FLT x182 = x96 * x174;
	const GEN_FLT x183 = -x180 - x90 * (x93 * x181 - x98 * (x182 + x95 * x179));
	const GEN_FLT x184 = x103 * x174;
	const GEN_FLT x185 = x78 * x181;
	const GEN_FLT x186 = x125 * lh_qk;
	const GEN_FLT x187 = x127 * x186;
	const GEN_FLT x188 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qk, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x189 = x143 + x12 * x188 + x186 * x130;
	const GEN_FLT x190 = x186 * x135;
	const GEN_FLT x191 = x54 * x186;
	const GEN_FLT x192 = x173 + x14 * x191 + x36 * x188;
	const GEN_FLT x193 = x6 * x186;
	const GEN_FLT x194 = -x193;
	const GEN_FLT x195 =
		x60 + x30 * (x176 - x187 + x189) + x34 * (x146 + x190 + x192) + x39 * (x194 + 2 * x15 * x188 + x57 * x193);
	const GEN_FLT x196 = x6 * x188;
	const GEN_FLT x197 = x186 * x148;
	const GEN_FLT x198 = x138 + x169 + x11 * x191;
	const GEN_FLT x199 =
		x75 + x30 * (2 * x132 + x194 + x66 * x193) + x34 * (-x196 - x197 + x198) + x39 * (x175 + x187 + x189);
	const GEN_FLT x200 = (x72 * x195 - x78 * x199) * x82;
	const GEN_FLT x201 =
		x91 + x30 * (x196 + x197 + x198) + x34 * (2 * x163 + x194 + x85 * x193) + x39 * (x147 - x190 + x192);
	const GEN_FLT x202 = x96 * x195;
	const GEN_FLT x203 = -x200 - x90 * (x93 * x201 - x98 * (x202 + x95 * x199));
	const GEN_FLT x204 = x103 * x195;
	const GEN_FLT x205 = x78 * x201;
	const GEN_FLT x206 = -x104;
	const GEN_FLT x207 = pow(1 - x79 * x108 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x208 = tilt_1 / sqrt(x107);
	const GEN_FLT x209 = 2 * x88;
	const GEN_FLT x210 = x92 * x209;
	const GEN_FLT x211 = (1.0 / 2.0) * x68 * tilt_1 / pow(x107, 3.0 / 2.0);
	const GEN_FLT x212 = -x207 * (-x211 * (x210 + x97) + x77 * x208) - (x105 + x206) * x109;
	const GEN_FLT x213 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x68 * x208) - atan2(-x88, x100)) * gibMag_1;
	const GEN_FLT x214 = 2 * x101 * curve_1;
	const GEN_FLT x215 = x76 * x208;
	const GEN_FLT x216 = -x207 * (x215 - x211 * (x97 + x209 * x113)) - (x116 + x206) * x109;
	const GEN_FLT x217 = -x207 * (x215 - (x119 + x210) * x211) - (x105 - x121) * x109;
	const GEN_FLT x218 = -x207 * (x208 * x152 - x211 * (x156 + x209 * x155)) - (-x158 + x159) * x109;
	const GEN_FLT x219 = -x207 * (x208 * x179 - x211 * (x182 + x209 * x181)) - (-x184 + x185) * x109;
	const GEN_FLT x220 = -x207 * (x208 * x199 - x211 * (x202 + x201 * x209)) - (-x204 + x205) * x109;
	*(out++) = x99 + x99 * x102 + (x104 + x106) * x110;
	*(out++) = x115 + x102 * x115 + (x104 - x116) * x110;
	*(out++) = x120 + x102 * x120 + (x106 + x121) * x110;
	*(out++) = x157 + x102 * x157 + (x158 - x159) * x110;
	*(out++) = x183 + x102 * x183 + (x184 - x185) * x110;
	*(out++) = x203 + x203 * x102 + (x204 - x205) * x110;
	*(out++) = x212 + x213 * x212 + x83 * x214;
	*(out++) = x216 + x213 * x216 + x214 * x112;
	*(out++) = x217 + x213 * x217 + x214 * x118;
	*(out++) = x218 + x213 * x218 + x214 * x153;
	*(out++) = x219 + x213 * x219 + x214 * x180;
	*(out++) = x220 + x213 * x220 + x214 * x200;
}

// Jacobian of reproject wrt [phase_0, tilt_0, curve_0, gibPhase_0, gibMag_0, ogeeMag_0, ogeePhase_0, phase_1, tilt_1,
// curve_1, gibPhase_1, gibMag_1, ogeeMag_1, ogeePhase_1]
static inline void gen_reproject_jac_bsd_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p, const FLT *sensor_pt,
													const LinmathAxisAnglePose *lh_p, const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (1));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x5 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x6 = cos(x1);
	const GEN_FLT x7 = 1 - x6;
	const GEN_FLT x8 = x4 * x5 * x7;
	const GEN_FLT x9 = -x3 + x8;
	const GEN_FLT x10 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x11 = sin(x10);
	const GEN_FLT x12 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x13 = x12 * x11;
	const GEN_FLT x14 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x15 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x16 = cos(x10);
	const GEN_FLT x17 = 1 - x16;
	const GEN_FLT x18 = x15 * x17;
	const GEN_FLT x19 = x14 * x18;
	const GEN_FLT x20 = x15 * x11;
	const GEN_FLT x21 = x12 * x17;
	const GEN_FLT x22 = x21 * x14;
	const GEN_FLT x23 =
		obj_pz + (x13 + x19) * sensor_y + (x16 + pow(x14, 2) * x17) * sensor_z + (-x20 + x22) * sensor_x;
	const GEN_FLT x24 = x14 * x11;
	const GEN_FLT x25 = x21 * x15;
	const GEN_FLT x26 =
		obj_py + (-x13 + x19) * sensor_z + (x16 + pow(x15, 2) * x17) * sensor_y + (x24 + x25) * sensor_x;
	const GEN_FLT x27 = x6 + pow(x5, 2) * x7;
	const GEN_FLT x28 = x2 * x4;
	const GEN_FLT x29 = x0 * x7;
	const GEN_FLT x30 = x5 * x29;
	const GEN_FLT x31 = x28 + x30;
	const GEN_FLT x32 =
		obj_px + (x16 + pow(x12, 2) * x17) * sensor_x + (x20 + x22) * sensor_z + (-x24 + x25) * sensor_y;
	const GEN_FLT x33 = lh_py + x26 * x27 + x32 * x31 + x9 * x23;
	const GEN_FLT x34 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x35 = x7 * x34;
	const GEN_FLT x36 = x4 * x35;
	const GEN_FLT x37 = x2 * x34;
	const GEN_FLT x38 = -x37;
	const GEN_FLT x39 = x0 * x35;
	const GEN_FLT x40 = x38 + x39;
	const GEN_FLT x41 = x3 + x8;
	const GEN_FLT x42 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x43 = x42 * x11;
	const GEN_FLT x44 = x42 * x21;
	const GEN_FLT x45 = x42 * x18;
	const GEN_FLT x46 = x44 + x45;
	const GEN_FLT x47 = -x43;
	const GEN_FLT x48 = x42 * x14 * x17;
	const GEN_FLT x49 = x45 + x48;
	const GEN_FLT x50 = 2 * x45 * sensor_y + (x43 + x46) * sensor_x + (x47 + x49) * sensor_z;
	const GEN_FLT x51 = x5 * x35;
	const GEN_FLT x52 = x36 + x51;
	const GEN_FLT x53 = x44 + x48;
	const GEN_FLT x54 = 2 * x44 * sensor_x + (x46 + x47) * sensor_y + (x43 + x53) * sensor_z;
	const GEN_FLT x55 = x2 * x5;
	const GEN_FLT x56 = x4 * x29;
	const GEN_FLT x57 = -x55 + x56;
	const GEN_FLT x58 = x6 + pow(x4, 2) * x7;
	const GEN_FLT x59 = 2 * x48 * sensor_z + (x43 + x49) * sensor_y + (x47 + x53) * sensor_x;
	const GEN_FLT x60 = 2 * x36 * x23 + x50 * x41 + x54 * x57 + x58 * x59 + (x36 + x40) * x32 + (x37 + x52) * x26;
	const GEN_FLT x61 = lh_pz + x41 * x26 + x57 * x32 + x58 * x23;
	const GEN_FLT x62 = pow(x61, 2);
	const GEN_FLT x63 = x60 / x62;
	const GEN_FLT x64 = x63 * x33;
	const GEN_FLT x65 = x37 + x39;
	const GEN_FLT x66 = x50 * x27 + 2 * x51 * x26 + x54 * x31 + x9 * x59 + (x38 + x52) * x23 + (x51 + x65) * x32;
	const GEN_FLT x67 = pow(x61, -1);
	const GEN_FLT x68 = x67 * x66;
	const GEN_FLT x69 = -x61;
	const GEN_FLT x70 = atan2(x33, x69);
	const GEN_FLT x71 = pow(x33, 2);
	const GEN_FLT x72 = x62 + x71;
	const GEN_FLT x73 = pow(x72, -1);
	const GEN_FLT x74 = x73 * x62;
	const GEN_FLT x75 = 2 * (x64 - x68) * x70 * x74 * curve_0;
	const GEN_FLT x76 = x55 + x56;
	const GEN_FLT x77 = -x28 + x30;
	const GEN_FLT x78 = x6 + pow(x0, 2) * x7;
	const GEN_FLT x79 = lh_px + x76 * x23 + x77 * x26 + x78 * x32;
	const GEN_FLT x80 = pow(x79, 2);
	const GEN_FLT x81 = x62 + x80;
	const GEN_FLT x82 = pow(x81, -1);
	const GEN_FLT x83 = 2 * x32 * x39 + x76 * x59 + x77 * x50 + x78 * x54 + (x40 + x51) * x26 + (x36 + x65) * x23;
	const GEN_FLT x84 = (x79 * x63 - x83 * x67) * x82 * x62;
	const GEN_FLT x85 = -x84;
	const GEN_FLT x86 = pow(1 - x82 * x71 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x87 = pow(x81, -1.0 / 2.0);
	const GEN_FLT x88 = 2 * x60 * x61;
	const GEN_FLT x89 = x87 * x66 * tilt_0 + (-1.0 / 2.0) * x33 * tilt_0 * (x88 + 2 * x83 * x79) / pow(x81, 3.0 / 2.0);
	const GEN_FLT x90 = x85 - x89 * x86;
	const GEN_FLT x91 = -1 + x90;
	const GEN_FLT x92 = x87 * x33;
	const GEN_FLT x93 = atan2(x79, x69);
	const GEN_FLT x94 = 1.5707963267949 + gibPhase_0 - phase_0 - x93 - asin(x92 * tilt_0);
	const GEN_FLT x95 = sin(x94) * gibMag_0;
	const GEN_FLT x96 = x85 - (x89 + x92) * x86;
	const GEN_FLT x97 = x75 + x90;
	const GEN_FLT x98 = x97 + x90 * x95;
	const GEN_FLT x99 = -(-x64 + x68) * x74;
	const GEN_FLT x100 = pow(1 - x80 * x73 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x101 = pow(x72, -1.0 / 2.0);
	const GEN_FLT x102 =
		x83 * x101 * tilt_1 + (-1.0 / 2.0) * x79 * tilt_1 * (x88 + 2 * x66 * x33) / pow(x72, 3.0 / 2.0);
	const GEN_FLT x103 = x99 - x100 * x102;
	const GEN_FLT x104 = x79 * x101;
	const GEN_FLT x105 = 1.5707963267949 + gibPhase_1 - phase_1 - asin(x104 * tilt_1) - atan2(-x33, x69);
	const GEN_FLT x106 = sin(x105) * gibMag_1;
	const GEN_FLT x107 = 2 * x84 * x93 * curve_1;
	const GEN_FLT x108 = x103 + x107;
	const GEN_FLT x109 = x108 + x103 * x106;
	const GEN_FLT x110 = -1 + x103;
	const GEN_FLT x111 = x99 - (x102 + x104) * x100;
	*(out++) = x75 + x91 + x91 * x95;
	*(out++) = x75 + x96 + x96 * x95;
	*(out++) = x98 + pow(x70, 2);
	*(out++) = x97 + x95 * (1 + x90);
	*(out++) = x98 - cos(x94);
	*(out++) = x98;
	*(out++) = x98;
	*(out++) = x98;
	*(out++) = x98;
	*(out++) = x98;
	*(out++) = x98;
	*(out++) = x98;
	*(out++) = x98;
	*(out++) = x98;
	*(out++) = x109;
	*(out++) = x109;
	*(out++) = x109;
	*(out++) = x109;
	*(out++) = x109;
	*(out++) = x109;
	*(out++) = x109;
	*(out++) = x107 + x110 + x106 * x110;
	*(out++) = x107 + x111 + x106 * x111;
	*(out++) = x109 + pow(x93, 2);
	*(out++) = x108 + x106 * (1 + x103);
	*(out++) = x109 - cos(x105);
	*(out++) = x109;
	*(out++) = x109;
}

/** Applying function <function reproject_axis_x at 0x7ffa3c1c73b0> */
static inline void gen_reproject_axis_x_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p, const FLT *sensor_pt,
												   const LinmathAxisAnglePose *lh_p, const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (1));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x5 = cos(x1);
	const GEN_FLT x6 = 1 - x5;
	const GEN_FLT x7 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x11 = sin(x10);
	const GEN_FLT x12 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x13 = x12 * x11;
	const GEN_FLT x14 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x15 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x16 = cos(x10);
	const GEN_FLT x17 = 1 - x16;
	const GEN_FLT x18 = x15 * x17;
	const GEN_FLT x19 = x14 * x18;
	const GEN_FLT x20 = x15 * x11;
	const GEN_FLT x21 = x14 * x12 * x17;
	const GEN_FLT x22 =
		obj_pz + (x13 + x19) * sensor_y + (x16 + pow(x14, 2) * x17) * sensor_z + (-x20 + x21) * sensor_x;
	const GEN_FLT x23 = x14 * x11;
	const GEN_FLT x24 = x12 * x18;
	const GEN_FLT x25 =
		obj_py + (-x13 + x19) * sensor_z + (x16 + pow(x15, 2) * x17) * sensor_y + (x23 + x24) * sensor_x;
	const GEN_FLT x26 = x2 * x4;
	const GEN_FLT x27 = x0 * x8;
	const GEN_FLT x28 =
		obj_px + (x16 + pow(x12, 2) * x17) * sensor_x + (x20 + x21) * sensor_z + (-x23 + x24) * sensor_y;
	const GEN_FLT x29 = lh_py + x25 * (x5 + x6 * pow(x7, 2)) + (x26 + x27) * x28 + (-x3 + x9) * x22;
	const GEN_FLT x30 = x2 * x7;
	const GEN_FLT x31 = x0 * x4 * x6;
	const GEN_FLT x32 = lh_pz + x22 * (x5 + pow(x4, 2) * x6) + (x3 + x9) * x25 + (-x30 + x31) * x28;
	const GEN_FLT x33 = -x32;
	const GEN_FLT x34 = lh_px + x28 * (x5 + pow(x0, 2) * x6) + (-x26 + x27) * x25 + (x30 + x31) * x22;
	const GEN_FLT x35 = -phase_0 - asin(x29 * tilt_0 / sqrt(pow(x32, 2) + pow(x34, 2))) - atan2(x34, x33);
	*(out++) = x35 - cos(1.5707963267949 + gibPhase_0 + x35) * gibMag_0 + pow(atan2(x29, x33), 2) * curve_0;
}

// Jacobian of reproject_axis_x wrt [obj_px, obj_py, obj_pz, obj_qi, obj_qj, obj_qk]
static inline void gen_reproject_axis_x_jac_obj_p_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
															 const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
															 const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = cos(x1);
	const GEN_FLT x3 = 1 - x2;
	const GEN_FLT x4 = x2 + pow(x0, 2) * x3;
	const GEN_FLT x5 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x6 = pow(obj_qk, 2);
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = x6 + x7 + x8;
	const GEN_FLT x10 = sqrt(x9);
	const GEN_FLT x11 = sin(x10);
	const GEN_FLT x12 = x5 * x11;
	const GEN_FLT x13 = -x12;
	const GEN_FLT x14 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x15 = cos(x10);
	const GEN_FLT x16 = 1 - x15;
	const GEN_FLT x17 = x5 * x16;
	const GEN_FLT x18 = x14 * x17;
	const GEN_FLT x19 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x20 = x19 * x16;
	const GEN_FLT x21 = x5 * x20;
	const GEN_FLT x22 = x18 + x21;
	const GEN_FLT x23 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x24 = x23 * x17;
	const GEN_FLT x25 = x18 + x24;
	const GEN_FLT x26 = 2 * x18 * sensor_z + (x13 + x22) * sensor_x + (x12 + x25) * sensor_y;
	const GEN_FLT x27 = x4 * x26;
	const GEN_FLT x28 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (1));
	const GEN_FLT x29 = sin(x1);
	const GEN_FLT x30 = x28 * x29;
	const GEN_FLT x31 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (0));
	const GEN_FLT x32 = x0 * x3 * x31;
	const GEN_FLT x33 = x30 + x32;
	const GEN_FLT x34 = x21 + x24;
	const GEN_FLT x35 = 2 * x24 * sensor_y + (x13 + x25) * sensor_z + (x12 + x34) * sensor_x;
	const GEN_FLT x36 = x33 * x35;
	const GEN_FLT x37 = 2 * x21 * sensor_x + (x12 + x22) * sensor_z + (x13 + x34) * sensor_y;
	const GEN_FLT x38 = 1 + x37;
	const GEN_FLT x39 = x31 * x29;
	const GEN_FLT x40 = x3 * x28;
	const GEN_FLT x41 = x0 * x40;
	const GEN_FLT x42 = -x39 + x41;
	const GEN_FLT x43 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x44 = x3 * x43;
	const GEN_FLT x45 = x0 * x44;
	const GEN_FLT x46 = x44 * x28;
	const GEN_FLT x47 = x43 * x29;
	const GEN_FLT x48 = -x47;
	const GEN_FLT x49 = x46 + x48;
	const GEN_FLT x50 = x23 * x11;
	const GEN_FLT x51 = x20 * x14;
	const GEN_FLT x52 = x14 * x11;
	const GEN_FLT x53 = x23 * x20;
	const GEN_FLT x54 = pow(x19, 2);
	const GEN_FLT x55 = obj_px + (x15 + x54 * x16) * sensor_x + (x50 + x51) * sensor_z + (-x52 + x53) * sensor_y;
	const GEN_FLT x56 = x11 * x19;
	const GEN_FLT x57 = x23 * x16;
	const GEN_FLT x58 = x57 * x14;
	const GEN_FLT x59 = pow(x23, 2);
	const GEN_FLT x60 = obj_py + (x15 + x59 * x16) * sensor_y + (x52 + x53) * sensor_x + (-x56 + x58) * sensor_z;
	const GEN_FLT x61 = x44 * x31;
	const GEN_FLT x62 = x47 + x61;
	const GEN_FLT x63 = pow(x14, 2);
	const GEN_FLT x64 = obj_pz + (x15 + x63 * x16) * sensor_z + (-x50 + x51) * sensor_x + (x56 + x58) * sensor_y;
	const GEN_FLT x65 = 2 * x64 * x45 + (x45 + x49) * x55 + (x45 + x62) * x60;
	const GEN_FLT x66 = x27 + x36 + x65 + x42 * x38;
	const GEN_FLT x67 = x39 + x41;
	const GEN_FLT x68 = x0 * x29;
	const GEN_FLT x69 = x40 * x31;
	const GEN_FLT x70 = -x68 + x69;
	const GEN_FLT x71 = x2 + x3 * pow(x28, 2);
	const GEN_FLT x72 = lh_px + x64 * x67 + x70 * x60 + x71 * x55;
	const GEN_FLT x73 = lh_pz + x4 * x64 + x55 * x42 + x60 * x33;
	const GEN_FLT x74 = pow(x73, 2);
	const GEN_FLT x75 = pow(x74, -1);
	const GEN_FLT x76 = x72 * x75;
	const GEN_FLT x77 = x70 * x35;
	const GEN_FLT x78 = 2 * x55 * x46 + x64 * (x45 + x46 + x47) + (x49 + x61) * x60;
	const GEN_FLT x79 = x78 + x67 * x26;
	const GEN_FLT x80 = x77 + x79 + x71 * x38;
	const GEN_FLT x81 = pow(x73, -1);
	const GEN_FLT x82 = x74 + pow(x72, 2);
	const GEN_FLT x83 = pow(x82, -1);
	const GEN_FLT x84 = x83 * x74;
	const GEN_FLT x85 = -x30 + x32;
	const GEN_FLT x86 = x2 + x3 * pow(x31, 2);
	const GEN_FLT x87 = x68 + x69;
	const GEN_FLT x88 = lh_py + x85 * x64 + x86 * x60 + x87 * x55;
	const GEN_FLT x89 = pow(x88, 2);
	const GEN_FLT x90 = pow(1 - x83 * x89 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x91 = x86 * x35;
	const GEN_FLT x92 = x85 * x26;
	const GEN_FLT x93 = 2 * x60 * x61 + x64 * (x45 + x48 + x61) + (x46 + x62) * x55;
	const GEN_FLT x94 = x91 + x92 + x93 + x87 * x38;
	const GEN_FLT x95 = tilt_0 / sqrt(x82);
	const GEN_FLT x96 = 2 * x72;
	const GEN_FLT x97 = 2 * x73;
	const GEN_FLT x98 = (1.0 / 2.0) * x88 * tilt_0 / pow(x82, 3.0 / 2.0);
	const GEN_FLT x99 = -x90 * (x95 * x94 - (x66 * x97 + x80 * x96) * x98) - (x76 * x66 - x80 * x81) * x84;
	const GEN_FLT x100 = -x73;
	const GEN_FLT x101 = sin(1.5707963267949 + gibPhase_0 - phase_0 - asin(x88 * x95) - atan2(x72, x100)) * gibMag_0;
	const GEN_FLT x102 = x88 * x75;
	const GEN_FLT x103 = 2 * x74 * atan2(x88, x100) * curve_0 / (x74 + x89);
	const GEN_FLT x104 = 1 + x35;
	const GEN_FLT x105 = x65 + x42 * x37;
	const GEN_FLT x106 = x105 + x27 + x33 * x104;
	const GEN_FLT x107 = x71 * x37;
	const GEN_FLT x108 = x107 + x79 + x70 * x104;
	const GEN_FLT x109 = x93 + x87 * x37;
	const GEN_FLT x110 = x109 + x92 + x86 * x104;
	const GEN_FLT x111 = -x90 * (x95 * x110 - (x96 * x108 + x97 * x106) * x98) - (x76 * x106 - x81 * x108) * x84;
	const GEN_FLT x112 = 1 + x26;
	const GEN_FLT x113 = x105 + x36 + x4 * x112;
	const GEN_FLT x114 = x107 + x77 + x78 + x67 * x112;
	const GEN_FLT x115 = x109 + x91 + x85 * x112;
	const GEN_FLT x116 = -x90 * (x95 * x115 - (x96 * x114 + x97 * x113) * x98) - (x76 * x113 - x81 * x114) * x84;
	const GEN_FLT x117 = pow(x10, -1);
	const GEN_FLT x118 = x117 * obj_qi;
	const GEN_FLT x119 = x11 * x118;
	const GEN_FLT x120 = -x119;
	const GEN_FLT x121 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qi, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x122 = x16 * x121;
	const GEN_FLT x123 = x15 * x14;
	const GEN_FLT x124 = x118 * x123;
	const GEN_FLT x125 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qk * obj_qi / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x126 = x11 * x125;
	const GEN_FLT x127 = -x126;
	const GEN_FLT x128 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qj * obj_qi / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x129 = x16 * x128;
	const GEN_FLT x130 = x19 * x129;
	const GEN_FLT x131 = x56 * x23;
	const GEN_FLT x132 = x130 + x118 * x131 + x23 * x122;
	const GEN_FLT x133 = x23 * x15;
	const GEN_FLT x134 = x118 * x133;
	const GEN_FLT x135 = x11 * x128;
	const GEN_FLT x136 = x56 * x14;
	const GEN_FLT x137 = x16 * x125;
	const GEN_FLT x138 = x19 * x137;
	const GEN_FLT x139 = x138 + x118 * x136 + x14 * x122;
	const GEN_FLT x140 = (x120 + 2 * x19 * x122 + x54 * x119) * sensor_x + (-x124 + x127 + x132) * sensor_y +
						 (x134 + x135 + x139) * sensor_z;
	const GEN_FLT x141 = x23 * x129;
	const GEN_FLT x142 = x15 * x19;
	const GEN_FLT x143 = x118 * x142;
	const GEN_FLT x144 = x11 * x121;
	const GEN_FLT x145 = x14 * x129;
	const GEN_FLT x146 = x23 * x137;
	const GEN_FLT x147 = x50 * x14;
	const GEN_FLT x148 = x145 + x146 + x118 * x147;
	const GEN_FLT x149 =
		(x120 + 2 * x141 + x59 * x119) * sensor_y + (x124 + x126 + x132) * sensor_x + (-x143 - x144 + x148) * sensor_z;
	const GEN_FLT x150 = -x135;
	const GEN_FLT x151 = x14 * x137;
	const GEN_FLT x152 =
		(x120 + 2 * x151 + x63 * x119) * sensor_z + (-x134 + x139 + x150) * sensor_x + (x143 + x144 + x148) * sensor_y;
	const GEN_FLT x153 = x65 + x33 * x149 + x4 * x152 + x42 * x140;
	const GEN_FLT x154 = x78 + x67 * x152 + x70 * x149 + x71 * x140;
	const GEN_FLT x155 = x93 + x85 * x152 + x86 * x149 + x87 * x140;
	const GEN_FLT x156 = -x90 * (x95 * x155 - (x96 * x154 + x97 * x153) * x98) - (x76 * x153 - x81 * x154) * x84;
	const GEN_FLT x157 = x117 * obj_qj;
	const GEN_FLT x158 = x123 * x157;
	const GEN_FLT x159 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qk * obj_qj / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x160 = x11 * x159;
	const GEN_FLT x161 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qj, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x162 = x141 + x131 * x157 + x20 * x161;
	const GEN_FLT x163 = x11 * x157;
	const GEN_FLT x164 = -x163;
	const GEN_FLT x165 = x142 * x157;
	const GEN_FLT x166 = x14 * x16;
	const GEN_FLT x167 = x57 * x159;
	const GEN_FLT x168 = x167 + x147 * x157 + x161 * x166;
	const GEN_FLT x169 = (x158 + x160 + x162) * sensor_x + (x164 + 2 * x57 * x161 + x59 * x163) * sensor_y +
						 (x150 - x165 + x168) * sensor_z;
	const GEN_FLT x170 = -x160;
	const GEN_FLT x171 = x133 * x157;
	const GEN_FLT x172 = x11 * x161;
	const GEN_FLT x173 = x20 * x159;
	const GEN_FLT x174 = x145 + x173 + x136 * x157;
	const GEN_FLT x175 =
		(-x158 + x162 + x170) * sensor_y + (2 * x130 + x164 + x54 * x163) * sensor_x + (x171 + x172 + x174) * sensor_z;
	const GEN_FLT x176 = x166 * x159;
	const GEN_FLT x177 =
		(x164 + 2 * x176 + x63 * x163) * sensor_z + (x135 + x165 + x168) * sensor_y + (-x171 - x172 + x174) * sensor_x;
	const GEN_FLT x178 = x65 + x33 * x169 + x4 * x177 + x42 * x175;
	const GEN_FLT x179 = x78 + x67 * x177 + x70 * x169 + x71 * x175;
	const GEN_FLT x180 = x93 + x85 * x177 + x86 * x169 + x87 * x175;
	const GEN_FLT x181 = -x90 * (x95 * x180 - (x96 * x179 + x97 * x178) * x98) - (x76 * x178 - x81 * x179) * x84;
	const GEN_FLT x182 = x117 * obj_qk;
	const GEN_FLT x183 = x123 * x182;
	const GEN_FLT x184 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qk, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x185 = x11 * x184;
	const GEN_FLT x186 = x146 + x173 + x182 * x131;
	const GEN_FLT x187 = x11 * x182;
	const GEN_FLT x188 = -x187;
	const GEN_FLT x189 = x182 * x142;
	const GEN_FLT x190 = x16 * x184;
	const GEN_FLT x191 = x176 + x182 * x147 + x23 * x190;
	const GEN_FLT x192 =
		(x127 - x189 + x191) * sensor_z + (2 * x167 + x188 + x59 * x187) * sensor_y + (x183 + x185 + x186) * sensor_x;
	const GEN_FLT x193 = x182 * x133;
	const GEN_FLT x194 = x151 + x182 * x136 + x19 * x190;
	const GEN_FLT x195 =
		(x160 + x193 + x194) * sensor_z + (-x183 - x185 + x186) * sensor_y + (2 * x138 + x188 + x54 * x187) * sensor_x;
	const GEN_FLT x196 = (x126 + x189 + x191) * sensor_y + (x170 - x193 + x194) * sensor_x +
						 (x188 + 2 * x14 * x190 + x63 * x187) * sensor_z;
	const GEN_FLT x197 = x65 + x33 * x192 + x4 * x196 + x42 * x195;
	const GEN_FLT x198 = x78 + x67 * x196 + x70 * x192 + x71 * x195;
	const GEN_FLT x199 = x93 + x85 * x196 + x86 * x192 + x87 * x195;
	const GEN_FLT x200 = -x90 * (x95 * x199 - (x96 * x198 + x97 * x197) * x98) - (x76 * x197 - x81 * x198) * x84;
	*(out++) = x99 + x103 * (x66 * x102 - x81 * x94) + x99 * x101;
	*(out++) = x111 + x101 * x111 + x103 * (x102 * x106 - x81 * x110);
	*(out++) = x116 + x101 * x116 + x103 * (x102 * x113 - x81 * x115);
	*(out++) = x156 + x101 * x156 + x103 * (x102 * x153 - x81 * x155);
	*(out++) = x181 + x101 * x181 + x103 * (x102 * x178 - x81 * x180);
	*(out++) = x200 + x103 * (x102 * x197 - x81 * x199) + x200 * x101;
}

// Jacobian of reproject_axis_x wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_reproject_axis_x_jac_sensor_pt_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
																 const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
																 const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x5 = cos(x1);
	const GEN_FLT x6 = 1 - x5;
	const GEN_FLT x7 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (1));
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = -x3 + x9;
	const GEN_FLT x11 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x12 = cos(x11);
	const GEN_FLT x13 = 1 - x12;
	const GEN_FLT x14 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x15 = x12 + pow(x14, 2) * x13;
	const GEN_FLT x16 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x17 = x14 * x13;
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x20 = x13 * x19;
	const GEN_FLT x21 = x20 * x16;
	const GEN_FLT x22 = sin(x11);
	const GEN_FLT x23 = x22 * x16;
	const GEN_FLT x24 = -x23;
	const GEN_FLT x25 = x18 + x24;
	const GEN_FLT x26 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x27 = x26 * x13 * x16;
	const GEN_FLT x28 = 2 * x18 * sensor_x + (x21 + x25) * sensor_y + (x18 + x23 + x27) * sensor_z;
	const GEN_FLT x29 = x15 + x28;
	const GEN_FLT x30 = x22 * x26;
	const GEN_FLT x31 = x20 * x14;
	const GEN_FLT x32 = x30 + x31;
	const GEN_FLT x33 = x21 + x23;
	const GEN_FLT x34 = 2 * x21 * sensor_y + (x18 + x33) * sensor_x + (x21 + x24 + x27) * sensor_z;
	const GEN_FLT x35 = x32 + x34;
	const GEN_FLT x36 = x2 * x7;
	const GEN_FLT x37 = x0 * x4 * x6;
	const GEN_FLT x38 = x36 + x37;
	const GEN_FLT x39 = x22 * x19;
	const GEN_FLT x40 = x26 * x17;
	const GEN_FLT x41 = -x39 + x40;
	const GEN_FLT x42 = 2 * x27 * sensor_z + (x25 + x27) * sensor_x + (x27 + x33) * sensor_y;
	const GEN_FLT x43 = x41 + x42;
	const GEN_FLT x44 = x5 + pow(x4, 2) * x6;
	const GEN_FLT x45 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x46 = x2 * x45;
	const GEN_FLT x47 = -x46;
	const GEN_FLT x48 = x6 * x45;
	const GEN_FLT x49 = x7 * x48;
	const GEN_FLT x50 = x4 * x48;
	const GEN_FLT x51 = x49 + x50;
	const GEN_FLT x52 = x39 + x40;
	const GEN_FLT x53 = -x30 + x31;
	const GEN_FLT x54 = obj_px + x15 * sensor_x + x52 * sensor_z + x53 * sensor_y;
	const GEN_FLT x55 = x22 * x14;
	const GEN_FLT x56 = x20 * x26;
	const GEN_FLT x57 = -x55 + x56;
	const GEN_FLT x58 = x12 + x13 * pow(x19, 2);
	const GEN_FLT x59 = obj_py + x32 * sensor_x + x57 * sensor_z + x58 * sensor_y;
	const GEN_FLT x60 = x0 * x48;
	const GEN_FLT x61 = x46 + x60;
	const GEN_FLT x62 = x55 + x56;
	const GEN_FLT x63 = x12 + pow(x26, 2) * x13;
	const GEN_FLT x64 = obj_pz + x41 * sensor_x + x62 * sensor_y + x63 * sensor_z;
	const GEN_FLT x65 = 2 * x64 * x50 + (x47 + x51) * x54 + (x50 + x61) * x59;
	const GEN_FLT x66 = x65 + x29 * x10 + x35 * x38 + x43 * x44;
	const GEN_FLT x67 = x3 + x9;
	const GEN_FLT x68 = x2 * x4;
	const GEN_FLT x69 = x0 * x8;
	const GEN_FLT x70 = -x68 + x69;
	const GEN_FLT x71 = x5 + x6 * pow(x7, 2);
	const GEN_FLT x72 = lh_px + x64 * x67 + x70 * x59 + x71 * x54;
	const GEN_FLT x73 = lh_pz + x54 * x10 + x59 * x38 + x64 * x44;
	const GEN_FLT x74 = pow(x73, 2);
	const GEN_FLT x75 = pow(x74, -1);
	const GEN_FLT x76 = x72 * x75;
	const GEN_FLT x77 = pow(x73, -1);
	const GEN_FLT x78 = x47 + x60;
	const GEN_FLT x79 = 2 * x54 * x49 + (x46 + x51) * x64 + (x49 + x78) * x59;
	const GEN_FLT x80 = x79 + x67 * x43 + x70 * x35 + x71 * x29;
	const GEN_FLT x81 = x74 + pow(x72, 2);
	const GEN_FLT x82 = pow(x81, -1);
	const GEN_FLT x83 = x82 * x74;
	const GEN_FLT x84 = x68 + x69;
	const GEN_FLT x85 = x5 + pow(x0, 2) * x6;
	const GEN_FLT x86 = -x36 + x37;
	const GEN_FLT x87 = 2 * x60 * x59 + (x49 + x61) * x54 + (x50 + x78) * x64;
	const GEN_FLT x88 = x87 + x84 * x29 + x85 * x35 + x86 * x43;
	const GEN_FLT x89 = tilt_0 / sqrt(x81);
	const GEN_FLT x90 = 2 * x72;
	const GEN_FLT x91 = 2 * x73;
	const GEN_FLT x92 = lh_py + x84 * x54 + x85 * x59 + x86 * x64;
	const GEN_FLT x93 = (1.0 / 2.0) * x92 * tilt_0 / pow(x81, 3.0 / 2.0);
	const GEN_FLT x94 = pow(x92, 2);
	const GEN_FLT x95 = pow(1 - x82 * x94 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x96 = -x95 * (x88 * x89 - (x66 * x91 + x80 * x90) * x93) - (x76 * x66 - x80 * x77) * x83;
	const GEN_FLT x97 = -x73;
	const GEN_FLT x98 = sin(1.5707963267949 + gibPhase_0 - phase_0 - asin(x89 * x92) - atan2(x72, x97)) * gibMag_0;
	const GEN_FLT x99 = x75 * x92;
	const GEN_FLT x100 = 2 * x74 * atan2(x92, x97) * curve_0 / (x74 + x94);
	const GEN_FLT x101 = x28 + x53;
	const GEN_FLT x102 = x34 + x58;
	const GEN_FLT x103 = x42 + x62;
	const GEN_FLT x104 = x65 + x10 * x101 + x38 * x102 + x44 * x103;
	const GEN_FLT x105 = x79 + x67 * x103 + x70 * x102 + x71 * x101;
	const GEN_FLT x106 = x87 + x84 * x101 + x85 * x102 + x86 * x103;
	const GEN_FLT x107 = -x95 * (x89 * x106 - (x90 * x105 + x91 * x104) * x93) - (x76 * x104 - x77 * x105) * x83;
	const GEN_FLT x108 = x34 + x57;
	const GEN_FLT x109 = x28 + x52;
	const GEN_FLT x110 = x42 + x63;
	const GEN_FLT x111 = x65 + x10 * x109 + x38 * x108 + x44 * x110;
	const GEN_FLT x112 = x79 + x67 * x110 + x70 * x108 + x71 * x109;
	const GEN_FLT x113 = x87 + x84 * x109 + x85 * x108 + x86 * x110;
	const GEN_FLT x114 = -x95 * (x89 * x113 - (x90 * x112 + x91 * x111) * x93) - (x76 * x111 - x77 * x112) * x83;
	*(out++) = x96 + x98 * x96 + (x66 * x99 - x88 * x77) * x100;
	*(out++) = x107 + x98 * x107 + (-x77 * x106 + x99 * x104) * x100;
	*(out++) = x114 + x98 * x114 + (-x77 * x113 + x99 * x111) * x100;
}

// Jacobian of reproject_axis_x wrt [lh_px, lh_py, lh_pz, lh_qi, lh_qj, lh_qk]
static inline void gen_reproject_axis_x_jac_lh_p_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
															const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
															const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x1 = pow(lh_qk, 2);
	const GEN_FLT x2 = pow(lh_qj, 2);
	const GEN_FLT x3 = pow(lh_qi, 2);
	const GEN_FLT x4 = x1 + x2 + x3;
	const GEN_FLT x5 = sqrt(x4);
	const GEN_FLT x6 = sin(x5);
	const GEN_FLT x7 = x0 * x6;
	const GEN_FLT x8 = -x7;
	const GEN_FLT x9 = cos(x5);
	const GEN_FLT x10 = 1 - x9;
	const GEN_FLT x11 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (0));
	const GEN_FLT x12 = x11 * x10;
	const GEN_FLT x13 = x0 * x12;
	const GEN_FLT x14 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (1));
	const GEN_FLT x15 = x14 * x10;
	const GEN_FLT x16 = x0 * x15;
	const GEN_FLT x17 = x13 + x16;
	const GEN_FLT x18 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x19 = sin(x18);
	const GEN_FLT x20 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x23 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x24 = cos(x18);
	const GEN_FLT x25 = 1 - x24;
	const GEN_FLT x26 = x25 * x23;
	const GEN_FLT x27 = x22 * x26;
	const GEN_FLT x28 = x22 * x19;
	const GEN_FLT x29 = x20 * x26;
	const GEN_FLT x30 =
		obj_px + (x21 + x27) * sensor_z + (x24 + x25 * pow(x23, 2)) * sensor_x + (-x28 + x29) * sensor_y;
	const GEN_FLT x31 = x23 * x19;
	const GEN_FLT x32 = x25 * x22 * x20;
	const GEN_FLT x33 =
		obj_py + (x24 + x25 * pow(x20, 2)) * sensor_y + (x28 + x29) * sensor_x + (-x31 + x32) * sensor_z;
	const GEN_FLT x34 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (0));
	const GEN_FLT x35 = x34 * x10;
	const GEN_FLT x36 = x0 * x35;
	const GEN_FLT x37 = x13 + x36;
	const GEN_FLT x38 =
		obj_pz + (-x21 + x27) * sensor_x + (x24 + x25 * pow(x22, 2)) * sensor_z + (x31 + x32) * sensor_y;
	const GEN_FLT x39 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x40 = x39 * x25;
	const GEN_FLT x41 = x40 * x23;
	const GEN_FLT x42 = x39 * x19;
	const GEN_FLT x43 = -x42;
	const GEN_FLT x44 = x40 * x20;
	const GEN_FLT x45 = x41 + x44;
	const GEN_FLT x46 = x40 * x22;
	const GEN_FLT x47 = x41 + x46;
	const GEN_FLT x48 = 2 * x41 * sensor_x + (x42 + x47) * sensor_z + (x43 + x45) * sensor_y;
	const GEN_FLT x49 = x6 * x34;
	const GEN_FLT x50 = x15 * x11;
	const GEN_FLT x51 = -x49 + x50;
	const GEN_FLT x52 = x6 * x14;
	const GEN_FLT x53 = x34 * x12;
	const GEN_FLT x54 = x52 + x53;
	const GEN_FLT x55 = x44 + x46;
	const GEN_FLT x56 = 2 * x44 * sensor_y + (x42 + x45) * sensor_x + (x43 + x55) * sensor_z;
	const GEN_FLT x57 = pow(x11, 2);
	const GEN_FLT x58 = x9 + x57 * x10;
	const GEN_FLT x59 = 2 * x46 * sensor_z + (x43 + x47) * sensor_x + (x42 + x55) * sensor_y;
	const GEN_FLT x60 = x51 * x48 + x54 * x56 + x58 * x59;
	const GEN_FLT x61 = x60 + x30 * (x17 + x8) + x33 * (x37 + x7) + 2 * x38 * x13;
	const GEN_FLT x62 = lh_pz + x51 * x30 + x54 * x33 + x58 * x38;
	const GEN_FLT x63 = pow(x62, 2);
	const GEN_FLT x64 = pow(x63, -1);
	const GEN_FLT x65 = x49 + x50;
	const GEN_FLT x66 = x6 * x11;
	const GEN_FLT x67 = x34 * x15;
	const GEN_FLT x68 = -x66 + x67;
	const GEN_FLT x69 = pow(x14, 2);
	const GEN_FLT x70 = x9 + x69 * x10;
	const GEN_FLT x71 = lh_px + x65 * x38 + x68 * x33 + x70 * x30;
	const GEN_FLT x72 = x71 * x64;
	const GEN_FLT x73 = x72 * x61;
	const GEN_FLT x74 = x16 + x36;
	const GEN_FLT x75 = x65 * x59 + x68 * x56 + x70 * x48;
	const GEN_FLT x76 = x75 + 2 * x30 * x16 + x33 * (x74 + x8) + x38 * (x17 + x7);
	const GEN_FLT x77 = 1 + x76;
	const GEN_FLT x78 = pow(x62, -1);
	const GEN_FLT x79 = x63 + pow(x71, 2);
	const GEN_FLT x80 = pow(x79, -1);
	const GEN_FLT x81 = x80 * x63;
	const GEN_FLT x82 = -x52 + x53;
	const GEN_FLT x83 = pow(x34, 2);
	const GEN_FLT x84 = x9 + x83 * x10;
	const GEN_FLT x85 = x66 + x67;
	const GEN_FLT x86 = lh_py + x82 * x38 + x84 * x33 + x85 * x30;
	const GEN_FLT x87 = pow(x86, 2);
	const GEN_FLT x88 = pow(1 - x80 * x87 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x89 = x82 * x59 + x84 * x56 + x85 * x48;
	const GEN_FLT x90 = x89 + x30 * (x7 + x74) + 2 * x33 * x36 + x38 * (x37 + x8);
	const GEN_FLT x91 = tilt_0 / sqrt(x79);
	const GEN_FLT x92 = x91 * x90;
	const GEN_FLT x93 = 2 * x71;
	const GEN_FLT x94 = 2 * x62;
	const GEN_FLT x95 = x61 * x94;
	const GEN_FLT x96 = (1.0 / 2.0) * x86 * tilt_0 / pow(x79, 3.0 / 2.0);
	const GEN_FLT x97 = -x81 * (x73 - x78 * x77) - x88 * (x92 - x96 * (x95 + x77 * x93));
	const GEN_FLT x98 = -x62;
	const GEN_FLT x99 = sin(1.5707963267949 + gibPhase_0 - phase_0 - asin(x86 * x91) - atan2(x71, x98)) * gibMag_0;
	const GEN_FLT x100 = x86 * x64;
	const GEN_FLT x101 = x61 * x100;
	const GEN_FLT x102 = -x78 * x90;
	const GEN_FLT x103 = 2 * x63 * atan2(x86, x98) * curve_0 / (x63 + x87);
	const GEN_FLT x104 = -x78 * x76;
	const GEN_FLT x105 = 1 + x90;
	const GEN_FLT x106 = x76 * x93;
	const GEN_FLT x107 = -x81 * (x104 + x73) - x88 * (x91 * x105 - x96 * (x106 + x95));
	const GEN_FLT x108 = 1 + x61;
	const GEN_FLT x109 = -x81 * (x104 + x72 * x108) - x88 * (x92 - x96 * (x106 + x94 * x108));
	const GEN_FLT x110 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qj * lh_qi / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x111 = x6 * x110;
	const GEN_FLT x112 = -x111;
	const GEN_FLT x113 = pow(x5, -1);
	const GEN_FLT x114 = x113 * lh_qi;
	const GEN_FLT x115 = x9 * x34;
	const GEN_FLT x116 = x114 * x115;
	const GEN_FLT x117 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qi, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x118 = x66 * x14;
	const GEN_FLT x119 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qk * lh_qi / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x120 = x15 * x119;
	const GEN_FLT x121 = x120 + x118 * x114 + x12 * x117;
	const GEN_FLT x122 = x6 * x117;
	const GEN_FLT x123 = x9 * x14;
	const GEN_FLT x124 = x114 * x123;
	const GEN_FLT x125 = x12 * x110;
	const GEN_FLT x126 = x35 * x119;
	const GEN_FLT x127 = x34 * x114;
	const GEN_FLT x128 = x125 + x126 + x66 * x127;
	const GEN_FLT x129 = x6 * x114;
	const GEN_FLT x130 = -x129;
	const GEN_FLT x131 = x12 * x119;
	const GEN_FLT x132 =
		x60 + x30 * (x112 - x116 + x121) + x33 * (x122 + x124 + x128) + x38 * (x130 + 2 * x131 + x57 * x129);
	const GEN_FLT x133 = x6 * x119;
	const GEN_FLT x134 = -x133;
	const GEN_FLT x135 = x9 * x11;
	const GEN_FLT x136 = x114 * x135;
	const GEN_FLT x137 = x15 * x110;
	const GEN_FLT x138 = x137 + x35 * x117 + x52 * x127;
	const GEN_FLT x139 =
		x75 + x30 * (x130 + 2 * x15 * x117 + x69 * x129) + x33 * (x134 - x136 + x138) + x38 * (x111 + x116 + x121);
	const GEN_FLT x140 = x35 * x110;
	const GEN_FLT x141 =
		x89 + x30 * (x133 + x136 + x138) + x33 * (x130 + 2 * x140 + x83 * x129) + x38 * (-x122 - x124 + x128);
	const GEN_FLT x142 = -x88 * (x91 * x141 - (x93 * x139 + x94 * x132) * x96) - (x72 * x132 - x78 * x139) * x81;
	const GEN_FLT x143 = x113 * lh_qj;
	const GEN_FLT x144 = x123 * x143;
	const GEN_FLT x145 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qk * lh_qj / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x146 = x35 * x145;
	const GEN_FLT x147 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qj, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x148 = x66 * x34;
	const GEN_FLT x149 = x146 + x12 * x147 + x143 * x148;
	const GEN_FLT x150 = x6 * x147;
	const GEN_FLT x151 = x115 * x143;
	const GEN_FLT x152 = x15 * x145;
	const GEN_FLT x153 = x125 + x152 + x118 * x143;
	const GEN_FLT x154 = x6 * x143;
	const GEN_FLT x155 = -x154;
	const GEN_FLT x156 = x12 * x145;
	const GEN_FLT x157 =
		x60 + x30 * (-x150 - x151 + x153) + x33 * (x111 + x144 + x149) + x38 * (x155 + 2 * x156 + x57 * x154);
	const GEN_FLT x158 = x64 * x157;
	const GEN_FLT x159 = x6 * x145;
	const GEN_FLT x160 = -x159;
	const GEN_FLT x161 = x135 * x143;
	const GEN_FLT x162 = x52 * x34;
	const GEN_FLT x163 = x140 + x15 * x147 + x162 * x143;
	const GEN_FLT x164 =
		x75 + x30 * (2 * x137 + x155 + x69 * x154) + x33 * (x160 - x161 + x163) + x38 * (x150 + x151 + x153);
	const GEN_FLT x165 =
		x89 + x30 * (x159 + x161 + x163) + x33 * (x155 + 2 * x35 * x147 + x83 * x154) + x38 * (x112 - x144 + x149);
	const GEN_FLT x166 = -x88 * (x91 * x165 - (x93 * x164 + x94 * x157) * x96) - (x71 * x158 - x78 * x164) * x81;
	const GEN_FLT x167 = x113 * lh_qk;
	const GEN_FLT x168 = x115 * x167;
	const GEN_FLT x169 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qk, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x170 = x131 + x118 * x167 + x15 * x169;
	const GEN_FLT x171 = x123 * x167;
	const GEN_FLT x172 = x156 + x167 * x148 + x35 * x169;
	const GEN_FLT x173 = x6 * x167;
	const GEN_FLT x174 = -x173;
	const GEN_FLT x175 =
		x60 + x30 * (x160 - x168 + x170) + x33 * (x133 + x171 + x172) + x38 * (x174 + 2 * x12 * x169 + x57 * x173);
	const GEN_FLT x176 = x6 * x169;
	const GEN_FLT x177 = x167 * x135;
	const GEN_FLT x178 = x126 + x152 + x167 * x162;
	const GEN_FLT x179 =
		x75 + x30 * (2 * x120 + x174 + x69 * x173) + x33 * (-x176 - x177 + x178) + x38 * (x159 + x168 + x170);
	const GEN_FLT x180 =
		x89 + x30 * (x176 + x177 + x178) + x33 * (2 * x146 + x174 + x83 * x173) + x38 * (x134 - x171 + x172);
	const GEN_FLT x181 = -x88 * (x91 * x180 - (x93 * x179 + x94 * x175) * x96) - (x72 * x175 - x78 * x179) * x81;
	*(out++) = x97 + x99 * x97 + (x101 + x102) * x103;
	*(out++) = x107 + x103 * (x101 - x78 * x105) + x99 * x107;
	*(out++) = x109 + x103 * (x102 + x100 * x108) + x99 * x109;
	*(out++) = x142 + x103 * (x100 * x132 - x78 * x141) + x99 * x142;
	*(out++) = x166 + x99 * x166 + (-x78 * x165 + x86 * x158) * x103;
	*(out++) = x181 + x103 * (x100 * x175 - x78 * x180) + x99 * x181;
}

// Jacobian of reproject_axis_x wrt [phase_0, tilt_0, curve_0, gibPhase_0, gibMag_0, ogeeMag_0, ogeePhase_0]
static inline void gen_reproject_axis_x_jac_bsc0_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
															const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
															const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (1));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x5 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x6 = cos(x1);
	const GEN_FLT x7 = 1 - x6;
	const GEN_FLT x8 = x4 * x5 * x7;
	const GEN_FLT x9 = -x3 + x8;
	const GEN_FLT x10 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x11 = sin(x10);
	const GEN_FLT x12 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x13 = x12 * x11;
	const GEN_FLT x14 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x15 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x16 = cos(x10);
	const GEN_FLT x17 = 1 - x16;
	const GEN_FLT x18 = x15 * x17;
	const GEN_FLT x19 = x14 * x18;
	const GEN_FLT x20 = x15 * x11;
	const GEN_FLT x21 = x12 * x17;
	const GEN_FLT x22 = x21 * x14;
	const GEN_FLT x23 =
		obj_pz + (x13 + x19) * sensor_y + (x16 + pow(x14, 2) * x17) * sensor_z + (-x20 + x22) * sensor_x;
	const GEN_FLT x24 = x14 * x11;
	const GEN_FLT x25 = x21 * x15;
	const GEN_FLT x26 =
		obj_py + (-x13 + x19) * sensor_z + (x16 + pow(x15, 2) * x17) * sensor_y + (x24 + x25) * sensor_x;
	const GEN_FLT x27 = x6 + pow(x5, 2) * x7;
	const GEN_FLT x28 = x2 * x4;
	const GEN_FLT x29 = x0 * x7;
	const GEN_FLT x30 = x5 * x29;
	const GEN_FLT x31 = x28 + x30;
	const GEN_FLT x32 =
		obj_px + (x16 + pow(x12, 2) * x17) * sensor_x + (x20 + x22) * sensor_z + (-x24 + x25) * sensor_y;
	const GEN_FLT x33 = lh_py + x26 * x27 + x32 * x31 + x9 * x23;
	const GEN_FLT x34 = pow(x33, 2);
	const GEN_FLT x35 = x6 + pow(x4, 2) * x7;
	const GEN_FLT x36 = x3 + x8;
	const GEN_FLT x37 = x2 * x5;
	const GEN_FLT x38 = x4 * x29;
	const GEN_FLT x39 = -x37 + x38;
	const GEN_FLT x40 = lh_pz + x32 * x39 + x35 * x23 + x36 * x26;
	const GEN_FLT x41 = pow(x40, 2);
	const GEN_FLT x42 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x43 = x2 * x42;
	const GEN_FLT x44 = -x43;
	const GEN_FLT x45 = x7 * x42;
	const GEN_FLT x46 = x4 * x45;
	const GEN_FLT x47 = x0 * x45;
	const GEN_FLT x48 = x46 + x47;
	const GEN_FLT x49 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x50 = x49 * x21;
	const GEN_FLT x51 = x49 * x18;
	const GEN_FLT x52 = x49 * x11;
	const GEN_FLT x53 = x51 + x52;
	const GEN_FLT x54 = x49 * x14 * x17;
	const GEN_FLT x55 = -x52;
	const GEN_FLT x56 = x54 + x55;
	const GEN_FLT x57 = 2 * x51 * sensor_y + (x50 + x53) * sensor_x + (x51 + x56) * sensor_z;
	const GEN_FLT x58 = x5 * x45;
	const GEN_FLT x59 = x46 + x58;
	const GEN_FLT x60 = 2 * x50 * sensor_x + (x50 + x51 + x55) * sensor_y + (x50 + x52 + x54) * sensor_z;
	const GEN_FLT x61 = 2 * x54 * sensor_z + (x50 + x56) * sensor_x + (x53 + x54) * sensor_y;
	const GEN_FLT x62 = 2 * x46 * x23 + x57 * x36 + x60 * x39 + x61 * x35 + (x44 + x48) * x32 + (x43 + x59) * x26;
	const GEN_FLT x63 = x62 / x41;
	const GEN_FLT x64 = x47 + x58;
	const GEN_FLT x65 = x57 * x27 + 2 * x58 * x26 + x60 * x31 + x9 * x61 + (x44 + x59) * x23 + (x43 + x64) * x32;
	const GEN_FLT x66 = pow(x40, -1);
	const GEN_FLT x67 = -x40;
	const GEN_FLT x68 = atan2(x33, x67);
	const GEN_FLT x69 = 2 * (x63 * x33 - x65 * x66) * x68 * x41 * curve_0 / (x34 + x41);
	const GEN_FLT x70 = x37 + x38;
	const GEN_FLT x71 = -x28 + x30;
	const GEN_FLT x72 = x6 + pow(x0, 2) * x7;
	const GEN_FLT x73 = lh_px + x70 * x23 + x71 * x26 + x72 * x32;
	const GEN_FLT x74 = x41 + pow(x73, 2);
	const GEN_FLT x75 = pow(x74, -1);
	const GEN_FLT x76 = 2 * x47 * x32 + x70 * x61 + x71 * x57 + x72 * x60 + (x43 + x48) * x23 + (x44 + x64) * x26;
	const GEN_FLT x77 = -(x73 * x63 - x76 * x66) * x75 * x41;
	const GEN_FLT x78 = pow(1 - x75 * x34 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x79 = pow(x74, -1.0 / 2.0);
	const GEN_FLT x80 =
		x79 * x65 * tilt_0 + (-1.0 / 2.0) * (2 * x62 * x40 + 2 * x73 * x76) * x33 * tilt_0 / pow(x74, 3.0 / 2.0);
	const GEN_FLT x81 = x77 - x80 * x78;
	const GEN_FLT x82 = -1 + x81;
	const GEN_FLT x83 = x79 * x33;
	const GEN_FLT x84 = 1.5707963267949 + gibPhase_0 - phase_0 - asin(x83 * tilt_0) - atan2(x73, x67);
	const GEN_FLT x85 = sin(x84) * gibMag_0;
	const GEN_FLT x86 = x77 - (x80 + x83) * x78;
	const GEN_FLT x87 = x69 + x81;
	const GEN_FLT x88 = x87 + x81 * x85;
	*(out++) = x69 + x82 + x82 * x85;
	*(out++) = x69 + x86 + x85 * x86;
	*(out++) = x88 + pow(x68, 2);
	*(out++) = x87 + x85 * (1 + x81);
	*(out++) = x88 - cos(x84);
	*(out++) = x88;
	*(out++) = x88;
}

/** Applying function <function reproject_axis_y at 0x7ffa3c1c7440> */
static inline void gen_reproject_axis_y_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p, const FLT *sensor_pt,
												   const LinmathAxisAnglePose *lh_p, const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (1));
	const GEN_FLT x5 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x6 = cos(x1);
	const GEN_FLT x7 = 1 - x6;
	const GEN_FLT x8 = x4 * x5 * x7;
	const GEN_FLT x9 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x10 = sin(x9);
	const GEN_FLT x11 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x12 = x11 * x10;
	const GEN_FLT x13 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x14 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x15 = cos(x9);
	const GEN_FLT x16 = 1 - x15;
	const GEN_FLT x17 = x14 * x16;
	const GEN_FLT x18 = x13 * x17;
	const GEN_FLT x19 = x14 * x10;
	const GEN_FLT x20 = x13 * x11 * x16;
	const GEN_FLT x21 =
		obj_pz + (x12 + x18) * sensor_y + (x15 + pow(x13, 2) * x16) * sensor_z + (-x19 + x20) * sensor_x;
	const GEN_FLT x22 = x2 * x5;
	const GEN_FLT x23 = x0 * x7;
	const GEN_FLT x24 = x4 * x23;
	const GEN_FLT x25 = x13 * x10;
	const GEN_FLT x26 = x11 * x17;
	const GEN_FLT x27 =
		obj_py + (-x12 + x18) * sensor_z + (x15 + pow(x14, 2) * x16) * sensor_y + (x25 + x26) * sensor_x;
	const GEN_FLT x28 =
		obj_px + (x15 + pow(x11, 2) * x16) * sensor_x + (x19 + x20) * sensor_z + (-x25 + x26) * sensor_y;
	const GEN_FLT x29 = lh_px + x28 * (x6 + pow(x4, 2) * x7) + (-x22 + x24) * x27 + (x3 + x8) * x21;
	const GEN_FLT x30 = x2 * x4;
	const GEN_FLT x31 = x5 * x23;
	const GEN_FLT x32 = lh_pz + x21 * (x6 + pow(x5, 2) * x7) + (-x3 + x8) * x28 + (x30 + x31) * x27;
	const GEN_FLT x33 = -x32;
	const GEN_FLT x34 = lh_py + x27 * (x6 + pow(x0, 2) * x7) + (x22 + x24) * x28 + (-x30 + x31) * x21;
	const GEN_FLT x35 = -phase_1 - asin(x29 * tilt_1 / sqrt(pow(x32, 2) + pow(x34, 2))) - atan2(-x34, x33);
	*(out++) = x35 - cos(1.5707963267949 + gibPhase_1 + x35) * gibMag_1 + pow(atan2(x29, x33), 2) * curve_1;
}

// Jacobian of reproject_axis_y wrt [obj_px, obj_py, obj_pz, obj_qi, obj_qj, obj_qk]
static inline void gen_reproject_axis_y_jac_obj_p_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
															 const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
															 const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = cos(x1);
	const GEN_FLT x3 = 1 - x2;
	const GEN_FLT x4 = x2 + pow(x0, 2) * x3;
	const GEN_FLT x5 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x6 = pow(obj_qk, 2);
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = x6 + x7 + x8;
	const GEN_FLT x10 = sqrt(x9);
	const GEN_FLT x11 = sin(x10);
	const GEN_FLT x12 = x5 * x11;
	const GEN_FLT x13 = -x12;
	const GEN_FLT x14 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x15 = cos(x10);
	const GEN_FLT x16 = 1 - x15;
	const GEN_FLT x17 = x14 * x16;
	const GEN_FLT x18 = x5 * x17;
	const GEN_FLT x19 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x20 = x19 * x16;
	const GEN_FLT x21 = x5 * x20;
	const GEN_FLT x22 = x18 + x21;
	const GEN_FLT x23 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x24 = x23 * x16;
	const GEN_FLT x25 = x5 * x24;
	const GEN_FLT x26 = x18 + x25;
	const GEN_FLT x27 = 2 * x18 * sensor_z + (x13 + x22) * sensor_x + (x12 + x26) * sensor_y;
	const GEN_FLT x28 = x4 * x27;
	const GEN_FLT x29 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (1));
	const GEN_FLT x30 = sin(x1);
	const GEN_FLT x31 = x30 * x29;
	const GEN_FLT x32 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (0));
	const GEN_FLT x33 = x3 * x32;
	const GEN_FLT x34 = x0 * x33;
	const GEN_FLT x35 = x31 + x34;
	const GEN_FLT x36 = x21 + x25;
	const GEN_FLT x37 = 2 * x25 * sensor_y + (x13 + x26) * sensor_z + (x12 + x36) * sensor_x;
	const GEN_FLT x38 = x35 * x37;
	const GEN_FLT x39 = 2 * x21 * sensor_x + (x12 + x22) * sensor_z + (x13 + x36) * sensor_y;
	const GEN_FLT x40 = 1 + x39;
	const GEN_FLT x41 = x30 * x32;
	const GEN_FLT x42 = x0 * x3 * x29;
	const GEN_FLT x43 = -x41 + x42;
	const GEN_FLT x44 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x45 = x44 * x30;
	const GEN_FLT x46 = -x45;
	const GEN_FLT x47 = x3 * x44;
	const GEN_FLT x48 = x0 * x47;
	const GEN_FLT x49 = x47 * x29;
	const GEN_FLT x50 = x48 + x49;
	const GEN_FLT x51 = x23 * x11;
	const GEN_FLT x52 = x19 * x17;
	const GEN_FLT x53 = x14 * x11;
	const GEN_FLT x54 = x24 * x19;
	const GEN_FLT x55 = pow(x19, 2);
	const GEN_FLT x56 = obj_px + (x15 + x55 * x16) * sensor_x + (x51 + x52) * sensor_z + (-x53 + x54) * sensor_y;
	const GEN_FLT x57 = x11 * x19;
	const GEN_FLT x58 = x23 * x17;
	const GEN_FLT x59 = pow(x23, 2);
	const GEN_FLT x60 = obj_py + (x15 + x59 * x16) * sensor_y + (x53 + x54) * sensor_x + (-x57 + x58) * sensor_z;
	const GEN_FLT x61 = x44 * x33;
	const GEN_FLT x62 = x48 + x61;
	const GEN_FLT x63 = pow(x14, 2);
	const GEN_FLT x64 = obj_pz + (x15 + x63 * x16) * sensor_z + (-x51 + x52) * sensor_x + (x57 + x58) * sensor_y;
	const GEN_FLT x65 = 2 * x64 * x48 + (x46 + x50) * x56 + (x45 + x62) * x60;
	const GEN_FLT x66 = x28 + x38 + x65 + x40 * x43;
	const GEN_FLT x67 = -x31 + x34;
	const GEN_FLT x68 = x2 + x3 * pow(x32, 2);
	const GEN_FLT x69 = x0 * x30;
	const GEN_FLT x70 = x33 * x29;
	const GEN_FLT x71 = x69 + x70;
	const GEN_FLT x72 = lh_py + x60 * x68 + x64 * x67 + x71 * x56;
	const GEN_FLT x73 = lh_pz + x4 * x64 + x56 * x43 + x60 * x35;
	const GEN_FLT x74 = pow(x73, 2);
	const GEN_FLT x75 = pow(x74, -1);
	const GEN_FLT x76 = x72 * x75;
	const GEN_FLT x77 = x68 * x37;
	const GEN_FLT x78 = x67 * x27;
	const GEN_FLT x79 = x49 + x61;
	const GEN_FLT x80 = 2 * x60 * x61 + (x46 + x62) * x64 + (x45 + x79) * x56;
	const GEN_FLT x81 = x77 + x78 + x80 + x71 * x40;
	const GEN_FLT x82 = pow(x73, -1);
	const GEN_FLT x83 = x74 + pow(x72, 2);
	const GEN_FLT x84 = pow(x83, -1);
	const GEN_FLT x85 = x84 * x74;
	const GEN_FLT x86 = x41 + x42;
	const GEN_FLT x87 = -x69 + x70;
	const GEN_FLT x88 = x2 + x3 * pow(x29, 2);
	const GEN_FLT x89 = lh_px + x86 * x64 + x87 * x60 + x88 * x56;
	const GEN_FLT x90 = pow(x89, 2);
	const GEN_FLT x91 = pow(1 - x84 * x90 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x92 = x87 * x37;
	const GEN_FLT x93 = 2 * x56 * x49 + (x45 + x50) * x64 + (x46 + x79) * x60;
	const GEN_FLT x94 = x93 + x86 * x27;
	const GEN_FLT x95 = x92 + x94 + x88 * x40;
	const GEN_FLT x96 = tilt_1 / sqrt(x83);
	const GEN_FLT x97 = 2 * x72;
	const GEN_FLT x98 = 2 * x73;
	const GEN_FLT x99 = (1.0 / 2.0) * x89 * tilt_1 / pow(x83, 3.0 / 2.0);
	const GEN_FLT x100 = -x91 * (x96 * x95 - (x66 * x98 + x81 * x97) * x99) - (-x76 * x66 + x81 * x82) * x85;
	const GEN_FLT x101 = -x73;
	const GEN_FLT x102 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x89 * x96) - atan2(-x72, x101)) * gibMag_1;
	const GEN_FLT x103 = x89 * x75;
	const GEN_FLT x104 = 2 * x74 * atan2(x89, x101) * curve_1 / (x74 + x90);
	const GEN_FLT x105 = 1 + x37;
	const GEN_FLT x106 = x65 + x43 * x39;
	const GEN_FLT x107 = x106 + x28 + x35 * x105;
	const GEN_FLT x108 = x80 + x71 * x39;
	const GEN_FLT x109 = x108 + x78 + x68 * x105;
	const GEN_FLT x110 = x88 * x39;
	const GEN_FLT x111 = x110 + x94 + x87 * x105;
	const GEN_FLT x112 = -x91 * (x96 * x111 - (x97 * x109 + x98 * x107) * x99) - (-x76 * x107 + x82 * x109) * x85;
	const GEN_FLT x113 = 1 + x27;
	const GEN_FLT x114 = x106 + x38 + x4 * x113;
	const GEN_FLT x115 = x108 + x77 + x67 * x113;
	const GEN_FLT x116 = x110 + x92 + x93 + x86 * x113;
	const GEN_FLT x117 = -x91 * (x96 * x116 - (x97 * x115 + x98 * x114) * x99) - (-x76 * x114 + x82 * x115) * x85;
	const GEN_FLT x118 = pow(x10, -1);
	const GEN_FLT x119 = x118 * obj_qi;
	const GEN_FLT x120 = x11 * x119;
	const GEN_FLT x121 = -x120;
	const GEN_FLT x122 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qi, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x123 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qk * obj_qi / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x124 = x11 * x123;
	const GEN_FLT x125 = -x124;
	const GEN_FLT x126 = x15 * x119;
	const GEN_FLT x127 = x14 * x126;
	const GEN_FLT x128 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qj * obj_qi / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x129 = x20 * x128;
	const GEN_FLT x130 = x57 * x23;
	const GEN_FLT x131 = x129 + x119 * x130 + x24 * x122;
	const GEN_FLT x132 = x11 * x128;
	const GEN_FLT x133 = x23 * x126;
	const GEN_FLT x134 = x57 * x14;
	const GEN_FLT x135 = x20 * x123;
	const GEN_FLT x136 = x135 + x119 * x134 + x17 * x122;
	const GEN_FLT x137 = (x121 + 2 * x20 * x122 + x55 * x120) * sensor_x + (x125 - x127 + x131) * sensor_y +
						 (x132 + x133 + x136) * sensor_z;
	const GEN_FLT x138 = x24 * x128;
	const GEN_FLT x139 = x19 * x126;
	const GEN_FLT x140 = x11 * x122;
	const GEN_FLT x141 = x17 * x128;
	const GEN_FLT x142 = x51 * x14;
	const GEN_FLT x143 = x24 * x123;
	const GEN_FLT x144 = x141 + x143 + x119 * x142;
	const GEN_FLT x145 =
		(x121 + 2 * x138 + x59 * x120) * sensor_y + (x124 + x127 + x131) * sensor_x + (-x139 - x140 + x144) * sensor_z;
	const GEN_FLT x146 = -x132;
	const GEN_FLT x147 = x17 * x123;
	const GEN_FLT x148 =
		(x121 + 2 * x147 + x63 * x120) * sensor_z + (-x133 + x136 + x146) * sensor_x + (x139 + x140 + x144) * sensor_y;
	const GEN_FLT x149 = x65 + x35 * x145 + x4 * x148 + x43 * x137;
	const GEN_FLT x150 = x75 * x149;
	const GEN_FLT x151 = x80 + x67 * x148 + x68 * x145 + x71 * x137;
	const GEN_FLT x152 = x93 + x86 * x148 + x87 * x145 + x88 * x137;
	const GEN_FLT x153 = -x91 * (x96 * x152 - (x97 * x151 + x98 * x149) * x99) - (-x72 * x150 + x82 * x151) * x85;
	const GEN_FLT x154 = x118 * obj_qj;
	const GEN_FLT x155 = x15 * x14;
	const GEN_FLT x156 = x154 * x155;
	const GEN_FLT x157 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-obj_qk * obj_qj / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x158 = x11 * x157;
	const GEN_FLT x159 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qj, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x160 = x138 + x130 * x154 + x20 * x159;
	const GEN_FLT x161 = x11 * x154;
	const GEN_FLT x162 = -x161;
	const GEN_FLT x163 = x15 * x154;
	const GEN_FLT x164 = x19 * x163;
	const GEN_FLT x165 = x24 * x157;
	const GEN_FLT x166 = x165 + x142 * x154 + x17 * x159;
	const GEN_FLT x167 = (x156 + x158 + x160) * sensor_x + (x162 + 2 * x24 * x159 + x59 * x161) * sensor_y +
						 (x146 - x164 + x166) * sensor_z;
	const GEN_FLT x168 = -x158;
	const GEN_FLT x169 = x23 * x163;
	const GEN_FLT x170 = x11 * x159;
	const GEN_FLT x171 = x20 * x157;
	const GEN_FLT x172 = x141 + x171 + x134 * x154;
	const GEN_FLT x173 =
		(2 * x129 + x162 + x55 * x161) * sensor_x + (-x156 + x160 + x168) * sensor_y + (x169 + x170 + x172) * sensor_z;
	const GEN_FLT x174 = x17 * x157;
	const GEN_FLT x175 =
		(x162 + 2 * x174 + x63 * x161) * sensor_z + (x132 + x164 + x166) * sensor_y + (-x169 - x170 + x172) * sensor_x;
	const GEN_FLT x176 = x65 + x35 * x167 + x4 * x175 + x43 * x173;
	const GEN_FLT x177 = x80 + x67 * x175 + x68 * x167 + x71 * x173;
	const GEN_FLT x178 = x93 + x86 * x175 + x87 * x167 + x88 * x173;
	const GEN_FLT x179 = -x91 * (x96 * x178 - (x97 * x177 + x98 * x176) * x99) - (-x76 * x176 + x82 * x177) * x85;
	const GEN_FLT x180 = x118 * obj_qk;
	const GEN_FLT x181 = x180 * x155;
	const GEN_FLT x182 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  ? (-pow(obj_qk, 2) / pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), 3.0 / 2.0) +
								 pow(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x183 = x11 * x182;
	const GEN_FLT x184 = x143 + x171 + x180 * x130;
	const GEN_FLT x185 = x11 * x180;
	const GEN_FLT x186 = -x185;
	const GEN_FLT x187 = x15 * x180;
	const GEN_FLT x188 = x19 * x187;
	const GEN_FLT x189 = x174 + x180 * x142 + x24 * x182;
	const GEN_FLT x190 =
		(x125 - x188 + x189) * sensor_z + (2 * x165 + x186 + x59 * x185) * sensor_y + (x181 + x183 + x184) * sensor_x;
	const GEN_FLT x191 = x23 * x187;
	const GEN_FLT x192 = x147 + x180 * x134 + x20 * x182;
	const GEN_FLT x193 =
		(-x181 - x183 + x184) * sensor_y + (2 * x135 + x186 + x55 * x185) * sensor_x + (x158 + x191 + x192) * sensor_z;
	const GEN_FLT x194 = (x124 + x188 + x189) * sensor_y + (x168 - x191 + x192) * sensor_x +
						 (x186 + 2 * x17 * x182 + x63 * x185) * sensor_z;
	const GEN_FLT x195 = x65 + x35 * x190 + x4 * x194 + x43 * x193;
	const GEN_FLT x196 = x80 + x67 * x194 + x68 * x190 + x71 * x193;
	const GEN_FLT x197 = x93 + x86 * x194 + x87 * x190 + x88 * x193;
	const GEN_FLT x198 = -x91 * (x96 * x197 - (x97 * x196 + x98 * x195) * x99) - (-x76 * x195 + x82 * x196) * x85;
	*(out++) = x100 + x100 * x102 + x104 * (x66 * x103 - x82 * x95);
	*(out++) = x112 + x102 * x112 + x104 * (x103 * x107 - x82 * x111);
	*(out++) = x117 + x102 * x117 + x104 * (x103 * x114 - x82 * x116);
	*(out++) = x153 + x102 * x153 + (-x82 * x152 + x89 * x150) * x104;
	*(out++) = x179 + x102 * x179 + x104 * (x103 * x176 - x82 * x178);
	*(out++) = x198 + x102 * x198 + x104 * (x103 * x195 - x82 * x197);
}

// Jacobian of reproject_axis_y wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_reproject_axis_y_jac_sensor_pt_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
																 const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
																 const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x5 = cos(x1);
	const GEN_FLT x6 = 1 - x5;
	const GEN_FLT x7 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (1));
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = -x3 + x9;
	const GEN_FLT x11 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x12 = cos(x11);
	const GEN_FLT x13 = 1 - x12;
	const GEN_FLT x14 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x15 = x12 + pow(x14, 2) * x13;
	const GEN_FLT x16 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x17 = x13 * x16;
	const GEN_FLT x18 = x14 * x17;
	const GEN_FLT x19 = sin(x11);
	const GEN_FLT x20 = x19 * x16;
	const GEN_FLT x21 = -x20;
	const GEN_FLT x22 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x23 = x22 * x17;
	const GEN_FLT x24 = x18 + x23;
	const GEN_FLT x25 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x26 = x25 * x17;
	const GEN_FLT x27 = x18 + x26;
	const GEN_FLT x28 = 2 * x18 * sensor_x + (x20 + x27) * sensor_z + (x21 + x24) * sensor_y;
	const GEN_FLT x29 = x15 + x28;
	const GEN_FLT x30 = x25 * x19;
	const GEN_FLT x31 = x22 * x13;
	const GEN_FLT x32 = x31 * x14;
	const GEN_FLT x33 = x30 + x32;
	const GEN_FLT x34 = x23 + x26;
	const GEN_FLT x35 = 2 * x23 * sensor_y + (x20 + x24) * sensor_x + (x21 + x34) * sensor_z;
	const GEN_FLT x36 = x33 + x35;
	const GEN_FLT x37 = x2 * x7;
	const GEN_FLT x38 = x0 * x4 * x6;
	const GEN_FLT x39 = x37 + x38;
	const GEN_FLT x40 = x22 * x19;
	const GEN_FLT x41 = x25 * x14 * x13;
	const GEN_FLT x42 = -x40 + x41;
	const GEN_FLT x43 = 2 * x26 * sensor_z + (x21 + x27) * sensor_x + (x20 + x34) * sensor_y;
	const GEN_FLT x44 = x42 + x43;
	const GEN_FLT x45 = x5 + pow(x4, 2) * x6;
	const GEN_FLT x46 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x47 = x2 * x46;
	const GEN_FLT x48 = -x47;
	const GEN_FLT x49 = x6 * x46;
	const GEN_FLT x50 = x7 * x49;
	const GEN_FLT x51 = x4 * x49;
	const GEN_FLT x52 = x50 + x51;
	const GEN_FLT x53 = x40 + x41;
	const GEN_FLT x54 = -x30 + x32;
	const GEN_FLT x55 = obj_px + x15 * sensor_x + x53 * sensor_z + x54 * sensor_y;
	const GEN_FLT x56 = x14 * x19;
	const GEN_FLT x57 = x31 * x25;
	const GEN_FLT x58 = -x56 + x57;
	const GEN_FLT x59 = x12 + pow(x22, 2) * x13;
	const GEN_FLT x60 = obj_py + x33 * sensor_x + x58 * sensor_z + x59 * sensor_y;
	const GEN_FLT x61 = x0 * x49;
	const GEN_FLT x62 = x51 + x61;
	const GEN_FLT x63 = x56 + x57;
	const GEN_FLT x64 = x12 + pow(x25, 2) * x13;
	const GEN_FLT x65 = obj_pz + x42 * sensor_x + x63 * sensor_y + x64 * sensor_z;
	const GEN_FLT x66 = 2 * x65 * x51 + (x48 + x52) * x55 + (x47 + x62) * x60;
	const GEN_FLT x67 = x66 + x29 * x10 + x36 * x39 + x44 * x45;
	const GEN_FLT x68 = -x37 + x38;
	const GEN_FLT x69 = x5 + pow(x0, 2) * x6;
	const GEN_FLT x70 = x2 * x4;
	const GEN_FLT x71 = x0 * x8;
	const GEN_FLT x72 = x70 + x71;
	const GEN_FLT x73 = lh_py + x60 * x69 + x68 * x65 + x72 * x55;
	const GEN_FLT x74 = lh_pz + x55 * x10 + x60 * x39 + x65 * x45;
	const GEN_FLT x75 = pow(x74, 2);
	const GEN_FLT x76 = pow(x75, -1);
	const GEN_FLT x77 = x73 * x76;
	const GEN_FLT x78 = pow(x74, -1);
	const GEN_FLT x79 = x50 + x61;
	const GEN_FLT x80 = 2 * x60 * x61 + (x48 + x62) * x65 + (x47 + x79) * x55;
	const GEN_FLT x81 = x80 + x68 * x44 + x69 * x36 + x72 * x29;
	const GEN_FLT x82 = x75 + pow(x73, 2);
	const GEN_FLT x83 = pow(x82, -1);
	const GEN_FLT x84 = x83 * x75;
	const GEN_FLT x85 = x3 + x9;
	const GEN_FLT x86 = -x70 + x71;
	const GEN_FLT x87 = x5 + x6 * pow(x7, 2);
	const GEN_FLT x88 = lh_px + x85 * x65 + x86 * x60 + x87 * x55;
	const GEN_FLT x89 = pow(x88, 2);
	const GEN_FLT x90 = pow(1 - x83 * x89 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x91 = 2 * x50 * x55 + (x47 + x52) * x65 + (x48 + x79) * x60;
	const GEN_FLT x92 = x91 + x85 * x44 + x86 * x36 + x87 * x29;
	const GEN_FLT x93 = tilt_1 / sqrt(x82);
	const GEN_FLT x94 = 2 * x73;
	const GEN_FLT x95 = 2 * x74;
	const GEN_FLT x96 = (1.0 / 2.0) * x88 * tilt_1 / pow(x82, 3.0 / 2.0);
	const GEN_FLT x97 = -x90 * (x93 * x92 - (x67 * x95 + x81 * x94) * x96) - (-x77 * x67 + x81 * x78) * x84;
	const GEN_FLT x98 = -x74;
	const GEN_FLT x99 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x88 * x93) - atan2(-x73, x98)) * gibMag_1;
	const GEN_FLT x100 = x88 * x76;
	const GEN_FLT x101 = 2 * x75 * atan2(x88, x98) * curve_1 / (x75 + x89);
	const GEN_FLT x102 = x28 + x54;
	const GEN_FLT x103 = x35 + x59;
	const GEN_FLT x104 = x43 + x63;
	const GEN_FLT x105 = x66 + x10 * x102 + x39 * x103 + x45 * x104;
	const GEN_FLT x106 = x80 + x68 * x104 + x69 * x103 + x72 * x102;
	const GEN_FLT x107 = x91 + x85 * x104 + x86 * x103 + x87 * x102;
	const GEN_FLT x108 = -x90 * (x93 * x107 - (x94 * x106 + x95 * x105) * x96) - (-x77 * x105 + x78 * x106) * x84;
	const GEN_FLT x109 = x35 + x58;
	const GEN_FLT x110 = x28 + x53;
	const GEN_FLT x111 = x43 + x64;
	const GEN_FLT x112 = x66 + x10 * x110 + x39 * x109 + x45 * x111;
	const GEN_FLT x113 = x80 + x68 * x111 + x69 * x109 + x72 * x110;
	const GEN_FLT x114 = x91 + x85 * x111 + x86 * x109 + x87 * x110;
	const GEN_FLT x115 = -x90 * (x93 * x114 - (x94 * x113 + x95 * x112) * x96) - (-x77 * x112 + x78 * x113) * x84;
	*(out++) = x97 + x101 * (x67 * x100 - x78 * x92) + x99 * x97;
	*(out++) = x108 + x101 * (x100 * x105 - x78 * x107) + x99 * x108;
	*(out++) = x115 + x101 * (x100 * x112 - x78 * x114) + x99 * x115;
}

// Jacobian of reproject_axis_y wrt [lh_px, lh_py, lh_pz, lh_qi, lh_qj, lh_qk]
static inline void gen_reproject_axis_y_jac_lh_p_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
															const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
															const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x1 = pow(lh_qk, 2);
	const GEN_FLT x2 = pow(lh_qj, 2);
	const GEN_FLT x3 = pow(lh_qi, 2);
	const GEN_FLT x4 = x1 + x2 + x3;
	const GEN_FLT x5 = sqrt(x4);
	const GEN_FLT x6 = cos(x5);
	const GEN_FLT x7 = 1 - x6;
	const GEN_FLT x8 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x9 = x8 * x7;
	const GEN_FLT x10 = x0 * x9;
	const GEN_FLT x11 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (1));
	const GEN_FLT x12 = x7 * x11;
	const GEN_FLT x13 = x0 * x12;
	const GEN_FLT x14 = sin(x5);
	const GEN_FLT x15 = x0 * x14;
	const GEN_FLT x16 = -x15;
	const GEN_FLT x17 = x13 + x16;
	const GEN_FLT x18 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x19 = sin(x18);
	const GEN_FLT x20 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x23 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x24 = cos(x18);
	const GEN_FLT x25 = 1 - x24;
	const GEN_FLT x26 = x25 * x23;
	const GEN_FLT x27 = x22 * x26;
	const GEN_FLT x28 = x22 * x19;
	const GEN_FLT x29 = x20 * x26;
	const GEN_FLT x30 =
		obj_px + (x21 + x27) * sensor_z + (x24 + x25 * pow(x23, 2)) * sensor_x + (-x28 + x29) * sensor_y;
	const GEN_FLT x31 = x23 * x19;
	const GEN_FLT x32 = x25 * x22 * x20;
	const GEN_FLT x33 =
		obj_py + (x24 + x25 * pow(x20, 2)) * sensor_y + (x28 + x29) * sensor_x + (-x31 + x32) * sensor_z;
	const GEN_FLT x34 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							 : (0));
	const GEN_FLT x35 = x7 * x34;
	const GEN_FLT x36 = x0 * x35;
	const GEN_FLT x37 = x15 + x36;
	const GEN_FLT x38 =
		obj_pz + (-x21 + x27) * sensor_x + (x24 + x25 * pow(x22, 2)) * sensor_z + (x31 + x32) * sensor_y;
	const GEN_FLT x39 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x40 = x39 * x26;
	const GEN_FLT x41 = x39 * x25;
	const GEN_FLT x42 = x41 * x20;
	const GEN_FLT x43 = x39 * x19;
	const GEN_FLT x44 = -x43;
	const GEN_FLT x45 = x42 + x44;
	const GEN_FLT x46 = x41 * x22;
	const GEN_FLT x47 = x40 + x43;
	const GEN_FLT x48 = 2 * x40 * sensor_x + (x40 + x45) * sensor_y + (x46 + x47) * sensor_z;
	const GEN_FLT x49 = x34 * x14;
	const GEN_FLT x50 = x8 * x12;
	const GEN_FLT x51 = -x49 + x50;
	const GEN_FLT x52 = x14 * x11;
	const GEN_FLT x53 = x8 * x35;
	const GEN_FLT x54 = x52 + x53;
	const GEN_FLT x55 = 2 * x42 * sensor_y + (x42 + x47) * sensor_x + (x45 + x46) * sensor_z;
	const GEN_FLT x56 = pow(x8, 2);
	const GEN_FLT x57 = x6 + x7 * x56;
	const GEN_FLT x58 = 2 * x46 * sensor_z + (x40 + x44 + x46) * sensor_x + (x42 + x43 + x46) * sensor_y;
	const GEN_FLT x59 = x51 * x48 + x54 * x55 + x58 * x57;
	const GEN_FLT x60 = x59 + 2 * x38 * x10 + (x10 + x17) * x30 + (x10 + x37) * x33;
	const GEN_FLT x61 = lh_pz + x51 * x30 + x54 * x33 + x57 * x38;
	const GEN_FLT x62 = pow(x61, 2);
	const GEN_FLT x63 = pow(x62, -1);
	const GEN_FLT x64 = -x52 + x53;
	const GEN_FLT x65 = pow(x34, 2);
	const GEN_FLT x66 = x6 + x7 * x65;
	const GEN_FLT x67 = x8 * x14;
	const GEN_FLT x68 = x34 * x12;
	const GEN_FLT x69 = x67 + x68;
	const GEN_FLT x70 = lh_py + x64 * x38 + x66 * x33 + x69 * x30;
	const GEN_FLT x71 = x70 * x63;
	const GEN_FLT x72 = -x71 * x60;
	const GEN_FLT x73 = x64 * x58 + x66 * x55 + x69 * x48;
	const GEN_FLT x74 = x73 + 2 * x33 * x36 + x38 * (x10 + x16 + x36) + (x13 + x37) * x30;
	const GEN_FLT x75 = pow(x61, -1);
	const GEN_FLT x76 = x75 * x74;
	const GEN_FLT x77 = x62 + pow(x70, 2);
	const GEN_FLT x78 = pow(x77, -1);
	const GEN_FLT x79 = x78 * x62;
	const GEN_FLT x80 = x49 + x50;
	const GEN_FLT x81 = -x67 + x68;
	const GEN_FLT x82 = pow(x11, 2);
	const GEN_FLT x83 = x6 + x7 * x82;
	const GEN_FLT x84 = lh_px + x80 * x38 + x81 * x33 + x83 * x30;
	const GEN_FLT x85 = pow(x84, 2);
	const GEN_FLT x86 = pow(1 - x85 * x78 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x87 = x80 * x58 + x81 * x55 + x83 * x48;
	const GEN_FLT x88 = x87 + 2 * x30 * x13 + x38 * (x10 + x13 + x15) + (x17 + x36) * x33;
	const GEN_FLT x89 = 1 + x88;
	const GEN_FLT x90 = tilt_1 / sqrt(x77);
	const GEN_FLT x91 = 2 * x70;
	const GEN_FLT x92 = x74 * x91;
	const GEN_FLT x93 = 2 * x61;
	const GEN_FLT x94 = x60 * x93;
	const GEN_FLT x95 = (1.0 / 2.0) * x84 * tilt_1 / pow(x77, 3.0 / 2.0);
	const GEN_FLT x96 = -x86 * (x89 * x90 - (x92 + x94) * x95) - (x72 + x76) * x79;
	const GEN_FLT x97 = -x61;
	const GEN_FLT x98 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x84 * x90) - atan2(-x70, x97)) * gibMag_1;
	const GEN_FLT x99 = x84 * x63;
	const GEN_FLT x100 = x60 * x99;
	const GEN_FLT x101 = 2 * x62 * atan2(x84, x97) * curve_1 / (x62 + x85);
	const GEN_FLT x102 = 1 + x74;
	const GEN_FLT x103 = x88 * x90;
	const GEN_FLT x104 = -x79 * (x72 + x75 * x102) - x86 * (x103 - x95 * (x94 + x91 * x102));
	const GEN_FLT x105 = -x88 * x75;
	const GEN_FLT x106 = 1 + x60;
	const GEN_FLT x107 = -x79 * (x76 - x71 * x106) - x86 * (x103 - x95 * (x92 + x93 * x106));
	const GEN_FLT x108 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qj * lh_qi / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x109 = x14 * x108;
	const GEN_FLT x110 = -x109;
	const GEN_FLT x111 = pow(x5, -1);
	const GEN_FLT x112 = x111 * lh_qi;
	const GEN_FLT x113 = x6 * x34;
	const GEN_FLT x114 = x112 * x113;
	const GEN_FLT x115 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qi, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x116 = x8 * x52;
	const GEN_FLT x117 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qk * lh_qi / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x118 = x7 * x117;
	const GEN_FLT x119 = x11 * x118;
	const GEN_FLT x120 = x119 + x112 * x116 + x9 * x115;
	const GEN_FLT x121 = x14 * x115;
	const GEN_FLT x122 = x6 * x11;
	const GEN_FLT x123 = x112 * x122;
	const GEN_FLT x124 = x34 * x118;
	const GEN_FLT x125 = x34 * x112;
	const GEN_FLT x126 = x9 * x108;
	const GEN_FLT x127 = x124 + x126 + x67 * x125;
	const GEN_FLT x128 = x14 * x112;
	const GEN_FLT x129 = -x128;
	const GEN_FLT x130 = x8 * x118;
	const GEN_FLT x131 =
		x59 + x30 * (x110 - x114 + x120) + x33 * (x121 + x123 + x127) + x38 * (x129 + 2 * x130 + x56 * x128);
	const GEN_FLT x132 = x14 * x117;
	const GEN_FLT x133 = x6 * x8;
	const GEN_FLT x134 = x112 * x133;
	const GEN_FLT x135 = x12 * x108;
	const GEN_FLT x136 = x135 + x35 * x115 + x52 * x125;
	const GEN_FLT x137 = x35 * x108;
	const GEN_FLT x138 =
		x73 + x30 * (x132 + x134 + x136) + x33 * (x129 + 2 * x137 + x65 * x128) + x38 * (-x121 - x123 + x127);
	const GEN_FLT x139 = -x132;
	const GEN_FLT x140 =
		x87 + x30 * (x129 + 2 * x12 * x115 + x82 * x128) + x33 * (-x134 + x136 + x139) + x38 * (x109 + x114 + x120);
	const GEN_FLT x141 = -x86 * (x90 * x140 - (x91 * x138 + x93 * x131) * x95) - (-x71 * x131 + x75 * x138) * x79;
	const GEN_FLT x142 = x111 * lh_qj;
	const GEN_FLT x143 = x122 * x142;
	const GEN_FLT x144 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-lh_qk * lh_qj / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0))
							  : (0));
	const GEN_FLT x145 = x35 * x144;
	const GEN_FLT x146 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qj, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x147 = x7 * x146;
	const GEN_FLT x148 = x67 * x34;
	const GEN_FLT x149 = x145 + x142 * x148 + x8 * x147;
	const GEN_FLT x150 = x14 * x146;
	const GEN_FLT x151 = x113 * x142;
	const GEN_FLT x152 = x12 * x144;
	const GEN_FLT x153 = x126 + x152 + x116 * x142;
	const GEN_FLT x154 = x14 * x142;
	const GEN_FLT x155 = -x154;
	const GEN_FLT x156 = x9 * x144;
	const GEN_FLT x157 =
		x59 + x30 * (-x150 - x151 + x153) + x33 * (x109 + x143 + x149) + x38 * (x155 + 2 * x156 + x56 * x154);
	const GEN_FLT x158 = x14 * x144;
	const GEN_FLT x159 = x133 * x142;
	const GEN_FLT x160 = x52 * x34;
	const GEN_FLT x161 = x137 + x11 * x147 + x160 * x142;
	const GEN_FLT x162 =
		x73 + x30 * (x158 + x159 + x161) + x33 * (x155 + 2 * x34 * x147 + x65 * x154) + x38 * (x110 - x143 + x149);
	const GEN_FLT x163 = -x158;
	const GEN_FLT x164 =
		x87 + x30 * (2 * x135 + x155 + x82 * x154) + x33 * (-x159 + x161 + x163) + x38 * (x150 + x151 + x153);
	const GEN_FLT x165 = -x86 * (x90 * x164 - (x91 * x162 + x93 * x157) * x95) - (-x71 * x157 + x75 * x162) * x79;
	const GEN_FLT x166 = x111 * lh_qk;
	const GEN_FLT x167 = x113 * x166;
	const GEN_FLT x168 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  ? (-pow(lh_qk, 2) / pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), 3.0 / 2.0) +
								 pow(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2), -1.0 / 2.0))
							  : (0));
	const GEN_FLT x169 = x130 + x116 * x166 + x12 * x168;
	const GEN_FLT x170 = x122 * x166;
	const GEN_FLT x171 = x156 + x166 * x148 + x35 * x168;
	const GEN_FLT x172 = x14 * x166;
	const GEN_FLT x173 = -x172;
	const GEN_FLT x174 =
		x59 + x30 * (x163 - x167 + x169) + x33 * (x132 + x170 + x171) + x38 * (x173 + x56 * x172 + 2 * x9 * x168);
	const GEN_FLT x175 = x14 * x168;
	const GEN_FLT x176 = x166 * x133;
	const GEN_FLT x177 = x124 + x152 + x160 * x166;
	const GEN_FLT x178 =
		x73 + x30 * (x175 + x176 + x177) + x33 * (2 * x145 + x173 + x65 * x172) + x38 * (x139 - x170 + x171);
	const GEN_FLT x179 =
		x87 + x30 * (2 * x119 + x173 + x82 * x172) + x33 * (-x175 - x176 + x177) + x38 * (x158 + x167 + x169);
	const GEN_FLT x180 = -x86 * (x90 * x179 - (x91 * x178 + x93 * x174) * x95) - (-x71 * x174 + x75 * x178) * x79;
	*(out++) = x96 + x101 * (x100 - x89 * x75) + x98 * x96;
	*(out++) = x104 + x98 * x104 + (x100 + x105) * x101;
	*(out++) = x107 + x101 * (x105 + x99 * x106) + x98 * x107;
	*(out++) = x141 + x98 * x141 + (-x75 * x140 + x99 * x131) * x101;
	*(out++) = x165 + x98 * x165 + (-x75 * x164 + x99 * x157) * x101;
	*(out++) = x180 + x98 * x180 + (-x75 * x179 + x99 * x174) * x101;
}

// Jacobian of reproject_axis_y wrt [phase_1, tilt_1, curve_1, gibPhase_1, gibMag_1, ogeeMag_1, ogeePhase_1]
static inline void gen_reproject_axis_y_jac_bsc1_axis_angle(FLT *out, const LinmathAxisAnglePose *obj_p,
															const FLT *sensor_pt, const LinmathAxisAnglePose *lh_p,
															const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qi = (*obj_p).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qi = (*lh_p).AxisAngleRot[0];
	const GEN_FLT lh_qj = (*lh_p).AxisAngleRot[1];
	const GEN_FLT lh_qk = (*lh_p).AxisAngleRot[2];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x1 = sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (0));
	const GEN_FLT x5 = cos(x1);
	const GEN_FLT x6 = 1 - x5;
	const GEN_FLT x7 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							: (1));
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2));
	const GEN_FLT x12 = sin(x11);
	const GEN_FLT x13 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1));
	const GEN_FLT x14 = x13 * x12;
	const GEN_FLT x15 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x16 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (0));
	const GEN_FLT x17 = cos(x11);
	const GEN_FLT x18 = 1 - x17;
	const GEN_FLT x19 = x18 * x16;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x15 * x12;
	const GEN_FLT x22 = x13 * x19;
	const GEN_FLT x23 =
		obj_pz + (x17 + x18 * pow(x16, 2)) * sensor_z + (x14 + x20) * sensor_y + (-x21 + x22) * sensor_x;
	const GEN_FLT x24 = x2 * x4;
	const GEN_FLT x25 = x0 * x8;
	const GEN_FLT x26 = -x24 + x25;
	const GEN_FLT x27 = x12 * x16;
	const GEN_FLT x28 = x15 * x13 * x18;
	const GEN_FLT x29 =
		obj_py + (x17 + pow(x15, 2) * x18) * sensor_y + (-x14 + x20) * sensor_z + (x27 + x28) * sensor_x;
	const GEN_FLT x30 =
		obj_px + (x17 + pow(x13, 2) * x18) * sensor_x + (x21 + x22) * sensor_z + (-x27 + x28) * sensor_y;
	const GEN_FLT x31 = x5 + x6 * pow(x7, 2);
	const GEN_FLT x32 = lh_px + x23 * x10 + x29 * x26 + x30 * x31;
	const GEN_FLT x33 = pow(x32, 2);
	const GEN_FLT x34 = x5 + pow(x4, 2) * x6;
	const GEN_FLT x35 = x2 * x7;
	const GEN_FLT x36 = x0 * x4 * x6;
	const GEN_FLT x37 = x35 + x36;
	const GEN_FLT x38 = -x3 + x9;
	const GEN_FLT x39 = lh_pz + x30 * x38 + x34 * x23 + x37 * x29;
	const GEN_FLT x40 = pow(x39, 2);
	const GEN_FLT x41 = ((0 < sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))) ? (0) : (0));
	const GEN_FLT x42 = x6 * x41;
	const GEN_FLT x43 = x4 * x42;
	const GEN_FLT x44 = x2 * x41;
	const GEN_FLT x45 = -x44;
	const GEN_FLT x46 = x7 * x42;
	const GEN_FLT x47 = x45 + x46;
	const GEN_FLT x48 = ((0 < sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))) ? (0) : (0));
	const GEN_FLT x49 = x48 * x12;
	const GEN_FLT x50 = x48 * x18;
	const GEN_FLT x51 = x50 * x15;
	const GEN_FLT x52 = x50 * x13;
	const GEN_FLT x53 = x51 + x52;
	const GEN_FLT x54 = -x49;
	const GEN_FLT x55 = x48 * x19;
	const GEN_FLT x56 = x51 + x55;
	const GEN_FLT x57 = 2 * x51 * sensor_y + (x49 + x53) * sensor_x + (x54 + x56) * sensor_z;
	const GEN_FLT x58 = x0 * x42;
	const GEN_FLT x59 = x43 + x58;
	const GEN_FLT x60 = x52 + x55;
	const GEN_FLT x61 = 2 * x52 * sensor_x + (x53 + x54) * sensor_y + (x49 + x60) * sensor_z;
	const GEN_FLT x62 = 2 * x55 * sensor_z + (x49 + x56) * sensor_y + (x54 + x60) * sensor_x;
	const GEN_FLT x63 = 2 * x43 * x23 + x57 * x37 + x61 * x38 + x62 * x34 + (x43 + x47) * x30 + (x44 + x59) * x29;
	const GEN_FLT x64 = x63 / x40;
	const GEN_FLT x65 = x44 + x46;
	const GEN_FLT x66 = 2 * x46 * x30 + x57 * x26 + x61 * x31 + x62 * x10 + (x47 + x58) * x29 + (x43 + x65) * x23;
	const GEN_FLT x67 = pow(x39, -1);
	const GEN_FLT x68 = -x39;
	const GEN_FLT x69 = atan2(x32, x68);
	const GEN_FLT x70 = 2 * (x64 * x32 - x67 * x66) * x69 * x40 * curve_1 / (x33 + x40);
	const GEN_FLT x71 = -x35 + x36;
	const GEN_FLT x72 = x5 + pow(x0, 2) * x6;
	const GEN_FLT x73 = x24 + x25;
	const GEN_FLT x74 = lh_py + x71 * x23 + x72 * x29 + x73 * x30;
	const GEN_FLT x75 = x40 + pow(x74, 2);
	const GEN_FLT x76 = pow(x75, -1);
	const GEN_FLT x77 = 2 * x58 * x29 + x71 * x62 + x72 * x57 + x73 * x61 + (x45 + x59) * x23 + (x58 + x65) * x30;
	const GEN_FLT x78 = -(-x74 * x64 + x77 * x67) * x76 * x40;
	const GEN_FLT x79 = pow(1 - x76 * x33 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x80 = pow(x75, -1.0 / 2.0);
	const GEN_FLT x81 =
		x80 * x66 * tilt_1 + (-1.0 / 2.0) * (2 * x63 * x39 + 2 * x74 * x77) * x32 * tilt_1 / pow(x75, 3.0 / 2.0);
	const GEN_FLT x82 = x78 - x81 * x79;
	const GEN_FLT x83 = -1 + x82;
	const GEN_FLT x84 = x80 * x32;
	const GEN_FLT x85 = 1.5707963267949 + gibPhase_1 - phase_1 - asin(x84 * tilt_1) - atan2(-x74, x68);
	const GEN_FLT x86 = sin(x85) * gibMag_1;
	const GEN_FLT x87 = x78 - (x81 + x84) * x79;
	const GEN_FLT x88 = x70 + x82;
	const GEN_FLT x89 = x88 + x82 * x86;
	*(out++) = x70 + x83 + x83 * x86;
	*(out++) = x70 + x87 + x86 * x87;
	*(out++) = x89 + pow(x69, 2);
	*(out++) = x88 + x86 * (1 + x82);
	*(out++) = x89 - cos(x85);
	*(out++) = x89;
	*(out++) = x89;
}

/** Applying function <function reproject_gen2 at 0x7ffa3c1c7b00> */
static inline void gen_reproject_gen2(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt, const SurvivePose *lh_p,
									  const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = 0.523598775598299 + tilt_0;
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = lh_qw * lh_qi;
	const GEN_FLT x3 = lh_qk * lh_qj;
	const GEN_FLT x4 = pow(obj_qj, 2);
	const GEN_FLT x5 = pow(obj_qi, 2);
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = 2 * sqrt(x6 + x7 + pow(obj_qw, 2));
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = obj_pz + x11 * (x10 + x9) + (1 - x6 * x8) * sensor_z + (-x12 + x13) * x14;
	const GEN_FLT x16 = pow(lh_qi, 2);
	const GEN_FLT x17 = pow(lh_qk, 2);
	const GEN_FLT x18 = pow(lh_qj, 2);
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 2 * sqrt(x16 + x19 + pow(lh_qw, 2));
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + x22 * (x10 - x9) + (1 - (x5 + x7) * x8) * sensor_y + (x23 + x24) * x14;
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 = obj_px + (1 - (x4 + x7) * x8) * sensor_x + (x12 + x13) * x22 + (-x23 + x24) * x11;
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = lh_py + x25 * (1 - (x16 + x17) * x20) + (-x2 + x3) * x21 + (x26 + x27) * x29;
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = lh_pz + x15 * (1 - (x16 + x18) * x20) + (x2 + x3) * x31 + (-x32 + x33) * x29;
	const GEN_FLT x35 = lh_px + x28 * (1 - x20 * x19) + (-x26 + x27) * x31 + (x32 + x33) * x21;
	const GEN_FLT x36 = pow(x34, 2) + pow(x35, 2);
	const GEN_FLT x37 = x30 / sqrt(x36 + pow(x30, 2));
	const GEN_FLT x38 = asin(x37 / x1);
	const GEN_FLT x39 = 0.0028679863 + x38 * (-8.0108022e-06 - 8.0108022e-06 * x38);
	const GEN_FLT x40 = 5.3685255e-06 + x38 * x39;
	const GEN_FLT x41 = 0.0076069798 + x40 * x38;
	const GEN_FLT x42 = x30 / sqrt(x36);
	const GEN_FLT x43 = tan(x0) * x42;
	const GEN_FLT x44 = atan2(-x34, x35);
	const GEN_FLT x45 = curve_0 + sin(ogeeMag_0 + x44 - asin(x43)) * ogeePhase_0;
	const GEN_FLT x46 = asin(
		x43 + x41 * x45 * pow(x38, 2) /
				  (x1 - sin(x0) * x45 *
							(x38 * (x41 + x38 * (x40 + x38 * (x39 + x38 * (-8.0108022e-06 - 1.60216044e-05 * x38)))) +
							 x41 * x38)));
	const GEN_FLT x47 = -x44;
	const GEN_FLT x48 = -1.5707963267949 + x44;
	const GEN_FLT x49 = 0.523598775598299 - tilt_1;
	const GEN_FLT x50 = -x42 * tan(x49);
	const GEN_FLT x51 = curve_1 + sin(ogeeMag_1 + x44 - asin(x50)) * ogeePhase_1;
	const GEN_FLT x52 = cos(x49);
	const GEN_FLT x53 = asin(x37 / x52);
	const GEN_FLT x54 = 0.0028679863 + x53 * (-8.0108022e-06 - 8.0108022e-06 * x53);
	const GEN_FLT x55 = 5.3685255e-06 + x54 * x53;
	const GEN_FLT x56 = 0.0076069798 + x53 * x55;
	const GEN_FLT x57 =
		asin(x50 +
			 pow(x53, 2) * x51 * x56 /
				 (x52 + x51 * sin(x49) *
							(x53 * x56 +
							 x53 * (x56 + x53 * (x55 + x53 * (x54 + x53 * (-8.0108022e-06 - 1.60216044e-05 * x53)))))));
	*(out++) = -phase_0 - x46 + x48 - sin(-gibPhase_0 + x46 + x47) * gibMag_0;
	*(out++) = -phase_1 + x48 - x57 - sin(-gibPhase_1 + x47 + x57) * gibMag_1;
}

// Jacobian of reproject_gen2 wrt [obj_px, obj_py, obj_pz, obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_reproject_gen2_jac_obj_p(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
												const SurvivePose *lh_p, const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = lh_qw * lh_qi;
	const GEN_FLT x1 = lh_qk * lh_qj;
	const GEN_FLT x2 = -x0 + x1;
	const GEN_FLT x3 = pow(obj_qj, 2);
	const GEN_FLT x4 = pow(obj_qi, 2);
	const GEN_FLT x5 = x3 + x4;
	const GEN_FLT x6 = pow(obj_qk, 2);
	const GEN_FLT x7 = x4 + x6;
	const GEN_FLT x8 = sqrt(x3 + x7 + pow(obj_qw, 2));
	const GEN_FLT x9 = 2 * x8;
	const GEN_FLT x10 = obj_qw * obj_qi;
	const GEN_FLT x11 = obj_qk * obj_qj;
	const GEN_FLT x12 = x10 + x11;
	const GEN_FLT x13 = x9 * sensor_y;
	const GEN_FLT x14 = obj_qw * obj_qj;
	const GEN_FLT x15 = obj_qk * obj_qi;
	const GEN_FLT x16 = -x14 + x15;
	const GEN_FLT x17 = x9 * sensor_x;
	const GEN_FLT x18 = obj_pz + x13 * x12 + x17 * x16 + (1 - x5 * x9) * sensor_z;
	const GEN_FLT x19 = pow(lh_qj, 2);
	const GEN_FLT x20 = pow(lh_qk, 2);
	const GEN_FLT x21 = pow(lh_qi, 2);
	const GEN_FLT x22 = x20 + x21;
	const GEN_FLT x23 = sqrt(x19 + x22 + pow(lh_qw, 2));
	const GEN_FLT x24 = 2 * x23;
	const GEN_FLT x25 = x24 * x18;
	const GEN_FLT x26 = 1 - x24 * x22;
	const GEN_FLT x27 = -x10 + x11;
	const GEN_FLT x28 = x9 * sensor_z;
	const GEN_FLT x29 = obj_qw * obj_qk;
	const GEN_FLT x30 = obj_qj * obj_qi;
	const GEN_FLT x31 = x29 + x30;
	const GEN_FLT x32 = obj_py + x28 * x27 + x31 * x17 + (1 - x7 * x9) * sensor_y;
	const GEN_FLT x33 = x14 + x15;
	const GEN_FLT x34 = -x29 + x30;
	const GEN_FLT x35 = x3 + x6;
	const GEN_FLT x36 = obj_px + x33 * x28 + x34 * x13 + (1 - x9 * x35) * sensor_x;
	const GEN_FLT x37 = lh_qw * lh_qk;
	const GEN_FLT x38 = lh_qj * lh_qi;
	const GEN_FLT x39 = x37 + x38;
	const GEN_FLT x40 = x39 * x24;
	const GEN_FLT x41 = lh_py + x2 * x25 + x32 * x26 + x40 * x36;
	const GEN_FLT x42 = pow(x41, 2);
	const GEN_FLT x43 = 1 - (x19 + x21) * x24;
	const GEN_FLT x44 = x0 + x1;
	const GEN_FLT x45 = x44 * x24;
	const GEN_FLT x46 = lh_qw * lh_qj;
	const GEN_FLT x47 = lh_qk * lh_qi;
	const GEN_FLT x48 = -x46 + x47;
	const GEN_FLT x49 = x48 * x24;
	const GEN_FLT x50 = lh_pz + x43 * x18 + x45 * x32 + x49 * x36;
	const GEN_FLT x51 = x46 + x47;
	const GEN_FLT x52 = -x37 + x38;
	const GEN_FLT x53 = x52 * x24;
	const GEN_FLT x54 = 1 - (x19 + x20) * x24;
	const GEN_FLT x55 = lh_px + x51 * x25 + x53 * x32 + x54 * x36;
	const GEN_FLT x56 = pow(x55, 2);
	const GEN_FLT x57 = x56 + pow(x50, 2);
	const GEN_FLT x58 = x42 + x57;
	const GEN_FLT x59 = pow(x58, -1.0 / 2.0);
	const GEN_FLT x60 = 0.523598775598299 + tilt_0;
	const GEN_FLT x61 = cos(x60);
	const GEN_FLT x62 = pow(x61, -1);
	const GEN_FLT x63 = x62 * x59;
	const GEN_FLT x64 = asin(x63 * x41);
	const GEN_FLT x65 = 8.0108022e-06 * x64;
	const GEN_FLT x66 = -8.0108022e-06 - x65;
	const GEN_FLT x67 = 0.0028679863 + x64 * x66;
	const GEN_FLT x68 = 5.3685255e-06 + x64 * x67;
	const GEN_FLT x69 = 0.0076069798 + x64 * x68;
	const GEN_FLT x70 = x64 * x69;
	const GEN_FLT x71 = -8.0108022e-06 - 1.60216044e-05 * x64;
	const GEN_FLT x72 = x67 + x71 * x64;
	const GEN_FLT x73 = x68 + x72 * x64;
	const GEN_FLT x74 = x69 + x73 * x64;
	const GEN_FLT x75 = x70 + x74 * x64;
	const GEN_FLT x76 = sin(x60);
	const GEN_FLT x77 = tan(x60);
	const GEN_FLT x78 = pow(x57, -1.0 / 2.0);
	const GEN_FLT x79 = x78 * x41;
	const GEN_FLT x80 = x79 * x77;
	const GEN_FLT x81 = atan2(-x50, x55);
	const GEN_FLT x82 = ogeeMag_0 + x81 - asin(x80);
	const GEN_FLT x83 = curve_0 + sin(x82) * ogeePhase_0;
	const GEN_FLT x84 = x83 * x76;
	const GEN_FLT x85 = x61 - x84 * x75;
	const GEN_FLT x86 = pow(x85, -1);
	const GEN_FLT x87 = pow(x64, 2);
	const GEN_FLT x88 = x83 * x87;
	const GEN_FLT x89 = x88 * x86;
	const GEN_FLT x90 = x80 + x89 * x69;
	const GEN_FLT x91 = pow(1 - pow(x90, 2), -1.0 / 2.0);
	const GEN_FLT x92 = x42 / x58;
	const GEN_FLT x93 = pow(1 - x92 / pow(x61, 2), -1.0 / 2.0);
	const GEN_FLT x94 = 4 * x23;
	const GEN_FLT x95 = x94 * x41;
	const GEN_FLT x96 = 2 * x55;
	const GEN_FLT x97 = x50 * x94;
	const GEN_FLT x98 = x54 * x96 + x97 * x48;
	const GEN_FLT x99 = x98 + x95 * x39;
	const GEN_FLT x100 = (1.0 / 2.0) * x41;
	const GEN_FLT x101 = x100 / pow(x58, 3.0 / 2.0);
	const GEN_FLT x102 = x62 * x101;
	const GEN_FLT x103 = x63 * x40 - x99 * x102;
	const GEN_FLT x104 = x93 * x103;
	const GEN_FLT x105 = x66 * x104;
	const GEN_FLT x106 = x64 * (x105 - x65 * x104) + x67 * x104;
	const GEN_FLT x107 = x64 * x106 + x68 * x104;
	const GEN_FLT x108 = pow(x56, -1);
	const GEN_FLT x109 = x50 * x108;
	const GEN_FLT x110 = pow(x55, -1);
	const GEN_FLT x111 = pow(x57, -1);
	const GEN_FLT x112 = x56 * x111;
	const GEN_FLT x113 = (-x49 * x110 + x54 * x109) * x112;
	const GEN_FLT x114 = x42 * x111;
	const GEN_FLT x115 = pow(1 - pow(x77, 2) * x114, -1.0 / 2.0);
	const GEN_FLT x116 = x100 / pow(x57, 3.0 / 2.0);
	const GEN_FLT x117 = x77 * x116;
	const GEN_FLT x118 = x78 * x40;
	const GEN_FLT x119 = x77 * x118 - x98 * x117;
	const GEN_FLT x120 = x113 - x119 * x115;
	const GEN_FLT x121 = cos(x82) * ogeePhase_0;
	const GEN_FLT x122 = x75 * x76;
	const GEN_FLT x123 = x122 * x121;
	const GEN_FLT x124 = 2.40324066e-05 * x64;
	const GEN_FLT x125 = x73 * x93;
	const GEN_FLT x126 = x88 * x69 / pow(x85, 2);
	const GEN_FLT x127 = x86 * x87 * x69;
	const GEN_FLT x128 = x121 * x127;
	const GEN_FLT x129 = 2 * x83 * x86 * x70;
	const GEN_FLT x130 =
		x91 *
		(x119 + x104 * x129 + x120 * x128 -
		 x126 * (-x120 * x123 - x84 * (x64 * x107 +
									   x64 * (x107 + x103 * x125 +
											  x64 * (x106 + x64 * (x105 - x104 * x124 + x71 * x104) + x72 * x104)) +
									   x69 * x104 + x74 * x104)) +
		 x89 * x107);
	const GEN_FLT x131 = -x113;
	const GEN_FLT x132 = -x81;
	const GEN_FLT x133 = cos(-gibPhase_0 + x132 + asin(x90)) * gibMag_0;
	const GEN_FLT x134 = 2 * x50;
	const GEN_FLT x135 = x23 * x108 * x134;
	const GEN_FLT x136 = (-x45 * x110 + x52 * x135) * x112;
	const GEN_FLT x137 = 2 * x41;
	const GEN_FLT x138 = x55 * x94;
	const GEN_FLT x139 = x52 * x138 + x97 * x44;
	const GEN_FLT x140 = x139 + x26 * x137;
	const GEN_FLT x141 = -x102 * x140 + x63 * x26;
	const GEN_FLT x142 = x93 * x141;
	const GEN_FLT x143 = x66 * x142;
	const GEN_FLT x144 = x64 * (x143 - x65 * x142) + x67 * x142;
	const GEN_FLT x145 = x64 * x144 + x68 * x142;
	const GEN_FLT x146 = x116 * x139;
	const GEN_FLT x147 = x78 * x26;
	const GEN_FLT x148 = -x77 * x146 + x77 * x147;
	const GEN_FLT x149 = x121 * (x136 - x115 * x148);
	const GEN_FLT x150 =
		x91 *
		(x148 -
		 x126 * (-x122 * x149 - x84 * (x64 * x145 +
									   x64 * (x145 + x125 * x141 +
											  x64 * (x144 + x64 * (x143 - x124 * x142 + x71 * x142) + x72 * x142)) +
									   x69 * x142 + x74 * x142)) +
		 x127 * x149 + x129 * x142 + x89 * x145);
	const GEN_FLT x151 = -x136;
	const GEN_FLT x152 = (-x43 * x110 + x51 * x135) * x112;
	const GEN_FLT x153 = x43 * x134 + x51 * x138;
	const GEN_FLT x154 = x101 * (x153 + x2 * x95);
	const GEN_FLT x155 = x2 * x24;
	const GEN_FLT x156 = -x62 * x154 + x63 * x155;
	const GEN_FLT x157 = x93 * x156;
	const GEN_FLT x158 = x66 * x157;
	const GEN_FLT x159 = x64 * (x158 - x65 * x157) + x67 * x157;
	const GEN_FLT x160 = x64 * x159 + x68 * x157;
	const GEN_FLT x161 = x78 * x155;
	const GEN_FLT x162 = -x117 * x153 + x77 * x161;
	const GEN_FLT x163 = x152 - x115 * x162;
	const GEN_FLT x164 =
		x91 *
		(x162 -
		 x126 * (-x123 * x163 - x84 * (x64 * x160 +
									   x64 * (x160 + x125 * x156 +
											  x64 * (x159 + x64 * (x158 - x124 * x157 + x71 * x157) + x72 * x157)) +
									   x69 * x157 + x74 * x157)) +
		 x128 * x163 + x129 * x157 + x89 * x160);
	const GEN_FLT x165 = -x152;
	const GEN_FLT x166 = 2 / x8;
	const GEN_FLT x167 = x35 * x166;
	const GEN_FLT x168 = x167 * sensor_x;
	const GEN_FLT x169 = x166 * obj_qw;
	const GEN_FLT x170 = x34 * sensor_y;
	const GEN_FLT x171 = x13 * obj_qk;
	const GEN_FLT x172 = x33 * sensor_z;
	const GEN_FLT x173 = x28 * obj_qj;
	const GEN_FLT x174 = -x171 + x173 - x168 * obj_qw + x169 * x170 + x169 * x172;
	const GEN_FLT x175 = x31 * sensor_x;
	const GEN_FLT x176 = x7 * x166;
	const GEN_FLT x177 = x176 * sensor_y;
	const GEN_FLT x178 = x17 * obj_qk;
	const GEN_FLT x179 = x27 * sensor_z;
	const GEN_FLT x180 = x28 * obj_qi;
	const GEN_FLT x181 = x178 - x180 + x169 * x175 + x169 * x179 - x177 * obj_qw;
	const GEN_FLT x182 = x17 * obj_qj;
	const GEN_FLT x183 = x16 * sensor_x;
	const GEN_FLT x184 = x12 * sensor_y;
	const GEN_FLT x185 = x166 * x184;
	const GEN_FLT x186 = x13 * obj_qi;
	const GEN_FLT x187 = x5 * x166;
	const GEN_FLT x188 = x187 * sensor_z;
	const GEN_FLT x189 = -x182 + x186 + x169 * x183 + x185 * obj_qw - x188 * obj_qw;
	const GEN_FLT x190 = x189 * x155 + x26 * x181 + x40 * x174;
	const GEN_FLT x191 = x51 * x24;
	const GEN_FLT x192 = x189 * x191 + x53 * x181 + x54 * x174;
	const GEN_FLT x193 = x43 * x189 + x45 * x181 + x49 * x174;
	const GEN_FLT x194 = x193 * x134 + x96 * x192;
	const GEN_FLT x195 = x194 + x190 * x137;
	const GEN_FLT x196 = -x102 * x195 + x63 * x190;
	const GEN_FLT x197 = x93 * x196;
	const GEN_FLT x198 = x66 * x197;
	const GEN_FLT x199 = x64 * (x198 - x65 * x197) + x67 * x197;
	const GEN_FLT x200 = x64 * x199 + x68 * x197;
	const GEN_FLT x201 = (x109 * x192 - x110 * x193) * x112;
	const GEN_FLT x202 = x78 * x190;
	const GEN_FLT x203 = -x117 * x194 + x77 * x202;
	const GEN_FLT x204 = x201 - x203 * x115;
	const GEN_FLT x205 =
		x91 *
		(x203 -
		 x126 * (-x204 * x123 - x84 * (x64 * x200 +
									   x64 * (x200 + x125 * x196 +
											  x64 * (x199 + x64 * (x198 - x124 * x197 + x71 * x197) + x72 * x197)) +
									   x69 * x197 + x74 * x197)) +
		 x129 * x197 + x204 * x128 + x89 * x200);
	const GEN_FLT x206 = -x201;
	const GEN_FLT x207 = x166 * obj_qi;
	const GEN_FLT x208 = x13 * obj_qj;
	const GEN_FLT x209 = x28 * obj_qk;
	const GEN_FLT x210 = x208 + x209 - x168 * obj_qi + x207 * x170 + x207 * x172;
	const GEN_FLT x211 = 4 * x8;
	const GEN_FLT x212 = -x211 * obj_qi;
	const GEN_FLT x213 = x28 * obj_qw;
	const GEN_FLT x214 = x182 - x213 + x207 * x175 + x207 * x179 + (x212 - x176 * obj_qi) * sensor_y;
	const GEN_FLT x215 = x13 * obj_qw;
	const GEN_FLT x216 = x178 + x215 + x185 * obj_qi + x207 * x183 + (x212 - x187 * obj_qi) * sensor_z;
	const GEN_FLT x217 = x216 * x191 + x53 * x214 + x54 * x210;
	const GEN_FLT x218 = x43 * x216 + x45 * x214 + x49 * x210;
	const GEN_FLT x219 = (x217 * x109 - x218 * x110) * x112;
	const GEN_FLT x220 = x216 * x155 + x26 * x214 + x40 * x210;
	const GEN_FLT x221 = x218 * x134 + x96 * x217;
	const GEN_FLT x222 = x221 + x220 * x137;
	const GEN_FLT x223 = -x222 * x102 + x63 * x220;
	const GEN_FLT x224 = x93 * x223;
	const GEN_FLT x225 = x66 * x224;
	const GEN_FLT x226 = x64 * (x225 - x65 * x224) + x67 * x224;
	const GEN_FLT x227 = x64 * x226 + x68 * x224;
	const GEN_FLT x228 = x78 * x220;
	const GEN_FLT x229 = -x221 * x117 + x77 * x228;
	const GEN_FLT x230 = x219 - x229 * x115;
	const GEN_FLT x231 =
		x91 *
		(x229 -
		 x126 * (-x230 * x123 - x84 * (x64 * x227 +
									   x64 * (x227 + x223 * x125 +
											  x64 * (x226 + x64 * (x225 - x224 * x124 + x71 * x224) + x72 * x224)) +
									   x69 * x224 + x74 * x224)) +
		 x224 * x129 + x230 * x128 + x89 * x227);
	const GEN_FLT x232 = -x219;
	const GEN_FLT x233 = -x211 * obj_qj;
	const GEN_FLT x234 = x166 * obj_qj;
	const GEN_FLT x235 = x186 + x213 + x234 * x170 + x234 * x172 + (x233 - x167 * obj_qj) * sensor_x;
	const GEN_FLT x236 = x17 * obj_qi;
	const GEN_FLT x237 = x209 + x236 - x177 * obj_qj + x234 * x175 + x234 * x179;
	const GEN_FLT x238 = x17 * obj_qw;
	const GEN_FLT x239 = x171 - x238 + x185 * obj_qj + x234 * x183 + (x233 - x187 * obj_qj) * sensor_z;
	const GEN_FLT x240 = x239 * x191 + x53 * x237 + x54 * x235;
	const GEN_FLT x241 = x43 * x239 + x45 * x237 + x49 * x235;
	const GEN_FLT x242 = (x240 * x109 - x241 * x110) * x112;
	const GEN_FLT x243 = x239 * x155 + x26 * x237 + x40 * x235;
	const GEN_FLT x244 = x241 * x134 + x96 * x240;
	const GEN_FLT x245 = x101 * (x244 + x243 * x137);
	const GEN_FLT x246 = x59 * x243;
	const GEN_FLT x247 = -x62 * x245 + x62 * x246;
	const GEN_FLT x248 = x93 * x247;
	const GEN_FLT x249 = x66 * x248;
	const GEN_FLT x250 = x64 * (x249 - x65 * x248) + x67 * x248;
	const GEN_FLT x251 = x64 * x250 + x68 * x248;
	const GEN_FLT x252 = x78 * x243;
	const GEN_FLT x253 = -x244 * x117 + x77 * x252;
	const GEN_FLT x254 = x242 - x253 * x115;
	const GEN_FLT x255 =
		x91 *
		(x253 -
		 x126 * (-x254 * x123 - x84 * (x64 * x251 +
									   x64 * (x251 + x247 * x125 +
											  x64 * (x250 + x64 * (x249 - x248 * x124 + x71 * x248) + x72 * x248)) +
									   x69 * x248 + x74 * x248)) +
		 x248 * x129 + x254 * x128 + x89 * x251);
	const GEN_FLT x256 = -x242;
	const GEN_FLT x257 = x166 * obj_qk;
	const GEN_FLT x258 = -x211 * obj_qk;
	const GEN_FLT x259 = x180 - x215 + x257 * x170 + x257 * x172 + (x258 - x167 * obj_qk) * sensor_x;
	const GEN_FLT x260 = x173 + x238 + x257 * x175 + x257 * x179 + (x258 - x176 * obj_qk) * sensor_y;
	const GEN_FLT x261 = x208 + x236 - x188 * obj_qk + x257 * x183 + x257 * x184;
	const GEN_FLT x262 = x261 * x191 + x53 * x260 + x54 * x259;
	const GEN_FLT x263 = x43 * x261 + x45 * x260 + x49 * x259;
	const GEN_FLT x264 = (x262 * x109 - x263 * x110) * x112;
	const GEN_FLT x265 = x26 * x260 + x261 * x155 + x40 * x259;
	const GEN_FLT x266 = x263 * x134 + x96 * x262;
	const GEN_FLT x267 = x266 + x265 * x137;
	const GEN_FLT x268 = -x267 * x102 + x63 * x265;
	const GEN_FLT x269 = x93 * x268;
	const GEN_FLT x270 = x66 * x269;
	const GEN_FLT x271 = x64 * (x270 - x65 * x269) + x67 * x269;
	const GEN_FLT x272 = x64 * x271 + x68 * x269;
	const GEN_FLT x273 = x78 * x265;
	const GEN_FLT x274 = -x266 * x117 + x77 * x273;
	const GEN_FLT x275 = x264 - x274 * x115;
	const GEN_FLT x276 =
		x91 *
		(x274 -
		 x126 * (-x275 * x123 - x84 * (x64 * x272 +
									   x64 * (x272 + x268 * x125 +
											  x64 * (x271 + x64 * (x270 - x269 * x124 + x71 * x269) + x72 * x269)) +
									   x69 * x269 + x74 * x269)) +
		 x269 * x129 + x275 * x128 + x89 * x272);
	const GEN_FLT x277 = -x264;
	const GEN_FLT x278 = 0.523598775598299 - tilt_1;
	const GEN_FLT x279 = cos(x278);
	const GEN_FLT x280 = pow(x279, -1);
	const GEN_FLT x281 = x59 * x280;
	const GEN_FLT x282 = asin(x41 * x281);
	const GEN_FLT x283 = 8.0108022e-06 * x282;
	const GEN_FLT x284 = -8.0108022e-06 - x283;
	const GEN_FLT x285 = 0.0028679863 + x282 * x284;
	const GEN_FLT x286 = 5.3685255e-06 + x282 * x285;
	const GEN_FLT x287 = 0.0076069798 + x286 * x282;
	const GEN_FLT x288 = x287 * x282;
	const GEN_FLT x289 = -8.0108022e-06 - 1.60216044e-05 * x282;
	const GEN_FLT x290 = x285 + x289 * x282;
	const GEN_FLT x291 = x286 + x290 * x282;
	const GEN_FLT x292 = x287 + x291 * x282;
	const GEN_FLT x293 = x288 + x292 * x282;
	const GEN_FLT x294 = tan(x278);
	const GEN_FLT x295 = -x79 * x294;
	const GEN_FLT x296 = ogeeMag_1 + x81 - asin(x295);
	const GEN_FLT x297 = curve_1 + sin(x296) * ogeePhase_1;
	const GEN_FLT x298 = sin(x278);
	const GEN_FLT x299 = x297 * x298;
	const GEN_FLT x300 = x279 + x299 * x293;
	const GEN_FLT x301 = pow(x300, -1);
	const GEN_FLT x302 = pow(x282, 2);
	const GEN_FLT x303 = x297 * x302;
	const GEN_FLT x304 = x301 * x303;
	const GEN_FLT x305 = x295 + x287 * x304;
	const GEN_FLT x306 = pow(1 - pow(x305, 2), -1.0 / 2.0);
	const GEN_FLT x307 = pow(1 - pow(x294, 2) * x114, -1.0 / 2.0);
	const GEN_FLT x308 = x294 * x116;
	const GEN_FLT x309 = -x294 * x118 + x98 * x308;
	const GEN_FLT x310 = x113 - x309 * x307;
	const GEN_FLT x311 = cos(x296) * ogeePhase_1;
	const GEN_FLT x312 = x287 * x301 * x302;
	const GEN_FLT x313 = x312 * x311;
	const GEN_FLT x314 = pow(1 - x92 / pow(x279, 2), -1.0 / 2.0);
	const GEN_FLT x315 = x280 * x101;
	const GEN_FLT x316 = x40 * x281 - x99 * x315;
	const GEN_FLT x317 = x314 * x316;
	const GEN_FLT x318 = 2 * x297 * x288 * x301;
	const GEN_FLT x319 = x298 * x293;
	const GEN_FLT x320 = x311 * x319;
	const GEN_FLT x321 = x284 * x317;
	const GEN_FLT x322 = 2.40324066e-05 * x282;
	const GEN_FLT x323 = x282 * (x321 - x283 * x317) + x285 * x317;
	const GEN_FLT x324 = x286 * x314;
	const GEN_FLT x325 = x282 * x323 + x324 * x316;
	const GEN_FLT x326 = x287 * x314;
	const GEN_FLT x327 = x287 * x303 / pow(x300, 2);
	const GEN_FLT x328 =
		x306 * (x309 + x304 * x325 + x313 * x310 + x317 * x318 -
				x327 * (x299 * (x282 * x325 +
								x282 * (x325 + x282 * (x323 + x282 * (x321 + x289 * x317 - x322 * x317) + x290 * x317) +
										x291 * x317) +
								x292 * x317 + x326 * x316) +
						x310 * x320));
	const GEN_FLT x329 = cos(-gibPhase_1 + x132 + asin(x305)) * gibMag_1;
	const GEN_FLT x330 = x294 * x146 - x294 * x147;
	const GEN_FLT x331 = x136 - x307 * x330;
	const GEN_FLT x332 = x26 * x281 - x315 * x140;
	const GEN_FLT x333 = x314 * x332;
	const GEN_FLT x334 = x284 * x333;
	const GEN_FLT x335 = x282 * (x334 - x283 * x333) + x285 * x333;
	const GEN_FLT x336 = x282 * x335 + x324 * x332;
	const GEN_FLT x337 =
		x306 * (x330 + x304 * x336 + x313 * x331 -
				x327 * (x299 * (x282 * x336 +
								x282 * (x336 + x282 * (x335 + x282 * (x334 + x289 * x333 - x322 * x333) + x290 * x333) +
										x291 * x333) +
								x292 * x333 + x326 * x332) +
						x320 * x331) +
				x333 * x318);
	const GEN_FLT x338 = -x294 * x161 + x308 * x153;
	const GEN_FLT x339 = x311 * (x152 - x307 * x338);
	const GEN_FLT x340 = -x280 * x154 + x281 * x155;
	const GEN_FLT x341 = x314 * x340;
	const GEN_FLT x342 = x284 * x341;
	const GEN_FLT x343 = x282 * (x342 - x283 * x341) + x285 * x341;
	const GEN_FLT x344 = x282 * x343 + x324 * x340;
	const GEN_FLT x345 =
		x306 * (x338 + x304 * x344 + x318 * x341 -
				x327 * (x299 * (x282 * x344 +
								x282 * (x344 + x282 * (x343 + x282 * (x342 + x289 * x341 - x322 * x341) + x290 * x341) +
										x291 * x341) +
								x292 * x341 + x326 * x340) +
						x339 * x319) +
				x339 * x312);
	const GEN_FLT x346 = -x202 * x294 + x308 * x194;
	const GEN_FLT x347 = x201 - x307 * x346;
	const GEN_FLT x348 = (x281 * x190 - x315 * x195) * x314;
	const GEN_FLT x349 = x284 * x348;
	const GEN_FLT x350 = x282 * (x349 - x283 * x348) + x285 * x348;
	const GEN_FLT x351 = x282 * x350 + x286 * x348;
	const GEN_FLT x352 =
		x306 * (x346 + x313 * x347 -
				x327 * (x299 * (x282 * x351 +
								x282 * (x351 + x282 * (x350 + x282 * (x349 + x289 * x348 - x322 * x348) + x290 * x348) +
										x291 * x348) +
								x287 * x348 + x292 * x348) +
						x320 * x347) +
				x348 * x318 + x351 * x304);
	const GEN_FLT x353 = x221 * x308 - x294 * x228;
	const GEN_FLT x354 = x219 - x353 * x307;
	const GEN_FLT x355 = -x222 * x315 + x281 * x220;
	const GEN_FLT x356 = x355 * x314;
	const GEN_FLT x357 = x284 * x356;
	const GEN_FLT x358 = x282 * (x357 - x283 * x356) + x285 * x356;
	const GEN_FLT x359 = x282 * x358 + x355 * x324;
	const GEN_FLT x360 =
		x306 * (x353 -
				x327 * (x299 * (x282 * x359 +
								x282 * (x359 + x282 * (x358 + x282 * (x357 + x289 * x356 - x356 * x322) + x290 * x356) +
										x291 * x356) +
								x287 * x356 + x292 * x356) +
						x354 * x320) +
				x354 * x313 + x356 * x318 + x359 * x304);
	const GEN_FLT x361 = x244 * x308 - x294 * x252;
	const GEN_FLT x362 = x242 - x361 * x307;
	const GEN_FLT x363 = -x280 * x245 + x280 * x246;
	const GEN_FLT x364 = x363 * x314;
	const GEN_FLT x365 = x284 * x364;
	const GEN_FLT x366 = x282 * (x365 - x283 * x364) + x285 * x364;
	const GEN_FLT x367 = x282 * x366 + x363 * x324;
	const GEN_FLT x368 =
		x306 * (x361 -
				x327 * (x299 * (x282 * x367 +
								x282 * (x367 + x282 * (x366 + x282 * (x365 + x289 * x364 - x364 * x322) + x290 * x364) +
										x291 * x364) +
								x292 * x364 + x363 * x326) +
						x362 * x320) +
				x362 * x313 + x364 * x318 + x367 * x304);
	const GEN_FLT x369 = x266 * x308 - x273 * x294;
	const GEN_FLT x370 = x264 - x369 * x307;
	const GEN_FLT x371 = (x265 * x281 - x267 * x315) * x314;
	const GEN_FLT x372 = x284 * x371;
	const GEN_FLT x373 = x282 * (x372 - x283 * x371) + x285 * x371;
	const GEN_FLT x374 = x282 * x373 + x286 * x371;
	const GEN_FLT x375 =
		x306 * (x369 -
				x327 * (x299 * (x282 * x374 +
								x282 * (x374 + x282 * (x373 + x282 * (x372 + x289 * x371 - x371 * x322) + x290 * x371) +
										x291 * x371) +
								x287 * x371 + x292 * x371) +
						x370 * x320) +
				x370 * x313 + x371 * x318 + x374 * x304);
	*(out++) = x113 - x130 - (x130 + x131) * x133;
	*(out++) = x136 - x150 - (x150 + x151) * x133;
	*(out++) = x152 - x164 - (x164 + x165) * x133;
	*(out++) = x201 - x205 - (x205 + x206) * x133;
	*(out++) = x219 - x231 - (x231 + x232) * x133;
	*(out++) = x242 - x255 - (x255 + x256) * x133;
	*(out++) = x264 - x276 - (x276 + x277) * x133;
	*(out++) = x113 - x328 - (x131 + x328) * x329;
	*(out++) = x136 - x337 - (x151 + x337) * x329;
	*(out++) = x152 - x345 - (x165 + x345) * x329;
	*(out++) = x201 - x352 - (x206 + x352) * x329;
	*(out++) = x219 - x360 - (x232 + x360) * x329;
	*(out++) = x242 - x368 - (x256 + x368) * x329;
	*(out++) = x264 - x375 - (x277 + x375) * x329;
}

// Jacobian of reproject_gen2 wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_reproject_gen2_jac_sensor_pt(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
													const SurvivePose *lh_p, const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt(x1 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - (x0 + x1) * x5;
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = pow(obj_qj, 2);
	const GEN_FLT x9 = pow(obj_qi, 2);
	const GEN_FLT x10 = x7 + x9;
	const GEN_FLT x11 = sqrt(x10 + x8 + pow(obj_qw, 2));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 - (x7 + x8) * x12;
	const GEN_FLT x14 = lh_qw * lh_qk;
	const GEN_FLT x15 = lh_qj * lh_qi;
	const GEN_FLT x16 = -x14 + x15;
	const GEN_FLT x17 = obj_qw * obj_qk;
	const GEN_FLT x18 = obj_qj * obj_qi;
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 4 * x4 * x11;
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = obj_qw * obj_qj;
	const GEN_FLT x23 = obj_qk * obj_qi;
	const GEN_FLT x24 = -x22 + x23;
	const GEN_FLT x25 = lh_qw * lh_qj;
	const GEN_FLT x26 = lh_qk * lh_qi;
	const GEN_FLT x27 = x25 + x26;
	const GEN_FLT x28 = x20 * x27;
	const GEN_FLT x29 = x21 * x16 + x24 * x28 + x6 * x13;
	const GEN_FLT x30 = 1 - (x1 + x2) * x5;
	const GEN_FLT x31 = 1 - (x8 + x9) * x12;
	const GEN_FLT x32 = obj_qw * obj_qi;
	const GEN_FLT x33 = obj_qk * obj_qj;
	const GEN_FLT x34 = x32 + x33;
	const GEN_FLT x35 = x12 * sensor_y;
	const GEN_FLT x36 = x12 * sensor_x;
	const GEN_FLT x37 = obj_pz + x31 * sensor_z + x34 * x35 + x36 * x24;
	const GEN_FLT x38 = lh_qw * lh_qi;
	const GEN_FLT x39 = lh_qk * lh_qj;
	const GEN_FLT x40 = x38 + x39;
	const GEN_FLT x41 = -x32 + x33;
	const GEN_FLT x42 = x12 * sensor_z;
	const GEN_FLT x43 = 1 - x12 * x10;
	const GEN_FLT x44 = obj_py + x36 * x19 + x41 * x42 + x43 * sensor_y;
	const GEN_FLT x45 = x5 * x44;
	const GEN_FLT x46 = x22 + x23;
	const GEN_FLT x47 = -x17 + x18;
	const GEN_FLT x48 = obj_px + x13 * sensor_x + x42 * x46 + x47 * x35;
	const GEN_FLT x49 = -x25 + x26;
	const GEN_FLT x50 = x5 * x49;
	const GEN_FLT x51 = lh_pz + x30 * x37 + x40 * x45 + x50 * x48;
	const GEN_FLT x52 = x5 * x37;
	const GEN_FLT x53 = lh_px + x45 * x16 + x52 * x27 + x6 * x48;
	const GEN_FLT x54 = pow(x53, 2);
	const GEN_FLT x55 = x51 / x54;
	const GEN_FLT x56 = pow(x53, -1);
	const GEN_FLT x57 = x30 * x12;
	const GEN_FLT x58 = x40 * x21 + x50 * x13 + x57 * x24;
	const GEN_FLT x59 = x54 + pow(x51, 2);
	const GEN_FLT x60 = pow(x59, -1);
	const GEN_FLT x61 = x60 * x54;
	const GEN_FLT x62 = (x55 * x29 - x58 * x56) * x61;
	const GEN_FLT x63 = 0.523598775598299 + tilt_0;
	const GEN_FLT x64 = cos(x63);
	const GEN_FLT x65 = pow(x64, -1);
	const GEN_FLT x66 = -x38 + x39;
	const GEN_FLT x67 = 1 - x3 * x5;
	const GEN_FLT x68 = x14 + x15;
	const GEN_FLT x69 = x5 * x68;
	const GEN_FLT x70 = lh_py + x66 * x52 + x67 * x44 + x69 * x48;
	const GEN_FLT x71 = pow(x70, 2);
	const GEN_FLT x72 = x59 + x71;
	const GEN_FLT x73 = pow(x72, -1.0 / 2.0);
	const GEN_FLT x74 = x70 * x73;
	const GEN_FLT x75 = asin(x74 * x65);
	const GEN_FLT x76 = 8.0108022e-06 * x75;
	const GEN_FLT x77 = -8.0108022e-06 - x76;
	const GEN_FLT x78 = 0.0028679863 + x75 * x77;
	const GEN_FLT x79 = 5.3685255e-06 + x78 * x75;
	const GEN_FLT x80 = 0.0076069798 + x79 * x75;
	const GEN_FLT x81 = x80 * x75;
	const GEN_FLT x82 = -8.0108022e-06 - 1.60216044e-05 * x75;
	const GEN_FLT x83 = x78 + x82 * x75;
	const GEN_FLT x84 = x79 + x83 * x75;
	const GEN_FLT x85 = x80 + x84 * x75;
	const GEN_FLT x86 = x81 + x85 * x75;
	const GEN_FLT x87 = sin(x63);
	const GEN_FLT x88 = tan(x63);
	const GEN_FLT x89 = pow(x59, -1.0 / 2.0);
	const GEN_FLT x90 = x88 * x89;
	const GEN_FLT x91 = x70 * x90;
	const GEN_FLT x92 = atan2(-x51, x53);
	const GEN_FLT x93 = ogeeMag_0 + x92 - asin(x91);
	const GEN_FLT x94 = curve_0 + sin(x93) * ogeePhase_0;
	const GEN_FLT x95 = x87 * x94;
	const GEN_FLT x96 = x64 - x86 * x95;
	const GEN_FLT x97 = pow(x96, -1);
	const GEN_FLT x98 = pow(x75, 2);
	const GEN_FLT x99 = x98 * x94;
	const GEN_FLT x100 = x99 * x97;
	const GEN_FLT x101 = x91 + x80 * x100;
	const GEN_FLT x102 = pow(1 - pow(x101, 2), -1.0 / 2.0);
	const GEN_FLT x103 = x71 / x72;
	const GEN_FLT x104 = pow(1 - x103 / pow(x64, 2), -1.0 / 2.0);
	const GEN_FLT x105 = x67 * x12;
	const GEN_FLT x106 = x66 * x20;
	const GEN_FLT x107 = x19 * x105 + x24 * x106 + x69 * x13;
	const GEN_FLT x108 = 2 * x70;
	const GEN_FLT x109 = 2 * x53;
	const GEN_FLT x110 = 2 * x51;
	const GEN_FLT x111 = x29 * x109 + x58 * x110;
	const GEN_FLT x112 = (1.0 / 2.0) * x70;
	const GEN_FLT x113 = x112 / pow(x72, 3.0 / 2.0);
	const GEN_FLT x114 = x113 * (x111 + x108 * x107);
	const GEN_FLT x115 = x73 * x107;
	const GEN_FLT x116 = (-x65 * x114 + x65 * x115) * x104;
	const GEN_FLT x117 = x77 * x116;
	const GEN_FLT x118 = x75 * (x117 - x76 * x116) + x78 * x116;
	const GEN_FLT x119 = x75 * x118 + x79 * x116;
	const GEN_FLT x120 = 2 * x81 * x97 * x94;
	const GEN_FLT x121 = x112 / pow(x59, 3.0 / 2.0);
	const GEN_FLT x122 = x88 * x121;
	const GEN_FLT x123 = -x111 * x122 + x90 * x107;
	const GEN_FLT x124 = x71 * x60;
	const GEN_FLT x125 = pow(1 - pow(x88, 2) * x124, -1.0 / 2.0);
	const GEN_FLT x126 = cos(x93) * ogeePhase_0;
	const GEN_FLT x127 = x126 * (x62 - x123 * x125);
	const GEN_FLT x128 = x86 * x87;
	const GEN_FLT x129 = 2.40324066e-05 * x75;
	const GEN_FLT x130 = x80 * x99 / pow(x96, 2);
	const GEN_FLT x131 = x80 * x98 * x97;
	const GEN_FLT x132 =
		x102 *
		(x123 + x100 * x119 + x116 * x120 + x127 * x131 -
		 x130 *
			 (-x128 * x127 -
			  x95 * (x75 * x119 +
					 x75 * (x119 + x75 * (x118 + x75 * (x117 - x116 * x129 + x82 * x116) + x83 * x116) + x84 * x116) +
					 x80 * x116 + x85 * x116)));
	const GEN_FLT x133 = -x62;
	const GEN_FLT x134 = -x92;
	const GEN_FLT x135 = cos(-gibPhase_0 + x134 + asin(x101)) * gibMag_0;
	const GEN_FLT x136 = x5 * x43;
	const GEN_FLT x137 = x6 * x12;
	const GEN_FLT x138 = x16 * x136 + x34 * x28 + x47 * x137;
	const GEN_FLT x139 = x47 * x20;
	const GEN_FLT x140 = x40 * x136 + x49 * x139 + x57 * x34;
	const GEN_FLT x141 = (x55 * x138 - x56 * x140) * x61;
	const GEN_FLT x142 = x34 * x106 + x67 * x43 + x68 * x139;
	const GEN_FLT x143 = x109 * x138 + x110 * x140;
	const GEN_FLT x144 = x143 + x108 * x142;
	const GEN_FLT x145 = x65 * x113;
	const GEN_FLT x146 = x73 * x142;
	const GEN_FLT x147 = x104 * (-x144 * x145 + x65 * x146);
	const GEN_FLT x148 = x77 * x147;
	const GEN_FLT x149 = x75 * (x148 - x76 * x147) + x78 * x147;
	const GEN_FLT x150 = x75 * x149 + x79 * x147;
	const GEN_FLT x151 = x89 * x142;
	const GEN_FLT x152 = -x122 * x143 + x88 * x151;
	const GEN_FLT x153 = x141 - x125 * x152;
	const GEN_FLT x154 = x128 * x126;
	const GEN_FLT x155 = x126 * x131;
	const GEN_FLT x156 =
		x102 *
		(x152 + x100 * x150 + x120 * x147 -
		 x130 *
			 (-x153 * x154 -
			  x95 * (x75 * x150 +
					 x75 * (x150 + x75 * (x149 + x75 * (x148 - x129 * x147 + x82 * x147) + x83 * x147) + x84 * x147) +
					 x80 * x147 + x85 * x147)) +
		 x153 * x155);
	const GEN_FLT x157 = -x141;
	const GEN_FLT x158 = x41 * x20;
	const GEN_FLT x159 = x5 * x31;
	const GEN_FLT x160 = x16 * x158 + x27 * x159 + x46 * x137;
	const GEN_FLT x161 = x46 * x20;
	const GEN_FLT x162 = x30 * x31 + x40 * x158 + x49 * x161;
	const GEN_FLT x163 = (x55 * x160 - x56 * x162) * x61;
	const GEN_FLT x164 = x41 * x105 + x66 * x159 + x68 * x161;
	const GEN_FLT x165 = x109 * x160 + x110 * x162;
	const GEN_FLT x166 = x165 + x108 * x164;
	const GEN_FLT x167 = x73 * x164;
	const GEN_FLT x168 = x104 * (-x166 * x145 + x65 * x167);
	const GEN_FLT x169 = x77 * x168;
	const GEN_FLT x170 = x75 * (x169 - x76 * x168) + x78 * x168;
	const GEN_FLT x171 = x75 * x170 + x79 * x168;
	const GEN_FLT x172 = x89 * x164;
	const GEN_FLT x173 = -x122 * x165 + x88 * x172;
	const GEN_FLT x174 = x163 - x125 * x173;
	const GEN_FLT x175 =
		x102 *
		(x173 + x100 * x171 + x120 * x168 -
		 x130 *
			 (-x174 * x154 -
			  x95 * (x75 * x171 +
					 x75 * (x171 + x75 * (x170 + x75 * (x169 - x129 * x168 + x82 * x168) + x83 * x168) + x84 * x168) +
					 x80 * x168 + x85 * x168)) +
		 x174 * x155);
	const GEN_FLT x176 = -x163;
	const GEN_FLT x177 = 0.523598775598299 - tilt_1;
	const GEN_FLT x178 = cos(x177);
	const GEN_FLT x179 = pow(x178, -1);
	const GEN_FLT x180 = asin(x74 * x179);
	const GEN_FLT x181 = 8.0108022e-06 * x180;
	const GEN_FLT x182 = -8.0108022e-06 - x181;
	const GEN_FLT x183 = 0.0028679863 + x180 * x182;
	const GEN_FLT x184 = 5.3685255e-06 + x180 * x183;
	const GEN_FLT x185 = 0.0076069798 + x180 * x184;
	const GEN_FLT x186 = pow(x180, 2);
	const GEN_FLT x187 = tan(x177);
	const GEN_FLT x188 = x89 * x187;
	const GEN_FLT x189 = -x70 * x188;
	const GEN_FLT x190 = ogeeMag_1 + x92 - asin(x189);
	const GEN_FLT x191 = curve_1 + sin(x190) * ogeePhase_1;
	const GEN_FLT x192 = x180 * x185;
	const GEN_FLT x193 = -8.0108022e-06 - 1.60216044e-05 * x180;
	const GEN_FLT x194 = x183 + x180 * x193;
	const GEN_FLT x195 = x184 + x180 * x194;
	const GEN_FLT x196 = x185 + x180 * x195;
	const GEN_FLT x197 = x192 + x180 * x196;
	const GEN_FLT x198 = sin(x177);
	const GEN_FLT x199 = x191 * x198;
	const GEN_FLT x200 = x178 + x197 * x199;
	const GEN_FLT x201 = pow(x200, -1);
	const GEN_FLT x202 = x201 * x191;
	const GEN_FLT x203 = x202 * x186;
	const GEN_FLT x204 = x189 + x203 * x185;
	const GEN_FLT x205 = pow(1 - pow(x204, 2), -1.0 / 2.0);
	const GEN_FLT x206 = x121 * x187;
	const GEN_FLT x207 = -x107 * x188 + x206 * x111;
	const GEN_FLT x208 = pow(1 - x124 * pow(x187, 2), -1.0 / 2.0);
	const GEN_FLT x209 = cos(x190) * ogeePhase_1;
	const GEN_FLT x210 = x209 * (x62 - x208 * x207);
	const GEN_FLT x211 = x186 * x185;
	const GEN_FLT x212 = x211 * x201;
	const GEN_FLT x213 = pow(1 - x103 / pow(x178, 2), -1.0 / 2.0);
	const GEN_FLT x214 = -x114 * x179 + x115 * x179;
	const GEN_FLT x215 = x213 * x214;
	const GEN_FLT x216 = 2 * x202 * x192;
	const GEN_FLT x217 = x197 * x198;
	const GEN_FLT x218 = x215 * x182;
	const GEN_FLT x219 = 2.40324066e-05 * x180;
	const GEN_FLT x220 = x180 * (x218 - x215 * x181) + x215 * x183;
	const GEN_FLT x221 = x215 * x184 + x220 * x180;
	const GEN_FLT x222 = x213 * x185;
	const GEN_FLT x223 = x213 * x196;
	const GEN_FLT x224 = x211 * x191 / pow(x200, 2);
	const GEN_FLT x225 =
		x205 * (x207 + x203 * x221 + x210 * x212 + x215 * x216 -
				x224 * (x199 * (x180 * (x221 + x180 * (x220 + x180 * (x218 + x215 * x193 - x219 * x215) + x215 * x194) +
										x215 * x195) +
								x214 * x222 + x214 * x223 + x221 * x180) +
						x210 * x217));
	const GEN_FLT x226 = cos(-gibPhase_1 + x134 + asin(x204)) * gibMag_1;
	const GEN_FLT x227 = -x187 * x151 + x206 * x143;
	const GEN_FLT x228 = x141 - x208 * x227;
	const GEN_FLT x229 = x212 * x209;
	const GEN_FLT x230 = x113 * x179;
	const GEN_FLT x231 = x179 * x146 - x230 * x144;
	const GEN_FLT x232 = x213 * x231;
	const GEN_FLT x233 = x217 * x209;
	const GEN_FLT x234 = x232 * x182;
	const GEN_FLT x235 = x180 * (x234 - x232 * x181) + x232 * x183;
	const GEN_FLT x236 = x232 * x184 + x235 * x180;
	const GEN_FLT x237 =
		x205 * (x227 + x216 * x232 -
				x224 * (x199 * (x180 * (x236 + x180 * (x235 + x180 * (x234 - x219 * x232 + x232 * x193) + x232 * x194) +
										x232 * x195) +
								x231 * x222 + x231 * x223 + x236 * x180) +
						x233 * x228) +
				x228 * x229 + x236 * x203);
	const GEN_FLT x238 = -x172 * x187 + x206 * x165;
	const GEN_FLT x239 = x163 - x238 * x208;
	const GEN_FLT x240 = x167 * x179 - x230 * x166;
	const GEN_FLT x241 = x213 * x240;
	const GEN_FLT x242 = x241 * x182;
	const GEN_FLT x243 = x180 * (x242 - x241 * x181) + x241 * x183;
	const GEN_FLT x244 = x241 * x184 + x243 * x180;
	const GEN_FLT x245 =
		x205 * (x238 + x203 * x244 + x216 * x241 -
				x224 * (x199 * (x180 * (x244 + x180 * (x243 + x180 * (x242 - x219 * x241 + x241 * x193) + x241 * x194) +
										x241 * x195) +
								x222 * x240 + x223 * x240 + x244 * x180) +
						x233 * x239) +
				x239 * x229);
	*(out++) = -x132 + x62 - (x132 + x133) * x135;
	*(out++) = x141 - x156 - (x156 + x157) * x135;
	*(out++) = x163 - x175 - (x175 + x176) * x135;
	*(out++) = -x225 + x62 - (x133 + x225) * x226;
	*(out++) = x141 - x237 - (x157 + x237) * x226;
	*(out++) = x163 - x245 - (x176 + x245) * x226;
}

// Jacobian of reproject_gen2 wrt [lh_px, lh_py, lh_pz, lh_qw, lh_qi, lh_qj, lh_qk]
static inline void gen_reproject_gen2_jac_lh_p(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
											   const SurvivePose *lh_p, const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qk, 2);
	const GEN_FLT x4 = x0 + x3;
	const GEN_FLT x5 = sqrt(x1 + x4 + pow(lh_qw, 2));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = pow(obj_qk, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = 2 * sqrt(x10 + x7 + pow(obj_qw, 2));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 = obj_pz + (1 - (x7 + x8) * x11) * sensor_z + (x12 + x13) * x14 + (-x15 + x16) * x17;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + (1 - x11 * x10) * sensor_y + (-x12 + x13) * x22 + (x23 + x24) * x17;
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = -x27 + x28;
	const GEN_FLT x30 = obj_px + (1 - (x7 + x9) * x11) * sensor_x + (x15 + x16) * x22 + (-x23 + x24) * x14;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + x18 * (1 - x2 * x6) + x21 * x26 + x31 * x29;
	const GEN_FLT x33 = x27 + x28;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = -x35 + x36;
	const GEN_FLT x38 = lh_px + x30 * (1 - x4 * x6) + x34 * x33 + x37 * x26;
	const GEN_FLT x39 = pow(x38, 2);
	const GEN_FLT x40 = x39 + pow(x32, 2);
	const GEN_FLT x41 = pow(x40, -1);
	const GEN_FLT x42 = x41 * x32;
	const GEN_FLT x43 = -x19 + x20;
	const GEN_FLT x44 = x1 + x3;
	const GEN_FLT x45 = x35 + x36;
	const GEN_FLT x46 = lh_py + x25 * (1 - x6 * x44) + x43 * x34 + x45 * x31;
	const GEN_FLT x47 = 0.523598775598299 + tilt_0;
	const GEN_FLT x48 = cos(x47);
	const GEN_FLT x49 = pow(x48, -1);
	const GEN_FLT x50 = pow(x46, 2);
	const GEN_FLT x51 = x40 + x50;
	const GEN_FLT x52 = pow(x51, -1.0 / 2.0);
	const GEN_FLT x53 = x52 * x49;
	const GEN_FLT x54 = asin(x53 * x46);
	const GEN_FLT x55 = 8.0108022e-06 * x54;
	const GEN_FLT x56 = -8.0108022e-06 - x55;
	const GEN_FLT x57 = 0.0028679863 + x54 * x56;
	const GEN_FLT x58 = 5.3685255e-06 + x54 * x57;
	const GEN_FLT x59 = 0.0076069798 + x54 * x58;
	const GEN_FLT x60 = x54 * x59;
	const GEN_FLT x61 = -8.0108022e-06 - 1.60216044e-05 * x54;
	const GEN_FLT x62 = x57 + x61 * x54;
	const GEN_FLT x63 = x58 + x62 * x54;
	const GEN_FLT x64 = x59 + x63 * x54;
	const GEN_FLT x65 = x60 + x64 * x54;
	const GEN_FLT x66 = sin(x47);
	const GEN_FLT x67 = tan(x47);
	const GEN_FLT x68 = pow(x40, -1.0 / 2.0);
	const GEN_FLT x69 = x67 * x68;
	const GEN_FLT x70 = x69 * x46;
	const GEN_FLT x71 = atan2(-x32, x38);
	const GEN_FLT x72 = ogeeMag_0 + x71 - asin(x70);
	const GEN_FLT x73 = curve_0 + sin(x72) * ogeePhase_0;
	const GEN_FLT x74 = x73 * x66;
	const GEN_FLT x75 = x48 - x74 * x65;
	const GEN_FLT x76 = pow(x75, -1);
	const GEN_FLT x77 = pow(x54, 2);
	const GEN_FLT x78 = x73 * x77;
	const GEN_FLT x79 = x78 * x76;
	const GEN_FLT x80 = x70 + x79 * x59;
	const GEN_FLT x81 = pow(1 - pow(x80, 2), -1.0 / 2.0);
	const GEN_FLT x82 = x46 / pow(x40, 3.0 / 2.0);
	const GEN_FLT x83 = x82 * x38;
	const GEN_FLT x84 = x83 * x67;
	const GEN_FLT x85 = x50 / x51;
	const GEN_FLT x86 = pow(1 - x85 / pow(x48, 2), -1.0 / 2.0);
	const GEN_FLT x87 = pow(x51, -3.0 / 2.0);
	const GEN_FLT x88 = x87 * x46;
	const GEN_FLT x89 = x88 * x38;
	const GEN_FLT x90 = x89 * x49;
	const GEN_FLT x91 = x86 * x90;
	const GEN_FLT x92 = -x56 * x91;
	const GEN_FLT x93 = x86 * x57;
	const GEN_FLT x94 = x54 * (x92 + x55 * x91) - x93 * x90;
	const GEN_FLT x95 = x54 * x94 - x58 * x91;
	const GEN_FLT x96 = x50 * x41;
	const GEN_FLT x97 = pow(1 - pow(x67, 2) * x96, -1.0 / 2.0);
	const GEN_FLT x98 = x42 + x84 * x97;
	const GEN_FLT x99 = cos(x72) * ogeePhase_0;
	const GEN_FLT x100 = x65 * x66;
	const GEN_FLT x101 = x99 * x100;
	const GEN_FLT x102 = 2.40324066e-05 * x54;
	const GEN_FLT x103 = x86 * x63;
	const GEN_FLT x104 = x86 * x64;
	const GEN_FLT x105 = x86 * x59;
	const GEN_FLT x106 = x78 * x59 / pow(x75, 2);
	const GEN_FLT x107 = x77 * x76 * x59;
	const GEN_FLT x108 = x99 * x107;
	const GEN_FLT x109 = 2 * x38;
	const GEN_FLT x110 = x88 * x109;
	const GEN_FLT x111 = x73 * x76 * x60;
	const GEN_FLT x112 = x86 * x49 * x111;
	const GEN_FLT x113 =
		x81 *
		(-x84 -
		 x106 * (-x74 * (x54 * x95 +
						 x54 * (x95 + x54 * (x94 + x54 * (x92 - x61 * x91 + x91 * x102) - x62 * x91) - x90 * x103) -
						 x90 * x104 - x90 * x105) -
				 x98 * x101) -
		 x110 * x112 + x79 * x95 + x98 * x108);
	const GEN_FLT x114 = -x42;
	const GEN_FLT x115 = -x71;
	const GEN_FLT x116 = cos(-gibPhase_0 + x115 + asin(x80)) * gibMag_0;
	const GEN_FLT x117 = x87 * x50;
	const GEN_FLT x118 = x53 - x49 * x117;
	const GEN_FLT x119 = x86 * x118;
	const GEN_FLT x120 = x56 * x119;
	const GEN_FLT x121 = x54 * (x120 - x55 * x119) + x93 * x118;
	const GEN_FLT x122 = x54 * x121 + x58 * x119;
	const GEN_FLT x123 = x69 * x97;
	const GEN_FLT x124 = 2 * x111;
	const GEN_FLT x125 =
		x81 *
		(x69 -
		 x106 * (x101 * x123 - x74 * (x104 * x118 + x105 * x118 + x54 * x122 +
									  x54 * (x122 + x103 * x118 +
											 x54 * (x121 + x54 * (x120 - x102 * x119 + x61 * x119) + x62 * x119)))) -
		 x108 * x123 + x119 * x124 + x79 * x122);
	const GEN_FLT x126 = x41 * x38;
	const GEN_FLT x127 = -x126;
	const GEN_FLT x128 = x88 * x32;
	const GEN_FLT x129 = x49 * x128;
	const GEN_FLT x130 = x86 * x129;
	const GEN_FLT x131 = -x56 * x130;
	const GEN_FLT x132 = x54 * (x131 + x55 * x130) - x93 * x129;
	const GEN_FLT x133 = x54 * x132 - x58 * x130;
	const GEN_FLT x134 = x82 * x32;
	const GEN_FLT x135 = x67 * x134;
	const GEN_FLT x136 = x127 + x97 * x135;
	const GEN_FLT x137 = 2 * x32;
	const GEN_FLT x138 = x88 * x137;
	const GEN_FLT x139 =
		x81 *
		(-x135 -
		 x106 * (-x101 * x136 - x74 * (-x104 * x129 - x105 * x129 + x54 * x133 +
									   x54 * (x133 - x103 * x129 +
											  x54 * (x132 + x54 * (x131 + x102 * x130 - x61 * x130) - x62 * x130)))) +
		 x108 * x136 - x112 * x138 + x79 * x133);
	const GEN_FLT x140 = 2 / x5;
	const GEN_FLT x141 = x4 * x140;
	const GEN_FLT x142 = x30 * x141;
	const GEN_FLT x143 = x140 * lh_qw;
	const GEN_FLT x144 = x25 * x143;
	const GEN_FLT x145 = x26 * lh_qk;
	const GEN_FLT x146 = x33 * x18;
	const GEN_FLT x147 = x34 * lh_qj;
	const GEN_FLT x148 = -x145 + x147 - x142 * lh_qw + x143 * x146 + x37 * x144;
	const GEN_FLT x149 = x32 / x39;
	const GEN_FLT x150 = pow(x38, -1);
	const GEN_FLT x151 = x30 * x29;
	const GEN_FLT x152 = x26 * lh_qi;
	const GEN_FLT x153 = x31 * lh_qj;
	const GEN_FLT x154 = x2 * x140;
	const GEN_FLT x155 = x152 - x153 + x143 * x151 + x21 * x144 - x18 * x154 * lh_qw;
	const GEN_FLT x156 = x41 * x39;
	const GEN_FLT x157 = (x148 * x149 - x150 * x155) * x156;
	const GEN_FLT x158 = x45 * x30;
	const GEN_FLT x159 = x31 * lh_qk;
	const GEN_FLT x160 = x44 * x140;
	const GEN_FLT x161 = x25 * x160;
	const GEN_FLT x162 = x43 * x18;
	const GEN_FLT x163 = x34 * lh_qi;
	const GEN_FLT x164 = x159 - x163 + x143 * x158 - x161 * lh_qw + x162 * x143;
	const GEN_FLT x165 = 2 * x46;
	const GEN_FLT x166 = x109 * x148 + x137 * x155;
	const GEN_FLT x167 = (1.0 / 2.0) * x88;
	const GEN_FLT x168 = x167 * (x166 + x165 * x164);
	const GEN_FLT x169 = -x49 * x168 + x53 * x164;
	const GEN_FLT x170 = x86 * x169;
	const GEN_FLT x171 = x56 * x170;
	const GEN_FLT x172 = x54 * (x171 - x55 * x170) + x93 * x169;
	const GEN_FLT x173 = x54 * x172 + x58 * x170;
	const GEN_FLT x174 = (1.0 / 2.0) * x82;
	const GEN_FLT x175 = x166 * x174;
	const GEN_FLT x176 = -x67 * x175 + x69 * x164;
	const GEN_FLT x177 = x157 - x97 * x176;
	const GEN_FLT x178 =
		x81 *
		(x176 -
		 x106 * (-x101 * x177 - x74 * (x104 * x169 + x105 * x169 + x54 * x173 +
									   x54 * (x173 + x103 * x169 +
											  x54 * (x172 + x54 * (x171 - x102 * x170 + x61 * x170) + x62 * x170)))) +
		 x108 * x177 + x124 * x170 + x79 * x173);
	const GEN_FLT x179 = -x157;
	const GEN_FLT x180 = x140 * lh_qi;
	const GEN_FLT x181 = x37 * x25;
	const GEN_FLT x182 = x26 * lh_qj;
	const GEN_FLT x183 = x34 * lh_qk;
	const GEN_FLT x184 = x182 + x183 - x142 * lh_qi + x180 * x146 + x181 * x180;
	const GEN_FLT x185 = x26 * lh_qw;
	const GEN_FLT x186 = x25 * x21;
	const GEN_FLT x187 = 4 * x5;
	const GEN_FLT x188 = -x187 * lh_qi;
	const GEN_FLT x189 = x159 + x185 + x18 * (x188 - x154 * lh_qi) + x180 * x151 + x180 * x186;
	const GEN_FLT x190 = (x184 * x149 - x189 * x150) * x156;
	const GEN_FLT x191 = x34 * lh_qw;
	const GEN_FLT x192 = x153 - x191 + x162 * x180 + x180 * x158 + x25 * (x188 - x160 * lh_qi);
	const GEN_FLT x193 = x109 * x184 + x189 * x137;
	const GEN_FLT x194 = x167 * (x193 + x165 * x192);
	const GEN_FLT x195 = (-x49 * x194 + x53 * x192) * x86;
	const GEN_FLT x196 = x56 * x195;
	const GEN_FLT x197 = x54 * (x196 - x55 * x195) + x57 * x195;
	const GEN_FLT x198 = x54 * x197 + x58 * x195;
	const GEN_FLT x199 = x174 * x193;
	const GEN_FLT x200 = -x67 * x199 + x69 * x192;
	const GEN_FLT x201 = x99 * (x190 - x97 * x200);
	const GEN_FLT x202 =
		x81 *
		(x200 -
		 x106 *
			 (-x201 * x100 -
			  x74 * (x54 * x198 +
					 x54 * (x198 + x54 * (x197 + x54 * (x196 - x102 * x195 + x61 * x195) + x62 * x195) + x63 * x195) +
					 x59 * x195 + x64 * x195)) +
		 x124 * x195 + x201 * x107 + x79 * x198);
	const GEN_FLT x203 = -x190;
	const GEN_FLT x204 = -x187 * lh_qj;
	const GEN_FLT x205 = x140 * lh_qj;
	const GEN_FLT x206 = x152 + x191 + x205 * x146 + x205 * x181 + x30 * (x204 - x141 * lh_qj);
	const GEN_FLT x207 = x140 * x151;
	const GEN_FLT x208 = x31 * lh_qw;
	const GEN_FLT x209 = x145 - x208 + x18 * (x204 - x154 * lh_qj) + x205 * x186 + x207 * lh_qj;
	const GEN_FLT x210 = (x206 * x149 - x209 * x150) * x156;
	const GEN_FLT x211 = x31 * lh_qi;
	const GEN_FLT x212 = x162 * x140;
	const GEN_FLT x213 = x183 + x211 - x161 * lh_qj + x205 * x158 + x212 * lh_qj;
	const GEN_FLT x214 = x206 * x109 + x209 * x137;
	const GEN_FLT x215 = x167 * (x214 + x213 * x165);
	const GEN_FLT x216 = -x49 * x215 + x53 * x213;
	const GEN_FLT x217 = x86 * x216;
	const GEN_FLT x218 = x56 * x217;
	const GEN_FLT x219 = x54 * (x218 - x55 * x217) + x93 * x216;
	const GEN_FLT x220 = x54 * x219 + x58 * x217;
	const GEN_FLT x221 = x214 * x174;
	const GEN_FLT x222 = -x67 * x221 + x69 * x213;
	const GEN_FLT x223 = x210 - x97 * x222;
	const GEN_FLT x224 =
		x81 *
		(x222 -
		 x106 * (-x223 * x101 - x74 * (x216 * x104 + x54 * x220 +
									   x54 * (x220 + x216 * x103 +
											  x54 * (x219 + x54 * (x218 - x217 * x102 + x61 * x217) + x62 * x217)) +
									   x59 * x217)) +
		 x217 * x124 + x223 * x108 + x79 * x220);
	const GEN_FLT x225 = -x210;
	const GEN_FLT x226 = -x187 * lh_qk;
	const GEN_FLT x227 = x140 * lh_qk;
	const GEN_FLT x228 = x18 * lh_qk;
	const GEN_FLT x229 = x163 - x185 + x227 * x181 + x30 * (x226 - x141 * lh_qk) + x33 * x228 * x140;
	const GEN_FLT x230 = x182 + x211 + x207 * lh_qk + x227 * x186 - x228 * x154;
	const GEN_FLT x231 = (x229 * x149 - x230 * x150) * x156;
	const GEN_FLT x232 = x147 + x208 + x212 * lh_qk + x227 * x158 + x25 * (x226 - x160 * lh_qk);
	const GEN_FLT x233 = x229 * x109 + x230 * x137;
	const GEN_FLT x234 = x167 * (x233 + x232 * x165);
	const GEN_FLT x235 = -x49 * x234 + x53 * x232;
	const GEN_FLT x236 = x86 * x235;
	const GEN_FLT x237 = x56 * x236;
	const GEN_FLT x238 = x54 * (x237 - x55 * x236) + x93 * x235;
	const GEN_FLT x239 = x54 * x238 + x58 * x236;
	const GEN_FLT x240 = x233 * x174;
	const GEN_FLT x241 = -x67 * x240 + x69 * x232;
	const GEN_FLT x242 = x231 - x97 * x241;
	const GEN_FLT x243 =
		x81 *
		(x241 -
		 x106 * (-x242 * x101 - x74 * (x54 * x239 +
									   x54 * (x239 + x235 * x103 +
											  x54 * (x238 + x54 * (x237 - x236 * x102 + x61 * x236) + x62 * x236)) +
									   x59 * x236 + x64 * x236)) +
		 x236 * x124 + x242 * x108 + x79 * x239);
	const GEN_FLT x244 = -x231;
	const GEN_FLT x245 = 0.523598775598299 - tilt_1;
	const GEN_FLT x246 = cos(x245);
	const GEN_FLT x247 = pow(x246, -1);
	const GEN_FLT x248 = x52 * x247;
	const GEN_FLT x249 = asin(x46 * x248);
	const GEN_FLT x250 = 8.0108022e-06 * x249;
	const GEN_FLT x251 = -8.0108022e-06 - x250;
	const GEN_FLT x252 = 0.0028679863 + x251 * x249;
	const GEN_FLT x253 = 5.3685255e-06 + x252 * x249;
	const GEN_FLT x254 = 0.0076069798 + x253 * x249;
	const GEN_FLT x255 = x254 * x249;
	const GEN_FLT x256 = -8.0108022e-06 - 1.60216044e-05 * x249;
	const GEN_FLT x257 = x252 + x256 * x249;
	const GEN_FLT x258 = x253 + x257 * x249;
	const GEN_FLT x259 = x254 + x258 * x249;
	const GEN_FLT x260 = x255 + x259 * x249;
	const GEN_FLT x261 = tan(x245);
	const GEN_FLT x262 = x68 * x261;
	const GEN_FLT x263 = -x46 * x262;
	const GEN_FLT x264 = ogeeMag_1 + x71 - asin(x263);
	const GEN_FLT x265 = curve_1 + sin(x264) * ogeePhase_1;
	const GEN_FLT x266 = sin(x245);
	const GEN_FLT x267 = x266 * x265;
	const GEN_FLT x268 = x246 + x267 * x260;
	const GEN_FLT x269 = pow(x268, -1);
	const GEN_FLT x270 = pow(x249, 2);
	const GEN_FLT x271 = x270 * x265;
	const GEN_FLT x272 = x271 * x269;
	const GEN_FLT x273 = x263 + x272 * x254;
	const GEN_FLT x274 = pow(1 - pow(x273, 2), -1.0 / 2.0);
	const GEN_FLT x275 = x83 * x261;
	const GEN_FLT x276 = pow(1 - x96 * pow(x261, 2), -1.0 / 2.0);
	const GEN_FLT x277 = cos(x264) * ogeePhase_1;
	const GEN_FLT x278 = x277 * (x42 - x276 * x275);
	const GEN_FLT x279 = x270 * x269 * x254;
	const GEN_FLT x280 = pow(1 - x85 / pow(x246, 2), -1.0 / 2.0);
	const GEN_FLT x281 = x269 * x265 * x255;
	const GEN_FLT x282 = x280 * x281 * x247;
	const GEN_FLT x283 = x266 * x260;
	const GEN_FLT x284 = x89 * x247;
	const GEN_FLT x285 = x284 * x280;
	const GEN_FLT x286 = -x285 * x251;
	const GEN_FLT x287 = 2.40324066e-05 * x249;
	const GEN_FLT x288 = x280 * x252;
	const GEN_FLT x289 = x249 * (x286 + x285 * x250) - x288 * x284;
	const GEN_FLT x290 = -x285 * x253 + x289 * x249;
	const GEN_FLT x291 = x271 * x254 / pow(x268, 2);
	const GEN_FLT x292 =
		x274 * (x275 + x272 * x290 + x279 * x278 - x282 * x110 -
				x291 * (x267 * (x249 * (x290 + x249 * (x289 + x249 * (x286 - x285 * x256 + x287 * x285) - x285 * x257) -
										x285 * x258) -
								x285 * x254 - x285 * x259 + x290 * x249) +
						x278 * x283));
	const GEN_FLT x293 = cos(-gibPhase_1 + x115 + asin(x273)) * gibMag_1;
	const GEN_FLT x294 = x279 * x277;
	const GEN_FLT x295 = x276 * x262;
	const GEN_FLT x296 = x248 - x247 * x117;
	const GEN_FLT x297 = x296 * x280;
	const GEN_FLT x298 = 2 * x281;
	const GEN_FLT x299 = x277 * x283;
	const GEN_FLT x300 = x297 * x251;
	const GEN_FLT x301 = x249 * (x300 - x297 * x250) + x296 * x288;
	const GEN_FLT x302 = x249 * x301 + x297 * x253;
	const GEN_FLT x303 =
		x274 * (-x262 + x272 * x302 -
				x291 * (x267 * (x249 * x302 +
								x249 * (x302 + x249 * (x301 + x249 * (x300 + x297 * x256 - x297 * x287) + x297 * x257) +
										x297 * x258) +
								x297 * x254 + x297 * x259) +
						x299 * x295) +
				x295 * x294 + x297 * x298);
	const GEN_FLT x304 = x261 * x134;
	const GEN_FLT x305 = x127 - x276 * x304;
	const GEN_FLT x306 = x247 * x128;
	const GEN_FLT x307 = x280 * x306;
	const GEN_FLT x308 = -x251 * x307;
	const GEN_FLT x309 = x249 * (x308 + x250 * x307) - x288 * x306;
	const GEN_FLT x310 = x249 * x309 - x253 * x307;
	const GEN_FLT x311 =
		x274 * (x304 + x272 * x310 - x282 * x138 -
				x291 * (x267 * (x249 * x310 +
								x249 * (x310 + x249 * (x309 + x249 * (x308 - x256 * x307 + x287 * x307) - x257 * x307) -
										x258 * x307) -
								x254 * x307 - x259 * x307) +
						x299 * x305) +
				x294 * x305);
	const GEN_FLT x312 = x261 * x175 - x262 * x164;
	const GEN_FLT x313 = x157 - x276 * x312;
	const GEN_FLT x314 = -x247 * x168 + x248 * x164;
	const GEN_FLT x315 = x280 * x314;
	const GEN_FLT x316 = x251 * x315;
	const GEN_FLT x317 = x249 * (x316 - x250 * x315) + x288 * x314;
	const GEN_FLT x318 = x249 * x317 + x253 * x315;
	const GEN_FLT x319 =
		x274 * (x312 + x272 * x318 -
				x291 * (x267 * (x249 * x318 +
								x249 * (x318 + x249 * (x317 + x249 * (x316 + x256 * x315 - x287 * x315) + x257 * x315) +
										x258 * x315) +
								x254 * x315 + x259 * x315) +
						x299 * x313) +
				x294 * x313 + x298 * x315);
	const GEN_FLT x320 = x261 * x199 - x262 * x192;
	const GEN_FLT x321 = x190 - x276 * x320;
	const GEN_FLT x322 = -x247 * x194 + x248 * x192;
	const GEN_FLT x323 = x280 * x322;
	const GEN_FLT x324 = x251 * x323;
	const GEN_FLT x325 = x249 * (x324 - x250 * x323) + x288 * x322;
	const GEN_FLT x326 = x249 * x325 + x253 * x323;
	const GEN_FLT x327 =
		x274 * (x320 + x272 * x326 -
				x291 * (x267 * (x249 * x326 +
								x249 * (x326 + x249 * (x325 + x249 * (x324 + x256 * x323 - x287 * x323) + x257 * x323) +
										x258 * x323) +
								x254 * x323 + x259 * x323) +
						x299 * x321) +
				x294 * x321 + x298 * x323);
	const GEN_FLT x328 = -x213 * x262 + x261 * x221;
	const GEN_FLT x329 = x210 - x276 * x328;
	const GEN_FLT x330 = (x213 * x248 - x215 * x247) * x280;
	const GEN_FLT x331 = x251 * x330;
	const GEN_FLT x332 = x249 * (x331 - x250 * x330) + x252 * x330;
	const GEN_FLT x333 = x249 * x332 + x253 * x330;
	const GEN_FLT x334 =
		x274 * (x328 + x272 * x333 -
				x291 * (x267 * (x249 * x333 +
								x249 * (x333 + x249 * (x332 + x249 * (x331 + x256 * x330 - x287 * x330) + x257 * x330) +
										x258 * x330) +
								x254 * x330 + x259 * x330) +
						x299 * x329) +
				x294 * x329 + x298 * x330);
	const GEN_FLT x335 = -x232 * x262 + x261 * x240;
	const GEN_FLT x336 = x231 - x276 * x335;
	const GEN_FLT x337 = (x232 * x248 - x234 * x247) * x280;
	const GEN_FLT x338 = x251 * x337;
	const GEN_FLT x339 = x249 * (x338 - x250 * x337) + x252 * x337;
	const GEN_FLT x340 = x249 * x339 + x253 * x337;
	const GEN_FLT x341 =
		x274 * (x335 + x272 * x340 -
				x291 * (x267 * (x249 * x340 +
								x249 * (x340 + x249 * (x339 + x249 * (x338 + x256 * x337 - x287 * x337) + x257 * x337) +
										x258 * x337) +
								x254 * x337 + x259 * x337) +
						x299 * x336) +
				x294 * x336 + x298 * x337);
	*(out++) = -x113 + x42 - (x113 + x114) * x116;
	*(out++) = -x125 - x116 * x125;
	*(out++) = x127 - x139 - (x126 + x139) * x116;
	*(out++) = x157 - x178 - (x178 + x179) * x116;
	*(out++) = x190 - x202 - (x202 + x203) * x116;
	*(out++) = x210 - x224 - (x224 + x225) * x116;
	*(out++) = x231 - x243 - (x243 + x244) * x116;
	*(out++) = -x292 + x42 - (x114 + x292) * x293;
	*(out++) = -x303 - x293 * x303;
	*(out++) = x127 - x311 - (x126 + x311) * x293;
	*(out++) = x157 - x319 - (x179 + x319) * x293;
	*(out++) = x190 - x327 - (x203 + x327) * x293;
	*(out++) = x210 - x334 - (x225 + x334) * x293;
	*(out++) = x231 - x341 - (x244 + x341) * x293;
}

// Jacobian of reproject_gen2 wrt [phase_0, tilt_0, curve_0, gibPhase_0, gibMag_0, ogeeMag_0, ogeePhase_0, phase_1,
// tilt_1, curve_1, gibPhase_1, gibMag_1, ogeeMag_1, ogeePhase_1]
static inline void gen_reproject_gen2_jac_bsd(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
											  const SurvivePose *lh_p, const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = 0.523598775598299 + tilt_0;
	const GEN_FLT x1 = tan(x0);
	const GEN_FLT x2 = lh_qw * lh_qi;
	const GEN_FLT x3 = lh_qk * lh_qj;
	const GEN_FLT x4 = pow(obj_qj, 2);
	const GEN_FLT x5 = pow(obj_qi, 2);
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = 2 * sqrt(x6 + x7 + pow(obj_qw, 2));
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = obj_pz + x11 * (x10 + x9) + (1 - x6 * x8) * sensor_z + (-x12 + x13) * x14;
	const GEN_FLT x16 = pow(lh_qi, 2);
	const GEN_FLT x17 = pow(lh_qk, 2);
	const GEN_FLT x18 = pow(lh_qj, 2);
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 2 * sqrt(x16 + x19 + pow(lh_qw, 2));
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + x22 * (x10 - x9) + (1 - (x5 + x7) * x8) * sensor_y + (x23 + x24) * x14;
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 = obj_px + (1 - (x4 + x7) * x8) * sensor_x + (x12 + x13) * x22 + (-x23 + x24) * x11;
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = lh_py + x25 * (1 - (x16 + x17) * x20) + (-x2 + x3) * x21 + (x26 + x27) * x29;
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = lh_pz + x15 * (1 - (x16 + x18) * x20) + (x2 + x3) * x31 + (-x32 + x33) * x29;
	const GEN_FLT x35 = lh_px + x28 * (1 - x20 * x19) + (-x26 + x27) * x31 + (x32 + x33) * x21;
	const GEN_FLT x36 = pow(x34, 2) + pow(x35, 2);
	const GEN_FLT x37 = x30 / sqrt(x36);
	const GEN_FLT x38 = x1 * x37;
	const GEN_FLT x39 = atan2(-x34, x35);
	const GEN_FLT x40 = ogeeMag_0 + x39 - asin(x38);
	const GEN_FLT x41 = sin(x40);
	const GEN_FLT x42 = curve_0 + x41 * ogeePhase_0;
	const GEN_FLT x43 = cos(x0);
	const GEN_FLT x44 = pow(x30, 2);
	const GEN_FLT x45 = x36 + x44;
	const GEN_FLT x46 = x30 / sqrt(x45);
	const GEN_FLT x47 = asin(x46 / x43);
	const GEN_FLT x48 = 8.0108022e-06 * x47;
	const GEN_FLT x49 = -8.0108022e-06 - x48;
	const GEN_FLT x50 = 0.0028679863 + x47 * x49;
	const GEN_FLT x51 = 5.3685255e-06 + x50 * x47;
	const GEN_FLT x52 = 0.0076069798 + x51 * x47;
	const GEN_FLT x53 = sin(x0);
	const GEN_FLT x54 = x52 * x47;
	const GEN_FLT x55 = -8.0108022e-06 - 1.60216044e-05 * x47;
	const GEN_FLT x56 = x50 + x55 * x47;
	const GEN_FLT x57 = x51 + x56 * x47;
	const GEN_FLT x58 = x52 + x57 * x47;
	const GEN_FLT x59 = x54 + x58 * x47;
	const GEN_FLT x60 = x59 * x42;
	const GEN_FLT x61 = x60 * x53;
	const GEN_FLT x62 = x43 - x61;
	const GEN_FLT x63 = pow(x62, -1);
	const GEN_FLT x64 = pow(x47, 2);
	const GEN_FLT x65 = x63 * x64;
	const GEN_FLT x66 = x65 * x52;
	const GEN_FLT x67 = x38 + x66 * x42;
	const GEN_FLT x68 = pow(1 - pow(x67, 2), -1.0 / 2.0);
	const GEN_FLT x69 = pow(x1, 2);
	const GEN_FLT x70 = x37 * (1 + x69);
	const GEN_FLT x71 = pow(x43, -2);
	const GEN_FLT x72 = x44 / x45;
	const GEN_FLT x73 = x71 * x46 / sqrt(1 - x71 * x72);
	const GEN_FLT x74 = x73 * x53;
	const GEN_FLT x75 = x74 * x49;
	const GEN_FLT x76 = x47 * (x75 - x74 * x48) + x74 * x50;
	const GEN_FLT x77 = x74 * x51 + x76 * x47;
	const GEN_FLT x78 = cos(x40) * ogeePhase_0;
	const GEN_FLT x79 = x44 / x36;
	const GEN_FLT x80 = x70 / sqrt(1 - x79 * x69);
	const GEN_FLT x81 = x53 * x42;
	const GEN_FLT x82 = x64 * x52 / pow(x62, 2);
	const GEN_FLT x83 = x78 * x66;
	const GEN_FLT x84 =
		x68 * (x70 - x80 * x83 + x77 * x65 * x42 -
			   x82 * x42 *
				   (-x53 - x60 * x43 -
					x81 * (x47 * (x77 + x47 * (x76 + x47 * (x75 - 2.40324066e-05 * x74 * x47 + x74 * x55) + x74 * x56) +
								  x74 * x57) +
						   x74 * x52 + x74 * x58 + x77 * x47) +
					x80 * x78 * x53 * x59) +
			   2 * x81 * x73 * x63 * x54);
	const GEN_FLT x85 = -x39;
	const GEN_FLT x86 = -gibPhase_0 + x85 + asin(x67);
	const GEN_FLT x87 = cos(x86) * gibMag_0;
	const GEN_FLT x88 = x82 * x61;
	const GEN_FLT x89 = (x66 + x88) * x68;
	const GEN_FLT x90 = x68 * (x83 + x88 * x78);
	const GEN_FLT x91 = (x66 * x41 + x88 * x41) * x68;
	const GEN_FLT x92 = 0.523598775598299 - tilt_1;
	const GEN_FLT x93 = tan(x92);
	const GEN_FLT x94 = -x93 * x37;
	const GEN_FLT x95 = ogeeMag_1 + x39 - asin(x94);
	const GEN_FLT x96 = sin(x95);
	const GEN_FLT x97 = curve_1 + x96 * ogeePhase_1;
	const GEN_FLT x98 = cos(x92);
	const GEN_FLT x99 = asin(x46 / x98);
	const GEN_FLT x100 = 8.0108022e-06 * x99;
	const GEN_FLT x101 = -8.0108022e-06 - x100;
	const GEN_FLT x102 = 0.0028679863 + x99 * x101;
	const GEN_FLT x103 = 5.3685255e-06 + x99 * x102;
	const GEN_FLT x104 = 0.0076069798 + x99 * x103;
	const GEN_FLT x105 = pow(x99, 2);
	const GEN_FLT x106 = x99 * x104;
	const GEN_FLT x107 = -8.0108022e-06 - 1.60216044e-05 * x99;
	const GEN_FLT x108 = x102 + x99 * x107;
	const GEN_FLT x109 = x103 + x99 * x108;
	const GEN_FLT x110 = x104 + x99 * x109;
	const GEN_FLT x111 = x106 + x99 * x110;
	const GEN_FLT x112 = sin(x92);
	const GEN_FLT x113 = x97 * x112;
	const GEN_FLT x114 = x111 * x113;
	const GEN_FLT x115 = x114 + x98;
	const GEN_FLT x116 = pow(x115, -1);
	const GEN_FLT x117 = x105 * x116;
	const GEN_FLT x118 = x104 * x117;
	const GEN_FLT x119 = x94 + x97 * x118;
	const GEN_FLT x120 = pow(1 - pow(x119, 2), -1.0 / 2.0);
	const GEN_FLT x121 = pow(x93, 2);
	const GEN_FLT x122 = x37 * (1 + x121);
	const GEN_FLT x123 = cos(x95) * ogeePhase_1;
	const GEN_FLT x124 = x118 * x123;
	const GEN_FLT x125 = x122 / sqrt(1 - x79 * x121);
	const GEN_FLT x126 = pow(x98, -2);
	const GEN_FLT x127 = x46 * x126 / sqrt(1 - x72 * x126);
	const GEN_FLT x128 = x112 * x127;
	const GEN_FLT x129 = -x101 * x128;
	const GEN_FLT x130 = -x102 * x128 + x99 * (x129 + x100 * x128);
	const GEN_FLT x131 = -x103 * x128 + x99 * x130;
	const GEN_FLT x132 = x105 * x104 / pow(x115, 2);
	const GEN_FLT x133 =
		x120 *
		(x122 - x124 * x125 + x97 * x117 * x131 -
		 x97 * x132 *
			 (x112 +
			  x113 * (-x104 * x128 - x110 * x128 + x99 * x131 +
					  x99 * (x131 - x109 * x128 +
							 x99 * (x130 - x108 * x128 + x99 * (x129 - x107 * x128 + 2.40324066e-05 * x99 * x128)))) -
			  x98 * x97 * x111 - x111 * x112 * x123 * x125) -
		 2 * x106 * x113 * x116 * x127);
	const GEN_FLT x134 = -gibPhase_1 + x85 + asin(x119);
	const GEN_FLT x135 = cos(x134) * gibMag_1;
	const GEN_FLT x136 = x114 * x132;
	const GEN_FLT x137 = (x118 - x136) * x120;
	const GEN_FLT x138 = x120 * (x124 - x123 * x136);
	const GEN_FLT x139 = (x96 * x118 - x96 * x136) * x120;
	*(out++) = -1;
	*(out++) = -x84 - x84 * x87;
	*(out++) = -x89 - x89 * x87;
	*(out++) = x87;
	*(out++) = -sin(x86);
	*(out++) = -x90 - x87 * x90;
	*(out++) = -x91 - x87 * x91;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = -1;
	*(out++) = -x133 - x133 * x135;
	*(out++) = -x137 - x137 * x135;
	*(out++) = x135;
	*(out++) = -sin(x134);
	*(out++) = -x138 - x138 * x135;
	*(out++) = -x139 - x135 * x139;
}

/** Applying function <function reproject_axis_x_gen2 at 0x7ffa3c1c79e0> */
static inline void gen_reproject_axis_x_gen2(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
											 const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = 0.523598775598299 + tilt_0;
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = lh_qw * lh_qi;
	const GEN_FLT x3 = lh_qk * lh_qj;
	const GEN_FLT x4 = pow(obj_qj, 2);
	const GEN_FLT x5 = pow(obj_qi, 2);
	const GEN_FLT x6 = pow(obj_qk, 2);
	const GEN_FLT x7 = x5 + x6;
	const GEN_FLT x8 = 2 * sqrt(x4 + x7 + pow(obj_qw, 2));
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = obj_pz + x11 * (x10 + x9) + (1 - (x4 + x5) * x8) * sensor_z + (-x12 + x13) * x14;
	const GEN_FLT x16 = pow(lh_qi, 2);
	const GEN_FLT x17 = pow(lh_qk, 2);
	const GEN_FLT x18 = pow(lh_qj, 2);
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 2 * sqrt(x16 + x19 + pow(lh_qw, 2));
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + x22 * (x10 - x9) + (1 - x8 * x7) * sensor_y + (x23 + x24) * x14;
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 = obj_px + (1 - (x4 + x6) * x8) * sensor_x + (x12 + x13) * x22 + (-x23 + x24) * x11;
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = lh_py + x25 * (1 - (x16 + x17) * x20) + (-x2 + x3) * x21 + (x26 + x27) * x29;
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = lh_pz + x15 * (1 - (x16 + x18) * x20) + (x2 + x3) * x31 + (-x32 + x33) * x29;
	const GEN_FLT x35 = lh_px + x28 * (1 - x20 * x19) + (-x26 + x27) * x31 + (x32 + x33) * x21;
	const GEN_FLT x36 = pow(x34, 2) + pow(x35, 2);
	const GEN_FLT x37 = asin(x30 / (x1 * sqrt(x36 + pow(x30, 2))));
	const GEN_FLT x38 = 0.0028679863 + x37 * (-8.0108022e-06 - 8.0108022e-06 * x37);
	const GEN_FLT x39 = 5.3685255e-06 + x38 * x37;
	const GEN_FLT x40 = 0.0076069798 + x37 * x39;
	const GEN_FLT x41 = tan(x0) * x30 / sqrt(x36);
	const GEN_FLT x42 = atan2(-x34, x35);
	const GEN_FLT x43 = curve_0 + sin(ogeeMag_0 + x42 - asin(x41)) * ogeePhase_0;
	const GEN_FLT x44 = asin(
		x41 + x40 * x43 * pow(x37, 2) /
				  (x1 - sin(x0) * x43 *
							(x37 * (x40 + x37 * (x39 + x37 * (x38 + x37 * (-8.0108022e-06 - 1.60216044e-05 * x37)))) +
							 x40 * x37)));
	*(out++) = -1.5707963267949 - phase_0 + x42 - x44 + sin(gibPhase_0 + x42 - x44) * gibMag_0;
}

// Jacobian of reproject_axis_x_gen2 wrt [obj_px, obj_py, obj_pz, obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_reproject_axis_x_gen2_jac_obj_p(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
													   const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = pow(obj_qj, 2);
	const GEN_FLT x1 = pow(obj_qi, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(obj_qk, 2);
	const GEN_FLT x4 = x1 + x3;
	const GEN_FLT x5 = sqrt(x0 + x4 + pow(obj_qw, 2));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x7 + x8;
	const GEN_FLT x10 = x6 * sensor_y;
	const GEN_FLT x11 = obj_qw * obj_qj;
	const GEN_FLT x12 = obj_qk * obj_qi;
	const GEN_FLT x13 = -x11 + x12;
	const GEN_FLT x14 = x6 * sensor_x;
	const GEN_FLT x15 = obj_pz + x14 * x13 + x9 * x10 + (1 - x2 * x6) * sensor_z;
	const GEN_FLT x16 = lh_qw * lh_qi;
	const GEN_FLT x17 = lh_qk * lh_qj;
	const GEN_FLT x18 = -x16 + x17;
	const GEN_FLT x19 = pow(lh_qj, 2);
	const GEN_FLT x20 = pow(lh_qk, 2);
	const GEN_FLT x21 = pow(lh_qi, 2);
	const GEN_FLT x22 = x20 + x21;
	const GEN_FLT x23 = sqrt(x19 + x22 + pow(lh_qw, 2));
	const GEN_FLT x24 = 2 * x23;
	const GEN_FLT x25 = x24 * x18;
	const GEN_FLT x26 = 1 - x24 * x22;
	const GEN_FLT x27 = -x7 + x8;
	const GEN_FLT x28 = x6 * sensor_z;
	const GEN_FLT x29 = obj_qw * obj_qk;
	const GEN_FLT x30 = obj_qj * obj_qi;
	const GEN_FLT x31 = x29 + x30;
	const GEN_FLT x32 = obj_py + x28 * x27 + x31 * x14 + (1 - x4 * x6) * sensor_y;
	const GEN_FLT x33 = x11 + x12;
	const GEN_FLT x34 = -x29 + x30;
	const GEN_FLT x35 = x0 + x3;
	const GEN_FLT x36 = obj_px + x33 * x28 + x34 * x10 + (1 - x6 * x35) * sensor_x;
	const GEN_FLT x37 = lh_qw * lh_qk;
	const GEN_FLT x38 = lh_qj * lh_qi;
	const GEN_FLT x39 = x37 + x38;
	const GEN_FLT x40 = x39 * x24;
	const GEN_FLT x41 = lh_py + x25 * x15 + x32 * x26 + x40 * x36;
	const GEN_FLT x42 = 0.523598775598299 + tilt_0;
	const GEN_FLT x43 = cos(x42);
	const GEN_FLT x44 = pow(x43, -1);
	const GEN_FLT x45 = pow(x41, 2);
	const GEN_FLT x46 = 1 - (x19 + x21) * x24;
	const GEN_FLT x47 = x16 + x17;
	const GEN_FLT x48 = x47 * x24;
	const GEN_FLT x49 = lh_qw * lh_qj;
	const GEN_FLT x50 = lh_qk * lh_qi;
	const GEN_FLT x51 = -x49 + x50;
	const GEN_FLT x52 = x51 * x24;
	const GEN_FLT x53 = lh_pz + x46 * x15 + x48 * x32 + x52 * x36;
	const GEN_FLT x54 = x49 + x50;
	const GEN_FLT x55 = x54 * x24;
	const GEN_FLT x56 = -x37 + x38;
	const GEN_FLT x57 = x56 * x24;
	const GEN_FLT x58 = 1 - (x19 + x20) * x24;
	const GEN_FLT x59 = lh_px + x55 * x15 + x57 * x32 + x58 * x36;
	const GEN_FLT x60 = pow(x59, 2);
	const GEN_FLT x61 = x60 + pow(x53, 2);
	const GEN_FLT x62 = x45 + x61;
	const GEN_FLT x63 = x44 / sqrt(x62);
	const GEN_FLT x64 = asin(x63 * x41);
	const GEN_FLT x65 = 8.0108022e-06 * x64;
	const GEN_FLT x66 = -8.0108022e-06 - x65;
	const GEN_FLT x67 = 0.0028679863 + x64 * x66;
	const GEN_FLT x68 = 5.3685255e-06 + x64 * x67;
	const GEN_FLT x69 = 0.0076069798 + x64 * x68;
	const GEN_FLT x70 = x64 * x69;
	const GEN_FLT x71 = -8.0108022e-06 - 1.60216044e-05 * x64;
	const GEN_FLT x72 = x67 + x71 * x64;
	const GEN_FLT x73 = x68 + x72 * x64;
	const GEN_FLT x74 = x69 + x73 * x64;
	const GEN_FLT x75 = x70 + x74 * x64;
	const GEN_FLT x76 = sin(x42);
	const GEN_FLT x77 = tan(x42);
	const GEN_FLT x78 = x77 / sqrt(x61);
	const GEN_FLT x79 = x78 * x41;
	const GEN_FLT x80 = atan2(-x53, x59);
	const GEN_FLT x81 = ogeeMag_0 + x80 - asin(x79);
	const GEN_FLT x82 = curve_0 + sin(x81) * ogeePhase_0;
	const GEN_FLT x83 = x82 * x76;
	const GEN_FLT x84 = x43 - x83 * x75;
	const GEN_FLT x85 = pow(x84, -1);
	const GEN_FLT x86 = pow(x64, 2);
	const GEN_FLT x87 = x82 * x86;
	const GEN_FLT x88 = x85 * x87;
	const GEN_FLT x89 = x79 + x88 * x69;
	const GEN_FLT x90 = pow(1 - pow(x89, 2), -1.0 / 2.0);
	const GEN_FLT x91 = pow(1 - x45 / (x62 * pow(x43, 2)), -1.0 / 2.0);
	const GEN_FLT x92 = 4 * x23;
	const GEN_FLT x93 = x92 * x41;
	const GEN_FLT x94 = 2 * x59;
	const GEN_FLT x95 = x53 * x92;
	const GEN_FLT x96 = x51 * x95 + x58 * x94;
	const GEN_FLT x97 = (1.0 / 2.0) * x41;
	const GEN_FLT x98 = x97 * x44 / pow(x62, 3.0 / 2.0);
	const GEN_FLT x99 = x63 * x40 - x98 * (x96 + x93 * x39);
	const GEN_FLT x100 = x91 * x99;
	const GEN_FLT x101 = x66 * x100;
	const GEN_FLT x102 = x67 * x91;
	const GEN_FLT x103 = x64 * (x101 - x65 * x100) + x99 * x102;
	const GEN_FLT x104 = x64 * x103 + x68 * x100;
	const GEN_FLT x105 = pow(x60, -1);
	const GEN_FLT x106 = x53 * x105;
	const GEN_FLT x107 = pow(x59, -1);
	const GEN_FLT x108 = pow(x61, -1);
	const GEN_FLT x109 = x60 * x108;
	const GEN_FLT x110 = (-x52 * x107 + x58 * x106) * x109;
	const GEN_FLT x111 = pow(1 - pow(x77, 2) * x45 * x108, -1.0 / 2.0);
	const GEN_FLT x112 = x77 * x97 / pow(x61, 3.0 / 2.0);
	const GEN_FLT x113 = x78 * x40 - x96 * x112;
	const GEN_FLT x114 = x110 - x111 * x113;
	const GEN_FLT x115 = cos(x81) * ogeePhase_0;
	const GEN_FLT x116 = x75 * x76;
	const GEN_FLT x117 = x115 * x116;
	const GEN_FLT x118 = 2.40324066e-05 * x64;
	const GEN_FLT x119 = x87 * x69 / pow(x84, 2);
	const GEN_FLT x120 = x85 * x86 * x69;
	const GEN_FLT x121 = x115 * x120;
	const GEN_FLT x122 = 2 * x82 * x85 * x70;
	const GEN_FLT x123 =
		x90 *
		(x113 + x100 * x122 + x114 * x121 -
		 x119 *
			 (-x114 * x117 -
			  x83 * (x64 * x104 +
					 x64 * (x104 + x64 * (x103 + x64 * (x101 - x100 * x118 + x71 * x100) + x72 * x100) + x73 * x100) +
					 x69 * x100 + x74 * x100)) +
		 x88 * x104);
	const GEN_FLT x124 = cos(gibPhase_0 + x80 - asin(x89)) * gibMag_0;
	const GEN_FLT x125 = 2 * x53;
	const GEN_FLT x126 = x23 * x105 * x125;
	const GEN_FLT x127 = (-x48 * x107 + x56 * x126) * x109;
	const GEN_FLT x128 = 2 * x41;
	const GEN_FLT x129 = x59 * x92;
	const GEN_FLT x130 = x56 * x129 + x95 * x47;
	const GEN_FLT x131 = x63 * x26 - x98 * (x130 + x26 * x128);
	const GEN_FLT x132 = x91 * x131;
	const GEN_FLT x133 = x66 * x132;
	const GEN_FLT x134 = x102 * x131 + x64 * (x133 - x65 * x132);
	const GEN_FLT x135 = x64 * x134 + x68 * x132;
	const GEN_FLT x136 = -x112 * x130 + x78 * x26;
	const GEN_FLT x137 = x115 * (x127 - x111 * x136);
	const GEN_FLT x138 =
		x90 *
		(x136 -
		 x119 *
			 (-x116 * x137 -
			  x83 * (x64 * x135 +
					 x64 * (x135 + x64 * (x134 + x64 * (x133 - x118 * x132 + x71 * x132) + x72 * x132) + x73 * x132) +
					 x69 * x132 + x74 * x132)) +
		 x120 * x137 + x122 * x132 + x88 * x135);
	const GEN_FLT x139 = (-x46 * x107 + x54 * x126) * x109;
	const GEN_FLT x140 = x46 * x125 + x54 * x129;
	const GEN_FLT x141 = x63 * x25 - x98 * (x140 + x93 * x18);
	const GEN_FLT x142 = x91 * x141;
	const GEN_FLT x143 = x66 * x142;
	const GEN_FLT x144 = x102 * x141 + x64 * (x143 - x65 * x142);
	const GEN_FLT x145 = x64 * x144 + x68 * x142;
	const GEN_FLT x146 = -x112 * x140 + x78 * x25;
	const GEN_FLT x147 = x139 - x111 * x146;
	const GEN_FLT x148 =
		x90 *
		(x146 -
		 x119 *
			 (-x117 * x147 -
			  x83 * (x64 * x145 +
					 x64 * (x145 + x64 * (x144 + x64 * (x143 - x118 * x142 + x71 * x142) + x72 * x142) + x73 * x142) +
					 x69 * x142 + x74 * x142)) +
		 x121 * x147 + x122 * x142 + x88 * x145);
	const GEN_FLT x149 = 2 / x5;
	const GEN_FLT x150 = x149 * obj_qw;
	const GEN_FLT x151 = x35 * sensor_x;
	const GEN_FLT x152 = x34 * sensor_y;
	const GEN_FLT x153 = x10 * obj_qk;
	const GEN_FLT x154 = x33 * sensor_z;
	const GEN_FLT x155 = x28 * obj_qj;
	const GEN_FLT x156 = -x153 + x155 - x150 * x151 + x150 * x152 + x150 * x154;
	const GEN_FLT x157 = x31 * sensor_x;
	const GEN_FLT x158 = x4 * sensor_y;
	const GEN_FLT x159 = x14 * obj_qk;
	const GEN_FLT x160 = x27 * sensor_z;
	const GEN_FLT x161 = x28 * obj_qi;
	const GEN_FLT x162 = x159 - x161 + x150 * x157 - x150 * x158 + x160 * x150;
	const GEN_FLT x163 = x14 * obj_qj;
	const GEN_FLT x164 = x13 * sensor_x;
	const GEN_FLT x165 = x9 * sensor_y;
	const GEN_FLT x166 = x10 * obj_qi;
	const GEN_FLT x167 = x2 * sensor_z;
	const GEN_FLT x168 = -x163 + x166 + x164 * x150 + x165 * x150 - x167 * x150;
	const GEN_FLT x169 = x25 * x168 + x26 * x162 + x40 * x156;
	const GEN_FLT x170 = x55 * x168 + x57 * x162 + x58 * x156;
	const GEN_FLT x171 = x46 * x168 + x48 * x162 + x52 * x156;
	const GEN_FLT x172 = x125 * x171 + x94 * x170;
	const GEN_FLT x173 = x63 * x169 - x98 * (x172 + x128 * x169);
	const GEN_FLT x174 = x91 * x173;
	const GEN_FLT x175 = x66 * x174;
	const GEN_FLT x176 = x102 * x173 + x64 * (x175 - x65 * x174);
	const GEN_FLT x177 = x64 * x176 + x68 * x174;
	const GEN_FLT x178 = (x106 * x170 - x107 * x171) * x109;
	const GEN_FLT x179 = -x112 * x172 + x78 * x169;
	const GEN_FLT x180 = x178 - x111 * x179;
	const GEN_FLT x181 =
		x90 *
		(x179 -
		 x119 *
			 (-x117 * x180 -
			  x83 * (x64 * x177 +
					 x64 * (x177 + x64 * (x176 + x64 * (x175 - x118 * x174 + x71 * x174) + x72 * x174) + x73 * x174) +
					 x69 * x174 + x74 * x174)) +
		 x121 * x180 + x122 * x174 + x88 * x177);
	const GEN_FLT x182 = x149 * obj_qi;
	const GEN_FLT x183 = x10 * obj_qj;
	const GEN_FLT x184 = x28 * obj_qk;
	const GEN_FLT x185 = x183 + x184 - x182 * x151 + x182 * x152 + x182 * x154;
	const GEN_FLT x186 = 4 * x5;
	const GEN_FLT x187 = -x186 * obj_qi;
	const GEN_FLT x188 = x28 * obj_qw;
	const GEN_FLT x189 = x163 - x188 + x160 * x182 + x182 * x157 + (x187 - x4 * x182) * sensor_y;
	const GEN_FLT x190 = x10 * obj_qw;
	const GEN_FLT x191 = x159 + x190 + x164 * x182 + x165 * x182 + (x187 - x2 * x182) * sensor_z;
	const GEN_FLT x192 = x55 * x191 + x57 * x189 + x58 * x185;
	const GEN_FLT x193 = x46 * x191 + x48 * x189 + x52 * x185;
	const GEN_FLT x194 = (x106 * x192 - x107 * x193) * x109;
	const GEN_FLT x195 = x25 * x191 + x26 * x189 + x40 * x185;
	const GEN_FLT x196 = x125 * x193 + x94 * x192;
	const GEN_FLT x197 = x91 * (x63 * x195 - x98 * (x196 + x128 * x195));
	const GEN_FLT x198 = x66 * x197;
	const GEN_FLT x199 = x64 * (x198 - x65 * x197) + x67 * x197;
	const GEN_FLT x200 = x64 * x199 + x68 * x197;
	const GEN_FLT x201 = -x112 * x196 + x78 * x195;
	const GEN_FLT x202 = x194 - x201 * x111;
	const GEN_FLT x203 =
		x90 *
		(x201 -
		 x119 *
			 (-x202 * x117 -
			  x83 * (x64 * x200 +
					 x64 * (x200 + x64 * (x199 + x64 * (x198 - x118 * x197 + x71 * x197) + x72 * x197) + x73 * x197) +
					 x69 * x197 + x74 * x197)) +
		 x122 * x197 + x202 * x121 + x88 * x200);
	const GEN_FLT x204 = x149 * obj_qj;
	const GEN_FLT x205 = -x186 * obj_qj;
	const GEN_FLT x206 = x166 + x188 + x204 * x152 + x204 * x154 + (x205 - x35 * x204) * sensor_x;
	const GEN_FLT x207 = x14 * obj_qi;
	const GEN_FLT x208 = x184 + x207 + x204 * x157 - x204 * x158 + x204 * x160;
	const GEN_FLT x209 = x14 * obj_qw;
	const GEN_FLT x210 = x153 - x209 + x204 * x164 + x204 * x165 + (x205 - x2 * x204) * sensor_z;
	const GEN_FLT x211 = x55 * x210 + x57 * x208 + x58 * x206;
	const GEN_FLT x212 = x46 * x210 + x48 * x208 + x52 * x206;
	const GEN_FLT x213 = (x211 * x106 - x212 * x107) * x109;
	const GEN_FLT x214 = x25 * x210 + x26 * x208 + x40 * x206;
	const GEN_FLT x215 = x212 * x125 + x94 * x211;
	const GEN_FLT x216 = x91 * (x63 * x214 - x98 * (x215 + x214 * x128));
	const GEN_FLT x217 = x66 * x216;
	const GEN_FLT x218 = x64 * (x217 - x65 * x216) + x67 * x216;
	const GEN_FLT x219 = x64 * x218 + x68 * x216;
	const GEN_FLT x220 = -x215 * x112 + x78 * x214;
	const GEN_FLT x221 = x213 - x220 * x111;
	const GEN_FLT x222 =
		x90 *
		(x220 -
		 x119 *
			 (-x221 * x117 -
			  x83 * (x64 * x219 +
					 x64 * (x219 + x64 * (x218 + x64 * (x217 - x216 * x118 + x71 * x216) + x72 * x216) + x73 * x216) +
					 x69 * x216 + x74 * x216)) +
		 x216 * x122 + x221 * x121 + x88 * x219);
	const GEN_FLT x223 = x149 * obj_qk;
	const GEN_FLT x224 = -x186 * obj_qk;
	const GEN_FLT x225 = x161 - x190 + x223 * x152 + x223 * x154 + (x224 - x35 * x223) * sensor_x;
	const GEN_FLT x226 = x155 + x209 + x223 * x157 + x223 * x160 + (x224 - x4 * x223) * sensor_y;
	const GEN_FLT x227 = x183 + x207 + x223 * x164 + x223 * x165 - x223 * x167;
	const GEN_FLT x228 = x55 * x227 + x57 * x226 + x58 * x225;
	const GEN_FLT x229 = x46 * x227 + x48 * x226 + x52 * x225;
	const GEN_FLT x230 = (x228 * x106 - x229 * x107) * x109;
	const GEN_FLT x231 = x25 * x227 + x26 * x226 + x40 * x225;
	const GEN_FLT x232 = x229 * x125 + x94 * x228;
	const GEN_FLT x233 = x91 * (x63 * x231 - x98 * (x232 + x231 * x128));
	const GEN_FLT x234 = x66 * x233;
	const GEN_FLT x235 = x64 * (x234 - x65 * x233) + x67 * x233;
	const GEN_FLT x236 = x64 * x235 + x68 * x233;
	const GEN_FLT x237 = -x232 * x112 + x78 * x231;
	const GEN_FLT x238 = x230 - x237 * x111;
	const GEN_FLT x239 =
		x90 *
		(x237 -
		 x119 *
			 (-x238 * x117 -
			  x83 * (x64 * x236 +
					 x64 * (x236 + x64 * (x235 + x64 * (x234 - x233 * x118 + x71 * x233) + x72 * x233) + x73 * x233) +
					 x69 * x233 + x74 * x233)) +
		 x233 * x122 + x238 * x121 + x88 * x236);
	*(out++) = x110 - x123 - (-x110 + x123) * x124;
	*(out++) = x127 - x138 - (-x127 + x138) * x124;
	*(out++) = x139 - x148 - (-x139 + x148) * x124;
	*(out++) = x178 - x181 - (-x178 + x181) * x124;
	*(out++) = x194 - x203 - (-x194 + x203) * x124;
	*(out++) = x213 - x222 - (-x213 + x222) * x124;
	*(out++) = x230 - x239 - (-x230 + x239) * x124;
}

// Jacobian of reproject_axis_x_gen2 wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_reproject_axis_x_gen2_jac_sensor_pt(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
														   const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt(x1 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - (x0 + x1) * x5;
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = pow(obj_qj, 2);
	const GEN_FLT x9 = pow(obj_qi, 2);
	const GEN_FLT x10 = x7 + x9;
	const GEN_FLT x11 = sqrt(x10 + x8 + pow(obj_qw, 2));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 - (x7 + x8) * x12;
	const GEN_FLT x14 = lh_qw * lh_qk;
	const GEN_FLT x15 = lh_qj * lh_qi;
	const GEN_FLT x16 = -x14 + x15;
	const GEN_FLT x17 = obj_qw * obj_qk;
	const GEN_FLT x18 = obj_qj * obj_qi;
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 4 * x4 * x11;
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = lh_qw * lh_qj;
	const GEN_FLT x23 = lh_qk * lh_qi;
	const GEN_FLT x24 = x22 + x23;
	const GEN_FLT x25 = obj_qw * obj_qj;
	const GEN_FLT x26 = obj_qk * obj_qi;
	const GEN_FLT x27 = -x25 + x26;
	const GEN_FLT x28 = x20 * x27;
	const GEN_FLT x29 = x21 * x16 + x24 * x28 + x6 * x13;
	const GEN_FLT x30 = 1 - (x1 + x2) * x5;
	const GEN_FLT x31 = 1 - (x8 + x9) * x12;
	const GEN_FLT x32 = obj_qw * obj_qi;
	const GEN_FLT x33 = obj_qk * obj_qj;
	const GEN_FLT x34 = x32 + x33;
	const GEN_FLT x35 = x34 * x12;
	const GEN_FLT x36 = x27 * x12;
	const GEN_FLT x37 = obj_pz + x31 * sensor_z + x35 * sensor_y + x36 * sensor_x;
	const GEN_FLT x38 = lh_qw * lh_qi;
	const GEN_FLT x39 = lh_qk * lh_qj;
	const GEN_FLT x40 = x38 + x39;
	const GEN_FLT x41 = -x32 + x33;
	const GEN_FLT x42 = x41 * x12;
	const GEN_FLT x43 = 1 - x12 * x10;
	const GEN_FLT x44 = x12 * x19;
	const GEN_FLT x45 = obj_py + x42 * sensor_z + x43 * sensor_y + x44 * sensor_x;
	const GEN_FLT x46 = x5 * x45;
	const GEN_FLT x47 = -x22 + x23;
	const GEN_FLT x48 = x25 + x26;
	const GEN_FLT x49 = -x17 + x18;
	const GEN_FLT x50 = obj_px + x13 * sensor_x + x48 * x12 * sensor_z + x49 * x12 * sensor_y;
	const GEN_FLT x51 = x5 * x50;
	const GEN_FLT x52 = lh_pz + x30 * x37 + x40 * x46 + x51 * x47;
	const GEN_FLT x53 = x5 * x37;
	const GEN_FLT x54 = lh_px + x46 * x16 + x53 * x24 + x6 * x50;
	const GEN_FLT x55 = pow(x54, 2);
	const GEN_FLT x56 = x52 / x55;
	const GEN_FLT x57 = pow(x54, -1);
	const GEN_FLT x58 = x5 * x13;
	const GEN_FLT x59 = x30 * x36 + x40 * x21 + x58 * x47;
	const GEN_FLT x60 = x55 + pow(x52, 2);
	const GEN_FLT x61 = pow(x60, -1);
	const GEN_FLT x62 = x61 * x55;
	const GEN_FLT x63 = (x56 * x29 - x57 * x59) * x62;
	const GEN_FLT x64 = -x38 + x39;
	const GEN_FLT x65 = 1 - x3 * x5;
	const GEN_FLT x66 = x14 + x15;
	const GEN_FLT x67 = lh_py + x64 * x53 + x65 * x45 + x66 * x51;
	const GEN_FLT x68 = 0.523598775598299 + tilt_0;
	const GEN_FLT x69 = cos(x68);
	const GEN_FLT x70 = pow(x69, -1);
	const GEN_FLT x71 = pow(x67, 2);
	const GEN_FLT x72 = x60 + x71;
	const GEN_FLT x73 = x70 / sqrt(x72);
	const GEN_FLT x74 = asin(x73 * x67);
	const GEN_FLT x75 = 8.0108022e-06 * x74;
	const GEN_FLT x76 = -8.0108022e-06 - x75;
	const GEN_FLT x77 = 0.0028679863 + x74 * x76;
	const GEN_FLT x78 = 5.3685255e-06 + x74 * x77;
	const GEN_FLT x79 = 0.0076069798 + x78 * x74;
	const GEN_FLT x80 = x79 * x74;
	const GEN_FLT x81 = -8.0108022e-06 - 1.60216044e-05 * x74;
	const GEN_FLT x82 = x77 + x81 * x74;
	const GEN_FLT x83 = x78 + x82 * x74;
	const GEN_FLT x84 = x79 + x83 * x74;
	const GEN_FLT x85 = x80 + x84 * x74;
	const GEN_FLT x86 = sin(x68);
	const GEN_FLT x87 = tan(x68);
	const GEN_FLT x88 = x87 / sqrt(x60);
	const GEN_FLT x89 = x88 * x67;
	const GEN_FLT x90 = atan2(-x52, x54);
	const GEN_FLT x91 = ogeeMag_0 + x90 - asin(x89);
	const GEN_FLT x92 = curve_0 + sin(x91) * ogeePhase_0;
	const GEN_FLT x93 = x86 * x92;
	const GEN_FLT x94 = x69 - x85 * x93;
	const GEN_FLT x95 = pow(x94, -1);
	const GEN_FLT x96 = pow(x74, 2);
	const GEN_FLT x97 = x92 * x96;
	const GEN_FLT x98 = x97 * x95;
	const GEN_FLT x99 = x89 + x79 * x98;
	const GEN_FLT x100 = pow(1 - pow(x99, 2), -1.0 / 2.0);
	const GEN_FLT x101 = pow(1 - x71 / (x72 * pow(x69, 2)), -1.0 / 2.0);
	const GEN_FLT x102 = x64 * x28 + x65 * x44 + x66 * x58;
	const GEN_FLT x103 = 2 * x67;
	const GEN_FLT x104 = 2 * x54;
	const GEN_FLT x105 = 2 * x52;
	const GEN_FLT x106 = x29 * x104 + x59 * x105;
	const GEN_FLT x107 = (1.0 / 2.0) * x67;
	const GEN_FLT x108 = x70 * x107 / pow(x72, 3.0 / 2.0);
	const GEN_FLT x109 = x101 * (-x108 * (x106 + x103 * x102) + x73 * x102);
	const GEN_FLT x110 = x76 * x109;
	const GEN_FLT x111 = x74 * (x110 - x75 * x109) + x77 * x109;
	const GEN_FLT x112 = x74 * x111 + x78 * x109;
	const GEN_FLT x113 = 2 * x80 * x92 * x95;
	const GEN_FLT x114 = x87 * x107 / pow(x60, 3.0 / 2.0);
	const GEN_FLT x115 = -x106 * x114 + x88 * x102;
	const GEN_FLT x116 = pow(1 - pow(x87, 2) * x71 * x61, -1.0 / 2.0);
	const GEN_FLT x117 = cos(x91) * ogeePhase_0;
	const GEN_FLT x118 = x117 * (x63 - x115 * x116);
	const GEN_FLT x119 = x85 * x86;
	const GEN_FLT x120 = 2.40324066e-05 * x74;
	const GEN_FLT x121 = x79 * x97 / pow(x94, 2);
	const GEN_FLT x122 = x79 * x96 * x95;
	const GEN_FLT x123 =
		x100 *
		(x115 + x109 * x113 + x118 * x122 -
		 x121 *
			 (-x118 * x119 -
			  x93 * (x74 * x112 +
					 x74 * (x112 + x74 * (x111 + x74 * (x110 - x109 * x120 + x81 * x109) + x82 * x109) + x83 * x109) +
					 x79 * x109 + x84 * x109)) +
		 x98 * x112);
	const GEN_FLT x124 = cos(gibPhase_0 + x90 - asin(x99)) * gibMag_0;
	const GEN_FLT x125 = x5 * x43;
	const GEN_FLT x126 = x6 * x12;
	const GEN_FLT x127 = x34 * x20;
	const GEN_FLT x128 = x16 * x125 + x24 * x127 + x49 * x126;
	const GEN_FLT x129 = x49 * x20;
	const GEN_FLT x130 = x30 * x35 + x40 * x125 + x47 * x129;
	const GEN_FLT x131 = (x56 * x128 - x57 * x130) * x62;
	const GEN_FLT x132 = x64 * x127 + x65 * x43 + x66 * x129;
	const GEN_FLT x133 = x104 * x128 + x105 * x130;
	const GEN_FLT x134 = x101 * (-x108 * (x133 + x103 * x132) + x73 * x132);
	const GEN_FLT x135 = x76 * x134;
	const GEN_FLT x136 = x74 * (x135 - x75 * x134) + x77 * x134;
	const GEN_FLT x137 = x74 * x136 + x78 * x134;
	const GEN_FLT x138 = -x114 * x133 + x88 * x132;
	const GEN_FLT x139 = x131 - x116 * x138;
	const GEN_FLT x140 = x119 * x117;
	const GEN_FLT x141 = x117 * x122;
	const GEN_FLT x142 =
		x100 *
		(x138 + x113 * x134 -
		 x121 *
			 (-x139 * x140 -
			  x93 * (x74 * x137 +
					 x74 * (x137 + x74 * (x136 + x74 * (x135 - x120 * x134 + x81 * x134) + x82 * x134) + x83 * x134) +
					 x79 * x134 + x84 * x134)) +
		 x139 * x141 + x98 * x137);
	const GEN_FLT x143 = x41 * x20;
	const GEN_FLT x144 = x5 * x31;
	const GEN_FLT x145 = x16 * x143 + x24 * x144 + x48 * x126;
	const GEN_FLT x146 = x48 * x20;
	const GEN_FLT x147 = x30 * x31 + x40 * x143 + x47 * x146;
	const GEN_FLT x148 = (x56 * x145 - x57 * x147) * x62;
	const GEN_FLT x149 = x64 * x144 + x65 * x42 + x66 * x146;
	const GEN_FLT x150 = x104 * x145 + x105 * x147;
	const GEN_FLT x151 = x101 * (-x108 * (x150 + x103 * x149) + x73 * x149);
	const GEN_FLT x152 = x76 * x151;
	const GEN_FLT x153 = x74 * (x152 - x75 * x151) + x77 * x151;
	const GEN_FLT x154 = x74 * x153 + x78 * x151;
	const GEN_FLT x155 = -x114 * x150 + x88 * x149;
	const GEN_FLT x156 = x148 - x116 * x155;
	const GEN_FLT x157 =
		x100 *
		(x155 + x113 * x151 -
		 x121 *
			 (-x140 * x156 -
			  x93 * (x74 * x154 +
					 x74 * (x154 + x74 * (x153 + x74 * (x152 - x120 * x151 + x81 * x151) + x82 * x151) + x83 * x151) +
					 x79 * x151 + x84 * x151)) +
		 x141 * x156 + x98 * x154);
	*(out++) = -x123 + x63 - x124 * (x123 - x63);
	*(out++) = x131 - x142 - (-x131 + x142) * x124;
	*(out++) = x148 - x157 - (-x148 + x157) * x124;
}

// Jacobian of reproject_axis_x_gen2 wrt [lh_px, lh_py, lh_pz, lh_qw, lh_qi, lh_qj, lh_qk]
static inline void gen_reproject_axis_x_gen2_jac_lh_p(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
													  const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qk, 2);
	const GEN_FLT x4 = x1 + x3;
	const GEN_FLT x5 = sqrt(x0 + x4 + pow(lh_qw, 2));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = pow(obj_qk, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = 2 * sqrt(x10 + x7 + pow(obj_qw, 2));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 = obj_pz + (1 - (x7 + x8) * x11) * sensor_z + (x12 + x13) * x14 + (-x15 + x16) * x17;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + (1 - x11 * x10) * sensor_y + (-x12 + x13) * x22 + (x23 + x24) * x17;
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = -x27 + x28;
	const GEN_FLT x30 = obj_px + (1 - (x7 + x9) * x11) * sensor_x + (x15 + x16) * x22 + (-x23 + x24) * x14;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + x18 * (1 - x2 * x6) + x21 * x26 + x31 * x29;
	const GEN_FLT x33 = x27 + x28;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = -x35 + x36;
	const GEN_FLT x38 = x0 + x3;
	const GEN_FLT x39 = lh_px + x30 * (1 - x6 * x38) + x34 * x33 + x37 * x26;
	const GEN_FLT x40 = pow(x39, 2);
	const GEN_FLT x41 = x40 + pow(x32, 2);
	const GEN_FLT x42 = pow(x41, -1);
	const GEN_FLT x43 = x42 * x32;
	const GEN_FLT x44 = -x19 + x20;
	const GEN_FLT x45 = x35 + x36;
	const GEN_FLT x46 = lh_py + x25 * (1 - x4 * x6) + x44 * x34 + x45 * x31;
	const GEN_FLT x47 = 0.523598775598299 + tilt_0;
	const GEN_FLT x48 = cos(x47);
	const GEN_FLT x49 = pow(x48, -1);
	const GEN_FLT x50 = pow(x46, 2);
	const GEN_FLT x51 = x41 + x50;
	const GEN_FLT x52 = x49 / sqrt(x51);
	const GEN_FLT x53 = asin(x52 * x46);
	const GEN_FLT x54 = 8.0108022e-06 * x53;
	const GEN_FLT x55 = -8.0108022e-06 - x54;
	const GEN_FLT x56 = 0.0028679863 + x53 * x55;
	const GEN_FLT x57 = 5.3685255e-06 + x53 * x56;
	const GEN_FLT x58 = 0.0076069798 + x53 * x57;
	const GEN_FLT x59 = x53 * x58;
	const GEN_FLT x60 = -8.0108022e-06 - 1.60216044e-05 * x53;
	const GEN_FLT x61 = x56 + x60 * x53;
	const GEN_FLT x62 = x57 + x61 * x53;
	const GEN_FLT x63 = x58 + x62 * x53;
	const GEN_FLT x64 = x59 + x63 * x53;
	const GEN_FLT x65 = sin(x47);
	const GEN_FLT x66 = tan(x47);
	const GEN_FLT x67 = x66 / sqrt(x41);
	const GEN_FLT x68 = x67 * x46;
	const GEN_FLT x69 = atan2(-x32, x39);
	const GEN_FLT x70 = ogeeMag_0 + x69 - asin(x68);
	const GEN_FLT x71 = curve_0 + sin(x70) * ogeePhase_0;
	const GEN_FLT x72 = x71 * x65;
	const GEN_FLT x73 = x48 - x72 * x64;
	const GEN_FLT x74 = pow(x73, -1);
	const GEN_FLT x75 = pow(x53, 2);
	const GEN_FLT x76 = x71 * x75;
	const GEN_FLT x77 = x74 * x76;
	const GEN_FLT x78 = x68 + x77 * x58;
	const GEN_FLT x79 = pow(1 - pow(x78, 2), -1.0 / 2.0);
	const GEN_FLT x80 = x66 / pow(x41, 3.0 / 2.0);
	const GEN_FLT x81 = x80 * x46;
	const GEN_FLT x82 = x81 * x39;
	const GEN_FLT x83 = pow(1 - x50 / (x51 * pow(x48, 2)), -1.0 / 2.0);
	const GEN_FLT x84 = x49 / pow(x51, 3.0 / 2.0);
	const GEN_FLT x85 = x84 * x46;
	const GEN_FLT x86 = x85 * x39;
	const GEN_FLT x87 = x83 * x86;
	const GEN_FLT x88 = -x87 * x55;
	const GEN_FLT x89 = x53 * (x88 + x87 * x54) - x87 * x56;
	const GEN_FLT x90 = -x87 * x57 + x89 * x53;
	const GEN_FLT x91 = pow(1 - pow(x66, 2) * x50 * x42, -1.0 / 2.0);
	const GEN_FLT x92 = cos(x70) * ogeePhase_0;
	const GEN_FLT x93 = x92 * (x43 + x82 * x91);
	const GEN_FLT x94 = x64 * x65;
	const GEN_FLT x95 = 2.40324066e-05 * x53;
	const GEN_FLT x96 = x83 * x60;
	const GEN_FLT x97 = x83 * x62;
	const GEN_FLT x98 = x83 * x58;
	const GEN_FLT x99 = x76 * x58 / pow(x73, 2);
	const GEN_FLT x100 = x75 * x74 * x58;
	const GEN_FLT x101 = 2 * x39;
	const GEN_FLT x102 = x71 * x74 * x59;
	const GEN_FLT x103 = x83 * x85 * x102;
	const GEN_FLT x104 =
		x79 * (-x82 - x101 * x103 + x77 * x90 + x93 * x100 -
			   x99 * (-x72 * (x53 * x90 +
							  x53 * (x90 + x53 * (x89 + x53 * (x88 - x86 * x96 + x87 * x95) - x87 * x61) - x86 * x97) -
							  x86 * x98 - x87 * x63) -
					  x93 * x94));
	const GEN_FLT x105 = cos(gibPhase_0 + x69 - asin(x78)) * gibMag_0;
	const GEN_FLT x106 = x83 * (x52 - x84 * x50);
	const GEN_FLT x107 = x55 * x106;
	const GEN_FLT x108 = x53 * (x107 - x54 * x106) + x56 * x106;
	const GEN_FLT x109 = x53 * x108 + x57 * x106;
	const GEN_FLT x110 = x92 * x94;
	const GEN_FLT x111 = x67 * x91;
	const GEN_FLT x112 = x92 * x100;
	const GEN_FLT x113 = 2 * x102;
	const GEN_FLT x114 =
		x79 *
		(x67 + x106 * x113 - x111 * x112 + x77 * x109 -
		 x99 * (x110 * x111 -
				x72 * (x53 * x109 +
					   x53 * (x109 + x53 * (x108 + x53 * (x107 + x60 * x106 - x95 * x106) + x61 * x106) + x62 * x106) +
					   x58 * x106 + x63 * x106)));
	const GEN_FLT x115 = x42 * x39;
	const GEN_FLT x116 = -x115;
	const GEN_FLT x117 = x46 * x32;
	const GEN_FLT x118 = x84 * x117;
	const GEN_FLT x119 = x83 * x118;
	const GEN_FLT x120 = -x55 * x119;
	const GEN_FLT x121 = x53 * (x120 + x54 * x119) - x56 * x119;
	const GEN_FLT x122 = x53 * x121 - x57 * x119;
	const GEN_FLT x123 = x80 * x117;
	const GEN_FLT x124 = x116 + x91 * x123;
	const GEN_FLT x125 = 2 * x32;
	const GEN_FLT x126 =
		x79 *
		(-x123 - x103 * x125 + x112 * x124 + x77 * x122 -
		 x99 * (-x110 * x124 -
				x72 * (x53 * x122 +
					   x53 * (x122 + x53 * (x121 + x53 * (x120 + x95 * x119 - x96 * x118) - x61 * x119) - x97 * x118) -
					   x63 * x119 - x98 * x118)));
	const GEN_FLT x127 = 2 / x5;
	const GEN_FLT x128 = x38 * x127;
	const GEN_FLT x129 = x30 * x128;
	const GEN_FLT x130 = x127 * lh_qw;
	const GEN_FLT x131 = x37 * x25;
	const GEN_FLT x132 = x26 * lh_qk;
	const GEN_FLT x133 = x33 * x18;
	const GEN_FLT x134 = x34 * lh_qj;
	const GEN_FLT x135 = -x132 + x134 - x129 * lh_qw + x130 * x131 + x130 * x133;
	const GEN_FLT x136 = x32 / x40;
	const GEN_FLT x137 = pow(x39, -1);
	const GEN_FLT x138 = x30 * x29;
	const GEN_FLT x139 = x25 * x21;
	const GEN_FLT x140 = x26 * lh_qi;
	const GEN_FLT x141 = x31 * lh_qj;
	const GEN_FLT x142 = x2 * x127;
	const GEN_FLT x143 = x18 * x142;
	const GEN_FLT x144 = x140 - x141 + x130 * x138 + x130 * x139 - x143 * lh_qw;
	const GEN_FLT x145 = x40 * x42;
	const GEN_FLT x146 = (x135 * x136 - x137 * x144) * x145;
	const GEN_FLT x147 = x45 * x30;
	const GEN_FLT x148 = x127 * x147;
	const GEN_FLT x149 = x31 * lh_qk;
	const GEN_FLT x150 = x4 * x127;
	const GEN_FLT x151 = x25 * x150;
	const GEN_FLT x152 = x44 * x18;
	const GEN_FLT x153 = x34 * lh_qi;
	const GEN_FLT x154 = x149 - x153 + x130 * x152 + x148 * lh_qw - x151 * lh_qw;
	const GEN_FLT x155 = 2 * x46;
	const GEN_FLT x156 = x101 * x135 + x125 * x144;
	const GEN_FLT x157 = (1.0 / 2.0) * x85;
	const GEN_FLT x158 = -x157 * (x156 + x154 * x155) + x52 * x154;
	const GEN_FLT x159 = x83 * x158;
	const GEN_FLT x160 = x55 * x159;
	const GEN_FLT x161 = x53 * (x160 - x54 * x159) + x56 * x159;
	const GEN_FLT x162 = x53 * x161 + x57 * x159;
	const GEN_FLT x163 = (1.0 / 2.0) * x81;
	const GEN_FLT x164 = -x163 * x156 + x67 * x154;
	const GEN_FLT x165 = x146 - x91 * x164;
	const GEN_FLT x166 =
		x79 *
		(x164 + x112 * x165 + x113 * x159 + x77 * x162 -
		 x99 * (-x110 * x165 -
				x72 * (x53 * x162 +
					   x53 * (x162 + x53 * (x161 + x53 * (x160 + x60 * x159 - x95 * x159) + x61 * x159) + x97 * x158) +
					   x63 * x159 + x98 * x158)));
	const GEN_FLT x167 = x127 * lh_qi;
	const GEN_FLT x168 = x26 * lh_qj;
	const GEN_FLT x169 = x34 * lh_qk;
	const GEN_FLT x170 = x168 + x169 - x129 * lh_qi + x167 * x131 + x167 * x133;
	const GEN_FLT x171 = x26 * lh_qw;
	const GEN_FLT x172 = 4 * x5;
	const GEN_FLT x173 = -x172 * lh_qi;
	const GEN_FLT x174 = x149 + x171 + x167 * x138 + x167 * x139 + x18 * (x173 - x142 * lh_qi);
	const GEN_FLT x175 = (x170 * x136 - x174 * x137) * x145;
	const GEN_FLT x176 = x34 * lh_qw;
	const GEN_FLT x177 = x141 - x176 + x148 * lh_qi + x167 * x152 + x25 * (x173 - x150 * lh_qi);
	const GEN_FLT x178 = x101 * x170 + x125 * x174;
	const GEN_FLT x179 = -x157 * (x178 + x177 * x155) + x52 * x177;
	const GEN_FLT x180 = x83 * x179;
	const GEN_FLT x181 = x55 * x180;
	const GEN_FLT x182 = x53 * (x181 - x54 * x180) + x56 * x180;
	const GEN_FLT x183 = x53 * x182 + x57 * x180;
	const GEN_FLT x184 = -x163 * x178 + x67 * x177;
	const GEN_FLT x185 = x175 - x91 * x184;
	const GEN_FLT x186 =
		x79 *
		(x184 + x112 * x185 + x113 * x180 + x77 * x183 -
		 x99 * (-x110 * x185 -
				x72 * (x53 * x183 +
					   x53 * (x183 + x53 * (x182 + x53 * (x181 + x60 * x180 - x95 * x180) + x61 * x180) + x97 * x179) +
					   x63 * x180 + x98 * x179)));
	const GEN_FLT x187 = -x172 * lh_qj;
	const GEN_FLT x188 = x127 * lh_qj;
	const GEN_FLT x189 = x140 + x176 + x188 * x131 + x188 * x133 + x30 * (x187 - x128 * lh_qj);
	const GEN_FLT x190 = x31 * lh_qw;
	const GEN_FLT x191 = x132 - x190 + x18 * (x187 - x142 * lh_qj) + x188 * x138 + x188 * x139;
	const GEN_FLT x192 = (x189 * x136 - x191 * x137) * x145;
	const GEN_FLT x193 = x31 * lh_qi;
	const GEN_FLT x194 = x169 + x193 + x148 * lh_qj - x151 * lh_qj + x188 * x152;
	const GEN_FLT x195 = x101 * x189 + x125 * x191;
	const GEN_FLT x196 = -x157 * (x195 + x194 * x155) + x52 * x194;
	const GEN_FLT x197 = x83 * x196;
	const GEN_FLT x198 = x55 * x197;
	const GEN_FLT x199 = x53 * (x198 - x54 * x197) + x56 * x197;
	const GEN_FLT x200 = x53 * x199 + x57 * x197;
	const GEN_FLT x201 = -x163 * x195 + x67 * x194;
	const GEN_FLT x202 = x192 - x91 * x201;
	const GEN_FLT x203 =
		x79 *
		(x201 + x113 * x197 + x202 * x112 + x77 * x200 -
		 x99 * (-x202 * x110 -
				x72 * (x53 * x200 +
					   x53 * (x200 + x53 * (x199 + x53 * (x198 + x60 * x197 - x95 * x197) + x61 * x197) + x97 * x196) +
					   x63 * x197 + x98 * x196)));
	const GEN_FLT x204 = -x172 * lh_qk;
	const GEN_FLT x205 = x127 * lh_qk;
	const GEN_FLT x206 = x18 * x205;
	const GEN_FLT x207 = x153 - x171 + x205 * x131 + x30 * (x204 - x128 * lh_qk) + x33 * x206;
	const GEN_FLT x208 = x168 + x193 - x143 * lh_qk + x205 * x138 + x205 * x139;
	const GEN_FLT x209 = (x207 * x136 - x208 * x137) * x145;
	const GEN_FLT x210 = x134 + x190 + x205 * x147 + x25 * (x204 - x150 * lh_qk) + x44 * x206;
	const GEN_FLT x211 = x207 * x101 + x208 * x125;
	const GEN_FLT x212 = -x157 * (x211 + x210 * x155) + x52 * x210;
	const GEN_FLT x213 = x83 * x212;
	const GEN_FLT x214 = x55 * x213;
	const GEN_FLT x215 = x53 * (x214 - x54 * x213) + x56 * x213;
	const GEN_FLT x216 = x53 * x215 + x57 * x213;
	const GEN_FLT x217 = -x211 * x163 + x67 * x210;
	const GEN_FLT x218 = x209 - x91 * x217;
	const GEN_FLT x219 =
		x79 *
		(x217 + x213 * x113 + x218 * x112 + x77 * x216 -
		 x99 * (-x218 * x110 -
				x72 * (x53 * x216 +
					   x53 * (x216 + x53 * (x215 + x53 * (x214 + x60 * x213 - x95 * x213) + x61 * x213) + x97 * x212) +
					   x63 * x213 + x98 * x212)));
	*(out++) = -x104 + x43 - x105 * (x104 - x43);
	*(out++) = -x114 - x105 * x114;
	*(out++) = x116 - x126 - (x115 + x126) * x105;
	*(out++) = x146 - x166 - (-x146 + x166) * x105;
	*(out++) = x175 - x186 - (-x175 + x186) * x105;
	*(out++) = x192 - x203 - (-x192 + x203) * x105;
	*(out++) = x209 - x219 - (-x209 + x219) * x105;
}

// Jacobian of reproject_axis_x_gen2 wrt [phase_0, tilt_0, curve_0, gibPhase_0, gibMag_0, ogeeMag_0, ogeePhase_0]
static inline void gen_reproject_axis_x_gen2_jac_bsc0(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
													  const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = 0.523598775598299 + tilt_0;
	const GEN_FLT x1 = tan(x0);
	const GEN_FLT x2 = lh_qw * lh_qi;
	const GEN_FLT x3 = lh_qk * lh_qj;
	const GEN_FLT x4 = pow(obj_qj, 2);
	const GEN_FLT x5 = pow(obj_qi, 2);
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = 2 * sqrt(x6 + x7 + pow(obj_qw, 2));
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = obj_pz + x11 * (x10 + x9) + (1 - x6 * x8) * sensor_z + (-x12 + x13) * x14;
	const GEN_FLT x16 = pow(lh_qi, 2);
	const GEN_FLT x17 = pow(lh_qk, 2);
	const GEN_FLT x18 = pow(lh_qj, 2);
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 2 * sqrt(x16 + x19 + pow(lh_qw, 2));
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + x22 * (x10 - x9) + (1 - (x5 + x7) * x8) * sensor_y + (x23 + x24) * x14;
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 = obj_px + (1 - (x4 + x7) * x8) * sensor_x + (x12 + x13) * x22 + (-x23 + x24) * x11;
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = lh_py + x25 * (1 - (x16 + x17) * x20) + (-x2 + x3) * x21 + (x26 + x27) * x29;
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = lh_pz + x15 * (1 - (x16 + x18) * x20) + (x2 + x3) * x31 + (-x32 + x33) * x29;
	const GEN_FLT x35 = lh_px + x28 * (1 - x20 * x19) + (-x26 + x27) * x31 + (x32 + x33) * x21;
	const GEN_FLT x36 = pow(x34, 2) + pow(x35, 2);
	const GEN_FLT x37 = x30 / sqrt(x36);
	const GEN_FLT x38 = x1 * x37;
	const GEN_FLT x39 = atan2(-x34, x35);
	const GEN_FLT x40 = ogeeMag_0 + x39 - asin(x38);
	const GEN_FLT x41 = sin(x40);
	const GEN_FLT x42 = curve_0 + x41 * ogeePhase_0;
	const GEN_FLT x43 = cos(x0);
	const GEN_FLT x44 = pow(x30, 2);
	const GEN_FLT x45 = x36 + x44;
	const GEN_FLT x46 = x30 / sqrt(x45);
	const GEN_FLT x47 = asin(x46 / x43);
	const GEN_FLT x48 = 8.0108022e-06 * x47;
	const GEN_FLT x49 = -8.0108022e-06 - x48;
	const GEN_FLT x50 = 0.0028679863 + x47 * x49;
	const GEN_FLT x51 = 5.3685255e-06 + x50 * x47;
	const GEN_FLT x52 = 0.0076069798 + x51 * x47;
	const GEN_FLT x53 = sin(x0);
	const GEN_FLT x54 = x52 * x47;
	const GEN_FLT x55 = -8.0108022e-06 - 1.60216044e-05 * x47;
	const GEN_FLT x56 = x50 + x55 * x47;
	const GEN_FLT x57 = x51 + x56 * x47;
	const GEN_FLT x58 = x52 + x57 * x47;
	const GEN_FLT x59 = x54 + x58 * x47;
	const GEN_FLT x60 = x59 * x42;
	const GEN_FLT x61 = x60 * x53;
	const GEN_FLT x62 = x43 - x61;
	const GEN_FLT x63 = pow(x62, -1);
	const GEN_FLT x64 = pow(x47, 2);
	const GEN_FLT x65 = x63 * x64;
	const GEN_FLT x66 = x65 * x52;
	const GEN_FLT x67 = x38 + x66 * x42;
	const GEN_FLT x68 = pow(1 - pow(x67, 2), -1.0 / 2.0);
	const GEN_FLT x69 = pow(x1, 2);
	const GEN_FLT x70 = x37 * (1 + x69);
	const GEN_FLT x71 = pow(x43, -2);
	const GEN_FLT x72 = x71 * x46 / sqrt(1 - x71 * x44 / x45);
	const GEN_FLT x73 = x72 * x53;
	const GEN_FLT x74 = x73 * x49;
	const GEN_FLT x75 = x47 * (x74 - x73 * x48) + x73 * x50;
	const GEN_FLT x76 = x73 * x51 + x75 * x47;
	const GEN_FLT x77 = cos(x40) * ogeePhase_0;
	const GEN_FLT x78 = x70 / sqrt(1 - x69 * x44 / x36);
	const GEN_FLT x79 = x53 * x42;
	const GEN_FLT x80 = x64 * x52 / pow(x62, 2);
	const GEN_FLT x81 = x77 * x66;
	const GEN_FLT x82 =
		x68 * (x70 - x81 * x78 + x76 * x65 * x42 -
			   x80 * x42 *
				   (-x53 - x60 * x43 -
					x79 * (x47 * (x76 + x47 * (x75 + x47 * (x74 - 2.40324066e-05 * x73 * x47 + x73 * x55) + x73 * x56) +
								  x73 * x57) +
						   x73 * x52 + x73 * x58 + x76 * x47) +
					x78 * x77 * x53 * x59) +
			   2 * x72 * x79 * x63 * x54);
	const GEN_FLT x83 = -gibPhase_0 - x39 + asin(x67);
	const GEN_FLT x84 = cos(x83) * gibMag_0;
	const GEN_FLT x85 = x80 * x61;
	const GEN_FLT x86 = (x66 + x85) * x68;
	const GEN_FLT x87 = x68 * (x81 + x85 * x77);
	const GEN_FLT x88 = (x66 * x41 + x85 * x41) * x68;
	*(out++) = -1;
	*(out++) = -x82 - x82 * x84;
	*(out++) = -x86 - x84 * x86;
	*(out++) = x84;
	*(out++) = -sin(x83);
	*(out++) = -x87 - x84 * x87;
	*(out++) = -x88 - x88 * x84;
}

/** Applying function <function reproject_axis_y_gen2 at 0x7ffa3c1c7a70> */
static inline void gen_reproject_axis_y_gen2(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
											 const SurvivePose *lh_p, const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = 0.523598775598299 - tilt_1;
	const GEN_FLT x1 = lh_qw * lh_qi;
	const GEN_FLT x2 = lh_qk * lh_qj;
	const GEN_FLT x3 = pow(obj_qj, 2);
	const GEN_FLT x4 = pow(obj_qi, 2);
	const GEN_FLT x5 = pow(obj_qk, 2);
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = 2 * sqrt(x3 + x6 + pow(obj_qw, 2));
	const GEN_FLT x8 = obj_qw * obj_qi;
	const GEN_FLT x9 = obj_qk * obj_qj;
	const GEN_FLT x10 = x7 * sensor_y;
	const GEN_FLT x11 = obj_qw * obj_qj;
	const GEN_FLT x12 = obj_qk * obj_qi;
	const GEN_FLT x13 = x7 * sensor_x;
	const GEN_FLT x14 = obj_pz + (1 - (x3 + x4) * x7) * sensor_z + (-x11 + x12) * x13 + (x8 + x9) * x10;
	const GEN_FLT x15 = pow(lh_qk, 2);
	const GEN_FLT x16 = pow(lh_qj, 2);
	const GEN_FLT x17 = pow(lh_qi, 2);
	const GEN_FLT x18 = x16 + x17;
	const GEN_FLT x19 = 2 * sqrt(x15 + x18 + pow(lh_qw, 2));
	const GEN_FLT x20 = x14 * x19;
	const GEN_FLT x21 = x7 * sensor_z;
	const GEN_FLT x22 = obj_qw * obj_qk;
	const GEN_FLT x23 = obj_qj * obj_qi;
	const GEN_FLT x24 = obj_py + (1 - x6 * x7) * sensor_y + (x22 + x23) * x13 + (-x8 + x9) * x21;
	const GEN_FLT x25 = lh_qw * lh_qk;
	const GEN_FLT x26 = lh_qj * lh_qi;
	const GEN_FLT x27 = obj_px + (1 - (x3 + x5) * x7) * sensor_x + (x11 + x12) * x21 + (-x22 + x23) * x10;
	const GEN_FLT x28 = x27 * x19;
	const GEN_FLT x29 = lh_py + x24 * (1 - (x15 + x17) * x19) + (-x1 + x2) * x20 + (x25 + x26) * x28;
	const GEN_FLT x30 = x24 * x19;
	const GEN_FLT x31 = lh_qw * lh_qj;
	const GEN_FLT x32 = lh_qk * lh_qi;
	const GEN_FLT x33 = lh_pz + x14 * (1 - x19 * x18) + (x1 + x2) * x30 + (-x31 + x32) * x28;
	const GEN_FLT x34 = lh_px + x27 * (1 - (x15 + x16) * x19) + (-x25 + x26) * x30 + (x31 + x32) * x20;
	const GEN_FLT x35 = pow(x33, 2) + pow(x34, 2);
	const GEN_FLT x36 = -tan(x0) * x29 / sqrt(x35);
	const GEN_FLT x37 = atan2(-x33, x34);
	const GEN_FLT x38 = curve_1 + sin(ogeeMag_1 + x37 - asin(x36)) * ogeePhase_1;
	const GEN_FLT x39 = cos(x0);
	const GEN_FLT x40 = asin(x29 / (x39 * sqrt(x35 + pow(x29, 2))));
	const GEN_FLT x41 = 0.0028679863 + x40 * (-8.0108022e-06 - 8.0108022e-06 * x40);
	const GEN_FLT x42 = 5.3685255e-06 + x40 * x41;
	const GEN_FLT x43 = 0.0076069798 + x40 * x42;
	const GEN_FLT x44 =
		asin(x36 +
			 pow(x40, 2) * x43 * x38 /
				 (x39 + sin(x0) * x38 *
							(x40 * x43 +
							 x40 * (x43 + x40 * (x42 + x40 * (x41 + x40 * (-8.0108022e-06 - 1.60216044e-05 * x40)))))));
	*(out++) = -1.5707963267949 - phase_1 + x37 - x44 - sin(-gibPhase_1 - x37 + x44) * gibMag_1;
}

// Jacobian of reproject_axis_y_gen2 wrt [obj_px, obj_py, obj_pz, obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_reproject_axis_y_gen2_jac_obj_p(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
													   const SurvivePose *lh_p, const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qi, 2);
	const GEN_FLT x4 = sqrt(x2 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - x2 * x5;
	const GEN_FLT x7 = 1 - (x1 + x3) * x5;
	const GEN_FLT x8 = pow(obj_qj, 2);
	const GEN_FLT x9 = pow(obj_qi, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = x11 + x8;
	const GEN_FLT x13 = sqrt(x12 + x9 + pow(obj_qw, 2));
	const GEN_FLT x14 = 2 * x13;
	const GEN_FLT x15 = obj_qw * obj_qi;
	const GEN_FLT x16 = obj_qk * obj_qj;
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = x14 * sensor_y;
	const GEN_FLT x19 = obj_qw * obj_qj;
	const GEN_FLT x20 = obj_qk * obj_qi;
	const GEN_FLT x21 = -x19 + x20;
	const GEN_FLT x22 = x14 * sensor_x;
	const GEN_FLT x23 = obj_pz + x18 * x17 + x22 * x21 + (1 - x14 * x10) * sensor_z;
	const GEN_FLT x24 = -x15 + x16;
	const GEN_FLT x25 = x14 * sensor_z;
	const GEN_FLT x26 = x11 + x9;
	const GEN_FLT x27 = obj_qw * obj_qk;
	const GEN_FLT x28 = obj_qj * obj_qi;
	const GEN_FLT x29 = x27 + x28;
	const GEN_FLT x30 = obj_py + x22 * x29 + x24 * x25 + (1 - x26 * x14) * sensor_y;
	const GEN_FLT x31 = lh_qw * lh_qi;
	const GEN_FLT x32 = lh_qk * lh_qj;
	const GEN_FLT x33 = x31 + x32;
	const GEN_FLT x34 = x5 * x33;
	const GEN_FLT x35 = x19 + x20;
	const GEN_FLT x36 = -x27 + x28;
	const GEN_FLT x37 = obj_px + x35 * x25 + x36 * x18 + (1 - x14 * x12) * sensor_x;
	const GEN_FLT x38 = lh_qw * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qi;
	const GEN_FLT x40 = -x38 + x39;
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = lh_pz + x30 * x34 + x41 * x37 + x7 * x23;
	const GEN_FLT x43 = x38 + x39;
	const GEN_FLT x44 = x5 * x23;
	const GEN_FLT x45 = lh_qw * lh_qk;
	const GEN_FLT x46 = lh_qj * lh_qi;
	const GEN_FLT x47 = -x45 + x46;
	const GEN_FLT x48 = x5 * x47;
	const GEN_FLT x49 = lh_px + x43 * x44 + x48 * x30 + x6 * x37;
	const GEN_FLT x50 = pow(x49, 2);
	const GEN_FLT x51 = pow(x50, -1);
	const GEN_FLT x52 = x51 * x42;
	const GEN_FLT x53 = pow(x49, -1);
	const GEN_FLT x54 = x50 + pow(x42, 2);
	const GEN_FLT x55 = pow(x54, -1);
	const GEN_FLT x56 = x50 * x55;
	const GEN_FLT x57 = x56 * (-x53 * x41 + x6 * x52);
	const GEN_FLT x58 = -x31 + x32;
	const GEN_FLT x59 = 1 - (x0 + x3) * x5;
	const GEN_FLT x60 = x45 + x46;
	const GEN_FLT x61 = x5 * x60;
	const GEN_FLT x62 = lh_py + x58 * x44 + x59 * x30 + x61 * x37;
	const GEN_FLT x63 = 0.523598775598299 - tilt_1;
	const GEN_FLT x64 = cos(x63);
	const GEN_FLT x65 = pow(x64, -1);
	const GEN_FLT x66 = pow(x62, 2);
	const GEN_FLT x67 = x54 + x66;
	const GEN_FLT x68 = x65 / sqrt(x67);
	const GEN_FLT x69 = asin(x62 * x68);
	const GEN_FLT x70 = 8.0108022e-06 * x69;
	const GEN_FLT x71 = -8.0108022e-06 - x70;
	const GEN_FLT x72 = 0.0028679863 + x71 * x69;
	const GEN_FLT x73 = 5.3685255e-06 + x72 * x69;
	const GEN_FLT x74 = 0.0076069798 + x73 * x69;
	const GEN_FLT x75 = x74 * x69;
	const GEN_FLT x76 = -8.0108022e-06 - 1.60216044e-05 * x69;
	const GEN_FLT x77 = x72 + x76 * x69;
	const GEN_FLT x78 = x73 + x77 * x69;
	const GEN_FLT x79 = x74 + x78 * x69;
	const GEN_FLT x80 = x75 + x79 * x69;
	const GEN_FLT x81 = tan(x63);
	const GEN_FLT x82 = x81 / sqrt(x54);
	const GEN_FLT x83 = -x82 * x62;
	const GEN_FLT x84 = atan2(-x42, x49);
	const GEN_FLT x85 = ogeeMag_1 + x84 - asin(x83);
	const GEN_FLT x86 = curve_1 + sin(x85) * ogeePhase_1;
	const GEN_FLT x87 = sin(x63);
	const GEN_FLT x88 = x86 * x87;
	const GEN_FLT x89 = x64 + x80 * x88;
	const GEN_FLT x90 = pow(x89, -1);
	const GEN_FLT x91 = pow(x69, 2);
	const GEN_FLT x92 = x86 * x91;
	const GEN_FLT x93 = x92 * x90;
	const GEN_FLT x94 = x83 + x74 * x93;
	const GEN_FLT x95 = pow(1 - pow(x94, 2), -1.0 / 2.0);
	const GEN_FLT x96 = pow(1 - pow(x81, 2) * x66 * x55, -1.0 / 2.0);
	const GEN_FLT x97 = 2 * x49;
	const GEN_FLT x98 = 4 * x4;
	const GEN_FLT x99 = x98 * x42;
	const GEN_FLT x100 = x6 * x97 + x99 * x40;
	const GEN_FLT x101 = (1.0 / 2.0) * x62;
	const GEN_FLT x102 = x81 * x101 / pow(x54, 3.0 / 2.0);
	const GEN_FLT x103 = x100 * x102 - x82 * x61;
	const GEN_FLT x104 = x57 - x96 * x103;
	const GEN_FLT x105 = cos(x85) * ogeePhase_1;
	const GEN_FLT x106 = x74 * x91 * x90;
	const GEN_FLT x107 = x105 * x106;
	const GEN_FLT x108 = pow(1 - x66 / (pow(x64, 2) * x67), -1.0 / 2.0);
	const GEN_FLT x109 = x62 * x98;
	const GEN_FLT x110 = x65 * x101 / pow(x67, 3.0 / 2.0);
	const GEN_FLT x111 = x108 * (-x110 * (x100 + x60 * x109) + x61 * x68);
	const GEN_FLT x112 = 2 * x86 * x75 * x90;
	const GEN_FLT x113 = x80 * x87;
	const GEN_FLT x114 = x105 * x113;
	const GEN_FLT x115 = x71 * x111;
	const GEN_FLT x116 = 2.40324066e-05 * x69;
	const GEN_FLT x117 = x69 * (x115 - x70 * x111) + x72 * x111;
	const GEN_FLT x118 = x69 * x117 + x73 * x111;
	const GEN_FLT x119 = x74 * x92 / pow(x89, 2);
	const GEN_FLT x120 =
		x95 *
		(x103 + x104 * x107 + x111 * x112 -
		 x119 *
			 (x104 * x114 +
			  x88 * (x69 * x118 +
					 x69 * (x118 + x69 * (x117 + x69 * (x115 - x111 * x116 + x76 * x111) + x77 * x111) + x78 * x111) +
					 x74 * x111 + x79 * x111)) +
		 x93 * x118);
	const GEN_FLT x121 = cos(gibPhase_1 + x84 - asin(x94)) * gibMag_1;
	const GEN_FLT x122 = 2 * x42;
	const GEN_FLT x123 = x4 * x51 * x122;
	const GEN_FLT x124 = x56 * (x47 * x123 - x53 * x34);
	const GEN_FLT x125 = x98 * x49;
	const GEN_FLT x126 = x47 * x125 + x99 * x33;
	const GEN_FLT x127 = x102 * x126 - x82 * x59;
	const GEN_FLT x128 = x105 * (x124 - x96 * x127);
	const GEN_FLT x129 = 2 * x62;
	const GEN_FLT x130 = x108 * (-x110 * (x126 + x59 * x129) + x68 * x59);
	const GEN_FLT x131 = x71 * x130;
	const GEN_FLT x132 = x69 * (x131 - x70 * x130) + x72 * x130;
	const GEN_FLT x133 = x69 * x132 + x73 * x130;
	const GEN_FLT x134 =
		x95 *
		(x127 + x106 * x128 + x112 * x130 -
		 x119 *
			 (x113 * x128 +
			  x88 * (x69 * x133 +
					 x69 * (x133 + x69 * (x132 + x69 * (x131 - x116 * x130 + x76 * x130) + x77 * x130) + x78 * x130) +
					 x74 * x130 + x79 * x130)) +
		 x93 * x133);
	const GEN_FLT x135 = x56 * (x43 * x123 - x7 * x53);
	const GEN_FLT x136 = x43 * x125 + x7 * x122;
	const GEN_FLT x137 = x5 * x58;
	const GEN_FLT x138 = x102 * x136 - x82 * x137;
	const GEN_FLT x139 = x135 - x96 * x138;
	const GEN_FLT x140 = x108 * (-x110 * (x136 + x58 * x109) + x68 * x137);
	const GEN_FLT x141 = x71 * x140;
	const GEN_FLT x142 = x69 * (x141 - x70 * x140) + x72 * x140;
	const GEN_FLT x143 = x69 * x142 + x73 * x140;
	const GEN_FLT x144 =
		x95 *
		(x138 + x107 * x139 + x112 * x140 -
		 x119 *
			 (x114 * x139 +
			  x88 * (x69 * x143 +
					 x69 * (x143 + x69 * (x142 + x69 * (x141 - x116 * x140 + x76 * x140) + x77 * x140) + x78 * x140) +
					 x74 * x140 + x79 * x140)) +
		 x93 * x143);
	const GEN_FLT x145 = 2 / x13;
	const GEN_FLT x146 = x12 * x145;
	const GEN_FLT x147 = x146 * sensor_x;
	const GEN_FLT x148 = x36 * sensor_y;
	const GEN_FLT x149 = x145 * x148;
	const GEN_FLT x150 = x14 * obj_qk;
	const GEN_FLT x151 = x150 * sensor_y;
	const GEN_FLT x152 = x35 * sensor_z;
	const GEN_FLT x153 = x145 * x152;
	const GEN_FLT x154 = x14 * obj_qj;
	const GEN_FLT x155 = x154 * sensor_z;
	const GEN_FLT x156 = -x151 + x155 - x147 * obj_qw + x149 * obj_qw + x153 * obj_qw;
	const GEN_FLT x157 = x29 * sensor_x;
	const GEN_FLT x158 = x145 * x157;
	const GEN_FLT x159 = x26 * x145;
	const GEN_FLT x160 = x159 * sensor_y;
	const GEN_FLT x161 = x22 * obj_qk;
	const GEN_FLT x162 = x24 * sensor_z;
	const GEN_FLT x163 = x162 * x145;
	const GEN_FLT x164 = x14 * obj_qi;
	const GEN_FLT x165 = x164 * sensor_z;
	const GEN_FLT x166 = x161 - x165 + x158 * obj_qw - x160 * obj_qw + x163 * obj_qw;
	const GEN_FLT x167 = x154 * sensor_x;
	const GEN_FLT x168 = x21 * sensor_x;
	const GEN_FLT x169 = x168 * x145;
	const GEN_FLT x170 = x17 * sensor_y;
	const GEN_FLT x171 = x170 * x145;
	const GEN_FLT x172 = x164 * sensor_y;
	const GEN_FLT x173 = x10 * x145;
	const GEN_FLT x174 = x173 * sensor_z;
	const GEN_FLT x175 = -x167 + x172 + x169 * obj_qw + x171 * obj_qw - x174 * obj_qw;
	const GEN_FLT x176 = x5 * x43;
	const GEN_FLT x177 = x176 * x175 + x48 * x166 + x6 * x156;
	const GEN_FLT x178 = x34 * x166 + x41 * x156 + x7 * x175;
	const GEN_FLT x179 = (x52 * x177 - x53 * x178) * x56;
	const GEN_FLT x180 = x122 * x178 + x97 * x177;
	const GEN_FLT x181 = x175 * x137 + x59 * x166 + x61 * x156;
	const GEN_FLT x182 = x102 * x180 - x82 * x181;
	const GEN_FLT x183 = x179 - x96 * x182;
	const GEN_FLT x184 = x108 * (-x110 * (x180 + x129 * x181) + x68 * x181);
	const GEN_FLT x185 = x71 * x184;
	const GEN_FLT x186 = x69 * (x185 - x70 * x184) + x72 * x184;
	const GEN_FLT x187 = x69 * x186 + x73 * x184;
	const GEN_FLT x188 =
		x95 *
		(x182 + x107 * x183 + x112 * x184 -
		 x119 *
			 (x114 * x183 +
			  x88 * (x69 * x187 +
					 x69 * (x187 + x69 * (x186 + x69 * (x185 - x116 * x184 + x76 * x184) + x77 * x184) + x78 * x184) +
					 x74 * x184 + x79 * x184)) +
		 x93 * x187);
	const GEN_FLT x189 = x154 * sensor_y;
	const GEN_FLT x190 = x150 * sensor_z;
	const GEN_FLT x191 = x189 + x190 - x147 * obj_qi + x149 * obj_qi + x153 * obj_qi;
	const GEN_FLT x192 = 4 * x13;
	const GEN_FLT x193 = -x192 * obj_qi;
	const GEN_FLT x194 = x14 * obj_qw;
	const GEN_FLT x195 = x194 * sensor_z;
	const GEN_FLT x196 = x167 - x195 + x158 * obj_qi + x163 * obj_qi + (x193 - x159 * obj_qi) * sensor_y;
	const GEN_FLT x197 = x194 * sensor_y;
	const GEN_FLT x198 = x161 + x197 + x169 * obj_qi + x171 * obj_qi + (x193 - x173 * obj_qi) * sensor_z;
	const GEN_FLT x199 = x176 * x198 + x48 * x196 + x6 * x191;
	const GEN_FLT x200 = x34 * x196 + x41 * x191 + x7 * x198;
	const GEN_FLT x201 = (x52 * x199 - x53 * x200) * x56;
	const GEN_FLT x202 = x200 * x122 + x97 * x199;
	const GEN_FLT x203 = x198 * x137 + x59 * x196 + x61 * x191;
	const GEN_FLT x204 = x202 * x102 - x82 * x203;
	const GEN_FLT x205 = x201 - x96 * x204;
	const GEN_FLT x206 = x108 * (-x110 * (x202 + x203 * x129) + x68 * x203);
	const GEN_FLT x207 = x71 * x206;
	const GEN_FLT x208 = x69 * (x207 - x70 * x206) + x72 * x206;
	const GEN_FLT x209 = x69 * x208 + x73 * x206;
	const GEN_FLT x210 =
		x95 *
		(x204 -
		 x119 *
			 (x205 * x114 +
			  x88 * (x69 * x209 +
					 x69 * (x209 + x69 * (x208 + x69 * (x207 - x206 * x116 + x76 * x206) + x77 * x206) + x78 * x206) +
					 x74 * x206 + x79 * x206)) +
		 x205 * x107 + x206 * x112 + x93 * x209);
	const GEN_FLT x211 = -x192 * obj_qj;
	const GEN_FLT x212 = x145 * obj_qj;
	const GEN_FLT x213 = x172 + x195 + x212 * x148 + x212 * x152 + (x211 - x146 * obj_qj) * sensor_x;
	const GEN_FLT x214 = x22 * obj_qi;
	const GEN_FLT x215 = x190 + x214 - x160 * obj_qj + x212 * x157 + x212 * x162;
	const GEN_FLT x216 = x22 * obj_qw;
	const GEN_FLT x217 = x151 - x216 + x212 * x168 + x212 * x170 + (x211 - x173 * obj_qj) * sensor_z;
	const GEN_FLT x218 = x217 * x176 + x48 * x215 + x6 * x213;
	const GEN_FLT x219 = x34 * x215 + x41 * x213 + x7 * x217;
	const GEN_FLT x220 = (x52 * x218 - x53 * x219) * x56;
	const GEN_FLT x221 = x219 * x122 + x97 * x218;
	const GEN_FLT x222 = x217 * x137 + x59 * x215 + x61 * x213;
	const GEN_FLT x223 = x221 * x102 - x82 * x222;
	const GEN_FLT x224 = x220 - x96 * x223;
	const GEN_FLT x225 = x108 * (-x110 * (x221 + x222 * x129) + x68 * x222);
	const GEN_FLT x226 = x71 * x225;
	const GEN_FLT x227 = x69 * (x226 - x70 * x225) + x72 * x225;
	const GEN_FLT x228 = x69 * x227 + x73 * x225;
	const GEN_FLT x229 =
		x95 *
		(x223 -
		 x119 *
			 (x224 * x114 +
			  x88 * (x69 * x228 +
					 x69 * (x228 + x69 * (x227 + x69 * (x226 - x225 * x116 + x76 * x225) + x77 * x225) + x78 * x225) +
					 x74 * x225 + x79 * x225)) +
		 x224 * x107 + x225 * x112 + x93 * x228);
	const GEN_FLT x230 = x145 * obj_qk;
	const GEN_FLT x231 = -x192 * obj_qk;
	const GEN_FLT x232 = x165 - x197 + x230 * x148 + x230 * x152 + (x231 - x146 * obj_qk) * sensor_x;
	const GEN_FLT x233 = x155 + x216 + x163 * obj_qk + x230 * x157 + (x231 - x159 * obj_qk) * sensor_y;
	const GEN_FLT x234 = x5 * x233;
	const GEN_FLT x235 = x189 + x214 - x174 * obj_qk + x230 * x168 + x230 * x170;
	const GEN_FLT x236 = x235 * x176 + x47 * x234 + x6 * x232;
	const GEN_FLT x237 = x33 * x234 + x41 * x232 + x7 * x235;
	const GEN_FLT x238 = (x52 * x236 - x53 * x237) * x56;
	const GEN_FLT x239 = x237 * x122 + x97 * x236;
	const GEN_FLT x240 = x235 * x137 + x59 * x233 + x61 * x232;
	const GEN_FLT x241 = x239 * x102 - x82 * x240;
	const GEN_FLT x242 = x238 - x96 * x241;
	const GEN_FLT x243 = x108 * (-x110 * (x239 + x240 * x129) + x68 * x240);
	const GEN_FLT x244 = x71 * x243;
	const GEN_FLT x245 = x69 * (x244 - x70 * x243) + x72 * x243;
	const GEN_FLT x246 = x69 * x245 + x73 * x243;
	const GEN_FLT x247 =
		x95 *
		(x241 -
		 x119 *
			 (x242 * x114 +
			  x88 * (x69 * x246 +
					 x69 * (x246 + x69 * (x245 + x69 * (x244 - x243 * x116 + x76 * x243) + x77 * x243) + x78 * x243) +
					 x74 * x243 + x79 * x243)) +
		 x242 * x107 + x243 * x112 + x93 * x246);
	*(out++) = -x120 + x57 - x121 * (x120 - x57);
	*(out++) = x124 - x134 - (-x124 + x134) * x121;
	*(out++) = x135 - x144 - (-x135 + x144) * x121;
	*(out++) = x179 - x188 - (-x179 + x188) * x121;
	*(out++) = x201 - x210 - (-x201 + x210) * x121;
	*(out++) = x220 - x229 - (-x220 + x229) * x121;
	*(out++) = x238 - x247 - (-x238 + x247) * x121;
}

// Jacobian of reproject_axis_y_gen2 wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_reproject_axis_y_gen2_jac_sensor_pt(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
														   const SurvivePose *lh_p, const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qi, 2);
	const GEN_FLT x4 = sqrt(x2 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - x2 * x5;
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = pow(obj_qj, 2);
	const GEN_FLT x9 = pow(obj_qi, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = sqrt(x10 + x7 + pow(obj_qw, 2));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 - (x7 + x8) * x12;
	const GEN_FLT x14 = lh_qw * lh_qk;
	const GEN_FLT x15 = lh_qj * lh_qi;
	const GEN_FLT x16 = -x14 + x15;
	const GEN_FLT x17 = obj_qw * obj_qk;
	const GEN_FLT x18 = obj_qj * obj_qi;
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 4 * x4 * x11;
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = obj_qw * obj_qj;
	const GEN_FLT x23 = obj_qk * obj_qi;
	const GEN_FLT x24 = -x22 + x23;
	const GEN_FLT x25 = lh_qw * lh_qj;
	const GEN_FLT x26 = lh_qk * lh_qi;
	const GEN_FLT x27 = x25 + x26;
	const GEN_FLT x28 = x20 * x27;
	const GEN_FLT x29 = x21 * x16 + x24 * x28 + x6 * x13;
	const GEN_FLT x30 = 1 - (x1 + x3) * x5;
	const GEN_FLT x31 = 1 - x12 * x10;
	const GEN_FLT x32 = obj_qw * obj_qi;
	const GEN_FLT x33 = obj_qk * obj_qj;
	const GEN_FLT x34 = x32 + x33;
	const GEN_FLT x35 = x12 * sensor_y;
	const GEN_FLT x36 = x12 * sensor_x;
	const GEN_FLT x37 = obj_pz + x31 * sensor_z + x34 * x35 + x36 * x24;
	const GEN_FLT x38 = lh_qw * lh_qi;
	const GEN_FLT x39 = lh_qk * lh_qj;
	const GEN_FLT x40 = x38 + x39;
	const GEN_FLT x41 = -x32 + x33;
	const GEN_FLT x42 = x12 * sensor_z;
	const GEN_FLT x43 = 1 - (x7 + x9) * x12;
	const GEN_FLT x44 = obj_py + x36 * x19 + x41 * x42 + x43 * sensor_y;
	const GEN_FLT x45 = x5 * x44;
	const GEN_FLT x46 = -x25 + x26;
	const GEN_FLT x47 = x22 + x23;
	const GEN_FLT x48 = -x17 + x18;
	const GEN_FLT x49 = obj_px + x13 * sensor_x + x42 * x47 + x48 * x35;
	const GEN_FLT x50 = x5 * x49;
	const GEN_FLT x51 = lh_pz + x30 * x37 + x40 * x45 + x50 * x46;
	const GEN_FLT x52 = x5 * x37;
	const GEN_FLT x53 = lh_px + x45 * x16 + x52 * x27 + x6 * x49;
	const GEN_FLT x54 = pow(x53, 2);
	const GEN_FLT x55 = x51 / x54;
	const GEN_FLT x56 = pow(x53, -1);
	const GEN_FLT x57 = x5 * x13;
	const GEN_FLT x58 = x30 * x12;
	const GEN_FLT x59 = x40 * x21 + x57 * x46 + x58 * x24;
	const GEN_FLT x60 = x54 + pow(x51, 2);
	const GEN_FLT x61 = pow(x60, -1);
	const GEN_FLT x62 = x61 * x54;
	const GEN_FLT x63 = (x55 * x29 - x56 * x59) * x62;
	const GEN_FLT x64 = -x38 + x39;
	const GEN_FLT x65 = 1 - (x0 + x3) * x5;
	const GEN_FLT x66 = x14 + x15;
	const GEN_FLT x67 = lh_py + x64 * x52 + x65 * x44 + x66 * x50;
	const GEN_FLT x68 = 0.523598775598299 - tilt_1;
	const GEN_FLT x69 = cos(x68);
	const GEN_FLT x70 = pow(x69, -1);
	const GEN_FLT x71 = pow(x67, 2);
	const GEN_FLT x72 = x60 + x71;
	const GEN_FLT x73 = x70 / sqrt(x72);
	const GEN_FLT x74 = asin(x73 * x67);
	const GEN_FLT x75 = 8.0108022e-06 * x74;
	const GEN_FLT x76 = -8.0108022e-06 - x75;
	const GEN_FLT x77 = 0.0028679863 + x74 * x76;
	const GEN_FLT x78 = 5.3685255e-06 + x74 * x77;
	const GEN_FLT x79 = 0.0076069798 + x78 * x74;
	const GEN_FLT x80 = x79 * x74;
	const GEN_FLT x81 = -8.0108022e-06 - 1.60216044e-05 * x74;
	const GEN_FLT x82 = x77 + x81 * x74;
	const GEN_FLT x83 = x78 + x82 * x74;
	const GEN_FLT x84 = x79 + x83 * x74;
	const GEN_FLT x85 = x80 + x84 * x74;
	const GEN_FLT x86 = tan(x68);
	const GEN_FLT x87 = x86 / sqrt(x60);
	const GEN_FLT x88 = -x87 * x67;
	const GEN_FLT x89 = atan2(-x51, x53);
	const GEN_FLT x90 = ogeeMag_1 + x89 - asin(x88);
	const GEN_FLT x91 = curve_1 + sin(x90) * ogeePhase_1;
	const GEN_FLT x92 = sin(x68);
	const GEN_FLT x93 = x92 * x91;
	const GEN_FLT x94 = x69 + x85 * x93;
	const GEN_FLT x95 = pow(x94, -1);
	const GEN_FLT x96 = pow(x74, 2);
	const GEN_FLT x97 = x91 * x96;
	const GEN_FLT x98 = x97 * x95;
	const GEN_FLT x99 = x88 + x79 * x98;
	const GEN_FLT x100 = pow(1 - pow(x99, 2), -1.0 / 2.0);
	const GEN_FLT x101 = 2 * x53;
	const GEN_FLT x102 = 2 * x51;
	const GEN_FLT x103 = x29 * x101 + x59 * x102;
	const GEN_FLT x104 = (1.0 / 2.0) * x67;
	const GEN_FLT x105 = x86 * x104 / pow(x60, 3.0 / 2.0);
	const GEN_FLT x106 = x65 * x12;
	const GEN_FLT x107 = x64 * x20;
	const GEN_FLT x108 = x19 * x106 + x24 * x107 + x66 * x57;
	const GEN_FLT x109 = x103 * x105 - x87 * x108;
	const GEN_FLT x110 = pow(1 - pow(x86, 2) * x71 * x61, -1.0 / 2.0);
	const GEN_FLT x111 = cos(x90) * ogeePhase_1;
	const GEN_FLT x112 = x111 * (x63 - x109 * x110);
	const GEN_FLT x113 = x79 * x96 * x95;
	const GEN_FLT x114 = pow(1 - x71 / (x72 * pow(x69, 2)), -1.0 / 2.0);
	const GEN_FLT x115 = 2 * x67;
	const GEN_FLT x116 = x70 * x104 / pow(x72, 3.0 / 2.0);
	const GEN_FLT x117 = -x116 * (x103 + x108 * x115) + x73 * x108;
	const GEN_FLT x118 = x114 * x117;
	const GEN_FLT x119 = 2 * x80 * x91 * x95;
	const GEN_FLT x120 = x85 * x92;
	const GEN_FLT x121 = x76 * x118;
	const GEN_FLT x122 = 2.40324066e-05 * x74;
	const GEN_FLT x123 = x74 * (x121 - x75 * x118) + x77 * x118;
	const GEN_FLT x124 = x74 * x123 + x78 * x118;
	const GEN_FLT x125 = x79 * x114;
	const GEN_FLT x126 = x84 * x114;
	const GEN_FLT x127 = x79 * x97 / pow(x94, 2);
	const GEN_FLT x128 =
		x100 *
		(x109 + x112 * x113 + x118 * x119 -
		 x127 *
			 (x112 * x120 +
			  x93 * (x117 * x125 + x117 * x126 + x74 * x124 +
					 x74 * (x124 + x74 * (x123 + x74 * (x121 - x118 * x122 + x81 * x118) + x82 * x118) + x83 * x118))) +
		 x98 * x124);
	const GEN_FLT x129 = cos(gibPhase_1 + x89 - asin(x99)) * gibMag_1;
	const GEN_FLT x130 = x5 * x43;
	const GEN_FLT x131 = x6 * x12;
	const GEN_FLT x132 = x16 * x130 + x34 * x28 + x48 * x131;
	const GEN_FLT x133 = x48 * x20;
	const GEN_FLT x134 = x40 * x130 + x46 * x133 + x58 * x34;
	const GEN_FLT x135 = (x55 * x132 - x56 * x134) * x62;
	const GEN_FLT x136 = x101 * x132 + x102 * x134;
	const GEN_FLT x137 = x34 * x107 + x65 * x43 + x66 * x133;
	const GEN_FLT x138 = x105 * x136 - x87 * x137;
	const GEN_FLT x139 = x135 - x110 * x138;
	const GEN_FLT x140 = x111 * x113;
	const GEN_FLT x141 = -x116 * (x136 + x115 * x137) + x73 * x137;
	const GEN_FLT x142 = x114 * x141;
	const GEN_FLT x143 = x111 * x120;
	const GEN_FLT x144 = x76 * x142;
	const GEN_FLT x145 = x74 * (x144 - x75 * x142) + x77 * x142;
	const GEN_FLT x146 = x74 * x145 + x78 * x142;
	const GEN_FLT x147 =
		x100 *
		(x138 + x119 * x142 -
		 x127 *
			 (x139 * x143 +
			  x93 * (x125 * x141 + x126 * x141 + x74 * x146 +
					 x74 * (x146 + x74 * (x145 + x74 * (x144 - x122 * x142 + x81 * x142) + x82 * x142) + x83 * x142))) +
		 x139 * x140 + x98 * x146);
	const GEN_FLT x148 = x41 * x20;
	const GEN_FLT x149 = x5 * x31;
	const GEN_FLT x150 = x16 * x148 + x27 * x149 + x47 * x131;
	const GEN_FLT x151 = x47 * x20;
	const GEN_FLT x152 = x30 * x31 + x40 * x148 + x46 * x151;
	const GEN_FLT x153 = (x55 * x150 - x56 * x152) * x62;
	const GEN_FLT x154 = x101 * x150 + x102 * x152;
	const GEN_FLT x155 = x41 * x106 + x64 * x149 + x66 * x151;
	const GEN_FLT x156 = x105 * x154 - x87 * x155;
	const GEN_FLT x157 = x153 - x110 * x156;
	const GEN_FLT x158 = -x116 * (x154 + x115 * x155) + x73 * x155;
	const GEN_FLT x159 = x114 * x158;
	const GEN_FLT x160 = x76 * x159;
	const GEN_FLT x161 = x74 * (x160 - x75 * x159) + x77 * x159;
	const GEN_FLT x162 = x74 * x161 + x78 * x159;
	const GEN_FLT x163 =
		x100 *
		(x156 + x119 * x159 -
		 x127 *
			 (x143 * x157 +
			  x93 * (x125 * x158 + x126 * x158 + x74 * x162 +
					 x74 * (x162 + x74 * (x161 + x74 * (x160 - x122 * x159 + x81 * x159) + x82 * x159) + x83 * x159))) +
		 x140 * x157 + x98 * x162);
	*(out++) = -x128 + x63 - x129 * (x128 - x63);
	*(out++) = x135 - x147 - (-x135 + x147) * x129;
	*(out++) = x153 - x163 - (-x153 + x163) * x129;
}

// Jacobian of reproject_axis_y_gen2 wrt [lh_px, lh_py, lh_pz, lh_qw, lh_qi, lh_qj, lh_qk]
static inline void gen_reproject_axis_y_gen2_jac_lh_p(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
													  const SurvivePose *lh_p, const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qk, 2);
	const GEN_FLT x4 = x1 + x3;
	const GEN_FLT x5 = sqrt(x0 + x4 + pow(lh_qw, 2));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = x7 + x8;
	const GEN_FLT x10 = pow(obj_qk, 2);
	const GEN_FLT x11 = 2 * sqrt(x10 + x9 + pow(obj_qw, 2));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 = obj_pz + (1 - x9 * x11) * sensor_z + (x12 + x13) * x14 + (-x15 + x16) * x17;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + (1 - x11 * (x10 + x8)) * sensor_y + (-x12 + x13) * x22 + (x23 + x24) * x17;
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = -x27 + x28;
	const GEN_FLT x30 = obj_px + (1 - x11 * (x10 + x7)) * sensor_x + (x15 + x16) * x22 + (-x23 + x24) * x14;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + x18 * (1 - x2 * x6) + x21 * x26 + x31 * x29;
	const GEN_FLT x33 = x27 + x28;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = -x35 + x36;
	const GEN_FLT x38 = x0 + x3;
	const GEN_FLT x39 = lh_px + x30 * (1 - x6 * x38) + x34 * x33 + x37 * x26;
	const GEN_FLT x40 = pow(x39, 2);
	const GEN_FLT x41 = x40 + pow(x32, 2);
	const GEN_FLT x42 = pow(x41, -1);
	const GEN_FLT x43 = x42 * x32;
	const GEN_FLT x44 = -x19 + x20;
	const GEN_FLT x45 = x35 + x36;
	const GEN_FLT x46 = lh_py + x25 * (1 - x4 * x6) + x44 * x34 + x45 * x31;
	const GEN_FLT x47 = 0.523598775598299 - tilt_1;
	const GEN_FLT x48 = cos(x47);
	const GEN_FLT x49 = pow(x48, -1);
	const GEN_FLT x50 = pow(x46, 2);
	const GEN_FLT x51 = x41 + x50;
	const GEN_FLT x52 = x49 / sqrt(x51);
	const GEN_FLT x53 = asin(x52 * x46);
	const GEN_FLT x54 = 8.0108022e-06 * x53;
	const GEN_FLT x55 = -8.0108022e-06 - x54;
	const GEN_FLT x56 = 0.0028679863 + x53 * x55;
	const GEN_FLT x57 = 5.3685255e-06 + x53 * x56;
	const GEN_FLT x58 = 0.0076069798 + x53 * x57;
	const GEN_FLT x59 = pow(x53, 2);
	const GEN_FLT x60 = tan(x47);
	const GEN_FLT x61 = x60 / sqrt(x41);
	const GEN_FLT x62 = -x61 * x46;
	const GEN_FLT x63 = atan2(-x32, x39);
	const GEN_FLT x64 = ogeeMag_1 + x63 - asin(x62);
	const GEN_FLT x65 = curve_1 + sin(x64) * ogeePhase_1;
	const GEN_FLT x66 = x53 * x58;
	const GEN_FLT x67 = -8.0108022e-06 - 1.60216044e-05 * x53;
	const GEN_FLT x68 = x56 + x67 * x53;
	const GEN_FLT x69 = x57 + x68 * x53;
	const GEN_FLT x70 = x58 + x69 * x53;
	const GEN_FLT x71 = x66 + x70 * x53;
	const GEN_FLT x72 = sin(x47);
	const GEN_FLT x73 = x72 * x65;
	const GEN_FLT x74 = x48 + x71 * x73;
	const GEN_FLT x75 = pow(x74, -1);
	const GEN_FLT x76 = x75 * x65;
	const GEN_FLT x77 = x76 * x59;
	const GEN_FLT x78 = x62 + x77 * x58;
	const GEN_FLT x79 = pow(1 - pow(x78, 2), -1.0 / 2.0);
	const GEN_FLT x80 = x46 * x39;
	const GEN_FLT x81 = x60 / pow(x41, 3.0 / 2.0);
	const GEN_FLT x82 = x80 * x81;
	const GEN_FLT x83 = pow(1 - pow(x60, 2) * x50 * x42, -1.0 / 2.0);
	const GEN_FLT x84 = cos(x64) * ogeePhase_1;
	const GEN_FLT x85 = x84 * (x43 - x82 * x83);
	const GEN_FLT x86 = x58 * x59;
	const GEN_FLT x87 = x86 * x75;
	const GEN_FLT x88 = x49 / pow(x51, 3.0 / 2.0);
	const GEN_FLT x89 = 2 * x46;
	const GEN_FLT x90 = pow(1 - x50 / (x51 * pow(x48, 2)), -1.0 / 2.0);
	const GEN_FLT x91 = x76 * x66;
	const GEN_FLT x92 = x91 * x90;
	const GEN_FLT x93 = x71 * x72;
	const GEN_FLT x94 = x55 * x90;
	const GEN_FLT x95 = x80 * x88;
	const GEN_FLT x96 = -x95 * x94;
	const GEN_FLT x97 = 2.40324066e-05 * x53;
	const GEN_FLT x98 = x90 * x95;
	const GEN_FLT x99 = x53 * (x96 + x54 * x98) - x56 * x98;
	const GEN_FLT x100 = x53 * x99 - x57 * x98;
	const GEN_FLT x101 = x86 * x65 / pow(x74, 2);
	const GEN_FLT x102 =
		x79 * (x82 -
			   x101 * (x73 * (x53 * x100 +
							  x53 * (x100 + x53 * (x99 + x53 * (x96 - x67 * x98 + x98 * x97) - x68 * x98) - x69 * x98) -
							  x58 * x98 - x70 * x98) +
					   x85 * x93) +
			   x77 * x100 + x85 * x87 - x88 * x89 * x92 * x39);
	const GEN_FLT x103 = cos(gibPhase_1 + x63 - asin(x78)) * gibMag_1;
	const GEN_FLT x104 = x84 * x87;
	const GEN_FLT x105 = x83 * x61;
	const GEN_FLT x106 = x90 * (x52 - x88 * x50);
	const GEN_FLT x107 = 2 * x91;
	const GEN_FLT x108 = x84 * x93;
	const GEN_FLT x109 = x55 * x106;
	const GEN_FLT x110 = x53 * (x109 - x54 * x106) + x56 * x106;
	const GEN_FLT x111 = x53 * x110 + x57 * x106;
	const GEN_FLT x112 =
		x79 *
		(-x61 -
		 x101 * (x108 * x105 +
				 x73 * (x53 * x111 +
						x53 * (x111 + x53 * (x110 + x53 * (x109 + x67 * x106 - x97 * x106) + x68 * x106) + x69 * x106) +
						x58 * x106 + x70 * x106)) +
		 x105 * x104 + x107 * x106 + x77 * x111);
	const GEN_FLT x113 = x42 * x39;
	const GEN_FLT x114 = -x113;
	const GEN_FLT x115 = x81 * x46;
	const GEN_FLT x116 = x32 * x115;
	const GEN_FLT x117 = x114 - x83 * x116;
	const GEN_FLT x118 = x88 * x46;
	const GEN_FLT x119 = x32 * x118;
	const GEN_FLT x120 = -x94 * x119;
	const GEN_FLT x121 = x90 * x119;
	const GEN_FLT x122 = x53 * (x120 + x54 * x121) - x56 * x121;
	const GEN_FLT x123 = x53 * x122 - x57 * x121;
	const GEN_FLT x124 = 2 * x32;
	const GEN_FLT x125 =
		x79 *
		(x116 -
		 x101 * (x108 * x117 +
				 x73 * (x53 * x123 +
						x53 * (x123 + x53 * (x122 + x53 * (x120 - x67 * x121 + x97 * x121) - x68 * x121) - x69 * x121) -
						x58 * x121 - x70 * x121)) +
		 x104 * x117 + x77 * x123 - x92 * x118 * x124);
	const GEN_FLT x126 = x30 * x38;
	const GEN_FLT x127 = 2 / x5;
	const GEN_FLT x128 = x127 * lh_qw;
	const GEN_FLT x129 = x37 * x25;
	const GEN_FLT x130 = x26 * lh_qk;
	const GEN_FLT x131 = x33 * x18;
	const GEN_FLT x132 = x34 * lh_qj;
	const GEN_FLT x133 = -x130 + x132 - x128 * x126 + x128 * x129 + x128 * x131;
	const GEN_FLT x134 = x32 / x40;
	const GEN_FLT x135 = pow(x39, -1);
	const GEN_FLT x136 = x30 * x29;
	const GEN_FLT x137 = x25 * x21;
	const GEN_FLT x138 = x26 * lh_qi;
	const GEN_FLT x139 = x31 * lh_qj;
	const GEN_FLT x140 = x2 * x127;
	const GEN_FLT x141 = x138 - x139 + x128 * x136 + x128 * x137 - x18 * x140 * lh_qw;
	const GEN_FLT x142 = x40 * x42;
	const GEN_FLT x143 = (x133 * x134 - x135 * x141) * x142;
	const GEN_FLT x144 = 2 * x39;
	const GEN_FLT x145 = x124 * x141 + x133 * x144;
	const GEN_FLT x146 = (1.0 / 2.0) * x115;
	const GEN_FLT x147 = x45 * x30;
	const GEN_FLT x148 = x31 * lh_qk;
	const GEN_FLT x149 = x4 * x25;
	const GEN_FLT x150 = x44 * x18;
	const GEN_FLT x151 = x34 * lh_qi;
	const GEN_FLT x152 = x148 - x151 + x128 * x147 - x128 * x149 + x128 * x150;
	const GEN_FLT x153 = x146 * x145 - x61 * x152;
	const GEN_FLT x154 = x143 - x83 * x153;
	const GEN_FLT x155 = (1.0 / 2.0) * x118;
	const GEN_FLT x156 = x90 * (-x155 * (x145 + x89 * x152) + x52 * x152);
	const GEN_FLT x157 = x55 * x156;
	const GEN_FLT x158 = x53 * (x157 - x54 * x156) + x56 * x156;
	const GEN_FLT x159 = x53 * x158 + x57 * x156;
	const GEN_FLT x160 =
		x79 *
		(x153 -
		 x101 * (x108 * x154 +
				 x73 * (x53 * x159 +
						x53 * (x159 + x53 * (x158 + x53 * (x157 + x67 * x156 - x97 * x156) + x68 * x156) + x69 * x156) +
						x58 * x156 + x70 * x156)) +
		 x104 * x154 + x107 * x156 + x77 * x159);
	const GEN_FLT x161 = x127 * lh_qi;
	const GEN_FLT x162 = x26 * lh_qj;
	const GEN_FLT x163 = x34 * lh_qk;
	const GEN_FLT x164 = x162 + x163 - x126 * x161 + x129 * x161 + x161 * x131;
	const GEN_FLT x165 = x26 * lh_qw;
	const GEN_FLT x166 = 4 * x5;
	const GEN_FLT x167 = -x166 * lh_qi;
	const GEN_FLT x168 = x148 + x165 + x161 * x136 + x161 * x137 + x18 * (x167 - x140 * lh_qi);
	const GEN_FLT x169 = (x164 * x134 - x168 * x135) * x142;
	const GEN_FLT x170 = x124 * x168 + x164 * x144;
	const GEN_FLT x171 = x34 * lh_qw;
	const GEN_FLT x172 = x139 - x171 + x161 * x147 + x161 * x150 + x25 * (x167 - x4 * x161);
	const GEN_FLT x173 = x170 * x146 - x61 * x172;
	const GEN_FLT x174 = x169 - x83 * x173;
	const GEN_FLT x175 = x90 * (-x155 * (x170 + x89 * x172) + x52 * x172);
	const GEN_FLT x176 = x55 * x175;
	const GEN_FLT x177 = x53 * (x176 - x54 * x175) + x56 * x175;
	const GEN_FLT x178 = x53 * x177 + x57 * x175;
	const GEN_FLT x179 =
		x79 *
		(x173 -
		 x101 * (x108 * x174 +
				 x73 * (x53 * x178 +
						x53 * (x178 + x53 * (x177 + x53 * (x176 + x67 * x175 - x97 * x175) + x68 * x175) + x69 * x175) +
						x58 * x175 + x70 * x175)) +
		 x104 * x174 + x107 * x175 + x77 * x178);
	const GEN_FLT x180 = x127 * lh_qj;
	const GEN_FLT x181 = -x166 * lh_qj;
	const GEN_FLT x182 = x138 + x171 + x129 * x180 + x180 * x131 + x30 * (x181 - x38 * x180);
	const GEN_FLT x183 = x31 * lh_qw;
	const GEN_FLT x184 = x130 - x183 + x18 * (x181 - x140 * lh_qj) + x180 * x136 + x180 * x137;
	const GEN_FLT x185 = (x182 * x134 - x184 * x135) * x142;
	const GEN_FLT x186 = x124 * x184 + x182 * x144;
	const GEN_FLT x187 = x31 * lh_qi;
	const GEN_FLT x188 = x163 + x187 + x180 * x147 - x180 * x149 + x180 * x150;
	const GEN_FLT x189 = x186 * x146 - x61 * x188;
	const GEN_FLT x190 = x185 - x83 * x189;
	const GEN_FLT x191 = x90 * (-x155 * (x186 + x89 * x188) + x52 * x188);
	const GEN_FLT x192 = x55 * x191;
	const GEN_FLT x193 = x53 * (x192 - x54 * x191) + x56 * x191;
	const GEN_FLT x194 = x53 * x193 + x57 * x191;
	const GEN_FLT x195 =
		x79 *
		(x189 -
		 x101 * (x108 * x190 +
				 x73 * (x53 * x194 +
						x53 * (x194 + x53 * (x193 + x53 * (x192 + x67 * x191 - x97 * x191) + x68 * x191) + x69 * x191) +
						x58 * x191 + x70 * x191)) +
		 x104 * x190 + x107 * x191 + x77 * x194);
	const GEN_FLT x196 = x127 * lh_qk;
	const GEN_FLT x197 = -x166 * lh_qk;
	const GEN_FLT x198 = x151 - x165 + x129 * x196 + x196 * x131 + x30 * (x197 - x38 * x196);
	const GEN_FLT x199 = x162 + x187 + x196 * x136 + x196 * x137 - x2 * x18 * x196;
	const GEN_FLT x200 = (x198 * x134 - x199 * x135) * x142;
	const GEN_FLT x201 = x124 * x199 + x198 * x144;
	const GEN_FLT x202 = x132 + x183 + x196 * x147 + x196 * x150 + x25 * (x197 - x4 * x196);
	const GEN_FLT x203 = x201 * x146 - x61 * x202;
	const GEN_FLT x204 = x200 - x83 * x203;
	const GEN_FLT x205 = x90 * (-x155 * (x201 + x89 * x202) + x52 * x202);
	const GEN_FLT x206 = x55 * x205;
	const GEN_FLT x207 = x53 * (x206 - x54 * x205) + x56 * x205;
	const GEN_FLT x208 = x53 * x207 + x57 * x205;
	const GEN_FLT x209 =
		x79 *
		(x203 -
		 x101 * (x204 * x108 +
				 x73 * (x53 * x208 +
						x53 * (x208 + x53 * (x207 + x53 * (x206 + x67 * x205 - x97 * x205) + x68 * x205) + x69 * x205) +
						x58 * x205 + x70 * x205)) +
		 x204 * x104 + x205 * x107 + x77 * x208);
	*(out++) = -x102 + x43 - x103 * (x102 - x43);
	*(out++) = -x112 - x103 * x112;
	*(out++) = x114 - x125 - (x113 + x125) * x103;
	*(out++) = x143 - x160 - (-x143 + x160) * x103;
	*(out++) = x169 - x179 - (-x169 + x179) * x103;
	*(out++) = x185 - x195 - (-x185 + x195) * x103;
	*(out++) = x200 - x209 - (-x200 + x209) * x103;
}

// Jacobian of reproject_axis_y_gen2 wrt [phase_1, tilt_1, curve_1, gibPhase_1, gibMag_1, ogeeMag_1, ogeePhase_1]
static inline void gen_reproject_axis_y_gen2_jac_bsc1(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
													  const SurvivePose *lh_p, const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = 0.523598775598299 - tilt_1;
	const GEN_FLT x1 = tan(x0);
	const GEN_FLT x2 = lh_qw * lh_qi;
	const GEN_FLT x3 = lh_qk * lh_qj;
	const GEN_FLT x4 = pow(obj_qj, 2);
	const GEN_FLT x5 = pow(obj_qi, 2);
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = 2 * sqrt(x6 + x7 + pow(obj_qw, 2));
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = obj_pz + x11 * (x10 + x9) + (1 - x6 * x8) * sensor_z + (-x12 + x13) * x14;
	const GEN_FLT x16 = pow(lh_qi, 2);
	const GEN_FLT x17 = pow(lh_qk, 2);
	const GEN_FLT x18 = pow(lh_qj, 2);
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 2 * sqrt(x16 + x19 + pow(lh_qw, 2));
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + x22 * (x10 - x9) + (1 - (x5 + x7) * x8) * sensor_y + (x23 + x24) * x14;
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 = obj_px + (1 - (x4 + x7) * x8) * sensor_x + (x12 + x13) * x22 + (-x23 + x24) * x11;
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = lh_py + x25 * (1 - (x16 + x17) * x20) + (-x2 + x3) * x21 + (x26 + x27) * x29;
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = lh_pz + x15 * (1 - (x16 + x18) * x20) + (x2 + x3) * x31 + (-x32 + x33) * x29;
	const GEN_FLT x35 = lh_px + x28 * (1 - x20 * x19) + (-x26 + x27) * x31 + (x32 + x33) * x21;
	const GEN_FLT x36 = pow(x34, 2) + pow(x35, 2);
	const GEN_FLT x37 = x30 / sqrt(x36);
	const GEN_FLT x38 = -x1 * x37;
	const GEN_FLT x39 = atan2(-x34, x35);
	const GEN_FLT x40 = ogeeMag_1 + x39 - asin(x38);
	const GEN_FLT x41 = sin(x40);
	const GEN_FLT x42 = curve_1 + x41 * ogeePhase_1;
	const GEN_FLT x43 = cos(x0);
	const GEN_FLT x44 = pow(x30, 2);
	const GEN_FLT x45 = x36 + x44;
	const GEN_FLT x46 = x30 / sqrt(x45);
	const GEN_FLT x47 = asin(x46 / x43);
	const GEN_FLT x48 = 8.0108022e-06 * x47;
	const GEN_FLT x49 = -8.0108022e-06 - x48;
	const GEN_FLT x50 = 0.0028679863 + x47 * x49;
	const GEN_FLT x51 = 5.3685255e-06 + x50 * x47;
	const GEN_FLT x52 = 0.0076069798 + x51 * x47;
	const GEN_FLT x53 = pow(x47, 2);
	const GEN_FLT x54 = x52 * x47;
	const GEN_FLT x55 = -8.0108022e-06 - 1.60216044e-05 * x47;
	const GEN_FLT x56 = x50 + x55 * x47;
	const GEN_FLT x57 = x51 + x56 * x47;
	const GEN_FLT x58 = x52 + x57 * x47;
	const GEN_FLT x59 = x54 + x58 * x47;
	const GEN_FLT x60 = sin(x0);
	const GEN_FLT x61 = x60 * x42;
	const GEN_FLT x62 = x61 * x59;
	const GEN_FLT x63 = x43 + x62;
	const GEN_FLT x64 = pow(x63, -1);
	const GEN_FLT x65 = x64 * x53;
	const GEN_FLT x66 = x65 * x52;
	const GEN_FLT x67 = x38 + x66 * x42;
	const GEN_FLT x68 = pow(1 - pow(x67, 2), -1.0 / 2.0);
	const GEN_FLT x69 = pow(x1, 2);
	const GEN_FLT x70 = x37 * (1 + x69);
	const GEN_FLT x71 = cos(x40) * ogeePhase_1;
	const GEN_FLT x72 = x71 * x66;
	const GEN_FLT x73 = x70 / sqrt(1 - x69 * x44 / x36);
	const GEN_FLT x74 = pow(x43, -2);
	const GEN_FLT x75 = x74 * x46 / sqrt(1 - x74 * x44 / x45);
	const GEN_FLT x76 = x75 * x60;
	const GEN_FLT x77 = -x76 * x49;
	const GEN_FLT x78 = x47 * (x77 + x76 * x48) - x76 * x50;
	const GEN_FLT x79 = -x76 * x51 + x78 * x47;
	const GEN_FLT x80 = x53 * x52 / pow(x63, 2);
	const GEN_FLT x81 =
		x68 * (x70 - x73 * x72 + x79 * x65 * x42 -
			   x80 * x42 *
				   (x60 +
					x61 * (x47 * (x79 + x47 * (x78 + x47 * (x77 + 2.40324066e-05 * x76 * x47 - x76 * x55) - x76 * x56) -
								  x76 * x57) -
						   x76 * x52 - x76 * x58 + x79 * x47) -
					x59 * x42 * x43 - x71 * x73 * x60 * x59) -
			   2 * x75 * x64 * x61 * x54);
	const GEN_FLT x82 = -gibPhase_1 - x39 + asin(x67);
	const GEN_FLT x83 = cos(x82) * gibMag_1;
	const GEN_FLT x84 = x80 * x62;
	const GEN_FLT x85 = (x66 - x84) * x68;
	const GEN_FLT x86 = x68 * (x72 - x84 * x71);
	const GEN_FLT x87 = (x66 * x41 - x84 * x41) * x68;
	*(out++) = -1;
	*(out++) = -x81 - x81 * x83;
	*(out++) = -x85 - x83 * x85;
	*(out++) = x83;
	*(out++) = -sin(x82);
	*(out++) = -x86 - x83 * x86;
	*(out++) = -x87 - x83 * x87;
}

/** Applying function <function reproject at 0x7ffa3c1c7320> */
static inline void gen_reproject(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt, const SurvivePose *lh_p,
								 const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = lh_qw * lh_qi;
	const GEN_FLT x1 = lh_qk * lh_qj;
	const GEN_FLT x2 = pow(obj_qj, 2);
	const GEN_FLT x3 = pow(obj_qi, 2);
	const GEN_FLT x4 = x2 + x3;
	const GEN_FLT x5 = pow(obj_qk, 2);
	const GEN_FLT x6 = 2 * sqrt(x4 + x5 + pow(obj_qw, 2));
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x6 * sensor_y;
	const GEN_FLT x10 = obj_qw * obj_qj;
	const GEN_FLT x11 = obj_qk * obj_qi;
	const GEN_FLT x12 = x6 * sensor_x;
	const GEN_FLT x13 = obj_pz + (1 - x4 * x6) * sensor_z + (-x10 + x11) * x12 + (x7 + x8) * x9;
	const GEN_FLT x14 = pow(lh_qi, 2);
	const GEN_FLT x15 = pow(lh_qk, 2);
	const GEN_FLT x16 = pow(lh_qj, 2);
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = 2 * sqrt(x14 + x17 + pow(lh_qw, 2));
	const GEN_FLT x19 = x13 * x18;
	const GEN_FLT x20 = x6 * sensor_z;
	const GEN_FLT x21 = obj_qw * obj_qk;
	const GEN_FLT x22 = obj_qj * obj_qi;
	const GEN_FLT x23 = obj_py + (1 - (x3 + x5) * x6) * sensor_y + (x21 + x22) * x12 + (-x7 + x8) * x20;
	const GEN_FLT x24 = lh_qw * lh_qk;
	const GEN_FLT x25 = lh_qj * lh_qi;
	const GEN_FLT x26 = obj_px + (1 - (x2 + x5) * x6) * sensor_x + (x10 + x11) * x20 + (-x21 + x22) * x9;
	const GEN_FLT x27 = x26 * x18;
	const GEN_FLT x28 = lh_py + x23 * (1 - (x14 + x15) * x18) + (-x0 + x1) * x19 + (x24 + x25) * x27;
	const GEN_FLT x29 = x23 * x18;
	const GEN_FLT x30 = lh_qw * lh_qj;
	const GEN_FLT x31 = lh_qk * lh_qi;
	const GEN_FLT x32 = lh_pz + x13 * (1 - (x14 + x16) * x18) + (x0 + x1) * x29 + (-x30 + x31) * x27;
	const GEN_FLT x33 = -x32;
	const GEN_FLT x34 = pow(x32, 2);
	const GEN_FLT x35 = lh_px + x26 * (1 - x18 * x17) + (-x24 + x25) * x29 + (x30 + x31) * x19;
	const GEN_FLT x36 = atan2(x35, x33);
	const GEN_FLT x37 = -phase_0 - x36 - asin(x28 * tilt_0 / sqrt(x34 + pow(x35, 2)));
	const GEN_FLT x38 = -phase_1 - asin(x35 * tilt_1 / sqrt(x34 + pow(x28, 2))) - atan2(-x28, x33);
	*(out++) = x37 - cos(1.5707963267949 + gibPhase_0 + x37) * gibMag_0 + pow(atan2(x28, x33), 2) * curve_0;
	*(out++) = x38 + pow(x36, 2) * curve_1 - cos(1.5707963267949 + gibPhase_1 + x38) * gibMag_1;
}

// Jacobian of reproject wrt [obj_px, obj_py, obj_pz, obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_reproject_jac_obj_p(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
										   const SurvivePose *lh_p, const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qi, 2);
	const GEN_FLT x4 = sqrt(x2 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - x2 * x5;
	const GEN_FLT x7 = 1 - (x1 + x3) * x5;
	const GEN_FLT x8 = pow(obj_qj, 2);
	const GEN_FLT x9 = pow(obj_qi, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = sqrt(x10 + x11 + pow(obj_qw, 2));
	const GEN_FLT x13 = 2 * x12;
	const GEN_FLT x14 = obj_qw * obj_qi;
	const GEN_FLT x15 = obj_qk * obj_qj;
	const GEN_FLT x16 = x14 + x15;
	const GEN_FLT x17 = x13 * sensor_y;
	const GEN_FLT x18 = obj_qw * obj_qj;
	const GEN_FLT x19 = obj_qk * obj_qi;
	const GEN_FLT x20 = -x18 + x19;
	const GEN_FLT x21 = x13 * sensor_x;
	const GEN_FLT x22 = obj_pz + x17 * x16 + x20 * x21 + (1 - x13 * x10) * sensor_z;
	const GEN_FLT x23 = -x14 + x15;
	const GEN_FLT x24 = x13 * sensor_z;
	const GEN_FLT x25 = x11 + x9;
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x26 + x27;
	const GEN_FLT x29 = obj_py + x21 * x28 + x24 * x23 + (1 - x25 * x13) * sensor_y;
	const GEN_FLT x30 = lh_qw * lh_qi;
	const GEN_FLT x31 = lh_qk * lh_qj;
	const GEN_FLT x32 = x30 + x31;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = x18 + x19;
	const GEN_FLT x35 = -x26 + x27;
	const GEN_FLT x36 = x11 + x8;
	const GEN_FLT x37 = obj_px + x34 * x24 + x35 * x17 + (1 - x36 * x13) * sensor_x;
	const GEN_FLT x38 = lh_qw * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qi;
	const GEN_FLT x40 = -x38 + x39;
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = lh_pz + x33 * x29 + x41 * x37 + x7 * x22;
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = pow(x42, 2);
	const GEN_FLT x45 = pow(x44, -1);
	const GEN_FLT x46 = x38 + x39;
	const GEN_FLT x47 = x5 * x46;
	const GEN_FLT x48 = lh_qw * lh_qk;
	const GEN_FLT x49 = lh_qj * lh_qi;
	const GEN_FLT x50 = -x48 + x49;
	const GEN_FLT x51 = x5 * x50;
	const GEN_FLT x52 = lh_px + x47 * x22 + x51 * x29 + x6 * x37;
	const GEN_FLT x53 = x52 * x45;
	const GEN_FLT x54 = pow(x52, 2);
	const GEN_FLT x55 = x44 + x54;
	const GEN_FLT x56 = pow(x55, -1);
	const GEN_FLT x57 = x56 * x44;
	const GEN_FLT x58 = x57 * (x53 * x41 - x6 * x43);
	const GEN_FLT x59 = -x30 + x31;
	const GEN_FLT x60 = x5 * x59;
	const GEN_FLT x61 = 1 - (x0 + x3) * x5;
	const GEN_FLT x62 = x48 + x49;
	const GEN_FLT x63 = x5 * x62;
	const GEN_FLT x64 = lh_py + x60 * x22 + x61 * x29 + x63 * x37;
	const GEN_FLT x65 = pow(x64, 2);
	const GEN_FLT x66 = pow(1 - x65 * x56 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x67 = 2 * x52;
	const GEN_FLT x68 = 4 * x4;
	const GEN_FLT x69 = x68 * x42;
	const GEN_FLT x70 = x69 * x40;
	const GEN_FLT x71 = (1.0 / 2.0) * x64 * tilt_0 / pow(x55, 3.0 / 2.0);
	const GEN_FLT x72 = tilt_0 / sqrt(x55);
	const GEN_FLT x73 = -x58 - x66 * (-x71 * (x70 + x6 * x67) + x72 * x63);
	const GEN_FLT x74 = -x42;
	const GEN_FLT x75 = atan2(x52, x74);
	const GEN_FLT x76 = sin(1.5707963267949 + gibPhase_0 - phase_0 - x75 - asin(x72 * x64)) * gibMag_0;
	const GEN_FLT x77 = x63 * x43;
	const GEN_FLT x78 = x64 * x45;
	const GEN_FLT x79 = x78 * x41;
	const GEN_FLT x80 = x44 + x65;
	const GEN_FLT x81 = pow(x80, -1);
	const GEN_FLT x82 = x81 * x44;
	const GEN_FLT x83 = 2 * x82 * atan2(x64, x74) * curve_0;
	const GEN_FLT x84 = (-x51 * x43 + x53 * x33) * x57;
	const GEN_FLT x85 = x68 * x52;
	const GEN_FLT x86 = x69 * x32;
	const GEN_FLT x87 = -x84 - x66 * (-x71 * (x86 + x85 * x50) + x72 * x61);
	const GEN_FLT x88 = x61 * x43;
	const GEN_FLT x89 = x78 * x33;
	const GEN_FLT x90 = x57 * (-x43 * x47 + x7 * x53);
	const GEN_FLT x91 = 2 * x42;
	const GEN_FLT x92 = x7 * x91;
	const GEN_FLT x93 = -x90 - x66 * (-x71 * (x92 + x85 * x46) + x72 * x60);
	const GEN_FLT x94 = x60 * x43;
	const GEN_FLT x95 = x7 * x78;
	const GEN_FLT x96 = 2 / x12;
	const GEN_FLT x97 = x96 * x36;
	const GEN_FLT x98 = x97 * sensor_x;
	const GEN_FLT x99 = x96 * obj_qw;
	const GEN_FLT x100 = x35 * sensor_y;
	const GEN_FLT x101 = x17 * obj_qk;
	const GEN_FLT x102 = x34 * sensor_z;
	const GEN_FLT x103 = x24 * obj_qj;
	const GEN_FLT x104 = -x101 + x103 - x98 * obj_qw + x99 * x100 + x99 * x102;
	const GEN_FLT x105 = x99 * sensor_x;
	const GEN_FLT x106 = x96 * x25;
	const GEN_FLT x107 = x106 * sensor_y;
	const GEN_FLT x108 = x21 * obj_qk;
	const GEN_FLT x109 = x23 * sensor_z;
	const GEN_FLT x110 = x24 * obj_qi;
	const GEN_FLT x111 = x108 - x110 - x107 * obj_qw + x28 * x105 + x99 * x109;
	const GEN_FLT x112 = x21 * obj_qj;
	const GEN_FLT x113 = x16 * sensor_y;
	const GEN_FLT x114 = x17 * obj_qi;
	const GEN_FLT x115 = x96 * x10;
	const GEN_FLT x116 = x115 * sensor_z;
	const GEN_FLT x117 = -x112 + x114 - x116 * obj_qw + x20 * x105 + x99 * x113;
	const GEN_FLT x118 = x47 * x117 + x51 * x111 + x6 * x104;
	const GEN_FLT x119 = x33 * x111 + x41 * x104 + x7 * x117;
	const GEN_FLT x120 = (-x43 * x118 + x53 * x119) * x57;
	const GEN_FLT x121 = x91 * x119;
	const GEN_FLT x122 = x60 * x117 + x61 * x111 + x63 * x104;
	const GEN_FLT x123 = -x120 - x66 * (-x71 * (x121 + x67 * x118) + x72 * x122);
	const GEN_FLT x124 = x43 * x122;
	const GEN_FLT x125 = x78 * x119;
	const GEN_FLT x126 = x96 * obj_qi;
	const GEN_FLT x127 = x17 * obj_qj;
	const GEN_FLT x128 = x24 * obj_qk;
	const GEN_FLT x129 = x127 + x128 + x100 * x126 + x102 * x126 - x98 * obj_qi;
	const GEN_FLT x130 = x28 * sensor_x;
	const GEN_FLT x131 = 4 * x12;
	const GEN_FLT x132 = -x131 * obj_qi;
	const GEN_FLT x133 = x24 * obj_qw;
	const GEN_FLT x134 = x112 - x133 + x109 * x126 + x126 * x130 + (x132 - x106 * obj_qi) * sensor_y;
	const GEN_FLT x135 = x5 * x134;
	const GEN_FLT x136 = x20 * sensor_x;
	const GEN_FLT x137 = x96 * x113;
	const GEN_FLT x138 = x17 * obj_qw;
	const GEN_FLT x139 = x108 + x138 + x126 * x136 + x137 * obj_qi + (x132 - x115 * obj_qi) * sensor_z;
	const GEN_FLT x140 = x47 * x139 + x50 * x135 + x6 * x129;
	const GEN_FLT x141 = x32 * x135 + x41 * x129 + x7 * x139;
	const GEN_FLT x142 = (-x43 * x140 + x53 * x141) * x57;
	const GEN_FLT x143 = x91 * x141;
	const GEN_FLT x144 = x60 * x139 + x61 * x134 + x63 * x129;
	const GEN_FLT x145 = -x142 - x66 * (-x71 * (x143 + x67 * x140) + x72 * x144);
	const GEN_FLT x146 = x43 * x144;
	const GEN_FLT x147 = x78 * x141;
	const GEN_FLT x148 = -x131 * obj_qj;
	const GEN_FLT x149 = x96 * x100;
	const GEN_FLT x150 = x96 * obj_qj;
	const GEN_FLT x151 = x114 + x133 + x102 * x150 + x149 * obj_qj + (x148 - x97 * obj_qj) * sensor_x;
	const GEN_FLT x152 = x21 * obj_qi;
	const GEN_FLT x153 = x96 * x109;
	const GEN_FLT x154 = x128 + x152 - x107 * obj_qj + x130 * x150 + x153 * obj_qj;
	const GEN_FLT x155 = x21 * obj_qw;
	const GEN_FLT x156 = x101 - x155 + x136 * x150 + x137 * obj_qj + (x148 - x115 * obj_qj) * sensor_z;
	const GEN_FLT x157 = x47 * x156 + x51 * x154 + x6 * x151;
	const GEN_FLT x158 = x33 * x154 + x41 * x151 + x7 * x156;
	const GEN_FLT x159 = (-x43 * x157 + x53 * x158) * x57;
	const GEN_FLT x160 = x91 * x158;
	const GEN_FLT x161 = x60 * x156 + x61 * x154 + x63 * x151;
	const GEN_FLT x162 = -x159 - x66 * (-x71 * (x160 + x67 * x157) + x72 * x161);
	const GEN_FLT x163 = x43 * x161;
	const GEN_FLT x164 = x78 * x158;
	const GEN_FLT x165 = -x131 * obj_qk;
	const GEN_FLT x166 = x96 * obj_qk;
	const GEN_FLT x167 = x110 - x138 + x102 * x166 + x149 * obj_qk + (x165 - x97 * obj_qk) * sensor_x;
	const GEN_FLT x168 = x103 + x155 + x153 * obj_qk + x166 * x130 + (x165 - x106 * obj_qk) * sensor_y;
	const GEN_FLT x169 = x5 * x168;
	const GEN_FLT x170 = x127 + x152 - x116 * obj_qk + x137 * obj_qk + x166 * x136;
	const GEN_FLT x171 = x47 * x170 + x50 * x169 + x6 * x167;
	const GEN_FLT x172 = x32 * x169 + x41 * x167 + x7 * x170;
	const GEN_FLT x173 = (-x43 * x171 + x53 * x172) * x57;
	const GEN_FLT x174 = x91 * x172;
	const GEN_FLT x175 = x60 * x170 + x61 * x168 + x63 * x167;
	const GEN_FLT x176 = -x173 - x66 * (-x71 * (x174 + x67 * x171) + x72 * x175);
	const GEN_FLT x177 = x43 * x175;
	const GEN_FLT x178 = x78 * x172;
	const GEN_FLT x179 = 2 * x75 * curve_1;
	const GEN_FLT x180 = pow(1 - x81 * x54 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x181 = tilt_1 / sqrt(x80);
	const GEN_FLT x182 = x64 * x68;
	const GEN_FLT x183 = (1.0 / 2.0) * x52 * tilt_1 / pow(x80, 3.0 / 2.0);
	const GEN_FLT x184 = -x180 * (-x183 * (x70 + x62 * x182) + x6 * x181) - (x77 - x79) * x82;
	const GEN_FLT x185 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x52 * x181) - atan2(-x64, x74)) * gibMag_1;
	const GEN_FLT x186 = 2 * x64;
	const GEN_FLT x187 = -x180 * (-x183 * (x86 + x61 * x186) + x51 * x181) - (x88 - x89) * x82;
	const GEN_FLT x188 = -x180 * (-x183 * (x92 + x59 * x182) + x47 * x181) - (x94 - x95) * x82;
	const GEN_FLT x189 = -x180 * (x118 * x181 - x183 * (x121 + x122 * x186)) - (x124 - x125) * x82;
	const GEN_FLT x190 = -x180 * (x181 * x140 - x183 * (x143 + x186 * x144)) - (x146 - x147) * x82;
	const GEN_FLT x191 = -x180 * (x181 * x157 - x183 * (x160 + x161 * x186)) - (x163 - x164) * x82;
	const GEN_FLT x192 = -x180 * (x171 * x181 - x183 * (x174 + x175 * x186)) - (x177 - x178) * x82;
	*(out++) = x73 + x73 * x76 + (-x77 + x79) * x83;
	*(out++) = x87 + x87 * x76 + (-x88 + x89) * x83;
	*(out++) = x93 + x76 * x93 + (-x94 + x95) * x83;
	*(out++) = x123 + x76 * x123 + (-x124 + x125) * x83;
	*(out++) = x145 + x76 * x145 + (-x146 + x147) * x83;
	*(out++) = x162 + x76 * x162 + (-x163 + x164) * x83;
	*(out++) = x176 + x76 * x176 + (-x177 + x178) * x83;
	*(out++) = x184 + x185 * x184 + x58 * x179;
	*(out++) = x187 + x187 * x185 + x84 * x179;
	*(out++) = x188 + x188 * x185 + x90 * x179;
	*(out++) = x189 + x120 * x179 + x189 * x185;
	*(out++) = x190 + x179 * x142 + x185 * x190;
	*(out++) = x191 + x179 * x159 + x185 * x191;
	*(out++) = x192 + x179 * x173 + x185 * x192;
}

// Jacobian of reproject wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_reproject_jac_sensor_pt(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
											   const SurvivePose *lh_p, const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = pow(lh_qk, 2);
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt(x1 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - (x0 + x1) * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = pow(obj_qk, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = sqrt(x10 + x7 + pow(obj_qw, 2));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 - (x7 + x8) * x12;
	const GEN_FLT x14 = obj_qw * obj_qi;
	const GEN_FLT x15 = obj_qk * obj_qj;
	const GEN_FLT x16 = x14 + x15;
	const GEN_FLT x17 = x12 * x16;
	const GEN_FLT x18 = obj_qw * obj_qj;
	const GEN_FLT x19 = obj_qk * obj_qi;
	const GEN_FLT x20 = -x18 + x19;
	const GEN_FLT x21 = x20 * x12;
	const GEN_FLT x22 = obj_pz + x13 * sensor_z + x17 * sensor_y + x21 * sensor_x;
	const GEN_FLT x23 = -x14 + x15;
	const GEN_FLT x24 = x23 * x12;
	const GEN_FLT x25 = 1 - x12 * x10;
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x26 + x27;
	const GEN_FLT x29 = x28 * x12;
	const GEN_FLT x30 = obj_py + x24 * sensor_z + x25 * sensor_y + x29 * sensor_x;
	const GEN_FLT x31 = lh_qw * lh_qi;
	const GEN_FLT x32 = lh_qk * lh_qj;
	const GEN_FLT x33 = x31 + x32;
	const GEN_FLT x34 = x5 * x33;
	const GEN_FLT x35 = x18 + x19;
	const GEN_FLT x36 = -x26 + x27;
	const GEN_FLT x37 = 1 - (x7 + x9) * x12;
	const GEN_FLT x38 = obj_px + x37 * sensor_x + x35 * x12 * sensor_z + x36 * x12 * sensor_y;
	const GEN_FLT x39 = lh_qw * lh_qj;
	const GEN_FLT x40 = lh_qk * lh_qi;
	const GEN_FLT x41 = -x39 + x40;
	const GEN_FLT x42 = x5 * x41;
	const GEN_FLT x43 = lh_pz + x30 * x34 + x42 * x38 + x6 * x22;
	const GEN_FLT x44 = pow(x43, -1);
	const GEN_FLT x45 = 1 - x3 * x5;
	const GEN_FLT x46 = lh_qw * lh_qk;
	const GEN_FLT x47 = lh_qj * lh_qi;
	const GEN_FLT x48 = -x46 + x47;
	const GEN_FLT x49 = 4 * x4 * x11;
	const GEN_FLT x50 = x49 * x28;
	const GEN_FLT x51 = x39 + x40;
	const GEN_FLT x52 = x49 * x20;
	const GEN_FLT x53 = x45 * x37 + x50 * x48 + x52 * x51;
	const GEN_FLT x54 = x42 * x37 + x50 * x33 + x6 * x21;
	const GEN_FLT x55 = pow(x43, 2);
	const GEN_FLT x56 = pow(x55, -1);
	const GEN_FLT x57 = x5 * x22;
	const GEN_FLT x58 = x5 * x48;
	const GEN_FLT x59 = lh_px + x45 * x38 + x51 * x57 + x58 * x30;
	const GEN_FLT x60 = x56 * x59;
	const GEN_FLT x61 = pow(x59, 2);
	const GEN_FLT x62 = x55 + x61;
	const GEN_FLT x63 = pow(x62, -1);
	const GEN_FLT x64 = x63 * x55;
	const GEN_FLT x65 = (-x53 * x44 + x60 * x54) * x64;
	const GEN_FLT x66 = -x31 + x32;
	const GEN_FLT x67 = 1 - (x1 + x2) * x5;
	const GEN_FLT x68 = x46 + x47;
	const GEN_FLT x69 = x5 * x68;
	const GEN_FLT x70 = lh_py + x66 * x57 + x67 * x30 + x69 * x38;
	const GEN_FLT x71 = pow(x70, 2);
	const GEN_FLT x72 = pow(1 - x71 * x63 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x73 = 2 * x59;
	const GEN_FLT x74 = 2 * x43;
	const GEN_FLT x75 = x74 * x54;
	const GEN_FLT x76 = (1.0 / 2.0) * x70 * tilt_0 / pow(x62, 3.0 / 2.0);
	const GEN_FLT x77 = x66 * x52 + x67 * x29 + x69 * x37;
	const GEN_FLT x78 = tilt_0 / sqrt(x62);
	const GEN_FLT x79 = -x65 - x72 * (-x76 * (x75 + x73 * x53) + x78 * x77);
	const GEN_FLT x80 = -x43;
	const GEN_FLT x81 = atan2(x59, x80);
	const GEN_FLT x82 = sin(1.5707963267949 + gibPhase_0 - phase_0 - x81 - asin(x70 * x78)) * gibMag_0;
	const GEN_FLT x83 = x77 * x44;
	const GEN_FLT x84 = x70 * x56;
	const GEN_FLT x85 = x84 * x54;
	const GEN_FLT x86 = x55 + x71;
	const GEN_FLT x87 = pow(x86, -1);
	const GEN_FLT x88 = x87 * x55;
	const GEN_FLT x89 = 2 * x88 * atan2(x70, x80) * curve_0;
	const GEN_FLT x90 = x45 * x12;
	const GEN_FLT x91 = x49 * x16;
	const GEN_FLT x92 = x51 * x91 + x58 * x25 + x90 * x36;
	const GEN_FLT x93 = x49 * x36;
	const GEN_FLT x94 = x34 * x25 + x6 * x17 + x93 * x41;
	const GEN_FLT x95 = (x60 * x94 - x92 * x44) * x64;
	const GEN_FLT x96 = x74 * x94;
	const GEN_FLT x97 = x66 * x91 + x67 * x25 + x68 * x93;
	const GEN_FLT x98 = -x95 - x72 * (-x76 * (x96 + x73 * x92) + x78 * x97);
	const GEN_FLT x99 = x97 * x44;
	const GEN_FLT x100 = x84 * x94;
	const GEN_FLT x101 = x49 * x23;
	const GEN_FLT x102 = x5 * x13;
	const GEN_FLT x103 = x48 * x101 + x51 * x102 + x90 * x35;
	const GEN_FLT x104 = x49 * x35;
	const GEN_FLT x105 = x33 * x101 + x41 * x104 + x6 * x13;
	const GEN_FLT x106 = (-x44 * x103 + x60 * x105) * x64;
	const GEN_FLT x107 = x74 * x105;
	const GEN_FLT x108 = x66 * x102 + x67 * x24 + x68 * x104;
	const GEN_FLT x109 = -x106 - x72 * (-x76 * (x107 + x73 * x103) + x78 * x108);
	const GEN_FLT x110 = x44 * x108;
	const GEN_FLT x111 = x84 * x105;
	const GEN_FLT x112 = 2 * x81 * curve_1;
	const GEN_FLT x113 = pow(1 - x87 * x61 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x114 = tilt_1 / sqrt(x86);
	const GEN_FLT x115 = 2 * x70;
	const GEN_FLT x116 = (1.0 / 2.0) * x59 * tilt_1 / pow(x86, 3.0 / 2.0);
	const GEN_FLT x117 = -x113 * (-x116 * (x75 + x77 * x115) + x53 * x114) - (x83 - x85) * x88;
	const GEN_FLT x118 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x59 * x114) - atan2(-x70, x80)) * gibMag_1;
	const GEN_FLT x119 = -x113 * (-x116 * (x96 + x97 * x115) + x92 * x114) - x88 * (-x100 + x99);
	const GEN_FLT x120 = -x113 * (x103 * x114 - x116 * (x107 + x108 * x115)) - (x110 - x111) * x88;
	*(out++) = x79 + x82 * x79 + (-x83 + x85) * x89;
	*(out++) = x98 + x82 * x98 + x89 * (x100 - x99);
	*(out++) = x109 + x82 * x109 + (-x110 + x111) * x89;
	*(out++) = x117 + x118 * x117 + x65 * x112;
	*(out++) = x119 + x118 * x119 + x95 * x112;
	*(out++) = x120 + x106 * x112 + x118 * x120;
}

// Jacobian of reproject wrt [lh_px, lh_py, lh_pz, lh_qw, lh_qi, lh_qj, lh_qk]
static inline void gen_reproject_jac_lh_p(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
										  const SurvivePose *lh_p, const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qk, 2);
	const GEN_FLT x4 = x1 + x3;
	const GEN_FLT x5 = sqrt(x0 + x4 + pow(lh_qw, 2));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = pow(obj_qk, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = 2 * sqrt(x10 + x7 + pow(obj_qw, 2));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 = obj_pz + (1 - (x7 + x8) * x11) * sensor_z + (x12 + x13) * x14 + (-x15 + x16) * x17;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + (1 - x11 * x10) * sensor_y + (-x12 + x13) * x22 + (x23 + x24) * x17;
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = -x27 + x28;
	const GEN_FLT x30 = obj_px + (1 - (x7 + x9) * x11) * sensor_x + (x15 + x16) * x22 + (-x23 + x24) * x14;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + x18 * (1 - x2 * x6) + x21 * x26 + x31 * x29;
	const GEN_FLT x33 = x27 + x28;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = -x35 + x36;
	const GEN_FLT x38 = x0 + x3;
	const GEN_FLT x39 = lh_px + x30 * (1 - x6 * x38) + x34 * x33 + x37 * x26;
	const GEN_FLT x40 = pow(x39, 2);
	const GEN_FLT x41 = pow(x32, 2);
	const GEN_FLT x42 = x40 + x41;
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = x43 * x32;
	const GEN_FLT x45 = -x19 + x20;
	const GEN_FLT x46 = x35 + x36;
	const GEN_FLT x47 = lh_py + x25 * (1 - x4 * x6) + x45 * x34 + x46 * x31;
	const GEN_FLT x48 = x47 * x39;
	const GEN_FLT x49 = pow(x47, 2);
	const GEN_FLT x50 = pow(1 - x43 * x49 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x51 = tilt_0 / pow(x42, 3.0 / 2.0);
	const GEN_FLT x52 = x50 * x51;
	const GEN_FLT x53 = x44 + x52 * x48;
	const GEN_FLT x54 = tilt_0 / sqrt(x42);
	const GEN_FLT x55 = -x32;
	const GEN_FLT x56 = atan2(x39, x55);
	const GEN_FLT x57 = sin(1.5707963267949 + gibPhase_0 - phase_0 - x56 - asin(x54 * x47)) * gibMag_0;
	const GEN_FLT x58 = x50 * x54;
	const GEN_FLT x59 = x41 + x49;
	const GEN_FLT x60 = pow(x59, -1);
	const GEN_FLT x61 = x60 * x32;
	const GEN_FLT x62 = 2 * atan2(x47, x55) * curve_0;
	const GEN_FLT x63 = x43 * x39;
	const GEN_FLT x64 = -x63 + x52 * x47 * x32;
	const GEN_FLT x65 = x60 * x47;
	const GEN_FLT x66 = pow(x32, -1);
	const GEN_FLT x67 = 2 / x5;
	const GEN_FLT x68 = x67 * x38;
	const GEN_FLT x69 = x30 * lh_qw;
	const GEN_FLT x70 = x67 * x25;
	const GEN_FLT x71 = x70 * x37;
	const GEN_FLT x72 = x26 * lh_qk;
	const GEN_FLT x73 = x67 * x18;
	const GEN_FLT x74 = x73 * x33;
	const GEN_FLT x75 = x34 * lh_qj;
	const GEN_FLT x76 = -x72 + x75 - x68 * x69 + x71 * lh_qw + x74 * lh_qw;
	const GEN_FLT x77 = x67 * x29;
	const GEN_FLT x78 = x70 * x21;
	const GEN_FLT x79 = x26 * lh_qi;
	const GEN_FLT x80 = x31 * lh_qj;
	const GEN_FLT x81 = x2 * x67;
	const GEN_FLT x82 = x81 * x18;
	const GEN_FLT x83 = x79 - x80 + x77 * x69 + x78 * lh_qw - x82 * lh_qw;
	const GEN_FLT x84 = pow(x41, -1);
	const GEN_FLT x85 = x84 * x39;
	const GEN_FLT x86 = x41 * x43;
	const GEN_FLT x87 = (-x76 * x66 + x83 * x85) * x86;
	const GEN_FLT x88 = 2 * x39;
	const GEN_FLT x89 = 2 * x32;
	const GEN_FLT x90 = x83 * x89;
	const GEN_FLT x91 = (1.0 / 2.0) * x51 * x47;
	const GEN_FLT x92 = x67 * x46;
	const GEN_FLT x93 = x31 * lh_qk;
	const GEN_FLT x94 = x4 * x67;
	const GEN_FLT x95 = x94 * x25;
	const GEN_FLT x96 = x73 * x45;
	const GEN_FLT x97 = x34 * lh_qi;
	const GEN_FLT x98 = x93 - x97 + x69 * x92 - x95 * lh_qw + x96 * lh_qw;
	const GEN_FLT x99 = -x87 - x50 * (x54 * x98 - x91 * (x90 + x88 * x76));
	const GEN_FLT x100 = x66 * x98;
	const GEN_FLT x101 = x84 * x47;
	const GEN_FLT x102 = x83 * x101;
	const GEN_FLT x103 = x60 * x41;
	const GEN_FLT x104 = x62 * x103;
	const GEN_FLT x105 = x30 * lh_qi;
	const GEN_FLT x106 = x26 * lh_qj;
	const GEN_FLT x107 = x73 * lh_qi;
	const GEN_FLT x108 = x34 * lh_qk;
	const GEN_FLT x109 = x106 + x108 + x33 * x107 - x68 * x105 + x71 * lh_qi;
	const GEN_FLT x110 = x26 * lh_qw;
	const GEN_FLT x111 = 4 * x5;
	const GEN_FLT x112 = -x111 * lh_qi;
	const GEN_FLT x113 = x110 + x93 + x18 * (x112 - x81 * lh_qi) + x77 * x105 + x78 * lh_qi;
	const GEN_FLT x114 = (-x66 * x109 + x85 * x113) * x86;
	const GEN_FLT x115 = x89 * x113;
	const GEN_FLT x116 = x34 * lh_qw;
	const GEN_FLT x117 = -x116 + x80 + x25 * (x112 - x94 * lh_qi) + x45 * x107 + x92 * x105;
	const GEN_FLT x118 = -x114 - x50 * (x54 * x117 - x91 * (x115 + x88 * x109));
	const GEN_FLT x119 = x66 * x117;
	const GEN_FLT x120 = x101 * x113;
	const GEN_FLT x121 = -x111 * lh_qj;
	const GEN_FLT x122 = x116 + x79 + x30 * (x121 - x68 * lh_qj) + x71 * lh_qj + x74 * lh_qj;
	const GEN_FLT x123 = x30 * lh_qj;
	const GEN_FLT x124 = x31 * lh_qw;
	const GEN_FLT x125 = -x124 + x72 + x18 * (x121 - x81 * lh_qj) + x77 * x123 + x78 * lh_qj;
	const GEN_FLT x126 = (-x66 * x122 + x85 * x125) * x86;
	const GEN_FLT x127 = x89 * x125;
	const GEN_FLT x128 = x31 * lh_qi;
	const GEN_FLT x129 = x108 + x128 + x92 * x123 - x95 * lh_qj + x96 * lh_qj;
	const GEN_FLT x130 = -x126 - x50 * (x54 * x129 - x91 * (x127 + x88 * x122));
	const GEN_FLT x131 = x66 * x129;
	const GEN_FLT x132 = x101 * x125;
	const GEN_FLT x133 = -x111 * lh_qk;
	const GEN_FLT x134 = -x110 + x97 + x30 * (x133 - x68 * lh_qk) + x71 * lh_qk + x74 * lh_qk;
	const GEN_FLT x135 = x30 * lh_qk;
	const GEN_FLT x136 = x106 + x128 + x77 * x135 + x78 * lh_qk - x82 * lh_qk;
	const GEN_FLT x137 = (-x66 * x134 + x85 * x136) * x86;
	const GEN_FLT x138 = x89 * x136;
	const GEN_FLT x139 = x124 + x75 + x25 * (x133 - x94 * lh_qk) + x92 * x135 + x96 * lh_qk;
	const GEN_FLT x140 = -x137 - x50 * (x54 * x139 - x91 * (x138 + x88 * x134));
	const GEN_FLT x141 = x66 * x139;
	const GEN_FLT x142 = x101 * x136;
	const GEN_FLT x143 = pow(1 - x60 * x40 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x144 = tilt_1 / sqrt(x59);
	const GEN_FLT x145 = x144 * x143;
	const GEN_FLT x146 = 2 * x56 * curve_1;
	const GEN_FLT x147 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x39 * x144) - atan2(-x47, x55)) * gibMag_1;
	const GEN_FLT x148 = tilt_1 / pow(x59, 3.0 / 2.0);
	const GEN_FLT x149 = x143 * x148;
	const GEN_FLT x150 = -x61 + x48 * x149;
	const GEN_FLT x151 = x65 + x32 * x39 * x149;
	const GEN_FLT x152 = 2 * x47;
	const GEN_FLT x153 = (1.0 / 2.0) * x39 * x148;
	const GEN_FLT x154 = -x143 * (-x153 * (x90 + x98 * x152) + x76 * x144) - (x100 - x102) * x103;
	const GEN_FLT x155 = -x143 * (x109 * x144 - x153 * (x115 + x117 * x152)) - (x119 - x120) * x103;
	const GEN_FLT x156 = -x143 * (x122 * x144 - x153 * (x127 + x129 * x152)) - (x131 - x132) * x103;
	const GEN_FLT x157 = -x143 * (x134 * x144 - x153 * (x138 + x139 * x152)) - (x141 - x142) * x103;
	*(out++) = x53 + x53 * x57;
	*(out++) = -x58 - x58 * x57 - x61 * x62;
	*(out++) = x64 + x62 * x65 + x64 * x57;
	*(out++) = x99 + x57 * x99 + (-x100 + x102) * x104;
	*(out++) = x118 + x57 * x118 + (-x119 + x120) * x104;
	*(out++) = x130 + x57 * x130 + (-x131 + x132) * x104;
	*(out++) = x140 + x57 * x140 + (-x141 + x142) * x104;
	*(out++) = -x145 - x145 * x147 - x44 * x146;
	*(out++) = x150 + x147 * x150;
	*(out++) = x151 + x147 * x151 + x63 * x146;
	*(out++) = x154 + x147 * x154 + x87 * x146;
	*(out++) = x155 + x114 * x146 + x147 * x155;
	*(out++) = x156 + x126 * x146 + x147 * x156;
	*(out++) = x157 + x137 * x146 + x147 * x157;
}

// Jacobian of reproject wrt [phase_0, tilt_0, curve_0, gibPhase_0, gibMag_0, ogeeMag_0, ogeePhase_0, phase_1, tilt_1,
// curve_1, gibPhase_1, gibMag_1, ogeeMag_1, ogeePhase_1]
static inline void gen_reproject_jac_bsd(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
										 const SurvivePose *lh_p, const BaseStationCal *bsd) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = bsd[0].phase;
	const GEN_FLT tilt_0 = bsd[0].tilt;
	const GEN_FLT curve_0 = bsd[0].curve;
	const GEN_FLT gibPhase_0 = bsd[0].gibpha;
	const GEN_FLT gibMag_0 = bsd[0].gibmag;
	const GEN_FLT ogeeMag_0 = bsd[0].ogeephase;
	const GEN_FLT ogeePhase_0 = bsd[0].ogeemag;
	const GEN_FLT phase_1 = bsd[1].phase;
	const GEN_FLT tilt_1 = bsd[1].tilt;
	const GEN_FLT curve_1 = bsd[1].curve;
	const GEN_FLT gibPhase_1 = bsd[1].gibpha;
	const GEN_FLT gibMag_1 = bsd[1].gibmag;
	const GEN_FLT ogeeMag_1 = bsd[1].ogeephase;
	const GEN_FLT ogeePhase_1 = bsd[1].ogeemag;
	const GEN_FLT x0 = lh_qw * lh_qi;
	const GEN_FLT x1 = lh_qk * lh_qj;
	const GEN_FLT x2 = pow(obj_qj, 2);
	const GEN_FLT x3 = pow(obj_qi, 2);
	const GEN_FLT x4 = x2 + x3;
	const GEN_FLT x5 = pow(obj_qk, 2);
	const GEN_FLT x6 = 2 * sqrt(x4 + x5 + pow(obj_qw, 2));
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x6 * sensor_y;
	const GEN_FLT x10 = obj_qw * obj_qj;
	const GEN_FLT x11 = obj_qk * obj_qi;
	const GEN_FLT x12 = x6 * sensor_x;
	const GEN_FLT x13 = obj_pz + (1 - x4 * x6) * sensor_z + (-x10 + x11) * x12 + (x7 + x8) * x9;
	const GEN_FLT x14 = pow(lh_qi, 2);
	const GEN_FLT x15 = pow(lh_qk, 2);
	const GEN_FLT x16 = pow(lh_qj, 2);
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = 2 * sqrt(x14 + x17 + pow(lh_qw, 2));
	const GEN_FLT x19 = x13 * x18;
	const GEN_FLT x20 = x6 * sensor_z;
	const GEN_FLT x21 = obj_qw * obj_qk;
	const GEN_FLT x22 = obj_qj * obj_qi;
	const GEN_FLT x23 = obj_py + (1 - (x3 + x5) * x6) * sensor_y + (x21 + x22) * x12 + (-x7 + x8) * x20;
	const GEN_FLT x24 = lh_qw * lh_qk;
	const GEN_FLT x25 = lh_qj * lh_qi;
	const GEN_FLT x26 = obj_px + (1 - (x2 + x5) * x6) * sensor_x + (x10 + x11) * x20 + (-x21 + x22) * x9;
	const GEN_FLT x27 = x26 * x18;
	const GEN_FLT x28 = lh_py + x23 * (1 - (x14 + x15) * x18) + (-x0 + x1) * x19 + (x24 + x25) * x27;
	const GEN_FLT x29 = x23 * x18;
	const GEN_FLT x30 = lh_qw * lh_qj;
	const GEN_FLT x31 = lh_qk * lh_qi;
	const GEN_FLT x32 = lh_pz + x13 * (1 - (x14 + x16) * x18) + (x0 + x1) * x29 + (-x30 + x31) * x27;
	const GEN_FLT x33 = pow(x32, 2);
	const GEN_FLT x34 = lh_px + x26 * (1 - x18 * x17) + (-x24 + x25) * x29 + (x30 + x31) * x19;
	const GEN_FLT x35 = pow(x34, 2);
	const GEN_FLT x36 = x33 + x35;
	const GEN_FLT x37 = x28 / sqrt(x36);
	const GEN_FLT x38 = -x32;
	const GEN_FLT x39 = atan2(x34, x38);
	const GEN_FLT x40 = 1.5707963267949 + gibPhase_0 - phase_0 - x39 - asin(x37 * tilt_0);
	const GEN_FLT x41 = sin(x40) * gibMag_0;
	const GEN_FLT x42 = pow(x28, 2);
	const GEN_FLT x43 = x37 / sqrt(1 - x42 * pow(tilt_0, 2) / x36);
	const GEN_FLT x44 = x33 + x42;
	const GEN_FLT x45 = x34 / sqrt(x44);
	const GEN_FLT x46 = 1.5707963267949 + gibPhase_1 - phase_1 - asin(x45 * tilt_1) - atan2(-x28, x38);
	const GEN_FLT x47 = sin(x46) * gibMag_1;
	const GEN_FLT x48 = x45 / sqrt(1 - x35 * pow(tilt_1, 2) / x44);
	*(out++) = -1 - x41;
	*(out++) = -x43 - x41 * x43;
	*(out++) = pow(atan2(x28, x38), 2);
	*(out++) = x41;
	*(out++) = -cos(x40);
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = 0;
	*(out++) = -1 - x47;
	*(out++) = -x48 - x47 * x48;
	*(out++) = pow(x39, 2);
	*(out++) = x47;
	*(out++) = -cos(x46);
	*(out++) = 0;
	*(out++) = 0;
}

/** Applying function <function reproject_axis_x at 0x7ffa3c1c73b0> */
static inline void gen_reproject_axis_x(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
										const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = lh_qw * lh_qi;
	const GEN_FLT x1 = lh_qk * lh_qj;
	const GEN_FLT x2 = pow(obj_qj, 2);
	const GEN_FLT x3 = pow(obj_qi, 2);
	const GEN_FLT x4 = x2 + x3;
	const GEN_FLT x5 = pow(obj_qk, 2);
	const GEN_FLT x6 = 2 * sqrt(x4 + x5 + pow(obj_qw, 2));
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x6 * sensor_y;
	const GEN_FLT x10 = obj_qw * obj_qj;
	const GEN_FLT x11 = obj_qk * obj_qi;
	const GEN_FLT x12 = x6 * sensor_x;
	const GEN_FLT x13 = obj_pz + (1 - x4 * x6) * sensor_z + (-x10 + x11) * x12 + (x7 + x8) * x9;
	const GEN_FLT x14 = pow(lh_qi, 2);
	const GEN_FLT x15 = pow(lh_qk, 2);
	const GEN_FLT x16 = pow(lh_qj, 2);
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = 2 * sqrt(x14 + x17 + pow(lh_qw, 2));
	const GEN_FLT x19 = x13 * x18;
	const GEN_FLT x20 = x6 * sensor_z;
	const GEN_FLT x21 = obj_qw * obj_qk;
	const GEN_FLT x22 = obj_qj * obj_qi;
	const GEN_FLT x23 = obj_py + (1 - (x3 + x5) * x6) * sensor_y + (x21 + x22) * x12 + (-x7 + x8) * x20;
	const GEN_FLT x24 = lh_qw * lh_qk;
	const GEN_FLT x25 = lh_qj * lh_qi;
	const GEN_FLT x26 = obj_px + (1 - (x2 + x5) * x6) * sensor_x + (x10 + x11) * x20 + (-x21 + x22) * x9;
	const GEN_FLT x27 = x26 * x18;
	const GEN_FLT x28 = lh_py + x23 * (1 - (x14 + x15) * x18) + (-x0 + x1) * x19 + (x24 + x25) * x27;
	const GEN_FLT x29 = x23 * x18;
	const GEN_FLT x30 = lh_qw * lh_qj;
	const GEN_FLT x31 = lh_qk * lh_qi;
	const GEN_FLT x32 = lh_pz + x13 * (1 - (x14 + x16) * x18) + (x0 + x1) * x29 + (-x30 + x31) * x27;
	const GEN_FLT x33 = -x32;
	const GEN_FLT x34 = lh_px + x26 * (1 - x18 * x17) + (-x24 + x25) * x29 + (x30 + x31) * x19;
	const GEN_FLT x35 = -phase_0 - asin(x28 * tilt_0 / sqrt(pow(x32, 2) + pow(x34, 2))) - atan2(x34, x33);
	*(out++) = x35 - cos(1.5707963267949 + gibPhase_0 + x35) * gibMag_0 + pow(atan2(x28, x33), 2) * curve_0;
}

// Jacobian of reproject_axis_x wrt [obj_px, obj_py, obj_pz, obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_reproject_axis_x_jac_obj_p(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
												  const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt(x1 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - (x0 + x1) * x5;
	const GEN_FLT x7 = 1 - (x1 + x2) * x5;
	const GEN_FLT x8 = pow(obj_qj, 2);
	const GEN_FLT x9 = pow(obj_qi, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = x11 + x9;
	const GEN_FLT x13 = sqrt(x12 + x8 + pow(obj_qw, 2));
	const GEN_FLT x14 = 2 * x13;
	const GEN_FLT x15 = obj_qw * obj_qi;
	const GEN_FLT x16 = obj_qk * obj_qj;
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = x14 * sensor_y;
	const GEN_FLT x19 = obj_qw * obj_qj;
	const GEN_FLT x20 = obj_qk * obj_qi;
	const GEN_FLT x21 = -x19 + x20;
	const GEN_FLT x22 = x14 * sensor_x;
	const GEN_FLT x23 = obj_pz + x18 * x17 + x22 * x21 + (1 - x14 * x10) * sensor_z;
	const GEN_FLT x24 = -x15 + x16;
	const GEN_FLT x25 = x14 * sensor_z;
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x26 + x27;
	const GEN_FLT x29 = obj_py + x22 * x28 + x24 * x25 + (1 - x14 * x12) * sensor_y;
	const GEN_FLT x30 = lh_qw * lh_qi;
	const GEN_FLT x31 = lh_qk * lh_qj;
	const GEN_FLT x32 = x30 + x31;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = x19 + x20;
	const GEN_FLT x35 = -x26 + x27;
	const GEN_FLT x36 = x11 + x8;
	const GEN_FLT x37 = obj_px + x34 * x25 + x35 * x18 + (1 - x36 * x14) * sensor_x;
	const GEN_FLT x38 = lh_qw * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qi;
	const GEN_FLT x40 = -x38 + x39;
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = lh_pz + x33 * x29 + x41 * x37 + x7 * x23;
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = pow(x42, 2);
	const GEN_FLT x45 = pow(x44, -1);
	const GEN_FLT x46 = x38 + x39;
	const GEN_FLT x47 = x5 * x23;
	const GEN_FLT x48 = lh_qw * lh_qk;
	const GEN_FLT x49 = lh_qj * lh_qi;
	const GEN_FLT x50 = -x48 + x49;
	const GEN_FLT x51 = x5 * x50;
	const GEN_FLT x52 = lh_px + x46 * x47 + x51 * x29 + x6 * x37;
	const GEN_FLT x53 = x52 * x45;
	const GEN_FLT x54 = x44 + pow(x52, 2);
	const GEN_FLT x55 = pow(x54, -1);
	const GEN_FLT x56 = x55 * x44;
	const GEN_FLT x57 = -x30 + x31;
	const GEN_FLT x58 = 1 - x3 * x5;
	const GEN_FLT x59 = (x48 + x49) * x5;
	const GEN_FLT x60 = lh_py + x57 * x47 + x58 * x29 + x59 * x37;
	const GEN_FLT x61 = pow(x60, 2);
	const GEN_FLT x62 = pow(1 - x61 * x55 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x63 = 2 * x52;
	const GEN_FLT x64 = 4 * x4;
	const GEN_FLT x65 = x64 * x42;
	const GEN_FLT x66 = (1.0 / 2.0) * x60 * tilt_0 / pow(x54, 3.0 / 2.0);
	const GEN_FLT x67 = tilt_0 / sqrt(x54);
	const GEN_FLT x68 = -x56 * (x53 * x41 - x6 * x43) - x62 * (-x66 * (x6 * x63 + x65 * x40) + x67 * x59);
	const GEN_FLT x69 = -x42;
	const GEN_FLT x70 = sin(1.5707963267949 + gibPhase_0 - phase_0 - asin(x60 * x67) - atan2(x52, x69)) * gibMag_0;
	const GEN_FLT x71 = x60 * x45;
	const GEN_FLT x72 = 2 * x44 * atan2(x60, x69) * curve_0 / (x44 + x61);
	const GEN_FLT x73 = x64 * x52;
	const GEN_FLT x74 = -x62 * (x67 * x58 - (x65 * x32 + x73 * x50) * x66) - (-x51 * x43 + x53 * x33) * x56;
	const GEN_FLT x75 = x5 * x46;
	const GEN_FLT x76 = 2 * x42;
	const GEN_FLT x77 = x5 * x57;
	const GEN_FLT x78 = -x56 * (x7 * x53 - x75 * x43) - x62 * (-x66 * (x7 * x76 + x73 * x46) + x77 * x67);
	const GEN_FLT x79 = 2 / x13;
	const GEN_FLT x80 = x79 * x36;
	const GEN_FLT x81 = x80 * sensor_x;
	const GEN_FLT x82 = x79 * sensor_y;
	const GEN_FLT x83 = x82 * x35;
	const GEN_FLT x84 = x14 * obj_qk;
	const GEN_FLT x85 = x84 * sensor_y;
	const GEN_FLT x86 = x34 * sensor_z;
	const GEN_FLT x87 = x86 * x79;
	const GEN_FLT x88 = x25 * obj_qj;
	const GEN_FLT x89 = -x85 + x88 - x81 * obj_qw + x83 * obj_qw + x87 * obj_qw;
	const GEN_FLT x90 = x28 * sensor_x;
	const GEN_FLT x91 = x79 * x90;
	const GEN_FLT x92 = x79 * x12;
	const GEN_FLT x93 = x92 * sensor_y;
	const GEN_FLT x94 = x22 * obj_qk;
	const GEN_FLT x95 = x24 * sensor_z;
	const GEN_FLT x96 = x79 * x95;
	const GEN_FLT x97 = x14 * obj_qi;
	const GEN_FLT x98 = x97 * sensor_z;
	const GEN_FLT x99 = x94 - x98 + x91 * obj_qw - x93 * obj_qw + x96 * obj_qw;
	const GEN_FLT x100 = x22 * obj_qj;
	const GEN_FLT x101 = x21 * sensor_x;
	const GEN_FLT x102 = x79 * x101;
	const GEN_FLT x103 = x82 * x17;
	const GEN_FLT x104 = x97 * sensor_y;
	const GEN_FLT x105 = x79 * x10;
	const GEN_FLT x106 = x105 * sensor_z;
	const GEN_FLT x107 = -x100 + x104 + x102 * obj_qw + x103 * obj_qw - x106 * obj_qw;
	const GEN_FLT x108 = x5 * x107;
	const GEN_FLT x109 = x46 * x108 + x51 * x99 + x6 * x89;
	const GEN_FLT x110 = x7 * x107 + x89 * x41 + x99 * x33;
	const GEN_FLT x111 = x57 * x108 + x58 * x99 + x89 * x59;
	const GEN_FLT x112 = -x62 * (x67 * x111 - (x63 * x109 + x76 * x110) * x66) - (-x43 * x109 + x53 * x110) * x56;
	const GEN_FLT x113 = x79 * obj_qi;
	const GEN_FLT x114 = x113 * sensor_y;
	const GEN_FLT x115 = x18 * obj_qj;
	const GEN_FLT x116 = x84 * sensor_z;
	const GEN_FLT x117 = x115 + x116 + x35 * x114 - x81 * obj_qi + x86 * x113;
	const GEN_FLT x118 = 4 * x13;
	const GEN_FLT x119 = -x118 * obj_qi;
	const GEN_FLT x120 = x14 * obj_qw;
	const GEN_FLT x121 = x120 * sensor_z;
	const GEN_FLT x122 = x100 - x121 + x90 * x113 + x95 * x113 + (x119 - x92 * obj_qi) * sensor_y;
	const GEN_FLT x123 = x120 * sensor_y;
	const GEN_FLT x124 = x123 + x94 + x101 * x113 + x17 * x114 + (x119 - x105 * obj_qi) * sensor_z;
	const GEN_FLT x125 = x51 * x122 + x6 * x117 + x75 * x124;
	const GEN_FLT x126 = x33 * x122 + x41 * x117 + x7 * x124;
	const GEN_FLT x127 = x58 * x122 + x59 * x117 + x77 * x124;
	const GEN_FLT x128 = -x62 * (x67 * x127 - (x63 * x125 + x76 * x126) * x66) - (-x43 * x125 + x53 * x126) * x56;
	const GEN_FLT x129 = -x118 * obj_qj;
	const GEN_FLT x130 = x79 * obj_qj;
	const GEN_FLT x131 = x104 + x121 + x83 * obj_qj + x86 * x130 + (x129 - x80 * obj_qj) * sensor_x;
	const GEN_FLT x132 = x22 * obj_qi;
	const GEN_FLT x133 = x116 + x132 + x91 * obj_qj - x93 * obj_qj + x96 * obj_qj;
	const GEN_FLT x134 = x22 * obj_qw;
	const GEN_FLT x135 = -x134 + x85 + x101 * x130 + x103 * obj_qj + (x129 - x105 * obj_qj) * sensor_z;
	const GEN_FLT x136 = x51 * x133 + x6 * x131 + x75 * x135;
	const GEN_FLT x137 = x33 * x133 + x41 * x131 + x7 * x135;
	const GEN_FLT x138 = x58 * x133 + x59 * x131 + x77 * x135;
	const GEN_FLT x139 = -x62 * (x67 * x138 - (x63 * x136 + x76 * x137) * x66) - (-x43 * x136 + x53 * x137) * x56;
	const GEN_FLT x140 = -x118 * obj_qk;
	const GEN_FLT x141 = -x123 + x98 + x83 * obj_qk + x87 * obj_qk + (x140 - x80 * obj_qk) * sensor_x;
	const GEN_FLT x142 = x134 + x88 + x91 * obj_qk + x96 * obj_qk + (x140 - x92 * obj_qk) * sensor_y;
	const GEN_FLT x143 = x115 + x132 + x102 * obj_qk + x103 * obj_qk - x106 * obj_qk;
	const GEN_FLT x144 = x5 * x143;
	const GEN_FLT x145 = x46 * x144 + x51 * x142 + x6 * x141;
	const GEN_FLT x146 = x33 * x142 + x41 * x141 + x7 * x143;
	const GEN_FLT x147 = x57 * x144 + x58 * x142 + x59 * x141;
	const GEN_FLT x148 = -x62 * (x67 * x147 - (x63 * x145 + x76 * x146) * x66) - (-x43 * x145 + x53 * x146) * x56;
	*(out++) = x68 + x70 * x68 + (-x59 * x43 + x71 * x41) * x72;
	*(out++) = x74 + x70 * x74 + (-x58 * x43 + x71 * x33) * x72;
	*(out++) = x78 + x70 * x78 + x72 * (x7 * x71 - x77 * x43);
	*(out++) = x112 + x70 * x112 + (-x43 * x111 + x71 * x110) * x72;
	*(out++) = x128 + x70 * x128 + (-x43 * x127 + x71 * x126) * x72;
	*(out++) = x139 + x70 * x139 + (-x43 * x138 + x71 * x137) * x72;
	*(out++) = x148 + x70 * x148 + (-x43 * x147 + x71 * x146) * x72;
}

// Jacobian of reproject_axis_x wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_reproject_axis_x_jac_sensor_pt(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
													  const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qk, 2);
	const GEN_FLT x4 = sqrt(x2 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - x2 * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = pow(obj_qk, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = sqrt(x10 + x7 + pow(obj_qw, 2));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 - (x7 + x8) * x12;
	const GEN_FLT x14 = obj_qw * obj_qi;
	const GEN_FLT x15 = obj_qk * obj_qj;
	const GEN_FLT x16 = x14 + x15;
	const GEN_FLT x17 = x12 * sensor_y;
	const GEN_FLT x18 = obj_qw * obj_qj;
	const GEN_FLT x19 = obj_qk * obj_qi;
	const GEN_FLT x20 = -x18 + x19;
	const GEN_FLT x21 = x12 * sensor_x;
	const GEN_FLT x22 = obj_pz + x13 * sensor_z + x17 * x16 + x20 * x21;
	const GEN_FLT x23 = lh_qw * lh_qi;
	const GEN_FLT x24 = lh_qk * lh_qj;
	const GEN_FLT x25 = x23 + x24;
	const GEN_FLT x26 = -x14 + x15;
	const GEN_FLT x27 = x12 * sensor_z;
	const GEN_FLT x28 = 1 - x12 * x10;
	const GEN_FLT x29 = obj_qw * obj_qk;
	const GEN_FLT x30 = obj_qj * obj_qi;
	const GEN_FLT x31 = x29 + x30;
	const GEN_FLT x32 = obj_py + x26 * x27 + x28 * sensor_y + x31 * x21;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = lh_qw * lh_qj;
	const GEN_FLT x35 = lh_qk * lh_qi;
	const GEN_FLT x36 = -x34 + x35;
	const GEN_FLT x37 = x18 + x19;
	const GEN_FLT x38 = -x29 + x30;
	const GEN_FLT x39 = 1 - (x7 + x9) * x12;
	const GEN_FLT x40 = obj_px + x37 * x27 + x38 * x17 + x39 * sensor_x;
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = lh_pz + x33 * x25 + x41 * x36 + x6 * x22;
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = 1 - (x0 + x3) * x5;
	const GEN_FLT x45 = lh_qw * lh_qk;
	const GEN_FLT x46 = lh_qj * lh_qi;
	const GEN_FLT x47 = -x45 + x46;
	const GEN_FLT x48 = 4 * x4 * x11;
	const GEN_FLT x49 = x48 * x31;
	const GEN_FLT x50 = x34 + x35;
	const GEN_FLT x51 = x48 * x20;
	const GEN_FLT x52 = x44 * x39 + x47 * x49 + x50 * x51;
	const GEN_FLT x53 = x5 * x39;
	const GEN_FLT x54 = x6 * x12;
	const GEN_FLT x55 = x49 * x25 + x53 * x36 + x54 * x20;
	const GEN_FLT x56 = x5 * x22;
	const GEN_FLT x57 = lh_px + x40 * x44 + x47 * x33 + x50 * x56;
	const GEN_FLT x58 = pow(x42, 2);
	const GEN_FLT x59 = pow(x58, -1);
	const GEN_FLT x60 = x57 * x59;
	const GEN_FLT x61 = x58 + pow(x57, 2);
	const GEN_FLT x62 = pow(x61, -1);
	const GEN_FLT x63 = x62 * x58;
	const GEN_FLT x64 = -x23 + x24;
	const GEN_FLT x65 = 1 - (x1 + x3) * x5;
	const GEN_FLT x66 = x45 + x46;
	const GEN_FLT x67 = lh_py + x64 * x56 + x65 * x32 + x66 * x41;
	const GEN_FLT x68 = pow(x67, 2);
	const GEN_FLT x69 = pow(1 - x62 * x68 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x70 = 2 * x57;
	const GEN_FLT x71 = 2 * x42;
	const GEN_FLT x72 = (1.0 / 2.0) * x67 * tilt_0 / pow(x61, 3.0 / 2.0);
	const GEN_FLT x73 = x65 * x12;
	const GEN_FLT x74 = x64 * x51 + x66 * x53 + x73 * x31;
	const GEN_FLT x75 = tilt_0 / sqrt(x61);
	const GEN_FLT x76 = -x69 * (x75 * x74 - (x70 * x52 + x71 * x55) * x72) - (-x52 * x43 + x60 * x55) * x63;
	const GEN_FLT x77 = -x42;
	const GEN_FLT x78 = sin(1.5707963267949 + gibPhase_0 - phase_0 - asin(x75 * x67) - atan2(x57, x77)) * gibMag_0;
	const GEN_FLT x79 = x67 * x59;
	const GEN_FLT x80 = 2 * x58 * atan2(x67, x77) * curve_0 / (x58 + x68);
	const GEN_FLT x81 = x5 * x28;
	const GEN_FLT x82 = x44 * x12;
	const GEN_FLT x83 = x48 * x16;
	const GEN_FLT x84 = x81 * x47 + x82 * x38 + x83 * x50;
	const GEN_FLT x85 = x48 * x38;
	const GEN_FLT x86 = x54 * x16 + x81 * x25 + x85 * x36;
	const GEN_FLT x87 = x65 * x28 + x83 * x64 + x85 * x66;
	const GEN_FLT x88 = -x69 * (x87 * x75 - (x84 * x70 + x86 * x71) * x72) - (-x84 * x43 + x86 * x60) * x63;
	const GEN_FLT x89 = x48 * x26;
	const GEN_FLT x90 = x5 * x13;
	const GEN_FLT x91 = x50 * x90 + x82 * x37 + x89 * x47;
	const GEN_FLT x92 = x48 * x37;
	const GEN_FLT x93 = x6 * x13 + x89 * x25 + x92 * x36;
	const GEN_FLT x94 = x64 * x90 + x66 * x92 + x73 * x26;
	const GEN_FLT x95 = -x69 * (x75 * x94 - (x70 * x91 + x71 * x93) * x72) - (x60 * x93 - x91 * x43) * x63;
	*(out++) = x76 + x78 * x76 + (-x74 * x43 + x79 * x55) * x80;
	*(out++) = x88 + x88 * x78 + (x86 * x79 - x87 * x43) * x80;
	*(out++) = x95 + x78 * x95 + (x79 * x93 - x94 * x43) * x80;
}

// Jacobian of reproject_axis_x wrt [lh_px, lh_py, lh_pz, lh_qw, lh_qi, lh_qj, lh_qk]
static inline void gen_reproject_axis_x_jac_lh_p(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
												 const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qk, 2);
	const GEN_FLT x4 = x1 + x3;
	const GEN_FLT x5 = sqrt(x0 + x4 + pow(lh_qw, 2));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = pow(obj_qk, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = 2 * sqrt(x10 + x7 + pow(obj_qw, 2));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 = obj_pz + (1 - (x7 + x8) * x11) * sensor_z + (x12 + x13) * x14 + (-x15 + x16) * x17;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + (1 - x11 * x10) * sensor_y + (-x12 + x13) * x22 + (x23 + x24) * x17;
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = -x27 + x28;
	const GEN_FLT x30 = obj_px + (1 - (x7 + x9) * x11) * sensor_x + (x15 + x16) * x22 + (-x23 + x24) * x14;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + x18 * (1 - x2 * x6) + x21 * x26 + x31 * x29;
	const GEN_FLT x33 = x27 + x28;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = -x35 + x36;
	const GEN_FLT x38 = x0 + x3;
	const GEN_FLT x39 = lh_px + x30 * (1 - x6 * x38) + x34 * x33 + x37 * x26;
	const GEN_FLT x40 = pow(x32, 2);
	const GEN_FLT x41 = x40 + pow(x39, 2);
	const GEN_FLT x42 = pow(x41, -1);
	const GEN_FLT x43 = -x19 + x20;
	const GEN_FLT x44 = x35 + x36;
	const GEN_FLT x45 = lh_py + x25 * (1 - x4 * x6) + x43 * x34 + x44 * x31;
	const GEN_FLT x46 = pow(x45, 2);
	const GEN_FLT x47 = pow(1 - x42 * x46 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x48 = x45 * tilt_0 / pow(x41, 3.0 / 2.0);
	const GEN_FLT x49 = x47 * x48;
	const GEN_FLT x50 = x42 * x32 + x49 * x39;
	const GEN_FLT x51 = tilt_0 / sqrt(x41);
	const GEN_FLT x52 = -x32;
	const GEN_FLT x53 = sin(1.5707963267949 + gibPhase_0 - phase_0 - asin(x51 * x45) - atan2(x39, x52)) * gibMag_0;
	const GEN_FLT x54 = x51 * x47;
	const GEN_FLT x55 = 2 * x32;
	const GEN_FLT x56 = atan2(x45, x52) * curve_0 / (x40 + x46);
	const GEN_FLT x57 = -x42 * x39 + x49 * x32;
	const GEN_FLT x58 = 2 * x56;
	const GEN_FLT x59 = pow(x32, -1);
	const GEN_FLT x60 = 2 / x5;
	const GEN_FLT x61 = x60 * x38;
	const GEN_FLT x62 = x61 * x30;
	const GEN_FLT x63 = x60 * lh_qw;
	const GEN_FLT x64 = x37 * x25;
	const GEN_FLT x65 = x26 * lh_qk;
	const GEN_FLT x66 = x60 * x18;
	const GEN_FLT x67 = x66 * x33;
	const GEN_FLT x68 = x34 * lh_qj;
	const GEN_FLT x69 = -x65 + x68 - x62 * lh_qw + x63 * x64 + x67 * lh_qw;
	const GEN_FLT x70 = x30 * x29;
	const GEN_FLT x71 = x25 * x21;
	const GEN_FLT x72 = x26 * lh_qi;
	const GEN_FLT x73 = x31 * lh_qj;
	const GEN_FLT x74 = x2 * x60;
	const GEN_FLT x75 = x74 * x18;
	const GEN_FLT x76 = x72 - x73 + x70 * x63 + x71 * x63 - x75 * lh_qw;
	const GEN_FLT x77 = pow(x40, -1);
	const GEN_FLT x78 = x77 * x39;
	const GEN_FLT x79 = x40 * x42;
	const GEN_FLT x80 = 2 * x39;
	const GEN_FLT x81 = (1.0 / 2.0) * x48;
	const GEN_FLT x82 = x44 * x30;
	const GEN_FLT x83 = x31 * lh_qk;
	const GEN_FLT x84 = x4 * x60;
	const GEN_FLT x85 = x84 * x25;
	const GEN_FLT x86 = x66 * x43;
	const GEN_FLT x87 = x34 * lh_qi;
	const GEN_FLT x88 = x83 - x87 + x82 * x63 - x85 * lh_qw + x86 * lh_qw;
	const GEN_FLT x89 = -x47 * (x88 * x51 - (x76 * x55 + x80 * x69) * x81) - (-x69 * x59 + x78 * x76) * x79;
	const GEN_FLT x90 = x77 * x45;
	const GEN_FLT x91 = x58 * x40;
	const GEN_FLT x92 = x60 * lh_qi;
	const GEN_FLT x93 = x26 * lh_qj;
	const GEN_FLT x94 = x34 * lh_qk;
	const GEN_FLT x95 = x93 + x94 - x62 * lh_qi + x64 * x92 + x67 * lh_qi;
	const GEN_FLT x96 = x26 * lh_qw;
	const GEN_FLT x97 = 4 * x5;
	const GEN_FLT x98 = -x97 * lh_qi;
	const GEN_FLT x99 = x83 + x96 + x18 * (x98 - x74 * lh_qi) + x70 * x92 + x71 * x92;
	const GEN_FLT x100 = x34 * lh_qw;
	const GEN_FLT x101 = -x100 + x73 + x25 * (x98 - x84 * lh_qi) + x82 * x92 + x86 * lh_qi;
	const GEN_FLT x102 = -x47 * (x51 * x101 - (x55 * x99 + x80 * x95) * x81) - (-x59 * x95 + x78 * x99) * x79;
	const GEN_FLT x103 = -x97 * lh_qj;
	const GEN_FLT x104 = x60 * lh_qj;
	const GEN_FLT x105 = x100 + x72 + x30 * (x103 - x61 * lh_qj) + x64 * x104 + x67 * lh_qj;
	const GEN_FLT x106 = x31 * lh_qw;
	const GEN_FLT x107 = -x106 + x65 + x18 * (x103 - x74 * lh_qj) + x70 * x104 + x71 * x104;
	const GEN_FLT x108 = x31 * lh_qi;
	const GEN_FLT x109 = x108 + x94 + x82 * x104 - x85 * lh_qj + x86 * lh_qj;
	const GEN_FLT x110 = -x47 * (x51 * x109 - (x55 * x107 + x80 * x105) * x81) - (-x59 * x105 + x78 * x107) * x79;
	const GEN_FLT x111 = -x97 * lh_qk;
	const GEN_FLT x112 = x60 * lh_qk;
	const GEN_FLT x113 = x25 * x112;
	const GEN_FLT x114 = x87 - x96 + x30 * (x111 - x61 * lh_qk) + x37 * x113 + x67 * lh_qk;
	const GEN_FLT x115 = x108 + x93 + x21 * x113 + x70 * x112 - x75 * lh_qk;
	const GEN_FLT x116 = x106 + x68 + x25 * (x111 - x84 * lh_qk) + x82 * x112 + x86 * lh_qk;
	const GEN_FLT x117 = -x47 * (x51 * x116 - (x55 * x115 + x80 * x114) * x81) - (-x59 * x114 + x78 * x115) * x79;
	*(out++) = x50 + x50 * x53;
	*(out++) = -x54 - x54 * x53 - x56 * x55;
	*(out++) = x57 + x53 * x57 + x58 * x45;
	*(out++) = x89 + x89 * x53 + (x76 * x90 - x88 * x59) * x91;
	*(out++) = x102 + x53 * x102 + x91 * (-x59 * x101 + x90 * x99);
	*(out++) = x110 + x53 * x110 + (-x59 * x109 + x90 * x107) * x91;
	*(out++) = x117 + x53 * x117 + (-x59 * x116 + x90 * x115) * x91;
}

// Jacobian of reproject_axis_x wrt [phase_0, tilt_0, curve_0, gibPhase_0, gibMag_0, ogeeMag_0, ogeePhase_0]
static inline void gen_reproject_axis_x_jac_bsc0(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
												 const SurvivePose *lh_p, const BaseStationCal *bsc0) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_0 = (*bsc0).phase;
	const GEN_FLT tilt_0 = (*bsc0).tilt;
	const GEN_FLT curve_0 = (*bsc0).curve;
	const GEN_FLT gibPhase_0 = (*bsc0).gibpha;
	const GEN_FLT gibMag_0 = (*bsc0).gibmag;
	const GEN_FLT ogeeMag_0 = (*bsc0).ogeephase;
	const GEN_FLT ogeePhase_0 = (*bsc0).ogeemag;
	const GEN_FLT x0 = lh_qw * lh_qi;
	const GEN_FLT x1 = lh_qk * lh_qj;
	const GEN_FLT x2 = pow(obj_qj, 2);
	const GEN_FLT x3 = pow(obj_qi, 2);
	const GEN_FLT x4 = x2 + x3;
	const GEN_FLT x5 = pow(obj_qk, 2);
	const GEN_FLT x6 = 2 * sqrt(x4 + x5 + pow(obj_qw, 2));
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x6 * sensor_y;
	const GEN_FLT x10 = obj_qw * obj_qj;
	const GEN_FLT x11 = obj_qk * obj_qi;
	const GEN_FLT x12 = x6 * sensor_x;
	const GEN_FLT x13 = obj_pz + (1 - x4 * x6) * sensor_z + (-x10 + x11) * x12 + (x7 + x8) * x9;
	const GEN_FLT x14 = pow(lh_qi, 2);
	const GEN_FLT x15 = pow(lh_qk, 2);
	const GEN_FLT x16 = pow(lh_qj, 2);
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = 2 * sqrt(x14 + x17 + pow(lh_qw, 2));
	const GEN_FLT x19 = x13 * x18;
	const GEN_FLT x20 = x6 * sensor_z;
	const GEN_FLT x21 = obj_qw * obj_qk;
	const GEN_FLT x22 = obj_qj * obj_qi;
	const GEN_FLT x23 = obj_py + (1 - (x3 + x5) * x6) * sensor_y + (x21 + x22) * x12 + (-x7 + x8) * x20;
	const GEN_FLT x24 = lh_qw * lh_qk;
	const GEN_FLT x25 = lh_qj * lh_qi;
	const GEN_FLT x26 = obj_px + (1 - (x2 + x5) * x6) * sensor_x + (x10 + x11) * x20 + (-x21 + x22) * x9;
	const GEN_FLT x27 = x26 * x18;
	const GEN_FLT x28 = lh_py + x23 * (1 - (x14 + x15) * x18) + (-x0 + x1) * x19 + (x24 + x25) * x27;
	const GEN_FLT x29 = x23 * x18;
	const GEN_FLT x30 = lh_qw * lh_qj;
	const GEN_FLT x31 = lh_qk * lh_qi;
	const GEN_FLT x32 = lh_pz + x13 * (1 - (x14 + x16) * x18) + (x0 + x1) * x29 + (-x30 + x31) * x27;
	const GEN_FLT x33 = lh_px + x26 * (1 - x18 * x17) + (-x24 + x25) * x29 + (x30 + x31) * x19;
	const GEN_FLT x34 = pow(x32, 2) + pow(x33, 2);
	const GEN_FLT x35 = x28 / sqrt(x34);
	const GEN_FLT x36 = -x32;
	const GEN_FLT x37 = 1.5707963267949 + gibPhase_0 - phase_0 - asin(x35 * tilt_0) - atan2(x33, x36);
	const GEN_FLT x38 = sin(x37) * gibMag_0;
	const GEN_FLT x39 = x35 / sqrt(1 - pow(x28, 2) * pow(tilt_0, 2) / x34);
	*(out++) = -1 - x38;
	*(out++) = -x39 - x38 * x39;
	*(out++) = pow(atan2(x28, x36), 2);
	*(out++) = x38;
	*(out++) = -cos(x37);
	*(out++) = 0;
	*(out++) = 0;
}

/** Applying function <function reproject_axis_y at 0x7ffa3c1c7440> */
static inline void gen_reproject_axis_y(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
										const SurvivePose *lh_p, const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = pow(lh_qk, 2);
	const GEN_FLT x3 = x1 + x2;
	const GEN_FLT x4 = 2 * sqrt(x0 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = pow(obj_qj, 2);
	const GEN_FLT x6 = pow(obj_qi, 2);
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = x6 + x7;
	const GEN_FLT x9 = 2 * sqrt(x5 + x8 + pow(obj_qw, 2));
	const GEN_FLT x10 = obj_qw * obj_qi;
	const GEN_FLT x11 = obj_qk * obj_qj;
	const GEN_FLT x12 = x9 * sensor_y;
	const GEN_FLT x13 = obj_qw * obj_qj;
	const GEN_FLT x14 = obj_qk * obj_qi;
	const GEN_FLT x15 = x9 * sensor_x;
	const GEN_FLT x16 = obj_pz + (1 - (x5 + x6) * x9) * sensor_z + (x10 + x11) * x12 + (-x13 + x14) * x15;
	const GEN_FLT x17 = lh_qw * lh_qi;
	const GEN_FLT x18 = lh_qk * lh_qj;
	const GEN_FLT x19 = x9 * sensor_z;
	const GEN_FLT x20 = obj_qw * obj_qk;
	const GEN_FLT x21 = obj_qj * obj_qi;
	const GEN_FLT x22 = obj_py + (1 - x8 * x9) * sensor_y + (-x10 + x11) * x19 + (x20 + x21) * x15;
	const GEN_FLT x23 = x4 * x22;
	const GEN_FLT x24 = lh_qw * lh_qj;
	const GEN_FLT x25 = lh_qk * lh_qi;
	const GEN_FLT x26 = obj_px + (1 - (x5 + x7) * x9) * sensor_x + (x13 + x14) * x19 + (-x20 + x21) * x12;
	const GEN_FLT x27 = x4 * x26;
	const GEN_FLT x28 = lh_pz + x16 * (1 - (x0 + x1) * x4) + (x17 + x18) * x23 + (-x24 + x25) * x27;
	const GEN_FLT x29 = x4 * x16;
	const GEN_FLT x30 = lh_qw * lh_qk;
	const GEN_FLT x31 = lh_qj * lh_qi;
	const GEN_FLT x32 = lh_py + x22 * (1 - x4 * x3) + (-x17 + x18) * x29 + (x30 + x31) * x27;
	const GEN_FLT x33 = lh_px + x26 * (1 - (x0 + x2) * x4) + (x24 + x25) * x29 + (-x30 + x31) * x23;
	const GEN_FLT x34 = -x28;
	const GEN_FLT x35 = -phase_1 - asin(x33 * tilt_1 / sqrt(pow(x28, 2) + pow(x32, 2))) - atan2(-x32, x34);
	*(out++) = x35 - cos(1.5707963267949 + gibPhase_1 + x35) * gibMag_1 + pow(atan2(x33, x34), 2) * curve_1;
}

// Jacobian of reproject_axis_y wrt [obj_px, obj_py, obj_pz, obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_reproject_axis_y_jac_obj_p(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
												  const SurvivePose *lh_p, const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt(x1 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - (x0 + x1) * x5;
	const GEN_FLT x7 = 1 - (x1 + x2) * x5;
	const GEN_FLT x8 = pow(obj_qj, 2);
	const GEN_FLT x9 = pow(obj_qi, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = x11 + x9;
	const GEN_FLT x13 = sqrt(x12 + x8 + pow(obj_qw, 2));
	const GEN_FLT x14 = 2 * x13;
	const GEN_FLT x15 = obj_qw * obj_qi;
	const GEN_FLT x16 = obj_qk * obj_qj;
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = x14 * sensor_y;
	const GEN_FLT x19 = obj_qw * obj_qj;
	const GEN_FLT x20 = obj_qk * obj_qi;
	const GEN_FLT x21 = -x19 + x20;
	const GEN_FLT x22 = x14 * sensor_x;
	const GEN_FLT x23 = obj_pz + x18 * x17 + x22 * x21 + (1 - x14 * x10) * sensor_z;
	const GEN_FLT x24 = -x15 + x16;
	const GEN_FLT x25 = x14 * sensor_z;
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x26 + x27;
	const GEN_FLT x29 = obj_py + x22 * x28 + x24 * x25 + (1 - x14 * x12) * sensor_y;
	const GEN_FLT x30 = lh_qw * lh_qi;
	const GEN_FLT x31 = lh_qk * lh_qj;
	const GEN_FLT x32 = x30 + x31;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = x19 + x20;
	const GEN_FLT x35 = -x26 + x27;
	const GEN_FLT x36 = x11 + x8;
	const GEN_FLT x37 = obj_px + x34 * x25 + x35 * x18 + (1 - x36 * x14) * sensor_x;
	const GEN_FLT x38 = lh_qw * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qi;
	const GEN_FLT x40 = -x38 + x39;
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = lh_pz + x33 * x29 + x41 * x37 + x7 * x23;
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = pow(x42, 2);
	const GEN_FLT x45 = pow(x44, -1);
	const GEN_FLT x46 = x38 + x39;
	const GEN_FLT x47 = x5 * x23;
	const GEN_FLT x48 = lh_qw * lh_qk;
	const GEN_FLT x49 = lh_qj * lh_qi;
	const GEN_FLT x50 = (-x48 + x49) * x5;
	const GEN_FLT x51 = lh_px + x46 * x47 + x50 * x29 + x6 * x37;
	const GEN_FLT x52 = x51 * x45;
	const GEN_FLT x53 = -x42;
	const GEN_FLT x54 = pow(x51, 2);
	const GEN_FLT x55 = 2 * x44 * atan2(x51, x53) * curve_1 / (x44 + x54);
	const GEN_FLT x56 = x48 + x49;
	const GEN_FLT x57 = x5 * x56;
	const GEN_FLT x58 = -x30 + x31;
	const GEN_FLT x59 = 1 - x3 * x5;
	const GEN_FLT x60 = lh_py + x57 * x37 + x58 * x47 + x59 * x29;
	const GEN_FLT x61 = 2 * x60;
	const GEN_FLT x62 = x4 * x61 * x45;
	const GEN_FLT x63 = x44 + pow(x60, 2);
	const GEN_FLT x64 = pow(x63, -1);
	const GEN_FLT x65 = x64 * x44;
	const GEN_FLT x66 = pow(1 - x64 * x54 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x67 = tilt_1 / sqrt(x63);
	const GEN_FLT x68 = 4 * x4;
	const GEN_FLT x69 = x60 * x68;
	const GEN_FLT x70 = x68 * x42;
	const GEN_FLT x71 = (1.0 / 2.0) * x51 * tilt_1 / pow(x63, 3.0 / 2.0);
	const GEN_FLT x72 = -x66 * (x6 * x67 - (x69 * x56 + x70 * x40) * x71) - (x57 * x43 - x62 * x40) * x65;
	const GEN_FLT x73 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x67 * x51) - atan2(-x60, x53)) * gibMag_1;
	const GEN_FLT x74 = -x66 * (x67 * x50 - (x61 * x59 + x70 * x32) * x71) - (x59 * x43 - x62 * x32) * x65;
	const GEN_FLT x75 = x5 * x43;
	const GEN_FLT x76 = x7 * x45;
	const GEN_FLT x77 = x5 * x46;
	const GEN_FLT x78 = 2 * x42;
	const GEN_FLT x79 = -x66 * (-x71 * (x69 * x58 + x7 * x78) + x77 * x67) - (x75 * x58 - x76 * x60) * x65;
	const GEN_FLT x80 = 2 / x13;
	const GEN_FLT x81 = x80 * x36;
	const GEN_FLT x82 = x81 * sensor_x;
	const GEN_FLT x83 = x35 * sensor_y;
	const GEN_FLT x84 = x80 * x83;
	const GEN_FLT x85 = x18 * obj_qk;
	const GEN_FLT x86 = x80 * obj_qw;
	const GEN_FLT x87 = x34 * sensor_z;
	const GEN_FLT x88 = x25 * obj_qj;
	const GEN_FLT x89 = -x85 + x88 - x82 * obj_qw + x84 * obj_qw + x86 * x87;
	const GEN_FLT x90 = x28 * sensor_x;
	const GEN_FLT x91 = x80 * x12;
	const GEN_FLT x92 = x91 * sensor_y;
	const GEN_FLT x93 = x22 * obj_qk;
	const GEN_FLT x94 = x24 * sensor_z;
	const GEN_FLT x95 = x25 * obj_qi;
	const GEN_FLT x96 = x93 - x95 + x86 * x90 + x86 * x94 - x92 * obj_qw;
	const GEN_FLT x97 = x22 * obj_qj;
	const GEN_FLT x98 = x21 * sensor_x;
	const GEN_FLT x99 = x17 * sensor_y;
	const GEN_FLT x100 = x80 * x99;
	const GEN_FLT x101 = x18 * obj_qi;
	const GEN_FLT x102 = x80 * x10;
	const GEN_FLT x103 = x102 * sensor_z;
	const GEN_FLT x104 = x101 - x97 + x100 * obj_qw - x103 * obj_qw + x86 * x98;
	const GEN_FLT x105 = x5 * x104;
	const GEN_FLT x106 = x46 * x105 + x50 * x96 + x6 * x89;
	const GEN_FLT x107 = x7 * x104 + x89 * x41 + x96 * x33;
	const GEN_FLT x108 = x58 * x105 + x59 * x96 + x89 * x57;
	const GEN_FLT x109 = x60 * x45;
	const GEN_FLT x110 = -x65 * (-x109 * x107 + x43 * x108) - x66 * (x67 * x106 - (x61 * x108 + x78 * x107) * x71);
	const GEN_FLT x111 = x80 * obj_qi;
	const GEN_FLT x112 = x18 * obj_qj;
	const GEN_FLT x113 = x111 * sensor_z;
	const GEN_FLT x114 = x25 * obj_qk;
	const GEN_FLT x115 = x112 + x114 + x34 * x113 - x82 * obj_qi + x83 * x111;
	const GEN_FLT x116 = x111 * sensor_x;
	const GEN_FLT x117 = 4 * x13;
	const GEN_FLT x118 = -x117 * obj_qi;
	const GEN_FLT x119 = x25 * obj_qw;
	const GEN_FLT x120 = -x119 + x97 + x24 * x113 + x28 * x116 + (x118 - x91 * obj_qi) * sensor_y;
	const GEN_FLT x121 = x18 * obj_qw;
	const GEN_FLT x122 = x121 + x93 + x21 * x116 + x99 * x111 + (x118 - x102 * obj_qi) * sensor_z;
	const GEN_FLT x123 = x50 * x120 + x6 * x115 + x77 * x122;
	const GEN_FLT x124 = x33 * x120 + x41 * x115 + x7 * x122;
	const GEN_FLT x125 = x5 * x58;
	const GEN_FLT x126 = x122 * x125 + x57 * x115 + x59 * x120;
	const GEN_FLT x127 = -x65 * (-x109 * x124 + x43 * x126) - x66 * (x67 * x123 - (x61 * x126 + x78 * x124) * x71);
	const GEN_FLT x128 = -x117 * obj_qj;
	const GEN_FLT x129 = x80 * obj_qj;
	const GEN_FLT x130 = x101 + x119 + x84 * obj_qj + x87 * x129 + (x128 - x81 * obj_qj) * sensor_x;
	const GEN_FLT x131 = x22 * obj_qi;
	const GEN_FLT x132 = x114 + x131 + x90 * x129 - x92 * obj_qj + x94 * x129;
	const GEN_FLT x133 = x22 * obj_qw;
	const GEN_FLT x134 = -x133 + x85 + x100 * obj_qj + x98 * x129 + (x128 - x102 * obj_qj) * sensor_z;
	const GEN_FLT x135 = x50 * x132 + x6 * x130 + x77 * x134;
	const GEN_FLT x136 = x33 * x132 + x41 * x130 + x7 * x134;
	const GEN_FLT x137 = x45 * x136;
	const GEN_FLT x138 = x125 * x134 + x57 * x130 + x59 * x132;
	const GEN_FLT x139 = -x66 * (x67 * x135 - (x61 * x138 + x78 * x136) * x71) - (x43 * x138 - x60 * x137) * x65;
	const GEN_FLT x140 = -x117 * obj_qk;
	const GEN_FLT x141 = x80 * obj_qk;
	const GEN_FLT x142 = -x121 + x95 + x84 * obj_qk + x87 * x141 + (x140 - x81 * obj_qk) * sensor_x;
	const GEN_FLT x143 = x133 + x88 + x90 * x141 + x94 * x141 + (x140 - x91 * obj_qk) * sensor_y;
	const GEN_FLT x144 = x112 + x131 + x100 * obj_qk - x103 * obj_qk + x98 * x141;
	const GEN_FLT x145 = x5 * x144;
	const GEN_FLT x146 = x46 * x145 + x50 * x143 + x6 * x142;
	const GEN_FLT x147 = x33 * x143 + x41 * x142 + x7 * x144;
	const GEN_FLT x148 = x45 * x147;
	const GEN_FLT x149 = x57 * x142 + x58 * x145 + x59 * x143;
	const GEN_FLT x150 = -x66 * (x67 * x146 - (x61 * x149 + x78 * x147) * x71) - (x43 * x149 - x60 * x148) * x65;
	*(out++) = x72 + x55 * (x52 * x41 - x6 * x43) + x73 * x72;
	*(out++) = x74 + x73 * x74 + (-x50 * x43 + x52 * x33) * x55;
	*(out++) = x79 + x73 * x79 + (-x75 * x46 + x76 * x51) * x55;
	*(out++) = x110 + x73 * x110 + (-x43 * x106 + x52 * x107) * x55;
	*(out++) = x127 + x73 * x127 + (-x43 * x123 + x52 * x124) * x55;
	*(out++) = x139 + x73 * x139 + (-x43 * x135 + x51 * x137) * x55;
	*(out++) = x150 + x73 * x150 + (-x43 * x146 + x51 * x148) * x55;
}

// Jacobian of reproject_axis_y wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_reproject_axis_y_jac_sensor_pt(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
													  const SurvivePose *lh_p, const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = pow(lh_qk, 2);
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt(x1 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - (x0 + x1) * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = x7 + x8;
	const GEN_FLT x10 = pow(obj_qk, 2);
	const GEN_FLT x11 = sqrt(x10 + x9 + pow(obj_qw, 2));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 - x9 * x12;
	const GEN_FLT x14 = obj_qw * obj_qi;
	const GEN_FLT x15 = obj_qk * obj_qj;
	const GEN_FLT x16 = x14 + x15;
	const GEN_FLT x17 = x12 * sensor_y;
	const GEN_FLT x18 = obj_qw * obj_qj;
	const GEN_FLT x19 = obj_qk * obj_qi;
	const GEN_FLT x20 = -x18 + x19;
	const GEN_FLT x21 = x12 * sensor_x;
	const GEN_FLT x22 = obj_pz + x13 * sensor_z + x17 * x16 + x20 * x21;
	const GEN_FLT x23 = -x14 + x15;
	const GEN_FLT x24 = x23 * x12;
	const GEN_FLT x25 = 1 - x12 * (x10 + x8);
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x26 + x27;
	const GEN_FLT x29 = obj_py + x21 * x28 + x24 * sensor_z + x25 * sensor_y;
	const GEN_FLT x30 = lh_qw * lh_qi;
	const GEN_FLT x31 = lh_qk * lh_qj;
	const GEN_FLT x32 = x30 + x31;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = x18 + x19;
	const GEN_FLT x35 = x34 * x12;
	const GEN_FLT x36 = -x26 + x27;
	const GEN_FLT x37 = 1 - x12 * (x10 + x7);
	const GEN_FLT x38 = obj_px + x35 * sensor_z + x36 * x17 + x37 * sensor_x;
	const GEN_FLT x39 = lh_qw * lh_qj;
	const GEN_FLT x40 = lh_qk * lh_qi;
	const GEN_FLT x41 = -x39 + x40;
	const GEN_FLT x42 = x5 * x41;
	const GEN_FLT x43 = lh_pz + x33 * x29 + x42 * x38 + x6 * x22;
	const GEN_FLT x44 = pow(x43, -1);
	const GEN_FLT x45 = 1 - x3 * x5;
	const GEN_FLT x46 = lh_qw * lh_qk;
	const GEN_FLT x47 = lh_qj * lh_qi;
	const GEN_FLT x48 = -x46 + x47;
	const GEN_FLT x49 = 4 * x4 * x11;
	const GEN_FLT x50 = x49 * x28;
	const GEN_FLT x51 = x39 + x40;
	const GEN_FLT x52 = x51 * x49;
	const GEN_FLT x53 = x45 * x37 + x50 * x48 + x52 * x20;
	const GEN_FLT x54 = x6 * x12;
	const GEN_FLT x55 = x42 * x37 + x50 * x32 + x54 * x20;
	const GEN_FLT x56 = pow(x43, 2);
	const GEN_FLT x57 = pow(x56, -1);
	const GEN_FLT x58 = x5 * x22;
	const GEN_FLT x59 = x5 * x48;
	const GEN_FLT x60 = lh_px + x45 * x38 + x51 * x58 + x59 * x29;
	const GEN_FLT x61 = x60 * x57;
	const GEN_FLT x62 = -x43;
	const GEN_FLT x63 = pow(x60, 2);
	const GEN_FLT x64 = 2 * x56 * atan2(x60, x62) * curve_1 / (x56 + x63);
	const GEN_FLT x65 = x46 + x47;
	const GEN_FLT x66 = x5 * x65;
	const GEN_FLT x67 = 1 - (x1 + x2) * x5;
	const GEN_FLT x68 = -x30 + x31;
	const GEN_FLT x69 = x68 * x49;
	const GEN_FLT x70 = x66 * x37 + x69 * x20 + x67 * x28 * x12;
	const GEN_FLT x71 = lh_py + x66 * x38 + x67 * x29 + x68 * x58;
	const GEN_FLT x72 = x71 * x57;
	const GEN_FLT x73 = x56 + pow(x71, 2);
	const GEN_FLT x74 = pow(x73, -1);
	const GEN_FLT x75 = x74 * x56;
	const GEN_FLT x76 = pow(1 - x74 * x63 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x77 = tilt_1 / sqrt(x73);
	const GEN_FLT x78 = 2 * x71;
	const GEN_FLT x79 = 2 * x43;
	const GEN_FLT x80 = (1.0 / 2.0) * x60 * tilt_1 / pow(x73, 3.0 / 2.0);
	const GEN_FLT x81 = -x76 * (x77 * x53 - (x70 * x78 + x79 * x55) * x80) - (x70 * x44 - x72 * x55) * x75;
	const GEN_FLT x82 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x77 * x60) - atan2(-x71, x62)) * gibMag_1;
	const GEN_FLT x83 = x52 * x16 + x59 * x25 + x45 * x36 * x12;
	const GEN_FLT x84 = x41 * x49;
	const GEN_FLT x85 = x33 * x25 + x54 * x16 + x84 * x36;
	const GEN_FLT x86 = x65 * x49;
	const GEN_FLT x87 = x67 * x25 + x69 * x16 + x86 * x36;
	const GEN_FLT x88 = -x76 * (x83 * x77 - (x85 * x79 + x87 * x78) * x80) - (-x85 * x72 + x87 * x44) * x75;
	const GEN_FLT x89 = x49 * x23;
	const GEN_FLT x90 = x5 * x13;
	const GEN_FLT x91 = x45 * x35 + x51 * x90 + x89 * x48;
	const GEN_FLT x92 = x6 * x13 + x84 * x34 + x89 * x32;
	const GEN_FLT x93 = x67 * x24 + x68 * x90 + x86 * x34;
	const GEN_FLT x94 = -x76 * (x77 * x91 - (x78 * x93 + x79 * x92) * x80) - (-x72 * x92 + x93 * x44) * x75;
	*(out++) = x81 + x81 * x82 + (-x53 * x44 + x61 * x55) * x64;
	*(out++) = x88 + x82 * x88 + (-x83 * x44 + x85 * x61) * x64;
	*(out++) = x94 + x82 * x94 + (x61 * x92 - x91 * x44) * x64;
}

// Jacobian of reproject_axis_y wrt [lh_px, lh_py, lh_pz, lh_qw, lh_qi, lh_qj, lh_qk]
static inline void gen_reproject_axis_y_jac_lh_p(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
												 const SurvivePose *lh_p, const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qk, 2);
	const GEN_FLT x4 = x0 + x3;
	const GEN_FLT x5 = sqrt(x1 + x4 + pow(lh_qw, 2));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = x7 + x8;
	const GEN_FLT x10 = pow(obj_qk, 2);
	const GEN_FLT x11 = 2 * sqrt(x10 + x9 + pow(obj_qw, 2));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 = obj_pz + (1 - x9 * x11) * sensor_z + (x12 + x13) * x14 + (-x15 + x16) * x17;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + (1 - x11 * (x10 + x8)) * sensor_y + (-x12 + x13) * x22 + (x23 + x24) * x17;
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = -x27 + x28;
	const GEN_FLT x30 = obj_px + (1 - x11 * (x10 + x7)) * sensor_x + (x15 + x16) * x22 + (-x23 + x24) * x14;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + x18 * (1 - x2 * x6) + x21 * x26 + x31 * x29;
	const GEN_FLT x33 = pow(x32, 2);
	const GEN_FLT x34 = -x19 + x20;
	const GEN_FLT x35 = x6 * x18;
	const GEN_FLT x36 = x1 + x3;
	const GEN_FLT x37 = lh_qw * lh_qk;
	const GEN_FLT x38 = lh_qj * lh_qi;
	const GEN_FLT x39 = x37 + x38;
	const GEN_FLT x40 = lh_py + x25 * (1 - x6 * x36) + x31 * x39 + x34 * x35;
	const GEN_FLT x41 = x33 + pow(x40, 2);
	const GEN_FLT x42 = pow(x41, -1);
	const GEN_FLT x43 = x27 + x28;
	const GEN_FLT x44 = -x37 + x38;
	const GEN_FLT x45 = lh_px + x30 * (1 - x4 * x6) + x43 * x35 + x44 * x26;
	const GEN_FLT x46 = pow(x45, 2);
	const GEN_FLT x47 = pow(1 - x42 * x46 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x48 = tilt_1 / sqrt(x41);
	const GEN_FLT x49 = x47 * x48;
	const GEN_FLT x50 = 2 * x32;
	const GEN_FLT x51 = -x32;
	const GEN_FLT x52 = atan2(x45, x51) * curve_1 / (x33 + x46);
	const GEN_FLT x53 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x45 * x48) - atan2(-x40, x51)) * gibMag_1;
	const GEN_FLT x54 = x45 * tilt_1 / pow(x41, 3.0 / 2.0);
	const GEN_FLT x55 = x54 * x47;
	const GEN_FLT x56 = -x42 * x32 + x55 * x40;
	const GEN_FLT x57 = 2 * x52;
	const GEN_FLT x58 = x40 * x42 + x55 * x32;
	const GEN_FLT x59 = pow(x32, -1);
	const GEN_FLT x60 = 2 / x5;
	const GEN_FLT x61 = x4 * x60;
	const GEN_FLT x62 = x61 * x30;
	const GEN_FLT x63 = x44 * x25;
	const GEN_FLT x64 = x60 * x63;
	const GEN_FLT x65 = x26 * lh_qk;
	const GEN_FLT x66 = x60 * x18;
	const GEN_FLT x67 = x66 * x43;
	const GEN_FLT x68 = x35 * lh_qj;
	const GEN_FLT x69 = -x65 + x68 - x62 * lh_qw + x64 * lh_qw + x67 * lh_qw;
	const GEN_FLT x70 = x30 * x29;
	const GEN_FLT x71 = x70 * x60;
	const GEN_FLT x72 = x25 * x21;
	const GEN_FLT x73 = x72 * x60;
	const GEN_FLT x74 = x26 * lh_qi;
	const GEN_FLT x75 = x31 * lh_qj;
	const GEN_FLT x76 = x2 * x60;
	const GEN_FLT x77 = x76 * x18;
	const GEN_FLT x78 = x74 - x75 + x71 * lh_qw + x73 * lh_qw - x77 * lh_qw;
	const GEN_FLT x79 = pow(x33, -1);
	const GEN_FLT x80 = x79 * x45;
	const GEN_FLT x81 = x57 * x33;
	const GEN_FLT x82 = x30 * x39;
	const GEN_FLT x83 = x82 * x60;
	const GEN_FLT x84 = x31 * lh_qk;
	const GEN_FLT x85 = x60 * x36;
	const GEN_FLT x86 = x85 * x25;
	const GEN_FLT x87 = x66 * x34;
	const GEN_FLT x88 = x35 * lh_qi;
	const GEN_FLT x89 = x84 - x88 + x83 * lh_qw - x86 * lh_qw + x87 * lh_qw;
	const GEN_FLT x90 = x79 * x40;
	const GEN_FLT x91 = x42 * x33;
	const GEN_FLT x92 = 2 * x40;
	const GEN_FLT x93 = (1.0 / 2.0) * x54;
	const GEN_FLT x94 = -x47 * (x69 * x48 - (x78 * x50 + x89 * x92) * x93) - (-x78 * x90 + x89 * x59) * x91;
	const GEN_FLT x95 = x26 * lh_qj;
	const GEN_FLT x96 = x35 * lh_qk;
	const GEN_FLT x97 = x95 + x96 - x62 * lh_qi + x64 * lh_qi + x67 * lh_qi;
	const GEN_FLT x98 = x26 * lh_qw;
	const GEN_FLT x99 = 4 * x5;
	const GEN_FLT x100 = -x99 * lh_qi;
	const GEN_FLT x101 = x84 + x98 + x18 * (x100 - x76 * lh_qi) + x71 * lh_qi + x73 * lh_qi;
	const GEN_FLT x102 = x79 * x101;
	const GEN_FLT x103 = x35 * lh_qw;
	const GEN_FLT x104 = -x103 + x75 + x25 * (x100 - x85 * lh_qi) + x83 * lh_qi + x87 * lh_qi;
	const GEN_FLT x105 = -x47 * (x97 * x48 - (x50 * x101 + x92 * x104) * x93) - (-x40 * x102 + x59 * x104) * x91;
	const GEN_FLT x106 = -x99 * lh_qj;
	const GEN_FLT x107 = x60 * lh_qj;
	const GEN_FLT x108 = x103 + x74 + x30 * (x106 - x61 * lh_qj) + x63 * x107 + x67 * lh_qj;
	const GEN_FLT x109 = x31 * lh_qw;
	const GEN_FLT x110 = -x109 + x65 + x18 * (x106 - x76 * lh_qj) + x70 * x107 + x72 * x107;
	const GEN_FLT x111 = x31 * lh_qi;
	const GEN_FLT x112 = x111 + x96 + x83 * lh_qj - x86 * lh_qj + x87 * lh_qj;
	const GEN_FLT x113 = -x47 * (x48 * x108 - (x50 * x110 + x92 * x112) * x93) - (x59 * x112 - x90 * x110) * x91;
	const GEN_FLT x114 = -x99 * lh_qk;
	const GEN_FLT x115 = x60 * lh_qk;
	const GEN_FLT x116 = x88 - x98 + x30 * (x114 - x61 * lh_qk) + x63 * x115 + x67 * lh_qk;
	const GEN_FLT x117 = x111 + x95 + x70 * x115 + x72 * x115 - x77 * lh_qk;
	const GEN_FLT x118 = x109 + x68 + x25 * (x114 - x85 * lh_qk) + x82 * x115 + x87 * lh_qk;
	const GEN_FLT x119 = -x47 * (x48 * x116 - (x50 * x117 + x92 * x118) * x93) - (x59 * x118 - x90 * x117) * x91;
	*(out++) = -x49 - x50 * x52 - x53 * x49;
	*(out++) = x56 + x53 * x56;
	*(out++) = x58 + x53 * x58 + x57 * x45;
	*(out++) = x94 + x53 * x94 + (-x69 * x59 + x80 * x78) * x81;
	*(out++) = x105 + x53 * x105 + x81 * (x45 * x102 - x59 * x97);
	*(out++) = x113 + x53 * x113 + (-x59 * x108 + x80 * x110) * x81;
	*(out++) = x119 + x53 * x119 + (-x59 * x116 + x80 * x117) * x81;
}

// Jacobian of reproject_axis_y wrt [phase_1, tilt_1, curve_1, gibPhase_1, gibMag_1, ogeeMag_1, ogeePhase_1]
static inline void gen_reproject_axis_y_jac_bsc1(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
												 const SurvivePose *lh_p, const BaseStationCal *bsc1) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT lh_px = (*lh_p).Pos[0];
	const GEN_FLT lh_py = (*lh_p).Pos[1];
	const GEN_FLT lh_pz = (*lh_p).Pos[2];
	const GEN_FLT lh_qw = (*lh_p).Rot[0];
	const GEN_FLT lh_qi = (*lh_p).Rot[1];
	const GEN_FLT lh_qj = (*lh_p).Rot[2];
	const GEN_FLT lh_qk = (*lh_p).Rot[3];
	const GEN_FLT phase_1 = (*bsc1).phase;
	const GEN_FLT tilt_1 = (*bsc1).tilt;
	const GEN_FLT curve_1 = (*bsc1).curve;
	const GEN_FLT gibPhase_1 = (*bsc1).gibpha;
	const GEN_FLT gibMag_1 = (*bsc1).gibmag;
	const GEN_FLT ogeeMag_1 = (*bsc1).ogeephase;
	const GEN_FLT ogeePhase_1 = (*bsc1).ogeemag;
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = pow(lh_qk, 2);
	const GEN_FLT x3 = x1 + x2;
	const GEN_FLT x4 = 2 * sqrt(x0 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = pow(obj_qj, 2);
	const GEN_FLT x6 = pow(obj_qi, 2);
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = x6 + x7;
	const GEN_FLT x9 = 2 * sqrt(x5 + x8 + pow(obj_qw, 2));
	const GEN_FLT x10 = obj_qw * obj_qi;
	const GEN_FLT x11 = obj_qk * obj_qj;
	const GEN_FLT x12 = x9 * sensor_y;
	const GEN_FLT x13 = obj_qw * obj_qj;
	const GEN_FLT x14 = obj_qk * obj_qi;
	const GEN_FLT x15 = x9 * sensor_x;
	const GEN_FLT x16 = obj_pz + (1 - (x5 + x6) * x9) * sensor_z + (x10 + x11) * x12 + (-x13 + x14) * x15;
	const GEN_FLT x17 = lh_qw * lh_qi;
	const GEN_FLT x18 = lh_qk * lh_qj;
	const GEN_FLT x19 = x9 * sensor_z;
	const GEN_FLT x20 = obj_qw * obj_qk;
	const GEN_FLT x21 = obj_qj * obj_qi;
	const GEN_FLT x22 = obj_py + (1 - x8 * x9) * sensor_y + (-x10 + x11) * x19 + (x20 + x21) * x15;
	const GEN_FLT x23 = x4 * x22;
	const GEN_FLT x24 = lh_qw * lh_qj;
	const GEN_FLT x25 = lh_qk * lh_qi;
	const GEN_FLT x26 = obj_px + (1 - (x5 + x7) * x9) * sensor_x + (x13 + x14) * x19 + (-x20 + x21) * x12;
	const GEN_FLT x27 = x4 * x26;
	const GEN_FLT x28 = lh_pz + x16 * (1 - (x0 + x1) * x4) + (x17 + x18) * x23 + (-x24 + x25) * x27;
	const GEN_FLT x29 = x4 * x16;
	const GEN_FLT x30 = lh_qw * lh_qk;
	const GEN_FLT x31 = lh_qj * lh_qi;
	const GEN_FLT x32 = lh_py + x22 * (1 - x4 * x3) + (-x17 + x18) * x29 + (x30 + x31) * x27;
	const GEN_FLT x33 = pow(x28, 2) + pow(x32, 2);
	const GEN_FLT x34 = lh_px + x26 * (1 - (x0 + x2) * x4) + (x24 + x25) * x29 + (-x30 + x31) * x23;
	const GEN_FLT x35 = x34 / sqrt(x33);
	const GEN_FLT x36 = -x28;
	const GEN_FLT x37 = 1.5707963267949 + gibPhase_1 - phase_1 - asin(x35 * tilt_1) - atan2(-x32, x36);
	const GEN_FLT x38 = sin(x37) * gibMag_1;
	const GEN_FLT x39 = x35 / sqrt(1 - pow(x34, 2) * pow(tilt_1, 2) / x33);
	*(out++) = -1 - x38;
	*(out++) = -x39 - x38 * x39;
	*(out++) = pow(atan2(x34, x36), 2);
	*(out++) = x38;
	*(out++) = -cos(x37);
	*(out++) = 0;
	*(out++) = 0;
}
