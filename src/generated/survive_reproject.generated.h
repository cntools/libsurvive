#pragma once
#include "common.h"
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
	const GEN_FLT x2 = pow(lh_qk, 2);
	const GEN_FLT x3 = pow(lh_qj, 2);
	const GEN_FLT x4 = pow(lh_qi, 2);
	const GEN_FLT x5 = 1e-10 + x4 + x3 + x2;
	const GEN_FLT x6 = pow(x5, 1.0 / 2.0);
	const GEN_FLT x7 = pow(x6, -1) * sin(x6);
	const GEN_FLT x8 = x7 * lh_qi;
	const GEN_FLT x9 = cos(x6);
	const GEN_FLT x10 = pow(x5, -1) * (1 + (-1 * x9));
	const GEN_FLT x11 = x10 * lh_qj;
	const GEN_FLT x12 = x11 * lh_qk;
	const GEN_FLT x13 = pow(obj_qk, 2);
	const GEN_FLT x14 = pow(obj_qj, 2);
	const GEN_FLT x15 = pow(obj_qi, 2);
	const GEN_FLT x16 = 1e-10 + x15 + x14 + x13;
	const GEN_FLT x17 = pow(x16, 1.0 / 2.0);
	const GEN_FLT x18 = cos(x17);
	const GEN_FLT x19 = pow(x16, -1) * (1 + (-1 * x18));
	const GEN_FLT x20 = pow(x17, -1) * sin(x17);
	const GEN_FLT x21 = x20 * obj_qi;
	const GEN_FLT x22 = x19 * obj_qk * obj_qj;
	const GEN_FLT x23 = x20 * obj_qj;
	const GEN_FLT x24 = x19 * obj_qi;
	const GEN_FLT x25 = x24 * obj_qk;
	const GEN_FLT x26 =
		((x25 + (-1 * x23)) * sensor_x) + ((x22 + x21) * sensor_y) + ((x18 + (x13 * x19)) * sensor_z) + obj_pz;
	const GEN_FLT x27 = x20 * obj_qk;
	const GEN_FLT x28 = x24 * obj_qj;
	const GEN_FLT x29 =
		((x28 + x27) * sensor_x) + ((x18 + (x14 * x19)) * sensor_y) + ((x22 + (-1 * x21)) * sensor_z) + obj_py;
	const GEN_FLT x30 =
		((x18 + (x15 * x19)) * sensor_x) + ((x28 + (-1 * x27)) * sensor_y) + ((x25 + x23) * sensor_z) + obj_px;
	const GEN_FLT x31 = x7 * lh_qk;
	const GEN_FLT x32 = x11 * lh_qi;
	const GEN_FLT x33 = ((x32 + x31) * x30) + (x29 * (x9 + (x3 * x10))) + lh_py + (x26 * (x12 + (-1 * x8)));
	const GEN_FLT x34 = x7 * lh_qj;
	const GEN_FLT x35 = x10 * lh_qk * lh_qi;
	const GEN_FLT x36 = ((x35 + (-1 * x34)) * x30) + (x29 * (x12 + x8)) + (x26 * (x9 + (x2 * x10))) + lh_pz;
	const GEN_FLT x37 = (x30 * (x9 + (x4 * x10))) + ((x32 + (-1 * x31)) * x29) + ((x35 + x34) * x26) + lh_px;
	const GEN_FLT x38 = pow(x37, 2) + pow(x36, 2);
	const GEN_FLT x39 = x33 * pow((x38 + pow(x33, 2)), -1.0 / 2.0);
	const GEN_FLT x40 = asin(pow(x1, -1) * x39);
	const GEN_FLT x41 = 0.0028679863 + (x40 * (-8.0108022e-06 + (-8.0108022e-06 * x40)));
	const GEN_FLT x42 = 5.3685255e-06 + (x40 * x41);
	const GEN_FLT x43 = 0.0076069798 + (x40 * x42);
	const GEN_FLT x44 = x33 * pow(x38, -1.0 / 2.0);
	const GEN_FLT x45 = tan(x0) * x44;
	const GEN_FLT x46 = atan2(-1 * x36, x37);
	const GEN_FLT x47 = (sin(x46 + (-1 * asin(x45)) + ogeeMag_0) * ogeePhase_0) + curve_0;
	const GEN_FLT x48 = asin(
		x45 +
		(pow(x40, 2) * x43 * x47 *
		 pow((x1 +
			  (-1 * sin(x0) * x47 *
			   ((x40 * (x43 + (x40 * (x42 + (x40 * (x41 + (x40 * (-8.0108022e-06 + (-1.60216044e-05 * x40))))))))) +
				(x40 * x43)))),
			 -1)));
	const GEN_FLT x49 = -1 * x46;
	const GEN_FLT x50 = -1.5707963267949 + x46;
	const GEN_FLT x51 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x52 = -1 * x44 * tan(x51);
	const GEN_FLT x53 = (sin(x46 + ogeeMag_1 + (-1 * asin(x52))) * ogeePhase_1) + curve_1;
	const GEN_FLT x54 = cos(x51);
	const GEN_FLT x55 = asin(pow(x54, -1) * x39);
	const GEN_FLT x56 = 0.0028679863 + (x55 * (-8.0108022e-06 + (-8.0108022e-06 * x55)));
	const GEN_FLT x57 = 5.3685255e-06 + (x56 * x55);
	const GEN_FLT x58 = 0.0076069798 + (x57 * x55);
	const GEN_FLT x59 = asin(
		x52 +
		(x53 * x58 * pow(x55, 2) *
		 pow((x54 +
			  (x53 * sin(x51) *
			   ((x55 * (x58 + (x55 * (x57 + (x55 * (x56 + (x55 * (-8.0108022e-06 + (-1.60216044e-05 * x55))))))))) +
				(x58 * x55)))),
			 -1)));
	out[0] = x50 + (-1 * x48) + (-1 * sin(x49 + x48 + (-1 * gibPhase_0)) * gibMag_0) + (-1 * phase_0);
	out[1] = x50 + (-1 * sin(x49 + x59 + (-1 * gibPhase_1)) * gibMag_1) + (-1 * x59) + (-1 * phase_1);
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qj;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qk * lh_qi;
	const GEN_FLT x10 = x9 + (-1 * x6);
	const GEN_FLT x11 = x9 + x6;
	const GEN_FLT x12 = pow(obj_qk, 2);
	const GEN_FLT x13 = pow(obj_qj, 2);
	const GEN_FLT x14 = pow(obj_qi, 2);
	const GEN_FLT x15 = 1e-10 + x14 + x13 + x12;
	const GEN_FLT x16 = pow(x15, -1);
	const GEN_FLT x17 = pow(x15, 1.0 / 2.0);
	const GEN_FLT x18 = cos(x17);
	const GEN_FLT x19 = 1 + (-1 * x18);
	const GEN_FLT x20 = x19 * x16;
	const GEN_FLT x21 = sin(x17);
	const GEN_FLT x22 = x21 * pow(x17, -1);
	const GEN_FLT x23 = x22 * obj_qi;
	const GEN_FLT x24 = x20 * obj_qj;
	const GEN_FLT x25 = x24 * obj_qk;
	const GEN_FLT x26 = x22 * obj_qj;
	const GEN_FLT x27 = -1 * x26;
	const GEN_FLT x28 = x20 * obj_qi;
	const GEN_FLT x29 = x28 * obj_qk;
	const GEN_FLT x30 = ((x29 + x27) * sensor_x) + ((x25 + x23) * sensor_y) + ((x18 + (x20 * x12)) * sensor_z) + obj_pz;
	const GEN_FLT x31 = -1 * x23;
	const GEN_FLT x32 = x22 * obj_qk;
	const GEN_FLT x33 = x24 * obj_qi;
	const GEN_FLT x34 = ((x33 + x32) * sensor_x) + ((x25 + x31) * sensor_z) + ((x18 + (x20 * x13)) * sensor_y) + obj_py;
	const GEN_FLT x35 = x5 * lh_qk;
	const GEN_FLT x36 = x8 * lh_qj;
	const GEN_FLT x37 = x36 * lh_qi;
	const GEN_FLT x38 = x37 + (-1 * x35);
	const GEN_FLT x39 = -1 * x32;
	const GEN_FLT x40 = ((x18 + (x20 * x14)) * sensor_x) + ((x33 + x39) * sensor_y) + ((x29 + x26) * sensor_z) + obj_px;
	const GEN_FLT x41 = x7 + (x2 * x8);
	const GEN_FLT x42 = (x40 * x41) + (x30 * x11) + (x34 * x38) + lh_px;
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = x7 + (x0 * x8);
	const GEN_FLT x45 = x5 * lh_qi;
	const GEN_FLT x46 = x36 * lh_qk;
	const GEN_FLT x47 = x46 + x45;
	const GEN_FLT x48 = (x40 * x10) + (x47 * x34) + (x44 * x30) + lh_pz;
	const GEN_FLT x49 = pow(x42, 2);
	const GEN_FLT x50 = x48 * pow(x49, -1);
	const GEN_FLT x51 = x49 + pow(x48, 2);
	const GEN_FLT x52 = pow(x51, -1);
	const GEN_FLT x53 = x52 * x49;
	const GEN_FLT x54 = ((x50 * x41) + (-1 * x43 * x10)) * x53;
	const GEN_FLT x55 = x46 + (-1 * x45);
	const GEN_FLT x56 = x7 + (x1 * x8);
	const GEN_FLT x57 = x37 + x35;
	const GEN_FLT x58 = (x57 * x40) + lh_py + (x56 * x34) + (x55 * x30);
	const GEN_FLT x59 = pow(x58, 2);
	const GEN_FLT x60 = x51 + x59;
	const GEN_FLT x61 = pow(x60, -1.0 / 2.0);
	const GEN_FLT x62 = 0.523598775598299 + tilt_0;
	const GEN_FLT x63 = cos(x62);
	const GEN_FLT x64 = pow(x63, -1);
	const GEN_FLT x65 = x64 * x61;
	const GEN_FLT x66 = asin(x65 * x58);
	const GEN_FLT x67 = 8.0108022e-06 * x66;
	const GEN_FLT x68 = -8.0108022e-06 + (-1 * x67);
	const GEN_FLT x69 = 0.0028679863 + (x68 * x66);
	const GEN_FLT x70 = 5.3685255e-06 + (x66 * x69);
	const GEN_FLT x71 = 0.0076069798 + (x70 * x66);
	const GEN_FLT x72 = x71 * x66;
	const GEN_FLT x73 = -8.0108022e-06 + (-1.60216044e-05 * x66);
	const GEN_FLT x74 = x69 + (x73 * x66);
	const GEN_FLT x75 = x70 + (x74 * x66);
	const GEN_FLT x76 = x71 + (x75 * x66);
	const GEN_FLT x77 = (x76 * x66) + x72;
	const GEN_FLT x78 = pow(x51, -1.0 / 2.0);
	const GEN_FLT x79 = tan(x62);
	const GEN_FLT x80 = x79 * x78;
	const GEN_FLT x81 = x80 * x58;
	const GEN_FLT x82 = atan2(-1 * x48, x42);
	const GEN_FLT x83 = x82 + (-1 * asin(x81)) + ogeeMag_0;
	const GEN_FLT x84 = (sin(x83) * ogeePhase_0) + curve_0;
	const GEN_FLT x85 = sin(x62);
	const GEN_FLT x86 = x84 * x85;
	const GEN_FLT x87 = x63 + (-1 * x86 * x77);
	const GEN_FLT x88 = pow(x87, -1);
	const GEN_FLT x89 = pow(x66, 2);
	const GEN_FLT x90 = x89 * x84;
	const GEN_FLT x91 = x88 * x90;
	const GEN_FLT x92 = x81 + (x71 * x91);
	const GEN_FLT x93 = pow((1 + (-1 * pow(x92, 2))), -1.0 / 2.0);
	const GEN_FLT x94 = pow(x60, -1) * x59;
	const GEN_FLT x95 = pow((1 + (-1 * pow(x63, -2) * x94)), -1.0 / 2.0);
	const GEN_FLT x96 = 2 * x58;
	const GEN_FLT x97 = 2 * x42;
	const GEN_FLT x98 = 2 * x48;
	const GEN_FLT x99 = (x98 * x10) + (x97 * x41);
	const GEN_FLT x100 = x99 + (x57 * x96);
	const GEN_FLT x101 = 1.0 / 2.0 * x58;
	const GEN_FLT x102 = pow(x60, -3.0 / 2.0) * x101;
	const GEN_FLT x103 = x64 * x102;
	const GEN_FLT x104 = (x65 * x57) + (-1 * x100 * x103);
	const GEN_FLT x105 = x95 * x104;
	const GEN_FLT x106 = 2 * x88 * x84 * x72;
	const GEN_FLT x107 = x68 * x95;
	const GEN_FLT x108 = x104 * x107;
	const GEN_FLT x109 = 2.40324066e-05 * x66;
	const GEN_FLT x110 = (x66 * (x108 + (-1 * x67 * x105))) + (x69 * x105);
	const GEN_FLT x111 = (x66 * x110) + (x70 * x105);
	const GEN_FLT x112 = x52 * x59;
	const GEN_FLT x113 = pow((1 + (-1 * pow(x79, 2) * x112)), -1.0 / 2.0);
	const GEN_FLT x114 = pow(x51, -3.0 / 2.0) * x101;
	const GEN_FLT x115 = x99 * x114;
	const GEN_FLT x116 = x78 * x57;
	const GEN_FLT x117 = (x79 * x116) + (-1 * x79 * x115);
	const GEN_FLT x118 = cos(x83) * ogeePhase_0;
	const GEN_FLT x119 = x118 * ((-1 * x113 * x117) + x54);
	const GEN_FLT x120 = x85 * x77;
	const GEN_FLT x121 = pow(x87, -2) * x71 * x90;
	const GEN_FLT x122 = x88 * x89 * x71;
	const GEN_FLT x123 =
		x93 * (x117 + (x91 * x111) + (x119 * x122) +
			   (-1 * x121 *
				((-1 * x119 * x120) +
				 (-1 * x86 *
				  ((x66 * x111) + (x71 * x105) +
				   (x66 * (x111 + (x66 * ((x74 * x105) + x110 + (x66 * ((x73 * x105) + (-1 * x109 * x105) + x108)))) +
						   (x75 * x105))) +
				   (x76 * x105))))) +
			   (x105 * x106));
	const GEN_FLT x124 = -1 * x54;
	const GEN_FLT x125 = -1 * x82;
	const GEN_FLT x126 = cos(x125 + asin(x92) + (-1 * gibPhase_0)) * gibMag_0;
	const GEN_FLT x127 = ((x50 * x38) + (-1 * x43 * x47)) * x53;
	const GEN_FLT x128 = (x98 * x47) + (x97 * x38);
	const GEN_FLT x129 = x128 + (x56 * x96);
	const GEN_FLT x130 = (x65 * x56) + (-1 * x103 * x129);
	const GEN_FLT x131 = x95 * x130;
	const GEN_FLT x132 = x107 * x130;
	const GEN_FLT x133 = (x66 * (x132 + (-1 * x67 * x131))) + (x69 * x131);
	const GEN_FLT x134 = (x66 * x133) + (x70 * x131);
	const GEN_FLT x135 = x79 * x114;
	const GEN_FLT x136 = (x80 * x56) + (-1 * x128 * x135);
	const GEN_FLT x137 = (-1 * x113 * x136) + x127;
	const GEN_FLT x138 = x118 * x120;
	const GEN_FLT x139 = x118 * x122;
	const GEN_FLT x140 =
		x93 * (x136 + (x137 * x139) +
			   (-1 * x121 *
				((-1 * x137 * x138) +
				 (-1 * x86 *
				  ((x66 * x134) + (x71 * x131) +
				   (x66 * (x134 + (x66 * (x133 + (x74 * x131) + (x66 * ((x73 * x131) + (-1 * x109 * x131) + x132)))) +
						   (x75 * x131))) +
				   (x76 * x131))))) +
			   (x91 * x134) + (x106 * x131));
	const GEN_FLT x141 = -1 * x127;
	const GEN_FLT x142 = ((x50 * x11) + (-1 * x43 * x44)) * x53;
	const GEN_FLT x143 = (x98 * x44) + (x97 * x11);
	const GEN_FLT x144 = x143 + (x55 * x96);
	const GEN_FLT x145 = (x65 * x55) + (-1 * x103 * x144);
	const GEN_FLT x146 = x95 * x145;
	const GEN_FLT x147 = (x80 * x55) + (-1 * x135 * x143);
	const GEN_FLT x148 = (-1 * x113 * x147) + x142;
	const GEN_FLT x149 = x107 * x145;
	const GEN_FLT x150 = (x66 * (x149 + (-1 * x67 * x146))) + (x69 * x146);
	const GEN_FLT x151 = (x66 * x150) + (x70 * x146);
	const GEN_FLT x152 =
		x93 * (x147 +
			   (-1 * x121 *
				((-1 * x138 * x148) +
				 (-1 * x86 *
				  ((x66 * x151) + (x71 * x146) +
				   (x66 * (x151 + (x66 * (x150 + (x74 * x146) + (x66 * ((x73 * x146) + (-1 * x109 * x146) + x149)))) +
						   (x75 * x146))) +
				   (x76 * x146))))) +
			   (x91 * x151) + (x139 * x148) + (x106 * x146));
	const GEN_FLT x153 = -1 * x142;
	const GEN_FLT x154 = pow(obj_qi, 3);
	const GEN_FLT x155 = x21 * pow(x15, -3.0 / 2.0);
	const GEN_FLT x156 = 2 * pow(x15, -2) * x19;
	const GEN_FLT x157 = x18 * x16;
	const GEN_FLT x158 = x157 * obj_qi;
	const GEN_FLT x159 = x158 * obj_qk;
	const GEN_FLT x160 = x155 * obj_qk * obj_qi;
	const GEN_FLT x161 = x160 + (-1 * x159);
	const GEN_FLT x162 = x14 * x155;
	const GEN_FLT x163 = x156 * obj_qj;
	const GEN_FLT x164 = (-1 * x14 * x163) + (x162 * obj_qj);
	const GEN_FLT x165 = x164 + x24;
	const GEN_FLT x166 = x20 * obj_qk;
	const GEN_FLT x167 = x156 * obj_qk;
	const GEN_FLT x168 = (-1 * x14 * x167) + (x162 * obj_qk);
	const GEN_FLT x169 = x168 + x166;
	const GEN_FLT x170 = x158 * obj_qj;
	const GEN_FLT x171 = x155 * obj_qj;
	const GEN_FLT x172 = x171 * obj_qi;
	const GEN_FLT x173 = (-1 * x172) + x170;
	const GEN_FLT x174 = ((x173 + x169) * sensor_z) + ((x165 + x161) * sensor_y) +
						 (((-1 * x154 * x156) + (x154 * x155) + (2 * x28) + x31) * sensor_x);
	const GEN_FLT x175 = (-1 * x160) + x159;
	const GEN_FLT x176 = x13 * x155;
	const GEN_FLT x177 = x156 * obj_qi;
	const GEN_FLT x178 = (-1 * x13 * x177) + (x176 * obj_qi);
	const GEN_FLT x179 = x14 * x157;
	const GEN_FLT x180 = x171 * obj_qk;
	const GEN_FLT x181 = obj_qk * obj_qj;
	const GEN_FLT x182 = (-1 * x177 * x181) + (x180 * obj_qi);
	const GEN_FLT x183 = x182 + (-1 * x22);
	const GEN_FLT x184 =
		((x183 + x162 + (-1 * x179)) * sensor_z) + ((x178 + x31) * sensor_y) + ((x165 + x175) * sensor_x);
	const GEN_FLT x185 = x172 + (-1 * x170);
	const GEN_FLT x186 = x182 + x22;
	const GEN_FLT x187 = x12 * x155;
	const GEN_FLT x188 = (-1 * x12 * x177) + (x187 * obj_qi);
	const GEN_FLT x189 =
		((x188 + x31) * sensor_z) + ((x186 + (-1 * x162) + x179) * sensor_y) + ((x185 + x169) * sensor_x);
	const GEN_FLT x190 = (x47 * x184) + (x44 * x189) + (x10 * x174);
	const GEN_FLT x191 = (x11 * x189) + (x38 * x184) + (x41 * x174);
	const GEN_FLT x192 = ((x50 * x191) + (-1 * x43 * x190)) * x53;
	const GEN_FLT x193 = (x55 * x189) + (x56 * x184) + (x57 * x174);
	const GEN_FLT x194 = (x98 * x190) + (x97 * x191);
	const GEN_FLT x195 = x194 + (x96 * x193);
	const GEN_FLT x196 = x95 * ((x65 * x193) + (-1 * x103 * x195));
	const GEN_FLT x197 = x68 * x196;
	const GEN_FLT x198 = (x66 * (x197 + (-1 * x67 * x196))) + (x69 * x196);
	const GEN_FLT x199 = (x66 * x198) + (x70 * x196);
	const GEN_FLT x200 = (x80 * x193) + (-1 * x194 * x135);
	const GEN_FLT x201 = (-1 * x200 * x113) + x192;
	const GEN_FLT x202 =
		x93 * (x200 + (x91 * x199) + (x201 * x139) +
			   (-1 * x121 *
				((-1 * x201 * x138) +
				 (-1 * x86 *
				  ((x71 * x196) +
				   (x66 * (x199 + (x66 * (x198 + (x74 * x196) + (x66 * ((x73 * x196) + (-1 * x109 * x196) + x197)))) +
						   (x75 * x196))) +
				   (x66 * x199) + (x76 * x196))))) +
			   (x106 * x196));
	const GEN_FLT x203 = -1 * x192;
	const GEN_FLT x204 = x181 * x157;
	const GEN_FLT x205 = (-1 * x180) + x204;
	const GEN_FLT x206 = x178 + x28;
	const GEN_FLT x207 = pow(obj_qj, 3);
	const GEN_FLT x208 = (-1 * x13 * x167) + (x176 * obj_qk);
	const GEN_FLT x209 = x208 + x166;
	const GEN_FLT x210 = ((x209 + x185) * sensor_z) +
						 (((x207 * x155) + (-1 * x207 * x156) + (2 * x24) + x27) * sensor_y) +
						 ((x206 + x205) * sensor_x);
	const GEN_FLT x211 = x180 + (-1 * x204);
	const GEN_FLT x212 = x13 * x157;
	const GEN_FLT x213 =
		((x186 + (-1 * x176) + x212) * sensor_z) + ((x211 + x206) * sensor_y) + ((x164 + x27) * sensor_x);
	const GEN_FLT x214 = (-1 * x12 * x163) + (x187 * obj_qj);
	const GEN_FLT x215 =
		((x214 + x27) * sensor_z) + ((x209 + x173) * sensor_y) + ((x183 + x176 + (-1 * x212)) * sensor_x);
	const GEN_FLT x216 = (x44 * x215) + (x10 * x213) + (x47 * x210);
	const GEN_FLT x217 = (x11 * x215) + (x38 * x210) + (x41 * x213);
	const GEN_FLT x218 = ((x50 * x217) + (-1 * x43 * x216)) * x53;
	const GEN_FLT x219 = (x55 * x215) + (x56 * x210) + (x57 * x213);
	const GEN_FLT x220 = (x98 * x216) + (x97 * x217);
	const GEN_FLT x221 = x220 + (x96 * x219);
	const GEN_FLT x222 = x95 * ((x65 * x219) + (-1 * x221 * x103));
	const GEN_FLT x223 = x68 * x222;
	const GEN_FLT x224 = (x66 * (x223 + (-1 * x67 * x222))) + (x69 * x222);
	const GEN_FLT x225 = (x66 * x224) + (x70 * x222);
	const GEN_FLT x226 = (x80 * x219) + (-1 * x220 * x135);
	const GEN_FLT x227 = (-1 * x226 * x113) + x218;
	const GEN_FLT x228 =
		x93 * (x226 + (x91 * x225) +
			   (-1 * x121 *
				((-1 * x227 * x138) +
				 (-1 * x86 *
				  ((x66 * x225) + (x71 * x222) +
				   (x66 * (x225 + (x66 * (x224 + (x74 * x222) + (x66 * ((x73 * x222) + (-1 * x222 * x109) + x223)))) +
						   (x75 * x222))) +
				   (x76 * x222))))) +
			   (x227 * x139) + (x222 * x106));
	const GEN_FLT x229 = -1 * x218;
	const GEN_FLT x230 = x12 * x157;
	const GEN_FLT x231 = x188 + x28;
	const GEN_FLT x232 =
		((x183 + x187 + (-1 * x230)) * sensor_y) + ((x205 + x231) * sensor_z) + ((x168 + x39) * sensor_x);
	const GEN_FLT x233 = x214 + x24;
	const GEN_FLT x234 =
		((x161 + x233) * sensor_z) + ((x208 + x39) * sensor_y) + ((x186 + (-1 * x187) + x230) * sensor_x);
	const GEN_FLT x235 = pow(obj_qk, 3);
	const GEN_FLT x236 = (((-1 * x235 * x156) + (x235 * x155) + x39 + (2 * x166)) * sensor_z) +
						 ((x175 + x233) * sensor_y) + ((x211 + x231) * sensor_x);
	const GEN_FLT x237 = (x44 * x236) + (x47 * x234) + (x10 * x232);
	const GEN_FLT x238 = (x11 * x236) + (x38 * x234) + (x41 * x232);
	const GEN_FLT x239 = ((x50 * x238) + (-1 * x43 * x237)) * x53;
	const GEN_FLT x240 = (x55 * x236) + (x56 * x234) + (x57 * x232);
	const GEN_FLT x241 = (x98 * x237) + (x97 * x238);
	const GEN_FLT x242 = x241 + (x96 * x240);
	const GEN_FLT x243 = x95 * ((x65 * x240) + (-1 * x242 * x103));
	const GEN_FLT x244 = x68 * x243;
	const GEN_FLT x245 = (x66 * (x244 + (-1 * x67 * x243))) + (x69 * x243);
	const GEN_FLT x246 = (x66 * x245) + (x70 * x243);
	const GEN_FLT x247 = (x80 * x240) + (-1 * x241 * x135);
	const GEN_FLT x248 = (-1 * x247 * x113) + x239;
	const GEN_FLT x249 =
		x93 * ((x91 * x246) + x247 + (x248 * x139) +
			   (-1 * x121 *
				((-1 * x248 * x138) +
				 (-1 * x86 *
				  ((x71 * x243) +
				   (x66 * (x246 + (x66 * (x245 + (x74 * x243) + (x66 * ((x73 * x243) + (-1 * x243 * x109) + x244)))) +
						   (x75 * x243))) +
				   (x66 * x246) + (x76 * x243))))) +
			   (x243 * x106));
	const GEN_FLT x250 = -1 * x239;
	const GEN_FLT x251 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x252 = cos(x251);
	const GEN_FLT x253 = pow(x252, -1);
	const GEN_FLT x254 = x61 * x253;
	const GEN_FLT x255 = asin(x58 * x254);
	const GEN_FLT x256 = 8.0108022e-06 * x255;
	const GEN_FLT x257 = -8.0108022e-06 + (-1 * x256);
	const GEN_FLT x258 = 0.0028679863 + (x255 * x257);
	const GEN_FLT x259 = 5.3685255e-06 + (x255 * x258);
	const GEN_FLT x260 = 0.0076069798 + (x255 * x259);
	const GEN_FLT x261 = x260 * x255;
	const GEN_FLT x262 = -8.0108022e-06 + (-1.60216044e-05 * x255);
	const GEN_FLT x263 = x258 + (x262 * x255);
	const GEN_FLT x264 = x259 + (x263 * x255);
	const GEN_FLT x265 = x260 + (x264 * x255);
	const GEN_FLT x266 = (x265 * x255) + x261;
	const GEN_FLT x267 = tan(x251);
	const GEN_FLT x268 = x78 * x267;
	const GEN_FLT x269 = -1 * x58 * x268;
	const GEN_FLT x270 = x82 + ogeeMag_1 + (-1 * asin(x269));
	const GEN_FLT x271 = (sin(x270) * ogeePhase_1) + curve_1;
	const GEN_FLT x272 = sin(x251);
	const GEN_FLT x273 = x272 * x271;
	const GEN_FLT x274 = x252 + (x273 * x266);
	const GEN_FLT x275 = pow(x274, -1);
	const GEN_FLT x276 = pow(x255, 2);
	const GEN_FLT x277 = x276 * x271;
	const GEN_FLT x278 = x277 * x275;
	const GEN_FLT x279 = x269 + (x278 * x260);
	const GEN_FLT x280 = pow((1 + (-1 * pow(x279, 2))), -1.0 / 2.0);
	const GEN_FLT x281 = pow((1 + (-1 * pow(x267, 2) * x112)), -1.0 / 2.0);
	const GEN_FLT x282 = (-1 * x267 * x116) + (x267 * x115);
	const GEN_FLT x283 = cos(x270) * ogeePhase_1;
	const GEN_FLT x284 = x283 * ((-1 * x282 * x281) + x54);
	const GEN_FLT x285 = x276 * x275 * x260;
	const GEN_FLT x286 = pow((1 + (-1 * x94 * pow(x252, -2))), -1.0 / 2.0);
	const GEN_FLT x287 = x253 * x102;
	const GEN_FLT x288 = x286 * ((x57 * x254) + (-1 * x287 * x100));
	const GEN_FLT x289 = 2 * x275 * x271 * x261;
	const GEN_FLT x290 = x288 * x257;
	const GEN_FLT x291 = (x288 * x258) + (x255 * (x290 + (-1 * x288 * x256)));
	const GEN_FLT x292 = (x288 * x259) + (x291 * x255);
	const GEN_FLT x293 = x272 * x266;
	const GEN_FLT x294 = 2.40324066e-05 * x255;
	const GEN_FLT x295 = x277 * pow(x274, -2) * x260;
	const GEN_FLT x296 =
		x280 *
		(x282 +
		 (-1 * x295 *
		  ((x273 *
			((x292 * x255) + (x260 * x288) +
			 (x255 * (x292 + (x255 * (x291 + (x263 * x288) + (x255 * ((x262 * x288) + (-1 * x294 * x288) + x290)))) +
					  (x264 * x288))) +
			 (x265 * x288))) +
		   (x293 * x284))) +
		 (x278 * x292) + (x288 * x289) + (x284 * x285));
	const GEN_FLT x297 = cos(x125 + asin(x279) + (-1 * gibPhase_1)) * gibMag_1;
	const GEN_FLT x298 = x267 * x114;
	const GEN_FLT x299 = (-1 * x56 * x268) + (x298 * x128);
	const GEN_FLT x300 = (-1 * x299 * x281) + x127;
	const GEN_FLT x301 = x283 * x285;
	const GEN_FLT x302 = x286 * ((x56 * x254) + (-1 * x287 * x129));
	const GEN_FLT x303 = x257 * x302;
	const GEN_FLT x304 = (x258 * x302) + (x255 * (x303 + (-1 * x256 * x302)));
	const GEN_FLT x305 = (x259 * x302) + (x255 * x304);
	const GEN_FLT x306 = x293 * x283;
	const GEN_FLT x307 =
		x280 *
		(x299 +
		 (-1 * x295 *
		  ((x273 *
			((x255 * x305) + (x260 * x302) +
			 (x255 * (x305 + (x255 * (x304 + (x263 * x302) + (x255 * ((x262 * x302) + (-1 * x294 * x302) + x303)))) +
					  (x264 * x302))) +
			 (x265 * x302))) +
		   (x300 * x306))) +
		 (x289 * x302) + (x278 * x305) + (x301 * x300));
	const GEN_FLT x308 = (-1 * x55 * x268) + (x298 * x143);
	const GEN_FLT x309 = (-1 * x281 * x308) + x142;
	const GEN_FLT x310 = (x55 * x254) + (-1 * x287 * x144);
	const GEN_FLT x311 = x265 * x286;
	const GEN_FLT x312 = x286 * x310;
	const GEN_FLT x313 = x257 * x312;
	const GEN_FLT x314 = (x258 * x312) + (x255 * (x313 + (-1 * x256 * x312)));
	const GEN_FLT x315 = (x259 * x312) + (x255 * x314);
	const GEN_FLT x316 =
		x280 *
		((x278 * x315) + x308 + (x289 * x312) +
		 (-1 * x295 *
		  ((x273 *
			((x255 * x315) + (x260 * x312) +
			 (x255 * (x315 + (x255 * (x314 + (x263 * x312) + (x255 * ((x262 * x312) + (-1 * x294 * x312) + x313)))) +
					  (x264 * x312))) +
			 (x311 * x310))) +
		   (x309 * x306))) +
		 (x309 * x301));
	const GEN_FLT x317 = (-1 * x268 * x193) + (x298 * x194);
	const GEN_FLT x318 = (-1 * x281 * x317) + x192;
	const GEN_FLT x319 = (x254 * x193) + (-1 * x287 * x195);
	const GEN_FLT x320 = x286 * x319;
	const GEN_FLT x321 = x257 * x320;
	const GEN_FLT x322 = (x258 * x320) + (x255 * (x321 + (-1 * x256 * x320)));
	const GEN_FLT x323 = (x259 * x320) + (x255 * x322);
	const GEN_FLT x324 =
		x280 *
		(x317 +
		 (-1 * x295 *
		  ((x273 *
			((x255 * x323) +
			 (x255 * (x323 + (x255 * (x322 + (x263 * x320) + (x255 * ((x262 * x320) + (-1 * x294 * x320) + x321)))) +
					  (x264 * x320))) +
			 (x260 * x320) + (x311 * x319))) +
		   (x306 * x318))) +
		 (x278 * x323) + (x289 * x320) + (x301 * x318));
	const GEN_FLT x325 = (-1 * x219 * x268) + (x298 * x220);
	const GEN_FLT x326 = (-1 * x281 * x325) + x218;
	const GEN_FLT x327 = (x219 * x254) + (-1 * x287 * x221);
	const GEN_FLT x328 = x286 * x327;
	const GEN_FLT x329 = x257 * x328;
	const GEN_FLT x330 = (x258 * x328) + (x255 * (x329 + (-1 * x256 * x328)));
	const GEN_FLT x331 = (x259 * x328) + (x255 * x330);
	const GEN_FLT x332 =
		x280 *
		((-1 * x295 *
		  ((x273 *
			((x255 * x331) + (x260 * x328) +
			 (x255 * (x331 + (x255 * (x330 + (x263 * x328) + (x255 * ((x262 * x328) + (-1 * x294 * x328) + x329)))) +
					  (x264 * x328))) +
			 (x327 * x311))) +
		   (x306 * x326))) +
		 x325 + (x278 * x331) + (x289 * x328) + (x301 * x326));
	const GEN_FLT x333 = (-1 * x268 * x240) + (x298 * x241);
	const GEN_FLT x334 = (-1 * x281 * x333) + x239;
	const GEN_FLT x335 = ((x254 * x240) + (-1 * x287 * x242)) * x286;
	const GEN_FLT x336 = x257 * x335;
	const GEN_FLT x337 = (x258 * x335) + (x255 * (x336 + (-1 * x256 * x335)));
	const GEN_FLT x338 = (x259 * x335) + (x255 * x337);
	const GEN_FLT x339 =
		x280 *
		((x278 * x338) + x333 +
		 (-1 * x295 *
		  ((x273 *
			((x255 * x338) + (x260 * x335) +
			 (x255 * (x338 + (x255 * (x337 + (x263 * x335) + (x255 * ((x262 * x335) + (-1 * x294 * x335) + x336)))) +
					  (x264 * x335))) +
			 (x265 * x335))) +
		   (x306 * x334))) +
		 (x289 * x335) + (x301 * x334));
	out[0] = (-1 * (x123 + x124) * x126) + (-1 * x123) + x54;
	out[1] = (-1 * (x140 + x141) * x126) + (-1 * x140) + x127;
	out[2] = (-1 * (x152 + x153) * x126) + (-1 * x152) + x142;
	out[3] = (-1 * (x202 + x203) * x126) + (-1 * x202) + x192;
	out[4] = (-1 * x228) + (-1 * (x228 + x229) * x126) + x218;
	out[5] = (-1 * (x249 + x250) * x126) + (-1 * x249) + x239;
	out[6] = (-1 * x296) + (-1 * (x296 + x124) * x297) + x54;
	out[7] = (-1 * x307) + (-1 * (x307 + x141) * x297) + x127;
	out[8] = (-1 * x316) + (-1 * (x316 + x153) * x297) + x142;
	out[9] = (-1 * x324) + (-1 * (x324 + x203) * x297) + x192;
	out[10] = (-1 * x332) + (-1 * (x332 + x229) * x297) + x218;
	out[11] = (-1 * x339) + (-1 * (x339 + x250) * x297) + x239;
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qj;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qk;
	const GEN_FLT x10 = x9 * lh_qi;
	const GEN_FLT x11 = x10 + x6;
	const GEN_FLT x12 = pow(obj_qk, 2);
	const GEN_FLT x13 = pow(obj_qj, 2);
	const GEN_FLT x14 = pow(obj_qi, 2);
	const GEN_FLT x15 = 1e-10 + x14 + x13 + x12;
	const GEN_FLT x16 = pow(x15, 1.0 / 2.0);
	const GEN_FLT x17 = cos(x16);
	const GEN_FLT x18 = pow(x15, -1) * (1 + (-1 * x17));
	const GEN_FLT x19 = x17 + (x12 * x18);
	const GEN_FLT x20 = pow(x16, -1) * sin(x16);
	const GEN_FLT x21 = x20 * obj_qi;
	const GEN_FLT x22 = x18 * obj_qk * obj_qj;
	const GEN_FLT x23 = x22 + x21;
	const GEN_FLT x24 = x20 * obj_qj;
	const GEN_FLT x25 = x18 * obj_qi;
	const GEN_FLT x26 = x25 * obj_qk;
	const GEN_FLT x27 = x26 + (-1 * x24);
	const GEN_FLT x28 = (x23 * sensor_y) + (x19 * sensor_z) + (x27 * sensor_x) + obj_pz;
	const GEN_FLT x29 = x22 + (-1 * x21);
	const GEN_FLT x30 = x17 + (x13 * x18);
	const GEN_FLT x31 = x20 * obj_qk;
	const GEN_FLT x32 = x25 * obj_qj;
	const GEN_FLT x33 = x32 + x31;
	const GEN_FLT x34 = (x30 * sensor_y) + (x33 * sensor_x) + (x29 * sensor_z) + obj_py;
	const GEN_FLT x35 = x5 * lh_qk;
	const GEN_FLT x36 = x8 * lh_qj * lh_qi;
	const GEN_FLT x37 = x36 + (-1 * x35);
	const GEN_FLT x38 = x26 + x24;
	const GEN_FLT x39 = x32 + (-1 * x31);
	const GEN_FLT x40 = x17 + (x14 * x18);
	const GEN_FLT x41 = (x40 * sensor_x) + (x39 * sensor_y) + (x38 * sensor_z) + obj_px;
	const GEN_FLT x42 = x7 + (x2 * x8);
	const GEN_FLT x43 = (x41 * x42) + (x34 * x37) + (x28 * x11) + lh_px;
	const GEN_FLT x44 = pow(x43, -1);
	const GEN_FLT x45 = x10 + (-1 * x6);
	const GEN_FLT x46 = x5 * lh_qi;
	const GEN_FLT x47 = x9 * lh_qj;
	const GEN_FLT x48 = x47 + x46;
	const GEN_FLT x49 = x7 + (x0 * x8);
	const GEN_FLT x50 = (x49 * x27) + (x48 * x33) + (x40 * x45);
	const GEN_FLT x51 = (x27 * x11) + (x33 * x37) + (x40 * x42);
	const GEN_FLT x52 = (x41 * x45) + (x48 * x34) + (x49 * x28) + lh_pz;
	const GEN_FLT x53 = pow(x43, 2);
	const GEN_FLT x54 = pow(x53, -1) * x52;
	const GEN_FLT x55 = x53 + pow(x52, 2);
	const GEN_FLT x56 = pow(x55, -1);
	const GEN_FLT x57 = x53 * x56;
	const GEN_FLT x58 = ((x54 * x51) + (-1 * x50 * x44)) * x57;
	const GEN_FLT x59 = 0.523598775598299 + tilt_0;
	const GEN_FLT x60 = cos(x59);
	const GEN_FLT x61 = x47 + (-1 * x46);
	const GEN_FLT x62 = x7 + (x1 * x8);
	const GEN_FLT x63 = x36 + x35;
	const GEN_FLT x64 = (x63 * x41) + lh_py + (x62 * x34) + (x61 * x28);
	const GEN_FLT x65 = pow(x64, 2);
	const GEN_FLT x66 = x55 + x65;
	const GEN_FLT x67 = x65 * pow(x66, -1);
	const GEN_FLT x68 = pow((1 + (-1 * pow(x60, -2) * x67)), -1.0 / 2.0);
	const GEN_FLT x69 = (x61 * x27) + (x62 * x33) + (x63 * x40);
	const GEN_FLT x70 = 2 * x64;
	const GEN_FLT x71 = 2 * x43;
	const GEN_FLT x72 = 2 * x52;
	const GEN_FLT x73 = (x72 * x50) + (x71 * x51);
	const GEN_FLT x74 = x73 + (x70 * x69);
	const GEN_FLT x75 = pow(x60, -1);
	const GEN_FLT x76 = 1.0 / 2.0 * x64;
	const GEN_FLT x77 = x76 * pow(x66, -3.0 / 2.0);
	const GEN_FLT x78 = x75 * x77;
	const GEN_FLT x79 = pow(x66, -1.0 / 2.0);
	const GEN_FLT x80 = x79 * x75;
	const GEN_FLT x81 = ((x80 * x69) + (-1 * x78 * x74)) * x68;
	const GEN_FLT x82 = tan(x59);
	const GEN_FLT x83 = pow(x55, -1.0 / 2.0);
	const GEN_FLT x84 = x83 * x64;
	const GEN_FLT x85 = x82 * x84;
	const GEN_FLT x86 = atan2(-1 * x52, x43);
	const GEN_FLT x87 = x86 + (-1 * asin(x85)) + ogeeMag_0;
	const GEN_FLT x88 = (sin(x87) * ogeePhase_0) + curve_0;
	const GEN_FLT x89 = asin(x80 * x64);
	const GEN_FLT x90 = 8.0108022e-06 * x89;
	const GEN_FLT x91 = -8.0108022e-06 + (-1 * x90);
	const GEN_FLT x92 = 0.0028679863 + (x89 * x91);
	const GEN_FLT x93 = 5.3685255e-06 + (x89 * x92);
	const GEN_FLT x94 = 0.0076069798 + (x89 * x93);
	const GEN_FLT x95 = x89 * x94;
	const GEN_FLT x96 = -8.0108022e-06 + (-1.60216044e-05 * x89);
	const GEN_FLT x97 = x92 + (x89 * x96);
	const GEN_FLT x98 = x93 + (x89 * x97);
	const GEN_FLT x99 = x94 + (x89 * x98);
	const GEN_FLT x100 = (x89 * x99) + x95;
	const GEN_FLT x101 = sin(x59);
	const GEN_FLT x102 = x88 * x101;
	const GEN_FLT x103 = x60 + (-1 * x100 * x102);
	const GEN_FLT x104 = pow(x103, -1);
	const GEN_FLT x105 = 2 * x88 * x95 * x104;
	const GEN_FLT x106 = x81 * x91;
	const GEN_FLT x107 = 2.40324066e-05 * x89;
	const GEN_FLT x108 = (x89 * (x106 + (-1 * x81 * x90))) + (x81 * x92);
	const GEN_FLT x109 = (x89 * x108) + (x81 * x93);
	const GEN_FLT x110 = x65 * x56;
	const GEN_FLT x111 = pow((1 + (-1 * pow(x82, 2) * x110)), -1.0 / 2.0);
	const GEN_FLT x112 = x76 * pow(x55, -3.0 / 2.0);
	const GEN_FLT x113 = x73 * x112;
	const GEN_FLT x114 = x83 * x69;
	const GEN_FLT x115 = (x82 * x114) + (-1 * x82 * x113);
	const GEN_FLT x116 = (-1 * x111 * x115) + x58;
	const GEN_FLT x117 = cos(x87) * ogeePhase_0;
	const GEN_FLT x118 = x101 * x100;
	const GEN_FLT x119 = x118 * x117;
	const GEN_FLT x120 = pow(x89, 2);
	const GEN_FLT x121 = x88 * x120;
	const GEN_FLT x122 = x94 * pow(x103, -2) * x121;
	const GEN_FLT x123 = x94 * x104 * x120;
	const GEN_FLT x124 = x117 * x123;
	const GEN_FLT x125 = x104 * x121;
	const GEN_FLT x126 = x85 + (x94 * x125);
	const GEN_FLT x127 = pow((1 + (-1 * pow(x126, 2))), -1.0 / 2.0);
	const GEN_FLT x128 =
		x127 * (x115 + (x109 * x125) +
				(-1 * x122 *
				 ((-1 * x119 * x116) +
				  (-1 * x102 *
				   ((x89 * x109) + (x81 * x94) + (x81 * x99) +
					(x89 * ((x89 * (x108 + (x81 * x97) + (x89 * ((x81 * x96) + (-1 * x81 * x107) + x106)))) + x109 +
							(x81 * x98))))))) +
				(x116 * x124) + (x81 * x105));
	const GEN_FLT x129 = -1 * x58;
	const GEN_FLT x130 = -1 * x86;
	const GEN_FLT x131 = cos(x130 + asin(x126) + (-1 * gibPhase_0)) * gibMag_0;
	const GEN_FLT x132 = (x49 * x23) + (x48 * x30) + (x45 * x39);
	const GEN_FLT x133 = (x23 * x11) + (x30 * x37) + (x42 * x39);
	const GEN_FLT x134 = ((x54 * x133) + (-1 * x44 * x132)) * x57;
	const GEN_FLT x135 = (x61 * x23) + (x62 * x30) + (x63 * x39);
	const GEN_FLT x136 = (x72 * x132) + (x71 * x133);
	const GEN_FLT x137 = x136 + (x70 * x135);
	const GEN_FLT x138 = ((x80 * x135) + (-1 * x78 * x137)) * x68;
	const GEN_FLT x139 = x91 * x138;
	const GEN_FLT x140 = (x89 * (x139 + (-1 * x90 * x138))) + (x92 * x138);
	const GEN_FLT x141 = (x89 * x140) + (x93 * x138);
	const GEN_FLT x142 = x112 * x136;
	const GEN_FLT x143 = x83 * x135;
	const GEN_FLT x144 = (x82 * x143) + (-1 * x82 * x142);
	const GEN_FLT x145 = x117 * ((-1 * x111 * x144) + x134);
	const GEN_FLT x146 =
		x127 * (x144 + (x123 * x145) + (x125 * x141) +
				(-1 * x122 *
				 ((-1 * x118 * x145) +
				  (-1 * x102 *
				   ((x89 * x141) +
					(x89 * (x141 + (x89 * (x140 + (x97 * x138) + (x89 * ((x96 * x138) + (-1 * x107 * x138) + x139)))) +
							(x98 * x138))) +
					(x94 * x138) + (x99 * x138))))) +
				(x105 * x138));
	const GEN_FLT x147 = -1 * x134;
	const GEN_FLT x148 = (x49 * x19) + (x48 * x29) + (x45 * x38);
	const GEN_FLT x149 = (x11 * x19) + (x37 * x29) + (x42 * x38);
	const GEN_FLT x150 = ((x54 * x149) + (-1 * x44 * x148)) * x57;
	const GEN_FLT x151 = (x61 * x19) + (x62 * x29) + (x63 * x38);
	const GEN_FLT x152 = (x72 * x148) + (x71 * x149);
	const GEN_FLT x153 = x152 + (x70 * x151);
	const GEN_FLT x154 = x79 * x151;
	const GEN_FLT x155 = ((x75 * x154) + (-1 * x78 * x153)) * x68;
	const GEN_FLT x156 = x91 * x155;
	const GEN_FLT x157 = (x89 * (x156 + (-1 * x90 * x155))) + (x92 * x155);
	const GEN_FLT x158 = (x89 * x157) + (x93 * x155);
	const GEN_FLT x159 = x112 * x152;
	const GEN_FLT x160 = x83 * x151;
	const GEN_FLT x161 = (x82 * x160) + (-1 * x82 * x159);
	const GEN_FLT x162 = (-1 * x111 * x161) + x150;
	const GEN_FLT x163 =
		x127 * ((x125 * x158) + (x124 * x162) + x161 +
				(-1 * x122 *
				 ((-1 * x119 * x162) +
				  (-1 * x102 *
				   ((x89 * x158) + (x94 * x155) +
					(x89 * (x158 + (x89 * (x157 + (x97 * x155) + (x89 * ((x96 * x155) + (-1 * x107 * x155) + x156)))) +
							(x98 * x155))) +
					(x99 * x155))))) +
				(x105 * x155));
	const GEN_FLT x164 = -1 * x150;
	const GEN_FLT x165 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x166 = tan(x165);
	const GEN_FLT x167 = pow((1 + (-1 * x110 * pow(x166, 2))), -1.0 / 2.0);
	const GEN_FLT x168 = (-1 * x114 * x166) + (x113 * x166);
	const GEN_FLT x169 = -1 * x84 * x166;
	const GEN_FLT x170 = x86 + ogeeMag_1 + (-1 * asin(x169));
	const GEN_FLT x171 = cos(x170) * ogeePhase_1;
	const GEN_FLT x172 = x171 * ((-1 * x168 * x167) + x58);
	const GEN_FLT x173 = cos(x165);
	const GEN_FLT x174 = pow(x173, -1);
	const GEN_FLT x175 = x79 * x174;
	const GEN_FLT x176 = asin(x64 * x175);
	const GEN_FLT x177 = 8.0108022e-06 * x176;
	const GEN_FLT x178 = -8.0108022e-06 + (-1 * x177);
	const GEN_FLT x179 = 0.0028679863 + (x178 * x176);
	const GEN_FLT x180 = 5.3685255e-06 + (x179 * x176);
	const GEN_FLT x181 = 0.0076069798 + (x176 * x180);
	const GEN_FLT x182 = pow(x176, 2);
	const GEN_FLT x183 = x176 * x181;
	const GEN_FLT x184 = -8.0108022e-06 + (-1.60216044e-05 * x176);
	const GEN_FLT x185 = x179 + (x176 * x184);
	const GEN_FLT x186 = x180 + (x176 * x185);
	const GEN_FLT x187 = x181 + (x176 * x186);
	const GEN_FLT x188 = (x176 * x187) + x183;
	const GEN_FLT x189 = (sin(x170) * ogeePhase_1) + curve_1;
	const GEN_FLT x190 = sin(x165);
	const GEN_FLT x191 = x189 * x190;
	const GEN_FLT x192 = x173 + (x188 * x191);
	const GEN_FLT x193 = pow(x192, -1);
	const GEN_FLT x194 = x181 * x182 * x193;
	const GEN_FLT x195 = pow((1 + (-1 * x67 * pow(x173, -2))), -1.0 / 2.0);
	const GEN_FLT x196 = x77 * x174;
	const GEN_FLT x197 = (x69 * x175) + (-1 * x74 * x196);
	const GEN_FLT x198 = x197 * x195;
	const GEN_FLT x199 = 2 * x189 * x183 * x193;
	const GEN_FLT x200 = x178 * x198;
	const GEN_FLT x201 = (x179 * x198) + (x176 * (x200 + (-1 * x177 * x198)));
	const GEN_FLT x202 = (x180 * x198) + (x201 * x176);
	const GEN_FLT x203 = x189 * x182;
	const GEN_FLT x204 = x203 * x193;
	const GEN_FLT x205 = x188 * x190;
	const GEN_FLT x206 = x186 * x195;
	const GEN_FLT x207 = 2.40324066e-05 * x176;
	const GEN_FLT x208 = x203 * x181 * pow(x192, -2);
	const GEN_FLT x209 = x169 + (x204 * x181);
	const GEN_FLT x210 = pow((1 + (-1 * pow(x209, 2))), -1.0 / 2.0);
	const GEN_FLT x211 =
		x210 *
		(x168 +
		 (-1 * x208 *
		  ((x191 * ((x202 * x176) + (x181 * x198) +
					(x176 * ((x176 * (x201 + (x185 * x198) + (x176 * ((x184 * x198) + (-1 * x207 * x198) + x200)))) +
							 x202 + (x206 * x197))) +
					(x187 * x198))) +
		   (x205 * x172))) +
		 (x202 * x204) + (x198 * x199) + (x172 * x194));
	const GEN_FLT x212 = cos(x130 + asin(x209) + (-1 * gibPhase_1)) * gibMag_1;
	const GEN_FLT x213 = (-1 * x166 * x143) + (x166 * x142);
	const GEN_FLT x214 = (-1 * x213 * x167) + x134;
	const GEN_FLT x215 = x171 * x194;
	const GEN_FLT x216 = (x175 * x135) + (-1 * x196 * x137);
	const GEN_FLT x217 = x216 * x195;
	const GEN_FLT x218 = x217 * x178;
	const GEN_FLT x219 = (x217 * x179) + (x176 * (x218 + (-1 * x217 * x177)));
	const GEN_FLT x220 = (x217 * x180) + (x219 * x176);
	const GEN_FLT x221 = x205 * x171;
	const GEN_FLT x222 =
		x210 *
		(x213 +
		 (-1 * x208 *
		  ((x191 *
			((x217 * x181) + (x220 * x176) +
			 (x176 * (x220 + (x176 * ((x217 * x185) + x219 + (x176 * ((x217 * x184) + (-1 * x217 * x207) + x218)))) +
					  (x216 * x206))) +
			 (x217 * x187))) +
		   (x214 * x221))) +
		 (x204 * x220) + (x217 * x199) + (x215 * x214));
	const GEN_FLT x223 = (-1 * x160 * x166) + (x166 * x159);
	const GEN_FLT x224 = (-1 * x223 * x167) + x150;
	const GEN_FLT x225 = (x174 * x154) + (-1 * x196 * x153);
	const GEN_FLT x226 = x225 * x195;
	const GEN_FLT x227 = x226 * x178;
	const GEN_FLT x228 = (x226 * x179) + (x176 * (x227 + (-1 * x226 * x177)));
	const GEN_FLT x229 = (x226 * x180) + (x228 * x176);
	const GEN_FLT x230 =
		x210 *
		(x223 +
		 (-1 * x208 *
		  ((x191 *
			((x229 * x176) + (x226 * x181) +
			 (x176 * (x229 + (x176 * (x228 + (x226 * x185) + (x176 * ((x226 * x184) + (-1 * x207 * x226) + x227)))) +
					  (x206 * x225))) +
			 (x226 * x187))) +
		   (x224 * x221))) +
		 (x204 * x229) + (x226 * x199) + (x215 * x224));
	out[0] = (-1 * (x128 + x129) * x131) + (-1 * x128) + x58;
	out[1] = (-1 * (x146 + x147) * x131) + (-1 * x146) + x134;
	out[2] = (-1 * (x163 + x164) * x131) + (-1 * x163) + x150;
	out[3] = (-1 * x211) + x58 + (-1 * (x211 + x129) * x212);
	out[4] = (-1 * x222) + (-1 * (x222 + x147) * x212) + x134;
	out[5] = (-1 * x230) + (-1 * (x230 + x164) * x212) + x150;
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
	const GEN_FLT x0 = pow(obj_qk, 2);
	const GEN_FLT x1 = pow(obj_qj, 2);
	const GEN_FLT x2 = pow(obj_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = pow(x3, -1) * (1 + (-1 * x5));
	const GEN_FLT x7 = pow(x4, -1) * sin(x4);
	const GEN_FLT x8 = x7 * obj_qi;
	const GEN_FLT x9 = x6 * obj_qk;
	const GEN_FLT x10 = x9 * obj_qj;
	const GEN_FLT x11 = x7 * obj_qj;
	const GEN_FLT x12 = x9 * obj_qi;
	const GEN_FLT x13 =
		((x12 + (-1 * x11)) * sensor_x) + ((x10 + x8) * sensor_y) + ((x5 + (x0 * x6)) * sensor_z) + obj_pz;
	const GEN_FLT x14 = pow(lh_qk, 2);
	const GEN_FLT x15 = pow(lh_qj, 2);
	const GEN_FLT x16 = pow(lh_qi, 2);
	const GEN_FLT x17 = 1e-10 + x16 + x15 + x14;
	const GEN_FLT x18 = pow(x17, 1.0 / 2.0);
	const GEN_FLT x19 = cos(x18);
	const GEN_FLT x20 = 1 + (-1 * x19);
	const GEN_FLT x21 = pow(x17, -1);
	const GEN_FLT x22 = x20 * x21;
	const GEN_FLT x23 = x7 * obj_qk;
	const GEN_FLT x24 = x6 * obj_qj * obj_qi;
	const GEN_FLT x25 =
		((x24 + x23) * sensor_x) + ((x5 + (x1 * x6)) * sensor_y) + ((x10 + (-1 * x8)) * sensor_z) + obj_py;
	const GEN_FLT x26 = sin(x18);
	const GEN_FLT x27 = x26 * pow(x18, -1);
	const GEN_FLT x28 = x27 * lh_qi;
	const GEN_FLT x29 = x22 * lh_qj;
	const GEN_FLT x30 = x29 * lh_qk;
	const GEN_FLT x31 =
		((x24 + (-1 * x23)) * sensor_y) + ((x5 + (x2 * x6)) * sensor_x) + ((x12 + x11) * sensor_z) + obj_px;
	const GEN_FLT x32 = x27 * lh_qj;
	const GEN_FLT x33 = -1 * x32;
	const GEN_FLT x34 = x22 * lh_qi;
	const GEN_FLT x35 = x34 * lh_qk;
	const GEN_FLT x36 = ((x35 + x33) * x31) + (x13 * (x19 + (x22 * x14))) + ((x30 + x28) * x25) + lh_pz;
	const GEN_FLT x37 = x27 * lh_qk;
	const GEN_FLT x38 = -1 * x37;
	const GEN_FLT x39 = x29 * lh_qi;
	const GEN_FLT x40 = (x31 * (x19 + (x22 * x16))) + ((x39 + x38) * x25) + ((x35 + x32) * x13) + lh_px;
	const GEN_FLT x41 = pow(x40, 2);
	const GEN_FLT x42 = x41 + pow(x36, 2);
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = x43 * x36;
	const GEN_FLT x45 = 0.523598775598299 + tilt_0;
	const GEN_FLT x46 = tan(x45);
	const GEN_FLT x47 = -1 * x28;
	const GEN_FLT x48 = ((x39 + x37) * x31) + lh_py + (x25 * (x19 + (x22 * x15))) + ((x30 + x47) * x13);
	const GEN_FLT x49 = pow(x42, -3.0 / 2.0) * x48;
	const GEN_FLT x50 = x40 * x49;
	const GEN_FLT x51 = x50 * x46;
	const GEN_FLT x52 = 2 * x40;
	const GEN_FLT x53 = pow(x48, 2);
	const GEN_FLT x54 = x42 + x53;
	const GEN_FLT x55 = pow(x54, -3.0 / 2.0);
	const GEN_FLT x56 = x55 * x48;
	const GEN_FLT x57 = x52 * x56;
	const GEN_FLT x58 = cos(x45);
	const GEN_FLT x59 = pow(x58, -1);
	const GEN_FLT x60 = pow(x54, -1) * x53;
	const GEN_FLT x61 = pow((1 + (-1 * x60 * pow(x58, -2))), -1.0 / 2.0);
	const GEN_FLT x62 = x61 * x59;
	const GEN_FLT x63 = pow(x42, -1.0 / 2.0);
	const GEN_FLT x64 = x63 * x46;
	const GEN_FLT x65 = x64 * x48;
	const GEN_FLT x66 = atan2(-1 * x36, x40);
	const GEN_FLT x67 = x66 + (-1 * asin(x65)) + ogeeMag_0;
	const GEN_FLT x68 = (sin(x67) * ogeePhase_0) + curve_0;
	const GEN_FLT x69 = pow(x54, -1.0 / 2.0);
	const GEN_FLT x70 = x69 * x59;
	const GEN_FLT x71 = asin(x70 * x48);
	const GEN_FLT x72 = 8.0108022e-06 * x71;
	const GEN_FLT x73 = -8.0108022e-06 + (-1 * x72);
	const GEN_FLT x74 = 0.0028679863 + (x71 * x73);
	const GEN_FLT x75 = 5.3685255e-06 + (x71 * x74);
	const GEN_FLT x76 = 0.0076069798 + (x71 * x75);
	const GEN_FLT x77 = x71 * x76;
	const GEN_FLT x78 = -8.0108022e-06 + (-1.60216044e-05 * x71);
	const GEN_FLT x79 = x74 + (x71 * x78);
	const GEN_FLT x80 = x75 + (x71 * x79);
	const GEN_FLT x81 = x76 + (x80 * x71);
	const GEN_FLT x82 = (x81 * x71) + x77;
	const GEN_FLT x83 = sin(x45);
	const GEN_FLT x84 = x83 * x68;
	const GEN_FLT x85 = x58 + (-1 * x82 * x84);
	const GEN_FLT x86 = pow(x85, -1);
	const GEN_FLT x87 = x86 * x77 * x68;
	const GEN_FLT x88 = x87 * x62;
	const GEN_FLT x89 = x56 * x40;
	const GEN_FLT x90 = x89 * x62;
	const GEN_FLT x91 = -1 * x73 * x90;
	const GEN_FLT x92 = 2.40324066e-05 * x71;
	const GEN_FLT x93 = x74 * x62;
	const GEN_FLT x94 = (x71 * (x91 + (x72 * x90))) + (-1 * x89 * x93);
	const GEN_FLT x95 = (x71 * x94) + (-1 * x75 * x90);
	const GEN_FLT x96 = x53 * x43;
	const GEN_FLT x97 = pow((1 + (-1 * x96 * pow(x46, 2))), -1.0 / 2.0);
	const GEN_FLT x98 = (x51 * x97) + x44;
	const GEN_FLT x99 = cos(x67) * ogeePhase_0;
	const GEN_FLT x100 = x82 * x83;
	const GEN_FLT x101 = x99 * x100;
	const GEN_FLT x102 = pow(x71, 2);
	const GEN_FLT x103 = x68 * x102;
	const GEN_FLT x104 = pow(x85, -2) * x76 * x103;
	const GEN_FLT x105 = x86 * x76 * x102;
	const GEN_FLT x106 = x99 * x105;
	const GEN_FLT x107 = x86 * x103;
	const GEN_FLT x108 = x65 + (x76 * x107);
	const GEN_FLT x109 = pow((1 + (-1 * pow(x108, 2))), -1.0 / 2.0);
	const GEN_FLT x110 =
		x109 * ((x95 * x107) + (x98 * x106) +
				(-1 * x104 *
				 ((-1 * x98 * x101) +
				  (-1 * x84 *
				   ((x71 * x95) + (-1 * x76 * x90) + (-1 * x81 * x90) +
					(x71 * (x95 + (x71 * (x94 + (-1 * x79 * x90) + (x71 * ((-1 * x78 * x90) + (x92 * x90) + x91)))) +
							(-1 * x80 * x90))))))) +
				(-1 * x88 * x57) + (-1 * x51));
	const GEN_FLT x111 = -1 * x44;
	const GEN_FLT x112 = -1 * x66;
	const GEN_FLT x113 = cos(x112 + asin(x108) + (-1 * gibPhase_0)) * gibMag_0;
	const GEN_FLT x114 = x53 * x55;
	const GEN_FLT x115 = x61 * (x70 + (-1 * x59 * x114));
	const GEN_FLT x116 = 2 * x87;
	const GEN_FLT x117 = x73 * x115;
	const GEN_FLT x118 = (x71 * (x117 + (-1 * x72 * x115))) + (x74 * x115);
	const GEN_FLT x119 = (x71 * x118) + (x75 * x115);
	const GEN_FLT x120 = x64 * x97;
	const GEN_FLT x121 =
		x109 * ((-1 * x106 * x120) +
				(-1 * x104 *
				 ((x101 * x120) +
				  (-1 * x84 *
				   ((x71 * x119) + (x81 * x115) + (x76 * x115) +
					(x71 * (x119 + (x71 * ((x79 * x115) + x118 + (x71 * ((x78 * x115) + (-1 * x92 * x115) + x117)))) +
							(x80 * x115))))))) +
				(x107 * x119) + (x115 * x116) + x64);
	const GEN_FLT x122 = x40 * x43;
	const GEN_FLT x123 = -1 * x122;
	const GEN_FLT x124 = x49 * x36;
	const GEN_FLT x125 = x46 * x124;
	const GEN_FLT x126 = 2 * x48;
	const GEN_FLT x127 = x55 * x36 * x126;
	const GEN_FLT x128 = x56 * x36;
	const GEN_FLT x129 = x62 * x128;
	const GEN_FLT x130 = -1 * x73 * x129;
	const GEN_FLT x131 = (x71 * (x130 + (x72 * x129))) + (-1 * x93 * x128);
	const GEN_FLT x132 = (x71 * x131) + (-1 * x75 * x129);
	const GEN_FLT x133 = (x97 * x125) + x123;
	const GEN_FLT x134 =
		x109 *
		((x106 * x133) +
		 (-1 * x104 *
		  ((-1 * x101 * x133) +
		   (-1 * x84 *
			((-1 * x76 * x129) + (x71 * x132) +
			 (x71 * (x132 + (x71 * (x131 + (-1 * x79 * x129) + (x71 * ((-1 * x78 * x129) + (x92 * x129) + x130)))) +
					 (-1 * x80 * x129))) +
			 (-1 * x81 * x129))))) +
		 (x107 * x132) + (-1 * x88 * x127) + (-1 * x125));
	const GEN_FLT x135 = x22 * lh_qk;
	const GEN_FLT x136 = x26 * pow(x17, -3.0 / 2.0);
	const GEN_FLT x137 = x16 * x136;
	const GEN_FLT x138 = 2 * x20 * pow(x17, -2);
	const GEN_FLT x139 = x138 * lh_qk;
	const GEN_FLT x140 = (-1 * x16 * x139) + (x137 * lh_qk);
	const GEN_FLT x141 = x140 + x135;
	const GEN_FLT x142 = x136 * lh_qj;
	const GEN_FLT x143 = x142 * lh_qi;
	const GEN_FLT x144 = x21 * x19;
	const GEN_FLT x145 = x144 * lh_qj;
	const GEN_FLT x146 = x145 * lh_qi;
	const GEN_FLT x147 = (-1 * x146) + x143;
	const GEN_FLT x148 = x16 * x144;
	const GEN_FLT x149 = lh_qk * lh_qi;
	const GEN_FLT x150 = x136 * x149;
	const GEN_FLT x151 = x138 * lh_qj;
	const GEN_FLT x152 = (-1 * x149 * x151) + (x150 * lh_qj);
	const GEN_FLT x153 = x152 + x27;
	const GEN_FLT x154 = x14 * x136;
	const GEN_FLT x155 = x138 * lh_qi;
	const GEN_FLT x156 = (-1 * x14 * x155) + (x154 * lh_qi);
	const GEN_FLT x157 = (x13 * (x156 + x47)) + (x25 * (x153 + x148 + (-1 * x137))) + ((x147 + x141) * x31);
	const GEN_FLT x158 = pow(x40, -1);
	const GEN_FLT x159 = pow(lh_qi, 3);
	const GEN_FLT x160 = (-1 * x16 * x151) + (x137 * lh_qj);
	const GEN_FLT x161 = x160 + x29;
	const GEN_FLT x162 = x144 * x149;
	const GEN_FLT x163 = (-1 * x162) + x150;
	const GEN_FLT x164 = x146 + (-1 * x143);
	const GEN_FLT x165 =
		((x164 + x141) * x13) + ((x163 + x161) * x25) + (((-1 * x138 * x159) + (x136 * x159) + (2 * x34) + x47) * x31);
	const GEN_FLT x166 = pow(x41, -1) * x36;
	const GEN_FLT x167 = x41 * x43;
	const GEN_FLT x168 = ((x166 * x165) + (-1 * x157 * x158)) * x167;
	const GEN_FLT x169 = x162 + (-1 * x150);
	const GEN_FLT x170 = x15 * x136;
	const GEN_FLT x171 = (-1 * x15 * x155) + (x170 * lh_qi);
	const GEN_FLT x172 = x152 + (-1 * x27);
	const GEN_FLT x173 = (x13 * (x172 + (-1 * x148) + x137)) + (x25 * (x171 + x47)) + ((x169 + x161) * x31);
	const GEN_FLT x174 = 2 * x36;
	const GEN_FLT x175 = (x174 * x157) + (x52 * x165);
	const GEN_FLT x176 = 1.0 / 2.0 * x56;
	const GEN_FLT x177 = x176 * (x175 + (x126 * x173));
	const GEN_FLT x178 = ((x70 * x173) + (-1 * x59 * x177)) * x61;
	const GEN_FLT x179 = 1.0 / 2.0 * x49;
	const GEN_FLT x180 = x179 * x175;
	const GEN_FLT x181 = (x64 * x173) + (-1 * x46 * x180);
	const GEN_FLT x182 = x99 * ((-1 * x97 * x181) + x168);
	const GEN_FLT x183 = x73 * x178;
	const GEN_FLT x184 = (x71 * (x183 + (-1 * x72 * x178))) + (x74 * x178);
	const GEN_FLT x185 = (x71 * x184) + (x75 * x178);
	const GEN_FLT x186 =
		x109 * (x181 + (x107 * x185) +
				(-1 * x104 *
				 ((-1 * x100 * x182) +
				  (-1 * x84 *
				   ((x71 * x185) + (x76 * x178) +
					(x71 * (x185 + (x71 * ((x79 * x178) + x184 + (x71 * ((x78 * x178) + (-1 * x92 * x178) + x183)))) +
							(x80 * x178))) +
					(x81 * x178))))) +
				(x105 * x182) + (x116 * x178));
	const GEN_FLT x187 = -1 * x168;
	const GEN_FLT x188 = x15 * x144;
	const GEN_FLT x189 = (-1 * x15 * x139) + (x170 * lh_qk);
	const GEN_FLT x190 = x189 + x135;
	const GEN_FLT x191 = (-1 * x14 * x151) + (x154 * lh_qj);
	const GEN_FLT x192 = (x13 * (x191 + x33)) + ((x190 + x164) * x25) + (x31 * (x172 + (-1 * x188) + x170));
	const GEN_FLT x193 = x171 + x34;
	const GEN_FLT x194 = x142 * lh_qk;
	const GEN_FLT x195 = x145 * lh_qk;
	const GEN_FLT x196 = (-1 * x195) + x194;
	const GEN_FLT x197 = (x13 * (x153 + x188 + (-1 * x170))) + ((x196 + x193) * x25) + (x31 * (x160 + x33));
	const GEN_FLT x198 = ((x166 * x197) + (-1 * x192 * x158)) * x167;
	const GEN_FLT x199 = pow(lh_qj, 3);
	const GEN_FLT x200 = x195 + (-1 * x194);
	const GEN_FLT x201 =
		((x190 + x147) * x13) + ((x200 + x193) * x31) + (((-1 * x199 * x138) + (x199 * x136) + (2 * x29) + x33) * x25);
	const GEN_FLT x202 = (x174 * x192) + (x52 * x197);
	const GEN_FLT x203 = x176 * (x202 + (x201 * x126));
	const GEN_FLT x204 = ((x70 * x201) + (-1 * x59 * x203)) * x61;
	const GEN_FLT x205 = x202 * x179;
	const GEN_FLT x206 = (x64 * x201) + (-1 * x46 * x205);
	const GEN_FLT x207 = (-1 * x97 * x206) + x198;
	const GEN_FLT x208 = x73 * x204;
	const GEN_FLT x209 = (x71 * (x208 + (-1 * x72 * x204))) + (x74 * x204);
	const GEN_FLT x210 = (x71 * x209) + (x75 * x204);
	const GEN_FLT x211 =
		x109 * (x206 + (x210 * x107) +
				(-1 * x104 *
				 ((-1 * x207 * x101) +
				  (-1 * x84 *
				   ((x71 * (x210 + (x71 * (x209 + (x79 * x204) + (x71 * ((-1 * x92 * x204) + (x78 * x204) + x208)))) +
							(x80 * x204))) +
					(x71 * x210) + (x76 * x204) + (x81 * x204))))) +
				(x207 * x106) + (x204 * x116));
	const GEN_FLT x212 = -1 * x198;
	const GEN_FLT x213 = x14 * x144;
	const GEN_FLT x214 = x191 + x29;
	const GEN_FLT x215 = ((x214 + x163) * x13) + (x25 * (x189 + x38)) + (x31 * (x153 + x213 + (-1 * x154)));
	const GEN_FLT x216 = x156 + x34;
	const GEN_FLT x217 = ((x216 + x200) * x13) + (x25 * (x172 + (-1 * x213) + x154)) + (x31 * (x140 + x38));
	const GEN_FLT x218 = pow(lh_qk, 3);
	const GEN_FLT x219 =
		(x13 * ((-1 * x218 * x138) + (2 * x135) + (x218 * x136) + x38)) + ((x214 + x169) * x25) + ((x216 + x196) * x31);
	const GEN_FLT x220 = (x219 * x174) + (x52 * x217);
	const GEN_FLT x221 = x176 * (x220 + (x215 * x126));
	const GEN_FLT x222 = ((x70 * x215) + (-1 * x59 * x221)) * x61;
	const GEN_FLT x223 = x73 * x222;
	const GEN_FLT x224 = (x71 * (x223 + (-1 * x72 * x222))) + (x74 * x222);
	const GEN_FLT x225 = (x71 * x224) + (x75 * x222);
	const GEN_FLT x226 = ((x217 * x166) + (-1 * x219 * x158)) * x167;
	const GEN_FLT x227 = x220 * x179;
	const GEN_FLT x228 = (x64 * x215) + (-1 * x46 * x227);
	const GEN_FLT x229 = (-1 * x97 * x228) + x226;
	const GEN_FLT x230 =
		x109 * (x228 + (x229 * x106) + (x225 * x107) +
				(-1 * x104 *
				 ((-1 * x229 * x101) +
				  (-1 * x84 *
				   ((x71 * x225) + (x76 * x222) +
					(x71 * (x225 + (x71 * (x224 + (x79 * x222) + (x71 * ((x78 * x222) + (-1 * x92 * x222) + x223)))) +
							(x80 * x222))) +
					(x81 * x222))))) +
				(x222 * x116));
	const GEN_FLT x231 = -1 * x226;
	const GEN_FLT x232 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x233 = cos(x232);
	const GEN_FLT x234 = pow(x233, -1);
	const GEN_FLT x235 = x69 * x234;
	const GEN_FLT x236 = asin(x48 * x235);
	const GEN_FLT x237 = 8.0108022e-06 * x236;
	const GEN_FLT x238 = -8.0108022e-06 + (-1 * x237);
	const GEN_FLT x239 = 0.0028679863 + (x238 * x236);
	const GEN_FLT x240 = 5.3685255e-06 + (x236 * x239);
	const GEN_FLT x241 = 0.0076069798 + (x236 * x240);
	const GEN_FLT x242 = x236 * x241;
	const GEN_FLT x243 = -8.0108022e-06 + (-1.60216044e-05 * x236);
	const GEN_FLT x244 = x239 + (x236 * x243);
	const GEN_FLT x245 = x240 + (x236 * x244);
	const GEN_FLT x246 = x241 + (x236 * x245);
	const GEN_FLT x247 = (x236 * x246) + x242;
	const GEN_FLT x248 = tan(x232);
	const GEN_FLT x249 = x63 * x248;
	const GEN_FLT x250 = -1 * x48 * x249;
	const GEN_FLT x251 = x66 + ogeeMag_1 + (-1 * asin(x250));
	const GEN_FLT x252 = (sin(x251) * ogeePhase_1) + curve_1;
	const GEN_FLT x253 = sin(x232);
	const GEN_FLT x254 = x252 * x253;
	const GEN_FLT x255 = x233 + (x254 * x247);
	const GEN_FLT x256 = pow(x255, -1);
	const GEN_FLT x257 = pow(x236, 2);
	const GEN_FLT x258 = x252 * x257;
	const GEN_FLT x259 = x256 * x258;
	const GEN_FLT x260 = x250 + (x259 * x241);
	const GEN_FLT x261 = pow((1 + (-1 * pow(x260, 2))), -1.0 / 2.0);
	const GEN_FLT x262 = x50 * x248;
	const GEN_FLT x263 = pow((1 + (-1 * x96 * pow(x248, 2))), -1.0 / 2.0);
	const GEN_FLT x264 = (-1 * x262 * x263) + x44;
	const GEN_FLT x265 = cos(x251) * ogeePhase_1;
	const GEN_FLT x266 = x256 * x257 * x241;
	const GEN_FLT x267 = x266 * x265;
	const GEN_FLT x268 = pow((1 + (-1 * x60 * pow(x233, -2))), -1.0 / 2.0);
	const GEN_FLT x269 = x234 * x268;
	const GEN_FLT x270 = x252 * x256 * x242;
	const GEN_FLT x271 = x270 * x269;
	const GEN_FLT x272 = x89 * x269;
	const GEN_FLT x273 = -1 * x238 * x272;
	const GEN_FLT x274 = (-1 * x239 * x272) + (x236 * (x273 + (x237 * x272)));
	const GEN_FLT x275 = (-1 * x272 * x240) + (x236 * x274);
	const GEN_FLT x276 = x253 * x247;
	const GEN_FLT x277 = x276 * x265;
	const GEN_FLT x278 = 2.40324066e-05 * x236;
	const GEN_FLT x279 = x268 * x244;
	const GEN_FLT x280 = x89 * x234;
	const GEN_FLT x281 = x268 * x246;
	const GEN_FLT x282 = pow(x255, -2) * x258 * x241;
	const GEN_FLT x283 =
		x261 *
		((-1 * x282 *
		  ((x254 *
			((x236 * x275) + (-1 * x280 * x281) + (-1 * x272 * x241) +
			 (x236 *
			  (x275 + (x236 * (x274 + (-1 * x279 * x280) + (x236 * ((-1 * x272 * x243) + (x278 * x272) + x273)))) +
			   (-1 * x272 * x245))))) +
		   (x277 * x264))) +
		 (x275 * x259) + (-1 * x57 * x271) + (x267 * x264) + x262);
	const GEN_FLT x284 = cos(asin(x260) + x112 + (-1 * gibPhase_1)) * gibMag_1;
	const GEN_FLT x285 = x263 * x249;
	const GEN_FLT x286 = x235 + (-1 * x234 * x114);
	const GEN_FLT x287 = x268 * x286;
	const GEN_FLT x288 = x238 * x287;
	const GEN_FLT x289 = (x239 * x287) + (x236 * (x288 + (-1 * x237 * x287)));
	const GEN_FLT x290 = (x287 * x240) + (x236 * x289);
	const GEN_FLT x291 = 2 * x270;
	const GEN_FLT x292 =
		x261 *
		((x291 * x287) +
		 (-1 * x282 *
		  ((x254 *
			((x236 * x290) + (x287 * x241) +
			 (x236 * (x290 + (x236 * (x289 + (x279 * x286) + (x236 * ((x287 * x243) + (-1 * x278 * x287) + x288)))) +
					  (x287 * x245))) +
			 (x286 * x281))) +
		   (x277 * x285))) +
		 (x290 * x259) + (x267 * x285) + (-1 * x249));
	const GEN_FLT x293 = x248 * x124;
	const GEN_FLT x294 = (-1 * x293 * x263) + x123;
	const GEN_FLT x295 = x269 * x128;
	const GEN_FLT x296 = -1 * x238 * x295;
	const GEN_FLT x297 = (-1 * x239 * x295) + (x236 * (x296 + (x237 * x295)));
	const GEN_FLT x298 = (-1 * x295 * x240) + (x236 * x297);
	const GEN_FLT x299 = x234 * x128;
	const GEN_FLT x300 =
		x261 *
		((-1 * x282 *
		  ((x254 *
			((x236 * x298) + (-1 * x295 * x241) +
			 (x236 *
			  (x298 + (x236 * (x297 + (-1 * x279 * x299) + (x236 * ((-1 * x295 * x243) + (x278 * x295) + x296)))) +
			   (-1 * x295 * x245))) +
			 (-1 * x299 * x281))) +
		   (x277 * x294))) +
		 x293 + (x298 * x259) + (-1 * x271 * x127) + (x294 * x267));
	const GEN_FLT x301 = (-1 * x249 * x173) + (x248 * x180);
	const GEN_FLT x302 = (-1 * x263 * x301) + x168;
	const GEN_FLT x303 = (x235 * x173) + (-1 * x234 * x177);
	const GEN_FLT x304 = x268 * x303;
	const GEN_FLT x305 = x238 * x304;
	const GEN_FLT x306 = (x239 * x304) + (x236 * (x305 + (-1 * x237 * x304)));
	const GEN_FLT x307 = (x240 * x304) + (x236 * x306);
	const GEN_FLT x308 =
		x261 *
		(x301 + (x259 * x307) +
		 (-1 * x282 *
		  ((x254 *
			((x236 * x307) + (x241 * x304) +
			 (x236 * (x307 + (x236 * (x306 + (x279 * x303) + (x236 * ((x243 * x304) + (-1 * x278 * x304) + x305)))) +
					  (x245 * x304))) +
			 (x281 * x303))) +
		   (x277 * x302))) +
		 (x291 * x304) + (x267 * x302));
	const GEN_FLT x309 = (-1 * x201 * x249) + (x205 * x248);
	const GEN_FLT x310 = (-1 * x263 * x309) + x198;
	const GEN_FLT x311 = (x235 * x201) + (-1 * x234 * x203);
	const GEN_FLT x312 = x268 * x311;
	const GEN_FLT x313 = x238 * x312;
	const GEN_FLT x314 = (x239 * x312) + (x236 * (x313 + (-1 * x237 * x312)));
	const GEN_FLT x315 = (x240 * x312) + (x236 * x314);
	const GEN_FLT x316 =
		x261 *
		(x309 +
		 (-1 * x282 *
		  ((x254 *
			((x236 * x315) + (x241 * x312) +
			 (x236 * (x315 + (x236 * (x314 + (x279 * x311) + (x236 * ((x243 * x312) + (-1 * x278 * x312) + x313)))) +
					  (x245 * x312))) +
			 (x281 * x311))) +
		   (x277 * x310))) +
		 (x259 * x315) + (x291 * x312) + (x267 * x310));
	const GEN_FLT x317 = (-1 * x215 * x249) + (x227 * x248);
	const GEN_FLT x318 = x265 * ((-1 * x263 * x317) + x226);
	const GEN_FLT x319 = (x215 * x235) + (-1 * x234 * x221);
	const GEN_FLT x320 = x268 * x319;
	const GEN_FLT x321 = x238 * x320;
	const GEN_FLT x322 = (x239 * x320) + (x236 * (x321 + (-1 * x237 * x320)));
	const GEN_FLT x323 = (x240 * x320) + (x236 * x322);
	const GEN_FLT x324 =
		x261 *
		(x317 +
		 (-1 * x282 *
		  ((x254 *
			((x236 * x323) + (x241 * x320) +
			 (x236 * (x323 + (x236 * (x322 + (x279 * x319) + (x236 * ((x243 * x320) + x321 + (-1 * x278 * x320))))) +
					  (x245 * x320))) +
			 (x281 * x319))) +
		   (x276 * x318))) +
		 (x259 * x323) + (x291 * x320) + (x266 * x318));
	out[0] = (-1 * x110) + (-1 * (x110 + x111) * x113) + x44;
	out[1] = (-1 * x113 * x121) + (-1 * x121);
	out[2] = (-1 * x134) + (-1 * (x134 + x122) * x113) + x123;
	out[3] = (-1 * (x186 + x187) * x113) + (-1 * x186) + x168;
	out[4] = (-1 * (x211 + x212) * x113) + (-1 * x211) + x198;
	out[5] = (-1 * (x230 + x231) * x113) + x226 + (-1 * x230);
	out[6] = x44 + (-1 * x283) + (-1 * (x283 + x111) * x284);
	out[7] = (-1 * x292) + (-1 * x292 * x284);
	out[8] = (-1 * x300) + (-1 * (x300 + x122) * x284) + x123;
	out[9] = (-1 * x308) + x168 + (-1 * (x308 + x187) * x284);
	out[10] = (-1 * x316) + (-1 * (x316 + x212) * x284) + x198;
	out[11] = (-1 * x324) + (-1 * (x324 + x231) * x284) + x226;
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
	const GEN_FLT x0 = 0.523598775598299 + tilt_0;
	const GEN_FLT x1 = tan(x0);
	const GEN_FLT x2 = pow(lh_qk, 2);
	const GEN_FLT x3 = pow(lh_qj, 2);
	const GEN_FLT x4 = pow(lh_qi, 2);
	const GEN_FLT x5 = 1e-10 + x4 + x3 + x2;
	const GEN_FLT x6 = pow(x5, 1.0 / 2.0);
	const GEN_FLT x7 = pow(x6, -1) * sin(x6);
	const GEN_FLT x8 = x7 * lh_qi;
	const GEN_FLT x9 = cos(x6);
	const GEN_FLT x10 = pow(x5, -1) * (1 + (-1 * x9));
	const GEN_FLT x11 = x10 * lh_qj;
	const GEN_FLT x12 = x11 * lh_qk;
	const GEN_FLT x13 = pow(obj_qk, 2);
	const GEN_FLT x14 = pow(obj_qj, 2);
	const GEN_FLT x15 = pow(obj_qi, 2);
	const GEN_FLT x16 = 1e-10 + x15 + x14 + x13;
	const GEN_FLT x17 = pow(x16, 1.0 / 2.0);
	const GEN_FLT x18 = cos(x17);
	const GEN_FLT x19 = pow(x16, -1) * (1 + (-1 * x18));
	const GEN_FLT x20 = pow(x17, -1) * sin(x17);
	const GEN_FLT x21 = x20 * obj_qi;
	const GEN_FLT x22 = x19 * obj_qk * obj_qj;
	const GEN_FLT x23 = x20 * obj_qj;
	const GEN_FLT x24 = x19 * obj_qi;
	const GEN_FLT x25 = x24 * obj_qk;
	const GEN_FLT x26 =
		((x25 + (-1 * x23)) * sensor_x) + ((x22 + x21) * sensor_y) + ((x18 + (x13 * x19)) * sensor_z) + obj_pz;
	const GEN_FLT x27 = x20 * obj_qk;
	const GEN_FLT x28 = x24 * obj_qj;
	const GEN_FLT x29 =
		((x28 + x27) * sensor_x) + ((x18 + (x14 * x19)) * sensor_y) + ((x22 + (-1 * x21)) * sensor_z) + obj_py;
	const GEN_FLT x30 =
		((x18 + (x15 * x19)) * sensor_x) + ((x28 + (-1 * x27)) * sensor_y) + ((x25 + x23) * sensor_z) + obj_px;
	const GEN_FLT x31 = x7 * lh_qk;
	const GEN_FLT x32 = x11 * lh_qi;
	const GEN_FLT x33 = ((x32 + x31) * x30) + (x29 * (x9 + (x3 * x10))) + lh_py + (x26 * (x12 + (-1 * x8)));
	const GEN_FLT x34 = x7 * lh_qj;
	const GEN_FLT x35 = x10 * lh_qk * lh_qi;
	const GEN_FLT x36 = ((x35 + (-1 * x34)) * x30) + (x29 * (x12 + x8)) + (x26 * (x9 + (x2 * x10))) + lh_pz;
	const GEN_FLT x37 = (x30 * (x9 + (x4 * x10))) + ((x32 + (-1 * x31)) * x29) + ((x35 + x34) * x26) + lh_px;
	const GEN_FLT x38 = pow(x37, 2) + pow(x36, 2);
	const GEN_FLT x39 = x33 * pow(x38, -1.0 / 2.0);
	const GEN_FLT x40 = x1 * x39;
	const GEN_FLT x41 = atan2(-1 * x36, x37);
	const GEN_FLT x42 = x41 + (-1 * asin(x40)) + ogeeMag_0;
	const GEN_FLT x43 = sin(x42);
	const GEN_FLT x44 = (x43 * ogeePhase_0) + curve_0;
	const GEN_FLT x45 = cos(x0);
	const GEN_FLT x46 = pow(x33, 2);
	const GEN_FLT x47 = x38 + x46;
	const GEN_FLT x48 = pow(x47, -1.0 / 2.0) * x33;
	const GEN_FLT x49 = asin(pow(x45, -1) * x48);
	const GEN_FLT x50 = 8.0108022e-06 * x49;
	const GEN_FLT x51 = -8.0108022e-06 + (-1 * x50);
	const GEN_FLT x52 = 0.0028679863 + (x51 * x49);
	const GEN_FLT x53 = 5.3685255e-06 + (x52 * x49);
	const GEN_FLT x54 = 0.0076069798 + (x53 * x49);
	const GEN_FLT x55 = pow(x49, 2);
	const GEN_FLT x56 = x54 * x49;
	const GEN_FLT x57 = -8.0108022e-06 + (-1.60216044e-05 * x49);
	const GEN_FLT x58 = x52 + (x57 * x49);
	const GEN_FLT x59 = x53 + (x58 * x49);
	const GEN_FLT x60 = x54 + (x59 * x49);
	const GEN_FLT x61 = (x60 * x49) + x56;
	const GEN_FLT x62 = sin(x0);
	const GEN_FLT x63 = x62 * x44;
	const GEN_FLT x64 = x63 * x61;
	const GEN_FLT x65 = x45 + (-1 * x64);
	const GEN_FLT x66 = pow(x65, -1);
	const GEN_FLT x67 = x66 * x55;
	const GEN_FLT x68 = x67 * x54;
	const GEN_FLT x69 = x40 + (x68 * x44);
	const GEN_FLT x70 = pow((1 + (-1 * pow(x69, 2))), -1.0 / 2.0);
	const GEN_FLT x71 = pow(x45, -2);
	const GEN_FLT x72 = x46 * pow(x47, -1);
	const GEN_FLT x73 = x71 * x48 * pow((1 + (-1 * x71 * x72)), -1.0 / 2.0);
	const GEN_FLT x74 = pow(x1, 2);
	const GEN_FLT x75 = x39 * (1 + x74);
	const GEN_FLT x76 = x73 * x62;
	const GEN_FLT x77 = x76 * x51;
	const GEN_FLT x78 = (x49 * (x77 + (-1 * x76 * x50))) + (x76 * x52);
	const GEN_FLT x79 = (x78 * x49) + (x76 * x53);
	const GEN_FLT x80 = cos(x42) * ogeePhase_0;
	const GEN_FLT x81 = x46 * pow(x38, -1);
	const GEN_FLT x82 = x75 * pow((1 + (-1 * x81 * x74)), -1.0 / 2.0);
	const GEN_FLT x83 = pow(x65, -2) * x54 * x55;
	const GEN_FLT x84 = x80 * x68;
	const GEN_FLT x85 =
		x70 *
		((x79 * x67 * x44) + (-1 * x82 * x84) +
		 (-1 * x83 * x44 *
		  ((-1 * x61 * x44 * x45) + (x80 * x82 * x61 * x62) +
		   (-1 * x63 *
			((x79 * x49) + (x76 * x54) +
			 (x49 * (x79 + (x49 * (x78 + (x76 * x58) + (x49 * ((x76 * x57) + (-2.40324066e-05 * x76 * x49) + x77)))) +
					 (x76 * x59))) +
			 (x76 * x60))) +
		   (-1 * x62))) +
		 x75 + (2 * x73 * x63 * x66 * x56));
	const GEN_FLT x86 = -1 * x41;
	const GEN_FLT x87 = asin(x69) + x86 + (-1 * gibPhase_0);
	const GEN_FLT x88 = cos(x87) * gibMag_0;
	const GEN_FLT x89 = x83 * x64;
	const GEN_FLT x90 = (x68 + x89) * x70;
	const GEN_FLT x91 = x70 * (x84 + (x80 * x89));
	const GEN_FLT x92 = ((x68 * x43) + (x89 * x43)) * x70;
	const GEN_FLT x93 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x94 = tan(x93);
	const GEN_FLT x95 = -1 * x94 * x39;
	const GEN_FLT x96 = x41 + ogeeMag_1 + (-1 * asin(x95));
	const GEN_FLT x97 = sin(x96);
	const GEN_FLT x98 = (x97 * ogeePhase_1) + curve_1;
	const GEN_FLT x99 = cos(x93);
	const GEN_FLT x100 = asin(pow(x99, -1) * x48);
	const GEN_FLT x101 = 8.0108022e-06 * x100;
	const GEN_FLT x102 = -8.0108022e-06 + (-1 * x101);
	const GEN_FLT x103 = 0.0028679863 + (x100 * x102);
	const GEN_FLT x104 = 5.3685255e-06 + (x100 * x103);
	const GEN_FLT x105 = 0.0076069798 + (x100 * x104);
	const GEN_FLT x106 = pow(x100, 2);
	const GEN_FLT x107 = x100 * x105;
	const GEN_FLT x108 = -8.0108022e-06 + (-1.60216044e-05 * x100);
	const GEN_FLT x109 = x103 + (x100 * x108);
	const GEN_FLT x110 = x104 + (x100 * x109);
	const GEN_FLT x111 = x105 + (x100 * x110);
	const GEN_FLT x112 = (x100 * x111) + x107;
	const GEN_FLT x113 = sin(x93);
	const GEN_FLT x114 = x98 * x113;
	const GEN_FLT x115 = x112 * x114;
	const GEN_FLT x116 = x99 + x115;
	const GEN_FLT x117 = pow(x116, -1);
	const GEN_FLT x118 = x106 * x117;
	const GEN_FLT x119 = x105 * x118;
	const GEN_FLT x120 = x95 + (x98 * x119);
	const GEN_FLT x121 = x86 + asin(x120) + (-1 * gibPhase_1);
	const GEN_FLT x122 = cos(x121) * gibMag_1;
	const GEN_FLT x123 = pow((1 + (-1 * pow(x120, 2))), -1.0 / 2.0);
	const GEN_FLT x124 = pow(x94, 2);
	const GEN_FLT x125 = x39 * (1 + x124);
	const GEN_FLT x126 = cos(x96) * ogeePhase_1;
	const GEN_FLT x127 = x119 * x126;
	const GEN_FLT x128 = x125 * pow((1 + (-1 * x81 * x124)), -1.0 / 2.0);
	const GEN_FLT x129 = pow(x99, -2);
	const GEN_FLT x130 = x48 * x129 * pow((1 + (-1 * x72 * x129)), -1.0 / 2.0);
	const GEN_FLT x131 = x113 * x130;
	const GEN_FLT x132 = -1 * x102 * x131;
	const GEN_FLT x133 = (-1 * x103 * x131) + (x100 * (x132 + (x101 * x131)));
	const GEN_FLT x134 = (-1 * x104 * x131) + (x100 * x133);
	const GEN_FLT x135 = x105 * x106 * pow(x116, -2);
	const GEN_FLT x136 =
		x123 * ((-1 * x98 * x135 *
				 ((x114 * ((x100 * x134) + (-1 * x105 * x131) +
						   (x100 * (x134 +
									(x100 * (x133 + (-1 * x109 * x131) +
											 (x100 * ((-1 * x108 * x131) + (2.40324066e-05 * x100 * x131) + x132)))) +
									(-1 * x110 * x131))) +
						   (-1 * x111 * x131))) +
				  (-1 * x99 * x98 * x112) + (-1 * x112 * x113 * x128 * x126) + x113)) +
				(x98 * x118 * x134) + (-2 * x107 * x114 * x117 * x130) + (-1 * x128 * x127) + x125);
	const GEN_FLT x137 = x115 * x135;
	const GEN_FLT x138 = ((-1 * x137) + x119) * x123;
	const GEN_FLT x139 = x123 * ((-1 * x126 * x137) + x127);
	const GEN_FLT x140 = ((-1 * x97 * x137) + (x97 * x119)) * x123;
	out[0] = -1;
	out[1] = (-1 * x88 * x85) + (-1 * x85);
	out[2] = (-1 * x88 * x90) + (-1 * x90);
	out[3] = x88;
	out[4] = -1 * sin(x87);
	out[5] = (-1 * x88 * x91) + (-1 * x91);
	out[6] = (-1 * x88 * x92) + (-1 * x92);
	out[7] = 0;
	out[8] = 0;
	out[9] = 0;
	out[10] = 0;
	out[11] = 0;
	out[12] = 0;
	out[13] = 0;
	out[14] = 0;
	out[15] = 0;
	out[16] = 0;
	out[17] = 0;
	out[18] = 0;
	out[19] = 0;
	out[20] = 0;
	out[21] = -1;
	out[22] = (-1 * x136) + (-1 * x122 * x136);
	out[23] = (-1 * x138) + (-1 * x122 * x138);
	out[24] = x122;
	out[25] = -1 * sin(x121);
	out[26] = (-1 * x139) + (-1 * x122 * x139);
	out[27] = (-1 * x140) + (-1 * x122 * x140);
}

static inline FLT gen_reproject_axis_x_gen2_axis_angle(const LinmathAxisAnglePose *obj_p, const FLT *sensor_pt,
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qi;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qj;
	const GEN_FLT x10 = x9 * lh_qk;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = pow(obj_qj, 2);
	const GEN_FLT x13 = pow(obj_qi, 2);
	const GEN_FLT x14 = 1e-10 + x13 + x12 + x11;
	const GEN_FLT x15 = pow(x14, 1.0 / 2.0);
	const GEN_FLT x16 = cos(x15);
	const GEN_FLT x17 = pow(x14, -1) * (1 + (-1 * x16));
	const GEN_FLT x18 = pow(x15, -1) * sin(x15);
	const GEN_FLT x19 = x18 * obj_qi;
	const GEN_FLT x20 = x17 * obj_qk;
	const GEN_FLT x21 = x20 * obj_qj;
	const GEN_FLT x22 = x18 * obj_qj;
	const GEN_FLT x23 = x20 * obj_qi;
	const GEN_FLT x24 =
		((x21 + x19) * sensor_y) + ((x23 + (-1 * x22)) * sensor_x) + ((x16 + (x11 * x17)) * sensor_z) + obj_pz;
	const GEN_FLT x25 = x18 * obj_qk;
	const GEN_FLT x26 = x17 * obj_qj * obj_qi;
	const GEN_FLT x27 =
		((x26 + x25) * sensor_x) + ((x16 + (x12 * x17)) * sensor_y) + ((x21 + (-1 * x19)) * sensor_z) + obj_py;
	const GEN_FLT x28 =
		((x26 + (-1 * x25)) * sensor_y) + ((x23 + x22) * sensor_z) + ((x16 + (x13 * x17)) * sensor_x) + obj_px;
	const GEN_FLT x29 = x5 * lh_qk;
	const GEN_FLT x30 = x9 * lh_qi;
	const GEN_FLT x31 = lh_py + ((x30 + x29) * x28) + (x27 * (x7 + (x1 * x8))) + (x24 * (x10 + (-1 * x6)));
	const GEN_FLT x32 = 0.523598775598299 + tilt_0;
	const GEN_FLT x33 = cos(x32);
	const GEN_FLT x34 = x5 * lh_qj;
	const GEN_FLT x35 = x8 * lh_qk * lh_qi;
	const GEN_FLT x36 = ((x35 + (-1 * x34)) * x28) + (x27 * (x10 + x6)) + (x24 * (x7 + (x0 * x8))) + lh_pz;
	const GEN_FLT x37 = ((x30 + (-1 * x29)) * x27) + ((x35 + x34) * x24) + (x28 * (x7 + (x2 * x8))) + lh_px;
	const GEN_FLT x38 = pow(x37, 2) + pow(x36, 2);
	const GEN_FLT x39 = asin(x31 * pow(x33, -1) * pow((x38 + pow(x31, 2)), -1.0 / 2.0));
	const GEN_FLT x40 = 0.0028679863 + (x39 * (-8.0108022e-06 + (-8.0108022e-06 * x39)));
	const GEN_FLT x41 = 5.3685255e-06 + (x40 * x39);
	const GEN_FLT x42 = 0.0076069798 + (x41 * x39);
	const GEN_FLT x43 = x31 * pow(x38, -1.0 / 2.0) * tan(x32);
	const GEN_FLT x44 = atan2(-1 * x36, x37);
	const GEN_FLT x45 = (sin(x44 + (-1 * asin(x43)) + ogeeMag_0) * ogeePhase_0) + curve_0;
	const GEN_FLT x46 = asin(
		x43 +
		(x42 * x45 * pow(x39, 2) *
		 pow((x33 +
			  (-1 * x45 * sin(x32) *
			   ((x39 * (x42 + (x39 * (x41 + (x39 * (x40 + (x39 * (-8.0108022e-06 + (-1.60216044e-05 * x39))))))))) +
				(x42 * x39)))),
			 -1)));
	return -1.5707963267949 + x44 + (-1 * x46) + (sin(x44 + (-1 * x46) + gibPhase_0) * gibMag_0) + (-1 * phase_0);
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qj;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qk * lh_qi;
	const GEN_FLT x10 = x9 + (-1 * x6);
	const GEN_FLT x11 = x9 + x6;
	const GEN_FLT x12 = pow(obj_qk, 2);
	const GEN_FLT x13 = pow(obj_qj, 2);
	const GEN_FLT x14 = pow(obj_qi, 2);
	const GEN_FLT x15 = 1e-10 + x14 + x13 + x12;
	const GEN_FLT x16 = pow(x15, -1);
	const GEN_FLT x17 = pow(x15, 1.0 / 2.0);
	const GEN_FLT x18 = cos(x17);
	const GEN_FLT x19 = 1 + (-1 * x18);
	const GEN_FLT x20 = x19 * x16;
	const GEN_FLT x21 = sin(x17);
	const GEN_FLT x22 = x21 * pow(x17, -1);
	const GEN_FLT x23 = x22 * obj_qi;
	const GEN_FLT x24 = x20 * obj_qk;
	const GEN_FLT x25 = x24 * obj_qj;
	const GEN_FLT x26 = x22 * obj_qj;
	const GEN_FLT x27 = -1 * x26;
	const GEN_FLT x28 = x24 * obj_qi;
	const GEN_FLT x29 = ((x28 + x27) * sensor_x) + ((x25 + x23) * sensor_y) + ((x18 + (x20 * x12)) * sensor_z) + obj_pz;
	const GEN_FLT x30 = -1 * x23;
	const GEN_FLT x31 = x22 * obj_qk;
	const GEN_FLT x32 = x20 * obj_qi;
	const GEN_FLT x33 = x32 * obj_qj;
	const GEN_FLT x34 = ((x33 + x31) * sensor_x) + ((x25 + x30) * sensor_z) + ((x18 + (x20 * x13)) * sensor_y) + obj_py;
	const GEN_FLT x35 = x5 * lh_qk;
	const GEN_FLT x36 = x8 * lh_qj;
	const GEN_FLT x37 = x36 * lh_qi;
	const GEN_FLT x38 = x37 + (-1 * x35);
	const GEN_FLT x39 = -1 * x31;
	const GEN_FLT x40 = ((x18 + (x20 * x14)) * sensor_x) + ((x33 + x39) * sensor_y) + ((x28 + x26) * sensor_z) + obj_px;
	const GEN_FLT x41 = x7 + (x2 * x8);
	const GEN_FLT x42 = (x40 * x41) + (x29 * x11) + (x34 * x38) + lh_px;
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = x7 + (x0 * x8);
	const GEN_FLT x45 = x5 * lh_qi;
	const GEN_FLT x46 = x36 * lh_qk;
	const GEN_FLT x47 = x46 + x45;
	const GEN_FLT x48 = (x40 * x10) + (x44 * x29) + (x47 * x34) + lh_pz;
	const GEN_FLT x49 = pow(x42, 2);
	const GEN_FLT x50 = x48 * pow(x49, -1);
	const GEN_FLT x51 = x49 + pow(x48, 2);
	const GEN_FLT x52 = pow(x51, -1);
	const GEN_FLT x53 = x52 * x49;
	const GEN_FLT x54 = ((x50 * x41) + (-1 * x43 * x10)) * x53;
	const GEN_FLT x55 = x46 + (-1 * x45);
	const GEN_FLT x56 = x7 + (x1 * x8);
	const GEN_FLT x57 = x37 + x35;
	const GEN_FLT x58 = (x57 * x40) + lh_py + (x56 * x34) + (x55 * x29);
	const GEN_FLT x59 = 0.523598775598299 + tilt_0;
	const GEN_FLT x60 = cos(x59);
	const GEN_FLT x61 = pow(x60, -1);
	const GEN_FLT x62 = pow(x58, 2);
	const GEN_FLT x63 = x51 + x62;
	const GEN_FLT x64 = pow(x63, -1.0 / 2.0) * x61;
	const GEN_FLT x65 = asin(x64 * x58);
	const GEN_FLT x66 = 8.0108022e-06 * x65;
	const GEN_FLT x67 = -8.0108022e-06 + (-1 * x66);
	const GEN_FLT x68 = 0.0028679863 + (x67 * x65);
	const GEN_FLT x69 = 5.3685255e-06 + (x68 * x65);
	const GEN_FLT x70 = 0.0076069798 + (x65 * x69);
	const GEN_FLT x71 = x70 * x65;
	const GEN_FLT x72 = -8.0108022e-06 + (-1.60216044e-05 * x65);
	const GEN_FLT x73 = x68 + (x72 * x65);
	const GEN_FLT x74 = x69 + (x73 * x65);
	const GEN_FLT x75 = x70 + (x74 * x65);
	const GEN_FLT x76 = (x75 * x65) + x71;
	const GEN_FLT x77 = tan(x59);
	const GEN_FLT x78 = x77 * pow(x51, -1.0 / 2.0);
	const GEN_FLT x79 = x78 * x58;
	const GEN_FLT x80 = atan2(-1 * x48, x42);
	const GEN_FLT x81 = x80 + (-1 * asin(x79)) + ogeeMag_0;
	const GEN_FLT x82 = (sin(x81) * ogeePhase_0) + curve_0;
	const GEN_FLT x83 = sin(x59);
	const GEN_FLT x84 = x82 * x83;
	const GEN_FLT x85 = x60 + (-1 * x84 * x76);
	const GEN_FLT x86 = pow(x85, -1);
	const GEN_FLT x87 = pow(x65, 2);
	const GEN_FLT x88 = x82 * x87;
	const GEN_FLT x89 = x88 * x86;
	const GEN_FLT x90 = x79 + (x89 * x70);
	const GEN_FLT x91 = pow((1 + (-1 * pow(x90, 2))), -1.0 / 2.0);
	const GEN_FLT x92 = pow((1 + (-1 * pow(x60, -2) * pow(x63, -1) * x62)), -1.0 / 2.0);
	const GEN_FLT x93 = 2 * x58;
	const GEN_FLT x94 = 2 * x42;
	const GEN_FLT x95 = 2 * x48;
	const GEN_FLT x96 = (x95 * x10) + (x94 * x41);
	const GEN_FLT x97 = 1.0 / 2.0 * x58;
	const GEN_FLT x98 = pow(x63, -3.0 / 2.0) * x61 * x97;
	const GEN_FLT x99 = x92 * ((x64 * x57) + (-1 * x98 * (x96 + (x57 * x93))));
	const GEN_FLT x100 = 2 * x82 * x86 * x71;
	const GEN_FLT x101 = x67 * x99;
	const GEN_FLT x102 = 2.40324066e-05 * x65;
	const GEN_FLT x103 = (x65 * (x101 + (-1 * x66 * x99))) + (x68 * x99);
	const GEN_FLT x104 = (x65 * x103) + (x69 * x99);
	const GEN_FLT x105 = pow((1 + (-1 * pow(x77, 2) * x62 * x52)), -1.0 / 2.0);
	const GEN_FLT x106 = x77 * pow(x51, -3.0 / 2.0) * x97;
	const GEN_FLT x107 = (x78 * x57) + (-1 * x96 * x106);
	const GEN_FLT x108 = cos(x81) * ogeePhase_0;
	const GEN_FLT x109 = x108 * ((-1 * x105 * x107) + x54);
	const GEN_FLT x110 = x83 * x76;
	const GEN_FLT x111 = x88 * pow(x85, -2) * x70;
	const GEN_FLT x112 = x86 * x87 * x70;
	const GEN_FLT x113 =
		x91 * (x107 + (x89 * x104) + (x109 * x112) +
			   (-1 * x111 *
				((-1 * x109 * x110) +
				 (-1 * x84 *
				  ((x65 * x104) + (x70 * x99) +
				   (x65 * (x104 + (x65 * (x103 + (x73 * x99) + (x65 * ((x72 * x99) + (-1 * x99 * x102) + x101)))) +
						   (x74 * x99))) +
				   (x75 * x99))))) +
			   (x99 * x100));
	const GEN_FLT x114 = cos(x80 + (-1 * asin(x90)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x115 = ((x50 * x38) + (-1 * x43 * x47)) * x53;
	const GEN_FLT x116 = (x95 * x47) + (x94 * x38);
	const GEN_FLT x117 = x92 * ((x64 * x56) + (-1 * x98 * (x116 + (x56 * x93))));
	const GEN_FLT x118 = x67 * x117;
	const GEN_FLT x119 = (x65 * (x118 + (-1 * x66 * x117))) + (x68 * x117);
	const GEN_FLT x120 = (x65 * x119) + (x69 * x117);
	const GEN_FLT x121 = (x78 * x56) + (-1 * x106 * x116);
	const GEN_FLT x122 = (-1 * x105 * x121) + x115;
	const GEN_FLT x123 = x108 * x110;
	const GEN_FLT x124 = x108 * x112;
	const GEN_FLT x125 =
		x91 * ((x124 * x122) +
			   (-1 * x111 *
				((-1 * x123 * x122) +
				 (-1 * x84 *
				  ((x70 * x117) +
				   (x65 * (x120 + (x65 * (x119 + (x73 * x117) + (x65 * ((x72 * x117) + (-1 * x102 * x117) + x118)))) +
						   (x74 * x117))) +
				   (x65 * x120) + (x75 * x117))))) +
			   (x89 * x120) + x121 + (x100 * x117));
	const GEN_FLT x126 = ((x50 * x11) + (-1 * x43 * x44)) * x53;
	const GEN_FLT x127 = (x95 * x44) + (x94 * x11);
	const GEN_FLT x128 = x92 * ((x64 * x55) + (-1 * x98 * (x127 + (x55 * x93))));
	const GEN_FLT x129 = (x78 * x55) + (-1 * x106 * x127);
	const GEN_FLT x130 = (-1 * x105 * x129) + x126;
	const GEN_FLT x131 = x67 * x128;
	const GEN_FLT x132 = (x65 * (x131 + (-1 * x66 * x128))) + (x68 * x128);
	const GEN_FLT x133 = (x65 * x132) + (x69 * x128);
	const GEN_FLT x134 =
		x91 * ((x89 * x133) + x129 +
			   (-1 * x111 *
				((-1 * x123 * x130) +
				 (-1 * x84 *
				  ((x70 * x128) + (x65 * x133) +
				   (x65 * (x133 + (x65 * (x132 + (x73 * x128) + (x65 * ((x72 * x128) + (-1 * x102 * x128) + x131)))) +
						   (x74 * x128))) +
				   (x75 * x128))))) +
			   (x124 * x130) + (x100 * x128));
	const GEN_FLT x135 = pow(obj_qi, 3);
	const GEN_FLT x136 = x21 * pow(x15, -3.0 / 2.0);
	const GEN_FLT x137 = 2 * pow(x15, -2) * x19;
	const GEN_FLT x138 = x18 * x16;
	const GEN_FLT x139 = x138 * obj_qk;
	const GEN_FLT x140 = x139 * obj_qi;
	const GEN_FLT x141 = x136 * obj_qk;
	const GEN_FLT x142 = x141 * obj_qi;
	const GEN_FLT x143 = x142 + (-1 * x140);
	const GEN_FLT x144 = x20 * obj_qj;
	const GEN_FLT x145 = x14 * x136;
	const GEN_FLT x146 = x14 * x137;
	const GEN_FLT x147 = (-1 * x146 * obj_qj) + (x145 * obj_qj);
	const GEN_FLT x148 = x147 + x144;
	const GEN_FLT x149 = (-1 * x146 * obj_qk) + (x145 * obj_qk);
	const GEN_FLT x150 = x149 + x24;
	const GEN_FLT x151 = obj_qj * obj_qi;
	const GEN_FLT x152 = x138 * x151;
	const GEN_FLT x153 = x136 * x151;
	const GEN_FLT x154 = (-1 * x153) + x152;
	const GEN_FLT x155 = ((x154 + x150) * sensor_z) + ((x148 + x143) * sensor_y) +
						 (((-1 * x137 * x135) + (x135 * x136) + (2 * x32) + x30) * sensor_x);
	const GEN_FLT x156 = (-1 * x142) + x140;
	const GEN_FLT x157 = x13 * x136;
	const GEN_FLT x158 = x137 * obj_qi;
	const GEN_FLT x159 = (-1 * x13 * x158) + (x157 * obj_qi);
	const GEN_FLT x160 = x14 * x138;
	const GEN_FLT x161 = x137 * obj_qk;
	const GEN_FLT x162 = (-1 * x161 * x151) + (x153 * obj_qk);
	const GEN_FLT x163 = x162 + (-1 * x22);
	const GEN_FLT x164 =
		((x163 + x145 + (-1 * x160)) * sensor_z) + ((x159 + x30) * sensor_y) + ((x148 + x156) * sensor_x);
	const GEN_FLT x165 = x153 + (-1 * x152);
	const GEN_FLT x166 = x162 + x22;
	const GEN_FLT x167 = x12 * x136;
	const GEN_FLT x168 = (-1 * x12 * x158) + (x167 * obj_qi);
	const GEN_FLT x169 =
		((x168 + x30) * sensor_z) + ((x166 + (-1 * x145) + x160) * sensor_y) + ((x150 + x165) * sensor_x);
	const GEN_FLT x170 = (x44 * x169) + (x47 * x164) + (x10 * x155);
	const GEN_FLT x171 = (x11 * x169) + (x38 * x164) + (x41 * x155);
	const GEN_FLT x172 = ((x50 * x171) + (-1 * x43 * x170)) * x53;
	const GEN_FLT x173 = (x56 * x164) + (x55 * x169) + (x57 * x155);
	const GEN_FLT x174 = (x95 * x170) + (x94 * x171);
	const GEN_FLT x175 = x92 * ((x64 * x173) + (-1 * x98 * (x174 + (x93 * x173))));
	const GEN_FLT x176 = x67 * x175;
	const GEN_FLT x177 = (x65 * (x176 + (-1 * x66 * x175))) + (x68 * x175);
	const GEN_FLT x178 = (x65 * x177) + (x69 * x175);
	const GEN_FLT x179 = (x78 * x173) + (-1 * x106 * x174);
	const GEN_FLT x180 = (-1 * x105 * x179) + x172;
	const GEN_FLT x181 =
		x91 * (x179 + (x89 * x178) + (x124 * x180) +
			   (-1 * x111 *
				((-1 * x123 * x180) +
				 (-1 * x84 *
				  ((x65 * x178) + (x70 * x175) +
				   (x65 * ((x65 * (x177 + (x73 * x175) + (x65 * ((x72 * x175) + (-1 * x102 * x175) + x176)))) + x178 +
						   (x74 * x175))) +
				   (x75 * x175))))) +
			   (x100 * x175));
	const GEN_FLT x182 = x159 + x32;
	const GEN_FLT x183 = x139 * obj_qj;
	const GEN_FLT x184 = x141 * obj_qj;
	const GEN_FLT x185 = (-1 * x184) + x183;
	const GEN_FLT x186 = pow(obj_qj, 3);
	const GEN_FLT x187 = (-1 * x13 * x161) + (x157 * obj_qk);
	const GEN_FLT x188 = x187 + x24;
	const GEN_FLT x189 = ((x165 + x188) * sensor_z) +
						 (((-1 * x186 * x137) + (x186 * x136) + (2 * x144) + x27) * sensor_y) +
						 ((x185 + x182) * sensor_x);
	const GEN_FLT x190 = x184 + (-1 * x183);
	const GEN_FLT x191 = x13 * x138;
	const GEN_FLT x192 =
		((x166 + (-1 * x157) + x191) * sensor_z) + ((x190 + x182) * sensor_y) + ((x147 + x27) * sensor_x);
	const GEN_FLT x193 = (-1 * x12 * x137 * obj_qj) + (x167 * obj_qj);
	const GEN_FLT x194 =
		((x154 + x188) * sensor_y) + ((x193 + x27) * sensor_z) + ((x163 + x157 + (-1 * x191)) * sensor_x);
	const GEN_FLT x195 = (x44 * x194) + (x10 * x192) + (x47 * x189);
	const GEN_FLT x196 = (x11 * x194) + (x38 * x189) + (x41 * x192);
	const GEN_FLT x197 = ((x50 * x196) + (-1 * x43 * x195)) * x53;
	const GEN_FLT x198 = (x55 * x194) + (x56 * x189) + (x57 * x192);
	const GEN_FLT x199 = (x95 * x195) + (x94 * x196);
	const GEN_FLT x200 = x92 * ((x64 * x198) + (-1 * x98 * (x199 + (x93 * x198))));
	const GEN_FLT x201 = x67 * x200;
	const GEN_FLT x202 = (x65 * (x201 + (-1 * x66 * x200))) + (x68 * x200);
	const GEN_FLT x203 = (x65 * x202) + (x69 * x200);
	const GEN_FLT x204 = (x78 * x198) + (-1 * x106 * x199);
	const GEN_FLT x205 = (-1 * x204 * x105) + x197;
	const GEN_FLT x206 =
		x91 * (x204 + (x205 * x124) + (x89 * x203) +
			   (-1 * x111 *
				((-1 * x205 * x123) +
				 (-1 * x84 *
				  ((x70 * x200) +
				   (x65 * (x203 + (x65 * ((x73 * x200) + x202 + (x65 * ((x72 * x200) + (-1 * x200 * x102) + x201)))) +
						   (x74 * x200))) +
				   (x65 * x203) + (x75 * x200))))) +
			   (x200 * x100));
	const GEN_FLT x207 = x12 * x138;
	const GEN_FLT x208 = x168 + x32;
	const GEN_FLT x209 =
		((x208 + x185) * sensor_z) + ((x163 + x167 + (-1 * x207)) * sensor_y) + ((x149 + x39) * sensor_x);
	const GEN_FLT x210 = x193 + x144;
	const GEN_FLT x211 =
		((x143 + x210) * sensor_z) + ((x187 + x39) * sensor_y) + ((x166 + (-1 * x167) + x207) * sensor_x);
	const GEN_FLT x212 = pow(obj_qk, 3);
	const GEN_FLT x213 = (((-1 * x212 * x137) + x39 + (x212 * x136) + (2 * x24)) * sensor_z) +
						 ((x156 + x210) * sensor_y) + ((x208 + x190) * sensor_x);
	const GEN_FLT x214 = (x44 * x213) + (x47 * x211) + (x10 * x209);
	const GEN_FLT x215 = (x11 * x213) + (x38 * x211) + (x41 * x209);
	const GEN_FLT x216 = ((x50 * x215) + (-1 * x43 * x214)) * x53;
	const GEN_FLT x217 = (x55 * x213) + (x56 * x211) + (x57 * x209);
	const GEN_FLT x218 = (x95 * x214) + (x94 * x215);
	const GEN_FLT x219 = x92 * ((x64 * x217) + (-1 * x98 * (x218 + (x93 * x217))));
	const GEN_FLT x220 = x67 * x219;
	const GEN_FLT x221 = (x65 * (x220 + (-1 * x66 * x219))) + (x68 * x219);
	const GEN_FLT x222 = (x65 * x221) + (x69 * x219);
	const GEN_FLT x223 = (x78 * x217) + (-1 * x218 * x106);
	const GEN_FLT x224 = (-1 * x223 * x105) + x216;
	const GEN_FLT x225 =
		x91 * (x223 + (x89 * x222) + (x224 * x124) +
			   (-1 * x111 *
				((-1 * x224 * x123) +
				 (-1 * x84 *
				  ((x70 * x219) +
				   (x65 * (x222 + (x65 * (x221 + (x73 * x219) + (x65 * ((x72 * x219) + (-1 * x219 * x102) + x220)))) +
						   (x74 * x219))) +
				   (x65 * x222) + (x75 * x219))))) +
			   (x219 * x100));
	out[0] = (-1 * x113) + (-1 * x114 * (x113 + (-1 * x54))) + x54;
	out[1] = (-1 * (x125 + (-1 * x115)) * x114) + (-1 * x125) + x115;
	out[2] = (-1 * (x134 + (-1 * x126)) * x114) + (-1 * x134) + x126;
	out[3] = (-1 * (x181 + (-1 * x172)) * x114) + (-1 * x181) + x172;
	out[4] = (-1 * (x206 + (-1 * x197)) * x114) + (-1 * x206) + x197;
	out[5] = (-1 * (x225 + (-1 * x216)) * x114) + (-1 * x225) + x216;
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qj;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qi;
	const GEN_FLT x10 = x9 * lh_qk;
	const GEN_FLT x11 = x10 + x6;
	const GEN_FLT x12 = pow(obj_qk, 2);
	const GEN_FLT x13 = pow(obj_qj, 2);
	const GEN_FLT x14 = pow(obj_qi, 2);
	const GEN_FLT x15 = 1e-10 + x14 + x13 + x12;
	const GEN_FLT x16 = pow(x15, 1.0 / 2.0);
	const GEN_FLT x17 = cos(x16);
	const GEN_FLT x18 = pow(x15, -1) * (1 + (-1 * x17));
	const GEN_FLT x19 = x17 + (x12 * x18);
	const GEN_FLT x20 = pow(x16, -1) * sin(x16);
	const GEN_FLT x21 = x20 * obj_qi;
	const GEN_FLT x22 = x18 * obj_qk;
	const GEN_FLT x23 = x22 * obj_qj;
	const GEN_FLT x24 = x23 + x21;
	const GEN_FLT x25 = x20 * obj_qj;
	const GEN_FLT x26 = x22 * obj_qi;
	const GEN_FLT x27 = x26 + (-1 * x25);
	const GEN_FLT x28 = (x19 * sensor_z) + (x27 * sensor_x) + (x24 * sensor_y) + obj_pz;
	const GEN_FLT x29 = x23 + (-1 * x21);
	const GEN_FLT x30 = x17 + (x13 * x18);
	const GEN_FLT x31 = x20 * obj_qk;
	const GEN_FLT x32 = x18 * obj_qj * obj_qi;
	const GEN_FLT x33 = x32 + x31;
	const GEN_FLT x34 = (x30 * sensor_y) + (x33 * sensor_x) + (x29 * sensor_z) + obj_py;
	const GEN_FLT x35 = x5 * lh_qk;
	const GEN_FLT x36 = x9 * lh_qj;
	const GEN_FLT x37 = x36 + (-1 * x35);
	const GEN_FLT x38 = x26 + x25;
	const GEN_FLT x39 = x32 + (-1 * x31);
	const GEN_FLT x40 = x17 + (x14 * x18);
	const GEN_FLT x41 = (x40 * sensor_x) + (x39 * sensor_y) + (x38 * sensor_z) + obj_px;
	const GEN_FLT x42 = x7 + (x2 * x8);
	const GEN_FLT x43 = (x41 * x42) + (x34 * x37) + (x28 * x11) + lh_px;
	const GEN_FLT x44 = pow(x43, -1);
	const GEN_FLT x45 = x10 + (-1 * x6);
	const GEN_FLT x46 = x5 * lh_qi;
	const GEN_FLT x47 = x8 * lh_qk * lh_qj;
	const GEN_FLT x48 = x47 + x46;
	const GEN_FLT x49 = x7 + (x0 * x8);
	const GEN_FLT x50 = (x49 * x27) + (x48 * x33) + (x40 * x45);
	const GEN_FLT x51 = (x27 * x11) + (x33 * x37) + (x40 * x42);
	const GEN_FLT x52 = (x41 * x45) + (x48 * x34) + (x49 * x28) + lh_pz;
	const GEN_FLT x53 = pow(x43, 2);
	const GEN_FLT x54 = pow(x53, -1) * x52;
	const GEN_FLT x55 = x53 + pow(x52, 2);
	const GEN_FLT x56 = pow(x55, -1);
	const GEN_FLT x57 = x53 * x56;
	const GEN_FLT x58 = ((x54 * x51) + (-1 * x50 * x44)) * x57;
	const GEN_FLT x59 = x47 + (-1 * x46);
	const GEN_FLT x60 = x7 + (x1 * x8);
	const GEN_FLT x61 = x36 + x35;
	const GEN_FLT x62 = (x61 * x41) + lh_py + (x60 * x34) + (x59 * x28);
	const GEN_FLT x63 = pow(x62, 2);
	const GEN_FLT x64 = 0.523598775598299 + tilt_0;
	const GEN_FLT x65 = cos(x64);
	const GEN_FLT x66 = x55 + x63;
	const GEN_FLT x67 = pow((1 + (-1 * x63 * pow(x65, -2) * pow(x66, -1))), -1.0 / 2.0);
	const GEN_FLT x68 = (x59 * x27) + (x60 * x33) + (x61 * x40);
	const GEN_FLT x69 = 2 * x62;
	const GEN_FLT x70 = 2 * x43;
	const GEN_FLT x71 = 2 * x52;
	const GEN_FLT x72 = (x71 * x50) + (x70 * x51);
	const GEN_FLT x73 = pow(x65, -1);
	const GEN_FLT x74 = 1.0 / 2.0 * x62;
	const GEN_FLT x75 = x73 * x74 * pow(x66, -3.0 / 2.0);
	const GEN_FLT x76 = x73 * pow(x66, -1.0 / 2.0);
	const GEN_FLT x77 = (x76 * x68) + (-1 * x75 * (x72 + (x68 * x69)));
	const GEN_FLT x78 = x77 * x67;
	const GEN_FLT x79 = tan(x64);
	const GEN_FLT x80 = x79 * pow(x55, -1.0 / 2.0);
	const GEN_FLT x81 = x80 * x62;
	const GEN_FLT x82 = atan2(-1 * x52, x43);
	const GEN_FLT x83 = x82 + (-1 * asin(x81)) + ogeeMag_0;
	const GEN_FLT x84 = (sin(x83) * ogeePhase_0) + curve_0;
	const GEN_FLT x85 = asin(x76 * x62);
	const GEN_FLT x86 = 8.0108022e-06 * x85;
	const GEN_FLT x87 = -8.0108022e-06 + (-1 * x86);
	const GEN_FLT x88 = 0.0028679863 + (x85 * x87);
	const GEN_FLT x89 = 5.3685255e-06 + (x88 * x85);
	const GEN_FLT x90 = 0.0076069798 + (x89 * x85);
	const GEN_FLT x91 = x85 * x90;
	const GEN_FLT x92 = -8.0108022e-06 + (-1.60216044e-05 * x85);
	const GEN_FLT x93 = x88 + (x85 * x92);
	const GEN_FLT x94 = x89 + (x85 * x93);
	const GEN_FLT x95 = x90 + (x85 * x94);
	const GEN_FLT x96 = (x85 * x95) + x91;
	const GEN_FLT x97 = sin(x64);
	const GEN_FLT x98 = x84 * x97;
	const GEN_FLT x99 = x65 + (-1 * x98 * x96);
	const GEN_FLT x100 = pow(x99, -1);
	const GEN_FLT x101 = 2 * x84 * x91 * x100;
	const GEN_FLT x102 = x87 * x78;
	const GEN_FLT x103 = 2.40324066e-05 * x85;
	const GEN_FLT x104 = (x85 * (x102 + (-1 * x86 * x78))) + (x88 * x78);
	const GEN_FLT x105 = x89 * x67;
	const GEN_FLT x106 = (x85 * x104) + (x77 * x105);
	const GEN_FLT x107 = pow((1 + (-1 * pow(x79, 2) * x63 * x56)), -1.0 / 2.0);
	const GEN_FLT x108 = x79 * x74 * pow(x55, -3.0 / 2.0);
	const GEN_FLT x109 = (x80 * x68) + (-1 * x72 * x108);
	const GEN_FLT x110 = (-1 * x109 * x107) + x58;
	const GEN_FLT x111 = cos(x83) * ogeePhase_0;
	const GEN_FLT x112 = x97 * x96;
	const GEN_FLT x113 = x111 * x112;
	const GEN_FLT x114 = pow(x85, 2);
	const GEN_FLT x115 = x84 * x114;
	const GEN_FLT x116 = x90 * pow(x99, -2) * x115;
	const GEN_FLT x117 = x90 * x100 * x114;
	const GEN_FLT x118 = x111 * x117;
	const GEN_FLT x119 = x100 * x115;
	const GEN_FLT x120 = x81 + (x90 * x119);
	const GEN_FLT x121 = pow((1 + (-1 * pow(x120, 2))), -1.0 / 2.0);
	const GEN_FLT x122 =
		x121 * ((x106 * x119) + (x110 * x118) + x109 +
				(-1 * x116 *
				 ((-1 * x110 * x113) +
				  (-1 * x98 *
				   ((x85 * x106) + (x78 * x90) + (x78 * x95) +
					(x85 * (x106 + (x85 * (x104 + (x78 * x93) + (x85 * ((x78 * x92) + (-1 * x78 * x103) + x102)))) +
							(x78 * x94))))))) +
				(x78 * x101));
	const GEN_FLT x123 = cos(x82 + (-1 * asin(x120)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x124 = (x49 * x24) + (x48 * x30) + (x45 * x39);
	const GEN_FLT x125 = (x24 * x11) + (x30 * x37) + (x42 * x39);
	const GEN_FLT x126 = ((x54 * x125) + (-1 * x44 * x124)) * x57;
	const GEN_FLT x127 = (x59 * x24) + (x60 * x30) + (x61 * x39);
	const GEN_FLT x128 = (x71 * x124) + (x70 * x125);
	const GEN_FLT x129 = (x76 * x127) + (-1 * x75 * (x128 + (x69 * x127)));
	const GEN_FLT x130 = x67 * x129;
	const GEN_FLT x131 = x87 * x130;
	const GEN_FLT x132 = (x85 * (x131 + (-1 * x86 * x130))) + (x88 * x130);
	const GEN_FLT x133 = (x85 * x132) + (x105 * x129);
	const GEN_FLT x134 = (x80 * x127) + (-1 * x108 * x128);
	const GEN_FLT x135 = (-1 * x107 * x134) + x126;
	const GEN_FLT x136 =
		x121 * (x134 + (x118 * x135) + (x119 * x133) +
				(-1 * x116 *
				 ((-1 * x113 * x135) +
				  (-1 * x98 *
				   ((x85 * x133) +
					(x85 * (x133 + (x85 * (x132 + (x93 * x130) + (x85 * ((x92 * x130) + (-1 * x103 * x130) + x131)))) +
							(x94 * x130))) +
					(x90 * x130) + (x95 * x130))))) +
				(x101 * x130));
	const GEN_FLT x137 = (x49 * x19) + (x48 * x29) + (x45 * x38);
	const GEN_FLT x138 = (x11 * x19) + (x37 * x29) + (x42 * x38);
	const GEN_FLT x139 = ((x54 * x138) + (-1 * x44 * x137)) * x57;
	const GEN_FLT x140 = (x59 * x19) + (x60 * x29) + (x61 * x38);
	const GEN_FLT x141 = (x71 * x137) + (x70 * x138);
	const GEN_FLT x142 = (x76 * x140) + (-1 * x75 * (x141 + (x69 * x140)));
	const GEN_FLT x143 = x67 * x142;
	const GEN_FLT x144 = x87 * x143;
	const GEN_FLT x145 = (x85 * (x144 + (-1 * x86 * x143))) + (x88 * x143);
	const GEN_FLT x146 = (x85 * x145) + (x105 * x142);
	const GEN_FLT x147 = (x80 * x140) + (-1 * x108 * x141);
	const GEN_FLT x148 = x111 * ((-1 * x107 * x147) + x139);
	const GEN_FLT x149 =
		x121 * (x147 + (x119 * x146) + (x117 * x148) +
				(-1 * x116 *
				 ((-1 * x112 * x148) +
				  (-1 * x98 *
				   ((x85 * x146) + (x90 * x143) +
					(x85 * (x146 + (x85 * (x145 + (x93 * x143) + (x85 * ((x92 * x143) + (-1 * x103 * x143) + x144)))) +
							(x94 * x143))) +
					(x95 * x143))))) +
				(x101 * x143));
	out[0] = (-1 * x123 * (x122 + (-1 * x58))) + (-1 * x122) + x58;
	out[1] = (-1 * (x136 + (-1 * x126)) * x123) + (-1 * x136) + x126;
	out[2] = (-1 * (x149 + (-1 * x139)) * x123) + (-1 * x149) + x139;
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
	const GEN_FLT x0 = pow(obj_qk, 2);
	const GEN_FLT x1 = pow(obj_qj, 2);
	const GEN_FLT x2 = pow(obj_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = pow(x3, -1) * (1 + (-1 * x5));
	const GEN_FLT x7 = pow(x4, -1) * sin(x4);
	const GEN_FLT x8 = x7 * obj_qi;
	const GEN_FLT x9 = x6 * obj_qk;
	const GEN_FLT x10 = x9 * obj_qj;
	const GEN_FLT x11 = x7 * obj_qj;
	const GEN_FLT x12 = x9 * obj_qi;
	const GEN_FLT x13 =
		((x12 + (-1 * x11)) * sensor_x) + ((x10 + x8) * sensor_y) + ((x5 + (x0 * x6)) * sensor_z) + obj_pz;
	const GEN_FLT x14 = pow(lh_qk, 2);
	const GEN_FLT x15 = pow(lh_qj, 2);
	const GEN_FLT x16 = pow(lh_qi, 2);
	const GEN_FLT x17 = 1e-10 + x16 + x15 + x14;
	const GEN_FLT x18 = pow(x17, -1);
	const GEN_FLT x19 = pow(x17, 1.0 / 2.0);
	const GEN_FLT x20 = cos(x19);
	const GEN_FLT x21 = 1 + (-1 * x20);
	const GEN_FLT x22 = x21 * x18;
	const GEN_FLT x23 = x7 * obj_qk;
	const GEN_FLT x24 = x6 * obj_qj * obj_qi;
	const GEN_FLT x25 =
		((x24 + x23) * sensor_x) + ((x5 + (x1 * x6)) * sensor_y) + ((x10 + (-1 * x8)) * sensor_z) + obj_py;
	const GEN_FLT x26 = sin(x19);
	const GEN_FLT x27 = x26 * pow(x19, -1);
	const GEN_FLT x28 = x27 * lh_qi;
	const GEN_FLT x29 = x22 * lh_qj;
	const GEN_FLT x30 = x29 * lh_qk;
	const GEN_FLT x31 =
		((x24 + (-1 * x23)) * sensor_y) + ((x5 + (x2 * x6)) * sensor_x) + ((x12 + x11) * sensor_z) + obj_px;
	const GEN_FLT x32 = x27 * lh_qj;
	const GEN_FLT x33 = -1 * x32;
	const GEN_FLT x34 = x22 * lh_qi;
	const GEN_FLT x35 = x34 * lh_qk;
	const GEN_FLT x36 = ((x35 + x33) * x31) + (x13 * (x20 + (x22 * x14))) + ((x30 + x28) * x25) + lh_pz;
	const GEN_FLT x37 = x27 * lh_qk;
	const GEN_FLT x38 = -1 * x37;
	const GEN_FLT x39 = x29 * lh_qi;
	const GEN_FLT x40 = (x31 * (x20 + (x22 * x16))) + ((x39 + x38) * x25) + ((x35 + x32) * x13) + lh_px;
	const GEN_FLT x41 = pow(x40, 2);
	const GEN_FLT x42 = x41 + pow(x36, 2);
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = x43 * x36;
	const GEN_FLT x45 = -1 * x28;
	const GEN_FLT x46 = lh_py + ((x39 + x37) * x31) + (x25 * (x20 + (x22 * x15))) + ((x30 + x45) * x13);
	const GEN_FLT x47 = x40 * x46;
	const GEN_FLT x48 = 0.523598775598299 + tilt_0;
	const GEN_FLT x49 = tan(x48);
	const GEN_FLT x50 = pow(x42, -3.0 / 2.0) * x49;
	const GEN_FLT x51 = x50 * x47;
	const GEN_FLT x52 = 2 * x40;
	const GEN_FLT x53 = pow(x46, 2);
	const GEN_FLT x54 = cos(x48);
	const GEN_FLT x55 = x42 + x53;
	const GEN_FLT x56 = pow((1 + (-1 * pow(x54, -2) * x53 * pow(x55, -1))), -1.0 / 2.0);
	const GEN_FLT x57 = pow(x54, -1);
	const GEN_FLT x58 = x57 * pow(x55, -3.0 / 2.0);
	const GEN_FLT x59 = x58 * x46;
	const GEN_FLT x60 = pow(x42, -1.0 / 2.0) * x49;
	const GEN_FLT x61 = x60 * x46;
	const GEN_FLT x62 = atan2(-1 * x36, x40);
	const GEN_FLT x63 = x62 + (-1 * asin(x61)) + ogeeMag_0;
	const GEN_FLT x64 = (sin(x63) * ogeePhase_0) + curve_0;
	const GEN_FLT x65 = x57 * pow(x55, -1.0 / 2.0);
	const GEN_FLT x66 = asin(x65 * x46);
	const GEN_FLT x67 = 8.0108022e-06 * x66;
	const GEN_FLT x68 = -8.0108022e-06 + (-1 * x67);
	const GEN_FLT x69 = 0.0028679863 + (x68 * x66);
	const GEN_FLT x70 = 5.3685255e-06 + (x66 * x69);
	const GEN_FLT x71 = 0.0076069798 + (x70 * x66);
	const GEN_FLT x72 = x71 * x66;
	const GEN_FLT x73 = -8.0108022e-06 + (-1.60216044e-05 * x66);
	const GEN_FLT x74 = x69 + (x73 * x66);
	const GEN_FLT x75 = x70 + (x74 * x66);
	const GEN_FLT x76 = x71 + (x75 * x66);
	const GEN_FLT x77 = (x76 * x66) + x72;
	const GEN_FLT x78 = sin(x48);
	const GEN_FLT x79 = x78 * x64;
	const GEN_FLT x80 = x54 + (-1 * x79 * x77);
	const GEN_FLT x81 = pow(x80, -1);
	const GEN_FLT x82 = x81 * x72 * x64;
	const GEN_FLT x83 = x82 * x56 * x59;
	const GEN_FLT x84 = x58 * x47;
	const GEN_FLT x85 = x84 * x56;
	const GEN_FLT x86 = -1 * x85 * x68;
	const GEN_FLT x87 = 2.40324066e-05 * x66;
	const GEN_FLT x88 = x69 * x56;
	const GEN_FLT x89 = (x66 * (x86 + (x85 * x67))) + (-1 * x88 * x84);
	const GEN_FLT x90 = (x89 * x66) + (-1 * x85 * x70);
	const GEN_FLT x91 = pow((1 + (-1 * x53 * x43 * pow(x49, 2))), -1.0 / 2.0);
	const GEN_FLT x92 = (x51 * x91) + x44;
	const GEN_FLT x93 = cos(x63) * ogeePhase_0;
	const GEN_FLT x94 = x78 * x77;
	const GEN_FLT x95 = x93 * x94;
	const GEN_FLT x96 = pow(x66, 2);
	const GEN_FLT x97 = x64 * x96;
	const GEN_FLT x98 = pow(x80, -2) * x71 * x97;
	const GEN_FLT x99 = x81 * x71 * x96;
	const GEN_FLT x100 = x93 * x99;
	const GEN_FLT x101 = x81 * x97;
	const GEN_FLT x102 = x61 + (x71 * x101);
	const GEN_FLT x103 = pow((1 + (-1 * pow(x102, 2))), -1.0 / 2.0);
	const GEN_FLT x104 =
		x103 * ((x90 * x101) + (x92 * x100) +
				(-1 * x98 *
				 ((-1 * x92 * x95) +
				  (-1 * x79 *
				   ((x66 * x90) + (-1 * x85 * x71) + (-1 * x85 * x76) +
					(x66 * (x90 + (x66 * (x89 + (-1 * x85 * x74) + (x66 * ((-1 * x85 * x73) + (x85 * x87) + x86)))) +
							(-1 * x85 * x75))))))) +
				(-1 * x83 * x52) + (-1 * x51));
	const GEN_FLT x105 = cos(x62 + (-1 * asin(x102)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x106 = x56 * (x65 + (-1 * x53 * x58));
	const GEN_FLT x107 = 2 * x82;
	const GEN_FLT x108 = x68 * x106;
	const GEN_FLT x109 = (x66 * (x108 + (-1 * x67 * x106))) + (x69 * x106);
	const GEN_FLT x110 = (x66 * x109) + (x70 * x106);
	const GEN_FLT x111 = x60 * x91;
	const GEN_FLT x112 =
		x103 * ((-1 * x100 * x111) +
				(-1 * x98 *
				 ((x95 * x111) +
				  (-1 * x79 *
				   ((x66 * x110) + (x71 * x106) + (x76 * x106) +
					(x66 * (x110 + (x66 * (x109 + (x74 * x106) + (x66 * ((x73 * x106) + (-1 * x87 * x106) + x108)))) +
							(x75 * x106))))))) +
				(x101 * x110) + (x107 * x106) + x60);
	const GEN_FLT x113 = x40 * x43;
	const GEN_FLT x114 = -1 * x113;
	const GEN_FLT x115 = x50 * x46;
	const GEN_FLT x116 = x36 * x115;
	const GEN_FLT x117 = 2 * x36;
	const GEN_FLT x118 = x59 * x36;
	const GEN_FLT x119 = x56 * x118;
	const GEN_FLT x120 = -1 * x68 * x119;
	const GEN_FLT x121 = (x66 * (x120 + (x67 * x119))) + (-1 * x88 * x118);
	const GEN_FLT x122 = (x66 * x121) + (-1 * x70 * x119);
	const GEN_FLT x123 = (x91 * x116) + x114;
	const GEN_FLT x124 =
		x103 *
		((x100 * x123) + (x101 * x122) + (-1 * x83 * x117) +
		 (-1 * x98 *
		  ((-1 * x95 * x123) +
		   (-1 * x79 *
			((x66 * x122) + (-1 * x71 * x119) +
			 (x66 * (x122 + (x66 * (x121 + (-1 * x74 * x119) + (x66 * ((-1 * x73 * x119) + (x87 * x119) + x120)))) +
					 (-1 * x75 * x119))) +
			 (-1 * x76 * x119))))) +
		 (-1 * x116));
	const GEN_FLT x125 = x22 * lh_qk;
	const GEN_FLT x126 = x26 * pow(x17, -3.0 / 2.0);
	const GEN_FLT x127 = x16 * x126;
	const GEN_FLT x128 = 2 * x21 * pow(x17, -2);
	const GEN_FLT x129 = x128 * lh_qk;
	const GEN_FLT x130 = (-1 * x16 * x129) + (x127 * lh_qk);
	const GEN_FLT x131 = x130 + x125;
	const GEN_FLT x132 = lh_qj * lh_qi;
	const GEN_FLT x133 = x126 * x132;
	const GEN_FLT x134 = x20 * x18;
	const GEN_FLT x135 = x134 * x132;
	const GEN_FLT x136 = (-1 * x135) + x133;
	const GEN_FLT x137 = x16 * x134;
	const GEN_FLT x138 = x126 * lh_qk;
	const GEN_FLT x139 = (-1 * x129 * x132) + (x132 * x138);
	const GEN_FLT x140 = x139 + x27;
	const GEN_FLT x141 = x14 * x126;
	const GEN_FLT x142 = x128 * lh_qi;
	const GEN_FLT x143 = (-1 * x14 * x142) + (x141 * lh_qi);
	const GEN_FLT x144 = (x13 * (x143 + x45)) + (x25 * (x140 + x137 + (-1 * x127))) + ((x136 + x131) * x31);
	const GEN_FLT x145 = pow(x40, -1);
	const GEN_FLT x146 = pow(lh_qi, 3);
	const GEN_FLT x147 = x128 * lh_qj;
	const GEN_FLT x148 = (-1 * x16 * x147) + (x127 * lh_qj);
	const GEN_FLT x149 = x148 + x29;
	const GEN_FLT x150 = x138 * lh_qi;
	const GEN_FLT x151 = x134 * lh_qk;
	const GEN_FLT x152 = x151 * lh_qi;
	const GEN_FLT x153 = (-1 * x152) + x150;
	const GEN_FLT x154 = x135 + (-1 * x133);
	const GEN_FLT x155 =
		((x154 + x131) * x13) + ((x153 + x149) * x25) + (((-1 * x128 * x146) + (x126 * x146) + (2 * x34) + x45) * x31);
	const GEN_FLT x156 = pow(x41, -1) * x36;
	const GEN_FLT x157 = x41 * x43;
	const GEN_FLT x158 = ((x155 * x156) + (-1 * x144 * x145)) * x157;
	const GEN_FLT x159 = x152 + (-1 * x150);
	const GEN_FLT x160 = x15 * x126;
	const GEN_FLT x161 = (-1 * x15 * x142) + (x160 * lh_qi);
	const GEN_FLT x162 = x139 + (-1 * x27);
	const GEN_FLT x163 = (x13 * (x162 + (-1 * x137) + x127)) + (x25 * (x161 + x45)) + ((x159 + x149) * x31);
	const GEN_FLT x164 = 2 * x46;
	const GEN_FLT x165 = (x117 * x144) + (x52 * x155);
	const GEN_FLT x166 = 1.0 / 2.0 * x59;
	const GEN_FLT x167 = x56 * ((x65 * x163) + (-1 * x166 * (x165 + (x164 * x163))));
	const GEN_FLT x168 = 1.0 / 2.0 * x115;
	const GEN_FLT x169 = (x60 * x163) + (-1 * x168 * x165);
	const GEN_FLT x170 = (-1 * x91 * x169) + x158;
	const GEN_FLT x171 = x68 * x167;
	const GEN_FLT x172 = (x66 * (x171 + (-1 * x67 * x167))) + (x69 * x167);
	const GEN_FLT x173 = (x66 * x172) + (x70 * x167);
	const GEN_FLT x174 =
		x103 * (x169 + (x101 * x173) +
				(-1 * x98 *
				 ((-1 * x95 * x170) +
				  (-1 * x79 *
				   ((x66 * x173) +
					(x66 * (x173 + (x66 * (x172 + (x74 * x167) + (x66 * ((x73 * x167) + (-1 * x87 * x167) + x171)))) +
							(x75 * x167))) +
					(x71 * x167) + (x76 * x167))))) +
				(x100 * x170) + (x107 * x167));
	const GEN_FLT x175 = x15 * x134;
	const GEN_FLT x176 = (-1 * x15 * x129) + (x160 * lh_qk);
	const GEN_FLT x177 = x176 + x125;
	const GEN_FLT x178 = (-1 * x14 * x147) + (x141 * lh_qj);
	const GEN_FLT x179 = (x13 * (x178 + x33)) + ((x177 + x154) * x25) + (x31 * (x162 + (-1 * x175) + x160));
	const GEN_FLT x180 = x161 + x34;
	const GEN_FLT x181 = x138 * lh_qj;
	const GEN_FLT x182 = x151 * lh_qj;
	const GEN_FLT x183 = (-1 * x182) + x181;
	const GEN_FLT x184 = (x13 * (x140 + x175 + (-1 * x160))) + ((x183 + x180) * x25) + (x31 * (x148 + x33));
	const GEN_FLT x185 = ((x184 * x156) + (-1 * x179 * x145)) * x157;
	const GEN_FLT x186 = pow(lh_qj, 3);
	const GEN_FLT x187 = x182 + (-1 * x181);
	const GEN_FLT x188 =
		((x177 + x136) * x13) + ((x187 + x180) * x31) + (((-1 * x128 * x186) + (x126 * x186) + (2 * x29) + x33) * x25);
	const GEN_FLT x189 = (x117 * x179) + (x52 * x184);
	const GEN_FLT x190 = x56 * ((x65 * x188) + (-1 * x166 * (x189 + (x164 * x188))));
	const GEN_FLT x191 = (x60 * x188) + (-1 * x168 * x189);
	const GEN_FLT x192 = (-1 * x91 * x191) + x185;
	const GEN_FLT x193 = x68 * x190;
	const GEN_FLT x194 = (x66 * (x193 + (-1 * x67 * x190))) + (x69 * x190);
	const GEN_FLT x195 = (x66 * x194) + (x70 * x190);
	const GEN_FLT x196 =
		x103 * (x191 + (x101 * x195) +
				(-1 * x98 *
				 ((-1 * x95 * x192) +
				  (-1 * x79 *
				   ((x66 * (x195 + (x66 * (x194 + (x74 * x190) + (x66 * ((x73 * x190) + (-1 * x87 * x190) + x193)))) +
							(x75 * x190))) +
					(x66 * x195) + (x71 * x190) + (x76 * x190))))) +
				(x100 * x192) + (x107 * x190));
	const GEN_FLT x197 = x14 * x134;
	const GEN_FLT x198 = x178 + x29;
	const GEN_FLT x199 = (x25 * (x176 + x38)) + ((x198 + x153) * x13) + (x31 * (x140 + x197 + (-1 * x141)));
	const GEN_FLT x200 = x143 + x34;
	const GEN_FLT x201 = (x25 * (x162 + (-1 * x197) + x141)) + ((x200 + x187) * x13) + (x31 * (x130 + x38));
	const GEN_FLT x202 = pow(lh_qk, 3);
	const GEN_FLT x203 =
		(x13 * ((x202 * x126) + (-1 * x202 * x128) + (2 * x125) + x38)) + ((x198 + x159) * x25) + ((x200 + x183) * x31);
	const GEN_FLT x204 = (x203 * x117) + (x52 * x201);
	const GEN_FLT x205 = x56 * ((x65 * x199) + (-1 * x166 * (x204 + (x164 * x199))));
	const GEN_FLT x206 = x68 * x205;
	const GEN_FLT x207 = (x66 * (x206 + (-1 * x67 * x205))) + (x69 * x205);
	const GEN_FLT x208 = (x66 * x207) + (x70 * x205);
	const GEN_FLT x209 = ((x201 * x156) + (-1 * x203 * x145)) * x157;
	const GEN_FLT x210 = (x60 * x199) + (-1 * x204 * x168);
	const GEN_FLT x211 = x93 * ((-1 * x91 * x210) + x209);
	const GEN_FLT x212 =
		x103 * (x210 + (x208 * x101) + (x99 * x211) +
				(-1 * x98 *
				 ((-1 * x94 * x211) +
				  (-1 * x79 *
				   ((x66 * x208) +
					(x66 * (x208 + (x66 * (x207 + (x74 * x205) + (x66 * ((x73 * x205) + (-1 * x87 * x205) + x206)))) +
							(x75 * x205))) +
					(x71 * x205) + (x76 * x205))))) +
				(x205 * x107));
	out[0] = (-1 * x105 * (x104 + (-1 * x44))) + (-1 * x104) + x44;
	out[1] = (-1 * x105 * x112) + (-1 * x112);
	out[2] = (-1 * (x124 + x113) * x105) + (-1 * x124) + x114;
	out[3] = (-1 * (x174 + (-1 * x158)) * x105) + (-1 * x174) + x158;
	out[4] = (-1 * (x196 + (-1 * x185)) * x105) + (-1 * x196) + x185;
	out[5] = (-1 * (x212 + (-1 * x209)) * x105) + x209 + (-1 * x212);
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
	const GEN_FLT x0 = 0.523598775598299 + tilt_0;
	const GEN_FLT x1 = tan(x0);
	const GEN_FLT x2 = pow(lh_qk, 2);
	const GEN_FLT x3 = pow(lh_qj, 2);
	const GEN_FLT x4 = pow(lh_qi, 2);
	const GEN_FLT x5 = 1e-10 + x4 + x3 + x2;
	const GEN_FLT x6 = pow(x5, 1.0 / 2.0);
	const GEN_FLT x7 = pow(x6, -1) * sin(x6);
	const GEN_FLT x8 = x7 * lh_qi;
	const GEN_FLT x9 = cos(x6);
	const GEN_FLT x10 = pow(x5, -1) * (1 + (-1 * x9));
	const GEN_FLT x11 = x10 * lh_qj;
	const GEN_FLT x12 = x11 * lh_qk;
	const GEN_FLT x13 = pow(obj_qk, 2);
	const GEN_FLT x14 = pow(obj_qj, 2);
	const GEN_FLT x15 = pow(obj_qi, 2);
	const GEN_FLT x16 = 1e-10 + x15 + x14 + x13;
	const GEN_FLT x17 = pow(x16, 1.0 / 2.0);
	const GEN_FLT x18 = cos(x17);
	const GEN_FLT x19 = pow(x16, -1) * (1 + (-1 * x18));
	const GEN_FLT x20 = pow(x17, -1) * sin(x17);
	const GEN_FLT x21 = x20 * obj_qi;
	const GEN_FLT x22 = x19 * obj_qk;
	const GEN_FLT x23 = x22 * obj_qj;
	const GEN_FLT x24 = x20 * obj_qj;
	const GEN_FLT x25 = x22 * obj_qi;
	const GEN_FLT x26 =
		((x25 + (-1 * x24)) * sensor_x) + ((x23 + x21) * sensor_y) + ((x18 + (x13 * x19)) * sensor_z) + obj_pz;
	const GEN_FLT x27 = x20 * obj_qk;
	const GEN_FLT x28 = x19 * obj_qj * obj_qi;
	const GEN_FLT x29 =
		((x28 + x27) * sensor_x) + ((x23 + (-1 * x21)) * sensor_z) + ((x18 + (x14 * x19)) * sensor_y) + obj_py;
	const GEN_FLT x30 =
		((x18 + (x15 * x19)) * sensor_x) + ((x28 + (-1 * x27)) * sensor_y) + ((x25 + x24) * sensor_z) + obj_px;
	const GEN_FLT x31 = x7 * lh_qk;
	const GEN_FLT x32 = x11 * lh_qi;
	const GEN_FLT x33 = ((x32 + x31) * x30) + (x29 * (x9 + (x3 * x10))) + lh_py + (x26 * (x12 + (-1 * x8)));
	const GEN_FLT x34 = x7 * lh_qj;
	const GEN_FLT x35 = x10 * lh_qk * lh_qi;
	const GEN_FLT x36 = ((x35 + (-1 * x34)) * x30) + (x29 * (x12 + x8)) + (x26 * (x9 + (x2 * x10))) + lh_pz;
	const GEN_FLT x37 = (x30 * (x9 + (x4 * x10))) + ((x32 + (-1 * x31)) * x29) + ((x35 + x34) * x26) + lh_px;
	const GEN_FLT x38 = pow(x37, 2) + pow(x36, 2);
	const GEN_FLT x39 = x33 * pow(x38, -1.0 / 2.0);
	const GEN_FLT x40 = x1 * x39;
	const GEN_FLT x41 = atan2(-1 * x36, x37);
	const GEN_FLT x42 = x41 + (-1 * asin(x40)) + ogeeMag_0;
	const GEN_FLT x43 = sin(x42);
	const GEN_FLT x44 = (x43 * ogeePhase_0) + curve_0;
	const GEN_FLT x45 = cos(x0);
	const GEN_FLT x46 = pow(x33, 2);
	const GEN_FLT x47 = x38 + x46;
	const GEN_FLT x48 = pow(x47, -1.0 / 2.0) * x33;
	const GEN_FLT x49 = asin(pow(x45, -1) * x48);
	const GEN_FLT x50 = 8.0108022e-06 * x49;
	const GEN_FLT x51 = -8.0108022e-06 + (-1 * x50);
	const GEN_FLT x52 = 0.0028679863 + (x51 * x49);
	const GEN_FLT x53 = 5.3685255e-06 + (x52 * x49);
	const GEN_FLT x54 = 0.0076069798 + (x53 * x49);
	const GEN_FLT x55 = pow(x49, 2);
	const GEN_FLT x56 = sin(x0);
	const GEN_FLT x57 = x54 * x49;
	const GEN_FLT x58 = -8.0108022e-06 + (-1.60216044e-05 * x49);
	const GEN_FLT x59 = x52 + (x58 * x49);
	const GEN_FLT x60 = x53 + (x59 * x49);
	const GEN_FLT x61 = x54 + (x60 * x49);
	const GEN_FLT x62 = (x61 * x49) + x57;
	const GEN_FLT x63 = x62 * x44;
	const GEN_FLT x64 = x63 * x56;
	const GEN_FLT x65 = x45 + (-1 * x64);
	const GEN_FLT x66 = pow(x65, -1);
	const GEN_FLT x67 = x66 * x55;
	const GEN_FLT x68 = x67 * x54;
	const GEN_FLT x69 = x40 + (x68 * x44);
	const GEN_FLT x70 = pow((1 + (-1 * pow(x69, 2))), -1.0 / 2.0);
	const GEN_FLT x71 = x56 * x44;
	const GEN_FLT x72 = pow(x45, -2);
	const GEN_FLT x73 = x72 * x48 * pow((1 + (-1 * x72 * x46 * pow(x47, -1))), -1.0 / 2.0);
	const GEN_FLT x74 = pow(x1, 2);
	const GEN_FLT x75 = x39 * (1 + x74);
	const GEN_FLT x76 = x73 * x56;
	const GEN_FLT x77 = x76 * x51;
	const GEN_FLT x78 = (x49 * (x77 + (-1 * x76 * x50))) + (x76 * x52);
	const GEN_FLT x79 = (x78 * x49) + (x76 * x53);
	const GEN_FLT x80 = cos(x42) * ogeePhase_0;
	const GEN_FLT x81 = x75 * pow((1 + (-1 * x74 * x46 * pow(x38, -1))), -1.0 / 2.0);
	const GEN_FLT x82 = pow(x65, -2) * x54 * x55;
	const GEN_FLT x83 = x80 * x68;
	const GEN_FLT x84 =
		x70 *
		((x79 * x67 * x44) + (-1 * x81 * x83) +
		 (-1 * x82 * x44 *
		  ((x80 * x81 * x62 * x56) +
		   (-1 * x71 *
			((x79 * x49) + (x76 * x54) +
			 (x49 * (x79 + (x49 * (x78 + (x76 * x59) + (x49 * ((x76 * x58) + (-2.40324066e-05 * x76 * x49) + x77)))) +
					 (x76 * x60))) +
			 (x76 * x61))) +
		   (-1 * x63 * x45) + (-1 * x56))) +
		 x75 + (2 * x71 * x73 * x66 * x57));
	const GEN_FLT x85 = (-1 * x41) + asin(x69) + (-1 * gibPhase_0);
	const GEN_FLT x86 = cos(x85) * gibMag_0;
	const GEN_FLT x87 = x82 * x64;
	const GEN_FLT x88 = (x68 + x87) * x70;
	const GEN_FLT x89 = x70 * (x83 + (x80 * x87));
	const GEN_FLT x90 = ((x68 * x43) + (x87 * x43)) * x70;
	out[0] = -1;
	out[1] = (-1 * x84 * x86) + (-1 * x84);
	out[2] = (-1 * x88 * x86) + (-1 * x88);
	out[3] = x86;
	out[4] = -1 * sin(x85);
	out[5] = (-1 * x89 * x86) + (-1 * x89);
	out[6] = (-1 * x86 * x90) + (-1 * x90);
}

static inline FLT gen_reproject_axis_y_gen2_axis_angle(const LinmathAxisAnglePose *obj_p, const FLT *sensor_pt,
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qi;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qj;
	const GEN_FLT x10 = x9 * lh_qk;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = pow(obj_qj, 2);
	const GEN_FLT x13 = pow(obj_qi, 2);
	const GEN_FLT x14 = 1e-10 + x13 + x12 + x11;
	const GEN_FLT x15 = pow(x14, 1.0 / 2.0);
	const GEN_FLT x16 = cos(x15);
	const GEN_FLT x17 = pow(x14, -1) * (1 + (-1 * x16));
	const GEN_FLT x18 = pow(x15, -1) * sin(x15);
	const GEN_FLT x19 = x18 * obj_qi;
	const GEN_FLT x20 = x17 * obj_qk;
	const GEN_FLT x21 = x20 * obj_qj;
	const GEN_FLT x22 = x18 * obj_qj;
	const GEN_FLT x23 = x20 * obj_qi;
	const GEN_FLT x24 =
		((x21 + x19) * sensor_y) + ((x23 + (-1 * x22)) * sensor_x) + ((x16 + (x11 * x17)) * sensor_z) + obj_pz;
	const GEN_FLT x25 = x18 * obj_qk;
	const GEN_FLT x26 = x17 * obj_qj * obj_qi;
	const GEN_FLT x27 =
		((x26 + x25) * sensor_x) + ((x16 + (x12 * x17)) * sensor_y) + ((x21 + (-1 * x19)) * sensor_z) + obj_py;
	const GEN_FLT x28 =
		((x26 + (-1 * x25)) * sensor_y) + ((x23 + x22) * sensor_z) + ((x16 + (x13 * x17)) * sensor_x) + obj_px;
	const GEN_FLT x29 = x5 * lh_qk;
	const GEN_FLT x30 = x9 * lh_qi;
	const GEN_FLT x31 = lh_py + ((x30 + x29) * x28) + (x27 * (x7 + (x1 * x8))) + (x24 * (x10 + (-1 * x6)));
	const GEN_FLT x32 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x33 = x5 * lh_qj;
	const GEN_FLT x34 = x8 * lh_qk * lh_qi;
	const GEN_FLT x35 = ((x34 + (-1 * x33)) * x28) + (x27 * (x10 + x6)) + (x24 * (x7 + (x0 * x8))) + lh_pz;
	const GEN_FLT x36 = ((x30 + (-1 * x29)) * x27) + ((x34 + x33) * x24) + (x28 * (x7 + (x2 * x8))) + lh_px;
	const GEN_FLT x37 = pow(x36, 2) + pow(x35, 2);
	const GEN_FLT x38 = -1 * x31 * pow(x37, -1.0 / 2.0) * tan(x32);
	const GEN_FLT x39 = atan2(-1 * x35, x36);
	const GEN_FLT x40 = (sin(x39 + ogeeMag_1 + (-1 * asin(x38))) * ogeePhase_1) + curve_1;
	const GEN_FLT x41 = cos(x32);
	const GEN_FLT x42 = asin(pow(x41, -1) * x31 * pow((x37 + pow(x31, 2)), -1.0 / 2.0));
	const GEN_FLT x43 = 0.0028679863 + (x42 * (-8.0108022e-06 + (-8.0108022e-06 * x42)));
	const GEN_FLT x44 = 5.3685255e-06 + (x42 * x43);
	const GEN_FLT x45 = 0.0076069798 + (x42 * x44);
	const GEN_FLT x46 = asin(
		x38 +
		(x40 * pow(x42, 2) * x45 *
		 pow((x41 +
			  (x40 * sin(x32) *
			   ((x42 * (x45 + (x42 * (x44 + (x42 * (x43 + (x42 * (-8.0108022e-06 + (-1.60216044e-05 * x42))))))))) +
				(x42 * x45)))),
			 -1)));
	return -1.5707963267949 + x39 + (-1 * sin((-1 * x39) + x46 + (-1 * gibPhase_1)) * gibMag_1) + (-1 * x46) +
		   (-1 * phase_1);
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qj;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qi;
	const GEN_FLT x10 = x9 * lh_qk;
	const GEN_FLT x11 = x10 + (-1 * x6);
	const GEN_FLT x12 = x10 + x6;
	const GEN_FLT x13 = pow(obj_qk, 2);
	const GEN_FLT x14 = pow(obj_qj, 2);
	const GEN_FLT x15 = pow(obj_qi, 2);
	const GEN_FLT x16 = 1e-10 + x15 + x14 + x13;
	const GEN_FLT x17 = pow(x16, -1);
	const GEN_FLT x18 = pow(x16, 1.0 / 2.0);
	const GEN_FLT x19 = cos(x18);
	const GEN_FLT x20 = 1 + (-1 * x19);
	const GEN_FLT x21 = x20 * x17;
	const GEN_FLT x22 = sin(x18);
	const GEN_FLT x23 = x22 * pow(x18, -1);
	const GEN_FLT x24 = x23 * obj_qi;
	const GEN_FLT x25 = x21 * obj_qj;
	const GEN_FLT x26 = x25 * obj_qk;
	const GEN_FLT x27 = x23 * obj_qj;
	const GEN_FLT x28 = -1 * x27;
	const GEN_FLT x29 = x21 * obj_qi;
	const GEN_FLT x30 = x29 * obj_qk;
	const GEN_FLT x31 = ((x30 + x28) * sensor_x) + ((x26 + x24) * sensor_y) + ((x19 + (x21 * x13)) * sensor_z) + obj_pz;
	const GEN_FLT x32 = -1 * x24;
	const GEN_FLT x33 = x23 * obj_qk;
	const GEN_FLT x34 = x25 * obj_qi;
	const GEN_FLT x35 = ((x34 + x33) * sensor_x) + ((x19 + (x21 * x14)) * sensor_y) + ((x26 + x32) * sensor_z) + obj_py;
	const GEN_FLT x36 = x5 * lh_qk;
	const GEN_FLT x37 = x9 * lh_qj;
	const GEN_FLT x38 = x37 + (-1 * x36);
	const GEN_FLT x39 = -1 * x33;
	const GEN_FLT x40 = ((x34 + x39) * sensor_y) + ((x19 + (x21 * x15)) * sensor_x) + ((x30 + x27) * sensor_z) + obj_px;
	const GEN_FLT x41 = x7 + (x2 * x8);
	const GEN_FLT x42 = (x40 * x41) + (x31 * x12) + (x35 * x38) + lh_px;
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = pow(x42, 2);
	const GEN_FLT x45 = x7 + (x0 * x8);
	const GEN_FLT x46 = x5 * lh_qi;
	const GEN_FLT x47 = x8 * lh_qk * lh_qj;
	const GEN_FLT x48 = x47 + x46;
	const GEN_FLT x49 = (x40 * x11) + (x45 * x31) + (x48 * x35) + lh_pz;
	const GEN_FLT x50 = pow(x44, -1) * x49;
	const GEN_FLT x51 = x44 + pow(x49, 2);
	const GEN_FLT x52 = pow(x51, -1);
	const GEN_FLT x53 = x52 * x44;
	const GEN_FLT x54 = ((x50 * x41) + (-1 * x43 * x11)) * x53;
	const GEN_FLT x55 = x47 + (-1 * x46);
	const GEN_FLT x56 = x7 + (x1 * x8);
	const GEN_FLT x57 = x37 + x36;
	const GEN_FLT x58 = (x57 * x40) + lh_py + (x56 * x35) + (x55 * x31);
	const GEN_FLT x59 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x60 = cos(x59);
	const GEN_FLT x61 = pow(x60, -1);
	const GEN_FLT x62 = pow(x58, 2);
	const GEN_FLT x63 = x51 + x62;
	const GEN_FLT x64 = pow(x63, -1.0 / 2.0) * x61;
	const GEN_FLT x65 = asin(x64 * x58);
	const GEN_FLT x66 = 8.0108022e-06 * x65;
	const GEN_FLT x67 = -8.0108022e-06 + (-1 * x66);
	const GEN_FLT x68 = 0.0028679863 + (x67 * x65);
	const GEN_FLT x69 = 5.3685255e-06 + (x68 * x65);
	const GEN_FLT x70 = 0.0076069798 + (x65 * x69);
	const GEN_FLT x71 = pow(x65, 2);
	const GEN_FLT x72 = tan(x59);
	const GEN_FLT x73 = x72 * pow(x51, -1.0 / 2.0);
	const GEN_FLT x74 = -1 * x73 * x58;
	const GEN_FLT x75 = atan2(-1 * x49, x42);
	const GEN_FLT x76 = x75 + ogeeMag_1 + (-1 * asin(x74));
	const GEN_FLT x77 = (sin(x76) * ogeePhase_1) + curve_1;
	const GEN_FLT x78 = x70 * x65;
	const GEN_FLT x79 = -8.0108022e-06 + (-1.60216044e-05 * x65);
	const GEN_FLT x80 = x68 + (x79 * x65);
	const GEN_FLT x81 = x69 + (x80 * x65);
	const GEN_FLT x82 = x70 + (x81 * x65);
	const GEN_FLT x83 = (x82 * x65) + x78;
	const GEN_FLT x84 = sin(x59);
	const GEN_FLT x85 = x84 * x77;
	const GEN_FLT x86 = x60 + (x83 * x85);
	const GEN_FLT x87 = pow(x86, -1);
	const GEN_FLT x88 = x87 * x77;
	const GEN_FLT x89 = x88 * x71;
	const GEN_FLT x90 = x74 + (x89 * x70);
	const GEN_FLT x91 = pow((1 + (-1 * pow(x90, 2))), -1.0 / 2.0);
	const GEN_FLT x92 = pow((1 + (-1 * pow(x72, 2) * x62 * x52)), -1.0 / 2.0);
	const GEN_FLT x93 = 2 * x42;
	const GEN_FLT x94 = 2 * x49;
	const GEN_FLT x95 = (x94 * x11) + (x93 * x41);
	const GEN_FLT x96 = 1.0 / 2.0 * x58;
	const GEN_FLT x97 = x72 * pow(x51, -3.0 / 2.0) * x96;
	const GEN_FLT x98 = (-1 * x73 * x57) + (x97 * x95);
	const GEN_FLT x99 = (-1 * x92 * x98) + x54;
	const GEN_FLT x100 = cos(x76) * ogeePhase_1;
	const GEN_FLT x101 = x71 * x70;
	const GEN_FLT x102 = x87 * x101;
	const GEN_FLT x103 = x100 * x102;
	const GEN_FLT x104 = pow((1 + (-1 * pow(x60, -2) * pow(x63, -1) * x62)), -1.0 / 2.0);
	const GEN_FLT x105 = 2 * x58;
	const GEN_FLT x106 = pow(x63, -3.0 / 2.0) * x61 * x96;
	const GEN_FLT x107 = (x64 * x57) + (-1 * x106 * (x95 + (x57 * x105)));
	const GEN_FLT x108 = x104 * x107;
	const GEN_FLT x109 = 2 * x88 * x78;
	const GEN_FLT x110 = x67 * x108;
	const GEN_FLT x111 = (x68 * x108) + (x65 * (x110 + (-1 * x66 * x108)));
	const GEN_FLT x112 = (x69 * x108) + (x65 * x111);
	const GEN_FLT x113 = x83 * x84;
	const GEN_FLT x114 = x100 * x113;
	const GEN_FLT x115 = x82 * x104;
	const GEN_FLT x116 = 2.40324066e-05 * x65;
	const GEN_FLT x117 = pow(x86, -2) * x77 * x101;
	const GEN_FLT x118 =
		x91 * (x98 +
			   (-1 * x117 *
				((x85 * ((x65 * x112) + (x70 * x108) +
						 (x65 * ((x65 * (x111 + (x80 * x108) + (x65 * ((x79 * x108) + (-1 * x108 * x116) + x110)))) +
								 x112 + (x81 * x108))) +
						 (x107 * x115))) +
				 (x99 * x114))) +
			   (x89 * x112) + (x109 * x108) + (x99 * x103));
	const GEN_FLT x119 = cos(x75 + (-1 * asin(x90)) + gibPhase_1) * gibMag_1;
	const GEN_FLT x120 = ((x50 * x38) + (-1 * x43 * x48)) * x53;
	const GEN_FLT x121 = (x94 * x48) + (x93 * x38);
	const GEN_FLT x122 = (-1 * x73 * x56) + (x97 * x121);
	const GEN_FLT x123 = x100 * ((-1 * x92 * x122) + x120);
	const GEN_FLT x124 = (x64 * x56) + (-1 * x106 * (x121 + (x56 * x105)));
	const GEN_FLT x125 = x104 * x124;
	const GEN_FLT x126 = x67 * x125;
	const GEN_FLT x127 = (x68 * x125) + (x65 * (x126 + (-1 * x66 * x125)));
	const GEN_FLT x128 = (x69 * x125) + (x65 * x127);
	const GEN_FLT x129 =
		x91 * ((-1 * x117 *
				((x85 * ((x65 * x128) + (x70 * x125) +
						 (x65 * ((x65 * (x127 + (x80 * x125) + (x65 * ((x79 * x125) + (-1 * x116 * x125) + x126)))) +
								 x128 + (x81 * x125))) +
						 (x115 * x124))) +
				 (x113 * x123))) +
			   (x109 * x125) + (x89 * x128) + x122 + (x102 * x123));
	const GEN_FLT x130 = ((x50 * x12) + (-1 * x43 * x45)) * x53;
	const GEN_FLT x131 = (x94 * x45) + (x93 * x12);
	const GEN_FLT x132 = (-1 * x73 * x55) + (x97 * x131);
	const GEN_FLT x133 = (-1 * x92 * x132) + x130;
	const GEN_FLT x134 = (x64 * x55) + (-1 * x106 * (x131 + (x55 * x105)));
	const GEN_FLT x135 = x104 * x134;
	const GEN_FLT x136 = x67 * x135;
	const GEN_FLT x137 = (x68 * x135) + (x65 * (x136 + (-1 * x66 * x135)));
	const GEN_FLT x138 = (x69 * x135) + (x65 * x137);
	const GEN_FLT x139 =
		x91 *
		(x132 + (x89 * x138) + (x109 * x135) +
		 (-1 * x117 *
		  ((x85 * ((x65 * x138) + (x70 * x135) +
				   (x65 * (x138 + (x65 * (x137 + (x80 * x135) + (x65 * ((x79 * x135) + (-1 * x116 * x135) + x136)))) +
						   (x81 * x135))) +
				   (x115 * x134))) +
		   (x114 * x133))) +
		 (x103 * x133));
	const GEN_FLT x140 = pow(obj_qi, 3);
	const GEN_FLT x141 = x22 * pow(x16, -3.0 / 2.0);
	const GEN_FLT x142 = 2 * x20 * pow(x16, -2);
	const GEN_FLT x143 = x19 * x17;
	const GEN_FLT x144 = x143 * obj_qk;
	const GEN_FLT x145 = x144 * obj_qi;
	const GEN_FLT x146 = x141 * obj_qk;
	const GEN_FLT x147 = x146 * obj_qi;
	const GEN_FLT x148 = x147 + (-1 * x145);
	const GEN_FLT x149 = x15 * x141;
	const GEN_FLT x150 = x15 * x142;
	const GEN_FLT x151 = (-1 * x150 * obj_qj) + (x149 * obj_qj);
	const GEN_FLT x152 = x151 + x25;
	const GEN_FLT x153 = obj_qj * obj_qi;
	const GEN_FLT x154 = x143 * x153;
	const GEN_FLT x155 = x141 * x153;
	const GEN_FLT x156 = (-1 * x155) + x154;
	const GEN_FLT x157 = x21 * obj_qk;
	const GEN_FLT x158 = (-1 * x150 * obj_qk) + (x149 * obj_qk);
	const GEN_FLT x159 = x158 + x157;
	const GEN_FLT x160 = ((x159 + x156) * sensor_z) + ((x152 + x148) * sensor_y) +
						 (((-1 * x140 * x142) + (x140 * x141) + (2 * x29) + x32) * sensor_x);
	const GEN_FLT x161 = (-1 * x147) + x145;
	const GEN_FLT x162 = x14 * x141;
	const GEN_FLT x163 = x142 * obj_qi;
	const GEN_FLT x164 = (-1 * x14 * x163) + (x162 * obj_qi);
	const GEN_FLT x165 = x15 * x143;
	const GEN_FLT x166 = x142 * obj_qk;
	const GEN_FLT x167 = (-1 * x166 * x153) + (x155 * obj_qk);
	const GEN_FLT x168 = x167 + (-1 * x23);
	const GEN_FLT x169 =
		((x168 + x149 + (-1 * x165)) * sensor_z) + ((x164 + x32) * sensor_y) + ((x152 + x161) * sensor_x);
	const GEN_FLT x170 = x155 + (-1 * x154);
	const GEN_FLT x171 = x167 + x23;
	const GEN_FLT x172 = x13 * x141;
	const GEN_FLT x173 = (-1 * x13 * x163) + (x172 * obj_qi);
	const GEN_FLT x174 =
		((x173 + x32) * sensor_z) + ((x171 + (-1 * x149) + x165) * sensor_y) + ((x170 + x159) * sensor_x);
	const GEN_FLT x175 = (x45 * x174) + (x48 * x169) + (x11 * x160);
	const GEN_FLT x176 = (x38 * x169) + (x12 * x174) + (x41 * x160);
	const GEN_FLT x177 = ((x50 * x176) + (-1 * x43 * x175)) * x53;
	const GEN_FLT x178 = (x94 * x175) + (x93 * x176);
	const GEN_FLT x179 = (x55 * x174) + (x56 * x169) + (x57 * x160);
	const GEN_FLT x180 = (-1 * x73 * x179) + (x97 * x178);
	const GEN_FLT x181 = (-1 * x92 * x180) + x177;
	const GEN_FLT x182 = (x64 * x179) + (-1 * x106 * (x178 + (x105 * x179)));
	const GEN_FLT x183 = x104 * x182;
	const GEN_FLT x184 = x67 * x183;
	const GEN_FLT x185 = (x68 * x183) + (x65 * (x184 + (-1 * x66 * x183)));
	const GEN_FLT x186 = (x69 * x183) + (x65 * x185);
	const GEN_FLT x187 =
		x91 *
		(x180 + (x89 * x186) + (x109 * x183) +
		 (-1 * x117 *
		  ((x85 * ((x65 * x186) + (x70 * x183) +
				   (x65 * (x186 + (x65 * (x185 + (x80 * x183) + (x65 * ((x79 * x183) + (-1 * x116 * x183) + x184)))) +
						   (x81 * x183))) +
				   (x115 * x182))) +
		   (x114 * x181))) +
		 (x103 * x181));
	const GEN_FLT x188 = x164 + x29;
	const GEN_FLT x189 = x144 * obj_qj;
	const GEN_FLT x190 = x146 * obj_qj;
	const GEN_FLT x191 = (-1 * x190) + x189;
	const GEN_FLT x192 = pow(obj_qj, 3);
	const GEN_FLT x193 = (-1 * x14 * x166) + (x162 * obj_qk);
	const GEN_FLT x194 = x193 + x157;
	const GEN_FLT x195 = ((x170 + x194) * sensor_z) +
						 (((-1 * x192 * x142) + (x192 * x141) + (2 * x25) + x28) * sensor_y) +
						 ((x191 + x188) * sensor_x);
	const GEN_FLT x196 = x190 + (-1 * x189);
	const GEN_FLT x197 = x14 * x143;
	const GEN_FLT x198 =
		((x171 + (-1 * x162) + x197) * sensor_z) + ((x196 + x188) * sensor_y) + ((x151 + x28) * sensor_x);
	const GEN_FLT x199 = (-1 * x13 * x142 * obj_qj) + (x172 * obj_qj);
	const GEN_FLT x200 =
		((x199 + x28) * sensor_z) + ((x156 + x194) * sensor_y) + ((x168 + x162 + (-1 * x197)) * sensor_x);
	const GEN_FLT x201 = (x11 * x198) + (x45 * x200) + (x48 * x195);
	const GEN_FLT x202 = (x12 * x200) + (x38 * x195) + (x41 * x198);
	const GEN_FLT x203 = ((x50 * x202) + (-1 * x43 * x201)) * x53;
	const GEN_FLT x204 = (x94 * x201) + (x93 * x202);
	const GEN_FLT x205 = (x55 * x200) + (x56 * x195) + (x57 * x198);
	const GEN_FLT x206 = (-1 * x73 * x205) + (x97 * x204);
	const GEN_FLT x207 = (-1 * x92 * x206) + x203;
	const GEN_FLT x208 = (x64 * x205) + (-1 * x106 * (x204 + (x205 * x105)));
	const GEN_FLT x209 = x208 * x104;
	const GEN_FLT x210 = x67 * x209;
	const GEN_FLT x211 = (x68 * x209) + (x65 * (x210 + (-1 * x66 * x209)));
	const GEN_FLT x212 = (x69 * x209) + (x65 * x211);
	const GEN_FLT x213 =
		x91 *
		(x206 +
		 (-1 * x117 *
		  ((x85 * ((x65 * x212) + (x70 * x209) +
				   (x65 * (x212 + (x65 * (x211 + (x80 * x209) + (x65 * ((x79 * x209) + (-1 * x209 * x116) + x210)))) +
						   (x81 * x209))) +
				   (x208 * x115))) +
		   (x207 * x114))) +
		 (x89 * x212) + (x209 * x109) + (x207 * x103));
	const GEN_FLT x214 = x13 * x143;
	const GEN_FLT x215 = x173 + x29;
	const GEN_FLT x216 =
		((x215 + x191) * sensor_z) + ((x168 + x172 + (-1 * x214)) * sensor_y) + ((x158 + x39) * sensor_x);
	const GEN_FLT x217 = x199 + x25;
	const GEN_FLT x218 =
		((x148 + x217) * sensor_z) + ((x193 + x39) * sensor_y) + ((x171 + (-1 * x172) + x214) * sensor_x);
	const GEN_FLT x219 = pow(obj_qk, 3);
	const GEN_FLT x220 = (((-1 * x219 * x142) + x39 + (x219 * x141) + (2 * x157)) * sensor_z) +
						 ((x161 + x217) * sensor_y) + ((x215 + x196) * sensor_x);
	const GEN_FLT x221 = (x48 * x218) + (x45 * x220) + (x11 * x216);
	const GEN_FLT x222 = (x12 * x220) + (x38 * x218) + (x41 * x216);
	const GEN_FLT x223 = ((x50 * x222) + (-1 * x43 * x221)) * x53;
	const GEN_FLT x224 = (x94 * x221) + (x93 * x222);
	const GEN_FLT x225 = (x55 * x220) + (x56 * x218) + (x57 * x216);
	const GEN_FLT x226 = (-1 * x73 * x225) + (x97 * x224);
	const GEN_FLT x227 = (-1 * x92 * x226) + x223;
	const GEN_FLT x228 = (x64 * x225) + (-1 * x106 * (x224 + (x225 * x105)));
	const GEN_FLT x229 = x228 * x104;
	const GEN_FLT x230 = x67 * x229;
	const GEN_FLT x231 = (x68 * x229) + (x65 * (x230 + (-1 * x66 * x229)));
	const GEN_FLT x232 = (x69 * x229) + (x65 * x231);
	const GEN_FLT x233 =
		x91 *
		(x226 + (x89 * x232) +
		 (-1 * x117 *
		  ((x85 * ((x65 * x232) + (x70 * x229) +
				   (x65 * (x232 + (x65 * (x231 + (x80 * x229) + (x65 * ((x79 * x229) + (-1 * x229 * x116) + x230)))) +
						   (x81 * x229))) +
				   (x228 * x115))) +
		   (x227 * x114))) +
		 (x229 * x109) + (x227 * x103));
	out[0] = (-1 * x118) + (-1 * x119 * (x118 + (-1 * x54))) + x54;
	out[1] = (-1 * x129) + (-1 * (x129 + (-1 * x120)) * x119) + x120;
	out[2] = (-1 * x139) + (-1 * (x139 + (-1 * x130)) * x119) + x130;
	out[3] = (-1 * x187) + (-1 * (x187 + (-1 * x177)) * x119) + x177;
	out[4] = (-1 * x213) + (-1 * (x213 + (-1 * x203)) * x119) + x203;
	out[5] = (-1 * x233) + (-1 * (x233 + (-1 * x223)) * x119) + x223;
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qj;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qk;
	const GEN_FLT x10 = x9 * lh_qi;
	const GEN_FLT x11 = x10 + x6;
	const GEN_FLT x12 = pow(obj_qk, 2);
	const GEN_FLT x13 = pow(obj_qj, 2);
	const GEN_FLT x14 = pow(obj_qi, 2);
	const GEN_FLT x15 = 1e-10 + x14 + x13 + x12;
	const GEN_FLT x16 = pow(x15, 1.0 / 2.0);
	const GEN_FLT x17 = cos(x16);
	const GEN_FLT x18 = pow(x15, -1) * (1 + (-1 * x17));
	const GEN_FLT x19 = x17 + (x12 * x18);
	const GEN_FLT x20 = pow(x16, -1) * sin(x16);
	const GEN_FLT x21 = x20 * obj_qi;
	const GEN_FLT x22 = x18 * obj_qk * obj_qj;
	const GEN_FLT x23 = x22 + x21;
	const GEN_FLT x24 = x20 * obj_qj;
	const GEN_FLT x25 = x18 * obj_qi;
	const GEN_FLT x26 = x25 * obj_qk;
	const GEN_FLT x27 = x26 + (-1 * x24);
	const GEN_FLT x28 = (x23 * sensor_y) + (x19 * sensor_z) + (x27 * sensor_x) + obj_pz;
	const GEN_FLT x29 = x22 + (-1 * x21);
	const GEN_FLT x30 = x17 + (x13 * x18);
	const GEN_FLT x31 = x20 * obj_qk;
	const GEN_FLT x32 = x25 * obj_qj;
	const GEN_FLT x33 = x32 + x31;
	const GEN_FLT x34 = (x30 * sensor_y) + (x33 * sensor_x) + (x29 * sensor_z) + obj_py;
	const GEN_FLT x35 = x5 * lh_qk;
	const GEN_FLT x36 = x8 * lh_qj * lh_qi;
	const GEN_FLT x37 = x36 + (-1 * x35);
	const GEN_FLT x38 = x26 + x24;
	const GEN_FLT x39 = x32 + (-1 * x31);
	const GEN_FLT x40 = x17 + (x14 * x18);
	const GEN_FLT x41 = (x40 * sensor_x) + (x39 * sensor_y) + (x38 * sensor_z) + obj_px;
	const GEN_FLT x42 = x7 + (x2 * x8);
	const GEN_FLT x43 = (x41 * x42) + (x34 * x37) + (x28 * x11) + lh_px;
	const GEN_FLT x44 = pow(x43, -1);
	const GEN_FLT x45 = x10 + (-1 * x6);
	const GEN_FLT x46 = x5 * lh_qi;
	const GEN_FLT x47 = x9 * lh_qj;
	const GEN_FLT x48 = x47 + x46;
	const GEN_FLT x49 = x7 + (x0 * x8);
	const GEN_FLT x50 = (x49 * x27) + (x48 * x33) + (x40 * x45);
	const GEN_FLT x51 = (x27 * x11) + (x33 * x37) + (x40 * x42);
	const GEN_FLT x52 = pow(x43, 2);
	const GEN_FLT x53 = (x41 * x45) + (x48 * x34) + (x49 * x28) + lh_pz;
	const GEN_FLT x54 = x53 * pow(x52, -1);
	const GEN_FLT x55 = x52 + pow(x53, 2);
	const GEN_FLT x56 = pow(x55, -1);
	const GEN_FLT x57 = x52 * x56;
	const GEN_FLT x58 = ((x54 * x51) + (-1 * x50 * x44)) * x57;
	const GEN_FLT x59 = x47 + (-1 * x46);
	const GEN_FLT x60 = x7 + (x1 * x8);
	const GEN_FLT x61 = x36 + x35;
	const GEN_FLT x62 = (x61 * x41) + lh_py + (x60 * x34) + (x59 * x28);
	const GEN_FLT x63 = pow(x62, 2);
	const GEN_FLT x64 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x65 = tan(x64);
	const GEN_FLT x66 = pow((1 + (-1 * x63 * pow(x65, 2) * x56)), -1.0 / 2.0);
	const GEN_FLT x67 = 2 * x43;
	const GEN_FLT x68 = 2 * x53;
	const GEN_FLT x69 = (x68 * x50) + (x67 * x51);
	const GEN_FLT x70 = 1.0 / 2.0 * x62;
	const GEN_FLT x71 = x70 * x65 * pow(x55, -3.0 / 2.0);
	const GEN_FLT x72 = (x59 * x27) + (x60 * x33) + (x61 * x40);
	const GEN_FLT x73 = x65 * pow(x55, -1.0 / 2.0);
	const GEN_FLT x74 = (-1 * x73 * x72) + (x71 * x69);
	const GEN_FLT x75 = -1 * x73 * x62;
	const GEN_FLT x76 = atan2(-1 * x53, x43);
	const GEN_FLT x77 = x76 + ogeeMag_1 + (-1 * asin(x75));
	const GEN_FLT x78 = cos(x77) * ogeePhase_1;
	const GEN_FLT x79 = x78 * ((-1 * x74 * x66) + x58);
	const GEN_FLT x80 = cos(x64);
	const GEN_FLT x81 = pow(x80, -1);
	const GEN_FLT x82 = x55 + x63;
	const GEN_FLT x83 = x81 * pow(x82, -1.0 / 2.0);
	const GEN_FLT x84 = asin(x83 * x62);
	const GEN_FLT x85 = 8.0108022e-06 * x84;
	const GEN_FLT x86 = -8.0108022e-06 + (-1 * x85);
	const GEN_FLT x87 = 0.0028679863 + (x84 * x86);
	const GEN_FLT x88 = 5.3685255e-06 + (x84 * x87);
	const GEN_FLT x89 = 0.0076069798 + (x88 * x84);
	const GEN_FLT x90 = pow(x84, 2);
	const GEN_FLT x91 = x89 * x84;
	const GEN_FLT x92 = -8.0108022e-06 + (-1.60216044e-05 * x84);
	const GEN_FLT x93 = x87 + (x84 * x92);
	const GEN_FLT x94 = x88 + (x84 * x93);
	const GEN_FLT x95 = x89 + (x84 * x94);
	const GEN_FLT x96 = (x84 * x95) + x91;
	const GEN_FLT x97 = (sin(x77) * ogeePhase_1) + curve_1;
	const GEN_FLT x98 = sin(x64);
	const GEN_FLT x99 = x98 * x97;
	const GEN_FLT x100 = x80 + (x99 * x96);
	const GEN_FLT x101 = pow(x100, -1);
	const GEN_FLT x102 = x89 * x90 * x101;
	const GEN_FLT x103 = pow((1 + (-1 * pow(x80, -2) * pow(x82, -1) * x63)), -1.0 / 2.0);
	const GEN_FLT x104 = 2 * x62;
	const GEN_FLT x105 = x81 * pow(x82, -3.0 / 2.0) * x70;
	const GEN_FLT x106 = x103 * ((x83 * x72) + (-1 * x105 * (x69 + (x72 * x104))));
	const GEN_FLT x107 = 2 * x91 * x97 * x101;
	const GEN_FLT x108 = x86 * x106;
	const GEN_FLT x109 = (x87 * x106) + (x84 * (x108 + (-1 * x85 * x106)));
	const GEN_FLT x110 = (x88 * x106) + (x84 * x109);
	const GEN_FLT x111 = x90 * x97;
	const GEN_FLT x112 = x101 * x111;
	const GEN_FLT x113 = x98 * x96;
	const GEN_FLT x114 = 2.40324066e-05 * x84;
	const GEN_FLT x115 = x89 * pow(x100, -2) * x111;
	const GEN_FLT x116 = x75 + (x89 * x112);
	const GEN_FLT x117 = pow((1 + (-1 * pow(x116, 2))), -1.0 / 2.0);
	const GEN_FLT x118 =
		x117 *
		(x74 +
		 (-1 * x115 *
		  ((x99 * ((x84 * x110) + (x89 * x106) +
				   (x84 * (x110 + (x84 * (x109 + (x93 * x106) + (x84 * ((x92 * x106) + (-1 * x106 * x114) + x108)))) +
						   (x94 * x106))) +
				   (x95 * x106))) +
		   (x79 * x113))) +
		 (x107 * x106) + (x110 * x112) + (x79 * x102));
	const GEN_FLT x119 = cos(x76 + (-1 * asin(x116)) + gibPhase_1) * gibMag_1;
	const GEN_FLT x120 = (x49 * x23) + (x48 * x30) + (x45 * x39);
	const GEN_FLT x121 = (x23 * x11) + (x30 * x37) + (x42 * x39);
	const GEN_FLT x122 = ((x54 * x121) + (-1 * x44 * x120)) * x57;
	const GEN_FLT x123 = (x68 * x120) + (x67 * x121);
	const GEN_FLT x124 = (x59 * x23) + (x60 * x30) + (x61 * x39);
	const GEN_FLT x125 = (-1 * x73 * x124) + (x71 * x123);
	const GEN_FLT x126 = (-1 * x66 * x125) + x122;
	const GEN_FLT x127 = x78 * x102;
	const GEN_FLT x128 = x103 * ((x83 * x124) + (-1 * x105 * (x123 + (x104 * x124))));
	const GEN_FLT x129 = x86 * x128;
	const GEN_FLT x130 = (x87 * x128) + (x84 * (x129 + (-1 * x85 * x128)));
	const GEN_FLT x131 = (x88 * x128) + (x84 * x130);
	const GEN_FLT x132 = x78 * x113;
	const GEN_FLT x133 =
		x117 *
		(x125 +
		 (-1 * x115 *
		  ((x99 * ((x84 * x131) + (x89 * x128) +
				   (x84 * (x131 + (x84 * ((x93 * x128) + x130 + (x84 * ((x92 * x128) + (-1 * x114 * x128) + x129)))) +
						   (x94 * x128))) +
				   (x95 * x128))) +
		   (x126 * x132))) +
		 (x112 * x131) + (x107 * x128) + (x127 * x126));
	const GEN_FLT x134 = (x49 * x19) + (x48 * x29) + (x45 * x38);
	const GEN_FLT x135 = (x11 * x19) + (x37 * x29) + (x42 * x38);
	const GEN_FLT x136 = ((x54 * x135) + (-1 * x44 * x134)) * x57;
	const GEN_FLT x137 = (x68 * x134) + (x67 * x135);
	const GEN_FLT x138 = (x59 * x19) + (x60 * x29) + (x61 * x38);
	const GEN_FLT x139 = (-1 * x73 * x138) + (x71 * x137);
	const GEN_FLT x140 = (-1 * x66 * x139) + x136;
	const GEN_FLT x141 = x103 * ((x83 * x138) + (-1 * x105 * (x137 + (x104 * x138))));
	const GEN_FLT x142 = x86 * x141;
	const GEN_FLT x143 = (x87 * x141) + (x84 * (x142 + (-1 * x85 * x141)));
	const GEN_FLT x144 = (x88 * x141) + (x84 * x143);
	const GEN_FLT x145 =
		x117 *
		(x139 +
		 (-1 * x115 *
		  ((x99 * ((x84 * x144) + (x89 * x141) +
				   (x84 * (x144 + (x84 * (x143 + (x93 * x141) + (x84 * ((x92 * x141) + (-1 * x114 * x141) + x142)))) +
						   (x94 * x141))) +
				   (x95 * x141))) +
		   (x132 * x140))) +
		 (x112 * x144) + (x107 * x141) + (x127 * x140));
	out[0] = (-1 * x118) + x58 + (-1 * x119 * (x118 + (-1 * x58)));
	out[1] = (-1 * x133) + (-1 * (x133 + (-1 * x122)) * x119) + x122;
	out[2] = (-1 * x145) + (-1 * (x145 + (-1 * x136)) * x119) + x136;
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
	const GEN_FLT x0 = pow(obj_qk, 2);
	const GEN_FLT x1 = pow(obj_qj, 2);
	const GEN_FLT x2 = pow(obj_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = pow(x3, -1) * (1 + (-1 * x5));
	const GEN_FLT x7 = pow(x4, -1) * sin(x4);
	const GEN_FLT x8 = x7 * obj_qi;
	const GEN_FLT x9 = x6 * obj_qk;
	const GEN_FLT x10 = x9 * obj_qj;
	const GEN_FLT x11 = x7 * obj_qj;
	const GEN_FLT x12 = x9 * obj_qi;
	const GEN_FLT x13 =
		((x12 + (-1 * x11)) * sensor_x) + ((x10 + x8) * sensor_y) + ((x5 + (x0 * x6)) * sensor_z) + obj_pz;
	const GEN_FLT x14 = pow(lh_qk, 2);
	const GEN_FLT x15 = pow(lh_qj, 2);
	const GEN_FLT x16 = pow(lh_qi, 2);
	const GEN_FLT x17 = 1e-10 + x16 + x15 + x14;
	const GEN_FLT x18 = pow(x17, -1);
	const GEN_FLT x19 = pow(x17, 1.0 / 2.0);
	const GEN_FLT x20 = cos(x19);
	const GEN_FLT x21 = 1 + (-1 * x20);
	const GEN_FLT x22 = x21 * x18;
	const GEN_FLT x23 = x7 * obj_qk;
	const GEN_FLT x24 = x6 * obj_qj * obj_qi;
	const GEN_FLT x25 =
		((x24 + x23) * sensor_x) + ((x5 + (x1 * x6)) * sensor_y) + ((x10 + (-1 * x8)) * sensor_z) + obj_py;
	const GEN_FLT x26 = sin(x19);
	const GEN_FLT x27 = x26 * pow(x19, -1);
	const GEN_FLT x28 = x27 * lh_qi;
	const GEN_FLT x29 = x22 * lh_qk;
	const GEN_FLT x30 = x29 * lh_qj;
	const GEN_FLT x31 =
		((x24 + (-1 * x23)) * sensor_y) + ((x5 + (x2 * x6)) * sensor_x) + ((x12 + x11) * sensor_z) + obj_px;
	const GEN_FLT x32 = x27 * lh_qj;
	const GEN_FLT x33 = -1 * x32;
	const GEN_FLT x34 = x29 * lh_qi;
	const GEN_FLT x35 = ((x34 + x33) * x31) + (x13 * (x20 + (x22 * x14))) + ((x30 + x28) * x25) + lh_pz;
	const GEN_FLT x36 = x27 * lh_qk;
	const GEN_FLT x37 = -1 * x36;
	const GEN_FLT x38 = x22 * lh_qi;
	const GEN_FLT x39 = x38 * lh_qj;
	const GEN_FLT x40 = (x31 * (x20 + (x22 * x16))) + ((x39 + x37) * x25) + ((x34 + x32) * x13) + lh_px;
	const GEN_FLT x41 = pow(x40, 2);
	const GEN_FLT x42 = x41 + pow(x35, 2);
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = x43 * x35;
	const GEN_FLT x45 = -1 * x28;
	const GEN_FLT x46 = ((x39 + x36) * x31) + lh_py + (x25 * (x20 + (x22 * x15))) + ((x30 + x45) * x13);
	const GEN_FLT x47 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x48 = cos(x47);
	const GEN_FLT x49 = pow(x48, -1);
	const GEN_FLT x50 = pow(x46, 2);
	const GEN_FLT x51 = x42 + x50;
	const GEN_FLT x52 = pow(x51, -1.0 / 2.0) * x49;
	const GEN_FLT x53 = asin(x52 * x46);
	const GEN_FLT x54 = 8.0108022e-06 * x53;
	const GEN_FLT x55 = -8.0108022e-06 + (-1 * x54);
	const GEN_FLT x56 = 0.0028679863 + (x53 * x55);
	const GEN_FLT x57 = 5.3685255e-06 + (x53 * x56);
	const GEN_FLT x58 = 0.0076069798 + (x53 * x57);
	const GEN_FLT x59 = pow(x53, 2);
	const GEN_FLT x60 = tan(x47);
	const GEN_FLT x61 = x60 * pow(x42, -1.0 / 2.0);
	const GEN_FLT x62 = -1 * x61 * x46;
	const GEN_FLT x63 = atan2(-1 * x35, x40);
	const GEN_FLT x64 = x63 + ogeeMag_1 + (-1 * asin(x62));
	const GEN_FLT x65 = (sin(x64) * ogeePhase_1) + curve_1;
	const GEN_FLT x66 = x53 * x58;
	const GEN_FLT x67 = -8.0108022e-06 + (-1.60216044e-05 * x53);
	const GEN_FLT x68 = x56 + (x67 * x53);
	const GEN_FLT x69 = x57 + (x68 * x53);
	const GEN_FLT x70 = x58 + (x69 * x53);
	const GEN_FLT x71 = (x70 * x53) + x66;
	const GEN_FLT x72 = sin(x47);
	const GEN_FLT x73 = x72 * x65;
	const GEN_FLT x74 = x48 + (x71 * x73);
	const GEN_FLT x75 = pow(x74, -1);
	const GEN_FLT x76 = x75 * x65;
	const GEN_FLT x77 = x76 * x59;
	const GEN_FLT x78 = x62 + (x77 * x58);
	const GEN_FLT x79 = pow((1 + (-1 * pow(x78, 2))), -1.0 / 2.0);
	const GEN_FLT x80 = x40 * x46;
	const GEN_FLT x81 = x60 * pow(x42, -3.0 / 2.0);
	const GEN_FLT x82 = x80 * x81;
	const GEN_FLT x83 = pow((1 + (-1 * pow(x60, 2) * x50 * x43)), -1.0 / 2.0);
	const GEN_FLT x84 = (-1 * x82 * x83) + x44;
	const GEN_FLT x85 = cos(x64) * ogeePhase_1;
	const GEN_FLT x86 = x58 * x59;
	const GEN_FLT x87 = x86 * x75;
	const GEN_FLT x88 = x85 * x87;
	const GEN_FLT x89 = pow((1 + (-1 * x50 * pow(x51, -1) * pow(x48, -2))), -1.0 / 2.0);
	const GEN_FLT x90 = 2 * x46;
	const GEN_FLT x91 = pow(x51, -3.0 / 2.0) * x49;
	const GEN_FLT x92 = x76 * x66;
	const GEN_FLT x93 = x89 * x92 * x91 * x90;
	const GEN_FLT x94 = x80 * x91;
	const GEN_FLT x95 = x89 * x94;
	const GEN_FLT x96 = x89 * x55;
	const GEN_FLT x97 = -1 * x96 * x94;
	const GEN_FLT x98 = (-1 * x56 * x95) + (x53 * (x97 + (x54 * x95)));
	const GEN_FLT x99 = (-1 * x57 * x95) + (x53 * x98);
	const GEN_FLT x100 = x71 * x72;
	const GEN_FLT x101 = x85 * x100;
	const GEN_FLT x102 = 2.40324066e-05 * x53;
	const GEN_FLT x103 = x89 * x58;
	const GEN_FLT x104 = x89 * x70;
	const GEN_FLT x105 = x86 * pow(x74, -2) * x65;
	const GEN_FLT x106 =
		x79 *
		((-1 * x105 *
		  ((x73 * ((x53 * x99) + (-1 * x94 * x104) + (-1 * x94 * x103) +
				   (x53 * (x99 + (x53 * (x98 + (-1 * x68 * x95) + (x53 * ((-1 * x67 * x95) + (x95 * x102) + x97)))) +
						   (-1 * x69 * x95))))) +
		   (x84 * x101))) +
		 (-1 * x93 * x40) + (x77 * x99) + (x88 * x84) + x82);
	const GEN_FLT x107 = cos(x63 + (-1 * asin(x78)) + gibPhase_1) * gibMag_1;
	const GEN_FLT x108 = x83 * x61;
	const GEN_FLT x109 = x52 + (-1 * x50 * x91);
	const GEN_FLT x110 = x89 * x109;
	const GEN_FLT x111 = x96 * x109;
	const GEN_FLT x112 = (x56 * x110) + (x53 * (x111 + (-1 * x54 * x110)));
	const GEN_FLT x113 = (x57 * x110) + (x53 * x112);
	const GEN_FLT x114 = 2 * x92;
	const GEN_FLT x115 =
		x79 *
		((x77 * x113) + (x110 * x114) +
		 (-1 * x105 *
		  ((x73 * ((x53 * x113) + (x58 * x110) +
				   (x53 * (x113 + (x53 * (x112 + (x68 * x110) + (x53 * ((x67 * x110) + (-1 * x102 * x110) + x111)))) +
						   (x69 * x110))) +
				   (x70 * x110))) +
		   (x101 * x108))) +
		 (x88 * x108) + (-1 * x61));
	const GEN_FLT x116 = x40 * x43;
	const GEN_FLT x117 = -1 * x116;
	const GEN_FLT x118 = x81 * x46;
	const GEN_FLT x119 = x35 * x118;
	const GEN_FLT x120 = (-1 * x83 * x119) + x117;
	const GEN_FLT x121 = x91 * x46;
	const GEN_FLT x122 = x35 * x121;
	const GEN_FLT x123 = x89 * x122;
	const GEN_FLT x124 = -1 * x96 * x122;
	const GEN_FLT x125 = (-1 * x56 * x123) + (x53 * (x124 + (x54 * x123)));
	const GEN_FLT x126 = (-1 * x57 * x123) + (x53 * x125);
	const GEN_FLT x127 =
		x79 *
		((-1 * x105 *
		  ((x73 *
			((x53 * x126) + (-1 * x103 * x122) +
			 (x53 * (x126 + (x53 * (x125 + (-1 * x68 * x123) + (x53 * ((-1 * x67 * x123) + (x102 * x123) + x124)))) +
					 (-1 * x69 * x123))) +
			 (-1 * x104 * x122))) +
		   (x101 * x120))) +
		 x119 + (x77 * x126) + (-1 * x93 * x35) + (x88 * x120));
	const GEN_FLT x128 = x26 * pow(x17, -3.0 / 2.0);
	const GEN_FLT x129 = x16 * x128;
	const GEN_FLT x130 = 2 * x21 * pow(x17, -2);
	const GEN_FLT x131 = x130 * lh_qk;
	const GEN_FLT x132 = (-1 * x16 * x131) + (x129 * lh_qk);
	const GEN_FLT x133 = x132 + x29;
	const GEN_FLT x134 = x128 * lh_qj * lh_qi;
	const GEN_FLT x135 = x20 * x18;
	const GEN_FLT x136 = x135 * lh_qj;
	const GEN_FLT x137 = x136 * lh_qi;
	const GEN_FLT x138 = (-1 * x137) + x134;
	const GEN_FLT x139 = x16 * x135;
	const GEN_FLT x140 = x128 * lh_qk;
	const GEN_FLT x141 = x140 * lh_qi;
	const GEN_FLT x142 = lh_qk * lh_qi;
	const GEN_FLT x143 = x130 * lh_qj;
	const GEN_FLT x144 = (-1 * x142 * x143) + (x141 * lh_qj);
	const GEN_FLT x145 = x144 + x27;
	const GEN_FLT x146 = x14 * x128;
	const GEN_FLT x147 = x14 * x130;
	const GEN_FLT x148 = (-1 * x147 * lh_qi) + (x146 * lh_qi);
	const GEN_FLT x149 = (x13 * (x148 + x45)) + (x25 * (x145 + x139 + (-1 * x129))) + ((x138 + x133) * x31);
	const GEN_FLT x150 = pow(x40, -1);
	const GEN_FLT x151 = pow(lh_qi, 3);
	const GEN_FLT x152 = x22 * lh_qj;
	const GEN_FLT x153 = (-1 * x16 * x143) + (x129 * lh_qj);
	const GEN_FLT x154 = x153 + x152;
	const GEN_FLT x155 = x135 * x142;
	const GEN_FLT x156 = (-1 * x155) + x141;
	const GEN_FLT x157 = x137 + (-1 * x134);
	const GEN_FLT x158 =
		((x157 + x133) * x13) + ((x156 + x154) * x25) + (((-1 * x130 * x151) + (x128 * x151) + (2 * x38) + x45) * x31);
	const GEN_FLT x159 = pow(x41, -1) * x35;
	const GEN_FLT x160 = x41 * x43;
	const GEN_FLT x161 = ((x158 * x159) + (-1 * x149 * x150)) * x160;
	const GEN_FLT x162 = 2 * x40;
	const GEN_FLT x163 = 2 * x35;
	const GEN_FLT x164 = (x163 * x149) + (x162 * x158);
	const GEN_FLT x165 = 1.0 / 2.0 * x118;
	const GEN_FLT x166 = x155 + (-1 * x141);
	const GEN_FLT x167 = x15 * x128;
	const GEN_FLT x168 = (-1 * x15 * x130 * lh_qi) + (x167 * lh_qi);
	const GEN_FLT x169 = x144 + (-1 * x27);
	const GEN_FLT x170 = (x13 * (x169 + (-1 * x139) + x129)) + (x25 * (x168 + x45)) + ((x166 + x154) * x31);
	const GEN_FLT x171 = (-1 * x61 * x170) + (x165 * x164);
	const GEN_FLT x172 = (-1 * x83 * x171) + x161;
	const GEN_FLT x173 = 1.0 / 2.0 * x121;
	const GEN_FLT x174 = (x52 * x170) + (-1 * x173 * (x164 + (x90 * x170)));
	const GEN_FLT x175 = x89 * x174;
	const GEN_FLT x176 = x96 * x174;
	const GEN_FLT x177 = (x56 * x175) + (x53 * (x176 + (-1 * x54 * x175)));
	const GEN_FLT x178 = (x57 * x175) + (x53 * x177);
	const GEN_FLT x179 =
		x79 *
		(x171 + (x77 * x178) +
		 (-1 * x105 *
		  ((x73 * ((x53 * x178) + (x58 * x175) +
				   (x53 * (x178 + (x53 * (x177 + (x68 * x175) + (x53 * ((x67 * x175) + (-1 * x102 * x175) + x176)))) +
						   (x69 * x175))) +
				   (x70 * x175))) +
		   (x101 * x172))) +
		 (x114 * x175) + (x88 * x172));
	const GEN_FLT x180 = x15 * x135;
	const GEN_FLT x181 = (-1 * x15 * x131) + (x167 * lh_qk);
	const GEN_FLT x182 = x181 + x29;
	const GEN_FLT x183 = (-1 * x147 * lh_qj) + (x146 * lh_qj);
	const GEN_FLT x184 = (x13 * (x183 + x33)) + ((x182 + x157) * x25) + (x31 * (x169 + (-1 * x180) + x167));
	const GEN_FLT x185 = x168 + x38;
	const GEN_FLT x186 = x140 * lh_qj;
	const GEN_FLT x187 = x136 * lh_qk;
	const GEN_FLT x188 = (-1 * x187) + x186;
	const GEN_FLT x189 = (x13 * (x145 + x180 + (-1 * x167))) + ((x188 + x185) * x25) + (x31 * (x153 + x33));
	const GEN_FLT x190 = ((x189 * x159) + (-1 * x184 * x150)) * x160;
	const GEN_FLT x191 = (x163 * x184) + (x162 * x189);
	const GEN_FLT x192 = pow(lh_qj, 3);
	const GEN_FLT x193 = x187 + (-1 * x186);
	const GEN_FLT x194 =
		((x138 + x182) * x13) + ((x193 + x185) * x31) + (x25 * ((-1 * x192 * x130) + (x128 * x192) + (2 * x152) + x33));
	const GEN_FLT x195 = (-1 * x61 * x194) + (x165 * x191);
	const GEN_FLT x196 = (-1 * x83 * x195) + x190;
	const GEN_FLT x197 = x89 * ((x52 * x194) + (-1 * x173 * (x191 + (x90 * x194))));
	const GEN_FLT x198 = x55 * x197;
	const GEN_FLT x199 = (x56 * x197) + (x53 * (x198 + (-1 * x54 * x197)));
	const GEN_FLT x200 = (x57 * x197) + (x53 * x199);
	const GEN_FLT x201 =
		x79 *
		(x195 +
		 (-1 * x105 *
		  ((x73 * ((x53 * x200) + (x58 * x197) +
				   (x53 * (x200 + (x53 * (x199 + (x68 * x197) + (x53 * ((-1 * x102 * x197) + (x67 * x197) + x198)))) +
						   (x69 * x197))) +
				   (x70 * x197))) +
		   (x101 * x196))) +
		 (x77 * x200) + (x114 * x197) + (x88 * x196));
	const GEN_FLT x202 = x148 + x38;
	const GEN_FLT x203 = x183 + x152;
	const GEN_FLT x204 = pow(lh_qk, 3);
	const GEN_FLT x205 =
		(((x204 * x128) + (-1 * x204 * x130) + (2 * x29) + x37) * x13) + ((x203 + x166) * x25) + ((x202 + x188) * x31);
	const GEN_FLT x206 = x14 * x135;
	const GEN_FLT x207 = ((x202 + x193) * x13) + (x25 * (x169 + (-1 * x206) + x146)) + (x31 * (x132 + x37));
	const GEN_FLT x208 = ((x207 * x159) + (-1 * x205 * x150)) * x160;
	const GEN_FLT x209 = (x205 * x163) + (x207 * x162);
	const GEN_FLT x210 = ((x203 + x156) * x13) + (x25 * (x181 + x37)) + (x31 * (x145 + x206 + (-1 * x146)));
	const GEN_FLT x211 = (-1 * x61 * x210) + (x209 * x165);
	const GEN_FLT x212 = x85 * ((-1 * x83 * x211) + x208);
	const GEN_FLT x213 = (x52 * x210) + (-1 * x173 * (x209 + (x90 * x210)));
	const GEN_FLT x214 = x89 * x213;
	const GEN_FLT x215 = x96 * x213;
	const GEN_FLT x216 = (x56 * x214) + (x53 * (x215 + (-1 * x54 * x214)));
	const GEN_FLT x217 = (x57 * x214) + (x53 * x216);
	const GEN_FLT x218 =
		x79 *
		(x211 +
		 (-1 * x105 *
		  ((x73 * ((x53 * x217) + (x58 * x214) +
				   (x53 * (x217 + (x53 * (x216 + (x68 * x214) + (x53 * ((x67 * x214) + x215 + (-1 * x214 * x102))))) +
						   (x69 * x214))) +
				   (x70 * x214))) +
		   (x212 * x100))) +
		 (x77 * x217) + (x214 * x114) + (x87 * x212));
	out[0] = x44 + (-1 * x106) + (-1 * x107 * (x106 + (-1 * x44)));
	out[1] = (-1 * x115) + (-1 * x107 * x115);
	out[2] = (-1 * x127) + (-1 * (x127 + x116) * x107) + x117;
	out[3] = (-1 * x179) + x161 + (-1 * (x179 + (-1 * x161)) * x107);
	out[4] = (-1 * x201) + (-1 * (x201 + (-1 * x190)) * x107) + x190;
	out[5] = (-1 * x218) + (-1 * (x218 + (-1 * x208)) * x107) + x208;
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
	const GEN_FLT x0 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x1 = tan(x0);
	const GEN_FLT x2 = pow(lh_qk, 2);
	const GEN_FLT x3 = pow(lh_qj, 2);
	const GEN_FLT x4 = pow(lh_qi, 2);
	const GEN_FLT x5 = 1e-10 + x4 + x3 + x2;
	const GEN_FLT x6 = pow(x5, 1.0 / 2.0);
	const GEN_FLT x7 = pow(x6, -1) * sin(x6);
	const GEN_FLT x8 = x7 * lh_qi;
	const GEN_FLT x9 = cos(x6);
	const GEN_FLT x10 = pow(x5, -1) * (1 + (-1 * x9));
	const GEN_FLT x11 = x10 * lh_qk;
	const GEN_FLT x12 = x11 * lh_qj;
	const GEN_FLT x13 = pow(obj_qk, 2);
	const GEN_FLT x14 = pow(obj_qj, 2);
	const GEN_FLT x15 = pow(obj_qi, 2);
	const GEN_FLT x16 = 1e-10 + x15 + x14 + x13;
	const GEN_FLT x17 = pow(x16, 1.0 / 2.0);
	const GEN_FLT x18 = cos(x17);
	const GEN_FLT x19 = pow(x16, -1) * (1 + (-1 * x18));
	const GEN_FLT x20 = pow(x17, -1) * sin(x17);
	const GEN_FLT x21 = x20 * obj_qi;
	const GEN_FLT x22 = x19 * obj_qk * obj_qj;
	const GEN_FLT x23 = x20 * obj_qj;
	const GEN_FLT x24 = x19 * obj_qi;
	const GEN_FLT x25 = x24 * obj_qk;
	const GEN_FLT x26 =
		((x25 + (-1 * x23)) * sensor_x) + ((x22 + x21) * sensor_y) + ((x18 + (x13 * x19)) * sensor_z) + obj_pz;
	const GEN_FLT x27 = x20 * obj_qk;
	const GEN_FLT x28 = x24 * obj_qj;
	const GEN_FLT x29 =
		((x28 + x27) * sensor_x) + ((x18 + (x14 * x19)) * sensor_y) + ((x22 + (-1 * x21)) * sensor_z) + obj_py;
	const GEN_FLT x30 =
		((x18 + (x15 * x19)) * sensor_x) + ((x28 + (-1 * x27)) * sensor_y) + ((x25 + x23) * sensor_z) + obj_px;
	const GEN_FLT x31 = x7 * lh_qk;
	const GEN_FLT x32 = x10 * lh_qj * lh_qi;
	const GEN_FLT x33 = ((x32 + x31) * x30) + (x29 * (x9 + (x3 * x10))) + lh_py + (x26 * (x12 + (-1 * x8)));
	const GEN_FLT x34 = x7 * lh_qj;
	const GEN_FLT x35 = x11 * lh_qi;
	const GEN_FLT x36 = ((x35 + (-1 * x34)) * x30) + (x29 * (x12 + x8)) + (x26 * (x9 + (x2 * x10))) + lh_pz;
	const GEN_FLT x37 = (x30 * (x9 + (x4 * x10))) + ((x32 + (-1 * x31)) * x29) + ((x35 + x34) * x26) + lh_px;
	const GEN_FLT x38 = pow(x37, 2) + pow(x36, 2);
	const GEN_FLT x39 = x33 * pow(x38, -1.0 / 2.0);
	const GEN_FLT x40 = -1 * x1 * x39;
	const GEN_FLT x41 = atan2(-1 * x36, x37);
	const GEN_FLT x42 = x41 + ogeeMag_1 + (-1 * asin(x40));
	const GEN_FLT x43 = sin(x42);
	const GEN_FLT x44 = (x43 * ogeePhase_1) + curve_1;
	const GEN_FLT x45 = cos(x0);
	const GEN_FLT x46 = pow(x33, 2);
	const GEN_FLT x47 = x38 + x46;
	const GEN_FLT x48 = pow(x47, -1.0 / 2.0) * x33;
	const GEN_FLT x49 = asin(pow(x45, -1) * x48);
	const GEN_FLT x50 = 8.0108022e-06 * x49;
	const GEN_FLT x51 = -8.0108022e-06 + (-1 * x50);
	const GEN_FLT x52 = 0.0028679863 + (x51 * x49);
	const GEN_FLT x53 = 5.3685255e-06 + (x52 * x49);
	const GEN_FLT x54 = 0.0076069798 + (x53 * x49);
	const GEN_FLT x55 = pow(x49, 2);
	const GEN_FLT x56 = x54 * x49;
	const GEN_FLT x57 = -8.0108022e-06 + (-1.60216044e-05 * x49);
	const GEN_FLT x58 = x52 + (x57 * x49);
	const GEN_FLT x59 = x53 + (x58 * x49);
	const GEN_FLT x60 = x54 + (x59 * x49);
	const GEN_FLT x61 = (x60 * x49) + x56;
	const GEN_FLT x62 = sin(x0);
	const GEN_FLT x63 = x62 * x44;
	const GEN_FLT x64 = x63 * x61;
	const GEN_FLT x65 = x45 + x64;
	const GEN_FLT x66 = pow(x65, -1);
	const GEN_FLT x67 = x66 * x55;
	const GEN_FLT x68 = x67 * x54;
	const GEN_FLT x69 = x40 + (x68 * x44);
	const GEN_FLT x70 = pow((1 + (-1 * pow(x69, 2))), -1.0 / 2.0);
	const GEN_FLT x71 = pow(x1, 2);
	const GEN_FLT x72 = x39 * (1 + x71);
	const GEN_FLT x73 = cos(x42) * ogeePhase_1;
	const GEN_FLT x74 = x73 * x68;
	const GEN_FLT x75 = x72 * pow((1 + (-1 * x71 * x46 * pow(x38, -1))), -1.0 / 2.0);
	const GEN_FLT x76 = pow(x45, -2);
	const GEN_FLT x77 = x76 * x48 * pow((1 + (-1 * x76 * x46 * pow(x47, -1))), -1.0 / 2.0);
	const GEN_FLT x78 = x77 * x62;
	const GEN_FLT x79 = -1 * x78 * x51;
	const GEN_FLT x80 = (-1 * x78 * x52) + (x49 * (x79 + (x78 * x50)));
	const GEN_FLT x81 = (-1 * x78 * x53) + (x80 * x49);
	const GEN_FLT x82 = pow(x65, -2) * x54 * x55;
	const GEN_FLT x83 =
		x70 * ((-1 * x82 * x44 *
				((-1 * x61 * x44 * x45) +
				 (x63 * ((x81 * x49) + (-1 * x78 * x54) +
						 (x49 * ((x49 * (x80 + (-1 * x78 * x58) +
										 (x49 * ((-1 * x78 * x57) + (2.40324066e-05 * x78 * x49) + x79)))) +
								 x81 + (-1 * x78 * x59))) +
						 (-1 * x78 * x60))) +
				 (-1 * x73 * x75 * x61 * x62) + x62)) +
			   (x81 * x67 * x44) + (-2 * x77 * x63 * x66 * x56) + (-1 * x75 * x74) + x72);
	const GEN_FLT x84 = (-1 * x41) + asin(x69) + (-1 * gibPhase_1);
	const GEN_FLT x85 = cos(x84) * gibMag_1;
	const GEN_FLT x86 = x82 * x64;
	const GEN_FLT x87 = ((-1 * x86) + x68) * x70;
	const GEN_FLT x88 = x70 * ((-1 * x86 * x73) + x74);
	const GEN_FLT x89 = ((-1 * x86 * x43) + (x68 * x43)) * x70;
	out[0] = -1;
	out[1] = (-1 * x83) + (-1 * x83 * x85);
	out[2] = (-1 * x87) + (-1 * x85 * x87);
	out[3] = x85;
	out[4] = -1 * sin(x84);
	out[5] = (-1 * x88) + (-1 * x88 * x85);
	out[6] = (-1 * x89) + (-1 * x89 * x85);
}

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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qi;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qj;
	const GEN_FLT x10 = x9 * lh_qk;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = pow(obj_qj, 2);
	const GEN_FLT x13 = pow(obj_qi, 2);
	const GEN_FLT x14 = 1e-10 + x13 + x12 + x11;
	const GEN_FLT x15 = pow(x14, 1.0 / 2.0);
	const GEN_FLT x16 = cos(x15);
	const GEN_FLT x17 = pow(x14, -1) * (1 + (-1 * x16));
	const GEN_FLT x18 = pow(x15, -1) * sin(x15);
	const GEN_FLT x19 = x18 * obj_qi;
	const GEN_FLT x20 = x17 * obj_qk;
	const GEN_FLT x21 = x20 * obj_qj;
	const GEN_FLT x22 = x18 * obj_qj;
	const GEN_FLT x23 = x20 * obj_qi;
	const GEN_FLT x24 =
		((x21 + x19) * sensor_y) + ((x23 + (-1 * x22)) * sensor_x) + ((x16 + (x11 * x17)) * sensor_z) + obj_pz;
	const GEN_FLT x25 = x18 * obj_qk;
	const GEN_FLT x26 = x17 * obj_qj * obj_qi;
	const GEN_FLT x27 =
		((x26 + x25) * sensor_x) + ((x16 + (x12 * x17)) * sensor_y) + ((x21 + (-1 * x19)) * sensor_z) + obj_py;
	const GEN_FLT x28 =
		((x26 + (-1 * x25)) * sensor_y) + ((x23 + x22) * sensor_z) + ((x16 + (x13 * x17)) * sensor_x) + obj_px;
	const GEN_FLT x29 = x5 * lh_qk;
	const GEN_FLT x30 = x9 * lh_qi;
	const GEN_FLT x31 = lh_py + ((x30 + x29) * x28) + (x27 * (x7 + (x1 * x8))) + (x24 * (x10 + (-1 * x6)));
	const GEN_FLT x32 = x5 * lh_qj;
	const GEN_FLT x33 = x8 * lh_qk * lh_qi;
	const GEN_FLT x34 = ((x33 + (-1 * x32)) * x28) + (x27 * (x10 + x6)) + (x24 * (x7 + (x0 * x8))) + lh_pz;
	const GEN_FLT x35 = -1 * x34;
	const GEN_FLT x36 = pow(x34, 2);
	const GEN_FLT x37 = ((x30 + (-1 * x29)) * x27) + ((x33 + x32) * x24) + (x28 * (x7 + (x2 * x8))) + lh_px;
	const GEN_FLT x38 = atan2(x37, x35);
	const GEN_FLT x39 = (-1 * x38) + (-1 * phase_0) + (-1 * asin(x31 * pow((pow(x37, 2) + x36), -1.0 / 2.0) * tilt_0));
	const GEN_FLT x40 =
		(-1 * atan2(-1 * x31, x35)) + (-1 * phase_1) + (-1 * asin(x37 * pow((pow(x31, 2) + x36), -1.0 / 2.0) * tilt_1));
	out[0] = x39 + (-1 * cos(1.5707963267949 + x39 + gibPhase_0) * gibMag_0) + (pow(atan2(x31, x35), 2) * curve_0);
	out[1] = x40 + (-1 * cos(1.5707963267949 + x40 + gibPhase_1) * gibMag_1) + (pow(x38, 2) * curve_1);
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qj;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qk;
	const GEN_FLT x10 = x9 * lh_qi;
	const GEN_FLT x11 = x10 + (-1 * x6);
	const GEN_FLT x12 = x10 + x6;
	const GEN_FLT x13 = pow(obj_qk, 2);
	const GEN_FLT x14 = pow(obj_qj, 2);
	const GEN_FLT x15 = pow(obj_qi, 2);
	const GEN_FLT x16 = 1e-10 + x15 + x14 + x13;
	const GEN_FLT x17 = pow(x16, -1);
	const GEN_FLT x18 = pow(x16, 1.0 / 2.0);
	const GEN_FLT x19 = cos(x18);
	const GEN_FLT x20 = 1 + (-1 * x19);
	const GEN_FLT x21 = x20 * x17;
	const GEN_FLT x22 = sin(x18);
	const GEN_FLT x23 = x22 * pow(x18, -1);
	const GEN_FLT x24 = x23 * obj_qi;
	const GEN_FLT x25 = x21 * obj_qk;
	const GEN_FLT x26 = x25 * obj_qj;
	const GEN_FLT x27 = x23 * obj_qj;
	const GEN_FLT x28 = -1 * x27;
	const GEN_FLT x29 = x25 * obj_qi;
	const GEN_FLT x30 = ((x29 + x28) * sensor_x) + ((x26 + x24) * sensor_y) + ((x19 + (x21 * x13)) * sensor_z) + obj_pz;
	const GEN_FLT x31 = -1 * x24;
	const GEN_FLT x32 = x23 * obj_qk;
	const GEN_FLT x33 = x21 * obj_qi;
	const GEN_FLT x34 = x33 * obj_qj;
	const GEN_FLT x35 = ((x19 + (x21 * x14)) * sensor_y) + ((x34 + x32) * sensor_x) + ((x26 + x31) * sensor_z) + obj_py;
	const GEN_FLT x36 = x5 * lh_qk;
	const GEN_FLT x37 = x8 * lh_qj * lh_qi;
	const GEN_FLT x38 = x37 + (-1 * x36);
	const GEN_FLT x39 = -1 * x32;
	const GEN_FLT x40 = ((x19 + (x21 * x15)) * sensor_x) + ((x34 + x39) * sensor_y) + ((x29 + x27) * sensor_z) + obj_px;
	const GEN_FLT x41 = x7 + (x2 * x8);
	const GEN_FLT x42 = (x40 * x41) + (x30 * x12) + (x35 * x38) + lh_px;
	const GEN_FLT x43 = x7 + (x0 * x8);
	const GEN_FLT x44 = x5 * lh_qi;
	const GEN_FLT x45 = x9 * lh_qj;
	const GEN_FLT x46 = x45 + x44;
	const GEN_FLT x47 = (x40 * x11) + (x46 * x35) + (x43 * x30) + lh_pz;
	const GEN_FLT x48 = pow(x47, 2);
	const GEN_FLT x49 = pow(x48, -1);
	const GEN_FLT x50 = x42 * x49;
	const GEN_FLT x51 = pow(x47, -1);
	const GEN_FLT x52 = pow(x42, 2);
	const GEN_FLT x53 = x48 + x52;
	const GEN_FLT x54 = pow(x53, -1);
	const GEN_FLT x55 = x54 * x48;
	const GEN_FLT x56 = ((-1 * x51 * x41) + (x50 * x11)) * x55;
	const GEN_FLT x57 = 2 * x42;
	const GEN_FLT x58 = 2 * x47;
	const GEN_FLT x59 = x58 * x11;
	const GEN_FLT x60 = x45 + (-1 * x44);
	const GEN_FLT x61 = x7 + (x1 * x8);
	const GEN_FLT x62 = x37 + x36;
	const GEN_FLT x63 = lh_py + (x61 * x35) + (x62 * x40) + (x60 * x30);
	const GEN_FLT x64 = 1.0 / 2.0 * x63 * pow(x53, -3.0 / 2.0) * tilt_0;
	const GEN_FLT x65 = pow(x53, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x66 = pow(x63, 2);
	const GEN_FLT x67 = pow((1 + (-1 * x66 * x54 * pow(tilt_0, 2))), -1.0 / 2.0);
	const GEN_FLT x68 = (-1 * x67 * ((x62 * x65) + (-1 * x64 * (x59 + (x57 * x41))))) + (-1 * x56);
	const GEN_FLT x69 = -1 * x47;
	const GEN_FLT x70 = atan2(x42, x69);
	const GEN_FLT x71 =
		sin(1.5707963267949 + (-1 * x70) + (-1 * phase_0) + (-1 * asin(x63 * x65)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x72 = x63 * x49;
	const GEN_FLT x73 = x72 * x11;
	const GEN_FLT x74 = x62 * x51;
	const GEN_FLT x75 = x48 + x66;
	const GEN_FLT x76 = pow(x75, -1);
	const GEN_FLT x77 = x76 * x48;
	const GEN_FLT x78 = 2 * x77 * atan2(x63, x69) * curve_0;
	const GEN_FLT x79 = ((-1 * x51 * x38) + (x50 * x46)) * x55;
	const GEN_FLT x80 = x58 * x46;
	const GEN_FLT x81 = (-1 * x67 * ((x61 * x65) + (-1 * x64 * (x80 + (x57 * x38))))) + (-1 * x79);
	const GEN_FLT x82 = x72 * x46;
	const GEN_FLT x83 = x61 * x51;
	const GEN_FLT x84 = ((-1 * x51 * x12) + (x50 * x43)) * x55;
	const GEN_FLT x85 = x58 * x43;
	const GEN_FLT x86 = (-1 * x67 * ((x60 * x65) + (-1 * x64 * (x85 + (x57 * x12))))) + (-1 * x84);
	const GEN_FLT x87 = x72 * x43;
	const GEN_FLT x88 = x60 * x51;
	const GEN_FLT x89 = pow(obj_qi, 3);
	const GEN_FLT x90 = x22 * pow(x16, -3.0 / 2.0);
	const GEN_FLT x91 = 2 * x20 * pow(x16, -2);
	const GEN_FLT x92 = x21 * obj_qj;
	const GEN_FLT x93 = x90 * x15;
	const GEN_FLT x94 = x91 * x15;
	const GEN_FLT x95 = (-1 * x94 * obj_qj) + (x93 * obj_qj);
	const GEN_FLT x96 = x95 + x92;
	const GEN_FLT x97 = x19 * x17;
	const GEN_FLT x98 = obj_qk * obj_qi;
	const GEN_FLT x99 = x98 * x97;
	const GEN_FLT x100 = x90 * x98;
	const GEN_FLT x101 = x100 + (-1 * x99);
	const GEN_FLT x102 = (-1 * x94 * obj_qk) + (x93 * obj_qk);
	const GEN_FLT x103 = x102 + x25;
	const GEN_FLT x104 = obj_qj * obj_qi;
	const GEN_FLT x105 = x97 * x104;
	const GEN_FLT x106 = x90 * x104;
	const GEN_FLT x107 = (-1 * x106) + x105;
	const GEN_FLT x108 = ((x107 + x103) * sensor_z) + ((x101 + x96) * sensor_y) +
						 (((-1 * x89 * x91) + (x89 * x90) + (2 * x33) + x31) * sensor_x);
	const GEN_FLT x109 = (-1 * x100) + x99;
	const GEN_FLT x110 = x90 * x14;
	const GEN_FLT x111 = x91 * obj_qi;
	const GEN_FLT x112 = (-1 * x14 * x111) + (x110 * obj_qi);
	const GEN_FLT x113 = x97 * x15;
	const GEN_FLT x114 = obj_qk * obj_qj;
	const GEN_FLT x115 = x90 * x114;
	const GEN_FLT x116 = (-1 * x111 * x114) + (x115 * obj_qi);
	const GEN_FLT x117 = x116 + (-1 * x23);
	const GEN_FLT x118 =
		((x117 + x93 + (-1 * x113)) * sensor_z) + ((x112 + x31) * sensor_y) + ((x109 + x96) * sensor_x);
	const GEN_FLT x119 = x106 + (-1 * x105);
	const GEN_FLT x120 = x116 + x23;
	const GEN_FLT x121 = x90 * x13;
	const GEN_FLT x122 = (-1 * x13 * x111) + (x121 * obj_qi);
	const GEN_FLT x123 =
		((x120 + (-1 * x93) + x113) * sensor_y) + ((x122 + x31) * sensor_z) + ((x119 + x103) * sensor_x);
	const GEN_FLT x124 = (x43 * x123) + (x46 * x118) + (x11 * x108);
	const GEN_FLT x125 = (x12 * x123) + (x38 * x118) + (x41 * x108);
	const GEN_FLT x126 = ((-1 * x51 * x125) + (x50 * x124)) * x55;
	const GEN_FLT x127 = x58 * x124;
	const GEN_FLT x128 = (x60 * x123) + (x61 * x118) + (x62 * x108);
	const GEN_FLT x129 = (-1 * x67 * ((x65 * x128) + (-1 * x64 * (x127 + (x57 * x125))))) + (-1 * x126);
	const GEN_FLT x130 = x72 * x124;
	const GEN_FLT x131 = x51 * x128;
	const GEN_FLT x132 = x97 * x114;
	const GEN_FLT x133 = (-1 * x115) + x132;
	const GEN_FLT x134 = x112 + x33;
	const GEN_FLT x135 = pow(obj_qj, 3);
	const GEN_FLT x136 = (-1 * x91 * x14 * obj_qk) + (x110 * obj_qk);
	const GEN_FLT x137 = x136 + x25;
	const GEN_FLT x138 = ((x137 + x119) * sensor_z) +
						 (((-1 * x91 * x135) + (x90 * x135) + (2 * x92) + x28) * sensor_y) + ((x134 + x133) * sensor_x);
	const GEN_FLT x139 = x115 + (-1 * x132);
	const GEN_FLT x140 = x97 * x14;
	const GEN_FLT x141 =
		((x120 + (-1 * x110) + x140) * sensor_z) + ((x134 + x139) * sensor_y) + ((x95 + x28) * sensor_x);
	const GEN_FLT x142 = (-1 * x91 * x13 * obj_qj) + (x121 * obj_qj);
	const GEN_FLT x143 =
		((x142 + x28) * sensor_z) + ((x107 + x137) * sensor_y) + ((x117 + x110 + (-1 * x140)) * sensor_x);
	const GEN_FLT x144 = (x43 * x143) + (x11 * x141) + (x46 * x138);
	const GEN_FLT x145 = (x12 * x143) + (x38 * x138) + (x41 * x141);
	const GEN_FLT x146 = ((-1 * x51 * x145) + (x50 * x144)) * x55;
	const GEN_FLT x147 = x58 * x144;
	const GEN_FLT x148 = (x60 * x143) + (x61 * x138) + (x62 * x141);
	const GEN_FLT x149 = (-1 * x67 * ((x65 * x148) + (-1 * x64 * (x147 + (x57 * x145))))) + (-1 * x146);
	const GEN_FLT x150 = x72 * x144;
	const GEN_FLT x151 = x51 * x148;
	const GEN_FLT x152 = x97 * x13;
	const GEN_FLT x153 = x122 + x33;
	const GEN_FLT x154 =
		((x133 + x153) * sensor_z) + ((x121 + x117 + (-1 * x152)) * sensor_y) + ((x102 + x39) * sensor_x);
	const GEN_FLT x155 = x142 + x92;
	const GEN_FLT x156 =
		((x155 + x101) * sensor_z) + ((x136 + x39) * sensor_y) + (((-1 * x121) + x120 + x152) * sensor_x);
	const GEN_FLT x157 = pow(obj_qk, 3);
	const GEN_FLT x158 = (((-1 * x91 * x157) + (x90 * x157) + x39 + (2 * x25)) * sensor_z) +
						 ((x155 + x109) * sensor_y) + ((x139 + x153) * sensor_x);
	const GEN_FLT x159 = (x43 * x158) + (x46 * x156) + (x11 * x154);
	const GEN_FLT x160 = (x12 * x158) + (x38 * x156) + (x41 * x154);
	const GEN_FLT x161 = ((-1 * x51 * x160) + (x50 * x159)) * x55;
	const GEN_FLT x162 = x58 * x159;
	const GEN_FLT x163 = (x60 * x158) + (x61 * x156) + (x62 * x154);
	const GEN_FLT x164 = (-1 * x67 * ((x65 * x163) + (-1 * x64 * (x162 + (x57 * x160))))) + (-1 * x161);
	const GEN_FLT x165 = x72 * x159;
	const GEN_FLT x166 = x51 * x163;
	const GEN_FLT x167 = pow((1 + (-1 * x76 * x52 * pow(tilt_1, 2))), -1.0 / 2.0);
	const GEN_FLT x168 = 2 * x63;
	const GEN_FLT x169 = 1.0 / 2.0 * pow(x75, -3.0 / 2.0) * x42 * tilt_1;
	const GEN_FLT x170 = pow(x75, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x171 =
		(-1 * x167 * ((x41 * x170) + (-1 * x169 * (x59 + (x62 * x168))))) + (-1 * (x74 + (-1 * x73)) * x77);
	const GEN_FLT x172 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x63, x69)) + (-1 * phase_1) + (-1 * asin(x42 * x170)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x173 = 2 * x70 * curve_1;
	const GEN_FLT x174 =
		(-1 * x167 * ((x38 * x170) + (-1 * x169 * (x80 + (x61 * x168))))) + (-1 * (x83 + (-1 * x82)) * x77);
	const GEN_FLT x175 =
		(-1 * x167 * ((x12 * x170) + (-1 * x169 * (x85 + (x60 * x168))))) + (-1 * (x88 + (-1 * x87)) * x77);
	const GEN_FLT x176 =
		(-1 * x167 * ((x125 * x170) + (-1 * x169 * (x127 + (x128 * x168))))) + (-1 * (x131 + (-1 * x130)) * x77);
	const GEN_FLT x177 =
		(-1 * x167 * ((x170 * x145) + (-1 * x169 * (x147 + (x168 * x148))))) + (-1 * (x151 + (-1 * x150)) * x77);
	const GEN_FLT x178 =
		(-1 * x167 * ((x160 * x170) + (-1 * x169 * (x162 + (x168 * x163))))) + (-1 * (x166 + (-1 * x165)) * x77);
	out[0] = (((-1 * x74) + x73) * x78) + x68 + (x71 * x68);
	out[1] = x81 + (((-1 * x83) + x82) * x78) + (x81 * x71);
	out[2] = (((-1 * x88) + x87) * x78) + x86 + (x86 * x71);
	out[3] = x129 + (((-1 * x131) + x130) * x78) + (x71 * x129);
	out[4] = x149 + (((-1 * x151) + x150) * x78) + (x71 * x149);
	out[5] = x164 + (((-1 * x166) + x165) * x78) + (x71 * x164);
	out[6] = x171 + (x56 * x173) + (x171 * x172);
	out[7] = x174 + (x79 * x173) + (x174 * x172);
	out[8] = x175 + (x84 * x173) + (x175 * x172);
	out[9] = x176 + (x126 * x173) + (x176 * x172);
	out[10] = x177 + (x173 * x146) + (x177 * x172);
	out[11] = x178 + (x161 * x173) + (x178 * x172);
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qj;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qk;
	const GEN_FLT x10 = x9 * lh_qi;
	const GEN_FLT x11 = x10 + (-1 * x6);
	const GEN_FLT x12 = pow(obj_qi, 2);
	const GEN_FLT x13 = pow(obj_qk, 2);
	const GEN_FLT x14 = pow(obj_qj, 2);
	const GEN_FLT x15 = 1e-10 + x12 + x14 + x13;
	const GEN_FLT x16 = pow(x15, 1.0 / 2.0);
	const GEN_FLT x17 = cos(x16);
	const GEN_FLT x18 = pow(x15, -1) * (1 + (-1 * x17));
	const GEN_FLT x19 = x17 + (x12 * x18);
	const GEN_FLT x20 = pow(x16, -1) * sin(x16);
	const GEN_FLT x21 = x20 * obj_qk;
	const GEN_FLT x22 = x18 * obj_qj * obj_qi;
	const GEN_FLT x23 = x22 + x21;
	const GEN_FLT x24 = x5 * lh_qi;
	const GEN_FLT x25 = x9 * lh_qj;
	const GEN_FLT x26 = x25 + x24;
	const GEN_FLT x27 = x20 * obj_qj;
	const GEN_FLT x28 = x18 * obj_qk;
	const GEN_FLT x29 = x28 * obj_qi;
	const GEN_FLT x30 = x29 + (-1 * x27);
	const GEN_FLT x31 = x7 + (x0 * x8);
	const GEN_FLT x32 = (x30 * x31) + (x23 * x26) + (x11 * x19);
	const GEN_FLT x33 = x17 + (x13 * x18);
	const GEN_FLT x34 = x20 * obj_qi;
	const GEN_FLT x35 = x28 * obj_qj;
	const GEN_FLT x36 = x35 + x34;
	const GEN_FLT x37 = (x30 * sensor_x) + (x33 * sensor_z) + (x36 * sensor_y) + obj_pz;
	const GEN_FLT x38 = x35 + (-1 * x34);
	const GEN_FLT x39 = x17 + (x14 * x18);
	const GEN_FLT x40 = (x39 * sensor_y) + (x23 * sensor_x) + (x38 * sensor_z) + obj_py;
	const GEN_FLT x41 = x29 + x27;
	const GEN_FLT x42 = x22 + (-1 * x21);
	const GEN_FLT x43 = (x19 * sensor_x) + (x41 * sensor_z) + (x42 * sensor_y) + obj_px;
	const GEN_FLT x44 = (x43 * x11) + (x31 * x37) + (x40 * x26) + lh_pz;
	const GEN_FLT x45 = pow(x44, 2);
	const GEN_FLT x46 = pow(x45, -1);
	const GEN_FLT x47 = x10 + x6;
	const GEN_FLT x48 = x5 * lh_qk;
	const GEN_FLT x49 = x8 * lh_qj * lh_qi;
	const GEN_FLT x50 = x49 + (-1 * x48);
	const GEN_FLT x51 = x7 + (x2 * x8);
	const GEN_FLT x52 = (x50 * x40) + (x51 * x43) + (x47 * x37) + lh_px;
	const GEN_FLT x53 = x52 * x46;
	const GEN_FLT x54 = (x47 * x30) + (x50 * x23) + (x51 * x19);
	const GEN_FLT x55 = pow(x44, -1);
	const GEN_FLT x56 = pow(x52, 2);
	const GEN_FLT x57 = x45 + x56;
	const GEN_FLT x58 = pow(x57, -1);
	const GEN_FLT x59 = x58 * x45;
	const GEN_FLT x60 = ((-1 * x54 * x55) + (x53 * x32)) * x59;
	const GEN_FLT x61 = x25 + (-1 * x24);
	const GEN_FLT x62 = x7 + (x1 * x8);
	const GEN_FLT x63 = x49 + x48;
	const GEN_FLT x64 = (x63 * x43) + lh_py + (x62 * x40) + (x61 * x37);
	const GEN_FLT x65 = pow(x64, 2);
	const GEN_FLT x66 = pow((1 + (-1 * x65 * x58 * pow(tilt_0, 2))), -1.0 / 2.0);
	const GEN_FLT x67 = 2 * x52;
	const GEN_FLT x68 = 2 * x44;
	const GEN_FLT x69 = x68 * x32;
	const GEN_FLT x70 = 1.0 / 2.0 * x64 * pow(x57, -3.0 / 2.0) * tilt_0;
	const GEN_FLT x71 = (x61 * x30) + (x62 * x23) + (x63 * x19);
	const GEN_FLT x72 = pow(x57, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x73 = (-1 * x66 * ((x71 * x72) + (-1 * x70 * (x69 + (x67 * x54))))) + (-1 * x60);
	const GEN_FLT x74 = -1 * x44;
	const GEN_FLT x75 = atan2(x52, x74);
	const GEN_FLT x76 =
		sin(1.5707963267949 + (-1 * x75) + (-1 * phase_0) + (-1 * asin(x72 * x64)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x77 = x64 * x46;
	const GEN_FLT x78 = x77 * x32;
	const GEN_FLT x79 = x71 * x55;
	const GEN_FLT x80 = x45 + x65;
	const GEN_FLT x81 = pow(x80, -1);
	const GEN_FLT x82 = x81 * x45;
	const GEN_FLT x83 = 2 * x82 * atan2(x64, x74) * curve_0;
	const GEN_FLT x84 = (x31 * x36) + (x39 * x26) + (x42 * x11);
	const GEN_FLT x85 = (x47 * x36) + (x50 * x39) + (x51 * x42);
	const GEN_FLT x86 = ((-1 * x85 * x55) + (x84 * x53)) * x59;
	const GEN_FLT x87 = x84 * x68;
	const GEN_FLT x88 = (x61 * x36) + (x62 * x39) + (x63 * x42);
	const GEN_FLT x89 = (-1 * x66 * ((x88 * x72) + (-1 * x70 * (x87 + (x85 * x67))))) + (-1 * x86);
	const GEN_FLT x90 = x84 * x77;
	const GEN_FLT x91 = x88 * x55;
	const GEN_FLT x92 = (x31 * x33) + (x38 * x26) + (x41 * x11);
	const GEN_FLT x93 = (x50 * x38) + (x47 * x33) + (x51 * x41);
	const GEN_FLT x94 = ((-1 * x55 * x93) + (x53 * x92)) * x59;
	const GEN_FLT x95 = x68 * x92;
	const GEN_FLT x96 = (x61 * x33) + (x62 * x38) + (x63 * x41);
	const GEN_FLT x97 = (-1 * x66 * ((x72 * x96) + (-1 * x70 * (x95 + (x67 * x93))))) + (-1 * x94);
	const GEN_FLT x98 = x77 * x92;
	const GEN_FLT x99 = x55 * x96;
	const GEN_FLT x100 = pow((1 + (-1 * x81 * x56 * pow(tilt_1, 2))), -1.0 / 2.0);
	const GEN_FLT x101 = 2 * x64;
	const GEN_FLT x102 = 1.0 / 2.0 * pow(x80, -3.0 / 2.0) * x52 * tilt_1;
	const GEN_FLT x103 = pow(x80, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x104 =
		(-1 * x100 * ((x54 * x103) + (-1 * x102 * (x69 + (x71 * x101))))) + (-1 * (x79 + (-1 * x78)) * x82);
	const GEN_FLT x105 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x64, x74)) + (-1 * phase_1) + (-1 * asin(x52 * x103)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x106 = 2 * x75 * curve_1;
	const GEN_FLT x107 =
		(-1 * x100 * ((x85 * x103) + (-1 * x102 * (x87 + (x88 * x101))))) + (-1 * (x91 + (-1 * x90)) * x82);
	const GEN_FLT x108 =
		(-1 * x100 * ((x93 * x103) + (-1 * x102 * (x95 + (x96 * x101))))) + (-1 * (x99 + (-1 * x98)) * x82);
	out[0] = x73 + (((-1 * x79) + x78) * x83) + (x73 * x76);
	out[1] = x89 + (((-1 * x91) + x90) * x83) + (x89 * x76);
	out[2] = x97 + (((-1 * x99) + x98) * x83) + (x76 * x97);
	out[3] = x104 + (x60 * x106) + (x105 * x104);
	out[4] = x107 + (x86 * x106) + (x105 * x107);
	out[5] = x108 + (x94 * x106) + (x108 * x105);
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = sin(x4);
	const GEN_FLT x6 = pow(x4, -1) * x5;
	const GEN_FLT x7 = x6 * lh_qj;
	const GEN_FLT x8 = cos(x4);
	const GEN_FLT x9 = 1 + (-1 * x8);
	const GEN_FLT x10 = pow(x3, -1);
	const GEN_FLT x11 = x9 * x10;
	const GEN_FLT x12 = x11 * lh_qi;
	const GEN_FLT x13 = x12 * lh_qk;
	const GEN_FLT x14 = pow(obj_qk, 2);
	const GEN_FLT x15 = pow(obj_qj, 2);
	const GEN_FLT x16 = pow(obj_qi, 2);
	const GEN_FLT x17 = 1e-10 + x16 + x15 + x14;
	const GEN_FLT x18 = pow(x17, 1.0 / 2.0);
	const GEN_FLT x19 = cos(x18);
	const GEN_FLT x20 = pow(x17, -1) * (1 + (-1 * x19));
	const GEN_FLT x21 = pow(x18, -1) * sin(x18);
	const GEN_FLT x22 = x21 * obj_qi;
	const GEN_FLT x23 = x20 * obj_qk * obj_qj;
	const GEN_FLT x24 = x21 * obj_qj;
	const GEN_FLT x25 = x20 * obj_qi;
	const GEN_FLT x26 = x25 * obj_qk;
	const GEN_FLT x27 =
		((x26 + (-1 * x24)) * sensor_x) + ((x23 + x22) * sensor_y) + ((x19 + (x20 * x14)) * sensor_z) + obj_pz;
	const GEN_FLT x28 = x21 * obj_qk;
	const GEN_FLT x29 = x25 * obj_qj;
	const GEN_FLT x30 =
		((x29 + x28) * sensor_x) + ((x19 + (x20 * x15)) * sensor_y) + ((x23 + (-1 * x22)) * sensor_z) + obj_py;
	const GEN_FLT x31 = x6 * lh_qk;
	const GEN_FLT x32 = -1 * x31;
	const GEN_FLT x33 = x11 * lh_qj;
	const GEN_FLT x34 = x33 * lh_qi;
	const GEN_FLT x35 =
		((x19 + (x20 * x16)) * sensor_x) + ((x29 + (-1 * x28)) * sensor_y) + ((x26 + x24) * sensor_z) + obj_px;
	const GEN_FLT x36 = (x35 * (x8 + (x2 * x11))) + ((x34 + x32) * x30) + (x27 * (x13 + x7)) + lh_px;
	const GEN_FLT x37 = pow(x36, 2);
	const GEN_FLT x38 = x6 * lh_qi;
	const GEN_FLT x39 = x33 * lh_qk;
	const GEN_FLT x40 = -1 * x7;
	const GEN_FLT x41 = ((x39 + x38) * x30) + (x27 * (x8 + (x0 * x11))) + ((x13 + x40) * x35) + lh_pz;
	const GEN_FLT x42 = pow(x41, 2);
	const GEN_FLT x43 = x42 + x37;
	const GEN_FLT x44 = pow(x43, -1);
	const GEN_FLT x45 = x41 * x44;
	const GEN_FLT x46 = -1 * x38;
	const GEN_FLT x47 = ((x34 + x31) * x35) + lh_py + (x30 * (x8 + (x1 * x11))) + ((x39 + x46) * x27);
	const GEN_FLT x48 = pow(x47, 2);
	const GEN_FLT x49 = pow((1 + (-1 * x44 * x48 * pow(tilt_0, 2))), -1.0 / 2.0);
	const GEN_FLT x50 = pow(x43, -3.0 / 2.0) * x47 * tilt_0;
	const GEN_FLT x51 = x50 * x49;
	const GEN_FLT x52 = (x51 * x36) + x45;
	const GEN_FLT x53 = pow(x43, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x54 = -1 * x41;
	const GEN_FLT x55 = atan2(x36, x54);
	const GEN_FLT x56 =
		sin(1.5707963267949 + (-1 * x55) + (-1 * phase_0) + (-1 * asin(x53 * x47)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x57 = x53 * x49;
	const GEN_FLT x58 = x42 + x48;
	const GEN_FLT x59 = pow(x58, -1);
	const GEN_FLT x60 = x59 * x41;
	const GEN_FLT x61 = 2 * atan2(x47, x54) * curve_0;
	const GEN_FLT x62 = x44 * x36;
	const GEN_FLT x63 = (x51 * x41) + (-1 * x62);
	const GEN_FLT x64 = x59 * x47;
	const GEN_FLT x65 = x11 * lh_qk;
	const GEN_FLT x66 = pow(x3, -3.0 / 2.0) * x5;
	const GEN_FLT x67 = x2 * x66;
	const GEN_FLT x68 = 2 * pow(x3, -2) * x9;
	const GEN_FLT x69 = x2 * x68;
	const GEN_FLT x70 = (-1 * x69 * lh_qk) + (x67 * lh_qk);
	const GEN_FLT x71 = x70 + x65;
	const GEN_FLT x72 = lh_qj * lh_qi;
	const GEN_FLT x73 = x72 * x66;
	const GEN_FLT x74 = x8 * x10;
	const GEN_FLT x75 = x72 * x74;
	const GEN_FLT x76 = (-1 * x75) + x73;
	const GEN_FLT x77 = x2 * x74;
	const GEN_FLT x78 = x66 * lh_qk;
	const GEN_FLT x79 = x78 * lh_qi;
	const GEN_FLT x80 = (-1 * x72 * x68 * lh_qk) + (x79 * lh_qj);
	const GEN_FLT x81 = x80 + x6;
	const GEN_FLT x82 = x0 * x66;
	const GEN_FLT x83 = x0 * x68;
	const GEN_FLT x84 = (-1 * x83 * lh_qi) + (x82 * lh_qi);
	const GEN_FLT x85 = ((x84 + x46) * x27) + (x30 * (x81 + x77 + (-1 * x67))) + ((x76 + x71) * x35);
	const GEN_FLT x86 = pow(x42, -1);
	const GEN_FLT x87 = x86 * x36;
	const GEN_FLT x88 = pow(lh_qi, 3);
	const GEN_FLT x89 = (-1 * x69 * lh_qj) + (x67 * lh_qj);
	const GEN_FLT x90 = x89 + x33;
	const GEN_FLT x91 = x74 * lh_qk;
	const GEN_FLT x92 = x91 * lh_qi;
	const GEN_FLT x93 = (-1 * x92) + x79;
	const GEN_FLT x94 = x75 + (-1 * x73);
	const GEN_FLT x95 =
		((x94 + x71) * x27) + ((x93 + x90) * x30) + (((-1 * x88 * x68) + (x88 * x66) + (2 * x12) + x46) * x35);
	const GEN_FLT x96 = pow(x41, -1);
	const GEN_FLT x97 = x42 * x44;
	const GEN_FLT x98 = ((-1 * x96 * x95) + (x85 * x87)) * x97;
	const GEN_FLT x99 = 2 * x36;
	const GEN_FLT x100 = 2 * x41;
	const GEN_FLT x101 = x85 * x100;
	const GEN_FLT x102 = 1.0 / 2.0 * x50;
	const GEN_FLT x103 = x92 + (-1 * x79);
	const GEN_FLT x104 = x1 * x66;
	const GEN_FLT x105 = x1 * x68;
	const GEN_FLT x106 = (-1 * x105 * lh_qi) + (x104 * lh_qi);
	const GEN_FLT x107 = x80 + (-1 * x6);
	const GEN_FLT x108 = (x27 * (x107 + (-1 * x77) + x67)) + (x30 * (x106 + x46)) + (x35 * (x103 + x90));
	const GEN_FLT x109 = (-1 * x49 * ((x53 * x108) + (-1 * x102 * (x101 + (x99 * x95))))) + (-1 * x98);
	const GEN_FLT x110 = x86 * x47;
	const GEN_FLT x111 = x85 * x110;
	const GEN_FLT x112 = x96 * x108;
	const GEN_FLT x113 = x59 * x42;
	const GEN_FLT x114 = x61 * x113;
	const GEN_FLT x115 = x1 * x74;
	const GEN_FLT x116 = (-1 * x105 * lh_qk) + (x104 * lh_qk);
	const GEN_FLT x117 = x116 + x65;
	const GEN_FLT x118 = (-1 * x83 * lh_qj) + (x82 * lh_qj);
	const GEN_FLT x119 = (x30 * (x117 + x94)) + (x27 * (x118 + x40)) + (x35 * (x107 + (-1 * x115) + x104));
	const GEN_FLT x120 = x106 + x12;
	const GEN_FLT x121 = x78 * lh_qj;
	const GEN_FLT x122 = x91 * lh_qj;
	const GEN_FLT x123 = (-1 * x122) + x121;
	const GEN_FLT x124 = ((x123 + x120) * x30) + (x27 * (x81 + x115 + (-1 * x104))) + ((x89 + x40) * x35);
	const GEN_FLT x125 = ((-1 * x96 * x124) + (x87 * x119)) * x97;
	const GEN_FLT x126 = x100 * x119;
	const GEN_FLT x127 = pow(lh_qj, 3);
	const GEN_FLT x128 = x122 + (-1 * x121);
	const GEN_FLT x129 =
		(x27 * (x117 + x76)) + ((x128 + x120) * x35) + (((-1 * x68 * x127) + (x66 * x127) + (2 * x33) + x40) * x30);
	const GEN_FLT x130 = (-1 * x49 * ((x53 * x129) + (-1 * x102 * (x126 + (x99 * x124))))) + (-1 * x125);
	const GEN_FLT x131 = x110 * x119;
	const GEN_FLT x132 = x96 * x129;
	const GEN_FLT x133 = x84 + x12;
	const GEN_FLT x134 = x118 + x33;
	const GEN_FLT x135 = pow(lh_qk, 3);
	const GEN_FLT x136 =
		(((x66 * x135) + (-1 * x68 * x135) + (2 * x65) + x32) * x27) + ((x134 + x103) * x30) + ((x133 + x123) * x35);
	const GEN_FLT x137 = x0 * x74;
	const GEN_FLT x138 = ((x133 + x128) * x27) + (x30 * (x107 + (-1 * x137) + x82)) + ((x70 + x32) * x35);
	const GEN_FLT x139 = ((-1 * x96 * x138) + (x87 * x136)) * x97;
	const GEN_FLT x140 = x100 * x136;
	const GEN_FLT x141 = (x30 * (x116 + x32)) + (x27 * (x134 + x93)) + (x35 * (x81 + x137 + (-1 * x82)));
	const GEN_FLT x142 = (-1 * x49 * ((x53 * x141) + (-1 * x102 * (x140 + (x99 * x138))))) + (-1 * x139);
	const GEN_FLT x143 = x110 * x136;
	const GEN_FLT x144 = x96 * x141;
	const GEN_FLT x145 = pow((1 + (-1 * x59 * x37 * pow(tilt_1, 2))), -1.0 / 2.0);
	const GEN_FLT x146 = pow(x58, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x147 = x146 * x145;
	const GEN_FLT x148 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x47, x54)) + (-1 * phase_1) + (-1 * asin(x36 * x146)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x149 = 2 * x55 * curve_1;
	const GEN_FLT x150 = pow(x58, -3.0 / 2.0) * x36 * tilt_1;
	const GEN_FLT x151 = x145 * x150;
	const GEN_FLT x152 = (x47 * x151) + (-1 * x60);
	const GEN_FLT x153 = (x41 * x151) + x64;
	const GEN_FLT x154 = 2 * x47;
	const GEN_FLT x155 = 1.0 / 2.0 * x150;
	const GEN_FLT x156 =
		(-1 * x145 * ((x95 * x146) + (-1 * x155 * (x101 + (x108 * x154))))) + (-1 * (x112 + (-1 * x111)) * x113);
	const GEN_FLT x157 =
		(-1 * x145 * ((x124 * x146) + (-1 * x155 * (x126 + (x129 * x154))))) + (-1 * (x132 + (-1 * x131)) * x113);
	const GEN_FLT x158 =
		(-1 * x145 * ((x138 * x146) + (-1 * x155 * (x140 + (x141 * x154))))) + (-1 * (x144 + (-1 * x143)) * x113);
	out[0] = x52 + (x52 * x56);
	out[1] = (-1 * x57 * x56) + (-1 * x60 * x61) + (-1 * x57);
	out[2] = x63 + (x64 * x61) + (x63 * x56);
	out[3] = (((-1 * x112) + x111) * x114) + x109 + (x56 * x109);
	out[4] = x130 + (((-1 * x132) + x131) * x114) + (x56 * x130);
	out[5] = x142 + (((-1 * x144) + x143) * x114) + (x56 * x142);
	out[6] = (-1 * x45 * x149) + (-1 * x148 * x147) + (-1 * x147);
	out[7] = x152 + (x148 * x152);
	out[8] = (x62 * x149) + x153 + (x148 * x153);
	out[9] = x156 + (x98 * x149) + (x148 * x156);
	out[10] = x157 + (x125 * x149) + (x148 * x157);
	out[11] = x158 + (x139 * x149) + (x148 * x158);
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qi;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qj;
	const GEN_FLT x10 = x9 * lh_qk;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = pow(obj_qj, 2);
	const GEN_FLT x13 = pow(obj_qi, 2);
	const GEN_FLT x14 = 1e-10 + x13 + x12 + x11;
	const GEN_FLT x15 = pow(x14, 1.0 / 2.0);
	const GEN_FLT x16 = cos(x15);
	const GEN_FLT x17 = pow(x14, -1) * (1 + (-1 * x16));
	const GEN_FLT x18 = pow(x15, -1) * sin(x15);
	const GEN_FLT x19 = x18 * obj_qi;
	const GEN_FLT x20 = x17 * obj_qk;
	const GEN_FLT x21 = x20 * obj_qj;
	const GEN_FLT x22 = x18 * obj_qj;
	const GEN_FLT x23 = x20 * obj_qi;
	const GEN_FLT x24 =
		((x21 + x19) * sensor_y) + ((x23 + (-1 * x22)) * sensor_x) + ((x16 + (x11 * x17)) * sensor_z) + obj_pz;
	const GEN_FLT x25 = x18 * obj_qk;
	const GEN_FLT x26 = x17 * obj_qj * obj_qi;
	const GEN_FLT x27 =
		((x26 + x25) * sensor_x) + ((x16 + (x12 * x17)) * sensor_y) + ((x21 + (-1 * x19)) * sensor_z) + obj_py;
	const GEN_FLT x28 =
		((x26 + (-1 * x25)) * sensor_y) + ((x23 + x22) * sensor_z) + ((x16 + (x13 * x17)) * sensor_x) + obj_px;
	const GEN_FLT x29 = x5 * lh_qk;
	const GEN_FLT x30 = x9 * lh_qi;
	const GEN_FLT x31 = lh_py + ((x30 + x29) * x28) + (x27 * (x7 + (x1 * x8))) + (x24 * (x10 + (-1 * x6)));
	const GEN_FLT x32 = x5 * lh_qj;
	const GEN_FLT x33 = x8 * lh_qk * lh_qi;
	const GEN_FLT x34 = ((x33 + (-1 * x32)) * x28) + (x27 * (x10 + x6)) + (x24 * (x7 + (x0 * x8))) + lh_pz;
	const GEN_FLT x35 = pow(x34, 2);
	const GEN_FLT x36 = ((x30 + (-1 * x29)) * x27) + ((x33 + x32) * x24) + (x28 * (x7 + (x2 * x8))) + lh_px;
	const GEN_FLT x37 = pow(x36, 2);
	const GEN_FLT x38 = x37 + x35;
	const GEN_FLT x39 = x31 * pow(x38, -1.0 / 2.0);
	const GEN_FLT x40 = -1 * x34;
	const GEN_FLT x41 = atan2(x36, x40);
	const GEN_FLT x42 = 1.5707963267949 + (-1 * x41) + (-1 * phase_0) + (-1 * asin(x39 * tilt_0)) + gibPhase_0;
	const GEN_FLT x43 = sin(x42) * gibMag_0;
	const GEN_FLT x44 = pow(x31, 2);
	const GEN_FLT x45 = x39 * pow((1 + (-1 * x44 * pow(x38, -1) * pow(tilt_0, 2))), -1.0 / 2.0);
	const GEN_FLT x46 = x44 + x35;
	const GEN_FLT x47 = pow(x46, -1.0 / 2.0) * x36;
	const GEN_FLT x48 =
		1.5707963267949 + (-1 * phase_1) + (-1 * atan2(-1 * x31, x40)) + (-1 * asin(x47 * tilt_1)) + gibPhase_1;
	const GEN_FLT x49 = sin(x48) * gibMag_1;
	const GEN_FLT x50 = x47 * pow((1 + (-1 * pow(x46, -1) * x37 * pow(tilt_1, 2))), -1.0 / 2.0);
	out[0] = -1 + (-1 * x43);
	out[1] = (-1 * x43 * x45) + (-1 * x45);
	out[2] = pow(atan2(x31, x40), 2);
	out[3] = x43;
	out[4] = -1 * cos(x42);
	out[5] = 0;
	out[6] = 0;
	out[7] = 0;
	out[8] = 0;
	out[9] = 0;
	out[10] = 0;
	out[11] = 0;
	out[12] = 0;
	out[13] = 0;
	out[14] = 0;
	out[15] = 0;
	out[16] = 0;
	out[17] = 0;
	out[18] = 0;
	out[19] = 0;
	out[20] = 0;
	out[21] = -1 + (-1 * x49);
	out[22] = (-1 * x50 * x49) + (-1 * x50);
	out[23] = pow(x41, 2);
	out[24] = x49;
	out[25] = -1 * cos(x48);
	out[26] = 0;
	out[27] = 0;
}

static inline FLT gen_reproject_axis_x_axis_angle(const LinmathAxisAnglePose *obj_p, const FLT *sensor_pt,
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qi;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qj;
	const GEN_FLT x10 = x9 * lh_qk;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = pow(obj_qj, 2);
	const GEN_FLT x13 = pow(obj_qi, 2);
	const GEN_FLT x14 = 1e-10 + x13 + x12 + x11;
	const GEN_FLT x15 = pow(x14, 1.0 / 2.0);
	const GEN_FLT x16 = cos(x15);
	const GEN_FLT x17 = pow(x14, -1) * (1 + (-1 * x16));
	const GEN_FLT x18 = pow(x15, -1) * sin(x15);
	const GEN_FLT x19 = x18 * obj_qi;
	const GEN_FLT x20 = x17 * obj_qk;
	const GEN_FLT x21 = x20 * obj_qj;
	const GEN_FLT x22 = x18 * obj_qj;
	const GEN_FLT x23 = x20 * obj_qi;
	const GEN_FLT x24 =
		((x21 + x19) * sensor_y) + ((x23 + (-1 * x22)) * sensor_x) + ((x16 + (x11 * x17)) * sensor_z) + obj_pz;
	const GEN_FLT x25 = x18 * obj_qk;
	const GEN_FLT x26 = x17 * obj_qj * obj_qi;
	const GEN_FLT x27 =
		((x26 + x25) * sensor_x) + ((x16 + (x12 * x17)) * sensor_y) + ((x21 + (-1 * x19)) * sensor_z) + obj_py;
	const GEN_FLT x28 =
		((x26 + (-1 * x25)) * sensor_y) + ((x23 + x22) * sensor_z) + ((x16 + (x13 * x17)) * sensor_x) + obj_px;
	const GEN_FLT x29 = x5 * lh_qk;
	const GEN_FLT x30 = x9 * lh_qi;
	const GEN_FLT x31 = lh_py + ((x30 + x29) * x28) + (x27 * (x7 + (x1 * x8))) + (x24 * (x10 + (-1 * x6)));
	const GEN_FLT x32 = x5 * lh_qj;
	const GEN_FLT x33 = x8 * lh_qk * lh_qi;
	const GEN_FLT x34 = ((x33 + (-1 * x32)) * x28) + (x27 * (x10 + x6)) + (x24 * (x7 + (x0 * x8))) + lh_pz;
	const GEN_FLT x35 = -1 * x34;
	const GEN_FLT x36 = ((x30 + (-1 * x29)) * x27) + ((x33 + x32) * x24) + (x28 * (x7 + (x2 * x8))) + lh_px;
	const GEN_FLT x37 = (-1 * atan2(x36, x35)) + (-1 * phase_0) +
						(-1 * asin(pow((pow(x36, 2) + pow(x34, 2)), -1.0 / 2.0) * x31 * tilt_0));
	return x37 + (-1 * cos(1.5707963267949 + x37 + gibPhase_0) * gibMag_0) + (pow(atan2(x31, x35), 2) * curve_0);
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qj;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qk * lh_qi;
	const GEN_FLT x10 = x9 + (-1 * x6);
	const GEN_FLT x11 = x9 + x6;
	const GEN_FLT x12 = pow(obj_qk, 2);
	const GEN_FLT x13 = pow(obj_qj, 2);
	const GEN_FLT x14 = pow(obj_qi, 2);
	const GEN_FLT x15 = 1e-10 + x14 + x13 + x12;
	const GEN_FLT x16 = pow(x15, -1);
	const GEN_FLT x17 = pow(x15, 1.0 / 2.0);
	const GEN_FLT x18 = cos(x17);
	const GEN_FLT x19 = 1 + (-1 * x18);
	const GEN_FLT x20 = x19 * x16;
	const GEN_FLT x21 = sin(x17);
	const GEN_FLT x22 = x21 * pow(x17, -1);
	const GEN_FLT x23 = x22 * obj_qi;
	const GEN_FLT x24 = x20 * obj_qj;
	const GEN_FLT x25 = x24 * obj_qk;
	const GEN_FLT x26 = x22 * obj_qj;
	const GEN_FLT x27 = -1 * x26;
	const GEN_FLT x28 = x20 * obj_qi;
	const GEN_FLT x29 = x28 * obj_qk;
	const GEN_FLT x30 = ((x29 + x27) * sensor_x) + ((x25 + x23) * sensor_y) + ((x18 + (x20 * x12)) * sensor_z) + obj_pz;
	const GEN_FLT x31 = -1 * x23;
	const GEN_FLT x32 = x22 * obj_qk;
	const GEN_FLT x33 = x24 * obj_qi;
	const GEN_FLT x34 = ((x33 + x32) * sensor_x) + ((x25 + x31) * sensor_z) + ((x18 + (x20 * x13)) * sensor_y) + obj_py;
	const GEN_FLT x35 = x5 * lh_qk;
	const GEN_FLT x36 = x8 * lh_qj;
	const GEN_FLT x37 = x36 * lh_qi;
	const GEN_FLT x38 = x37 + (-1 * x35);
	const GEN_FLT x39 = -1 * x32;
	const GEN_FLT x40 = ((x18 + (x20 * x14)) * sensor_x) + ((x33 + x39) * sensor_y) + ((x29 + x26) * sensor_z) + obj_px;
	const GEN_FLT x41 = x7 + (x2 * x8);
	const GEN_FLT x42 = (x40 * x41) + (x30 * x11) + (x34 * x38) + lh_px;
	const GEN_FLT x43 = x7 + (x0 * x8);
	const GEN_FLT x44 = x5 * lh_qi;
	const GEN_FLT x45 = x36 * lh_qk;
	const GEN_FLT x46 = x45 + x44;
	const GEN_FLT x47 = (x40 * x10) + (x46 * x34) + (x43 * x30) + lh_pz;
	const GEN_FLT x48 = pow(x47, 2);
	const GEN_FLT x49 = pow(x48, -1);
	const GEN_FLT x50 = x42 * x49;
	const GEN_FLT x51 = pow(x47, -1);
	const GEN_FLT x52 = x48 + pow(x42, 2);
	const GEN_FLT x53 = pow(x52, -1);
	const GEN_FLT x54 = x53 * x48;
	const GEN_FLT x55 = 2 * x42;
	const GEN_FLT x56 = 2 * x47;
	const GEN_FLT x57 = x45 + (-1 * x44);
	const GEN_FLT x58 = x7 + (x1 * x8);
	const GEN_FLT x59 = x37 + x35;
	const GEN_FLT x60 = (x59 * x40) + lh_py + (x58 * x34) + (x57 * x30);
	const GEN_FLT x61 = 1.0 / 2.0 * x60 * pow(x52, -3.0 / 2.0) * tilt_0;
	const GEN_FLT x62 = pow(x52, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x63 = pow(x60, 2);
	const GEN_FLT x64 = pow((1 + (-1 * x63 * x53 * pow(tilt_0, 2))), -1.0 / 2.0);
	const GEN_FLT x65 = (-1 * x64 * ((x62 * x59) + (-1 * ((x56 * x10) + (x55 * x41)) * x61))) +
						(-1 * ((-1 * x51 * x41) + (x50 * x10)) * x54);
	const GEN_FLT x66 = -1 * x47;
	const GEN_FLT x67 =
		sin(1.5707963267949 + (-1 * phase_0) + (-1 * atan2(x42, x66)) + (-1 * asin(x60 * x62)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x68 = x60 * x49;
	const GEN_FLT x69 = 2 * pow((x48 + x63), -1) * x48 * atan2(x60, x66) * curve_0;
	const GEN_FLT x70 = (-1 * x64 * ((x62 * x58) + (-1 * ((x56 * x46) + (x55 * x38)) * x61))) +
						(-1 * ((-1 * x51 * x38) + (x50 * x46)) * x54);
	const GEN_FLT x71 = (-1 * x64 * ((x62 * x57) + (-1 * ((x56 * x43) + (x55 * x11)) * x61))) +
						(-1 * ((-1 * x51 * x11) + (x50 * x43)) * x54);
	const GEN_FLT x72 = pow(obj_qi, 3);
	const GEN_FLT x73 = x21 * pow(x15, -3.0 / 2.0);
	const GEN_FLT x74 = 2 * pow(x15, -2) * x19;
	const GEN_FLT x75 = x73 * x14;
	const GEN_FLT x76 = x74 * obj_qj;
	const GEN_FLT x77 = (-1 * x76 * x14) + (x75 * obj_qj);
	const GEN_FLT x78 = x77 + x24;
	const GEN_FLT x79 = x18 * x16;
	const GEN_FLT x80 = x79 * obj_qk;
	const GEN_FLT x81 = x80 * obj_qi;
	const GEN_FLT x82 = x73 * obj_qk;
	const GEN_FLT x83 = x82 * obj_qi;
	const GEN_FLT x84 = x83 + (-1 * x81);
	const GEN_FLT x85 = x20 * obj_qk;
	const GEN_FLT x86 = x74 * obj_qk;
	const GEN_FLT x87 = (-1 * x86 * x14) + (x75 * obj_qk);
	const GEN_FLT x88 = x87 + x85;
	const GEN_FLT x89 = obj_qj * obj_qi;
	const GEN_FLT x90 = x89 * x79;
	const GEN_FLT x91 = x89 * x73;
	const GEN_FLT x92 = (-1 * x91) + x90;
	const GEN_FLT x93 = ((x92 + x88) * sensor_z) + ((x84 + x78) * sensor_y) +
						(((-1 * x72 * x74) + (x73 * x72) + (2 * x28) + x31) * sensor_x);
	const GEN_FLT x94 = (-1 * x83) + x81;
	const GEN_FLT x95 = x73 * x13;
	const GEN_FLT x96 = x74 * obj_qi;
	const GEN_FLT x97 = (-1 * x96 * x13) + (x95 * obj_qi);
	const GEN_FLT x98 = x79 * x14;
	const GEN_FLT x99 = x82 * obj_qj;
	const GEN_FLT x100 = (-1 * x89 * x86) + (x99 * obj_qi);
	const GEN_FLT x101 = x100 + (-1 * x22);
	const GEN_FLT x102 = ((x97 + x31) * sensor_y) + ((x75 + x101 + (-1 * x98)) * sensor_z) + ((x94 + x78) * sensor_x);
	const GEN_FLT x103 = x91 + (-1 * x90);
	const GEN_FLT x104 = x100 + x22;
	const GEN_FLT x105 = x73 * x12;
	const GEN_FLT x106 = (-1 * x96 * x12) + (x105 * obj_qi);
	const GEN_FLT x107 = ((x106 + x31) * sensor_z) + ((x104 + (-1 * x75) + x98) * sensor_y) + ((x103 + x88) * sensor_x);
	const GEN_FLT x108 = (x43 * x107) + (x46 * x102) + (x93 * x10);
	const GEN_FLT x109 = (x11 * x107) + (x38 * x102) + (x93 * x41);
	const GEN_FLT x110 = (x57 * x107) + (x58 * x102) + (x59 * x93);
	const GEN_FLT x111 = (-1 * x64 * ((x62 * x110) + (-1 * ((x56 * x108) + (x55 * x109)) * x61))) +
						 (-1 * ((-1 * x51 * x109) + (x50 * x108)) * x54);
	const GEN_FLT x112 = x80 * obj_qj;
	const GEN_FLT x113 = (-1 * x99) + x112;
	const GEN_FLT x114 = x97 + x28;
	const GEN_FLT x115 = pow(obj_qj, 3);
	const GEN_FLT x116 = (-1 * x86 * x13) + (x95 * obj_qk);
	const GEN_FLT x117 = x116 + x85;
	const GEN_FLT x118 = ((x103 + x117) * sensor_z) +
						 (((-1 * x74 * x115) + (x73 * x115) + (2 * x24) + x27) * sensor_y) + ((x114 + x113) * sensor_x);
	const GEN_FLT x119 = x99 + (-1 * x112);
	const GEN_FLT x120 = x79 * x13;
	const GEN_FLT x121 =
		(((-1 * x95) + x104 + x120) * sensor_z) + ((x114 + x119) * sensor_y) + ((x77 + x27) * sensor_x);
	const GEN_FLT x122 = (-1 * x76 * x12) + (x105 * obj_qj);
	const GEN_FLT x123 =
		((x122 + x27) * sensor_z) + ((x117 + x92) * sensor_y) + ((x101 + x95 + (-1 * x120)) * sensor_x);
	const GEN_FLT x124 = (x43 * x123) + (x10 * x121) + (x46 * x118);
	const GEN_FLT x125 = x49 * x124;
	const GEN_FLT x126 = (x11 * x123) + (x38 * x118) + (x41 * x121);
	const GEN_FLT x127 = (x57 * x123) + (x58 * x118) + (x59 * x121);
	const GEN_FLT x128 = (-1 * x64 * ((x62 * x127) + (-1 * ((x56 * x124) + (x55 * x126)) * x61))) +
						 (-1 * ((-1 * x51 * x126) + (x42 * x125)) * x54);
	const GEN_FLT x129 = x79 * x12;
	const GEN_FLT x130 = x106 + x28;
	const GEN_FLT x131 =
		((x113 + x130) * sensor_z) + ((x101 + x105 + (-1 * x129)) * sensor_y) + ((x87 + x39) * sensor_x);
	const GEN_FLT x132 = x122 + x24;
	const GEN_FLT x133 =
		((x132 + x84) * sensor_z) + ((x116 + x39) * sensor_y) + ((x104 + (-1 * x105) + x129) * sensor_x);
	const GEN_FLT x134 = pow(obj_qk, 3);
	const GEN_FLT x135 = ((x39 + (x73 * x134) + (-1 * x74 * x134) + (2 * x85)) * sensor_z) + ((x132 + x94) * sensor_y) +
						 ((x119 + x130) * sensor_x);
	const GEN_FLT x136 = (x43 * x135) + (x46 * x133) + (x10 * x131);
	const GEN_FLT x137 = (x11 * x135) + (x38 * x133) + (x41 * x131);
	const GEN_FLT x138 = (x57 * x135) + (x58 * x133) + (x59 * x131);
	const GEN_FLT x139 = (-1 * x64 * ((x62 * x138) + (-1 * ((x56 * x136) + (x55 * x137)) * x61))) +
						 (-1 * ((-1 * x51 * x137) + (x50 * x136)) * x54);
	out[0] = x65 + (((-1 * x51 * x59) + (x68 * x10)) * x69) + (x67 * x65);
	out[1] = x70 + (((-1 * x51 * x58) + (x68 * x46)) * x69) + (x70 * x67);
	out[2] = (((-1 * x51 * x57) + (x68 * x43)) * x69) + x71 + (x71 * x67);
	out[3] = x111 + (((-1 * x51 * x110) + (x68 * x108)) * x69) + (x67 * x111);
	out[4] = x128 + (((-1 * x51 * x127) + (x60 * x125)) * x69) + (x67 * x128);
	out[5] = x139 + (((-1 * x51 * x138) + (x68 * x136)) * x69) + (x67 * x139);
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qj;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qi;
	const GEN_FLT x10 = x9 * lh_qk;
	const GEN_FLT x11 = x10 + (-1 * x6);
	const GEN_FLT x12 = pow(obj_qi, 2);
	const GEN_FLT x13 = pow(obj_qk, 2);
	const GEN_FLT x14 = pow(obj_qj, 2);
	const GEN_FLT x15 = 1e-10 + x12 + x14 + x13;
	const GEN_FLT x16 = pow(x15, 1.0 / 2.0);
	const GEN_FLT x17 = cos(x16);
	const GEN_FLT x18 = pow(x15, -1) * (1 + (-1 * x17));
	const GEN_FLT x19 = x17 + (x12 * x18);
	const GEN_FLT x20 = pow(x16, -1) * sin(x16);
	const GEN_FLT x21 = x20 * obj_qk;
	const GEN_FLT x22 = x18 * obj_qj * obj_qi;
	const GEN_FLT x23 = x22 + x21;
	const GEN_FLT x24 = x5 * lh_qi;
	const GEN_FLT x25 = x8 * lh_qk * lh_qj;
	const GEN_FLT x26 = x25 + x24;
	const GEN_FLT x27 = x20 * obj_qj;
	const GEN_FLT x28 = x18 * obj_qk;
	const GEN_FLT x29 = x28 * obj_qi;
	const GEN_FLT x30 = x29 + (-1 * x27);
	const GEN_FLT x31 = x7 + (x0 * x8);
	const GEN_FLT x32 = (x30 * x31) + (x23 * x26) + (x11 * x19);
	const GEN_FLT x33 = x10 + x6;
	const GEN_FLT x34 = x17 + (x13 * x18);
	const GEN_FLT x35 = x20 * obj_qi;
	const GEN_FLT x36 = x28 * obj_qj;
	const GEN_FLT x37 = x36 + x35;
	const GEN_FLT x38 = (x30 * sensor_x) + (x37 * sensor_y) + (x34 * sensor_z) + obj_pz;
	const GEN_FLT x39 = x36 + (-1 * x35);
	const GEN_FLT x40 = x17 + (x14 * x18);
	const GEN_FLT x41 = (x40 * sensor_y) + (x39 * sensor_z) + (x23 * sensor_x) + obj_py;
	const GEN_FLT x42 = x5 * lh_qk;
	const GEN_FLT x43 = x9 * lh_qj;
	const GEN_FLT x44 = x43 + (-1 * x42);
	const GEN_FLT x45 = x29 + x27;
	const GEN_FLT x46 = x22 + (-1 * x21);
	const GEN_FLT x47 = (x19 * sensor_x) + (x45 * sensor_z) + (x46 * sensor_y) + obj_px;
	const GEN_FLT x48 = x7 + (x2 * x8);
	const GEN_FLT x49 = (x47 * x48) + (x41 * x44) + (x33 * x38) + lh_px;
	const GEN_FLT x50 = (x47 * x11) + (x31 * x38) + (x41 * x26) + lh_pz;
	const GEN_FLT x51 = pow(x50, 2);
	const GEN_FLT x52 = pow(x51, -1);
	const GEN_FLT x53 = x52 * x49;
	const GEN_FLT x54 = (x30 * x33) + (x44 * x23) + (x48 * x19);
	const GEN_FLT x55 = pow(x50, -1);
	const GEN_FLT x56 = x51 + pow(x49, 2);
	const GEN_FLT x57 = pow(x56, -1);
	const GEN_FLT x58 = x51 * x57;
	const GEN_FLT x59 = x25 + (-1 * x24);
	const GEN_FLT x60 = x7 + (x1 * x8);
	const GEN_FLT x61 = x43 + x42;
	const GEN_FLT x62 = (x61 * x47) + lh_py + (x60 * x41) + (x59 * x38);
	const GEN_FLT x63 = pow(x62, 2);
	const GEN_FLT x64 = pow((1 + (-1 * x63 * x57 * pow(tilt_0, 2))), -1.0 / 2.0);
	const GEN_FLT x65 = 2 * x49;
	const GEN_FLT x66 = 2 * x50;
	const GEN_FLT x67 = 1.0 / 2.0 * x62 * pow(x56, -3.0 / 2.0) * tilt_0;
	const GEN_FLT x68 = (x59 * x30) + (x60 * x23) + (x61 * x19);
	const GEN_FLT x69 = pow(x56, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x70 = (-1 * x64 * ((x68 * x69) + (-1 * ((x66 * x32) + (x65 * x54)) * x67))) +
						(-1 * ((-1 * x54 * x55) + (x53 * x32)) * x58);
	const GEN_FLT x71 = -1 * x50;
	const GEN_FLT x72 =
		sin(1.5707963267949 + (-1 * atan2(x49, x71)) + (-1 * phase_0) + (-1 * asin(x62 * x69)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x73 = x62 * x52;
	const GEN_FLT x74 = 2 * pow((x51 + x63), -1) * x51 * atan2(x62, x71) * curve_0;
	const GEN_FLT x75 = (x40 * x26) + (x31 * x37) + (x46 * x11);
	const GEN_FLT x76 = (x33 * x37) + (x40 * x44) + (x46 * x48);
	const GEN_FLT x77 = (x59 * x37) + (x60 * x40) + (x61 * x46);
	const GEN_FLT x78 = (-1 * x64 * ((x77 * x69) + (-1 * ((x75 * x66) + (x76 * x65)) * x67))) +
						(-1 * ((-1 * x76 * x55) + (x75 * x53)) * x58);
	const GEN_FLT x79 = (x31 * x34) + (x39 * x26) + (x45 * x11);
	const GEN_FLT x80 = (x34 * x33) + (x44 * x39) + (x45 * x48);
	const GEN_FLT x81 = (x59 * x34) + (x60 * x39) + (x61 * x45);
	const GEN_FLT x82 = (-1 * x64 * ((x81 * x69) + (-1 * ((x79 * x66) + (x80 * x65)) * x67))) +
						(-1 * ((-1 * x80 * x55) + (x79 * x53)) * x58);
	out[0] = x70 + (((-1 * x68 * x55) + (x73 * x32)) * x74) + (x70 * x72);
	out[1] = x78 + (((-1 * x77 * x55) + (x73 * x75)) * x74) + (x72 * x78);
	out[2] = x82 + (((-1 * x81 * x55) + (x73 * x79)) * x74) + (x82 * x72);
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = sin(x4);
	const GEN_FLT x6 = pow(x4, -1) * x5;
	const GEN_FLT x7 = x6 * lh_qj;
	const GEN_FLT x8 = cos(x4);
	const GEN_FLT x9 = 1 + (-1 * x8);
	const GEN_FLT x10 = pow(x3, -1);
	const GEN_FLT x11 = x9 * x10;
	const GEN_FLT x12 = x11 * lh_qk;
	const GEN_FLT x13 = x12 * lh_qi;
	const GEN_FLT x14 = pow(obj_qk, 2);
	const GEN_FLT x15 = pow(obj_qj, 2);
	const GEN_FLT x16 = pow(obj_qi, 2);
	const GEN_FLT x17 = 1e-10 + x16 + x15 + x14;
	const GEN_FLT x18 = pow(x17, 1.0 / 2.0);
	const GEN_FLT x19 = cos(x18);
	const GEN_FLT x20 = pow(x17, -1) * (1 + (-1 * x19));
	const GEN_FLT x21 = pow(x18, -1) * sin(x18);
	const GEN_FLT x22 = x21 * obj_qi;
	const GEN_FLT x23 = x20 * obj_qk * obj_qj;
	const GEN_FLT x24 = x21 * obj_qj;
	const GEN_FLT x25 = x20 * obj_qi;
	const GEN_FLT x26 = x25 * obj_qk;
	const GEN_FLT x27 =
		((x26 + (-1 * x24)) * sensor_x) + ((x23 + x22) * sensor_y) + ((x19 + (x20 * x14)) * sensor_z) + obj_pz;
	const GEN_FLT x28 = x21 * obj_qk;
	const GEN_FLT x29 = x25 * obj_qj;
	const GEN_FLT x30 =
		((x29 + x28) * sensor_x) + ((x19 + (x20 * x15)) * sensor_y) + ((x23 + (-1 * x22)) * sensor_z) + obj_py;
	const GEN_FLT x31 = x6 * lh_qk;
	const GEN_FLT x32 = -1 * x31;
	const GEN_FLT x33 = x11 * lh_qj;
	const GEN_FLT x34 = x33 * lh_qi;
	const GEN_FLT x35 =
		((x19 + (x20 * x16)) * sensor_x) + ((x29 + (-1 * x28)) * sensor_y) + ((x26 + x24) * sensor_z) + obj_px;
	const GEN_FLT x36 = (x35 * (x8 + (x2 * x11))) + ((x34 + x32) * x30) + (x27 * (x13 + x7)) + lh_px;
	const GEN_FLT x37 = x6 * lh_qi;
	const GEN_FLT x38 = x33 * lh_qk;
	const GEN_FLT x39 = -1 * x7;
	const GEN_FLT x40 = ((x13 + x39) * x35) + ((x38 + x37) * x30) + (x27 * (x8 + (x0 * x11))) + lh_pz;
	const GEN_FLT x41 = pow(x40, 2);
	const GEN_FLT x42 = x41 + pow(x36, 2);
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = -1 * x37;
	const GEN_FLT x45 = (x30 * (x8 + (x1 * x11))) + ((x34 + x31) * x35) + lh_py + ((x38 + x44) * x27);
	const GEN_FLT x46 = pow(x45, 2);
	const GEN_FLT x47 = pow((1 + (-1 * x43 * x46 * pow(tilt_0, 2))), -1.0 / 2.0);
	const GEN_FLT x48 = pow(x42, -3.0 / 2.0) * x45 * tilt_0;
	const GEN_FLT x49 = x47 * x48;
	const GEN_FLT x50 = (x49 * x36) + (x40 * x43);
	const GEN_FLT x51 = pow(x42, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x52 = -1 * x40;
	const GEN_FLT x53 =
		sin(1.5707963267949 + (-1 * atan2(x36, x52)) + (-1 * phase_0) + (-1 * asin(x51 * x45)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x54 = x51 * x47;
	const GEN_FLT x55 = 2 * x40;
	const GEN_FLT x56 = pow((x41 + x46), -1) * atan2(x45, x52) * curve_0;
	const GEN_FLT x57 = (x40 * x49) + (-1 * x43 * x36);
	const GEN_FLT x58 = 2 * x56;
	const GEN_FLT x59 = pow(x3, -3.0 / 2.0) * x5;
	const GEN_FLT x60 = x2 * x59;
	const GEN_FLT x61 = 2 * pow(x3, -2) * x9;
	const GEN_FLT x62 = x61 * lh_qk;
	const GEN_FLT x63 = (-1 * x2 * x62) + (x60 * lh_qk);
	const GEN_FLT x64 = x63 + x12;
	const GEN_FLT x65 = x59 * lh_qj;
	const GEN_FLT x66 = x65 * lh_qi;
	const GEN_FLT x67 = x8 * x10;
	const GEN_FLT x68 = lh_qj * lh_qi;
	const GEN_FLT x69 = x67 * x68;
	const GEN_FLT x70 = (-1 * x69) + x66;
	const GEN_FLT x71 = x2 * x67;
	const GEN_FLT x72 = (-1 * x62 * x68) + (x66 * lh_qk);
	const GEN_FLT x73 = x72 + x6;
	const GEN_FLT x74 = x0 * x59;
	const GEN_FLT x75 = x0 * x61;
	const GEN_FLT x76 = (-1 * x75 * lh_qi) + (x74 * lh_qi);
	const GEN_FLT x77 = ((x76 + x44) * x27) + (x30 * (x73 + x71 + (-1 * x60))) + ((x70 + x64) * x35);
	const GEN_FLT x78 = pow(x41, -1);
	const GEN_FLT x79 = x78 * x36;
	const GEN_FLT x80 = x11 * lh_qi;
	const GEN_FLT x81 = pow(lh_qi, 3);
	const GEN_FLT x82 = (-1 * x2 * x61 * lh_qj) + (x60 * lh_qj);
	const GEN_FLT x83 = x82 + x33;
	const GEN_FLT x84 = x59 * lh_qk * lh_qi;
	const GEN_FLT x85 = x67 * lh_qk;
	const GEN_FLT x86 = x85 * lh_qi;
	const GEN_FLT x87 = (-1 * x86) + x84;
	const GEN_FLT x88 = x69 + (-1 * x66);
	const GEN_FLT x89 =
		((x88 + x64) * x27) + ((x87 + x83) * x30) + (((-1 * x81 * x61) + (x81 * x59) + (2 * x80) + x44) * x35);
	const GEN_FLT x90 = pow(x40, -1);
	const GEN_FLT x91 = x41 * x43;
	const GEN_FLT x92 = 2 * x36;
	const GEN_FLT x93 = 1.0 / 2.0 * x48;
	const GEN_FLT x94 = x86 + (-1 * x84);
	const GEN_FLT x95 = x1 * x59;
	const GEN_FLT x96 = (-1 * x1 * x61 * lh_qi) + (x95 * lh_qi);
	const GEN_FLT x97 = x72 + (-1 * x6);
	const GEN_FLT x98 = (x27 * (x97 + (-1 * x71) + x60)) + ((x96 + x44) * x30) + ((x94 + x83) * x35);
	const GEN_FLT x99 = (-1 * x47 * ((x51 * x98) + (-1 * ((x77 * x55) + (x89 * x92)) * x93))) +
						(-1 * ((-1 * x89 * x90) + (x79 * x77)) * x91);
	const GEN_FLT x100 = x78 * x45;
	const GEN_FLT x101 = x58 * x41;
	const GEN_FLT x102 = x1 * x67;
	const GEN_FLT x103 = (-1 * x1 * x62) + (x95 * lh_qk);
	const GEN_FLT x104 = x103 + x12;
	const GEN_FLT x105 = (-1 * x75 * lh_qj) + (x74 * lh_qj);
	const GEN_FLT x106 = (x27 * (x105 + x39)) + (x30 * (x104 + x88)) + (x35 * (x97 + (-1 * x102) + x95));
	const GEN_FLT x107 = x96 + x80;
	const GEN_FLT x108 = x65 * lh_qk;
	const GEN_FLT x109 = x85 * lh_qj;
	const GEN_FLT x110 = (-1 * x109) + x108;
	const GEN_FLT x111 = (x27 * (x73 + x102 + (-1 * x95))) + ((x110 + x107) * x30) + ((x82 + x39) * x35);
	const GEN_FLT x112 = pow(lh_qj, 3);
	const GEN_FLT x113 = x109 + (-1 * x108);
	const GEN_FLT x114 =
		(x27 * (x104 + x70)) + ((x113 + x107) * x35) + (((-1 * x61 * x112) + (x59 * x112) + (2 * x33) + x39) * x30);
	const GEN_FLT x115 = (-1 * x47 * ((x51 * x114) + (-1 * ((x55 * x106) + (x92 * x111)) * x93))) +
						 (-1 * ((-1 * x90 * x111) + (x79 * x106)) * x91);
	const GEN_FLT x116 = x76 + x80;
	const GEN_FLT x117 = x105 + x33;
	const GEN_FLT x118 = pow(lh_qk, 3);
	const GEN_FLT x119 =
		(((x59 * x118) + (-1 * x61 * x118) + (2 * x12) + x32) * x27) + (x30 * (x117 + x94)) + ((x116 + x110) * x35);
	const GEN_FLT x120 = x0 * x67;
	const GEN_FLT x121 = ((x116 + x113) * x27) + (x30 * (x97 + (-1 * x120) + x74)) + ((x63 + x32) * x35);
	const GEN_FLT x122 = (x27 * (x117 + x87)) + (x30 * (x103 + x32)) + (x35 * (x73 + x120 + (-1 * x74)));
	const GEN_FLT x123 = (-1 * x47 * ((x51 * x122) + (-1 * ((x55 * x119) + (x92 * x121)) * x93))) +
						 (-1 * ((-1 * x90 * x121) + (x79 * x119)) * x91);
	out[0] = x50 + (x50 * x53);
	out[1] = (-1 * x56 * x55) + (-1 * x54 * x53) + (-1 * x54);
	out[2] = x57 + (x58 * x45) + (x53 * x57);
	out[3] = x99 + (x101 * ((-1 * x90 * x98) + (x77 * x100))) + (x53 * x99);
	out[4] = x115 + (x101 * ((-1 * x90 * x114) + (x100 * x106))) + (x53 * x115);
	out[5] = x123 + (x101 * ((-1 * x90 * x122) + (x100 * x119))) + (x53 * x123);
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qi;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qk;
	const GEN_FLT x10 = x9 * lh_qj;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = pow(obj_qj, 2);
	const GEN_FLT x13 = pow(obj_qi, 2);
	const GEN_FLT x14 = 1e-10 + x13 + x12 + x11;
	const GEN_FLT x15 = pow(x14, 1.0 / 2.0);
	const GEN_FLT x16 = cos(x15);
	const GEN_FLT x17 = pow(x14, -1) * (1 + (-1 * x16));
	const GEN_FLT x18 = pow(x15, -1) * sin(x15);
	const GEN_FLT x19 = x18 * obj_qi;
	const GEN_FLT x20 = x17 * obj_qj;
	const GEN_FLT x21 = x20 * obj_qk;
	const GEN_FLT x22 = x18 * obj_qj;
	const GEN_FLT x23 = x17 * obj_qk * obj_qi;
	const GEN_FLT x24 =
		((x21 + x19) * sensor_y) + ((x23 + (-1 * x22)) * sensor_x) + ((x16 + (x11 * x17)) * sensor_z) + obj_pz;
	const GEN_FLT x25 = x18 * obj_qk;
	const GEN_FLT x26 = x20 * obj_qi;
	const GEN_FLT x27 =
		((x26 + x25) * sensor_x) + ((x16 + (x12 * x17)) * sensor_y) + ((x21 + (-1 * x19)) * sensor_z) + obj_py;
	const GEN_FLT x28 =
		((x26 + (-1 * x25)) * sensor_y) + ((x23 + x22) * sensor_z) + ((x16 + (x13 * x17)) * sensor_x) + obj_px;
	const GEN_FLT x29 = x5 * lh_qk;
	const GEN_FLT x30 = x8 * lh_qj * lh_qi;
	const GEN_FLT x31 = lh_py + ((x30 + x29) * x28) + (x27 * (x7 + (x1 * x8))) + (x24 * (x10 + (-1 * x6)));
	const GEN_FLT x32 = x5 * lh_qj;
	const GEN_FLT x33 = x9 * lh_qi;
	const GEN_FLT x34 = ((x33 + (-1 * x32)) * x28) + (x27 * (x10 + x6)) + (x24 * (x7 + (x0 * x8))) + lh_pz;
	const GEN_FLT x35 = ((x30 + (-1 * x29)) * x27) + ((x33 + x32) * x24) + (x28 * (x7 + (x2 * x8))) + lh_px;
	const GEN_FLT x36 = pow(x35, 2) + pow(x34, 2);
	const GEN_FLT x37 = x31 * pow(x36, -1.0 / 2.0);
	const GEN_FLT x38 = -1 * x34;
	const GEN_FLT x39 =
		1.5707963267949 + (-1 * atan2(x35, x38)) + (-1 * phase_0) + (-1 * asin(x37 * tilt_0)) + gibPhase_0;
	const GEN_FLT x40 = sin(x39) * gibMag_0;
	const GEN_FLT x41 = x37 * pow((1 + (-1 * pow(x31, 2) * pow(x36, -1) * pow(tilt_0, 2))), -1.0 / 2.0);
	out[0] = -1 + (-1 * x40);
	out[1] = (-1 * x40 * x41) + (-1 * x41);
	out[2] = pow(atan2(x31, x38), 2);
	out[3] = x40;
	out[4] = -1 * cos(x39);
	out[5] = 0;
	out[6] = 0;
}

static inline FLT gen_reproject_axis_y_axis_angle(const LinmathAxisAnglePose *obj_p, const FLT *sensor_pt,
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qj;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qk * lh_qi;
	const GEN_FLT x10 = pow(obj_qk, 2);
	const GEN_FLT x11 = pow(obj_qj, 2);
	const GEN_FLT x12 = pow(obj_qi, 2);
	const GEN_FLT x13 = 1e-10 + x12 + x11 + x10;
	const GEN_FLT x14 = pow(x13, 1.0 / 2.0);
	const GEN_FLT x15 = cos(x14);
	const GEN_FLT x16 = pow(x13, -1) * (1 + (-1 * x15));
	const GEN_FLT x17 = pow(x14, -1) * sin(x14);
	const GEN_FLT x18 = x17 * obj_qi;
	const GEN_FLT x19 = x16 * obj_qk;
	const GEN_FLT x20 = x19 * obj_qj;
	const GEN_FLT x21 = x17 * obj_qj;
	const GEN_FLT x22 = x19 * obj_qi;
	const GEN_FLT x23 =
		((x20 + x18) * sensor_y) + ((x22 + (-1 * x21)) * sensor_x) + ((x15 + (x10 * x16)) * sensor_z) + obj_pz;
	const GEN_FLT x24 = x17 * obj_qk;
	const GEN_FLT x25 = x16 * obj_qj * obj_qi;
	const GEN_FLT x26 =
		((x25 + x24) * sensor_x) + ((x15 + (x11 * x16)) * sensor_y) + ((x20 + (-1 * x18)) * sensor_z) + obj_py;
	const GEN_FLT x27 = x5 * lh_qk;
	const GEN_FLT x28 = x8 * lh_qj;
	const GEN_FLT x29 = x28 * lh_qi;
	const GEN_FLT x30 =
		((x15 + (x12 * x16)) * sensor_x) + ((x25 + (-1 * x24)) * sensor_y) + ((x22 + x21) * sensor_z) + obj_px;
	const GEN_FLT x31 = ((x9 + x6) * x23) + (x30 * (x7 + (x2 * x8))) + ((x29 + (-1 * x27)) * x26) + lh_px;
	const GEN_FLT x32 = x5 * lh_qi;
	const GEN_FLT x33 = x28 * lh_qk;
	const GEN_FLT x34 = ((x9 + (-1 * x6)) * x30) + ((x33 + x32) * x26) + (x23 * (x7 + (x0 * x8))) + lh_pz;
	const GEN_FLT x35 = -1 * x34;
	const GEN_FLT x36 = ((x29 + x27) * x30) + lh_py + (x26 * (x7 + (x1 * x8))) + ((x33 + (-1 * x32)) * x23);
	const GEN_FLT x37 = (-1 * atan2(-1 * x36, x35)) + (-1 * phase_1) +
						(-1 * asin(pow((pow(x36, 2) + pow(x34, 2)), -1.0 / 2.0) * x31 * tilt_1));
	return x37 + (-1 * cos(1.5707963267949 + x37 + gibPhase_1) * gibMag_1) + (pow(atan2(x31, x35), 2) * curve_1);
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qj;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qk * lh_qi;
	const GEN_FLT x10 = x9 + (-1 * x6);
	const GEN_FLT x11 = x5 * lh_qi;
	const GEN_FLT x12 = x8 * lh_qj;
	const GEN_FLT x13 = x12 * lh_qk;
	const GEN_FLT x14 = x13 + (-1 * x11);
	const GEN_FLT x15 = pow(obj_qk, 2);
	const GEN_FLT x16 = pow(obj_qj, 2);
	const GEN_FLT x17 = pow(obj_qi, 2);
	const GEN_FLT x18 = 1e-10 + x17 + x16 + x15;
	const GEN_FLT x19 = pow(x18, -1);
	const GEN_FLT x20 = pow(x18, 1.0 / 2.0);
	const GEN_FLT x21 = cos(x20);
	const GEN_FLT x22 = 1 + (-1 * x21);
	const GEN_FLT x23 = x22 * x19;
	const GEN_FLT x24 = sin(x20);
	const GEN_FLT x25 = x24 * pow(x20, -1);
	const GEN_FLT x26 = x25 * obj_qi;
	const GEN_FLT x27 = x23 * obj_qk;
	const GEN_FLT x28 = x27 * obj_qj;
	const GEN_FLT x29 = x25 * obj_qj;
	const GEN_FLT x30 = -1 * x29;
	const GEN_FLT x31 = x27 * obj_qi;
	const GEN_FLT x32 = ((x31 + x30) * sensor_x) + ((x28 + x26) * sensor_y) + ((x21 + (x23 * x15)) * sensor_z) + obj_pz;
	const GEN_FLT x33 = -1 * x26;
	const GEN_FLT x34 = x25 * obj_qk;
	const GEN_FLT x35 = x23 * obj_qi;
	const GEN_FLT x36 = x35 * obj_qj;
	const GEN_FLT x37 = ((x36 + x34) * sensor_x) + ((x21 + (x23 * x16)) * sensor_y) + ((x28 + x33) * sensor_z) + obj_py;
	const GEN_FLT x38 = x7 + (x1 * x8);
	const GEN_FLT x39 = -1 * x34;
	const GEN_FLT x40 = ((x21 + (x23 * x17)) * sensor_x) + ((x36 + x39) * sensor_y) + ((x31 + x29) * sensor_z) + obj_px;
	const GEN_FLT x41 = x5 * lh_qk;
	const GEN_FLT x42 = x12 * lh_qi;
	const GEN_FLT x43 = x42 + x41;
	const GEN_FLT x44 = (x40 * x43) + lh_py + (x38 * x37) + (x32 * x14);
	const GEN_FLT x45 = x7 + (x0 * x8);
	const GEN_FLT x46 = x13 + x11;
	const GEN_FLT x47 = (x40 * x10) + (x46 * x37) + (x45 * x32) + lh_pz;
	const GEN_FLT x48 = pow(x47, 2);
	const GEN_FLT x49 = pow(x48, -1);
	const GEN_FLT x50 = x44 * x49;
	const GEN_FLT x51 = pow(x47, -1);
	const GEN_FLT x52 = pow(x44, 2) + x48;
	const GEN_FLT x53 = pow(x52, -1);
	const GEN_FLT x54 = x53 * x48;
	const GEN_FLT x55 = x9 + x6;
	const GEN_FLT x56 = x42 + (-1 * x41);
	const GEN_FLT x57 = x7 + (x2 * x8);
	const GEN_FLT x58 = (x57 * x40) + (x56 * x37) + (x55 * x32) + lh_px;
	const GEN_FLT x59 = pow(x58, 2);
	const GEN_FLT x60 = pow((1 + (-1 * x53 * x59 * pow(tilt_1, 2))), -1.0 / 2.0);
	const GEN_FLT x61 = 2 * x44;
	const GEN_FLT x62 = 2 * x47;
	const GEN_FLT x63 = 1.0 / 2.0 * pow(x52, -3.0 / 2.0) * x58 * tilt_1;
	const GEN_FLT x64 = pow(x52, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x65 = (-1 * x60 * ((x64 * x57) + (-1 * ((x62 * x10) + (x61 * x43)) * x63))) +
						(-1 * ((x51 * x43) + (-1 * x50 * x10)) * x54);
	const GEN_FLT x66 = -1 * x47;
	const GEN_FLT x67 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x44, x66)) + (-1 * phase_1) + (-1 * asin(x64 * x58)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x68 = x58 * x49;
	const GEN_FLT x69 = 2 * pow((x48 + x59), -1) * x48 * atan2(x58, x66) * curve_1;
	const GEN_FLT x70 = (-1 * x60 * ((x64 * x56) + (-1 * ((x62 * x46) + (x61 * x38)) * x63))) +
						(-1 * ((x51 * x38) + (-1 * x50 * x46)) * x54);
	const GEN_FLT x71 = (-1 * x60 * ((x64 * x55) + (-1 * ((x62 * x45) + (x61 * x14)) * x63))) +
						(-1 * ((x51 * x14) + (-1 * x50 * x45)) * x54);
	const GEN_FLT x72 = pow(obj_qi, 3);
	const GEN_FLT x73 = x24 * pow(x18, -3.0 / 2.0);
	const GEN_FLT x74 = 2 * x22 * pow(x18, -2);
	const GEN_FLT x75 = x21 * x19;
	const GEN_FLT x76 = x75 * obj_qk;
	const GEN_FLT x77 = x76 * obj_qi;
	const GEN_FLT x78 = x73 * obj_qk;
	const GEN_FLT x79 = x78 * obj_qi;
	const GEN_FLT x80 = x79 + (-1 * x77);
	const GEN_FLT x81 = x23 * obj_qj;
	const GEN_FLT x82 = x73 * x17;
	const GEN_FLT x83 = (-1 * x74 * x17 * obj_qj) + (x82 * obj_qj);
	const GEN_FLT x84 = x83 + x81;
	const GEN_FLT x85 = obj_qj * obj_qi;
	const GEN_FLT x86 = x85 * x75;
	const GEN_FLT x87 = x85 * x73;
	const GEN_FLT x88 = (-1 * x87) + x86;
	const GEN_FLT x89 = x74 * obj_qk;
	const GEN_FLT x90 = (-1 * x89 * x17) + (x82 * obj_qk);
	const GEN_FLT x91 = x90 + x27;
	const GEN_FLT x92 = ((x91 + x88) * sensor_z) + ((x84 + x80) * sensor_y) +
						(((-1 * x72 * x74) + (x73 * x72) + (2 * x35) + x33) * sensor_x);
	const GEN_FLT x93 = (-1 * x79) + x77;
	const GEN_FLT x94 = x73 * x16;
	const GEN_FLT x95 = (-1 * x74 * x16 * obj_qi) + (x94 * obj_qi);
	const GEN_FLT x96 = x75 * x17;
	const GEN_FLT x97 = x78 * obj_qj;
	const GEN_FLT x98 = (-1 * x89 * x85) + (x97 * obj_qi);
	const GEN_FLT x99 = x98 + (-1 * x25);
	const GEN_FLT x100 = ((x99 + x82 + (-1 * x96)) * sensor_z) + ((x95 + x33) * sensor_y) + ((x84 + x93) * sensor_x);
	const GEN_FLT x101 = x87 + (-1 * x86);
	const GEN_FLT x102 = x98 + x25;
	const GEN_FLT x103 = x73 * x15;
	const GEN_FLT x104 = x74 * x15;
	const GEN_FLT x105 = (-1 * x104 * obj_qi) + (x103 * obj_qi);
	const GEN_FLT x106 = ((x105 + x33) * sensor_z) + ((x102 + (-1 * x82) + x96) * sensor_y) + ((x91 + x101) * sensor_x);
	const GEN_FLT x107 = (x45 * x106) + (x46 * x100) + (x92 * x10);
	const GEN_FLT x108 = (x14 * x106) + (x38 * x100) + (x92 * x43);
	const GEN_FLT x109 = (x55 * x106) + (x56 * x100) + (x57 * x92);
	const GEN_FLT x110 = (-1 * x60 * ((x64 * x109) + (-1 * ((x62 * x107) + (x61 * x108)) * x63))) +
						 (-1 * ((x51 * x108) + (-1 * x50 * x107)) * x54);
	const GEN_FLT x111 = x76 * obj_qj;
	const GEN_FLT x112 = (-1 * x97) + x111;
	const GEN_FLT x113 = x95 + x35;
	const GEN_FLT x114 = pow(obj_qj, 3);
	const GEN_FLT x115 = (-1 * x89 * x16) + (x94 * obj_qk);
	const GEN_FLT x116 = x115 + x27;
	const GEN_FLT x117 = ((x101 + x116) * sensor_z) +
						 (((x73 * x114) + (-1 * x74 * x114) + (2 * x81) + x30) * sensor_y) + ((x113 + x112) * sensor_x);
	const GEN_FLT x118 = x97 + (-1 * x111);
	const GEN_FLT x119 = x75 * x16;
	const GEN_FLT x120 =
		((x102 + (-1 * x94) + x119) * sensor_z) + ((x113 + x118) * sensor_y) + ((x83 + x30) * sensor_x);
	const GEN_FLT x121 = (-1 * x104 * obj_qj) + (x103 * obj_qj);
	const GEN_FLT x122 = ((x121 + x30) * sensor_z) + ((x88 + x116) * sensor_y) + ((x99 + x94 + (-1 * x119)) * sensor_x);
	const GEN_FLT x123 = (x45 * x122) + (x10 * x120) + (x46 * x117);
	const GEN_FLT x124 = (x14 * x122) + (x38 * x117) + (x43 * x120);
	const GEN_FLT x125 = (x55 * x122) + (x56 * x117) + (x57 * x120);
	const GEN_FLT x126 = (-1 * x60 * ((x64 * x125) + (-1 * ((x62 * x123) + (x61 * x124)) * x63))) +
						 (-1 * ((x51 * x124) + (-1 * x50 * x123)) * x54);
	const GEN_FLT x127 = x75 * x15;
	const GEN_FLT x128 = x105 + x35;
	const GEN_FLT x129 =
		((x112 + x128) * sensor_z) + ((x103 + x99 + (-1 * x127)) * sensor_y) + ((x90 + x39) * sensor_x);
	const GEN_FLT x130 = x121 + x81;
	const GEN_FLT x131 =
		((x80 + x130) * sensor_z) + ((x115 + x39) * sensor_y) + ((x102 + (-1 * x103) + x127) * sensor_x);
	const GEN_FLT x132 = pow(obj_qk, 3);
	const GEN_FLT x133 = (((-1 * x74 * x132) + x39 + (x73 * x132) + (2 * x27)) * sensor_z) + ((x93 + x130) * sensor_y) +
						 ((x118 + x128) * sensor_x);
	const GEN_FLT x134 = (x45 * x133) + (x46 * x131) + (x10 * x129);
	const GEN_FLT x135 = (x38 * x131) + (x14 * x133) + (x43 * x129);
	const GEN_FLT x136 = (x55 * x133) + (x56 * x131) + (x57 * x129);
	const GEN_FLT x137 = (-1 * x60 * ((x64 * x136) + (-1 * ((x62 * x134) + (x61 * x135)) * x63))) +
						 (-1 * ((x51 * x135) + (-1 * x50 * x134)) * x54);
	out[0] = x65 + (((-1 * x51 * x57) + (x68 * x10)) * x69) + (x67 * x65);
	out[1] = x70 + (((-1 * x51 * x56) + (x68 * x46)) * x69) + (x70 * x67);
	out[2] = (((-1 * x51 * x55) + (x68 * x45)) * x69) + x71 + (x71 * x67);
	out[3] = x110 + (((-1 * x51 * x109) + (x68 * x107)) * x69) + (x67 * x110);
	out[4] = x126 + (((-1 * x51 * x125) + (x68 * x123)) * x69) + (x67 * x126);
	out[5] = x137 + (((-1 * x51 * x136) + (x68 * x134)) * x69) + (x67 * x137);
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qj;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qk;
	const GEN_FLT x10 = x9 * lh_qi;
	const GEN_FLT x11 = x10 + (-1 * x6);
	const GEN_FLT x12 = pow(obj_qi, 2);
	const GEN_FLT x13 = pow(obj_qk, 2);
	const GEN_FLT x14 = pow(obj_qj, 2);
	const GEN_FLT x15 = 1e-10 + x12 + x14 + x13;
	const GEN_FLT x16 = pow(x15, 1.0 / 2.0);
	const GEN_FLT x17 = cos(x16);
	const GEN_FLT x18 = pow(x15, -1) * (1 + (-1 * x17));
	const GEN_FLT x19 = x17 + (x12 * x18);
	const GEN_FLT x20 = pow(x16, -1) * sin(x16);
	const GEN_FLT x21 = x20 * obj_qk;
	const GEN_FLT x22 = x18 * obj_qj * obj_qi;
	const GEN_FLT x23 = x22 + x21;
	const GEN_FLT x24 = x5 * lh_qi;
	const GEN_FLT x25 = x9 * lh_qj;
	const GEN_FLT x26 = x25 + x24;
	const GEN_FLT x27 = x20 * obj_qj;
	const GEN_FLT x28 = x18 * obj_qk;
	const GEN_FLT x29 = x28 * obj_qi;
	const GEN_FLT x30 = x29 + (-1 * x27);
	const GEN_FLT x31 = x7 + (x0 * x8);
	const GEN_FLT x32 = (x30 * x31) + (x23 * x26) + (x11 * x19);
	const GEN_FLT x33 = x17 + (x13 * x18);
	const GEN_FLT x34 = x20 * obj_qi;
	const GEN_FLT x35 = x28 * obj_qj;
	const GEN_FLT x36 = x35 + x34;
	const GEN_FLT x37 = (x30 * sensor_x) + (x33 * sensor_z) + (x36 * sensor_y) + obj_pz;
	const GEN_FLT x38 = x35 + (-1 * x34);
	const GEN_FLT x39 = x17 + (x14 * x18);
	const GEN_FLT x40 = (x39 * sensor_y) + (x23 * sensor_x) + (x38 * sensor_z) + obj_py;
	const GEN_FLT x41 = x29 + x27;
	const GEN_FLT x42 = x22 + (-1 * x21);
	const GEN_FLT x43 = (x19 * sensor_x) + (x41 * sensor_z) + (x42 * sensor_y) + obj_px;
	const GEN_FLT x44 = (x43 * x11) + (x31 * x37) + (x40 * x26) + lh_pz;
	const GEN_FLT x45 = pow(x44, 2);
	const GEN_FLT x46 = pow(x45, -1);
	const GEN_FLT x47 = x25 + (-1 * x24);
	const GEN_FLT x48 = x7 + (x1 * x8);
	const GEN_FLT x49 = x5 * lh_qk;
	const GEN_FLT x50 = x8 * lh_qj * lh_qi;
	const GEN_FLT x51 = x50 + x49;
	const GEN_FLT x52 = lh_py + (x40 * x48) + (x51 * x43) + (x47 * x37);
	const GEN_FLT x53 = x52 * x46;
	const GEN_FLT x54 = (x47 * x30) + (x48 * x23) + (x51 * x19);
	const GEN_FLT x55 = pow(x44, -1);
	const GEN_FLT x56 = x45 + pow(x52, 2);
	const GEN_FLT x57 = pow(x56, -1);
	const GEN_FLT x58 = x57 * x45;
	const GEN_FLT x59 = x10 + x6;
	const GEN_FLT x60 = x50 + (-1 * x49);
	const GEN_FLT x61 = x7 + (x2 * x8);
	const GEN_FLT x62 = (x61 * x43) + (x60 * x40) + (x59 * x37) + lh_px;
	const GEN_FLT x63 = pow(x62, 2);
	const GEN_FLT x64 = pow((1 + (-1 * x63 * x57 * pow(tilt_1, 2))), -1.0 / 2.0);
	const GEN_FLT x65 = 2 * x52;
	const GEN_FLT x66 = 2 * x44;
	const GEN_FLT x67 = 1.0 / 2.0 * x62 * pow(x56, -3.0 / 2.0) * tilt_1;
	const GEN_FLT x68 = (x59 * x30) + (x60 * x23) + (x61 * x19);
	const GEN_FLT x69 = pow(x56, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x70 = (-1 * x64 * ((x68 * x69) + (-1 * ((x66 * x32) + (x65 * x54)) * x67))) +
						(-1 * ((x54 * x55) + (-1 * x53 * x32)) * x58);
	const GEN_FLT x71 = -1 * x44;
	const GEN_FLT x72 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x52, x71)) + (-1 * phase_1) + (-1 * asin(x62 * x69)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x73 = x62 * x46;
	const GEN_FLT x74 = 2 * pow((x45 + x63), -1) * x45 * atan2(x62, x71) * curve_1;
	const GEN_FLT x75 = (x31 * x36) + (x39 * x26) + (x42 * x11);
	const GEN_FLT x76 = x75 * x46;
	const GEN_FLT x77 = (x47 * x36) + (x48 * x39) + (x51 * x42);
	const GEN_FLT x78 = (x59 * x36) + (x60 * x39) + (x61 * x42);
	const GEN_FLT x79 = (-1 * x64 * ((x78 * x69) + (-1 * ((x75 * x66) + (x77 * x65)) * x67))) +
						(-1 * ((x77 * x55) + (-1 * x76 * x52)) * x58);
	const GEN_FLT x80 = (x31 * x33) + (x38 * x26) + (x41 * x11);
	const GEN_FLT x81 = (x48 * x38) + (x47 * x33) + (x51 * x41);
	const GEN_FLT x82 = (x59 * x33) + (x60 * x38) + (x61 * x41);
	const GEN_FLT x83 = (-1 * x64 * ((x82 * x69) + (-1 * ((x80 * x66) + (x81 * x65)) * x67))) +
						(-1 * ((x81 * x55) + (-1 * x80 * x53)) * x58);
	out[0] = x70 + (((-1 * x68 * x55) + (x73 * x32)) * x74) + (x70 * x72);
	out[1] = x79 + (((-1 * x78 * x55) + (x76 * x62)) * x74) + (x72 * x79);
	out[2] = x83 + (((-1 * x82 * x55) + (x80 * x73)) * x74) + (x83 * x72);
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = sin(x4);
	const GEN_FLT x6 = pow(x4, -1) * x5;
	const GEN_FLT x7 = x6 * lh_qj;
	const GEN_FLT x8 = pow(x3, -1);
	const GEN_FLT x9 = cos(x4);
	const GEN_FLT x10 = 1 + (-1 * x9);
	const GEN_FLT x11 = x8 * x10;
	const GEN_FLT x12 = x11 * lh_qi;
	const GEN_FLT x13 = x12 * lh_qk;
	const GEN_FLT x14 = pow(obj_qk, 2);
	const GEN_FLT x15 = pow(obj_qj, 2);
	const GEN_FLT x16 = pow(obj_qi, 2);
	const GEN_FLT x17 = 1e-10 + x16 + x15 + x14;
	const GEN_FLT x18 = pow(x17, 1.0 / 2.0);
	const GEN_FLT x19 = cos(x18);
	const GEN_FLT x20 = pow(x17, -1) * (1 + (-1 * x19));
	const GEN_FLT x21 = pow(x18, -1) * sin(x18);
	const GEN_FLT x22 = x21 * obj_qi;
	const GEN_FLT x23 = x20 * obj_qk * obj_qj;
	const GEN_FLT x24 = x21 * obj_qj;
	const GEN_FLT x25 = x20 * obj_qi;
	const GEN_FLT x26 = x25 * obj_qk;
	const GEN_FLT x27 =
		((x26 + (-1 * x24)) * sensor_x) + ((x23 + x22) * sensor_y) + ((x19 + (x20 * x14)) * sensor_z) + obj_pz;
	const GEN_FLT x28 = x21 * obj_qk;
	const GEN_FLT x29 = x25 * obj_qj;
	const GEN_FLT x30 =
		((x29 + x28) * sensor_x) + ((x19 + (x20 * x15)) * sensor_y) + ((x23 + (-1 * x22)) * sensor_z) + obj_py;
	const GEN_FLT x31 = x6 * lh_qk;
	const GEN_FLT x32 = -1 * x31;
	const GEN_FLT x33 = x12 * lh_qj;
	const GEN_FLT x34 =
		((x19 + (x20 * x16)) * sensor_x) + ((x29 + (-1 * x28)) * sensor_y) + ((x26 + x24) * sensor_z) + obj_px;
	const GEN_FLT x35 = (x34 * (x9 + (x2 * x11))) + ((x33 + x32) * x30) + (x27 * (x13 + x7)) + lh_px;
	const GEN_FLT x36 = pow(x35, 2);
	const GEN_FLT x37 = x6 * lh_qi;
	const GEN_FLT x38 = x11 * lh_qj;
	const GEN_FLT x39 = x38 * lh_qk;
	const GEN_FLT x40 = -1 * x7;
	const GEN_FLT x41 = ((x39 + x37) * x30) + (x27 * (x9 + (x0 * x11))) + ((x13 + x40) * x34) + lh_pz;
	const GEN_FLT x42 = pow(x41, 2);
	const GEN_FLT x43 = -1 * x37;
	const GEN_FLT x44 = ((x33 + x31) * x34) + lh_py + (x30 * (x9 + (x1 * x11))) + ((x39 + x43) * x27);
	const GEN_FLT x45 = pow(x44, 2) + x42;
	const GEN_FLT x46 = pow(x45, -1);
	const GEN_FLT x47 = pow((1 + (-1 * x46 * x36 * pow(tilt_1, 2))), -1.0 / 2.0);
	const GEN_FLT x48 = pow(x45, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x49 = x47 * x48;
	const GEN_FLT x50 = -1 * x41;
	const GEN_FLT x51 =
		sin(1.5707963267949 + (-1 * phase_1) + (-1 * asin(x48 * x35)) + (-1 * atan2(-1 * x44, x50)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x52 = 2 * x41;
	const GEN_FLT x53 = pow((x42 + x36), -1) * atan2(x35, x50) * curve_1;
	const GEN_FLT x54 = pow(x45, -3.0 / 2.0) * x35 * tilt_1;
	const GEN_FLT x55 = x54 * x47;
	const GEN_FLT x56 = (x55 * x44) + (-1 * x41 * x46);
	const GEN_FLT x57 = (x55 * x41) + (x44 * x46);
	const GEN_FLT x58 = 2 * x53;
	const GEN_FLT x59 = x11 * lh_qk;
	const GEN_FLT x60 = pow(x3, -3.0 / 2.0) * x5;
	const GEN_FLT x61 = x2 * x60;
	const GEN_FLT x62 = 2 * pow(x3, -2) * x10;
	const GEN_FLT x63 = x62 * lh_qk;
	const GEN_FLT x64 = (-1 * x2 * x63) + (x61 * lh_qk);
	const GEN_FLT x65 = x64 + x59;
	const GEN_FLT x66 = x60 * lh_qj;
	const GEN_FLT x67 = x66 * lh_qi;
	const GEN_FLT x68 = x8 * x9;
	const GEN_FLT x69 = x68 * lh_qj * lh_qi;
	const GEN_FLT x70 = (-1 * x69) + x67;
	const GEN_FLT x71 = x2 * x68;
	const GEN_FLT x72 = lh_qk * lh_qi;
	const GEN_FLT x73 = x62 * lh_qj;
	const GEN_FLT x74 = (-1 * x73 * x72) + (x67 * lh_qk);
	const GEN_FLT x75 = x74 + x6;
	const GEN_FLT x76 = x0 * x60;
	const GEN_FLT x77 = x62 * lh_qi;
	const GEN_FLT x78 = (-1 * x0 * x77) + (x76 * lh_qi);
	const GEN_FLT x79 = ((x78 + x43) * x27) + (x30 * (x75 + x71 + (-1 * x61))) + ((x70 + x65) * x34);
	const GEN_FLT x80 = pow(x42, -1);
	const GEN_FLT x81 = x80 * x44;
	const GEN_FLT x82 = (-1 * x2 * x73) + (x61 * lh_qj);
	const GEN_FLT x83 = x82 + x38;
	const GEN_FLT x84 = x72 * x60;
	const GEN_FLT x85 = x68 * lh_qk;
	const GEN_FLT x86 = x85 * lh_qi;
	const GEN_FLT x87 = x86 + (-1 * x84);
	const GEN_FLT x88 = x1 * x60;
	const GEN_FLT x89 = (-1 * x1 * x77) + (x88 * lh_qi);
	const GEN_FLT x90 = x74 + (-1 * x6);
	const GEN_FLT x91 = (x27 * (x90 + (-1 * x71) + x61)) + ((x89 + x43) * x30) + ((x87 + x83) * x34);
	const GEN_FLT x92 = pow(x41, -1);
	const GEN_FLT x93 = x42 * x46;
	const GEN_FLT x94 = 2 * x44;
	const GEN_FLT x95 = 1.0 / 2.0 * x54;
	const GEN_FLT x96 = pow(lh_qi, 3);
	const GEN_FLT x97 = (-1 * x86) + x84;
	const GEN_FLT x98 = x69 + (-1 * x67);
	const GEN_FLT x99 =
		((x65 + x98) * x27) + ((x97 + x83) * x30) + (((x60 * x96) + (-1 * x62 * x96) + (2 * x12) + x43) * x34);
	const GEN_FLT x100 = (-1 * x47 * ((x99 * x48) + (-1 * ((x79 * x52) + (x91 * x94)) * x95))) +
						 (-1 * ((x92 * x91) + (-1 * x81 * x79)) * x93);
	const GEN_FLT x101 = x80 * x35;
	const GEN_FLT x102 = x58 * x42;
	const GEN_FLT x103 = x1 * x68;
	const GEN_FLT x104 = (-1 * x1 * x63) + (x88 * lh_qk);
	const GEN_FLT x105 = x104 + x59;
	const GEN_FLT x106 = (-1 * x0 * x73) + (x76 * lh_qj);
	const GEN_FLT x107 = (x27 * (x106 + x40)) + (x30 * (x98 + x105)) + (x34 * (x90 + (-1 * x103) + x88));
	const GEN_FLT x108 = pow(lh_qj, 3);
	const GEN_FLT x109 = x89 + x12;
	const GEN_FLT x110 = x66 * lh_qk;
	const GEN_FLT x111 = x85 * lh_qj;
	const GEN_FLT x112 = x111 + (-1 * x110);
	const GEN_FLT x113 =
		(x27 * (x70 + x105)) + ((x112 + x109) * x34) + (((x60 * x108) + (2 * x38) + (-1 * x62 * x108) + x40) * x30);
	const GEN_FLT x114 = (-1 * x111) + x110;
	const GEN_FLT x115 = (x27 * (x75 + x103 + (-1 * x88))) + ((x114 + x109) * x30) + ((x82 + x40) * x34);
	const GEN_FLT x116 = (-1 * x47 * ((x48 * x115) + (-1 * ((x52 * x107) + (x94 * x113)) * x95))) +
						 (-1 * ((x92 * x113) + (-1 * x81 * x107)) * x93);
	const GEN_FLT x117 = x78 + x12;
	const GEN_FLT x118 = x106 + x38;
	const GEN_FLT x119 = pow(lh_qk, 3);
	const GEN_FLT x120 =
		(((x60 * x119) + (-1 * x62 * x119) + (2 * x59) + x32) * x27) + (x30 * (x118 + x87)) + ((x117 + x114) * x34);
	const GEN_FLT x121 = x0 * x68;
	const GEN_FLT x122 = (x27 * (x118 + x97)) + (x30 * (x104 + x32)) + (x34 * (x75 + x121 + (-1 * x76)));
	const GEN_FLT x123 = ((x117 + x112) * x27) + (x30 * (x90 + (-1 * x121) + x76)) + ((x64 + x32) * x34);
	const GEN_FLT x124 = (-1 * x47 * ((x48 * x123) + (-1 * ((x52 * x120) + (x94 * x122)) * x95))) +
						 (-1 * ((x92 * x122) + (-1 * x81 * x120)) * x93);
	out[0] = (-1 * x53 * x52) + (-1 * x51 * x49) + (-1 * x49);
	out[1] = x56 + (x51 * x56);
	out[2] = x57 + (x58 * x35) + (x51 * x57);
	out[3] = x100 + (x102 * ((-1 * x92 * x99) + (x79 * x101))) + (x51 * x100);
	out[4] = x116 + (x102 * ((-1 * x92 * x115) + (x101 * x107))) + (x51 * x116);
	out[5] = x124 + (x102 * ((-1 * x92 * x123) + (x101 * x120))) + (x51 * x124);
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = pow(x3, 1.0 / 2.0);
	const GEN_FLT x5 = pow(x4, -1) * sin(x4);
	const GEN_FLT x6 = x5 * lh_qj;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = pow(x3, -1) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * lh_qk;
	const GEN_FLT x10 = x9 * lh_qi;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = pow(obj_qj, 2);
	const GEN_FLT x13 = pow(obj_qi, 2);
	const GEN_FLT x14 = 1e-10 + x13 + x12 + x11;
	const GEN_FLT x15 = pow(x14, 1.0 / 2.0);
	const GEN_FLT x16 = cos(x15);
	const GEN_FLT x17 = pow(x14, -1) * (1 + (-1 * x16));
	const GEN_FLT x18 = pow(x15, -1) * sin(x15);
	const GEN_FLT x19 = x18 * obj_qi;
	const GEN_FLT x20 = x17 * obj_qk;
	const GEN_FLT x21 = x20 * obj_qj;
	const GEN_FLT x22 = x18 * obj_qj;
	const GEN_FLT x23 = x20 * obj_qi;
	const GEN_FLT x24 =
		((x21 + x19) * sensor_y) + ((x23 + (-1 * x22)) * sensor_x) + ((x16 + (x11 * x17)) * sensor_z) + obj_pz;
	const GEN_FLT x25 = x18 * obj_qk;
	const GEN_FLT x26 = x17 * obj_qj * obj_qi;
	const GEN_FLT x27 =
		((x26 + x25) * sensor_x) + ((x16 + (x12 * x17)) * sensor_y) + ((x21 + (-1 * x19)) * sensor_z) + obj_py;
	const GEN_FLT x28 = x5 * lh_qk;
	const GEN_FLT x29 = x8 * lh_qj * lh_qi;
	const GEN_FLT x30 =
		((x26 + (-1 * x25)) * sensor_y) + ((x23 + x22) * sensor_z) + ((x16 + (x13 * x17)) * sensor_x) + obj_px;
	const GEN_FLT x31 = ((x29 + (-1 * x28)) * x27) + (x24 * (x10 + x6)) + (x30 * (x7 + (x2 * x8))) + lh_px;
	const GEN_FLT x32 = x5 * lh_qi;
	const GEN_FLT x33 = x9 * lh_qj;
	const GEN_FLT x34 = (x30 * (x10 + (-1 * x6))) + ((x33 + x32) * x27) + (x24 * (x7 + (x0 * x8))) + lh_pz;
	const GEN_FLT x35 = ((x29 + x28) * x30) + (x27 * (x7 + (x1 * x8))) + lh_py + ((x33 + (-1 * x32)) * x24);
	const GEN_FLT x36 = pow(x35, 2) + pow(x34, 2);
	const GEN_FLT x37 = x31 * pow(x36, -1.0 / 2.0);
	const GEN_FLT x38 = -1 * x34;
	const GEN_FLT x39 =
		1.5707963267949 + (-1 * atan2(-1 * x35, x38)) + (-1 * phase_1) + (-1 * asin(x37 * tilt_1)) + gibPhase_1;
	const GEN_FLT x40 = sin(x39) * gibMag_1;
	const GEN_FLT x41 = x37 * pow((1 + (-1 * pow(x31, 2) * pow(x36, -1) * pow(tilt_1, 2))), -1.0 / 2.0);
	out[0] = -1 + (-1 * x40);
	out[1] = (-1 * x40 * x41) + (-1 * x41);
	out[2] = pow(atan2(x31, x38), 2);
	out[3] = x40;
	out[4] = -1 * cos(x39);
	out[5] = 0;
	out[6] = 0;
}

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
	const GEN_FLT x1 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x2 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x3 = sensor_x + (2 * ((x2 * obj_qj) + (-1 * x1 * obj_qk))) + obj_px;
	const GEN_FLT x4 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x5 = sensor_y + (2 * ((x4 * obj_qk) + (-1 * x2 * obj_qi))) + obj_py;
	const GEN_FLT x6 = sensor_z + (2 * ((x1 * obj_qi) + (-1 * x4 * obj_qj))) + obj_pz;
	const GEN_FLT x7 = (x6 * lh_qj) + (-1 * x5 * lh_qk) + (x3 * lh_qw);
	const GEN_FLT x8 = (x3 * lh_qk) + (-1 * x6 * lh_qi) + (x5 * lh_qw);
	const GEN_FLT x9 = x6 + (2 * ((x8 * lh_qi) + (-1 * x7 * lh_qj))) + lh_pz;
	const GEN_FLT x10 = (x5 * lh_qi) + (-1 * x3 * lh_qj) + (x6 * lh_qw);
	const GEN_FLT x11 = x3 + (2 * ((x10 * lh_qj) + (-1 * x8 * lh_qk))) + lh_px;
	const GEN_FLT x12 = pow(x11, 2) + pow(x9, 2);
	const GEN_FLT x13 = x5 + (2 * ((x7 * lh_qk) + (-1 * x10 * lh_qi))) + lh_py;
	const GEN_FLT x14 = x13 * pow(x12, -1.0 / 2.0);
	const GEN_FLT x15 = tan(x0) * x14;
	const GEN_FLT x16 = atan2(-1 * x9, x11);
	const GEN_FLT x17 = (sin(x16 + (-1 * asin(x15)) + ogeeMag_0) * ogeePhase_0) + curve_0;
	const GEN_FLT x18 = cos(x0);
	const GEN_FLT x19 = x13 * pow((x12 + pow(x13, 2)), -1.0 / 2.0);
	const GEN_FLT x20 = asin(x19 * pow(x18, -1));
	const GEN_FLT x21 = 0.0028679863 + (x20 * (-8.0108022e-06 + (-8.0108022e-06 * x20)));
	const GEN_FLT x22 = 5.3685255e-06 + (x20 * x21);
	const GEN_FLT x23 = 0.0076069798 + (x22 * x20);
	const GEN_FLT x24 = asin(
		x15 +
		(x23 * pow(x20, 2) * x17 *
		 pow((x18 +
			  (-1 * sin(x0) * x17 *
			   ((x20 * (x23 + (x20 * (x22 + (x20 * (x21 + (x20 * (-8.0108022e-06 + (-1.60216044e-05 * x20))))))))) +
				(x23 * x20)))),
			 -1)));
	const GEN_FLT x25 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x26 = cos(x25);
	const GEN_FLT x27 = asin(pow(x26, -1) * x19);
	const GEN_FLT x28 = 0.0028679863 + (x27 * (-8.0108022e-06 + (-8.0108022e-06 * x27)));
	const GEN_FLT x29 = 5.3685255e-06 + (x28 * x27);
	const GEN_FLT x30 = 0.0076069798 + (x29 * x27);
	const GEN_FLT x31 = -1 * x14 * tan(x25);
	const GEN_FLT x32 = (sin(x16 + (-1 * asin(x31)) + ogeeMag_1) * ogeePhase_1) + curve_1;
	const GEN_FLT x33 =
		(-1 *
		 asin(x31 + (x30 * x32 * pow(x27, 2) *
					 pow((x26 + (x32 * sin(x25) *
								 ((x27 * (x30 + (x27 * (x29 + (x27 * (x28 + (x27 * (-8.0108022e-06 +
																					(-1.60216044e-05 * x27))))))))) +
								  (x30 * x27)))),
						 -1)))) +
		x16;
	out[0] = -1.5707963267949 + x16 + (-1 * x24) + (-1 * sin((-1 * x16) + x24 + (-1 * gibPhase_0)) * gibMag_0) +
			 (-1 * phase_0);
	out[1] = -1.5707963267949 + x33 + (sin(x33 + gibPhase_1) * gibMag_1) + (-1 * phase_1);
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
	const GEN_FLT x0 = 2 * lh_qj;
	const GEN_FLT x1 = x0 * lh_qw;
	const GEN_FLT x2 = 2 * lh_qk;
	const GEN_FLT x3 = x2 * lh_qi;
	const GEN_FLT x4 = x3 + (-1 * x1);
	const GEN_FLT x5 = obj_qj * sensor_x;
	const GEN_FLT x6 = obj_qw * sensor_z;
	const GEN_FLT x7 = obj_qi * sensor_y;
	const GEN_FLT x8 = x7 + x6 + (-1 * x5);
	const GEN_FLT x9 = obj_qw * sensor_x;
	const GEN_FLT x10 = obj_qk * sensor_y;
	const GEN_FLT x11 = obj_qj * sensor_z;
	const GEN_FLT x12 = x11 + (-1 * x10) + x9;
	const GEN_FLT x13 = sensor_y + (2 * ((x12 * obj_qk) + (-1 * x8 * obj_qi))) + obj_py;
	const GEN_FLT x14 = obj_qw * sensor_y;
	const GEN_FLT x15 = obj_qi * sensor_z;
	const GEN_FLT x16 = obj_qk * sensor_x;
	const GEN_FLT x17 = x16 + (-1 * x15) + x14;
	const GEN_FLT x18 = sensor_z + (2 * ((x17 * obj_qi) + (-1 * x12 * obj_qj))) + obj_pz;
	const GEN_FLT x19 = sensor_x + (2 * ((x8 * obj_qj) + (-1 * x17 * obj_qk))) + obj_px;
	const GEN_FLT x20 = (x19 * lh_qk) + (-1 * x18 * lh_qi) + (x13 * lh_qw);
	const GEN_FLT x21 = (x13 * lh_qi) + (-1 * x19 * lh_qj) + (x18 * lh_qw);
	const GEN_FLT x22 = x19 + (2 * ((x21 * lh_qj) + (-1 * x20 * lh_qk))) + lh_px;
	const GEN_FLT x23 = pow(x22, -1);
	const GEN_FLT x24 = -2 * pow(lh_qk, 2);
	const GEN_FLT x25 = -2 * pow(lh_qj, 2);
	const GEN_FLT x26 = 1 + x25 + x24;
	const GEN_FLT x27 = pow(x22, 2);
	const GEN_FLT x28 = (x18 * lh_qj) + (-1 * x13 * lh_qk) + (x19 * lh_qw);
	const GEN_FLT x29 = x18 + (2 * ((x20 * lh_qi) + (-1 * x28 * lh_qj))) + lh_pz;
	const GEN_FLT x30 = x29 * pow(x27, -1);
	const GEN_FLT x31 = x27 + pow(x29, 2);
	const GEN_FLT x32 = pow(x31, -1);
	const GEN_FLT x33 = x32 * x27;
	const GEN_FLT x34 = x33 * ((x30 * x26) + (-1 * x4 * x23));
	const GEN_FLT x35 = 0.523598775598299 + tilt_0;
	const GEN_FLT x36 = cos(x35);
	const GEN_FLT x37 = pow(x36, -1);
	const GEN_FLT x38 = x13 + (2 * ((x28 * lh_qk) + (-1 * x21 * lh_qi))) + lh_py;
	const GEN_FLT x39 = pow(x38, 2);
	const GEN_FLT x40 = x31 + x39;
	const GEN_FLT x41 = pow(x40, -1.0 / 2.0);
	const GEN_FLT x42 = x41 * x38;
	const GEN_FLT x43 = asin(x42 * x37);
	const GEN_FLT x44 = 8.0108022e-06 * x43;
	const GEN_FLT x45 = -8.0108022e-06 + (-1 * x44);
	const GEN_FLT x46 = 0.0028679863 + (x43 * x45);
	const GEN_FLT x47 = 5.3685255e-06 + (x43 * x46);
	const GEN_FLT x48 = 0.0076069798 + (x43 * x47);
	const GEN_FLT x49 = x43 * x48;
	const GEN_FLT x50 = -8.0108022e-06 + (-1.60216044e-05 * x43);
	const GEN_FLT x51 = x46 + (x50 * x43);
	const GEN_FLT x52 = x47 + (x51 * x43);
	const GEN_FLT x53 = x48 + (x52 * x43);
	const GEN_FLT x54 = (x53 * x43) + x49;
	const GEN_FLT x55 = sin(x35);
	const GEN_FLT x56 = pow(x31, -1.0 / 2.0);
	const GEN_FLT x57 = tan(x35);
	const GEN_FLT x58 = x57 * x56;
	const GEN_FLT x59 = x58 * x38;
	const GEN_FLT x60 = atan2(-1 * x29, x22);
	const GEN_FLT x61 = x60 + (-1 * asin(x59)) + ogeeMag_0;
	const GEN_FLT x62 = (sin(x61) * ogeePhase_0) + curve_0;
	const GEN_FLT x63 = x62 * x55;
	const GEN_FLT x64 = x36 + (-1 * x63 * x54);
	const GEN_FLT x65 = pow(x64, -1);
	const GEN_FLT x66 = pow(x43, 2);
	const GEN_FLT x67 = x62 * x66;
	const GEN_FLT x68 = x67 * x65;
	const GEN_FLT x69 = x59 + (x68 * x48);
	const GEN_FLT x70 = pow((1 + (-1 * pow(x69, 2))), -1.0 / 2.0);
	const GEN_FLT x71 = x32 * x39;
	const GEN_FLT x72 = pow((1 + (-1 * x71 * pow(x57, 2))), -1.0 / 2.0);
	const GEN_FLT x73 = x0 * lh_qi;
	const GEN_FLT x74 = x2 * lh_qw;
	const GEN_FLT x75 = x74 + x73;
	const GEN_FLT x76 = x75 * x56;
	const GEN_FLT x77 = 2 * x22;
	const GEN_FLT x78 = 2 * x29;
	const GEN_FLT x79 = (x4 * x78) + (x77 * x26);
	const GEN_FLT x80 = 1.0 / 2.0 * x38;
	const GEN_FLT x81 = x80 * pow(x31, -3.0 / 2.0);
	const GEN_FLT x82 = x81 * x57;
	const GEN_FLT x83 = (-1 * x82 * x79) + (x76 * x57);
	const GEN_FLT x84 = cos(x61) * ogeePhase_0;
	const GEN_FLT x85 = x84 * ((-1 * x83 * x72) + x34);
	const GEN_FLT x86 = x65 * x66 * x48;
	const GEN_FLT x87 = 2 * x38;
	const GEN_FLT x88 = x79 + (x87 * x75);
	const GEN_FLT x89 = x80 * pow(x40, -3.0 / 2.0);
	const GEN_FLT x90 = x89 * x37;
	const GEN_FLT x91 = x75 * x41;
	const GEN_FLT x92 = (x91 * x37) + (-1 * x88 * x90);
	const GEN_FLT x93 = pow(x40, -1) * x39;
	const GEN_FLT x94 = pow((1 + (-1 * x93 * pow(x36, -2))), -1.0 / 2.0);
	const GEN_FLT x95 = x94 * x47;
	const GEN_FLT x96 = x94 * x44;
	const GEN_FLT x97 = x94 * x45;
	const GEN_FLT x98 = x92 * x97;
	const GEN_FLT x99 = x94 * x46;
	const GEN_FLT x100 = (x92 * x99) + (x43 * (x98 + (-1 * x92 * x96)));
	const GEN_FLT x101 = (x43 * x100) + (x92 * x95);
	const GEN_FLT x102 = x53 * x94;
	const GEN_FLT x103 = x51 * x94;
	const GEN_FLT x104 = 2.40324066e-05 * x43;
	const GEN_FLT x105 = x94 * x104;
	const GEN_FLT x106 = x50 * x94;
	const GEN_FLT x107 = x52 * x94;
	const GEN_FLT x108 = x94 * x48;
	const GEN_FLT x109 = x54 * x55;
	const GEN_FLT x110 = pow(x64, -2) * x67 * x48;
	const GEN_FLT x111 = 2 * x62 * x65 * x49;
	const GEN_FLT x112 = x94 * x111;
	const GEN_FLT x113 =
		x70 * (x83 +
			   (-1 * x110 *
				((-1 * x85 * x109) +
				 (-1 * x63 *
				  ((x43 * x101) + (x92 * x108) +
				   (x43 * ((x92 * x107) + x101 +
						   (x43 * ((x43 * ((x92 * x106) + (-1 * x92 * x105) + x98)) + x100 + (x92 * x103))))) +
				   (x92 * x102))))) +
			   (x92 * x112) + (x68 * x101) + (x85 * x86));
	const GEN_FLT x114 = cos((-1 * asin(x69)) + x60 + gibPhase_0) * gibMag_0;
	const GEN_FLT x115 = x2 * lh_qj;
	const GEN_FLT x116 = 2 * lh_qi;
	const GEN_FLT x117 = x116 * lh_qw;
	const GEN_FLT x118 = x117 + x115;
	const GEN_FLT x119 = x73 + (-1 * x74);
	const GEN_FLT x120 = ((x30 * x119) + (-1 * x23 * x118)) * x33;
	const GEN_FLT x121 = 1 + (-2 * pow(lh_qi, 2));
	const GEN_FLT x122 = x121 + x24;
	const GEN_FLT x123 = (x78 * x118) + (x77 * x119);
	const GEN_FLT x124 = x81 * x123;
	const GEN_FLT x125 = (-1 * x57 * x124) + (x58 * x122);
	const GEN_FLT x126 = (-1 * x72 * x125) + x120;
	const GEN_FLT x127 = x84 * x86;
	const GEN_FLT x128 = x123 + (x87 * x122);
	const GEN_FLT x129 = x41 * x122;
	const GEN_FLT x130 = (x37 * x129) + (-1 * x90 * x128);
	const GEN_FLT x131 = x94 * x130;
	const GEN_FLT x132 = x45 * x131;
	const GEN_FLT x133 = (x46 * x131) + (x43 * (x132 + (-1 * x44 * x131)));
	const GEN_FLT x134 = (x43 * x133) + (x47 * x131);
	const GEN_FLT x135 = x84 * x109;
	const GEN_FLT x136 =
		x70 * (x125 + (x111 * x131) +
			   (-1 * x110 *
				((-1 * x126 * x135) +
				 (-1 * x63 *
				  ((x108 * x130) + (x43 * x134) +
				   (x43 * (x134 + (x107 * x130) +
						   (x43 * (x133 + (x43 * ((x50 * x131) + (-1 * x104 * x131) + x132)) + (x103 * x130))))) +
				   (x53 * x131))))) +
			   (x68 * x134) + (x127 * x126));
	const GEN_FLT x137 = x121 + x25;
	const GEN_FLT x138 = x1 + x3;
	const GEN_FLT x139 = ((x30 * x138) + (-1 * x23 * x137)) * x33;
	const GEN_FLT x140 = x115 + (-1 * x117);
	const GEN_FLT x141 = x56 * x140;
	const GEN_FLT x142 = (x78 * x137) + (x77 * x138);
	const GEN_FLT x143 = x81 * x142;
	const GEN_FLT x144 = (-1 * x57 * x143) + (x57 * x141);
	const GEN_FLT x145 = (-1 * x72 * x144) + x139;
	const GEN_FLT x146 = x142 + (x87 * x140);
	const GEN_FLT x147 = x41 * x140;
	const GEN_FLT x148 = (x37 * x147) + (-1 * x90 * x146);
	const GEN_FLT x149 = x94 * x148;
	const GEN_FLT x150 = x45 * x149;
	const GEN_FLT x151 = (x46 * x149) + (x43 * (x150 + (-1 * x44 * x149)));
	const GEN_FLT x152 = (x43 * x151) + (x47 * x149);
	const GEN_FLT x153 =
		x70 * (x144 +
			   (-1 * x110 *
				((-1 * x135 * x145) +
				 (-1 * x63 *
				  ((x108 * x148) + (x43 * x152) +
				   (x43 * (x152 + (x52 * x149) +
						   (x43 * ((x43 * ((-1 * x104 * x149) + (x50 * x149) + x150)) + x151 + (x51 * x149))))) +
				   (x53 * x149))))) +
			   (x68 * x152) + (x111 * x149) + (x127 * x145));
	const GEN_FLT x154 = 2 * x5;
	const GEN_FLT x155 = 2 * x7;
	const GEN_FLT x156 = x155 + (-1 * x154);
	const GEN_FLT x157 = 2 * x15;
	const GEN_FLT x158 = 2 * x16;
	const GEN_FLT x159 = x158 + (-1 * x157);
	const GEN_FLT x160 = 2 * x10;
	const GEN_FLT x161 = 2 * x11;
	const GEN_FLT x162 = x161 + (-1 * x160);
	const GEN_FLT x163 = (x162 * lh_qw) + (-1 * x159 * lh_qk) + (x156 * lh_qj);
	const GEN_FLT x164 = (-1 * x156 * lh_qi) + (x159 * lh_qw) + (x162 * lh_qk);
	const GEN_FLT x165 = x156 + (x116 * x164) + (-1 * x0 * x163);
	const GEN_FLT x166 = (x156 * lh_qw) + (-1 * x162 * lh_qj) + (x159 * lh_qi);
	const GEN_FLT x167 = x162 + (x0 * x166) + (-1 * x2 * x164);
	const GEN_FLT x168 = ((x30 * x167) + (-1 * x23 * x165)) * x33;
	const GEN_FLT x169 = x159 + (x2 * x163) + (-1 * x116 * x166);
	const GEN_FLT x170 = (x78 * x165) + (x77 * x167);
	const GEN_FLT x171 = x81 * x170;
	const GEN_FLT x172 = (-1 * x57 * x171) + (x58 * x169);
	const GEN_FLT x173 = (-1 * x72 * x172) + x168;
	const GEN_FLT x174 = x170 + (x87 * x169);
	const GEN_FLT x175 = x41 * x169;
	const GEN_FLT x176 = (x37 * x175) + (-1 * x90 * x174);
	const GEN_FLT x177 = x94 * x176;
	const GEN_FLT x178 = x97 * x176;
	const GEN_FLT x179 = (x99 * x176) + (x43 * (x178 + (-1 * x44 * x177)));
	const GEN_FLT x180 = (x43 * x179) + (x95 * x176);
	const GEN_FLT x181 =
		x70 * (x172 + (x111 * x177) +
			   (-1 * x110 *
				((-1 * x173 * x135) +
				 (-1 * x63 *
				  ((x108 * x176) + (x43 * x180) +
				   (x43 * (x180 + (x107 * x176) +
						   (x43 * (x179 + (x43 * ((x106 * x176) + (-1 * x104 * x177) + x178)) + (x103 * x176))))) +
				   (x102 * x176))))) +
			   (x68 * x180) + (x127 * x173));
	const GEN_FLT x182 = 2 * x14;
	const GEN_FLT x183 = x182 + (-4 * x15) + x158;
	const GEN_FLT x184 = 2 * x6;
	const GEN_FLT x185 = x154 + (-1 * x184) + (-4 * x7);
	const GEN_FLT x186 = 2 * obj_qk * sensor_z;
	const GEN_FLT x187 = 2 * obj_qj * sensor_y;
	const GEN_FLT x188 = x187 + x186;
	const GEN_FLT x189 = (x188 * lh_qw) + (-1 * x185 * lh_qk) + (x183 * lh_qj);
	const GEN_FLT x190 = (x185 * lh_qw) + (-1 * x183 * lh_qi) + (x188 * lh_qk);
	const GEN_FLT x191 = x183 + (x116 * x190) + (-1 * x0 * x189);
	const GEN_FLT x192 = (-1 * x188 * lh_qj) + (x183 * lh_qw) + (x185 * lh_qi);
	const GEN_FLT x193 = x188 + (x0 * x192) + (-1 * x2 * x190);
	const GEN_FLT x194 = ((x30 * x193) + (-1 * x23 * x191)) * x33;
	const GEN_FLT x195 = x185 + (x2 * x189) + (-1 * x116 * x192);
	const GEN_FLT x196 = x56 * x195;
	const GEN_FLT x197 = (x78 * x191) + (x77 * x193);
	const GEN_FLT x198 = x81 * x197;
	const GEN_FLT x199 = (-1 * x57 * x198) + (x57 * x196);
	const GEN_FLT x200 = (-1 * x72 * x199) + x194;
	const GEN_FLT x201 = x89 * (x197 + (x87 * x195));
	const GEN_FLT x202 = x41 * x195;
	const GEN_FLT x203 = (x37 * x202) + (-1 * x37 * x201);
	const GEN_FLT x204 = x97 * x203;
	const GEN_FLT x205 = (x99 * x203) + (x43 * (x204 + (-1 * x96 * x203)));
	const GEN_FLT x206 = (x43 * x205) + (x95 * x203);
	const GEN_FLT x207 =
		x70 * ((x203 * x112) + x199 +
			   (-1 * x110 *
				((-1 * x200 * x135) +
				 (-1 * x63 *
				  ((x203 * x108) + (x43 * x206) +
				   (x43 * ((x203 * x107) + x206 +
						   (x43 * ((x43 * ((x203 * x106) + x204 + (-1 * x203 * x105))) + x205 + (x203 * x103))))) +
				   (x203 * x102))))) +
			   (x68 * x206) + (x200 * x127));
	const GEN_FLT x208 = 2 * x9;
	const GEN_FLT x209 = (-1 * x208) + x160 + (-4 * x11);
	const GEN_FLT x210 = 2 * obj_qi * sensor_x;
	const GEN_FLT x211 = x186 + x210;
	const GEN_FLT x212 = (-4 * x5) + x184 + x155;
	const GEN_FLT x213 = (x212 * lh_qw) + (-1 * x211 * lh_qk) + (x209 * lh_qj);
	const GEN_FLT x214 = (x211 * lh_qw) + (-1 * x209 * lh_qi) + (x212 * lh_qk);
	const GEN_FLT x215 = x209 + (x214 * x116) + (-1 * x0 * x213);
	const GEN_FLT x216 = 2 * ((x209 * lh_qw) + (-1 * x212 * lh_qj) + (x211 * lh_qi));
	const GEN_FLT x217 = x212 + (x216 * lh_qj) + (-1 * x2 * x214);
	const GEN_FLT x218 = ((x30 * x217) + (-1 * x23 * x215)) * x33;
	const GEN_FLT x219 = x211 + (x2 * x213) + (-1 * x216 * lh_qi);
	const GEN_FLT x220 = x56 * x219;
	const GEN_FLT x221 = (x78 * x215) + (x77 * x217);
	const GEN_FLT x222 = (-1 * x82 * x221) + (x57 * x220);
	const GEN_FLT x223 = (-1 * x72 * x222) + x218;
	const GEN_FLT x224 = x221 + (x87 * x219);
	const GEN_FLT x225 = x41 * x219;
	const GEN_FLT x226 = (x37 * x225) + (-1 * x90 * x224);
	const GEN_FLT x227 = x97 * x226;
	const GEN_FLT x228 = (x99 * x226) + (x43 * (x227 + (-1 * x96 * x226)));
	const GEN_FLT x229 = (x43 * x228) + (x95 * x226);
	const GEN_FLT x230 =
		x70 * (x222 + (x226 * x112) +
			   (-1 * x110 *
				((-1 * x223 * x135) +
				 (-1 * x63 *
				  ((x43 * x229) + (x226 * x108) +
				   (x43 * (x229 + (x226 * x107) +
						   (x43 * (x228 + (x43 * ((x226 * x106) + (-1 * x226 * x105) + x227)) + (x226 * x103))))) +
				   (x226 * x102))))) +
			   (x68 * x229) + (x223 * x127));
	const GEN_FLT x231 = x210 + x187;
	const GEN_FLT x232 = x208 + (-4 * x10) + x161;
	const GEN_FLT x233 = (-1 * x182) + x157 + (-4 * x16);
	const GEN_FLT x234 = (x233 * lh_qw) + (-1 * x232 * lh_qk) + (x231 * lh_qj);
	const GEN_FLT x235 = (x232 * lh_qw) + (-1 * x231 * lh_qi) + (x233 * lh_qk);
	const GEN_FLT x236 = x231 + (x235 * x116) + (-1 * x0 * x234);
	const GEN_FLT x237 = (x231 * lh_qw) + (-1 * x233 * lh_qj) + (x232 * lh_qi);
	const GEN_FLT x238 = x233 + (x0 * x237) + (-1 * x2 * x235);
	const GEN_FLT x239 = ((x30 * x238) + (-1 * x23 * x236)) * x33;
	const GEN_FLT x240 = x232 + (x2 * x234) + (-1 * x237 * x116);
	const GEN_FLT x241 = x56 * x240;
	const GEN_FLT x242 = (x78 * x236) + (x77 * x238);
	const GEN_FLT x243 = x81 * x242;
	const GEN_FLT x244 = (-1 * x57 * x243) + (x57 * x241);
	const GEN_FLT x245 = (-1 * x72 * x244) + x239;
	const GEN_FLT x246 = x242 + (x87 * x240);
	const GEN_FLT x247 = x41 * x240;
	const GEN_FLT x248 = (x37 * x247) + (-1 * x90 * x246);
	const GEN_FLT x249 = x97 * x248;
	const GEN_FLT x250 = (x99 * x248) + (x43 * (x249 + (-1 * x96 * x248)));
	const GEN_FLT x251 = (x43 * x250) + (x95 * x248);
	const GEN_FLT x252 =
		x70 * (x244 + (x248 * x112) +
			   (-1 * x110 *
				((-1 * x245 * x135) +
				 (-1 * x63 *
				  ((x248 * x108) + (x43 * x251) +
				   (x43 * (x251 + (x248 * x107) +
						   (x43 * (x250 + (x43 * ((-1 * x248 * x105) + (x248 * x106) + x249)) + (x248 * x103))))) +
				   (x248 * x102))))) +
			   (x68 * x251) + (x245 * x127));
	const GEN_FLT x253 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x254 = cos(x253);
	const GEN_FLT x255 = pow(x254, -1);
	const GEN_FLT x256 = asin(x42 * x255);
	const GEN_FLT x257 = 8.0108022e-06 * x256;
	const GEN_FLT x258 = pow((1 + (-1 * x93 * pow(x254, -2))), -1.0 / 2.0);
	const GEN_FLT x259 = x89 * x255;
	const GEN_FLT x260 = (x91 * x255) + (-1 * x88 * x259);
	const GEN_FLT x261 = x260 * x258;
	const GEN_FLT x262 = -8.0108022e-06 + (-1 * x257);
	const GEN_FLT x263 = x262 * x258;
	const GEN_FLT x264 = x263 * x260;
	const GEN_FLT x265 = 0.0028679863 + (x262 * x256);
	const GEN_FLT x266 = x265 * x258;
	const GEN_FLT x267 = (x266 * x260) + (x256 * (x264 + (-1 * x261 * x257)));
	const GEN_FLT x268 = 5.3685255e-06 + (x265 * x256);
	const GEN_FLT x269 = x268 * x258;
	const GEN_FLT x270 = (x269 * x260) + (x267 * x256);
	const GEN_FLT x271 = 0.0076069798 + (x268 * x256);
	const GEN_FLT x272 = x271 * x256;
	const GEN_FLT x273 = -8.0108022e-06 + (-1.60216044e-05 * x256);
	const GEN_FLT x274 = x265 + (x273 * x256);
	const GEN_FLT x275 = x268 + (x274 * x256);
	const GEN_FLT x276 = x271 + (x275 * x256);
	const GEN_FLT x277 = (x276 * x256) + x272;
	const GEN_FLT x278 = sin(x253);
	const GEN_FLT x279 = tan(x253);
	const GEN_FLT x280 = x56 * x279;
	const GEN_FLT x281 = -1 * x38 * x280;
	const GEN_FLT x282 = x60 + (-1 * asin(x281)) + ogeeMag_1;
	const GEN_FLT x283 = (sin(x282) * ogeePhase_1) + curve_1;
	const GEN_FLT x284 = x278 * x283;
	const GEN_FLT x285 = x254 + (x277 * x284);
	const GEN_FLT x286 = pow(x285, -1);
	const GEN_FLT x287 = pow(x256, 2);
	const GEN_FLT x288 = x287 * x283;
	const GEN_FLT x289 = x286 * x288;
	const GEN_FLT x290 = pow((1 + (-1 * x71 * pow(x279, 2))), -1.0 / 2.0);
	const GEN_FLT x291 = x81 * x279;
	const GEN_FLT x292 = (x79 * x291) + (-1 * x76 * x279);
	const GEN_FLT x293 = cos(x282) * ogeePhase_1;
	const GEN_FLT x294 = x293 * ((-1 * x292 * x290) + x34);
	const GEN_FLT x295 = x271 * x286 * x287;
	const GEN_FLT x296 = 2 * x272 * x286 * x283;
	const GEN_FLT x297 = x275 * x258;
	const GEN_FLT x298 = 2.40324066e-05 * x256;
	const GEN_FLT x299 = x273 * x258;
	const GEN_FLT x300 = x274 * x258;
	const GEN_FLT x301 = x276 * x258;
	const GEN_FLT x302 = x271 * x258;
	const GEN_FLT x303 = x278 * x277;
	const GEN_FLT x304 = x271 * x288 * pow(x285, -2);
	const GEN_FLT x305 = x281 + (x271 * x289);
	const GEN_FLT x306 = pow((1 + (-1 * pow(x305, 2))), -1.0 / 2.0);
	const GEN_FLT x307 =
		(-1 * x306 *
		 (x292 +
		  (-1 * x304 *
		   ((x294 * x303) +
			(x284 *
			 ((x260 * x302) + (x270 * x256) + (x260 * x301) +
			  (x256 * (x270 + (x256 * (x267 + (x260 * x300) + (x256 * ((x299 * x260) + (-1 * x298 * x261) + x264)))) +
					   (x297 * x260))))))) +
		  (x296 * x261) + (x295 * x294) + (x270 * x289))) +
		x34;
	const GEN_FLT x308 = cos(x60 + (-1 * asin(x305)) + gibPhase_1) * gibMag_1;
	const GEN_FLT x309 = (x255 * x129) + (-1 * x259 * x128);
	const GEN_FLT x310 = x258 * x257;
	const GEN_FLT x311 = x263 * x309;
	const GEN_FLT x312 = (x266 * x309) + (x256 * (x311 + (-1 * x309 * x310)));
	const GEN_FLT x313 = (x269 * x309) + (x256 * x312);
	const GEN_FLT x314 = (x279 * x124) + (-1 * x280 * x122);
	const GEN_FLT x315 = (-1 * x290 * x314) + x120;
	const GEN_FLT x316 = x293 * x295;
	const GEN_FLT x317 = x296 * x258;
	const GEN_FLT x318 = x298 * x258;
	const GEN_FLT x319 = x293 * x303;
	const GEN_FLT x320 =
		(-1 * x306 *
		 (x314 +
		  (-1 * x304 *
		   ((x315 * x319) +
			(x284 *
			 ((x309 * x302) + (x256 * x313) + (x309 * x301) +
			  (x256 * (x313 + (x256 * (x312 + (x309 * x300) + (x256 * ((x299 * x309) + x311 + (-1 * x309 * x318))))) +
					   (x297 * x309))))))) +
		  (x309 * x317) + (x315 * x316) + (x289 * x313))) +
		x120;
	const GEN_FLT x321 = ((x255 * x147) + (-1 * x259 * x146)) * x258;
	const GEN_FLT x322 = x262 * x321;
	const GEN_FLT x323 = (x265 * x321) + (x256 * (x322 + (-1 * x257 * x321)));
	const GEN_FLT x324 = (x268 * x321) + (x256 * x323);
	const GEN_FLT x325 = (x279 * x143) + (-1 * x279 * x141);
	const GEN_FLT x326 = (-1 * x290 * x325) + x139;
	const GEN_FLT x327 =
		(-1 * x306 *
		 ((-1 * x304 *
		   ((x326 * x319) +
			(x284 *
			 ((x271 * x321) + (x256 * x324) + (x276 * x321) +
			  (x256 * (x324 + (x256 * (x323 + (x274 * x321) + (x256 * ((-1 * x298 * x321) + (x273 * x321) + x322)))) +
					   (x275 * x321))))))) +
		  (x296 * x321) + (x326 * x316) + x325 + (x289 * x324))) +
		x139;
	const GEN_FLT x328 = (x255 * x175) + (-1 * x259 * x174);
	const GEN_FLT x329 = x263 * x328;
	const GEN_FLT x330 = (x266 * x328) + (x256 * (x329 + (-1 * x328 * x310)));
	const GEN_FLT x331 = (x269 * x328) + (x256 * x330);
	const GEN_FLT x332 = (x279 * x171) + (-1 * x280 * x169);
	const GEN_FLT x333 = (-1 * x290 * x332) + x168;
	const GEN_FLT x334 =
		(-1 * x306 *
		 (x332 + (x328 * x317) + (x333 * x316) +
		  (-1 * x304 *
		   ((x333 * x319) +
			(x284 *
			 ((x302 * x328) + (x301 * x328) +
			  (x256 * (x331 + (x256 * (x330 + (x300 * x328) + (x256 * ((x299 * x328) + (-1 * x328 * x318) + x329)))) +
					   (x297 * x328))) +
			  (x256 * x331))))) +
		  (x289 * x331))) +
		x168;
	const GEN_FLT x335 = (x202 * x255) + (-1 * x201 * x255);
	const GEN_FLT x336 = x263 * x335;
	const GEN_FLT x337 = (x266 * x335) + (x256 * (x336 + (-1 * x310 * x335)));
	const GEN_FLT x338 = (x269 * x335) + (x256 * x337);
	const GEN_FLT x339 = (x279 * x198) + (-1 * x279 * x196);
	const GEN_FLT x340 = (-1 * x290 * x339) + x194;
	const GEN_FLT x341 =
		(-1 * x306 *
		 ((-1 * x304 *
		   ((x319 * x340) +
			(x284 *
			 ((x256 * x338) + (x302 * x335) + (x301 * x335) +
			  (x256 * (x338 + (x256 * (x337 + (x300 * x335) + (x256 * ((x299 * x335) + (-1 * x335 * x318) + x336)))) +
					   (x297 * x335))))))) +
		  (x335 * x317) + x339 + (x316 * x340) + (x289 * x338))) +
		x194;
	const GEN_FLT x342 = (x255 * x225) + (-1 * x259 * x224);
	const GEN_FLT x343 = x263 * x342;
	const GEN_FLT x344 = (x266 * x342) + (x256 * (x343 + (-1 * x310 * x342)));
	const GEN_FLT x345 = (x269 * x342) + (x256 * x344);
	const GEN_FLT x346 = (x291 * x221) + (-1 * x279 * x220);
	const GEN_FLT x347 = (-1 * x290 * x346) + x218;
	const GEN_FLT x348 =
		(-1 * x306 *
		 ((x317 * x342) +
		  (-1 * x304 *
		   ((x347 * x319) +
			(x284 *
			 ((x302 * x342) + (x256 * x345) + (x301 * x342) +
			  (x256 * (x345 + (x256 * (x344 + (x300 * x342) + (x256 * ((x299 * x342) + (-1 * x318 * x342) + x343)))) +
					   (x297 * x342))))))) +
		  x346 + (x347 * x316) + (x289 * x345))) +
		x218;
	const GEN_FLT x349 = ((x255 * x247) + (-1 * x259 * x246)) * x258;
	const GEN_FLT x350 = x262 * x349;
	const GEN_FLT x351 = (x265 * x349) + (x256 * (x350 + (-1 * x257 * x349)));
	const GEN_FLT x352 = (x268 * x349) + (x256 * x351);
	const GEN_FLT x353 = (x279 * x243) + (-1 * x279 * x241);
	const GEN_FLT x354 = (-1 * x290 * x353) + x239;
	const GEN_FLT x355 =
		(-1 * x306 *
		 ((-1 * x304 *
		   ((x354 * x319) +
			(x284 *
			 ((x271 * x349) + (x256 * x352) + (x276 * x349) +
			  (x256 * (x352 + (x256 * (x351 + (x274 * x349) + (x256 * ((x273 * x349) + (-1 * x298 * x349) + x350)))) +
					   (x275 * x349))))))) +
		  x353 + (x354 * x316) + (x296 * x349) + (x289 * x352))) +
		x239;
	out[0] = (-1 * x114 * (x113 + (-1 * x34))) + (-1 * x113) + x34;
	out[1] = (-1 * (x136 + (-1 * x120)) * x114) + x120 + (-1 * x136);
	out[2] = (-1 * (x153 + (-1 * x139)) * x114) + (-1 * x153) + x139;
	out[3] = (-1 * x181) + (-1 * (x181 + (-1 * x168)) * x114) + x168;
	out[4] = (-1 * (x207 + (-1 * x194)) * x114) + (-1 * x207) + x194;
	out[5] = (-1 * (x230 + (-1 * x218)) * x114) + x218 + (-1 * x230);
	out[6] = (-1 * (x252 + (-1 * x239)) * x114) + (-1 * x252) + x239;
	out[7] = x307 + (x307 * x308);
	out[8] = x320 + (x308 * x320);
	out[9] = x327 + (x308 * x327);
	out[10] = x334 + (x308 * x334);
	out[11] = x341 + (x308 * x341);
	out[12] = x348 + (x308 * x348);
	out[13] = x355 + (x355 * x308);
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
	const GEN_FLT x0 = 2 * obj_qj;
	const GEN_FLT x1 = x0 * obj_qw;
	const GEN_FLT x2 = 2 * obj_qk;
	const GEN_FLT x3 = x2 * obj_qi;
	const GEN_FLT x4 = x3 + (-1 * x1);
	const GEN_FLT x5 = x0 * obj_qi;
	const GEN_FLT x6 = x2 * obj_qw;
	const GEN_FLT x7 = x6 + x5;
	const GEN_FLT x8 = -2 * pow(obj_qk, 2);
	const GEN_FLT x9 = 1 + (-2 * pow(obj_qj, 2));
	const GEN_FLT x10 = x9 + x8;
	const GEN_FLT x11 = (x10 * lh_qw) + (-1 * x7 * lh_qk) + (x4 * lh_qj);
	const GEN_FLT x12 = 2 * lh_qj;
	const GEN_FLT x13 = (x7 * lh_qw) + (-1 * x4 * lh_qi) + (x10 * lh_qk);
	const GEN_FLT x14 = 2 * lh_qi;
	const GEN_FLT x15 = x4 + (x14 * x13) + (-1 * x12 * x11);
	const GEN_FLT x16 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x17 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x18 = sensor_y + (2 * ((x17 * obj_qk) + (-1 * x16 * obj_qi))) + obj_py;
	const GEN_FLT x19 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x20 = sensor_z + (2 * ((x19 * obj_qi) + (-1 * x17 * obj_qj))) + obj_pz;
	const GEN_FLT x21 = sensor_x + (2 * ((x16 * obj_qj) + (-1 * x19 * obj_qk))) + obj_px;
	const GEN_FLT x22 = (x21 * lh_qk) + (-1 * x20 * lh_qi) + (x18 * lh_qw);
	const GEN_FLT x23 = (x18 * lh_qi) + (-1 * x21 * lh_qj) + (x20 * lh_qw);
	const GEN_FLT x24 = x21 + (2 * ((x23 * lh_qj) + (-1 * x22 * lh_qk))) + lh_px;
	const GEN_FLT x25 = pow(x24, -1);
	const GEN_FLT x26 = 2 * lh_qk;
	const GEN_FLT x27 = (x4 * lh_qw) + (-1 * x10 * lh_qj) + (x7 * lh_qi);
	const GEN_FLT x28 = x10 + (x27 * x12) + (-1 * x26 * x13);
	const GEN_FLT x29 = pow(x24, 2);
	const GEN_FLT x30 = (x20 * lh_qj) + (-1 * x18 * lh_qk) + (x21 * lh_qw);
	const GEN_FLT x31 = x20 + (2 * ((x22 * lh_qi) + (-1 * x30 * lh_qj))) + lh_pz;
	const GEN_FLT x32 = x31 * pow(x29, -1);
	const GEN_FLT x33 = x29 + pow(x31, 2);
	const GEN_FLT x34 = pow(x33, -1);
	const GEN_FLT x35 = x34 * x29;
	const GEN_FLT x36 = ((x32 * x28) + (-1 * x25 * x15)) * x35;
	const GEN_FLT x37 = 0.523598775598299 + tilt_0;
	const GEN_FLT x38 = cos(x37);
	const GEN_FLT x39 = pow(x38, -1);
	const GEN_FLT x40 = x18 + (2 * ((x30 * lh_qk) + (-1 * x23 * lh_qi))) + lh_py;
	const GEN_FLT x41 = pow(x40, 2);
	const GEN_FLT x42 = x33 + x41;
	const GEN_FLT x43 = pow(x42, -1.0 / 2.0);
	const GEN_FLT x44 = x40 * x43;
	const GEN_FLT x45 = asin(x44 * x39);
	const GEN_FLT x46 = 8.0108022e-06 * x45;
	const GEN_FLT x47 = -8.0108022e-06 + (-1 * x46);
	const GEN_FLT x48 = 0.0028679863 + (x45 * x47);
	const GEN_FLT x49 = 5.3685255e-06 + (x45 * x48);
	const GEN_FLT x50 = 0.0076069798 + (x45 * x49);
	const GEN_FLT x51 = x50 * x45;
	const GEN_FLT x52 = -8.0108022e-06 + (-1.60216044e-05 * x45);
	const GEN_FLT x53 = x48 + (x52 * x45);
	const GEN_FLT x54 = x49 + (x53 * x45);
	const GEN_FLT x55 = x50 + (x54 * x45);
	const GEN_FLT x56 = (x55 * x45) + x51;
	const GEN_FLT x57 = sin(x37);
	const GEN_FLT x58 = tan(x37);
	const GEN_FLT x59 = pow(x33, -1.0 / 2.0);
	const GEN_FLT x60 = x59 * x40;
	const GEN_FLT x61 = x60 * x58;
	const GEN_FLT x62 = atan2(-1 * x31, x24);
	const GEN_FLT x63 = x62 + (-1 * asin(x61)) + ogeeMag_0;
	const GEN_FLT x64 = (sin(x63) * ogeePhase_0) + curve_0;
	const GEN_FLT x65 = x64 * x57;
	const GEN_FLT x66 = x38 + (-1 * x65 * x56);
	const GEN_FLT x67 = pow(x66, -1);
	const GEN_FLT x68 = pow(x45, 2);
	const GEN_FLT x69 = x64 * x68;
	const GEN_FLT x70 = x67 * x69;
	const GEN_FLT x71 = x61 + (x70 * x50);
	const GEN_FLT x72 = pow((1 + (-1 * pow(x71, 2))), -1.0 / 2.0);
	const GEN_FLT x73 = x7 + (x26 * x11) + (-1 * x27 * x14);
	const GEN_FLT x74 = x73 * x59;
	const GEN_FLT x75 = 2 * x24;
	const GEN_FLT x76 = 2 * x31;
	const GEN_FLT x77 = (x76 * x15) + (x75 * x28);
	const GEN_FLT x78 = 1.0 / 2.0 * x40;
	const GEN_FLT x79 = x78 * pow(x33, -3.0 / 2.0);
	const GEN_FLT x80 = x79 * x58;
	const GEN_FLT x81 = (-1 * x80 * x77) + (x74 * x58);
	const GEN_FLT x82 = x41 * x34;
	const GEN_FLT x83 = pow((1 + (-1 * x82 * pow(x58, 2))), -1.0 / 2.0);
	const GEN_FLT x84 = (-1 * x81 * x83) + x36;
	const GEN_FLT x85 = cos(x63) * ogeePhase_0;
	const GEN_FLT x86 = x67 * x68 * x50;
	const GEN_FLT x87 = x85 * x86;
	const GEN_FLT x88 = x41 * pow(x42, -1);
	const GEN_FLT x89 = pow((1 + (-1 * x88 * pow(x38, -2))), -1.0 / 2.0);
	const GEN_FLT x90 = 2 * x40;
	const GEN_FLT x91 = x78 * pow(x42, -3.0 / 2.0);
	const GEN_FLT x92 = x91 * (x77 + (x73 * x90));
	const GEN_FLT x93 = x73 * x43;
	const GEN_FLT x94 = (x93 * x39) + (-1 * x92 * x39);
	const GEN_FLT x95 = x89 * x94;
	const GEN_FLT x96 = x95 * x47;
	const GEN_FLT x97 = (x95 * x48) + (x45 * (x96 + (-1 * x95 * x46)));
	const GEN_FLT x98 = (x97 * x45) + (x95 * x49);
	const GEN_FLT x99 = x89 * x55;
	const GEN_FLT x100 = 2.40324066e-05 * x45;
	const GEN_FLT x101 = x57 * x56;
	const GEN_FLT x102 = x85 * x101;
	const GEN_FLT x103 = pow(x66, -2) * x69 * x50;
	const GEN_FLT x104 = 2 * x64 * x67 * x51;
	const GEN_FLT x105 =
		x72 * (x81 + (x95 * x104) +
			   (-1 * x103 *
				((-1 * x84 * x102) +
				 (-1 * x65 *
				  ((x50 * x95) + (x98 * x45) +
				   (x45 * (x98 + (x54 * x95) +
						   (x45 * (x97 + (x45 * ((x52 * x95) + (-1 * x95 * x100) + x96)) + (x53 * x95))))) +
				   (x99 * x94))))) +
			   (x70 * x98) + (x84 * x87));
	const GEN_FLT x106 = cos(x62 + (-1 * asin(x71)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x107 = x2 * obj_qj;
	const GEN_FLT x108 = 2 * obj_qw * obj_qi;
	const GEN_FLT x109 = x108 + x107;
	const GEN_FLT x110 = -2 * pow(obj_qi, 2);
	const GEN_FLT x111 = 1 + x8 + x110;
	const GEN_FLT x112 = x5 + (-1 * x6);
	const GEN_FLT x113 = (x112 * lh_qw) + (-1 * x111 * lh_qk) + (x109 * lh_qj);
	const GEN_FLT x114 = (-1 * x109 * lh_qi) + (x111 * lh_qw) + (x112 * lh_qk);
	const GEN_FLT x115 = x109 + (x14 * x114) + (-1 * x12 * x113);
	const GEN_FLT x116 = (x109 * lh_qw) + (-1 * x112 * lh_qj) + (x111 * lh_qi);
	const GEN_FLT x117 = x112 + (x12 * x116) + (-1 * x26 * x114);
	const GEN_FLT x118 = ((x32 * x117) + (-1 * x25 * x115)) * x35;
	const GEN_FLT x119 = (x26 * x113) + x111 + (-1 * x14 * x116);
	const GEN_FLT x120 = x59 * x119;
	const GEN_FLT x121 = (x76 * x115) + (x75 * x117);
	const GEN_FLT x122 = (-1 * x80 * x121) + (x58 * x120);
	const GEN_FLT x123 = x85 * ((-1 * x83 * x122) + x118);
	const GEN_FLT x124 = x91 * (x121 + (x90 * x119));
	const GEN_FLT x125 = x43 * x119;
	const GEN_FLT x126 = (x39 * x125) + (-1 * x39 * x124);
	const GEN_FLT x127 = x89 * x126;
	const GEN_FLT x128 = x47 * x127;
	const GEN_FLT x129 = (x48 * x127) + (x45 * (x128 + (-1 * x46 * x127)));
	const GEN_FLT x130 = (x45 * x129) + (x49 * x127);
	const GEN_FLT x131 =
		x72 * ((x104 * x127) +
			   (-1 * x103 *
				((-1 * x101 * x123) +
				 (-1 * x65 *
				  ((x50 * x127) + (x45 * x130) + (x99 * x126) +
				   (x45 * (x130 + (x54 * x127) +
						   (x45 * (x129 + (x45 * ((x52 * x127) + (-1 * x100 * x127) + x128)) + (x53 * x127))))))))) +
			   (x70 * x130) + x122 + (x86 * x123));
	const GEN_FLT x132 = x9 + x110;
	const GEN_FLT x133 = x107 + (-1 * x108);
	const GEN_FLT x134 = x1 + x3;
	const GEN_FLT x135 = (x134 * lh_qw) + (-1 * x133 * lh_qk) + (x132 * lh_qj);
	const GEN_FLT x136 = (x133 * lh_qw) + (-1 * x132 * lh_qi) + (x134 * lh_qk);
	const GEN_FLT x137 = x132 + (x14 * x136) + (-1 * x12 * x135);
	const GEN_FLT x138 = (x132 * lh_qw) + (-1 * x134 * lh_qj) + (x133 * lh_qi);
	const GEN_FLT x139 = (x12 * x138) + x134 + (-1 * x26 * x136);
	const GEN_FLT x140 = ((x32 * x139) + (-1 * x25 * x137)) * x35;
	const GEN_FLT x141 = x133 + (x26 * x135) + (-1 * x14 * x138);
	const GEN_FLT x142 = x59 * x141;
	const GEN_FLT x143 = (x76 * x137) + (x75 * x139);
	const GEN_FLT x144 = (-1 * x80 * x143) + (x58 * x142);
	const GEN_FLT x145 = (-1 * x83 * x144) + x140;
	const GEN_FLT x146 = x91 * (x143 + (x90 * x141));
	const GEN_FLT x147 = x43 * x141;
	const GEN_FLT x148 = (x39 * x147) + (-1 * x39 * x146);
	const GEN_FLT x149 = x89 * x148;
	const GEN_FLT x150 = x47 * x149;
	const GEN_FLT x151 = (x48 * x149) + (x45 * (x150 + (-1 * x46 * x149)));
	const GEN_FLT x152 = (x45 * x151) + (x49 * x149);
	const GEN_FLT x153 =
		x72 * (x144 + (x104 * x149) +
			   (-1 * x103 *
				((-1 * x102 * x145) +
				 (-1 * x65 *
				  ((x50 * x149) + (x45 * x152) +
				   (x45 * (x152 + (x54 * x149) +
						   (x45 * (x151 + (x45 * ((x52 * x149) + (-1 * x100 * x149) + x150)) + (x53 * x149))))) +
				   (x99 * x148))))) +
			   (x70 * x152) + (x87 * x145));
	const GEN_FLT x154 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x155 = cos(x154);
	const GEN_FLT x156 = pow(x155, -1);
	const GEN_FLT x157 = asin(x44 * x156);
	const GEN_FLT x158 = 8.0108022e-06 * x157;
	const GEN_FLT x159 = pow((1 + (-1 * x88 * pow(x155, -2))), -1.0 / 2.0);
	const GEN_FLT x160 = ((x93 * x156) + (-1 * x92 * x156)) * x159;
	const GEN_FLT x161 = -8.0108022e-06 + (-1 * x158);
	const GEN_FLT x162 = x161 * x160;
	const GEN_FLT x163 = 0.0028679863 + (x161 * x157);
	const GEN_FLT x164 = (x160 * x163) + (x157 * (x162 + (-1 * x160 * x158)));
	const GEN_FLT x165 = 5.3685255e-06 + (x163 * x157);
	const GEN_FLT x166 = (x160 * x165) + (x164 * x157);
	const GEN_FLT x167 = 0.0076069798 + (x165 * x157);
	const GEN_FLT x168 = x167 * x157;
	const GEN_FLT x169 = -8.0108022e-06 + (-1.60216044e-05 * x157);
	const GEN_FLT x170 = x163 + (x169 * x157);
	const GEN_FLT x171 = x165 + (x170 * x157);
	const GEN_FLT x172 = x167 + (x171 * x157);
	const GEN_FLT x173 = (x172 * x157) + x168;
	const GEN_FLT x174 = sin(x154);
	const GEN_FLT x175 = tan(x154);
	const GEN_FLT x176 = -1 * x60 * x175;
	const GEN_FLT x177 = x62 + (-1 * asin(x176)) + ogeeMag_1;
	const GEN_FLT x178 = (sin(x177) * ogeePhase_1) + curve_1;
	const GEN_FLT x179 = x178 * x174;
	const GEN_FLT x180 = x155 + (x179 * x173);
	const GEN_FLT x181 = pow(x180, -1);
	const GEN_FLT x182 = pow(x157, 2);
	const GEN_FLT x183 = x178 * x182;
	const GEN_FLT x184 = x181 * x183;
	const GEN_FLT x185 = x79 * x175;
	const GEN_FLT x186 = (x77 * x185) + (-1 * x74 * x175);
	const GEN_FLT x187 = pow((1 + (-1 * x82 * pow(x175, 2))), -1.0 / 2.0);
	const GEN_FLT x188 = (-1 * x187 * x186) + x36;
	const GEN_FLT x189 = cos(x177) * ogeePhase_1;
	const GEN_FLT x190 = x167 * x181 * x182;
	const GEN_FLT x191 = x189 * x190;
	const GEN_FLT x192 = 2 * x168 * x178 * x181;
	const GEN_FLT x193 = 2.40324066e-05 * x157;
	const GEN_FLT x194 = x174 * x173;
	const GEN_FLT x195 = x189 * x194;
	const GEN_FLT x196 = x167 * pow(x180, -2) * x183;
	const GEN_FLT x197 = x176 + (x167 * x184);
	const GEN_FLT x198 = pow((1 + (-1 * pow(x197, 2))), -1.0 / 2.0);
	const GEN_FLT x199 =
		(-1 * x198 *
		 (x186 +
		  (-1 * x196 *
		   ((x188 * x195) +
			(x179 *
			 ((x160 * x167) + (x166 * x157) + (x160 * x172) +
			  (x157 * (x166 + (x157 * (x164 + (x160 * x170) + (x157 * ((x160 * x169) + (-1 * x160 * x193) + x162)))) +
					   (x160 * x171))))))) +
		  (x160 * x192) + (x188 * x191) + (x166 * x184))) +
		x36;
	const GEN_FLT x200 = cos(x62 + (-1 * asin(x197)) + gibPhase_1) * gibMag_1;
	const GEN_FLT x201 = ((x125 * x156) + (-1 * x124 * x156)) * x159;
	const GEN_FLT x202 = x201 * x161;
	const GEN_FLT x203 = (x201 * x163) + (x157 * (x202 + (-1 * x201 * x158)));
	const GEN_FLT x204 = (x201 * x165) + (x203 * x157);
	const GEN_FLT x205 = (x121 * x185) + (-1 * x120 * x175);
	const GEN_FLT x206 = x189 * ((-1 * x205 * x187) + x118);
	const GEN_FLT x207 =
		(-1 * x198 *
		 (x205 + (x201 * x192) +
		  (-1 * x196 *
		   ((x206 * x194) +
			(x179 *
			 ((x201 * x167) + (x204 * x157) + (x201 * x172) +
			  (x157 * (x204 + (x157 * ((x201 * x170) + x203 + (x157 * ((x201 * x169) + (-1 * x201 * x193) + x202)))) +
					   (x201 * x171))))))) +
		  (x206 * x190) + (x204 * x184))) +
		x118;
	const GEN_FLT x208 = ((x147 * x156) + (-1 * x146 * x156)) * x159;
	const GEN_FLT x209 = x208 * x161;
	const GEN_FLT x210 = (x208 * x163) + (x157 * (x209 + (-1 * x208 * x158)));
	const GEN_FLT x211 = (x208 * x165) + (x210 * x157);
	const GEN_FLT x212 = (x185 * x143) + (-1 * x175 * x142);
	const GEN_FLT x213 = (-1 * x212 * x187) + x140;
	const GEN_FLT x214 =
		(-1 * x198 *
		 (x212 +
		  (-1 * x196 *
		   ((x213 * x195) +
			(x179 *
			 ((x208 * x167) + (x211 * x157) + (x208 * x172) +
			  (x157 * (x211 + (x157 * (x210 + (x208 * x170) + (x157 * ((x208 * x169) + (-1 * x208 * x193) + x209)))) +
					   (x208 * x171))))))) +
		  (x208 * x192) + (x213 * x191) + (x211 * x184))) +
		x140;
	out[0] = (-1 * x106 * (x105 + (-1 * x36))) + (-1 * x105) + x36;
	out[1] = (-1 * (x131 + (-1 * x118)) * x106) + (-1 * x131) + x118;
	out[2] = (-1 * (x153 + (-1 * x140)) * x106) + (-1 * x153) + x140;
	out[3] = x199 + (x200 * x199);
	out[4] = x207 + (x200 * x207);
	out[5] = x214 + (x214 * x200);
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
	const GEN_FLT x0 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x1 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x2 = 2 * ((x1 * obj_qj) + (-1 * x0 * obj_qk));
	const GEN_FLT x3 = sensor_x + x2 + obj_px;
	const GEN_FLT x4 = x3 * lh_qw;
	const GEN_FLT x5 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x6 = 2 * ((x5 * obj_qk) + (-1 * x1 * obj_qi));
	const GEN_FLT x7 = sensor_y + x6 + obj_py;
	const GEN_FLT x8 = x7 * lh_qk;
	const GEN_FLT x9 = 2 * ((x0 * obj_qi) + (-1 * x5 * obj_qj));
	const GEN_FLT x10 = sensor_z + x9 + obj_pz;
	const GEN_FLT x11 = x10 * lh_qj;
	const GEN_FLT x12 = (-1 * x8) + x11 + x4;
	const GEN_FLT x13 = x7 * lh_qw;
	const GEN_FLT x14 = x10 * lh_qi;
	const GEN_FLT x15 = x3 * lh_qk;
	const GEN_FLT x16 = x15 + (-1 * x14) + x13;
	const GEN_FLT x17 = x10 + (2 * ((x16 * lh_qi) + (-1 * x12 * lh_qj))) + lh_pz;
	const GEN_FLT x18 = x10 * lh_qw;
	const GEN_FLT x19 = x3 * lh_qj;
	const GEN_FLT x20 = x7 * lh_qi;
	const GEN_FLT x21 = x20 + (-1 * x19) + x18;
	const GEN_FLT x22 = x3 + (2 * ((x21 * lh_qj) + (-1 * x16 * lh_qk))) + lh_px;
	const GEN_FLT x23 = pow(x22, 2);
	const GEN_FLT x24 = x23 + pow(x17, 2);
	const GEN_FLT x25 = pow(x24, -1);
	const GEN_FLT x26 = x25 * x17;
	const GEN_FLT x27 = x7 + (2 * ((x12 * lh_qk) + (-1 * x21 * lh_qi))) + lh_py;
	const GEN_FLT x28 = 0.523598775598299 + tilt_0;
	const GEN_FLT x29 = cos(x28);
	const GEN_FLT x30 = pow(x29, -1);
	const GEN_FLT x31 = pow(x27, 2);
	const GEN_FLT x32 = x24 + x31;
	const GEN_FLT x33 = pow(x32, -1.0 / 2.0);
	const GEN_FLT x34 = x30 * x33;
	const GEN_FLT x35 = asin(x34 * x27);
	const GEN_FLT x36 = 8.0108022e-06 * x35;
	const GEN_FLT x37 = -8.0108022e-06 + (-1 * x36);
	const GEN_FLT x38 = 0.0028679863 + (x35 * x37);
	const GEN_FLT x39 = 5.3685255e-06 + (x35 * x38);
	const GEN_FLT x40 = 0.0076069798 + (x35 * x39);
	const GEN_FLT x41 = x40 * x35;
	const GEN_FLT x42 = -8.0108022e-06 + (-1.60216044e-05 * x35);
	const GEN_FLT x43 = x38 + (x42 * x35);
	const GEN_FLT x44 = x39 + (x43 * x35);
	const GEN_FLT x45 = x40 + (x44 * x35);
	const GEN_FLT x46 = (x45 * x35) + x41;
	const GEN_FLT x47 = sin(x28);
	const GEN_FLT x48 = pow(x24, -1.0 / 2.0);
	const GEN_FLT x49 = tan(x28);
	const GEN_FLT x50 = x48 * x49;
	const GEN_FLT x51 = x50 * x27;
	const GEN_FLT x52 = atan2(-1 * x17, x22);
	const GEN_FLT x53 = x52 + (-1 * asin(x51)) + ogeeMag_0;
	const GEN_FLT x54 = (sin(x53) * ogeePhase_0) + curve_0;
	const GEN_FLT x55 = x54 * x47;
	const GEN_FLT x56 = x29 + (-1 * x55 * x46);
	const GEN_FLT x57 = pow(x56, -1);
	const GEN_FLT x58 = pow(x35, 2);
	const GEN_FLT x59 = x54 * x58;
	const GEN_FLT x60 = x57 * x59;
	const GEN_FLT x61 = x51 + (x60 * x40);
	const GEN_FLT x62 = pow((1 + (-1 * pow(x61, 2))), -1.0 / 2.0);
	const GEN_FLT x63 = pow(x24, -3.0 / 2.0) * x27;
	const GEN_FLT x64 = x63 * x22;
	const GEN_FLT x65 = x64 * x49;
	const GEN_FLT x66 = x31 * x25;
	const GEN_FLT x67 = pow((1 + (-1 * x66 * pow(x49, 2))), -1.0 / 2.0);
	const GEN_FLT x68 = (x67 * x65) + x26;
	const GEN_FLT x69 = cos(x53) * ogeePhase_0;
	const GEN_FLT x70 = x58 * x57 * x40;
	const GEN_FLT x71 = x70 * x69;
	const GEN_FLT x72 = pow(x32, -3.0 / 2.0);
	const GEN_FLT x73 = x72 * x27;
	const GEN_FLT x74 = x73 * x22;
	const GEN_FLT x75 = pow(x32, -1) * x31;
	const GEN_FLT x76 = pow((1 + (-1 * x75 * pow(x29, -2))), -1.0 / 2.0);
	const GEN_FLT x77 = x76 * x30;
	const GEN_FLT x78 = x74 * x77;
	const GEN_FLT x79 = -1 * x78 * x37;
	const GEN_FLT x80 = (-1 * x78 * x38) + (x35 * (x79 + (x78 * x36)));
	const GEN_FLT x81 = (x80 * x35) + (-1 * x78 * x39);
	const GEN_FLT x82 = 2.40324066e-05 * x35;
	const GEN_FLT x83 = x46 * x47;
	const GEN_FLT x84 = x83 * x69;
	const GEN_FLT x85 = pow(x56, -2) * x59 * x40;
	const GEN_FLT x86 = 2 * x27;
	const GEN_FLT x87 = x86 * x72 * x22;
	const GEN_FLT x88 = x54 * x57 * x41;
	const GEN_FLT x89 = x88 * x77;
	const GEN_FLT x90 =
		x62 * ((-1 * x89 * x87) +
			   (-1 * x85 *
				((-1 * x84 * x68) +
				 (-1 * x55 *
				  ((-1 * x78 * x40) + (-1 * x78 * x45) + (x81 * x35) +
				   (x35 * (x81 + (-1 * x78 * x44) +
						   (x35 * (x80 + (x35 * ((-1 * x78 * x42) + x79 + (x82 * x78))) + (-1 * x78 * x43))))))))) +
			   (x81 * x60) + (x71 * x68) + (-1 * x65));
	const GEN_FLT x91 = cos(x52 + (-1 * asin(x61)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x92 = x67 * x50;
	const GEN_FLT x93 = x72 * x31;
	const GEN_FLT x94 = x76 * (x34 + (-1 * x93 * x30));
	const GEN_FLT x95 = x94 * x37;
	const GEN_FLT x96 = (x94 * x38) + (x35 * (x95 + (-1 * x94 * x36)));
	const GEN_FLT x97 = (x96 * x35) + (x94 * x39);
	const GEN_FLT x98 = 2 * x88;
	const GEN_FLT x99 =
		x62 *
		((-1 * x85 *
		  ((x84 * x92) + (-1 * x55 *
						  ((x94 * x40) +
						   (x35 * (x97 + (x94 * x44) +
								   (x35 * (x96 + (x35 * ((x94 * x42) + x95 + (-1 * x82 * x94))) + (x94 * x43))))) +
						   (x97 * x35) + (x94 * x45))))) +
		 (x98 * x94) + (x60 * x97) + (-1 * x71 * x92) + x50);
	const GEN_FLT x100 = x25 * x22;
	const GEN_FLT x101 = -1 * x100;
	const GEN_FLT x102 = x63 * x17;
	const GEN_FLT x103 = x49 * x102;
	const GEN_FLT x104 = x69 * ((x67 * x103) + x101);
	const GEN_FLT x105 = x73 * x17;
	const GEN_FLT x106 = x77 * x105;
	const GEN_FLT x107 = -1 * x37 * x106;
	const GEN_FLT x108 = (-1 * x38 * x106) + (x35 * (x107 + (x36 * x106)));
	const GEN_FLT x109 = (x35 * x108) + (-1 * x39 * x106);
	const GEN_FLT x110 = 2 * x17;
	const GEN_FLT x111 = x73 * x110;
	const GEN_FLT x112 =
		x62 * ((-1 * x89 * x111) +
			   (-1 * x85 *
				((-1 * x83 * x104) +
				 (-1 * x55 *
				  ((-1 * x40 * x106) + (x35 * x109) +
				   (x35 * (x109 + (-1 * x44 * x106) +
						   (x35 * ((x35 * ((-1 * x42 * x106) + (x82 * x106) + x107)) + x108 + (-1 * x43 * x106))))) +
				   (-1 * x45 * x106))))) +
			   (x60 * x109) + (x70 * x104) + (-1 * x103));
	const GEN_FLT x113 = pow(x22, -1);
	const GEN_FLT x114 = 2 * x19;
	const GEN_FLT x115 = (2 * x20) + (-1 * x114);
	const GEN_FLT x116 = 2 * x8;
	const GEN_FLT x117 = (2 * x11) + (-1 * x116);
	const GEN_FLT x118 = pow(x23, -1) * x17;
	const GEN_FLT x119 = x25 * x23;
	const GEN_FLT x120 = ((x118 * x117) + (-1 * x113 * x115)) * x119;
	const GEN_FLT x121 = 2 * x14;
	const GEN_FLT x122 = (2 * x15) + (-1 * x121);
	const GEN_FLT x123 = 2 * x22;
	const GEN_FLT x124 = (x110 * x115) + (x117 * x123);
	const GEN_FLT x125 = 1.0 / 2.0 * x63;
	const GEN_FLT x126 = x49 * x125;
	const GEN_FLT x127 = (-1 * x124 * x126) + (x50 * x122);
	const GEN_FLT x128 = (-1 * x67 * x127) + x120;
	const GEN_FLT x129 = x124 + (x86 * x122);
	const GEN_FLT x130 = 1.0 / 2.0 * x73;
	const GEN_FLT x131 = x30 * x130;
	const GEN_FLT x132 = x76 * ((x34 * x122) + (-1 * x129 * x131));
	const GEN_FLT x133 = x37 * x132;
	const GEN_FLT x134 = (x38 * x132) + (x35 * (x133 + (-1 * x36 * x132)));
	const GEN_FLT x135 = (x35 * x134) + (x39 * x132);
	const GEN_FLT x136 =
		x62 * (x127 + (x60 * x135) +
			   (-1 * x85 *
				((-1 * x84 * x128) +
				 (-1 * x55 *
				  ((x40 * x132) + (x35 * x135) +
				   (x35 * (x135 + (x44 * x132) +
						   (x35 * (x134 + (x35 * ((-1 * x82 * x132) + (x42 * x132) + x133)) + (x43 * x132))))) +
				   (x45 * x132))))) +
			   (x98 * x132) + (x71 * x128));
	const GEN_FLT x137 = (-1 * sensor_z) + (-1 * x9) + (-1 * obj_pz);
	const GEN_FLT x138 = 2 * lh_qi;
	const GEN_FLT x139 = 2 * x13;
	const GEN_FLT x140 = x122 + x139 + (x137 * x138);
	const GEN_FLT x141 = 2 * lh_qk;
	const GEN_FLT x142 = 2 * lh_qj;
	const GEN_FLT x143 = (x7 * x142) + (-1 * x137 * x141);
	const GEN_FLT x144 = ((x118 * x143) + (-1 * x113 * x140)) * x119;
	const GEN_FLT x145 = 2 * x18;
	const GEN_FLT x146 = (-1 * x145) + x114 + (-4 * x20);
	const GEN_FLT x147 = (x110 * x140) + (x123 * x143);
	const GEN_FLT x148 = (-1 * x126 * x147) + (x50 * x146);
	const GEN_FLT x149 = (-1 * x67 * x148) + x144;
	const GEN_FLT x150 = x147 + (x86 * x146);
	const GEN_FLT x151 = x76 * ((x34 * x146) + (-1 * x131 * x150));
	const GEN_FLT x152 = x37 * x151;
	const GEN_FLT x153 = (x38 * x151) + (x35 * (x152 + (-1 * x36 * x151)));
	const GEN_FLT x154 = (x35 * x153) + (x39 * x151);
	const GEN_FLT x155 =
		x62 * (x148 +
			   (-1 * x85 *
				((-1 * x84 * x149) +
				 (-1 * x55 *
				  ((x35 * x154) +
				   (x35 * (x154 + (x44 * x151) +
						   (x35 * (x153 + (x35 * ((x42 * x151) + (-1 * x82 * x151) + x152)) + (x43 * x151))))) +
				   (x40 * x151) + (x45 * x151))))) +
			   (x98 * x151) + (x60 * x154) + (x71 * x149));
	const GEN_FLT x156 = 2 * x4;
	const GEN_FLT x157 = (-1 * x156) + x116 + (-4 * x11);
	const GEN_FLT x158 = (-1 * sensor_x) + (-1 * x2) + (-1 * obj_px);
	const GEN_FLT x159 = x115 + x145 + (x142 * x158);
	const GEN_FLT x160 = ((x118 * x159) + (-1 * x113 * x157)) * x119;
	const GEN_FLT x161 = (x10 * x141) + (-1 * x138 * x158);
	const GEN_FLT x162 = (x110 * x157) + (x123 * x159);
	const GEN_FLT x163 = (-1 * x126 * x162) + (x50 * x161);
	const GEN_FLT x164 = (-1 * x67 * x163) + x160;
	const GEN_FLT x165 = x130 * (x162 + (x86 * x161));
	const GEN_FLT x166 = ((x34 * x161) + (-1 * x30 * x165)) * x76;
	const GEN_FLT x167 = x37 * x166;
	const GEN_FLT x168 = (x38 * x166) + (x35 * (x167 + (-1 * x36 * x166)));
	const GEN_FLT x169 = (x35 * x168) + (x39 * x166);
	const GEN_FLT x170 =
		x62 * (x163 +
			   (-1 * x85 *
				((-1 * x84 * x164) +
				 (-1 * x55 *
				  ((x40 * x166) +
				   (x35 * (x169 + (x44 * x166) +
						   (x35 * (x168 + (x35 * ((x42 * x166) + (-1 * x82 * x166) + x167)) + (x43 * x166))))) +
				   (x35 * x169) + (x45 * x166))))) +
			   (x98 * x166) + (x60 * x169) + (x71 * x164));
	const GEN_FLT x171 = (-1 * sensor_y) + (-1 * x6) + (-1 * obj_py);
	const GEN_FLT x172 = (x3 * x138) + (-1 * x171 * x142);
	const GEN_FLT x173 = (-1 * x139) + x121 + (-4 * x15);
	const GEN_FLT x174 = ((x118 * x173) + (-1 * x113 * x172)) * x119;
	const GEN_FLT x175 = x117 + x156 + (x171 * x141);
	const GEN_FLT x176 = (x110 * x172) + (x123 * x173);
	const GEN_FLT x177 = (-1 * x126 * x176) + (x50 * x175);
	const GEN_FLT x178 = (-1 * x67 * x177) + x174;
	const GEN_FLT x179 = x176 + (x86 * x175);
	const GEN_FLT x180 = x76 * ((x34 * x175) + (-1 * x179 * x131));
	const GEN_FLT x181 = x37 * x180;
	const GEN_FLT x182 = (x38 * x180) + (x35 * (x181 + (-1 * x36 * x180)));
	const GEN_FLT x183 = (x35 * x182) + (x39 * x180);
	const GEN_FLT x184 =
		x62 * ((x98 * x180) + x177 +
			   (-1 * x85 *
				((-1 * x84 * x178) +
				 (-1 * x55 *
				  ((x40 * x180) + (x45 * x180) + (x35 * x183) +
				   (x35 * (x183 + (x44 * x180) +
						   (x35 * (x182 + (x35 * ((x42 * x180) + (-1 * x82 * x180) + x181)) + (x43 * x180))))))))) +
			   (x60 * x183) + (x71 * x178));
	const GEN_FLT x185 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x186 = tan(x185);
	const GEN_FLT x187 = x64 * x186;
	const GEN_FLT x188 = cos(x185);
	const GEN_FLT x189 = pow(x188, -1);
	const GEN_FLT x190 = x33 * x189;
	const GEN_FLT x191 = asin(x27 * x190);
	const GEN_FLT x192 = 8.0108022e-06 * x191;
	const GEN_FLT x193 = pow((1 + (-1 * x75 * pow(x188, -2))), -1.0 / 2.0);
	const GEN_FLT x194 = x189 * x193;
	const GEN_FLT x195 = x74 * x194;
	const GEN_FLT x196 = -8.0108022e-06 + (-1 * x192);
	const GEN_FLT x197 = -1 * x196 * x195;
	const GEN_FLT x198 = 0.0028679863 + (x191 * x196);
	const GEN_FLT x199 = (-1 * x198 * x195) + (x191 * (x197 + (x192 * x195)));
	const GEN_FLT x200 = 5.3685255e-06 + (x191 * x198);
	const GEN_FLT x201 = (-1 * x200 * x195) + (x191 * x199);
	const GEN_FLT x202 = 0.0076069798 + (x200 * x191);
	const GEN_FLT x203 = x202 * x191;
	const GEN_FLT x204 = -8.0108022e-06 + (-1.60216044e-05 * x191);
	const GEN_FLT x205 = x198 + (x204 * x191);
	const GEN_FLT x206 = x200 + (x205 * x191);
	const GEN_FLT x207 = x202 + (x206 * x191);
	const GEN_FLT x208 = (x207 * x191) + x203;
	const GEN_FLT x209 = sin(x185);
	const GEN_FLT x210 = x48 * x186;
	const GEN_FLT x211 = -1 * x27 * x210;
	const GEN_FLT x212 = x52 + (-1 * asin(x211)) + ogeeMag_1;
	const GEN_FLT x213 = (sin(x212) * ogeePhase_1) + curve_1;
	const GEN_FLT x214 = x213 * x209;
	const GEN_FLT x215 = x188 + (x214 * x208);
	const GEN_FLT x216 = pow(x215, -1);
	const GEN_FLT x217 = pow(x191, 2);
	const GEN_FLT x218 = x213 * x217;
	const GEN_FLT x219 = x218 * x216;
	const GEN_FLT x220 = pow((1 + (-1 * x66 * pow(x186, 2))), -1.0 / 2.0);
	const GEN_FLT x221 = (-1 * x220 * x187) + x26;
	const GEN_FLT x222 = cos(x212) * ogeePhase_1;
	const GEN_FLT x223 = x217 * x216 * x202;
	const GEN_FLT x224 = x223 * x222;
	const GEN_FLT x225 = x213 * x216 * x203;
	const GEN_FLT x226 = x225 * x194;
	const GEN_FLT x227 = 2.40324066e-05 * x191;
	const GEN_FLT x228 = x208 * x209;
	const GEN_FLT x229 = x222 * x228;
	const GEN_FLT x230 = x218 * pow(x215, -2) * x202;
	const GEN_FLT x231 = x211 + (x219 * x202);
	const GEN_FLT x232 = pow((1 + (-1 * pow(x231, 2))), -1.0 / 2.0);
	const GEN_FLT x233 =
		(-1 * x232 *
		 ((-1 * x230 *
		   ((x221 * x229) + (x214 * ((-1 * x202 * x195) + (x201 * x191) + (-1 * x207 * x195) +
									 (x191 * (x201 +
											  (x191 * (x199 + (-1 * x205 * x195) +
													   (x191 * ((-1 * x204 * x195) + (x227 * x195) + x197)))) +
											  (-1 * x206 * x195))))))) +
		  (-1 * x87 * x226) + (x224 * x221) + (x219 * x201) + x187)) +
		x26;
	const GEN_FLT x234 = cos(x52 + (-1 * asin(x231)) + gibPhase_1) * gibMag_1;
	const GEN_FLT x235 = x193 * (x190 + (-1 * x93 * x189));
	const GEN_FLT x236 = x235 * x196;
	const GEN_FLT x237 = (x235 * x198) + (x191 * (x236 + (-1 * x235 * x192)));
	const GEN_FLT x238 = (x235 * x200) + (x237 * x191);
	const GEN_FLT x239 = 2 * x225;
	const GEN_FLT x240 = x210 * x220;
	const GEN_FLT x241 =
		x232 *
		((-1 * x230 *
		  ((x229 * x240) +
		   (x214 *
			((x235 * x202) + (x238 * x191) + (x235 * x207) +
			 (x191 * (x238 + (x191 * (x237 + (x235 * x205) + (x191 * ((-1 * x235 * x227) + (x235 * x204) + x236)))) +
					  (x235 * x206))))))) +
		 (-1 * x210) + (x224 * x240) + (x235 * x239) + (x219 * x238));
	const GEN_FLT x242 = x102 * x186;
	const GEN_FLT x243 = x105 * x194;
	const GEN_FLT x244 = -1 * x243 * x196;
	const GEN_FLT x245 = (-1 * x243 * x198) + (x191 * (x244 + (x243 * x192)));
	const GEN_FLT x246 = (-1 * x200 * x243) + (x245 * x191);
	const GEN_FLT x247 = (-1 * x220 * x242) + x101;
	const GEN_FLT x248 =
		(-1 * x232 *
		 ((-1 * x230 *
		   ((x229 * x247) + (x214 * ((x246 * x191) + (-1 * x202 * x243) + (-1 * x207 * x243) +
									 (x191 * (x246 +
											  (x191 * (x245 + (-1 * x205 * x243) +
													   (x191 * ((x227 * x243) + (-1 * x204 * x243) + x244)))) +
											  (-1 * x206 * x243))))))) +
		  (x224 * x247) + (x219 * x246) + (-1 * x226 * x111) + x242)) +
		x101;
	const GEN_FLT x249 = x189 * x130;
	const GEN_FLT x250 = ((x122 * x190) + (-1 * x249 * x129)) * x193;
	const GEN_FLT x251 = x250 * x196;
	const GEN_FLT x252 = (x250 * x198) + (x191 * (x251 + (-1 * x250 * x192)));
	const GEN_FLT x253 = (x200 * x250) + (x252 * x191);
	const GEN_FLT x254 = x125 * x186;
	const GEN_FLT x255 = (x254 * x124) + (-1 * x210 * x122);
	const GEN_FLT x256 = (-1 * x255 * x220) + x120;
	const GEN_FLT x257 =
		(-1 * x232 *
		 (x255 +
		  (-1 * x230 *
		   ((x256 * x229) +
			(x214 *
			 ((x202 * x250) + (x253 * x191) + (x207 * x250) +
			  (x191 * (x253 + (x191 * ((x205 * x250) + x252 + (x191 * ((x204 * x250) + (-1 * x250 * x227) + x251)))) +
					   (x206 * x250))))))) +
		  (x239 * x250) + (x256 * x224) + (x219 * x253))) +
		x120;
	const GEN_FLT x258 = ((x190 * x146) + (-1 * x249 * x150)) * x193;
	const GEN_FLT x259 = x258 * x196;
	const GEN_FLT x260 = (x258 * x198) + (x191 * (x259 + (-1 * x258 * x192)));
	const GEN_FLT x261 = (x200 * x258) + (x260 * x191);
	const GEN_FLT x262 = (x254 * x147) + (-1 * x210 * x146);
	const GEN_FLT x263 = x222 * ((-1 * x262 * x220) + x144);
	const GEN_FLT x264 =
		(-1 * x232 *
		 (x262 + (x263 * x223) + (x239 * x258) +
		  (-1 * x230 *
		   ((x263 * x228) +
			(x214 *
			 ((x202 * x258) + (x261 * x191) + (x207 * x258) +
			  (x191 * (x261 + (x191 * (x260 + (x205 * x258) + (x191 * ((x204 * x258) + (-1 * x258 * x227) + x259)))) +
					   (x206 * x258))))))) +
		  (x219 * x261))) +
		x144;
	const GEN_FLT x265 = ((x161 * x190) + (-1 * x165 * x189)) * x193;
	const GEN_FLT x266 = x265 * x196;
	const GEN_FLT x267 = (x265 * x198) + (x191 * (x266 + (-1 * x265 * x192)));
	const GEN_FLT x268 = (x200 * x265) + (x267 * x191);
	const GEN_FLT x269 = (x254 * x162) + (-1 * x210 * x161);
	const GEN_FLT x270 = (-1 * x269 * x220) + x160;
	const GEN_FLT x271 =
		(-1 * x232 *
		 (x269 +
		  (-1 * x230 *
		   ((x270 * x229) +
			(x214 *
			 ((x202 * x265) + (x268 * x191) + (x207 * x265) +
			  (x191 * (x268 + (x191 * (x267 + (x205 * x265) + (x191 * ((x204 * x265) + (-1 * x265 * x227) + x266)))) +
					   (x206 * x265))))))) +
		  (x239 * x265) + (x270 * x224) + (x219 * x268))) +
		x160;
	const GEN_FLT x272 = ((x175 * x190) + (-1 * x249 * x179)) * x193;
	const GEN_FLT x273 = x272 * x196;
	const GEN_FLT x274 = (x272 * x198) + (x191 * (x273 + (-1 * x272 * x192)));
	const GEN_FLT x275 = (x200 * x272) + (x274 * x191);
	const GEN_FLT x276 = (x254 * x176) + (-1 * x210 * x175);
	const GEN_FLT x277 = (-1 * x276 * x220) + x174;
	const GEN_FLT x278 =
		(-1 * x232 *
		 (x276 + (x239 * x272) + (x277 * x224) +
		  (-1 * x230 *
		   ((x277 * x229) +
			(x214 *
			 ((x202 * x272) + (x275 * x191) + (x207 * x272) +
			  (x191 * (x275 + (x191 * (x274 + (x205 * x272) + (x191 * ((x204 * x272) + (-1 * x272 * x227) + x273)))) +
					   (x206 * x272))))))) +
		  (x219 * x275))) +
		x174;
	out[0] = (-1 * (x90 + (-1 * x26)) * x91) + (-1 * x90) + x26;
	out[1] = (-1 * x91 * x99) + (-1 * x99);
	out[2] = (-1 * (x112 + x100) * x91) + (-1 * x112) + x101;
	out[3] = (-1 * (x136 + (-1 * x120)) * x91) + (-1 * x136) + x120;
	out[4] = (-1 * x155) + (-1 * (x155 + (-1 * x144)) * x91) + x144;
	out[5] = (-1 * (x170 + (-1 * x160)) * x91) + (-1 * x170) + x160;
	out[6] = (-1 * (x184 + (-1 * x174)) * x91) + x174 + (-1 * x184);
	out[7] = x233 + (x234 * x233);
	out[8] = (-1 * x241) + (-1 * x234 * x241);
	out[9] = x248 + (x234 * x248);
	out[10] = x257 + (x234 * x257);
	out[11] = x264 + (x234 * x264);
	out[12] = x271 + (x234 * x271);
	out[13] = x278 + (x234 * x278);
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
	const GEN_FLT x2 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x3 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x4 = sensor_z + (2 * ((x3 * obj_qi) + (-1 * x2 * obj_qj))) + obj_pz;
	const GEN_FLT x5 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x6 = sensor_x + (2 * ((x5 * obj_qj) + (-1 * x3 * obj_qk))) + obj_px;
	const GEN_FLT x7 = sensor_y + (2 * ((x2 * obj_qk) + (-1 * x5 * obj_qi))) + obj_py;
	const GEN_FLT x8 = (x7 * lh_qi) + (-1 * x6 * lh_qj) + (x4 * lh_qw);
	const GEN_FLT x9 = (x4 * lh_qj) + (-1 * x7 * lh_qk) + (x6 * lh_qw);
	const GEN_FLT x10 = x7 + (2 * ((x9 * lh_qk) + (-1 * x8 * lh_qi))) + lh_py;
	const GEN_FLT x11 = (x6 * lh_qk) + (-1 * x4 * lh_qi) + (x7 * lh_qw);
	const GEN_FLT x12 = x4 + (2 * ((x11 * lh_qi) + (-1 * x9 * lh_qj))) + lh_pz;
	const GEN_FLT x13 = x6 + (2 * ((x8 * lh_qj) + (-1 * x11 * lh_qk))) + lh_px;
	const GEN_FLT x14 = pow(x13, 2) + pow(x12, 2);
	const GEN_FLT x15 = pow(x14, -1.0 / 2.0) * x10;
	const GEN_FLT x16 = x1 * x15;
	const GEN_FLT x17 = atan2(-1 * x12, x13);
	const GEN_FLT x18 = x17 + (-1 * asin(x16)) + ogeeMag_0;
	const GEN_FLT x19 = sin(x18);
	const GEN_FLT x20 = (x19 * ogeePhase_0) + curve_0;
	const GEN_FLT x21 = cos(x0);
	const GEN_FLT x22 = pow(x10, 2);
	const GEN_FLT x23 = x14 + x22;
	const GEN_FLT x24 = pow(x23, -1.0 / 2.0) * x10;
	const GEN_FLT x25 = asin(x24 * pow(x21, -1));
	const GEN_FLT x26 = 8.0108022e-06 * x25;
	const GEN_FLT x27 = -8.0108022e-06 + (-1 * x26);
	const GEN_FLT x28 = 0.0028679863 + (x25 * x27);
	const GEN_FLT x29 = 5.3685255e-06 + (x25 * x28);
	const GEN_FLT x30 = 0.0076069798 + (x25 * x29);
	const GEN_FLT x31 = x30 * x25;
	const GEN_FLT x32 = -8.0108022e-06 + (-1.60216044e-05 * x25);
	const GEN_FLT x33 = x28 + (x32 * x25);
	const GEN_FLT x34 = x29 + (x33 * x25);
	const GEN_FLT x35 = x30 + (x34 * x25);
	const GEN_FLT x36 = (x35 * x25) + x31;
	const GEN_FLT x37 = sin(x0);
	const GEN_FLT x38 = x37 * x20;
	const GEN_FLT x39 = x36 * x38;
	const GEN_FLT x40 = x21 + (-1 * x39);
	const GEN_FLT x41 = pow(x40, -1);
	const GEN_FLT x42 = pow(x25, 2);
	const GEN_FLT x43 = x41 * x42;
	const GEN_FLT x44 = x43 * x30;
	const GEN_FLT x45 = x16 + (x44 * x20);
	const GEN_FLT x46 = pow((1 + (-1 * pow(x45, 2))), -1.0 / 2.0);
	const GEN_FLT x47 = pow(x1, 2);
	const GEN_FLT x48 = x15 * (1 + x47);
	const GEN_FLT x49 = cos(x18) * ogeePhase_0;
	const GEN_FLT x50 = x44 * x49;
	const GEN_FLT x51 = x22 * pow(x14, -1);
	const GEN_FLT x52 = x48 * pow((1 + (-1 * x51 * x47)), -1.0 / 2.0);
	const GEN_FLT x53 = pow(x21, -2);
	const GEN_FLT x54 = x22 * pow(x23, -1);
	const GEN_FLT x55 = x53 * x24 * pow((1 + (-1 * x54 * x53)), -1.0 / 2.0);
	const GEN_FLT x56 = x55 * x37;
	const GEN_FLT x57 = x56 * x27;
	const GEN_FLT x58 = (x56 * x28) + (x25 * (x57 + (-1 * x56 * x26)));
	const GEN_FLT x59 = (x58 * x25) + (x56 * x29);
	const GEN_FLT x60 = pow(x40, -2) * x42 * x30;
	const GEN_FLT x61 =
		x46 * ((-1 * x60 * x20 *
				((x52 * x49 * x36 * x37) +
				 (-1 * x38 *
				  ((x56 * x30) +
				   (x25 * ((x56 * x34) + x59 +
						   (x25 * (x58 + (x25 * ((x56 * x32) + (-2.40324066e-05 * x56 * x25) + x57)) + (x56 * x33))))) +
				   (x59 * x25) + (x56 * x35))) +
				 (-1 * x36 * x20 * x21) + (-1 * x37))) +
			   (x59 * x43 * x20) + (2 * x55 * x41 * x31 * x38) + (-1 * x50 * x52) + x48);
	const GEN_FLT x62 = (-1 * x17) + asin(x45) + (-1 * gibPhase_0);
	const GEN_FLT x63 = cos(x62) * gibMag_0;
	const GEN_FLT x64 = x60 * x39;
	const GEN_FLT x65 = (x64 + x44) * x46;
	const GEN_FLT x66 = x46 * ((x64 * x49) + x50);
	const GEN_FLT x67 = ((x64 * x19) + (x44 * x19)) * x46;
	const GEN_FLT x68 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x69 = tan(x68);
	const GEN_FLT x70 = pow(x69, 2);
	const GEN_FLT x71 = x15 * (1 + x70);
	const GEN_FLT x72 = cos(x68);
	const GEN_FLT x73 = asin(pow(x72, -1) * x24);
	const GEN_FLT x74 = 8.0108022e-06 * x73;
	const GEN_FLT x75 = sin(x68);
	const GEN_FLT x76 = pow(x72, -2);
	const GEN_FLT x77 = x76 * x24 * pow((1 + (-1 * x76 * x54)), -1.0 / 2.0);
	const GEN_FLT x78 = x75 * x77;
	const GEN_FLT x79 = -8.0108022e-06 + (-1 * x74);
	const GEN_FLT x80 = -1 * x79 * x78;
	const GEN_FLT x81 = 0.0028679863 + (x73 * x79);
	const GEN_FLT x82 = (-1 * x81 * x78) + (x73 * (x80 + (x78 * x74)));
	const GEN_FLT x83 = 5.3685255e-06 + (x81 * x73);
	const GEN_FLT x84 = (-1 * x83 * x78) + (x82 * x73);
	const GEN_FLT x85 = -1 * x69 * x15;
	const GEN_FLT x86 = x17 + (-1 * asin(x85)) + ogeeMag_1;
	const GEN_FLT x87 = sin(x86);
	const GEN_FLT x88 = (x87 * ogeePhase_1) + curve_1;
	const GEN_FLT x89 = pow(x73, 2);
	const GEN_FLT x90 = 0.0076069798 + (x83 * x73);
	const GEN_FLT x91 = x73 * x90;
	const GEN_FLT x92 = -8.0108022e-06 + (-1.60216044e-05 * x73);
	const GEN_FLT x93 = x81 + (x73 * x92);
	const GEN_FLT x94 = x83 + (x73 * x93);
	const GEN_FLT x95 = x90 + (x73 * x94);
	const GEN_FLT x96 = (x73 * x95) + x91;
	const GEN_FLT x97 = x88 * x96;
	const GEN_FLT x98 = x75 * x97;
	const GEN_FLT x99 = x72 + x98;
	const GEN_FLT x100 = pow(x99, -1);
	const GEN_FLT x101 = x89 * x100;
	const GEN_FLT x102 = x90 * x101;
	const GEN_FLT x103 = cos(x86) * ogeePhase_1;
	const GEN_FLT x104 = x103 * x102;
	const GEN_FLT x105 = x71 * pow((1 + (-1 * x70 * x51)), -1.0 / 2.0);
	const GEN_FLT x106 = x88 * x75;
	const GEN_FLT x107 = x89 * x90 * pow(x99, -2);
	const GEN_FLT x108 = x85 + (x88 * x102);
	const GEN_FLT x109 = pow((1 + (-1 * pow(x108, 2))), -1.0 / 2.0);
	const GEN_FLT x110 =
		x109 * ((-1 * x88 * x107 *
				 ((-1 * x72 * x97) +
				  (x106 * ((x84 * x73) + (-1 * x78 * x90) + (-1 * x78 * x95) +
						   (x73 * (x84 +
								   (x73 * ((-1 * x78 * x93) + x82 +
										   (x73 * ((-1 * x78 * x92) + (2.40324066e-05 * x73 * x78) + x80)))) +
								   (-1 * x78 * x94))))) +
				  (-1 * x75 * x96 * x103 * x105) + x75)) +
				(-2 * x77 * x91 * x100 * x106) + (-1 * x105 * x104) + (x88 * x84 * x101) + x71);
	const GEN_FLT x111 = x17 + (-1 * asin(x108)) + gibPhase_1;
	const GEN_FLT x112 = cos(x111) * gibMag_1;
	const GEN_FLT x113 = x98 * x107;
	const GEN_FLT x114 = ((-1 * x113) + x102) * x109;
	const GEN_FLT x115 = x109 * ((-1 * x103 * x113) + x104);
	const GEN_FLT x116 = ((-1 * x87 * x113) + (x87 * x102)) * x109;
	out[0] = -1;
	out[1] = (-1 * x63 * x61) + (-1 * x61);
	out[2] = (-1 * x63 * x65) + (-1 * x65);
	out[3] = x63;
	out[4] = -1 * sin(x62);
	out[5] = (-1 * x63 * x66) + (-1 * x66);
	out[6] = (-1 * x63 * x67) + (-1 * x67);
	out[7] = 0;
	out[8] = 0;
	out[9] = 0;
	out[10] = 0;
	out[11] = 0;
	out[12] = 0;
	out[13] = 0;
	out[14] = 0;
	out[15] = 0;
	out[16] = 0;
	out[17] = 0;
	out[18] = 0;
	out[19] = 0;
	out[20] = 0;
	out[21] = -1;
	out[22] = (-1 * x110) + (-1 * x110 * x112);
	out[23] = (-1 * x114) + (-1 * x112 * x114);
	out[24] = x112;
	out[25] = sin(x111);
	out[26] = (-1 * x115) + (-1 * x112 * x115);
	out[27] = (-1 * x116) + (-1 * x112 * x116);
}

static inline FLT gen_reproject_axis_x_gen2(const SurvivePose *obj_p, const FLT *sensor_pt, const SurvivePose *lh_p,
											const BaseStationCal *bsc0) {
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
	const GEN_FLT x0 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x1 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x2 = sensor_x + (2 * ((x1 * obj_qj) + (-1 * x0 * obj_qk))) + obj_px;
	const GEN_FLT x3 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x4 = sensor_y + (2 * ((x3 * obj_qk) + (-1 * x1 * obj_qi))) + obj_py;
	const GEN_FLT x5 = sensor_z + (2 * ((x0 * obj_qi) + (-1 * x3 * obj_qj))) + obj_pz;
	const GEN_FLT x6 = (x5 * lh_qj) + (-1 * x4 * lh_qk) + (x2 * lh_qw);
	const GEN_FLT x7 = (x2 * lh_qk) + (-1 * x5 * lh_qi) + (x4 * lh_qw);
	const GEN_FLT x8 = x5 + (2 * ((x7 * lh_qi) + (-1 * x6 * lh_qj))) + lh_pz;
	const GEN_FLT x9 = (x4 * lh_qi) + (-1 * x2 * lh_qj) + (x5 * lh_qw);
	const GEN_FLT x10 = x2 + (2 * ((x9 * lh_qj) + (-1 * x7 * lh_qk))) + lh_px;
	const GEN_FLT x11 = pow(x10, 2) + pow(x8, 2);
	const GEN_FLT x12 = 0.523598775598299 + tilt_0;
	const GEN_FLT x13 = (2 * ((x6 * lh_qk) + (-1 * x9 * lh_qi))) + x4 + lh_py;
	const GEN_FLT x14 = x13 * pow(x11, -1.0 / 2.0) * tan(x12);
	const GEN_FLT x15 = atan2(-1 * x8, x10);
	const GEN_FLT x16 = (sin(x15 + (-1 * asin(x14)) + ogeeMag_0) * ogeePhase_0) + curve_0;
	const GEN_FLT x17 = cos(x12);
	const GEN_FLT x18 = asin(x13 * pow(x17, -1) * pow((x11 + pow(x13, 2)), -1.0 / 2.0));
	const GEN_FLT x19 = 0.0028679863 + (x18 * (-8.0108022e-06 + (-8.0108022e-06 * x18)));
	const GEN_FLT x20 = 5.3685255e-06 + (x19 * x18);
	const GEN_FLT x21 = 0.0076069798 + (x20 * x18);
	const GEN_FLT x22 = asin(
		x14 +
		(x21 * pow(x18, 2) * x16 *
		 pow((x17 +
			  (-1 * x16 * sin(x12) *
			   ((x18 * (x21 + (x18 * (x20 + (x18 * (x19 + (x18 * (-8.0108022e-06 + (-1.60216044e-05 * x18))))))))) +
				(x21 * x18)))),
			 -1)));
	return -1.5707963267949 + x15 + (-1 * x22) + (-1 * sin((-1 * x15) + x22 + (-1 * gibPhase_0)) * gibMag_0) +
		   (-1 * phase_0);
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
	const GEN_FLT x0 = 2 * lh_qw;
	const GEN_FLT x1 = x0 * lh_qj;
	const GEN_FLT x2 = 2 * lh_qk;
	const GEN_FLT x3 = x2 * lh_qi;
	const GEN_FLT x4 = x3 + (-1 * x1);
	const GEN_FLT x5 = obj_qj * sensor_x;
	const GEN_FLT x6 = obj_qw * sensor_z;
	const GEN_FLT x7 = obj_qi * sensor_y;
	const GEN_FLT x8 = x7 + x6 + (-1 * x5);
	const GEN_FLT x9 = obj_qw * sensor_x;
	const GEN_FLT x10 = obj_qk * sensor_y;
	const GEN_FLT x11 = obj_qj * sensor_z;
	const GEN_FLT x12 = x11 + (-1 * x10) + x9;
	const GEN_FLT x13 = sensor_y + (2 * ((x12 * obj_qk) + (-1 * x8 * obj_qi))) + obj_py;
	const GEN_FLT x14 = obj_qw * sensor_y;
	const GEN_FLT x15 = obj_qi * sensor_z;
	const GEN_FLT x16 = obj_qk * sensor_x;
	const GEN_FLT x17 = x16 + (-1 * x15) + x14;
	const GEN_FLT x18 = sensor_z + (2 * ((x17 * obj_qi) + (-1 * x12 * obj_qj))) + obj_pz;
	const GEN_FLT x19 = sensor_x + (2 * ((x8 * obj_qj) + (-1 * x17 * obj_qk))) + obj_px;
	const GEN_FLT x20 = (x19 * lh_qk) + (-1 * x18 * lh_qi) + (x13 * lh_qw);
	const GEN_FLT x21 = (x13 * lh_qi) + (-1 * x19 * lh_qj) + (x18 * lh_qw);
	const GEN_FLT x22 = x19 + (2 * ((x21 * lh_qj) + (-1 * x20 * lh_qk))) + lh_px;
	const GEN_FLT x23 = pow(x22, -1);
	const GEN_FLT x24 = -2 * pow(lh_qk, 2);
	const GEN_FLT x25 = -2 * pow(lh_qj, 2);
	const GEN_FLT x26 = 1 + x25 + x24;
	const GEN_FLT x27 = (x18 * lh_qj) + (-1 * x13 * lh_qk) + (x19 * lh_qw);
	const GEN_FLT x28 = x18 + (2 * ((x20 * lh_qi) + (-1 * x27 * lh_qj))) + lh_pz;
	const GEN_FLT x29 = pow(x22, 2);
	const GEN_FLT x30 = x28 * pow(x29, -1);
	const GEN_FLT x31 = x29 + pow(x28, 2);
	const GEN_FLT x32 = pow(x31, -1);
	const GEN_FLT x33 = x32 * x29;
	const GEN_FLT x34 = x33 * ((x30 * x26) + (-1 * x4 * x23));
	const GEN_FLT x35 = x13 + (2 * ((x27 * lh_qk) + (-1 * x21 * lh_qi))) + lh_py;
	const GEN_FLT x36 = 0.523598775598299 + tilt_0;
	const GEN_FLT x37 = cos(x36);
	const GEN_FLT x38 = pow(x37, -1);
	const GEN_FLT x39 = pow(x35, 2);
	const GEN_FLT x40 = x31 + x39;
	const GEN_FLT x41 = pow(x40, -1.0 / 2.0) * x38;
	const GEN_FLT x42 = asin(x41 * x35);
	const GEN_FLT x43 = 8.0108022e-06 * x42;
	const GEN_FLT x44 = -8.0108022e-06 + (-1 * x43);
	const GEN_FLT x45 = 0.0028679863 + (x42 * x44);
	const GEN_FLT x46 = 5.3685255e-06 + (x42 * x45);
	const GEN_FLT x47 = 0.0076069798 + (x42 * x46);
	const GEN_FLT x48 = x42 * x47;
	const GEN_FLT x49 = -8.0108022e-06 + (-1.60216044e-05 * x42);
	const GEN_FLT x50 = x45 + (x42 * x49);
	const GEN_FLT x51 = x46 + (x50 * x42);
	const GEN_FLT x52 = x47 + (x51 * x42);
	const GEN_FLT x53 = (x52 * x42) + x48;
	const GEN_FLT x54 = sin(x36);
	const GEN_FLT x55 = tan(x36);
	const GEN_FLT x56 = x55 * pow(x31, -1.0 / 2.0);
	const GEN_FLT x57 = x56 * x35;
	const GEN_FLT x58 = atan2(-1 * x28, x22);
	const GEN_FLT x59 = x58 + (-1 * asin(x57)) + ogeeMag_0;
	const GEN_FLT x60 = (sin(x59) * ogeePhase_0) + curve_0;
	const GEN_FLT x61 = x60 * x54;
	const GEN_FLT x62 = x37 + (-1 * x61 * x53);
	const GEN_FLT x63 = pow(x62, -1);
	const GEN_FLT x64 = pow(x42, 2);
	const GEN_FLT x65 = x60 * x64;
	const GEN_FLT x66 = x63 * x65;
	const GEN_FLT x67 = x57 + (x66 * x47);
	const GEN_FLT x68 = pow((1 + (-1 * pow(x67, 2))), -1.0 / 2.0);
	const GEN_FLT x69 = pow((1 + (-1 * pow(x55, 2) * x32 * x39)), -1.0 / 2.0);
	const GEN_FLT x70 = 2 * lh_qi;
	const GEN_FLT x71 = x70 * lh_qj;
	const GEN_FLT x72 = x0 * lh_qk;
	const GEN_FLT x73 = x72 + x71;
	const GEN_FLT x74 = 2 * x22;
	const GEN_FLT x75 = 2 * x28;
	const GEN_FLT x76 = (x4 * x75) + (x74 * x26);
	const GEN_FLT x77 = 1.0 / 2.0 * x35;
	const GEN_FLT x78 = x77 * x55 * pow(x31, -3.0 / 2.0);
	const GEN_FLT x79 = (-1 * x78 * x76) + (x73 * x56);
	const GEN_FLT x80 = (-1 * x79 * x69) + x34;
	const GEN_FLT x81 = cos(x59) * ogeePhase_0;
	const GEN_FLT x82 = x63 * x64 * x47;
	const GEN_FLT x83 = x81 * x82;
	const GEN_FLT x84 = pow((1 + (-1 * pow(x40, -1) * pow(x37, -2) * x39)), -1.0 / 2.0);
	const GEN_FLT x85 = 2 * x35;
	const GEN_FLT x86 = x77 * pow(x40, -3.0 / 2.0) * x38;
	const GEN_FLT x87 = x84 * ((x73 * x41) + (-1 * x86 * (x76 + (x85 * x73))));
	const GEN_FLT x88 = x87 * x44;
	const GEN_FLT x89 = (x87 * x45) + (x42 * (x88 + (-1 * x87 * x43)));
	const GEN_FLT x90 = (x89 * x42) + (x87 * x46);
	const GEN_FLT x91 = 2.40324066e-05 * x42;
	const GEN_FLT x92 = x54 * x53;
	const GEN_FLT x93 = x81 * x92;
	const GEN_FLT x94 = pow(x62, -2) * x65 * x47;
	const GEN_FLT x95 = 2 * x60 * x63 * x48;
	const GEN_FLT x96 =
		x68 *
		(x79 +
		 (-1 * x94 *
		  ((-1 * x80 * x93) + (-1 * x61 *
							   ((x87 * x47) + (x90 * x42) +
								(x42 * (x90 + (x87 * x51) +
										(x42 * ((x42 * ((-1 * x87 * x91) + (x87 * x49) + x88)) + x89 + (x87 * x50))))) +
								(x87 * x52))))) +
		 (x66 * x90) + (x87 * x95) + (x80 * x83));
	const GEN_FLT x97 = cos(x58 + (-1 * asin(x67)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x98 = x2 * lh_qj;
	const GEN_FLT x99 = x0 * lh_qi;
	const GEN_FLT x100 = x99 + x98;
	const GEN_FLT x101 = x71 + (-1 * x72);
	const GEN_FLT x102 = ((x30 * x101) + (-1 * x23 * x100)) * x33;
	const GEN_FLT x103 = 1 + (-2 * pow(lh_qi, 2));
	const GEN_FLT x104 = x103 + x24;
	const GEN_FLT x105 = (x75 * x100) + (x74 * x101);
	const GEN_FLT x106 = (-1 * x78 * x105) + (x56 * x104);
	const GEN_FLT x107 = (-1 * x69 * x106) + x102;
	const GEN_FLT x108 = x84 * ((x41 * x104) + (-1 * x86 * (x105 + (x85 * x104))));
	const GEN_FLT x109 = x44 * x108;
	const GEN_FLT x110 = (x45 * x108) + (x42 * (x109 + (-1 * x43 * x108)));
	const GEN_FLT x111 = (x42 * x110) + (x46 * x108);
	const GEN_FLT x112 =
		x68 * (x106 +
			   (-1 * x94 *
				((-1 * x93 * x107) +
				 (-1 * x61 *
				  ((x47 * x108) + (x42 * x111) +
				   (x42 * (x111 + (x51 * x108) +
						   (x42 * (x110 + (x42 * ((x49 * x108) + (-1 * x91 * x108) + x109)) + (x50 * x108))))) +
				   (x52 * x108))))) +
			   (x95 * x108) + (x66 * x111) + (x83 * x107));
	const GEN_FLT x113 = x103 + x25;
	const GEN_FLT x114 = x1 + x3;
	const GEN_FLT x115 = ((x30 * x114) + (-1 * x23 * x113)) * x33;
	const GEN_FLT x116 = x98 + (-1 * x99);
	const GEN_FLT x117 = (x75 * x113) + (x74 * x114);
	const GEN_FLT x118 = (-1 * x78 * x117) + (x56 * x116);
	const GEN_FLT x119 = (-1 * x69 * x118) + x115;
	const GEN_FLT x120 = (x41 * x116) + (-1 * x86 * (x117 + (x85 * x116)));
	const GEN_FLT x121 = x84 * x120;
	const GEN_FLT x122 = x44 * x121;
	const GEN_FLT x123 = (x45 * x121) + (x42 * (x122 + (-1 * x43 * x121)));
	const GEN_FLT x124 = (x42 * x123) + (x46 * x121);
	const GEN_FLT x125 = x84 * x50;
	const GEN_FLT x126 =
		x68 * ((x95 * x121) + (x66 * x124) + x118 +
			   (-1 * x94 *
				((-1 * x93 * x119) +
				 (-1 * x61 *
				  ((x47 * x121) + (x42 * x124) +
				   (x42 * (x124 + (x51 * x121) +
						   (x42 * (x123 + (x42 * ((-1 * x91 * x121) + (x49 * x121) + x122)) + (x120 * x125))))) +
				   (x52 * x121))))) +
			   (x83 * x119));
	const GEN_FLT x127 = 2 * x5;
	const GEN_FLT x128 = 2 * x7;
	const GEN_FLT x129 = x128 + (-1 * x127);
	const GEN_FLT x130 = 2 * x15;
	const GEN_FLT x131 = 2 * x16;
	const GEN_FLT x132 = x131 + (-1 * x130);
	const GEN_FLT x133 = 2 * x10;
	const GEN_FLT x134 = 2 * x11;
	const GEN_FLT x135 = x134 + (-1 * x133);
	const GEN_FLT x136 = (x135 * lh_qw) + (-1 * x132 * lh_qk) + (x129 * lh_qj);
	const GEN_FLT x137 = 2 * lh_qj;
	const GEN_FLT x138 = (-1 * x129 * lh_qi) + (x132 * lh_qw) + (x135 * lh_qk);
	const GEN_FLT x139 = x129 + (x70 * x138) + (-1 * x137 * x136);
	const GEN_FLT x140 = (x129 * lh_qw) + (-1 * x135 * lh_qj) + (x132 * lh_qi);
	const GEN_FLT x141 = x135 + (x137 * x140) + (-1 * x2 * x138);
	const GEN_FLT x142 = ((x30 * x141) + (-1 * x23 * x139)) * x33;
	const GEN_FLT x143 = x132 + (x2 * x136) + (-1 * x70 * x140);
	const GEN_FLT x144 = (x75 * x139) + (x74 * x141);
	const GEN_FLT x145 = (-1 * x78 * x144) + (x56 * x143);
	const GEN_FLT x146 = (-1 * x69 * x145) + x142;
	const GEN_FLT x147 = (x41 * x143) + (-1 * x86 * (x144 + (x85 * x143)));
	const GEN_FLT x148 = x84 * x147;
	const GEN_FLT x149 = x44 * x148;
	const GEN_FLT x150 = (x45 * x148) + (x42 * (x149 + (-1 * x43 * x148)));
	const GEN_FLT x151 = (x42 * x150) + (x46 * x148);
	const GEN_FLT x152 =
		x68 * (x145 + (x95 * x148) +
			   (-1 * x94 *
				((-1 * x93 * x146) +
				 (-1 * x61 *
				  ((x47 * x148) + (x42 * x151) +
				   (x42 * (x151 + (x51 * x148) +
						   (x42 * (x150 + (x42 * ((x49 * x148) + (-1 * x91 * x148) + x149)) + (x125 * x147))))) +
				   (x52 * x148))))) +
			   (x66 * x151) + (x83 * x146));
	const GEN_FLT x153 = 2 * x14;
	const GEN_FLT x154 = x153 + (-4 * x15) + x131;
	const GEN_FLT x155 = 2 * x6;
	const GEN_FLT x156 = x127 + (-1 * x155) + (-4 * x7);
	const GEN_FLT x157 = 2 * obj_qk * sensor_z;
	const GEN_FLT x158 = 2 * obj_qj * sensor_y;
	const GEN_FLT x159 = x158 + x157;
	const GEN_FLT x160 = 2 * ((x159 * lh_qw) + (-1 * x156 * lh_qk) + (x154 * lh_qj));
	const GEN_FLT x161 = (x156 * lh_qw) + (-1 * x154 * lh_qi) + (x159 * lh_qk);
	const GEN_FLT x162 = x154 + (x70 * x161) + (-1 * x160 * lh_qj);
	const GEN_FLT x163 = (x154 * lh_qw) + (-1 * x159 * lh_qj) + (x156 * lh_qi);
	const GEN_FLT x164 = x159 + (x163 * x137) + (-1 * x2 * x161);
	const GEN_FLT x165 = ((x30 * x164) + (-1 * x23 * x162)) * x33;
	const GEN_FLT x166 = x156 + (x160 * lh_qk) + (-1 * x70 * x163);
	const GEN_FLT x167 = (x75 * x162) + (x74 * x164);
	const GEN_FLT x168 = (-1 * x78 * x167) + (x56 * x166);
	const GEN_FLT x169 = (-1 * x69 * x168) + x165;
	const GEN_FLT x170 = (x41 * x166) + (-1 * x86 * (x167 + (x85 * x166)));
	const GEN_FLT x171 = x84 * x170;
	const GEN_FLT x172 = x44 * x171;
	const GEN_FLT x173 = (x45 * x171) + (x42 * (x172 + (-1 * x43 * x171)));
	const GEN_FLT x174 = (x42 * x173) + (x46 * x171);
	const GEN_FLT x175 =
		x68 * (x168 + (x95 * x171) +
			   (-1 * x94 *
				((-1 * x93 * x169) +
				 (-1 * x61 *
				  ((x47 * x171) + (x42 * x174) +
				   (x42 * (x174 + (x51 * x171) +
						   (x42 * (x173 + (x42 * ((x49 * x171) + x172 + (-1 * x91 * x171))) + (x125 * x170))))) +
				   (x52 * x171))))) +
			   (x66 * x174) + (x83 * x169));
	const GEN_FLT x176 = 2 * x9;
	const GEN_FLT x177 = (-1 * x176) + x133 + (-4 * x11);
	const GEN_FLT x178 = 2 * obj_qi * sensor_x;
	const GEN_FLT x179 = x157 + x178;
	const GEN_FLT x180 = (-4 * x5) + x155 + x128;
	const GEN_FLT x181 = (-1 * x179 * lh_qk) + (x180 * lh_qw) + (x177 * lh_qj);
	const GEN_FLT x182 = (x179 * lh_qw) + (-1 * x177 * lh_qi) + (x180 * lh_qk);
	const GEN_FLT x183 = x177 + (x70 * x182) + (-1 * x181 * x137);
	const GEN_FLT x184 = (x177 * lh_qw) + (-1 * x180 * lh_qj) + (x179 * lh_qi);
	const GEN_FLT x185 = x180 + (x184 * x137) + (-1 * x2 * x182);
	const GEN_FLT x186 = ((x30 * x185) + (-1 * x23 * x183)) * x33;
	const GEN_FLT x187 = (x2 * x181) + x179 + (-1 * x70 * x184);
	const GEN_FLT x188 = (x75 * x183) + (x74 * x185);
	const GEN_FLT x189 = (-1 * x78 * x188) + (x56 * x187);
	const GEN_FLT x190 = (-1 * x69 * x189) + x186;
	const GEN_FLT x191 = (x41 * x187) + (-1 * x86 * (x188 + (x85 * x187)));
	const GEN_FLT x192 = x84 * x191;
	const GEN_FLT x193 = x44 * x192;
	const GEN_FLT x194 = (x45 * x192) + (x42 * (x193 + (-1 * x43 * x192)));
	const GEN_FLT x195 = (x42 * x194) + (x46 * x192);
	const GEN_FLT x196 =
		x68 * (x189 + (x95 * x192) + (x66 * x195) +
			   (-1 * x94 *
				((-1 * x93 * x190) +
				 (-1 * x61 *
				  ((x47 * x192) + (x42 * x195) +
				   (x42 * (x195 + (x51 * x192) +
						   (x42 * (x194 + (x42 * ((x49 * x192) + (-1 * x91 * x192) + x193)) + (x125 * x191))))) +
				   (x52 * x192))))) +
			   (x83 * x190));
	const GEN_FLT x197 = x178 + x158;
	const GEN_FLT x198 = x176 + (-4 * x10) + x134;
	const GEN_FLT x199 = (-1 * x153) + x130 + (-4 * x16);
	const GEN_FLT x200 = (-1 * x198 * lh_qk) + (x199 * lh_qw) + (x197 * lh_qj);
	const GEN_FLT x201 = (-1 * x197 * lh_qi) + (x198 * lh_qw) + (x199 * lh_qk);
	const GEN_FLT x202 = x197 + (x70 * x201) + (-1 * x200 * x137);
	const GEN_FLT x203 = (x197 * lh_qw) + (-1 * x199 * lh_qj) + (x198 * lh_qi);
	const GEN_FLT x204 = x199 + (x203 * x137) + (-1 * x2 * x201);
	const GEN_FLT x205 = ((x30 * x204) + (-1 * x23 * x202)) * x33;
	const GEN_FLT x206 = x198 + (x2 * x200) + (-1 * x70 * x203);
	const GEN_FLT x207 = (x75 * x202) + (x74 * x204);
	const GEN_FLT x208 = (-1 * x78 * x207) + (x56 * x206);
	const GEN_FLT x209 = x81 * ((-1 * x69 * x208) + x205);
	const GEN_FLT x210 = x84 * ((x41 * x206) + (-1 * x86 * (x207 + (x85 * x206))));
	const GEN_FLT x211 = x44 * x210;
	const GEN_FLT x212 = (x45 * x210) + (x42 * (x211 + (-1 * x43 * x210)));
	const GEN_FLT x213 = (x42 * x212) + (x46 * x210);
	const GEN_FLT x214 =
		x68 * (x208 +
			   (-1 * x94 *
				((-1 * x92 * x209) +
				 (-1 * x61 *
				  ((x47 * x210) + (x42 * x213) +
				   (x42 * (x213 + (x51 * x210) +
						   (x42 * (x212 + (x42 * ((x49 * x210) + (-1 * x91 * x210) + x211)) + (x50 * x210))))) +
				   (x52 * x210))))) +
			   (x66 * x213) + (x95 * x210) + (x82 * x209));
	out[0] = (-1 * (x96 + (-1 * x34)) * x97) + (-1 * x96) + x34;
	out[1] = x102 + (-1 * (x112 + (-1 * x102)) * x97) + (-1 * x112);
	out[2] = (-1 * (x126 + (-1 * x115)) * x97) + (-1 * x126) + x115;
	out[3] = (-1 * (x152 + (-1 * x142)) * x97) + (-1 * x152) + x142;
	out[4] = (-1 * (x175 + (-1 * x165)) * x97) + (-1 * x175) + x165;
	out[5] = (-1 * (x196 + (-1 * x186)) * x97) + x186 + (-1 * x196);
	out[6] = (-1 * (x214 + (-1 * x205)) * x97) + (-1 * x214) + x205;
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
	const GEN_FLT x0 = 2 * obj_qw;
	const GEN_FLT x1 = x0 * obj_qj;
	const GEN_FLT x2 = 2 * obj_qi;
	const GEN_FLT x3 = x2 * obj_qk;
	const GEN_FLT x4 = x3 + (-1 * x1);
	const GEN_FLT x5 = x2 * obj_qj;
	const GEN_FLT x6 = x0 * obj_qk;
	const GEN_FLT x7 = x6 + x5;
	const GEN_FLT x8 = -2 * pow(obj_qk, 2);
	const GEN_FLT x9 = 1 + (-2 * pow(obj_qj, 2));
	const GEN_FLT x10 = x9 + x8;
	const GEN_FLT x11 = 2 * ((x10 * lh_qw) + (-1 * x7 * lh_qk) + (x4 * lh_qj));
	const GEN_FLT x12 = 2 * ((x7 * lh_qw) + (-1 * x4 * lh_qi) + (x10 * lh_qk));
	const GEN_FLT x13 = x4 + (x12 * lh_qi) + (-1 * x11 * lh_qj);
	const GEN_FLT x14 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x15 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x16 = sensor_y + (2 * ((x15 * obj_qk) + (-1 * x14 * obj_qi))) + obj_py;
	const GEN_FLT x17 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x18 = sensor_z + (2 * ((x17 * obj_qi) + (-1 * x15 * obj_qj))) + obj_pz;
	const GEN_FLT x19 = sensor_x + (2 * ((x14 * obj_qj) + (-1 * x17 * obj_qk))) + obj_px;
	const GEN_FLT x20 = (x19 * lh_qk) + (-1 * x18 * lh_qi) + (x16 * lh_qw);
	const GEN_FLT x21 = (x16 * lh_qi) + (-1 * x19 * lh_qj) + (x18 * lh_qw);
	const GEN_FLT x22 = x19 + (2 * ((x21 * lh_qj) + (-1 * x20 * lh_qk))) + lh_px;
	const GEN_FLT x23 = pow(x22, -1);
	const GEN_FLT x24 = 2 * ((x4 * lh_qw) + (-1 * x10 * lh_qj) + (x7 * lh_qi));
	const GEN_FLT x25 = (x24 * lh_qj) + x10 + (-1 * x12 * lh_qk);
	const GEN_FLT x26 = (x18 * lh_qj) + (-1 * x16 * lh_qk) + (x19 * lh_qw);
	const GEN_FLT x27 = x18 + (2 * ((x20 * lh_qi) + (-1 * x26 * lh_qj))) + lh_pz;
	const GEN_FLT x28 = pow(x22, 2);
	const GEN_FLT x29 = pow(x28, -1) * x27;
	const GEN_FLT x30 = x28 + pow(x27, 2);
	const GEN_FLT x31 = pow(x30, -1);
	const GEN_FLT x32 = x31 * x28;
	const GEN_FLT x33 = ((x25 * x29) + (-1 * x23 * x13)) * x32;
	const GEN_FLT x34 = (2 * ((x26 * lh_qk) + (-1 * x21 * lh_qi))) + x16 + lh_py;
	const GEN_FLT x35 = 0.523598775598299 + tilt_0;
	const GEN_FLT x36 = cos(x35);
	const GEN_FLT x37 = pow(x36, -1);
	const GEN_FLT x38 = pow(x34, 2);
	const GEN_FLT x39 = x30 + x38;
	const GEN_FLT x40 = x37 * pow(x39, -1.0 / 2.0);
	const GEN_FLT x41 = asin(x40 * x34);
	const GEN_FLT x42 = 8.0108022e-06 * x41;
	const GEN_FLT x43 = -8.0108022e-06 + (-1 * x42);
	const GEN_FLT x44 = 0.0028679863 + (x41 * x43);
	const GEN_FLT x45 = 5.3685255e-06 + (x41 * x44);
	const GEN_FLT x46 = 0.0076069798 + (x41 * x45);
	const GEN_FLT x47 = x41 * x46;
	const GEN_FLT x48 = -8.0108022e-06 + (-1.60216044e-05 * x41);
	const GEN_FLT x49 = x44 + (x41 * x48);
	const GEN_FLT x50 = x45 + (x41 * x49);
	const GEN_FLT x51 = x46 + (x50 * x41);
	const GEN_FLT x52 = (x51 * x41) + x47;
	const GEN_FLT x53 = sin(x35);
	const GEN_FLT x54 = tan(x35);
	const GEN_FLT x55 = x54 * pow(x30, -1.0 / 2.0);
	const GEN_FLT x56 = x55 * x34;
	const GEN_FLT x57 = atan2(-1 * x27, x22);
	const GEN_FLT x58 = x57 + (-1 * asin(x56)) + ogeeMag_0;
	const GEN_FLT x59 = (sin(x58) * ogeePhase_0) + curve_0;
	const GEN_FLT x60 = x53 * x59;
	const GEN_FLT x61 = x36 + (-1 * x60 * x52);
	const GEN_FLT x62 = pow(x61, -1);
	const GEN_FLT x63 = pow(x41, 2);
	const GEN_FLT x64 = x63 * x59;
	const GEN_FLT x65 = x64 * x62;
	const GEN_FLT x66 = x56 + (x65 * x46);
	const GEN_FLT x67 = pow((1 + (-1 * pow(x66, 2))), -1.0 / 2.0);
	const GEN_FLT x68 = x7 + (x11 * lh_qk) + (-1 * x24 * lh_qi);
	const GEN_FLT x69 = 2 * x22;
	const GEN_FLT x70 = 2 * x27;
	const GEN_FLT x71 = (x70 * x13) + (x69 * x25);
	const GEN_FLT x72 = 1.0 / 2.0 * x34;
	const GEN_FLT x73 = x72 * x54 * pow(x30, -3.0 / 2.0);
	const GEN_FLT x74 = (-1 * x71 * x73) + (x68 * x55);
	const GEN_FLT x75 = pow((1 + (-1 * pow(x54, 2) * x31 * x38)), -1.0 / 2.0);
	const GEN_FLT x76 = (-1 * x75 * x74) + x33;
	const GEN_FLT x77 = cos(x58) * ogeePhase_0;
	const GEN_FLT x78 = x63 * x62 * x46;
	const GEN_FLT x79 = x78 * x77;
	const GEN_FLT x80 = pow((1 + (-1 * pow(x36, -2) * x38 * pow(x39, -1))), -1.0 / 2.0);
	const GEN_FLT x81 = 2 * x34;
	const GEN_FLT x82 = x72 * x37 * pow(x39, -3.0 / 2.0);
	const GEN_FLT x83 = x80 * ((x68 * x40) + (-1 * x82 * (x71 + (x81 * x68))));
	const GEN_FLT x84 = x83 * x43;
	const GEN_FLT x85 = (x83 * x44) + (x41 * (x84 + (-1 * x83 * x42)));
	const GEN_FLT x86 = (x85 * x41) + (x83 * x45);
	const GEN_FLT x87 = 2.40324066e-05 * x41;
	const GEN_FLT x88 = x53 * x52;
	const GEN_FLT x89 = x88 * x77;
	const GEN_FLT x90 = x64 * pow(x61, -2) * x46;
	const GEN_FLT x91 = 2 * x62 * x59 * x47;
	const GEN_FLT x92 =
		x67 *
		((x83 * x91) +
		 (-1 * x90 *
		  ((-1 * x89 * x76) + (-1 * x60 *
							   ((x83 * x46) + (x86 * x41) +
								(x41 * (x86 + (x83 * x50) +
										(x41 * (x85 + (x41 * ((x83 * x48) + (-1 * x83 * x87) + x84)) + (x83 * x49))))) +
								(x83 * x51))))) +
		 (x86 * x65) + x74 + (x79 * x76));
	const GEN_FLT x93 = cos((-1 * asin(x66)) + x57 + gibPhase_0) * gibMag_0;
	const GEN_FLT x94 = 2 * obj_qk * obj_qj;
	const GEN_FLT x95 = x2 * obj_qw;
	const GEN_FLT x96 = x95 + x94;
	const GEN_FLT x97 = -2 * pow(obj_qi, 2);
	const GEN_FLT x98 = 1 + x8 + x97;
	const GEN_FLT x99 = x5 + (-1 * x6);
	const GEN_FLT x100 = 2 * ((x99 * lh_qw) + (-1 * x98 * lh_qk) + (x96 * lh_qj));
	const GEN_FLT x101 = 2 * ((x98 * lh_qw) + (-1 * x96 * lh_qi) + (x99 * lh_qk));
	const GEN_FLT x102 = x96 + (x101 * lh_qi) + (-1 * x100 * lh_qj);
	const GEN_FLT x103 = 2 * ((-1 * x99 * lh_qj) + (x96 * lh_qw) + (x98 * lh_qi));
	const GEN_FLT x104 = x99 + (x103 * lh_qj) + (-1 * x101 * lh_qk);
	const GEN_FLT x105 = ((x29 * x104) + (-1 * x23 * x102)) * x32;
	const GEN_FLT x106 = x98 + (x100 * lh_qk) + (-1 * x103 * lh_qi);
	const GEN_FLT x107 = (x70 * x102) + (x69 * x104);
	const GEN_FLT x108 = (-1 * x73 * x107) + (x55 * x106);
	const GEN_FLT x109 = (-1 * x75 * x108) + x105;
	const GEN_FLT x110 = x80 * ((x40 * x106) + (-1 * x82 * (x107 + (x81 * x106))));
	const GEN_FLT x111 = x43 * x110;
	const GEN_FLT x112 = (x44 * x110) + (x41 * (x111 + (-1 * x42 * x110)));
	const GEN_FLT x113 = (x41 * x112) + (x45 * x110);
	const GEN_FLT x114 =
		x67 * (x108 + (x91 * x110) +
			   (-1 * x90 *
				((-1 * x89 * x109) +
				 (-1 * x60 *
				  ((x46 * x110) + (x41 * x113) + (x51 * x110) +
				   (x41 * (x113 + (x50 * x110) +
						   (x41 * (x112 + (x41 * ((x48 * x110) + (-1 * x87 * x110) + x111)) + (x49 * x110))))))))) +
			   (x65 * x113) + (x79 * x109));
	const GEN_FLT x115 = x9 + x97;
	const GEN_FLT x116 = x94 + (-1 * x95);
	const GEN_FLT x117 = x1 + x3;
	const GEN_FLT x118 = 2 * ((x117 * lh_qw) + (-1 * x116 * lh_qk) + (x115 * lh_qj));
	const GEN_FLT x119 = 2 * ((x116 * lh_qw) + (-1 * x115 * lh_qi) + (x117 * lh_qk));
	const GEN_FLT x120 = x115 + (x119 * lh_qi) + (-1 * x118 * lh_qj);
	const GEN_FLT x121 = 2 * ((x115 * lh_qw) + (-1 * x117 * lh_qj) + (x116 * lh_qi));
	const GEN_FLT x122 = x117 + (x121 * lh_qj) + (-1 * x119 * lh_qk);
	const GEN_FLT x123 = ((x29 * x122) + (-1 * x23 * x120)) * x32;
	const GEN_FLT x124 = x116 + (x118 * lh_qk) + (-1 * x121 * lh_qi);
	const GEN_FLT x125 = (x70 * x120) + (x69 * x122);
	const GEN_FLT x126 = (-1 * x73 * x125) + (x55 * x124);
	const GEN_FLT x127 = x77 * ((-1 * x75 * x126) + x123);
	const GEN_FLT x128 = x80 * ((x40 * x124) + (-1 * x82 * (x125 + (x81 * x124))));
	const GEN_FLT x129 = x43 * x128;
	const GEN_FLT x130 = (x44 * x128) + (x41 * (x129 + (-1 * x42 * x128)));
	const GEN_FLT x131 = (x41 * x130) + (x45 * x128);
	const GEN_FLT x132 =
		x67 * (x126 + (x91 * x128) +
			   (-1 * x90 *
				((-1 * x88 * x127) +
				 (-1 * x60 *
				  ((x41 * x131) +
				   (x41 * (x131 + (x50 * x128) +
						   (x41 * (x130 + (x41 * ((x48 * x128) + (-1 * x87 * x128) + x129)) + (x49 * x128))))) +
				   (x46 * x128) + (x51 * x128))))) +
			   (x65 * x131) + (x78 * x127));
	out[0] = (-1 * (x92 + (-1 * x33)) * x93) + (-1 * x92) + x33;
	out[1] = (-1 * (x114 + (-1 * x105)) * x93) + (-1 * x114) + x105;
	out[2] = (-1 * (x132 + (-1 * x123)) * x93) + (-1 * x132) + x123;
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
	const GEN_FLT x0 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x1 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x2 = 2 * ((x1 * obj_qj) + (-1 * x0 * obj_qk));
	const GEN_FLT x3 = sensor_x + x2 + obj_px;
	const GEN_FLT x4 = x3 * lh_qw;
	const GEN_FLT x5 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x6 = 2 * ((x5 * obj_qk) + (-1 * x1 * obj_qi));
	const GEN_FLT x7 = sensor_y + x6 + obj_py;
	const GEN_FLT x8 = x7 * lh_qk;
	const GEN_FLT x9 = 2 * ((x0 * obj_qi) + (-1 * x5 * obj_qj));
	const GEN_FLT x10 = sensor_z + x9 + obj_pz;
	const GEN_FLT x11 = x10 * lh_qj;
	const GEN_FLT x12 = (-1 * x8) + x11 + x4;
	const GEN_FLT x13 = x7 * lh_qw;
	const GEN_FLT x14 = x10 * lh_qi;
	const GEN_FLT x15 = x3 * lh_qk;
	const GEN_FLT x16 = x15 + (-1 * x14) + x13;
	const GEN_FLT x17 = x10 + (2 * ((x16 * lh_qi) + (-1 * x12 * lh_qj))) + lh_pz;
	const GEN_FLT x18 = x10 * lh_qw;
	const GEN_FLT x19 = x3 * lh_qj;
	const GEN_FLT x20 = x7 * lh_qi;
	const GEN_FLT x21 = x20 + (-1 * x19) + x18;
	const GEN_FLT x22 = x3 + (2 * ((x21 * lh_qj) + (-1 * x16 * lh_qk))) + lh_px;
	const GEN_FLT x23 = pow(x22, 2);
	const GEN_FLT x24 = x23 + pow(x17, 2);
	const GEN_FLT x25 = pow(x24, -1);
	const GEN_FLT x26 = x25 * x17;
	const GEN_FLT x27 = x7 + (2 * ((x12 * lh_qk) + (-1 * x21 * lh_qi))) + lh_py;
	const GEN_FLT x28 = 0.523598775598299 + tilt_0;
	const GEN_FLT x29 = cos(x28);
	const GEN_FLT x30 = pow(x29, -1);
	const GEN_FLT x31 = pow(x27, 2);
	const GEN_FLT x32 = x24 + x31;
	const GEN_FLT x33 = x30 * pow(x32, -1.0 / 2.0);
	const GEN_FLT x34 = asin(x33 * x27);
	const GEN_FLT x35 = 8.0108022e-06 * x34;
	const GEN_FLT x36 = -8.0108022e-06 + (-1 * x35);
	const GEN_FLT x37 = 0.0028679863 + (x34 * x36);
	const GEN_FLT x38 = 5.3685255e-06 + (x34 * x37);
	const GEN_FLT x39 = 0.0076069798 + (x34 * x38);
	const GEN_FLT x40 = x34 * x39;
	const GEN_FLT x41 = -8.0108022e-06 + (-1.60216044e-05 * x34);
	const GEN_FLT x42 = x37 + (x41 * x34);
	const GEN_FLT x43 = x38 + (x42 * x34);
	const GEN_FLT x44 = x39 + (x43 * x34);
	const GEN_FLT x45 = (x44 * x34) + x40;
	const GEN_FLT x46 = sin(x28);
	const GEN_FLT x47 = tan(x28);
	const GEN_FLT x48 = x47 * pow(x24, -1.0 / 2.0);
	const GEN_FLT x49 = x48 * x27;
	const GEN_FLT x50 = atan2(-1 * x17, x22);
	const GEN_FLT x51 = x50 + (-1 * asin(x49)) + ogeeMag_0;
	const GEN_FLT x52 = (sin(x51) * ogeePhase_0) + curve_0;
	const GEN_FLT x53 = x52 * x46;
	const GEN_FLT x54 = x29 + (-1 * x53 * x45);
	const GEN_FLT x55 = pow(x54, -1);
	const GEN_FLT x56 = pow(x34, 2);
	const GEN_FLT x57 = x52 * x56;
	const GEN_FLT x58 = x57 * x55;
	const GEN_FLT x59 = x49 + (x58 * x39);
	const GEN_FLT x60 = pow((1 + (-1 * pow(x59, 2))), -1.0 / 2.0);
	const GEN_FLT x61 = x22 * x27;
	const GEN_FLT x62 = x47 * pow(x24, -3.0 / 2.0);
	const GEN_FLT x63 = x61 * x62;
	const GEN_FLT x64 = pow((1 + (-1 * pow(x47, 2) * x31 * x25)), -1.0 / 2.0);
	const GEN_FLT x65 = (x63 * x64) + x26;
	const GEN_FLT x66 = cos(x51) * ogeePhase_0;
	const GEN_FLT x67 = x56 * x55 * x39;
	const GEN_FLT x68 = x67 * x66;
	const GEN_FLT x69 = pow((1 + (-1 * pow(x32, -1) * x31 * pow(x29, -2))), -1.0 / 2.0);
	const GEN_FLT x70 = x30 * pow(x32, -3.0 / 2.0);
	const GEN_FLT x71 = x70 * x61;
	const GEN_FLT x72 = x71 * x69;
	const GEN_FLT x73 = -1 * x72 * x36;
	const GEN_FLT x74 = x69 * x37;
	const GEN_FLT x75 = (-1 * x71 * x74) + (x34 * (x73 + (x72 * x35)));
	const GEN_FLT x76 = (x75 * x34) + (-1 * x72 * x38);
	const GEN_FLT x77 = 2.40324066e-05 * x34;
	const GEN_FLT x78 = x69 * x44;
	const GEN_FLT x79 = x45 * x46;
	const GEN_FLT x80 = x79 * x66;
	const GEN_FLT x81 = pow(x54, -2) * x57 * x39;
	const GEN_FLT x82 = 2 * x27;
	const GEN_FLT x83 = x52 * x55 * x40;
	const GEN_FLT x84 = x83 * x69;
	const GEN_FLT x85 =
		x60 * ((-1 * x81 *
				((-1 * x80 * x65) +
				 (-1 * x53 *
				  ((-1 * x72 * x39) + (-1 * x71 * x78) + (x76 * x34) +
				   (x34 * (x76 + (-1 * x72 * x43) +
						   (x34 * (x75 + (x34 * ((-1 * x72 * x41) + x73 + (x72 * x77))) + (-1 * x72 * x42))))))))) +
			   (-1 * x82 * x84 * x70 * x22) + (x76 * x58) + (x68 * x65) + (-1 * x63));
	const GEN_FLT x86 = cos(x50 + (-1 * asin(x59)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x87 = x64 * x48;
	const GEN_FLT x88 = x69 * (x33 + (-1 * x70 * x31));
	const GEN_FLT x89 = x88 * x36;
	const GEN_FLT x90 = (x88 * x37) + (x34 * (x89 + (-1 * x88 * x35)));
	const GEN_FLT x91 = (x90 * x34) + (x88 * x38);
	const GEN_FLT x92 = 2 * x83;
	const GEN_FLT x93 =
		x60 *
		((x88 * x92) + (x58 * x91) + (-1 * x87 * x68) +
		 (-1 * x81 *
		  ((x80 * x87) + (-1 * x53 *
						  ((x34 * (x91 + (x88 * x43) +
								   (x34 * (x90 + (x34 * ((x88 * x41) + x89 + (-1 * x88 * x77))) + (x88 * x42))))) +
						   (x91 * x34) + (x88 * x39) + (x88 * x44))))) +
		 x48);
	const GEN_FLT x94 = x25 * x22;
	const GEN_FLT x95 = -1 * x94;
	const GEN_FLT x96 = x62 * x27;
	const GEN_FLT x97 = x96 * x17;
	const GEN_FLT x98 = (x64 * x97) + x95;
	const GEN_FLT x99 = x70 * x27;
	const GEN_FLT x100 = x99 * x17;
	const GEN_FLT x101 = x69 * x100;
	const GEN_FLT x102 = -1 * x36 * x101;
	const GEN_FLT x103 = (-1 * x74 * x100) + (x34 * (x102 + (x35 * x101)));
	const GEN_FLT x104 = (x34 * x103) + (-1 * x38 * x101);
	const GEN_FLT x105 = 2 * x17;
	const GEN_FLT x106 =
		x60 * ((-1 * x84 * x99 * x105) +
			   (-1 * x81 *
				((-1 * x80 * x98) +
				 (-1 * x53 *
				  ((-1 * x39 * x101) + (x34 * x104) +
				   (x34 * ((-1 * x43 * x101) + x104 +
						   (x34 * (x103 + (x34 * ((-1 * x41 * x101) + (x77 * x101) + x102)) + (-1 * x42 * x101))))) +
				   (-1 * x78 * x100))))) +
			   (x58 * x104) + (x68 * x98) + (-1 * x97));
	const GEN_FLT x107 = pow(x22, -1);
	const GEN_FLT x108 = 2 * x19;
	const GEN_FLT x109 = (2 * x20) + (-1 * x108);
	const GEN_FLT x110 = 2 * x8;
	const GEN_FLT x111 = (2 * x11) + (-1 * x110);
	const GEN_FLT x112 = pow(x23, -1) * x17;
	const GEN_FLT x113 = x25 * x23;
	const GEN_FLT x114 = ((x111 * x112) + (-1 * x109 * x107)) * x113;
	const GEN_FLT x115 = 2 * x14;
	const GEN_FLT x116 = (2 * x15) + (-1 * x115);
	const GEN_FLT x117 = 2 * x22;
	const GEN_FLT x118 = (x109 * x105) + (x111 * x117);
	const GEN_FLT x119 = 1.0 / 2.0 * x96;
	const GEN_FLT x120 = (-1 * x118 * x119) + (x48 * x116);
	const GEN_FLT x121 = (-1 * x64 * x120) + x114;
	const GEN_FLT x122 = 1.0 / 2.0 * x99;
	const GEN_FLT x123 = x69 * ((x33 * x116) + (-1 * x122 * (x118 + (x82 * x116))));
	const GEN_FLT x124 = x36 * x123;
	const GEN_FLT x125 = (x37 * x123) + (x34 * (x124 + (-1 * x35 * x123)));
	const GEN_FLT x126 = (x34 * x125) + (x38 * x123);
	const GEN_FLT x127 =
		x60 * ((-1 * x81 *
				((-1 * x80 * x121) +
				 (-1 * x53 *
				  ((x34 * x126) + (x39 * x123) +
				   (x34 * (x126 + (x43 * x123) +
						   (x34 * (x125 + (x34 * ((x41 * x123) + (-1 * x77 * x123) + x124)) + (x42 * x123))))) +
				   (x44 * x123))))) +
			   (x58 * x126) + x120 + (x92 * x123) + (x68 * x121));
	const GEN_FLT x128 = (-1 * sensor_z) + (-1 * x9) + (-1 * obj_pz);
	const GEN_FLT x129 = 2 * lh_qi;
	const GEN_FLT x130 = 2 * x13;
	const GEN_FLT x131 = x116 + x130 + (x128 * x129);
	const GEN_FLT x132 = 2 * lh_qk;
	const GEN_FLT x133 = (2 * x7 * lh_qj) + (-1 * x128 * x132);
	const GEN_FLT x134 = ((x112 * x133) + (-1 * x107 * x131)) * x113;
	const GEN_FLT x135 = 2 * x18;
	const GEN_FLT x136 = (-1 * x135) + x108 + (-4 * x20);
	const GEN_FLT x137 = (x105 * x131) + (x117 * x133);
	const GEN_FLT x138 = (-1 * x119 * x137) + (x48 * x136);
	const GEN_FLT x139 = (-1 * x64 * x138) + x134;
	const GEN_FLT x140 = x69 * ((x33 * x136) + (-1 * x122 * (x137 + (x82 * x136))));
	const GEN_FLT x141 = x36 * x140;
	const GEN_FLT x142 = (x37 * x140) + (x34 * (x141 + (-1 * x35 * x140)));
	const GEN_FLT x143 = (x34 * x142) + (x38 * x140);
	const GEN_FLT x144 =
		x60 * (x138 + (x92 * x140) +
			   (-1 * x81 *
				((-1 * x80 * x139) +
				 (-1 * x53 *
				  ((x34 * x143) + (x39 * x140) +
				   (x34 * (x143 + (x43 * x140) +
						   (x34 * (x142 + (x34 * ((x41 * x140) + (-1 * x77 * x140) + x141)) + (x42 * x140))))) +
				   (x44 * x140))))) +
			   (x58 * x143) + (x68 * x139));
	const GEN_FLT x145 = 2 * x4;
	const GEN_FLT x146 = (-1 * x145) + x110 + (-4 * x11);
	const GEN_FLT x147 = 2 * ((-1 * sensor_x) + (-1 * x2) + (-1 * obj_px));
	const GEN_FLT x148 = x109 + x135 + (x147 * lh_qj);
	const GEN_FLT x149 = ((x112 * x148) + (-1 * x107 * x146)) * x113;
	const GEN_FLT x150 = (x10 * x132) + (-1 * x147 * lh_qi);
	const GEN_FLT x151 = (x105 * x146) + (x117 * x148);
	const GEN_FLT x152 = (-1 * x119 * x151) + (x48 * x150);
	const GEN_FLT x153 = (-1 * x64 * x152) + x149;
	const GEN_FLT x154 = x69 * ((x33 * x150) + (-1 * x122 * (x151 + (x82 * x150))));
	const GEN_FLT x155 = x36 * x154;
	const GEN_FLT x156 = (x37 * x154) + (x34 * (x155 + (-1 * x35 * x154)));
	const GEN_FLT x157 = (x34 * x156) + (x38 * x154);
	const GEN_FLT x158 =
		x60 * ((x92 * x154) +
			   (-1 * x81 *
				((-1 * x80 * x153) +
				 (-1 * x53 *
				  ((x39 * x154) + (x34 * x157) +
				   (x34 * (x157 + (x43 * x154) +
						   (x34 * (x156 + (x34 * ((x41 * x154) + (-1 * x77 * x154) + x155)) + (x42 * x154))))) +
				   (x44 * x154))))) +
			   (x58 * x157) + x152 + (x68 * x153));
	const GEN_FLT x159 = 2 * ((-1 * sensor_y) + (-1 * x6) + (-1 * obj_py));
	const GEN_FLT x160 = (x3 * x129) + (-1 * x159 * lh_qj);
	const GEN_FLT x161 = (-1 * x130) + x115 + (-4 * x15);
	const GEN_FLT x162 = ((x112 * x161) + (-1 * x107 * x160)) * x113;
	const GEN_FLT x163 = x111 + x145 + (x159 * lh_qk);
	const GEN_FLT x164 = (x105 * x160) + (x117 * x161);
	const GEN_FLT x165 = (-1 * x119 * x164) + (x48 * x163);
	const GEN_FLT x166 = x66 * ((-1 * x64 * x165) + x162);
	const GEN_FLT x167 = x69 * ((x33 * x163) + (-1 * x122 * (x164 + (x82 * x163))));
	const GEN_FLT x168 = x36 * x167;
	const GEN_FLT x169 = (x37 * x167) + (x34 * (x168 + (-1 * x35 * x167)));
	const GEN_FLT x170 = (x34 * x169) + (x38 * x167);
	const GEN_FLT x171 =
		x60 * ((x92 * x167) + x165 +
			   (-1 * x81 *
				((-1 * x79 * x166) +
				 (-1 * x53 *
				  ((x44 * x167) + (x34 * x170) + (x39 * x167) +
				   (x34 * (x170 + (x43 * x167) +
						   (x34 * (x169 + (x34 * ((x41 * x167) + (-1 * x77 * x167) + x168)) + (x42 * x167))))))))) +
			   (x58 * x170) + (x67 * x166));
	out[0] = (-1 * (x85 + (-1 * x26)) * x86) + (-1 * x85) + x26;
	out[1] = (-1 * x86 * x93) + (-1 * x93);
	out[2] = (-1 * x86 * (x106 + x94)) + (-1 * x106) + x95;
	out[3] = (-1 * (x127 + (-1 * x114)) * x86) + (-1 * x127) + x114;
	out[4] = (-1 * x144) + (-1 * (x144 + (-1 * x134)) * x86) + x134;
	out[5] = (-1 * x158) + (-1 * (x158 + (-1 * x149)) * x86) + x149;
	out[6] = (-1 * (x171 + (-1 * x162)) * x86) + x162 + (-1 * x171);
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
	const GEN_FLT x2 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x3 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x4 = sensor_x + (2 * ((x3 * obj_qj) + (-1 * x2 * obj_qk))) + obj_px;
	const GEN_FLT x5 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x6 = sensor_y + (2 * ((x5 * obj_qk) + (-1 * x3 * obj_qi))) + obj_py;
	const GEN_FLT x7 = sensor_z + (2 * ((x2 * obj_qi) + (-1 * x5 * obj_qj))) + obj_pz;
	const GEN_FLT x8 = (x7 * lh_qj) + (-1 * x6 * lh_qk) + (x4 * lh_qw);
	const GEN_FLT x9 = (x4 * lh_qk) + (-1 * x7 * lh_qi) + (x6 * lh_qw);
	const GEN_FLT x10 = x7 + (2 * ((x9 * lh_qi) + (-1 * x8 * lh_qj))) + lh_pz;
	const GEN_FLT x11 = (x6 * lh_qi) + (-1 * x4 * lh_qj) + (x7 * lh_qw);
	const GEN_FLT x12 = x4 + (2 * ((x11 * lh_qj) + (-1 * x9 * lh_qk))) + lh_px;
	const GEN_FLT x13 = pow(x12, 2) + pow(x10, 2);
	const GEN_FLT x14 = x6 + (2 * ((x8 * lh_qk) + (-1 * x11 * lh_qi))) + lh_py;
	const GEN_FLT x15 = x14 * pow(x13, -1.0 / 2.0);
	const GEN_FLT x16 = x1 * x15;
	const GEN_FLT x17 = atan2(-1 * x10, x12);
	const GEN_FLT x18 = x17 + (-1 * asin(x16)) + ogeeMag_0;
	const GEN_FLT x19 = sin(x18);
	const GEN_FLT x20 = (x19 * ogeePhase_0) + curve_0;
	const GEN_FLT x21 = cos(x0);
	const GEN_FLT x22 = pow(x14, 2);
	const GEN_FLT x23 = x13 + x22;
	const GEN_FLT x24 = pow(x23, -1.0 / 2.0) * x14;
	const GEN_FLT x25 = asin(x24 * pow(x21, -1));
	const GEN_FLT x26 = 8.0108022e-06 * x25;
	const GEN_FLT x27 = -8.0108022e-06 + (-1 * x26);
	const GEN_FLT x28 = 0.0028679863 + (x25 * x27);
	const GEN_FLT x29 = 5.3685255e-06 + (x25 * x28);
	const GEN_FLT x30 = 0.0076069798 + (x25 * x29);
	const GEN_FLT x31 = sin(x0);
	const GEN_FLT x32 = x30 * x25;
	const GEN_FLT x33 = -8.0108022e-06 + (-1.60216044e-05 * x25);
	const GEN_FLT x34 = x28 + (x33 * x25);
	const GEN_FLT x35 = x29 + (x34 * x25);
	const GEN_FLT x36 = x30 + (x35 * x25);
	const GEN_FLT x37 = (x36 * x25) + x32;
	const GEN_FLT x38 = x37 * x20;
	const GEN_FLT x39 = x31 * x38;
	const GEN_FLT x40 = x21 + (-1 * x39);
	const GEN_FLT x41 = pow(x40, -1);
	const GEN_FLT x42 = pow(x25, 2);
	const GEN_FLT x43 = x41 * x42;
	const GEN_FLT x44 = x43 * x30;
	const GEN_FLT x45 = x16 + (x44 * x20);
	const GEN_FLT x46 = pow((1 + (-1 * pow(x45, 2))), -1.0 / 2.0);
	const GEN_FLT x47 = pow(x1, 2);
	const GEN_FLT x48 = x15 * (1 + x47);
	const GEN_FLT x49 = cos(x18) * ogeePhase_0;
	const GEN_FLT x50 = x44 * x49;
	const GEN_FLT x51 = x48 * pow((1 + (-1 * x47 * x22 * pow(x13, -1))), -1.0 / 2.0);
	const GEN_FLT x52 = pow(x21, -2);
	const GEN_FLT x53 = x52 * x24 * pow((1 + (-1 * x52 * x22 * pow(x23, -1))), -1.0 / 2.0);
	const GEN_FLT x54 = x53 * x31;
	const GEN_FLT x55 = x54 * x27;
	const GEN_FLT x56 = (x54 * x28) + (x25 * (x55 + (-1 * x54 * x26)));
	const GEN_FLT x57 = (x56 * x25) + (x54 * x29);
	const GEN_FLT x58 = x31 * x20;
	const GEN_FLT x59 = pow(x40, -2) * x42 * x30;
	const GEN_FLT x60 =
		x46 * ((2 * x53 * x58 * x41 * x32) +
			   (-1 * x59 * x20 *
				((x51 * x49 * x31 * x37) +
				 (-1 * x58 *
				  ((x54 * x30) + (x57 * x25) +
				   (x25 * (x57 + (x54 * x35) +
						   (x25 * (x56 + (x25 * ((x54 * x33) + (-2.40324066e-05 * x54 * x25) + x55)) + (x54 * x34))))) +
				   (x54 * x36))) +
				 (-1 * x38 * x21) + (-1 * x31))) +
			   (x57 * x43 * x20) + (-1 * x50 * x51) + x48);
	const GEN_FLT x61 = (-1 * x17) + asin(x45) + (-1 * gibPhase_0);
	const GEN_FLT x62 = cos(x61) * gibMag_0;
	const GEN_FLT x63 = x59 * x39;
	const GEN_FLT x64 = (x63 + x44) * x46;
	const GEN_FLT x65 = x46 * ((x63 * x49) + x50);
	const GEN_FLT x66 = ((x63 * x19) + (x44 * x19)) * x46;
	out[0] = -1;
	out[1] = (-1 * x60 * x62) + (-1 * x60);
	out[2] = (-1 * x64 * x62) + (-1 * x64);
	out[3] = x62;
	out[4] = -1 * sin(x61);
	out[5] = (-1 * x62 * x65) + (-1 * x65);
	out[6] = (-1 * x62 * x66) + (-1 * x66);
}

static inline FLT gen_reproject_axis_y_gen2(const SurvivePose *obj_p, const FLT *sensor_pt, const SurvivePose *lh_p,
											const BaseStationCal *bsc1) {
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
	const GEN_FLT x0 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x3 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x4 = sensor_z + (2 * ((x3 * obj_qi) + (-1 * x2 * obj_qj))) + obj_pz;
	const GEN_FLT x5 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x6 = sensor_x + (2 * ((x5 * obj_qj) + (-1 * x3 * obj_qk))) + obj_px;
	const GEN_FLT x7 = sensor_y + (2 * ((x2 * obj_qk) + (-1 * x5 * obj_qi))) + obj_py;
	const GEN_FLT x8 = (x7 * lh_qi) + (-1 * x6 * lh_qj) + (x4 * lh_qw);
	const GEN_FLT x9 = (x4 * lh_qj) + (-1 * x7 * lh_qk) + (x6 * lh_qw);
	const GEN_FLT x10 = x7 + (2 * ((x9 * lh_qk) + (-1 * x8 * lh_qi))) + lh_py;
	const GEN_FLT x11 = (x6 * lh_qk) + (-1 * x4 * lh_qi) + (x7 * lh_qw);
	const GEN_FLT x12 = x4 + (2 * ((x11 * lh_qi) + (-1 * x9 * lh_qj))) + lh_pz;
	const GEN_FLT x13 = x6 + (2 * ((x8 * lh_qj) + (-1 * x11 * lh_qk))) + lh_px;
	const GEN_FLT x14 = pow(x13, 2) + pow(x12, 2);
	const GEN_FLT x15 = asin(pow(x1, -1) * x10 * pow((x14 + pow(x10, 2)), -1.0 / 2.0));
	const GEN_FLT x16 = 0.0028679863 + (x15 * (-8.0108022e-06 + (-8.0108022e-06 * x15)));
	const GEN_FLT x17 = 5.3685255e-06 + (x15 * x16);
	const GEN_FLT x18 = 0.0076069798 + (x15 * x17);
	const GEN_FLT x19 = -1 * tan(x0) * pow(x14, -1.0 / 2.0) * x10;
	const GEN_FLT x20 = atan2(-1 * x12, x13);
	const GEN_FLT x21 = (sin(x20 + (-1 * asin(x19)) + ogeeMag_1) * ogeePhase_1) + curve_1;
	const GEN_FLT x22 =
		x20 +
		(-1 *
		 asin(x19 + (x21 * pow(x15, 2) * x18 *
					 pow((x1 + (sin(x0) * x21 *
								((x15 * (x18 + (x15 * (x17 + (x15 * (x16 + (x15 * (-8.0108022e-06 +
																				   (-1.60216044e-05 * x15))))))))) +
								 (x15 * x18)))),
						 -1))));
	return -1.5707963267949 + x22 + (sin(x22 + gibPhase_1) * gibMag_1) + (-1 * phase_1);
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
	const GEN_FLT x0 = 2 * lh_qw;
	const GEN_FLT x1 = x0 * lh_qj;
	const GEN_FLT x2 = 2 * lh_qk;
	const GEN_FLT x3 = x2 * lh_qi;
	const GEN_FLT x4 = x3 + (-1 * x1);
	const GEN_FLT x5 = obj_qj * sensor_x;
	const GEN_FLT x6 = obj_qw * sensor_z;
	const GEN_FLT x7 = obj_qi * sensor_y;
	const GEN_FLT x8 = x7 + x6 + (-1 * x5);
	const GEN_FLT x9 = obj_qw * sensor_x;
	const GEN_FLT x10 = obj_qk * sensor_y;
	const GEN_FLT x11 = obj_qj * sensor_z;
	const GEN_FLT x12 = x11 + (-1 * x10) + x9;
	const GEN_FLT x13 = sensor_y + (2 * ((x12 * obj_qk) + (-1 * x8 * obj_qi))) + obj_py;
	const GEN_FLT x14 = obj_qw * sensor_y;
	const GEN_FLT x15 = obj_qi * sensor_z;
	const GEN_FLT x16 = obj_qk * sensor_x;
	const GEN_FLT x17 = x16 + (-1 * x15) + x14;
	const GEN_FLT x18 = sensor_z + (2 * ((x17 * obj_qi) + (-1 * x12 * obj_qj))) + obj_pz;
	const GEN_FLT x19 = sensor_x + (2 * ((x8 * obj_qj) + (-1 * x17 * obj_qk))) + obj_px;
	const GEN_FLT x20 = (x19 * lh_qk) + (-1 * x18 * lh_qi) + (x13 * lh_qw);
	const GEN_FLT x21 = (x13 * lh_qi) + (-1 * x19 * lh_qj) + (x18 * lh_qw);
	const GEN_FLT x22 = x19 + (2 * ((x21 * lh_qj) + (-1 * x20 * lh_qk))) + lh_px;
	const GEN_FLT x23 = pow(x22, -1);
	const GEN_FLT x24 = -2 * pow(lh_qk, 2);
	const GEN_FLT x25 = -2 * pow(lh_qj, 2);
	const GEN_FLT x26 = 1 + x25 + x24;
	const GEN_FLT x27 = (x18 * lh_qj) + (-1 * x13 * lh_qk) + (x19 * lh_qw);
	const GEN_FLT x28 = x18 + (2 * ((x20 * lh_qi) + (-1 * x27 * lh_qj))) + lh_pz;
	const GEN_FLT x29 = pow(x22, 2);
	const GEN_FLT x30 = x28 * pow(x29, -1);
	const GEN_FLT x31 = x29 + pow(x28, 2);
	const GEN_FLT x32 = pow(x31, -1);
	const GEN_FLT x33 = x32 * x29;
	const GEN_FLT x34 = x33 * ((x30 * x26) + (-1 * x4 * x23));
	const GEN_FLT x35 = x13 + (2 * ((x27 * lh_qk) + (-1 * x21 * lh_qi))) + lh_py;
	const GEN_FLT x36 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x37 = cos(x36);
	const GEN_FLT x38 = pow(x37, -1);
	const GEN_FLT x39 = pow(x35, 2);
	const GEN_FLT x40 = x31 + x39;
	const GEN_FLT x41 = pow(x40, -1.0 / 2.0) * x38;
	const GEN_FLT x42 = asin(x41 * x35);
	const GEN_FLT x43 = 8.0108022e-06 * x42;
	const GEN_FLT x44 = pow((1 + (-1 * pow(x40, -1) * pow(x37, -2) * x39)), -1.0 / 2.0);
	const GEN_FLT x45 = 2 * lh_qj;
	const GEN_FLT x46 = x45 * lh_qi;
	const GEN_FLT x47 = x2 * lh_qw;
	const GEN_FLT x48 = x47 + x46;
	const GEN_FLT x49 = 2 * x35;
	const GEN_FLT x50 = 2 * x22;
	const GEN_FLT x51 = 2 * x28;
	const GEN_FLT x52 = (x4 * x51) + (x50 * x26);
	const GEN_FLT x53 = 1.0 / 2.0 * x35;
	const GEN_FLT x54 = x53 * pow(x40, -3.0 / 2.0) * x38;
	const GEN_FLT x55 = x44 * ((x41 * x48) + (-1 * x54 * (x52 + (x48 * x49))));
	const GEN_FLT x56 = -8.0108022e-06 + (-1 * x43);
	const GEN_FLT x57 = x56 * x55;
	const GEN_FLT x58 = 0.0028679863 + (x56 * x42);
	const GEN_FLT x59 = (x58 * x55) + (x42 * (x57 + (-1 * x55 * x43)));
	const GEN_FLT x60 = 5.3685255e-06 + (x58 * x42);
	const GEN_FLT x61 = (x60 * x55) + (x59 * x42);
	const GEN_FLT x62 = 0.0076069798 + (x60 * x42);
	const GEN_FLT x63 = x62 * x42;
	const GEN_FLT x64 = -8.0108022e-06 + (-1.60216044e-05 * x42);
	const GEN_FLT x65 = x58 + (x64 * x42);
	const GEN_FLT x66 = x60 + (x65 * x42);
	const GEN_FLT x67 = x62 + (x66 * x42);
	const GEN_FLT x68 = (x67 * x42) + x63;
	const GEN_FLT x69 = sin(x36);
	const GEN_FLT x70 = tan(x36);
	const GEN_FLT x71 = x70 * pow(x31, -1.0 / 2.0);
	const GEN_FLT x72 = -1 * x71 * x35;
	const GEN_FLT x73 = atan2(-1 * x28, x22);
	const GEN_FLT x74 = x73 + (-1 * asin(x72)) + ogeeMag_1;
	const GEN_FLT x75 = (sin(x74) * ogeePhase_1) + curve_1;
	const GEN_FLT x76 = x75 * x69;
	const GEN_FLT x77 = x37 + (x76 * x68);
	const GEN_FLT x78 = pow(x77, -1);
	const GEN_FLT x79 = pow(x42, 2);
	const GEN_FLT x80 = x79 * x75;
	const GEN_FLT x81 = x80 * x78;
	const GEN_FLT x82 = pow((1 + (-1 * pow(x70, 2) * x32 * x39)), -1.0 / 2.0);
	const GEN_FLT x83 = x70 * x53 * pow(x31, -3.0 / 2.0);
	const GEN_FLT x84 = (x83 * x52) + (-1 * x71 * x48);
	const GEN_FLT x85 = (-1 * x82 * x84) + x34;
	const GEN_FLT x86 = cos(x74) * ogeePhase_1;
	const GEN_FLT x87 = x79 * x78 * x62;
	const GEN_FLT x88 = x86 * x87;
	const GEN_FLT x89 = 2 * x78 * x75 * x63;
	const GEN_FLT x90 = 2.40324066e-05 * x42;
	const GEN_FLT x91 = x68 * x69;
	const GEN_FLT x92 = x86 * x91;
	const GEN_FLT x93 = x80 * pow(x77, -2) * x62;
	const GEN_FLT x94 = x72 + (x81 * x62);
	const GEN_FLT x95 = pow((1 + (-1 * pow(x94, 2))), -1.0 / 2.0);
	const GEN_FLT x96 =
		(-1 * x95 *
		 ((-1 * x93 *
		   ((x85 * x92) +
			(x76 * ((x62 * x55) + (x61 * x42) + (x67 * x55) +
					(x42 * (x61 + (x42 * (x59 + (x65 * x55) + (x42 * ((x64 * x55) + (-1 * x55 * x90) + x57)))) +
							(x66 * x55))))))) +
		  (x89 * x55) + (x88 * x85) + x84 + (x81 * x61))) +
		x34;
	const GEN_FLT x97 = cos(x73 + (-1 * asin(x94)) + gibPhase_1) * gibMag_1;
	const GEN_FLT x98 = x2 * lh_qj;
	const GEN_FLT x99 = x0 * lh_qi;
	const GEN_FLT x100 = x99 + x98;
	const GEN_FLT x101 = x46 + (-1 * x47);
	const GEN_FLT x102 = ((x30 * x101) + (-1 * x23 * x100)) * x33;
	const GEN_FLT x103 = 1 + (-2 * pow(lh_qi, 2));
	const GEN_FLT x104 = x103 + x24;
	const GEN_FLT x105 = (x51 * x100) + (x50 * x101);
	const GEN_FLT x106 = x44 * ((x41 * x104) + (-1 * x54 * (x105 + (x49 * x104))));
	const GEN_FLT x107 = x56 * x106;
	const GEN_FLT x108 = (x58 * x106) + (x42 * (x107 + (-1 * x43 * x106)));
	const GEN_FLT x109 = (x60 * x106) + (x42 * x108);
	const GEN_FLT x110 = (x83 * x105) + (-1 * x71 * x104);
	const GEN_FLT x111 = (-1 * x82 * x110) + x102;
	const GEN_FLT x112 =
		(-1 * x95 *
		 (x110 + (x89 * x106) +
		  (-1 * x93 *
		   ((x92 * x111) +
			(x76 * ((x62 * x106) + (x42 * x109) + (x67 * x106) +
					(x42 * (x109 + (x42 * ((x65 * x106) + x108 + (x42 * (x107 + (x64 * x106) + (-1 * x90 * x106))))) +
							(x66 * x106))))))) +
		  (x88 * x111) + (x81 * x109))) +
		x102;
	const GEN_FLT x113 = x103 + x25;
	const GEN_FLT x114 = x1 + x3;
	const GEN_FLT x115 = ((x30 * x114) + (-1 * x23 * x113)) * x33;
	const GEN_FLT x116 = x98 + (-1 * x99);
	const GEN_FLT x117 = (x51 * x113) + (x50 * x114);
	const GEN_FLT x118 = (x41 * x116) + (-1 * x54 * (x117 + (x49 * x116)));
	const GEN_FLT x119 = x44 * x118;
	const GEN_FLT x120 = x56 * x44;
	const GEN_FLT x121 = x118 * x120;
	const GEN_FLT x122 = (x58 * x119) + (x42 * (x121 + (-1 * x43 * x119)));
	const GEN_FLT x123 = (x60 * x119) + (x42 * x122);
	const GEN_FLT x124 = (x83 * x117) + (-1 * x71 * x116);
	const GEN_FLT x125 = (-1 * x82 * x124) + x115;
	const GEN_FLT x126 = x66 * x44;
	const GEN_FLT x127 = x64 * x44;
	const GEN_FLT x128 =
		(-1 * x95 *
		 (x124 + (x89 * x119) +
		  (-1 * x93 *
		   ((x92 * x125) +
			(x76 * ((x62 * x119) + (x42 * x123) + (x67 * x119) +
					(x42 * (x123 + (x42 * (x122 + (x65 * x119) + (x42 * ((x118 * x127) + (-1 * x90 * x119) + x121)))) +
							(x118 * x126))))))) +
		  (x88 * x125) + (x81 * x123))) +
		x115;
	const GEN_FLT x129 = 2 * x5;
	const GEN_FLT x130 = 2 * x7;
	const GEN_FLT x131 = x130 + (-1 * x129);
	const GEN_FLT x132 = 2 * x15;
	const GEN_FLT x133 = 2 * x16;
	const GEN_FLT x134 = x133 + (-1 * x132);
	const GEN_FLT x135 = 2 * x10;
	const GEN_FLT x136 = 2 * x11;
	const GEN_FLT x137 = x136 + (-1 * x135);
	const GEN_FLT x138 = (x137 * lh_qw) + (-1 * x134 * lh_qk) + (x131 * lh_qj);
	const GEN_FLT x139 = (x134 * lh_qw) + (-1 * x131 * lh_qi) + (x137 * lh_qk);
	const GEN_FLT x140 = 2 * lh_qi;
	const GEN_FLT x141 = x131 + (x139 * x140) + (-1 * x45 * x138);
	const GEN_FLT x142 = (x131 * lh_qw) + (-1 * x137 * lh_qj) + (x134 * lh_qi);
	const GEN_FLT x143 = x137 + (x45 * x142) + (-1 * x2 * x139);
	const GEN_FLT x144 = ((x30 * x143) + (-1 * x23 * x141)) * x33;
	const GEN_FLT x145 = (x2 * x138) + x134 + (-1 * x140 * x142);
	const GEN_FLT x146 = (x51 * x141) + (x50 * x143);
	const GEN_FLT x147 = x44 * ((x41 * x145) + (-1 * x54 * (x146 + (x49 * x145))));
	const GEN_FLT x148 = x56 * x147;
	const GEN_FLT x149 = (x58 * x147) + (x42 * (x148 + (-1 * x43 * x147)));
	const GEN_FLT x150 = (x60 * x147) + (x42 * x149);
	const GEN_FLT x151 = (x83 * x146) + (-1 * x71 * x145);
	const GEN_FLT x152 = (-1 * x82 * x151) + x144;
	const GEN_FLT x153 =
		(-1 * x95 *
		 (x151 +
		  (-1 * x93 *
		   ((x92 * x152) +
			(x76 * ((x62 * x147) + (x67 * x147) +
					(x42 * (x150 + (x42 * (x149 + (x65 * x147) + (x42 * ((x64 * x147) + (-1 * x90 * x147) + x148)))) +
							(x66 * x147))) +
					(x42 * x150))))) +
		  (x89 * x147) + (x88 * x152) + (x81 * x150))) +
		x144;
	const GEN_FLT x154 = 2 * x14;
	const GEN_FLT x155 = x154 + (-4 * x15) + x133;
	const GEN_FLT x156 = 2 * x6;
	const GEN_FLT x157 = x129 + (-1 * x156) + (-4 * x7);
	const GEN_FLT x158 = 2 * obj_qk * sensor_z;
	const GEN_FLT x159 = 2 * obj_qj * sensor_y;
	const GEN_FLT x160 = x159 + x158;
	const GEN_FLT x161 = (x160 * lh_qw) + (-1 * x157 * lh_qk) + (x155 * lh_qj);
	const GEN_FLT x162 = (x157 * lh_qw) + (-1 * x155 * lh_qi) + (x160 * lh_qk);
	const GEN_FLT x163 = x155 + (x162 * x140) + (-1 * x45 * x161);
	const GEN_FLT x164 = (x155 * lh_qw) + (-1 * x160 * lh_qj) + (x157 * lh_qi);
	const GEN_FLT x165 = x160 + (x45 * x164) + (-1 * x2 * x162);
	const GEN_FLT x166 = ((x30 * x165) + (-1 * x23 * x163)) * x33;
	const GEN_FLT x167 = x157 + (x2 * x161) + (-1 * x164 * x140);
	const GEN_FLT x168 = (x51 * x163) + (x50 * x165);
	const GEN_FLT x169 = (x41 * x167) + (-1 * x54 * (x168 + (x49 * x167)));
	const GEN_FLT x170 = x44 * x169;
	const GEN_FLT x171 = x120 * x169;
	const GEN_FLT x172 = x58 * x44;
	const GEN_FLT x173 = (x169 * x172) + (x42 * (x171 + (-1 * x43 * x170)));
	const GEN_FLT x174 = (x60 * x170) + (x42 * x173);
	const GEN_FLT x175 = (x83 * x168) + (-1 * x71 * x167);
	const GEN_FLT x176 = x86 * ((-1 * x82 * x175) + x166);
	const GEN_FLT x177 = x62 * x44;
	const GEN_FLT x178 =
		(-1 * x95 *
		 (x175 +
		  (-1 * x93 *
		   ((x91 * x176) +
			(x76 * ((x42 * x174) + (x67 * x170) + (x169 * x177) +
					(x42 * (x174 + (x42 * (x173 + (x65 * x170) + (x42 * ((x127 * x169) + (-1 * x90 * x170) + x171)))) +
							(x126 * x169))))))) +
		  (x89 * x170) + (x87 * x176) + (x81 * x174))) +
		x166;
	const GEN_FLT x179 = 2 * x9;
	const GEN_FLT x180 = (-1 * x179) + x135 + (-4 * x11);
	const GEN_FLT x181 = 2 * obj_qi * sensor_x;
	const GEN_FLT x182 = x158 + x181;
	const GEN_FLT x183 = (-4 * x5) + x156 + x130;
	const GEN_FLT x184 = (x183 * lh_qw) + (-1 * x182 * lh_qk) + (x180 * lh_qj);
	const GEN_FLT x185 = (x182 * lh_qw) + (-1 * x180 * lh_qi) + (x183 * lh_qk);
	const GEN_FLT x186 = x180 + (x185 * x140) + (-1 * x45 * x184);
	const GEN_FLT x187 = (-1 * x183 * lh_qj) + (x180 * lh_qw) + (x182 * lh_qi);
	const GEN_FLT x188 = (x45 * x187) + x183 + (-1 * x2 * x185);
	const GEN_FLT x189 = ((x30 * x188) + (-1 * x23 * x186)) * x33;
	const GEN_FLT x190 = x182 + (x2 * x184) + (-1 * x187 * x140);
	const GEN_FLT x191 = (x51 * x186) + (x50 * x188);
	const GEN_FLT x192 = (x41 * x190) + (-1 * x54 * (x191 + (x49 * x190)));
	const GEN_FLT x193 = x44 * x192;
	const GEN_FLT x194 = x120 * x192;
	const GEN_FLT x195 = (x172 * x192) + (x42 * (x194 + (-1 * x43 * x193)));
	const GEN_FLT x196 = (x60 * x193) + (x42 * x195);
	const GEN_FLT x197 = (x83 * x191) + (-1 * x71 * x190);
	const GEN_FLT x198 = (-1 * x82 * x197) + x189;
	const GEN_FLT x199 =
		(-1 * x95 *
		 (x197 + (x89 * x193) +
		  (-1 * x93 *
		   ((x92 * x198) +
			(x76 * ((x177 * x192) + (x42 * x196) + (x67 * x193) +
					(x42 * (x196 + (x42 * (x195 + (x65 * x193) + (x42 * ((x127 * x192) + (-1 * x90 * x193) + x194)))) +
							(x126 * x192))))))) +
		  (x88 * x198) + (x81 * x196))) +
		x189;
	const GEN_FLT x200 = x181 + x159;
	const GEN_FLT x201 = x179 + (-4 * x10) + x136;
	const GEN_FLT x202 = (-1 * x154) + x132 + (-4 * x16);
	const GEN_FLT x203 = (-1 * x201 * lh_qk) + (x202 * lh_qw) + (x200 * lh_qj);
	const GEN_FLT x204 = (-1 * x200 * lh_qi) + (x201 * lh_qw) + (x202 * lh_qk);
	const GEN_FLT x205 = x200 + (x204 * x140) + (-1 * x45 * x203);
	const GEN_FLT x206 = (x200 * lh_qw) + (-1 * x202 * lh_qj) + (x201 * lh_qi);
	const GEN_FLT x207 = x202 + (x45 * x206) + (-1 * x2 * x204);
	const GEN_FLT x208 = ((x30 * x207) + (-1 * x23 * x205)) * x33;
	const GEN_FLT x209 = x201 + (x2 * x203) + (-1 * x206 * x140);
	const GEN_FLT x210 = (x51 * x205) + (x50 * x207);
	const GEN_FLT x211 = (x41 * x209) + (-1 * x54 * (x210 + (x49 * x209)));
	const GEN_FLT x212 = x44 * x211;
	const GEN_FLT x213 = x211 * x120;
	const GEN_FLT x214 = (x211 * x172) + (x42 * (x213 + (-1 * x43 * x212)));
	const GEN_FLT x215 = (x60 * x212) + (x42 * x214);
	const GEN_FLT x216 = (x83 * x210) + (-1 * x71 * x209);
	const GEN_FLT x217 = (-1 * x82 * x216) + x208;
	const GEN_FLT x218 =
		(-1 * x95 *
		 (x216 +
		  (-1 * x93 *
		   ((x92 * x217) +
			(x76 * ((x42 * x215) + (x67 * x212) + (x211 * x177) +
					(x42 * (x215 + (x42 * (x214 + (x65 * x212) + (x42 * ((x211 * x127) + (-1 * x90 * x212) + x213)))) +
							(x211 * x126))))))) +
		  (x89 * x212) + (x88 * x217) + (x81 * x215))) +
		x208;
	out[0] = x96 + (x97 * x96);
	out[1] = x112 + (x97 * x112);
	out[2] = x128 + (x97 * x128);
	out[3] = x153 + (x97 * x153);
	out[4] = x178 + (x97 * x178);
	out[5] = x199 + (x97 * x199);
	out[6] = x218 + (x97 * x218);
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
	const GEN_FLT x0 = 2 * obj_qw;
	const GEN_FLT x1 = x0 * obj_qj;
	const GEN_FLT x2 = 2 * obj_qi;
	const GEN_FLT x3 = x2 * obj_qk;
	const GEN_FLT x4 = x3 + (-1 * x1);
	const GEN_FLT x5 = x2 * obj_qj;
	const GEN_FLT x6 = x0 * obj_qk;
	const GEN_FLT x7 = x6 + x5;
	const GEN_FLT x8 = -2 * pow(obj_qj, 2);
	const GEN_FLT x9 = 1 + (-2 * pow(obj_qk, 2));
	const GEN_FLT x10 = x9 + x8;
	const GEN_FLT x11 = (x10 * lh_qw) + (-1 * x7 * lh_qk) + (x4 * lh_qj);
	const GEN_FLT x12 = 2 * lh_qj;
	const GEN_FLT x13 = (x7 * lh_qw) + (-1 * x4 * lh_qi) + (x10 * lh_qk);
	const GEN_FLT x14 = 2 * lh_qi;
	const GEN_FLT x15 = x4 + (x14 * x13) + (-1 * x12 * x11);
	const GEN_FLT x16 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x17 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x18 = sensor_y + (2 * ((x17 * obj_qk) + (-1 * x16 * obj_qi))) + obj_py;
	const GEN_FLT x19 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x20 = sensor_z + (2 * ((x19 * obj_qi) + (-1 * x17 * obj_qj))) + obj_pz;
	const GEN_FLT x21 = sensor_x + (2 * ((x16 * obj_qj) + (-1 * x19 * obj_qk))) + obj_px;
	const GEN_FLT x22 = (x21 * lh_qk) + (-1 * x20 * lh_qi) + (x18 * lh_qw);
	const GEN_FLT x23 = (x18 * lh_qi) + (-1 * x21 * lh_qj) + (x20 * lh_qw);
	const GEN_FLT x24 = x21 + (2 * ((x23 * lh_qj) + (-1 * x22 * lh_qk))) + lh_px;
	const GEN_FLT x25 = pow(x24, -1);
	const GEN_FLT x26 = 2 * lh_qk;
	const GEN_FLT x27 = (x4 * lh_qw) + (-1 * x10 * lh_qj) + (x7 * lh_qi);
	const GEN_FLT x28 = x10 + (x27 * x12) + (-1 * x26 * x13);
	const GEN_FLT x29 = (x20 * lh_qj) + (-1 * x18 * lh_qk) + (x21 * lh_qw);
	const GEN_FLT x30 = x20 + (2 * ((x22 * lh_qi) + (-1 * x29 * lh_qj))) + lh_pz;
	const GEN_FLT x31 = pow(x24, 2);
	const GEN_FLT x32 = x30 * pow(x31, -1);
	const GEN_FLT x33 = x31 + pow(x30, 2);
	const GEN_FLT x34 = pow(x33, -1);
	const GEN_FLT x35 = x31 * x34;
	const GEN_FLT x36 = ((x32 * x28) + (-1 * x25 * x15)) * x35;
	const GEN_FLT x37 = x18 + (2 * ((x29 * lh_qk) + (-1 * x23 * lh_qi))) + lh_py;
	const GEN_FLT x38 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x39 = cos(x38);
	const GEN_FLT x40 = pow(x39, -1);
	const GEN_FLT x41 = pow(x37, 2);
	const GEN_FLT x42 = x33 + x41;
	const GEN_FLT x43 = x40 * pow(x42, -1.0 / 2.0);
	const GEN_FLT x44 = asin(x43 * x37);
	const GEN_FLT x45 = 8.0108022e-06 * x44;
	const GEN_FLT x46 = pow((1 + (-1 * x41 * pow(x42, -1) * pow(x39, -2))), -1.0 / 2.0);
	const GEN_FLT x47 = x7 + (x26 * x11) + (-1 * x27 * x14);
	const GEN_FLT x48 = 2 * x37;
	const GEN_FLT x49 = 2 * x24;
	const GEN_FLT x50 = 2 * x30;
	const GEN_FLT x51 = (x50 * x15) + (x49 * x28);
	const GEN_FLT x52 = 1.0 / 2.0 * x37;
	const GEN_FLT x53 = x52 * x40 * pow(x42, -3.0 / 2.0);
	const GEN_FLT x54 = (x43 * x47) + (-1 * x53 * (x51 + (x47 * x48)));
	const GEN_FLT x55 = x54 * x46;
	const GEN_FLT x56 = -8.0108022e-06 + (-1 * x45);
	const GEN_FLT x57 = x56 * x55;
	const GEN_FLT x58 = 0.0028679863 + (x56 * x44);
	const GEN_FLT x59 = x58 * x46;
	const GEN_FLT x60 = (x54 * x59) + (x44 * (x57 + (-1 * x55 * x45)));
	const GEN_FLT x61 = 5.3685255e-06 + (x58 * x44);
	const GEN_FLT x62 = (x61 * x55) + (x60 * x44);
	const GEN_FLT x63 = 0.0076069798 + (x61 * x44);
	const GEN_FLT x64 = x63 * x44;
	const GEN_FLT x65 = -8.0108022e-06 + (-1.60216044e-05 * x44);
	const GEN_FLT x66 = x58 + (x65 * x44);
	const GEN_FLT x67 = x61 + (x66 * x44);
	const GEN_FLT x68 = x63 + (x67 * x44);
	const GEN_FLT x69 = (x68 * x44) + x64;
	const GEN_FLT x70 = sin(x38);
	const GEN_FLT x71 = tan(x38);
	const GEN_FLT x72 = x71 * pow(x33, -1.0 / 2.0);
	const GEN_FLT x73 = -1 * x72 * x37;
	const GEN_FLT x74 = atan2(-1 * x30, x24);
	const GEN_FLT x75 = x74 + (-1 * asin(x73)) + ogeeMag_1;
	const GEN_FLT x76 = (sin(x75) * ogeePhase_1) + curve_1;
	const GEN_FLT x77 = x70 * x76;
	const GEN_FLT x78 = x39 + (x77 * x69);
	const GEN_FLT x79 = pow(x78, -1);
	const GEN_FLT x80 = pow(x44, 2);
	const GEN_FLT x81 = x80 * x76;
	const GEN_FLT x82 = x81 * x79;
	const GEN_FLT x83 = x71 * x52 * pow(x33, -3.0 / 2.0);
	const GEN_FLT x84 = (x83 * x51) + (-1 * x72 * x47);
	const GEN_FLT x85 = pow((1 + (-1 * pow(x71, 2) * x41 * x34)), -1.0 / 2.0);
	const GEN_FLT x86 = (-1 * x84 * x85) + x36;
	const GEN_FLT x87 = cos(x75) * ogeePhase_1;
	const GEN_FLT x88 = x80 * x79 * x63;
	const GEN_FLT x89 = x88 * x87;
	const GEN_FLT x90 = 2 * x79 * x76 * x64;
	const GEN_FLT x91 = 2.40324066e-05 * x44;
	const GEN_FLT x92 = x70 * x69;
	const GEN_FLT x93 = x87 * x92;
	const GEN_FLT x94 = x81 * pow(x78, -2) * x63;
	const GEN_FLT x95 = x73 + (x82 * x63);
	const GEN_FLT x96 = pow((1 + (-1 * pow(x95, 2))), -1.0 / 2.0);
	const GEN_FLT x97 =
		(-1 * x96 *
		 ((-1 * x94 *
		   ((x86 * x93) +
			(x77 * ((x63 * x55) + (x68 * x55) + (x62 * x44) +
					(x44 * (x62 + (x44 * (x60 + (x66 * x55) + (x44 * ((x65 * x55) + (-1 * x55 * x91) + x57)))) +
							(x67 * x55))))))) +
		  (x55 * x90) + (x89 * x86) + x84 + (x82 * x62))) +
		x36;
	const GEN_FLT x98 = cos(x74 + (-1 * asin(x95)) + gibPhase_1) * gibMag_1;
	const GEN_FLT x99 = 2 * obj_qk * obj_qj;
	const GEN_FLT x100 = x0 * obj_qi;
	const GEN_FLT x101 = x100 + x99;
	const GEN_FLT x102 = -2 * pow(obj_qi, 2);
	const GEN_FLT x103 = x9 + x102;
	const GEN_FLT x104 = x5 + (-1 * x6);
	const GEN_FLT x105 = (x104 * lh_qw) + (-1 * x103 * lh_qk) + (x101 * lh_qj);
	const GEN_FLT x106 = 2 * ((x103 * lh_qw) + (-1 * x101 * lh_qi) + (x104 * lh_qk));
	const GEN_FLT x107 = x101 + (x106 * lh_qi) + (-1 * x12 * x105);
	const GEN_FLT x108 = (x101 * lh_qw) + (-1 * x104 * lh_qj) + (x103 * lh_qi);
	const GEN_FLT x109 = x104 + (x12 * x108) + (-1 * x106 * lh_qk);
	const GEN_FLT x110 = ((x32 * x109) + (-1 * x25 * x107)) * x35;
	const GEN_FLT x111 = x103 + (x26 * x105) + (-1 * x14 * x108);
	const GEN_FLT x112 = (x50 * x107) + (x49 * x109);
	const GEN_FLT x113 = (x43 * x111) + (-1 * x53 * (x112 + (x48 * x111)));
	const GEN_FLT x114 = x46 * x113;
	const GEN_FLT x115 = x56 * x114;
	const GEN_FLT x116 = (x59 * x113) + (x44 * (x115 + (-1 * x45 * x114)));
	const GEN_FLT x117 = (x61 * x114) + (x44 * x116);
	const GEN_FLT x118 = (x83 * x112) + (-1 * x72 * x111);
	const GEN_FLT x119 = x87 * ((-1 * x85 * x118) + x110);
	const GEN_FLT x120 =
		(-1 * x96 *
		 (x118 +
		  (-1 * x94 *
		   ((x92 * x119) +
			(x77 * ((x63 * x114) + (x44 * x117) + (x68 * x114) +
					(x44 * ((x44 * (x116 + (x66 * x114) + (x44 * ((x65 * x114) + (-1 * x91 * x114) + x115)))) + x117 +
							(x67 * x114))))))) +
		  (x90 * x114) + (x88 * x119) + (x82 * x117))) +
		x110;
	const GEN_FLT x121 = 1 + x102 + x8;
	const GEN_FLT x122 = x99 + (-1 * x100);
	const GEN_FLT x123 = x1 + x3;
	const GEN_FLT x124 = (x123 * lh_qw) + (-1 * x122 * lh_qk) + (x121 * lh_qj);
	const GEN_FLT x125 = (-1 * x121 * lh_qi) + (x122 * lh_qw) + (x123 * lh_qk);
	const GEN_FLT x126 = x121 + (x14 * x125) + (-1 * x12 * x124);
	const GEN_FLT x127 = (x121 * lh_qw) + (-1 * x123 * lh_qj) + (x122 * lh_qi);
	const GEN_FLT x128 = x123 + (x12 * x127) + (-1 * x26 * x125);
	const GEN_FLT x129 = ((x32 * x128) + (-1 * x25 * x126)) * x35;
	const GEN_FLT x130 = x122 + (x26 * x124) + (-1 * x14 * x127);
	const GEN_FLT x131 = (x50 * x126) + (x49 * x128);
	const GEN_FLT x132 = (x43 * x130) + (-1 * x53 * (x131 + (x48 * x130)));
	const GEN_FLT x133 = x46 * x132;
	const GEN_FLT x134 = x56 * x133;
	const GEN_FLT x135 = (x59 * x132) + (x44 * (x134 + (-1 * x45 * x133)));
	const GEN_FLT x136 = (x61 * x133) + (x44 * x135);
	const GEN_FLT x137 = (x83 * x131) + (-1 * x72 * x130);
	const GEN_FLT x138 = (-1 * x85 * x137) + x129;
	const GEN_FLT x139 =
		(-1 * x96 *
		 ((x90 * x133) +
		  (-1 * x94 *
		   ((x93 * x138) +
			(x77 * ((x63 * x133) + (x44 * x136) + (x68 * x133) +
					(x44 * (x136 + (x44 * ((x66 * x133) + x135 + (x44 * ((x65 * x133) + (-1 * x91 * x133) + x134)))) +
							(x67 * x133))))))) +
		  (x89 * x138) + x137 + (x82 * x136))) +
		x129;
	out[0] = x97 + (x98 * x97);
	out[1] = x120 + (x98 * x120);
	out[2] = x139 + (x98 * x139);
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
	const GEN_FLT x0 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x1 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x2 = 2 * ((x1 * obj_qj) + (-1 * x0 * obj_qk));
	const GEN_FLT x3 = sensor_x + x2 + obj_px;
	const GEN_FLT x4 = x3 * lh_qw;
	const GEN_FLT x5 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x6 = 2 * ((x5 * obj_qk) + (-1 * x1 * obj_qi));
	const GEN_FLT x7 = sensor_y + x6 + obj_py;
	const GEN_FLT x8 = x7 * lh_qk;
	const GEN_FLT x9 = 2 * ((x0 * obj_qi) + (-1 * x5 * obj_qj));
	const GEN_FLT x10 = sensor_z + x9 + obj_pz;
	const GEN_FLT x11 = x10 * lh_qj;
	const GEN_FLT x12 = (-1 * x8) + x11 + x4;
	const GEN_FLT x13 = x7 * lh_qw;
	const GEN_FLT x14 = x10 * lh_qi;
	const GEN_FLT x15 = x3 * lh_qk;
	const GEN_FLT x16 = x15 + (-1 * x14) + x13;
	const GEN_FLT x17 = x10 + (2 * ((x16 * lh_qi) + (-1 * x12 * lh_qj))) + lh_pz;
	const GEN_FLT x18 = x10 * lh_qw;
	const GEN_FLT x19 = x3 * lh_qj;
	const GEN_FLT x20 = x7 * lh_qi;
	const GEN_FLT x21 = x20 + (-1 * x19) + x18;
	const GEN_FLT x22 = x3 + (2 * ((x21 * lh_qj) + (-1 * x16 * lh_qk))) + lh_px;
	const GEN_FLT x23 = pow(x22, 2);
	const GEN_FLT x24 = x23 + pow(x17, 2);
	const GEN_FLT x25 = pow(x24, -1);
	const GEN_FLT x26 = x25 * x17;
	const GEN_FLT x27 = x7 + (2 * ((x12 * lh_qk) + (-1 * x21 * lh_qi))) + lh_py;
	const GEN_FLT x28 = x22 * x27;
	const GEN_FLT x29 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x30 = tan(x29);
	const GEN_FLT x31 = x30 * pow(x24, -3.0 / 2.0);
	const GEN_FLT x32 = x31 * x28;
	const GEN_FLT x33 = cos(x29);
	const GEN_FLT x34 = pow(x33, -1);
	const GEN_FLT x35 = pow(x27, 2);
	const GEN_FLT x36 = x24 + x35;
	const GEN_FLT x37 = x34 * pow(x36, -1.0 / 2.0);
	const GEN_FLT x38 = asin(x37 * x27);
	const GEN_FLT x39 = 8.0108022e-06 * x38;
	const GEN_FLT x40 = pow((1 + (-1 * pow(x33, -2) * pow(x36, -1) * x35)), -1.0 / 2.0);
	const GEN_FLT x41 = x34 * pow(x36, -3.0 / 2.0);
	const GEN_FLT x42 = x41 * x28;
	const GEN_FLT x43 = x40 * x42;
	const GEN_FLT x44 = -8.0108022e-06 + (-1 * x39);
	const GEN_FLT x45 = x40 * x44;
	const GEN_FLT x46 = -1 * x42 * x45;
	const GEN_FLT x47 = 0.0028679863 + (x44 * x38);
	const GEN_FLT x48 = (-1 * x43 * x47) + (x38 * (x46 + (x43 * x39)));
	const GEN_FLT x49 = 5.3685255e-06 + (x47 * x38);
	const GEN_FLT x50 = (-1 * x43 * x49) + (x48 * x38);
	const GEN_FLT x51 = 0.0076069798 + (x49 * x38);
	const GEN_FLT x52 = x51 * x38;
	const GEN_FLT x53 = -8.0108022e-06 + (-1.60216044e-05 * x38);
	const GEN_FLT x54 = x47 + (x53 * x38);
	const GEN_FLT x55 = x49 + (x54 * x38);
	const GEN_FLT x56 = x51 + (x55 * x38);
	const GEN_FLT x57 = (x56 * x38) + x52;
	const GEN_FLT x58 = sin(x29);
	const GEN_FLT x59 = x30 * pow(x24, -1.0 / 2.0);
	const GEN_FLT x60 = -1 * x59 * x27;
	const GEN_FLT x61 = atan2(-1 * x17, x22);
	const GEN_FLT x62 = x61 + (-1 * asin(x60)) + ogeeMag_1;
	const GEN_FLT x63 = (sin(x62) * ogeePhase_1) + curve_1;
	const GEN_FLT x64 = x63 * x58;
	const GEN_FLT x65 = x33 + (x64 * x57);
	const GEN_FLT x66 = pow(x65, -1);
	const GEN_FLT x67 = pow(x38, 2);
	const GEN_FLT x68 = x63 * x67;
	const GEN_FLT x69 = x68 * x66;
	const GEN_FLT x70 = pow((1 + (-1 * pow(x30, 2) * x35 * x25)), -1.0 / 2.0);
	const GEN_FLT x71 = (-1 * x70 * x32) + x26;
	const GEN_FLT x72 = cos(x62) * ogeePhase_1;
	const GEN_FLT x73 = x67 * x66 * x51;
	const GEN_FLT x74 = x73 * x72;
	const GEN_FLT x75 = 2 * x22;
	const GEN_FLT x76 = x41 * x27;
	const GEN_FLT x77 = x63 * x66 * x52;
	const GEN_FLT x78 = x77 * x40;
	const GEN_FLT x79 = 2.40324066e-05 * x38;
	const GEN_FLT x80 = x51 * x40;
	const GEN_FLT x81 = x58 * x57;
	const GEN_FLT x82 = x81 * x72;
	const GEN_FLT x83 = x68 * pow(x65, -2) * x51;
	const GEN_FLT x84 = x60 + (x69 * x51);
	const GEN_FLT x85 = pow((1 + (-1 * pow(x84, 2))), -1.0 / 2.0);
	const GEN_FLT x86 =
		(-1 * x85 *
		 ((-1 * x83 *
		   ((x82 * x71) +
			(x64 * ((-1 * x80 * x42) + (x50 * x38) + (-1 * x56 * x43) +
					(x38 * (x50 + (x38 * (x48 + (-1 * x54 * x43) + (x38 * ((-1 * x53 * x43) + (x79 * x43) + x46)))) +
							(-1 * x55 * x43))))))) +
		  (x71 * x74) + (x69 * x50) + (-1 * x78 * x75 * x76) + x32)) +
		x26;
	const GEN_FLT x87 = cos((-1 * asin(x84)) + x61 + gibPhase_1) * gibMag_1;
	const GEN_FLT x88 = x37 + (-1 * x41 * x35);
	const GEN_FLT x89 = x88 * x40;
	const GEN_FLT x90 = x89 * x44;
	const GEN_FLT x91 = (x89 * x47) + (x38 * (x90 + (-1 * x89 * x39)));
	const GEN_FLT x92 = (x89 * x49) + (x91 * x38);
	const GEN_FLT x93 = 2 * x77;
	const GEN_FLT x94 = x70 * x59;
	const GEN_FLT x95 =
		x85 *
		((-1 * x83 *
		  ((x82 * x94) + (x64 * ((x80 * x88) + (x92 * x38) + (x89 * x56) +
								 (x38 * ((x38 * (x91 + (x89 * x54) + (x38 * ((x89 * x53) + (-1 * x89 * x79) + x90)))) +
										 x92 + (x89 * x55))))))) +
		 (-1 * x59) + (x74 * x94) + (x89 * x93) + (x69 * x92));
	const GEN_FLT x96 = -1 * x25 * x22;
	const GEN_FLT x97 = x31 * x27;
	const GEN_FLT x98 = x97 * x17;
	const GEN_FLT x99 = x76 * x17;
	const GEN_FLT x100 = x99 * x40;
	const GEN_FLT x101 = -1 * x99 * x45;
	const GEN_FLT x102 = (-1 * x47 * x100) + (x38 * (x101 + (x39 * x100)));
	const GEN_FLT x103 = (-1 * x49 * x100) + (x38 * x102);
	const GEN_FLT x104 = (-1 * x70 * x98) + x96;
	const GEN_FLT x105 = 2 * x27;
	const GEN_FLT x106 =
		(-1 * x85 *
		 ((-1 * x83 *
		   ((x82 * x104) +
			(x64 *
			 ((-1 * x80 * x99) + (x38 * x103) + (-1 * x56 * x100) +
			  (x38 * (x103 + (x38 * (x102 + (-1 * x54 * x100) + (x38 * ((-1 * x53 * x100) + (x79 * x100) + x101)))) +
					  (-1 * x55 * x100))))))) +
		  (-1 * x78 * x41 * x17 * x105) + (x74 * x104) + (x69 * x103) + x98)) +
		x96;
	const GEN_FLT x107 = pow(x22, -1);
	const GEN_FLT x108 = 2 * x19;
	const GEN_FLT x109 = (2 * x20) + (-1 * x108);
	const GEN_FLT x110 = 2 * x8;
	const GEN_FLT x111 = (2 * x11) + (-1 * x110);
	const GEN_FLT x112 = pow(x23, -1) * x17;
	const GEN_FLT x113 = x25 * x23;
	const GEN_FLT x114 = ((x111 * x112) + (-1 * x109 * x107)) * x113;
	const GEN_FLT x115 = 2 * x14;
	const GEN_FLT x116 = (2 * x15) + (-1 * x115);
	const GEN_FLT x117 = 2 * x17;
	const GEN_FLT x118 = (x109 * x117) + (x75 * x111);
	const GEN_FLT x119 = 1.0 / 2.0 * x76;
	const GEN_FLT x120 = (x37 * x116) + (-1 * x119 * (x118 + (x105 * x116)));
	const GEN_FLT x121 = x40 * x120;
	const GEN_FLT x122 = x44 * x121;
	const GEN_FLT x123 = (x47 * x121) + (x38 * (x122 + (-1 * x39 * x121)));
	const GEN_FLT x124 = (x49 * x121) + (x38 * x123);
	const GEN_FLT x125 = 1.0 / 2.0 * x97;
	const GEN_FLT x126 = (x118 * x125) + (-1 * x59 * x116);
	const GEN_FLT x127 = x72 * ((-1 * x70 * x126) + x114);
	const GEN_FLT x128 =
		(-1 * x85 *
		 (x126 +
		  (-1 * x83 *
		   ((x81 * x127) +
			(x64 * ((x80 * x120) + (x38 * x124) + (x56 * x121) +
					(x38 * (x124 + (x38 * (x123 + (x54 * x121) + (x38 * ((x53 * x121) + (-1 * x79 * x121) + x122)))) +
							(x55 * x121))))))) +
		  (x93 * x121) + (x73 * x127) + (x69 * x124))) +
		x114;
	const GEN_FLT x129 = 2 * ((-1 * sensor_z) + (-1 * x9) + (-1 * obj_pz));
	const GEN_FLT x130 = 2 * x13;
	const GEN_FLT x131 = x116 + x130 + (x129 * lh_qi);
	const GEN_FLT x132 = 2 * lh_qj;
	const GEN_FLT x133 = (x7 * x132) + (-1 * x129 * lh_qk);
	const GEN_FLT x134 = ((x112 * x133) + (-1 * x107 * x131)) * x113;
	const GEN_FLT x135 = 2 * x18;
	const GEN_FLT x136 = (-1 * x135) + x108 + (-4 * x20);
	const GEN_FLT x137 = (x117 * x131) + (x75 * x133);
	const GEN_FLT x138 = (x37 * x136) + (-1 * x119 * (x137 + (x105 * x136)));
	const GEN_FLT x139 = x40 * x138;
	const GEN_FLT x140 = x44 * x139;
	const GEN_FLT x141 = (x47 * x139) + (x38 * (x140 + (-1 * x39 * x139)));
	const GEN_FLT x142 = (x49 * x139) + (x38 * x141);
	const GEN_FLT x143 = (x125 * x137) + (-1 * x59 * x136);
	const GEN_FLT x144 = (-1 * x70 * x143) + x134;
	const GEN_FLT x145 =
		(-1 * x85 *
		 (x143 + (x74 * x144) +
		  (-1 * x83 *
		   ((x82 * x144) +
			(x64 * ((x80 * x138) + (x38 * x142) + (x56 * x139) +
					(x38 * ((x38 * (x141 + (x54 * x139) + (x38 * ((x53 * x139) + (-1 * x79 * x139) + x140)))) + x142 +
							(x55 * x139))))))) +
		  (x93 * x139) + (x69 * x142))) +
		x134;
	const GEN_FLT x146 = 2 * x4;
	const GEN_FLT x147 = (-1 * x146) + x110 + (-4 * x11);
	const GEN_FLT x148 = (-1 * sensor_x) + (-1 * x2) + (-1 * obj_px);
	const GEN_FLT x149 = x109 + x135 + (x132 * x148);
	const GEN_FLT x150 = ((x112 * x149) + (-1 * x107 * x147)) * x113;
	const GEN_FLT x151 = 2 * lh_qi;
	const GEN_FLT x152 = 2 * lh_qk;
	const GEN_FLT x153 = (x10 * x152) + (-1 * x148 * x151);
	const GEN_FLT x154 = (x117 * x147) + (x75 * x149);
	const GEN_FLT x155 = (x37 * x153) + (-1 * x119 * (x154 + (x105 * x153)));
	const GEN_FLT x156 = x40 * x155;
	const GEN_FLT x157 = x44 * x156;
	const GEN_FLT x158 = (x47 * x156) + (x38 * (x157 + (-1 * x39 * x156)));
	const GEN_FLT x159 = (x49 * x156) + (x38 * x158);
	const GEN_FLT x160 = (x125 * x154) + (-1 * x59 * x153);
	const GEN_FLT x161 = (-1 * x70 * x160) + x150;
	const GEN_FLT x162 =
		(-1 * x85 *
		 (x160 +
		  (-1 * x83 *
		   ((x82 * x161) +
			(x64 * ((x80 * x155) + (x38 * x159) + (x56 * x156) +
					(x38 * (x159 + (x38 * (x158 + (x54 * x156) + (x38 * ((x53 * x156) + (-1 * x79 * x156) + x157)))) +
							(x55 * x156))))))) +
		  (x93 * x156) + (x74 * x161) + (x69 * x159))) +
		x150;
	const GEN_FLT x163 = (-1 * sensor_y) + (-1 * x6) + (-1 * obj_py);
	const GEN_FLT x164 = (x3 * x151) + (-1 * x163 * x132);
	const GEN_FLT x165 = (-1 * x130) + x115 + (-4 * x15);
	const GEN_FLT x166 = ((x112 * x165) + (-1 * x107 * x164)) * x113;
	const GEN_FLT x167 = x111 + x146 + (x163 * x152);
	const GEN_FLT x168 = (x117 * x164) + (x75 * x165);
	const GEN_FLT x169 = (x37 * x167) + (-1 * x119 * (x168 + (x105 * x167)));
	const GEN_FLT x170 = x40 * x169;
	const GEN_FLT x171 = x44 * x170;
	const GEN_FLT x172 = (x47 * x170) + (x38 * (x171 + (-1 * x39 * x170)));
	const GEN_FLT x173 = (x49 * x170) + (x38 * x172);
	const GEN_FLT x174 = (x125 * x168) + (-1 * x59 * x167);
	const GEN_FLT x175 = (-1 * x70 * x174) + x166;
	const GEN_FLT x176 =
		(-1 * x85 *
		 ((-1 * x83 *
		   ((x82 * x175) +
			(x64 * ((x80 * x169) + (x38 * x173) + (x56 * x170) +
					(x38 * (x173 + (x38 * (x172 + (x54 * x170) + (x38 * ((x53 * x170) + (-1 * x79 * x170) + x171)))) +
							(x55 * x170))))))) +
		  (x93 * x170) + (x74 * x175) + x174 + (x69 * x173))) +
		x166;
	out[0] = x86 + (x86 * x87);
	out[1] = (-1 * x95) + (-1 * x87 * x95);
	out[2] = x106 + (x87 * x106);
	out[3] = x128 + (x87 * x128);
	out[4] = x145 + (x87 * x145);
	out[5] = x162 + (x87 * x162);
	out[6] = x176 + (x87 * x176);
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
	const GEN_FLT x0 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x1 = tan(x0);
	const GEN_FLT x2 = pow(x1, 2);
	const GEN_FLT x3 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x4 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x5 = sensor_z + (2 * ((x4 * obj_qi) + (-1 * x3 * obj_qj))) + obj_pz;
	const GEN_FLT x6 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x7 = sensor_x + (2 * ((x6 * obj_qj) + (-1 * x4 * obj_qk))) + obj_px;
	const GEN_FLT x8 = sensor_y + (2 * ((x3 * obj_qk) + (-1 * x6 * obj_qi))) + obj_py;
	const GEN_FLT x9 = (x8 * lh_qi) + (-1 * x7 * lh_qj) + (x5 * lh_qw);
	const GEN_FLT x10 = (x5 * lh_qj) + (-1 * x8 * lh_qk) + (x7 * lh_qw);
	const GEN_FLT x11 = x8 + (2 * ((x10 * lh_qk) + (-1 * x9 * lh_qi))) + lh_py;
	const GEN_FLT x12 = (x7 * lh_qk) + (-1 * x5 * lh_qi) + (x8 * lh_qw);
	const GEN_FLT x13 = x5 + (2 * ((x12 * lh_qi) + (-1 * x10 * lh_qj))) + lh_pz;
	const GEN_FLT x14 = x7 + (2 * ((x9 * lh_qj) + (-1 * x12 * lh_qk))) + lh_px;
	const GEN_FLT x15 = pow(x14, 2) + pow(x13, 2);
	const GEN_FLT x16 = pow(x15, -1.0 / 2.0) * x11;
	const GEN_FLT x17 = (1 + x2) * x16;
	const GEN_FLT x18 = cos(x0);
	const GEN_FLT x19 = pow(x11, 2);
	const GEN_FLT x20 = x15 + x19;
	const GEN_FLT x21 = pow(x20, -1.0 / 2.0) * x11;
	const GEN_FLT x22 = asin(x21 * pow(x18, -1));
	const GEN_FLT x23 = 8.0108022e-06 * x22;
	const GEN_FLT x24 = sin(x0);
	const GEN_FLT x25 = pow(x18, -2);
	const GEN_FLT x26 = x25 * x21 * pow((1 + (-1 * x25 * pow(x20, -1) * x19)), -1.0 / 2.0);
	const GEN_FLT x27 = x24 * x26;
	const GEN_FLT x28 = -8.0108022e-06 + (-1 * x23);
	const GEN_FLT x29 = -1 * x28 * x27;
	const GEN_FLT x30 = 0.0028679863 + (x22 * x28);
	const GEN_FLT x31 = (-1 * x30 * x27) + (x22 * (x29 + (x23 * x27)));
	const GEN_FLT x32 = 5.3685255e-06 + (x30 * x22);
	const GEN_FLT x33 = (-1 * x32 * x27) + (x31 * x22);
	const GEN_FLT x34 = -1 * x1 * x16;
	const GEN_FLT x35 = atan2(-1 * x13, x14);
	const GEN_FLT x36 = x35 + (-1 * asin(x34)) + ogeeMag_1;
	const GEN_FLT x37 = sin(x36);
	const GEN_FLT x38 = (x37 * ogeePhase_1) + curve_1;
	const GEN_FLT x39 = pow(x22, 2);
	const GEN_FLT x40 = 0.0076069798 + (x32 * x22);
	const GEN_FLT x41 = x40 * x22;
	const GEN_FLT x42 = -8.0108022e-06 + (-1.60216044e-05 * x22);
	const GEN_FLT x43 = x30 + (x42 * x22);
	const GEN_FLT x44 = x32 + (x43 * x22);
	const GEN_FLT x45 = x40 + (x44 * x22);
	const GEN_FLT x46 = (x45 * x22) + x41;
	const GEN_FLT x47 = x38 * x24;
	const GEN_FLT x48 = x46 * x47;
	const GEN_FLT x49 = x18 + x48;
	const GEN_FLT x50 = pow(x49, -1);
	const GEN_FLT x51 = x50 * x39;
	const GEN_FLT x52 = x51 * x40;
	const GEN_FLT x53 = cos(x36) * ogeePhase_1;
	const GEN_FLT x54 = x53 * x52;
	const GEN_FLT x55 = x17 * pow((1 + (-1 * x2 * pow(x15, -1) * x19)), -1.0 / 2.0);
	const GEN_FLT x56 = x40 * pow(x49, -2) * x39;
	const GEN_FLT x57 = x34 + (x52 * x38);
	const GEN_FLT x58 = pow((1 + (-1 * pow(x57, 2))), -1.0 / 2.0);
	const GEN_FLT x59 =
		x58 * ((-1 * x56 * x38 *
				((-1 * x53 * x55 * x46 * x24) + (-1 * x46 * x38 * x18) +
				 (x47 * ((-1 * x40 * x27) + (x33 * x22) + (-1 * x45 * x27) +
						 (x22 * (x33 +
								 (x22 * (x31 + (-1 * x43 * x27) +
										 (x22 * ((-1 * x42 * x27) + (2.40324066e-05 * x22 * x27) + x29)))) +
								 (-1 * x44 * x27))))) +
				 x24)) +
			   (-2 * x50 * x41 * x47 * x26) + (-1 * x54 * x55) + (x51 * x33 * x38) + x17);
	const GEN_FLT x60 = x35 + (-1 * asin(x57)) + gibPhase_1;
	const GEN_FLT x61 = cos(x60) * gibMag_1;
	const GEN_FLT x62 = x56 * x48;
	const GEN_FLT x63 = ((-1 * x62) + x52) * x58;
	const GEN_FLT x64 = x58 * ((-1 * x62 * x53) + x54);
	const GEN_FLT x65 = ((-1 * x62 * x37) + (x52 * x37)) * x58;
	out[0] = -1;
	out[1] = (-1 * x59) + (-1 * x61 * x59);
	out[2] = (-1 * x63) + (-1 * x63 * x61);
	out[3] = x61;
	out[4] = sin(x60);
	out[5] = (-1 * x64) + (-1 * x64 * x61);
	out[6] = (-1 * x65) + (-1 * x61 * x65);
}

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
	const GEN_FLT x0 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x1 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x2 = sensor_z + (2 * ((x1 * obj_qi) + (-1 * x0 * obj_qj))) + obj_pz;
	const GEN_FLT x3 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x4 = sensor_x + (2 * ((x3 * obj_qj) + (-1 * x1 * obj_qk))) + obj_px;
	const GEN_FLT x5 = sensor_y + (2 * ((x0 * obj_qk) + (-1 * x3 * obj_qi))) + obj_py;
	const GEN_FLT x6 = (x5 * lh_qi) + (-1 * x4 * lh_qj) + (x2 * lh_qw);
	const GEN_FLT x7 = (-1 * x5 * lh_qk) + (x2 * lh_qj) + (x4 * lh_qw);
	const GEN_FLT x8 = x5 + (2 * ((x7 * lh_qk) + (-1 * x6 * lh_qi))) + lh_py;
	const GEN_FLT x9 = (x4 * lh_qk) + (-1 * x2 * lh_qi) + (x5 * lh_qw);
	const GEN_FLT x10 = x2 + (2 * ((x9 * lh_qi) + (-1 * x7 * lh_qj))) + lh_pz;
	const GEN_FLT x11 = -1 * x10;
	const GEN_FLT x12 = pow(x10, 2);
	const GEN_FLT x13 = x4 + (2 * ((x6 * lh_qj) + (-1 * x9 * lh_qk))) + lh_px;
	const GEN_FLT x14 = atan2(x13, x11);
	const GEN_FLT x15 = (-1 * x14) + (-1 * phase_0) + (-1 * asin(x8 * pow((pow(x13, 2) + x12), -1.0 / 2.0) * tilt_0));
	const GEN_FLT x16 =
		(-1 * atan2(-1 * x8, x11)) + (-1 * asin(x13 * pow((pow(x8, 2) + x12), -1.0 / 2.0) * tilt_1)) + (-1 * phase_1);
	out[0] = x15 + (-1 * cos(1.5707963267949 + x15 + gibPhase_0) * gibMag_0) + (pow(atan2(x8, x11), 2) * curve_0);
	out[1] = x16 + (pow(x14, 2) * curve_1) + (-1 * cos(1.5707963267949 + x16 + gibPhase_1) * gibMag_1);
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
	const GEN_FLT x0 = 2 * lh_qj;
	const GEN_FLT x1 = x0 * lh_qw;
	const GEN_FLT x2 = 2 * lh_qi;
	const GEN_FLT x3 = x2 * lh_qk;
	const GEN_FLT x4 = x3 + (-1 * x1);
	const GEN_FLT x5 = obj_qj * sensor_x;
	const GEN_FLT x6 = obj_qw * sensor_z;
	const GEN_FLT x7 = obj_qi * sensor_y;
	const GEN_FLT x8 = x7 + x6 + (-1 * x5);
	const GEN_FLT x9 = obj_qw * sensor_x;
	const GEN_FLT x10 = obj_qk * sensor_y;
	const GEN_FLT x11 = obj_qj * sensor_z;
	const GEN_FLT x12 = x11 + (-1 * x10) + x9;
	const GEN_FLT x13 = sensor_y + (2 * ((x12 * obj_qk) + (-1 * x8 * obj_qi))) + obj_py;
	const GEN_FLT x14 = obj_qw * sensor_y;
	const GEN_FLT x15 = obj_qi * sensor_z;
	const GEN_FLT x16 = obj_qk * sensor_x;
	const GEN_FLT x17 = x16 + (-1 * x15) + x14;
	const GEN_FLT x18 = sensor_z + (2 * ((x17 * obj_qi) + (-1 * x12 * obj_qj))) + obj_pz;
	const GEN_FLT x19 = sensor_x + (2 * ((x8 * obj_qj) + (-1 * x17 * obj_qk))) + obj_px;
	const GEN_FLT x20 = (x19 * lh_qk) + (-1 * x18 * lh_qi) + (x13 * lh_qw);
	const GEN_FLT x21 = (x13 * lh_qi) + (-1 * x19 * lh_qj) + (x18 * lh_qw);
	const GEN_FLT x22 = x19 + (2 * ((x21 * lh_qj) + (-1 * x20 * lh_qk))) + lh_px;
	const GEN_FLT x23 = (x18 * lh_qj) + (-1 * x13 * lh_qk) + (x19 * lh_qw);
	const GEN_FLT x24 = x18 + (2 * ((x20 * lh_qi) + (-1 * x23 * lh_qj))) + lh_pz;
	const GEN_FLT x25 = pow(x24, 2);
	const GEN_FLT x26 = pow(x25, -1);
	const GEN_FLT x27 = x22 * x26;
	const GEN_FLT x28 = -2 * pow(lh_qk, 2);
	const GEN_FLT x29 = -2 * pow(lh_qj, 2);
	const GEN_FLT x30 = 1 + x29 + x28;
	const GEN_FLT x31 = pow(x24, -1);
	const GEN_FLT x32 = pow(x22, 2);
	const GEN_FLT x33 = x25 + x32;
	const GEN_FLT x34 = pow(x33, -1);
	const GEN_FLT x35 = x34 * x25;
	const GEN_FLT x36 = x35 * ((-1 * x30 * x31) + (x4 * x27));
	const GEN_FLT x37 = x0 * lh_qi;
	const GEN_FLT x38 = 2 * lh_qw;
	const GEN_FLT x39 = x38 * lh_qk;
	const GEN_FLT x40 = x39 + x37;
	const GEN_FLT x41 = pow(x33, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x42 = 2 * x22;
	const GEN_FLT x43 = 2 * x24;
	const GEN_FLT x44 = x4 * x43;
	const GEN_FLT x45 = x13 + (2 * ((x23 * lh_qk) + (-1 * x21 * lh_qi))) + lh_py;
	const GEN_FLT x46 = 1.0 / 2.0 * x45 * pow(x33, -3.0 / 2.0) * tilt_0;
	const GEN_FLT x47 = pow(x45, 2);
	const GEN_FLT x48 = pow((1 + (-1 * x47 * x34 * pow(tilt_0, 2))), -1.0 / 2.0);
	const GEN_FLT x49 = (-1 * x48 * ((-1 * x46 * (x44 + (x42 * x30))) + (x40 * x41))) + (-1 * x36);
	const GEN_FLT x50 = -1 * x24;
	const GEN_FLT x51 = atan2(x22, x50);
	const GEN_FLT x52 =
		sin(1.5707963267949 + (-1 * x51) + (-1 * phase_0) + (-1 * asin(x41 * x45)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x53 = x45 * x26;
	const GEN_FLT x54 = x4 * x53;
	const GEN_FLT x55 = x40 * x31;
	const GEN_FLT x56 = x25 + x47;
	const GEN_FLT x57 = pow(x56, -1);
	const GEN_FLT x58 = x57 * x25;
	const GEN_FLT x59 = 2 * x58 * atan2(x45, x50) * curve_0;
	const GEN_FLT x60 = x0 * lh_qk;
	const GEN_FLT x61 = x38 * lh_qi;
	const GEN_FLT x62 = x61 + x60;
	const GEN_FLT x63 = x37 + (-1 * x39);
	const GEN_FLT x64 = ((-1 * x63 * x31) + (x62 * x27)) * x35;
	const GEN_FLT x65 = 1 + (-2 * pow(lh_qi, 2));
	const GEN_FLT x66 = x65 + x28;
	const GEN_FLT x67 = x62 * x43;
	const GEN_FLT x68 = (-1 * x48 * ((-1 * x46 * (x67 + (x63 * x42))) + (x66 * x41))) + (-1 * x64);
	const GEN_FLT x69 = x62 * x53;
	const GEN_FLT x70 = x66 * x31;
	const GEN_FLT x71 = x65 + x29;
	const GEN_FLT x72 = x1 + x3;
	const GEN_FLT x73 = ((-1 * x72 * x31) + (x71 * x27)) * x35;
	const GEN_FLT x74 = x60 + (-1 * x61);
	const GEN_FLT x75 = x71 * x43;
	const GEN_FLT x76 = (-1 * x48 * ((-1 * x46 * (x75 + (x72 * x42))) + (x74 * x41))) + (-1 * x73);
	const GEN_FLT x77 = x71 * x53;
	const GEN_FLT x78 = x74 * x31;
	const GEN_FLT x79 = 2 * x5;
	const GEN_FLT x80 = 2 * x7;
	const GEN_FLT x81 = x80 + (-1 * x79);
	const GEN_FLT x82 = 2 * x15;
	const GEN_FLT x83 = 2 * x16;
	const GEN_FLT x84 = x83 + (-1 * x82);
	const GEN_FLT x85 = 2 * x10;
	const GEN_FLT x86 = 2 * x11;
	const GEN_FLT x87 = x86 + (-1 * x85);
	const GEN_FLT x88 = (x87 * lh_qw) + (-1 * x84 * lh_qk) + (x81 * lh_qj);
	const GEN_FLT x89 = (x84 * lh_qw) + (-1 * x81 * lh_qi) + (x87 * lh_qk);
	const GEN_FLT x90 = x81 + (x2 * x89) + (-1 * x0 * x88);
	const GEN_FLT x91 = 2 * lh_qk;
	const GEN_FLT x92 = (x81 * lh_qw) + (-1 * x87 * lh_qj) + (x84 * lh_qi);
	const GEN_FLT x93 = x87 + (x0 * x92) + (-1 * x89 * x91);
	const GEN_FLT x94 = ((-1 * x93 * x31) + (x90 * x27)) * x35;
	const GEN_FLT x95 = x84 + (x88 * x91) + (-1 * x2 * x92);
	const GEN_FLT x96 = x90 * x43;
	const GEN_FLT x97 = (-1 * x48 * ((-1 * x46 * (x96 + (x93 * x42))) + (x95 * x41))) + (-1 * x94);
	const GEN_FLT x98 = x53 * x90;
	const GEN_FLT x99 = x95 * x31;
	const GEN_FLT x100 = 2 * x14;
	const GEN_FLT x101 = (-4 * x15) + x100 + x83;
	const GEN_FLT x102 = 2 * x6;
	const GEN_FLT x103 = x79 + (-1 * x102) + (-4 * x7);
	const GEN_FLT x104 = 2 * obj_qk * sensor_z;
	const GEN_FLT x105 = 2 * obj_qj * sensor_y;
	const GEN_FLT x106 = x105 + x104;
	const GEN_FLT x107 = (x106 * lh_qw) + (-1 * x103 * lh_qk) + (x101 * lh_qj);
	const GEN_FLT x108 = (x103 * lh_qw) + (-1 * x101 * lh_qi) + (x106 * lh_qk);
	const GEN_FLT x109 = x101 + (x2 * x108) + (-1 * x0 * x107);
	const GEN_FLT x110 = x26 * x109;
	const GEN_FLT x111 = (x101 * lh_qw) + (-1 * x106 * lh_qj) + (x103 * lh_qi);
	const GEN_FLT x112 = x106 + (x0 * x111) + (-1 * x91 * x108);
	const GEN_FLT x113 = ((-1 * x31 * x112) + (x22 * x110)) * x35;
	const GEN_FLT x114 = x103 + (x91 * x107) + (-1 * x2 * x111);
	const GEN_FLT x115 = x43 * x109;
	const GEN_FLT x116 = (-1 * x48 * ((-1 * x46 * (x115 + (x42 * x112))) + (x41 * x114))) + (-1 * x113);
	const GEN_FLT x117 = x45 * x110;
	const GEN_FLT x118 = x31 * x114;
	const GEN_FLT x119 = 2 * x9;
	const GEN_FLT x120 = (-1 * x119) + x85 + (-4 * x11);
	const GEN_FLT x121 = 2 * obj_qi * sensor_x;
	const GEN_FLT x122 = x104 + x121;
	const GEN_FLT x123 = (-4 * x5) + x102 + x80;
	const GEN_FLT x124 = (x123 * lh_qw) + (-1 * x122 * lh_qk) + (x120 * lh_qj);
	const GEN_FLT x125 = (-1 * x120 * lh_qi) + (x122 * lh_qw) + (x123 * lh_qk);
	const GEN_FLT x126 = x120 + (x2 * x125) + (-1 * x0 * x124);
	const GEN_FLT x127 = (-1 * x123 * lh_qj) + (x120 * lh_qw) + (x122 * lh_qi);
	const GEN_FLT x128 = x123 + (x0 * x127) + (-1 * x91 * x125);
	const GEN_FLT x129 = ((-1 * x31 * x128) + (x27 * x126)) * x35;
	const GEN_FLT x130 = x122 + (x91 * x124) + (-1 * x2 * x127);
	const GEN_FLT x131 = x43 * x126;
	const GEN_FLT x132 = (-1 * x48 * ((-1 * x46 * (x131 + (x42 * x128))) + (x41 * x130))) + (-1 * x129);
	const GEN_FLT x133 = x53 * x126;
	const GEN_FLT x134 = x31 * x130;
	const GEN_FLT x135 = x121 + x105;
	const GEN_FLT x136 = x119 + (-4 * x10) + x86;
	const GEN_FLT x137 = (-1 * x100) + x82 + (-4 * x16);
	const GEN_FLT x138 = (-1 * x136 * lh_qk) + (x137 * lh_qw) + (x135 * lh_qj);
	const GEN_FLT x139 = (x136 * lh_qw) + (-1 * x135 * lh_qi) + (x137 * lh_qk);
	const GEN_FLT x140 = x135 + (x2 * x139) + (-1 * x0 * x138);
	const GEN_FLT x141 = (x135 * lh_qw) + (-1 * x137 * lh_qj) + (x136 * lh_qi);
	const GEN_FLT x142 = x137 + (x0 * x141) + (-1 * x91 * x139);
	const GEN_FLT x143 = ((-1 * x31 * x142) + (x27 * x140)) * x35;
	const GEN_FLT x144 = x136 + (x91 * x138) + (-1 * x2 * x141);
	const GEN_FLT x145 = x43 * x140;
	const GEN_FLT x146 = (-1 * x48 * ((-1 * x46 * (x145 + (x42 * x142))) + (x41 * x144))) + (-1 * x143);
	const GEN_FLT x147 = x53 * x140;
	const GEN_FLT x148 = x31 * x144;
	const GEN_FLT x149 = 2 * x51 * curve_1;
	const GEN_FLT x150 = pow((1 + (-1 * x57 * x32 * pow(tilt_1, 2))), -1.0 / 2.0);
	const GEN_FLT x151 = pow(x56, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x152 = 2 * x45;
	const GEN_FLT x153 = 1.0 / 2.0 * pow(x56, -3.0 / 2.0) * x22 * tilt_1;
	const GEN_FLT x154 =
		(-1 * x150 * ((-1 * x153 * (x44 + (x40 * x152))) + (x30 * x151))) + (-1 * (x55 + (-1 * x54)) * x58);
	const GEN_FLT x155 =
		sin(1.5707963267949 + (-1 * asin(x22 * x151)) + (-1 * atan2(-1 * x45, x50)) + (-1 * phase_1) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x156 =
		(-1 * x150 * ((-1 * x153 * (x67 + (x66 * x152))) + (x63 * x151))) + (-1 * (x70 + (-1 * x69)) * x58);
	const GEN_FLT x157 =
		(-1 * x150 * ((-1 * x153 * (x75 + (x74 * x152))) + (x72 * x151))) + (-1 * (x78 + (-1 * x77)) * x58);
	const GEN_FLT x158 =
		(-1 * x150 * ((-1 * x153 * (x96 + (x95 * x152))) + (x93 * x151))) + (-1 * (x99 + (-1 * x98)) * x58);
	const GEN_FLT x159 =
		(-1 * x150 * ((-1 * x153 * (x115 + (x114 * x152))) + (x112 * x151))) + (-1 * (x118 + (-1 * x117)) * x58);
	const GEN_FLT x160 =
		(-1 * x150 * ((-1 * x153 * (x131 + (x130 * x152))) + (x128 * x151))) + (-1 * (x134 + (-1 * x133)) * x58);
	const GEN_FLT x161 =
		(-1 * x150 * ((-1 * x153 * (x145 + (x144 * x152))) + (x142 * x151))) + (-1 * (x148 + (-1 * x147)) * x58);
	out[0] = x49 + (((-1 * x55) + x54) * x59) + (x52 * x49);
	out[1] = x68 + (((-1 * x70) + x69) * x59) + (x68 * x52);
	out[2] = (((-1 * x78) + x77) * x59) + x76 + (x76 * x52);
	out[3] = x97 + (((-1 * x99) + x98) * x59) + (x52 * x97);
	out[4] = x116 + (((-1 * x118) + x117) * x59) + (x52 * x116);
	out[5] = x132 + (((-1 * x134) + x133) * x59) + (x52 * x132);
	out[6] = x146 + (((-1 * x148) + x147) * x59) + (x52 * x146);
	out[7] = x154 + (x154 * x155) + (x36 * x149);
	out[8] = x156 + (x155 * x156) + (x64 * x149);
	out[9] = x157 + (x155 * x157) + (x73 * x149);
	out[10] = x158 + (x155 * x158) + (x94 * x149);
	out[11] = x159 + (x155 * x159) + (x113 * x149);
	out[12] = (x160 * x155) + x160 + (x129 * x149);
	out[13] = x161 + (x161 * x155) + (x143 * x149);
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
	const GEN_FLT x0 = 2 * obj_qj;
	const GEN_FLT x1 = x0 * obj_qw;
	const GEN_FLT x2 = 2 * obj_qk;
	const GEN_FLT x3 = x2 * obj_qi;
	const GEN_FLT x4 = x3 + (-1 * x1);
	const GEN_FLT x5 = x0 * obj_qi;
	const GEN_FLT x6 = x2 * obj_qw;
	const GEN_FLT x7 = x6 + x5;
	const GEN_FLT x8 = -2 * pow(obj_qk, 2);
	const GEN_FLT x9 = -2 * pow(obj_qj, 2);
	const GEN_FLT x10 = 1 + x9 + x8;
	const GEN_FLT x11 = (x10 * lh_qw) + (-1 * x7 * lh_qk) + (x4 * lh_qj);
	const GEN_FLT x12 = 2 * lh_qj;
	const GEN_FLT x13 = (x7 * lh_qw) + (-1 * x4 * lh_qi) + (x10 * lh_qk);
	const GEN_FLT x14 = 2 * lh_qi;
	const GEN_FLT x15 = x4 + (x14 * x13) + (-1 * x12 * x11);
	const GEN_FLT x16 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x17 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x18 = sensor_x + (2 * ((x17 * obj_qj) + (-1 * x16 * obj_qk))) + obj_px;
	const GEN_FLT x19 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x20 = sensor_y + (2 * ((x19 * obj_qk) + (-1 * x17 * obj_qi))) + obj_py;
	const GEN_FLT x21 = sensor_z + (2 * ((x16 * obj_qi) + (-1 * x19 * obj_qj))) + obj_pz;
	const GEN_FLT x22 = (-1 * x20 * lh_qk) + (x21 * lh_qj) + (x18 * lh_qw);
	const GEN_FLT x23 = (x18 * lh_qk) + (-1 * x21 * lh_qi) + (x20 * lh_qw);
	const GEN_FLT x24 = x21 + (2 * ((x23 * lh_qi) + (-1 * x22 * lh_qj))) + lh_pz;
	const GEN_FLT x25 = pow(x24, 2);
	const GEN_FLT x26 = pow(x25, -1);
	const GEN_FLT x27 = (-1 * x18 * lh_qj) + (x20 * lh_qi) + (x21 * lh_qw);
	const GEN_FLT x28 = x18 + (2 * ((x27 * lh_qj) + (-1 * x23 * lh_qk))) + lh_px;
	const GEN_FLT x29 = x28 * x26;
	const GEN_FLT x30 = 2 * lh_qk;
	const GEN_FLT x31 = (x4 * lh_qw) + (-1 * x10 * lh_qj) + (x7 * lh_qi);
	const GEN_FLT x32 = x10 + (x31 * x12) + (-1 * x30 * x13);
	const GEN_FLT x33 = pow(x24, -1);
	const GEN_FLT x34 = pow(x28, 2);
	const GEN_FLT x35 = x25 + x34;
	const GEN_FLT x36 = pow(x35, -1);
	const GEN_FLT x37 = x36 * x25;
	const GEN_FLT x38 = ((-1 * x32 * x33) + (x29 * x15)) * x37;
	const GEN_FLT x39 = x20 + (2 * ((x22 * lh_qk) + (-1 * x27 * lh_qi))) + lh_py;
	const GEN_FLT x40 = pow(x39, 2);
	const GEN_FLT x41 = pow((1 + (-1 * x40 * x36 * pow(tilt_0, 2))), -1.0 / 2.0);
	const GEN_FLT x42 = x7 + (x30 * x11) + (-1 * x31 * x14);
	const GEN_FLT x43 = pow(x35, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x44 = 2 * x28;
	const GEN_FLT x45 = 2 * x24;
	const GEN_FLT x46 = x45 * x15;
	const GEN_FLT x47 = 1.0 / 2.0 * pow(x35, -3.0 / 2.0) * x39 * tilt_0;
	const GEN_FLT x48 = (-1 * x41 * ((-1 * x47 * (x46 + (x44 * x32))) + (x42 * x43))) + (-1 * x38);
	const GEN_FLT x49 = -1 * x24;
	const GEN_FLT x50 = atan2(x28, x49);
	const GEN_FLT x51 =
		sin(1.5707963267949 + (-1 * x50) + (-1 * phase_0) + (-1 * asin(x43 * x39)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x52 = x39 * x26;
	const GEN_FLT x53 = x52 * x15;
	const GEN_FLT x54 = x42 * x33;
	const GEN_FLT x55 = x25 + x40;
	const GEN_FLT x56 = pow(x55, -1);
	const GEN_FLT x57 = x56 * x25;
	const GEN_FLT x58 = 2 * x57 * atan2(x39, x49) * curve_0;
	const GEN_FLT x59 = x2 * obj_qj;
	const GEN_FLT x60 = 2 * obj_qw * obj_qi;
	const GEN_FLT x61 = x60 + x59;
	const GEN_FLT x62 = 1 + (-2 * pow(obj_qi, 2));
	const GEN_FLT x63 = x62 + x8;
	const GEN_FLT x64 = x5 + (-1 * x6);
	const GEN_FLT x65 = (x64 * lh_qw) + (-1 * x63 * lh_qk) + (x61 * lh_qj);
	const GEN_FLT x66 = (x63 * lh_qw) + (-1 * x61 * lh_qi) + (x64 * lh_qk);
	const GEN_FLT x67 = x61 + (x66 * x14) + (-1 * x65 * x12);
	const GEN_FLT x68 = (x61 * lh_qw) + (-1 * x64 * lh_qj) + (x63 * lh_qi);
	const GEN_FLT x69 = x64 + (x68 * x12) + (-1 * x66 * x30);
	const GEN_FLT x70 = ((-1 * x69 * x33) + (x67 * x29)) * x37;
	const GEN_FLT x71 = (x65 * x30) + x63 + (-1 * x68 * x14);
	const GEN_FLT x72 = x67 * x45;
	const GEN_FLT x73 = (-1 * x41 * ((-1 * x47 * (x72 + (x69 * x44))) + (x71 * x43))) + (-1 * x70);
	const GEN_FLT x74 = x67 * x52;
	const GEN_FLT x75 = x71 * x33;
	const GEN_FLT x76 = x62 + x9;
	const GEN_FLT x77 = x59 + (-1 * x60);
	const GEN_FLT x78 = x1 + x3;
	const GEN_FLT x79 = (x78 * lh_qw) + (-1 * x77 * lh_qk) + (x76 * lh_qj);
	const GEN_FLT x80 = (x77 * lh_qw) + (-1 * x76 * lh_qi) + (x78 * lh_qk);
	const GEN_FLT x81 = x76 + (x80 * x14) + (-1 * x79 * x12);
	const GEN_FLT x82 = (x76 * lh_qw) + (-1 * x78 * lh_qj) + (x77 * lh_qi);
	const GEN_FLT x83 = x78 + (x82 * x12) + (-1 * x80 * x30);
	const GEN_FLT x84 = ((-1 * x83 * x33) + (x81 * x29)) * x37;
	const GEN_FLT x85 = x77 + (x79 * x30) + (-1 * x82 * x14);
	const GEN_FLT x86 = x81 * x45;
	const GEN_FLT x87 = (-1 * x41 * ((-1 * x47 * (x86 + (x83 * x44))) + (x85 * x43))) + (-1 * x84);
	const GEN_FLT x88 = x81 * x52;
	const GEN_FLT x89 = x85 * x33;
	const GEN_FLT x90 = 2 * x50 * curve_1;
	const GEN_FLT x91 = pow((1 + (-1 * x56 * x34 * pow(tilt_1, 2))), -1.0 / 2.0);
	const GEN_FLT x92 = pow(x55, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x93 = 2 * x39;
	const GEN_FLT x94 = 1.0 / 2.0 * pow(x55, -3.0 / 2.0) * x28 * tilt_1;
	const GEN_FLT x95 = (-1 * x91 * ((-1 * x94 * (x46 + (x93 * x42))) + (x92 * x32))) + (-1 * (x54 + (-1 * x53)) * x57);
	const GEN_FLT x96 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x39, x49)) + (-1 * phase_1) + (-1 * asin(x92 * x28)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x97 = (-1 * x91 * ((-1 * x94 * (x72 + (x71 * x93))) + (x69 * x92))) + (-1 * (x75 + (-1 * x74)) * x57);
	const GEN_FLT x98 = (-1 * x91 * ((-1 * x94 * (x86 + (x85 * x93))) + (x83 * x92))) + (-1 * (x89 + (-1 * x88)) * x57);
	out[0] = x48 + (((-1 * x54) + x53) * x58) + (x51 * x48);
	out[1] = x73 + (((-1 * x75) + x74) * x58) + (x73 * x51);
	out[2] = (((-1 * x89) + x88) * x58) + x87 + (x87 * x51);
	out[3] = x95 + (x96 * x95) + (x90 * x38);
	out[4] = x97 + (x97 * x96) + (x70 * x90);
	out[5] = (x98 * x96) + x98 + (x84 * x90);
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
	const GEN_FLT x0 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x1 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x2 = 2 * ((x1 * obj_qk) + (-1 * x0 * obj_qi));
	const GEN_FLT x3 = sensor_y + x2 + obj_py;
	const GEN_FLT x4 = x3 * lh_qw;
	const GEN_FLT x5 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x6 = 2 * ((x5 * obj_qi) + (-1 * x1 * obj_qj));
	const GEN_FLT x7 = sensor_z + x6 + obj_pz;
	const GEN_FLT x8 = x7 * lh_qi;
	const GEN_FLT x9 = 2 * ((x0 * obj_qj) + (-1 * x5 * obj_qk));
	const GEN_FLT x10 = sensor_x + x9 + obj_px;
	const GEN_FLT x11 = x10 * lh_qk;
	const GEN_FLT x12 = (-1 * x8) + x11 + x4;
	const GEN_FLT x13 = x7 * lh_qw;
	const GEN_FLT x14 = x10 * lh_qj;
	const GEN_FLT x15 = x3 * lh_qi;
	const GEN_FLT x16 = x15 + (-1 * x14) + x13;
	const GEN_FLT x17 = x10 + (2 * ((x16 * lh_qj) + (-1 * x12 * lh_qk))) + lh_px;
	const GEN_FLT x18 = pow(x17, 2);
	const GEN_FLT x19 = x10 * lh_qw;
	const GEN_FLT x20 = x3 * lh_qk;
	const GEN_FLT x21 = x7 * lh_qj;
	const GEN_FLT x22 = x21 + (-1 * x20) + x19;
	const GEN_FLT x23 = x7 + (2 * ((x12 * lh_qi) + (-1 * x22 * lh_qj))) + lh_pz;
	const GEN_FLT x24 = pow(x23, 2);
	const GEN_FLT x25 = x24 + x18;
	const GEN_FLT x26 = pow(x25, -1);
	const GEN_FLT x27 = x23 * x26;
	const GEN_FLT x28 = x3 + (2 * ((x22 * lh_qk) + (-1 * x16 * lh_qi))) + lh_py;
	const GEN_FLT x29 = pow(x28, 2);
	const GEN_FLT x30 = pow((1 + (-1 * x29 * x26 * pow(tilt_0, 2))), -1.0 / 2.0);
	const GEN_FLT x31 = pow(x25, -3.0 / 2.0) * x28 * tilt_0;
	const GEN_FLT x32 = x30 * x31;
	const GEN_FLT x33 = (x32 * x17) + x27;
	const GEN_FLT x34 = pow(x25, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x35 = -1 * x23;
	const GEN_FLT x36 = atan2(x17, x35);
	const GEN_FLT x37 =
		sin(1.5707963267949 + (-1 * x36) + (-1 * phase_0) + (-1 * asin(x34 * x28)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x38 = x30 * x34;
	const GEN_FLT x39 = x24 + x29;
	const GEN_FLT x40 = pow(x39, -1);
	const GEN_FLT x41 = x40 * x23;
	const GEN_FLT x42 = 2 * atan2(x28, x35) * curve_0;
	const GEN_FLT x43 = x26 * x17;
	const GEN_FLT x44 = (x32 * x23) + (-1 * x43);
	const GEN_FLT x45 = x40 * x28;
	const GEN_FLT x46 = 2 * x14;
	const GEN_FLT x47 = (2 * x15) + (-1 * x46);
	const GEN_FLT x48 = pow(x24, -1);
	const GEN_FLT x49 = x48 * x17;
	const GEN_FLT x50 = 2 * x20;
	const GEN_FLT x51 = (2 * x21) + (-1 * x50);
	const GEN_FLT x52 = pow(x23, -1);
	const GEN_FLT x53 = x24 * x26;
	const GEN_FLT x54 = ((-1 * x52 * x51) + (x47 * x49)) * x53;
	const GEN_FLT x55 = 2 * x8;
	const GEN_FLT x56 = (2 * x11) + (-1 * x55);
	const GEN_FLT x57 = 2 * x17;
	const GEN_FLT x58 = 2 * x23;
	const GEN_FLT x59 = x58 * x47;
	const GEN_FLT x60 = 1.0 / 2.0 * x31;
	const GEN_FLT x61 = (-1 * x30 * ((-1 * x60 * (x59 + (x51 * x57))) + (x56 * x34))) + (-1 * x54);
	const GEN_FLT x62 = x48 * x28;
	const GEN_FLT x63 = x62 * x47;
	const GEN_FLT x64 = x52 * x56;
	const GEN_FLT x65 = x40 * x24;
	const GEN_FLT x66 = x65 * x42;
	const GEN_FLT x67 = (-1 * sensor_z) + (-1 * x6) + (-1 * obj_pz);
	const GEN_FLT x68 = 2 * lh_qi;
	const GEN_FLT x69 = 2 * x4;
	const GEN_FLT x70 = x56 + x69 + (x67 * x68);
	const GEN_FLT x71 = 2 * lh_qk;
	const GEN_FLT x72 = 2 * lh_qj;
	const GEN_FLT x73 = (x3 * x72) + (-1 * x71 * x67);
	const GEN_FLT x74 = ((-1 * x73 * x52) + (x70 * x49)) * x53;
	const GEN_FLT x75 = 2 * x13;
	const GEN_FLT x76 = (-1 * x75) + x46 + (-4 * x15);
	const GEN_FLT x77 = x70 * x58;
	const GEN_FLT x78 = (-1 * x30 * ((-1 * x60 * (x77 + (x73 * x57))) + (x76 * x34))) + (-1 * x74);
	const GEN_FLT x79 = x70 * x62;
	const GEN_FLT x80 = x76 * x52;
	const GEN_FLT x81 = 2 * x19;
	const GEN_FLT x82 = (-1 * x81) + x50 + (-4 * x21);
	const GEN_FLT x83 = 2 * ((-1 * sensor_x) + (-1 * x9) + (-1 * obj_px));
	const GEN_FLT x84 = x47 + x75 + (x83 * lh_qj);
	const GEN_FLT x85 = ((-1 * x84 * x52) + (x82 * x49)) * x53;
	const GEN_FLT x86 = (x7 * x71) + (-1 * x83 * lh_qi);
	const GEN_FLT x87 = x82 * x58;
	const GEN_FLT x88 = (-1 * x30 * ((-1 * x60 * (x87 + (x84 * x57))) + (x86 * x34))) + (-1 * x85);
	const GEN_FLT x89 = x82 * x62;
	const GEN_FLT x90 = x86 * x52;
	const GEN_FLT x91 = (-1 * sensor_y) + (-1 * x2) + (-1 * obj_py);
	const GEN_FLT x92 = (x68 * x10) + (-1 * x72 * x91);
	const GEN_FLT x93 = x55 + (-1 * x69) + (-4 * x11);
	const GEN_FLT x94 = ((-1 * x52 * x93) + (x92 * x49)) * x53;
	const GEN_FLT x95 = x51 + x81 + (x71 * x91);
	const GEN_FLT x96 = x58 * x92;
	const GEN_FLT x97 = (-1 * x30 * ((-1 * x60 * (x96 + (x57 * x93))) + (x95 * x34))) + (-1 * x94);
	const GEN_FLT x98 = x62 * x92;
	const GEN_FLT x99 = x52 * x95;
	const GEN_FLT x100 = pow((1 + (-1 * x40 * x18 * pow(tilt_1, 2))), -1.0 / 2.0);
	const GEN_FLT x101 = pow(x39, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x102 = x101 * x100;
	const GEN_FLT x103 = 2 * x36 * curve_1;
	const GEN_FLT x104 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x28, x35)) + (-1 * asin(x17 * x101)) + (-1 * phase_1) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x105 = pow(x39, -3.0 / 2.0) * x17 * tilt_1;
	const GEN_FLT x106 = x100 * x105;
	const GEN_FLT x107 = (x28 * x106) + (-1 * x41);
	const GEN_FLT x108 = (x23 * x106) + x45;
	const GEN_FLT x109 = 2 * x28;
	const GEN_FLT x110 = 1.0 / 2.0 * x105;
	const GEN_FLT x111 =
		(-1 * x100 * ((-1 * x110 * (x59 + (x56 * x109))) + (x51 * x101))) + (-1 * (x64 + (-1 * x63)) * x65);
	const GEN_FLT x112 =
		(-1 * x100 * ((-1 * x110 * (x77 + (x76 * x109))) + (x73 * x101))) + (-1 * (x80 + (-1 * x79)) * x65);
	const GEN_FLT x113 =
		(-1 * x100 * ((-1 * x110 * (x87 + (x86 * x109))) + (x84 * x101))) + (-1 * (x90 + (-1 * x89)) * x65);
	const GEN_FLT x114 =
		(-1 * x100 * ((-1 * x110 * (x96 + (x95 * x109))) + (x93 * x101))) + (-1 * (x99 + (-1 * x98)) * x65);
	out[0] = x33 + (x33 * x37);
	out[1] = (-1 * x41 * x42) + (-1 * x38) + (-1 * x38 * x37);
	out[2] = x44 + (x42 * x45) + (x44 * x37);
	out[3] = x61 + (((-1 * x64) + x63) * x66) + (x61 * x37);
	out[4] = x78 + (((-1 * x80) + x79) * x66) + (x78 * x37);
	out[5] = x88 + (((-1 * x90) + x89) * x66) + (x88 * x37);
	out[6] = x97 + (((-1 * x99) + x98) * x66) + (x97 * x37);
	out[7] = (-1 * x102 * x104) + (-1 * x27 * x103) + (-1 * x102);
	out[8] = x107 + (x104 * x107);
	out[9] = x108 + (x108 * x104) + (x43 * x103);
	out[10] = (x104 * x111) + x111 + (x54 * x103);
	out[11] = x112 + (x104 * x112) + (x74 * x103);
	out[12] = x113 + (x104 * x113) + (x85 * x103);
	out[13] = (x104 * x114) + x114 + (x94 * x103);
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
	const GEN_FLT x0 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x1 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x2 = sensor_x + (2 * ((x1 * obj_qj) + (-1 * x0 * obj_qk))) + obj_px;
	const GEN_FLT x3 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x4 = sensor_y + (2 * ((x3 * obj_qk) + (-1 * x1 * obj_qi))) + obj_py;
	const GEN_FLT x5 = sensor_z + (2 * ((x0 * obj_qi) + (-1 * x3 * obj_qj))) + obj_pz;
	const GEN_FLT x6 = (x5 * lh_qj) + (-1 * x4 * lh_qk) + (x2 * lh_qw);
	const GEN_FLT x7 = (x2 * lh_qk) + (-1 * x5 * lh_qi) + (x4 * lh_qw);
	const GEN_FLT x8 = x5 + (2 * ((x7 * lh_qi) + (-1 * x6 * lh_qj))) + lh_pz;
	const GEN_FLT x9 = pow(x8, 2);
	const GEN_FLT x10 = (x4 * lh_qi) + (-1 * x2 * lh_qj) + (x5 * lh_qw);
	const GEN_FLT x11 = x2 + (2 * ((x10 * lh_qj) + (-1 * x7 * lh_qk))) + lh_px;
	const GEN_FLT x12 = pow(x11, 2);
	const GEN_FLT x13 = x12 + x9;
	const GEN_FLT x14 = (2 * ((x6 * lh_qk) + (-1 * x10 * lh_qi))) + x4 + lh_py;
	const GEN_FLT x15 = x14 * pow(x13, -1.0 / 2.0);
	const GEN_FLT x16 = -1 * x8;
	const GEN_FLT x17 = atan2(x11, x16);
	const GEN_FLT x18 = 1.5707963267949 + (-1 * x17) + (-1 * phase_0) + (-1 * asin(x15 * tilt_0)) + gibPhase_0;
	const GEN_FLT x19 = sin(x18) * gibMag_0;
	const GEN_FLT x20 = pow(x14, 2);
	const GEN_FLT x21 = x15 * pow((1 + (-1 * x20 * pow(x13, -1) * pow(tilt_0, 2))), -1.0 / 2.0);
	const GEN_FLT x22 = x20 + x9;
	const GEN_FLT x23 = pow(x22, -1.0 / 2.0) * x11;
	const GEN_FLT x24 =
		1.5707963267949 + (-1 * atan2(-1 * x14, x16)) + (-1 * asin(x23 * tilt_1)) + (-1 * phase_1) + gibPhase_1;
	const GEN_FLT x25 = sin(x24) * gibMag_1;
	const GEN_FLT x26 = x23 * pow((1 + (-1 * pow(x22, -1) * x12 * pow(tilt_1, 2))), -1.0 / 2.0);
	out[0] = -1 + (-1 * x19);
	out[1] = (-1 * x21) + (-1 * x21 * x19);
	out[2] = pow(atan2(x14, x16), 2);
	out[3] = x19;
	out[4] = -1 * cos(x18);
	out[5] = 0;
	out[6] = 0;
	out[7] = 0;
	out[8] = 0;
	out[9] = 0;
	out[10] = 0;
	out[11] = 0;
	out[12] = 0;
	out[13] = 0;
	out[14] = 0;
	out[15] = 0;
	out[16] = 0;
	out[17] = 0;
	out[18] = 0;
	out[19] = 0;
	out[20] = 0;
	out[21] = -1 + (-1 * x25);
	out[22] = (-1 * x25 * x26) + (-1 * x26);
	out[23] = pow(x17, 2);
	out[24] = x25;
	out[25] = -1 * cos(x24);
	out[26] = 0;
	out[27] = 0;
}

static inline FLT gen_reproject_axis_x(const SurvivePose *obj_p, const FLT *sensor_pt, const SurvivePose *lh_p,
									   const BaseStationCal *bsc0) {
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
	const GEN_FLT x0 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x1 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x2 = sensor_z + (2 * ((x1 * obj_qi) + (-1 * x0 * obj_qj))) + obj_pz;
	const GEN_FLT x3 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x4 = sensor_x + (2 * ((x3 * obj_qj) + (-1 * x1 * obj_qk))) + obj_px;
	const GEN_FLT x5 = sensor_y + (2 * ((x0 * obj_qk) + (-1 * x3 * obj_qi))) + obj_py;
	const GEN_FLT x6 = (x5 * lh_qi) + (-1 * x4 * lh_qj) + (x2 * lh_qw);
	const GEN_FLT x7 = (-1 * x5 * lh_qk) + (x2 * lh_qj) + (x4 * lh_qw);
	const GEN_FLT x8 = x5 + (2 * ((x7 * lh_qk) + (-1 * x6 * lh_qi))) + lh_py;
	const GEN_FLT x9 = (x4 * lh_qk) + (-1 * x2 * lh_qi) + (x5 * lh_qw);
	const GEN_FLT x10 = x2 + (2 * ((x9 * lh_qi) + (-1 * x7 * lh_qj))) + lh_pz;
	const GEN_FLT x11 = -1 * x10;
	const GEN_FLT x12 = x4 + (2 * ((x6 * lh_qj) + (-1 * x9 * lh_qk))) + lh_px;
	const GEN_FLT x13 = (-1 * atan2(x12, x11)) + (-1 * phase_0) +
						(-1 * asin(pow((pow(x12, 2) + pow(x10, 2)), -1.0 / 2.0) * x8 * tilt_0));
	return x13 + (-1 * cos(1.5707963267949 + x13 + gibPhase_0) * gibMag_0) + (pow(atan2(x8, x11), 2) * curve_0);
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
	const GEN_FLT x0 = 2 * lh_qj;
	const GEN_FLT x1 = x0 * lh_qw;
	const GEN_FLT x2 = 2 * lh_qk;
	const GEN_FLT x3 = x2 * lh_qi;
	const GEN_FLT x4 = x3 + (-1 * x1);
	const GEN_FLT x5 = obj_qw * sensor_y;
	const GEN_FLT x6 = obj_qi * sensor_z;
	const GEN_FLT x7 = obj_qk * sensor_x;
	const GEN_FLT x8 = x7 + (-1 * x6) + x5;
	const GEN_FLT x9 = obj_qj * sensor_x;
	const GEN_FLT x10 = obj_qw * sensor_z;
	const GEN_FLT x11 = obj_qi * sensor_y;
	const GEN_FLT x12 = x11 + x10 + (-1 * x9);
	const GEN_FLT x13 = sensor_x + (2 * ((x12 * obj_qj) + (-1 * x8 * obj_qk))) + obj_px;
	const GEN_FLT x14 = obj_qw * sensor_x;
	const GEN_FLT x15 = obj_qk * sensor_y;
	const GEN_FLT x16 = obj_qj * sensor_z;
	const GEN_FLT x17 = x16 + (-1 * x15) + x14;
	const GEN_FLT x18 = sensor_y + (2 * ((x17 * obj_qk) + (-1 * x12 * obj_qi))) + obj_py;
	const GEN_FLT x19 = sensor_z + (2 * ((x8 * obj_qi) + (-1 * x17 * obj_qj))) + obj_pz;
	const GEN_FLT x20 = (x19 * lh_qj) + (-1 * x18 * lh_qk) + (x13 * lh_qw);
	const GEN_FLT x21 = (x13 * lh_qk) + (-1 * x19 * lh_qi) + (x18 * lh_qw);
	const GEN_FLT x22 = (2 * ((x21 * lh_qi) + (-1 * x20 * lh_qj))) + x19 + lh_pz;
	const GEN_FLT x23 = pow(x22, 2);
	const GEN_FLT x24 = pow(x23, -1);
	const GEN_FLT x25 = (x18 * lh_qi) + (-1 * x13 * lh_qj) + (x19 * lh_qw);
	const GEN_FLT x26 = x13 + (2 * ((x25 * lh_qj) + (-1 * x21 * lh_qk))) + lh_px;
	const GEN_FLT x27 = x24 * x26;
	const GEN_FLT x28 = -2 * pow(lh_qk, 2);
	const GEN_FLT x29 = -2 * pow(lh_qj, 2);
	const GEN_FLT x30 = 1 + x29 + x28;
	const GEN_FLT x31 = pow(x22, -1);
	const GEN_FLT x32 = x23 + pow(x26, 2);
	const GEN_FLT x33 = pow(x32, -1);
	const GEN_FLT x34 = x33 * x23;
	const GEN_FLT x35 = 2 * lh_qi;
	const GEN_FLT x36 = x35 * lh_qj;
	const GEN_FLT x37 = x2 * lh_qw;
	const GEN_FLT x38 = x37 + x36;
	const GEN_FLT x39 = pow(x32, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x40 = 2 * x26;
	const GEN_FLT x41 = 2 * x22;
	const GEN_FLT x42 = x18 + (2 * ((x20 * lh_qk) + (-1 * x25 * lh_qi))) + lh_py;
	const GEN_FLT x43 = 1.0 / 2.0 * x42 * pow(x32, -3.0 / 2.0) * tilt_0;
	const GEN_FLT x44 = pow(x42, 2);
	const GEN_FLT x45 = pow((1 + (-1 * x44 * x33 * pow(tilt_0, 2))), -1.0 / 2.0);
	const GEN_FLT x46 = (-1 * x45 * ((-1 * x43 * ((x4 * x41) + (x40 * x30))) + (x38 * x39))) +
						(-1 * x34 * ((-1 * x30 * x31) + (x4 * x27)));
	const GEN_FLT x47 = -1 * x22;
	const GEN_FLT x48 =
		sin(1.5707963267949 + (-1 * atan2(x26, x47)) + (-1 * phase_0) + (-1 * asin(x42 * x39)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x49 = x42 * x24;
	const GEN_FLT x50 = 2 * pow((x23 + x44), -1) * x23 * atan2(x42, x47) * curve_0;
	const GEN_FLT x51 = x2 * lh_qj;
	const GEN_FLT x52 = x35 * lh_qw;
	const GEN_FLT x53 = x52 + x51;
	const GEN_FLT x54 = x53 * x24;
	const GEN_FLT x55 = x36 + (-1 * x37);
	const GEN_FLT x56 = 1 + (-2 * pow(lh_qi, 2));
	const GEN_FLT x57 = x56 + x28;
	const GEN_FLT x58 = (-1 * x45 * ((-1 * ((x53 * x41) + (x55 * x40)) * x43) + (x57 * x39))) +
						(-1 * ((-1 * x55 * x31) + (x54 * x26)) * x34);
	const GEN_FLT x59 = x56 + x29;
	const GEN_FLT x60 = x1 + x3;
	const GEN_FLT x61 = x51 + (-1 * x52);
	const GEN_FLT x62 = (-1 * x45 * ((-1 * ((x59 * x41) + (x60 * x40)) * x43) + (x61 * x39))) +
						(-1 * ((-1 * x60 * x31) + (x59 * x27)) * x34);
	const GEN_FLT x63 = 2 * x9;
	const GEN_FLT x64 = 2 * x11;
	const GEN_FLT x65 = x64 + (-1 * x63);
	const GEN_FLT x66 = 2 * x6;
	const GEN_FLT x67 = 2 * x7;
	const GEN_FLT x68 = x67 + (-1 * x66);
	const GEN_FLT x69 = 2 * x15;
	const GEN_FLT x70 = 2 * x16;
	const GEN_FLT x71 = x70 + (-1 * x69);
	const GEN_FLT x72 = (x71 * lh_qw) + (-1 * x68 * lh_qk) + (x65 * lh_qj);
	const GEN_FLT x73 = (x68 * lh_qw) + (-1 * x65 * lh_qi) + (x71 * lh_qk);
	const GEN_FLT x74 = x65 + (x73 * x35) + (-1 * x0 * x72);
	const GEN_FLT x75 = (x65 * lh_qw) + (-1 * x71 * lh_qj) + (x68 * lh_qi);
	const GEN_FLT x76 = x71 + (x0 * x75) + (-1 * x2 * x73);
	const GEN_FLT x77 = x68 + (x2 * x72) + (-1 * x75 * x35);
	const GEN_FLT x78 = (-1 * x45 * ((-1 * ((x74 * x41) + (x76 * x40)) * x43) + (x77 * x39))) +
						(-1 * ((-1 * x76 * x31) + (x74 * x27)) * x34);
	const GEN_FLT x79 = 2 * x5;
	const GEN_FLT x80 = x79 + (-4 * x6) + x67;
	const GEN_FLT x81 = 2 * x10;
	const GEN_FLT x82 = x63 + (-1 * x81) + (-4 * x11);
	const GEN_FLT x83 = 2 * obj_qk * sensor_z;
	const GEN_FLT x84 = 2 * obj_qj * sensor_y;
	const GEN_FLT x85 = x84 + x83;
	const GEN_FLT x86 = (x85 * lh_qw) + (-1 * x82 * lh_qk) + (x80 * lh_qj);
	const GEN_FLT x87 = (x82 * lh_qw) + (-1 * x80 * lh_qi) + (x85 * lh_qk);
	const GEN_FLT x88 = (x87 * x35) + x80 + (-1 * x0 * x86);
	const GEN_FLT x89 = x88 * x24;
	const GEN_FLT x90 = (x80 * lh_qw) + (-1 * x85 * lh_qj) + (x82 * lh_qi);
	const GEN_FLT x91 = x85 + (x0 * x90) + (-1 * x2 * x87);
	const GEN_FLT x92 = x82 + (x2 * x86) + (-1 * x90 * x35);
	const GEN_FLT x93 = (-1 * x45 * ((-1 * ((x88 * x41) + (x91 * x40)) * x43) + (x92 * x39))) +
						(-1 * ((-1 * x91 * x31) + (x89 * x26)) * x34);
	const GEN_FLT x94 = 2 * x14;
	const GEN_FLT x95 = (-1 * x94) + x69 + (-4 * x16);
	const GEN_FLT x96 = 2 * obj_qi * sensor_x;
	const GEN_FLT x97 = x83 + x96;
	const GEN_FLT x98 = (-4 * x9) + x81 + x64;
	const GEN_FLT x99 = (-1 * x97 * lh_qk) + (x98 * lh_qw) + (x95 * lh_qj);
	const GEN_FLT x100 = (x97 * lh_qw) + (-1 * x95 * lh_qi) + (x98 * lh_qk);
	const GEN_FLT x101 = (x35 * x100) + x95 + (-1 * x0 * x99);
	const GEN_FLT x102 = (x95 * lh_qw) + (-1 * x98 * lh_qj) + (x97 * lh_qi);
	const GEN_FLT x103 = x98 + (x0 * x102) + (-1 * x2 * x100);
	const GEN_FLT x104 = x97 + (x2 * x99) + (-1 * x35 * x102);
	const GEN_FLT x105 = (-1 * x45 * ((-1 * ((x41 * x101) + (x40 * x103)) * x43) + (x39 * x104))) +
						 (-1 * ((-1 * x31 * x103) + (x27 * x101)) * x34);
	const GEN_FLT x106 = x96 + x84;
	const GEN_FLT x107 = x94 + (-4 * x15) + x70;
	const GEN_FLT x108 = (-1 * x79) + x66 + (-4 * x7);
	const GEN_FLT x109 = (x108 * lh_qw) + (-1 * x107 * lh_qk) + (x106 * lh_qj);
	const GEN_FLT x110 = (x107 * lh_qw) + (-1 * x106 * lh_qi) + (x108 * lh_qk);
	const GEN_FLT x111 = x106 + (x35 * x110) + (-1 * x0 * x109);
	const GEN_FLT x112 = (x106 * lh_qw) + (-1 * x108 * lh_qj) + (x107 * lh_qi);
	const GEN_FLT x113 = x108 + (x0 * x112) + (-1 * x2 * x110);
	const GEN_FLT x114 = x107 + (x2 * x109) + (-1 * x35 * x112);
	const GEN_FLT x115 = (-1 * x45 * ((-1 * ((x41 * x111) + (x40 * x113)) * x43) + (x39 * x114))) +
						 (-1 * ((-1 * x31 * x113) + (x27 * x111)) * x34);
	out[0] = x46 + (x50 * ((-1 * x31 * x38) + (x4 * x49))) + (x46 * x48);
	out[1] = x58 + (((-1 * x57 * x31) + (x54 * x42)) * x50) + (x58 * x48);
	out[2] = x62 + (((-1 * x61 * x31) + (x59 * x49)) * x50) + (x62 * x48);
	out[3] = x78 + (((-1 * x77 * x31) + (x74 * x49)) * x50) + (x78 * x48);
	out[4] = x93 + (((-1 * x92 * x31) + (x89 * x42)) * x50) + (x93 * x48);
	out[5] = x105 + (((-1 * x31 * x104) + (x49 * x101)) * x50) + (x48 * x105);
	out[6] = x115 + (((-1 * x31 * x114) + (x49 * x111)) * x50) + (x48 * x115);
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
	const GEN_FLT x0 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x1 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x2 = sensor_y + (2 * ((x1 * obj_qk) + (-1 * x0 * obj_qi))) + obj_py;
	const GEN_FLT x3 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x4 = sensor_z + (2 * ((x3 * obj_qi) + (-1 * x1 * obj_qj))) + obj_pz;
	const GEN_FLT x5 = sensor_x + (2 * ((x0 * obj_qj) + (-1 * x3 * obj_qk))) + obj_px;
	const GEN_FLT x6 = (x5 * lh_qk) + (-1 * x4 * lh_qi) + (x2 * lh_qw);
	const GEN_FLT x7 = (x2 * lh_qi) + (-1 * x5 * lh_qj) + (x4 * lh_qw);
	const GEN_FLT x8 = x5 + (2 * ((x7 * lh_qj) + (-1 * x6 * lh_qk))) + lh_px;
	const GEN_FLT x9 = 2 * obj_qw;
	const GEN_FLT x10 = x9 * obj_qj;
	const GEN_FLT x11 = 2 * obj_qi;
	const GEN_FLT x12 = x11 * obj_qk;
	const GEN_FLT x13 = x12 + (-1 * x10);
	const GEN_FLT x14 = x11 * obj_qj;
	const GEN_FLT x15 = x9 * obj_qk;
	const GEN_FLT x16 = x15 + x14;
	const GEN_FLT x17 = -2 * pow(obj_qj, 2);
	const GEN_FLT x18 = 1 + (-2 * pow(obj_qk, 2));
	const GEN_FLT x19 = x18 + x17;
	const GEN_FLT x20 = (x19 * lh_qw) + (-1 * x16 * lh_qk) + (x13 * lh_qj);
	const GEN_FLT x21 = 2 * lh_qj;
	const GEN_FLT x22 = (x16 * lh_qw) + (-1 * x13 * lh_qi) + (x19 * lh_qk);
	const GEN_FLT x23 = 2 * lh_qi;
	const GEN_FLT x24 = x13 + (x22 * x23) + (-1 * x20 * x21);
	const GEN_FLT x25 = (x4 * lh_qj) + (-1 * x2 * lh_qk) + (x5 * lh_qw);
	const GEN_FLT x26 = x4 + (2 * ((x6 * lh_qi) + (-1 * x25 * lh_qj))) + lh_pz;
	const GEN_FLT x27 = pow(x26, 2);
	const GEN_FLT x28 = pow(x27, -1);
	const GEN_FLT x29 = x24 * x28;
	const GEN_FLT x30 = 2 * lh_qk;
	const GEN_FLT x31 = (x13 * lh_qw) + (-1 * x19 * lh_qj) + (x16 * lh_qi);
	const GEN_FLT x32 = x19 + (x31 * x21) + (-1 * x30 * x22);
	const GEN_FLT x33 = pow(x26, -1);
	const GEN_FLT x34 = x27 + pow(x8, 2);
	const GEN_FLT x35 = pow(x34, -1);
	const GEN_FLT x36 = x35 * x27;
	const GEN_FLT x37 = x2 + (2 * ((x25 * lh_qk) + (-1 * x7 * lh_qi))) + lh_py;
	const GEN_FLT x38 = pow(x37, 2);
	const GEN_FLT x39 = pow((1 + (-1 * x35 * x38 * pow(tilt_0, 2))), -1.0 / 2.0);
	const GEN_FLT x40 = x16 + (x30 * x20) + (-1 * x31 * x23);
	const GEN_FLT x41 = pow(x34, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x42 = 2 * x8;
	const GEN_FLT x43 = 2 * x26;
	const GEN_FLT x44 = 1.0 / 2.0 * pow(x34, -3.0 / 2.0) * x37 * tilt_0;
	const GEN_FLT x45 = (-1 * x39 * ((-1 * ((x43 * x24) + (x42 * x32)) * x44) + (x40 * x41))) +
						(-1 * x36 * ((-1 * x32 * x33) + (x8 * x29)));
	const GEN_FLT x46 = -1 * x26;
	const GEN_FLT x47 =
		sin(1.5707963267949 + (-1 * atan2(x8, x46)) + (-1 * phase_0) + (-1 * asin(x41 * x37)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x48 = 2 * pow((x27 + x38), -1) * x27 * atan2(x37, x46) * curve_0;
	const GEN_FLT x49 = 2 * obj_qk * obj_qj;
	const GEN_FLT x50 = x9 * obj_qi;
	const GEN_FLT x51 = x50 + x49;
	const GEN_FLT x52 = -2 * pow(obj_qi, 2);
	const GEN_FLT x53 = x18 + x52;
	const GEN_FLT x54 = x14 + (-1 * x15);
	const GEN_FLT x55 = (x54 * lh_qw) + (-1 * x53 * lh_qk) + (x51 * lh_qj);
	const GEN_FLT x56 = (x53 * lh_qw) + (-1 * x51 * lh_qi) + (x54 * lh_qk);
	const GEN_FLT x57 = x51 + (x56 * x23) + (-1 * x55 * x21);
	const GEN_FLT x58 = x8 * x28;
	const GEN_FLT x59 = (x51 * lh_qw) + (-1 * x54 * lh_qj) + (x53 * lh_qi);
	const GEN_FLT x60 = x54 + (x59 * x21) + (-1 * x56 * x30);
	const GEN_FLT x61 = x53 + (x55 * x30) + (-1 * x59 * x23);
	const GEN_FLT x62 = (-1 * x39 * ((-1 * ((x57 * x43) + (x60 * x42)) * x44) + (x61 * x41))) +
						(-1 * ((-1 * x60 * x33) + (x58 * x57)) * x36);
	const GEN_FLT x63 = x37 * x28;
	const GEN_FLT x64 = 1 + x52 + x17;
	const GEN_FLT x65 = x49 + (-1 * x50);
	const GEN_FLT x66 = x10 + x12;
	const GEN_FLT x67 = (x66 * lh_qw) + (-1 * x65 * lh_qk) + (x64 * lh_qj);
	const GEN_FLT x68 = (x65 * lh_qw) + (-1 * x64 * lh_qi) + (x66 * lh_qk);
	const GEN_FLT x69 = x64 + (x68 * x23) + (-1 * x67 * x21);
	const GEN_FLT x70 = (x64 * lh_qw) + (-1 * x66 * lh_qj) + (x65 * lh_qi);
	const GEN_FLT x71 = x66 + (x70 * x21) + (-1 * x68 * x30);
	const GEN_FLT x72 = x65 + (x67 * x30) + (-1 * x70 * x23);
	const GEN_FLT x73 = (-1 * x39 * ((-1 * ((x69 * x43) + (x71 * x42)) * x44) + (x72 * x41))) +
						(-1 * ((-1 * x71 * x33) + (x69 * x58)) * x36);
	out[0] = x45 + (((-1 * x40 * x33) + (x37 * x29)) * x48) + (x45 * x47);
	out[1] = x62 + (((-1 * x61 * x33) + (x63 * x57)) * x48) + (x62 * x47);
	out[2] = x73 + (((-1 * x72 * x33) + (x63 * x69)) * x48) + (x73 * x47);
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
	const GEN_FLT x0 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x1 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x2 = 2 * ((x1 * obj_qk) + (-1 * x0 * obj_qi));
	const GEN_FLT x3 = sensor_y + x2 + obj_py;
	const GEN_FLT x4 = x3 * lh_qw;
	const GEN_FLT x5 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x6 = 2 * ((x5 * obj_qi) + (-1 * x1 * obj_qj));
	const GEN_FLT x7 = sensor_z + x6 + obj_pz;
	const GEN_FLT x8 = x7 * lh_qi;
	const GEN_FLT x9 = 2 * ((x0 * obj_qj) + (-1 * x5 * obj_qk));
	const GEN_FLT x10 = sensor_x + x9 + obj_px;
	const GEN_FLT x11 = x10 * lh_qk;
	const GEN_FLT x12 = (-1 * x8) + x11 + x4;
	const GEN_FLT x13 = x7 * lh_qw;
	const GEN_FLT x14 = x10 * lh_qj;
	const GEN_FLT x15 = x3 * lh_qi;
	const GEN_FLT x16 = x15 + (-1 * x14) + x13;
	const GEN_FLT x17 = x10 + (2 * ((x16 * lh_qj) + (-1 * x12 * lh_qk))) + lh_px;
	const GEN_FLT x18 = x10 * lh_qw;
	const GEN_FLT x19 = x3 * lh_qk;
	const GEN_FLT x20 = x7 * lh_qj;
	const GEN_FLT x21 = x20 + (-1 * x19) + x18;
	const GEN_FLT x22 = x7 + (2 * ((x12 * lh_qi) + (-1 * x21 * lh_qj))) + lh_pz;
	const GEN_FLT x23 = pow(x22, 2);
	const GEN_FLT x24 = x23 + pow(x17, 2);
	const GEN_FLT x25 = pow(x24, -1);
	const GEN_FLT x26 = x3 + (2 * ((x21 * lh_qk) + (-1 * x16 * lh_qi))) + lh_py;
	const GEN_FLT x27 = pow(x26, 2);
	const GEN_FLT x28 = pow((1 + (-1 * x25 * x27 * pow(tilt_0, 2))), -1.0 / 2.0);
	const GEN_FLT x29 = pow(x24, -3.0 / 2.0) * x26 * tilt_0;
	const GEN_FLT x30 = x28 * x29;
	const GEN_FLT x31 = (x30 * x17) + (x25 * x22);
	const GEN_FLT x32 = pow(x24, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x33 = -1 * x22;
	const GEN_FLT x34 =
		sin(1.5707963267949 + (-1 * atan2(x17, x33)) + (-1 * phase_0) + (-1 * asin(x32 * x26)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x35 = x32 * x28;
	const GEN_FLT x36 = 2 * x22;
	const GEN_FLT x37 = pow((x23 + x27), -1) * atan2(x26, x33) * curve_0;
	const GEN_FLT x38 = (x30 * x22) + (-1 * x25 * x17);
	const GEN_FLT x39 = 2 * x37;
	const GEN_FLT x40 = 2 * x14;
	const GEN_FLT x41 = (2 * x15) + (-1 * x40);
	const GEN_FLT x42 = pow(x23, -1);
	const GEN_FLT x43 = x42 * x17;
	const GEN_FLT x44 = 2 * x19;
	const GEN_FLT x45 = (2 * x20) + (-1 * x44);
	const GEN_FLT x46 = pow(x22, -1);
	const GEN_FLT x47 = x25 * x23;
	const GEN_FLT x48 = 2 * x8;
	const GEN_FLT x49 = (2 * x11) + (-1 * x48);
	const GEN_FLT x50 = 2 * x17;
	const GEN_FLT x51 = 1.0 / 2.0 * x29;
	const GEN_FLT x52 = (-1 * x28 * ((-1 * ((x41 * x36) + (x50 * x45)) * x51) + (x49 * x32))) +
						(-1 * ((-1 * x45 * x46) + (x41 * x43)) * x47);
	const GEN_FLT x53 = x42 * x26;
	const GEN_FLT x54 = x39 * x23;
	const GEN_FLT x55 = (-1 * sensor_z) + (-1 * x6) + (-1 * obj_pz);
	const GEN_FLT x56 = 2 * lh_qi;
	const GEN_FLT x57 = 2 * x4;
	const GEN_FLT x58 = x49 + x57 + (x56 * x55);
	const GEN_FLT x59 = 2 * lh_qk;
	const GEN_FLT x60 = 2 * lh_qj;
	const GEN_FLT x61 = (x3 * x60) + (-1 * x55 * x59);
	const GEN_FLT x62 = 2 * x13;
	const GEN_FLT x63 = (-1 * x62) + x40 + (-4 * x15);
	const GEN_FLT x64 = (-1 * x28 * ((-1 * ((x58 * x36) + (x61 * x50)) * x51) + (x63 * x32))) +
						(-1 * ((-1 * x61 * x46) + (x58 * x43)) * x47);
	const GEN_FLT x65 = 2 * x18;
	const GEN_FLT x66 = (-1 * x65) + x44 + (-4 * x20);
	const GEN_FLT x67 = (-1 * sensor_x) + (-1 * x9) + (-1 * obj_px);
	const GEN_FLT x68 = x41 + x62 + (x60 * x67);
	const GEN_FLT x69 = (x7 * x59) + (-1 * x67 * x56);
	const GEN_FLT x70 = (-1 * x28 * ((-1 * ((x66 * x36) + (x68 * x50)) * x51) + (x69 * x32))) +
						(-1 * ((-1 * x68 * x46) + (x66 * x43)) * x47);
	const GEN_FLT x71 = (-1 * sensor_y) + (-1 * x2) + (-1 * obj_py);
	const GEN_FLT x72 = (x56 * x10) + (-1 * x71 * x60);
	const GEN_FLT x73 = (-1 * x57) + x48 + (-4 * x11);
	const GEN_FLT x74 = x45 + x65 + (x71 * x59);
	const GEN_FLT x75 = (-1 * x28 * ((-1 * ((x72 * x36) + (x73 * x50)) * x51) + (x74 * x32))) +
						(-1 * ((-1 * x73 * x46) + (x72 * x43)) * x47);
	out[0] = x31 + (x31 * x34);
	out[1] = (-1 * x36 * x37) + (-1 * x35) + (-1 * x34 * x35);
	out[2] = x38 + (x39 * x26) + (x34 * x38);
	out[3] = x52 + (((-1 * x46 * x49) + (x53 * x41)) * x54) + (x52 * x34);
	out[4] = x64 + (((-1 * x63 * x46) + (x53 * x58)) * x54) + (x64 * x34);
	out[5] = x70 + (((-1 * x69 * x46) + (x66 * x53)) * x54) + (x70 * x34);
	out[6] = x75 + (((-1 * x74 * x46) + (x72 * x53)) * x54) + (x75 * x34);
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
	const GEN_FLT x0 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x1 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x2 = sensor_x + (2 * ((x1 * obj_qj) + (-1 * x0 * obj_qk))) + obj_px;
	const GEN_FLT x3 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x4 = sensor_y + (2 * ((x3 * obj_qk) + (-1 * x1 * obj_qi))) + obj_py;
	const GEN_FLT x5 = sensor_z + (2 * ((x0 * obj_qi) + (-1 * x3 * obj_qj))) + obj_pz;
	const GEN_FLT x6 = (x5 * lh_qj) + (-1 * x4 * lh_qk) + (x2 * lh_qw);
	const GEN_FLT x7 = (x2 * lh_qk) + (-1 * x5 * lh_qi) + (x4 * lh_qw);
	const GEN_FLT x8 = x5 + (2 * ((x7 * lh_qi) + (-1 * x6 * lh_qj))) + lh_pz;
	const GEN_FLT x9 = (x4 * lh_qi) + (-1 * x2 * lh_qj) + (x5 * lh_qw);
	const GEN_FLT x10 = x2 + (2 * ((x9 * lh_qj) + (-1 * x7 * lh_qk))) + lh_px;
	const GEN_FLT x11 = pow(x10, 2) + pow(x8, 2);
	const GEN_FLT x12 = (2 * ((x6 * lh_qk) + (-1 * x9 * lh_qi))) + x4 + lh_py;
	const GEN_FLT x13 = x12 * pow(x11, -1.0 / 2.0);
	const GEN_FLT x14 = -1 * x8;
	const GEN_FLT x15 =
		1.5707963267949 + (-1 * atan2(x10, x14)) + (-1 * phase_0) + (-1 * asin(x13 * tilt_0)) + gibPhase_0;
	const GEN_FLT x16 = sin(x15) * gibMag_0;
	const GEN_FLT x17 = x13 * pow((1 + (-1 * pow(x12, 2) * pow(x11, -1) * pow(tilt_0, 2))), -1.0 / 2.0);
	out[0] = -1 + (-1 * x16);
	out[1] = (-1 * x17) + (-1 * x17 * x16);
	out[2] = pow(atan2(x12, x14), 2);
	out[3] = x16;
	out[4] = -1 * cos(x15);
	out[5] = 0;
	out[6] = 0;
}

static inline FLT gen_reproject_axis_y(const SurvivePose *obj_p, const FLT *sensor_pt, const SurvivePose *lh_p,
									   const BaseStationCal *bsc1) {
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
	const GEN_FLT x0 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x1 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x2 = sensor_x + (2 * ((x1 * obj_qj) + (-1 * x0 * obj_qk))) + obj_px;
	const GEN_FLT x3 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x4 = sensor_y + (2 * ((x3 * obj_qk) + (-1 * x1 * obj_qi))) + obj_py;
	const GEN_FLT x5 = sensor_z + (2 * ((x0 * obj_qi) + (-1 * x3 * obj_qj))) + obj_pz;
	const GEN_FLT x6 = (x5 * lh_qj) + (-1 * x4 * lh_qk) + (x2 * lh_qw);
	const GEN_FLT x7 = (x2 * lh_qk) + (-1 * x5 * lh_qi) + (x4 * lh_qw);
	const GEN_FLT x8 = x5 + (2 * ((x7 * lh_qi) + (-1 * x6 * lh_qj))) + lh_pz;
	const GEN_FLT x9 = (x4 * lh_qi) + (-1 * x2 * lh_qj) + (x5 * lh_qw);
	const GEN_FLT x10 = (2 * ((x6 * lh_qk) + (-1 * x9 * lh_qi))) + x4 + lh_py;
	const GEN_FLT x11 = x2 + (2 * ((x9 * lh_qj) + (-1 * x7 * lh_qk))) + lh_px;
	const GEN_FLT x12 = -1 * x8;
	const GEN_FLT x13 = (-1 * atan2(-1 * x10, x12)) +
						(-1 * asin(x11 * pow((pow(x10, 2) + pow(x8, 2)), -1.0 / 2.0) * tilt_1)) + (-1 * phase_1);
	return x13 + (pow(atan2(x11, x12), 2) * curve_1) + (-1 * cos(1.5707963267949 + x13 + gibPhase_1) * gibMag_1);
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
	const GEN_FLT x0 = 2 * lh_qj;
	const GEN_FLT x1 = x0 * lh_qw;
	const GEN_FLT x2 = 2 * lh_qk;
	const GEN_FLT x3 = x2 * lh_qi;
	const GEN_FLT x4 = x3 + (-1 * x1);
	const GEN_FLT x5 = obj_qj * sensor_x;
	const GEN_FLT x6 = obj_qw * sensor_z;
	const GEN_FLT x7 = obj_qi * sensor_y;
	const GEN_FLT x8 = x7 + x6 + (-1 * x5);
	const GEN_FLT x9 = obj_qw * sensor_x;
	const GEN_FLT x10 = obj_qk * sensor_y;
	const GEN_FLT x11 = obj_qj * sensor_z;
	const GEN_FLT x12 = x11 + (-1 * x10) + x9;
	const GEN_FLT x13 = sensor_y + (2 * ((x12 * obj_qk) + (-1 * x8 * obj_qi))) + obj_py;
	const GEN_FLT x14 = obj_qw * sensor_y;
	const GEN_FLT x15 = obj_qi * sensor_z;
	const GEN_FLT x16 = obj_qk * sensor_x;
	const GEN_FLT x17 = x16 + (-1 * x15) + x14;
	const GEN_FLT x18 = sensor_z + (2 * ((x17 * obj_qi) + (-1 * x12 * obj_qj))) + obj_pz;
	const GEN_FLT x19 = sensor_x + (2 * ((x8 * obj_qj) + (-1 * x17 * obj_qk))) + obj_px;
	const GEN_FLT x20 = (x19 * lh_qk) + (-1 * x18 * lh_qi) + (x13 * lh_qw);
	const GEN_FLT x21 = (x13 * lh_qi) + (-1 * x19 * lh_qj) + (x18 * lh_qw);
	const GEN_FLT x22 = x19 + (2 * ((x21 * lh_qj) + (-1 * x20 * lh_qk))) + lh_px;
	const GEN_FLT x23 = (x18 * lh_qj) + (-1 * x13 * lh_qk) + (x19 * lh_qw);
	const GEN_FLT x24 = x18 + (2 * ((x20 * lh_qi) + (-1 * x23 * lh_qj))) + lh_pz;
	const GEN_FLT x25 = pow(x24, 2);
	const GEN_FLT x26 = pow(x25, -1);
	const GEN_FLT x27 = x22 * x26;
	const GEN_FLT x28 = -2 * pow(lh_qk, 2);
	const GEN_FLT x29 = -2 * pow(lh_qj, 2);
	const GEN_FLT x30 = 1 + x29 + x28;
	const GEN_FLT x31 = pow(x24, -1);
	const GEN_FLT x32 = pow(x22, 2);
	const GEN_FLT x33 = -1 * x24;
	const GEN_FLT x34 = 2 * pow((x25 + x32), -1) * x25 * atan2(x22, x33) * curve_1;
	const GEN_FLT x35 = x13 + (2 * ((x23 * lh_qk) + (-1 * x21 * lh_qi))) + lh_py;
	const GEN_FLT x36 = x35 * x26;
	const GEN_FLT x37 = x0 * lh_qi;
	const GEN_FLT x38 = x2 * lh_qw;
	const GEN_FLT x39 = x38 + x37;
	const GEN_FLT x40 = x25 + pow(x35, 2);
	const GEN_FLT x41 = pow(x40, -1);
	const GEN_FLT x42 = x41 * x25;
	const GEN_FLT x43 = pow((1 + (-1 * x41 * x32 * pow(tilt_1, 2))), -1.0 / 2.0);
	const GEN_FLT x44 = pow(x40, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x45 = 2 * x35;
	const GEN_FLT x46 = 2 * x24;
	const GEN_FLT x47 = 1.0 / 2.0 * pow(x40, -3.0 / 2.0) * x22 * tilt_1;
	const GEN_FLT x48 = (-1 * x43 * ((-1 * x47 * ((x4 * x46) + (x45 * x39))) + (x44 * x30))) +
						(-1 * x42 * ((x31 * x39) + (-1 * x4 * x36)));
	const GEN_FLT x49 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x35, x33)) + (-1 * asin(x44 * x22)) + (-1 * phase_1) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x50 = x2 * lh_qj;
	const GEN_FLT x51 = 2 * lh_qi;
	const GEN_FLT x52 = x51 * lh_qw;
	const GEN_FLT x53 = x52 + x50;
	const GEN_FLT x54 = x37 + (-1 * x38);
	const GEN_FLT x55 = 1 + (-2 * pow(lh_qi, 2));
	const GEN_FLT x56 = x55 + x28;
	const GEN_FLT x57 = (-1 * x43 * ((-1 * ((x53 * x46) + (x56 * x45)) * x47) + (x54 * x44))) +
						(-1 * ((x56 * x31) + (-1 * x53 * x36)) * x42);
	const GEN_FLT x58 = x55 + x29;
	const GEN_FLT x59 = x1 + x3;
	const GEN_FLT x60 = x50 + (-1 * x52);
	const GEN_FLT x61 = (-1 * x43 * ((-1 * ((x58 * x46) + (x60 * x45)) * x47) + (x59 * x44))) +
						(-1 * ((x60 * x31) + (-1 * x58 * x36)) * x42);
	const GEN_FLT x62 = 2 * x5;
	const GEN_FLT x63 = 2 * x7;
	const GEN_FLT x64 = x63 + (-1 * x62);
	const GEN_FLT x65 = 2 * x15;
	const GEN_FLT x66 = 2 * x16;
	const GEN_FLT x67 = x66 + (-1 * x65);
	const GEN_FLT x68 = 2 * x10;
	const GEN_FLT x69 = 2 * x11;
	const GEN_FLT x70 = x69 + (-1 * x68);
	const GEN_FLT x71 = (x70 * lh_qw) + (-1 * x67 * lh_qk) + (x64 * lh_qj);
	const GEN_FLT x72 = (x67 * lh_qw) + (-1 * x64 * lh_qi) + (x70 * lh_qk);
	const GEN_FLT x73 = x64 + (x72 * x51) + (-1 * x0 * x71);
	const GEN_FLT x74 = (x64 * lh_qw) + (-1 * x70 * lh_qj) + (x67 * lh_qi);
	const GEN_FLT x75 = x70 + (x0 * x74) + (-1 * x2 * x72);
	const GEN_FLT x76 = x67 + (x2 * x71) + (-1 * x74 * x51);
	const GEN_FLT x77 = (-1 * x43 * ((-1 * ((x73 * x46) + (x76 * x45)) * x47) + (x75 * x44))) +
						(-1 * ((x76 * x31) + (-1 * x73 * x36)) * x42);
	const GEN_FLT x78 = 2 * x14;
	const GEN_FLT x79 = (-4 * x15) + x78 + x66;
	const GEN_FLT x80 = 2 * x6;
	const GEN_FLT x81 = x62 + (-1 * x80) + (-4 * x7);
	const GEN_FLT x82 = 2 * obj_qk * sensor_z;
	const GEN_FLT x83 = 2 * obj_qj * sensor_y;
	const GEN_FLT x84 = x83 + x82;
	const GEN_FLT x85 = (x84 * lh_qw) + (-1 * x81 * lh_qk) + (x79 * lh_qj);
	const GEN_FLT x86 = (x81 * lh_qw) + (-1 * x79 * lh_qi) + (x84 * lh_qk);
	const GEN_FLT x87 = x79 + (x86 * x51) + (-1 * x0 * x85);
	const GEN_FLT x88 = (x79 * lh_qw) + (-1 * x84 * lh_qj) + (x81 * lh_qi);
	const GEN_FLT x89 = x84 + (x0 * x88) + (-1 * x2 * x86);
	const GEN_FLT x90 = x81 + (x2 * x85) + (-1 * x88 * x51);
	const GEN_FLT x91 = (-1 * x43 * ((-1 * ((x87 * x46) + (x90 * x45)) * x47) + (x89 * x44))) +
						(-1 * ((x90 * x31) + (-1 * x87 * x36)) * x42);
	const GEN_FLT x92 = 2 * x9;
	const GEN_FLT x93 = (-1 * x92) + x68 + (-4 * x11);
	const GEN_FLT x94 = 2 * obj_qi * sensor_x;
	const GEN_FLT x95 = x82 + x94;
	const GEN_FLT x96 = (-4 * x5) + x80 + x63;
	const GEN_FLT x97 = (x96 * lh_qw) + (-1 * x95 * lh_qk) + (x93 * lh_qj);
	const GEN_FLT x98 = (x95 * lh_qw) + (-1 * x93 * lh_qi) + (x96 * lh_qk);
	const GEN_FLT x99 = x93 + (x51 * x98) + (-1 * x0 * x97);
	const GEN_FLT x100 = (x93 * lh_qw) + (-1 * x96 * lh_qj) + (x95 * lh_qi);
	const GEN_FLT x101 = x96 + (x0 * x100) + (-1 * x2 * x98);
	const GEN_FLT x102 = x95 + (x2 * x97) + (-1 * x51 * x100);
	const GEN_FLT x103 = (-1 * x43 * ((-1 * x47 * ((x99 * x46) + (x45 * x102))) + (x44 * x101))) +
						 (-1 * x42 * ((x31 * x102) + (-1 * x99 * x36)));
	const GEN_FLT x104 = x94 + x83;
	const GEN_FLT x105 = x92 + (-4 * x10) + x69;
	const GEN_FLT x106 = (-1 * x78) + x65 + (-4 * x16);
	const GEN_FLT x107 = (-1 * x105 * lh_qk) + (x106 * lh_qw) + (x104 * lh_qj);
	const GEN_FLT x108 = (x105 * lh_qw) + (-1 * x104 * lh_qi) + (x106 * lh_qk);
	const GEN_FLT x109 = x104 + (x51 * x108) + (-1 * x0 * x107);
	const GEN_FLT x110 = 2 * ((x104 * lh_qw) + (-1 * x106 * lh_qj) + (x105 * lh_qi));
	const GEN_FLT x111 = (x110 * lh_qj) + x106 + (-1 * x2 * x108);
	const GEN_FLT x112 = x105 + (x2 * x107) + (-1 * x110 * lh_qi);
	const GEN_FLT x113 = (-1 * x43 * ((-1 * ((x46 * x109) + (x45 * x112)) * x47) + (x44 * x111))) +
						 (-1 * ((x31 * x112) + (-1 * x36 * x109)) * x42);
	out[0] = x48 + (x48 * x49) + (x34 * ((-1 * x30 * x31) + (x4 * x27)));
	out[1] = x57 + (x57 * x49) + (((-1 * x54 * x31) + (x53 * x27)) * x34);
	out[2] = x61 + (x61 * x49) + (((-1 * x59 * x31) + (x58 * x27)) * x34);
	out[3] = x77 + (x77 * x49) + (((-1 * x75 * x31) + (x73 * x27)) * x34);
	out[4] = x91 + (x91 * x49) + (((-1 * x89 * x31) + (x87 * x27)) * x34);
	out[5] = x103 + (x49 * x103) + (x34 * ((-1 * x31 * x101) + (x99 * x27)));
	out[6] = x113 + (x49 * x113) + (((-1 * x31 * x111) + (x27 * x109)) * x34);
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
	const GEN_FLT x0 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x1 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x2 = sensor_y + (2 * ((x1 * obj_qk) + (-1 * x0 * obj_qi))) + obj_py;
	const GEN_FLT x3 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x4 = sensor_z + (2 * ((x3 * obj_qi) + (-1 * x1 * obj_qj))) + obj_pz;
	const GEN_FLT x5 = sensor_x + (2 * ((x0 * obj_qj) + (-1 * x3 * obj_qk))) + obj_px;
	const GEN_FLT x6 = (x5 * lh_qk) + (-1 * x4 * lh_qi) + (x2 * lh_qw);
	const GEN_FLT x7 = (x2 * lh_qi) + (-1 * x5 * lh_qj) + (x4 * lh_qw);
	const GEN_FLT x8 = x5 + (2 * ((x7 * lh_qj) + (-1 * x6 * lh_qk))) + lh_px;
	const GEN_FLT x9 = 2 * obj_qw;
	const GEN_FLT x10 = x9 * obj_qj;
	const GEN_FLT x11 = 2 * obj_qi;
	const GEN_FLT x12 = x11 * obj_qk;
	const GEN_FLT x13 = x12 + (-1 * x10);
	const GEN_FLT x14 = x11 * obj_qj;
	const GEN_FLT x15 = x9 * obj_qk;
	const GEN_FLT x16 = x15 + x14;
	const GEN_FLT x17 = -2 * pow(obj_qk, 2);
	const GEN_FLT x18 = 1 + (-2 * pow(obj_qj, 2));
	const GEN_FLT x19 = x18 + x17;
	const GEN_FLT x20 = (x19 * lh_qw) + (-1 * x16 * lh_qk) + (x13 * lh_qj);
	const GEN_FLT x21 = 2 * lh_qj;
	const GEN_FLT x22 = (x16 * lh_qw) + (-1 * x13 * lh_qi) + (x19 * lh_qk);
	const GEN_FLT x23 = 2 * lh_qi;
	const GEN_FLT x24 = x13 + (x22 * x23) + (-1 * x20 * x21);
	const GEN_FLT x25 = (x4 * lh_qj) + (-1 * x2 * lh_qk) + (x5 * lh_qw);
	const GEN_FLT x26 = x4 + (2 * ((x6 * lh_qi) + (-1 * x25 * lh_qj))) + lh_pz;
	const GEN_FLT x27 = pow(x26, 2);
	const GEN_FLT x28 = pow(x27, -1);
	const GEN_FLT x29 = x24 * x28;
	const GEN_FLT x30 = 2 * lh_qk;
	const GEN_FLT x31 = (x13 * lh_qw) + (-1 * x19 * lh_qj) + (x16 * lh_qi);
	const GEN_FLT x32 = x19 + (x31 * x21) + (-1 * x30 * x22);
	const GEN_FLT x33 = pow(x26, -1);
	const GEN_FLT x34 = pow(x8, 2);
	const GEN_FLT x35 = -1 * x26;
	const GEN_FLT x36 = 2 * pow((x27 + x34), -1) * x27 * atan2(x8, x35) * curve_1;
	const GEN_FLT x37 = x2 + (2 * ((x25 * lh_qk) + (-1 * x7 * lh_qi))) + lh_py;
	const GEN_FLT x38 = x16 + (x30 * x20) + (-1 * x31 * x23);
	const GEN_FLT x39 = x27 + pow(x37, 2);
	const GEN_FLT x40 = pow(x39, -1);
	const GEN_FLT x41 = x40 * x27;
	const GEN_FLT x42 = pow((1 + (-1 * x40 * x34 * pow(tilt_1, 2))), -1.0 / 2.0);
	const GEN_FLT x43 = pow(x39, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x44 = 2 * x37;
	const GEN_FLT x45 = 2 * x26;
	const GEN_FLT x46 = 1.0 / 2.0 * x8 * pow(x39, -3.0 / 2.0) * tilt_1;
	const GEN_FLT x47 = (-1 * x42 * ((-1 * ((x45 * x24) + (x44 * x38)) * x46) + (x43 * x32))) +
						(-1 * ((x33 * x38) + (-1 * x37 * x29)) * x41);
	const GEN_FLT x48 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x37, x35)) + (-1 * asin(x8 * x43)) + (-1 * phase_1) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x49 = 2 * obj_qk * obj_qj;
	const GEN_FLT x50 = x9 * obj_qi;
	const GEN_FLT x51 = x50 + x49;
	const GEN_FLT x52 = -2 * pow(obj_qi, 2);
	const GEN_FLT x53 = 1 + x17 + x52;
	const GEN_FLT x54 = x14 + (-1 * x15);
	const GEN_FLT x55 = (x54 * lh_qw) + (-1 * x53 * lh_qk) + (x51 * lh_qj);
	const GEN_FLT x56 = (x53 * lh_qw) + (-1 * x51 * lh_qi) + (x54 * lh_qk);
	const GEN_FLT x57 = x51 + (x56 * x23) + (-1 * x55 * x21);
	const GEN_FLT x58 = x8 * x28;
	const GEN_FLT x59 = (x51 * lh_qw) + (-1 * x54 * lh_qj) + (x53 * lh_qi);
	const GEN_FLT x60 = x54 + (x59 * x21) + (-1 * x56 * x30);
	const GEN_FLT x61 = x37 * x28;
	const GEN_FLT x62 = x53 + (x55 * x30) + (-1 * x59 * x23);
	const GEN_FLT x63 = (-1 * x42 * ((-1 * ((x57 * x45) + (x62 * x44)) * x46) + (x60 * x43))) +
						(-1 * ((x62 * x33) + (-1 * x61 * x57)) * x41);
	const GEN_FLT x64 = x18 + x52;
	const GEN_FLT x65 = x49 + (-1 * x50);
	const GEN_FLT x66 = x10 + x12;
	const GEN_FLT x67 = (x66 * lh_qw) + (-1 * x65 * lh_qk) + (x64 * lh_qj);
	const GEN_FLT x68 = (x65 * lh_qw) + (-1 * x64 * lh_qi) + (x66 * lh_qk);
	const GEN_FLT x69 = x64 + (x68 * x23) + (-1 * x67 * x21);
	const GEN_FLT x70 = (x64 * lh_qw) + (-1 * x66 * lh_qj) + (x65 * lh_qi);
	const GEN_FLT x71 = x66 + (x70 * x21) + (-1 * x68 * x30);
	const GEN_FLT x72 = x65 + (x67 * x30) + (-1 * x70 * x23);
	const GEN_FLT x73 = (-1 * x42 * ((-1 * ((x69 * x45) + (x72 * x44)) * x46) + (x71 * x43))) +
						(-1 * ((x72 * x33) + (-1 * x61 * x69)) * x41);
	out[0] = x47 + (x47 * x48) + (x36 * ((-1 * x32 * x33) + (x8 * x29)));
	out[1] = x63 + (x63 * x48) + (((-1 * x60 * x33) + (x58 * x57)) * x36);
	out[2] = x73 + (x73 * x48) + (((-1 * x71 * x33) + (x69 * x58)) * x36);
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
	const GEN_FLT x0 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x1 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x2 = 2 * ((x1 * obj_qj) + (-1 * x0 * obj_qk));
	const GEN_FLT x3 = sensor_x + x2 + obj_px;
	const GEN_FLT x4 = x3 * lh_qw;
	const GEN_FLT x5 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x6 = 2 * ((x5 * obj_qk) + (-1 * x1 * obj_qi));
	const GEN_FLT x7 = sensor_y + x6 + obj_py;
	const GEN_FLT x8 = x7 * lh_qk;
	const GEN_FLT x9 = 2 * ((x0 * obj_qi) + (-1 * x5 * obj_qj));
	const GEN_FLT x10 = sensor_z + x9 + obj_pz;
	const GEN_FLT x11 = x10 * lh_qj;
	const GEN_FLT x12 = (-1 * x8) + x11 + x4;
	const GEN_FLT x13 = x7 * lh_qw;
	const GEN_FLT x14 = x10 * lh_qi;
	const GEN_FLT x15 = x3 * lh_qk;
	const GEN_FLT x16 = x15 + (-1 * x14) + x13;
	const GEN_FLT x17 = x10 + (2 * ((x16 * lh_qi) + (-1 * x12 * lh_qj))) + lh_pz;
	const GEN_FLT x18 = pow(x17, 2);
	const GEN_FLT x19 = x10 * lh_qw;
	const GEN_FLT x20 = x3 * lh_qj;
	const GEN_FLT x21 = x7 * lh_qi;
	const GEN_FLT x22 = x21 + (-1 * x20) + x19;
	const GEN_FLT x23 = x7 + (2 * ((x12 * lh_qk) + (-1 * x22 * lh_qi))) + lh_py;
	const GEN_FLT x24 = pow(x23, 2) + x18;
	const GEN_FLT x25 = pow(x24, -1);
	const GEN_FLT x26 = x3 + (2 * ((x22 * lh_qj) + (-1 * x16 * lh_qk))) + lh_px;
	const GEN_FLT x27 = pow(x26, 2);
	const GEN_FLT x28 = pow((1 + (-1 * x25 * x27 * pow(tilt_1, 2))), -1.0 / 2.0);
	const GEN_FLT x29 = pow(x24, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x30 = x28 * x29;
	const GEN_FLT x31 = 2 * x17;
	const GEN_FLT x32 = -1 * x17;
	const GEN_FLT x33 = pow((x18 + x27), -1) * atan2(x26, x32) * curve_1;
	const GEN_FLT x34 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x23, x32)) + (-1 * asin(x29 * x26)) + (-1 * phase_1) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x35 = pow(x24, -3.0 / 2.0) * x26 * tilt_1;
	const GEN_FLT x36 = x35 * x28;
	const GEN_FLT x37 = (x36 * x23) + (-1 * x25 * x17);
	const GEN_FLT x38 = 2 * x33;
	const GEN_FLT x39 = (x36 * x17) + (x25 * x23);
	const GEN_FLT x40 = 2 * x20;
	const GEN_FLT x41 = (2 * x21) + (-1 * x40);
	const GEN_FLT x42 = pow(x18, -1);
	const GEN_FLT x43 = x42 * x26;
	const GEN_FLT x44 = 2 * x8;
	const GEN_FLT x45 = (2 * x11) + (-1 * x44);
	const GEN_FLT x46 = pow(x17, -1);
	const GEN_FLT x47 = x38 * x18;
	const GEN_FLT x48 = x42 * x23;
	const GEN_FLT x49 = 2 * x14;
	const GEN_FLT x50 = (2 * x15) + (-1 * x49);
	const GEN_FLT x51 = x25 * x18;
	const GEN_FLT x52 = 2 * x23;
	const GEN_FLT x53 = 1.0 / 2.0 * x35;
	const GEN_FLT x54 = (-1 * x28 * ((-1 * ((x41 * x31) + (x50 * x52)) * x53) + (x45 * x29))) +
						(-1 * ((x50 * x46) + (-1 * x41 * x48)) * x51);
	const GEN_FLT x55 = (-1 * sensor_z) + (-1 * x9) + (-1 * obj_pz);
	const GEN_FLT x56 = 2 * lh_qi;
	const GEN_FLT x57 = 2 * x13;
	const GEN_FLT x58 = x50 + x57 + (x56 * x55);
	const GEN_FLT x59 = 2 * lh_qk;
	const GEN_FLT x60 = 2 * lh_qj;
	const GEN_FLT x61 = (x7 * x60) + (-1 * x55 * x59);
	const GEN_FLT x62 = 2 * x19;
	const GEN_FLT x63 = (-1 * x62) + x40 + (-4 * x21);
	const GEN_FLT x64 = (-1 * x28 * ((-1 * ((x58 * x31) + (x63 * x52)) * x53) + (x61 * x29))) +
						(-1 * ((x63 * x46) + (-1 * x58 * x48)) * x51);
	const GEN_FLT x65 = 2 * x4;
	const GEN_FLT x66 = (-1 * x65) + x44 + (-4 * x11);
	const GEN_FLT x67 = (-1 * sensor_x) + (-1 * x2) + (-1 * obj_px);
	const GEN_FLT x68 = x41 + x62 + (x60 * x67);
	const GEN_FLT x69 = (x59 * x10) + (-1 * x67 * x56);
	const GEN_FLT x70 = (-1 * x28 * ((-1 * ((x66 * x31) + (x69 * x52)) * x53) + (x68 * x29))) +
						(-1 * ((x69 * x46) + (-1 * x66 * x48)) * x51);
	const GEN_FLT x71 = (-1 * sensor_y) + (-1 * x6) + (-1 * obj_py);
	const GEN_FLT x72 = (x3 * x56) + (-1 * x71 * x60);
	const GEN_FLT x73 = (-1 * x57) + x49 + (-4 * x15);
	const GEN_FLT x74 = x45 + x65 + (x71 * x59);
	const GEN_FLT x75 = (-1 * x28 * ((-1 * ((x72 * x31) + (x74 * x52)) * x53) + (x73 * x29))) +
						(-1 * ((x74 * x46) + (-1 * x72 * x48)) * x51);
	out[0] = (-1 * x30 * x34) + (-1 * x31 * x33) + (-1 * x30);
	out[1] = x37 + (x34 * x37);
	out[2] = x39 + (x34 * x39) + (x38 * x26);
	out[3] = x54 + (x54 * x34) + (((-1 * x45 * x46) + (x41 * x43)) * x47);
	out[4] = (x64 * x34) + x64 + (((-1 * x61 * x46) + (x58 * x43)) * x47);
	out[5] = x70 + (x70 * x34) + (((-1 * x68 * x46) + (x66 * x43)) * x47);
	out[6] = x75 + (x75 * x34) + (((-1 * x73 * x46) + (x72 * x43)) * x47);
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
	const GEN_FLT x0 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x1 = (obj_qi * sensor_y) + (obj_qw * sensor_z) + (-1 * obj_qj * sensor_x);
	const GEN_FLT x2 = sensor_x + (2 * ((x1 * obj_qj) + (-1 * x0 * obj_qk))) + obj_px;
	const GEN_FLT x3 = (obj_qj * sensor_z) + (-1 * obj_qk * sensor_y) + (obj_qw * sensor_x);
	const GEN_FLT x4 = sensor_y + (2 * ((x3 * obj_qk) + (-1 * x1 * obj_qi))) + obj_py;
	const GEN_FLT x5 = sensor_z + (2 * ((x0 * obj_qi) + (-1 * x3 * obj_qj))) + obj_pz;
	const GEN_FLT x6 = (x5 * lh_qj) + (-1 * x4 * lh_qk) + (x2 * lh_qw);
	const GEN_FLT x7 = (x2 * lh_qk) + (-1 * x5 * lh_qi) + (x4 * lh_qw);
	const GEN_FLT x8 = x5 + (2 * ((x7 * lh_qi) + (-1 * x6 * lh_qj))) + lh_pz;
	const GEN_FLT x9 = (x4 * lh_qi) + (-1 * x2 * lh_qj) + (x5 * lh_qw);
	const GEN_FLT x10 = (2 * ((x6 * lh_qk) + (-1 * x9 * lh_qi))) + x4 + lh_py;
	const GEN_FLT x11 = pow(x10, 2) + pow(x8, 2);
	const GEN_FLT x12 = x2 + (2 * ((x9 * lh_qj) + (-1 * x7 * lh_qk))) + lh_px;
	const GEN_FLT x13 = x12 * pow(x11, -1.0 / 2.0);
	const GEN_FLT x14 = -1 * x8;
	const GEN_FLT x15 =
		1.5707963267949 + (-1 * atan2(-1 * x10, x14)) + (-1 * asin(x13 * tilt_1)) + (-1 * phase_1) + gibPhase_1;
	const GEN_FLT x16 = sin(x15) * gibMag_1;
	const GEN_FLT x17 = x13 * pow((1 + (-1 * pow(x12, 2) * pow(x11, -1) * pow(tilt_1, 2))), -1.0 / 2.0);
	out[0] = -1 + (-1 * x16);
	out[1] = (-1 * x17 * x16) + (-1 * x17);
	out[2] = pow(atan2(x12, x14), 2);
	out[3] = x16;
	out[4] = -1 * cos(x15);
	out[5] = 0;
	out[6] = 0;
}
