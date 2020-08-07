#pragma once
#include "common.h"
/** Applying function <function imu_rot_f at 0x7fb707230f80> */
static inline void gen_imu_rot_f(FLT *out, const FLT time, const FLT *imu_rot) {
	const GEN_FLT obj_qw = imu_rot[0];
	const GEN_FLT obj_qi = imu_rot[1];
	const GEN_FLT obj_qj = imu_rot[2];
	const GEN_FLT obj_qk = imu_rot[3];
	const GEN_FLT aa_x = imu_rot[4];
	const GEN_FLT aa_y = imu_rot[5];
	const GEN_FLT aa_z = imu_rot[6];
	const GEN_FLT x0 = pow(time, 2);
	const GEN_FLT x1 = x0 * pow(aa_z, 2);
	const GEN_FLT x2 =
		((1e-20 < pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2))
			 ? (sqrt(pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2)))
			 : (1e-10));
	const GEN_FLT x3 = 0.5 * x2;
	const GEN_FLT x4 = sin(x3);
	const GEN_FLT x5 = pow(x4, 2) / pow(x2, 2);
	const GEN_FLT x6 = x0 * pow(aa_y, 2);
	const GEN_FLT x7 = x0 * pow(aa_x, 2);
	const GEN_FLT x8 = cos(x3);
	const GEN_FLT x9 = pow(x1 * x5 + x5 * x7 + x6 * x5 + pow(x8, 2), -1.0 / 2.0);
	const GEN_FLT x10 = x4 * x9 * time / x2;
	const GEN_FLT x11 = x10 * aa_z;
	const GEN_FLT x12 = x10 * aa_y;
	const GEN_FLT x13 = x10 * aa_x;
	const GEN_FLT x14 = x8 * x9;
	out[0] = -x11 * obj_qk - x12 * obj_qj - x13 * obj_qi + x14 * obj_qw;
	out[1] = -x11 * obj_qj + x12 * obj_qk + x13 * obj_qw + x14 * obj_qi;
	out[2] = x11 * obj_qi + x12 * obj_qw - x13 * obj_qk + x14 * obj_qj;
	out[3] = x11 * obj_qw - x12 * obj_qi + x13 * obj_qj + x14 * obj_qk;
	out[4] = aa_x;
	out[5] = aa_y;
	out[6] = aa_z;
}

// Jacobian of imu_rot_f wrt [time]
static inline void gen_imu_rot_f_jac_time(FLT *out, const FLT time, const FLT *imu_rot) {
	const GEN_FLT obj_qw = imu_rot[0];
	const GEN_FLT obj_qi = imu_rot[1];
	const GEN_FLT obj_qj = imu_rot[2];
	const GEN_FLT obj_qk = imu_rot[3];
	const GEN_FLT aa_x = imu_rot[4];
	const GEN_FLT aa_y = imu_rot[5];
	const GEN_FLT aa_z = imu_rot[6];
	const GEN_FLT x0 =
		((1e-20 < pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2))
			 ? (sqrt(pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2)))
			 : (1e-10));
	const GEN_FLT x1 = 0.5 * x0;
	const GEN_FLT x2 = cos(x1);
	const GEN_FLT x3 = pow(time, 2);
	const GEN_FLT x4 = pow(aa_z, 2);
	const GEN_FLT x5 = x4 * x3;
	const GEN_FLT x6 = sin(x1);
	const GEN_FLT x7 = pow(x6, 2);
	const GEN_FLT x8 = pow(x0, -2);
	const GEN_FLT x9 = x8 * x7;
	const GEN_FLT x10 = pow(aa_y, 2);
	const GEN_FLT x11 = x3 * x10;
	const GEN_FLT x12 = pow(aa_x, 2);
	const GEN_FLT x13 = x3 * x12;
	const GEN_FLT x14 = x5 * x9 + x9 * x11 + x9 * x13 + pow(x2, 2);
	const GEN_FLT x15 =
		((1e-20 < pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2))
			 ? ((1.0 / 2.0) * (2 * time * pow(aa_x, 2) + 2 * time * pow(aa_y, 2) + 2 * time * pow(aa_z, 2)) /
				sqrt(pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2)))
			 : (0));
	const GEN_FLT x16 = x6 * x15;
	const GEN_FLT x17 = 1.0 * x2 * x16;
	const GEN_FLT x18 = x8 * x17;
	const GEN_FLT x19 = 2 * time;
	const GEN_FLT x20 = x10 * x19;
	const GEN_FLT x21 = 2 * x7 * x15 / pow(x0, 3);
	const GEN_FLT x22 = x12 * x19;
	const GEN_FLT x23 = x4 * x19;
	const GEN_FLT x24 =
		(1.0 / 2.0) *
		(-x17 + x11 * x18 + x13 * x18 - x21 * x11 - x21 * x13 + x5 * x18 - x5 * x21 + x9 * x20 + x9 * x22 + x9 * x23) /
		pow(x14, 3.0 / 2.0);
	const GEN_FLT x25 = x2 * x24;
	const GEN_FLT x26 = 0.5 * x16;
	const GEN_FLT x27 = pow(x14, -1.0 / 2.0);
	const GEN_FLT x28 = x27 * obj_qw;
	const GEN_FLT x29 = x27 * obj_qi;
	const GEN_FLT x30 = pow(x0, -1);
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = x31 * aa_x;
	const GEN_FLT x33 = 0.5 * x2 * x30 * x15;
	const GEN_FLT x34 = x33 * time;
	const GEN_FLT x35 = x34 * x29;
	const GEN_FLT x36 = x8 * x16;
	const GEN_FLT x37 = x27 * obj_qk;
	const GEN_FLT x38 = x37 * time;
	const GEN_FLT x39 = x38 * aa_z;
	const GEN_FLT x40 = x27 * obj_qj;
	const GEN_FLT x41 = x31 * aa_y;
	const GEN_FLT x42 = x31 * aa_z;
	const GEN_FLT x43 = x36 * aa_y;
	const GEN_FLT x44 = x43 * time;
	const GEN_FLT x45 = x24 * time;
	const GEN_FLT x46 = x45 * x32;
	const GEN_FLT x47 = x40 * x34;
	const GEN_FLT x48 = x42 * x45;
	const GEN_FLT x49 = x41 * x45;
	const GEN_FLT x50 = x36 * time;
	const GEN_FLT x51 = x50 * aa_x;
	const GEN_FLT x52 = x50 * aa_z;
	const GEN_FLT x53 = x34 * x28;
	const GEN_FLT x54 = x38 * aa_x;
	out[0] = -x25 * obj_qw - x28 * x26 - x32 * x29 - x33 * x39 - x35 * aa_x + x36 * x39 - x40 * x41 + x40 * x44 -
			 x42 * x37 + x46 * obj_qi - x47 * aa_y + x48 * obj_qk + x49 * obj_qj + x51 * x29;
	out[1] = -x25 * obj_qi - x29 * x26 + x32 * x28 - x40 * x42 + x41 * x37 - x43 * x38 - x46 * obj_qw - x47 * aa_z +
			 x48 * obj_qj - x49 * obj_qk - x51 * x28 + x52 * x40 + x53 * aa_x + x33 * x38 * aa_y;
	out[2] = -x25 * obj_qj - x32 * x37 + x35 * aa_z - x40 * x26 + x41 * x28 + x42 * x29 - x44 * x28 + x46 * obj_qk -
			 x48 * obj_qi - x49 * obj_qw - x52 * x29 + x53 * aa_y - x54 * x33 + x54 * x36;
	out[3] = -x25 * obj_qk - x35 * aa_y - x37 * x26 + x40 * x32 - x41 * x29 + x42 * x28 + x44 * x29 - x46 * obj_qj +
			 x47 * aa_x - x48 * obj_qw + x49 * obj_qi - x51 * x40 - x52 * x28 + x53 * aa_z;
	out[4] = 0;
	out[5] = 0;
	out[6] = 0;
}

// Jacobian of imu_rot_f wrt [obj_qw, obj_qi, obj_qj, obj_qk, aa_x, aa_y, aa_z]
static inline void gen_imu_rot_f_jac_imu_rot(FLT *out, const FLT time, const FLT *imu_rot) {
	const GEN_FLT obj_qw = imu_rot[0];
	const GEN_FLT obj_qi = imu_rot[1];
	const GEN_FLT obj_qj = imu_rot[2];
	const GEN_FLT obj_qk = imu_rot[3];
	const GEN_FLT aa_x = imu_rot[4];
	const GEN_FLT aa_y = imu_rot[5];
	const GEN_FLT aa_z = imu_rot[6];
	const GEN_FLT x0 = pow(time, 2);
	const GEN_FLT x1 = x0 * pow(aa_z, 2);
	const GEN_FLT x2 =
		((1e-20 < pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2))
			 ? (sqrt(pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2)))
			 : (1e-10));
	const GEN_FLT x3 = 0.5 * x2;
	const GEN_FLT x4 = sin(x3);
	const GEN_FLT x5 = pow(x4, 2);
	const GEN_FLT x6 = pow(x2, -2);
	const GEN_FLT x7 = x6 * x5;
	const GEN_FLT x8 = x0 * pow(aa_y, 2);
	const GEN_FLT x9 = x0 * pow(aa_x, 2);
	const GEN_FLT x10 = cos(x3);
	const GEN_FLT x11 = x1 * x7 + x7 * x9 + x8 * x7 + pow(x10, 2);
	const GEN_FLT x12 = pow(x11, -1.0 / 2.0);
	const GEN_FLT x13 = x12 * x10;
	const GEN_FLT x14 =
		((1e-20 < pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2)) ? (0) : (0));
	const GEN_FLT x15 = 1.0 * x10;
	const GEN_FLT x16 = x4 * x15;
	const GEN_FLT x17 = x14 * x16;
	const GEN_FLT x18 = x6 * x9;
	const GEN_FLT x19 = x6 * x8;
	const GEN_FLT x20 = 2 * x5 / pow(x2, 3);
	const GEN_FLT x21 = x8 * x20;
	const GEN_FLT x22 = x9 * x20;
	const GEN_FLT x23 = x1 * x6;
	const GEN_FLT x24 = x1 * x20;
	const GEN_FLT x25 = -x17 + x18 * x17 + x19 * x17 - x21 * x14 - x22 * x14 + x23 * x17 - x24 * x14;
	const GEN_FLT x26 = (1.0 / 2.0) / pow(x11, 3.0 / 2.0);
	const GEN_FLT x27 = x26 * x10;
	const GEN_FLT x28 = x27 * obj_qw;
	const GEN_FLT x29 = x4 * x12;
	const GEN_FLT x30 = x6 * time;
	const GEN_FLT x31 = x30 * x29;
	const GEN_FLT x32 = x31 * aa_z;
	const GEN_FLT x33 = x14 * obj_qk;
	const GEN_FLT x34 = time / x2;
	const GEN_FLT x35 = x34 * aa_z;
	const GEN_FLT x36 = 0.5 * obj_qk;
	const GEN_FLT x37 = x36 * x13;
	const GEN_FLT x38 = x35 * x37;
	const GEN_FLT x39 = x14 * obj_qj;
	const GEN_FLT x40 = 0.5 * x13;
	const GEN_FLT x41 = x40 * x34;
	const GEN_FLT x42 = x41 * aa_y;
	const GEN_FLT x43 = x4 * x26;
	const GEN_FLT x44 = x43 * x25;
	const GEN_FLT x45 = x34 * aa_y;
	const GEN_FLT x46 = x44 * x45;
	const GEN_FLT x47 = x44 * obj_qk;
	const GEN_FLT x48 = x31 * aa_y;
	const GEN_FLT x49 = x31 * aa_x;
	const GEN_FLT x50 = x49 * x14;
	const GEN_FLT x51 = x34 * aa_x;
	const GEN_FLT x52 = x51 * obj_qi;
	const GEN_FLT x53 = x40 * x14;
	const GEN_FLT x54 = 0.5 * x29;
	const GEN_FLT x55 = x54 * x14;
	const GEN_FLT x56 = -x25 * x28 + x32 * x33 - x38 * x14 - x42 * x39 + x46 * obj_qj + x47 * x35 + x48 * x39 +
						x50 * obj_qi + x52 * x44 - x53 * x52 - x55 * obj_qw;
	const GEN_FLT x57 = x51 * x29;
	const GEN_FLT x58 = x40 * x39;
	const GEN_FLT x59 = x35 * obj_qj;
	const GEN_FLT x60 = x25 * x27;
	const GEN_FLT x61 = x14 * aa_y;
	const GEN_FLT x62 = x34 * x37;
	const GEN_FLT x63 = x51 * obj_qw;
	const GEN_FLT x64 = x32 * x39 - x45 * x47 - x48 * x33 - x50 * obj_qw - x55 * obj_qi - x58 * x35 + x59 * x44 -
						x60 * obj_qi + x61 * x62 - x63 * x44 + x63 * x53;
	const GEN_FLT x65 = x34 * x29;
	const GEN_FLT x66 = x65 * aa_y;
	const GEN_FLT x67 = aa_z * obj_qi;
	const GEN_FLT x68 = x67 * x31;
	const GEN_FLT x69 = x67 * x41;
	const GEN_FLT x70 = x67 * x34;
	const GEN_FLT x71 = x34 * obj_qw;
	const GEN_FLT x72 = x71 * x40;
	const GEN_FLT x73 = x31 * obj_qw;
	const GEN_FLT x74 = x71 * aa_y;
	const GEN_FLT x75 = x51 * x37;
	const GEN_FLT x76 = x49 * x33 + x51 * x47 - x55 * obj_qj - x60 * obj_qj - x68 * x14 + x69 * x14 - x70 * x44 +
						x72 * x61 - x73 * x61 - x74 * x44 - x75 * x14;
	const GEN_FLT x77 = x65 * aa_z;
	const GEN_FLT x78 = x71 * aa_z;
	const GEN_FLT x79 = x61 * obj_qi;
	const GEN_FLT x80 = x73 * aa_z;
	const GEN_FLT x81 = x51 * obj_qj;
	const GEN_FLT x82 = x46 * obj_qi - x49 * x39 + x51 * x58 - x55 * obj_qk - x60 * obj_qk - x78 * x44 + x78 * x53 +
						x79 * x31 - x79 * x41 - x80 * x14 - x81 * x44;
	const GEN_FLT x83 = -x57;
	const GEN_FLT x84 = -x66;
	const GEN_FLT x85 = -x77;
	const GEN_FLT x86 = 2 * x0 * x7;
	const GEN_FLT x87 =
		((1e-20 < pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2))
			 ? (pow(time, 2) * aa_x /
				sqrt(pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2)))
			 : (0));
	const GEN_FLT x88 = x87 * x16;
	const GEN_FLT x89 = -x88 + x86 * aa_x - x87 * x21 - x87 * x22 - x87 * x24 + x88 * x18 + x88 * x19 + x88 * x23;
	const GEN_FLT x90 = x89 * x43;
	const GEN_FLT x91 = x87 * obj_qk;
	const GEN_FLT x92 = x87 * x54;
	const GEN_FLT x93 = x35 * obj_qk;
	const GEN_FLT x94 = x87 * obj_qj;
	const GEN_FLT x95 = x87 * obj_qi;
	const GEN_FLT x96 = x51 * x40;
	const GEN_FLT x97 = x90 * x45;
	const GEN_FLT x98 = x87 * x49;
	const GEN_FLT x99 = x65 * obj_qi;
	const GEN_FLT x100 = -x99;
	const GEN_FLT x101 = x87 * x40;
	const GEN_FLT x102 = x89 * x27;
	const GEN_FLT x103 = x54 * obj_qi;
	const GEN_FLT x104 = x45 * x37;
	const GEN_FLT x105 = x65 * obj_qw;
	const GEN_FLT x106 = x87 * x73;
	const GEN_FLT x107 = x65 * obj_qk;
	const GEN_FLT x108 = -x107;
	const GEN_FLT x109 = x51 * x90;
	const GEN_FLT x110 = x65 * obj_qj;
	const GEN_FLT x111 =
		((1e-20 < pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2))
			 ? (pow(time, 2) * aa_y /
				sqrt(pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2)))
			 : (0));
	const GEN_FLT x112 = x54 * x111;
	const GEN_FLT x113 = x32 * x111;
	const GEN_FLT x114 = x16 * x111;
	const GEN_FLT x115 =
		-x114 + x18 * x114 + x19 * x114 - x21 * x111 - x22 * x111 + x23 * x114 - x24 * x111 + x86 * aa_y;
	const GEN_FLT x116 = x43 * x115;
	const GEN_FLT x117 = x111 * aa_y;
	const GEN_FLT x118 = x31 * x117;
	const GEN_FLT x119 = x40 * x111;
	const GEN_FLT x120 = x27 * x115;
	const GEN_FLT x121 = x41 * x117;
	const GEN_FLT x122 = x45 * x116;
	const GEN_FLT x123 = -x110;
	const GEN_FLT x124 = x49 * x111;
	const GEN_FLT x125 = x51 * x116;
	const GEN_FLT x126 =
		((1e-20 < pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2))
			 ? (pow(time, 2) * aa_z /
				sqrt(pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2)))
			 : (0));
	const GEN_FLT x127 = x4 * x126;
	const GEN_FLT x128 = x12 * x127;
	const GEN_FLT x129 = 0.5 * x128;
	const GEN_FLT x130 = x30 * x128;
	const GEN_FLT x131 = x130 * obj_qk;
	const GEN_FLT x132 = x42 * x126;
	const GEN_FLT x133 = x40 * x126;
	const GEN_FLT x134 = x15 * x127;
	const GEN_FLT x135 = x6 * x134;
	const GEN_FLT x136 = -x134 + x1 * x135 - x21 * x126 - x22 * x126 - x24 * x126 + x8 * x135 + x86 * aa_z + x9 * x135;
	const GEN_FLT x137 = x43 * x136;
	const GEN_FLT x138 = x130 * obj_qj;
	const GEN_FLT x139 = x130 * obj_qi;
	const GEN_FLT x140 = x45 * x137;
	const GEN_FLT x141 = x27 * x136;
	const GEN_FLT x142 = x130 * obj_qw;
	const GEN_FLT x143 = x51 * x137;
	out[0] = x13 + x56;
	out[1] = x57 + x64;
	out[2] = x66 + x76;
	out[3] = x77 + x82;
	out[4] = 0;
	out[5] = 0;
	out[6] = 0;
	out[7] = x56 + x83;
	out[8] = x13 + x64;
	out[9] = x76 + x77;
	out[10] = x82 + x84;
	out[11] = 0;
	out[12] = 0;
	out[13] = 0;
	out[14] = x56 + x84;
	out[15] = x64 + x85;
	out[16] = x13 + x76;
	out[17] = x57 + x82;
	out[18] = 0;
	out[19] = 0;
	out[20] = 0;
	out[21] = x56 + x85;
	out[22] = x64 + x66;
	out[23] = x76 + x83;
	out[24] = x13 + x82;
	out[25] = 0;
	out[26] = 0;
	out[27] = 0;
	out[28] = x100 + x52 * x90 - x87 * x38 - x89 * x28 + x91 * x32 - x92 * obj_qw + x93 * x90 - x94 * x42 + x94 * x48 -
			  x96 * x95 + x97 * obj_qj + x98 * obj_qi;
	out[29] = x105 - x102 * obj_qi + x59 * x90 + x63 * x101 - x63 * x90 - x87 * x103 + x87 * x104 - x91 * x48 +
			  x94 * x32 - x97 * obj_qk - x98 * obj_qw - x94 * x40 * x35;
	out[30] = x108 - x102 * obj_qj - x106 * aa_y + x109 * obj_qk + x70 * x101 - x70 * x90 + x74 * x101 - x74 * x90 -
			  x87 * x68 - x87 * x75 - x92 * obj_qj + x98 * obj_qk;
	out[31] = x110 - x102 * obj_qk - x106 * aa_z - x109 * obj_qj + x78 * x101 - x78 * x90 - x92 * obj_qk - x95 * x42 +
			  x95 * x48 + x96 * x94 + x97 * obj_qi - x98 * obj_qj;
	out[32] = 1;
	out[33] = 0;
	out[34] = 0;
	out[35] = x123 - x112 * obj_qw + x113 * obj_qk + x118 * obj_qj - x120 * obj_qw - x121 * obj_qj + x122 * obj_qj +
			  x124 * obj_qi - x38 * x111 + x52 * x116 - x52 * x119 + x93 * x116;
	out[36] = x107 - x103 * x111 + x113 * obj_qj - x118 * obj_qk - x120 * obj_qi - x122 * obj_qk - x124 * obj_qw +
			  x59 * x116 - x59 * x119 + x62 * x117 - x63 * x116 + x63 * x119;
	out[37] = x105 - x112 * obj_qj - x120 * obj_qj + x124 * obj_qk + x125 * obj_qk - x68 * x111 + x69 * x111 -
			  x70 * x116 + x72 * x117 - x73 * x117 - x74 * x116 - x75 * x111;
	out[38] = x100 - x112 * obj_qk + x118 * obj_qi - x120 * obj_qk - x121 * obj_qi + x122 * obj_qi - x124 * obj_qj -
			  x125 * obj_qj - x78 * x116 - x80 * x111 + x81 * x119 + x72 * x111 * aa_z;
	out[39] = 0;
	out[40] = 1;
	out[41] = 0;
	out[42] = x108 - x129 * obj_qw + x131 * aa_z - x132 * obj_qj + x138 * aa_y + x139 * aa_x + x140 * obj_qj -
			  x28 * x136 - x38 * x126 - x52 * x133 + x52 * x137 + x93 * x137;
	out[43] = x123 + x104 * x126 - x129 * obj_qi - x131 * aa_y + x138 * aa_z - x140 * obj_qk - x141 * obj_qi -
			  x142 * aa_x - x59 * x133 + x59 * x137 + x63 * x133 - x63 * x137;
	out[44] = x99 - x129 * obj_qj + x131 * aa_x - x141 * obj_qj - x142 * aa_y + x143 * obj_qk - x67 * x130 +
			  x69 * x126 - x70 * x137 + x74 * x133 - x74 * x137 - x75 * x126;
	out[45] = x105 - x132 * obj_qi - x138 * aa_x + x139 * aa_y + x140 * obj_qi - x141 * obj_qk - x142 * aa_z -
			  x143 * obj_qj - x36 * x128 + x78 * x133 - x78 * x137 + x81 * x133;
	out[46] = 0;
	out[47] = 0;
	out[48] = 1;
}

/** Applying function <function imu_rot_f_aa at 0x7fb6d8f63a70> */
static inline void gen_imu_rot_f_aa(FLT *out, const FLT time, const FLT *imu_rot_aa) {
	const GEN_FLT aa_x = imu_rot_aa[0];
	const GEN_FLT aa_y = imu_rot_aa[1];
	const GEN_FLT aa_z = imu_rot_aa[2];
	const GEN_FLT lh_qi = imu_rot_aa[3];
	const GEN_FLT lh_qj = imu_rot_aa[4];
	const GEN_FLT lh_qk = imu_rot_aa[5];
	const GEN_FLT x0 =
		((1e-20 < pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)) ? (sqrt(pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)))
															  : (1e-10));
	const GEN_FLT x1 = 0.5 * x0;
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x2 / x0;
	const GEN_FLT x4 =
		((1e-20 < pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2))
			 ? (sqrt(pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2)))
			 : (1e-10));
	const GEN_FLT x5 = 0.5 * x4;
	const GEN_FLT x6 = sin(x5);
	const GEN_FLT x7 = pow(aa_z, 2);
	const GEN_FLT x8 = pow(x2, 2) / pow(x0, 2);
	const GEN_FLT x9 = pow(aa_y, 2);
	const GEN_FLT x10 = pow(aa_x, 2);
	const GEN_FLT x11 = cos(x1);
	const GEN_FLT x12 = pow(time, 2);
	const GEN_FLT x13 = x12 * pow(lh_qk, 2);
	const GEN_FLT x14 = pow(x6, 2) / pow(x4, 2);
	const GEN_FLT x15 = x12 * pow(lh_qj, 2);
	const GEN_FLT x16 = x12 * pow(lh_qi, 2);
	const GEN_FLT x17 = cos(x5);
	const GEN_FLT x18 =
		1 / (sqrt(x8 * x10 + x8 * x7 + x8 * x9 + pow(x11, 2)) * sqrt(x14 * x13 + x14 * x16 + x15 * x14 + pow(x17, 2)));
	const GEN_FLT x19 = x6 * x18 * time / x4;
	const GEN_FLT x20 = x3 * x19;
	const GEN_FLT x21 = x20 * lh_qj;
	const GEN_FLT x22 = x11 * x19;
	const GEN_FLT x23 = x20 * lh_qk;
	const GEN_FLT x24 = x18 * x17;
	const GEN_FLT x25 = x3 * x24;
	const GEN_FLT x26 = x21 * aa_z + x22 * lh_qi - x23 * aa_y + x25 * aa_x;
	const GEN_FLT x27 = x20 * lh_qi;
	const GEN_FLT x28 = x22 * lh_qj + x23 * aa_x + x25 * aa_y - x27 * aa_z;
	const GEN_FLT x29 = -x21 * aa_y - x23 * aa_z + x24 * x11 - x27 * aa_x;
	const GEN_FLT x30 = 1e-10 + pow(x26, 2) + pow(x28, 2) + pow(x29, 2);
	const GEN_FLT x31 = 2 * atan2(x30, x29) / x30;
	out[0] = x31 * x26;
	out[1] = x31 * x28;
	out[2] = x31 * (-x21 * aa_x + x22 * lh_qk + x25 * aa_z + x27 * aa_y);
	out[3] = lh_qi;
	out[4] = lh_qj;
	out[5] = lh_qk;
}

// Jacobian of imu_rot_f_aa wrt [time]
static inline void gen_imu_rot_f_aa_jac_time(FLT *out, const FLT time, const FLT *imu_rot_aa) {
	const GEN_FLT aa_x = imu_rot_aa[0];
	const GEN_FLT aa_y = imu_rot_aa[1];
	const GEN_FLT aa_z = imu_rot_aa[2];
	const GEN_FLT lh_qi = imu_rot_aa[3];
	const GEN_FLT lh_qj = imu_rot_aa[4];
	const GEN_FLT lh_qk = imu_rot_aa[5];
	const GEN_FLT x0 = pow(aa_z, 2);
	const GEN_FLT x1 =
		((1e-20 < pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)) ? (sqrt(pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)))
															  : (1e-10));
	const GEN_FLT x2 = pow(x1, -2);
	const GEN_FLT x3 = 0.5 * x1;
	const GEN_FLT x4 = sin(x3);
	const GEN_FLT x5 = pow(x4, 2);
	const GEN_FLT x6 = x2 * x5;
	const GEN_FLT x7 = pow(aa_y, 2);
	const GEN_FLT x8 = pow(aa_x, 2);
	const GEN_FLT x9 = cos(x3);
	const GEN_FLT x10 = x0 * x6 + x6 * x7 + x6 * x8 + pow(x9, 2);
	const GEN_FLT x11 = pow(x10, -1.0 / 2.0);
	const GEN_FLT x12 = pow(time, 2);
	const GEN_FLT x13 = pow(lh_qk, 2);
	const GEN_FLT x14 = x13 * x12;
	const GEN_FLT x15 =
		((1e-20 < pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2))
			 ? (sqrt(pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2)))
			 : (1e-10));
	const GEN_FLT x16 = pow(x15, -2);
	const GEN_FLT x17 = 0.5 * x15;
	const GEN_FLT x18 = sin(x17);
	const GEN_FLT x19 = pow(x18, 2);
	const GEN_FLT x20 = x19 * x16;
	const GEN_FLT x21 = pow(lh_qj, 2);
	const GEN_FLT x22 = x21 * x12;
	const GEN_FLT x23 = pow(lh_qi, 2);
	const GEN_FLT x24 = x23 * x12;
	const GEN_FLT x25 = cos(x17);
	const GEN_FLT x26 = x20 * x14 + x22 * x20 + x24 * x20 + pow(x25, 2);
	const GEN_FLT x27 = pow(x26, -1.0 / 2.0);
	const GEN_FLT x28 = x27 * x11;
	const GEN_FLT x29 = pow(x1, -1);
	const GEN_FLT x30 = x4 * x29;
	const GEN_FLT x31 = x30 * x28;
	const GEN_FLT x32 = x31 * aa_z;
	const GEN_FLT x33 = pow(x15, -1);
	const GEN_FLT x34 = x33 * x18;
	const GEN_FLT x35 = x34 * lh_qj;
	const GEN_FLT x36 = x32 * x35;
	const GEN_FLT x37 = x9 * x28;
	const GEN_FLT x38 = x37 * lh_qi;
	const GEN_FLT x39 = x34 * x38;
	const GEN_FLT x40 = x39 * time;
	const GEN_FLT x41 = x33 * lh_qk;
	const GEN_FLT x42 = x41 * x18;
	const GEN_FLT x43 = x31 * aa_y;
	const GEN_FLT x44 = x42 * x43;
	const GEN_FLT x45 = x25 * x28;
	const GEN_FLT x46 = x29 * aa_x;
	const GEN_FLT x47 = x4 * x46;
	const GEN_FLT x48 = x45 * x47;
	const GEN_FLT x49 = x40 + x48 + x36 * time - x44 * time;
	const GEN_FLT x50 =
		((1e-20 < pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2))
			 ? ((1.0 / 2.0) * (2 * time * pow(lh_qi, 2) + 2 * time * pow(lh_qj, 2) + 2 * time * pow(lh_qk, 2)) /
				sqrt(pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2)))
			 : (0));
	const GEN_FLT x51 = x50 * x18;
	const GEN_FLT x52 = 0.5 * x51;
	const GEN_FLT x53 = x47 * x28;
	const GEN_FLT x54 = 2 * time;
	const GEN_FLT x55 = x54 * x23;
	const GEN_FLT x56 = 2 * x50 * x19 / pow(x15, 3);
	const GEN_FLT x57 = 1.0 * x51 * x25;
	const GEN_FLT x58 = x57 * x16;
	const GEN_FLT x59 = x54 * x13;
	const GEN_FLT x60 = x54 * x21;
	const GEN_FLT x61 = x11 *
						(-x57 + x55 * x20 - x56 * x14 - x56 * x22 - x56 * x24 + x58 * x14 + x58 * x22 + x58 * x24 +
						 x59 * x20 + x60 * x20) /
						pow(x26, 3.0 / 2.0);
	const GEN_FLT x62 = (1.0 / 2.0) * x25;
	const GEN_FLT x63 = x62 * x47;
	const GEN_FLT x64 = ((1e-20 < pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)) ? (0) : (0));
	const GEN_FLT x65 = x4 * x64;
	const GEN_FLT x66 = x65 * x45;
	const GEN_FLT x67 = x2 * x66;
	const GEN_FLT x68 = x34 * lh_qi;
	const GEN_FLT x69 = x68 * time;
	const GEN_FLT x70 = 1.0 * x9 * x65;
	const GEN_FLT x71 = 2 * x5 * x64 / pow(x1, 3);
	const GEN_FLT x72 = x2 * x70;
	const GEN_FLT x73 =
		x27 * (-x70 - x0 * x71 + x0 * x72 - x7 * x71 + x7 * x72 - x8 * x71 + x8 * x72) / pow(x10, 3.0 / 2.0);
	const GEN_FLT x74 = (1.0 / 2.0) * x9;
	const GEN_FLT x75 = x73 * x74;
	const GEN_FLT x76 = x33 * lh_qj;
	const GEN_FLT x77 = x45 * x30;
	const GEN_FLT x78 = x77 * aa_z;
	const GEN_FLT x79 = x50 * time;
	const GEN_FLT x80 = 0.5 * x79;
	const GEN_FLT x81 = x80 * x78;
	const GEN_FLT x82 = x51 * x16 * time;
	const GEN_FLT x83 = x35 * time;
	const GEN_FLT x84 = x65 * x28;
	const GEN_FLT x85 = x2 * x84;
	const GEN_FLT x86 = x85 * aa_z;
	const GEN_FLT x87 = x82 * lh_qj;
	const GEN_FLT x88 = (1.0 / 2.0) * x83;
	const GEN_FLT x89 = x88 * aa_z;
	const GEN_FLT x90 = x73 * x30;
	const GEN_FLT x91 = x61 * x30;
	const GEN_FLT x92 = 0.5 * x84;
	const GEN_FLT x93 = x9 * x25;
	const GEN_FLT x94 = x93 * x28;
	const GEN_FLT x95 = 0.5 * x94;
	const GEN_FLT x96 = x79 * x95;
	const GEN_FLT x97 = x96 * x33;
	const GEN_FLT x98 = x42 * time;
	const GEN_FLT x99 = x85 * aa_y;
	const GEN_FLT x100 = x74 * x61;
	const GEN_FLT x101 = x82 * lh_qk;
	const GEN_FLT x102 = x42 * x37;
	const GEN_FLT x103 = time * x102;
	const GEN_FLT x104 = 0.5 * x64;
	const GEN_FLT x105 = x29 * x104;
	const GEN_FLT x106 = x103 * x105;
	const GEN_FLT x107 = x37 * lh_qj;
	const GEN_FLT x108 = x34 * x107;
	const GEN_FLT x109 = time * x108;
	const GEN_FLT x110 = x105 * aa_z;
	const GEN_FLT x111 = (1.0 / 2.0) * x98;
	const GEN_FLT x112 = x90 * aa_y;
	const GEN_FLT x113 = x77 * aa_y;
	const GEN_FLT x114 = x80 * x113;
	const GEN_FLT x115 = x91 * aa_y;
	const GEN_FLT x116 = x64 * x95;
	const GEN_FLT x117 =
		2 * (x36 + x39 - x44 - x106 * aa_y + x109 * x110 + x111 * x112 + x111 * x115 - x41 * x114 + x43 * x101 +
			 x46 * x116 - x53 * x52 - x63 * x61 - x67 * aa_x - x69 * x100 - x69 * x92 - x73 * x63 - x75 * x69 +
			 x81 * x76 - x82 * x38 - x83 * x86 - x87 * x32 - x89 * x90 - x89 * x91 + x97 * lh_qi + x99 * x98);
	const GEN_FLT x118 = (1.0 / 2.0) * x93;
	const GEN_FLT x119 = x42 * x32;
	const GEN_FLT x120 = x111 * aa_z;
	const GEN_FLT x121 = x105 * aa_y;
	const GEN_FLT x122 = x46 * x104;
	const GEN_FLT x123 = x80 * x48;
	const GEN_FLT x124 = x33 * lh_qi;
	const GEN_FLT x125 = (1.0 / 2.0) * x69;
	const GEN_FLT x126 = x47 * x125;
	const GEN_FLT x127 = x85 * x69;
	const GEN_FLT x128 = x43 * x35;
	const GEN_FLT x129 = x82 * lh_qi;
	const GEN_FLT x130 = x68 * x53;
	const GEN_FLT x131 = -x119 - x128 - x130 - 0.5 * x66 - x106 * aa_z - x109 * x121 - x124 * x123 + x127 * aa_x +
						 x32 * x101 - x40 * x122 - x52 * x37 + x53 * x129 - x61 * x118 + x61 * x126 - x73 * x118 +
						 x73 * x126 - x76 * x114 - x81 * x41 + x83 * x99 + x86 * x98 + x87 * x43 + x88 * x112 +
						 x88 * x115 + x90 * x120 + x91 * x120;
	const GEN_FLT x132 = x94 - time * x119 - time * x128 - time * x130;
	const GEN_FLT x133 = x53 * x42;
	const GEN_FLT x134 = x68 * x32;
	const GEN_FLT x135 = x109 + x113 + time * x133 - time * x134;
	const GEN_FLT x136 = x29 * x116;
	const GEN_FLT x137 = x69 * aa_z;
	const GEN_FLT x138 = (1.0 / 2.0) * x137;
	const GEN_FLT x139 = x47 * x111;
	const GEN_FLT x140 = x62 * x30;
	const GEN_FLT x141 = x61 * x140;
	const GEN_FLT x142 = x85 * aa_x;
	const GEN_FLT x143 = x73 * x140;
	const GEN_FLT x144 =
		2 * (x108 + x133 - x134 + x103 * x122 + x136 * aa_y - x141 * aa_y - x143 * aa_y + x32 * x129 - x40 * x110 +
			 x41 * x123 - x52 * x43 - x53 * x101 - x61 * x139 - x67 * aa_y - x73 * x139 - x81 * x124 - x82 * x107 -
			 x83 * x100 - x83 * x75 - x83 * x92 + x85 * x137 + x90 * x138 + x91 * x138 + x97 * lh_qj - x98 * x142);
	const GEN_FLT x145 = 2 * x131 * x132 + x135 * x144 + x49 * x117;
	const GEN_FLT x146 = pow(x132, 2);
	const GEN_FLT x147 = 1e-10 + x146 + pow(x135, 2) + pow(x49, 2);
	const GEN_FLT x148 = atan2(x147, x132);
	const GEN_FLT x149 = pow(x147, 2);
	const GEN_FLT x150 = x145 * x148 / x149;
	const GEN_FLT x151 = 2 * x150;
	const GEN_FLT x152 = pow(x147, -1);
	const GEN_FLT x153 = x146 * x152 * (x145 / x132 - x131 * x147 / x146) / (x146 + x149);
	const GEN_FLT x154 = 2 * x153;
	const GEN_FLT x155 = x148 * x152;
	const GEN_FLT x156 = 2 * x135;
	const GEN_FLT x157 = x53 * x35;
	const GEN_FLT x158 = x68 * x43;
	const GEN_FLT x159 = x103 + x78 - time * x157 + time * x158;
	const GEN_FLT x160 = x88 * x47;
	out[0] = x117 * x155 - x49 * x151 + x49 * x154;
	out[1] = x144 * x155 - x150 * x156 + x153 * x156;
	out[2] =
		-x151 * x159 + x154 * x159 +
		2 * x155 *
			(x102 - x157 + x158 - x109 * x122 - x112 * x125 + x114 * x124 - x115 * x125 - x127 * aa_y + x136 * aa_z -
			 x141 * aa_z - x143 * aa_z - x37 * x101 + x40 * x121 - x43 * x129 - x52 * x32 + x61 * x160 - x67 * aa_z +
			 x73 * x160 - x75 * x98 - x76 * x123 + x83 * x142 + x87 * x53 - x92 * x98 + x96 * x41 - x98 * x100);
	out[3] = 0;
	out[4] = 0;
	out[5] = 0;
}

// Jacobian of imu_rot_f_aa wrt [aa_x, aa_y, aa_z, lh_qi, lh_qj, lh_qk]
static inline void gen_imu_rot_f_aa_jac_imu_rot_aa(FLT *out, const FLT time, const FLT *imu_rot_aa) {
	const GEN_FLT aa_x = imu_rot_aa[0];
	const GEN_FLT aa_y = imu_rot_aa[1];
	const GEN_FLT aa_z = imu_rot_aa[2];
	const GEN_FLT lh_qi = imu_rot_aa[3];
	const GEN_FLT lh_qj = imu_rot_aa[4];
	const GEN_FLT lh_qk = imu_rot_aa[5];
	const GEN_FLT x0 =
		((1e-20 < pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2))
			 ? (sqrt(pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2)))
			 : (1e-10));
	const GEN_FLT x1 = time / x0;
	const GEN_FLT x2 = pow(aa_z, 2);
	const GEN_FLT x3 =
		((1e-20 < pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)) ? (sqrt(pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)))
															  : (1e-10));
	const GEN_FLT x4 = pow(x3, -2);
	const GEN_FLT x5 = 0.5 * x3;
	const GEN_FLT x6 = sin(x5);
	const GEN_FLT x7 = pow(x6, 2);
	const GEN_FLT x8 = x4 * x7;
	const GEN_FLT x9 = pow(aa_y, 2);
	const GEN_FLT x10 = pow(aa_x, 2);
	const GEN_FLT x11 = cos(x5);
	const GEN_FLT x12 = x2 * x8 + x8 * x10 + x8 * x9 + pow(x11, 2);
	const GEN_FLT x13 = pow(x12, -1.0 / 2.0);
	const GEN_FLT x14 = 0.5 * x0;
	const GEN_FLT x15 = sin(x14);
	const GEN_FLT x16 = x15 * x13;
	const GEN_FLT x17 = pow(time, 2);
	const GEN_FLT x18 = x17 * pow(lh_qk, 2);
	const GEN_FLT x19 = pow(x0, -2);
	const GEN_FLT x20 = pow(x15, 2);
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = x17 * pow(lh_qj, 2);
	const GEN_FLT x23 = x17 * pow(lh_qi, 2);
	const GEN_FLT x24 = cos(x14);
	const GEN_FLT x25 = x21 * x18 + x22 * x21 + x23 * x21 + pow(x24, 2);
	const GEN_FLT x26 = pow(x25, -1.0 / 2.0);
	const GEN_FLT x27 = pow(x3, -1);
	const GEN_FLT x28 = x6 * x27;
	const GEN_FLT x29 = x28 * x26;
	const GEN_FLT x30 = x29 * x16;
	const GEN_FLT x31 = x1 * x30;
	const GEN_FLT x32 = x31 * lh_qi;
	const GEN_FLT x33 = -x32;
	const GEN_FLT x34 = ((1e-20 < pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2))
							 ? (aa_x / sqrt(pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)))
							 : (0));
	const GEN_FLT x35 = x6 * x34;
	const GEN_FLT x36 = x1 * lh_qj;
	const GEN_FLT x37 = x36 * x16;
	const GEN_FLT x38 = x4 * x26;
	const GEN_FLT x39 = x38 * x37;
	const GEN_FLT x40 = x39 * aa_y;
	const GEN_FLT x41 = 1.0 * x11;
	const GEN_FLT x42 = x6 * x41;
	const GEN_FLT x43 = x42 * x34;
	const GEN_FLT x44 = 2 * x8;
	const GEN_FLT x45 = x4 * x10;
	const GEN_FLT x46 = 2 * x7 / pow(x3, 3);
	const GEN_FLT x47 = x46 * x10;
	const GEN_FLT x48 = x9 * x46;
	const GEN_FLT x49 = x4 * x9;
	const GEN_FLT x50 = x2 * x46;
	const GEN_FLT x51 = x2 * x4;
	const GEN_FLT x52 = -x43 + x43 * x45 + x43 * x49 + x44 * aa_x - x47 * x34 - x48 * x34 - x50 * x34 + x51 * x43;
	const GEN_FLT x53 = x29 * aa_y;
	const GEN_FLT x54 = (1.0 / 2.0) / pow(x12, 3.0 / 2.0);
	const GEN_FLT x55 = x54 * x15;
	const GEN_FLT x56 = x55 * x36;
	const GEN_FLT x57 = x53 * x56;
	const GEN_FLT x58 = x26 * x11;
	const GEN_FLT x59 = x54 * x24;
	const GEN_FLT x60 = x58 * x59;
	const GEN_FLT x61 = 0.5 * aa_y;
	const GEN_FLT x62 = x58 * x16;
	const GEN_FLT x63 = x1 * x62;
	const GEN_FLT x64 = x63 * lh_qj;
	const GEN_FLT x65 = x64 * x27;
	const GEN_FLT x66 = x61 * x65;
	const GEN_FLT x67 = x29 * aa_x;
	const GEN_FLT x68 = x1 * lh_qi;
	const GEN_FLT x69 = x52 * x55;
	const GEN_FLT x70 = x68 * x69;
	const GEN_FLT x71 = 0.5 * aa_z;
	const GEN_FLT x72 = x63 * lh_qk;
	const GEN_FLT x73 = x72 * x27;
	const GEN_FLT x74 = x71 * x73;
	const GEN_FLT x75 = x28 * aa_z;
	const GEN_FLT x76 = x75 * x26;
	const GEN_FLT x77 = x1 * lh_qk;
	const GEN_FLT x78 = x77 * x69;
	const GEN_FLT x79 = x1 * x16;
	const GEN_FLT x80 = x79 * lh_qk;
	const GEN_FLT x81 = x80 * x38;
	const GEN_FLT x82 = x81 * x35;
	const GEN_FLT x83 = x79 * lh_qi;
	const GEN_FLT x84 = x83 * x38;
	const GEN_FLT x85 = x35 * aa_x;
	const GEN_FLT x86 = x63 * lh_qi;
	const GEN_FLT x87 = x86 * x27;
	const GEN_FLT x88 = 0.5 * aa_x;
	const GEN_FLT x89 = x88 * x34;
	const GEN_FLT x90 = x24 * x13;
	const GEN_FLT x91 = x90 * x26;
	const GEN_FLT x92 = 0.5 * x91;
	const GEN_FLT x93 = x6 * x92;
	const GEN_FLT x94 = x28 * aa_y;
	const GEN_FLT x95 =
		((1e-20 < pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2)) ? (0)
																											  : (0));
	const GEN_FLT x96 = 1.0 * x24;
	const GEN_FLT x97 = x96 * x15;
	const GEN_FLT x98 = x97 * x95;
	const GEN_FLT x99 = 2 * x20 / pow(x0, 3);
	const GEN_FLT x100 = x99 * x22;
	const GEN_FLT x101 = x23 * x19;
	const GEN_FLT x102 = x22 * x19;
	const GEN_FLT x103 = x99 * x23;
	const GEN_FLT x104 = x99 * x18;
	const GEN_FLT x105 = x19 * x18;
	const GEN_FLT x106 = -x98 - x95 * x100 - x95 * x103 - x95 * x104 + x98 * x101 + x98 * x102 + x98 * x105;
	const GEN_FLT x107 = (1.0 / 2.0) / pow(x25, 3.0 / 2.0);
	const GEN_FLT x108 = x107 * x106;
	const GEN_FLT x109 = x94 * x108;
	const GEN_FLT x110 = 0.5 * x62;
	const GEN_FLT x111 = x91 * x28;
	const GEN_FLT x112 = x111 * aa_z;
	const GEN_FLT x113 = 0.5 * x95;
	const GEN_FLT x114 = x77 * x113;
	const GEN_FLT x115 = x19 * time;
	const GEN_FLT x116 = x115 * lh_qk;
	const GEN_FLT x117 = x76 * x16;
	const GEN_FLT x118 = x95 * x117;
	const GEN_FLT x119 = x111 * aa_y;
	const GEN_FLT x120 = x36 * x113;
	const GEN_FLT x121 = x80 * x107;
	const GEN_FLT x122 = x75 * x121;
	const GEN_FLT x123 = x30 * aa_y;
	const GEN_FLT x124 = x115 * lh_qj;
	const GEN_FLT x125 = x95 * x124;
	const GEN_FLT x126 = x90 * x107;
	const GEN_FLT x127 = x11 * x126;
	const GEN_FLT x128 = x28 * aa_x;
	const GEN_FLT x129 = x83 * x108;
	const GEN_FLT x130 = x111 * aa_x;
	const GEN_FLT x131 = x68 * x130;
	const GEN_FLT x132 = x67 * x16;
	const GEN_FLT x133 = x95 * x132;
	const GEN_FLT x134 = x115 * lh_qi;
	const GEN_FLT x135 = x106 * x122 - x106 * x127 - x112 * x114 - x113 * x131 + x118 * x116 - x119 * x120 +
						 x123 * x125 + x128 * x129 + x133 * x134 + x37 * x109 - x95 * x110;
	const GEN_FLT x136 = x135 + x33 + x40 * x35 + x52 * x57 - x60 * x52 - x66 * x34 + x70 * x67 - x74 * x34 +
						 x78 * x76 + x82 * aa_z + x84 * x85 - x89 * x87 - x93 * x34;
	const GEN_FLT x137 = x79 * x76;
	const GEN_FLT x138 = x30 * x36;
	const GEN_FLT x139 = x79 * x67;
	const GEN_FLT x140 = x58 * x90;
	const GEN_FLT x141 = x140 - x137 * lh_qk - x138 * aa_y - x139 * lh_qi;
	const GEN_FLT x142 = 2 * x141;
	const GEN_FLT x143 = x58 * x55;
	const GEN_FLT x144 = x68 * x143;
	const GEN_FLT x145 = x39 * aa_z;
	const GEN_FLT x146 = x67 * x59;
	const GEN_FLT x147 = 0.5 * x26;
	const GEN_FLT x148 = x83 * x147;
	const GEN_FLT x149 = x71 * x65;
	const GEN_FLT x150 = x73 * x61;
	const GEN_FLT x151 = x27 * x140;
	const GEN_FLT x152 = x34 * x151;
	const GEN_FLT x153 = x4 * x91;
	const GEN_FLT x154 = x6 * x153;
	const GEN_FLT x155 = x154 * aa_x;
	const GEN_FLT x156 = x36 * x112;
	const GEN_FLT x157 = x95 * x30;
	const GEN_FLT x158 = x75 * x37;
	const GEN_FLT x159 = x94 * x121;
	const GEN_FLT x160 = x62 * x115;
	const GEN_FLT x161 = x160 * lh_qi;
	const GEN_FLT x162 = x11 * x107;
	const GEN_FLT x163 = x106 * x162;
	const GEN_FLT x164 = 0.5 * x140;
	const GEN_FLT x165 = x68 * x164;
	const GEN_FLT x166 = x116 * x123;
	const GEN_FLT x167 = x106 * x126;
	const GEN_FLT x168 = x106 * x159 - x108 * x158 + x113 * x156 - x117 * x125 - x119 * x114 - x128 * x167 -
						 x83 * x163 - x88 * x157 - x95 * x161 + x95 * x165 + x95 * x166;
	const GEN_FLT x169 = x111 + x168 + x34 * x149 - x34 * x150 - x34 * x155 - x35 * x145 - x35 * x148 - x52 * x144 -
						 x52 * x146 + x78 * x53 + x82 * aa_y + x88 * x152 - x76 * x52 * x56;
	const GEN_FLT x170 = x1 * x123;
	const GEN_FLT x171 = x130 + x86 - x170 * lh_qk + x76 * x37;
	const GEN_FLT x172 = 2 * x171;
	const GEN_FLT x173 = x87 * x71;
	const GEN_FLT x174 = x31 * lh_qk;
	const GEN_FLT x175 = x53 * x59;
	const GEN_FLT x176 = x34 * x154;
	const GEN_FLT x177 = x84 * aa_z;
	const GEN_FLT x178 = x37 * x147;
	const GEN_FLT x179 = x6 * x178;
	const GEN_FLT x180 = x36 * x143;
	const GEN_FLT x181 = x121 * x128;
	const GEN_FLT x182 = x77 * x130;
	const GEN_FLT x183 = x68 * x112;
	const GEN_FLT x184 = x36 * x164;
	const GEN_FLT x185 = x160 * lh_qj;
	const GEN_FLT x186 = x94 * x126;
	const GEN_FLT x187 = -x106 * x181 - x106 * x186 + x113 * x182 - x113 * x183 - x116 * x133 + x118 * x134 -
						 x37 * x163 - x61 * x157 + x75 * x129 + x95 * x184 - x95 * x185;
	const GEN_FLT x188 = x174 + x187 - x176 * aa_y - x34 * x173 - x34 * x179 + x35 * x177 - x52 * x175 - x52 * x180 +
						 x61 * x152 + x70 * x76 - x78 * x67 - x82 * aa_x + x89 * x73;
	const GEN_FLT x189 = x119 + x64 - x137 * lh_qi + x139 * lh_qk;
	const GEN_FLT x190 = 2 * x189;
	const GEN_FLT x191 = x136 * x142 + x169 * x172 + x188 * x190;
	const GEN_FLT x192 = pow(x141, 2);
	const GEN_FLT x193 = 1e-10 + x192 + pow(x171, 2) + pow(x189, 2);
	const GEN_FLT x194 = atan2(x193, x141);
	const GEN_FLT x195 = pow(x193, 2);
	const GEN_FLT x196 = pow(x195, -1);
	const GEN_FLT x197 = x196 * x194;
	const GEN_FLT x198 = x172 * x197;
	const GEN_FLT x199 = pow(x141, -1);
	const GEN_FLT x200 = x193 / x192;
	const GEN_FLT x201 = x191 * x199 - x200 * x136;
	const GEN_FLT x202 = pow(x193, -1);
	const GEN_FLT x203 = x202 * x192 / (x192 + x195);
	const GEN_FLT x204 = x203 * x172;
	const GEN_FLT x205 = 2 * x194;
	const GEN_FLT x206 = x202 * x205;
	const GEN_FLT x207 = x190 * x197;
	const GEN_FLT x208 = x203 * x190;
	const GEN_FLT x209 = x112 + x72 + x170 * lh_qi - x67 * x37;
	const GEN_FLT x210 = x209 * x205;
	const GEN_FLT x211 = x210 * x196;
	const GEN_FLT x212 = 2 * x203 * x209;
	const GEN_FLT x213 = -x138;
	const GEN_FLT x214 = x76 * x59;
	const GEN_FLT x215 = x67 * x56;
	const GEN_FLT x216 = x87 * x61;
	const GEN_FLT x217 = x80 * x147;
	const GEN_FLT x218 = x84 * aa_y;
	const GEN_FLT x219 = x77 * x143;
	const GEN_FLT x220 = x71 * x30;
	const GEN_FLT x221 = x37 * x128;
	const GEN_FLT x222 = x160 * lh_qk;
	const GEN_FLT x223 = x77 * x164;
	const GEN_FLT x224 = x68 * x119;
	const GEN_FLT x225 = x123 * x134;
	const GEN_FLT x226 = -x120 * x130 + x124 * x133 + x221 * x108 + x224 * x113 - x75 * x167 - x80 * x163 - x83 * x109 -
						 x95 * x220 - x95 * x222 + x95 * x223 - x95 * x225;
	const GEN_FLT x227 = ((1e-20 < pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2))
							  ? (aa_y / sqrt(pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)))
							  : (0));
	const GEN_FLT x228 = x42 * x227;
	const GEN_FLT x229 = x46 * x227;
	const GEN_FLT x230 = -x228 - x2 * x229 + x44 * aa_y + x45 * x228 - x47 * x227 + x49 * x228 + x51 * x228 - x9 * x229;
	const GEN_FLT x231 = x88 * x227;
	const GEN_FLT x232 = x6 * x227;
	const GEN_FLT x233 = x81 * x232;
	const GEN_FLT x234 = x68 * x55;
	const GEN_FLT x235 = x67 * x234;
	const GEN_FLT x236 = x232 * aa_x;
	const GEN_FLT x237 = x76 * x230;
	const GEN_FLT x238 = x77 * x55;
	const GEN_FLT x239 = x135 + x213 + x230 * x235 + x233 * aa_z + x238 * x237 + x40 * x232 + x57 * x230 - x60 * x230 -
						 x66 * x227 - x74 * x227 + x84 * x236 - x87 * x231 - x93 * x227;
	const GEN_FLT x240 = -x174;
	const GEN_FLT x241 = x53 * x230;
	const GEN_FLT x242 = x59 * x230;
	const GEN_FLT x243 = x227 * x151;
	const GEN_FLT x244 = x230 * x143;
	const GEN_FLT x245 = x168 + x240 + x227 * x149 - x227 * x150 - x227 * x155 - x232 * x145 - x232 * x148 +
						 x233 * aa_y + x238 * x241 - x56 * x237 - x67 * x242 - x68 * x244 + x88 * x243;
	const GEN_FLT x246 = x67 * x238;
	const GEN_FLT x247 = x84 * x232;
	const GEN_FLT x248 = x227 * x154;
	const GEN_FLT x249 = x111 + x187 - x227 * x173 - x227 * x179 - x230 * x246 - x233 * aa_x + x234 * x237 +
						 x247 * aa_z - x248 * aa_y - x36 * x244 - x53 * x242 + x61 * x243 + x73 * x231;
	const GEN_FLT x250 = x239 * x142 + x245 * x172 + x249 * x190;
	const GEN_FLT x251 = -x239 * x200 + x250 * x199;
	const GEN_FLT x252 = ((1e-20 < pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2))
							  ? (aa_z / sqrt(pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)))
							  : (0));
	const GEN_FLT x253 = x6 * x252;
	const GEN_FLT x254 = x81 * x253;
	const GEN_FLT x255 = x42 * x252;
	const GEN_FLT x256 =
		-x255 + x44 * aa_z + x45 * x255 - x47 * x252 - x48 * x252 + x49 * x255 - x50 * x252 + x51 * x255;
	const GEN_FLT x257 = x76 * x256;
	const GEN_FLT x258 = x252 * aa_x;
	const GEN_FLT x259 = x6 * x258;
	const GEN_FLT x260 = 0.5 * x258;
	const GEN_FLT x261 = x135 + x240 + x235 * x256 + x238 * x257 + x254 * aa_z + x40 * x253 + x57 * x256 - x60 * x256 -
						 x66 * x252 - x74 * x252 + x84 * x259 - x87 * x260 - x93 * x252;
	const GEN_FLT x262 = x53 * x238;
	const GEN_FLT x263 = x61 * x252;
	const GEN_FLT x264 = x256 * x143;
	const GEN_FLT x265 = x138 + x168 + x252 * x149 - x253 * x145 - x253 * x148 + x254 * aa_y - x256 * x146 -
						 x258 * x154 + x260 * x151 + x262 * x256 - x56 * x257 - x68 * x264 - x73 * x263;
	const GEN_FLT x266 = x252 * x151;
	const GEN_FLT x267 = x252 * x154;
	const GEN_FLT x268 = x187 + x33 + x234 * x257 - x252 * x173 + x253 * x177 - x253 * x178 - x256 * x175 -
						 x256 * x246 - x267 * aa_y - x36 * x264 + x61 * x266 + x73 * x260 - x81 * x259;
	const GEN_FLT x269 = x261 * x142 + x265 * x172 + x268 * x190;
	const GEN_FLT x270 = -x200 * x261 + x269 * x199;
	const GEN_FLT x271 = x53 * x234;
	const GEN_FLT x272 = -x139;
	const GEN_FLT x273 =
		((1e-20 < pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2))
			 ? (pow(time, 2) * lh_qi /
				sqrt(pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2)))
			 : (0));
	const GEN_FLT x274 = 0.5 * x273;
	const GEN_FLT x275 = x273 * x117;
	const GEN_FLT x276 = x36 * x274;
	const GEN_FLT x277 = x77 * x112;
	const GEN_FLT x278 = x37 * x107;
	const GEN_FLT x279 = 2 * x21 * x17;
	const GEN_FLT x280 = x97 * x273;
	const GEN_FLT x281 =
		-x280 - x273 * x100 - x273 * x103 - x273 * x104 + x279 * lh_qi + x280 * x101 + x280 * x102 + x280 * x105;
	const GEN_FLT x282 = x94 * x281;
	const GEN_FLT x283 = x273 * x132;
	const GEN_FLT x284 = x75 * x281;
	const GEN_FLT x285 = x83 * x107;
	const GEN_FLT x286 = x281 * x128;
	const GEN_FLT x287 = ((1e-20 < pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)) ? (0) : (0));
	const GEN_FLT x288 = x6 * x287;
	const GEN_FLT x289 = x41 * x288;
	const GEN_FLT x290 = x46 * x287;
	const GEN_FLT x291 = x4 * x289;
	const GEN_FLT x292 = -x289 - x2 * x290 + x2 * x291 + x45 * x289 - x47 * x287 - x9 * x290 + x9 * x291;
	const GEN_FLT x293 = x56 * x292;
	const GEN_FLT x294 = x38 * x288;
	const GEN_FLT x295 = x37 * x294;
	const GEN_FLT x296 = x76 * x292;
	const GEN_FLT x297 = x81 * x288;
	const GEN_FLT x298 = x71 * x287;
	const GEN_FLT x299 = x88 * x287;
	const GEN_FLT x300 = x83 * x294;
	const GEN_FLT x301 = x235 * x292 + x238 * x296 + x295 * aa_y + x297 * aa_z + x300 * aa_x + x53 * x293 - x60 * x292 -
						 x66 * x287 - x73 * x298 - x87 * x299 - x92 * x288;
	const GEN_FLT x302 = x272 + x301 - x274 * x131 + x275 * x116 - x276 * x119 - x277 * x274 + x278 * x282 -
						 x281 * x127 + x283 * x134 + x284 * x121 + x286 * x285 - x62 * x274 + x273 * x124 * x123;
	const GEN_FLT x303 = x77 * x119;
	const GEN_FLT x304 = x274 * x140;
	const GEN_FLT x305 = x281 * x162;
	const GEN_FLT x306 = x16 * x274;
	const GEN_FLT x307 = x284 * x107;
	const GEN_FLT x308 = x281 * x126;
	const GEN_FLT x309 = x288 * x147;
	const GEN_FLT x310 = x287 * x151;
	const GEN_FLT x311 = x61 * x287;
	const GEN_FLT x312 = x288 * x153;
	const GEN_FLT x313 = x59 * x292;
	const GEN_FLT x314 = x287 * x149 - x292 * x144 + x292 * x262 - x295 * aa_z + x297 * aa_y - x312 * aa_x -
						 x67 * x313 - x73 * x311 - x76 * x293 - x83 * x309 + x88 * x310;
	const GEN_FLT x315 = x314 + x63 - x273 * x161 + x273 * x166 - x274 * x303 - x275 * x124 + x276 * x112 +
						 x281 * x159 - x308 * x128 - x37 * x307 - x67 * x306 + x68 * x304 - x83 * x305;
	const GEN_FLT x316 = -x137;
	const GEN_FLT x317 = x234 * x296 - x288 * x178 - x292 * x180 - x292 * x246 - x297 * aa_x + x300 * aa_z -
						 x312 * aa_y - x53 * x313 + x61 * x310 + x73 * x299 - x87 * x298;
	const GEN_FLT x318 = x316 + x317 - x273 * x185 - x274 * x123 + x274 * x182 - x274 * x183 + x275 * x134 -
						 x281 * x181 - x281 * x186 - x283 * x116 + x36 * x304 - x37 * x305 + x83 * x307;
	const GEN_FLT x319 = x302 * x142 + x315 * x172 + x318 * x190;
	const GEN_FLT x320 = -x200 * x302 + x319 * x199;
	const GEN_FLT x321 = x319 * x196;
	const GEN_FLT x322 = x321 * x194;
	const GEN_FLT x323 = -x219 * x292 - x271 * x292 + x295 * aa_x - x300 * aa_y - x312 * aa_z - x65 * x299 +
						 x67 * x293 + x71 * x310 - x76 * x313 - x80 * x309 + x87 * x311;
	const GEN_FLT x324 =
		((1e-20 < pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2))
			 ? (pow(time, 2) * lh_qj /
				sqrt(pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2)))
			 : (0));
	const GEN_FLT x325 = x324 * x116;
	const GEN_FLT x326 = 0.5 * x324;
	const GEN_FLT x327 = x97 * x324;
	const GEN_FLT x328 = x99 * x324;
	const GEN_FLT x329 = x19 * x327;
	const GEN_FLT x330 =
		-x327 - x22 * x328 + x22 * x329 - x23 * x328 + x23 * x329 + x279 * lh_qj - x324 * x104 + x327 * x105;
	const GEN_FLT x331 = x326 * x119;
	const GEN_FLT x332 = x324 * x115;
	const GEN_FLT x333 = x332 * x123;
	const GEN_FLT x334 = -x170;
	const GEN_FLT x335 = x330 * x128;
	const GEN_FLT x336 = x332 * lh_qi;
	const GEN_FLT x337 = x301 + x334 - x277 * x326 + x285 * x335 - x324 * x110 + x325 * x117 - x326 * x131 +
						 x330 * x122 - x330 * x127 + x333 * lh_qj + x336 * x132 - x36 * x331 + x94 * x278 * x330;
	const GEN_FLT x338 = x332 * lh_qj;
	const GEN_FLT x339 = x330 * x107;
	const GEN_FLT x340 = x330 * x162;
	const GEN_FLT x341 = x330 * x126;
	const GEN_FLT x342 = x30 * x324;
	const GEN_FLT x343 = x137 + x314 - x324 * x161 + x324 * x165 + x324 * x166 + x326 * x156 + x330 * x159 -
						 x338 * x117 - x339 * x158 - x341 * x128 - x77 * x331 - x83 * x340 - x88 * x342;
	const GEN_FLT x344 = x83 * x339;
	const GEN_FLT x345 = x317 + x63 + x324 * x184 - x324 * x185 - x325 * x132 + x326 * x182 - x326 * x183 -
						 x335 * x121 + x336 * x117 - x37 * x340 - x61 * x342 + x75 * x344 - x94 * x341;
	const GEN_FLT x346 = x337 * x142 + x343 * x172 + x345 * x190;
	const GEN_FLT x347 = -x200 * x337 + x346 * x199;
	const GEN_FLT x348 =
		((1e-20 < pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2))
			 ? (pow(time, 2) * lh_qk /
				sqrt(pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2)))
			 : (0));
	const GEN_FLT x349 = x15 * x348;
	const GEN_FLT x350 = x13 * x349;
	const GEN_FLT x351 = x58 * x350;
	const GEN_FLT x352 = x67 * x350;
	const GEN_FLT x353 = x76 * x350;
	const GEN_FLT x354 = 0.5 * x348;
	const GEN_FLT x355 = x96 * x349;
	const GEN_FLT x356 =
		-x355 + x279 * lh_qk - x348 * x100 - x348 * x103 - x348 * x104 + x355 * x101 + x355 * x102 + x355 * x105;
	const GEN_FLT x357 = x356 * x107;
	const GEN_FLT x358 = x36 * x354;
	const GEN_FLT x359 = x350 * x124;
	const GEN_FLT x360 = x83 * x357;
	const GEN_FLT x361 = x301 + x316 - 0.5 * x351 - x277 * x354 + x352 * x134 + x353 * x116 - x354 * x131 +
						 x356 * x122 - x356 * x127 - x358 * x119 + x360 * x128 + x53 * x359 + x94 * x37 * x357;
	const GEN_FLT x362 = x29 * x350;
	const GEN_FLT x363 = x356 * x126;
	const GEN_FLT x364 = x53 * x350;
	const GEN_FLT x365 = x356 * x162;
	const GEN_FLT x366 = x351 * x115;
	const GEN_FLT x367 = x348 * x164;
	const GEN_FLT x368 = x314 + x334 - x353 * x124 - x354 * x303 + x356 * x159 - x357 * x158 + x358 * x112 -
						 x363 * x128 + x364 * x116 - x366 * lh_qi + x68 * x367 - x83 * x365 - x88 * x362;
	const GEN_FLT x369 = x139 + x317 - x352 * x116 + x353 * x134 + x354 * x182 - x354 * x183 - x356 * x181 -
						 x356 * x186 + x36 * x367 - x366 * lh_qj - x37 * x365 - x61 * x362 + x75 * x360;
	const GEN_FLT x370 = x361 * x142 + x368 * x172 + x369 * x190;
	const GEN_FLT x371 = -x200 * x361 + x370 * x199;
	out[0] = -x191 * x198 + x201 * x204 + x206 * x169;
	out[1] = x201 * x208 + x206 * x188 - x207 * x191;
	out[2] = x206 * (x213 + x226 - x176 * aa_z + x34 * x216 - x35 * x217 - x35 * x218 - x52 * x214 + x52 * x215 -
					 x52 * x219 - x70 * x53 + x71 * x152 + x85 * x39 - x89 * x65) -
			 x211 * x191 + x212 * x201;
	out[3] = 0;
	out[4] = 0;
	out[5] = 0;
	out[6] = x204 * x251 + x206 * x245 - x250 * x198;
	out[7] = x206 * x249 - x207 * x250 + x208 * x251;
	out[8] = x206 * (x226 + x32 + x215 * x230 + x216 * x227 - x217 * x232 - x234 * x241 - x247 * aa_y - x248 * aa_z +
					 x39 * x236 - x65 * x231 + x71 * x243 - x76 * x242 - x77 * x244) -
			 x211 * x250 + x212 * x251;
	out[9] = 0;
	out[10] = 0;
	out[11] = 0;
	out[12] = x204 * x270 + x206 * x265 - x269 * x198;
	out[13] = x206 * x268 - x207 * x269 + x208 * x270;
	out[14] = x206 * (x111 + x226 - x214 * x256 + x215 * x256 - x217 * x253 - x218 * x253 - x267 * aa_z - x271 * x256 +
					  x39 * x259 - x65 * x260 + x71 * x266 - x77 * x264 + x87 * x263) -
			  x211 * x269 + x212 * x270;
	out[15] = 0;
	out[16] = 0;
	out[17] = 0;
	out[18] = x204 * x320 + x206 * x315 - x322 * x172;
	out[19] = x206 * x318 + x208 * x320 - x322 * x190;
	out[20] = x206 * (x170 + x323 - x273 * x222 - x273 * x225 + x274 * x224 - x276 * x130 + x278 * x286 - x282 * x285 +
					  x283 * x124 - x75 * x308 - x76 * x306 + x77 * x304 - x80 * x305) -
			  x210 * x321 + x212 * x320;
	out[21] = 1;
	out[22] = 0;
	out[23] = 0;
	out[24] = x204 * x347 + x206 * x343 - x346 * x198;
	out[25] = x206 * x345 - x207 * x346 + x208 * x347;
	out[26] = x206 * (x272 + x323 - x220 * x324 - x222 * x324 + x223 * x324 + x224 * x326 + x278 * x335 - x333 * lh_qi +
					  x338 * x132 - x75 * x341 - x80 * x340 - x94 * x344 - x36 * x326 * x130) -
			  x211 * x346 + x212 * x347;
	out[27] = 0;
	out[28] = 1;
	out[29] = 0;
	out[30] = x204 * x371 + x206 * x368 - x370 * x198;
	out[31] = x206 * x369 - x207 * x370 + x208 * x371;
	out[32] = x206 * (x323 + x63 + x221 * x357 + x224 * x354 - x358 * x130 - x364 * x134 - x366 * lh_qk + x67 * x359 -
					  x71 * x362 - x75 * x363 + x77 * x367 - x80 * x365 - x94 * x360) -
			  x211 * x370 + x212 * x371;
	out[33] = 0;
	out[34] = 0;
	out[35] = 1;
}
