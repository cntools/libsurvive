#pragma once
#include "common.h"
/** Applying function <function imu_rot_f at 0x7f3ddf3fcf80> */
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
		((0 < pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2))
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
	const GEN_FLT x12 = x10 * obj_qj;
	const GEN_FLT x13 = x10 * aa_x;
	const GEN_FLT x14 = x8 * x9;
	const GEN_FLT x15 = x10 * aa_y;
	out[0] = -x11 * obj_qk - x12 * aa_y - x13 * obj_qi + x14 * obj_qw;
	out[1] = -x12 * aa_z + x13 * obj_qw + x14 * obj_qi + x15 * obj_qk;
	out[2] = x11 * obj_qi - x13 * obj_qk + x14 * obj_qj + x15 * obj_qw;
	out[3] = x11 * obj_qw + x12 * aa_x + x14 * obj_qk - x15 * obj_qi;
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
		((0 < pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2))
			 ? (sqrt(pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2)))
			 : (1e-10));
	const GEN_FLT x1 = 0.5 * x0;
	const GEN_FLT x2 = cos(x1);
	const GEN_FLT x3 = pow(time, 2);
	const GEN_FLT x4 = pow(aa_x, 2);
	const GEN_FLT x5 = x4 * x3;
	const GEN_FLT x6 = sin(x1);
	const GEN_FLT x7 = pow(x6, 2);
	const GEN_FLT x8 =
		((0 < pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2))
			 ? ((1.0 / 2.0) * (2 * time * pow(aa_x, 2) + 2 * time * pow(aa_y, 2) + 2 * time * pow(aa_z, 2)) /
				sqrt(pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2)))
			 : (0));
	const GEN_FLT x9 = 2 * x8 * x7 / pow(x0, 3);
	const GEN_FLT x10 = pow(x0, -2);
	const GEN_FLT x11 = x6 * x8;
	const GEN_FLT x12 = 1.0 * x2 * x11;
	const GEN_FLT x13 = x12 * x10;
	const GEN_FLT x14 = 2 * time;
	const GEN_FLT x15 = x4 * x14;
	const GEN_FLT x16 = x7 * x10;
	const GEN_FLT x17 = pow(aa_y, 2);
	const GEN_FLT x18 = x14 * x17;
	const GEN_FLT x19 = x3 * x17;
	const GEN_FLT x20 = pow(aa_z, 2);
	const GEN_FLT x21 = x20 * x14;
	const GEN_FLT x22 = x3 * x20;
	const GEN_FLT x23 = x19 * x16 + x22 * x16 + x5 * x16 + pow(x2, 2);
	const GEN_FLT x24 =
		(1.0 / 2.0) *
		(-x12 + x13 * x19 + x15 * x16 + x18 * x16 + x21 * x16 + x22 * x13 + x5 * x13 - x5 * x9 - x9 * x19 - x9 * x22) /
		pow(x23, 3.0 / 2.0);
	const GEN_FLT x25 = x2 * x24;
	const GEN_FLT x26 = pow(x23, -1.0 / 2.0);
	const GEN_FLT x27 = x26 * obj_qw;
	const GEN_FLT x28 = 0.5 * x11;
	const GEN_FLT x29 = pow(x0, -1);
	const GEN_FLT x30 = x6 * x29;
	const GEN_FLT x31 = x30 * x26;
	const GEN_FLT x32 = x31 * aa_x;
	const GEN_FLT x33 = 0.5 * x8 * x29;
	const GEN_FLT x34 = time * aa_z;
	const GEN_FLT x35 = x34 * x33;
	const GEN_FLT x36 = time * aa_y;
	const GEN_FLT x37 = x30 * x24;
	const GEN_FLT x38 = x37 * obj_qj;
	const GEN_FLT x39 = x11 * x10;
	const GEN_FLT x40 = x39 * time;
	const GEN_FLT x41 = x40 * x26;
	const GEN_FLT x42 = x41 * aa_x;
	const GEN_FLT x43 = x41 * obj_qk;
	const GEN_FLT x44 = x31 * aa_z;
	const GEN_FLT x45 = time * aa_x;
	const GEN_FLT x46 = x37 * obj_qi;
	const GEN_FLT x47 = x36 * x26;
	const GEN_FLT x48 = x47 * x39;
	const GEN_FLT x49 = x37 * obj_qk;
	const GEN_FLT x50 = x47 * x33;
	const GEN_FLT x51 = x2 * x50;
	const GEN_FLT x52 = x33 * time;
	const GEN_FLT x53 = x2 * obj_qi;
	const GEN_FLT x54 = x53 * x26;
	const GEN_FLT x55 = x31 * aa_y;
	const GEN_FLT x56 = x28 * x26;
	const GEN_FLT x57 = x27 * aa_x;
	const GEN_FLT x58 = aa_z * obj_qj;
	const GEN_FLT x59 = x2 * x52;
	const GEN_FLT x60 = x59 * x26;
	const GEN_FLT x61 = x37 * time;
	const GEN_FLT x62 = x37 * obj_qw;
	const GEN_FLT x63 = aa_z * obj_qi;
	const GEN_FLT x64 = x31 * obj_qi;
	const GEN_FLT x65 = x36 * x27;
	const GEN_FLT x66 = x30 * x27;
	const GEN_FLT x67 = x60 * aa_x;
	const GEN_FLT x68 = x27 * aa_z;
	out[0] = -x25 * obj_qw - x28 * x27 - x32 * obj_qi + x36 * x38 + x42 * obj_qi + x43 * aa_z - x44 * obj_qk +
			 x45 * x46 + x48 * obj_qj + x49 * x34 - x51 * obj_qj - x55 * obj_qj - x54 * x52 * aa_x -
			 x2 * x35 * x26 * obj_qk;
	out[1] = -x44 * obj_qj - x48 * obj_qk - x49 * x36 + x51 * obj_qk - x53 * x24 + x55 * obj_qk - x56 * obj_qi +
			 x57 * x30 - x57 * x40 + x57 * x59 + x58 * x41 - x60 * x58 + x61 * x58 - x62 * x45;
	out[2] = -x25 * obj_qj - x32 * obj_qk + x43 * aa_x + x45 * x49 + x54 * x35 - x56 * obj_qj - x62 * x36 - x63 * x41 -
			 x63 * x61 + x64 * aa_z - x65 * x39 + x66 * aa_y - x67 * obj_qk + x2 * x65 * x33;
	out[3] = -x25 * obj_qk + x32 * obj_qj - x42 * obj_qj - x45 * x38 + x46 * x36 + x48 * obj_qi - x50 * x53 -
			 x56 * obj_qk - x62 * x34 - x64 * aa_y + x66 * aa_z + x67 * obj_qj - x68 * x40 + x68 * x59;
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
		((0 < pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2))
			 ? (sqrt(pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2)))
			 : (1e-10));
	const GEN_FLT x3 = pow(x2, -2);
	const GEN_FLT x4 = 0.5 * x2;
	const GEN_FLT x5 = sin(x4);
	const GEN_FLT x6 = pow(x5, 2);
	const GEN_FLT x7 = x3 * x6;
	const GEN_FLT x8 = x0 * pow(aa_y, 2);
	const GEN_FLT x9 = x0 * pow(aa_x, 2);
	const GEN_FLT x10 = cos(x4);
	const GEN_FLT x11 = x1 * x7 + x7 * x9 + x8 * x7 + pow(x10, 2);
	const GEN_FLT x12 = pow(x11, -1.0 / 2.0);
	const GEN_FLT x13 = x12 * x10;
	const GEN_FLT x14 =
		((0 < pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2)) ? (0) : (0));
	const GEN_FLT x15 = 1.0 * x10;
	const GEN_FLT x16 = x5 * x15;
	const GEN_FLT x17 = x14 * x16;
	const GEN_FLT x18 = x3 * x9;
	const GEN_FLT x19 = 2 * x6 / pow(x2, 3);
	const GEN_FLT x20 = x8 * x19;
	const GEN_FLT x21 = x3 * x17;
	const GEN_FLT x22 = x1 * x19;
	const GEN_FLT x23 = x9 * x19;
	const GEN_FLT x24 = -x17 + x1 * x21 + x18 * x17 - x20 * x14 - x22 * x14 - x23 * x14 + x8 * x21;
	const GEN_FLT x25 = (1.0 / 2.0) / pow(x11, 3.0 / 2.0);
	const GEN_FLT x26 = x25 * x10;
	const GEN_FLT x27 = x24 * x26;
	const GEN_FLT x28 = 0.5 * x14;
	const GEN_FLT x29 = x5 * x12;
	const GEN_FLT x30 = x29 * obj_qw;
	const GEN_FLT x31 = x28 * x13;
	const GEN_FLT x32 = time / x2;
	const GEN_FLT x33 = x32 * obj_qk;
	const GEN_FLT x34 = x33 * aa_z;
	const GEN_FLT x35 = x5 * x25;
	const GEN_FLT x36 = x35 * x24;
	const GEN_FLT x37 = x29 * obj_qk;
	const GEN_FLT x38 = x3 * time;
	const GEN_FLT x39 = x38 * x14;
	const GEN_FLT x40 = x39 * aa_z;
	const GEN_FLT x41 = x32 * obj_qj;
	const GEN_FLT x42 = x41 * aa_y;
	const GEN_FLT x43 = x29 * obj_qj;
	const GEN_FLT x44 = x43 * aa_y;
	const GEN_FLT x45 = x32 * aa_x;
	const GEN_FLT x46 = x45 * obj_qi;
	const GEN_FLT x47 = x29 * obj_qi;
	const GEN_FLT x48 = x38 * aa_x;
	const GEN_FLT x49 = x48 * x14;
	const GEN_FLT x50 = -x27 * obj_qw - x30 * x28 - x31 * x34 + x34 * x36 + x40 * x37 - x42 * x31 + x42 * x36 +
						x44 * x39 - x46 * x31 + x46 * x36 + x47 * x49;
	const GEN_FLT x51 = x32 * x29;
	const GEN_FLT x52 = x51 * aa_x;
	const GEN_FLT x53 = x26 * obj_qi;
	const GEN_FLT x54 = x41 * aa_z;
	const GEN_FLT x55 = x33 * aa_y;
	const GEN_FLT x56 = x37 * aa_y;
	const GEN_FLT x57 = x45 * obj_qw;
	const GEN_FLT x58 = x40 * x43 - x47 * x28 - x49 * x30 - x53 * x24 - x54 * x31 + x54 * x36 + x55 * x31 - x55 * x36 -
						x56 * x39 + x57 * x31 - x57 * x36;
	const GEN_FLT x59 = x51 * aa_y;
	const GEN_FLT x60 = x32 * aa_z;
	const GEN_FLT x61 = x60 * obj_qi;
	const GEN_FLT x62 = x32 * aa_y;
	const GEN_FLT x63 = x62 * obj_qw;
	const GEN_FLT x64 = x39 * aa_y;
	const GEN_FLT x65 = x33 * aa_x;
	const GEN_FLT x66 = -x27 * obj_qj - x40 * x47 - x43 * x28 + x49 * x37 + x61 * x31 - x61 * x36 + x63 * x31 -
						x63 * x36 - x64 * x30 - x65 * x31 + x65 * x36;
	const GEN_FLT x67 = x51 * aa_z;
	const GEN_FLT x68 = x60 * obj_qw;
	const GEN_FLT x69 = x62 * obj_qi;
	const GEN_FLT x70 = x45 * obj_qj;
	const GEN_FLT x71 = -x27 * obj_qk - x37 * x28 - x40 * x30 - x43 * x49 + x64 * x47 + x68 * x31 - x68 * x36 -
						x69 * x31 + x69 * x36 + x70 * x31 - x70 * x36;
	const GEN_FLT x72 = -x52;
	const GEN_FLT x73 = -x59;
	const GEN_FLT x74 = -x67;
	const GEN_FLT x75 =
		((0 < pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2))
			 ? (pow(time, 2) * aa_x /
				sqrt(pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2)))
			 : (0));
	const GEN_FLT x76 = x5 * x75;
	const GEN_FLT x77 = x76 * x12;
	const GEN_FLT x78 = x77 * x48;
	const GEN_FLT x79 = x75 * x13;
	const GEN_FLT x80 = 0.5 * x79;
	const GEN_FLT x81 = x76 * x15;
	const GEN_FLT x82 = 2 * x0 * x7;
	const GEN_FLT x83 = x3 * x8;
	const GEN_FLT x84 = x75 * x19;
	const GEN_FLT x85 = x1 * x3;
	const GEN_FLT x86 = -x81 - x1 * x84 - x75 * x23 - x8 * x84 + x81 * x18 + x81 * x83 + x81 * x85 + x82 * aa_x;
	const GEN_FLT x87 = x86 * x35;
	const GEN_FLT x88 = x86 * x26;
	const GEN_FLT x89 = x38 * aa_z;
	const GEN_FLT x90 = x89 * x77;
	const GEN_FLT x91 = x38 * aa_y;
	const GEN_FLT x92 = x77 * x91;
	const GEN_FLT x93 = 0.5 * obj_qw;
	const GEN_FLT x94 = x47 * x32;
	const GEN_FLT x95 = -x94;
	const GEN_FLT x96 = 0.5 * x77;
	const GEN_FLT x97 = x79 * x93;
	const GEN_FLT x98 = x30 * x32;
	const GEN_FLT x99 = x32 * x37;
	const GEN_FLT x100 = -x99;
	const GEN_FLT x101 = x43 * x32;
	const GEN_FLT x102 = -x101;
	const GEN_FLT x103 =
		((0 < pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2))
			 ? (pow(time, 2) * aa_y /
				sqrt(pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2)))
			 : (0));
	const GEN_FLT x104 = x16 * x103;
	const GEN_FLT x105 =
		-x104 + x18 * x104 - x20 * x103 - x22 * x103 - x23 * x103 + x82 * aa_y + x83 * x104 + x85 * x104;
	const GEN_FLT x106 = x26 * x105;
	const GEN_FLT x107 = 0.5 * x103;
	const GEN_FLT x108 = x13 * x107;
	const GEN_FLT x109 = x38 * x103;
	const GEN_FLT x110 = x109 * aa_z;
	const GEN_FLT x111 = x48 * x103;
	const GEN_FLT x112 = x35 * x105;
	const GEN_FLT x113 = x93 * x13 * x103;
	const GEN_FLT x114 = x30 * x109;
	const GEN_FLT x115 =
		((0 < pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2))
			 ? (pow(time, 2) * aa_z /
				sqrt(pow(time, 2) * pow(aa_x, 2) + pow(time, 2) * pow(aa_y, 2) + pow(time, 2) * pow(aa_z, 2)))
			 : (0));
	const GEN_FLT x116 = x16 * x115;
	const GEN_FLT x117 =
		-x116 + x18 * x116 - x20 * x115 - x22 * x115 - x23 * x115 + x82 * aa_z + x83 * x116 + x85 * x116;
	const GEN_FLT x118 = x26 * x117;
	const GEN_FLT x119 = 0.5 * x115;
	const GEN_FLT x120 = x13 * x119;
	const GEN_FLT x121 = x89 * x115;
	const GEN_FLT x122 = x35 * x117;
	const GEN_FLT x123 = x48 * x115;
	const GEN_FLT x124 = x91 * x115;
	out[0] = x13 + x50;
	out[1] = x52 + x58;
	out[2] = x59 + x66;
	out[3] = x67 + x71;
	out[4] = 0;
	out[5] = 0;
	out[6] = 0;
	out[7] = x50 + x72;
	out[8] = x13 + x58;
	out[9] = x66 + x67;
	out[10] = x71 + x73;
	out[11] = 0;
	out[12] = 0;
	out[13] = 0;
	out[14] = x50 + x73;
	out[15] = x58 + x74;
	out[16] = x13 + x66;
	out[17] = x52 + x71;
	out[18] = 0;
	out[19] = 0;
	out[20] = 0;
	out[21] = x50 + x74;
	out[22] = x58 + x59;
	out[23] = x66 + x72;
	out[24] = x13 + x71;
	out[25] = 0;
	out[26] = 0;
	out[27] = 0;
	out[28] = x95 - x77 * x93 + x78 * obj_qi - x80 * x34 - x80 * x42 - x80 * x46 + x87 * x34 + x87 * x42 + x87 * x46 -
			  x88 * obj_qw + x90 * obj_qk + x92 * obj_qj;
	out[29] = x98 - x78 * obj_qw - x80 * x54 + x80 * x55 - x86 * x53 + x87 * x54 - x87 * x55 - x87 * x57 +
			  x90 * obj_qj - x92 * obj_qk - x96 * obj_qi + x97 * x45;
	out[30] = x100 + x62 * x97 + x78 * obj_qk + x80 * x61 - x80 * x65 - x87 * x61 - x87 * x63 + x87 * x65 -
			  x88 * obj_qj - x90 * obj_qi - x92 * obj_qw - x96 * obj_qj;
	out[31] = x101 + x60 * x97 - x78 * obj_qj - x80 * x69 + x80 * x70 - x87 * x68 + x87 * x69 - x87 * x70 -
			  x88 * obj_qk - x90 * obj_qw + x92 * obj_qi - x96 * obj_qk;
	out[32] = 1;
	out[33] = 0;
	out[34] = 0;
	out[35] = x102 - x106 * obj_qw - x30 * x107 - x34 * x108 + x34 * x112 + x37 * x110 - x42 * x108 + x42 * x112 +
			  x44 * x109 - x46 * x108 + x46 * x112 + x47 * x111;
	out[36] = x99 - x30 * x111 + x43 * x110 + x45 * x113 - x47 * x107 - x53 * x105 - x54 * x108 + x54 * x112 +
			  x55 * x108 - x55 * x112 - x56 * x109 - x57 * x112;
	out[37] = x98 - x106 * obj_qj - x114 * aa_y + x37 * x111 - x43 * x107 - x47 * x110 + x61 * x108 - x61 * x112 +
			  x62 * x113 - x63 * x112 - x65 * x108 + x65 * x112;
	out[38] = x95 - x106 * obj_qk - x114 * aa_z - x37 * x107 - x43 * x111 + x60 * x113 - x68 * x112 - x69 * x108 +
			  x69 * x112 + x70 * x108 - x70 * x112 + x47 * x109 * aa_y;
	out[39] = 0;
	out[40] = 1;
	out[41] = 0;
	out[42] = x100 - x118 * obj_qw - x30 * x119 - x34 * x120 + x34 * x122 + x37 * x121 - x42 * x120 + x42 * x122 -
			  x46 * x120 + x46 * x122 + x47 * x123 + x44 * x38 * x115;
	out[43] = x102 - x30 * x123 - x37 * x124 + x43 * x121 - x47 * x119 - x53 * x117 - x54 * x120 + x54 * x122 +
			  x55 * x120 - x55 * x122 + x57 * x120 - x57 * x122;
	out[44] = x94 - x118 * obj_qj - x30 * x124 + x37 * x123 - x43 * x119 - x47 * x121 + x61 * x120 - x61 * x122 +
			  x63 * x120 - x63 * x122 - x65 * x120 + x65 * x122;
	out[45] = x98 - x118 * obj_qk - x30 * x121 - x37 * x119 - x43 * x123 + x47 * x124 + x68 * x120 - x68 * x122 -
			  x69 * x120 + x69 * x122 + x70 * x120 - x70 * x122;
	out[46] = 0;
	out[47] = 0;
	out[48] = 1;
}

/** Applying function <function imu_rot_f_aa at 0x7f3db112ea70> */
static inline void gen_imu_rot_f_aa(FLT *out, const FLT time, const FLT *imu_rot_aa) {
	const GEN_FLT aa_x = imu_rot_aa[0];
	const GEN_FLT aa_y = imu_rot_aa[1];
	const GEN_FLT aa_z = imu_rot_aa[2];
	const GEN_FLT lh_qi = imu_rot_aa[3];
	const GEN_FLT lh_qj = imu_rot_aa[4];
	const GEN_FLT lh_qk = imu_rot_aa[5];
	const GEN_FLT x0 =
		((0 < pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)) ? (sqrt(pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)))
														  : (1e-10));
	const GEN_FLT x1 = 0.5 * x0;
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x2 / x0;
	const GEN_FLT x4 =
		((0 < pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2))
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
	const GEN_FLT x13 = x12 * pow(lh_qj, 2);
	const GEN_FLT x14 = pow(x6, 2) / pow(x4, 2);
	const GEN_FLT x15 = x12 * pow(lh_qk, 2);
	const GEN_FLT x16 = x12 * pow(lh_qi, 2);
	const GEN_FLT x17 = cos(x5);
	const GEN_FLT x18 =
		1 / (sqrt(x8 * x10 + x8 * x7 + x8 * x9 + pow(x11, 2)) * sqrt(x14 * x13 + x14 * x16 + x15 * x14 + pow(x17, 2)));
	const GEN_FLT x19 = x6 * x18 * time / x4;
	const GEN_FLT x20 = x3 * x19;
	const GEN_FLT x21 = x20 * lh_qk;
	const GEN_FLT x22 = x11 * x19;
	const GEN_FLT x23 = x20 * lh_qj;
	const GEN_FLT x24 = x18 * x17;
	const GEN_FLT x25 = x3 * x24;
	const GEN_FLT x26 = -x21 * aa_y + x22 * lh_qi + x23 * aa_z + x25 * aa_x;
	const GEN_FLT x27 = x20 * aa_z;
	const GEN_FLT x28 = x21 * aa_x + x22 * lh_qj + x25 * aa_y - x27 * lh_qi;
	const GEN_FLT x29 = x20 * lh_qi;
	const GEN_FLT x30 = -x23 * aa_y + x24 * x11 - x27 * lh_qk - x29 * aa_x;
	const GEN_FLT x31 = 1e-10 + pow(x26, 2) + pow(x28, 2) + pow(x30, 2);
	const GEN_FLT x32 = 2 * atan2(x31, x30) / x31;
	out[0] = x32 * x26;
	out[1] = x32 * x28;
	out[2] = x32 * (x22 * lh_qk - x23 * aa_x + x25 * aa_z + x29 * aa_y);
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
	const GEN_FLT x0 =
		((0 < pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2))
			 ? (sqrt(pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2)))
			 : (1e-10));
	const GEN_FLT x1 = 0.5 * x0;
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = pow(x0, -1);
	const GEN_FLT x4 = x2 * x3;
	const GEN_FLT x5 = x4 * lh_qk;
	const GEN_FLT x6 =
		((0 < pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)) ? (sqrt(pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)))
														  : (1e-10));
	const GEN_FLT x7 = 0.5 * x6;
	const GEN_FLT x8 = sin(x7);
	const GEN_FLT x9 = pow(x6, -1);
	const GEN_FLT x10 = x8 * x9;
	const GEN_FLT x11 = pow(aa_z, 2);
	const GEN_FLT x12 = pow(x6, -2);
	const GEN_FLT x13 = pow(x8, 2);
	const GEN_FLT x14 = x13 * x12;
	const GEN_FLT x15 = pow(aa_y, 2);
	const GEN_FLT x16 = pow(aa_x, 2);
	const GEN_FLT x17 = cos(x7);
	const GEN_FLT x18 = x14 * x11 + x14 * x16 + x15 * x14 + pow(x17, 2);
	const GEN_FLT x19 = pow(x18, -1.0 / 2.0);
	const GEN_FLT x20 = pow(time, 2);
	const GEN_FLT x21 = pow(lh_qj, 2);
	const GEN_FLT x22 = x20 * x21;
	const GEN_FLT x23 = pow(x2, 2);
	const GEN_FLT x24 = pow(x0, -2);
	const GEN_FLT x25 = x24 * x23;
	const GEN_FLT x26 = pow(lh_qk, 2);
	const GEN_FLT x27 = x20 * x26;
	const GEN_FLT x28 = pow(lh_qi, 2);
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = cos(x1);
	const GEN_FLT x31 = x25 * x22 + x25 * x27 + x25 * x29 + pow(x30, 2);
	const GEN_FLT x32 = pow(x31, -1.0 / 2.0);
	const GEN_FLT x33 = x32 * x19;
	const GEN_FLT x34 = x33 * x10;
	const GEN_FLT x35 = x5 * x34;
	const GEN_FLT x36 = x35 * aa_y;
	const GEN_FLT x37 = x32 * x17;
	const GEN_FLT x38 = x37 * x19;
	const GEN_FLT x39 = x3 * lh_qi;
	const GEN_FLT x40 = x2 * x39;
	const GEN_FLT x41 = x40 * x38;
	const GEN_FLT x42 = x41 * time;
	const GEN_FLT x43 = x4 * lh_qj;
	const GEN_FLT x44 = x34 * aa_z;
	const GEN_FLT x45 = x43 * x44;
	const GEN_FLT x46 = x30 * x19;
	const GEN_FLT x47 = x46 * x32;
	const GEN_FLT x48 = x47 * x10;
	const GEN_FLT x49 = x48 * aa_x;
	const GEN_FLT x50 = x42 + x49 - x36 * time + x45 * time;
	const GEN_FLT x51 = 2 * x50;
	const GEN_FLT x52 =
		((0 < pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2))
			 ? ((1.0 / 2.0) * (2 * time * pow(lh_qi, 2) + 2 * time * pow(lh_qj, 2) + 2 * time * pow(lh_qk, 2)) /
				sqrt(pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2)))
			 : (0));
	const GEN_FLT x53 = x2 * x52;
	const GEN_FLT x54 = 0.5 * x53;
	const GEN_FLT x55 = 1.0 * x53 * x30;
	const GEN_FLT x56 = 2 * time;
	const GEN_FLT x57 = x56 * x28;
	const GEN_FLT x58 = x55 * x24;
	const GEN_FLT x59 = 2 * x52 * x23 / pow(x0, 3);
	const GEN_FLT x60 = x56 * x21;
	const GEN_FLT x61 = x56 * x26;
	const GEN_FLT x62 = (1.0 / 2.0) *
						(-x55 + x57 * x25 + x58 * x22 + x58 * x27 + x58 * x29 - x59 * x22 - x59 * x27 - x59 * x29 +
						 x60 * x25 + x61 * x25) /
						pow(x31, 3.0 / 2.0);
	const GEN_FLT x63 = x62 * x46;
	const GEN_FLT x64 = x53 * x24 * time;
	const GEN_FLT x65 = x64 * lh_qk;
	const GEN_FLT x66 = x34 * aa_y;
	const GEN_FLT x67 = x64 * lh_qj;
	const GEN_FLT x68 = ((0 < pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)) ? (0) : (0));
	const GEN_FLT x69 = x9 * x68;
	const GEN_FLT x70 = 0.5 * x69;
	const GEN_FLT x71 = x70 * aa_x;
	const GEN_FLT x72 = x47 * aa_z;
	const GEN_FLT x73 = x72 * x10;
	const GEN_FLT x74 = x52 * time;
	const GEN_FLT x75 = 0.5 * x74;
	const GEN_FLT x76 = x3 * lh_qk;
	const GEN_FLT x77 = x75 * x76;
	const GEN_FLT x78 = x5 * time;
	const GEN_FLT x79 = x8 * x68;
	const GEN_FLT x80 = x79 * x12;
	const GEN_FLT x81 = x80 * x33;
	const GEN_FLT x82 = x81 * x78;
	const GEN_FLT x83 = x62 * x19;
	const GEN_FLT x84 = x10 * aa_z;
	const GEN_FLT x85 = x84 * x78;
	const GEN_FLT x86 = x40 * time;
	const GEN_FLT x87 = x10 * aa_x;
	const GEN_FLT x88 = 1.0 * x79 * x17;
	const GEN_FLT x89 = 2 * x68 * x13 / pow(x6, 3);
	const GEN_FLT x90 = x88 * x12;
	const GEN_FLT x91 = (1.0 / 2.0) * (-x88 - x89 * x11 - x89 * x15 - x89 * x16 + x90 * x11 + x90 * x15 + x90 * x16) /
						pow(x18, 3.0 / 2.0);
	const GEN_FLT x92 = x91 * x32;
	const GEN_FLT x93 = x87 * x92;
	const GEN_FLT x94 = x91 * x30;
	const GEN_FLT x95 = x79 * x47;
	const GEN_FLT x96 = x43 * time;
	const GEN_FLT x97 = x81 * x96;
	const GEN_FLT x98 = x3 * lh_qj;
	const GEN_FLT x99 = x48 * aa_y;
	const GEN_FLT x100 = x75 * x99;
	const GEN_FLT x101 = x10 * aa_y;
	const GEN_FLT x102 = x83 * x96;
	const GEN_FLT x103 = x81 * x86;
	const GEN_FLT x104 = x92 * x96;
	const GEN_FLT x105 = x35 * aa_z;
	const GEN_FLT x106 = x38 * lh_qj;
	const GEN_FLT x107 = x4 * x106;
	const GEN_FLT x108 = time * x107;
	const GEN_FLT x109 = x70 * x108;
	const GEN_FLT x110 = x66 * x43;
	const GEN_FLT x111 = x87 * x33;
	const GEN_FLT x112 = x64 * lh_qi;
	const GEN_FLT x113 = x83 * x86;
	const GEN_FLT x114 = x75 * x49;
	const GEN_FLT x115 = x5 * x38;
	const GEN_FLT x116 = time * x115;
	const GEN_FLT x117 = x70 * x116;
	const GEN_FLT x118 = x40 * x111;
	const GEN_FLT x119 = -x105 - x110 - x118 - 0.5 * x95 + x101 * x102 + x101 * x104 + x103 * aa_x - x109 * aa_y +
						 x111 * x112 - x117 * aa_z - x39 * x114 - x54 * x38 - x63 * x17 + x65 * x44 + x67 * x66 -
						 x71 * x42 - x73 * x77 + x82 * aa_z + x83 * x85 + x85 * x92 + x86 * x93 + x87 * x113 -
						 x94 * x37 + x97 * aa_y - x98 * x100;
	const GEN_FLT x120 = x46 * x37;
	const GEN_FLT x121 = x120 - time * x105 - time * x110 - time * x118;
	const GEN_FLT x122 = x94 * x32;
	const GEN_FLT x123 = 0.5 * x120;
	const GEN_FLT x124 = x74 * x123;
	const GEN_FLT x125 = x64 * x38;
	const GEN_FLT x126 = x91 * x37;
	const GEN_FLT x127 = x78 * x92;
	const GEN_FLT x128 = x83 * x78;
	const GEN_FLT x129 = 0.5 * x79 * x33;
	const GEN_FLT x130 = x73 * x75;
	const GEN_FLT x131 = x69 * x123;
	const GEN_FLT x132 = x83 * x17;
	const GEN_FLT x133 = x95 * x12;
	const GEN_FLT x134 =
		2 * (-x36 + x41 + x45 + x101 * x127 + x101 * x128 + x109 * aa_z - x117 * aa_y - x125 * lh_qi + x131 * aa_x -
			 x133 * aa_x + x39 * x124 - x54 * x111 + x65 * x66 - x67 * x44 - x77 * x99 + x82 * aa_y - x84 * x102 -
			 x84 * x104 - x86 * x126 - x86 * x129 - x86 * x132 - x87 * x122 - x87 * x63 - x97 * aa_z + x98 * x130);
	const GEN_FLT x135 = x5 * x111;
	const GEN_FLT x136 = x40 * x44;
	const GEN_FLT x137 = x108 + x99 + time * x135 - time * x136;
	const GEN_FLT x138 = x54 * x34;
	const GEN_FLT x139 = x63 * x10;
	const GEN_FLT x140 = x70 * x42;
	const GEN_FLT x141 = x86 * x92;
	const GEN_FLT x142 = x3 * x124;
	const GEN_FLT x143 =
		2 * (x107 + x135 - x136 - x101 * x122 + x103 * aa_z + x131 * aa_y - x133 * aa_y - x138 * aa_y - x139 * aa_y -
			 x140 * aa_z + x142 * lh_qj - x39 * x130 + x44 * x112 - x64 * x106 - x65 * x111 + x71 * x116 + x76 * x114 -
			 x82 * aa_x + x84 * x113 + x84 * x141 - x87 * x127 - x87 * x128 - x96 * x126 - x96 * x129 - x96 * x132);
	const GEN_FLT x144 = 2 * x119 * x121 + x137 * x143 + x50 * x134;
	const GEN_FLT x145 = pow(x121, 2);
	const GEN_FLT x146 = 1e-10 + x145 + pow(x137, 2) + pow(x50, 2);
	const GEN_FLT x147 = atan2(x146, x121);
	const GEN_FLT x148 = pow(x146, 2);
	const GEN_FLT x149 = x144 * x147 / x148;
	const GEN_FLT x150 = pow(x146, -1);
	const GEN_FLT x151 = x145 * x150 * (x144 / x121 - x119 * x146 / x145) / (x145 + x148);
	const GEN_FLT x152 = x147 * x150;
	const GEN_FLT x153 = 2 * x149;
	const GEN_FLT x154 = 2 * x151;
	const GEN_FLT x155 = x43 * x111;
	const GEN_FLT x156 = x66 * x40;
	const GEN_FLT x157 = x116 + x73 - time * x155 + time * x156;
	out[0] = x134 * x152 - x51 * x149 + x51 * x151;
	out[1] = -x137 * x153 + x137 * x154 + x143 * x152;
	out[2] = 2 * x152 *
				 (x115 - x155 + x156 - x101 * x113 - x101 * x141 - x103 * aa_y - x109 * aa_x - x125 * lh_qk +
				  x131 * aa_z - x138 * aa_z - x139 * aa_z + x140 * aa_y + x142 * lh_qk + x39 * x100 - x66 * x112 +
				  x67 * x111 - x78 * x126 - x78 * x129 - x78 * x132 - x80 * x72 - x84 * x122 + x87 * x102 + x93 * x96 +
				  x97 * aa_x - x98 * x114) -
			 x153 * x157 + x154 * x157;
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
		((0 < pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)) ? (sqrt(pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)))
														  : (1e-10));
	const GEN_FLT x1 = 0.5 * x0;
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = pow(x0, -1);
	const GEN_FLT x4 = x2 * x3;
	const GEN_FLT x5 =
		((0 < pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2))
			 ? (sqrt(pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2)))
			 : (1e-10));
	const GEN_FLT x6 = time / x5;
	const GEN_FLT x7 = pow(aa_z, 2);
	const GEN_FLT x8 = pow(x0, -2);
	const GEN_FLT x9 = pow(x2, 2);
	const GEN_FLT x10 = x8 * x9;
	const GEN_FLT x11 = pow(aa_y, 2);
	const GEN_FLT x12 = pow(aa_x, 2);
	const GEN_FLT x13 = cos(x1);
	const GEN_FLT x14 = x11 * x10 + x12 * x10 + x7 * x10 + pow(x13, 2);
	const GEN_FLT x15 = pow(x14, -1.0 / 2.0);
	const GEN_FLT x16 = 0.5 * x5;
	const GEN_FLT x17 = sin(x16);
	const GEN_FLT x18 = pow(time, 2);
	const GEN_FLT x19 = x18 * pow(lh_qj, 2);
	const GEN_FLT x20 = pow(x17, 2);
	const GEN_FLT x21 = pow(x5, -2);
	const GEN_FLT x22 = x20 * x21;
	const GEN_FLT x23 = x18 * pow(lh_qk, 2);
	const GEN_FLT x24 = x18 * pow(lh_qi, 2);
	const GEN_FLT x25 = cos(x16);
	const GEN_FLT x26 = x22 * x19 + x22 * x23 + x24 * x22 + pow(x25, 2);
	const GEN_FLT x27 = pow(x26, -1.0 / 2.0);
	const GEN_FLT x28 = x27 * x17;
	const GEN_FLT x29 = x28 * x15;
	const GEN_FLT x30 = x6 * x29;
	const GEN_FLT x31 = x4 * x30;
	const GEN_FLT x32 = x31 * lh_qi;
	const GEN_FLT x33 = -x32;
	const GEN_FLT x34 = 2 * x10;
	const GEN_FLT x35 =
		((0 < pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)) ? (aa_x / sqrt(pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)))
														  : (0));
	const GEN_FLT x36 = 2 * x9 / pow(x0, 3);
	const GEN_FLT x37 = x36 * x11;
	const GEN_FLT x38 = 1.0 * x13;
	const GEN_FLT x39 = x2 * x38;
	const GEN_FLT x40 = x35 * x39;
	const GEN_FLT x41 = x36 * x12;
	const GEN_FLT x42 = x8 * x12;
	const GEN_FLT x43 = x8 * x11;
	const GEN_FLT x44 = x7 * x36;
	const GEN_FLT x45 = x8 * x7;
	const GEN_FLT x46 = -x40 + x34 * aa_x - x35 * x37 + x40 * x42 + x40 * x43 + x40 * x45 - x41 * x35 - x44 * x35;
	const GEN_FLT x47 = x25 * x27;
	const GEN_FLT x48 = (1.0 / 2.0) / pow(x14, 3.0 / 2.0);
	const GEN_FLT x49 = x47 * x48;
	const GEN_FLT x50 = x49 * x13;
	const GEN_FLT x51 = x35 * aa_y;
	const GEN_FLT x52 = x15 * x13;
	const GEN_FLT x53 = x52 * x28;
	const GEN_FLT x54 = x6 * x53;
	const GEN_FLT x55 = x54 * lh_qj;
	const GEN_FLT x56 = x3 * x55;
	const GEN_FLT x57 = 0.5 * x56;
	const GEN_FLT x58 = x54 * lh_qi;
	const GEN_FLT x59 = x3 * x58;
	const GEN_FLT x60 = 0.5 * aa_x;
	const GEN_FLT x61 = x60 * x35;
	const GEN_FLT x62 = x48 * x28;
	const GEN_FLT x63 = x4 * aa_z;
	const GEN_FLT x64 = x6 * lh_qk;
	const GEN_FLT x65 = x63 * x64;
	const GEN_FLT x66 = x62 * x65;
	const GEN_FLT x67 = x2 * x30;
	const GEN_FLT x68 = x8 * x67;
	const GEN_FLT x69 = x68 * x51;
	const GEN_FLT x70 = x54 * lh_qk;
	const GEN_FLT x71 = 0.5 * aa_z;
	const GEN_FLT x72 = x71 * x70;
	const GEN_FLT x73 = x3 * x35;
	const GEN_FLT x74 = x8 * aa_z;
	const GEN_FLT x75 = x67 * x35;
	const GEN_FLT x76 = x75 * lh_qk;
	const GEN_FLT x77 = x6 * lh_qj;
	const GEN_FLT x78 = x4 * aa_y;
	const GEN_FLT x79 = x78 * x62;
	const GEN_FLT x80 = x79 * x77;
	const GEN_FLT x81 = x47 * x15;
	const GEN_FLT x82 = 0.5 * x81;
	const GEN_FLT x83 = x2 * x82;
	const GEN_FLT x84 = x4 * aa_x;
	const GEN_FLT x85 = x84 * x62;
	const GEN_FLT x86 = x6 * lh_qi;
	const GEN_FLT x87 = x86 * x46;
	const GEN_FLT x88 = x8 * aa_x;
	const GEN_FLT x89 = x75 * lh_qi;
	const GEN_FLT x90 =
		((0 < pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x91 = x90 * x17;
	const GEN_FLT x92 = 1.0 * x25;
	const GEN_FLT x93 = x92 * x91;
	const GEN_FLT x94 = x24 * x21;
	const GEN_FLT x95 = 2 * x20 / pow(x5, 3);
	const GEN_FLT x96 = x95 * x24;
	const GEN_FLT x97 = x93 * x21;
	const GEN_FLT x98 = x90 * x95;
	const GEN_FLT x99 = -x93 - x90 * x96 + x93 * x94 + x97 * x19 + x97 * x23 - x98 * x19 - x98 * x23;
	const GEN_FLT x100 = (1.0 / 2.0) / pow(x26, 3.0 / 2.0);
	const GEN_FLT x101 = x17 * x100;
	const GEN_FLT x102 = x15 * x101;
	const GEN_FLT x103 = x78 * x102;
	const GEN_FLT x104 = x99 * x103;
	const GEN_FLT x105 = x4 * x81;
	const GEN_FLT x106 = x105 * aa_z;
	const GEN_FLT x107 = 0.5 * x64;
	const GEN_FLT x108 = x107 * x106;
	const GEN_FLT x109 = x99 * x102;
	const GEN_FLT x110 = x91 * x27;
	const GEN_FLT x111 = 0.5 * x110;
	const GEN_FLT x112 = x84 * lh_qi;
	const GEN_FLT x113 = x6 * x101;
	const GEN_FLT x114 = x15 * x113;
	const GEN_FLT x115 = x112 * x114;
	const GEN_FLT x116 = x21 * time;
	const GEN_FLT x117 = x110 * x116;
	const GEN_FLT x118 = x15 * x117;
	const GEN_FLT x119 = x63 * lh_qk;
	const GEN_FLT x120 = 0.5 * lh_qj;
	const GEN_FLT x121 = x105 * aa_y;
	const GEN_FLT x122 = x90 * x121;
	const GEN_FLT x123 = x6 * x122;
	const GEN_FLT x124 = x78 * lh_qj;
	const GEN_FLT x125 = 0.5 * lh_qi;
	const GEN_FLT x126 = x105 * aa_x;
	const GEN_FLT x127 = x6 * x126;
	const GEN_FLT x128 = x90 * x127;
	const GEN_FLT x129 = x25 * x100;
	const GEN_FLT x130 = x52 * x129;
	const GEN_FLT x131 = x112 * x118 + x118 * x119 + x118 * x124 - x120 * x123 - x128 * x125 - x52 * x111 + x65 * x109 +
						 x77 * x104 - x90 * x108 + x99 * x115 - x99 * x130;
	const GEN_FLT x132 = x131 + x33 - x50 * x46 - x51 * x57 - x61 * x59 + x66 * x46 + x69 * lh_qj - x73 * x72 +
						 x74 * x76 + x80 * x46 - x83 * x35 + x85 * x87 + x88 * x89;
	const GEN_FLT x133 = x63 * x30;
	const GEN_FLT x134 = x78 * x30;
	const GEN_FLT x135 = x84 * x30;
	const GEN_FLT x136 = x52 * x47;
	const GEN_FLT x137 = x136 - x133 * lh_qk - x134 * lh_qj - x135 * lh_qi;
	const GEN_FLT x138 = 2 * x137;
	const GEN_FLT x139 = x79 * x64;
	const GEN_FLT x140 = x8 * x81;
	const GEN_FLT x141 = x2 * x140;
	const GEN_FLT x142 = x35 * x141;
	const GEN_FLT x143 = x84 * x49;
	const GEN_FLT x144 = 0.5 * x3 * x70;
	const GEN_FLT x145 = x71 * x55;
	const GEN_FLT x146 = x77 * x63;
	const GEN_FLT x147 = x62 * x146;
	const GEN_FLT x148 = x62 * x13;
	const GEN_FLT x149 = x86 * x148;
	const GEN_FLT x150 = x3 * x136;
	const GEN_FLT x151 = x60 * x150;
	const GEN_FLT x152 = x52 * lh_qi;
	const GEN_FLT x153 = x113 * x152;
	const GEN_FLT x154 = x15 * x129;
	const GEN_FLT x155 = x99 * x154;
	const GEN_FLT x156 = x6 * x136;
	const GEN_FLT x157 = x125 * x156;
	const GEN_FLT x158 = x78 * lh_qk;
	const GEN_FLT x159 = x63 * lh_qj;
	const GEN_FLT x160 = x6 * x106;
	const GEN_FLT x161 = x120 * x160;
	const GEN_FLT x162 = x15 * x111;
	const GEN_FLT x163 = -x107 * x122 - x109 * x146 - x117 * x152 + x118 * x158 - x118 * x159 + x64 * x104 -
						 x84 * x155 - x84 * x162 + x90 * x157 + x90 * x161 - x99 * x153;
	const GEN_FLT x164 = x105 + x163 - x142 * aa_x + x35 * x151 + x46 * x139 - x46 * x143 - x46 * x147 - x46 * x149 -
						 x51 * x144 + x69 * lh_qk + x73 * x145 - x75 * x125 - x75 * x74 * lh_qj;
	const GEN_FLT x165 = x126 + x58 + x133 * lh_qj - x134 * lh_qk;
	const GEN_FLT x166 = 2 * x165;
	const GEN_FLT x167 = x78 * x49;
	const GEN_FLT x168 = x31 * lh_qk;
	const GEN_FLT x169 = 0.5 * x150;
	const GEN_FLT x170 = x70 * x60;
	const GEN_FLT x171 = x77 * x148;
	const GEN_FLT x172 = x85 * x64;
	const GEN_FLT x173 = x71 * x59;
	const GEN_FLT x174 = x63 * x62;
	const GEN_FLT x175 = x63 * lh_qi;
	const GEN_FLT x176 = x114 * x175;
	const GEN_FLT x177 = x52 * x117;
	const GEN_FLT x178 = x107 * x126;
	const GEN_FLT x179 = x84 * lh_qk;
	const GEN_FLT x180 = x52 * x101;
	const GEN_FLT x181 = x99 * x180;
	const GEN_FLT x182 = x120 * x156;
	const GEN_FLT x183 = x84 * x102;
	const GEN_FLT x184 = x64 * x183;
	const GEN_FLT x185 = x125 * x160;
	const GEN_FLT x186 = x118 * x175 - x118 * x179 - x177 * lh_qj - x77 * x181 - x78 * x155 - x78 * x162 + x90 * x178 +
						 x90 * x182 - x90 * x185 + x99 * x176 - x99 * x184;
	const GEN_FLT x187 = x168 + x186 - x142 * aa_y - x35 * x173 - x46 * x167 - x46 * x171 - x46 * x172 + x51 * x169 +
						 x73 * x170 - x75 * x120 + x87 * x174 - x88 * x76 + x89 * x74;
	const GEN_FLT x188 = x121 + x55 - x133 * lh_qi + x135 * lh_qk;
	const GEN_FLT x189 = 2 * x188;
	const GEN_FLT x190 = x132 * x138 + x166 * x164 + x187 * x189;
	const GEN_FLT x191 = pow(x137, 2);
	const GEN_FLT x192 = 1e-10 + x191 + pow(x165, 2) + pow(x188, 2);
	const GEN_FLT x193 = atan2(x192, x137);
	const GEN_FLT x194 = pow(x192, 2);
	const GEN_FLT x195 = pow(x194, -1);
	const GEN_FLT x196 = x193 * x195;
	const GEN_FLT x197 = x166 * x196;
	const GEN_FLT x198 = pow(x137, -1);
	const GEN_FLT x199 = x192 / x191;
	const GEN_FLT x200 = x190 * x198 - x199 * x132;
	const GEN_FLT x201 = pow(x192, -1);
	const GEN_FLT x202 = x201 * x191 / (x191 + x194);
	const GEN_FLT x203 = x202 * x166;
	const GEN_FLT x204 = 2 * x193;
	const GEN_FLT x205 = x201 * x204;
	const GEN_FLT x206 = x189 * x196;
	const GEN_FLT x207 = x202 * x189;
	const GEN_FLT x208 = x106 + x70 + x134 * lh_qi - x135 * lh_qj;
	const GEN_FLT x209 = x208 * x204 * x195;
	const GEN_FLT x210 = 2 * x208;
	const GEN_FLT x211 = x210 * x202;
	const GEN_FLT x212 = x63 * x49;
	const GEN_FLT x213 = x88 * lh_qj;
	const GEN_FLT x214 = x64 * x148;
	const GEN_FLT x215 = x169 * aa_z;
	const GEN_FLT x216 = x31 * lh_qj;
	const GEN_FLT x217 = -x216;
	const GEN_FLT x218 = 0.5 * x59;
	const GEN_FLT x219 = 0.5 * lh_qk;
	const GEN_FLT x220 = x68 * lh_qi;
	const GEN_FLT x221 = x85 * x77;
	const GEN_FLT x222 = x77 * x183;
	const GEN_FLT x223 = x84 * lh_qj;
	const GEN_FLT x224 = x78 * lh_qi;
	const GEN_FLT x225 = x224 * x114;
	const GEN_FLT x226 = x219 * x156;
	const GEN_FLT x227 = -x120 * x128 + x123 * x125 - x177 * lh_qk + x223 * x118 - x224 * x118 - x63 * x155 -
						 x63 * x162 - x64 * x181 + x90 * x226 + x99 * x222 - x99 * x225;
	const GEN_FLT x228 =
		((0 < pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)) ? (aa_y / sqrt(pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)))
														  : (0));
	const GEN_FLT x229 = x39 * x228;
	const GEN_FLT x230 = x8 * x229;
	const GEN_FLT x231 =
		-x229 + x11 * x230 + x12 * x230 + x34 * aa_y - x37 * x228 - x41 * x228 - x44 * x228 + x7 * x230;
	const GEN_FLT x232 = x2 * x228;
	const GEN_FLT x233 = x30 * x232;
	const GEN_FLT x234 = x233 * lh_qk;
	const GEN_FLT x235 = x233 * lh_qj;
	const GEN_FLT x236 = x8 * aa_y;
	const GEN_FLT x237 = x3 * x228;
	const GEN_FLT x238 = 0.5 * aa_y;
	const GEN_FLT x239 = x70 * x237;
	const GEN_FLT x240 = x86 * x231;
	const GEN_FLT x241 = x77 * x231;
	const GEN_FLT x242 = x233 * lh_qi;
	const GEN_FLT x243 = x60 * x237;
	const GEN_FLT x244 = x131 + x217 + x236 * x235 - x50 * x231 - x58 * x243 + x66 * x231 - x71 * x239 + x74 * x234 +
						 x79 * x241 - x82 * x232 + x85 * x240 + x88 * x242 - x55 * x238 * x237;
	const GEN_FLT x245 = -x168;
	const GEN_FLT x246 = x231 * x148;
	const GEN_FLT x247 = x232 * x140;
	const GEN_FLT x248 = x163 + x245 + x228 * x151 + x231 * x139 - x231 * x143 - x231 * x147 - x233 * x125 +
						 x234 * x236 + x237 * x145 - x238 * x239 - x247 * aa_x - x74 * x235 - x86 * x246;
	const GEN_FLT x249 = x169 * aa_y;
	const GEN_FLT x250 = x58 * x237;
	const GEN_FLT x251 = x105 + x186 + x228 * x249 - x231 * x167 - x231 * x172 - x233 * x120 + x240 * x174 -
						 x247 * aa_y + x60 * x239 - x71 * x250 + x74 * x242 - x77 * x246 - x88 * x234;
	const GEN_FLT x252 = x244 * x138 + x248 * x166 + x251 * x189;
	const GEN_FLT x253 = -x244 * x199 + x252 * x198;
	const GEN_FLT x254 =
		((0 < pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)) ? (aa_z / sqrt(pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)))
														  : (0));
	const GEN_FLT x255 = x2 * x254;
	const GEN_FLT x256 = x38 * x255;
	const GEN_FLT x257 =
		-x256 + x34 * aa_z - x37 * x254 - x41 * x254 + x42 * x256 + x43 * x256 - x44 * x254 + x45 * x256;
	const GEN_FLT x258 = x30 * x255;
	const GEN_FLT x259 = x236 * x258;
	const GEN_FLT x260 = x3 * x254;
	const GEN_FLT x261 = x258 * lh_qk;
	const GEN_FLT x262 = x55 * x260;
	const GEN_FLT x263 = x86 * x257;
	const GEN_FLT x264 = x79 * x257;
	const GEN_FLT x265 = x59 * x254;
	const GEN_FLT x266 = x88 * x258;
	const GEN_FLT x267 = x131 + x245 - x238 * x262 + x259 * lh_qj + x266 * lh_qi - x50 * x257 - x60 * x265 +
						 x66 * x257 - x72 * x260 + x74 * x261 + x77 * x264 - x82 * x255 + x85 * x263;
	const GEN_FLT x268 = x255 * x140;
	const GEN_FLT x269 = x74 * x258;
	const GEN_FLT x270 = x163 + x216 + x236 * x261 + x254 * x151 + x257 * x139 - x257 * x143 - x257 * x147 -
						 x257 * x149 - x258 * x125 + x260 * x145 - x268 * aa_x - x269 * lh_qj - x70 * x238 * x260;
	const GEN_FLT x271 = x186 + x33 - x254 * x173 + x254 * x249 - x257 * x167 - x257 * x171 - x257 * x172 -
						 x258 * x120 + x260 * x170 + x263 * x174 - x268 * aa_y + x269 * lh_qi - x88 * x261;
	const GEN_FLT x272 = x267 * x138 + x270 * x166 + x271 * x189;
	const GEN_FLT x273 = -x267 * x199 + x272 * x198;
	const GEN_FLT x274 =
		((0 < pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2))
			 ? (pow(time, 2) * lh_qi /
				sqrt(pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2)))
			 : (0));
	const GEN_FLT x275 = x29 * x274 * x116;
	const GEN_FLT x276 = 0.5 * x274;
	const GEN_FLT x277 = x276 * x106;
	const GEN_FLT x278 = x276 * x121;
	const GEN_FLT x279 = x92 * x17;
	const GEN_FLT x280 = x279 * x274;
	const GEN_FLT x281 = 2 * x22 * x18;
	const GEN_FLT x282 = x21 * x280;
	const GEN_FLT x283 = x95 * x274;
	const GEN_FLT x284 =
		-x280 + x19 * x282 - x19 * x283 + x23 * x282 - x23 * x283 + x281 * lh_qi + x94 * x280 - x96 * x274;
	const GEN_FLT x285 = x284 * x102;
	const GEN_FLT x286 = x78 * x285;
	const GEN_FLT x287 = x276 * x126;
	const GEN_FLT x288 = -x135;
	const GEN_FLT x289 = ((0 < pow(aa_x, 2) + pow(aa_y, 2) + pow(aa_z, 2)) ? (0) : (0));
	const GEN_FLT x290 = x39 * x289;
	const GEN_FLT x291 = -x290 - x37 * x289 - x41 * x289 + x42 * x290 + x43 * x290 - x44 * x289 + x45 * x290;
	const GEN_FLT x292 = x86 * x291;
	const GEN_FLT x293 = x3 * x289;
	const GEN_FLT x294 = x60 * x293;
	const GEN_FLT x295 = x67 * x289;
	const GEN_FLT x296 = x74 * x295;
	const GEN_FLT x297 = x289 * aa_y;
	const GEN_FLT x298 = x68 * x297;
	const GEN_FLT x299 = x88 * x295;
	const GEN_FLT x300 = x296 * lh_qk + x298 * lh_qj + x299 * lh_qi - x50 * x291 - x57 * x297 - x58 * x294 +
						 x66 * x291 - x72 * x293 + x80 * x291 - x83 * x289 + x85 * x292;
	const GEN_FLT x301 = x288 + x300 + x275 * x112 + x275 * x119 + x275 * x124 + x284 * x115 - x284 * x130 -
						 x53 * x276 - x64 * x277 + x65 * x285 - x77 * x278 + x77 * x286 - x86 * x287;
	const GEN_FLT x302 = x276 * x156;
	const GEN_FLT x303 = x284 * x154;
	const GEN_FLT x304 = x53 * x116;
	const GEN_FLT x305 = x304 * lh_qi;
	const GEN_FLT x306 = x29 * x276;
	const GEN_FLT x307 = x289 * x141;
	const GEN_FLT x308 = x289 * x151 + x291 * x139 - x291 * x143 - x291 * x147 - x291 * x149 + x293 * x145 -
						 x295 * x125 - x296 * lh_qj - x297 * x144 + x298 * lh_qk - x307 * aa_x;
	const GEN_FLT x309 = x308 + x54 - x274 * x305 + x275 * x158 - x275 * x159 - x284 * x153 - x285 * x146 +
						 x302 * lh_qi - x64 * x278 + x64 * x286 + x77 * x277 - x84 * x303 - x84 * x306;
	const GEN_FLT x310 = -x133;
	const GEN_FLT x311 = x304 * lh_qj;
	const GEN_FLT x312 = x284 * x180;
	const GEN_FLT x313 = -x291 * x167 - x291 * x171 - x291 * x172 + x292 * x174 + x293 * x170 - x295 * x120 +
						 x296 * lh_qi - x297 * x141 + x297 * x169 - x299 * lh_qk - x71 * x58 * x293;
	const GEN_FLT x314 = x310 + x313 - x274 * x311 + x275 * x175 - x275 * x179 + x284 * x176 - x284 * x184 +
						 x302 * lh_qj + x64 * x287 - x77 * x312 - x78 * x303 - x78 * x306 - x86 * x277;
	const GEN_FLT x315 = x301 * x138 + x309 * x166 + x314 * x189;
	const GEN_FLT x316 = -x301 * x199 + x315 * x198;
	const GEN_FLT x317 = x304 * lh_qk;
	const GEN_FLT x318 = -x212 * x291 + x213 * x295 - x214 * x291 + x215 * x289 + x218 * x297 - x219 * x295 +
						 x291 * x221 - x297 * x220 - x307 * aa_z - x55 * x294 - x79 * x292;
	const GEN_FLT x319 =
		((0 < pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2))
			 ? (pow(time, 2) * lh_qj /
				sqrt(pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2)))
			 : (0));
	const GEN_FLT x320 = x279 * x319;
	const GEN_FLT x321 = x21 * x320;
	const GEN_FLT x322 = x95 * x319;
	const GEN_FLT x323 =
		-x320 + x19 * x321 - x19 * x322 + x23 * x321 - x23 * x322 + x24 * x321 + x281 * lh_qj - x96 * x319;
	const GEN_FLT x324 = x77 * x323;
	const GEN_FLT x325 = x6 * x121;
	const GEN_FLT x326 = x319 * x120;
	const GEN_FLT x327 = x29 * x319;
	const GEN_FLT x328 = x327 * x116;
	const GEN_FLT x329 = -x134;
	const GEN_FLT x330 = 0.5 * x53;
	const GEN_FLT x331 = x323 * x102;
	const GEN_FLT x332 = x319 * x125;
	const GEN_FLT x333 = x300 + x329 - x319 * x108 + x323 * x115 - x323 * x130 + x324 * x103 - x326 * x325 +
						 x328 * x112 + x328 * x119 + x328 * x124 - x330 * x319 - x332 * x127 + x65 * x331;
	const GEN_FLT x334 = x319 * x107;
	const GEN_FLT x335 = x323 * x154;
	const GEN_FLT x336 = x4 * x60;
	const GEN_FLT x337 = x133 + x308 - x305 * x319 + x319 * x157 + x319 * x161 - x323 * x153 + x328 * x158 -
						 x328 * x159 - x331 * x146 - x334 * x121 - x336 * x327 - x84 * x335 + x64 * x323 * x103;
	const GEN_FLT x338 = x323 * x180;
	const GEN_FLT x339 = 0.5 * x78;
	const GEN_FLT x340 = x323 * x114;
	const GEN_FLT x341 = x313 + x54 - x311 * x319 + x319 * x182 - x319 * x185 - x323 * x184 + x328 * x175 -
						 x328 * x179 + x334 * x126 - x339 * x327 + x340 * x175 - x77 * x338 - x78 * x335;
	const GEN_FLT x342 = x333 * x138 + x337 * x166 + x341 * x189;
	const GEN_FLT x343 = -x333 * x199 + x342 * x198;
	const GEN_FLT x344 = 0.5 * x63;
	const GEN_FLT x345 =
		((0 < pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2))
			 ? (pow(time, 2) * lh_qk /
				sqrt(pow(time, 2) * pow(lh_qi, 2) + pow(time, 2) * pow(lh_qj, 2) + pow(time, 2) * pow(lh_qk, 2)))
			 : (0));
	const GEN_FLT x346 = x29 * x345;
	const GEN_FLT x347 = x346 * x116;
	const GEN_FLT x348 = x279 * x345;
	const GEN_FLT x349 = x21 * x348;
	const GEN_FLT x350 = x95 * x345;
	const GEN_FLT x351 =
		-x348 + x19 * x349 - x19 * x350 + x23 * x349 - x23 * x350 + x281 * lh_qk + x94 * x348 - x96 * x345;
	const GEN_FLT x352 = x351 * x102;
	const GEN_FLT x353 = x345 * x120;
	const GEN_FLT x354 = x78 * x352;
	const GEN_FLT x355 = x345 * x125;
	const GEN_FLT x356 = x300 + x310 - x330 * x345 - x345 * x108 + x347 * x112 + x347 * x119 + x347 * x124 +
						 x351 * x115 - x351 * x130 - x353 * x325 - x355 * x127 + x65 * x352 + x77 * x354;
	const GEN_FLT x357 = x351 * x154;
	const GEN_FLT x358 = x308 + x329 - x305 * x345 - x336 * x346 + x345 * x157 + x345 * x161 + x347 * x158 -
						 x347 * x159 - x351 * x153 - x352 * x146 + x64 * x354 - x84 * x357 - x345 * x107 * x121;
	const GEN_FLT x359 = x351 * x180;
	const GEN_FLT x360 = x84 * x352;
	const GEN_FLT x361 = x347 * lh_qi;
	const GEN_FLT x362 = x135 + x313 - x311 * x345 - x339 * x346 + x345 * x178 + x345 * x182 - x347 * x179 +
						 x351 * x176 - x355 * x160 + x63 * x361 - x64 * x360 - x77 * x359 - x78 * x357;
	const GEN_FLT x363 = x356 * x138 + x358 * x166 + x362 * x189;
	const GEN_FLT x364 = (-x356 * x199 + x363 * x198) * x202;
	out[0] = -x190 * x197 + x200 * x203 + x205 * x164;
	out[1] = x200 * x207 + x205 * x187 - x206 * x190;
	out[2] = x205 * (x217 + x227 - x142 * aa_z + x35 * x215 - x46 * x212 - x46 * x214 + x46 * x221 + x51 * x218 -
					 x51 * x220 - x61 * x56 + x75 * x213 - x75 * x219 - x87 * x79) -
			 x209 * x190 + x211 * x200;
	out[3] = 0;
	out[4] = 0;
	out[5] = 0;
	out[6] = x203 * x253 + x205 * x248 - x252 * x197;
	out[7] = x205 * x251 - x206 * x252 + x207 * x253;
	out[8] = x205 * (x227 + x32 - x212 * x231 - x214 * x231 + x215 * x228 - x219 * x233 - x236 * x242 + x238 * x250 -
					 x247 * aa_z - x55 * x243 - x79 * x240 + x85 * x241 + x88 * x235) -
			 x209 * x252 + x211 * x253;
	out[9] = 0;
	out[10] = 0;
	out[11] = 0;
	out[12] = x203 * x273 + x205 * x270 - x272 * x197;
	out[13] = x205 * x271 - x206 * x272 + x207 * x273;
	out[14] = x205 * (x105 + x227 - x212 * x257 - x214 * x257 + x215 * x254 - x219 * x258 + x238 * x265 + x257 * x221 -
					  x259 * lh_qi + x266 * lh_qj - x268 * aa_z - x60 * x262 - x86 * x264) -
			  x209 * x272 + x211 * x273;
	out[15] = 0;
	out[16] = 0;
	out[17] = 0;
	out[18] = x203 * x316 + x205 * x309 - x315 * x197;
	out[19] = x205 * x314 - x206 * x315 + x207 * x316;
	out[20] = x205 * (x134 + x318 - x274 * x317 + x275 * x223 - x275 * x224 + x284 * x222 - x284 * x225 + x302 * lh_qk -
					  x63 * x303 - x63 * x306 - x64 * x312 - x77 * x287 + x86 * x278) -
			  x209 * x315 + x211 * x316;
	out[21] = 1;
	out[22] = 0;
	out[23] = 0;
	out[24] = x203 * x343 + x205 * x337 - x342 * x197;
	out[25] = x205 * x341 - x206 * x342 + x207 * x343;
	out[26] = x205 * (x288 + x318 + x223 * x328 - x224 * x328 - x224 * x340 + x226 * x319 - x317 * x319 + x324 * x183 +
					  x325 * x332 - x326 * x127 - x327 * x344 - x63 * x335 - x64 * x338) -
			  x209 * x342 + x211 * x343;
	out[27] = 0;
	out[28] = 1;
	out[29] = 0;
	out[30] = x205 * x358 - x363 * x197 + x364 * x166;
	out[31] = x205 * x362 - x206 * x363 + x364 * x189;
	out[32] = x205 * (x318 + x54 + x223 * x347 - x225 * x351 + x226 * x345 - x317 * x345 - x346 * x344 - x353 * x127 +
					  x355 * x325 - x63 * x357 - x64 * x359 + x77 * x360 - x78 * x361) -
			  x209 * x363 + x210 * x364;
	out[33] = 0;
	out[34] = 0;
	out[35] = 1;
}
