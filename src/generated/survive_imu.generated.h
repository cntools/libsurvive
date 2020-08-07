#pragma once
#include "common.h"
/** Applying function <function imu_rot_f at 0x7fefa15d6f80> */
static inline void gen_imu_rot_f(FLT *out, const FLT time, const FLT *imu_rot) {
	const GEN_FLT obj_qw = imu_rot[0];
	const GEN_FLT obj_qi = imu_rot[1];
	const GEN_FLT obj_qj = imu_rot[2];
	const GEN_FLT obj_qk = imu_rot[3];
	const GEN_FLT aa_x = imu_rot[4];
	const GEN_FLT aa_y = imu_rot[5];
	const GEN_FLT aa_z = imu_rot[6];
	const GEN_FLT x0 = time * time;
	const GEN_FLT x1 = x0 * (aa_z * aa_z);
	const GEN_FLT x2 =
		(1e-20 < (((time * time) * (aa_z * aa_z)) + ((time * time) * (aa_y * aa_y)) + ((time * time) * (aa_x * aa_x))))
			? sqrt(
				  (((time * time) * (aa_z * aa_z)) + ((time * time) * (aa_y * aa_y)) + ((time * time) * (aa_x * aa_x))))
			: 1e-10;
	const GEN_FLT x3 = 0.5 * x2;
	const GEN_FLT x4 = sin(x3);
	const GEN_FLT x5 = pow(x2, -2) * (x4 * x4);
	const GEN_FLT x6 = x0 * (aa_y * aa_y);
	const GEN_FLT x7 = x0 * (aa_x * aa_x);
	const GEN_FLT x8 = cos(x3);
	const GEN_FLT x9 = pow(((x1 * x5) + (x6 * x5) + (x5 * x7) + (x8 * x8)), -1.0 / 2.0);
	const GEN_FLT x10 = pow(x2, -1) * x4 * x9 * time;
	const GEN_FLT x11 = x10 * aa_z;
	const GEN_FLT x12 = x10 * aa_y;
	const GEN_FLT x13 = x10 * aa_x;
	const GEN_FLT x14 = x8 * x9;
	out[0] = (-1 * x11 * obj_qk) + (-1 * x12 * obj_qj) + (-1 * x13 * obj_qi) + (x14 * obj_qw);
	out[1] = (-1 * x11 * obj_qj) + (x12 * obj_qk) + (x13 * obj_qw) + (x14 * obj_qi);
	out[2] = (x11 * obj_qi) + (x12 * obj_qw) + (-1 * x13 * obj_qk) + (x14 * obj_qj);
	out[3] = (x11 * obj_qw) + (-1 * x12 * obj_qi) + (x13 * obj_qj) + (x14 * obj_qk);
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
		(1e-20 < (((time * time) * (aa_z * aa_z)) + ((time * time) * (aa_y * aa_y)) + ((time * time) * (aa_x * aa_x))))
			? sqrt(
				  (((time * time) * (aa_z * aa_z)) + ((time * time) * (aa_y * aa_y)) + ((time * time) * (aa_x * aa_x))))
			: 1e-10;
	const GEN_FLT x1 = 0.5 * x0;
	const GEN_FLT x2 = cos(x1);
	const GEN_FLT x3 = time * time;
	const GEN_FLT x4 = aa_z * aa_z;
	const GEN_FLT x5 = x4 * x3;
	const GEN_FLT x6 = sin(x1);
	const GEN_FLT x7 = x6 * x6;
	const GEN_FLT x8 = pow(x0, -2);
	const GEN_FLT x9 = x8 * x7;
	const GEN_FLT x10 = aa_y * aa_y;
	const GEN_FLT x11 = x3 * x10;
	const GEN_FLT x12 = aa_x * aa_x;
	const GEN_FLT x13 = x3 * x12;
	const GEN_FLT x14 = (x5 * x9) + (x9 * x11) + (x9 * x13) + (x2 * x2);
	const GEN_FLT x15 =
		(1e-20 < (((time * time) * (aa_z * aa_z)) + ((time * time) * (aa_y * aa_y)) + ((time * time) * (aa_x * aa_x))))
			? (1.0 / 2.0 * ((2 * time * (aa_x * aa_x)) + (2 * time * (aa_y * aa_y)) + (2 * time * (aa_z * aa_z))) *
			   pow((((time * time) * (aa_z * aa_z)) + ((time * time) * (aa_y * aa_y)) +
					((time * time) * (aa_x * aa_x))),
				   -1.0 / 2.0))
			: 0;
	const GEN_FLT x16 = x6 * x15;
	const GEN_FLT x17 = 1.0 * x2 * x16;
	const GEN_FLT x18 = x8 * x17;
	const GEN_FLT x19 = 2 * time;
	const GEN_FLT x20 = x10 * x19;
	const GEN_FLT x21 = 2 * pow(x0, -3) * x7 * x15;
	const GEN_FLT x22 = x12 * x19;
	const GEN_FLT x23 = x4 * x19;
	const GEN_FLT x24 = 1.0 / 2.0 * pow(x14, -3.0 / 2.0) *
						((x13 * x18) + (x9 * x20) + (-1 * x5 * x21) + (x11 * x18) + (-1 * x17) + (-1 * x21 * x11) +
						 (x9 * x23) + (x9 * x22) + (-1 * x21 * x13) + (x5 * x18));
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
	out[0] = (x46 * obj_qi) + (-1 * x42 * x37) + (x49 * obj_qj) + (-1 * x40 * x41) + (x51 * x29) + (-1 * x35 * aa_x) +
			 (-1 * x25 * obj_qw) + (-1 * x28 * x26) + (-1 * x32 * x29) + (x40 * x44) + (x48 * obj_qk) + (x36 * x39) +
			 (-1 * x47 * aa_y) + (-1 * x33 * x39);
	out[1] = (x33 * x38 * aa_y) + (x48 * obj_qj) + (x32 * x28) + (-1 * x40 * x42) + (-1 * x47 * aa_z) +
			 (-1 * x43 * x38) + (x53 * aa_x) + (x52 * x40) + (-1 * x51 * x28) + (-1 * x25 * obj_qi) + (-1 * x29 * x26) +
			 (-1 * x49 * obj_qk) + (x41 * x37) + (-1 * x46 * obj_qw);
	out[2] = (-1 * x52 * x29) + (-1 * x49 * obj_qw) + (-1 * x25 * obj_qj) + (x54 * x36) + (x41 * x28) +
			 (-1 * x40 * x26) + (-1 * x32 * x37) + (x42 * x29) + (-1 * x54 * x33) + (-1 * x44 * x28) + (x46 * obj_qk) +
			 (x35 * aa_z) + (x53 * aa_y) + (-1 * x48 * obj_qi);
	out[3] = (x49 * obj_qi) + (x40 * x32) + (-1 * x41 * x29) + (-1 * x35 * aa_y) + (x47 * aa_x) + (-1 * x37 * x26) +
			 (-1 * x48 * obj_qw) + (-1 * x51 * x40) + (-1 * x52 * x28) + (-1 * x46 * obj_qj) + (x42 * x28) +
			 (-1 * x25 * obj_qk) + (x44 * x29) + (x53 * aa_z);
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
	const GEN_FLT x0 = time * time;
	const GEN_FLT x1 = x0 * (aa_z * aa_z);
	const GEN_FLT x2 =
		(1e-20 < (((time * time) * (aa_z * aa_z)) + ((time * time) * (aa_y * aa_y)) + ((time * time) * (aa_x * aa_x))))
			? sqrt(
				  (((time * time) * (aa_z * aa_z)) + ((time * time) * (aa_y * aa_y)) + ((time * time) * (aa_x * aa_x))))
			: 1e-10;
	const GEN_FLT x3 = 0.5 * x2;
	const GEN_FLT x4 = sin(x3);
	const GEN_FLT x5 = x4 * x4;
	const GEN_FLT x6 = pow(x2, -2);
	const GEN_FLT x7 = x6 * x5;
	const GEN_FLT x8 = x0 * (aa_y * aa_y);
	const GEN_FLT x9 = x0 * (aa_x * aa_x);
	const GEN_FLT x10 = cos(x3);
	const GEN_FLT x11 = (x1 * x7) + (x8 * x7) + (x7 * x9) + (x10 * x10);
	const GEN_FLT x12 = pow(x11, -1.0 / 2.0);
	const GEN_FLT x13 = x12 * x10;
	const GEN_FLT x14 =
		(1e-20 < (((time * time) * (aa_z * aa_z)) + ((time * time) * (aa_y * aa_y)) + ((time * time) * (aa_x * aa_x))))
			? 0
			: 0;
	const GEN_FLT x15 = 1.0 * x10;
	const GEN_FLT x16 = x4 * x15;
	const GEN_FLT x17 = x14 * x16;
	const GEN_FLT x18 = x6 * x9;
	const GEN_FLT x19 = x6 * x8;
	const GEN_FLT x20 = 2 * pow(x2, -3) * x5;
	const GEN_FLT x21 = x8 * x20;
	const GEN_FLT x22 = x9 * x20;
	const GEN_FLT x23 = x1 * x6;
	const GEN_FLT x24 = x1 * x20;
	const GEN_FLT x25 =
		(-1 * x17) + (x18 * x17) + (x19 * x17) + (-1 * x21 * x14) + (-1 * x22 * x14) + (x23 * x17) + (-1 * x24 * x14);
	const GEN_FLT x26 = 1.0 / 2.0 * pow(x11, -3.0 / 2.0);
	const GEN_FLT x27 = x26 * x10;
	const GEN_FLT x28 = x27 * obj_qw;
	const GEN_FLT x29 = x4 * x12;
	const GEN_FLT x30 = x6 * time;
	const GEN_FLT x31 = x30 * x29;
	const GEN_FLT x32 = x31 * aa_z;
	const GEN_FLT x33 = x14 * obj_qk;
	const GEN_FLT x34 = pow(x2, -1) * time;
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
	const GEN_FLT x56 = (x50 * obj_qi) + (-1 * x25 * x28) + (x32 * x33) + (x52 * x44) + (-1 * x55 * obj_qw) +
						(-1 * x42 * x39) + (x46 * obj_qj) + (-1 * x53 * x52) + (x47 * x35) + (-1 * x38 * x14) +
						(x48 * x39);
	const GEN_FLT x57 = x51 * x29;
	const GEN_FLT x58 = x40 * x39;
	const GEN_FLT x59 = x35 * obj_qj;
	const GEN_FLT x60 = x25 * x27;
	const GEN_FLT x61 = x14 * aa_y;
	const GEN_FLT x62 = x34 * x37;
	const GEN_FLT x63 = x51 * obj_qw;
	const GEN_FLT x64 = (x59 * x44) + (-1 * x58 * x35) + (x32 * x39) + (x61 * x62) + (-1 * x63 * x44) +
						(-1 * x48 * x33) + (-1 * x60 * obj_qi) + (-1 * x55 * obj_qi) + (x63 * x53) +
						(-1 * x50 * obj_qw) + (-1 * x45 * x47);
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
	const GEN_FLT x76 = (x49 * x33) + (-1 * x55 * obj_qj) + (-1 * x60 * obj_qj) + (-1 * x75 * x14) + (x51 * x47) +
						(-1 * x68 * x14) + (-1 * x70 * x44) + (x72 * x61) + (x69 * x14) + (-1 * x74 * x44) +
						(-1 * x73 * x61);
	const GEN_FLT x77 = x65 * aa_z;
	const GEN_FLT x78 = x71 * aa_z;
	const GEN_FLT x79 = x61 * obj_qi;
	const GEN_FLT x80 = x73 * aa_z;
	const GEN_FLT x81 = x51 * obj_qj;
	const GEN_FLT x82 = (-1 * x49 * x39) + (-1 * x55 * obj_qk) + (-1 * x78 * x44) + (x79 * x31) + (x51 * x58) +
						(x46 * obj_qi) + (-1 * x80 * x14) + (-1 * x79 * x41) + (x78 * x53) + (-1 * x81 * x44) +
						(-1 * x60 * obj_qk);
	const GEN_FLT x83 = -1 * x57;
	const GEN_FLT x84 = -1 * x66;
	const GEN_FLT x85 = -1 * x77;
	const GEN_FLT x86 = 2 * x0 * x7;
	const GEN_FLT x87 =
		(1e-20 < (((time * time) * (aa_z * aa_z)) + ((time * time) * (aa_y * aa_y)) + ((time * time) * (aa_x * aa_x))))
			? ((time * time) * aa_x *
			   pow((((time * time) * (aa_z * aa_z)) + ((time * time) * (aa_y * aa_y)) +
					((time * time) * (aa_x * aa_x))),
				   -1.0 / 2.0))
			: 0;
	const GEN_FLT x88 = x87 * x16;
	const GEN_FLT x89 = (x86 * aa_x) + (-1 * x87 * x22) + (-1 * x87 * x21) + (x88 * x19) + (x88 * x18) + (x88 * x23) +
						(-1 * x88) + (-1 * x87 * x24);
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
	const GEN_FLT x100 = -1 * x99;
	const GEN_FLT x101 = x87 * x40;
	const GEN_FLT x102 = x89 * x27;
	const GEN_FLT x103 = x54 * obj_qi;
	const GEN_FLT x104 = x45 * x37;
	const GEN_FLT x105 = x65 * obj_qw;
	const GEN_FLT x106 = x87 * x73;
	const GEN_FLT x107 = x65 * obj_qk;
	const GEN_FLT x108 = -1 * x107;
	const GEN_FLT x109 = x51 * x90;
	const GEN_FLT x110 = x65 * obj_qj;
	const GEN_FLT x111 =
		(1e-20 < (((time * time) * (aa_z * aa_z)) + ((time * time) * (aa_y * aa_y)) + ((time * time) * (aa_x * aa_x))))
			? ((time * time) * aa_y *
			   pow((((time * time) * (aa_z * aa_z)) + ((time * time) * (aa_y * aa_y)) +
					((time * time) * (aa_x * aa_x))),
				   -1.0 / 2.0))
			: 0;
	const GEN_FLT x112 = x54 * x111;
	const GEN_FLT x113 = x32 * x111;
	const GEN_FLT x114 = x16 * x111;
	const GEN_FLT x115 = (-1 * x114) + (x18 * x114) + (-1 * x22 * x111) + (x86 * aa_y) + (-1 * x21 * x111) +
						 (x19 * x114) + (x23 * x114) + (-1 * x24 * x111);
	const GEN_FLT x116 = x43 * x115;
	const GEN_FLT x117 = x111 * aa_y;
	const GEN_FLT x118 = x31 * x117;
	const GEN_FLT x119 = x40 * x111;
	const GEN_FLT x120 = x27 * x115;
	const GEN_FLT x121 = x41 * x117;
	const GEN_FLT x122 = x45 * x116;
	const GEN_FLT x123 = -1 * x110;
	const GEN_FLT x124 = x49 * x111;
	const GEN_FLT x125 = x51 * x116;
	const GEN_FLT x126 =
		(1e-20 < (((time * time) * (aa_z * aa_z)) + ((time * time) * (aa_y * aa_y)) + ((time * time) * (aa_x * aa_x))))
			? ((time * time) * aa_z *
			   pow((((time * time) * (aa_z * aa_z)) + ((time * time) * (aa_y * aa_y)) +
					((time * time) * (aa_x * aa_x))),
				   -1.0 / 2.0))
			: 0;
	const GEN_FLT x127 = x4 * x126;
	const GEN_FLT x128 = x12 * x127;
	const GEN_FLT x129 = 0.5 * x128;
	const GEN_FLT x130 = x30 * x128;
	const GEN_FLT x131 = x130 * obj_qk;
	const GEN_FLT x132 = x42 * x126;
	const GEN_FLT x133 = x40 * x126;
	const GEN_FLT x134 = x15 * x127;
	const GEN_FLT x135 = x6 * x134;
	const GEN_FLT x136 = (-1 * x134) + (-1 * x22 * x126) + (x9 * x135) + (-1 * x21 * x126) + (x86 * aa_z) +
						 (x1 * x135) + (x8 * x135) + (-1 * x24 * x126);
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
	out[7] = x83 + x56;
	out[8] = x13 + x64;
	out[9] = x77 + x76;
	out[10] = x84 + x82;
	out[11] = 0;
	out[12] = 0;
	out[13] = 0;
	out[14] = x84 + x56;
	out[15] = x85 + x64;
	out[16] = x13 + x76;
	out[17] = x57 + x82;
	out[18] = 0;
	out[19] = 0;
	out[20] = 0;
	out[21] = x85 + x56;
	out[22] = x66 + x64;
	out[23] = x83 + x76;
	out[24] = x13 + x82;
	out[25] = 0;
	out[26] = 0;
	out[27] = 0;
	out[28] = (-1 * x96 * x95) + (x94 * x48) + x100 + (x93 * x90) + (-1 * x87 * x38) + (-1 * x89 * x28) + (x52 * x90) +
			  (-1 * x94 * x42) + (x91 * x32) + (x98 * obj_qi) + (-1 * x92 * obj_qw) + (x97 * obj_qj);
	out[29] = (-1 * x98 * obj_qw) + x105 + (-1 * x102 * obj_qi) + (x63 * x101) + (-1 * x91 * x48) +
			  (-1 * x94 * x40 * x35) + (x87 * x104) + (-1 * x97 * obj_qk) + (-1 * x63 * x90) + (-1 * x87 * x103) +
			  (x94 * x32) + (x59 * x90);
	out[30] = (x109 * obj_qk) + (-1 * x92 * obj_qj) + (-1 * x102 * obj_qj) + (-1 * x74 * x90) + (-1 * x87 * x68) +
			  (x74 * x101) + (x70 * x101) + (-1 * x70 * x90) + (-1 * x87 * x75) + (x98 * obj_qk) + x108 +
			  (-1 * x106 * aa_y);
	out[31] = (-1 * x98 * obj_qj) + (x78 * x101) + (x96 * x94) + (-1 * x78 * x90) + (-1 * x109 * obj_qj) + x110 +
			  (-1 * x92 * obj_qk) + (-1 * x102 * obj_qk) + (-1 * x106 * aa_z) + (x95 * x48) + (-1 * x95 * x42) +
			  (x97 * obj_qi);
	out[32] = 1;
	out[33] = 0;
	out[34] = 0;
	out[35] = x123 + (-1 * x112 * obj_qw) + (-1 * x38 * x111) + (x118 * obj_qj) + (-1 * x120 * obj_qw) + (x52 * x116) +
			  (x124 * obj_qi) + (x93 * x116) + (-1 * x52 * x119) + (-1 * x121 * obj_qj) + (x122 * obj_qj) +
			  (x113 * obj_qk);
	out[36] = (-1 * x122 * obj_qk) + (x113 * obj_qj) + x107 + (-1 * x120 * obj_qi) + (-1 * x103 * x111) +
			  (-1 * x63 * x116) + (-1 * x59 * x119) + (x63 * x119) + (x62 * x117) + (-1 * x124 * obj_qw) +
			  (x59 * x116) + (-1 * x118 * obj_qk);
	out[37] = x105 + (-1 * x74 * x116) + (x124 * obj_qk) + (-1 * x70 * x116) + (-1 * x120 * obj_qj) + (x69 * x111) +
			  (-1 * x73 * x117) + (-1 * x112 * obj_qj) + (-1 * x68 * x111) + (x72 * x117) + (-1 * x75 * x111) +
			  (x125 * obj_qk);
	out[38] = (x72 * x111 * aa_z) + (-1 * x121 * obj_qi) + (-1 * x112 * obj_qk) + (-1 * x124 * obj_qj) +
			  (-1 * x80 * x111) + (-1 * x78 * x116) + (x122 * obj_qi) + (x118 * obj_qi) + (-1 * x125 * obj_qj) + x100 +
			  (-1 * x120 * obj_qk) + (x81 * x119);
	out[39] = 0;
	out[40] = 1;
	out[41] = 0;
	out[42] = (-1 * x28 * x136) + (x140 * obj_qj) + (x139 * aa_x) + (x93 * x137) + x108 + (-1 * x38 * x126) +
			  (-1 * x129 * obj_qw) + (x138 * aa_y) + (x52 * x137) + (x131 * aa_z) + (-1 * x132 * obj_qj) +
			  (-1 * x52 * x133);
	out[43] = x123 + (x59 * x137) + (-1 * x142 * aa_x) + (-1 * x141 * obj_qi) + (x138 * aa_z) + (-1 * x129 * obj_qi) +
			  (x104 * x126) + (-1 * x63 * x137) + (-1 * x140 * obj_qk) + (x63 * x133) + (-1 * x59 * x133) +
			  (-1 * x131 * aa_y);
	out[44] = (x143 * obj_qk) + x99 + (-1 * x75 * x126) + (x69 * x126) + (-1 * x141 * obj_qj) + (-1 * x129 * obj_qj) +
			  (-1 * x74 * x137) + (-1 * x142 * aa_y) + (x131 * aa_x) + (x74 * x133) + (-1 * x67 * x130) +
			  (-1 * x70 * x137);
	out[45] = (x140 * obj_qi) + (-1 * x78 * x137) + (-1 * x143 * obj_qj) + (-1 * x36 * x128) + (-1 * x132 * obj_qi) +
			  (x78 * x133) + (-1 * x142 * aa_z) + (x81 * x133) + (-1 * x141 * obj_qk) + x105 + (-1 * x138 * aa_x) +
			  (x139 * aa_y);
	out[46] = 0;
	out[47] = 0;
	out[48] = 1;
}

/** Applying function <function imu_rot_f_aa at 0x7fef73309dd0> */
static inline void gen_imu_rot_f_aa(FLT *out, const FLT time, const FLT *imu_rot_aa) {
	const GEN_FLT aa_x = imu_rot_aa[0];
	const GEN_FLT aa_y = imu_rot_aa[1];
	const GEN_FLT aa_z = imu_rot_aa[2];
	const GEN_FLT lh_qi = imu_rot_aa[3];
	const GEN_FLT lh_qj = imu_rot_aa[4];
	const GEN_FLT lh_qk = imu_rot_aa[5];
	const GEN_FLT x0 = (1e-20 < ((aa_z * aa_z) + (aa_y * aa_y) + (aa_x * aa_x)))
						   ? sqrt(((aa_z * aa_z) + (aa_y * aa_y) + (aa_x * aa_x)))
						   : 1e-10;
	const GEN_FLT x1 = 0.5 * x0;
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = pow(x0, -1) * x2;
	const GEN_FLT x4 = (1e-20 < (((time * time) * (lh_qk * lh_qk)) + ((time * time) * (lh_qj * lh_qj)) +
								 ((time * time) * (lh_qi * lh_qi))))
						   ? sqrt((((time * time) * (lh_qk * lh_qk)) + ((time * time) * (lh_qj * lh_qj)) +
								   ((time * time) * (lh_qi * lh_qi))))
						   : 1e-10;
	const GEN_FLT x5 = 0.5 * x4;
	const GEN_FLT x6 = sin(x5);
	const GEN_FLT x7 = aa_z * aa_z;
	const GEN_FLT x8 = pow(x0, -2) * (x2 * x2);
	const GEN_FLT x9 = aa_y * aa_y;
	const GEN_FLT x10 = aa_x * aa_x;
	const GEN_FLT x11 = cos(x1);
	const GEN_FLT x12 = time * time;
	const GEN_FLT x13 = x12 * (lh_qk * lh_qk);
	const GEN_FLT x14 = pow(x4, -2) * (x6 * x6);
	const GEN_FLT x15 = x12 * (lh_qj * lh_qj);
	const GEN_FLT x16 = x12 * (lh_qi * lh_qi);
	const GEN_FLT x17 = cos(x5);
	const GEN_FLT x18 = pow(((x8 * x7) + (x8 * x9) + (x8 * x10) + (x11 * x11)), -1.0 / 2.0) *
						pow(((x14 * x13) + (x15 * x14) + (x14 * x16) + (x17 * x17)), -1.0 / 2.0);
	const GEN_FLT x19 = pow(x4, -1) * x6 * x18 * time;
	const GEN_FLT x20 = x3 * x19;
	const GEN_FLT x21 = x20 * lh_qj;
	const GEN_FLT x22 = x11 * x19;
	const GEN_FLT x23 = x20 * lh_qk;
	const GEN_FLT x24 = x18 * x17;
	const GEN_FLT x25 = x3 * x24;
	const GEN_FLT x26 = (x21 * aa_z) + (x22 * lh_qi) + (-1 * x23 * aa_y) + (x25 * aa_x);
	const GEN_FLT x27 = x20 * lh_qi;
	const GEN_FLT x28 = (x23 * aa_x) + (-1 * x27 * aa_z) + (x25 * aa_y) + (x22 * lh_qj);
	const GEN_FLT x29 = (-1 * x23 * aa_z) + (-1 * x21 * aa_y) + (-1 * x27 * aa_x) + (x24 * x11);
	const GEN_FLT x30 = 1e-10 + (x28 * x28) + (x26 * x26) + (x29 * x29);
	const GEN_FLT x31 = 2 * pow(x30, -1) * atan2(x30, x29);
	out[0] = x31 * x26;
	out[1] = x31 * x28;
	out[2] = x31 * ((x22 * lh_qk) + (-1 * x21 * aa_x) + (x27 * aa_y) + (x25 * aa_z));
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
	const GEN_FLT x0 = aa_z * aa_z;
	const GEN_FLT x1 = (1e-20 < ((aa_z * aa_z) + (aa_y * aa_y) + (aa_x * aa_x)))
						   ? sqrt(((aa_z * aa_z) + (aa_y * aa_y) + (aa_x * aa_x)))
						   : 1e-10;
	const GEN_FLT x2 = pow(x1, -2);
	const GEN_FLT x3 = 0.5 * x1;
	const GEN_FLT x4 = sin(x3);
	const GEN_FLT x5 = x4 * x4;
	const GEN_FLT x6 = x2 * x5;
	const GEN_FLT x7 = aa_y * aa_y;
	const GEN_FLT x8 = aa_x * aa_x;
	const GEN_FLT x9 = cos(x3);
	const GEN_FLT x10 = (x0 * x6) + (x6 * x7) + (x6 * x8) + (x9 * x9);
	const GEN_FLT x11 = pow(x10, -1.0 / 2.0);
	const GEN_FLT x12 = time * time;
	const GEN_FLT x13 = lh_qk * lh_qk;
	const GEN_FLT x14 = x13 * x12;
	const GEN_FLT x15 = (1e-20 < (((time * time) * (lh_qk * lh_qk)) + ((time * time) * (lh_qj * lh_qj)) +
								  ((time * time) * (lh_qi * lh_qi))))
							? sqrt((((time * time) * (lh_qk * lh_qk)) + ((time * time) * (lh_qj * lh_qj)) +
									((time * time) * (lh_qi * lh_qi))))
							: 1e-10;
	const GEN_FLT x16 = pow(x15, -2);
	const GEN_FLT x17 = 0.5 * x15;
	const GEN_FLT x18 = sin(x17);
	const GEN_FLT x19 = x18 * x18;
	const GEN_FLT x20 = x19 * x16;
	const GEN_FLT x21 = lh_qj * lh_qj;
	const GEN_FLT x22 = x21 * x12;
	const GEN_FLT x23 = lh_qi * lh_qi;
	const GEN_FLT x24 = x23 * x12;
	const GEN_FLT x25 = cos(x17);
	const GEN_FLT x26 = (x20 * x14) + (x25 * x25) + (x22 * x20) + (x24 * x20);
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
	const GEN_FLT x49 = (x36 * time) + x40 + (-1 * x44 * time) + x48;
	const GEN_FLT x50 =
		(1e-20 <
		 (((time * time) * (lh_qk * lh_qk)) + ((time * time) * (lh_qj * lh_qj)) + ((time * time) * (lh_qi * lh_qi))))
			? (1.0 / 2.0 *
			   ((2 * time * (lh_qi * lh_qi)) + (2 * time * (lh_qj * lh_qj)) + (2 * time * (lh_qk * lh_qk))) *
			   pow((((time * time) * (lh_qk * lh_qk)) + ((time * time) * (lh_qj * lh_qj)) +
					((time * time) * (lh_qi * lh_qi))),
				   -1.0 / 2.0))
			: 0;
	const GEN_FLT x51 = x50 * x18;
	const GEN_FLT x52 = 0.5 * x51;
	const GEN_FLT x53 = x47 * x28;
	const GEN_FLT x54 = 2 * time;
	const GEN_FLT x55 = x54 * x23;
	const GEN_FLT x56 = 2 * x50 * pow(x15, -3) * x19;
	const GEN_FLT x57 = 1.0 * x51 * x25;
	const GEN_FLT x58 = x57 * x16;
	const GEN_FLT x59 = x54 * x13;
	const GEN_FLT x60 = x54 * x21;
	const GEN_FLT x61 = pow(x26, -3.0 / 2.0) * x11 *
						((x55 * x20) + (-1 * x56 * x24) + (-1 * x56 * x22) + (x60 * x20) + (x58 * x24) + (-1 * x57) +
						 (x58 * x22) + (-1 * x56 * x14) + (x59 * x20) + (x58 * x14));
	const GEN_FLT x62 = 1.0 / 2.0 * x25;
	const GEN_FLT x63 = x62 * x47;
	const GEN_FLT x64 = (1e-20 < ((aa_z * aa_z) + (aa_y * aa_y) + (aa_x * aa_x))) ? 0 : 0;
	const GEN_FLT x65 = x4 * x64;
	const GEN_FLT x66 = x65 * x45;
	const GEN_FLT x67 = x2 * x66;
	const GEN_FLT x68 = x34 * lh_qi;
	const GEN_FLT x69 = x68 * time;
	const GEN_FLT x70 = 1.0 * x9 * x65;
	const GEN_FLT x71 = 2 * pow(x1, -3) * x5 * x64;
	const GEN_FLT x72 = x2 * x70;
	const GEN_FLT x73 =
		x27 * pow(x10, -3.0 / 2.0) *
		((-1 * x70) + (-1 * x8 * x71) + (x7 * x72) + (x8 * x72) + (-1 * x7 * x71) + (-1 * x0 * x71) + (x0 * x72));
	const GEN_FLT x74 = 1.0 / 2.0 * x9;
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
	const GEN_FLT x88 = 1.0 / 2.0 * x83;
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
	const GEN_FLT x111 = 1.0 / 2.0 * x98;
	const GEN_FLT x112 = x90 * aa_y;
	const GEN_FLT x113 = x77 * aa_y;
	const GEN_FLT x114 = x80 * x113;
	const GEN_FLT x115 = x91 * aa_y;
	const GEN_FLT x116 = x64 * x95;
	const GEN_FLT x117 = 2 * ((-1 * x69 * x100) + (-1 * x89 * x91) + x39 + (-1 * x67 * aa_x) + (x109 * x110) +
							  (-1 * x63 * x61) + (-1 * x83 * x86) + (-1 * x44) + (x111 * x112) + (x99 * x98) +
							  (x81 * x76) + (x46 * x116) + (-1 * x87 * x32) + (-1 * x73 * x63) + (x111 * x115) +
							  (-1 * x41 * x114) + (-1 * x69 * x92) + x36 + (-1 * x53 * x52) + (-1 * x106 * aa_y) +
							  (-1 * x75 * x69) + (-1 * x89 * x90) + (x43 * x101) + (-1 * x82 * x38) + (x97 * lh_qi));
	const GEN_FLT x118 = 1.0 / 2.0 * x93;
	const GEN_FLT x119 = x42 * x32;
	const GEN_FLT x120 = x111 * aa_z;
	const GEN_FLT x121 = x105 * aa_y;
	const GEN_FLT x122 = x46 * x104;
	const GEN_FLT x123 = x80 * x48;
	const GEN_FLT x124 = x33 * lh_qi;
	const GEN_FLT x125 = 1.0 / 2.0 * x69;
	const GEN_FLT x126 = x47 * x125;
	const GEN_FLT x127 = x85 * x69;
	const GEN_FLT x128 = x43 * x35;
	const GEN_FLT x129 = x82 * lh_qi;
	const GEN_FLT x130 = x68 * x53;
	const GEN_FLT x131 = (x88 * x112) + (-1 * x61 * x118) + (-1 * x76 * x114) + (-1 * x73 * x118) + (-1 * x128) +
						 (-1 * x52 * x37) + (x90 * x120) + (-1 * x109 * x121) + (x127 * aa_x) + (x83 * x99) +
						 (-0.5 * x66) + (x32 * x101) + (-1 * x81 * x41) + (-1 * x106 * aa_z) + (x61 * x126) +
						 (x87 * x43) + (-1 * x130) + (-1 * x119) + (-1 * x40 * x122) + (x53 * x129) +
						 (-1 * x124 * x123) + (x86 * x98) + (x73 * x126) + (x91 * x120) + (x88 * x115);
	const GEN_FLT x132 = (-1 * time * x119) + (-1 * time * x128) + (-1 * time * x130) + x94;
	const GEN_FLT x133 = x53 * x42;
	const GEN_FLT x134 = x68 * x32;
	const GEN_FLT x135 = (time * x133) + (-1 * time * x134) + x109 + x113;
	const GEN_FLT x136 = x29 * x116;
	const GEN_FLT x137 = x69 * aa_z;
	const GEN_FLT x138 = 1.0 / 2.0 * x137;
	const GEN_FLT x139 = x47 * x111;
	const GEN_FLT x140 = x62 * x30;
	const GEN_FLT x141 = x61 * x140;
	const GEN_FLT x142 = x85 * aa_x;
	const GEN_FLT x143 = x73 * x140;
	const GEN_FLT x144 =
		2 * ((-1 * x83 * x92) + (-1 * x141 * aa_y) + (-1 * x40 * x110) + (-1 * x83 * x100) + (x103 * x122) +
			 (-1 * x81 * x124) + (-1 * x134) + x108 + (x85 * x137) + x133 + (-1 * x73 * x139) + (x97 * lh_qj) +
			 (x41 * x123) + (-1 * x98 * x142) + (-1 * x82 * x107) + (-1 * x53 * x101) + (-1 * x61 * x139) +
			 (x90 * x138) + (-1 * x67 * aa_y) + (-1 * x143 * aa_y) + (-1 * x52 * x43) + (x91 * x138) +
			 (-1 * x83 * x75) + (x32 * x129) + (x136 * aa_y));
	const GEN_FLT x145 = (x49 * x117) + (x135 * x144) + (2 * x131 * x132);
	const GEN_FLT x146 = x132 * x132;
	const GEN_FLT x147 = 1e-10 + (x135 * x135) + (x49 * x49) + x146;
	const GEN_FLT x148 = atan2(x147, x132);
	const GEN_FLT x149 = x147 * x147;
	const GEN_FLT x150 = x145 * x148 * pow(x149, -1);
	const GEN_FLT x151 = 2 * x150;
	const GEN_FLT x152 = pow(x147, -1);
	const GEN_FLT x153 =
		pow((x149 + x146), -1) * x146 * x152 * ((pow(x132, -1) * x145) + (-1 * x131 * pow(x146, -1) * x147));
	const GEN_FLT x154 = 2 * x153;
	const GEN_FLT x155 = x148 * x152;
	const GEN_FLT x156 = 2 * x135;
	const GEN_FLT x157 = x53 * x35;
	const GEN_FLT x158 = x68 * x43;
	const GEN_FLT x159 = x103 + (-1 * time * x157) + (time * x158) + x78;
	const GEN_FLT x160 = x88 * x47;
	out[0] = (-1 * x49 * x151) + (x49 * x154) + (x117 * x155);
	out[1] = (-1 * x150 * x156) + (x153 * x156) + (x144 * x155);
	out[2] =
		(-1 * x151 * x159) + (x154 * x159) +
		(2 * x155 *
		 ((-1 * x43 * x129) + (-1 * x157) + (-1 * x75 * x98) + (x61 * x160) + (-1 * x127 * aa_y) + (-1 * x109 * x122) +
		  (x73 * x160) + (-1 * x92 * x98) + (-1 * x37 * x101) + (-1 * x67 * aa_z) + x158 + (-1 * x141 * aa_z) + x102 +
		  (x83 * x142) + (x136 * aa_z) + (-1 * x52 * x32) + (-1 * x143 * aa_z) + (x96 * x41) + (-1 * x115 * x125) +
		  (-1 * x76 * x123) + (-1 * x112 * x125) + (x87 * x53) + (-1 * x98 * x100) + (x40 * x121) + (x114 * x124)));
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
	const GEN_FLT x0 = (1e-20 < (((time * time) * (lh_qk * lh_qk)) + ((time * time) * (lh_qj * lh_qj)) +
								 ((time * time) * (lh_qi * lh_qi))))
						   ? sqrt((((time * time) * (lh_qk * lh_qk)) + ((time * time) * (lh_qj * lh_qj)) +
								   ((time * time) * (lh_qi * lh_qi))))
						   : 1e-10;
	const GEN_FLT x1 = pow(x0, -1) * time;
	const GEN_FLT x2 = aa_z * aa_z;
	const GEN_FLT x3 = (1e-20 < ((aa_z * aa_z) + (aa_y * aa_y) + (aa_x * aa_x)))
						   ? sqrt(((aa_z * aa_z) + (aa_y * aa_y) + (aa_x * aa_x)))
						   : 1e-10;
	const GEN_FLT x4 = pow(x3, -2);
	const GEN_FLT x5 = 0.5 * x3;
	const GEN_FLT x6 = sin(x5);
	const GEN_FLT x7 = x6 * x6;
	const GEN_FLT x8 = x4 * x7;
	const GEN_FLT x9 = aa_y * aa_y;
	const GEN_FLT x10 = aa_x * aa_x;
	const GEN_FLT x11 = cos(x5);
	const GEN_FLT x12 = (x2 * x8) + (x8 * x9) + (x8 * x10) + (x11 * x11);
	const GEN_FLT x13 = pow(x12, -1.0 / 2.0);
	const GEN_FLT x14 = 0.5 * x0;
	const GEN_FLT x15 = sin(x14);
	const GEN_FLT x16 = x15 * x13;
	const GEN_FLT x17 = time * time;
	const GEN_FLT x18 = x17 * (lh_qk * lh_qk);
	const GEN_FLT x19 = pow(x0, -2);
	const GEN_FLT x20 = x15 * x15;
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = x17 * (lh_qj * lh_qj);
	const GEN_FLT x23 = x17 * (lh_qi * lh_qi);
	const GEN_FLT x24 = cos(x14);
	const GEN_FLT x25 = (x21 * x18) + (x24 * x24) + (x22 * x21) + (x23 * x21);
	const GEN_FLT x26 = pow(x25, -1.0 / 2.0);
	const GEN_FLT x27 = pow(x3, -1);
	const GEN_FLT x28 = x6 * x27;
	const GEN_FLT x29 = x28 * x26;
	const GEN_FLT x30 = x29 * x16;
	const GEN_FLT x31 = x1 * x30;
	const GEN_FLT x32 = x31 * lh_qi;
	const GEN_FLT x33 = -1 * x32;
	const GEN_FLT x34 = (1e-20 < ((aa_z * aa_z) + (aa_y * aa_y) + (aa_x * aa_x)))
							? (aa_x * pow(((aa_z * aa_z) + (aa_y * aa_y) + (aa_x * aa_x)), -1.0 / 2.0))
							: 0;
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
	const GEN_FLT x46 = 2 * pow(x3, -3) * x7;
	const GEN_FLT x47 = x46 * x10;
	const GEN_FLT x48 = x9 * x46;
	const GEN_FLT x49 = x4 * x9;
	const GEN_FLT x50 = x2 * x46;
	const GEN_FLT x51 = x2 * x4;
	const GEN_FLT x52 = (-1 * x43) + (x44 * aa_x) + (-1 * x48 * x34) + (x43 * x49) + (x43 * x45) + (-1 * x47 * x34) +
						(-1 * x50 * x34) + (x51 * x43);
	const GEN_FLT x53 = x29 * aa_y;
	const GEN_FLT x54 = 1.0 / 2.0 * pow(x12, -3.0 / 2.0);
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
	const GEN_FLT x95 = (1e-20 < (((time * time) * (lh_qk * lh_qk)) + ((time * time) * (lh_qj * lh_qj)) +
								  ((time * time) * (lh_qi * lh_qi))))
							? 0
							: 0;
	const GEN_FLT x96 = 1.0 * x24;
	const GEN_FLT x97 = x96 * x15;
	const GEN_FLT x98 = x97 * x95;
	const GEN_FLT x99 = 2 * pow(x0, -3) * x20;
	const GEN_FLT x100 = x99 * x22;
	const GEN_FLT x101 = x23 * x19;
	const GEN_FLT x102 = x22 * x19;
	const GEN_FLT x103 = x99 * x23;
	const GEN_FLT x104 = x99 * x18;
	const GEN_FLT x105 = x19 * x18;
	const GEN_FLT x106 = (-1 * x98) + (x98 * x105) + (-1 * x95 * x100) + (x98 * x101) + (x98 * x102) +
						 (-1 * x95 * x103) + (-1 * x95 * x104);
	const GEN_FLT x107 = 1.0 / 2.0 * pow(x25, -3.0 / 2.0);
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
	const GEN_FLT x135 = (x123 * x125) + (x133 * x134) + (-1 * x95 * x110) + (x118 * x116) + (x106 * x122) +
						 (-1 * x113 * x131) + (x37 * x109) + (-1 * x106 * x127) + (x128 * x129) + (-1 * x112 * x114) +
						 (-1 * x119 * x120);
	const GEN_FLT x136 = (x84 * x85) + (x82 * aa_z) + (x78 * x76) + (-1 * x93 * x34) + (-1 * x60 * x52) + x33 +
						 (x40 * x35) + (-1 * x74 * x34) + (-1 * x89 * x87) + (x52 * x57) + x135 + (-1 * x66 * x34) +
						 (x70 * x67);
	const GEN_FLT x137 = x79 * x76;
	const GEN_FLT x138 = x30 * x36;
	const GEN_FLT x139 = x79 * x67;
	const GEN_FLT x140 = x58 * x90;
	const GEN_FLT x141 = (-1 * x137 * lh_qk) + (-1 * x138 * aa_y) + (-1 * x139 * lh_qi) + x140;
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
	const GEN_FLT x168 = (-1 * x119 * x114) + (x113 * x156) + (-1 * x88 * x157) + (-1 * x117 * x125) + (x95 * x166) +
						 (-1 * x108 * x158) + (x95 * x165) + (-1 * x128 * x167) + (x106 * x159) + (-1 * x95 * x161) +
						 (-1 * x83 * x163);
	const GEN_FLT x169 = (x78 * x53) + (x88 * x152) + (x82 * aa_y) + (-1 * x34 * x155) + (-1 * x76 * x52 * x56) + x111 +
						 (-1 * x52 * x144) + (-1 * x35 * x145) + (-1 * x35 * x148) + (-1 * x52 * x146) + (x34 * x149) +
						 x168 + (-1 * x34 * x150);
	const GEN_FLT x170 = x1 * x123;
	const GEN_FLT x171 = (x76 * x37) + x86 + (-1 * x170 * lh_qk) + x130;
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
	const GEN_FLT x187 = (-1 * x106 * x186) + (x75 * x129) + (-1 * x116 * x133) + (-1 * x61 * x157) +
						 (-1 * x106 * x181) + (-1 * x37 * x163) + (x113 * x182) + (-1 * x113 * x183) + (x95 * x184) +
						 (-1 * x95 * x185) + (x118 * x134);
	const GEN_FLT x188 = (-1 * x52 * x180) + (x35 * x177) + (-1 * x34 * x173) + x187 + (x61 * x152) +
						 (-1 * x52 * x175) + (x70 * x76) + (-1 * x82 * aa_x) + x174 + (x89 * x73) + (-1 * x78 * x67) +
						 (-1 * x34 * x179) + (-1 * x176 * aa_y);
	const GEN_FLT x189 = (x139 * lh_qk) + (-1 * x137 * lh_qi) + x64 + x119;
	const GEN_FLT x190 = 2 * x189;
	const GEN_FLT x191 = (x136 * x142) + (x169 * x172) + (x188 * x190);
	const GEN_FLT x192 = x141 * x141;
	const GEN_FLT x193 = 1e-10 + (x189 * x189) + (x171 * x171) + x192;
	const GEN_FLT x194 = atan2(x193, x141);
	const GEN_FLT x195 = x193 * x193;
	const GEN_FLT x196 = pow(x195, -1);
	const GEN_FLT x197 = x196 * x194;
	const GEN_FLT x198 = x172 * x197;
	const GEN_FLT x199 = pow(x141, -1);
	const GEN_FLT x200 = pow(x192, -1) * x193;
	const GEN_FLT x201 = (x191 * x199) + (-1 * x200 * x136);
	const GEN_FLT x202 = pow(x193, -1);
	const GEN_FLT x203 = pow((x195 + x192), -1) * x202 * x192;
	const GEN_FLT x204 = x203 * x172;
	const GEN_FLT x205 = 2 * x194;
	const GEN_FLT x206 = x202 * x205;
	const GEN_FLT x207 = x190 * x197;
	const GEN_FLT x208 = x203 * x190;
	const GEN_FLT x209 = x72 + (-1 * x67 * x37) + (x170 * lh_qi) + x112;
	const GEN_FLT x210 = x209 * x205;
	const GEN_FLT x211 = x210 * x196;
	const GEN_FLT x212 = 2 * x203 * x209;
	const GEN_FLT x213 = -1 * x138;
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
	const GEN_FLT x226 = (-1 * x95 * x225) + (-1 * x75 * x167) + (-1 * x80 * x163) + (x224 * x113) + (-1 * x95 * x220) +
						 (x221 * x108) + (-1 * x120 * x130) + (-1 * x83 * x109) + (x95 * x223) + (-1 * x95 * x222) +
						 (x124 * x133);
	const GEN_FLT x227 = (1e-20 < ((aa_z * aa_z) + (aa_y * aa_y) + (aa_x * aa_x)))
							 ? (aa_y * pow(((aa_z * aa_z) + (aa_y * aa_y) + (aa_x * aa_x)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x228 = x42 * x227;
	const GEN_FLT x229 = x46 * x227;
	const GEN_FLT x230 = (-1 * x228) + (-1 * x47 * x227) + (-1 * x9 * x229) + (x45 * x228) + (x44 * aa_y) +
						 (x49 * x228) + (-1 * x2 * x229) + (x51 * x228);
	const GEN_FLT x231 = x88 * x227;
	const GEN_FLT x232 = x6 * x227;
	const GEN_FLT x233 = x81 * x232;
	const GEN_FLT x234 = x68 * x55;
	const GEN_FLT x235 = x67 * x234;
	const GEN_FLT x236 = x232 * aa_x;
	const GEN_FLT x237 = x76 * x230;
	const GEN_FLT x238 = x77 * x55;
	const GEN_FLT x239 = (x84 * x236) + (x230 * x235) + x135 + (-1 * x93 * x227) + (x238 * x237) + (x57 * x230) +
						 (-1 * x66 * x227) + (-1 * x87 * x231) + (x233 * aa_z) + x213 + (-1 * x74 * x227) +
						 (x40 * x232) + (-1 * x60 * x230);
	const GEN_FLT x240 = -1 * x174;
	const GEN_FLT x241 = x53 * x230;
	const GEN_FLT x242 = x59 * x230;
	const GEN_FLT x243 = x227 * x151;
	const GEN_FLT x244 = x230 * x143;
	const GEN_FLT x245 = (-1 * x227 * x155) + (-1 * x56 * x237) + (-1 * x232 * x145) + x240 + x168 + (x227 * x149) +
						 (x238 * x241) + (-1 * x227 * x150) + (-1 * x232 * x148) + (-1 * x67 * x242) + (x88 * x243) +
						 (-1 * x68 * x244) + (x233 * aa_y);
	const GEN_FLT x246 = x67 * x238;
	const GEN_FLT x247 = x84 * x232;
	const GEN_FLT x248 = x227 * x154;
	const GEN_FLT x249 = (-1 * x53 * x242) + (-1 * x248 * aa_y) + x111 + (x234 * x237) + (-1 * x230 * x246) +
						 (-1 * x227 * x173) + (x61 * x243) + (-1 * x233 * aa_x) + (x247 * aa_z) + (-1 * x36 * x244) +
						 (-1 * x227 * x179) + (x73 * x231) + x187;
	const GEN_FLT x250 = (x239 * x142) + (x245 * x172) + (x249 * x190);
	const GEN_FLT x251 = (x250 * x199) + (-1 * x239 * x200);
	const GEN_FLT x252 = (1e-20 < ((aa_z * aa_z) + (aa_y * aa_y) + (aa_x * aa_x)))
							 ? (aa_z * pow(((aa_z * aa_z) + (aa_y * aa_y) + (aa_x * aa_x)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x253 = x6 * x252;
	const GEN_FLT x254 = x81 * x253;
	const GEN_FLT x255 = x42 * x252;
	const GEN_FLT x256 = (-1 * x255) + (x45 * x255) + (-1 * x47 * x252) + (-1 * x48 * x252) + (x49 * x255) +
						 (x44 * aa_z) + (-1 * x50 * x252) + (x51 * x255);
	const GEN_FLT x257 = x76 * x256;
	const GEN_FLT x258 = x252 * aa_x;
	const GEN_FLT x259 = x6 * x258;
	const GEN_FLT x260 = 0.5 * x258;
	const GEN_FLT x261 = (x235 * x256) + (-1 * x74 * x252) + (-1 * x60 * x256) + (-1 * x87 * x260) + x240 +
						 (-1 * x93 * x252) + (x254 * aa_z) + (x238 * x257) + (x84 * x259) + (x57 * x256) + x135 +
						 (x40 * x253) + (-1 * x66 * x252);
	const GEN_FLT x262 = x53 * x238;
	const GEN_FLT x263 = x61 * x252;
	const GEN_FLT x264 = x256 * x143;
	const GEN_FLT x265 = x138 + (-1 * x73 * x263) + (x254 * aa_y) + x168 + (-1 * x258 * x154) + (-1 * x256 * x146) +
						 (-1 * x56 * x257) + (x252 * x149) + (-1 * x253 * x145) + (-1 * x253 * x148) + (x262 * x256) +
						 (-1 * x68 * x264) + (x260 * x151);
	const GEN_FLT x266 = x252 * x151;
	const GEN_FLT x267 = x252 * x154;
	const GEN_FLT x268 = (x253 * x177) + x33 + (-1 * x253 * x178) + (-1 * x36 * x264) + (x61 * x266) +
						 (-1 * x256 * x246) + x187 + (x73 * x260) + (-1 * x81 * x259) + (-1 * x267 * aa_y) +
						 (-1 * x256 * x175) + (x234 * x257) + (-1 * x252 * x173);
	const GEN_FLT x269 = (x261 * x142) + (x268 * x190) + (x265 * x172);
	const GEN_FLT x270 = (x269 * x199) + (-1 * x200 * x261);
	const GEN_FLT x271 = x53 * x234;
	const GEN_FLT x272 = -1 * x139;
	const GEN_FLT x273 = (1e-20 < (((time * time) * (lh_qk * lh_qk)) + ((time * time) * (lh_qj * lh_qj)) +
								   ((time * time) * (lh_qi * lh_qi))))
							 ? ((time * time) * lh_qi *
								pow((((time * time) * (lh_qk * lh_qk)) + ((time * time) * (lh_qj * lh_qj)) +
									 ((time * time) * (lh_qi * lh_qi))),
									-1.0 / 2.0))
							 : 0;
	const GEN_FLT x274 = 0.5 * x273;
	const GEN_FLT x275 = x273 * x117;
	const GEN_FLT x276 = x36 * x274;
	const GEN_FLT x277 = x77 * x112;
	const GEN_FLT x278 = x37 * x107;
	const GEN_FLT x279 = 2 * x21 * x17;
	const GEN_FLT x280 = x97 * x273;
	const GEN_FLT x281 = (x279 * lh_qi) + (-1 * x273 * x100) + (-1 * x273 * x103) + (x280 * x101) + (-1 * x280) +
						 (-1 * x273 * x104) + (x280 * x102) + (x280 * x105);
	const GEN_FLT x282 = x94 * x281;
	const GEN_FLT x283 = x273 * x132;
	const GEN_FLT x284 = x75 * x281;
	const GEN_FLT x285 = x83 * x107;
	const GEN_FLT x286 = x281 * x128;
	const GEN_FLT x287 = (1e-20 < ((aa_z * aa_z) + (aa_y * aa_y) + (aa_x * aa_x))) ? 0 : 0;
	const GEN_FLT x288 = x6 * x287;
	const GEN_FLT x289 = x41 * x288;
	const GEN_FLT x290 = x46 * x287;
	const GEN_FLT x291 = x4 * x289;
	const GEN_FLT x292 = (-1 * x289) + (-1 * x47 * x287) + (x9 * x291) + (x45 * x289) + (-1 * x9 * x290) +
						 (-1 * x2 * x290) + (x2 * x291);
	const GEN_FLT x293 = x56 * x292;
	const GEN_FLT x294 = x38 * x288;
	const GEN_FLT x295 = x37 * x294;
	const GEN_FLT x296 = x76 * x292;
	const GEN_FLT x297 = x81 * x288;
	const GEN_FLT x298 = x71 * x287;
	const GEN_FLT x299 = x88 * x287;
	const GEN_FLT x300 = x83 * x294;
	const GEN_FLT x301 = (-1 * x60 * x292) + (x53 * x293) + (x235 * x292) + (x238 * x296) + (x295 * aa_y) +
						 (x297 * aa_z) + (-1 * x66 * x287) + (-1 * x92 * x288) + (-1 * x73 * x298) + (-1 * x87 * x299) +
						 (x300 * aa_x);
	const GEN_FLT x302 = (x283 * x134) + (-1 * x281 * x127) + (-1 * x274 * x131) + (-1 * x62 * x274) + x301 +
						 (x278 * x282) + x272 + (x284 * x121) + (x275 * x116) + (x286 * x285) + (-1 * x276 * x119) +
						 (-1 * x277 * x274) + (x273 * x124 * x123);
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
	const GEN_FLT x314 = (x292 * x262) + (-1 * x67 * x313) + (-1 * x83 * x309) + (x88 * x310) + (-1 * x76 * x293) +
						 (x287 * x149) + (-1 * x295 * aa_z) + (x297 * aa_y) + (-1 * x73 * x311) + (-1 * x292 * x144) +
						 (-1 * x312 * aa_x);
	const GEN_FLT x315 = (-1 * x37 * x307) + (x68 * x304) + (-1 * x67 * x306) + (-1 * x308 * x128) + x63 +
						 (x281 * x159) + (-1 * x274 * x303) + (-1 * x273 * x161) + (x273 * x166) + (-1 * x83 * x305) +
						 (-1 * x275 * x124) + x314 + (x276 * x112);
	const GEN_FLT x316 = -1 * x137;
	const GEN_FLT x317 = (x234 * x296) + (x300 * aa_z) + (-1 * x292 * x180) + (-1 * x53 * x313) + (-1 * x87 * x298) +
						 (x73 * x299) + (x61 * x310) + (-1 * x292 * x246) + (-1 * x288 * x178) + (-1 * x312 * aa_y) +
						 (-1 * x297 * aa_x);
	const GEN_FLT x318 = (x36 * x304) + (x83 * x307) + (-1 * x281 * x186) + x317 + (-1 * x273 * x185) +
						 (-1 * x37 * x305) + (-1 * x274 * x123) + (x274 * x182) + (x275 * x134) + (-1 * x281 * x181) +
						 x316 + (-1 * x283 * x116) + (-1 * x274 * x183);
	const GEN_FLT x319 = (x302 * x142) + (x315 * x172) + (x318 * x190);
	const GEN_FLT x320 = (x319 * x199) + (-1 * x200 * x302);
	const GEN_FLT x321 = x319 * x196;
	const GEN_FLT x322 = x321 * x194;
	const GEN_FLT x323 = (x71 * x310) + (-1 * x65 * x299) + (-1 * x271 * x292) + (x67 * x293) + (-1 * x219 * x292) +
						 (-1 * x80 * x309) + (x295 * aa_x) + (-1 * x300 * aa_y) + (-1 * x312 * aa_z) +
						 (-1 * x76 * x313) + (x87 * x311);
	const GEN_FLT x324 = (1e-20 < (((time * time) * (lh_qk * lh_qk)) + ((time * time) * (lh_qj * lh_qj)) +
								   ((time * time) * (lh_qi * lh_qi))))
							 ? ((time * time) * lh_qj *
								pow((((time * time) * (lh_qk * lh_qk)) + ((time * time) * (lh_qj * lh_qj)) +
									 ((time * time) * (lh_qi * lh_qi))),
									-1.0 / 2.0))
							 : 0;
	const GEN_FLT x325 = x324 * x116;
	const GEN_FLT x326 = 0.5 * x324;
	const GEN_FLT x327 = x97 * x324;
	const GEN_FLT x328 = x99 * x324;
	const GEN_FLT x329 = x19 * x327;
	const GEN_FLT x330 = (-1 * x327) + (x22 * x329) + (-1 * x23 * x328) + (x279 * lh_qj) + (-1 * x324 * x104) +
						 (x23 * x329) + (-1 * x22 * x328) + (x327 * x105);
	const GEN_FLT x331 = x326 * x119;
	const GEN_FLT x332 = x324 * x115;
	const GEN_FLT x333 = x332 * x123;
	const GEN_FLT x334 = -1 * x170;
	const GEN_FLT x335 = x330 * x128;
	const GEN_FLT x336 = x332 * lh_qi;
	const GEN_FLT x337 = (-1 * x330 * x127) + (x330 * x122) + x301 + (x94 * x278 * x330) + (x336 * x132) +
						 (-1 * x326 * x131) + (-1 * x324 * x110) + (x325 * x117) + x334 + (-1 * x36 * x331) +
						 (x333 * lh_qj) + (x285 * x335) + (-1 * x277 * x326);
	const GEN_FLT x338 = x332 * lh_qj;
	const GEN_FLT x339 = x330 * x107;
	const GEN_FLT x340 = x330 * x162;
	const GEN_FLT x341 = x330 * x126;
	const GEN_FLT x342 = x30 * x324;
	const GEN_FLT x343 = (x324 * x166) + (-1 * x324 * x161) + (-1 * x341 * x128) + (x330 * x159) + (x326 * x156) +
						 (-1 * x77 * x331) + x314 + (-1 * x338 * x117) + (-1 * x83 * x340) + (-1 * x339 * x158) +
						 (x324 * x165) + (-1 * x88 * x342) + x137;
	const GEN_FLT x344 = x83 * x339;
	const GEN_FLT x345 = (-1 * x324 * x185) + (x324 * x184) + (-1 * x335 * x121) + x317 + (-1 * x61 * x342) + x63 +
						 (-1 * x326 * x183) + (-1 * x94 * x341) + (x75 * x344) + (x326 * x182) + (x336 * x117) +
						 (-1 * x325 * x132) + (-1 * x37 * x340);
	const GEN_FLT x346 = (x337 * x142) + (x343 * x172) + (x345 * x190);
	const GEN_FLT x347 = (x346 * x199) + (-1 * x200 * x337);
	const GEN_FLT x348 = (1e-20 < (((time * time) * (lh_qk * lh_qk)) + ((time * time) * (lh_qj * lh_qj)) +
								   ((time * time) * (lh_qi * lh_qi))))
							 ? ((time * time) * lh_qk *
								pow((((time * time) * (lh_qk * lh_qk)) + ((time * time) * (lh_qj * lh_qj)) +
									 ((time * time) * (lh_qi * lh_qi))),
									-1.0 / 2.0))
							 : 0;
	const GEN_FLT x349 = x15 * x348;
	const GEN_FLT x350 = x13 * x349;
	const GEN_FLT x351 = x58 * x350;
	const GEN_FLT x352 = x67 * x350;
	const GEN_FLT x353 = x76 * x350;
	const GEN_FLT x354 = 0.5 * x348;
	const GEN_FLT x355 = x96 * x349;
	const GEN_FLT x356 = (-1 * x348 * x103) + (-1 * x348 * x104) + (x355 * x101) + (-1 * x355) + (x279 * lh_qk) +
						 (x355 * x105) + (-1 * x348 * x100) + (x355 * x102);
	const GEN_FLT x357 = x356 * x107;
	const GEN_FLT x358 = x36 * x354;
	const GEN_FLT x359 = x350 * x124;
	const GEN_FLT x360 = x83 * x357;
	const GEN_FLT x361 = (x53 * x359) + (-1 * x354 * x131) + x301 + (x360 * x128) + x316 + (-0.5 * x351) +
						 (x352 * x134) + (x353 * x116) + (x94 * x37 * x357) + (-1 * x358 * x119) + (x356 * x122) +
						 (-1 * x356 * x127) + (-1 * x277 * x354);
	const GEN_FLT x362 = x29 * x350;
	const GEN_FLT x363 = x356 * x126;
	const GEN_FLT x364 = x53 * x350;
	const GEN_FLT x365 = x356 * x162;
	const GEN_FLT x366 = x351 * x115;
	const GEN_FLT x367 = x348 * x164;
	const GEN_FLT x368 = (x68 * x367) + (x358 * x112) + x314 + (-1 * x354 * x303) + (-1 * x366 * lh_qi) +
						 (x356 * x159) + (-1 * x88 * x362) + (-1 * x83 * x365) + (-1 * x357 * x158) + x334 +
						 (-1 * x363 * x128) + (-1 * x353 * x124) + (x364 * x116);
	const GEN_FLT x369 = (-1 * x356 * x181) + (x36 * x367) + (-1 * x37 * x365) + (x75 * x360) + x139 + (x353 * x134) +
						 (x354 * x182) + (-1 * x352 * x116) + (-1 * x366 * lh_qj) + x317 + (-1 * x354 * x183) +
						 (-1 * x356 * x186) + (-1 * x61 * x362);
	const GEN_FLT x370 = (x361 * x142) + (x368 * x172) + (x369 * x190);
	const GEN_FLT x371 = (x370 * x199) + (-1 * x200 * x361);
	out[0] = (-1 * x191 * x198) + (x201 * x204) + (x206 * x169);
	out[1] = (-1 * x207 * x191) + (x201 * x208) + (x206 * x188);
	out[2] = (-1 * x211 * x191) + (x212 * x201) +
			 (x206 * ((-1 * x35 * x218) + (x34 * x216) + (-1 * x89 * x65) + (-1 * x52 * x219) + x213 +
					  (-1 * x52 * x214) + (-1 * x35 * x217) + (x71 * x152) + (x85 * x39) + (x52 * x215) + x226 +
					  (-1 * x176 * aa_z) + (-1 * x70 * x53)));
	out[3] = 0;
	out[4] = 0;
	out[5] = 0;
	out[6] = (-1 * x250 * x198) + (x204 * x251) + (x206 * x245);
	out[7] = (-1 * x207 * x250) + (x208 * x251) + (x206 * x249);
	out[8] = (-1 * x211 * x250) + (x212 * x251) +
			 (x206 * ((-1 * x76 * x242) + x226 + (x39 * x236) + (x71 * x243) + (-1 * x77 * x244) + (x216 * x227) +
					  (x215 * x230) + (-1 * x248 * aa_z) + (-1 * x217 * x232) + x32 + (-1 * x65 * x231) +
					  (-1 * x234 * x241) + (-1 * x247 * aa_y)));
	out[9] = 0;
	out[10] = 0;
	out[11] = 0;
	out[12] = (-1 * x269 * x198) + (x206 * x265) + (x204 * x270);
	out[13] = (-1 * x207 * x269) + (x208 * x270) + (x206 * x268);
	out[14] = (-1 * x211 * x269) + (x212 * x270) +
			  (x206 * ((-1 * x218 * x253) + (x87 * x263) + x111 + (-1 * x65 * x260) + (x39 * x259) +
					   (-1 * x267 * aa_z) + (-1 * x214 * x256) + (x71 * x266) + x226 + (x215 * x256) +
					   (-1 * x77 * x264) + (-1 * x217 * x253) + (-1 * x271 * x256)));
	out[15] = 0;
	out[16] = 0;
	out[17] = 0;
	out[18] = (x204 * x320) + (-1 * x322 * x172) + (x206 * x315);
	out[19] = (-1 * x322 * x190) + (x208 * x320) + (x206 * x318);
	out[20] = (-1 * x210 * x321) + (x212 * x320) +
			  (x206 * ((x283 * x124) + x170 + (-1 * x76 * x306) + (-1 * x75 * x308) + (x274 * x224) +
					   (-1 * x273 * x222) + (-1 * x276 * x130) + (-1 * x282 * x285) + (x278 * x286) +
					   (-1 * x273 * x225) + x323 + (x77 * x304) + (-1 * x80 * x305)));
	out[21] = 1;
	out[22] = 0;
	out[23] = 0;
	out[24] = (-1 * x346 * x198) + (x204 * x347) + (x206 * x343);
	out[25] = (-1 * x207 * x346) + (x206 * x345) + (x208 * x347);
	out[26] = (-1 * x211 * x346) + (x212 * x347) +
			  (x206 * ((-1 * x94 * x344) + (x223 * x324) + x323 + (-1 * x222 * x324) + x272 + (-1 * x80 * x340) +
					   (-1 * x220 * x324) + (-1 * x75 * x341) + (x278 * x335) + (-1 * x36 * x326 * x130) +
					   (-1 * x333 * lh_qi) + (x224 * x326) + (x338 * x132)));
	out[27] = 0;
	out[28] = 1;
	out[29] = 0;
	out[30] = (-1 * x370 * x198) + (x204 * x371) + (x206 * x368);
	out[31] = (-1 * x207 * x370) + (x208 * x371) + (x206 * x369);
	out[32] = (-1 * x211 * x370) + (x212 * x371) +
			  (x206 * ((-1 * x94 * x360) + (-1 * x366 * lh_qk) + (x67 * x359) + x323 + (x77 * x367) +
					   (-1 * x75 * x363) + (-1 * x364 * x134) + x63 + (x221 * x357) + (-1 * x71 * x362) +
					   (-1 * x80 * x365) + (x224 * x354) + (-1 * x358 * x130)));
	out[33] = 0;
	out[34] = 0;
	out[35] = 1;
}
