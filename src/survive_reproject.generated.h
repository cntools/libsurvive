// NOTE: Auto-generated code; see tools/generate_reprojection_functions
#include <linmath.h>
#include <math.h>
static inline double __safe_asin(double x) { return asin(linmath_enforce_range(x, -1, 1)); }
#define asin __safe_asin
#ifndef WIN32
#include <complex.h>
static inline double __safe_pow(double x, double y) { return x >= 0 ? pow(x, y) : creal(cpow(x, y)); }
#define pow __safe_pow
#endif
#define GEN_FLT FLT
static inline void gen_reproject_jac_obj_p_gen2(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
												const FLT phase_0, const FLT phase_1, const FLT tilt_0,
												const FLT tilt_1, const FLT curve_0, const FLT curve_1,
												const FLT gibPhase_0, const FLT gibPhase_1, const FLT gibMag_0,
												const FLT gibMag_1, const FLT ogeePhase_0, const FLT ogeePhase_1,
												const FLT ogeeMag_0, const FLT ogeeMag_1) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qw = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qw = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qj;
	const GEN_FLT x1 = lh_qk * lh_qw;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = obj_qi * obj_qk;
	const GEN_FLT x4 = obj_qj * obj_qw;
	const GEN_FLT x5 = x3 + x4;
	const GEN_FLT x6 = obj_qi * obj_qi;
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qk * obj_qk;
	const GEN_FLT x9 = x7 + x8;
	const GEN_FLT x10 = sqrt(obj_qw * obj_qw + x6 + x9);
	const GEN_FLT x11 = 2 * x10;
	const GEN_FLT x12 = sensor_z * x11;
	const GEN_FLT x13 = obj_qi * obj_qj;
	const GEN_FLT x14 = obj_qk * obj_qw;
	const GEN_FLT x15 = x13 - x14;
	const GEN_FLT x16 = sensor_y * x11;
	const GEN_FLT x17 = obj_px + sensor_x * (-x11 * x9 + 1) + x12 * x5 + x15 * x16;
	const GEN_FLT x18 = lh_qi * lh_qi;
	const GEN_FLT x19 = lh_qj * lh_qj;
	const GEN_FLT x20 = lh_qk * lh_qk;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = sqrt(lh_qw * lh_qw + x18 + x21);
	const GEN_FLT x23 = 2 * x22;
	const GEN_FLT x24 = x17 * x23;
	const GEN_FLT x25 = lh_qj * lh_qk;
	const GEN_FLT x26 = lh_qi * lh_qw;
	const GEN_FLT x27 = x25 - x26;
	const GEN_FLT x28 = obj_qi * obj_qw;
	const GEN_FLT x29 = obj_qj * obj_qk;
	const GEN_FLT x30 = x28 + x29;
	const GEN_FLT x31 = x3 - x4;
	const GEN_FLT x32 = sensor_x * x11;
	const GEN_FLT x33 = x6 + x7;
	const GEN_FLT x34 = obj_pz + sensor_z * (-x11 * x33 + 1) + x16 * x30 + x31 * x32;
	const GEN_FLT x35 = x23 * x34;
	const GEN_FLT x36 = x18 + x20;
	const GEN_FLT x37 = x23 * x36;
	const GEN_FLT x38 = x13 + x14;
	const GEN_FLT x39 = -x28 + x29;
	const GEN_FLT x40 = x6 + x8;
	const GEN_FLT x41 = obj_py + sensor_y * (-x11 * x40 + 1) + x12 * x39 + x32 * x38;
	const GEN_FLT x42 = -lh_py - x2 * x24 - x27 * x35 - x41 * (-x37 + 1);
	const GEN_FLT x43 = lh_qi * lh_qk;
	const GEN_FLT x44 = lh_qj * lh_qw;
	const GEN_FLT x45 = x43 + x44;
	const GEN_FLT x46 = x0 - x1;
	const GEN_FLT x47 = x23 * x41;
	const GEN_FLT x48 = -x21 * x23 + 1;
	const GEN_FLT x49 = lh_px + x17 * x48 + x35 * x45 + x46 * x47;
	const GEN_FLT x50 = x25 + x26;
	const GEN_FLT x51 = x47 * x50;
	const GEN_FLT x52 = x43 - x44;
	const GEN_FLT x53 = x24 * x52;
	const GEN_FLT x54 = x18 + x19;
	const GEN_FLT x55 = x23 * x54;
	const GEN_FLT x56 = x34 * (-x55 + 1);
	const GEN_FLT x57 = -lh_pz - x51 - x53 - x56;
	const GEN_FLT x58 = x49 * x49 + x57 * x57;
	const GEN_FLT x59 = pow(x58, -1.0 / 2.0);
	const GEN_FLT x60 = tilt_0 - 0.52359877559829882;
	const GEN_FLT x61 = tan(x60);
	const GEN_FLT x62 = x59 * x61;
	const GEN_FLT x63 = x42 * x62;
	const GEN_FLT x64 = x42 * x42;
	const GEN_FLT x65 = x58 + x64;
	const GEN_FLT x66 = pow(x65, -1.0 / 2.0);
	const GEN_FLT x67 = cos(x60);
	const GEN_FLT x68 = 1.0 / x67;
	const GEN_FLT x69 = x66 * x68;
	const GEN_FLT x70 = -asin(x42 * x69);
	const GEN_FLT x71 = 8.0108022e-6 * x70;
	const GEN_FLT x72 = x70 * (x71 - 8.0108022e-6);
	const GEN_FLT x73 = -x72 + 0.0028679863;
	const GEN_FLT x74 = x70 * x73;
	const GEN_FLT x75 = -x74 + 5.3685255000000001e-6;
	const GEN_FLT x76 = x70 * x75;
	const GEN_FLT x77 = -x76 + 0.0076069798000000001;
	const GEN_FLT x78 = x70 * x70;
	const GEN_FLT x79 = atan2(x57, x49);
	const GEN_FLT x80 = ogeePhase_0 + x79 - asin(x63);
	const GEN_FLT x81 = ogeeMag_0 * sin(x80);
	const GEN_FLT x82 = curve_0 + x81;
	const GEN_FLT x83 = x70 * x77;
	const GEN_FLT x84 = 1.60216044e-5 * x70;
	const GEN_FLT x85 = x70 * (x84 - 8.0108022e-6);
	const GEN_FLT x86 = x70 * (x73 - x85);
	const GEN_FLT x87 = x70 * (x75 - x86);
	const GEN_FLT x88 = sin(x60);
	const GEN_FLT x89 = x88 * (-x70 * (x77 - x87) - x83);
	const GEN_FLT x90 = x67 - x82 * x89;
	const GEN_FLT x91 = 1.0 / x90;
	const GEN_FLT x92 = x82 * x91;
	const GEN_FLT x93 = x78 * x92;
	const GEN_FLT x94 = x63 + x77 * x93;
	const GEN_FLT x95 = pow(-x94 * x94 + 1, -1.0 / 2.0);
	const GEN_FLT x96 = x2 * x23;
	const GEN_FLT x97 = x62 * x96;
	const GEN_FLT x98 = 4 * x22;
	const GEN_FLT x99 = (1.0 / 2.0) * x49;
	const GEN_FLT x100 = x23 * x57;
	const GEN_FLT x101 = x100 * x52 - x99 * (-x21 * x98 + 2);
	const GEN_FLT x102 = x42 / pow(x58, 3.0 / 2.0);
	const GEN_FLT x103 = x102 * x61;
	const GEN_FLT x104 = x101 * x103;
	const GEN_FLT x105 = x64 / x65;
	const GEN_FLT x106 = pow(-x105 / (x67 * x67) + 1, -1.0 / 2.0);
	const GEN_FLT x107 = x23 * x42;
	const GEN_FLT x108 = x101 + x107 * x2;
	const GEN_FLT x109 = x42 / pow(x65, 3.0 / 2.0);
	const GEN_FLT x110 = x109 * x68;
	const GEN_FLT x111 = x106 * (-x108 * x110 + x69 * x96);
	const GEN_FLT x112 = 2 * x83 * x92;
	const GEN_FLT x113 = 1.0 / x58;
	const GEN_FLT x114 = x113 * x64;
	const GEN_FLT x115 = pow(-x114 * x61 * x61 + 1, -1.0 / 2.0);
	const GEN_FLT x116 = x113 * (lh_pz + x51 + x53 + x56);
	const GEN_FLT x117 = x113 * x49;
	const GEN_FLT x118 = x117 * x23;
	const GEN_FLT x119 = x116 * x48 - x118 * x52;
	const GEN_FLT x120 = x115 * (-x104 + x97) + x119;
	const GEN_FLT x121 = ogeeMag_0 * cos(x80);
	const GEN_FLT x122 = x77 * x78;
	const GEN_FLT x123 = x121 * x122 * x91;
	const GEN_FLT x124 = x74 - 5.3685255000000001e-6;
	const GEN_FLT x125 = x111 * x124;
	const GEN_FLT x126 = x72 - 0.0028679863;
	const GEN_FLT x127 = -x71 + 8.0108022e-6;
	const GEN_FLT x128 = -x111 * x127;
	const GEN_FLT x129 = -x111 * x126 - x70 * (x111 * x71 + x128);
	const GEN_FLT x130 = x129 * x70;
	const GEN_FLT x131 = x121 * x89;
	const GEN_FLT x132 = x76 - 0.0076069798000000001;
	const GEN_FLT x133 = x132 + x87;
	const GEN_FLT x134 = -x125 - x130;
	const GEN_FLT x135 = 2.40324066e-5 * x70;
	const GEN_FLT x136 = -x84 + 8.0108022e-6;
	const GEN_FLT x137 = x126 + x85;
	const GEN_FLT x138 = x124 + x86;
	const GEN_FLT x139 = x88 * (-curve_0 - x81);
	const GEN_FLT x140 = x122 * x82 / ((x90 * x90));
	const GEN_FLT x141 =
		x119 - x95 * (x104 + x111 * x112 + x120 * x123 +
					  x140 * (x120 * x131 -
							  x139 * (x111 * x132 + x111 * x133 + x134 * x70 +
									  x70 * (-x111 * x138 + x134 -
											 x70 * (-x111 * x137 + x129 - x70 * (x111 * x135 - x111 * x136 + x128))))) +
					  x93 * (x125 + x130) - x97);
	const GEN_FLT x142 = gibMag_0 * cos(gibPhase_0 + x79 - asin(x94));
	const GEN_FLT x143 = x37 - 1;
	const GEN_FLT x144 = x143 * x62;
	const GEN_FLT x145 = x23 * x49;
	const GEN_FLT x146 = x100 * x50 - x145 * x46;
	const GEN_FLT x147 = x103 * x146;
	const GEN_FLT x148 = (1.0 / 2.0) * x42;
	const GEN_FLT x149 = x146 - x148 * (x36 * x98 - 2);
	const GEN_FLT x150 = x106 * (-x110 * x149 - x143 * x69);
	const GEN_FLT x151 = x116 * x23;
	const GEN_FLT x152 = -x118 * x50 + x151 * x46;
	const GEN_FLT x153 = x115 * (-x144 - x147) + x152;
	const GEN_FLT x154 = x124 * x150;
	const GEN_FLT x155 = -x127 * x150;
	const GEN_FLT x156 = -x126 * x150 - x70 * (x150 * x71 + x155);
	const GEN_FLT x157 = x156 * x70;
	const GEN_FLT x158 = -x154 - x157;
	const GEN_FLT x159 =
		x152 - x95 * (x112 * x150 + x123 * x153 +
					  x140 * (x131 * x153 -
							  x139 * (x132 * x150 + x133 * x150 + x158 * x70 +
									  x70 * (-x138 * x150 + x158 -
											 x70 * (-x137 * x150 + x156 - x70 * (x135 * x150 - x136 * x150 + x155))))) +
					  x144 + x147 + x93 * (x154 + x157));
	const GEN_FLT x160 = x23 * x27;
	const GEN_FLT x161 = x160 * x62;
	const GEN_FLT x162 = (1.0 / 2.0) * x57;
	const GEN_FLT x163 = -x145 * x45 - x162 * (x54 * x98 - 2);
	const GEN_FLT x164 = x103 * x163;
	const GEN_FLT x165 = x107 * x27 + x163;
	const GEN_FLT x166 = x106 * (-x110 * x165 + x160 * x69);
	const GEN_FLT x167 = x55 - 1;
	const GEN_FLT x168 = x117 * x167 + x151 * x45;
	const GEN_FLT x169 = x115 * (x161 - x164) + x168;
	const GEN_FLT x170 = x124 * x166;
	const GEN_FLT x171 = -x127 * x166;
	const GEN_FLT x172 = -x126 * x166 - x70 * (x166 * x71 + x171);
	const GEN_FLT x173 = x172 * x70;
	const GEN_FLT x174 = -x170 - x173;
	const GEN_FLT x175 =
		x168 - x95 * (x112 * x166 + x123 * x169 +
					  x140 * (x131 * x169 -
							  x139 * (x132 * x166 + x133 * x166 + x174 * x70 +
									  x70 * (-x138 * x166 + x174 -
											 x70 * (-x137 * x166 + x172 - x70 * (x135 * x166 - x136 * x166 + x171))))) -
					  x161 + x164 + x93 * (x170 + x173));
	const GEN_FLT x176 = obj_qj * x12;
	const GEN_FLT x177 = obj_qk * x16;
	const GEN_FLT x178 = 1.0 / x10;
	const GEN_FLT x179 = obj_qw * x178;
	const GEN_FLT x180 = 2 * x179;
	const GEN_FLT x181 = sensor_z * x180;
	const GEN_FLT x182 = sensor_y * x180;
	const GEN_FLT x183 = -2 * x7;
	const GEN_FLT x184 = -2 * x8;
	const GEN_FLT x185 = sensor_x * (x183 + x184);
	const GEN_FLT x186 = x15 * x182 + x176 - x177 + x179 * x185 + x181 * x5;
	const GEN_FLT x187 = x186 * x2;
	const GEN_FLT x188 = obj_qi * x16;
	const GEN_FLT x189 = obj_qj * x32;
	const GEN_FLT x190 = sensor_x * x31;
	const GEN_FLT x191 = -2 * x6;
	const GEN_FLT x192 = sensor_z * (x183 + x191);
	const GEN_FLT x193 = x179 * x192 + x180 * x190 + x182 * x30 + x188 - x189;
	const GEN_FLT x194 = x193 * x27;
	const GEN_FLT x195 = obj_qi * x12;
	const GEN_FLT x196 = obj_qk * x32;
	const GEN_FLT x197 = sensor_x * x38;
	const GEN_FLT x198 = sensor_y * (x184 + x191);
	const GEN_FLT x199 = x179 * x198 + x180 * x197 + x181 * x39 - x195 + x196;
	const GEN_FLT x200 = x143 * x199;
	const GEN_FLT x201 = -x187 * x23 - x194 * x23 + x200;
	const GEN_FLT x202 = x201 * x62;
	const GEN_FLT x203 = x193 * x45;
	const GEN_FLT x204 = x199 * x98;
	const GEN_FLT x205 = x186 * x48;
	const GEN_FLT x206 = x186 * x52;
	const GEN_FLT x207 = x167 * x193;
	const GEN_FLT x208 = -x162 * (-x204 * x50 - x206 * x98 + 2 * x207) - x99 * (x203 * x98 + x204 * x46 + 2 * x205);
	const GEN_FLT x209 = x103 * x208;
	const GEN_FLT x210 = -x148 * (-x187 * x98 - x194 * x98 + 2 * x200) + x208;
	const GEN_FLT x211 = x106 * (-x110 * x210 - x201 * x69);
	const GEN_FLT x212 = x199 * x23;
	const GEN_FLT x213 = x116 * (x203 * x23 + x205 + x212 * x46) + x117 * (-x206 * x23 + x207 - x212 * x50);
	const GEN_FLT x214 = x115 * (-x202 - x209) + x213;
	const GEN_FLT x215 = x124 * x211;
	const GEN_FLT x216 = -x127 * x211;
	const GEN_FLT x217 = -x126 * x211 - x70 * (x211 * x71 + x216);
	const GEN_FLT x218 = x217 * x70;
	const GEN_FLT x219 = -x215 - x218;
	const GEN_FLT x220 =
		x213 - x95 * (x112 * x211 + x123 * x214 +
					  x140 * (x131 * x214 -
							  x139 * (x132 * x211 + x133 * x211 + x219 * x70 +
									  x70 * (-x138 * x211 + x219 -
											 x70 * (-x137 * x211 + x217 - x70 * (x135 * x211 - x136 * x211 + x216))))) +
					  x202 + x209 + x93 * (x215 + x218));
	const GEN_FLT x221 = obj_qj * x16;
	const GEN_FLT x222 = obj_qk * x12;
	const GEN_FLT x223 = obj_qi * x178;
	const GEN_FLT x224 = 2 * x223;
	const GEN_FLT x225 = sensor_z * x224;
	const GEN_FLT x226 = sensor_y * x224;
	const GEN_FLT x227 = x15 * x226 + x185 * x223 + x221 + x222 + x225 * x5;
	const GEN_FLT x228 = x2 * x227;
	const GEN_FLT x229 = obj_qw * x16;
	const GEN_FLT x230 = 4 * x10;
	const GEN_FLT x231 = -obj_qi * x230;
	const GEN_FLT x232 = sensor_z * (-x224 * x33 + x231) + x190 * x224 + x196 + x226 * x30 + x229;
	const GEN_FLT x233 = x232 * x27;
	const GEN_FLT x234 = obj_qw * x12;
	const GEN_FLT x235 = sensor_y * (-x224 * x40 + x231) + x189 + x197 * x224 + x225 * x39 - x234;
	const GEN_FLT x236 = x143 * x235;
	const GEN_FLT x237 = -x228 * x23 - x23 * x233 + x236;
	const GEN_FLT x238 = x237 * x62;
	const GEN_FLT x239 = x227 * x48;
	const GEN_FLT x240 = x232 * x45;
	const GEN_FLT x241 = x235 * x98;
	const GEN_FLT x242 = x227 * x52;
	const GEN_FLT x243 = x167 * x232;
	const GEN_FLT x244 = -x162 * (-x241 * x50 - x242 * x98 + 2 * x243) - x99 * (2 * x239 + x240 * x98 + x241 * x46);
	const GEN_FLT x245 = x103 * x244;
	const GEN_FLT x246 = -x148 * (-x228 * x98 - x233 * x98 + 2 * x236) + x244;
	const GEN_FLT x247 = x106 * (-x110 * x246 - x237 * x69);
	const GEN_FLT x248 = x23 * x235;
	const GEN_FLT x249 = x116 * (x23 * x240 + x239 + x248 * x46) + x117 * (-x23 * x242 + x243 - x248 * x50);
	const GEN_FLT x250 = x115 * (-x238 - x245) + x249;
	const GEN_FLT x251 = x124 * x247;
	const GEN_FLT x252 = -x127 * x247;
	const GEN_FLT x253 = -x126 * x247 - x70 * (x247 * x71 + x252);
	const GEN_FLT x254 = x253 * x70;
	const GEN_FLT x255 = -x251 - x254;
	const GEN_FLT x256 =
		x249 - x95 * (x112 * x247 + x123 * x250 +
					  x140 * (x131 * x250 -
							  x139 * (x132 * x247 + x133 * x247 + x255 * x70 +
									  x70 * (-x138 * x247 + x255 -
											 x70 * (-x137 * x247 + x253 - x70 * (x135 * x247 - x136 * x247 + x252))))) +
					  x238 + x245 + x93 * (x251 + x254));
	const GEN_FLT x257 = obj_qi * x32;
	const GEN_FLT x258 = obj_qj * x178;
	const GEN_FLT x259 = 2 * x258;
	const GEN_FLT x260 = sensor_z * x259;
	const GEN_FLT x261 = x197 * x259 + x198 * x258 + x222 + x257 + x260 * x39;
	const GEN_FLT x262 = x143 * x261;
	const GEN_FLT x263 = sensor_y * x259;
	const GEN_FLT x264 = -obj_qj * x230;
	const GEN_FLT x265 = sensor_x * (-x259 * x9 + x264) + x15 * x263 + x188 + x234 + x260 * x5;
	const GEN_FLT x266 = x2 * x265;
	const GEN_FLT x267 = obj_qw * x32;
	const GEN_FLT x268 = sensor_z * (-x259 * x33 + x264) + x177 + x190 * x259 + x263 * x30 - x267;
	const GEN_FLT x269 = x268 * x27;
	const GEN_FLT x270 = -x23 * x266 - x23 * x269 + x262;
	const GEN_FLT x271 = x270 * x62;
	const GEN_FLT x272 = x261 * x98;
	const GEN_FLT x273 = x268 * x45;
	const GEN_FLT x274 = x265 * x48;
	const GEN_FLT x275 = x265 * x52;
	const GEN_FLT x276 = x167 * x268;
	const GEN_FLT x277 = -x162 * (-x272 * x50 - x275 * x98 + 2 * x276) - x99 * (x272 * x46 + x273 * x98 + 2 * x274);
	const GEN_FLT x278 = x103 * x277;
	const GEN_FLT x279 = -x148 * (2 * x262 - x266 * x98 - x269 * x98) + x277;
	const GEN_FLT x280 = x106 * (-x110 * x279 - x270 * x69);
	const GEN_FLT x281 = x23 * x261;
	const GEN_FLT x282 = x116 * (x23 * x273 + x274 + x281 * x46) + x117 * (-x23 * x275 + x276 - x281 * x50);
	const GEN_FLT x283 = x115 * (-x271 - x278) + x282;
	const GEN_FLT x284 = x124 * x280;
	const GEN_FLT x285 = -x127 * x280;
	const GEN_FLT x286 = -x126 * x280 - x70 * (x280 * x71 + x285);
	const GEN_FLT x287 = x286 * x70;
	const GEN_FLT x288 = -x284 - x287;
	const GEN_FLT x289 =
		x282 - x95 * (x112 * x280 + x123 * x283 +
					  x140 * (x131 * x283 -
							  x139 * (x132 * x280 + x133 * x280 + x288 * x70 +
									  x70 * (-x138 * x280 + x288 -
											 x70 * (-x137 * x280 + x286 - x70 * (x135 * x280 - x136 * x280 + x285))))) +
					  x271 + x278 + x93 * (x284 + x287));
	const GEN_FLT x290 = obj_qk * x178;
	const GEN_FLT x291 = 2 * x290;
	const GEN_FLT x292 = sensor_y * x291;
	const GEN_FLT x293 = x190 * x291 + x192 * x290 + x221 + x257 + x292 * x30;
	const GEN_FLT x294 = x27 * x293;
	const GEN_FLT x295 = sensor_z * x291;
	const GEN_FLT x296 = -obj_qk * x230;
	const GEN_FLT x297 = sensor_x * (-x291 * x9 + x296) + x15 * x292 + x195 - x229 + x295 * x5;
	const GEN_FLT x298 = x2 * x297;
	const GEN_FLT x299 = sensor_y * (-x291 * x40 + x296) + x176 + x197 * x291 + x267 + x295 * x39;
	const GEN_FLT x300 = x143 * x299;
	const GEN_FLT x301 = -x23 * x294 - x23 * x298 + x300;
	const GEN_FLT x302 = x301 * x62;
	const GEN_FLT x303 = x293 * x45;
	const GEN_FLT x304 = x299 * x98;
	const GEN_FLT x305 = x297 * x48;
	const GEN_FLT x306 = x167 * x293;
	const GEN_FLT x307 = x297 * x52;
	const GEN_FLT x308 = -x162 * (-x304 * x50 + 2 * x306 - x307 * x98) - x99 * (x303 * x98 + x304 * x46 + 2 * x305);
	const GEN_FLT x309 = x103 * x308;
	const GEN_FLT x310 = -x148 * (-x294 * x98 - x298 * x98 + 2 * x300) + x308;
	const GEN_FLT x311 = x106 * (-x110 * x310 - x301 * x69);
	const GEN_FLT x312 = x23 * x299;
	const GEN_FLT x313 = x116 * (x23 * x303 + x305 + x312 * x46) + x117 * (-x23 * x307 + x306 - x312 * x50);
	const GEN_FLT x314 = x115 * (-x302 - x309) + x313;
	const GEN_FLT x315 = x124 * x311;
	const GEN_FLT x316 = -x127 * x311;
	const GEN_FLT x317 = -x126 * x311 - x70 * (x311 * x71 + x316);
	const GEN_FLT x318 = x317 * x70;
	const GEN_FLT x319 = -x315 - x318;
	const GEN_FLT x320 =
		x313 - x95 * (x112 * x311 + x123 * x314 +
					  x140 * (x131 * x314 -
							  x139 * (x132 * x311 + x133 * x311 + x319 * x70 +
									  x70 * (-x138 * x311 + x319 -
											 x70 * (-x137 * x311 + x317 - x70 * (x135 * x311 - x136 * x311 + x316))))) +
					  x302 + x309 + x93 * (x315 + x318));
	const GEN_FLT x321 = tilt_1 + 0.52359877559829882;
	const GEN_FLT x322 = tan(x321);
	const GEN_FLT x323 = x322 * x59;
	const GEN_FLT x324 = x323 * x42;
	const GEN_FLT x325 = cos(x321);
	const GEN_FLT x326 = 1.0 / x325;
	const GEN_FLT x327 = x326 * x66;
	const GEN_FLT x328 = -asin(x327 * x42);
	const GEN_FLT x329 = 8.0108022e-6 * x328;
	const GEN_FLT x330 = x328 * (x329 - 8.0108022e-6);
	const GEN_FLT x331 = -x330 + 0.0028679863;
	const GEN_FLT x332 = x328 * x331;
	const GEN_FLT x333 = -x332 + 5.3685255000000001e-6;
	const GEN_FLT x334 = x328 * x333;
	const GEN_FLT x335 = -x334 + 0.0076069798000000001;
	const GEN_FLT x336 = x328 * x328;
	const GEN_FLT x337 = ogeePhase_1 + x79 - asin(x324);
	const GEN_FLT x338 = ogeeMag_1 * sin(x337);
	const GEN_FLT x339 = curve_1 + x338;
	const GEN_FLT x340 = x328 * x335;
	const GEN_FLT x341 = 1.60216044e-5 * x328;
	const GEN_FLT x342 = x328 * (x341 - 8.0108022e-6);
	const GEN_FLT x343 = x328 * (x331 - x342);
	const GEN_FLT x344 = x328 * (x333 - x343);
	const GEN_FLT x345 = sin(x321);
	const GEN_FLT x346 = x345 * (-x328 * (x335 - x344) - x340);
	const GEN_FLT x347 = x325 - x339 * x346;
	const GEN_FLT x348 = 1.0 / x347;
	const GEN_FLT x349 = x339 * x348;
	const GEN_FLT x350 = x336 * x349;
	const GEN_FLT x351 = x324 + x335 * x350;
	const GEN_FLT x352 = pow(-x351 * x351 + 1, -1.0 / 2.0);
	const GEN_FLT x353 = x323 * x96;
	const GEN_FLT x354 = x102 * x322;
	const GEN_FLT x355 = x101 * x354;
	const GEN_FLT x356 = pow(-x105 / (x325 * x325) + 1, -1.0 / 2.0);
	const GEN_FLT x357 = x109 * x326;
	const GEN_FLT x358 = x356 * (-x108 * x357 + x327 * x96);
	const GEN_FLT x359 = 2 * x340 * x349;
	const GEN_FLT x360 = pow(-x114 * x322 * x322 + 1, -1.0 / 2.0);
	const GEN_FLT x361 = x119 + x360 * (x353 - x355);
	const GEN_FLT x362 = ogeeMag_1 * cos(x337);
	const GEN_FLT x363 = x335 * x336;
	const GEN_FLT x364 = x348 * x362 * x363;
	const GEN_FLT x365 = x332 - 5.3685255000000001e-6;
	const GEN_FLT x366 = x358 * x365;
	const GEN_FLT x367 = x330 - 0.0028679863;
	const GEN_FLT x368 = -x329 + 8.0108022e-6;
	const GEN_FLT x369 = -x358 * x368;
	const GEN_FLT x370 = -x328 * (x329 * x358 + x369) - x358 * x367;
	const GEN_FLT x371 = x328 * x370;
	const GEN_FLT x372 = x346 * x362;
	const GEN_FLT x373 = x334 - 0.0076069798000000001;
	const GEN_FLT x374 = x344 + x373;
	const GEN_FLT x375 = -x366 - x371;
	const GEN_FLT x376 = 2.40324066e-5 * x328;
	const GEN_FLT x377 = -x341 + 8.0108022e-6;
	const GEN_FLT x378 = x342 + x367;
	const GEN_FLT x379 = x343 + x365;
	const GEN_FLT x380 = x345 * (-curve_1 - x338);
	const GEN_FLT x381 = x339 * x363 / ((x347 * x347));
	const GEN_FLT x382 =
		x119 -
		x352 * (x350 * (x366 + x371) - x353 + x355 + x358 * x359 + x361 * x364 +
				x381 * (x361 * x372 -
						x380 * (x328 * x375 +
								x328 * (-x328 * (-x328 * (x358 * x376 - x358 * x377 + x369) - x358 * x378 + x370) -
										x358 * x379 + x375) +
								x358 * x373 + x358 * x374)));
	const GEN_FLT x383 = gibMag_1 * cos(gibPhase_1 + x79 - asin(x351));
	const GEN_FLT x384 = x143 * x323;
	const GEN_FLT x385 = x146 * x354;
	const GEN_FLT x386 = x356 * (-x143 * x327 - x149 * x357);
	const GEN_FLT x387 = x152 + x360 * (-x384 - x385);
	const GEN_FLT x388 = x365 * x386;
	const GEN_FLT x389 = -x368 * x386;
	const GEN_FLT x390 = -x328 * (x329 * x386 + x389) - x367 * x386;
	const GEN_FLT x391 = x328 * x390;
	const GEN_FLT x392 = -x388 - x391;
	const GEN_FLT x393 =
		x152 -
		x352 * (x350 * (x388 + x391) + x359 * x386 + x364 * x387 +
				x381 * (x372 * x387 -
						x380 * (x328 * x392 +
								x328 * (-x328 * (-x328 * (x376 * x386 - x377 * x386 + x389) - x378 * x386 + x390) -
										x379 * x386 + x392) +
								x373 * x386 + x374 * x386)) +
				x384 + x385);
	const GEN_FLT x394 = x160 * x323;
	const GEN_FLT x395 = x163 * x354;
	const GEN_FLT x396 = x356 * (x160 * x327 - x165 * x357);
	const GEN_FLT x397 = x168 + x360 * (x394 - x395);
	const GEN_FLT x398 = x365 * x396;
	const GEN_FLT x399 = -x368 * x396;
	const GEN_FLT x400 = -x328 * (x329 * x396 + x399) - x367 * x396;
	const GEN_FLT x401 = x328 * x400;
	const GEN_FLT x402 = -x398 - x401;
	const GEN_FLT x403 =
		x168 -
		x352 * (x350 * (x398 + x401) + x359 * x396 + x364 * x397 +
				x381 * (x372 * x397 -
						x380 * (x328 * x402 +
								x328 * (-x328 * (-x328 * (x376 * x396 - x377 * x396 + x399) - x378 * x396 + x400) -
										x379 * x396 + x402) +
								x373 * x396 + x374 * x396)) -
				x394 + x395);
	const GEN_FLT x404 = x201 * x323;
	const GEN_FLT x405 = x208 * x354;
	const GEN_FLT x406 = x356 * (-x201 * x327 - x210 * x357);
	const GEN_FLT x407 = x213 + x360 * (-x404 - x405);
	const GEN_FLT x408 = x365 * x406;
	const GEN_FLT x409 = -x368 * x406;
	const GEN_FLT x410 = -x328 * (x329 * x406 + x409) - x367 * x406;
	const GEN_FLT x411 = x328 * x410;
	const GEN_FLT x412 = -x408 - x411;
	const GEN_FLT x413 =
		x213 -
		x352 * (x350 * (x408 + x411) + x359 * x406 + x364 * x407 +
				x381 * (x372 * x407 -
						x380 * (x328 * x412 +
								x328 * (-x328 * (-x328 * (x376 * x406 - x377 * x406 + x409) - x378 * x406 + x410) -
										x379 * x406 + x412) +
								x373 * x406 + x374 * x406)) +
				x404 + x405);
	const GEN_FLT x414 = x237 * x323;
	const GEN_FLT x415 = x244 * x354;
	const GEN_FLT x416 = x356 * (-x237 * x327 - x246 * x357);
	const GEN_FLT x417 = x249 + x360 * (-x414 - x415);
	const GEN_FLT x418 = x365 * x416;
	const GEN_FLT x419 = -x368 * x416;
	const GEN_FLT x420 = -x328 * (x329 * x416 + x419) - x367 * x416;
	const GEN_FLT x421 = x328 * x420;
	const GEN_FLT x422 = -x418 - x421;
	const GEN_FLT x423 =
		x249 -
		x352 * (x350 * (x418 + x421) + x359 * x416 + x364 * x417 +
				x381 * (x372 * x417 -
						x380 * (x328 * x422 +
								x328 * (-x328 * (-x328 * (x376 * x416 - x377 * x416 + x419) - x378 * x416 + x420) -
										x379 * x416 + x422) +
								x373 * x416 + x374 * x416)) +
				x414 + x415);
	const GEN_FLT x424 = x270 * x323;
	const GEN_FLT x425 = x277 * x354;
	const GEN_FLT x426 = x356 * (-x270 * x327 - x279 * x357);
	const GEN_FLT x427 = x282 + x360 * (-x424 - x425);
	const GEN_FLT x428 = x365 * x426;
	const GEN_FLT x429 = -x368 * x426;
	const GEN_FLT x430 = -x328 * (x329 * x426 + x429) - x367 * x426;
	const GEN_FLT x431 = x328 * x430;
	const GEN_FLT x432 = -x428 - x431;
	const GEN_FLT x433 =
		x282 -
		x352 * (x350 * (x428 + x431) + x359 * x426 + x364 * x427 +
				x381 * (x372 * x427 -
						x380 * (x328 * x432 +
								x328 * (-x328 * (-x328 * (x376 * x426 - x377 * x426 + x429) - x378 * x426 + x430) -
										x379 * x426 + x432) +
								x373 * x426 + x374 * x426)) +
				x424 + x425);
	const GEN_FLT x434 = x301 * x323;
	const GEN_FLT x435 = x308 * x354;
	const GEN_FLT x436 = x356 * (-x301 * x327 - x310 * x357);
	const GEN_FLT x437 = x313 + x360 * (-x434 - x435);
	const GEN_FLT x438 = x365 * x436;
	const GEN_FLT x439 = -x368 * x436;
	const GEN_FLT x440 = -x328 * (x329 * x436 + x439) - x367 * x436;
	const GEN_FLT x441 = x328 * x440;
	const GEN_FLT x442 = -x438 - x441;
	const GEN_FLT x443 =
		x313 -
		x352 * (x350 * (x438 + x441) + x359 * x436 + x364 * x437 +
				x381 * (x372 * x437 -
						x380 * (x328 * x442 +
								x328 * (-x328 * (-x328 * (x376 * x436 - x377 * x436 + x439) - x378 * x436 + x440) -
										x379 * x436 + x442) +
								x373 * x436 + x374 * x436)) +
				x434 + x435);
	*(out++) = x141 * x142 + x141;
	*(out++) = x142 * x159 + x159;
	*(out++) = x142 * x175 + x175;
	*(out++) = x142 * x220 + x220;
	*(out++) = x142 * x256 + x256;
	*(out++) = x142 * x289 + x289;
	*(out++) = x142 * x320 + x320;
	*(out++) = x382 * x383 + x382;
	*(out++) = x383 * x393 + x393;
	*(out++) = x383 * x403 + x403;
	*(out++) = x383 * x413 + x413;
	*(out++) = x383 * x423 + x423;
	*(out++) = x383 * x433 + x433;
	*(out++) = x383 * x443 + x443;
}

static inline void gen_reproject_axis_x_jac_obj_p_gen2(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
													   const FLT phase_0, const FLT tilt_0, const FLT curve_0,
													   const FLT gibPhase_0, const FLT gibMag_0, const FLT ogeePhase_0,
													   const FLT ogeeMag_0) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qw = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qw = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qj;
	const GEN_FLT x1 = lh_qk * lh_qw;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = obj_qi * obj_qk;
	const GEN_FLT x4 = obj_qj * obj_qw;
	const GEN_FLT x5 = x3 + x4;
	const GEN_FLT x6 = obj_qi * obj_qi;
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qk * obj_qk;
	const GEN_FLT x9 = x7 + x8;
	const GEN_FLT x10 = sqrt(obj_qw * obj_qw + x6 + x9);
	const GEN_FLT x11 = 2 * x10;
	const GEN_FLT x12 = sensor_z * x11;
	const GEN_FLT x13 = obj_qi * obj_qj;
	const GEN_FLT x14 = obj_qk * obj_qw;
	const GEN_FLT x15 = x13 - x14;
	const GEN_FLT x16 = sensor_y * x11;
	const GEN_FLT x17 = obj_px + sensor_x * (-x11 * x9 + 1) + x12 * x5 + x15 * x16;
	const GEN_FLT x18 = lh_qi * lh_qi;
	const GEN_FLT x19 = lh_qj * lh_qj;
	const GEN_FLT x20 = lh_qk * lh_qk;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = sqrt(lh_qw * lh_qw + x18 + x21);
	const GEN_FLT x23 = 2 * x22;
	const GEN_FLT x24 = x17 * x23;
	const GEN_FLT x25 = lh_qj * lh_qk;
	const GEN_FLT x26 = lh_qi * lh_qw;
	const GEN_FLT x27 = x25 - x26;
	const GEN_FLT x28 = obj_qi * obj_qw;
	const GEN_FLT x29 = obj_qj * obj_qk;
	const GEN_FLT x30 = x28 + x29;
	const GEN_FLT x31 = x3 - x4;
	const GEN_FLT x32 = sensor_x * x11;
	const GEN_FLT x33 = x6 + x7;
	const GEN_FLT x34 = obj_pz + sensor_z * (-x11 * x33 + 1) + x16 * x30 + x31 * x32;
	const GEN_FLT x35 = x23 * x34;
	const GEN_FLT x36 = x18 + x20;
	const GEN_FLT x37 = x23 * x36;
	const GEN_FLT x38 = x13 + x14;
	const GEN_FLT x39 = -x28 + x29;
	const GEN_FLT x40 = x6 + x8;
	const GEN_FLT x41 = obj_py + sensor_y * (-x11 * x40 + 1) + x12 * x39 + x32 * x38;
	const GEN_FLT x42 = -lh_py - x2 * x24 - x27 * x35 - x41 * (-x37 + 1);
	const GEN_FLT x43 = lh_qi * lh_qk;
	const GEN_FLT x44 = lh_qj * lh_qw;
	const GEN_FLT x45 = x43 + x44;
	const GEN_FLT x46 = x0 - x1;
	const GEN_FLT x47 = x23 * x41;
	const GEN_FLT x48 = -x21 * x23 + 1;
	const GEN_FLT x49 = lh_px + x17 * x48 + x35 * x45 + x46 * x47;
	const GEN_FLT x50 = x25 + x26;
	const GEN_FLT x51 = x47 * x50;
	const GEN_FLT x52 = x43 - x44;
	const GEN_FLT x53 = x24 * x52;
	const GEN_FLT x54 = x18 + x19;
	const GEN_FLT x55 = x23 * x54;
	const GEN_FLT x56 = x34 * (-x55 + 1);
	const GEN_FLT x57 = -lh_pz - x51 - x53 - x56;
	const GEN_FLT x58 = x49 * x49 + x57 * x57;
	const GEN_FLT x59 = tilt_0 - 0.52359877559829882;
	const GEN_FLT x60 = tan(x59);
	const GEN_FLT x61 = x60 / sqrt(x58);
	const GEN_FLT x62 = x42 * x61;
	const GEN_FLT x63 = x42 * x42;
	const GEN_FLT x64 = x58 + x63;
	const GEN_FLT x65 = cos(x59);
	const GEN_FLT x66 = 1.0 / x65;
	const GEN_FLT x67 = x66 / sqrt(x64);
	const GEN_FLT x68 = -asin(x42 * x67);
	const GEN_FLT x69 = 8.0108022e-6 * x68;
	const GEN_FLT x70 = x68 * (x69 - 8.0108022e-6);
	const GEN_FLT x71 = -x70 + 0.0028679863;
	const GEN_FLT x72 = x68 * x71;
	const GEN_FLT x73 = -x72 + 5.3685255000000001e-6;
	const GEN_FLT x74 = x68 * x73;
	const GEN_FLT x75 = -x74 + 0.0076069798000000001;
	const GEN_FLT x76 = x68 * x68;
	const GEN_FLT x77 = atan2(x57, x49);
	const GEN_FLT x78 = ogeePhase_0 + x77 - asin(x62);
	const GEN_FLT x79 = ogeeMag_0 * sin(x78);
	const GEN_FLT x80 = curve_0 + x79;
	const GEN_FLT x81 = x68 * x75;
	const GEN_FLT x82 = 1.60216044e-5 * x68;
	const GEN_FLT x83 = x68 * (x82 - 8.0108022e-6);
	const GEN_FLT x84 = x68 * (x71 - x83);
	const GEN_FLT x85 = x68 * (x73 - x84);
	const GEN_FLT x86 = sin(x59);
	const GEN_FLT x87 = x86 * (-x68 * (x75 - x85) - x81);
	const GEN_FLT x88 = x65 - x80 * x87;
	const GEN_FLT x89 = 1.0 / x88;
	const GEN_FLT x90 = x80 * x89;
	const GEN_FLT x91 = x76 * x90;
	const GEN_FLT x92 = x62 + x75 * x91;
	const GEN_FLT x93 = pow(-x92 * x92 + 1, -1.0 / 2.0);
	const GEN_FLT x94 = x2 * x23;
	const GEN_FLT x95 = x61 * x94;
	const GEN_FLT x96 = 4 * x22;
	const GEN_FLT x97 = (1.0 / 2.0) * x49;
	const GEN_FLT x98 = x23 * x57;
	const GEN_FLT x99 = x52 * x98 - x97 * (-x21 * x96 + 2);
	const GEN_FLT x100 = x42 * x60 / pow(x58, 3.0 / 2.0);
	const GEN_FLT x101 = x100 * x99;
	const GEN_FLT x102 = pow(-x63 / (x64 * (x65 * x65)) + 1, -1.0 / 2.0);
	const GEN_FLT x103 = x23 * x42;
	const GEN_FLT x104 = x42 * x66 / pow(x64, 3.0 / 2.0);
	const GEN_FLT x105 = x102 * (-x104 * (x103 * x2 + x99) + x67 * x94);
	const GEN_FLT x106 = 2 * x81 * x90;
	const GEN_FLT x107 = 1.0 / x58;
	const GEN_FLT x108 = pow(-x107 * x63 * x60 * x60 + 1, -1.0 / 2.0);
	const GEN_FLT x109 = x107 * (lh_pz + x51 + x53 + x56);
	const GEN_FLT x110 = x107 * x49;
	const GEN_FLT x111 = x110 * x23;
	const GEN_FLT x112 = x109 * x48 - x111 * x52;
	const GEN_FLT x113 = x108 * (-x101 + x95) + x112;
	const GEN_FLT x114 = ogeeMag_0 * cos(x78);
	const GEN_FLT x115 = x75 * x76;
	const GEN_FLT x116 = x114 * x115 * x89;
	const GEN_FLT x117 = x72 - 5.3685255000000001e-6;
	const GEN_FLT x118 = x105 * x117;
	const GEN_FLT x119 = x70 - 0.0028679863;
	const GEN_FLT x120 = -x69 + 8.0108022e-6;
	const GEN_FLT x121 = -x105 * x120;
	const GEN_FLT x122 = -x105 * x119 - x68 * (x105 * x69 + x121);
	const GEN_FLT x123 = x122 * x68;
	const GEN_FLT x124 = x114 * x87;
	const GEN_FLT x125 = x74 - 0.0076069798000000001;
	const GEN_FLT x126 = x125 + x85;
	const GEN_FLT x127 = -x118 - x123;
	const GEN_FLT x128 = 2.40324066e-5 * x68;
	const GEN_FLT x129 = -x82 + 8.0108022e-6;
	const GEN_FLT x130 = x119 + x83;
	const GEN_FLT x131 = x117 + x84;
	const GEN_FLT x132 = x86 * (-curve_0 - x79);
	const GEN_FLT x133 = x115 * x80 / ((x88 * x88));
	const GEN_FLT x134 =
		x112 - x93 * (x101 + x105 * x106 + x113 * x116 +
					  x133 * (x113 * x124 -
							  x132 * (x105 * x125 + x105 * x126 + x127 * x68 +
									  x68 * (-x105 * x131 + x127 -
											 x68 * (-x105 * x130 + x122 - x68 * (x105 * x128 - x105 * x129 + x121))))) +
					  x91 * (x118 + x123) - x95);
	const GEN_FLT x135 = gibMag_0 * cos(gibPhase_0 + x77 - asin(x92));
	const GEN_FLT x136 = x37 - 1;
	const GEN_FLT x137 = x136 * x61;
	const GEN_FLT x138 = x23 * x49;
	const GEN_FLT x139 = -x138 * x46 + x50 * x98;
	const GEN_FLT x140 = x100 * x139;
	const GEN_FLT x141 = (1.0 / 2.0) * x42;
	const GEN_FLT x142 = x102 * (-x104 * (x139 - x141 * (x36 * x96 - 2)) - x136 * x67);
	const GEN_FLT x143 = x109 * x23;
	const GEN_FLT x144 = -x111 * x50 + x143 * x46;
	const GEN_FLT x145 = x108 * (-x137 - x140) + x144;
	const GEN_FLT x146 = x117 * x142;
	const GEN_FLT x147 = -x120 * x142;
	const GEN_FLT x148 = -x119 * x142 - x68 * (x142 * x69 + x147);
	const GEN_FLT x149 = x148 * x68;
	const GEN_FLT x150 = -x146 - x149;
	const GEN_FLT x151 =
		x144 - x93 * (x106 * x142 + x116 * x145 +
					  x133 * (x124 * x145 -
							  x132 * (x125 * x142 + x126 * x142 + x150 * x68 +
									  x68 * (-x131 * x142 + x150 -
											 x68 * (-x130 * x142 + x148 - x68 * (x128 * x142 - x129 * x142 + x147))))) +
					  x137 + x140 + x91 * (x146 + x149));
	const GEN_FLT x152 = x23 * x27;
	const GEN_FLT x153 = x152 * x61;
	const GEN_FLT x154 = (1.0 / 2.0) * x57;
	const GEN_FLT x155 = -x138 * x45 - x154 * (x54 * x96 - 2);
	const GEN_FLT x156 = x100 * x155;
	const GEN_FLT x157 = x102 * (-x104 * (x103 * x27 + x155) + x152 * x67);
	const GEN_FLT x158 = x55 - 1;
	const GEN_FLT x159 = x110 * x158 + x143 * x45;
	const GEN_FLT x160 = x108 * (x153 - x156) + x159;
	const GEN_FLT x161 = x117 * x157;
	const GEN_FLT x162 = -x120 * x157;
	const GEN_FLT x163 = -x119 * x157 - x68 * (x157 * x69 + x162);
	const GEN_FLT x164 = x163 * x68;
	const GEN_FLT x165 = -x161 - x164;
	const GEN_FLT x166 =
		x159 - x93 * (x106 * x157 + x116 * x160 +
					  x133 * (x124 * x160 -
							  x132 * (x125 * x157 + x126 * x157 + x165 * x68 +
									  x68 * (-x131 * x157 + x165 -
											 x68 * (-x130 * x157 + x163 - x68 * (x128 * x157 - x129 * x157 + x162))))) -
					  x153 + x156 + x91 * (x161 + x164));
	const GEN_FLT x167 = obj_qj * x12;
	const GEN_FLT x168 = obj_qk * x16;
	const GEN_FLT x169 = 1.0 / x10;
	const GEN_FLT x170 = obj_qw * x169;
	const GEN_FLT x171 = 2 * x170;
	const GEN_FLT x172 = sensor_z * x171;
	const GEN_FLT x173 = sensor_y * x171;
	const GEN_FLT x174 = -2 * x7;
	const GEN_FLT x175 = -2 * x8;
	const GEN_FLT x176 = sensor_x * (x174 + x175);
	const GEN_FLT x177 = x15 * x173 + x167 - x168 + x170 * x176 + x172 * x5;
	const GEN_FLT x178 = x177 * x2;
	const GEN_FLT x179 = obj_qi * x16;
	const GEN_FLT x180 = obj_qj * x32;
	const GEN_FLT x181 = sensor_x * x31;
	const GEN_FLT x182 = -2 * x6;
	const GEN_FLT x183 = sensor_z * (x174 + x182);
	const GEN_FLT x184 = x170 * x183 + x171 * x181 + x173 * x30 + x179 - x180;
	const GEN_FLT x185 = x184 * x27;
	const GEN_FLT x186 = obj_qi * x12;
	const GEN_FLT x187 = obj_qk * x32;
	const GEN_FLT x188 = sensor_x * x38;
	const GEN_FLT x189 = sensor_y * (x175 + x182);
	const GEN_FLT x190 = x170 * x189 + x171 * x188 + x172 * x39 - x186 + x187;
	const GEN_FLT x191 = x136 * x190;
	const GEN_FLT x192 = -x178 * x23 - x185 * x23 + x191;
	const GEN_FLT x193 = x192 * x61;
	const GEN_FLT x194 = x184 * x45;
	const GEN_FLT x195 = x190 * x96;
	const GEN_FLT x196 = x177 * x48;
	const GEN_FLT x197 = x177 * x52;
	const GEN_FLT x198 = x158 * x184;
	const GEN_FLT x199 = -x154 * (-x195 * x50 - x197 * x96 + 2 * x198) - x97 * (x194 * x96 + x195 * x46 + 2 * x196);
	const GEN_FLT x200 = x100 * x199;
	const GEN_FLT x201 = x102 * (-x104 * (-x141 * (-x178 * x96 - x185 * x96 + 2 * x191) + x199) - x192 * x67);
	const GEN_FLT x202 = x190 * x23;
	const GEN_FLT x203 = x109 * (x194 * x23 + x196 + x202 * x46) + x110 * (-x197 * x23 + x198 - x202 * x50);
	const GEN_FLT x204 = x108 * (-x193 - x200) + x203;
	const GEN_FLT x205 = x117 * x201;
	const GEN_FLT x206 = -x120 * x201;
	const GEN_FLT x207 = -x119 * x201 - x68 * (x201 * x69 + x206);
	const GEN_FLT x208 = x207 * x68;
	const GEN_FLT x209 = -x205 - x208;
	const GEN_FLT x210 =
		x203 - x93 * (x106 * x201 + x116 * x204 +
					  x133 * (x124 * x204 -
							  x132 * (x125 * x201 + x126 * x201 + x209 * x68 +
									  x68 * (-x131 * x201 + x209 -
											 x68 * (-x130 * x201 + x207 - x68 * (x128 * x201 - x129 * x201 + x206))))) +
					  x193 + x200 + x91 * (x205 + x208));
	const GEN_FLT x211 = obj_qj * x16;
	const GEN_FLT x212 = obj_qk * x12;
	const GEN_FLT x213 = obj_qi * x169;
	const GEN_FLT x214 = 2 * x213;
	const GEN_FLT x215 = sensor_z * x214;
	const GEN_FLT x216 = sensor_y * x214;
	const GEN_FLT x217 = x15 * x216 + x176 * x213 + x211 + x212 + x215 * x5;
	const GEN_FLT x218 = x2 * x217;
	const GEN_FLT x219 = obj_qw * x16;
	const GEN_FLT x220 = 4 * x10;
	const GEN_FLT x221 = -obj_qi * x220;
	const GEN_FLT x222 = sensor_z * (-x214 * x33 + x221) + x181 * x214 + x187 + x216 * x30 + x219;
	const GEN_FLT x223 = x222 * x27;
	const GEN_FLT x224 = obj_qw * x12;
	const GEN_FLT x225 = sensor_y * (-x214 * x40 + x221) + x180 + x188 * x214 + x215 * x39 - x224;
	const GEN_FLT x226 = x136 * x225;
	const GEN_FLT x227 = -x218 * x23 - x223 * x23 + x226;
	const GEN_FLT x228 = x227 * x61;
	const GEN_FLT x229 = x217 * x48;
	const GEN_FLT x230 = x222 * x45;
	const GEN_FLT x231 = x225 * x96;
	const GEN_FLT x232 = x217 * x52;
	const GEN_FLT x233 = x158 * x222;
	const GEN_FLT x234 = -x154 * (-x231 * x50 - x232 * x96 + 2 * x233) - x97 * (2 * x229 + x230 * x96 + x231 * x46);
	const GEN_FLT x235 = x100 * x234;
	const GEN_FLT x236 = x102 * (-x104 * (-x141 * (-x218 * x96 - x223 * x96 + 2 * x226) + x234) - x227 * x67);
	const GEN_FLT x237 = x225 * x23;
	const GEN_FLT x238 = x109 * (x229 + x23 * x230 + x237 * x46) + x110 * (-x23 * x232 + x233 - x237 * x50);
	const GEN_FLT x239 = x108 * (-x228 - x235) + x238;
	const GEN_FLT x240 = x117 * x236;
	const GEN_FLT x241 = -x120 * x236;
	const GEN_FLT x242 = -x119 * x236 - x68 * (x236 * x69 + x241);
	const GEN_FLT x243 = x242 * x68;
	const GEN_FLT x244 = -x240 - x243;
	const GEN_FLT x245 =
		x238 - x93 * (x106 * x236 + x116 * x239 +
					  x133 * (x124 * x239 -
							  x132 * (x125 * x236 + x126 * x236 + x244 * x68 +
									  x68 * (-x131 * x236 + x244 -
											 x68 * (-x130 * x236 + x242 - x68 * (x128 * x236 - x129 * x236 + x241))))) +
					  x228 + x235 + x91 * (x240 + x243));
	const GEN_FLT x246 = obj_qi * x32;
	const GEN_FLT x247 = obj_qj * x169;
	const GEN_FLT x248 = 2 * x247;
	const GEN_FLT x249 = sensor_z * x248;
	const GEN_FLT x250 = x188 * x248 + x189 * x247 + x212 + x246 + x249 * x39;
	const GEN_FLT x251 = x136 * x250;
	const GEN_FLT x252 = sensor_y * x248;
	const GEN_FLT x253 = -obj_qj * x220;
	const GEN_FLT x254 = sensor_x * (-x248 * x9 + x253) + x15 * x252 + x179 + x224 + x249 * x5;
	const GEN_FLT x255 = x2 * x254;
	const GEN_FLT x256 = obj_qw * x32;
	const GEN_FLT x257 = sensor_z * (-x248 * x33 + x253) + x168 + x181 * x248 + x252 * x30 - x256;
	const GEN_FLT x258 = x257 * x27;
	const GEN_FLT x259 = -x23 * x255 - x23 * x258 + x251;
	const GEN_FLT x260 = x259 * x61;
	const GEN_FLT x261 = x250 * x96;
	const GEN_FLT x262 = x257 * x45;
	const GEN_FLT x263 = x254 * x48;
	const GEN_FLT x264 = x254 * x52;
	const GEN_FLT x265 = x158 * x257;
	const GEN_FLT x266 = -x154 * (-x261 * x50 - x264 * x96 + 2 * x265) - x97 * (x261 * x46 + x262 * x96 + 2 * x263);
	const GEN_FLT x267 = x100 * x266;
	const GEN_FLT x268 = x102 * (-x104 * (-x141 * (2 * x251 - x255 * x96 - x258 * x96) + x266) - x259 * x67);
	const GEN_FLT x269 = x23 * x250;
	const GEN_FLT x270 = x109 * (x23 * x262 + x263 + x269 * x46) + x110 * (-x23 * x264 + x265 - x269 * x50);
	const GEN_FLT x271 = x108 * (-x260 - x267) + x270;
	const GEN_FLT x272 = x117 * x268;
	const GEN_FLT x273 = -x120 * x268;
	const GEN_FLT x274 = -x119 * x268 - x68 * (x268 * x69 + x273);
	const GEN_FLT x275 = x274 * x68;
	const GEN_FLT x276 = -x272 - x275;
	const GEN_FLT x277 =
		x270 - x93 * (x106 * x268 + x116 * x271 +
					  x133 * (x124 * x271 -
							  x132 * (x125 * x268 + x126 * x268 + x276 * x68 +
									  x68 * (-x131 * x268 + x276 -
											 x68 * (-x130 * x268 + x274 - x68 * (x128 * x268 - x129 * x268 + x273))))) +
					  x260 + x267 + x91 * (x272 + x275));
	const GEN_FLT x278 = obj_qk * x169;
	const GEN_FLT x279 = 2 * x278;
	const GEN_FLT x280 = sensor_y * x279;
	const GEN_FLT x281 = x181 * x279 + x183 * x278 + x211 + x246 + x280 * x30;
	const GEN_FLT x282 = x27 * x281;
	const GEN_FLT x283 = sensor_z * x279;
	const GEN_FLT x284 = -obj_qk * x220;
	const GEN_FLT x285 = sensor_x * (-x279 * x9 + x284) + x15 * x280 + x186 - x219 + x283 * x5;
	const GEN_FLT x286 = x2 * x285;
	const GEN_FLT x287 = sensor_y * (-x279 * x40 + x284) + x167 + x188 * x279 + x256 + x283 * x39;
	const GEN_FLT x288 = x136 * x287;
	const GEN_FLT x289 = -x23 * x282 - x23 * x286 + x288;
	const GEN_FLT x290 = x289 * x61;
	const GEN_FLT x291 = x281 * x45;
	const GEN_FLT x292 = x287 * x96;
	const GEN_FLT x293 = x285 * x48;
	const GEN_FLT x294 = x158 * x281;
	const GEN_FLT x295 = x285 * x52;
	const GEN_FLT x296 = -x154 * (-x292 * x50 + 2 * x294 - x295 * x96) - x97 * (x291 * x96 + x292 * x46 + 2 * x293);
	const GEN_FLT x297 = x100 * x296;
	const GEN_FLT x298 = x102 * (-x104 * (-x141 * (-x282 * x96 - x286 * x96 + 2 * x288) + x296) - x289 * x67);
	const GEN_FLT x299 = x23 * x287;
	const GEN_FLT x300 = x109 * (x23 * x291 + x293 + x299 * x46) + x110 * (-x23 * x295 + x294 - x299 * x50);
	const GEN_FLT x301 = x108 * (-x290 - x297) + x300;
	const GEN_FLT x302 = x117 * x298;
	const GEN_FLT x303 = -x120 * x298;
	const GEN_FLT x304 = -x119 * x298 - x68 * (x298 * x69 + x303);
	const GEN_FLT x305 = x304 * x68;
	const GEN_FLT x306 = -x302 - x305;
	const GEN_FLT x307 =
		x300 - x93 * (x106 * x298 + x116 * x301 +
					  x133 * (x124 * x301 -
							  x132 * (x125 * x298 + x126 * x298 + x306 * x68 +
									  x68 * (-x131 * x298 + x306 -
											 x68 * (-x130 * x298 + x304 - x68 * (x128 * x298 - x129 * x298 + x303))))) +
					  x290 + x297 + x91 * (x302 + x305));
	*(out++) = x134 * x135 + x134;
	*(out++) = x135 * x151 + x151;
	*(out++) = x135 * x166 + x166;
	*(out++) = x135 * x210 + x210;
	*(out++) = x135 * x245 + x245;
	*(out++) = x135 * x277 + x277;
	*(out++) = x135 * x307 + x307;
}

static inline void gen_reproject_axis_y_jac_obj_p_gen2(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
													   const FLT phase_0, const FLT tilt_0, const FLT curve_0,
													   const FLT gibPhase_0, const FLT gibMag_0, const FLT ogeePhase_0,
													   const FLT ogeeMag_0) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qw = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qw = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qj;
	const GEN_FLT x1 = lh_qk * lh_qw;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = obj_qi * obj_qk;
	const GEN_FLT x4 = obj_qj * obj_qw;
	const GEN_FLT x5 = x3 + x4;
	const GEN_FLT x6 = obj_qi * obj_qi;
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qk * obj_qk;
	const GEN_FLT x9 = x7 + x8;
	const GEN_FLT x10 = sqrt(obj_qw * obj_qw + x6 + x9);
	const GEN_FLT x11 = 2 * x10;
	const GEN_FLT x12 = sensor_z * x11;
	const GEN_FLT x13 = obj_qi * obj_qj;
	const GEN_FLT x14 = obj_qk * obj_qw;
	const GEN_FLT x15 = x13 - x14;
	const GEN_FLT x16 = sensor_y * x11;
	const GEN_FLT x17 = obj_px + sensor_x * (-x11 * x9 + 1) + x12 * x5 + x15 * x16;
	const GEN_FLT x18 = lh_qi * lh_qi;
	const GEN_FLT x19 = lh_qj * lh_qj;
	const GEN_FLT x20 = lh_qk * lh_qk;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = sqrt(lh_qw * lh_qw + x18 + x21);
	const GEN_FLT x23 = 2 * x22;
	const GEN_FLT x24 = x17 * x23;
	const GEN_FLT x25 = lh_qj * lh_qk;
	const GEN_FLT x26 = lh_qi * lh_qw;
	const GEN_FLT x27 = x25 - x26;
	const GEN_FLT x28 = obj_qi * obj_qw;
	const GEN_FLT x29 = obj_qj * obj_qk;
	const GEN_FLT x30 = x28 + x29;
	const GEN_FLT x31 = x3 - x4;
	const GEN_FLT x32 = sensor_x * x11;
	const GEN_FLT x33 = x6 + x7;
	const GEN_FLT x34 = obj_pz + sensor_z * (-x11 * x33 + 1) + x16 * x30 + x31 * x32;
	const GEN_FLT x35 = x23 * x34;
	const GEN_FLT x36 = x18 + x20;
	const GEN_FLT x37 = x23 * x36;
	const GEN_FLT x38 = x13 + x14;
	const GEN_FLT x39 = -x28 + x29;
	const GEN_FLT x40 = x6 + x8;
	const GEN_FLT x41 = obj_py + sensor_y * (-x11 * x40 + 1) + x12 * x39 + x32 * x38;
	const GEN_FLT x42 = -lh_py - x2 * x24 - x27 * x35 - x41 * (-x37 + 1);
	const GEN_FLT x43 = lh_qi * lh_qk;
	const GEN_FLT x44 = lh_qj * lh_qw;
	const GEN_FLT x45 = x43 + x44;
	const GEN_FLT x46 = x0 - x1;
	const GEN_FLT x47 = x23 * x41;
	const GEN_FLT x48 = -x21 * x23 + 1;
	const GEN_FLT x49 = lh_px + x17 * x48 + x35 * x45 + x46 * x47;
	const GEN_FLT x50 = x25 + x26;
	const GEN_FLT x51 = x47 * x50;
	const GEN_FLT x52 = x43 - x44;
	const GEN_FLT x53 = x24 * x52;
	const GEN_FLT x54 = x18 + x19;
	const GEN_FLT x55 = x23 * x54;
	const GEN_FLT x56 = x34 * (-x55 + 1);
	const GEN_FLT x57 = -lh_pz - x51 - x53 - x56;
	const GEN_FLT x58 = x49 * x49 + x57 * x57;
	const GEN_FLT x59 = tilt_0 + 0.52359877559829882;
	const GEN_FLT x60 = tan(x59);
	const GEN_FLT x61 = x60 / sqrt(x58);
	const GEN_FLT x62 = x42 * x61;
	const GEN_FLT x63 = x42 * x42;
	const GEN_FLT x64 = x58 + x63;
	const GEN_FLT x65 = cos(x59);
	const GEN_FLT x66 = 1.0 / x65;
	const GEN_FLT x67 = x66 / sqrt(x64);
	const GEN_FLT x68 = -asin(x42 * x67);
	const GEN_FLT x69 = 8.0108022e-6 * x68;
	const GEN_FLT x70 = x68 * (x69 - 8.0108022e-6);
	const GEN_FLT x71 = -x70 + 0.0028679863;
	const GEN_FLT x72 = x68 * x71;
	const GEN_FLT x73 = -x72 + 5.3685255000000001e-6;
	const GEN_FLT x74 = x68 * x73;
	const GEN_FLT x75 = -x74 + 0.0076069798000000001;
	const GEN_FLT x76 = x68 * x68;
	const GEN_FLT x77 = atan2(x57, x49);
	const GEN_FLT x78 = ogeePhase_0 + x77 - asin(x62);
	const GEN_FLT x79 = ogeeMag_0 * sin(x78);
	const GEN_FLT x80 = curve_0 + x79;
	const GEN_FLT x81 = x68 * x75;
	const GEN_FLT x82 = 1.60216044e-5 * x68;
	const GEN_FLT x83 = x68 * (x82 - 8.0108022e-6);
	const GEN_FLT x84 = x68 * (x71 - x83);
	const GEN_FLT x85 = x68 * (x73 - x84);
	const GEN_FLT x86 = sin(x59);
	const GEN_FLT x87 = x86 * (-x68 * (x75 - x85) - x81);
	const GEN_FLT x88 = x65 - x80 * x87;
	const GEN_FLT x89 = 1.0 / x88;
	const GEN_FLT x90 = x80 * x89;
	const GEN_FLT x91 = x76 * x90;
	const GEN_FLT x92 = x62 + x75 * x91;
	const GEN_FLT x93 = pow(-x92 * x92 + 1, -1.0 / 2.0);
	const GEN_FLT x94 = x2 * x23;
	const GEN_FLT x95 = x61 * x94;
	const GEN_FLT x96 = 4 * x22;
	const GEN_FLT x97 = (1.0 / 2.0) * x49;
	const GEN_FLT x98 = x23 * x57;
	const GEN_FLT x99 = x52 * x98 - x97 * (-x21 * x96 + 2);
	const GEN_FLT x100 = x42 * x60 / pow(x58, 3.0 / 2.0);
	const GEN_FLT x101 = x100 * x99;
	const GEN_FLT x102 = pow(-x63 / (x64 * (x65 * x65)) + 1, -1.0 / 2.0);
	const GEN_FLT x103 = x23 * x42;
	const GEN_FLT x104 = x42 * x66 / pow(x64, 3.0 / 2.0);
	const GEN_FLT x105 = x102 * (-x104 * (x103 * x2 + x99) + x67 * x94);
	const GEN_FLT x106 = 2 * x81 * x90;
	const GEN_FLT x107 = 1.0 / x58;
	const GEN_FLT x108 = pow(-x107 * x63 * x60 * x60 + 1, -1.0 / 2.0);
	const GEN_FLT x109 = x107 * (lh_pz + x51 + x53 + x56);
	const GEN_FLT x110 = x107 * x49;
	const GEN_FLT x111 = x110 * x23;
	const GEN_FLT x112 = x109 * x48 - x111 * x52;
	const GEN_FLT x113 = x108 * (-x101 + x95) + x112;
	const GEN_FLT x114 = ogeeMag_0 * cos(x78);
	const GEN_FLT x115 = x75 * x76;
	const GEN_FLT x116 = x114 * x115 * x89;
	const GEN_FLT x117 = x72 - 5.3685255000000001e-6;
	const GEN_FLT x118 = x105 * x117;
	const GEN_FLT x119 = x70 - 0.0028679863;
	const GEN_FLT x120 = -x69 + 8.0108022e-6;
	const GEN_FLT x121 = -x105 * x120;
	const GEN_FLT x122 = -x105 * x119 - x68 * (x105 * x69 + x121);
	const GEN_FLT x123 = x122 * x68;
	const GEN_FLT x124 = x114 * x87;
	const GEN_FLT x125 = x74 - 0.0076069798000000001;
	const GEN_FLT x126 = x125 + x85;
	const GEN_FLT x127 = -x118 - x123;
	const GEN_FLT x128 = 2.40324066e-5 * x68;
	const GEN_FLT x129 = -x82 + 8.0108022e-6;
	const GEN_FLT x130 = x119 + x83;
	const GEN_FLT x131 = x117 + x84;
	const GEN_FLT x132 = x86 * (-curve_0 - x79);
	const GEN_FLT x133 = x115 * x80 / ((x88 * x88));
	const GEN_FLT x134 =
		x112 - x93 * (x101 + x105 * x106 + x113 * x116 +
					  x133 * (x113 * x124 -
							  x132 * (x105 * x125 + x105 * x126 + x127 * x68 +
									  x68 * (-x105 * x131 + x127 -
											 x68 * (-x105 * x130 + x122 - x68 * (x105 * x128 - x105 * x129 + x121))))) +
					  x91 * (x118 + x123) - x95);
	const GEN_FLT x135 = gibMag_0 * cos(gibPhase_0 + x77 - asin(x92));
	const GEN_FLT x136 = x37 - 1;
	const GEN_FLT x137 = x136 * x61;
	const GEN_FLT x138 = x23 * x49;
	const GEN_FLT x139 = -x138 * x46 + x50 * x98;
	const GEN_FLT x140 = x100 * x139;
	const GEN_FLT x141 = (1.0 / 2.0) * x42;
	const GEN_FLT x142 = x102 * (-x104 * (x139 - x141 * (x36 * x96 - 2)) - x136 * x67);
	const GEN_FLT x143 = x109 * x23;
	const GEN_FLT x144 = -x111 * x50 + x143 * x46;
	const GEN_FLT x145 = x108 * (-x137 - x140) + x144;
	const GEN_FLT x146 = x117 * x142;
	const GEN_FLT x147 = -x120 * x142;
	const GEN_FLT x148 = -x119 * x142 - x68 * (x142 * x69 + x147);
	const GEN_FLT x149 = x148 * x68;
	const GEN_FLT x150 = -x146 - x149;
	const GEN_FLT x151 =
		x144 - x93 * (x106 * x142 + x116 * x145 +
					  x133 * (x124 * x145 -
							  x132 * (x125 * x142 + x126 * x142 + x150 * x68 +
									  x68 * (-x131 * x142 + x150 -
											 x68 * (-x130 * x142 + x148 - x68 * (x128 * x142 - x129 * x142 + x147))))) +
					  x137 + x140 + x91 * (x146 + x149));
	const GEN_FLT x152 = x23 * x27;
	const GEN_FLT x153 = x152 * x61;
	const GEN_FLT x154 = (1.0 / 2.0) * x57;
	const GEN_FLT x155 = -x138 * x45 - x154 * (x54 * x96 - 2);
	const GEN_FLT x156 = x100 * x155;
	const GEN_FLT x157 = x102 * (-x104 * (x103 * x27 + x155) + x152 * x67);
	const GEN_FLT x158 = x55 - 1;
	const GEN_FLT x159 = x110 * x158 + x143 * x45;
	const GEN_FLT x160 = x108 * (x153 - x156) + x159;
	const GEN_FLT x161 = x117 * x157;
	const GEN_FLT x162 = -x120 * x157;
	const GEN_FLT x163 = -x119 * x157 - x68 * (x157 * x69 + x162);
	const GEN_FLT x164 = x163 * x68;
	const GEN_FLT x165 = -x161 - x164;
	const GEN_FLT x166 =
		x159 - x93 * (x106 * x157 + x116 * x160 +
					  x133 * (x124 * x160 -
							  x132 * (x125 * x157 + x126 * x157 + x165 * x68 +
									  x68 * (-x131 * x157 + x165 -
											 x68 * (-x130 * x157 + x163 - x68 * (x128 * x157 - x129 * x157 + x162))))) -
					  x153 + x156 + x91 * (x161 + x164));
	const GEN_FLT x167 = obj_qj * x12;
	const GEN_FLT x168 = obj_qk * x16;
	const GEN_FLT x169 = 1.0 / x10;
	const GEN_FLT x170 = obj_qw * x169;
	const GEN_FLT x171 = 2 * x170;
	const GEN_FLT x172 = sensor_z * x171;
	const GEN_FLT x173 = sensor_y * x171;
	const GEN_FLT x174 = -2 * x7;
	const GEN_FLT x175 = -2 * x8;
	const GEN_FLT x176 = sensor_x * (x174 + x175);
	const GEN_FLT x177 = x15 * x173 + x167 - x168 + x170 * x176 + x172 * x5;
	const GEN_FLT x178 = x177 * x2;
	const GEN_FLT x179 = obj_qi * x16;
	const GEN_FLT x180 = obj_qj * x32;
	const GEN_FLT x181 = sensor_x * x31;
	const GEN_FLT x182 = -2 * x6;
	const GEN_FLT x183 = sensor_z * (x174 + x182);
	const GEN_FLT x184 = x170 * x183 + x171 * x181 + x173 * x30 + x179 - x180;
	const GEN_FLT x185 = x184 * x27;
	const GEN_FLT x186 = obj_qi * x12;
	const GEN_FLT x187 = obj_qk * x32;
	const GEN_FLT x188 = sensor_x * x38;
	const GEN_FLT x189 = sensor_y * (x175 + x182);
	const GEN_FLT x190 = x170 * x189 + x171 * x188 + x172 * x39 - x186 + x187;
	const GEN_FLT x191 = x136 * x190;
	const GEN_FLT x192 = -x178 * x23 - x185 * x23 + x191;
	const GEN_FLT x193 = x192 * x61;
	const GEN_FLT x194 = x184 * x45;
	const GEN_FLT x195 = x190 * x96;
	const GEN_FLT x196 = x177 * x48;
	const GEN_FLT x197 = x177 * x52;
	const GEN_FLT x198 = x158 * x184;
	const GEN_FLT x199 = -x154 * (-x195 * x50 - x197 * x96 + 2 * x198) - x97 * (x194 * x96 + x195 * x46 + 2 * x196);
	const GEN_FLT x200 = x100 * x199;
	const GEN_FLT x201 = x102 * (-x104 * (-x141 * (-x178 * x96 - x185 * x96 + 2 * x191) + x199) - x192 * x67);
	const GEN_FLT x202 = x190 * x23;
	const GEN_FLT x203 = x109 * (x194 * x23 + x196 + x202 * x46) + x110 * (-x197 * x23 + x198 - x202 * x50);
	const GEN_FLT x204 = x108 * (-x193 - x200) + x203;
	const GEN_FLT x205 = x117 * x201;
	const GEN_FLT x206 = -x120 * x201;
	const GEN_FLT x207 = -x119 * x201 - x68 * (x201 * x69 + x206);
	const GEN_FLT x208 = x207 * x68;
	const GEN_FLT x209 = -x205 - x208;
	const GEN_FLT x210 =
		x203 - x93 * (x106 * x201 + x116 * x204 +
					  x133 * (x124 * x204 -
							  x132 * (x125 * x201 + x126 * x201 + x209 * x68 +
									  x68 * (-x131 * x201 + x209 -
											 x68 * (-x130 * x201 + x207 - x68 * (x128 * x201 - x129 * x201 + x206))))) +
					  x193 + x200 + x91 * (x205 + x208));
	const GEN_FLT x211 = obj_qj * x16;
	const GEN_FLT x212 = obj_qk * x12;
	const GEN_FLT x213 = obj_qi * x169;
	const GEN_FLT x214 = 2 * x213;
	const GEN_FLT x215 = sensor_z * x214;
	const GEN_FLT x216 = sensor_y * x214;
	const GEN_FLT x217 = x15 * x216 + x176 * x213 + x211 + x212 + x215 * x5;
	const GEN_FLT x218 = x2 * x217;
	const GEN_FLT x219 = obj_qw * x16;
	const GEN_FLT x220 = 4 * x10;
	const GEN_FLT x221 = -obj_qi * x220;
	const GEN_FLT x222 = sensor_z * (-x214 * x33 + x221) + x181 * x214 + x187 + x216 * x30 + x219;
	const GEN_FLT x223 = x222 * x27;
	const GEN_FLT x224 = obj_qw * x12;
	const GEN_FLT x225 = sensor_y * (-x214 * x40 + x221) + x180 + x188 * x214 + x215 * x39 - x224;
	const GEN_FLT x226 = x136 * x225;
	const GEN_FLT x227 = -x218 * x23 - x223 * x23 + x226;
	const GEN_FLT x228 = x227 * x61;
	const GEN_FLT x229 = x217 * x48;
	const GEN_FLT x230 = x222 * x45;
	const GEN_FLT x231 = x225 * x96;
	const GEN_FLT x232 = x217 * x52;
	const GEN_FLT x233 = x158 * x222;
	const GEN_FLT x234 = -x154 * (-x231 * x50 - x232 * x96 + 2 * x233) - x97 * (2 * x229 + x230 * x96 + x231 * x46);
	const GEN_FLT x235 = x100 * x234;
	const GEN_FLT x236 = x102 * (-x104 * (-x141 * (-x218 * x96 - x223 * x96 + 2 * x226) + x234) - x227 * x67);
	const GEN_FLT x237 = x225 * x23;
	const GEN_FLT x238 = x109 * (x229 + x23 * x230 + x237 * x46) + x110 * (-x23 * x232 + x233 - x237 * x50);
	const GEN_FLT x239 = x108 * (-x228 - x235) + x238;
	const GEN_FLT x240 = x117 * x236;
	const GEN_FLT x241 = -x120 * x236;
	const GEN_FLT x242 = -x119 * x236 - x68 * (x236 * x69 + x241);
	const GEN_FLT x243 = x242 * x68;
	const GEN_FLT x244 = -x240 - x243;
	const GEN_FLT x245 =
		x238 - x93 * (x106 * x236 + x116 * x239 +
					  x133 * (x124 * x239 -
							  x132 * (x125 * x236 + x126 * x236 + x244 * x68 +
									  x68 * (-x131 * x236 + x244 -
											 x68 * (-x130 * x236 + x242 - x68 * (x128 * x236 - x129 * x236 + x241))))) +
					  x228 + x235 + x91 * (x240 + x243));
	const GEN_FLT x246 = obj_qi * x32;
	const GEN_FLT x247 = obj_qj * x169;
	const GEN_FLT x248 = 2 * x247;
	const GEN_FLT x249 = sensor_z * x248;
	const GEN_FLT x250 = x188 * x248 + x189 * x247 + x212 + x246 + x249 * x39;
	const GEN_FLT x251 = x136 * x250;
	const GEN_FLT x252 = sensor_y * x248;
	const GEN_FLT x253 = -obj_qj * x220;
	const GEN_FLT x254 = sensor_x * (-x248 * x9 + x253) + x15 * x252 + x179 + x224 + x249 * x5;
	const GEN_FLT x255 = x2 * x254;
	const GEN_FLT x256 = obj_qw * x32;
	const GEN_FLT x257 = sensor_z * (-x248 * x33 + x253) + x168 + x181 * x248 + x252 * x30 - x256;
	const GEN_FLT x258 = x257 * x27;
	const GEN_FLT x259 = -x23 * x255 - x23 * x258 + x251;
	const GEN_FLT x260 = x259 * x61;
	const GEN_FLT x261 = x250 * x96;
	const GEN_FLT x262 = x257 * x45;
	const GEN_FLT x263 = x254 * x48;
	const GEN_FLT x264 = x254 * x52;
	const GEN_FLT x265 = x158 * x257;
	const GEN_FLT x266 = -x154 * (-x261 * x50 - x264 * x96 + 2 * x265) - x97 * (x261 * x46 + x262 * x96 + 2 * x263);
	const GEN_FLT x267 = x100 * x266;
	const GEN_FLT x268 = x102 * (-x104 * (-x141 * (2 * x251 - x255 * x96 - x258 * x96) + x266) - x259 * x67);
	const GEN_FLT x269 = x23 * x250;
	const GEN_FLT x270 = x109 * (x23 * x262 + x263 + x269 * x46) + x110 * (-x23 * x264 + x265 - x269 * x50);
	const GEN_FLT x271 = x108 * (-x260 - x267) + x270;
	const GEN_FLT x272 = x117 * x268;
	const GEN_FLT x273 = -x120 * x268;
	const GEN_FLT x274 = -x119 * x268 - x68 * (x268 * x69 + x273);
	const GEN_FLT x275 = x274 * x68;
	const GEN_FLT x276 = -x272 - x275;
	const GEN_FLT x277 =
		x270 - x93 * (x106 * x268 + x116 * x271 +
					  x133 * (x124 * x271 -
							  x132 * (x125 * x268 + x126 * x268 + x276 * x68 +
									  x68 * (-x131 * x268 + x276 -
											 x68 * (-x130 * x268 + x274 - x68 * (x128 * x268 - x129 * x268 + x273))))) +
					  x260 + x267 + x91 * (x272 + x275));
	const GEN_FLT x278 = obj_qk * x169;
	const GEN_FLT x279 = 2 * x278;
	const GEN_FLT x280 = sensor_y * x279;
	const GEN_FLT x281 = x181 * x279 + x183 * x278 + x211 + x246 + x280 * x30;
	const GEN_FLT x282 = x27 * x281;
	const GEN_FLT x283 = sensor_z * x279;
	const GEN_FLT x284 = -obj_qk * x220;
	const GEN_FLT x285 = sensor_x * (-x279 * x9 + x284) + x15 * x280 + x186 - x219 + x283 * x5;
	const GEN_FLT x286 = x2 * x285;
	const GEN_FLT x287 = sensor_y * (-x279 * x40 + x284) + x167 + x188 * x279 + x256 + x283 * x39;
	const GEN_FLT x288 = x136 * x287;
	const GEN_FLT x289 = -x23 * x282 - x23 * x286 + x288;
	const GEN_FLT x290 = x289 * x61;
	const GEN_FLT x291 = x281 * x45;
	const GEN_FLT x292 = x287 * x96;
	const GEN_FLT x293 = x285 * x48;
	const GEN_FLT x294 = x158 * x281;
	const GEN_FLT x295 = x285 * x52;
	const GEN_FLT x296 = -x154 * (-x292 * x50 + 2 * x294 - x295 * x96) - x97 * (x291 * x96 + x292 * x46 + 2 * x293);
	const GEN_FLT x297 = x100 * x296;
	const GEN_FLT x298 = x102 * (-x104 * (-x141 * (-x282 * x96 - x286 * x96 + 2 * x288) + x296) - x289 * x67);
	const GEN_FLT x299 = x23 * x287;
	const GEN_FLT x300 = x109 * (x23 * x291 + x293 + x299 * x46) + x110 * (-x23 * x295 + x294 - x299 * x50);
	const GEN_FLT x301 = x108 * (-x290 - x297) + x300;
	const GEN_FLT x302 = x117 * x298;
	const GEN_FLT x303 = -x120 * x298;
	const GEN_FLT x304 = -x119 * x298 - x68 * (x298 * x69 + x303);
	const GEN_FLT x305 = x304 * x68;
	const GEN_FLT x306 = -x302 - x305;
	const GEN_FLT x307 =
		x300 - x93 * (x106 * x298 + x116 * x301 +
					  x133 * (x124 * x301 -
							  x132 * (x125 * x298 + x126 * x298 + x306 * x68 +
									  x68 * (-x131 * x298 + x306 -
											 x68 * (-x130 * x298 + x304 - x68 * (x128 * x298 - x129 * x298 + x303))))) +
					  x290 + x297 + x91 * (x302 + x305));
	*(out++) = x134 * x135 + x134;
	*(out++) = x135 * x151 + x151;
	*(out++) = x135 * x166 + x166;
	*(out++) = x135 * x210 + x210;
	*(out++) = x135 * x245 + x245;
	*(out++) = x135 * x277 + x277;
	*(out++) = x135 * x307 + x307;
}

static inline void gen_reproject_jac_obj_p(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
										   const FLT phase_0, const FLT phase_1, const FLT tilt_0, const FLT tilt_1,
										   const FLT curve_0, const FLT curve_1, const FLT gibPhase_0,
										   const FLT gibPhase_1, const FLT gibMag_0, const FLT gibMag_1) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qw = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qw = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qk;
	const GEN_FLT x1 = lh_qj * lh_qw;
	const GEN_FLT x2 = x0 - x1;
	const GEN_FLT x3 = lh_qi * lh_qi;
	const GEN_FLT x4 = lh_qj * lh_qj;
	const GEN_FLT x5 = lh_qk * lh_qk;
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = sqrt(lh_qw * lh_qw + x3 + x6);
	const GEN_FLT x8 = 2 * x7;
	const GEN_FLT x9 = x0 + x1;
	const GEN_FLT x10 = obj_qi * obj_qw;
	const GEN_FLT x11 = obj_qj * obj_qk;
	const GEN_FLT x12 = x10 + x11;
	const GEN_FLT x13 = obj_qi * obj_qi;
	const GEN_FLT x14 = obj_qj * obj_qj;
	const GEN_FLT x15 = obj_qk * obj_qk;
	const GEN_FLT x16 = x14 + x15;
	const GEN_FLT x17 = sqrt(obj_qw * obj_qw + x13 + x16);
	const GEN_FLT x18 = 2 * x17;
	const GEN_FLT x19 = sensor_y * x18;
	const GEN_FLT x20 = obj_qi * obj_qk;
	const GEN_FLT x21 = obj_qj * obj_qw;
	const GEN_FLT x22 = x20 - x21;
	const GEN_FLT x23 = sensor_x * x18;
	const GEN_FLT x24 = x13 + x14;
	const GEN_FLT x25 = obj_pz + sensor_z * (-x18 * x24 + 1) + x12 * x19 + x22 * x23;
	const GEN_FLT x26 = x25 * x8;
	const GEN_FLT x27 = x26 * x9;
	const GEN_FLT x28 = lh_qi * lh_qj;
	const GEN_FLT x29 = lh_qk * lh_qw;
	const GEN_FLT x30 = x28 - x29;
	const GEN_FLT x31 = obj_qi * obj_qj;
	const GEN_FLT x32 = obj_qk * obj_qw;
	const GEN_FLT x33 = x31 + x32;
	const GEN_FLT x34 = -x10 + x11;
	const GEN_FLT x35 = sensor_z * x18;
	const GEN_FLT x36 = x13 + x15;
	const GEN_FLT x37 = obj_py + sensor_y * (-x18 * x36 + 1) + x23 * x33 + x34 * x35;
	const GEN_FLT x38 = x37 * x8;
	const GEN_FLT x39 = x30 * x38;
	const GEN_FLT x40 = -x6 * x8 + 1;
	const GEN_FLT x41 = x20 + x21;
	const GEN_FLT x42 = x31 - x32;
	const GEN_FLT x43 = obj_px + sensor_x * (-x16 * x18 + 1) + x19 * x42 + x35 * x41;
	const GEN_FLT x44 = x40 * x43;
	const GEN_FLT x45 = lh_px + x27 + x39 + x44;
	const GEN_FLT x46 = x45 * x45;
	const GEN_FLT x47 = lh_qi * lh_qw;
	const GEN_FLT x48 = lh_qj * lh_qk;
	const GEN_FLT x49 = x47 + x48;
	const GEN_FLT x50 = x43 * x8;
	const GEN_FLT x51 = x3 + x4;
	const GEN_FLT x52 = x51 * x8;
	const GEN_FLT x53 = -lh_pz - x2 * x50 - x25 * (-x52 + 1) - x38 * x49;
	const GEN_FLT x54 = x53 * x53;
	const GEN_FLT x55 = x46 + x54;
	const GEN_FLT x56 = 1.0 / x55;
	const GEN_FLT x57 = x56 * (-lh_px - x27 - x39 - x44);
	const GEN_FLT x58 = x57 * x8;
	const GEN_FLT x59 = x2 * x58;
	const GEN_FLT x60 = x53 * x56;
	const GEN_FLT x61 = x40 * x60;
	const GEN_FLT x62 = x28 + x29;
	const GEN_FLT x63 = x50 * x62;
	const GEN_FLT x64 = -x47 + x48;
	const GEN_FLT x65 = x26 * x64;
	const GEN_FLT x66 = x3 + x5;
	const GEN_FLT x67 = -x66 * x8 + 1;
	const GEN_FLT x68 = x37 * x67;
	const GEN_FLT x69 = lh_py + x63 + x65 + x68;
	const GEN_FLT x70 = x69 * x69;
	const GEN_FLT x71 = x54 + x70;
	const GEN_FLT x72 = 1.0 / x71;
	const GEN_FLT x73 = x53 * x72;
	const GEN_FLT x74 = 4 * x7;
	const GEN_FLT x75 = x62 * x74;
	const GEN_FLT x76 = x72 * (-lh_py - x63 - x65 - x68);
	const GEN_FLT x77 = x2 * x74;
	const GEN_FLT x78 = atan2(x69, x53);
	const GEN_FLT x79 = curve_0 * x78;
	const GEN_FLT x80 = pow(-x56 * x70 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x81 = tilt_0 / sqrt(x55);
	const GEN_FLT x82 = x8 * x81;
	const GEN_FLT x83 = (1.0 / 2.0) * x45;
	const GEN_FLT x84 = x53 * x8;
	const GEN_FLT x85 = x2 * x84;
	const GEN_FLT x86 = tilt_0 * x69 / pow(x55, 3.0 / 2.0);
	const GEN_FLT x87 = x80 * (x62 * x82 + x86 * (-x83 * (-x6 * x74 + 2) + x85));
	const GEN_FLT x88 = atan2(x45, x53);
	const GEN_FLT x89 = gibMag_0 * sin(-gibPhase_0 + phase_0 + x88 + asin(x69 * x81) - 1.5707963267948966);
	const GEN_FLT x90 = x49 * x58;
	const GEN_FLT x91 = x60 * x8;
	const GEN_FLT x92 = x30 * x91;
	const GEN_FLT x93 = x49 * x74;
	const GEN_FLT x94 = x67 * x73;
	const GEN_FLT x95 = x45 * x8;
	const GEN_FLT x96 = x49 * x84;
	const GEN_FLT x97 = x80 * (x67 * x81 + x86 * (-x30 * x95 + x96));
	const GEN_FLT x98 = x9 * x91;
	const GEN_FLT x99 = x52 - 1;
	const GEN_FLT x100 = x57 * x99;
	const GEN_FLT x101 = x64 * x74;
	const GEN_FLT x102 = x76 * x99;
	const GEN_FLT x103 = (1.0 / 2.0) * x53;
	const GEN_FLT x104 = -x103 * (x51 * x74 - 2);
	const GEN_FLT x105 = x80 * (x64 * x82 + x86 * (x104 - x9 * x95));
	const GEN_FLT x106 = obj_qi * x19;
	const GEN_FLT x107 = obj_qj * x23;
	const GEN_FLT x108 = 1.0 / x17;
	const GEN_FLT x109 = obj_qw * x108;
	const GEN_FLT x110 = 2 * x109;
	const GEN_FLT x111 = sensor_y * x110;
	const GEN_FLT x112 = sensor_x * x22;
	const GEN_FLT x113 = -2 * x13;
	const GEN_FLT x114 = -2 * x14;
	const GEN_FLT x115 = sensor_z * (x113 + x114);
	const GEN_FLT x116 = x106 - x107 + x109 * x115 + x110 * x112 + x111 * x12;
	const GEN_FLT x117 = x116 * x8;
	const GEN_FLT x118 = obj_qi * x35;
	const GEN_FLT x119 = obj_qk * x23;
	const GEN_FLT x120 = sensor_x * x33;
	const GEN_FLT x121 = sensor_z * x110;
	const GEN_FLT x122 = -2 * x15;
	const GEN_FLT x123 = sensor_y * (x113 + x122);
	const GEN_FLT x124 = x109 * x123 + x110 * x120 - x118 + x119 + x121 * x34;
	const GEN_FLT x125 = x124 * x8;
	const GEN_FLT x126 = obj_qj * x35;
	const GEN_FLT x127 = obj_qk * x19;
	const GEN_FLT x128 = sensor_x * (x114 + x122);
	const GEN_FLT x129 = x109 * x128 + x111 * x42 + x121 * x41 + x126 - x127;
	const GEN_FLT x130 = x129 * x40;
	const GEN_FLT x131 = x117 * x9 + x125 * x30 + x130;
	const GEN_FLT x132 = x131 * x60;
	const GEN_FLT x133 = x129 * x8;
	const GEN_FLT x134 = x116 * x99;
	const GEN_FLT x135 = -x125 * x49 - x133 * x2 + x134;
	const GEN_FLT x136 = x135 * x57;
	const GEN_FLT x137 = x124 * x67;
	const GEN_FLT x138 = x117 * x64 + x133 * x62 + x137;
	const GEN_FLT x139 = x138 * x73;
	const GEN_FLT x140 = x135 * x76;
	const GEN_FLT x141 = x74 * x9;
	const GEN_FLT x142 = x124 * x74;
	const GEN_FLT x143 = -x103 * (-x129 * x77 + 2 * x134 - x142 * x49);
	const GEN_FLT x144 = x80 * (x138 * x81 + x86 * (x143 - x83 * (x116 * x141 + 2 * x130 + x142 * x30)));
	const GEN_FLT x145 = obj_qj * x19;
	const GEN_FLT x146 = obj_qk * x35;
	const GEN_FLT x147 = obj_qi * x108;
	const GEN_FLT x148 = 2 * x147;
	const GEN_FLT x149 = sensor_z * x148;
	const GEN_FLT x150 = sensor_y * x148;
	const GEN_FLT x151 = x128 * x147 + x145 + x146 + x149 * x41 + x150 * x42;
	const GEN_FLT x152 = x151 * x40;
	const GEN_FLT x153 = obj_qw * x19;
	const GEN_FLT x154 = 4 * x17;
	const GEN_FLT x155 = -obj_qi * x154;
	const GEN_FLT x156 = sensor_z * (-x148 * x24 + x155) + x112 * x148 + x119 + x12 * x150 + x153;
	const GEN_FLT x157 = x156 * x8;
	const GEN_FLT x158 = obj_qw * x35;
	const GEN_FLT x159 = sensor_y * (-x148 * x36 + x155) + x107 + x120 * x148 + x149 * x34 - x158;
	const GEN_FLT x160 = x159 * x8;
	const GEN_FLT x161 = x152 + x157 * x9 + x160 * x30;
	const GEN_FLT x162 = x161 * x60;
	const GEN_FLT x163 = x151 * x8;
	const GEN_FLT x164 = x156 * x99;
	const GEN_FLT x165 = -x160 * x49 - x163 * x2 + x164;
	const GEN_FLT x166 = x165 * x57;
	const GEN_FLT x167 = x159 * x67;
	const GEN_FLT x168 = x157 * x64 + x163 * x62 + x167;
	const GEN_FLT x169 = x168 * x73;
	const GEN_FLT x170 = x165 * x76;
	const GEN_FLT x171 = x159 * x74;
	const GEN_FLT x172 = -x103 * (-x151 * x77 + 2 * x164 - x171 * x49);
	const GEN_FLT x173 = x80 * (x168 * x81 + x86 * (x172 - x83 * (x141 * x156 + 2 * x152 + x171 * x30)));
	const GEN_FLT x174 = obj_qi * x23;
	const GEN_FLT x175 = obj_qj * x108;
	const GEN_FLT x176 = 2 * x175;
	const GEN_FLT x177 = sensor_z * x176;
	const GEN_FLT x178 = x120 * x176 + x123 * x175 + x146 + x174 + x177 * x34;
	const GEN_FLT x179 = x178 * x8;
	const GEN_FLT x180 = obj_qw * x23;
	const GEN_FLT x181 = sensor_y * x176;
	const GEN_FLT x182 = -obj_qj * x154;
	const GEN_FLT x183 = sensor_z * (-x176 * x24 + x182) + x112 * x176 + x12 * x181 + x127 - x180;
	const GEN_FLT x184 = x183 * x8;
	const GEN_FLT x185 = sensor_x * (-x16 * x176 + x182) + x106 + x158 + x177 * x41 + x181 * x42;
	const GEN_FLT x186 = x185 * x40;
	const GEN_FLT x187 = x179 * x30 + x184 * x9 + x186;
	const GEN_FLT x188 = x187 * x60;
	const GEN_FLT x189 = x185 * x8;
	const GEN_FLT x190 = x183 * x99;
	const GEN_FLT x191 = -x179 * x49 - x189 * x2 + x190;
	const GEN_FLT x192 = x191 * x57;
	const GEN_FLT x193 = x178 * x67;
	const GEN_FLT x194 = x184 * x64 + x189 * x62 + x193;
	const GEN_FLT x195 = x194 * x73;
	const GEN_FLT x196 = x191 * x76;
	const GEN_FLT x197 = x178 * x74;
	const GEN_FLT x198 = -x103 * (-x185 * x77 + 2 * x190 - x197 * x49);
	const GEN_FLT x199 = x80 * (x194 * x81 + x86 * (x198 - x83 * (x141 * x183 + 2 * x186 + x197 * x30)));
	const GEN_FLT x200 = obj_qk * x108;
	const GEN_FLT x201 = 2 * x200;
	const GEN_FLT x202 = sensor_y * x201;
	const GEN_FLT x203 = x112 * x201 + x115 * x200 + x12 * x202 + x145 + x174;
	const GEN_FLT x204 = x203 * x8;
	const GEN_FLT x205 = sensor_z * x201;
	const GEN_FLT x206 = -obj_qk * x154;
	const GEN_FLT x207 = sensor_y * (-x201 * x36 + x206) + x120 * x201 + x126 + x180 + x205 * x34;
	const GEN_FLT x208 = x207 * x8;
	const GEN_FLT x209 = sensor_x * (-x16 * x201 + x206) + x118 - x153 + x202 * x42 + x205 * x41;
	const GEN_FLT x210 = x209 * x40;
	const GEN_FLT x211 = x204 * x9 + x208 * x30 + x210;
	const GEN_FLT x212 = x211 * x60;
	const GEN_FLT x213 = x203 * x99;
	const GEN_FLT x214 = x209 * x8;
	const GEN_FLT x215 = -x2 * x214 - x208 * x49 + x213;
	const GEN_FLT x216 = x215 * x57;
	const GEN_FLT x217 = x207 * x67;
	const GEN_FLT x218 = x204 * x64 + x214 * x62 + x217;
	const GEN_FLT x219 = x218 * x73;
	const GEN_FLT x220 = x215 * x76;
	const GEN_FLT x221 = x207 * x74;
	const GEN_FLT x222 = -x103 * (-x209 * x77 + 2 * x213 - x221 * x49);
	const GEN_FLT x223 = x80 * (x218 * x81 + x86 * (x222 - x83 * (x141 * x203 + 2 * x210 + x221 * x30)));
	const GEN_FLT x224 = curve_1 * x88;
	const GEN_FLT x225 = x73 * x8;
	const GEN_FLT x226 = x76 * x8;
	const GEN_FLT x227 = pow(-x46 * x72 * tilt_1 * tilt_1 + 1, -1.0 / 2.0);
	const GEN_FLT x228 = tilt_1 / sqrt(x71);
	const GEN_FLT x229 = x69 * x8;
	const GEN_FLT x230 = tilt_1 * x45 / pow(x71, 3.0 / 2.0);
	const GEN_FLT x231 = -x2 * x226 + x225 * x62 - x227 * (x228 * x40 + x230 * (-x229 * x62 + x85));
	const GEN_FLT x232 = gibMag_1 * sin(gibPhase_1 - phase_1 + x78 - asin(x228 * x45) + 1.5707963267948966);
	const GEN_FLT x233 = x228 * x8;
	const GEN_FLT x234 = (1.0 / 2.0) * x69;
	const GEN_FLT x235 = -x226 * x49 - x227 * (x230 * (-x234 * (-x66 * x74 + 2) + x96) + x233 * x30) + x94;
	const GEN_FLT x236 = x102 + x225 * x64 - x227 * (x230 * (x104 - x229 * x64) + x233 * x9);
	const GEN_FLT x237 =
		x139 + x140 - x227 * (x131 * x228 + x230 * (x143 - x234 * (x101 * x116 + x129 * x75 + 2 * x137)));
	const GEN_FLT x238 =
		x169 + x170 - x227 * (x161 * x228 + x230 * (x172 - x234 * (x101 * x156 + x151 * x75 + 2 * x167)));
	const GEN_FLT x239 =
		x195 + x196 - x227 * (x187 * x228 + x230 * (x198 - x234 * (x101 * x183 + x185 * x75 + 2 * x193)));
	const GEN_FLT x240 =
		x219 + x220 - x227 * (x211 * x228 + x230 * (x222 - x234 * (x101 * x203 + x209 * x75 + 2 * x217)));
	*(out++) = x59 - x61 + x79 * (x73 * x75 - x76 * x77) - x87 + x89 * (-x59 + x61 + x87);
	*(out++) = x79 * (-x76 * x93 + 2 * x94) + x89 * (-x90 + x92 + x97) + x90 - x92 - x97;
	*(out++) = -x100 - x105 + x79 * (x101 * x73 + 2 * x102) + x89 * (x100 + x105 + x98) - x98;
	*(out++) = -x132 - x136 - x144 + x79 * (2 * x139 + 2 * x140) + x89 * (x132 + x136 + x144);
	*(out++) = -x162 - x166 - x173 + x79 * (2 * x169 + 2 * x170) + x89 * (x162 + x166 + x173);
	*(out++) = -x188 - x192 - x199 + x79 * (2 * x195 + 2 * x196) + x89 * (x188 + x192 + x199);
	*(out++) = -x212 - x216 - x223 + x79 * (2 * x219 + 2 * x220) + x89 * (x212 + x216 + x223);
	*(out++) = x224 * (-x57 * x77 + 2 * x61) + x231 * x232 + x231;
	*(out++) = x224 * (x30 * x60 * x74 - x57 * x93) + x232 * x235 + x235;
	*(out++) = x224 * (2 * x100 + x141 * x60) + x232 * x236 + x236;
	*(out++) = x224 * (2 * x132 + 2 * x136) + x232 * x237 + x237;
	*(out++) = x224 * (2 * x162 + 2 * x166) + x232 * x238 + x238;
	*(out++) = x224 * (2 * x188 + 2 * x192) + x232 * x239 + x239;
	*(out++) = x224 * (2 * x212 + 2 * x216) + x232 * x240 + x240;
}

static inline void gen_reproject_axis_x_jac_obj_p(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
												  const FLT phase_0, const FLT tilt_0, const FLT curve_0,
												  const FLT gibPhase_0, const FLT gibMag_0) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qw = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qw = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qk;
	const GEN_FLT x1 = lh_qj * lh_qw;
	const GEN_FLT x2 = x0 - x1;
	const GEN_FLT x3 = lh_qi * lh_qi;
	const GEN_FLT x4 = lh_qj * lh_qj;
	const GEN_FLT x5 = lh_qk * lh_qk;
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = sqrt(lh_qw * lh_qw + x3 + x6);
	const GEN_FLT x8 = 2 * x7;
	const GEN_FLT x9 = x0 + x1;
	const GEN_FLT x10 = obj_qi * obj_qw;
	const GEN_FLT x11 = obj_qj * obj_qk;
	const GEN_FLT x12 = x10 + x11;
	const GEN_FLT x13 = obj_qi * obj_qi;
	const GEN_FLT x14 = obj_qj * obj_qj;
	const GEN_FLT x15 = obj_qk * obj_qk;
	const GEN_FLT x16 = x14 + x15;
	const GEN_FLT x17 = sqrt(obj_qw * obj_qw + x13 + x16);
	const GEN_FLT x18 = 2 * x17;
	const GEN_FLT x19 = sensor_y * x18;
	const GEN_FLT x20 = obj_qi * obj_qk;
	const GEN_FLT x21 = obj_qj * obj_qw;
	const GEN_FLT x22 = x20 - x21;
	const GEN_FLT x23 = sensor_x * x18;
	const GEN_FLT x24 = x13 + x14;
	const GEN_FLT x25 = obj_pz + sensor_z * (-x18 * x24 + 1) + x12 * x19 + x22 * x23;
	const GEN_FLT x26 = x25 * x8;
	const GEN_FLT x27 = x26 * x9;
	const GEN_FLT x28 = lh_qi * lh_qj;
	const GEN_FLT x29 = lh_qk * lh_qw;
	const GEN_FLT x30 = x28 - x29;
	const GEN_FLT x31 = obj_qi * obj_qj;
	const GEN_FLT x32 = obj_qk * obj_qw;
	const GEN_FLT x33 = x31 + x32;
	const GEN_FLT x34 = -x10 + x11;
	const GEN_FLT x35 = sensor_z * x18;
	const GEN_FLT x36 = x13 + x15;
	const GEN_FLT x37 = obj_py + sensor_y * (-x18 * x36 + 1) + x23 * x33 + x34 * x35;
	const GEN_FLT x38 = x37 * x8;
	const GEN_FLT x39 = x30 * x38;
	const GEN_FLT x40 = -x6 * x8 + 1;
	const GEN_FLT x41 = x20 + x21;
	const GEN_FLT x42 = x31 - x32;
	const GEN_FLT x43 = obj_px + sensor_x * (-x16 * x18 + 1) + x19 * x42 + x35 * x41;
	const GEN_FLT x44 = x40 * x43;
	const GEN_FLT x45 = lh_px + x27 + x39 + x44;
	const GEN_FLT x46 = lh_qi * lh_qw;
	const GEN_FLT x47 = lh_qj * lh_qk;
	const GEN_FLT x48 = x46 + x47;
	const GEN_FLT x49 = x43 * x8;
	const GEN_FLT x50 = x3 + x4;
	const GEN_FLT x51 = x50 * x8;
	const GEN_FLT x52 = -lh_pz - x2 * x49 - x25 * (-x51 + 1) - x38 * x48;
	const GEN_FLT x53 = x52 * x52;
	const GEN_FLT x54 = x45 * x45 + x53;
	const GEN_FLT x55 = 1.0 / x54;
	const GEN_FLT x56 = x55 * (-lh_px - x27 - x39 - x44);
	const GEN_FLT x57 = x56 * x8;
	const GEN_FLT x58 = x2 * x57;
	const GEN_FLT x59 = x52 * x55;
	const GEN_FLT x60 = x40 * x59;
	const GEN_FLT x61 = x28 + x29;
	const GEN_FLT x62 = x49 * x61;
	const GEN_FLT x63 = -x46 + x47;
	const GEN_FLT x64 = x26 * x63;
	const GEN_FLT x65 = -x8 * (x3 + x5) + 1;
	const GEN_FLT x66 = x37 * x65;
	const GEN_FLT x67 = lh_py + x62 + x64 + x66;
	const GEN_FLT x68 = x67 * x67;
	const GEN_FLT x69 = 1.0 / (x53 + x68);
	const GEN_FLT x70 = 4 * x7;
	const GEN_FLT x71 = x52 * x69 * x70;
	const GEN_FLT x72 = x2 * x70;
	const GEN_FLT x73 = -lh_py - x62 - x64 - x66;
	const GEN_FLT x74 = x69 * x73;
	const GEN_FLT x75 = curve_0 * atan2(x67, x52);
	const GEN_FLT x76 = pow(-x55 * x68 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x77 = tilt_0 / sqrt(x54);
	const GEN_FLT x78 = x77 * x8;
	const GEN_FLT x79 = (1.0 / 2.0) * x45;
	const GEN_FLT x80 = x52 * x8;
	const GEN_FLT x81 = tilt_0 * x67 / pow(x54, 3.0 / 2.0);
	const GEN_FLT x82 = x76 * (x61 * x78 + x81 * (x2 * x80 - x79 * (-x6 * x70 + 2)));
	const GEN_FLT x83 = gibMag_0 * sin(-gibPhase_0 + phase_0 + asin(x67 * x77) + atan2(x45, x52) - 1.5707963267948966);
	const GEN_FLT x84 = x48 * x57;
	const GEN_FLT x85 = x59 * x8;
	const GEN_FLT x86 = x30 * x85;
	const GEN_FLT x87 = 2 * x69;
	const GEN_FLT x88 = x52 * x87;
	const GEN_FLT x89 = x45 * x8;
	const GEN_FLT x90 = x76 * (x65 * x77 + x81 * (-x30 * x89 + x48 * x80));
	const GEN_FLT x91 = x85 * x9;
	const GEN_FLT x92 = x51 - 1;
	const GEN_FLT x93 = x56 * x92;
	const GEN_FLT x94 = x73 * x87;
	const GEN_FLT x95 = (1.0 / 2.0) * x52;
	const GEN_FLT x96 = x76 * (x63 * x78 + x81 * (-x89 * x9 - x95 * (x50 * x70 - 2)));
	const GEN_FLT x97 = obj_qi * x19;
	const GEN_FLT x98 = obj_qj * x23;
	const GEN_FLT x99 = 1.0 / x17;
	const GEN_FLT x100 = obj_qw * x99;
	const GEN_FLT x101 = 2 * x100;
	const GEN_FLT x102 = sensor_y * x101;
	const GEN_FLT x103 = sensor_x * x22;
	const GEN_FLT x104 = -2 * x13;
	const GEN_FLT x105 = -2 * x14;
	const GEN_FLT x106 = sensor_z * (x104 + x105);
	const GEN_FLT x107 = x100 * x106 + x101 * x103 + x102 * x12 + x97 - x98;
	const GEN_FLT x108 = x107 * x8;
	const GEN_FLT x109 = obj_qi * x35;
	const GEN_FLT x110 = obj_qk * x23;
	const GEN_FLT x111 = sensor_x * x33;
	const GEN_FLT x112 = sensor_z * x101;
	const GEN_FLT x113 = -2 * x15;
	const GEN_FLT x114 = sensor_y * (x104 + x113);
	const GEN_FLT x115 = x100 * x114 + x101 * x111 - x109 + x110 + x112 * x34;
	const GEN_FLT x116 = x115 * x8;
	const GEN_FLT x117 = obj_qj * x35;
	const GEN_FLT x118 = obj_qk * x19;
	const GEN_FLT x119 = sensor_x * (x105 + x113);
	const GEN_FLT x120 = x100 * x119 + x102 * x42 + x112 * x41 + x117 - x118;
	const GEN_FLT x121 = x120 * x40;
	const GEN_FLT x122 = x59 * (x108 * x9 + x116 * x30 + x121);
	const GEN_FLT x123 = x120 * x8;
	const GEN_FLT x124 = x107 * x92;
	const GEN_FLT x125 = -x116 * x48 - x123 * x2 + x124;
	const GEN_FLT x126 = x125 * x56;
	const GEN_FLT x127 = x108 * x63 + x115 * x65 + x123 * x61;
	const GEN_FLT x128 = x70 * x9;
	const GEN_FLT x129 = x115 * x70;
	const GEN_FLT x130 = x76 * (x127 * x77 + x81 * (-x79 * (x107 * x128 + 2 * x121 + x129 * x30) -
													x95 * (-x120 * x72 + 2 * x124 - x129 * x48)));
	const GEN_FLT x131 = obj_qj * x19;
	const GEN_FLT x132 = obj_qk * x35;
	const GEN_FLT x133 = obj_qi * x99;
	const GEN_FLT x134 = 2 * x133;
	const GEN_FLT x135 = sensor_z * x134;
	const GEN_FLT x136 = sensor_y * x134;
	const GEN_FLT x137 = x119 * x133 + x131 + x132 + x135 * x41 + x136 * x42;
	const GEN_FLT x138 = x137 * x40;
	const GEN_FLT x139 = obj_qw * x19;
	const GEN_FLT x140 = 4 * x17;
	const GEN_FLT x141 = -obj_qi * x140;
	const GEN_FLT x142 = sensor_z * (-x134 * x24 + x141) + x103 * x134 + x110 + x12 * x136 + x139;
	const GEN_FLT x143 = x142 * x8;
	const GEN_FLT x144 = obj_qw * x35;
	const GEN_FLT x145 = sensor_y * (-x134 * x36 + x141) + x111 * x134 + x135 * x34 - x144 + x98;
	const GEN_FLT x146 = x145 * x8;
	const GEN_FLT x147 = x59 * (x138 + x143 * x9 + x146 * x30);
	const GEN_FLT x148 = x137 * x8;
	const GEN_FLT x149 = x142 * x92;
	const GEN_FLT x150 = -x146 * x48 - x148 * x2 + x149;
	const GEN_FLT x151 = x150 * x56;
	const GEN_FLT x152 = x143 * x63 + x145 * x65 + x148 * x61;
	const GEN_FLT x153 = x145 * x70;
	const GEN_FLT x154 = x76 * (x152 * x77 + x81 * (-x79 * (x128 * x142 + 2 * x138 + x153 * x30) -
													x95 * (-x137 * x72 + 2 * x149 - x153 * x48)));
	const GEN_FLT x155 = obj_qi * x23;
	const GEN_FLT x156 = obj_qj * x99;
	const GEN_FLT x157 = 2 * x156;
	const GEN_FLT x158 = sensor_z * x157;
	const GEN_FLT x159 = x111 * x157 + x114 * x156 + x132 + x155 + x158 * x34;
	const GEN_FLT x160 = x159 * x8;
	const GEN_FLT x161 = obj_qw * x23;
	const GEN_FLT x162 = sensor_y * x157;
	const GEN_FLT x163 = -obj_qj * x140;
	const GEN_FLT x164 = sensor_z * (-x157 * x24 + x163) + x103 * x157 + x118 + x12 * x162 - x161;
	const GEN_FLT x165 = x164 * x8;
	const GEN_FLT x166 = sensor_x * (-x157 * x16 + x163) + x144 + x158 * x41 + x162 * x42 + x97;
	const GEN_FLT x167 = x166 * x40;
	const GEN_FLT x168 = x59 * (x160 * x30 + x165 * x9 + x167);
	const GEN_FLT x169 = x166 * x8;
	const GEN_FLT x170 = x164 * x92;
	const GEN_FLT x171 = -x160 * x48 - x169 * x2 + x170;
	const GEN_FLT x172 = x171 * x56;
	const GEN_FLT x173 = x159 * x65 + x165 * x63 + x169 * x61;
	const GEN_FLT x174 = x159 * x70;
	const GEN_FLT x175 = x76 * (x173 * x77 + x81 * (-x79 * (x128 * x164 + 2 * x167 + x174 * x30) -
													x95 * (-x166 * x72 + 2 * x170 - x174 * x48)));
	const GEN_FLT x176 = obj_qk * x99;
	const GEN_FLT x177 = 2 * x176;
	const GEN_FLT x178 = sensor_y * x177;
	const GEN_FLT x179 = x103 * x177 + x106 * x176 + x12 * x178 + x131 + x155;
	const GEN_FLT x180 = x179 * x8;
	const GEN_FLT x181 = sensor_z * x177;
	const GEN_FLT x182 = -obj_qk * x140;
	const GEN_FLT x183 = sensor_y * (-x177 * x36 + x182) + x111 * x177 + x117 + x161 + x181 * x34;
	const GEN_FLT x184 = x183 * x8;
	const GEN_FLT x185 = sensor_x * (-x16 * x177 + x182) + x109 - x139 + x178 * x42 + x181 * x41;
	const GEN_FLT x186 = x185 * x40;
	const GEN_FLT x187 = x59 * (x180 * x9 + x184 * x30 + x186);
	const GEN_FLT x188 = x179 * x92;
	const GEN_FLT x189 = x185 * x8;
	const GEN_FLT x190 = -x184 * x48 + x188 - x189 * x2;
	const GEN_FLT x191 = x190 * x56;
	const GEN_FLT x192 = x180 * x63 + x183 * x65 + x189 * x61;
	const GEN_FLT x193 = x183 * x70;
	const GEN_FLT x194 = x76 * (x192 * x77 + x81 * (-x79 * (x128 * x179 + 2 * x186 + x193 * x30) -
													x95 * (-x185 * x72 + 2 * x188 - x193 * x48)));
	*(out++) = x58 - x60 + x75 * (x61 * x71 - x72 * x74) - x82 + x83 * (-x58 + x60 + x82);
	*(out++) = x75 * (-x48 * x70 * x74 + x65 * x88) + x83 * (-x84 + x86 + x90) + x84 - x86 - x90;
	*(out++) = x75 * (x63 * x71 + x92 * x94) + x83 * (x91 + x93 + x96) - x91 - x93 - x96;
	*(out++) = -x122 - x126 - x130 + x75 * (x125 * x94 + x127 * x88) + x83 * (x122 + x126 + x130);
	*(out++) = -x147 - x151 - x154 + x75 * (x150 * x94 + x152 * x88) + x83 * (x147 + x151 + x154);
	*(out++) = -x168 - x172 - x175 + x75 * (x171 * x94 + x173 * x88) + x83 * (x168 + x172 + x175);
	*(out++) = -x187 - x191 - x194 + x75 * (x190 * x94 + x192 * x88) + x83 * (x187 + x191 + x194);
}

static inline void gen_reproject_axis_y_jac_obj_p(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
												  const FLT phase_0, const FLT tilt_0, const FLT curve_0,
												  const FLT gibPhase_0, const FLT gibMag_0) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qw = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qw = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qk;
	const GEN_FLT x1 = lh_qj * lh_qw;
	const GEN_FLT x2 = x0 - x1;
	const GEN_FLT x3 = lh_qj * lh_qj;
	const GEN_FLT x4 = lh_qi * lh_qi;
	const GEN_FLT x5 = lh_qk * lh_qk;
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = sqrt(lh_qw * lh_qw + x3 + x6);
	const GEN_FLT x8 = 4 * x7;
	const GEN_FLT x9 = x0 + x1;
	const GEN_FLT x10 = obj_qi * obj_qw;
	const GEN_FLT x11 = obj_qj * obj_qk;
	const GEN_FLT x12 = x10 + x11;
	const GEN_FLT x13 = obj_qj * obj_qj;
	const GEN_FLT x14 = obj_qi * obj_qi;
	const GEN_FLT x15 = obj_qk * obj_qk;
	const GEN_FLT x16 = x14 + x15;
	const GEN_FLT x17 = sqrt(obj_qw * obj_qw + x13 + x16);
	const GEN_FLT x18 = 2 * x17;
	const GEN_FLT x19 = sensor_y * x18;
	const GEN_FLT x20 = obj_qi * obj_qk;
	const GEN_FLT x21 = obj_qj * obj_qw;
	const GEN_FLT x22 = x20 - x21;
	const GEN_FLT x23 = sensor_x * x18;
	const GEN_FLT x24 = x13 + x14;
	const GEN_FLT x25 = obj_pz + sensor_z * (-x18 * x24 + 1) + x12 * x19 + x22 * x23;
	const GEN_FLT x26 = 2 * x7;
	const GEN_FLT x27 = x25 * x26;
	const GEN_FLT x28 = x27 * x9;
	const GEN_FLT x29 = lh_qi * lh_qj;
	const GEN_FLT x30 = lh_qk * lh_qw;
	const GEN_FLT x31 = x29 - x30;
	const GEN_FLT x32 = obj_qi * obj_qj;
	const GEN_FLT x33 = obj_qk * obj_qw;
	const GEN_FLT x34 = x32 + x33;
	const GEN_FLT x35 = -x10 + x11;
	const GEN_FLT x36 = sensor_z * x18;
	const GEN_FLT x37 = obj_py + sensor_y * (-x16 * x18 + 1) + x23 * x34 + x35 * x36;
	const GEN_FLT x38 = x26 * x37;
	const GEN_FLT x39 = x31 * x38;
	const GEN_FLT x40 = -x26 * (x3 + x5) + 1;
	const GEN_FLT x41 = x20 + x21;
	const GEN_FLT x42 = x32 - x33;
	const GEN_FLT x43 = x13 + x15;
	const GEN_FLT x44 = obj_px + sensor_x * (-x18 * x43 + 1) + x19 * x42 + x36 * x41;
	const GEN_FLT x45 = x40 * x44;
	const GEN_FLT x46 = lh_px + x28 + x39 + x45;
	const GEN_FLT x47 = x46 * x46;
	const GEN_FLT x48 = lh_qi * lh_qw;
	const GEN_FLT x49 = lh_qj * lh_qk;
	const GEN_FLT x50 = x48 + x49;
	const GEN_FLT x51 = x26 * x44;
	const GEN_FLT x52 = x3 + x4;
	const GEN_FLT x53 = x26 * x52;
	const GEN_FLT x54 = -lh_pz - x2 * x51 - x25 * (-x53 + 1) - x38 * x50;
	const GEN_FLT x55 = x54 * x54;
	const GEN_FLT x56 = 1.0 / (x47 + x55);
	const GEN_FLT x57 = -lh_px - x28 - x39 - x45;
	const GEN_FLT x58 = x56 * x57;
	const GEN_FLT x59 = 2 * x56;
	const GEN_FLT x60 = x54 * x59;
	const GEN_FLT x61 = curve_0 * atan2(x46, x54);
	const GEN_FLT x62 = x29 + x30;
	const GEN_FLT x63 = x51 * x62;
	const GEN_FLT x64 = -x48 + x49;
	const GEN_FLT x65 = x27 * x64;
	const GEN_FLT x66 = -x26 * x6 + 1;
	const GEN_FLT x67 = x37 * x66;
	const GEN_FLT x68 = lh_py + x63 + x65 + x67;
	const GEN_FLT x69 = x55 + x68 * x68;
	const GEN_FLT x70 = 1.0 / x69;
	const GEN_FLT x71 = x54 * x70;
	const GEN_FLT x72 = x26 * x71;
	const GEN_FLT x73 = x70 * (-lh_py - x63 - x65 - x67);
	const GEN_FLT x74 = x26 * x73;
	const GEN_FLT x75 = pow(-x47 * x70 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x76 = tilt_0 / sqrt(x69);
	const GEN_FLT x77 = x26 * x68;
	const GEN_FLT x78 = x26 * x54;
	const GEN_FLT x79 = tilt_0 * x46 / pow(x69, 3.0 / 2.0);
	const GEN_FLT x80 = -x2 * x74 + x62 * x72 - x75 * (x40 * x76 + x79 * (x2 * x78 - x62 * x77));
	const GEN_FLT x81 = gibMag_0 * sin(gibPhase_0 - phase_0 - asin(x46 * x76) + atan2(x68, x54) + 1.5707963267948966);
	const GEN_FLT x82 = x50 * x8;
	const GEN_FLT x83 = x54 * x56 * x8;
	const GEN_FLT x84 = x26 * x76;
	const GEN_FLT x85 = (1.0 / 2.0) * x68;
	const GEN_FLT x86 = -x50 * x74 + x66 * x71 - x75 * (x31 * x84 + x79 * (x50 * x78 - x85 * (-x6 * x8 + 2)));
	const GEN_FLT x87 = x53 - 1;
	const GEN_FLT x88 = x57 * x59;
	const GEN_FLT x89 = (1.0 / 2.0) * x54;
	const GEN_FLT x90 = x64 * x72 + x73 * x87 - x75 * (x79 * (-x64 * x77 - x89 * (x52 * x8 - 2)) + x84 * x9);
	const GEN_FLT x91 = obj_qi * x19;
	const GEN_FLT x92 = obj_qj * x23;
	const GEN_FLT x93 = 1.0 / x17;
	const GEN_FLT x94 = obj_qw * x93;
	const GEN_FLT x95 = 2 * x94;
	const GEN_FLT x96 = sensor_y * x12;
	const GEN_FLT x97 = sensor_x * x95;
	const GEN_FLT x98 = -2 * x14;
	const GEN_FLT x99 = -2 * x13;
	const GEN_FLT x100 = sensor_z * (x98 + x99);
	const GEN_FLT x101 = x100 * x94 + x22 * x97 + x91 - x92 + x95 * x96;
	const GEN_FLT x102 = x101 * x26;
	const GEN_FLT x103 = obj_qi * x36;
	const GEN_FLT x104 = obj_qk * x23;
	const GEN_FLT x105 = sensor_z * x95;
	const GEN_FLT x106 = -2 * x15;
	const GEN_FLT x107 = sensor_y * (x106 + x98);
	const GEN_FLT x108 = -x103 + x104 + x105 * x35 + x107 * x94 + x34 * x97;
	const GEN_FLT x109 = x108 * x26;
	const GEN_FLT x110 = obj_qj * x36;
	const GEN_FLT x111 = obj_qk * x19;
	const GEN_FLT x112 = sensor_y * x42;
	const GEN_FLT x113 = sensor_x * (x106 + x99);
	const GEN_FLT x114 = x105 * x41 + x110 - x111 + x112 * x95 + x113 * x94;
	const GEN_FLT x115 = x102 * x9 + x109 * x31 + x114 * x40;
	const GEN_FLT x116 = x114 * x26;
	const GEN_FLT x117 = x101 * x87;
	const GEN_FLT x118 = -x109 * x50 - x116 * x2 + x117;
	const GEN_FLT x119 = x108 * x66;
	const GEN_FLT x120 = x114 * x8;
	const GEN_FLT x121 = x64 * x8;
	const GEN_FLT x122 = x118 * x73 + x71 * (x102 * x64 + x116 * x62 + x119) -
						 x75 * (x115 * x76 + x79 * (-x85 * (x101 * x121 + 2 * x119 + x120 * x62) -
													x89 * (-x108 * x82 + 2 * x117 - x120 * x2)));
	const GEN_FLT x123 = obj_qj * x19;
	const GEN_FLT x124 = obj_qk * x36;
	const GEN_FLT x125 = obj_qi * x93;
	const GEN_FLT x126 = 2 * x125;
	const GEN_FLT x127 = sensor_z * x126;
	const GEN_FLT x128 = x112 * x126 + x113 * x125 + x123 + x124 + x127 * x41;
	const GEN_FLT x129 = obj_qw * x19;
	const GEN_FLT x130 = sensor_x * x126;
	const GEN_FLT x131 = 4 * x17;
	const GEN_FLT x132 = -obj_qi * x131;
	const GEN_FLT x133 = sensor_z * (-x126 * x24 + x132) + x104 + x126 * x96 + x129 + x130 * x22;
	const GEN_FLT x134 = x133 * x26;
	const GEN_FLT x135 = obj_qw * x36;
	const GEN_FLT x136 = sensor_y * (-x126 * x16 + x132) + x127 * x35 + x130 * x34 - x135 + x92;
	const GEN_FLT x137 = x136 * x26;
	const GEN_FLT x138 = x128 * x40 + x134 * x9 + x137 * x31;
	const GEN_FLT x139 = x128 * x26;
	const GEN_FLT x140 = x133 * x87;
	const GEN_FLT x141 = -x137 * x50 - x139 * x2 + x140;
	const GEN_FLT x142 = x136 * x66;
	const GEN_FLT x143 = x128 * x8;
	const GEN_FLT x144 = x141 * x73 + x71 * (x134 * x64 + x139 * x62 + x142) -
						 x75 * (x138 * x76 + x79 * (-x85 * (x121 * x133 + 2 * x142 + x143 * x62) -
													x89 * (-x136 * x82 + 2 * x140 - x143 * x2)));
	const GEN_FLT x145 = obj_qi * x23;
	const GEN_FLT x146 = obj_qj * x93;
	const GEN_FLT x147 = 2 * x146;
	const GEN_FLT x148 = sensor_x * x147;
	const GEN_FLT x149 = sensor_z * x147;
	const GEN_FLT x150 = x107 * x146 + x124 + x145 + x148 * x34 + x149 * x35;
	const GEN_FLT x151 = x150 * x26;
	const GEN_FLT x152 = obj_qw * x23;
	const GEN_FLT x153 = -obj_qj * x131;
	const GEN_FLT x154 = sensor_z * (-x147 * x24 + x153) + x111 + x147 * x96 + x148 * x22 - x152;
	const GEN_FLT x155 = x154 * x26;
	const GEN_FLT x156 = sensor_x * (-x147 * x43 + x153) + x112 * x147 + x135 + x149 * x41 + x91;
	const GEN_FLT x157 = x151 * x31 + x155 * x9 + x156 * x40;
	const GEN_FLT x158 = x156 * x26;
	const GEN_FLT x159 = x154 * x87;
	const GEN_FLT x160 = -x151 * x50 - x158 * x2 + x159;
	const GEN_FLT x161 = x150 * x66;
	const GEN_FLT x162 = x156 * x8;
	const GEN_FLT x163 = x160 * x73 + x71 * (x155 * x64 + x158 * x62 + x161) -
						 x75 * (x157 * x76 + x79 * (-x85 * (x121 * x154 + 2 * x161 + x162 * x62) -
													x89 * (-x150 * x82 + 2 * x159 - x162 * x2)));
	const GEN_FLT x164 = obj_qk * x93;
	const GEN_FLT x165 = 2 * x164;
	const GEN_FLT x166 = sensor_x * x165;
	const GEN_FLT x167 = x100 * x164 + x123 + x145 + x165 * x96 + x166 * x22;
	const GEN_FLT x168 = x167 * x26;
	const GEN_FLT x169 = sensor_z * x165;
	const GEN_FLT x170 = -obj_qk * x131;
	const GEN_FLT x171 = sensor_y * (-x16 * x165 + x170) + x110 + x152 + x166 * x34 + x169 * x35;
	const GEN_FLT x172 = x171 * x26;
	const GEN_FLT x173 = sensor_x * (-x165 * x43 + x170) + x103 + x112 * x165 - x129 + x169 * x41;
	const GEN_FLT x174 = x168 * x9 + x172 * x31 + x173 * x40;
	const GEN_FLT x175 = x167 * x87;
	const GEN_FLT x176 = x173 * x26;
	const GEN_FLT x177 = -x172 * x50 + x175 - x176 * x2;
	const GEN_FLT x178 = x171 * x66;
	const GEN_FLT x179 = x173 * x8;
	const GEN_FLT x180 = x177 * x73 + x71 * (x168 * x64 + x176 * x62 + x178) -
						 x75 * (x174 * x76 + x79 * (-x85 * (x121 * x167 + 2 * x178 + x179 * x62) -
													x89 * (-x171 * x82 + 2 * x175 - x179 * x2)));
	*(out++) = x61 * (-x2 * x58 * x8 + x40 * x60) + x80 * x81 + x80;
	*(out++) = x61 * (x31 * x83 - x58 * x82) + x81 * x86 + x86;
	*(out++) = x61 * (x83 * x9 + x87 * x88) + x81 * x90 + x90;
	*(out++) = x122 * x81 + x122 + x61 * (x115 * x60 + x118 * x88);
	*(out++) = x144 * x81 + x144 + x61 * (x138 * x60 + x141 * x88);
	*(out++) = x163 * x81 + x163 + x61 * (x157 * x60 + x160 * x88);
	*(out++) = x180 * x81 + x180 + x61 * (x174 * x60 + x177 * x88);
}

