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
static inline void gen_reproject_axisangle_jac_all_gen2(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
														const FLT phase_0, const FLT phase_1, const FLT tilt_0,
														const FLT tilt_1, const FLT curve_0, const FLT curve_1,
														const FLT gibPhase_0, const FLT gibPhase_1, const FLT gibMag_0,
														const FLT gibMag_1, const FLT ogeePhase_0,
														const FLT ogeePhase_1, const FLT ogeeMag_0,
														const FLT ogeeMag_1) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = -x5;
	const GEN_FLT x7 = x6 + 1;
	const GEN_FLT x8 = 1.0 / x3;
	const GEN_FLT x9 = x4 > 0;
	const GEN_FLT x10 = ((x9) ? (x1 * x8) : (0));
	const GEN_FLT x11 = x10 * x7;
	const GEN_FLT x12 = x11 + x5;
	const GEN_FLT x13 = obj_qi * obj_qi;
	const GEN_FLT x14 = obj_qj * obj_qj;
	const GEN_FLT x15 = obj_qk * obj_qk;
	const GEN_FLT x16 = x13 + x14 + x15;
	const GEN_FLT x17 = sqrt(x16);
	const GEN_FLT x18 = cos(x17);
	const GEN_FLT x19 = 1 - x18;
	const GEN_FLT x20 = 1.0 / x16;
	const GEN_FLT x21 = x17 > 0;
	const GEN_FLT x22 = ((x21) ? (x14 * x20) : (0));
	const GEN_FLT x23 = sin(x17);
	const GEN_FLT x24 = 1.0 / x17;
	const GEN_FLT x25 = obj_qk * x24;
	const GEN_FLT x26 = ((x21) ? (x25) : (0));
	const GEN_FLT x27 = x23 * x26;
	const GEN_FLT x28 = obj_qj * x24;
	const GEN_FLT x29 = ((x21) ? (x28) : (0));
	const GEN_FLT x30 = obj_qi * x24;
	const GEN_FLT x31 = ((x21) ? (x30) : (1));
	const GEN_FLT x32 = x19 * x31;
	const GEN_FLT x33 = x29 * x32;
	const GEN_FLT x34 = x23 * x31;
	const GEN_FLT x35 = x19 * x29;
	const GEN_FLT x36 = x26 * x35;
	const GEN_FLT x37 = obj_py + sensor_x * (x27 + x33) + sensor_y * (x18 + x19 * x22) + sensor_z * (-x34 + x36);
	const GEN_FLT x38 = x12 * x37;
	const GEN_FLT x39 = sin(x4);
	const GEN_FLT x40 = 1.0 / x4;
	const GEN_FLT x41 = lh_qk * x40;
	const GEN_FLT x42 = ((x9) ? (x41) : (0));
	const GEN_FLT x43 = x39 * x42;
	const GEN_FLT x44 = lh_qj * x40;
	const GEN_FLT x45 = ((x9) ? (x44) : (0));
	const GEN_FLT x46 = lh_qi * x40;
	const GEN_FLT x47 = ((x9) ? (x46) : (1));
	const GEN_FLT x48 = x47 * x7;
	const GEN_FLT x49 = x45 * x48;
	const GEN_FLT x50 = x43 + x49;
	const GEN_FLT x51 = ((x21) ? (x13 * x20) : (1));
	const GEN_FLT x52 = x23 * x29;
	const GEN_FLT x53 = x26 * x32;
	const GEN_FLT x54 = obj_px + sensor_x * (x18 + x19 * x51) + sensor_y * (-x27 + x33) + sensor_z * (x52 + x53);
	const GEN_FLT x55 = x50 * x54;
	const GEN_FLT x56 = x39 * x47;
	const GEN_FLT x57 = -x56;
	const GEN_FLT x58 = x45 * x7;
	const GEN_FLT x59 = x42 * x58;
	const GEN_FLT x60 = x57 + x59;
	const GEN_FLT x61 = ((x21) ? (x15 * x20) : (0));
	const GEN_FLT x62 = obj_pz + sensor_x * (-x52 + x53) + sensor_y * (x34 + x36) + sensor_z * (x18 + x19 * x61);
	const GEN_FLT x63 = x60 * x62;
	const GEN_FLT x64 = lh_py + x38 + x55 + x63;
	const GEN_FLT x65 = tilt_0 + 0.52359877559829882;
	const GEN_FLT x66 = tan(x65);
	const GEN_FLT x67 = ((x9) ? (x0 * x8) : (1));
	const GEN_FLT x68 = x67 * x7;
	const GEN_FLT x69 = x5 + x68;
	const GEN_FLT x70 = x54 * x69;
	const GEN_FLT x71 = x39 * x45;
	const GEN_FLT x72 = x42 * x48;
	const GEN_FLT x73 = x71 + x72;
	const GEN_FLT x74 = x62 * x73;
	const GEN_FLT x75 = -x43 + x49;
	const GEN_FLT x76 = x37 * x75;
	const GEN_FLT x77 = lh_px + x70 + x74 + x76;
	const GEN_FLT x78 = ((x9) ? (x2 * x8) : (0));
	const GEN_FLT x79 = x7 * x78;
	const GEN_FLT x80 = x62 * (x5 + x79);
	const GEN_FLT x81 = x37 * (x56 + x59);
	const GEN_FLT x82 = x54 * (-x71 + x72);
	const GEN_FLT x83 = -lh_pz - x80 - x81 - x82;
	const GEN_FLT x84 = x77 * x77 + x83 * x83;
	const GEN_FLT x85 = pow(x84, -1.0 / 2.0);
	const GEN_FLT x86 = x66 * x85;
	const GEN_FLT x87 = x64 * x86;
	const GEN_FLT x88 = cos(x65);
	const GEN_FLT x89 = 1.0 / x88;
	const GEN_FLT x90 = x64 * x64;
	const GEN_FLT x91 = x84 + x90;
	const GEN_FLT x92 = pow(x91, -1.0 / 2.0);
	const GEN_FLT x93 = x89 * x92;
	const GEN_FLT x94 = asin(x64 * x93);
	const GEN_FLT x95 = 8.0108022e-6 * x94;
	const GEN_FLT x96 = -x95 - 8.0108022e-6;
	const GEN_FLT x97 = x94 * x96 + 0.0028679863;
	const GEN_FLT x98 = x94 * x97 + 5.3685255000000001e-6;
	const GEN_FLT x99 = x94 * x98 + 0.0076069798000000001;
	const GEN_FLT x100 = x94 * x94;
	const GEN_FLT x101 = atan2(x83, x77);
	const GEN_FLT x102 = ogeePhase_0 + x101 - asin(x87);
	const GEN_FLT x103 = ogeeMag_0 * sin(x102);
	const GEN_FLT x104 = curve_0 + x103;
	const GEN_FLT x105 = x94 * x99;
	const GEN_FLT x106 = -1.60216044e-5 * x94 - 8.0108022e-6;
	const GEN_FLT x107 = x106 * x94 + x97;
	const GEN_FLT x108 = x107 * x94 + x98;
	const GEN_FLT x109 = x108 * x94 + x99;
	const GEN_FLT x110 = sin(x65);
	const GEN_FLT x111 = x110 * (x105 + x109 * x94);
	const GEN_FLT x112 = -x104 * x111 + x88;
	const GEN_FLT x113 = 1.0 / x112;
	const GEN_FLT x114 = x104 * x113;
	const GEN_FLT x115 = x100 * x114;
	const GEN_FLT x116 = x115 * x99 + x87;
	const GEN_FLT x117 = pow(1 - x116 * x116, -1.0 / 2.0);
	const GEN_FLT x118 = x90 / x91;
	const GEN_FLT x119 = pow(-x118 / (x88 * x88) + 1, -1.0 / 2.0);
	const GEN_FLT x120 = 2 * x43;
	const GEN_FLT x121 = 2 * x49;
	const GEN_FLT x122 = (1.0 / 2.0) * x64;
	const GEN_FLT x123 = 2 * x5;
	const GEN_FLT x124 = (1.0 / 2.0) * x77;
	const GEN_FLT x125 = 2 * x71;
	const GEN_FLT x126 = 2 * x72;
	const GEN_FLT x127 = (1.0 / 2.0) * x83;
	const GEN_FLT x128 = -x124 * (x123 + 2 * x68) - x127 * (x125 - x126);
	const GEN_FLT x129 = -x122 * (x120 + x121) + x128;
	const GEN_FLT x130 = x64 / pow(x91, 3.0 / 2.0);
	const GEN_FLT x131 = x130 * x89;
	const GEN_FLT x132 = x119 * (x129 * x131 + x50 * x93);
	const GEN_FLT x133 = x132 * x96;
	const GEN_FLT x134 = x132 * x97 + x94 * (-x132 * x95 + x133);
	const GEN_FLT x135 = x132 * x98 + x134 * x94;
	const GEN_FLT x136 = 1.0 / x84;
	const GEN_FLT x137 = x136 * x90;
	const GEN_FLT x138 = pow(-x137 * x66 * x66 + 1, -1.0 / 2.0);
	const GEN_FLT x139 = x64 / pow(x84, 3.0 / 2.0);
	const GEN_FLT x140 = x139 * x66;
	const GEN_FLT x141 = x128 * x140 + x50 * x86;
	const GEN_FLT x142 = x136 * (lh_pz + x80 + x81 + x82);
	const GEN_FLT x143 = x71 - x72;
	const GEN_FLT x144 = x136 * x77;
	const GEN_FLT x145 = x142 * x69 + x143 * x144;
	const GEN_FLT x146 = -x138 * x141 + x145;
	const GEN_FLT x147 = ogeeMag_0 * cos(x102);
	const GEN_FLT x148 = x111 * x147;
	const GEN_FLT x149 = 2.40324066e-5 * x94;
	const GEN_FLT x150 = x110 * (-curve_0 - x103);
	const GEN_FLT x151 = x100 * x99;
	const GEN_FLT x152 = x104 * x151 / ((x112 * x112));
	const GEN_FLT x153 = x113 * x147 * x151;
	const GEN_FLT x154 = 2 * x105 * x114;
	const GEN_FLT x155 =
		-x117 * (x115 * x135 + x132 * x154 + x141 + x146 * x153 +
				 x152 * (x146 * x148 -
						 x150 * (x109 * x132 + x132 * x99 + x135 * x94 +
								 x94 * (x108 * x132 + x135 +
										x94 * (x107 * x132 + x134 + x94 * (x106 * x132 - x132 * x149 + x133)))))) +
		x145;
	const GEN_FLT x156 = gibMag_0 * cos(gibPhase_0 + x101 - asin(x116));
	const GEN_FLT x157 = -2 * x56;
	const GEN_FLT x158 = 2 * x59;
	const GEN_FLT x159 = -x124 * (-x120 + x121) - x127 * (x157 - x158);
	const GEN_FLT x160 = -x122 * (2 * x11 + x123) + x159;
	const GEN_FLT x161 = x119 * (x12 * x93 + x131 * x160);
	const GEN_FLT x162 = x161 * x96;
	const GEN_FLT x163 = x161 * x97 + x94 * (-x161 * x95 + x162);
	const GEN_FLT x164 = x161 * x98 + x163 * x94;
	const GEN_FLT x165 = x12 * x86 + x140 * x159;
	const GEN_FLT x166 = x57 - x59;
	const GEN_FLT x167 = x142 * x75 + x144 * x166;
	const GEN_FLT x168 = -x138 * x165 + x167;
	const GEN_FLT x169 =
		-x117 * (x115 * x164 +
				 x152 * (x148 * x168 -
						 x150 * (x109 * x161 + x161 * x99 + x164 * x94 +
								 x94 * (x108 * x161 + x164 +
										x94 * (x107 * x161 + x163 + x94 * (x106 * x161 - x149 * x161 + x162))))) +
				 x153 * x168 + x154 * x161 + x165) +
		x167;
	const GEN_FLT x170 = -x124 * (x125 + x126) - x127 * (-x123 - 2 * x79);
	const GEN_FLT x171 = -x122 * (x157 + x158) + x170;
	const GEN_FLT x172 = x119 * (x131 * x171 + x60 * x93);
	const GEN_FLT x173 = x172 * x96;
	const GEN_FLT x174 = x172 * x97 + x94 * (-x172 * x95 + x173);
	const GEN_FLT x175 = x172 * x98 + x174 * x94;
	const GEN_FLT x176 = x140 * x170 + x60 * x86;
	const GEN_FLT x177 = x6 - x79;
	const GEN_FLT x178 = x142 * x73 + x144 * x177;
	const GEN_FLT x179 = -x138 * x176 + x178;
	const GEN_FLT x180 =
		-x117 * (x115 * x175 +
				 x152 * (x148 * x179 -
						 x150 * (x109 * x172 + x172 * x99 + x175 * x94 +
								 x94 * (x108 * x172 + x175 +
										x94 * (x107 * x172 + x174 + x94 * (x106 * x172 - x149 * x172 + x173))))) +
				 x153 * x179 + x154 * x172 + x176) +
		x178;
	const GEN_FLT x181 = x23 * x30;
	const GEN_FLT x182 = -x181;
	const GEN_FLT x183 = 2 / ((x16 * x16));
	const GEN_FLT x184 = obj_qi * x183;
	const GEN_FLT x185 = pow(x16, -3.0 / 2.0);
	const GEN_FLT x186 = obj_qi * x185;
	const GEN_FLT x187 = ((x21) ? (-obj_qk * x186) : (0));
	const GEN_FLT x188 = x187 * x23;
	const GEN_FLT x189 = x18 * x30;
	const GEN_FLT x190 = x189 * x26;
	const GEN_FLT x191 = ((x21) ? (-obj_qj * x186) : (0));
	const GEN_FLT x192 = ((x21) ? (-x13 * x185 + x24) : (0));
	const GEN_FLT x193 = x30 * x31;
	const GEN_FLT x194 = x191 * x32 + x192 * x35 + x193 * x52;
	const GEN_FLT x195 = x192 * x23;
	const GEN_FLT x196 = x189 * x31;
	const GEN_FLT x197 = x187 * x35;
	const GEN_FLT x198 = x19 * x26;
	const GEN_FLT x199 = x191 * x198;
	const GEN_FLT x200 = x27 * x29;
	const GEN_FLT x201 = x197 + x199 + x200 * x30;
	const GEN_FLT x202 = sensor_x * (x188 + x190 + x194) +
						 sensor_y * (x181 * x22 + x182 + x19 * ((x21) ? (-x14 * x184) : (0))) +
						 sensor_z * (-x195 - x196 + x201);
	const GEN_FLT x203 = x12 * x202;
	const GEN_FLT x204 = x191 * x23;
	const GEN_FLT x205 = -x204;
	const GEN_FLT x206 = x189 * x29;
	const GEN_FLT x207 = x187 * x32 + x192 * x198 + x193 * x27;
	const GEN_FLT x208 = sensor_x * (x205 - x206 + x207) + sensor_y * (x195 + x196 + x201) +
						 sensor_z * (x181 * x61 + x182 + x19 * ((x21) ? (-x15 * x184) : (0)));
	const GEN_FLT x209 = x208 * x60;
	const GEN_FLT x210 = 2 * x20;
	const GEN_FLT x211 = -x188;
	const GEN_FLT x212 =
		sensor_x * (x181 * x51 + x182 + x19 * ((x21) ? (obj_qi * x210 - x183 * obj_qi * (obj_qi * obj_qi)) : (0))) +
		sensor_y * (-x190 + x194 + x211) + sensor_z * (x204 + x206 + x207);
	const GEN_FLT x213 = x212 * x50;
	const GEN_FLT x214 = x203 + x209 + x213;
	const GEN_FLT x215 = x212 * x69;
	const GEN_FLT x216 = x208 * x73;
	const GEN_FLT x217 = x202 * x75;
	const GEN_FLT x218 = x177 * x208;
	const GEN_FLT x219 = x166 * x202;
	const GEN_FLT x220 = x143 * x212;
	const GEN_FLT x221 = -x124 * (2 * x215 + 2 * x216 + 2 * x217) - x127 * (2 * x218 + 2 * x219 + 2 * x220);
	const GEN_FLT x222 = -x122 * (2 * x203 + 2 * x209 + 2 * x213) + x221;
	const GEN_FLT x223 = x119 * (x131 * x222 + x214 * x93);
	const GEN_FLT x224 = x223 * x96;
	const GEN_FLT x225 = x223 * x97 + x94 * (-x223 * x95 + x224);
	const GEN_FLT x226 = x223 * x98 + x225 * x94;
	const GEN_FLT x227 = x140 * x221 + x214 * x86;
	const GEN_FLT x228 = x142 * (x215 + x216 + x217) + x144 * (x218 + x219 + x220);
	const GEN_FLT x229 = -x138 * x227 + x228;
	const GEN_FLT x230 =
		-x117 * (x115 * x226 +
				 x152 * (x148 * x229 -
						 x150 * (x109 * x223 + x223 * x99 + x226 * x94 +
								 x94 * (x108 * x223 + x226 +
										x94 * (x107 * x223 + x225 + x94 * (x106 * x223 - x149 * x223 + x224))))) +
				 x153 * x229 + x154 * x223 + x227) +
		x228;
	const GEN_FLT x231 = x23 * x28;
	const GEN_FLT x232 = -x231;
	const GEN_FLT x233 = ((x21) ? (-obj_qj * obj_qk * x185) : (0));
	const GEN_FLT x234 = x23 * x233;
	const GEN_FLT x235 = x18 * x28;
	const GEN_FLT x236 = x235 * x26;
	const GEN_FLT x237 = ((x21) ? (-x14 * x185 + x24) : (0));
	const GEN_FLT x238 = x28 * x31;
	const GEN_FLT x239 = x191 * x35 + x237 * x32 + x238 * x52;
	const GEN_FLT x240 = x235 * x31;
	const GEN_FLT x241 = x198 * x237 + x200 * x28 + x233 * x35;
	const GEN_FLT x242 =
		sensor_x * (x234 + x236 + x239) +
		sensor_y * (x19 * ((x21) ? (obj_qj * x210 - x183 * obj_qj * (obj_qj * obj_qj)) : (0)) + x22 * x231 + x232) +
		sensor_z * (x205 - x240 + x241);
	const GEN_FLT x243 = x12 * x242;
	const GEN_FLT x244 = obj_qj * x183;
	const GEN_FLT x245 = x23 * x237;
	const GEN_FLT x246 = x235 * x29;
	const GEN_FLT x247 = x233 * x32;
	const GEN_FLT x248 = x199 + x238 * x27 + x247;
	const GEN_FLT x249 = -x234;
	const GEN_FLT x250 = sensor_x * (x19 * ((x21) ? (-x13 * x244) : (0)) + x231 * x51 + x232) +
						 sensor_y * (-x236 + x239 + x249) + sensor_z * (x245 + x246 + x248);
	const GEN_FLT x251 = x250 * x50;
	const GEN_FLT x252 = sensor_x * (-x245 - x246 + x248) + sensor_y * (x204 + x240 + x241) +
						 sensor_z * (x19 * ((x21) ? (-x15 * x244) : (0)) + x231 * x61 + x232);
	const GEN_FLT x253 = x252 * x60;
	const GEN_FLT x254 = x243 + x251 + x253;
	const GEN_FLT x255 = x250 * x69;
	const GEN_FLT x256 = x252 * x73;
	const GEN_FLT x257 = x242 * x75;
	const GEN_FLT x258 = x177 * x252;
	const GEN_FLT x259 = x143 * x250;
	const GEN_FLT x260 = x166 * x242;
	const GEN_FLT x261 = -x124 * (2 * x255 + 2 * x256 + 2 * x257) - x127 * (2 * x258 + 2 * x259 + 2 * x260);
	const GEN_FLT x262 = -x122 * (2 * x243 + 2 * x251 + 2 * x253) + x261;
	const GEN_FLT x263 = x119 * (x131 * x262 + x254 * x93);
	const GEN_FLT x264 = x263 * x96;
	const GEN_FLT x265 = x263 * x97 + x94 * (-x263 * x95 + x264);
	const GEN_FLT x266 = x263 * x98 + x265 * x94;
	const GEN_FLT x267 = x140 * x261 + x254 * x86;
	const GEN_FLT x268 = x142 * (x255 + x256 + x257) + x144 * (x258 + x259 + x260);
	const GEN_FLT x269 = -x138 * x267 + x268;
	const GEN_FLT x270 =
		-x117 * (x115 * x266 +
				 x152 * (x148 * x269 -
						 x150 * (x109 * x263 + x263 * x99 + x266 * x94 +
								 x94 * (x108 * x263 + x266 +
										x94 * (x107 * x263 + x265 + x94 * (x106 * x263 - x149 * x263 + x264))))) +
				 x153 * x269 + x154 * x263 + x267) +
		x268;
	const GEN_FLT x271 = x23 * x25;
	const GEN_FLT x272 = -x271;
	const GEN_FLT x273 = obj_qk * x183;
	const GEN_FLT x274 = ((x21) ? (-x15 * x185 + x24) : (0));
	const GEN_FLT x275 = x23 * x274;
	const GEN_FLT x276 = x18 * x25;
	const GEN_FLT x277 = x26 * x276;
	const GEN_FLT x278 = x25 * x31;
	const GEN_FLT x279 = x197 + x247 + x278 * x52;
	const GEN_FLT x280 = x276 * x31;
	const GEN_FLT x281 = x198 * x233 + x200 * x25 + x274 * x35;
	const GEN_FLT x282 = sensor_x * (x275 + x277 + x279) +
						 sensor_y * (x19 * ((x21) ? (-x14 * x273) : (0)) + x22 * x271 + x272) +
						 sensor_z * (x211 - x280 + x281);
	const GEN_FLT x283 = x12 * x282;
	const GEN_FLT x284 = x276 * x29;
	const GEN_FLT x285 = x187 * x198 + x27 * x278 + x274 * x32;
	const GEN_FLT x286 = sensor_x * (x19 * ((x21) ? (-x13 * x273) : (0)) + x271 * x51 + x272) +
						 sensor_y * (-x275 - x277 + x279) + sensor_z * (x234 + x284 + x285);
	const GEN_FLT x287 = x286 * x50;
	const GEN_FLT x288 =
		sensor_x * (x249 - x284 + x285) + sensor_y * (x188 + x280 + x281) +
		sensor_z * (x19 * ((x21) ? (obj_qk * x210 - x183 * obj_qk * (obj_qk * obj_qk)) : (0)) + x271 * x61 + x272);
	const GEN_FLT x289 = x288 * x60;
	const GEN_FLT x290 = x283 + x287 + x289;
	const GEN_FLT x291 = x286 * x69;
	const GEN_FLT x292 = x282 * x75;
	const GEN_FLT x293 = x288 * x73;
	const GEN_FLT x294 = x177 * x288;
	const GEN_FLT x295 = x143 * x286;
	const GEN_FLT x296 = x166 * x282;
	const GEN_FLT x297 = -x124 * (2 * x291 + 2 * x292 + 2 * x293) - x127 * (2 * x294 + 2 * x295 + 2 * x296);
	const GEN_FLT x298 = -x122 * (2 * x283 + 2 * x287 + 2 * x289) + x297;
	const GEN_FLT x299 = x119 * (x131 * x298 + x290 * x93);
	const GEN_FLT x300 = x299 * x96;
	const GEN_FLT x301 = x299 * x97 + x94 * (-x299 * x95 + x300);
	const GEN_FLT x302 = x299 * x98 + x301 * x94;
	const GEN_FLT x303 = x140 * x297 + x290 * x86;
	const GEN_FLT x304 = x142 * (x291 + x292 + x293) + x144 * (x294 + x295 + x296);
	const GEN_FLT x305 = -x138 * x303 + x304;
	const GEN_FLT x306 =
		-x117 * (x115 * x302 +
				 x152 * (x148 * x305 -
						 x150 * (x109 * x299 + x299 * x99 + x302 * x94 +
								 x94 * (x108 * x299 + x302 +
										x94 * (x107 * x299 + x301 + x94 * (x106 * x299 - x149 * x299 + x300))))) +
				 x153 * x305 + x154 * x299 + x303) +
		x304;
	const GEN_FLT x307 = -lh_px - x70 - x74 - x76;
	const GEN_FLT x308 = x140 * x307;
	const GEN_FLT x309 = x119 * x131;
	const GEN_FLT x310 = x307 * x309;
	const GEN_FLT x311 = -x138 * x308 + x142;
	const GEN_FLT x312 = x310 * x96;
	const GEN_FLT x313 = x310 * x97 + x94 * (-x310 * x95 + x312);
	const GEN_FLT x314 = x310 * x98 + x313 * x94;
	const GEN_FLT x315 =
		-x117 * (x115 * x314 +
				 x152 * (x148 * x311 -
						 x150 * (x109 * x310 + x310 * x99 + x314 * x94 +
								 x94 * (x108 * x310 + x314 +
										x94 * (x107 * x310 + x313 + x94 * (x106 * x310 - x149 * x310 + x312))))) +
				 x153 * x311 + x154 * x310 + x308) +
		x142;
	const GEN_FLT x316 = x138 * x86;
	const GEN_FLT x317 = -lh_py - x38 - x55 - x63;
	const GEN_FLT x318 = x119 * (x131 * x317 + x93);
	const GEN_FLT x319 = x318 * x96;
	const GEN_FLT x320 = x318 * x97 + x94 * (-x318 * x95 + x319);
	const GEN_FLT x321 = x318 * x98 + x320 * x94;
	const GEN_FLT x322 =
		x117 * (x115 * x321 +
				x152 * (-x148 * x316 -
						x150 * (x109 * x318 + x318 * x99 + x321 * x94 +
								x94 * (x108 * x318 + x321 +
									   x94 * (x107 * x318 + x320 + x94 * (x106 * x318 - x149 * x318 + x319))))) -
				x153 * x316 + x154 * x318 + x86);
	const GEN_FLT x323 = -x144;
	const GEN_FLT x324 = x140 * x83;
	const GEN_FLT x325 = x309 * x83;
	const GEN_FLT x326 = -x138 * x324 + x323;
	const GEN_FLT x327 = x325 * x96;
	const GEN_FLT x328 = x325 * x97 + x94 * (-x325 * x95 + x327);
	const GEN_FLT x329 = x325 * x98 + x328 * x94;
	const GEN_FLT x330 =
		-x117 * (x115 * x329 +
				 x152 * (x148 * x326 -
						 x150 * (x109 * x325 + x325 * x99 + x329 * x94 +
								 x94 * (x108 * x325 + x329 +
										x94 * (x107 * x325 + x328 + x94 * (x106 * x325 - x149 * x325 + x327))))) +
				 x153 * x326 + x154 * x325 + x324) +
		x323;
	const GEN_FLT x331 = x39 * x46;
	const GEN_FLT x332 = -x331;
	const GEN_FLT x333 = 2 / ((x3 * x3));
	const GEN_FLT x334 = lh_qi * x333;
	const GEN_FLT x335 = x37 * (x10 * x331 + x332 + x7 * ((x9) ? (-x1 * x334) : (0)));
	const GEN_FLT x336 = pow(x3, -3.0 / 2.0);
	const GEN_FLT x337 = lh_qi * x336;
	const GEN_FLT x338 = ((x9) ? (-lh_qk * x337) : (0));
	const GEN_FLT x339 = x338 * x39;
	const GEN_FLT x340 = x46 * x5;
	const GEN_FLT x341 = x340 * x42;
	const GEN_FLT x342 = ((x9) ? (-lh_qj * x337) : (0));
	const GEN_FLT x343 = ((x9) ? (-x0 * x336 + x40) : (0));
	const GEN_FLT x344 = x46 * x71;
	const GEN_FLT x345 = x342 * x48 + x343 * x58 + x344 * x47;
	const GEN_FLT x346 = x54 * (x339 + x341 + x345);
	const GEN_FLT x347 = x338 * x58;
	const GEN_FLT x348 = x42 * x7;
	const GEN_FLT x349 = x342 * x348;
	const GEN_FLT x350 = x344 * x42;
	const GEN_FLT x351 = -x340 * x47 - x343 * x39;
	const GEN_FLT x352 = x62 * (x347 + x349 + x350 + x351);
	const GEN_FLT x353 = x335 + x346 + x352;
	const GEN_FLT x354 = x62 * (-x331 * x78 + x331 - x7 * ((x9) ? (-x2 * x334) : (0)));
	const GEN_FLT x355 = x338 * x48;
	const GEN_FLT x356 = x343 * x348;
	const GEN_FLT x357 = x43 * x47;
	const GEN_FLT x358 = x357 * x46;
	const GEN_FLT x359 = x342 * x39;
	const GEN_FLT x360 = x340 * x45 + x359;
	const GEN_FLT x361 = x54 * (-x355 - x356 - x358 + x360);
	const GEN_FLT x362 = -x349;
	const GEN_FLT x363 = x37 * (-x347 - x350 + x351 + x362);
	const GEN_FLT x364 = 2 * x8;
	const GEN_FLT x365 =
		x54 * (x331 * x67 + x332 + x7 * ((x9) ? (lh_qi * x364 - x333 * lh_qi * (lh_qi * lh_qi)) : (0)));
	const GEN_FLT x366 = x62 * (x355 + x356 + x358 + x360);
	const GEN_FLT x367 = -x339;
	const GEN_FLT x368 = x37 * (-x341 + x345 + x367);
	const GEN_FLT x369 = -x124 * (2 * x365 + 2 * x366 + 2 * x368) - x127 * (2 * x354 + 2 * x361 + 2 * x363);
	const GEN_FLT x370 = -x122 * (2 * x335 + 2 * x346 + 2 * x352) + x369;
	const GEN_FLT x371 = x119 * (x131 * x370 + x353 * x93);
	const GEN_FLT x372 = x371 * x96;
	const GEN_FLT x373 = x371 * x97 + x94 * (-x371 * x95 + x372);
	const GEN_FLT x374 = x371 * x98 + x373 * x94;
	const GEN_FLT x375 = x140 * x369 + x353 * x86;
	const GEN_FLT x376 = x142 * (x365 + x366 + x368) + x144 * (x354 + x361 + x363);
	const GEN_FLT x377 = -x138 * x375 + x376;
	const GEN_FLT x378 =
		-x117 * (x115 * x374 +
				 x152 * (x148 * x377 -
						 x150 * (x109 * x371 + x371 * x99 + x374 * x94 +
								 x94 * (x108 * x371 + x374 +
										x94 * (x107 * x371 + x373 + x94 * (x106 * x371 - x149 * x371 + x372))))) +
				 x153 * x377 + x154 * x371 + x375) +
		x376;
	const GEN_FLT x379 = x39 * x44;
	const GEN_FLT x380 = -x379;
	const GEN_FLT x381 =
		x37 * (x10 * x379 + x380 + x7 * ((x9) ? (lh_qj * x364 - x333 * lh_qj * (lh_qj * lh_qj)) : (0)));
	const GEN_FLT x382 = ((x9) ? (-lh_qj * lh_qk * x336) : (0));
	const GEN_FLT x383 = x382 * x39;
	const GEN_FLT x384 = x44 * x5;
	const GEN_FLT x385 = x384 * x42;
	const GEN_FLT x386 = ((x9) ? (-x1 * x336 + x40) : (0));
	const GEN_FLT x387 = x44 * x71;
	const GEN_FLT x388 = x342 * x58 + x386 * x48 + x387 * x47;
	const GEN_FLT x389 = x54 * (x383 + x385 + x388);
	const GEN_FLT x390 = x382 * x58;
	const GEN_FLT x391 = x348 * x386;
	const GEN_FLT x392 = x387 * x42;
	const GEN_FLT x393 = -x359 - x384 * x47;
	const GEN_FLT x394 = x62 * (x390 + x391 + x392 + x393);
	const GEN_FLT x395 = x381 + x389 + x394;
	const GEN_FLT x396 = lh_qj * x333;
	const GEN_FLT x397 = x54 * (x379 * x67 + x380 + x7 * ((x9) ? (-x0 * x396) : (0)));
	const GEN_FLT x398 = x382 * x48;
	const GEN_FLT x399 = x357 * x44;
	const GEN_FLT x400 = x384 * x45 + x386 * x39;
	const GEN_FLT x401 = x62 * (x349 + x398 + x399 + x400);
	const GEN_FLT x402 = x37 * (-x383 - x385 + x388);
	const GEN_FLT x403 = x62 * (-x379 * x78 + x379 - x7 * ((x9) ? (-x2 * x396) : (0)));
	const GEN_FLT x404 = x54 * (x362 - x398 - x399 + x400);
	const GEN_FLT x405 = x37 * (-x390 - x391 - x392 + x393);
	const GEN_FLT x406 = -x124 * (2 * x397 + 2 * x401 + 2 * x402) - x127 * (2 * x403 + 2 * x404 + 2 * x405);
	const GEN_FLT x407 = -x122 * (2 * x381 + 2 * x389 + 2 * x394) + x406;
	const GEN_FLT x408 = x119 * (x131 * x407 + x395 * x93);
	const GEN_FLT x409 = x408 * x96;
	const GEN_FLT x410 = x408 * x97 + x94 * (-x408 * x95 + x409);
	const GEN_FLT x411 = x408 * x98 + x410 * x94;
	const GEN_FLT x412 = x140 * x406 + x395 * x86;
	const GEN_FLT x413 = x142 * (x397 + x401 + x402) + x144 * (x403 + x404 + x405);
	const GEN_FLT x414 = -x138 * x412 + x413;
	const GEN_FLT x415 =
		-x117 * (x115 * x411 +
				 x152 * (x148 * x414 -
						 x150 * (x109 * x408 + x408 * x99 + x411 * x94 +
								 x94 * (x108 * x408 + x411 +
										x94 * (x107 * x408 + x410 + x94 * (x106 * x408 - x149 * x408 + x409))))) +
				 x153 * x414 + x154 * x408 + x412) +
		x413;
	const GEN_FLT x416 = x39 * x41;
	const GEN_FLT x417 = -x416;
	const GEN_FLT x418 = lh_qk * x333;
	const GEN_FLT x419 = x37 * (x10 * x416 + x417 + x7 * ((x9) ? (-x1 * x418) : (0)));
	const GEN_FLT x420 = ((x9) ? (-x2 * x336 + x40) : (0));
	const GEN_FLT x421 = x39 * x420;
	const GEN_FLT x422 = x41 * x5;
	const GEN_FLT x423 = x42 * x422;
	const GEN_FLT x424 = x41 * x71;
	const GEN_FLT x425 = x347 + x398 + x424 * x47;
	const GEN_FLT x426 = x54 * (x421 + x423 + x425);
	const GEN_FLT x427 = x420 * x58;
	const GEN_FLT x428 = x348 * x382;
	const GEN_FLT x429 = x42 * x424;
	const GEN_FLT x430 = x367 - x422 * x47;
	const GEN_FLT x431 = x62 * (x427 + x428 + x429 + x430);
	const GEN_FLT x432 = x419 + x426 + x431;
	const GEN_FLT x433 = x54 * (x416 * x67 + x417 + x7 * ((x9) ? (-x0 * x418) : (0)));
	const GEN_FLT x434 = x420 * x48;
	const GEN_FLT x435 = x338 * x348;
	const GEN_FLT x436 = x357 * x41;
	const GEN_FLT x437 = x383 + x422 * x45;
	const GEN_FLT x438 = x62 * (x434 + x435 + x436 + x437);
	const GEN_FLT x439 = x37 * (-x421 - x423 + x425);
	const GEN_FLT x440 =
		x62 * (-x416 * x78 + x416 - x7 * ((x9) ? (lh_qk * x364 - x333 * lh_qk * (lh_qk * lh_qk)) : (0)));
	const GEN_FLT x441 = x54 * (-x434 - x435 - x436 + x437);
	const GEN_FLT x442 = x37 * (-x427 - x428 - x429 + x430);
	const GEN_FLT x443 = -x124 * (2 * x433 + 2 * x438 + 2 * x439) - x127 * (2 * x440 + 2 * x441 + 2 * x442);
	const GEN_FLT x444 = -x122 * (2 * x419 + 2 * x426 + 2 * x431) + x443;
	const GEN_FLT x445 = x119 * (x131 * x444 + x432 * x93);
	const GEN_FLT x446 = x445 * x96;
	const GEN_FLT x447 = x445 * x97 + x94 * (-x445 * x95 + x446);
	const GEN_FLT x448 = x445 * x98 + x447 * x94;
	const GEN_FLT x449 = x140 * x443 + x432 * x86;
	const GEN_FLT x450 = x142 * (x433 + x438 + x439) + x144 * (x440 + x441 + x442);
	const GEN_FLT x451 = -x138 * x449 + x450;
	const GEN_FLT x452 =
		-x117 * (x115 * x448 +
				 x152 * (x148 * x451 -
						 x150 * (x109 * x445 + x445 * x99 + x448 * x94 +
								 x94 * (x108 * x445 + x448 +
										x94 * (x107 * x445 + x447 + x94 * (x106 * x445 - x149 * x445 + x446))))) +
				 x153 * x451 + x154 * x445 + x449) +
		x450;
	const GEN_FLT x453 = tilt_1 - 0.52359877559829882;
	const GEN_FLT x454 = tan(x453);
	const GEN_FLT x455 = x454 * x85;
	const GEN_FLT x456 = x455 * x64;
	const GEN_FLT x457 = cos(x453);
	const GEN_FLT x458 = 1.0 / x457;
	const GEN_FLT x459 = x458 * x92;
	const GEN_FLT x460 = asin(x459 * x64);
	const GEN_FLT x461 = 8.0108022e-6 * x460;
	const GEN_FLT x462 = -x461 - 8.0108022e-6;
	const GEN_FLT x463 = x460 * x462 + 0.0028679863;
	const GEN_FLT x464 = x460 * x463 + 5.3685255000000001e-6;
	const GEN_FLT x465 = x460 * x464 + 0.0076069798000000001;
	const GEN_FLT x466 = x460 * x460;
	const GEN_FLT x467 = ogeePhase_1 + x101 - asin(x456);
	const GEN_FLT x468 = ogeeMag_1 * sin(x467);
	const GEN_FLT x469 = curve_1 + x468;
	const GEN_FLT x470 = x460 * x465;
	const GEN_FLT x471 = -1.60216044e-5 * x460 - 8.0108022e-6;
	const GEN_FLT x472 = x460 * x471 + x463;
	const GEN_FLT x473 = x460 * x472 + x464;
	const GEN_FLT x474 = x460 * x473 + x465;
	const GEN_FLT x475 = sin(x453);
	const GEN_FLT x476 = x475 * (x460 * x474 + x470);
	const GEN_FLT x477 = x457 - x469 * x476;
	const GEN_FLT x478 = 1.0 / x477;
	const GEN_FLT x479 = x469 * x478;
	const GEN_FLT x480 = x466 * x479;
	const GEN_FLT x481 = x456 + x465 * x480;
	const GEN_FLT x482 = pow(1 - x481 * x481, -1.0 / 2.0);
	const GEN_FLT x483 = pow(-x118 / (x457 * x457) + 1, -1.0 / 2.0);
	const GEN_FLT x484 = x130 * x458;
	const GEN_FLT x485 = x483 * (x129 * x484 + x459 * x50);
	const GEN_FLT x486 = x462 * x485;
	const GEN_FLT x487 = x460 * (-x461 * x485 + x486) + x463 * x485;
	const GEN_FLT x488 = x460 * x487 + x464 * x485;
	const GEN_FLT x489 = pow(-x137 * x454 * x454 + 1, -1.0 / 2.0);
	const GEN_FLT x490 = x139 * x454;
	const GEN_FLT x491 = x128 * x490 + x455 * x50;
	const GEN_FLT x492 = x145 - x489 * x491;
	const GEN_FLT x493 = ogeeMag_1 * cos(x467);
	const GEN_FLT x494 = x476 * x493;
	const GEN_FLT x495 = 2.40324066e-5 * x460;
	const GEN_FLT x496 = x475 * (-curve_1 - x468);
	const GEN_FLT x497 = x465 * x466;
	const GEN_FLT x498 = x469 * x497 / ((x477 * x477));
	const GEN_FLT x499 = x478 * x493 * x497;
	const GEN_FLT x500 = 2 * x470 * x479;
	const GEN_FLT x501 =
		x145 - x482 * (x480 * x488 + x485 * x500 + x491 + x492 * x499 +
					   x498 * (x492 * x494 -
							   x496 * (x460 * x488 +
									   x460 * (x460 * (x460 * (x471 * x485 - x485 * x495 + x486) + x472 * x485 + x487) +
											   x473 * x485 + x488) +
									   x465 * x485 + x474 * x485)));
	const GEN_FLT x502 = gibMag_1 * cos(gibPhase_1 + x101 - asin(x481));
	const GEN_FLT x503 = x483 * (x12 * x459 + x160 * x484);
	const GEN_FLT x504 = x462 * x503;
	const GEN_FLT x505 = x460 * (-x461 * x503 + x504) + x463 * x503;
	const GEN_FLT x506 = x460 * x505 + x464 * x503;
	const GEN_FLT x507 = x12 * x455 + x159 * x490;
	const GEN_FLT x508 = x167 - x489 * x507;
	const GEN_FLT x509 =
		x167 - x482 * (x480 * x506 +
					   x498 * (x494 * x508 -
							   x496 * (x460 * x506 +
									   x460 * (x460 * (x460 * (x471 * x503 - x495 * x503 + x504) + x472 * x503 + x505) +
											   x473 * x503 + x506) +
									   x465 * x503 + x474 * x503)) +
					   x499 * x508 + x500 * x503 + x507);
	const GEN_FLT x510 = x483 * (x171 * x484 + x459 * x60);
	const GEN_FLT x511 = x462 * x510;
	const GEN_FLT x512 = x460 * (-x461 * x510 + x511) + x463 * x510;
	const GEN_FLT x513 = x460 * x512 + x464 * x510;
	const GEN_FLT x514 = x170 * x490 + x455 * x60;
	const GEN_FLT x515 = x178 - x489 * x514;
	const GEN_FLT x516 =
		x178 - x482 * (x480 * x513 +
					   x498 * (x494 * x515 -
							   x496 * (x460 * x513 +
									   x460 * (x460 * (x460 * (x471 * x510 - x495 * x510 + x511) + x472 * x510 + x512) +
											   x473 * x510 + x513) +
									   x465 * x510 + x474 * x510)) +
					   x499 * x515 + x500 * x510 + x514);
	const GEN_FLT x517 = x483 * (x214 * x459 + x222 * x484);
	const GEN_FLT x518 = x462 * x517;
	const GEN_FLT x519 = x460 * (-x461 * x517 + x518) + x463 * x517;
	const GEN_FLT x520 = x460 * x519 + x464 * x517;
	const GEN_FLT x521 = x214 * x455 + x221 * x490;
	const GEN_FLT x522 = x228 - x489 * x521;
	const GEN_FLT x523 =
		x228 - x482 * (x480 * x520 +
					   x498 * (x494 * x522 -
							   x496 * (x460 * x520 +
									   x460 * (x460 * (x460 * (x471 * x517 - x495 * x517 + x518) + x472 * x517 + x519) +
											   x473 * x517 + x520) +
									   x465 * x517 + x474 * x517)) +
					   x499 * x522 + x500 * x517 + x521);
	const GEN_FLT x524 = x483 * (x254 * x459 + x262 * x484);
	const GEN_FLT x525 = x462 * x524;
	const GEN_FLT x526 = x460 * (-x461 * x524 + x525) + x463 * x524;
	const GEN_FLT x527 = x460 * x526 + x464 * x524;
	const GEN_FLT x528 = x254 * x455 + x261 * x490;
	const GEN_FLT x529 = x268 - x489 * x528;
	const GEN_FLT x530 =
		x268 - x482 * (x480 * x527 +
					   x498 * (x494 * x529 -
							   x496 * (x460 * x527 +
									   x460 * (x460 * (x460 * (x471 * x524 - x495 * x524 + x525) + x472 * x524 + x526) +
											   x473 * x524 + x527) +
									   x465 * x524 + x474 * x524)) +
					   x499 * x529 + x500 * x524 + x528);
	const GEN_FLT x531 = x483 * (x290 * x459 + x298 * x484);
	const GEN_FLT x532 = x462 * x531;
	const GEN_FLT x533 = x460 * (-x461 * x531 + x532) + x463 * x531;
	const GEN_FLT x534 = x460 * x533 + x464 * x531;
	const GEN_FLT x535 = x290 * x455 + x297 * x490;
	const GEN_FLT x536 = x304 - x489 * x535;
	const GEN_FLT x537 =
		x304 - x482 * (x480 * x534 +
					   x498 * (x494 * x536 -
							   x496 * (x460 * x534 +
									   x460 * (x460 * (x460 * (x471 * x531 - x495 * x531 + x532) + x472 * x531 + x533) +
											   x473 * x531 + x534) +
									   x465 * x531 + x474 * x531)) +
					   x499 * x536 + x500 * x531 + x535);
	const GEN_FLT x538 = x307 * x490;
	const GEN_FLT x539 = x483 * x484;
	const GEN_FLT x540 = x307 * x539;
	const GEN_FLT x541 = x142 - x489 * x538;
	const GEN_FLT x542 = x462 * x540;
	const GEN_FLT x543 = x460 * (-x461 * x540 + x542) + x463 * x540;
	const GEN_FLT x544 = x460 * x543 + x464 * x540;
	const GEN_FLT x545 =
		x142 - x482 * (x480 * x544 +
					   x498 * (x494 * x541 -
							   x496 * (x460 * x544 +
									   x460 * (x460 * (x460 * (x471 * x540 - x495 * x540 + x542) + x472 * x540 + x543) +
											   x473 * x540 + x544) +
									   x465 * x540 + x474 * x540)) +
					   x499 * x541 + x500 * x540 + x538);
	const GEN_FLT x546 = x455 * x489;
	const GEN_FLT x547 = x483 * (x317 * x484 + x459);
	const GEN_FLT x548 = x462 * x547;
	const GEN_FLT x549 = x460 * (-x461 * x547 + x548) + x463 * x547;
	const GEN_FLT x550 = x460 * x549 + x464 * x547;
	const GEN_FLT x551 =
		x482 * (x455 + x480 * x550 +
				x498 * (-x494 * x546 -
						x496 * (x460 * x550 +
								x460 * (x460 * (x460 * (x471 * x547 - x495 * x547 + x548) + x472 * x547 + x549) +
										x473 * x547 + x550) +
								x465 * x547 + x474 * x547)) -
				x499 * x546 + x500 * x547);
	const GEN_FLT x552 = x490 * x83;
	const GEN_FLT x553 = x539 * x83;
	const GEN_FLT x554 = x323 - x489 * x552;
	const GEN_FLT x555 = x462 * x553;
	const GEN_FLT x556 = x460 * (-x461 * x553 + x555) + x463 * x553;
	const GEN_FLT x557 = x460 * x556 + x464 * x553;
	const GEN_FLT x558 =
		x323 - x482 * (x480 * x557 +
					   x498 * (x494 * x554 -
							   x496 * (x460 * x557 +
									   x460 * (x460 * (x460 * (x471 * x553 - x495 * x553 + x555) + x472 * x553 + x556) +
											   x473 * x553 + x557) +
									   x465 * x553 + x474 * x553)) +
					   x499 * x554 + x500 * x553 + x552);
	const GEN_FLT x559 = x483 * (x353 * x459 + x370 * x484);
	const GEN_FLT x560 = x462 * x559;
	const GEN_FLT x561 = x460 * (-x461 * x559 + x560) + x463 * x559;
	const GEN_FLT x562 = x460 * x561 + x464 * x559;
	const GEN_FLT x563 = x353 * x455 + x369 * x490;
	const GEN_FLT x564 = x376 - x489 * x563;
	const GEN_FLT x565 =
		x376 - x482 * (x480 * x562 +
					   x498 * (x494 * x564 -
							   x496 * (x460 * x562 +
									   x460 * (x460 * (x460 * (x471 * x559 - x495 * x559 + x560) + x472 * x559 + x561) +
											   x473 * x559 + x562) +
									   x465 * x559 + x474 * x559)) +
					   x499 * x564 + x500 * x559 + x563);
	const GEN_FLT x566 = x483 * (x395 * x459 + x407 * x484);
	const GEN_FLT x567 = x462 * x566;
	const GEN_FLT x568 = x460 * (-x461 * x566 + x567) + x463 * x566;
	const GEN_FLT x569 = x460 * x568 + x464 * x566;
	const GEN_FLT x570 = x395 * x455 + x406 * x490;
	const GEN_FLT x571 = x413 - x489 * x570;
	const GEN_FLT x572 =
		x413 - x482 * (x480 * x569 +
					   x498 * (x494 * x571 -
							   x496 * (x460 * x569 +
									   x460 * (x460 * (x460 * (x471 * x566 - x495 * x566 + x567) + x472 * x566 + x568) +
											   x473 * x566 + x569) +
									   x465 * x566 + x474 * x566)) +
					   x499 * x571 + x500 * x566 + x570);
	const GEN_FLT x573 = x483 * (x432 * x459 + x444 * x484);
	const GEN_FLT x574 = x462 * x573;
	const GEN_FLT x575 = x460 * (-x461 * x573 + x574) + x463 * x573;
	const GEN_FLT x576 = x460 * x575 + x464 * x573;
	const GEN_FLT x577 = x432 * x455 + x443 * x490;
	const GEN_FLT x578 = x450 - x489 * x577;
	const GEN_FLT x579 =
		x450 - x482 * (x480 * x576 +
					   x498 * (x494 * x578 -
							   x496 * (x460 * x576 +
									   x460 * (x460 * (x460 * (x471 * x573 - x495 * x573 + x574) + x472 * x573 + x575) +
											   x473 * x573 + x576) +
									   x465 * x573 + x474 * x573)) +
					   x499 * x578 + x500 * x573 + x577);
	*(out++) = x155 * x156 + x155;
	*(out++) = x156 * x169 + x169;
	*(out++) = x156 * x180 + x180;
	*(out++) = x156 * x230 + x230;
	*(out++) = x156 * x270 + x270;
	*(out++) = x156 * x306 + x306;
	*(out++) = x156 * x315 + x315;
	*(out++) = -x156 * x322 - x322;
	*(out++) = x156 * x330 + x330;
	*(out++) = x156 * x378 + x378;
	*(out++) = x156 * x415 + x415;
	*(out++) = x156 * x452 + x452;
	*(out++) = x501 * x502 + x501;
	*(out++) = x502 * x509 + x509;
	*(out++) = x502 * x516 + x516;
	*(out++) = x502 * x523 + x523;
	*(out++) = x502 * x530 + x530;
	*(out++) = x502 * x537 + x537;
	*(out++) = x502 * x545 + x545;
	*(out++) = -x502 * x551 - x551;
	*(out++) = x502 * x558 + x558;
	*(out++) = x502 * x565 + x565;
	*(out++) = x502 * x572 + x572;
	*(out++) = x502 * x579 + x579;
}

static inline void gen_reproject_axisangle_axis_x_jac_all_gen2(FLT *out, const FLT *obj, const FLT *sensor,
															   const FLT *lh, const FLT phase_0, const FLT tilt_0,
															   const FLT curve_0, const FLT gibPhase_0,
															   const FLT gibMag_0, const FLT ogeePhase_0,
															   const FLT ogeeMag_0) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = -x5;
	const GEN_FLT x7 = x6 + 1;
	const GEN_FLT x8 = 1.0 / x3;
	const GEN_FLT x9 = x4 > 0;
	const GEN_FLT x10 = ((x9) ? (x1 * x8) : (0));
	const GEN_FLT x11 = x10 * x7;
	const GEN_FLT x12 = x11 + x5;
	const GEN_FLT x13 = obj_qi * obj_qi;
	const GEN_FLT x14 = obj_qj * obj_qj;
	const GEN_FLT x15 = obj_qk * obj_qk;
	const GEN_FLT x16 = x13 + x14 + x15;
	const GEN_FLT x17 = sqrt(x16);
	const GEN_FLT x18 = cos(x17);
	const GEN_FLT x19 = 1 - x18;
	const GEN_FLT x20 = 1.0 / x16;
	const GEN_FLT x21 = x17 > 0;
	const GEN_FLT x22 = ((x21) ? (x14 * x20) : (0));
	const GEN_FLT x23 = sin(x17);
	const GEN_FLT x24 = 1.0 / x17;
	const GEN_FLT x25 = obj_qk * x24;
	const GEN_FLT x26 = ((x21) ? (x25) : (0));
	const GEN_FLT x27 = x23 * x26;
	const GEN_FLT x28 = obj_qj * x24;
	const GEN_FLT x29 = ((x21) ? (x28) : (0));
	const GEN_FLT x30 = obj_qi * x24;
	const GEN_FLT x31 = ((x21) ? (x30) : (1));
	const GEN_FLT x32 = x19 * x31;
	const GEN_FLT x33 = x29 * x32;
	const GEN_FLT x34 = x23 * x31;
	const GEN_FLT x35 = x19 * x29;
	const GEN_FLT x36 = x26 * x35;
	const GEN_FLT x37 = obj_py + sensor_x * (x27 + x33) + sensor_y * (x18 + x19 * x22) + sensor_z * (-x34 + x36);
	const GEN_FLT x38 = x12 * x37;
	const GEN_FLT x39 = sin(x4);
	const GEN_FLT x40 = 1.0 / x4;
	const GEN_FLT x41 = lh_qk * x40;
	const GEN_FLT x42 = ((x9) ? (x41) : (0));
	const GEN_FLT x43 = x39 * x42;
	const GEN_FLT x44 = lh_qj * x40;
	const GEN_FLT x45 = ((x9) ? (x44) : (0));
	const GEN_FLT x46 = lh_qi * x40;
	const GEN_FLT x47 = ((x9) ? (x46) : (1));
	const GEN_FLT x48 = x47 * x7;
	const GEN_FLT x49 = x45 * x48;
	const GEN_FLT x50 = x43 + x49;
	const GEN_FLT x51 = ((x21) ? (x13 * x20) : (1));
	const GEN_FLT x52 = x23 * x29;
	const GEN_FLT x53 = x26 * x32;
	const GEN_FLT x54 = obj_px + sensor_x * (x18 + x19 * x51) + sensor_y * (-x27 + x33) + sensor_z * (x52 + x53);
	const GEN_FLT x55 = x50 * x54;
	const GEN_FLT x56 = x39 * x47;
	const GEN_FLT x57 = -x56;
	const GEN_FLT x58 = x45 * x7;
	const GEN_FLT x59 = x42 * x58;
	const GEN_FLT x60 = x57 + x59;
	const GEN_FLT x61 = ((x21) ? (x15 * x20) : (0));
	const GEN_FLT x62 = obj_pz + sensor_x * (-x52 + x53) + sensor_y * (x34 + x36) + sensor_z * (x18 + x19 * x61);
	const GEN_FLT x63 = x60 * x62;
	const GEN_FLT x64 = lh_py + x38 + x55 + x63;
	const GEN_FLT x65 = tilt_0 + 0.52359877559829882;
	const GEN_FLT x66 = tan(x65);
	const GEN_FLT x67 = ((x9) ? (x0 * x8) : (1));
	const GEN_FLT x68 = x67 * x7;
	const GEN_FLT x69 = x5 + x68;
	const GEN_FLT x70 = x54 * x69;
	const GEN_FLT x71 = x39 * x45;
	const GEN_FLT x72 = x42 * x48;
	const GEN_FLT x73 = x71 + x72;
	const GEN_FLT x74 = x62 * x73;
	const GEN_FLT x75 = -x43 + x49;
	const GEN_FLT x76 = x37 * x75;
	const GEN_FLT x77 = lh_px + x70 + x74 + x76;
	const GEN_FLT x78 = ((x9) ? (x2 * x8) : (0));
	const GEN_FLT x79 = x7 * x78;
	const GEN_FLT x80 = x62 * (x5 + x79);
	const GEN_FLT x81 = x37 * (x56 + x59);
	const GEN_FLT x82 = x54 * (-x71 + x72);
	const GEN_FLT x83 = -lh_pz - x80 - x81 - x82;
	const GEN_FLT x84 = x77 * x77 + x83 * x83;
	const GEN_FLT x85 = x66 / sqrt(x84);
	const GEN_FLT x86 = x64 * x85;
	const GEN_FLT x87 = cos(x65);
	const GEN_FLT x88 = 1.0 / x87;
	const GEN_FLT x89 = x64 * x64;
	const GEN_FLT x90 = x84 + x89;
	const GEN_FLT x91 = x88 / sqrt(x90);
	const GEN_FLT x92 = asin(x64 * x91);
	const GEN_FLT x93 = 8.0108022e-6 * x92;
	const GEN_FLT x94 = -x93 - 8.0108022e-6;
	const GEN_FLT x95 = x92 * x94 + 0.0028679863;
	const GEN_FLT x96 = x92 * x95 + 5.3685255000000001e-6;
	const GEN_FLT x97 = x92 * x96 + 0.0076069798000000001;
	const GEN_FLT x98 = x92 * x92;
	const GEN_FLT x99 = atan2(x83, x77);
	const GEN_FLT x100 = ogeePhase_0 + x99 - asin(x86);
	const GEN_FLT x101 = ogeeMag_0 * sin(x100);
	const GEN_FLT x102 = curve_0 + x101;
	const GEN_FLT x103 = x92 * x97;
	const GEN_FLT x104 = -1.60216044e-5 * x92 - 8.0108022e-6;
	const GEN_FLT x105 = x104 * x92 + x95;
	const GEN_FLT x106 = x105 * x92 + x96;
	const GEN_FLT x107 = x106 * x92 + x97;
	const GEN_FLT x108 = sin(x65);
	const GEN_FLT x109 = x108 * (x103 + x107 * x92);
	const GEN_FLT x110 = -x102 * x109 + x87;
	const GEN_FLT x111 = 1.0 / x110;
	const GEN_FLT x112 = x102 * x111;
	const GEN_FLT x113 = x112 * x98;
	const GEN_FLT x114 = x113 * x97 + x86;
	const GEN_FLT x115 = pow(1 - x114 * x114, -1.0 / 2.0);
	const GEN_FLT x116 = pow(-x89 / (x90 * (x87 * x87)) + 1, -1.0 / 2.0);
	const GEN_FLT x117 = 2 * x43;
	const GEN_FLT x118 = 2 * x49;
	const GEN_FLT x119 = (1.0 / 2.0) * x64;
	const GEN_FLT x120 = 2 * x5;
	const GEN_FLT x121 = (1.0 / 2.0) * x77;
	const GEN_FLT x122 = 2 * x71;
	const GEN_FLT x123 = 2 * x72;
	const GEN_FLT x124 = (1.0 / 2.0) * x83;
	const GEN_FLT x125 = -x121 * (x120 + 2 * x68) - x124 * (x122 - x123);
	const GEN_FLT x126 = x64 * x88 / pow(x90, 3.0 / 2.0);
	const GEN_FLT x127 = x116 * (x126 * (-x119 * (x117 + x118) + x125) + x50 * x91);
	const GEN_FLT x128 = x127 * x94;
	const GEN_FLT x129 = x127 * x95 + x92 * (-x127 * x93 + x128);
	const GEN_FLT x130 = x127 * x96 + x129 * x92;
	const GEN_FLT x131 = 1.0 / x84;
	const GEN_FLT x132 = pow(-x131 * x89 * x66 * x66 + 1, -1.0 / 2.0);
	const GEN_FLT x133 = x64 * x66 / pow(x84, 3.0 / 2.0);
	const GEN_FLT x134 = x125 * x133 + x50 * x85;
	const GEN_FLT x135 = x131 * (lh_pz + x80 + x81 + x82);
	const GEN_FLT x136 = x71 - x72;
	const GEN_FLT x137 = x131 * x77;
	const GEN_FLT x138 = x135 * x69 + x136 * x137;
	const GEN_FLT x139 = -x132 * x134 + x138;
	const GEN_FLT x140 = ogeeMag_0 * cos(x100);
	const GEN_FLT x141 = x109 * x140;
	const GEN_FLT x142 = 2.40324066e-5 * x92;
	const GEN_FLT x143 = x108 * (-curve_0 - x101);
	const GEN_FLT x144 = x97 * x98;
	const GEN_FLT x145 = x102 * x144 / ((x110 * x110));
	const GEN_FLT x146 = x111 * x140 * x144;
	const GEN_FLT x147 = 2 * x103 * x112;
	const GEN_FLT x148 =
		-x115 * (x113 * x130 + x127 * x147 + x134 + x139 * x146 +
				 x145 * (x139 * x141 -
						 x143 * (x107 * x127 + x127 * x97 + x130 * x92 +
								 x92 * (x106 * x127 + x130 +
										x92 * (x105 * x127 + x129 + x92 * (x104 * x127 - x127 * x142 + x128)))))) +
		x138;
	const GEN_FLT x149 = gibMag_0 * cos(gibPhase_0 + x99 - asin(x114));
	const GEN_FLT x150 = -2 * x56;
	const GEN_FLT x151 = 2 * x59;
	const GEN_FLT x152 = -x121 * (-x117 + x118) - x124 * (x150 - x151);
	const GEN_FLT x153 = x116 * (x12 * x91 + x126 * (-x119 * (2 * x11 + x120) + x152));
	const GEN_FLT x154 = x153 * x94;
	const GEN_FLT x155 = x153 * x95 + x92 * (-x153 * x93 + x154);
	const GEN_FLT x156 = x153 * x96 + x155 * x92;
	const GEN_FLT x157 = x12 * x85 + x133 * x152;
	const GEN_FLT x158 = x57 - x59;
	const GEN_FLT x159 = x135 * x75 + x137 * x158;
	const GEN_FLT x160 = -x132 * x157 + x159;
	const GEN_FLT x161 =
		-x115 * (x113 * x156 +
				 x145 * (x141 * x160 -
						 x143 * (x107 * x153 + x153 * x97 + x156 * x92 +
								 x92 * (x106 * x153 + x156 +
										x92 * (x105 * x153 + x155 + x92 * (x104 * x153 - x142 * x153 + x154))))) +
				 x146 * x160 + x147 * x153 + x157) +
		x159;
	const GEN_FLT x162 = -x121 * (x122 + x123) - x124 * (-x120 - 2 * x79);
	const GEN_FLT x163 = x116 * (x126 * (-x119 * (x150 + x151) + x162) + x60 * x91);
	const GEN_FLT x164 = x163 * x94;
	const GEN_FLT x165 = x163 * x95 + x92 * (-x163 * x93 + x164);
	const GEN_FLT x166 = x163 * x96 + x165 * x92;
	const GEN_FLT x167 = x133 * x162 + x60 * x85;
	const GEN_FLT x168 = x6 - x79;
	const GEN_FLT x169 = x135 * x73 + x137 * x168;
	const GEN_FLT x170 = -x132 * x167 + x169;
	const GEN_FLT x171 =
		-x115 * (x113 * x166 +
				 x145 * (x141 * x170 -
						 x143 * (x107 * x163 + x163 * x97 + x166 * x92 +
								 x92 * (x106 * x163 + x166 +
										x92 * (x105 * x163 + x165 + x92 * (x104 * x163 - x142 * x163 + x164))))) +
				 x146 * x170 + x147 * x163 + x167) +
		x169;
	const GEN_FLT x172 = x23 * x30;
	const GEN_FLT x173 = -x172;
	const GEN_FLT x174 = 2 / ((x16 * x16));
	const GEN_FLT x175 = obj_qi * x174;
	const GEN_FLT x176 = pow(x16, -3.0 / 2.0);
	const GEN_FLT x177 = obj_qi * x176;
	const GEN_FLT x178 = ((x21) ? (-obj_qk * x177) : (0));
	const GEN_FLT x179 = x178 * x23;
	const GEN_FLT x180 = x18 * x30;
	const GEN_FLT x181 = x180 * x26;
	const GEN_FLT x182 = ((x21) ? (-obj_qj * x177) : (0));
	const GEN_FLT x183 = ((x21) ? (-x13 * x176 + x24) : (0));
	const GEN_FLT x184 = x30 * x31;
	const GEN_FLT x185 = x182 * x32 + x183 * x35 + x184 * x52;
	const GEN_FLT x186 = x183 * x23;
	const GEN_FLT x187 = x180 * x31;
	const GEN_FLT x188 = x178 * x35;
	const GEN_FLT x189 = x19 * x26;
	const GEN_FLT x190 = x182 * x189;
	const GEN_FLT x191 = x27 * x29;
	const GEN_FLT x192 = x188 + x190 + x191 * x30;
	const GEN_FLT x193 = sensor_x * (x179 + x181 + x185) +
						 sensor_y * (x172 * x22 + x173 + x19 * ((x21) ? (-x14 * x175) : (0))) +
						 sensor_z * (-x186 - x187 + x192);
	const GEN_FLT x194 = x12 * x193;
	const GEN_FLT x195 = x182 * x23;
	const GEN_FLT x196 = -x195;
	const GEN_FLT x197 = x180 * x29;
	const GEN_FLT x198 = x178 * x32 + x183 * x189 + x184 * x27;
	const GEN_FLT x199 = sensor_x * (x196 - x197 + x198) + sensor_y * (x186 + x187 + x192) +
						 sensor_z * (x172 * x61 + x173 + x19 * ((x21) ? (-x15 * x175) : (0)));
	const GEN_FLT x200 = x199 * x60;
	const GEN_FLT x201 = 2 * x20;
	const GEN_FLT x202 = -x179;
	const GEN_FLT x203 =
		sensor_x * (x172 * x51 + x173 + x19 * ((x21) ? (obj_qi * x201 - x174 * obj_qi * (obj_qi * obj_qi)) : (0))) +
		sensor_y * (-x181 + x185 + x202) + sensor_z * (x195 + x197 + x198);
	const GEN_FLT x204 = x203 * x50;
	const GEN_FLT x205 = x194 + x200 + x204;
	const GEN_FLT x206 = x203 * x69;
	const GEN_FLT x207 = x199 * x73;
	const GEN_FLT x208 = x193 * x75;
	const GEN_FLT x209 = x168 * x199;
	const GEN_FLT x210 = x158 * x193;
	const GEN_FLT x211 = x136 * x203;
	const GEN_FLT x212 = -x121 * (2 * x206 + 2 * x207 + 2 * x208) - x124 * (2 * x209 + 2 * x210 + 2 * x211);
	const GEN_FLT x213 = x116 * (x126 * (-x119 * (2 * x194 + 2 * x200 + 2 * x204) + x212) + x205 * x91);
	const GEN_FLT x214 = x213 * x94;
	const GEN_FLT x215 = x213 * x95 + x92 * (-x213 * x93 + x214);
	const GEN_FLT x216 = x213 * x96 + x215 * x92;
	const GEN_FLT x217 = x133 * x212 + x205 * x85;
	const GEN_FLT x218 = x135 * (x206 + x207 + x208) + x137 * (x209 + x210 + x211);
	const GEN_FLT x219 = -x132 * x217 + x218;
	const GEN_FLT x220 =
		-x115 * (x113 * x216 +
				 x145 * (x141 * x219 -
						 x143 * (x107 * x213 + x213 * x97 + x216 * x92 +
								 x92 * (x106 * x213 + x216 +
										x92 * (x105 * x213 + x215 + x92 * (x104 * x213 - x142 * x213 + x214))))) +
				 x146 * x219 + x147 * x213 + x217) +
		x218;
	const GEN_FLT x221 = x23 * x28;
	const GEN_FLT x222 = -x221;
	const GEN_FLT x223 = ((x21) ? (-obj_qj * obj_qk * x176) : (0));
	const GEN_FLT x224 = x223 * x23;
	const GEN_FLT x225 = x18 * x28;
	const GEN_FLT x226 = x225 * x26;
	const GEN_FLT x227 = ((x21) ? (-x14 * x176 + x24) : (0));
	const GEN_FLT x228 = x28 * x31;
	const GEN_FLT x229 = x182 * x35 + x227 * x32 + x228 * x52;
	const GEN_FLT x230 = x225 * x31;
	const GEN_FLT x231 = x189 * x227 + x191 * x28 + x223 * x35;
	const GEN_FLT x232 =
		sensor_x * (x224 + x226 + x229) +
		sensor_y * (x19 * ((x21) ? (obj_qj * x201 - x174 * obj_qj * (obj_qj * obj_qj)) : (0)) + x22 * x221 + x222) +
		sensor_z * (x196 - x230 + x231);
	const GEN_FLT x233 = x12 * x232;
	const GEN_FLT x234 = obj_qj * x174;
	const GEN_FLT x235 = x227 * x23;
	const GEN_FLT x236 = x225 * x29;
	const GEN_FLT x237 = x223 * x32;
	const GEN_FLT x238 = x190 + x228 * x27 + x237;
	const GEN_FLT x239 = -x224;
	const GEN_FLT x240 = sensor_x * (x19 * ((x21) ? (-x13 * x234) : (0)) + x221 * x51 + x222) +
						 sensor_y * (-x226 + x229 + x239) + sensor_z * (x235 + x236 + x238);
	const GEN_FLT x241 = x240 * x50;
	const GEN_FLT x242 = sensor_x * (-x235 - x236 + x238) + sensor_y * (x195 + x230 + x231) +
						 sensor_z * (x19 * ((x21) ? (-x15 * x234) : (0)) + x221 * x61 + x222);
	const GEN_FLT x243 = x242 * x60;
	const GEN_FLT x244 = x233 + x241 + x243;
	const GEN_FLT x245 = x240 * x69;
	const GEN_FLT x246 = x242 * x73;
	const GEN_FLT x247 = x232 * x75;
	const GEN_FLT x248 = x168 * x242;
	const GEN_FLT x249 = x136 * x240;
	const GEN_FLT x250 = x158 * x232;
	const GEN_FLT x251 = -x121 * (2 * x245 + 2 * x246 + 2 * x247) - x124 * (2 * x248 + 2 * x249 + 2 * x250);
	const GEN_FLT x252 = x116 * (x126 * (-x119 * (2 * x233 + 2 * x241 + 2 * x243) + x251) + x244 * x91);
	const GEN_FLT x253 = x252 * x94;
	const GEN_FLT x254 = x252 * x95 + x92 * (-x252 * x93 + x253);
	const GEN_FLT x255 = x252 * x96 + x254 * x92;
	const GEN_FLT x256 = x133 * x251 + x244 * x85;
	const GEN_FLT x257 = x135 * (x245 + x246 + x247) + x137 * (x248 + x249 + x250);
	const GEN_FLT x258 = -x132 * x256 + x257;
	const GEN_FLT x259 =
		-x115 * (x113 * x255 +
				 x145 * (x141 * x258 -
						 x143 * (x107 * x252 + x252 * x97 + x255 * x92 +
								 x92 * (x106 * x252 + x255 +
										x92 * (x105 * x252 + x254 + x92 * (x104 * x252 - x142 * x252 + x253))))) +
				 x146 * x258 + x147 * x252 + x256) +
		x257;
	const GEN_FLT x260 = x23 * x25;
	const GEN_FLT x261 = -x260;
	const GEN_FLT x262 = obj_qk * x174;
	const GEN_FLT x263 = ((x21) ? (-x15 * x176 + x24) : (0));
	const GEN_FLT x264 = x23 * x263;
	const GEN_FLT x265 = x18 * x25;
	const GEN_FLT x266 = x26 * x265;
	const GEN_FLT x267 = x25 * x31;
	const GEN_FLT x268 = x188 + x237 + x267 * x52;
	const GEN_FLT x269 = x265 * x31;
	const GEN_FLT x270 = x189 * x223 + x191 * x25 + x263 * x35;
	const GEN_FLT x271 = sensor_x * (x264 + x266 + x268) +
						 sensor_y * (x19 * ((x21) ? (-x14 * x262) : (0)) + x22 * x260 + x261) +
						 sensor_z * (x202 - x269 + x270);
	const GEN_FLT x272 = x12 * x271;
	const GEN_FLT x273 = x265 * x29;
	const GEN_FLT x274 = x178 * x189 + x263 * x32 + x267 * x27;
	const GEN_FLT x275 = sensor_x * (x19 * ((x21) ? (-x13 * x262) : (0)) + x260 * x51 + x261) +
						 sensor_y * (-x264 - x266 + x268) + sensor_z * (x224 + x273 + x274);
	const GEN_FLT x276 = x275 * x50;
	const GEN_FLT x277 =
		sensor_x * (x239 - x273 + x274) + sensor_y * (x179 + x269 + x270) +
		sensor_z * (x19 * ((x21) ? (obj_qk * x201 - x174 * obj_qk * (obj_qk * obj_qk)) : (0)) + x260 * x61 + x261);
	const GEN_FLT x278 = x277 * x60;
	const GEN_FLT x279 = x272 + x276 + x278;
	const GEN_FLT x280 = x275 * x69;
	const GEN_FLT x281 = x271 * x75;
	const GEN_FLT x282 = x277 * x73;
	const GEN_FLT x283 = x168 * x277;
	const GEN_FLT x284 = x136 * x275;
	const GEN_FLT x285 = x158 * x271;
	const GEN_FLT x286 = -x121 * (2 * x280 + 2 * x281 + 2 * x282) - x124 * (2 * x283 + 2 * x284 + 2 * x285);
	const GEN_FLT x287 = x116 * (x126 * (-x119 * (2 * x272 + 2 * x276 + 2 * x278) + x286) + x279 * x91);
	const GEN_FLT x288 = x287 * x94;
	const GEN_FLT x289 = x287 * x95 + x92 * (-x287 * x93 + x288);
	const GEN_FLT x290 = x287 * x96 + x289 * x92;
	const GEN_FLT x291 = x133 * x286 + x279 * x85;
	const GEN_FLT x292 = x135 * (x280 + x281 + x282) + x137 * (x283 + x284 + x285);
	const GEN_FLT x293 = -x132 * x291 + x292;
	const GEN_FLT x294 =
		-x115 * (x113 * x290 +
				 x145 * (x141 * x293 -
						 x143 * (x107 * x287 + x287 * x97 + x290 * x92 +
								 x92 * (x106 * x287 + x290 +
										x92 * (x105 * x287 + x289 + x92 * (x104 * x287 - x142 * x287 + x288))))) +
				 x146 * x293 + x147 * x287 + x291) +
		x292;
	const GEN_FLT x295 = -lh_px - x70 - x74 - x76;
	const GEN_FLT x296 = x133 * x295;
	const GEN_FLT x297 = x116 * x126;
	const GEN_FLT x298 = x295 * x297;
	const GEN_FLT x299 = -x132 * x296 + x135;
	const GEN_FLT x300 = x298 * x94;
	const GEN_FLT x301 = x298 * x95 + x92 * (-x298 * x93 + x300);
	const GEN_FLT x302 = x298 * x96 + x301 * x92;
	const GEN_FLT x303 =
		-x115 * (x113 * x302 +
				 x145 * (x141 * x299 -
						 x143 * (x107 * x298 + x298 * x97 + x302 * x92 +
								 x92 * (x106 * x298 + x302 +
										x92 * (x105 * x298 + x301 + x92 * (x104 * x298 - x142 * x298 + x300))))) +
				 x146 * x299 + x147 * x298 + x296) +
		x135;
	const GEN_FLT x304 = x132 * x85;
	const GEN_FLT x305 = x116 * (x126 * (-lh_py - x38 - x55 - x63) + x91);
	const GEN_FLT x306 = x305 * x94;
	const GEN_FLT x307 = x305 * x95 + x92 * (-x305 * x93 + x306);
	const GEN_FLT x308 = x305 * x96 + x307 * x92;
	const GEN_FLT x309 =
		x115 * (x113 * x308 +
				x145 * (-x141 * x304 -
						x143 * (x107 * x305 + x305 * x97 + x308 * x92 +
								x92 * (x106 * x305 + x308 +
									   x92 * (x105 * x305 + x307 + x92 * (x104 * x305 - x142 * x305 + x306))))) -
				x146 * x304 + x147 * x305 + x85);
	const GEN_FLT x310 = -x137;
	const GEN_FLT x311 = x133 * x83;
	const GEN_FLT x312 = x297 * x83;
	const GEN_FLT x313 = -x132 * x311 + x310;
	const GEN_FLT x314 = x312 * x94;
	const GEN_FLT x315 = x312 * x95 + x92 * (-x312 * x93 + x314);
	const GEN_FLT x316 = x312 * x96 + x315 * x92;
	const GEN_FLT x317 =
		-x115 * (x113 * x316 +
				 x145 * (x141 * x313 -
						 x143 * (x107 * x312 + x312 * x97 + x316 * x92 +
								 x92 * (x106 * x312 + x316 +
										x92 * (x105 * x312 + x315 + x92 * (x104 * x312 - x142 * x312 + x314))))) +
				 x146 * x313 + x147 * x312 + x311) +
		x310;
	const GEN_FLT x318 = x39 * x46;
	const GEN_FLT x319 = -x318;
	const GEN_FLT x320 = 2 / ((x3 * x3));
	const GEN_FLT x321 = lh_qi * x320;
	const GEN_FLT x322 = x37 * (x10 * x318 + x319 + x7 * ((x9) ? (-x1 * x321) : (0)));
	const GEN_FLT x323 = pow(x3, -3.0 / 2.0);
	const GEN_FLT x324 = lh_qi * x323;
	const GEN_FLT x325 = ((x9) ? (-lh_qk * x324) : (0));
	const GEN_FLT x326 = x325 * x39;
	const GEN_FLT x327 = x46 * x5;
	const GEN_FLT x328 = x327 * x42;
	const GEN_FLT x329 = ((x9) ? (-lh_qj * x324) : (0));
	const GEN_FLT x330 = ((x9) ? (-x0 * x323 + x40) : (0));
	const GEN_FLT x331 = x46 * x71;
	const GEN_FLT x332 = x329 * x48 + x330 * x58 + x331 * x47;
	const GEN_FLT x333 = x54 * (x326 + x328 + x332);
	const GEN_FLT x334 = x325 * x58;
	const GEN_FLT x335 = x42 * x7;
	const GEN_FLT x336 = x329 * x335;
	const GEN_FLT x337 = x331 * x42;
	const GEN_FLT x338 = -x327 * x47 - x330 * x39;
	const GEN_FLT x339 = x62 * (x334 + x336 + x337 + x338);
	const GEN_FLT x340 = x322 + x333 + x339;
	const GEN_FLT x341 = x62 * (-x318 * x78 + x318 - x7 * ((x9) ? (-x2 * x321) : (0)));
	const GEN_FLT x342 = x325 * x48;
	const GEN_FLT x343 = x330 * x335;
	const GEN_FLT x344 = x43 * x47;
	const GEN_FLT x345 = x344 * x46;
	const GEN_FLT x346 = x329 * x39;
	const GEN_FLT x347 = x327 * x45 + x346;
	const GEN_FLT x348 = x54 * (-x342 - x343 - x345 + x347);
	const GEN_FLT x349 = -x336;
	const GEN_FLT x350 = x37 * (-x334 - x337 + x338 + x349);
	const GEN_FLT x351 = 2 * x8;
	const GEN_FLT x352 =
		x54 * (x318 * x67 + x319 + x7 * ((x9) ? (lh_qi * x351 - x320 * lh_qi * (lh_qi * lh_qi)) : (0)));
	const GEN_FLT x353 = x62 * (x342 + x343 + x345 + x347);
	const GEN_FLT x354 = -x326;
	const GEN_FLT x355 = x37 * (-x328 + x332 + x354);
	const GEN_FLT x356 = -x121 * (2 * x352 + 2 * x353 + 2 * x355) - x124 * (2 * x341 + 2 * x348 + 2 * x350);
	const GEN_FLT x357 = x116 * (x126 * (-x119 * (2 * x322 + 2 * x333 + 2 * x339) + x356) + x340 * x91);
	const GEN_FLT x358 = x357 * x94;
	const GEN_FLT x359 = x357 * x95 + x92 * (-x357 * x93 + x358);
	const GEN_FLT x360 = x357 * x96 + x359 * x92;
	const GEN_FLT x361 = x133 * x356 + x340 * x85;
	const GEN_FLT x362 = x135 * (x352 + x353 + x355) + x137 * (x341 + x348 + x350);
	const GEN_FLT x363 = -x132 * x361 + x362;
	const GEN_FLT x364 =
		-x115 * (x113 * x360 +
				 x145 * (x141 * x363 -
						 x143 * (x107 * x357 + x357 * x97 + x360 * x92 +
								 x92 * (x106 * x357 + x360 +
										x92 * (x105 * x357 + x359 + x92 * (x104 * x357 - x142 * x357 + x358))))) +
				 x146 * x363 + x147 * x357 + x361) +
		x362;
	const GEN_FLT x365 = x39 * x44;
	const GEN_FLT x366 = -x365;
	const GEN_FLT x367 =
		x37 * (x10 * x365 + x366 + x7 * ((x9) ? (lh_qj * x351 - x320 * lh_qj * (lh_qj * lh_qj)) : (0)));
	const GEN_FLT x368 = ((x9) ? (-lh_qj * lh_qk * x323) : (0));
	const GEN_FLT x369 = x368 * x39;
	const GEN_FLT x370 = x44 * x5;
	const GEN_FLT x371 = x370 * x42;
	const GEN_FLT x372 = ((x9) ? (-x1 * x323 + x40) : (0));
	const GEN_FLT x373 = x44 * x71;
	const GEN_FLT x374 = x329 * x58 + x372 * x48 + x373 * x47;
	const GEN_FLT x375 = x54 * (x369 + x371 + x374);
	const GEN_FLT x376 = x368 * x58;
	const GEN_FLT x377 = x335 * x372;
	const GEN_FLT x378 = x373 * x42;
	const GEN_FLT x379 = -x346 - x370 * x47;
	const GEN_FLT x380 = x62 * (x376 + x377 + x378 + x379);
	const GEN_FLT x381 = x367 + x375 + x380;
	const GEN_FLT x382 = lh_qj * x320;
	const GEN_FLT x383 = x54 * (x365 * x67 + x366 + x7 * ((x9) ? (-x0 * x382) : (0)));
	const GEN_FLT x384 = x368 * x48;
	const GEN_FLT x385 = x344 * x44;
	const GEN_FLT x386 = x370 * x45 + x372 * x39;
	const GEN_FLT x387 = x62 * (x336 + x384 + x385 + x386);
	const GEN_FLT x388 = x37 * (-x369 - x371 + x374);
	const GEN_FLT x389 = x62 * (-x365 * x78 + x365 - x7 * ((x9) ? (-x2 * x382) : (0)));
	const GEN_FLT x390 = x54 * (x349 - x384 - x385 + x386);
	const GEN_FLT x391 = x37 * (-x376 - x377 - x378 + x379);
	const GEN_FLT x392 = -x121 * (2 * x383 + 2 * x387 + 2 * x388) - x124 * (2 * x389 + 2 * x390 + 2 * x391);
	const GEN_FLT x393 = x116 * (x126 * (-x119 * (2 * x367 + 2 * x375 + 2 * x380) + x392) + x381 * x91);
	const GEN_FLT x394 = x393 * x94;
	const GEN_FLT x395 = x393 * x95 + x92 * (-x393 * x93 + x394);
	const GEN_FLT x396 = x393 * x96 + x395 * x92;
	const GEN_FLT x397 = x133 * x392 + x381 * x85;
	const GEN_FLT x398 = x135 * (x383 + x387 + x388) + x137 * (x389 + x390 + x391);
	const GEN_FLT x399 = -x132 * x397 + x398;
	const GEN_FLT x400 =
		-x115 * (x113 * x396 +
				 x145 * (x141 * x399 -
						 x143 * (x107 * x393 + x393 * x97 + x396 * x92 +
								 x92 * (x106 * x393 + x396 +
										x92 * (x105 * x393 + x395 + x92 * (x104 * x393 - x142 * x393 + x394))))) +
				 x146 * x399 + x147 * x393 + x397) +
		x398;
	const GEN_FLT x401 = x39 * x41;
	const GEN_FLT x402 = -x401;
	const GEN_FLT x403 = lh_qk * x320;
	const GEN_FLT x404 = x37 * (x10 * x401 + x402 + x7 * ((x9) ? (-x1 * x403) : (0)));
	const GEN_FLT x405 = ((x9) ? (-x2 * x323 + x40) : (0));
	const GEN_FLT x406 = x39 * x405;
	const GEN_FLT x407 = x41 * x5;
	const GEN_FLT x408 = x407 * x42;
	const GEN_FLT x409 = x41 * x71;
	const GEN_FLT x410 = x334 + x384 + x409 * x47;
	const GEN_FLT x411 = x54 * (x406 + x408 + x410);
	const GEN_FLT x412 = x405 * x58;
	const GEN_FLT x413 = x335 * x368;
	const GEN_FLT x414 = x409 * x42;
	const GEN_FLT x415 = x354 - x407 * x47;
	const GEN_FLT x416 = x62 * (x412 + x413 + x414 + x415);
	const GEN_FLT x417 = x404 + x411 + x416;
	const GEN_FLT x418 = x54 * (x401 * x67 + x402 + x7 * ((x9) ? (-x0 * x403) : (0)));
	const GEN_FLT x419 = x405 * x48;
	const GEN_FLT x420 = x325 * x335;
	const GEN_FLT x421 = x344 * x41;
	const GEN_FLT x422 = x369 + x407 * x45;
	const GEN_FLT x423 = x62 * (x419 + x420 + x421 + x422);
	const GEN_FLT x424 = x37 * (-x406 - x408 + x410);
	const GEN_FLT x425 =
		x62 * (-x401 * x78 + x401 - x7 * ((x9) ? (lh_qk * x351 - x320 * lh_qk * (lh_qk * lh_qk)) : (0)));
	const GEN_FLT x426 = x54 * (-x419 - x420 - x421 + x422);
	const GEN_FLT x427 = x37 * (-x412 - x413 - x414 + x415);
	const GEN_FLT x428 = -x121 * (2 * x418 + 2 * x423 + 2 * x424) - x124 * (2 * x425 + 2 * x426 + 2 * x427);
	const GEN_FLT x429 = x116 * (x126 * (-x119 * (2 * x404 + 2 * x411 + 2 * x416) + x428) + x417 * x91);
	const GEN_FLT x430 = x429 * x94;
	const GEN_FLT x431 = x429 * x95 + x92 * (-x429 * x93 + x430);
	const GEN_FLT x432 = x429 * x96 + x431 * x92;
	const GEN_FLT x433 = x133 * x428 + x417 * x85;
	const GEN_FLT x434 = x135 * (x418 + x423 + x424) + x137 * (x425 + x426 + x427);
	const GEN_FLT x435 = -x132 * x433 + x434;
	const GEN_FLT x436 =
		-x115 * (x113 * x432 +
				 x145 * (x141 * x435 -
						 x143 * (x107 * x429 + x429 * x97 + x432 * x92 +
								 x92 * (x106 * x429 + x432 +
										x92 * (x105 * x429 + x431 + x92 * (x104 * x429 - x142 * x429 + x430))))) +
				 x146 * x435 + x147 * x429 + x433) +
		x434;
	*(out++) = x148 * x149 + x148;
	*(out++) = x149 * x161 + x161;
	*(out++) = x149 * x171 + x171;
	*(out++) = x149 * x220 + x220;
	*(out++) = x149 * x259 + x259;
	*(out++) = x149 * x294 + x294;
	*(out++) = x149 * x303 + x303;
	*(out++) = -x149 * x309 - x309;
	*(out++) = x149 * x317 + x317;
	*(out++) = x149 * x364 + x364;
	*(out++) = x149 * x400 + x400;
	*(out++) = x149 * x436 + x436;
}

static inline void gen_reproject_axisangle_axis_y_jac_all_gen2(FLT *out, const FLT *obj, const FLT *sensor,
															   const FLT *lh, const FLT phase_0, const FLT tilt_0,
															   const FLT curve_0, const FLT gibPhase_0,
															   const FLT gibMag_0, const FLT ogeePhase_0,
															   const FLT ogeeMag_0) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = -x5;
	const GEN_FLT x7 = x6 + 1;
	const GEN_FLT x8 = 1.0 / x3;
	const GEN_FLT x9 = x4 > 0;
	const GEN_FLT x10 = ((x9) ? (x1 * x8) : (0));
	const GEN_FLT x11 = x10 * x7;
	const GEN_FLT x12 = x11 + x5;
	const GEN_FLT x13 = obj_qi * obj_qi;
	const GEN_FLT x14 = obj_qj * obj_qj;
	const GEN_FLT x15 = obj_qk * obj_qk;
	const GEN_FLT x16 = x13 + x14 + x15;
	const GEN_FLT x17 = sqrt(x16);
	const GEN_FLT x18 = cos(x17);
	const GEN_FLT x19 = 1 - x18;
	const GEN_FLT x20 = 1.0 / x16;
	const GEN_FLT x21 = x17 > 0;
	const GEN_FLT x22 = ((x21) ? (x14 * x20) : (0));
	const GEN_FLT x23 = sin(x17);
	const GEN_FLT x24 = 1.0 / x17;
	const GEN_FLT x25 = obj_qk * x24;
	const GEN_FLT x26 = ((x21) ? (x25) : (0));
	const GEN_FLT x27 = x23 * x26;
	const GEN_FLT x28 = obj_qj * x24;
	const GEN_FLT x29 = ((x21) ? (x28) : (0));
	const GEN_FLT x30 = obj_qi * x24;
	const GEN_FLT x31 = ((x21) ? (x30) : (1));
	const GEN_FLT x32 = x19 * x31;
	const GEN_FLT x33 = x29 * x32;
	const GEN_FLT x34 = x23 * x31;
	const GEN_FLT x35 = x19 * x29;
	const GEN_FLT x36 = x26 * x35;
	const GEN_FLT x37 = obj_py + sensor_x * (x27 + x33) + sensor_y * (x18 + x19 * x22) + sensor_z * (-x34 + x36);
	const GEN_FLT x38 = x12 * x37;
	const GEN_FLT x39 = sin(x4);
	const GEN_FLT x40 = 1.0 / x4;
	const GEN_FLT x41 = lh_qk * x40;
	const GEN_FLT x42 = ((x9) ? (x41) : (0));
	const GEN_FLT x43 = x39 * x42;
	const GEN_FLT x44 = lh_qj * x40;
	const GEN_FLT x45 = ((x9) ? (x44) : (0));
	const GEN_FLT x46 = lh_qi * x40;
	const GEN_FLT x47 = ((x9) ? (x46) : (1));
	const GEN_FLT x48 = x47 * x7;
	const GEN_FLT x49 = x45 * x48;
	const GEN_FLT x50 = x43 + x49;
	const GEN_FLT x51 = ((x21) ? (x13 * x20) : (1));
	const GEN_FLT x52 = x23 * x29;
	const GEN_FLT x53 = x26 * x32;
	const GEN_FLT x54 = obj_px + sensor_x * (x18 + x19 * x51) + sensor_y * (-x27 + x33) + sensor_z * (x52 + x53);
	const GEN_FLT x55 = x50 * x54;
	const GEN_FLT x56 = x39 * x47;
	const GEN_FLT x57 = -x56;
	const GEN_FLT x58 = x45 * x7;
	const GEN_FLT x59 = x42 * x58;
	const GEN_FLT x60 = x57 + x59;
	const GEN_FLT x61 = ((x21) ? (x15 * x20) : (0));
	const GEN_FLT x62 = obj_pz + sensor_x * (-x52 + x53) + sensor_y * (x34 + x36) + sensor_z * (x18 + x19 * x61);
	const GEN_FLT x63 = x60 * x62;
	const GEN_FLT x64 = lh_py + x38 + x55 + x63;
	const GEN_FLT x65 = tilt_0 - 0.52359877559829882;
	const GEN_FLT x66 = tan(x65);
	const GEN_FLT x67 = ((x9) ? (x0 * x8) : (1));
	const GEN_FLT x68 = x67 * x7;
	const GEN_FLT x69 = x5 + x68;
	const GEN_FLT x70 = x54 * x69;
	const GEN_FLT x71 = x39 * x45;
	const GEN_FLT x72 = x42 * x48;
	const GEN_FLT x73 = x71 + x72;
	const GEN_FLT x74 = x62 * x73;
	const GEN_FLT x75 = -x43 + x49;
	const GEN_FLT x76 = x37 * x75;
	const GEN_FLT x77 = lh_px + x70 + x74 + x76;
	const GEN_FLT x78 = ((x9) ? (x2 * x8) : (0));
	const GEN_FLT x79 = x7 * x78;
	const GEN_FLT x80 = x62 * (x5 + x79);
	const GEN_FLT x81 = x37 * (x56 + x59);
	const GEN_FLT x82 = x54 * (-x71 + x72);
	const GEN_FLT x83 = -lh_pz - x80 - x81 - x82;
	const GEN_FLT x84 = x77 * x77 + x83 * x83;
	const GEN_FLT x85 = x66 / sqrt(x84);
	const GEN_FLT x86 = x64 * x85;
	const GEN_FLT x87 = cos(x65);
	const GEN_FLT x88 = 1.0 / x87;
	const GEN_FLT x89 = x64 * x64;
	const GEN_FLT x90 = x84 + x89;
	const GEN_FLT x91 = x88 / sqrt(x90);
	const GEN_FLT x92 = asin(x64 * x91);
	const GEN_FLT x93 = 8.0108022e-6 * x92;
	const GEN_FLT x94 = -x93 - 8.0108022e-6;
	const GEN_FLT x95 = x92 * x94 + 0.0028679863;
	const GEN_FLT x96 = x92 * x95 + 5.3685255000000001e-6;
	const GEN_FLT x97 = x92 * x96 + 0.0076069798000000001;
	const GEN_FLT x98 = x92 * x92;
	const GEN_FLT x99 = atan2(x83, x77);
	const GEN_FLT x100 = ogeePhase_0 + x99 - asin(x86);
	const GEN_FLT x101 = ogeeMag_0 * sin(x100);
	const GEN_FLT x102 = curve_0 + x101;
	const GEN_FLT x103 = x92 * x97;
	const GEN_FLT x104 = -1.60216044e-5 * x92 - 8.0108022e-6;
	const GEN_FLT x105 = x104 * x92 + x95;
	const GEN_FLT x106 = x105 * x92 + x96;
	const GEN_FLT x107 = x106 * x92 + x97;
	const GEN_FLT x108 = sin(x65);
	const GEN_FLT x109 = x108 * (x103 + x107 * x92);
	const GEN_FLT x110 = -x102 * x109 + x87;
	const GEN_FLT x111 = 1.0 / x110;
	const GEN_FLT x112 = x102 * x111;
	const GEN_FLT x113 = x112 * x98;
	const GEN_FLT x114 = x113 * x97 + x86;
	const GEN_FLT x115 = pow(1 - x114 * x114, -1.0 / 2.0);
	const GEN_FLT x116 = pow(-x89 / (x90 * (x87 * x87)) + 1, -1.0 / 2.0);
	const GEN_FLT x117 = 2 * x43;
	const GEN_FLT x118 = 2 * x49;
	const GEN_FLT x119 = (1.0 / 2.0) * x64;
	const GEN_FLT x120 = 2 * x5;
	const GEN_FLT x121 = (1.0 / 2.0) * x77;
	const GEN_FLT x122 = 2 * x71;
	const GEN_FLT x123 = 2 * x72;
	const GEN_FLT x124 = (1.0 / 2.0) * x83;
	const GEN_FLT x125 = -x121 * (x120 + 2 * x68) - x124 * (x122 - x123);
	const GEN_FLT x126 = x64 * x88 / pow(x90, 3.0 / 2.0);
	const GEN_FLT x127 = x116 * (x126 * (-x119 * (x117 + x118) + x125) + x50 * x91);
	const GEN_FLT x128 = x127 * x94;
	const GEN_FLT x129 = x127 * x95 + x92 * (-x127 * x93 + x128);
	const GEN_FLT x130 = x127 * x96 + x129 * x92;
	const GEN_FLT x131 = 1.0 / x84;
	const GEN_FLT x132 = pow(-x131 * x89 * x66 * x66 + 1, -1.0 / 2.0);
	const GEN_FLT x133 = x64 * x66 / pow(x84, 3.0 / 2.0);
	const GEN_FLT x134 = x125 * x133 + x50 * x85;
	const GEN_FLT x135 = x131 * (lh_pz + x80 + x81 + x82);
	const GEN_FLT x136 = x71 - x72;
	const GEN_FLT x137 = x131 * x77;
	const GEN_FLT x138 = x135 * x69 + x136 * x137;
	const GEN_FLT x139 = -x132 * x134 + x138;
	const GEN_FLT x140 = ogeeMag_0 * cos(x100);
	const GEN_FLT x141 = x109 * x140;
	const GEN_FLT x142 = 2.40324066e-5 * x92;
	const GEN_FLT x143 = x108 * (-curve_0 - x101);
	const GEN_FLT x144 = x97 * x98;
	const GEN_FLT x145 = x102 * x144 / ((x110 * x110));
	const GEN_FLT x146 = x111 * x140 * x144;
	const GEN_FLT x147 = 2 * x103 * x112;
	const GEN_FLT x148 =
		-x115 * (x113 * x130 + x127 * x147 + x134 + x139 * x146 +
				 x145 * (x139 * x141 -
						 x143 * (x107 * x127 + x127 * x97 + x130 * x92 +
								 x92 * (x106 * x127 + x130 +
										x92 * (x105 * x127 + x129 + x92 * (x104 * x127 - x127 * x142 + x128)))))) +
		x138;
	const GEN_FLT x149 = gibMag_0 * cos(gibPhase_0 + x99 - asin(x114));
	const GEN_FLT x150 = -2 * x56;
	const GEN_FLT x151 = 2 * x59;
	const GEN_FLT x152 = -x121 * (-x117 + x118) - x124 * (x150 - x151);
	const GEN_FLT x153 = x116 * (x12 * x91 + x126 * (-x119 * (2 * x11 + x120) + x152));
	const GEN_FLT x154 = x153 * x94;
	const GEN_FLT x155 = x153 * x95 + x92 * (-x153 * x93 + x154);
	const GEN_FLT x156 = x153 * x96 + x155 * x92;
	const GEN_FLT x157 = x12 * x85 + x133 * x152;
	const GEN_FLT x158 = x57 - x59;
	const GEN_FLT x159 = x135 * x75 + x137 * x158;
	const GEN_FLT x160 = -x132 * x157 + x159;
	const GEN_FLT x161 =
		-x115 * (x113 * x156 +
				 x145 * (x141 * x160 -
						 x143 * (x107 * x153 + x153 * x97 + x156 * x92 +
								 x92 * (x106 * x153 + x156 +
										x92 * (x105 * x153 + x155 + x92 * (x104 * x153 - x142 * x153 + x154))))) +
				 x146 * x160 + x147 * x153 + x157) +
		x159;
	const GEN_FLT x162 = -x121 * (x122 + x123) - x124 * (-x120 - 2 * x79);
	const GEN_FLT x163 = x116 * (x126 * (-x119 * (x150 + x151) + x162) + x60 * x91);
	const GEN_FLT x164 = x163 * x94;
	const GEN_FLT x165 = x163 * x95 + x92 * (-x163 * x93 + x164);
	const GEN_FLT x166 = x163 * x96 + x165 * x92;
	const GEN_FLT x167 = x133 * x162 + x60 * x85;
	const GEN_FLT x168 = x6 - x79;
	const GEN_FLT x169 = x135 * x73 + x137 * x168;
	const GEN_FLT x170 = -x132 * x167 + x169;
	const GEN_FLT x171 =
		-x115 * (x113 * x166 +
				 x145 * (x141 * x170 -
						 x143 * (x107 * x163 + x163 * x97 + x166 * x92 +
								 x92 * (x106 * x163 + x166 +
										x92 * (x105 * x163 + x165 + x92 * (x104 * x163 - x142 * x163 + x164))))) +
				 x146 * x170 + x147 * x163 + x167) +
		x169;
	const GEN_FLT x172 = x23 * x30;
	const GEN_FLT x173 = -x172;
	const GEN_FLT x174 = 2 / ((x16 * x16));
	const GEN_FLT x175 = obj_qi * x174;
	const GEN_FLT x176 = pow(x16, -3.0 / 2.0);
	const GEN_FLT x177 = obj_qi * x176;
	const GEN_FLT x178 = ((x21) ? (-obj_qk * x177) : (0));
	const GEN_FLT x179 = x178 * x23;
	const GEN_FLT x180 = x18 * x30;
	const GEN_FLT x181 = x180 * x26;
	const GEN_FLT x182 = ((x21) ? (-obj_qj * x177) : (0));
	const GEN_FLT x183 = ((x21) ? (-x13 * x176 + x24) : (0));
	const GEN_FLT x184 = x30 * x31;
	const GEN_FLT x185 = x182 * x32 + x183 * x35 + x184 * x52;
	const GEN_FLT x186 = x183 * x23;
	const GEN_FLT x187 = x180 * x31;
	const GEN_FLT x188 = x178 * x35;
	const GEN_FLT x189 = x19 * x26;
	const GEN_FLT x190 = x182 * x189;
	const GEN_FLT x191 = x27 * x29;
	const GEN_FLT x192 = x188 + x190 + x191 * x30;
	const GEN_FLT x193 = sensor_x * (x179 + x181 + x185) +
						 sensor_y * (x172 * x22 + x173 + x19 * ((x21) ? (-x14 * x175) : (0))) +
						 sensor_z * (-x186 - x187 + x192);
	const GEN_FLT x194 = x12 * x193;
	const GEN_FLT x195 = x182 * x23;
	const GEN_FLT x196 = -x195;
	const GEN_FLT x197 = x180 * x29;
	const GEN_FLT x198 = x178 * x32 + x183 * x189 + x184 * x27;
	const GEN_FLT x199 = sensor_x * (x196 - x197 + x198) + sensor_y * (x186 + x187 + x192) +
						 sensor_z * (x172 * x61 + x173 + x19 * ((x21) ? (-x15 * x175) : (0)));
	const GEN_FLT x200 = x199 * x60;
	const GEN_FLT x201 = 2 * x20;
	const GEN_FLT x202 = -x179;
	const GEN_FLT x203 =
		sensor_x * (x172 * x51 + x173 + x19 * ((x21) ? (obj_qi * x201 - x174 * obj_qi * (obj_qi * obj_qi)) : (0))) +
		sensor_y * (-x181 + x185 + x202) + sensor_z * (x195 + x197 + x198);
	const GEN_FLT x204 = x203 * x50;
	const GEN_FLT x205 = x194 + x200 + x204;
	const GEN_FLT x206 = x203 * x69;
	const GEN_FLT x207 = x199 * x73;
	const GEN_FLT x208 = x193 * x75;
	const GEN_FLT x209 = x168 * x199;
	const GEN_FLT x210 = x158 * x193;
	const GEN_FLT x211 = x136 * x203;
	const GEN_FLT x212 = -x121 * (2 * x206 + 2 * x207 + 2 * x208) - x124 * (2 * x209 + 2 * x210 + 2 * x211);
	const GEN_FLT x213 = x116 * (x126 * (-x119 * (2 * x194 + 2 * x200 + 2 * x204) + x212) + x205 * x91);
	const GEN_FLT x214 = x213 * x94;
	const GEN_FLT x215 = x213 * x95 + x92 * (-x213 * x93 + x214);
	const GEN_FLT x216 = x213 * x96 + x215 * x92;
	const GEN_FLT x217 = x133 * x212 + x205 * x85;
	const GEN_FLT x218 = x135 * (x206 + x207 + x208) + x137 * (x209 + x210 + x211);
	const GEN_FLT x219 = -x132 * x217 + x218;
	const GEN_FLT x220 =
		-x115 * (x113 * x216 +
				 x145 * (x141 * x219 -
						 x143 * (x107 * x213 + x213 * x97 + x216 * x92 +
								 x92 * (x106 * x213 + x216 +
										x92 * (x105 * x213 + x215 + x92 * (x104 * x213 - x142 * x213 + x214))))) +
				 x146 * x219 + x147 * x213 + x217) +
		x218;
	const GEN_FLT x221 = x23 * x28;
	const GEN_FLT x222 = -x221;
	const GEN_FLT x223 = ((x21) ? (-obj_qj * obj_qk * x176) : (0));
	const GEN_FLT x224 = x223 * x23;
	const GEN_FLT x225 = x18 * x28;
	const GEN_FLT x226 = x225 * x26;
	const GEN_FLT x227 = ((x21) ? (-x14 * x176 + x24) : (0));
	const GEN_FLT x228 = x28 * x31;
	const GEN_FLT x229 = x182 * x35 + x227 * x32 + x228 * x52;
	const GEN_FLT x230 = x225 * x31;
	const GEN_FLT x231 = x189 * x227 + x191 * x28 + x223 * x35;
	const GEN_FLT x232 =
		sensor_x * (x224 + x226 + x229) +
		sensor_y * (x19 * ((x21) ? (obj_qj * x201 - x174 * obj_qj * (obj_qj * obj_qj)) : (0)) + x22 * x221 + x222) +
		sensor_z * (x196 - x230 + x231);
	const GEN_FLT x233 = x12 * x232;
	const GEN_FLT x234 = obj_qj * x174;
	const GEN_FLT x235 = x227 * x23;
	const GEN_FLT x236 = x225 * x29;
	const GEN_FLT x237 = x223 * x32;
	const GEN_FLT x238 = x190 + x228 * x27 + x237;
	const GEN_FLT x239 = -x224;
	const GEN_FLT x240 = sensor_x * (x19 * ((x21) ? (-x13 * x234) : (0)) + x221 * x51 + x222) +
						 sensor_y * (-x226 + x229 + x239) + sensor_z * (x235 + x236 + x238);
	const GEN_FLT x241 = x240 * x50;
	const GEN_FLT x242 = sensor_x * (-x235 - x236 + x238) + sensor_y * (x195 + x230 + x231) +
						 sensor_z * (x19 * ((x21) ? (-x15 * x234) : (0)) + x221 * x61 + x222);
	const GEN_FLT x243 = x242 * x60;
	const GEN_FLT x244 = x233 + x241 + x243;
	const GEN_FLT x245 = x240 * x69;
	const GEN_FLT x246 = x242 * x73;
	const GEN_FLT x247 = x232 * x75;
	const GEN_FLT x248 = x168 * x242;
	const GEN_FLT x249 = x136 * x240;
	const GEN_FLT x250 = x158 * x232;
	const GEN_FLT x251 = -x121 * (2 * x245 + 2 * x246 + 2 * x247) - x124 * (2 * x248 + 2 * x249 + 2 * x250);
	const GEN_FLT x252 = x116 * (x126 * (-x119 * (2 * x233 + 2 * x241 + 2 * x243) + x251) + x244 * x91);
	const GEN_FLT x253 = x252 * x94;
	const GEN_FLT x254 = x252 * x95 + x92 * (-x252 * x93 + x253);
	const GEN_FLT x255 = x252 * x96 + x254 * x92;
	const GEN_FLT x256 = x133 * x251 + x244 * x85;
	const GEN_FLT x257 = x135 * (x245 + x246 + x247) + x137 * (x248 + x249 + x250);
	const GEN_FLT x258 = -x132 * x256 + x257;
	const GEN_FLT x259 =
		-x115 * (x113 * x255 +
				 x145 * (x141 * x258 -
						 x143 * (x107 * x252 + x252 * x97 + x255 * x92 +
								 x92 * (x106 * x252 + x255 +
										x92 * (x105 * x252 + x254 + x92 * (x104 * x252 - x142 * x252 + x253))))) +
				 x146 * x258 + x147 * x252 + x256) +
		x257;
	const GEN_FLT x260 = x23 * x25;
	const GEN_FLT x261 = -x260;
	const GEN_FLT x262 = obj_qk * x174;
	const GEN_FLT x263 = ((x21) ? (-x15 * x176 + x24) : (0));
	const GEN_FLT x264 = x23 * x263;
	const GEN_FLT x265 = x18 * x25;
	const GEN_FLT x266 = x26 * x265;
	const GEN_FLT x267 = x25 * x31;
	const GEN_FLT x268 = x188 + x237 + x267 * x52;
	const GEN_FLT x269 = x265 * x31;
	const GEN_FLT x270 = x189 * x223 + x191 * x25 + x263 * x35;
	const GEN_FLT x271 = sensor_x * (x264 + x266 + x268) +
						 sensor_y * (x19 * ((x21) ? (-x14 * x262) : (0)) + x22 * x260 + x261) +
						 sensor_z * (x202 - x269 + x270);
	const GEN_FLT x272 = x12 * x271;
	const GEN_FLT x273 = x265 * x29;
	const GEN_FLT x274 = x178 * x189 + x263 * x32 + x267 * x27;
	const GEN_FLT x275 = sensor_x * (x19 * ((x21) ? (-x13 * x262) : (0)) + x260 * x51 + x261) +
						 sensor_y * (-x264 - x266 + x268) + sensor_z * (x224 + x273 + x274);
	const GEN_FLT x276 = x275 * x50;
	const GEN_FLT x277 =
		sensor_x * (x239 - x273 + x274) + sensor_y * (x179 + x269 + x270) +
		sensor_z * (x19 * ((x21) ? (obj_qk * x201 - x174 * obj_qk * (obj_qk * obj_qk)) : (0)) + x260 * x61 + x261);
	const GEN_FLT x278 = x277 * x60;
	const GEN_FLT x279 = x272 + x276 + x278;
	const GEN_FLT x280 = x275 * x69;
	const GEN_FLT x281 = x271 * x75;
	const GEN_FLT x282 = x277 * x73;
	const GEN_FLT x283 = x168 * x277;
	const GEN_FLT x284 = x136 * x275;
	const GEN_FLT x285 = x158 * x271;
	const GEN_FLT x286 = -x121 * (2 * x280 + 2 * x281 + 2 * x282) - x124 * (2 * x283 + 2 * x284 + 2 * x285);
	const GEN_FLT x287 = x116 * (x126 * (-x119 * (2 * x272 + 2 * x276 + 2 * x278) + x286) + x279 * x91);
	const GEN_FLT x288 = x287 * x94;
	const GEN_FLT x289 = x287 * x95 + x92 * (-x287 * x93 + x288);
	const GEN_FLT x290 = x287 * x96 + x289 * x92;
	const GEN_FLT x291 = x133 * x286 + x279 * x85;
	const GEN_FLT x292 = x135 * (x280 + x281 + x282) + x137 * (x283 + x284 + x285);
	const GEN_FLT x293 = -x132 * x291 + x292;
	const GEN_FLT x294 =
		-x115 * (x113 * x290 +
				 x145 * (x141 * x293 -
						 x143 * (x107 * x287 + x287 * x97 + x290 * x92 +
								 x92 * (x106 * x287 + x290 +
										x92 * (x105 * x287 + x289 + x92 * (x104 * x287 - x142 * x287 + x288))))) +
				 x146 * x293 + x147 * x287 + x291) +
		x292;
	const GEN_FLT x295 = -lh_px - x70 - x74 - x76;
	const GEN_FLT x296 = x133 * x295;
	const GEN_FLT x297 = x116 * x126;
	const GEN_FLT x298 = x295 * x297;
	const GEN_FLT x299 = -x132 * x296 + x135;
	const GEN_FLT x300 = x298 * x94;
	const GEN_FLT x301 = x298 * x95 + x92 * (-x298 * x93 + x300);
	const GEN_FLT x302 = x298 * x96 + x301 * x92;
	const GEN_FLT x303 =
		-x115 * (x113 * x302 +
				 x145 * (x141 * x299 -
						 x143 * (x107 * x298 + x298 * x97 + x302 * x92 +
								 x92 * (x106 * x298 + x302 +
										x92 * (x105 * x298 + x301 + x92 * (x104 * x298 - x142 * x298 + x300))))) +
				 x146 * x299 + x147 * x298 + x296) +
		x135;
	const GEN_FLT x304 = x132 * x85;
	const GEN_FLT x305 = x116 * (x126 * (-lh_py - x38 - x55 - x63) + x91);
	const GEN_FLT x306 = x305 * x94;
	const GEN_FLT x307 = x305 * x95 + x92 * (-x305 * x93 + x306);
	const GEN_FLT x308 = x305 * x96 + x307 * x92;
	const GEN_FLT x309 =
		x115 * (x113 * x308 +
				x145 * (-x141 * x304 -
						x143 * (x107 * x305 + x305 * x97 + x308 * x92 +
								x92 * (x106 * x305 + x308 +
									   x92 * (x105 * x305 + x307 + x92 * (x104 * x305 - x142 * x305 + x306))))) -
				x146 * x304 + x147 * x305 + x85);
	const GEN_FLT x310 = -x137;
	const GEN_FLT x311 = x133 * x83;
	const GEN_FLT x312 = x297 * x83;
	const GEN_FLT x313 = -x132 * x311 + x310;
	const GEN_FLT x314 = x312 * x94;
	const GEN_FLT x315 = x312 * x95 + x92 * (-x312 * x93 + x314);
	const GEN_FLT x316 = x312 * x96 + x315 * x92;
	const GEN_FLT x317 =
		-x115 * (x113 * x316 +
				 x145 * (x141 * x313 -
						 x143 * (x107 * x312 + x312 * x97 + x316 * x92 +
								 x92 * (x106 * x312 + x316 +
										x92 * (x105 * x312 + x315 + x92 * (x104 * x312 - x142 * x312 + x314))))) +
				 x146 * x313 + x147 * x312 + x311) +
		x310;
	const GEN_FLT x318 = x39 * x46;
	const GEN_FLT x319 = -x318;
	const GEN_FLT x320 = 2 / ((x3 * x3));
	const GEN_FLT x321 = lh_qi * x320;
	const GEN_FLT x322 = x37 * (x10 * x318 + x319 + x7 * ((x9) ? (-x1 * x321) : (0)));
	const GEN_FLT x323 = pow(x3, -3.0 / 2.0);
	const GEN_FLT x324 = lh_qi * x323;
	const GEN_FLT x325 = ((x9) ? (-lh_qk * x324) : (0));
	const GEN_FLT x326 = x325 * x39;
	const GEN_FLT x327 = x46 * x5;
	const GEN_FLT x328 = x327 * x42;
	const GEN_FLT x329 = ((x9) ? (-lh_qj * x324) : (0));
	const GEN_FLT x330 = ((x9) ? (-x0 * x323 + x40) : (0));
	const GEN_FLT x331 = x46 * x71;
	const GEN_FLT x332 = x329 * x48 + x330 * x58 + x331 * x47;
	const GEN_FLT x333 = x54 * (x326 + x328 + x332);
	const GEN_FLT x334 = x325 * x58;
	const GEN_FLT x335 = x42 * x7;
	const GEN_FLT x336 = x329 * x335;
	const GEN_FLT x337 = x331 * x42;
	const GEN_FLT x338 = -x327 * x47 - x330 * x39;
	const GEN_FLT x339 = x62 * (x334 + x336 + x337 + x338);
	const GEN_FLT x340 = x322 + x333 + x339;
	const GEN_FLT x341 = x62 * (-x318 * x78 + x318 - x7 * ((x9) ? (-x2 * x321) : (0)));
	const GEN_FLT x342 = x325 * x48;
	const GEN_FLT x343 = x330 * x335;
	const GEN_FLT x344 = x43 * x47;
	const GEN_FLT x345 = x344 * x46;
	const GEN_FLT x346 = x329 * x39;
	const GEN_FLT x347 = x327 * x45 + x346;
	const GEN_FLT x348 = x54 * (-x342 - x343 - x345 + x347);
	const GEN_FLT x349 = -x336;
	const GEN_FLT x350 = x37 * (-x334 - x337 + x338 + x349);
	const GEN_FLT x351 = 2 * x8;
	const GEN_FLT x352 =
		x54 * (x318 * x67 + x319 + x7 * ((x9) ? (lh_qi * x351 - x320 * lh_qi * (lh_qi * lh_qi)) : (0)));
	const GEN_FLT x353 = x62 * (x342 + x343 + x345 + x347);
	const GEN_FLT x354 = -x326;
	const GEN_FLT x355 = x37 * (-x328 + x332 + x354);
	const GEN_FLT x356 = -x121 * (2 * x352 + 2 * x353 + 2 * x355) - x124 * (2 * x341 + 2 * x348 + 2 * x350);
	const GEN_FLT x357 = x116 * (x126 * (-x119 * (2 * x322 + 2 * x333 + 2 * x339) + x356) + x340 * x91);
	const GEN_FLT x358 = x357 * x94;
	const GEN_FLT x359 = x357 * x95 + x92 * (-x357 * x93 + x358);
	const GEN_FLT x360 = x357 * x96 + x359 * x92;
	const GEN_FLT x361 = x133 * x356 + x340 * x85;
	const GEN_FLT x362 = x135 * (x352 + x353 + x355) + x137 * (x341 + x348 + x350);
	const GEN_FLT x363 = -x132 * x361 + x362;
	const GEN_FLT x364 =
		-x115 * (x113 * x360 +
				 x145 * (x141 * x363 -
						 x143 * (x107 * x357 + x357 * x97 + x360 * x92 +
								 x92 * (x106 * x357 + x360 +
										x92 * (x105 * x357 + x359 + x92 * (x104 * x357 - x142 * x357 + x358))))) +
				 x146 * x363 + x147 * x357 + x361) +
		x362;
	const GEN_FLT x365 = x39 * x44;
	const GEN_FLT x366 = -x365;
	const GEN_FLT x367 =
		x37 * (x10 * x365 + x366 + x7 * ((x9) ? (lh_qj * x351 - x320 * lh_qj * (lh_qj * lh_qj)) : (0)));
	const GEN_FLT x368 = ((x9) ? (-lh_qj * lh_qk * x323) : (0));
	const GEN_FLT x369 = x368 * x39;
	const GEN_FLT x370 = x44 * x5;
	const GEN_FLT x371 = x370 * x42;
	const GEN_FLT x372 = ((x9) ? (-x1 * x323 + x40) : (0));
	const GEN_FLT x373 = x44 * x71;
	const GEN_FLT x374 = x329 * x58 + x372 * x48 + x373 * x47;
	const GEN_FLT x375 = x54 * (x369 + x371 + x374);
	const GEN_FLT x376 = x368 * x58;
	const GEN_FLT x377 = x335 * x372;
	const GEN_FLT x378 = x373 * x42;
	const GEN_FLT x379 = -x346 - x370 * x47;
	const GEN_FLT x380 = x62 * (x376 + x377 + x378 + x379);
	const GEN_FLT x381 = x367 + x375 + x380;
	const GEN_FLT x382 = lh_qj * x320;
	const GEN_FLT x383 = x54 * (x365 * x67 + x366 + x7 * ((x9) ? (-x0 * x382) : (0)));
	const GEN_FLT x384 = x368 * x48;
	const GEN_FLT x385 = x344 * x44;
	const GEN_FLT x386 = x370 * x45 + x372 * x39;
	const GEN_FLT x387 = x62 * (x336 + x384 + x385 + x386);
	const GEN_FLT x388 = x37 * (-x369 - x371 + x374);
	const GEN_FLT x389 = x62 * (-x365 * x78 + x365 - x7 * ((x9) ? (-x2 * x382) : (0)));
	const GEN_FLT x390 = x54 * (x349 - x384 - x385 + x386);
	const GEN_FLT x391 = x37 * (-x376 - x377 - x378 + x379);
	const GEN_FLT x392 = -x121 * (2 * x383 + 2 * x387 + 2 * x388) - x124 * (2 * x389 + 2 * x390 + 2 * x391);
	const GEN_FLT x393 = x116 * (x126 * (-x119 * (2 * x367 + 2 * x375 + 2 * x380) + x392) + x381 * x91);
	const GEN_FLT x394 = x393 * x94;
	const GEN_FLT x395 = x393 * x95 + x92 * (-x393 * x93 + x394);
	const GEN_FLT x396 = x393 * x96 + x395 * x92;
	const GEN_FLT x397 = x133 * x392 + x381 * x85;
	const GEN_FLT x398 = x135 * (x383 + x387 + x388) + x137 * (x389 + x390 + x391);
	const GEN_FLT x399 = -x132 * x397 + x398;
	const GEN_FLT x400 =
		-x115 * (x113 * x396 +
				 x145 * (x141 * x399 -
						 x143 * (x107 * x393 + x393 * x97 + x396 * x92 +
								 x92 * (x106 * x393 + x396 +
										x92 * (x105 * x393 + x395 + x92 * (x104 * x393 - x142 * x393 + x394))))) +
				 x146 * x399 + x147 * x393 + x397) +
		x398;
	const GEN_FLT x401 = x39 * x41;
	const GEN_FLT x402 = -x401;
	const GEN_FLT x403 = lh_qk * x320;
	const GEN_FLT x404 = x37 * (x10 * x401 + x402 + x7 * ((x9) ? (-x1 * x403) : (0)));
	const GEN_FLT x405 = ((x9) ? (-x2 * x323 + x40) : (0));
	const GEN_FLT x406 = x39 * x405;
	const GEN_FLT x407 = x41 * x5;
	const GEN_FLT x408 = x407 * x42;
	const GEN_FLT x409 = x41 * x71;
	const GEN_FLT x410 = x334 + x384 + x409 * x47;
	const GEN_FLT x411 = x54 * (x406 + x408 + x410);
	const GEN_FLT x412 = x405 * x58;
	const GEN_FLT x413 = x335 * x368;
	const GEN_FLT x414 = x409 * x42;
	const GEN_FLT x415 = x354 - x407 * x47;
	const GEN_FLT x416 = x62 * (x412 + x413 + x414 + x415);
	const GEN_FLT x417 = x404 + x411 + x416;
	const GEN_FLT x418 = x54 * (x401 * x67 + x402 + x7 * ((x9) ? (-x0 * x403) : (0)));
	const GEN_FLT x419 = x405 * x48;
	const GEN_FLT x420 = x325 * x335;
	const GEN_FLT x421 = x344 * x41;
	const GEN_FLT x422 = x369 + x407 * x45;
	const GEN_FLT x423 = x62 * (x419 + x420 + x421 + x422);
	const GEN_FLT x424 = x37 * (-x406 - x408 + x410);
	const GEN_FLT x425 =
		x62 * (-x401 * x78 + x401 - x7 * ((x9) ? (lh_qk * x351 - x320 * lh_qk * (lh_qk * lh_qk)) : (0)));
	const GEN_FLT x426 = x54 * (-x419 - x420 - x421 + x422);
	const GEN_FLT x427 = x37 * (-x412 - x413 - x414 + x415);
	const GEN_FLT x428 = -x121 * (2 * x418 + 2 * x423 + 2 * x424) - x124 * (2 * x425 + 2 * x426 + 2 * x427);
	const GEN_FLT x429 = x116 * (x126 * (-x119 * (2 * x404 + 2 * x411 + 2 * x416) + x428) + x417 * x91);
	const GEN_FLT x430 = x429 * x94;
	const GEN_FLT x431 = x429 * x95 + x92 * (-x429 * x93 + x430);
	const GEN_FLT x432 = x429 * x96 + x431 * x92;
	const GEN_FLT x433 = x133 * x428 + x417 * x85;
	const GEN_FLT x434 = x135 * (x418 + x423 + x424) + x137 * (x425 + x426 + x427);
	const GEN_FLT x435 = -x132 * x433 + x434;
	const GEN_FLT x436 =
		-x115 * (x113 * x432 +
				 x145 * (x141 * x435 -
						 x143 * (x107 * x429 + x429 * x97 + x432 * x92 +
								 x92 * (x106 * x429 + x432 +
										x92 * (x105 * x429 + x431 + x92 * (x104 * x429 - x142 * x429 + x430))))) +
				 x146 * x435 + x147 * x429 + x433) +
		x434;
	*(out++) = x148 * x149 + x148;
	*(out++) = x149 * x161 + x161;
	*(out++) = x149 * x171 + x171;
	*(out++) = x149 * x220 + x220;
	*(out++) = x149 * x259 + x259;
	*(out++) = x149 * x294 + x294;
	*(out++) = x149 * x303 + x303;
	*(out++) = -x149 * x309 - x309;
	*(out++) = x149 * x317 + x317;
	*(out++) = x149 * x364 + x364;
	*(out++) = x149 * x400 + x400;
	*(out++) = x149 * x436 + x436;
}

static inline void gen_reproject_axisangle_jac_all(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
												   const FLT phase_0, const FLT phase_1, const FLT tilt_0,
												   const FLT tilt_1, const FLT curve_0, const FLT curve_1,
												   const FLT gibPhase_0, const FLT gibPhase_1, const FLT gibMag_0,
												   const FLT gibMag_1) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = -x5;
	const GEN_FLT x7 = x6 + 1;
	const GEN_FLT x8 = 1.0 / x3;
	const GEN_FLT x9 = x4 > 0;
	const GEN_FLT x10 = ((x9) ? (x0 * x8) : (1));
	const GEN_FLT x11 = x10 * x7;
	const GEN_FLT x12 = x11 + x5;
	const GEN_FLT x13 = ((x9) ? (x2 * x8) : (0));
	const GEN_FLT x14 = x13 * x7;
	const GEN_FLT x15 = obj_qi * obj_qi;
	const GEN_FLT x16 = obj_qj * obj_qj;
	const GEN_FLT x17 = obj_qk * obj_qk;
	const GEN_FLT x18 = x15 + x16 + x17;
	const GEN_FLT x19 = sqrt(x18);
	const GEN_FLT x20 = cos(x19);
	const GEN_FLT x21 = 1 - x20;
	const GEN_FLT x22 = 1.0 / x18;
	const GEN_FLT x23 = x19 > 0;
	const GEN_FLT x24 = ((x23) ? (x17 * x22) : (0));
	const GEN_FLT x25 = sin(x19);
	const GEN_FLT x26 = 1.0 / x19;
	const GEN_FLT x27 = obj_qi * x26;
	const GEN_FLT x28 = ((x23) ? (x27) : (1));
	const GEN_FLT x29 = x25 * x28;
	const GEN_FLT x30 = obj_qk * x26;
	const GEN_FLT x31 = ((x23) ? (x30) : (0));
	const GEN_FLT x32 = obj_qj * x26;
	const GEN_FLT x33 = ((x23) ? (x32) : (0));
	const GEN_FLT x34 = x21 * x33;
	const GEN_FLT x35 = x31 * x34;
	const GEN_FLT x36 = x25 * x33;
	const GEN_FLT x37 = x21 * x28;
	const GEN_FLT x38 = x31 * x37;
	const GEN_FLT x39 = obj_pz + sensor_x * (-x36 + x38) + sensor_y * (x29 + x35) + sensor_z * (x20 + x21 * x24);
	const GEN_FLT x40 = sin(x4);
	const GEN_FLT x41 = 1.0 / x4;
	const GEN_FLT x42 = lh_qi * x41;
	const GEN_FLT x43 = ((x9) ? (x42) : (1));
	const GEN_FLT x44 = x40 * x43;
	const GEN_FLT x45 = lh_qk * x41;
	const GEN_FLT x46 = ((x9) ? (x45) : (0));
	const GEN_FLT x47 = lh_qj * x41;
	const GEN_FLT x48 = ((x9) ? (x47) : (0));
	const GEN_FLT x49 = x48 * x7;
	const GEN_FLT x50 = x46 * x49;
	const GEN_FLT x51 = ((x23) ? (x16 * x22) : (0));
	const GEN_FLT x52 = x25 * x31;
	const GEN_FLT x53 = x33 * x37;
	const GEN_FLT x54 = obj_py + sensor_x * (x52 + x53) + sensor_y * (x20 + x21 * x51) + sensor_z * (-x29 + x35);
	const GEN_FLT x55 = x40 * x48;
	const GEN_FLT x56 = x43 * x7;
	const GEN_FLT x57 = x46 * x56;
	const GEN_FLT x58 = ((x23) ? (x15 * x22) : (1));
	const GEN_FLT x59 = obj_px + sensor_x * (x20 + x21 * x58) + sensor_y * (-x52 + x53) + sensor_z * (x36 + x38);
	const GEN_FLT x60 = -lh_pz - x39 * (x14 + x5) - x54 * (x44 + x50) - x59 * (-x55 + x57);
	const GEN_FLT x61 = x12 * x59;
	const GEN_FLT x62 = x55 + x57;
	const GEN_FLT x63 = x39 * x62;
	const GEN_FLT x64 = x40 * x46;
	const GEN_FLT x65 = -x64;
	const GEN_FLT x66 = x48 * x56;
	const GEN_FLT x67 = x65 + x66;
	const GEN_FLT x68 = x54 * x67;
	const GEN_FLT x69 = lh_px + x61 + x63 + x68;
	const GEN_FLT x70 = x69 * x69;
	const GEN_FLT x71 = x60 * x60;
	const GEN_FLT x72 = x70 + x71;
	const GEN_FLT x73 = 1.0 / x72;
	const GEN_FLT x74 = x60 * x73;
	const GEN_FLT x75 = x12 * x74;
	const GEN_FLT x76 = x55 - x57;
	const GEN_FLT x77 = -lh_px - x61 - x63 - x68;
	const GEN_FLT x78 = x73 * x77;
	const GEN_FLT x79 = x76 * x78;
	const GEN_FLT x80 = x64 + x66;
	const GEN_FLT x81 = ((x9) ? (x1 * x8) : (0));
	const GEN_FLT x82 = x7 * x81;
	const GEN_FLT x83 = x5 + x82;
	const GEN_FLT x84 = x54 * x83;
	const GEN_FLT x85 = x59 * x80;
	const GEN_FLT x86 = -x44;
	const GEN_FLT x87 = x50 + x86;
	const GEN_FLT x88 = x39 * x87;
	const GEN_FLT x89 = lh_py + x84 + x85 + x88;
	const GEN_FLT x90 = x89 * x89;
	const GEN_FLT x91 = 2 / (x71 + x90);
	const GEN_FLT x92 = x60 * x91;
	const GEN_FLT x93 = -lh_py - x84 - x85 - x88;
	const GEN_FLT x94 = x91 * x93;
	const GEN_FLT x95 = curve_0 * atan2(x89, x60);
	const GEN_FLT x96 = pow(-x73 * x90 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x97 = tilt_0 / sqrt(x72);
	const GEN_FLT x98 = 2 * x5;
	const GEN_FLT x99 = (1.0 / 2.0) * x69;
	const GEN_FLT x100 = 2 * x55;
	const GEN_FLT x101 = 2 * x57;
	const GEN_FLT x102 = (1.0 / 2.0) * x60;
	const GEN_FLT x103 = -x102 * (x100 - x101);
	const GEN_FLT x104 = tilt_0 * x89 / pow(x72, 3.0 / 2.0);
	const GEN_FLT x105 = x96 * (x104 * (x103 - x99 * (2 * x11 + x98)) + x80 * x97);
	const GEN_FLT x106 = atan2(x69, x60);
	const GEN_FLT x107 = gibMag_0 * sin(-gibPhase_0 + phase_0 + x106 + asin(x89 * x97) - 1.5707963267948966);
	const GEN_FLT x108 = x67 * x74;
	const GEN_FLT x109 = -x50;
	const GEN_FLT x110 = x109 + x86;
	const GEN_FLT x111 = x110 * x78;
	const GEN_FLT x112 = -2 * x64;
	const GEN_FLT x113 = 2 * x66;
	const GEN_FLT x114 = 2 * x44;
	const GEN_FLT x115 = -2 * x50;
	const GEN_FLT x116 = -x102 * (-x114 + x115);
	const GEN_FLT x117 = x96 * (x104 * (x116 - x99 * (x112 + x113)) + x83 * x97);
	const GEN_FLT x118 = -x14 + x6;
	const GEN_FLT x119 = x118 * x78;
	const GEN_FLT x120 = x62 * x74;
	const GEN_FLT x121 = -x98;
	const GEN_FLT x122 = -x102 * (x121 - 2 * x14);
	const GEN_FLT x123 = x96 * (x104 * (x122 - x99 * (x100 + x101)) + x87 * x97);
	const GEN_FLT x124 = x25 * x27;
	const GEN_FLT x125 = -x124;
	const GEN_FLT x126 = 2 * x22;
	const GEN_FLT x127 = 2 / ((x18 * x18));
	const GEN_FLT x128 = pow(x18, -3.0 / 2.0);
	const GEN_FLT x129 = obj_qi * x128;
	const GEN_FLT x130 = ((x23) ? (-obj_qj * x129) : (0));
	const GEN_FLT x131 = x130 * x25;
	const GEN_FLT x132 = x20 * x27;
	const GEN_FLT x133 = x132 * x33;
	const GEN_FLT x134 = ((x23) ? (-obj_qk * x129) : (0));
	const GEN_FLT x135 = ((x23) ? (-x128 * x15 + x26) : (0));
	const GEN_FLT x136 = x21 * x31;
	const GEN_FLT x137 = x27 * x28;
	const GEN_FLT x138 = x134 * x37 + x135 * x136 + x137 * x52;
	const GEN_FLT x139 = x134 * x25;
	const GEN_FLT x140 = -x139;
	const GEN_FLT x141 = x132 * x31;
	const GEN_FLT x142 = x130 * x37 + x135 * x34 + x137 * x36;
	const GEN_FLT x143 =
		sensor_x * (x124 * x58 + x125 + x21 * ((x23) ? (obj_qi * x126 - x127 * obj_qi * (obj_qi * obj_qi)) : (0))) +
		sensor_y * (x140 - x141 + x142) + sensor_z * (x131 + x133 + x138);
	const GEN_FLT x144 = x12 * x143;
	const GEN_FLT x145 = obj_qi * x127;
	const GEN_FLT x146 = x135 * x25;
	const GEN_FLT x147 = x132 * x28;
	const GEN_FLT x148 = x134 * x34;
	const GEN_FLT x149 = x130 * x136;
	const GEN_FLT x150 = x33 * x52;
	const GEN_FLT x151 = x148 + x149 + x150 * x27;
	const GEN_FLT x152 = -x131;
	const GEN_FLT x153 = sensor_x * (-x133 + x138 + x152) + sensor_y * (x146 + x147 + x151) +
						 sensor_z * (x124 * x24 + x125 + x21 * ((x23) ? (-x145 * x17) : (0)));
	const GEN_FLT x154 = x153 * x62;
	const GEN_FLT x155 = sensor_x * (x139 + x141 + x142) +
						 sensor_y * (x124 * x51 + x125 + x21 * ((x23) ? (-x145 * x16) : (0))) +
						 sensor_z * (-x146 - x147 + x151);
	const GEN_FLT x156 = x155 * x67;
	const GEN_FLT x157 = x144 + x154 + x156;
	const GEN_FLT x158 = x157 * x74;
	const GEN_FLT x159 = x118 * x153;
	const GEN_FLT x160 = x110 * x155;
	const GEN_FLT x161 = x143 * x76;
	const GEN_FLT x162 = x159 + x160 + x161;
	const GEN_FLT x163 = x162 * x78;
	const GEN_FLT x164 = x143 * x80 + x153 * x87 + x155 * x83;
	const GEN_FLT x165 = -x102 * (2 * x159 + 2 * x160 + 2 * x161);
	const GEN_FLT x166 = x96 * (x104 * (x165 - x99 * (2 * x144 + 2 * x154 + 2 * x156)) + x164 * x97);
	const GEN_FLT x167 = x25 * x32;
	const GEN_FLT x168 = -x167;
	const GEN_FLT x169 = obj_qj * x127;
	const GEN_FLT x170 = ((x23) ? (-x128 * x16 + x26) : (0));
	const GEN_FLT x171 = x170 * x25;
	const GEN_FLT x172 = x20 * x32;
	const GEN_FLT x173 = x172 * x33;
	const GEN_FLT x174 = ((x23) ? (-obj_qj * obj_qk * x128) : (0));
	const GEN_FLT x175 = x174 * x37;
	const GEN_FLT x176 = x28 * x32;
	const GEN_FLT x177 = x149 + x175 + x176 * x52;
	const GEN_FLT x178 = x174 * x25;
	const GEN_FLT x179 = -x178;
	const GEN_FLT x180 = x172 * x31;
	const GEN_FLT x181 = x130 * x34 + x170 * x37 + x176 * x36;
	const GEN_FLT x182 = sensor_x * (x167 * x58 + x168 + x21 * ((x23) ? (-x15 * x169) : (0))) +
						 sensor_y * (x179 - x180 + x181) + sensor_z * (x171 + x173 + x177);
	const GEN_FLT x183 = x12 * x182;
	const GEN_FLT x184 = x172 * x28;
	const GEN_FLT x185 = x136 * x170 + x150 * x32 + x174 * x34;
	const GEN_FLT x186 = sensor_x * (-x171 - x173 + x177) + sensor_y * (x131 + x184 + x185) +
						 sensor_z * (x167 * x24 + x168 + x21 * ((x23) ? (-x169 * x17) : (0)));
	const GEN_FLT x187 = x186 * x62;
	const GEN_FLT x188 =
		sensor_x * (x178 + x180 + x181) +
		sensor_y * (x167 * x51 + x168 + x21 * ((x23) ? (obj_qj * x126 - x127 * obj_qj * (obj_qj * obj_qj)) : (0))) +
		sensor_z * (x152 - x184 + x185);
	const GEN_FLT x189 = x188 * x67;
	const GEN_FLT x190 = x183 + x187 + x189;
	const GEN_FLT x191 = x190 * x74;
	const GEN_FLT x192 = x118 * x186;
	const GEN_FLT x193 = x182 * x76;
	const GEN_FLT x194 = x110 * x188;
	const GEN_FLT x195 = x192 + x193 + x194;
	const GEN_FLT x196 = x195 * x78;
	const GEN_FLT x197 = x182 * x80 + x186 * x87 + x188 * x83;
	const GEN_FLT x198 = -x102 * (2 * x192 + 2 * x193 + 2 * x194);
	const GEN_FLT x199 = x96 * (x104 * (x198 - x99 * (2 * x183 + 2 * x187 + 2 * x189)) + x197 * x97);
	const GEN_FLT x200 = x25 * x30;
	const GEN_FLT x201 = -x200;
	const GEN_FLT x202 = obj_qk * x127;
	const GEN_FLT x203 = x20 * x30;
	const GEN_FLT x204 = x203 * x33;
	const GEN_FLT x205 = ((x23) ? (-x128 * x17 + x26) : (0));
	const GEN_FLT x206 = x28 * x30;
	const GEN_FLT x207 = x134 * x136 + x205 * x37 + x206 * x52;
	const GEN_FLT x208 = x205 * x25;
	const GEN_FLT x209 = x203 * x31;
	const GEN_FLT x210 = x148 + x175 + x206 * x36;
	const GEN_FLT x211 = sensor_x * (x200 * x58 + x201 + x21 * ((x23) ? (-x15 * x202) : (0))) +
						 sensor_y * (-x208 - x209 + x210) + sensor_z * (x178 + x204 + x207);
	const GEN_FLT x212 = x12 * x211;
	const GEN_FLT x213 = x203 * x28;
	const GEN_FLT x214 = x136 * x174 + x150 * x30 + x205 * x34;
	const GEN_FLT x215 = sensor_x * (x208 + x209 + x210) +
						 sensor_y * (x200 * x51 + x201 + x21 * ((x23) ? (-x16 * x202) : (0))) +
						 sensor_z * (x140 - x213 + x214);
	const GEN_FLT x216 = x215 * x67;
	const GEN_FLT x217 =
		sensor_x * (x179 - x204 + x207) + sensor_y * (x139 + x213 + x214) +
		sensor_z * (x200 * x24 + x201 + x21 * ((x23) ? (obj_qk * x126 - x127 * obj_qk * (obj_qk * obj_qk)) : (0)));
	const GEN_FLT x218 = x217 * x62;
	const GEN_FLT x219 = x212 + x216 + x218;
	const GEN_FLT x220 = x219 * x74;
	const GEN_FLT x221 = x118 * x217;
	const GEN_FLT x222 = x211 * x76;
	const GEN_FLT x223 = x110 * x215;
	const GEN_FLT x224 = x221 + x222 + x223;
	const GEN_FLT x225 = x224 * x78;
	const GEN_FLT x226 = x211 * x80 + x215 * x83 + x217 * x87;
	const GEN_FLT x227 = -x102 * (2 * x221 + 2 * x222 + 2 * x223);
	const GEN_FLT x228 = x96 * (x104 * (x227 - x99 * (2 * x212 + 2 * x216 + 2 * x218)) + x226 * x97);
	const GEN_FLT x229 = x104 * x96;
	const GEN_FLT x230 = x229 * x77;
	const GEN_FLT x231 = x96 * x97;
	const GEN_FLT x232 = x229 * x60;
	const GEN_FLT x233 = x40 * x42;
	const GEN_FLT x234 = 2 / ((x3 * x3));
	const GEN_FLT x235 = lh_qi * x234;
	const GEN_FLT x236 = x39 * (-x13 * x233 + x233 - x7 * ((x9) ? (-x2 * x235) : (0)));
	const GEN_FLT x237 = pow(x3, -3.0 / 2.0);
	const GEN_FLT x238 = lh_qi * x237;
	const GEN_FLT x239 = ((x9) ? (-lh_qk * x238) : (0));
	const GEN_FLT x240 = x239 * x56;
	const GEN_FLT x241 = ((x9) ? (-x0 * x237 + x41) : (0));
	const GEN_FLT x242 = x46 * x7;
	const GEN_FLT x243 = x241 * x242;
	const GEN_FLT x244 = x43 * x64;
	const GEN_FLT x245 = x244 * x42;
	const GEN_FLT x246 = ((x9) ? (-lh_qj * x238) : (0));
	const GEN_FLT x247 = x246 * x40;
	const GEN_FLT x248 = x42 * x5;
	const GEN_FLT x249 = x247 + x248 * x48;
	const GEN_FLT x250 = x59 * (-x240 - x243 - x245 + x249);
	const GEN_FLT x251 = x241 * x40;
	const GEN_FLT x252 = x248 * x43;
	const GEN_FLT x253 = -x251 - x252;
	const GEN_FLT x254 = x239 * x49;
	const GEN_FLT x255 = -x254;
	const GEN_FLT x256 = x242 * x246;
	const GEN_FLT x257 = -x256;
	const GEN_FLT x258 = x42 * x55;
	const GEN_FLT x259 = x258 * x46;
	const GEN_FLT x260 = x255 + x257 - x259;
	const GEN_FLT x261 = x54 * (x253 + x260);
	const GEN_FLT x262 = x236 + x250 + x261;
	const GEN_FLT x263 = x262 * x78;
	const GEN_FLT x264 = -x233;
	const GEN_FLT x265 = 2 * x8;
	const GEN_FLT x266 =
		x59 * (x10 * x233 + x264 + x7 * ((x9) ? (lh_qi * x265 - x234 * lh_qi * (lh_qi * lh_qi)) : (0)));
	const GEN_FLT x267 = x39 * (x240 + x243 + x245 + x249);
	const GEN_FLT x268 = x246 * x56;
	const GEN_FLT x269 = x241 * x49;
	const GEN_FLT x270 = x258 * x43;
	const GEN_FLT x271 = x268 + x269 + x270;
	const GEN_FLT x272 = x239 * x40;
	const GEN_FLT x273 = -x272;
	const GEN_FLT x274 = x248 * x46;
	const GEN_FLT x275 = x273 - x274;
	const GEN_FLT x276 = x54 * (x271 + x275);
	const GEN_FLT x277 = x266 + x267 + x276;
	const GEN_FLT x278 = x277 * x74;
	const GEN_FLT x279 = x7 * ((x9) ? (-x1 * x235) : (0));
	const GEN_FLT x280 = x233 * x81;
	const GEN_FLT x281 = x39 * (x253 + x254 + x256 + x259) + x54 * (x264 + x279 + x280) + x59 * (x271 + x272 + x274);
	const GEN_FLT x282 = -x102 * (2 * x236 + 2 * x250 + 2 * x261);
	const GEN_FLT x283 = x96 * (x104 * (x282 - x99 * (2 * x266 + 2 * x267 + 2 * x276)) + x281 * x97);
	const GEN_FLT x284 = x40 * x47;
	const GEN_FLT x285 = -x284;
	const GEN_FLT x286 = lh_qj * x234;
	const GEN_FLT x287 = x59 * (x10 * x284 + x285 + x7 * ((x9) ? (-x0 * x286) : (0)));
	const GEN_FLT x288 = ((x9) ? (-lh_qj * lh_qk * x237) : (0));
	const GEN_FLT x289 = x288 * x56;
	const GEN_FLT x290 = x244 * x47;
	const GEN_FLT x291 = ((x9) ? (-x1 * x237 + x41) : (0));
	const GEN_FLT x292 = x47 * x5;
	const GEN_FLT x293 = x291 * x40 + x292 * x48;
	const GEN_FLT x294 = x39 * (x256 + x289 + x290 + x293);
	const GEN_FLT x295 = x291 * x56;
	const GEN_FLT x296 = x246 * x49;
	const GEN_FLT x297 = x47 * x55;
	const GEN_FLT x298 = x297 * x43;
	const GEN_FLT x299 = x295 + x296 + x298;
	const GEN_FLT x300 = x288 * x40;
	const GEN_FLT x301 = x292 * x46;
	const GEN_FLT x302 = -x300 - x301;
	const GEN_FLT x303 = x54 * (x299 + x302);
	const GEN_FLT x304 = x287 + x294 + x303;
	const GEN_FLT x305 = x304 * x74;
	const GEN_FLT x306 = x39 * (-x13 * x284 + x284 - x7 * ((x9) ? (-x2 * x286) : (0)));
	const GEN_FLT x307 = -x289;
	const GEN_FLT x308 = x59 * (x257 - x290 + x293 + x307);
	const GEN_FLT x309 = x292 * x43;
	const GEN_FLT x310 = -x247 - x309;
	const GEN_FLT x311 = x288 * x49;
	const GEN_FLT x312 = x242 * x291;
	const GEN_FLT x313 = x297 * x46;
	const GEN_FLT x314 = -x311 - x312 - x313;
	const GEN_FLT x315 = x54 * (x310 + x314);
	const GEN_FLT x316 = x306 + x308 + x315;
	const GEN_FLT x317 = x316 * x78;
	const GEN_FLT x318 = x284 * x81;
	const GEN_FLT x319 = x7 * ((x9) ? (lh_qj * x265 - x234 * lh_qj * (lh_qj * lh_qj)) : (0));
	const GEN_FLT x320 = x39 * (x310 + x311 + x312 + x313) + x54 * (x285 + x318 + x319) + x59 * (x299 + x300 + x301);
	const GEN_FLT x321 = -x102 * (2 * x306 + 2 * x308 + 2 * x315);
	const GEN_FLT x322 = x96 * (x104 * (x321 - x99 * (2 * x287 + 2 * x294 + 2 * x303)) + x320 * x97);
	const GEN_FLT x323 = x40 * x45;
	const GEN_FLT x324 = -x323;
	const GEN_FLT x325 = lh_qk * x234;
	const GEN_FLT x326 = x59 * (x10 * x323 + x324 + x7 * ((x9) ? (-x0 * x325) : (0)));
	const GEN_FLT x327 = ((x9) ? (-x2 * x237 + x41) : (0));
	const GEN_FLT x328 = x327 * x56;
	const GEN_FLT x329 = x239 * x242;
	const GEN_FLT x330 = x244 * x45;
	const GEN_FLT x331 = x45 * x5;
	const GEN_FLT x332 = x300 + x331 * x48;
	const GEN_FLT x333 = x39 * (x328 + x329 + x330 + x332);
	const GEN_FLT x334 = x45 * x55;
	const GEN_FLT x335 = x334 * x43;
	const GEN_FLT x336 = x254 + x289 + x335;
	const GEN_FLT x337 = x327 * x40;
	const GEN_FLT x338 = x331 * x46;
	const GEN_FLT x339 = -x337 - x338;
	const GEN_FLT x340 = x54 * (x336 + x339);
	const GEN_FLT x341 = x326 + x333 + x340;
	const GEN_FLT x342 = x341 * x74;
	const GEN_FLT x343 =
		x39 * (-x13 * x323 + x323 - x7 * ((x9) ? (lh_qk * x265 - x234 * lh_qk * (lh_qk * lh_qk)) : (0)));
	const GEN_FLT x344 = x59 * (-x328 - x329 - x330 + x332);
	const GEN_FLT x345 = x331 * x43;
	const GEN_FLT x346 = x273 - x345;
	const GEN_FLT x347 = x327 * x49;
	const GEN_FLT x348 = x242 * x288;
	const GEN_FLT x349 = x334 * x46;
	const GEN_FLT x350 = -x347 - x348 - x349;
	const GEN_FLT x351 = x54 * (x346 + x350);
	const GEN_FLT x352 = x343 + x344 + x351;
	const GEN_FLT x353 = x352 * x78;
	const GEN_FLT x354 = x7 * ((x9) ? (-x1 * x325) : (0));
	const GEN_FLT x355 = x323 * x81;
	const GEN_FLT x356 = x39 * (x346 + x347 + x348 + x349) + x54 * (x324 + x354 + x355) + x59 * (x336 + x337 + x338);
	const GEN_FLT x357 = -x102 * (2 * x343 + 2 * x344 + 2 * x351);
	const GEN_FLT x358 = x96 * (x104 * (x357 - x99 * (2 * x326 + 2 * x333 + 2 * x340)) + x356 * x97);
	const GEN_FLT x359 = x71 + x93 * x93;
	const GEN_FLT x360 = 1.0 / x359;
	const GEN_FLT x361 = x360 * x89;
	const GEN_FLT x362 = x361 * x76;
	const GEN_FLT x363 = x65 - x66;
	const GEN_FLT x364 = x360 * x60;
	const GEN_FLT x365 = x363 * x364;
	const GEN_FLT x366 = curve_1 * x106;
	const GEN_FLT x367 = pow(-x360 * x70 * tilt_1 * tilt_1 + 1, -1.0 / 2.0);
	const GEN_FLT x368 = tilt_1 / sqrt(x359);
	const GEN_FLT x369 = (1.0 / 2.0) * x93;
	const GEN_FLT x370 = tilt_1 * x69 / pow(x359, 3.0 / 2.0);
	const GEN_FLT x371 = x367 * (x12 * x368 + x370 * (x103 - x369 * (x112 - x113)));
	const GEN_FLT x372 =
		gibMag_1 * sin(-gibPhase_1 + phase_1 + asin(x368 * x69) + atan2(x93, x60) - 1.5707963267948966);
	const GEN_FLT x373 = x6 - x82;
	const GEN_FLT x374 = x364 * x373;
	const GEN_FLT x375 = x110 * x361;
	const GEN_FLT x376 = x367 * (x368 * x67 + x370 * (x116 - x369 * (x121 - 2 * x82)));
	const GEN_FLT x377 = x118 * x361;
	const GEN_FLT x378 = x109 + x44;
	const GEN_FLT x379 = x364 * x378;
	const GEN_FLT x380 = x367 * (x368 * x62 + x370 * (x122 - x369 * (x114 + x115)));
	const GEN_FLT x381 = x162 * x361;
	const GEN_FLT x382 = x155 * x373;
	const GEN_FLT x383 = x153 * x378;
	const GEN_FLT x384 = x143 * x363;
	const GEN_FLT x385 = x364 * (x382 + x383 + x384);
	const GEN_FLT x386 = x367 * (x157 * x368 + x370 * (x165 - x369 * (2 * x382 + 2 * x383 + 2 * x384)));
	const GEN_FLT x387 = x195 * x361;
	const GEN_FLT x388 = x188 * x373;
	const GEN_FLT x389 = x186 * x378;
	const GEN_FLT x390 = x182 * x363;
	const GEN_FLT x391 = x364 * (x388 + x389 + x390);
	const GEN_FLT x392 = x367 * (x190 * x368 + x370 * (x198 - x369 * (2 * x388 + 2 * x389 + 2 * x390)));
	const GEN_FLT x393 = x224 * x361;
	const GEN_FLT x394 = x215 * x373;
	const GEN_FLT x395 = x211 * x363;
	const GEN_FLT x396 = x217 * x378;
	const GEN_FLT x397 = x364 * (x394 + x395 + x396);
	const GEN_FLT x398 = x367 * (x219 * x368 + x370 * (x227 - x369 * (2 * x394 + 2 * x395 + 2 * x396)));
	const GEN_FLT x399 = 2 * x366;
	const GEN_FLT x400 = x367 * x368;
	const GEN_FLT x401 = x367 * x370;
	const GEN_FLT x402 = x401 * x93;
	const GEN_FLT x403 = x401 * x60;
	const GEN_FLT x404 = x262 * x361;
	const GEN_FLT x405 = x54 * (x233 - x279 - x280);
	const GEN_FLT x406 = x39 * (x251 + x252 + x260);
	const GEN_FLT x407 = x59 * (-x268 - x269 - x270 + x275);
	const GEN_FLT x408 = x364 * (x405 + x406 + x407);
	const GEN_FLT x409 = x367 * (x277 * x368 + x370 * (x282 - x369 * (2 * x405 + 2 * x406 + 2 * x407)));
	const GEN_FLT x410 = x316 * x361;
	const GEN_FLT x411 = x54 * (x284 - x318 - x319);
	const GEN_FLT x412 = x39 * (x247 + x309 + x314);
	const GEN_FLT x413 = x59 * (-x295 - x296 - x298 + x302);
	const GEN_FLT x414 = x364 * (x411 + x412 + x413);
	const GEN_FLT x415 = x367 * (x304 * x368 + x370 * (x321 - x369 * (2 * x411 + 2 * x412 + 2 * x413)));
	const GEN_FLT x416 = x54 * (x323 - x354 - x355);
	const GEN_FLT x417 = x39 * (x272 + x345 + x350);
	const GEN_FLT x418 = x59 * (x255 + x307 - x335 + x339);
	const GEN_FLT x419 = x364 * (x416 + x417 + x418);
	const GEN_FLT x420 = x352 * x361;
	const GEN_FLT x421 = x367 * (x341 * x368 + x370 * (x357 - x369 * (2 * x416 + 2 * x417 + 2 * x418)));
	*(out++) = -x105 + x107 * (x105 + x75 + x79) - x75 - x79 + x95 * (x76 * x94 + x80 * x92);
	*(out++) = x107 * (x108 + x111 + x117) - x108 - x111 - x117 + x95 * (x110 * x94 + x83 * x92);
	*(out++) = x107 * (x119 + x120 + x123) - x119 - x120 - x123 + x95 * (x118 * x94 + x87 * x92);
	*(out++) = x107 * (x158 + x163 + x166) - x158 - x163 - x166 + x95 * (x162 * x94 + x164 * x92);
	*(out++) = x107 * (x191 + x196 + x199) - x191 - x196 - x199 + x95 * (x195 * x94 + x197 * x92);
	*(out++) = x107 * (x220 + x225 + x228) - x220 - x225 - x228 + x95 * (x224 * x94 + x226 * x92);
	*(out++) = x107 * (x230 + x74) - x230 - x74;
	*(out++) = x107 * x231 - x231 + x92 * x95;
	*(out++) = x107 * (x232 - x78) - x232 + x78 - x94 * x95;
	*(out++) = x107 * (x263 + x278 + x283) - x263 - x278 - x283 + x95 * (x262 * x94 + x281 * x92);
	*(out++) = x107 * (x305 + x317 + x322) - x305 - x317 - x322 + x95 * (x316 * x94 + x320 * x92);
	*(out++) = x107 * (x342 + x353 + x358) - x342 - x353 - x358 + x95 * (x352 * x94 + x356 * x92);
	*(out++) = -x362 - x365 + x366 * (2 * x75 + 2 * x79) - x371 + x372 * (x362 + x365 + x371);
	*(out++) = x366 * (2 * x108 + 2 * x111) + x372 * (x374 + x375 + x376) - x374 - x375 - x376;
	*(out++) = x366 * (2 * x119 + 2 * x120) + x372 * (x377 + x379 + x380) - x377 - x379 - x380;
	*(out++) = x366 * (2 * x158 + 2 * x163) + x372 * (x381 + x385 + x386) - x381 - x385 - x386;
	*(out++) = x366 * (2 * x191 + 2 * x196) + x372 * (x387 + x391 + x392) - x387 - x391 - x392;
	*(out++) = x366 * (2 * x220 + 2 * x225) + x372 * (x393 + x397 + x398) - x393 - x397 - x398;
	*(out++) = x372 * x400 + x399 * x74 - x400;
	*(out++) = x364 + x372 * (-x364 + x402) - x402;
	*(out++) = x361 + x372 * (-x361 + x403) - x399 * x78 - x403;
	*(out++) = x366 * (2 * x263 + 2 * x278) + x372 * (x404 + x408 + x409) - x404 - x408 - x409;
	*(out++) = x366 * (2 * x305 + 2 * x317) + x372 * (x410 + x414 + x415) - x410 - x414 - x415;
	*(out++) = x366 * (2 * x342 + 2 * x353) + x372 * (x419 + x420 + x421) - x419 - x420 - x421;
}

static inline void gen_reproject_axisangle_axis_x_jac_all(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
														  const FLT phase_0, const FLT tilt_0, const FLT curve_0,
														  const FLT gibPhase_0, const FLT gibMag_0) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = -x5;
	const GEN_FLT x7 = x6 + 1;
	const GEN_FLT x8 = 1.0 / x3;
	const GEN_FLT x9 = x4 > 0;
	const GEN_FLT x10 = ((x9) ? (x0 * x8) : (1));
	const GEN_FLT x11 = x10 * x7;
	const GEN_FLT x12 = x11 + x5;
	const GEN_FLT x13 = ((x9) ? (x2 * x8) : (0));
	const GEN_FLT x14 = x13 * x7;
	const GEN_FLT x15 = obj_qi * obj_qi;
	const GEN_FLT x16 = obj_qj * obj_qj;
	const GEN_FLT x17 = obj_qk * obj_qk;
	const GEN_FLT x18 = x15 + x16 + x17;
	const GEN_FLT x19 = sqrt(x18);
	const GEN_FLT x20 = cos(x19);
	const GEN_FLT x21 = 1 - x20;
	const GEN_FLT x22 = 1.0 / x18;
	const GEN_FLT x23 = x19 > 0;
	const GEN_FLT x24 = ((x23) ? (x17 * x22) : (0));
	const GEN_FLT x25 = sin(x19);
	const GEN_FLT x26 = 1.0 / x19;
	const GEN_FLT x27 = obj_qi * x26;
	const GEN_FLT x28 = ((x23) ? (x27) : (1));
	const GEN_FLT x29 = x25 * x28;
	const GEN_FLT x30 = obj_qk * x26;
	const GEN_FLT x31 = ((x23) ? (x30) : (0));
	const GEN_FLT x32 = obj_qj * x26;
	const GEN_FLT x33 = ((x23) ? (x32) : (0));
	const GEN_FLT x34 = x21 * x33;
	const GEN_FLT x35 = x31 * x34;
	const GEN_FLT x36 = x25 * x33;
	const GEN_FLT x37 = x21 * x28;
	const GEN_FLT x38 = x31 * x37;
	const GEN_FLT x39 = obj_pz + sensor_x * (-x36 + x38) + sensor_y * (x29 + x35) + sensor_z * (x20 + x21 * x24);
	const GEN_FLT x40 = sin(x4);
	const GEN_FLT x41 = 1.0 / x4;
	const GEN_FLT x42 = lh_qi * x41;
	const GEN_FLT x43 = ((x9) ? (x42) : (1));
	const GEN_FLT x44 = x40 * x43;
	const GEN_FLT x45 = lh_qk * x41;
	const GEN_FLT x46 = ((x9) ? (x45) : (0));
	const GEN_FLT x47 = lh_qj * x41;
	const GEN_FLT x48 = ((x9) ? (x47) : (0));
	const GEN_FLT x49 = x48 * x7;
	const GEN_FLT x50 = x46 * x49;
	const GEN_FLT x51 = ((x23) ? (x16 * x22) : (0));
	const GEN_FLT x52 = x25 * x31;
	const GEN_FLT x53 = x33 * x37;
	const GEN_FLT x54 = obj_py + sensor_x * (x52 + x53) + sensor_y * (x20 + x21 * x51) + sensor_z * (-x29 + x35);
	const GEN_FLT x55 = x40 * x48;
	const GEN_FLT x56 = x43 * x7;
	const GEN_FLT x57 = x46 * x56;
	const GEN_FLT x58 = ((x23) ? (x15 * x22) : (1));
	const GEN_FLT x59 = obj_px + sensor_x * (x20 + x21 * x58) + sensor_y * (-x52 + x53) + sensor_z * (x36 + x38);
	const GEN_FLT x60 = -lh_pz - x39 * (x14 + x5) - x54 * (x44 + x50) - x59 * (-x55 + x57);
	const GEN_FLT x61 = x12 * x59;
	const GEN_FLT x62 = x55 + x57;
	const GEN_FLT x63 = x39 * x62;
	const GEN_FLT x64 = x40 * x46;
	const GEN_FLT x65 = x48 * x56;
	const GEN_FLT x66 = -x64 + x65;
	const GEN_FLT x67 = x54 * x66;
	const GEN_FLT x68 = lh_px + x61 + x63 + x67;
	const GEN_FLT x69 = x60 * x60;
	const GEN_FLT x70 = x68 * x68 + x69;
	const GEN_FLT x71 = 1.0 / x70;
	const GEN_FLT x72 = x60 * x71;
	const GEN_FLT x73 = x12 * x72;
	const GEN_FLT x74 = x55 - x57;
	const GEN_FLT x75 = -lh_px - x61 - x63 - x67;
	const GEN_FLT x76 = x71 * x75;
	const GEN_FLT x77 = x74 * x76;
	const GEN_FLT x78 = x64 + x65;
	const GEN_FLT x79 = ((x9) ? (x1 * x8) : (0));
	const GEN_FLT x80 = x5 + x7 * x79;
	const GEN_FLT x81 = x54 * x80;
	const GEN_FLT x82 = x59 * x78;
	const GEN_FLT x83 = -x44;
	const GEN_FLT x84 = x50 + x83;
	const GEN_FLT x85 = x39 * x84;
	const GEN_FLT x86 = lh_py + x81 + x82 + x85;
	const GEN_FLT x87 = x86 * x86;
	const GEN_FLT x88 = 2 / (x69 + x87);
	const GEN_FLT x89 = x60 * x88;
	const GEN_FLT x90 = x88 * (-lh_py - x81 - x82 - x85);
	const GEN_FLT x91 = curve_0 * atan2(x86, x60);
	const GEN_FLT x92 = pow(-x71 * x87 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x93 = tilt_0 / sqrt(x70);
	const GEN_FLT x94 = 2 * x5;
	const GEN_FLT x95 = (1.0 / 2.0) * x68;
	const GEN_FLT x96 = 2 * x55;
	const GEN_FLT x97 = 2 * x57;
	const GEN_FLT x98 = (1.0 / 2.0) * x60;
	const GEN_FLT x99 = tilt_0 * x86 / pow(x70, 3.0 / 2.0);
	const GEN_FLT x100 = x92 * (x78 * x93 + x99 * (-x95 * (2 * x11 + x94) - x98 * (x96 - x97)));
	const GEN_FLT x101 = gibMag_0 * sin(-gibPhase_0 + phase_0 + asin(x86 * x93) + atan2(x68, x60) - 1.5707963267948966);
	const GEN_FLT x102 = x66 * x72;
	const GEN_FLT x103 = -x50 + x83;
	const GEN_FLT x104 = x103 * x76;
	const GEN_FLT x105 = x92 * (x80 * x93 + x99 * (-x95 * (-2 * x64 + 2 * x65) - x98 * (-2 * x44 - 2 * x50)));
	const GEN_FLT x106 = -x14 + x6;
	const GEN_FLT x107 = x106 * x76;
	const GEN_FLT x108 = x62 * x72;
	const GEN_FLT x109 = x92 * (x84 * x93 + x99 * (-x95 * (x96 + x97) - x98 * (-2 * x14 - x94)));
	const GEN_FLT x110 = x25 * x27;
	const GEN_FLT x111 = -x110;
	const GEN_FLT x112 = 2 * x22;
	const GEN_FLT x113 = 2 / ((x18 * x18));
	const GEN_FLT x114 = pow(x18, -3.0 / 2.0);
	const GEN_FLT x115 = obj_qi * x114;
	const GEN_FLT x116 = ((x23) ? (-obj_qj * x115) : (0));
	const GEN_FLT x117 = x116 * x25;
	const GEN_FLT x118 = x20 * x27;
	const GEN_FLT x119 = x118 * x33;
	const GEN_FLT x120 = ((x23) ? (-obj_qk * x115) : (0));
	const GEN_FLT x121 = ((x23) ? (-x114 * x15 + x26) : (0));
	const GEN_FLT x122 = x21 * x31;
	const GEN_FLT x123 = x27 * x28;
	const GEN_FLT x124 = x120 * x37 + x121 * x122 + x123 * x52;
	const GEN_FLT x125 = x120 * x25;
	const GEN_FLT x126 = -x125;
	const GEN_FLT x127 = x118 * x31;
	const GEN_FLT x128 = x116 * x37 + x121 * x34 + x123 * x36;
	const GEN_FLT x129 =
		sensor_x * (x110 * x58 + x111 + x21 * ((x23) ? (obj_qi * x112 - x113 * obj_qi * (obj_qi * obj_qi)) : (0))) +
		sensor_y * (x126 - x127 + x128) + sensor_z * (x117 + x119 + x124);
	const GEN_FLT x130 = x12 * x129;
	const GEN_FLT x131 = obj_qi * x113;
	const GEN_FLT x132 = x121 * x25;
	const GEN_FLT x133 = x118 * x28;
	const GEN_FLT x134 = x120 * x34;
	const GEN_FLT x135 = x116 * x122;
	const GEN_FLT x136 = x33 * x52;
	const GEN_FLT x137 = x134 + x135 + x136 * x27;
	const GEN_FLT x138 = -x117;
	const GEN_FLT x139 = sensor_x * (-x119 + x124 + x138) + sensor_y * (x132 + x133 + x137) +
						 sensor_z * (x110 * x24 + x111 + x21 * ((x23) ? (-x131 * x17) : (0)));
	const GEN_FLT x140 = x139 * x62;
	const GEN_FLT x141 = sensor_x * (x125 + x127 + x128) +
						 sensor_y * (x110 * x51 + x111 + x21 * ((x23) ? (-x131 * x16) : (0))) +
						 sensor_z * (-x132 - x133 + x137);
	const GEN_FLT x142 = x141 * x66;
	const GEN_FLT x143 = x72 * (x130 + x140 + x142);
	const GEN_FLT x144 = x106 * x139;
	const GEN_FLT x145 = x103 * x141;
	const GEN_FLT x146 = x129 * x74;
	const GEN_FLT x147 = x144 + x145 + x146;
	const GEN_FLT x148 = x147 * x76;
	const GEN_FLT x149 = x129 * x78 + x139 * x84 + x141 * x80;
	const GEN_FLT x150 =
		x92 * (x149 * x93 + x99 * (-x95 * (2 * x130 + 2 * x140 + 2 * x142) - x98 * (2 * x144 + 2 * x145 + 2 * x146)));
	const GEN_FLT x151 = x25 * x32;
	const GEN_FLT x152 = -x151;
	const GEN_FLT x153 = obj_qj * x113;
	const GEN_FLT x154 = ((x23) ? (-x114 * x16 + x26) : (0));
	const GEN_FLT x155 = x154 * x25;
	const GEN_FLT x156 = x20 * x32;
	const GEN_FLT x157 = x156 * x33;
	const GEN_FLT x158 = ((x23) ? (-obj_qj * obj_qk * x114) : (0));
	const GEN_FLT x159 = x158 * x37;
	const GEN_FLT x160 = x28 * x32;
	const GEN_FLT x161 = x135 + x159 + x160 * x52;
	const GEN_FLT x162 = x158 * x25;
	const GEN_FLT x163 = -x162;
	const GEN_FLT x164 = x156 * x31;
	const GEN_FLT x165 = x116 * x34 + x154 * x37 + x160 * x36;
	const GEN_FLT x166 = sensor_x * (x151 * x58 + x152 + x21 * ((x23) ? (-x15 * x153) : (0))) +
						 sensor_y * (x163 - x164 + x165) + sensor_z * (x155 + x157 + x161);
	const GEN_FLT x167 = x12 * x166;
	const GEN_FLT x168 = x156 * x28;
	const GEN_FLT x169 = x122 * x154 + x136 * x32 + x158 * x34;
	const GEN_FLT x170 = sensor_x * (-x155 - x157 + x161) + sensor_y * (x117 + x168 + x169) +
						 sensor_z * (x151 * x24 + x152 + x21 * ((x23) ? (-x153 * x17) : (0)));
	const GEN_FLT x171 = x170 * x62;
	const GEN_FLT x172 =
		sensor_x * (x162 + x164 + x165) +
		sensor_y * (x151 * x51 + x152 + x21 * ((x23) ? (obj_qj * x112 - x113 * obj_qj * (obj_qj * obj_qj)) : (0))) +
		sensor_z * (x138 - x168 + x169);
	const GEN_FLT x173 = x172 * x66;
	const GEN_FLT x174 = x72 * (x167 + x171 + x173);
	const GEN_FLT x175 = x106 * x170;
	const GEN_FLT x176 = x166 * x74;
	const GEN_FLT x177 = x103 * x172;
	const GEN_FLT x178 = x175 + x176 + x177;
	const GEN_FLT x179 = x178 * x76;
	const GEN_FLT x180 = x166 * x78 + x170 * x84 + x172 * x80;
	const GEN_FLT x181 =
		x92 * (x180 * x93 + x99 * (-x95 * (2 * x167 + 2 * x171 + 2 * x173) - x98 * (2 * x175 + 2 * x176 + 2 * x177)));
	const GEN_FLT x182 = x25 * x30;
	const GEN_FLT x183 = -x182;
	const GEN_FLT x184 = obj_qk * x113;
	const GEN_FLT x185 = x20 * x30;
	const GEN_FLT x186 = x185 * x33;
	const GEN_FLT x187 = ((x23) ? (-x114 * x17 + x26) : (0));
	const GEN_FLT x188 = x28 * x30;
	const GEN_FLT x189 = x120 * x122 + x187 * x37 + x188 * x52;
	const GEN_FLT x190 = x187 * x25;
	const GEN_FLT x191 = x185 * x31;
	const GEN_FLT x192 = x134 + x159 + x188 * x36;
	const GEN_FLT x193 = sensor_x * (x182 * x58 + x183 + x21 * ((x23) ? (-x15 * x184) : (0))) +
						 sensor_y * (-x190 - x191 + x192) + sensor_z * (x162 + x186 + x189);
	const GEN_FLT x194 = x12 * x193;
	const GEN_FLT x195 = x185 * x28;
	const GEN_FLT x196 = x122 * x158 + x136 * x30 + x187 * x34;
	const GEN_FLT x197 = sensor_x * (x190 + x191 + x192) +
						 sensor_y * (x182 * x51 + x183 + x21 * ((x23) ? (-x16 * x184) : (0))) +
						 sensor_z * (x126 - x195 + x196);
	const GEN_FLT x198 = x197 * x66;
	const GEN_FLT x199 =
		sensor_x * (x163 - x186 + x189) + sensor_y * (x125 + x195 + x196) +
		sensor_z * (x182 * x24 + x183 + x21 * ((x23) ? (obj_qk * x112 - x113 * obj_qk * (obj_qk * obj_qk)) : (0)));
	const GEN_FLT x200 = x199 * x62;
	const GEN_FLT x201 = x72 * (x194 + x198 + x200);
	const GEN_FLT x202 = x106 * x199;
	const GEN_FLT x203 = x193 * x74;
	const GEN_FLT x204 = x103 * x197;
	const GEN_FLT x205 = x202 + x203 + x204;
	const GEN_FLT x206 = x205 * x76;
	const GEN_FLT x207 = x193 * x78 + x197 * x80 + x199 * x84;
	const GEN_FLT x208 =
		x92 * (x207 * x93 + x99 * (-x95 * (2 * x194 + 2 * x198 + 2 * x200) - x98 * (2 * x202 + 2 * x203 + 2 * x204)));
	const GEN_FLT x209 = x92 * x99;
	const GEN_FLT x210 = x209 * x75;
	const GEN_FLT x211 = x92 * x93;
	const GEN_FLT x212 = x209 * x60;
	const GEN_FLT x213 = x40 * x42;
	const GEN_FLT x214 = 2 / ((x3 * x3));
	const GEN_FLT x215 = lh_qi * x214;
	const GEN_FLT x216 = x39 * (-x13 * x213 + x213 - x7 * ((x9) ? (-x2 * x215) : (0)));
	const GEN_FLT x217 = pow(x3, -3.0 / 2.0);
	const GEN_FLT x218 = lh_qi * x217;
	const GEN_FLT x219 = ((x9) ? (-lh_qk * x218) : (0));
	const GEN_FLT x220 = x219 * x56;
	const GEN_FLT x221 = ((x9) ? (-x0 * x217 + x41) : (0));
	const GEN_FLT x222 = x46 * x7;
	const GEN_FLT x223 = x221 * x222;
	const GEN_FLT x224 = x43 * x64;
	const GEN_FLT x225 = x224 * x42;
	const GEN_FLT x226 = ((x9) ? (-lh_qj * x218) : (0));
	const GEN_FLT x227 = x226 * x40;
	const GEN_FLT x228 = x42 * x5;
	const GEN_FLT x229 = x227 + x228 * x48;
	const GEN_FLT x230 = x59 * (-x220 - x223 - x225 + x229);
	const GEN_FLT x231 = x219 * x49;
	const GEN_FLT x232 = x222 * x226;
	const GEN_FLT x233 = -x232;
	const GEN_FLT x234 = x42 * x55;
	const GEN_FLT x235 = x234 * x46;
	const GEN_FLT x236 = -x221 * x40 - x228 * x43;
	const GEN_FLT x237 = x54 * (-x231 + x233 - x235 + x236);
	const GEN_FLT x238 = x216 + x230 + x237;
	const GEN_FLT x239 = x238 * x76;
	const GEN_FLT x240 = -x213;
	const GEN_FLT x241 = 2 * x8;
	const GEN_FLT x242 =
		x59 * (x10 * x213 + x240 + x7 * ((x9) ? (lh_qi * x241 - x214 * lh_qi * (lh_qi * lh_qi)) : (0)));
	const GEN_FLT x243 = x39 * (x220 + x223 + x225 + x229);
	const GEN_FLT x244 = x219 * x40;
	const GEN_FLT x245 = -x244;
	const GEN_FLT x246 = x228 * x46;
	const GEN_FLT x247 = x221 * x49 + x226 * x56 + x234 * x43;
	const GEN_FLT x248 = x54 * (x245 - x246 + x247);
	const GEN_FLT x249 = x72 * (x242 + x243 + x248);
	const GEN_FLT x250 = x39 * (x231 + x232 + x235 + x236) +
						 x54 * (x213 * x79 + x240 + x7 * ((x9) ? (-x1 * x215) : (0))) + x59 * (x244 + x246 + x247);
	const GEN_FLT x251 =
		x92 * (x250 * x93 + x99 * (-x95 * (2 * x242 + 2 * x243 + 2 * x248) - x98 * (2 * x216 + 2 * x230 + 2 * x237)));
	const GEN_FLT x252 = x40 * x47;
	const GEN_FLT x253 = -x252;
	const GEN_FLT x254 = lh_qj * x214;
	const GEN_FLT x255 = x59 * (x10 * x252 + x253 + x7 * ((x9) ? (-x0 * x254) : (0)));
	const GEN_FLT x256 = ((x9) ? (-lh_qj * lh_qk * x217) : (0));
	const GEN_FLT x257 = x256 * x56;
	const GEN_FLT x258 = x224 * x47;
	const GEN_FLT x259 = ((x9) ? (-x1 * x217 + x41) : (0));
	const GEN_FLT x260 = x47 * x5;
	const GEN_FLT x261 = x259 * x40 + x260 * x48;
	const GEN_FLT x262 = x39 * (x232 + x257 + x258 + x261);
	const GEN_FLT x263 = x256 * x40;
	const GEN_FLT x264 = x260 * x46;
	const GEN_FLT x265 = x47 * x55;
	const GEN_FLT x266 = x226 * x49 + x259 * x56 + x265 * x43;
	const GEN_FLT x267 = x54 * (-x263 - x264 + x266);
	const GEN_FLT x268 = x72 * (x255 + x262 + x267);
	const GEN_FLT x269 = x39 * (-x13 * x252 + x252 - x7 * ((x9) ? (-x2 * x254) : (0)));
	const GEN_FLT x270 = x59 * (x233 - x257 - x258 + x261);
	const GEN_FLT x271 = x256 * x49;
	const GEN_FLT x272 = x222 * x259;
	const GEN_FLT x273 = x265 * x46;
	const GEN_FLT x274 = -x227 - x260 * x43;
	const GEN_FLT x275 = x54 * (-x271 - x272 - x273 + x274);
	const GEN_FLT x276 = x269 + x270 + x275;
	const GEN_FLT x277 = x276 * x76;
	const GEN_FLT x278 =
		x39 * (x271 + x272 + x273 + x274) +
		x54 * (x252 * x79 + x253 + x7 * ((x9) ? (lh_qj * x241 - x214 * lh_qj * (lh_qj * lh_qj)) : (0))) +
		x59 * (x263 + x264 + x266);
	const GEN_FLT x279 =
		x92 * (x278 * x93 + x99 * (-x95 * (2 * x255 + 2 * x262 + 2 * x267) - x98 * (2 * x269 + 2 * x270 + 2 * x275)));
	const GEN_FLT x280 = x40 * x45;
	const GEN_FLT x281 = -x280;
	const GEN_FLT x282 = lh_qk * x214;
	const GEN_FLT x283 = x59 * (x10 * x280 + x281 + x7 * ((x9) ? (-x0 * x282) : (0)));
	const GEN_FLT x284 = ((x9) ? (-x2 * x217 + x41) : (0));
	const GEN_FLT x285 = x284 * x56;
	const GEN_FLT x286 = x219 * x222;
	const GEN_FLT x287 = x224 * x45;
	const GEN_FLT x288 = x45 * x5;
	const GEN_FLT x289 = x263 + x288 * x48;
	const GEN_FLT x290 = x39 * (x285 + x286 + x287 + x289);
	const GEN_FLT x291 = x284 * x40;
	const GEN_FLT x292 = x288 * x46;
	const GEN_FLT x293 = x45 * x55;
	const GEN_FLT x294 = x231 + x257 + x293 * x43;
	const GEN_FLT x295 = x54 * (-x291 - x292 + x294);
	const GEN_FLT x296 = x72 * (x283 + x290 + x295);
	const GEN_FLT x297 =
		x39 * (-x13 * x280 + x280 - x7 * ((x9) ? (lh_qk * x241 - x214 * lh_qk * (lh_qk * lh_qk)) : (0)));
	const GEN_FLT x298 = x59 * (-x285 - x286 - x287 + x289);
	const GEN_FLT x299 = x284 * x49;
	const GEN_FLT x300 = x222 * x256;
	const GEN_FLT x301 = x293 * x46;
	const GEN_FLT x302 = x245 - x288 * x43;
	const GEN_FLT x303 = x54 * (-x299 - x300 - x301 + x302);
	const GEN_FLT x304 = x297 + x298 + x303;
	const GEN_FLT x305 = x304 * x76;
	const GEN_FLT x306 = x39 * (x299 + x300 + x301 + x302) +
						 x54 * (x280 * x79 + x281 + x7 * ((x9) ? (-x1 * x282) : (0))) + x59 * (x291 + x292 + x294);
	const GEN_FLT x307 =
		x92 * (x306 * x93 + x99 * (-x95 * (2 * x283 + 2 * x290 + 2 * x295) - x98 * (2 * x297 + 2 * x298 + 2 * x303)));
	*(out++) = -x100 + x101 * (x100 + x73 + x77) - x73 - x77 + x91 * (x74 * x90 + x78 * x89);
	*(out++) = x101 * (x102 + x104 + x105) - x102 - x104 - x105 + x91 * (x103 * x90 + x80 * x89);
	*(out++) = x101 * (x107 + x108 + x109) - x107 - x108 - x109 + x91 * (x106 * x90 + x84 * x89);
	*(out++) = x101 * (x143 + x148 + x150) - x143 - x148 - x150 + x91 * (x147 * x90 + x149 * x89);
	*(out++) = x101 * (x174 + x179 + x181) - x174 - x179 - x181 + x91 * (x178 * x90 + x180 * x89);
	*(out++) = x101 * (x201 + x206 + x208) - x201 - x206 - x208 + x91 * (x205 * x90 + x207 * x89);
	*(out++) = x101 * (x210 + x72) - x210 - x72;
	*(out++) = x101 * x211 - x211 + x89 * x91;
	*(out++) = x101 * (x212 - x76) - x212 + x76 - x90 * x91;
	*(out++) = x101 * (x239 + x249 + x251) - x239 - x249 - x251 + x91 * (x238 * x90 + x250 * x89);
	*(out++) = x101 * (x268 + x277 + x279) - x268 - x277 - x279 + x91 * (x276 * x90 + x278 * x89);
	*(out++) = x101 * (x296 + x305 + x307) - x296 - x305 - x307 + x91 * (x304 * x90 + x306 * x89);
}

static inline void gen_reproject_axisangle_axis_y_jac_all(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
														  const FLT phase_0, const FLT tilt_0, const FLT curve_0,
														  const FLT gibPhase_0, const FLT gibMag_0) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = sin(x4);
	const GEN_FLT x6 = 1.0 / x4;
	const GEN_FLT x7 = lh_qj * x6;
	const GEN_FLT x8 = x4 > 0;
	const GEN_FLT x9 = ((x8) ? (x7) : (0));
	const GEN_FLT x10 = x5 * x9;
	const GEN_FLT x11 = lh_qk * x6;
	const GEN_FLT x12 = ((x8) ? (x11) : (0));
	const GEN_FLT x13 = cos(x4);
	const GEN_FLT x14 = -x13;
	const GEN_FLT x15 = x14 + 1;
	const GEN_FLT x16 = lh_qi * x6;
	const GEN_FLT x17 = ((x8) ? (x16) : (1));
	const GEN_FLT x18 = x15 * x17;
	const GEN_FLT x19 = x12 * x18;
	const GEN_FLT x20 = x10 - x19;
	const GEN_FLT x21 = 1.0 / x3;
	const GEN_FLT x22 = ((x8) ? (x1 * x21) : (0));
	const GEN_FLT x23 = x15 * x22;
	const GEN_FLT x24 = obj_qi * obj_qi;
	const GEN_FLT x25 = obj_qj * obj_qj;
	const GEN_FLT x26 = obj_qk * obj_qk;
	const GEN_FLT x27 = x24 + x25 + x26;
	const GEN_FLT x28 = sqrt(x27);
	const GEN_FLT x29 = cos(x28);
	const GEN_FLT x30 = 1 - x29;
	const GEN_FLT x31 = 1.0 / x27;
	const GEN_FLT x32 = x28 > 0;
	const GEN_FLT x33 = ((x32) ? (x25 * x31) : (0));
	const GEN_FLT x34 = sin(x28);
	const GEN_FLT x35 = 1.0 / x28;
	const GEN_FLT x36 = obj_qk * x35;
	const GEN_FLT x37 = ((x32) ? (x36) : (0));
	const GEN_FLT x38 = x34 * x37;
	const GEN_FLT x39 = obj_qi * x35;
	const GEN_FLT x40 = ((x32) ? (x39) : (1));
	const GEN_FLT x41 = obj_qj * x35;
	const GEN_FLT x42 = ((x32) ? (x41) : (0));
	const GEN_FLT x43 = x30 * x42;
	const GEN_FLT x44 = x40 * x43;
	const GEN_FLT x45 = x34 * x40;
	const GEN_FLT x46 = x37 * x43;
	const GEN_FLT x47 = obj_py + sensor_x * (x38 + x44) + sensor_y * (x29 + x30 * x33) + sensor_z * (-x45 + x46);
	const GEN_FLT x48 = x47 * (x13 + x23);
	const GEN_FLT x49 = x12 * x5;
	const GEN_FLT x50 = x15 * x9;
	const GEN_FLT x51 = x17 * x50;
	const GEN_FLT x52 = ((x32) ? (x24 * x31) : (1));
	const GEN_FLT x53 = x34 * x42;
	const GEN_FLT x54 = x30 * x40;
	const GEN_FLT x55 = x37 * x54;
	const GEN_FLT x56 = obj_px + sensor_x * (x29 + x30 * x52) + sensor_y * (-x38 + x44) + sensor_z * (x53 + x55);
	const GEN_FLT x57 = x56 * (x49 + x51);
	const GEN_FLT x58 = x17 * x5;
	const GEN_FLT x59 = -x58;
	const GEN_FLT x60 = x12 * x50;
	const GEN_FLT x61 = ((x32) ? (x26 * x31) : (0));
	const GEN_FLT x62 = obj_pz + sensor_x * (-x53 + x55) + sensor_y * (x45 + x46) + sensor_z * (x29 + x30 * x61);
	const GEN_FLT x63 = x62 * (x59 + x60);
	const GEN_FLT x64 = -lh_py - x48 - x57 - x63;
	const GEN_FLT x65 = ((x8) ? (x2 * x21) : (0));
	const GEN_FLT x66 = x15 * x65;
	const GEN_FLT x67 = -lh_pz - x47 * (x58 + x60) - x56 * (-x10 + x19) - x62 * (x13 + x66);
	const GEN_FLT x68 = x67 * x67;
	const GEN_FLT x69 = x64 * x64 + x68;
	const GEN_FLT x70 = 1.0 / x69;
	const GEN_FLT x71 = x70 * (lh_py + x48 + x57 + x63);
	const GEN_FLT x72 = x20 * x71;
	const GEN_FLT x73 = -x49;
	const GEN_FLT x74 = -x51 + x73;
	const GEN_FLT x75 = x67 * x70;
	const GEN_FLT x76 = x74 * x75;
	const GEN_FLT x77 = ((x8) ? (x0 * x21) : (1));
	const GEN_FLT x78 = x13 + x15 * x77;
	const GEN_FLT x79 = x56 * x78;
	const GEN_FLT x80 = x10 + x19;
	const GEN_FLT x81 = x62 * x80;
	const GEN_FLT x82 = x51 + x73;
	const GEN_FLT x83 = x47 * x82;
	const GEN_FLT x84 = lh_px + x79 + x81 + x83;
	const GEN_FLT x85 = x84 * x84;
	const GEN_FLT x86 = 2 / (x68 + x85);
	const GEN_FLT x87 = x67 * x86;
	const GEN_FLT x88 = x86 * (-lh_px - x79 - x81 - x83);
	const GEN_FLT x89 = curve_0 * atan2(x84, x67);
	const GEN_FLT x90 = pow(-x70 * x85 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x91 = tilt_0 / sqrt(x69);
	const GEN_FLT x92 = (1.0 / 2.0) * x64;
	const GEN_FLT x93 = (1.0 / 2.0) * x67;
	const GEN_FLT x94 = tilt_0 * x84 / pow(x69, 3.0 / 2.0);
	const GEN_FLT x95 = x90 * (x78 * x91 + x94 * (-x92 * (-2 * x49 - 2 * x51) - x93 * (2 * x10 - 2 * x19)));
	const GEN_FLT x96 = gibMag_0 * sin(-gibPhase_0 + phase_0 + asin(x84 * x91) + atan2(x64, x67) - 1.5707963267948966);
	const GEN_FLT x97 = x14 - x23;
	const GEN_FLT x98 = x75 * x97;
	const GEN_FLT x99 = -x60;
	const GEN_FLT x100 = x59 + x99;
	const GEN_FLT x101 = x100 * x71;
	const GEN_FLT x102 = -2 * x13;
	const GEN_FLT x103 = 2 * x58;
	const GEN_FLT x104 = -2 * x60;
	const GEN_FLT x105 = x90 * (x82 * x91 + x94 * (-x92 * (x102 - 2 * x23) - x93 * (-x103 + x104)));
	const GEN_FLT x106 = x14 - x66;
	const GEN_FLT x107 = x106 * x71;
	const GEN_FLT x108 = x58 + x99;
	const GEN_FLT x109 = x108 * x75;
	const GEN_FLT x110 = x90 * (x80 * x91 + x94 * (-x92 * (x103 + x104) - x93 * (x102 - 2 * x66)));
	const GEN_FLT x111 = x34 * x39;
	const GEN_FLT x112 = -x111;
	const GEN_FLT x113 = 2 / ((x27 * x27));
	const GEN_FLT x114 = obj_qi * x113;
	const GEN_FLT x115 = pow(x27, -3.0 / 2.0);
	const GEN_FLT x116 = ((x32) ? (-x115 * x24 + x35) : (0));
	const GEN_FLT x117 = x116 * x34;
	const GEN_FLT x118 = x29 * x39;
	const GEN_FLT x119 = x118 * x40;
	const GEN_FLT x120 = obj_qi * x115;
	const GEN_FLT x121 = ((x32) ? (-obj_qk * x120) : (0));
	const GEN_FLT x122 = x121 * x43;
	const GEN_FLT x123 = ((x32) ? (-obj_qj * x120) : (0));
	const GEN_FLT x124 = x30 * x37;
	const GEN_FLT x125 = x123 * x124;
	const GEN_FLT x126 = x39 * x42;
	const GEN_FLT x127 = x122 + x125 + x126 * x38;
	const GEN_FLT x128 = x123 * x34;
	const GEN_FLT x129 = -x128;
	const GEN_FLT x130 = x118 * x42;
	const GEN_FLT x131 = x38 * x40;
	const GEN_FLT x132 = x116 * x124 + x121 * x54 + x131 * x39;
	const GEN_FLT x133 = sensor_x * (x129 - x130 + x132) + sensor_y * (x117 + x119 + x127) +
						 sensor_z * (x111 * x61 + x112 + x30 * ((x32) ? (-x114 * x26) : (0)));
	const GEN_FLT x134 = x106 * x133;
	const GEN_FLT x135 = x121 * x34;
	const GEN_FLT x136 = x118 * x37;
	const GEN_FLT x137 = x116 * x43 + x123 * x54 + x126 * x45;
	const GEN_FLT x138 = sensor_x * (x135 + x136 + x137) +
						 sensor_y * (x111 * x33 + x112 + x30 * ((x32) ? (-x114 * x25) : (0))) +
						 sensor_z * (-x117 - x119 + x127);
	const GEN_FLT x139 = x100 * x138;
	const GEN_FLT x140 = 2 * x31;
	const GEN_FLT x141 = -x135;
	const GEN_FLT x142 =
		sensor_x * (x111 * x52 + x112 + x30 * ((x32) ? (obj_qi * x140 - x113 * obj_qi * (obj_qi * obj_qi)) : (0))) +
		sensor_y * (-x136 + x137 + x141) + sensor_z * (x128 + x130 + x132);
	const GEN_FLT x143 = x142 * x20;
	const GEN_FLT x144 = x134 + x139 + x143;
	const GEN_FLT x145 = x144 * x71;
	const GEN_FLT x146 = x138 * x97;
	const GEN_FLT x147 = x108 * x133;
	const GEN_FLT x148 = x142 * x74;
	const GEN_FLT x149 = x75 * (x146 + x147 + x148);
	const GEN_FLT x150 = x133 * x80 + x138 * x82 + x142 * x78;
	const GEN_FLT x151 =
		x90 * (x150 * x91 + x94 * (-x92 * (2 * x146 + 2 * x147 + 2 * x148) - x93 * (2 * x134 + 2 * x139 + 2 * x143)));
	const GEN_FLT x152 = x34 * x41;
	const GEN_FLT x153 = -x152;
	const GEN_FLT x154 = obj_qj * x113;
	const GEN_FLT x155 = x29 * x41;
	const GEN_FLT x156 = x155 * x40;
	const GEN_FLT x157 = ((x32) ? (-obj_qj * obj_qk * x115) : (0));
	const GEN_FLT x158 = ((x32) ? (-x115 * x25 + x35) : (0));
	const GEN_FLT x159 = x41 * x42;
	const GEN_FLT x160 = x124 * x158 + x157 * x43 + x159 * x38;
	const GEN_FLT x161 = x158 * x34;
	const GEN_FLT x162 = x155 * x42;
	const GEN_FLT x163 = x157 * x54;
	const GEN_FLT x164 = x125 + x131 * x41 + x163;
	const GEN_FLT x165 = sensor_x * (-x161 - x162 + x164) + sensor_y * (x128 + x156 + x160) +
						 sensor_z * (x152 * x61 + x153 + x30 * ((x32) ? (-x154 * x26) : (0)));
	const GEN_FLT x166 = x106 * x165;
	const GEN_FLT x167 = x157 * x34;
	const GEN_FLT x168 = -x167;
	const GEN_FLT x169 = x155 * x37;
	const GEN_FLT x170 = x123 * x43 + x158 * x54 + x159 * x45;
	const GEN_FLT x171 = sensor_x * (x152 * x52 + x153 + x30 * ((x32) ? (-x154 * x24) : (0))) +
						 sensor_y * (x168 - x169 + x170) + sensor_z * (x161 + x162 + x164);
	const GEN_FLT x172 = x171 * x20;
	const GEN_FLT x173 =
		sensor_x * (x167 + x169 + x170) +
		sensor_y * (x152 * x33 + x153 + x30 * ((x32) ? (obj_qj * x140 - x113 * obj_qj * (obj_qj * obj_qj)) : (0))) +
		sensor_z * (x129 - x156 + x160);
	const GEN_FLT x174 = x100 * x173;
	const GEN_FLT x175 = x166 + x172 + x174;
	const GEN_FLT x176 = x175 * x71;
	const GEN_FLT x177 = x173 * x97;
	const GEN_FLT x178 = x108 * x165;
	const GEN_FLT x179 = x171 * x74;
	const GEN_FLT x180 = x75 * (x177 + x178 + x179);
	const GEN_FLT x181 = x165 * x80 + x171 * x78 + x173 * x82;
	const GEN_FLT x182 =
		x90 * (x181 * x91 + x94 * (-x92 * (2 * x177 + 2 * x178 + 2 * x179) - x93 * (2 * x166 + 2 * x172 + 2 * x174)));
	const GEN_FLT x183 = x34 * x36;
	const GEN_FLT x184 = -x183;
	const GEN_FLT x185 = x29 * x36;
	const GEN_FLT x186 = x185 * x40;
	const GEN_FLT x187 = ((x32) ? (-x115 * x26 + x35) : (0));
	const GEN_FLT x188 = x36 * x42;
	const GEN_FLT x189 = x124 * x157 + x187 * x43 + x188 * x38;
	const GEN_FLT x190 = x185 * x42;
	const GEN_FLT x191 = x121 * x124 + x131 * x36 + x187 * x54;
	const GEN_FLT x192 =
		sensor_x * (x168 - x190 + x191) + sensor_y * (x135 + x186 + x189) +
		sensor_z * (x183 * x61 + x184 + x30 * ((x32) ? (obj_qk * x140 - x113 * obj_qk * (obj_qk * obj_qk)) : (0)));
	const GEN_FLT x193 = x106 * x192;
	const GEN_FLT x194 = obj_qk * x113;
	const GEN_FLT x195 = x187 * x34;
	const GEN_FLT x196 = x185 * x37;
	const GEN_FLT x197 = x122 + x163 + x188 * x45;
	const GEN_FLT x198 = sensor_x * (x183 * x52 + x184 + x30 * ((x32) ? (-x194 * x24) : (0))) +
						 sensor_y * (-x195 - x196 + x197) + sensor_z * (x167 + x190 + x191);
	const GEN_FLT x199 = x198 * x20;
	const GEN_FLT x200 = sensor_x * (x195 + x196 + x197) +
						 sensor_y * (x183 * x33 + x184 + x30 * ((x32) ? (-x194 * x25) : (0))) +
						 sensor_z * (x141 - x186 + x189);
	const GEN_FLT x201 = x100 * x200;
	const GEN_FLT x202 = x193 + x199 + x201;
	const GEN_FLT x203 = x202 * x71;
	const GEN_FLT x204 = x200 * x97;
	const GEN_FLT x205 = x198 * x74;
	const GEN_FLT x206 = x108 * x192;
	const GEN_FLT x207 = x75 * (x204 + x205 + x206);
	const GEN_FLT x208 = x192 * x80 + x198 * x78 + x200 * x82;
	const GEN_FLT x209 =
		x90 * (x208 * x91 + x94 * (-x92 * (2 * x204 + 2 * x205 + 2 * x206) - x93 * (2 * x193 + 2 * x199 + 2 * x201)));
	const GEN_FLT x210 = x90 * x91;
	const GEN_FLT x211 = x90 * x94;
	const GEN_FLT x212 = x211 * x64;
	const GEN_FLT x213 = x211 * x67;
	const GEN_FLT x214 = x16 * x5;
	const GEN_FLT x215 = 2 / ((x3 * x3));
	const GEN_FLT x216 = lh_qi * x215;
	const GEN_FLT x217 = x62 * (-x15 * ((x8) ? (-x2 * x216) : (0)) - x214 * x65 + x214);
	const GEN_FLT x218 = pow(x3, -3.0 / 2.0);
	const GEN_FLT x219 = lh_qi * x218;
	const GEN_FLT x220 = ((x8) ? (-lh_qk * x219) : (0));
	const GEN_FLT x221 = x18 * x220;
	const GEN_FLT x222 = ((x8) ? (-x0 * x218 + x6) : (0));
	const GEN_FLT x223 = x12 * x15;
	const GEN_FLT x224 = x222 * x223;
	const GEN_FLT x225 = x17 * x49;
	const GEN_FLT x226 = x16 * x225;
	const GEN_FLT x227 = ((x8) ? (-lh_qj * x219) : (0));
	const GEN_FLT x228 = x227 * x5;
	const GEN_FLT x229 = x13 * x16;
	const GEN_FLT x230 = x228 + x229 * x9;
	const GEN_FLT x231 = x56 * (-x221 - x224 - x226 + x230);
	const GEN_FLT x232 = x222 * x5;
	const GEN_FLT x233 = x17 * x229;
	const GEN_FLT x234 = x220 * x50;
	const GEN_FLT x235 = -x234;
	const GEN_FLT x236 = x223 * x227;
	const GEN_FLT x237 = -x236;
	const GEN_FLT x238 = x16 * x9;
	const GEN_FLT x239 = x235 + x237 - x238 * x49;
	const GEN_FLT x240 = x47 * (-x232 - x233 + x239);
	const GEN_FLT x241 = x217 + x231 + x240;
	const GEN_FLT x242 = x241 * x71;
	const GEN_FLT x243 = x47 * (-x15 * ((x8) ? (-x1 * x216) : (0)) - x214 * x22 + x214);
	const GEN_FLT x244 = x62 * (x232 + x233 + x239);
	const GEN_FLT x245 = x18 * x227;
	const GEN_FLT x246 = x222 * x50;
	const GEN_FLT x247 = x238 * x58;
	const GEN_FLT x248 = x220 * x5;
	const GEN_FLT x249 = -x248;
	const GEN_FLT x250 = -x12 * x229 + x249;
	const GEN_FLT x251 = x56 * (-x245 - x246 - x247 + x250);
	const GEN_FLT x252 = x75 * (x243 + x244 + x251);
	const GEN_FLT x253 = 2 * x21;
	const GEN_FLT x254 =
		x47 * (x245 + x246 + x247 + x250) +
		x56 * (x15 * ((x8) ? (lh_qi * x253 - x215 * lh_qi * (lh_qi * lh_qi)) : (0)) + x214 * x77 - x214) +
		x62 * (x221 + x224 + x226 + x230);
	const GEN_FLT x255 =
		x90 * (x254 * x91 + x94 * (-x92 * (2 * x243 + 2 * x244 + 2 * x251) - x93 * (2 * x217 + 2 * x231 + 2 * x240)));
	const GEN_FLT x256 = x5 * x7;
	const GEN_FLT x257 = lh_qj * x215;
	const GEN_FLT x258 = x62 * (-x15 * ((x8) ? (-x2 * x257) : (0)) - x256 * x65 + x256);
	const GEN_FLT x259 = ((x8) ? (-lh_qj * lh_qk * x218) : (0));
	const GEN_FLT x260 = x18 * x259;
	const GEN_FLT x261 = -x260;
	const GEN_FLT x262 = x225 * x7;
	const GEN_FLT x263 = ((x8) ? (-x1 * x218 + x6) : (0));
	const GEN_FLT x264 = x13 * x7;
	const GEN_FLT x265 = x263 * x5 + x264 * x9;
	const GEN_FLT x266 = x56 * (x237 + x261 - x262 + x265);
	const GEN_FLT x267 = x17 * x264;
	const GEN_FLT x268 = x7 * x9;
	const GEN_FLT x269 = -x223 * x263 - x259 * x50 - x268 * x49;
	const GEN_FLT x270 = x47 * (-x228 - x267 + x269);
	const GEN_FLT x271 = x258 + x266 + x270;
	const GEN_FLT x272 = x271 * x71;
	const GEN_FLT x273 =
		x47 * (-x15 * ((x8) ? (lh_qj * x253 - x215 * lh_qj * (lh_qj * lh_qj)) : (0)) - x22 * x256 + x256);
	const GEN_FLT x274 = x62 * (x228 + x267 + x269);
	const GEN_FLT x275 = x18 * x263;
	const GEN_FLT x276 = x227 * x50;
	const GEN_FLT x277 = x268 * x58;
	const GEN_FLT x278 = x259 * x5;
	const GEN_FLT x279 = -x12 * x264 - x278;
	const GEN_FLT x280 = x56 * (-x275 - x276 - x277 + x279);
	const GEN_FLT x281 = x75 * (x273 + x274 + x280);
	const GEN_FLT x282 = x47 * (x275 + x276 + x277 + x279) +
						 x56 * (x15 * ((x8) ? (-x0 * x257) : (0)) + x256 * x77 - x256) +
						 x62 * (x236 + x260 + x262 + x265);
	const GEN_FLT x283 =
		x90 * (x282 * x91 + x94 * (-x92 * (2 * x273 + 2 * x274 + 2 * x280) - x93 * (2 * x258 + 2 * x266 + 2 * x270)));
	const GEN_FLT x284 = x11 * x5;
	const GEN_FLT x285 = lh_qk * x215;
	const GEN_FLT x286 = x47 * (-x15 * ((x8) ? (-x1 * x285) : (0)) - x22 * x284 + x284);
	const GEN_FLT x287 = x11 * x13;
	const GEN_FLT x288 = x17 * x287;
	const GEN_FLT x289 = ((x8) ? (-x2 * x218 + x6) : (0));
	const GEN_FLT x290 = x11 * x9;
	const GEN_FLT x291 = -x223 * x259 - x289 * x50 - x290 * x49;
	const GEN_FLT x292 = x62 * (x248 + x288 + x291);
	const GEN_FLT x293 = x290 * x58;
	const GEN_FLT x294 = -x12 * x287 - x289 * x5;
	const GEN_FLT x295 = x56 * (x235 + x261 - x293 + x294);
	const GEN_FLT x296 = x75 * (x286 + x292 + x295);
	const GEN_FLT x297 =
		x62 * (-x15 * ((x8) ? (lh_qk * x253 - x215 * lh_qk * (lh_qk * lh_qk)) : (0)) - x284 * x65 + x284);
	const GEN_FLT x298 = x18 * x289;
	const GEN_FLT x299 = x220 * x223;
	const GEN_FLT x300 = x11 * x225;
	const GEN_FLT x301 = x278 + x287 * x9;
	const GEN_FLT x302 = x56 * (-x298 - x299 - x300 + x301);
	const GEN_FLT x303 = x47 * (x249 - x288 + x291);
	const GEN_FLT x304 = x297 + x302 + x303;
	const GEN_FLT x305 = x304 * x71;
	const GEN_FLT x306 = x47 * (x234 + x260 + x293 + x294) +
						 x56 * (x15 * ((x8) ? (-x0 * x285) : (0)) + x284 * x77 - x284) +
						 x62 * (x298 + x299 + x300 + x301);
	const GEN_FLT x307 =
		x90 * (x306 * x91 + x94 * (-x92 * (2 * x286 + 2 * x292 + 2 * x295) - x93 * (2 * x297 + 2 * x302 + 2 * x303)));
	*(out++) = -x72 - x76 + x89 * (x20 * x88 + x78 * x87) - x95 + x96 * (x72 + x76 + x95);
	*(out++) = -x101 - x105 + x89 * (x100 * x88 + x82 * x87) + x96 * (x101 + x105 + x98) - x98;
	*(out++) = -x107 - x109 - x110 + x89 * (x106 * x88 + x80 * x87) + x96 * (x107 + x109 + x110);
	*(out++) = -x145 - x149 - x151 + x89 * (x144 * x88 + x150 * x87) + x96 * (x145 + x149 + x151);
	*(out++) = -x176 - x180 - x182 + x89 * (x175 * x88 + x181 * x87) + x96 * (x176 + x180 + x182);
	*(out++) = -x203 - x207 - x209 + x89 * (x202 * x88 + x208 * x87) + x96 * (x203 + x207 + x209);
	*(out++) = x210 * x96 - x210 + x87 * x89;
	*(out++) = -x212 + x75 + x96 * (x212 - x75);
	*(out++) = -x213 + x71 - x88 * x89 + x96 * (x213 - x71);
	*(out++) = -x242 - x252 - x255 + x89 * (x241 * x88 + x254 * x87) + x96 * (x242 + x252 + x255);
	*(out++) = -x272 - x281 - x283 + x89 * (x271 * x88 + x282 * x87) + x96 * (x272 + x281 + x283);
	*(out++) = -x296 - x305 - x307 + x89 * (x304 * x88 + x306 * x87) + x96 * (x296 + x305 + x307);
}

static inline void gen_reproject_axisangle_jac_lh_p_gen2(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
														 const FLT phase_0, const FLT phase_1, const FLT tilt_0,
														 const FLT tilt_1, const FLT curve_0, const FLT curve_1,
														 const FLT gibPhase_0, const FLT gibPhase_1, const FLT gibMag_0,
														 const FLT gibMag_1, const FLT ogeePhase_0,
														 const FLT ogeePhase_1, const FLT ogeeMag_0,
														 const FLT ogeeMag_1) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = 1 - x5;
	const GEN_FLT x7 = 1.0 / x3;
	const GEN_FLT x8 = x4 > 0;
	const GEN_FLT x9 = ((x8) ? (x2 * x7) : (0));
	const GEN_FLT x10 = obj_qi * obj_qi;
	const GEN_FLT x11 = obj_qj * obj_qj;
	const GEN_FLT x12 = obj_qk * obj_qk;
	const GEN_FLT x13 = x10 + x11 + x12;
	const GEN_FLT x14 = sqrt(x13);
	const GEN_FLT x15 = cos(x14);
	const GEN_FLT x16 = 1 - x15;
	const GEN_FLT x17 = 1.0 / x13;
	const GEN_FLT x18 = x14 > 0;
	const GEN_FLT x19 = sin(x14);
	const GEN_FLT x20 = 1.0 / x14;
	const GEN_FLT x21 = ((x18) ? (obj_qi * x20) : (1));
	const GEN_FLT x22 = x19 * x21;
	const GEN_FLT x23 = ((x18) ? (obj_qj * x20) : (0));
	const GEN_FLT x24 = ((x18) ? (obj_qk * x20) : (0));
	const GEN_FLT x25 = x16 * x23 * x24;
	const GEN_FLT x26 = x19 * x23;
	const GEN_FLT x27 = x16 * x21;
	const GEN_FLT x28 = x24 * x27;
	const GEN_FLT x29 = obj_pz + sensor_x * (-x26 + x28) + sensor_y * (x22 + x25) +
						sensor_z * (x15 + x16 * ((x18) ? (x12 * x17) : (0)));
	const GEN_FLT x30 = x29 * (x5 + x6 * x9);
	const GEN_FLT x31 = sin(x4);
	const GEN_FLT x32 = 1.0 / x4;
	const GEN_FLT x33 = lh_qi * x32;
	const GEN_FLT x34 = ((x8) ? (x33) : (1));
	const GEN_FLT x35 = x31 * x34;
	const GEN_FLT x36 = lh_qk * x32;
	const GEN_FLT x37 = ((x8) ? (x36) : (0));
	const GEN_FLT x38 = lh_qj * x32;
	const GEN_FLT x39 = ((x8) ? (x38) : (0));
	const GEN_FLT x40 = x39 * x6;
	const GEN_FLT x41 = x37 * x40;
	const GEN_FLT x42 = x19 * x24;
	const GEN_FLT x43 = x23 * x27;
	const GEN_FLT x44 = obj_py + sensor_x * (x42 + x43) + sensor_y * (x15 + x16 * ((x18) ? (x11 * x17) : (0))) +
						sensor_z * (-x22 + x25);
	const GEN_FLT x45 = x44 * (x35 + x41);
	const GEN_FLT x46 = x31 * x39;
	const GEN_FLT x47 = x34 * x6;
	const GEN_FLT x48 = x37 * x47;
	const GEN_FLT x49 = obj_px + sensor_x * (x15 + x16 * ((x18) ? (x10 * x17) : (1))) + sensor_y * (-x42 + x43) +
						sensor_z * (x26 + x28);
	const GEN_FLT x50 = x49 * (-x46 + x48);
	const GEN_FLT x51 = ((x8) ? (x0 * x7) : (1));
	const GEN_FLT x52 = x49 * (x5 + x51 * x6);
	const GEN_FLT x53 = x29 * (x46 + x48);
	const GEN_FLT x54 = x31 * x37;
	const GEN_FLT x55 = x39 * x47;
	const GEN_FLT x56 = x44 * (-x54 + x55);
	const GEN_FLT x57 = lh_px + x52 + x53 + x56;
	const GEN_FLT x58 = -lh_pz - x30 - x45 - x50;
	const GEN_FLT x59 = x57 * x57 + x58 * x58;
	const GEN_FLT x60 = 1.0 / x59;
	const GEN_FLT x61 = x60 * (lh_pz + x30 + x45 + x50);
	const GEN_FLT x62 = ((x8) ? (x1 * x7) : (0));
	const GEN_FLT x63 = x44 * (x5 + x6 * x62);
	const GEN_FLT x64 = x49 * (x54 + x55);
	const GEN_FLT x65 = x29 * (-x35 + x41);
	const GEN_FLT x66 = lh_py + x63 + x64 + x65;
	const GEN_FLT x67 = tilt_0 + 0.52359877559829882;
	const GEN_FLT x68 = tan(x67);
	const GEN_FLT x69 = pow(x59, -1.0 / 2.0);
	const GEN_FLT x70 = x68 * x69;
	const GEN_FLT x71 = x66 * x70;
	const GEN_FLT x72 = cos(x67);
	const GEN_FLT x73 = 1.0 / x72;
	const GEN_FLT x74 = x66 * x66;
	const GEN_FLT x75 = x59 + x74;
	const GEN_FLT x76 = pow(x75, -1.0 / 2.0);
	const GEN_FLT x77 = x73 * x76;
	const GEN_FLT x78 = asin(x66 * x77);
	const GEN_FLT x79 = 8.0108022e-6 * x78;
	const GEN_FLT x80 = -x79 - 8.0108022e-6;
	const GEN_FLT x81 = x78 * x80 + 0.0028679863;
	const GEN_FLT x82 = x78 * x81 + 5.3685255000000001e-6;
	const GEN_FLT x83 = x78 * x82 + 0.0076069798000000001;
	const GEN_FLT x84 = x78 * x78;
	const GEN_FLT x85 = atan2(x58, x57);
	const GEN_FLT x86 = ogeePhase_0 + x85 - asin(x71);
	const GEN_FLT x87 = ogeeMag_0 * sin(x86);
	const GEN_FLT x88 = curve_0 + x87;
	const GEN_FLT x89 = x78 * x83;
	const GEN_FLT x90 = -1.60216044e-5 * x78 - 8.0108022e-6;
	const GEN_FLT x91 = x78 * x90 + x81;
	const GEN_FLT x92 = x78 * x91 + x82;
	const GEN_FLT x93 = x78 * x92 + x83;
	const GEN_FLT x94 = sin(x67);
	const GEN_FLT x95 = x94 * (x78 * x93 + x89);
	const GEN_FLT x96 = x72 - x88 * x95;
	const GEN_FLT x97 = 1.0 / x96;
	const GEN_FLT x98 = x88 * x97;
	const GEN_FLT x99 = x84 * x98;
	const GEN_FLT x100 = x71 + x83 * x99;
	const GEN_FLT x101 = pow(1 - x100 * x100, -1.0 / 2.0);
	const GEN_FLT x102 = -lh_px - x52 - x53 - x56;
	const GEN_FLT x103 = x66 / pow(x59, 3.0 / 2.0);
	const GEN_FLT x104 = x103 * x68;
	const GEN_FLT x105 = x102 * x104;
	const GEN_FLT x106 = x74 / x75;
	const GEN_FLT x107 = pow(-x106 / (x72 * x72) + 1, -1.0 / 2.0);
	const GEN_FLT x108 = x66 / pow(x75, 3.0 / 2.0);
	const GEN_FLT x109 = x108 * x73;
	const GEN_FLT x110 = x107 * x109;
	const GEN_FLT x111 = x102 * x110;
	const GEN_FLT x112 = 2 * x89 * x98;
	const GEN_FLT x113 = x60 * x74;
	const GEN_FLT x114 = pow(-x113 * x68 * x68 + 1, -1.0 / 2.0);
	const GEN_FLT x115 = -x105 * x114 + x61;
	const GEN_FLT x116 = ogeeMag_0 * cos(x86);
	const GEN_FLT x117 = x83 * x84;
	const GEN_FLT x118 = x116 * x117 * x97;
	const GEN_FLT x119 = x111 * x80;
	const GEN_FLT x120 = x111 * x81 + x78 * (-x111 * x79 + x119);
	const GEN_FLT x121 = x111 * x82 + x120 * x78;
	const GEN_FLT x122 = x116 * x95;
	const GEN_FLT x123 = 2.40324066e-5 * x78;
	const GEN_FLT x124 = x94 * (-curve_0 - x87);
	const GEN_FLT x125 = x117 * x88 / ((x96 * x96));
	const GEN_FLT x126 =
		-x101 * (x105 + x111 * x112 + x115 * x118 + x121 * x99 +
				 x125 * (x115 * x122 -
						 x124 * (x111 * x83 + x111 * x93 + x121 * x78 +
								 x78 * (x111 * x92 + x121 +
										x78 * (x111 * x91 + x120 + x78 * (-x111 * x123 + x111 * x90 + x119)))))) +
		x61;
	const GEN_FLT x127 = gibMag_0 * cos(gibPhase_0 + x85 - asin(x100));
	const GEN_FLT x128 = x114 * x70;
	const GEN_FLT x129 = -lh_py - x63 - x64 - x65;
	const GEN_FLT x130 = x107 * (x109 * x129 + x77);
	const GEN_FLT x131 = x130 * x80;
	const GEN_FLT x132 = x130 * x81 + x78 * (-x130 * x79 + x131);
	const GEN_FLT x133 = x130 * x82 + x132 * x78;
	const GEN_FLT x134 =
		x101 *
		(x112 * x130 - x118 * x128 +
		 x125 * (-x122 * x128 - x124 * (x130 * x83 + x130 * x93 + x133 * x78 +
										x78 * (x130 * x92 + x133 +
											   x78 * (x130 * x91 + x132 + x78 * (-x123 * x130 + x130 * x90 + x131))))) +
		 x133 * x99 + x70);
	const GEN_FLT x135 = x57 * x60;
	const GEN_FLT x136 = -x135;
	const GEN_FLT x137 = x104 * x58;
	const GEN_FLT x138 = x110 * x58;
	const GEN_FLT x139 = -x114 * x137 + x136;
	const GEN_FLT x140 = x138 * x80;
	const GEN_FLT x141 = x138 * x81 + x78 * (-x138 * x79 + x140);
	const GEN_FLT x142 = x138 * x82 + x141 * x78;
	const GEN_FLT x143 =
		-x101 * (x112 * x138 + x118 * x139 +
				 x125 * (x122 * x139 -
						 x124 * (x138 * x83 + x138 * x93 + x142 * x78 +
								 x78 * (x138 * x92 + x142 +
										x78 * (x138 * x91 + x141 + x78 * (-x123 * x138 + x138 * x90 + x140))))) +
				 x137 + x142 * x99) +
		x136;
	const GEN_FLT x144 = x31 * x33;
	const GEN_FLT x145 = -x144;
	const GEN_FLT x146 = 2 / ((x3 * x3));
	const GEN_FLT x147 = lh_qi * x146;
	const GEN_FLT x148 = x44 * (x144 * x62 + x145 + x6 * ((x8) ? (-x1 * x147) : (0)));
	const GEN_FLT x149 = pow(x3, -3.0 / 2.0);
	const GEN_FLT x150 = lh_qi * x149;
	const GEN_FLT x151 = ((x8) ? (-lh_qk * x150) : (0));
	const GEN_FLT x152 = x151 * x31;
	const GEN_FLT x153 = x33 * x5;
	const GEN_FLT x154 = x153 * x37;
	const GEN_FLT x155 = ((x8) ? (-lh_qj * x150) : (0));
	const GEN_FLT x156 = ((x8) ? (-x0 * x149 + x32) : (0));
	const GEN_FLT x157 = x33 * x46;
	const GEN_FLT x158 = x155 * x47 + x156 * x40 + x157 * x34;
	const GEN_FLT x159 = x49 * (x152 + x154 + x158);
	const GEN_FLT x160 = x151 * x40;
	const GEN_FLT x161 = x37 * x6;
	const GEN_FLT x162 = x155 * x161;
	const GEN_FLT x163 = x157 * x37;
	const GEN_FLT x164 = -x153 * x34 - x156 * x31;
	const GEN_FLT x165 = x29 * (x160 + x162 + x163 + x164);
	const GEN_FLT x166 = x148 + x159 + x165;
	const GEN_FLT x167 = (1.0 / 2.0) * x66;
	const GEN_FLT x168 = x29 * (-x144 * x9 + x144 - x6 * ((x8) ? (-x147 * x2) : (0)));
	const GEN_FLT x169 = x151 * x47;
	const GEN_FLT x170 = x156 * x161;
	const GEN_FLT x171 = x34 * x54;
	const GEN_FLT x172 = x171 * x33;
	const GEN_FLT x173 = x155 * x31;
	const GEN_FLT x174 = x153 * x39 + x173;
	const GEN_FLT x175 = x49 * (-x169 - x170 - x172 + x174);
	const GEN_FLT x176 = -x162;
	const GEN_FLT x177 = x44 * (-x160 - x163 + x164 + x176);
	const GEN_FLT x178 = (1.0 / 2.0) * x58;
	const GEN_FLT x179 = 2 * x7;
	const GEN_FLT x180 =
		x49 * (x144 * x51 + x145 + x6 * ((x8) ? (lh_qi * x179 - x146 * lh_qi * (lh_qi * lh_qi)) : (0)));
	const GEN_FLT x181 = x29 * (x169 + x170 + x172 + x174);
	const GEN_FLT x182 = -x152;
	const GEN_FLT x183 = x44 * (-x154 + x158 + x182);
	const GEN_FLT x184 = (1.0 / 2.0) * x57;
	const GEN_FLT x185 = -x178 * (2 * x168 + 2 * x175 + 2 * x177) - x184 * (2 * x180 + 2 * x181 + 2 * x183);
	const GEN_FLT x186 = -x167 * (2 * x148 + 2 * x159 + 2 * x165) + x185;
	const GEN_FLT x187 = x107 * (x109 * x186 + x166 * x77);
	const GEN_FLT x188 = x187 * x80;
	const GEN_FLT x189 = x187 * x81 + x78 * (-x187 * x79 + x188);
	const GEN_FLT x190 = x187 * x82 + x189 * x78;
	const GEN_FLT x191 = x104 * x185 + x166 * x70;
	const GEN_FLT x192 = x135 * (x168 + x175 + x177) + x61 * (x180 + x181 + x183);
	const GEN_FLT x193 = -x114 * x191 + x192;
	const GEN_FLT x194 =
		-x101 * (x112 * x187 + x118 * x193 +
				 x125 * (x122 * x193 -
						 x124 * (x187 * x83 + x187 * x93 + x190 * x78 +
								 x78 * (x187 * x92 + x190 +
										x78 * (x187 * x91 + x189 + x78 * (-x123 * x187 + x187 * x90 + x188))))) +
				 x190 * x99 + x191) +
		x192;
	const GEN_FLT x195 = x31 * x38;
	const GEN_FLT x196 = -x195;
	const GEN_FLT x197 =
		x44 * (x195 * x62 + x196 + x6 * ((x8) ? (lh_qj * x179 - x146 * lh_qj * (lh_qj * lh_qj)) : (0)));
	const GEN_FLT x198 = ((x8) ? (-lh_qj * lh_qk * x149) : (0));
	const GEN_FLT x199 = x198 * x31;
	const GEN_FLT x200 = x38 * x5;
	const GEN_FLT x201 = x200 * x37;
	const GEN_FLT x202 = ((x8) ? (-x1 * x149 + x32) : (0));
	const GEN_FLT x203 = x38 * x46;
	const GEN_FLT x204 = x155 * x40 + x202 * x47 + x203 * x34;
	const GEN_FLT x205 = x49 * (x199 + x201 + x204);
	const GEN_FLT x206 = x198 * x40;
	const GEN_FLT x207 = x161 * x202;
	const GEN_FLT x208 = x203 * x37;
	const GEN_FLT x209 = -x173 - x200 * x34;
	const GEN_FLT x210 = x29 * (x206 + x207 + x208 + x209);
	const GEN_FLT x211 = x197 + x205 + x210;
	const GEN_FLT x212 = lh_qj * x146;
	const GEN_FLT x213 = x49 * (x195 * x51 + x196 + x6 * ((x8) ? (-x0 * x212) : (0)));
	const GEN_FLT x214 = x198 * x47;
	const GEN_FLT x215 = x171 * x38;
	const GEN_FLT x216 = x200 * x39 + x202 * x31;
	const GEN_FLT x217 = x29 * (x162 + x214 + x215 + x216);
	const GEN_FLT x218 = x44 * (-x199 - x201 + x204);
	const GEN_FLT x219 = x29 * (-x195 * x9 + x195 - x6 * ((x8) ? (-x2 * x212) : (0)));
	const GEN_FLT x220 = x49 * (x176 - x214 - x215 + x216);
	const GEN_FLT x221 = x44 * (-x206 - x207 - x208 + x209);
	const GEN_FLT x222 = -x178 * (2 * x219 + 2 * x220 + 2 * x221) - x184 * (2 * x213 + 2 * x217 + 2 * x218);
	const GEN_FLT x223 = -x167 * (2 * x197 + 2 * x205 + 2 * x210) + x222;
	const GEN_FLT x224 = x107 * (x109 * x223 + x211 * x77);
	const GEN_FLT x225 = x224 * x80;
	const GEN_FLT x226 = x224 * x81 + x78 * (-x224 * x79 + x225);
	const GEN_FLT x227 = x224 * x82 + x226 * x78;
	const GEN_FLT x228 = x104 * x222 + x211 * x70;
	const GEN_FLT x229 = x135 * (x219 + x220 + x221) + x61 * (x213 + x217 + x218);
	const GEN_FLT x230 = -x114 * x228 + x229;
	const GEN_FLT x231 =
		-x101 * (x112 * x224 + x118 * x230 +
				 x125 * (x122 * x230 -
						 x124 * (x224 * x83 + x224 * x93 + x227 * x78 +
								 x78 * (x224 * x92 + x227 +
										x78 * (x224 * x91 + x226 + x78 * (-x123 * x224 + x224 * x90 + x225))))) +
				 x227 * x99 + x228) +
		x229;
	const GEN_FLT x232 = x31 * x36;
	const GEN_FLT x233 = -x232;
	const GEN_FLT x234 = lh_qk * x146;
	const GEN_FLT x235 = x44 * (x232 * x62 + x233 + x6 * ((x8) ? (-x1 * x234) : (0)));
	const GEN_FLT x236 = ((x8) ? (-x149 * x2 + x32) : (0));
	const GEN_FLT x237 = x236 * x31;
	const GEN_FLT x238 = x36 * x5;
	const GEN_FLT x239 = x238 * x37;
	const GEN_FLT x240 = x36 * x46;
	const GEN_FLT x241 = x160 + x214 + x240 * x34;
	const GEN_FLT x242 = x49 * (x237 + x239 + x241);
	const GEN_FLT x243 = x236 * x40;
	const GEN_FLT x244 = x161 * x198;
	const GEN_FLT x245 = x240 * x37;
	const GEN_FLT x246 = x182 - x238 * x34;
	const GEN_FLT x247 = x29 * (x243 + x244 + x245 + x246);
	const GEN_FLT x248 = x235 + x242 + x247;
	const GEN_FLT x249 = x49 * (x232 * x51 + x233 + x6 * ((x8) ? (-x0 * x234) : (0)));
	const GEN_FLT x250 = x236 * x47;
	const GEN_FLT x251 = x151 * x161;
	const GEN_FLT x252 = x171 * x36;
	const GEN_FLT x253 = x199 + x238 * x39;
	const GEN_FLT x254 = x29 * (x250 + x251 + x252 + x253);
	const GEN_FLT x255 = x44 * (-x237 - x239 + x241);
	const GEN_FLT x256 =
		x29 * (-x232 * x9 + x232 - x6 * ((x8) ? (lh_qk * x179 - x146 * lh_qk * (lh_qk * lh_qk)) : (0)));
	const GEN_FLT x257 = x49 * (-x250 - x251 - x252 + x253);
	const GEN_FLT x258 = x44 * (-x243 - x244 - x245 + x246);
	const GEN_FLT x259 = -x178 * (2 * x256 + 2 * x257 + 2 * x258) - x184 * (2 * x249 + 2 * x254 + 2 * x255);
	const GEN_FLT x260 = -x167 * (2 * x235 + 2 * x242 + 2 * x247) + x259;
	const GEN_FLT x261 = x107 * (x109 * x260 + x248 * x77);
	const GEN_FLT x262 = x261 * x80;
	const GEN_FLT x263 = x261 * x81 + x78 * (-x261 * x79 + x262);
	const GEN_FLT x264 = x261 * x82 + x263 * x78;
	const GEN_FLT x265 = x104 * x259 + x248 * x70;
	const GEN_FLT x266 = x135 * (x256 + x257 + x258) + x61 * (x249 + x254 + x255);
	const GEN_FLT x267 = -x114 * x265 + x266;
	const GEN_FLT x268 =
		-x101 * (x112 * x261 + x118 * x267 +
				 x125 * (x122 * x267 -
						 x124 * (x261 * x83 + x261 * x93 + x264 * x78 +
								 x78 * (x261 * x92 + x264 +
										x78 * (x261 * x91 + x263 + x78 * (-x123 * x261 + x261 * x90 + x262))))) +
				 x264 * x99 + x265) +
		x266;
	const GEN_FLT x269 = tilt_1 - 0.52359877559829882;
	const GEN_FLT x270 = tan(x269);
	const GEN_FLT x271 = x270 * x69;
	const GEN_FLT x272 = x271 * x66;
	const GEN_FLT x273 = cos(x269);
	const GEN_FLT x274 = 1.0 / x273;
	const GEN_FLT x275 = x274 * x76;
	const GEN_FLT x276 = asin(x275 * x66);
	const GEN_FLT x277 = 8.0108022e-6 * x276;
	const GEN_FLT x278 = -x277 - 8.0108022e-6;
	const GEN_FLT x279 = x276 * x278 + 0.0028679863;
	const GEN_FLT x280 = x276 * x279 + 5.3685255000000001e-6;
	const GEN_FLT x281 = x276 * x280 + 0.0076069798000000001;
	const GEN_FLT x282 = x276 * x276;
	const GEN_FLT x283 = ogeePhase_1 + x85 - asin(x272);
	const GEN_FLT x284 = ogeeMag_1 * sin(x283);
	const GEN_FLT x285 = curve_1 + x284;
	const GEN_FLT x286 = x276 * x281;
	const GEN_FLT x287 = -1.60216044e-5 * x276 - 8.0108022e-6;
	const GEN_FLT x288 = x276 * x287 + x279;
	const GEN_FLT x289 = x276 * x288 + x280;
	const GEN_FLT x290 = x276 * x289 + x281;
	const GEN_FLT x291 = sin(x269);
	const GEN_FLT x292 = x291 * (x276 * x290 + x286);
	const GEN_FLT x293 = x273 - x285 * x292;
	const GEN_FLT x294 = 1.0 / x293;
	const GEN_FLT x295 = x285 * x294;
	const GEN_FLT x296 = x282 * x295;
	const GEN_FLT x297 = x272 + x281 * x296;
	const GEN_FLT x298 = pow(1 - x297 * x297, -1.0 / 2.0);
	const GEN_FLT x299 = x103 * x270;
	const GEN_FLT x300 = x102 * x299;
	const GEN_FLT x301 = pow(-x106 / (x273 * x273) + 1, -1.0 / 2.0);
	const GEN_FLT x302 = x108 * x274;
	const GEN_FLT x303 = x301 * x302;
	const GEN_FLT x304 = x102 * x303;
	const GEN_FLT x305 = 2 * x286 * x295;
	const GEN_FLT x306 = pow(-x113 * x270 * x270 + 1, -1.0 / 2.0);
	const GEN_FLT x307 = -x300 * x306 + x61;
	const GEN_FLT x308 = ogeeMag_1 * cos(x283);
	const GEN_FLT x309 = x281 * x282;
	const GEN_FLT x310 = x294 * x308 * x309;
	const GEN_FLT x311 = x278 * x304;
	const GEN_FLT x312 = x276 * (-x277 * x304 + x311) + x279 * x304;
	const GEN_FLT x313 = x276 * x312 + x280 * x304;
	const GEN_FLT x314 = x292 * x308;
	const GEN_FLT x315 = 2.40324066e-5 * x276;
	const GEN_FLT x316 = x291 * (-curve_1 - x284);
	const GEN_FLT x317 = x285 * x309 / ((x293 * x293));
	const GEN_FLT x318 =
		-x298 * (x296 * x313 + x300 + x304 * x305 + x307 * x310 +
				 x317 * (x307 * x314 -
						 x316 * (x276 * x313 +
								 x276 * (x276 * (x276 * (x287 * x304 - x304 * x315 + x311) + x288 * x304 + x312) +
										 x289 * x304 + x313) +
								 x281 * x304 + x290 * x304))) +
		x61;
	const GEN_FLT x319 = gibMag_1 * cos(gibPhase_1 + x85 - asin(x297));
	const GEN_FLT x320 = x271 * x306;
	const GEN_FLT x321 = x301 * (x129 * x302 + x275);
	const GEN_FLT x322 = x278 * x321;
	const GEN_FLT x323 = x276 * (-x277 * x321 + x322) + x279 * x321;
	const GEN_FLT x324 = x276 * x323 + x280 * x321;
	const GEN_FLT x325 =
		x298 * (x271 + x296 * x324 + x305 * x321 - x310 * x320 +
				x317 * (-x314 * x320 -
						x316 * (x276 * x324 +
								x276 * (x276 * (x276 * (x287 * x321 - x315 * x321 + x322) + x288 * x321 + x323) +
										x289 * x321 + x324) +
								x281 * x321 + x290 * x321)));
	const GEN_FLT x326 = x299 * x58;
	const GEN_FLT x327 = x303 * x58;
	const GEN_FLT x328 = x136 - x306 * x326;
	const GEN_FLT x329 = x278 * x327;
	const GEN_FLT x330 = x276 * (-x277 * x327 + x329) + x279 * x327;
	const GEN_FLT x331 = x276 * x330 + x280 * x327;
	const GEN_FLT x332 =
		x136 - x298 * (x296 * x331 + x305 * x327 + x310 * x328 +
					   x317 * (x314 * x328 -
							   x316 * (x276 * x331 +
									   x276 * (x276 * (x276 * (x287 * x327 - x315 * x327 + x329) + x288 * x327 + x330) +
											   x289 * x327 + x331) +
									   x281 * x327 + x290 * x327)) +
					   x326);
	const GEN_FLT x333 = x301 * (x166 * x275 + x186 * x302);
	const GEN_FLT x334 = x278 * x333;
	const GEN_FLT x335 = x276 * (-x277 * x333 + x334) + x279 * x333;
	const GEN_FLT x336 = x276 * x335 + x280 * x333;
	const GEN_FLT x337 = x166 * x271 + x185 * x299;
	const GEN_FLT x338 = x192 - x306 * x337;
	const GEN_FLT x339 =
		x192 - x298 * (x296 * x336 + x305 * x333 + x310 * x338 +
					   x317 * (x314 * x338 -
							   x316 * (x276 * x336 +
									   x276 * (x276 * (x276 * (x287 * x333 - x315 * x333 + x334) + x288 * x333 + x335) +
											   x289 * x333 + x336) +
									   x281 * x333 + x290 * x333)) +
					   x337);
	const GEN_FLT x340 = x301 * (x211 * x275 + x223 * x302);
	const GEN_FLT x341 = x278 * x340;
	const GEN_FLT x342 = x276 * (-x277 * x340 + x341) + x279 * x340;
	const GEN_FLT x343 = x276 * x342 + x280 * x340;
	const GEN_FLT x344 = x211 * x271 + x222 * x299;
	const GEN_FLT x345 = x229 - x306 * x344;
	const GEN_FLT x346 =
		x229 - x298 * (x296 * x343 + x305 * x340 + x310 * x345 +
					   x317 * (x314 * x345 -
							   x316 * (x276 * x343 +
									   x276 * (x276 * (x276 * (x287 * x340 - x315 * x340 + x341) + x288 * x340 + x342) +
											   x289 * x340 + x343) +
									   x281 * x340 + x290 * x340)) +
					   x344);
	const GEN_FLT x347 = x301 * (x248 * x275 + x260 * x302);
	const GEN_FLT x348 = x278 * x347;
	const GEN_FLT x349 = x276 * (-x277 * x347 + x348) + x279 * x347;
	const GEN_FLT x350 = x276 * x349 + x280 * x347;
	const GEN_FLT x351 = x248 * x271 + x259 * x299;
	const GEN_FLT x352 = x266 - x306 * x351;
	const GEN_FLT x353 =
		x266 - x298 * (x296 * x350 + x305 * x347 + x310 * x352 +
					   x317 * (x314 * x352 -
							   x316 * (x276 * x350 +
									   x276 * (x276 * (x276 * (x287 * x347 - x315 * x347 + x348) + x288 * x347 + x349) +
											   x289 * x347 + x350) +
									   x281 * x347 + x290 * x347)) +
					   x351);
	*(out++) = x126 * x127 + x126;
	*(out++) = -x127 * x134 - x134;
	*(out++) = x127 * x143 + x143;
	*(out++) = x127 * x194 + x194;
	*(out++) = x127 * x231 + x231;
	*(out++) = x127 * x268 + x268;
	*(out++) = x318 * x319 + x318;
	*(out++) = -x319 * x325 - x325;
	*(out++) = x319 * x332 + x332;
	*(out++) = x319 * x339 + x339;
	*(out++) = x319 * x346 + x346;
	*(out++) = x319 * x353 + x353;
}

static inline void gen_reproject_axisangle_axis_x_jac_lh_p_gen2(FLT *out, const FLT *obj, const FLT *sensor,
																const FLT *lh, const FLT phase_0, const FLT tilt_0,
																const FLT curve_0, const FLT gibPhase_0,
																const FLT gibMag_0, const FLT ogeePhase_0,
																const FLT ogeeMag_0) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = 1 - x5;
	const GEN_FLT x7 = 1.0 / x3;
	const GEN_FLT x8 = x4 > 0;
	const GEN_FLT x9 = ((x8) ? (x2 * x7) : (0));
	const GEN_FLT x10 = obj_qi * obj_qi;
	const GEN_FLT x11 = obj_qj * obj_qj;
	const GEN_FLT x12 = obj_qk * obj_qk;
	const GEN_FLT x13 = x10 + x11 + x12;
	const GEN_FLT x14 = sqrt(x13);
	const GEN_FLT x15 = cos(x14);
	const GEN_FLT x16 = 1 - x15;
	const GEN_FLT x17 = 1.0 / x13;
	const GEN_FLT x18 = x14 > 0;
	const GEN_FLT x19 = sin(x14);
	const GEN_FLT x20 = 1.0 / x14;
	const GEN_FLT x21 = ((x18) ? (obj_qi * x20) : (1));
	const GEN_FLT x22 = x19 * x21;
	const GEN_FLT x23 = ((x18) ? (obj_qj * x20) : (0));
	const GEN_FLT x24 = ((x18) ? (obj_qk * x20) : (0));
	const GEN_FLT x25 = x16 * x23 * x24;
	const GEN_FLT x26 = x19 * x23;
	const GEN_FLT x27 = x16 * x21;
	const GEN_FLT x28 = x24 * x27;
	const GEN_FLT x29 = obj_pz + sensor_x * (-x26 + x28) + sensor_y * (x22 + x25) +
						sensor_z * (x15 + x16 * ((x18) ? (x12 * x17) : (0)));
	const GEN_FLT x30 = x29 * (x5 + x6 * x9);
	const GEN_FLT x31 = sin(x4);
	const GEN_FLT x32 = 1.0 / x4;
	const GEN_FLT x33 = lh_qi * x32;
	const GEN_FLT x34 = ((x8) ? (x33) : (1));
	const GEN_FLT x35 = x31 * x34;
	const GEN_FLT x36 = lh_qk * x32;
	const GEN_FLT x37 = ((x8) ? (x36) : (0));
	const GEN_FLT x38 = lh_qj * x32;
	const GEN_FLT x39 = ((x8) ? (x38) : (0));
	const GEN_FLT x40 = x39 * x6;
	const GEN_FLT x41 = x37 * x40;
	const GEN_FLT x42 = x19 * x24;
	const GEN_FLT x43 = x23 * x27;
	const GEN_FLT x44 = obj_py + sensor_x * (x42 + x43) + sensor_y * (x15 + x16 * ((x18) ? (x11 * x17) : (0))) +
						sensor_z * (-x22 + x25);
	const GEN_FLT x45 = x44 * (x35 + x41);
	const GEN_FLT x46 = x31 * x39;
	const GEN_FLT x47 = x34 * x6;
	const GEN_FLT x48 = x37 * x47;
	const GEN_FLT x49 = obj_px + sensor_x * (x15 + x16 * ((x18) ? (x10 * x17) : (1))) + sensor_y * (-x42 + x43) +
						sensor_z * (x26 + x28);
	const GEN_FLT x50 = x49 * (-x46 + x48);
	const GEN_FLT x51 = ((x8) ? (x0 * x7) : (1));
	const GEN_FLT x52 = x49 * (x5 + x51 * x6);
	const GEN_FLT x53 = x29 * (x46 + x48);
	const GEN_FLT x54 = x31 * x37;
	const GEN_FLT x55 = x39 * x47;
	const GEN_FLT x56 = x44 * (-x54 + x55);
	const GEN_FLT x57 = lh_px + x52 + x53 + x56;
	const GEN_FLT x58 = -lh_pz - x30 - x45 - x50;
	const GEN_FLT x59 = x57 * x57 + x58 * x58;
	const GEN_FLT x60 = 1.0 / x59;
	const GEN_FLT x61 = x60 * (lh_pz + x30 + x45 + x50);
	const GEN_FLT x62 = ((x8) ? (x1 * x7) : (0));
	const GEN_FLT x63 = x44 * (x5 + x6 * x62);
	const GEN_FLT x64 = x49 * (x54 + x55);
	const GEN_FLT x65 = x29 * (-x35 + x41);
	const GEN_FLT x66 = lh_py + x63 + x64 + x65;
	const GEN_FLT x67 = tilt_0 + 0.52359877559829882;
	const GEN_FLT x68 = tan(x67);
	const GEN_FLT x69 = x68 / sqrt(x59);
	const GEN_FLT x70 = x66 * x69;
	const GEN_FLT x71 = cos(x67);
	const GEN_FLT x72 = 1.0 / x71;
	const GEN_FLT x73 = x66 * x66;
	const GEN_FLT x74 = x59 + x73;
	const GEN_FLT x75 = x72 / sqrt(x74);
	const GEN_FLT x76 = asin(x66 * x75);
	const GEN_FLT x77 = 8.0108022e-6 * x76;
	const GEN_FLT x78 = -x77 - 8.0108022e-6;
	const GEN_FLT x79 = x76 * x78 + 0.0028679863;
	const GEN_FLT x80 = x76 * x79 + 5.3685255000000001e-6;
	const GEN_FLT x81 = x76 * x80 + 0.0076069798000000001;
	const GEN_FLT x82 = x76 * x76;
	const GEN_FLT x83 = atan2(x58, x57);
	const GEN_FLT x84 = ogeePhase_0 + x83 - asin(x70);
	const GEN_FLT x85 = ogeeMag_0 * sin(x84);
	const GEN_FLT x86 = curve_0 + x85;
	const GEN_FLT x87 = x76 * x81;
	const GEN_FLT x88 = -1.60216044e-5 * x76 - 8.0108022e-6;
	const GEN_FLT x89 = x76 * x88 + x79;
	const GEN_FLT x90 = x76 * x89 + x80;
	const GEN_FLT x91 = x76 * x90 + x81;
	const GEN_FLT x92 = sin(x67);
	const GEN_FLT x93 = x92 * (x76 * x91 + x87);
	const GEN_FLT x94 = x71 - x86 * x93;
	const GEN_FLT x95 = 1.0 / x94;
	const GEN_FLT x96 = x86 * x95;
	const GEN_FLT x97 = x82 * x96;
	const GEN_FLT x98 = x70 + x81 * x97;
	const GEN_FLT x99 = pow(1 - x98 * x98, -1.0 / 2.0);
	const GEN_FLT x100 = x66 * (-lh_px - x52 - x53 - x56);
	const GEN_FLT x101 = x68 / pow(x59, 3.0 / 2.0);
	const GEN_FLT x102 = x100 * x101;
	const GEN_FLT x103 = pow(-x73 / (x74 * (x71 * x71)) + 1, -1.0 / 2.0);
	const GEN_FLT x104 = x72 / pow(x74, 3.0 / 2.0);
	const GEN_FLT x105 = x100 * x104;
	const GEN_FLT x106 = x103 * x105;
	const GEN_FLT x107 = 2 * x87 * x96;
	const GEN_FLT x108 = pow(-x60 * x73 * x68 * x68 + 1, -1.0 / 2.0);
	const GEN_FLT x109 = -x102 * x108 + x61;
	const GEN_FLT x110 = ogeeMag_0 * cos(x84);
	const GEN_FLT x111 = x81 * x82;
	const GEN_FLT x112 = x110 * x111 * x95;
	const GEN_FLT x113 = x103 * x78;
	const GEN_FLT x114 = x105 * x113;
	const GEN_FLT x115 = x106 * x79 + x76 * (-x106 * x77 + x114);
	const GEN_FLT x116 = x106 * x80 + x115 * x76;
	const GEN_FLT x117 = x110 * x93;
	const GEN_FLT x118 = 2.40324066e-5 * x76;
	const GEN_FLT x119 = x92 * (-curve_0 - x85);
	const GEN_FLT x120 = x111 * x86 / ((x94 * x94));
	const GEN_FLT x121 =
		x61 - x99 * (x102 + x106 * x107 + x109 * x112 + x116 * x97 +
					 x120 * (x109 * x117 -
							 x119 * (x106 * x81 + x106 * x91 + x116 * x76 +
									 x76 * (x106 * x90 + x116 +
											x76 * (x106 * x89 + x115 + x76 * (-x106 * x118 + x106 * x88 + x114))))));
	const GEN_FLT x122 = gibMag_0 * cos(gibPhase_0 + x83 - asin(x98));
	const GEN_FLT x123 = x108 * x69;
	const GEN_FLT x124 = x104 * x66;
	const GEN_FLT x125 = x103 * (x124 * (-lh_py - x63 - x64 - x65) + x75);
	const GEN_FLT x126 = x125 * x78;
	const GEN_FLT x127 = x125 * x79 + x76 * (-x125 * x77 + x126);
	const GEN_FLT x128 = x125 * x80 + x127 * x76;
	const GEN_FLT x129 =
		x99 *
		(x107 * x125 - x112 * x123 +
		 x120 * (-x117 * x123 - x119 * (x125 * x81 + x125 * x91 + x128 * x76 +
										x76 * (x125 * x90 + x128 +
											   x76 * (x125 * x89 + x127 + x76 * (-x118 * x125 + x125 * x88 + x126))))) +
		 x128 * x97 + x69);
	const GEN_FLT x130 = x57 * x60;
	const GEN_FLT x131 = -x130;
	const GEN_FLT x132 = x101 * x66;
	const GEN_FLT x133 = x132 * x58;
	const GEN_FLT x134 = x124 * x58;
	const GEN_FLT x135 = x103 * x134;
	const GEN_FLT x136 = -x108 * x133 + x131;
	const GEN_FLT x137 = x113 * x134;
	const GEN_FLT x138 = x135 * x79 + x76 * (-x135 * x77 + x137);
	const GEN_FLT x139 = x135 * x80 + x138 * x76;
	const GEN_FLT x140 =
		x131 - x99 * (x107 * x135 + x112 * x136 +
					  x120 * (x117 * x136 -
							  x119 * (x135 * x81 + x135 * x91 + x139 * x76 +
									  x76 * (x135 * x90 + x139 +
											 x76 * (x135 * x89 + x138 + x76 * (-x118 * x135 + x135 * x88 + x137))))) +
					  x133 + x139 * x97);
	const GEN_FLT x141 = x31 * x33;
	const GEN_FLT x142 = -x141;
	const GEN_FLT x143 = 2 / ((x3 * x3));
	const GEN_FLT x144 = lh_qi * x143;
	const GEN_FLT x145 = x44 * (x141 * x62 + x142 + x6 * ((x8) ? (-x1 * x144) : (0)));
	const GEN_FLT x146 = pow(x3, -3.0 / 2.0);
	const GEN_FLT x147 = lh_qi * x146;
	const GEN_FLT x148 = ((x8) ? (-lh_qk * x147) : (0));
	const GEN_FLT x149 = x148 * x31;
	const GEN_FLT x150 = x33 * x5;
	const GEN_FLT x151 = x150 * x37;
	const GEN_FLT x152 = ((x8) ? (-lh_qj * x147) : (0));
	const GEN_FLT x153 = ((x8) ? (-x0 * x146 + x32) : (0));
	const GEN_FLT x154 = x33 * x46;
	const GEN_FLT x155 = x152 * x47 + x153 * x40 + x154 * x34;
	const GEN_FLT x156 = x49 * (x149 + x151 + x155);
	const GEN_FLT x157 = x148 * x40;
	const GEN_FLT x158 = x37 * x6;
	const GEN_FLT x159 = x152 * x158;
	const GEN_FLT x160 = x154 * x37;
	const GEN_FLT x161 = -x150 * x34 - x153 * x31;
	const GEN_FLT x162 = x29 * (x157 + x159 + x160 + x161);
	const GEN_FLT x163 = x145 + x156 + x162;
	const GEN_FLT x164 = (1.0 / 2.0) * x66;
	const GEN_FLT x165 = x29 * (-x141 * x9 + x141 - x6 * ((x8) ? (-x144 * x2) : (0)));
	const GEN_FLT x166 = x148 * x47;
	const GEN_FLT x167 = x153 * x158;
	const GEN_FLT x168 = x34 * x54;
	const GEN_FLT x169 = x168 * x33;
	const GEN_FLT x170 = x152 * x31;
	const GEN_FLT x171 = x150 * x39 + x170;
	const GEN_FLT x172 = x49 * (-x166 - x167 - x169 + x171);
	const GEN_FLT x173 = -x159;
	const GEN_FLT x174 = x44 * (-x157 - x160 + x161 + x173);
	const GEN_FLT x175 = (1.0 / 2.0) * x58;
	const GEN_FLT x176 = 2 * x7;
	const GEN_FLT x177 =
		x49 * (x141 * x51 + x142 + x6 * ((x8) ? (lh_qi * x176 - x143 * lh_qi * (lh_qi * lh_qi)) : (0)));
	const GEN_FLT x178 = x29 * (x166 + x167 + x169 + x171);
	const GEN_FLT x179 = -x149;
	const GEN_FLT x180 = x44 * (-x151 + x155 + x179);
	const GEN_FLT x181 = (1.0 / 2.0) * x57;
	const GEN_FLT x182 = -x175 * (2 * x165 + 2 * x172 + 2 * x174) - x181 * (2 * x177 + 2 * x178 + 2 * x180);
	const GEN_FLT x183 = x103 * (x124 * (-x164 * (2 * x145 + 2 * x156 + 2 * x162) + x182) + x163 * x75);
	const GEN_FLT x184 = x183 * x78;
	const GEN_FLT x185 = x183 * x79 + x76 * (-x183 * x77 + x184);
	const GEN_FLT x186 = x183 * x80 + x185 * x76;
	const GEN_FLT x187 = x132 * x182 + x163 * x69;
	const GEN_FLT x188 = x130 * (x165 + x172 + x174) + x61 * (x177 + x178 + x180);
	const GEN_FLT x189 = -x108 * x187 + x188;
	const GEN_FLT x190 =
		x188 - x99 * (x107 * x183 + x112 * x189 +
					  x120 * (x117 * x189 -
							  x119 * (x183 * x81 + x183 * x91 + x186 * x76 +
									  x76 * (x183 * x90 + x186 +
											 x76 * (x183 * x89 + x185 + x76 * (-x118 * x183 + x183 * x88 + x184))))) +
					  x186 * x97 + x187);
	const GEN_FLT x191 = x31 * x38;
	const GEN_FLT x192 = -x191;
	const GEN_FLT x193 =
		x44 * (x191 * x62 + x192 + x6 * ((x8) ? (lh_qj * x176 - x143 * lh_qj * (lh_qj * lh_qj)) : (0)));
	const GEN_FLT x194 = ((x8) ? (-lh_qj * lh_qk * x146) : (0));
	const GEN_FLT x195 = x194 * x31;
	const GEN_FLT x196 = x38 * x5;
	const GEN_FLT x197 = x196 * x37;
	const GEN_FLT x198 = ((x8) ? (-x1 * x146 + x32) : (0));
	const GEN_FLT x199 = x38 * x46;
	const GEN_FLT x200 = x152 * x40 + x198 * x47 + x199 * x34;
	const GEN_FLT x201 = x49 * (x195 + x197 + x200);
	const GEN_FLT x202 = x194 * x40;
	const GEN_FLT x203 = x158 * x198;
	const GEN_FLT x204 = x199 * x37;
	const GEN_FLT x205 = -x170 - x196 * x34;
	const GEN_FLT x206 = x29 * (x202 + x203 + x204 + x205);
	const GEN_FLT x207 = x193 + x201 + x206;
	const GEN_FLT x208 = lh_qj * x143;
	const GEN_FLT x209 = x49 * (x191 * x51 + x192 + x6 * ((x8) ? (-x0 * x208) : (0)));
	const GEN_FLT x210 = x194 * x47;
	const GEN_FLT x211 = x168 * x38;
	const GEN_FLT x212 = x196 * x39 + x198 * x31;
	const GEN_FLT x213 = x29 * (x159 + x210 + x211 + x212);
	const GEN_FLT x214 = x44 * (-x195 - x197 + x200);
	const GEN_FLT x215 = x29 * (-x191 * x9 + x191 - x6 * ((x8) ? (-x2 * x208) : (0)));
	const GEN_FLT x216 = x49 * (x173 - x210 - x211 + x212);
	const GEN_FLT x217 = x44 * (-x202 - x203 - x204 + x205);
	const GEN_FLT x218 = -x175 * (2 * x215 + 2 * x216 + 2 * x217) - x181 * (2 * x209 + 2 * x213 + 2 * x214);
	const GEN_FLT x219 = x103 * (x124 * (-x164 * (2 * x193 + 2 * x201 + 2 * x206) + x218) + x207 * x75);
	const GEN_FLT x220 = x219 * x78;
	const GEN_FLT x221 = x219 * x79 + x76 * (-x219 * x77 + x220);
	const GEN_FLT x222 = x219 * x80 + x221 * x76;
	const GEN_FLT x223 = x132 * x218 + x207 * x69;
	const GEN_FLT x224 = x130 * (x215 + x216 + x217) + x61 * (x209 + x213 + x214);
	const GEN_FLT x225 = -x108 * x223 + x224;
	const GEN_FLT x226 =
		x224 - x99 * (x107 * x219 + x112 * x225 +
					  x120 * (x117 * x225 -
							  x119 * (x219 * x81 + x219 * x91 + x222 * x76 +
									  x76 * (x219 * x90 + x222 +
											 x76 * (x219 * x89 + x221 + x76 * (-x118 * x219 + x219 * x88 + x220))))) +
					  x222 * x97 + x223);
	const GEN_FLT x227 = x31 * x36;
	const GEN_FLT x228 = -x227;
	const GEN_FLT x229 = lh_qk * x143;
	const GEN_FLT x230 = x44 * (x227 * x62 + x228 + x6 * ((x8) ? (-x1 * x229) : (0)));
	const GEN_FLT x231 = ((x8) ? (-x146 * x2 + x32) : (0));
	const GEN_FLT x232 = x231 * x31;
	const GEN_FLT x233 = x36 * x5;
	const GEN_FLT x234 = x233 * x37;
	const GEN_FLT x235 = x36 * x46;
	const GEN_FLT x236 = x157 + x210 + x235 * x34;
	const GEN_FLT x237 = x49 * (x232 + x234 + x236);
	const GEN_FLT x238 = x231 * x40;
	const GEN_FLT x239 = x158 * x194;
	const GEN_FLT x240 = x235 * x37;
	const GEN_FLT x241 = x179 - x233 * x34;
	const GEN_FLT x242 = x29 * (x238 + x239 + x240 + x241);
	const GEN_FLT x243 = x230 + x237 + x242;
	const GEN_FLT x244 = x49 * (x227 * x51 + x228 + x6 * ((x8) ? (-x0 * x229) : (0)));
	const GEN_FLT x245 = x231 * x47;
	const GEN_FLT x246 = x148 * x158;
	const GEN_FLT x247 = x168 * x36;
	const GEN_FLT x248 = x195 + x233 * x39;
	const GEN_FLT x249 = x29 * (x245 + x246 + x247 + x248);
	const GEN_FLT x250 = x44 * (-x232 - x234 + x236);
	const GEN_FLT x251 =
		x29 * (-x227 * x9 + x227 - x6 * ((x8) ? (lh_qk * x176 - x143 * lh_qk * (lh_qk * lh_qk)) : (0)));
	const GEN_FLT x252 = x49 * (-x245 - x246 - x247 + x248);
	const GEN_FLT x253 = x44 * (-x238 - x239 - x240 + x241);
	const GEN_FLT x254 = -x175 * (2 * x251 + 2 * x252 + 2 * x253) - x181 * (2 * x244 + 2 * x249 + 2 * x250);
	const GEN_FLT x255 = x103 * (x124 * (-x164 * (2 * x230 + 2 * x237 + 2 * x242) + x254) + x243 * x75);
	const GEN_FLT x256 = x255 * x78;
	const GEN_FLT x257 = x255 * x79 + x76 * (-x255 * x77 + x256);
	const GEN_FLT x258 = x255 * x80 + x257 * x76;
	const GEN_FLT x259 = x132 * x254 + x243 * x69;
	const GEN_FLT x260 = x130 * (x251 + x252 + x253) + x61 * (x244 + x249 + x250);
	const GEN_FLT x261 = -x108 * x259 + x260;
	const GEN_FLT x262 =
		x260 - x99 * (x107 * x255 + x112 * x261 +
					  x120 * (x117 * x261 -
							  x119 * (x255 * x81 + x255 * x91 + x258 * x76 +
									  x76 * (x255 * x90 + x258 +
											 x76 * (x255 * x89 + x257 + x76 * (-x118 * x255 + x255 * x88 + x256))))) +
					  x258 * x97 + x259);
	*(out++) = x121 * x122 + x121;
	*(out++) = -x122 * x129 - x129;
	*(out++) = x122 * x140 + x140;
	*(out++) = x122 * x190 + x190;
	*(out++) = x122 * x226 + x226;
	*(out++) = x122 * x262 + x262;
}

static inline void gen_reproject_axisangle_axis_y_jac_lh_p_gen2(FLT *out, const FLT *obj, const FLT *sensor,
																const FLT *lh, const FLT phase_0, const FLT tilt_0,
																const FLT curve_0, const FLT gibPhase_0,
																const FLT gibMag_0, const FLT ogeePhase_0,
																const FLT ogeeMag_0) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = 1 - x5;
	const GEN_FLT x7 = 1.0 / x3;
	const GEN_FLT x8 = x4 > 0;
	const GEN_FLT x9 = ((x8) ? (x2 * x7) : (0));
	const GEN_FLT x10 = obj_qi * obj_qi;
	const GEN_FLT x11 = obj_qj * obj_qj;
	const GEN_FLT x12 = obj_qk * obj_qk;
	const GEN_FLT x13 = x10 + x11 + x12;
	const GEN_FLT x14 = sqrt(x13);
	const GEN_FLT x15 = cos(x14);
	const GEN_FLT x16 = 1 - x15;
	const GEN_FLT x17 = 1.0 / x13;
	const GEN_FLT x18 = x14 > 0;
	const GEN_FLT x19 = sin(x14);
	const GEN_FLT x20 = 1.0 / x14;
	const GEN_FLT x21 = ((x18) ? (obj_qi * x20) : (1));
	const GEN_FLT x22 = x19 * x21;
	const GEN_FLT x23 = ((x18) ? (obj_qj * x20) : (0));
	const GEN_FLT x24 = ((x18) ? (obj_qk * x20) : (0));
	const GEN_FLT x25 = x16 * x23 * x24;
	const GEN_FLT x26 = x19 * x23;
	const GEN_FLT x27 = x16 * x21;
	const GEN_FLT x28 = x24 * x27;
	const GEN_FLT x29 = obj_pz + sensor_x * (-x26 + x28) + sensor_y * (x22 + x25) +
						sensor_z * (x15 + x16 * ((x18) ? (x12 * x17) : (0)));
	const GEN_FLT x30 = x29 * (x5 + x6 * x9);
	const GEN_FLT x31 = sin(x4);
	const GEN_FLT x32 = 1.0 / x4;
	const GEN_FLT x33 = lh_qi * x32;
	const GEN_FLT x34 = ((x8) ? (x33) : (1));
	const GEN_FLT x35 = x31 * x34;
	const GEN_FLT x36 = lh_qk * x32;
	const GEN_FLT x37 = ((x8) ? (x36) : (0));
	const GEN_FLT x38 = lh_qj * x32;
	const GEN_FLT x39 = ((x8) ? (x38) : (0));
	const GEN_FLT x40 = x39 * x6;
	const GEN_FLT x41 = x37 * x40;
	const GEN_FLT x42 = x19 * x24;
	const GEN_FLT x43 = x23 * x27;
	const GEN_FLT x44 = obj_py + sensor_x * (x42 + x43) + sensor_y * (x15 + x16 * ((x18) ? (x11 * x17) : (0))) +
						sensor_z * (-x22 + x25);
	const GEN_FLT x45 = x44 * (x35 + x41);
	const GEN_FLT x46 = x31 * x39;
	const GEN_FLT x47 = x34 * x6;
	const GEN_FLT x48 = x37 * x47;
	const GEN_FLT x49 = obj_px + sensor_x * (x15 + x16 * ((x18) ? (x10 * x17) : (1))) + sensor_y * (-x42 + x43) +
						sensor_z * (x26 + x28);
	const GEN_FLT x50 = x49 * (-x46 + x48);
	const GEN_FLT x51 = ((x8) ? (x0 * x7) : (1));
	const GEN_FLT x52 = x49 * (x5 + x51 * x6);
	const GEN_FLT x53 = x29 * (x46 + x48);
	const GEN_FLT x54 = x31 * x37;
	const GEN_FLT x55 = x39 * x47;
	const GEN_FLT x56 = x44 * (-x54 + x55);
	const GEN_FLT x57 = lh_px + x52 + x53 + x56;
	const GEN_FLT x58 = -lh_pz - x30 - x45 - x50;
	const GEN_FLT x59 = x57 * x57 + x58 * x58;
	const GEN_FLT x60 = 1.0 / x59;
	const GEN_FLT x61 = x60 * (lh_pz + x30 + x45 + x50);
	const GEN_FLT x62 = ((x8) ? (x1 * x7) : (0));
	const GEN_FLT x63 = x44 * (x5 + x6 * x62);
	const GEN_FLT x64 = x49 * (x54 + x55);
	const GEN_FLT x65 = x29 * (-x35 + x41);
	const GEN_FLT x66 = lh_py + x63 + x64 + x65;
	const GEN_FLT x67 = tilt_0 - 0.52359877559829882;
	const GEN_FLT x68 = tan(x67);
	const GEN_FLT x69 = x68 / sqrt(x59);
	const GEN_FLT x70 = x66 * x69;
	const GEN_FLT x71 = cos(x67);
	const GEN_FLT x72 = 1.0 / x71;
	const GEN_FLT x73 = x66 * x66;
	const GEN_FLT x74 = x59 + x73;
	const GEN_FLT x75 = x72 / sqrt(x74);
	const GEN_FLT x76 = asin(x66 * x75);
	const GEN_FLT x77 = 8.0108022e-6 * x76;
	const GEN_FLT x78 = -x77 - 8.0108022e-6;
	const GEN_FLT x79 = x76 * x78 + 0.0028679863;
	const GEN_FLT x80 = x76 * x79 + 5.3685255000000001e-6;
	const GEN_FLT x81 = x76 * x80 + 0.0076069798000000001;
	const GEN_FLT x82 = x76 * x76;
	const GEN_FLT x83 = atan2(x58, x57);
	const GEN_FLT x84 = ogeePhase_0 + x83 - asin(x70);
	const GEN_FLT x85 = ogeeMag_0 * sin(x84);
	const GEN_FLT x86 = curve_0 + x85;
	const GEN_FLT x87 = x76 * x81;
	const GEN_FLT x88 = -1.60216044e-5 * x76 - 8.0108022e-6;
	const GEN_FLT x89 = x76 * x88 + x79;
	const GEN_FLT x90 = x76 * x89 + x80;
	const GEN_FLT x91 = x76 * x90 + x81;
	const GEN_FLT x92 = sin(x67);
	const GEN_FLT x93 = x92 * (x76 * x91 + x87);
	const GEN_FLT x94 = x71 - x86 * x93;
	const GEN_FLT x95 = 1.0 / x94;
	const GEN_FLT x96 = x86 * x95;
	const GEN_FLT x97 = x82 * x96;
	const GEN_FLT x98 = x70 + x81 * x97;
	const GEN_FLT x99 = pow(1 - x98 * x98, -1.0 / 2.0);
	const GEN_FLT x100 = x66 * (-lh_px - x52 - x53 - x56);
	const GEN_FLT x101 = x68 / pow(x59, 3.0 / 2.0);
	const GEN_FLT x102 = x100 * x101;
	const GEN_FLT x103 = pow(-x73 / (x74 * (x71 * x71)) + 1, -1.0 / 2.0);
	const GEN_FLT x104 = x72 / pow(x74, 3.0 / 2.0);
	const GEN_FLT x105 = x100 * x104;
	const GEN_FLT x106 = x103 * x105;
	const GEN_FLT x107 = 2 * x87 * x96;
	const GEN_FLT x108 = pow(-x60 * x73 * x68 * x68 + 1, -1.0 / 2.0);
	const GEN_FLT x109 = -x102 * x108 + x61;
	const GEN_FLT x110 = ogeeMag_0 * cos(x84);
	const GEN_FLT x111 = x81 * x82;
	const GEN_FLT x112 = x110 * x111 * x95;
	const GEN_FLT x113 = x103 * x78;
	const GEN_FLT x114 = x105 * x113;
	const GEN_FLT x115 = x106 * x79 + x76 * (-x106 * x77 + x114);
	const GEN_FLT x116 = x106 * x80 + x115 * x76;
	const GEN_FLT x117 = x110 * x93;
	const GEN_FLT x118 = 2.40324066e-5 * x76;
	const GEN_FLT x119 = x92 * (-curve_0 - x85);
	const GEN_FLT x120 = x111 * x86 / ((x94 * x94));
	const GEN_FLT x121 =
		x61 - x99 * (x102 + x106 * x107 + x109 * x112 + x116 * x97 +
					 x120 * (x109 * x117 -
							 x119 * (x106 * x81 + x106 * x91 + x116 * x76 +
									 x76 * (x106 * x90 + x116 +
											x76 * (x106 * x89 + x115 + x76 * (-x106 * x118 + x106 * x88 + x114))))));
	const GEN_FLT x122 = gibMag_0 * cos(gibPhase_0 + x83 - asin(x98));
	const GEN_FLT x123 = x108 * x69;
	const GEN_FLT x124 = x104 * x66;
	const GEN_FLT x125 = x103 * (x124 * (-lh_py - x63 - x64 - x65) + x75);
	const GEN_FLT x126 = x125 * x78;
	const GEN_FLT x127 = x125 * x79 + x76 * (-x125 * x77 + x126);
	const GEN_FLT x128 = x125 * x80 + x127 * x76;
	const GEN_FLT x129 =
		x99 *
		(x107 * x125 - x112 * x123 +
		 x120 * (-x117 * x123 - x119 * (x125 * x81 + x125 * x91 + x128 * x76 +
										x76 * (x125 * x90 + x128 +
											   x76 * (x125 * x89 + x127 + x76 * (-x118 * x125 + x125 * x88 + x126))))) +
		 x128 * x97 + x69);
	const GEN_FLT x130 = x57 * x60;
	const GEN_FLT x131 = -x130;
	const GEN_FLT x132 = x101 * x66;
	const GEN_FLT x133 = x132 * x58;
	const GEN_FLT x134 = x124 * x58;
	const GEN_FLT x135 = x103 * x134;
	const GEN_FLT x136 = -x108 * x133 + x131;
	const GEN_FLT x137 = x113 * x134;
	const GEN_FLT x138 = x135 * x79 + x76 * (-x135 * x77 + x137);
	const GEN_FLT x139 = x135 * x80 + x138 * x76;
	const GEN_FLT x140 =
		x131 - x99 * (x107 * x135 + x112 * x136 +
					  x120 * (x117 * x136 -
							  x119 * (x135 * x81 + x135 * x91 + x139 * x76 +
									  x76 * (x135 * x90 + x139 +
											 x76 * (x135 * x89 + x138 + x76 * (-x118 * x135 + x135 * x88 + x137))))) +
					  x133 + x139 * x97);
	const GEN_FLT x141 = x31 * x33;
	const GEN_FLT x142 = -x141;
	const GEN_FLT x143 = 2 / ((x3 * x3));
	const GEN_FLT x144 = lh_qi * x143;
	const GEN_FLT x145 = x44 * (x141 * x62 + x142 + x6 * ((x8) ? (-x1 * x144) : (0)));
	const GEN_FLT x146 = pow(x3, -3.0 / 2.0);
	const GEN_FLT x147 = lh_qi * x146;
	const GEN_FLT x148 = ((x8) ? (-lh_qk * x147) : (0));
	const GEN_FLT x149 = x148 * x31;
	const GEN_FLT x150 = x33 * x5;
	const GEN_FLT x151 = x150 * x37;
	const GEN_FLT x152 = ((x8) ? (-lh_qj * x147) : (0));
	const GEN_FLT x153 = ((x8) ? (-x0 * x146 + x32) : (0));
	const GEN_FLT x154 = x33 * x46;
	const GEN_FLT x155 = x152 * x47 + x153 * x40 + x154 * x34;
	const GEN_FLT x156 = x49 * (x149 + x151 + x155);
	const GEN_FLT x157 = x148 * x40;
	const GEN_FLT x158 = x37 * x6;
	const GEN_FLT x159 = x152 * x158;
	const GEN_FLT x160 = x154 * x37;
	const GEN_FLT x161 = -x150 * x34 - x153 * x31;
	const GEN_FLT x162 = x29 * (x157 + x159 + x160 + x161);
	const GEN_FLT x163 = x145 + x156 + x162;
	const GEN_FLT x164 = (1.0 / 2.0) * x66;
	const GEN_FLT x165 = x29 * (-x141 * x9 + x141 - x6 * ((x8) ? (-x144 * x2) : (0)));
	const GEN_FLT x166 = x148 * x47;
	const GEN_FLT x167 = x153 * x158;
	const GEN_FLT x168 = x34 * x54;
	const GEN_FLT x169 = x168 * x33;
	const GEN_FLT x170 = x152 * x31;
	const GEN_FLT x171 = x150 * x39 + x170;
	const GEN_FLT x172 = x49 * (-x166 - x167 - x169 + x171);
	const GEN_FLT x173 = -x159;
	const GEN_FLT x174 = x44 * (-x157 - x160 + x161 + x173);
	const GEN_FLT x175 = (1.0 / 2.0) * x58;
	const GEN_FLT x176 = 2 * x7;
	const GEN_FLT x177 =
		x49 * (x141 * x51 + x142 + x6 * ((x8) ? (lh_qi * x176 - x143 * lh_qi * (lh_qi * lh_qi)) : (0)));
	const GEN_FLT x178 = x29 * (x166 + x167 + x169 + x171);
	const GEN_FLT x179 = -x149;
	const GEN_FLT x180 = x44 * (-x151 + x155 + x179);
	const GEN_FLT x181 = (1.0 / 2.0) * x57;
	const GEN_FLT x182 = -x175 * (2 * x165 + 2 * x172 + 2 * x174) - x181 * (2 * x177 + 2 * x178 + 2 * x180);
	const GEN_FLT x183 = x103 * (x124 * (-x164 * (2 * x145 + 2 * x156 + 2 * x162) + x182) + x163 * x75);
	const GEN_FLT x184 = x183 * x78;
	const GEN_FLT x185 = x183 * x79 + x76 * (-x183 * x77 + x184);
	const GEN_FLT x186 = x183 * x80 + x185 * x76;
	const GEN_FLT x187 = x132 * x182 + x163 * x69;
	const GEN_FLT x188 = x130 * (x165 + x172 + x174) + x61 * (x177 + x178 + x180);
	const GEN_FLT x189 = -x108 * x187 + x188;
	const GEN_FLT x190 =
		x188 - x99 * (x107 * x183 + x112 * x189 +
					  x120 * (x117 * x189 -
							  x119 * (x183 * x81 + x183 * x91 + x186 * x76 +
									  x76 * (x183 * x90 + x186 +
											 x76 * (x183 * x89 + x185 + x76 * (-x118 * x183 + x183 * x88 + x184))))) +
					  x186 * x97 + x187);
	const GEN_FLT x191 = x31 * x38;
	const GEN_FLT x192 = -x191;
	const GEN_FLT x193 =
		x44 * (x191 * x62 + x192 + x6 * ((x8) ? (lh_qj * x176 - x143 * lh_qj * (lh_qj * lh_qj)) : (0)));
	const GEN_FLT x194 = ((x8) ? (-lh_qj * lh_qk * x146) : (0));
	const GEN_FLT x195 = x194 * x31;
	const GEN_FLT x196 = x38 * x5;
	const GEN_FLT x197 = x196 * x37;
	const GEN_FLT x198 = ((x8) ? (-x1 * x146 + x32) : (0));
	const GEN_FLT x199 = x38 * x46;
	const GEN_FLT x200 = x152 * x40 + x198 * x47 + x199 * x34;
	const GEN_FLT x201 = x49 * (x195 + x197 + x200);
	const GEN_FLT x202 = x194 * x40;
	const GEN_FLT x203 = x158 * x198;
	const GEN_FLT x204 = x199 * x37;
	const GEN_FLT x205 = -x170 - x196 * x34;
	const GEN_FLT x206 = x29 * (x202 + x203 + x204 + x205);
	const GEN_FLT x207 = x193 + x201 + x206;
	const GEN_FLT x208 = lh_qj * x143;
	const GEN_FLT x209 = x49 * (x191 * x51 + x192 + x6 * ((x8) ? (-x0 * x208) : (0)));
	const GEN_FLT x210 = x194 * x47;
	const GEN_FLT x211 = x168 * x38;
	const GEN_FLT x212 = x196 * x39 + x198 * x31;
	const GEN_FLT x213 = x29 * (x159 + x210 + x211 + x212);
	const GEN_FLT x214 = x44 * (-x195 - x197 + x200);
	const GEN_FLT x215 = x29 * (-x191 * x9 + x191 - x6 * ((x8) ? (-x2 * x208) : (0)));
	const GEN_FLT x216 = x49 * (x173 - x210 - x211 + x212);
	const GEN_FLT x217 = x44 * (-x202 - x203 - x204 + x205);
	const GEN_FLT x218 = -x175 * (2 * x215 + 2 * x216 + 2 * x217) - x181 * (2 * x209 + 2 * x213 + 2 * x214);
	const GEN_FLT x219 = x103 * (x124 * (-x164 * (2 * x193 + 2 * x201 + 2 * x206) + x218) + x207 * x75);
	const GEN_FLT x220 = x219 * x78;
	const GEN_FLT x221 = x219 * x79 + x76 * (-x219 * x77 + x220);
	const GEN_FLT x222 = x219 * x80 + x221 * x76;
	const GEN_FLT x223 = x132 * x218 + x207 * x69;
	const GEN_FLT x224 = x130 * (x215 + x216 + x217) + x61 * (x209 + x213 + x214);
	const GEN_FLT x225 = -x108 * x223 + x224;
	const GEN_FLT x226 =
		x224 - x99 * (x107 * x219 + x112 * x225 +
					  x120 * (x117 * x225 -
							  x119 * (x219 * x81 + x219 * x91 + x222 * x76 +
									  x76 * (x219 * x90 + x222 +
											 x76 * (x219 * x89 + x221 + x76 * (-x118 * x219 + x219 * x88 + x220))))) +
					  x222 * x97 + x223);
	const GEN_FLT x227 = x31 * x36;
	const GEN_FLT x228 = -x227;
	const GEN_FLT x229 = lh_qk * x143;
	const GEN_FLT x230 = x44 * (x227 * x62 + x228 + x6 * ((x8) ? (-x1 * x229) : (0)));
	const GEN_FLT x231 = ((x8) ? (-x146 * x2 + x32) : (0));
	const GEN_FLT x232 = x231 * x31;
	const GEN_FLT x233 = x36 * x5;
	const GEN_FLT x234 = x233 * x37;
	const GEN_FLT x235 = x36 * x46;
	const GEN_FLT x236 = x157 + x210 + x235 * x34;
	const GEN_FLT x237 = x49 * (x232 + x234 + x236);
	const GEN_FLT x238 = x231 * x40;
	const GEN_FLT x239 = x158 * x194;
	const GEN_FLT x240 = x235 * x37;
	const GEN_FLT x241 = x179 - x233 * x34;
	const GEN_FLT x242 = x29 * (x238 + x239 + x240 + x241);
	const GEN_FLT x243 = x230 + x237 + x242;
	const GEN_FLT x244 = x49 * (x227 * x51 + x228 + x6 * ((x8) ? (-x0 * x229) : (0)));
	const GEN_FLT x245 = x231 * x47;
	const GEN_FLT x246 = x148 * x158;
	const GEN_FLT x247 = x168 * x36;
	const GEN_FLT x248 = x195 + x233 * x39;
	const GEN_FLT x249 = x29 * (x245 + x246 + x247 + x248);
	const GEN_FLT x250 = x44 * (-x232 - x234 + x236);
	const GEN_FLT x251 =
		x29 * (-x227 * x9 + x227 - x6 * ((x8) ? (lh_qk * x176 - x143 * lh_qk * (lh_qk * lh_qk)) : (0)));
	const GEN_FLT x252 = x49 * (-x245 - x246 - x247 + x248);
	const GEN_FLT x253 = x44 * (-x238 - x239 - x240 + x241);
	const GEN_FLT x254 = -x175 * (2 * x251 + 2 * x252 + 2 * x253) - x181 * (2 * x244 + 2 * x249 + 2 * x250);
	const GEN_FLT x255 = x103 * (x124 * (-x164 * (2 * x230 + 2 * x237 + 2 * x242) + x254) + x243 * x75);
	const GEN_FLT x256 = x255 * x78;
	const GEN_FLT x257 = x255 * x79 + x76 * (-x255 * x77 + x256);
	const GEN_FLT x258 = x255 * x80 + x257 * x76;
	const GEN_FLT x259 = x132 * x254 + x243 * x69;
	const GEN_FLT x260 = x130 * (x251 + x252 + x253) + x61 * (x244 + x249 + x250);
	const GEN_FLT x261 = -x108 * x259 + x260;
	const GEN_FLT x262 =
		x260 - x99 * (x107 * x255 + x112 * x261 +
					  x120 * (x117 * x261 -
							  x119 * (x255 * x81 + x255 * x91 + x258 * x76 +
									  x76 * (x255 * x90 + x258 +
											 x76 * (x255 * x89 + x257 + x76 * (-x118 * x255 + x255 * x88 + x256))))) +
					  x258 * x97 + x259);
	*(out++) = x121 * x122 + x121;
	*(out++) = -x122 * x129 - x129;
	*(out++) = x122 * x140 + x140;
	*(out++) = x122 * x190 + x190;
	*(out++) = x122 * x226 + x226;
	*(out++) = x122 * x262 + x262;
}

static inline void gen_reproject_axisangle_jac_lh_p(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
													const FLT phase_0, const FLT phase_1, const FLT tilt_0,
													const FLT tilt_1, const FLT curve_0, const FLT curve_1,
													const FLT gibPhase_0, const FLT gibPhase_1, const FLT gibMag_0,
													const FLT gibMag_1) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = 1 - x5;
	const GEN_FLT x7 = 1.0 / x3;
	const GEN_FLT x8 = x4 > 0;
	const GEN_FLT x9 = ((x8) ? (x2 * x7) : (0));
	const GEN_FLT x10 = obj_qi * obj_qi;
	const GEN_FLT x11 = obj_qj * obj_qj;
	const GEN_FLT x12 = obj_qk * obj_qk;
	const GEN_FLT x13 = x10 + x11 + x12;
	const GEN_FLT x14 = sqrt(x13);
	const GEN_FLT x15 = cos(x14);
	const GEN_FLT x16 = 1 - x15;
	const GEN_FLT x17 = 1.0 / x13;
	const GEN_FLT x18 = x14 > 0;
	const GEN_FLT x19 = sin(x14);
	const GEN_FLT x20 = 1.0 / x14;
	const GEN_FLT x21 = ((x18) ? (obj_qi * x20) : (1));
	const GEN_FLT x22 = x19 * x21;
	const GEN_FLT x23 = ((x18) ? (obj_qj * x20) : (0));
	const GEN_FLT x24 = ((x18) ? (obj_qk * x20) : (0));
	const GEN_FLT x25 = x16 * x23 * x24;
	const GEN_FLT x26 = x19 * x23;
	const GEN_FLT x27 = x16 * x21;
	const GEN_FLT x28 = x24 * x27;
	const GEN_FLT x29 = obj_pz + sensor_x * (-x26 + x28) + sensor_y * (x22 + x25) +
						sensor_z * (x15 + x16 * ((x18) ? (x12 * x17) : (0)));
	const GEN_FLT x30 = sin(x4);
	const GEN_FLT x31 = 1.0 / x4;
	const GEN_FLT x32 = lh_qi * x31;
	const GEN_FLT x33 = ((x8) ? (x32) : (1));
	const GEN_FLT x34 = x30 * x33;
	const GEN_FLT x35 = lh_qk * x31;
	const GEN_FLT x36 = ((x8) ? (x35) : (0));
	const GEN_FLT x37 = lh_qj * x31;
	const GEN_FLT x38 = ((x8) ? (x37) : (0));
	const GEN_FLT x39 = x38 * x6;
	const GEN_FLT x40 = x36 * x39;
	const GEN_FLT x41 = x19 * x24;
	const GEN_FLT x42 = x23 * x27;
	const GEN_FLT x43 = obj_py + sensor_x * (x41 + x42) + sensor_y * (x15 + x16 * ((x18) ? (x11 * x17) : (0))) +
						sensor_z * (-x22 + x25);
	const GEN_FLT x44 = x30 * x38;
	const GEN_FLT x45 = x33 * x6;
	const GEN_FLT x46 = x36 * x45;
	const GEN_FLT x47 = obj_px + sensor_x * (x15 + x16 * ((x18) ? (x10 * x17) : (1))) + sensor_y * (-x41 + x42) +
						sensor_z * (x26 + x28);
	const GEN_FLT x48 = -lh_pz - x29 * (x5 + x6 * x9) - x43 * (x34 + x40) - x47 * (-x44 + x46);
	const GEN_FLT x49 = ((x8) ? (x0 * x7) : (1));
	const GEN_FLT x50 = x47 * (x49 * x6 + x5);
	const GEN_FLT x51 = x29 * (x44 + x46);
	const GEN_FLT x52 = x30 * x36;
	const GEN_FLT x53 = x38 * x45;
	const GEN_FLT x54 = x43 * (-x52 + x53);
	const GEN_FLT x55 = lh_px + x50 + x51 + x54;
	const GEN_FLT x56 = x55 * x55;
	const GEN_FLT x57 = x48 * x48;
	const GEN_FLT x58 = x56 + x57;
	const GEN_FLT x59 = 1.0 / x58;
	const GEN_FLT x60 = x48 * x59;
	const GEN_FLT x61 = -lh_px - x50 - x51 - x54;
	const GEN_FLT x62 = ((x8) ? (x1 * x7) : (0));
	const GEN_FLT x63 = x43 * (x5 + x6 * x62);
	const GEN_FLT x64 = x47 * (x52 + x53);
	const GEN_FLT x65 = x29 * (-x34 + x40);
	const GEN_FLT x66 = lh_py + x63 + x64 + x65;
	const GEN_FLT x67 = x66 * x66;
	const GEN_FLT x68 = pow(-x59 * x67 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x69 = tilt_0 * x66 / pow(x58, 3.0 / 2.0);
	const GEN_FLT x70 = x68 * x69;
	const GEN_FLT x71 = x61 * x70;
	const GEN_FLT x72 = atan2(x55, x48);
	const GEN_FLT x73 = tilt_0 / sqrt(x58);
	const GEN_FLT x74 = gibMag_0 * sin(-gibPhase_0 + phase_0 + x72 + asin(x66 * x73) - 1.5707963267948966);
	const GEN_FLT x75 = curve_0 * atan2(x66, x48);
	const GEN_FLT x76 = 2 / (x57 + x67);
	const GEN_FLT x77 = x48 * x76;
	const GEN_FLT x78 = x68 * x73;
	const GEN_FLT x79 = x59 * x61;
	const GEN_FLT x80 = -lh_py - x63 - x64 - x65;
	const GEN_FLT x81 = x76 * x80;
	const GEN_FLT x82 = x48 * x70;
	const GEN_FLT x83 = x30 * x32;
	const GEN_FLT x84 = 2 / ((x3 * x3));
	const GEN_FLT x85 = lh_qi * x84;
	const GEN_FLT x86 = x29 * (-x6 * ((x8) ? (-x2 * x85) : (0)) - x83 * x9 + x83);
	const GEN_FLT x87 = pow(x3, -3.0 / 2.0);
	const GEN_FLT x88 = lh_qi * x87;
	const GEN_FLT x89 = ((x8) ? (-lh_qk * x88) : (0));
	const GEN_FLT x90 = x45 * x89;
	const GEN_FLT x91 = ((x8) ? (-x0 * x87 + x31) : (0));
	const GEN_FLT x92 = x36 * x6;
	const GEN_FLT x93 = x91 * x92;
	const GEN_FLT x94 = x33 * x52;
	const GEN_FLT x95 = x32 * x94;
	const GEN_FLT x96 = ((x8) ? (-lh_qj * x88) : (0));
	const GEN_FLT x97 = x30 * x96;
	const GEN_FLT x98 = x32 * x5;
	const GEN_FLT x99 = x38 * x98 + x97;
	const GEN_FLT x100 = x47 * (-x90 - x93 - x95 + x99);
	const GEN_FLT x101 = x30 * x91;
	const GEN_FLT x102 = x33 * x98;
	const GEN_FLT x103 = -x101 - x102;
	const GEN_FLT x104 = x39 * x89;
	const GEN_FLT x105 = -x104;
	const GEN_FLT x106 = x92 * x96;
	const GEN_FLT x107 = -x106;
	const GEN_FLT x108 = x32 * x44;
	const GEN_FLT x109 = x108 * x36;
	const GEN_FLT x110 = x105 + x107 - x109;
	const GEN_FLT x111 = x43 * (x103 + x110);
	const GEN_FLT x112 = x100 + x111 + x86;
	const GEN_FLT x113 = x112 * x79;
	const GEN_FLT x114 = -x83;
	const GEN_FLT x115 = 2 * x7;
	const GEN_FLT x116 = x47 * (x114 + x49 * x83 + x6 * ((x8) ? (lh_qi * x115 - x84 * lh_qi * (lh_qi * lh_qi)) : (0)));
	const GEN_FLT x117 = x29 * (x90 + x93 + x95 + x99);
	const GEN_FLT x118 = x45 * x96;
	const GEN_FLT x119 = x39 * x91;
	const GEN_FLT x120 = x108 * x33;
	const GEN_FLT x121 = x118 + x119 + x120;
	const GEN_FLT x122 = x30 * x89;
	const GEN_FLT x123 = -x122;
	const GEN_FLT x124 = x36 * x98;
	const GEN_FLT x125 = x123 - x124;
	const GEN_FLT x126 = x43 * (x121 + x125);
	const GEN_FLT x127 = x116 + x117 + x126;
	const GEN_FLT x128 = x127 * x60;
	const GEN_FLT x129 = x6 * ((x8) ? (-x1 * x85) : (0));
	const GEN_FLT x130 = x62 * x83;
	const GEN_FLT x131 = x29 * (x103 + x104 + x106 + x109) + x43 * (x114 + x129 + x130) + x47 * (x121 + x122 + x124);
	const GEN_FLT x132 = (1.0 / 2.0) * x48;
	const GEN_FLT x133 = -x132 * (2 * x100 + 2 * x111 + 2 * x86);
	const GEN_FLT x134 = (1.0 / 2.0) * x55;
	const GEN_FLT x135 = x68 * (x131 * x73 + x69 * (x133 - x134 * (2 * x116 + 2 * x117 + 2 * x126)));
	const GEN_FLT x136 = x30 * x37;
	const GEN_FLT x137 = -x136;
	const GEN_FLT x138 = lh_qj * x84;
	const GEN_FLT x139 = x47 * (x136 * x49 + x137 + x6 * ((x8) ? (-x0 * x138) : (0)));
	const GEN_FLT x140 = ((x8) ? (-lh_qj * lh_qk * x87) : (0));
	const GEN_FLT x141 = x140 * x45;
	const GEN_FLT x142 = x37 * x94;
	const GEN_FLT x143 = ((x8) ? (-x1 * x87 + x31) : (0));
	const GEN_FLT x144 = x37 * x5;
	const GEN_FLT x145 = x143 * x30 + x144 * x38;
	const GEN_FLT x146 = x29 * (x106 + x141 + x142 + x145);
	const GEN_FLT x147 = x143 * x45;
	const GEN_FLT x148 = x39 * x96;
	const GEN_FLT x149 = x37 * x44;
	const GEN_FLT x150 = x149 * x33;
	const GEN_FLT x151 = x147 + x148 + x150;
	const GEN_FLT x152 = x140 * x30;
	const GEN_FLT x153 = x144 * x36;
	const GEN_FLT x154 = -x152 - x153;
	const GEN_FLT x155 = x43 * (x151 + x154);
	const GEN_FLT x156 = x139 + x146 + x155;
	const GEN_FLT x157 = x156 * x60;
	const GEN_FLT x158 = x29 * (-x136 * x9 + x136 - x6 * ((x8) ? (-x138 * x2) : (0)));
	const GEN_FLT x159 = -x141;
	const GEN_FLT x160 = x47 * (x107 - x142 + x145 + x159);
	const GEN_FLT x161 = x144 * x33;
	const GEN_FLT x162 = -x161 - x97;
	const GEN_FLT x163 = x140 * x39;
	const GEN_FLT x164 = x143 * x92;
	const GEN_FLT x165 = x149 * x36;
	const GEN_FLT x166 = -x163 - x164 - x165;
	const GEN_FLT x167 = x43 * (x162 + x166);
	const GEN_FLT x168 = x158 + x160 + x167;
	const GEN_FLT x169 = x168 * x79;
	const GEN_FLT x170 = x136 * x62;
	const GEN_FLT x171 = x6 * ((x8) ? (lh_qj * x115 - x84 * lh_qj * (lh_qj * lh_qj)) : (0));
	const GEN_FLT x172 = x29 * (x162 + x163 + x164 + x165) + x43 * (x137 + x170 + x171) + x47 * (x151 + x152 + x153);
	const GEN_FLT x173 = -x132 * (2 * x158 + 2 * x160 + 2 * x167);
	const GEN_FLT x174 = x68 * (x172 * x73 + x69 * (-x134 * (2 * x139 + 2 * x146 + 2 * x155) + x173));
	const GEN_FLT x175 = x30 * x35;
	const GEN_FLT x176 = -x175;
	const GEN_FLT x177 = lh_qk * x84;
	const GEN_FLT x178 = x47 * (x175 * x49 + x176 + x6 * ((x8) ? (-x0 * x177) : (0)));
	const GEN_FLT x179 = ((x8) ? (-x2 * x87 + x31) : (0));
	const GEN_FLT x180 = x179 * x45;
	const GEN_FLT x181 = x89 * x92;
	const GEN_FLT x182 = x35 * x94;
	const GEN_FLT x183 = x35 * x5;
	const GEN_FLT x184 = x152 + x183 * x38;
	const GEN_FLT x185 = x29 * (x180 + x181 + x182 + x184);
	const GEN_FLT x186 = x35 * x44;
	const GEN_FLT x187 = x186 * x33;
	const GEN_FLT x188 = x104 + x141 + x187;
	const GEN_FLT x189 = x179 * x30;
	const GEN_FLT x190 = x183 * x36;
	const GEN_FLT x191 = -x189 - x190;
	const GEN_FLT x192 = x43 * (x188 + x191);
	const GEN_FLT x193 = x178 + x185 + x192;
	const GEN_FLT x194 = x193 * x60;
	const GEN_FLT x195 = x29 * (-x175 * x9 + x175 - x6 * ((x8) ? (lh_qk * x115 - x84 * lh_qk * (lh_qk * lh_qk)) : (0)));
	const GEN_FLT x196 = x47 * (-x180 - x181 - x182 + x184);
	const GEN_FLT x197 = x183 * x33;
	const GEN_FLT x198 = x123 - x197;
	const GEN_FLT x199 = x179 * x39;
	const GEN_FLT x200 = x140 * x92;
	const GEN_FLT x201 = x186 * x36;
	const GEN_FLT x202 = -x199 - x200 - x201;
	const GEN_FLT x203 = x43 * (x198 + x202);
	const GEN_FLT x204 = x195 + x196 + x203;
	const GEN_FLT x205 = x204 * x79;
	const GEN_FLT x206 = x6 * ((x8) ? (-x1 * x177) : (0));
	const GEN_FLT x207 = x175 * x62;
	const GEN_FLT x208 = x29 * (x198 + x199 + x200 + x201) + x43 * (x176 + x206 + x207) + x47 * (x188 + x189 + x190);
	const GEN_FLT x209 = -x132 * (2 * x195 + 2 * x196 + 2 * x203);
	const GEN_FLT x210 = x68 * (x208 * x73 + x69 * (-x134 * (2 * x178 + 2 * x185 + 2 * x192) + x209));
	const GEN_FLT x211 = curve_1 * x72;
	const GEN_FLT x212 = 2 * x211;
	const GEN_FLT x213 = x57 + x80 * x80;
	const GEN_FLT x214 = 1.0 / x213;
	const GEN_FLT x215 = pow(-x214 * x56 * tilt_1 * tilt_1 + 1, -1.0 / 2.0);
	const GEN_FLT x216 = tilt_1 / sqrt(x213);
	const GEN_FLT x217 = x215 * x216;
	const GEN_FLT x218 =
		gibMag_1 * sin(-gibPhase_1 + phase_1 + asin(x216 * x55) + atan2(x80, x48) - 1.5707963267948966);
	const GEN_FLT x219 = x214 * x48;
	const GEN_FLT x220 = tilt_1 * x55 / pow(x213, 3.0 / 2.0);
	const GEN_FLT x221 = x215 * x220;
	const GEN_FLT x222 = x221 * x80;
	const GEN_FLT x223 = x214 * x66;
	const GEN_FLT x224 = x221 * x48;
	const GEN_FLT x225 = x112 * x223;
	const GEN_FLT x226 = x43 * (-x129 - x130 + x83);
	const GEN_FLT x227 = x29 * (x101 + x102 + x110);
	const GEN_FLT x228 = x47 * (-x118 - x119 - x120 + x125);
	const GEN_FLT x229 = x219 * (x226 + x227 + x228);
	const GEN_FLT x230 = (1.0 / 2.0) * x80;
	const GEN_FLT x231 = x215 * (x127 * x216 + x220 * (x133 - x230 * (2 * x226 + 2 * x227 + 2 * x228)));
	const GEN_FLT x232 = x168 * x223;
	const GEN_FLT x233 = x43 * (x136 - x170 - x171);
	const GEN_FLT x234 = x29 * (x161 + x166 + x97);
	const GEN_FLT x235 = x47 * (-x147 - x148 - x150 + x154);
	const GEN_FLT x236 = x219 * (x233 + x234 + x235);
	const GEN_FLT x237 = x215 * (x156 * x216 + x220 * (x173 - x230 * (2 * x233 + 2 * x234 + 2 * x235)));
	const GEN_FLT x238 = x43 * (x175 - x206 - x207);
	const GEN_FLT x239 = x29 * (x122 + x197 + x202);
	const GEN_FLT x240 = x47 * (x105 + x159 - x187 + x191);
	const GEN_FLT x241 = x219 * (x238 + x239 + x240);
	const GEN_FLT x242 = x204 * x223;
	const GEN_FLT x243 = x215 * (x193 * x216 + x220 * (x209 - x230 * (2 * x238 + 2 * x239 + 2 * x240)));
	*(out++) = -x60 - x71 + x74 * (x60 + x71);
	*(out++) = x74 * x78 + x75 * x77 - x78;
	*(out++) = x74 * (-x79 + x82) - x75 * x81 + x79 - x82;
	*(out++) = -x113 - x128 - x135 + x74 * (x113 + x128 + x135) + x75 * (x112 * x81 + x131 * x77);
	*(out++) = -x157 - x169 - x174 + x74 * (x157 + x169 + x174) + x75 * (x168 * x81 + x172 * x77);
	*(out++) = -x194 - x205 - x210 + x74 * (x194 + x205 + x210) + x75 * (x204 * x81 + x208 * x77);
	*(out++) = x212 * x60 + x217 * x218 - x217;
	*(out++) = x218 * (-x219 + x222) + x219 - x222;
	*(out++) = -x212 * x79 + x218 * (-x223 + x224) + x223 - x224;
	*(out++) = x211 * (2 * x113 + 2 * x128) + x218 * (x225 + x229 + x231) - x225 - x229 - x231;
	*(out++) = x211 * (2 * x157 + 2 * x169) + x218 * (x232 + x236 + x237) - x232 - x236 - x237;
	*(out++) = x211 * (2 * x194 + 2 * x205) + x218 * (x241 + x242 + x243) - x241 - x242 - x243;
}

static inline void gen_reproject_axisangle_axis_x_jac_lh_p(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
														   const FLT phase_0, const FLT tilt_0, const FLT curve_0,
														   const FLT gibPhase_0, const FLT gibMag_0) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = 1 - x5;
	const GEN_FLT x7 = 1.0 / x3;
	const GEN_FLT x8 = x4 > 0;
	const GEN_FLT x9 = ((x8) ? (x2 * x7) : (0));
	const GEN_FLT x10 = obj_qi * obj_qi;
	const GEN_FLT x11 = obj_qj * obj_qj;
	const GEN_FLT x12 = obj_qk * obj_qk;
	const GEN_FLT x13 = x10 + x11 + x12;
	const GEN_FLT x14 = sqrt(x13);
	const GEN_FLT x15 = cos(x14);
	const GEN_FLT x16 = 1 - x15;
	const GEN_FLT x17 = 1.0 / x13;
	const GEN_FLT x18 = x14 > 0;
	const GEN_FLT x19 = sin(x14);
	const GEN_FLT x20 = 1.0 / x14;
	const GEN_FLT x21 = ((x18) ? (obj_qi * x20) : (1));
	const GEN_FLT x22 = x19 * x21;
	const GEN_FLT x23 = ((x18) ? (obj_qj * x20) : (0));
	const GEN_FLT x24 = ((x18) ? (obj_qk * x20) : (0));
	const GEN_FLT x25 = x16 * x23 * x24;
	const GEN_FLT x26 = x19 * x23;
	const GEN_FLT x27 = x16 * x21;
	const GEN_FLT x28 = x24 * x27;
	const GEN_FLT x29 = obj_pz + sensor_x * (-x26 + x28) + sensor_y * (x22 + x25) +
						sensor_z * (x15 + x16 * ((x18) ? (x12 * x17) : (0)));
	const GEN_FLT x30 = sin(x4);
	const GEN_FLT x31 = 1.0 / x4;
	const GEN_FLT x32 = lh_qi * x31;
	const GEN_FLT x33 = ((x8) ? (x32) : (1));
	const GEN_FLT x34 = x30 * x33;
	const GEN_FLT x35 = lh_qk * x31;
	const GEN_FLT x36 = ((x8) ? (x35) : (0));
	const GEN_FLT x37 = lh_qj * x31;
	const GEN_FLT x38 = ((x8) ? (x37) : (0));
	const GEN_FLT x39 = x38 * x6;
	const GEN_FLT x40 = x36 * x39;
	const GEN_FLT x41 = x19 * x24;
	const GEN_FLT x42 = x23 * x27;
	const GEN_FLT x43 = obj_py + sensor_x * (x41 + x42) + sensor_y * (x15 + x16 * ((x18) ? (x11 * x17) : (0))) +
						sensor_z * (-x22 + x25);
	const GEN_FLT x44 = x30 * x38;
	const GEN_FLT x45 = x33 * x6;
	const GEN_FLT x46 = x36 * x45;
	const GEN_FLT x47 = obj_px + sensor_x * (x15 + x16 * ((x18) ? (x10 * x17) : (1))) + sensor_y * (-x41 + x42) +
						sensor_z * (x26 + x28);
	const GEN_FLT x48 = -lh_pz - x29 * (x5 + x6 * x9) - x43 * (x34 + x40) - x47 * (-x44 + x46);
	const GEN_FLT x49 = ((x8) ? (x0 * x7) : (1));
	const GEN_FLT x50 = x47 * (x49 * x6 + x5);
	const GEN_FLT x51 = x29 * (x44 + x46);
	const GEN_FLT x52 = x30 * x36;
	const GEN_FLT x53 = x38 * x45;
	const GEN_FLT x54 = x43 * (-x52 + x53);
	const GEN_FLT x55 = lh_px + x50 + x51 + x54;
	const GEN_FLT x56 = x48 * x48;
	const GEN_FLT x57 = x55 * x55 + x56;
	const GEN_FLT x58 = 1.0 / x57;
	const GEN_FLT x59 = x48 * x58;
	const GEN_FLT x60 = -lh_px - x50 - x51 - x54;
	const GEN_FLT x61 = ((x8) ? (x1 * x7) : (0));
	const GEN_FLT x62 = x43 * (x5 + x6 * x61);
	const GEN_FLT x63 = x47 * (x52 + x53);
	const GEN_FLT x64 = x29 * (-x34 + x40);
	const GEN_FLT x65 = lh_py + x62 + x63 + x64;
	const GEN_FLT x66 = x65 * x65;
	const GEN_FLT x67 = pow(-x58 * x66 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x68 = tilt_0 * x65 / pow(x57, 3.0 / 2.0);
	const GEN_FLT x69 = x67 * x68;
	const GEN_FLT x70 = x60 * x69;
	const GEN_FLT x71 = tilt_0 / sqrt(x57);
	const GEN_FLT x72 = gibMag_0 * sin(-gibPhase_0 + phase_0 + asin(x65 * x71) + atan2(x55, x48) - 1.5707963267948966);
	const GEN_FLT x73 = curve_0 * atan2(x65, x48);
	const GEN_FLT x74 = 2 / (x56 + x66);
	const GEN_FLT x75 = x48 * x74;
	const GEN_FLT x76 = x67 * x71;
	const GEN_FLT x77 = x58 * x60;
	const GEN_FLT x78 = x74 * (-lh_py - x62 - x63 - x64);
	const GEN_FLT x79 = x48 * x69;
	const GEN_FLT x80 = x30 * x32;
	const GEN_FLT x81 = 2 / ((x3 * x3));
	const GEN_FLT x82 = lh_qi * x81;
	const GEN_FLT x83 = x29 * (-x6 * ((x8) ? (-x2 * x82) : (0)) - x80 * x9 + x80);
	const GEN_FLT x84 = pow(x3, -3.0 / 2.0);
	const GEN_FLT x85 = lh_qi * x84;
	const GEN_FLT x86 = ((x8) ? (-lh_qk * x85) : (0));
	const GEN_FLT x87 = x45 * x86;
	const GEN_FLT x88 = ((x8) ? (-x0 * x84 + x31) : (0));
	const GEN_FLT x89 = x36 * x6;
	const GEN_FLT x90 = x88 * x89;
	const GEN_FLT x91 = x33 * x52;
	const GEN_FLT x92 = x32 * x91;
	const GEN_FLT x93 = ((x8) ? (-lh_qj * x85) : (0));
	const GEN_FLT x94 = x30 * x93;
	const GEN_FLT x95 = x32 * x5;
	const GEN_FLT x96 = x38 * x95 + x94;
	const GEN_FLT x97 = x47 * (-x87 - x90 - x92 + x96);
	const GEN_FLT x98 = x39 * x86;
	const GEN_FLT x99 = x89 * x93;
	const GEN_FLT x100 = -x99;
	const GEN_FLT x101 = x32 * x44;
	const GEN_FLT x102 = x101 * x36;
	const GEN_FLT x103 = -x30 * x88 - x33 * x95;
	const GEN_FLT x104 = x43 * (x100 - x102 + x103 - x98);
	const GEN_FLT x105 = x104 + x83 + x97;
	const GEN_FLT x106 = x105 * x77;
	const GEN_FLT x107 = -x80;
	const GEN_FLT x108 = 2 * x7;
	const GEN_FLT x109 = x47 * (x107 + x49 * x80 + x6 * ((x8) ? (lh_qi * x108 - x81 * lh_qi * (lh_qi * lh_qi)) : (0)));
	const GEN_FLT x110 = x29 * (x87 + x90 + x92 + x96);
	const GEN_FLT x111 = x30 * x86;
	const GEN_FLT x112 = -x111;
	const GEN_FLT x113 = x36 * x95;
	const GEN_FLT x114 = x101 * x33 + x39 * x88 + x45 * x93;
	const GEN_FLT x115 = x43 * (x112 - x113 + x114);
	const GEN_FLT x116 = x59 * (x109 + x110 + x115);
	const GEN_FLT x117 = x29 * (x102 + x103 + x98 + x99) + x43 * (x107 + x6 * ((x8) ? (-x1 * x82) : (0)) + x61 * x80) +
						 x47 * (x111 + x113 + x114);
	const GEN_FLT x118 = (1.0 / 2.0) * x48;
	const GEN_FLT x119 = (1.0 / 2.0) * x55;
	const GEN_FLT x120 =
		x67 * (x117 * x71 + x68 * (-x118 * (2 * x104 + 2 * x83 + 2 * x97) - x119 * (2 * x109 + 2 * x110 + 2 * x115)));
	const GEN_FLT x121 = x30 * x37;
	const GEN_FLT x122 = -x121;
	const GEN_FLT x123 = lh_qj * x81;
	const GEN_FLT x124 = x47 * (x121 * x49 + x122 + x6 * ((x8) ? (-x0 * x123) : (0)));
	const GEN_FLT x125 = ((x8) ? (-lh_qj * lh_qk * x84) : (0));
	const GEN_FLT x126 = x125 * x45;
	const GEN_FLT x127 = x37 * x91;
	const GEN_FLT x128 = ((x8) ? (-x1 * x84 + x31) : (0));
	const GEN_FLT x129 = x37 * x5;
	const GEN_FLT x130 = x128 * x30 + x129 * x38;
	const GEN_FLT x131 = x29 * (x126 + x127 + x130 + x99);
	const GEN_FLT x132 = x125 * x30;
	const GEN_FLT x133 = x129 * x36;
	const GEN_FLT x134 = x37 * x44;
	const GEN_FLT x135 = x128 * x45 + x134 * x33 + x39 * x93;
	const GEN_FLT x136 = x43 * (-x132 - x133 + x135);
	const GEN_FLT x137 = x59 * (x124 + x131 + x136);
	const GEN_FLT x138 = x29 * (-x121 * x9 + x121 - x6 * ((x8) ? (-x123 * x2) : (0)));
	const GEN_FLT x139 = x47 * (x100 - x126 - x127 + x130);
	const GEN_FLT x140 = x125 * x39;
	const GEN_FLT x141 = x128 * x89;
	const GEN_FLT x142 = x134 * x36;
	const GEN_FLT x143 = -x129 * x33 - x94;
	const GEN_FLT x144 = x43 * (-x140 - x141 - x142 + x143);
	const GEN_FLT x145 = x138 + x139 + x144;
	const GEN_FLT x146 = x145 * x77;
	const GEN_FLT x147 =
		x29 * (x140 + x141 + x142 + x143) +
		x43 * (x121 * x61 + x122 + x6 * ((x8) ? (lh_qj * x108 - x81 * lh_qj * (lh_qj * lh_qj)) : (0))) +
		x47 * (x132 + x133 + x135);
	const GEN_FLT x148 =
		x67 * (x147 * x71 + x68 * (-x118 * (2 * x138 + 2 * x139 + 2 * x144) - x119 * (2 * x124 + 2 * x131 + 2 * x136)));
	const GEN_FLT x149 = x30 * x35;
	const GEN_FLT x150 = -x149;
	const GEN_FLT x151 = lh_qk * x81;
	const GEN_FLT x152 = x47 * (x149 * x49 + x150 + x6 * ((x8) ? (-x0 * x151) : (0)));
	const GEN_FLT x153 = ((x8) ? (-x2 * x84 + x31) : (0));
	const GEN_FLT x154 = x153 * x45;
	const GEN_FLT x155 = x86 * x89;
	const GEN_FLT x156 = x35 * x91;
	const GEN_FLT x157 = x35 * x5;
	const GEN_FLT x158 = x132 + x157 * x38;
	const GEN_FLT x159 = x29 * (x154 + x155 + x156 + x158);
	const GEN_FLT x160 = x153 * x30;
	const GEN_FLT x161 = x157 * x36;
	const GEN_FLT x162 = x35 * x44;
	const GEN_FLT x163 = x126 + x162 * x33 + x98;
	const GEN_FLT x164 = x43 * (-x160 - x161 + x163);
	const GEN_FLT x165 = x59 * (x152 + x159 + x164);
	const GEN_FLT x166 = x29 * (-x149 * x9 + x149 - x6 * ((x8) ? (lh_qk * x108 - x81 * lh_qk * (lh_qk * lh_qk)) : (0)));
	const GEN_FLT x167 = x47 * (-x154 - x155 - x156 + x158);
	const GEN_FLT x168 = x153 * x39;
	const GEN_FLT x169 = x125 * x89;
	const GEN_FLT x170 = x162 * x36;
	const GEN_FLT x171 = x112 - x157 * x33;
	const GEN_FLT x172 = x43 * (-x168 - x169 - x170 + x171);
	const GEN_FLT x173 = x166 + x167 + x172;
	const GEN_FLT x174 = x173 * x77;
	const GEN_FLT x175 = x29 * (x168 + x169 + x170 + x171) +
						 x43 * (x149 * x61 + x150 + x6 * ((x8) ? (-x1 * x151) : (0))) + x47 * (x160 + x161 + x163);
	const GEN_FLT x176 =
		x67 * (x175 * x71 + x68 * (-x118 * (2 * x166 + 2 * x167 + 2 * x172) - x119 * (2 * x152 + 2 * x159 + 2 * x164)));
	*(out++) = -x59 - x70 + x72 * (x59 + x70);
	*(out++) = x72 * x76 + x73 * x75 - x76;
	*(out++) = x72 * (-x77 + x79) - x73 * x78 + x77 - x79;
	*(out++) = -x106 - x116 - x120 + x72 * (x106 + x116 + x120) + x73 * (x105 * x78 + x117 * x75);
	*(out++) = -x137 - x146 - x148 + x72 * (x137 + x146 + x148) + x73 * (x145 * x78 + x147 * x75);
	*(out++) = -x165 - x174 - x176 + x72 * (x165 + x174 + x176) + x73 * (x173 * x78 + x175 * x75);
}

static inline void gen_reproject_axisangle_axis_y_jac_lh_p(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
														   const FLT phase_0, const FLT tilt_0, const FLT curve_0,
														   const FLT gibPhase_0, const FLT gibMag_0) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = 1 - x5;
	const GEN_FLT x7 = 1.0 / x3;
	const GEN_FLT x8 = x4 > 0;
	const GEN_FLT x9 = ((x8) ? (x0 * x7) : (1));
	const GEN_FLT x10 = obj_qi * obj_qi;
	const GEN_FLT x11 = obj_qj * obj_qj;
	const GEN_FLT x12 = obj_qk * obj_qk;
	const GEN_FLT x13 = x10 + x11 + x12;
	const GEN_FLT x14 = sqrt(x13);
	const GEN_FLT x15 = cos(x14);
	const GEN_FLT x16 = 1 - x15;
	const GEN_FLT x17 = 1.0 / x13;
	const GEN_FLT x18 = x14 > 0;
	const GEN_FLT x19 = sin(x14);
	const GEN_FLT x20 = 1.0 / x14;
	const GEN_FLT x21 = ((x18) ? (obj_qj * x20) : (0));
	const GEN_FLT x22 = x19 * x21;
	const GEN_FLT x23 = ((x18) ? (obj_qi * x20) : (1));
	const GEN_FLT x24 = ((x18) ? (obj_qk * x20) : (0));
	const GEN_FLT x25 = x16 * x23 * x24;
	const GEN_FLT x26 = x19 * x24;
	const GEN_FLT x27 = x16 * x21;
	const GEN_FLT x28 = x23 * x27;
	const GEN_FLT x29 = obj_px + sensor_x * (x15 + x16 * ((x18) ? (x10 * x17) : (1))) + sensor_y * (-x26 + x28) +
						sensor_z * (x22 + x25);
	const GEN_FLT x30 = x29 * (x5 + x6 * x9);
	const GEN_FLT x31 = sin(x4);
	const GEN_FLT x32 = 1.0 / x4;
	const GEN_FLT x33 = lh_qj * x32;
	const GEN_FLT x34 = ((x8) ? (x33) : (0));
	const GEN_FLT x35 = x31 * x34;
	const GEN_FLT x36 = lh_qk * x32;
	const GEN_FLT x37 = ((x8) ? (x36) : (0));
	const GEN_FLT x38 = lh_qi * x32;
	const GEN_FLT x39 = ((x8) ? (x38) : (1));
	const GEN_FLT x40 = x39 * x6;
	const GEN_FLT x41 = x37 * x40;
	const GEN_FLT x42 = x19 * x23;
	const GEN_FLT x43 = x24 * x27;
	const GEN_FLT x44 = obj_pz + sensor_x * (-x22 + x25) + sensor_y * (x42 + x43) +
						sensor_z * (x15 + x16 * ((x18) ? (x12 * x17) : (0)));
	const GEN_FLT x45 = x44 * (x35 + x41);
	const GEN_FLT x46 = x31 * x37;
	const GEN_FLT x47 = x34 * x6;
	const GEN_FLT x48 = x39 * x47;
	const GEN_FLT x49 = obj_py + sensor_x * (x26 + x28) + sensor_y * (x15 + x16 * ((x18) ? (x11 * x17) : (0))) +
						sensor_z * (-x42 + x43);
	const GEN_FLT x50 = x49 * (-x46 + x48);
	const GEN_FLT x51 = lh_px + x30 + x45 + x50;
	const GEN_FLT x52 = ((x8) ? (x2 * x7) : (0));
	const GEN_FLT x53 = x31 * x39;
	const GEN_FLT x54 = x37 * x47;
	const GEN_FLT x55 = -lh_pz - x29 * (-x35 + x41) - x44 * (x5 + x52 * x6) - x49 * (x53 + x54);
	const GEN_FLT x56 = curve_0 * atan2(x51, x55);
	const GEN_FLT x57 = x51 * x51;
	const GEN_FLT x58 = x55 * x55;
	const GEN_FLT x59 = 2 / (x57 + x58);
	const GEN_FLT x60 = x55 * x59;
	const GEN_FLT x61 = ((x8) ? (x1 * x7) : (0));
	const GEN_FLT x62 = x49 * (x5 + x6 * x61);
	const GEN_FLT x63 = x29 * (x46 + x48);
	const GEN_FLT x64 = x44 * (-x53 + x54);
	const GEN_FLT x65 = -lh_py - x62 - x63 - x64;
	const GEN_FLT x66 = x58 + x65 * x65;
	const GEN_FLT x67 = 1.0 / x66;
	const GEN_FLT x68 = pow(-x57 * x67 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x69 = tilt_0 / sqrt(x66);
	const GEN_FLT x70 = x68 * x69;
	const GEN_FLT x71 = gibMag_0 * sin(-gibPhase_0 + phase_0 + asin(x51 * x69) + atan2(x65, x55) - 1.5707963267948966);
	const GEN_FLT x72 = x55 * x67;
	const GEN_FLT x73 = tilt_0 * x51 / pow(x66, 3.0 / 2.0);
	const GEN_FLT x74 = x68 * x73;
	const GEN_FLT x75 = x65 * x74;
	const GEN_FLT x76 = x67 * (lh_py + x62 + x63 + x64);
	const GEN_FLT x77 = x59 * (-lh_px - x30 - x45 - x50);
	const GEN_FLT x78 = x55 * x74;
	const GEN_FLT x79 = x31 * x38;
	const GEN_FLT x80 = 2 / ((x3 * x3));
	const GEN_FLT x81 = lh_qi * x80;
	const GEN_FLT x82 = x44 * (-x52 * x79 - x6 * ((x8) ? (-x2 * x81) : (0)) + x79);
	const GEN_FLT x83 = pow(x3, -3.0 / 2.0);
	const GEN_FLT x84 = lh_qi * x83;
	const GEN_FLT x85 = ((x8) ? (-lh_qk * x84) : (0));
	const GEN_FLT x86 = x40 * x85;
	const GEN_FLT x87 = ((x8) ? (-x0 * x83 + x32) : (0));
	const GEN_FLT x88 = x37 * x6;
	const GEN_FLT x89 = x87 * x88;
	const GEN_FLT x90 = x39 * x46;
	const GEN_FLT x91 = x38 * x90;
	const GEN_FLT x92 = ((x8) ? (-lh_qj * x84) : (0));
	const GEN_FLT x93 = x31 * x92;
	const GEN_FLT x94 = x38 * x5;
	const GEN_FLT x95 = x34 * x94 + x93;
	const GEN_FLT x96 = x29 * (-x86 - x89 - x91 + x95);
	const GEN_FLT x97 = x31 * x87;
	const GEN_FLT x98 = x39 * x94;
	const GEN_FLT x99 = x47 * x85;
	const GEN_FLT x100 = -x99;
	const GEN_FLT x101 = x88 * x92;
	const GEN_FLT x102 = -x101;
	const GEN_FLT x103 = x34 * x38;
	const GEN_FLT x104 = x100 + x102 - x103 * x46;
	const GEN_FLT x105 = x49 * (x104 - x97 - x98);
	const GEN_FLT x106 = x105 + x82 + x96;
	const GEN_FLT x107 = x106 * x76;
	const GEN_FLT x108 = x49 * (-x6 * ((x8) ? (-x1 * x81) : (0)) - x61 * x79 + x79);
	const GEN_FLT x109 = x44 * (x104 + x97 + x98);
	const GEN_FLT x110 = x40 * x92;
	const GEN_FLT x111 = x47 * x87;
	const GEN_FLT x112 = x103 * x53;
	const GEN_FLT x113 = x31 * x85;
	const GEN_FLT x114 = -x113;
	const GEN_FLT x115 = x114 - x37 * x94;
	const GEN_FLT x116 = x29 * (-x110 - x111 - x112 + x115);
	const GEN_FLT x117 = x72 * (x108 + x109 + x116);
	const GEN_FLT x118 = 2 * x7;
	const GEN_FLT x119 = x29 * (x6 * ((x8) ? (lh_qi * x118 - x80 * lh_qi * (lh_qi * lh_qi)) : (0)) + x79 * x9 - x79) +
						 x44 * (x86 + x89 + x91 + x95) + x49 * (x110 + x111 + x112 + x115);
	const GEN_FLT x120 = (1.0 / 2.0) * x65;
	const GEN_FLT x121 = (1.0 / 2.0) * x55;
	const GEN_FLT x122 =
		x68 * (x119 * x69 + x73 * (-x120 * (2 * x108 + 2 * x109 + 2 * x116) - x121 * (2 * x105 + 2 * x82 + 2 * x96)));
	const GEN_FLT x123 = x31 * x33;
	const GEN_FLT x124 = lh_qj * x80;
	const GEN_FLT x125 = x44 * (-x123 * x52 + x123 - x6 * ((x8) ? (-x124 * x2) : (0)));
	const GEN_FLT x126 = ((x8) ? (-lh_qj * lh_qk * x83) : (0));
	const GEN_FLT x127 = x126 * x40;
	const GEN_FLT x128 = -x127;
	const GEN_FLT x129 = x33 * x90;
	const GEN_FLT x130 = ((x8) ? (-x1 * x83 + x32) : (0));
	const GEN_FLT x131 = x33 * x5;
	const GEN_FLT x132 = x130 * x31 + x131 * x34;
	const GEN_FLT x133 = x29 * (x102 + x128 - x129 + x132);
	const GEN_FLT x134 = x131 * x39;
	const GEN_FLT x135 = x33 * x34;
	const GEN_FLT x136 = -x126 * x47 - x130 * x88 - x135 * x46;
	const GEN_FLT x137 = x49 * (-x134 + x136 - x93);
	const GEN_FLT x138 = x125 + x133 + x137;
	const GEN_FLT x139 = x138 * x76;
	const GEN_FLT x140 =
		x49 * (-x123 * x61 + x123 - x6 * ((x8) ? (lh_qj * x118 - x80 * lh_qj * (lh_qj * lh_qj)) : (0)));
	const GEN_FLT x141 = x44 * (x134 + x136 + x93);
	const GEN_FLT x142 = x130 * x40;
	const GEN_FLT x143 = x47 * x92;
	const GEN_FLT x144 = x135 * x53;
	const GEN_FLT x145 = x126 * x31;
	const GEN_FLT x146 = -x131 * x37 - x145;
	const GEN_FLT x147 = x29 * (-x142 - x143 - x144 + x146);
	const GEN_FLT x148 = x72 * (x140 + x141 + x147);
	const GEN_FLT x149 = x29 * (x123 * x9 - x123 + x6 * ((x8) ? (-x0 * x124) : (0))) +
						 x44 * (x101 + x127 + x129 + x132) + x49 * (x142 + x143 + x144 + x146);
	const GEN_FLT x150 =
		x68 * (x149 * x69 + x73 * (-x120 * (2 * x140 + 2 * x141 + 2 * x147) - x121 * (2 * x125 + 2 * x133 + 2 * x137)));
	const GEN_FLT x151 = x31 * x36;
	const GEN_FLT x152 = lh_qk * x80;
	const GEN_FLT x153 = x49 * (-x151 * x61 + x151 - x6 * ((x8) ? (-x1 * x152) : (0)));
	const GEN_FLT x154 = x36 * x5;
	const GEN_FLT x155 = x154 * x39;
	const GEN_FLT x156 = ((x8) ? (-x2 * x83 + x32) : (0));
	const GEN_FLT x157 = x34 * x36;
	const GEN_FLT x158 = -x126 * x88 - x156 * x47 - x157 * x46;
	const GEN_FLT x159 = x44 * (x113 + x155 + x158);
	const GEN_FLT x160 = x157 * x53;
	const GEN_FLT x161 = -x154 * x37 - x156 * x31;
	const GEN_FLT x162 = x29 * (x100 + x128 - x160 + x161);
	const GEN_FLT x163 = x72 * (x153 + x159 + x162);
	const GEN_FLT x164 =
		x44 * (-x151 * x52 + x151 - x6 * ((x8) ? (lh_qk * x118 - x80 * lh_qk * (lh_qk * lh_qk)) : (0)));
	const GEN_FLT x165 = x156 * x40;
	const GEN_FLT x166 = x85 * x88;
	const GEN_FLT x167 = x36 * x90;
	const GEN_FLT x168 = x145 + x154 * x34;
	const GEN_FLT x169 = x29 * (-x165 - x166 - x167 + x168);
	const GEN_FLT x170 = x49 * (x114 - x155 + x158);
	const GEN_FLT x171 = x164 + x169 + x170;
	const GEN_FLT x172 = x171 * x76;
	const GEN_FLT x173 = x29 * (x151 * x9 - x151 + x6 * ((x8) ? (-x0 * x152) : (0))) +
						 x44 * (x165 + x166 + x167 + x168) + x49 * (x127 + x160 + x161 + x99);
	const GEN_FLT x174 =
		x68 * (x173 * x69 + x73 * (-x120 * (2 * x153 + 2 * x159 + 2 * x162) - x121 * (2 * x164 + 2 * x169 + 2 * x170)));
	*(out++) = x56 * x60 + x70 * x71 - x70;
	*(out++) = x71 * (-x72 + x75) + x72 - x75;
	*(out++) = -x56 * x77 + x71 * (-x76 + x78) + x76 - x78;
	*(out++) = -x107 - x117 - x122 + x56 * (x106 * x77 + x119 * x60) + x71 * (x107 + x117 + x122);
	*(out++) = -x139 - x148 - x150 + x56 * (x138 * x77 + x149 * x60) + x71 * (x139 + x148 + x150);
	*(out++) = -x163 - x172 - x174 + x56 * (x171 * x77 + x173 * x60) + x71 * (x163 + x172 + x174);
}

static inline void gen_reproject_axisangle_jac_obj_p_gen2(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
														  const FLT phase_0, const FLT phase_1, const FLT tilt_0,
														  const FLT tilt_1, const FLT curve_0, const FLT curve_1,
														  const FLT gibPhase_0, const FLT gibPhase_1,
														  const FLT gibMag_0, const FLT gibMag_1, const FLT ogeePhase_0,
														  const FLT ogeePhase_1, const FLT ogeeMag_0,
														  const FLT ogeeMag_1) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = -x5;
	const GEN_FLT x7 = x6 + 1;
	const GEN_FLT x8 = 1.0 / x3;
	const GEN_FLT x9 = x4 > 0;
	const GEN_FLT x10 = x7 * ((x9) ? (x1 * x8) : (0));
	const GEN_FLT x11 = x10 + x5;
	const GEN_FLT x12 = obj_qi * obj_qi;
	const GEN_FLT x13 = obj_qj * obj_qj;
	const GEN_FLT x14 = obj_qk * obj_qk;
	const GEN_FLT x15 = x12 + x13 + x14;
	const GEN_FLT x16 = sqrt(x15);
	const GEN_FLT x17 = cos(x16);
	const GEN_FLT x18 = 1 - x17;
	const GEN_FLT x19 = 1.0 / x15;
	const GEN_FLT x20 = x16 > 0;
	const GEN_FLT x21 = ((x20) ? (x13 * x19) : (0));
	const GEN_FLT x22 = sin(x16);
	const GEN_FLT x23 = 1.0 / x16;
	const GEN_FLT x24 = obj_qk * x23;
	const GEN_FLT x25 = ((x20) ? (x24) : (0));
	const GEN_FLT x26 = x22 * x25;
	const GEN_FLT x27 = obj_qj * x23;
	const GEN_FLT x28 = ((x20) ? (x27) : (0));
	const GEN_FLT x29 = obj_qi * x23;
	const GEN_FLT x30 = ((x20) ? (x29) : (1));
	const GEN_FLT x31 = x18 * x30;
	const GEN_FLT x32 = x28 * x31;
	const GEN_FLT x33 = x22 * x30;
	const GEN_FLT x34 = x18 * x28;
	const GEN_FLT x35 = x25 * x34;
	const GEN_FLT x36 = obj_py + sensor_x * (x26 + x32) + sensor_y * (x17 + x18 * x21) + sensor_z * (-x33 + x35);
	const GEN_FLT x37 = sin(x4);
	const GEN_FLT x38 = 1.0 / x4;
	const GEN_FLT x39 = ((x9) ? (lh_qk * x38) : (0));
	const GEN_FLT x40 = x37 * x39;
	const GEN_FLT x41 = ((x9) ? (lh_qj * x38) : (0));
	const GEN_FLT x42 = ((x9) ? (lh_qi * x38) : (1));
	const GEN_FLT x43 = x42 * x7;
	const GEN_FLT x44 = x41 * x43;
	const GEN_FLT x45 = x40 + x44;
	const GEN_FLT x46 = ((x20) ? (x12 * x19) : (1));
	const GEN_FLT x47 = x22 * x28;
	const GEN_FLT x48 = x25 * x31;
	const GEN_FLT x49 = obj_px + sensor_x * (x17 + x18 * x46) + sensor_y * (-x26 + x32) + sensor_z * (x47 + x48);
	const GEN_FLT x50 = x37 * x42;
	const GEN_FLT x51 = -x50;
	const GEN_FLT x52 = x39 * x41 * x7;
	const GEN_FLT x53 = x51 + x52;
	const GEN_FLT x54 = ((x20) ? (x14 * x19) : (0));
	const GEN_FLT x55 = obj_pz + sensor_x * (-x47 + x48) + sensor_y * (x33 + x35) + sensor_z * (x17 + x18 * x54);
	const GEN_FLT x56 = lh_py + x11 * x36 + x45 * x49 + x53 * x55;
	const GEN_FLT x57 = x7 * ((x9) ? (x0 * x8) : (1));
	const GEN_FLT x58 = x5 + x57;
	const GEN_FLT x59 = x37 * x41;
	const GEN_FLT x60 = x39 * x43;
	const GEN_FLT x61 = x59 + x60;
	const GEN_FLT x62 = -x40 + x44;
	const GEN_FLT x63 = lh_px + x36 * x62 + x49 * x58 + x55 * x61;
	const GEN_FLT x64 = x7 * ((x9) ? (x2 * x8) : (0));
	const GEN_FLT x65 = x55 * (x5 + x64);
	const GEN_FLT x66 = x36 * (x50 + x52);
	const GEN_FLT x67 = x49 * (-x59 + x60);
	const GEN_FLT x68 = -lh_pz - x65 - x66 - x67;
	const GEN_FLT x69 = x63 * x63 + x68 * x68;
	const GEN_FLT x70 = pow(x69, -1.0 / 2.0);
	const GEN_FLT x71 = tilt_0 + 0.52359877559829882;
	const GEN_FLT x72 = tan(x71);
	const GEN_FLT x73 = x70 * x72;
	const GEN_FLT x74 = x56 * x73;
	const GEN_FLT x75 = x56 * x56;
	const GEN_FLT x76 = x69 + x75;
	const GEN_FLT x77 = pow(x76, -1.0 / 2.0);
	const GEN_FLT x78 = cos(x71);
	const GEN_FLT x79 = 1.0 / x78;
	const GEN_FLT x80 = x77 * x79;
	const GEN_FLT x81 = asin(x56 * x80);
	const GEN_FLT x82 = 8.0108022e-6 * x81;
	const GEN_FLT x83 = -x82 - 8.0108022e-6;
	const GEN_FLT x84 = x81 * x83 + 0.0028679863;
	const GEN_FLT x85 = x81 * x84 + 5.3685255000000001e-6;
	const GEN_FLT x86 = x81 * x85 + 0.0076069798000000001;
	const GEN_FLT x87 = x81 * x81;
	const GEN_FLT x88 = atan2(x68, x63);
	const GEN_FLT x89 = ogeePhase_0 + x88 - asin(x74);
	const GEN_FLT x90 = ogeeMag_0 * sin(x89);
	const GEN_FLT x91 = curve_0 + x90;
	const GEN_FLT x92 = x81 * x86;
	const GEN_FLT x93 = -1.60216044e-5 * x81 - 8.0108022e-6;
	const GEN_FLT x94 = x81 * x93 + x84;
	const GEN_FLT x95 = x81 * x94 + x85;
	const GEN_FLT x96 = x81 * x95 + x86;
	const GEN_FLT x97 = sin(x71);
	const GEN_FLT x98 = x97 * (x81 * x96 + x92);
	const GEN_FLT x99 = x78 - x91 * x98;
	const GEN_FLT x100 = 1.0 / x99;
	const GEN_FLT x101 = x100 * x91;
	const GEN_FLT x102 = x101 * x87;
	const GEN_FLT x103 = x102 * x86 + x74;
	const GEN_FLT x104 = pow(1 - x103 * x103, -1.0 / 2.0);
	const GEN_FLT x105 = x75 / x76;
	const GEN_FLT x106 = pow(-x105 / (x78 * x78) + 1, -1.0 / 2.0);
	const GEN_FLT x107 = 2 * x40;
	const GEN_FLT x108 = 2 * x44;
	const GEN_FLT x109 = (1.0 / 2.0) * x56;
	const GEN_FLT x110 = 2 * x5;
	const GEN_FLT x111 = (1.0 / 2.0) * x63;
	const GEN_FLT x112 = 2 * x59;
	const GEN_FLT x113 = 2 * x60;
	const GEN_FLT x114 = (1.0 / 2.0) * x68;
	const GEN_FLT x115 = -x111 * (x110 + 2 * x57) - x114 * (x112 - x113);
	const GEN_FLT x116 = -x109 * (x107 + x108) + x115;
	const GEN_FLT x117 = x56 / pow(x76, 3.0 / 2.0);
	const GEN_FLT x118 = x117 * x79;
	const GEN_FLT x119 = x106 * (x116 * x118 + x45 * x80);
	const GEN_FLT x120 = x119 * x83;
	const GEN_FLT x121 = x119 * x84 + x81 * (-x119 * x82 + x120);
	const GEN_FLT x122 = x119 * x85 + x121 * x81;
	const GEN_FLT x123 = 1.0 / x69;
	const GEN_FLT x124 = x123 * x75;
	const GEN_FLT x125 = pow(-x124 * x72 * x72 + 1, -1.0 / 2.0);
	const GEN_FLT x126 = x56 / pow(x69, 3.0 / 2.0);
	const GEN_FLT x127 = x126 * x72;
	const GEN_FLT x128 = x115 * x127 + x45 * x73;
	const GEN_FLT x129 = x123 * (lh_pz + x65 + x66 + x67);
	const GEN_FLT x130 = x59 - x60;
	const GEN_FLT x131 = x123 * x63;
	const GEN_FLT x132 = x129 * x58 + x130 * x131;
	const GEN_FLT x133 = -x125 * x128 + x132;
	const GEN_FLT x134 = ogeeMag_0 * cos(x89);
	const GEN_FLT x135 = x134 * x98;
	const GEN_FLT x136 = 2.40324066e-5 * x81;
	const GEN_FLT x137 = x97 * (-curve_0 - x90);
	const GEN_FLT x138 = x86 * x87;
	const GEN_FLT x139 = x138 * x91 / ((x99 * x99));
	const GEN_FLT x140 = x100 * x134 * x138;
	const GEN_FLT x141 = 2 * x101 * x92;
	const GEN_FLT x142 =
		-x104 * (x102 * x122 + x119 * x141 + x128 + x133 * x140 +
				 x139 * (x133 * x135 -
						 x137 * (x119 * x86 + x119 * x96 + x122 * x81 +
								 x81 * (x119 * x95 + x122 +
										x81 * (x119 * x94 + x121 + x81 * (-x119 * x136 + x119 * x93 + x120)))))) +
		x132;
	const GEN_FLT x143 = gibMag_0 * cos(gibPhase_0 + x88 - asin(x103));
	const GEN_FLT x144 = -2 * x50;
	const GEN_FLT x145 = 2 * x52;
	const GEN_FLT x146 = -x111 * (-x107 + x108) - x114 * (x144 - x145);
	const GEN_FLT x147 = -x109 * (2 * x10 + x110) + x146;
	const GEN_FLT x148 = x106 * (x11 * x80 + x118 * x147);
	const GEN_FLT x149 = x148 * x83;
	const GEN_FLT x150 = x148 * x84 + x81 * (-x148 * x82 + x149);
	const GEN_FLT x151 = x148 * x85 + x150 * x81;
	const GEN_FLT x152 = x11 * x73 + x127 * x146;
	const GEN_FLT x153 = x51 - x52;
	const GEN_FLT x154 = x129 * x62 + x131 * x153;
	const GEN_FLT x155 = -x125 * x152 + x154;
	const GEN_FLT x156 =
		-x104 * (x102 * x151 +
				 x139 * (x135 * x155 -
						 x137 * (x148 * x86 + x148 * x96 + x151 * x81 +
								 x81 * (x148 * x95 + x151 +
										x81 * (x148 * x94 + x150 + x81 * (-x136 * x148 + x148 * x93 + x149))))) +
				 x140 * x155 + x141 * x148 + x152) +
		x154;
	const GEN_FLT x157 = -x111 * (x112 + x113) - x114 * (-x110 - 2 * x64);
	const GEN_FLT x158 = -x109 * (x144 + x145) + x157;
	const GEN_FLT x159 = x106 * (x118 * x158 + x53 * x80);
	const GEN_FLT x160 = x159 * x83;
	const GEN_FLT x161 = x159 * x84 + x81 * (-x159 * x82 + x160);
	const GEN_FLT x162 = x159 * x85 + x161 * x81;
	const GEN_FLT x163 = x127 * x157 + x53 * x73;
	const GEN_FLT x164 = x6 - x64;
	const GEN_FLT x165 = x129 * x61 + x131 * x164;
	const GEN_FLT x166 = -x125 * x163 + x165;
	const GEN_FLT x167 =
		-x104 * (x102 * x162 +
				 x139 * (x135 * x166 -
						 x137 * (x159 * x86 + x159 * x96 + x162 * x81 +
								 x81 * (x159 * x95 + x162 +
										x81 * (x159 * x94 + x161 + x81 * (-x136 * x159 + x159 * x93 + x160))))) +
				 x140 * x166 + x141 * x159 + x163) +
		x165;
	const GEN_FLT x168 = x22 * x29;
	const GEN_FLT x169 = -x168;
	const GEN_FLT x170 = 2 / ((x15 * x15));
	const GEN_FLT x171 = obj_qi * x170;
	const GEN_FLT x172 = pow(x15, -3.0 / 2.0);
	const GEN_FLT x173 = obj_qi * x172;
	const GEN_FLT x174 = ((x20) ? (-obj_qk * x173) : (0));
	const GEN_FLT x175 = x174 * x22;
	const GEN_FLT x176 = x17 * x29;
	const GEN_FLT x177 = x176 * x25;
	const GEN_FLT x178 = ((x20) ? (-obj_qj * x173) : (0));
	const GEN_FLT x179 = ((x20) ? (-x12 * x172 + x23) : (0));
	const GEN_FLT x180 = x29 * x30;
	const GEN_FLT x181 = x178 * x31 + x179 * x34 + x180 * x47;
	const GEN_FLT x182 = x179 * x22;
	const GEN_FLT x183 = x176 * x30;
	const GEN_FLT x184 = x174 * x34;
	const GEN_FLT x185 = x18 * x25;
	const GEN_FLT x186 = x178 * x185;
	const GEN_FLT x187 = x26 * x28;
	const GEN_FLT x188 = x184 + x186 + x187 * x29;
	const GEN_FLT x189 = sensor_x * (x175 + x177 + x181) +
						 sensor_y * (x168 * x21 + x169 + x18 * ((x20) ? (-x13 * x171) : (0))) +
						 sensor_z * (-x182 - x183 + x188);
	const GEN_FLT x190 = x11 * x189;
	const GEN_FLT x191 = x178 * x22;
	const GEN_FLT x192 = -x191;
	const GEN_FLT x193 = x176 * x28;
	const GEN_FLT x194 = x174 * x31 + x179 * x185 + x180 * x26;
	const GEN_FLT x195 = sensor_x * (x192 - x193 + x194) + sensor_y * (x182 + x183 + x188) +
						 sensor_z * (x168 * x54 + x169 + x18 * ((x20) ? (-x14 * x171) : (0)));
	const GEN_FLT x196 = x195 * x53;
	const GEN_FLT x197 = 2 * x19;
	const GEN_FLT x198 = -x175;
	const GEN_FLT x199 =
		sensor_x * (x168 * x46 + x169 + x18 * ((x20) ? (obj_qi * x197 - x170 * obj_qi * (obj_qi * obj_qi)) : (0))) +
		sensor_y * (-x177 + x181 + x198) + sensor_z * (x191 + x193 + x194);
	const GEN_FLT x200 = x199 * x45;
	const GEN_FLT x201 = x190 + x196 + x200;
	const GEN_FLT x202 = x199 * x58;
	const GEN_FLT x203 = x195 * x61;
	const GEN_FLT x204 = x189 * x62;
	const GEN_FLT x205 = x164 * x195;
	const GEN_FLT x206 = x153 * x189;
	const GEN_FLT x207 = x130 * x199;
	const GEN_FLT x208 = -x111 * (2 * x202 + 2 * x203 + 2 * x204) - x114 * (2 * x205 + 2 * x206 + 2 * x207);
	const GEN_FLT x209 = -x109 * (2 * x190 + 2 * x196 + 2 * x200) + x208;
	const GEN_FLT x210 = x106 * (x118 * x209 + x201 * x80);
	const GEN_FLT x211 = x210 * x83;
	const GEN_FLT x212 = x210 * x84 + x81 * (-x210 * x82 + x211);
	const GEN_FLT x213 = x210 * x85 + x212 * x81;
	const GEN_FLT x214 = x127 * x208 + x201 * x73;
	const GEN_FLT x215 = x129 * (x202 + x203 + x204) + x131 * (x205 + x206 + x207);
	const GEN_FLT x216 = -x125 * x214 + x215;
	const GEN_FLT x217 =
		-x104 * (x102 * x213 +
				 x139 * (x135 * x216 -
						 x137 * (x210 * x86 + x210 * x96 + x213 * x81 +
								 x81 * (x210 * x95 + x213 +
										x81 * (x210 * x94 + x212 + x81 * (-x136 * x210 + x210 * x93 + x211))))) +
				 x140 * x216 + x141 * x210 + x214) +
		x215;
	const GEN_FLT x218 = x22 * x27;
	const GEN_FLT x219 = -x218;
	const GEN_FLT x220 = ((x20) ? (-obj_qj * obj_qk * x172) : (0));
	const GEN_FLT x221 = x22 * x220;
	const GEN_FLT x222 = x17 * x27;
	const GEN_FLT x223 = x222 * x25;
	const GEN_FLT x224 = ((x20) ? (-x13 * x172 + x23) : (0));
	const GEN_FLT x225 = x27 * x30;
	const GEN_FLT x226 = x178 * x34 + x224 * x31 + x225 * x47;
	const GEN_FLT x227 = x222 * x30;
	const GEN_FLT x228 = x185 * x224 + x187 * x27 + x220 * x34;
	const GEN_FLT x229 =
		sensor_x * (x221 + x223 + x226) +
		sensor_y * (x18 * ((x20) ? (obj_qj * x197 - x170 * obj_qj * (obj_qj * obj_qj)) : (0)) + x21 * x218 + x219) +
		sensor_z * (x192 - x227 + x228);
	const GEN_FLT x230 = x11 * x229;
	const GEN_FLT x231 = obj_qj * x170;
	const GEN_FLT x232 = x22 * x224;
	const GEN_FLT x233 = x222 * x28;
	const GEN_FLT x234 = x220 * x31;
	const GEN_FLT x235 = x186 + x225 * x26 + x234;
	const GEN_FLT x236 = -x221;
	const GEN_FLT x237 = sensor_x * (x18 * ((x20) ? (-x12 * x231) : (0)) + x218 * x46 + x219) +
						 sensor_y * (-x223 + x226 + x236) + sensor_z * (x232 + x233 + x235);
	const GEN_FLT x238 = x237 * x45;
	const GEN_FLT x239 = sensor_x * (-x232 - x233 + x235) + sensor_y * (x191 + x227 + x228) +
						 sensor_z * (x18 * ((x20) ? (-x14 * x231) : (0)) + x218 * x54 + x219);
	const GEN_FLT x240 = x239 * x53;
	const GEN_FLT x241 = x230 + x238 + x240;
	const GEN_FLT x242 = x237 * x58;
	const GEN_FLT x243 = x239 * x61;
	const GEN_FLT x244 = x229 * x62;
	const GEN_FLT x245 = x164 * x239;
	const GEN_FLT x246 = x130 * x237;
	const GEN_FLT x247 = x153 * x229;
	const GEN_FLT x248 = -x111 * (2 * x242 + 2 * x243 + 2 * x244) - x114 * (2 * x245 + 2 * x246 + 2 * x247);
	const GEN_FLT x249 = -x109 * (2 * x230 + 2 * x238 + 2 * x240) + x248;
	const GEN_FLT x250 = x106 * (x118 * x249 + x241 * x80);
	const GEN_FLT x251 = x250 * x83;
	const GEN_FLT x252 = x250 * x84 + x81 * (-x250 * x82 + x251);
	const GEN_FLT x253 = x250 * x85 + x252 * x81;
	const GEN_FLT x254 = x127 * x248 + x241 * x73;
	const GEN_FLT x255 = x129 * (x242 + x243 + x244) + x131 * (x245 + x246 + x247);
	const GEN_FLT x256 = -x125 * x254 + x255;
	const GEN_FLT x257 =
		-x104 * (x102 * x253 +
				 x139 * (x135 * x256 -
						 x137 * (x250 * x86 + x250 * x96 + x253 * x81 +
								 x81 * (x250 * x95 + x253 +
										x81 * (x250 * x94 + x252 + x81 * (-x136 * x250 + x250 * x93 + x251))))) +
				 x140 * x256 + x141 * x250 + x254) +
		x255;
	const GEN_FLT x258 = x22 * x24;
	const GEN_FLT x259 = -x258;
	const GEN_FLT x260 = obj_qk * x170;
	const GEN_FLT x261 = ((x20) ? (-x14 * x172 + x23) : (0));
	const GEN_FLT x262 = x22 * x261;
	const GEN_FLT x263 = x17 * x24;
	const GEN_FLT x264 = x25 * x263;
	const GEN_FLT x265 = x24 * x30;
	const GEN_FLT x266 = x184 + x234 + x265 * x47;
	const GEN_FLT x267 = x263 * x30;
	const GEN_FLT x268 = x185 * x220 + x187 * x24 + x261 * x34;
	const GEN_FLT x269 = sensor_x * (x262 + x264 + x266) +
						 sensor_y * (x18 * ((x20) ? (-x13 * x260) : (0)) + x21 * x258 + x259) +
						 sensor_z * (x198 - x267 + x268);
	const GEN_FLT x270 = x11 * x269;
	const GEN_FLT x271 = x263 * x28;
	const GEN_FLT x272 = x174 * x185 + x26 * x265 + x261 * x31;
	const GEN_FLT x273 = sensor_x * (x18 * ((x20) ? (-x12 * x260) : (0)) + x258 * x46 + x259) +
						 sensor_y * (-x262 - x264 + x266) + sensor_z * (x221 + x271 + x272);
	const GEN_FLT x274 = x273 * x45;
	const GEN_FLT x275 =
		sensor_x * (x236 - x271 + x272) + sensor_y * (x175 + x267 + x268) +
		sensor_z * (x18 * ((x20) ? (obj_qk * x197 - x170 * obj_qk * (obj_qk * obj_qk)) : (0)) + x258 * x54 + x259);
	const GEN_FLT x276 = x275 * x53;
	const GEN_FLT x277 = x270 + x274 + x276;
	const GEN_FLT x278 = x273 * x58;
	const GEN_FLT x279 = x269 * x62;
	const GEN_FLT x280 = x275 * x61;
	const GEN_FLT x281 = x164 * x275;
	const GEN_FLT x282 = x130 * x273;
	const GEN_FLT x283 = x153 * x269;
	const GEN_FLT x284 = -x111 * (2 * x278 + 2 * x279 + 2 * x280) - x114 * (2 * x281 + 2 * x282 + 2 * x283);
	const GEN_FLT x285 = -x109 * (2 * x270 + 2 * x274 + 2 * x276) + x284;
	const GEN_FLT x286 = x106 * (x118 * x285 + x277 * x80);
	const GEN_FLT x287 = x286 * x83;
	const GEN_FLT x288 = x286 * x84 + x81 * (-x286 * x82 + x287);
	const GEN_FLT x289 = x286 * x85 + x288 * x81;
	const GEN_FLT x290 = x127 * x284 + x277 * x73;
	const GEN_FLT x291 = x129 * (x278 + x279 + x280) + x131 * (x281 + x282 + x283);
	const GEN_FLT x292 = -x125 * x290 + x291;
	const GEN_FLT x293 =
		-x104 * (x102 * x289 +
				 x139 * (x135 * x292 -
						 x137 * (x286 * x86 + x286 * x96 + x289 * x81 +
								 x81 * (x286 * x95 + x289 +
										x81 * (x286 * x94 + x288 + x81 * (-x136 * x286 + x286 * x93 + x287))))) +
				 x140 * x292 + x141 * x286 + x290) +
		x291;
	const GEN_FLT x294 = tilt_1 - 0.52359877559829882;
	const GEN_FLT x295 = tan(x294);
	const GEN_FLT x296 = x295 * x70;
	const GEN_FLT x297 = x296 * x56;
	const GEN_FLT x298 = cos(x294);
	const GEN_FLT x299 = 1.0 / x298;
	const GEN_FLT x300 = x299 * x77;
	const GEN_FLT x301 = asin(x300 * x56);
	const GEN_FLT x302 = 8.0108022e-6 * x301;
	const GEN_FLT x303 = -x302 - 8.0108022e-6;
	const GEN_FLT x304 = x301 * x303 + 0.0028679863;
	const GEN_FLT x305 = x301 * x304 + 5.3685255000000001e-6;
	const GEN_FLT x306 = x301 * x305 + 0.0076069798000000001;
	const GEN_FLT x307 = x301 * x301;
	const GEN_FLT x308 = ogeePhase_1 + x88 - asin(x297);
	const GEN_FLT x309 = ogeeMag_1 * sin(x308);
	const GEN_FLT x310 = curve_1 + x309;
	const GEN_FLT x311 = x301 * x306;
	const GEN_FLT x312 = -1.60216044e-5 * x301 - 8.0108022e-6;
	const GEN_FLT x313 = x301 * x312 + x304;
	const GEN_FLT x314 = x301 * x313 + x305;
	const GEN_FLT x315 = x301 * x314 + x306;
	const GEN_FLT x316 = sin(x294);
	const GEN_FLT x317 = x316 * (x301 * x315 + x311);
	const GEN_FLT x318 = x298 - x310 * x317;
	const GEN_FLT x319 = 1.0 / x318;
	const GEN_FLT x320 = x310 * x319;
	const GEN_FLT x321 = x307 * x320;
	const GEN_FLT x322 = x297 + x306 * x321;
	const GEN_FLT x323 = pow(1 - x322 * x322, -1.0 / 2.0);
	const GEN_FLT x324 = pow(-x105 / (x298 * x298) + 1, -1.0 / 2.0);
	const GEN_FLT x325 = x117 * x299;
	const GEN_FLT x326 = x324 * (x116 * x325 + x300 * x45);
	const GEN_FLT x327 = x303 * x326;
	const GEN_FLT x328 = x301 * (-x302 * x326 + x327) + x304 * x326;
	const GEN_FLT x329 = x301 * x328 + x305 * x326;
	const GEN_FLT x330 = pow(-x124 * x295 * x295 + 1, -1.0 / 2.0);
	const GEN_FLT x331 = x126 * x295;
	const GEN_FLT x332 = x115 * x331 + x296 * x45;
	const GEN_FLT x333 = x132 - x330 * x332;
	const GEN_FLT x334 = ogeeMag_1 * cos(x308);
	const GEN_FLT x335 = x317 * x334;
	const GEN_FLT x336 = 2.40324066e-5 * x301;
	const GEN_FLT x337 = x316 * (-curve_1 - x309);
	const GEN_FLT x338 = x306 * x307;
	const GEN_FLT x339 = x310 * x338 / ((x318 * x318));
	const GEN_FLT x340 = x319 * x334 * x338;
	const GEN_FLT x341 = 2 * x311 * x320;
	const GEN_FLT x342 =
		x132 - x323 * (x321 * x329 + x326 * x341 + x332 + x333 * x340 +
					   x339 * (x333 * x335 -
							   x337 * (x301 * x329 +
									   x301 * (x301 * (x301 * (x312 * x326 - x326 * x336 + x327) + x313 * x326 + x328) +
											   x314 * x326 + x329) +
									   x306 * x326 + x315 * x326)));
	const GEN_FLT x343 = gibMag_1 * cos(gibPhase_1 + x88 - asin(x322));
	const GEN_FLT x344 = x324 * (x11 * x300 + x147 * x325);
	const GEN_FLT x345 = x303 * x344;
	const GEN_FLT x346 = x301 * (-x302 * x344 + x345) + x304 * x344;
	const GEN_FLT x347 = x301 * x346 + x305 * x344;
	const GEN_FLT x348 = x11 * x296 + x146 * x331;
	const GEN_FLT x349 = x154 - x330 * x348;
	const GEN_FLT x350 =
		x154 - x323 * (x321 * x347 +
					   x339 * (x335 * x349 -
							   x337 * (x301 * x347 +
									   x301 * (x301 * (x301 * (x312 * x344 - x336 * x344 + x345) + x313 * x344 + x346) +
											   x314 * x344 + x347) +
									   x306 * x344 + x315 * x344)) +
					   x340 * x349 + x341 * x344 + x348);
	const GEN_FLT x351 = x324 * (x158 * x325 + x300 * x53);
	const GEN_FLT x352 = x303 * x351;
	const GEN_FLT x353 = x301 * (-x302 * x351 + x352) + x304 * x351;
	const GEN_FLT x354 = x301 * x353 + x305 * x351;
	const GEN_FLT x355 = x157 * x331 + x296 * x53;
	const GEN_FLT x356 = x165 - x330 * x355;
	const GEN_FLT x357 =
		x165 - x323 * (x321 * x354 +
					   x339 * (x335 * x356 -
							   x337 * (x301 * x354 +
									   x301 * (x301 * (x301 * (x312 * x351 - x336 * x351 + x352) + x313 * x351 + x353) +
											   x314 * x351 + x354) +
									   x306 * x351 + x315 * x351)) +
					   x340 * x356 + x341 * x351 + x355);
	const GEN_FLT x358 = x324 * (x201 * x300 + x209 * x325);
	const GEN_FLT x359 = x303 * x358;
	const GEN_FLT x360 = x301 * (-x302 * x358 + x359) + x304 * x358;
	const GEN_FLT x361 = x301 * x360 + x305 * x358;
	const GEN_FLT x362 = x201 * x296 + x208 * x331;
	const GEN_FLT x363 = x215 - x330 * x362;
	const GEN_FLT x364 =
		x215 - x323 * (x321 * x361 +
					   x339 * (x335 * x363 -
							   x337 * (x301 * x361 +
									   x301 * (x301 * (x301 * (x312 * x358 - x336 * x358 + x359) + x313 * x358 + x360) +
											   x314 * x358 + x361) +
									   x306 * x358 + x315 * x358)) +
					   x340 * x363 + x341 * x358 + x362);
	const GEN_FLT x365 = x324 * (x241 * x300 + x249 * x325);
	const GEN_FLT x366 = x303 * x365;
	const GEN_FLT x367 = x301 * (-x302 * x365 + x366) + x304 * x365;
	const GEN_FLT x368 = x301 * x367 + x305 * x365;
	const GEN_FLT x369 = x241 * x296 + x248 * x331;
	const GEN_FLT x370 = x255 - x330 * x369;
	const GEN_FLT x371 =
		x255 - x323 * (x321 * x368 +
					   x339 * (x335 * x370 -
							   x337 * (x301 * x368 +
									   x301 * (x301 * (x301 * (x312 * x365 - x336 * x365 + x366) + x313 * x365 + x367) +
											   x314 * x365 + x368) +
									   x306 * x365 + x315 * x365)) +
					   x340 * x370 + x341 * x365 + x369);
	const GEN_FLT x372 = x324 * (x277 * x300 + x285 * x325);
	const GEN_FLT x373 = x303 * x372;
	const GEN_FLT x374 = x301 * (-x302 * x372 + x373) + x304 * x372;
	const GEN_FLT x375 = x301 * x374 + x305 * x372;
	const GEN_FLT x376 = x277 * x296 + x284 * x331;
	const GEN_FLT x377 = x291 - x330 * x376;
	const GEN_FLT x378 =
		x291 - x323 * (x321 * x375 +
					   x339 * (x335 * x377 -
							   x337 * (x301 * x375 +
									   x301 * (x301 * (x301 * (x312 * x372 - x336 * x372 + x373) + x313 * x372 + x374) +
											   x314 * x372 + x375) +
									   x306 * x372 + x315 * x372)) +
					   x340 * x377 + x341 * x372 + x376);
	*(out++) = x142 * x143 + x142;
	*(out++) = x143 * x156 + x156;
	*(out++) = x143 * x167 + x167;
	*(out++) = x143 * x217 + x217;
	*(out++) = x143 * x257 + x257;
	*(out++) = x143 * x293 + x293;
	*(out++) = x342 * x343 + x342;
	*(out++) = x343 * x350 + x350;
	*(out++) = x343 * x357 + x357;
	*(out++) = x343 * x364 + x364;
	*(out++) = x343 * x371 + x371;
	*(out++) = x343 * x378 + x378;
}

static inline void gen_reproject_axisangle_axis_x_jac_obj_p_gen2(FLT *out, const FLT *obj, const FLT *sensor,
																 const FLT *lh, const FLT phase_0, const FLT tilt_0,
																 const FLT curve_0, const FLT gibPhase_0,
																 const FLT gibMag_0, const FLT ogeePhase_0,
																 const FLT ogeeMag_0) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = -x5;
	const GEN_FLT x7 = x6 + 1;
	const GEN_FLT x8 = 1.0 / x3;
	const GEN_FLT x9 = x4 > 0;
	const GEN_FLT x10 = x7 * ((x9) ? (x1 * x8) : (0));
	const GEN_FLT x11 = x10 + x5;
	const GEN_FLT x12 = obj_qi * obj_qi;
	const GEN_FLT x13 = obj_qj * obj_qj;
	const GEN_FLT x14 = obj_qk * obj_qk;
	const GEN_FLT x15 = x12 + x13 + x14;
	const GEN_FLT x16 = sqrt(x15);
	const GEN_FLT x17 = cos(x16);
	const GEN_FLT x18 = 1 - x17;
	const GEN_FLT x19 = 1.0 / x15;
	const GEN_FLT x20 = x16 > 0;
	const GEN_FLT x21 = ((x20) ? (x13 * x19) : (0));
	const GEN_FLT x22 = sin(x16);
	const GEN_FLT x23 = 1.0 / x16;
	const GEN_FLT x24 = obj_qk * x23;
	const GEN_FLT x25 = ((x20) ? (x24) : (0));
	const GEN_FLT x26 = x22 * x25;
	const GEN_FLT x27 = obj_qj * x23;
	const GEN_FLT x28 = ((x20) ? (x27) : (0));
	const GEN_FLT x29 = obj_qi * x23;
	const GEN_FLT x30 = ((x20) ? (x29) : (1));
	const GEN_FLT x31 = x18 * x30;
	const GEN_FLT x32 = x28 * x31;
	const GEN_FLT x33 = x22 * x30;
	const GEN_FLT x34 = x18 * x28;
	const GEN_FLT x35 = x25 * x34;
	const GEN_FLT x36 = obj_py + sensor_x * (x26 + x32) + sensor_y * (x17 + x18 * x21) + sensor_z * (-x33 + x35);
	const GEN_FLT x37 = sin(x4);
	const GEN_FLT x38 = 1.0 / x4;
	const GEN_FLT x39 = ((x9) ? (lh_qk * x38) : (0));
	const GEN_FLT x40 = x37 * x39;
	const GEN_FLT x41 = ((x9) ? (lh_qj * x38) : (0));
	const GEN_FLT x42 = ((x9) ? (lh_qi * x38) : (1));
	const GEN_FLT x43 = x42 * x7;
	const GEN_FLT x44 = x41 * x43;
	const GEN_FLT x45 = x40 + x44;
	const GEN_FLT x46 = ((x20) ? (x12 * x19) : (1));
	const GEN_FLT x47 = x22 * x28;
	const GEN_FLT x48 = x25 * x31;
	const GEN_FLT x49 = obj_px + sensor_x * (x17 + x18 * x46) + sensor_y * (-x26 + x32) + sensor_z * (x47 + x48);
	const GEN_FLT x50 = x37 * x42;
	const GEN_FLT x51 = -x50;
	const GEN_FLT x52 = x39 * x41 * x7;
	const GEN_FLT x53 = x51 + x52;
	const GEN_FLT x54 = ((x20) ? (x14 * x19) : (0));
	const GEN_FLT x55 = obj_pz + sensor_x * (-x47 + x48) + sensor_y * (x33 + x35) + sensor_z * (x17 + x18 * x54);
	const GEN_FLT x56 = lh_py + x11 * x36 + x45 * x49 + x53 * x55;
	const GEN_FLT x57 = x7 * ((x9) ? (x0 * x8) : (1));
	const GEN_FLT x58 = x5 + x57;
	const GEN_FLT x59 = x37 * x41;
	const GEN_FLT x60 = x39 * x43;
	const GEN_FLT x61 = x59 + x60;
	const GEN_FLT x62 = -x40 + x44;
	const GEN_FLT x63 = lh_px + x36 * x62 + x49 * x58 + x55 * x61;
	const GEN_FLT x64 = x7 * ((x9) ? (x2 * x8) : (0));
	const GEN_FLT x65 = x55 * (x5 + x64);
	const GEN_FLT x66 = x36 * (x50 + x52);
	const GEN_FLT x67 = x49 * (-x59 + x60);
	const GEN_FLT x68 = -lh_pz - x65 - x66 - x67;
	const GEN_FLT x69 = x63 * x63 + x68 * x68;
	const GEN_FLT x70 = tilt_0 + 0.52359877559829882;
	const GEN_FLT x71 = tan(x70);
	const GEN_FLT x72 = x71 / sqrt(x69);
	const GEN_FLT x73 = x56 * x72;
	const GEN_FLT x74 = x56 * x56;
	const GEN_FLT x75 = x69 + x74;
	const GEN_FLT x76 = cos(x70);
	const GEN_FLT x77 = 1.0 / x76;
	const GEN_FLT x78 = x77 / sqrt(x75);
	const GEN_FLT x79 = asin(x56 * x78);
	const GEN_FLT x80 = 8.0108022e-6 * x79;
	const GEN_FLT x81 = -x80 - 8.0108022e-6;
	const GEN_FLT x82 = x79 * x81 + 0.0028679863;
	const GEN_FLT x83 = x79 * x82 + 5.3685255000000001e-6;
	const GEN_FLT x84 = x79 * x83 + 0.0076069798000000001;
	const GEN_FLT x85 = x79 * x79;
	const GEN_FLT x86 = atan2(x68, x63);
	const GEN_FLT x87 = ogeePhase_0 + x86 - asin(x73);
	const GEN_FLT x88 = ogeeMag_0 * sin(x87);
	const GEN_FLT x89 = curve_0 + x88;
	const GEN_FLT x90 = x79 * x84;
	const GEN_FLT x91 = -1.60216044e-5 * x79 - 8.0108022e-6;
	const GEN_FLT x92 = x79 * x91 + x82;
	const GEN_FLT x93 = x79 * x92 + x83;
	const GEN_FLT x94 = x79 * x93 + x84;
	const GEN_FLT x95 = sin(x70);
	const GEN_FLT x96 = x95 * (x79 * x94 + x90);
	const GEN_FLT x97 = x76 - x89 * x96;
	const GEN_FLT x98 = 1.0 / x97;
	const GEN_FLT x99 = x89 * x98;
	const GEN_FLT x100 = x85 * x99;
	const GEN_FLT x101 = x100 * x84 + x73;
	const GEN_FLT x102 = pow(1 - x101 * x101, -1.0 / 2.0);
	const GEN_FLT x103 = pow(-x74 / (x75 * (x76 * x76)) + 1, -1.0 / 2.0);
	const GEN_FLT x104 = 2 * x40;
	const GEN_FLT x105 = 2 * x44;
	const GEN_FLT x106 = (1.0 / 2.0) * x56;
	const GEN_FLT x107 = 2 * x5;
	const GEN_FLT x108 = (1.0 / 2.0) * x63;
	const GEN_FLT x109 = 2 * x59;
	const GEN_FLT x110 = 2 * x60;
	const GEN_FLT x111 = (1.0 / 2.0) * x68;
	const GEN_FLT x112 = -x108 * (x107 + 2 * x57) - x111 * (x109 - x110);
	const GEN_FLT x113 = x56 * x77 / pow(x75, 3.0 / 2.0);
	const GEN_FLT x114 = x103 * (x113 * (-x106 * (x104 + x105) + x112) + x45 * x78);
	const GEN_FLT x115 = x114 * x81;
	const GEN_FLT x116 = x114 * x82 + x79 * (-x114 * x80 + x115);
	const GEN_FLT x117 = x114 * x83 + x116 * x79;
	const GEN_FLT x118 = 1.0 / x69;
	const GEN_FLT x119 = pow(-x118 * x74 * x71 * x71 + 1, -1.0 / 2.0);
	const GEN_FLT x120 = x56 * x71 / pow(x69, 3.0 / 2.0);
	const GEN_FLT x121 = x112 * x120 + x45 * x72;
	const GEN_FLT x122 = x118 * (lh_pz + x65 + x66 + x67);
	const GEN_FLT x123 = x59 - x60;
	const GEN_FLT x124 = x118 * x63;
	const GEN_FLT x125 = x122 * x58 + x123 * x124;
	const GEN_FLT x126 = -x119 * x121 + x125;
	const GEN_FLT x127 = ogeeMag_0 * cos(x87);
	const GEN_FLT x128 = x127 * x96;
	const GEN_FLT x129 = 2.40324066e-5 * x79;
	const GEN_FLT x130 = x95 * (-curve_0 - x88);
	const GEN_FLT x131 = x84 * x85;
	const GEN_FLT x132 = x131 * x89 / ((x97 * x97));
	const GEN_FLT x133 = x127 * x131 * x98;
	const GEN_FLT x134 = 2 * x90 * x99;
	const GEN_FLT x135 =
		-x102 * (x100 * x117 + x114 * x134 + x121 + x126 * x133 +
				 x132 * (x126 * x128 -
						 x130 * (x114 * x84 + x114 * x94 + x117 * x79 +
								 x79 * (x114 * x93 + x117 +
										x79 * (x114 * x92 + x116 + x79 * (-x114 * x129 + x114 * x91 + x115)))))) +
		x125;
	const GEN_FLT x136 = gibMag_0 * cos(gibPhase_0 + x86 - asin(x101));
	const GEN_FLT x137 = -2 * x50;
	const GEN_FLT x138 = 2 * x52;
	const GEN_FLT x139 = -x108 * (-x104 + x105) - x111 * (x137 - x138);
	const GEN_FLT x140 = x103 * (x11 * x78 + x113 * (-x106 * (2 * x10 + x107) + x139));
	const GEN_FLT x141 = x140 * x81;
	const GEN_FLT x142 = x140 * x82 + x79 * (-x140 * x80 + x141);
	const GEN_FLT x143 = x140 * x83 + x142 * x79;
	const GEN_FLT x144 = x11 * x72 + x120 * x139;
	const GEN_FLT x145 = x51 - x52;
	const GEN_FLT x146 = x122 * x62 + x124 * x145;
	const GEN_FLT x147 = -x119 * x144 + x146;
	const GEN_FLT x148 =
		-x102 * (x100 * x143 +
				 x132 * (x128 * x147 -
						 x130 * (x140 * x84 + x140 * x94 + x143 * x79 +
								 x79 * (x140 * x93 + x143 +
										x79 * (x140 * x92 + x142 + x79 * (-x129 * x140 + x140 * x91 + x141))))) +
				 x133 * x147 + x134 * x140 + x144) +
		x146;
	const GEN_FLT x149 = -x108 * (x109 + x110) - x111 * (-x107 - 2 * x64);
	const GEN_FLT x150 = x103 * (x113 * (-x106 * (x137 + x138) + x149) + x53 * x78);
	const GEN_FLT x151 = x150 * x81;
	const GEN_FLT x152 = x150 * x82 + x79 * (-x150 * x80 + x151);
	const GEN_FLT x153 = x150 * x83 + x152 * x79;
	const GEN_FLT x154 = x120 * x149 + x53 * x72;
	const GEN_FLT x155 = x6 - x64;
	const GEN_FLT x156 = x122 * x61 + x124 * x155;
	const GEN_FLT x157 = -x119 * x154 + x156;
	const GEN_FLT x158 =
		-x102 * (x100 * x153 +
				 x132 * (x128 * x157 -
						 x130 * (x150 * x84 + x150 * x94 + x153 * x79 +
								 x79 * (x150 * x93 + x153 +
										x79 * (x150 * x92 + x152 + x79 * (-x129 * x150 + x150 * x91 + x151))))) +
				 x133 * x157 + x134 * x150 + x154) +
		x156;
	const GEN_FLT x159 = x22 * x29;
	const GEN_FLT x160 = -x159;
	const GEN_FLT x161 = 2 / ((x15 * x15));
	const GEN_FLT x162 = obj_qi * x161;
	const GEN_FLT x163 = pow(x15, -3.0 / 2.0);
	const GEN_FLT x164 = obj_qi * x163;
	const GEN_FLT x165 = ((x20) ? (-obj_qk * x164) : (0));
	const GEN_FLT x166 = x165 * x22;
	const GEN_FLT x167 = x17 * x29;
	const GEN_FLT x168 = x167 * x25;
	const GEN_FLT x169 = ((x20) ? (-obj_qj * x164) : (0));
	const GEN_FLT x170 = ((x20) ? (-x12 * x163 + x23) : (0));
	const GEN_FLT x171 = x29 * x30;
	const GEN_FLT x172 = x169 * x31 + x170 * x34 + x171 * x47;
	const GEN_FLT x173 = x170 * x22;
	const GEN_FLT x174 = x167 * x30;
	const GEN_FLT x175 = x165 * x34;
	const GEN_FLT x176 = x18 * x25;
	const GEN_FLT x177 = x169 * x176;
	const GEN_FLT x178 = x26 * x28;
	const GEN_FLT x179 = x175 + x177 + x178 * x29;
	const GEN_FLT x180 = sensor_x * (x166 + x168 + x172) +
						 sensor_y * (x159 * x21 + x160 + x18 * ((x20) ? (-x13 * x162) : (0))) +
						 sensor_z * (-x173 - x174 + x179);
	const GEN_FLT x181 = x11 * x180;
	const GEN_FLT x182 = x169 * x22;
	const GEN_FLT x183 = -x182;
	const GEN_FLT x184 = x167 * x28;
	const GEN_FLT x185 = x165 * x31 + x170 * x176 + x171 * x26;
	const GEN_FLT x186 = sensor_x * (x183 - x184 + x185) + sensor_y * (x173 + x174 + x179) +
						 sensor_z * (x159 * x54 + x160 + x18 * ((x20) ? (-x14 * x162) : (0)));
	const GEN_FLT x187 = x186 * x53;
	const GEN_FLT x188 = 2 * x19;
	const GEN_FLT x189 = -x166;
	const GEN_FLT x190 =
		sensor_x * (x159 * x46 + x160 + x18 * ((x20) ? (obj_qi * x188 - x161 * obj_qi * (obj_qi * obj_qi)) : (0))) +
		sensor_y * (-x168 + x172 + x189) + sensor_z * (x182 + x184 + x185);
	const GEN_FLT x191 = x190 * x45;
	const GEN_FLT x192 = x181 + x187 + x191;
	const GEN_FLT x193 = x190 * x58;
	const GEN_FLT x194 = x186 * x61;
	const GEN_FLT x195 = x180 * x62;
	const GEN_FLT x196 = x155 * x186;
	const GEN_FLT x197 = x145 * x180;
	const GEN_FLT x198 = x123 * x190;
	const GEN_FLT x199 = -x108 * (2 * x193 + 2 * x194 + 2 * x195) - x111 * (2 * x196 + 2 * x197 + 2 * x198);
	const GEN_FLT x200 = x103 * (x113 * (-x106 * (2 * x181 + 2 * x187 + 2 * x191) + x199) + x192 * x78);
	const GEN_FLT x201 = x200 * x81;
	const GEN_FLT x202 = x200 * x82 + x79 * (-x200 * x80 + x201);
	const GEN_FLT x203 = x200 * x83 + x202 * x79;
	const GEN_FLT x204 = x120 * x199 + x192 * x72;
	const GEN_FLT x205 = x122 * (x193 + x194 + x195) + x124 * (x196 + x197 + x198);
	const GEN_FLT x206 = -x119 * x204 + x205;
	const GEN_FLT x207 =
		-x102 * (x100 * x203 +
				 x132 * (x128 * x206 -
						 x130 * (x200 * x84 + x200 * x94 + x203 * x79 +
								 x79 * (x200 * x93 + x203 +
										x79 * (x200 * x92 + x202 + x79 * (-x129 * x200 + x200 * x91 + x201))))) +
				 x133 * x206 + x134 * x200 + x204) +
		x205;
	const GEN_FLT x208 = x22 * x27;
	const GEN_FLT x209 = -x208;
	const GEN_FLT x210 = ((x20) ? (-obj_qj * obj_qk * x163) : (0));
	const GEN_FLT x211 = x210 * x22;
	const GEN_FLT x212 = x17 * x27;
	const GEN_FLT x213 = x212 * x25;
	const GEN_FLT x214 = ((x20) ? (-x13 * x163 + x23) : (0));
	const GEN_FLT x215 = x27 * x30;
	const GEN_FLT x216 = x169 * x34 + x214 * x31 + x215 * x47;
	const GEN_FLT x217 = x212 * x30;
	const GEN_FLT x218 = x176 * x214 + x178 * x27 + x210 * x34;
	const GEN_FLT x219 =
		sensor_x * (x211 + x213 + x216) +
		sensor_y * (x18 * ((x20) ? (obj_qj * x188 - x161 * obj_qj * (obj_qj * obj_qj)) : (0)) + x208 * x21 + x209) +
		sensor_z * (x183 - x217 + x218);
	const GEN_FLT x220 = x11 * x219;
	const GEN_FLT x221 = obj_qj * x161;
	const GEN_FLT x222 = x214 * x22;
	const GEN_FLT x223 = x212 * x28;
	const GEN_FLT x224 = x210 * x31;
	const GEN_FLT x225 = x177 + x215 * x26 + x224;
	const GEN_FLT x226 = -x211;
	const GEN_FLT x227 = sensor_x * (x18 * ((x20) ? (-x12 * x221) : (0)) + x208 * x46 + x209) +
						 sensor_y * (-x213 + x216 + x226) + sensor_z * (x222 + x223 + x225);
	const GEN_FLT x228 = x227 * x45;
	const GEN_FLT x229 = sensor_x * (-x222 - x223 + x225) + sensor_y * (x182 + x217 + x218) +
						 sensor_z * (x18 * ((x20) ? (-x14 * x221) : (0)) + x208 * x54 + x209);
	const GEN_FLT x230 = x229 * x53;
	const GEN_FLT x231 = x220 + x228 + x230;
	const GEN_FLT x232 = x227 * x58;
	const GEN_FLT x233 = x229 * x61;
	const GEN_FLT x234 = x219 * x62;
	const GEN_FLT x235 = x155 * x229;
	const GEN_FLT x236 = x123 * x227;
	const GEN_FLT x237 = x145 * x219;
	const GEN_FLT x238 = -x108 * (2 * x232 + 2 * x233 + 2 * x234) - x111 * (2 * x235 + 2 * x236 + 2 * x237);
	const GEN_FLT x239 = x103 * (x113 * (-x106 * (2 * x220 + 2 * x228 + 2 * x230) + x238) + x231 * x78);
	const GEN_FLT x240 = x239 * x81;
	const GEN_FLT x241 = x239 * x82 + x79 * (-x239 * x80 + x240);
	const GEN_FLT x242 = x239 * x83 + x241 * x79;
	const GEN_FLT x243 = x120 * x238 + x231 * x72;
	const GEN_FLT x244 = x122 * (x232 + x233 + x234) + x124 * (x235 + x236 + x237);
	const GEN_FLT x245 = -x119 * x243 + x244;
	const GEN_FLT x246 =
		-x102 * (x100 * x242 +
				 x132 * (x128 * x245 -
						 x130 * (x239 * x84 + x239 * x94 + x242 * x79 +
								 x79 * (x239 * x93 + x242 +
										x79 * (x239 * x92 + x241 + x79 * (-x129 * x239 + x239 * x91 + x240))))) +
				 x133 * x245 + x134 * x239 + x243) +
		x244;
	const GEN_FLT x247 = x22 * x24;
	const GEN_FLT x248 = -x247;
	const GEN_FLT x249 = obj_qk * x161;
	const GEN_FLT x250 = ((x20) ? (-x14 * x163 + x23) : (0));
	const GEN_FLT x251 = x22 * x250;
	const GEN_FLT x252 = x17 * x24;
	const GEN_FLT x253 = x25 * x252;
	const GEN_FLT x254 = x24 * x30;
	const GEN_FLT x255 = x175 + x224 + x254 * x47;
	const GEN_FLT x256 = x252 * x30;
	const GEN_FLT x257 = x176 * x210 + x178 * x24 + x250 * x34;
	const GEN_FLT x258 = sensor_x * (x251 + x253 + x255) +
						 sensor_y * (x18 * ((x20) ? (-x13 * x249) : (0)) + x21 * x247 + x248) +
						 sensor_z * (x189 - x256 + x257);
	const GEN_FLT x259 = x11 * x258;
	const GEN_FLT x260 = x252 * x28;
	const GEN_FLT x261 = x165 * x176 + x250 * x31 + x254 * x26;
	const GEN_FLT x262 = sensor_x * (x18 * ((x20) ? (-x12 * x249) : (0)) + x247 * x46 + x248) +
						 sensor_y * (-x251 - x253 + x255) + sensor_z * (x211 + x260 + x261);
	const GEN_FLT x263 = x262 * x45;
	const GEN_FLT x264 =
		sensor_x * (x226 - x260 + x261) + sensor_y * (x166 + x256 + x257) +
		sensor_z * (x18 * ((x20) ? (obj_qk * x188 - x161 * obj_qk * (obj_qk * obj_qk)) : (0)) + x247 * x54 + x248);
	const GEN_FLT x265 = x264 * x53;
	const GEN_FLT x266 = x259 + x263 + x265;
	const GEN_FLT x267 = x262 * x58;
	const GEN_FLT x268 = x258 * x62;
	const GEN_FLT x269 = x264 * x61;
	const GEN_FLT x270 = x155 * x264;
	const GEN_FLT x271 = x123 * x262;
	const GEN_FLT x272 = x145 * x258;
	const GEN_FLT x273 = -x108 * (2 * x267 + 2 * x268 + 2 * x269) - x111 * (2 * x270 + 2 * x271 + 2 * x272);
	const GEN_FLT x274 = x103 * (x113 * (-x106 * (2 * x259 + 2 * x263 + 2 * x265) + x273) + x266 * x78);
	const GEN_FLT x275 = x274 * x81;
	const GEN_FLT x276 = x274 * x82 + x79 * (-x274 * x80 + x275);
	const GEN_FLT x277 = x274 * x83 + x276 * x79;
	const GEN_FLT x278 = x120 * x273 + x266 * x72;
	const GEN_FLT x279 = x122 * (x267 + x268 + x269) + x124 * (x270 + x271 + x272);
	const GEN_FLT x280 = -x119 * x278 + x279;
	const GEN_FLT x281 =
		-x102 * (x100 * x277 +
				 x132 * (x128 * x280 -
						 x130 * (x274 * x84 + x274 * x94 + x277 * x79 +
								 x79 * (x274 * x93 + x277 +
										x79 * (x274 * x92 + x276 + x79 * (-x129 * x274 + x274 * x91 + x275))))) +
				 x133 * x280 + x134 * x274 + x278) +
		x279;
	*(out++) = x135 * x136 + x135;
	*(out++) = x136 * x148 + x148;
	*(out++) = x136 * x158 + x158;
	*(out++) = x136 * x207 + x207;
	*(out++) = x136 * x246 + x246;
	*(out++) = x136 * x281 + x281;
}

static inline void gen_reproject_axisangle_axis_y_jac_obj_p_gen2(FLT *out, const FLT *obj, const FLT *sensor,
																 const FLT *lh, const FLT phase_0, const FLT tilt_0,
																 const FLT curve_0, const FLT gibPhase_0,
																 const FLT gibMag_0, const FLT ogeePhase_0,
																 const FLT ogeeMag_0) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = -x5;
	const GEN_FLT x7 = x6 + 1;
	const GEN_FLT x8 = 1.0 / x3;
	const GEN_FLT x9 = x4 > 0;
	const GEN_FLT x10 = x7 * ((x9) ? (x1 * x8) : (0));
	const GEN_FLT x11 = x10 + x5;
	const GEN_FLT x12 = obj_qi * obj_qi;
	const GEN_FLT x13 = obj_qj * obj_qj;
	const GEN_FLT x14 = obj_qk * obj_qk;
	const GEN_FLT x15 = x12 + x13 + x14;
	const GEN_FLT x16 = sqrt(x15);
	const GEN_FLT x17 = cos(x16);
	const GEN_FLT x18 = 1 - x17;
	const GEN_FLT x19 = 1.0 / x15;
	const GEN_FLT x20 = x16 > 0;
	const GEN_FLT x21 = ((x20) ? (x13 * x19) : (0));
	const GEN_FLT x22 = sin(x16);
	const GEN_FLT x23 = 1.0 / x16;
	const GEN_FLT x24 = obj_qk * x23;
	const GEN_FLT x25 = ((x20) ? (x24) : (0));
	const GEN_FLT x26 = x22 * x25;
	const GEN_FLT x27 = obj_qj * x23;
	const GEN_FLT x28 = ((x20) ? (x27) : (0));
	const GEN_FLT x29 = obj_qi * x23;
	const GEN_FLT x30 = ((x20) ? (x29) : (1));
	const GEN_FLT x31 = x18 * x30;
	const GEN_FLT x32 = x28 * x31;
	const GEN_FLT x33 = x22 * x30;
	const GEN_FLT x34 = x18 * x28;
	const GEN_FLT x35 = x25 * x34;
	const GEN_FLT x36 = obj_py + sensor_x * (x26 + x32) + sensor_y * (x17 + x18 * x21) + sensor_z * (-x33 + x35);
	const GEN_FLT x37 = sin(x4);
	const GEN_FLT x38 = 1.0 / x4;
	const GEN_FLT x39 = ((x9) ? (lh_qk * x38) : (0));
	const GEN_FLT x40 = x37 * x39;
	const GEN_FLT x41 = ((x9) ? (lh_qj * x38) : (0));
	const GEN_FLT x42 = ((x9) ? (lh_qi * x38) : (1));
	const GEN_FLT x43 = x42 * x7;
	const GEN_FLT x44 = x41 * x43;
	const GEN_FLT x45 = x40 + x44;
	const GEN_FLT x46 = ((x20) ? (x12 * x19) : (1));
	const GEN_FLT x47 = x22 * x28;
	const GEN_FLT x48 = x25 * x31;
	const GEN_FLT x49 = obj_px + sensor_x * (x17 + x18 * x46) + sensor_y * (-x26 + x32) + sensor_z * (x47 + x48);
	const GEN_FLT x50 = x37 * x42;
	const GEN_FLT x51 = -x50;
	const GEN_FLT x52 = x39 * x41 * x7;
	const GEN_FLT x53 = x51 + x52;
	const GEN_FLT x54 = ((x20) ? (x14 * x19) : (0));
	const GEN_FLT x55 = obj_pz + sensor_x * (-x47 + x48) + sensor_y * (x33 + x35) + sensor_z * (x17 + x18 * x54);
	const GEN_FLT x56 = lh_py + x11 * x36 + x45 * x49 + x53 * x55;
	const GEN_FLT x57 = x7 * ((x9) ? (x0 * x8) : (1));
	const GEN_FLT x58 = x5 + x57;
	const GEN_FLT x59 = x37 * x41;
	const GEN_FLT x60 = x39 * x43;
	const GEN_FLT x61 = x59 + x60;
	const GEN_FLT x62 = -x40 + x44;
	const GEN_FLT x63 = lh_px + x36 * x62 + x49 * x58 + x55 * x61;
	const GEN_FLT x64 = x7 * ((x9) ? (x2 * x8) : (0));
	const GEN_FLT x65 = x55 * (x5 + x64);
	const GEN_FLT x66 = x36 * (x50 + x52);
	const GEN_FLT x67 = x49 * (-x59 + x60);
	const GEN_FLT x68 = -lh_pz - x65 - x66 - x67;
	const GEN_FLT x69 = x63 * x63 + x68 * x68;
	const GEN_FLT x70 = tilt_0 - 0.52359877559829882;
	const GEN_FLT x71 = tan(x70);
	const GEN_FLT x72 = x71 / sqrt(x69);
	const GEN_FLT x73 = x56 * x72;
	const GEN_FLT x74 = x56 * x56;
	const GEN_FLT x75 = x69 + x74;
	const GEN_FLT x76 = cos(x70);
	const GEN_FLT x77 = 1.0 / x76;
	const GEN_FLT x78 = x77 / sqrt(x75);
	const GEN_FLT x79 = asin(x56 * x78);
	const GEN_FLT x80 = 8.0108022e-6 * x79;
	const GEN_FLT x81 = -x80 - 8.0108022e-6;
	const GEN_FLT x82 = x79 * x81 + 0.0028679863;
	const GEN_FLT x83 = x79 * x82 + 5.3685255000000001e-6;
	const GEN_FLT x84 = x79 * x83 + 0.0076069798000000001;
	const GEN_FLT x85 = x79 * x79;
	const GEN_FLT x86 = atan2(x68, x63);
	const GEN_FLT x87 = ogeePhase_0 + x86 - asin(x73);
	const GEN_FLT x88 = ogeeMag_0 * sin(x87);
	const GEN_FLT x89 = curve_0 + x88;
	const GEN_FLT x90 = x79 * x84;
	const GEN_FLT x91 = -1.60216044e-5 * x79 - 8.0108022e-6;
	const GEN_FLT x92 = x79 * x91 + x82;
	const GEN_FLT x93 = x79 * x92 + x83;
	const GEN_FLT x94 = x79 * x93 + x84;
	const GEN_FLT x95 = sin(x70);
	const GEN_FLT x96 = x95 * (x79 * x94 + x90);
	const GEN_FLT x97 = x76 - x89 * x96;
	const GEN_FLT x98 = 1.0 / x97;
	const GEN_FLT x99 = x89 * x98;
	const GEN_FLT x100 = x85 * x99;
	const GEN_FLT x101 = x100 * x84 + x73;
	const GEN_FLT x102 = pow(1 - x101 * x101, -1.0 / 2.0);
	const GEN_FLT x103 = pow(-x74 / (x75 * (x76 * x76)) + 1, -1.0 / 2.0);
	const GEN_FLT x104 = 2 * x40;
	const GEN_FLT x105 = 2 * x44;
	const GEN_FLT x106 = (1.0 / 2.0) * x56;
	const GEN_FLT x107 = 2 * x5;
	const GEN_FLT x108 = (1.0 / 2.0) * x63;
	const GEN_FLT x109 = 2 * x59;
	const GEN_FLT x110 = 2 * x60;
	const GEN_FLT x111 = (1.0 / 2.0) * x68;
	const GEN_FLT x112 = -x108 * (x107 + 2 * x57) - x111 * (x109 - x110);
	const GEN_FLT x113 = x56 * x77 / pow(x75, 3.0 / 2.0);
	const GEN_FLT x114 = x103 * (x113 * (-x106 * (x104 + x105) + x112) + x45 * x78);
	const GEN_FLT x115 = x114 * x81;
	const GEN_FLT x116 = x114 * x82 + x79 * (-x114 * x80 + x115);
	const GEN_FLT x117 = x114 * x83 + x116 * x79;
	const GEN_FLT x118 = 1.0 / x69;
	const GEN_FLT x119 = pow(-x118 * x74 * x71 * x71 + 1, -1.0 / 2.0);
	const GEN_FLT x120 = x56 * x71 / pow(x69, 3.0 / 2.0);
	const GEN_FLT x121 = x112 * x120 + x45 * x72;
	const GEN_FLT x122 = x118 * (lh_pz + x65 + x66 + x67);
	const GEN_FLT x123 = x59 - x60;
	const GEN_FLT x124 = x118 * x63;
	const GEN_FLT x125 = x122 * x58 + x123 * x124;
	const GEN_FLT x126 = -x119 * x121 + x125;
	const GEN_FLT x127 = ogeeMag_0 * cos(x87);
	const GEN_FLT x128 = x127 * x96;
	const GEN_FLT x129 = 2.40324066e-5 * x79;
	const GEN_FLT x130 = x95 * (-curve_0 - x88);
	const GEN_FLT x131 = x84 * x85;
	const GEN_FLT x132 = x131 * x89 / ((x97 * x97));
	const GEN_FLT x133 = x127 * x131 * x98;
	const GEN_FLT x134 = 2 * x90 * x99;
	const GEN_FLT x135 =
		-x102 * (x100 * x117 + x114 * x134 + x121 + x126 * x133 +
				 x132 * (x126 * x128 -
						 x130 * (x114 * x84 + x114 * x94 + x117 * x79 +
								 x79 * (x114 * x93 + x117 +
										x79 * (x114 * x92 + x116 + x79 * (-x114 * x129 + x114 * x91 + x115)))))) +
		x125;
	const GEN_FLT x136 = gibMag_0 * cos(gibPhase_0 + x86 - asin(x101));
	const GEN_FLT x137 = -2 * x50;
	const GEN_FLT x138 = 2 * x52;
	const GEN_FLT x139 = -x108 * (-x104 + x105) - x111 * (x137 - x138);
	const GEN_FLT x140 = x103 * (x11 * x78 + x113 * (-x106 * (2 * x10 + x107) + x139));
	const GEN_FLT x141 = x140 * x81;
	const GEN_FLT x142 = x140 * x82 + x79 * (-x140 * x80 + x141);
	const GEN_FLT x143 = x140 * x83 + x142 * x79;
	const GEN_FLT x144 = x11 * x72 + x120 * x139;
	const GEN_FLT x145 = x51 - x52;
	const GEN_FLT x146 = x122 * x62 + x124 * x145;
	const GEN_FLT x147 = -x119 * x144 + x146;
	const GEN_FLT x148 =
		-x102 * (x100 * x143 +
				 x132 * (x128 * x147 -
						 x130 * (x140 * x84 + x140 * x94 + x143 * x79 +
								 x79 * (x140 * x93 + x143 +
										x79 * (x140 * x92 + x142 + x79 * (-x129 * x140 + x140 * x91 + x141))))) +
				 x133 * x147 + x134 * x140 + x144) +
		x146;
	const GEN_FLT x149 = -x108 * (x109 + x110) - x111 * (-x107 - 2 * x64);
	const GEN_FLT x150 = x103 * (x113 * (-x106 * (x137 + x138) + x149) + x53 * x78);
	const GEN_FLT x151 = x150 * x81;
	const GEN_FLT x152 = x150 * x82 + x79 * (-x150 * x80 + x151);
	const GEN_FLT x153 = x150 * x83 + x152 * x79;
	const GEN_FLT x154 = x120 * x149 + x53 * x72;
	const GEN_FLT x155 = x6 - x64;
	const GEN_FLT x156 = x122 * x61 + x124 * x155;
	const GEN_FLT x157 = -x119 * x154 + x156;
	const GEN_FLT x158 =
		-x102 * (x100 * x153 +
				 x132 * (x128 * x157 -
						 x130 * (x150 * x84 + x150 * x94 + x153 * x79 +
								 x79 * (x150 * x93 + x153 +
										x79 * (x150 * x92 + x152 + x79 * (-x129 * x150 + x150 * x91 + x151))))) +
				 x133 * x157 + x134 * x150 + x154) +
		x156;
	const GEN_FLT x159 = x22 * x29;
	const GEN_FLT x160 = -x159;
	const GEN_FLT x161 = 2 / ((x15 * x15));
	const GEN_FLT x162 = obj_qi * x161;
	const GEN_FLT x163 = pow(x15, -3.0 / 2.0);
	const GEN_FLT x164 = obj_qi * x163;
	const GEN_FLT x165 = ((x20) ? (-obj_qk * x164) : (0));
	const GEN_FLT x166 = x165 * x22;
	const GEN_FLT x167 = x17 * x29;
	const GEN_FLT x168 = x167 * x25;
	const GEN_FLT x169 = ((x20) ? (-obj_qj * x164) : (0));
	const GEN_FLT x170 = ((x20) ? (-x12 * x163 + x23) : (0));
	const GEN_FLT x171 = x29 * x30;
	const GEN_FLT x172 = x169 * x31 + x170 * x34 + x171 * x47;
	const GEN_FLT x173 = x170 * x22;
	const GEN_FLT x174 = x167 * x30;
	const GEN_FLT x175 = x165 * x34;
	const GEN_FLT x176 = x18 * x25;
	const GEN_FLT x177 = x169 * x176;
	const GEN_FLT x178 = x26 * x28;
	const GEN_FLT x179 = x175 + x177 + x178 * x29;
	const GEN_FLT x180 = sensor_x * (x166 + x168 + x172) +
						 sensor_y * (x159 * x21 + x160 + x18 * ((x20) ? (-x13 * x162) : (0))) +
						 sensor_z * (-x173 - x174 + x179);
	const GEN_FLT x181 = x11 * x180;
	const GEN_FLT x182 = x169 * x22;
	const GEN_FLT x183 = -x182;
	const GEN_FLT x184 = x167 * x28;
	const GEN_FLT x185 = x165 * x31 + x170 * x176 + x171 * x26;
	const GEN_FLT x186 = sensor_x * (x183 - x184 + x185) + sensor_y * (x173 + x174 + x179) +
						 sensor_z * (x159 * x54 + x160 + x18 * ((x20) ? (-x14 * x162) : (0)));
	const GEN_FLT x187 = x186 * x53;
	const GEN_FLT x188 = 2 * x19;
	const GEN_FLT x189 = -x166;
	const GEN_FLT x190 =
		sensor_x * (x159 * x46 + x160 + x18 * ((x20) ? (obj_qi * x188 - x161 * obj_qi * (obj_qi * obj_qi)) : (0))) +
		sensor_y * (-x168 + x172 + x189) + sensor_z * (x182 + x184 + x185);
	const GEN_FLT x191 = x190 * x45;
	const GEN_FLT x192 = x181 + x187 + x191;
	const GEN_FLT x193 = x190 * x58;
	const GEN_FLT x194 = x186 * x61;
	const GEN_FLT x195 = x180 * x62;
	const GEN_FLT x196 = x155 * x186;
	const GEN_FLT x197 = x145 * x180;
	const GEN_FLT x198 = x123 * x190;
	const GEN_FLT x199 = -x108 * (2 * x193 + 2 * x194 + 2 * x195) - x111 * (2 * x196 + 2 * x197 + 2 * x198);
	const GEN_FLT x200 = x103 * (x113 * (-x106 * (2 * x181 + 2 * x187 + 2 * x191) + x199) + x192 * x78);
	const GEN_FLT x201 = x200 * x81;
	const GEN_FLT x202 = x200 * x82 + x79 * (-x200 * x80 + x201);
	const GEN_FLT x203 = x200 * x83 + x202 * x79;
	const GEN_FLT x204 = x120 * x199 + x192 * x72;
	const GEN_FLT x205 = x122 * (x193 + x194 + x195) + x124 * (x196 + x197 + x198);
	const GEN_FLT x206 = -x119 * x204 + x205;
	const GEN_FLT x207 =
		-x102 * (x100 * x203 +
				 x132 * (x128 * x206 -
						 x130 * (x200 * x84 + x200 * x94 + x203 * x79 +
								 x79 * (x200 * x93 + x203 +
										x79 * (x200 * x92 + x202 + x79 * (-x129 * x200 + x200 * x91 + x201))))) +
				 x133 * x206 + x134 * x200 + x204) +
		x205;
	const GEN_FLT x208 = x22 * x27;
	const GEN_FLT x209 = -x208;
	const GEN_FLT x210 = ((x20) ? (-obj_qj * obj_qk * x163) : (0));
	const GEN_FLT x211 = x210 * x22;
	const GEN_FLT x212 = x17 * x27;
	const GEN_FLT x213 = x212 * x25;
	const GEN_FLT x214 = ((x20) ? (-x13 * x163 + x23) : (0));
	const GEN_FLT x215 = x27 * x30;
	const GEN_FLT x216 = x169 * x34 + x214 * x31 + x215 * x47;
	const GEN_FLT x217 = x212 * x30;
	const GEN_FLT x218 = x176 * x214 + x178 * x27 + x210 * x34;
	const GEN_FLT x219 =
		sensor_x * (x211 + x213 + x216) +
		sensor_y * (x18 * ((x20) ? (obj_qj * x188 - x161 * obj_qj * (obj_qj * obj_qj)) : (0)) + x208 * x21 + x209) +
		sensor_z * (x183 - x217 + x218);
	const GEN_FLT x220 = x11 * x219;
	const GEN_FLT x221 = obj_qj * x161;
	const GEN_FLT x222 = x214 * x22;
	const GEN_FLT x223 = x212 * x28;
	const GEN_FLT x224 = x210 * x31;
	const GEN_FLT x225 = x177 + x215 * x26 + x224;
	const GEN_FLT x226 = -x211;
	const GEN_FLT x227 = sensor_x * (x18 * ((x20) ? (-x12 * x221) : (0)) + x208 * x46 + x209) +
						 sensor_y * (-x213 + x216 + x226) + sensor_z * (x222 + x223 + x225);
	const GEN_FLT x228 = x227 * x45;
	const GEN_FLT x229 = sensor_x * (-x222 - x223 + x225) + sensor_y * (x182 + x217 + x218) +
						 sensor_z * (x18 * ((x20) ? (-x14 * x221) : (0)) + x208 * x54 + x209);
	const GEN_FLT x230 = x229 * x53;
	const GEN_FLT x231 = x220 + x228 + x230;
	const GEN_FLT x232 = x227 * x58;
	const GEN_FLT x233 = x229 * x61;
	const GEN_FLT x234 = x219 * x62;
	const GEN_FLT x235 = x155 * x229;
	const GEN_FLT x236 = x123 * x227;
	const GEN_FLT x237 = x145 * x219;
	const GEN_FLT x238 = -x108 * (2 * x232 + 2 * x233 + 2 * x234) - x111 * (2 * x235 + 2 * x236 + 2 * x237);
	const GEN_FLT x239 = x103 * (x113 * (-x106 * (2 * x220 + 2 * x228 + 2 * x230) + x238) + x231 * x78);
	const GEN_FLT x240 = x239 * x81;
	const GEN_FLT x241 = x239 * x82 + x79 * (-x239 * x80 + x240);
	const GEN_FLT x242 = x239 * x83 + x241 * x79;
	const GEN_FLT x243 = x120 * x238 + x231 * x72;
	const GEN_FLT x244 = x122 * (x232 + x233 + x234) + x124 * (x235 + x236 + x237);
	const GEN_FLT x245 = -x119 * x243 + x244;
	const GEN_FLT x246 =
		-x102 * (x100 * x242 +
				 x132 * (x128 * x245 -
						 x130 * (x239 * x84 + x239 * x94 + x242 * x79 +
								 x79 * (x239 * x93 + x242 +
										x79 * (x239 * x92 + x241 + x79 * (-x129 * x239 + x239 * x91 + x240))))) +
				 x133 * x245 + x134 * x239 + x243) +
		x244;
	const GEN_FLT x247 = x22 * x24;
	const GEN_FLT x248 = -x247;
	const GEN_FLT x249 = obj_qk * x161;
	const GEN_FLT x250 = ((x20) ? (-x14 * x163 + x23) : (0));
	const GEN_FLT x251 = x22 * x250;
	const GEN_FLT x252 = x17 * x24;
	const GEN_FLT x253 = x25 * x252;
	const GEN_FLT x254 = x24 * x30;
	const GEN_FLT x255 = x175 + x224 + x254 * x47;
	const GEN_FLT x256 = x252 * x30;
	const GEN_FLT x257 = x176 * x210 + x178 * x24 + x250 * x34;
	const GEN_FLT x258 = sensor_x * (x251 + x253 + x255) +
						 sensor_y * (x18 * ((x20) ? (-x13 * x249) : (0)) + x21 * x247 + x248) +
						 sensor_z * (x189 - x256 + x257);
	const GEN_FLT x259 = x11 * x258;
	const GEN_FLT x260 = x252 * x28;
	const GEN_FLT x261 = x165 * x176 + x250 * x31 + x254 * x26;
	const GEN_FLT x262 = sensor_x * (x18 * ((x20) ? (-x12 * x249) : (0)) + x247 * x46 + x248) +
						 sensor_y * (-x251 - x253 + x255) + sensor_z * (x211 + x260 + x261);
	const GEN_FLT x263 = x262 * x45;
	const GEN_FLT x264 =
		sensor_x * (x226 - x260 + x261) + sensor_y * (x166 + x256 + x257) +
		sensor_z * (x18 * ((x20) ? (obj_qk * x188 - x161 * obj_qk * (obj_qk * obj_qk)) : (0)) + x247 * x54 + x248);
	const GEN_FLT x265 = x264 * x53;
	const GEN_FLT x266 = x259 + x263 + x265;
	const GEN_FLT x267 = x262 * x58;
	const GEN_FLT x268 = x258 * x62;
	const GEN_FLT x269 = x264 * x61;
	const GEN_FLT x270 = x155 * x264;
	const GEN_FLT x271 = x123 * x262;
	const GEN_FLT x272 = x145 * x258;
	const GEN_FLT x273 = -x108 * (2 * x267 + 2 * x268 + 2 * x269) - x111 * (2 * x270 + 2 * x271 + 2 * x272);
	const GEN_FLT x274 = x103 * (x113 * (-x106 * (2 * x259 + 2 * x263 + 2 * x265) + x273) + x266 * x78);
	const GEN_FLT x275 = x274 * x81;
	const GEN_FLT x276 = x274 * x82 + x79 * (-x274 * x80 + x275);
	const GEN_FLT x277 = x274 * x83 + x276 * x79;
	const GEN_FLT x278 = x120 * x273 + x266 * x72;
	const GEN_FLT x279 = x122 * (x267 + x268 + x269) + x124 * (x270 + x271 + x272);
	const GEN_FLT x280 = -x119 * x278 + x279;
	const GEN_FLT x281 =
		-x102 * (x100 * x277 +
				 x132 * (x128 * x280 -
						 x130 * (x274 * x84 + x274 * x94 + x277 * x79 +
								 x79 * (x274 * x93 + x277 +
										x79 * (x274 * x92 + x276 + x79 * (-x129 * x274 + x274 * x91 + x275))))) +
				 x133 * x280 + x134 * x274 + x278) +
		x279;
	*(out++) = x135 * x136 + x135;
	*(out++) = x136 * x148 + x148;
	*(out++) = x136 * x158 + x158;
	*(out++) = x136 * x207 + x207;
	*(out++) = x136 * x246 + x246;
	*(out++) = x136 * x281 + x281;
}

static inline void gen_reproject_axisangle_jac_obj_p(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
													 const FLT phase_0, const FLT phase_1, const FLT tilt_0,
													 const FLT tilt_1, const FLT curve_0, const FLT curve_1,
													 const FLT gibPhase_0, const FLT gibPhase_1, const FLT gibMag_0,
													 const FLT gibMag_1) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = -x5;
	const GEN_FLT x7 = x6 + 1;
	const GEN_FLT x8 = 1.0 / x3;
	const GEN_FLT x9 = x4 > 0;
	const GEN_FLT x10 = x7 * ((x9) ? (x0 * x8) : (1));
	const GEN_FLT x11 = x10 + x5;
	const GEN_FLT x12 = obj_qi * obj_qi;
	const GEN_FLT x13 = obj_qj * obj_qj;
	const GEN_FLT x14 = obj_qk * obj_qk;
	const GEN_FLT x15 = x12 + x13 + x14;
	const GEN_FLT x16 = sqrt(x15);
	const GEN_FLT x17 = cos(x16);
	const GEN_FLT x18 = 1 - x17;
	const GEN_FLT x19 = 1.0 / x15;
	const GEN_FLT x20 = x16 > 0;
	const GEN_FLT x21 = ((x20) ? (x12 * x19) : (1));
	const GEN_FLT x22 = sin(x16);
	const GEN_FLT x23 = 1.0 / x16;
	const GEN_FLT x24 = obj_qj * x23;
	const GEN_FLT x25 = ((x20) ? (x24) : (0));
	const GEN_FLT x26 = x22 * x25;
	const GEN_FLT x27 = obj_qk * x23;
	const GEN_FLT x28 = ((x20) ? (x27) : (0));
	const GEN_FLT x29 = obj_qi * x23;
	const GEN_FLT x30 = ((x20) ? (x29) : (1));
	const GEN_FLT x31 = x18 * x30;
	const GEN_FLT x32 = x28 * x31;
	const GEN_FLT x33 = x22 * x28;
	const GEN_FLT x34 = x25 * x31;
	const GEN_FLT x35 = obj_px + sensor_x * (x17 + x18 * x21) + sensor_y * (-x33 + x34) + sensor_z * (x26 + x32);
	const GEN_FLT x36 = x11 * x35;
	const GEN_FLT x37 = sin(x4);
	const GEN_FLT x38 = 1.0 / x4;
	const GEN_FLT x39 = ((x9) ? (lh_qj * x38) : (0));
	const GEN_FLT x40 = x37 * x39;
	const GEN_FLT x41 = ((x9) ? (lh_qk * x38) : (0));
	const GEN_FLT x42 = ((x9) ? (lh_qi * x38) : (1));
	const GEN_FLT x43 = x42 * x7;
	const GEN_FLT x44 = x41 * x43;
	const GEN_FLT x45 = x40 + x44;
	const GEN_FLT x46 = ((x20) ? (x14 * x19) : (0));
	const GEN_FLT x47 = x22 * x30;
	const GEN_FLT x48 = x18 * x25;
	const GEN_FLT x49 = x28 * x48;
	const GEN_FLT x50 = obj_pz + sensor_x * (-x26 + x32) + sensor_y * (x47 + x49) + sensor_z * (x17 + x18 * x46);
	const GEN_FLT x51 = x45 * x50;
	const GEN_FLT x52 = x37 * x41;
	const GEN_FLT x53 = -x52;
	const GEN_FLT x54 = x39 * x43;
	const GEN_FLT x55 = x53 + x54;
	const GEN_FLT x56 = ((x20) ? (x13 * x19) : (0));
	const GEN_FLT x57 = obj_py + sensor_x * (x33 + x34) + sensor_y * (x17 + x18 * x56) + sensor_z * (-x47 + x49);
	const GEN_FLT x58 = x55 * x57;
	const GEN_FLT x59 = lh_px + x36 + x51 + x58;
	const GEN_FLT x60 = x59 * x59;
	const GEN_FLT x61 = x7 * ((x9) ? (x2 * x8) : (0));
	const GEN_FLT x62 = x37 * x42;
	const GEN_FLT x63 = x39 * x41 * x7;
	const GEN_FLT x64 = -lh_pz - x35 * (-x40 + x44) - x50 * (x5 + x61) - x57 * (x62 + x63);
	const GEN_FLT x65 = x64 * x64;
	const GEN_FLT x66 = x60 + x65;
	const GEN_FLT x67 = 1.0 / x66;
	const GEN_FLT x68 = x64 * x67;
	const GEN_FLT x69 = x11 * x68;
	const GEN_FLT x70 = x40 - x44;
	const GEN_FLT x71 = x67 * (-lh_px - x36 - x51 - x58);
	const GEN_FLT x72 = x70 * x71;
	const GEN_FLT x73 = x52 + x54;
	const GEN_FLT x74 = x7 * ((x9) ? (x1 * x8) : (0));
	const GEN_FLT x75 = x5 + x74;
	const GEN_FLT x76 = x57 * x75;
	const GEN_FLT x77 = x35 * x73;
	const GEN_FLT x78 = -x62;
	const GEN_FLT x79 = x63 + x78;
	const GEN_FLT x80 = x50 * x79;
	const GEN_FLT x81 = lh_py + x76 + x77 + x80;
	const GEN_FLT x82 = x81 * x81;
	const GEN_FLT x83 = 2 / (x65 + x82);
	const GEN_FLT x84 = x64 * x83;
	const GEN_FLT x85 = -lh_py - x76 - x77 - x80;
	const GEN_FLT x86 = x83 * x85;
	const GEN_FLT x87 = curve_0 * atan2(x81, x64);
	const GEN_FLT x88 = pow(-x67 * x82 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x89 = tilt_0 / sqrt(x66);
	const GEN_FLT x90 = 2 * x5;
	const GEN_FLT x91 = (1.0 / 2.0) * x59;
	const GEN_FLT x92 = 2 * x40;
	const GEN_FLT x93 = 2 * x44;
	const GEN_FLT x94 = (1.0 / 2.0) * x64;
	const GEN_FLT x95 = -x94 * (x92 - x93);
	const GEN_FLT x96 = tilt_0 * x81 / pow(x66, 3.0 / 2.0);
	const GEN_FLT x97 = x88 * (x73 * x89 + x96 * (-x91 * (2 * x10 + x90) + x95));
	const GEN_FLT x98 = atan2(x59, x64);
	const GEN_FLT x99 = gibMag_0 * sin(-gibPhase_0 + phase_0 + x98 + asin(x81 * x89) - 1.5707963267948966);
	const GEN_FLT x100 = x55 * x68;
	const GEN_FLT x101 = -x63;
	const GEN_FLT x102 = x101 + x78;
	const GEN_FLT x103 = x102 * x71;
	const GEN_FLT x104 = -2 * x52;
	const GEN_FLT x105 = 2 * x54;
	const GEN_FLT x106 = 2 * x62;
	const GEN_FLT x107 = -2 * x63;
	const GEN_FLT x108 = -x94 * (-x106 + x107);
	const GEN_FLT x109 = x88 * (x75 * x89 + x96 * (x108 - x91 * (x104 + x105)));
	const GEN_FLT x110 = x6 - x61;
	const GEN_FLT x111 = x110 * x71;
	const GEN_FLT x112 = x45 * x68;
	const GEN_FLT x113 = -x90;
	const GEN_FLT x114 = -x94 * (x113 - 2 * x61);
	const GEN_FLT x115 = x88 * (x79 * x89 + x96 * (x114 - x91 * (x92 + x93)));
	const GEN_FLT x116 = x22 * x29;
	const GEN_FLT x117 = -x116;
	const GEN_FLT x118 = 2 * x19;
	const GEN_FLT x119 = 2 / ((x15 * x15));
	const GEN_FLT x120 = pow(x15, -3.0 / 2.0);
	const GEN_FLT x121 = obj_qi * x120;
	const GEN_FLT x122 = ((x20) ? (-obj_qj * x121) : (0));
	const GEN_FLT x123 = x122 * x22;
	const GEN_FLT x124 = x17 * x29;
	const GEN_FLT x125 = x124 * x25;
	const GEN_FLT x126 = ((x20) ? (-obj_qk * x121) : (0));
	const GEN_FLT x127 = ((x20) ? (-x12 * x120 + x23) : (0));
	const GEN_FLT x128 = x18 * x28;
	const GEN_FLT x129 = x29 * x30;
	const GEN_FLT x130 = x126 * x31 + x127 * x128 + x129 * x33;
	const GEN_FLT x131 = x126 * x22;
	const GEN_FLT x132 = -x131;
	const GEN_FLT x133 = x124 * x28;
	const GEN_FLT x134 = x122 * x31 + x127 * x48 + x129 * x26;
	const GEN_FLT x135 =
		sensor_x * (x116 * x21 + x117 + x18 * ((x20) ? (obj_qi * x118 - x119 * obj_qi * (obj_qi * obj_qi)) : (0))) +
		sensor_y * (x132 - x133 + x134) + sensor_z * (x123 + x125 + x130);
	const GEN_FLT x136 = x11 * x135;
	const GEN_FLT x137 = obj_qi * x119;
	const GEN_FLT x138 = x127 * x22;
	const GEN_FLT x139 = x124 * x30;
	const GEN_FLT x140 = x126 * x48;
	const GEN_FLT x141 = x122 * x128;
	const GEN_FLT x142 = x25 * x33;
	const GEN_FLT x143 = x140 + x141 + x142 * x29;
	const GEN_FLT x144 = -x123;
	const GEN_FLT x145 = sensor_x * (-x125 + x130 + x144) + sensor_y * (x138 + x139 + x143) +
						 sensor_z * (x116 * x46 + x117 + x18 * ((x20) ? (-x137 * x14) : (0)));
	const GEN_FLT x146 = x145 * x45;
	const GEN_FLT x147 = sensor_x * (x131 + x133 + x134) +
						 sensor_y * (x116 * x56 + x117 + x18 * ((x20) ? (-x13 * x137) : (0))) +
						 sensor_z * (-x138 - x139 + x143);
	const GEN_FLT x148 = x147 * x55;
	const GEN_FLT x149 = x136 + x146 + x148;
	const GEN_FLT x150 = x149 * x68;
	const GEN_FLT x151 = x110 * x145;
	const GEN_FLT x152 = x102 * x147;
	const GEN_FLT x153 = x135 * x70;
	const GEN_FLT x154 = x151 + x152 + x153;
	const GEN_FLT x155 = x154 * x71;
	const GEN_FLT x156 = x135 * x73 + x145 * x79 + x147 * x75;
	const GEN_FLT x157 = -x94 * (2 * x151 + 2 * x152 + 2 * x153);
	const GEN_FLT x158 = x88 * (x156 * x89 + x96 * (x157 - x91 * (2 * x136 + 2 * x146 + 2 * x148)));
	const GEN_FLT x159 = x22 * x24;
	const GEN_FLT x160 = -x159;
	const GEN_FLT x161 = obj_qj * x119;
	const GEN_FLT x162 = ((x20) ? (-x120 * x13 + x23) : (0));
	const GEN_FLT x163 = x162 * x22;
	const GEN_FLT x164 = x17 * x24;
	const GEN_FLT x165 = x164 * x25;
	const GEN_FLT x166 = ((x20) ? (-obj_qj * obj_qk * x120) : (0));
	const GEN_FLT x167 = x166 * x31;
	const GEN_FLT x168 = x24 * x30;
	const GEN_FLT x169 = x141 + x167 + x168 * x33;
	const GEN_FLT x170 = x166 * x22;
	const GEN_FLT x171 = -x170;
	const GEN_FLT x172 = x164 * x28;
	const GEN_FLT x173 = x122 * x48 + x162 * x31 + x168 * x26;
	const GEN_FLT x174 = sensor_x * (x159 * x21 + x160 + x18 * ((x20) ? (-x12 * x161) : (0))) +
						 sensor_y * (x171 - x172 + x173) + sensor_z * (x163 + x165 + x169);
	const GEN_FLT x175 = x11 * x174;
	const GEN_FLT x176 = x164 * x30;
	const GEN_FLT x177 = x128 * x162 + x142 * x24 + x166 * x48;
	const GEN_FLT x178 = sensor_x * (-x163 - x165 + x169) + sensor_y * (x123 + x176 + x177) +
						 sensor_z * (x159 * x46 + x160 + x18 * ((x20) ? (-x14 * x161) : (0)));
	const GEN_FLT x179 = x178 * x45;
	const GEN_FLT x180 =
		sensor_x * (x170 + x172 + x173) +
		sensor_y * (x159 * x56 + x160 + x18 * ((x20) ? (obj_qj * x118 - x119 * obj_qj * (obj_qj * obj_qj)) : (0))) +
		sensor_z * (x144 - x176 + x177);
	const GEN_FLT x181 = x180 * x55;
	const GEN_FLT x182 = x175 + x179 + x181;
	const GEN_FLT x183 = x182 * x68;
	const GEN_FLT x184 = x110 * x178;
	const GEN_FLT x185 = x174 * x70;
	const GEN_FLT x186 = x102 * x180;
	const GEN_FLT x187 = x184 + x185 + x186;
	const GEN_FLT x188 = x187 * x71;
	const GEN_FLT x189 = x174 * x73 + x178 * x79 + x180 * x75;
	const GEN_FLT x190 = -x94 * (2 * x184 + 2 * x185 + 2 * x186);
	const GEN_FLT x191 = x88 * (x189 * x89 + x96 * (x190 - x91 * (2 * x175 + 2 * x179 + 2 * x181)));
	const GEN_FLT x192 = x22 * x27;
	const GEN_FLT x193 = -x192;
	const GEN_FLT x194 = obj_qk * x119;
	const GEN_FLT x195 = x17 * x27;
	const GEN_FLT x196 = x195 * x25;
	const GEN_FLT x197 = ((x20) ? (-x120 * x14 + x23) : (0));
	const GEN_FLT x198 = x27 * x30;
	const GEN_FLT x199 = x126 * x128 + x197 * x31 + x198 * x33;
	const GEN_FLT x200 = x197 * x22;
	const GEN_FLT x201 = x195 * x28;
	const GEN_FLT x202 = x140 + x167 + x198 * x26;
	const GEN_FLT x203 = sensor_x * (x18 * ((x20) ? (-x12 * x194) : (0)) + x192 * x21 + x193) +
						 sensor_y * (-x200 - x201 + x202) + sensor_z * (x170 + x196 + x199);
	const GEN_FLT x204 = x11 * x203;
	const GEN_FLT x205 = x195 * x30;
	const GEN_FLT x206 = x128 * x166 + x142 * x27 + x197 * x48;
	const GEN_FLT x207 = sensor_x * (x200 + x201 + x202) +
						 sensor_y * (x18 * ((x20) ? (-x13 * x194) : (0)) + x192 * x56 + x193) +
						 sensor_z * (x132 - x205 + x206);
	const GEN_FLT x208 = x207 * x55;
	const GEN_FLT x209 =
		sensor_x * (x171 - x196 + x199) + sensor_y * (x131 + x205 + x206) +
		sensor_z * (x18 * ((x20) ? (obj_qk * x118 - x119 * obj_qk * (obj_qk * obj_qk)) : (0)) + x192 * x46 + x193);
	const GEN_FLT x210 = x209 * x45;
	const GEN_FLT x211 = x204 + x208 + x210;
	const GEN_FLT x212 = x211 * x68;
	const GEN_FLT x213 = x110 * x209;
	const GEN_FLT x214 = x203 * x70;
	const GEN_FLT x215 = x102 * x207;
	const GEN_FLT x216 = x213 + x214 + x215;
	const GEN_FLT x217 = x216 * x71;
	const GEN_FLT x218 = x203 * x73 + x207 * x75 + x209 * x79;
	const GEN_FLT x219 = -x94 * (2 * x213 + 2 * x214 + 2 * x215);
	const GEN_FLT x220 = x88 * (x218 * x89 + x96 * (x219 - x91 * (2 * x204 + 2 * x208 + 2 * x210)));
	const GEN_FLT x221 = x65 + x85 * x85;
	const GEN_FLT x222 = 1.0 / x221;
	const GEN_FLT x223 = x222 * x81;
	const GEN_FLT x224 = x223 * x70;
	const GEN_FLT x225 = x53 - x54;
	const GEN_FLT x226 = x222 * x64;
	const GEN_FLT x227 = x225 * x226;
	const GEN_FLT x228 = curve_1 * x98;
	const GEN_FLT x229 = pow(-x222 * x60 * tilt_1 * tilt_1 + 1, -1.0 / 2.0);
	const GEN_FLT x230 = tilt_1 / sqrt(x221);
	const GEN_FLT x231 = (1.0 / 2.0) * x85;
	const GEN_FLT x232 = tilt_1 * x59 / pow(x221, 3.0 / 2.0);
	const GEN_FLT x233 = x229 * (x11 * x230 + x232 * (-x231 * (x104 - x105) + x95));
	const GEN_FLT x234 =
		gibMag_1 * sin(-gibPhase_1 + phase_1 + asin(x230 * x59) + atan2(x85, x64) - 1.5707963267948966);
	const GEN_FLT x235 = x6 - x74;
	const GEN_FLT x236 = x226 * x235;
	const GEN_FLT x237 = x102 * x223;
	const GEN_FLT x238 = x229 * (x230 * x55 + x232 * (x108 - x231 * (x113 - 2 * x74)));
	const GEN_FLT x239 = x110 * x223;
	const GEN_FLT x240 = x101 + x62;
	const GEN_FLT x241 = x226 * x240;
	const GEN_FLT x242 = x229 * (x230 * x45 + x232 * (x114 - x231 * (x106 + x107)));
	const GEN_FLT x243 = x154 * x223;
	const GEN_FLT x244 = x147 * x235;
	const GEN_FLT x245 = x145 * x240;
	const GEN_FLT x246 = x135 * x225;
	const GEN_FLT x247 = x226 * (x244 + x245 + x246);
	const GEN_FLT x248 = x229 * (x149 * x230 + x232 * (x157 - x231 * (2 * x244 + 2 * x245 + 2 * x246)));
	const GEN_FLT x249 = x187 * x223;
	const GEN_FLT x250 = x180 * x235;
	const GEN_FLT x251 = x178 * x240;
	const GEN_FLT x252 = x174 * x225;
	const GEN_FLT x253 = x226 * (x250 + x251 + x252);
	const GEN_FLT x254 = x229 * (x182 * x230 + x232 * (x190 - x231 * (2 * x250 + 2 * x251 + 2 * x252)));
	const GEN_FLT x255 = x216 * x223;
	const GEN_FLT x256 = x207 * x235;
	const GEN_FLT x257 = x203 * x225;
	const GEN_FLT x258 = x209 * x240;
	const GEN_FLT x259 = x226 * (x256 + x257 + x258);
	const GEN_FLT x260 = x229 * (x211 * x230 + x232 * (x219 - x231 * (2 * x256 + 2 * x257 + 2 * x258)));
	*(out++) = -x69 - x72 + x87 * (x70 * x86 + x73 * x84) - x97 + x99 * (x69 + x72 + x97);
	*(out++) = -x100 - x103 - x109 + x87 * (x102 * x86 + x75 * x84) + x99 * (x100 + x103 + x109);
	*(out++) = -x111 - x112 - x115 + x87 * (x110 * x86 + x79 * x84) + x99 * (x111 + x112 + x115);
	*(out++) = -x150 - x155 - x158 + x87 * (x154 * x86 + x156 * x84) + x99 * (x150 + x155 + x158);
	*(out++) = -x183 - x188 - x191 + x87 * (x187 * x86 + x189 * x84) + x99 * (x183 + x188 + x191);
	*(out++) = -x212 - x217 - x220 + x87 * (x216 * x86 + x218 * x84) + x99 * (x212 + x217 + x220);
	*(out++) = -x224 - x227 + x228 * (2 * x69 + 2 * x72) - x233 + x234 * (x224 + x227 + x233);
	*(out++) = x228 * (2 * x100 + 2 * x103) + x234 * (x236 + x237 + x238) - x236 - x237 - x238;
	*(out++) = x228 * (2 * x111 + 2 * x112) + x234 * (x239 + x241 + x242) - x239 - x241 - x242;
	*(out++) = x228 * (2 * x150 + 2 * x155) + x234 * (x243 + x247 + x248) - x243 - x247 - x248;
	*(out++) = x228 * (2 * x183 + 2 * x188) + x234 * (x249 + x253 + x254) - x249 - x253 - x254;
	*(out++) = x228 * (2 * x212 + 2 * x217) + x234 * (x255 + x259 + x260) - x255 - x259 - x260;
}

static inline void gen_reproject_axisangle_axis_x_jac_obj_p(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
															const FLT phase_0, const FLT tilt_0, const FLT curve_0,
															const FLT gibPhase_0, const FLT gibMag_0) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = -x5;
	const GEN_FLT x7 = x6 + 1;
	const GEN_FLT x8 = 1.0 / x3;
	const GEN_FLT x9 = x4 > 0;
	const GEN_FLT x10 = x7 * ((x9) ? (x0 * x8) : (1));
	const GEN_FLT x11 = x10 + x5;
	const GEN_FLT x12 = obj_qi * obj_qi;
	const GEN_FLT x13 = obj_qj * obj_qj;
	const GEN_FLT x14 = obj_qk * obj_qk;
	const GEN_FLT x15 = x12 + x13 + x14;
	const GEN_FLT x16 = sqrt(x15);
	const GEN_FLT x17 = cos(x16);
	const GEN_FLT x18 = 1 - x17;
	const GEN_FLT x19 = 1.0 / x15;
	const GEN_FLT x20 = x16 > 0;
	const GEN_FLT x21 = ((x20) ? (x12 * x19) : (1));
	const GEN_FLT x22 = sin(x16);
	const GEN_FLT x23 = 1.0 / x16;
	const GEN_FLT x24 = obj_qj * x23;
	const GEN_FLT x25 = ((x20) ? (x24) : (0));
	const GEN_FLT x26 = x22 * x25;
	const GEN_FLT x27 = obj_qk * x23;
	const GEN_FLT x28 = ((x20) ? (x27) : (0));
	const GEN_FLT x29 = obj_qi * x23;
	const GEN_FLT x30 = ((x20) ? (x29) : (1));
	const GEN_FLT x31 = x18 * x30;
	const GEN_FLT x32 = x28 * x31;
	const GEN_FLT x33 = x22 * x28;
	const GEN_FLT x34 = x25 * x31;
	const GEN_FLT x35 = obj_px + sensor_x * (x17 + x18 * x21) + sensor_y * (-x33 + x34) + sensor_z * (x26 + x32);
	const GEN_FLT x36 = x11 * x35;
	const GEN_FLT x37 = sin(x4);
	const GEN_FLT x38 = 1.0 / x4;
	const GEN_FLT x39 = ((x9) ? (lh_qj * x38) : (0));
	const GEN_FLT x40 = x37 * x39;
	const GEN_FLT x41 = ((x9) ? (lh_qk * x38) : (0));
	const GEN_FLT x42 = ((x9) ? (lh_qi * x38) : (1));
	const GEN_FLT x43 = x42 * x7;
	const GEN_FLT x44 = x41 * x43;
	const GEN_FLT x45 = x40 + x44;
	const GEN_FLT x46 = ((x20) ? (x14 * x19) : (0));
	const GEN_FLT x47 = x22 * x30;
	const GEN_FLT x48 = x18 * x25;
	const GEN_FLT x49 = x28 * x48;
	const GEN_FLT x50 = obj_pz + sensor_x * (-x26 + x32) + sensor_y * (x47 + x49) + sensor_z * (x17 + x18 * x46);
	const GEN_FLT x51 = x45 * x50;
	const GEN_FLT x52 = x37 * x41;
	const GEN_FLT x53 = x39 * x43;
	const GEN_FLT x54 = -x52 + x53;
	const GEN_FLT x55 = ((x20) ? (x13 * x19) : (0));
	const GEN_FLT x56 = obj_py + sensor_x * (x33 + x34) + sensor_y * (x17 + x18 * x55) + sensor_z * (-x47 + x49);
	const GEN_FLT x57 = x54 * x56;
	const GEN_FLT x58 = lh_px + x36 + x51 + x57;
	const GEN_FLT x59 = x7 * ((x9) ? (x2 * x8) : (0));
	const GEN_FLT x60 = x37 * x42;
	const GEN_FLT x61 = x39 * x41 * x7;
	const GEN_FLT x62 = -lh_pz - x35 * (-x40 + x44) - x50 * (x5 + x59) - x56 * (x60 + x61);
	const GEN_FLT x63 = x62 * x62;
	const GEN_FLT x64 = x58 * x58 + x63;
	const GEN_FLT x65 = 1.0 / x64;
	const GEN_FLT x66 = x62 * x65;
	const GEN_FLT x67 = x11 * x66;
	const GEN_FLT x68 = x40 - x44;
	const GEN_FLT x69 = x65 * (-lh_px - x36 - x51 - x57);
	const GEN_FLT x70 = x68 * x69;
	const GEN_FLT x71 = x52 + x53;
	const GEN_FLT x72 = x5 + x7 * ((x9) ? (x1 * x8) : (0));
	const GEN_FLT x73 = x56 * x72;
	const GEN_FLT x74 = x35 * x71;
	const GEN_FLT x75 = -x60;
	const GEN_FLT x76 = x61 + x75;
	const GEN_FLT x77 = x50 * x76;
	const GEN_FLT x78 = lh_py + x73 + x74 + x77;
	const GEN_FLT x79 = x78 * x78;
	const GEN_FLT x80 = 2 / (x63 + x79);
	const GEN_FLT x81 = x62 * x80;
	const GEN_FLT x82 = x80 * (-lh_py - x73 - x74 - x77);
	const GEN_FLT x83 = curve_0 * atan2(x78, x62);
	const GEN_FLT x84 = pow(-x65 * x79 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x85 = tilt_0 / sqrt(x64);
	const GEN_FLT x86 = 2 * x5;
	const GEN_FLT x87 = (1.0 / 2.0) * x58;
	const GEN_FLT x88 = 2 * x40;
	const GEN_FLT x89 = 2 * x44;
	const GEN_FLT x90 = (1.0 / 2.0) * x62;
	const GEN_FLT x91 = tilt_0 * x78 / pow(x64, 3.0 / 2.0);
	const GEN_FLT x92 = x84 * (x71 * x85 + x91 * (-x87 * (2 * x10 + x86) - x90 * (x88 - x89)));
	const GEN_FLT x93 = gibMag_0 * sin(-gibPhase_0 + phase_0 + asin(x78 * x85) + atan2(x58, x62) - 1.5707963267948966);
	const GEN_FLT x94 = x54 * x66;
	const GEN_FLT x95 = -x61 + x75;
	const GEN_FLT x96 = x69 * x95;
	const GEN_FLT x97 = x84 * (x72 * x85 + x91 * (-x87 * (-2 * x52 + 2 * x53) - x90 * (-2 * x60 - 2 * x61)));
	const GEN_FLT x98 = -x59 + x6;
	const GEN_FLT x99 = x69 * x98;
	const GEN_FLT x100 = x45 * x66;
	const GEN_FLT x101 = x84 * (x76 * x85 + x91 * (-x87 * (x88 + x89) - x90 * (-2 * x59 - x86)));
	const GEN_FLT x102 = x22 * x29;
	const GEN_FLT x103 = -x102;
	const GEN_FLT x104 = 2 * x19;
	const GEN_FLT x105 = 2 / ((x15 * x15));
	const GEN_FLT x106 = pow(x15, -3.0 / 2.0);
	const GEN_FLT x107 = obj_qi * x106;
	const GEN_FLT x108 = ((x20) ? (-obj_qj * x107) : (0));
	const GEN_FLT x109 = x108 * x22;
	const GEN_FLT x110 = x17 * x29;
	const GEN_FLT x111 = x110 * x25;
	const GEN_FLT x112 = ((x20) ? (-obj_qk * x107) : (0));
	const GEN_FLT x113 = ((x20) ? (-x106 * x12 + x23) : (0));
	const GEN_FLT x114 = x18 * x28;
	const GEN_FLT x115 = x29 * x30;
	const GEN_FLT x116 = x112 * x31 + x113 * x114 + x115 * x33;
	const GEN_FLT x117 = x112 * x22;
	const GEN_FLT x118 = -x117;
	const GEN_FLT x119 = x110 * x28;
	const GEN_FLT x120 = x108 * x31 + x113 * x48 + x115 * x26;
	const GEN_FLT x121 =
		sensor_x * (x102 * x21 + x103 + x18 * ((x20) ? (obj_qi * x104 - x105 * obj_qi * (obj_qi * obj_qi)) : (0))) +
		sensor_y * (x118 - x119 + x120) + sensor_z * (x109 + x111 + x116);
	const GEN_FLT x122 = x11 * x121;
	const GEN_FLT x123 = obj_qi * x105;
	const GEN_FLT x124 = x113 * x22;
	const GEN_FLT x125 = x110 * x30;
	const GEN_FLT x126 = x112 * x48;
	const GEN_FLT x127 = x108 * x114;
	const GEN_FLT x128 = x25 * x33;
	const GEN_FLT x129 = x126 + x127 + x128 * x29;
	const GEN_FLT x130 = -x109;
	const GEN_FLT x131 = sensor_x * (-x111 + x116 + x130) + sensor_y * (x124 + x125 + x129) +
						 sensor_z * (x102 * x46 + x103 + x18 * ((x20) ? (-x123 * x14) : (0)));
	const GEN_FLT x132 = x131 * x45;
	const GEN_FLT x133 = sensor_x * (x117 + x119 + x120) +
						 sensor_y * (x102 * x55 + x103 + x18 * ((x20) ? (-x123 * x13) : (0))) +
						 sensor_z * (-x124 - x125 + x129);
	const GEN_FLT x134 = x133 * x54;
	const GEN_FLT x135 = x66 * (x122 + x132 + x134);
	const GEN_FLT x136 = x131 * x98;
	const GEN_FLT x137 = x133 * x95;
	const GEN_FLT x138 = x121 * x68;
	const GEN_FLT x139 = x136 + x137 + x138;
	const GEN_FLT x140 = x139 * x69;
	const GEN_FLT x141 = x121 * x71 + x131 * x76 + x133 * x72;
	const GEN_FLT x142 =
		x84 * (x141 * x85 + x91 * (-x87 * (2 * x122 + 2 * x132 + 2 * x134) - x90 * (2 * x136 + 2 * x137 + 2 * x138)));
	const GEN_FLT x143 = x22 * x24;
	const GEN_FLT x144 = -x143;
	const GEN_FLT x145 = obj_qj * x105;
	const GEN_FLT x146 = ((x20) ? (-x106 * x13 + x23) : (0));
	const GEN_FLT x147 = x146 * x22;
	const GEN_FLT x148 = x17 * x24;
	const GEN_FLT x149 = x148 * x25;
	const GEN_FLT x150 = ((x20) ? (-obj_qj * obj_qk * x106) : (0));
	const GEN_FLT x151 = x150 * x31;
	const GEN_FLT x152 = x24 * x30;
	const GEN_FLT x153 = x127 + x151 + x152 * x33;
	const GEN_FLT x154 = x150 * x22;
	const GEN_FLT x155 = -x154;
	const GEN_FLT x156 = x148 * x28;
	const GEN_FLT x157 = x108 * x48 + x146 * x31 + x152 * x26;
	const GEN_FLT x158 = sensor_x * (x143 * x21 + x144 + x18 * ((x20) ? (-x12 * x145) : (0))) +
						 sensor_y * (x155 - x156 + x157) + sensor_z * (x147 + x149 + x153);
	const GEN_FLT x159 = x11 * x158;
	const GEN_FLT x160 = x148 * x30;
	const GEN_FLT x161 = x114 * x146 + x128 * x24 + x150 * x48;
	const GEN_FLT x162 = sensor_x * (-x147 - x149 + x153) + sensor_y * (x109 + x160 + x161) +
						 sensor_z * (x143 * x46 + x144 + x18 * ((x20) ? (-x14 * x145) : (0)));
	const GEN_FLT x163 = x162 * x45;
	const GEN_FLT x164 =
		sensor_x * (x154 + x156 + x157) +
		sensor_y * (x143 * x55 + x144 + x18 * ((x20) ? (obj_qj * x104 - x105 * obj_qj * (obj_qj * obj_qj)) : (0))) +
		sensor_z * (x130 - x160 + x161);
	const GEN_FLT x165 = x164 * x54;
	const GEN_FLT x166 = x66 * (x159 + x163 + x165);
	const GEN_FLT x167 = x162 * x98;
	const GEN_FLT x168 = x158 * x68;
	const GEN_FLT x169 = x164 * x95;
	const GEN_FLT x170 = x167 + x168 + x169;
	const GEN_FLT x171 = x170 * x69;
	const GEN_FLT x172 = x158 * x71 + x162 * x76 + x164 * x72;
	const GEN_FLT x173 =
		x84 * (x172 * x85 + x91 * (-x87 * (2 * x159 + 2 * x163 + 2 * x165) - x90 * (2 * x167 + 2 * x168 + 2 * x169)));
	const GEN_FLT x174 = x22 * x27;
	const GEN_FLT x175 = -x174;
	const GEN_FLT x176 = obj_qk * x105;
	const GEN_FLT x177 = x17 * x27;
	const GEN_FLT x178 = x177 * x25;
	const GEN_FLT x179 = ((x20) ? (-x106 * x14 + x23) : (0));
	const GEN_FLT x180 = x27 * x30;
	const GEN_FLT x181 = x112 * x114 + x179 * x31 + x180 * x33;
	const GEN_FLT x182 = x179 * x22;
	const GEN_FLT x183 = x177 * x28;
	const GEN_FLT x184 = x126 + x151 + x180 * x26;
	const GEN_FLT x185 = sensor_x * (x174 * x21 + x175 + x18 * ((x20) ? (-x12 * x176) : (0))) +
						 sensor_y * (-x182 - x183 + x184) + sensor_z * (x154 + x178 + x181);
	const GEN_FLT x186 = x11 * x185;
	const GEN_FLT x187 = x177 * x30;
	const GEN_FLT x188 = x114 * x150 + x128 * x27 + x179 * x48;
	const GEN_FLT x189 = sensor_x * (x182 + x183 + x184) +
						 sensor_y * (x174 * x55 + x175 + x18 * ((x20) ? (-x13 * x176) : (0))) +
						 sensor_z * (x118 - x187 + x188);
	const GEN_FLT x190 = x189 * x54;
	const GEN_FLT x191 =
		sensor_x * (x155 - x178 + x181) + sensor_y * (x117 + x187 + x188) +
		sensor_z * (x174 * x46 + x175 + x18 * ((x20) ? (obj_qk * x104 - x105 * obj_qk * (obj_qk * obj_qk)) : (0)));
	const GEN_FLT x192 = x191 * x45;
	const GEN_FLT x193 = x66 * (x186 + x190 + x192);
	const GEN_FLT x194 = x191 * x98;
	const GEN_FLT x195 = x185 * x68;
	const GEN_FLT x196 = x189 * x95;
	const GEN_FLT x197 = x194 + x195 + x196;
	const GEN_FLT x198 = x197 * x69;
	const GEN_FLT x199 = x185 * x71 + x189 * x72 + x191 * x76;
	const GEN_FLT x200 =
		x84 * (x199 * x85 + x91 * (-x87 * (2 * x186 + 2 * x190 + 2 * x192) - x90 * (2 * x194 + 2 * x195 + 2 * x196)));
	*(out++) = -x67 - x70 + x83 * (x68 * x82 + x71 * x81) - x92 + x93 * (x67 + x70 + x92);
	*(out++) = x83 * (x72 * x81 + x82 * x95) + x93 * (x94 + x96 + x97) - x94 - x96 - x97;
	*(out++) = -x100 - x101 + x83 * (x76 * x81 + x82 * x98) + x93 * (x100 + x101 + x99) - x99;
	*(out++) = -x135 - x140 - x142 + x83 * (x139 * x82 + x141 * x81) + x93 * (x135 + x140 + x142);
	*(out++) = -x166 - x171 - x173 + x83 * (x170 * x82 + x172 * x81) + x93 * (x166 + x171 + x173);
	*(out++) = -x193 - x198 - x200 + x83 * (x197 * x82 + x199 * x81) + x93 * (x193 + x198 + x200);
}

static inline void gen_reproject_axisangle_axis_y_jac_obj_p(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
															const FLT phase_0, const FLT tilt_0, const FLT curve_0,
															const FLT gibPhase_0, const FLT gibMag_0) {
	const GEN_FLT obj_px = *(obj++);
	const GEN_FLT obj_py = *(obj++);
	const GEN_FLT obj_pz = *(obj++);
	const GEN_FLT obj_qi = *(obj++);
	const GEN_FLT obj_qj = *(obj++);
	const GEN_FLT obj_qk = *(obj++);
	const GEN_FLT sensor_x = *(sensor++);
	const GEN_FLT sensor_y = *(sensor++);
	const GEN_FLT sensor_z = *(sensor++);
	const GEN_FLT lh_px = *(lh++);
	const GEN_FLT lh_py = *(lh++);
	const GEN_FLT lh_pz = *(lh++);
	const GEN_FLT lh_qi = *(lh++);
	const GEN_FLT lh_qj = *(lh++);
	const GEN_FLT lh_qk = *(lh++);
	const GEN_FLT x0 = lh_qi * lh_qi;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x0 + x1 + x2;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = sin(x4);
	const GEN_FLT x6 = 1.0 / x4;
	const GEN_FLT x7 = x4 > 0;
	const GEN_FLT x8 = ((x7) ? (lh_qj * x6) : (0));
	const GEN_FLT x9 = x5 * x8;
	const GEN_FLT x10 = cos(x4);
	const GEN_FLT x11 = -x10;
	const GEN_FLT x12 = x11 + 1;
	const GEN_FLT x13 = ((x7) ? (lh_qi * x6) : (1));
	const GEN_FLT x14 = ((x7) ? (lh_qk * x6) : (0));
	const GEN_FLT x15 = x12 * x13 * x14;
	const GEN_FLT x16 = -x15 + x9;
	const GEN_FLT x17 = 1.0 / x3;
	const GEN_FLT x18 = x12 * ((x7) ? (x1 * x17) : (0));
	const GEN_FLT x19 = obj_qi * obj_qi;
	const GEN_FLT x20 = obj_qj * obj_qj;
	const GEN_FLT x21 = obj_qk * obj_qk;
	const GEN_FLT x22 = x19 + x20 + x21;
	const GEN_FLT x23 = sqrt(x22);
	const GEN_FLT x24 = cos(x23);
	const GEN_FLT x25 = 1 - x24;
	const GEN_FLT x26 = 1.0 / x22;
	const GEN_FLT x27 = x23 > 0;
	const GEN_FLT x28 = ((x27) ? (x20 * x26) : (0));
	const GEN_FLT x29 = sin(x23);
	const GEN_FLT x30 = 1.0 / x23;
	const GEN_FLT x31 = obj_qk * x30;
	const GEN_FLT x32 = ((x27) ? (x31) : (0));
	const GEN_FLT x33 = x29 * x32;
	const GEN_FLT x34 = obj_qi * x30;
	const GEN_FLT x35 = ((x27) ? (x34) : (1));
	const GEN_FLT x36 = obj_qj * x30;
	const GEN_FLT x37 = ((x27) ? (x36) : (0));
	const GEN_FLT x38 = x25 * x37;
	const GEN_FLT x39 = x35 * x38;
	const GEN_FLT x40 = x29 * x35;
	const GEN_FLT x41 = x32 * x38;
	const GEN_FLT x42 = obj_py + sensor_x * (x33 + x39) + sensor_y * (x24 + x25 * x28) + sensor_z * (-x40 + x41);
	const GEN_FLT x43 = x42 * (x10 + x18);
	const GEN_FLT x44 = x14 * x5;
	const GEN_FLT x45 = x12 * x8;
	const GEN_FLT x46 = x13 * x45;
	const GEN_FLT x47 = ((x27) ? (x19 * x26) : (1));
	const GEN_FLT x48 = x29 * x37;
	const GEN_FLT x49 = x25 * x35;
	const GEN_FLT x50 = x32 * x49;
	const GEN_FLT x51 = obj_px + sensor_x * (x24 + x25 * x47) + sensor_y * (-x33 + x39) + sensor_z * (x48 + x50);
	const GEN_FLT x52 = x51 * (x44 + x46);
	const GEN_FLT x53 = x13 * x5;
	const GEN_FLT x54 = -x53;
	const GEN_FLT x55 = x14 * x45;
	const GEN_FLT x56 = ((x27) ? (x21 * x26) : (0));
	const GEN_FLT x57 = obj_pz + sensor_x * (-x48 + x50) + sensor_y * (x40 + x41) + sensor_z * (x24 + x25 * x56);
	const GEN_FLT x58 = x57 * (x54 + x55);
	const GEN_FLT x59 = -lh_py - x43 - x52 - x58;
	const GEN_FLT x60 = x12 * ((x7) ? (x17 * x2) : (0));
	const GEN_FLT x61 = -lh_pz - x42 * (x53 + x55) - x51 * (x15 - x9) - x57 * (x10 + x60);
	const GEN_FLT x62 = x61 * x61;
	const GEN_FLT x63 = x59 * x59 + x62;
	const GEN_FLT x64 = 1.0 / x63;
	const GEN_FLT x65 = x64 * (lh_py + x43 + x52 + x58);
	const GEN_FLT x66 = x16 * x65;
	const GEN_FLT x67 = -x44;
	const GEN_FLT x68 = -x46 + x67;
	const GEN_FLT x69 = x61 * x64;
	const GEN_FLT x70 = x68 * x69;
	const GEN_FLT x71 = x10 + x12 * ((x7) ? (x0 * x17) : (1));
	const GEN_FLT x72 = x51 * x71;
	const GEN_FLT x73 = x15 + x9;
	const GEN_FLT x74 = x57 * x73;
	const GEN_FLT x75 = x46 + x67;
	const GEN_FLT x76 = x42 * x75;
	const GEN_FLT x77 = lh_px + x72 + x74 + x76;
	const GEN_FLT x78 = x77 * x77;
	const GEN_FLT x79 = 2 / (x62 + x78);
	const GEN_FLT x80 = x61 * x79;
	const GEN_FLT x81 = x79 * (-lh_px - x72 - x74 - x76);
	const GEN_FLT x82 = curve_0 * atan2(x77, x61);
	const GEN_FLT x83 = pow(-x64 * x78 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x84 = tilt_0 / sqrt(x63);
	const GEN_FLT x85 = (1.0 / 2.0) * x59;
	const GEN_FLT x86 = (1.0 / 2.0) * x61;
	const GEN_FLT x87 = tilt_0 * x77 / pow(x63, 3.0 / 2.0);
	const GEN_FLT x88 = x83 * (x71 * x84 + x87 * (-x85 * (-2 * x44 - 2 * x46) - x86 * (-2 * x15 + 2 * x9)));
	const GEN_FLT x89 = gibMag_0 * sin(-gibPhase_0 + phase_0 + asin(x77 * x84) + atan2(x59, x61) - 1.5707963267948966);
	const GEN_FLT x90 = x11 - x18;
	const GEN_FLT x91 = x69 * x90;
	const GEN_FLT x92 = -x55;
	const GEN_FLT x93 = x54 + x92;
	const GEN_FLT x94 = x65 * x93;
	const GEN_FLT x95 = -2 * x10;
	const GEN_FLT x96 = 2 * x53;
	const GEN_FLT x97 = -2 * x55;
	const GEN_FLT x98 = x83 * (x75 * x84 + x87 * (-x85 * (-2 * x18 + x95) - x86 * (-x96 + x97)));
	const GEN_FLT x99 = x11 - x60;
	const GEN_FLT x100 = x65 * x99;
	const GEN_FLT x101 = x53 + x92;
	const GEN_FLT x102 = x101 * x69;
	const GEN_FLT x103 = x83 * (x73 * x84 + x87 * (-x85 * (x96 + x97) - x86 * (-2 * x60 + x95)));
	const GEN_FLT x104 = x29 * x34;
	const GEN_FLT x105 = -x104;
	const GEN_FLT x106 = 2 / ((x22 * x22));
	const GEN_FLT x107 = obj_qi * x106;
	const GEN_FLT x108 = pow(x22, -3.0 / 2.0);
	const GEN_FLT x109 = ((x27) ? (-x108 * x19 + x30) : (0));
	const GEN_FLT x110 = x109 * x29;
	const GEN_FLT x111 = x24 * x34;
	const GEN_FLT x112 = x111 * x35;
	const GEN_FLT x113 = obj_qi * x108;
	const GEN_FLT x114 = ((x27) ? (-obj_qk * x113) : (0));
	const GEN_FLT x115 = x114 * x38;
	const GEN_FLT x116 = ((x27) ? (-obj_qj * x113) : (0));
	const GEN_FLT x117 = x25 * x32;
	const GEN_FLT x118 = x116 * x117;
	const GEN_FLT x119 = x34 * x37;
	const GEN_FLT x120 = x115 + x118 + x119 * x33;
	const GEN_FLT x121 = x116 * x29;
	const GEN_FLT x122 = -x121;
	const GEN_FLT x123 = x111 * x37;
	const GEN_FLT x124 = x33 * x35;
	const GEN_FLT x125 = x109 * x117 + x114 * x49 + x124 * x34;
	const GEN_FLT x126 = sensor_x * (x122 - x123 + x125) + sensor_y * (x110 + x112 + x120) +
						 sensor_z * (x104 * x56 + x105 + x25 * ((x27) ? (-x107 * x21) : (0)));
	const GEN_FLT x127 = x126 * x99;
	const GEN_FLT x128 = x114 * x29;
	const GEN_FLT x129 = x111 * x32;
	const GEN_FLT x130 = x109 * x38 + x116 * x49 + x119 * x40;
	const GEN_FLT x131 = sensor_x * (x128 + x129 + x130) +
						 sensor_y * (x104 * x28 + x105 + x25 * ((x27) ? (-x107 * x20) : (0))) +
						 sensor_z * (-x110 - x112 + x120);
	const GEN_FLT x132 = x131 * x93;
	const GEN_FLT x133 = 2 * x26;
	const GEN_FLT x134 = -x128;
	const GEN_FLT x135 =
		sensor_x * (x104 * x47 + x105 + x25 * ((x27) ? (obj_qi * x133 - x106 * obj_qi * (obj_qi * obj_qi)) : (0))) +
		sensor_y * (-x129 + x130 + x134) + sensor_z * (x121 + x123 + x125);
	const GEN_FLT x136 = x135 * x16;
	const GEN_FLT x137 = x127 + x132 + x136;
	const GEN_FLT x138 = x137 * x65;
	const GEN_FLT x139 = x131 * x90;
	const GEN_FLT x140 = x101 * x126;
	const GEN_FLT x141 = x135 * x68;
	const GEN_FLT x142 = x69 * (x139 + x140 + x141);
	const GEN_FLT x143 = x126 * x73 + x131 * x75 + x135 * x71;
	const GEN_FLT x144 =
		x83 * (x143 * x84 + x87 * (-x85 * (2 * x139 + 2 * x140 + 2 * x141) - x86 * (2 * x127 + 2 * x132 + 2 * x136)));
	const GEN_FLT x145 = x29 * x36;
	const GEN_FLT x146 = -x145;
	const GEN_FLT x147 = obj_qj * x106;
	const GEN_FLT x148 = x24 * x36;
	const GEN_FLT x149 = x148 * x35;
	const GEN_FLT x150 = ((x27) ? (-obj_qj * obj_qk * x108) : (0));
	const GEN_FLT x151 = ((x27) ? (-x108 * x20 + x30) : (0));
	const GEN_FLT x152 = x36 * x37;
	const GEN_FLT x153 = x117 * x151 + x150 * x38 + x152 * x33;
	const GEN_FLT x154 = x151 * x29;
	const GEN_FLT x155 = x148 * x37;
	const GEN_FLT x156 = x150 * x49;
	const GEN_FLT x157 = x118 + x124 * x36 + x156;
	const GEN_FLT x158 = sensor_x * (-x154 - x155 + x157) + sensor_y * (x121 + x149 + x153) +
						 sensor_z * (x145 * x56 + x146 + x25 * ((x27) ? (-x147 * x21) : (0)));
	const GEN_FLT x159 = x158 * x99;
	const GEN_FLT x160 = x150 * x29;
	const GEN_FLT x161 = -x160;
	const GEN_FLT x162 = x148 * x32;
	const GEN_FLT x163 = x116 * x38 + x151 * x49 + x152 * x40;
	const GEN_FLT x164 = sensor_x * (x145 * x47 + x146 + x25 * ((x27) ? (-x147 * x19) : (0))) +
						 sensor_y * (x161 - x162 + x163) + sensor_z * (x154 + x155 + x157);
	const GEN_FLT x165 = x16 * x164;
	const GEN_FLT x166 =
		sensor_x * (x160 + x162 + x163) +
		sensor_y * (x145 * x28 + x146 + x25 * ((x27) ? (obj_qj * x133 - x106 * obj_qj * (obj_qj * obj_qj)) : (0))) +
		sensor_z * (x122 - x149 + x153);
	const GEN_FLT x167 = x166 * x93;
	const GEN_FLT x168 = x159 + x165 + x167;
	const GEN_FLT x169 = x168 * x65;
	const GEN_FLT x170 = x166 * x90;
	const GEN_FLT x171 = x101 * x158;
	const GEN_FLT x172 = x164 * x68;
	const GEN_FLT x173 = x69 * (x170 + x171 + x172);
	const GEN_FLT x174 = x158 * x73 + x164 * x71 + x166 * x75;
	const GEN_FLT x175 =
		x83 * (x174 * x84 + x87 * (-x85 * (2 * x170 + 2 * x171 + 2 * x172) - x86 * (2 * x159 + 2 * x165 + 2 * x167)));
	const GEN_FLT x176 = x29 * x31;
	const GEN_FLT x177 = -x176;
	const GEN_FLT x178 = x24 * x31;
	const GEN_FLT x179 = x178 * x35;
	const GEN_FLT x180 = ((x27) ? (-x108 * x21 + x30) : (0));
	const GEN_FLT x181 = x31 * x37;
	const GEN_FLT x182 = x117 * x150 + x180 * x38 + x181 * x33;
	const GEN_FLT x183 = x178 * x37;
	const GEN_FLT x184 = x114 * x117 + x124 * x31 + x180 * x49;
	const GEN_FLT x185 =
		sensor_x * (x161 - x183 + x184) + sensor_y * (x128 + x179 + x182) +
		sensor_z * (x176 * x56 + x177 + x25 * ((x27) ? (obj_qk * x133 - x106 * obj_qk * (obj_qk * obj_qk)) : (0)));
	const GEN_FLT x186 = x185 * x99;
	const GEN_FLT x187 = obj_qk * x106;
	const GEN_FLT x188 = x180 * x29;
	const GEN_FLT x189 = x178 * x32;
	const GEN_FLT x190 = x115 + x156 + x181 * x40;
	const GEN_FLT x191 = sensor_x * (x176 * x47 + x177 + x25 * ((x27) ? (-x187 * x19) : (0))) +
						 sensor_y * (-x188 - x189 + x190) + sensor_z * (x160 + x183 + x184);
	const GEN_FLT x192 = x16 * x191;
	const GEN_FLT x193 = sensor_x * (x188 + x189 + x190) +
						 sensor_y * (x176 * x28 + x177 + x25 * ((x27) ? (-x187 * x20) : (0))) +
						 sensor_z * (x134 - x179 + x182);
	const GEN_FLT x194 = x193 * x93;
	const GEN_FLT x195 = x186 + x192 + x194;
	const GEN_FLT x196 = x195 * x65;
	const GEN_FLT x197 = x193 * x90;
	const GEN_FLT x198 = x191 * x68;
	const GEN_FLT x199 = x101 * x185;
	const GEN_FLT x200 = x69 * (x197 + x198 + x199);
	const GEN_FLT x201 = x185 * x73 + x191 * x71 + x193 * x75;
	const GEN_FLT x202 =
		x83 * (x201 * x84 + x87 * (-x85 * (2 * x197 + 2 * x198 + 2 * x199) - x86 * (2 * x186 + 2 * x192 + 2 * x194)));
	*(out++) = -x66 - x70 + x82 * (x16 * x81 + x71 * x80) - x88 + x89 * (x66 + x70 + x88);
	*(out++) = x82 * (x75 * x80 + x81 * x93) + x89 * (x91 + x94 + x98) - x91 - x94 - x98;
	*(out++) = -x100 - x102 - x103 + x82 * (x73 * x80 + x81 * x99) + x89 * (x100 + x102 + x103);
	*(out++) = -x138 - x142 - x144 + x82 * (x137 * x81 + x143 * x80) + x89 * (x138 + x142 + x144);
	*(out++) = -x169 - x173 - x175 + x82 * (x168 * x81 + x174 * x80) + x89 * (x169 + x173 + x175);
	*(out++) = -x196 - x200 - x202 + x82 * (x195 * x81 + x201 * x80) + x89 * (x196 + x200 + x202);
}
