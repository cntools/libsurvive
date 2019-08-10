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
	const GEN_FLT x37 = -x23 * x36 + 1;
	const GEN_FLT x38 = x13 + x14;
	const GEN_FLT x39 = -x28 + x29;
	const GEN_FLT x40 = x6 + x8;
	const GEN_FLT x41 = obj_py + sensor_y * (-x11 * x40 + 1) + x12 * x39 + x32 * x38;
	const GEN_FLT x42 = lh_py + x2 * x24 + x27 * x35 + x37 * x41;
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
	const GEN_FLT x60 = tilt_0 + 0.52359877559829882;
	const GEN_FLT x61 = tan(x60);
	const GEN_FLT x62 = x59 * x61;
	const GEN_FLT x63 = x42 * x62;
	const GEN_FLT x64 = x42 * x42;
	const GEN_FLT x65 = x58 + x64;
	const GEN_FLT x66 = pow(x65, -1.0 / 2.0);
	const GEN_FLT x67 = cos(x60);
	const GEN_FLT x68 = 1.0 / x67;
	const GEN_FLT x69 = x66 * x68;
	const GEN_FLT x70 = asin(x42 * x69);
	const GEN_FLT x71 = 8.0108022e-6 * x70;
	const GEN_FLT x72 = -x71 - 8.0108022e-6;
	const GEN_FLT x73 = x70 * x72 + 0.0028679863;
	const GEN_FLT x74 = x70 * x73 + 5.3685255000000001e-6;
	const GEN_FLT x75 = x70 * x74 + 0.0076069798000000001;
	const GEN_FLT x76 = x70 * x70;
	const GEN_FLT x77 = atan2(x57, x49);
	const GEN_FLT x78 = ogeePhase_0 + x77 - asin(x63);
	const GEN_FLT x79 = ogeeMag_0 * sin(x78);
	const GEN_FLT x80 = curve_0 + x79;
	const GEN_FLT x81 = x70 * x75;
	const GEN_FLT x82 = -1.60216044e-5 * x70 - 8.0108022e-6;
	const GEN_FLT x83 = x70 * x82 + x73;
	const GEN_FLT x84 = x70 * x83 + x74;
	const GEN_FLT x85 = x70 * x84 + x75;
	const GEN_FLT x86 = sin(x60);
	const GEN_FLT x87 = x86 * (x70 * x85 + x81);
	const GEN_FLT x88 = x67 - x80 * x87;
	const GEN_FLT x89 = 1.0 / x88;
	const GEN_FLT x90 = x80 * x89;
	const GEN_FLT x91 = x76 * x90;
	const GEN_FLT x92 = x63 + x75 * x91;
	const GEN_FLT x93 = pow(-x92 * x92 + 1, -1.0 / 2.0);
	const GEN_FLT x94 = x64 / x65;
	const GEN_FLT x95 = pow(-x94 / (x67 * x67) + 1, -1.0 / 2.0);
	const GEN_FLT x96 = x2 * x23;
	const GEN_FLT x97 = x23 * x42;
	const GEN_FLT x98 = 4 * x22;
	const GEN_FLT x99 = (1.0 / 2.0) * x49;
	const GEN_FLT x100 = x23 * x57;
	const GEN_FLT x101 = x100 * x52 - x99 * (-x21 * x98 + 2);
	const GEN_FLT x102 = x101 - x2 * x97;
	const GEN_FLT x103 = x42 / pow(x65, 3.0 / 2.0);
	const GEN_FLT x104 = x103 * x68;
	const GEN_FLT x105 = x95 * (x102 * x104 + x69 * x96);
	const GEN_FLT x106 = x105 * x72;
	const GEN_FLT x107 = x105 * x73 + x70 * (-x105 * x71 + x106);
	const GEN_FLT x108 = x105 * x74 + x107 * x70;
	const GEN_FLT x109 = 1.0 / x58;
	const GEN_FLT x110 = x109 * x64;
	const GEN_FLT x111 = pow(-x110 * x61 * x61 + 1, -1.0 / 2.0);
	const GEN_FLT x112 = x42 / pow(x58, 3.0 / 2.0);
	const GEN_FLT x113 = x112 * x61;
	const GEN_FLT x114 = x101 * x113 + x62 * x96;
	const GEN_FLT x115 = x109 * (lh_pz + x51 + x53 + x56);
	const GEN_FLT x116 = x109 * x49;
	const GEN_FLT x117 = x116 * x23;
	const GEN_FLT x118 = x115 * x48 - x117 * x52;
	const GEN_FLT x119 = -x111 * x114 + x118;
	const GEN_FLT x120 = ogeeMag_0 * cos(x78);
	const GEN_FLT x121 = x120 * x87;
	const GEN_FLT x122 = 2.40324066e-5 * x70;
	const GEN_FLT x123 = x86 * (-curve_0 - x79);
	const GEN_FLT x124 = x75 * x76;
	const GEN_FLT x125 = x124 * x80 / ((x88 * x88));
	const GEN_FLT x126 = x120 * x124 * x89;
	const GEN_FLT x127 = 2 * x81 * x90;
	const GEN_FLT x128 =
		x118 - x93 * (x105 * x127 + x108 * x91 + x114 + x119 * x126 +
					  x125 * (x119 * x121 -
							  x123 * (x105 * x75 + x105 * x85 + x108 * x70 +
									  x70 * (x105 * x84 + x108 +
											 x70 * (x105 * x83 + x107 + x70 * (-x105 * x122 + x105 * x82 + x106))))));
	const GEN_FLT x129 = gibMag_0 * cos(gibPhase_0 + x77 - asin(x92));
	const GEN_FLT x130 = (1.0 / 2.0) * x42;
	const GEN_FLT x131 = x23 * x49;
	const GEN_FLT x132 = x100 * x50 - x131 * x46;
	const GEN_FLT x133 = -x130 * (-x36 * x98 + 2) + x132;
	const GEN_FLT x134 = x95 * (x104 * x133 + x37 * x69);
	const GEN_FLT x135 = x134 * x72;
	const GEN_FLT x136 = x134 * x73 + x70 * (-x134 * x71 + x135);
	const GEN_FLT x137 = x134 * x74 + x136 * x70;
	const GEN_FLT x138 = x113 * x132 + x37 * x62;
	const GEN_FLT x139 = x115 * x23;
	const GEN_FLT x140 = -x117 * x50 + x139 * x46;
	const GEN_FLT x141 = -x111 * x138 + x140;
	const GEN_FLT x142 =
		x140 - x93 * (x125 * (x121 * x141 -
							  x123 * (x134 * x75 + x134 * x85 + x137 * x70 +
									  x70 * (x134 * x84 + x137 +
											 x70 * (x134 * x83 + x136 + x70 * (-x122 * x134 + x134 * x82 + x135))))) +
					  x126 * x141 + x127 * x134 + x137 * x91 + x138);
	const GEN_FLT x143 = x23 * x27;
	const GEN_FLT x144 = (1.0 / 2.0) * x57;
	const GEN_FLT x145 = -x131 * x45 - x144 * (x54 * x98 - 2);
	const GEN_FLT x146 = x145 - x27 * x97;
	const GEN_FLT x147 = x95 * (x104 * x146 + x143 * x69);
	const GEN_FLT x148 = x147 * x72;
	const GEN_FLT x149 = x147 * x73 + x70 * (-x147 * x71 + x148);
	const GEN_FLT x150 = x147 * x74 + x149 * x70;
	const GEN_FLT x151 = x113 * x145 + x143 * x62;
	const GEN_FLT x152 = x55 - 1;
	const GEN_FLT x153 = x116 * x152 + x139 * x45;
	const GEN_FLT x154 = -x111 * x151 + x153;
	const GEN_FLT x155 =
		x153 - x93 * (x125 * (x121 * x154 -
							  x123 * (x147 * x75 + x147 * x85 + x150 * x70 +
									  x70 * (x147 * x84 + x150 +
											 x70 * (x147 * x83 + x149 + x70 * (-x122 * x147 + x147 * x82 + x148))))) +
					  x126 * x154 + x127 * x147 + x150 * x91 + x151);
	const GEN_FLT x156 = obj_qj * x12;
	const GEN_FLT x157 = obj_qk * x16;
	const GEN_FLT x158 = 1.0 / x10;
	const GEN_FLT x159 = obj_qw * x158;
	const GEN_FLT x160 = 2 * x159;
	const GEN_FLT x161 = sensor_z * x160;
	const GEN_FLT x162 = sensor_y * x160;
	const GEN_FLT x163 = -2 * x7;
	const GEN_FLT x164 = -2 * x8;
	const GEN_FLT x165 = sensor_x * (x163 + x164);
	const GEN_FLT x166 = x15 * x162 + x156 - x157 + x159 * x165 + x161 * x5;
	const GEN_FLT x167 = x166 * x2;
	const GEN_FLT x168 = obj_qi * x16;
	const GEN_FLT x169 = obj_qj * x32;
	const GEN_FLT x170 = sensor_x * x31;
	const GEN_FLT x171 = -2 * x6;
	const GEN_FLT x172 = sensor_z * (x163 + x171);
	const GEN_FLT x173 = x159 * x172 + x160 * x170 + x162 * x30 + x168 - x169;
	const GEN_FLT x174 = x173 * x27;
	const GEN_FLT x175 = obj_qi * x12;
	const GEN_FLT x176 = obj_qk * x32;
	const GEN_FLT x177 = sensor_x * x38;
	const GEN_FLT x178 = sensor_y * (x164 + x171);
	const GEN_FLT x179 = x159 * x178 + x160 * x177 + x161 * x39 - x175 + x176;
	const GEN_FLT x180 = x179 * x37;
	const GEN_FLT x181 = x167 * x23 + x174 * x23 + x180;
	const GEN_FLT x182 = x173 * x45;
	const GEN_FLT x183 = x179 * x98;
	const GEN_FLT x184 = x166 * x48;
	const GEN_FLT x185 = x166 * x52;
	const GEN_FLT x186 = x152 * x173;
	const GEN_FLT x187 = -x144 * (-x183 * x50 - x185 * x98 + 2 * x186) - x99 * (x182 * x98 + x183 * x46 + 2 * x184);
	const GEN_FLT x188 = -x130 * (x167 * x98 + x174 * x98 + 2 * x180) + x187;
	const GEN_FLT x189 = x95 * (x104 * x188 + x181 * x69);
	const GEN_FLT x190 = x189 * x72;
	const GEN_FLT x191 = x189 * x73 + x70 * (-x189 * x71 + x190);
	const GEN_FLT x192 = x189 * x74 + x191 * x70;
	const GEN_FLT x193 = x113 * x187 + x181 * x62;
	const GEN_FLT x194 = x179 * x23;
	const GEN_FLT x195 = x115 * (x182 * x23 + x184 + x194 * x46) + x116 * (-x185 * x23 + x186 - x194 * x50);
	const GEN_FLT x196 = -x111 * x193 + x195;
	const GEN_FLT x197 =
		x195 - x93 * (x125 * (x121 * x196 -
							  x123 * (x189 * x75 + x189 * x85 + x192 * x70 +
									  x70 * (x189 * x84 + x192 +
											 x70 * (x189 * x83 + x191 + x70 * (-x122 * x189 + x189 * x82 + x190))))) +
					  x126 * x196 + x127 * x189 + x192 * x91 + x193);
	const GEN_FLT x198 = obj_qj * x16;
	const GEN_FLT x199 = obj_qk * x12;
	const GEN_FLT x200 = obj_qi * x158;
	const GEN_FLT x201 = 2 * x200;
	const GEN_FLT x202 = sensor_z * x201;
	const GEN_FLT x203 = sensor_y * x201;
	const GEN_FLT x204 = x15 * x203 + x165 * x200 + x198 + x199 + x202 * x5;
	const GEN_FLT x205 = x2 * x204;
	const GEN_FLT x206 = obj_qw * x16;
	const GEN_FLT x207 = 4 * x10;
	const GEN_FLT x208 = -obj_qi * x207;
	const GEN_FLT x209 = sensor_z * (-x201 * x33 + x208) + x170 * x201 + x176 + x203 * x30 + x206;
	const GEN_FLT x210 = x209 * x27;
	const GEN_FLT x211 = obj_qw * x12;
	const GEN_FLT x212 = sensor_y * (-x201 * x40 + x208) + x169 + x177 * x201 + x202 * x39 - x211;
	const GEN_FLT x213 = x212 * x37;
	const GEN_FLT x214 = x205 * x23 + x210 * x23 + x213;
	const GEN_FLT x215 = x204 * x48;
	const GEN_FLT x216 = x209 * x45;
	const GEN_FLT x217 = x212 * x98;
	const GEN_FLT x218 = x204 * x52;
	const GEN_FLT x219 = x152 * x209;
	const GEN_FLT x220 = -x144 * (-x217 * x50 - x218 * x98 + 2 * x219) - x99 * (2 * x215 + x216 * x98 + x217 * x46);
	const GEN_FLT x221 = -x130 * (x205 * x98 + x210 * x98 + 2 * x213) + x220;
	const GEN_FLT x222 = x95 * (x104 * x221 + x214 * x69);
	const GEN_FLT x223 = x222 * x72;
	const GEN_FLT x224 = x222 * x73 + x70 * (-x222 * x71 + x223);
	const GEN_FLT x225 = x222 * x74 + x224 * x70;
	const GEN_FLT x226 = x113 * x220 + x214 * x62;
	const GEN_FLT x227 = x212 * x23;
	const GEN_FLT x228 = x115 * (x215 + x216 * x23 + x227 * x46) + x116 * (-x218 * x23 + x219 - x227 * x50);
	const GEN_FLT x229 = -x111 * x226 + x228;
	const GEN_FLT x230 =
		x228 - x93 * (x125 * (x121 * x229 -
							  x123 * (x222 * x75 + x222 * x85 + x225 * x70 +
									  x70 * (x222 * x84 + x225 +
											 x70 * (x222 * x83 + x224 + x70 * (-x122 * x222 + x222 * x82 + x223))))) +
					  x126 * x229 + x127 * x222 + x225 * x91 + x226);
	const GEN_FLT x231 = obj_qi * x32;
	const GEN_FLT x232 = obj_qj * x158;
	const GEN_FLT x233 = 2 * x232;
	const GEN_FLT x234 = sensor_z * x233;
	const GEN_FLT x235 = x177 * x233 + x178 * x232 + x199 + x231 + x234 * x39;
	const GEN_FLT x236 = x235 * x37;
	const GEN_FLT x237 = sensor_y * x233;
	const GEN_FLT x238 = -obj_qj * x207;
	const GEN_FLT x239 = sensor_x * (-x233 * x9 + x238) + x15 * x237 + x168 + x211 + x234 * x5;
	const GEN_FLT x240 = x2 * x239;
	const GEN_FLT x241 = obj_qw * x32;
	const GEN_FLT x242 = sensor_z * (-x233 * x33 + x238) + x157 + x170 * x233 + x237 * x30 - x241;
	const GEN_FLT x243 = x242 * x27;
	const GEN_FLT x244 = x23 * x240 + x23 * x243 + x236;
	const GEN_FLT x245 = x235 * x98;
	const GEN_FLT x246 = x242 * x45;
	const GEN_FLT x247 = x239 * x48;
	const GEN_FLT x248 = x239 * x52;
	const GEN_FLT x249 = x152 * x242;
	const GEN_FLT x250 = -x144 * (-x245 * x50 - x248 * x98 + 2 * x249) - x99 * (x245 * x46 + x246 * x98 + 2 * x247);
	const GEN_FLT x251 = -x130 * (2 * x236 + x240 * x98 + x243 * x98) + x250;
	const GEN_FLT x252 = x95 * (x104 * x251 + x244 * x69);
	const GEN_FLT x253 = x252 * x72;
	const GEN_FLT x254 = x252 * x73 + x70 * (-x252 * x71 + x253);
	const GEN_FLT x255 = x252 * x74 + x254 * x70;
	const GEN_FLT x256 = x113 * x250 + x244 * x62;
	const GEN_FLT x257 = x23 * x235;
	const GEN_FLT x258 = x115 * (x23 * x246 + x247 + x257 * x46) + x116 * (-x23 * x248 + x249 - x257 * x50);
	const GEN_FLT x259 = -x111 * x256 + x258;
	const GEN_FLT x260 =
		x258 - x93 * (x125 * (x121 * x259 -
							  x123 * (x252 * x75 + x252 * x85 + x255 * x70 +
									  x70 * (x252 * x84 + x255 +
											 x70 * (x252 * x83 + x254 + x70 * (-x122 * x252 + x252 * x82 + x253))))) +
					  x126 * x259 + x127 * x252 + x255 * x91 + x256);
	const GEN_FLT x261 = obj_qk * x158;
	const GEN_FLT x262 = 2 * x261;
	const GEN_FLT x263 = sensor_y * x262;
	const GEN_FLT x264 = x170 * x262 + x172 * x261 + x198 + x231 + x263 * x30;
	const GEN_FLT x265 = x264 * x27;
	const GEN_FLT x266 = sensor_z * x262;
	const GEN_FLT x267 = -obj_qk * x207;
	const GEN_FLT x268 = sensor_x * (-x262 * x9 + x267) + x15 * x263 + x175 - x206 + x266 * x5;
	const GEN_FLT x269 = x2 * x268;
	const GEN_FLT x270 = sensor_y * (-x262 * x40 + x267) + x156 + x177 * x262 + x241 + x266 * x39;
	const GEN_FLT x271 = x270 * x37;
	const GEN_FLT x272 = x23 * x265 + x23 * x269 + x271;
	const GEN_FLT x273 = x264 * x45;
	const GEN_FLT x274 = x270 * x98;
	const GEN_FLT x275 = x268 * x48;
	const GEN_FLT x276 = x152 * x264;
	const GEN_FLT x277 = x268 * x52;
	const GEN_FLT x278 = -x144 * (-x274 * x50 + 2 * x276 - x277 * x98) - x99 * (x273 * x98 + x274 * x46 + 2 * x275);
	const GEN_FLT x279 = -x130 * (x265 * x98 + x269 * x98 + 2 * x271) + x278;
	const GEN_FLT x280 = x95 * (x104 * x279 + x272 * x69);
	const GEN_FLT x281 = x280 * x72;
	const GEN_FLT x282 = x280 * x73 + x70 * (-x280 * x71 + x281);
	const GEN_FLT x283 = x280 * x74 + x282 * x70;
	const GEN_FLT x284 = x113 * x278 + x272 * x62;
	const GEN_FLT x285 = x23 * x270;
	const GEN_FLT x286 = x115 * (x23 * x273 + x275 + x285 * x46) + x116 * (-x23 * x277 + x276 - x285 * x50);
	const GEN_FLT x287 = -x111 * x284 + x286;
	const GEN_FLT x288 =
		x286 - x93 * (x125 * (x121 * x287 -
							  x123 * (x280 * x75 + x280 * x85 + x283 * x70 +
									  x70 * (x280 * x84 + x283 +
											 x70 * (x280 * x83 + x282 + x70 * (-x122 * x280 + x280 * x82 + x281))))) +
					  x126 * x287 + x127 * x280 + x283 * x91 + x284);
	const GEN_FLT x289 = tilt_1 - 0.52359877559829882;
	const GEN_FLT x290 = tan(x289);
	const GEN_FLT x291 = x290 * x59;
	const GEN_FLT x292 = x291 * x42;
	const GEN_FLT x293 = cos(x289);
	const GEN_FLT x294 = 1.0 / x293;
	const GEN_FLT x295 = x294 * x66;
	const GEN_FLT x296 = asin(x295 * x42);
	const GEN_FLT x297 = 8.0108022e-6 * x296;
	const GEN_FLT x298 = -x297 - 8.0108022e-6;
	const GEN_FLT x299 = x296 * x298 + 0.0028679863;
	const GEN_FLT x300 = x296 * x299 + 5.3685255000000001e-6;
	const GEN_FLT x301 = x296 * x300 + 0.0076069798000000001;
	const GEN_FLT x302 = x296 * x296;
	const GEN_FLT x303 = ogeePhase_1 + x77 - asin(x292);
	const GEN_FLT x304 = ogeeMag_1 * sin(x303);
	const GEN_FLT x305 = curve_1 + x304;
	const GEN_FLT x306 = x296 * x301;
	const GEN_FLT x307 = -1.60216044e-5 * x296 - 8.0108022e-6;
	const GEN_FLT x308 = x296 * x307 + x299;
	const GEN_FLT x309 = x296 * x308 + x300;
	const GEN_FLT x310 = x296 * x309 + x301;
	const GEN_FLT x311 = sin(x289);
	const GEN_FLT x312 = x311 * (x296 * x310 + x306);
	const GEN_FLT x313 = x293 - x305 * x312;
	const GEN_FLT x314 = 1.0 / x313;
	const GEN_FLT x315 = x305 * x314;
	const GEN_FLT x316 = x302 * x315;
	const GEN_FLT x317 = x292 + x301 * x316;
	const GEN_FLT x318 = pow(-x317 * x317 + 1, -1.0 / 2.0);
	const GEN_FLT x319 = pow(-x94 / (x293 * x293) + 1, -1.0 / 2.0);
	const GEN_FLT x320 = x103 * x294;
	const GEN_FLT x321 = x319 * (x102 * x320 + x295 * x96);
	const GEN_FLT x322 = x298 * x321;
	const GEN_FLT x323 = x296 * (-x297 * x321 + x322) + x299 * x321;
	const GEN_FLT x324 = x296 * x323 + x300 * x321;
	const GEN_FLT x325 = pow(-x110 * x290 * x290 + 1, -1.0 / 2.0);
	const GEN_FLT x326 = x112 * x290;
	const GEN_FLT x327 = x101 * x326 + x291 * x96;
	const GEN_FLT x328 = x118 - x325 * x327;
	const GEN_FLT x329 = ogeeMag_1 * cos(x303);
	const GEN_FLT x330 = x312 * x329;
	const GEN_FLT x331 = 2.40324066e-5 * x296;
	const GEN_FLT x332 = x311 * (-curve_1 - x304);
	const GEN_FLT x333 = x301 * x302;
	const GEN_FLT x334 = x305 * x333 / ((x313 * x313));
	const GEN_FLT x335 = x314 * x329 * x333;
	const GEN_FLT x336 = 2 * x306 * x315;
	const GEN_FLT x337 =
		x118 - x318 * (x316 * x324 + x321 * x336 + x327 + x328 * x335 +
					   x334 * (x328 * x330 -
							   x332 * (x296 * x324 +
									   x296 * (x296 * (x296 * (x307 * x321 - x321 * x331 + x322) + x308 * x321 + x323) +
											   x309 * x321 + x324) +
									   x301 * x321 + x310 * x321)));
	const GEN_FLT x338 = gibMag_1 * cos(gibPhase_1 + x77 - asin(x317));
	const GEN_FLT x339 = x319 * (x133 * x320 + x295 * x37);
	const GEN_FLT x340 = x298 * x339;
	const GEN_FLT x341 = x296 * (-x297 * x339 + x340) + x299 * x339;
	const GEN_FLT x342 = x296 * x341 + x300 * x339;
	const GEN_FLT x343 = x132 * x326 + x291 * x37;
	const GEN_FLT x344 = x140 - x325 * x343;
	const GEN_FLT x345 =
		x140 - x318 * (x316 * x342 +
					   x334 * (x330 * x344 -
							   x332 * (x296 * x342 +
									   x296 * (x296 * (x296 * (x307 * x339 - x331 * x339 + x340) + x308 * x339 + x341) +
											   x309 * x339 + x342) +
									   x301 * x339 + x310 * x339)) +
					   x335 * x344 + x336 * x339 + x343);
	const GEN_FLT x346 = x319 * (x143 * x295 + x146 * x320);
	const GEN_FLT x347 = x298 * x346;
	const GEN_FLT x348 = x296 * (-x297 * x346 + x347) + x299 * x346;
	const GEN_FLT x349 = x296 * x348 + x300 * x346;
	const GEN_FLT x350 = x143 * x291 + x145 * x326;
	const GEN_FLT x351 = x153 - x325 * x350;
	const GEN_FLT x352 =
		x153 - x318 * (x316 * x349 +
					   x334 * (x330 * x351 -
							   x332 * (x296 * x349 +
									   x296 * (x296 * (x296 * (x307 * x346 - x331 * x346 + x347) + x308 * x346 + x348) +
											   x309 * x346 + x349) +
									   x301 * x346 + x310 * x346)) +
					   x335 * x351 + x336 * x346 + x350);
	const GEN_FLT x353 = x319 * (x181 * x295 + x188 * x320);
	const GEN_FLT x354 = x298 * x353;
	const GEN_FLT x355 = x296 * (-x297 * x353 + x354) + x299 * x353;
	const GEN_FLT x356 = x296 * x355 + x300 * x353;
	const GEN_FLT x357 = x181 * x291 + x187 * x326;
	const GEN_FLT x358 = x195 - x325 * x357;
	const GEN_FLT x359 =
		x195 - x318 * (x316 * x356 +
					   x334 * (x330 * x358 -
							   x332 * (x296 * x356 +
									   x296 * (x296 * (x296 * (x307 * x353 - x331 * x353 + x354) + x308 * x353 + x355) +
											   x309 * x353 + x356) +
									   x301 * x353 + x310 * x353)) +
					   x335 * x358 + x336 * x353 + x357);
	const GEN_FLT x360 = x319 * (x214 * x295 + x221 * x320);
	const GEN_FLT x361 = x298 * x360;
	const GEN_FLT x362 = x296 * (-x297 * x360 + x361) + x299 * x360;
	const GEN_FLT x363 = x296 * x362 + x300 * x360;
	const GEN_FLT x364 = x214 * x291 + x220 * x326;
	const GEN_FLT x365 = x228 - x325 * x364;
	const GEN_FLT x366 =
		x228 - x318 * (x316 * x363 +
					   x334 * (x330 * x365 -
							   x332 * (x296 * x363 +
									   x296 * (x296 * (x296 * (x307 * x360 - x331 * x360 + x361) + x308 * x360 + x362) +
											   x309 * x360 + x363) +
									   x301 * x360 + x310 * x360)) +
					   x335 * x365 + x336 * x360 + x364);
	const GEN_FLT x367 = x319 * (x244 * x295 + x251 * x320);
	const GEN_FLT x368 = x298 * x367;
	const GEN_FLT x369 = x296 * (-x297 * x367 + x368) + x299 * x367;
	const GEN_FLT x370 = x296 * x369 + x300 * x367;
	const GEN_FLT x371 = x244 * x291 + x250 * x326;
	const GEN_FLT x372 = x258 - x325 * x371;
	const GEN_FLT x373 =
		x258 - x318 * (x316 * x370 +
					   x334 * (x330 * x372 -
							   x332 * (x296 * x370 +
									   x296 * (x296 * (x296 * (x307 * x367 - x331 * x367 + x368) + x308 * x367 + x369) +
											   x309 * x367 + x370) +
									   x301 * x367 + x310 * x367)) +
					   x335 * x372 + x336 * x367 + x371);
	const GEN_FLT x374 = x319 * (x272 * x295 + x279 * x320);
	const GEN_FLT x375 = x298 * x374;
	const GEN_FLT x376 = x296 * (-x297 * x374 + x375) + x299 * x374;
	const GEN_FLT x377 = x296 * x376 + x300 * x374;
	const GEN_FLT x378 = x272 * x291 + x278 * x326;
	const GEN_FLT x379 = x286 - x325 * x378;
	const GEN_FLT x380 =
		x286 - x318 * (x316 * x377 +
					   x334 * (x330 * x379 -
							   x332 * (x296 * x377 +
									   x296 * (x296 * (x296 * (x307 * x374 - x331 * x374 + x375) + x308 * x374 + x376) +
											   x309 * x374 + x377) +
									   x301 * x374 + x310 * x374)) +
					   x335 * x379 + x336 * x374 + x378);
	*(out++) = x128 * x129 + x128;
	*(out++) = x129 * x142 + x142;
	*(out++) = x129 * x155 + x155;
	*(out++) = x129 * x197 + x197;
	*(out++) = x129 * x230 + x230;
	*(out++) = x129 * x260 + x260;
	*(out++) = x129 * x288 + x288;
	*(out++) = x337 * x338 + x337;
	*(out++) = x338 * x345 + x345;
	*(out++) = x338 * x352 + x352;
	*(out++) = x338 * x359 + x359;
	*(out++) = x338 * x366 + x366;
	*(out++) = x338 * x373 + x373;
	*(out++) = x338 * x380 + x380;
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
	const GEN_FLT x37 = -x23 * x36 + 1;
	const GEN_FLT x38 = x13 + x14;
	const GEN_FLT x39 = -x28 + x29;
	const GEN_FLT x40 = x6 + x8;
	const GEN_FLT x41 = obj_py + sensor_y * (-x11 * x40 + 1) + x12 * x39 + x32 * x38;
	const GEN_FLT x42 = lh_py + x2 * x24 + x27 * x35 + x37 * x41;
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
	const GEN_FLT x68 = asin(x42 * x67);
	const GEN_FLT x69 = 8.0108022e-6 * x68;
	const GEN_FLT x70 = -x69 - 8.0108022e-6;
	const GEN_FLT x71 = x68 * x70 + 0.0028679863;
	const GEN_FLT x72 = x68 * x71 + 5.3685255000000001e-6;
	const GEN_FLT x73 = x68 * x72 + 0.0076069798000000001;
	const GEN_FLT x74 = x68 * x68;
	const GEN_FLT x75 = atan2(x57, x49);
	const GEN_FLT x76 = ogeePhase_0 + x75 - asin(x62);
	const GEN_FLT x77 = ogeeMag_0 * sin(x76);
	const GEN_FLT x78 = curve_0 + x77;
	const GEN_FLT x79 = x68 * x73;
	const GEN_FLT x80 = -1.60216044e-5 * x68 - 8.0108022e-6;
	const GEN_FLT x81 = x68 * x80 + x71;
	const GEN_FLT x82 = x68 * x81 + x72;
	const GEN_FLT x83 = x68 * x82 + x73;
	const GEN_FLT x84 = sin(x59);
	const GEN_FLT x85 = x84 * (x68 * x83 + x79);
	const GEN_FLT x86 = x65 - x78 * x85;
	const GEN_FLT x87 = 1.0 / x86;
	const GEN_FLT x88 = x78 * x87;
	const GEN_FLT x89 = x74 * x88;
	const GEN_FLT x90 = x62 + x73 * x89;
	const GEN_FLT x91 = pow(-x90 * x90 + 1, -1.0 / 2.0);
	const GEN_FLT x92 = pow(-x63 / (x64 * (x65 * x65)) + 1, -1.0 / 2.0);
	const GEN_FLT x93 = x2 * x23;
	const GEN_FLT x94 = x23 * x42;
	const GEN_FLT x95 = 4 * x22;
	const GEN_FLT x96 = (1.0 / 2.0) * x49;
	const GEN_FLT x97 = x23 * x57;
	const GEN_FLT x98 = x52 * x97 - x96 * (-x21 * x95 + 2);
	const GEN_FLT x99 = x42 * x66 / pow(x64, 3.0 / 2.0);
	const GEN_FLT x100 = x92 * (x67 * x93 + x99 * (-x2 * x94 + x98));
	const GEN_FLT x101 = x100 * x70;
	const GEN_FLT x102 = x100 * x71 + x68 * (-x100 * x69 + x101);
	const GEN_FLT x103 = x100 * x72 + x102 * x68;
	const GEN_FLT x104 = 1.0 / x58;
	const GEN_FLT x105 = pow(-x104 * x63 * x60 * x60 + 1, -1.0 / 2.0);
	const GEN_FLT x106 = x42 * x60 / pow(x58, 3.0 / 2.0);
	const GEN_FLT x107 = x106 * x98 + x61 * x93;
	const GEN_FLT x108 = x104 * (lh_pz + x51 + x53 + x56);
	const GEN_FLT x109 = x104 * x49;
	const GEN_FLT x110 = x109 * x23;
	const GEN_FLT x111 = x108 * x48 - x110 * x52;
	const GEN_FLT x112 = -x105 * x107 + x111;
	const GEN_FLT x113 = ogeeMag_0 * cos(x76);
	const GEN_FLT x114 = x113 * x85;
	const GEN_FLT x115 = 2.40324066e-5 * x68;
	const GEN_FLT x116 = x84 * (-curve_0 - x77);
	const GEN_FLT x117 = x73 * x74;
	const GEN_FLT x118 = x117 * x78 / ((x86 * x86));
	const GEN_FLT x119 = x113 * x117 * x87;
	const GEN_FLT x120 = 2 * x79 * x88;
	const GEN_FLT x121 =
		x111 - x91 * (x100 * x120 + x103 * x89 + x107 + x112 * x119 +
					  x118 * (x112 * x114 -
							  x116 * (x100 * x73 + x100 * x83 + x103 * x68 +
									  x68 * (x100 * x82 + x103 +
											 x68 * (x100 * x81 + x102 + x68 * (-x100 * x115 + x100 * x80 + x101))))));
	const GEN_FLT x122 = gibMag_0 * cos(gibPhase_0 + x75 - asin(x90));
	const GEN_FLT x123 = (1.0 / 2.0) * x42;
	const GEN_FLT x124 = x23 * x49;
	const GEN_FLT x125 = -x124 * x46 + x50 * x97;
	const GEN_FLT x126 = x92 * (x37 * x67 + x99 * (-x123 * (-x36 * x95 + 2) + x125));
	const GEN_FLT x127 = x126 * x70;
	const GEN_FLT x128 = x126 * x71 + x68 * (-x126 * x69 + x127);
	const GEN_FLT x129 = x126 * x72 + x128 * x68;
	const GEN_FLT x130 = x106 * x125 + x37 * x61;
	const GEN_FLT x131 = x108 * x23;
	const GEN_FLT x132 = -x110 * x50 + x131 * x46;
	const GEN_FLT x133 = -x105 * x130 + x132;
	const GEN_FLT x134 =
		x132 - x91 * (x118 * (x114 * x133 -
							  x116 * (x126 * x73 + x126 * x83 + x129 * x68 +
									  x68 * (x126 * x82 + x129 +
											 x68 * (x126 * x81 + x128 + x68 * (-x115 * x126 + x126 * x80 + x127))))) +
					  x119 * x133 + x120 * x126 + x129 * x89 + x130);
	const GEN_FLT x135 = x23 * x27;
	const GEN_FLT x136 = (1.0 / 2.0) * x57;
	const GEN_FLT x137 = -x124 * x45 - x136 * (x54 * x95 - 2);
	const GEN_FLT x138 = x92 * (x135 * x67 + x99 * (x137 - x27 * x94));
	const GEN_FLT x139 = x138 * x70;
	const GEN_FLT x140 = x138 * x71 + x68 * (-x138 * x69 + x139);
	const GEN_FLT x141 = x138 * x72 + x140 * x68;
	const GEN_FLT x142 = x106 * x137 + x135 * x61;
	const GEN_FLT x143 = x55 - 1;
	const GEN_FLT x144 = x109 * x143 + x131 * x45;
	const GEN_FLT x145 = -x105 * x142 + x144;
	const GEN_FLT x146 =
		x144 - x91 * (x118 * (x114 * x145 -
							  x116 * (x138 * x73 + x138 * x83 + x141 * x68 +
									  x68 * (x138 * x82 + x141 +
											 x68 * (x138 * x81 + x140 + x68 * (-x115 * x138 + x138 * x80 + x139))))) +
					  x119 * x145 + x120 * x138 + x141 * x89 + x142);
	const GEN_FLT x147 = obj_qj * x12;
	const GEN_FLT x148 = obj_qk * x16;
	const GEN_FLT x149 = 1.0 / x10;
	const GEN_FLT x150 = obj_qw * x149;
	const GEN_FLT x151 = 2 * x150;
	const GEN_FLT x152 = sensor_z * x151;
	const GEN_FLT x153 = sensor_y * x151;
	const GEN_FLT x154 = -2 * x7;
	const GEN_FLT x155 = -2 * x8;
	const GEN_FLT x156 = sensor_x * (x154 + x155);
	const GEN_FLT x157 = x147 - x148 + x15 * x153 + x150 * x156 + x152 * x5;
	const GEN_FLT x158 = x157 * x2;
	const GEN_FLT x159 = obj_qi * x16;
	const GEN_FLT x160 = obj_qj * x32;
	const GEN_FLT x161 = sensor_x * x31;
	const GEN_FLT x162 = -2 * x6;
	const GEN_FLT x163 = sensor_z * (x154 + x162);
	const GEN_FLT x164 = x150 * x163 + x151 * x161 + x153 * x30 + x159 - x160;
	const GEN_FLT x165 = x164 * x27;
	const GEN_FLT x166 = obj_qi * x12;
	const GEN_FLT x167 = obj_qk * x32;
	const GEN_FLT x168 = sensor_x * x38;
	const GEN_FLT x169 = sensor_y * (x155 + x162);
	const GEN_FLT x170 = x150 * x169 + x151 * x168 + x152 * x39 - x166 + x167;
	const GEN_FLT x171 = x170 * x37;
	const GEN_FLT x172 = x158 * x23 + x165 * x23 + x171;
	const GEN_FLT x173 = x164 * x45;
	const GEN_FLT x174 = x170 * x95;
	const GEN_FLT x175 = x157 * x48;
	const GEN_FLT x176 = x157 * x52;
	const GEN_FLT x177 = x143 * x164;
	const GEN_FLT x178 = -x136 * (-x174 * x50 - x176 * x95 + 2 * x177) - x96 * (x173 * x95 + x174 * x46 + 2 * x175);
	const GEN_FLT x179 = x92 * (x172 * x67 + x99 * (-x123 * (x158 * x95 + x165 * x95 + 2 * x171) + x178));
	const GEN_FLT x180 = x179 * x70;
	const GEN_FLT x181 = x179 * x71 + x68 * (-x179 * x69 + x180);
	const GEN_FLT x182 = x179 * x72 + x181 * x68;
	const GEN_FLT x183 = x106 * x178 + x172 * x61;
	const GEN_FLT x184 = x170 * x23;
	const GEN_FLT x185 = x108 * (x173 * x23 + x175 + x184 * x46) + x109 * (-x176 * x23 + x177 - x184 * x50);
	const GEN_FLT x186 = -x105 * x183 + x185;
	const GEN_FLT x187 =
		x185 - x91 * (x118 * (x114 * x186 -
							  x116 * (x179 * x73 + x179 * x83 + x182 * x68 +
									  x68 * (x179 * x82 + x182 +
											 x68 * (x179 * x81 + x181 + x68 * (-x115 * x179 + x179 * x80 + x180))))) +
					  x119 * x186 + x120 * x179 + x182 * x89 + x183);
	const GEN_FLT x188 = obj_qj * x16;
	const GEN_FLT x189 = obj_qk * x12;
	const GEN_FLT x190 = obj_qi * x149;
	const GEN_FLT x191 = 2 * x190;
	const GEN_FLT x192 = sensor_z * x191;
	const GEN_FLT x193 = sensor_y * x191;
	const GEN_FLT x194 = x15 * x193 + x156 * x190 + x188 + x189 + x192 * x5;
	const GEN_FLT x195 = x194 * x2;
	const GEN_FLT x196 = obj_qw * x16;
	const GEN_FLT x197 = 4 * x10;
	const GEN_FLT x198 = -obj_qi * x197;
	const GEN_FLT x199 = sensor_z * (-x191 * x33 + x198) + x161 * x191 + x167 + x193 * x30 + x196;
	const GEN_FLT x200 = x199 * x27;
	const GEN_FLT x201 = obj_qw * x12;
	const GEN_FLT x202 = sensor_y * (-x191 * x40 + x198) + x160 + x168 * x191 + x192 * x39 - x201;
	const GEN_FLT x203 = x202 * x37;
	const GEN_FLT x204 = x195 * x23 + x200 * x23 + x203;
	const GEN_FLT x205 = x194 * x48;
	const GEN_FLT x206 = x199 * x45;
	const GEN_FLT x207 = x202 * x95;
	const GEN_FLT x208 = x194 * x52;
	const GEN_FLT x209 = x143 * x199;
	const GEN_FLT x210 = -x136 * (-x207 * x50 - x208 * x95 + 2 * x209) - x96 * (2 * x205 + x206 * x95 + x207 * x46);
	const GEN_FLT x211 = x92 * (x204 * x67 + x99 * (-x123 * (x195 * x95 + x200 * x95 + 2 * x203) + x210));
	const GEN_FLT x212 = x211 * x70;
	const GEN_FLT x213 = x211 * x71 + x68 * (-x211 * x69 + x212);
	const GEN_FLT x214 = x211 * x72 + x213 * x68;
	const GEN_FLT x215 = x106 * x210 + x204 * x61;
	const GEN_FLT x216 = x202 * x23;
	const GEN_FLT x217 = x108 * (x205 + x206 * x23 + x216 * x46) + x109 * (-x208 * x23 + x209 - x216 * x50);
	const GEN_FLT x218 = -x105 * x215 + x217;
	const GEN_FLT x219 =
		x217 - x91 * (x118 * (x114 * x218 -
							  x116 * (x211 * x73 + x211 * x83 + x214 * x68 +
									  x68 * (x211 * x82 + x214 +
											 x68 * (x211 * x81 + x213 + x68 * (-x115 * x211 + x211 * x80 + x212))))) +
					  x119 * x218 + x120 * x211 + x214 * x89 + x215);
	const GEN_FLT x220 = obj_qi * x32;
	const GEN_FLT x221 = obj_qj * x149;
	const GEN_FLT x222 = 2 * x221;
	const GEN_FLT x223 = sensor_z * x222;
	const GEN_FLT x224 = x168 * x222 + x169 * x221 + x189 + x220 + x223 * x39;
	const GEN_FLT x225 = x224 * x37;
	const GEN_FLT x226 = sensor_y * x222;
	const GEN_FLT x227 = -obj_qj * x197;
	const GEN_FLT x228 = sensor_x * (-x222 * x9 + x227) + x15 * x226 + x159 + x201 + x223 * x5;
	const GEN_FLT x229 = x2 * x228;
	const GEN_FLT x230 = obj_qw * x32;
	const GEN_FLT x231 = sensor_z * (-x222 * x33 + x227) + x148 + x161 * x222 + x226 * x30 - x230;
	const GEN_FLT x232 = x231 * x27;
	const GEN_FLT x233 = x225 + x229 * x23 + x23 * x232;
	const GEN_FLT x234 = x224 * x95;
	const GEN_FLT x235 = x231 * x45;
	const GEN_FLT x236 = x228 * x48;
	const GEN_FLT x237 = x228 * x52;
	const GEN_FLT x238 = x143 * x231;
	const GEN_FLT x239 = -x136 * (-x234 * x50 - x237 * x95 + 2 * x238) - x96 * (x234 * x46 + x235 * x95 + 2 * x236);
	const GEN_FLT x240 = x92 * (x233 * x67 + x99 * (-x123 * (2 * x225 + x229 * x95 + x232 * x95) + x239));
	const GEN_FLT x241 = x240 * x70;
	const GEN_FLT x242 = x240 * x71 + x68 * (-x240 * x69 + x241);
	const GEN_FLT x243 = x240 * x72 + x242 * x68;
	const GEN_FLT x244 = x106 * x239 + x233 * x61;
	const GEN_FLT x245 = x224 * x23;
	const GEN_FLT x246 = x108 * (x23 * x235 + x236 + x245 * x46) + x109 * (-x23 * x237 + x238 - x245 * x50);
	const GEN_FLT x247 = -x105 * x244 + x246;
	const GEN_FLT x248 =
		x246 - x91 * (x118 * (x114 * x247 -
							  x116 * (x240 * x73 + x240 * x83 + x243 * x68 +
									  x68 * (x240 * x82 + x243 +
											 x68 * (x240 * x81 + x242 + x68 * (-x115 * x240 + x240 * x80 + x241))))) +
					  x119 * x247 + x120 * x240 + x243 * x89 + x244);
	const GEN_FLT x249 = obj_qk * x149;
	const GEN_FLT x250 = 2 * x249;
	const GEN_FLT x251 = sensor_y * x250;
	const GEN_FLT x252 = x161 * x250 + x163 * x249 + x188 + x220 + x251 * x30;
	const GEN_FLT x253 = x252 * x27;
	const GEN_FLT x254 = sensor_z * x250;
	const GEN_FLT x255 = -obj_qk * x197;
	const GEN_FLT x256 = sensor_x * (-x250 * x9 + x255) + x15 * x251 + x166 - x196 + x254 * x5;
	const GEN_FLT x257 = x2 * x256;
	const GEN_FLT x258 = sensor_y * (-x250 * x40 + x255) + x147 + x168 * x250 + x230 + x254 * x39;
	const GEN_FLT x259 = x258 * x37;
	const GEN_FLT x260 = x23 * x253 + x23 * x257 + x259;
	const GEN_FLT x261 = x252 * x45;
	const GEN_FLT x262 = x258 * x95;
	const GEN_FLT x263 = x256 * x48;
	const GEN_FLT x264 = x143 * x252;
	const GEN_FLT x265 = x256 * x52;
	const GEN_FLT x266 = -x136 * (-x262 * x50 + 2 * x264 - x265 * x95) - x96 * (x261 * x95 + x262 * x46 + 2 * x263);
	const GEN_FLT x267 = x92 * (x260 * x67 + x99 * (-x123 * (x253 * x95 + x257 * x95 + 2 * x259) + x266));
	const GEN_FLT x268 = x267 * x70;
	const GEN_FLT x269 = x267 * x71 + x68 * (-x267 * x69 + x268);
	const GEN_FLT x270 = x267 * x72 + x269 * x68;
	const GEN_FLT x271 = x106 * x266 + x260 * x61;
	const GEN_FLT x272 = x23 * x258;
	const GEN_FLT x273 = x108 * (x23 * x261 + x263 + x272 * x46) + x109 * (-x23 * x265 + x264 - x272 * x50);
	const GEN_FLT x274 = -x105 * x271 + x273;
	const GEN_FLT x275 =
		x273 - x91 * (x118 * (x114 * x274 -
							  x116 * (x267 * x73 + x267 * x83 + x270 * x68 +
									  x68 * (x267 * x82 + x270 +
											 x68 * (x267 * x81 + x269 + x68 * (-x115 * x267 + x267 * x80 + x268))))) +
					  x119 * x274 + x120 * x267 + x270 * x89 + x271);
	*(out++) = x121 * x122 + x121;
	*(out++) = x122 * x134 + x134;
	*(out++) = x122 * x146 + x146;
	*(out++) = x122 * x187 + x187;
	*(out++) = x122 * x219 + x219;
	*(out++) = x122 * x248 + x248;
	*(out++) = x122 * x275 + x275;
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
	const GEN_FLT x37 = -x23 * x36 + 1;
	const GEN_FLT x38 = x13 + x14;
	const GEN_FLT x39 = -x28 + x29;
	const GEN_FLT x40 = x6 + x8;
	const GEN_FLT x41 = obj_py + sensor_y * (-x11 * x40 + 1) + x12 * x39 + x32 * x38;
	const GEN_FLT x42 = lh_py + x2 * x24 + x27 * x35 + x37 * x41;
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
	const GEN_FLT x68 = asin(x42 * x67);
	const GEN_FLT x69 = 8.0108022e-6 * x68;
	const GEN_FLT x70 = -x69 - 8.0108022e-6;
	const GEN_FLT x71 = x68 * x70 + 0.0028679863;
	const GEN_FLT x72 = x68 * x71 + 5.3685255000000001e-6;
	const GEN_FLT x73 = x68 * x72 + 0.0076069798000000001;
	const GEN_FLT x74 = x68 * x68;
	const GEN_FLT x75 = atan2(x57, x49);
	const GEN_FLT x76 = ogeePhase_0 + x75 - asin(x62);
	const GEN_FLT x77 = ogeeMag_0 * sin(x76);
	const GEN_FLT x78 = curve_0 + x77;
	const GEN_FLT x79 = x68 * x73;
	const GEN_FLT x80 = -1.60216044e-5 * x68 - 8.0108022e-6;
	const GEN_FLT x81 = x68 * x80 + x71;
	const GEN_FLT x82 = x68 * x81 + x72;
	const GEN_FLT x83 = x68 * x82 + x73;
	const GEN_FLT x84 = sin(x59);
	const GEN_FLT x85 = x84 * (x68 * x83 + x79);
	const GEN_FLT x86 = x65 - x78 * x85;
	const GEN_FLT x87 = 1.0 / x86;
	const GEN_FLT x88 = x78 * x87;
	const GEN_FLT x89 = x74 * x88;
	const GEN_FLT x90 = x62 + x73 * x89;
	const GEN_FLT x91 = pow(-x90 * x90 + 1, -1.0 / 2.0);
	const GEN_FLT x92 = pow(-x63 / (x64 * (x65 * x65)) + 1, -1.0 / 2.0);
	const GEN_FLT x93 = x2 * x23;
	const GEN_FLT x94 = x23 * x42;
	const GEN_FLT x95 = 4 * x22;
	const GEN_FLT x96 = (1.0 / 2.0) * x49;
	const GEN_FLT x97 = x23 * x57;
	const GEN_FLT x98 = x52 * x97 - x96 * (-x21 * x95 + 2);
	const GEN_FLT x99 = x42 * x66 / pow(x64, 3.0 / 2.0);
	const GEN_FLT x100 = x92 * (x67 * x93 + x99 * (-x2 * x94 + x98));
	const GEN_FLT x101 = x100 * x70;
	const GEN_FLT x102 = x100 * x71 + x68 * (-x100 * x69 + x101);
	const GEN_FLT x103 = x100 * x72 + x102 * x68;
	const GEN_FLT x104 = 1.0 / x58;
	const GEN_FLT x105 = pow(-x104 * x63 * x60 * x60 + 1, -1.0 / 2.0);
	const GEN_FLT x106 = x42 * x60 / pow(x58, 3.0 / 2.0);
	const GEN_FLT x107 = x106 * x98 + x61 * x93;
	const GEN_FLT x108 = x104 * (lh_pz + x51 + x53 + x56);
	const GEN_FLT x109 = x104 * x49;
	const GEN_FLT x110 = x109 * x23;
	const GEN_FLT x111 = x108 * x48 - x110 * x52;
	const GEN_FLT x112 = -x105 * x107 + x111;
	const GEN_FLT x113 = ogeeMag_0 * cos(x76);
	const GEN_FLT x114 = x113 * x85;
	const GEN_FLT x115 = 2.40324066e-5 * x68;
	const GEN_FLT x116 = x84 * (-curve_0 - x77);
	const GEN_FLT x117 = x73 * x74;
	const GEN_FLT x118 = x117 * x78 / ((x86 * x86));
	const GEN_FLT x119 = x113 * x117 * x87;
	const GEN_FLT x120 = 2 * x79 * x88;
	const GEN_FLT x121 =
		x111 - x91 * (x100 * x120 + x103 * x89 + x107 + x112 * x119 +
					  x118 * (x112 * x114 -
							  x116 * (x100 * x73 + x100 * x83 + x103 * x68 +
									  x68 * (x100 * x82 + x103 +
											 x68 * (x100 * x81 + x102 + x68 * (-x100 * x115 + x100 * x80 + x101))))));
	const GEN_FLT x122 = gibMag_0 * cos(gibPhase_0 + x75 - asin(x90));
	const GEN_FLT x123 = (1.0 / 2.0) * x42;
	const GEN_FLT x124 = x23 * x49;
	const GEN_FLT x125 = -x124 * x46 + x50 * x97;
	const GEN_FLT x126 = x92 * (x37 * x67 + x99 * (-x123 * (-x36 * x95 + 2) + x125));
	const GEN_FLT x127 = x126 * x70;
	const GEN_FLT x128 = x126 * x71 + x68 * (-x126 * x69 + x127);
	const GEN_FLT x129 = x126 * x72 + x128 * x68;
	const GEN_FLT x130 = x106 * x125 + x37 * x61;
	const GEN_FLT x131 = x108 * x23;
	const GEN_FLT x132 = -x110 * x50 + x131 * x46;
	const GEN_FLT x133 = -x105 * x130 + x132;
	const GEN_FLT x134 =
		x132 - x91 * (x118 * (x114 * x133 -
							  x116 * (x126 * x73 + x126 * x83 + x129 * x68 +
									  x68 * (x126 * x82 + x129 +
											 x68 * (x126 * x81 + x128 + x68 * (-x115 * x126 + x126 * x80 + x127))))) +
					  x119 * x133 + x120 * x126 + x129 * x89 + x130);
	const GEN_FLT x135 = x23 * x27;
	const GEN_FLT x136 = (1.0 / 2.0) * x57;
	const GEN_FLT x137 = -x124 * x45 - x136 * (x54 * x95 - 2);
	const GEN_FLT x138 = x92 * (x135 * x67 + x99 * (x137 - x27 * x94));
	const GEN_FLT x139 = x138 * x70;
	const GEN_FLT x140 = x138 * x71 + x68 * (-x138 * x69 + x139);
	const GEN_FLT x141 = x138 * x72 + x140 * x68;
	const GEN_FLT x142 = x106 * x137 + x135 * x61;
	const GEN_FLT x143 = x55 - 1;
	const GEN_FLT x144 = x109 * x143 + x131 * x45;
	const GEN_FLT x145 = -x105 * x142 + x144;
	const GEN_FLT x146 =
		x144 - x91 * (x118 * (x114 * x145 -
							  x116 * (x138 * x73 + x138 * x83 + x141 * x68 +
									  x68 * (x138 * x82 + x141 +
											 x68 * (x138 * x81 + x140 + x68 * (-x115 * x138 + x138 * x80 + x139))))) +
					  x119 * x145 + x120 * x138 + x141 * x89 + x142);
	const GEN_FLT x147 = obj_qj * x12;
	const GEN_FLT x148 = obj_qk * x16;
	const GEN_FLT x149 = 1.0 / x10;
	const GEN_FLT x150 = obj_qw * x149;
	const GEN_FLT x151 = 2 * x150;
	const GEN_FLT x152 = sensor_z * x151;
	const GEN_FLT x153 = sensor_y * x151;
	const GEN_FLT x154 = -2 * x7;
	const GEN_FLT x155 = -2 * x8;
	const GEN_FLT x156 = sensor_x * (x154 + x155);
	const GEN_FLT x157 = x147 - x148 + x15 * x153 + x150 * x156 + x152 * x5;
	const GEN_FLT x158 = x157 * x2;
	const GEN_FLT x159 = obj_qi * x16;
	const GEN_FLT x160 = obj_qj * x32;
	const GEN_FLT x161 = sensor_x * x31;
	const GEN_FLT x162 = -2 * x6;
	const GEN_FLT x163 = sensor_z * (x154 + x162);
	const GEN_FLT x164 = x150 * x163 + x151 * x161 + x153 * x30 + x159 - x160;
	const GEN_FLT x165 = x164 * x27;
	const GEN_FLT x166 = obj_qi * x12;
	const GEN_FLT x167 = obj_qk * x32;
	const GEN_FLT x168 = sensor_x * x38;
	const GEN_FLT x169 = sensor_y * (x155 + x162);
	const GEN_FLT x170 = x150 * x169 + x151 * x168 + x152 * x39 - x166 + x167;
	const GEN_FLT x171 = x170 * x37;
	const GEN_FLT x172 = x158 * x23 + x165 * x23 + x171;
	const GEN_FLT x173 = x164 * x45;
	const GEN_FLT x174 = x170 * x95;
	const GEN_FLT x175 = x157 * x48;
	const GEN_FLT x176 = x157 * x52;
	const GEN_FLT x177 = x143 * x164;
	const GEN_FLT x178 = -x136 * (-x174 * x50 - x176 * x95 + 2 * x177) - x96 * (x173 * x95 + x174 * x46 + 2 * x175);
	const GEN_FLT x179 = x92 * (x172 * x67 + x99 * (-x123 * (x158 * x95 + x165 * x95 + 2 * x171) + x178));
	const GEN_FLT x180 = x179 * x70;
	const GEN_FLT x181 = x179 * x71 + x68 * (-x179 * x69 + x180);
	const GEN_FLT x182 = x179 * x72 + x181 * x68;
	const GEN_FLT x183 = x106 * x178 + x172 * x61;
	const GEN_FLT x184 = x170 * x23;
	const GEN_FLT x185 = x108 * (x173 * x23 + x175 + x184 * x46) + x109 * (-x176 * x23 + x177 - x184 * x50);
	const GEN_FLT x186 = -x105 * x183 + x185;
	const GEN_FLT x187 =
		x185 - x91 * (x118 * (x114 * x186 -
							  x116 * (x179 * x73 + x179 * x83 + x182 * x68 +
									  x68 * (x179 * x82 + x182 +
											 x68 * (x179 * x81 + x181 + x68 * (-x115 * x179 + x179 * x80 + x180))))) +
					  x119 * x186 + x120 * x179 + x182 * x89 + x183);
	const GEN_FLT x188 = obj_qj * x16;
	const GEN_FLT x189 = obj_qk * x12;
	const GEN_FLT x190 = obj_qi * x149;
	const GEN_FLT x191 = 2 * x190;
	const GEN_FLT x192 = sensor_z * x191;
	const GEN_FLT x193 = sensor_y * x191;
	const GEN_FLT x194 = x15 * x193 + x156 * x190 + x188 + x189 + x192 * x5;
	const GEN_FLT x195 = x194 * x2;
	const GEN_FLT x196 = obj_qw * x16;
	const GEN_FLT x197 = 4 * x10;
	const GEN_FLT x198 = -obj_qi * x197;
	const GEN_FLT x199 = sensor_z * (-x191 * x33 + x198) + x161 * x191 + x167 + x193 * x30 + x196;
	const GEN_FLT x200 = x199 * x27;
	const GEN_FLT x201 = obj_qw * x12;
	const GEN_FLT x202 = sensor_y * (-x191 * x40 + x198) + x160 + x168 * x191 + x192 * x39 - x201;
	const GEN_FLT x203 = x202 * x37;
	const GEN_FLT x204 = x195 * x23 + x200 * x23 + x203;
	const GEN_FLT x205 = x194 * x48;
	const GEN_FLT x206 = x199 * x45;
	const GEN_FLT x207 = x202 * x95;
	const GEN_FLT x208 = x194 * x52;
	const GEN_FLT x209 = x143 * x199;
	const GEN_FLT x210 = -x136 * (-x207 * x50 - x208 * x95 + 2 * x209) - x96 * (2 * x205 + x206 * x95 + x207 * x46);
	const GEN_FLT x211 = x92 * (x204 * x67 + x99 * (-x123 * (x195 * x95 + x200 * x95 + 2 * x203) + x210));
	const GEN_FLT x212 = x211 * x70;
	const GEN_FLT x213 = x211 * x71 + x68 * (-x211 * x69 + x212);
	const GEN_FLT x214 = x211 * x72 + x213 * x68;
	const GEN_FLT x215 = x106 * x210 + x204 * x61;
	const GEN_FLT x216 = x202 * x23;
	const GEN_FLT x217 = x108 * (x205 + x206 * x23 + x216 * x46) + x109 * (-x208 * x23 + x209 - x216 * x50);
	const GEN_FLT x218 = -x105 * x215 + x217;
	const GEN_FLT x219 =
		x217 - x91 * (x118 * (x114 * x218 -
							  x116 * (x211 * x73 + x211 * x83 + x214 * x68 +
									  x68 * (x211 * x82 + x214 +
											 x68 * (x211 * x81 + x213 + x68 * (-x115 * x211 + x211 * x80 + x212))))) +
					  x119 * x218 + x120 * x211 + x214 * x89 + x215);
	const GEN_FLT x220 = obj_qi * x32;
	const GEN_FLT x221 = obj_qj * x149;
	const GEN_FLT x222 = 2 * x221;
	const GEN_FLT x223 = sensor_z * x222;
	const GEN_FLT x224 = x168 * x222 + x169 * x221 + x189 + x220 + x223 * x39;
	const GEN_FLT x225 = x224 * x37;
	const GEN_FLT x226 = sensor_y * x222;
	const GEN_FLT x227 = -obj_qj * x197;
	const GEN_FLT x228 = sensor_x * (-x222 * x9 + x227) + x15 * x226 + x159 + x201 + x223 * x5;
	const GEN_FLT x229 = x2 * x228;
	const GEN_FLT x230 = obj_qw * x32;
	const GEN_FLT x231 = sensor_z * (-x222 * x33 + x227) + x148 + x161 * x222 + x226 * x30 - x230;
	const GEN_FLT x232 = x231 * x27;
	const GEN_FLT x233 = x225 + x229 * x23 + x23 * x232;
	const GEN_FLT x234 = x224 * x95;
	const GEN_FLT x235 = x231 * x45;
	const GEN_FLT x236 = x228 * x48;
	const GEN_FLT x237 = x228 * x52;
	const GEN_FLT x238 = x143 * x231;
	const GEN_FLT x239 = -x136 * (-x234 * x50 - x237 * x95 + 2 * x238) - x96 * (x234 * x46 + x235 * x95 + 2 * x236);
	const GEN_FLT x240 = x92 * (x233 * x67 + x99 * (-x123 * (2 * x225 + x229 * x95 + x232 * x95) + x239));
	const GEN_FLT x241 = x240 * x70;
	const GEN_FLT x242 = x240 * x71 + x68 * (-x240 * x69 + x241);
	const GEN_FLT x243 = x240 * x72 + x242 * x68;
	const GEN_FLT x244 = x106 * x239 + x233 * x61;
	const GEN_FLT x245 = x224 * x23;
	const GEN_FLT x246 = x108 * (x23 * x235 + x236 + x245 * x46) + x109 * (-x23 * x237 + x238 - x245 * x50);
	const GEN_FLT x247 = -x105 * x244 + x246;
	const GEN_FLT x248 =
		x246 - x91 * (x118 * (x114 * x247 -
							  x116 * (x240 * x73 + x240 * x83 + x243 * x68 +
									  x68 * (x240 * x82 + x243 +
											 x68 * (x240 * x81 + x242 + x68 * (-x115 * x240 + x240 * x80 + x241))))) +
					  x119 * x247 + x120 * x240 + x243 * x89 + x244);
	const GEN_FLT x249 = obj_qk * x149;
	const GEN_FLT x250 = 2 * x249;
	const GEN_FLT x251 = sensor_y * x250;
	const GEN_FLT x252 = x161 * x250 + x163 * x249 + x188 + x220 + x251 * x30;
	const GEN_FLT x253 = x252 * x27;
	const GEN_FLT x254 = sensor_z * x250;
	const GEN_FLT x255 = -obj_qk * x197;
	const GEN_FLT x256 = sensor_x * (-x250 * x9 + x255) + x15 * x251 + x166 - x196 + x254 * x5;
	const GEN_FLT x257 = x2 * x256;
	const GEN_FLT x258 = sensor_y * (-x250 * x40 + x255) + x147 + x168 * x250 + x230 + x254 * x39;
	const GEN_FLT x259 = x258 * x37;
	const GEN_FLT x260 = x23 * x253 + x23 * x257 + x259;
	const GEN_FLT x261 = x252 * x45;
	const GEN_FLT x262 = x258 * x95;
	const GEN_FLT x263 = x256 * x48;
	const GEN_FLT x264 = x143 * x252;
	const GEN_FLT x265 = x256 * x52;
	const GEN_FLT x266 = -x136 * (-x262 * x50 + 2 * x264 - x265 * x95) - x96 * (x261 * x95 + x262 * x46 + 2 * x263);
	const GEN_FLT x267 = x92 * (x260 * x67 + x99 * (-x123 * (x253 * x95 + x257 * x95 + 2 * x259) + x266));
	const GEN_FLT x268 = x267 * x70;
	const GEN_FLT x269 = x267 * x71 + x68 * (-x267 * x69 + x268);
	const GEN_FLT x270 = x267 * x72 + x269 * x68;
	const GEN_FLT x271 = x106 * x266 + x260 * x61;
	const GEN_FLT x272 = x23 * x258;
	const GEN_FLT x273 = x108 * (x23 * x261 + x263 + x272 * x46) + x109 * (-x23 * x265 + x264 - x272 * x50);
	const GEN_FLT x274 = -x105 * x271 + x273;
	const GEN_FLT x275 =
		x273 - x91 * (x118 * (x114 * x274 -
							  x116 * (x267 * x73 + x267 * x83 + x270 * x68 +
									  x68 * (x267 * x82 + x270 +
											 x68 * (x267 * x81 + x269 + x68 * (-x115 * x267 + x267 * x80 + x268))))) +
					  x119 * x274 + x120 * x267 + x270 * x89 + x271);
	*(out++) = x121 * x122 + x121;
	*(out++) = x122 * x134 + x134;
	*(out++) = x122 * x146 + x146;
	*(out++) = x122 * x187 + x187;
	*(out++) = x122 * x219 + x219;
	*(out++) = x122 * x248 + x248;
	*(out++) = x122 * x275 + x275;
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

