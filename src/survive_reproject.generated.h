// NOTE: Auto-generated code; see tools/generate_reprojection_functions
static inline void gen_reproject_jac_all_gen2(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
											  const FLT phase_0, const FLT phase_1, const FLT tilt_0, const FLT tilt_1,
											  const FLT curve_0, const FLT curve_1, const FLT gibPhase_0,
											  const FLT gibPhase_1, const FLT gibMag_0, const FLT gibMag_1,
											  const FLT ogeePhase_0, const FLT ogeePhase_1, const FLT ogeeMag_0,
											  const FLT ogeeMag_1) {
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
	const GEN_FLT x25 = x2 * x24;
	const GEN_FLT x26 = lh_qj * lh_qk;
	const GEN_FLT x27 = lh_qi * lh_qw;
	const GEN_FLT x28 = x26 - x27;
	const GEN_FLT x29 = obj_qi * obj_qw;
	const GEN_FLT x30 = obj_qj * obj_qk;
	const GEN_FLT x31 = x29 + x30;
	const GEN_FLT x32 = x3 - x4;
	const GEN_FLT x33 = sensor_x * x11;
	const GEN_FLT x34 = x6 + x7;
	const GEN_FLT x35 = obj_pz + sensor_z * (-x11 * x34 + 1) + x16 * x31 + x32 * x33;
	const GEN_FLT x36 = x23 * x35;
	const GEN_FLT x37 = x28 * x36;
	const GEN_FLT x38 = x18 + x20;
	const GEN_FLT x39 = -x23 * x38 + 1;
	const GEN_FLT x40 = x13 + x14;
	const GEN_FLT x41 = -x29 + x30;
	const GEN_FLT x42 = x6 + x8;
	const GEN_FLT x43 = obj_py + sensor_y * (-x11 * x42 + 1) + x12 * x41 + x33 * x40;
	const GEN_FLT x44 = x39 * x43;
	const GEN_FLT x45 = lh_py + x25 + x37 + x44;
	const GEN_FLT x46 = tilt_0 + 0.52359877559829882;
	const GEN_FLT x47 = tan(x46);
	const GEN_FLT x48 = lh_qi * lh_qk;
	const GEN_FLT x49 = lh_qj * lh_qw;
	const GEN_FLT x50 = x48 + x49;
	const GEN_FLT x51 = x36 * x50;
	const GEN_FLT x52 = x0 - x1;
	const GEN_FLT x53 = x23 * x43;
	const GEN_FLT x54 = x52 * x53;
	const GEN_FLT x55 = -x21 * x23 + 1;
	const GEN_FLT x56 = x17 * x55;
	const GEN_FLT x57 = lh_px + x51 + x54 + x56;
	const GEN_FLT x58 = x26 + x27;
	const GEN_FLT x59 = x53 * x58;
	const GEN_FLT x60 = x48 - x49;
	const GEN_FLT x61 = x24 * x60;
	const GEN_FLT x62 = x18 + x19;
	const GEN_FLT x63 = x23 * x62;
	const GEN_FLT x64 = x35 * (1 - x63);
	const GEN_FLT x65 = -lh_pz - x59 - x61 - x64;
	const GEN_FLT x66 = x57 * x57 + x65 * x65;
	const GEN_FLT x67 = pow(x66, -1.0 / 2.0);
	const GEN_FLT x68 = x47 * x67;
	const GEN_FLT x69 = x45 * x68;
	const GEN_FLT x70 = cos(x46);
	const GEN_FLT x71 = 1.0 / x70;
	const GEN_FLT x72 = x45 * x45;
	const GEN_FLT x73 = x66 + x72;
	const GEN_FLT x74 = pow(x73, -1.0 / 2.0);
	const GEN_FLT x75 = x71 * x74;
	const GEN_FLT x76 = asin(x45 * x75);
	const GEN_FLT x77 = 8.0108022e-6 * x76;
	const GEN_FLT x78 = -x77 - 8.0108022e-6;
	const GEN_FLT x79 = x76 * x78 + 0.0028679863;
	const GEN_FLT x80 = x76 * x79 + 5.3685255000000001e-6;
	const GEN_FLT x81 = x76 * x80 + 0.0076069798000000001;
	const GEN_FLT x82 = x76 * x76;
	const GEN_FLT x83 = atan2(x65, x57);
	const GEN_FLT x84 = ogeePhase_0 + x83 - asin(x69);
	const GEN_FLT x85 = ogeeMag_0 * sin(x84);
	const GEN_FLT x86 = curve_0 + x85;
	const GEN_FLT x87 = x76 * x81;
	const GEN_FLT x88 = -1.60216044e-5 * x76 - 8.0108022e-6;
	const GEN_FLT x89 = x76 * x88 + x79;
	const GEN_FLT x90 = x76 * x89 + x80;
	const GEN_FLT x91 = x76 * x90 + x81;
	const GEN_FLT x92 = sin(x46);
	const GEN_FLT x93 = x92 * (x76 * x91 + x87);
	const GEN_FLT x94 = x70 - x86 * x93;
	const GEN_FLT x95 = 1.0 / x94;
	const GEN_FLT x96 = x86 * x95;
	const GEN_FLT x97 = x82 * x96;
	const GEN_FLT x98 = x69 + x81 * x97;
	const GEN_FLT x99 = pow(1 - x98 * x98, -1.0 / 2.0);
	const GEN_FLT x100 = x72 / x73;
	const GEN_FLT x101 = pow(-x100 / (x70 * x70) + 1, -1.0 / 2.0);
	const GEN_FLT x102 = x2 * x23;
	const GEN_FLT x103 = x23 * x45;
	const GEN_FLT x104 = 4 * x22;
	const GEN_FLT x105 = (1.0 / 2.0) * x57;
	const GEN_FLT x106 = x23 * x65;
	const GEN_FLT x107 = -x105 * (-x104 * x21 + 2) + x106 * x60;
	const GEN_FLT x108 = -x103 * x2 + x107;
	const GEN_FLT x109 = x45 / pow(x73, 3.0 / 2.0);
	const GEN_FLT x110 = x109 * x71;
	const GEN_FLT x111 = x101 * (x102 * x75 + x108 * x110);
	const GEN_FLT x112 = x111 * x78;
	const GEN_FLT x113 = x111 * x79 + x76 * (-x111 * x77 + x112);
	const GEN_FLT x114 = x111 * x80 + x113 * x76;
	const GEN_FLT x115 = 1.0 / x66;
	const GEN_FLT x116 = x115 * x72;
	const GEN_FLT x117 = pow(-x116 * x47 * x47 + 1, -1.0 / 2.0);
	const GEN_FLT x118 = x45 / pow(x66, 3.0 / 2.0);
	const GEN_FLT x119 = x118 * x47;
	const GEN_FLT x120 = x102 * x68 + x107 * x119;
	const GEN_FLT x121 = x115 * (lh_pz + x59 + x61 + x64);
	const GEN_FLT x122 = x115 * x57;
	const GEN_FLT x123 = x122 * x23;
	const GEN_FLT x124 = x121 * x55 - x123 * x60;
	const GEN_FLT x125 = -x117 * x120 + x124;
	const GEN_FLT x126 = ogeeMag_0 * cos(x84);
	const GEN_FLT x127 = x126 * x93;
	const GEN_FLT x128 = 2.40324066e-5 * x76;
	const GEN_FLT x129 = x92 * (-curve_0 - x85);
	const GEN_FLT x130 = x81 * x82;
	const GEN_FLT x131 = x130 * x86 / ((x94 * x94));
	const GEN_FLT x132 = x126 * x130 * x95;
	const GEN_FLT x133 = 2 * x87 * x96;
	const GEN_FLT x134 =
		x124 - x99 * (x111 * x133 + x114 * x97 + x120 + x125 * x132 +
					  x131 * (x125 * x127 -
							  x129 * (x111 * x81 + x111 * x91 + x114 * x76 +
									  x76 * (x111 * x90 + x114 +
											 x76 * (x111 * x89 + x113 + x76 * (-x111 * x128 + x111 * x88 + x112))))));
	const GEN_FLT x135 = gibMag_0 * cos(gibPhase_0 + x83 - asin(x98));
	const GEN_FLT x136 = (1.0 / 2.0) * x45;
	const GEN_FLT x137 = x23 * x57;
	const GEN_FLT x138 = x106 * x58 - x137 * x52;
	const GEN_FLT x139 = -x136 * (-x104 * x38 + 2) + x138;
	const GEN_FLT x140 = x101 * (x110 * x139 + x39 * x75);
	const GEN_FLT x141 = x140 * x78;
	const GEN_FLT x142 = x140 * x79 + x76 * (-x140 * x77 + x141);
	const GEN_FLT x143 = x140 * x80 + x142 * x76;
	const GEN_FLT x144 = x119 * x138 + x39 * x68;
	const GEN_FLT x145 = x121 * x23;
	const GEN_FLT x146 = -x123 * x58 + x145 * x52;
	const GEN_FLT x147 = -x117 * x144 + x146;
	const GEN_FLT x148 =
		x146 - x99 * (x131 * (x127 * x147 -
							  x129 * (x140 * x81 + x140 * x91 + x143 * x76 +
									  x76 * (x140 * x90 + x143 +
											 x76 * (x140 * x89 + x142 + x76 * (-x128 * x140 + x140 * x88 + x141))))) +
					  x132 * x147 + x133 * x140 + x143 * x97 + x144);
	const GEN_FLT x149 = x23 * x28;
	const GEN_FLT x150 = (1.0 / 2.0) * x65;
	const GEN_FLT x151 = -x137 * x50 - x150 * (x104 * x62 - 2);
	const GEN_FLT x152 = -x103 * x28 + x151;
	const GEN_FLT x153 = x101 * (x110 * x152 + x149 * x75);
	const GEN_FLT x154 = x153 * x78;
	const GEN_FLT x155 = x153 * x79 + x76 * (-x153 * x77 + x154);
	const GEN_FLT x156 = x153 * x80 + x155 * x76;
	const GEN_FLT x157 = x119 * x151 + x149 * x68;
	const GEN_FLT x158 = x63 - 1;
	const GEN_FLT x159 = x122 * x158 + x145 * x50;
	const GEN_FLT x160 = -x117 * x157 + x159;
	const GEN_FLT x161 =
		x159 - x99 * (x131 * (x127 * x160 -
							  x129 * (x153 * x81 + x153 * x91 + x156 * x76 +
									  x76 * (x153 * x90 + x156 +
											 x76 * (x153 * x89 + x155 + x76 * (-x128 * x153 + x153 * x88 + x154))))) +
					  x132 * x160 + x133 * x153 + x156 * x97 + x157);
	const GEN_FLT x162 = obj_qj * x12;
	const GEN_FLT x163 = obj_qk * x16;
	const GEN_FLT x164 = 2 / x10;
	const GEN_FLT x165 = obj_qw * x164;
	const GEN_FLT x166 = sensor_x * x165;
	const GEN_FLT x167 = sensor_z * x165;
	const GEN_FLT x168 = sensor_y * x165;
	const GEN_FLT x169 = x15 * x168 + x162 - x163 - x166 * x9 + x167 * x5;
	const GEN_FLT x170 = x169 * x2;
	const GEN_FLT x171 = obj_qi * x16;
	const GEN_FLT x172 = obj_qj * x33;
	const GEN_FLT x173 = x166 * x32 - x167 * x34 + x168 * x31 + x171 - x172;
	const GEN_FLT x174 = x173 * x28;
	const GEN_FLT x175 = obj_qi * x12;
	const GEN_FLT x176 = obj_qk * x33;
	const GEN_FLT x177 = sensor_y * x42;
	const GEN_FLT x178 = -x165 * x177 + x166 * x40 + x167 * x41 - x175 + x176;
	const GEN_FLT x179 = x178 * x39;
	const GEN_FLT x180 = x170 * x23 + x174 * x23 + x179;
	const GEN_FLT x181 = x173 * x50;
	const GEN_FLT x182 = x104 * x178;
	const GEN_FLT x183 = x169 * x55;
	const GEN_FLT x184 = x169 * x60;
	const GEN_FLT x185 = x158 * x173;
	const GEN_FLT x186 = -x105 * (x104 * x181 + x182 * x52 + 2 * x183) - x150 * (-x104 * x184 - x182 * x58 + 2 * x185);
	const GEN_FLT x187 = -x136 * (x104 * x170 + x104 * x174 + 2 * x179) + x186;
	const GEN_FLT x188 = x101 * (x110 * x187 + x180 * x75);
	const GEN_FLT x189 = x188 * x78;
	const GEN_FLT x190 = x188 * x79 + x76 * (-x188 * x77 + x189);
	const GEN_FLT x191 = x188 * x80 + x190 * x76;
	const GEN_FLT x192 = x119 * x186 + x180 * x68;
	const GEN_FLT x193 = x178 * x23;
	const GEN_FLT x194 = x121 * (x181 * x23 + x183 + x193 * x52) + x122 * (-x184 * x23 + x185 - x193 * x58);
	const GEN_FLT x195 = -x117 * x192 + x194;
	const GEN_FLT x196 =
		x194 - x99 * (x131 * (x127 * x195 -
							  x129 * (x188 * x81 + x188 * x91 + x191 * x76 +
									  x76 * (x188 * x90 + x191 +
											 x76 * (x188 * x89 + x190 + x76 * (-x128 * x188 + x188 * x88 + x189))))) +
					  x132 * x195 + x133 * x188 + x191 * x97 + x192);
	const GEN_FLT x197 = obj_qj * x16;
	const GEN_FLT x198 = obj_qk * x12;
	const GEN_FLT x199 = obj_qi * x164;
	const GEN_FLT x200 = sensor_x * x199;
	const GEN_FLT x201 = sensor_z * x199;
	const GEN_FLT x202 = sensor_y * x199;
	const GEN_FLT x203 = x15 * x202 + x197 + x198 - x200 * x9 + x201 * x5;
	const GEN_FLT x204 = x2 * x203;
	const GEN_FLT x205 = obj_qw * x16;
	const GEN_FLT x206 = 4 * x10;
	const GEN_FLT x207 = -obj_qi * x206;
	const GEN_FLT x208 = sensor_z * (-x199 * x34 + x207) + x176 + x200 * x32 + x202 * x31 + x205;
	const GEN_FLT x209 = x208 * x28;
	const GEN_FLT x210 = obj_qw * x12;
	const GEN_FLT x211 = sensor_y * (-x199 * x42 + x207) + x172 + x200 * x40 + x201 * x41 - x210;
	const GEN_FLT x212 = x211 * x39;
	const GEN_FLT x213 = x204 * x23 + x209 * x23 + x212;
	const GEN_FLT x214 = x203 * x55;
	const GEN_FLT x215 = x208 * x50;
	const GEN_FLT x216 = x104 * x211;
	const GEN_FLT x217 = x203 * x60;
	const GEN_FLT x218 = x158 * x208;
	const GEN_FLT x219 = -x105 * (x104 * x215 + 2 * x214 + x216 * x52) - x150 * (-x104 * x217 - x216 * x58 + 2 * x218);
	const GEN_FLT x220 = -x136 * (x104 * x204 + x104 * x209 + 2 * x212) + x219;
	const GEN_FLT x221 = x101 * (x110 * x220 + x213 * x75);
	const GEN_FLT x222 = x221 * x78;
	const GEN_FLT x223 = x221 * x79 + x76 * (-x221 * x77 + x222);
	const GEN_FLT x224 = x221 * x80 + x223 * x76;
	const GEN_FLT x225 = x119 * x219 + x213 * x68;
	const GEN_FLT x226 = x211 * x23;
	const GEN_FLT x227 = x121 * (x214 + x215 * x23 + x226 * x52) + x122 * (-x217 * x23 + x218 - x226 * x58);
	const GEN_FLT x228 = -x117 * x225 + x227;
	const GEN_FLT x229 =
		x227 - x99 * (x131 * (x127 * x228 -
							  x129 * (x221 * x81 + x221 * x91 + x224 * x76 +
									  x76 * (x221 * x90 + x224 +
											 x76 * (x221 * x89 + x223 + x76 * (-x128 * x221 + x221 * x88 + x222))))) +
					  x132 * x228 + x133 * x221 + x224 * x97 + x225);
	const GEN_FLT x230 = obj_qi * x33;
	const GEN_FLT x231 = obj_qj * x164;
	const GEN_FLT x232 = sensor_x * x231;
	const GEN_FLT x233 = sensor_z * x231;
	const GEN_FLT x234 = -x177 * x231 + x198 + x230 + x232 * x40 + x233 * x41;
	const GEN_FLT x235 = x234 * x39;
	const GEN_FLT x236 = sensor_y * x231;
	const GEN_FLT x237 = -obj_qj * x206;
	const GEN_FLT x238 = sensor_x * (-x231 * x9 + x237) + x15 * x236 + x171 + x210 + x233 * x5;
	const GEN_FLT x239 = x2 * x238;
	const GEN_FLT x240 = obj_qw * x33;
	const GEN_FLT x241 = sensor_z * (-x231 * x34 + x237) + x163 + x232 * x32 + x236 * x31 - x240;
	const GEN_FLT x242 = x241 * x28;
	const GEN_FLT x243 = x23 * x239 + x23 * x242 + x235;
	const GEN_FLT x244 = x104 * x234;
	const GEN_FLT x245 = x241 * x50;
	const GEN_FLT x246 = x238 * x55;
	const GEN_FLT x247 = x238 * x60;
	const GEN_FLT x248 = x158 * x241;
	const GEN_FLT x249 = -x105 * (x104 * x245 + x244 * x52 + 2 * x246) - x150 * (-x104 * x247 - x244 * x58 + 2 * x248);
	const GEN_FLT x250 = -x136 * (x104 * x239 + x104 * x242 + 2 * x235) + x249;
	const GEN_FLT x251 = x101 * (x110 * x250 + x243 * x75);
	const GEN_FLT x252 = x251 * x78;
	const GEN_FLT x253 = x251 * x79 + x76 * (-x251 * x77 + x252);
	const GEN_FLT x254 = x251 * x80 + x253 * x76;
	const GEN_FLT x255 = x119 * x249 + x243 * x68;
	const GEN_FLT x256 = x23 * x234;
	const GEN_FLT x257 = x121 * (x23 * x245 + x246 + x256 * x52) + x122 * (-x23 * x247 + x248 - x256 * x58);
	const GEN_FLT x258 = -x117 * x255 + x257;
	const GEN_FLT x259 =
		x257 - x99 * (x131 * (x127 * x258 -
							  x129 * (x251 * x81 + x251 * x91 + x254 * x76 +
									  x76 * (x251 * x90 + x254 +
											 x76 * (x251 * x89 + x253 + x76 * (-x128 * x251 + x251 * x88 + x252))))) +
					  x132 * x258 + x133 * x251 + x254 * x97 + x255);
	const GEN_FLT x260 = obj_qk * x164;
	const GEN_FLT x261 = sensor_y * x260;
	const GEN_FLT x262 = sensor_z * x260;
	const GEN_FLT x263 = sensor_x * x260;
	const GEN_FLT x264 = x197 + x230 + x261 * x31 - x262 * x34 + x263 * x32;
	const GEN_FLT x265 = x264 * x28;
	const GEN_FLT x266 = -obj_qk * x206;
	const GEN_FLT x267 = sensor_x * (-x260 * x9 + x266) + x15 * x261 + x175 - x205 + x262 * x5;
	const GEN_FLT x268 = x2 * x267;
	const GEN_FLT x269 = sensor_y * (-x260 * x42 + x266) + x162 + x240 + x262 * x41 + x263 * x40;
	const GEN_FLT x270 = x269 * x39;
	const GEN_FLT x271 = x23 * x265 + x23 * x268 + x270;
	const GEN_FLT x272 = x264 * x50;
	const GEN_FLT x273 = x104 * x269;
	const GEN_FLT x274 = x267 * x55;
	const GEN_FLT x275 = x158 * x264;
	const GEN_FLT x276 = x267 * x60;
	const GEN_FLT x277 = -x105 * (x104 * x272 + x273 * x52 + 2 * x274) - x150 * (-x104 * x276 - x273 * x58 + 2 * x275);
	const GEN_FLT x278 = -x136 * (x104 * x265 + x104 * x268 + 2 * x270) + x277;
	const GEN_FLT x279 = x101 * (x110 * x278 + x271 * x75);
	const GEN_FLT x280 = x279 * x78;
	const GEN_FLT x281 = x279 * x79 + x76 * (-x279 * x77 + x280);
	const GEN_FLT x282 = x279 * x80 + x281 * x76;
	const GEN_FLT x283 = x119 * x277 + x271 * x68;
	const GEN_FLT x284 = x23 * x269;
	const GEN_FLT x285 = x121 * (x23 * x272 + x274 + x284 * x52) + x122 * (-x23 * x276 + x275 - x284 * x58);
	const GEN_FLT x286 = -x117 * x283 + x285;
	const GEN_FLT x287 =
		x285 - x99 * (x131 * (x127 * x286 -
							  x129 * (x279 * x81 + x279 * x91 + x282 * x76 +
									  x76 * (x279 * x90 + x282 +
											 x76 * (x279 * x89 + x281 + x76 * (-x128 * x279 + x279 * x88 + x280))))) +
					  x132 * x286 + x133 * x279 + x282 * x97 + x283);
	const GEN_FLT x288 = -lh_px - x51 - x54 - x56;
	const GEN_FLT x289 = x119 * x288;
	const GEN_FLT x290 = x101 * x110;
	const GEN_FLT x291 = x288 * x290;
	const GEN_FLT x292 = -x117 * x289 + x121;
	const GEN_FLT x293 = x291 * x78;
	const GEN_FLT x294 = x291 * x79 + x76 * (-x291 * x77 + x293);
	const GEN_FLT x295 = x291 * x80 + x294 * x76;
	const GEN_FLT x296 =
		x121 - x99 * (x131 * (x127 * x292 -
							  x129 * (x291 * x81 + x291 * x91 + x295 * x76 +
									  x76 * (x291 * x90 + x295 +
											 x76 * (x291 * x89 + x294 + x76 * (-x128 * x291 + x291 * x88 + x293))))) +
					  x132 * x292 + x133 * x291 + x289 + x295 * x97);
	const GEN_FLT x297 = x117 * x68;
	const GEN_FLT x298 = -lh_py - x25 - x37 - x44;
	const GEN_FLT x299 = x101 * (x110 * x298 + x75);
	const GEN_FLT x300 = x299 * x78;
	const GEN_FLT x301 = x299 * x79 + x76 * (-x299 * x77 + x300);
	const GEN_FLT x302 = x299 * x80 + x301 * x76;
	const GEN_FLT x303 =
		x99 *
		(x131 * (-x127 * x297 - x129 * (x299 * x81 + x299 * x91 + x302 * x76 +
										x76 * (x299 * x90 + x302 +
											   x76 * (x299 * x89 + x301 + x76 * (-x128 * x299 + x299 * x88 + x300))))) -
		 x132 * x297 + x133 * x299 + x302 * x97 + x68);
	const GEN_FLT x304 = -x122;
	const GEN_FLT x305 = x119 * x65;
	const GEN_FLT x306 = x290 * x65;
	const GEN_FLT x307 = -x117 * x305 + x304;
	const GEN_FLT x308 = x306 * x78;
	const GEN_FLT x309 = x306 * x79 + x76 * (-x306 * x77 + x308);
	const GEN_FLT x310 = x306 * x80 + x309 * x76;
	const GEN_FLT x311 =
		x304 - x99 * (x131 * (x127 * x307 -
							  x129 * (x306 * x81 + x306 * x91 + x310 * x76 +
									  x76 * (x306 * x90 + x310 +
											 x76 * (x306 * x89 + x309 + x76 * (-x128 * x306 + x306 * x88 + x308))))) +
					  x132 * x307 + x133 * x306 + x305 + x310 * x97);
	const GEN_FLT x312 = lh_qi * x36;
	const GEN_FLT x313 = lh_qk * x24;
	const GEN_FLT x314 = 1.0 / x22;
	const GEN_FLT x315 = 2 * x314;
	const GEN_FLT x316 = lh_qw * x315;
	const GEN_FLT x317 = x38 * x43;
	const GEN_FLT x318 = x17 * x2;
	const GEN_FLT x319 = x28 * x35;
	const GEN_FLT x320 = -x312 + x313 - x316 * x317 + x316 * x318 + x316 * x319;
	const GEN_FLT x321 = lh_qi * x104;
	const GEN_FLT x322 = x321 * x35;
	const GEN_FLT x323 = lh_qk * x104;
	const GEN_FLT x324 = x17 * x323;
	const GEN_FLT x325 = 4 * x314;
	const GEN_FLT x326 = lh_qw * x325;
	const GEN_FLT x327 = lh_qj * x104;
	const GEN_FLT x328 = x327 * x35;
	const GEN_FLT x329 = -x323 * x43;
	const GEN_FLT x330 = x17 * x21;
	const GEN_FLT x331 = x326 * x35;
	const GEN_FLT x332 = x326 * x43;
	const GEN_FLT x333 = x321 * x43;
	const GEN_FLT x334 = x17 * x327;
	const GEN_FLT x335 = x17 * x60;
	const GEN_FLT x336 = -x105 * (-x326 * x330 + x328 + x329 + x331 * x50 + x332 * x52) -
						 x150 * (-x326 * x335 + x331 * x62 - x332 * x58 - x333 + x334);
	const GEN_FLT x337 = -x136 * (-x317 * x326 + x318 * x326 + x319 * x326 - x322 + x324) + x336;
	const GEN_FLT x338 = x101 * (x110 * x337 + x320 * x75);
	const GEN_FLT x339 = x338 * x78;
	const GEN_FLT x340 = x338 * x79 + x76 * (-x338 * x77 + x339);
	const GEN_FLT x341 = x338 * x80 + x340 * x76;
	const GEN_FLT x342 = x119 * x336 + x320 * x68;
	const GEN_FLT x343 = lh_qi * x53;
	const GEN_FLT x344 = lh_qj * x24;
	const GEN_FLT x345 = x316 * x35;
	const GEN_FLT x346 = x316 * x43;
	const GEN_FLT x347 = lh_qj * x36;
	const GEN_FLT x348 = -lh_qk * x53;
	const GEN_FLT x349 = x121 * (-x316 * x330 + x345 * x50 + x346 * x52 + x347 + x348) +
						 x122 * (-x316 * x335 - x343 + x344 + x345 * x62 - x346 * x58);
	const GEN_FLT x350 = -x117 * x342 + x349;
	const GEN_FLT x351 =
		x349 - x99 * (x131 * (x127 * x350 -
							  x129 * (x338 * x81 + x338 * x91 + x341 * x76 +
									  x76 * (x338 * x90 + x341 +
											 x76 * (x338 * x89 + x340 + x76 * (-x128 * x338 + x338 * x88 + x339))))) +
					  x132 * x350 + x133 * x338 + x341 * x97 + x342);
	const GEN_FLT x352 = lh_qw * x36;
	const GEN_FLT x353 = lh_qi * x315;
	const GEN_FLT x354 = x43 * (-x321 - x353 * x38);
	const GEN_FLT x355 = x318 * x353 + x319 * x353 + x344 - x352 + x354;
	const GEN_FLT x356 = lh_qw * x104;
	const GEN_FLT x357 = x35 * x356;
	const GEN_FLT x358 = lh_qi * x325;
	const GEN_FLT x359 = x327 * x43;
	const GEN_FLT x360 = x323 * x35;
	const GEN_FLT x361 = x35 * x50;
	const GEN_FLT x362 = x358 * x43;
	const GEN_FLT x363 = -x356 * x43;
	const GEN_FLT x364 = x35 * (x321 + x353 * x62);
	const GEN_FLT x365 = -x105 * (-x330 * x358 + x358 * x361 + x359 + x360 + x362 * x52) -
						 x150 * (-x324 - x335 * x358 - x362 * x58 + x363 + 2 * x364);
	const GEN_FLT x366 = -x136 * (x318 * x358 + x319 * x358 + x334 + 2 * x354 - x357) + x365;
	const GEN_FLT x367 = x101 * (x110 * x366 + x355 * x75);
	const GEN_FLT x368 = x367 * x78;
	const GEN_FLT x369 = x367 * x79 + x76 * (-x367 * x77 + x368);
	const GEN_FLT x370 = x367 * x80 + x369 * x76;
	const GEN_FLT x371 = x119 * x365 + x355 * x68;
	const GEN_FLT x372 = -lh_qw * x53;
	const GEN_FLT x373 = x353 * x43;
	const GEN_FLT x374 = lh_qj * x53;
	const GEN_FLT x375 = lh_qk * x36;
	const GEN_FLT x376 = x121 * (-x330 * x353 + x353 * x361 + x373 * x52 + x374 + x375) +
						 x122 * (-x313 - x335 * x353 + x364 + x372 - x373 * x58);
	const GEN_FLT x377 = -x117 * x371 + x376;
	const GEN_FLT x378 =
		x376 - x99 * (x131 * (x127 * x377 -
							  x129 * (x367 * x81 + x367 * x91 + x370 * x76 +
									  x76 * (x367 * x90 + x370 +
											 x76 * (x367 * x89 + x369 + x76 * (-x128 * x367 + x367 * x88 + x368))))) +
					  x132 * x377 + x133 * x367 + x370 * x97 + x371);
	const GEN_FLT x379 = lh_qi * x24;
	const GEN_FLT x380 = lh_qj * x315;
	const GEN_FLT x381 = -x317 * x380 + x318 * x380 + x319 * x380 + x375 + x379;
	const GEN_FLT x382 = x17 * x321;
	const GEN_FLT x383 = lh_qj * x325;
	const GEN_FLT x384 = x43 * x52;
	const GEN_FLT x385 = x17 * (-x21 * x380 - x327);
	const GEN_FLT x386 = x17 * x356;
	const GEN_FLT x387 = x43 * x58;
	const GEN_FLT x388 = x35 * (x327 + x380 * x62);
	const GEN_FLT x389 = -x105 * (x333 + x357 + x361 * x383 + x383 * x384 + 2 * x385) -
						 x150 * (x329 - x335 * x383 - x383 * x387 + x386 + 2 * x388);
	const GEN_FLT x390 = -x136 * (-x317 * x383 + x318 * x383 + x319 * x383 + x360 + x382) + x389;
	const GEN_FLT x391 = x101 * (x110 * x390 + x381 * x75);
	const GEN_FLT x392 = x391 * x78;
	const GEN_FLT x393 = x391 * x79 + x76 * (-x391 * x77 + x392);
	const GEN_FLT x394 = x391 * x80 + x393 * x76;
	const GEN_FLT x395 = x119 * x389 + x381 * x68;
	const GEN_FLT x396 = lh_qw * x24;
	const GEN_FLT x397 = x121 * (x343 + x352 + x361 * x380 + x380 * x384 + x385) +
						 x122 * (-x335 * x380 + x348 - x380 * x387 + x388 + x396);
	const GEN_FLT x398 = -x117 * x395 + x397;
	const GEN_FLT x399 =
		x397 - x99 * (x131 * (x127 * x398 -
							  x129 * (x391 * x81 + x391 * x91 + x394 * x76 +
									  x76 * (x391 * x90 + x394 +
											 x76 * (x391 * x89 + x393 + x76 * (-x128 * x391 + x391 * x88 + x392))))) +
					  x132 * x398 + x133 * x391 + x394 * x97 + x395);
	const GEN_FLT x400 = lh_qk * x315;
	const GEN_FLT x401 = -x323;
	const GEN_FLT x402 = x43 * (-x38 * x400 + x401);
	const GEN_FLT x403 = x318 * x400 + x319 * x400 + x347 + x396 + x402;
	const GEN_FLT x404 = lh_qk * x325;
	const GEN_FLT x405 = x35 * x62;
	const GEN_FLT x406 = x17 * (-x21 * x400 + x401);
	const GEN_FLT x407 = -x105 * (x322 + x361 * x404 + x363 + x384 * x404 + 2 * x406) -
						 x150 * (-x335 * x404 - x359 - x382 - x387 * x404 + x404 * x405);
	const GEN_FLT x408 = -x136 * (x318 * x404 + x319 * x404 + x328 + x386 + 2 * x402) + x407;
	const GEN_FLT x409 = x101 * (x110 * x408 + x403 * x75);
	const GEN_FLT x410 = x409 * x78;
	const GEN_FLT x411 = x409 * x79 + x76 * (-x409 * x77 + x410);
	const GEN_FLT x412 = x409 * x80 + x411 * x76;
	const GEN_FLT x413 = x119 * x407 + x403 * x68;
	const GEN_FLT x414 = x121 * (x312 + x361 * x400 + x372 + x384 * x400 + x406) +
						 x122 * (-x335 * x400 - x374 - x379 - x387 * x400 + x400 * x405);
	const GEN_FLT x415 = -x117 * x413 + x414;
	const GEN_FLT x416 =
		x414 - x99 * (x131 * (x127 * x415 -
							  x129 * (x409 * x81 + x409 * x91 + x412 * x76 +
									  x76 * (x409 * x90 + x412 +
											 x76 * (x409 * x89 + x411 + x76 * (-x128 * x409 + x409 * x88 + x410))))) +
					  x132 * x415 + x133 * x409 + x412 * x97 + x413);
	const GEN_FLT x417 = tilt_1 - 0.52359877559829882;
	const GEN_FLT x418 = tan(x417);
	const GEN_FLT x419 = x418 * x67;
	const GEN_FLT x420 = x419 * x45;
	const GEN_FLT x421 = cos(x417);
	const GEN_FLT x422 = 1.0 / x421;
	const GEN_FLT x423 = x422 * x74;
	const GEN_FLT x424 = asin(x423 * x45);
	const GEN_FLT x425 = 8.0108022e-6 * x424;
	const GEN_FLT x426 = -x425 - 8.0108022e-6;
	const GEN_FLT x427 = x424 * x426 + 0.0028679863;
	const GEN_FLT x428 = x424 * x427 + 5.3685255000000001e-6;
	const GEN_FLT x429 = x424 * x428 + 0.0076069798000000001;
	const GEN_FLT x430 = x424 * x424;
	const GEN_FLT x431 = ogeePhase_1 + x83 - asin(x420);
	const GEN_FLT x432 = ogeeMag_1 * sin(x431);
	const GEN_FLT x433 = curve_1 + x432;
	const GEN_FLT x434 = x424 * x429;
	const GEN_FLT x435 = -1.60216044e-5 * x424 - 8.0108022e-6;
	const GEN_FLT x436 = x424 * x435 + x427;
	const GEN_FLT x437 = x424 * x436 + x428;
	const GEN_FLT x438 = x424 * x437 + x429;
	const GEN_FLT x439 = sin(x417);
	const GEN_FLT x440 = x439 * (x424 * x438 + x434);
	const GEN_FLT x441 = x421 - x433 * x440;
	const GEN_FLT x442 = 1.0 / x441;
	const GEN_FLT x443 = x433 * x442;
	const GEN_FLT x444 = x430 * x443;
	const GEN_FLT x445 = x420 + x429 * x444;
	const GEN_FLT x446 = pow(1 - x445 * x445, -1.0 / 2.0);
	const GEN_FLT x447 = pow(-x100 / (x421 * x421) + 1, -1.0 / 2.0);
	const GEN_FLT x448 = x109 * x422;
	const GEN_FLT x449 = x447 * (x102 * x423 + x108 * x448);
	const GEN_FLT x450 = x426 * x449;
	const GEN_FLT x451 = x424 * (-x425 * x449 + x450) + x427 * x449;
	const GEN_FLT x452 = x424 * x451 + x428 * x449;
	const GEN_FLT x453 = pow(-x116 * x418 * x418 + 1, -1.0 / 2.0);
	const GEN_FLT x454 = x118 * x418;
	const GEN_FLT x455 = x102 * x419 + x107 * x454;
	const GEN_FLT x456 = x124 - x453 * x455;
	const GEN_FLT x457 = ogeeMag_1 * cos(x431);
	const GEN_FLT x458 = x440 * x457;
	const GEN_FLT x459 = 2.40324066e-5 * x424;
	const GEN_FLT x460 = x439 * (-curve_1 - x432);
	const GEN_FLT x461 = x429 * x430;
	const GEN_FLT x462 = x433 * x461 / ((x441 * x441));
	const GEN_FLT x463 = x442 * x457 * x461;
	const GEN_FLT x464 = 2 * x434 * x443;
	const GEN_FLT x465 =
		x124 - x446 * (x444 * x452 + x449 * x464 + x455 + x456 * x463 +
					   x462 * (x456 * x458 -
							   x460 * (x424 * x452 +
									   x424 * (x424 * (x424 * (x435 * x449 - x449 * x459 + x450) + x436 * x449 + x451) +
											   x437 * x449 + x452) +
									   x429 * x449 + x438 * x449)));
	const GEN_FLT x466 = gibMag_1 * cos(gibPhase_1 + x83 - asin(x445));
	const GEN_FLT x467 = x447 * (x139 * x448 + x39 * x423);
	const GEN_FLT x468 = x426 * x467;
	const GEN_FLT x469 = x424 * (-x425 * x467 + x468) + x427 * x467;
	const GEN_FLT x470 = x424 * x469 + x428 * x467;
	const GEN_FLT x471 = x138 * x454 + x39 * x419;
	const GEN_FLT x472 = x146 - x453 * x471;
	const GEN_FLT x473 =
		x146 - x446 * (x444 * x470 +
					   x462 * (x458 * x472 -
							   x460 * (x424 * x470 +
									   x424 * (x424 * (x424 * (x435 * x467 - x459 * x467 + x468) + x436 * x467 + x469) +
											   x437 * x467 + x470) +
									   x429 * x467 + x438 * x467)) +
					   x463 * x472 + x464 * x467 + x471);
	const GEN_FLT x474 = x447 * (x149 * x423 + x152 * x448);
	const GEN_FLT x475 = x426 * x474;
	const GEN_FLT x476 = x424 * (-x425 * x474 + x475) + x427 * x474;
	const GEN_FLT x477 = x424 * x476 + x428 * x474;
	const GEN_FLT x478 = x149 * x419 + x151 * x454;
	const GEN_FLT x479 = x159 - x453 * x478;
	const GEN_FLT x480 =
		x159 - x446 * (x444 * x477 +
					   x462 * (x458 * x479 -
							   x460 * (x424 * x477 +
									   x424 * (x424 * (x424 * (x435 * x474 - x459 * x474 + x475) + x436 * x474 + x476) +
											   x437 * x474 + x477) +
									   x429 * x474 + x438 * x474)) +
					   x463 * x479 + x464 * x474 + x478);
	const GEN_FLT x481 = x447 * (x180 * x423 + x187 * x448);
	const GEN_FLT x482 = x426 * x481;
	const GEN_FLT x483 = x424 * (-x425 * x481 + x482) + x427 * x481;
	const GEN_FLT x484 = x424 * x483 + x428 * x481;
	const GEN_FLT x485 = x180 * x419 + x186 * x454;
	const GEN_FLT x486 = x194 - x453 * x485;
	const GEN_FLT x487 =
		x194 - x446 * (x444 * x484 +
					   x462 * (x458 * x486 -
							   x460 * (x424 * x484 +
									   x424 * (x424 * (x424 * (x435 * x481 - x459 * x481 + x482) + x436 * x481 + x483) +
											   x437 * x481 + x484) +
									   x429 * x481 + x438 * x481)) +
					   x463 * x486 + x464 * x481 + x485);
	const GEN_FLT x488 = x447 * (x213 * x423 + x220 * x448);
	const GEN_FLT x489 = x426 * x488;
	const GEN_FLT x490 = x424 * (-x425 * x488 + x489) + x427 * x488;
	const GEN_FLT x491 = x424 * x490 + x428 * x488;
	const GEN_FLT x492 = x213 * x419 + x219 * x454;
	const GEN_FLT x493 = x227 - x453 * x492;
	const GEN_FLT x494 =
		x227 - x446 * (x444 * x491 +
					   x462 * (x458 * x493 -
							   x460 * (x424 * x491 +
									   x424 * (x424 * (x424 * (x435 * x488 - x459 * x488 + x489) + x436 * x488 + x490) +
											   x437 * x488 + x491) +
									   x429 * x488 + x438 * x488)) +
					   x463 * x493 + x464 * x488 + x492);
	const GEN_FLT x495 = x447 * (x243 * x423 + x250 * x448);
	const GEN_FLT x496 = x426 * x495;
	const GEN_FLT x497 = x424 * (-x425 * x495 + x496) + x427 * x495;
	const GEN_FLT x498 = x424 * x497 + x428 * x495;
	const GEN_FLT x499 = x243 * x419 + x249 * x454;
	const GEN_FLT x500 = x257 - x453 * x499;
	const GEN_FLT x501 =
		x257 - x446 * (x444 * x498 +
					   x462 * (x458 * x500 -
							   x460 * (x424 * x498 +
									   x424 * (x424 * (x424 * (x435 * x495 - x459 * x495 + x496) + x436 * x495 + x497) +
											   x437 * x495 + x498) +
									   x429 * x495 + x438 * x495)) +
					   x463 * x500 + x464 * x495 + x499);
	const GEN_FLT x502 = x447 * (x271 * x423 + x278 * x448);
	const GEN_FLT x503 = x426 * x502;
	const GEN_FLT x504 = x424 * (-x425 * x502 + x503) + x427 * x502;
	const GEN_FLT x505 = x424 * x504 + x428 * x502;
	const GEN_FLT x506 = x271 * x419 + x277 * x454;
	const GEN_FLT x507 = x285 - x453 * x506;
	const GEN_FLT x508 =
		x285 - x446 * (x444 * x505 +
					   x462 * (x458 * x507 -
							   x460 * (x424 * x505 +
									   x424 * (x424 * (x424 * (x435 * x502 - x459 * x502 + x503) + x436 * x502 + x504) +
											   x437 * x502 + x505) +
									   x429 * x502 + x438 * x502)) +
					   x463 * x507 + x464 * x502 + x506);
	const GEN_FLT x509 = x288 * x454;
	const GEN_FLT x510 = x447 * x448;
	const GEN_FLT x511 = x288 * x510;
	const GEN_FLT x512 = x121 - x453 * x509;
	const GEN_FLT x513 = x426 * x511;
	const GEN_FLT x514 = x424 * (-x425 * x511 + x513) + x427 * x511;
	const GEN_FLT x515 = x424 * x514 + x428 * x511;
	const GEN_FLT x516 =
		x121 - x446 * (x444 * x515 +
					   x462 * (x458 * x512 -
							   x460 * (x424 * x515 +
									   x424 * (x424 * (x424 * (x435 * x511 - x459 * x511 + x513) + x436 * x511 + x514) +
											   x437 * x511 + x515) +
									   x429 * x511 + x438 * x511)) +
					   x463 * x512 + x464 * x511 + x509);
	const GEN_FLT x517 = x419 * x453;
	const GEN_FLT x518 = x447 * (x298 * x448 + x423);
	const GEN_FLT x519 = x426 * x518;
	const GEN_FLT x520 = x424 * (-x425 * x518 + x519) + x427 * x518;
	const GEN_FLT x521 = x424 * x520 + x428 * x518;
	const GEN_FLT x522 =
		x446 * (x419 + x444 * x521 +
				x462 * (-x458 * x517 -
						x460 * (x424 * x521 +
								x424 * (x424 * (x424 * (x435 * x518 - x459 * x518 + x519) + x436 * x518 + x520) +
										x437 * x518 + x521) +
								x429 * x518 + x438 * x518)) -
				x463 * x517 + x464 * x518);
	const GEN_FLT x523 = x454 * x65;
	const GEN_FLT x524 = x510 * x65;
	const GEN_FLT x525 = x304 - x453 * x523;
	const GEN_FLT x526 = x426 * x524;
	const GEN_FLT x527 = x424 * (-x425 * x524 + x526) + x427 * x524;
	const GEN_FLT x528 = x424 * x527 + x428 * x524;
	const GEN_FLT x529 =
		x304 - x446 * (x444 * x528 +
					   x462 * (x458 * x525 -
							   x460 * (x424 * x528 +
									   x424 * (x424 * (x424 * (x435 * x524 - x459 * x524 + x526) + x436 * x524 + x527) +
											   x437 * x524 + x528) +
									   x429 * x524 + x438 * x524)) +
					   x463 * x525 + x464 * x524 + x523);
	const GEN_FLT x530 = x447 * (x320 * x423 + x337 * x448);
	const GEN_FLT x531 = x426 * x530;
	const GEN_FLT x532 = x424 * (-x425 * x530 + x531) + x427 * x530;
	const GEN_FLT x533 = x424 * x532 + x428 * x530;
	const GEN_FLT x534 = x320 * x419 + x336 * x454;
	const GEN_FLT x535 = x349 - x453 * x534;
	const GEN_FLT x536 =
		x349 - x446 * (x444 * x533 +
					   x462 * (x458 * x535 -
							   x460 * (x424 * x533 +
									   x424 * (x424 * (x424 * (x435 * x530 - x459 * x530 + x531) + x436 * x530 + x532) +
											   x437 * x530 + x533) +
									   x429 * x530 + x438 * x530)) +
					   x463 * x535 + x464 * x530 + x534);
	const GEN_FLT x537 = x447 * (x355 * x423 + x366 * x448);
	const GEN_FLT x538 = x426 * x537;
	const GEN_FLT x539 = x424 * (-x425 * x537 + x538) + x427 * x537;
	const GEN_FLT x540 = x424 * x539 + x428 * x537;
	const GEN_FLT x541 = x355 * x419 + x365 * x454;
	const GEN_FLT x542 = x376 - x453 * x541;
	const GEN_FLT x543 =
		x376 - x446 * (x444 * x540 +
					   x462 * (x458 * x542 -
							   x460 * (x424 * x540 +
									   x424 * (x424 * (x424 * (x435 * x537 - x459 * x537 + x538) + x436 * x537 + x539) +
											   x437 * x537 + x540) +
									   x429 * x537 + x438 * x537)) +
					   x463 * x542 + x464 * x537 + x541);
	const GEN_FLT x544 = x447 * (x381 * x423 + x390 * x448);
	const GEN_FLT x545 = x426 * x544;
	const GEN_FLT x546 = x424 * (-x425 * x544 + x545) + x427 * x544;
	const GEN_FLT x547 = x424 * x546 + x428 * x544;
	const GEN_FLT x548 = x381 * x419 + x389 * x454;
	const GEN_FLT x549 = x397 - x453 * x548;
	const GEN_FLT x550 =
		x397 - x446 * (x444 * x547 +
					   x462 * (x458 * x549 -
							   x460 * (x424 * x547 +
									   x424 * (x424 * (x424 * (x435 * x544 - x459 * x544 + x545) + x436 * x544 + x546) +
											   x437 * x544 + x547) +
									   x429 * x544 + x438 * x544)) +
					   x463 * x549 + x464 * x544 + x548);
	const GEN_FLT x551 = x447 * (x403 * x423 + x408 * x448);
	const GEN_FLT x552 = x426 * x551;
	const GEN_FLT x553 = x424 * (-x425 * x551 + x552) + x427 * x551;
	const GEN_FLT x554 = x424 * x553 + x428 * x551;
	const GEN_FLT x555 = x403 * x419 + x407 * x454;
	const GEN_FLT x556 = x414 - x453 * x555;
	const GEN_FLT x557 =
		x414 - x446 * (x444 * x554 +
					   x462 * (x458 * x556 -
							   x460 * (x424 * x554 +
									   x424 * (x424 * (x424 * (x435 * x551 - x459 * x551 + x552) + x436 * x551 + x553) +
											   x437 * x551 + x554) +
									   x429 * x551 + x438 * x551)) +
					   x463 * x556 + x464 * x551 + x555);
	*(out++) = x134 * x135 + x134;
	*(out++) = x135 * x148 + x148;
	*(out++) = x135 * x161 + x161;
	*(out++) = x135 * x196 + x196;
	*(out++) = x135 * x229 + x229;
	*(out++) = x135 * x259 + x259;
	*(out++) = x135 * x287 + x287;
	*(out++) = x135 * x296 + x296;
	*(out++) = -x135 * x303 - x303;
	*(out++) = x135 * x311 + x311;
	*(out++) = x135 * x351 + x351;
	*(out++) = x135 * x378 + x378;
	*(out++) = x135 * x399 + x399;
	*(out++) = x135 * x416 + x416;
	*(out++) = x465 * x466 + x465;
	*(out++) = x466 * x473 + x473;
	*(out++) = x466 * x480 + x480;
	*(out++) = x466 * x487 + x487;
	*(out++) = x466 * x494 + x494;
	*(out++) = x466 * x501 + x501;
	*(out++) = x466 * x508 + x508;
	*(out++) = x466 * x516 + x516;
	*(out++) = -x466 * x522 - x522;
	*(out++) = x466 * x529 + x529;
	*(out++) = x466 * x536 + x536;
	*(out++) = x466 * x543 + x543;
	*(out++) = x466 * x550 + x550;
	*(out++) = x466 * x557 + x557;
}

static inline void gen_reproject_axis_x_jac_all_gen2(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
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
	const GEN_FLT x25 = x2 * x24;
	const GEN_FLT x26 = lh_qj * lh_qk;
	const GEN_FLT x27 = lh_qi * lh_qw;
	const GEN_FLT x28 = x26 - x27;
	const GEN_FLT x29 = obj_qi * obj_qw;
	const GEN_FLT x30 = obj_qj * obj_qk;
	const GEN_FLT x31 = x29 + x30;
	const GEN_FLT x32 = x3 - x4;
	const GEN_FLT x33 = sensor_x * x11;
	const GEN_FLT x34 = x6 + x7;
	const GEN_FLT x35 = obj_pz + sensor_z * (-x11 * x34 + 1) + x16 * x31 + x32 * x33;
	const GEN_FLT x36 = x23 * x35;
	const GEN_FLT x37 = x28 * x36;
	const GEN_FLT x38 = x18 + x20;
	const GEN_FLT x39 = -x23 * x38 + 1;
	const GEN_FLT x40 = x13 + x14;
	const GEN_FLT x41 = -x29 + x30;
	const GEN_FLT x42 = x6 + x8;
	const GEN_FLT x43 = obj_py + sensor_y * (-x11 * x42 + 1) + x12 * x41 + x33 * x40;
	const GEN_FLT x44 = x39 * x43;
	const GEN_FLT x45 = lh_py + x25 + x37 + x44;
	const GEN_FLT x46 = tilt_0 + 0.52359877559829882;
	const GEN_FLT x47 = tan(x46);
	const GEN_FLT x48 = lh_qi * lh_qk;
	const GEN_FLT x49 = lh_qj * lh_qw;
	const GEN_FLT x50 = x48 + x49;
	const GEN_FLT x51 = x36 * x50;
	const GEN_FLT x52 = x0 - x1;
	const GEN_FLT x53 = x23 * x43;
	const GEN_FLT x54 = x52 * x53;
	const GEN_FLT x55 = -x21 * x23 + 1;
	const GEN_FLT x56 = x17 * x55;
	const GEN_FLT x57 = lh_px + x51 + x54 + x56;
	const GEN_FLT x58 = x26 + x27;
	const GEN_FLT x59 = x53 * x58;
	const GEN_FLT x60 = x48 - x49;
	const GEN_FLT x61 = x24 * x60;
	const GEN_FLT x62 = x18 + x19;
	const GEN_FLT x63 = x23 * x62;
	const GEN_FLT x64 = x35 * (1 - x63);
	const GEN_FLT x65 = -lh_pz - x59 - x61 - x64;
	const GEN_FLT x66 = x57 * x57 + x65 * x65;
	const GEN_FLT x67 = x47 / sqrt(x66);
	const GEN_FLT x68 = x45 * x67;
	const GEN_FLT x69 = cos(x46);
	const GEN_FLT x70 = 1.0 / x69;
	const GEN_FLT x71 = x45 * x45;
	const GEN_FLT x72 = x66 + x71;
	const GEN_FLT x73 = x70 / sqrt(x72);
	const GEN_FLT x74 = asin(x45 * x73);
	const GEN_FLT x75 = 8.0108022e-6 * x74;
	const GEN_FLT x76 = -x75 - 8.0108022e-6;
	const GEN_FLT x77 = x74 * x76 + 0.0028679863;
	const GEN_FLT x78 = x74 * x77 + 5.3685255000000001e-6;
	const GEN_FLT x79 = x74 * x78 + 0.0076069798000000001;
	const GEN_FLT x80 = x74 * x74;
	const GEN_FLT x81 = atan2(x65, x57);
	const GEN_FLT x82 = ogeePhase_0 + x81 - asin(x68);
	const GEN_FLT x83 = ogeeMag_0 * sin(x82);
	const GEN_FLT x84 = curve_0 + x83;
	const GEN_FLT x85 = x74 * x79;
	const GEN_FLT x86 = -1.60216044e-5 * x74 - 8.0108022e-6;
	const GEN_FLT x87 = x74 * x86 + x77;
	const GEN_FLT x88 = x74 * x87 + x78;
	const GEN_FLT x89 = x74 * x88 + x79;
	const GEN_FLT x90 = sin(x46);
	const GEN_FLT x91 = x90 * (x74 * x89 + x85);
	const GEN_FLT x92 = x69 - x84 * x91;
	const GEN_FLT x93 = 1.0 / x92;
	const GEN_FLT x94 = x84 * x93;
	const GEN_FLT x95 = x80 * x94;
	const GEN_FLT x96 = x68 + x79 * x95;
	const GEN_FLT x97 = pow(1 - x96 * x96, -1.0 / 2.0);
	const GEN_FLT x98 = pow(-x71 / (x72 * (x69 * x69)) + 1, -1.0 / 2.0);
	const GEN_FLT x99 = x2 * x23;
	const GEN_FLT x100 = x23 * x45;
	const GEN_FLT x101 = 4 * x22;
	const GEN_FLT x102 = (1.0 / 2.0) * x57;
	const GEN_FLT x103 = x23 * x65;
	const GEN_FLT x104 = -x102 * (-x101 * x21 + 2) + x103 * x60;
	const GEN_FLT x105 = x45 * x70 / pow(x72, 3.0 / 2.0);
	const GEN_FLT x106 = x98 * (x105 * (-x100 * x2 + x104) + x73 * x99);
	const GEN_FLT x107 = x106 * x76;
	const GEN_FLT x108 = x106 * x77 + x74 * (-x106 * x75 + x107);
	const GEN_FLT x109 = x106 * x78 + x108 * x74;
	const GEN_FLT x110 = 1.0 / x66;
	const GEN_FLT x111 = pow(-x110 * x71 * x47 * x47 + 1, -1.0 / 2.0);
	const GEN_FLT x112 = x45 * x47 / pow(x66, 3.0 / 2.0);
	const GEN_FLT x113 = x104 * x112 + x67 * x99;
	const GEN_FLT x114 = x110 * (lh_pz + x59 + x61 + x64);
	const GEN_FLT x115 = x110 * x57;
	const GEN_FLT x116 = x115 * x23;
	const GEN_FLT x117 = x114 * x55 - x116 * x60;
	const GEN_FLT x118 = -x111 * x113 + x117;
	const GEN_FLT x119 = ogeeMag_0 * cos(x82);
	const GEN_FLT x120 = x119 * x91;
	const GEN_FLT x121 = 2.40324066e-5 * x74;
	const GEN_FLT x122 = x90 * (-curve_0 - x83);
	const GEN_FLT x123 = x79 * x80;
	const GEN_FLT x124 = x123 * x84 / ((x92 * x92));
	const GEN_FLT x125 = x119 * x123 * x93;
	const GEN_FLT x126 = 2 * x85 * x94;
	const GEN_FLT x127 =
		x117 - x97 * (x106 * x126 + x109 * x95 + x113 + x118 * x125 +
					  x124 * (x118 * x120 -
							  x122 * (x106 * x79 + x106 * x89 + x109 * x74 +
									  x74 * (x106 * x88 + x109 +
											 x74 * (x106 * x87 + x108 + x74 * (-x106 * x121 + x106 * x86 + x107))))));
	const GEN_FLT x128 = gibMag_0 * cos(gibPhase_0 + x81 - asin(x96));
	const GEN_FLT x129 = (1.0 / 2.0) * x45;
	const GEN_FLT x130 = x23 * x57;
	const GEN_FLT x131 = x103 * x58 - x130 * x52;
	const GEN_FLT x132 = x98 * (x105 * (-x129 * (-x101 * x38 + 2) + x131) + x39 * x73);
	const GEN_FLT x133 = x132 * x76;
	const GEN_FLT x134 = x132 * x77 + x74 * (-x132 * x75 + x133);
	const GEN_FLT x135 = x132 * x78 + x134 * x74;
	const GEN_FLT x136 = x112 * x131 + x39 * x67;
	const GEN_FLT x137 = x114 * x23;
	const GEN_FLT x138 = -x116 * x58 + x137 * x52;
	const GEN_FLT x139 = -x111 * x136 + x138;
	const GEN_FLT x140 =
		x138 - x97 * (x124 * (x120 * x139 -
							  x122 * (x132 * x79 + x132 * x89 + x135 * x74 +
									  x74 * (x132 * x88 + x135 +
											 x74 * (x132 * x87 + x134 + x74 * (-x121 * x132 + x132 * x86 + x133))))) +
					  x125 * x139 + x126 * x132 + x135 * x95 + x136);
	const GEN_FLT x141 = x23 * x28;
	const GEN_FLT x142 = (1.0 / 2.0) * x65;
	const GEN_FLT x143 = -x130 * x50 - x142 * (x101 * x62 - 2);
	const GEN_FLT x144 = x98 * (x105 * (-x100 * x28 + x143) + x141 * x73);
	const GEN_FLT x145 = x144 * x76;
	const GEN_FLT x146 = x144 * x77 + x74 * (-x144 * x75 + x145);
	const GEN_FLT x147 = x144 * x78 + x146 * x74;
	const GEN_FLT x148 = x112 * x143 + x141 * x67;
	const GEN_FLT x149 = x63 - 1;
	const GEN_FLT x150 = x115 * x149 + x137 * x50;
	const GEN_FLT x151 = -x111 * x148 + x150;
	const GEN_FLT x152 =
		x150 - x97 * (x124 * (x120 * x151 -
							  x122 * (x144 * x79 + x144 * x89 + x147 * x74 +
									  x74 * (x144 * x88 + x147 +
											 x74 * (x144 * x87 + x146 + x74 * (-x121 * x144 + x144 * x86 + x145))))) +
					  x125 * x151 + x126 * x144 + x147 * x95 + x148);
	const GEN_FLT x153 = obj_qj * x12;
	const GEN_FLT x154 = obj_qk * x16;
	const GEN_FLT x155 = 2 / x10;
	const GEN_FLT x156 = obj_qw * x155;
	const GEN_FLT x157 = sensor_x * x156;
	const GEN_FLT x158 = sensor_z * x156;
	const GEN_FLT x159 = sensor_y * x156;
	const GEN_FLT x160 = x15 * x159 + x153 - x154 - x157 * x9 + x158 * x5;
	const GEN_FLT x161 = x160 * x2;
	const GEN_FLT x162 = obj_qi * x16;
	const GEN_FLT x163 = obj_qj * x33;
	const GEN_FLT x164 = x157 * x32 - x158 * x34 + x159 * x31 + x162 - x163;
	const GEN_FLT x165 = x164 * x28;
	const GEN_FLT x166 = obj_qi * x12;
	const GEN_FLT x167 = obj_qk * x33;
	const GEN_FLT x168 = sensor_y * x42;
	const GEN_FLT x169 = -x156 * x168 + x157 * x40 + x158 * x41 - x166 + x167;
	const GEN_FLT x170 = x169 * x39;
	const GEN_FLT x171 = x161 * x23 + x165 * x23 + x170;
	const GEN_FLT x172 = x164 * x50;
	const GEN_FLT x173 = x101 * x169;
	const GEN_FLT x174 = x160 * x55;
	const GEN_FLT x175 = x160 * x60;
	const GEN_FLT x176 = x149 * x164;
	const GEN_FLT x177 = -x102 * (x101 * x172 + x173 * x52 + 2 * x174) - x142 * (-x101 * x175 - x173 * x58 + 2 * x176);
	const GEN_FLT x178 = x98 * (x105 * (-x129 * (x101 * x161 + x101 * x165 + 2 * x170) + x177) + x171 * x73);
	const GEN_FLT x179 = x178 * x76;
	const GEN_FLT x180 = x178 * x77 + x74 * (-x178 * x75 + x179);
	const GEN_FLT x181 = x178 * x78 + x180 * x74;
	const GEN_FLT x182 = x112 * x177 + x171 * x67;
	const GEN_FLT x183 = x169 * x23;
	const GEN_FLT x184 = x114 * (x172 * x23 + x174 + x183 * x52) + x115 * (-x175 * x23 + x176 - x183 * x58);
	const GEN_FLT x185 = -x111 * x182 + x184;
	const GEN_FLT x186 =
		x184 - x97 * (x124 * (x120 * x185 -
							  x122 * (x178 * x79 + x178 * x89 + x181 * x74 +
									  x74 * (x178 * x88 + x181 +
											 x74 * (x178 * x87 + x180 + x74 * (-x121 * x178 + x178 * x86 + x179))))) +
					  x125 * x185 + x126 * x178 + x181 * x95 + x182);
	const GEN_FLT x187 = obj_qj * x16;
	const GEN_FLT x188 = obj_qk * x12;
	const GEN_FLT x189 = obj_qi * x155;
	const GEN_FLT x190 = sensor_x * x189;
	const GEN_FLT x191 = sensor_z * x189;
	const GEN_FLT x192 = sensor_y * x189;
	const GEN_FLT x193 = x15 * x192 + x187 + x188 - x190 * x9 + x191 * x5;
	const GEN_FLT x194 = x193 * x2;
	const GEN_FLT x195 = obj_qw * x16;
	const GEN_FLT x196 = 4 * x10;
	const GEN_FLT x197 = -obj_qi * x196;
	const GEN_FLT x198 = sensor_z * (-x189 * x34 + x197) + x167 + x190 * x32 + x192 * x31 + x195;
	const GEN_FLT x199 = x198 * x28;
	const GEN_FLT x200 = obj_qw * x12;
	const GEN_FLT x201 = sensor_y * (-x189 * x42 + x197) + x163 + x190 * x40 + x191 * x41 - x200;
	const GEN_FLT x202 = x201 * x39;
	const GEN_FLT x203 = x194 * x23 + x199 * x23 + x202;
	const GEN_FLT x204 = x193 * x55;
	const GEN_FLT x205 = x198 * x50;
	const GEN_FLT x206 = x101 * x201;
	const GEN_FLT x207 = x193 * x60;
	const GEN_FLT x208 = x149 * x198;
	const GEN_FLT x209 = -x102 * (x101 * x205 + 2 * x204 + x206 * x52) - x142 * (-x101 * x207 - x206 * x58 + 2 * x208);
	const GEN_FLT x210 = x98 * (x105 * (-x129 * (x101 * x194 + x101 * x199 + 2 * x202) + x209) + x203 * x73);
	const GEN_FLT x211 = x210 * x76;
	const GEN_FLT x212 = x210 * x77 + x74 * (-x210 * x75 + x211);
	const GEN_FLT x213 = x210 * x78 + x212 * x74;
	const GEN_FLT x214 = x112 * x209 + x203 * x67;
	const GEN_FLT x215 = x201 * x23;
	const GEN_FLT x216 = x114 * (x204 + x205 * x23 + x215 * x52) + x115 * (-x207 * x23 + x208 - x215 * x58);
	const GEN_FLT x217 = -x111 * x214 + x216;
	const GEN_FLT x218 =
		x216 - x97 * (x124 * (x120 * x217 -
							  x122 * (x210 * x79 + x210 * x89 + x213 * x74 +
									  x74 * (x210 * x88 + x213 +
											 x74 * (x210 * x87 + x212 + x74 * (-x121 * x210 + x210 * x86 + x211))))) +
					  x125 * x217 + x126 * x210 + x213 * x95 + x214);
	const GEN_FLT x219 = obj_qi * x33;
	const GEN_FLT x220 = obj_qj * x155;
	const GEN_FLT x221 = sensor_x * x220;
	const GEN_FLT x222 = sensor_z * x220;
	const GEN_FLT x223 = -x168 * x220 + x188 + x219 + x221 * x40 + x222 * x41;
	const GEN_FLT x224 = x223 * x39;
	const GEN_FLT x225 = sensor_y * x220;
	const GEN_FLT x226 = -obj_qj * x196;
	const GEN_FLT x227 = sensor_x * (-x220 * x9 + x226) + x15 * x225 + x162 + x200 + x222 * x5;
	const GEN_FLT x228 = x2 * x227;
	const GEN_FLT x229 = obj_qw * x33;
	const GEN_FLT x230 = sensor_z * (-x220 * x34 + x226) + x154 + x221 * x32 + x225 * x31 - x229;
	const GEN_FLT x231 = x230 * x28;
	const GEN_FLT x232 = x224 + x228 * x23 + x23 * x231;
	const GEN_FLT x233 = x101 * x223;
	const GEN_FLT x234 = x230 * x50;
	const GEN_FLT x235 = x227 * x55;
	const GEN_FLT x236 = x227 * x60;
	const GEN_FLT x237 = x149 * x230;
	const GEN_FLT x238 = -x102 * (x101 * x234 + x233 * x52 + 2 * x235) - x142 * (-x101 * x236 - x233 * x58 + 2 * x237);
	const GEN_FLT x239 = x98 * (x105 * (-x129 * (x101 * x228 + x101 * x231 + 2 * x224) + x238) + x232 * x73);
	const GEN_FLT x240 = x239 * x76;
	const GEN_FLT x241 = x239 * x77 + x74 * (-x239 * x75 + x240);
	const GEN_FLT x242 = x239 * x78 + x241 * x74;
	const GEN_FLT x243 = x112 * x238 + x232 * x67;
	const GEN_FLT x244 = x223 * x23;
	const GEN_FLT x245 = x114 * (x23 * x234 + x235 + x244 * x52) + x115 * (-x23 * x236 + x237 - x244 * x58);
	const GEN_FLT x246 = -x111 * x243 + x245;
	const GEN_FLT x247 =
		x245 - x97 * (x124 * (x120 * x246 -
							  x122 * (x239 * x79 + x239 * x89 + x242 * x74 +
									  x74 * (x239 * x88 + x242 +
											 x74 * (x239 * x87 + x241 + x74 * (-x121 * x239 + x239 * x86 + x240))))) +
					  x125 * x246 + x126 * x239 + x242 * x95 + x243);
	const GEN_FLT x248 = obj_qk * x155;
	const GEN_FLT x249 = sensor_y * x248;
	const GEN_FLT x250 = sensor_z * x248;
	const GEN_FLT x251 = sensor_x * x248;
	const GEN_FLT x252 = x187 + x219 + x249 * x31 - x250 * x34 + x251 * x32;
	const GEN_FLT x253 = x252 * x28;
	const GEN_FLT x254 = -obj_qk * x196;
	const GEN_FLT x255 = sensor_x * (-x248 * x9 + x254) + x15 * x249 + x166 - x195 + x250 * x5;
	const GEN_FLT x256 = x2 * x255;
	const GEN_FLT x257 = sensor_y * (-x248 * x42 + x254) + x153 + x229 + x250 * x41 + x251 * x40;
	const GEN_FLT x258 = x257 * x39;
	const GEN_FLT x259 = x23 * x253 + x23 * x256 + x258;
	const GEN_FLT x260 = x252 * x50;
	const GEN_FLT x261 = x101 * x257;
	const GEN_FLT x262 = x255 * x55;
	const GEN_FLT x263 = x149 * x252;
	const GEN_FLT x264 = x255 * x60;
	const GEN_FLT x265 = -x102 * (x101 * x260 + x261 * x52 + 2 * x262) - x142 * (-x101 * x264 - x261 * x58 + 2 * x263);
	const GEN_FLT x266 = x98 * (x105 * (-x129 * (x101 * x253 + x101 * x256 + 2 * x258) + x265) + x259 * x73);
	const GEN_FLT x267 = x266 * x76;
	const GEN_FLT x268 = x266 * x77 + x74 * (-x266 * x75 + x267);
	const GEN_FLT x269 = x266 * x78 + x268 * x74;
	const GEN_FLT x270 = x112 * x265 + x259 * x67;
	const GEN_FLT x271 = x23 * x257;
	const GEN_FLT x272 = x114 * (x23 * x260 + x262 + x271 * x52) + x115 * (-x23 * x264 + x263 - x271 * x58);
	const GEN_FLT x273 = -x111 * x270 + x272;
	const GEN_FLT x274 =
		x272 - x97 * (x124 * (x120 * x273 -
							  x122 * (x266 * x79 + x266 * x89 + x269 * x74 +
									  x74 * (x266 * x88 + x269 +
											 x74 * (x266 * x87 + x268 + x74 * (-x121 * x266 + x266 * x86 + x267))))) +
					  x125 * x273 + x126 * x266 + x269 * x95 + x270);
	const GEN_FLT x275 = -lh_px - x51 - x54 - x56;
	const GEN_FLT x276 = x112 * x275;
	const GEN_FLT x277 = x105 * x98;
	const GEN_FLT x278 = x275 * x277;
	const GEN_FLT x279 = -x111 * x276 + x114;
	const GEN_FLT x280 = x278 * x76;
	const GEN_FLT x281 = x278 * x77 + x74 * (-x278 * x75 + x280);
	const GEN_FLT x282 = x278 * x78 + x281 * x74;
	const GEN_FLT x283 =
		x114 - x97 * (x124 * (x120 * x279 -
							  x122 * (x278 * x79 + x278 * x89 + x282 * x74 +
									  x74 * (x278 * x88 + x282 +
											 x74 * (x278 * x87 + x281 + x74 * (-x121 * x278 + x278 * x86 + x280))))) +
					  x125 * x279 + x126 * x278 + x276 + x282 * x95);
	const GEN_FLT x284 = x111 * x67;
	const GEN_FLT x285 = x98 * (x105 * (-lh_py - x25 - x37 - x44) + x73);
	const GEN_FLT x286 = x285 * x76;
	const GEN_FLT x287 = x285 * x77 + x74 * (-x285 * x75 + x286);
	const GEN_FLT x288 = x285 * x78 + x287 * x74;
	const GEN_FLT x289 =
		x97 *
		(x124 * (-x120 * x284 - x122 * (x285 * x79 + x285 * x89 + x288 * x74 +
										x74 * (x285 * x88 + x288 +
											   x74 * (x285 * x87 + x287 + x74 * (-x121 * x285 + x285 * x86 + x286))))) -
		 x125 * x284 + x126 * x285 + x288 * x95 + x67);
	const GEN_FLT x290 = -x115;
	const GEN_FLT x291 = x112 * x65;
	const GEN_FLT x292 = x277 * x65;
	const GEN_FLT x293 = -x111 * x291 + x290;
	const GEN_FLT x294 = x292 * x76;
	const GEN_FLT x295 = x292 * x77 + x74 * (-x292 * x75 + x294);
	const GEN_FLT x296 = x292 * x78 + x295 * x74;
	const GEN_FLT x297 =
		x290 - x97 * (x124 * (x120 * x293 -
							  x122 * (x292 * x79 + x292 * x89 + x296 * x74 +
									  x74 * (x292 * x88 + x296 +
											 x74 * (x292 * x87 + x295 + x74 * (-x121 * x292 + x292 * x86 + x294))))) +
					  x125 * x293 + x126 * x292 + x291 + x296 * x95);
	const GEN_FLT x298 = lh_qi * x36;
	const GEN_FLT x299 = lh_qk * x24;
	const GEN_FLT x300 = 1.0 / x22;
	const GEN_FLT x301 = 2 * x300;
	const GEN_FLT x302 = lh_qw * x301;
	const GEN_FLT x303 = x38 * x43;
	const GEN_FLT x304 = x17 * x2;
	const GEN_FLT x305 = x28 * x35;
	const GEN_FLT x306 = -x298 + x299 - x302 * x303 + x302 * x304 + x302 * x305;
	const GEN_FLT x307 = lh_qi * x101;
	const GEN_FLT x308 = x307 * x35;
	const GEN_FLT x309 = lh_qk * x101;
	const GEN_FLT x310 = x17 * x309;
	const GEN_FLT x311 = 4 * x300;
	const GEN_FLT x312 = lh_qw * x311;
	const GEN_FLT x313 = lh_qj * x101;
	const GEN_FLT x314 = x313 * x35;
	const GEN_FLT x315 = -x309 * x43;
	const GEN_FLT x316 = x17 * x21;
	const GEN_FLT x317 = x312 * x35;
	const GEN_FLT x318 = x312 * x43;
	const GEN_FLT x319 = x307 * x43;
	const GEN_FLT x320 = x17 * x313;
	const GEN_FLT x321 = x17 * x60;
	const GEN_FLT x322 = -x102 * (-x312 * x316 + x314 + x315 + x317 * x50 + x318 * x52) -
						 x142 * (-x312 * x321 + x317 * x62 - x318 * x58 - x319 + x320);
	const GEN_FLT x323 =
		x98 * (x105 * (-x129 * (-x303 * x312 + x304 * x312 + x305 * x312 - x308 + x310) + x322) + x306 * x73);
	const GEN_FLT x324 = x323 * x76;
	const GEN_FLT x325 = x323 * x77 + x74 * (-x323 * x75 + x324);
	const GEN_FLT x326 = x323 * x78 + x325 * x74;
	const GEN_FLT x327 = x112 * x322 + x306 * x67;
	const GEN_FLT x328 = lh_qi * x53;
	const GEN_FLT x329 = lh_qj * x24;
	const GEN_FLT x330 = x302 * x35;
	const GEN_FLT x331 = x302 * x43;
	const GEN_FLT x332 = lh_qj * x36;
	const GEN_FLT x333 = -lh_qk * x53;
	const GEN_FLT x334 = x114 * (-x302 * x316 + x330 * x50 + x331 * x52 + x332 + x333) +
						 x115 * (-x302 * x321 - x328 + x329 + x330 * x62 - x331 * x58);
	const GEN_FLT x335 = -x111 * x327 + x334;
	const GEN_FLT x336 =
		x334 - x97 * (x124 * (x120 * x335 -
							  x122 * (x323 * x79 + x323 * x89 + x326 * x74 +
									  x74 * (x323 * x88 + x326 +
											 x74 * (x323 * x87 + x325 + x74 * (-x121 * x323 + x323 * x86 + x324))))) +
					  x125 * x335 + x126 * x323 + x326 * x95 + x327);
	const GEN_FLT x337 = lh_qw * x36;
	const GEN_FLT x338 = lh_qi * x301;
	const GEN_FLT x339 = x43 * (-x307 - x338 * x38);
	const GEN_FLT x340 = x304 * x338 + x305 * x338 + x329 - x337 + x339;
	const GEN_FLT x341 = lh_qw * x101;
	const GEN_FLT x342 = x341 * x35;
	const GEN_FLT x343 = lh_qi * x311;
	const GEN_FLT x344 = x313 * x43;
	const GEN_FLT x345 = x309 * x35;
	const GEN_FLT x346 = x35 * x50;
	const GEN_FLT x347 = x343 * x43;
	const GEN_FLT x348 = -x341 * x43;
	const GEN_FLT x349 = x35 * (x307 + x338 * x62);
	const GEN_FLT x350 = -x102 * (-x316 * x343 + x343 * x346 + x344 + x345 + x347 * x52) -
						 x142 * (-x310 - x321 * x343 - x347 * x58 + x348 + 2 * x349);
	const GEN_FLT x351 =
		x98 * (x105 * (-x129 * (x304 * x343 + x305 * x343 + x320 + 2 * x339 - x342) + x350) + x340 * x73);
	const GEN_FLT x352 = x351 * x76;
	const GEN_FLT x353 = x351 * x77 + x74 * (-x351 * x75 + x352);
	const GEN_FLT x354 = x351 * x78 + x353 * x74;
	const GEN_FLT x355 = x112 * x350 + x340 * x67;
	const GEN_FLT x356 = -lh_qw * x53;
	const GEN_FLT x357 = x338 * x43;
	const GEN_FLT x358 = lh_qj * x53;
	const GEN_FLT x359 = lh_qk * x36;
	const GEN_FLT x360 = x114 * (-x316 * x338 + x338 * x346 + x357 * x52 + x358 + x359) +
						 x115 * (-x299 - x321 * x338 + x349 + x356 - x357 * x58);
	const GEN_FLT x361 = -x111 * x355 + x360;
	const GEN_FLT x362 =
		x360 - x97 * (x124 * (x120 * x361 -
							  x122 * (x351 * x79 + x351 * x89 + x354 * x74 +
									  x74 * (x351 * x88 + x354 +
											 x74 * (x351 * x87 + x353 + x74 * (-x121 * x351 + x351 * x86 + x352))))) +
					  x125 * x361 + x126 * x351 + x354 * x95 + x355);
	const GEN_FLT x363 = lh_qi * x24;
	const GEN_FLT x364 = lh_qj * x301;
	const GEN_FLT x365 = -x303 * x364 + x304 * x364 + x305 * x364 + x359 + x363;
	const GEN_FLT x366 = x17 * x307;
	const GEN_FLT x367 = lh_qj * x311;
	const GEN_FLT x368 = x43 * x52;
	const GEN_FLT x369 = x17 * (-x21 * x364 - x313);
	const GEN_FLT x370 = x17 * x341;
	const GEN_FLT x371 = x43 * x58;
	const GEN_FLT x372 = x35 * (x313 + x364 * x62);
	const GEN_FLT x373 = -x102 * (x319 + x342 + x346 * x367 + x367 * x368 + 2 * x369) -
						 x142 * (x315 - x321 * x367 - x367 * x371 + x370 + 2 * x372);
	const GEN_FLT x374 =
		x98 * (x105 * (-x129 * (-x303 * x367 + x304 * x367 + x305 * x367 + x345 + x366) + x373) + x365 * x73);
	const GEN_FLT x375 = x374 * x76;
	const GEN_FLT x376 = x374 * x77 + x74 * (-x374 * x75 + x375);
	const GEN_FLT x377 = x374 * x78 + x376 * x74;
	const GEN_FLT x378 = x112 * x373 + x365 * x67;
	const GEN_FLT x379 = lh_qw * x24;
	const GEN_FLT x380 = x114 * (x328 + x337 + x346 * x364 + x364 * x368 + x369) +
						 x115 * (-x321 * x364 + x333 - x364 * x371 + x372 + x379);
	const GEN_FLT x381 = -x111 * x378 + x380;
	const GEN_FLT x382 =
		x380 - x97 * (x124 * (x120 * x381 -
							  x122 * (x374 * x79 + x374 * x89 + x377 * x74 +
									  x74 * (x374 * x88 + x377 +
											 x74 * (x374 * x87 + x376 + x74 * (-x121 * x374 + x374 * x86 + x375))))) +
					  x125 * x381 + x126 * x374 + x377 * x95 + x378);
	const GEN_FLT x383 = lh_qk * x301;
	const GEN_FLT x384 = -x309;
	const GEN_FLT x385 = x43 * (-x38 * x383 + x384);
	const GEN_FLT x386 = x304 * x383 + x305 * x383 + x332 + x379 + x385;
	const GEN_FLT x387 = lh_qk * x311;
	const GEN_FLT x388 = x35 * x62;
	const GEN_FLT x389 = x17 * (-x21 * x383 + x384);
	const GEN_FLT x390 = -x102 * (x308 + x346 * x387 + x348 + x368 * x387 + 2 * x389) -
						 x142 * (-x321 * x387 - x344 - x366 - x371 * x387 + x387 * x388);
	const GEN_FLT x391 =
		x98 * (x105 * (-x129 * (x304 * x387 + x305 * x387 + x314 + x370 + 2 * x385) + x390) + x386 * x73);
	const GEN_FLT x392 = x391 * x76;
	const GEN_FLT x393 = x391 * x77 + x74 * (-x391 * x75 + x392);
	const GEN_FLT x394 = x391 * x78 + x393 * x74;
	const GEN_FLT x395 = x112 * x390 + x386 * x67;
	const GEN_FLT x396 = x114 * (x298 + x346 * x383 + x356 + x368 * x383 + x389) +
						 x115 * (-x321 * x383 - x358 - x363 - x371 * x383 + x383 * x388);
	const GEN_FLT x397 = -x111 * x395 + x396;
	const GEN_FLT x398 =
		x396 - x97 * (x124 * (x120 * x397 -
							  x122 * (x391 * x79 + x391 * x89 + x394 * x74 +
									  x74 * (x391 * x88 + x394 +
											 x74 * (x391 * x87 + x393 + x74 * (-x121 * x391 + x391 * x86 + x392))))) +
					  x125 * x397 + x126 * x391 + x394 * x95 + x395);
	*(out++) = x127 * x128 + x127;
	*(out++) = x128 * x140 + x140;
	*(out++) = x128 * x152 + x152;
	*(out++) = x128 * x186 + x186;
	*(out++) = x128 * x218 + x218;
	*(out++) = x128 * x247 + x247;
	*(out++) = x128 * x274 + x274;
	*(out++) = x128 * x283 + x283;
	*(out++) = -x128 * x289 - x289;
	*(out++) = x128 * x297 + x297;
	*(out++) = x128 * x336 + x336;
	*(out++) = x128 * x362 + x362;
	*(out++) = x128 * x382 + x382;
	*(out++) = x128 * x398 + x398;
}

static inline void gen_reproject_axis_y_jac_all_gen2(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
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
	const GEN_FLT x25 = x2 * x24;
	const GEN_FLT x26 = lh_qj * lh_qk;
	const GEN_FLT x27 = lh_qi * lh_qw;
	const GEN_FLT x28 = x26 - x27;
	const GEN_FLT x29 = obj_qi * obj_qw;
	const GEN_FLT x30 = obj_qj * obj_qk;
	const GEN_FLT x31 = x29 + x30;
	const GEN_FLT x32 = x3 - x4;
	const GEN_FLT x33 = sensor_x * x11;
	const GEN_FLT x34 = x6 + x7;
	const GEN_FLT x35 = obj_pz + sensor_z * (-x11 * x34 + 1) + x16 * x31 + x32 * x33;
	const GEN_FLT x36 = x23 * x35;
	const GEN_FLT x37 = x28 * x36;
	const GEN_FLT x38 = x18 + x20;
	const GEN_FLT x39 = -x23 * x38 + 1;
	const GEN_FLT x40 = x13 + x14;
	const GEN_FLT x41 = -x29 + x30;
	const GEN_FLT x42 = x6 + x8;
	const GEN_FLT x43 = obj_py + sensor_y * (-x11 * x42 + 1) + x12 * x41 + x33 * x40;
	const GEN_FLT x44 = x39 * x43;
	const GEN_FLT x45 = lh_py + x25 + x37 + x44;
	const GEN_FLT x46 = tilt_0 - 0.52359877559829882;
	const GEN_FLT x47 = tan(x46);
	const GEN_FLT x48 = lh_qi * lh_qk;
	const GEN_FLT x49 = lh_qj * lh_qw;
	const GEN_FLT x50 = x48 + x49;
	const GEN_FLT x51 = x36 * x50;
	const GEN_FLT x52 = x0 - x1;
	const GEN_FLT x53 = x23 * x43;
	const GEN_FLT x54 = x52 * x53;
	const GEN_FLT x55 = -x21 * x23 + 1;
	const GEN_FLT x56 = x17 * x55;
	const GEN_FLT x57 = lh_px + x51 + x54 + x56;
	const GEN_FLT x58 = x26 + x27;
	const GEN_FLT x59 = x53 * x58;
	const GEN_FLT x60 = x48 - x49;
	const GEN_FLT x61 = x24 * x60;
	const GEN_FLT x62 = x18 + x19;
	const GEN_FLT x63 = x23 * x62;
	const GEN_FLT x64 = x35 * (1 - x63);
	const GEN_FLT x65 = -lh_pz - x59 - x61 - x64;
	const GEN_FLT x66 = x57 * x57 + x65 * x65;
	const GEN_FLT x67 = x47 / sqrt(x66);
	const GEN_FLT x68 = x45 * x67;
	const GEN_FLT x69 = cos(x46);
	const GEN_FLT x70 = 1.0 / x69;
	const GEN_FLT x71 = x45 * x45;
	const GEN_FLT x72 = x66 + x71;
	const GEN_FLT x73 = x70 / sqrt(x72);
	const GEN_FLT x74 = asin(x45 * x73);
	const GEN_FLT x75 = 8.0108022e-6 * x74;
	const GEN_FLT x76 = -x75 - 8.0108022e-6;
	const GEN_FLT x77 = x74 * x76 + 0.0028679863;
	const GEN_FLT x78 = x74 * x77 + 5.3685255000000001e-6;
	const GEN_FLT x79 = x74 * x78 + 0.0076069798000000001;
	const GEN_FLT x80 = x74 * x74;
	const GEN_FLT x81 = atan2(x65, x57);
	const GEN_FLT x82 = ogeePhase_0 + x81 - asin(x68);
	const GEN_FLT x83 = ogeeMag_0 * sin(x82);
	const GEN_FLT x84 = curve_0 + x83;
	const GEN_FLT x85 = x74 * x79;
	const GEN_FLT x86 = -1.60216044e-5 * x74 - 8.0108022e-6;
	const GEN_FLT x87 = x74 * x86 + x77;
	const GEN_FLT x88 = x74 * x87 + x78;
	const GEN_FLT x89 = x74 * x88 + x79;
	const GEN_FLT x90 = sin(x46);
	const GEN_FLT x91 = x90 * (x74 * x89 + x85);
	const GEN_FLT x92 = x69 - x84 * x91;
	const GEN_FLT x93 = 1.0 / x92;
	const GEN_FLT x94 = x84 * x93;
	const GEN_FLT x95 = x80 * x94;
	const GEN_FLT x96 = x68 + x79 * x95;
	const GEN_FLT x97 = pow(1 - x96 * x96, -1.0 / 2.0);
	const GEN_FLT x98 = pow(-x71 / (x72 * (x69 * x69)) + 1, -1.0 / 2.0);
	const GEN_FLT x99 = x2 * x23;
	const GEN_FLT x100 = x23 * x45;
	const GEN_FLT x101 = 4 * x22;
	const GEN_FLT x102 = (1.0 / 2.0) * x57;
	const GEN_FLT x103 = x23 * x65;
	const GEN_FLT x104 = -x102 * (-x101 * x21 + 2) + x103 * x60;
	const GEN_FLT x105 = x45 * x70 / pow(x72, 3.0 / 2.0);
	const GEN_FLT x106 = x98 * (x105 * (-x100 * x2 + x104) + x73 * x99);
	const GEN_FLT x107 = x106 * x76;
	const GEN_FLT x108 = x106 * x77 + x74 * (-x106 * x75 + x107);
	const GEN_FLT x109 = x106 * x78 + x108 * x74;
	const GEN_FLT x110 = 1.0 / x66;
	const GEN_FLT x111 = pow(-x110 * x71 * x47 * x47 + 1, -1.0 / 2.0);
	const GEN_FLT x112 = x45 * x47 / pow(x66, 3.0 / 2.0);
	const GEN_FLT x113 = x104 * x112 + x67 * x99;
	const GEN_FLT x114 = x110 * (lh_pz + x59 + x61 + x64);
	const GEN_FLT x115 = x110 * x57;
	const GEN_FLT x116 = x115 * x23;
	const GEN_FLT x117 = x114 * x55 - x116 * x60;
	const GEN_FLT x118 = -x111 * x113 + x117;
	const GEN_FLT x119 = ogeeMag_0 * cos(x82);
	const GEN_FLT x120 = x119 * x91;
	const GEN_FLT x121 = 2.40324066e-5 * x74;
	const GEN_FLT x122 = x90 * (-curve_0 - x83);
	const GEN_FLT x123 = x79 * x80;
	const GEN_FLT x124 = x123 * x84 / ((x92 * x92));
	const GEN_FLT x125 = x119 * x123 * x93;
	const GEN_FLT x126 = 2 * x85 * x94;
	const GEN_FLT x127 =
		x117 - x97 * (x106 * x126 + x109 * x95 + x113 + x118 * x125 +
					  x124 * (x118 * x120 -
							  x122 * (x106 * x79 + x106 * x89 + x109 * x74 +
									  x74 * (x106 * x88 + x109 +
											 x74 * (x106 * x87 + x108 + x74 * (-x106 * x121 + x106 * x86 + x107))))));
	const GEN_FLT x128 = gibMag_0 * cos(gibPhase_0 + x81 - asin(x96));
	const GEN_FLT x129 = (1.0 / 2.0) * x45;
	const GEN_FLT x130 = x23 * x57;
	const GEN_FLT x131 = x103 * x58 - x130 * x52;
	const GEN_FLT x132 = x98 * (x105 * (-x129 * (-x101 * x38 + 2) + x131) + x39 * x73);
	const GEN_FLT x133 = x132 * x76;
	const GEN_FLT x134 = x132 * x77 + x74 * (-x132 * x75 + x133);
	const GEN_FLT x135 = x132 * x78 + x134 * x74;
	const GEN_FLT x136 = x112 * x131 + x39 * x67;
	const GEN_FLT x137 = x114 * x23;
	const GEN_FLT x138 = -x116 * x58 + x137 * x52;
	const GEN_FLT x139 = -x111 * x136 + x138;
	const GEN_FLT x140 =
		x138 - x97 * (x124 * (x120 * x139 -
							  x122 * (x132 * x79 + x132 * x89 + x135 * x74 +
									  x74 * (x132 * x88 + x135 +
											 x74 * (x132 * x87 + x134 + x74 * (-x121 * x132 + x132 * x86 + x133))))) +
					  x125 * x139 + x126 * x132 + x135 * x95 + x136);
	const GEN_FLT x141 = x23 * x28;
	const GEN_FLT x142 = (1.0 / 2.0) * x65;
	const GEN_FLT x143 = -x130 * x50 - x142 * (x101 * x62 - 2);
	const GEN_FLT x144 = x98 * (x105 * (-x100 * x28 + x143) + x141 * x73);
	const GEN_FLT x145 = x144 * x76;
	const GEN_FLT x146 = x144 * x77 + x74 * (-x144 * x75 + x145);
	const GEN_FLT x147 = x144 * x78 + x146 * x74;
	const GEN_FLT x148 = x112 * x143 + x141 * x67;
	const GEN_FLT x149 = x63 - 1;
	const GEN_FLT x150 = x115 * x149 + x137 * x50;
	const GEN_FLT x151 = -x111 * x148 + x150;
	const GEN_FLT x152 =
		x150 - x97 * (x124 * (x120 * x151 -
							  x122 * (x144 * x79 + x144 * x89 + x147 * x74 +
									  x74 * (x144 * x88 + x147 +
											 x74 * (x144 * x87 + x146 + x74 * (-x121 * x144 + x144 * x86 + x145))))) +
					  x125 * x151 + x126 * x144 + x147 * x95 + x148);
	const GEN_FLT x153 = obj_qj * x12;
	const GEN_FLT x154 = obj_qk * x16;
	const GEN_FLT x155 = 2 / x10;
	const GEN_FLT x156 = obj_qw * x155;
	const GEN_FLT x157 = sensor_x * x156;
	const GEN_FLT x158 = sensor_z * x156;
	const GEN_FLT x159 = sensor_y * x156;
	const GEN_FLT x160 = x15 * x159 + x153 - x154 - x157 * x9 + x158 * x5;
	const GEN_FLT x161 = x160 * x2;
	const GEN_FLT x162 = obj_qi * x16;
	const GEN_FLT x163 = obj_qj * x33;
	const GEN_FLT x164 = x157 * x32 - x158 * x34 + x159 * x31 + x162 - x163;
	const GEN_FLT x165 = x164 * x28;
	const GEN_FLT x166 = obj_qi * x12;
	const GEN_FLT x167 = obj_qk * x33;
	const GEN_FLT x168 = sensor_y * x42;
	const GEN_FLT x169 = -x156 * x168 + x157 * x40 + x158 * x41 - x166 + x167;
	const GEN_FLT x170 = x169 * x39;
	const GEN_FLT x171 = x161 * x23 + x165 * x23 + x170;
	const GEN_FLT x172 = x164 * x50;
	const GEN_FLT x173 = x101 * x169;
	const GEN_FLT x174 = x160 * x55;
	const GEN_FLT x175 = x160 * x60;
	const GEN_FLT x176 = x149 * x164;
	const GEN_FLT x177 = -x102 * (x101 * x172 + x173 * x52 + 2 * x174) - x142 * (-x101 * x175 - x173 * x58 + 2 * x176);
	const GEN_FLT x178 = x98 * (x105 * (-x129 * (x101 * x161 + x101 * x165 + 2 * x170) + x177) + x171 * x73);
	const GEN_FLT x179 = x178 * x76;
	const GEN_FLT x180 = x178 * x77 + x74 * (-x178 * x75 + x179);
	const GEN_FLT x181 = x178 * x78 + x180 * x74;
	const GEN_FLT x182 = x112 * x177 + x171 * x67;
	const GEN_FLT x183 = x169 * x23;
	const GEN_FLT x184 = x114 * (x172 * x23 + x174 + x183 * x52) + x115 * (-x175 * x23 + x176 - x183 * x58);
	const GEN_FLT x185 = -x111 * x182 + x184;
	const GEN_FLT x186 =
		x184 - x97 * (x124 * (x120 * x185 -
							  x122 * (x178 * x79 + x178 * x89 + x181 * x74 +
									  x74 * (x178 * x88 + x181 +
											 x74 * (x178 * x87 + x180 + x74 * (-x121 * x178 + x178 * x86 + x179))))) +
					  x125 * x185 + x126 * x178 + x181 * x95 + x182);
	const GEN_FLT x187 = obj_qj * x16;
	const GEN_FLT x188 = obj_qk * x12;
	const GEN_FLT x189 = obj_qi * x155;
	const GEN_FLT x190 = sensor_x * x189;
	const GEN_FLT x191 = sensor_z * x189;
	const GEN_FLT x192 = sensor_y * x189;
	const GEN_FLT x193 = x15 * x192 + x187 + x188 - x190 * x9 + x191 * x5;
	const GEN_FLT x194 = x193 * x2;
	const GEN_FLT x195 = obj_qw * x16;
	const GEN_FLT x196 = 4 * x10;
	const GEN_FLT x197 = -obj_qi * x196;
	const GEN_FLT x198 = sensor_z * (-x189 * x34 + x197) + x167 + x190 * x32 + x192 * x31 + x195;
	const GEN_FLT x199 = x198 * x28;
	const GEN_FLT x200 = obj_qw * x12;
	const GEN_FLT x201 = sensor_y * (-x189 * x42 + x197) + x163 + x190 * x40 + x191 * x41 - x200;
	const GEN_FLT x202 = x201 * x39;
	const GEN_FLT x203 = x194 * x23 + x199 * x23 + x202;
	const GEN_FLT x204 = x193 * x55;
	const GEN_FLT x205 = x198 * x50;
	const GEN_FLT x206 = x101 * x201;
	const GEN_FLT x207 = x193 * x60;
	const GEN_FLT x208 = x149 * x198;
	const GEN_FLT x209 = -x102 * (x101 * x205 + 2 * x204 + x206 * x52) - x142 * (-x101 * x207 - x206 * x58 + 2 * x208);
	const GEN_FLT x210 = x98 * (x105 * (-x129 * (x101 * x194 + x101 * x199 + 2 * x202) + x209) + x203 * x73);
	const GEN_FLT x211 = x210 * x76;
	const GEN_FLT x212 = x210 * x77 + x74 * (-x210 * x75 + x211);
	const GEN_FLT x213 = x210 * x78 + x212 * x74;
	const GEN_FLT x214 = x112 * x209 + x203 * x67;
	const GEN_FLT x215 = x201 * x23;
	const GEN_FLT x216 = x114 * (x204 + x205 * x23 + x215 * x52) + x115 * (-x207 * x23 + x208 - x215 * x58);
	const GEN_FLT x217 = -x111 * x214 + x216;
	const GEN_FLT x218 =
		x216 - x97 * (x124 * (x120 * x217 -
							  x122 * (x210 * x79 + x210 * x89 + x213 * x74 +
									  x74 * (x210 * x88 + x213 +
											 x74 * (x210 * x87 + x212 + x74 * (-x121 * x210 + x210 * x86 + x211))))) +
					  x125 * x217 + x126 * x210 + x213 * x95 + x214);
	const GEN_FLT x219 = obj_qi * x33;
	const GEN_FLT x220 = obj_qj * x155;
	const GEN_FLT x221 = sensor_x * x220;
	const GEN_FLT x222 = sensor_z * x220;
	const GEN_FLT x223 = -x168 * x220 + x188 + x219 + x221 * x40 + x222 * x41;
	const GEN_FLT x224 = x223 * x39;
	const GEN_FLT x225 = sensor_y * x220;
	const GEN_FLT x226 = -obj_qj * x196;
	const GEN_FLT x227 = sensor_x * (-x220 * x9 + x226) + x15 * x225 + x162 + x200 + x222 * x5;
	const GEN_FLT x228 = x2 * x227;
	const GEN_FLT x229 = obj_qw * x33;
	const GEN_FLT x230 = sensor_z * (-x220 * x34 + x226) + x154 + x221 * x32 + x225 * x31 - x229;
	const GEN_FLT x231 = x230 * x28;
	const GEN_FLT x232 = x224 + x228 * x23 + x23 * x231;
	const GEN_FLT x233 = x101 * x223;
	const GEN_FLT x234 = x230 * x50;
	const GEN_FLT x235 = x227 * x55;
	const GEN_FLT x236 = x227 * x60;
	const GEN_FLT x237 = x149 * x230;
	const GEN_FLT x238 = -x102 * (x101 * x234 + x233 * x52 + 2 * x235) - x142 * (-x101 * x236 - x233 * x58 + 2 * x237);
	const GEN_FLT x239 = x98 * (x105 * (-x129 * (x101 * x228 + x101 * x231 + 2 * x224) + x238) + x232 * x73);
	const GEN_FLT x240 = x239 * x76;
	const GEN_FLT x241 = x239 * x77 + x74 * (-x239 * x75 + x240);
	const GEN_FLT x242 = x239 * x78 + x241 * x74;
	const GEN_FLT x243 = x112 * x238 + x232 * x67;
	const GEN_FLT x244 = x223 * x23;
	const GEN_FLT x245 = x114 * (x23 * x234 + x235 + x244 * x52) + x115 * (-x23 * x236 + x237 - x244 * x58);
	const GEN_FLT x246 = -x111 * x243 + x245;
	const GEN_FLT x247 =
		x245 - x97 * (x124 * (x120 * x246 -
							  x122 * (x239 * x79 + x239 * x89 + x242 * x74 +
									  x74 * (x239 * x88 + x242 +
											 x74 * (x239 * x87 + x241 + x74 * (-x121 * x239 + x239 * x86 + x240))))) +
					  x125 * x246 + x126 * x239 + x242 * x95 + x243);
	const GEN_FLT x248 = obj_qk * x155;
	const GEN_FLT x249 = sensor_y * x248;
	const GEN_FLT x250 = sensor_z * x248;
	const GEN_FLT x251 = sensor_x * x248;
	const GEN_FLT x252 = x187 + x219 + x249 * x31 - x250 * x34 + x251 * x32;
	const GEN_FLT x253 = x252 * x28;
	const GEN_FLT x254 = -obj_qk * x196;
	const GEN_FLT x255 = sensor_x * (-x248 * x9 + x254) + x15 * x249 + x166 - x195 + x250 * x5;
	const GEN_FLT x256 = x2 * x255;
	const GEN_FLT x257 = sensor_y * (-x248 * x42 + x254) + x153 + x229 + x250 * x41 + x251 * x40;
	const GEN_FLT x258 = x257 * x39;
	const GEN_FLT x259 = x23 * x253 + x23 * x256 + x258;
	const GEN_FLT x260 = x252 * x50;
	const GEN_FLT x261 = x101 * x257;
	const GEN_FLT x262 = x255 * x55;
	const GEN_FLT x263 = x149 * x252;
	const GEN_FLT x264 = x255 * x60;
	const GEN_FLT x265 = -x102 * (x101 * x260 + x261 * x52 + 2 * x262) - x142 * (-x101 * x264 - x261 * x58 + 2 * x263);
	const GEN_FLT x266 = x98 * (x105 * (-x129 * (x101 * x253 + x101 * x256 + 2 * x258) + x265) + x259 * x73);
	const GEN_FLT x267 = x266 * x76;
	const GEN_FLT x268 = x266 * x77 + x74 * (-x266 * x75 + x267);
	const GEN_FLT x269 = x266 * x78 + x268 * x74;
	const GEN_FLT x270 = x112 * x265 + x259 * x67;
	const GEN_FLT x271 = x23 * x257;
	const GEN_FLT x272 = x114 * (x23 * x260 + x262 + x271 * x52) + x115 * (-x23 * x264 + x263 - x271 * x58);
	const GEN_FLT x273 = -x111 * x270 + x272;
	const GEN_FLT x274 =
		x272 - x97 * (x124 * (x120 * x273 -
							  x122 * (x266 * x79 + x266 * x89 + x269 * x74 +
									  x74 * (x266 * x88 + x269 +
											 x74 * (x266 * x87 + x268 + x74 * (-x121 * x266 + x266 * x86 + x267))))) +
					  x125 * x273 + x126 * x266 + x269 * x95 + x270);
	const GEN_FLT x275 = -lh_px - x51 - x54 - x56;
	const GEN_FLT x276 = x112 * x275;
	const GEN_FLT x277 = x105 * x98;
	const GEN_FLT x278 = x275 * x277;
	const GEN_FLT x279 = -x111 * x276 + x114;
	const GEN_FLT x280 = x278 * x76;
	const GEN_FLT x281 = x278 * x77 + x74 * (-x278 * x75 + x280);
	const GEN_FLT x282 = x278 * x78 + x281 * x74;
	const GEN_FLT x283 =
		x114 - x97 * (x124 * (x120 * x279 -
							  x122 * (x278 * x79 + x278 * x89 + x282 * x74 +
									  x74 * (x278 * x88 + x282 +
											 x74 * (x278 * x87 + x281 + x74 * (-x121 * x278 + x278 * x86 + x280))))) +
					  x125 * x279 + x126 * x278 + x276 + x282 * x95);
	const GEN_FLT x284 = x111 * x67;
	const GEN_FLT x285 = x98 * (x105 * (-lh_py - x25 - x37 - x44) + x73);
	const GEN_FLT x286 = x285 * x76;
	const GEN_FLT x287 = x285 * x77 + x74 * (-x285 * x75 + x286);
	const GEN_FLT x288 = x285 * x78 + x287 * x74;
	const GEN_FLT x289 =
		x97 *
		(x124 * (-x120 * x284 - x122 * (x285 * x79 + x285 * x89 + x288 * x74 +
										x74 * (x285 * x88 + x288 +
											   x74 * (x285 * x87 + x287 + x74 * (-x121 * x285 + x285 * x86 + x286))))) -
		 x125 * x284 + x126 * x285 + x288 * x95 + x67);
	const GEN_FLT x290 = -x115;
	const GEN_FLT x291 = x112 * x65;
	const GEN_FLT x292 = x277 * x65;
	const GEN_FLT x293 = -x111 * x291 + x290;
	const GEN_FLT x294 = x292 * x76;
	const GEN_FLT x295 = x292 * x77 + x74 * (-x292 * x75 + x294);
	const GEN_FLT x296 = x292 * x78 + x295 * x74;
	const GEN_FLT x297 =
		x290 - x97 * (x124 * (x120 * x293 -
							  x122 * (x292 * x79 + x292 * x89 + x296 * x74 +
									  x74 * (x292 * x88 + x296 +
											 x74 * (x292 * x87 + x295 + x74 * (-x121 * x292 + x292 * x86 + x294))))) +
					  x125 * x293 + x126 * x292 + x291 + x296 * x95);
	const GEN_FLT x298 = lh_qi * x36;
	const GEN_FLT x299 = lh_qk * x24;
	const GEN_FLT x300 = 1.0 / x22;
	const GEN_FLT x301 = 2 * x300;
	const GEN_FLT x302 = lh_qw * x301;
	const GEN_FLT x303 = x38 * x43;
	const GEN_FLT x304 = x17 * x2;
	const GEN_FLT x305 = x28 * x35;
	const GEN_FLT x306 = -x298 + x299 - x302 * x303 + x302 * x304 + x302 * x305;
	const GEN_FLT x307 = lh_qi * x101;
	const GEN_FLT x308 = x307 * x35;
	const GEN_FLT x309 = lh_qk * x101;
	const GEN_FLT x310 = x17 * x309;
	const GEN_FLT x311 = 4 * x300;
	const GEN_FLT x312 = lh_qw * x311;
	const GEN_FLT x313 = lh_qj * x101;
	const GEN_FLT x314 = x313 * x35;
	const GEN_FLT x315 = -x309 * x43;
	const GEN_FLT x316 = x17 * x21;
	const GEN_FLT x317 = x312 * x35;
	const GEN_FLT x318 = x312 * x43;
	const GEN_FLT x319 = x307 * x43;
	const GEN_FLT x320 = x17 * x313;
	const GEN_FLT x321 = x17 * x60;
	const GEN_FLT x322 = -x102 * (-x312 * x316 + x314 + x315 + x317 * x50 + x318 * x52) -
						 x142 * (-x312 * x321 + x317 * x62 - x318 * x58 - x319 + x320);
	const GEN_FLT x323 =
		x98 * (x105 * (-x129 * (-x303 * x312 + x304 * x312 + x305 * x312 - x308 + x310) + x322) + x306 * x73);
	const GEN_FLT x324 = x323 * x76;
	const GEN_FLT x325 = x323 * x77 + x74 * (-x323 * x75 + x324);
	const GEN_FLT x326 = x323 * x78 + x325 * x74;
	const GEN_FLT x327 = x112 * x322 + x306 * x67;
	const GEN_FLT x328 = lh_qi * x53;
	const GEN_FLT x329 = lh_qj * x24;
	const GEN_FLT x330 = x302 * x35;
	const GEN_FLT x331 = x302 * x43;
	const GEN_FLT x332 = lh_qj * x36;
	const GEN_FLT x333 = -lh_qk * x53;
	const GEN_FLT x334 = x114 * (-x302 * x316 + x330 * x50 + x331 * x52 + x332 + x333) +
						 x115 * (-x302 * x321 - x328 + x329 + x330 * x62 - x331 * x58);
	const GEN_FLT x335 = -x111 * x327 + x334;
	const GEN_FLT x336 =
		x334 - x97 * (x124 * (x120 * x335 -
							  x122 * (x323 * x79 + x323 * x89 + x326 * x74 +
									  x74 * (x323 * x88 + x326 +
											 x74 * (x323 * x87 + x325 + x74 * (-x121 * x323 + x323 * x86 + x324))))) +
					  x125 * x335 + x126 * x323 + x326 * x95 + x327);
	const GEN_FLT x337 = lh_qw * x36;
	const GEN_FLT x338 = lh_qi * x301;
	const GEN_FLT x339 = x43 * (-x307 - x338 * x38);
	const GEN_FLT x340 = x304 * x338 + x305 * x338 + x329 - x337 + x339;
	const GEN_FLT x341 = lh_qw * x101;
	const GEN_FLT x342 = x341 * x35;
	const GEN_FLT x343 = lh_qi * x311;
	const GEN_FLT x344 = x313 * x43;
	const GEN_FLT x345 = x309 * x35;
	const GEN_FLT x346 = x35 * x50;
	const GEN_FLT x347 = x343 * x43;
	const GEN_FLT x348 = -x341 * x43;
	const GEN_FLT x349 = x35 * (x307 + x338 * x62);
	const GEN_FLT x350 = -x102 * (-x316 * x343 + x343 * x346 + x344 + x345 + x347 * x52) -
						 x142 * (-x310 - x321 * x343 - x347 * x58 + x348 + 2 * x349);
	const GEN_FLT x351 =
		x98 * (x105 * (-x129 * (x304 * x343 + x305 * x343 + x320 + 2 * x339 - x342) + x350) + x340 * x73);
	const GEN_FLT x352 = x351 * x76;
	const GEN_FLT x353 = x351 * x77 + x74 * (-x351 * x75 + x352);
	const GEN_FLT x354 = x351 * x78 + x353 * x74;
	const GEN_FLT x355 = x112 * x350 + x340 * x67;
	const GEN_FLT x356 = -lh_qw * x53;
	const GEN_FLT x357 = x338 * x43;
	const GEN_FLT x358 = lh_qj * x53;
	const GEN_FLT x359 = lh_qk * x36;
	const GEN_FLT x360 = x114 * (-x316 * x338 + x338 * x346 + x357 * x52 + x358 + x359) +
						 x115 * (-x299 - x321 * x338 + x349 + x356 - x357 * x58);
	const GEN_FLT x361 = -x111 * x355 + x360;
	const GEN_FLT x362 =
		x360 - x97 * (x124 * (x120 * x361 -
							  x122 * (x351 * x79 + x351 * x89 + x354 * x74 +
									  x74 * (x351 * x88 + x354 +
											 x74 * (x351 * x87 + x353 + x74 * (-x121 * x351 + x351 * x86 + x352))))) +
					  x125 * x361 + x126 * x351 + x354 * x95 + x355);
	const GEN_FLT x363 = lh_qi * x24;
	const GEN_FLT x364 = lh_qj * x301;
	const GEN_FLT x365 = -x303 * x364 + x304 * x364 + x305 * x364 + x359 + x363;
	const GEN_FLT x366 = x17 * x307;
	const GEN_FLT x367 = lh_qj * x311;
	const GEN_FLT x368 = x43 * x52;
	const GEN_FLT x369 = x17 * (-x21 * x364 - x313);
	const GEN_FLT x370 = x17 * x341;
	const GEN_FLT x371 = x43 * x58;
	const GEN_FLT x372 = x35 * (x313 + x364 * x62);
	const GEN_FLT x373 = -x102 * (x319 + x342 + x346 * x367 + x367 * x368 + 2 * x369) -
						 x142 * (x315 - x321 * x367 - x367 * x371 + x370 + 2 * x372);
	const GEN_FLT x374 =
		x98 * (x105 * (-x129 * (-x303 * x367 + x304 * x367 + x305 * x367 + x345 + x366) + x373) + x365 * x73);
	const GEN_FLT x375 = x374 * x76;
	const GEN_FLT x376 = x374 * x77 + x74 * (-x374 * x75 + x375);
	const GEN_FLT x377 = x374 * x78 + x376 * x74;
	const GEN_FLT x378 = x112 * x373 + x365 * x67;
	const GEN_FLT x379 = lh_qw * x24;
	const GEN_FLT x380 = x114 * (x328 + x337 + x346 * x364 + x364 * x368 + x369) +
						 x115 * (-x321 * x364 + x333 - x364 * x371 + x372 + x379);
	const GEN_FLT x381 = -x111 * x378 + x380;
	const GEN_FLT x382 =
		x380 - x97 * (x124 * (x120 * x381 -
							  x122 * (x374 * x79 + x374 * x89 + x377 * x74 +
									  x74 * (x374 * x88 + x377 +
											 x74 * (x374 * x87 + x376 + x74 * (-x121 * x374 + x374 * x86 + x375))))) +
					  x125 * x381 + x126 * x374 + x377 * x95 + x378);
	const GEN_FLT x383 = lh_qk * x301;
	const GEN_FLT x384 = -x309;
	const GEN_FLT x385 = x43 * (-x38 * x383 + x384);
	const GEN_FLT x386 = x304 * x383 + x305 * x383 + x332 + x379 + x385;
	const GEN_FLT x387 = lh_qk * x311;
	const GEN_FLT x388 = x35 * x62;
	const GEN_FLT x389 = x17 * (-x21 * x383 + x384);
	const GEN_FLT x390 = -x102 * (x308 + x346 * x387 + x348 + x368 * x387 + 2 * x389) -
						 x142 * (-x321 * x387 - x344 - x366 - x371 * x387 + x387 * x388);
	const GEN_FLT x391 =
		x98 * (x105 * (-x129 * (x304 * x387 + x305 * x387 + x314 + x370 + 2 * x385) + x390) + x386 * x73);
	const GEN_FLT x392 = x391 * x76;
	const GEN_FLT x393 = x391 * x77 + x74 * (-x391 * x75 + x392);
	const GEN_FLT x394 = x391 * x78 + x393 * x74;
	const GEN_FLT x395 = x112 * x390 + x386 * x67;
	const GEN_FLT x396 = x114 * (x298 + x346 * x383 + x356 + x368 * x383 + x389) +
						 x115 * (-x321 * x383 - x358 - x363 - x371 * x383 + x383 * x388);
	const GEN_FLT x397 = -x111 * x395 + x396;
	const GEN_FLT x398 =
		x396 - x97 * (x124 * (x120 * x397 -
							  x122 * (x391 * x79 + x391 * x89 + x394 * x74 +
									  x74 * (x391 * x88 + x394 +
											 x74 * (x391 * x87 + x393 + x74 * (-x121 * x391 + x391 * x86 + x392))))) +
					  x125 * x397 + x126 * x391 + x394 * x95 + x395);
	*(out++) = x127 * x128 + x127;
	*(out++) = x128 * x140 + x140;
	*(out++) = x128 * x152 + x152;
	*(out++) = x128 * x186 + x186;
	*(out++) = x128 * x218 + x218;
	*(out++) = x128 * x247 + x247;
	*(out++) = x128 * x274 + x274;
	*(out++) = x128 * x283 + x283;
	*(out++) = -x128 * x289 - x289;
	*(out++) = x128 * x297 + x297;
	*(out++) = x128 * x336 + x336;
	*(out++) = x128 * x362 + x362;
	*(out++) = x128 * x382 + x382;
	*(out++) = x128 * x398 + x398;
}

static inline void gen_reproject_jac_all(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh, const FLT phase_0,
										 const FLT phase_1, const FLT tilt_0, const FLT tilt_1, const FLT curve_0,
										 const FLT curve_1, const FLT gibPhase_0, const FLT gibPhase_1,
										 const FLT gibMag_0, const FLT gibMag_1) {
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
	const GEN_FLT x3 = x0 + x1;
	const GEN_FLT x4 = obj_qi * obj_qw;
	const GEN_FLT x5 = obj_qj * obj_qk;
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = obj_qi * obj_qi;
	const GEN_FLT x8 = obj_qj * obj_qj;
	const GEN_FLT x9 = obj_qk * obj_qk;
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = sqrt(obj_qw * obj_qw + x10 + x7);
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = sensor_y * x12;
	const GEN_FLT x14 = obj_qi * obj_qk;
	const GEN_FLT x15 = obj_qj * obj_qw;
	const GEN_FLT x16 = x14 - x15;
	const GEN_FLT x17 = sensor_x * x12;
	const GEN_FLT x18 = x7 + x8;
	const GEN_FLT x19 = obj_pz + sensor_z * (-x12 * x18 + 1) + x13 * x6 + x16 * x17;
	const GEN_FLT x20 = lh_qi * lh_qi;
	const GEN_FLT x21 = lh_qj * lh_qj;
	const GEN_FLT x22 = lh_qk * lh_qk;
	const GEN_FLT x23 = x21 + x22;
	const GEN_FLT x24 = sqrt(lh_qw * lh_qw + x20 + x23);
	const GEN_FLT x25 = 2 * x24;
	const GEN_FLT x26 = x19 * x25;
	const GEN_FLT x27 = x26 * x3;
	const GEN_FLT x28 = lh_qi * lh_qj;
	const GEN_FLT x29 = lh_qk * lh_qw;
	const GEN_FLT x30 = x28 - x29;
	const GEN_FLT x31 = obj_qi * obj_qj;
	const GEN_FLT x32 = obj_qk * obj_qw;
	const GEN_FLT x33 = x31 + x32;
	const GEN_FLT x34 = -x4 + x5;
	const GEN_FLT x35 = sensor_z * x12;
	const GEN_FLT x36 = x7 + x9;
	const GEN_FLT x37 = obj_py + sensor_y * (-x12 * x36 + 1) + x17 * x33 + x34 * x35;
	const GEN_FLT x38 = x25 * x37;
	const GEN_FLT x39 = x30 * x38;
	const GEN_FLT x40 = -x23 * x25 + 1;
	const GEN_FLT x41 = x14 + x15;
	const GEN_FLT x42 = x31 - x32;
	const GEN_FLT x43 = obj_px + sensor_x * (-x10 * x12 + 1) + x13 * x42 + x35 * x41;
	const GEN_FLT x44 = x40 * x43;
	const GEN_FLT x45 = -lh_px - x27 - x39 - x44;
	const GEN_FLT x46 = lh_px + x27 + x39 + x44;
	const GEN_FLT x47 = x46 * x46;
	const GEN_FLT x48 = lh_qi * lh_qw;
	const GEN_FLT x49 = lh_qj * lh_qk;
	const GEN_FLT x50 = x48 + x49;
	const GEN_FLT x51 = x25 * x43;
	const GEN_FLT x52 = x20 + x21;
	const GEN_FLT x53 = x25 * x52;
	const GEN_FLT x54 = -lh_pz - x19 * (1 - x53) - x2 * x51 - x38 * x50;
	const GEN_FLT x55 = x54 * x54;
	const GEN_FLT x56 = x47 + x55;
	const GEN_FLT x57 = 1.0 / x56;
	const GEN_FLT x58 = x45 * x57;
	const GEN_FLT x59 = x25 * x58;
	const GEN_FLT x60 = x2 * x59;
	const GEN_FLT x61 = x54 * x57;
	const GEN_FLT x62 = x40 * x61;
	const GEN_FLT x63 = x28 + x29;
	const GEN_FLT x64 = x51 * x63;
	const GEN_FLT x65 = -x48 + x49;
	const GEN_FLT x66 = x26 * x65;
	const GEN_FLT x67 = x20 + x22;
	const GEN_FLT x68 = -x25 * x67 + 1;
	const GEN_FLT x69 = x37 * x68;
	const GEN_FLT x70 = lh_py + x64 + x66 + x69;
	const GEN_FLT x71 = x70 * x70;
	const GEN_FLT x72 = x55 + x71;
	const GEN_FLT x73 = 1.0 / x72;
	const GEN_FLT x74 = x54 * x73;
	const GEN_FLT x75 = 4 * x24;
	const GEN_FLT x76 = x63 * x75;
	const GEN_FLT x77 = -lh_py - x64 - x66 - x69;
	const GEN_FLT x78 = x73 * x77;
	const GEN_FLT x79 = x2 * x75;
	const GEN_FLT x80 = atan2(x70, x54);
	const GEN_FLT x81 = curve_0 * x80;
	const GEN_FLT x82 = pow(-x57 * x71 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x83 = tilt_0 / sqrt(x56);
	const GEN_FLT x84 = x25 * x83;
	const GEN_FLT x85 = (1.0 / 2.0) * x46;
	const GEN_FLT x86 = x25 * x54;
	const GEN_FLT x87 = x2 * x86;
	const GEN_FLT x88 = tilt_0 * x70 / pow(x56, 3.0 / 2.0);
	const GEN_FLT x89 = x82 * (x63 * x84 + x88 * (-x85 * (-x23 * x75 + 2) + x87));
	const GEN_FLT x90 = atan2(x46, x54);
	const GEN_FLT x91 = gibMag_0 * sin(-gibPhase_0 + phase_0 + x90 + asin(x70 * x83) - 1.5707963267948966);
	const GEN_FLT x92 = x50 * x59;
	const GEN_FLT x93 = x25 * x61;
	const GEN_FLT x94 = x30 * x93;
	const GEN_FLT x95 = x50 * x75;
	const GEN_FLT x96 = x68 * x74;
	const GEN_FLT x97 = x25 * x46;
	const GEN_FLT x98 = x50 * x86;
	const GEN_FLT x99 = x82 * (x68 * x83 + x88 * (-x30 * x97 + x98));
	const GEN_FLT x100 = x3 * x93;
	const GEN_FLT x101 = x53 - 1;
	const GEN_FLT x102 = x101 * x58;
	const GEN_FLT x103 = x65 * x75;
	const GEN_FLT x104 = x101 * x78;
	const GEN_FLT x105 = (1.0 / 2.0) * x54;
	const GEN_FLT x106 = -x105 * (x52 * x75 - 2);
	const GEN_FLT x107 = x82 * (x65 * x84 + x88 * (x106 - x3 * x97));
	const GEN_FLT x108 = obj_qi * x13;
	const GEN_FLT x109 = obj_qj * x17;
	const GEN_FLT x110 = 2 / x11;
	const GEN_FLT x111 = obj_qw * x110;
	const GEN_FLT x112 = sensor_y * x111;
	const GEN_FLT x113 = sensor_z * x111;
	const GEN_FLT x114 = sensor_x * x111;
	const GEN_FLT x115 = x108 - x109 + x112 * x6 - x113 * x18 + x114 * x16;
	const GEN_FLT x116 = x115 * x25;
	const GEN_FLT x117 = obj_qi * x35;
	const GEN_FLT x118 = obj_qk * x17;
	const GEN_FLT x119 = sensor_y * x36;
	const GEN_FLT x120 = -x111 * x119 + x113 * x34 + x114 * x33 - x117 + x118;
	const GEN_FLT x121 = x120 * x25;
	const GEN_FLT x122 = obj_qj * x35;
	const GEN_FLT x123 = obj_qk * x13;
	const GEN_FLT x124 = -x10 * x114 + x112 * x42 + x113 * x41 + x122 - x123;
	const GEN_FLT x125 = x124 * x40;
	const GEN_FLT x126 = x116 * x3 + x121 * x30 + x125;
	const GEN_FLT x127 = x126 * x61;
	const GEN_FLT x128 = x124 * x25;
	const GEN_FLT x129 = x101 * x115;
	const GEN_FLT x130 = -x121 * x50 - x128 * x2 + x129;
	const GEN_FLT x131 = x130 * x58;
	const GEN_FLT x132 = x120 * x68;
	const GEN_FLT x133 = x116 * x65 + x128 * x63 + x132;
	const GEN_FLT x134 = x133 * x74;
	const GEN_FLT x135 = x130 * x78;
	const GEN_FLT x136 = x3 * x75;
	const GEN_FLT x137 = x120 * x75;
	const GEN_FLT x138 = -x105 * (-x124 * x79 + 2 * x129 - x137 * x50);
	const GEN_FLT x139 = x82 * (x133 * x83 + x88 * (x138 - x85 * (x115 * x136 + 2 * x125 + x137 * x30)));
	const GEN_FLT x140 = obj_qj * x13;
	const GEN_FLT x141 = obj_qk * x35;
	const GEN_FLT x142 = obj_qi * x110;
	const GEN_FLT x143 = sensor_x * x142;
	const GEN_FLT x144 = sensor_z * x142;
	const GEN_FLT x145 = sensor_y * x142;
	const GEN_FLT x146 = -x10 * x143 + x140 + x141 + x144 * x41 + x145 * x42;
	const GEN_FLT x147 = x146 * x40;
	const GEN_FLT x148 = obj_qw * x13;
	const GEN_FLT x149 = 4 * x11;
	const GEN_FLT x150 = -obj_qi * x149;
	const GEN_FLT x151 = sensor_z * (-x142 * x18 + x150) + x118 + x143 * x16 + x145 * x6 + x148;
	const GEN_FLT x152 = x151 * x25;
	const GEN_FLT x153 = obj_qw * x35;
	const GEN_FLT x154 = sensor_y * (-x142 * x36 + x150) + x109 + x143 * x33 + x144 * x34 - x153;
	const GEN_FLT x155 = x154 * x25;
	const GEN_FLT x156 = x147 + x152 * x3 + x155 * x30;
	const GEN_FLT x157 = x156 * x61;
	const GEN_FLT x158 = x146 * x25;
	const GEN_FLT x159 = x101 * x151;
	const GEN_FLT x160 = -x155 * x50 - x158 * x2 + x159;
	const GEN_FLT x161 = x160 * x58;
	const GEN_FLT x162 = x154 * x68;
	const GEN_FLT x163 = x152 * x65 + x158 * x63 + x162;
	const GEN_FLT x164 = x163 * x74;
	const GEN_FLT x165 = x160 * x78;
	const GEN_FLT x166 = x154 * x75;
	const GEN_FLT x167 = -x105 * (-x146 * x79 + 2 * x159 - x166 * x50);
	const GEN_FLT x168 = x82 * (x163 * x83 + x88 * (x167 - x85 * (x136 * x151 + 2 * x147 + x166 * x30)));
	const GEN_FLT x169 = obj_qi * x17;
	const GEN_FLT x170 = obj_qj * x110;
	const GEN_FLT x171 = sensor_x * x170;
	const GEN_FLT x172 = sensor_z * x170;
	const GEN_FLT x173 = -x119 * x170 + x141 + x169 + x171 * x33 + x172 * x34;
	const GEN_FLT x174 = x173 * x25;
	const GEN_FLT x175 = obj_qw * x17;
	const GEN_FLT x176 = sensor_y * x170;
	const GEN_FLT x177 = -obj_qj * x149;
	const GEN_FLT x178 = sensor_z * (-x170 * x18 + x177) + x123 + x16 * x171 - x175 + x176 * x6;
	const GEN_FLT x179 = x178 * x25;
	const GEN_FLT x180 = sensor_x * (-x10 * x170 + x177) + x108 + x153 + x172 * x41 + x176 * x42;
	const GEN_FLT x181 = x180 * x40;
	const GEN_FLT x182 = x174 * x30 + x179 * x3 + x181;
	const GEN_FLT x183 = x182 * x61;
	const GEN_FLT x184 = x180 * x25;
	const GEN_FLT x185 = x101 * x178;
	const GEN_FLT x186 = -x174 * x50 - x184 * x2 + x185;
	const GEN_FLT x187 = x186 * x58;
	const GEN_FLT x188 = x173 * x68;
	const GEN_FLT x189 = x179 * x65 + x184 * x63 + x188;
	const GEN_FLT x190 = x189 * x74;
	const GEN_FLT x191 = x186 * x78;
	const GEN_FLT x192 = x173 * x75;
	const GEN_FLT x193 = -x105 * (-x180 * x79 + 2 * x185 - x192 * x50);
	const GEN_FLT x194 = x82 * (x189 * x83 + x88 * (x193 - x85 * (x136 * x178 + 2 * x181 + x192 * x30)));
	const GEN_FLT x195 = obj_qk * x110;
	const GEN_FLT x196 = sensor_y * x195;
	const GEN_FLT x197 = sensor_z * x195;
	const GEN_FLT x198 = sensor_x * x195;
	const GEN_FLT x199 = x140 + x16 * x198 + x169 - x18 * x197 + x196 * x6;
	const GEN_FLT x200 = x199 * x25;
	const GEN_FLT x201 = -obj_qk * x149;
	const GEN_FLT x202 = sensor_y * (-x195 * x36 + x201) + x122 + x175 + x197 * x34 + x198 * x33;
	const GEN_FLT x203 = x202 * x25;
	const GEN_FLT x204 = sensor_x * (-x10 * x195 + x201) + x117 - x148 + x196 * x42 + x197 * x41;
	const GEN_FLT x205 = x204 * x40;
	const GEN_FLT x206 = x200 * x3 + x203 * x30 + x205;
	const GEN_FLT x207 = x206 * x61;
	const GEN_FLT x208 = x101 * x199;
	const GEN_FLT x209 = x204 * x25;
	const GEN_FLT x210 = -x2 * x209 - x203 * x50 + x208;
	const GEN_FLT x211 = x210 * x58;
	const GEN_FLT x212 = x202 * x68;
	const GEN_FLT x213 = x200 * x65 + x209 * x63 + x212;
	const GEN_FLT x214 = x213 * x74;
	const GEN_FLT x215 = x210 * x78;
	const GEN_FLT x216 = x202 * x75;
	const GEN_FLT x217 = -x105 * (-x204 * x79 + 2 * x208 - x216 * x50);
	const GEN_FLT x218 = x82 * (x213 * x83 + x88 * (x217 - x85 * (x136 * x199 + 2 * x205 + x216 * x30)));
	const GEN_FLT x219 = x82 * x88;
	const GEN_FLT x220 = x219 * x45;
	const GEN_FLT x221 = 2 * x81;
	const GEN_FLT x222 = x82 * x83;
	const GEN_FLT x223 = x219 * x54;
	const GEN_FLT x224 = lh_qi * x38;
	const GEN_FLT x225 = lh_qj * x51;
	const GEN_FLT x226 = 1.0 / x24;
	const GEN_FLT x227 = 2 * x226;
	const GEN_FLT x228 = lh_qw * x227;
	const GEN_FLT x229 = x19 * x228;
	const GEN_FLT x230 = x228 * x37;
	const GEN_FLT x231 = x228 * x43;
	const GEN_FLT x232 = -x2 * x231 - x224 + x225 + x229 * x52 - x230 * x50;
	const GEN_FLT x233 = x232 * x58;
	const GEN_FLT x234 = lh_qj * x26;
	const GEN_FLT x235 = -lh_qk * x38;
	const GEN_FLT x236 = x229 * x3 - x23 * x231 + x230 * x30 + x234 + x235;
	const GEN_FLT x237 = x236 * x61;
	const GEN_FLT x238 = x232 * x78;
	const GEN_FLT x239 = lh_qi * x26;
	const GEN_FLT x240 = lh_qk * x51;
	const GEN_FLT x241 = x37 * x67;
	const GEN_FLT x242 = -x228 * x241 + x229 * x65 + x231 * x63 - x239 + x240;
	const GEN_FLT x243 = x242 * x74;
	const GEN_FLT x244 = lh_qj * x75;
	const GEN_FLT x245 = x19 * x244;
	const GEN_FLT x246 = lh_qk * x75;
	const GEN_FLT x247 = -x246 * x37;
	const GEN_FLT x248 = x23 * x43;
	const GEN_FLT x249 = 4 * x226;
	const GEN_FLT x250 = lh_qw * x249;
	const GEN_FLT x251 = x19 * x250;
	const GEN_FLT x252 = x250 * x37;
	const GEN_FLT x253 = lh_qi * x75;
	const GEN_FLT x254 = x253 * x37;
	const GEN_FLT x255 = x244 * x43;
	const GEN_FLT x256 = x2 * x43;
	const GEN_FLT x257 = -x105 * (-x250 * x256 + x251 * x52 - x252 * x50 - x254 + x255);
	const GEN_FLT x258 = x82 * (x242 * x83 + x88 * (x257 - x85 * (x245 + x247 - x248 * x250 + x251 * x3 + x252 * x30)));
	const GEN_FLT x259 = lh_qj * x38;
	const GEN_FLT x260 = lh_qk * x26;
	const GEN_FLT x261 = lh_qi * x227;
	const GEN_FLT x262 = x19 * x261;
	const GEN_FLT x263 = x261 * x37;
	const GEN_FLT x264 = -x248 * x261 + x259 + x260 + x262 * x3 + x263 * x30;
	const GEN_FLT x265 = x264 * x61;
	const GEN_FLT x266 = -lh_qw * x38;
	const GEN_FLT x267 = x19 * (x253 + x261 * x52);
	const GEN_FLT x268 = -x240 - x256 * x261 - x263 * x50 + x266 + x267;
	const GEN_FLT x269 = x268 * x58;
	const GEN_FLT x270 = x268 * x78;
	const GEN_FLT x271 = lh_qw * x26;
	const GEN_FLT x272 = x43 * x63;
	const GEN_FLT x273 = x37 * (-x253 - x261 * x67);
	const GEN_FLT x274 = x225 + x261 * x272 + x262 * x65 - x271 + x273;
	const GEN_FLT x275 = x274 * x74;
	const GEN_FLT x276 = x244 * x37;
	const GEN_FLT x277 = x19 * x246;
	const GEN_FLT x278 = lh_qi * x249;
	const GEN_FLT x279 = x19 * x3;
	const GEN_FLT x280 = x278 * x37;
	const GEN_FLT x281 = x246 * x43;
	const GEN_FLT x282 = lh_qw * x75;
	const GEN_FLT x283 = -x282 * x37;
	const GEN_FLT x284 = -x105 * (-x256 * x278 + 2 * x267 - x280 * x50 - x281 + x283);
	const GEN_FLT x285 =
		x82 * (x274 * x83 + x88 * (x284 - x85 * (-x248 * x278 + x276 + x277 + x278 * x279 + x280 * x30)));
	const GEN_FLT x286 = lh_qw * x51;
	const GEN_FLT x287 = lh_qj * x227;
	const GEN_FLT x288 = x37 * x50;
	const GEN_FLT x289 = x19 * (x244 + x287 * x52);
	const GEN_FLT x290 = x235 - x256 * x287 + x286 - x287 * x288 + x289;
	const GEN_FLT x291 = x290 * x58;
	const GEN_FLT x292 = x30 * x37;
	const GEN_FLT x293 = x43 * (-x23 * x287 - x244);
	const GEN_FLT x294 = x224 + x271 + x279 * x287 + x287 * x292 + x293;
	const GEN_FLT x295 = x294 * x61;
	const GEN_FLT x296 = lh_qi * x51;
	const GEN_FLT x297 = x19 * x65;
	const GEN_FLT x298 = -x241 * x287 + x260 + x272 * x287 + x287 * x297 + x296;
	const GEN_FLT x299 = x298 * x74;
	const GEN_FLT x300 = x290 * x78;
	const GEN_FLT x301 = x19 * x282;
	const GEN_FLT x302 = lh_qj * x249;
	const GEN_FLT x303 = x282 * x43;
	const GEN_FLT x304 = -x105 * (x247 - x256 * x302 - x288 * x302 + 2 * x289 + x303);
	const GEN_FLT x305 = x82 * (x298 * x83 + x88 * (x304 - x85 * (x254 + x279 * x302 + x292 * x302 + 2 * x293 + x301)));
	const GEN_FLT x306 = lh_qk * x227;
	const GEN_FLT x307 = x19 * x52;
	const GEN_FLT x308 = -x256 * x306 - x259 - x288 * x306 - x296 + x306 * x307;
	const GEN_FLT x309 = x308 * x58;
	const GEN_FLT x310 = -x246;
	const GEN_FLT x311 = x43 * (-x23 * x306 + x310);
	const GEN_FLT x312 = x239 + x266 + x279 * x306 + x292 * x306 + x311;
	const GEN_FLT x313 = x312 * x61;
	const GEN_FLT x314 = x308 * x78;
	const GEN_FLT x315 = x37 * (-x306 * x67 + x310);
	const GEN_FLT x316 = x234 + x272 * x306 + x286 + x297 * x306 + x315;
	const GEN_FLT x317 = x316 * x74;
	const GEN_FLT x318 = x253 * x43;
	const GEN_FLT x319 = lh_qk * x249;
	const GEN_FLT x320 = -x105 * (-x256 * x319 - x276 - x288 * x319 + x307 * x319 - x318);
	const GEN_FLT x321 = x19 * x253;
	const GEN_FLT x322 = x82 * (x316 * x83 + x88 * (x320 - x85 * (x279 * x319 + x283 + x292 * x319 + 2 * x311 + x321)));
	const GEN_FLT x323 = curve_1 * x90;
	const GEN_FLT x324 = x25 * x74;
	const GEN_FLT x325 = x25 * x78;
	const GEN_FLT x326 = pow(-x47 * x73 * tilt_1 * tilt_1 + 1, -1.0 / 2.0);
	const GEN_FLT x327 = tilt_1 / sqrt(x72);
	const GEN_FLT x328 = x25 * x70;
	const GEN_FLT x329 = tilt_1 * x46 / pow(x72, 3.0 / 2.0);
	const GEN_FLT x330 = -x2 * x325 + x324 * x63 - x326 * (x327 * x40 + x329 * (-x328 * x63 + x87));
	const GEN_FLT x331 = gibMag_1 * sin(gibPhase_1 - phase_1 + x80 - asin(x327 * x46) + 1.5707963267948966);
	const GEN_FLT x332 = x25 * x327;
	const GEN_FLT x333 = (1.0 / 2.0) * x70;
	const GEN_FLT x334 = -x325 * x50 - x326 * (x30 * x332 + x329 * (-x333 * (-x67 * x75 + 2) + x98)) + x96;
	const GEN_FLT x335 = x104 + x324 * x65 - x326 * (x3 * x332 + x329 * (x106 - x328 * x65));
	const GEN_FLT x336 =
		x134 + x135 - x326 * (x126 * x327 + x329 * (x138 - x333 * (x103 * x115 + x124 * x76 + 2 * x132)));
	const GEN_FLT x337 =
		x164 + x165 - x326 * (x156 * x327 + x329 * (x167 - x333 * (x103 * x151 + x146 * x76 + 2 * x162)));
	const GEN_FLT x338 =
		x190 + x191 - x326 * (x182 * x327 + x329 * (x193 - x333 * (x103 * x178 + x180 * x76 + 2 * x188)));
	const GEN_FLT x339 =
		x214 + x215 - x326 * (x206 * x327 + x329 * (x217 - x333 * (x103 * x199 + x204 * x76 + 2 * x212)));
	const GEN_FLT x340 = 2 * x323;
	const GEN_FLT x341 = x326 * x327;
	const GEN_FLT x342 = x326 * x329;
	const GEN_FLT x343 = -x342 * x77 + x74;
	const GEN_FLT x344 = -x342 * x54 - x78;
	const GEN_FLT x345 =
		x238 + x243 -
		x326 * (x236 * x327 + x329 * (x257 - x333 * (-x241 * x250 + x250 * x272 + x251 * x65 + x281 - x321)));
	const GEN_FLT x346 =
		x270 + x275 -
		x326 * (x264 * x327 + x329 * (x284 - x333 * (x255 + x272 * x278 + 2 * x273 + x278 * x297 - x301)));
	const GEN_FLT x347 =
		x299 + x300 -
		x326 * (x294 * x327 + x329 * (x304 - x333 * (-x241 * x302 + x272 * x302 + x277 + x297 * x302 + x318)));
	const GEN_FLT x348 =
		x314 + x317 -
		x326 * (x312 * x327 + x329 * (x320 - x333 * (x245 + x272 * x319 + x297 * x319 + x303 + 2 * x315)));
	*(out++) = x60 - x62 + x81 * (x74 * x76 - x78 * x79) - x89 + x91 * (-x60 + x62 + x89);
	*(out++) = x81 * (-x78 * x95 + 2 * x96) + x91 * (-x92 + x94 + x99) + x92 - x94 - x99;
	*(out++) = -x100 - x102 - x107 + x81 * (x103 * x74 + 2 * x104) + x91 * (x100 + x102 + x107);
	*(out++) = -x127 - x131 - x139 + x81 * (2 * x134 + 2 * x135) + x91 * (x127 + x131 + x139);
	*(out++) = -x157 - x161 - x168 + x81 * (2 * x164 + 2 * x165) + x91 * (x157 + x161 + x168);
	*(out++) = -x183 - x187 - x194 + x81 * (2 * x190 + 2 * x191) + x91 * (x183 + x187 + x194);
	*(out++) = -x207 - x211 - x218 + x81 * (2 * x214 + 2 * x215) + x91 * (x207 + x211 + x218);
	*(out++) = -x220 - x61 + x91 * (x220 + x61);
	*(out++) = x221 * x74 + x222 * x91 - x222;
	*(out++) = -x221 * x78 - x223 + x58 + x91 * (x223 - x58);
	*(out++) = -x233 - x237 - x258 + x81 * (2 * x238 + 2 * x243) + x91 * (x233 + x237 + x258);
	*(out++) = -x265 - x269 - x285 + x81 * (2 * x270 + 2 * x275) + x91 * (x265 + x269 + x285);
	*(out++) = -x291 - x295 - x305 + x81 * (2 * x299 + 2 * x300) + x91 * (x291 + x295 + x305);
	*(out++) = -x309 - x313 - x322 + x81 * (2 * x314 + 2 * x317) + x91 * (x309 + x313 + x322);
	*(out++) = x323 * (-x58 * x79 + 2 * x62) + x330 * x331 + x330;
	*(out++) = x323 * (x30 * x61 * x75 - x58 * x95) + x331 * x334 + x334;
	*(out++) = x323 * (2 * x102 + x136 * x61) + x331 * x335 + x335;
	*(out++) = x323 * (2 * x127 + 2 * x131) + x331 * x336 + x336;
	*(out++) = x323 * (2 * x157 + 2 * x161) + x331 * x337 + x337;
	*(out++) = x323 * (2 * x183 + 2 * x187) + x331 * x338 + x338;
	*(out++) = x323 * (2 * x207 + 2 * x211) + x331 * x339 + x339;
	*(out++) = -x331 * x341 + x340 * x61 - x341;
	*(out++) = x331 * x343 + x343;
	*(out++) = x331 * x344 - x340 * x58 + x344;
	*(out++) = x323 * (2 * x233 + 2 * x237) + x331 * x345 + x345;
	*(out++) = x323 * (2 * x265 + 2 * x269) + x331 * x346 + x346;
	*(out++) = x323 * (2 * x291 + 2 * x295) + x331 * x347 + x347;
	*(out++) = x323 * (2 * x309 + 2 * x313) + x331 * x348 + x348;
}

static inline void gen_reproject_axis_x_jac_all(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
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
	const GEN_FLT x3 = x0 + x1;
	const GEN_FLT x4 = obj_qi * obj_qw;
	const GEN_FLT x5 = obj_qj * obj_qk;
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = obj_qi * obj_qi;
	const GEN_FLT x8 = obj_qj * obj_qj;
	const GEN_FLT x9 = obj_qk * obj_qk;
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = sqrt(obj_qw * obj_qw + x10 + x7);
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = sensor_y * x12;
	const GEN_FLT x14 = obj_qi * obj_qk;
	const GEN_FLT x15 = obj_qj * obj_qw;
	const GEN_FLT x16 = x14 - x15;
	const GEN_FLT x17 = sensor_x * x12;
	const GEN_FLT x18 = x7 + x8;
	const GEN_FLT x19 = obj_pz + sensor_z * (-x12 * x18 + 1) + x13 * x6 + x16 * x17;
	const GEN_FLT x20 = lh_qi * lh_qi;
	const GEN_FLT x21 = lh_qj * lh_qj;
	const GEN_FLT x22 = lh_qk * lh_qk;
	const GEN_FLT x23 = x21 + x22;
	const GEN_FLT x24 = sqrt(lh_qw * lh_qw + x20 + x23);
	const GEN_FLT x25 = 2 * x24;
	const GEN_FLT x26 = x19 * x25;
	const GEN_FLT x27 = x26 * x3;
	const GEN_FLT x28 = lh_qi * lh_qj;
	const GEN_FLT x29 = lh_qk * lh_qw;
	const GEN_FLT x30 = x28 - x29;
	const GEN_FLT x31 = obj_qi * obj_qj;
	const GEN_FLT x32 = obj_qk * obj_qw;
	const GEN_FLT x33 = x31 + x32;
	const GEN_FLT x34 = -x4 + x5;
	const GEN_FLT x35 = sensor_z * x12;
	const GEN_FLT x36 = x7 + x9;
	const GEN_FLT x37 = obj_py + sensor_y * (-x12 * x36 + 1) + x17 * x33 + x34 * x35;
	const GEN_FLT x38 = x25 * x37;
	const GEN_FLT x39 = x30 * x38;
	const GEN_FLT x40 = -x23 * x25 + 1;
	const GEN_FLT x41 = x14 + x15;
	const GEN_FLT x42 = x31 - x32;
	const GEN_FLT x43 = obj_px + sensor_x * (-x10 * x12 + 1) + x13 * x42 + x35 * x41;
	const GEN_FLT x44 = x40 * x43;
	const GEN_FLT x45 = -lh_px - x27 - x39 - x44;
	const GEN_FLT x46 = lh_px + x27 + x39 + x44;
	const GEN_FLT x47 = lh_qi * lh_qw;
	const GEN_FLT x48 = lh_qj * lh_qk;
	const GEN_FLT x49 = x47 + x48;
	const GEN_FLT x50 = x25 * x43;
	const GEN_FLT x51 = x20 + x21;
	const GEN_FLT x52 = x25 * x51;
	const GEN_FLT x53 = -lh_pz - x19 * (1 - x52) - x2 * x50 - x38 * x49;
	const GEN_FLT x54 = x53 * x53;
	const GEN_FLT x55 = x46 * x46 + x54;
	const GEN_FLT x56 = 1.0 / x55;
	const GEN_FLT x57 = x45 * x56;
	const GEN_FLT x58 = x25 * x57;
	const GEN_FLT x59 = x2 * x58;
	const GEN_FLT x60 = x53 * x56;
	const GEN_FLT x61 = x40 * x60;
	const GEN_FLT x62 = x28 + x29;
	const GEN_FLT x63 = x50 * x62;
	const GEN_FLT x64 = -x47 + x48;
	const GEN_FLT x65 = x26 * x64;
	const GEN_FLT x66 = x20 + x22;
	const GEN_FLT x67 = -x25 * x66 + 1;
	const GEN_FLT x68 = x37 * x67;
	const GEN_FLT x69 = lh_py + x63 + x65 + x68;
	const GEN_FLT x70 = x69 * x69;
	const GEN_FLT x71 = 1.0 / (x54 + x70);
	const GEN_FLT x72 = 4 * x24;
	const GEN_FLT x73 = x53 * x71 * x72;
	const GEN_FLT x74 = x2 * x72;
	const GEN_FLT x75 = -lh_py - x63 - x65 - x68;
	const GEN_FLT x76 = x71 * x75;
	const GEN_FLT x77 = curve_0 * atan2(x69, x53);
	const GEN_FLT x78 = pow(-x56 * x70 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x79 = tilt_0 / sqrt(x55);
	const GEN_FLT x80 = x25 * x79;
	const GEN_FLT x81 = (1.0 / 2.0) * x46;
	const GEN_FLT x82 = x25 * x53;
	const GEN_FLT x83 = tilt_0 * x69 / pow(x55, 3.0 / 2.0);
	const GEN_FLT x84 = x78 * (x62 * x80 + x83 * (x2 * x82 - x81 * (-x23 * x72 + 2)));
	const GEN_FLT x85 = gibMag_0 * sin(-gibPhase_0 + phase_0 + asin(x69 * x79) + atan2(x46, x53) - 1.5707963267948966);
	const GEN_FLT x86 = x49 * x58;
	const GEN_FLT x87 = x25 * x60;
	const GEN_FLT x88 = x30 * x87;
	const GEN_FLT x89 = 2 * x71;
	const GEN_FLT x90 = x53 * x89;
	const GEN_FLT x91 = x25 * x46;
	const GEN_FLT x92 = x78 * (x67 * x79 + x83 * (-x30 * x91 + x49 * x82));
	const GEN_FLT x93 = x3 * x87;
	const GEN_FLT x94 = x52 - 1;
	const GEN_FLT x95 = x57 * x94;
	const GEN_FLT x96 = x75 * x89;
	const GEN_FLT x97 = (1.0 / 2.0) * x53;
	const GEN_FLT x98 = x78 * (x64 * x80 + x83 * (-x3 * x91 - x97 * (x51 * x72 - 2)));
	const GEN_FLT x99 = obj_qi * x13;
	const GEN_FLT x100 = obj_qj * x17;
	const GEN_FLT x101 = 2 / x11;
	const GEN_FLT x102 = obj_qw * x101;
	const GEN_FLT x103 = sensor_y * x102;
	const GEN_FLT x104 = sensor_z * x102;
	const GEN_FLT x105 = sensor_x * x102;
	const GEN_FLT x106 = -x100 + x103 * x6 - x104 * x18 + x105 * x16 + x99;
	const GEN_FLT x107 = x106 * x25;
	const GEN_FLT x108 = obj_qi * x35;
	const GEN_FLT x109 = obj_qk * x17;
	const GEN_FLT x110 = sensor_y * x36;
	const GEN_FLT x111 = -x102 * x110 + x104 * x34 + x105 * x33 - x108 + x109;
	const GEN_FLT x112 = x111 * x25;
	const GEN_FLT x113 = obj_qj * x35;
	const GEN_FLT x114 = obj_qk * x13;
	const GEN_FLT x115 = -x10 * x105 + x103 * x42 + x104 * x41 + x113 - x114;
	const GEN_FLT x116 = x115 * x40;
	const GEN_FLT x117 = x60 * (x107 * x3 + x112 * x30 + x116);
	const GEN_FLT x118 = x115 * x25;
	const GEN_FLT x119 = x106 * x94;
	const GEN_FLT x120 = -x112 * x49 - x118 * x2 + x119;
	const GEN_FLT x121 = x120 * x57;
	const GEN_FLT x122 = x107 * x64 + x111 * x67 + x118 * x62;
	const GEN_FLT x123 = x3 * x72;
	const GEN_FLT x124 = x111 * x72;
	const GEN_FLT x125 = x78 * (x122 * x79 + x83 * (-x81 * (x106 * x123 + 2 * x116 + x124 * x30) -
													x97 * (-x115 * x74 + 2 * x119 - x124 * x49)));
	const GEN_FLT x126 = obj_qj * x13;
	const GEN_FLT x127 = obj_qk * x35;
	const GEN_FLT x128 = obj_qi * x101;
	const GEN_FLT x129 = sensor_x * x128;
	const GEN_FLT x130 = sensor_z * x128;
	const GEN_FLT x131 = sensor_y * x128;
	const GEN_FLT x132 = -x10 * x129 + x126 + x127 + x130 * x41 + x131 * x42;
	const GEN_FLT x133 = x132 * x40;
	const GEN_FLT x134 = obj_qw * x13;
	const GEN_FLT x135 = 4 * x11;
	const GEN_FLT x136 = -obj_qi * x135;
	const GEN_FLT x137 = sensor_z * (-x128 * x18 + x136) + x109 + x129 * x16 + x131 * x6 + x134;
	const GEN_FLT x138 = x137 * x25;
	const GEN_FLT x139 = obj_qw * x35;
	const GEN_FLT x140 = sensor_y * (-x128 * x36 + x136) + x100 + x129 * x33 + x130 * x34 - x139;
	const GEN_FLT x141 = x140 * x25;
	const GEN_FLT x142 = x60 * (x133 + x138 * x3 + x141 * x30);
	const GEN_FLT x143 = x132 * x25;
	const GEN_FLT x144 = x137 * x94;
	const GEN_FLT x145 = -x141 * x49 - x143 * x2 + x144;
	const GEN_FLT x146 = x145 * x57;
	const GEN_FLT x147 = x138 * x64 + x140 * x67 + x143 * x62;
	const GEN_FLT x148 = x140 * x72;
	const GEN_FLT x149 = x78 * (x147 * x79 + x83 * (-x81 * (x123 * x137 + 2 * x133 + x148 * x30) -
													x97 * (-x132 * x74 + 2 * x144 - x148 * x49)));
	const GEN_FLT x150 = obj_qi * x17;
	const GEN_FLT x151 = obj_qj * x101;
	const GEN_FLT x152 = sensor_x * x151;
	const GEN_FLT x153 = sensor_z * x151;
	const GEN_FLT x154 = -x110 * x151 + x127 + x150 + x152 * x33 + x153 * x34;
	const GEN_FLT x155 = x154 * x25;
	const GEN_FLT x156 = obj_qw * x17;
	const GEN_FLT x157 = sensor_y * x151;
	const GEN_FLT x158 = -obj_qj * x135;
	const GEN_FLT x159 = sensor_z * (-x151 * x18 + x158) + x114 + x152 * x16 - x156 + x157 * x6;
	const GEN_FLT x160 = x159 * x25;
	const GEN_FLT x161 = sensor_x * (-x10 * x151 + x158) + x139 + x153 * x41 + x157 * x42 + x99;
	const GEN_FLT x162 = x161 * x40;
	const GEN_FLT x163 = x60 * (x155 * x30 + x160 * x3 + x162);
	const GEN_FLT x164 = x161 * x25;
	const GEN_FLT x165 = x159 * x94;
	const GEN_FLT x166 = -x155 * x49 - x164 * x2 + x165;
	const GEN_FLT x167 = x166 * x57;
	const GEN_FLT x168 = x154 * x67 + x160 * x64 + x164 * x62;
	const GEN_FLT x169 = x154 * x72;
	const GEN_FLT x170 = x78 * (x168 * x79 + x83 * (-x81 * (x123 * x159 + 2 * x162 + x169 * x30) -
													x97 * (-x161 * x74 + 2 * x165 - x169 * x49)));
	const GEN_FLT x171 = obj_qk * x101;
	const GEN_FLT x172 = sensor_y * x171;
	const GEN_FLT x173 = sensor_z * x171;
	const GEN_FLT x174 = sensor_x * x171;
	const GEN_FLT x175 = x126 + x150 + x16 * x174 + x172 * x6 - x173 * x18;
	const GEN_FLT x176 = x175 * x25;
	const GEN_FLT x177 = -obj_qk * x135;
	const GEN_FLT x178 = sensor_y * (-x171 * x36 + x177) + x113 + x156 + x173 * x34 + x174 * x33;
	const GEN_FLT x179 = x178 * x25;
	const GEN_FLT x180 = sensor_x * (-x10 * x171 + x177) + x108 - x134 + x172 * x42 + x173 * x41;
	const GEN_FLT x181 = x180 * x40;
	const GEN_FLT x182 = x60 * (x176 * x3 + x179 * x30 + x181);
	const GEN_FLT x183 = x175 * x94;
	const GEN_FLT x184 = x180 * x25;
	const GEN_FLT x185 = -x179 * x49 + x183 - x184 * x2;
	const GEN_FLT x186 = x185 * x57;
	const GEN_FLT x187 = x176 * x64 + x178 * x67 + x184 * x62;
	const GEN_FLT x188 = x178 * x72;
	const GEN_FLT x189 = x78 * (x187 * x79 + x83 * (-x81 * (x123 * x175 + 2 * x181 + x188 * x30) -
													x97 * (-x180 * x74 + 2 * x183 - x188 * x49)));
	const GEN_FLT x190 = x78 * x83;
	const GEN_FLT x191 = x190 * x45;
	const GEN_FLT x192 = x78 * x79;
	const GEN_FLT x193 = x190 * x53;
	const GEN_FLT x194 = lh_qi * x38;
	const GEN_FLT x195 = lh_qj * x50;
	const GEN_FLT x196 = 1.0 / x24;
	const GEN_FLT x197 = 2 * x196;
	const GEN_FLT x198 = lh_qw * x197;
	const GEN_FLT x199 = x19 * x198;
	const GEN_FLT x200 = x198 * x37;
	const GEN_FLT x201 = x198 * x43;
	const GEN_FLT x202 = -x194 + x195 + x199 * x51 - x2 * x201 - x200 * x49;
	const GEN_FLT x203 = x202 * x57;
	const GEN_FLT x204 = lh_qj * x26;
	const GEN_FLT x205 = -lh_qk * x38;
	const GEN_FLT x206 = x60 * (x199 * x3 + x200 * x30 - x201 * x23 + x204 + x205);
	const GEN_FLT x207 = lh_qi * x26;
	const GEN_FLT x208 = lh_qk * x50;
	const GEN_FLT x209 = x37 * x66;
	const GEN_FLT x210 = -x198 * x209 + x199 * x64 + x201 * x62 - x207 + x208;
	const GEN_FLT x211 = lh_qj * x72;
	const GEN_FLT x212 = lh_qk * x72;
	const GEN_FLT x213 = -x212 * x37;
	const GEN_FLT x214 = x23 * x43;
	const GEN_FLT x215 = 4 * x196;
	const GEN_FLT x216 = lh_qw * x215;
	const GEN_FLT x217 = x19 * x216;
	const GEN_FLT x218 = x216 * x37;
	const GEN_FLT x219 = lh_qi * x72;
	const GEN_FLT x220 = x219 * x37;
	const GEN_FLT x221 = x2 * x43;
	const GEN_FLT x222 = x78 * (x210 * x79 + x83 * (-x81 * (x19 * x211 + x213 - x214 * x216 + x217 * x3 + x218 * x30) -
													x97 * (x211 * x43 - x216 * x221 + x217 * x51 - x218 * x49 - x220)));
	const GEN_FLT x223 = lh_qj * x38;
	const GEN_FLT x224 = lh_qk * x26;
	const GEN_FLT x225 = lh_qi * x197;
	const GEN_FLT x226 = x19 * x225;
	const GEN_FLT x227 = x225 * x37;
	const GEN_FLT x228 = x60 * (-x214 * x225 + x223 + x224 + x226 * x3 + x227 * x30);
	const GEN_FLT x229 = -lh_qw * x38;
	const GEN_FLT x230 = x19 * (x219 + x225 * x51);
	const GEN_FLT x231 = -x208 - x221 * x225 - x227 * x49 + x229 + x230;
	const GEN_FLT x232 = x231 * x57;
	const GEN_FLT x233 = lh_qw * x26;
	const GEN_FLT x234 = x43 * x62;
	const GEN_FLT x235 = x195 + x225 * x234 + x226 * x64 - x233 + x37 * (-x219 - x225 * x66);
	const GEN_FLT x236 = x211 * x37;
	const GEN_FLT x237 = lh_qi * x215;
	const GEN_FLT x238 = x19 * x3;
	const GEN_FLT x239 = x237 * x37;
	const GEN_FLT x240 = lh_qw * x72;
	const GEN_FLT x241 = -x240 * x37;
	const GEN_FLT x242 =
		x78 * (x235 * x79 + x83 * (-x81 * (x19 * x212 - x214 * x237 + x236 + x237 * x238 + x239 * x30) -
								   x97 * (-x212 * x43 - x221 * x237 + 2 * x230 - x239 * x49 + x241)));
	const GEN_FLT x243 = lh_qw * x50;
	const GEN_FLT x244 = lh_qj * x197;
	const GEN_FLT x245 = x37 * x49;
	const GEN_FLT x246 = x19 * (x211 + x244 * x51);
	const GEN_FLT x247 = x205 - x221 * x244 + x243 - x244 * x245 + x246;
	const GEN_FLT x248 = x247 * x57;
	const GEN_FLT x249 = x30 * x37;
	const GEN_FLT x250 = x43 * (-x211 - x23 * x244);
	const GEN_FLT x251 = x60 * (x194 + x233 + x238 * x244 + x244 * x249 + x250);
	const GEN_FLT x252 = lh_qi * x50;
	const GEN_FLT x253 = x19 * x64;
	const GEN_FLT x254 = -x209 * x244 + x224 + x234 * x244 + x244 * x253 + x252;
	const GEN_FLT x255 = lh_qj * x215;
	const GEN_FLT x256 = x78 * (x254 * x79 + x83 * (-x81 * (x19 * x240 + x220 + x238 * x255 + x249 * x255 + 2 * x250) -
													x97 * (x213 - x221 * x255 + x240 * x43 - x245 * x255 + 2 * x246)));
	const GEN_FLT x257 = lh_qk * x197;
	const GEN_FLT x258 = x19 * x51;
	const GEN_FLT x259 = -x221 * x257 - x223 - x245 * x257 - x252 + x257 * x258;
	const GEN_FLT x260 = x259 * x57;
	const GEN_FLT x261 = -x212;
	const GEN_FLT x262 = x43 * (-x23 * x257 + x261);
	const GEN_FLT x263 = x60 * (x207 + x229 + x238 * x257 + x249 * x257 + x262);
	const GEN_FLT x264 = x204 + x234 * x257 + x243 + x253 * x257 + x37 * (-x257 * x66 + x261);
	const GEN_FLT x265 = lh_qk * x215;
	const GEN_FLT x266 =
		x78 * (x264 * x79 + x83 * (-x81 * (x19 * x219 + x238 * x265 + x241 + x249 * x265 + 2 * x262) -
								   x97 * (-x219 * x43 - x221 * x265 - x236 - x245 * x265 + x258 * x265)));
	*(out++) = x59 - x61 + x77 * (x62 * x73 - x74 * x76) - x84 + x85 * (-x59 + x61 + x84);
	*(out++) = x77 * (-x49 * x72 * x76 + x67 * x90) + x85 * (-x86 + x88 + x92) + x86 - x88 - x92;
	*(out++) = x77 * (x64 * x73 + x94 * x96) + x85 * (x93 + x95 + x98) - x93 - x95 - x98;
	*(out++) = -x117 - x121 - x125 + x77 * (x120 * x96 + x122 * x90) + x85 * (x117 + x121 + x125);
	*(out++) = -x142 - x146 - x149 + x77 * (x145 * x96 + x147 * x90) + x85 * (x142 + x146 + x149);
	*(out++) = -x163 - x167 - x170 + x77 * (x166 * x96 + x168 * x90) + x85 * (x163 + x167 + x170);
	*(out++) = -x182 - x186 - x189 + x77 * (x185 * x96 + x187 * x90) + x85 * (x182 + x186 + x189);
	*(out++) = -x191 - x60 + x85 * (x191 + x60);
	*(out++) = x192 * x85 - x192 + x77 * x90;
	*(out++) = -x193 + x57 - x77 * x96 + x85 * (x193 - x57);
	*(out++) = -x203 - x206 - x222 + x77 * (x202 * x96 + x210 * x90) + x85 * (x203 + x206 + x222);
	*(out++) = -x228 - x232 - x242 + x77 * (x231 * x96 + x235 * x90) + x85 * (x228 + x232 + x242);
	*(out++) = -x248 - x251 - x256 + x77 * (x247 * x96 + x254 * x90) + x85 * (x248 + x251 + x256);
	*(out++) = -x260 - x263 - x266 + x77 * (x259 * x96 + x264 * x90) + x85 * (x260 + x263 + x266);
}

static inline void gen_reproject_axis_y_jac_all(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
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
	const GEN_FLT x40 = x3 + x5;
	const GEN_FLT x41 = -x26 * x40 + 1;
	const GEN_FLT x42 = x20 + x21;
	const GEN_FLT x43 = x32 - x33;
	const GEN_FLT x44 = x13 + x15;
	const GEN_FLT x45 = obj_px + sensor_x * (-x18 * x44 + 1) + x19 * x43 + x36 * x42;
	const GEN_FLT x46 = x41 * x45;
	const GEN_FLT x47 = lh_px + x28 + x39 + x46;
	const GEN_FLT x48 = x47 * x47;
	const GEN_FLT x49 = lh_qi * lh_qw;
	const GEN_FLT x50 = lh_qj * lh_qk;
	const GEN_FLT x51 = x49 + x50;
	const GEN_FLT x52 = x26 * x45;
	const GEN_FLT x53 = x3 + x4;
	const GEN_FLT x54 = x26 * x53;
	const GEN_FLT x55 = -lh_pz - x2 * x52 - x25 * (1 - x54) - x38 * x51;
	const GEN_FLT x56 = x55 * x55;
	const GEN_FLT x57 = 1.0 / (x48 + x56);
	const GEN_FLT x58 = -lh_px - x28 - x39 - x46;
	const GEN_FLT x59 = x57 * x58;
	const GEN_FLT x60 = 2 * x57;
	const GEN_FLT x61 = x55 * x60;
	const GEN_FLT x62 = curve_0 * atan2(x47, x55);
	const GEN_FLT x63 = x29 + x30;
	const GEN_FLT x64 = x52 * x63;
	const GEN_FLT x65 = -x49 + x50;
	const GEN_FLT x66 = x27 * x65;
	const GEN_FLT x67 = -x26 * x6 + 1;
	const GEN_FLT x68 = x37 * x67;
	const GEN_FLT x69 = lh_py + x64 + x66 + x68;
	const GEN_FLT x70 = x56 + x69 * x69;
	const GEN_FLT x71 = 1.0 / x70;
	const GEN_FLT x72 = x55 * x71;
	const GEN_FLT x73 = x26 * x72;
	const GEN_FLT x74 = -lh_py - x64 - x66 - x68;
	const GEN_FLT x75 = x71 * x74;
	const GEN_FLT x76 = x26 * x75;
	const GEN_FLT x77 = pow(-x48 * x71 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x78 = tilt_0 / sqrt(x70);
	const GEN_FLT x79 = x26 * x69;
	const GEN_FLT x80 = x26 * x55;
	const GEN_FLT x81 = tilt_0 * x47 / pow(x70, 3.0 / 2.0);
	const GEN_FLT x82 = -x2 * x76 + x63 * x73 - x77 * (x41 * x78 + x81 * (x2 * x80 - x63 * x79));
	const GEN_FLT x83 = gibMag_0 * sin(gibPhase_0 - phase_0 - asin(x47 * x78) + atan2(x69, x55) + 1.5707963267948966);
	const GEN_FLT x84 = x51 * x8;
	const GEN_FLT x85 = x55 * x57 * x8;
	const GEN_FLT x86 = x26 * x78;
	const GEN_FLT x87 = (1.0 / 2.0) * x69;
	const GEN_FLT x88 = -x51 * x76 + x67 * x72 - x77 * (x31 * x86 + x81 * (x51 * x80 - x87 * (-x6 * x8 + 2)));
	const GEN_FLT x89 = x54 - 1;
	const GEN_FLT x90 = x58 * x60;
	const GEN_FLT x91 = (1.0 / 2.0) * x55;
	const GEN_FLT x92 = x65 * x73 + x75 * x89 - x77 * (x81 * (-x65 * x79 - x91 * (x53 * x8 - 2)) + x86 * x9);
	const GEN_FLT x93 = obj_qi * x19;
	const GEN_FLT x94 = obj_qj * x23;
	const GEN_FLT x95 = 2 / x17;
	const GEN_FLT x96 = obj_qw * x95;
	const GEN_FLT x97 = sensor_y * x96;
	const GEN_FLT x98 = sensor_z * x96;
	const GEN_FLT x99 = sensor_x * x96;
	const GEN_FLT x100 = x12 * x97 + x22 * x99 - x24 * x98 + x93 - x94;
	const GEN_FLT x101 = x100 * x26;
	const GEN_FLT x102 = obj_qi * x36;
	const GEN_FLT x103 = obj_qk * x23;
	const GEN_FLT x104 = -x102 + x103 - x16 * x97 + x34 * x99 + x35 * x98;
	const GEN_FLT x105 = x104 * x26;
	const GEN_FLT x106 = obj_qj * x36;
	const GEN_FLT x107 = obj_qk * x19;
	const GEN_FLT x108 = sensor_x * x44;
	const GEN_FLT x109 = x106 - x107 - x108 * x96 + x42 * x98 + x43 * x97;
	const GEN_FLT x110 = x101 * x9 + x105 * x31 + x109 * x41;
	const GEN_FLT x111 = x109 * x26;
	const GEN_FLT x112 = x100 * x89;
	const GEN_FLT x113 = -x105 * x51 - x111 * x2 + x112;
	const GEN_FLT x114 = x104 * x67;
	const GEN_FLT x115 = x109 * x8;
	const GEN_FLT x116 = x65 * x8;
	const GEN_FLT x117 = x113 * x75 + x72 * (x101 * x65 + x111 * x63 + x114) -
						 x77 * (x110 * x78 + x81 * (-x87 * (x100 * x116 + 2 * x114 + x115 * x63) -
													x91 * (-x104 * x84 + 2 * x112 - x115 * x2)));
	const GEN_FLT x118 = obj_qj * x19;
	const GEN_FLT x119 = obj_qk * x36;
	const GEN_FLT x120 = obj_qi * x95;
	const GEN_FLT x121 = sensor_z * x120;
	const GEN_FLT x122 = sensor_y * x120;
	const GEN_FLT x123 = -x108 * x120 + x118 + x119 + x121 * x42 + x122 * x43;
	const GEN_FLT x124 = obj_qw * x19;
	const GEN_FLT x125 = sensor_x * x120;
	const GEN_FLT x126 = 4 * x17;
	const GEN_FLT x127 = -obj_qi * x126;
	const GEN_FLT x128 = sensor_z * (-x120 * x24 + x127) + x103 + x12 * x122 + x124 + x125 * x22;
	const GEN_FLT x129 = x128 * x26;
	const GEN_FLT x130 = obj_qw * x36;
	const GEN_FLT x131 = sensor_y * (-x120 * x16 + x127) + x121 * x35 + x125 * x34 - x130 + x94;
	const GEN_FLT x132 = x131 * x26;
	const GEN_FLT x133 = x123 * x41 + x129 * x9 + x132 * x31;
	const GEN_FLT x134 = x123 * x26;
	const GEN_FLT x135 = x128 * x89;
	const GEN_FLT x136 = -x132 * x51 - x134 * x2 + x135;
	const GEN_FLT x137 = x131 * x67;
	const GEN_FLT x138 = x123 * x8;
	const GEN_FLT x139 = x136 * x75 + x72 * (x129 * x65 + x134 * x63 + x137) -
						 x77 * (x133 * x78 + x81 * (-x87 * (x116 * x128 + 2 * x137 + x138 * x63) -
													x91 * (-x131 * x84 + 2 * x135 - x138 * x2)));
	const GEN_FLT x140 = obj_qi * x23;
	const GEN_FLT x141 = obj_qj * x95;
	const GEN_FLT x142 = sensor_x * x141;
	const GEN_FLT x143 = sensor_y * x141;
	const GEN_FLT x144 = sensor_z * x141;
	const GEN_FLT x145 = x119 + x140 + x142 * x34 - x143 * x16 + x144 * x35;
	const GEN_FLT x146 = x145 * x26;
	const GEN_FLT x147 = obj_qw * x23;
	const GEN_FLT x148 = -obj_qj * x126;
	const GEN_FLT x149 = sensor_z * (-x141 * x24 + x148) + x107 + x12 * x143 + x142 * x22 - x147;
	const GEN_FLT x150 = x149 * x26;
	const GEN_FLT x151 = sensor_x * (-x141 * x44 + x148) + x130 + x143 * x43 + x144 * x42 + x93;
	const GEN_FLT x152 = x146 * x31 + x150 * x9 + x151 * x41;
	const GEN_FLT x153 = x151 * x26;
	const GEN_FLT x154 = x149 * x89;
	const GEN_FLT x155 = -x146 * x51 - x153 * x2 + x154;
	const GEN_FLT x156 = x145 * x67;
	const GEN_FLT x157 = x151 * x8;
	const GEN_FLT x158 = x155 * x75 + x72 * (x150 * x65 + x153 * x63 + x156) -
						 x77 * (x152 * x78 + x81 * (-x87 * (x116 * x149 + 2 * x156 + x157 * x63) -
													x91 * (-x145 * x84 + 2 * x154 - x157 * x2)));
	const GEN_FLT x159 = obj_qk * x95;
	const GEN_FLT x160 = sensor_y * x159;
	const GEN_FLT x161 = sensor_z * x159;
	const GEN_FLT x162 = sensor_x * x159;
	const GEN_FLT x163 = x118 + x12 * x160 + x140 - x161 * x24 + x162 * x22;
	const GEN_FLT x164 = x163 * x26;
	const GEN_FLT x165 = -obj_qk * x126;
	const GEN_FLT x166 = sensor_y * (-x159 * x16 + x165) + x106 + x147 + x161 * x35 + x162 * x34;
	const GEN_FLT x167 = x166 * x26;
	const GEN_FLT x168 = sensor_x * (-x159 * x44 + x165) + x102 - x124 + x160 * x43 + x161 * x42;
	const GEN_FLT x169 = x164 * x9 + x167 * x31 + x168 * x41;
	const GEN_FLT x170 = x163 * x89;
	const GEN_FLT x171 = x168 * x26;
	const GEN_FLT x172 = -x167 * x51 + x170 - x171 * x2;
	const GEN_FLT x173 = x166 * x67;
	const GEN_FLT x174 = x168 * x8;
	const GEN_FLT x175 = x172 * x75 + x72 * (x164 * x65 + x171 * x63 + x173) -
						 x77 * (x169 * x78 + x81 * (-x87 * (x116 * x163 + 2 * x173 + x174 * x63) -
													x91 * (-x166 * x84 + 2 * x170 - x174 * x2)));
	const GEN_FLT x176 = x77 * x78;
	const GEN_FLT x177 = x77 * x81;
	const GEN_FLT x178 = -x177 * x74 + x72;
	const GEN_FLT x179 = -x177 * x55 - x75;
	const GEN_FLT x180 = lh_qi * x38;
	const GEN_FLT x181 = lh_qj * x52;
	const GEN_FLT x182 = 1.0 / x7;
	const GEN_FLT x183 = 2 * x182;
	const GEN_FLT x184 = lh_qw * x183;
	const GEN_FLT x185 = x184 * x25;
	const GEN_FLT x186 = x184 * x37;
	const GEN_FLT x187 = x184 * x45;
	const GEN_FLT x188 = -x180 + x181 + x185 * x53 - x186 * x51 - x187 * x2;
	const GEN_FLT x189 = lh_qj * x27;
	const GEN_FLT x190 = -lh_qk * x38;
	const GEN_FLT x191 = x40 * x45;
	const GEN_FLT x192 = -x184 * x191 + x185 * x9 + x186 * x31 + x189 + x190;
	const GEN_FLT x193 = lh_qi * x27;
	const GEN_FLT x194 = lh_qk * x52;
	const GEN_FLT x195 = lh_qi * x8;
	const GEN_FLT x196 = lh_qk * x8;
	const GEN_FLT x197 = x196 * x45;
	const GEN_FLT x198 = x37 * x6;
	const GEN_FLT x199 = 4 * x182;
	const GEN_FLT x200 = lh_qw * x199;
	const GEN_FLT x201 = x200 * x45;
	const GEN_FLT x202 = x200 * x25;
	const GEN_FLT x203 = lh_qj * x8;
	const GEN_FLT x204 = x203 * x45;
	const GEN_FLT x205 = x37 * x51;
	const GEN_FLT x206 =
		x188 * x75 + x72 * (x185 * x65 - x186 * x6 + x187 * x63 - x193 + x194) -
		x77 * (x192 * x78 + x81 * (-x87 * (-x195 * x25 + x197 - x198 * x200 + x201 * x63 + x202 * x65) -
								   x91 * (-x195 * x37 - x2 * x201 - x200 * x205 + x202 * x53 + x204)));
	const GEN_FLT x207 = lh_qj * x38;
	const GEN_FLT x208 = lh_qk * x27;
	const GEN_FLT x209 = lh_qi * x183;
	const GEN_FLT x210 = x209 * x25;
	const GEN_FLT x211 = x31 * x37;
	const GEN_FLT x212 = -x191 * x209 + x207 + x208 + x209 * x211 + x210 * x9;
	const GEN_FLT x213 = -lh_qw * x38;
	const GEN_FLT x214 = x2 * x45;
	const GEN_FLT x215 = x25 * (x195 + x209 * x53);
	const GEN_FLT x216 = -x194 - x205 * x209 - x209 * x214 + x213 + x215;
	const GEN_FLT x217 = lh_qw * x27;
	const GEN_FLT x218 = x45 * x63;
	const GEN_FLT x219 = x37 * (-x195 - x209 * x6);
	const GEN_FLT x220 = lh_qw * x8;
	const GEN_FLT x221 = lh_qi * x199;
	const GEN_FLT x222 = x25 * x65;
	const GEN_FLT x223 = x216 * x75 + x72 * (x181 + x209 * x218 + x210 * x65 - x217 + x219) -
						 x77 * (x212 * x78 + x81 * (-x87 * (x204 + x218 * x221 + 2 * x219 - x220 * x25 + x221 * x222) -
													x91 * (-x197 - x205 * x221 - x214 * x221 + 2 * x215 - x220 * x37)));
	const GEN_FLT x224 = lh_qw * x52;
	const GEN_FLT x225 = lh_qj * x183;
	const GEN_FLT x226 = x25 * (x203 + x225 * x53);
	const GEN_FLT x227 = x190 - x205 * x225 - x214 * x225 + x224 + x226;
	const GEN_FLT x228 = x25 * x9;
	const GEN_FLT x229 = x180 + x211 * x225 + x217 + x225 * x228 + x45 * (-x203 - x225 * x40);
	const GEN_FLT x230 = lh_qi * x52;
	const GEN_FLT x231 = x195 * x45;
	const GEN_FLT x232 = lh_qj * x199;
	const GEN_FLT x233 = x220 * x45;
	const GEN_FLT x234 =
		x227 * x75 + x72 * (-x198 * x225 + x208 + x218 * x225 + x222 * x225 + x230) -
		x77 * (x229 * x78 + x81 * (-x87 * (x196 * x25 - x198 * x232 + x218 * x232 + x222 * x232 + x231) -
								   x91 * (-x196 * x37 - x205 * x232 - x214 * x232 + 2 * x226 + x233)));
	const GEN_FLT x235 = lh_qk * x183;
	const GEN_FLT x236 = x25 * x53;
	const GEN_FLT x237 = -x205 * x235 - x207 - x214 * x235 - x230 + x235 * x236;
	const GEN_FLT x238 = -x196;
	const GEN_FLT x239 = x193 + x211 * x235 + x213 + x228 * x235 + x45 * (-x235 * x40 + x238);
	const GEN_FLT x240 = x37 * (-x235 * x6 + x238);
	const GEN_FLT x241 = lh_qk * x199;
	const GEN_FLT x242 =
		x237 * x75 + x72 * (x189 + x218 * x235 + x222 * x235 + x224 + x240) -
		x77 * (x239 * x78 + x81 * (-x87 * (x203 * x25 + x218 * x241 + x222 * x241 + x233 + 2 * x240) -
								   x91 * (-x203 * x37 - x205 * x241 - x214 * x241 - x231 + x236 * x241)));
	*(out++) = x62 * (-x2 * x59 * x8 + x41 * x61) + x82 * x83 + x82;
	*(out++) = x62 * (x31 * x85 - x59 * x84) + x83 * x88 + x88;
	*(out++) = x62 * (x85 * x9 + x89 * x90) + x83 * x92 + x92;
	*(out++) = x117 * x83 + x117 + x62 * (x110 * x61 + x113 * x90);
	*(out++) = x139 * x83 + x139 + x62 * (x133 * x61 + x136 * x90);
	*(out++) = x158 * x83 + x158 + x62 * (x152 * x61 + x155 * x90);
	*(out++) = x175 * x83 + x175 + x62 * (x169 * x61 + x172 * x90);
	*(out++) = -x176 * x83 - x176 + x61 * x62;
	*(out++) = x178 * x83 + x178;
	*(out++) = x179 * x83 + x179 - x62 * x90;
	*(out++) = x206 * x83 + x206 + x62 * (x188 * x90 + x192 * x61);
	*(out++) = x223 * x83 + x223 + x62 * (x212 * x61 + x216 * x90);
	*(out++) = x234 * x83 + x234 + x62 * (x227 * x90 + x229 * x61);
	*(out++) = x242 * x83 + x242 + x62 * (x237 * x90 + x239 * x61);
}

static inline void gen_reproject_jac_lh_p_gen2(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
											   const FLT phase_0, const FLT phase_1, const FLT tilt_0, const FLT tilt_1,
											   const FLT curve_0, const FLT curve_1, const FLT gibPhase_0,
											   const FLT gibPhase_1, const FLT gibMag_0, const FLT gibMag_1,
											   const FLT ogeePhase_0, const FLT ogeePhase_1, const FLT ogeeMag_0,
											   const FLT ogeeMag_1) {
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
	const GEN_FLT x0 = lh_qi * lh_qw;
	const GEN_FLT x1 = lh_qj * lh_qk;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = obj_qi * obj_qj;
	const GEN_FLT x4 = obj_qk * obj_qw;
	const GEN_FLT x5 = obj_qi * obj_qi;
	const GEN_FLT x6 = obj_qj * obj_qj;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = x6 + x7;
	const GEN_FLT x9 = 2 * sqrt(obj_qw * obj_qw + x5 + x8);
	const GEN_FLT x10 = sensor_x * x9;
	const GEN_FLT x11 = obj_qj * obj_qk;
	const GEN_FLT x12 = obj_qi * obj_qw;
	const GEN_FLT x13 = sensor_z * x9;
	const GEN_FLT x14 = obj_py + sensor_y * (-x9 * (x5 + x7) + 1) + x10 * (x3 + x4) + x13 * (x11 - x12);
	const GEN_FLT x15 = lh_qi * lh_qi;
	const GEN_FLT x16 = lh_qj * lh_qj;
	const GEN_FLT x17 = lh_qk * lh_qk;
	const GEN_FLT x18 = x16 + x17;
	const GEN_FLT x19 = sqrt(lh_qw * lh_qw + x15 + x18);
	const GEN_FLT x20 = 2 * x19;
	const GEN_FLT x21 = x14 * x20;
	const GEN_FLT x22 = x2 * x21;
	const GEN_FLT x23 = lh_qi * lh_qk;
	const GEN_FLT x24 = lh_qj * lh_qw;
	const GEN_FLT x25 = x23 - x24;
	const GEN_FLT x26 = obj_qi * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qw;
	const GEN_FLT x28 = sensor_y * x9;
	const GEN_FLT x29 = obj_px + sensor_x * (-x8 * x9 + 1) + x13 * (x26 + x27) + x28 * (x3 - x4);
	const GEN_FLT x30 = x20 * x29;
	const GEN_FLT x31 = x25 * x30;
	const GEN_FLT x32 = x15 + x16;
	const GEN_FLT x33 = obj_pz + sensor_z * (-x9 * (x5 + x6) + 1) + x10 * (x26 - x27) + x28 * (x11 + x12);
	const GEN_FLT x34 = x33 * (-x20 * x32 + 1);
	const GEN_FLT x35 = x23 + x24;
	const GEN_FLT x36 = x20 * x33;
	const GEN_FLT x37 = x35 * x36;
	const GEN_FLT x38 = lh_qi * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qw;
	const GEN_FLT x40 = x38 - x39;
	const GEN_FLT x41 = x21 * x40;
	const GEN_FLT x42 = x29 * (-x18 * x20 + 1);
	const GEN_FLT x43 = lh_px + x37 + x41 + x42;
	const GEN_FLT x44 = -lh_pz - x22 - x31 - x34;
	const GEN_FLT x45 = x43 * x43 + x44 * x44;
	const GEN_FLT x46 = 1.0 / x45;
	const GEN_FLT x47 = x46 * (lh_pz + x22 + x31 + x34);
	const GEN_FLT x48 = x38 + x39;
	const GEN_FLT x49 = x30 * x48;
	const GEN_FLT x50 = -x0 + x1;
	const GEN_FLT x51 = x36 * x50;
	const GEN_FLT x52 = x15 + x17;
	const GEN_FLT x53 = x14 * (-x20 * x52 + 1);
	const GEN_FLT x54 = lh_py + x49 + x51 + x53;
	const GEN_FLT x55 = tilt_0 + 0.52359877559829882;
	const GEN_FLT x56 = tan(x55);
	const GEN_FLT x57 = pow(x45, -1.0 / 2.0);
	const GEN_FLT x58 = x56 * x57;
	const GEN_FLT x59 = x54 * x58;
	const GEN_FLT x60 = cos(x55);
	const GEN_FLT x61 = 1.0 / x60;
	const GEN_FLT x62 = x54 * x54;
	const GEN_FLT x63 = x45 + x62;
	const GEN_FLT x64 = pow(x63, -1.0 / 2.0);
	const GEN_FLT x65 = x61 * x64;
	const GEN_FLT x66 = asin(x54 * x65);
	const GEN_FLT x67 = 8.0108022e-6 * x66;
	const GEN_FLT x68 = -x67 - 8.0108022e-6;
	const GEN_FLT x69 = x66 * x68 + 0.0028679863;
	const GEN_FLT x70 = x66 * x69 + 5.3685255000000001e-6;
	const GEN_FLT x71 = x66 * x70 + 0.0076069798000000001;
	const GEN_FLT x72 = x66 * x66;
	const GEN_FLT x73 = atan2(x44, x43);
	const GEN_FLT x74 = ogeePhase_0 + x73 - asin(x59);
	const GEN_FLT x75 = ogeeMag_0 * sin(x74);
	const GEN_FLT x76 = curve_0 + x75;
	const GEN_FLT x77 = x66 * x71;
	const GEN_FLT x78 = -1.60216044e-5 * x66 - 8.0108022e-6;
	const GEN_FLT x79 = x66 * x78 + x69;
	const GEN_FLT x80 = x66 * x79 + x70;
	const GEN_FLT x81 = x66 * x80 + x71;
	const GEN_FLT x82 = sin(x55);
	const GEN_FLT x83 = x82 * (x66 * x81 + x77);
	const GEN_FLT x84 = x60 - x76 * x83;
	const GEN_FLT x85 = 1.0 / x84;
	const GEN_FLT x86 = x76 * x85;
	const GEN_FLT x87 = x72 * x86;
	const GEN_FLT x88 = x59 + x71 * x87;
	const GEN_FLT x89 = pow(1 - x88 * x88, -1.0 / 2.0);
	const GEN_FLT x90 = -lh_px - x37 - x41 - x42;
	const GEN_FLT x91 = x54 / pow(x45, 3.0 / 2.0);
	const GEN_FLT x92 = x56 * x91;
	const GEN_FLT x93 = x90 * x92;
	const GEN_FLT x94 = x62 / x63;
	const GEN_FLT x95 = pow(-x94 / (x60 * x60) + 1, -1.0 / 2.0);
	const GEN_FLT x96 = x54 / pow(x63, 3.0 / 2.0);
	const GEN_FLT x97 = x61 * x96;
	const GEN_FLT x98 = x95 * x97;
	const GEN_FLT x99 = x90 * x98;
	const GEN_FLT x100 = 2 * x77 * x86;
	const GEN_FLT x101 = x46 * x62;
	const GEN_FLT x102 = pow(-x101 * x56 * x56 + 1, -1.0 / 2.0);
	const GEN_FLT x103 = -x102 * x93 + x47;
	const GEN_FLT x104 = ogeeMag_0 * cos(x74);
	const GEN_FLT x105 = x71 * x72;
	const GEN_FLT x106 = x104 * x105 * x85;
	const GEN_FLT x107 = x68 * x99;
	const GEN_FLT x108 = x66 * (x107 - x67 * x99) + x69 * x99;
	const GEN_FLT x109 = x108 * x66 + x70 * x99;
	const GEN_FLT x110 = x104 * x83;
	const GEN_FLT x111 = 2.40324066e-5 * x66;
	const GEN_FLT x112 = x82 * (-curve_0 - x75);
	const GEN_FLT x113 = x105 * x76 / ((x84 * x84));
	const GEN_FLT x114 =
		x47 - x89 * (x100 * x99 + x103 * x106 + x109 * x87 +
					 x113 * (x103 * x110 -
							 x112 * (x109 * x66 +
									 x66 * (x109 + x66 * (x108 + x66 * (x107 - x111 * x99 + x78 * x99) + x79 * x99) +
											x80 * x99) +
									 x71 * x99 + x81 * x99)) +
					 x93);
	const GEN_FLT x115 = gibMag_0 * cos(gibPhase_0 + x73 - asin(x88));
	const GEN_FLT x116 = x102 * x58;
	const GEN_FLT x117 = -lh_py - x49 - x51 - x53;
	const GEN_FLT x118 = x95 * (x117 * x97 + x65);
	const GEN_FLT x119 = x118 * x68;
	const GEN_FLT x120 = x118 * x69 + x66 * (-x118 * x67 + x119);
	const GEN_FLT x121 = x118 * x70 + x120 * x66;
	const GEN_FLT x122 =
		x89 *
		(x100 * x118 - x106 * x116 +
		 x113 * (-x110 * x116 - x112 * (x118 * x71 + x118 * x81 + x121 * x66 +
										x66 * (x118 * x80 + x121 +
											   x66 * (x118 * x79 + x120 + x66 * (-x111 * x118 + x118 * x78 + x119))))) +
		 x121 * x87 + x58);
	const GEN_FLT x123 = x43 * x46;
	const GEN_FLT x124 = -x123;
	const GEN_FLT x125 = x44 * x92;
	const GEN_FLT x126 = x44 * x98;
	const GEN_FLT x127 = -x102 * x125 + x124;
	const GEN_FLT x128 = x126 * x68;
	const GEN_FLT x129 = x126 * x69 + x66 * (-x126 * x67 + x128);
	const GEN_FLT x130 = x126 * x70 + x129 * x66;
	const GEN_FLT x131 =
		x124 - x89 * (x100 * x126 + x106 * x127 +
					  x113 * (x110 * x127 -
							  x112 * (x126 * x71 + x126 * x81 + x130 * x66 +
									  x66 * (x126 * x80 + x130 +
											 x66 * (x126 * x79 + x129 + x66 * (-x111 * x126 + x126 * x78 + x128))))) +
					  x125 + x130 * x87);
	const GEN_FLT x132 = lh_qi * x36;
	const GEN_FLT x133 = lh_qk * x30;
	const GEN_FLT x134 = 1.0 / x19;
	const GEN_FLT x135 = 2 * x134;
	const GEN_FLT x136 = lh_qw * x135;
	const GEN_FLT x137 = x14 * x52;
	const GEN_FLT x138 = x29 * x48;
	const GEN_FLT x139 = x33 * x50;
	const GEN_FLT x140 = -x132 + x133 - x136 * x137 + x136 * x138 + x136 * x139;
	const GEN_FLT x141 = 4 * x19;
	const GEN_FLT x142 = lh_qi * x141;
	const GEN_FLT x143 = x142 * x33;
	const GEN_FLT x144 = lh_qk * x141;
	const GEN_FLT x145 = x144 * x29;
	const GEN_FLT x146 = 4 * x134;
	const GEN_FLT x147 = lh_qw * x146;
	const GEN_FLT x148 = (1.0 / 2.0) * x54;
	const GEN_FLT x149 = lh_qj * x141;
	const GEN_FLT x150 = x149 * x33;
	const GEN_FLT x151 = -x14 * x144;
	const GEN_FLT x152 = x18 * x29;
	const GEN_FLT x153 = x147 * x33;
	const GEN_FLT x154 = x14 * x147;
	const GEN_FLT x155 = (1.0 / 2.0) * x43;
	const GEN_FLT x156 = x14 * x142;
	const GEN_FLT x157 = x149 * x29;
	const GEN_FLT x158 = x25 * x29;
	const GEN_FLT x159 = (1.0 / 2.0) * x44;
	const GEN_FLT x160 = -x155 * (-x147 * x152 + x150 + x151 + x153 * x35 + x154 * x40) -
						 x159 * (-x147 * x158 + x153 * x32 - x154 * x2 - x156 + x157);
	const GEN_FLT x161 = -x148 * (-x137 * x147 + x138 * x147 + x139 * x147 - x143 + x145) + x160;
	const GEN_FLT x162 = x95 * (x140 * x65 + x161 * x97);
	const GEN_FLT x163 = x162 * x68;
	const GEN_FLT x164 = x162 * x69 + x66 * (-x162 * x67 + x163);
	const GEN_FLT x165 = x162 * x70 + x164 * x66;
	const GEN_FLT x166 = x140 * x58 + x160 * x92;
	const GEN_FLT x167 = lh_qi * x21;
	const GEN_FLT x168 = lh_qj * x30;
	const GEN_FLT x169 = x136 * x33;
	const GEN_FLT x170 = x136 * x14;
	const GEN_FLT x171 = lh_qj * x36;
	const GEN_FLT x172 = -lh_qk * x21;
	const GEN_FLT x173 = x123 * (-x136 * x158 - x167 + x168 + x169 * x32 - x170 * x2) +
						 x47 * (-x136 * x152 + x169 * x35 + x170 * x40 + x171 + x172);
	const GEN_FLT x174 = -x102 * x166 + x173;
	const GEN_FLT x175 =
		x173 - x89 * (x100 * x162 + x106 * x174 +
					  x113 * (x110 * x174 -
							  x112 * (x162 * x71 + x162 * x81 + x165 * x66 +
									  x66 * (x162 * x80 + x165 +
											 x66 * (x162 * x79 + x164 + x66 * (-x111 * x162 + x162 * x78 + x163))))) +
					  x165 * x87 + x166);
	const GEN_FLT x176 = lh_qw * x36;
	const GEN_FLT x177 = lh_qi * x135;
	const GEN_FLT x178 = x14 * (-x142 - x177 * x52);
	const GEN_FLT x179 = x138 * x177 + x139 * x177 + x168 - x176 + x178;
	const GEN_FLT x180 = lh_qw * x141;
	const GEN_FLT x181 = x180 * x33;
	const GEN_FLT x182 = lh_qi * x146;
	const GEN_FLT x183 = x14 * x149;
	const GEN_FLT x184 = x144 * x33;
	const GEN_FLT x185 = x33 * x35;
	const GEN_FLT x186 = x14 * x182;
	const GEN_FLT x187 = -x14 * x180;
	const GEN_FLT x188 = x33 * (x142 + x177 * x32);
	const GEN_FLT x189 = -x155 * (-x152 * x182 + x182 * x185 + x183 + x184 + x186 * x40) -
						 x159 * (-x145 - x158 * x182 - x186 * x2 + x187 + 2 * x188);
	const GEN_FLT x190 = -x148 * (x138 * x182 + x139 * x182 + x157 + 2 * x178 - x181) + x189;
	const GEN_FLT x191 = x95 * (x179 * x65 + x190 * x97);
	const GEN_FLT x192 = x191 * x68;
	const GEN_FLT x193 = x191 * x69 + x66 * (-x191 * x67 + x192);
	const GEN_FLT x194 = x191 * x70 + x193 * x66;
	const GEN_FLT x195 = x179 * x58 + x189 * x92;
	const GEN_FLT x196 = -lh_qw * x21;
	const GEN_FLT x197 = x14 * x177;
	const GEN_FLT x198 = lh_qj * x21;
	const GEN_FLT x199 = lh_qk * x36;
	const GEN_FLT x200 = x123 * (-x133 - x158 * x177 + x188 + x196 - x197 * x2) +
						 x47 * (-x152 * x177 + x177 * x185 + x197 * x40 + x198 + x199);
	const GEN_FLT x201 = -x102 * x195 + x200;
	const GEN_FLT x202 =
		x200 - x89 * (x100 * x191 + x106 * x201 +
					  x113 * (x110 * x201 -
							  x112 * (x191 * x71 + x191 * x81 + x194 * x66 +
									  x66 * (x191 * x80 + x194 +
											 x66 * (x191 * x79 + x193 + x66 * (-x111 * x191 + x191 * x78 + x192))))) +
					  x194 * x87 + x195);
	const GEN_FLT x203 = lh_qi * x30;
	const GEN_FLT x204 = lh_qj * x135;
	const GEN_FLT x205 = -x137 * x204 + x138 * x204 + x139 * x204 + x199 + x203;
	const GEN_FLT x206 = x142 * x29;
	const GEN_FLT x207 = lh_qj * x146;
	const GEN_FLT x208 = x14 * x40;
	const GEN_FLT x209 = x29 * (-x149 - x18 * x204);
	const GEN_FLT x210 = x180 * x29;
	const GEN_FLT x211 = x14 * x2;
	const GEN_FLT x212 = x33 * (x149 + x204 * x32);
	const GEN_FLT x213 = -x155 * (x156 + x181 + x185 * x207 + x207 * x208 + 2 * x209) -
						 x159 * (x151 - x158 * x207 - x207 * x211 + x210 + 2 * x212);
	const GEN_FLT x214 = -x148 * (-x137 * x207 + x138 * x207 + x139 * x207 + x184 + x206) + x213;
	const GEN_FLT x215 = x95 * (x205 * x65 + x214 * x97);
	const GEN_FLT x216 = x215 * x68;
	const GEN_FLT x217 = x215 * x69 + x66 * (-x215 * x67 + x216);
	const GEN_FLT x218 = x215 * x70 + x217 * x66;
	const GEN_FLT x219 = x205 * x58 + x213 * x92;
	const GEN_FLT x220 = lh_qw * x30;
	const GEN_FLT x221 = x123 * (-x158 * x204 + x172 - x204 * x211 + x212 + x220) +
						 x47 * (x167 + x176 + x185 * x204 + x204 * x208 + x209);
	const GEN_FLT x222 = -x102 * x219 + x221;
	const GEN_FLT x223 =
		x221 - x89 * (x100 * x215 + x106 * x222 +
					  x113 * (x110 * x222 -
							  x112 * (x215 * x71 + x215 * x81 + x218 * x66 +
									  x66 * (x215 * x80 + x218 +
											 x66 * (x215 * x79 + x217 + x66 * (-x111 * x215 + x215 * x78 + x216))))) +
					  x218 * x87 + x219);
	const GEN_FLT x224 = lh_qk * x135;
	const GEN_FLT x225 = -x144;
	const GEN_FLT x226 = x14 * (-x224 * x52 + x225);
	const GEN_FLT x227 = x138 * x224 + x139 * x224 + x171 + x220 + x226;
	const GEN_FLT x228 = lh_qk * x146;
	const GEN_FLT x229 = x32 * x33;
	const GEN_FLT x230 = x29 * (-x18 * x224 + x225);
	const GEN_FLT x231 = -x155 * (x143 + x185 * x228 + x187 + x208 * x228 + 2 * x230) -
						 x159 * (-x158 * x228 - x183 - x206 - x211 * x228 + x228 * x229);
	const GEN_FLT x232 = -x148 * (x138 * x228 + x139 * x228 + x150 + x210 + 2 * x226) + x231;
	const GEN_FLT x233 = x95 * (x227 * x65 + x232 * x97);
	const GEN_FLT x234 = x233 * x68;
	const GEN_FLT x235 = x233 * x69 + x66 * (-x233 * x67 + x234);
	const GEN_FLT x236 = x233 * x70 + x235 * x66;
	const GEN_FLT x237 = x227 * x58 + x231 * x92;
	const GEN_FLT x238 = x123 * (-x158 * x224 - x198 - x203 - x211 * x224 + x224 * x229) +
						 x47 * (x132 + x185 * x224 + x196 + x208 * x224 + x230);
	const GEN_FLT x239 = -x102 * x237 + x238;
	const GEN_FLT x240 =
		x238 - x89 * (x100 * x233 + x106 * x239 +
					  x113 * (x110 * x239 -
							  x112 * (x233 * x71 + x233 * x81 + x236 * x66 +
									  x66 * (x233 * x80 + x236 +
											 x66 * (x233 * x79 + x235 + x66 * (-x111 * x233 + x233 * x78 + x234))))) +
					  x236 * x87 + x237);
	const GEN_FLT x241 = tilt_1 - 0.52359877559829882;
	const GEN_FLT x242 = tan(x241);
	const GEN_FLT x243 = x242 * x57;
	const GEN_FLT x244 = x243 * x54;
	const GEN_FLT x245 = cos(x241);
	const GEN_FLT x246 = 1.0 / x245;
	const GEN_FLT x247 = x246 * x64;
	const GEN_FLT x248 = asin(x247 * x54);
	const GEN_FLT x249 = 8.0108022e-6 * x248;
	const GEN_FLT x250 = -x249 - 8.0108022e-6;
	const GEN_FLT x251 = x248 * x250 + 0.0028679863;
	const GEN_FLT x252 = x248 * x251 + 5.3685255000000001e-6;
	const GEN_FLT x253 = x248 * x252 + 0.0076069798000000001;
	const GEN_FLT x254 = x248 * x248;
	const GEN_FLT x255 = ogeePhase_1 + x73 - asin(x244);
	const GEN_FLT x256 = ogeeMag_1 * sin(x255);
	const GEN_FLT x257 = curve_1 + x256;
	const GEN_FLT x258 = x248 * x253;
	const GEN_FLT x259 = -1.60216044e-5 * x248 - 8.0108022e-6;
	const GEN_FLT x260 = x248 * x259 + x251;
	const GEN_FLT x261 = x248 * x260 + x252;
	const GEN_FLT x262 = x248 * x261 + x253;
	const GEN_FLT x263 = sin(x241);
	const GEN_FLT x264 = x263 * (x248 * x262 + x258);
	const GEN_FLT x265 = x245 - x257 * x264;
	const GEN_FLT x266 = 1.0 / x265;
	const GEN_FLT x267 = x257 * x266;
	const GEN_FLT x268 = x254 * x267;
	const GEN_FLT x269 = x244 + x253 * x268;
	const GEN_FLT x270 = pow(1 - x269 * x269, -1.0 / 2.0);
	const GEN_FLT x271 = x242 * x91;
	const GEN_FLT x272 = x271 * x90;
	const GEN_FLT x273 = pow(-x94 / (x245 * x245) + 1, -1.0 / 2.0);
	const GEN_FLT x274 = x246 * x96;
	const GEN_FLT x275 = x273 * x274;
	const GEN_FLT x276 = x275 * x90;
	const GEN_FLT x277 = 2 * x258 * x267;
	const GEN_FLT x278 = pow(-x101 * x242 * x242 + 1, -1.0 / 2.0);
	const GEN_FLT x279 = -x272 * x278 + x47;
	const GEN_FLT x280 = ogeeMag_1 * cos(x255);
	const GEN_FLT x281 = x253 * x254;
	const GEN_FLT x282 = x266 * x280 * x281;
	const GEN_FLT x283 = x250 * x276;
	const GEN_FLT x284 = x248 * (-x249 * x276 + x283) + x251 * x276;
	const GEN_FLT x285 = x248 * x284 + x252 * x276;
	const GEN_FLT x286 = x264 * x280;
	const GEN_FLT x287 = 2.40324066e-5 * x248;
	const GEN_FLT x288 = x263 * (-curve_1 - x256);
	const GEN_FLT x289 = x257 * x281 / ((x265 * x265));
	const GEN_FLT x290 =
		-x270 * (x268 * x285 + x272 + x276 * x277 + x279 * x282 +
				 x289 * (x279 * x286 -
						 x288 * (x248 * x285 +
								 x248 * (x248 * (x248 * (x259 * x276 - x276 * x287 + x283) + x260 * x276 + x284) +
										 x261 * x276 + x285) +
								 x253 * x276 + x262 * x276))) +
		x47;
	const GEN_FLT x291 = gibMag_1 * cos(gibPhase_1 + x73 - asin(x269));
	const GEN_FLT x292 = x243 * x278;
	const GEN_FLT x293 = x273 * (x117 * x274 + x247);
	const GEN_FLT x294 = x250 * x293;
	const GEN_FLT x295 = x248 * (-x249 * x293 + x294) + x251 * x293;
	const GEN_FLT x296 = x248 * x295 + x252 * x293;
	const GEN_FLT x297 =
		x270 * (x243 + x268 * x296 + x277 * x293 - x282 * x292 +
				x289 * (-x286 * x292 -
						x288 * (x248 * x296 +
								x248 * (x248 * (x248 * (x259 * x293 - x287 * x293 + x294) + x260 * x293 + x295) +
										x261 * x293 + x296) +
								x253 * x293 + x262 * x293)));
	const GEN_FLT x298 = x271 * x44;
	const GEN_FLT x299 = x275 * x44;
	const GEN_FLT x300 = x124 - x278 * x298;
	const GEN_FLT x301 = x250 * x299;
	const GEN_FLT x302 = x248 * (-x249 * x299 + x301) + x251 * x299;
	const GEN_FLT x303 = x248 * x302 + x252 * x299;
	const GEN_FLT x304 =
		x124 - x270 * (x268 * x303 + x277 * x299 + x282 * x300 +
					   x289 * (x286 * x300 -
							   x288 * (x248 * x303 +
									   x248 * (x248 * (x248 * (x259 * x299 - x287 * x299 + x301) + x260 * x299 + x302) +
											   x261 * x299 + x303) +
									   x253 * x299 + x262 * x299)) +
					   x298);
	const GEN_FLT x305 = x273 * (x140 * x247 + x161 * x274);
	const GEN_FLT x306 = x250 * x305;
	const GEN_FLT x307 = x248 * (-x249 * x305 + x306) + x251 * x305;
	const GEN_FLT x308 = x248 * x307 + x252 * x305;
	const GEN_FLT x309 = x140 * x243 + x160 * x271;
	const GEN_FLT x310 = x173 - x278 * x309;
	const GEN_FLT x311 =
		x173 - x270 * (x268 * x308 + x277 * x305 + x282 * x310 +
					   x289 * (x286 * x310 -
							   x288 * (x248 * x308 +
									   x248 * (x248 * (x248 * (x259 * x305 - x287 * x305 + x306) + x260 * x305 + x307) +
											   x261 * x305 + x308) +
									   x253 * x305 + x262 * x305)) +
					   x309);
	const GEN_FLT x312 = x273 * (x179 * x247 + x190 * x274);
	const GEN_FLT x313 = x250 * x312;
	const GEN_FLT x314 = x248 * (-x249 * x312 + x313) + x251 * x312;
	const GEN_FLT x315 = x248 * x314 + x252 * x312;
	const GEN_FLT x316 = x179 * x243 + x189 * x271;
	const GEN_FLT x317 = x200 - x278 * x316;
	const GEN_FLT x318 =
		x200 - x270 * (x268 * x315 + x277 * x312 + x282 * x317 +
					   x289 * (x286 * x317 -
							   x288 * (x248 * x315 +
									   x248 * (x248 * (x248 * (x259 * x312 - x287 * x312 + x313) + x260 * x312 + x314) +
											   x261 * x312 + x315) +
									   x253 * x312 + x262 * x312)) +
					   x316);
	const GEN_FLT x319 = x273 * (x205 * x247 + x214 * x274);
	const GEN_FLT x320 = x250 * x319;
	const GEN_FLT x321 = x248 * (-x249 * x319 + x320) + x251 * x319;
	const GEN_FLT x322 = x248 * x321 + x252 * x319;
	const GEN_FLT x323 = x205 * x243 + x213 * x271;
	const GEN_FLT x324 = x221 - x278 * x323;
	const GEN_FLT x325 =
		x221 - x270 * (x268 * x322 + x277 * x319 + x282 * x324 +
					   x289 * (x286 * x324 -
							   x288 * (x248 * x322 +
									   x248 * (x248 * (x248 * (x259 * x319 - x287 * x319 + x320) + x260 * x319 + x321) +
											   x261 * x319 + x322) +
									   x253 * x319 + x262 * x319)) +
					   x323);
	const GEN_FLT x326 = x273 * (x227 * x247 + x232 * x274);
	const GEN_FLT x327 = x250 * x326;
	const GEN_FLT x328 = x248 * (-x249 * x326 + x327) + x251 * x326;
	const GEN_FLT x329 = x248 * x328 + x252 * x326;
	const GEN_FLT x330 = x227 * x243 + x231 * x271;
	const GEN_FLT x331 = x238 - x278 * x330;
	const GEN_FLT x332 =
		x238 - x270 * (x268 * x329 + x277 * x326 + x282 * x331 +
					   x289 * (x286 * x331 -
							   x288 * (x248 * x329 +
									   x248 * (x248 * (x248 * (x259 * x326 - x287 * x326 + x327) + x260 * x326 + x328) +
											   x261 * x326 + x329) +
									   x253 * x326 + x262 * x326)) +
					   x330);
	*(out++) = x114 * x115 + x114;
	*(out++) = -x115 * x122 - x122;
	*(out++) = x115 * x131 + x131;
	*(out++) = x115 * x175 + x175;
	*(out++) = x115 * x202 + x202;
	*(out++) = x115 * x223 + x223;
	*(out++) = x115 * x240 + x240;
	*(out++) = x290 * x291 + x290;
	*(out++) = -x291 * x297 - x297;
	*(out++) = x291 * x304 + x304;
	*(out++) = x291 * x311 + x311;
	*(out++) = x291 * x318 + x318;
	*(out++) = x291 * x325 + x325;
	*(out++) = x291 * x332 + x332;
}

static inline void gen_reproject_axis_x_jac_lh_p_gen2(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
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
	const GEN_FLT x0 = lh_qi * lh_qw;
	const GEN_FLT x1 = lh_qj * lh_qk;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = obj_qi * obj_qj;
	const GEN_FLT x4 = obj_qk * obj_qw;
	const GEN_FLT x5 = obj_qi * obj_qi;
	const GEN_FLT x6 = obj_qj * obj_qj;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = x6 + x7;
	const GEN_FLT x9 = 2 * sqrt(obj_qw * obj_qw + x5 + x8);
	const GEN_FLT x10 = sensor_x * x9;
	const GEN_FLT x11 = obj_qj * obj_qk;
	const GEN_FLT x12 = obj_qi * obj_qw;
	const GEN_FLT x13 = sensor_z * x9;
	const GEN_FLT x14 = obj_py + sensor_y * (-x9 * (x5 + x7) + 1) + x10 * (x3 + x4) + x13 * (x11 - x12);
	const GEN_FLT x15 = lh_qi * lh_qi;
	const GEN_FLT x16 = lh_qj * lh_qj;
	const GEN_FLT x17 = lh_qk * lh_qk;
	const GEN_FLT x18 = x16 + x17;
	const GEN_FLT x19 = sqrt(lh_qw * lh_qw + x15 + x18);
	const GEN_FLT x20 = 2 * x19;
	const GEN_FLT x21 = x14 * x20;
	const GEN_FLT x22 = x2 * x21;
	const GEN_FLT x23 = lh_qi * lh_qk;
	const GEN_FLT x24 = lh_qj * lh_qw;
	const GEN_FLT x25 = x23 - x24;
	const GEN_FLT x26 = obj_qi * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qw;
	const GEN_FLT x28 = sensor_y * x9;
	const GEN_FLT x29 = obj_px + sensor_x * (-x8 * x9 + 1) + x13 * (x26 + x27) + x28 * (x3 - x4);
	const GEN_FLT x30 = x20 * x29;
	const GEN_FLT x31 = x25 * x30;
	const GEN_FLT x32 = x15 + x16;
	const GEN_FLT x33 = obj_pz + sensor_z * (-x9 * (x5 + x6) + 1) + x10 * (x26 - x27) + x28 * (x11 + x12);
	const GEN_FLT x34 = x33 * (-x20 * x32 + 1);
	const GEN_FLT x35 = x23 + x24;
	const GEN_FLT x36 = x20 * x33;
	const GEN_FLT x37 = x35 * x36;
	const GEN_FLT x38 = lh_qi * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qw;
	const GEN_FLT x40 = x38 - x39;
	const GEN_FLT x41 = x21 * x40;
	const GEN_FLT x42 = x29 * (-x18 * x20 + 1);
	const GEN_FLT x43 = lh_px + x37 + x41 + x42;
	const GEN_FLT x44 = -lh_pz - x22 - x31 - x34;
	const GEN_FLT x45 = x43 * x43 + x44 * x44;
	const GEN_FLT x46 = 1.0 / x45;
	const GEN_FLT x47 = x46 * (lh_pz + x22 + x31 + x34);
	const GEN_FLT x48 = x38 + x39;
	const GEN_FLT x49 = x30 * x48;
	const GEN_FLT x50 = -x0 + x1;
	const GEN_FLT x51 = x36 * x50;
	const GEN_FLT x52 = x15 + x17;
	const GEN_FLT x53 = x14 * (-x20 * x52 + 1);
	const GEN_FLT x54 = lh_py + x49 + x51 + x53;
	const GEN_FLT x55 = tilt_0 + 0.52359877559829882;
	const GEN_FLT x56 = tan(x55);
	const GEN_FLT x57 = x56 / sqrt(x45);
	const GEN_FLT x58 = x54 * x57;
	const GEN_FLT x59 = cos(x55);
	const GEN_FLT x60 = 1.0 / x59;
	const GEN_FLT x61 = x54 * x54;
	const GEN_FLT x62 = x45 + x61;
	const GEN_FLT x63 = x60 / sqrt(x62);
	const GEN_FLT x64 = asin(x54 * x63);
	const GEN_FLT x65 = 8.0108022e-6 * x64;
	const GEN_FLT x66 = -x65 - 8.0108022e-6;
	const GEN_FLT x67 = x64 * x66 + 0.0028679863;
	const GEN_FLT x68 = x64 * x67 + 5.3685255000000001e-6;
	const GEN_FLT x69 = x64 * x68 + 0.0076069798000000001;
	const GEN_FLT x70 = x64 * x64;
	const GEN_FLT x71 = atan2(x44, x43);
	const GEN_FLT x72 = ogeePhase_0 + x71 - asin(x58);
	const GEN_FLT x73 = ogeeMag_0 * sin(x72);
	const GEN_FLT x74 = curve_0 + x73;
	const GEN_FLT x75 = x64 * x69;
	const GEN_FLT x76 = -1.60216044e-5 * x64 - 8.0108022e-6;
	const GEN_FLT x77 = x64 * x76 + x67;
	const GEN_FLT x78 = x64 * x77 + x68;
	const GEN_FLT x79 = x64 * x78 + x69;
	const GEN_FLT x80 = sin(x55);
	const GEN_FLT x81 = x80 * (x64 * x79 + x75);
	const GEN_FLT x82 = x59 - x74 * x81;
	const GEN_FLT x83 = 1.0 / x82;
	const GEN_FLT x84 = x74 * x83;
	const GEN_FLT x85 = x70 * x84;
	const GEN_FLT x86 = x58 + x69 * x85;
	const GEN_FLT x87 = pow(1 - x86 * x86, -1.0 / 2.0);
	const GEN_FLT x88 = x54 * (-lh_px - x37 - x41 - x42);
	const GEN_FLT x89 = x56 / pow(x45, 3.0 / 2.0);
	const GEN_FLT x90 = x88 * x89;
	const GEN_FLT x91 = pow(-x61 / (x62 * (x59 * x59)) + 1, -1.0 / 2.0);
	const GEN_FLT x92 = x60 / pow(x62, 3.0 / 2.0);
	const GEN_FLT x93 = x88 * x92;
	const GEN_FLT x94 = x91 * x93;
	const GEN_FLT x95 = 2 * x75 * x84;
	const GEN_FLT x96 = pow(-x46 * x61 * x56 * x56 + 1, -1.0 / 2.0);
	const GEN_FLT x97 = x47 - x90 * x96;
	const GEN_FLT x98 = ogeeMag_0 * cos(x72);
	const GEN_FLT x99 = x69 * x70;
	const GEN_FLT x100 = x83 * x98 * x99;
	const GEN_FLT x101 = x66 * x91;
	const GEN_FLT x102 = x101 * x93;
	const GEN_FLT x103 = x64 * (x102 - x65 * x94) + x67 * x94;
	const GEN_FLT x104 = x103 * x64 + x68 * x94;
	const GEN_FLT x105 = x81 * x98;
	const GEN_FLT x106 = 2.40324066e-5 * x64;
	const GEN_FLT x107 = x80 * (-curve_0 - x73);
	const GEN_FLT x108 = x74 * x99 / ((x82 * x82));
	const GEN_FLT x109 =
		x47 - x87 * (x100 * x97 + x104 * x85 +
					 x108 * (x105 * x97 -
							 x107 * (x104 * x64 +
									 x64 * (x104 + x64 * (x103 + x64 * (x102 - x106 * x94 + x76 * x94) + x77 * x94) +
											x78 * x94) +
									 x69 * x94 + x79 * x94)) +
					 x90 + x94 * x95);
	const GEN_FLT x110 = gibMag_0 * cos(gibPhase_0 + x71 - asin(x86));
	const GEN_FLT x111 = x57 * x96;
	const GEN_FLT x112 = x54 * x92;
	const GEN_FLT x113 = x91 * (x112 * (-lh_py - x49 - x51 - x53) + x63);
	const GEN_FLT x114 = x113 * x66;
	const GEN_FLT x115 = x113 * x67 + x64 * (-x113 * x65 + x114);
	const GEN_FLT x116 = x113 * x68 + x115 * x64;
	const GEN_FLT x117 =
		x87 *
		(-x100 * x111 +
		 x108 * (-x105 * x111 - x107 * (x113 * x69 + x113 * x79 + x116 * x64 +
										x64 * (x113 * x78 + x116 +
											   x64 * (x113 * x77 + x115 + x64 * (-x106 * x113 + x113 * x76 + x114))))) +
		 x113 * x95 + x116 * x85 + x57);
	const GEN_FLT x118 = x43 * x46;
	const GEN_FLT x119 = -x118;
	const GEN_FLT x120 = x54 * x89;
	const GEN_FLT x121 = x120 * x44;
	const GEN_FLT x122 = x112 * x44;
	const GEN_FLT x123 = x122 * x91;
	const GEN_FLT x124 = x119 - x121 * x96;
	const GEN_FLT x125 = x101 * x122;
	const GEN_FLT x126 = x123 * x67 + x64 * (-x123 * x65 + x125);
	const GEN_FLT x127 = x123 * x68 + x126 * x64;
	const GEN_FLT x128 =
		x119 - x87 * (x100 * x124 +
					  x108 * (x105 * x124 -
							  x107 * (x123 * x69 + x123 * x79 + x127 * x64 +
									  x64 * (x123 * x78 + x127 +
											 x64 * (x123 * x77 + x126 + x64 * (-x106 * x123 + x123 * x76 + x125))))) +
					  x121 + x123 * x95 + x127 * x85);
	const GEN_FLT x129 = lh_qi * x36;
	const GEN_FLT x130 = lh_qk * x30;
	const GEN_FLT x131 = 1.0 / x19;
	const GEN_FLT x132 = 2 * x131;
	const GEN_FLT x133 = lh_qw * x132;
	const GEN_FLT x134 = x14 * x52;
	const GEN_FLT x135 = x29 * x48;
	const GEN_FLT x136 = x33 * x50;
	const GEN_FLT x137 = -x129 + x130 - x133 * x134 + x133 * x135 + x133 * x136;
	const GEN_FLT x138 = 4 * x19;
	const GEN_FLT x139 = lh_qi * x138;
	const GEN_FLT x140 = x139 * x33;
	const GEN_FLT x141 = lh_qk * x138;
	const GEN_FLT x142 = x141 * x29;
	const GEN_FLT x143 = 4 * x131;
	const GEN_FLT x144 = lh_qw * x143;
	const GEN_FLT x145 = (1.0 / 2.0) * x54;
	const GEN_FLT x146 = lh_qj * x138;
	const GEN_FLT x147 = x146 * x33;
	const GEN_FLT x148 = -x14 * x141;
	const GEN_FLT x149 = x18 * x29;
	const GEN_FLT x150 = x144 * x33;
	const GEN_FLT x151 = x14 * x144;
	const GEN_FLT x152 = (1.0 / 2.0) * x43;
	const GEN_FLT x153 = x139 * x14;
	const GEN_FLT x154 = x146 * x29;
	const GEN_FLT x155 = x25 * x29;
	const GEN_FLT x156 = (1.0 / 2.0) * x44;
	const GEN_FLT x157 = -x152 * (-x144 * x149 + x147 + x148 + x150 * x35 + x151 * x40) -
						 x156 * (-x144 * x155 + x150 * x32 - x151 * x2 - x153 + x154);
	const GEN_FLT x158 =
		x91 * (x112 * (-x145 * (-x134 * x144 + x135 * x144 + x136 * x144 - x140 + x142) + x157) + x137 * x63);
	const GEN_FLT x159 = x158 * x66;
	const GEN_FLT x160 = x158 * x67 + x64 * (-x158 * x65 + x159);
	const GEN_FLT x161 = x158 * x68 + x160 * x64;
	const GEN_FLT x162 = x120 * x157 + x137 * x57;
	const GEN_FLT x163 = lh_qi * x21;
	const GEN_FLT x164 = lh_qj * x30;
	const GEN_FLT x165 = x133 * x33;
	const GEN_FLT x166 = x133 * x14;
	const GEN_FLT x167 = lh_qj * x36;
	const GEN_FLT x168 = -lh_qk * x21;
	const GEN_FLT x169 = x118 * (-x133 * x155 - x163 + x164 + x165 * x32 - x166 * x2) +
						 x47 * (-x133 * x149 + x165 * x35 + x166 * x40 + x167 + x168);
	const GEN_FLT x170 = -x162 * x96 + x169;
	const GEN_FLT x171 =
		x169 - x87 * (x100 * x170 +
					  x108 * (x105 * x170 -
							  x107 * (x158 * x69 + x158 * x79 + x161 * x64 +
									  x64 * (x158 * x78 + x161 +
											 x64 * (x158 * x77 + x160 + x64 * (-x106 * x158 + x158 * x76 + x159))))) +
					  x158 * x95 + x161 * x85 + x162);
	const GEN_FLT x172 = lh_qw * x36;
	const GEN_FLT x173 = lh_qi * x132;
	const GEN_FLT x174 = x14 * (-x139 - x173 * x52);
	const GEN_FLT x175 = x135 * x173 + x136 * x173 + x164 - x172 + x174;
	const GEN_FLT x176 = lh_qw * x138;
	const GEN_FLT x177 = x176 * x33;
	const GEN_FLT x178 = lh_qi * x143;
	const GEN_FLT x179 = x14 * x146;
	const GEN_FLT x180 = x141 * x33;
	const GEN_FLT x181 = x33 * x35;
	const GEN_FLT x182 = x14 * x178;
	const GEN_FLT x183 = -x14 * x176;
	const GEN_FLT x184 = x33 * (x139 + x173 * x32);
	const GEN_FLT x185 = -x152 * (-x149 * x178 + x178 * x181 + x179 + x180 + x182 * x40) -
						 x156 * (-x142 - x155 * x178 - x182 * x2 + x183 + 2 * x184);
	const GEN_FLT x186 =
		x91 * (x112 * (-x145 * (x135 * x178 + x136 * x178 + x154 + 2 * x174 - x177) + x185) + x175 * x63);
	const GEN_FLT x187 = x186 * x66;
	const GEN_FLT x188 = x186 * x67 + x64 * (-x186 * x65 + x187);
	const GEN_FLT x189 = x186 * x68 + x188 * x64;
	const GEN_FLT x190 = x120 * x185 + x175 * x57;
	const GEN_FLT x191 = -lh_qw * x21;
	const GEN_FLT x192 = x14 * x173;
	const GEN_FLT x193 = lh_qj * x21;
	const GEN_FLT x194 = lh_qk * x36;
	const GEN_FLT x195 = x118 * (-x130 - x155 * x173 + x184 + x191 - x192 * x2) +
						 x47 * (-x149 * x173 + x173 * x181 + x192 * x40 + x193 + x194);
	const GEN_FLT x196 = -x190 * x96 + x195;
	const GEN_FLT x197 =
		x195 - x87 * (x100 * x196 +
					  x108 * (x105 * x196 -
							  x107 * (x186 * x69 + x186 * x79 + x189 * x64 +
									  x64 * (x186 * x78 + x189 +
											 x64 * (x186 * x77 + x188 + x64 * (-x106 * x186 + x186 * x76 + x187))))) +
					  x186 * x95 + x189 * x85 + x190);
	const GEN_FLT x198 = lh_qi * x30;
	const GEN_FLT x199 = lh_qj * x132;
	const GEN_FLT x200 = -x134 * x199 + x135 * x199 + x136 * x199 + x194 + x198;
	const GEN_FLT x201 = x139 * x29;
	const GEN_FLT x202 = lh_qj * x143;
	const GEN_FLT x203 = x14 * x40;
	const GEN_FLT x204 = x29 * (-x146 - x18 * x199);
	const GEN_FLT x205 = x176 * x29;
	const GEN_FLT x206 = x14 * x2;
	const GEN_FLT x207 = x33 * (x146 + x199 * x32);
	const GEN_FLT x208 = -x152 * (x153 + x177 + x181 * x202 + x202 * x203 + 2 * x204) -
						 x156 * (x148 - x155 * x202 - x202 * x206 + x205 + 2 * x207);
	const GEN_FLT x209 =
		x91 * (x112 * (-x145 * (-x134 * x202 + x135 * x202 + x136 * x202 + x180 + x201) + x208) + x200 * x63);
	const GEN_FLT x210 = x209 * x66;
	const GEN_FLT x211 = x209 * x67 + x64 * (-x209 * x65 + x210);
	const GEN_FLT x212 = x209 * x68 + x211 * x64;
	const GEN_FLT x213 = x120 * x208 + x200 * x57;
	const GEN_FLT x214 = lh_qw * x30;
	const GEN_FLT x215 = x118 * (-x155 * x199 + x168 - x199 * x206 + x207 + x214) +
						 x47 * (x163 + x172 + x181 * x199 + x199 * x203 + x204);
	const GEN_FLT x216 = -x213 * x96 + x215;
	const GEN_FLT x217 =
		x215 - x87 * (x100 * x216 +
					  x108 * (x105 * x216 -
							  x107 * (x209 * x69 + x209 * x79 + x212 * x64 +
									  x64 * (x209 * x78 + x212 +
											 x64 * (x209 * x77 + x211 + x64 * (-x106 * x209 + x209 * x76 + x210))))) +
					  x209 * x95 + x212 * x85 + x213);
	const GEN_FLT x218 = lh_qk * x132;
	const GEN_FLT x219 = -x141;
	const GEN_FLT x220 = x14 * (-x218 * x52 + x219);
	const GEN_FLT x221 = x135 * x218 + x136 * x218 + x167 + x214 + x220;
	const GEN_FLT x222 = lh_qk * x143;
	const GEN_FLT x223 = x32 * x33;
	const GEN_FLT x224 = x29 * (-x18 * x218 + x219);
	const GEN_FLT x225 = -x152 * (x140 + x181 * x222 + x183 + x203 * x222 + 2 * x224) -
						 x156 * (-x155 * x222 - x179 - x201 - x206 * x222 + x222 * x223);
	const GEN_FLT x226 =
		x91 * (x112 * (-x145 * (x135 * x222 + x136 * x222 + x147 + x205 + 2 * x220) + x225) + x221 * x63);
	const GEN_FLT x227 = x226 * x66;
	const GEN_FLT x228 = x226 * x67 + x64 * (-x226 * x65 + x227);
	const GEN_FLT x229 = x226 * x68 + x228 * x64;
	const GEN_FLT x230 = x120 * x225 + x221 * x57;
	const GEN_FLT x231 = x118 * (-x155 * x218 - x193 - x198 - x206 * x218 + x218 * x223) +
						 x47 * (x129 + x181 * x218 + x191 + x203 * x218 + x224);
	const GEN_FLT x232 = -x230 * x96 + x231;
	const GEN_FLT x233 =
		x231 - x87 * (x100 * x232 +
					  x108 * (x105 * x232 -
							  x107 * (x226 * x69 + x226 * x79 + x229 * x64 +
									  x64 * (x226 * x78 + x229 +
											 x64 * (x226 * x77 + x228 + x64 * (-x106 * x226 + x226 * x76 + x227))))) +
					  x226 * x95 + x229 * x85 + x230);
	*(out++) = x109 * x110 + x109;
	*(out++) = -x110 * x117 - x117;
	*(out++) = x110 * x128 + x128;
	*(out++) = x110 * x171 + x171;
	*(out++) = x110 * x197 + x197;
	*(out++) = x110 * x217 + x217;
	*(out++) = x110 * x233 + x233;
}

static inline void gen_reproject_axis_y_jac_lh_p_gen2(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
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
	const GEN_FLT x0 = lh_qi * lh_qw;
	const GEN_FLT x1 = lh_qj * lh_qk;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = obj_qi * obj_qj;
	const GEN_FLT x4 = obj_qk * obj_qw;
	const GEN_FLT x5 = obj_qi * obj_qi;
	const GEN_FLT x6 = obj_qj * obj_qj;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = x6 + x7;
	const GEN_FLT x9 = 2 * sqrt(obj_qw * obj_qw + x5 + x8);
	const GEN_FLT x10 = sensor_x * x9;
	const GEN_FLT x11 = obj_qj * obj_qk;
	const GEN_FLT x12 = obj_qi * obj_qw;
	const GEN_FLT x13 = sensor_z * x9;
	const GEN_FLT x14 = obj_py + sensor_y * (-x9 * (x5 + x7) + 1) + x10 * (x3 + x4) + x13 * (x11 - x12);
	const GEN_FLT x15 = lh_qi * lh_qi;
	const GEN_FLT x16 = lh_qj * lh_qj;
	const GEN_FLT x17 = lh_qk * lh_qk;
	const GEN_FLT x18 = x16 + x17;
	const GEN_FLT x19 = sqrt(lh_qw * lh_qw + x15 + x18);
	const GEN_FLT x20 = 2 * x19;
	const GEN_FLT x21 = x14 * x20;
	const GEN_FLT x22 = x2 * x21;
	const GEN_FLT x23 = lh_qi * lh_qk;
	const GEN_FLT x24 = lh_qj * lh_qw;
	const GEN_FLT x25 = x23 - x24;
	const GEN_FLT x26 = obj_qi * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qw;
	const GEN_FLT x28 = sensor_y * x9;
	const GEN_FLT x29 = obj_px + sensor_x * (-x8 * x9 + 1) + x13 * (x26 + x27) + x28 * (x3 - x4);
	const GEN_FLT x30 = x20 * x29;
	const GEN_FLT x31 = x25 * x30;
	const GEN_FLT x32 = x15 + x16;
	const GEN_FLT x33 = obj_pz + sensor_z * (-x9 * (x5 + x6) + 1) + x10 * (x26 - x27) + x28 * (x11 + x12);
	const GEN_FLT x34 = x33 * (-x20 * x32 + 1);
	const GEN_FLT x35 = x23 + x24;
	const GEN_FLT x36 = x20 * x33;
	const GEN_FLT x37 = x35 * x36;
	const GEN_FLT x38 = lh_qi * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qw;
	const GEN_FLT x40 = x38 - x39;
	const GEN_FLT x41 = x21 * x40;
	const GEN_FLT x42 = x29 * (-x18 * x20 + 1);
	const GEN_FLT x43 = lh_px + x37 + x41 + x42;
	const GEN_FLT x44 = -lh_pz - x22 - x31 - x34;
	const GEN_FLT x45 = x43 * x43 + x44 * x44;
	const GEN_FLT x46 = 1.0 / x45;
	const GEN_FLT x47 = x46 * (lh_pz + x22 + x31 + x34);
	const GEN_FLT x48 = x38 + x39;
	const GEN_FLT x49 = x30 * x48;
	const GEN_FLT x50 = -x0 + x1;
	const GEN_FLT x51 = x36 * x50;
	const GEN_FLT x52 = x15 + x17;
	const GEN_FLT x53 = x14 * (-x20 * x52 + 1);
	const GEN_FLT x54 = lh_py + x49 + x51 + x53;
	const GEN_FLT x55 = tilt_0 - 0.52359877559829882;
	const GEN_FLT x56 = tan(x55);
	const GEN_FLT x57 = x56 / sqrt(x45);
	const GEN_FLT x58 = x54 * x57;
	const GEN_FLT x59 = cos(x55);
	const GEN_FLT x60 = 1.0 / x59;
	const GEN_FLT x61 = x54 * x54;
	const GEN_FLT x62 = x45 + x61;
	const GEN_FLT x63 = x60 / sqrt(x62);
	const GEN_FLT x64 = asin(x54 * x63);
	const GEN_FLT x65 = 8.0108022e-6 * x64;
	const GEN_FLT x66 = -x65 - 8.0108022e-6;
	const GEN_FLT x67 = x64 * x66 + 0.0028679863;
	const GEN_FLT x68 = x64 * x67 + 5.3685255000000001e-6;
	const GEN_FLT x69 = x64 * x68 + 0.0076069798000000001;
	const GEN_FLT x70 = x64 * x64;
	const GEN_FLT x71 = atan2(x44, x43);
	const GEN_FLT x72 = ogeePhase_0 + x71 - asin(x58);
	const GEN_FLT x73 = ogeeMag_0 * sin(x72);
	const GEN_FLT x74 = curve_0 + x73;
	const GEN_FLT x75 = x64 * x69;
	const GEN_FLT x76 = -1.60216044e-5 * x64 - 8.0108022e-6;
	const GEN_FLT x77 = x64 * x76 + x67;
	const GEN_FLT x78 = x64 * x77 + x68;
	const GEN_FLT x79 = x64 * x78 + x69;
	const GEN_FLT x80 = sin(x55);
	const GEN_FLT x81 = x80 * (x64 * x79 + x75);
	const GEN_FLT x82 = x59 - x74 * x81;
	const GEN_FLT x83 = 1.0 / x82;
	const GEN_FLT x84 = x74 * x83;
	const GEN_FLT x85 = x70 * x84;
	const GEN_FLT x86 = x58 + x69 * x85;
	const GEN_FLT x87 = pow(1 - x86 * x86, -1.0 / 2.0);
	const GEN_FLT x88 = x54 * (-lh_px - x37 - x41 - x42);
	const GEN_FLT x89 = x56 / pow(x45, 3.0 / 2.0);
	const GEN_FLT x90 = x88 * x89;
	const GEN_FLT x91 = pow(-x61 / (x62 * (x59 * x59)) + 1, -1.0 / 2.0);
	const GEN_FLT x92 = x60 / pow(x62, 3.0 / 2.0);
	const GEN_FLT x93 = x88 * x92;
	const GEN_FLT x94 = x91 * x93;
	const GEN_FLT x95 = 2 * x75 * x84;
	const GEN_FLT x96 = pow(-x46 * x61 * x56 * x56 + 1, -1.0 / 2.0);
	const GEN_FLT x97 = x47 - x90 * x96;
	const GEN_FLT x98 = ogeeMag_0 * cos(x72);
	const GEN_FLT x99 = x69 * x70;
	const GEN_FLT x100 = x83 * x98 * x99;
	const GEN_FLT x101 = x66 * x91;
	const GEN_FLT x102 = x101 * x93;
	const GEN_FLT x103 = x64 * (x102 - x65 * x94) + x67 * x94;
	const GEN_FLT x104 = x103 * x64 + x68 * x94;
	const GEN_FLT x105 = x81 * x98;
	const GEN_FLT x106 = 2.40324066e-5 * x64;
	const GEN_FLT x107 = x80 * (-curve_0 - x73);
	const GEN_FLT x108 = x74 * x99 / ((x82 * x82));
	const GEN_FLT x109 =
		x47 - x87 * (x100 * x97 + x104 * x85 +
					 x108 * (x105 * x97 -
							 x107 * (x104 * x64 +
									 x64 * (x104 + x64 * (x103 + x64 * (x102 - x106 * x94 + x76 * x94) + x77 * x94) +
											x78 * x94) +
									 x69 * x94 + x79 * x94)) +
					 x90 + x94 * x95);
	const GEN_FLT x110 = gibMag_0 * cos(gibPhase_0 + x71 - asin(x86));
	const GEN_FLT x111 = x57 * x96;
	const GEN_FLT x112 = x54 * x92;
	const GEN_FLT x113 = x91 * (x112 * (-lh_py - x49 - x51 - x53) + x63);
	const GEN_FLT x114 = x113 * x66;
	const GEN_FLT x115 = x113 * x67 + x64 * (-x113 * x65 + x114);
	const GEN_FLT x116 = x113 * x68 + x115 * x64;
	const GEN_FLT x117 =
		x87 *
		(-x100 * x111 +
		 x108 * (-x105 * x111 - x107 * (x113 * x69 + x113 * x79 + x116 * x64 +
										x64 * (x113 * x78 + x116 +
											   x64 * (x113 * x77 + x115 + x64 * (-x106 * x113 + x113 * x76 + x114))))) +
		 x113 * x95 + x116 * x85 + x57);
	const GEN_FLT x118 = x43 * x46;
	const GEN_FLT x119 = -x118;
	const GEN_FLT x120 = x54 * x89;
	const GEN_FLT x121 = x120 * x44;
	const GEN_FLT x122 = x112 * x44;
	const GEN_FLT x123 = x122 * x91;
	const GEN_FLT x124 = x119 - x121 * x96;
	const GEN_FLT x125 = x101 * x122;
	const GEN_FLT x126 = x123 * x67 + x64 * (-x123 * x65 + x125);
	const GEN_FLT x127 = x123 * x68 + x126 * x64;
	const GEN_FLT x128 =
		x119 - x87 * (x100 * x124 +
					  x108 * (x105 * x124 -
							  x107 * (x123 * x69 + x123 * x79 + x127 * x64 +
									  x64 * (x123 * x78 + x127 +
											 x64 * (x123 * x77 + x126 + x64 * (-x106 * x123 + x123 * x76 + x125))))) +
					  x121 + x123 * x95 + x127 * x85);
	const GEN_FLT x129 = lh_qi * x36;
	const GEN_FLT x130 = lh_qk * x30;
	const GEN_FLT x131 = 1.0 / x19;
	const GEN_FLT x132 = 2 * x131;
	const GEN_FLT x133 = lh_qw * x132;
	const GEN_FLT x134 = x14 * x52;
	const GEN_FLT x135 = x29 * x48;
	const GEN_FLT x136 = x33 * x50;
	const GEN_FLT x137 = -x129 + x130 - x133 * x134 + x133 * x135 + x133 * x136;
	const GEN_FLT x138 = 4 * x19;
	const GEN_FLT x139 = lh_qi * x138;
	const GEN_FLT x140 = x139 * x33;
	const GEN_FLT x141 = lh_qk * x138;
	const GEN_FLT x142 = x141 * x29;
	const GEN_FLT x143 = 4 * x131;
	const GEN_FLT x144 = lh_qw * x143;
	const GEN_FLT x145 = (1.0 / 2.0) * x54;
	const GEN_FLT x146 = lh_qj * x138;
	const GEN_FLT x147 = x146 * x33;
	const GEN_FLT x148 = -x14 * x141;
	const GEN_FLT x149 = x18 * x29;
	const GEN_FLT x150 = x144 * x33;
	const GEN_FLT x151 = x14 * x144;
	const GEN_FLT x152 = (1.0 / 2.0) * x43;
	const GEN_FLT x153 = x139 * x14;
	const GEN_FLT x154 = x146 * x29;
	const GEN_FLT x155 = x25 * x29;
	const GEN_FLT x156 = (1.0 / 2.0) * x44;
	const GEN_FLT x157 = -x152 * (-x144 * x149 + x147 + x148 + x150 * x35 + x151 * x40) -
						 x156 * (-x144 * x155 + x150 * x32 - x151 * x2 - x153 + x154);
	const GEN_FLT x158 =
		x91 * (x112 * (-x145 * (-x134 * x144 + x135 * x144 + x136 * x144 - x140 + x142) + x157) + x137 * x63);
	const GEN_FLT x159 = x158 * x66;
	const GEN_FLT x160 = x158 * x67 + x64 * (-x158 * x65 + x159);
	const GEN_FLT x161 = x158 * x68 + x160 * x64;
	const GEN_FLT x162 = x120 * x157 + x137 * x57;
	const GEN_FLT x163 = lh_qi * x21;
	const GEN_FLT x164 = lh_qj * x30;
	const GEN_FLT x165 = x133 * x33;
	const GEN_FLT x166 = x133 * x14;
	const GEN_FLT x167 = lh_qj * x36;
	const GEN_FLT x168 = -lh_qk * x21;
	const GEN_FLT x169 = x118 * (-x133 * x155 - x163 + x164 + x165 * x32 - x166 * x2) +
						 x47 * (-x133 * x149 + x165 * x35 + x166 * x40 + x167 + x168);
	const GEN_FLT x170 = -x162 * x96 + x169;
	const GEN_FLT x171 =
		x169 - x87 * (x100 * x170 +
					  x108 * (x105 * x170 -
							  x107 * (x158 * x69 + x158 * x79 + x161 * x64 +
									  x64 * (x158 * x78 + x161 +
											 x64 * (x158 * x77 + x160 + x64 * (-x106 * x158 + x158 * x76 + x159))))) +
					  x158 * x95 + x161 * x85 + x162);
	const GEN_FLT x172 = lh_qw * x36;
	const GEN_FLT x173 = lh_qi * x132;
	const GEN_FLT x174 = x14 * (-x139 - x173 * x52);
	const GEN_FLT x175 = x135 * x173 + x136 * x173 + x164 - x172 + x174;
	const GEN_FLT x176 = lh_qw * x138;
	const GEN_FLT x177 = x176 * x33;
	const GEN_FLT x178 = lh_qi * x143;
	const GEN_FLT x179 = x14 * x146;
	const GEN_FLT x180 = x141 * x33;
	const GEN_FLT x181 = x33 * x35;
	const GEN_FLT x182 = x14 * x178;
	const GEN_FLT x183 = -x14 * x176;
	const GEN_FLT x184 = x33 * (x139 + x173 * x32);
	const GEN_FLT x185 = -x152 * (-x149 * x178 + x178 * x181 + x179 + x180 + x182 * x40) -
						 x156 * (-x142 - x155 * x178 - x182 * x2 + x183 + 2 * x184);
	const GEN_FLT x186 =
		x91 * (x112 * (-x145 * (x135 * x178 + x136 * x178 + x154 + 2 * x174 - x177) + x185) + x175 * x63);
	const GEN_FLT x187 = x186 * x66;
	const GEN_FLT x188 = x186 * x67 + x64 * (-x186 * x65 + x187);
	const GEN_FLT x189 = x186 * x68 + x188 * x64;
	const GEN_FLT x190 = x120 * x185 + x175 * x57;
	const GEN_FLT x191 = -lh_qw * x21;
	const GEN_FLT x192 = x14 * x173;
	const GEN_FLT x193 = lh_qj * x21;
	const GEN_FLT x194 = lh_qk * x36;
	const GEN_FLT x195 = x118 * (-x130 - x155 * x173 + x184 + x191 - x192 * x2) +
						 x47 * (-x149 * x173 + x173 * x181 + x192 * x40 + x193 + x194);
	const GEN_FLT x196 = -x190 * x96 + x195;
	const GEN_FLT x197 =
		x195 - x87 * (x100 * x196 +
					  x108 * (x105 * x196 -
							  x107 * (x186 * x69 + x186 * x79 + x189 * x64 +
									  x64 * (x186 * x78 + x189 +
											 x64 * (x186 * x77 + x188 + x64 * (-x106 * x186 + x186 * x76 + x187))))) +
					  x186 * x95 + x189 * x85 + x190);
	const GEN_FLT x198 = lh_qi * x30;
	const GEN_FLT x199 = lh_qj * x132;
	const GEN_FLT x200 = -x134 * x199 + x135 * x199 + x136 * x199 + x194 + x198;
	const GEN_FLT x201 = x139 * x29;
	const GEN_FLT x202 = lh_qj * x143;
	const GEN_FLT x203 = x14 * x40;
	const GEN_FLT x204 = x29 * (-x146 - x18 * x199);
	const GEN_FLT x205 = x176 * x29;
	const GEN_FLT x206 = x14 * x2;
	const GEN_FLT x207 = x33 * (x146 + x199 * x32);
	const GEN_FLT x208 = -x152 * (x153 + x177 + x181 * x202 + x202 * x203 + 2 * x204) -
						 x156 * (x148 - x155 * x202 - x202 * x206 + x205 + 2 * x207);
	const GEN_FLT x209 =
		x91 * (x112 * (-x145 * (-x134 * x202 + x135 * x202 + x136 * x202 + x180 + x201) + x208) + x200 * x63);
	const GEN_FLT x210 = x209 * x66;
	const GEN_FLT x211 = x209 * x67 + x64 * (-x209 * x65 + x210);
	const GEN_FLT x212 = x209 * x68 + x211 * x64;
	const GEN_FLT x213 = x120 * x208 + x200 * x57;
	const GEN_FLT x214 = lh_qw * x30;
	const GEN_FLT x215 = x118 * (-x155 * x199 + x168 - x199 * x206 + x207 + x214) +
						 x47 * (x163 + x172 + x181 * x199 + x199 * x203 + x204);
	const GEN_FLT x216 = -x213 * x96 + x215;
	const GEN_FLT x217 =
		x215 - x87 * (x100 * x216 +
					  x108 * (x105 * x216 -
							  x107 * (x209 * x69 + x209 * x79 + x212 * x64 +
									  x64 * (x209 * x78 + x212 +
											 x64 * (x209 * x77 + x211 + x64 * (-x106 * x209 + x209 * x76 + x210))))) +
					  x209 * x95 + x212 * x85 + x213);
	const GEN_FLT x218 = lh_qk * x132;
	const GEN_FLT x219 = -x141;
	const GEN_FLT x220 = x14 * (-x218 * x52 + x219);
	const GEN_FLT x221 = x135 * x218 + x136 * x218 + x167 + x214 + x220;
	const GEN_FLT x222 = lh_qk * x143;
	const GEN_FLT x223 = x32 * x33;
	const GEN_FLT x224 = x29 * (-x18 * x218 + x219);
	const GEN_FLT x225 = -x152 * (x140 + x181 * x222 + x183 + x203 * x222 + 2 * x224) -
						 x156 * (-x155 * x222 - x179 - x201 - x206 * x222 + x222 * x223);
	const GEN_FLT x226 =
		x91 * (x112 * (-x145 * (x135 * x222 + x136 * x222 + x147 + x205 + 2 * x220) + x225) + x221 * x63);
	const GEN_FLT x227 = x226 * x66;
	const GEN_FLT x228 = x226 * x67 + x64 * (-x226 * x65 + x227);
	const GEN_FLT x229 = x226 * x68 + x228 * x64;
	const GEN_FLT x230 = x120 * x225 + x221 * x57;
	const GEN_FLT x231 = x118 * (-x155 * x218 - x193 - x198 - x206 * x218 + x218 * x223) +
						 x47 * (x129 + x181 * x218 + x191 + x203 * x218 + x224);
	const GEN_FLT x232 = -x230 * x96 + x231;
	const GEN_FLT x233 =
		x231 - x87 * (x100 * x232 +
					  x108 * (x105 * x232 -
							  x107 * (x226 * x69 + x226 * x79 + x229 * x64 +
									  x64 * (x226 * x78 + x229 +
											 x64 * (x226 * x77 + x228 + x64 * (-x106 * x226 + x226 * x76 + x227))))) +
					  x226 * x95 + x229 * x85 + x230);
	*(out++) = x109 * x110 + x109;
	*(out++) = -x110 * x117 - x117;
	*(out++) = x110 * x128 + x128;
	*(out++) = x110 * x171 + x171;
	*(out++) = x110 * x197 + x197;
	*(out++) = x110 * x217 + x217;
	*(out++) = x110 * x233 + x233;
}

static inline void gen_reproject_jac_lh_p(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh, const FLT phase_0,
										  const FLT phase_1, const FLT tilt_0, const FLT tilt_1, const FLT curve_0,
										  const FLT curve_1, const FLT gibPhase_0, const FLT gibPhase_1,
										  const FLT gibMag_0, const FLT gibMag_1) {
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
	const GEN_FLT x0 = lh_qi * lh_qw;
	const GEN_FLT x1 = lh_qj * lh_qk;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = obj_qi * obj_qj;
	const GEN_FLT x4 = obj_qk * obj_qw;
	const GEN_FLT x5 = obj_qi * obj_qi;
	const GEN_FLT x6 = obj_qj * obj_qj;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = x6 + x7;
	const GEN_FLT x9 = 2 * sqrt(obj_qw * obj_qw + x5 + x8);
	const GEN_FLT x10 = sensor_x * x9;
	const GEN_FLT x11 = obj_qj * obj_qk;
	const GEN_FLT x12 = obj_qi * obj_qw;
	const GEN_FLT x13 = sensor_z * x9;
	const GEN_FLT x14 = obj_py + sensor_y * (-x9 * (x5 + x7) + 1) + x10 * (x3 + x4) + x13 * (x11 - x12);
	const GEN_FLT x15 = lh_qi * lh_qi;
	const GEN_FLT x16 = lh_qj * lh_qj;
	const GEN_FLT x17 = lh_qk * lh_qk;
	const GEN_FLT x18 = x16 + x17;
	const GEN_FLT x19 = sqrt(lh_qw * lh_qw + x15 + x18);
	const GEN_FLT x20 = 2 * x19;
	const GEN_FLT x21 = x14 * x20;
	const GEN_FLT x22 = lh_qi * lh_qk;
	const GEN_FLT x23 = lh_qj * lh_qw;
	const GEN_FLT x24 = x22 - x23;
	const GEN_FLT x25 = obj_qi * obj_qk;
	const GEN_FLT x26 = obj_qj * obj_qw;
	const GEN_FLT x27 = sensor_y * x9;
	const GEN_FLT x28 = obj_px + sensor_x * (-x8 * x9 + 1) + x13 * (x25 + x26) + x27 * (x3 - x4);
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = x15 + x16;
	const GEN_FLT x31 = obj_pz + sensor_z * (-x9 * (x5 + x6) + 1) + x10 * (x25 - x26) + x27 * (x11 + x12);
	const GEN_FLT x32 = -lh_pz - x2 * x21 - x24 * x29 - x31 * (-x20 * x30 + 1);
	const GEN_FLT x33 = x22 + x23;
	const GEN_FLT x34 = x20 * x31;
	const GEN_FLT x35 = x33 * x34;
	const GEN_FLT x36 = lh_qi * lh_qj;
	const GEN_FLT x37 = lh_qk * lh_qw;
	const GEN_FLT x38 = x36 - x37;
	const GEN_FLT x39 = x21 * x38;
	const GEN_FLT x40 = x28 * (-x18 * x20 + 1);
	const GEN_FLT x41 = lh_px + x35 + x39 + x40;
	const GEN_FLT x42 = x41 * x41;
	const GEN_FLT x43 = x32 * x32;
	const GEN_FLT x44 = x42 + x43;
	const GEN_FLT x45 = 1.0 / x44;
	const GEN_FLT x46 = x32 * x45;
	const GEN_FLT x47 = -lh_px - x35 - x39 - x40;
	const GEN_FLT x48 = x36 + x37;
	const GEN_FLT x49 = x29 * x48;
	const GEN_FLT x50 = -x0 + x1;
	const GEN_FLT x51 = x34 * x50;
	const GEN_FLT x52 = x15 + x17;
	const GEN_FLT x53 = x14 * (-x20 * x52 + 1);
	const GEN_FLT x54 = lh_py + x49 + x51 + x53;
	const GEN_FLT x55 = x54 * x54;
	const GEN_FLT x56 = pow(-x45 * x55 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x57 = tilt_0 * x54 / pow(x44, 3.0 / 2.0);
	const GEN_FLT x58 = x56 * x57;
	const GEN_FLT x59 = x47 * x58;
	const GEN_FLT x60 = atan2(x41, x32);
	const GEN_FLT x61 = tilt_0 / sqrt(x44);
	const GEN_FLT x62 = gibMag_0 * sin(-gibPhase_0 + phase_0 + x60 + asin(x54 * x61) - 1.5707963267948966);
	const GEN_FLT x63 = x43 + x55;
	const GEN_FLT x64 = 1.0 / x63;
	const GEN_FLT x65 = x32 * x64;
	const GEN_FLT x66 = atan2(x54, x32);
	const GEN_FLT x67 = curve_0 * x66;
	const GEN_FLT x68 = 2 * x67;
	const GEN_FLT x69 = x56 * x61;
	const GEN_FLT x70 = x45 * x47;
	const GEN_FLT x71 = -lh_py - x49 - x51 - x53;
	const GEN_FLT x72 = x64 * x71;
	const GEN_FLT x73 = x32 * x58;
	const GEN_FLT x74 = lh_qi * x21;
	const GEN_FLT x75 = lh_qj * x29;
	const GEN_FLT x76 = 1.0 / x19;
	const GEN_FLT x77 = 2 * x76;
	const GEN_FLT x78 = lh_qw * x77;
	const GEN_FLT x79 = x31 * x78;
	const GEN_FLT x80 = x14 * x78;
	const GEN_FLT x81 = x28 * x78;
	const GEN_FLT x82 = -x2 * x80 - x24 * x81 + x30 * x79 - x74 + x75;
	const GEN_FLT x83 = x70 * x82;
	const GEN_FLT x84 = lh_qj * x34;
	const GEN_FLT x85 = -lh_qk * x21;
	const GEN_FLT x86 = -x18 * x81 + x33 * x79 + x38 * x80 + x84 + x85;
	const GEN_FLT x87 = x46 * x86;
	const GEN_FLT x88 = x72 * x82;
	const GEN_FLT x89 = lh_qi * x34;
	const GEN_FLT x90 = lh_qk * x29;
	const GEN_FLT x91 = x14 * x52;
	const GEN_FLT x92 = x48 * x81 + x50 * x79 - x78 * x91 - x89 + x90;
	const GEN_FLT x93 = x65 * x92;
	const GEN_FLT x94 = 4 * x19;
	const GEN_FLT x95 = lh_qj * x94;
	const GEN_FLT x96 = x31 * x95;
	const GEN_FLT x97 = lh_qk * x94;
	const GEN_FLT x98 = -x14 * x97;
	const GEN_FLT x99 = x18 * x28;
	const GEN_FLT x100 = 4 * x76;
	const GEN_FLT x101 = lh_qw * x100;
	const GEN_FLT x102 = x101 * x31;
	const GEN_FLT x103 = x101 * x14;
	const GEN_FLT x104 = (1.0 / 2.0) * x41;
	const GEN_FLT x105 = lh_qi * x94;
	const GEN_FLT x106 = x105 * x14;
	const GEN_FLT x107 = x28 * x95;
	const GEN_FLT x108 = x24 * x28;
	const GEN_FLT x109 = (1.0 / 2.0) * x32;
	const GEN_FLT x110 = -x109 * (-x101 * x108 + x102 * x30 - x103 * x2 - x106 + x107);
	const GEN_FLT x111 = x56 * (x57 * (-x104 * (-x101 * x99 + x102 * x33 + x103 * x38 + x96 + x98) + x110) + x61 * x92);
	const GEN_FLT x112 = lh_qj * x21;
	const GEN_FLT x113 = lh_qk * x34;
	const GEN_FLT x114 = lh_qi * x77;
	const GEN_FLT x115 = x114 * x31;
	const GEN_FLT x116 = x114 * x14;
	const GEN_FLT x117 = x112 + x113 - x114 * x99 + x115 * x33 + x116 * x38;
	const GEN_FLT x118 = x117 * x46;
	const GEN_FLT x119 = -lh_qw * x21;
	const GEN_FLT x120 = x31 * (x105 + x114 * x30);
	const GEN_FLT x121 = -x108 * x114 - x116 * x2 + x119 + x120 - x90;
	const GEN_FLT x122 = x121 * x70;
	const GEN_FLT x123 = x121 * x72;
	const GEN_FLT x124 = lh_qw * x34;
	const GEN_FLT x125 = x28 * x48;
	const GEN_FLT x126 = x14 * (-x105 - x114 * x52);
	const GEN_FLT x127 = x114 * x125 + x115 * x50 - x124 + x126 + x75;
	const GEN_FLT x128 = x127 * x65;
	const GEN_FLT x129 = x14 * x95;
	const GEN_FLT x130 = x31 * x97;
	const GEN_FLT x131 = lh_qi * x100;
	const GEN_FLT x132 = x31 * x33;
	const GEN_FLT x133 = x131 * x14;
	const GEN_FLT x134 = x28 * x97;
	const GEN_FLT x135 = lh_qw * x94;
	const GEN_FLT x136 = -x135 * x14;
	const GEN_FLT x137 = -x109 * (-x108 * x131 + 2 * x120 - x133 * x2 - x134 + x136);
	const GEN_FLT x138 =
		x56 * (x127 * x61 + x57 * (-x104 * (x129 + x130 + x131 * x132 - x131 * x99 + x133 * x38) + x137));
	const GEN_FLT x139 = lh_qw * x29;
	const GEN_FLT x140 = lh_qj * x77;
	const GEN_FLT x141 = x14 * x2;
	const GEN_FLT x142 = x31 * (x140 * x30 + x95);
	const GEN_FLT x143 = -x108 * x140 + x139 - x140 * x141 + x142 + x85;
	const GEN_FLT x144 = x143 * x70;
	const GEN_FLT x145 = x14 * x38;
	const GEN_FLT x146 = x28 * (-x140 * x18 - x95);
	const GEN_FLT x147 = x124 + x132 * x140 + x140 * x145 + x146 + x74;
	const GEN_FLT x148 = x147 * x46;
	const GEN_FLT x149 = lh_qi * x29;
	const GEN_FLT x150 = x31 * x50;
	const GEN_FLT x151 = x113 + x125 * x140 + x140 * x150 - x140 * x91 + x149;
	const GEN_FLT x152 = x151 * x65;
	const GEN_FLT x153 = x143 * x72;
	const GEN_FLT x154 = x135 * x31;
	const GEN_FLT x155 = lh_qj * x100;
	const GEN_FLT x156 = x135 * x28;
	const GEN_FLT x157 = -x109 * (-x108 * x155 - x141 * x155 + 2 * x142 + x156 + x98);
	const GEN_FLT x158 =
		x56 * (x151 * x61 + x57 * (-x104 * (x106 + x132 * x155 + x145 * x155 + 2 * x146 + x154) + x157));
	const GEN_FLT x159 = lh_qk * x77;
	const GEN_FLT x160 = x30 * x31;
	const GEN_FLT x161 = -x108 * x159 - x112 - x141 * x159 - x149 + x159 * x160;
	const GEN_FLT x162 = x161 * x70;
	const GEN_FLT x163 = -x97;
	const GEN_FLT x164 = x28 * (-x159 * x18 + x163);
	const GEN_FLT x165 = x119 + x132 * x159 + x145 * x159 + x164 + x89;
	const GEN_FLT x166 = x165 * x46;
	const GEN_FLT x167 = x161 * x72;
	const GEN_FLT x168 = x14 * (-x159 * x52 + x163);
	const GEN_FLT x169 = x125 * x159 + x139 + x150 * x159 + x168 + x84;
	const GEN_FLT x170 = x169 * x65;
	const GEN_FLT x171 = x105 * x28;
	const GEN_FLT x172 = lh_qk * x100;
	const GEN_FLT x173 = -x109 * (-x108 * x172 - x129 - x141 * x172 + x160 * x172 - x171);
	const GEN_FLT x174 = x105 * x31;
	const GEN_FLT x175 =
		x56 * (x169 * x61 + x57 * (-x104 * (x132 * x172 + x136 + x145 * x172 + 2 * x164 + x174) + x173));
	const GEN_FLT x176 = curve_1 * x60;
	const GEN_FLT x177 = 2 * x176;
	const GEN_FLT x178 = pow(-x42 * x64 * tilt_1 * tilt_1 + 1, -1.0 / 2.0);
	const GEN_FLT x179 = tilt_1 / sqrt(x63);
	const GEN_FLT x180 = x178 * x179;
	const GEN_FLT x181 = gibMag_1 * sin(gibPhase_1 - phase_1 + x66 - asin(x179 * x41) + 1.5707963267948966);
	const GEN_FLT x182 = tilt_1 * x41 / pow(x63, 3.0 / 2.0);
	const GEN_FLT x183 = x178 * x182;
	const GEN_FLT x184 = -x183 * x71 + x65;
	const GEN_FLT x185 = -x183 * x32 - x72;
	const GEN_FLT x186 = (1.0 / 2.0) * x54;
	const GEN_FLT x187 =
		-x178 * (x179 * x86 + x182 * (x110 - x186 * (x101 * x125 - x101 * x91 + x102 * x50 + x134 - x174))) + x88 + x93;
	const GEN_FLT x188 =
		x123 + x128 -
		x178 * (x117 * x179 + x182 * (x137 - x186 * (x107 + x125 * x131 + 2 * x126 + x131 * x150 - x154)));
	const GEN_FLT x189 =
		x152 + x153 -
		x178 * (x147 * x179 + x182 * (x157 - x186 * (x125 * x155 + x130 + x150 * x155 - x155 * x91 + x171)));
	const GEN_FLT x190 =
		x167 + x170 - x178 * (x165 * x179 + x182 * (x173 - x186 * (x125 * x172 + x150 * x172 + x156 + 2 * x168 + x96)));
	*(out++) = -x46 - x59 + x62 * (x46 + x59);
	*(out++) = x62 * x69 + x65 * x68 - x69;
	*(out++) = x62 * (-x70 + x73) - x68 * x72 + x70 - x73;
	*(out++) = -x111 + x62 * (x111 + x83 + x87) + x67 * (2 * x88 + 2 * x93) - x83 - x87;
	*(out++) = -x118 - x122 - x138 + x62 * (x118 + x122 + x138) + x67 * (2 * x123 + 2 * x128);
	*(out++) = -x144 - x148 - x158 + x62 * (x144 + x148 + x158) + x67 * (2 * x152 + 2 * x153);
	*(out++) = -x162 - x166 - x175 + x62 * (x162 + x166 + x175) + x67 * (2 * x167 + 2 * x170);
	*(out++) = x177 * x46 - x180 * x181 - x180;
	*(out++) = x181 * x184 + x184;
	*(out++) = -x177 * x70 + x181 * x185 + x185;
	*(out++) = x176 * (2 * x83 + 2 * x87) + x181 * x187 + x187;
	*(out++) = x176 * (2 * x118 + 2 * x122) + x181 * x188 + x188;
	*(out++) = x176 * (2 * x144 + 2 * x148) + x181 * x189 + x189;
	*(out++) = x176 * (2 * x162 + 2 * x166) + x181 * x190 + x190;
}

static inline void gen_reproject_axis_x_jac_lh_p(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
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
	const GEN_FLT x0 = lh_qi * lh_qw;
	const GEN_FLT x1 = lh_qj * lh_qk;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = obj_qi * obj_qj;
	const GEN_FLT x4 = obj_qk * obj_qw;
	const GEN_FLT x5 = obj_qi * obj_qi;
	const GEN_FLT x6 = obj_qj * obj_qj;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = x6 + x7;
	const GEN_FLT x9 = 2 * sqrt(obj_qw * obj_qw + x5 + x8);
	const GEN_FLT x10 = sensor_x * x9;
	const GEN_FLT x11 = obj_qj * obj_qk;
	const GEN_FLT x12 = obj_qi * obj_qw;
	const GEN_FLT x13 = sensor_z * x9;
	const GEN_FLT x14 = obj_py + sensor_y * (-x9 * (x5 + x7) + 1) + x10 * (x3 + x4) + x13 * (x11 - x12);
	const GEN_FLT x15 = lh_qi * lh_qi;
	const GEN_FLT x16 = lh_qj * lh_qj;
	const GEN_FLT x17 = lh_qk * lh_qk;
	const GEN_FLT x18 = x16 + x17;
	const GEN_FLT x19 = sqrt(lh_qw * lh_qw + x15 + x18);
	const GEN_FLT x20 = 2 * x19;
	const GEN_FLT x21 = x14 * x20;
	const GEN_FLT x22 = lh_qi * lh_qk;
	const GEN_FLT x23 = lh_qj * lh_qw;
	const GEN_FLT x24 = x22 - x23;
	const GEN_FLT x25 = obj_qi * obj_qk;
	const GEN_FLT x26 = obj_qj * obj_qw;
	const GEN_FLT x27 = sensor_y * x9;
	const GEN_FLT x28 = obj_px + sensor_x * (-x8 * x9 + 1) + x13 * (x25 + x26) + x27 * (x3 - x4);
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = x15 + x16;
	const GEN_FLT x31 = obj_pz + sensor_z * (-x9 * (x5 + x6) + 1) + x10 * (x25 - x26) + x27 * (x11 + x12);
	const GEN_FLT x32 = -lh_pz - x2 * x21 - x24 * x29 - x31 * (-x20 * x30 + 1);
	const GEN_FLT x33 = x22 + x23;
	const GEN_FLT x34 = x20 * x31;
	const GEN_FLT x35 = x33 * x34;
	const GEN_FLT x36 = lh_qi * lh_qj;
	const GEN_FLT x37 = lh_qk * lh_qw;
	const GEN_FLT x38 = x36 - x37;
	const GEN_FLT x39 = x21 * x38;
	const GEN_FLT x40 = x28 * (-x18 * x20 + 1);
	const GEN_FLT x41 = lh_px + x35 + x39 + x40;
	const GEN_FLT x42 = x32 * x32;
	const GEN_FLT x43 = x41 * x41 + x42;
	const GEN_FLT x44 = 1.0 / x43;
	const GEN_FLT x45 = x32 * x44;
	const GEN_FLT x46 = -lh_px - x35 - x39 - x40;
	const GEN_FLT x47 = x36 + x37;
	const GEN_FLT x48 = x29 * x47;
	const GEN_FLT x49 = -x0 + x1;
	const GEN_FLT x50 = x34 * x49;
	const GEN_FLT x51 = x15 + x17;
	const GEN_FLT x52 = x14 * (-x20 * x51 + 1);
	const GEN_FLT x53 = lh_py + x48 + x50 + x52;
	const GEN_FLT x54 = x53 * x53;
	const GEN_FLT x55 = pow(-x44 * x54 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x56 = tilt_0 * x53 / pow(x43, 3.0 / 2.0);
	const GEN_FLT x57 = x55 * x56;
	const GEN_FLT x58 = x46 * x57;
	const GEN_FLT x59 = tilt_0 / sqrt(x43);
	const GEN_FLT x60 = gibMag_0 * sin(-gibPhase_0 + phase_0 + asin(x53 * x59) + atan2(x41, x32) - 1.5707963267948966);
	const GEN_FLT x61 = curve_0 * atan2(x53, x32);
	const GEN_FLT x62 = 2 / (x42 + x54);
	const GEN_FLT x63 = x32 * x62;
	const GEN_FLT x64 = x55 * x59;
	const GEN_FLT x65 = x44 * x46;
	const GEN_FLT x66 = x62 * (-lh_py - x48 - x50 - x52);
	const GEN_FLT x67 = x32 * x57;
	const GEN_FLT x68 = lh_qi * x21;
	const GEN_FLT x69 = lh_qj * x29;
	const GEN_FLT x70 = 1.0 / x19;
	const GEN_FLT x71 = 2 * x70;
	const GEN_FLT x72 = lh_qw * x71;
	const GEN_FLT x73 = x31 * x72;
	const GEN_FLT x74 = x14 * x72;
	const GEN_FLT x75 = x28 * x72;
	const GEN_FLT x76 = -x2 * x74 - x24 * x75 + x30 * x73 - x68 + x69;
	const GEN_FLT x77 = x65 * x76;
	const GEN_FLT x78 = lh_qj * x34;
	const GEN_FLT x79 = -lh_qk * x21;
	const GEN_FLT x80 = x45 * (-x18 * x75 + x33 * x73 + x38 * x74 + x78 + x79);
	const GEN_FLT x81 = lh_qi * x34;
	const GEN_FLT x82 = lh_qk * x29;
	const GEN_FLT x83 = x14 * x51;
	const GEN_FLT x84 = x47 * x75 + x49 * x73 - x72 * x83 - x81 + x82;
	const GEN_FLT x85 = 4 * x19;
	const GEN_FLT x86 = lh_qj * x85;
	const GEN_FLT x87 = lh_qk * x85;
	const GEN_FLT x88 = -x14 * x87;
	const GEN_FLT x89 = x18 * x28;
	const GEN_FLT x90 = 4 * x70;
	const GEN_FLT x91 = lh_qw * x90;
	const GEN_FLT x92 = x31 * x91;
	const GEN_FLT x93 = x14 * x91;
	const GEN_FLT x94 = (1.0 / 2.0) * x41;
	const GEN_FLT x95 = lh_qi * x85;
	const GEN_FLT x96 = x14 * x95;
	const GEN_FLT x97 = x24 * x28;
	const GEN_FLT x98 = (1.0 / 2.0) * x32;
	const GEN_FLT x99 = x55 * (x56 * (-x94 * (x31 * x86 + x33 * x92 + x38 * x93 + x88 - x89 * x91) -
									  x98 * (-x2 * x93 + x28 * x86 + x30 * x92 - x91 * x97 - x96)) +
							   x59 * x84);
	const GEN_FLT x100 = lh_qj * x21;
	const GEN_FLT x101 = lh_qk * x34;
	const GEN_FLT x102 = lh_qi * x71;
	const GEN_FLT x103 = x102 * x31;
	const GEN_FLT x104 = x102 * x14;
	const GEN_FLT x105 = x45 * (x100 + x101 - x102 * x89 + x103 * x33 + x104 * x38);
	const GEN_FLT x106 = -lh_qw * x21;
	const GEN_FLT x107 = x31 * (x102 * x30 + x95);
	const GEN_FLT x108 = -x102 * x97 - x104 * x2 + x106 + x107 - x82;
	const GEN_FLT x109 = x108 * x65;
	const GEN_FLT x110 = lh_qw * x34;
	const GEN_FLT x111 = x28 * x47;
	const GEN_FLT x112 = x102 * x111 + x103 * x49 - x110 + x14 * (-x102 * x51 - x95) + x69;
	const GEN_FLT x113 = x14 * x86;
	const GEN_FLT x114 = lh_qi * x90;
	const GEN_FLT x115 = x31 * x33;
	const GEN_FLT x116 = x114 * x14;
	const GEN_FLT x117 = lh_qw * x85;
	const GEN_FLT x118 = -x117 * x14;
	const GEN_FLT x119 = x55 * (x112 * x59 + x56 * (-x94 * (x113 + x114 * x115 - x114 * x89 + x116 * x38 + x31 * x87) -
													x98 * (2 * x107 - x114 * x97 - x116 * x2 + x118 - x28 * x87)));
	const GEN_FLT x120 = lh_qw * x29;
	const GEN_FLT x121 = lh_qj * x71;
	const GEN_FLT x122 = x14 * x2;
	const GEN_FLT x123 = x31 * (x121 * x30 + x86);
	const GEN_FLT x124 = x120 - x121 * x122 - x121 * x97 + x123 + x79;
	const GEN_FLT x125 = x124 * x65;
	const GEN_FLT x126 = x14 * x38;
	const GEN_FLT x127 = x28 * (-x121 * x18 - x86);
	const GEN_FLT x128 = x45 * (x110 + x115 * x121 + x121 * x126 + x127 + x68);
	const GEN_FLT x129 = lh_qi * x29;
	const GEN_FLT x130 = x31 * x49;
	const GEN_FLT x131 = x101 + x111 * x121 + x121 * x130 - x121 * x83 + x129;
	const GEN_FLT x132 = lh_qj * x90;
	const GEN_FLT x133 = x55 * (x131 * x59 + x56 * (-x94 * (x115 * x132 + x117 * x31 + x126 * x132 + 2 * x127 + x96) -
													x98 * (x117 * x28 - x122 * x132 + 2 * x123 - x132 * x97 + x88)));
	const GEN_FLT x134 = lh_qk * x71;
	const GEN_FLT x135 = x30 * x31;
	const GEN_FLT x136 = -x100 - x122 * x134 - x129 + x134 * x135 - x134 * x97;
	const GEN_FLT x137 = x136 * x65;
	const GEN_FLT x138 = -x87;
	const GEN_FLT x139 = x28 * (-x134 * x18 + x138);
	const GEN_FLT x140 = x45 * (x106 + x115 * x134 + x126 * x134 + x139 + x81);
	const GEN_FLT x141 = x111 * x134 + x120 + x130 * x134 + x14 * (-x134 * x51 + x138) + x78;
	const GEN_FLT x142 = lh_qk * x90;
	const GEN_FLT x143 =
		x55 * (x141 * x59 + x56 * (-x94 * (x115 * x142 + x118 + x126 * x142 + 2 * x139 + x31 * x95) -
								   x98 * (-x113 - x122 * x142 + x135 * x142 - x142 * x97 - x28 * x95)));
	*(out++) = -x45 - x58 + x60 * (x45 + x58);
	*(out++) = x60 * x64 + x61 * x63 - x64;
	*(out++) = x60 * (-x65 + x67) - x61 * x66 + x65 - x67;
	*(out++) = x60 * (x77 + x80 + x99) + x61 * (x63 * x84 + x66 * x76) - x77 - x80 - x99;
	*(out++) = -x105 - x109 - x119 + x60 * (x105 + x109 + x119) + x61 * (x108 * x66 + x112 * x63);
	*(out++) = -x125 - x128 - x133 + x60 * (x125 + x128 + x133) + x61 * (x124 * x66 + x131 * x63);
	*(out++) = -x137 - x140 - x143 + x60 * (x137 + x140 + x143) + x61 * (x136 * x66 + x141 * x63);
}

static inline void gen_reproject_axis_y_jac_lh_p(FLT *out, const FLT *obj, const FLT *sensor, const FLT *lh,
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
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = obj_qi * obj_qw;
	const GEN_FLT x4 = obj_qj * obj_qk;
	const GEN_FLT x5 = obj_qj * obj_qj;
	const GEN_FLT x6 = obj_qi * obj_qi;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = x6 + x7;
	const GEN_FLT x9 = 2 * sqrt(obj_qw * obj_qw + x5 + x8);
	const GEN_FLT x10 = sensor_y * x9;
	const GEN_FLT x11 = obj_qi * obj_qk;
	const GEN_FLT x12 = obj_qj * obj_qw;
	const GEN_FLT x13 = sensor_x * x9;
	const GEN_FLT x14 = obj_pz + sensor_z * (-x9 * (x5 + x6) + 1) + x10 * (x3 + x4) + x13 * (x11 - x12);
	const GEN_FLT x15 = lh_qj * lh_qj;
	const GEN_FLT x16 = lh_qi * lh_qi;
	const GEN_FLT x17 = lh_qk * lh_qk;
	const GEN_FLT x18 = x16 + x17;
	const GEN_FLT x19 = sqrt(lh_qw * lh_qw + x15 + x18);
	const GEN_FLT x20 = 2 * x19;
	const GEN_FLT x21 = x14 * x20;
	const GEN_FLT x22 = x2 * x21;
	const GEN_FLT x23 = lh_qi * lh_qj;
	const GEN_FLT x24 = lh_qk * lh_qw;
	const GEN_FLT x25 = x23 - x24;
	const GEN_FLT x26 = obj_qi * obj_qj;
	const GEN_FLT x27 = obj_qk * obj_qw;
	const GEN_FLT x28 = sensor_z * x9;
	const GEN_FLT x29 = obj_py + sensor_y * (-x8 * x9 + 1) + x13 * (x26 + x27) + x28 * (-x3 + x4);
	const GEN_FLT x30 = x20 * x29;
	const GEN_FLT x31 = x25 * x30;
	const GEN_FLT x32 = x15 + x17;
	const GEN_FLT x33 = obj_px + sensor_x * (-x9 * (x5 + x7) + 1) + x10 * (x26 - x27) + x28 * (x11 + x12);
	const GEN_FLT x34 = x33 * (-x20 * x32 + 1);
	const GEN_FLT x35 = lh_px + x22 + x31 + x34;
	const GEN_FLT x36 = lh_qi * lh_qw;
	const GEN_FLT x37 = lh_qj * lh_qk;
	const GEN_FLT x38 = x36 + x37;
	const GEN_FLT x39 = x0 - x1;
	const GEN_FLT x40 = x20 * x33;
	const GEN_FLT x41 = x15 + x16;
	const GEN_FLT x42 = -lh_pz - x14 * (-x20 * x41 + 1) - x30 * x38 - x39 * x40;
	const GEN_FLT x43 = curve_0 * atan2(x35, x42);
	const GEN_FLT x44 = x35 * x35;
	const GEN_FLT x45 = x42 * x42;
	const GEN_FLT x46 = 2 / (x44 + x45);
	const GEN_FLT x47 = x42 * x46;
	const GEN_FLT x48 = x23 + x24;
	const GEN_FLT x49 = x40 * x48;
	const GEN_FLT x50 = -x36 + x37;
	const GEN_FLT x51 = x21 * x50;
	const GEN_FLT x52 = x29 * (-x18 * x20 + 1);
	const GEN_FLT x53 = lh_py + x49 + x51 + x52;
	const GEN_FLT x54 = x45 + x53 * x53;
	const GEN_FLT x55 = 1.0 / x54;
	const GEN_FLT x56 = pow(-x44 * x55 * tilt_0 * tilt_0 + 1, -1.0 / 2.0);
	const GEN_FLT x57 = tilt_0 / sqrt(x54);
	const GEN_FLT x58 = x56 * x57;
	const GEN_FLT x59 = gibMag_0 * sin(gibPhase_0 - phase_0 - asin(x35 * x57) + atan2(x53, x42) + 1.5707963267948966);
	const GEN_FLT x60 = x42 * x55;
	const GEN_FLT x61 = -lh_py - x49 - x51 - x52;
	const GEN_FLT x62 = tilt_0 * x35 / pow(x54, 3.0 / 2.0);
	const GEN_FLT x63 = x56 * x62;
	const GEN_FLT x64 = x60 - x61 * x63;
	const GEN_FLT x65 = x55 * x61;
	const GEN_FLT x66 = -x42 * x63 - x65;
	const GEN_FLT x67 = x46 * (-lh_px - x22 - x31 - x34);
	const GEN_FLT x68 = lh_qi * x30;
	const GEN_FLT x69 = lh_qj * x40;
	const GEN_FLT x70 = 1.0 / x19;
	const GEN_FLT x71 = 2 * x70;
	const GEN_FLT x72 = lh_qw * x71;
	const GEN_FLT x73 = x14 * x72;
	const GEN_FLT x74 = x29 * x72;
	const GEN_FLT x75 = x33 * x72;
	const GEN_FLT x76 = -x38 * x74 - x39 * x75 + x41 * x73 - x68 + x69;
	const GEN_FLT x77 = lh_qj * x21;
	const GEN_FLT x78 = -lh_qk * x30;
	const GEN_FLT x79 = x32 * x33;
	const GEN_FLT x80 = x2 * x73 + x25 * x74 - x72 * x79 + x77 + x78;
	const GEN_FLT x81 = lh_qi * x21;
	const GEN_FLT x82 = lh_qk * x40;
	const GEN_FLT x83 = 4 * x19;
	const GEN_FLT x84 = lh_qi * x83;
	const GEN_FLT x85 = lh_qk * x83;
	const GEN_FLT x86 = x33 * x85;
	const GEN_FLT x87 = x18 * x29;
	const GEN_FLT x88 = 4 * x70;
	const GEN_FLT x89 = lh_qw * x88;
	const GEN_FLT x90 = x33 * x89;
	const GEN_FLT x91 = x14 * x89;
	const GEN_FLT x92 = (1.0 / 2.0) * x53;
	const GEN_FLT x93 = lh_qj * x83;
	const GEN_FLT x94 = x33 * x93;
	const GEN_FLT x95 = x29 * x38;
	const GEN_FLT x96 = (1.0 / 2.0) * x42;
	const GEN_FLT x97 = -x56 * (x57 * x80 + x62 * (-x92 * (-x14 * x84 + x48 * x90 + x50 * x91 + x86 - x87 * x89) -
												   x96 * (-x29 * x84 - x39 * x90 + x41 * x91 - x89 * x95 + x94))) +
						x60 * (-x18 * x74 + x48 * x75 + x50 * x73 - x81 + x82) + x65 * x76;
	const GEN_FLT x98 = lh_qj * x30;
	const GEN_FLT x99 = lh_qk * x21;
	const GEN_FLT x100 = lh_qi * x71;
	const GEN_FLT x101 = x100 * x14;
	const GEN_FLT x102 = x25 * x29;
	const GEN_FLT x103 = x100 * x102 - x100 * x79 + x101 * x2 + x98 + x99;
	const GEN_FLT x104 = -lh_qw * x30;
	const GEN_FLT x105 = x33 * x39;
	const GEN_FLT x106 = x14 * (x100 * x41 + x84);
	const GEN_FLT x107 = -x100 * x105 - x100 * x95 + x104 + x106 - x82;
	const GEN_FLT x108 = lh_qw * x21;
	const GEN_FLT x109 = x33 * x48;
	const GEN_FLT x110 = x29 * (-x100 * x18 - x84);
	const GEN_FLT x111 = lh_qw * x83;
	const GEN_FLT x112 = lh_qi * x88;
	const GEN_FLT x113 = x14 * x50;
	const GEN_FLT x114 = x107 * x65 -
						 x56 * (x103 * x57 + x62 * (-x92 * (x109 * x112 + 2 * x110 - x111 * x14 + x112 * x113 + x94) -
													x96 * (-x105 * x112 + 2 * x106 - x111 * x29 - x112 * x95 - x86))) +
						 x60 * (x100 * x109 + x101 * x50 - x108 + x110 + x69);
	const GEN_FLT x115 = lh_qw * x40;
	const GEN_FLT x116 = lh_qj * x71;
	const GEN_FLT x117 = x14 * (x116 * x41 + x93);
	const GEN_FLT x118 = -x105 * x116 + x115 - x116 * x95 + x117 + x78;
	const GEN_FLT x119 = x14 * x2;
	const GEN_FLT x120 = x102 * x116 + x108 + x116 * x119 + x33 * (-x116 * x32 - x93) + x68;
	const GEN_FLT x121 = lh_qi * x40;
	const GEN_FLT x122 = x33 * x84;
	const GEN_FLT x123 = lh_qj * x88;
	const GEN_FLT x124 = x111 * x33;
	const GEN_FLT x125 = x118 * x65 -
						 x56 * (x120 * x57 + x62 * (-x92 * (x109 * x123 + x113 * x123 + x122 - x123 * x87 + x14 * x85) -
													x96 * (-x105 * x123 + 2 * x117 - x123 * x95 + x124 - x29 * x85))) +
						 x60 * (x109 * x116 + x113 * x116 - x116 * x87 + x121 + x99);
	const GEN_FLT x126 = lh_qk * x71;
	const GEN_FLT x127 = x14 * x41;
	const GEN_FLT x128 = -x105 * x126 - x121 + x126 * x127 - x126 * x95 - x98;
	const GEN_FLT x129 = -x85;
	const GEN_FLT x130 = x102 * x126 + x104 + x119 * x126 + x33 * (-x126 * x32 + x129) + x81;
	const GEN_FLT x131 = x29 * (-x126 * x18 + x129);
	const GEN_FLT x132 = lh_qk * x88;
	const GEN_FLT x133 =
		x128 * x65 -
		x56 * (x130 * x57 + x62 * (-x92 * (x109 * x132 + x113 * x132 + x124 + 2 * x131 + x14 * x93) -
								   x96 * (-x105 * x132 - x122 + x127 * x132 - x132 * x95 - x29 * x93))) +
		x60 * (x109 * x126 + x113 * x126 + x115 + x131 + x77);
	*(out++) = x43 * x47 - x58 * x59 - x58;
	*(out++) = x59 * x64 + x64;
	*(out++) = -x43 * x67 + x59 * x66 + x66;
	*(out++) = x43 * (x47 * x80 + x67 * x76) + x59 * x97 + x97;
	*(out++) = x114 * x59 + x114 + x43 * (x103 * x47 + x107 * x67);
	*(out++) = x125 * x59 + x125 + x43 * (x118 * x67 + x120 * x47);
	*(out++) = x133 * x59 + x133 + x43 * (x128 * x67 + x130 * x47);
}

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
	const GEN_FLT x56 = x34 * (1 - x55);
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
	const GEN_FLT x93 = pow(1 - x92 * x92, -1.0 / 2.0);
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
	const GEN_FLT x158 = 2 / x10;
	const GEN_FLT x159 = obj_qw * x158;
	const GEN_FLT x160 = sensor_x * x159;
	const GEN_FLT x161 = sensor_z * x159;
	const GEN_FLT x162 = sensor_y * x159;
	const GEN_FLT x163 = x15 * x162 + x156 - x157 - x160 * x9 + x161 * x5;
	const GEN_FLT x164 = x163 * x2;
	const GEN_FLT x165 = obj_qi * x16;
	const GEN_FLT x166 = obj_qj * x32;
	const GEN_FLT x167 = x160 * x31 - x161 * x33 + x162 * x30 + x165 - x166;
	const GEN_FLT x168 = x167 * x27;
	const GEN_FLT x169 = obj_qi * x12;
	const GEN_FLT x170 = obj_qk * x32;
	const GEN_FLT x171 = sensor_y * x40;
	const GEN_FLT x172 = -x159 * x171 + x160 * x38 + x161 * x39 - x169 + x170;
	const GEN_FLT x173 = x172 * x37;
	const GEN_FLT x174 = x164 * x23 + x168 * x23 + x173;
	const GEN_FLT x175 = x167 * x45;
	const GEN_FLT x176 = x172 * x98;
	const GEN_FLT x177 = x163 * x48;
	const GEN_FLT x178 = x163 * x52;
	const GEN_FLT x179 = x152 * x167;
	const GEN_FLT x180 = -x144 * (-x176 * x50 - x178 * x98 + 2 * x179) - x99 * (x175 * x98 + x176 * x46 + 2 * x177);
	const GEN_FLT x181 = -x130 * (x164 * x98 + x168 * x98 + 2 * x173) + x180;
	const GEN_FLT x182 = x95 * (x104 * x181 + x174 * x69);
	const GEN_FLT x183 = x182 * x72;
	const GEN_FLT x184 = x182 * x73 + x70 * (-x182 * x71 + x183);
	const GEN_FLT x185 = x182 * x74 + x184 * x70;
	const GEN_FLT x186 = x113 * x180 + x174 * x62;
	const GEN_FLT x187 = x172 * x23;
	const GEN_FLT x188 = x115 * (x175 * x23 + x177 + x187 * x46) + x116 * (-x178 * x23 + x179 - x187 * x50);
	const GEN_FLT x189 = -x111 * x186 + x188;
	const GEN_FLT x190 =
		x188 - x93 * (x125 * (x121 * x189 -
							  x123 * (x182 * x75 + x182 * x85 + x185 * x70 +
									  x70 * (x182 * x84 + x185 +
											 x70 * (x182 * x83 + x184 + x70 * (-x122 * x182 + x182 * x82 + x183))))) +
					  x126 * x189 + x127 * x182 + x185 * x91 + x186);
	const GEN_FLT x191 = obj_qj * x16;
	const GEN_FLT x192 = obj_qk * x12;
	const GEN_FLT x193 = obj_qi * x158;
	const GEN_FLT x194 = sensor_x * x193;
	const GEN_FLT x195 = sensor_z * x193;
	const GEN_FLT x196 = sensor_y * x193;
	const GEN_FLT x197 = x15 * x196 + x191 + x192 - x194 * x9 + x195 * x5;
	const GEN_FLT x198 = x197 * x2;
	const GEN_FLT x199 = obj_qw * x16;
	const GEN_FLT x200 = 4 * x10;
	const GEN_FLT x201 = -obj_qi * x200;
	const GEN_FLT x202 = sensor_z * (-x193 * x33 + x201) + x170 + x194 * x31 + x196 * x30 + x199;
	const GEN_FLT x203 = x202 * x27;
	const GEN_FLT x204 = obj_qw * x12;
	const GEN_FLT x205 = sensor_y * (-x193 * x40 + x201) + x166 + x194 * x38 + x195 * x39 - x204;
	const GEN_FLT x206 = x205 * x37;
	const GEN_FLT x207 = x198 * x23 + x203 * x23 + x206;
	const GEN_FLT x208 = x197 * x48;
	const GEN_FLT x209 = x202 * x45;
	const GEN_FLT x210 = x205 * x98;
	const GEN_FLT x211 = x197 * x52;
	const GEN_FLT x212 = x152 * x202;
	const GEN_FLT x213 = -x144 * (-x210 * x50 - x211 * x98 + 2 * x212) - x99 * (2 * x208 + x209 * x98 + x210 * x46);
	const GEN_FLT x214 = -x130 * (x198 * x98 + x203 * x98 + 2 * x206) + x213;
	const GEN_FLT x215 = x95 * (x104 * x214 + x207 * x69);
	const GEN_FLT x216 = x215 * x72;
	const GEN_FLT x217 = x215 * x73 + x70 * (-x215 * x71 + x216);
	const GEN_FLT x218 = x215 * x74 + x217 * x70;
	const GEN_FLT x219 = x113 * x213 + x207 * x62;
	const GEN_FLT x220 = x205 * x23;
	const GEN_FLT x221 = x115 * (x208 + x209 * x23 + x220 * x46) + x116 * (-x211 * x23 + x212 - x220 * x50);
	const GEN_FLT x222 = -x111 * x219 + x221;
	const GEN_FLT x223 =
		x221 - x93 * (x125 * (x121 * x222 -
							  x123 * (x215 * x75 + x215 * x85 + x218 * x70 +
									  x70 * (x215 * x84 + x218 +
											 x70 * (x215 * x83 + x217 + x70 * (-x122 * x215 + x215 * x82 + x216))))) +
					  x126 * x222 + x127 * x215 + x218 * x91 + x219);
	const GEN_FLT x224 = obj_qi * x32;
	const GEN_FLT x225 = obj_qj * x158;
	const GEN_FLT x226 = sensor_x * x225;
	const GEN_FLT x227 = sensor_z * x225;
	const GEN_FLT x228 = -x171 * x225 + x192 + x224 + x226 * x38 + x227 * x39;
	const GEN_FLT x229 = x228 * x37;
	const GEN_FLT x230 = sensor_y * x225;
	const GEN_FLT x231 = -obj_qj * x200;
	const GEN_FLT x232 = sensor_x * (-x225 * x9 + x231) + x15 * x230 + x165 + x204 + x227 * x5;
	const GEN_FLT x233 = x2 * x232;
	const GEN_FLT x234 = obj_qw * x32;
	const GEN_FLT x235 = sensor_z * (-x225 * x33 + x231) + x157 + x226 * x31 + x230 * x30 - x234;
	const GEN_FLT x236 = x235 * x27;
	const GEN_FLT x237 = x229 + x23 * x233 + x23 * x236;
	const GEN_FLT x238 = x228 * x98;
	const GEN_FLT x239 = x235 * x45;
	const GEN_FLT x240 = x232 * x48;
	const GEN_FLT x241 = x232 * x52;
	const GEN_FLT x242 = x152 * x235;
	const GEN_FLT x243 = -x144 * (-x238 * x50 - x241 * x98 + 2 * x242) - x99 * (x238 * x46 + x239 * x98 + 2 * x240);
	const GEN_FLT x244 = -x130 * (2 * x229 + x233 * x98 + x236 * x98) + x243;
	const GEN_FLT x245 = x95 * (x104 * x244 + x237 * x69);
	const GEN_FLT x246 = x245 * x72;
	const GEN_FLT x247 = x245 * x73 + x70 * (-x245 * x71 + x246);
	const GEN_FLT x248 = x245 * x74 + x247 * x70;
	const GEN_FLT x249 = x113 * x243 + x237 * x62;
	const GEN_FLT x250 = x228 * x23;
	const GEN_FLT x251 = x115 * (x23 * x239 + x240 + x250 * x46) + x116 * (-x23 * x241 + x242 - x250 * x50);
	const GEN_FLT x252 = -x111 * x249 + x251;
	const GEN_FLT x253 =
		x251 - x93 * (x125 * (x121 * x252 -
							  x123 * (x245 * x75 + x245 * x85 + x248 * x70 +
									  x70 * (x245 * x84 + x248 +
											 x70 * (x245 * x83 + x247 + x70 * (-x122 * x245 + x245 * x82 + x246))))) +
					  x126 * x252 + x127 * x245 + x248 * x91 + x249);
	const GEN_FLT x254 = obj_qk * x158;
	const GEN_FLT x255 = sensor_y * x254;
	const GEN_FLT x256 = sensor_z * x254;
	const GEN_FLT x257 = sensor_x * x254;
	const GEN_FLT x258 = x191 + x224 + x255 * x30 - x256 * x33 + x257 * x31;
	const GEN_FLT x259 = x258 * x27;
	const GEN_FLT x260 = -obj_qk * x200;
	const GEN_FLT x261 = sensor_x * (-x254 * x9 + x260) + x15 * x255 + x169 - x199 + x256 * x5;
	const GEN_FLT x262 = x2 * x261;
	const GEN_FLT x263 = sensor_y * (-x254 * x40 + x260) + x156 + x234 + x256 * x39 + x257 * x38;
	const GEN_FLT x264 = x263 * x37;
	const GEN_FLT x265 = x23 * x259 + x23 * x262 + x264;
	const GEN_FLT x266 = x258 * x45;
	const GEN_FLT x267 = x263 * x98;
	const GEN_FLT x268 = x261 * x48;
	const GEN_FLT x269 = x152 * x258;
	const GEN_FLT x270 = x261 * x52;
	const GEN_FLT x271 = -x144 * (-x267 * x50 + 2 * x269 - x270 * x98) - x99 * (x266 * x98 + x267 * x46 + 2 * x268);
	const GEN_FLT x272 = -x130 * (x259 * x98 + x262 * x98 + 2 * x264) + x271;
	const GEN_FLT x273 = x95 * (x104 * x272 + x265 * x69);
	const GEN_FLT x274 = x273 * x72;
	const GEN_FLT x275 = x273 * x73 + x70 * (-x273 * x71 + x274);
	const GEN_FLT x276 = x273 * x74 + x275 * x70;
	const GEN_FLT x277 = x113 * x271 + x265 * x62;
	const GEN_FLT x278 = x23 * x263;
	const GEN_FLT x279 = x115 * (x23 * x266 + x268 + x278 * x46) + x116 * (-x23 * x270 + x269 - x278 * x50);
	const GEN_FLT x280 = -x111 * x277 + x279;
	const GEN_FLT x281 =
		x279 - x93 * (x125 * (x121 * x280 -
							  x123 * (x273 * x75 + x273 * x85 + x276 * x70 +
									  x70 * (x273 * x84 + x276 +
											 x70 * (x273 * x83 + x275 + x70 * (-x122 * x273 + x273 * x82 + x274))))) +
					  x126 * x280 + x127 * x273 + x276 * x91 + x277);
	const GEN_FLT x282 = tilt_1 - 0.52359877559829882;
	const GEN_FLT x283 = tan(x282);
	const GEN_FLT x284 = x283 * x59;
	const GEN_FLT x285 = x284 * x42;
	const GEN_FLT x286 = cos(x282);
	const GEN_FLT x287 = 1.0 / x286;
	const GEN_FLT x288 = x287 * x66;
	const GEN_FLT x289 = asin(x288 * x42);
	const GEN_FLT x290 = 8.0108022e-6 * x289;
	const GEN_FLT x291 = -x290 - 8.0108022e-6;
	const GEN_FLT x292 = x289 * x291 + 0.0028679863;
	const GEN_FLT x293 = x289 * x292 + 5.3685255000000001e-6;
	const GEN_FLT x294 = x289 * x293 + 0.0076069798000000001;
	const GEN_FLT x295 = x289 * x289;
	const GEN_FLT x296 = ogeePhase_1 + x77 - asin(x285);
	const GEN_FLT x297 = ogeeMag_1 * sin(x296);
	const GEN_FLT x298 = curve_1 + x297;
	const GEN_FLT x299 = x289 * x294;
	const GEN_FLT x300 = -1.60216044e-5 * x289 - 8.0108022e-6;
	const GEN_FLT x301 = x289 * x300 + x292;
	const GEN_FLT x302 = x289 * x301 + x293;
	const GEN_FLT x303 = x289 * x302 + x294;
	const GEN_FLT x304 = sin(x282);
	const GEN_FLT x305 = x304 * (x289 * x303 + x299);
	const GEN_FLT x306 = x286 - x298 * x305;
	const GEN_FLT x307 = 1.0 / x306;
	const GEN_FLT x308 = x298 * x307;
	const GEN_FLT x309 = x295 * x308;
	const GEN_FLT x310 = x285 + x294 * x309;
	const GEN_FLT x311 = pow(1 - x310 * x310, -1.0 / 2.0);
	const GEN_FLT x312 = pow(-x94 / (x286 * x286) + 1, -1.0 / 2.0);
	const GEN_FLT x313 = x103 * x287;
	const GEN_FLT x314 = x312 * (x102 * x313 + x288 * x96);
	const GEN_FLT x315 = x291 * x314;
	const GEN_FLT x316 = x289 * (-x290 * x314 + x315) + x292 * x314;
	const GEN_FLT x317 = x289 * x316 + x293 * x314;
	const GEN_FLT x318 = pow(-x110 * x283 * x283 + 1, -1.0 / 2.0);
	const GEN_FLT x319 = x112 * x283;
	const GEN_FLT x320 = x101 * x319 + x284 * x96;
	const GEN_FLT x321 = x118 - x318 * x320;
	const GEN_FLT x322 = ogeeMag_1 * cos(x296);
	const GEN_FLT x323 = x305 * x322;
	const GEN_FLT x324 = 2.40324066e-5 * x289;
	const GEN_FLT x325 = x304 * (-curve_1 - x297);
	const GEN_FLT x326 = x294 * x295;
	const GEN_FLT x327 = x298 * x326 / ((x306 * x306));
	const GEN_FLT x328 = x307 * x322 * x326;
	const GEN_FLT x329 = 2 * x299 * x308;
	const GEN_FLT x330 =
		x118 - x311 * (x309 * x317 + x314 * x329 + x320 + x321 * x328 +
					   x327 * (x321 * x323 -
							   x325 * (x289 * x317 +
									   x289 * (x289 * (x289 * (x300 * x314 - x314 * x324 + x315) + x301 * x314 + x316) +
											   x302 * x314 + x317) +
									   x294 * x314 + x303 * x314)));
	const GEN_FLT x331 = gibMag_1 * cos(gibPhase_1 + x77 - asin(x310));
	const GEN_FLT x332 = x312 * (x133 * x313 + x288 * x37);
	const GEN_FLT x333 = x291 * x332;
	const GEN_FLT x334 = x289 * (-x290 * x332 + x333) + x292 * x332;
	const GEN_FLT x335 = x289 * x334 + x293 * x332;
	const GEN_FLT x336 = x132 * x319 + x284 * x37;
	const GEN_FLT x337 = x140 - x318 * x336;
	const GEN_FLT x338 =
		x140 - x311 * (x309 * x335 +
					   x327 * (x323 * x337 -
							   x325 * (x289 * x335 +
									   x289 * (x289 * (x289 * (x300 * x332 - x324 * x332 + x333) + x301 * x332 + x334) +
											   x302 * x332 + x335) +
									   x294 * x332 + x303 * x332)) +
					   x328 * x337 + x329 * x332 + x336);
	const GEN_FLT x339 = x312 * (x143 * x288 + x146 * x313);
	const GEN_FLT x340 = x291 * x339;
	const GEN_FLT x341 = x289 * (-x290 * x339 + x340) + x292 * x339;
	const GEN_FLT x342 = x289 * x341 + x293 * x339;
	const GEN_FLT x343 = x143 * x284 + x145 * x319;
	const GEN_FLT x344 = x153 - x318 * x343;
	const GEN_FLT x345 =
		x153 - x311 * (x309 * x342 +
					   x327 * (x323 * x344 -
							   x325 * (x289 * x342 +
									   x289 * (x289 * (x289 * (x300 * x339 - x324 * x339 + x340) + x301 * x339 + x341) +
											   x302 * x339 + x342) +
									   x294 * x339 + x303 * x339)) +
					   x328 * x344 + x329 * x339 + x343);
	const GEN_FLT x346 = x312 * (x174 * x288 + x181 * x313);
	const GEN_FLT x347 = x291 * x346;
	const GEN_FLT x348 = x289 * (-x290 * x346 + x347) + x292 * x346;
	const GEN_FLT x349 = x289 * x348 + x293 * x346;
	const GEN_FLT x350 = x174 * x284 + x180 * x319;
	const GEN_FLT x351 = x188 - x318 * x350;
	const GEN_FLT x352 =
		x188 - x311 * (x309 * x349 +
					   x327 * (x323 * x351 -
							   x325 * (x289 * x349 +
									   x289 * (x289 * (x289 * (x300 * x346 - x324 * x346 + x347) + x301 * x346 + x348) +
											   x302 * x346 + x349) +
									   x294 * x346 + x303 * x346)) +
					   x328 * x351 + x329 * x346 + x350);
	const GEN_FLT x353 = x312 * (x207 * x288 + x214 * x313);
	const GEN_FLT x354 = x291 * x353;
	const GEN_FLT x355 = x289 * (-x290 * x353 + x354) + x292 * x353;
	const GEN_FLT x356 = x289 * x355 + x293 * x353;
	const GEN_FLT x357 = x207 * x284 + x213 * x319;
	const GEN_FLT x358 = x221 - x318 * x357;
	const GEN_FLT x359 =
		x221 - x311 * (x309 * x356 +
					   x327 * (x323 * x358 -
							   x325 * (x289 * x356 +
									   x289 * (x289 * (x289 * (x300 * x353 - x324 * x353 + x354) + x301 * x353 + x355) +
											   x302 * x353 + x356) +
									   x294 * x353 + x303 * x353)) +
					   x328 * x358 + x329 * x353 + x357);
	const GEN_FLT x360 = x312 * (x237 * x288 + x244 * x313);
	const GEN_FLT x361 = x291 * x360;
	const GEN_FLT x362 = x289 * (-x290 * x360 + x361) + x292 * x360;
	const GEN_FLT x363 = x289 * x362 + x293 * x360;
	const GEN_FLT x364 = x237 * x284 + x243 * x319;
	const GEN_FLT x365 = x251 - x318 * x364;
	const GEN_FLT x366 =
		x251 - x311 * (x309 * x363 +
					   x327 * (x323 * x365 -
							   x325 * (x289 * x363 +
									   x289 * (x289 * (x289 * (x300 * x360 - x324 * x360 + x361) + x301 * x360 + x362) +
											   x302 * x360 + x363) +
									   x294 * x360 + x303 * x360)) +
					   x328 * x365 + x329 * x360 + x364);
	const GEN_FLT x367 = x312 * (x265 * x288 + x272 * x313);
	const GEN_FLT x368 = x291 * x367;
	const GEN_FLT x369 = x289 * (-x290 * x367 + x368) + x292 * x367;
	const GEN_FLT x370 = x289 * x369 + x293 * x367;
	const GEN_FLT x371 = x265 * x284 + x271 * x319;
	const GEN_FLT x372 = x279 - x318 * x371;
	const GEN_FLT x373 =
		x279 - x311 * (x309 * x370 +
					   x327 * (x323 * x372 -
							   x325 * (x289 * x370 +
									   x289 * (x289 * (x289 * (x300 * x367 - x324 * x367 + x368) + x301 * x367 + x369) +
											   x302 * x367 + x370) +
									   x294 * x367 + x303 * x367)) +
					   x328 * x372 + x329 * x367 + x371);
	*(out++) = x128 * x129 + x128;
	*(out++) = x129 * x142 + x142;
	*(out++) = x129 * x155 + x155;
	*(out++) = x129 * x190 + x190;
	*(out++) = x129 * x223 + x223;
	*(out++) = x129 * x253 + x253;
	*(out++) = x129 * x281 + x281;
	*(out++) = x330 * x331 + x330;
	*(out++) = x331 * x338 + x338;
	*(out++) = x331 * x345 + x345;
	*(out++) = x331 * x352 + x352;
	*(out++) = x331 * x359 + x359;
	*(out++) = x331 * x366 + x366;
	*(out++) = x331 * x373 + x373;
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
	const GEN_FLT x56 = x34 * (1 - x55);
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
	const GEN_FLT x91 = pow(1 - x90 * x90, -1.0 / 2.0);
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
	const GEN_FLT x149 = 2 / x10;
	const GEN_FLT x150 = obj_qw * x149;
	const GEN_FLT x151 = sensor_x * x150;
	const GEN_FLT x152 = sensor_z * x150;
	const GEN_FLT x153 = sensor_y * x150;
	const GEN_FLT x154 = x147 - x148 + x15 * x153 - x151 * x9 + x152 * x5;
	const GEN_FLT x155 = x154 * x2;
	const GEN_FLT x156 = obj_qi * x16;
	const GEN_FLT x157 = obj_qj * x32;
	const GEN_FLT x158 = x151 * x31 - x152 * x33 + x153 * x30 + x156 - x157;
	const GEN_FLT x159 = x158 * x27;
	const GEN_FLT x160 = obj_qi * x12;
	const GEN_FLT x161 = obj_qk * x32;
	const GEN_FLT x162 = sensor_y * x40;
	const GEN_FLT x163 = -x150 * x162 + x151 * x38 + x152 * x39 - x160 + x161;
	const GEN_FLT x164 = x163 * x37;
	const GEN_FLT x165 = x155 * x23 + x159 * x23 + x164;
	const GEN_FLT x166 = x158 * x45;
	const GEN_FLT x167 = x163 * x95;
	const GEN_FLT x168 = x154 * x48;
	const GEN_FLT x169 = x154 * x52;
	const GEN_FLT x170 = x143 * x158;
	const GEN_FLT x171 = -x136 * (-x167 * x50 - x169 * x95 + 2 * x170) - x96 * (x166 * x95 + x167 * x46 + 2 * x168);
	const GEN_FLT x172 = x92 * (x165 * x67 + x99 * (-x123 * (x155 * x95 + x159 * x95 + 2 * x164) + x171));
	const GEN_FLT x173 = x172 * x70;
	const GEN_FLT x174 = x172 * x71 + x68 * (-x172 * x69 + x173);
	const GEN_FLT x175 = x172 * x72 + x174 * x68;
	const GEN_FLT x176 = x106 * x171 + x165 * x61;
	const GEN_FLT x177 = x163 * x23;
	const GEN_FLT x178 = x108 * (x166 * x23 + x168 + x177 * x46) + x109 * (-x169 * x23 + x170 - x177 * x50);
	const GEN_FLT x179 = -x105 * x176 + x178;
	const GEN_FLT x180 =
		x178 - x91 * (x118 * (x114 * x179 -
							  x116 * (x172 * x73 + x172 * x83 + x175 * x68 +
									  x68 * (x172 * x82 + x175 +
											 x68 * (x172 * x81 + x174 + x68 * (-x115 * x172 + x172 * x80 + x173))))) +
					  x119 * x179 + x120 * x172 + x175 * x89 + x176);
	const GEN_FLT x181 = obj_qj * x16;
	const GEN_FLT x182 = obj_qk * x12;
	const GEN_FLT x183 = obj_qi * x149;
	const GEN_FLT x184 = sensor_x * x183;
	const GEN_FLT x185 = sensor_z * x183;
	const GEN_FLT x186 = sensor_y * x183;
	const GEN_FLT x187 = x15 * x186 + x181 + x182 - x184 * x9 + x185 * x5;
	const GEN_FLT x188 = x187 * x2;
	const GEN_FLT x189 = obj_qw * x16;
	const GEN_FLT x190 = 4 * x10;
	const GEN_FLT x191 = -obj_qi * x190;
	const GEN_FLT x192 = sensor_z * (-x183 * x33 + x191) + x161 + x184 * x31 + x186 * x30 + x189;
	const GEN_FLT x193 = x192 * x27;
	const GEN_FLT x194 = obj_qw * x12;
	const GEN_FLT x195 = sensor_y * (-x183 * x40 + x191) + x157 + x184 * x38 + x185 * x39 - x194;
	const GEN_FLT x196 = x195 * x37;
	const GEN_FLT x197 = x188 * x23 + x193 * x23 + x196;
	const GEN_FLT x198 = x187 * x48;
	const GEN_FLT x199 = x192 * x45;
	const GEN_FLT x200 = x195 * x95;
	const GEN_FLT x201 = x187 * x52;
	const GEN_FLT x202 = x143 * x192;
	const GEN_FLT x203 = -x136 * (-x200 * x50 - x201 * x95 + 2 * x202) - x96 * (2 * x198 + x199 * x95 + x200 * x46);
	const GEN_FLT x204 = x92 * (x197 * x67 + x99 * (-x123 * (x188 * x95 + x193 * x95 + 2 * x196) + x203));
	const GEN_FLT x205 = x204 * x70;
	const GEN_FLT x206 = x204 * x71 + x68 * (-x204 * x69 + x205);
	const GEN_FLT x207 = x204 * x72 + x206 * x68;
	const GEN_FLT x208 = x106 * x203 + x197 * x61;
	const GEN_FLT x209 = x195 * x23;
	const GEN_FLT x210 = x108 * (x198 + x199 * x23 + x209 * x46) + x109 * (-x201 * x23 + x202 - x209 * x50);
	const GEN_FLT x211 = -x105 * x208 + x210;
	const GEN_FLT x212 =
		x210 - x91 * (x118 * (x114 * x211 -
							  x116 * (x204 * x73 + x204 * x83 + x207 * x68 +
									  x68 * (x204 * x82 + x207 +
											 x68 * (x204 * x81 + x206 + x68 * (-x115 * x204 + x204 * x80 + x205))))) +
					  x119 * x211 + x120 * x204 + x207 * x89 + x208);
	const GEN_FLT x213 = obj_qi * x32;
	const GEN_FLT x214 = obj_qj * x149;
	const GEN_FLT x215 = sensor_x * x214;
	const GEN_FLT x216 = sensor_z * x214;
	const GEN_FLT x217 = -x162 * x214 + x182 + x213 + x215 * x38 + x216 * x39;
	const GEN_FLT x218 = x217 * x37;
	const GEN_FLT x219 = sensor_y * x214;
	const GEN_FLT x220 = -obj_qj * x190;
	const GEN_FLT x221 = sensor_x * (-x214 * x9 + x220) + x15 * x219 + x156 + x194 + x216 * x5;
	const GEN_FLT x222 = x2 * x221;
	const GEN_FLT x223 = obj_qw * x32;
	const GEN_FLT x224 = sensor_z * (-x214 * x33 + x220) + x148 + x215 * x31 + x219 * x30 - x223;
	const GEN_FLT x225 = x224 * x27;
	const GEN_FLT x226 = x218 + x222 * x23 + x225 * x23;
	const GEN_FLT x227 = x217 * x95;
	const GEN_FLT x228 = x224 * x45;
	const GEN_FLT x229 = x221 * x48;
	const GEN_FLT x230 = x221 * x52;
	const GEN_FLT x231 = x143 * x224;
	const GEN_FLT x232 = -x136 * (-x227 * x50 - x230 * x95 + 2 * x231) - x96 * (x227 * x46 + x228 * x95 + 2 * x229);
	const GEN_FLT x233 = x92 * (x226 * x67 + x99 * (-x123 * (2 * x218 + x222 * x95 + x225 * x95) + x232));
	const GEN_FLT x234 = x233 * x70;
	const GEN_FLT x235 = x233 * x71 + x68 * (-x233 * x69 + x234);
	const GEN_FLT x236 = x233 * x72 + x235 * x68;
	const GEN_FLT x237 = x106 * x232 + x226 * x61;
	const GEN_FLT x238 = x217 * x23;
	const GEN_FLT x239 = x108 * (x228 * x23 + x229 + x238 * x46) + x109 * (-x23 * x230 + x231 - x238 * x50);
	const GEN_FLT x240 = -x105 * x237 + x239;
	const GEN_FLT x241 =
		x239 - x91 * (x118 * (x114 * x240 -
							  x116 * (x233 * x73 + x233 * x83 + x236 * x68 +
									  x68 * (x233 * x82 + x236 +
											 x68 * (x233 * x81 + x235 + x68 * (-x115 * x233 + x233 * x80 + x234))))) +
					  x119 * x240 + x120 * x233 + x236 * x89 + x237);
	const GEN_FLT x242 = obj_qk * x149;
	const GEN_FLT x243 = sensor_y * x242;
	const GEN_FLT x244 = sensor_z * x242;
	const GEN_FLT x245 = sensor_x * x242;
	const GEN_FLT x246 = x181 + x213 + x243 * x30 - x244 * x33 + x245 * x31;
	const GEN_FLT x247 = x246 * x27;
	const GEN_FLT x248 = -obj_qk * x190;
	const GEN_FLT x249 = sensor_x * (-x242 * x9 + x248) + x15 * x243 + x160 - x189 + x244 * x5;
	const GEN_FLT x250 = x2 * x249;
	const GEN_FLT x251 = sensor_y * (-x242 * x40 + x248) + x147 + x223 + x244 * x39 + x245 * x38;
	const GEN_FLT x252 = x251 * x37;
	const GEN_FLT x253 = x23 * x247 + x23 * x250 + x252;
	const GEN_FLT x254 = x246 * x45;
	const GEN_FLT x255 = x251 * x95;
	const GEN_FLT x256 = x249 * x48;
	const GEN_FLT x257 = x143 * x246;
	const GEN_FLT x258 = x249 * x52;
	const GEN_FLT x259 = -x136 * (-x255 * x50 + 2 * x257 - x258 * x95) - x96 * (x254 * x95 + x255 * x46 + 2 * x256);
	const GEN_FLT x260 = x92 * (x253 * x67 + x99 * (-x123 * (x247 * x95 + x250 * x95 + 2 * x252) + x259));
	const GEN_FLT x261 = x260 * x70;
	const GEN_FLT x262 = x260 * x71 + x68 * (-x260 * x69 + x261);
	const GEN_FLT x263 = x260 * x72 + x262 * x68;
	const GEN_FLT x264 = x106 * x259 + x253 * x61;
	const GEN_FLT x265 = x23 * x251;
	const GEN_FLT x266 = x108 * (x23 * x254 + x256 + x265 * x46) + x109 * (-x23 * x258 + x257 - x265 * x50);
	const GEN_FLT x267 = -x105 * x264 + x266;
	const GEN_FLT x268 =
		x266 - x91 * (x118 * (x114 * x267 -
							  x116 * (x260 * x73 + x260 * x83 + x263 * x68 +
									  x68 * (x260 * x82 + x263 +
											 x68 * (x260 * x81 + x262 + x68 * (-x115 * x260 + x260 * x80 + x261))))) +
					  x119 * x267 + x120 * x260 + x263 * x89 + x264);
	*(out++) = x121 * x122 + x121;
	*(out++) = x122 * x134 + x134;
	*(out++) = x122 * x146 + x146;
	*(out++) = x122 * x180 + x180;
	*(out++) = x122 * x212 + x212;
	*(out++) = x122 * x241 + x241;
	*(out++) = x122 * x268 + x268;
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
	const GEN_FLT x56 = x34 * (1 - x55);
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
	const GEN_FLT x91 = pow(1 - x90 * x90, -1.0 / 2.0);
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
	const GEN_FLT x149 = 2 / x10;
	const GEN_FLT x150 = obj_qw * x149;
	const GEN_FLT x151 = sensor_x * x150;
	const GEN_FLT x152 = sensor_z * x150;
	const GEN_FLT x153 = sensor_y * x150;
	const GEN_FLT x154 = x147 - x148 + x15 * x153 - x151 * x9 + x152 * x5;
	const GEN_FLT x155 = x154 * x2;
	const GEN_FLT x156 = obj_qi * x16;
	const GEN_FLT x157 = obj_qj * x32;
	const GEN_FLT x158 = x151 * x31 - x152 * x33 + x153 * x30 + x156 - x157;
	const GEN_FLT x159 = x158 * x27;
	const GEN_FLT x160 = obj_qi * x12;
	const GEN_FLT x161 = obj_qk * x32;
	const GEN_FLT x162 = sensor_y * x40;
	const GEN_FLT x163 = -x150 * x162 + x151 * x38 + x152 * x39 - x160 + x161;
	const GEN_FLT x164 = x163 * x37;
	const GEN_FLT x165 = x155 * x23 + x159 * x23 + x164;
	const GEN_FLT x166 = x158 * x45;
	const GEN_FLT x167 = x163 * x95;
	const GEN_FLT x168 = x154 * x48;
	const GEN_FLT x169 = x154 * x52;
	const GEN_FLT x170 = x143 * x158;
	const GEN_FLT x171 = -x136 * (-x167 * x50 - x169 * x95 + 2 * x170) - x96 * (x166 * x95 + x167 * x46 + 2 * x168);
	const GEN_FLT x172 = x92 * (x165 * x67 + x99 * (-x123 * (x155 * x95 + x159 * x95 + 2 * x164) + x171));
	const GEN_FLT x173 = x172 * x70;
	const GEN_FLT x174 = x172 * x71 + x68 * (-x172 * x69 + x173);
	const GEN_FLT x175 = x172 * x72 + x174 * x68;
	const GEN_FLT x176 = x106 * x171 + x165 * x61;
	const GEN_FLT x177 = x163 * x23;
	const GEN_FLT x178 = x108 * (x166 * x23 + x168 + x177 * x46) + x109 * (-x169 * x23 + x170 - x177 * x50);
	const GEN_FLT x179 = -x105 * x176 + x178;
	const GEN_FLT x180 =
		x178 - x91 * (x118 * (x114 * x179 -
							  x116 * (x172 * x73 + x172 * x83 + x175 * x68 +
									  x68 * (x172 * x82 + x175 +
											 x68 * (x172 * x81 + x174 + x68 * (-x115 * x172 + x172 * x80 + x173))))) +
					  x119 * x179 + x120 * x172 + x175 * x89 + x176);
	const GEN_FLT x181 = obj_qj * x16;
	const GEN_FLT x182 = obj_qk * x12;
	const GEN_FLT x183 = obj_qi * x149;
	const GEN_FLT x184 = sensor_x * x183;
	const GEN_FLT x185 = sensor_z * x183;
	const GEN_FLT x186 = sensor_y * x183;
	const GEN_FLT x187 = x15 * x186 + x181 + x182 - x184 * x9 + x185 * x5;
	const GEN_FLT x188 = x187 * x2;
	const GEN_FLT x189 = obj_qw * x16;
	const GEN_FLT x190 = 4 * x10;
	const GEN_FLT x191 = -obj_qi * x190;
	const GEN_FLT x192 = sensor_z * (-x183 * x33 + x191) + x161 + x184 * x31 + x186 * x30 + x189;
	const GEN_FLT x193 = x192 * x27;
	const GEN_FLT x194 = obj_qw * x12;
	const GEN_FLT x195 = sensor_y * (-x183 * x40 + x191) + x157 + x184 * x38 + x185 * x39 - x194;
	const GEN_FLT x196 = x195 * x37;
	const GEN_FLT x197 = x188 * x23 + x193 * x23 + x196;
	const GEN_FLT x198 = x187 * x48;
	const GEN_FLT x199 = x192 * x45;
	const GEN_FLT x200 = x195 * x95;
	const GEN_FLT x201 = x187 * x52;
	const GEN_FLT x202 = x143 * x192;
	const GEN_FLT x203 = -x136 * (-x200 * x50 - x201 * x95 + 2 * x202) - x96 * (2 * x198 + x199 * x95 + x200 * x46);
	const GEN_FLT x204 = x92 * (x197 * x67 + x99 * (-x123 * (x188 * x95 + x193 * x95 + 2 * x196) + x203));
	const GEN_FLT x205 = x204 * x70;
	const GEN_FLT x206 = x204 * x71 + x68 * (-x204 * x69 + x205);
	const GEN_FLT x207 = x204 * x72 + x206 * x68;
	const GEN_FLT x208 = x106 * x203 + x197 * x61;
	const GEN_FLT x209 = x195 * x23;
	const GEN_FLT x210 = x108 * (x198 + x199 * x23 + x209 * x46) + x109 * (-x201 * x23 + x202 - x209 * x50);
	const GEN_FLT x211 = -x105 * x208 + x210;
	const GEN_FLT x212 =
		x210 - x91 * (x118 * (x114 * x211 -
							  x116 * (x204 * x73 + x204 * x83 + x207 * x68 +
									  x68 * (x204 * x82 + x207 +
											 x68 * (x204 * x81 + x206 + x68 * (-x115 * x204 + x204 * x80 + x205))))) +
					  x119 * x211 + x120 * x204 + x207 * x89 + x208);
	const GEN_FLT x213 = obj_qi * x32;
	const GEN_FLT x214 = obj_qj * x149;
	const GEN_FLT x215 = sensor_x * x214;
	const GEN_FLT x216 = sensor_z * x214;
	const GEN_FLT x217 = -x162 * x214 + x182 + x213 + x215 * x38 + x216 * x39;
	const GEN_FLT x218 = x217 * x37;
	const GEN_FLT x219 = sensor_y * x214;
	const GEN_FLT x220 = -obj_qj * x190;
	const GEN_FLT x221 = sensor_x * (-x214 * x9 + x220) + x15 * x219 + x156 + x194 + x216 * x5;
	const GEN_FLT x222 = x2 * x221;
	const GEN_FLT x223 = obj_qw * x32;
	const GEN_FLT x224 = sensor_z * (-x214 * x33 + x220) + x148 + x215 * x31 + x219 * x30 - x223;
	const GEN_FLT x225 = x224 * x27;
	const GEN_FLT x226 = x218 + x222 * x23 + x225 * x23;
	const GEN_FLT x227 = x217 * x95;
	const GEN_FLT x228 = x224 * x45;
	const GEN_FLT x229 = x221 * x48;
	const GEN_FLT x230 = x221 * x52;
	const GEN_FLT x231 = x143 * x224;
	const GEN_FLT x232 = -x136 * (-x227 * x50 - x230 * x95 + 2 * x231) - x96 * (x227 * x46 + x228 * x95 + 2 * x229);
	const GEN_FLT x233 = x92 * (x226 * x67 + x99 * (-x123 * (2 * x218 + x222 * x95 + x225 * x95) + x232));
	const GEN_FLT x234 = x233 * x70;
	const GEN_FLT x235 = x233 * x71 + x68 * (-x233 * x69 + x234);
	const GEN_FLT x236 = x233 * x72 + x235 * x68;
	const GEN_FLT x237 = x106 * x232 + x226 * x61;
	const GEN_FLT x238 = x217 * x23;
	const GEN_FLT x239 = x108 * (x228 * x23 + x229 + x238 * x46) + x109 * (-x23 * x230 + x231 - x238 * x50);
	const GEN_FLT x240 = -x105 * x237 + x239;
	const GEN_FLT x241 =
		x239 - x91 * (x118 * (x114 * x240 -
							  x116 * (x233 * x73 + x233 * x83 + x236 * x68 +
									  x68 * (x233 * x82 + x236 +
											 x68 * (x233 * x81 + x235 + x68 * (-x115 * x233 + x233 * x80 + x234))))) +
					  x119 * x240 + x120 * x233 + x236 * x89 + x237);
	const GEN_FLT x242 = obj_qk * x149;
	const GEN_FLT x243 = sensor_y * x242;
	const GEN_FLT x244 = sensor_z * x242;
	const GEN_FLT x245 = sensor_x * x242;
	const GEN_FLT x246 = x181 + x213 + x243 * x30 - x244 * x33 + x245 * x31;
	const GEN_FLT x247 = x246 * x27;
	const GEN_FLT x248 = -obj_qk * x190;
	const GEN_FLT x249 = sensor_x * (-x242 * x9 + x248) + x15 * x243 + x160 - x189 + x244 * x5;
	const GEN_FLT x250 = x2 * x249;
	const GEN_FLT x251 = sensor_y * (-x242 * x40 + x248) + x147 + x223 + x244 * x39 + x245 * x38;
	const GEN_FLT x252 = x251 * x37;
	const GEN_FLT x253 = x23 * x247 + x23 * x250 + x252;
	const GEN_FLT x254 = x246 * x45;
	const GEN_FLT x255 = x251 * x95;
	const GEN_FLT x256 = x249 * x48;
	const GEN_FLT x257 = x143 * x246;
	const GEN_FLT x258 = x249 * x52;
	const GEN_FLT x259 = -x136 * (-x255 * x50 + 2 * x257 - x258 * x95) - x96 * (x254 * x95 + x255 * x46 + 2 * x256);
	const GEN_FLT x260 = x92 * (x253 * x67 + x99 * (-x123 * (x247 * x95 + x250 * x95 + 2 * x252) + x259));
	const GEN_FLT x261 = x260 * x70;
	const GEN_FLT x262 = x260 * x71 + x68 * (-x260 * x69 + x261);
	const GEN_FLT x263 = x260 * x72 + x262 * x68;
	const GEN_FLT x264 = x106 * x259 + x253 * x61;
	const GEN_FLT x265 = x23 * x251;
	const GEN_FLT x266 = x108 * (x23 * x254 + x256 + x265 * x46) + x109 * (-x23 * x258 + x257 - x265 * x50);
	const GEN_FLT x267 = -x105 * x264 + x266;
	const GEN_FLT x268 =
		x266 - x91 * (x118 * (x114 * x267 -
							  x116 * (x260 * x73 + x260 * x83 + x263 * x68 +
									  x68 * (x260 * x82 + x263 +
											 x68 * (x260 * x81 + x262 + x68 * (-x115 * x260 + x260 * x80 + x261))))) +
					  x119 * x267 + x120 * x260 + x263 * x89 + x264);
	*(out++) = x121 * x122 + x121;
	*(out++) = x122 * x134 + x134;
	*(out++) = x122 * x146 + x146;
	*(out++) = x122 * x180 + x180;
	*(out++) = x122 * x212 + x212;
	*(out++) = x122 * x241 + x241;
	*(out++) = x122 * x268 + x268;
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
	const GEN_FLT x53 = -lh_pz - x2 * x50 - x25 * (1 - x52) - x38 * x49;
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
	const GEN_FLT x108 = 2 / x17;
	const GEN_FLT x109 = obj_qw * x108;
	const GEN_FLT x110 = sensor_y * x109;
	const GEN_FLT x111 = sensor_z * x109;
	const GEN_FLT x112 = sensor_x * x109;
	const GEN_FLT x113 = x106 - x107 + x110 * x12 - x111 * x24 + x112 * x22;
	const GEN_FLT x114 = x113 * x8;
	const GEN_FLT x115 = obj_qi * x35;
	const GEN_FLT x116 = obj_qk * x23;
	const GEN_FLT x117 = sensor_y * x36;
	const GEN_FLT x118 = -x109 * x117 + x111 * x34 + x112 * x33 - x115 + x116;
	const GEN_FLT x119 = x118 * x8;
	const GEN_FLT x120 = obj_qj * x35;
	const GEN_FLT x121 = obj_qk * x19;
	const GEN_FLT x122 = x110 * x42 + x111 * x41 - x112 * x16 + x120 - x121;
	const GEN_FLT x123 = x122 * x40;
	const GEN_FLT x124 = x114 * x9 + x119 * x30 + x123;
	const GEN_FLT x125 = x124 * x60;
	const GEN_FLT x126 = x122 * x8;
	const GEN_FLT x127 = x113 * x99;
	const GEN_FLT x128 = -x119 * x49 - x126 * x2 + x127;
	const GEN_FLT x129 = x128 * x57;
	const GEN_FLT x130 = x118 * x67;
	const GEN_FLT x131 = x114 * x64 + x126 * x62 + x130;
	const GEN_FLT x132 = x131 * x73;
	const GEN_FLT x133 = x128 * x76;
	const GEN_FLT x134 = x74 * x9;
	const GEN_FLT x135 = x118 * x74;
	const GEN_FLT x136 = -x103 * (-x122 * x77 + 2 * x127 - x135 * x49);
	const GEN_FLT x137 = x80 * (x131 * x81 + x86 * (x136 - x83 * (x113 * x134 + 2 * x123 + x135 * x30)));
	const GEN_FLT x138 = obj_qj * x19;
	const GEN_FLT x139 = obj_qk * x35;
	const GEN_FLT x140 = obj_qi * x108;
	const GEN_FLT x141 = sensor_x * x140;
	const GEN_FLT x142 = sensor_z * x140;
	const GEN_FLT x143 = sensor_y * x140;
	const GEN_FLT x144 = x138 + x139 - x141 * x16 + x142 * x41 + x143 * x42;
	const GEN_FLT x145 = x144 * x40;
	const GEN_FLT x146 = obj_qw * x19;
	const GEN_FLT x147 = 4 * x17;
	const GEN_FLT x148 = -obj_qi * x147;
	const GEN_FLT x149 = sensor_z * (-x140 * x24 + x148) + x116 + x12 * x143 + x141 * x22 + x146;
	const GEN_FLT x150 = x149 * x8;
	const GEN_FLT x151 = obj_qw * x35;
	const GEN_FLT x152 = sensor_y * (-x140 * x36 + x148) + x107 + x141 * x33 + x142 * x34 - x151;
	const GEN_FLT x153 = x152 * x8;
	const GEN_FLT x154 = x145 + x150 * x9 + x153 * x30;
	const GEN_FLT x155 = x154 * x60;
	const GEN_FLT x156 = x144 * x8;
	const GEN_FLT x157 = x149 * x99;
	const GEN_FLT x158 = -x153 * x49 - x156 * x2 + x157;
	const GEN_FLT x159 = x158 * x57;
	const GEN_FLT x160 = x152 * x67;
	const GEN_FLT x161 = x150 * x64 + x156 * x62 + x160;
	const GEN_FLT x162 = x161 * x73;
	const GEN_FLT x163 = x158 * x76;
	const GEN_FLT x164 = x152 * x74;
	const GEN_FLT x165 = -x103 * (-x144 * x77 + 2 * x157 - x164 * x49);
	const GEN_FLT x166 = x80 * (x161 * x81 + x86 * (x165 - x83 * (x134 * x149 + 2 * x145 + x164 * x30)));
	const GEN_FLT x167 = obj_qi * x23;
	const GEN_FLT x168 = obj_qj * x108;
	const GEN_FLT x169 = sensor_x * x168;
	const GEN_FLT x170 = sensor_z * x168;
	const GEN_FLT x171 = -x117 * x168 + x139 + x167 + x169 * x33 + x170 * x34;
	const GEN_FLT x172 = x171 * x8;
	const GEN_FLT x173 = obj_qw * x23;
	const GEN_FLT x174 = sensor_y * x168;
	const GEN_FLT x175 = -obj_qj * x147;
	const GEN_FLT x176 = sensor_z * (-x168 * x24 + x175) + x12 * x174 + x121 + x169 * x22 - x173;
	const GEN_FLT x177 = x176 * x8;
	const GEN_FLT x178 = sensor_x * (-x16 * x168 + x175) + x106 + x151 + x170 * x41 + x174 * x42;
	const GEN_FLT x179 = x178 * x40;
	const GEN_FLT x180 = x172 * x30 + x177 * x9 + x179;
	const GEN_FLT x181 = x180 * x60;
	const GEN_FLT x182 = x178 * x8;
	const GEN_FLT x183 = x176 * x99;
	const GEN_FLT x184 = -x172 * x49 - x182 * x2 + x183;
	const GEN_FLT x185 = x184 * x57;
	const GEN_FLT x186 = x171 * x67;
	const GEN_FLT x187 = x177 * x64 + x182 * x62 + x186;
	const GEN_FLT x188 = x187 * x73;
	const GEN_FLT x189 = x184 * x76;
	const GEN_FLT x190 = x171 * x74;
	const GEN_FLT x191 = -x103 * (-x178 * x77 + 2 * x183 - x190 * x49);
	const GEN_FLT x192 = x80 * (x187 * x81 + x86 * (x191 - x83 * (x134 * x176 + 2 * x179 + x190 * x30)));
	const GEN_FLT x193 = obj_qk * x108;
	const GEN_FLT x194 = sensor_y * x193;
	const GEN_FLT x195 = sensor_z * x193;
	const GEN_FLT x196 = sensor_x * x193;
	const GEN_FLT x197 = x12 * x194 + x138 + x167 - x195 * x24 + x196 * x22;
	const GEN_FLT x198 = x197 * x8;
	const GEN_FLT x199 = -obj_qk * x147;
	const GEN_FLT x200 = sensor_y * (-x193 * x36 + x199) + x120 + x173 + x195 * x34 + x196 * x33;
	const GEN_FLT x201 = x200 * x8;
	const GEN_FLT x202 = sensor_x * (-x16 * x193 + x199) + x115 - x146 + x194 * x42 + x195 * x41;
	const GEN_FLT x203 = x202 * x40;
	const GEN_FLT x204 = x198 * x9 + x201 * x30 + x203;
	const GEN_FLT x205 = x204 * x60;
	const GEN_FLT x206 = x197 * x99;
	const GEN_FLT x207 = x202 * x8;
	const GEN_FLT x208 = -x2 * x207 - x201 * x49 + x206;
	const GEN_FLT x209 = x208 * x57;
	const GEN_FLT x210 = x200 * x67;
	const GEN_FLT x211 = x198 * x64 + x207 * x62 + x210;
	const GEN_FLT x212 = x211 * x73;
	const GEN_FLT x213 = x208 * x76;
	const GEN_FLT x214 = x200 * x74;
	const GEN_FLT x215 = -x103 * (-x202 * x77 + 2 * x206 - x214 * x49);
	const GEN_FLT x216 = x80 * (x211 * x81 + x86 * (x215 - x83 * (x134 * x197 + 2 * x203 + x214 * x30)));
	const GEN_FLT x217 = curve_1 * x88;
	const GEN_FLT x218 = x73 * x8;
	const GEN_FLT x219 = x76 * x8;
	const GEN_FLT x220 = pow(-x46 * x72 * tilt_1 * tilt_1 + 1, -1.0 / 2.0);
	const GEN_FLT x221 = tilt_1 / sqrt(x71);
	const GEN_FLT x222 = x69 * x8;
	const GEN_FLT x223 = tilt_1 * x45 / pow(x71, 3.0 / 2.0);
	const GEN_FLT x224 = -x2 * x219 + x218 * x62 - x220 * (x221 * x40 + x223 * (-x222 * x62 + x85));
	const GEN_FLT x225 = gibMag_1 * sin(gibPhase_1 - phase_1 + x78 - asin(x221 * x45) + 1.5707963267948966);
	const GEN_FLT x226 = x221 * x8;
	const GEN_FLT x227 = (1.0 / 2.0) * x69;
	const GEN_FLT x228 = -x219 * x49 - x220 * (x223 * (-x227 * (-x66 * x74 + 2) + x96) + x226 * x30) + x94;
	const GEN_FLT x229 = x102 + x218 * x64 - x220 * (x223 * (x104 - x222 * x64) + x226 * x9);
	const GEN_FLT x230 =
		x132 + x133 - x220 * (x124 * x221 + x223 * (x136 - x227 * (x101 * x113 + x122 * x75 + 2 * x130)));
	const GEN_FLT x231 =
		x162 + x163 - x220 * (x154 * x221 + x223 * (x165 - x227 * (x101 * x149 + x144 * x75 + 2 * x160)));
	const GEN_FLT x232 =
		x188 + x189 - x220 * (x180 * x221 + x223 * (x191 - x227 * (x101 * x176 + x178 * x75 + 2 * x186)));
	const GEN_FLT x233 =
		x212 + x213 - x220 * (x204 * x221 + x223 * (x215 - x227 * (x101 * x197 + x202 * x75 + 2 * x210)));
	*(out++) = x59 - x61 + x79 * (x73 * x75 - x76 * x77) - x87 + x89 * (-x59 + x61 + x87);
	*(out++) = x79 * (-x76 * x93 + 2 * x94) + x89 * (-x90 + x92 + x97) + x90 - x92 - x97;
	*(out++) = -x100 - x105 + x79 * (x101 * x73 + 2 * x102) + x89 * (x100 + x105 + x98) - x98;
	*(out++) = -x125 - x129 - x137 + x79 * (2 * x132 + 2 * x133) + x89 * (x125 + x129 + x137);
	*(out++) = -x155 - x159 - x166 + x79 * (2 * x162 + 2 * x163) + x89 * (x155 + x159 + x166);
	*(out++) = -x181 - x185 - x192 + x79 * (2 * x188 + 2 * x189) + x89 * (x181 + x185 + x192);
	*(out++) = -x205 - x209 - x216 + x79 * (2 * x212 + 2 * x213) + x89 * (x205 + x209 + x216);
	*(out++) = x217 * (-x57 * x77 + 2 * x61) + x224 * x225 + x224;
	*(out++) = x217 * (x30 * x60 * x74 - x57 * x93) + x225 * x228 + x228;
	*(out++) = x217 * (2 * x100 + x134 * x60) + x225 * x229 + x229;
	*(out++) = x217 * (2 * x125 + 2 * x129) + x225 * x230 + x230;
	*(out++) = x217 * (2 * x155 + 2 * x159) + x225 * x231 + x231;
	*(out++) = x217 * (2 * x181 + 2 * x185) + x225 * x232 + x232;
	*(out++) = x217 * (2 * x205 + 2 * x209) + x225 * x233 + x233;
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
	const GEN_FLT x52 = -lh_pz - x2 * x49 - x25 * (1 - x51) - x38 * x48;
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
	const GEN_FLT x99 = 2 / x17;
	const GEN_FLT x100 = obj_qw * x99;
	const GEN_FLT x101 = sensor_y * x100;
	const GEN_FLT x102 = sensor_z * x100;
	const GEN_FLT x103 = sensor_x * x100;
	const GEN_FLT x104 = x101 * x12 - x102 * x24 + x103 * x22 + x97 - x98;
	const GEN_FLT x105 = x104 * x8;
	const GEN_FLT x106 = obj_qi * x35;
	const GEN_FLT x107 = obj_qk * x23;
	const GEN_FLT x108 = sensor_y * x36;
	const GEN_FLT x109 = -x100 * x108 + x102 * x34 + x103 * x33 - x106 + x107;
	const GEN_FLT x110 = x109 * x8;
	const GEN_FLT x111 = obj_qj * x35;
	const GEN_FLT x112 = obj_qk * x19;
	const GEN_FLT x113 = x101 * x42 + x102 * x41 - x103 * x16 + x111 - x112;
	const GEN_FLT x114 = x113 * x40;
	const GEN_FLT x115 = x59 * (x105 * x9 + x110 * x30 + x114);
	const GEN_FLT x116 = x113 * x8;
	const GEN_FLT x117 = x104 * x92;
	const GEN_FLT x118 = -x110 * x48 - x116 * x2 + x117;
	const GEN_FLT x119 = x118 * x56;
	const GEN_FLT x120 = x105 * x63 + x109 * x65 + x116 * x61;
	const GEN_FLT x121 = x70 * x9;
	const GEN_FLT x122 = x109 * x70;
	const GEN_FLT x123 = x76 * (x120 * x77 + x81 * (-x79 * (x104 * x121 + 2 * x114 + x122 * x30) -
													x95 * (-x113 * x72 + 2 * x117 - x122 * x48)));
	const GEN_FLT x124 = obj_qj * x19;
	const GEN_FLT x125 = obj_qk * x35;
	const GEN_FLT x126 = obj_qi * x99;
	const GEN_FLT x127 = sensor_x * x126;
	const GEN_FLT x128 = sensor_z * x126;
	const GEN_FLT x129 = sensor_y * x126;
	const GEN_FLT x130 = x124 + x125 - x127 * x16 + x128 * x41 + x129 * x42;
	const GEN_FLT x131 = x130 * x40;
	const GEN_FLT x132 = obj_qw * x19;
	const GEN_FLT x133 = 4 * x17;
	const GEN_FLT x134 = -obj_qi * x133;
	const GEN_FLT x135 = sensor_z * (-x126 * x24 + x134) + x107 + x12 * x129 + x127 * x22 + x132;
	const GEN_FLT x136 = x135 * x8;
	const GEN_FLT x137 = obj_qw * x35;
	const GEN_FLT x138 = sensor_y * (-x126 * x36 + x134) + x127 * x33 + x128 * x34 - x137 + x98;
	const GEN_FLT x139 = x138 * x8;
	const GEN_FLT x140 = x59 * (x131 + x136 * x9 + x139 * x30);
	const GEN_FLT x141 = x130 * x8;
	const GEN_FLT x142 = x135 * x92;
	const GEN_FLT x143 = -x139 * x48 - x141 * x2 + x142;
	const GEN_FLT x144 = x143 * x56;
	const GEN_FLT x145 = x136 * x63 + x138 * x65 + x141 * x61;
	const GEN_FLT x146 = x138 * x70;
	const GEN_FLT x147 = x76 * (x145 * x77 + x81 * (-x79 * (x121 * x135 + 2 * x131 + x146 * x30) -
													x95 * (-x130 * x72 + 2 * x142 - x146 * x48)));
	const GEN_FLT x148 = obj_qi * x23;
	const GEN_FLT x149 = obj_qj * x99;
	const GEN_FLT x150 = sensor_x * x149;
	const GEN_FLT x151 = sensor_z * x149;
	const GEN_FLT x152 = -x108 * x149 + x125 + x148 + x150 * x33 + x151 * x34;
	const GEN_FLT x153 = x152 * x8;
	const GEN_FLT x154 = obj_qw * x23;
	const GEN_FLT x155 = sensor_y * x149;
	const GEN_FLT x156 = -obj_qj * x133;
	const GEN_FLT x157 = sensor_z * (-x149 * x24 + x156) + x112 + x12 * x155 + x150 * x22 - x154;
	const GEN_FLT x158 = x157 * x8;
	const GEN_FLT x159 = sensor_x * (-x149 * x16 + x156) + x137 + x151 * x41 + x155 * x42 + x97;
	const GEN_FLT x160 = x159 * x40;
	const GEN_FLT x161 = x59 * (x153 * x30 + x158 * x9 + x160);
	const GEN_FLT x162 = x159 * x8;
	const GEN_FLT x163 = x157 * x92;
	const GEN_FLT x164 = -x153 * x48 - x162 * x2 + x163;
	const GEN_FLT x165 = x164 * x56;
	const GEN_FLT x166 = x152 * x65 + x158 * x63 + x162 * x61;
	const GEN_FLT x167 = x152 * x70;
	const GEN_FLT x168 = x76 * (x166 * x77 + x81 * (-x79 * (x121 * x157 + 2 * x160 + x167 * x30) -
													x95 * (-x159 * x72 + 2 * x163 - x167 * x48)));
	const GEN_FLT x169 = obj_qk * x99;
	const GEN_FLT x170 = sensor_y * x169;
	const GEN_FLT x171 = sensor_z * x169;
	const GEN_FLT x172 = sensor_x * x169;
	const GEN_FLT x173 = x12 * x170 + x124 + x148 - x171 * x24 + x172 * x22;
	const GEN_FLT x174 = x173 * x8;
	const GEN_FLT x175 = -obj_qk * x133;
	const GEN_FLT x176 = sensor_y * (-x169 * x36 + x175) + x111 + x154 + x171 * x34 + x172 * x33;
	const GEN_FLT x177 = x176 * x8;
	const GEN_FLT x178 = sensor_x * (-x16 * x169 + x175) + x106 - x132 + x170 * x42 + x171 * x41;
	const GEN_FLT x179 = x178 * x40;
	const GEN_FLT x180 = x59 * (x174 * x9 + x177 * x30 + x179);
	const GEN_FLT x181 = x173 * x92;
	const GEN_FLT x182 = x178 * x8;
	const GEN_FLT x183 = -x177 * x48 + x181 - x182 * x2;
	const GEN_FLT x184 = x183 * x56;
	const GEN_FLT x185 = x174 * x63 + x176 * x65 + x182 * x61;
	const GEN_FLT x186 = x176 * x70;
	const GEN_FLT x187 = x76 * (x185 * x77 + x81 * (-x79 * (x121 * x173 + 2 * x179 + x186 * x30) -
													x95 * (-x178 * x72 + 2 * x181 - x186 * x48)));
	*(out++) = x58 - x60 + x75 * (x61 * x71 - x72 * x74) - x82 + x83 * (-x58 + x60 + x82);
	*(out++) = x75 * (-x48 * x70 * x74 + x65 * x88) + x83 * (-x84 + x86 + x90) + x84 - x86 - x90;
	*(out++) = x75 * (x63 * x71 + x92 * x94) + x83 * (x91 + x93 + x96) - x91 - x93 - x96;
	*(out++) = -x115 - x119 - x123 + x75 * (x118 * x94 + x120 * x88) + x83 * (x115 + x119 + x123);
	*(out++) = -x140 - x144 - x147 + x75 * (x143 * x94 + x145 * x88) + x83 * (x140 + x144 + x147);
	*(out++) = -x161 - x165 - x168 + x75 * (x164 * x94 + x166 * x88) + x83 * (x161 + x165 + x168);
	*(out++) = -x180 - x184 - x187 + x75 * (x183 * x94 + x185 * x88) + x83 * (x180 + x184 + x187);
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
	const GEN_FLT x54 = -lh_pz - x2 * x51 - x25 * (1 - x53) - x38 * x50;
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
	const GEN_FLT x93 = 2 / x17;
	const GEN_FLT x94 = obj_qw * x93;
	const GEN_FLT x95 = sensor_y * x94;
	const GEN_FLT x96 = sensor_z * x94;
	const GEN_FLT x97 = sensor_x * x94;
	const GEN_FLT x98 = x12 * x95 + x22 * x97 - x24 * x96 + x91 - x92;
	const GEN_FLT x99 = x26 * x98;
	const GEN_FLT x100 = obj_qi * x36;
	const GEN_FLT x101 = obj_qk * x23;
	const GEN_FLT x102 = -x100 + x101 - x16 * x95 + x34 * x97 + x35 * x96;
	const GEN_FLT x103 = x102 * x26;
	const GEN_FLT x104 = obj_qj * x36;
	const GEN_FLT x105 = obj_qk * x19;
	const GEN_FLT x106 = sensor_x * x43;
	const GEN_FLT x107 = x104 - x105 - x106 * x94 + x41 * x96 + x42 * x95;
	const GEN_FLT x108 = x103 * x31 + x107 * x40 + x9 * x99;
	const GEN_FLT x109 = x107 * x26;
	const GEN_FLT x110 = x87 * x98;
	const GEN_FLT x111 = -x103 * x50 - x109 * x2 + x110;
	const GEN_FLT x112 = x102 * x66;
	const GEN_FLT x113 = x107 * x8;
	const GEN_FLT x114 = x64 * x8;
	const GEN_FLT x115 = x111 * x73 + x71 * (x109 * x62 + x112 + x64 * x99) -
						 x75 * (x108 * x76 + x79 * (-x85 * (2 * x112 + x113 * x62 + x114 * x98) -
													x89 * (-x102 * x82 + 2 * x110 - x113 * x2)));
	const GEN_FLT x116 = obj_qj * x19;
	const GEN_FLT x117 = obj_qk * x36;
	const GEN_FLT x118 = obj_qi * x93;
	const GEN_FLT x119 = sensor_z * x118;
	const GEN_FLT x120 = sensor_y * x118;
	const GEN_FLT x121 = -x106 * x118 + x116 + x117 + x119 * x41 + x120 * x42;
	const GEN_FLT x122 = obj_qw * x19;
	const GEN_FLT x123 = sensor_x * x118;
	const GEN_FLT x124 = 4 * x17;
	const GEN_FLT x125 = -obj_qi * x124;
	const GEN_FLT x126 = sensor_z * (-x118 * x24 + x125) + x101 + x12 * x120 + x122 + x123 * x22;
	const GEN_FLT x127 = x126 * x26;
	const GEN_FLT x128 = obj_qw * x36;
	const GEN_FLT x129 = sensor_y * (-x118 * x16 + x125) + x119 * x35 + x123 * x34 - x128 + x92;
	const GEN_FLT x130 = x129 * x26;
	const GEN_FLT x131 = x121 * x40 + x127 * x9 + x130 * x31;
	const GEN_FLT x132 = x121 * x26;
	const GEN_FLT x133 = x126 * x87;
	const GEN_FLT x134 = -x130 * x50 - x132 * x2 + x133;
	const GEN_FLT x135 = x129 * x66;
	const GEN_FLT x136 = x121 * x8;
	const GEN_FLT x137 = x134 * x73 + x71 * (x127 * x64 + x132 * x62 + x135) -
						 x75 * (x131 * x76 + x79 * (-x85 * (x114 * x126 + 2 * x135 + x136 * x62) -
													x89 * (-x129 * x82 + 2 * x133 - x136 * x2)));
	const GEN_FLT x138 = obj_qi * x23;
	const GEN_FLT x139 = obj_qj * x93;
	const GEN_FLT x140 = sensor_x * x139;
	const GEN_FLT x141 = sensor_y * x139;
	const GEN_FLT x142 = sensor_z * x139;
	const GEN_FLT x143 = x117 + x138 + x140 * x34 - x141 * x16 + x142 * x35;
	const GEN_FLT x144 = x143 * x26;
	const GEN_FLT x145 = obj_qw * x23;
	const GEN_FLT x146 = -obj_qj * x124;
	const GEN_FLT x147 = sensor_z * (-x139 * x24 + x146) + x105 + x12 * x141 + x140 * x22 - x145;
	const GEN_FLT x148 = x147 * x26;
	const GEN_FLT x149 = sensor_x * (-x139 * x43 + x146) + x128 + x141 * x42 + x142 * x41 + x91;
	const GEN_FLT x150 = x144 * x31 + x148 * x9 + x149 * x40;
	const GEN_FLT x151 = x149 * x26;
	const GEN_FLT x152 = x147 * x87;
	const GEN_FLT x153 = -x144 * x50 - x151 * x2 + x152;
	const GEN_FLT x154 = x143 * x66;
	const GEN_FLT x155 = x149 * x8;
	const GEN_FLT x156 = x153 * x73 + x71 * (x148 * x64 + x151 * x62 + x154) -
						 x75 * (x150 * x76 + x79 * (-x85 * (x114 * x147 + 2 * x154 + x155 * x62) -
													x89 * (-x143 * x82 + 2 * x152 - x155 * x2)));
	const GEN_FLT x157 = obj_qk * x93;
	const GEN_FLT x158 = sensor_y * x157;
	const GEN_FLT x159 = sensor_z * x157;
	const GEN_FLT x160 = sensor_x * x157;
	const GEN_FLT x161 = x116 + x12 * x158 + x138 - x159 * x24 + x160 * x22;
	const GEN_FLT x162 = x161 * x26;
	const GEN_FLT x163 = -obj_qk * x124;
	const GEN_FLT x164 = sensor_y * (-x157 * x16 + x163) + x104 + x145 + x159 * x35 + x160 * x34;
	const GEN_FLT x165 = x164 * x26;
	const GEN_FLT x166 = sensor_x * (-x157 * x43 + x163) + x100 - x122 + x158 * x42 + x159 * x41;
	const GEN_FLT x167 = x162 * x9 + x165 * x31 + x166 * x40;
	const GEN_FLT x168 = x161 * x87;
	const GEN_FLT x169 = x166 * x26;
	const GEN_FLT x170 = -x165 * x50 + x168 - x169 * x2;
	const GEN_FLT x171 = x164 * x66;
	const GEN_FLT x172 = x166 * x8;
	const GEN_FLT x173 = x170 * x73 + x71 * (x162 * x64 + x169 * x62 + x171) -
						 x75 * (x167 * x76 + x79 * (-x85 * (x114 * x161 + 2 * x171 + x172 * x62) -
													x89 * (-x164 * x82 + 2 * x168 - x172 * x2)));
	*(out++) = x61 * (-x2 * x58 * x8 + x40 * x60) + x80 * x81 + x80;
	*(out++) = x61 * (x31 * x83 - x58 * x82) + x81 * x86 + x86;
	*(out++) = x61 * (x83 * x9 + x87 * x88) + x81 * x90 + x90;
	*(out++) = x115 * x81 + x115 + x61 * (x108 * x60 + x111 * x88);
	*(out++) = x137 * x81 + x137 + x61 * (x131 * x60 + x134 * x88);
	*(out++) = x156 * x81 + x156 + x61 * (x150 * x60 + x153 * x88);
	*(out++) = x173 * x81 + x173 + x61 * (x167 * x60 + x170 * x88);
}
