#pragma once
#include "common.h"
/** Applying function <function reproject_gen2 at 0x7f35a4af8a70> */
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 + (-1 * x1);
	const GEN_FLT x3 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x4 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 0;
	const GEN_FLT x5 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   : 1e-10;
	const GEN_FLT x6 = cos(x5);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 1;
	const GEN_FLT x9 = sin(x5);
	const GEN_FLT x10 = x8 * x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x4 * x7 * x11;
	const GEN_FLT x13 = x9 * x11;
	const GEN_FLT x14 = x8 * x7;
	const GEN_FLT x15 = x4 * x14;
	const GEN_FLT x16 =
		obj_pz + ((((x4 * x4) * x7) + x6) * sensor_z) + ((x10 + x12) * sensor_y) + (((-1 * x13) + x15) * sensor_x);
	const GEN_FLT x17 = sin(x0);
	const GEN_FLT x18 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 1;
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 0;
	const GEN_FLT x21 = x2 * x3;
	const GEN_FLT x22 = x20 * x21;
	const GEN_FLT x23 = x4 * x9;
	const GEN_FLT x24 = x14 * x11;
	const GEN_FLT x25 =
		obj_py + (((-1 * x10) + x12) * sensor_z) + (((x7 * (x11 * x11)) + x6) * sensor_y) + ((x23 + x24) * sensor_x);
	const GEN_FLT x26 = x20 * x17;
	const GEN_FLT x27 = x21 * x18;
	const GEN_FLT x28 =
		obj_px + ((x13 + x15) * sensor_z) + (((-1 * x23) + x24) * sensor_y) + ((((x8 * x8) * x7) + x6) * sensor_x);
	const GEN_FLT x29 = lh_pz + (x16 * ((x2 * (x3 * x3)) + x1)) + ((x19 + x22) * x25) + (((-1 * x26) + x27) * x28);
	const GEN_FLT x30 = x3 * x17;
	const GEN_FLT x31 = x2 * x20 * x18;
	const GEN_FLT x32 = lh_px + ((x26 + x27) * x16) + (((-1 * x30) + x31) * x25) + (x28 * ((x2 * (x18 * x18)) + x1));
	const GEN_FLT x33 = atan2(-1 * x29, x32);
	const GEN_FLT x34 = 0.523598775598299 + tilt_0;
	const GEN_FLT x35 = cos(x34);
	const GEN_FLT x36 = lh_py + (((-1 * x19) + x22) * x16) + (x25 * ((x2 * (x20 * x20)) + x1)) + ((x30 + x31) * x28);
	const GEN_FLT x37 = (x29 * x29) + (x32 * x32);
	const GEN_FLT x38 = x36 * pow(((x36 * x36) + x37), -1.0 / 2.0);
	const GEN_FLT x39 = asin(pow(x35, -1) * x38);
	const GEN_FLT x40 = 0.0028679863 + (x39 * (-8.0108022e-06 + (-8.0108022e-06 * x39)));
	const GEN_FLT x41 = 5.3685255e-06 + (x40 * x39);
	const GEN_FLT x42 = 0.0076069798 + (x41 * x39);
	const GEN_FLT x43 = x36 * pow(x37, -1.0 / 2.0);
	const GEN_FLT x44 = x43 * tan(x34);
	const GEN_FLT x45 = curve_0 + (sin(ogeeMag_0 + (-1 * asin(x44)) + x33) * ogeePhase_0);
	const GEN_FLT x46 =
		x33 +
		(-1 * asin((x42 * x45 * (x39 * x39) *
					pow(((-1 * x45 * sin(x34) *
						  ((x42 * x39) +
						   (x39 * ((x39 * ((x39 * ((x39 * (-8.0108022e-06 + (-1.60216044e-05 * x39))) + x40)) + x41)) +
								   x42)))) +
						 x35),
						-1)) +
				   x44));
	const GEN_FLT x47 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x48 = -1 * x43 * tan(x47);
	const GEN_FLT x49 = curve_1 + (sin(ogeeMag_1 + (-1 * asin(x48)) + x33) * ogeePhase_1);
	const GEN_FLT x50 = cos(x47);
	const GEN_FLT x51 = asin(pow(x50, -1) * x38);
	const GEN_FLT x52 = 0.0028679863 + (x51 * (-8.0108022e-06 + (-8.0108022e-06 * x51)));
	const GEN_FLT x53 = 5.3685255e-06 + (x52 * x51);
	const GEN_FLT x54 = 0.0076069798 + (x53 * x51);
	const GEN_FLT x55 =
		(-1 * asin((x54 * (x51 * x51) * x49 *
					pow(((x49 * sin(x47) *
						  ((x54 * x51) +
						   (x51 * ((x51 * ((x51 * ((x51 * (-8.0108022e-06 + (-1.60216044e-05 * x51))) + x52)) + x53)) +
								   x54)))) +
						 x50),
						-1)) +
				   x48)) +
		x33;
	out[0] = -1.5707963267949 + (sin(gibPhase_0 + x46) * gibMag_0) + (-1 * phase_0) + x46;
	out[1] = -1.5707963267949 + (-1 * phase_1) + (sin(gibPhase_1 + x55) * gibMag_1) + x55;
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x5 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x20 * x11;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x15 * x11;
	const GEN_FLT x24 = x23 * x16;
	const GEN_FLT x25 =
		obj_pz + (((x15 * x12) + x14) * sensor_z) + ((x18 + x21) * sensor_y) + (((-1 * x22) + x24) * sensor_x);
	const GEN_FLT x26 = x1 * x4;
	const GEN_FLT x27 = x2 * x8;
	const GEN_FLT x28 = (-1 * x26) + x27;
	const GEN_FLT x29 = x19 * x19;
	const GEN_FLT x30 = x11 * x17;
	const GEN_FLT x31 = x20 * x16;
	const GEN_FLT x32 =
		obj_py + (((-1 * x18) + x21) * sensor_z) + (((x29 * x15) + x14) * sensor_y) + ((x30 + x31) * sensor_x);
	const GEN_FLT x33 = x5 * x5;
	const GEN_FLT x34 = (x7 * x33) + x6;
	const GEN_FLT x35 = x16 * x16;
	const GEN_FLT x36 =
		obj_px + ((x22 + x24) * sensor_z) + (((-1 * x30) + x31) * sensor_y) + (((x35 * x15) + x14) * sensor_x);
	const GEN_FLT x37 = lh_px + (x25 * x10) + (x32 * x28) + (x34 * x36);
	const GEN_FLT x38 = pow(x37, -1);
	const GEN_FLT x39 = (-1 * x3) + x9;
	const GEN_FLT x40 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x41 = x15 * x16;
	const GEN_FLT x42 = 2 * x41;
	const GEN_FLT x43 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x44 = x43 * x17;
	const GEN_FLT x45 = -1 * x44;
	const GEN_FLT x46 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x47 = x46 * x17;
	const GEN_FLT x48 = x14 * x11;
	const GEN_FLT x49 = x43 * x48;
	const GEN_FLT x50 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x51 = (x50 * x41) + (x43 * x19 * x18) + (x40 * x20);
	const GEN_FLT x52 = x50 * x17;
	const GEN_FLT x53 = x43 * x14;
	const GEN_FLT x54 = x53 * x19;
	const GEN_FLT x55 = x43 * x11;
	const GEN_FLT x56 = (x40 * x23) + (x55 * x18) + (x41 * x46);
	const GEN_FLT x57 = (((x40 * x42) + x45 + (x44 * x35)) * sensor_x) + (((-1 * x47) + (-1 * x49) + x51) * sensor_y) +
						((x52 + x54 + x56) * sensor_z);
	const GEN_FLT x58 = 1 + x57;
	const GEN_FLT x59 = x4 * x4;
	const GEN_FLT x60 = (x7 * x59) + x6;
	const GEN_FLT x61 = x40 * x17;
	const GEN_FLT x62 = x53 * x16;
	const GEN_FLT x63 = (x50 * x23) + (x46 * x20) + (x55 * x22);
	const GEN_FLT x64 = 2 * x23;
	const GEN_FLT x65 = (((-1 * x52) + (-1 * x54) + x56) * sensor_x) + ((x61 + x62 + x63) * sensor_y) +
						((x45 + (x64 * x46) + (x44 * x12)) * sensor_z);
	const GEN_FLT x66 = x60 * x65;
	const GEN_FLT x67 = 2 * x20;
	const GEN_FLT x68 = ((x47 + x49 + x51) * sensor_x) + ((x45 + (x67 * x50) + (x44 * x29)) * sensor_y) +
						(((-1 * x61) + (-1 * x62) + x63) * sensor_z);
	const GEN_FLT x69 = x1 * x5;
	const GEN_FLT x70 = x2 * x7;
	const GEN_FLT x71 = x4 * x70;
	const GEN_FLT x72 = x69 + x71;
	const GEN_FLT x73 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x74 = x6 * x73;
	const GEN_FLT x75 = x2 * x74;
	const GEN_FLT x76 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x77 = x1 * x76;
	const GEN_FLT x78 = x1 * x73;
	const GEN_FLT x79 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x80 = x4 * x7;
	const GEN_FLT x81 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x82 = (x4 * x5 * x78) + (x80 * x79) + (x8 * x81);
	const GEN_FLT x83 = -1 * x78;
	const GEN_FLT x84 = x5 * x74;
	const GEN_FLT x85 = x1 * x79;
	const GEN_FLT x86 = x2 * x78;
	const GEN_FLT x87 = (x4 * x86) + (x81 * x70) + (x80 * x76);
	const GEN_FLT x88 = (x36 * ((-1 * x75) + (-1 * x77) + x82)) + (x25 * (x83 + (x78 * x59) + (2 * x80 * x81))) +
						(x32 * (x84 + x85 + x87));
	const GEN_FLT x89 = (x72 * x68) + x88;
	const GEN_FLT x90 = (x58 * x39) + x66 + x89;
	const GEN_FLT x91 = x68 * x28;
	const GEN_FLT x92 = x4 * x74;
	const GEN_FLT x93 = x1 * x81;
	const GEN_FLT x94 = (x70 * x79) + (x8 * x76) + (x5 * x86);
	const GEN_FLT x95 = (x36 * (x83 + (x78 * x33) + (2 * x8 * x79))) + (x25 * (x75 + x77 + x82)) +
						(x32 * ((-1 * x92) + (-1 * x93) + x94));
	const GEN_FLT x96 = (x65 * x10) + x95;
	const GEN_FLT x97 = x91 + (x58 * x34) + x96;
	const GEN_FLT x98 = x37 * x37;
	const GEN_FLT x99 = lh_pz + (x60 * x25) + (x72 * x32) + (x36 * x39);
	const GEN_FLT x100 = x99 * pow(x98, -1);
	const GEN_FLT x101 = (x99 * x99) + x98;
	const GEN_FLT x102 = pow(x101, -1);
	const GEN_FLT x103 = x98 * x102;
	const GEN_FLT x104 = x103 * ((-1 * x90 * x38) + (x97 * x100));
	const GEN_FLT x105 = x26 + x27;
	const GEN_FLT x106 = (-1 * x69) + x71;
	const GEN_FLT x107 = x65 * x106;
	const GEN_FLT x108 = x2 * x2;
	const GEN_FLT x109 = (x7 * x108) + x6;
	const GEN_FLT x110 = (x36 * (x92 + x93 + x94)) + (x32 * (x83 + (x78 * x108) + (2 * x70 * x76))) +
						 (x25 * ((-1 * x84) + (-1 * x85) + x87));
	const GEN_FLT x111 = (x68 * x109) + x110;
	const GEN_FLT x112 = (x58 * x105) + x107 + x111;
	const GEN_FLT x113 = lh_py + (x25 * x106) + (x32 * x109) + (x36 * x105);
	const GEN_FLT x114 = 2 * x113;
	const GEN_FLT x115 = 2 * x37;
	const GEN_FLT x116 = 2 * x99;
	const GEN_FLT x117 = (x97 * x115) + (x90 * x116);
	const GEN_FLT x118 = (x112 * x114) + x117;
	const GEN_FLT x119 = 0.523598775598299 + tilt_0;
	const GEN_FLT x120 = cos(x119);
	const GEN_FLT x121 = pow(x120, -1);
	const GEN_FLT x122 = x113 * x113;
	const GEN_FLT x123 = x122 + x101;
	const GEN_FLT x124 = 1.0 / 2.0 * x113;
	const GEN_FLT x125 = x124 * pow(x123, -3.0 / 2.0);
	const GEN_FLT x126 = x121 * x125;
	const GEN_FLT x127 = pow(x123, -1.0 / 2.0);
	const GEN_FLT x128 = x121 * x127;
	const GEN_FLT x129 = pow(x123, -1) * x122;
	const GEN_FLT x130 = pow((1 + (-1 * pow(x120, -2) * x129)), -1.0 / 2.0);
	const GEN_FLT x131 = ((-1 * x118 * x126) + (x112 * x128)) * x130;
	const GEN_FLT x132 = pow(x101, -1.0 / 2.0);
	const GEN_FLT x133 = tan(x119);
	const GEN_FLT x134 = x133 * x132;
	const GEN_FLT x135 = x113 * x134;
	const GEN_FLT x136 = atan2(-1 * x99, x37);
	const GEN_FLT x137 = ogeeMag_0 + (-1 * asin(x135)) + x136;
	const GEN_FLT x138 = curve_0 + (sin(x137) * ogeePhase_0);
	const GEN_FLT x139 = asin(x113 * x128);
	const GEN_FLT x140 = 8.0108022e-06 * x139;
	const GEN_FLT x141 = -8.0108022e-06 + (-1 * x140);
	const GEN_FLT x142 = 0.0028679863 + (x139 * x141);
	const GEN_FLT x143 = 5.3685255e-06 + (x139 * x142);
	const GEN_FLT x144 = 0.0076069798 + (x139 * x143);
	const GEN_FLT x145 = x139 * x144;
	const GEN_FLT x146 = -8.0108022e-06 + (-1.60216044e-05 * x139);
	const GEN_FLT x147 = (x139 * x146) + x142;
	const GEN_FLT x148 = (x139 * x147) + x143;
	const GEN_FLT x149 = (x139 * x148) + x144;
	const GEN_FLT x150 = x145 + (x139 * x149);
	const GEN_FLT x151 = sin(x119);
	const GEN_FLT x152 = x138 * x151;
	const GEN_FLT x153 = (-1 * x150 * x152) + x120;
	const GEN_FLT x154 = pow(x153, -1);
	const GEN_FLT x155 = 2 * x138 * x145 * x154;
	const GEN_FLT x156 = pow(x101, -3.0 / 2.0) * x124;
	const GEN_FLT x157 = x133 * x156;
	const GEN_FLT x158 = (-1 * x117 * x157) + (x112 * x134);
	const GEN_FLT x159 = x102 * x122;
	const GEN_FLT x160 = pow((1 + (-1 * (x133 * x133) * x159)), -1.0 / 2.0);
	const GEN_FLT x161 = cos(x137) * ogeePhase_0;
	const GEN_FLT x162 = x161 * (x104 + (-1 * x160 * x158));
	const GEN_FLT x163 = x139 * x139;
	const GEN_FLT x164 = x163 * x144 * x154;
	const GEN_FLT x165 = x131 * x141;
	const GEN_FLT x166 = 2.40324066e-05 * x139;
	const GEN_FLT x167 = (x131 * x142) + (x139 * ((-1 * x131 * x140) + x165));
	const GEN_FLT x168 = (x131 * x143) + (x167 * x139);
	const GEN_FLT x169 = x150 * x151;
	const GEN_FLT x170 = x163 * x138;
	const GEN_FLT x171 = x170 * x144 * pow(x153, -2);
	const GEN_FLT x172 = x170 * x154;
	const GEN_FLT x173 = (x172 * x144) + x135;
	const GEN_FLT x174 = pow((1 + (-1 * (x173 * x173))), -1.0 / 2.0);
	const GEN_FLT x175 =
		x104 +
		(-1 * x174 *
		 ((x131 * x155) + (x164 * x162) +
		  (-1 * x171 *
		   ((-1 * x152 *
			 ((x131 * x149) + (x168 * x139) +
			  (x139 * ((x131 * x148) +
					   (x139 * ((x131 * x147) + (x139 * (x165 + (-1 * x166 * x131) + (x131 * x146))) + x167)) + x168)) +
			  (x131 * x144))) +
			(-1 * x169 * x162))) +
		  x158 + (x168 * x172)));
	const GEN_FLT x176 = cos(gibPhase_0 + (-1 * asin(x173)) + x136) * gibMag_0;
	const GEN_FLT x177 = x57 * x39;
	const GEN_FLT x178 = 1 + x68;
	const GEN_FLT x179 = x177 + x66 + (x72 * x178) + x88;
	const GEN_FLT x180 = x57 * x34;
	const GEN_FLT x181 = x180 + (x28 * x178) + x96;
	const GEN_FLT x182 = x103 * ((-1 * x38 * x179) + (x100 * x181));
	const GEN_FLT x183 = x57 * x105;
	const GEN_FLT x184 = x183 + x110 + x107 + (x109 * x178);
	const GEN_FLT x185 = (x115 * x181) + (x116 * x179);
	const GEN_FLT x186 = (x114 * x184) + x185;
	const GEN_FLT x187 = ((-1 * x126 * x186) + (x128 * x184)) * x130;
	const GEN_FLT x188 = x187 * x141;
	const GEN_FLT x189 = (x187 * x142) + (x139 * ((-1 * x187 * x140) + x188));
	const GEN_FLT x190 = (x187 * x143) + (x189 * x139);
	const GEN_FLT x191 = (-1 * x185 * x157) + (x184 * x134);
	const GEN_FLT x192 = x182 + (-1 * x160 * x191);
	const GEN_FLT x193 = x161 * x169;
	const GEN_FLT x194 = x161 * x164;
	const GEN_FLT x195 =
		x182 +
		(-1 * x174 *
		 ((x187 * x155) +
		  (-1 * x171 *
		   ((-1 * x152 *
			 ((x187 * x149) +
			  (x139 * ((x187 * x148) +
					   (x139 * ((x187 * x147) + (x139 * (x188 + (-1 * x166 * x187) + (x187 * x146))) + x189)) + x190)) +
			  (x187 * x144) + (x190 * x139))) +
			(-1 * x192 * x193))) +
		  (x172 * x190) + (x192 * x194) + x191));
	const GEN_FLT x196 = 1 + x65;
	const GEN_FLT x197 = x177 + (x60 * x196) + x89;
	const GEN_FLT x198 = x180 + x91 + (x10 * x196) + x95;
	const GEN_FLT x199 = x103 * ((-1 * x38 * x197) + (x100 * x198));
	const GEN_FLT x200 = x183 + (x106 * x196) + x111;
	const GEN_FLT x201 = (x115 * x198) + (x116 * x197);
	const GEN_FLT x202 = (x200 * x114) + x201;
	const GEN_FLT x203 = (-1 * x202 * x126) + (x200 * x128);
	const GEN_FLT x204 = x203 * x130;
	const GEN_FLT x205 = x204 * x141;
	const GEN_FLT x206 = (x204 * x142) + (x139 * ((-1 * x204 * x140) + x205));
	const GEN_FLT x207 = (x204 * x143) + (x206 * x139);
	const GEN_FLT x208 = x130 * x144;
	const GEN_FLT x209 = (-1 * x201 * x157) + (x200 * x134);
	const GEN_FLT x210 = x199 + (-1 * x209 * x160);
	const GEN_FLT x211 =
		x199 +
		(-1 * x174 *
		 ((x204 * x155) +
		  (-1 * x171 *
		   ((-1 * x152 *
			 ((x204 * x149) + (x207 * x139) +
			  (x139 * ((x204 * x148) +
					   (x139 * ((x204 * x147) + (x139 * (x205 + (-1 * x204 * x166) + (x204 * x146))) + x206)) + x207)) +
			  (x203 * x208))) +
			(-1 * x210 * x193))) +
		  (x207 * x172) + (x210 * x194) + x209));
	const GEN_FLT x212 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x213 = x17 * x212;
	const GEN_FLT x214 = -1 * x213;
	const GEN_FLT x215 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qi *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x216 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qk *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x217 = x17 * x216;
	const GEN_FLT x218 = x48 * x212;
	const GEN_FLT x219 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qj *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x220 = x19 * x16;
	const GEN_FLT x221 = (x20 * x215) + (x41 * x219) + (x213 * x220);
	const GEN_FLT x222 = x17 * x219;
	const GEN_FLT x223 = x14 * x19;
	const GEN_FLT x224 = x212 * x223;
	const GEN_FLT x225 = x11 * x16;
	const GEN_FLT x226 = (x23 * x215) + (x41 * x216) + (x213 * x225);
	const GEN_FLT x227 = ((x214 + (x42 * x215) + (x35 * x213)) * sensor_x) +
						 (((-1 * x217) + x221 + (-1 * x218)) * sensor_y) + ((x222 + x224 + x226) * sensor_z);
	const GEN_FLT x228 = x17 * x215;
	const GEN_FLT x229 = x14 * x16;
	const GEN_FLT x230 = x212 * x229;
	const GEN_FLT x231 = x11 * x19;
	const GEN_FLT x232 = (x23 * x219) + (x20 * x216) + (x213 * x231);
	const GEN_FLT x233 = ((x217 + x221 + x218) * sensor_x) + (((x67 * x219) + x214 + (x29 * x213)) * sensor_y) +
						 (((-1 * x228) + (-1 * x230) + x232) * sensor_z);
	const GEN_FLT x234 = (((-1 * x222) + (-1 * x224) + x226) * sensor_x) + ((x228 + x230 + x232) * sensor_y) +
						 ((x214 + (x64 * x216) + (x12 * x213)) * sensor_z);
	const GEN_FLT x235 = (x39 * x227) + (x72 * x233) + (x60 * x234) + x88;
	const GEN_FLT x236 = (x34 * x227) + (x28 * x233) + (x10 * x234) + x95;
	const GEN_FLT x237 = x103 * ((-1 * x38 * x235) + (x236 * x100));
	const GEN_FLT x238 = (x227 * x105) + (x233 * x109) + x110 + (x234 * x106);
	const GEN_FLT x239 = (x236 * x115) + (x235 * x116);
	const GEN_FLT x240 = (x238 * x114) + x239;
	const GEN_FLT x241 = x238 * x127;
	const GEN_FLT x242 = (-1 * x240 * x126) + (x241 * x121);
	const GEN_FLT x243 = x242 * x130;
	const GEN_FLT x244 = x243 * x141;
	const GEN_FLT x245 = (x243 * x142) + (x139 * ((-1 * x243 * x140) + x244));
	const GEN_FLT x246 = (x243 * x143) + (x245 * x139);
	const GEN_FLT x247 = (-1 * x239 * x157) + (x238 * x134);
	const GEN_FLT x248 = x237 + (-1 * x247 * x160);
	const GEN_FLT x249 =
		x237 +
		(-1 * x174 *
		 ((x243 * x155) +
		  (-1 * x171 *
		   ((-1 * x152 *
			 ((x243 * x149) +
			  (x139 * ((x243 * x148) +
					   (x139 * ((x243 * x147) + (x139 * (x244 + (-1 * x243 * x166) + (x243 * x146))) + x245)) + x246)) +
			  (x208 * x242) + (x246 * x139))) +
			(-1 * x248 * x193))) +
		  (x246 * x172) + (x248 * x194) + x247));
	const GEN_FLT x250 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x251 = x17 * x250;
	const GEN_FLT x252 = -1 * x251;
	const GEN_FLT x253 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qi *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x254 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qk *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x255 = x17 * x254;
	const GEN_FLT x256 = x48 * x250;
	const GEN_FLT x257 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qj *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x258 = (x20 * x253) + (x41 * x257) + (x251 * x220);
	const GEN_FLT x259 = x17 * x257;
	const GEN_FLT x260 = x250 * x223;
	const GEN_FLT x261 = (x23 * x253) + (x251 * x225) + (x41 * x254);
	const GEN_FLT x262 = ((x252 + (x35 * x251) + (x42 * x253)) * sensor_x) +
						 (((-1 * x255) + (-1 * x256) + x258) * sensor_y) + ((x259 + x260 + x261) * sensor_z);
	const GEN_FLT x263 = x17 * x253;
	const GEN_FLT x264 = x250 * x229;
	const GEN_FLT x265 = (x20 * x254) + (x231 * x251) + (x23 * x257);
	const GEN_FLT x266 = ((x252 + (x67 * x257) + (x29 * x251)) * sensor_y) + ((x255 + x256 + x258) * sensor_x) +
						 (((-1 * x263) + (-1 * x264) + x265) * sensor_z);
	const GEN_FLT x267 = (((-1 * x259) + (-1 * x260) + x261) * sensor_x) + ((x263 + x264 + x265) * sensor_y) +
						 ((x252 + (x64 * x254) + (x12 * x251)) * sensor_z);
	const GEN_FLT x268 = (x39 * x262) + (x72 * x266) + (x60 * x267) + x88;
	const GEN_FLT x269 = (x34 * x262) + (x28 * x266) + (x10 * x267) + x95;
	const GEN_FLT x270 = x103 * ((-1 * x38 * x268) + (x269 * x100));
	const GEN_FLT x271 = (x262 * x105) + (x266 * x109) + (x267 * x106) + x110;
	const GEN_FLT x272 = (x269 * x115) + (x268 * x116);
	const GEN_FLT x273 = (x271 * x114) + x272;
	const GEN_FLT x274 = ((-1 * x273 * x126) + (x271 * x128)) * x130;
	const GEN_FLT x275 = x274 * x141;
	const GEN_FLT x276 = (x274 * x142) + (x139 * ((-1 * x274 * x140) + x275));
	const GEN_FLT x277 = (x274 * x143) + (x276 * x139);
	const GEN_FLT x278 = (-1 * x272 * x157) + (x271 * x134);
	const GEN_FLT x279 = x270 + (-1 * x278 * x160);
	const GEN_FLT x280 =
		x270 +
		(-1 * x174 *
		 ((x274 * x155) +
		  (-1 * x171 *
		   ((-1 * x152 *
			 ((x274 * x149) + (x277 * x139) +
			  (x139 * ((x274 * x148) +
					   (x139 * ((x274 * x147) + (x139 * (x275 + (-1 * x274 * x166) + (x274 * x146))) + x276)) + x277)) +
			  (x274 * x144))) +
			(-1 * x279 * x193))) +
		  (x277 * x172) + (x279 * x194) + x278));
	const GEN_FLT x281 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x282 = x17 * x281;
	const GEN_FLT x283 = -1 * x282;
	const GEN_FLT x284 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qi *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x285 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qk *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x286 = x17 * x285;
	const GEN_FLT x287 = x48 * x281;
	const GEN_FLT x288 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qj *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x289 = (x20 * x284) + (x41 * x288) + (x282 * x220);
	const GEN_FLT x290 = x17 * x288;
	const GEN_FLT x291 = x281 * x223;
	const GEN_FLT x292 = x11 * x282;
	const GEN_FLT x293 = (x23 * x284) + (x16 * x292) + (x41 * x285);
	const GEN_FLT x294 = ((x283 + (x42 * x284) + (x35 * x282)) * sensor_x) +
						 (((-1 * x286) + (-1 * x287) + x289) * sensor_y) + ((x290 + x291 + x293) * sensor_z);
	const GEN_FLT x295 = x17 * x284;
	const GEN_FLT x296 = x281 * x229;
	const GEN_FLT x297 = (x23 * x288) + (x20 * x285) + (x19 * x292);
	const GEN_FLT x298 = ((x286 + x287 + x289) * sensor_x) + (((x67 * x288) + (x29 * x282) + x283) * sensor_y) +
						 (((-1 * x295) + (-1 * x296) + x297) * sensor_z);
	const GEN_FLT x299 = (((-1 * x290) + (-1 * x291) + x293) * sensor_x) +
						 ((x283 + (x64 * x285) + (x12 * x282)) * sensor_z) + ((x295 + x296 + x297) * sensor_y);
	const GEN_FLT x300 = (x39 * x294) + (x72 * x298) + (x60 * x299) + x88;
	const GEN_FLT x301 = (x34 * x294) + (x28 * x298) + (x10 * x299) + x95;
	const GEN_FLT x302 = x103 * ((-1 * x38 * x300) + (x301 * x100));
	const GEN_FLT x303 = (x294 * x105) + x110 + (x298 * x109) + (x299 * x106);
	const GEN_FLT x304 = (x301 * x115) + (x300 * x116);
	const GEN_FLT x305 = (x303 * x114) + x304;
	const GEN_FLT x306 = x303 * x127;
	const GEN_FLT x307 = ((-1 * x305 * x126) + (x306 * x121)) * x130;
	const GEN_FLT x308 = (-1 * x304 * x157) + (x303 * x134);
	const GEN_FLT x309 = x302 + (-1 * x308 * x160);
	const GEN_FLT x310 = x307 * x141;
	const GEN_FLT x311 = (x307 * x142) + (x139 * ((-1 * x307 * x140) + x310));
	const GEN_FLT x312 = (x307 * x143) + (x311 * x139);
	const GEN_FLT x313 =
		x302 +
		(-1 * x174 *
		 ((x307 * x155) + (x309 * x194) + x308 +
		  (-1 * x171 *
		   ((-1 * x152 *
			 ((x307 * x149) +
			  (x139 * ((x307 * x148) +
					   (x139 * ((x307 * x147) + (x139 * (x310 + (-1 * x307 * x166) + (x307 * x146))) + x311)) + x312)) +
			  (x307 * x144) + (x312 * x139))) +
			(-1 * x309 * x193))) +
		  (x312 * x172)));
	const GEN_FLT x314 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x315 = cos(x314);
	const GEN_FLT x316 = pow(x315, -1);
	const GEN_FLT x317 = x316 * x127;
	const GEN_FLT x318 = asin(x317 * x113);
	const GEN_FLT x319 = 8.0108022e-06 * x318;
	const GEN_FLT x320 = -8.0108022e-06 + (-1 * x319);
	const GEN_FLT x321 = 0.0028679863 + (x320 * x318);
	const GEN_FLT x322 = 5.3685255e-06 + (x321 * x318);
	const GEN_FLT x323 = 0.0076069798 + (x322 * x318);
	const GEN_FLT x324 = x318 * x318;
	const GEN_FLT x325 = tan(x314);
	const GEN_FLT x326 = x325 * x132;
	const GEN_FLT x327 = -1 * x326 * x113;
	const GEN_FLT x328 = ogeeMag_1 + (-1 * asin(x327)) + x136;
	const GEN_FLT x329 = curve_1 + (sin(x328) * ogeePhase_1);
	const GEN_FLT x330 = x323 * x318;
	const GEN_FLT x331 = -8.0108022e-06 + (-1.60216044e-05 * x318);
	const GEN_FLT x332 = (x331 * x318) + x321;
	const GEN_FLT x333 = (x332 * x318) + x322;
	const GEN_FLT x334 = (x333 * x318) + x323;
	const GEN_FLT x335 = x330 + (x334 * x318);
	const GEN_FLT x336 = sin(x314);
	const GEN_FLT x337 = x336 * x329;
	const GEN_FLT x338 = (x337 * x335) + x315;
	const GEN_FLT x339 = pow(x338, -1);
	const GEN_FLT x340 = x339 * x329;
	const GEN_FLT x341 = x324 * x340;
	const GEN_FLT x342 = (x323 * x341) + x327;
	const GEN_FLT x343 = pow((1 + (-1 * (x342 * x342))), -1.0 / 2.0);
	const GEN_FLT x344 = x325 * x156;
	const GEN_FLT x345 = (x344 * x117) + (-1 * x326 * x112);
	const GEN_FLT x346 = pow((1 + (-1 * (x325 * x325) * x159)), -1.0 / 2.0);
	const GEN_FLT x347 = x104 + (-1 * x346 * x345);
	const GEN_FLT x348 = cos(x328) * ogeePhase_1;
	const GEN_FLT x349 = x323 * x324;
	const GEN_FLT x350 = x339 * x349;
	const GEN_FLT x351 = x350 * x348;
	const GEN_FLT x352 = pow((1 + (-1 * pow(x315, -2) * x129)), -1.0 / 2.0);
	const GEN_FLT x353 = x316 * x125;
	const GEN_FLT x354 = (-1 * x353 * x118) + (x317 * x112);
	const GEN_FLT x355 = x352 * x354;
	const GEN_FLT x356 = 2 * x330 * x340;
	const GEN_FLT x357 = x355 * x320;
	const GEN_FLT x358 = x352 * x321;
	const GEN_FLT x359 = (x318 * ((-1 * x355 * x319) + x357)) + (x354 * x358);
	const GEN_FLT x360 = (x355 * x322) + (x359 * x318);
	const GEN_FLT x361 = x336 * x335;
	const GEN_FLT x362 = x361 * x348;
	const GEN_FLT x363 = x352 * x334;
	const GEN_FLT x364 = x352 * x333;
	const GEN_FLT x365 = 2.40324066e-05 * x318;
	const GEN_FLT x366 = x352 * x331;
	const GEN_FLT x367 = x352 * x332;
	const GEN_FLT x368 = pow(x338, -2) * x329 * x349;
	const GEN_FLT x369 =
		x104 +
		(-1 * x343 *
		 ((x351 * x347) + (x356 * x355) + (x360 * x341) +
		  (-1 * x368 *
		   ((x362 * x347) +
			(x337 *
			 ((x363 * x354) + (x355 * x323) +
			  (x318 * ((x364 * x354) +
					   (x318 * ((x318 * (x357 + (-1 * x365 * x355) + (x366 * x354))) + (x367 * x354) + x359)) + x360)) +
			  (x360 * x318))))) +
		  x345));
	const GEN_FLT x370 = cos(gibPhase_1 + x136 + (-1 * asin(x342))) * gibMag_1;
	const GEN_FLT x371 = (x344 * x185) + (-1 * x326 * x184);
	const GEN_FLT x372 = x348 * (x182 + (-1 * x371 * x346));
	const GEN_FLT x373 = (-1 * x353 * x186) + (x317 * x184);
	const GEN_FLT x374 = x373 * x352;
	const GEN_FLT x375 = x374 * x320;
	const GEN_FLT x376 = (x318 * ((-1 * x374 * x319) + x375)) + (x373 * x358);
	const GEN_FLT x377 = (x374 * x322) + (x376 * x318);
	const GEN_FLT x378 =
		x182 +
		(-1 * x343 *
		 ((x372 * x350) + (x374 * x356) + (x377 * x341) +
		  (-1 * x368 *
		   ((x372 * x361) +
			(x337 *
			 ((x374 * x334) +
			  (x318 * ((x373 * x364) +
					   (x318 * ((x318 * (x375 + (-1 * x374 * x365) + (x373 * x366))) + x376 + (x373 * x367))) + x377)) +
			  (x377 * x318) + (x374 * x323))))) +
		  x371));
	const GEN_FLT x379 = (x201 * x344) + (-1 * x200 * x326);
	const GEN_FLT x380 = x199 + (-1 * x379 * x346);
	const GEN_FLT x381 = (-1 * x202 * x353) + (x200 * x317);
	const GEN_FLT x382 = x352 * x322;
	const GEN_FLT x383 = x381 * x352;
	const GEN_FLT x384 = x352 * x320;
	const GEN_FLT x385 = x384 * x381;
	const GEN_FLT x386 = (x318 * ((-1 * x383 * x319) + x385)) + (x381 * x358);
	const GEN_FLT x387 = (x381 * x382) + (x386 * x318);
	const GEN_FLT x388 = x352 * x323;
	const GEN_FLT x389 =
		x199 +
		(-1 * x343 *
		 ((x351 * x380) + (x387 * x341) + (x383 * x356) +
		  (-1 * x368 *
		   ((x362 * x380) +
			(x337 *
			 ((x318 * ((x364 * x381) +
					   (x318 * ((x318 * (x385 + (-1 * x365 * x383) + (x366 * x381))) + (x367 * x381) + x386)) + x387)) +
			  (x363 * x381) + (x387 * x318) + (x381 * x388))))) +
		  x379));
	const GEN_FLT x390 = (x239 * x344) + (-1 * x238 * x326);
	const GEN_FLT x391 = x237 + (-1 * x390 * x346);
	const GEN_FLT x392 = (-1 * x240 * x353) + (x241 * x316);
	const GEN_FLT x393 = x392 * x352;
	const GEN_FLT x394 = x392 * x384;
	const GEN_FLT x395 = (x318 * ((-1 * x393 * x319) + x394)) + (x392 * x358);
	const GEN_FLT x396 = (x392 * x382) + (x395 * x318);
	const GEN_FLT x397 =
		x237 +
		(-1 * x343 *
		 ((x391 * x351) + (x393 * x356) + (x396 * x341) +
		  (-1 * x368 *
		   ((x391 * x362) +
			(x337 *
			 ((x392 * x363) +
			  (x318 * ((x392 * x364) +
					   (x318 * ((x318 * (x394 + (-1 * x393 * x365) + (x392 * x366))) + (x392 * x367) + x395)) + x396)) +
			  (x396 * x318) + (x392 * x388))))) +
		  x390));
	const GEN_FLT x398 = (x272 * x344) + (-1 * x271 * x326);
	const GEN_FLT x399 = x270 + (-1 * x398 * x346);
	const GEN_FLT x400 = (-1 * x273 * x353) + (x271 * x317);
	const GEN_FLT x401 = x400 * x352;
	const GEN_FLT x402 = x400 * x384;
	const GEN_FLT x403 = (x318 * ((-1 * x401 * x319) + x402)) + (x400 * x358);
	const GEN_FLT x404 = (x400 * x382) + (x403 * x318);
	const GEN_FLT x405 =
		x270 +
		(-1 * x343 *
		 ((x399 * x351) + (x404 * x341) + (x401 * x356) +
		  (-1 * x368 *
		   ((x399 * x362) +
			(x337 *
			 ((x318 * ((x400 * x364) +
					   (x318 * ((x318 * ((-1 * x401 * x365) + x402 + (x400 * x366))) + (x400 * x367) + x403)) + x404)) +
			  (x400 * x363) + (x404 * x318) + (x400 * x388))))) +
		  x398));
	const GEN_FLT x406 = (x304 * x344) + (-1 * x303 * x326);
	const GEN_FLT x407 = x302 + (-1 * x406 * x346);
	const GEN_FLT x408 = (-1 * x353 * x305) + (x306 * x316);
	const GEN_FLT x409 = x408 * x352;
	const GEN_FLT x410 = x409 * x320;
	const GEN_FLT x411 = (x318 * ((-1 * x409 * x319) + x410)) + (x408 * x358);
	const GEN_FLT x412 = (x409 * x322) + (x411 * x318);
	const GEN_FLT x413 =
		x302 +
		(-1 * x343 *
		 ((x407 * x351) + (x412 * x341) +
		  (-1 * x368 *
		   ((x407 * x362) +
			(x337 *
			 ((x318 * ((x408 * x364) +
					   (x318 * ((x318 * (x410 + (-1 * x409 * x365) + (x408 * x366))) + x411 + (x408 * x367))) + x412)) +
			  (x412 * x318) + (x409 * x334) + (x409 * x323))))) +
		  (x409 * x356) + x406));
	out[0] = (x176 * x175) + x175;
	out[1] = (x176 * x195) + x195;
	out[2] = (x211 * x176) + x211;
	out[3] = (x249 * x176) + x249;
	out[4] = (x280 * x176) + x280;
	out[5] = (x313 * x176) + x313;
	out[6] = (x369 * x370) + x369;
	out[7] = (x378 * x370) + x378;
	out[8] = (x389 * x370) + x389;
	out[9] = (x397 * x370) + x397;
	out[10] = (x405 * x370) + x405;
	out[11] = (x413 * x370) + x413;
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x5 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (x15 * x12) + x14;
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x18 = sin(x13);
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x21 * x11;
	const GEN_FLT x23 = x19 + x22;
	const GEN_FLT x24 = x20 * x18;
	const GEN_FLT x25 = x15 * x17;
	const GEN_FLT x26 = x25 * x11;
	const GEN_FLT x27 = (-1 * x24) + x26;
	const GEN_FLT x28 = obj_pz + (x27 * sensor_x) + (x16 * sensor_z) + (x23 * sensor_y);
	const GEN_FLT x29 = x1 * x4;
	const GEN_FLT x30 = x2 * x8;
	const GEN_FLT x31 = (-1 * x29) + x30;
	const GEN_FLT x32 = (-1 * x19) + x22;
	const GEN_FLT x33 = x20 * x20;
	const GEN_FLT x34 = (x33 * x15) + x14;
	const GEN_FLT x35 = x11 * x18;
	const GEN_FLT x36 = x21 * x17;
	const GEN_FLT x37 = x35 + x36;
	const GEN_FLT x38 = obj_py + (x32 * sensor_z) + (x34 * sensor_y) + (x37 * sensor_x);
	const GEN_FLT x39 = x5 * x5;
	const GEN_FLT x40 = (x7 * x39) + x6;
	const GEN_FLT x41 = x24 + x26;
	const GEN_FLT x42 = (-1 * x35) + x36;
	const GEN_FLT x43 = x17 * x17;
	const GEN_FLT x44 = (x43 * x15) + x14;
	const GEN_FLT x45 = obj_px + (x42 * sensor_y) + (x41 * sensor_z) + (x44 * sensor_x);
	const GEN_FLT x46 = lh_px + (x28 * x10) + (x40 * x45) + (x31 * x38);
	const GEN_FLT x47 = pow(x46, -1);
	const GEN_FLT x48 = (-1 * x3) + x9;
	const GEN_FLT x49 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x50 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x51 = x50 * x18;
	const GEN_FLT x52 = -1 * x51;
	const GEN_FLT x53 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x54 = x53 * x18;
	const GEN_FLT x55 = x50 * x14;
	const GEN_FLT x56 = x55 * x11;
	const GEN_FLT x57 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x58 = x50 * x19;
	const GEN_FLT x59 = (x57 * x25) + (x58 * x20) + (x49 * x21);
	const GEN_FLT x60 = x57 * x18;
	const GEN_FLT x61 = x55 * x20;
	const GEN_FLT x62 = x15 * x11;
	const GEN_FLT x63 = (x62 * x49) + (x58 * x11) + (x53 * x25);
	const GEN_FLT x64 = (((2 * x49 * x25) + x52 + (x51 * x43)) * sensor_x) +
						(((-1 * x54) + (-1 * x56) + x59) * sensor_y) + ((x60 + x61 + x63) * sensor_z);
	const GEN_FLT x65 = x44 + x64;
	const GEN_FLT x66 = x1 * x5;
	const GEN_FLT x67 = x2 * x7;
	const GEN_FLT x68 = x4 * x67;
	const GEN_FLT x69 = x66 + x68;
	const GEN_FLT x70 = x49 * x18;
	const GEN_FLT x71 = x55 * x17;
	const GEN_FLT x72 = (x62 * x57) + (x53 * x21) + (x51 * x20 * x11);
	const GEN_FLT x73 = ((x54 + x56 + x59) * sensor_x) + (((-1 * x70) + (-1 * x71) + x72) * sensor_z) +
						((x52 + (2 * x57 * x21) + (x51 * x33)) * sensor_y);
	const GEN_FLT x74 = x37 + x73;
	const GEN_FLT x75 = x4 * x4;
	const GEN_FLT x76 = (x7 * x75) + x6;
	const GEN_FLT x77 = (((-1 * x60) + (-1 * x61) + x63) * sensor_x) + ((x70 + x71 + x72) * sensor_y) +
						((x52 + (2 * x62 * x53) + (x51 * x12)) * sensor_z);
	const GEN_FLT x78 = x27 + x77;
	const GEN_FLT x79 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x80 = x6 * x79;
	const GEN_FLT x81 = x2 * x80;
	const GEN_FLT x82 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x83 = x1 * x82;
	const GEN_FLT x84 = x1 * x79;
	const GEN_FLT x85 = x5 * x84;
	const GEN_FLT x86 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x87 = x4 * x7;
	const GEN_FLT x88 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x89 = (x4 * x85) + (x86 * x87) + (x8 * x88);
	const GEN_FLT x90 = x5 * x80;
	const GEN_FLT x91 = x1 * x86;
	const GEN_FLT x92 = (x2 * x4 * x84) + (x88 * x67) + (x82 * x87);
	const GEN_FLT x93 = -1 * x84;
	const GEN_FLT x94 = (x45 * ((-1 * x81) + (-1 * x83) + x89)) + (x38 * (x90 + x91 + x92)) +
						(x28 * (x93 + (x84 * x75) + (2 * x88 * x87)));
	const GEN_FLT x95 = (x65 * x48) + (x74 * x69) + (x78 * x76) + x94;
	const GEN_FLT x96 = x4 * x80;
	const GEN_FLT x97 = x1 * x88;
	const GEN_FLT x98 = (x86 * x67) + (x8 * x82) + (x2 * x85);
	const GEN_FLT x99 = (x45 * (x93 + (x84 * x39) + (2 * x8 * x86))) + (x38 * ((-1 * x96) + (-1 * x97) + x98)) +
						(x28 * (x81 + x83 + x89));
	const GEN_FLT x100 = (x65 * x40) + (x74 * x31) + (x78 * x10) + x99;
	const GEN_FLT x101 = x46 * x46;
	const GEN_FLT x102 = lh_pz + (x76 * x28) + (x69 * x38) + (x45 * x48);
	const GEN_FLT x103 = pow(x101, -1) * x102;
	const GEN_FLT x104 = (x102 * x102) + x101;
	const GEN_FLT x105 = pow(x104, -1);
	const GEN_FLT x106 = x101 * x105;
	const GEN_FLT x107 = x106 * ((-1 * x95 * x47) + (x100 * x103));
	const GEN_FLT x108 = 0.523598775598299 + tilt_0;
	const GEN_FLT x109 = cos(x108);
	const GEN_FLT x110 = (-1 * x66) + x68;
	const GEN_FLT x111 = x2 * x2;
	const GEN_FLT x112 = (x7 * x111) + x6;
	const GEN_FLT x113 = x29 + x30;
	const GEN_FLT x114 = lh_py + (x28 * x110) + (x45 * x113) + (x38 * x112);
	const GEN_FLT x115 = x114 * x114;
	const GEN_FLT x116 = x115 + x104;
	const GEN_FLT x117 = x115 * pow(x116, -1);
	const GEN_FLT x118 = pow((1 + (-1 * pow(x109, -2) * x117)), -1.0 / 2.0);
	const GEN_FLT x119 = pow(x109, -1);
	const GEN_FLT x120 = (x45 * (x96 + x97 + x98)) + (x38 * (x93 + (x84 * x111) + (2 * x82 * x67))) +
						 (x28 * ((-1 * x90) + (-1 * x91) + x92));
	const GEN_FLT x121 = (x65 * x113) + (x74 * x112) + (x78 * x110) + x120;
	const GEN_FLT x122 = 2 * x114;
	const GEN_FLT x123 = 2 * x46;
	const GEN_FLT x124 = 2 * x102;
	const GEN_FLT x125 = (x100 * x123) + (x95 * x124);
	const GEN_FLT x126 = 1.0 / 2.0 * x114;
	const GEN_FLT x127 = pow(x116, -3.0 / 2.0) * x126;
	const GEN_FLT x128 = x127 * ((x122 * x121) + x125);
	const GEN_FLT x129 = pow(x116, -1.0 / 2.0);
	const GEN_FLT x130 = x121 * x129;
	const GEN_FLT x131 = ((-1 * x119 * x128) + (x119 * x130)) * x118;
	const GEN_FLT x132 = x114 * x129;
	const GEN_FLT x133 = asin(x119 * x132);
	const GEN_FLT x134 = 8.0108022e-06 * x133;
	const GEN_FLT x135 = -8.0108022e-06 + (-1 * x134);
	const GEN_FLT x136 = 0.0028679863 + (x133 * x135);
	const GEN_FLT x137 = 5.3685255e-06 + (x133 * x136);
	const GEN_FLT x138 = 0.0076069798 + (x133 * x137);
	const GEN_FLT x139 = x133 * x138;
	const GEN_FLT x140 = tan(x108);
	const GEN_FLT x141 = pow(x104, -1.0 / 2.0);
	const GEN_FLT x142 = x114 * x141;
	const GEN_FLT x143 = x140 * x142;
	const GEN_FLT x144 = atan2(-1 * x102, x46);
	const GEN_FLT x145 = ogeeMag_0 + (-1 * asin(x143)) + x144;
	const GEN_FLT x146 = curve_0 + (sin(x145) * ogeePhase_0);
	const GEN_FLT x147 = -8.0108022e-06 + (-1.60216044e-05 * x133);
	const GEN_FLT x148 = (x133 * x147) + x136;
	const GEN_FLT x149 = (x133 * x148) + x137;
	const GEN_FLT x150 = (x133 * x149) + x138;
	const GEN_FLT x151 = x139 + (x133 * x150);
	const GEN_FLT x152 = sin(x108);
	const GEN_FLT x153 = x146 * x152;
	const GEN_FLT x154 = (-1 * x151 * x153) + x109;
	const GEN_FLT x155 = pow(x154, -1);
	const GEN_FLT x156 = x146 * x155;
	const GEN_FLT x157 = 2 * x139 * x156;
	const GEN_FLT x158 = x131 * x135;
	const GEN_FLT x159 = 2.40324066e-05 * x133;
	const GEN_FLT x160 = (x131 * x136) + (x133 * ((-1 * x134 * x131) + x158));
	const GEN_FLT x161 = (x131 * x137) + (x160 * x133);
	const GEN_FLT x162 = x105 * x115;
	const GEN_FLT x163 = pow((1 + (-1 * x162 * (x140 * x140))), -1.0 / 2.0);
	const GEN_FLT x164 = pow(x104, -3.0 / 2.0) * x126;
	const GEN_FLT x165 = x125 * x164;
	const GEN_FLT x166 = x121 * x141;
	const GEN_FLT x167 = (-1 * x165 * x140) + (x166 * x140);
	const GEN_FLT x168 = x107 + (-1 * x167 * x163);
	const GEN_FLT x169 = cos(x145) * ogeePhase_0;
	const GEN_FLT x170 = x151 * x152;
	const GEN_FLT x171 = x169 * x170;
	const GEN_FLT x172 = x133 * x133;
	const GEN_FLT x173 = x172 * x138;
	const GEN_FLT x174 = x173 * x146 * pow(x154, -2);
	const GEN_FLT x175 = x172 * x156;
	const GEN_FLT x176 = x173 * x155;
	const GEN_FLT x177 = x169 * x176;
	const GEN_FLT x178 = (x175 * x138) + x143;
	const GEN_FLT x179 = pow((1 + (-1 * (x178 * x178))), -1.0 / 2.0);
	const GEN_FLT x180 =
		x107 +
		(-1 * x179 *
		 ((x131 * x157) +
		  (-1 * x174 *
		   ((-1 * x153 *
			 ((x133 * ((x131 * x149) +
					   (x133 * ((x131 * x148) + (x133 * (x158 + (-1 * x131 * x159) + (x131 * x147))) + x160)) + x161)) +
			  (x131 * x150) + (x131 * x138) + (x161 * x133))) +
			(-1 * x168 * x171))) +
		  (x161 * x175) + (x168 * x177) + x167));
	const GEN_FLT x181 = cos(gibPhase_0 + (-1 * asin(x178)) + x144) * gibMag_0;
	const GEN_FLT x182 = x42 + x64;
	const GEN_FLT x183 = x34 + x73;
	const GEN_FLT x184 = x23 + x77;
	const GEN_FLT x185 = (x48 * x182) + (x69 * x183) + (x76 * x184) + x94;
	const GEN_FLT x186 = (x40 * x182) + (x31 * x183) + (x10 * x184) + x99;
	const GEN_FLT x187 = x106 * ((-1 * x47 * x185) + (x103 * x186));
	const GEN_FLT x188 = (x113 * x182) + (x112 * x183) + (x110 * x184) + x120;
	const GEN_FLT x189 = (x123 * x186) + (x124 * x185);
	const GEN_FLT x190 = x127 * ((x122 * x188) + x189);
	const GEN_FLT x191 = x129 * x188;
	const GEN_FLT x192 = ((-1 * x119 * x190) + (x119 * x191)) * x118;
	const GEN_FLT x193 = x192 * x135;
	const GEN_FLT x194 = (x192 * x136) + (x133 * ((-1 * x192 * x134) + x193));
	const GEN_FLT x195 = (x192 * x137) + (x194 * x133);
	const GEN_FLT x196 = x164 * x189;
	const GEN_FLT x197 = x188 * x141;
	const GEN_FLT x198 = (-1 * x196 * x140) + (x197 * x140);
	const GEN_FLT x199 = x187 + (-1 * x163 * x198);
	const GEN_FLT x200 =
		x187 +
		(-1 * x179 *
		 ((x192 * x157) +
		  (-1 * x174 *
		   ((-1 * x153 *
			 ((x133 * ((x192 * x149) +
					   (x133 * ((x192 * x148) + (x133 * (x193 + (-1 * x192 * x159) + (x192 * x147))) + x194)) + x195)) +
			  (x192 * x150) + (x192 * x138) + (x195 * x133))) +
			(-1 * x171 * x199))) +
		  (x175 * x195) + (x177 * x199) + x198));
	const GEN_FLT x201 = x41 + x64;
	const GEN_FLT x202 = x32 + x73;
	const GEN_FLT x203 = x16 + x77;
	const GEN_FLT x204 = (x48 * x201) + (x69 * x202) + (x76 * x203) + x94;
	const GEN_FLT x205 = (x40 * x201) + (x31 * x202) + x99 + (x10 * x203);
	const GEN_FLT x206 = x106 * ((-1 * x47 * x204) + (x205 * x103));
	const GEN_FLT x207 = (x202 * x112) + (x201 * x113) + (x203 * x110) + x120;
	const GEN_FLT x208 = (x205 * x123) + (x204 * x124);
	const GEN_FLT x209 = x127 * ((x207 * x122) + x208);
	const GEN_FLT x210 = x207 * x129;
	const GEN_FLT x211 = ((-1 * x209 * x119) + (x210 * x119)) * x118;
	const GEN_FLT x212 = x211 * x135;
	const GEN_FLT x213 = (x211 * x136) + (x133 * ((-1 * x211 * x134) + x212));
	const GEN_FLT x214 = (x211 * x137) + (x213 * x133);
	const GEN_FLT x215 = x208 * x164;
	const GEN_FLT x216 = x207 * x141;
	const GEN_FLT x217 = (-1 * x215 * x140) + (x216 * x140);
	const GEN_FLT x218 = x169 * (x206 + (-1 * x217 * x163));
	const GEN_FLT x219 =
		x206 +
		(-1 * x179 *
		 ((x211 * x157) +
		  (-1 * x174 *
		   ((-1 * x153 *
			 ((x133 * ((x211 * x149) +
					   (x133 * ((x211 * x148) + (x133 * (x212 + (-1 * x211 * x159) + (x211 * x147))) + x213)) + x214)) +
			  (x211 * x150) + (x211 * x138) + (x214 * x133))) +
			(-1 * x218 * x170))) +
		  (x218 * x176) + (x214 * x175) + x217));
	const GEN_FLT x220 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x221 = tan(x220);
	const GEN_FLT x222 = pow((1 + (-1 * (x221 * x221) * x162)), -1.0 / 2.0);
	const GEN_FLT x223 = (x221 * x165) + (-1 * x221 * x166);
	const GEN_FLT x224 = x107 + (-1 * x223 * x222);
	const GEN_FLT x225 = -1 * x221 * x142;
	const GEN_FLT x226 = ogeeMag_1 + (-1 * asin(x225)) + x144;
	const GEN_FLT x227 = cos(x226) * ogeePhase_1;
	const GEN_FLT x228 = cos(x220);
	const GEN_FLT x229 = pow(x228, -1);
	const GEN_FLT x230 = asin(x229 * x132);
	const GEN_FLT x231 = 8.0108022e-06 * x230;
	const GEN_FLT x232 = -8.0108022e-06 + (-1 * x231);
	const GEN_FLT x233 = 0.0028679863 + (x230 * x232);
	const GEN_FLT x234 = 5.3685255e-06 + (x230 * x233);
	const GEN_FLT x235 = 0.0076069798 + (x230 * x234);
	const GEN_FLT x236 = x230 * x235;
	const GEN_FLT x237 = -8.0108022e-06 + (-1.60216044e-05 * x230);
	const GEN_FLT x238 = (x230 * x237) + x233;
	const GEN_FLT x239 = (x230 * x238) + x234;
	const GEN_FLT x240 = (x230 * x239) + x235;
	const GEN_FLT x241 = x236 + (x230 * x240);
	const GEN_FLT x242 = curve_1 + (sin(x226) * ogeePhase_1);
	const GEN_FLT x243 = sin(x220);
	const GEN_FLT x244 = x242 * x243;
	const GEN_FLT x245 = (x241 * x244) + x228;
	const GEN_FLT x246 = pow(x245, -1);
	const GEN_FLT x247 = x230 * x230;
	const GEN_FLT x248 = x235 * x247;
	const GEN_FLT x249 = x246 * x248;
	const GEN_FLT x250 = x227 * x249;
	const GEN_FLT x251 = pow((1 + (-1 * pow(x228, -2) * x117)), -1.0 / 2.0);
	const GEN_FLT x252 = (-1 * x229 * x128) + (x229 * x130);
	const GEN_FLT x253 = x252 * x251;
	const GEN_FLT x254 = x232 * x251;
	const GEN_FLT x255 = x252 * x254;
	const GEN_FLT x256 = (x230 * ((-1 * x231 * x253) + x255)) + (x233 * x253);
	const GEN_FLT x257 = (x234 * x253) + (x230 * x256);
	const GEN_FLT x258 = x242 * x246;
	const GEN_FLT x259 = x258 * x247;
	const GEN_FLT x260 = 2 * x236 * x258;
	const GEN_FLT x261 = x241 * x243;
	const GEN_FLT x262 = x261 * x227;
	const GEN_FLT x263 = x239 * x251;
	const GEN_FLT x264 = 2.40324066e-05 * x230;
	const GEN_FLT x265 = x242 * pow(x245, -2) * x248;
	const GEN_FLT x266 = (x235 * x259) + x225;
	const GEN_FLT x267 = pow((1 + (-1 * (x266 * x266))), -1.0 / 2.0);
	const GEN_FLT x268 =
		x107 +
		(-1 * x267 *
		 ((x250 * x224) + (x257 * x259) + (x260 * x253) +
		  (-1 * x265 *
		   ((x262 * x224) +
			(x244 *
			 ((x253 * x240) +
			  (x230 * ((x263 * x252) +
					   (x230 * ((x230 * (x255 + (-1 * x264 * x253) + (x237 * x253))) + (x238 * x253) + x256)) + x257)) +
			  (x230 * x257) + (x235 * x253))))) +
		  x223));
	const GEN_FLT x269 = cos(gibPhase_1 + (-1 * asin(x266)) + x144) * gibMag_1;
	const GEN_FLT x270 = (x221 * x196) + (-1 * x221 * x197);
	const GEN_FLT x271 = x187 + (-1 * x270 * x222);
	const GEN_FLT x272 = (-1 * x229 * x190) + (x229 * x191);
	const GEN_FLT x273 = x272 * x251;
	const GEN_FLT x274 = x272 * x254;
	const GEN_FLT x275 = (x230 * ((-1 * x231 * x273) + x274)) + (x233 * x273);
	const GEN_FLT x276 = (x234 * x273) + (x230 * x275);
	const GEN_FLT x277 =
		x187 +
		(-1 * x267 *
		 ((x271 * x250) +
		  (-1 * x265 *
		   ((x271 * x262) +
			(x244 *
			 ((x273 * x240) +
			  (x230 * ((x272 * x263) +
					   (x230 * ((x230 * ((-1 * x273 * x264) + x274 + (x237 * x273))) + (x238 * x273) + x275)) + x276)) +
			  (x230 * x276) + (x235 * x273))))) +
		  (x276 * x259) + (x273 * x260) + x270));
	const GEN_FLT x278 = (x215 * x221) + (-1 * x216 * x221);
	const GEN_FLT x279 = x227 * (x206 + (-1 * x278 * x222));
	const GEN_FLT x280 = (-1 * x209 * x229) + (x210 * x229);
	const GEN_FLT x281 = x280 * x251;
	const GEN_FLT x282 = x280 * x254;
	const GEN_FLT x283 = (x230 * ((-1 * x231 * x281) + x282)) + (x233 * x281);
	const GEN_FLT x284 = (x234 * x281) + (x230 * x283);
	const GEN_FLT x285 =
		x206 +
		(-1 * x267 *
		 ((x279 * x249) + (x284 * x259) + (x260 * x281) +
		  (-1 * x265 *
		   ((x279 * x261) +
			(x244 *
			 ((x281 * x240) +
			  (x230 * ((x263 * x280) +
					   (x230 * ((x230 * (x282 + (-1 * x264 * x281) + (x237 * x281))) + (x238 * x281) + x283)) + x284)) +
			  (x230 * x284) + (x235 * x281))))) +
		  x278));
	out[0] = (x181 * x180) + x180;
	out[1] = (x200 * x181) + x200;
	out[2] = (x219 * x181) + x219;
	out[3] = (x268 * x269) + x268;
	out[4] = (x277 * x269) + x277;
	out[5] = (x269 * x285) + x285;
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x20 * x11;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x15 * x11;
	const GEN_FLT x24 = x23 * x16;
	const GEN_FLT x25 =
		obj_pz + (((x15 * x12) + x14) * sensor_z) + ((x18 + x21) * sensor_y) + (((-1 * x22) + x24) * sensor_x);
	const GEN_FLT x26 = x1 * x4;
	const GEN_FLT x27 = x2 * x8;
	const GEN_FLT x28 = (-1 * x26) + x27;
	const GEN_FLT x29 = x19 * x19;
	const GEN_FLT x30 = x11 * x17;
	const GEN_FLT x31 = x20 * x16;
	const GEN_FLT x32 =
		obj_py + (((-1 * x18) + x21) * sensor_z) + (((x29 * x15) + x14) * sensor_y) + ((x30 + x31) * sensor_x);
	const GEN_FLT x33 = x7 * x7;
	const GEN_FLT x34 = (x6 * x33) + x5;
	const GEN_FLT x35 = x16 * x16;
	const GEN_FLT x36 =
		obj_px + ((x22 + x24) * sensor_z) + (((-1 * x30) + x31) * sensor_y) + (((x35 * x15) + x14) * sensor_x);
	const GEN_FLT x37 = lh_px + (x25 * x10) + (x32 * x28) + (x34 * x36);
	const GEN_FLT x38 = pow(x37, -1);
	const GEN_FLT x39 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x40 = x2 * x5;
	const GEN_FLT x41 = x40 * x39;
	const GEN_FLT x42 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x43 = x1 * x42;
	const GEN_FLT x44 = x1 * x39;
	const GEN_FLT x45 = x4 * x7;
	const GEN_FLT x46 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x47 = x4 * x6;
	const GEN_FLT x48 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x49 = (x44 * x45) + (x46 * x47) + (x8 * x48);
	const GEN_FLT x50 = x5 * x7;
	const GEN_FLT x51 = x50 * x39;
	const GEN_FLT x52 = x1 * x46;
	const GEN_FLT x53 = x3 * x39;
	const GEN_FLT x54 = x2 * x6;
	const GEN_FLT x55 = (x4 * x53) + (x54 * x48) + (x42 * x47);
	const GEN_FLT x56 = -1 * x44;
	const GEN_FLT x57 = x4 * x4;
	const GEN_FLT x58 = 2 * x47;
	const GEN_FLT x59 = (-1 * x3) + x9;
	const GEN_FLT x60 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x61 = x15 * x16;
	const GEN_FLT x62 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x63 = x62 * x17;
	const GEN_FLT x64 = -1 * x63;
	const GEN_FLT x65 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x66 = x65 * x17;
	const GEN_FLT x67 = x62 * x14;
	const GEN_FLT x68 = x67 * x11;
	const GEN_FLT x69 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x70 = x62 * x18;
	const GEN_FLT x71 = (x61 * x69) + (x70 * x19) + (x60 * x20);
	const GEN_FLT x72 = x69 * x17;
	const GEN_FLT x73 = x67 * x19;
	const GEN_FLT x74 = (x60 * x23) + (x70 * x11) + (x61 * x65);
	const GEN_FLT x75 = (((2 * x60 * x61) + x64 + (x63 * x35)) * sensor_x) +
						(((-1 * x66) + (-1 * x68) + x71) * sensor_y) + ((x72 + x73 + x74) * sensor_z);
	const GEN_FLT x76 = x60 * x17;
	const GEN_FLT x77 = x67 * x16;
	const GEN_FLT x78 = (x69 * x23) + (x65 * x20) + (x63 * x11 * x19);
	const GEN_FLT x79 = ((x66 + x68 + x71) * sensor_x) + ((x64 + (2 * x69 * x20) + (x63 * x29)) * sensor_y) +
						(((-1 * x76) + (-1 * x77) + x78) * sensor_z);
	const GEN_FLT x80 = x1 * x7;
	const GEN_FLT x81 = x2 * x47;
	const GEN_FLT x82 = x80 + x81;
	const GEN_FLT x83 = (x6 * x57) + x5;
	const GEN_FLT x84 = (((-1 * x72) + (-1 * x73) + x74) * sensor_x) + ((x76 + x77 + x78) * sensor_y) +
						((x64 + (2 * x65 * x23) + (x63 * x12)) * sensor_z);
	const GEN_FLT x85 = (x75 * x59) + (x82 * x79) + (x83 * x84);
	const GEN_FLT x86 = (x36 * ((-1 * x41) + (-1 * x43) + x49)) + (x32 * (x51 + x52 + x55)) +
						(x25 * (x56 + (x58 * x48) + (x57 * x44))) + x85;
	const GEN_FLT x87 = -1 * x86 * x38;
	const GEN_FLT x88 = 2 * x8;
	const GEN_FLT x89 = x4 * x5;
	const GEN_FLT x90 = x89 * x39;
	const GEN_FLT x91 = x1 * x48;
	const GEN_FLT x92 = (x54 * x46) + (x7 * x53) + (x8 * x42);
	const GEN_FLT x93 = (x75 * x34) + (x79 * x28) + (x84 * x10);
	const GEN_FLT x94 = (x36 * (x56 + (x44 * x33) + (x88 * x46))) + (x32 * ((-1 * x90) + (-1 * x91) + x92)) +
						(x25 * (x41 + x43 + x49)) + x93;
	const GEN_FLT x95 = 1 + x94;
	const GEN_FLT x96 = x37 * x37;
	const GEN_FLT x97 = lh_pz + (x82 * x32) + (x83 * x25) + (x59 * x36);
	const GEN_FLT x98 = x97 * pow(x96, -1);
	const GEN_FLT x99 = (x97 * x97) + x96;
	const GEN_FLT x100 = pow(x99, -1);
	const GEN_FLT x101 = x96 * x100;
	const GEN_FLT x102 = x101 * (x87 + (x98 * x95));
	const GEN_FLT x103 = x2 * x2;
	const GEN_FLT x104 = 2 * x54;
	const GEN_FLT x105 = (x6 * x103) + x5;
	const GEN_FLT x106 = x26 + x27;
	const GEN_FLT x107 = (-1 * x80) + x81;
	const GEN_FLT x108 = (x79 * x105) + (x75 * x106) + (x84 * x107);
	const GEN_FLT x109 = (x36 * (x90 + x91 + x92)) + (x32 * (x56 + (x44 * x103) + (x42 * x104))) +
						 (x25 * ((-1 * x51) + (-1 * x52) + x55)) + x108;
	const GEN_FLT x110 = lh_py + (x25 * x107) + (x32 * x105) + (x36 * x106);
	const GEN_FLT x111 = 2 * x110;
	const GEN_FLT x112 = x109 * x111;
	const GEN_FLT x113 = 2 * x37;
	const GEN_FLT x114 = 2 * x97;
	const GEN_FLT x115 = x86 * x114;
	const GEN_FLT x116 = (x95 * x113) + x115;
	const GEN_FLT x117 = x112 + x116;
	const GEN_FLT x118 = 0.523598775598299 + tilt_0;
	const GEN_FLT x119 = cos(x118);
	const GEN_FLT x120 = pow(x119, -1);
	const GEN_FLT x121 = x110 * x110;
	const GEN_FLT x122 = x121 + x99;
	const GEN_FLT x123 = 1.0 / 2.0 * x110;
	const GEN_FLT x124 = x123 * pow(x122, -3.0 / 2.0);
	const GEN_FLT x125 = x120 * x124;
	const GEN_FLT x126 = pow(x122, -1.0 / 2.0);
	const GEN_FLT x127 = x109 * x126;
	const GEN_FLT x128 = x120 * x127;
	const GEN_FLT x129 = (-1 * x117 * x125) + x128;
	const GEN_FLT x130 = pow(x122, -1) * x121;
	const GEN_FLT x131 = pow((1 + (-1 * pow(x119, -2) * x130)), -1.0 / 2.0);
	const GEN_FLT x132 = tan(x118);
	const GEN_FLT x133 = pow(x99, -1.0 / 2.0);
	const GEN_FLT x134 = x110 * x133;
	const GEN_FLT x135 = x134 * x132;
	const GEN_FLT x136 = atan2(-1 * x97, x37);
	const GEN_FLT x137 = ogeeMag_0 + (-1 * asin(x135)) + x136;
	const GEN_FLT x138 = curve_0 + (sin(x137) * ogeePhase_0);
	const GEN_FLT x139 = x110 * x126;
	const GEN_FLT x140 = asin(x120 * x139);
	const GEN_FLT x141 = 8.0108022e-06 * x140;
	const GEN_FLT x142 = -8.0108022e-06 + (-1 * x141);
	const GEN_FLT x143 = 0.0028679863 + (x140 * x142);
	const GEN_FLT x144 = 5.3685255e-06 + (x140 * x143);
	const GEN_FLT x145 = 0.0076069798 + (x140 * x144);
	const GEN_FLT x146 = x140 * x145;
	const GEN_FLT x147 = -8.0108022e-06 + (-1.60216044e-05 * x140);
	const GEN_FLT x148 = (x140 * x147) + x143;
	const GEN_FLT x149 = (x140 * x148) + x144;
	const GEN_FLT x150 = (x140 * x149) + x145;
	const GEN_FLT x151 = x146 + (x140 * x150);
	const GEN_FLT x152 = sin(x118);
	const GEN_FLT x153 = x138 * x152;
	const GEN_FLT x154 = (-1 * x151 * x153) + x119;
	const GEN_FLT x155 = pow(x154, -1);
	const GEN_FLT x156 = 2 * x138 * x146 * x155;
	const GEN_FLT x157 = x131 * x156;
	const GEN_FLT x158 = x131 * x150;
	const GEN_FLT x159 = x131 * x149;
	const GEN_FLT x160 = x131 * x148;
	const GEN_FLT x161 = x131 * x142;
	const GEN_FLT x162 = x129 * x161;
	const GEN_FLT x163 = 2.40324066e-05 * x140;
	const GEN_FLT x164 = x163 * x131;
	const GEN_FLT x165 = x131 * x147;
	const GEN_FLT x166 = x131 * x143;
	const GEN_FLT x167 = x131 * x141;
	const GEN_FLT x168 = (x129 * x166) + (x140 * ((-1 * x129 * x167) + x162));
	const GEN_FLT x169 = x131 * x144;
	const GEN_FLT x170 = (x129 * x169) + (x168 * x140);
	const GEN_FLT x171 = x131 * x145;
	const GEN_FLT x172 = x100 * x121;
	const GEN_FLT x173 = pow((1 + (-1 * x172 * (x132 * x132))), -1.0 / 2.0);
	const GEN_FLT x174 = pow(x99, -3.0 / 2.0) * x123;
	const GEN_FLT x175 = x116 * x174;
	const GEN_FLT x176 = x133 * x132;
	const GEN_FLT x177 = x109 * x176;
	const GEN_FLT x178 = (-1 * x175 * x132) + x177;
	const GEN_FLT x179 = x102 + (-1 * x178 * x173);
	const GEN_FLT x180 = cos(x137) * ogeePhase_0;
	const GEN_FLT x181 = x151 * x152;
	const GEN_FLT x182 = x181 * x180;
	const GEN_FLT x183 = x140 * x140;
	const GEN_FLT x184 = x183 * x138;
	const GEN_FLT x185 = x184 * x145 * pow(x154, -2);
	const GEN_FLT x186 = x184 * x155;
	const GEN_FLT x187 = x183 * x145 * x155;
	const GEN_FLT x188 = x180 * x187;
	const GEN_FLT x189 = (x186 * x145) + x135;
	const GEN_FLT x190 = pow((1 + (-1 * (x189 * x189))), -1.0 / 2.0);
	const GEN_FLT x191 =
		x102 +
		(-1 * x190 *
		 ((x129 * x157) +
		  (-1 * x185 *
		   ((-1 * x153 *
			 ((x129 * x158) + (x129 * x171) +
			  (x140 * ((x129 * x159) +
					   (x140 * ((x129 * x160) + (x140 * (x162 + (-1 * x129 * x164) + (x129 * x165))) + x168)) + x170)) +
			  (x170 * x140))) +
			(-1 * x179 * x182))) +
		  (x170 * x186) + (x179 * x188) + x178));
	const GEN_FLT x192 = cos(gibPhase_0 + (-1 * asin(x189)) + x136) * gibMag_0;
	const GEN_FLT x193 = x98 * x94;
	const GEN_FLT x194 = x101 * (x87 + x193);
	const GEN_FLT x195 = 1 + x109;
	const GEN_FLT x196 = x94 * x113;
	const GEN_FLT x197 = x196 + x115;
	const GEN_FLT x198 = (x111 * x195) + x197;
	const GEN_FLT x199 = x126 * x195;
	const GEN_FLT x200 = (-1 * x125 * x198) + (x120 * x199);
	const GEN_FLT x201 = x200 * x131;
	const GEN_FLT x202 = x200 * x161;
	const GEN_FLT x203 = (x201 * x143) + (x140 * ((-1 * x201 * x141) + x202));
	const GEN_FLT x204 = (x201 * x144) + (x203 * x140);
	const GEN_FLT x205 = x174 * x197;
	const GEN_FLT x206 = (-1 * x205 * x132) + (x176 * x195);
	const GEN_FLT x207 = x194 + (-1 * x206 * x173);
	const GEN_FLT x208 =
		x194 +
		(-1 * x190 *
		 ((x201 * x156) +
		  (-1 * x185 *
		   ((-1 * x153 *
			 ((x201 * x150) +
			  (x140 * ((x201 * x149) +
					   (x140 * ((x201 * x148) + (x140 * (x202 + (-1 * x201 * x163) + (x201 * x147))) + x203)) + x204)) +
			  (x200 * x171) + (x204 * x140))) +
			(-1 * x207 * x182))) +
		  (x204 * x186) + (x207 * x188) + x206));
	const GEN_FLT x209 = 1 + x86;
	const GEN_FLT x210 = x101 * ((-1 * x38 * x209) + x193);
	const GEN_FLT x211 = x196 + (x209 * x114);
	const GEN_FLT x212 = x112 + x211;
	const GEN_FLT x213 = (-1 * x212 * x125) + x128;
	const GEN_FLT x214 = x213 * x131;
	const GEN_FLT x215 = x213 * x161;
	const GEN_FLT x216 = (x213 * x166) + (x140 * ((-1 * x213 * x167) + x215));
	const GEN_FLT x217 = (x213 * x169) + (x216 * x140);
	const GEN_FLT x218 = x174 * x132;
	const GEN_FLT x219 = (-1 * x211 * x218) + x177;
	const GEN_FLT x220 = x210 + (-1 * x219 * x173);
	const GEN_FLT x221 =
		x210 +
		(-1 * x190 *
		 ((x214 * x156) +
		  (-1 * x185 *
		   ((-1 * x153 *
			 ((x213 * x158) +
			  (x140 * ((x213 * x159) +
					   (x140 * ((x213 * x160) + (x140 * (x215 + (-1 * x214 * x163) + (x213 * x165))) + x216)) + x217)) +
			  (x213 * x171) + (x217 * x140))) +
			(-1 * x220 * x182))) +
		  (x220 * x188) + (x217 * x186) + x219));
	const GEN_FLT x222 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x223 = x40 * x222;
	const GEN_FLT x224 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qj *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x225 = x1 * x224;
	const GEN_FLT x226 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qi *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x227 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qk *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x228 = x1 * x222;
	const GEN_FLT x229 = x7 * x228;
	const GEN_FLT x230 = (x47 * x226) + (x8 * x227) + (x4 * x229);
	const GEN_FLT x231 = x50 * x222;
	const GEN_FLT x232 = x1 * x226;
	const GEN_FLT x233 = x2 * x4;
	const GEN_FLT x234 = (x233 * x228) + (x54 * x227) + (x47 * x224);
	const GEN_FLT x235 = -1 * x228;
	const GEN_FLT x236 = (x36 * ((-1 * x223) + (-1 * x225) + x230)) + (x32 * (x231 + x232 + x234)) +
						 (x25 * (x235 + (x57 * x228) + (x58 * x227))) + x85;
	const GEN_FLT x237 = x89 * x222;
	const GEN_FLT x238 = x1 * x227;
	const GEN_FLT x239 = (x2 * x229) + (x54 * x226) + (x8 * x224);
	const GEN_FLT x240 = (x36 * (x235 + (x33 * x228) + (x88 * x226))) + (x32 * ((-1 * x237) + (-1 * x238) + x239)) +
						 (x25 * (x223 + x225 + x230)) + x93;
	const GEN_FLT x241 = ((-1 * x38 * x236) + (x98 * x240)) * x101;
	const GEN_FLT x242 = (x32 * (x235 + (x228 * x103) + (x224 * x104))) + (x25 * ((-1 * x231) + (-1 * x232) + x234)) +
						 (x36 * (x237 + x238 + x239)) + x108;
	const GEN_FLT x243 = (x240 * x113) + (x236 * x114);
	const GEN_FLT x244 = (x242 * x111) + x243;
	const GEN_FLT x245 = x242 * x126;
	const GEN_FLT x246 = (-1 * x244 * x125) + (x245 * x120);
	const GEN_FLT x247 = x246 * x161;
	const GEN_FLT x248 = (x246 * x166) + (x140 * ((-1 * x246 * x167) + x247));
	const GEN_FLT x249 = (x246 * x169) + (x248 * x140);
	const GEN_FLT x250 = x243 * x174;
	const GEN_FLT x251 = x242 * x133;
	const GEN_FLT x252 = (-1 * x250 * x132) + (x251 * x132);
	const GEN_FLT x253 = x241 + (-1 * x252 * x173);
	const GEN_FLT x254 =
		x241 +
		(-1 * x190 *
		 ((x246 * x157) + (x249 * x186) +
		  (-1 * x185 *
		   ((-1 * x153 *
			 ((x246 * x158) +
			  (x140 * ((x246 * x159) +
					   (x140 * ((x246 * x160) + (x140 * (x247 + (-1 * x246 * x164) + (x246 * x165))) + x248)) + x249)) +
			  (x246 * x171) + (x249 * x140))) +
			(-1 * x253 * x182))) +
		  (x253 * x188) + x252));
	const GEN_FLT x255 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x256 = x40 * x255;
	const GEN_FLT x257 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qj *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x258 = x1 * x257;
	const GEN_FLT x259 = x1 * x255;
	const GEN_FLT x260 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qi *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x261 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qk *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x262 = (x45 * x259) + (x47 * x260) + (x8 * x261);
	const GEN_FLT x263 = x50 * x255;
	const GEN_FLT x264 = x1 * x260;
	const GEN_FLT x265 = (x233 * x259) + (x54 * x261) + (x47 * x257);
	const GEN_FLT x266 = -1 * x259;
	const GEN_FLT x267 = (x36 * ((-1 * x256) + (-1 * x258) + x262)) + (x32 * (x263 + x264 + x265)) +
						 (x25 * (x266 + (x57 * x259) + (x58 * x261))) + x85;
	const GEN_FLT x268 = x89 * x255;
	const GEN_FLT x269 = x1 * x261;
	const GEN_FLT x270 = x2 * x7;
	const GEN_FLT x271 = (x270 * x259) + (x8 * x257) + (x54 * x260);
	const GEN_FLT x272 = (x36 * ((x33 * x259) + x266 + (x88 * x260))) + (x32 * ((-1 * x268) + (-1 * x269) + x271)) +
						 (x25 * (x256 + x258 + x262)) + x93;
	const GEN_FLT x273 = ((-1 * x38 * x267) + (x98 * x272)) * x101;
	const GEN_FLT x274 = (x36 * (x268 + x269 + x271)) + (x32 * (x266 + (x259 * x103) + (x257 * x104))) +
						 (x25 * ((-1 * x263) + (-1 * x264) + x265)) + x108;
	const GEN_FLT x275 = (x272 * x113) + (x267 * x114);
	const GEN_FLT x276 = x124 * ((x274 * x111) + x275);
	const GEN_FLT x277 = x274 * x126;
	const GEN_FLT x278 = (-1 * x276 * x120) + (x277 * x120);
	const GEN_FLT x279 = x278 * x131;
	const GEN_FLT x280 = x278 * x161;
	const GEN_FLT x281 = (x279 * x143) + (x140 * ((-1 * x279 * x141) + x280));
	const GEN_FLT x282 = (x279 * x144) + (x281 * x140);
	const GEN_FLT x283 = x274 * x133;
	const GEN_FLT x284 = (-1 * x218 * x275) + (x283 * x132);
	const GEN_FLT x285 = x273 + (-1 * x284 * x173);
	const GEN_FLT x286 =
		x273 +
		(-1 * x190 *
		 ((x279 * x156) +
		  (-1 * x185 *
		   ((-1 * x153 *
			 ((x279 * x150) + (x282 * x140) +
			  (x140 * ((x279 * x149) +
					   (x140 * ((x279 * x148) + (x140 * (x280 + (-1 * x279 * x163) + (x279 * x147))) + x281)) + x282)) +
			  (x278 * x171))) +
			(-1 * x285 * x182))) +
		  (x285 * x188) + x284 + (x282 * x186)));
	const GEN_FLT x287 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x288 = x40 * x287;
	const GEN_FLT x289 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qj *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x290 = x1 * x289;
	const GEN_FLT x291 = x1 * x287;
	const GEN_FLT x292 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qi *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x293 = x6 * x292;
	const GEN_FLT x294 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qk *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x295 = (x45 * x291) + (x4 * x293) + (x8 * x294);
	const GEN_FLT x296 = x50 * x287;
	const GEN_FLT x297 = x1 * x292;
	const GEN_FLT x298 = (x233 * x291) + (x54 * x294) + (x47 * x289);
	const GEN_FLT x299 = -1 * x291;
	const GEN_FLT x300 = (x36 * ((-1 * x288) + (-1 * x290) + x295)) + (x32 * (x296 + x297 + x298)) +
						 (x25 * (x299 + (x57 * x291) + (x58 * x294))) + x85;
	const GEN_FLT x301 = x89 * x287;
	const GEN_FLT x302 = x1 * x294;
	const GEN_FLT x303 = (x270 * x291) + (x2 * x293) + (x8 * x289);
	const GEN_FLT x304 = (x36 * (x299 + (x33 * x291) + (2 * x7 * x293))) + (x32 * ((-1 * x301) + (-1 * x302) + x303)) +
						 (x25 * (x288 + x290 + x295)) + x93;
	const GEN_FLT x305 = ((-1 * x38 * x300) + (x98 * x304)) * x101;
	const GEN_FLT x306 = (x36 * (x301 + x302 + x303)) + (x32 * (x299 + (x291 * x103) + (x289 * x104))) +
						 (x25 * ((-1 * x296) + (-1 * x297) + x298)) + x108;
	const GEN_FLT x307 = (x304 * x113) + (x300 * x114);
	const GEN_FLT x308 = x124 * ((x306 * x111) + x307);
	const GEN_FLT x309 = x306 * x126;
	const GEN_FLT x310 = (-1 * x308 * x120) + (x309 * x120);
	const GEN_FLT x311 = x310 * x161;
	const GEN_FLT x312 = (x310 * x166) + (x140 * ((-1 * x310 * x167) + x311));
	const GEN_FLT x313 = (x310 * x169) + (x312 * x140);
	const GEN_FLT x314 = x306 * x133;
	const GEN_FLT x315 = (-1 * x218 * x307) + (x314 * x132);
	const GEN_FLT x316 = x180 * (x305 + (-1 * x315 * x173));
	const GEN_FLT x317 =
		x305 +
		(-1 * x190 *
		 ((x310 * x157) + x315 +
		  (-1 * x185 *
		   ((-1 * x153 *
			 ((x310 * x158) +
			  (x140 * ((x310 * x159) +
					   (x140 * ((x310 * x160) + (x140 * (x311 + (-1 * x310 * x164) + (x310 * x165))) + x312)) + x313)) +
			  (x310 * x171) + (x313 * x140))) +
			(-1 * x316 * x181))) +
		  (x316 * x187) + (x313 * x186)));
	const GEN_FLT x318 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x319 = tan(x318);
	const GEN_FLT x320 = pow((1 + (-1 * (x319 * x319) * x172)), -1.0 / 2.0);
	const GEN_FLT x321 = x319 * x133;
	const GEN_FLT x322 = -1 * x321 * x109;
	const GEN_FLT x323 = (x319 * x175) + x322;
	const GEN_FLT x324 = x102 + (-1 * x323 * x320);
	const GEN_FLT x325 = -1 * x319 * x134;
	const GEN_FLT x326 = ogeeMag_1 + (-1 * asin(x325)) + x136;
	const GEN_FLT x327 = cos(x326) * ogeePhase_1;
	const GEN_FLT x328 = cos(x318);
	const GEN_FLT x329 = pow(x328, -1);
	const GEN_FLT x330 = asin(x329 * x139);
	const GEN_FLT x331 = 8.0108022e-06 * x330;
	const GEN_FLT x332 = -8.0108022e-06 + (-1 * x331);
	const GEN_FLT x333 = 0.0028679863 + (x332 * x330);
	const GEN_FLT x334 = 5.3685255e-06 + (x333 * x330);
	const GEN_FLT x335 = 0.0076069798 + (x334 * x330);
	const GEN_FLT x336 = x330 * x330;
	const GEN_FLT x337 = x335 * x330;
	const GEN_FLT x338 = -8.0108022e-06 + (-1.60216044e-05 * x330);
	const GEN_FLT x339 = (x338 * x330) + x333;
	const GEN_FLT x340 = (x339 * x330) + x334;
	const GEN_FLT x341 = (x330 * x340) + x335;
	const GEN_FLT x342 = x337 + (x330 * x341);
	const GEN_FLT x343 = curve_1 + (sin(x326) * ogeePhase_1);
	const GEN_FLT x344 = sin(x318);
	const GEN_FLT x345 = x344 * x343;
	const GEN_FLT x346 = (x345 * x342) + x328;
	const GEN_FLT x347 = pow(x346, -1);
	const GEN_FLT x348 = x336 * x347 * x335;
	const GEN_FLT x349 = x327 * x348;
	const GEN_FLT x350 = pow((1 + (-1 * pow(x328, -2) * x130)), -1.0 / 2.0);
	const GEN_FLT x351 = x329 * x124;
	const GEN_FLT x352 = x329 * x127;
	const GEN_FLT x353 = (-1 * x351 * x117) + x352;
	const GEN_FLT x354 = x350 * x353;
	const GEN_FLT x355 = x354 * x332;
	const GEN_FLT x356 = x350 * x333;
	const GEN_FLT x357 = (x330 * ((-1 * x354 * x331) + x355)) + (x353 * x356);
	const GEN_FLT x358 = (x354 * x334) + (x357 * x330);
	const GEN_FLT x359 = x336 * x343;
	const GEN_FLT x360 = x359 * x347;
	const GEN_FLT x361 = 2 * x337 * x347 * x343;
	const GEN_FLT x362 = x344 * x342;
	const GEN_FLT x363 = x362 * x327;
	const GEN_FLT x364 = 2.40324066e-05 * x330;
	const GEN_FLT x365 = x350 * x339;
	const GEN_FLT x366 = x359 * pow(x346, -2) * x335;
	const GEN_FLT x367 = (x360 * x335) + x325;
	const GEN_FLT x368 = pow((1 + (-1 * (x367 * x367))), -1.0 / 2.0);
	const GEN_FLT x369 =
		x102 +
		(-1 * x368 *
		 ((x324 * x349) + (x360 * x358) + (x361 * x354) +
		  (-1 * x366 *
		   ((x363 * x324) +
			(x345 *
			 ((x330 * ((x354 * x340) +
					   (x330 * ((x330 * (x355 + (-1 * x364 * x354) + (x354 * x338))) + (x365 * x353) + x357)) + x358)) +
			  (x358 * x330) + (x354 * x341) + (x354 * x335))))) +
		  x323));
	const GEN_FLT x370 = cos(gibPhase_1 + x136 + (-1 * asin(x367))) * gibMag_1;
	const GEN_FLT x371 = (x205 * x319) + (-1 * x321 * x195);
	const GEN_FLT x372 = x194 + (-1 * x371 * x320);
	const GEN_FLT x373 = (-1 * x351 * x198) + (x329 * x199);
	const GEN_FLT x374 = x373 * x350;
	const GEN_FLT x375 = x374 * x332;
	const GEN_FLT x376 = (x330 * ((-1 * x374 * x331) + x375)) + (x373 * x356);
	const GEN_FLT x377 = (x374 * x334) + (x376 * x330);
	const GEN_FLT x378 =
		x194 +
		(-1 * x368 *
		 ((x372 * x349) + (x377 * x360) + (x374 * x361) +
		  (-1 * x366 *
		   ((x372 * x363) +
			(x345 *
			 ((x374 * x341) +
			  (x330 * ((x374 * x340) +
					   (x330 * ((x330 * (x375 + (-1 * x374 * x364) + (x374 * x338))) + (x373 * x365) + x376)) + x377)) +
			  (x377 * x330) + (x374 * x335))))) +
		  x371));
	const GEN_FLT x379 = x319 * x174;
	const GEN_FLT x380 = (x211 * x379) + x322;
	const GEN_FLT x381 = x210 + (-1 * x380 * x320);
	const GEN_FLT x382 = (-1 * x212 * x351) + x352;
	const GEN_FLT x383 = x350 * x382;
	const GEN_FLT x384 = x383 * x332;
	const GEN_FLT x385 = (x330 * ((-1 * x383 * x331) + x384)) + (x382 * x356);
	const GEN_FLT x386 = (x383 * x334) + (x385 * x330);
	const GEN_FLT x387 =
		x210 +
		(-1 * x368 *
		 ((x381 * x349) + (x360 * x386) +
		  (-1 * x366 *
		   ((x363 * x381) +
			(x345 *
			 ((x383 * x341) +
			  (x330 * ((x383 * x340) +
					   (x330 * ((x330 * (x384 + (-1 * x364 * x383) + (x383 * x338))) + (x365 * x382) + x385)) + x386)) +
			  (x383 * x335) + (x386 * x330))))) +
		  (x361 * x383) + x380));
	const GEN_FLT x388 = (x250 * x319) + (-1 * x251 * x319);
	const GEN_FLT x389 = x327 * (x241 + (-1 * x388 * x320));
	const GEN_FLT x390 = (-1 * x244 * x351) + (x245 * x329);
	const GEN_FLT x391 = x390 * x350;
	const GEN_FLT x392 = x391 * x332;
	const GEN_FLT x393 = (x330 * ((-1 * x391 * x331) + x392)) + (x390 * x356);
	const GEN_FLT x394 = (x391 * x334) + (x393 * x330);
	const GEN_FLT x395 =
		x241 +
		(-1 * x368 *
		 ((x389 * x348) + (x391 * x361) + (x394 * x360) +
		  (-1 * x366 *
		   ((x362 * x389) +
			(x345 *
			 ((x391 * x341) +
			  (x330 * ((x391 * x340) +
					   (x330 * ((x330 * (x392 + (-1 * x391 * x364) + (x391 * x338))) + (x391 * x339) + x393)) + x394)) +
			  (x394 * x330) + (x391 * x335))))) +
		  x388));
	const GEN_FLT x396 = (x275 * x379) + (-1 * x283 * x319);
	const GEN_FLT x397 = x273 + (-1 * x396 * x320);
	const GEN_FLT x398 = (-1 * x276 * x329) + (x277 * x329);
	const GEN_FLT x399 = x398 * x350;
	const GEN_FLT x400 = x399 * x332;
	const GEN_FLT x401 = (x330 * ((-1 * x399 * x331) + x400)) + (x398 * x356);
	const GEN_FLT x402 = (x399 * x334) + (x401 * x330);
	const GEN_FLT x403 =
		x273 +
		(-1 * x368 *
		 ((x397 * x349) + (x402 * x360) +
		  (-1 * x366 *
		   ((x397 * x363) +
			(x345 *
			 ((x330 * ((x399 * x340) +
					   (x330 * ((x330 * (x400 + (-1 * x399 * x364) + (x399 * x338))) + x401 + (x399 * x339))) + x402)) +
			  (x399 * x335) + (x402 * x330) + (x399 * x341))))) +
		  (x399 * x361) + x396));
	const GEN_FLT x404 = (x379 * x307) + (-1 * x314 * x319);
	const GEN_FLT x405 = x305 + (-1 * x404 * x320);
	const GEN_FLT x406 = (-1 * x308 * x329) + (x309 * x329);
	const GEN_FLT x407 = x406 * x350;
	const GEN_FLT x408 = x407 * x332;
	const GEN_FLT x409 = (x330 * ((-1 * x407 * x331) + x408)) + (x406 * x356);
	const GEN_FLT x410 = (x407 * x334) + (x409 * x330);
	const GEN_FLT x411 =
		x305 +
		(-1 * x368 *
		 ((x405 * x349) + (x410 * x360) + (x407 * x361) +
		  (-1 * x366 *
		   ((x405 * x363) +
			(x345 *
			 ((x407 * x341) + (x410 * x330) +
			  (x330 * ((x407 * x340) +
					   (x330 * ((x330 * (x408 + (-1 * x407 * x364) + (x407 * x338))) + (x406 * x365) + x409)) + x410)) +
			  (x407 * x335))))) +
		  x404));
	out[0] = (x191 * x192) + x191;
	out[1] = (x208 * x192) + x208;
	out[2] = (x221 * x192) + x221;
	out[3] = (x254 * x192) + x254;
	out[4] = (x286 * x192) + x286;
	out[5] = (x317 * x192) + x317;
	out[6] = (x369 * x370) + x369;
	out[7] = (x378 * x370) + x378;
	out[8] = (x387 * x370) + x387;
	out[9] = (x395 * x370) + x395;
	out[10] = (x403 * x370) + x403;
	out[11] = (x411 * x370) + x411;
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x20 = x15 * x11;
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x20 * x16;
	const GEN_FLT x24 =
		obj_pz + (((x15 * x12) + x14) * sensor_z) + ((x18 + x21) * sensor_y) + (((-1 * x22) + x23) * sensor_x);
	const GEN_FLT x25 = x1 * x4;
	const GEN_FLT x26 = x2 * x8;
	const GEN_FLT x27 = (-1 * x25) + x26;
	const GEN_FLT x28 = x19 * x19;
	const GEN_FLT x29 = x11 * x17;
	const GEN_FLT x30 = x15 * x19;
	const GEN_FLT x31 = x30 * x16;
	const GEN_FLT x32 =
		obj_py + (((x28 * x15) + x14) * sensor_y) + (((-1 * x18) + x21) * sensor_z) + ((x29 + x31) * sensor_x);
	const GEN_FLT x33 = x7 * x7;
	const GEN_FLT x34 = (x6 * x33) + x5;
	const GEN_FLT x35 = x16 * x16;
	const GEN_FLT x36 =
		obj_px + ((x22 + x23) * sensor_z) + (((-1 * x29) + x31) * sensor_y) + (((x35 * x15) + x14) * sensor_x);
	const GEN_FLT x37 = lh_px + (x24 * x10) + (x34 * x36) + (x32 * x27);
	const GEN_FLT x38 = x37 * x37;
	const GEN_FLT x39 = x4 * x4;
	const GEN_FLT x40 = (x6 * x39) + x5;
	const GEN_FLT x41 = x1 * x7;
	const GEN_FLT x42 = x4 * x6;
	const GEN_FLT x43 = x2 * x42;
	const GEN_FLT x44 = x41 + x43;
	const GEN_FLT x45 = (-1 * x3) + x9;
	const GEN_FLT x46 = lh_pz + (x40 * x24) + (x44 * x32) + (x45 * x36);
	const GEN_FLT x47 = (x46 * x46) + x38;
	const GEN_FLT x48 = pow(x47, -1);
	const GEN_FLT x49 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x50 = x5 * x49;
	const GEN_FLT x51 = x2 * x50;
	const GEN_FLT x52 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x53 = x1 * x52;
	const GEN_FLT x54 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x55 = x6 * x54;
	const GEN_FLT x56 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x57 = (x7 * x49 * x25) + (x4 * x55) + (x8 * x56);
	const GEN_FLT x58 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x59 = x15 * x16;
	const GEN_FLT x60 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x61 = x60 * x17;
	const GEN_FLT x62 = -1 * x61;
	const GEN_FLT x63 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x64 = x63 * x17;
	const GEN_FLT x65 = x60 * x14;
	const GEN_FLT x66 = x65 * x11;
	const GEN_FLT x67 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x68 = x60 * x19;
	const GEN_FLT x69 = (x67 * x59) + (x68 * x18) + (x58 * x30);
	const GEN_FLT x70 = x67 * x17;
	const GEN_FLT x71 = x65 * x19;
	const GEN_FLT x72 = (x58 * x20) + (x60 * x11 * x18) + (x63 * x59);
	const GEN_FLT x73 = (((2 * x58 * x59) + x62 + (x61 * x35)) * sensor_x) +
						(((-1 * x64) + (-1 * x66) + x69) * sensor_y) + ((x70 + x71 + x72) * sensor_z);
	const GEN_FLT x74 = x7 * x50;
	const GEN_FLT x75 = x1 * x54;
	const GEN_FLT x76 = x3 * x49;
	const GEN_FLT x77 = x2 * x6;
	const GEN_FLT x78 = (x4 * x76) + (x77 * x56) + (x52 * x42);
	const GEN_FLT x79 = x58 * x17;
	const GEN_FLT x80 = x65 * x16;
	const GEN_FLT x81 = (x67 * x20) + (x63 * x30) + (x68 * x29);
	const GEN_FLT x82 = ((x64 + x66 + x69) * sensor_x) + ((x62 + (2 * x67 * x30) + (x61 * x28)) * sensor_y) +
						(((-1 * x79) + (-1 * x80) + x81) * sensor_z);
	const GEN_FLT x83 = x1 * x49;
	const GEN_FLT x84 = -1 * x83;
	const GEN_FLT x85 = (((-1 * x70) + (-1 * x71) + x72) * sensor_x) + ((x79 + x80 + x81) * sensor_y) +
						((x62 + (2 * x63 * x20) + (x61 * x12)) * sensor_z);
	const GEN_FLT x86 = (x36 * ((-1 * x51) + (-1 * x53) + x57)) + (x73 * x45) + (x32 * (x74 + x75 + x78)) +
						(x85 * x40) + (x82 * x44) + (x24 * (x84 + (x83 * x39) + (2 * x56 * x42)));
	const GEN_FLT x87 = x4 * x50;
	const GEN_FLT x88 = x1 * x56;
	const GEN_FLT x89 = (x2 * x55) + (x8 * x52) + (x7 * x76);
	const GEN_FLT x90 = (x36 * (x84 + (x83 * x33) + (2 * x8 * x54))) + (x32 * ((-1 * x87) + (-1 * x88) + x89)) +
						(x82 * x27) + (x73 * x34) + (x24 * (x51 + x53 + x57)) + (x85 * x10);
	const GEN_FLT x91 = x48 * x38 * ((-1 * x86 * pow(x37, -1)) + (x90 * x46 * pow(x38, -1)));
	const GEN_FLT x92 = 0.523598775598299 + tilt_0;
	const GEN_FLT x93 = cos(x92);
	const GEN_FLT x94 = pow(x93, -1);
	const GEN_FLT x95 = (-1 * x41) + x43;
	const GEN_FLT x96 = x2 * x2;
	const GEN_FLT x97 = (x6 * x96) + x5;
	const GEN_FLT x98 = x25 + x26;
	const GEN_FLT x99 = lh_py + (x95 * x24) + (x97 * x32) + (x98 * x36);
	const GEN_FLT x100 = x99 * x99;
	const GEN_FLT x101 = x100 + x47;
	const GEN_FLT x102 = pow(x101, -1.0 / 2.0);
	const GEN_FLT x103 = x99 * x102;
	const GEN_FLT x104 = asin(x94 * x103);
	const GEN_FLT x105 = -8.0108022e-06 + (-1.60216044e-05 * x104);
	const GEN_FLT x106 = 8.0108022e-06 * x104;
	const GEN_FLT x107 = -8.0108022e-06 + (-1 * x106);
	const GEN_FLT x108 = 0.0028679863 + (x104 * x107);
	const GEN_FLT x109 = (x105 * x104) + x108;
	const GEN_FLT x110 = 5.3685255e-06 + (x108 * x104);
	const GEN_FLT x111 = (x109 * x104) + x110;
	const GEN_FLT x112 = 0.0076069798 + (x104 * x110);
	const GEN_FLT x113 = (x104 * x111) + x112;
	const GEN_FLT x114 = pow(x93, -2);
	const GEN_FLT x115 = pow(x101, -1) * x100;
	const GEN_FLT x116 = pow((1 + (-1 * x114 * x115)), -1.0 / 2.0);
	const GEN_FLT x117 = (x36 * (x87 + x88 + x89)) + (x73 * x98) + (x32 * (x84 + (x83 * x96) + (2 * x77 * x52))) +
						 (x82 * x97) + (x85 * x95) + (x24 * ((-1 * x74) + (-1 * x75) + x78));
	const GEN_FLT x118 = (2 * x90 * x37) + (2 * x86 * x46);
	const GEN_FLT x119 = 1.0 / 2.0 * x99;
	const GEN_FLT x120 = pow(x101, -3.0 / 2.0) * x119 * ((2 * x99 * x117) + x118);
	const GEN_FLT x121 = x102 * x117;
	const GEN_FLT x122 = (-1 * x94 * x120) + (x94 * x121);
	const GEN_FLT x123 = x116 * x122;
	const GEN_FLT x124 = x107 * x123;
	const GEN_FLT x125 = 2.40324066e-05 * x104;
	const GEN_FLT x126 = (x108 * x123) + (x104 * ((-1 * x106 * x123) + x124));
	const GEN_FLT x127 = x110 * x116;
	const GEN_FLT x128 = (x122 * x127) + (x104 * x126);
	const GEN_FLT x129 = sin(x92);
	const GEN_FLT x130 = tan(x92);
	const GEN_FLT x131 = pow(x47, -1.0 / 2.0);
	const GEN_FLT x132 = x99 * x131;
	const GEN_FLT x133 = x130 * x132;
	const GEN_FLT x134 = atan2(-1 * x46, x37);
	const GEN_FLT x135 = ogeeMag_0 + (-1 * asin(x133)) + x134;
	const GEN_FLT x136 = sin(x135);
	const GEN_FLT x137 = curve_0 + (x136 * ogeePhase_0);
	const GEN_FLT x138 = x129 * x137;
	const GEN_FLT x139 =
		-1 * x138 *
		((x113 * x123) + (x112 * x123) +
		 (x104 * ((x111 * x123) +
				  (x104 * ((x109 * x123) + (x104 * (x124 + (-1 * x123 * x125) + (x105 * x123))) + x126)) + x128)) +
		 (x104 * x128));
	const GEN_FLT x140 = x104 * x112;
	const GEN_FLT x141 = x140 + (x104 * x113);
	const GEN_FLT x142 = x129 * x141;
	const GEN_FLT x143 = x130 * x130;
	const GEN_FLT x144 = x48 * x100;
	const GEN_FLT x145 = pow((1 + (-1 * x144 * x143)), -1.0 / 2.0);
	const GEN_FLT x146 = pow(x47, -3.0 / 2.0) * x118 * x119;
	const GEN_FLT x147 = x117 * x131;
	const GEN_FLT x148 = (-1 * x130 * x146) + (x130 * x147);
	const GEN_FLT x149 = x91 + (-1 * x145 * x148);
	const GEN_FLT x150 = cos(x135) * ogeePhase_0;
	const GEN_FLT x151 = x149 * x150;
	const GEN_FLT x152 = (-1 * x138 * x141) + x93;
	const GEN_FLT x153 = x104 * x104;
	const GEN_FLT x154 = x112 * x153;
	const GEN_FLT x155 = x137 * pow(x152, -2) * x154;
	const GEN_FLT x156 = pow(x152, -1);
	const GEN_FLT x157 = x154 * x156;
	const GEN_FLT x158 = x137 * x156;
	const GEN_FLT x159 = 2 * x140 * x158;
	const GEN_FLT x160 = x153 * x158;
	const GEN_FLT x161 = (x123 * x159) + (x128 * x160) + x148;
	const GEN_FLT x162 = (x154 * x158) + x133;
	const GEN_FLT x163 = pow((1 + (-1 * (x162 * x162))), -1.0 / 2.0);
	const GEN_FLT x164 = x91 + (-1 * x163 * ((-1 * x155 * (x139 + (-1 * x142 * x151))) + (x151 * x157) + x161));
	const GEN_FLT x165 = gibPhase_0 + (-1 * asin(x162)) + x134;
	const GEN_FLT x166 = cos(x165) * gibMag_0;
	const GEN_FLT x167 = (x166 * x164) + x164;
	const GEN_FLT x168 = (x103 * x114 * x129) + x122;
	const GEN_FLT x169 = x116 * x168;
	const GEN_FLT x170 = x107 * x169;
	const GEN_FLT x171 = (x108 * x169) + (x104 * ((-1 * x106 * x169) + x170));
	const GEN_FLT x172 = (x127 * x168) + (x104 * x171);
	const GEN_FLT x173 = (x132 * (1 + x143)) + x148;
	const GEN_FLT x174 = x91 + (-1 * x173 * x145);
	const GEN_FLT x175 = x142 * x150;
	const GEN_FLT x176 = x150 * x157;
	const GEN_FLT x177 =
		x91 +
		(-1 * x163 *
		 ((x169 * x159) +
		  (-1 * x155 *
		   ((-1 * x129) + (-1 * x93 * x137 * x141) +
			(-1 * x138 *
			 ((x113 * x169) +
			  (x104 * ((x111 * x169) +
					   (x104 * ((x109 * x169) + (x104 * (x170 + (-1 * x125 * x169) + (x105 * x169))) + x171)) + x172)) +
			  (x104 * x172) + (x112 * x169))) +
			(-1 * x174 * x175))) +
		  (x176 * x174) + (x160 * x172) + x173));
	const GEN_FLT x178 = 1 + x151;
	const GEN_FLT x179 = x91 + (-1 * x163 * ((-1 * x155 * (x139 + (-1 * x178 * x142))) + (x178 * x157) + x161));
	const GEN_FLT x180 = 1 + x149;
	const GEN_FLT x181 = x91 + (-1 * x163 * ((-1 * x155 * (x139 + (-1 * x175 * x180))) + (x176 * x180) + x161));
	const GEN_FLT x182 = x151 + x136;
	const GEN_FLT x183 = x91 + (-1 * x163 * ((-1 * x155 * (x139 + (-1 * x182 * x142))) + (x182 * x157) + x161));
	const GEN_FLT x184 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x185 = cos(x184);
	const GEN_FLT x186 = pow(x185, -1);
	const GEN_FLT x187 = asin(x103 * x186);
	const GEN_FLT x188 = 8.0108022e-06 * x187;
	const GEN_FLT x189 = -8.0108022e-06 + (-1 * x188);
	const GEN_FLT x190 = 0.0028679863 + (x187 * x189);
	const GEN_FLT x191 = 5.3685255e-06 + (x187 * x190);
	const GEN_FLT x192 = 0.0076069798 + (x187 * x191);
	const GEN_FLT x193 = tan(x184);
	const GEN_FLT x194 = -1 * x193 * x132;
	const GEN_FLT x195 = ogeeMag_1 + (-1 * asin(x194)) + x134;
	const GEN_FLT x196 = sin(x195);
	const GEN_FLT x197 = curve_1 + (x196 * ogeePhase_1);
	const GEN_FLT x198 = x187 * x187;
	const GEN_FLT x199 = sin(x184);
	const GEN_FLT x200 = x187 * x192;
	const GEN_FLT x201 = -8.0108022e-06 + (-1.60216044e-05 * x187);
	const GEN_FLT x202 = (x201 * x187) + x190;
	const GEN_FLT x203 = (x202 * x187) + x191;
	const GEN_FLT x204 = (x203 * x187) + x192;
	const GEN_FLT x205 = x200 + (x204 * x187);
	const GEN_FLT x206 = x205 * x199;
	const GEN_FLT x207 = (x206 * x197) + x185;
	const GEN_FLT x208 = pow(x207, -1);
	const GEN_FLT x209 = x208 * x198;
	const GEN_FLT x210 = x209 * x197;
	const GEN_FLT x211 = (x210 * x192) + x194;
	const GEN_FLT x212 = pow((1 + (-1 * (x211 * x211))), -1.0 / 2.0);
	const GEN_FLT x213 = x193 * x193;
	const GEN_FLT x214 = pow((1 + (-1 * x213 * x144)), -1.0 / 2.0);
	const GEN_FLT x215 = (x193 * x146) + (-1 * x193 * x147);
	const GEN_FLT x216 = x91 + (-1 * x215 * x214);
	const GEN_FLT x217 = cos(x195) * ogeePhase_1;
	const GEN_FLT x218 = x217 * x216;
	const GEN_FLT x219 = x209 * x192;
	const GEN_FLT x220 = (-1 * x120 * x186) + (x121 * x186);
	const GEN_FLT x221 = pow(x185, -2);
	const GEN_FLT x222 = pow((1 + (-1 * x221 * x115)), -1.0 / 2.0);
	const GEN_FLT x223 = x220 * x222;
	const GEN_FLT x224 = x223 * x189;
	const GEN_FLT x225 = 2.40324066e-05 * x187;
	const GEN_FLT x226 = (x187 * ((-1 * x223 * x188) + x224)) + (x223 * x190);
	const GEN_FLT x227 = (x223 * x191) + (x226 * x187);
	const GEN_FLT x228 = x197 * x199;
	const GEN_FLT x229 =
		x228 *
		((x204 * x223) +
		 (x187 * ((x203 * x223) +
				  (x187 * ((x187 * (x224 + (-1 * x223 * x225) + (x201 * x223))) + (x202 * x223) + x226)) + x227)) +
		 (x227 * x187) + (x223 * x192));
	const GEN_FLT x230 = pow(x207, -2) * x197 * x198 * x192;
	const GEN_FLT x231 = 2 * x200 * x208 * x197;
	const GEN_FLT x232 = (x210 * x227) + (x231 * x223) + x215;
	const GEN_FLT x233 = x91 + (-1 * x212 * ((x219 * x218) + (-1 * x230 * ((x218 * x206) + x229)) + x232));
	const GEN_FLT x234 = gibPhase_1 + x134 + (-1 * asin(x211));
	const GEN_FLT x235 = cos(x234) * gibMag_1;
	const GEN_FLT x236 = (x233 * x235) + x233;
	const GEN_FLT x237 = (x132 * (1 + x213)) + x215;
	const GEN_FLT x238 = x91 + (-1 * x214 * x237);
	const GEN_FLT x239 = x219 * x217;
	const GEN_FLT x240 = x222 * ((-1 * x221 * x103 * x199) + x220);
	const GEN_FLT x241 = x240 * x189;
	const GEN_FLT x242 = (x187 * ((-1 * x240 * x188) + x241)) + (x240 * x190);
	const GEN_FLT x243 = (x240 * x191) + (x242 * x187);
	const GEN_FLT x244 = x217 * x206;
	const GEN_FLT x245 =
		x91 +
		(-1 * x212 *
		 ((x238 * x239) + (x210 * x243) + (x231 * x240) +
		  (-1 * x230 *
		   ((x238 * x244) + x199 + (-1 * x205 * x185 * x197) +
			(x228 *
			 ((x204 * x240) +
			  (x187 * ((x203 * x240) +
					   (x187 * ((x187 * (x241 + (-1 * x225 * x240) + (x201 * x240))) + (x202 * x240) + x242)) + x243)) +
			  (x243 * x187) + (x240 * x192))))) +
		  x237));
	const GEN_FLT x246 = 1 + x218;
	const GEN_FLT x247 = x91 + (-1 * x212 * ((x219 * x246) + (-1 * x230 * ((x206 * x246) + x229)) + x232));
	const GEN_FLT x248 = 1 + x216;
	const GEN_FLT x249 = x91 + (-1 * x212 * ((x239 * x248) + (-1 * x230 * ((x244 * x248) + x229)) + x232));
	const GEN_FLT x250 = x218 + x196;
	const GEN_FLT x251 = x91 + (-1 * x212 * ((x219 * x250) + (-1 * x230 * ((x206 * x250) + x229)) + x232));
	out[0] = -1 + x167;
	out[1] = (x166 * x177) + x177;
	out[2] = (x166 * x179) + x179;
	out[3] = (x166 * (1 + x164)) + x164;
	out[4] = sin(x165) + x167;
	out[5] = (x166 * x181) + x181;
	out[6] = (x166 * x183) + x183;
	out[7] = x167;
	out[8] = x167;
	out[9] = x167;
	out[10] = x167;
	out[11] = x167;
	out[12] = x167;
	out[13] = x167;
	out[14] = x236;
	out[15] = x236;
	out[16] = x236;
	out[17] = x236;
	out[18] = x236;
	out[19] = x236;
	out[20] = x236;
	out[21] = -1 + x236;
	out[22] = (x235 * x245) + x245;
	out[23] = (x235 * x247) + x247;
	out[24] = (x235 * (1 + x233)) + x233;
	out[25] = sin(x234) + x236;
	out[26] = (x235 * x249) + x249;
	out[27] = (x235 * x251) + x251;
}

/** Applying function <function reproject_axis_x_gen2 at 0x7f35a4af8950> */
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x11 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x12 = cos(x11);
	const GEN_FLT x13 = 1 + (-1 * x12);
	const GEN_FLT x14 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x15 = sin(x11);
	const GEN_FLT x16 = x15 * x14;
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x18 = x13 * x10 * x17;
	const GEN_FLT x19 = x15 * x17;
	const GEN_FLT x20 = x14 * x13;
	const GEN_FLT x21 = x20 * x10;
	const GEN_FLT x22 =
		obj_pz + (((x13 * (x10 * x10)) + x12) * sensor_z) + ((x16 + x18) * sensor_y) + (((-1 * x19) + x21) * sensor_x);
	const GEN_FLT x23 = x15 * x10;
	const GEN_FLT x24 = x20 * x17;
	const GEN_FLT x25 =
		obj_py + (((-1 * x16) + x18) * sensor_z) + (((x13 * (x17 * x17)) + x12) * sensor_y) + ((x23 + x24) * sensor_x);
	const GEN_FLT x26 = x1 * x7;
	const GEN_FLT x27 = x2 * x4 * x6;
	const GEN_FLT x28 =
		obj_px + ((x19 + x21) * sensor_z) + (((-1 * x23) + x24) * sensor_y) + ((((x14 * x14) * x13) + x12) * sensor_x);
	const GEN_FLT x29 = lh_py + (((-1 * x3) + x9) * x22) + (x25 * (((x4 * x4) * x6) + x5)) + ((x26 + x27) * x28);
	const GEN_FLT x30 = 0.523598775598299 + tilt_0;
	const GEN_FLT x31 = cos(x30);
	const GEN_FLT x32 = x1 * x4;
	const GEN_FLT x33 = x2 * x8;
	const GEN_FLT x34 = lh_pz + (x22 * ((x6 * (x7 * x7)) + x5)) + ((x3 + x9) * x25) + (((-1 * x32) + x33) * x28);
	const GEN_FLT x35 = lh_px + ((x32 + x33) * x22) + (((-1 * x26) + x27) * x25) + (x28 * (((x2 * x2) * x6) + x5));
	const GEN_FLT x36 = (x34 * x34) + (x35 * x35);
	const GEN_FLT x37 = asin(pow(x31, -1) * x29 * pow(((x29 * x29) + x36), -1.0 / 2.0));
	const GEN_FLT x38 = 0.0028679863 + (x37 * (-8.0108022e-06 + (-8.0108022e-06 * x37)));
	const GEN_FLT x39 = 5.3685255e-06 + (x38 * x37);
	const GEN_FLT x40 = 0.0076069798 + (x37 * x39);
	const GEN_FLT x41 = pow(x36, -1.0 / 2.0) * x29 * tan(x30);
	const GEN_FLT x42 = atan2(-1 * x34, x35);
	const GEN_FLT x43 = curve_0 + (sin(ogeeMag_0 + (-1 * asin(x41)) + x42) * ogeePhase_0);
	const GEN_FLT x44 =
		(-1 * asin((x40 * x43 * (x37 * x37) *
					pow(((-1 * x43 * sin(x30) *
						  ((x40 * x37) +
						   (x37 * ((x37 * ((x37 * ((x37 * (-8.0108022e-06 + (-1.60216044e-05 * x37))) + x38)) + x39)) +
								   x40)))) +
						 x31),
						-1)) +
				   x41)) +
		x42;
	out[0] = -1.5707963267949 + (sin(gibPhase_0 + x44) * gibMag_0) + (-1 * phase_0) + x44;
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x5 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x20 * x11;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x15 * x11;
	const GEN_FLT x24 = x23 * x16;
	const GEN_FLT x25 =
		obj_pz + (((x15 * x12) + x14) * sensor_z) + ((x18 + x21) * sensor_y) + (((-1 * x22) + x24) * sensor_x);
	const GEN_FLT x26 = x1 * x5;
	const GEN_FLT x27 = x2 * x7;
	const GEN_FLT x28 = x4 * x27;
	const GEN_FLT x29 = (-1 * x26) + x28;
	const GEN_FLT x30 = x19 * x19;
	const GEN_FLT x31 = x11 * x17;
	const GEN_FLT x32 = x20 * x16;
	const GEN_FLT x33 =
		obj_py + (((-1 * x18) + x21) * sensor_z) + ((x31 + x32) * sensor_x) + (((x30 * x15) + x14) * sensor_y);
	const GEN_FLT x34 = x4 * x4;
	const GEN_FLT x35 = (x7 * x34) + x6;
	const GEN_FLT x36 = x16 * x16;
	const GEN_FLT x37 =
		obj_px + (((x36 * x15) + x14) * sensor_x) + ((x22 + x24) * sensor_z) + (((-1 * x31) + x32) * sensor_y);
	const GEN_FLT x38 = lh_px + (x25 * x10) + (x33 * x29) + (x35 * x37);
	const GEN_FLT x39 = pow(x38, -1);
	const GEN_FLT x40 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x41 = x40 * x17;
	const GEN_FLT x42 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x43 = x42 * x14;
	const GEN_FLT x44 = x43 * x11;
	const GEN_FLT x45 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x46 = x15 * x16;
	const GEN_FLT x47 = x42 * x18;
	const GEN_FLT x48 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x49 = (x45 * x46) + (x47 * x19) + (x48 * x20);
	const GEN_FLT x50 = x42 * x17;
	const GEN_FLT x51 = -1 * x50;
	const GEN_FLT x52 = 2 * x20;
	const GEN_FLT x53 = x48 * x17;
	const GEN_FLT x54 = x43 * x16;
	const GEN_FLT x55 = x11 * x19;
	const GEN_FLT x56 = (x45 * x23) + (x40 * x20) + (x50 * x55);
	const GEN_FLT x57 = ((x41 + x44 + x49) * sensor_x) + ((x51 + (x52 * x45) + (x50 * x30)) * sensor_y) +
						(((-1 * x53) + (-1 * x54) + x56) * sensor_z);
	const GEN_FLT x58 = x1 * x4;
	const GEN_FLT x59 = x2 * x8;
	const GEN_FLT x60 = x58 + x59;
	const GEN_FLT x61 = x60 * x57;
	const GEN_FLT x62 = (-1 * x3) + x9;
	const GEN_FLT x63 = 2 * x46;
	const GEN_FLT x64 = x45 * x17;
	const GEN_FLT x65 = x43 * x19;
	const GEN_FLT x66 = (x48 * x23) + (x47 * x11) + (x40 * x46);
	const GEN_FLT x67 = (((x63 * x48) + x51 + (x50 * x36)) * sensor_x) + (((-1 * x41) + (-1 * x44) + x49) * sensor_y) +
						((x64 + x66 + x65) * sensor_z);
	const GEN_FLT x68 = 1 + x67;
	const GEN_FLT x69 = x5 * x5;
	const GEN_FLT x70 = (x7 * x69) + x6;
	const GEN_FLT x71 = 2 * x23;
	const GEN_FLT x72 = (((-1 * x64) + x66 + (-1 * x65)) * sensor_x) + ((x53 + x54 + x56) * sensor_y) +
						((x51 + (x71 * x40) + (x50 * x12)) * sensor_z);
	const GEN_FLT x73 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x74 = x6 * x73;
	const GEN_FLT x75 = x4 * x74;
	const GEN_FLT x76 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x77 = x1 * x76;
	const GEN_FLT x78 = x1 * x73;
	const GEN_FLT x79 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x80 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x81 = (x2 * x5 * x78) + (x79 * x27) + (x8 * x80);
	const GEN_FLT x82 = -1 * x78;
	const GEN_FLT x83 = x2 * x74;
	const GEN_FLT x84 = x1 * x80;
	const GEN_FLT x85 = x4 * x78;
	const GEN_FLT x86 = x4 * x7;
	const GEN_FLT x87 = (x5 * x85) + (x8 * x76) + (x86 * x79);
	const GEN_FLT x88 = (x33 * (x75 + x77 + x81)) + (x25 * (x82 + (x78 * x69) + (2 * x8 * x79))) +
						(x37 * ((-1 * x83) + (-1 * x84) + x87));
	const GEN_FLT x89 = (x70 * x72) + x88;
	const GEN_FLT x90 = x61 + (x62 * x68) + x89;
	const GEN_FLT x91 = x72 * x10;
	const GEN_FLT x92 = x5 * x74;
	const GEN_FLT x93 = x1 * x79;
	const GEN_FLT x94 = (x76 * x27) + (x80 * x86) + (x2 * x85);
	const GEN_FLT x95 = (x37 * (x82 + (x78 * x34) + (2 * x86 * x76))) + (x25 * (x83 + x84 + x87)) +
						(x33 * ((-1 * x92) + (-1 * x93) + x94));
	const GEN_FLT x96 = (x57 * x29) + x95;
	const GEN_FLT x97 = (x68 * x35) + x91 + x96;
	const GEN_FLT x98 = x38 * x38;
	const GEN_FLT x99 = lh_pz + (x70 * x25) + (x60 * x33) + (x62 * x37);
	const GEN_FLT x100 = x99 * pow(x98, -1);
	const GEN_FLT x101 = (x99 * x99) + x98;
	const GEN_FLT x102 = pow(x101, -1);
	const GEN_FLT x103 = x98 * x102;
	const GEN_FLT x104 = x103 * ((-1 * x90 * x39) + (x97 * x100));
	const GEN_FLT x105 = x26 + x28;
	const GEN_FLT x106 = x2 * x2;
	const GEN_FLT x107 = (x7 * x106) + x6;
	const GEN_FLT x108 = x57 * x107;
	const GEN_FLT x109 = (-1 * x58) + x59;
	const GEN_FLT x110 = (x33 * (x82 + (x78 * x106) + (2 * x80 * x27))) + (x25 * ((-1 * x75) + (-1 * x77) + x81)) +
						 (x37 * (x92 + x93 + x94));
	const GEN_FLT x111 = (x72 * x109) + x110;
	const GEN_FLT x112 = (x68 * x105) + x111 + x108;
	const GEN_FLT x113 = lh_py + (x33 * x107) + (x25 * x109) + (x37 * x105);
	const GEN_FLT x114 = 2 * x113;
	const GEN_FLT x115 = 2 * x38;
	const GEN_FLT x116 = 2 * x99;
	const GEN_FLT x117 = (x97 * x115) + (x90 * x116);
	const GEN_FLT x118 = 0.523598775598299 + tilt_0;
	const GEN_FLT x119 = cos(x118);
	const GEN_FLT x120 = pow(x119, -1);
	const GEN_FLT x121 = x113 * x113;
	const GEN_FLT x122 = x121 + x101;
	const GEN_FLT x123 = 1.0 / 2.0 * x113;
	const GEN_FLT x124 = x120 * x123 * pow(x122, -3.0 / 2.0);
	const GEN_FLT x125 = x120 * pow(x122, -1.0 / 2.0);
	const GEN_FLT x126 = pow((1 + (-1 * pow(x119, -2) * pow(x122, -1) * x121)), -1.0 / 2.0);
	const GEN_FLT x127 = x126 * ((-1 * x124 * ((x112 * x114) + x117)) + (x112 * x125));
	const GEN_FLT x128 = tan(x118);
	const GEN_FLT x129 = pow(x101, -1.0 / 2.0) * x128;
	const GEN_FLT x130 = x113 * x129;
	const GEN_FLT x131 = atan2(-1 * x99, x38);
	const GEN_FLT x132 = ogeeMag_0 + (-1 * asin(x130)) + x131;
	const GEN_FLT x133 = curve_0 + (sin(x132) * ogeePhase_0);
	const GEN_FLT x134 = asin(x113 * x125);
	const GEN_FLT x135 = 8.0108022e-06 * x134;
	const GEN_FLT x136 = -8.0108022e-06 + (-1 * x135);
	const GEN_FLT x137 = 0.0028679863 + (x134 * x136);
	const GEN_FLT x138 = 5.3685255e-06 + (x134 * x137);
	const GEN_FLT x139 = 0.0076069798 + (x134 * x138);
	const GEN_FLT x140 = x134 * x139;
	const GEN_FLT x141 = -8.0108022e-06 + (-1.60216044e-05 * x134);
	const GEN_FLT x142 = (x134 * x141) + x137;
	const GEN_FLT x143 = (x134 * x142) + x138;
	const GEN_FLT x144 = (x134 * x143) + x139;
	const GEN_FLT x145 = x140 + (x134 * x144);
	const GEN_FLT x146 = sin(x118);
	const GEN_FLT x147 = x133 * x146;
	const GEN_FLT x148 = (-1 * x145 * x147) + x119;
	const GEN_FLT x149 = pow(x148, -1);
	const GEN_FLT x150 = 2 * x133 * x140 * x149;
	const GEN_FLT x151 = pow(x101, -3.0 / 2.0) * x123 * x128;
	const GEN_FLT x152 = (-1 * x117 * x151) + (x112 * x129);
	const GEN_FLT x153 = pow((1 + (-1 * x102 * x121 * (x128 * x128))), -1.0 / 2.0);
	const GEN_FLT x154 = x104 + (-1 * x152 * x153);
	const GEN_FLT x155 = cos(x132) * ogeePhase_0;
	const GEN_FLT x156 = x134 * x134;
	const GEN_FLT x157 = x139 * x149 * x156;
	const GEN_FLT x158 = x155 * x157;
	const GEN_FLT x159 = x127 * x136;
	const GEN_FLT x160 = 2.40324066e-05 * x134;
	const GEN_FLT x161 = (x127 * x137) + (x134 * ((-1 * x127 * x135) + x159));
	const GEN_FLT x162 = (x127 * x138) + (x161 * x134);
	const GEN_FLT x163 = x146 * x145;
	const GEN_FLT x164 = x163 * x155;
	const GEN_FLT x165 = x133 * x156;
	const GEN_FLT x166 = x165 * x139 * pow(x148, -2);
	const GEN_FLT x167 = x165 * x149;
	const GEN_FLT x168 = (x167 * x139) + x130;
	const GEN_FLT x169 = pow((1 + (-1 * (x168 * x168))), -1.0 / 2.0);
	const GEN_FLT x170 =
		x104 +
		(-1 * x169 *
		 ((x127 * x150) + (x154 * x158) +
		  (-1 * x166 *
		   ((-1 * x147 *
			 ((x127 * x144) + (x162 * x134) +
			  (x134 * ((x127 * x143) +
					   (x134 * ((x127 * x142) + (x134 * (x159 + (x127 * x141) + (-1 * x127 * x160))) + x161)) + x162)) +
			  (x127 * x139))) +
			(-1 * x164 * x154))) +
		  (x167 * x162) + x152));
	const GEN_FLT x171 = cos(gibPhase_0 + (-1 * asin(x168)) + x131) * gibMag_0;
	const GEN_FLT x172 = x62 * x67;
	const GEN_FLT x173 = 1 + x57;
	const GEN_FLT x174 = x172 + (x60 * x173) + x89;
	const GEN_FLT x175 = x67 * x35;
	const GEN_FLT x176 = x91 + x175 + (x29 * x173) + x95;
	const GEN_FLT x177 = x103 * ((-1 * x39 * x174) + (x100 * x176));
	const GEN_FLT x178 = x67 * x105;
	const GEN_FLT x179 = x178 + (x107 * x173) + x111;
	const GEN_FLT x180 = (x115 * x176) + (x116 * x174);
	const GEN_FLT x181 = (-1 * x124 * ((x114 * x179) + x180)) + (x125 * x179);
	const GEN_FLT x182 = x126 * x181;
	const GEN_FLT x183 = x182 * x136;
	const GEN_FLT x184 = (x182 * x137) + (x134 * ((-1 * x182 * x135) + x183));
	const GEN_FLT x185 = (x182 * x138) + (x184 * x134);
	const GEN_FLT x186 = x126 * x139;
	const GEN_FLT x187 = (-1 * x180 * x151) + (x129 * x179);
	const GEN_FLT x188 = x177 + (-1 * x187 * x153);
	const GEN_FLT x189 =
		x177 +
		(-1 * x169 *
		 ((x182 * x150) +
		  (-1 * x166 *
		   ((-1 * x147 *
			 ((x182 * x144) +
			  (x134 * ((x182 * x143) +
					   (x134 * ((x182 * x142) + (x134 * (x183 + (-1 * x160 * x182) + (x182 * x141))) + x184)) + x185)) +
			  (x181 * x186) + (x185 * x134))) +
			(-1 * x164 * x188))) +
		  (x167 * x185) + (x188 * x158) + x187));
	const GEN_FLT x190 = 1 + x72;
	const GEN_FLT x191 = x172 + (x70 * x190) + x61 + x88;
	const GEN_FLT x192 = x175 + (x10 * x190) + x96;
	const GEN_FLT x193 = x103 * ((-1 * x39 * x191) + (x100 * x192));
	const GEN_FLT x194 = x178 + x108 + (x109 * x190) + x110;
	const GEN_FLT x195 = (x115 * x192) + (x116 * x191);
	const GEN_FLT x196 = (-1 * x124 * ((x114 * x194) + x195)) + (x125 * x194);
	const GEN_FLT x197 = x126 * x196;
	const GEN_FLT x198 = x197 * x136;
	const GEN_FLT x199 = (x197 * x137) + (x134 * ((-1 * x197 * x135) + x198));
	const GEN_FLT x200 = (x197 * x138) + (x199 * x134);
	const GEN_FLT x201 = (-1 * x195 * x151) + (x129 * x194);
	const GEN_FLT x202 = x155 * (x193 + (-1 * x201 * x153));
	const GEN_FLT x203 =
		x193 +
		(-1 * x169 *
		 ((x197 * x150) + x201 +
		  (-1 * x166 *
		   ((-1 * x147 *
			 ((x197 * x144) + (x186 * x196) +
			  (x134 * ((x197 * x143) +
					   (x134 * ((x197 * x142) + (x134 * (x198 + (-1 * x160 * x197) + (x197 * x141))) + x199)) + x200)) +
			  (x200 * x134))) +
			(-1 * x202 * x163))) +
		  (x200 * x167) + (x202 * x157)));
	const GEN_FLT x204 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x205 = x17 * x204;
	const GEN_FLT x206 = -1 * x205;
	const GEN_FLT x207 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qi *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x208 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qk *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x209 = x17 * x208;
	const GEN_FLT x210 = x14 * x11;
	const GEN_FLT x211 = x210 * x204;
	const GEN_FLT x212 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qj *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x213 = x19 * x16;
	const GEN_FLT x214 = (x20 * x207) + (x46 * x212) + (x213 * x205);
	const GEN_FLT x215 = x17 * x212;
	const GEN_FLT x216 = x14 * x19;
	const GEN_FLT x217 = x216 * x204;
	const GEN_FLT x218 = x11 * x16;
	const GEN_FLT x219 = (x23 * x207) + (x46 * x208) + (x218 * x205);
	const GEN_FLT x220 = ((x206 + (x63 * x207) + (x36 * x205)) * sensor_x) +
						 (((-1 * x209) + (-1 * x211) + x214) * sensor_y) + ((x215 + x217 + x219) * sensor_z);
	const GEN_FLT x221 = x17 * x207;
	const GEN_FLT x222 = x14 * x16;
	const GEN_FLT x223 = x204 * x222;
	const GEN_FLT x224 = (x23 * x212) + (x20 * x208) + (x55 * x205);
	const GEN_FLT x225 = ((x209 + x211 + x214) * sensor_x) + (((x52 * x212) + x206 + (x30 * x205)) * sensor_y) +
						 (((-1 * x221) + (-1 * x223) + x224) * sensor_z);
	const GEN_FLT x226 = (((-1 * x215) + (-1 * x217) + x219) * sensor_x) + ((x221 + x223 + x224) * sensor_y) +
						 ((x206 + (x71 * x208) + (x12 * x205)) * sensor_z);
	const GEN_FLT x227 = (x62 * x220) + (x60 * x225) + (x70 * x226) + x88;
	const GEN_FLT x228 = (x35 * x220) + (x29 * x225) + (x10 * x226) + x95;
	const GEN_FLT x229 = x103 * ((-1 * x39 * x227) + (x228 * x100));
	const GEN_FLT x230 = (x220 * x105) + (x225 * x107) + (x226 * x109) + x110;
	const GEN_FLT x231 = (x228 * x115) + (x227 * x116);
	const GEN_FLT x232 = (-1 * x124 * ((x230 * x114) + x231)) + (x230 * x125);
	const GEN_FLT x233 = x232 * x126;
	const GEN_FLT x234 = x233 * x136;
	const GEN_FLT x235 = (x233 * x137) + (x134 * ((-1 * x233 * x135) + x234));
	const GEN_FLT x236 = (x233 * x138) + (x235 * x134);
	const GEN_FLT x237 = (-1 * x231 * x151) + (x230 * x129);
	const GEN_FLT x238 = x229 + (-1 * x237 * x153);
	const GEN_FLT x239 =
		x229 +
		(-1 * x169 *
		 ((x233 * x150) +
		  (-1 * x166 *
		   ((-1 * x147 *
			 ((x233 * x144) +
			  (x134 * ((x233 * x143) +
					   (x134 * ((x233 * x142) + x235 + (x134 * (x234 + (-1 * x233 * x160) + (x233 * x141))))) + x236)) +
			  (x232 * x186) + (x236 * x134))) +
			(-1 * x238 * x164))) +
		  x237 + (x236 * x167) + (x238 * x158)));
	const GEN_FLT x240 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x241 = x17 * x240;
	const GEN_FLT x242 = -1 * x241;
	const GEN_FLT x243 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qi *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x244 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qk *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x245 = x17 * x244;
	const GEN_FLT x246 = x210 * x240;
	const GEN_FLT x247 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qj *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x248 = (x20 * x243) + (x46 * x247) + (x213 * x241);
	const GEN_FLT x249 = x17 * x247;
	const GEN_FLT x250 = x216 * x240;
	const GEN_FLT x251 = (x23 * x243) + (x218 * x241) + (x46 * x244);
	const GEN_FLT x252 = ((x242 + (x63 * x243) + (x36 * x241)) * sensor_x) +
						 (((-1 * x245) + (-1 * x246) + x248) * sensor_y) + ((x249 + x250 + x251) * sensor_z);
	const GEN_FLT x253 = x17 * x243;
	const GEN_FLT x254 = x222 * x240;
	const GEN_FLT x255 = (x20 * x244) + (x55 * x241) + (x23 * x247);
	const GEN_FLT x256 = ((x242 + (x52 * x247) + (x30 * x241)) * sensor_y) + ((x245 + x246 + x248) * sensor_x) +
						 (((-1 * x253) + (-1 * x254) + x255) * sensor_z);
	const GEN_FLT x257 = (((-1 * x249) + (-1 * x250) + x251) * sensor_x) + ((x253 + x254 + x255) * sensor_y) +
						 ((x242 + (x71 * x244) + (x12 * x241)) * sensor_z);
	const GEN_FLT x258 = (x62 * x252) + (x60 * x256) + (x70 * x257) + x88;
	const GEN_FLT x259 = (x35 * x252) + (x29 * x256) + x95 + (x10 * x257);
	const GEN_FLT x260 = x103 * ((-1 * x39 * x258) + (x259 * x100));
	const GEN_FLT x261 = (x252 * x105) + (x256 * x107) + (x257 * x109) + x110;
	const GEN_FLT x262 = (x259 * x115) + (x258 * x116);
	const GEN_FLT x263 = x126 * ((-1 * x124 * ((x261 * x114) + x262)) + (x261 * x125));
	const GEN_FLT x264 = x263 * x136;
	const GEN_FLT x265 = (x263 * x137) + (x134 * ((-1 * x263 * x135) + x264));
	const GEN_FLT x266 = (x263 * x138) + (x265 * x134);
	const GEN_FLT x267 = (-1 * x262 * x151) + (x261 * x129);
	const GEN_FLT x268 = x260 + (-1 * x267 * x153);
	const GEN_FLT x269 =
		x260 +
		(-1 * x169 *
		 ((x263 * x150) +
		  (-1 * x166 *
		   ((-1 * x147 *
			 ((x263 * x144) +
			  (x134 * ((x263 * x143) +
					   (x134 * ((x263 * x142) + (x134 * (x264 + (-1 * x263 * x160) + (x263 * x141))) + x265)) + x266)) +
			  (x263 * x139) + (x266 * x134))) +
			(-1 * x268 * x164))) +
		  (x266 * x167) + (x268 * x158) + x267));
	const GEN_FLT x270 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x271 = x17 * x270;
	const GEN_FLT x272 = -1 * x271;
	const GEN_FLT x273 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qi *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x274 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qk *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x275 = x17 * x274;
	const GEN_FLT x276 = x210 * x270;
	const GEN_FLT x277 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qj *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x278 = x15 * x277;
	const GEN_FLT x279 = (x20 * x273) + (x16 * x278) + (x213 * x271);
	const GEN_FLT x280 = x17 * x277;
	const GEN_FLT x281 = x216 * x270;
	const GEN_FLT x282 = (x23 * x273) + (x218 * x271) + (x46 * x274);
	const GEN_FLT x283 = ((x272 + (x63 * x273) + (x36 * x271)) * sensor_x) +
						 (((-1 * x275) + (-1 * x276) + x279) * sensor_y) + ((x280 + x282 + x281) * sensor_z);
	const GEN_FLT x284 = x17 * x273;
	const GEN_FLT x285 = x270 * x222;
	const GEN_FLT x286 = (x11 * x278) + (x20 * x274) + (x55 * x271);
	const GEN_FLT x287 = ((x275 + x276 + x279) * sensor_x) + (((x52 * x277) + (x30 * x271) + x272) * sensor_y) +
						 (((-1 * x284) + (-1 * x285) + x286) * sensor_z);
	const GEN_FLT x288 = (((-1 * x280) + x282 + (-1 * x281)) * sensor_x) + ((x284 + x285 + x286) * sensor_y) +
						 ((x272 + (x71 * x274) + (x12 * x271)) * sensor_z);
	const GEN_FLT x289 = (x62 * x283) + (x60 * x287) + (x70 * x288) + x88;
	const GEN_FLT x290 = (x35 * x283) + (x10 * x288) + (x29 * x287) + x95;
	const GEN_FLT x291 = x103 * ((-1 * x39 * x289) + (x290 * x100));
	const GEN_FLT x292 = (x283 * x105) + (x287 * x107) + x110 + (x288 * x109);
	const GEN_FLT x293 = (x290 * x115) + (x289 * x116);
	const GEN_FLT x294 = (-1 * x124 * ((x292 * x114) + x293)) + (x292 * x125);
	const GEN_FLT x295 = x294 * x126;
	const GEN_FLT x296 = (-1 * x293 * x151) + (x292 * x129);
	const GEN_FLT x297 = x291 + (-1 * x296 * x153);
	const GEN_FLT x298 = x295 * x136;
	const GEN_FLT x299 = (x295 * x137) + (x134 * ((-1 * x295 * x135) + x298));
	const GEN_FLT x300 = (x295 * x138) + (x299 * x134);
	const GEN_FLT x301 =
		x291 +
		(-1 * x169 *
		 ((x295 * x150) + (x297 * x158) +
		  (-1 * x166 *
		   ((-1 * x147 *
			 ((x295 * x144) +
			  (x134 * ((x295 * x143) +
					   (x134 * ((x295 * x142) + (x134 * (x298 + (-1 * x295 * x160) + (x295 * x141))) + x299)) + x300)) +
			  (x294 * x186) + (x300 * x134))) +
			(-1 * x297 * x164))) +
		  (x300 * x167) + x296));
	out[0] = (x170 * x171) + x170;
	out[1] = (x171 * x189) + x189;
	out[2] = (x203 * x171) + x203;
	out[3] = (x239 * x171) + x239;
	out[4] = (x269 * x171) + x269;
	out[5] = (x301 * x171) + x301;
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (x15 * x12) + x14;
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x18 = sin(x13);
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x21 = x15 * x11;
	const GEN_FLT x22 = x20 * x21;
	const GEN_FLT x23 = x19 + x22;
	const GEN_FLT x24 = x20 * x18;
	const GEN_FLT x25 = x21 * x17;
	const GEN_FLT x26 = (-1 * x24) + x25;
	const GEN_FLT x27 = obj_pz + (x16 * sensor_z) + (x23 * sensor_y) + (x26 * sensor_x);
	const GEN_FLT x28 = x1 * x4;
	const GEN_FLT x29 = x2 * x6;
	const GEN_FLT x30 = x7 * x29;
	const GEN_FLT x31 = (-1 * x28) + x30;
	const GEN_FLT x32 = (-1 * x19) + x22;
	const GEN_FLT x33 = x20 * x20;
	const GEN_FLT x34 = (x33 * x15) + x14;
	const GEN_FLT x35 = x11 * x18;
	const GEN_FLT x36 = x20 * x15;
	const GEN_FLT x37 = x36 * x17;
	const GEN_FLT x38 = x35 + x37;
	const GEN_FLT x39 = obj_py + (x32 * sensor_z) + (x34 * sensor_y) + (x38 * sensor_x);
	const GEN_FLT x40 = x7 * x7;
	const GEN_FLT x41 = (x6 * x40) + x5;
	const GEN_FLT x42 = x24 + x25;
	const GEN_FLT x43 = (-1 * x35) + x37;
	const GEN_FLT x44 = x17 * x17;
	const GEN_FLT x45 = (x44 * x15) + x14;
	const GEN_FLT x46 = obj_px + (x42 * sensor_z) + (x43 * sensor_y) + (x45 * sensor_x);
	const GEN_FLT x47 = lh_px + (x27 * x10) + (x31 * x39) + (x41 * x46);
	const GEN_FLT x48 = pow(x47, -1);
	const GEN_FLT x49 = (-1 * x3) + x9;
	const GEN_FLT x50 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x51 = x15 * x17;
	const GEN_FLT x52 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x53 = x52 * x18;
	const GEN_FLT x54 = -1 * x53;
	const GEN_FLT x55 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x56 = x55 * x18;
	const GEN_FLT x57 = x52 * x14;
	const GEN_FLT x58 = x57 * x11;
	const GEN_FLT x59 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x60 = x52 * x19;
	const GEN_FLT x61 = (x51 * x59) + (x60 * x20) + (x50 * x36);
	const GEN_FLT x62 = x59 * x18;
	const GEN_FLT x63 = x57 * x20;
	const GEN_FLT x64 = (x50 * x21) + (x60 * x11) + (x51 * x55);
	const GEN_FLT x65 = (((2 * x50 * x51) + x54 + (x53 * x44)) * sensor_x) +
						(((-1 * x56) + (-1 * x58) + x61) * sensor_y) + ((x62 + x63 + x64) * sensor_z);
	const GEN_FLT x66 = x45 + x65;
	const GEN_FLT x67 = x1 * x7;
	const GEN_FLT x68 = x4 * x29;
	const GEN_FLT x69 = x67 + x68;
	const GEN_FLT x70 = x50 * x18;
	const GEN_FLT x71 = x57 * x17;
	const GEN_FLT x72 = (x59 * x21) + (x55 * x36) + (x52 * x35 * x20);
	const GEN_FLT x73 = ((x56 + x58 + x61) * sensor_x) + ((x54 + (x53 * x33) + (2 * x59 * x36)) * sensor_y) +
						(((-1 * x70) + (-1 * x71) + x72) * sensor_z);
	const GEN_FLT x74 = x38 + x73;
	const GEN_FLT x75 = x4 * x4;
	const GEN_FLT x76 = (x6 * x75) + x5;
	const GEN_FLT x77 = (((-1 * x62) + (-1 * x63) + x64) * sensor_x) + ((x70 + x71 + x72) * sensor_y) +
						((x54 + (2 * x55 * x21) + (x53 * x12)) * sensor_z);
	const GEN_FLT x78 = x26 + x77;
	const GEN_FLT x79 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x80 = x5 * x79;
	const GEN_FLT x81 = x2 * x80;
	const GEN_FLT x82 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x83 = x1 * x82;
	const GEN_FLT x84 = x7 * x79;
	const GEN_FLT x85 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x86 = x6 * x85;
	const GEN_FLT x87 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x88 = (x84 * x28) + (x4 * x86) + (x8 * x87);
	const GEN_FLT x89 = x7 * x80;
	const GEN_FLT x90 = x1 * x85;
	const GEN_FLT x91 = x4 * x6;
	const GEN_FLT x92 = (x4 * x3 * x79) + (x87 * x29) + (x82 * x91);
	const GEN_FLT x93 = x1 * x79;
	const GEN_FLT x94 = -1 * x93;
	const GEN_FLT x95 = (x46 * ((-1 * x81) + (-1 * x83) + x88)) + (x39 * (x89 + x90 + x92)) +
						(x27 * (x94 + (x75 * x93) + (2 * x87 * x91)));
	const GEN_FLT x96 = (x66 * x49) + (x74 * x69) + (x78 * x76) + x95;
	const GEN_FLT x97 = x4 * x80;
	const GEN_FLT x98 = x1 * x87;
	const GEN_FLT x99 = (x2 * x86) + (x8 * x82) + (x3 * x84);
	const GEN_FLT x100 = (x39 * ((-1 * x97) + (-1 * x98) + x99)) + (x27 * (x81 + x83 + x88)) +
						 (x46 * (x94 + (x93 * x40) + (2 * x7 * x86)));
	const GEN_FLT x101 = (x66 * x41) + x100 + (x74 * x31) + (x78 * x10);
	const GEN_FLT x102 = x47 * x47;
	const GEN_FLT x103 = lh_pz + (x76 * x27) + (x69 * x39) + (x46 * x49);
	const GEN_FLT x104 = x103 * pow(x102, -1);
	const GEN_FLT x105 = (x103 * x103) + x102;
	const GEN_FLT x106 = pow(x105, -1);
	const GEN_FLT x107 = x102 * x106;
	const GEN_FLT x108 = x107 * ((-1 * x96 * x48) + (x101 * x104));
	const GEN_FLT x109 = x28 + x30;
	const GEN_FLT x110 = x2 * x2;
	const GEN_FLT x111 = (x6 * x110) + x5;
	const GEN_FLT x112 = (-1 * x67) + x68;
	const GEN_FLT x113 = (x46 * (x97 + x98 + x99)) + (x39 * (x94 + (x93 * x110) + (2 * x82 * x29))) +
						 (x27 * ((-1 * x89) + (-1 * x90) + x92));
	const GEN_FLT x114 = (x66 * x109) + (x74 * x111) + (x78 * x112) + x113;
	const GEN_FLT x115 = lh_py + (x39 * x111) + (x27 * x112) + (x46 * x109);
	const GEN_FLT x116 = 2 * x115;
	const GEN_FLT x117 = 2 * x47;
	const GEN_FLT x118 = 2 * x103;
	const GEN_FLT x119 = (x101 * x117) + (x96 * x118);
	const GEN_FLT x120 = 0.523598775598299 + tilt_0;
	const GEN_FLT x121 = cos(x120);
	const GEN_FLT x122 = pow(x121, -1);
	const GEN_FLT x123 = x115 * x115;
	const GEN_FLT x124 = x123 + x105;
	const GEN_FLT x125 = 1.0 / 2.0 * x115;
	const GEN_FLT x126 = pow(x124, -3.0 / 2.0) * x122 * x125;
	const GEN_FLT x127 = pow(x124, -1.0 / 2.0) * x122;
	const GEN_FLT x128 = pow((1 + (-1 * pow(x124, -1) * x123 * pow(x121, -2))), -1.0 / 2.0);
	const GEN_FLT x129 = x128 * ((-1 * x126 * ((x114 * x116) + x119)) + (x114 * x127));
	const GEN_FLT x130 = tan(x120);
	const GEN_FLT x131 = pow(x105, -1.0 / 2.0) * x130;
	const GEN_FLT x132 = x115 * x131;
	const GEN_FLT x133 = atan2(-1 * x103, x47);
	const GEN_FLT x134 = ogeeMag_0 + (-1 * asin(x132)) + x133;
	const GEN_FLT x135 = curve_0 + (sin(x134) * ogeePhase_0);
	const GEN_FLT x136 = asin(x115 * x127);
	const GEN_FLT x137 = 8.0108022e-06 * x136;
	const GEN_FLT x138 = -8.0108022e-06 + (-1 * x137);
	const GEN_FLT x139 = 0.0028679863 + (x138 * x136);
	const GEN_FLT x140 = 5.3685255e-06 + (x136 * x139);
	const GEN_FLT x141 = 0.0076069798 + (x136 * x140);
	const GEN_FLT x142 = x136 * x141;
	const GEN_FLT x143 = -8.0108022e-06 + (-1.60216044e-05 * x136);
	const GEN_FLT x144 = (x136 * x143) + x139;
	const GEN_FLT x145 = (x136 * x144) + x140;
	const GEN_FLT x146 = (x136 * x145) + x141;
	const GEN_FLT x147 = x142 + (x136 * x146);
	const GEN_FLT x148 = sin(x120);
	const GEN_FLT x149 = x135 * x148;
	const GEN_FLT x150 = (-1 * x147 * x149) + x121;
	const GEN_FLT x151 = pow(x150, -1);
	const GEN_FLT x152 = 2 * x135 * x142 * x151;
	const GEN_FLT x153 = x129 * x138;
	const GEN_FLT x154 = 2.40324066e-05 * x136;
	const GEN_FLT x155 = (x129 * x139) + (x136 * ((-1 * x129 * x137) + x153));
	const GEN_FLT x156 = (x129 * x140) + (x136 * x155);
	const GEN_FLT x157 = pow((1 + (-1 * x106 * x123 * (x130 * x130))), -1.0 / 2.0);
	const GEN_FLT x158 = pow(x105, -3.0 / 2.0) * x125 * x130;
	const GEN_FLT x159 = (-1 * x119 * x158) + (x114 * x131);
	const GEN_FLT x160 = x108 + (-1 * x157 * x159);
	const GEN_FLT x161 = cos(x134) * ogeePhase_0;
	const GEN_FLT x162 = x148 * x147;
	const GEN_FLT x163 = x161 * x162;
	const GEN_FLT x164 = x136 * x136;
	const GEN_FLT x165 = x164 * x135;
	const GEN_FLT x166 = x165 * x141 * pow(x150, -2);
	const GEN_FLT x167 = x165 * x151;
	const GEN_FLT x168 = x164 * x141 * x151;
	const GEN_FLT x169 = x161 * x168;
	const GEN_FLT x170 = (x167 * x141) + x132;
	const GEN_FLT x171 = pow((1 + (-1 * (x170 * x170))), -1.0 / 2.0);
	const GEN_FLT x172 =
		x108 +
		(-1 * x171 *
		 ((x129 * x152) +
		  (-1 * x166 *
		   ((-1 * x149 *
			 ((x136 * ((x129 * x145) +
					   (x136 * ((x129 * x144) + (x136 * (x153 + (-1 * x129 * x154) + (x129 * x143))) + x155)) + x156)) +
			  (x129 * x146) + (x129 * x141) + (x136 * x156))) +
			(-1 * x160 * x163))) +
		  x159 + (x167 * x156) + (x160 * x169)));
	const GEN_FLT x173 = cos(gibPhase_0 + (-1 * asin(x170)) + x133) * gibMag_0;
	const GEN_FLT x174 = x43 + x65;
	const GEN_FLT x175 = x34 + x73;
	const GEN_FLT x176 = x23 + x77;
	const GEN_FLT x177 = (x49 * x174) + (x69 * x175) + (x76 * x176) + x95;
	const GEN_FLT x178 = (x41 * x174) + (x31 * x175) + (x10 * x176) + x100;
	const GEN_FLT x179 = x107 * ((-1 * x48 * x177) + (x104 * x178));
	const GEN_FLT x180 = (x109 * x174) + (x111 * x175) + (x112 * x176) + x113;
	const GEN_FLT x181 = (x117 * x178) + (x118 * x177);
	const GEN_FLT x182 = x128 * ((-1 * x126 * ((x116 * x180) + x181)) + (x127 * x180));
	const GEN_FLT x183 = x182 * x138;
	const GEN_FLT x184 = (x182 * x139) + (x136 * ((-1 * x182 * x137) + x183));
	const GEN_FLT x185 = (x182 * x140) + (x184 * x136);
	const GEN_FLT x186 = (-1 * x181 * x158) + (x180 * x131);
	const GEN_FLT x187 = x179 + (-1 * x186 * x157);
	const GEN_FLT x188 =
		x179 +
		(-1 * x171 *
		 ((x182 * x152) + (x167 * x185) +
		  (-1 * x166 *
		   ((-1 * x149 *
			 ((x136 * ((x182 * x145) +
					   (x136 * ((x182 * x144) + (x136 * (x183 + (-1 * x182 * x154) + (x182 * x143))) + x184)) + x185)) +
			  (x182 * x146) + (x182 * x141) + (x185 * x136))) +
			(-1 * x163 * x187))) +
		  (x169 * x187) + x186));
	const GEN_FLT x189 = x42 + x65;
	const GEN_FLT x190 = x32 + x73;
	const GEN_FLT x191 = x16 + x77;
	const GEN_FLT x192 = (x49 * x189) + (x69 * x190) + (x76 * x191) + x95;
	const GEN_FLT x193 = (x41 * x189) + (x10 * x191) + (x31 * x190) + x100;
	const GEN_FLT x194 = x107 * ((-1 * x48 * x192) + (x104 * x193));
	const GEN_FLT x195 = (x111 * x190) + (x112 * x191) + (x109 * x189) + x113;
	const GEN_FLT x196 = (x117 * x193) + (x118 * x192);
	const GEN_FLT x197 = x128 * ((-1 * x126 * ((x116 * x195) + x196)) + (x127 * x195));
	const GEN_FLT x198 = x197 * x138;
	const GEN_FLT x199 = (x197 * x139) + (x136 * ((-1 * x197 * x137) + x198));
	const GEN_FLT x200 = (x197 * x140) + (x199 * x136);
	const GEN_FLT x201 = (-1 * x196 * x158) + (x195 * x131);
	const GEN_FLT x202 = x161 * (x194 + (-1 * x201 * x157));
	const GEN_FLT x203 =
		x194 +
		(-1 * x171 *
		 ((x197 * x152) +
		  (-1 * x166 *
		   ((-1 * x149 *
			 ((x136 * ((x197 * x145) +
					   (x136 * ((x197 * x144) + (x136 * (x198 + (-1 * x197 * x154) + (x197 * x143))) + x199)) + x200)) +
			  (x200 * x136) + (x197 * x146) + (x197 * x141))) +
			(-1 * x202 * x162))) +
		  (x202 * x168) + x201 + (x200 * x167)));
	out[0] = (x172 * x173) + x172;
	out[1] = (x173 * x188) + x188;
	out[2] = (x203 * x173) + x203;
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x20 * x11;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x15 * x16;
	const GEN_FLT x24 = x23 * x11;
	const GEN_FLT x25 =
		obj_pz + (((x15 * x12) + x14) * sensor_z) + ((x18 + x21) * sensor_y) + (((-1 * x22) + x24) * sensor_x);
	const GEN_FLT x26 = x1 * x7;
	const GEN_FLT x27 = x4 * x6;
	const GEN_FLT x28 = x2 * x27;
	const GEN_FLT x29 = (-1 * x26) + x28;
	const GEN_FLT x30 = x19 * x19;
	const GEN_FLT x31 = x11 * x17;
	const GEN_FLT x32 = x20 * x16;
	const GEN_FLT x33 =
		obj_py + (((-1 * x18) + x21) * sensor_z) + ((x31 + x32) * sensor_x) + (((x30 * x15) + x14) * sensor_y);
	const GEN_FLT x34 = x4 * x4;
	const GEN_FLT x35 = (x6 * x34) + x5;
	const GEN_FLT x36 = x16 * x16;
	const GEN_FLT x37 =
		obj_px + (((x36 * x15) + x14) * sensor_x) + ((x22 + x24) * sensor_z) + (((-1 * x31) + x32) * sensor_y);
	const GEN_FLT x38 = lh_px + (x25 * x10) + (x33 * x29) + (x35 * x37);
	const GEN_FLT x39 = pow(x38, -1);
	const GEN_FLT x40 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = x2 * x41;
	const GEN_FLT x43 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x44 = x1 * x43;
	const GEN_FLT x45 = x1 * x40;
	const GEN_FLT x46 = x4 * x7;
	const GEN_FLT x47 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x48 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x49 = (x45 * x46) + (x8 * x47) + (x48 * x27);
	const GEN_FLT x50 = x4 * x41;
	const GEN_FLT x51 = x1 * x47;
	const GEN_FLT x52 = x2 * x7;
	const GEN_FLT x53 = x2 * x6;
	const GEN_FLT x54 = (x52 * x45) + (x53 * x48) + (x8 * x43);
	const GEN_FLT x55 = -1 * x45;
	const GEN_FLT x56 = x7 * x7;
	const GEN_FLT x57 = 2 * x8;
	const GEN_FLT x58 = (-1 * x3) + x9;
	const GEN_FLT x59 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x60 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x61 = x60 * x17;
	const GEN_FLT x62 = -1 * x61;
	const GEN_FLT x63 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x64 = x63 * x17;
	const GEN_FLT x65 = x60 * x14;
	const GEN_FLT x66 = x65 * x11;
	const GEN_FLT x67 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x68 = x60 * x18;
	const GEN_FLT x69 = (x67 * x23) + (x68 * x19) + (x59 * x20);
	const GEN_FLT x70 = x67 * x17;
	const GEN_FLT x71 = x65 * x19;
	const GEN_FLT x72 = x15 * x11;
	const GEN_FLT x73 = (x72 * x59) + (x68 * x11) + (x63 * x23);
	const GEN_FLT x74 = (((2 * x59 * x23) + x62 + (x61 * x36)) * sensor_x) +
						(((-1 * x64) + (-1 * x66) + x69) * sensor_y) + ((x70 + x71 + x73) * sensor_z);
	const GEN_FLT x75 = x59 * x17;
	const GEN_FLT x76 = x65 * x16;
	const GEN_FLT x77 = (x72 * x67) + (x60 * x22 * x11) + (x63 * x20);
	const GEN_FLT x78 = ((x64 + x66 + x69) * sensor_x) + ((x62 + (2 * x67 * x20) + (x61 * x30)) * sensor_y) +
						(((-1 * x75) + (-1 * x76) + x77) * sensor_z);
	const GEN_FLT x79 = x1 * x4;
	const GEN_FLT x80 = x2 * x8;
	const GEN_FLT x81 = x79 + x80;
	const GEN_FLT x82 = (x6 * x56) + x5;
	const GEN_FLT x83 = (((-1 * x70) + (-1 * x71) + x73) * sensor_x) + ((x75 + x76 + x77) * sensor_y) +
						((x62 + (2 * x72 * x63) + (x61 * x12)) * sensor_z);
	const GEN_FLT x84 = (x74 * x58) + (x81 * x78) + (x82 * x83);
	const GEN_FLT x85 = (x37 * ((-1 * x42) + (-1 * x44) + x49)) + (x33 * (x50 + x51 + x54)) + x84 +
						(x25 * (x55 + (x56 * x45) + (x57 * x48)));
	const GEN_FLT x86 = -1 * x85 * x39;
	const GEN_FLT x87 = 2 * x27;
	const GEN_FLT x88 = x7 * x41;
	const GEN_FLT x89 = x1 * x48;
	const GEN_FLT x90 = x2 * x4;
	const GEN_FLT x91 = (x53 * x47) + (x43 * x27) + (x90 * x45);
	const GEN_FLT x92 = (x74 * x35) + (x78 * x29) + (x83 * x10);
	const GEN_FLT x93 = (x37 * (x55 + (x45 * x34) + (x87 * x47))) + (x25 * (x42 + x44 + x49)) +
						(x33 * ((-1 * x88) + (-1 * x89) + x91)) + x92;
	const GEN_FLT x94 = 1 + x93;
	const GEN_FLT x95 = x38 * x38;
	const GEN_FLT x96 = lh_pz + (x82 * x25) + (x81 * x33) + (x58 * x37);
	const GEN_FLT x97 = x96 * pow(x95, -1);
	const GEN_FLT x98 = (x96 * x96) + x95;
	const GEN_FLT x99 = pow(x98, -1);
	const GEN_FLT x100 = x99 * x95;
	const GEN_FLT x101 = x100 * (x86 + (x97 * x94));
	const GEN_FLT x102 = (-1 * x79) + x80;
	const GEN_FLT x103 = x2 * x2;
	const GEN_FLT x104 = (x6 * x103) + x5;
	const GEN_FLT x105 = x26 + x28;
	const GEN_FLT x106 = lh_py + (x25 * x102) + (x33 * x104) + (x37 * x105);
	const GEN_FLT x107 = x106 * x106;
	const GEN_FLT x108 = 0.523598775598299 + tilt_0;
	const GEN_FLT x109 = cos(x108);
	const GEN_FLT x110 = x107 + x98;
	const GEN_FLT x111 = pow((1 + (-1 * pow(x109, -2) * x107 * pow(x110, -1))), -1.0 / 2.0);
	const GEN_FLT x112 = 2 * x53;
	const GEN_FLT x113 = (x74 * x105) + (x78 * x104) + (x83 * x102);
	const GEN_FLT x114 = (x37 * (x88 + x89 + x91)) + (x33 * (x55 + (x45 * x103) + (x43 * x112))) +
						 (x25 * ((-1 * x50) + (-1 * x51) + x54)) + x113;
	const GEN_FLT x115 = 2 * x106;
	const GEN_FLT x116 = x114 * x115;
	const GEN_FLT x117 = 2 * x38;
	const GEN_FLT x118 = 2 * x96;
	const GEN_FLT x119 = x85 * x118;
	const GEN_FLT x120 = (x94 * x117) + x119;
	const GEN_FLT x121 = pow(x109, -1);
	const GEN_FLT x122 = 1.0 / 2.0 * x106;
	const GEN_FLT x123 = pow(x110, -3.0 / 2.0) * x122 * x121;
	const GEN_FLT x124 = pow(x110, -1.0 / 2.0) * x121;
	const GEN_FLT x125 = x114 * x124;
	const GEN_FLT x126 = (-1 * (x116 + x120) * x123) + x125;
	const GEN_FLT x127 = x111 * x126;
	const GEN_FLT x128 = tan(x108);
	const GEN_FLT x129 = pow(x98, -1.0 / 2.0) * x128;
	const GEN_FLT x130 = x106 * x129;
	const GEN_FLT x131 = atan2(-1 * x96, x38);
	const GEN_FLT x132 = ogeeMag_0 + (-1 * asin(x130)) + x131;
	const GEN_FLT x133 = curve_0 + (sin(x132) * ogeePhase_0);
	const GEN_FLT x134 = asin(x106 * x124);
	const GEN_FLT x135 = 8.0108022e-06 * x134;
	const GEN_FLT x136 = -8.0108022e-06 + (-1 * x135);
	const GEN_FLT x137 = 0.0028679863 + (x134 * x136);
	const GEN_FLT x138 = 5.3685255e-06 + (x134 * x137);
	const GEN_FLT x139 = 0.0076069798 + (x134 * x138);
	const GEN_FLT x140 = x134 * x139;
	const GEN_FLT x141 = -8.0108022e-06 + (-1.60216044e-05 * x134);
	const GEN_FLT x142 = (x134 * x141) + x137;
	const GEN_FLT x143 = (x134 * x142) + x138;
	const GEN_FLT x144 = (x134 * x143) + x139;
	const GEN_FLT x145 = x140 + (x134 * x144);
	const GEN_FLT x146 = sin(x108);
	const GEN_FLT x147 = x133 * x146;
	const GEN_FLT x148 = (-1 * x145 * x147) + x109;
	const GEN_FLT x149 = pow(x148, -1);
	const GEN_FLT x150 = 2 * x133 * x140 * x149;
	const GEN_FLT x151 = x111 * x144;
	const GEN_FLT x152 = x111 * x143;
	const GEN_FLT x153 = x111 * x142;
	const GEN_FLT x154 = x111 * x136;
	const GEN_FLT x155 = x126 * x154;
	const GEN_FLT x156 = 2.40324066e-05 * x134;
	const GEN_FLT x157 = x111 * x137;
	const GEN_FLT x158 = (x126 * x157) + (x134 * ((-1 * x127 * x135) + x155));
	const GEN_FLT x159 = (x127 * x138) + (x134 * x158);
	const GEN_FLT x160 = x111 * x139;
	const GEN_FLT x161 = pow((1 + (-1 * x99 * x107 * (x128 * x128))), -1.0 / 2.0);
	const GEN_FLT x162 = pow(x98, -3.0 / 2.0) * x122 * x128;
	const GEN_FLT x163 = x114 * x129;
	const GEN_FLT x164 = (-1 * x120 * x162) + x163;
	const GEN_FLT x165 = x101 + (-1 * x161 * x164);
	const GEN_FLT x166 = cos(x132) * ogeePhase_0;
	const GEN_FLT x167 = x146 * x145;
	const GEN_FLT x168 = x167 * x166;
	const GEN_FLT x169 = x134 * x134;
	const GEN_FLT x170 = x169 * x133;
	const GEN_FLT x171 = x170 * x139 * pow(x148, -2);
	const GEN_FLT x172 = x170 * x149;
	const GEN_FLT x173 = x169 * x139 * x149;
	const GEN_FLT x174 = x166 * x173;
	const GEN_FLT x175 = (x172 * x139) + x130;
	const GEN_FLT x176 = pow((1 + (-1 * (x175 * x175))), -1.0 / 2.0);
	const GEN_FLT x177 =
		x101 +
		(-1 * x176 *
		 ((x127 * x150) + (x165 * x174) +
		  (-1 * x171 *
		   ((-1 * x147 *
			 ((x126 * x151) + (x126 * x160) +
			  (x134 * ((x126 * x152) +
					   (x134 * ((x126 * x153) + (x134 * (x155 + (-1 * x127 * x156) + (x127 * x141))) + x158)) + x159)) +
			  (x134 * x159))) +
			(-1 * x168 * x165))) +
		  (x172 * x159) + x164));
	const GEN_FLT x178 = cos(gibPhase_0 + (-1 * asin(x175)) + x131) * gibMag_0;
	const GEN_FLT x179 = x93 * x97;
	const GEN_FLT x180 = x100 * (x86 + x179);
	const GEN_FLT x181 = 1 + x114;
	const GEN_FLT x182 = x93 * x117;
	const GEN_FLT x183 = x182 + x119;
	const GEN_FLT x184 = (-1 * x123 * ((x115 * x181) + x183)) + (x124 * x181);
	const GEN_FLT x185 = x111 * x184;
	const GEN_FLT x186 = x184 * x154;
	const GEN_FLT x187 = (x185 * x137) + (x134 * ((-1 * x185 * x135) + x186));
	const GEN_FLT x188 = (x185 * x138) + (x187 * x134);
	const GEN_FLT x189 = (-1 * x162 * x183) + (x129 * x181);
	const GEN_FLT x190 = x180 + (-1 * x161 * x189);
	const GEN_FLT x191 =
		x180 +
		(-1 * x176 *
		 ((x185 * x150) +
		  (-1 * x171 *
		   ((-1 * x147 *
			 ((x185 * x144) +
			  (x134 * ((x185 * x143) +
					   (x134 * ((x185 * x142) + (x134 * (x186 + (-1 * x185 * x156) + (x185 * x141))) + x187)) + x188)) +
			  (x160 * x184) + (x188 * x134))) +
			(-1 * x168 * x190))) +
		  (x174 * x190) + (x172 * x188) + x189));
	const GEN_FLT x192 = 1 + x85;
	const GEN_FLT x193 = x100 * ((-1 * x39 * x192) + x179);
	const GEN_FLT x194 = x182 + (x118 * x192);
	const GEN_FLT x195 = (-1 * (x116 + x194) * x123) + x125;
	const GEN_FLT x196 = x111 * x195;
	const GEN_FLT x197 = x195 * x154;
	const GEN_FLT x198 = (x195 * x157) + (x134 * ((-1 * x196 * x135) + x197));
	const GEN_FLT x199 = (x196 * x138) + (x198 * x134);
	const GEN_FLT x200 = (-1 * x162 * x194) + x163;
	const GEN_FLT x201 = x166 * (x193 + (-1 * x200 * x161));
	const GEN_FLT x202 =
		x193 +
		(-1 * x176 *
		 ((x196 * x150) + (x201 * x173) +
		  (-1 * x171 *
		   ((-1 * x147 *
			 ((x195 * x151) + (x160 * x195) +
			  (x134 * ((x195 * x152) +
					   (x134 * ((x195 * x153) + (x134 * (x197 + (-1 * x196 * x156) + (x196 * x141))) + x198)) + x199)) +
			  (x199 * x134))) +
			(-1 * x201 * x167))) +
		  (x172 * x199) + x200));
	const GEN_FLT x203 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x204 = x5 * x203;
	const GEN_FLT x205 = x2 * x204;
	const GEN_FLT x206 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qj *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x207 = x1 * x206;
	const GEN_FLT x208 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qi *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x209 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qk *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x210 = x1 * x203;
	const GEN_FLT x211 = (x8 * x208) + (x27 * x209) + (x46 * x210);
	const GEN_FLT x212 = x4 * x204;
	const GEN_FLT x213 = x1 * x208;
	const GEN_FLT x214 = x2 * x210;
	const GEN_FLT x215 = (x7 * x214) + (x53 * x209) + (x8 * x206);
	const GEN_FLT x216 = -1 * x210;
	const GEN_FLT x217 = (x37 * ((-1 * x205) + (-1 * x207) + x211)) + (x33 * (x212 + x213 + x215)) + x84 +
						 (x25 * (x216 + (x56 * x210) + (x57 * x209)));
	const GEN_FLT x218 = x7 * x204;
	const GEN_FLT x219 = x1 * x209;
	const GEN_FLT x220 = (x4 * x214) + (x53 * x208) + (x27 * x206);
	const GEN_FLT x221 = (x37 * (x216 + (x34 * x210) + (x87 * x208))) + (x33 * ((-1 * x218) + x220 + (-1 * x219))) +
						 (x25 * (x205 + x207 + x211)) + x92;
	const GEN_FLT x222 = ((-1 * x39 * x217) + (x97 * x221)) * x100;
	const GEN_FLT x223 = (x33 * (x216 + (x210 * x103) + (x206 * x112))) + (x25 * ((-1 * x212) + (-1 * x213) + x215)) +
						 x113 + (x37 * (x218 + x220 + x219));
	const GEN_FLT x224 = (x221 * x117) + (x217 * x118);
	const GEN_FLT x225 = (-1 * x123 * ((x223 * x115) + x224)) + (x223 * x124);
	const GEN_FLT x226 = x225 * x111;
	const GEN_FLT x227 = x225 * x154;
	const GEN_FLT x228 = (x225 * x157) + (x134 * ((-1 * x226 * x135) + x227));
	const GEN_FLT x229 = (x226 * x138) + (x228 * x134);
	const GEN_FLT x230 = (-1 * x224 * x162) + (x223 * x129);
	const GEN_FLT x231 = x222 + (-1 * x230 * x161);
	const GEN_FLT x232 =
		x222 +
		(-1 * x176 *
		 ((x226 * x150) + (x231 * x174) +
		  (-1 * x171 *
		   ((-1 * x147 *
			 ((x225 * x151) +
			  (x134 * ((x225 * x152) +
					   (x134 * ((x225 * x153) + (x134 * (x227 + (-1 * x226 * x156) + (x226 * x141))) + x228)) + x229)) +
			  (x225 * x160) + (x229 * x134))) +
			(-1 * x231 * x168))) +
		  x230 + (x229 * x172)));
	const GEN_FLT x233 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x234 = x5 * x233;
	const GEN_FLT x235 = x2 * x234;
	const GEN_FLT x236 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qj *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x237 = x1 * x236;
	const GEN_FLT x238 = x1 * x233;
	const GEN_FLT x239 = x7 * x238;
	const GEN_FLT x240 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qi *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x241 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qk *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x242 = (x4 * x239) + (x8 * x240) + (x27 * x241);
	const GEN_FLT x243 = x4 * x234;
	const GEN_FLT x244 = x1 * x240;
	const GEN_FLT x245 = (x2 * x239) + (x8 * x236) + (x53 * x241);
	const GEN_FLT x246 = -1 * x238;
	const GEN_FLT x247 = (x37 * ((-1 * x235) + (-1 * x237) + x242)) + (x33 * (x243 + x244 + x245)) +
						 (x25 * (x246 + (x56 * x238) + (x57 * x241))) + x84;
	const GEN_FLT x248 = x7 * x234;
	const GEN_FLT x249 = x1 * x241;
	const GEN_FLT x250 = (x90 * x238) + (x27 * x236) + (x53 * x240);
	const GEN_FLT x251 = (x37 * ((x34 * x238) + x246 + (x87 * x240))) + (x33 * ((-1 * x248) + (-1 * x249) + x250)) +
						 x92 + (x25 * (x235 + x237 + x242));
	const GEN_FLT x252 = ((-1 * x39 * x247) + (x97 * x251)) * x100;
	const GEN_FLT x253 = (x37 * (x248 + x249 + x250)) + (x33 * (x246 + (x238 * x103) + (x236 * x112))) +
						 (x25 * ((-1 * x243) + (-1 * x244) + x245)) + x113;
	const GEN_FLT x254 = (x251 * x117) + (x247 * x118);
	const GEN_FLT x255 = x111 * ((-1 * x123 * ((x253 * x115) + x254)) + (x253 * x124));
	const GEN_FLT x256 = x255 * x136;
	const GEN_FLT x257 = (x255 * x137) + (x134 * ((-1 * x255 * x135) + x256));
	const GEN_FLT x258 = (x255 * x138) + (x257 * x134);
	const GEN_FLT x259 = (-1 * x254 * x162) + (x253 * x129);
	const GEN_FLT x260 = x252 + (-1 * x259 * x161);
	const GEN_FLT x261 =
		x252 +
		(-1 * x176 *
		 ((x255 * x150) +
		  (-1 * x171 *
		   ((-1 * x147 *
			 ((x255 * x144) +
			  (x134 * ((x255 * x143) +
					   (x134 * ((x255 * x142) + (x134 * (x256 + (-1 * x255 * x156) + (x255 * x141))) + x257)) + x258)) +
			  (x255 * x139) + (x258 * x134))) +
			(-1 * x260 * x168))) +
		  (x258 * x172) + (x260 * x174) + x259));
	const GEN_FLT x262 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x263 = x5 * x262;
	const GEN_FLT x264 = x2 * x263;
	const GEN_FLT x265 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qj *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x266 = x1 * x265;
	const GEN_FLT x267 = x1 * x262;
	const GEN_FLT x268 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qi *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x269 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qk *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x270 = (x46 * x267) + (x27 * x269) + (x8 * x268);
	const GEN_FLT x271 = x4 * x263;
	const GEN_FLT x272 = x1 * x268;
	const GEN_FLT x273 = (x52 * x267) + (x53 * x269) + (x8 * x265);
	const GEN_FLT x274 = -1 * x267;
	const GEN_FLT x275 = (x37 * ((-1 * x264) + (-1 * x266) + x270)) + (x33 * (x271 + x272 + x273)) +
						 (x25 * (x274 + (x56 * x267) + (x57 * x269))) + x84;
	const GEN_FLT x276 = x7 * x263;
	const GEN_FLT x277 = x1 * x269;
	const GEN_FLT x278 = (x90 * x267) + (x53 * x268) + (x27 * x265);
	const GEN_FLT x279 = (x37 * (x274 + (x34 * x267) + (x87 * x268))) + (x33 * ((-1 * x276) + (-1 * x277) + x278)) +
						 (x25 * (x264 + x266 + x270)) + x92;
	const GEN_FLT x280 = ((-1 * x39 * x275) + (x97 * x279)) * x100;
	const GEN_FLT x281 = (x37 * (x276 + x277 + x278)) + (x33 * (x274 + (x265 * x112) + (x267 * x103))) +
						 (x25 * ((-1 * x271) + (-1 * x272) + x273)) + x113;
	const GEN_FLT x282 = (x279 * x117) + (x275 * x118);
	const GEN_FLT x283 = (-1 * x123 * ((x281 * x115) + x282)) + (x281 * x124);
	const GEN_FLT x284 = x283 * x111;
	const GEN_FLT x285 = x283 * x154;
	const GEN_FLT x286 = (x283 * x157) + (x134 * ((-1 * x284 * x135) + x285));
	const GEN_FLT x287 = (x284 * x138) + (x286 * x134);
	const GEN_FLT x288 = (-1 * x282 * x162) + (x281 * x129);
	const GEN_FLT x289 = x280 + (-1 * x288 * x161);
	const GEN_FLT x290 =
		x280 + (-1 * x176 *
				((x284 * x150) +
				 (-1 * x171 *
				  ((-1 * x147 *
					((x283 * x151) +
					 (x134 * ((x283 * x152) + x287 +
							  (x134 * ((x283 * x153) + (x134 * (x285 + (-1 * x284 * x156) + (x284 * x141))) + x286)))) +
					 (x283 * x160) + (x287 * x134))) +
				   (-1 * x289 * x168))) +
				 (x289 * x174) + (x287 * x172) + x288));
	out[0] = (x178 * x177) + x177;
	out[1] = (x178 * x191) + x191;
	out[2] = (x202 * x178) + x202;
	out[3] = (x232 * x178) + x232;
	out[4] = (x261 * x178) + x261;
	out[5] = (x290 * x178) + x290;
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x5 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x20 * x11;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x15 * x16;
	const GEN_FLT x24 = x23 * x11;
	const GEN_FLT x25 =
		obj_pz + (((x15 * x12) + x14) * sensor_z) + ((x18 + x21) * sensor_y) + (((-1 * x22) + x24) * sensor_x);
	const GEN_FLT x26 = x1 * x5;
	const GEN_FLT x27 = x4 * x7;
	const GEN_FLT x28 = x2 * x27;
	const GEN_FLT x29 = (-1 * x26) + x28;
	const GEN_FLT x30 = x19 * x19;
	const GEN_FLT x31 = x11 * x17;
	const GEN_FLT x32 = x20 * x16;
	const GEN_FLT x33 =
		obj_py + (((-1 * x18) + x21) * sensor_z) + ((x31 + x32) * sensor_x) + (((x30 * x15) + x14) * sensor_y);
	const GEN_FLT x34 = x4 * x4;
	const GEN_FLT x35 = (x7 * x34) + x6;
	const GEN_FLT x36 = x16 * x16;
	const GEN_FLT x37 =
		obj_px + (((x36 * x15) + x14) * sensor_x) + ((x22 + x24) * sensor_z) + (((-1 * x31) + x32) * sensor_y);
	const GEN_FLT x38 = lh_px + (x25 * x10) + (x33 * x29) + (x35 * x37);
	const GEN_FLT x39 = x38 * x38;
	const GEN_FLT x40 = x5 * x5;
	const GEN_FLT x41 = (x7 * x40) + x6;
	const GEN_FLT x42 = x1 * x4;
	const GEN_FLT x43 = x2 * x8;
	const GEN_FLT x44 = x42 + x43;
	const GEN_FLT x45 = (-1 * x3) + x9;
	const GEN_FLT x46 = lh_pz + (x41 * x25) + (x44 * x33) + (x45 * x37);
	const GEN_FLT x47 = (x46 * x46) + x39;
	const GEN_FLT x48 = pow(x47, -1);
	const GEN_FLT x49 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x50 = x6 * x49;
	const GEN_FLT x51 = x2 * x50;
	const GEN_FLT x52 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x53 = x1 * x52;
	const GEN_FLT x54 = x4 * x49;
	const GEN_FLT x55 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x56 = x7 * x55;
	const GEN_FLT x57 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x58 = (x54 * x26) + (x5 * x56) + (x57 * x27);
	const GEN_FLT x59 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x60 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x61 = x60 * x17;
	const GEN_FLT x62 = -1 * x61;
	const GEN_FLT x63 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x64 = x63 * x17;
	const GEN_FLT x65 = x60 * x14;
	const GEN_FLT x66 = x65 * x11;
	const GEN_FLT x67 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x68 = x60 * x18;
	const GEN_FLT x69 = (x67 * x23) + (x68 * x19) + (x59 * x20);
	const GEN_FLT x70 = x67 * x17;
	const GEN_FLT x71 = x65 * x19;
	const GEN_FLT x72 = x15 * x11;
	const GEN_FLT x73 = (x72 * x59) + (x68 * x11) + (x63 * x23);
	const GEN_FLT x74 = (((2 * x59 * x23) + x62 + (x61 * x36)) * sensor_x) +
						(((-1 * x64) + (-1 * x66) + x69) * sensor_y) + ((x70 + x71 + x73) * sensor_z);
	const GEN_FLT x75 = x4 * x50;
	const GEN_FLT x76 = x1 * x55;
	const GEN_FLT x77 = x2 * x7;
	const GEN_FLT x78 = (x3 * x5 * x49) + (x77 * x57) + (x8 * x52);
	const GEN_FLT x79 = x59 * x17;
	const GEN_FLT x80 = x65 * x16;
	const GEN_FLT x81 = (x72 * x67) + (x63 * x20) + (x60 * x31 * x19);
	const GEN_FLT x82 = ((x64 + x66 + x69) * sensor_x) + ((x62 + (2 * x67 * x20) + (x61 * x30)) * sensor_y) +
						(((-1 * x79) + (-1 * x80) + x81) * sensor_z);
	const GEN_FLT x83 = x1 * x49;
	const GEN_FLT x84 = -1 * x83;
	const GEN_FLT x85 = (((-1 * x70) + (-1 * x71) + x73) * sensor_x) + ((x79 + x80 + x81) * sensor_y) +
						((x62 + (2 * x72 * x63) + (x61 * x12)) * sensor_z);
	const GEN_FLT x86 = (x37 * ((-1 * x51) + (-1 * x53) + x58)) + (x74 * x45) + (x33 * (x75 + x76 + x78)) +
						(x82 * x44) + (x25 * (x84 + (2 * x8 * x57) + (x83 * x40))) + (x85 * x41);
	const GEN_FLT x87 = x5 * x50;
	const GEN_FLT x88 = x1 * x57;
	const GEN_FLT x89 = (x2 * x56) + (x52 * x27) + (x3 * x54);
	const GEN_FLT x90 = (x37 * (x84 + (x83 * x34) + (2 * x4 * x56))) + (x74 * x35) +
						(x33 * ((-1 * x87) + (-1 * x88) + x89)) + (x82 * x29) + (x25 * (x51 + x53 + x58)) + (x85 * x10);
	const GEN_FLT x91 = x48 * x39 * ((-1 * x86 * pow(x38, -1)) + (x90 * x46 * pow(x39, -1)));
	const GEN_FLT x92 = (-1 * x42) + x43;
	const GEN_FLT x93 = x2 * x2;
	const GEN_FLT x94 = (x7 * x93) + x6;
	const GEN_FLT x95 = x26 + x28;
	const GEN_FLT x96 = lh_py + (x92 * x25) + (x94 * x33) + (x95 * x37);
	const GEN_FLT x97 = 0.523598775598299 + tilt_0;
	const GEN_FLT x98 = cos(x97);
	const GEN_FLT x99 = pow(x98, -1);
	const GEN_FLT x100 = x96 * x96;
	const GEN_FLT x101 = x100 + x47;
	const GEN_FLT x102 = pow(x101, -1.0 / 2.0);
	const GEN_FLT x103 = x99 * x102;
	const GEN_FLT x104 = asin(x96 * x103);
	const GEN_FLT x105 = -8.0108022e-06 + (-1.60216044e-05 * x104);
	const GEN_FLT x106 = 8.0108022e-06 * x104;
	const GEN_FLT x107 = -8.0108022e-06 + (-1 * x106);
	const GEN_FLT x108 = 0.0028679863 + (x104 * x107);
	const GEN_FLT x109 = (x105 * x104) + x108;
	const GEN_FLT x110 = 5.3685255e-06 + (x108 * x104);
	const GEN_FLT x111 = (x109 * x104) + x110;
	const GEN_FLT x112 = 0.0076069798 + (x104 * x110);
	const GEN_FLT x113 = (x104 * x111) + x112;
	const GEN_FLT x114 = pow(x98, -2);
	const GEN_FLT x115 = pow((1 + (-1 * pow(x101, -1) * x100 * x114)), -1.0 / 2.0);
	const GEN_FLT x116 = (x37 * (x87 + x88 + x89)) + (x74 * x95) + (x33 * (x84 + (x83 * x93) + (2 * x77 * x52))) +
						 (x82 * x94) + (x25 * ((-1 * x75) + (-1 * x76) + x78)) + (x85 * x92);
	const GEN_FLT x117 = (2 * x90 * x38) + (2 * x86 * x46);
	const GEN_FLT x118 = 1.0 / 2.0 * x96;
	const GEN_FLT x119 = (-1 * x99 * pow(x101, -3.0 / 2.0) * x118 * ((2 * x96 * x116) + x117)) + (x103 * x116);
	const GEN_FLT x120 = x119 * x115;
	const GEN_FLT x121 = x107 * x120;
	const GEN_FLT x122 = 2.40324066e-05 * x104;
	const GEN_FLT x123 = (x108 * x120) + (x104 * ((-1 * x106 * x120) + x121));
	const GEN_FLT x124 = (x110 * x120) + (x104 * x123);
	const GEN_FLT x125 = sin(x97);
	const GEN_FLT x126 = tan(x97);
	const GEN_FLT x127 = pow(x47, -1.0 / 2.0);
	const GEN_FLT x128 = x96 * x127;
	const GEN_FLT x129 = x128 * x126;
	const GEN_FLT x130 = atan2(-1 * x46, x38);
	const GEN_FLT x131 = ogeeMag_0 + (-1 * asin(x129)) + x130;
	const GEN_FLT x132 = sin(x131);
	const GEN_FLT x133 = curve_0 + (x132 * ogeePhase_0);
	const GEN_FLT x134 = x125 * x133;
	const GEN_FLT x135 =
		-1 * x134 *
		((x113 * x120) +
		 (x104 * ((x111 * x120) +
				  (x104 * ((x109 * x120) + (x104 * (x121 + (-1 * x120 * x122) + (x105 * x120))) + x123)) + x124)) +
		 (x112 * x120) + (x104 * x124));
	const GEN_FLT x136 = x104 * x112;
	const GEN_FLT x137 = x136 + (x104 * x113);
	const GEN_FLT x138 = x125 * x137;
	const GEN_FLT x139 = x126 * x126;
	const GEN_FLT x140 = pow((1 + (-1 * x48 * x100 * x139)), -1.0 / 2.0);
	const GEN_FLT x141 = (-1 * pow(x47, -3.0 / 2.0) * x118 * x117 * x126) + (x116 * x127 * x126);
	const GEN_FLT x142 = x91 + (-1 * x140 * x141);
	const GEN_FLT x143 = cos(x131) * ogeePhase_0;
	const GEN_FLT x144 = x142 * x143;
	const GEN_FLT x145 = x104 * x104;
	const GEN_FLT x146 = (-1 * x133 * x138) + x98;
	const GEN_FLT x147 = x112 * x133 * pow(x146, -2) * x145;
	const GEN_FLT x148 = pow(x146, -1);
	const GEN_FLT x149 = x145 * x148;
	const GEN_FLT x150 = x112 * x149;
	const GEN_FLT x151 = 2 * x133 * x136 * x148;
	const GEN_FLT x152 = x133 * x149;
	const GEN_FLT x153 = (x120 * x151) + (x124 * x152) + x141;
	const GEN_FLT x154 = (x112 * x152) + x129;
	const GEN_FLT x155 = pow((1 + (-1 * (x154 * x154))), -1.0 / 2.0);
	const GEN_FLT x156 = x91 + (-1 * x155 * ((-1 * x147 * (x135 + (-1 * x138 * x144))) + (x144 * x150) + x153));
	const GEN_FLT x157 = gibPhase_0 + (-1 * asin(x154)) + x130;
	const GEN_FLT x158 = cos(x157) * gibMag_0;
	const GEN_FLT x159 = (x156 * x158) + x156;
	const GEN_FLT x160 = x115 * ((x96 * x102 * x114 * x125) + x119);
	const GEN_FLT x161 = x107 * x160;
	const GEN_FLT x162 = (x108 * x160) + (x104 * ((-1 * x106 * x160) + x161));
	const GEN_FLT x163 = (x110 * x160) + (x104 * x162);
	const GEN_FLT x164 = (x128 * (1 + x139)) + x141;
	const GEN_FLT x165 = x91 + (-1 * x164 * x140);
	const GEN_FLT x166 = x138 * x143;
	const GEN_FLT x167 = x143 * x150;
	const GEN_FLT x168 =
		x91 +
		(-1 * x155 *
		 ((x160 * x151) +
		  (-1 * x147 *
		   ((-1 * x125) +
			(-1 * x134 *
			 ((x113 * x160) + (x104 * x163) +
			  (x104 * ((x111 * x160) +
					   (x104 * ((x109 * x160) + (x104 * (x161 + (x105 * x160) + (-1 * x122 * x160))) + x162)) + x163)) +
			  (x112 * x160))) +
			(-1 * x166 * x165) + (-1 * x98 * x133 * x137))) +
		  (x167 * x165) + (x163 * x152) + x164));
	const GEN_FLT x169 = 1 + x144;
	const GEN_FLT x170 = x91 + (-1 * x155 * ((-1 * x147 * (x135 + (-1 * x169 * x138))) + (x169 * x150) + x153));
	const GEN_FLT x171 = 1 + x142;
	const GEN_FLT x172 = x91 + (-1 * x155 * ((-1 * x147 * (x135 + (-1 * x166 * x171))) + (x167 * x171) + x153));
	const GEN_FLT x173 = x144 + x132;
	const GEN_FLT x174 = x91 + (-1 * x155 * ((-1 * x147 * (x135 + (-1 * x173 * x138))) + (x173 * x150) + x153));
	out[0] = -1 + x159;
	out[1] = (x168 * x158) + x168;
	out[2] = (x170 * x158) + x170;
	out[3] = (x158 * (1 + x156)) + x156;
	out[4] = sin(x157) + x159;
	out[5] = (x172 * x158) + x172;
	out[6] = (x174 * x158) + x174;
}

/** Applying function <function reproject_axis_y_gen2 at 0x7f35a4af89e0> */
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 + (-1 * x1);
	const GEN_FLT x3 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x4 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 0;
	const GEN_FLT x5 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   : 1e-10;
	const GEN_FLT x6 = cos(x5);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 1;
	const GEN_FLT x9 = sin(x5);
	const GEN_FLT x10 = x8 * x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x7 * x11;
	const GEN_FLT x13 = x4 * x12;
	const GEN_FLT x14 = x9 * x11;
	const GEN_FLT x15 = x4 * x8 * x7;
	const GEN_FLT x16 =
		obj_pz + ((((x4 * x4) * x7) + x6) * sensor_z) + ((x10 + x13) * sensor_y) + (((-1 * x14) + x15) * sensor_x);
	const GEN_FLT x17 = sin(x0);
	const GEN_FLT x18 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 1;
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 0;
	const GEN_FLT x21 = x2 * x20;
	const GEN_FLT x22 = x3 * x21;
	const GEN_FLT x23 = x4 * x9;
	const GEN_FLT x24 = x8 * x12;
	const GEN_FLT x25 =
		obj_py + (((-1 * x10) + x13) * sensor_z) + (((x7 * (x11 * x11)) + x6) * sensor_y) + ((x23 + x24) * sensor_x);
	const GEN_FLT x26 = x20 * x17;
	const GEN_FLT x27 = x2 * x3 * x18;
	const GEN_FLT x28 =
		obj_px + ((x14 + x15) * sensor_z) + (((-1 * x23) + x24) * sensor_y) + ((((x8 * x8) * x7) + x6) * sensor_x);
	const GEN_FLT x29 = lh_pz + (x16 * ((x2 * (x3 * x3)) + x1)) + ((x19 + x22) * x25) + (((-1 * x26) + x27) * x28);
	const GEN_FLT x30 = x3 * x17;
	const GEN_FLT x31 = x21 * x18;
	const GEN_FLT x32 = lh_px + ((x26 + x27) * x16) + (((-1 * x30) + x31) * x25) + (x28 * ((x2 * (x18 * x18)) + x1));
	const GEN_FLT x33 = atan2(-1 * x29, x32);
	const GEN_FLT x34 = lh_py + (((-1 * x19) + x22) * x16) + (x25 * ((x2 * (x20 * x20)) + x1)) + ((x30 + x31) * x28);
	const GEN_FLT x35 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x36 = (x29 * x29) + (x32 * x32);
	const GEN_FLT x37 = -1 * x34 * pow(x36, -1.0 / 2.0) * tan(x35);
	const GEN_FLT x38 = curve_1 + (sin(ogeeMag_1 + (-1 * asin(x37)) + x33) * ogeePhase_1);
	const GEN_FLT x39 = cos(x35);
	const GEN_FLT x40 = asin(x34 * pow(x39, -1) * pow(((x34 * x34) + x36), -1.0 / 2.0));
	const GEN_FLT x41 = 0.0028679863 + (x40 * (-8.0108022e-06 + (-8.0108022e-06 * x40)));
	const GEN_FLT x42 = 5.3685255e-06 + (x40 * x41);
	const GEN_FLT x43 = 0.0076069798 + (x40 * x42);
	const GEN_FLT x44 =
		x33 +
		(-1 * asin(((x40 * x40) * x43 * x38 *
					pow(((x38 * sin(x35) *
						  ((x40 * x43) +
						   (x40 * ((x40 * ((x40 * ((x40 * (-8.0108022e-06 + (-1.60216044e-05 * x40))) + x41)) + x42)) +
								   x43)))) +
						 x39),
						-1)) +
				   x37));
	out[0] = -1.5707963267949 + (-1 * phase_1) + (sin(gibPhase_1 + x44) * gibMag_1) + x44;
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x20 = x15 * x11;
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x15 * x16;
	const GEN_FLT x24 = x23 * x11;
	const GEN_FLT x25 =
		obj_pz + (((x15 * x12) + x14) * sensor_z) + ((x18 + x21) * sensor_y) + (((-1 * x22) + x24) * sensor_x);
	const GEN_FLT x26 = x1 * x7;
	const GEN_FLT x27 = x2 * x6;
	const GEN_FLT x28 = x4 * x27;
	const GEN_FLT x29 = (-1 * x26) + x28;
	const GEN_FLT x30 = x19 * x19;
	const GEN_FLT x31 = x11 * x17;
	const GEN_FLT x32 = x23 * x19;
	const GEN_FLT x33 =
		obj_py + (((-1 * x18) + x21) * sensor_z) + ((x31 + x32) * sensor_x) + (((x30 * x15) + x14) * sensor_y);
	const GEN_FLT x34 = x4 * x4;
	const GEN_FLT x35 = (x6 * x34) + x5;
	const GEN_FLT x36 = x16 * x16;
	const GEN_FLT x37 =
		obj_px + (((x36 * x15) + x14) * sensor_x) + ((x22 + x24) * sensor_z) + (((-1 * x31) + x32) * sensor_y);
	const GEN_FLT x38 = lh_px + (x25 * x10) + (x33 * x29) + (x35 * x37);
	const GEN_FLT x39 = pow(x38, -1);
	const GEN_FLT x40 = x7 * x7;
	const GEN_FLT x41 = (x6 * x40) + x5;
	const GEN_FLT x42 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x43 = x42 * x17;
	const GEN_FLT x44 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x45 = x44 * x14;
	const GEN_FLT x46 = x45 * x19;
	const GEN_FLT x47 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x48 = x44 * x18;
	const GEN_FLT x49 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x50 = (x47 * x20) + (x48 * x11) + (x49 * x23);
	const GEN_FLT x51 = x47 * x17;
	const GEN_FLT x52 = x45 * x16;
	const GEN_FLT x53 = x15 * x19;
	const GEN_FLT x54 = (x42 * x20) + (x53 * x49) + (x44 * x22 * x11);
	const GEN_FLT x55 = x44 * x17;
	const GEN_FLT x56 = -1 * x55;
	const GEN_FLT x57 = 2 * x20;
	const GEN_FLT x58 = (((-1 * x43) + (-1 * x46) + x50) * sensor_x) + ((x51 + x52 + x54) * sensor_y) +
						((x56 + (x57 * x49) + (x55 * x12)) * sensor_z);
	const GEN_FLT x59 = x58 * x41;
	const GEN_FLT x60 = x49 * x17;
	const GEN_FLT x61 = x14 * x11;
	const GEN_FLT x62 = x61 * x44;
	const GEN_FLT x63 = (x42 * x23) + (x48 * x19) + (x53 * x47);
	const GEN_FLT x64 = 2 * x53;
	const GEN_FLT x65 = ((x60 + x62 + x63) * sensor_x) + ((x56 + (x64 * x42) + (x55 * x30)) * sensor_y) +
						(((-1 * x51) + (-1 * x52) + x54) * sensor_z);
	const GEN_FLT x66 = x1 * x4;
	const GEN_FLT x67 = x2 * x8;
	const GEN_FLT x68 = x66 + x67;
	const GEN_FLT x69 = x68 * x65;
	const GEN_FLT x70 = (-1 * x3) + x9;
	const GEN_FLT x71 = 2 * x23;
	const GEN_FLT x72 = (((x71 * x47) + x56 + (x55 * x36)) * sensor_x) + (((-1 * x60) + (-1 * x62) + x63) * sensor_y) +
						((x43 + x46 + x50) * sensor_z);
	const GEN_FLT x73 = 1 + x72;
	const GEN_FLT x74 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x75 = x5 * x74;
	const GEN_FLT x76 = x4 * x75;
	const GEN_FLT x77 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x78 = x1 * x77;
	const GEN_FLT x79 = x1 * x74;
	const GEN_FLT x80 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x81 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x82 = x6 * x81;
	const GEN_FLT x83 = (x2 * x7 * x79) + (x80 * x27) + (x7 * x82);
	const GEN_FLT x84 = -1 * x79;
	const GEN_FLT x85 = x2 * x75;
	const GEN_FLT x86 = x1 * x81;
	const GEN_FLT x87 = x4 * x79;
	const GEN_FLT x88 = x4 * x6;
	const GEN_FLT x89 = (x7 * x87) + (x8 * x77) + (x80 * x88);
	const GEN_FLT x90 = (x33 * (x76 + x78 + x83)) + (x25 * (x84 + (x79 * x40) + (2 * x8 * x80))) +
						(x37 * ((-1 * x85) + (-1 * x86) + x89));
	const GEN_FLT x91 = x59 + x69 + (x70 * x73) + x90;
	const GEN_FLT x92 = x65 * x29;
	const GEN_FLT x93 = x58 * x10;
	const GEN_FLT x94 = x7 * x75;
	const GEN_FLT x95 = x1 * x80;
	const GEN_FLT x96 = (x77 * x27) + (x4 * x82) + (x2 * x87);
	const GEN_FLT x97 = (x33 * ((-1 * x94) + (-1 * x95) + x96)) + (x25 * (x85 + x86 + x89)) +
						(x37 * (x84 + (x79 * x34) + (2 * x88 * x77)));
	const GEN_FLT x98 = x92 + x93 + (x73 * x35) + x97;
	const GEN_FLT x99 = x38 * x38;
	const GEN_FLT x100 = lh_pz + (x41 * x25) + (x68 * x33) + (x70 * x37);
	const GEN_FLT x101 = pow(x99, -1) * x100;
	const GEN_FLT x102 = (x100 * x100) + x99;
	const GEN_FLT x103 = pow(x102, -1);
	const GEN_FLT x104 = x99 * x103;
	const GEN_FLT x105 = x104 * ((-1 * x91 * x39) + (x98 * x101));
	const GEN_FLT x106 = (-1 * x66) + x67;
	const GEN_FLT x107 = x2 * x2;
	const GEN_FLT x108 = (x6 * x107) + x5;
	const GEN_FLT x109 = x26 + x28;
	const GEN_FLT x110 = lh_py + (x25 * x106) + (x37 * x109) + (x33 * x108);
	const GEN_FLT x111 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x112 = cos(x111);
	const GEN_FLT x113 = pow(x112, -1);
	const GEN_FLT x114 = x110 * x110;
	const GEN_FLT x115 = x114 + x102;
	const GEN_FLT x116 = x113 * pow(x115, -1.0 / 2.0);
	const GEN_FLT x117 = asin(x110 * x116);
	const GEN_FLT x118 = 8.0108022e-06 * x117;
	const GEN_FLT x119 = -8.0108022e-06 + (-1 * x118);
	const GEN_FLT x120 = 0.0028679863 + (x119 * x117);
	const GEN_FLT x121 = 5.3685255e-06 + (x117 * x120);
	const GEN_FLT x122 = 0.0076069798 + (x117 * x121);
	const GEN_FLT x123 = x117 * x122;
	const GEN_FLT x124 = -8.0108022e-06 + (-1.60216044e-05 * x117);
	const GEN_FLT x125 = (x117 * x124) + x120;
	const GEN_FLT x126 = (x117 * x125) + x121;
	const GEN_FLT x127 = (x117 * x126) + x122;
	const GEN_FLT x128 = x123 + (x117 * x127);
	const GEN_FLT x129 = tan(x111);
	const GEN_FLT x130 = pow(x102, -1.0 / 2.0) * x129;
	const GEN_FLT x131 = -1 * x110 * x130;
	const GEN_FLT x132 = atan2(-1 * x100, x38);
	const GEN_FLT x133 = ogeeMag_1 + (-1 * asin(x131)) + x132;
	const GEN_FLT x134 = curve_1 + (sin(x133) * ogeePhase_1);
	const GEN_FLT x135 = sin(x111);
	const GEN_FLT x136 = x134 * x135;
	const GEN_FLT x137 = (x128 * x136) + x112;
	const GEN_FLT x138 = pow(x137, -1);
	const GEN_FLT x139 = x117 * x117;
	const GEN_FLT x140 = x134 * x139;
	const GEN_FLT x141 = x138 * x140;
	const GEN_FLT x142 = (x122 * x141) + x131;
	const GEN_FLT x143 = pow((1 + (-1 * (x142 * x142))), -1.0 / 2.0);
	const GEN_FLT x144 = 2 * x38;
	const GEN_FLT x145 = 2 * x100;
	const GEN_FLT x146 = (x98 * x144) + (x91 * x145);
	const GEN_FLT x147 = 1.0 / 2.0 * x110;
	const GEN_FLT x148 = pow(x102, -3.0 / 2.0) * x129 * x147;
	const GEN_FLT x149 = x65 * x108;
	const GEN_FLT x150 = (x33 * (x84 + (x79 * x107) + (2 * x2 * x82))) + (x25 * ((-1 * x76) + (-1 * x78) + x83)) +
						 (x37 * (x94 + x95 + x96));
	const GEN_FLT x151 = (x58 * x106) + x150;
	const GEN_FLT x152 = (x73 * x109) + x149 + x151;
	const GEN_FLT x153 = (x146 * x148) + (-1 * x130 * x152);
	const GEN_FLT x154 = pow((1 + (-1 * x103 * x114 * (x129 * x129))), -1.0 / 2.0);
	const GEN_FLT x155 = cos(x133) * ogeePhase_1;
	const GEN_FLT x156 = x155 * (x105 + (-1 * x153 * x154));
	const GEN_FLT x157 = x122 * x138 * x139;
	const GEN_FLT x158 = pow((1 + (-1 * pow(x112, -2) * x114 * pow(x115, -1))), -1.0 / 2.0);
	const GEN_FLT x159 = 2 * x110;
	const GEN_FLT x160 = x113 * pow(x115, -3.0 / 2.0) * x147;
	const GEN_FLT x161 = x158 * ((-1 * x160 * ((x152 * x159) + x146)) + (x116 * x152));
	const GEN_FLT x162 = 2 * x123 * x134 * x138;
	const GEN_FLT x163 = x119 * x161;
	const GEN_FLT x164 = (x117 * ((-1 * x118 * x161) + x163)) + (x120 * x161);
	const GEN_FLT x165 = (x121 * x161) + (x117 * x164);
	const GEN_FLT x166 = x128 * x135;
	const GEN_FLT x167 = 2.40324066e-05 * x117;
	const GEN_FLT x168 = x122 * pow(x137, -2) * x140;
	const GEN_FLT x169 =
		x105 +
		(-1 * x143 *
		 ((x156 * x157) + (x161 * x162) + x153 + (x165 * x141) +
		  (-1 * x168 *
		   ((x166 * x156) +
			(x136 * ((x127 * x161) +
					 (x117 * ((x126 * x161) + x165 +
							  (x117 * ((x117 * (x163 + (-1 * x161 * x167) + (x124 * x161))) + (x125 * x161) + x164)))) +
					 (x117 * x165) + (x122 * x161)))))));
	const GEN_FLT x170 = cos(gibPhase_1 + (-1 * asin(x142)) + x132) * gibMag_1;
	const GEN_FLT x171 = 1 + x65;
	const GEN_FLT x172 = (x70 * x72) + x90;
	const GEN_FLT x173 = (x68 * x171) + x172 + x59;
	const GEN_FLT x174 = (x72 * x35) + x97;
	const GEN_FLT x175 = (x29 * x171) + x93 + x174;
	const GEN_FLT x176 = x104 * ((-1 * x39 * x173) + (x101 * x175));
	const GEN_FLT x177 = (x175 * x144) + (x173 * x145);
	const GEN_FLT x178 = x72 * x109;
	const GEN_FLT x179 = x178 + (x108 * x171) + x151;
	const GEN_FLT x180 = (x177 * x148) + (-1 * x179 * x130);
	const GEN_FLT x181 = x176 + (-1 * x180 * x154);
	const GEN_FLT x182 = x155 * x157;
	const GEN_FLT x183 = (-1 * x160 * ((x179 * x159) + x177)) + (x116 * x179);
	const GEN_FLT x184 = x183 * x158;
	const GEN_FLT x185 = x119 * x184;
	const GEN_FLT x186 = (x117 * ((-1 * x118 * x184) + x185)) + (x120 * x184);
	const GEN_FLT x187 = (x121 * x184) + (x117 * x186);
	const GEN_FLT x188 = x166 * x155;
	const GEN_FLT x189 = x126 * x158;
	const GEN_FLT x190 =
		x176 +
		(-1 * x143 *
		 ((x181 * x182) + (x162 * x184) + (x187 * x141) +
		  (-1 * x168 *
		   ((x181 * x188) +
			(x136 *
			 ((x127 * x184) +
			  (x117 * ((x189 * x183) +
					   (x117 * ((x117 * (x185 + (-1 * x167 * x184) + (x124 * x184))) + (x125 * x184) + x186)) + x187)) +
			  (x122 * x184) + (x117 * x187))))) +
		  x180));
	const GEN_FLT x191 = 1 + x58;
	const GEN_FLT x192 = x69 + x172 + (x41 * x191);
	const GEN_FLT x193 = x92 + (x10 * x191) + x174;
	const GEN_FLT x194 = x104 * ((-1 * x39 * x192) + (x101 * x193));
	const GEN_FLT x195 = (x193 * x144) + (x192 * x145);
	const GEN_FLT x196 = x178 + x149 + (x106 * x191) + x150;
	const GEN_FLT x197 = (x195 * x148) + (-1 * x196 * x130);
	const GEN_FLT x198 = x194 + (-1 * x197 * x154);
	const GEN_FLT x199 = x158 * ((-1 * x160 * ((x196 * x159) + x195)) + (x116 * x196));
	const GEN_FLT x200 = x119 * x199;
	const GEN_FLT x201 = (x117 * ((-1 * x118 * x199) + x200)) + (x120 * x199);
	const GEN_FLT x202 = (x121 * x199) + (x201 * x117);
	const GEN_FLT x203 =
		x194 +
		(-1 * x143 *
		 ((x182 * x198) + (x202 * x141) + x197 + (x162 * x199) +
		  (-1 * x168 *
		   ((x188 * x198) +
			(x136 *
			 ((x117 * ((x126 * x199) +
					   (x117 * ((x117 * (x200 + (-1 * x167 * x199) + (x124 * x199))) + (x125 * x199) + x201)) + x202)) +
			  (x122 * x199) + (x127 * x199) + (x202 * x117)))))));
	const GEN_FLT x204 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x205 = x17 * x204;
	const GEN_FLT x206 = -1 * x205;
	const GEN_FLT x207 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qi *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x208 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qk *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x209 = x17 * x208;
	const GEN_FLT x210 = x61 * x204;
	const GEN_FLT x211 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qj *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x212 = x16 * x205;
	const GEN_FLT x213 = (x53 * x207) + (x23 * x211) + (x19 * x212);
	const GEN_FLT x214 = x17 * x211;
	const GEN_FLT x215 = x14 * x19;
	const GEN_FLT x216 = x215 * x204;
	const GEN_FLT x217 = (x20 * x207) + (x23 * x208) + (x11 * x212);
	const GEN_FLT x218 = ((x206 + (x71 * x207) + (x36 * x205)) * sensor_x) +
						 (((-1 * x209) + (-1 * x210) + x213) * sensor_y) + ((x214 + x216 + x217) * sensor_z);
	const GEN_FLT x219 = x17 * x207;
	const GEN_FLT x220 = x14 * x16;
	const GEN_FLT x221 = x204 * x220;
	const GEN_FLT x222 = x11 * x19;
	const GEN_FLT x223 = (x20 * x211) + (x53 * x208) + (x205 * x222);
	const GEN_FLT x224 = ((x209 + x210 + x213) * sensor_x) + (((x64 * x211) + x206 + (x30 * x205)) * sensor_y) +
						 (((-1 * x219) + (-1 * x221) + x223) * sensor_z);
	const GEN_FLT x225 = (((-1 * x214) + (-1 * x216) + x217) * sensor_x) + ((x219 + x221 + x223) * sensor_y) +
						 ((x206 + (x57 * x208) + (x12 * x205)) * sensor_z);
	const GEN_FLT x226 = (x70 * x218) + (x68 * x224) + (x41 * x225) + x90;
	const GEN_FLT x227 = (x35 * x218) + (x29 * x224) + (x10 * x225) + x97;
	const GEN_FLT x228 = x104 * ((-1 * x39 * x226) + (x227 * x101));
	const GEN_FLT x229 = (x227 * x144) + (x226 * x145);
	const GEN_FLT x230 = (x218 * x109) + (x224 * x108) + (x225 * x106) + x150;
	const GEN_FLT x231 = (x229 * x148) + (-1 * x230 * x130);
	const GEN_FLT x232 = x228 + (-1 * x231 * x154);
	const GEN_FLT x233 = (-1 * x160 * ((x230 * x159) + x229)) + (x230 * x116);
	const GEN_FLT x234 = x233 * x158;
	const GEN_FLT x235 = x234 * x119;
	const GEN_FLT x236 = (x117 * ((-1 * x234 * x118) + x235)) + (x234 * x120);
	const GEN_FLT x237 = (x234 * x121) + (x236 * x117);
	const GEN_FLT x238 =
		x228 +
		(-1 * x143 *
		 ((x232 * x182) + (x234 * x162) + (x237 * x141) +
		  (-1 * x168 *
		   ((x232 * x188) +
			(x136 *
			 ((x234 * x127) + (x234 * x122) +
			  (x117 * ((x233 * x189) +
					   (x117 * ((x117 * (x235 + (x234 * x124) + (-1 * x234 * x167))) + (x234 * x125) + x236)) + x237)) +
			  (x237 * x117))))) +
		  x231));
	const GEN_FLT x239 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x240 = x17 * x239;
	const GEN_FLT x241 = -1 * x240;
	const GEN_FLT x242 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qi *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x243 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qk *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x244 = x17 * x243;
	const GEN_FLT x245 = x61 * x239;
	const GEN_FLT x246 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qj *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x247 = x19 * x16;
	const GEN_FLT x248 = (x53 * x242) + (x23 * x246) + (x240 * x247);
	const GEN_FLT x249 = x17 * x246;
	const GEN_FLT x250 = x215 * x239;
	const GEN_FLT x251 = x11 * x16;
	const GEN_FLT x252 = (x20 * x242) + (x251 * x240) + (x23 * x243);
	const GEN_FLT x253 = ((x241 + (x71 * x242) + (x36 * x240)) * sensor_x) + ((x249 + x250 + x252) * sensor_z) +
						 (((-1 * x244) + (-1 * x245) + x248) * sensor_y);
	const GEN_FLT x254 = x17 * x242;
	const GEN_FLT x255 = x239 * x220;
	const GEN_FLT x256 = (x53 * x243) + (x222 * x240) + (x20 * x246);
	const GEN_FLT x257 = ((x241 + (x30 * x240) + (x64 * x246)) * sensor_y) + ((x244 + x245 + x248) * sensor_x) +
						 (((-1 * x254) + (-1 * x255) + x256) * sensor_z);
	const GEN_FLT x258 = (((-1 * x249) + (-1 * x250) + x252) * sensor_x) + ((x254 + x255 + x256) * sensor_y) +
						 ((x241 + (x57 * x243) + (x12 * x240)) * sensor_z);
	const GEN_FLT x259 = (x70 * x253) + (x68 * x257) + (x41 * x258) + x90;
	const GEN_FLT x260 = (x35 * x253) + (x29 * x257) + (x10 * x258) + x97;
	const GEN_FLT x261 = x104 * ((-1 * x39 * x259) + (x260 * x101));
	const GEN_FLT x262 = (x260 * x144) + (x259 * x145);
	const GEN_FLT x263 = (x253 * x109) + (x257 * x108) + (x258 * x106) + x150;
	const GEN_FLT x264 = (x262 * x148) + (-1 * x263 * x130);
	const GEN_FLT x265 = x261 + (-1 * x264 * x154);
	const GEN_FLT x266 = (-1 * x160 * ((x263 * x159) + x262)) + (x263 * x116);
	const GEN_FLT x267 = x266 * x158;
	const GEN_FLT x268 = x267 * x119;
	const GEN_FLT x269 = (x117 * ((-1 * x267 * x118) + x268)) + (x267 * x120);
	const GEN_FLT x270 = (x267 * x121) + (x269 * x117);
	const GEN_FLT x271 =
		x261 +
		(-1 * x143 *
		 ((x265 * x182) + (x270 * x141) + (x267 * x162) +
		  (-1 * x168 *
		   ((x265 * x188) +
			(x136 *
			 ((x117 * ((x266 * x189) +
					   (x117 * ((x117 * ((-1 * x267 * x167) + (x267 * x124) + x268)) + (x267 * x125) + x269)) + x270)) +
			  (x267 * x127) + (x270 * x117) + (x267 * x122))))) +
		  x264));
	const GEN_FLT x272 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x273 = x17 * x272;
	const GEN_FLT x274 = -1 * x273;
	const GEN_FLT x275 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qi *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x276 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qk *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x277 = x17 * x276;
	const GEN_FLT x278 = x61 * x272;
	const GEN_FLT x279 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qj *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x280 = (x53 * x275) + (x23 * x279) + (x273 * x247);
	const GEN_FLT x281 = x17 * x279;
	const GEN_FLT x282 = x215 * x272;
	const GEN_FLT x283 = (x20 * x275) + (x273 * x251) + (x23 * x276);
	const GEN_FLT x284 = ((x274 + (x71 * x275) + (x36 * x273)) * sensor_x) +
						 (((-1 * x277) + (-1 * x278) + x280) * sensor_y) + ((x281 + x283 + x282) * sensor_z);
	const GEN_FLT x285 = x17 * x275;
	const GEN_FLT x286 = x272 * x220;
	const GEN_FLT x287 = (x20 * x279) + (x53 * x276) + (x273 * x222);
	const GEN_FLT x288 = ((x277 + x278 + x280) * sensor_x) + (((x64 * x279) + x274 + (x30 * x273)) * sensor_y) +
						 (((-1 * x285) + (-1 * x286) + x287) * sensor_z);
	const GEN_FLT x289 = (((-1 * x281) + x283 + (-1 * x282)) * sensor_x) + ((x285 + x286 + x287) * sensor_y) +
						 ((x274 + (x57 * x276) + (x12 * x273)) * sensor_z);
	const GEN_FLT x290 = (x70 * x284) + (x68 * x288) + (x41 * x289) + x90;
	const GEN_FLT x291 = (x35 * x284) + (x29 * x288) + (x10 * x289) + x97;
	const GEN_FLT x292 = x104 * ((-1 * x39 * x290) + (x291 * x101));
	const GEN_FLT x293 = (x291 * x144) + (x290 * x145);
	const GEN_FLT x294 = (x284 * x109) + (x288 * x108) + (x289 * x106) + x150;
	const GEN_FLT x295 = (x293 * x148) + (-1 * x294 * x130);
	const GEN_FLT x296 = x292 + (-1 * x295 * x154);
	const GEN_FLT x297 = x158 * ((-1 * x160 * ((x294 * x159) + x293)) + (x294 * x116));
	const GEN_FLT x298 = x297 * x119;
	const GEN_FLT x299 = (x117 * ((-1 * x297 * x118) + x298)) + (x297 * x120);
	const GEN_FLT x300 = (x297 * x121) + (x299 * x117);
	const GEN_FLT x301 =
		x292 +
		(-1 * x143 *
		 ((x296 * x182) + (x300 * x141) + (x297 * x162) +
		  (-1 * x168 *
		   ((x296 * x188) +
			(x136 *
			 ((x117 * ((x297 * x126) +
					   (x117 * ((x117 * (x298 + (-1 * x297 * x167) + (x297 * x124))) + (x297 * x125) + x299)) + x300)) +
			  (x300 * x117) + (x297 * x127) + (x297 * x122))))) +
		  x295));
	out[0] = (x169 * x170) + x169;
	out[1] = (x170 * x190) + x190;
	out[2] = (x203 * x170) + x203;
	out[3] = (x238 * x170) + x238;
	out[4] = (x271 * x170) + x271;
	out[5] = (x301 * x170) + x301;
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (x15 * x12) + x14;
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x18 = sin(x13);
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x21 * x11;
	const GEN_FLT x23 = x19 + x22;
	const GEN_FLT x24 = x20 * x18;
	const GEN_FLT x25 = x15 * x11;
	const GEN_FLT x26 = x25 * x17;
	const GEN_FLT x27 = (-1 * x24) + x26;
	const GEN_FLT x28 = obj_pz + (x27 * sensor_x) + (x16 * sensor_z) + (x23 * sensor_y);
	const GEN_FLT x29 = x1 * x4;
	const GEN_FLT x30 = x2 * x8;
	const GEN_FLT x31 = (-1 * x29) + x30;
	const GEN_FLT x32 = (-1 * x19) + x22;
	const GEN_FLT x33 = x20 * x20;
	const GEN_FLT x34 = (x33 * x15) + x14;
	const GEN_FLT x35 = x11 * x18;
	const GEN_FLT x36 = x21 * x17;
	const GEN_FLT x37 = x35 + x36;
	const GEN_FLT x38 = obj_py + (x32 * sensor_z) + (x34 * sensor_y) + (x37 * sensor_x);
	const GEN_FLT x39 = x7 * x7;
	const GEN_FLT x40 = (x6 * x39) + x5;
	const GEN_FLT x41 = x24 + x26;
	const GEN_FLT x42 = (-1 * x35) + x36;
	const GEN_FLT x43 = x17 * x17;
	const GEN_FLT x44 = (x43 * x15) + x14;
	const GEN_FLT x45 = obj_px + (x42 * sensor_y) + (x41 * sensor_z) + (x44 * sensor_x);
	const GEN_FLT x46 = lh_px + (x28 * x10) + (x40 * x45) + (x31 * x38);
	const GEN_FLT x47 = pow(x46, -1);
	const GEN_FLT x48 = (-1 * x3) + x9;
	const GEN_FLT x49 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x50 = x15 * x17;
	const GEN_FLT x51 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x52 = x51 * x18;
	const GEN_FLT x53 = -1 * x52;
	const GEN_FLT x54 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x55 = x54 * x18;
	const GEN_FLT x56 = x51 * x14;
	const GEN_FLT x57 = x56 * x11;
	const GEN_FLT x58 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x59 = x51 * x19;
	const GEN_FLT x60 = (x50 * x58) + (x49 * x21) + (x59 * x20);
	const GEN_FLT x61 = x58 * x18;
	const GEN_FLT x62 = x56 * x20;
	const GEN_FLT x63 = (x49 * x25) + (x59 * x11) + (x50 * x54);
	const GEN_FLT x64 = (((2 * x50 * x49) + x53 + (x52 * x43)) * sensor_x) +
						(((-1 * x55) + (-1 * x57) + x60) * sensor_y) + ((x61 + x62 + x63) * sensor_z);
	const GEN_FLT x65 = x44 + x64;
	const GEN_FLT x66 = x1 * x7;
	const GEN_FLT x67 = x2 * x6;
	const GEN_FLT x68 = x4 * x67;
	const GEN_FLT x69 = x66 + x68;
	const GEN_FLT x70 = x49 * x18;
	const GEN_FLT x71 = x56 * x17;
	const GEN_FLT x72 = (x58 * x25) + (x54 * x21) + (x51 * x24 * x11);
	const GEN_FLT x73 = ((x55 + x57 + x60) * sensor_x) + (((-1 * x70) + (-1 * x71) + x72) * sensor_z) +
						((x53 + (2 * x58 * x21) + (x52 * x33)) * sensor_y);
	const GEN_FLT x74 = x37 + x73;
	const GEN_FLT x75 = x4 * x4;
	const GEN_FLT x76 = (x6 * x75) + x5;
	const GEN_FLT x77 = (((-1 * x61) + (-1 * x62) + x63) * sensor_x) + ((x70 + x71 + x72) * sensor_y) +
						((x53 + (2 * x54 * x25) + (x52 * x12)) * sensor_z);
	const GEN_FLT x78 = x27 + x77;
	const GEN_FLT x79 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x80 = x5 * x79;
	const GEN_FLT x81 = x2 * x80;
	const GEN_FLT x82 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x83 = x1 * x82;
	const GEN_FLT x84 = x1 * x79;
	const GEN_FLT x85 = x7 * x84;
	const GEN_FLT x86 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x87 = x4 * x6;
	const GEN_FLT x88 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x89 = (x4 * x85) + (x86 * x87) + (x8 * x88);
	const GEN_FLT x90 = x7 * x80;
	const GEN_FLT x91 = x1 * x86;
	const GEN_FLT x92 = (x2 * x4 * x84) + (x88 * x67) + (x82 * x87);
	const GEN_FLT x93 = -1 * x84;
	const GEN_FLT x94 = (x45 * ((-1 * x81) + (-1 * x83) + x89)) + (x38 * (x90 + x91 + x92)) +
						(x28 * (x93 + (x84 * x75) + (2 * x88 * x87)));
	const GEN_FLT x95 = (x65 * x48) + (x74 * x69) + (x78 * x76) + x94;
	const GEN_FLT x96 = x4 * x80;
	const GEN_FLT x97 = x1 * x88;
	const GEN_FLT x98 = (x86 * x67) + (x8 * x82) + (x2 * x85);
	const GEN_FLT x99 = (x45 * (x93 + (x84 * x39) + (2 * x8 * x86))) + (x38 * ((-1 * x96) + (-1 * x97) + x98)) +
						(x28 * (x81 + x83 + x89));
	const GEN_FLT x100 = (x65 * x40) + (x74 * x31) + (x78 * x10) + x99;
	const GEN_FLT x101 = x46 * x46;
	const GEN_FLT x102 = lh_pz + (x76 * x28) + (x69 * x38) + (x45 * x48);
	const GEN_FLT x103 = pow(x101, -1) * x102;
	const GEN_FLT x104 = (x102 * x102) + x101;
	const GEN_FLT x105 = pow(x104, -1);
	const GEN_FLT x106 = x101 * x105;
	const GEN_FLT x107 = x106 * ((-1 * x95 * x47) + (x100 * x103));
	const GEN_FLT x108 = (-1 * x66) + x68;
	const GEN_FLT x109 = x2 * x2;
	const GEN_FLT x110 = (x6 * x109) + x5;
	const GEN_FLT x111 = x29 + x30;
	const GEN_FLT x112 = lh_py + (x38 * x110) + (x28 * x108) + (x45 * x111);
	const GEN_FLT x113 = x112 * x112;
	const GEN_FLT x114 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x115 = tan(x114);
	const GEN_FLT x116 = pow((1 + (-1 * x105 * x113 * (x115 * x115))), -1.0 / 2.0);
	const GEN_FLT x117 = 2 * x46;
	const GEN_FLT x118 = 2 * x102;
	const GEN_FLT x119 = (x100 * x117) + (x95 * x118);
	const GEN_FLT x120 = 1.0 / 2.0 * x112;
	const GEN_FLT x121 = pow(x104, -3.0 / 2.0) * x115 * x120;
	const GEN_FLT x122 = (x45 * (x96 + x97 + x98)) + (x38 * (x93 + (x84 * x109) + (2 * x82 * x67))) +
						 (x28 * ((-1 * x90) + (-1 * x91) + x92));
	const GEN_FLT x123 = (x65 * x111) + (x74 * x110) + (x78 * x108) + x122;
	const GEN_FLT x124 = pow(x104, -1.0 / 2.0) * x115;
	const GEN_FLT x125 = (x119 * x121) + (-1 * x124 * x123);
	const GEN_FLT x126 = x107 + (-1 * x116 * x125);
	const GEN_FLT x127 = -1 * x112 * x124;
	const GEN_FLT x128 = atan2(-1 * x102, x46);
	const GEN_FLT x129 = ogeeMag_1 + (-1 * asin(x127)) + x128;
	const GEN_FLT x130 = cos(x129) * ogeePhase_1;
	const GEN_FLT x131 = cos(x114);
	const GEN_FLT x132 = pow(x131, -1);
	const GEN_FLT x133 = x113 + x104;
	const GEN_FLT x134 = pow(x133, -1.0 / 2.0) * x132;
	const GEN_FLT x135 = asin(x112 * x134);
	const GEN_FLT x136 = 8.0108022e-06 * x135;
	const GEN_FLT x137 = -8.0108022e-06 + (-1 * x136);
	const GEN_FLT x138 = 0.0028679863 + (x137 * x135);
	const GEN_FLT x139 = 5.3685255e-06 + (x138 * x135);
	const GEN_FLT x140 = 0.0076069798 + (x135 * x139);
	const GEN_FLT x141 = x135 * x140;
	const GEN_FLT x142 = -8.0108022e-06 + (-1.60216044e-05 * x135);
	const GEN_FLT x143 = (x135 * x142) + x138;
	const GEN_FLT x144 = (x135 * x143) + x139;
	const GEN_FLT x145 = (x135 * x144) + x140;
	const GEN_FLT x146 = x141 + (x135 * x145);
	const GEN_FLT x147 = curve_1 + (sin(x129) * ogeePhase_1);
	const GEN_FLT x148 = sin(x114);
	const GEN_FLT x149 = x148 * x147;
	const GEN_FLT x150 = (x146 * x149) + x131;
	const GEN_FLT x151 = pow(x150, -1);
	const GEN_FLT x152 = x135 * x135;
	const GEN_FLT x153 = x140 * x152;
	const GEN_FLT x154 = x151 * x153;
	const GEN_FLT x155 = x130 * x154;
	const GEN_FLT x156 = 2 * x112;
	const GEN_FLT x157 = x120 * pow(x133, -3.0 / 2.0) * x132;
	const GEN_FLT x158 = pow((1 + (-1 * x113 * pow(x133, -1) * pow(x131, -2))), -1.0 / 2.0);
	const GEN_FLT x159 = x158 * ((-1 * x157 * ((x123 * x156) + x119)) + (x123 * x134));
	const GEN_FLT x160 = x137 * x159;
	const GEN_FLT x161 = (x135 * ((-1 * x136 * x159) + x160)) + (x138 * x159);
	const GEN_FLT x162 = (x139 * x159) + (x161 * x135);
	const GEN_FLT x163 = x147 * x151;
	const GEN_FLT x164 = x163 * x152;
	const GEN_FLT x165 = 2 * x163 * x141;
	const GEN_FLT x166 = x146 * x148;
	const GEN_FLT x167 = x166 * x130;
	const GEN_FLT x168 = 2.40324066e-05 * x135;
	const GEN_FLT x169 = x147 * pow(x150, -2) * x153;
	const GEN_FLT x170 = (x164 * x140) + x127;
	const GEN_FLT x171 = pow((1 + (-1 * (x170 * x170))), -1.0 / 2.0);
	const GEN_FLT x172 =
		x107 +
		(-1 * x171 *
		 ((x126 * x155) + (x165 * x159) + (x164 * x162) +
		  (-1 * x169 *
		   ((x126 * x167) +
			(x149 *
			 ((x145 * x159) + (x140 * x159) +
			  (x135 * ((x144 * x159) +
					   (x135 * ((x135 * (x160 + (x142 * x159) + (-1 * x168 * x159))) + (x143 * x159) + x161)) + x162)) +
			  (x162 * x135))))) +
		  x125));
	const GEN_FLT x173 = cos(gibPhase_1 + (-1 * asin(x170)) + x128) * gibMag_1;
	const GEN_FLT x174 = x42 + x64;
	const GEN_FLT x175 = x34 + x73;
	const GEN_FLT x176 = x23 + x77;
	const GEN_FLT x177 = (x48 * x174) + (x69 * x175) + (x76 * x176) + x94;
	const GEN_FLT x178 = (x40 * x174) + (x31 * x175) + (x10 * x176) + x99;
	const GEN_FLT x179 = x106 * ((-1 * x47 * x177) + (x103 * x178));
	const GEN_FLT x180 = (x117 * x178) + (x118 * x177);
	const GEN_FLT x181 = (x111 * x174) + (x110 * x175) + (x108 * x176) + x122;
	const GEN_FLT x182 = (x121 * x180) + (-1 * x124 * x181);
	const GEN_FLT x183 = x179 + (-1 * x116 * x182);
	const GEN_FLT x184 = x158 * ((-1 * x157 * ((x181 * x156) + x180)) + (x181 * x134));
	const GEN_FLT x185 = x184 * x137;
	const GEN_FLT x186 = (x135 * ((-1 * x184 * x136) + x185)) + (x184 * x138);
	const GEN_FLT x187 = (x184 * x139) + (x186 * x135);
	const GEN_FLT x188 =
		x179 +
		(-1 * x171 *
		 ((x183 * x155) +
		  (-1 * x169 *
		   ((x167 * x183) +
			(x149 * ((x184 * x145) + (x184 * x140) +
					 (x135 * ((x184 * x144) + x187 +
							  (x135 * ((x135 * ((-1 * x168 * x184) + x185 + (x184 * x142))) + x186 + (x184 * x143))))) +
					 (x187 * x135))))) +
		  (x164 * x187) + x182 + (x165 * x184)));
	const GEN_FLT x189 = x41 + x64;
	const GEN_FLT x190 = x32 + x73;
	const GEN_FLT x191 = x16 + x77;
	const GEN_FLT x192 = (x48 * x189) + (x69 * x190) + (x76 * x191) + x94;
	const GEN_FLT x193 = (x40 * x189) + (x31 * x190) + (x10 * x191) + x99;
	const GEN_FLT x194 = x106 * ((-1 * x47 * x192) + (x103 * x193));
	const GEN_FLT x195 = (x117 * x193) + (x118 * x192);
	const GEN_FLT x196 = (x110 * x190) + (x111 * x189) + (x108 * x191) + x122;
	const GEN_FLT x197 = (x121 * x195) + (-1 * x124 * x196);
	const GEN_FLT x198 = x130 * (x194 + (-1 * x116 * x197));
	const GEN_FLT x199 = x158 * ((-1 * x157 * ((x196 * x156) + x195)) + (x196 * x134));
	const GEN_FLT x200 = x199 * x137;
	const GEN_FLT x201 = (x135 * ((-1 * x199 * x136) + x200)) + (x199 * x138);
	const GEN_FLT x202 = (x199 * x139) + (x201 * x135);
	const GEN_FLT x203 =
		x194 +
		(-1 * x171 *
		 ((x198 * x154) + (x202 * x164) + (x165 * x199) +
		  (-1 * x169 *
		   ((x166 * x198) +
			(x149 *
			 ((x199 * x145) +
			  (x135 * ((x199 * x144) +
					   (x135 * ((x135 * (x200 + (-1 * x168 * x199) + (x199 * x142))) + (x199 * x143) + x201)) + x202)) +
			  (x202 * x135) + (x199 * x140))))) +
		  x197));
	out[0] = (x172 * x173) + x172;
	out[1] = (x173 * x188) + x188;
	out[2] = (x203 * x173) + x203;
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x20 * x11;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x15 * x16;
	const GEN_FLT x24 = x23 * x11;
	const GEN_FLT x25 =
		obj_pz + (((x15 * x12) + x14) * sensor_z) + ((x18 + x21) * sensor_y) + (((-1 * x22) + x24) * sensor_x);
	const GEN_FLT x26 = x1 * x4;
	const GEN_FLT x27 = x2 * x8;
	const GEN_FLT x28 = (-1 * x26) + x27;
	const GEN_FLT x29 = x19 * x19;
	const GEN_FLT x30 = x11 * x17;
	const GEN_FLT x31 = x20 * x16;
	const GEN_FLT x32 =
		obj_py + (((-1 * x18) + x21) * sensor_z) + (((x29 * x15) + x14) * sensor_y) + ((x30 + x31) * sensor_x);
	const GEN_FLT x33 = x7 * x7;
	const GEN_FLT x34 = (x6 * x33) + x5;
	const GEN_FLT x35 = x16 * x16;
	const GEN_FLT x36 =
		obj_px + ((x22 + x24) * sensor_z) + (((-1 * x30) + x31) * sensor_y) + (((x35 * x15) + x14) * sensor_x);
	const GEN_FLT x37 = lh_px + (x25 * x10) + (x32 * x28) + (x34 * x36);
	const GEN_FLT x38 = pow(x37, -1);
	const GEN_FLT x39 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x40 = x5 * x39;
	const GEN_FLT x41 = x2 * x40;
	const GEN_FLT x42 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x43 = x1 * x42;
	const GEN_FLT x44 = x1 * x39;
	const GEN_FLT x45 = x4 * x44;
	const GEN_FLT x46 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x47 = x4 * x6;
	const GEN_FLT x48 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x49 = (x7 * x45) + (x46 * x47) + (x8 * x48);
	const GEN_FLT x50 = x5 * x7;
	const GEN_FLT x51 = x50 * x39;
	const GEN_FLT x52 = x1 * x46;
	const GEN_FLT x53 = x2 * x6;
	const GEN_FLT x54 = (x2 * x45) + (x53 * x48) + (x42 * x47);
	const GEN_FLT x55 = -1 * x44;
	const GEN_FLT x56 = x4 * x4;
	const GEN_FLT x57 = 2 * x47;
	const GEN_FLT x58 = (-1 * x3) + x9;
	const GEN_FLT x59 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x60 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x61 = x60 * x17;
	const GEN_FLT x62 = -1 * x61;
	const GEN_FLT x63 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x64 = x63 * x17;
	const GEN_FLT x65 = x60 * x14;
	const GEN_FLT x66 = x65 * x11;
	const GEN_FLT x67 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x68 = x60 * x18;
	const GEN_FLT x69 = (x67 * x23) + (x68 * x19) + (x59 * x20);
	const GEN_FLT x70 = x67 * x17;
	const GEN_FLT x71 = x65 * x19;
	const GEN_FLT x72 = x15 * x11;
	const GEN_FLT x73 = (x72 * x59) + (x68 * x11) + (x63 * x23);
	const GEN_FLT x74 = (((2 * x59 * x23) + x62 + (x61 * x35)) * sensor_x) +
						(((-1 * x64) + (-1 * x66) + x69) * sensor_y) + ((x70 + x71 + x73) * sensor_z);
	const GEN_FLT x75 = x59 * x17;
	const GEN_FLT x76 = x65 * x16;
	const GEN_FLT x77 = (x72 * x67) + (x63 * x20) + (x61 * x11 * x19);
	const GEN_FLT x78 = ((x64 + x66 + x69) * sensor_x) + ((x62 + (2 * x67 * x20) + (x61 * x29)) * sensor_y) +
						(((-1 * x75) + (-1 * x76) + x77) * sensor_z);
	const GEN_FLT x79 = x1 * x7;
	const GEN_FLT x80 = x2 * x47;
	const GEN_FLT x81 = x79 + x80;
	const GEN_FLT x82 = (x6 * x56) + x5;
	const GEN_FLT x83 = (((-1 * x70) + (-1 * x71) + x73) * sensor_x) + ((x75 + x76 + x77) * sensor_y) +
						((x62 + (2 * x72 * x63) + (x61 * x12)) * sensor_z);
	const GEN_FLT x84 = (x74 * x58) + (x81 * x78) + (x82 * x83);
	const GEN_FLT x85 = (x36 * ((-1 * x41) + (-1 * x43) + x49)) + (x32 * (x51 + x52 + x54)) +
						(x25 * (x55 + (x56 * x44) + (x57 * x48))) + x84;
	const GEN_FLT x86 = -1 * x85 * x38;
	const GEN_FLT x87 = 2 * x8;
	const GEN_FLT x88 = x4 * x40;
	const GEN_FLT x89 = x1 * x48;
	const GEN_FLT x90 = x2 * x7;
	const GEN_FLT x91 = (x53 * x46) + (x8 * x42) + (x90 * x44);
	const GEN_FLT x92 = (x74 * x34) + (x78 * x28) + (x83 * x10);
	const GEN_FLT x93 = (x36 * (x55 + (x44 * x33) + (x87 * x46))) + (x32 * ((-1 * x88) + (-1 * x89) + x91)) + x92 +
						(x25 * (x41 + x43 + x49));
	const GEN_FLT x94 = 1 + x93;
	const GEN_FLT x95 = x37 * x37;
	const GEN_FLT x96 = lh_pz + (x82 * x25) + (x81 * x32) + (x58 * x36);
	const GEN_FLT x97 = x96 * pow(x95, -1);
	const GEN_FLT x98 = (x96 * x96) + x95;
	const GEN_FLT x99 = pow(x98, -1);
	const GEN_FLT x100 = x99 * x95;
	const GEN_FLT x101 = x100 * (x86 + (x97 * x94));
	const GEN_FLT x102 = (-1 * x79) + x80;
	const GEN_FLT x103 = x2 * x2;
	const GEN_FLT x104 = (x6 * x103) + x5;
	const GEN_FLT x105 = x26 + x27;
	const GEN_FLT x106 = lh_py + (x25 * x102) + (x32 * x104) + (x36 * x105);
	const GEN_FLT x107 = x106 * x106;
	const GEN_FLT x108 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x109 = tan(x108);
	const GEN_FLT x110 = pow((1 + (-1 * x99 * (x109 * x109) * x107)), -1.0 / 2.0);
	const GEN_FLT x111 = 2 * x37;
	const GEN_FLT x112 = 2 * x96;
	const GEN_FLT x113 = x85 * x112;
	const GEN_FLT x114 = (x94 * x111) + x113;
	const GEN_FLT x115 = 1.0 / 2.0 * x106;
	const GEN_FLT x116 = pow(x98, -3.0 / 2.0) * x109 * x115;
	const GEN_FLT x117 = 2 * x53;
	const GEN_FLT x118 = (x74 * x105) + (x78 * x104) + (x83 * x102);
	const GEN_FLT x119 = (x36 * (x88 + x89 + x91)) + (x32 * (x55 + (x44 * x103) + (x42 * x117))) +
						 (x25 * ((-1 * x51) + (-1 * x52) + x54)) + x118;
	const GEN_FLT x120 = pow(x98, -1.0 / 2.0) * x109;
	const GEN_FLT x121 = -1 * x119 * x120;
	const GEN_FLT x122 = (x114 * x116) + x121;
	const GEN_FLT x123 = x101 + (-1 * x110 * x122);
	const GEN_FLT x124 = -1 * x106 * x120;
	const GEN_FLT x125 = atan2(-1 * x96, x37);
	const GEN_FLT x126 = ogeeMag_1 + (-1 * asin(x124)) + x125;
	const GEN_FLT x127 = cos(x126) * ogeePhase_1;
	const GEN_FLT x128 = cos(x108);
	const GEN_FLT x129 = pow(x128, -1);
	const GEN_FLT x130 = x107 + x98;
	const GEN_FLT x131 = x129 * pow(x130, -1.0 / 2.0);
	const GEN_FLT x132 = asin(x106 * x131);
	const GEN_FLT x133 = 8.0108022e-06 * x132;
	const GEN_FLT x134 = -8.0108022e-06 + (-1 * x133);
	const GEN_FLT x135 = 0.0028679863 + (x134 * x132);
	const GEN_FLT x136 = 5.3685255e-06 + (x132 * x135);
	const GEN_FLT x137 = 0.0076069798 + (x132 * x136);
	const GEN_FLT x138 = x132 * x132;
	const GEN_FLT x139 = x132 * x137;
	const GEN_FLT x140 = -8.0108022e-06 + (-1.60216044e-05 * x132);
	const GEN_FLT x141 = (x132 * x140) + x135;
	const GEN_FLT x142 = (x132 * x141) + x136;
	const GEN_FLT x143 = (x132 * x142) + x137;
	const GEN_FLT x144 = x139 + (x132 * x143);
	const GEN_FLT x145 = curve_1 + (sin(x126) * ogeePhase_1);
	const GEN_FLT x146 = sin(x108);
	const GEN_FLT x147 = x146 * x145;
	const GEN_FLT x148 = (x144 * x147) + x128;
	const GEN_FLT x149 = pow(x148, -1);
	const GEN_FLT x150 = x137 * x138 * x149;
	const GEN_FLT x151 = x127 * x150;
	const GEN_FLT x152 = 2 * x106;
	const GEN_FLT x153 = x119 * x152;
	const GEN_FLT x154 = x115 * x129 * pow(x130, -3.0 / 2.0);
	const GEN_FLT x155 = x119 * x131;
	const GEN_FLT x156 = (-1 * (x153 + x114) * x154) + x155;
	const GEN_FLT x157 = pow((1 + (-1 * x107 * pow(x128, -2) * pow(x130, -1))), -1.0 / 2.0);
	const GEN_FLT x158 = x136 * x157;
	const GEN_FLT x159 = x133 * x157;
	const GEN_FLT x160 = x134 * x157;
	const GEN_FLT x161 = x160 * x156;
	const GEN_FLT x162 = x135 * x157;
	const GEN_FLT x163 = (x132 * ((-1 * x156 * x159) + x161)) + (x162 * x156);
	const GEN_FLT x164 = (x156 * x158) + (x163 * x132);
	const GEN_FLT x165 = x138 * x145;
	const GEN_FLT x166 = x165 * x149;
	const GEN_FLT x167 = 2 * x139 * x145 * x149;
	const GEN_FLT x168 = x167 * x157;
	const GEN_FLT x169 = x144 * x146;
	const GEN_FLT x170 = x127 * x169;
	const GEN_FLT x171 = x142 * x157;
	const GEN_FLT x172 = 2.40324066e-05 * x132;
	const GEN_FLT x173 = x172 * x157;
	const GEN_FLT x174 = x140 * x157;
	const GEN_FLT x175 = x141 * x157;
	const GEN_FLT x176 = x143 * x157;
	const GEN_FLT x177 = x137 * x157;
	const GEN_FLT x178 = x165 * x137 * pow(x148, -2);
	const GEN_FLT x179 = (x166 * x137) + x124;
	const GEN_FLT x180 = pow((1 + (-1 * (x179 * x179))), -1.0 / 2.0);
	const GEN_FLT x181 =
		x101 +
		(-1 * x180 *
		 ((x123 * x151) + (x166 * x164) + (x168 * x156) +
		  (-1 * x178 *
		   ((x123 * x170) +
			(x147 *
			 ((x132 * ((x171 * x156) +
					   (x132 * ((x132 * (x161 + (-1 * x173 * x156) + (x174 * x156))) + (x175 * x156) + x163)) + x164)) +
			  (x164 * x132) + (x176 * x156) + (x177 * x156))))) +
		  x122));
	const GEN_FLT x182 = cos(gibPhase_1 + (-1 * asin(x179)) + x125) * gibMag_1;
	const GEN_FLT x183 = x93 * x97;
	const GEN_FLT x184 = x100 * (x86 + x183);
	const GEN_FLT x185 = x93 * x111;
	const GEN_FLT x186 = x185 + x113;
	const GEN_FLT x187 = 1 + x119;
	const GEN_FLT x188 = (x116 * x186) + (-1 * x120 * x187);
	const GEN_FLT x189 = x184 + (-1 * x110 * x188);
	const GEN_FLT x190 = (-1 * x154 * ((x187 * x152) + x186)) + (x187 * x131);
	const GEN_FLT x191 = x190 * x157;
	const GEN_FLT x192 = x191 * x134;
	const GEN_FLT x193 = (x132 * ((-1 * x191 * x133) + x192)) + (x191 * x135);
	const GEN_FLT x194 = (x191 * x136) + (x193 * x132);
	const GEN_FLT x195 =
		x184 +
		(-1 * x180 *
		 ((x189 * x151) + (x166 * x194) + (x167 * x191) +
		  (-1 * x178 *
		   ((x170 * x189) +
			(x147 *
			 ((x176 * x190) +
			  (x132 * ((x191 * x142) +
					   (x132 * ((x132 * (x192 + (-1 * x172 * x191) + (x191 * x140))) + (x191 * x141) + x193)) + x194)) +
			  (x194 * x132) + (x177 * x190))))) +
		  x188));
	const GEN_FLT x196 = 1 + x85;
	const GEN_FLT x197 = x100 * ((-1 * x38 * x196) + x183);
	const GEN_FLT x198 = x185 + (x112 * x196);
	const GEN_FLT x199 = (x116 * x198) + x121;
	const GEN_FLT x200 = x197 + (-1 * x110 * x199);
	const GEN_FLT x201 = (-1 * (x153 + x198) * x154) + x155;
	const GEN_FLT x202 = x201 * x157;
	const GEN_FLT x203 = x202 * x134;
	const GEN_FLT x204 = (x132 * ((-1 * x202 * x133) + x203)) + (x201 * x162);
	const GEN_FLT x205 = (x202 * x136) + (x204 * x132);
	const GEN_FLT x206 =
		x197 +
		(-1 * x180 *
		 ((x200 * x151) + (x205 * x166) +
		  (-1 * x178 *
		   ((x200 * x170) +
			(x147 *
			 ((x201 * x176) +
			  (x132 * ((x201 * x171) +
					   (x132 * ((x132 * (x203 + (-1 * x202 * x172) + (x202 * x140))) + (x202 * x141) + x204)) + x205)) +
			  (x205 * x132) + (x201 * x177))))) +
		  (x202 * x167) + x199));
	const GEN_FLT x207 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x208 = x5 * x207;
	const GEN_FLT x209 = x2 * x208;
	const GEN_FLT x210 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qj *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x211 = x1 * x210;
	const GEN_FLT x212 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qi *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x213 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qk *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x214 = x1 * x207;
	const GEN_FLT x215 = x7 * x214;
	const GEN_FLT x216 = (x47 * x212) + (x8 * x213) + (x4 * x215);
	const GEN_FLT x217 = x50 * x207;
	const GEN_FLT x218 = x1 * x212;
	const GEN_FLT x219 = x2 * x4;
	const GEN_FLT x220 = (x219 * x214) + (x53 * x213) + (x47 * x210);
	const GEN_FLT x221 = -1 * x214;
	const GEN_FLT x222 = (x36 * ((-1 * x209) + (-1 * x211) + x216)) + (x32 * (x217 + x218 + x220)) +
						 (x25 * (x221 + (x56 * x214) + (x57 * x213))) + x84;
	const GEN_FLT x223 = x4 * x208;
	const GEN_FLT x224 = x1 * x213;
	const GEN_FLT x225 = (x2 * x215) + (x53 * x212) + (x8 * x210);
	const GEN_FLT x226 = (x36 * (x221 + (x87 * x212) + (x33 * x214))) + (x32 * ((-1 * x223) + x225 + (-1 * x224))) +
						 (x25 * (x209 + x211 + x216)) + x92;
	const GEN_FLT x227 = ((-1 * x38 * x222) + (x97 * x226)) * x100;
	const GEN_FLT x228 = (x226 * x111) + (x222 * x112);
	const GEN_FLT x229 = (x32 * (x221 + (x214 * x103) + (x210 * x117))) + (x25 * ((-1 * x217) + (-1 * x218) + x220)) +
						 (x36 * (x223 + x225 + x224)) + x118;
	const GEN_FLT x230 = (x228 * x116) + (-1 * x229 * x120);
	const GEN_FLT x231 = x127 * (x227 + (-1 * x230 * x110));
	const GEN_FLT x232 = (-1 * x154 * ((x229 * x152) + x228)) + (x229 * x131);
	const GEN_FLT x233 = x232 * x157;
	const GEN_FLT x234 = x233 * x134;
	const GEN_FLT x235 = (x132 * ((-1 * x233 * x133) + x234)) + (x232 * x162);
	const GEN_FLT x236 = (x233 * x136) + (x235 * x132);
	const GEN_FLT x237 =
		x227 +
		(-1 * x180 *
		 ((x231 * x150) + (x236 * x166) + (x233 * x167) +
		  (-1 * x178 *
		   ((x231 * x169) +
			(x147 *
			 ((x232 * x176) +
			  (x132 * ((x232 * x171) +
					   (x132 * ((x132 * (x234 + (x233 * x140) + (-1 * x233 * x172))) + (x233 * x141) + x235)) + x236)) +
			  (x236 * x132) + (x232 * x177))))) +
		  x230));
	const GEN_FLT x238 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x239 = x5 * x238;
	const GEN_FLT x240 = x2 * x239;
	const GEN_FLT x241 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qj *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x242 = x1 * x241;
	const GEN_FLT x243 = x26 * x238;
	const GEN_FLT x244 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qi *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x245 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qk *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x246 = (x7 * x243) + (x47 * x244) + (x8 * x245);
	const GEN_FLT x247 = x50 * x238;
	const GEN_FLT x248 = x1 * x244;
	const GEN_FLT x249 = (x2 * x243) + (x47 * x241) + (x53 * x245);
	const GEN_FLT x250 = x1 * x238;
	const GEN_FLT x251 = -1 * x250;
	const GEN_FLT x252 = (x36 * ((-1 * x240) + x246 + (-1 * x242))) + (x32 * (x247 + x248 + x249)) +
						 (x25 * (x251 + (x56 * x250) + (x57 * x245))) + x84;
	const GEN_FLT x253 = x4 * x239;
	const GEN_FLT x254 = x1 * x245;
	const GEN_FLT x255 = (x2 * x79 * x238) + (x8 * x241) + (x53 * x244);
	const GEN_FLT x256 = (x36 * ((x33 * x250) + x251 + (x87 * x244))) + (x32 * ((-1 * x253) + (-1 * x254) + x255)) +
						 (x25 * (x240 + x246 + x242)) + x92;
	const GEN_FLT x257 = ((-1 * x38 * x252) + (x97 * x256)) * x100;
	const GEN_FLT x258 = (x256 * x111) + (x252 * x112);
	const GEN_FLT x259 = (x36 * (x253 + x254 + x255)) + (x25 * ((-1 * x247) + (-1 * x248) + x249)) +
						 (x32 * (x251 + (x250 * x103) + (x241 * x117))) + x118;
	const GEN_FLT x260 = (x258 * x116) + (-1 * x259 * x120);
	const GEN_FLT x261 = x257 + (-1 * x260 * x110);
	const GEN_FLT x262 = (-1 * x154 * ((x259 * x152) + x258)) + (x259 * x131);
	const GEN_FLT x263 = x262 * x160;
	const GEN_FLT x264 = (x132 * ((-1 * x262 * x159) + x263)) + (x262 * x162);
	const GEN_FLT x265 = (x262 * x158) + (x264 * x132);
	const GEN_FLT x266 =
		x257 +
		(-1 * x180 *
		 ((x261 * x151) + (x265 * x166) + (x262 * x168) +
		  (-1 * x178 *
		   ((x261 * x170) +
			(x147 * ((x132 * ((x262 * x171) + x265 +
							  (x132 * ((x132 * (x263 + (-1 * x262 * x173) + (x262 * x174))) + (x262 * x175) + x264)))) +
					 (x265 * x132) + (x262 * x177) + (x262 * x176))))) +
		  x260));
	const GEN_FLT x267 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x268 = x5 * x267;
	const GEN_FLT x269 = x2 * x268;
	const GEN_FLT x270 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qj *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x271 = x1 * x270;
	const GEN_FLT x272 = x1 * x267;
	const GEN_FLT x273 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qi *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x274 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qk *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x275 = (x4 * x7 * x272) + (x47 * x273) + (x8 * x274);
	const GEN_FLT x276 = x50 * x267;
	const GEN_FLT x277 = x1 * x273;
	const GEN_FLT x278 = (x219 * x272) + (x53 * x274) + (x47 * x270);
	const GEN_FLT x279 = -1 * x272;
	const GEN_FLT x280 = (x36 * ((-1 * x269) + (-1 * x271) + x275)) + (x32 * (x276 + x277 + x278)) +
						 (x25 * (x279 + (x56 * x272) + (x57 * x274))) + x84;
	const GEN_FLT x281 = x4 * x268;
	const GEN_FLT x282 = x1 * x274;
	const GEN_FLT x283 = (x90 * x272) + (x8 * x270) + (x53 * x273);
	const GEN_FLT x284 = (x36 * (x279 + (x33 * x272) + (x87 * x273))) + (x32 * ((-1 * x281) + x283 + (-1 * x282))) +
						 (x25 * (x269 + x271 + x275)) + x92;
	const GEN_FLT x285 = ((-1 * x38 * x280) + (x97 * x284)) * x100;
	const GEN_FLT x286 = (x284 * x111) + (x280 * x112);
	const GEN_FLT x287 = (x36 * (x281 + x283 + x282)) + (x32 * (x279 + (x272 * x103) + (x270 * x117))) +
						 (x25 * ((-1 * x276) + (-1 * x277) + x278)) + x118;
	const GEN_FLT x288 = (x286 * x116) + (-1 * x287 * x120);
	const GEN_FLT x289 = x285 + (-1 * x288 * x110);
	const GEN_FLT x290 = (-1 * x154 * ((x287 * x152) + x286)) + (x287 * x131);
	const GEN_FLT x291 = x290 * x157;
	const GEN_FLT x292 = x291 * x134;
	const GEN_FLT x293 = (x132 * ((-1 * x291 * x133) + x292)) + (x291 * x135);
	const GEN_FLT x294 = (x291 * x136) + (x293 * x132);
	const GEN_FLT x295 =
		x285 +
		(-1 * x180 *
		 ((x289 * x151) + (x294 * x166) + (x291 * x167) +
		  (-1 * x178 *
		   ((x289 * x170) +
			(x147 *
			 ((x290 * x176) +
			  (x132 * ((x291 * x142) +
					   (x132 * ((x132 * (x292 + (-1 * x291 * x172) + (x291 * x140))) + (x291 * x141) + x293)) + x294)) +
			  (x294 * x132) + (x291 * x137))))) +
		  x288));
	out[0] = (x181 * x182) + x181;
	out[1] = (x182 * x195) + x195;
	out[2] = (x206 * x182) + x206;
	out[3] = (x237 * x182) + x237;
	out[4] = (x266 * x182) + x266;
	out[5] = (x295 * x182) + x295;
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x20 = x15 * x11;
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x20 * x16;
	const GEN_FLT x24 =
		obj_pz + (((x15 * x12) + x14) * sensor_z) + ((x18 + x21) * sensor_y) + (((-1 * x22) + x23) * sensor_x);
	const GEN_FLT x25 = x1 * x7;
	const GEN_FLT x26 = x4 * x6;
	const GEN_FLT x27 = x2 * x26;
	const GEN_FLT x28 = (-1 * x25) + x27;
	const GEN_FLT x29 = x19 * x19;
	const GEN_FLT x30 = x11 * x17;
	const GEN_FLT x31 = x15 * x19;
	const GEN_FLT x32 = x31 * x16;
	const GEN_FLT x33 =
		obj_py + (((-1 * x18) + x21) * sensor_z) + (((x29 * x15) + x14) * sensor_y) + ((x30 + x32) * sensor_x);
	const GEN_FLT x34 = x4 * x4;
	const GEN_FLT x35 = (x6 * x34) + x5;
	const GEN_FLT x36 = x16 * x16;
	const GEN_FLT x37 =
		obj_px + (((x36 * x15) + x14) * sensor_x) + ((x22 + x23) * sensor_z) + (((-1 * x30) + x32) * sensor_y);
	const GEN_FLT x38 = lh_px + (x24 * x10) + (x35 * x37) + (x33 * x28);
	const GEN_FLT x39 = x38 * x38;
	const GEN_FLT x40 = x7 * x7;
	const GEN_FLT x41 = (x6 * x40) + x5;
	const GEN_FLT x42 = x1 * x4;
	const GEN_FLT x43 = x2 * x8;
	const GEN_FLT x44 = x42 + x43;
	const GEN_FLT x45 = (-1 * x3) + x9;
	const GEN_FLT x46 = lh_pz + (x41 * x24) + (x44 * x33) + (x45 * x37);
	const GEN_FLT x47 = (x46 * x46) + x39;
	const GEN_FLT x48 = pow(x47, -1);
	const GEN_FLT x49 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x50 = x5 * x49;
	const GEN_FLT x51 = x2 * x50;
	const GEN_FLT x52 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x53 = x1 * x52;
	const GEN_FLT x54 = x1 * x49;
	const GEN_FLT x55 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x56 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x57 = (x4 * x7 * x54) + (x8 * x55) + (x56 * x26);
	const GEN_FLT x58 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x59 = x15 * x16;
	const GEN_FLT x60 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x61 = x60 * x17;
	const GEN_FLT x62 = -1 * x61;
	const GEN_FLT x63 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x64 = x63 * x17;
	const GEN_FLT x65 = x60 * x14;
	const GEN_FLT x66 = x65 * x11;
	const GEN_FLT x67 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x68 = x60 * x19;
	const GEN_FLT x69 = (x67 * x59) + (x68 * x18) + (x58 * x31);
	const GEN_FLT x70 = x67 * x17;
	const GEN_FLT x71 = x65 * x19;
	const GEN_FLT x72 = (x58 * x20) + (x60 * x11 * x18) + (x63 * x59);
	const GEN_FLT x73 = (((2 * x58 * x59) + x62 + (x61 * x36)) * sensor_x) + ((x70 + x71 + x72) * sensor_z) +
						(((-1 * x64) + (-1 * x66) + x69) * sensor_y);
	const GEN_FLT x74 = x4 * x50;
	const GEN_FLT x75 = x1 * x55;
	const GEN_FLT x76 = x3 * x49;
	const GEN_FLT x77 = x2 * x6;
	const GEN_FLT x78 = (x7 * x76) + (x77 * x56) + (x8 * x52);
	const GEN_FLT x79 = x58 * x17;
	const GEN_FLT x80 = x65 * x16;
	const GEN_FLT x81 = (x67 * x20) + (x68 * x30) + (x63 * x31);
	const GEN_FLT x82 = ((x64 + x66 + x69) * sensor_x) + ((x62 + (2 * x67 * x31) + (x61 * x29)) * sensor_y) +
						(((-1 * x79) + (-1 * x80) + x81) * sensor_z);
	const GEN_FLT x83 = -1 * x54;
	const GEN_FLT x84 = (((-1 * x70) + (-1 * x71) + x72) * sensor_x) + ((x79 + x80 + x81) * sensor_y) +
						((x62 + (2 * x63 * x20) + (x61 * x12)) * sensor_z);
	const GEN_FLT x85 = (x37 * ((-1 * x51) + (-1 * x53) + x57)) + (x73 * x45) + (x84 * x41) +
						(x33 * (x74 + x75 + x78)) + (x82 * x44) + (x24 * (x83 + (x54 * x40) + (2 * x8 * x56)));
	const GEN_FLT x86 = x7 * x50;
	const GEN_FLT x87 = x1 * x56;
	const GEN_FLT x88 = (x77 * x55) + (x52 * x26) + (x4 * x76);
	const GEN_FLT x89 = (x37 * (x83 + (x54 * x34) + (2 * x55 * x26))) + (x73 * x35) +
						(x33 * ((-1 * x86) + (-1 * x87) + x88)) + (x82 * x28) + (x24 * (x51 + x53 + x57)) + (x84 * x10);
	const GEN_FLT x90 = x48 * x39 * ((-1 * x85 * pow(x38, -1)) + (x89 * x46 * pow(x39, -1)));
	const GEN_FLT x91 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x92 = tan(x91);
	const GEN_FLT x93 = (-1 * x42) + x43;
	const GEN_FLT x94 = x2 * x2;
	const GEN_FLT x95 = (x6 * x94) + x5;
	const GEN_FLT x96 = x25 + x27;
	const GEN_FLT x97 = lh_py + (x93 * x24) + (x95 * x33) + (x96 * x37);
	const GEN_FLT x98 = pow(x47, -1.0 / 2.0);
	const GEN_FLT x99 = x98 * x97;
	const GEN_FLT x100 = -1 * x92 * x99;
	const GEN_FLT x101 = atan2(-1 * x46, x38);
	const GEN_FLT x102 = ogeeMag_1 + (-1 * asin(x100)) + x101;
	const GEN_FLT x103 = sin(x102);
	const GEN_FLT x104 = curve_1 + (x103 * ogeePhase_1);
	const GEN_FLT x105 = sin(x91);
	const GEN_FLT x106 = cos(x91);
	const GEN_FLT x107 = pow(x106, -1);
	const GEN_FLT x108 = x97 * x97;
	const GEN_FLT x109 = x108 + x47;
	const GEN_FLT x110 = pow(x109, -1.0 / 2.0);
	const GEN_FLT x111 = x107 * x110;
	const GEN_FLT x112 = asin(x97 * x111);
	const GEN_FLT x113 = 8.0108022e-06 * x112;
	const GEN_FLT x114 = -8.0108022e-06 + (-1 * x113);
	const GEN_FLT x115 = 0.0028679863 + (x112 * x114);
	const GEN_FLT x116 = 5.3685255e-06 + (x112 * x115);
	const GEN_FLT x117 = 0.0076069798 + (x112 * x116);
	const GEN_FLT x118 = x112 * x117;
	const GEN_FLT x119 = -8.0108022e-06 + (-1.60216044e-05 * x112);
	const GEN_FLT x120 = (x112 * x119) + x115;
	const GEN_FLT x121 = (x112 * x120) + x116;
	const GEN_FLT x122 = (x112 * x121) + x117;
	const GEN_FLT x123 = x118 + (x112 * x122);
	const GEN_FLT x124 = x105 * x123;
	const GEN_FLT x125 = (x104 * x124) + x106;
	const GEN_FLT x126 = pow(x125, -1);
	const GEN_FLT x127 = x104 * x126;
	const GEN_FLT x128 = x112 * x112;
	const GEN_FLT x129 = x117 * x128;
	const GEN_FLT x130 = (x127 * x129) + x100;
	const GEN_FLT x131 = pow((1 + (-1 * (x130 * x130))), -1.0 / 2.0);
	const GEN_FLT x132 = x92 * x92;
	const GEN_FLT x133 = pow((1 + (-1 * x48 * x108 * x132)), -1.0 / 2.0);
	const GEN_FLT x134 = (2 * x89 * x38) + (2 * x85 * x46);
	const GEN_FLT x135 = 1.0 / 2.0 * x97;
	const GEN_FLT x136 = (x37 * (x86 + x87 + x88)) + (x33 * (x83 + (x54 * x94) + (2 * x77 * x52))) + (x73 * x96) +
						 (x82 * x95) + (x24 * ((-1 * x74) + (-1 * x75) + x78)) + (x84 * x93);
	const GEN_FLT x137 = (x92 * pow(x47, -3.0 / 2.0) * x134 * x135) + (-1 * x92 * x98 * x136);
	const GEN_FLT x138 = x90 + (-1 * x133 * x137);
	const GEN_FLT x139 = cos(x102) * ogeePhase_1;
	const GEN_FLT x140 = x138 * x139;
	const GEN_FLT x141 = x126 * x129;
	const GEN_FLT x142 = (-1 * pow(x109, -3.0 / 2.0) * x107 * x135 * ((2 * x97 * x136) + x134)) + (x111 * x136);
	const GEN_FLT x143 = pow(x106, -2);
	const GEN_FLT x144 = pow((1 + (-1 * pow(x109, -1) * x108 * x143)), -1.0 / 2.0);
	const GEN_FLT x145 = x142 * x144;
	const GEN_FLT x146 = x114 * x145;
	const GEN_FLT x147 = 2.40324066e-05 * x112;
	const GEN_FLT x148 = (x112 * ((-1 * x113 * x145) + x146)) + (x115 * x145);
	const GEN_FLT x149 = (x116 * x145) + (x112 * x148);
	const GEN_FLT x150 = x105 * x104;
	const GEN_FLT x151 =
		x150 *
		((x122 * x145) +
		 (x112 * ((x121 * x145) +
				  (x112 * ((x112 * (x146 + (-1 * x145 * x147) + (x119 * x145))) + (x120 * x145) + x148)) + x149)) +
		 (x112 * x149) + (x117 * x145));
	const GEN_FLT x152 = x104 * pow(x125, -2) * x129;
	const GEN_FLT x153 = x128 * x127;
	const GEN_FLT x154 = 2 * x118 * x127;
	const GEN_FLT x155 = (x149 * x153) + (x145 * x154) + x137;
	const GEN_FLT x156 = x90 + (-1 * x131 * ((x140 * x141) + (-1 * x152 * ((x124 * x140) + x151)) + x155));
	const GEN_FLT x157 = gibPhase_1 + x101 + (-1 * asin(x130));
	const GEN_FLT x158 = cos(x157) * gibMag_1;
	const GEN_FLT x159 = (x156 * x158) + x156;
	const GEN_FLT x160 = (x99 * (1 + x132)) + x137;
	const GEN_FLT x161 = x90 + (-1 * x160 * x133);
	const GEN_FLT x162 = x139 * x141;
	const GEN_FLT x163 = x144 * ((-1 * x97 * x105 * x110 * x143) + x142);
	const GEN_FLT x164 = x114 * x163;
	const GEN_FLT x165 = (x112 * ((-1 * x113 * x163) + x164)) + (x115 * x163);
	const GEN_FLT x166 = (x116 * x163) + (x112 * x165);
	const GEN_FLT x167 = x124 * x139;
	const GEN_FLT x168 =
		x90 +
		(-1 * x131 *
		 ((x161 * x162) +
		  (-1 * x152 *
		   ((x161 * x167) + x105 + (-1 * x104 * x106 * x123) +
			(x150 *
			 ((x122 * x163) +
			  (x112 * ((x121 * x163) +
					   (x112 * ((x112 * (x164 + (-1 * x163 * x147) + (x119 * x163))) + (x120 * x163) + x165)) + x166)) +
			  (x112 * x166) + (x117 * x163))))) +
		  (x166 * x153) + (x163 * x154) + x160));
	const GEN_FLT x169 = 1 + x140;
	const GEN_FLT x170 = x90 + (-1 * x131 * ((x169 * x141) + (-1 * x152 * ((x124 * x169) + x151)) + x155));
	const GEN_FLT x171 = 1 + x138;
	const GEN_FLT x172 = x90 + (-1 * x131 * ((x162 * x171) + (-1 * x152 * ((x167 * x171) + x151)) + x155));
	const GEN_FLT x173 = x140 + x103;
	const GEN_FLT x174 = x90 + (-1 * x131 * ((x173 * x141) + (-1 * x152 * ((x124 * x173) + x151)) + x155));
	out[0] = -1 + x159;
	out[1] = (x168 * x158) + x168;
	out[2] = (x170 * x158) + x170;
	out[3] = (x158 * (1 + x156)) + x156;
	out[4] = sin(x157) + x159;
	out[5] = (x172 * x158) + x172;
	out[6] = (x174 * x158) + x174;
}

/** Applying function <function reproject at 0x7f35a4af8200> */
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x11 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x12 = cos(x11);
	const GEN_FLT x13 = 1 + (-1 * x12);
	const GEN_FLT x14 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x15 = sin(x11);
	const GEN_FLT x16 = x15 * x14;
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x18 = x13 * x10;
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = x15 * x17;
	const GEN_FLT x21 = x14 * x18;
	const GEN_FLT x22 =
		obj_pz + (((x13 * (x10 * x10)) + x12) * sensor_z) + ((x16 + x19) * sensor_y) + (((-1 * x20) + x21) * sensor_x);
	const GEN_FLT x23 = x15 * x10;
	const GEN_FLT x24 = x14 * x13 * x17;
	const GEN_FLT x25 =
		obj_py + (((-1 * x16) + x19) * sensor_z) + (((x13 * (x17 * x17)) + x12) * sensor_y) + ((x23 + x24) * sensor_x);
	const GEN_FLT x26 = x1 * x4;
	const GEN_FLT x27 = x2 * x8;
	const GEN_FLT x28 =
		obj_px + ((x20 + x21) * sensor_z) + (((-1 * x23) + x24) * sensor_y) + ((((x14 * x14) * x13) + x12) * sensor_x);
	const GEN_FLT x29 = lh_py + (x25 * ((x6 * (x7 * x7)) + x5)) + (((-1 * x3) + x9) * x22) + ((x26 + x27) * x28);
	const GEN_FLT x30 = x1 * x7;
	const GEN_FLT x31 = x2 * x4 * x6;
	const GEN_FLT x32 = lh_pz + (x22 * (((x4 * x4) * x6) + x5)) + ((x3 + x9) * x25) + (((-1 * x30) + x31) * x28);
	const GEN_FLT x33 = -1 * x32;
	const GEN_FLT x34 = x32 * x32;
	const GEN_FLT x35 = lh_px + ((x30 + x31) * x22) + (((-1 * x26) + x27) * x25) + (x28 * (((x2 * x2) * x6) + x5));
	const GEN_FLT x36 = atan2(x35, x33);
	const GEN_FLT x37 = (-1 * asin(x29 * pow((x34 + (x35 * x35)), -1.0 / 2.0) * tilt_0)) + (-1 * phase_0) + (-1 * x36);
	const GEN_FLT x38 =
		(-1 * asin(x35 * pow((x34 + (x29 * x29)), -1.0 / 2.0) * tilt_1)) + (-1 * phase_1) + (-1 * atan2(-1 * x29, x33));
	out[0] = ((atan2(x29, x33) * atan2(x29, x33)) * curve_0) +
			 (-1 * cos(1.5707963267949 + gibPhase_0 + x37) * gibMag_0) + x37;
	out[1] = (-1 * cos(1.5707963267949 + gibPhase_1 + x38) * gibMag_1) + ((x36 * x36) * curve_1) + x38;
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
	const GEN_FLT x0 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (-1 * obj_qk *
							  pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									   ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									   : 1e-10),
								  -2) *
							  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
						   : 0;
	const GEN_FLT x1 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   : 1e-10;
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x5 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 0;
	const GEN_FLT x6 = cos(x1);
	const GEN_FLT x7 = x6 * x5;
	const GEN_FLT x8 = x4 * x7;
	const GEN_FLT x9 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (-1 * obj_qj *
							  pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									   ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									   : 1e-10),
								  -2) *
							  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
						   : 0;
	const GEN_FLT x10 = 1 + (-1 * x6);
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x12 = x11 * x10;
	const GEN_FLT x13 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x14 = x2 * x11;
	const GEN_FLT x15 = x4 * x14;
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x17 = x13 * x10;
	const GEN_FLT x18 = (x9 * x12) + (x15 * x13) + (x17 * x16);
	const GEN_FLT x19 = x2 * x4;
	const GEN_FLT x20 = -1 * x19;
	const GEN_FLT x21 = 2 * x17;
	const GEN_FLT x22 = x13 * x13;
	const GEN_FLT x23 = x2 * x16;
	const GEN_FLT x24 = x4 * x6;
	const GEN_FLT x25 = x24 * x11;
	const GEN_FLT x26 = x5 * x10;
	const GEN_FLT x27 = x2 * x13;
	const GEN_FLT x28 = (x9 * x26) + (x0 * x17) + (x4 * x5 * x27);
	const GEN_FLT x29 = ((x3 + x8 + x18) * sensor_x) + ((x20 + (x9 * x21) + (x22 * x19)) * sensor_y) +
						(((-1 * x23) + (-1 * x25) + x28) * sensor_z);
	const GEN_FLT x30 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							: 1e-10;
	const GEN_FLT x31 = sin(x30);
	const GEN_FLT x32 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 1;
	const GEN_FLT x33 = x32 * x31;
	const GEN_FLT x34 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 0;
	const GEN_FLT x35 = cos(x30);
	const GEN_FLT x36 = 1 + (-1 * x35);
	const GEN_FLT x37 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 0;
	const GEN_FLT x38 = x36 * x37;
	const GEN_FLT x39 = x34 * x38;
	const GEN_FLT x40 = x33 + x39;
	const GEN_FLT x41 = x40 * x29;
	const GEN_FLT x42 = x31 * x34;
	const GEN_FLT x43 = x32 * x36;
	const GEN_FLT x44 = x43 * x37;
	const GEN_FLT x45 = (-1 * x42) + x44;
	const GEN_FLT x46 = 2 * x12;
	const GEN_FLT x47 = x11 * x11;
	const GEN_FLT x48 = x2 * x9;
	const GEN_FLT x49 = x24 * x13;
	const GEN_FLT x50 = (x26 * x16) + (x5 * x15) + (x0 * x12);
	const GEN_FLT x51 = (((x46 * x16) + x20 + (x47 * x19)) * sensor_x) + (((-1 * x3) + (-1 * x8) + x18) * sensor_y) +
						((x48 + x49 + x50) * sensor_z);
	const GEN_FLT x52 = 1 + x51;
	const GEN_FLT x53 = x37 * x37;
	const GEN_FLT x54 = (x53 * x36) + x35;
	const GEN_FLT x55 = 2 * x26;
	const GEN_FLT x56 = x5 * x5;
	const GEN_FLT x57 = (((-1 * x48) + (-1 * x49) + x50) * sensor_x) + ((x23 + x25 + x28) * sensor_y) +
						((x20 + (x0 * x55) + (x56 * x19)) * sensor_z);
	const GEN_FLT x58 = x26 * x11;
	const GEN_FLT x59 = x2 * x5;
	const GEN_FLT x60 = x11 * x17;
	const GEN_FLT x61 =
		obj_px + ((x27 + x58) * sensor_z) + (((-1 * x59) + x60) * sensor_y) + (((x47 * x10) + x6) * sensor_x);
	const GEN_FLT x62 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x63 = x62 * x35;
	const GEN_FLT x64 = x63 * x34;
	const GEN_FLT x65 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x66 = x65 * x31;
	const GEN_FLT x67 = x62 * x31;
	const GEN_FLT x68 = x67 * x37;
	const GEN_FLT x69 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x70 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x71 = (x68 * x32) + (x69 * x38) + (x70 * x43);
	const GEN_FLT x72 = x26 * x13;
	const GEN_FLT x73 =
		obj_pz + (((x56 * x10) + x6) * sensor_z) + (((-1 * x27) + x58) * sensor_x) + ((x14 + x72) * sensor_y);
	const GEN_FLT x74 = -1 * x67;
	const GEN_FLT x75 = x63 * x32;
	const GEN_FLT x76 = x69 * x31;
	const GEN_FLT x77 = x34 * x36;
	const GEN_FLT x78 = (x68 * x34) + (x70 * x77) + (x65 * x38);
	const GEN_FLT x79 =
		obj_py + (((x22 * x10) + x6) * sensor_y) + ((x59 + x60) * sensor_x) + (((-1 * x14) + x72) * sensor_z);
	const GEN_FLT x80 = (x61 * ((-1 * x64) + (-1 * x66) + x71)) + (x73 * (x74 + (2 * x70 * x38) + (x67 * x53))) +
						(x79 * (x75 + x76 + x78));
	const GEN_FLT x81 = (x54 * x57) + x80;
	const GEN_FLT x82 = x41 + (x52 * x45) + x81;
	const GEN_FLT x83 = x42 + x44;
	const GEN_FLT x84 = x31 * x37;
	const GEN_FLT x85 = x43 * x34;
	const GEN_FLT x86 = (-1 * x84) + x85;
	const GEN_FLT x87 = x32 * x32;
	const GEN_FLT x88 = (x87 * x36) + x35;
	const GEN_FLT x89 = lh_px + (x83 * x73) + (x88 * x61) + (x86 * x79);
	const GEN_FLT x90 = lh_pz + (x73 * x54) + (x79 * x40) + (x61 * x45);
	const GEN_FLT x91 = x90 * x90;
	const GEN_FLT x92 = pow(x91, -1);
	const GEN_FLT x93 = x89 * x92;
	const GEN_FLT x94 = pow(x90, -1);
	const GEN_FLT x95 = x83 * x57;
	const GEN_FLT x96 = x86 * x29;
	const GEN_FLT x97 = x63 * x37;
	const GEN_FLT x98 = x70 * x31;
	const GEN_FLT x99 = (x77 * x69) + (x65 * x43) + (x67 * x32 * x34);
	const GEN_FLT x100 = (x61 * (x74 + (x87 * x67) + (2 * x69 * x43))) + (x79 * ((-1 * x97) + (-1 * x98) + x99)) +
						 (x73 * (x64 + x66 + x71));
	const GEN_FLT x101 = x95 + x96 + (x88 * x52) + x100;
	const GEN_FLT x102 = x89 * x89;
	const GEN_FLT x103 = x102 + x91;
	const GEN_FLT x104 = pow(x103, -1);
	const GEN_FLT x105 = x91 * x104;
	const GEN_FLT x106 = x105 * ((x82 * x93) + (-1 * x94 * x101));
	const GEN_FLT x107 = (-1 * x33) + x39;
	const GEN_FLT x108 = x34 * x34;
	const GEN_FLT x109 = (x36 * x108) + x35;
	const GEN_FLT x110 = x84 + x85;
	const GEN_FLT x111 = lh_py + (x73 * x107) + (x79 * x109) + (x61 * x110);
	const GEN_FLT x112 = x111 * x111;
	const GEN_FLT x113 = pow((1 + (-1 * x104 * x112 * (tilt_0 * tilt_0))), -1.0 / 2.0);
	const GEN_FLT x114 = 2 * x89;
	const GEN_FLT x115 = 2 * x90;
	const GEN_FLT x116 = x82 * x115;
	const GEN_FLT x117 = 1.0 / 2.0 * pow(x103, -3.0 / 2.0) * x111 * tilt_0;
	const GEN_FLT x118 = x29 * x109;
	const GEN_FLT x119 = (x79 * (x74 + (x67 * x108) + (2 * x77 * x65))) + (x73 * ((-1 * x75) + (-1 * x76) + x78)) +
						 (x61 * (x97 + x98 + x99));
	const GEN_FLT x120 = (x57 * x107) + x119;
	const GEN_FLT x121 = (x52 * x110) + x118 + x120;
	const GEN_FLT x122 = pow(x103, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x123 = (-1 * x106) + (-1 * x113 * ((-1 * x117 * ((x101 * x114) + x116)) + (x122 * x121)));
	const GEN_FLT x124 = -1 * x90;
	const GEN_FLT x125 = atan2(x89, x124);
	const GEN_FLT x126 =
		sin(1.5707963267949 + gibPhase_0 + (-1 * asin(x111 * x122)) + (-1 * phase_0) + (-1 * x125)) * gibMag_0;
	const GEN_FLT x127 = x94 * x121;
	const GEN_FLT x128 = x92 * x111;
	const GEN_FLT x129 = x82 * x128;
	const GEN_FLT x130 = x112 + x91;
	const GEN_FLT x131 = pow(x130, -1);
	const GEN_FLT x132 = x91 * x131;
	const GEN_FLT x133 = 2 * x132 * atan2(x111, x124) * curve_0;
	const GEN_FLT x134 = x51 * x45;
	const GEN_FLT x135 = 1 + x29;
	const GEN_FLT x136 = x134 + (x40 * x135) + x81;
	const GEN_FLT x137 = (x88 * x51) + x100;
	const GEN_FLT x138 = (x86 * x135) + x95 + x137;
	const GEN_FLT x139 = ((x93 * x136) + (-1 * x94 * x138)) * x105;
	const GEN_FLT x140 = x115 * x136;
	const GEN_FLT x141 = x51 * x110;
	const GEN_FLT x142 = x141 + (x109 * x135) + x120;
	const GEN_FLT x143 = (-1 * x139) + (-1 * x113 * ((-1 * x117 * ((x114 * x138) + x140)) + (x122 * x142)));
	const GEN_FLT x144 = x94 * x142;
	const GEN_FLT x145 = x128 * x136;
	const GEN_FLT x146 = 1 + x57;
	const GEN_FLT x147 = x134 + x41 + (x54 * x146) + x80;
	const GEN_FLT x148 = x96 + (x83 * x146) + x137;
	const GEN_FLT x149 = ((x93 * x147) + (-1 * x94 * x148)) * x105;
	const GEN_FLT x150 = x115 * x147;
	const GEN_FLT x151 = x141 + x118 + (x107 * x146) + x119;
	const GEN_FLT x152 = (-1 * x149) + (-1 * x113 * ((-1 * x117 * ((x114 * x148) + x150)) + (x122 * x151)));
	const GEN_FLT x153 = x94 * x151;
	const GEN_FLT x154 = x128 * x147;
	const GEN_FLT x155 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x156 = x2 * x155;
	const GEN_FLT x157 = -1 * x156;
	const GEN_FLT x158 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qi *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x159 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qk *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x160 = x2 * x159;
	const GEN_FLT x161 = x7 * x155;
	const GEN_FLT x162 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qj *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x163 = x13 * x11;
	const GEN_FLT x164 = (x17 * x158) + (x12 * x162) + (x163 * x156);
	const GEN_FLT x165 = x2 * x162;
	const GEN_FLT x166 = x6 * x13;
	const GEN_FLT x167 = x166 * x155;
	const GEN_FLT x168 = x5 * x11;
	const GEN_FLT x169 = (x26 * x158) + (x12 * x159) + (x168 * x156);
	const GEN_FLT x170 = ((x157 + (x46 * x158) + (x47 * x156)) * sensor_x) +
						 (((-1 * x160) + x164 + (-1 * x161)) * sensor_y) + ((x165 + x167 + x169) * sensor_z);
	const GEN_FLT x171 = x2 * x158;
	const GEN_FLT x172 = x6 * x11;
	const GEN_FLT x173 = x172 * x155;
	const GEN_FLT x174 = x5 * x13;
	const GEN_FLT x175 = (x26 * x162) + (x17 * x159) + (x174 * x156);
	const GEN_FLT x176 = ((x160 + x164 + x161) * sensor_x) + (((x21 * x162) + x157 + (x22 * x156)) * sensor_y) +
						 (((-1 * x171) + x175 + (-1 * x173)) * sensor_z);
	const GEN_FLT x177 = (((-1 * x165) + (-1 * x167) + x169) * sensor_x) + ((x171 + x175 + x173) * sensor_y) +
						 ((x157 + (x55 * x159) + (x56 * x156)) * sensor_z);
	const GEN_FLT x178 = (x45 * x170) + (x54 * x177) + (x40 * x176) + x80;
	const GEN_FLT x179 = (x88 * x170) + (x86 * x176) + (x83 * x177) + x100;
	const GEN_FLT x180 = ((x93 * x178) + (-1 * x94 * x179)) * x105;
	const GEN_FLT x181 = x115 * x178;
	const GEN_FLT x182 = (x110 * x170) + (x109 * x176) + (x107 * x177) + x119;
	const GEN_FLT x183 = (-1 * x180) + (-1 * x113 * ((-1 * x117 * ((x114 * x179) + x181)) + (x122 * x182)));
	const GEN_FLT x184 = x94 * x182;
	const GEN_FLT x185 = x128 * x178;
	const GEN_FLT x186 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x187 = x2 * x186;
	const GEN_FLT x188 = -1 * x187;
	const GEN_FLT x189 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qi *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x190 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qk *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x191 = x2 * x190;
	const GEN_FLT x192 = x7 * x186;
	const GEN_FLT x193 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qj *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x194 = (x17 * x189) + (x12 * x193) + (x163 * x187);
	const GEN_FLT x195 = x2 * x193;
	const GEN_FLT x196 = x166 * x186;
	const GEN_FLT x197 = (x26 * x189) + (x168 * x187) + (x12 * x190);
	const GEN_FLT x198 = ((x188 + (x46 * x189) + (x47 * x187)) * sensor_x) +
						 (((-1 * x191) + (-1 * x192) + x194) * sensor_y) + ((x195 + x196 + x197) * sensor_z);
	const GEN_FLT x199 = x2 * x189;
	const GEN_FLT x200 = x172 * x186;
	const GEN_FLT x201 = (x17 * x190) + (x174 * x187) + (x26 * x193);
	const GEN_FLT x202 = ((x188 + (x21 * x193) + (x22 * x187)) * sensor_y) + ((x191 + x192 + x194) * sensor_x) +
						 (((-1 * x199) + (-1 * x200) + x201) * sensor_z);
	const GEN_FLT x203 = (((-1 * x195) + (-1 * x196) + x197) * sensor_x) + ((x199 + x200 + x201) * sensor_y) +
						 ((x188 + (x55 * x190) + (x56 * x187)) * sensor_z);
	const GEN_FLT x204 = (x45 * x198) + (x40 * x202) + (x54 * x203) + x80;
	const GEN_FLT x205 = (x88 * x198) + (x83 * x203) + x100 + (x86 * x202);
	const GEN_FLT x206 = ((x93 * x204) + (-1 * x94 * x205)) * x105;
	const GEN_FLT x207 = x204 * x115;
	const GEN_FLT x208 = (x110 * x198) + (x202 * x109) + (x203 * x107) + x119;
	const GEN_FLT x209 = (-1 * x206) + (-1 * x113 * ((-1 * x117 * ((x205 * x114) + x207)) + (x208 * x122)));
	const GEN_FLT x210 = x94 * x208;
	const GEN_FLT x211 = x204 * x128;
	const GEN_FLT x212 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x213 = x2 * x212;
	const GEN_FLT x214 = -1 * x213;
	const GEN_FLT x215 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qi *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x216 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qk *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x217 = x2 * x216;
	const GEN_FLT x218 = x7 * x212;
	const GEN_FLT x219 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qj *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x220 = (x17 * x215) + (x12 * x219) + (x213 * x163);
	const GEN_FLT x221 = x2 * x219;
	const GEN_FLT x222 = x212 * x166;
	const GEN_FLT x223 = (x26 * x215) + (x213 * x168) + (x12 * x216);
	const GEN_FLT x224 = ((x214 + (x46 * x215) + (x47 * x213)) * sensor_x) +
						 (((-1 * x217) + (-1 * x218) + x220) * sensor_y) + ((x221 + x222 + x223) * sensor_z);
	const GEN_FLT x225 = x2 * x215;
	const GEN_FLT x226 = x212 * x172;
	const GEN_FLT x227 = (x26 * x219) + (x17 * x216) + (x213 * x174);
	const GEN_FLT x228 = ((x217 + x218 + x220) * sensor_x) + (((x21 * x219) + x214 + (x22 * x213)) * sensor_y) +
						 (((-1 * x225) + (-1 * x226) + x227) * sensor_z);
	const GEN_FLT x229 = (((-1 * x221) + (-1 * x222) + x223) * sensor_x) + ((x225 + x226 + x227) * sensor_y) +
						 ((x214 + (x55 * x216) + (x56 * x213)) * sensor_z);
	const GEN_FLT x230 = (x45 * x224) + (x40 * x228) + (x54 * x229) + x80;
	const GEN_FLT x231 = (x88 * x224) + (x86 * x228) + (x83 * x229) + x100;
	const GEN_FLT x232 = ((x93 * x230) + (-1 * x94 * x231)) * x105;
	const GEN_FLT x233 = x230 * x115;
	const GEN_FLT x234 = (x224 * x110) + (x228 * x109) + (x229 * x107) + x119;
	const GEN_FLT x235 = (-1 * x232) + (-1 * x113 * ((-1 * x117 * ((x231 * x114) + x233)) + (x234 * x122)));
	const GEN_FLT x236 = x94 * x234;
	const GEN_FLT x237 = x230 * x128;
	const GEN_FLT x238 = 2 * x125 * curve_1;
	const GEN_FLT x239 = 2 * x111;
	const GEN_FLT x240 = 1.0 / 2.0 * x89 * pow(x130, -3.0 / 2.0) * tilt_1;
	const GEN_FLT x241 = pow(x130, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x242 = pow((1 + (-1 * x102 * x131 * (tilt_1 * tilt_1))), -1.0 / 2.0);
	const GEN_FLT x243 =
		(-1 * (x127 + (-1 * x129)) * x132) + (-1 * x242 * ((-1 * x240 * ((x239 * x121) + x116)) + (x241 * x101)));
	const GEN_FLT x244 =
		sin(1.5707963267949 + gibPhase_1 + (-1 * asin(x89 * x241)) + (-1 * phase_1) + (-1 * atan2(-1 * x111, x124))) *
		gibMag_1;
	const GEN_FLT x245 =
		(-1 * (x144 + (-1 * x145)) * x132) + (-1 * x242 * ((-1 * x240 * ((x239 * x142) + x140)) + (x241 * x138)));
	const GEN_FLT x246 =
		(-1 * (x153 + (-1 * x154)) * x132) + (-1 * x242 * ((-1 * x240 * ((x239 * x151) + x150)) + (x241 * x148)));
	const GEN_FLT x247 =
		(-1 * (x184 + (-1 * x185)) * x132) + (-1 * x242 * ((-1 * x240 * ((x239 * x182) + x181)) + (x241 * x179)));
	const GEN_FLT x248 =
		(-1 * (x210 + (-1 * x211)) * x132) + (-1 * x242 * ((-1 * x240 * ((x239 * x208) + x207)) + (x205 * x241)));
	const GEN_FLT x249 =
		(-1 * (x236 + (-1 * x237)) * x132) + (-1 * x242 * ((-1 * x240 * ((x234 * x239) + x233)) + (x231 * x241)));
	out[0] = (x123 * x126) + (((-1 * x127) + x129) * x133) + x123;
	out[1] = (x126 * x143) + x143 + (((-1 * x144) + x145) * x133);
	out[2] = (x126 * x152) + (((-1 * x153) + x154) * x133) + x152;
	out[3] = (x126 * x183) + x183 + (((-1 * x184) + x185) * x133);
	out[4] = (x209 * x126) + (((-1 * x210) + x211) * x133) + x209;
	out[5] = (x235 * x126) + (((-1 * x236) + x237) * x133) + x235;
	out[6] = (x238 * x106) + (x243 * x244) + x243;
	out[7] = (x238 * x139) + (x244 * x245) + x245;
	out[8] = (x238 * x149) + (x244 * x246) + x246;
	out[9] = (x238 * x180) + (x244 * x247) + x247;
	out[10] = (x238 * x206) + (x244 * x248) + x248;
	out[11] = (x232 * x238) + (x244 * x249) + x249;
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x5 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (x15 * x12) + x14;
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x18 = sin(x13);
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x21 * x11;
	const GEN_FLT x23 = x19 + x22;
	const GEN_FLT x24 = x20 * x18;
	const GEN_FLT x25 = x15 * x17;
	const GEN_FLT x26 = x25 * x11;
	const GEN_FLT x27 = (-1 * x24) + x26;
	const GEN_FLT x28 = obj_pz + (x27 * sensor_x) + (x16 * sensor_z) + (x23 * sensor_y);
	const GEN_FLT x29 = x1 * x5;
	const GEN_FLT x30 = x2 * x7;
	const GEN_FLT x31 = x4 * x30;
	const GEN_FLT x32 = (-1 * x29) + x31;
	const GEN_FLT x33 = (-1 * x19) + x22;
	const GEN_FLT x34 = x20 * x20;
	const GEN_FLT x35 = (x34 * x15) + x14;
	const GEN_FLT x36 = x11 * x18;
	const GEN_FLT x37 = x25 * x20;
	const GEN_FLT x38 = x36 + x37;
	const GEN_FLT x39 = obj_py + (x33 * sensor_z) + (x35 * sensor_y) + (x38 * sensor_x);
	const GEN_FLT x40 = x4 * x4;
	const GEN_FLT x41 = (x7 * x40) + x6;
	const GEN_FLT x42 = x24 + x26;
	const GEN_FLT x43 = (-1 * x36) + x37;
	const GEN_FLT x44 = x17 * x17;
	const GEN_FLT x45 = (x44 * x15) + x14;
	const GEN_FLT x46 = obj_px + (x42 * sensor_z) + (x43 * sensor_y) + (x45 * sensor_x);
	const GEN_FLT x47 = lh_px + (x28 * x10) + (x32 * x39) + (x41 * x46);
	const GEN_FLT x48 = x5 * x5;
	const GEN_FLT x49 = (x7 * x48) + x6;
	const GEN_FLT x50 = x1 * x4;
	const GEN_FLT x51 = x5 * x30;
	const GEN_FLT x52 = x50 + x51;
	const GEN_FLT x53 = (-1 * x3) + x9;
	const GEN_FLT x54 = lh_pz + (x49 * x28) + (x52 * x39) + (x53 * x46);
	const GEN_FLT x55 = x54 * x54;
	const GEN_FLT x56 = pow(x55, -1);
	const GEN_FLT x57 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x58 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x59 = x58 * x18;
	const GEN_FLT x60 = -1 * x59;
	const GEN_FLT x61 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x62 = x61 * x18;
	const GEN_FLT x63 = x58 * x14;
	const GEN_FLT x64 = x63 * x11;
	const GEN_FLT x65 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x66 = x58 * x19;
	const GEN_FLT x67 = (x65 * x25) + (x66 * x20) + (x57 * x21);
	const GEN_FLT x68 = x65 * x18;
	const GEN_FLT x69 = x63 * x20;
	const GEN_FLT x70 = x15 * x11;
	const GEN_FLT x71 = (x70 * x57) + (x61 * x25) + (x66 * x11);
	const GEN_FLT x72 = (((2 * x57 * x25) + x60 + (x59 * x44)) * sensor_x) +
						(((-1 * x62) + (-1 * x64) + x67) * sensor_y) + ((x68 + x69 + x71) * sensor_z);
	const GEN_FLT x73 = x45 + x72;
	const GEN_FLT x74 = x57 * x18;
	const GEN_FLT x75 = x63 * x17;
	const GEN_FLT x76 = (x70 * x65) + (x61 * x21) + (x58 * x36 * x20);
	const GEN_FLT x77 = ((x62 + x64 + x67) * sensor_x) + ((x60 + (2 * x65 * x21) + (x59 * x34)) * sensor_y) +
						(((-1 * x74) + (-1 * x75) + x76) * sensor_z);
	const GEN_FLT x78 = x38 + x77;
	const GEN_FLT x79 = (((-1 * x68) + (-1 * x69) + x71) * sensor_x) + ((x74 + x75 + x76) * sensor_y) +
						((x60 + (2 * x70 * x61) + (x59 * x12)) * sensor_z);
	const GEN_FLT x80 = x27 + x79;
	const GEN_FLT x81 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x82 = x6 * x81;
	const GEN_FLT x83 = x2 * x82;
	const GEN_FLT x84 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x85 = x1 * x84;
	const GEN_FLT x86 = x4 * x81;
	const GEN_FLT x87 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x88 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x89 = x4 * x7;
	const GEN_FLT x90 = (x86 * x29) + (x8 * x87) + (x88 * x89);
	const GEN_FLT x91 = x4 * x82;
	const GEN_FLT x92 = x1 * x87;
	const GEN_FLT x93 = (x3 * x5 * x81) + (x88 * x30) + (x8 * x84);
	const GEN_FLT x94 = x1 * x81;
	const GEN_FLT x95 = -1 * x94;
	const GEN_FLT x96 = (x46 * ((-1 * x83) + (-1 * x85) + x90)) + (x39 * (x91 + x92 + x93)) +
						(x28 * (x95 + (x94 * x48) + (2 * x8 * x88)));
	const GEN_FLT x97 = (x73 * x53) + (x78 * x52) + (x80 * x49) + x96;
	const GEN_FLT x98 = x56 * x97;
	const GEN_FLT x99 = pow(x54, -1);
	const GEN_FLT x100 = x5 * x82;
	const GEN_FLT x101 = x1 * x88;
	const GEN_FLT x102 = (x87 * x30) + (x89 * x84) + (x3 * x86);
	const GEN_FLT x103 = (x39 * ((-1 * x100) + (-1 * x101) + x102)) + (x28 * (x83 + x85 + x90)) +
						 (x46 * (x95 + (x94 * x40) + (2 * x89 * x87)));
	const GEN_FLT x104 = (x73 * x41) + (x78 * x32) + (x80 * x10) + x103;
	const GEN_FLT x105 = x47 * x47;
	const GEN_FLT x106 = x105 + x55;
	const GEN_FLT x107 = pow(x106, -1);
	const GEN_FLT x108 = x55 * x107;
	const GEN_FLT x109 = x108 * ((x98 * x47) + (-1 * x99 * x104));
	const GEN_FLT x110 = (-1 * x50) + x51;
	const GEN_FLT x111 = x2 * x2;
	const GEN_FLT x112 = (x7 * x111) + x6;
	const GEN_FLT x113 = x29 + x31;
	const GEN_FLT x114 = lh_py + (x28 * x110) + (x39 * x112) + (x46 * x113);
	const GEN_FLT x115 = x114 * x114;
	const GEN_FLT x116 = pow((1 + (-1 * x107 * x115 * (tilt_0 * tilt_0))), -1.0 / 2.0);
	const GEN_FLT x117 = 2 * x47;
	const GEN_FLT x118 = 2 * x54;
	const GEN_FLT x119 = x97 * x118;
	const GEN_FLT x120 = 1.0 / 2.0 * pow(x106, -3.0 / 2.0) * x114 * tilt_0;
	const GEN_FLT x121 = (x46 * (x100 + x101 + x102)) + (x39 * (x95 + (x94 * x111) + (2 * x84 * x30))) +
						 (x28 * ((-1 * x91) + (-1 * x92) + x93));
	const GEN_FLT x122 = (x73 * x113) + (x78 * x112) + (x80 * x110) + x121;
	const GEN_FLT x123 = pow(x106, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x124 = (-1 * x109) + (-1 * x116 * ((-1 * x120 * ((x104 * x117) + x119)) + (x123 * x122)));
	const GEN_FLT x125 = -1 * x54;
	const GEN_FLT x126 = atan2(x47, x125);
	const GEN_FLT x127 =
		sin(1.5707963267949 + gibPhase_0 + (-1 * asin(x114 * x123)) + (-1 * phase_0) + (-1 * x126)) * gibMag_0;
	const GEN_FLT x128 = x99 * x122;
	const GEN_FLT x129 = x98 * x114;
	const GEN_FLT x130 = x115 + x55;
	const GEN_FLT x131 = pow(x130, -1);
	const GEN_FLT x132 = x55 * x131;
	const GEN_FLT x133 = 2 * x132 * atan2(x114, x125) * curve_0;
	const GEN_FLT x134 = x43 + x72;
	const GEN_FLT x135 = x35 + x77;
	const GEN_FLT x136 = x23 + x79;
	const GEN_FLT x137 = (x53 * x134) + (x52 * x135) + (x49 * x136) + x96;
	const GEN_FLT x138 = x56 * x137;
	const GEN_FLT x139 = (x41 * x134) + x103 + (x32 * x135) + (x10 * x136);
	const GEN_FLT x140 = ((x47 * x138) + (-1 * x99 * x139)) * x108;
	const GEN_FLT x141 = x118 * x137;
	const GEN_FLT x142 = (x113 * x134) + (x112 * x135) + (x110 * x136) + x121;
	const GEN_FLT x143 = (-1 * x140) + (-1 * x116 * ((-1 * x120 * ((x117 * x139) + x141)) + (x123 * x142)));
	const GEN_FLT x144 = x99 * x142;
	const GEN_FLT x145 = x114 * x138;
	const GEN_FLT x146 = x42 + x72;
	const GEN_FLT x147 = x33 + x77;
	const GEN_FLT x148 = x16 + x79;
	const GEN_FLT x149 = (x53 * x146) + (x52 * x147) + (x49 * x148) + x96;
	const GEN_FLT x150 = x56 * x149;
	const GEN_FLT x151 = (x41 * x146) + (x32 * x147) + (x10 * x148) + x103;
	const GEN_FLT x152 = ((x47 * x150) + (-1 * x99 * x151)) * x108;
	const GEN_FLT x153 = x118 * x149;
	const GEN_FLT x154 = (x112 * x147) + (x113 * x146) + (x110 * x148) + x121;
	const GEN_FLT x155 = (-1 * x152) + (-1 * x116 * ((-1 * x120 * ((x117 * x151) + x153)) + (x123 * x154)));
	const GEN_FLT x156 = x99 * x154;
	const GEN_FLT x157 = x114 * x150;
	const GEN_FLT x158 = 2 * x126 * curve_1;
	const GEN_FLT x159 = 2 * x114;
	const GEN_FLT x160 = 1.0 / 2.0 * x47 * pow(x130, -3.0 / 2.0) * tilt_1;
	const GEN_FLT x161 = pow(x130, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x162 = pow((1 + (-1 * x105 * x131 * (tilt_1 * tilt_1))), -1.0 / 2.0);
	const GEN_FLT x163 =
		(-1 * (x128 + (-1 * x129)) * x132) + (-1 * x162 * ((-1 * x160 * ((x122 * x159) + x119)) + (x104 * x161)));
	const GEN_FLT x164 =
		sin(1.5707963267949 + gibPhase_1 + (-1 * asin(x47 * x161)) + (-1 * phase_1) + (-1 * atan2(-1 * x114, x125))) *
		gibMag_1;
	const GEN_FLT x165 =
		(-1 * (x144 + (-1 * x145)) * x132) + (-1 * x162 * ((-1 * x160 * ((x142 * x159) + x141)) + (x161 * x139)));
	const GEN_FLT x166 =
		(-1 * (x156 + (-1 * x157)) * x132) + (-1 * x162 * ((-1 * x160 * ((x154 * x159) + x153)) + (x161 * x151)));
	out[0] = (x124 * x127) + (((-1 * x128) + x129) * x133) + x124;
	out[1] = (x127 * x143) + (((-1 * x144) + x145) * x133) + x143;
	out[2] = (x127 * x155) + x155 + (((-1 * x156) + x157) * x133);
	out[3] = (x109 * x158) + (x164 * x163) + x163;
	out[4] = (x140 * x158) + (x165 * x164) + x165;
	out[5] = (x152 * x158) + (x166 * x164) + x166;
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
	const GEN_FLT x0 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 0;
	const GEN_FLT x1 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   : 1e-10;
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 1;
	const GEN_FLT x5 = cos(x1);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x2 * x7;
	const GEN_FLT x11 = x0 * x6;
	const GEN_FLT x12 = x4 * x11;
	const GEN_FLT x13 = x4 * x4;
	const GEN_FLT x14 =
		obj_px + ((x3 + x9) * sensor_z) + (((-1 * x10) + x12) * sensor_y) + (((x6 * x13) + x5) * sensor_x);
	const GEN_FLT x15 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 0;
	const GEN_FLT x16 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x17 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							: 1e-10;
	const GEN_FLT x18 = cos(x17);
	const GEN_FLT x19 = x18 * x16;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = sin(x17);
	const GEN_FLT x22 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 0;
	const GEN_FLT x25 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 1;
	const GEN_FLT x26 = x21 * x16;
	const GEN_FLT x27 = x25 * x26;
	const GEN_FLT x28 = 1 + (-1 * x18);
	const GEN_FLT x29 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x30 = x28 * x29;
	const GEN_FLT x31 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x32 = x25 * x28;
	const GEN_FLT x33 = (x24 * x27) + (x30 * x24) + (x32 * x31);
	const GEN_FLT x34 = x25 * x19;
	const GEN_FLT x35 = x21 * x29;
	const GEN_FLT x36 = x24 * x15;
	const GEN_FLT x37 = x28 * x15;
	const GEN_FLT x38 = x24 * x28;
	const GEN_FLT x39 = (x36 * x26) + (x31 * x37) + (x38 * x22);
	const GEN_FLT x40 = x2 * x4;
	const GEN_FLT x41 = x7 * x11;
	const GEN_FLT x42 = x0 * x0;
	const GEN_FLT x43 =
		obj_py + (((-1 * x40) + x41) * sensor_z) + (((x6 * x42) + x5) * sensor_y) + ((x10 + x12) * sensor_x);
	const GEN_FLT x44 = x7 * x7;
	const GEN_FLT x45 =
		obj_pz + (((x6 * x44) + x5) * sensor_z) + ((x40 + x41) * sensor_y) + (((-1 * x3) + x9) * sensor_x);
	const GEN_FLT x46 = -1 * x26;
	const GEN_FLT x47 = x24 * x24;
	const GEN_FLT x48 = 2 * x38;
	const GEN_FLT x49 = x21 * x15;
	const GEN_FLT x50 = x32 * x24;
	const GEN_FLT x51 = (-1 * x49) + x50;
	const GEN_FLT x52 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x53 = x4 * x6;
	const GEN_FLT x54 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x55 = x2 * x54;
	const GEN_FLT x56 = -1 * x55;
	const GEN_FLT x57 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x58 = x2 * x57;
	const GEN_FLT x59 = x5 * x54;
	const GEN_FLT x60 = x7 * x59;
	const GEN_FLT x61 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x62 = x54 * x40;
	const GEN_FLT x63 = (x61 * x53) + (x0 * x62) + (x52 * x11);
	const GEN_FLT x64 = x2 * x61;
	const GEN_FLT x65 = x0 * x59;
	const GEN_FLT x66 = (x8 * x52) + (x7 * x62) + (x53 * x57);
	const GEN_FLT x67 = (((2 * x53 * x52) + x56 + (x55 * x13)) * sensor_x) +
						(((-1 * x58) + (-1 * x60) + x63) * sensor_y) + ((x64 + x66 + x65) * sensor_z);
	const GEN_FLT x68 = x2 * x52;
	const GEN_FLT x69 = x4 * x59;
	const GEN_FLT x70 = (x8 * x61) + (x57 * x11) + (x0 * x7 * x55);
	const GEN_FLT x71 = ((x58 + x60 + x63) * sensor_x) + ((x56 + (2 * x61 * x11) + (x55 * x42)) * sensor_y) +
						(((-1 * x68) + x70 + (-1 * x69)) * sensor_z);
	const GEN_FLT x72 = x25 * x21;
	const GEN_FLT x73 = x37 * x24;
	const GEN_FLT x74 = x72 + x73;
	const GEN_FLT x75 = (x47 * x28) + x18;
	const GEN_FLT x76 = (((-1 * x64) + x66 + (-1 * x65)) * sensor_x) + ((x68 + x70 + x69) * sensor_y) +
						((x56 + (x55 * x44) + (2 * x8 * x57)) * sensor_z);
	const GEN_FLT x77 = (x67 * x51) + (x71 * x74) + (x75 * x76);
	const GEN_FLT x78 = (x14 * ((-1 * x20) + (-1 * x23) + x33)) + (x43 * (x34 + x35 + x39)) +
						(x45 * (x46 + (x47 * x26) + (x48 * x31))) + x77;
	const GEN_FLT x79 = x49 + x50;
	const GEN_FLT x80 = x24 * x21;
	const GEN_FLT x81 = x32 * x15;
	const GEN_FLT x82 = (-1 * x80) + x81;
	const GEN_FLT x83 = x25 * x25;
	const GEN_FLT x84 = (x83 * x28) + x18;
	const GEN_FLT x85 = lh_px + (x79 * x45) + (x82 * x43) + (x84 * x14);
	const GEN_FLT x86 = lh_pz + (x75 * x45) + (x51 * x14) + (x74 * x43);
	const GEN_FLT x87 = x86 * x86;
	const GEN_FLT x88 = pow(x87, -1);
	const GEN_FLT x89 = x88 * x85;
	const GEN_FLT x90 = x89 * x78;
	const GEN_FLT x91 = pow(x86, -1);
	const GEN_FLT x92 = 2 * x32;
	const GEN_FLT x93 = x24 * x19;
	const GEN_FLT x94 = x31 * x21;
	const GEN_FLT x95 = (x30 * x15) + (x32 * x22) + (x27 * x15);
	const GEN_FLT x96 = (x84 * x67) + (x82 * x71) + (x79 * x76);
	const GEN_FLT x97 = (x14 * (x46 + (x83 * x26) + (x92 * x29))) + (x43 * ((-1 * x93) + (-1 * x94) + x95)) + x96 +
						(x45 * (x20 + x23 + x33));
	const GEN_FLT x98 = 1 + x97;
	const GEN_FLT x99 = x85 * x85;
	const GEN_FLT x100 = x99 + x87;
	const GEN_FLT x101 = pow(x100, -1);
	const GEN_FLT x102 = x87 * x101;
	const GEN_FLT x103 = x102 * (x90 + (-1 * x91 * x98));
	const GEN_FLT x104 = 2 * x85;
	const GEN_FLT x105 = 2 * x86;
	const GEN_FLT x106 = x78 * x105;
	const GEN_FLT x107 = (-1 * x72) + x73;
	const GEN_FLT x108 = x15 * x15;
	const GEN_FLT x109 = (x28 * x108) + x18;
	const GEN_FLT x110 = x80 + x81;
	const GEN_FLT x111 = lh_py + (x43 * x109) + (x45 * x107) + (x14 * x110);
	const GEN_FLT x112 = 1.0 / 2.0 * pow(x100, -3.0 / 2.0) * x111 * tilt_0;
	const GEN_FLT x113 = 2 * x37;
	const GEN_FLT x114 = (x67 * x110) + (x71 * x109) + (x76 * x107);
	const GEN_FLT x115 = (x14 * (x93 + x94 + x95)) + (x43 * (x46 + (x26 * x108) + (x22 * x113))) +
						 (x45 * ((-1 * x34) + (-1 * x35) + x39)) + x114;
	const GEN_FLT x116 = pow(x100, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x117 = x115 * x116;
	const GEN_FLT x118 = x111 * x111;
	const GEN_FLT x119 = pow((1 + (-1 * x101 * x118 * (tilt_0 * tilt_0))), -1.0 / 2.0);
	const GEN_FLT x120 = (-1 * x103) + (-1 * x119 * ((-1 * x112 * ((x98 * x104) + x106)) + x117));
	const GEN_FLT x121 = -1 * x86;
	const GEN_FLT x122 = atan2(x85, x121);
	const GEN_FLT x123 =
		sin(1.5707963267949 + gibPhase_0 + (-1 * asin(x111 * x116)) + (-1 * phase_0) + (-1 * x122)) * gibMag_0;
	const GEN_FLT x124 = x91 * x115;
	const GEN_FLT x125 = -1 * x124;
	const GEN_FLT x126 = x88 * x111;
	const GEN_FLT x127 = x78 * x126;
	const GEN_FLT x128 = x118 + x87;
	const GEN_FLT x129 = pow(x128, -1);
	const GEN_FLT x130 = x87 * x129;
	const GEN_FLT x131 = 2 * x130 * atan2(x111, x121) * curve_0;
	const GEN_FLT x132 = -1 * x91 * x97;
	const GEN_FLT x133 = x102 * (x90 + x132);
	const GEN_FLT x134 = x97 * x104;
	const GEN_FLT x135 = 1 + x115;
	const GEN_FLT x136 = (-1 * x133) + (-1 * x119 * ((-1 * (x134 + x106) * x112) + (x116 * x135)));
	const GEN_FLT x137 = x91 * x135;
	const GEN_FLT x138 = 1 + x78;
	const GEN_FLT x139 = x102 * ((x89 * x138) + x132);
	const GEN_FLT x140 = x105 * x138;
	const GEN_FLT x141 = (-1 * x139) + (-1 * x119 * ((-1 * (x134 + x140) * x112) + x117));
	const GEN_FLT x142 = x126 * x138;
	const GEN_FLT x143 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x144 = x18 * x143;
	const GEN_FLT x145 = x15 * x144;
	const GEN_FLT x146 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qj *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x147 = x21 * x146;
	const GEN_FLT x148 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qi *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x149 = x28 * x148;
	const GEN_FLT x150 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qk *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x151 = x21 * x143;
	const GEN_FLT x152 = x25 * x151;
	const GEN_FLT x153 = (x24 * x149) + (x32 * x150) + (x24 * x152);
	const GEN_FLT x154 = x25 * x144;
	const GEN_FLT x155 = x21 * x148;
	const GEN_FLT x156 = (x36 * x151) + (x37 * x150) + (x38 * x146);
	const GEN_FLT x157 = -1 * x151;
	const GEN_FLT x158 = (x14 * ((-1 * x145) + (-1 * x147) + x153)) + (x43 * (x154 + x155 + x156)) + x77 +
						 (x45 * (x157 + (x47 * x151) + (x48 * x150)));
	const GEN_FLT x159 = x24 * x144;
	const GEN_FLT x160 = x21 * x150;
	const GEN_FLT x161 = (x15 * x152) + (x15 * x149) + (x32 * x146);
	const GEN_FLT x162 = (x14 * (x157 + (x83 * x151) + (x92 * x148))) + (x43 * ((-1 * x159) + (-1 * x160) + x161)) +
						 (x45 * (x145 + x147 + x153)) + x96;
	const GEN_FLT x163 = ((x89 * x158) + (-1 * x91 * x162)) * x102;
	const GEN_FLT x164 = x105 * x158;
	const GEN_FLT x165 = (x43 * (x157 + (x108 * x151) + (x113 * x146))) + (x45 * ((-1 * x154) + (-1 * x155) + x156)) +
						 (x14 * (x159 + x160 + x161)) + x114;
	const GEN_FLT x166 = (-1 * x163) + (-1 * x119 * ((-1 * x112 * ((x104 * x162) + x164)) + (x116 * x165)));
	const GEN_FLT x167 = x91 * x165;
	const GEN_FLT x168 = x126 * x158;
	const GEN_FLT x169 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x170 = x18 * x169;
	const GEN_FLT x171 = x15 * x170;
	const GEN_FLT x172 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qj *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x173 = x21 * x172;
	const GEN_FLT x174 = x21 * x169;
	const GEN_FLT x175 = x25 * x174;
	const GEN_FLT x176 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qi *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x177 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qk *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x178 = (x24 * x175) + (x38 * x176) + (x32 * x177);
	const GEN_FLT x179 = x25 * x170;
	const GEN_FLT x180 = x21 * x176;
	const GEN_FLT x181 = (x36 * x174) + (x37 * x177) + (x38 * x172);
	const GEN_FLT x182 = -1 * x174;
	const GEN_FLT x183 = (x14 * ((-1 * x171) + (-1 * x173) + x178)) + (x43 * (x179 + x180 + x181)) +
						 (x45 * (x182 + (x47 * x174) + (x48 * x177))) + x77;
	const GEN_FLT x184 = x24 * x170;
	const GEN_FLT x185 = x21 * x177;
	const GEN_FLT x186 = (x15 * x175) + (x32 * x172) + (x37 * x176);
	const GEN_FLT x187 = (x14 * ((x83 * x174) + x182 + (x92 * x176))) + (x45 * (x171 + x173 + x178)) +
						 (x43 * ((-1 * x184) + (-1 * x185) + x186)) + x96;
	const GEN_FLT x188 = ((x89 * x183) + (-1 * x91 * x187)) * x102;
	const GEN_FLT x189 = x105 * x183;
	const GEN_FLT x190 = (x14 * (x184 + x185 + x186)) + (x43 * (x182 + (x108 * x174) + (x113 * x172))) +
						 (x45 * ((-1 * x179) + (-1 * x180) + x181)) + x114;
	const GEN_FLT x191 = (-1 * x188) + (-1 * x119 * ((-1 * x112 * ((x104 * x187) + x189)) + (x116 * x190)));
	const GEN_FLT x192 = x91 * x190;
	const GEN_FLT x193 = x126 * x183;
	const GEN_FLT x194 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x195 = x18 * x194;
	const GEN_FLT x196 = x15 * x195;
	const GEN_FLT x197 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qj *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x198 = x21 * x197;
	const GEN_FLT x199 = x21 * x194;
	const GEN_FLT x200 = x25 * x199;
	const GEN_FLT x201 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qi *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x202 = x28 * x201;
	const GEN_FLT x203 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qk *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x204 = (x24 * x200) + (x24 * x202) + (x32 * x203);
	const GEN_FLT x205 = x25 * x195;
	const GEN_FLT x206 = x21 * x201;
	const GEN_FLT x207 = (x36 * x199) + (x37 * x203) + (x38 * x197);
	const GEN_FLT x208 = -1 * x199;
	const GEN_FLT x209 = (x14 * ((-1 * x196) + (-1 * x198) + x204)) + (x43 * (x205 + x206 + x207)) +
						 (x45 * (x208 + (x47 * x199) + (x48 * x203))) + x77;
	const GEN_FLT x210 = x24 * x195;
	const GEN_FLT x211 = x21 * x203;
	const GEN_FLT x212 = (x15 * x200) + (x15 * x202) + (x32 * x197);
	const GEN_FLT x213 = (x14 * (x208 + (x83 * x199) + (2 * x25 * x202))) + (x43 * ((-1 * x210) + (-1 * x211) + x212)) +
						 x96 + (x45 * (x196 + x198 + x204));
	const GEN_FLT x214 = ((x89 * x209) + (-1 * x91 * x213)) * x102;
	const GEN_FLT x215 = x209 * x105;
	const GEN_FLT x216 = (x14 * (x210 + x211 + x212)) + (x43 * (x208 + (x108 * x199) + (x113 * x197))) + x114 +
						 (x45 * ((-1 * x205) + (-1 * x206) + x207));
	const GEN_FLT x217 = (-1 * x214) + (-1 * x119 * ((-1 * x112 * ((x213 * x104) + x215)) + (x216 * x116)));
	const GEN_FLT x218 = x91 * x216;
	const GEN_FLT x219 = x209 * x126;
	const GEN_FLT x220 = 2 * x122 * curve_1;
	const GEN_FLT x221 = -1 * x127;
	const GEN_FLT x222 = 2 * x111;
	const GEN_FLT x223 = x222 * x115;
	const GEN_FLT x224 = 1.0 / 2.0 * x85 * pow(x128, -3.0 / 2.0) * tilt_1;
	const GEN_FLT x225 = pow(x128, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x226 = pow((1 + (-1 * x99 * x129 * (tilt_1 * tilt_1))), -1.0 / 2.0);
	const GEN_FLT x227 = (-1 * (x124 + x221) * x130) + (-1 * x226 * ((-1 * (x223 + x106) * x224) + (x98 * x225)));
	const GEN_FLT x228 =
		sin(1.5707963267949 + gibPhase_1 + (-1 * asin(x85 * x225)) + (-1 * phase_1) + (-1 * atan2(-1 * x111, x121))) *
		gibMag_1;
	const GEN_FLT x229 = x97 * x225;
	const GEN_FLT x230 = (-1 * (x137 + x221) * x130) + (-1 * x226 * ((-1 * x224 * ((x222 * x135) + x106)) + x229));
	const GEN_FLT x231 = (-1 * (x124 + (-1 * x142)) * x130) + (-1 * x226 * ((-1 * (x223 + x140) * x224) + x229));
	const GEN_FLT x232 =
		(-1 * (x167 + (-1 * x168)) * x130) + (-1 * x226 * ((-1 * x224 * ((x222 * x165) + x164)) + (x225 * x162)));
	const GEN_FLT x233 =
		(-1 * (x192 + (-1 * x193)) * x130) + (-1 * x226 * ((-1 * x224 * ((x222 * x190) + x189)) + (x225 * x187)));
	const GEN_FLT x234 =
		(-1 * (x218 + (-1 * x219)) * x130) + (-1 * x226 * ((-1 * x224 * ((x216 * x222) + x215)) + (x213 * x225)));
	out[0] = (x120 * x123) + ((x125 + x127) * x131) + x120;
	out[1] = (x123 * x136) + (((-1 * x137) + x127) * x131) + x136;
	out[2] = (x123 * x141) + ((x125 + x142) * x131) + x141;
	out[3] = (x123 * x166) + x166 + (((-1 * x167) + x168) * x131);
	out[4] = (x123 * x191) + (((-1 * x192) + x193) * x131) + x191;
	out[5] = (x217 * x123) + (((-1 * x218) + x219) * x131) + x217;
	out[6] = (x220 * x103) + (x227 * x228) + x227;
	out[7] = (x220 * x133) + (x230 * x228) + x230;
	out[8] = (x220 * x139) + (x231 * x228) + x231;
	out[9] = (x220 * x163) + (x232 * x228) + x232;
	out[10] = (x220 * x188) + (x233 * x228) + x233;
	out[11] = (x214 * x220) + (x234 * x228) + x234;
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 + (-1 * x1);
	const GEN_FLT x3 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x4 = x3 * x3;
	const GEN_FLT x5 = (x2 * x4) + x1;
	const GEN_FLT x6 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 0;
	const GEN_FLT x7 = x6 * x6;
	const GEN_FLT x8 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   : 1e-10;
	const GEN_FLT x9 = cos(x8);
	const GEN_FLT x10 = 1 + (-1 * x9);
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x12 = sin(x8);
	const GEN_FLT x13 = x12 * x11;
	const GEN_FLT x14 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x15 = x14 * x10;
	const GEN_FLT x16 = x6 * x15;
	const GEN_FLT x17 = x14 * x12;
	const GEN_FLT x18 = x11 * x10;
	const GEN_FLT x19 = x6 * x18;
	const GEN_FLT x20 =
		obj_pz + (((x7 * x10) + x9) * sensor_z) + ((x13 + x16) * sensor_y) + (((-1 * x17) + x19) * sensor_x);
	const GEN_FLT x21 = sin(x0);
	const GEN_FLT x22 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 1;
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 0;
	const GEN_FLT x25 = x2 * x3;
	const GEN_FLT x26 = x24 * x25;
	const GEN_FLT x27 = x23 + x26;
	const GEN_FLT x28 = x14 * x14;
	const GEN_FLT x29 = x6 * x12;
	const GEN_FLT x30 = x15 * x11;
	const GEN_FLT x31 =
		obj_py + (((x28 * x10) + x9) * sensor_y) + (((-1 * x13) + x16) * sensor_z) + ((x29 + x30) * sensor_x);
	const GEN_FLT x32 = x24 * x21;
	const GEN_FLT x33 = x25 * x22;
	const GEN_FLT x34 = (-1 * x32) + x33;
	const GEN_FLT x35 = x11 * x11;
	const GEN_FLT x36 =
		obj_px + ((x17 + x19) * sensor_z) + (((-1 * x29) + x30) * sensor_y) + (((x35 * x10) + x9) * sensor_x);
	const GEN_FLT x37 = lh_pz + (x5 * x20) + (x31 * x27) + (x34 * x36);
	const GEN_FLT x38 = pow(x37, -1);
	const GEN_FLT x39 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x40 = x1 * x39;
	const GEN_FLT x41 = x3 * x40;
	const GEN_FLT x42 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x43 = x42 * x21;
	const GEN_FLT x44 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x45 = x2 * x24;
	const GEN_FLT x46 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x47 = x2 * x22;
	const GEN_FLT x48 = x32 * x39;
	const GEN_FLT x49 = (x44 * x45) + (x46 * x47) + (x48 * x22);
	const GEN_FLT x50 = x3 * x21;
	const GEN_FLT x51 = x45 * x22;
	const GEN_FLT x52 = x50 + x51;
	const GEN_FLT x53 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x54 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x55 = x54 * x12;
	const GEN_FLT x56 = -1 * x55;
	const GEN_FLT x57 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x58 = x57 * x12;
	const GEN_FLT x59 = x9 * x54;
	const GEN_FLT x60 = x6 * x59;
	const GEN_FLT x61 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x62 = x54 * x13;
	const GEN_FLT x63 = (x61 * x18) + (x62 * x14) + (x53 * x15);
	const GEN_FLT x64 = x61 * x12;
	const GEN_FLT x65 = x59 * x14;
	const GEN_FLT x66 = x6 * x10;
	const GEN_FLT x67 = (x66 * x53) + (x6 * x62) + (x57 * x18);
	const GEN_FLT x68 = (((2 * x53 * x18) + x56 + (x55 * x35)) * sensor_x) +
						(((-1 * x58) + (-1 * x60) + x63) * sensor_y) + ((x64 + x65 + x67) * sensor_z);
	const GEN_FLT x69 = x39 * x21;
	const GEN_FLT x70 = -1 * x69;
	const GEN_FLT x71 = x24 * x24;
	const GEN_FLT x72 = x53 * x12;
	const GEN_FLT x73 = x59 * x11;
	const GEN_FLT x74 = (x61 * x66) + (x57 * x15) + (x6 * x55 * x14);
	const GEN_FLT x75 = ((x58 + x60 + x63) * sensor_x) + ((x56 + (2 * x61 * x15) + (x55 * x28)) * sensor_y) +
						(((-1 * x72) + (-1 * x73) + x74) * sensor_z);
	const GEN_FLT x76 = (x2 * x71) + x1;
	const GEN_FLT x77 = x40 * x22;
	const GEN_FLT x78 = x44 * x21;
	const GEN_FLT x79 = (x3 * x48) + (x42 * x45) + (x46 * x25);
	const GEN_FLT x80 = (-1 * x23) + x26;
	const GEN_FLT x81 = (((-1 * x64) + (-1 * x65) + x67) * sensor_x) + ((x72 + x73 + x74) * sensor_y) +
						((x56 + (2 * x66 * x57) + (x7 * x55)) * sensor_z);
	const GEN_FLT x82 = (x36 * (x41 + x43 + x49)) + (x31 * (x70 + (x71 * x69) + (2 * x45 * x46))) + (x68 * x52) +
						(x20 * ((-1 * x77) + (-1 * x78) + x79)) + (x75 * x76) + (x80 * x81);
	const GEN_FLT x83 = x82 * x38;
	const GEN_FLT x84 = lh_py + (x80 * x20) + (x76 * x31) + (x52 * x36);
	const GEN_FLT x85 = x37 * x37;
	const GEN_FLT x86 = x40 * x24;
	const GEN_FLT x87 = x46 * x21;
	const GEN_FLT x88 = (x3 * x69 * x22) + (x44 * x25) + (x42 * x47);
	const GEN_FLT x89 = (x36 * ((-1 * x86) + (-1 * x87) + x88)) + (x31 * (x77 + x78 + x79)) + (x68 * x34) + (x5 * x81) +
						(x75 * x27) + (x20 * (x70 + (2 * x42 * x25) + (x4 * x69)));
	const GEN_FLT x90 = x89 * pow(x85, -1);
	const GEN_FLT x91 = x84 * x90;
	const GEN_FLT x92 = -1 * x37;
	const GEN_FLT x93 = atan2(x84, x92);
	const GEN_FLT x94 = x84 * x84;
	const GEN_FLT x95 = x94 + x85;
	const GEN_FLT x96 = pow(x95, -1);
	const GEN_FLT x97 = x85 * x96;
	const GEN_FLT x98 = 2 * ((-1 * x83) + x91) * x93 * x97 * curve_0;
	const GEN_FLT x99 = x32 + x33;
	const GEN_FLT x100 = (-1 * x50) + x51;
	const GEN_FLT x101 = x22 * x22;
	const GEN_FLT x102 = (x2 * x101) + x1;
	const GEN_FLT x103 = lh_px + (x99 * x20) + (x31 * x100) + (x36 * x102);
	const GEN_FLT x104 = (x36 * (x70 + (x69 * x101) + (2 * x44 * x47))) + (x68 * x102) +
						 (x31 * ((-1 * x41) + (-1 * x43) + x49)) + (x75 * x100) + (x20 * (x86 + x87 + x88)) +
						 (x81 * x99);
	const GEN_FLT x105 = x103 * x103;
	const GEN_FLT x106 = x105 + x85;
	const GEN_FLT x107 = pow(x106, -1);
	const GEN_FLT x108 = ((x90 * x103) + (-1 * x38 * x104)) * x85 * x107;
	const GEN_FLT x109 = -1 * x108;
	const GEN_FLT x110 = pow((1 + (-1 * x94 * x107 * (tilt_0 * tilt_0))), -1.0 / 2.0);
	const GEN_FLT x111 = 2 * x89 * x37;
	const GEN_FLT x112 = pow(x106, -1.0 / 2.0);
	const GEN_FLT x113 =
		(-1.0 / 2.0 * x84 * pow(x106, -3.0 / 2.0) * ((2 * x103 * x104) + x111) * tilt_0) + (x82 * x112 * tilt_0);
	const GEN_FLT x114 = x109 + (-1 * x110 * x113);
	const GEN_FLT x115 = -1 + x114;
	const GEN_FLT x116 = x84 * x112;
	const GEN_FLT x117 = atan2(x103, x92);
	const GEN_FLT x118 = 1.5707963267949 + gibPhase_0 + (-1 * asin(x116 * tilt_0)) + (-1 * phase_0) + (-1 * x117);
	const GEN_FLT x119 = sin(x118) * gibMag_0;
	const GEN_FLT x120 = x109 + (-1 * (x116 + x113) * x110);
	const GEN_FLT x121 = x98 + x114;
	const GEN_FLT x122 = (x119 * x114) + x121;
	const GEN_FLT x123 = -1 * (x83 + (-1 * x91)) * x97;
	const GEN_FLT x124 = pow(x95, -1.0 / 2.0);
	const GEN_FLT x125 =
		(-1.0 / 2.0 * pow(x95, -3.0 / 2.0) * x103 * tilt_1 * ((2 * x82 * x84) + x111)) + (x104 * x124 * tilt_1);
	const GEN_FLT x126 = pow((1 + (-1 * x96 * x105 * (tilt_1 * tilt_1))), -1.0 / 2.0);
	const GEN_FLT x127 = x123 + (-1 * x126 * x125);
	const GEN_FLT x128 = x103 * x124;
	const GEN_FLT x129 =
		1.5707963267949 + gibPhase_1 + (-1 * asin(x128 * tilt_1)) + (-1 * phase_1) + (-1 * atan2(-1 * x84, x92));
	const GEN_FLT x130 = sin(x129) * gibMag_1;
	const GEN_FLT x131 = 2 * x108 * x117 * curve_1;
	const GEN_FLT x132 = x131 + x127;
	const GEN_FLT x133 = (x127 * x130) + x132;
	const GEN_FLT x134 = -1 + x127;
	const GEN_FLT x135 = x123 + (-1 * (x128 + x125) * x126);
	out[0] = x98 + (x119 * x115) + x115;
	out[1] = x98 + (x119 * x120) + x120;
	out[2] = (x93 * x93) + x122;
	out[3] = (x119 * (1 + x114)) + x121;
	out[4] = (-1 * cos(x118)) + x122;
	out[5] = x122;
	out[6] = x122;
	out[7] = x122;
	out[8] = x122;
	out[9] = x122;
	out[10] = x122;
	out[11] = x122;
	out[12] = x122;
	out[13] = x122;
	out[14] = x133;
	out[15] = x133;
	out[16] = x133;
	out[17] = x133;
	out[18] = x133;
	out[19] = x133;
	out[20] = x133;
	out[21] = x131 + (x130 * x134) + x134;
	out[22] = x131 + (x130 * x135) + x135;
	out[23] = (x117 * x117) + x133;
	out[24] = (x130 * (1 + x127)) + x132;
	out[25] = (-1 * cos(x129)) + x133;
	out[26] = x133;
	out[27] = x133;
}

/** Applying function <function reproject_axis_x at 0x7f35a4af8290> */
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x11 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x12 = cos(x11);
	const GEN_FLT x13 = 1 + (-1 * x12);
	const GEN_FLT x14 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x15 = sin(x11);
	const GEN_FLT x16 = x15 * x14;
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x18 = x13 * x17;
	const GEN_FLT x19 = x10 * x18;
	const GEN_FLT x20 = x15 * x17;
	const GEN_FLT x21 = x14 * x13 * x10;
	const GEN_FLT x22 =
		obj_pz + (((x13 * (x10 * x10)) + x12) * sensor_z) + ((x16 + x19) * sensor_y) + (((-1 * x20) + x21) * sensor_x);
	const GEN_FLT x23 = x15 * x10;
	const GEN_FLT x24 = x14 * x18;
	const GEN_FLT x25 =
		obj_py + (((-1 * x16) + x19) * sensor_z) + (((x13 * (x17 * x17)) + x12) * sensor_y) + ((x23 + x24) * sensor_x);
	const GEN_FLT x26 = x1 * x4;
	const GEN_FLT x27 = x2 * x8;
	const GEN_FLT x28 =
		obj_px + ((x20 + x21) * sensor_z) + (((-1 * x23) + x24) * sensor_y) + ((((x14 * x14) * x13) + x12) * sensor_x);
	const GEN_FLT x29 = lh_py + (x25 * ((x6 * (x7 * x7)) + x5)) + (((-1 * x3) + x9) * x22) + ((x26 + x27) * x28);
	const GEN_FLT x30 = x1 * x7;
	const GEN_FLT x31 = x2 * x4 * x6;
	const GEN_FLT x32 = lh_pz + (x22 * (((x4 * x4) * x6) + x5)) + ((x3 + x9) * x25) + (((-1 * x30) + x31) * x28);
	const GEN_FLT x33 = -1 * x32;
	const GEN_FLT x34 = lh_px + ((x30 + x31) * x22) + (((-1 * x26) + x27) * x25) + (x28 * (((x2 * x2) * x6) + x5));
	const GEN_FLT x35 = (-1 * asin(pow(((x32 * x32) + (x34 * x34)), -1.0 / 2.0) * x29 * tilt_0)) + (-1 * phase_0) +
						(-1 * atan2(x34, x33));
	out[0] = ((atan2(x29, x33) * atan2(x29, x33)) * curve_0) +
			 (-1 * cos(1.5707963267949 + gibPhase_0 + x35) * gibMag_0) + x35;
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
	const GEN_FLT x0 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (-1 * obj_qk *
							  pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									   ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									   : 1e-10),
								  -2) *
							  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
						   : 0;
	const GEN_FLT x1 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   : 1e-10;
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 0;
	const GEN_FLT x5 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x6 = cos(x1);
	const GEN_FLT x7 = x6 * x5;
	const GEN_FLT x8 = x4 * x7;
	const GEN_FLT x9 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (-1 * obj_qj *
							  pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									   ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									   : 1e-10),
								  -2) *
							  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
						   : 0;
	const GEN_FLT x10 = 1 + (-1 * x6);
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x12 = x11 * x10;
	const GEN_FLT x13 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x14 = x2 * x11;
	const GEN_FLT x15 = x14 * x13;
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x17 = x13 * x10;
	const GEN_FLT x18 = (x9 * x12) + (x5 * x15) + (x17 * x16);
	const GEN_FLT x19 = x2 * x5;
	const GEN_FLT x20 = -1 * x19;
	const GEN_FLT x21 = 2 * x17;
	const GEN_FLT x22 = x13 * x13;
	const GEN_FLT x23 = x2 * x16;
	const GEN_FLT x24 = x7 * x11;
	const GEN_FLT x25 = x4 * x10;
	const GEN_FLT x26 = x4 * x13;
	const GEN_FLT x27 = (x9 * x25) + (x0 * x17) + (x26 * x19);
	const GEN_FLT x28 = ((x3 + x8 + x18) * sensor_x) + ((x20 + (x9 * x21) + (x22 * x19)) * sensor_y) +
						(((-1 * x23) + (-1 * x24) + x27) * sensor_z);
	const GEN_FLT x29 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							: 1e-10;
	const GEN_FLT x30 = sin(x29);
	const GEN_FLT x31 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 1;
	const GEN_FLT x32 = x30 * x31;
	const GEN_FLT x33 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 0;
	const GEN_FLT x34 = cos(x29);
	const GEN_FLT x35 = 1 + (-1 * x34);
	const GEN_FLT x36 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 0;
	const GEN_FLT x37 = x36 * x35;
	const GEN_FLT x38 = x33 * x37;
	const GEN_FLT x39 = x32 + x38;
	const GEN_FLT x40 = x39 * x28;
	const GEN_FLT x41 = x33 * x33;
	const GEN_FLT x42 = (x41 * x35) + x34;
	const GEN_FLT x43 = x2 * x9;
	const GEN_FLT x44 = x7 * x13;
	const GEN_FLT x45 = x4 * x14;
	const GEN_FLT x46 = (x25 * x16) + (x0 * x12) + (x5 * x45);
	const GEN_FLT x47 = 2 * x25;
	const GEN_FLT x48 = x4 * x4;
	const GEN_FLT x49 = (((-1 * x43) + (-1 * x44) + x46) * sensor_x) + ((x20 + (x0 * x47) + (x48 * x19)) * sensor_z) +
						((x23 + x24 + x27) * sensor_y);
	const GEN_FLT x50 = x42 * x49;
	const GEN_FLT x51 = x30 * x36;
	const GEN_FLT x52 = x33 * x35;
	const GEN_FLT x53 = x52 * x31;
	const GEN_FLT x54 = (-1 * x51) + x53;
	const GEN_FLT x55 = 2 * x12;
	const GEN_FLT x56 = x11 * x11;
	const GEN_FLT x57 = (((x55 * x16) + x20 + (x56 * x19)) * sensor_x) + (((-1 * x3) + (-1 * x8) + x18) * sensor_y) +
						((x43 + x44 + x46) * sensor_z);
	const GEN_FLT x58 = 1 + x57;
	const GEN_FLT x59 = x2 * x13;
	const GEN_FLT x60 = x25 * x11;
	const GEN_FLT x61 = x2 * x4;
	const GEN_FLT x62 = x11 * x17;
	const GEN_FLT x63 =
		obj_px + (((x56 * x10) + x6) * sensor_x) + ((x59 + x60) * sensor_z) + (((-1 * x61) + x62) * sensor_y);
	const GEN_FLT x64 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x65 = x64 * x34;
	const GEN_FLT x66 = x65 * x36;
	const GEN_FLT x67 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x68 = x67 * x30;
	const GEN_FLT x69 = x64 * x30;
	const GEN_FLT x70 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x71 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x72 = x31 * x35;
	const GEN_FLT x73 = (x69 * x31 * x33) + (x70 * x52) + (x71 * x72);
	const GEN_FLT x74 = x65 * x31;
	const GEN_FLT x75 = x70 * x30;
	const GEN_FLT x76 = x69 * x36;
	const GEN_FLT x77 = (x76 * x33) + (x71 * x37) + (x67 * x52);
	const GEN_FLT x78 = x25 * x13;
	const GEN_FLT x79 =
		obj_py + (((x22 * x10) + x6) * sensor_y) + (((-1 * x14) + x78) * sensor_z) + ((x61 + x62) * sensor_x);
	const GEN_FLT x80 =
		obj_pz + (((x48 * x10) + x6) * sensor_z) + ((x14 + x78) * sensor_y) + (((-1 * x59) + x60) * sensor_x);
	const GEN_FLT x81 = -1 * x69;
	const GEN_FLT x82 = (x63 * ((-1 * x66) + (-1 * x68) + x73)) + (x79 * (x74 + x75 + x77)) +
						(x80 * (x81 + (x69 * x41) + (2 * x71 * x52)));
	const GEN_FLT x83 = x40 + x50 + (x54 * x58) + x82;
	const GEN_FLT x84 = x51 + x53;
	const GEN_FLT x85 = x30 * x33;
	const GEN_FLT x86 = x31 * x37;
	const GEN_FLT x87 = (-1 * x85) + x86;
	const GEN_FLT x88 = x31 * x31;
	const GEN_FLT x89 = (x88 * x35) + x34;
	const GEN_FLT x90 = lh_px + (x80 * x84) + (x87 * x79) + (x89 * x63);
	const GEN_FLT x91 = lh_pz + (x80 * x42) + (x79 * x39) + (x63 * x54);
	const GEN_FLT x92 = x91 * x91;
	const GEN_FLT x93 = pow(x92, -1);
	const GEN_FLT x94 = x93 * x90;
	const GEN_FLT x95 = pow(x91, -1);
	const GEN_FLT x96 = x84 * x49;
	const GEN_FLT x97 = x65 * x33;
	const GEN_FLT x98 = x71 * x30;
	const GEN_FLT x99 = (x70 * x37) + (x72 * x67) + (x76 * x31);
	const GEN_FLT x100 = (x63 * (x81 + (x88 * x69) + (2 * x70 * x72))) + (x79 * ((-1 * x97) + (-1 * x98) + x99)) +
						 (x80 * (x66 + x68 + x73));
	const GEN_FLT x101 = (x87 * x28) + x100;
	const GEN_FLT x102 = (x89 * x58) + x96 + x101;
	const GEN_FLT x103 = (x90 * x90) + x92;
	const GEN_FLT x104 = pow(x103, -1);
	const GEN_FLT x105 = x92 * x104;
	const GEN_FLT x106 = (-1 * x32) + x38;
	const GEN_FLT x107 = x36 * x36;
	const GEN_FLT x108 = (x35 * x107) + x34;
	const GEN_FLT x109 = x85 + x86;
	const GEN_FLT x110 = lh_py + (x80 * x106) + (x79 * x108) + (x63 * x109);
	const GEN_FLT x111 = x110 * x110;
	const GEN_FLT x112 = pow((1 + (-1 * x104 * x111 * (tilt_0 * tilt_0))), -1.0 / 2.0);
	const GEN_FLT x113 = 2 * x90;
	const GEN_FLT x114 = 2 * x91;
	const GEN_FLT x115 = 1.0 / 2.0 * pow(x103, -3.0 / 2.0) * x110 * tilt_0;
	const GEN_FLT x116 = x28 * x108;
	const GEN_FLT x117 = (x79 * (x81 + (x69 * x107) + (2 * x67 * x37))) + (x63 * (x97 + x98 + x99)) +
						 (x80 * ((-1 * x74) + (-1 * x75) + x77));
	const GEN_FLT x118 = (x49 * x106) + x117;
	const GEN_FLT x119 = (x58 * x109) + x116 + x118;
	const GEN_FLT x120 = pow(x103, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x121 = (-1 * x105 * ((x83 * x94) + (-1 * x95 * x102))) +
						 (-1 * x112 * ((-1 * x115 * ((x102 * x113) + (x83 * x114))) + (x119 * x120)));
	const GEN_FLT x122 = -1 * x91;
	const GEN_FLT x123 =
		sin(1.5707963267949 + gibPhase_0 + (-1 * asin(x110 * x120)) + (-1 * phase_0) + (-1 * atan2(x90, x122))) *
		gibMag_0;
	const GEN_FLT x124 = x93 * x110;
	const GEN_FLT x125 = 2 * x92 * pow((x111 + x92), -1) * atan2(x110, x122) * curve_0;
	const GEN_FLT x126 = 1 + x28;
	const GEN_FLT x127 = (x54 * x57) + x82;
	const GEN_FLT x128 = (x39 * x126) + x50 + x127;
	const GEN_FLT x129 = x89 * x57;
	const GEN_FLT x130 = x96 + x129 + (x87 * x126) + x100;
	const GEN_FLT x131 = x57 * x109;
	const GEN_FLT x132 = x131 + (x108 * x126) + x118;
	const GEN_FLT x133 = (-1 * ((x94 * x128) + (-1 * x95 * x130)) * x105) +
						 (-1 * x112 * ((-1 * ((x113 * x130) + (x114 * x128)) * x115) + (x120 * x132)));
	const GEN_FLT x134 = 1 + x49;
	const GEN_FLT x135 = x40 + (x42 * x134) + x127;
	const GEN_FLT x136 = x129 + (x84 * x134) + x101;
	const GEN_FLT x137 = x131 + x116 + (x106 * x134) + x117;
	const GEN_FLT x138 = (-1 * ((x94 * x135) + (-1 * x95 * x136)) * x105) +
						 (-1 * x112 * ((-1 * ((x113 * x136) + (x114 * x135)) * x115) + (x120 * x137)));
	const GEN_FLT x139 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x140 = x2 * x139;
	const GEN_FLT x141 = -1 * x140;
	const GEN_FLT x142 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qi *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x143 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qk *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x144 = x2 * x143;
	const GEN_FLT x145 = x4 * x6;
	const GEN_FLT x146 = x139 * x145;
	const GEN_FLT x147 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qj *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x148 = x13 * x11;
	const GEN_FLT x149 = (x17 * x142) + (x12 * x147) + (x140 * x148);
	const GEN_FLT x150 = x2 * x147;
	const GEN_FLT x151 = x6 * x13;
	const GEN_FLT x152 = x139 * x151;
	const GEN_FLT x153 = x4 * x11;
	const GEN_FLT x154 = (x25 * x142) + (x12 * x143) + (x140 * x153);
	const GEN_FLT x155 = ((x141 + (x55 * x142) + (x56 * x140)) * sensor_x) +
						 (((-1 * x144) + (-1 * x146) + x149) * sensor_y) + ((x150 + x152 + x154) * sensor_z);
	const GEN_FLT x156 = x2 * x142;
	const GEN_FLT x157 = x6 * x11;
	const GEN_FLT x158 = x139 * x157;
	const GEN_FLT x159 = (x25 * x147) + (x17 * x143) + (x26 * x140);
	const GEN_FLT x160 = ((x144 + x146 + x149) * sensor_x) + (((x21 * x147) + x141 + (x22 * x140)) * sensor_y) +
						 (((-1 * x156) + (-1 * x158) + x159) * sensor_z);
	const GEN_FLT x161 = (((-1 * x150) + (-1 * x152) + x154) * sensor_x) + ((x156 + x158 + x159) * sensor_y) +
						 ((x141 + (x47 * x143) + (x48 * x140)) * sensor_z);
	const GEN_FLT x162 = (x54 * x155) + (x42 * x161) + (x39 * x160) + x82;
	const GEN_FLT x163 = (x89 * x155) + (x87 * x160) + (x84 * x161) + x100;
	const GEN_FLT x164 = (x109 * x155) + (x108 * x160) + (x106 * x161) + x117;
	const GEN_FLT x165 = (-1 * ((x94 * x162) + (-1 * x95 * x163)) * x105) +
						 (-1 * x112 * ((-1 * ((x113 * x163) + (x114 * x162)) * x115) + (x120 * x164)));
	const GEN_FLT x166 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x167 = x2 * x166;
	const GEN_FLT x168 = -1 * x167;
	const GEN_FLT x169 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qi *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x170 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qk *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x171 = x2 * x170;
	const GEN_FLT x172 = x166 * x145;
	const GEN_FLT x173 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qj *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x174 = (x17 * x169) + (x12 * x173) + (x167 * x148);
	const GEN_FLT x175 = x2 * x173;
	const GEN_FLT x176 = x166 * x151;
	const GEN_FLT x177 = (x25 * x169) + (x167 * x153) + (x12 * x170);
	const GEN_FLT x178 = ((x168 + (x55 * x169) + (x56 * x167)) * sensor_x) +
						 (((-1 * x171) + (-1 * x172) + x174) * sensor_y) + ((x175 + x176 + x177) * sensor_z);
	const GEN_FLT x179 = x2 * x169;
	const GEN_FLT x180 = x166 * x157;
	const GEN_FLT x181 = (x17 * x170) + (x26 * x167) + (x25 * x173);
	const GEN_FLT x182 = ((x168 + (x21 * x173) + (x22 * x167)) * sensor_y) + ((x171 + x172 + x174) * sensor_x) +
						 (((-1 * x179) + (-1 * x180) + x181) * sensor_z);
	const GEN_FLT x183 = (((-1 * x175) + (-1 * x176) + x177) * sensor_x) + ((x179 + x180 + x181) * sensor_y) +
						 ((x168 + (x47 * x170) + (x48 * x167)) * sensor_z);
	const GEN_FLT x184 = (x54 * x178) + (x39 * x182) + x82 + (x42 * x183);
	const GEN_FLT x185 = (x89 * x178) + (x87 * x182) + x100 + (x84 * x183);
	const GEN_FLT x186 = (x109 * x178) + (x108 * x182) + (x106 * x183) + x117;
	const GEN_FLT x187 = (-1 * ((x94 * x184) + (-1 * x95 * x185)) * x105) +
						 (-1 * x112 * ((-1 * ((x113 * x185) + (x114 * x184)) * x115) + (x120 * x186)));
	const GEN_FLT x188 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x189 = x2 * x188;
	const GEN_FLT x190 = -1 * x189;
	const GEN_FLT x191 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qi *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x192 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qk *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x193 = x2 * x192;
	const GEN_FLT x194 = x188 * x145;
	const GEN_FLT x195 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qj *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x196 = (x17 * x191) + (x12 * x195) + (x15 * x188);
	const GEN_FLT x197 = x2 * x195;
	const GEN_FLT x198 = x188 * x151;
	const GEN_FLT x199 = (x25 * x191) + (x45 * x188) + (x12 * x192);
	const GEN_FLT x200 = ((x190 + (x55 * x191) + (x56 * x189)) * sensor_x) +
						 (((-1 * x193) + (-1 * x194) + x196) * sensor_y) + ((x197 + x198 + x199) * sensor_z);
	const GEN_FLT x201 = x2 * x191;
	const GEN_FLT x202 = x188 * x157;
	const GEN_FLT x203 = (x25 * x195) + (x17 * x192) + (x26 * x189);
	const GEN_FLT x204 = ((x193 + x194 + x196) * sensor_x) + (((x21 * x195) + x190 + (x22 * x189)) * sensor_y) +
						 (((-1 * x201) + (-1 * x202) + x203) * sensor_z);
	const GEN_FLT x205 = (((-1 * x197) + (-1 * x198) + x199) * sensor_x) + ((x201 + x202 + x203) * sensor_y) +
						 ((x190 + (x47 * x192) + (x48 * x189)) * sensor_z);
	const GEN_FLT x206 = (x54 * x200) + (x42 * x205) + (x39 * x204) + x82;
	const GEN_FLT x207 = (x89 * x200) + (x87 * x204) + (x84 * x205) + x100;
	const GEN_FLT x208 = (x200 * x109) + (x204 * x108) + (x205 * x106) + x117;
	const GEN_FLT x209 = (-1 * ((x94 * x206) + (-1 * x95 * x207)) * x105) +
						 (-1 * x112 * ((-1 * ((x207 * x113) + (x206 * x114)) * x115) + (x208 * x120)));
	out[0] = (x123 * x121) + (((-1 * x95 * x119) + (x83 * x124)) * x125) + x121;
	out[1] = (x123 * x133) + (x125 * ((-1 * x95 * x132) + (x124 * x128))) + x133;
	out[2] = (x123 * x138) + (x125 * ((-1 * x95 * x137) + (x124 * x135))) + x138;
	out[3] = (x123 * x165) + (x125 * ((-1 * x95 * x164) + (x124 * x162))) + x165;
	out[4] = (x123 * x187) + (x125 * ((-1 * x95 * x186) + (x124 * x184))) + x187;
	out[5] = (x209 * x123) + x209 + (x125 * ((-1 * x95 * x208) + (x206 * x124)));
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (x15 * x12) + x14;
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x18 = sin(x13);
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x21 * x11;
	const GEN_FLT x23 = x19 + x22;
	const GEN_FLT x24 = x20 * x18;
	const GEN_FLT x25 = x15 * x17;
	const GEN_FLT x26 = x25 * x11;
	const GEN_FLT x27 = (-1 * x24) + x26;
	const GEN_FLT x28 = obj_pz + (x27 * sensor_x) + (x16 * sensor_z) + (x23 * sensor_y);
	const GEN_FLT x29 = x1 * x7;
	const GEN_FLT x30 = x2 * x6;
	const GEN_FLT x31 = x4 * x30;
	const GEN_FLT x32 = (-1 * x29) + x31;
	const GEN_FLT x33 = (-1 * x19) + x22;
	const GEN_FLT x34 = x20 * x20;
	const GEN_FLT x35 = (x34 * x15) + x14;
	const GEN_FLT x36 = x11 * x18;
	const GEN_FLT x37 = x21 * x17;
	const GEN_FLT x38 = x36 + x37;
	const GEN_FLT x39 = obj_py + (x33 * sensor_z) + (x35 * sensor_y) + (x38 * sensor_x);
	const GEN_FLT x40 = x4 * x4;
	const GEN_FLT x41 = (x6 * x40) + x5;
	const GEN_FLT x42 = x24 + x26;
	const GEN_FLT x43 = (-1 * x36) + x37;
	const GEN_FLT x44 = x17 * x17;
	const GEN_FLT x45 = (x44 * x15) + x14;
	const GEN_FLT x46 = obj_px + (x42 * sensor_z) + (x43 * sensor_y) + (x45 * sensor_x);
	const GEN_FLT x47 = lh_px + (x28 * x10) + (x32 * x39) + (x41 * x46);
	const GEN_FLT x48 = x7 * x7;
	const GEN_FLT x49 = (x6 * x48) + x5;
	const GEN_FLT x50 = x1 * x4;
	const GEN_FLT x51 = x2 * x8;
	const GEN_FLT x52 = x50 + x51;
	const GEN_FLT x53 = (-1 * x3) + x9;
	const GEN_FLT x54 = lh_pz + (x49 * x28) + (x52 * x39) + (x53 * x46);
	const GEN_FLT x55 = x54 * x54;
	const GEN_FLT x56 = pow(x55, -1);
	const GEN_FLT x57 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x58 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x59 = x58 * x18;
	const GEN_FLT x60 = -1 * x59;
	const GEN_FLT x61 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x62 = x61 * x18;
	const GEN_FLT x63 = x58 * x14;
	const GEN_FLT x64 = x63 * x11;
	const GEN_FLT x65 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x66 = x58 * x19;
	const GEN_FLT x67 = (x65 * x25) + (x66 * x20) + (x57 * x21);
	const GEN_FLT x68 = x65 * x18;
	const GEN_FLT x69 = x63 * x20;
	const GEN_FLT x70 = x15 * x11;
	const GEN_FLT x71 = (x70 * x57) + (x61 * x25) + (x66 * x11);
	const GEN_FLT x72 = (((2 * x57 * x25) + x60 + (x59 * x44)) * sensor_x) +
						(((-1 * x62) + (-1 * x64) + x67) * sensor_y) + ((x68 + x69 + x71) * sensor_z);
	const GEN_FLT x73 = x45 + x72;
	const GEN_FLT x74 = x57 * x18;
	const GEN_FLT x75 = x63 * x17;
	const GEN_FLT x76 = (x70 * x65) + (x61 * x21) + (x59 * x20 * x11);
	const GEN_FLT x77 = ((x62 + x64 + x67) * sensor_x) + ((x60 + (2 * x65 * x21) + (x59 * x34)) * sensor_y) +
						(((-1 * x74) + (-1 * x75) + x76) * sensor_z);
	const GEN_FLT x78 = x38 + x77;
	const GEN_FLT x79 = (((-1 * x68) + (-1 * x69) + x71) * sensor_x) + ((x74 + x75 + x76) * sensor_y) +
						((x60 + (2 * x70 * x61) + (x59 * x12)) * sensor_z);
	const GEN_FLT x80 = x27 + x79;
	const GEN_FLT x81 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x82 = x5 * x81;
	const GEN_FLT x83 = x2 * x82;
	const GEN_FLT x84 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x85 = x1 * x84;
	const GEN_FLT x86 = x1 * x81;
	const GEN_FLT x87 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x88 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x89 = x4 * x6;
	const GEN_FLT x90 = (x4 * x7 * x86) + (x8 * x87) + (x88 * x89);
	const GEN_FLT x91 = x4 * x82;
	const GEN_FLT x92 = x1 * x87;
	const GEN_FLT x93 = x2 * x86;
	const GEN_FLT x94 = (x7 * x93) + (x88 * x30) + (x8 * x84);
	const GEN_FLT x95 = -1 * x86;
	const GEN_FLT x96 = (x46 * ((-1 * x83) + (-1 * x85) + x90)) + (x28 * (x95 + (x86 * x48) + (2 * x8 * x88))) +
						(x39 * (x91 + x92 + x94));
	const GEN_FLT x97 = (x73 * x53) + (x78 * x52) + (x80 * x49) + x96;
	const GEN_FLT x98 = x56 * x97;
	const GEN_FLT x99 = pow(x54, -1);
	const GEN_FLT x100 = x7 * x82;
	const GEN_FLT x101 = x1 * x88;
	const GEN_FLT x102 = (x87 * x30) + (x89 * x84) + (x4 * x93);
	const GEN_FLT x103 = (x46 * (x95 + (x86 * x40) + (2 * x89 * x87))) + (x39 * ((-1 * x100) + (-1 * x101) + x102)) +
						 (x28 * (x83 + x85 + x90));
	const GEN_FLT x104 = (x73 * x41) + (x78 * x32) + (x80 * x10) + x103;
	const GEN_FLT x105 = (x47 * x47) + x55;
	const GEN_FLT x106 = pow(x105, -1);
	const GEN_FLT x107 = x55 * x106;
	const GEN_FLT x108 = (-1 * x50) + x51;
	const GEN_FLT x109 = x2 * x2;
	const GEN_FLT x110 = (x6 * x109) + x5;
	const GEN_FLT x111 = x29 + x31;
	const GEN_FLT x112 = lh_py + (x28 * x108) + (x39 * x110) + (x46 * x111);
	const GEN_FLT x113 = x112 * x112;
	const GEN_FLT x114 = pow((1 + (-1 * x106 * x113 * (tilt_0 * tilt_0))), -1.0 / 2.0);
	const GEN_FLT x115 = 2 * x47;
	const GEN_FLT x116 = 2 * x54;
	const GEN_FLT x117 = 1.0 / 2.0 * pow(x105, -3.0 / 2.0) * x112 * tilt_0;
	const GEN_FLT x118 = (x46 * (x100 + x101 + x102)) + (x28 * ((-1 * x91) + (-1 * x92) + x94)) +
						 (x39 * (x95 + (x86 * x109) + (2 * x84 * x30)));
	const GEN_FLT x119 = (x73 * x111) + (x78 * x110) + (x80 * x108) + x118;
	const GEN_FLT x120 = pow(x105, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x121 = (-1 * x107 * ((x98 * x47) + (-1 * x99 * x104))) +
						 (-1 * x114 * ((-1 * x117 * ((x104 * x115) + (x97 * x116))) + (x119 * x120)));
	const GEN_FLT x122 = -1 * x54;
	const GEN_FLT x123 =
		sin(1.5707963267949 + gibPhase_0 + (-1 * asin(x112 * x120)) + (-1 * phase_0) + (-1 * atan2(x47, x122))) *
		gibMag_0;
	const GEN_FLT x124 = 2 * x55 * pow((x113 + x55), -1) * atan2(x112, x122) * curve_0;
	const GEN_FLT x125 = x43 + x72;
	const GEN_FLT x126 = x35 + x77;
	const GEN_FLT x127 = x23 + x79;
	const GEN_FLT x128 = (x53 * x125) + (x52 * x126) + x96 + (x49 * x127);
	const GEN_FLT x129 = x56 * x128;
	const GEN_FLT x130 = (x41 * x125) + (x32 * x126) + (x10 * x127) + x103;
	const GEN_FLT x131 = (x111 * x125) + (x110 * x126) + x118 + (x108 * x127);
	const GEN_FLT x132 = (-1 * ((x47 * x129) + (-1 * x99 * x130)) * x107) +
						 (-1 * x114 * ((-1 * ((x115 * x130) + (x116 * x128)) * x117) + (x120 * x131)));
	const GEN_FLT x133 = x42 + x72;
	const GEN_FLT x134 = x33 + x77;
	const GEN_FLT x135 = x16 + x79;
	const GEN_FLT x136 = (x53 * x133) + x96 + (x52 * x134) + (x49 * x135);
	const GEN_FLT x137 = x56 * x136;
	const GEN_FLT x138 = (x41 * x133) + (x10 * x135) + (x32 * x134) + x103;
	const GEN_FLT x139 = (x110 * x134) + (x111 * x133) + x118 + (x108 * x135);
	const GEN_FLT x140 = (-1 * ((x47 * x137) + (-1 * x99 * x138)) * x107) +
						 (-1 * x114 * ((-1 * ((x115 * x138) + (x116 * x136)) * x117) + (x120 * x139)));
	out[0] = (x123 * x121) + (((-1 * x99 * x119) + (x98 * x112)) * x124) + x121;
	out[1] = (x123 * x132) + (x124 * ((-1 * x99 * x131) + (x112 * x129))) + x132;
	out[2] = (x123 * x140) + x140 + (x124 * ((-1 * x99 * x139) + (x112 * x137)));
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
	const GEN_FLT x0 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 0;
	const GEN_FLT x1 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   : 1e-10;
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 0;
	const GEN_FLT x5 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 1;
	const GEN_FLT x6 = cos(x1);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x2 * x4;
	const GEN_FLT x11 = x0 * x7;
	const GEN_FLT x12 = x5 * x11;
	const GEN_FLT x13 = x5 * x5;
	const GEN_FLT x14 =
		obj_px + ((x3 + x9) * sensor_z) + (((-1 * x10) + x12) * sensor_y) + (((x7 * x13) + x6) * sensor_x);
	const GEN_FLT x15 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 0;
	const GEN_FLT x16 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x17 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							: 1e-10;
	const GEN_FLT x18 = cos(x17);
	const GEN_FLT x19 = x18 * x16;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = sin(x17);
	const GEN_FLT x22 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 1;
	const GEN_FLT x25 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 0;
	const GEN_FLT x26 = x25 * x21;
	const GEN_FLT x27 = x24 * x26;
	const GEN_FLT x28 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x29 = 1 + (-1 * x18);
	const GEN_FLT x30 = x25 * x29;
	const GEN_FLT x31 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x32 = x24 * x29;
	const GEN_FLT x33 = (x27 * x16) + (x30 * x28) + (x32 * x31);
	const GEN_FLT x34 = x24 * x19;
	const GEN_FLT x35 = x21 * x28;
	const GEN_FLT x36 = x21 * x15;
	const GEN_FLT x37 = x36 * x25;
	const GEN_FLT x38 = x29 * x15;
	const GEN_FLT x39 = (x37 * x16) + (x31 * x38) + (x30 * x22);
	const GEN_FLT x40 = x2 * x5;
	const GEN_FLT x41 = x4 * x11;
	const GEN_FLT x42 = x0 * x0;
	const GEN_FLT x43 =
		obj_py + (((-1 * x40) + x41) * sensor_z) + ((x10 + x12) * sensor_x) + (((x7 * x42) + x6) * sensor_y);
	const GEN_FLT x44 = x4 * x4;
	const GEN_FLT x45 =
		obj_pz + (((x7 * x44) + x6) * sensor_z) + ((x40 + x41) * sensor_y) + (((-1 * x3) + x9) * sensor_x);
	const GEN_FLT x46 = x21 * x16;
	const GEN_FLT x47 = -1 * x46;
	const GEN_FLT x48 = x25 * x25;
	const GEN_FLT x49 = 2 * x30;
	const GEN_FLT x50 = x30 * x24;
	const GEN_FLT x51 = (-1 * x36) + x50;
	const GEN_FLT x52 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x53 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x54 = x2 * x53;
	const GEN_FLT x55 = -1 * x54;
	const GEN_FLT x56 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x57 = x2 * x56;
	const GEN_FLT x58 = x6 * x53;
	const GEN_FLT x59 = x4 * x58;
	const GEN_FLT x60 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x61 = x53 * x40;
	const GEN_FLT x62 = (x8 * x60) + (x0 * x61) + (x52 * x11);
	const GEN_FLT x63 = x2 * x60;
	const GEN_FLT x64 = x0 * x58;
	const GEN_FLT x65 = x4 * x7;
	const GEN_FLT x66 = (x65 * x52) + (x4 * x61) + (x8 * x56);
	const GEN_FLT x67 = (((2 * x8 * x52) + x55 + (x54 * x13)) * sensor_x) +
						(((-1 * x57) + (-1 * x59) + x62) * sensor_y) + ((x63 + x64 + x66) * sensor_z);
	const GEN_FLT x68 = x2 * x52;
	const GEN_FLT x69 = x5 * x58;
	const GEN_FLT x70 = (x60 * x65) + (x56 * x11) + (x0 * x53 * x10);
	const GEN_FLT x71 = ((x57 + x59 + x62) * sensor_x) + ((x55 + (2 * x60 * x11) + (x54 * x42)) * sensor_y) +
						(((-1 * x68) + x70 + (-1 * x69)) * sensor_z);
	const GEN_FLT x72 = x24 * x21;
	const GEN_FLT x73 = x30 * x15;
	const GEN_FLT x74 = x72 + x73;
	const GEN_FLT x75 = (x48 * x29) + x18;
	const GEN_FLT x76 = (((-1 * x63) + (-1 * x64) + x66) * sensor_x) + ((x68 + x70 + x69) * sensor_y) +
						((x55 + (2 * x65 * x56) + (x54 * x44)) * sensor_z);
	const GEN_FLT x77 = (x67 * x51) + (x71 * x74) + (x75 * x76);
	const GEN_FLT x78 = (x14 * ((-1 * x20) + (-1 * x23) + x33)) + (x43 * (x34 + x35 + x39)) +
						(x45 * (x47 + (x49 * x31) + (x46 * x48))) + x77;
	const GEN_FLT x79 = lh_pz + (x75 * x45) + (x51 * x14) + (x74 * x43);
	const GEN_FLT x80 = x79 * x79;
	const GEN_FLT x81 = pow(x80, -1);
	const GEN_FLT x82 = x36 + x50;
	const GEN_FLT x83 = x38 * x24;
	const GEN_FLT x84 = (-1 * x26) + x83;
	const GEN_FLT x85 = x24 * x24;
	const GEN_FLT x86 = (x85 * x29) + x18;
	const GEN_FLT x87 = lh_px + (x82 * x45) + (x84 * x43) + (x86 * x14);
	const GEN_FLT x88 = x81 * x87;
	const GEN_FLT x89 = x88 * x78;
	const GEN_FLT x90 = pow(x79, -1);
	const GEN_FLT x91 = 2 * x32;
	const GEN_FLT x92 = x25 * x18;
	const GEN_FLT x93 = x92 * x16;
	const GEN_FLT x94 = x31 * x21;
	const GEN_FLT x95 = x36 * x24;
	const GEN_FLT x96 = (x38 * x28) + (x32 * x22) + (x95 * x16);
	const GEN_FLT x97 = (x86 * x67) + (x84 * x71) + (x82 * x76);
	const GEN_FLT x98 = (x14 * (x47 + (x85 * x46) + (x91 * x28))) + (x43 * ((-1 * x93) + (-1 * x94) + x96)) +
						(x45 * (x20 + x23 + x33)) + x97;
	const GEN_FLT x99 = 1 + x98;
	const GEN_FLT x100 = (x87 * x87) + x80;
	const GEN_FLT x101 = pow(x100, -1);
	const GEN_FLT x102 = x80 * x101;
	const GEN_FLT x103 = 2 * x87;
	const GEN_FLT x104 = 2 * x79;
	const GEN_FLT x105 = x78 * x104;
	const GEN_FLT x106 = (-1 * x72) + x73;
	const GEN_FLT x107 = x15 * x15;
	const GEN_FLT x108 = (x29 * x107) + x18;
	const GEN_FLT x109 = x26 + x83;
	const GEN_FLT x110 = lh_py + (x14 * x109) + (x45 * x106) + (x43 * x108);
	const GEN_FLT x111 = 1.0 / 2.0 * pow(x100, -3.0 / 2.0) * x110 * tilt_0;
	const GEN_FLT x112 = 2 * x38;
	const GEN_FLT x113 = (x71 * x108) + (x67 * x109) + (x76 * x106);
	const GEN_FLT x114 = (x14 * (x93 + x94 + x96)) + (x45 * ((-1 * x34) + (-1 * x35) + x39)) +
						 (x43 * (x47 + (x46 * x107) + (x22 * x112))) + x113;
	const GEN_FLT x115 = pow(x100, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x116 = x114 * x115;
	const GEN_FLT x117 = x110 * x110;
	const GEN_FLT x118 = pow((1 + (-1 * x101 * x117 * (tilt_0 * tilt_0))), -1.0 / 2.0);
	const GEN_FLT x119 =
		(-1 * x102 * (x89 + (-1 * x90 * x99))) + (-1 * x118 * ((-1 * x111 * ((x99 * x103) + x105)) + x116));
	const GEN_FLT x120 = -1 * x79;
	const GEN_FLT x121 =
		sin(1.5707963267949 + gibPhase_0 + (-1 * asin(x110 * x115)) + (-1 * phase_0) + (-1 * atan2(x87, x120))) *
		gibMag_0;
	const GEN_FLT x122 = -1 * x90 * x114;
	const GEN_FLT x123 = x81 * x110;
	const GEN_FLT x124 = x78 * x123;
	const GEN_FLT x125 = 2 * x80 * pow((x117 + x80), -1) * atan2(x110, x120) * curve_0;
	const GEN_FLT x126 = -1 * x90 * x98;
	const GEN_FLT x127 = x98 * x103;
	const GEN_FLT x128 = 1 + x114;
	const GEN_FLT x129 = (-1 * x102 * (x89 + x126)) + (-1 * x118 * ((-1 * (x127 + x105) * x111) + (x115 * x128)));
	const GEN_FLT x130 = 1 + x78;
	const GEN_FLT x131 =
		(-1 * x102 * ((x88 * x130) + x126)) + (-1 * x118 * ((-1 * x111 * (x127 + (x104 * x130))) + x116));
	const GEN_FLT x132 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x133 = x18 * x132;
	const GEN_FLT x134 = x15 * x133;
	const GEN_FLT x135 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qj *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x136 = x21 * x135;
	const GEN_FLT x137 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qi *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x138 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qk *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x139 = (x30 * x137) + (x32 * x138) + (x27 * x132);
	const GEN_FLT x140 = x24 * x133;
	const GEN_FLT x141 = x21 * x137;
	const GEN_FLT x142 = x36 * x132;
	const GEN_FLT x143 = (x25 * x142) + (x38 * x138) + (x30 * x135);
	const GEN_FLT x144 = x21 * x132;
	const GEN_FLT x145 = -1 * x144;
	const GEN_FLT x146 = (x14 * ((-1 * x134) + (-1 * x136) + x139)) + (x43 * (x140 + x141 + x143)) +
						 (x45 * (x145 + (x48 * x144) + (x49 * x138))) + x77;
	const GEN_FLT x147 = x81 * x146;
	const GEN_FLT x148 = x92 * x132;
	const GEN_FLT x149 = x21 * x138;
	const GEN_FLT x150 = (x24 * x142) + (x38 * x137) + (x32 * x135);
	const GEN_FLT x151 = (x14 * (x145 + (x85 * x144) + (x91 * x137))) + (x43 * ((-1 * x148) + (-1 * x149) + x150)) +
						 (x45 * (x134 + x136 + x139)) + x97;
	const GEN_FLT x152 = (x43 * (x145 + (x107 * x144) + (x112 * x135))) + (x45 * ((-1 * x140) + (-1 * x141) + x143)) +
						 (x14 * (x148 + x149 + x150)) + x113;
	const GEN_FLT x153 = (-1 * ((x87 * x147) + (-1 * x90 * x151)) * x102) +
						 (-1 * x118 * ((-1 * ((x103 * x151) + (x104 * x146)) * x111) + (x115 * x152)));
	const GEN_FLT x154 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x155 = x15 * x18;
	const GEN_FLT x156 = x154 * x155;
	const GEN_FLT x157 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qj *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x158 = x21 * x157;
	const GEN_FLT x159 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qi *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x160 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qk *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x161 = (x27 * x154) + (x32 * x160) + (x30 * x159);
	const GEN_FLT x162 = x24 * x18;
	const GEN_FLT x163 = x162 * x154;
	const GEN_FLT x164 = x21 * x159;
	const GEN_FLT x165 = (x37 * x154) + (x38 * x160) + (x30 * x157);
	const GEN_FLT x166 = x21 * x154;
	const GEN_FLT x167 = -1 * x166;
	const GEN_FLT x168 = (x14 * ((-1 * x156) + (-1 * x158) + x161)) + (x45 * (x167 + (x48 * x166) + (x49 * x160))) +
						 (x43 * (x163 + x164 + x165)) + x77;
	const GEN_FLT x169 = x81 * x168;
	const GEN_FLT x170 = x92 * x154;
	const GEN_FLT x171 = x21 * x160;
	const GEN_FLT x172 = (x95 * x154) + (x32 * x157) + (x38 * x159);
	const GEN_FLT x173 = (x14 * ((x85 * x166) + x167 + (x91 * x159))) + (x45 * (x156 + x158 + x161)) +
						 (x43 * ((-1 * x170) + (-1 * x171) + x172)) + x97;
	const GEN_FLT x174 = (x14 * (x170 + x171 + x172)) + (x43 * (x167 + (x107 * x166) + (x112 * x157))) +
						 (x45 * ((-1 * x163) + (-1 * x164) + x165)) + x113;
	const GEN_FLT x175 = (-1 * ((x87 * x169) + (-1 * x90 * x173)) * x102) +
						 (-1 * x118 * ((-1 * ((x103 * x173) + (x104 * x168)) * x111) + (x115 * x174)));
	const GEN_FLT x176 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x177 = x176 * x155;
	const GEN_FLT x178 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qj *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x179 = x21 * x178;
	const GEN_FLT x180 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qi *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x181 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qk *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x182 = (x27 * x176) + (x30 * x180) + (x32 * x181);
	const GEN_FLT x183 = x162 * x176;
	const GEN_FLT x184 = x21 * x180;
	const GEN_FLT x185 = (x37 * x176) + (x38 * x181) + (x30 * x178);
	const GEN_FLT x186 = x21 * x176;
	const GEN_FLT x187 = -1 * x186;
	const GEN_FLT x188 = (x14 * ((-1 * x177) + x182 + (-1 * x179))) + (x43 * (x183 + x184 + x185)) +
						 (x45 * (x187 + (x49 * x181) + (x48 * x186))) + x77;
	const GEN_FLT x189 = x92 * x176;
	const GEN_FLT x190 = x21 * x181;
	const GEN_FLT x191 = (x95 * x176) + (x38 * x180) + (x32 * x178);
	const GEN_FLT x192 = (x14 * (x187 + (x85 * x186) + (x91 * x180))) + (x43 * ((-1 * x189) + (-1 * x190) + x191)) +
						 (x45 * (x177 + x182 + x179)) + x97;
	const GEN_FLT x193 = (x14 * (x189 + x190 + x191)) + (x43 * (x187 + (x107 * x186) + (x112 * x178))) +
						 (x45 * ((-1 * x183) + (-1 * x184) + x185)) + x113;
	const GEN_FLT x194 = (-1 * ((x88 * x188) + (-1 * x90 * x192)) * x102) +
						 (-1 * x118 * ((-1 * ((x103 * x192) + (x104 * x188)) * x111) + (x115 * x193)));
	out[0] = (x119 * x121) + ((x122 + x124) * x125) + x119;
	out[1] = (x121 * x129) + (x125 * ((-1 * x90 * x128) + x124)) + x129;
	out[2] = (x121 * x131) + (x125 * (x122 + (x123 * x130))) + x131;
	out[3] = (x121 * x153) + (x125 * ((-1 * x90 * x152) + (x110 * x147))) + x153;
	out[4] = (x121 * x175) + x175 + (x125 * ((-1 * x90 * x174) + (x110 * x169)));
	out[5] = (x121 * x194) + (x125 * ((-1 * x90 * x193) + (x123 * x188))) + x194;
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 + (-1 * x1);
	const GEN_FLT x3 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x4 = x3 * x3;
	const GEN_FLT x5 = (x2 * x4) + x1;
	const GEN_FLT x6 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 0;
	const GEN_FLT x7 = x6 * x6;
	const GEN_FLT x8 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   : 1e-10;
	const GEN_FLT x9 = cos(x8);
	const GEN_FLT x10 = 1 + (-1 * x9);
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x12 = sin(x8);
	const GEN_FLT x13 = x12 * x11;
	const GEN_FLT x14 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x15 = x14 * x10;
	const GEN_FLT x16 = x6 * x15;
	const GEN_FLT x17 = x14 * x12;
	const GEN_FLT x18 = x6 * x10;
	const GEN_FLT x19 = x11 * x18;
	const GEN_FLT x20 =
		obj_pz + (((x7 * x10) + x9) * sensor_z) + ((x13 + x16) * sensor_y) + (((-1 * x17) + x19) * sensor_x);
	const GEN_FLT x21 = sin(x0);
	const GEN_FLT x22 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 1;
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 0;
	const GEN_FLT x25 = x2 * x3;
	const GEN_FLT x26 = x24 * x25;
	const GEN_FLT x27 = x23 + x26;
	const GEN_FLT x28 = x14 * x14;
	const GEN_FLT x29 = x6 * x12;
	const GEN_FLT x30 = x15 * x11;
	const GEN_FLT x31 =
		obj_py + (((x28 * x10) + x9) * sensor_y) + (((-1 * x13) + x16) * sensor_z) + ((x29 + x30) * sensor_x);
	const GEN_FLT x32 = x24 * x21;
	const GEN_FLT x33 = x25 * x22;
	const GEN_FLT x34 = (-1 * x32) + x33;
	const GEN_FLT x35 = x11 * x11;
	const GEN_FLT x36 =
		obj_px + ((x17 + x19) * sensor_z) + (((-1 * x29) + x30) * sensor_y) + (((x35 * x10) + x9) * sensor_x);
	const GEN_FLT x37 = lh_pz + (x5 * x20) + (x31 * x27) + (x34 * x36);
	const GEN_FLT x38 = x37 * x37;
	const GEN_FLT x39 = (-1 * x23) + x26;
	const GEN_FLT x40 = x24 * x24;
	const GEN_FLT x41 = (x2 * x40) + x1;
	const GEN_FLT x42 = x3 * x21;
	const GEN_FLT x43 = x2 * x22;
	const GEN_FLT x44 = x43 * x24;
	const GEN_FLT x45 = x42 + x44;
	const GEN_FLT x46 = lh_py + (x39 * x20) + (x41 * x31) + (x45 * x36);
	const GEN_FLT x47 = x46 * x46;
	const GEN_FLT x48 = pow(x37, -1);
	const GEN_FLT x49 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x50 = x1 * x49;
	const GEN_FLT x51 = x3 * x50;
	const GEN_FLT x52 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x53 = x52 * x21;
	const GEN_FLT x54 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x55 = x2 * x54;
	const GEN_FLT x56 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x57 = x49 * x24;
	const GEN_FLT x58 = (x55 * x24) + (x56 * x43) + (x57 * x23);
	const GEN_FLT x59 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x60 = x11 * x10;
	const GEN_FLT x61 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x62 = x61 * x12;
	const GEN_FLT x63 = -1 * x62;
	const GEN_FLT x64 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x65 = x64 * x12;
	const GEN_FLT x66 = x9 * x61;
	const GEN_FLT x67 = x6 * x66;
	const GEN_FLT x68 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x69 = x61 * x13;
	const GEN_FLT x70 = (x60 * x68) + (x69 * x14) + (x59 * x15);
	const GEN_FLT x71 = x68 * x12;
	const GEN_FLT x72 = x66 * x14;
	const GEN_FLT x73 = (x59 * x18) + (x6 * x69) + (x60 * x64);
	const GEN_FLT x74 = (((2 * x60 * x59) + x63 + (x62 * x35)) * sensor_x) +
						(((-1 * x65) + (-1 * x67) + x70) * sensor_y) + ((x71 + x72 + x73) * sensor_z);
	const GEN_FLT x75 = x49 * x21;
	const GEN_FLT x76 = -1 * x75;
	const GEN_FLT x77 = x2 * x24;
	const GEN_FLT x78 = x59 * x12;
	const GEN_FLT x79 = x66 * x11;
	const GEN_FLT x80 = (x68 * x18) + (x64 * x15) + (x61 * x29 * x14);
	const GEN_FLT x81 = ((x65 + x67 + x70) * sensor_x) + ((x63 + (2 * x68 * x15) + (x62 * x28)) * sensor_y) +
						(((-1 * x78) + (-1 * x79) + x80) * sensor_z);
	const GEN_FLT x82 = x50 * x22;
	const GEN_FLT x83 = x54 * x21;
	const GEN_FLT x84 = (x57 * x42) + (x77 * x52) + (x56 * x25);
	const GEN_FLT x85 = (((-1 * x71) + (-1 * x72) + x73) * sensor_x) + ((x78 + x79 + x80) * sensor_y) +
						((x63 + (2 * x64 * x18) + (x7 * x62)) * sensor_z);
	const GEN_FLT x86 = (x36 * (x51 + x53 + x58)) + (x74 * x45) + (x31 * (x76 + (x75 * x40) + (2 * x77 * x56))) +
						(x81 * x41) + (x20 * ((-1 * x82) + (-1 * x83) + x84)) + (x85 * x39);
	const GEN_FLT x87 = x50 * x24;
	const GEN_FLT x88 = x56 * x21;
	const GEN_FLT x89 = (x3 * x49 * x23) + (x3 * x55) + (x52 * x43);
	const GEN_FLT x90 = (x36 * ((-1 * x87) + (-1 * x88) + x89)) + (x81 * x27) + (x74 * x34) +
						(x31 * (x82 + x83 + x84)) + (x20 * (x76 + (x4 * x75) + (2 * x52 * x25))) + (x5 * x85);
	const GEN_FLT x91 = x90 * pow(x38, -1);
	const GEN_FLT x92 = -1 * x37;
	const GEN_FLT x93 = atan2(x46, x92);
	const GEN_FLT x94 = 2 * pow((x47 + x38), -1) * ((-1 * x86 * x48) + (x91 * x46)) * x93 * x38 * curve_0;
	const GEN_FLT x95 = x32 + x33;
	const GEN_FLT x96 = (-1 * x42) + x44;
	const GEN_FLT x97 = x22 * x22;
	const GEN_FLT x98 = (x2 * x97) + x1;
	const GEN_FLT x99 = lh_px + (x95 * x20) + (x96 * x31) + (x98 * x36);
	const GEN_FLT x100 = (x36 * (x76 + (x75 * x97) + (2 * x55 * x22))) + (x74 * x98) + (x81 * x96) +
						 (x31 * ((-1 * x51) + (-1 * x53) + x58)) + (x20 * (x87 + x88 + x89)) + (x85 * x95);
	const GEN_FLT x101 = (x99 * x99) + x38;
	const GEN_FLT x102 = pow(x101, -1);
	const GEN_FLT x103 = -1 * x38 * x102 * ((x91 * x99) + (-1 * x48 * x100));
	const GEN_FLT x104 = pow((1 + (-1 * x47 * x102 * (tilt_0 * tilt_0))), -1.0 / 2.0);
	const GEN_FLT x105 = pow(x101, -1.0 / 2.0);
	const GEN_FLT x106 = (-1.0 / 2.0 * x46 * pow(x101, -3.0 / 2.0) * tilt_0 * ((2 * x99 * x100) + (2 * x90 * x37))) +
						 (x86 * x105 * tilt_0);
	const GEN_FLT x107 = x103 + (-1 * x104 * x106);
	const GEN_FLT x108 = -1 + x107;
	const GEN_FLT x109 = x46 * x105;
	const GEN_FLT x110 =
		1.5707963267949 + gibPhase_0 + (-1 * asin(x109 * tilt_0)) + (-1 * phase_0) + (-1 * atan2(x99, x92));
	const GEN_FLT x111 = sin(x110) * gibMag_0;
	const GEN_FLT x112 = x103 + (-1 * (x109 + x106) * x104);
	const GEN_FLT x113 = x94 + x107;
	const GEN_FLT x114 = (x107 * x111) + x113;
	out[0] = x94 + (x108 * x111) + x108;
	out[1] = x94 + (x111 * x112) + x112;
	out[2] = (x93 * x93) + x114;
	out[3] = (x111 * (1 + x107)) + x113;
	out[4] = (-1 * cos(x110)) + x114;
	out[5] = x114;
	out[6] = x114;
}

/** Applying function <function reproject_axis_y at 0x7f35a4af8320> */
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = cos(x0);
	const GEN_FLT x5 = 1 + (-1 * x4);
	const GEN_FLT x6 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x8 = x6 * x5 * x7;
	const GEN_FLT x9 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 0;
	const GEN_FLT x10 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x11 = cos(x10);
	const GEN_FLT x12 = 1 + (-1 * x11);
	const GEN_FLT x13 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x14 = sin(x10);
	const GEN_FLT x15 = x14 * x13;
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x17 = x12 * x16;
	const GEN_FLT x18 = x9 * x17;
	const GEN_FLT x19 = x14 * x16;
	const GEN_FLT x20 = x9 * x13 * x12;
	const GEN_FLT x21 =
		obj_pz + ((((x9 * x9) * x12) + x11) * sensor_z) + ((x15 + x18) * sensor_y) + (((-1 * x19) + x20) * sensor_x);
	const GEN_FLT x22 = x1 * x7;
	const GEN_FLT x23 = x2 * x5;
	const GEN_FLT x24 = x6 * x23;
	const GEN_FLT x25 = x9 * x14;
	const GEN_FLT x26 = x13 * x17;
	const GEN_FLT x27 =
		obj_py + (((-1 * x15) + x18) * sensor_z) + (((x12 * (x16 * x16)) + x11) * sensor_y) + ((x25 + x26) * sensor_x);
	const GEN_FLT x28 =
		obj_px + ((x19 + x20) * sensor_z) + ((((x13 * x13) * x12) + x11) * sensor_x) + (((-1 * x25) + x26) * sensor_y);
	const GEN_FLT x29 = lh_px + ((x3 + x8) * x21) + (((-1 * x22) + x24) * x27) + (x28 * (((x6 * x6) * x5) + x4));
	const GEN_FLT x30 = x1 * x6;
	const GEN_FLT x31 = x7 * x23;
	const GEN_FLT x32 = lh_pz + ((x30 + x31) * x27) + (x21 * ((x5 * (x7 * x7)) + x4)) + (((-1 * x3) + x8) * x28);
	const GEN_FLT x33 = lh_py + (((-1 * x30) + x31) * x21) + (x27 * (((x2 * x2) * x5) + x4)) + ((x22 + x24) * x28);
	const GEN_FLT x34 = -1 * x32;
	const GEN_FLT x35 = (-1 * asin(pow(((x32 * x32) + (x33 * x33)), -1.0 / 2.0) * x29 * tilt_1)) + (-1 * phase_1) +
						(-1 * atan2(-1 * x33, x34));
	out[0] = (-1 * cos(1.5707963267949 + gibPhase_1 + x35) * gibMag_1) +
			 ((atan2(x29, x34) * atan2(x29, x34)) * curve_1) + x35;
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
	const GEN_FLT x0 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (-1 * obj_qk *
							  pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									   ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									   : 1e-10),
								  -2) *
							  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
						   : 0;
	const GEN_FLT x1 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   : 1e-10;
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 0;
	const GEN_FLT x5 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x6 = cos(x1);
	const GEN_FLT x7 = x6 * x5;
	const GEN_FLT x8 = x4 * x7;
	const GEN_FLT x9 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (-1 * obj_qj *
							  pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									   ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									   : 1e-10),
								  -2) *
							  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
						   : 0;
	const GEN_FLT x10 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x11 = 1 + (-1 * x6);
	const GEN_FLT x12 = x11 * x10;
	const GEN_FLT x13 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x14 = x2 * x10;
	const GEN_FLT x15 = x14 * x13;
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x17 = x13 * x11;
	const GEN_FLT x18 = (x9 * x12) + (x5 * x15) + (x17 * x16);
	const GEN_FLT x19 = x2 * x5;
	const GEN_FLT x20 = -1 * x19;
	const GEN_FLT x21 = 2 * x17;
	const GEN_FLT x22 = x13 * x13;
	const GEN_FLT x23 = x2 * x16;
	const GEN_FLT x24 = x6 * x10;
	const GEN_FLT x25 = x5 * x24;
	const GEN_FLT x26 = x4 * x11;
	const GEN_FLT x27 = x2 * x4;
	const GEN_FLT x28 = (x9 * x26) + (x0 * x17) + (x5 * x27 * x13);
	const GEN_FLT x29 = ((x3 + x8 + x18) * sensor_x) + ((x20 + (x9 * x21) + (x22 * x19)) * sensor_y) +
						(((-1 * x23) + (-1 * x25) + x28) * sensor_z);
	const GEN_FLT x30 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							: 1e-10;
	const GEN_FLT x31 = sin(x30);
	const GEN_FLT x32 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 1;
	const GEN_FLT x33 = x32 * x31;
	const GEN_FLT x34 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 0;
	const GEN_FLT x35 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 0;
	const GEN_FLT x36 = cos(x30);
	const GEN_FLT x37 = 1 + (-1 * x36);
	const GEN_FLT x38 = x35 * x37;
	const GEN_FLT x39 = x34 * x38;
	const GEN_FLT x40 = x33 + x39;
	const GEN_FLT x41 = x40 * x29;
	const GEN_FLT x42 = x31 * x34;
	const GEN_FLT x43 = x32 * x37;
	const GEN_FLT x44 = x43 * x35;
	const GEN_FLT x45 = (-1 * x42) + x44;
	const GEN_FLT x46 = 2 * x12;
	const GEN_FLT x47 = x10 * x10;
	const GEN_FLT x48 = x2 * x9;
	const GEN_FLT x49 = x7 * x13;
	const GEN_FLT x50 = x4 * x14;
	const GEN_FLT x51 = (x26 * x16) + (x5 * x50) + (x0 * x12);
	const GEN_FLT x52 = (((x46 * x16) + x20 + (x47 * x19)) * sensor_x) + ((x48 + x49 + x51) * sensor_z) +
						(((-1 * x3) + (-1 * x8) + x18) * sensor_y);
	const GEN_FLT x53 = 1 + x52;
	const GEN_FLT x54 = x35 * x35;
	const GEN_FLT x55 = (x54 * x37) + x36;
	const GEN_FLT x56 = 2 * x26;
	const GEN_FLT x57 = x4 * x4;
	const GEN_FLT x58 = (((-1 * x48) + (-1 * x49) + x51) * sensor_x) + ((x23 + x25 + x28) * sensor_y) +
						((x20 + (x0 * x56) + (x57 * x19)) * sensor_z);
	const GEN_FLT x59 = x2 * x13;
	const GEN_FLT x60 = x26 * x10;
	const GEN_FLT x61 = x10 * x17;
	const GEN_FLT x62 =
		obj_px + ((x59 + x60) * sensor_z) + (((-1 * x27) + x61) * sensor_y) + (((x47 * x11) + x6) * sensor_x);
	const GEN_FLT x63 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x64 = x63 * x36;
	const GEN_FLT x65 = x64 * x34;
	const GEN_FLT x66 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x67 = x66 * x31;
	const GEN_FLT x68 = x31 * x35;
	const GEN_FLT x69 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x70 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x71 = (x63 * x68 * x32) + (x69 * x38) + (x70 * x43);
	const GEN_FLT x72 = x64 * x32;
	const GEN_FLT x73 = x69 * x31;
	const GEN_FLT x74 = x63 * x42;
	const GEN_FLT x75 = x34 * x37;
	const GEN_FLT x76 = (x74 * x35) + (x70 * x75) + (x66 * x38);
	const GEN_FLT x77 = x26 * x13;
	const GEN_FLT x78 =
		obj_py + (((-1 * x14) + x77) * sensor_z) + (((x22 * x11) + x6) * sensor_y) + ((x27 + x61) * sensor_x);
	const GEN_FLT x79 =
		obj_pz + (((x57 * x11) + x6) * sensor_z) + ((x14 + x77) * sensor_y) + (((-1 * x59) + x60) * sensor_x);
	const GEN_FLT x80 = x63 * x31;
	const GEN_FLT x81 = -1 * x80;
	const GEN_FLT x82 = (x62 * ((-1 * x65) + (-1 * x67) + x71)) + (x78 * (x72 + x73 + x76)) +
						(x79 * (x81 + (x80 * x54) + (2 * x70 * x38)));
	const GEN_FLT x83 = (x58 * x55) + x82;
	const GEN_FLT x84 = x41 + (x53 * x45) + x83;
	const GEN_FLT x85 = x42 + x44;
	const GEN_FLT x86 = x43 * x34;
	const GEN_FLT x87 = (-1 * x68) + x86;
	const GEN_FLT x88 = x32 * x32;
	const GEN_FLT x89 = (x88 * x37) + x36;
	const GEN_FLT x90 = lh_px + (x85 * x79) + (x87 * x78) + (x89 * x62);
	const GEN_FLT x91 = lh_pz + (x79 * x55) + (x78 * x40) + (x62 * x45);
	const GEN_FLT x92 = x91 * x91;
	const GEN_FLT x93 = pow(x92, -1);
	const GEN_FLT x94 = x93 * x90;
	const GEN_FLT x95 = pow(x91, -1);
	const GEN_FLT x96 = x87 * x29;
	const GEN_FLT x97 = x64 * x35;
	const GEN_FLT x98 = x70 * x31;
	const GEN_FLT x99 = (x75 * x69) + (x66 * x43) + (x74 * x32);
	const GEN_FLT x100 = (x62 * (x81 + (x80 * x88) + (2 * x69 * x43))) + (x78 * ((-1 * x97) + (-1 * x98) + x99)) +
						 (x79 * (x65 + x67 + x71));
	const GEN_FLT x101 = (x85 * x58) + x100;
	const GEN_FLT x102 = x96 + (x89 * x53) + x101;
	const GEN_FLT x103 = x90 * x90;
	const GEN_FLT x104 = -1 * x91;
	const GEN_FLT x105 = 2 * x92 * atan2(x90, x104) * pow((x103 + x92), -1) * curve_1;
	const GEN_FLT x106 = x68 + x86;
	const GEN_FLT x107 = x34 * x34;
	const GEN_FLT x108 = (x37 * x107) + x36;
	const GEN_FLT x109 = x29 * x108;
	const GEN_FLT x110 = (-1 * x33) + x39;
	const GEN_FLT x111 = (x62 * (x97 + x98 + x99)) + (x78 * (x81 + (x80 * x107) + (2 * x75 * x66))) +
						 (x79 * ((-1 * x72) + (-1 * x73) + x76));
	const GEN_FLT x112 = (x58 * x110) + x111;
	const GEN_FLT x113 = (x53 * x106) + x109 + x112;
	const GEN_FLT x114 = lh_py + (x79 * x110) + (x78 * x108) + (x62 * x106);
	const GEN_FLT x115 = x93 * x114;
	const GEN_FLT x116 = (x114 * x114) + x92;
	const GEN_FLT x117 = pow(x116, -1);
	const GEN_FLT x118 = x92 * x117;
	const GEN_FLT x119 = 2 * x114;
	const GEN_FLT x120 = 2 * x91;
	const GEN_FLT x121 = 1.0 / 2.0 * x90 * pow(x116, -3.0 / 2.0) * tilt_1;
	const GEN_FLT x122 = pow(x116, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x123 = pow((1 + (-1 * x103 * x117 * (tilt_1 * tilt_1))), -1.0 / 2.0);
	const GEN_FLT x124 = (-1 * ((x95 * x113) + (-1 * x84 * x115)) * x118) +
						 (-1 * x123 * ((-1 * x121 * ((x113 * x119) + (x84 * x120))) + (x102 * x122)));
	const GEN_FLT x125 =
		sin(1.5707963267949 + gibPhase_1 + (-1 * atan2(-1 * x114, x104)) + (-1 * asin(x90 * x122)) + (-1 * phase_1)) *
		gibMag_1;
	const GEN_FLT x126 = x52 * x45;
	const GEN_FLT x127 = 1 + x29;
	const GEN_FLT x128 = x126 + (x40 * x127) + x83;
	const GEN_FLT x129 = x89 * x52;
	const GEN_FLT x130 = x129 + (x87 * x127) + x101;
	const GEN_FLT x131 = x52 * x106;
	const GEN_FLT x132 = x131 + (x108 * x127) + x112;
	const GEN_FLT x133 = (-1 * x118 * ((x95 * x132) + (-1 * x115 * x128))) +
						 (-1 * x123 * ((-1 * ((x119 * x132) + (x120 * x128)) * x121) + (x122 * x130)));
	const GEN_FLT x134 = 1 + x58;
	const GEN_FLT x135 = x126 + x41 + x82 + (x55 * x134);
	const GEN_FLT x136 = x96 + x129 + (x85 * x134) + x100;
	const GEN_FLT x137 = x131 + x109 + (x110 * x134) + x111;
	const GEN_FLT x138 = (-1 * x118 * ((x95 * x137) + (-1 * x115 * x135))) +
						 (-1 * x123 * ((-1 * ((x119 * x137) + (x120 * x135)) * x121) + (x122 * x136)));
	const GEN_FLT x139 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x140 = x2 * x139;
	const GEN_FLT x141 = -1 * x140;
	const GEN_FLT x142 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qi *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x143 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qk *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x144 = x2 * x143;
	const GEN_FLT x145 = x6 * x139;
	const GEN_FLT x146 = x4 * x145;
	const GEN_FLT x147 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qj *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qi * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x148 = (x17 * x142) + (x12 * x147) + (x15 * x139);
	const GEN_FLT x149 = x2 * x147;
	const GEN_FLT x150 = x13 * x145;
	const GEN_FLT x151 = (x26 * x142) + (x12 * x143) + (x50 * x139);
	const GEN_FLT x152 = ((x141 + (x46 * x142) + (x47 * x140)) * sensor_x) +
						 (((-1 * x144) + (-1 * x146) + x148) * sensor_y) + ((x149 + x150 + x151) * sensor_z);
	const GEN_FLT x153 = x2 * x142;
	const GEN_FLT x154 = x24 * x139;
	const GEN_FLT x155 = x4 * x13;
	const GEN_FLT x156 = (x26 * x147) + (x17 * x143) + (x140 * x155);
	const GEN_FLT x157 = ((x144 + x146 + x148) * sensor_x) + (((x21 * x147) + x141 + (x22 * x140)) * sensor_y) +
						 (((-1 * x153) + (-1 * x154) + x156) * sensor_z);
	const GEN_FLT x158 = (((-1 * x149) + (-1 * x150) + x151) * sensor_x) + ((x153 + x154 + x156) * sensor_y) +
						 ((x141 + (x56 * x143) + (x57 * x140)) * sensor_z);
	const GEN_FLT x159 = (x45 * x152) + (x40 * x157) + (x55 * x158) + x82;
	const GEN_FLT x160 = (x89 * x152) + (x87 * x157) + (x85 * x158) + x100;
	const GEN_FLT x161 = (x106 * x152) + (x108 * x157) + (x110 * x158) + x111;
	const GEN_FLT x162 = (-1 * x118 * ((x95 * x161) + (-1 * x115 * x159))) +
						 (-1 * x123 * ((-1 * ((x119 * x161) + (x120 * x159)) * x121) + (x122 * x160)));
	const GEN_FLT x163 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x164 = x2 * x163;
	const GEN_FLT x165 = -1 * x164;
	const GEN_FLT x166 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qi *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x167 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qk *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x168 = x2 * x167;
	const GEN_FLT x169 = x6 * x163;
	const GEN_FLT x170 = x4 * x169;
	const GEN_FLT x171 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qj *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qj * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x172 = (x17 * x166) + (x12 * x171) + (x15 * x163);
	const GEN_FLT x173 = x2 * x171;
	const GEN_FLT x174 = x13 * x169;
	const GEN_FLT x175 = (x26 * x166) + (x50 * x163) + (x12 * x167);
	const GEN_FLT x176 = ((x165 + (x46 * x166) + (x47 * x164)) * sensor_x) +
						 (((-1 * x168) + (-1 * x170) + x172) * sensor_y) + ((x173 + x174 + x175) * sensor_z);
	const GEN_FLT x177 = x2 * x166;
	const GEN_FLT x178 = x10 * x169;
	const GEN_FLT x179 = (x17 * x167) + (x164 * x155) + (x26 * x171);
	const GEN_FLT x180 = ((x165 + (x21 * x171) + (x22 * x164)) * sensor_y) + ((x168 + x170 + x172) * sensor_x) +
						 (((-1 * x177) + (-1 * x178) + x179) * sensor_z);
	const GEN_FLT x181 = (((-1 * x173) + (-1 * x174) + x175) * sensor_x) + ((x177 + x178 + x179) * sensor_y) +
						 ((x165 + (x56 * x167) + (x57 * x164)) * sensor_z);
	const GEN_FLT x182 = (x45 * x176) + (x40 * x180) + (x55 * x181) + x82;
	const GEN_FLT x183 = (x89 * x176) + (x87 * x180) + (x85 * x181) + x100;
	const GEN_FLT x184 = (x106 * x176) + (x108 * x180) + (x110 * x181) + x111;
	const GEN_FLT x185 = (-1 * x118 * ((x95 * x184) + (-1 * x115 * x182))) +
						 (-1 * x123 * ((-1 * ((x119 * x184) + (x120 * x182)) * x121) + (x122 * x183)));
	const GEN_FLT x186 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x187 = x2 * x186;
	const GEN_FLT x188 = -1 * x187;
	const GEN_FLT x189 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qi *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x190 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? ((-1 * obj_qk *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					 : 0) *
				pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						 : 1e-10),
					-2)) +
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -1))
			: 0;
	const GEN_FLT x191 = x2 * x190;
	const GEN_FLT x192 = x6 * x186;
	const GEN_FLT x193 = x4 * x192;
	const GEN_FLT x194 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  : 1e-10))
			? (-1 * obj_qj *
			   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					? (obj_qk * pow(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)), -1.0 / 2.0))
					: 0) *
			   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						: 1e-10),
				   -2))
			: 0;
	const GEN_FLT x195 = (x17 * x189) + (x15 * x186) + (x12 * x194);
	const GEN_FLT x196 = x2 * x194;
	const GEN_FLT x197 = x13 * x192;
	const GEN_FLT x198 = (x26 * x189) + (x50 * x186) + (x12 * x190);
	const GEN_FLT x199 = ((x188 + (x46 * x189) + (x47 * x187)) * sensor_x) +
						 (((-1 * x191) + x195 + (-1 * x193)) * sensor_y) + ((x196 + x197 + x198) * sensor_z);
	const GEN_FLT x200 = x2 * x189;
	const GEN_FLT x201 = x24 * x186;
	const GEN_FLT x202 = (x26 * x194) + (x17 * x190) + (x187 * x155);
	const GEN_FLT x203 = ((x191 + x195 + x193) * sensor_x) + (((x21 * x194) + x188 + (x22 * x187)) * sensor_y) +
						 (((-1 * x200) + (-1 * x201) + x202) * sensor_z);
	const GEN_FLT x204 = (((-1 * x196) + (-1 * x197) + x198) * sensor_x) + ((x200 + x201 + x202) * sensor_y) +
						 ((x188 + (x56 * x190) + (x57 * x187)) * sensor_z);
	const GEN_FLT x205 = (x45 * x199) + (x40 * x203) + (x55 * x204) + x82;
	const GEN_FLT x206 = (x89 * x199) + (x87 * x203) + (x85 * x204) + x100;
	const GEN_FLT x207 = (x106 * x199) + (x203 * x108) + (x204 * x110) + x111;
	const GEN_FLT x208 = (-1 * x118 * ((x95 * x207) + (-1 * x205 * x115))) +
						 (-1 * x123 * ((-1 * ((x207 * x119) + (x205 * x120)) * x121) + (x206 * x122)));
	out[0] = (x105 * ((x84 * x94) + (-1 * x95 * x102))) + (x124 * x125) + x124;
	out[1] = (((x94 * x128) + (-1 * x95 * x130)) * x105) + (x125 * x133) + x133;
	out[2] = (((x94 * x135) + (-1 * x95 * x136)) * x105) + (x125 * x138) + x138;
	out[3] = (((x94 * x159) + (-1 * x95 * x160)) * x105) + (x125 * x162) + x162;
	out[4] = (((x94 * x182) + (-1 * x95 * x183)) * x105) + (x125 * x185) + x185;
	out[5] = (((x94 * x205) + (-1 * x95 * x206)) * x105) + (x208 * x125) + x208;
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x5 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (x15 * x12) + x14;
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x18 = sin(x13);
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x21 * x11;
	const GEN_FLT x23 = x19 + x22;
	const GEN_FLT x24 = x20 * x18;
	const GEN_FLT x25 = x15 * x11;
	const GEN_FLT x26 = x25 * x17;
	const GEN_FLT x27 = (-1 * x24) + x26;
	const GEN_FLT x28 = obj_pz + (x27 * sensor_x) + (x16 * sensor_z) + (x23 * sensor_y);
	const GEN_FLT x29 = x1 * x4;
	const GEN_FLT x30 = x2 * x8;
	const GEN_FLT x31 = (-1 * x29) + x30;
	const GEN_FLT x32 = (-1 * x19) + x22;
	const GEN_FLT x33 = x20 * x20;
	const GEN_FLT x34 = (x33 * x15) + x14;
	const GEN_FLT x35 = x11 * x18;
	const GEN_FLT x36 = x21 * x17;
	const GEN_FLT x37 = x35 + x36;
	const GEN_FLT x38 = obj_py + (x32 * sensor_z) + (x34 * sensor_y) + (x37 * sensor_x);
	const GEN_FLT x39 = x5 * x5;
	const GEN_FLT x40 = (x7 * x39) + x6;
	const GEN_FLT x41 = x24 + x26;
	const GEN_FLT x42 = (-1 * x35) + x36;
	const GEN_FLT x43 = x17 * x17;
	const GEN_FLT x44 = (x43 * x15) + x14;
	const GEN_FLT x45 = obj_px + (x42 * sensor_y) + (x41 * sensor_z) + (x44 * sensor_x);
	const GEN_FLT x46 = lh_px + (x28 * x10) + (x40 * x45) + (x31 * x38);
	const GEN_FLT x47 = x4 * x4;
	const GEN_FLT x48 = (x7 * x47) + x6;
	const GEN_FLT x49 = x1 * x5;
	const GEN_FLT x50 = x2 * x7;
	const GEN_FLT x51 = x4 * x50;
	const GEN_FLT x52 = x49 + x51;
	const GEN_FLT x53 = (-1 * x3) + x9;
	const GEN_FLT x54 = lh_pz + (x48 * x28) + (x52 * x38) + (x53 * x45);
	const GEN_FLT x55 = x54 * x54;
	const GEN_FLT x56 = pow(x55, -1);
	const GEN_FLT x57 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x58 = x15 * x17;
	const GEN_FLT x59 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x60 = x59 * x18;
	const GEN_FLT x61 = -1 * x60;
	const GEN_FLT x62 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x63 = x62 * x18;
	const GEN_FLT x64 = x59 * x14;
	const GEN_FLT x65 = x64 * x11;
	const GEN_FLT x66 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x67 = x59 * x20;
	const GEN_FLT x68 = (x66 * x58) + (x67 * x19) + (x57 * x21);
	const GEN_FLT x69 = x66 * x18;
	const GEN_FLT x70 = x64 * x20;
	const GEN_FLT x71 = (x57 * x25) + (x59 * x11 * x19) + (x62 * x58);
	const GEN_FLT x72 = (((2 * x58 * x57) + x61 + (x60 * x43)) * sensor_x) +
						(((-1 * x63) + (-1 * x65) + x68) * sensor_y) + ((x69 + x70 + x71) * sensor_z);
	const GEN_FLT x73 = x44 + x72;
	const GEN_FLT x74 = x57 * x18;
	const GEN_FLT x75 = x64 * x17;
	const GEN_FLT x76 = (x66 * x25) + (x62 * x21) + (x67 * x35);
	const GEN_FLT x77 = ((x63 + x65 + x68) * sensor_x) + ((x61 + (2 * x66 * x21) + (x60 * x33)) * sensor_y) +
						(((-1 * x74) + (-1 * x75) + x76) * sensor_z);
	const GEN_FLT x78 = x37 + x77;
	const GEN_FLT x79 = (((-1 * x69) + (-1 * x70) + x71) * sensor_x) + ((x74 + x75 + x76) * sensor_y) +
						((x61 + (2 * x62 * x25) + (x60 * x12)) * sensor_z);
	const GEN_FLT x80 = x27 + x79;
	const GEN_FLT x81 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x82 = x6 * x81;
	const GEN_FLT x83 = x2 * x82;
	const GEN_FLT x84 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x85 = x1 * x84;
	const GEN_FLT x86 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x87 = x7 * x86;
	const GEN_FLT x88 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x89 = (x4 * x81 * x49) + (x4 * x87) + (x8 * x88);
	const GEN_FLT x90 = x5 * x82;
	const GEN_FLT x91 = x1 * x86;
	const GEN_FLT x92 = x2 * x81;
	const GEN_FLT x93 = x4 * x7;
	const GEN_FLT x94 = (x92 * x29) + (x88 * x50) + (x84 * x93);
	const GEN_FLT x95 = x1 * x81;
	const GEN_FLT x96 = -1 * x95;
	const GEN_FLT x97 = (x45 * ((-1 * x83) + x89 + (-1 * x85))) + (x38 * (x90 + x91 + x94)) +
						(x28 * (x96 + (x95 * x47) + (2 * x88 * x93)));
	const GEN_FLT x98 = (x73 * x53) + (x78 * x52) + (x80 * x48) + x97;
	const GEN_FLT x99 = x56 * x98;
	const GEN_FLT x100 = pow(x54, -1);
	const GEN_FLT x101 = x4 * x82;
	const GEN_FLT x102 = x1 * x88;
	const GEN_FLT x103 = (x2 * x87) + (x8 * x84) + (x92 * x49);
	const GEN_FLT x104 = (x45 * (x96 + (x95 * x39) + (2 * x5 * x87))) + (x38 * ((-1 * x101) + (-1 * x102) + x103)) +
						 (x28 * (x83 + x89 + x85));
	const GEN_FLT x105 = (x73 * x40) + (x78 * x31) + (x80 * x10) + x104;
	const GEN_FLT x106 = x46 * x46;
	const GEN_FLT x107 = -1 * x54;
	const GEN_FLT x108 = 2 * x55 * atan2(x46, x107) * pow((x106 + x55), -1) * curve_1;
	const GEN_FLT x109 = x29 + x30;
	const GEN_FLT x110 = x2 * x2;
	const GEN_FLT x111 = (x7 * x110) + x6;
	const GEN_FLT x112 = (-1 * x49) + x51;
	const GEN_FLT x113 = (x45 * (x101 + x102 + x103)) + (x38 * (x96 + (2 * x84 * x50) + (x95 * x110))) +
						 (x28 * ((-1 * x90) + (-1 * x91) + x94));
	const GEN_FLT x114 = (x73 * x109) + (x78 * x111) + (x80 * x112) + x113;
	const GEN_FLT x115 = lh_py + (x28 * x112) + (x38 * x111) + (x45 * x109);
	const GEN_FLT x116 = (x115 * x115) + x55;
	const GEN_FLT x117 = pow(x116, -1);
	const GEN_FLT x118 = x55 * x117;
	const GEN_FLT x119 = 2 * x115;
	const GEN_FLT x120 = 2 * x54;
	const GEN_FLT x121 = 1.0 / 2.0 * x46 * pow(x116, -3.0 / 2.0) * tilt_1;
	const GEN_FLT x122 = pow(x116, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x123 = pow((1 + (-1 * x106 * x117 * (tilt_1 * tilt_1))), -1.0 / 2.0);
	const GEN_FLT x124 = (-1 * x118 * ((x100 * x114) + (-1 * x99 * x115))) +
						 (-1 * x123 * ((-1 * x121 * ((x119 * x114) + (x98 * x120))) + (x105 * x122)));
	const GEN_FLT x125 =
		sin(1.5707963267949 + gibPhase_1 + (-1 * asin(x46 * x122)) + (-1 * atan2(-1 * x115, x107)) + (-1 * phase_1)) *
		gibMag_1;
	const GEN_FLT x126 = x42 + x72;
	const GEN_FLT x127 = x34 + x77;
	const GEN_FLT x128 = x23 + x79;
	const GEN_FLT x129 = (x53 * x126) + (x52 * x127) + (x48 * x128) + x97;
	const GEN_FLT x130 = x56 * x129;
	const GEN_FLT x131 = (x40 * x126) + (x31 * x127) + (x10 * x128) + x104;
	const GEN_FLT x132 = (x109 * x126) + (x111 * x127) + (x112 * x128) + x113;
	const GEN_FLT x133 = (-1 * ((x100 * x132) + (-1 * x115 * x130)) * x118) +
						 (-1 * x123 * ((-1 * ((x119 * x132) + (x120 * x129)) * x121) + (x122 * x131)));
	const GEN_FLT x134 = x41 + x72;
	const GEN_FLT x135 = x32 + x77;
	const GEN_FLT x136 = x16 + x79;
	const GEN_FLT x137 = (x53 * x134) + (x48 * x136) + (x52 * x135) + x97;
	const GEN_FLT x138 = x56 * x137;
	const GEN_FLT x139 = (x40 * x134) + (x31 * x135) + (x10 * x136) + x104;
	const GEN_FLT x140 = (x111 * x135) + x113 + (x109 * x134) + (x112 * x136);
	const GEN_FLT x141 = (-1 * ((x100 * x140) + (-1 * x115 * x138)) * x118) +
						 (-1 * x123 * ((-1 * ((x119 * x140) + (x120 * x137)) * x121) + (x122 * x139)));
	out[0] = (x108 * ((x99 * x46) + (-1 * x100 * x105))) + (x124 * x125) + x124;
	out[1] = (x108 * ((x46 * x130) + (-1 * x100 * x131))) + (x125 * x133) + x133;
	out[2] = (x108 * ((x46 * x138) + (-1 * x100 * x139))) + (x125 * x141) + x141;
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
	const GEN_FLT x0 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 0;
	const GEN_FLT x1 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   : 1e-10;
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 1;
	const GEN_FLT x5 = cos(x1);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 : 1e-10))
						   ? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												: 1e-10),
										   -1))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x2 * x7;
	const GEN_FLT x11 = x0 * x6;
	const GEN_FLT x12 = x4 * x11;
	const GEN_FLT x13 = x4 * x4;
	const GEN_FLT x14 =
		obj_px + ((x3 + x9) * sensor_z) + (((-1 * x10) + x12) * sensor_y) + (((x6 * x13) + x5) * sensor_x);
	const GEN_FLT x15 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 0;
	const GEN_FLT x16 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x17 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							: 1e-10;
	const GEN_FLT x18 = cos(x17);
	const GEN_FLT x19 = x18 * x16;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = sin(x17);
	const GEN_FLT x22 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 1;
	const GEN_FLT x25 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												: 1e-10),
										   -1))
							: 0;
	const GEN_FLT x26 = x25 * x21;
	const GEN_FLT x27 = x24 * x26;
	const GEN_FLT x28 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x29 = 1 + (-1 * x18);
	const GEN_FLT x30 = x25 * x29;
	const GEN_FLT x31 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x32 = x24 * x29;
	const GEN_FLT x33 = (x27 * x16) + (x30 * x28) + (x32 * x31);
	const GEN_FLT x34 = x24 * x18;
	const GEN_FLT x35 = x34 * x16;
	const GEN_FLT x36 = x21 * x28;
	const GEN_FLT x37 = x26 * x15;
	const GEN_FLT x38 = x29 * x15;
	const GEN_FLT x39 = (x37 * x16) + (x31 * x38) + (x30 * x22);
	const GEN_FLT x40 = x2 * x4;
	const GEN_FLT x41 = x7 * x11;
	const GEN_FLT x42 = x0 * x0;
	const GEN_FLT x43 =
		obj_py + (((-1 * x40) + x41) * sensor_z) + (((x6 * x42) + x5) * sensor_y) + ((x10 + x12) * sensor_x);
	const GEN_FLT x44 = x7 * x7;
	const GEN_FLT x45 =
		obj_pz + (((x6 * x44) + x5) * sensor_z) + ((x40 + x41) * sensor_y) + (((-1 * x3) + x9) * sensor_x);
	const GEN_FLT x46 = x21 * x16;
	const GEN_FLT x47 = -1 * x46;
	const GEN_FLT x48 = x25 * x25;
	const GEN_FLT x49 = 2 * x30;
	const GEN_FLT x50 = x21 * x15;
	const GEN_FLT x51 = x32 * x25;
	const GEN_FLT x52 = (-1 * x50) + x51;
	const GEN_FLT x53 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x54 = x4 * x6;
	const GEN_FLT x55 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x56 = x2 * x55;
	const GEN_FLT x57 = -1 * x56;
	const GEN_FLT x58 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x59 = x2 * x58;
	const GEN_FLT x60 = x5 * x55;
	const GEN_FLT x61 = x7 * x60;
	const GEN_FLT x62 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x63 = x55 * x40;
	const GEN_FLT x64 = (x62 * x54) + (x0 * x63) + (x53 * x11);
	const GEN_FLT x65 = x2 * x62;
	const GEN_FLT x66 = x0 * x60;
	const GEN_FLT x67 = (x8 * x53) + (x7 * x63) + (x54 * x58);
	const GEN_FLT x68 = (((2 * x54 * x53) + x57 + (x56 * x13)) * sensor_x) +
						(((-1 * x59) + (-1 * x61) + x64) * sensor_y) + ((x65 + x66 + x67) * sensor_z);
	const GEN_FLT x69 = x2 * x53;
	const GEN_FLT x70 = x4 * x60;
	const GEN_FLT x71 = (x8 * x62) + (x58 * x11) + (x0 * x7 * x56);
	const GEN_FLT x72 = ((x59 + x61 + x64) * sensor_x) + ((x57 + (2 * x62 * x11) + (x56 * x42)) * sensor_y) +
						(((-1 * x69) + (-1 * x70) + x71) * sensor_z);
	const GEN_FLT x73 = x24 * x21;
	const GEN_FLT x74 = x38 * x25;
	const GEN_FLT x75 = x73 + x74;
	const GEN_FLT x76 = (x48 * x29) + x18;
	const GEN_FLT x77 = (((-1 * x65) + (-1 * x66) + x67) * sensor_x) + ((x69 + x70 + x71) * sensor_y) +
						((x57 + (2 * x8 * x58) + (x56 * x44)) * sensor_z);
	const GEN_FLT x78 = (x68 * x52) + (x72 * x75) + (x77 * x76);
	const GEN_FLT x79 = (x14 * ((-1 * x20) + (-1 * x23) + x33)) + (x43 * (x35 + x36 + x39)) +
						(x45 * (x47 + (x49 * x31) + (x46 * x48))) + x78;
	const GEN_FLT x80 = lh_pz + (x76 * x45) + (x75 * x43) + (x52 * x14);
	const GEN_FLT x81 = x80 * x80;
	const GEN_FLT x82 = pow(x81, -1);
	const GEN_FLT x83 = x50 + x51;
	const GEN_FLT x84 = x32 * x15;
	const GEN_FLT x85 = (-1 * x26) + x84;
	const GEN_FLT x86 = x24 * x24;
	const GEN_FLT x87 = (x86 * x29) + x18;
	const GEN_FLT x88 = lh_px + (x85 * x43) + (x83 * x45) + (x87 * x14);
	const GEN_FLT x89 = x82 * x88;
	const GEN_FLT x90 = x89 * x79;
	const GEN_FLT x91 = pow(x80, -1);
	const GEN_FLT x92 = 2 * x32;
	const GEN_FLT x93 = x25 * x19;
	const GEN_FLT x94 = x31 * x21;
	const GEN_FLT x95 = x73 * x15;
	const GEN_FLT x96 = (x38 * x28) + (x32 * x22) + (x95 * x16);
	const GEN_FLT x97 = (x87 * x68) + (x85 * x72) + (x83 * x77);
	const GEN_FLT x98 = (x14 * (x47 + (x92 * x28) + (x86 * x46))) + (x43 * ((-1 * x93) + (-1 * x94) + x96)) +
						(x45 * (x20 + x23 + x33)) + x97;
	const GEN_FLT x99 = 1 + x98;
	const GEN_FLT x100 = x88 * x88;
	const GEN_FLT x101 = -1 * x80;
	const GEN_FLT x102 = 2 * x81 * atan2(x88, x101) * pow((x100 + x81), -1) * curve_1;
	const GEN_FLT x103 = x15 * x15;
	const GEN_FLT x104 = 2 * x38;
	const GEN_FLT x105 = x26 + x84;
	const GEN_FLT x106 = (x29 * x103) + x18;
	const GEN_FLT x107 = (-1 * x73) + x74;
	const GEN_FLT x108 = (x68 * x105) + (x72 * x106) + (x77 * x107);
	const GEN_FLT x109 = (x14 * (x93 + x94 + x96)) + (x45 * ((-1 * x35) + (-1 * x36) + x39)) +
						 (x43 * (x47 + (x46 * x103) + (x22 * x104))) + x108;
	const GEN_FLT x110 = x91 * x109;
	const GEN_FLT x111 = lh_py + (x45 * x107) + (x43 * x106) + (x14 * x105);
	const GEN_FLT x112 = x82 * x111;
	const GEN_FLT x113 = -1 * x79 * x112;
	const GEN_FLT x114 = (x111 * x111) + x81;
	const GEN_FLT x115 = pow(x114, -1);
	const GEN_FLT x116 = x81 * x115;
	const GEN_FLT x117 = 2 * x111;
	const GEN_FLT x118 = x109 * x117;
	const GEN_FLT x119 = 2 * x80;
	const GEN_FLT x120 = x79 * x119;
	const GEN_FLT x121 = 1.0 / 2.0 * x88 * pow(x114, -3.0 / 2.0) * tilt_1;
	const GEN_FLT x122 = pow(x114, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x123 = pow((1 + (-1 * x100 * x115 * (tilt_1 * tilt_1))), -1.0 / 2.0);
	const GEN_FLT x124 = (-1 * (x110 + x113) * x116) + (-1 * x123 * ((-1 * (x118 + x120) * x121) + (x99 * x122)));
	const GEN_FLT x125 =
		sin(1.5707963267949 + gibPhase_1 + (-1 * asin(x88 * x122)) + (-1 * phase_1) + (-1 * atan2(-1 * x111, x101))) *
		gibMag_1;
	const GEN_FLT x126 = -1 * x91 * x98;
	const GEN_FLT x127 = 1 + x109;
	const GEN_FLT x128 = x98 * x122;
	const GEN_FLT x129 =
		(-1 * x116 * ((x91 * x127) + x113)) + (-1 * x123 * ((-1 * x121 * ((x117 * x127) + x120)) + x128));
	const GEN_FLT x130 = 1 + x79;
	const GEN_FLT x131 =
		(-1 * x116 * (x110 + (-1 * x112 * x130))) + (-1 * x123 * ((-1 * x121 * (x118 + (x119 * x130))) + x128));
	const GEN_FLT x132 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x133 = x18 * x132;
	const GEN_FLT x134 = x15 * x133;
	const GEN_FLT x135 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qj *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x136 = x21 * x135;
	const GEN_FLT x137 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qi *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x138 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qk *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x139 = (x30 * x137) + (x32 * x138) + (x27 * x132);
	const GEN_FLT x140 = x34 * x132;
	const GEN_FLT x141 = x21 * x137;
	const GEN_FLT x142 = (x37 * x132) + (x38 * x138) + (x30 * x135);
	const GEN_FLT x143 = x21 * x132;
	const GEN_FLT x144 = -1 * x143;
	const GEN_FLT x145 = (x14 * ((-1 * x134) + (-1 * x136) + x139)) + x78 + (x43 * (x140 + x141 + x142)) +
						 (x45 * (x144 + (x48 * x143) + (x49 * x138)));
	const GEN_FLT x146 = x25 * x133;
	const GEN_FLT x147 = x21 * x138;
	const GEN_FLT x148 = (x95 * x132) + (x38 * x137) + (x32 * x135);
	const GEN_FLT x149 = (x14 * (x144 + (x86 * x143) + (x92 * x137))) + (x43 * ((-1 * x146) + (-1 * x147) + x148)) +
						 (x45 * (x134 + x136 + x139)) + x97;
	const GEN_FLT x150 = (x43 * (x144 + (x103 * x143) + (x104 * x135))) + (x45 * ((-1 * x140) + (-1 * x141) + x142)) +
						 (x14 * (x146 + x147 + x148)) + x108;
	const GEN_FLT x151 = (-1 * x116 * ((x91 * x150) + (-1 * x112 * x145))) +
						 (-1 * x123 * ((-1 * ((x117 * x150) + (x119 * x145)) * x121) + (x122 * x149)));
	const GEN_FLT x152 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x153 = x15 * x18;
	const GEN_FLT x154 = x152 * x153;
	const GEN_FLT x155 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qj *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x156 = x21 * x155;
	const GEN_FLT x157 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qi *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x158 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qk *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x159 = (x27 * x152) + (x30 * x157) + (x32 * x158);
	const GEN_FLT x160 = x34 * x152;
	const GEN_FLT x161 = x21 * x157;
	const GEN_FLT x162 = x15 * x152;
	const GEN_FLT x163 = (x26 * x162) + (x38 * x158) + (x30 * x155);
	const GEN_FLT x164 = x21 * x152;
	const GEN_FLT x165 = -1 * x164;
	const GEN_FLT x166 = (x14 * ((-1 * x154) + (-1 * x156) + x159)) + (x43 * (x160 + x161 + x163)) +
						 (x45 * (x165 + (x48 * x164) + (x49 * x158))) + x78;
	const GEN_FLT x167 = x25 * x18;
	const GEN_FLT x168 = x167 * x152;
	const GEN_FLT x169 = x21 * x158;
	const GEN_FLT x170 = (x73 * x162) + (x32 * x155) + (x38 * x157);
	const GEN_FLT x171 = (x14 * ((x86 * x164) + x165 + (x92 * x157))) + (x43 * ((-1 * x168) + (-1 * x169) + x170)) +
						 (x45 * (x154 + x156 + x159)) + x97;
	const GEN_FLT x172 = (x14 * (x168 + x169 + x170)) + x108 + (x43 * (x165 + (x103 * x164) + (x104 * x155))) +
						 (x45 * ((-1 * x160) + (-1 * x161) + x163));
	const GEN_FLT x173 = (-1 * x116 * ((x91 * x172) + (-1 * x112 * x166))) +
						 (-1 * x123 * ((-1 * ((x117 * x172) + (x119 * x166)) * x121) + (x122 * x171)));
	const GEN_FLT x174 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
							 : 0;
	const GEN_FLT x175 = x174 * x153;
	const GEN_FLT x176 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qj *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x177 = x21 * x176;
	const GEN_FLT x178 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? (-1 * lh_qi *
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-2) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									 : 0))
							 : 0;
	const GEN_FLT x179 = x29 * x178;
	const GEN_FLT x180 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   : 1e-10))
							 ? ((-1 * lh_qk *
								 pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										  : 1e-10),
									 -2) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qk * pow(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)), -1.0 / 2.0))
									  : 0)) +
								pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										 : 1e-10),
									-1))
							 : 0;
	const GEN_FLT x181 = (x27 * x174) + (x25 * x179) + (x32 * x180);
	const GEN_FLT x182 = x34 * x174;
	const GEN_FLT x183 = x21 * x178;
	const GEN_FLT x184 = (x37 * x174) + (x38 * x180) + (x30 * x176);
	const GEN_FLT x185 = x21 * x174;
	const GEN_FLT x186 = -1 * x185;
	const GEN_FLT x187 = (x14 * ((-1 * x175) + (-1 * x177) + x181)) + (x43 * (x182 + x183 + x184)) +
						 (x45 * (x186 + (x48 * x185) + (x49 * x180))) + x78;
	const GEN_FLT x188 = x167 * x174;
	const GEN_FLT x189 = x21 * x180;
	const GEN_FLT x190 = (x95 * x174) + (x15 * x179) + (x32 * x176);
	const GEN_FLT x191 = (x14 * (x186 + (x86 * x185) + (x92 * x178))) + x97 +
						 (x43 * ((-1 * x188) + (-1 * x189) + x190)) + (x45 * (x175 + x177 + x181));
	const GEN_FLT x192 = (x14 * (x188 + x189 + x190)) + (x43 * (x186 + (x103 * x185) + (x104 * x176))) +
						 (x45 * ((-1 * x182) + (-1 * x183) + x184)) + x108;
	const GEN_FLT x193 = (-1 * x116 * ((x91 * x192) + (-1 * x112 * x187))) +
						 (-1 * x123 * ((-1 * ((x117 * x192) + (x119 * x187)) * x121) + (x122 * x191)));
	out[0] = (x102 * (x90 + (-1 * x91 * x99))) + (x124 * x125) + x124;
	out[1] = (x102 * (x90 + x126)) + (x125 * x129) + x129;
	out[2] = (x102 * ((x89 * x130) + x126)) + (x125 * x131) + x131;
	out[3] = (((x89 * x145) + (-1 * x91 * x149)) * x102) + (x125 * x151) + x151;
	out[4] = (((x89 * x166) + (-1 * x91 * x171)) * x102) + (x125 * x173) + x173;
	out[5] = (((x89 * x187) + (-1 * x91 * x191)) * x102) + (x125 * x193) + x193;
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
	const GEN_FLT x0 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qj * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qk * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 0;
	const GEN_FLT x5 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 : 1e-10))
						   ? (lh_qi * pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											   : 1e-10),
										  -1))
						   : 1;
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x3 + x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qk * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qi * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (obj_qj * pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												 : 1e-10),
											-1))
							: 0;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x20 * x11;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x15 * x16;
	const GEN_FLT x24 = x23 * x11;
	const GEN_FLT x25 =
		obj_pz + (((x15 * x12) + x14) * sensor_z) + ((x18 + x21) * sensor_y) + (((-1 * x22) + x24) * sensor_x);
	const GEN_FLT x26 = x1 * x4;
	const GEN_FLT x27 = x2 * x7;
	const GEN_FLT x28 = x5 * x27;
	const GEN_FLT x29 = (-1 * x26) + x28;
	const GEN_FLT x30 = x19 * x19;
	const GEN_FLT x31 = x11 * x17;
	const GEN_FLT x32 = x20 * x16;
	const GEN_FLT x33 =
		obj_py + (((-1 * x18) + x21) * sensor_z) + ((x31 + x32) * sensor_x) + (((x30 * x15) + x14) * sensor_y);
	const GEN_FLT x34 = x5 * x5;
	const GEN_FLT x35 = (x7 * x34) + x6;
	const GEN_FLT x36 = x16 * x16;
	const GEN_FLT x37 =
		obj_px + (((x36 * x15) + x14) * sensor_x) + ((x22 + x24) * sensor_z) + (((-1 * x31) + x32) * sensor_y);
	const GEN_FLT x38 = lh_px + (x25 * x10) + (x33 * x29) + (x35 * x37);
	const GEN_FLT x39 = x4 * x4;
	const GEN_FLT x40 = (x7 * x39) + x6;
	const GEN_FLT x41 = x1 * x5;
	const GEN_FLT x42 = x4 * x27;
	const GEN_FLT x43 = x41 + x42;
	const GEN_FLT x44 = (-1 * x3) + x9;
	const GEN_FLT x45 = lh_pz + (x40 * x25) + (x43 * x33) + (x44 * x37);
	const GEN_FLT x46 = x45 * x45;
	const GEN_FLT x47 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x48 = x6 * x47;
	const GEN_FLT x49 = x2 * x48;
	const GEN_FLT x50 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qj *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x51 = x1 * x50;
	const GEN_FLT x52 = x41 * x47;
	const GEN_FLT x53 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qi *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x54 = x4 * x7;
	const GEN_FLT x55 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  : 1e-10))
							? (-1 * lh_qk *
							   pow(((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										? sqrt(((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x56 = (x4 * x52) + (x54 * x53) + (x8 * x55);
	const GEN_FLT x57 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qi *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x58 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x59 = x58 * x17;
	const GEN_FLT x60 = -1 * x59;
	const GEN_FLT x61 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qk *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x62 = x61 * x17;
	const GEN_FLT x63 = x58 * x14;
	const GEN_FLT x64 = x63 * x11;
	const GEN_FLT x65 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  : 1e-10))
							? (-1 * obj_qj *
							   pow(((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										? sqrt(((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										: 1e-10),
								   -2) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x66 = x58 * x19;
	const GEN_FLT x67 = (x65 * x23) + (x66 * x18) + (x57 * x20);
	const GEN_FLT x68 = x65 * x17;
	const GEN_FLT x69 = x63 * x19;
	const GEN_FLT x70 = x15 * x11;
	const GEN_FLT x71 = (x70 * x57) + (x58 * x11 * x18) + (x61 * x23);
	const GEN_FLT x72 = (((2 * x57 * x23) + (x59 * x36) + x60) * sensor_x) +
						(((-1 * x62) + (-1 * x64) + x67) * sensor_y) + ((x68 + x69 + x71) * sensor_z);
	const GEN_FLT x73 = x5 * x48;
	const GEN_FLT x74 = x1 * x53;
	const GEN_FLT x75 = (x2 * x47 * x26) + (x55 * x27) + (x50 * x54);
	const GEN_FLT x76 = x57 * x17;
	const GEN_FLT x77 = x63 * x16;
	const GEN_FLT x78 = (x70 * x65) + (x61 * x20) + (x66 * x31);
	const GEN_FLT x79 = ((x62 + x64 + x67) * sensor_x) + ((x60 + (2 * x65 * x20) + (x59 * x30)) * sensor_y) +
						(((-1 * x76) + (-1 * x77) + x78) * sensor_z);
	const GEN_FLT x80 = x1 * x47;
	const GEN_FLT x81 = -1 * x80;
	const GEN_FLT x82 = (((-1 * x68) + (-1 * x69) + x71) * sensor_x) + ((x76 + x77 + x78) * sensor_y) +
						((x60 + (2 * x70 * x61) + (x59 * x12)) * sensor_z);
	const GEN_FLT x83 = (x37 * ((-1 * x49) + (-1 * x51) + x56)) + (x72 * x44) + (x79 * x43) +
						(x33 * (x73 + x74 + x75)) + (x25 * (x81 + (x80 * x39) + (2 * x54 * x55))) + (x82 * x40);
	const GEN_FLT x84 = x83 * pow(x46, -1);
	const GEN_FLT x85 = pow(x45, -1);
	const GEN_FLT x86 = x4 * x48;
	const GEN_FLT x87 = x1 * x55;
	const GEN_FLT x88 = (x53 * x27) + (x8 * x50) + (x2 * x52);
	const GEN_FLT x89 = (x37 * (x81 + (x80 * x34) + (2 * x8 * x53))) + (x72 * x35) + (x25 * (x49 + x51 + x56)) +
						(x33 * ((-1 * x86) + (-1 * x87) + x88)) + (x79 * x29) + (x82 * x10);
	const GEN_FLT x90 = x38 * x38;
	const GEN_FLT x91 = -1 * x45;
	const GEN_FLT x92 = atan2(x38, x91);
	const GEN_FLT x93 = 2 * pow((x90 + x46), -1) * ((x84 * x38) + (-1 * x89 * x85)) * x92 * x46 * curve_1;
	const GEN_FLT x94 = (-1 * x41) + x42;
	const GEN_FLT x95 = x2 * x2;
	const GEN_FLT x96 = (x7 * x95) + x6;
	const GEN_FLT x97 = x26 + x28;
	const GEN_FLT x98 = lh_py + (x94 * x25) + (x96 * x33) + (x97 * x37);
	const GEN_FLT x99 = (x98 * x98) + x46;
	const GEN_FLT x100 = pow(x99, -1);
	const GEN_FLT x101 = (x37 * (x86 + x87 + x88)) + (x72 * x97) + (x82 * x94) +
						 (x33 * (x81 + (x80 * x95) + (2 * x50 * x27))) + (x79 * x96) +
						 (x25 * ((-1 * x73) + (-1 * x74) + x75));
	const GEN_FLT x102 = -1 * x46 * x100 * ((x85 * x101) + (-1 * x84 * x98));
	const GEN_FLT x103 = pow(x99, -1.0 / 2.0);
	const GEN_FLT x104 = (-1.0 / 2.0 * pow(x99, -3.0 / 2.0) * x38 * tilt_1 * ((2 * x98 * x101) + (2 * x83 * x45))) +
						 (x89 * x103 * tilt_1);
	const GEN_FLT x105 = pow((1 + (-1 * x90 * x100 * (tilt_1 * tilt_1))), -1.0 / 2.0);
	const GEN_FLT x106 = x102 + (-1 * x105 * x104);
	const GEN_FLT x107 = -1 + x106;
	const GEN_FLT x108 = x38 * x103;
	const GEN_FLT x109 =
		1.5707963267949 + gibPhase_1 + (-1 * asin(x108 * tilt_1)) + (-1 * phase_1) + (-1 * atan2(-1 * x98, x91));
	const GEN_FLT x110 = sin(x109) * gibMag_1;
	const GEN_FLT x111 = x102 + (-1 * (x108 + x104) * x105);
	const GEN_FLT x112 = x93 + x106;
	const GEN_FLT x113 = (x106 * x110) + x112;
	out[0] = x93 + (x107 * x110) + x107;
	out[1] = x93 + (x110 * x111) + x111;
	out[2] = (x92 * x92) + x113;
	out[3] = (x110 * (1 + x106)) + x112;
	out[4] = (-1 * cos(x109)) + x113;
	out[5] = x113;
	out[6] = x113;
}

/** Applying function <function reproject_gen2 at 0x7f35a4af8a70> */
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
	const GEN_FLT x4 = obj_qj * obj_qj;
	const GEN_FLT x5 = obj_qi * obj_qi;
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = 2 * sqrt((x7 + (obj_qw * obj_qw) + x6));
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = obj_pz + ((1 + (-1 * x6 * x8)) * sensor_z) + (x11 * (x9 + x10)) + (((-1 * x12) + x13) * x14);
	const GEN_FLT x16 = lh_qi * lh_qi;
	const GEN_FLT x17 = lh_qk * lh_qk;
	const GEN_FLT x18 = lh_qj * lh_qj;
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 2 * sqrt((x16 + (lh_qw * lh_qw) + x19));
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 =
		(x22 * ((-1 * x9) + x10)) + ((x23 + x24) * x14) + obj_py + ((1 + (-1 * (x7 + x5) * x8)) * sensor_y);
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 =
		obj_px + ((x12 + x13) * x22) + (((-1 * x23) + x24) * x11) + ((1 + (-1 * (x7 + x4) * x8)) * sensor_x);
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = lh_py + (((-1 * x2) + x3) * x21) + ((x26 + x27) * x29) + (x25 * (1 + (-1 * (x17 + x16) * x20)));
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = lh_pz + (x15 * (1 + (-1 * (x18 + x16) * x20))) + ((x2 + x3) * x31) + (((-1 * x32) + x33) * x29);
	const GEN_FLT x35 = lh_px + ((x32 + x33) * x21) + (((-1 * x26) + x27) * x31) + (x28 * (1 + (-1 * x20 * x19)));
	const GEN_FLT x36 = (x34 * x34) + (x35 * x35);
	const GEN_FLT x37 = x30 * pow(((x30 * x30) + x36), -1.0 / 2.0);
	const GEN_FLT x38 = asin(pow(x1, -1) * x37);
	const GEN_FLT x39 = 0.0028679863 + (x38 * (-8.0108022e-06 + (-8.0108022e-06 * x38)));
	const GEN_FLT x40 = 5.3685255e-06 + (x38 * x39);
	const GEN_FLT x41 = 0.0076069798 + (x40 * x38);
	const GEN_FLT x42 = x30 * pow(x36, -1.0 / 2.0);
	const GEN_FLT x43 = tan(x0) * x42;
	const GEN_FLT x44 = atan2(-1 * x34, x35);
	const GEN_FLT x45 = curve_0 + (sin(ogeeMag_0 + (-1 * asin(x43)) + x44) * ogeePhase_0);
	const GEN_FLT x46 = asin(
		(x41 * x45 * (x38 * x38) *
		 pow(((-1 * sin(x0) * x45 *
			   ((x41 * x38) +
				(x38 * ((x38 * ((x38 * ((x38 * (-8.0108022e-06 + (-1.60216044e-05 * x38))) + x39)) + x40)) + x41)))) +
			  x1),
			 -1)) +
		x43);
	const GEN_FLT x47 = -1 * x44;
	const GEN_FLT x48 = -1.5707963267949 + x44;
	const GEN_FLT x49 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x50 = -1 * x42 * tan(x49);
	const GEN_FLT x51 = curve_1 + (sin(ogeeMag_1 + (-1 * asin(x50)) + x44) * ogeePhase_1);
	const GEN_FLT x52 = cos(x49);
	const GEN_FLT x53 = asin(pow(x52, -1) * x37);
	const GEN_FLT x54 = 0.0028679863 + (x53 * (-8.0108022e-06 + (-8.0108022e-06 * x53)));
	const GEN_FLT x55 = 5.3685255e-06 + (x54 * x53);
	const GEN_FLT x56 = 0.0076069798 + (x53 * x55);
	const GEN_FLT x57 = asin(
		((x53 * x53) * x51 * x56 *
		 pow(((x51 * sin(x49) *
			   ((x53 * x56) +
				(x53 * ((x53 * ((x53 * ((x53 * (-8.0108022e-06 + (-1.60216044e-05 * x53))) + x54)) + x55)) + x56)))) +
			  x52),
			 -1)) +
		x50);
	out[0] = (-1 * phase_0) + (-1 * sin((-1 * gibPhase_0) + x46 + x47) * gibMag_0) + (-1 * x46) + x48;
	out[1] = (-1 * phase_1) + (-1 * sin((-1 * gibPhase_1) + x47 + x57) * gibMag_1) + (-1 * x57) + x48;
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
	const GEN_FLT x2 = (-1 * x0) + x1;
	const GEN_FLT x3 = obj_qj * obj_qj;
	const GEN_FLT x4 = obj_qi * obj_qi;
	const GEN_FLT x5 = x3 + x4;
	const GEN_FLT x6 = obj_qk * obj_qk;
	const GEN_FLT x7 = x6 + x4;
	const GEN_FLT x8 = sqrt((x3 + (obj_qw * obj_qw) + x7));
	const GEN_FLT x9 = 2 * x8;
	const GEN_FLT x10 = obj_qw * obj_qi;
	const GEN_FLT x11 = obj_qk * obj_qj;
	const GEN_FLT x12 = x10 + x11;
	const GEN_FLT x13 = x9 * sensor_y;
	const GEN_FLT x14 = obj_qw * obj_qj;
	const GEN_FLT x15 = obj_qk * obj_qi;
	const GEN_FLT x16 = (-1 * x14) + x15;
	const GEN_FLT x17 = x9 * sensor_x;
	const GEN_FLT x18 = obj_pz + ((1 + (-1 * x5 * x9)) * sensor_z) + (x13 * x12) + (x17 * x16);
	const GEN_FLT x19 = lh_qj * lh_qj;
	const GEN_FLT x20 = lh_qk * lh_qk;
	const GEN_FLT x21 = lh_qi * lh_qi;
	const GEN_FLT x22 = x20 + x21;
	const GEN_FLT x23 = sqrt((x19 + (lh_qw * lh_qw) + x22));
	const GEN_FLT x24 = 2 * x23;
	const GEN_FLT x25 = x24 * x18;
	const GEN_FLT x26 = 1 + (-1 * x24 * x22);
	const GEN_FLT x27 = (-1 * x10) + x11;
	const GEN_FLT x28 = x9 * sensor_z;
	const GEN_FLT x29 = obj_qw * obj_qk;
	const GEN_FLT x30 = obj_qj * obj_qi;
	const GEN_FLT x31 = x29 + x30;
	const GEN_FLT x32 = (x28 * x27) + obj_py + ((1 + (-1 * x7 * x9)) * sensor_y) + (x31 * x17);
	const GEN_FLT x33 = x14 + x15;
	const GEN_FLT x34 = (-1 * x29) + x30;
	const GEN_FLT x35 = x6 + x3;
	const GEN_FLT x36 = obj_px + (x33 * x28) + (x34 * x13) + ((1 + (-1 * x9 * x35)) * sensor_x);
	const GEN_FLT x37 = lh_qw * lh_qk;
	const GEN_FLT x38 = lh_qj * lh_qi;
	const GEN_FLT x39 = x37 + x38;
	const GEN_FLT x40 = x39 * x24;
	const GEN_FLT x41 = lh_py + (x2 * x25) + (x32 * x26) + (x40 * x36);
	const GEN_FLT x42 = x41 * x41;
	const GEN_FLT x43 = 1 + (-1 * (x19 + x21) * x24);
	const GEN_FLT x44 = x0 + x1;
	const GEN_FLT x45 = x44 * x24;
	const GEN_FLT x46 = lh_qw * lh_qj;
	const GEN_FLT x47 = lh_qk * lh_qi;
	const GEN_FLT x48 = (-1 * x46) + x47;
	const GEN_FLT x49 = x48 * x24;
	const GEN_FLT x50 = lh_pz + (x43 * x18) + (x45 * x32) + (x49 * x36);
	const GEN_FLT x51 = x46 + x47;
	const GEN_FLT x52 = (-1 * x37) + x38;
	const GEN_FLT x53 = x52 * x24;
	const GEN_FLT x54 = 1 + (-1 * (x20 + x19) * x24);
	const GEN_FLT x55 = lh_px + (x51 * x25) + (x53 * x32) + (x54 * x36);
	const GEN_FLT x56 = x55 * x55;
	const GEN_FLT x57 = (x50 * x50) + x56;
	const GEN_FLT x58 = x42 + x57;
	const GEN_FLT x59 = pow(x58, -1.0 / 2.0);
	const GEN_FLT x60 = 0.523598775598299 + tilt_0;
	const GEN_FLT x61 = cos(x60);
	const GEN_FLT x62 = pow(x61, -1);
	const GEN_FLT x63 = x62 * x59;
	const GEN_FLT x64 = asin(x63 * x41);
	const GEN_FLT x65 = 8.0108022e-06 * x64;
	const GEN_FLT x66 = -8.0108022e-06 + (-1 * x65);
	const GEN_FLT x67 = 0.0028679863 + (x64 * x66);
	const GEN_FLT x68 = 5.3685255e-06 + (x64 * x67);
	const GEN_FLT x69 = 0.0076069798 + (x64 * x68);
	const GEN_FLT x70 = x64 * x69;
	const GEN_FLT x71 = -8.0108022e-06 + (-1.60216044e-05 * x64);
	const GEN_FLT x72 = (x71 * x64) + x67;
	const GEN_FLT x73 = (x72 * x64) + x68;
	const GEN_FLT x74 = (x73 * x64) + x69;
	const GEN_FLT x75 = x70 + (x74 * x64);
	const GEN_FLT x76 = sin(x60);
	const GEN_FLT x77 = tan(x60);
	const GEN_FLT x78 = pow(x57, -1.0 / 2.0);
	const GEN_FLT x79 = x78 * x41;
	const GEN_FLT x80 = x79 * x77;
	const GEN_FLT x81 = atan2(-1 * x50, x55);
	const GEN_FLT x82 = ogeeMag_0 + (-1 * asin(x80)) + x81;
	const GEN_FLT x83 = curve_0 + (sin(x82) * ogeePhase_0);
	const GEN_FLT x84 = x83 * x76;
	const GEN_FLT x85 = (-1 * x84 * x75) + x61;
	const GEN_FLT x86 = pow(x85, -1);
	const GEN_FLT x87 = x64 * x64;
	const GEN_FLT x88 = x83 * x87;
	const GEN_FLT x89 = x88 * x86;
	const GEN_FLT x90 = (x89 * x69) + x80;
	const GEN_FLT x91 = pow((1 + (-1 * (x90 * x90))), -1.0 / 2.0);
	const GEN_FLT x92 = pow(x58, -1) * x42;
	const GEN_FLT x93 = pow((1 + (-1 * pow(x61, -2) * x92)), -1.0 / 2.0);
	const GEN_FLT x94 = 4 * x23;
	const GEN_FLT x95 = x94 * x41;
	const GEN_FLT x96 = 2 * x55;
	const GEN_FLT x97 = x50 * x94;
	const GEN_FLT x98 = (x54 * x96) + (x97 * x48);
	const GEN_FLT x99 = (x95 * x39) + x98;
	const GEN_FLT x100 = 1.0 / 2.0 * x41;
	const GEN_FLT x101 = pow(x58, -3.0 / 2.0) * x100;
	const GEN_FLT x102 = x62 * x101;
	const GEN_FLT x103 = (-1 * x99 * x102) + (x63 * x40);
	const GEN_FLT x104 = x93 * x103;
	const GEN_FLT x105 = x66 * x104;
	const GEN_FLT x106 = (x64 * ((-1 * x65 * x104) + x105)) + (x67 * x104);
	const GEN_FLT x107 = (x68 * x104) + (x64 * x106);
	const GEN_FLT x108 = pow(x56, -1);
	const GEN_FLT x109 = x50 * x108;
	const GEN_FLT x110 = pow(x55, -1);
	const GEN_FLT x111 = pow(x57, -1);
	const GEN_FLT x112 = x56 * x111;
	const GEN_FLT x113 = ((x54 * x109) + (-1 * x49 * x110)) * x112;
	const GEN_FLT x114 = x42 * x111;
	const GEN_FLT x115 = pow((1 + (-1 * (x77 * x77) * x114)), -1.0 / 2.0);
	const GEN_FLT x116 = pow(x57, -3.0 / 2.0) * x100;
	const GEN_FLT x117 = x77 * x116;
	const GEN_FLT x118 = x78 * x40;
	const GEN_FLT x119 = (-1 * x98 * x117) + (x77 * x118);
	const GEN_FLT x120 = x113 + (-1 * x119 * x115);
	const GEN_FLT x121 = cos(x82) * ogeePhase_0;
	const GEN_FLT x122 = x75 * x76;
	const GEN_FLT x123 = x122 * x121;
	const GEN_FLT x124 = 2.40324066e-05 * x64;
	const GEN_FLT x125 = x73 * x93;
	const GEN_FLT x126 = x88 * pow(x85, -2) * x69;
	const GEN_FLT x127 = x86 * x87 * x69;
	const GEN_FLT x128 = x121 * x127;
	const GEN_FLT x129 = 2 * x83 * x86 * x70;
	const GEN_FLT x130 =
		x91 * ((x89 * x107) +
			   (-1 * x126 *
				((-1 * x120 * x123) +
				 (-1 * x84 *
				  ((x64 * ((x64 * ((x64 * (x105 + (-1 * x104 * x124) + (x71 * x104))) + x106 + (x72 * x104))) +
						   (x103 * x125) + x107)) +
				   (x74 * x104) + (x64 * x107) + (x69 * x104))))) +
			   (x120 * x128) + (x104 * x129) + x119);
	const GEN_FLT x131 = -1 * x113;
	const GEN_FLT x132 = -1 * x81;
	const GEN_FLT x133 = cos((-1 * gibPhase_0) + asin(x90) + x132) * gibMag_0;
	const GEN_FLT x134 = 2 * x50;
	const GEN_FLT x135 = x23 * x108 * x134;
	const GEN_FLT x136 = ((x52 * x135) + (-1 * x45 * x110)) * x112;
	const GEN_FLT x137 = 2 * x41;
	const GEN_FLT x138 = x55 * x94;
	const GEN_FLT x139 = (x52 * x138) + (x97 * x44);
	const GEN_FLT x140 = (x26 * x137) + x139;
	const GEN_FLT x141 = (-1 * x102 * x140) + (x63 * x26);
	const GEN_FLT x142 = x93 * x141;
	const GEN_FLT x143 = x66 * x142;
	const GEN_FLT x144 = (x64 * ((-1 * x65 * x142) + x143)) + (x67 * x142);
	const GEN_FLT x145 = (x68 * x142) + (x64 * x144);
	const GEN_FLT x146 = x116 * x139;
	const GEN_FLT x147 = x78 * x26;
	const GEN_FLT x148 = (-1 * x77 * x146) + (x77 * x147);
	const GEN_FLT x149 = x121 * (x136 + (-1 * x115 * x148));
	const GEN_FLT x150 =
		x91 * ((x89 * x145) +
			   (-1 * x126 *
				((-1 * x122 * x149) +
				 (-1 * x84 *
				  ((x64 * ((x64 * ((x64 * ((-1 * x124 * x142) + x143 + (x71 * x142))) + (x72 * x142) + x144)) +
						   (x125 * x141) + x145)) +
				   (x74 * x142) + (x64 * x145) + (x69 * x142))))) +
			   (x129 * x142) + (x127 * x149) + x148);
	const GEN_FLT x151 = -1 * x136;
	const GEN_FLT x152 = ((x51 * x135) + (-1 * x43 * x110)) * x112;
	const GEN_FLT x153 = (x51 * x138) + (x43 * x134);
	const GEN_FLT x154 = x101 * ((x2 * x95) + x153);
	const GEN_FLT x155 = x2 * x24;
	const GEN_FLT x156 = (-1 * x62 * x154) + (x63 * x155);
	const GEN_FLT x157 = x93 * x156;
	const GEN_FLT x158 = x66 * x157;
	const GEN_FLT x159 = (x64 * ((-1 * x65 * x157) + x158)) + (x67 * x157);
	const GEN_FLT x160 = (x68 * x157) + (x64 * x159);
	const GEN_FLT x161 = x78 * x155;
	const GEN_FLT x162 = (-1 * x117 * x153) + (x77 * x161);
	const GEN_FLT x163 = x152 + (-1 * x115 * x162);
	const GEN_FLT x164 =
		x91 * ((x89 * x160) + (x128 * x163) +
			   (-1 * x126 *
				((-1 * x123 * x163) +
				 (-1 * x84 *
				  ((x74 * x157) + (x69 * x157) +
				   (x64 * ((x64 * ((x64 * (x158 + (-1 * x124 * x157) + (x71 * x157))) + (x72 * x157) + x159)) +
						   (x125 * x156) + x160)) +
				   (x64 * x160))))) +
			   (x129 * x157) + x162);
	const GEN_FLT x165 = -1 * x152;
	const GEN_FLT x166 = 2 * pow(x8, -1);
	const GEN_FLT x167 = x35 * x166;
	const GEN_FLT x168 = x167 * sensor_x;
	const GEN_FLT x169 = x166 * obj_qw;
	const GEN_FLT x170 = x34 * sensor_y;
	const GEN_FLT x171 = x13 * obj_qk;
	const GEN_FLT x172 = x33 * sensor_z;
	const GEN_FLT x173 = x28 * obj_qj;
	const GEN_FLT x174 = (-1 * x168 * obj_qw) + (x169 * x170) + (-1 * x171) + (x169 * x172) + x173;
	const GEN_FLT x175 = x31 * sensor_x;
	const GEN_FLT x176 = x7 * x166;
	const GEN_FLT x177 = x176 * sensor_y;
	const GEN_FLT x178 = x17 * obj_qk;
	const GEN_FLT x179 = x27 * sensor_z;
	const GEN_FLT x180 = x28 * obj_qi;
	const GEN_FLT x181 = (x169 * x175) + (-1 * x177 * obj_qw) + x178 + (x169 * x179) + (-1 * x180);
	const GEN_FLT x182 = x17 * obj_qj;
	const GEN_FLT x183 = x16 * sensor_x;
	const GEN_FLT x184 = x12 * sensor_y;
	const GEN_FLT x185 = x166 * x184;
	const GEN_FLT x186 = x13 * obj_qi;
	const GEN_FLT x187 = x5 * x166;
	const GEN_FLT x188 = x187 * sensor_z;
	const GEN_FLT x189 = (-1 * x182) + (x169 * x183) + (x185 * obj_qw) + x186 + (-1 * x188 * obj_qw);
	const GEN_FLT x190 = (x40 * x174) + (x26 * x181) + (x189 * x155);
	const GEN_FLT x191 = x51 * x24;
	const GEN_FLT x192 = (x54 * x174) + (x53 * x181) + (x189 * x191);
	const GEN_FLT x193 = (x49 * x174) + (x45 * x181) + (x43 * x189);
	const GEN_FLT x194 = (x96 * x192) + (x193 * x134);
	const GEN_FLT x195 = (x190 * x137) + x194;
	const GEN_FLT x196 = (-1 * x102 * x195) + (x63 * x190);
	const GEN_FLT x197 = x93 * x196;
	const GEN_FLT x198 = x66 * x197;
	const GEN_FLT x199 = (x64 * ((-1 * x65 * x197) + x198)) + (x67 * x197);
	const GEN_FLT x200 = (x68 * x197) + (x64 * x199);
	const GEN_FLT x201 = ((x109 * x192) + (-1 * x110 * x193)) * x112;
	const GEN_FLT x202 = x78 * x190;
	const GEN_FLT x203 = (-1 * x117 * x194) + (x77 * x202);
	const GEN_FLT x204 = x201 + (-1 * x203 * x115);
	const GEN_FLT x205 =
		x91 * ((x89 * x200) +
			   (-1 * x126 *
				((-1 * x204 * x123) +
				 (-1 * x84 *
				  ((x64 * ((x64 * ((x64 * (x198 + (-1 * x124 * x197) + (x71 * x197))) + (x72 * x197) + x199)) +
						   (x125 * x196) + x200)) +
				   (x74 * x197) + (x64 * x200) + (x69 * x197))))) +
			   (x204 * x128) + x203 + (x129 * x197));
	const GEN_FLT x206 = -1 * x201;
	const GEN_FLT x207 = x166 * obj_qi;
	const GEN_FLT x208 = x13 * obj_qj;
	const GEN_FLT x209 = x28 * obj_qk;
	const GEN_FLT x210 = (-1 * x168 * obj_qi) + (x207 * x170) + x208 + (x207 * x172) + x209;
	const GEN_FLT x211 = 4 * x8;
	const GEN_FLT x212 = -1 * x211 * obj_qi;
	const GEN_FLT x213 = x28 * obj_qw;
	const GEN_FLT x214 =
		(x207 * x175) + x182 + (((-1 * x176 * obj_qi) + x212) * sensor_y) + (x207 * x179) + (-1 * x213);
	const GEN_FLT x215 = x13 * obj_qw;
	const GEN_FLT x216 = (x207 * x183) + x178 + (x185 * obj_qi) + x215 + (((-1 * x187 * obj_qi) + x212) * sensor_z);
	const GEN_FLT x217 = (x54 * x210) + (x53 * x214) + (x216 * x191);
	const GEN_FLT x218 = (x49 * x210) + (x45 * x214) + (x43 * x216);
	const GEN_FLT x219 = ((x217 * x109) + (-1 * x218 * x110)) * x112;
	const GEN_FLT x220 = (x40 * x210) + (x26 * x214) + (x216 * x155);
	const GEN_FLT x221 = (x96 * x217) + (x218 * x134);
	const GEN_FLT x222 = (x220 * x137) + x221;
	const GEN_FLT x223 = (-1 * x222 * x102) + (x63 * x220);
	const GEN_FLT x224 = x93 * x223;
	const GEN_FLT x225 = x66 * x224;
	const GEN_FLT x226 = (x64 * ((-1 * x65 * x224) + x225)) + (x67 * x224);
	const GEN_FLT x227 = (x68 * x224) + (x64 * x226);
	const GEN_FLT x228 = x78 * x220;
	const GEN_FLT x229 = (-1 * x221 * x117) + (x77 * x228);
	const GEN_FLT x230 = x219 + (-1 * x229 * x115);
	const GEN_FLT x231 =
		x91 * ((x89 * x227) +
			   (-1 * x126 *
				((-1 * x230 * x123) +
				 (-1 * x84 *
				  ((x64 * ((x64 * ((x64 * (x225 + (-1 * x224 * x124) + (x71 * x224))) + (x72 * x224) + x226)) +
						   (x223 * x125) + x227)) +
				   (x74 * x224) + (x64 * x227) + (x69 * x224))))) +
			   (x230 * x128) + (x224 * x129) + x229);
	const GEN_FLT x232 = -1 * x219;
	const GEN_FLT x233 = -1 * x211 * obj_qj;
	const GEN_FLT x234 = x166 * obj_qj;
	const GEN_FLT x235 = (((-1 * x167 * obj_qj) + x233) * sensor_x) + (x234 * x170) + x186 + (x234 * x172) + x213;
	const GEN_FLT x236 = x17 * obj_qi;
	const GEN_FLT x237 = (x234 * x175) + x236 + (-1 * x177 * obj_qj) + (x234 * x179) + x209;
	const GEN_FLT x238 = x17 * obj_qw;
	const GEN_FLT x239 =
		(x234 * x183) + x171 + (-1 * x238) + (x185 * obj_qj) + (((-1 * x187 * obj_qj) + x233) * sensor_z);
	const GEN_FLT x240 = (x54 * x235) + (x53 * x237) + (x239 * x191);
	const GEN_FLT x241 = (x49 * x235) + (x45 * x237) + (x43 * x239);
	const GEN_FLT x242 = ((x240 * x109) + (-1 * x241 * x110)) * x112;
	const GEN_FLT x243 = (x40 * x235) + (x26 * x237) + (x239 * x155);
	const GEN_FLT x244 = (x96 * x240) + (x241 * x134);
	const GEN_FLT x245 = x101 * ((x243 * x137) + x244);
	const GEN_FLT x246 = x59 * x243;
	const GEN_FLT x247 = (-1 * x62 * x245) + (x62 * x246);
	const GEN_FLT x248 = x93 * x247;
	const GEN_FLT x249 = x66 * x248;
	const GEN_FLT x250 = (x64 * ((-1 * x65 * x248) + x249)) + (x67 * x248);
	const GEN_FLT x251 = (x68 * x248) + (x64 * x250);
	const GEN_FLT x252 = x78 * x243;
	const GEN_FLT x253 = (-1 * x244 * x117) + (x77 * x252);
	const GEN_FLT x254 = x242 + (-1 * x253 * x115);
	const GEN_FLT x255 =
		x91 * ((x89 * x251) +
			   (-1 * x126 *
				((-1 * x254 * x123) +
				 (-1 * x84 *
				  ((x64 * ((x64 * ((x64 * (x249 + (-1 * x248 * x124) + (x71 * x248))) + (x72 * x248) + x250)) +
						   (x247 * x125) + x251)) +
				   (x64 * x251) + (x69 * x248) + (x74 * x248))))) +
			   (x248 * x129) + x253 + (x254 * x128));
	const GEN_FLT x256 = -1 * x242;
	const GEN_FLT x257 = x166 * obj_qk;
	const GEN_FLT x258 = -1 * x211 * obj_qk;
	const GEN_FLT x259 =
		(x257 * x170) + (-1 * x215) + (((-1 * x167 * obj_qk) + x258) * sensor_x) + (x257 * x172) + x180;
	const GEN_FLT x260 = (x257 * x175) + x238 + (((-1 * x176 * obj_qk) + x258) * sensor_y) + (x257 * x179) + x173;
	const GEN_FLT x261 = x236 + (-1 * x188 * obj_qk) + (x257 * x184) + (x257 * x183) + x208;
	const GEN_FLT x262 = (x54 * x259) + (x53 * x260) + (x261 * x191);
	const GEN_FLT x263 = (x49 * x259) + (x45 * x260) + (x43 * x261);
	const GEN_FLT x264 = ((x262 * x109) + (-1 * x263 * x110)) * x112;
	const GEN_FLT x265 = (x40 * x259) + (x26 * x260) + (x261 * x155);
	const GEN_FLT x266 = (x96 * x262) + (x263 * x134);
	const GEN_FLT x267 = (x265 * x137) + x266;
	const GEN_FLT x268 = (-1 * x267 * x102) + (x63 * x265);
	const GEN_FLT x269 = x93 * x268;
	const GEN_FLT x270 = x66 * x269;
	const GEN_FLT x271 = (x64 * ((-1 * x65 * x269) + x270)) + (x67 * x269);
	const GEN_FLT x272 = (x68 * x269) + (x64 * x271);
	const GEN_FLT x273 = x78 * x265;
	const GEN_FLT x274 = (-1 * x266 * x117) + (x77 * x273);
	const GEN_FLT x275 = x264 + (-1 * x274 * x115);
	const GEN_FLT x276 =
		x91 * ((x89 * x272) +
			   (-1 * x126 *
				((-1 * x275 * x123) +
				 (-1 * x84 *
				  ((x64 * ((x64 * ((x64 * (x270 + (x71 * x269) + (-1 * x269 * x124))) + x271 + (x72 * x269))) +
						   (x268 * x125) + x272)) +
				   (x74 * x269) + (x64 * x272) + (x69 * x269))))) +
			   (x275 * x128) + (x269 * x129) + x274);
	const GEN_FLT x277 = -1 * x264;
	const GEN_FLT x278 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x279 = cos(x278);
	const GEN_FLT x280 = pow(x279, -1);
	const GEN_FLT x281 = x59 * x280;
	const GEN_FLT x282 = asin(x41 * x281);
	const GEN_FLT x283 = 8.0108022e-06 * x282;
	const GEN_FLT x284 = -8.0108022e-06 + (-1 * x283);
	const GEN_FLT x285 = 0.0028679863 + (x282 * x284);
	const GEN_FLT x286 = 5.3685255e-06 + (x282 * x285);
	const GEN_FLT x287 = 0.0076069798 + (x286 * x282);
	const GEN_FLT x288 = x287 * x282;
	const GEN_FLT x289 = -8.0108022e-06 + (-1.60216044e-05 * x282);
	const GEN_FLT x290 = (x289 * x282) + x285;
	const GEN_FLT x291 = (x290 * x282) + x286;
	const GEN_FLT x292 = (x291 * x282) + x287;
	const GEN_FLT x293 = x288 + (x292 * x282);
	const GEN_FLT x294 = tan(x278);
	const GEN_FLT x295 = -1 * x79 * x294;
	const GEN_FLT x296 = ogeeMag_1 + (-1 * asin(x295)) + x81;
	const GEN_FLT x297 = curve_1 + (sin(x296) * ogeePhase_1);
	const GEN_FLT x298 = sin(x278);
	const GEN_FLT x299 = x297 * x298;
	const GEN_FLT x300 = (x299 * x293) + x279;
	const GEN_FLT x301 = pow(x300, -1);
	const GEN_FLT x302 = x282 * x282;
	const GEN_FLT x303 = x297 * x302;
	const GEN_FLT x304 = x301 * x303;
	const GEN_FLT x305 = (x287 * x304) + x295;
	const GEN_FLT x306 = pow((1 + (-1 * (x305 * x305))), -1.0 / 2.0);
	const GEN_FLT x307 = pow((1 + (-1 * (x294 * x294) * x114)), -1.0 / 2.0);
	const GEN_FLT x308 = x294 * x116;
	const GEN_FLT x309 = (x98 * x308) + (-1 * x294 * x118);
	const GEN_FLT x310 = x113 + (-1 * x309 * x307);
	const GEN_FLT x311 = cos(x296) * ogeePhase_1;
	const GEN_FLT x312 = x287 * x301 * x302;
	const GEN_FLT x313 = x312 * x311;
	const GEN_FLT x314 = pow((1 + (-1 * x92 * pow(x279, -2))), -1.0 / 2.0);
	const GEN_FLT x315 = x280 * x101;
	const GEN_FLT x316 = (-1 * x99 * x315) + (x40 * x281);
	const GEN_FLT x317 = x314 * x316;
	const GEN_FLT x318 = 2 * x297 * x288 * x301;
	const GEN_FLT x319 = x298 * x293;
	const GEN_FLT x320 = x311 * x319;
	const GEN_FLT x321 = x284 * x317;
	const GEN_FLT x322 = 2.40324066e-05 * x282;
	const GEN_FLT x323 = (x285 * x317) + (x282 * ((-1 * x283 * x317) + x321));
	const GEN_FLT x324 = x286 * x314;
	const GEN_FLT x325 = (x282 * x323) + (x324 * x316);
	const GEN_FLT x326 = x287 * x314;
	const GEN_FLT x327 = x287 * pow(x300, -2) * x303;
	const GEN_FLT x328 =
		x306 *
		((x313 * x310) + (x317 * x318) +
		 (-1 * x327 *
		  ((x310 * x320) +
		   (x299 * ((x282 * ((x282 * ((x282 * (x321 + (-1 * x322 * x317) + (x289 * x317))) + (x290 * x317) + x323)) +
							 (x291 * x317) + x325)) +
					(x292 * x317) + (x326 * x316) + (x282 * x325))))) +
		 x309 + (x304 * x325));
	const GEN_FLT x329 = cos((-1 * gibPhase_1) + asin(x305) + x132) * gibMag_1;
	const GEN_FLT x330 = (x294 * x146) + (-1 * x294 * x147);
	const GEN_FLT x331 = x136 + (-1 * x307 * x330);
	const GEN_FLT x332 = (-1 * x315 * x140) + (x26 * x281);
	const GEN_FLT x333 = x314 * x332;
	const GEN_FLT x334 = x284 * x333;
	const GEN_FLT x335 = (x285 * x333) + (x282 * ((-1 * x283 * x333) + x334));
	const GEN_FLT x336 = (x282 * x335) + (x324 * x332);
	const GEN_FLT x337 =
		x306 *
		((x313 * x331) +
		 (-1 * x327 *
		  ((x320 * x331) +
		   (x299 * ((x282 * ((x282 * ((x282 * ((-1 * x322 * x333) + x334 + (x289 * x333))) + (x290 * x333) + x335)) +
							 (x291 * x333) + x336)) +
					(x292 * x333) + (x326 * x332) + (x282 * x336))))) +
		 (x304 * x336) + x330 + (x333 * x318));
	const GEN_FLT x338 = (x308 * x153) + (-1 * x294 * x161);
	const GEN_FLT x339 = x311 * (x152 + (-1 * x307 * x338));
	const GEN_FLT x340 = (-1 * x280 * x154) + (x281 * x155);
	const GEN_FLT x341 = x314 * x340;
	const GEN_FLT x342 = x284 * x341;
	const GEN_FLT x343 = (x285 * x341) + (x282 * ((-1 * x283 * x341) + x342));
	const GEN_FLT x344 = (x282 * x343) + (x324 * x340);
	const GEN_FLT x345 =
		x306 *
		((x339 * x312) + (x304 * x344) + (x318 * x341) +
		 (-1 * x327 *
		  ((x339 * x319) +
		   (x299 * ((x282 * ((x282 * ((x282 * (x342 + (-1 * x322 * x341) + (x289 * x341))) + (x290 * x341) + x343)) +
							 (x291 * x341) + x344)) +
					(x292 * x341) + (x326 * x340) + (x282 * x344))))) +
		 x338);
	const GEN_FLT x346 = (x308 * x194) + (-1 * x202 * x294);
	const GEN_FLT x347 = x201 + (-1 * x307 * x346);
	const GEN_FLT x348 = ((-1 * x315 * x195) + (x281 * x190)) * x314;
	const GEN_FLT x349 = x284 * x348;
	const GEN_FLT x350 = (x285 * x348) + (x282 * ((-1 * x283 * x348) + x349));
	const GEN_FLT x351 = (x282 * x350) + (x286 * x348);
	const GEN_FLT x352 =
		x306 *
		((x313 * x347) + (x351 * x304) + (x348 * x318) +
		 (-1 * x327 *
		  ((x320 * x347) +
		   (x299 * ((x282 * ((x282 * ((x282 * (x349 + (-1 * x322 * x348) + (x289 * x348))) + (x290 * x348) + x350)) +
							 (x291 * x348) + x351)) +
					(x287 * x348) + (x292 * x348) + (x282 * x351))))) +
		 x346);
	const GEN_FLT x353 = (x221 * x308) + (-1 * x294 * x228);
	const GEN_FLT x354 = x219 + (-1 * x353 * x307);
	const GEN_FLT x355 = (-1 * x222 * x315) + (x281 * x220);
	const GEN_FLT x356 = x355 * x314;
	const GEN_FLT x357 = x284 * x356;
	const GEN_FLT x358 = (x285 * x356) + (x282 * ((-1 * x283 * x356) + x357));
	const GEN_FLT x359 = (x282 * x358) + (x355 * x324);
	const GEN_FLT x360 =
		x306 *
		((x354 * x313) + (x356 * x318) +
		 (-1 * x327 *
		  ((x354 * x320) +
		   (x299 * ((x292 * x356) + (x287 * x356) +
					(x282 * ((x282 * ((x282 * (x357 + (-1 * x356 * x322) + (x289 * x356))) + x358 + (x290 * x356))) +
							 (x291 * x356) + x359)) +
					(x282 * x359))))) +
		 (x359 * x304) + x353);
	const GEN_FLT x361 = (x244 * x308) + (-1 * x294 * x252);
	const GEN_FLT x362 = x242 + (-1 * x361 * x307);
	const GEN_FLT x363 = (-1 * x280 * x245) + (x280 * x246);
	const GEN_FLT x364 = x363 * x314;
	const GEN_FLT x365 = x284 * x364;
	const GEN_FLT x366 = (x285 * x364) + (x282 * ((-1 * x283 * x364) + x365));
	const GEN_FLT x367 = (x282 * x366) + (x363 * x324);
	const GEN_FLT x368 =
		x306 *
		((x362 * x313) + (x364 * x318) +
		 (-1 * x327 *
		  ((x362 * x320) +
		   (x299 * ((x282 * ((x282 * ((x282 * (x365 + (-1 * x364 * x322) + (x289 * x364))) + (x290 * x364) + x366)) +
							 (x291 * x364) + x367)) +
					(x292 * x364) + (x363 * x326) + (x282 * x367))))) +
		 (x367 * x304) + x361);
	const GEN_FLT x369 = (x266 * x308) + (-1 * x273 * x294);
	const GEN_FLT x370 = x264 + (-1 * x369 * x307);
	const GEN_FLT x371 = ((-1 * x267 * x315) + (x265 * x281)) * x314;
	const GEN_FLT x372 = x284 * x371;
	const GEN_FLT x373 = (x285 * x371) + (x282 * ((-1 * x283 * x371) + x372));
	const GEN_FLT x374 = (x282 * x373) + (x286 * x371);
	const GEN_FLT x375 =
		x306 *
		((x370 * x313) + (x371 * x318) +
		 (-1 * x327 *
		  ((x370 * x320) +
		   (x299 * ((x282 * ((x282 * ((x282 * (x372 + (-1 * x371 * x322) + (x289 * x371))) + (x290 * x371) + x373)) +
							 (x291 * x371) + x374)) +
					(x292 * x371) + (x287 * x371) + (x282 * x374))))) +
		 x369 + (x374 * x304));
	out[0] = (-1 * x130) + x113 + (-1 * (x131 + x130) * x133);
	out[1] = x136 + (-1 * x150) + (-1 * (x151 + x150) * x133);
	out[2] = x152 + (-1 * x164) + (-1 * (x165 + x164) * x133);
	out[3] = (-1 * x205) + x201 + (-1 * (x206 + x205) * x133);
	out[4] = x219 + (-1 * x231) + (-1 * (x232 + x231) * x133);
	out[5] = x242 + (-1 * x255) + (-1 * (x256 + x255) * x133);
	out[6] = x264 + (-1 * (x277 + x276) * x133) + (-1 * x276);
	out[7] = x113 + (-1 * x328) + (-1 * (x131 + x328) * x329);
	out[8] = x136 + (-1 * x337) + (-1 * (x151 + x337) * x329);
	out[9] = x152 + (-1 * x345) + (-1 * (x165 + x345) * x329);
	out[10] = x201 + (-1 * x352) + (-1 * (x206 + x352) * x329);
	out[11] = x219 + (-1 * x360) + (-1 * (x232 + x360) * x329);
	out[12] = x242 + (-1 * x368) + (-1 * (x256 + x368) * x329);
	out[13] = x264 + (-1 * x375) + (-1 * (x277 + x375) * x329);
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
	const GEN_FLT x0 = lh_qk * lh_qk;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qi * lh_qi;
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt((x1 + (lh_qw * lh_qw) + x3));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * (x0 + x1) * x5);
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = obj_qj * obj_qj;
	const GEN_FLT x9 = obj_qi * obj_qi;
	const GEN_FLT x10 = x7 + x9;
	const GEN_FLT x11 = sqrt((x8 + (obj_qw * obj_qw) + x10));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 + (-1 * (x7 + x8) * x12);
	const GEN_FLT x14 = lh_qw * lh_qk;
	const GEN_FLT x15 = lh_qj * lh_qi;
	const GEN_FLT x16 = (-1 * x14) + x15;
	const GEN_FLT x17 = obj_qw * obj_qk;
	const GEN_FLT x18 = obj_qj * obj_qi;
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 4 * x4 * x11;
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = obj_qw * obj_qj;
	const GEN_FLT x23 = obj_qk * obj_qi;
	const GEN_FLT x24 = (-1 * x22) + x23;
	const GEN_FLT x25 = lh_qw * lh_qj;
	const GEN_FLT x26 = lh_qk * lh_qi;
	const GEN_FLT x27 = x25 + x26;
	const GEN_FLT x28 = x20 * x27;
	const GEN_FLT x29 = (x6 * x13) + (x21 * x16) + (x24 * x28);
	const GEN_FLT x30 = 1 + (-1 * (x1 + x2) * x5);
	const GEN_FLT x31 = 1 + (-1 * (x8 + x9) * x12);
	const GEN_FLT x32 = obj_qw * obj_qi;
	const GEN_FLT x33 = obj_qk * obj_qj;
	const GEN_FLT x34 = x32 + x33;
	const GEN_FLT x35 = x12 * sensor_y;
	const GEN_FLT x36 = x12 * sensor_x;
	const GEN_FLT x37 = obj_pz + (x31 * sensor_z) + (x34 * x35) + (x36 * x24);
	const GEN_FLT x38 = lh_qw * lh_qi;
	const GEN_FLT x39 = lh_qk * lh_qj;
	const GEN_FLT x40 = x38 + x39;
	const GEN_FLT x41 = (-1 * x32) + x33;
	const GEN_FLT x42 = x12 * sensor_z;
	const GEN_FLT x43 = 1 + (-1 * x12 * x10);
	const GEN_FLT x44 = (x41 * x42) + obj_py + (x43 * sensor_y) + (x36 * x19);
	const GEN_FLT x45 = x5 * x44;
	const GEN_FLT x46 = x22 + x23;
	const GEN_FLT x47 = (-1 * x17) + x18;
	const GEN_FLT x48 = obj_px + (x42 * x46) + (x47 * x35) + (x13 * sensor_x);
	const GEN_FLT x49 = (-1 * x25) + x26;
	const GEN_FLT x50 = x5 * x49;
	const GEN_FLT x51 = lh_pz + (x30 * x37) + (x40 * x45) + (x50 * x48);
	const GEN_FLT x52 = x5 * x37;
	const GEN_FLT x53 = lh_px + (x52 * x27) + (x45 * x16) + (x6 * x48);
	const GEN_FLT x54 = x53 * x53;
	const GEN_FLT x55 = pow(x54, -1) * x51;
	const GEN_FLT x56 = pow(x53, -1);
	const GEN_FLT x57 = x30 * x12;
	const GEN_FLT x58 = (x50 * x13) + (x40 * x21) + (x57 * x24);
	const GEN_FLT x59 = (x51 * x51) + x54;
	const GEN_FLT x60 = pow(x59, -1);
	const GEN_FLT x61 = x60 * x54;
	const GEN_FLT x62 = ((x55 * x29) + (-1 * x58 * x56)) * x61;
	const GEN_FLT x63 = 0.523598775598299 + tilt_0;
	const GEN_FLT x64 = cos(x63);
	const GEN_FLT x65 = pow(x64, -1);
	const GEN_FLT x66 = (-1 * x38) + x39;
	const GEN_FLT x67 = 1 + (-1 * x3 * x5);
	const GEN_FLT x68 = x14 + x15;
	const GEN_FLT x69 = x5 * x68;
	const GEN_FLT x70 = lh_py + (x66 * x52) + (x69 * x48) + (x67 * x44);
	const GEN_FLT x71 = x70 * x70;
	const GEN_FLT x72 = x71 + x59;
	const GEN_FLT x73 = pow(x72, -1.0 / 2.0);
	const GEN_FLT x74 = x70 * x73;
	const GEN_FLT x75 = asin(x74 * x65);
	const GEN_FLT x76 = 8.0108022e-06 * x75;
	const GEN_FLT x77 = -8.0108022e-06 + (-1 * x76);
	const GEN_FLT x78 = 0.0028679863 + (x75 * x77);
	const GEN_FLT x79 = 5.3685255e-06 + (x78 * x75);
	const GEN_FLT x80 = 0.0076069798 + (x79 * x75);
	const GEN_FLT x81 = x80 * x75;
	const GEN_FLT x82 = -8.0108022e-06 + (-1.60216044e-05 * x75);
	const GEN_FLT x83 = (x82 * x75) + x78;
	const GEN_FLT x84 = (x83 * x75) + x79;
	const GEN_FLT x85 = (x84 * x75) + x80;
	const GEN_FLT x86 = x81 + (x85 * x75);
	const GEN_FLT x87 = sin(x63);
	const GEN_FLT x88 = tan(x63);
	const GEN_FLT x89 = pow(x59, -1.0 / 2.0);
	const GEN_FLT x90 = x88 * x89;
	const GEN_FLT x91 = x70 * x90;
	const GEN_FLT x92 = atan2(-1 * x51, x53);
	const GEN_FLT x93 = ogeeMag_0 + (-1 * asin(x91)) + x92;
	const GEN_FLT x94 = curve_0 + (sin(x93) * ogeePhase_0);
	const GEN_FLT x95 = x87 * x94;
	const GEN_FLT x96 = (-1 * x86 * x95) + x64;
	const GEN_FLT x97 = pow(x96, -1);
	const GEN_FLT x98 = x75 * x75;
	const GEN_FLT x99 = x98 * x94;
	const GEN_FLT x100 = x99 * x97;
	const GEN_FLT x101 = (x80 * x100) + x91;
	const GEN_FLT x102 = pow((1 + (-1 * (x101 * x101))), -1.0 / 2.0);
	const GEN_FLT x103 = x71 * pow(x72, -1);
	const GEN_FLT x104 = pow((1 + (-1 * pow(x64, -2) * x103)), -1.0 / 2.0);
	const GEN_FLT x105 = x67 * x12;
	const GEN_FLT x106 = x66 * x20;
	const GEN_FLT x107 = (x69 * x13) + (x19 * x105) + (x24 * x106);
	const GEN_FLT x108 = 2 * x70;
	const GEN_FLT x109 = 2 * x53;
	const GEN_FLT x110 = 2 * x51;
	const GEN_FLT x111 = (x29 * x109) + (x58 * x110);
	const GEN_FLT x112 = 1.0 / 2.0 * x70;
	const GEN_FLT x113 = pow(x72, -3.0 / 2.0) * x112;
	const GEN_FLT x114 = x113 * ((x108 * x107) + x111);
	const GEN_FLT x115 = x73 * x107;
	const GEN_FLT x116 = ((-1 * x65 * x114) + (x65 * x115)) * x104;
	const GEN_FLT x117 = x77 * x116;
	const GEN_FLT x118 = (x75 * ((-1 * x76 * x116) + x117)) + (x78 * x116);
	const GEN_FLT x119 = (x79 * x116) + (x75 * x118);
	const GEN_FLT x120 = 2 * x81 * x97 * x94;
	const GEN_FLT x121 = pow(x59, -3.0 / 2.0) * x112;
	const GEN_FLT x122 = x88 * x121;
	const GEN_FLT x123 = (-1 * x111 * x122) + (x90 * x107);
	const GEN_FLT x124 = x71 * x60;
	const GEN_FLT x125 = pow((1 + (-1 * (x88 * x88) * x124)), -1.0 / 2.0);
	const GEN_FLT x126 = cos(x93) * ogeePhase_0;
	const GEN_FLT x127 = x126 * (x62 + (-1 * x123 * x125));
	const GEN_FLT x128 = x86 * x87;
	const GEN_FLT x129 = 2.40324066e-05 * x75;
	const GEN_FLT x130 = x80 * x99 * pow(x96, -2);
	const GEN_FLT x131 = x80 * x98 * x97;
	const GEN_FLT x132 =
		x102 * ((x100 * x119) + (x116 * x120) +
				(-1 * x130 *
				 ((-1 * x128 * x127) +
				  (-1 * x95 *
				   ((x75 * ((x75 * ((x75 * (x117 + (-1 * x116 * x129) + (x82 * x116))) + (x83 * x116) + x118)) +
							(x84 * x116) + x119)) +
					(x85 * x116) + (x75 * x119) + (x80 * x116))))) +
				x123 + (x127 * x131));
	const GEN_FLT x133 = -1 * x62;
	const GEN_FLT x134 = -1 * x92;
	const GEN_FLT x135 = cos((-1 * gibPhase_0) + asin(x101) + x134) * gibMag_0;
	const GEN_FLT x136 = x5 * x43;
	const GEN_FLT x137 = x6 * x12;
	const GEN_FLT x138 = (x16 * x136) + (x47 * x137) + (x34 * x28);
	const GEN_FLT x139 = x47 * x20;
	const GEN_FLT x140 = (x49 * x139) + (x40 * x136) + (x57 * x34);
	const GEN_FLT x141 = ((x55 * x138) + (-1 * x56 * x140)) * x61;
	const GEN_FLT x142 = (x68 * x139) + (x67 * x43) + (x34 * x106);
	const GEN_FLT x143 = (x109 * x138) + (x110 * x140);
	const GEN_FLT x144 = (x108 * x142) + x143;
	const GEN_FLT x145 = x65 * x113;
	const GEN_FLT x146 = x73 * x142;
	const GEN_FLT x147 = x104 * ((-1 * x144 * x145) + (x65 * x146));
	const GEN_FLT x148 = x77 * x147;
	const GEN_FLT x149 = (x75 * ((-1 * x76 * x147) + x148)) + (x78 * x147);
	const GEN_FLT x150 = (x79 * x147) + (x75 * x149);
	const GEN_FLT x151 = x89 * x142;
	const GEN_FLT x152 = (-1 * x122 * x143) + (x88 * x151);
	const GEN_FLT x153 = x141 + (-1 * x125 * x152);
	const GEN_FLT x154 = x128 * x126;
	const GEN_FLT x155 = x126 * x131;
	const GEN_FLT x156 =
		x102 * ((x100 * x150) + (x153 * x155) +
				(-1 * x130 *
				 ((-1 * x153 * x154) +
				  (-1 * x95 *
				   ((x75 * ((x75 * ((x75 * (x148 + (-1 * x129 * x147) + (x82 * x147))) + (x83 * x147) + x149)) +
							(x84 * x147) + x150)) +
					(x85 * x147) + (x75 * x150) + (x80 * x147))))) +
				(x120 * x147) + x152);
	const GEN_FLT x157 = -1 * x141;
	const GEN_FLT x158 = x41 * x20;
	const GEN_FLT x159 = x5 * x31;
	const GEN_FLT x160 = (x46 * x137) + (x16 * x158) + (x27 * x159);
	const GEN_FLT x161 = x46 * x20;
	const GEN_FLT x162 = (x49 * x161) + (x40 * x158) + (x30 * x31);
	const GEN_FLT x163 = ((x55 * x160) + (-1 * x56 * x162)) * x61;
	const GEN_FLT x164 = (x68 * x161) + (x41 * x105) + (x66 * x159);
	const GEN_FLT x165 = (x109 * x160) + (x110 * x162);
	const GEN_FLT x166 = (x108 * x164) + x165;
	const GEN_FLT x167 = x73 * x164;
	const GEN_FLT x168 = x104 * ((-1 * x166 * x145) + (x65 * x167));
	const GEN_FLT x169 = x77 * x168;
	const GEN_FLT x170 = (x75 * ((-1 * x76 * x168) + x169)) + (x78 * x168);
	const GEN_FLT x171 = (x79 * x168) + (x75 * x170);
	const GEN_FLT x172 = x89 * x164;
	const GEN_FLT x173 = (-1 * x122 * x165) + (x88 * x172);
	const GEN_FLT x174 = x163 + (-1 * x125 * x173);
	const GEN_FLT x175 =
		x102 * ((x100 * x171) + (x120 * x168) +
				(-1 * x130 *
				 ((-1 * x174 * x154) +
				  (-1 * x95 *
				   ((x75 * ((x75 * ((x75 * (x169 + (-1 * x129 * x168) + (x82 * x168))) + (x83 * x168) + x170)) +
							(x84 * x168) + x171)) +
					(x85 * x168) + (x75 * x171) + (x80 * x168))))) +
				(x174 * x155) + x173);
	const GEN_FLT x176 = -1 * x163;
	const GEN_FLT x177 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x178 = cos(x177);
	const GEN_FLT x179 = pow(x178, -1);
	const GEN_FLT x180 = asin(x74 * x179);
	const GEN_FLT x181 = 8.0108022e-06 * x180;
	const GEN_FLT x182 = -8.0108022e-06 + (-1 * x181);
	const GEN_FLT x183 = 0.0028679863 + (x180 * x182);
	const GEN_FLT x184 = 5.3685255e-06 + (x180 * x183);
	const GEN_FLT x185 = 0.0076069798 + (x180 * x184);
	const GEN_FLT x186 = x180 * x180;
	const GEN_FLT x187 = tan(x177);
	const GEN_FLT x188 = x89 * x187;
	const GEN_FLT x189 = -1 * x70 * x188;
	const GEN_FLT x190 = ogeeMag_1 + (-1 * asin(x189)) + x92;
	const GEN_FLT x191 = curve_1 + (sin(x190) * ogeePhase_1);
	const GEN_FLT x192 = x180 * x185;
	const GEN_FLT x193 = -8.0108022e-06 + (-1.60216044e-05 * x180);
	const GEN_FLT x194 = (x180 * x193) + x183;
	const GEN_FLT x195 = (x180 * x194) + x184;
	const GEN_FLT x196 = (x180 * x195) + x185;
	const GEN_FLT x197 = x192 + (x180 * x196);
	const GEN_FLT x198 = sin(x177);
	const GEN_FLT x199 = x191 * x198;
	const GEN_FLT x200 = (x197 * x199) + x178;
	const GEN_FLT x201 = pow(x200, -1);
	const GEN_FLT x202 = x201 * x191;
	const GEN_FLT x203 = x202 * x186;
	const GEN_FLT x204 = (x203 * x185) + x189;
	const GEN_FLT x205 = pow((1 + (-1 * (x204 * x204))), -1.0 / 2.0);
	const GEN_FLT x206 = x121 * x187;
	const GEN_FLT x207 = (x206 * x111) + (-1 * x107 * x188);
	const GEN_FLT x208 = pow((1 + (-1 * x124 * (x187 * x187))), -1.0 / 2.0);
	const GEN_FLT x209 = cos(x190) * ogeePhase_1;
	const GEN_FLT x210 = x209 * (x62 + (-1 * x208 * x207));
	const GEN_FLT x211 = x186 * x185;
	const GEN_FLT x212 = x211 * x201;
	const GEN_FLT x213 = pow((1 + (-1 * x103 * pow(x178, -2))), -1.0 / 2.0);
	const GEN_FLT x214 = (-1 * x114 * x179) + (x115 * x179);
	const GEN_FLT x215 = x213 * x214;
	const GEN_FLT x216 = 2 * x202 * x192;
	const GEN_FLT x217 = x197 * x198;
	const GEN_FLT x218 = x215 * x182;
	const GEN_FLT x219 = 2.40324066e-05 * x180;
	const GEN_FLT x220 = (x215 * x183) + (x180 * ((-1 * x215 * x181) + x218));
	const GEN_FLT x221 = (x220 * x180) + (x215 * x184);
	const GEN_FLT x222 = x213 * x185;
	const GEN_FLT x223 = x213 * x196;
	const GEN_FLT x224 = x211 * pow(x200, -2) * x191;
	const GEN_FLT x225 =
		x205 *
		((x210 * x212) + (x203 * x221) + x207 + (x215 * x216) +
		 (-1 * x224 *
		  ((x210 * x217) +
		   (x199 * ((x180 * ((x180 * ((x180 * (x218 + (-1 * x219 * x215) + (x215 * x193))) + (x215 * x194) + x220)) +
							 x221 + (x215 * x195))) +
					(x214 * x222) + (x214 * x223) + (x221 * x180))))));
	const GEN_FLT x226 = cos((-1 * gibPhase_1) + x134 + asin(x204)) * gibMag_1;
	const GEN_FLT x227 = (x206 * x143) + (-1 * x187 * x151);
	const GEN_FLT x228 = x141 + (-1 * x208 * x227);
	const GEN_FLT x229 = x212 * x209;
	const GEN_FLT x230 = x113 * x179;
	const GEN_FLT x231 = (-1 * x230 * x144) + (x179 * x146);
	const GEN_FLT x232 = x213 * x231;
	const GEN_FLT x233 = x217 * x209;
	const GEN_FLT x234 = x232 * x182;
	const GEN_FLT x235 = (x232 * x183) + (x180 * ((-1 * x232 * x181) + x234));
	const GEN_FLT x236 = (x235 * x180) + (x232 * x184);
	const GEN_FLT x237 =
		x205 *
		((x228 * x229) + (x216 * x232) +
		 (-1 * x224 *
		  ((x233 * x228) +
		   (x199 * ((x231 * x223) + (x231 * x222) +
					(x180 * ((x180 * ((x180 * (x234 + (-1 * x219 * x232) + (x232 * x193))) + (x232 * x194) + x235)) +
							 (x232 * x195) + x236)) +
					(x236 * x180))))) +
		 (x236 * x203) + x227);
	const GEN_FLT x238 = (x206 * x165) + (-1 * x172 * x187);
	const GEN_FLT x239 = x163 + (-1 * x238 * x208);
	const GEN_FLT x240 = (-1 * x230 * x166) + (x167 * x179);
	const GEN_FLT x241 = x213 * x240;
	const GEN_FLT x242 = x241 * x182;
	const GEN_FLT x243 = (x241 * x183) + (x180 * ((-1 * x241 * x181) + x242));
	const GEN_FLT x244 = (x243 * x180) + (x241 * x184);
	const GEN_FLT x245 =
		x205 *
		((x239 * x229) + (x216 * x241) + (x203 * x244) + x238 +
		 (-1 * x224 *
		  ((x233 * x239) +
		   (x199 * ((x180 * ((x180 * ((x180 * (x242 + (-1 * x219 * x241) + (x241 * x193))) + (x241 * x194) + x243)) +
							 (x241 * x195) + x244)) +
					(x223 * x240) + (x222 * x240) + (x244 * x180))))));
	out[0] = x62 + (-1 * x132) + (-1 * (x133 + x132) * x135);
	out[1] = x141 + (-1 * x156) + (-1 * (x157 + x156) * x135);
	out[2] = x163 + (-1 * x175) + (-1 * (x176 + x175) * x135);
	out[3] = x62 + (-1 * x225) + (-1 * (x133 + x225) * x226);
	out[4] = x141 + (-1 * x237) + (-1 * (x157 + x237) * x226);
	out[5] = x163 + (-1 * x245) + (-1 * (x176 + x245) * x226);
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
	const GEN_FLT x0 = lh_qj * lh_qj;
	const GEN_FLT x1 = lh_qi * lh_qi;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = lh_qk * lh_qk;
	const GEN_FLT x4 = x3 + x0;
	const GEN_FLT x5 = sqrt((x1 + (lh_qw * lh_qw) + x4));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = obj_qk * obj_qk;
	const GEN_FLT x10 = x9 + x8;
	const GEN_FLT x11 = 2 * sqrt((x7 + (obj_qw * obj_qw) + x10));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 =
		obj_pz + ((1 + (-1 * (x7 + x8) * x11)) * sensor_z) + ((x12 + x13) * x14) + (((-1 * x15) + x16) * x17);
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = (((-1 * x12) + x13) * x22) + obj_py + ((1 + (-1 * x11 * x10)) * sensor_y) + ((x23 + x24) * x17);
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = (-1 * x27) + x28;
	const GEN_FLT x30 =
		obj_px + ((x15 + x16) * x22) + (((-1 * x23) + x24) * x14) + ((1 + (-1 * (x9 + x7) * x11)) * sensor_x);
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + (x18 * (1 + (-1 * x2 * x6))) + (x21 * x26) + (x31 * x29);
	const GEN_FLT x33 = x27 + x28;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = (-1 * x35) + x36;
	const GEN_FLT x38 = lh_px + (x34 * x33) + (x37 * x26) + (x30 * (1 + (-1 * x4 * x6)));
	const GEN_FLT x39 = x38 * x38;
	const GEN_FLT x40 = (x32 * x32) + x39;
	const GEN_FLT x41 = pow(x40, -1);
	const GEN_FLT x42 = x41 * x32;
	const GEN_FLT x43 = (-1 * x19) + x20;
	const GEN_FLT x44 = x3 + x1;
	const GEN_FLT x45 = x35 + x36;
	const GEN_FLT x46 = lh_py + (x43 * x34) + (x25 * (1 + (-1 * x6 * x44))) + (x45 * x31);
	const GEN_FLT x47 = 0.523598775598299 + tilt_0;
	const GEN_FLT x48 = cos(x47);
	const GEN_FLT x49 = pow(x48, -1);
	const GEN_FLT x50 = x46 * x46;
	const GEN_FLT x51 = x50 + x40;
	const GEN_FLT x52 = pow(x51, -1.0 / 2.0);
	const GEN_FLT x53 = x52 * x49;
	const GEN_FLT x54 = asin(x53 * x46);
	const GEN_FLT x55 = 8.0108022e-06 * x54;
	const GEN_FLT x56 = -8.0108022e-06 + (-1 * x55);
	const GEN_FLT x57 = 0.0028679863 + (x54 * x56);
	const GEN_FLT x58 = 5.3685255e-06 + (x54 * x57);
	const GEN_FLT x59 = 0.0076069798 + (x54 * x58);
	const GEN_FLT x60 = x54 * x59;
	const GEN_FLT x61 = -8.0108022e-06 + (-1.60216044e-05 * x54);
	const GEN_FLT x62 = (x61 * x54) + x57;
	const GEN_FLT x63 = (x62 * x54) + x58;
	const GEN_FLT x64 = (x63 * x54) + x59;
	const GEN_FLT x65 = x60 + (x64 * x54);
	const GEN_FLT x66 = sin(x47);
	const GEN_FLT x67 = tan(x47);
	const GEN_FLT x68 = pow(x40, -1.0 / 2.0);
	const GEN_FLT x69 = x67 * x68;
	const GEN_FLT x70 = x69 * x46;
	const GEN_FLT x71 = atan2(-1 * x32, x38);
	const GEN_FLT x72 = ogeeMag_0 + (-1 * asin(x70)) + x71;
	const GEN_FLT x73 = curve_0 + (sin(x72) * ogeePhase_0);
	const GEN_FLT x74 = x73 * x66;
	const GEN_FLT x75 = (-1 * x74 * x65) + x48;
	const GEN_FLT x76 = pow(x75, -1);
	const GEN_FLT x77 = x54 * x54;
	const GEN_FLT x78 = x73 * x77;
	const GEN_FLT x79 = x78 * x76;
	const GEN_FLT x80 = (x79 * x59) + x70;
	const GEN_FLT x81 = pow((1 + (-1 * (x80 * x80))), -1.0 / 2.0);
	const GEN_FLT x82 = pow(x40, -3.0 / 2.0) * x46;
	const GEN_FLT x83 = x82 * x38;
	const GEN_FLT x84 = x83 * x67;
	const GEN_FLT x85 = x50 * pow(x51, -1);
	const GEN_FLT x86 = pow((1 + (-1 * x85 * pow(x48, -2))), -1.0 / 2.0);
	const GEN_FLT x87 = pow(x51, -3.0 / 2.0);
	const GEN_FLT x88 = x87 * x46;
	const GEN_FLT x89 = x88 * x38;
	const GEN_FLT x90 = x89 * x49;
	const GEN_FLT x91 = x86 * x90;
	const GEN_FLT x92 = -1 * x56 * x91;
	const GEN_FLT x93 = x86 * x57;
	const GEN_FLT x94 = (x54 * ((x55 * x91) + x92)) + (-1 * x93 * x90);
	const GEN_FLT x95 = (-1 * x58 * x91) + (x54 * x94);
	const GEN_FLT x96 = x50 * x41;
	const GEN_FLT x97 = pow((1 + (-1 * (x67 * x67) * x96)), -1.0 / 2.0);
	const GEN_FLT x98 = x42 + (x84 * x97);
	const GEN_FLT x99 = cos(x72) * ogeePhase_0;
	const GEN_FLT x100 = x65 * x66;
	const GEN_FLT x101 = x99 * x100;
	const GEN_FLT x102 = 2.40324066e-05 * x54;
	const GEN_FLT x103 = x86 * x63;
	const GEN_FLT x104 = x86 * x64;
	const GEN_FLT x105 = x86 * x59;
	const GEN_FLT x106 = x78 * pow(x75, -2) * x59;
	const GEN_FLT x107 = x77 * x76 * x59;
	const GEN_FLT x108 = x99 * x107;
	const GEN_FLT x109 = 2 * x38;
	const GEN_FLT x110 = x88 * x109;
	const GEN_FLT x111 = x73 * x76 * x60;
	const GEN_FLT x112 = x86 * x49 * x111;
	const GEN_FLT x113 =
		x81 * ((-1 * x84) + (x79 * x95) +
			   (-1 * x106 *
				((-1 * x98 * x101) +
				 (-1 * x74 *
				  ((x54 * ((x54 * ((x54 * (x92 + (x91 * x102) + (-1 * x61 * x91))) + (-1 * x62 * x91) + x94)) +
						   (-1 * x90 * x103) + x95)) +
				   (-1 * x90 * x104) + (x54 * x95) + (-1 * x90 * x105))))) +
			   (x98 * x108) + (-1 * x110 * x112));
	const GEN_FLT x114 = -1 * x42;
	const GEN_FLT x115 = -1 * x71;
	const GEN_FLT x116 = cos((-1 * gibPhase_0) + asin(x80) + x115) * gibMag_0;
	const GEN_FLT x117 = x87 * x50;
	const GEN_FLT x118 = (-1 * x49 * x117) + x53;
	const GEN_FLT x119 = x86 * x118;
	const GEN_FLT x120 = x56 * x119;
	const GEN_FLT x121 = (x54 * ((-1 * x55 * x119) + x120)) + (x93 * x118);
	const GEN_FLT x122 = (x58 * x119) + (x54 * x121);
	const GEN_FLT x123 = x69 * x97;
	const GEN_FLT x124 = 2 * x111;
	const GEN_FLT x125 =
		x81 * (x69 +
			   (-1 * x106 *
				((x101 * x123) +
				 (-1 * x74 *
				  ((x54 * ((x54 * ((x54 * (x120 + (-1 * x102 * x119) + (x61 * x119))) + (x62 * x119) + x121)) +
						   (x103 * x118) + x122)) +
				   (x104 * x118) + (x54 * x122) + (x105 * x118))))) +
			   (x79 * x122) + (-1 * x108 * x123) + (x119 * x124));
	const GEN_FLT x126 = x41 * x38;
	const GEN_FLT x127 = -1 * x126;
	const GEN_FLT x128 = x88 * x32;
	const GEN_FLT x129 = x49 * x128;
	const GEN_FLT x130 = x86 * x129;
	const GEN_FLT x131 = -1 * x56 * x130;
	const GEN_FLT x132 = (x54 * ((x55 * x130) + x131)) + (-1 * x93 * x129);
	const GEN_FLT x133 = (-1 * x58 * x130) + (x54 * x132);
	const GEN_FLT x134 = x82 * x32;
	const GEN_FLT x135 = x67 * x134;
	const GEN_FLT x136 = x127 + (x97 * x135);
	const GEN_FLT x137 = 2 * x32;
	const GEN_FLT x138 = x88 * x137;
	const GEN_FLT x139 =
		x81 * ((x79 * x133) +
			   (-1 * x106 *
				((-1 * x101 * x136) +
				 (-1 * x74 *
				  ((-1 * x104 * x129) +
				   (x54 * ((x54 * ((x54 * (x131 + (x102 * x130) + (-1 * x61 * x130))) + (-1 * x62 * x130) + x132)) +
						   (-1 * x103 * x129) + x133)) +
				   (x54 * x133) + (-1 * x105 * x129))))) +
			   (x108 * x136) + (-1 * x135) + (-1 * x112 * x138));
	const GEN_FLT x140 = 2 * pow(x5, -1);
	const GEN_FLT x141 = x4 * x140;
	const GEN_FLT x142 = x30 * x141;
	const GEN_FLT x143 = x140 * lh_qw;
	const GEN_FLT x144 = x25 * x143;
	const GEN_FLT x145 = x26 * lh_qk;
	const GEN_FLT x146 = x33 * x18;
	const GEN_FLT x147 = x34 * lh_qj;
	const GEN_FLT x148 = (-1 * x142 * lh_qw) + (x37 * x144) + (-1 * x145) + (x143 * x146) + x147;
	const GEN_FLT x149 = x32 * pow(x39, -1);
	const GEN_FLT x150 = pow(x38, -1);
	const GEN_FLT x151 = x30 * x29;
	const GEN_FLT x152 = x26 * lh_qi;
	const GEN_FLT x153 = x31 * lh_qj;
	const GEN_FLT x154 = x2 * x140;
	const GEN_FLT x155 = (x143 * x151) + (x21 * x144) + x152 + (-1 * x153) + (-1 * x18 * x154 * lh_qw);
	const GEN_FLT x156 = x41 * x39;
	const GEN_FLT x157 = ((x148 * x149) + (-1 * x150 * x155)) * x156;
	const GEN_FLT x158 = x45 * x30;
	const GEN_FLT x159 = x31 * lh_qk;
	const GEN_FLT x160 = x44 * x140;
	const GEN_FLT x161 = x25 * x160;
	const GEN_FLT x162 = x43 * x18;
	const GEN_FLT x163 = x34 * lh_qi;
	const GEN_FLT x164 = (x143 * x158) + x159 + (-1 * x161 * lh_qw) + (x162 * x143) + (-1 * x163);
	const GEN_FLT x165 = 2 * x46;
	const GEN_FLT x166 = (x109 * x148) + (x137 * x155);
	const GEN_FLT x167 = 1.0 / 2.0 * x88;
	const GEN_FLT x168 = x167 * ((x165 * x164) + x166);
	const GEN_FLT x169 = (-1 * x49 * x168) + (x53 * x164);
	const GEN_FLT x170 = x86 * x169;
	const GEN_FLT x171 = x56 * x170;
	const GEN_FLT x172 = (x54 * ((-1 * x55 * x170) + x171)) + (x93 * x169);
	const GEN_FLT x173 = (x58 * x170) + (x54 * x172);
	const GEN_FLT x174 = 1.0 / 2.0 * x82;
	const GEN_FLT x175 = x166 * x174;
	const GEN_FLT x176 = (-1 * x67 * x175) + (x69 * x164);
	const GEN_FLT x177 = x157 + (-1 * x97 * x176);
	const GEN_FLT x178 =
		x81 * ((x79 * x173) + (x108 * x177) + (x124 * x170) +
			   (-1 * x106 *
				((-1 * x101 * x177) +
				 (-1 * x74 *
				  ((x54 * ((x54 * ((x54 * (x171 + (-1 * x102 * x170) + (x61 * x170))) + x172 + (x62 * x170))) +
						   (x103 * x169) + x173)) +
				   (x54 * x173) + (x104 * x169) + (x105 * x169))))) +
			   x176);
	const GEN_FLT x179 = -1 * x157;
	const GEN_FLT x180 = x140 * lh_qi;
	const GEN_FLT x181 = x37 * x25;
	const GEN_FLT x182 = x26 * lh_qj;
	const GEN_FLT x183 = x34 * lh_qk;
	const GEN_FLT x184 = (-1 * x142 * lh_qi) + (x181 * x180) + x182 + (x180 * x146) + x183;
	const GEN_FLT x185 = x26 * lh_qw;
	const GEN_FLT x186 = x25 * x21;
	const GEN_FLT x187 = 4 * x5;
	const GEN_FLT x188 = -1 * x187 * lh_qi;
	const GEN_FLT x189 = (x180 * x151) + x159 + x185 + (x18 * ((-1 * x154 * lh_qi) + x188)) + (x180 * x186);
	const GEN_FLT x190 = ((x184 * x149) + (-1 * x189 * x150)) * x156;
	const GEN_FLT x191 = x34 * lh_qw;
	const GEN_FLT x192 = (x180 * x158) + x153 + (x25 * ((-1 * x160 * lh_qi) + x188)) + (x162 * x180) + (-1 * x191);
	const GEN_FLT x193 = (x109 * x184) + (x189 * x137);
	const GEN_FLT x194 = x167 * ((x165 * x192) + x193);
	const GEN_FLT x195 = ((-1 * x49 * x194) + (x53 * x192)) * x86;
	const GEN_FLT x196 = x56 * x195;
	const GEN_FLT x197 = (x54 * ((-1 * x55 * x195) + x196)) + (x57 * x195);
	const GEN_FLT x198 = (x58 * x195) + (x54 * x197);
	const GEN_FLT x199 = x174 * x193;
	const GEN_FLT x200 = (-1 * x67 * x199) + (x69 * x192);
	const GEN_FLT x201 = x99 * (x190 + (-1 * x97 * x200));
	const GEN_FLT x202 =
		x81 * ((x79 * x198) + (x124 * x195) +
			   (-1 * x106 *
				((-1 * x201 * x100) +
				 (-1 * x74 *
				  ((x54 * ((x54 * ((x54 * (x196 + (-1 * x102 * x195) + (x61 * x195))) + (x62 * x195) + x197)) +
						   (x63 * x195) + x198)) +
				   (x64 * x195) + (x54 * x198) + (x59 * x195))))) +
			   (x201 * x107) + x200);
	const GEN_FLT x203 = -1 * x190;
	const GEN_FLT x204 = -1 * x187 * lh_qj;
	const GEN_FLT x205 = x140 * lh_qj;
	const GEN_FLT x206 = (x30 * ((-1 * x141 * lh_qj) + x204)) + (x205 * x181) + x152 + (x205 * x146) + x191;
	const GEN_FLT x207 = x140 * x151;
	const GEN_FLT x208 = x31 * lh_qw;
	const GEN_FLT x209 = (x207 * lh_qj) + (-1 * x208) + (x205 * x186) + (x18 * ((-1 * x154 * lh_qj) + x204)) + x145;
	const GEN_FLT x210 = ((x206 * x149) + (-1 * x209 * x150)) * x156;
	const GEN_FLT x211 = x31 * lh_qi;
	const GEN_FLT x212 = x162 * x140;
	const GEN_FLT x213 = (x205 * x158) + x211 + (-1 * x161 * lh_qj) + x183 + (x212 * lh_qj);
	const GEN_FLT x214 = (x206 * x109) + (x209 * x137);
	const GEN_FLT x215 = x167 * ((x213 * x165) + x214);
	const GEN_FLT x216 = (-1 * x49 * x215) + (x53 * x213);
	const GEN_FLT x217 = x86 * x216;
	const GEN_FLT x218 = x56 * x217;
	const GEN_FLT x219 = (x54 * ((-1 * x55 * x217) + x218)) + (x93 * x216);
	const GEN_FLT x220 = (x58 * x217) + (x54 * x219);
	const GEN_FLT x221 = x214 * x174;
	const GEN_FLT x222 = (-1 * x67 * x221) + (x69 * x213);
	const GEN_FLT x223 = x210 + (-1 * x97 * x222);
	const GEN_FLT x224 =
		x81 * ((x79 * x220) + x222 +
			   (-1 * x106 *
				((-1 * x223 * x101) +
				 (-1 * x74 *
				  ((x54 * ((x54 * ((x54 * (x218 + (x61 * x217) + (-1 * x217 * x102))) + (x62 * x217) + x219)) +
						   (x216 * x103) + x220)) +
				   (x216 * x104) + (x54 * x220) + (x59 * x217))))) +
			   (x217 * x124) + (x223 * x108));
	const GEN_FLT x225 = -1 * x210;
	const GEN_FLT x226 = -1 * x187 * lh_qk;
	const GEN_FLT x227 = x140 * lh_qk;
	const GEN_FLT x228 = x18 * lh_qk;
	const GEN_FLT x229 =
		(x30 * ((-1 * x141 * lh_qk) + x226)) + (-1 * x185) + (x227 * x181) + (x33 * x228 * x140) + x163;
	const GEN_FLT x230 = (x207 * lh_qk) + (x227 * x186) + x211 + (-1 * x228 * x154) + x182;
	const GEN_FLT x231 = ((x229 * x149) + (-1 * x230 * x150)) * x156;
	const GEN_FLT x232 = x208 + (x25 * ((-1 * x160 * lh_qk) + x226)) + (x227 * x158) + x147 + (x212 * lh_qk);
	const GEN_FLT x233 = (x229 * x109) + (x230 * x137);
	const GEN_FLT x234 = x167 * ((x232 * x165) + x233);
	const GEN_FLT x235 = (-1 * x49 * x234) + (x53 * x232);
	const GEN_FLT x236 = x86 * x235;
	const GEN_FLT x237 = x56 * x236;
	const GEN_FLT x238 = (x54 * ((-1 * x55 * x236) + x237)) + (x93 * x235);
	const GEN_FLT x239 = (x58 * x236) + (x54 * x238);
	const GEN_FLT x240 = x233 * x174;
	const GEN_FLT x241 = (-1 * x67 * x240) + (x69 * x232);
	const GEN_FLT x242 = x231 + (-1 * x97 * x241);
	const GEN_FLT x243 =
		x81 * ((x79 * x239) + (x242 * x108) +
			   (-1 * x106 *
				((-1 * x242 * x101) +
				 (-1 * x74 *
				  ((x54 * ((x54 * ((x54 * (x237 + (-1 * x236 * x102) + (x61 * x236))) + (x62 * x236) + x238)) +
						   (x235 * x103) + x239)) +
				   (x64 * x236) + (x54 * x239) + (x59 * x236))))) +
			   (x236 * x124) + x241);
	const GEN_FLT x244 = -1 * x231;
	const GEN_FLT x245 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x246 = cos(x245);
	const GEN_FLT x247 = pow(x246, -1);
	const GEN_FLT x248 = x52 * x247;
	const GEN_FLT x249 = asin(x46 * x248);
	const GEN_FLT x250 = 8.0108022e-06 * x249;
	const GEN_FLT x251 = -8.0108022e-06 + (-1 * x250);
	const GEN_FLT x252 = 0.0028679863 + (x251 * x249);
	const GEN_FLT x253 = 5.3685255e-06 + (x252 * x249);
	const GEN_FLT x254 = 0.0076069798 + (x253 * x249);
	const GEN_FLT x255 = x254 * x249;
	const GEN_FLT x256 = -8.0108022e-06 + (-1.60216044e-05 * x249);
	const GEN_FLT x257 = (x256 * x249) + x252;
	const GEN_FLT x258 = (x257 * x249) + x253;
	const GEN_FLT x259 = (x258 * x249) + x254;
	const GEN_FLT x260 = x255 + (x259 * x249);
	const GEN_FLT x261 = tan(x245);
	const GEN_FLT x262 = x68 * x261;
	const GEN_FLT x263 = -1 * x46 * x262;
	const GEN_FLT x264 = ogeeMag_1 + (-1 * asin(x263)) + x71;
	const GEN_FLT x265 = curve_1 + (sin(x264) * ogeePhase_1);
	const GEN_FLT x266 = sin(x245);
	const GEN_FLT x267 = x266 * x265;
	const GEN_FLT x268 = (x267 * x260) + x246;
	const GEN_FLT x269 = pow(x268, -1);
	const GEN_FLT x270 = x249 * x249;
	const GEN_FLT x271 = x270 * x265;
	const GEN_FLT x272 = x271 * x269;
	const GEN_FLT x273 = (x272 * x254) + x263;
	const GEN_FLT x274 = pow((1 + (-1 * (x273 * x273))), -1.0 / 2.0);
	const GEN_FLT x275 = x83 * x261;
	const GEN_FLT x276 = pow((1 + (-1 * x96 * (x261 * x261))), -1.0 / 2.0);
	const GEN_FLT x277 = cos(x264) * ogeePhase_1;
	const GEN_FLT x278 = x277 * (x42 + (-1 * x276 * x275));
	const GEN_FLT x279 = x270 * x269 * x254;
	const GEN_FLT x280 = pow((1 + (-1 * x85 * pow(x246, -2))), -1.0 / 2.0);
	const GEN_FLT x281 = x269 * x265 * x255;
	const GEN_FLT x282 = x280 * x281 * x247;
	const GEN_FLT x283 = x266 * x260;
	const GEN_FLT x284 = x89 * x247;
	const GEN_FLT x285 = x284 * x280;
	const GEN_FLT x286 = -1 * x285 * x251;
	const GEN_FLT x287 = 2.40324066e-05 * x249;
	const GEN_FLT x288 = x280 * x252;
	const GEN_FLT x289 = (-1 * x288 * x284) + (x249 * ((x285 * x250) + x286));
	const GEN_FLT x290 = (x289 * x249) + (-1 * x285 * x253);
	const GEN_FLT x291 = x271 * pow(x268, -2) * x254;
	const GEN_FLT x292 =
		x274 * (x275 + (x279 * x278) + (-1 * x282 * x110) +
				(-1 * x291 *
				 ((x278 * x283) + (x267 * ((x249 * ((x249 * ((x249 * (x286 + (x287 * x285) + (-1 * x285 * x256))) +
															 (-1 * x285 * x257) + x289)) +
													(-1 * x285 * x258) + x290)) +
										   (-1 * x285 * x259) + (-1 * x285 * x254) + (x290 * x249))))) +
				(x272 * x290));
	const GEN_FLT x293 = cos((-1 * gibPhase_1) + asin(x273) + x115) * gibMag_1;
	const GEN_FLT x294 = x279 * x277;
	const GEN_FLT x295 = x276 * x262;
	const GEN_FLT x296 = (-1 * x247 * x117) + x248;
	const GEN_FLT x297 = x296 * x280;
	const GEN_FLT x298 = 2 * x281;
	const GEN_FLT x299 = x277 * x283;
	const GEN_FLT x300 = x297 * x251;
	const GEN_FLT x301 = (x296 * x288) + (x249 * ((-1 * x297 * x250) + x300));
	const GEN_FLT x302 = (x249 * x301) + (x297 * x253);
	const GEN_FLT x303 =
		x274 *
		((x295 * x294) + (x297 * x298) + (-1 * x262) +
		 (-1 * x291 *
		  ((x299 * x295) +
		   (x267 * ((x249 * ((x249 * ((x249 * (x300 + (-1 * x297 * x287) + (x297 * x256))) + (x297 * x257) + x301)) +
							 (x297 * x258) + x302)) +
					(x297 * x254) + (x297 * x259) + (x249 * x302))))) +
		 (x272 * x302));
	const GEN_FLT x304 = x261 * x134;
	const GEN_FLT x305 = x127 + (-1 * x276 * x304);
	const GEN_FLT x306 = x247 * x128;
	const GEN_FLT x307 = x280 * x306;
	const GEN_FLT x308 = -1 * x251 * x307;
	const GEN_FLT x309 = (-1 * x288 * x306) + (x249 * ((x250 * x307) + x308));
	const GEN_FLT x310 = (x249 * x309) + (-1 * x253 * x307);
	const GEN_FLT x311 =
		x274 * (x304 + (-1 * x282 * x138) + (x294 * x305) +
				(-1 * x291 *
				 ((x299 * x305) + (x267 * ((x249 * ((x249 * ((x249 * (x308 + (x287 * x307) + (-1 * x256 * x307))) +
															 (-1 * x257 * x307) + x309)) +
													(-1 * x258 * x307) + x310)) +
										   (-1 * x259 * x307) + (-1 * x254 * x307) + (x249 * x310))))) +
				(x272 * x310));
	const GEN_FLT x312 = (x261 * x175) + (-1 * x262 * x164);
	const GEN_FLT x313 = x157 + (-1 * x276 * x312);
	const GEN_FLT x314 = (-1 * x247 * x168) + (x248 * x164);
	const GEN_FLT x315 = x280 * x314;
	const GEN_FLT x316 = x251 * x315;
	const GEN_FLT x317 = (x288 * x314) + (x249 * ((-1 * x250 * x315) + x316));
	const GEN_FLT x318 = (x249 * x317) + (x253 * x315);
	const GEN_FLT x319 =
		x274 *
		((x294 * x313) + (x298 * x315) +
		 (-1 * x291 *
		  ((x299 * x313) +
		   (x267 * ((x249 * ((x249 * ((x249 * (x316 + (-1 * x287 * x315) + (x256 * x315))) + x317 + (x257 * x315))) +
							 (x258 * x315) + x318)) +
					(x249 * x318) + (x259 * x315) + (x254 * x315))))) +
		 (x272 * x318) + x312);
	const GEN_FLT x320 = (x261 * x199) + (-1 * x262 * x192);
	const GEN_FLT x321 = x190 + (-1 * x276 * x320);
	const GEN_FLT x322 = (-1 * x247 * x194) + (x248 * x192);
	const GEN_FLT x323 = x280 * x322;
	const GEN_FLT x324 = x251 * x323;
	const GEN_FLT x325 = (x288 * x322) + (x249 * ((-1 * x250 * x323) + x324));
	const GEN_FLT x326 = (x249 * x325) + (x253 * x323);
	const GEN_FLT x327 =
		x274 *
		((x294 * x321) + (x298 * x323) +
		 (-1 * x291 *
		  ((x299 * x321) +
		   (x267 * ((x249 * ((x249 * ((x249 * (x324 + (-1 * x287 * x323) + (x256 * x323))) + (x257 * x323) + x325)) +
							 (x258 * x323) + x326)) +
					(x259 * x323) + (x254 * x323) + (x249 * x326))))) +
		 (x272 * x326) + x320);
	const GEN_FLT x328 = (x261 * x221) + (-1 * x213 * x262);
	const GEN_FLT x329 = x210 + (-1 * x276 * x328);
	const GEN_FLT x330 = ((-1 * x215 * x247) + (x213 * x248)) * x280;
	const GEN_FLT x331 = x251 * x330;
	const GEN_FLT x332 = (x252 * x330) + (x249 * ((-1 * x250 * x330) + x331));
	const GEN_FLT x333 = (x249 * x332) + (x253 * x330);
	const GEN_FLT x334 =
		x274 *
		((x294 * x329) + (x298 * x330) + x328 +
		 (-1 * x291 *
		  ((x299 * x329) +
		   (x267 * ((x249 * ((x249 * ((x249 * (x331 + (-1 * x287 * x330) + (x256 * x330))) + (x257 * x330) + x332)) +
							 (x258 * x330) + x333)) +
					(x259 * x330) + (x254 * x330) + (x249 * x333))))) +
		 (x272 * x333));
	const GEN_FLT x335 = (x261 * x240) + (-1 * x232 * x262);
	const GEN_FLT x336 = x231 + (-1 * x276 * x335);
	const GEN_FLT x337 = ((-1 * x234 * x247) + (x232 * x248)) * x280;
	const GEN_FLT x338 = x251 * x337;
	const GEN_FLT x339 = (x252 * x337) + (x249 * ((-1 * x250 * x337) + x338));
	const GEN_FLT x340 = (x249 * x339) + (x253 * x337);
	const GEN_FLT x341 =
		x274 *
		((x294 * x336) + (x298 * x337) + (x272 * x340) +
		 (-1 * x291 *
		  ((x299 * x336) +
		   (x267 * ((x249 * ((x249 * ((x249 * (x338 + (-1 * x287 * x337) + (x256 * x337))) + (x257 * x337) + x339)) +
							 (x258 * x337) + x340)) +
					(x259 * x337) + (x254 * x337) + (x249 * x340))))) +
		 x335);
	out[0] = x42 + (-1 * x113) + (-1 * (x114 + x113) * x116);
	out[1] = (-1 * x125) + (-1 * x116 * x125);
	out[2] = x127 + (-1 * x139) + (-1 * (x126 + x139) * x116);
	out[3] = x157 + (-1 * x178) + (-1 * (x179 + x178) * x116);
	out[4] = x190 + (-1 * x202) + (-1 * (x203 + x202) * x116);
	out[5] = x210 + (-1 * (x225 + x224) * x116) + (-1 * x224);
	out[6] = x231 + (-1 * (x244 + x243) * x116) + (-1 * x243);
	out[7] = x42 + (-1 * x292) + (-1 * (x114 + x292) * x293);
	out[8] = (-1 * x303) + (-1 * x293 * x303);
	out[9] = x127 + (-1 * x311) + (-1 * (x126 + x311) * x293);
	out[10] = x157 + (-1 * x319) + (-1 * (x179 + x319) * x293);
	out[11] = (-1 * x327) + x190 + (-1 * (x203 + x327) * x293);
	out[12] = x210 + (-1 * x334) + (-1 * (x225 + x334) * x293);
	out[13] = x231 + (-1 * x341) + (-1 * (x244 + x341) * x293);
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
	const GEN_FLT x4 = obj_qj * obj_qj;
	const GEN_FLT x5 = obj_qi * obj_qi;
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = 2 * sqrt((x7 + (obj_qw * obj_qw) + x6));
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = obj_pz + ((1 + (-1 * x6 * x8)) * sensor_z) + (x11 * (x9 + x10)) + (((-1 * x12) + x13) * x14);
	const GEN_FLT x16 = lh_qi * lh_qi;
	const GEN_FLT x17 = lh_qk * lh_qk;
	const GEN_FLT x18 = lh_qj * lh_qj;
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 2 * sqrt((x16 + (lh_qw * lh_qw) + x19));
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 =
		(x22 * ((-1 * x9) + x10)) + ((x23 + x24) * x14) + obj_py + ((1 + (-1 * (x7 + x5) * x8)) * sensor_y);
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 =
		obj_px + ((x12 + x13) * x22) + (((-1 * x23) + x24) * x11) + ((1 + (-1 * (x7 + x4) * x8)) * sensor_x);
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = lh_py + (((-1 * x2) + x3) * x21) + ((x26 + x27) * x29) + (x25 * (1 + (-1 * (x17 + x16) * x20)));
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = lh_pz + (x15 * (1 + (-1 * (x18 + x16) * x20))) + ((x2 + x3) * x31) + (((-1 * x32) + x33) * x29);
	const GEN_FLT x35 = lh_px + ((x32 + x33) * x21) + (((-1 * x26) + x27) * x31) + (x28 * (1 + (-1 * x20 * x19)));
	const GEN_FLT x36 = (x34 * x34) + (x35 * x35);
	const GEN_FLT x37 = x30 * pow(x36, -1.0 / 2.0);
	const GEN_FLT x38 = x1 * x37;
	const GEN_FLT x39 = atan2(-1 * x34, x35);
	const GEN_FLT x40 = ogeeMag_0 + (-1 * asin(x38)) + x39;
	const GEN_FLT x41 = sin(x40);
	const GEN_FLT x42 = curve_0 + (x41 * ogeePhase_0);
	const GEN_FLT x43 = cos(x0);
	const GEN_FLT x44 = x30 * x30;
	const GEN_FLT x45 = x44 + x36;
	const GEN_FLT x46 = pow(x45, -1.0 / 2.0) * x30;
	const GEN_FLT x47 = asin(pow(x43, -1) * x46);
	const GEN_FLT x48 = 8.0108022e-06 * x47;
	const GEN_FLT x49 = -8.0108022e-06 + (-1 * x48);
	const GEN_FLT x50 = 0.0028679863 + (x47 * x49);
	const GEN_FLT x51 = 5.3685255e-06 + (x50 * x47);
	const GEN_FLT x52 = 0.0076069798 + (x51 * x47);
	const GEN_FLT x53 = sin(x0);
	const GEN_FLT x54 = x52 * x47;
	const GEN_FLT x55 = -8.0108022e-06 + (-1.60216044e-05 * x47);
	const GEN_FLT x56 = (x55 * x47) + x50;
	const GEN_FLT x57 = (x56 * x47) + x51;
	const GEN_FLT x58 = (x57 * x47) + x52;
	const GEN_FLT x59 = x54 + (x58 * x47);
	const GEN_FLT x60 = x59 * x42;
	const GEN_FLT x61 = x60 * x53;
	const GEN_FLT x62 = (-1 * x61) + x43;
	const GEN_FLT x63 = pow(x62, -1);
	const GEN_FLT x64 = x47 * x47;
	const GEN_FLT x65 = x63 * x64;
	const GEN_FLT x66 = x65 * x52;
	const GEN_FLT x67 = (x66 * x42) + x38;
	const GEN_FLT x68 = pow((1 + (-1 * (x67 * x67))), -1.0 / 2.0);
	const GEN_FLT x69 = x1 * x1;
	const GEN_FLT x70 = x37 * (1 + x69);
	const GEN_FLT x71 = pow(x43, -2);
	const GEN_FLT x72 = x44 * pow(x45, -1);
	const GEN_FLT x73 = x71 * x46 * pow((1 + (-1 * x71 * x72)), -1.0 / 2.0);
	const GEN_FLT x74 = x73 * x53;
	const GEN_FLT x75 = x74 * x49;
	const GEN_FLT x76 = (x47 * ((-1 * x74 * x48) + x75)) + (x74 * x50);
	const GEN_FLT x77 = (x74 * x51) + (x76 * x47);
	const GEN_FLT x78 = cos(x40) * ogeePhase_0;
	const GEN_FLT x79 = x44 * pow(x36, -1);
	const GEN_FLT x80 = x70 * pow((1 + (-1 * x79 * x69)), -1.0 / 2.0);
	const GEN_FLT x81 = x53 * x42;
	const GEN_FLT x82 = x64 * pow(x62, -2) * x52;
	const GEN_FLT x83 = x78 * x66;
	const GEN_FLT x84 =
		x68 * (x70 + (x77 * x65 * x42) +
			   (-1 * x82 * x42 *
				((-1 * x53) + (-1 * x60 * x43) + (x80 * x78 * x53 * x59) +
				 (-1 * x81 *
				  ((x47 * ((x47 * ((x47 * (x75 + (-2.40324066e-05 * x74 * x47) + (x74 * x55))) + (x74 * x56) + x76)) +
						   (x74 * x57) + x77)) +
				   (x74 * x58) + (x77 * x47) + (x74 * x52))))) +
			   (-1 * x80 * x83) + (2 * x81 * x73 * x63 * x54));
	const GEN_FLT x85 = -1 * x39;
	const GEN_FLT x86 = (-1 * gibPhase_0) + asin(x67) + x85;
	const GEN_FLT x87 = cos(x86) * gibMag_0;
	const GEN_FLT x88 = x82 * x61;
	const GEN_FLT x89 = (x88 + x66) * x68;
	const GEN_FLT x90 = x68 * ((x88 * x78) + x83);
	const GEN_FLT x91 = ((x88 * x41) + (x66 * x41)) * x68;
	const GEN_FLT x92 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x93 = tan(x92);
	const GEN_FLT x94 = -1 * x93 * x37;
	const GEN_FLT x95 = ogeeMag_1 + (-1 * asin(x94)) + x39;
	const GEN_FLT x96 = sin(x95);
	const GEN_FLT x97 = curve_1 + (x96 * ogeePhase_1);
	const GEN_FLT x98 = cos(x92);
	const GEN_FLT x99 = asin(pow(x98, -1) * x46);
	const GEN_FLT x100 = 8.0108022e-06 * x99;
	const GEN_FLT x101 = -8.0108022e-06 + (-1 * x100);
	const GEN_FLT x102 = 0.0028679863 + (x99 * x101);
	const GEN_FLT x103 = 5.3685255e-06 + (x99 * x102);
	const GEN_FLT x104 = 0.0076069798 + (x99 * x103);
	const GEN_FLT x105 = x99 * x99;
	const GEN_FLT x106 = x99 * x104;
	const GEN_FLT x107 = -8.0108022e-06 + (-1.60216044e-05 * x99);
	const GEN_FLT x108 = (x99 * x107) + x102;
	const GEN_FLT x109 = (x99 * x108) + x103;
	const GEN_FLT x110 = (x99 * x109) + x104;
	const GEN_FLT x111 = x106 + (x99 * x110);
	const GEN_FLT x112 = sin(x92);
	const GEN_FLT x113 = x97 * x112;
	const GEN_FLT x114 = x111 * x113;
	const GEN_FLT x115 = x114 + x98;
	const GEN_FLT x116 = pow(x115, -1);
	const GEN_FLT x117 = x105 * x116;
	const GEN_FLT x118 = x104 * x117;
	const GEN_FLT x119 = (x97 * x118) + x94;
	const GEN_FLT x120 = pow((1 + (-1 * (x119 * x119))), -1.0 / 2.0);
	const GEN_FLT x121 = x93 * x93;
	const GEN_FLT x122 = x37 * (1 + x121);
	const GEN_FLT x123 = cos(x95) * ogeePhase_1;
	const GEN_FLT x124 = x118 * x123;
	const GEN_FLT x125 = x122 * pow((1 + (-1 * x79 * x121)), -1.0 / 2.0);
	const GEN_FLT x126 = pow(x98, -2);
	const GEN_FLT x127 = x46 * x126 * pow((1 + (-1 * x72 * x126)), -1.0 / 2.0);
	const GEN_FLT x128 = x112 * x127;
	const GEN_FLT x129 = -1 * x101 * x128;
	const GEN_FLT x130 = (-1 * x102 * x128) + (x99 * ((x100 * x128) + x129));
	const GEN_FLT x131 = (x99 * x130) + (-1 * x103 * x128);
	const GEN_FLT x132 = x105 * x104 * pow(x115, -2);
	const GEN_FLT x133 =
		x120 * (x122 +
				(-1 * x97 * x132 *
				 (x112 + (-1 * x111 * x112 * x123 * x125) + (-1 * x98 * x97 * x111) +
				  (x113 * ((x99 * ((x99 * ((x99 * (x129 + (2.40324066e-05 * x99 * x128) + (-1 * x107 * x128))) +
										   (-1 * x108 * x128) + x130)) +
								   (-1 * x109 * x128) + x131)) +
						   (-1 * x110 * x128) + (-1 * x104 * x128) + (x99 * x131))))) +
				(-1 * x124 * x125) + (x97 * x117 * x131) + (-2 * x106 * x113 * x116 * x127));
	const GEN_FLT x134 = (-1 * gibPhase_1) + asin(x119) + x85;
	const GEN_FLT x135 = cos(x134) * gibMag_1;
	const GEN_FLT x136 = x114 * x132;
	const GEN_FLT x137 = (x118 + (-1 * x136)) * x120;
	const GEN_FLT x138 = x120 * (x124 + (-1 * x123 * x136));
	const GEN_FLT x139 = ((x96 * x118) + (-1 * x96 * x136)) * x120;
	out[0] = -1;
	out[1] = (-1 * x84) + (-1 * x84 * x87);
	out[2] = (-1 * x89) + (-1 * x89 * x87);
	out[3] = x87;
	out[4] = -1 * sin(x86);
	out[5] = (-1 * x90) + (-1 * x87 * x90);
	out[6] = (-1 * x91) + (-1 * x87 * x91);
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
	out[22] = (-1 * x133) + (-1 * x133 * x135);
	out[23] = (-1 * x137) + (-1 * x137 * x135);
	out[24] = x135;
	out[25] = -1 * sin(x134);
	out[26] = (-1 * x138) + (-1 * x138 * x135);
	out[27] = (-1 * x139) + (-1 * x135 * x139);
}

/** Applying function <function reproject_axis_x_gen2 at 0x7f35a4af8950> */
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
	const GEN_FLT x4 = obj_qj * obj_qj;
	const GEN_FLT x5 = obj_qi * obj_qi;
	const GEN_FLT x6 = obj_qk * obj_qk;
	const GEN_FLT x7 = x6 + x5;
	const GEN_FLT x8 = 2 * sqrt((x4 + (obj_qw * obj_qw) + x7));
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 =
		obj_pz + ((1 + (-1 * (x4 + x5) * x8)) * sensor_z) + (x11 * (x9 + x10)) + (((-1 * x12) + x13) * x14);
	const GEN_FLT x16 = lh_qi * lh_qi;
	const GEN_FLT x17 = lh_qk * lh_qk;
	const GEN_FLT x18 = lh_qj * lh_qj;
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 2 * sqrt((x16 + (lh_qw * lh_qw) + x19));
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = (x22 * ((-1 * x9) + x10)) + ((x23 + x24) * x14) + obj_py + ((1 + (-1 * x8 * x7)) * sensor_y);
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 =
		obj_px + ((x12 + x13) * x22) + (((-1 * x23) + x24) * x11) + ((1 + (-1 * (x6 + x4) * x8)) * sensor_x);
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = lh_py + (((-1 * x2) + x3) * x21) + ((x26 + x27) * x29) + (x25 * (1 + (-1 * (x17 + x16) * x20)));
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = lh_pz + (x15 * (1 + (-1 * (x18 + x16) * x20))) + ((x2 + x3) * x31) + (((-1 * x32) + x33) * x29);
	const GEN_FLT x35 = lh_px + ((x32 + x33) * x21) + (((-1 * x26) + x27) * x31) + (x28 * (1 + (-1 * x20 * x19)));
	const GEN_FLT x36 = (x34 * x34) + (x35 * x35);
	const GEN_FLT x37 = asin(pow(x1, -1) * x30 * pow(((x30 * x30) + x36), -1.0 / 2.0));
	const GEN_FLT x38 = 0.0028679863 + (x37 * (-8.0108022e-06 + (-8.0108022e-06 * x37)));
	const GEN_FLT x39 = 5.3685255e-06 + (x38 * x37);
	const GEN_FLT x40 = 0.0076069798 + (x37 * x39);
	const GEN_FLT x41 = tan(x0) * x30 * pow(x36, -1.0 / 2.0);
	const GEN_FLT x42 = atan2(-1 * x34, x35);
	const GEN_FLT x43 = curve_0 + (sin(ogeeMag_0 + (-1 * asin(x41)) + x42) * ogeePhase_0);
	const GEN_FLT x44 = asin(
		(x40 * x43 * (x37 * x37) *
		 pow(((-1 * sin(x0) * x43 *
			   ((x40 * x37) +
				(x37 * ((x37 * ((x37 * ((x37 * (-8.0108022e-06 + (-1.60216044e-05 * x37))) + x38)) + x39)) + x40)))) +
			  x1),
			 -1)) +
		x41);
	out[0] = -1.5707963267949 + (-1 * phase_0) + (sin(gibPhase_0 + (-1 * x44) + x42) * gibMag_0) + (-1 * x44) + x42;
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
	const GEN_FLT x0 = obj_qj * obj_qj;
	const GEN_FLT x1 = obj_qi * obj_qi;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = obj_qk * obj_qk;
	const GEN_FLT x4 = x3 + x1;
	const GEN_FLT x5 = sqrt((x0 + (obj_qw * obj_qw) + x4));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x7 + x8;
	const GEN_FLT x10 = x6 * sensor_y;
	const GEN_FLT x11 = obj_qw * obj_qj;
	const GEN_FLT x12 = obj_qk * obj_qi;
	const GEN_FLT x13 = (-1 * x11) + x12;
	const GEN_FLT x14 = x6 * sensor_x;
	const GEN_FLT x15 = obj_pz + ((1 + (-1 * x2 * x6)) * sensor_z) + (x9 * x10) + (x14 * x13);
	const GEN_FLT x16 = lh_qw * lh_qi;
	const GEN_FLT x17 = lh_qk * lh_qj;
	const GEN_FLT x18 = (-1 * x16) + x17;
	const GEN_FLT x19 = lh_qj * lh_qj;
	const GEN_FLT x20 = lh_qk * lh_qk;
	const GEN_FLT x21 = lh_qi * lh_qi;
	const GEN_FLT x22 = x20 + x21;
	const GEN_FLT x23 = sqrt((x19 + (lh_qw * lh_qw) + x22));
	const GEN_FLT x24 = 2 * x23;
	const GEN_FLT x25 = x24 * x18;
	const GEN_FLT x26 = 1 + (-1 * x24 * x22);
	const GEN_FLT x27 = (-1 * x7) + x8;
	const GEN_FLT x28 = x6 * sensor_z;
	const GEN_FLT x29 = obj_qw * obj_qk;
	const GEN_FLT x30 = obj_qj * obj_qi;
	const GEN_FLT x31 = x29 + x30;
	const GEN_FLT x32 = (x28 * x27) + obj_py + ((1 + (-1 * x4 * x6)) * sensor_y) + (x31 * x14);
	const GEN_FLT x33 = x11 + x12;
	const GEN_FLT x34 = (-1 * x29) + x30;
	const GEN_FLT x35 = x3 + x0;
	const GEN_FLT x36 = obj_px + (x33 * x28) + (x34 * x10) + ((1 + (-1 * x6 * x35)) * sensor_x);
	const GEN_FLT x37 = lh_qw * lh_qk;
	const GEN_FLT x38 = lh_qj * lh_qi;
	const GEN_FLT x39 = x37 + x38;
	const GEN_FLT x40 = x39 * x24;
	const GEN_FLT x41 = lh_py + (x25 * x15) + (x32 * x26) + (x40 * x36);
	const GEN_FLT x42 = 0.523598775598299 + tilt_0;
	const GEN_FLT x43 = cos(x42);
	const GEN_FLT x44 = pow(x43, -1);
	const GEN_FLT x45 = x41 * x41;
	const GEN_FLT x46 = 1 + (-1 * (x19 + x21) * x24);
	const GEN_FLT x47 = x16 + x17;
	const GEN_FLT x48 = x47 * x24;
	const GEN_FLT x49 = lh_qw * lh_qj;
	const GEN_FLT x50 = lh_qk * lh_qi;
	const GEN_FLT x51 = (-1 * x49) + x50;
	const GEN_FLT x52 = x51 * x24;
	const GEN_FLT x53 = lh_pz + (x46 * x15) + (x48 * x32) + (x52 * x36);
	const GEN_FLT x54 = x49 + x50;
	const GEN_FLT x55 = x54 * x24;
	const GEN_FLT x56 = (-1 * x37) + x38;
	const GEN_FLT x57 = x56 * x24;
	const GEN_FLT x58 = 1 + (-1 * (x20 + x19) * x24);
	const GEN_FLT x59 = lh_px + (x55 * x15) + (x57 * x32) + (x58 * x36);
	const GEN_FLT x60 = x59 * x59;
	const GEN_FLT x61 = (x53 * x53) + x60;
	const GEN_FLT x62 = x45 + x61;
	const GEN_FLT x63 = pow(x62, -1.0 / 2.0) * x44;
	const GEN_FLT x64 = asin(x63 * x41);
	const GEN_FLT x65 = 8.0108022e-06 * x64;
	const GEN_FLT x66 = -8.0108022e-06 + (-1 * x65);
	const GEN_FLT x67 = 0.0028679863 + (x64 * x66);
	const GEN_FLT x68 = 5.3685255e-06 + (x64 * x67);
	const GEN_FLT x69 = 0.0076069798 + (x64 * x68);
	const GEN_FLT x70 = x64 * x69;
	const GEN_FLT x71 = -8.0108022e-06 + (-1.60216044e-05 * x64);
	const GEN_FLT x72 = (x71 * x64) + x67;
	const GEN_FLT x73 = (x72 * x64) + x68;
	const GEN_FLT x74 = (x73 * x64) + x69;
	const GEN_FLT x75 = x70 + (x74 * x64);
	const GEN_FLT x76 = sin(x42);
	const GEN_FLT x77 = tan(x42);
	const GEN_FLT x78 = x77 * pow(x61, -1.0 / 2.0);
	const GEN_FLT x79 = x78 * x41;
	const GEN_FLT x80 = atan2(-1 * x53, x59);
	const GEN_FLT x81 = ogeeMag_0 + (-1 * asin(x79)) + x80;
	const GEN_FLT x82 = curve_0 + (sin(x81) * ogeePhase_0);
	const GEN_FLT x83 = x82 * x76;
	const GEN_FLT x84 = (-1 * x83 * x75) + x43;
	const GEN_FLT x85 = pow(x84, -1);
	const GEN_FLT x86 = x64 * x64;
	const GEN_FLT x87 = x82 * x86;
	const GEN_FLT x88 = x85 * x87;
	const GEN_FLT x89 = (x88 * x69) + x79;
	const GEN_FLT x90 = pow((1 + (-1 * (x89 * x89))), -1.0 / 2.0);
	const GEN_FLT x91 = pow((1 + (-1 * pow(x62, -1) * pow(x43, -2) * x45)), -1.0 / 2.0);
	const GEN_FLT x92 = 4 * x23;
	const GEN_FLT x93 = x92 * x41;
	const GEN_FLT x94 = 2 * x59;
	const GEN_FLT x95 = x53 * x92;
	const GEN_FLT x96 = (x58 * x94) + (x51 * x95);
	const GEN_FLT x97 = 1.0 / 2.0 * x41;
	const GEN_FLT x98 = pow(x62, -3.0 / 2.0) * x97 * x44;
	const GEN_FLT x99 = (-1 * x98 * ((x93 * x39) + x96)) + (x63 * x40);
	const GEN_FLT x100 = x91 * x99;
	const GEN_FLT x101 = x66 * x100;
	const GEN_FLT x102 = x67 * x91;
	const GEN_FLT x103 = (x64 * ((-1 * x65 * x100) + x101)) + (x99 * x102);
	const GEN_FLT x104 = (x68 * x100) + (x64 * x103);
	const GEN_FLT x105 = pow(x60, -1);
	const GEN_FLT x106 = x53 * x105;
	const GEN_FLT x107 = pow(x59, -1);
	const GEN_FLT x108 = pow(x61, -1);
	const GEN_FLT x109 = x60 * x108;
	const GEN_FLT x110 = ((x58 * x106) + (-1 * x52 * x107)) * x109;
	const GEN_FLT x111 = pow((1 + (-1 * (x77 * x77) * x45 * x108)), -1.0 / 2.0);
	const GEN_FLT x112 = x77 * pow(x61, -3.0 / 2.0) * x97;
	const GEN_FLT x113 = (-1 * x96 * x112) + (x78 * x40);
	const GEN_FLT x114 = x110 + (-1 * x111 * x113);
	const GEN_FLT x115 = cos(x81) * ogeePhase_0;
	const GEN_FLT x116 = x75 * x76;
	const GEN_FLT x117 = x115 * x116;
	const GEN_FLT x118 = 2.40324066e-05 * x64;
	const GEN_FLT x119 = pow(x84, -2) * x87 * x69;
	const GEN_FLT x120 = x85 * x86 * x69;
	const GEN_FLT x121 = x115 * x120;
	const GEN_FLT x122 = 2 * x82 * x85 * x70;
	const GEN_FLT x123 =
		x90 * ((x88 * x104) + (x100 * x122) +
			   (-1 * x119 *
				((-1 * x114 * x117) +
				 (-1 * x83 *
				  ((x64 * ((x64 * ((x64 * (x101 + (-1 * x100 * x118) + (x71 * x100))) + (x72 * x100) + x103)) +
						   (x73 * x100) + x104)) +
				   (x74 * x100) + (x64 * x104) + (x69 * x100))))) +
			   (x114 * x121) + x113);
	const GEN_FLT x124 = cos(gibPhase_0 + (-1 * asin(x89)) + x80) * gibMag_0;
	const GEN_FLT x125 = 2 * x53;
	const GEN_FLT x126 = x23 * x105 * x125;
	const GEN_FLT x127 = ((x56 * x126) + (-1 * x48 * x107)) * x109;
	const GEN_FLT x128 = 2 * x41;
	const GEN_FLT x129 = x59 * x92;
	const GEN_FLT x130 = (x56 * x129) + (x95 * x47);
	const GEN_FLT x131 = (-1 * x98 * ((x26 * x128) + x130)) + (x63 * x26);
	const GEN_FLT x132 = x91 * x131;
	const GEN_FLT x133 = x66 * x132;
	const GEN_FLT x134 = (x64 * ((-1 * x65 * x132) + x133)) + (x102 * x131);
	const GEN_FLT x135 = (x68 * x132) + (x64 * x134);
	const GEN_FLT x136 = (-1 * x112 * x130) + (x78 * x26);
	const GEN_FLT x137 = x115 * (x127 + (-1 * x111 * x136));
	const GEN_FLT x138 =
		x90 * ((x88 * x135) +
			   (-1 * x119 *
				((-1 * x116 * x137) +
				 (-1 * x83 *
				  ((x64 * ((x64 * ((x64 * ((-1 * x118 * x132) + x133 + (x71 * x132))) + (x72 * x132) + x134)) +
						   (x73 * x132) + x135)) +
				   (x74 * x132) + (x64 * x135) + (x69 * x132))))) +
			   (x122 * x132) + (x120 * x137) + x136);
	const GEN_FLT x139 = ((x54 * x126) + (-1 * x46 * x107)) * x109;
	const GEN_FLT x140 = (x54 * x129) + (x46 * x125);
	const GEN_FLT x141 = (-1 * x98 * ((x93 * x18) + x140)) + (x63 * x25);
	const GEN_FLT x142 = x91 * x141;
	const GEN_FLT x143 = x66 * x142;
	const GEN_FLT x144 = (x64 * ((-1 * x65 * x142) + x143)) + (x102 * x141);
	const GEN_FLT x145 = (x68 * x142) + (x64 * x144);
	const GEN_FLT x146 = (-1 * x112 * x140) + (x78 * x25);
	const GEN_FLT x147 = x139 + (-1 * x111 * x146);
	const GEN_FLT x148 =
		x90 * ((x88 * x145) +
			   (-1 * x119 *
				((-1 * x117 * x147) +
				 (-1 * x83 *
				  ((x74 * x142) +
				   (x64 * ((x64 * ((x64 * (x143 + (-1 * x118 * x142) + (x71 * x142))) + (x72 * x142) + x144)) +
						   (x73 * x142) + x145)) +
				   (x64 * x145) + (x69 * x142))))) +
			   (x121 * x147) + (x122 * x142) + x146);
	const GEN_FLT x149 = 2 * pow(x5, -1);
	const GEN_FLT x150 = x149 * obj_qw;
	const GEN_FLT x151 = x35 * sensor_x;
	const GEN_FLT x152 = x34 * sensor_y;
	const GEN_FLT x153 = x10 * obj_qk;
	const GEN_FLT x154 = x33 * sensor_z;
	const GEN_FLT x155 = x28 * obj_qj;
	const GEN_FLT x156 = (-1 * x150 * x151) + (x150 * x152) + (-1 * x153) + (x150 * x154) + x155;
	const GEN_FLT x157 = x31 * sensor_x;
	const GEN_FLT x158 = x4 * sensor_y;
	const GEN_FLT x159 = x14 * obj_qk;
	const GEN_FLT x160 = x27 * sensor_z;
	const GEN_FLT x161 = x28 * obj_qi;
	const GEN_FLT x162 = (x150 * x157) + (-1 * x150 * x158) + x159 + (x160 * x150) + (-1 * x161);
	const GEN_FLT x163 = x14 * obj_qj;
	const GEN_FLT x164 = x13 * sensor_x;
	const GEN_FLT x165 = x9 * sensor_y;
	const GEN_FLT x166 = x10 * obj_qi;
	const GEN_FLT x167 = x2 * sensor_z;
	const GEN_FLT x168 = (-1 * x163) + (x164 * x150) + (x165 * x150) + (-1 * x167 * x150) + x166;
	const GEN_FLT x169 = (x40 * x156) + (x26 * x162) + (x25 * x168);
	const GEN_FLT x170 = (x58 * x156) + (x57 * x162) + (x55 * x168);
	const GEN_FLT x171 = (x52 * x156) + (x48 * x162) + (x46 * x168);
	const GEN_FLT x172 = (x94 * x170) + (x125 * x171);
	const GEN_FLT x173 = (-1 * x98 * ((x128 * x169) + x172)) + (x63 * x169);
	const GEN_FLT x174 = x91 * x173;
	const GEN_FLT x175 = x66 * x174;
	const GEN_FLT x176 = (x64 * ((-1 * x65 * x174) + x175)) + (x102 * x173);
	const GEN_FLT x177 = (x68 * x174) + (x64 * x176);
	const GEN_FLT x178 = ((x106 * x170) + (-1 * x107 * x171)) * x109;
	const GEN_FLT x179 = (-1 * x112 * x172) + (x78 * x169);
	const GEN_FLT x180 = x178 + (-1 * x111 * x179);
	const GEN_FLT x181 =
		x90 * ((x88 * x177) +
			   (-1 * x119 *
				((-1 * x117 * x180) +
				 (-1 * x83 *
				  ((x64 * ((x64 * ((x64 * (x175 + (-1 * x118 * x174) + (x71 * x174))) + (x72 * x174) + x176)) +
						   (x73 * x174) + x177)) +
				   (x74 * x174) + (x64 * x177) + (x69 * x174))))) +
			   (x121 * x180) + (x122 * x174) + x179);
	const GEN_FLT x182 = x149 * obj_qi;
	const GEN_FLT x183 = x10 * obj_qj;
	const GEN_FLT x184 = x28 * obj_qk;
	const GEN_FLT x185 = (-1 * x182 * x151) + (x182 * x152) + x183 + (x182 * x154) + x184;
	const GEN_FLT x186 = 4 * x5;
	const GEN_FLT x187 = -1 * x186 * obj_qi;
	const GEN_FLT x188 = x28 * obj_qw;
	const GEN_FLT x189 = (x182 * x157) + x163 + (((-1 * x4 * x182) + x187) * sensor_y) + (-1 * x188) + (x160 * x182);
	const GEN_FLT x190 = x10 * obj_qw;
	const GEN_FLT x191 = (x164 * x182) + x159 + (x165 * x182) + x190 + (((-1 * x2 * x182) + x187) * sensor_z);
	const GEN_FLT x192 = (x58 * x185) + (x55 * x191) + (x57 * x189);
	const GEN_FLT x193 = (x52 * x185) + (x48 * x189) + (x46 * x191);
	const GEN_FLT x194 = ((x106 * x192) + (-1 * x107 * x193)) * x109;
	const GEN_FLT x195 = (x40 * x185) + (x26 * x189) + (x25 * x191);
	const GEN_FLT x196 = (x94 * x192) + (x125 * x193);
	const GEN_FLT x197 = x91 * ((-1 * x98 * ((x128 * x195) + x196)) + (x63 * x195));
	const GEN_FLT x198 = x66 * x197;
	const GEN_FLT x199 = (x64 * ((-1 * x65 * x197) + x198)) + (x67 * x197);
	const GEN_FLT x200 = (x68 * x197) + (x64 * x199);
	const GEN_FLT x201 = (-1 * x112 * x196) + (x78 * x195);
	const GEN_FLT x202 = x194 + (-1 * x201 * x111);
	const GEN_FLT x203 =
		x90 * ((x88 * x200) +
			   (-1 * x119 *
				((-1 * x202 * x117) +
				 (-1 * x83 *
				  ((x64 * ((x64 * ((x64 * (x198 + (-1 * x118 * x197) + (x71 * x197))) + (x72 * x197) + x199)) +
						   (x73 * x197) + x200)) +
				   (x64 * x200) + (x74 * x197) + (x69 * x197))))) +
			   (x202 * x121) + (x122 * x197) + x201);
	const GEN_FLT x204 = x149 * obj_qj;
	const GEN_FLT x205 = -1 * x186 * obj_qj;
	const GEN_FLT x206 = (((-1 * x35 * x204) + x205) * sensor_x) + x188 + (x204 * x152) + x166 + (x204 * x154);
	const GEN_FLT x207 = x14 * obj_qi;
	const GEN_FLT x208 = (x204 * x157) + x207 + (-1 * x204 * x158) + (x204 * x160) + x184;
	const GEN_FLT x209 = x14 * obj_qw;
	const GEN_FLT x210 = (x204 * x164) + (-1 * x209) + (x204 * x165) + x153 + (((-1 * x2 * x204) + x205) * sensor_z);
	const GEN_FLT x211 = (x58 * x206) + (x57 * x208) + (x55 * x210);
	const GEN_FLT x212 = (x52 * x206) + (x48 * x208) + (x46 * x210);
	const GEN_FLT x213 = ((x211 * x106) + (-1 * x212 * x107)) * x109;
	const GEN_FLT x214 = (x40 * x206) + (x26 * x208) + (x25 * x210);
	const GEN_FLT x215 = (x94 * x211) + (x212 * x125);
	const GEN_FLT x216 = x91 * ((-1 * x98 * ((x214 * x128) + x215)) + (x63 * x214));
	const GEN_FLT x217 = x66 * x216;
	const GEN_FLT x218 = (x64 * ((-1 * x65 * x216) + x217)) + (x67 * x216);
	const GEN_FLT x219 = (x68 * x216) + (x64 * x218);
	const GEN_FLT x220 = (-1 * x215 * x112) + (x78 * x214);
	const GEN_FLT x221 = x213 + (-1 * x220 * x111);
	const GEN_FLT x222 =
		x90 * ((x88 * x219) +
			   (-1 * x119 *
				((-1 * x221 * x117) +
				 (-1 * x83 *
				  ((x64 * ((x64 * ((x64 * (x217 + (-1 * x216 * x118) + (x71 * x216))) + (x72 * x216) + x218)) +
						   (x73 * x216) + x219)) +
				   (x74 * x216) + (x64 * x219) + (x69 * x216))))) +
			   (x216 * x122) + (x221 * x121) + x220);
	const GEN_FLT x223 = x149 * obj_qk;
	const GEN_FLT x224 = -1 * x186 * obj_qk;
	const GEN_FLT x225 = (x223 * x152) + (-1 * x190) + (((-1 * x35 * x223) + x224) * sensor_x) + (x223 * x154) + x161;
	const GEN_FLT x226 = (x223 * x157) + x209 + (x223 * x160) + x155 + (((-1 * x4 * x223) + x224) * sensor_y);
	const GEN_FLT x227 = x207 + (x223 * x165) + (x223 * x164) + (-1 * x223 * x167) + x183;
	const GEN_FLT x228 = (x58 * x225) + (x57 * x226) + (x55 * x227);
	const GEN_FLT x229 = (x52 * x225) + (x48 * x226) + (x46 * x227);
	const GEN_FLT x230 = ((x228 * x106) + (-1 * x229 * x107)) * x109;
	const GEN_FLT x231 = (x40 * x225) + (x25 * x227) + (x26 * x226);
	const GEN_FLT x232 = (x94 * x228) + (x229 * x125);
	const GEN_FLT x233 = x91 * ((-1 * x98 * ((x231 * x128) + x232)) + (x63 * x231));
	const GEN_FLT x234 = x66 * x233;
	const GEN_FLT x235 = (x64 * ((-1 * x65 * x233) + x234)) + (x67 * x233);
	const GEN_FLT x236 = (x68 * x233) + (x64 * x235);
	const GEN_FLT x237 = (-1 * x232 * x112) + (x78 * x231);
	const GEN_FLT x238 = x230 + (-1 * x237 * x111);
	const GEN_FLT x239 =
		x90 * ((x88 * x236) +
			   (-1 * x119 *
				((-1 * x238 * x117) +
				 (-1 * x83 *
				  ((x64 * ((x64 * ((x64 * (x234 + (-1 * x233 * x118) + (x71 * x233))) + (x72 * x233) + x235)) +
						   (x73 * x233) + x236)) +
				   (x74 * x233) + (x64 * x236) + (x69 * x233))))) +
			   (x238 * x121) + (x233 * x122) + x237);
	out[0] = (-1 * x123) + x110 + (-1 * ((-1 * x110) + x123) * x124);
	out[1] = x127 + (-1 * x138) + (-1 * ((-1 * x127) + x138) * x124);
	out[2] = x139 + (-1 * x148) + (-1 * ((-1 * x139) + x148) * x124);
	out[3] = (-1 * x181) + x178 + (-1 * ((-1 * x178) + x181) * x124);
	out[4] = x194 + (-1 * x203) + (-1 * ((-1 * x194) + x203) * x124);
	out[5] = x213 + (-1 * x222) + (-1 * ((-1 * x213) + x222) * x124);
	out[6] = x230 + (-1 * ((-1 * x230) + x239) * x124) + (-1 * x239);
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
	const GEN_FLT x0 = lh_qk * lh_qk;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qi * lh_qi;
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt((x1 + (lh_qw * lh_qw) + x3));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * (x0 + x1) * x5);
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = obj_qj * obj_qj;
	const GEN_FLT x9 = obj_qi * obj_qi;
	const GEN_FLT x10 = x7 + x9;
	const GEN_FLT x11 = sqrt((x8 + (obj_qw * obj_qw) + x10));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 + (-1 * (x7 + x8) * x12);
	const GEN_FLT x14 = lh_qw * lh_qk;
	const GEN_FLT x15 = lh_qj * lh_qi;
	const GEN_FLT x16 = (-1 * x14) + x15;
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
	const GEN_FLT x27 = (-1 * x25) + x26;
	const GEN_FLT x28 = x20 * x27;
	const GEN_FLT x29 = (x6 * x13) + (x21 * x16) + (x24 * x28);
	const GEN_FLT x30 = 1 + (-1 * (x1 + x2) * x5);
	const GEN_FLT x31 = 1 + (-1 * (x8 + x9) * x12);
	const GEN_FLT x32 = obj_qw * obj_qi;
	const GEN_FLT x33 = obj_qk * obj_qj;
	const GEN_FLT x34 = x32 + x33;
	const GEN_FLT x35 = x34 * x12;
	const GEN_FLT x36 = x27 * x12;
	const GEN_FLT x37 = obj_pz + (x31 * sensor_z) + (x35 * sensor_y) + (x36 * sensor_x);
	const GEN_FLT x38 = lh_qw * lh_qi;
	const GEN_FLT x39 = lh_qk * lh_qj;
	const GEN_FLT x40 = x38 + x39;
	const GEN_FLT x41 = (-1 * x32) + x33;
	const GEN_FLT x42 = x41 * x12;
	const GEN_FLT x43 = 1 + (-1 * x12 * x10);
	const GEN_FLT x44 = x12 * x19;
	const GEN_FLT x45 = (x42 * sensor_z) + obj_py + (x43 * sensor_y) + (x44 * sensor_x);
	const GEN_FLT x46 = x5 * x45;
	const GEN_FLT x47 = (-1 * x22) + x23;
	const GEN_FLT x48 = x25 + x26;
	const GEN_FLT x49 = (-1 * x17) + x18;
	const GEN_FLT x50 = obj_px + (x48 * x12 * sensor_z) + (x49 * x12 * sensor_y) + (x13 * sensor_x);
	const GEN_FLT x51 = x5 * x50;
	const GEN_FLT x52 = lh_pz + (x30 * x37) + (x40 * x46) + (x51 * x47);
	const GEN_FLT x53 = x5 * x37;
	const GEN_FLT x54 = lh_px + (x53 * x24) + (x46 * x16) + (x6 * x50);
	const GEN_FLT x55 = x54 * x54;
	const GEN_FLT x56 = x52 * pow(x55, -1);
	const GEN_FLT x57 = pow(x54, -1);
	const GEN_FLT x58 = x5 * x13;
	const GEN_FLT x59 = (x58 * x47) + (x40 * x21) + (x30 * x36);
	const GEN_FLT x60 = (x52 * x52) + x55;
	const GEN_FLT x61 = pow(x60, -1);
	const GEN_FLT x62 = x61 * x55;
	const GEN_FLT x63 = ((x56 * x29) + (-1 * x57 * x59)) * x62;
	const GEN_FLT x64 = (-1 * x38) + x39;
	const GEN_FLT x65 = 1 + (-1 * x3 * x5);
	const GEN_FLT x66 = x14 + x15;
	const GEN_FLT x67 = lh_py + (x64 * x53) + (x65 * x45) + (x66 * x51);
	const GEN_FLT x68 = 0.523598775598299 + tilt_0;
	const GEN_FLT x69 = cos(x68);
	const GEN_FLT x70 = pow(x69, -1);
	const GEN_FLT x71 = x67 * x67;
	const GEN_FLT x72 = x71 + x60;
	const GEN_FLT x73 = x70 * pow(x72, -1.0 / 2.0);
	const GEN_FLT x74 = asin(x73 * x67);
	const GEN_FLT x75 = 8.0108022e-06 * x74;
	const GEN_FLT x76 = -8.0108022e-06 + (-1 * x75);
	const GEN_FLT x77 = 0.0028679863 + (x74 * x76);
	const GEN_FLT x78 = 5.3685255e-06 + (x74 * x77);
	const GEN_FLT x79 = 0.0076069798 + (x78 * x74);
	const GEN_FLT x80 = x79 * x74;
	const GEN_FLT x81 = -8.0108022e-06 + (-1.60216044e-05 * x74);
	const GEN_FLT x82 = (x81 * x74) + x77;
	const GEN_FLT x83 = (x82 * x74) + x78;
	const GEN_FLT x84 = (x83 * x74) + x79;
	const GEN_FLT x85 = x80 + (x84 * x74);
	const GEN_FLT x86 = sin(x68);
	const GEN_FLT x87 = tan(x68);
	const GEN_FLT x88 = x87 * pow(x60, -1.0 / 2.0);
	const GEN_FLT x89 = x88 * x67;
	const GEN_FLT x90 = atan2(-1 * x52, x54);
	const GEN_FLT x91 = ogeeMag_0 + (-1 * asin(x89)) + x90;
	const GEN_FLT x92 = curve_0 + (sin(x91) * ogeePhase_0);
	const GEN_FLT x93 = x86 * x92;
	const GEN_FLT x94 = (-1 * x85 * x93) + x69;
	const GEN_FLT x95 = pow(x94, -1);
	const GEN_FLT x96 = x74 * x74;
	const GEN_FLT x97 = x92 * x96;
	const GEN_FLT x98 = x97 * x95;
	const GEN_FLT x99 = (x79 * x98) + x89;
	const GEN_FLT x100 = pow((1 + (-1 * (x99 * x99))), -1.0 / 2.0);
	const GEN_FLT x101 = pow((1 + (-1 * x71 * pow(x72, -1) * pow(x69, -2))), -1.0 / 2.0);
	const GEN_FLT x102 = (x66 * x58) + (x65 * x44) + (x64 * x28);
	const GEN_FLT x103 = 2 * x67;
	const GEN_FLT x104 = 2 * x54;
	const GEN_FLT x105 = 2 * x52;
	const GEN_FLT x106 = (x29 * x104) + (x59 * x105);
	const GEN_FLT x107 = 1.0 / 2.0 * x67;
	const GEN_FLT x108 = x70 * pow(x72, -3.0 / 2.0) * x107;
	const GEN_FLT x109 = x101 * ((-1 * x108 * ((x103 * x102) + x106)) + (x73 * x102));
	const GEN_FLT x110 = x76 * x109;
	const GEN_FLT x111 = (x74 * ((-1 * x75 * x109) + x110)) + (x77 * x109);
	const GEN_FLT x112 = (x78 * x109) + (x74 * x111);
	const GEN_FLT x113 = 2 * x80 * x92 * x95;
	const GEN_FLT x114 = x87 * pow(x60, -3.0 / 2.0) * x107;
	const GEN_FLT x115 = (-1 * x106 * x114) + (x88 * x102);
	const GEN_FLT x116 = pow((1 + (-1 * (x87 * x87) * x71 * x61)), -1.0 / 2.0);
	const GEN_FLT x117 = cos(x91) * ogeePhase_0;
	const GEN_FLT x118 = x117 * (x63 + (-1 * x115 * x116));
	const GEN_FLT x119 = x85 * x86;
	const GEN_FLT x120 = 2.40324066e-05 * x74;
	const GEN_FLT x121 = x79 * x97 * pow(x94, -2);
	const GEN_FLT x122 = x79 * x96 * x95;
	const GEN_FLT x123 =
		x100 * ((x98 * x112) + (x109 * x113) +
				(-1 * x121 *
				 ((-1 * x118 * x119) +
				  (-1 * x93 *
				   ((x74 * ((x74 * ((x74 * (x110 + (-1 * x109 * x120) + (x81 * x109))) + x111 + (x82 * x109))) +
							(x83 * x109) + x112)) +
					(x84 * x109) + (x74 * x112) + (x79 * x109))))) +
				(x118 * x122) + x115);
	const GEN_FLT x124 = cos(gibPhase_0 + (-1 * asin(x99)) + x90) * gibMag_0;
	const GEN_FLT x125 = x5 * x43;
	const GEN_FLT x126 = x6 * x12;
	const GEN_FLT x127 = x34 * x20;
	const GEN_FLT x128 = (x16 * x125) + (x49 * x126) + (x24 * x127);
	const GEN_FLT x129 = x49 * x20;
	const GEN_FLT x130 = (x47 * x129) + (x40 * x125) + (x30 * x35);
	const GEN_FLT x131 = ((x56 * x128) + (-1 * x57 * x130)) * x62;
	const GEN_FLT x132 = (x66 * x129) + (x65 * x43) + (x64 * x127);
	const GEN_FLT x133 = (x104 * x128) + (x105 * x130);
	const GEN_FLT x134 = x101 * ((-1 * x108 * ((x103 * x132) + x133)) + (x73 * x132));
	const GEN_FLT x135 = x76 * x134;
	const GEN_FLT x136 = (x74 * ((-1 * x75 * x134) + x135)) + (x77 * x134);
	const GEN_FLT x137 = (x78 * x134) + (x74 * x136);
	const GEN_FLT x138 = (-1 * x114 * x133) + (x88 * x132);
	const GEN_FLT x139 = x131 + (-1 * x116 * x138);
	const GEN_FLT x140 = x119 * x117;
	const GEN_FLT x141 = x117 * x122;
	const GEN_FLT x142 =
		x100 * ((x98 * x137) + (x113 * x134) +
				(-1 * x121 *
				 ((-1 * x139 * x140) +
				  (-1 * x93 *
				   ((x74 * ((x74 * ((x74 * (x135 + (-1 * x120 * x134) + (x81 * x134))) + (x82 * x134) + x136)) +
							(x83 * x134) + x137)) +
					(x84 * x134) + (x74 * x137) + (x79 * x134))))) +
				(x139 * x141) + x138);
	const GEN_FLT x143 = x41 * x20;
	const GEN_FLT x144 = x5 * x31;
	const GEN_FLT x145 = (x48 * x126) + (x16 * x143) + (x24 * x144);
	const GEN_FLT x146 = x48 * x20;
	const GEN_FLT x147 = (x47 * x146) + (x40 * x143) + (x30 * x31);
	const GEN_FLT x148 = ((x56 * x145) + (-1 * x57 * x147)) * x62;
	const GEN_FLT x149 = (x66 * x146) + (x65 * x42) + (x64 * x144);
	const GEN_FLT x150 = (x104 * x145) + (x105 * x147);
	const GEN_FLT x151 = x101 * ((-1 * x108 * ((x103 * x149) + x150)) + (x73 * x149));
	const GEN_FLT x152 = x76 * x151;
	const GEN_FLT x153 = (x74 * ((-1 * x75 * x151) + x152)) + (x77 * x151);
	const GEN_FLT x154 = (x78 * x151) + (x74 * x153);
	const GEN_FLT x155 = (-1 * x114 * x150) + (x88 * x149);
	const GEN_FLT x156 = x148 + (-1 * x116 * x155);
	const GEN_FLT x157 =
		x100 * ((x98 * x154) +
				(-1 * x121 *
				 ((-1 * x140 * x156) +
				  (-1 * x93 *
				   ((x74 * ((x74 * ((x74 * (x152 + (-1 * x120 * x151) + (x81 * x151))) + (x82 * x151) + x153)) +
							(x83 * x151) + x154)) +
					(x84 * x151) + (x74 * x154) + (x79 * x151))))) +
				(x141 * x156) + (x113 * x151) + x155);
	out[0] = x63 + (-1 * x123) + (-1 * x124 * ((-1 * x63) + x123));
	out[1] = x131 + (-1 * x142) + (-1 * ((-1 * x131) + x142) * x124);
	out[2] = x148 + (-1 * ((-1 * x148) + x157) * x124) + (-1 * x157);
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
	const GEN_FLT x0 = lh_qj * lh_qj;
	const GEN_FLT x1 = lh_qi * lh_qi;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = lh_qk * lh_qk;
	const GEN_FLT x4 = x3 + x1;
	const GEN_FLT x5 = sqrt((x0 + (lh_qw * lh_qw) + x4));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = obj_qk * obj_qk;
	const GEN_FLT x10 = x9 + x8;
	const GEN_FLT x11 = 2 * sqrt((x7 + (obj_qw * obj_qw) + x10));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 =
		obj_pz + ((1 + (-1 * (x7 + x8) * x11)) * sensor_z) + ((x12 + x13) * x14) + (((-1 * x15) + x16) * x17);
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = (((-1 * x12) + x13) * x22) + obj_py + ((1 + (-1 * x11 * x10)) * sensor_y) + ((x23 + x24) * x17);
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = (-1 * x27) + x28;
	const GEN_FLT x30 =
		obj_px + ((x15 + x16) * x22) + (((-1 * x23) + x24) * x14) + ((1 + (-1 * (x9 + x7) * x11)) * sensor_x);
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + (x18 * (1 + (-1 * x2 * x6))) + (x21 * x26) + (x31 * x29);
	const GEN_FLT x33 = x27 + x28;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = (-1 * x35) + x36;
	const GEN_FLT x38 = x3 + x0;
	const GEN_FLT x39 = lh_px + (x34 * x33) + (x37 * x26) + (x30 * (1 + (-1 * x6 * x38)));
	const GEN_FLT x40 = x39 * x39;
	const GEN_FLT x41 = (x32 * x32) + x40;
	const GEN_FLT x42 = pow(x41, -1);
	const GEN_FLT x43 = x42 * x32;
	const GEN_FLT x44 = (-1 * x19) + x20;
	const GEN_FLT x45 = x35 + x36;
	const GEN_FLT x46 = lh_py + (x44 * x34) + (x25 * (1 + (-1 * x4 * x6))) + (x45 * x31);
	const GEN_FLT x47 = 0.523598775598299 + tilt_0;
	const GEN_FLT x48 = cos(x47);
	const GEN_FLT x49 = pow(x48, -1);
	const GEN_FLT x50 = x46 * x46;
	const GEN_FLT x51 = x50 + x41;
	const GEN_FLT x52 = pow(x51, -1.0 / 2.0) * x49;
	const GEN_FLT x53 = asin(x52 * x46);
	const GEN_FLT x54 = 8.0108022e-06 * x53;
	const GEN_FLT x55 = -8.0108022e-06 + (-1 * x54);
	const GEN_FLT x56 = 0.0028679863 + (x53 * x55);
	const GEN_FLT x57 = 5.3685255e-06 + (x53 * x56);
	const GEN_FLT x58 = 0.0076069798 + (x53 * x57);
	const GEN_FLT x59 = x53 * x58;
	const GEN_FLT x60 = -8.0108022e-06 + (-1.60216044e-05 * x53);
	const GEN_FLT x61 = (x60 * x53) + x56;
	const GEN_FLT x62 = (x61 * x53) + x57;
	const GEN_FLT x63 = (x62 * x53) + x58;
	const GEN_FLT x64 = x59 + (x63 * x53);
	const GEN_FLT x65 = sin(x47);
	const GEN_FLT x66 = tan(x47);
	const GEN_FLT x67 = x66 * pow(x41, -1.0 / 2.0);
	const GEN_FLT x68 = x67 * x46;
	const GEN_FLT x69 = atan2(-1 * x32, x39);
	const GEN_FLT x70 = ogeeMag_0 + (-1 * asin(x68)) + x69;
	const GEN_FLT x71 = curve_0 + (sin(x70) * ogeePhase_0);
	const GEN_FLT x72 = x71 * x65;
	const GEN_FLT x73 = (-1 * x72 * x64) + x48;
	const GEN_FLT x74 = pow(x73, -1);
	const GEN_FLT x75 = x53 * x53;
	const GEN_FLT x76 = x71 * x75;
	const GEN_FLT x77 = x74 * x76;
	const GEN_FLT x78 = (x77 * x58) + x68;
	const GEN_FLT x79 = pow((1 + (-1 * (x78 * x78))), -1.0 / 2.0);
	const GEN_FLT x80 = x66 * pow(x41, -3.0 / 2.0);
	const GEN_FLT x81 = x80 * x46;
	const GEN_FLT x82 = x81 * x39;
	const GEN_FLT x83 = pow((1 + (-1 * x50 * pow(x51, -1) * pow(x48, -2))), -1.0 / 2.0);
	const GEN_FLT x84 = pow(x51, -3.0 / 2.0) * x49;
	const GEN_FLT x85 = x84 * x46;
	const GEN_FLT x86 = x85 * x39;
	const GEN_FLT x87 = x83 * x86;
	const GEN_FLT x88 = -1 * x87 * x55;
	const GEN_FLT x89 = (x53 * ((x87 * x54) + x88)) + (-1 * x87 * x56);
	const GEN_FLT x90 = (-1 * x87 * x57) + (x89 * x53);
	const GEN_FLT x91 = pow((1 + (-1 * (x66 * x66) * x50 * x42)), -1.0 / 2.0);
	const GEN_FLT x92 = cos(x70) * ogeePhase_0;
	const GEN_FLT x93 = x92 * (x43 + (x82 * x91));
	const GEN_FLT x94 = x64 * x65;
	const GEN_FLT x95 = 2.40324066e-05 * x53;
	const GEN_FLT x96 = x83 * x60;
	const GEN_FLT x97 = x83 * x62;
	const GEN_FLT x98 = x83 * x58;
	const GEN_FLT x99 = pow(x73, -2) * x76 * x58;
	const GEN_FLT x100 = x75 * x74 * x58;
	const GEN_FLT x101 = 2 * x39;
	const GEN_FLT x102 = x71 * x74 * x59;
	const GEN_FLT x103 = x83 * x85 * x102;
	const GEN_FLT x104 =
		x79 * ((-1 * x82) +
			   (-1 * x99 *
				((-1 * x93 * x94) +
				 (-1 * x72 *
				  ((x53 * ((x53 * ((x53 * (x88 + (x87 * x95) + (-1 * x86 * x96))) + (-1 * x87 * x61) + x89)) +
						   (-1 * x86 * x97) + x90)) +
				   (-1 * x86 * x98) + (-1 * x87 * x63) + (x53 * x90))))) +
			   (x77 * x90) + (x93 * x100) + (-1 * x101 * x103));
	const GEN_FLT x105 = cos(gibPhase_0 + (-1 * asin(x78)) + x69) * gibMag_0;
	const GEN_FLT x106 = x83 * ((-1 * x84 * x50) + x52);
	const GEN_FLT x107 = x55 * x106;
	const GEN_FLT x108 = (x53 * ((-1 * x54 * x106) + x107)) + (x56 * x106);
	const GEN_FLT x109 = (x57 * x106) + (x53 * x108);
	const GEN_FLT x110 = x92 * x94;
	const GEN_FLT x111 = x67 * x91;
	const GEN_FLT x112 = x92 * x100;
	const GEN_FLT x113 = 2 * x102;
	const GEN_FLT x114 =
		x79 *
		(x67 + (x77 * x109) +
		 (-1 * x99 *
		  ((x110 * x111) + (-1 * x72 *
							((x53 * ((x53 * ((x53 * (x107 + (-1 * x95 * x106) + (x60 * x106))) + (x61 * x106) + x108)) +
									 (x62 * x106) + x109)) +
							 (x63 * x106) + (x53 * x109) + (x58 * x106))))) +
		 (-1 * x111 * x112) + (x106 * x113));
	const GEN_FLT x115 = x42 * x39;
	const GEN_FLT x116 = -1 * x115;
	const GEN_FLT x117 = x46 * x32;
	const GEN_FLT x118 = x84 * x117;
	const GEN_FLT x119 = x83 * x118;
	const GEN_FLT x120 = -1 * x55 * x119;
	const GEN_FLT x121 = (x53 * ((x54 * x119) + x120)) + (-1 * x56 * x119);
	const GEN_FLT x122 = (-1 * x57 * x119) + (x53 * x121);
	const GEN_FLT x123 = x80 * x117;
	const GEN_FLT x124 = x116 + (x91 * x123);
	const GEN_FLT x125 = 2 * x32;
	const GEN_FLT x126 =
		x79 * ((x77 * x122) + (-1 * x123) +
			   (-1 * x99 *
				((-1 * x110 * x124) +
				 (-1 * x72 *
				  ((-1 * x63 * x119) +
				   (x53 * ((x53 * ((x53 * (x120 + (x95 * x119) + (-1 * x96 * x118))) + (-1 * x61 * x119) + x121)) +
						   (-1 * x97 * x118) + x122)) +
				   (x53 * x122) + (-1 * x98 * x118))))) +
			   (x112 * x124) + (-1 * x103 * x125));
	const GEN_FLT x127 = 2 * pow(x5, -1);
	const GEN_FLT x128 = x38 * x127;
	const GEN_FLT x129 = x30 * x128;
	const GEN_FLT x130 = x127 * lh_qw;
	const GEN_FLT x131 = x37 * x25;
	const GEN_FLT x132 = x26 * lh_qk;
	const GEN_FLT x133 = x33 * x18;
	const GEN_FLT x134 = x34 * lh_qj;
	const GEN_FLT x135 = (-1 * x129 * lh_qw) + (x130 * x131) + (-1 * x132) + (x130 * x133) + x134;
	const GEN_FLT x136 = pow(x40, -1) * x32;
	const GEN_FLT x137 = pow(x39, -1);
	const GEN_FLT x138 = x30 * x29;
	const GEN_FLT x139 = x25 * x21;
	const GEN_FLT x140 = x26 * lh_qi;
	const GEN_FLT x141 = x31 * lh_qj;
	const GEN_FLT x142 = x2 * x127;
	const GEN_FLT x143 = x18 * x142;
	const GEN_FLT x144 = (x130 * x138) + (x130 * x139) + x140 + (-1 * x141) + (-1 * x143 * lh_qw);
	const GEN_FLT x145 = x40 * x42;
	const GEN_FLT x146 = ((x135 * x136) + (-1 * x137 * x144)) * x145;
	const GEN_FLT x147 = x45 * x30;
	const GEN_FLT x148 = x127 * x147;
	const GEN_FLT x149 = x31 * lh_qk;
	const GEN_FLT x150 = x4 * x127;
	const GEN_FLT x151 = x25 * x150;
	const GEN_FLT x152 = x44 * x18;
	const GEN_FLT x153 = x34 * lh_qi;
	const GEN_FLT x154 = (x148 * lh_qw) + x149 + (-1 * x151 * lh_qw) + (x130 * x152) + (-1 * x153);
	const GEN_FLT x155 = 2 * x46;
	const GEN_FLT x156 = (x101 * x135) + (x125 * x144);
	const GEN_FLT x157 = 1.0 / 2.0 * x85;
	const GEN_FLT x158 = (-1 * x157 * ((x154 * x155) + x156)) + (x52 * x154);
	const GEN_FLT x159 = x83 * x158;
	const GEN_FLT x160 = x55 * x159;
	const GEN_FLT x161 = (x53 * ((-1 * x54 * x159) + x160)) + (x56 * x159);
	const GEN_FLT x162 = (x57 * x159) + (x53 * x161);
	const GEN_FLT x163 = 1.0 / 2.0 * x81;
	const GEN_FLT x164 = (-1 * x163 * x156) + (x67 * x154);
	const GEN_FLT x165 = x146 + (-1 * x91 * x164);
	const GEN_FLT x166 =
		x79 * ((x77 * x162) + x164 + (x112 * x165) + (x113 * x159) +
			   (-1 * x99 *
				((-1 * x110 * x165) +
				 (-1 * x72 *
				  ((x53 * ((x53 * ((x53 * (x160 + (-1 * x95 * x159) + (x60 * x159))) + (x61 * x159) + x161)) +
						   (x97 * x158) + x162)) +
				   (x53 * x162) + (x63 * x159) + (x98 * x158))))));
	const GEN_FLT x167 = x127 * lh_qi;
	const GEN_FLT x168 = x26 * lh_qj;
	const GEN_FLT x169 = x34 * lh_qk;
	const GEN_FLT x170 = (-1 * x129 * lh_qi) + (x167 * x131) + x168 + (x167 * x133) + x169;
	const GEN_FLT x171 = x26 * lh_qw;
	const GEN_FLT x172 = 4 * x5;
	const GEN_FLT x173 = -1 * x172 * lh_qi;
	const GEN_FLT x174 = (x167 * x138) + x171 + x149 + (x167 * x139) + (x18 * ((-1 * x142 * lh_qi) + x173));
	const GEN_FLT x175 = ((x170 * x136) + (-1 * x174 * x137)) * x145;
	const GEN_FLT x176 = x34 * lh_qw;
	const GEN_FLT x177 = (x148 * lh_qi) + (x25 * ((-1 * x150 * lh_qi) + x173)) + x141 + (-1 * x176) + (x167 * x152);
	const GEN_FLT x178 = (x101 * x170) + (x125 * x174);
	const GEN_FLT x179 = (-1 * x157 * ((x177 * x155) + x178)) + (x52 * x177);
	const GEN_FLT x180 = x83 * x179;
	const GEN_FLT x181 = x55 * x180;
	const GEN_FLT x182 = (x53 * ((-1 * x54 * x180) + x181)) + (x56 * x180);
	const GEN_FLT x183 = (x57 * x180) + (x53 * x182);
	const GEN_FLT x184 = (-1 * x163 * x178) + (x67 * x177);
	const GEN_FLT x185 = x175 + (-1 * x91 * x184);
	const GEN_FLT x186 =
		x79 * ((x77 * x183) + (x113 * x180) +
			   (-1 * x99 *
				((-1 * x110 * x185) +
				 (-1 * x72 *
				  ((x53 * ((x53 * ((x53 * (x181 + (-1 * x95 * x180) + (x60 * x180))) + (x61 * x180) + x182)) +
						   (x97 * x179) + x183)) +
				   (x63 * x180) + (x53 * x183) + (x98 * x179))))) +
			   (x112 * x185) + x184);
	const GEN_FLT x187 = -1 * x172 * lh_qj;
	const GEN_FLT x188 = x127 * lh_qj;
	const GEN_FLT x189 = (x30 * ((-1 * x128 * lh_qj) + x187)) + (x188 * x131) + x140 + (x188 * x133) + x176;
	const GEN_FLT x190 = x31 * lh_qw;
	const GEN_FLT x191 = (x188 * x138) + (x18 * ((-1 * x142 * lh_qj) + x187)) + (-1 * x190) + (x188 * x139) + x132;
	const GEN_FLT x192 = ((x189 * x136) + (-1 * x191 * x137)) * x145;
	const GEN_FLT x193 = x31 * lh_qi;
	const GEN_FLT x194 = (x148 * lh_qj) + x169 + x193 + (-1 * x151 * lh_qj) + (x188 * x152);
	const GEN_FLT x195 = (x101 * x189) + (x125 * x191);
	const GEN_FLT x196 = (-1 * x157 * ((x194 * x155) + x195)) + (x52 * x194);
	const GEN_FLT x197 = x83 * x196;
	const GEN_FLT x198 = x55 * x197;
	const GEN_FLT x199 = (x53 * ((-1 * x54 * x197) + x198)) + (x56 * x197);
	const GEN_FLT x200 = (x57 * x197) + (x53 * x199);
	const GEN_FLT x201 = (-1 * x163 * x195) + (x67 * x194);
	const GEN_FLT x202 = x192 + (-1 * x91 * x201);
	const GEN_FLT x203 =
		x79 * ((x77 * x200) +
			   (-1 * x99 *
				((-1 * x202 * x110) +
				 (-1 * x72 *
				  ((x53 * ((x53 * ((x53 * (x198 + (-1 * x95 * x197) + (x60 * x197))) + x199 + (x61 * x197))) + x200 +
						   (x97 * x196))) +
				   (x63 * x197) + (x53 * x200) + (x98 * x196))))) +
			   (x113 * x197) + (x202 * x112) + x201);
	const GEN_FLT x204 = -1 * x172 * lh_qk;
	const GEN_FLT x205 = x127 * lh_qk;
	const GEN_FLT x206 = x18 * x205;
	const GEN_FLT x207 = (x30 * ((-1 * x128 * lh_qk) + x204)) + (-1 * x171) + (x205 * x131) + (x33 * x206) + x153;
	const GEN_FLT x208 = (x205 * x138) + (x205 * x139) + (-1 * x143 * lh_qk) + x193 + x168;
	const GEN_FLT x209 = ((x207 * x136) + (-1 * x208 * x137)) * x145;
	const GEN_FLT x210 = x190 + (x25 * ((-1 * x150 * lh_qk) + x204)) + (x205 * x147) + (x44 * x206) + x134;
	const GEN_FLT x211 = (x207 * x101) + (x208 * x125);
	const GEN_FLT x212 = (-1 * x157 * ((x210 * x155) + x211)) + (x52 * x210);
	const GEN_FLT x213 = x83 * x212;
	const GEN_FLT x214 = x55 * x213;
	const GEN_FLT x215 = (x53 * ((-1 * x54 * x213) + x214)) + (x56 * x213);
	const GEN_FLT x216 = (x57 * x213) + (x53 * x215);
	const GEN_FLT x217 = (-1 * x211 * x163) + (x67 * x210);
	const GEN_FLT x218 = x209 + (-1 * x91 * x217);
	const GEN_FLT x219 =
		x79 * ((x77 * x216) +
			   (-1 * x99 *
				((-1 * x218 * x110) +
				 (-1 * x72 *
				  ((x53 * ((x53 * ((x53 * (x214 + (-1 * x95 * x213) + (x60 * x213))) + (x61 * x213) + x215)) +
						   (x97 * x212) + x216)) +
				   (x63 * x213) + (x53 * x216) + (x98 * x212))))) +
			   (x218 * x112) + x217 + (x213 * x113));
	out[0] = x43 + (-1 * x104) + (-1 * x105 * ((-1 * x43) + x104));
	out[1] = (-1 * x114) + (-1 * x105 * x114);
	out[2] = x116 + (-1 * x126) + (-1 * (x115 + x126) * x105);
	out[3] = x146 + (-1 * x166) + (-1 * ((-1 * x146) + x166) * x105);
	out[4] = x175 + (-1 * x186) + (-1 * ((-1 * x175) + x186) * x105);
	out[5] = x192 + (-1 * x203) + (-1 * ((-1 * x192) + x203) * x105);
	out[6] = x209 + (-1 * x219) + (-1 * ((-1 * x209) + x219) * x105);
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
	const GEN_FLT x4 = obj_qj * obj_qj;
	const GEN_FLT x5 = obj_qi * obj_qi;
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = 2 * sqrt((x7 + (obj_qw * obj_qw) + x6));
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = obj_pz + ((1 + (-1 * x6 * x8)) * sensor_z) + (x11 * (x9 + x10)) + (((-1 * x12) + x13) * x14);
	const GEN_FLT x16 = lh_qi * lh_qi;
	const GEN_FLT x17 = lh_qk * lh_qk;
	const GEN_FLT x18 = lh_qj * lh_qj;
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 2 * sqrt((x16 + (lh_qw * lh_qw) + x19));
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 =
		(x22 * ((-1 * x9) + x10)) + ((x23 + x24) * x14) + obj_py + ((1 + (-1 * (x7 + x5) * x8)) * sensor_y);
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 =
		obj_px + ((x12 + x13) * x22) + (((-1 * x23) + x24) * x11) + ((1 + (-1 * (x7 + x4) * x8)) * sensor_x);
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = lh_py + (((-1 * x2) + x3) * x21) + ((x26 + x27) * x29) + (x25 * (1 + (-1 * (x17 + x16) * x20)));
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = lh_pz + (x15 * (1 + (-1 * (x18 + x16) * x20))) + ((x2 + x3) * x31) + (((-1 * x32) + x33) * x29);
	const GEN_FLT x35 = lh_px + ((x32 + x33) * x21) + (((-1 * x26) + x27) * x31) + (x28 * (1 + (-1 * x20 * x19)));
	const GEN_FLT x36 = (x34 * x34) + (x35 * x35);
	const GEN_FLT x37 = x30 * pow(x36, -1.0 / 2.0);
	const GEN_FLT x38 = x1 * x37;
	const GEN_FLT x39 = atan2(-1 * x34, x35);
	const GEN_FLT x40 = ogeeMag_0 + (-1 * asin(x38)) + x39;
	const GEN_FLT x41 = sin(x40);
	const GEN_FLT x42 = curve_0 + (x41 * ogeePhase_0);
	const GEN_FLT x43 = cos(x0);
	const GEN_FLT x44 = x30 * x30;
	const GEN_FLT x45 = x44 + x36;
	const GEN_FLT x46 = pow(x45, -1.0 / 2.0) * x30;
	const GEN_FLT x47 = asin(pow(x43, -1) * x46);
	const GEN_FLT x48 = 8.0108022e-06 * x47;
	const GEN_FLT x49 = -8.0108022e-06 + (-1 * x48);
	const GEN_FLT x50 = 0.0028679863 + (x47 * x49);
	const GEN_FLT x51 = 5.3685255e-06 + (x50 * x47);
	const GEN_FLT x52 = 0.0076069798 + (x51 * x47);
	const GEN_FLT x53 = sin(x0);
	const GEN_FLT x54 = x52 * x47;
	const GEN_FLT x55 = -8.0108022e-06 + (-1.60216044e-05 * x47);
	const GEN_FLT x56 = (x55 * x47) + x50;
	const GEN_FLT x57 = (x56 * x47) + x51;
	const GEN_FLT x58 = (x57 * x47) + x52;
	const GEN_FLT x59 = x54 + (x58 * x47);
	const GEN_FLT x60 = x59 * x42;
	const GEN_FLT x61 = x60 * x53;
	const GEN_FLT x62 = (-1 * x61) + x43;
	const GEN_FLT x63 = pow(x62, -1);
	const GEN_FLT x64 = x47 * x47;
	const GEN_FLT x65 = x63 * x64;
	const GEN_FLT x66 = x65 * x52;
	const GEN_FLT x67 = (x66 * x42) + x38;
	const GEN_FLT x68 = pow((1 + (-1 * (x67 * x67))), -1.0 / 2.0);
	const GEN_FLT x69 = x1 * x1;
	const GEN_FLT x70 = x37 * (1 + x69);
	const GEN_FLT x71 = pow(x43, -2);
	const GEN_FLT x72 = x71 * x46 * pow((1 + (-1 * x71 * x44 * pow(x45, -1))), -1.0 / 2.0);
	const GEN_FLT x73 = x72 * x53;
	const GEN_FLT x74 = x73 * x49;
	const GEN_FLT x75 = (x47 * ((-1 * x73 * x48) + x74)) + (x73 * x50);
	const GEN_FLT x76 = (x73 * x51) + (x75 * x47);
	const GEN_FLT x77 = cos(x40) * ogeePhase_0;
	const GEN_FLT x78 = x70 * pow((1 + (-1 * x69 * x44 * pow(x36, -1))), -1.0 / 2.0);
	const GEN_FLT x79 = x53 * x42;
	const GEN_FLT x80 = x64 * pow(x62, -2) * x52;
	const GEN_FLT x81 = x77 * x66;
	const GEN_FLT x82 =
		x68 * (x70 + (x76 * x65 * x42) +
			   (-1 * x80 * x42 *
				((-1 * x53) + (-1 * x60 * x43) + (x78 * x77 * x53 * x59) +
				 (-1 * x79 *
				  ((x47 * ((x47 * ((x47 * (x74 + (-2.40324066e-05 * x73 * x47) + (x73 * x55))) + (x73 * x56) + x75)) +
						   (x73 * x57) + x76)) +
				   (x73 * x58) + (x76 * x47) + (x73 * x52))))) +
			   (-1 * x81 * x78) + (2 * x72 * x79 * x63 * x54));
	const GEN_FLT x83 = (-1 * gibPhase_0) + asin(x67) + (-1 * x39);
	const GEN_FLT x84 = cos(x83) * gibMag_0;
	const GEN_FLT x85 = x80 * x61;
	const GEN_FLT x86 = (x85 + x66) * x68;
	const GEN_FLT x87 = x68 * ((x85 * x77) + x81);
	const GEN_FLT x88 = ((x85 * x41) + (x66 * x41)) * x68;
	out[0] = -1;
	out[1] = (-1 * x82) + (-1 * x82 * x84);
	out[2] = (-1 * x86) + (-1 * x84 * x86);
	out[3] = x84;
	out[4] = -1 * sin(x83);
	out[5] = (-1 * x87) + (-1 * x84 * x87);
	out[6] = (-1 * x88) + (-1 * x88 * x84);
}

/** Applying function <function reproject_axis_y_gen2 at 0x7f35a4af89e0> */
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
	const GEN_FLT x0 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x1 = lh_qw * lh_qi;
	const GEN_FLT x2 = lh_qk * lh_qj;
	const GEN_FLT x3 = obj_qj * obj_qj;
	const GEN_FLT x4 = obj_qi * obj_qi;
	const GEN_FLT x5 = obj_qk * obj_qk;
	const GEN_FLT x6 = x5 + x4;
	const GEN_FLT x7 = 2 * sqrt((x3 + (obj_qw * obj_qw) + x6));
	const GEN_FLT x8 = obj_qw * obj_qi;
	const GEN_FLT x9 = obj_qk * obj_qj;
	const GEN_FLT x10 = x7 * sensor_y;
	const GEN_FLT x11 = obj_qw * obj_qj;
	const GEN_FLT x12 = obj_qk * obj_qi;
	const GEN_FLT x13 = x7 * sensor_x;
	const GEN_FLT x14 =
		obj_pz + ((1 + (-1 * (x3 + x4) * x7)) * sensor_z) + ((x8 + x9) * x10) + (((-1 * x11) + x12) * x13);
	const GEN_FLT x15 = lh_qk * lh_qk;
	const GEN_FLT x16 = lh_qj * lh_qj;
	const GEN_FLT x17 = lh_qi * lh_qi;
	const GEN_FLT x18 = x16 + x17;
	const GEN_FLT x19 = 2 * sqrt((x15 + (lh_qw * lh_qw) + x18));
	const GEN_FLT x20 = x14 * x19;
	const GEN_FLT x21 = x7 * sensor_z;
	const GEN_FLT x22 = obj_qw * obj_qk;
	const GEN_FLT x23 = obj_qj * obj_qi;
	const GEN_FLT x24 = (((-1 * x8) + x9) * x21) + obj_py + ((1 + (-1 * x6 * x7)) * sensor_y) + ((x22 + x23) * x13);
	const GEN_FLT x25 = lh_qw * lh_qk;
	const GEN_FLT x26 = lh_qj * lh_qi;
	const GEN_FLT x27 =
		obj_px + (((-1 * x22) + x23) * x10) + ((x11 + x12) * x21) + ((1 + (-1 * (x5 + x3) * x7)) * sensor_x);
	const GEN_FLT x28 = x27 * x19;
	const GEN_FLT x29 = lh_py + (((-1 * x1) + x2) * x20) + (x24 * (1 + (-1 * (x15 + x17) * x19))) + ((x25 + x26) * x28);
	const GEN_FLT x30 = x24 * x19;
	const GEN_FLT x31 = lh_qw * lh_qj;
	const GEN_FLT x32 = lh_qk * lh_qi;
	const GEN_FLT x33 = lh_pz + (x14 * (1 + (-1 * x19 * x18))) + (((-1 * x31) + x32) * x28) + ((x1 + x2) * x30);
	const GEN_FLT x34 =
		lh_px + (((-1 * x25) + x26) * x30) + ((x31 + x32) * x20) + (x27 * (1 + (-1 * (x15 + x16) * x19)));
	const GEN_FLT x35 = (x33 * x33) + (x34 * x34);
	const GEN_FLT x36 = -1 * tan(x0) * pow(x35, -1.0 / 2.0) * x29;
	const GEN_FLT x37 = atan2(-1 * x33, x34);
	const GEN_FLT x38 = curve_1 + (sin(ogeeMag_1 + (-1 * asin(x36)) + x37) * ogeePhase_1);
	const GEN_FLT x39 = cos(x0);
	const GEN_FLT x40 = asin(pow(x39, -1) * x29 * pow(((x29 * x29) + x35), -1.0 / 2.0));
	const GEN_FLT x41 = 0.0028679863 + (x40 * (-8.0108022e-06 + (-8.0108022e-06 * x40)));
	const GEN_FLT x42 = 5.3685255e-06 + (x40 * x41);
	const GEN_FLT x43 = 0.0076069798 + (x40 * x42);
	const GEN_FLT x44 = asin(
		((x40 * x40) * x43 * x38 *
		 pow(((sin(x0) * x38 *
			   ((x40 * x43) +
				(x40 * ((x40 * ((x40 * ((x40 * (-8.0108022e-06 + (-1.60216044e-05 * x40))) + x41)) + x42)) + x43)))) +
			  x39),
			 -1)) +
		x36);
	out[0] = -1.5707963267949 + (-1 * phase_1) + (-1 * sin((-1 * gibPhase_1) + x44 + (-1 * x37)) * gibMag_1) + x37 +
			 (-1 * x44);
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
	const GEN_FLT x0 = lh_qk * lh_qk;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = lh_qi * lh_qi;
	const GEN_FLT x4 = sqrt((x3 + (lh_qw * lh_qw) + x2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * x2 * x5);
	const GEN_FLT x7 = 1 + (-1 * (x1 + x3) * x5);
	const GEN_FLT x8 = obj_qj * obj_qj;
	const GEN_FLT x9 = obj_qi * obj_qi;
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = obj_qk * obj_qk;
	const GEN_FLT x12 = x11 + x8;
	const GEN_FLT x13 = sqrt((x9 + (obj_qw * obj_qw) + x12));
	const GEN_FLT x14 = 2 * x13;
	const GEN_FLT x15 = obj_qw * obj_qi;
	const GEN_FLT x16 = obj_qk * obj_qj;
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = x14 * sensor_y;
	const GEN_FLT x19 = obj_qw * obj_qj;
	const GEN_FLT x20 = obj_qk * obj_qi;
	const GEN_FLT x21 = (-1 * x19) + x20;
	const GEN_FLT x22 = x14 * sensor_x;
	const GEN_FLT x23 = obj_pz + ((1 + (-1 * x14 * x10)) * sensor_z) + (x18 * x17) + (x22 * x21);
	const GEN_FLT x24 = (-1 * x15) + x16;
	const GEN_FLT x25 = x14 * sensor_z;
	const GEN_FLT x26 = x11 + x9;
	const GEN_FLT x27 = obj_qw * obj_qk;
	const GEN_FLT x28 = obj_qj * obj_qi;
	const GEN_FLT x29 = x27 + x28;
	const GEN_FLT x30 = (x24 * x25) + obj_py + ((1 + (-1 * x26 * x14)) * sensor_y) + (x22 * x29);
	const GEN_FLT x31 = lh_qw * lh_qi;
	const GEN_FLT x32 = lh_qk * lh_qj;
	const GEN_FLT x33 = x31 + x32;
	const GEN_FLT x34 = x5 * x33;
	const GEN_FLT x35 = x19 + x20;
	const GEN_FLT x36 = (-1 * x27) + x28;
	const GEN_FLT x37 = obj_px + (x35 * x25) + (x36 * x18) + ((1 + (-1 * x14 * x12)) * sensor_x);
	const GEN_FLT x38 = lh_qw * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qi;
	const GEN_FLT x40 = (-1 * x38) + x39;
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = lh_pz + (x7 * x23) + (x30 * x34) + (x41 * x37);
	const GEN_FLT x43 = x38 + x39;
	const GEN_FLT x44 = x5 * x23;
	const GEN_FLT x45 = lh_qw * lh_qk;
	const GEN_FLT x46 = lh_qj * lh_qi;
	const GEN_FLT x47 = (-1 * x45) + x46;
	const GEN_FLT x48 = x5 * x47;
	const GEN_FLT x49 = lh_px + (x48 * x30) + (x43 * x44) + (x6 * x37);
	const GEN_FLT x50 = x49 * x49;
	const GEN_FLT x51 = pow(x50, -1);
	const GEN_FLT x52 = x51 * x42;
	const GEN_FLT x53 = pow(x49, -1);
	const GEN_FLT x54 = (x42 * x42) + x50;
	const GEN_FLT x55 = pow(x54, -1);
	const GEN_FLT x56 = x50 * x55;
	const GEN_FLT x57 = x56 * ((x6 * x52) + (-1 * x53 * x41));
	const GEN_FLT x58 = (-1 * x31) + x32;
	const GEN_FLT x59 = 1 + (-1 * (x0 + x3) * x5);
	const GEN_FLT x60 = x45 + x46;
	const GEN_FLT x61 = x5 * x60;
	const GEN_FLT x62 = lh_py + (x58 * x44) + (x59 * x30) + (x61 * x37);
	const GEN_FLT x63 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x64 = cos(x63);
	const GEN_FLT x65 = pow(x64, -1);
	const GEN_FLT x66 = x62 * x62;
	const GEN_FLT x67 = x66 + x54;
	const GEN_FLT x68 = pow(x67, -1.0 / 2.0) * x65;
	const GEN_FLT x69 = asin(x62 * x68);
	const GEN_FLT x70 = 8.0108022e-06 * x69;
	const GEN_FLT x71 = -8.0108022e-06 + (-1 * x70);
	const GEN_FLT x72 = 0.0028679863 + (x71 * x69);
	const GEN_FLT x73 = 5.3685255e-06 + (x72 * x69);
	const GEN_FLT x74 = 0.0076069798 + (x73 * x69);
	const GEN_FLT x75 = x74 * x69;
	const GEN_FLT x76 = -8.0108022e-06 + (-1.60216044e-05 * x69);
	const GEN_FLT x77 = (x76 * x69) + x72;
	const GEN_FLT x78 = (x77 * x69) + x73;
	const GEN_FLT x79 = (x78 * x69) + x74;
	const GEN_FLT x80 = x75 + (x79 * x69);
	const GEN_FLT x81 = tan(x63);
	const GEN_FLT x82 = x81 * pow(x54, -1.0 / 2.0);
	const GEN_FLT x83 = -1 * x82 * x62;
	const GEN_FLT x84 = atan2(-1 * x42, x49);
	const GEN_FLT x85 = ogeeMag_1 + (-1 * asin(x83)) + x84;
	const GEN_FLT x86 = curve_1 + (sin(x85) * ogeePhase_1);
	const GEN_FLT x87 = sin(x63);
	const GEN_FLT x88 = x86 * x87;
	const GEN_FLT x89 = (x80 * x88) + x64;
	const GEN_FLT x90 = pow(x89, -1);
	const GEN_FLT x91 = x69 * x69;
	const GEN_FLT x92 = x86 * x91;
	const GEN_FLT x93 = x92 * x90;
	const GEN_FLT x94 = (x74 * x93) + x83;
	const GEN_FLT x95 = pow((1 + (-1 * (x94 * x94))), -1.0 / 2.0);
	const GEN_FLT x96 = pow((1 + (-1 * (x81 * x81) * x66 * x55)), -1.0 / 2.0);
	const GEN_FLT x97 = 2 * x49;
	const GEN_FLT x98 = 4 * x4;
	const GEN_FLT x99 = x98 * x42;
	const GEN_FLT x100 = (x6 * x97) + (x99 * x40);
	const GEN_FLT x101 = 1.0 / 2.0 * x62;
	const GEN_FLT x102 = x81 * pow(x54, -3.0 / 2.0) * x101;
	const GEN_FLT x103 = (x100 * x102) + (-1 * x82 * x61);
	const GEN_FLT x104 = x57 + (-1 * x96 * x103);
	const GEN_FLT x105 = cos(x85) * ogeePhase_1;
	const GEN_FLT x106 = x74 * x91 * x90;
	const GEN_FLT x107 = x105 * x106;
	const GEN_FLT x108 = pow((1 + (-1 * pow(x64, -2) * pow(x67, -1) * x66)), -1.0 / 2.0);
	const GEN_FLT x109 = x62 * x98;
	const GEN_FLT x110 = pow(x67, -3.0 / 2.0) * x65 * x101;
	const GEN_FLT x111 = x108 * ((-1 * x110 * ((x60 * x109) + x100)) + (x61 * x68));
	const GEN_FLT x112 = 2 * x86 * x75 * x90;
	const GEN_FLT x113 = x80 * x87;
	const GEN_FLT x114 = x105 * x113;
	const GEN_FLT x115 = x71 * x111;
	const GEN_FLT x116 = 2.40324066e-05 * x69;
	const GEN_FLT x117 = (x72 * x111) + (x69 * ((-1 * x70 * x111) + x115));
	const GEN_FLT x118 = (x69 * x117) + (x73 * x111);
	const GEN_FLT x119 = pow(x89, -2) * x74 * x92;
	const GEN_FLT x120 =
		x95 * ((x104 * x107) + (x111 * x112) +
			   (-1 * x119 *
				((x104 * x114) +
				 (x88 * ((x69 * ((x69 * ((x69 * (x115 + (-1 * x111 * x116) + (x76 * x111))) + (x77 * x111) + x117)) +
								 x118 + (x78 * x111))) +
						 (x79 * x111) + (x74 * x111) + (x69 * x118))))) +
			   (x93 * x118) + x103);
	const GEN_FLT x121 = cos(gibPhase_1 + (-1 * asin(x94)) + x84) * gibMag_1;
	const GEN_FLT x122 = 2 * x42;
	const GEN_FLT x123 = x4 * x51 * x122;
	const GEN_FLT x124 = x56 * ((x47 * x123) + (-1 * x53 * x34));
	const GEN_FLT x125 = x98 * x49;
	const GEN_FLT x126 = (x47 * x125) + (x99 * x33);
	const GEN_FLT x127 = (x102 * x126) + (-1 * x82 * x59);
	const GEN_FLT x128 = x105 * (x124 + (-1 * x96 * x127));
	const GEN_FLT x129 = 2 * x62;
	const GEN_FLT x130 = x108 * ((-1 * x110 * ((x59 * x129) + x126)) + (x68 * x59));
	const GEN_FLT x131 = x71 * x130;
	const GEN_FLT x132 = (x72 * x130) + (x69 * ((-1 * x70 * x130) + x131));
	const GEN_FLT x133 = (x69 * x132) + (x73 * x130);
	const GEN_FLT x134 =
		x95 * ((x106 * x128) +
			   (-1 * x119 *
				((x113 * x128) +
				 (x88 * ((x69 * ((x69 * ((x69 * ((-1 * x116 * x130) + x131 + (x76 * x130))) + (x77 * x130) + x132)) +
								 (x78 * x130) + x133)) +
						 (x79 * x130) + (x74 * x130) + (x69 * x133))))) +
			   (x112 * x130) + x127 + (x93 * x133));
	const GEN_FLT x135 = x56 * ((x43 * x123) + (-1 * x7 * x53));
	const GEN_FLT x136 = (x43 * x125) + (x7 * x122);
	const GEN_FLT x137 = x5 * x58;
	const GEN_FLT x138 = (x102 * x136) + (-1 * x82 * x137);
	const GEN_FLT x139 = x135 + (-1 * x96 * x138);
	const GEN_FLT x140 = x108 * ((-1 * x110 * ((x58 * x109) + x136)) + (x68 * x137));
	const GEN_FLT x141 = x71 * x140;
	const GEN_FLT x142 = (x72 * x140) + (x69 * ((-1 * x70 * x140) + x141));
	const GEN_FLT x143 = (x69 * x142) + (x73 * x140);
	const GEN_FLT x144 =
		x95 * ((x107 * x139) + (x112 * x140) +
			   (-1 * x119 *
				((x114 * x139) +
				 (x88 * ((x69 * ((x69 * ((x69 * (x141 + (-1 * x116 * x140) + (x76 * x140))) + (x77 * x140) + x142)) +
								 x143 + (x78 * x140))) +
						 (x79 * x140) + (x74 * x140) + (x69 * x143))))) +
			   (x93 * x143) + x138);
	const GEN_FLT x145 = 2 * pow(x13, -1);
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
	const GEN_FLT x156 = (-1 * x147 * obj_qw) + (x149 * obj_qw) + x155 + (-1 * x151) + (x153 * obj_qw);
	const GEN_FLT x157 = x29 * sensor_x;
	const GEN_FLT x158 = x145 * x157;
	const GEN_FLT x159 = x26 * x145;
	const GEN_FLT x160 = x159 * sensor_y;
	const GEN_FLT x161 = x22 * obj_qk;
	const GEN_FLT x162 = x24 * sensor_z;
	const GEN_FLT x163 = x162 * x145;
	const GEN_FLT x164 = x14 * obj_qi;
	const GEN_FLT x165 = x164 * sensor_z;
	const GEN_FLT x166 = (x158 * obj_qw) + (-1 * x160 * obj_qw) + x161 + (x163 * obj_qw) + (-1 * x165);
	const GEN_FLT x167 = x154 * sensor_x;
	const GEN_FLT x168 = x21 * sensor_x;
	const GEN_FLT x169 = x168 * x145;
	const GEN_FLT x170 = x17 * sensor_y;
	const GEN_FLT x171 = x170 * x145;
	const GEN_FLT x172 = x164 * sensor_y;
	const GEN_FLT x173 = x10 * x145;
	const GEN_FLT x174 = x173 * sensor_z;
	const GEN_FLT x175 = (-1 * x167) + (x169 * obj_qw) + (x171 * obj_qw) + x172 + (-1 * x174 * obj_qw);
	const GEN_FLT x176 = x5 * x43;
	const GEN_FLT x177 = (x6 * x156) + (x48 * x166) + (x176 * x175);
	const GEN_FLT x178 = (x41 * x156) + (x34 * x166) + (x7 * x175);
	const GEN_FLT x179 = ((x52 * x177) + (-1 * x53 * x178)) * x56;
	const GEN_FLT x180 = (x97 * x177) + (x122 * x178);
	const GEN_FLT x181 = (x61 * x156) + (x59 * x166) + (x175 * x137);
	const GEN_FLT x182 = (x102 * x180) + (-1 * x82 * x181);
	const GEN_FLT x183 = x179 + (-1 * x96 * x182);
	const GEN_FLT x184 = x108 * ((-1 * x110 * ((x129 * x181) + x180)) + (x68 * x181));
	const GEN_FLT x185 = x71 * x184;
	const GEN_FLT x186 = (x72 * x184) + (x69 * ((-1 * x70 * x184) + x185));
	const GEN_FLT x187 = (x69 * x186) + (x73 * x184);
	const GEN_FLT x188 =
		x95 * ((x107 * x183) + (x93 * x187) + (x112 * x184) +
			   (-1 * x119 *
				((x114 * x183) +
				 (x88 * ((x69 * ((x69 * ((x69 * (x185 + (-1 * x116 * x184) + (x76 * x184))) + (x77 * x184) + x186)) +
								 (x78 * x184) + x187)) +
						 (x69 * x187) + (x79 * x184) + (x74 * x184))))) +
			   x182);
	const GEN_FLT x189 = x154 * sensor_y;
	const GEN_FLT x190 = x150 * sensor_z;
	const GEN_FLT x191 = (-1 * x147 * obj_qi) + (x149 * obj_qi) + x189 + (x153 * obj_qi) + x190;
	const GEN_FLT x192 = 4 * x13;
	const GEN_FLT x193 = -1 * x192 * obj_qi;
	const GEN_FLT x194 = x14 * obj_qw;
	const GEN_FLT x195 = x194 * sensor_z;
	const GEN_FLT x196 =
		(x158 * obj_qi) + x167 + (x163 * obj_qi) + (((-1 * x159 * obj_qi) + x193) * sensor_y) + (-1 * x195);
	const GEN_FLT x197 = x194 * sensor_y;
	const GEN_FLT x198 = (x169 * obj_qi) + x197 + x161 + (x171 * obj_qi) + (((-1 * x173 * obj_qi) + x193) * sensor_z);
	const GEN_FLT x199 = (x6 * x191) + (x48 * x196) + (x176 * x198);
	const GEN_FLT x200 = (x41 * x191) + (x34 * x196) + (x7 * x198);
	const GEN_FLT x201 = ((x52 * x199) + (-1 * x53 * x200)) * x56;
	const GEN_FLT x202 = (x97 * x199) + (x200 * x122);
	const GEN_FLT x203 = (x61 * x191) + (x59 * x196) + (x198 * x137);
	const GEN_FLT x204 = (x202 * x102) + (-1 * x82 * x203);
	const GEN_FLT x205 = x201 + (-1 * x96 * x204);
	const GEN_FLT x206 = x108 * ((-1 * x110 * ((x203 * x129) + x202)) + (x68 * x203));
	const GEN_FLT x207 = x71 * x206;
	const GEN_FLT x208 = (x72 * x206) + (x69 * ((-1 * x70 * x206) + x207));
	const GEN_FLT x209 = (x69 * x208) + (x73 * x206);
	const GEN_FLT x210 =
		x95 * ((x205 * x107) + (x206 * x112) +
			   (-1 * x119 *
				((x205 * x114) +
				 (x88 * ((x79 * x206) + (x74 * x206) + (x69 * x209) +
						 (x69 * ((x69 * ((x69 * (x207 + (-1 * x206 * x116) + (x76 * x206))) + (x77 * x206) + x208)) +
								 (x78 * x206) + x209)))))) +
			   (x93 * x209) + x204);
	const GEN_FLT x211 = -1 * x192 * obj_qj;
	const GEN_FLT x212 = x145 * obj_qj;
	const GEN_FLT x213 = (((-1 * x146 * obj_qj) + x211) * sensor_x) + (x212 * x148) + x195 + x172 + (x212 * x152);
	const GEN_FLT x214 = x22 * obj_qi;
	const GEN_FLT x215 = (x212 * x157) + x214 + (-1 * x160 * obj_qj) + (x212 * x162) + x190;
	const GEN_FLT x216 = x22 * obj_qw;
	const GEN_FLT x217 =
		(x212 * x168) + (-1 * x216) + (x212 * x170) + x151 + (((-1 * x173 * obj_qj) + x211) * sensor_z);
	const GEN_FLT x218 = (x6 * x213) + (x48 * x215) + (x217 * x176);
	const GEN_FLT x219 = (x41 * x213) + (x34 * x215) + (x7 * x217);
	const GEN_FLT x220 = ((x52 * x218) + (-1 * x53 * x219)) * x56;
	const GEN_FLT x221 = (x97 * x218) + (x219 * x122);
	const GEN_FLT x222 = (x61 * x213) + (x59 * x215) + (x217 * x137);
	const GEN_FLT x223 = (x221 * x102) + (-1 * x82 * x222);
	const GEN_FLT x224 = x220 + (-1 * x96 * x223);
	const GEN_FLT x225 = x108 * ((-1 * x110 * ((x222 * x129) + x221)) + (x68 * x222));
	const GEN_FLT x226 = x71 * x225;
	const GEN_FLT x227 = (x72 * x225) + (x69 * ((-1 * x70 * x225) + x226));
	const GEN_FLT x228 = (x69 * x227) + (x73 * x225);
	const GEN_FLT x229 =
		x95 * ((x224 * x107) + x223 + (x225 * x112) +
			   (-1 * x119 *
				((x224 * x114) +
				 (x88 * ((x69 * ((x69 * ((x69 * (x226 + (-1 * x225 * x116) + (x76 * x225))) + (x77 * x225) + x227)) +
								 (x78 * x225) + x228)) +
						 (x79 * x225) + (x74 * x225) + (x69 * x228))))) +
			   (x93 * x228));
	const GEN_FLT x230 = x145 * obj_qk;
	const GEN_FLT x231 = -1 * x192 * obj_qk;
	const GEN_FLT x232 =
		(x230 * x148) + (-1 * x197) + (((-1 * x146 * obj_qk) + x231) * sensor_x) + (x230 * x152) + x165;
	const GEN_FLT x233 = (x230 * x157) + x216 + (((-1 * x159 * obj_qk) + x231) * sensor_y) + (x163 * obj_qk) + x155;
	const GEN_FLT x234 = x5 * x233;
	const GEN_FLT x235 = x214 + (x230 * x170) + (x230 * x168) + x189 + (-1 * x174 * obj_qk);
	const GEN_FLT x236 = (x6 * x232) + (x47 * x234) + (x235 * x176);
	const GEN_FLT x237 = (x41 * x232) + (x7 * x235) + (x33 * x234);
	const GEN_FLT x238 = ((x52 * x236) + (-1 * x53 * x237)) * x56;
	const GEN_FLT x239 = (x97 * x236) + (x237 * x122);
	const GEN_FLT x240 = (x61 * x232) + (x59 * x233) + (x235 * x137);
	const GEN_FLT x241 = (x239 * x102) + (-1 * x82 * x240);
	const GEN_FLT x242 = x238 + (-1 * x96 * x241);
	const GEN_FLT x243 = x108 * ((-1 * x110 * ((x240 * x129) + x239)) + (x68 * x240));
	const GEN_FLT x244 = x71 * x243;
	const GEN_FLT x245 = (x72 * x243) + (x69 * ((-1 * x70 * x243) + x244));
	const GEN_FLT x246 = (x69 * x245) + (x73 * x243);
	const GEN_FLT x247 =
		x95 * ((x242 * x107) + (x243 * x112) +
			   (-1 * x119 *
				((x242 * x114) +
				 (x88 * ((x69 * ((x69 * ((x69 * (x244 + (-1 * x243 * x116) + (x76 * x243))) + (x77 * x243) + x245)) +
								 (x78 * x243) + x246)) +
						 (x79 * x243) + (x74 * x243) + (x69 * x246))))) +
			   (x93 * x246) + x241);
	out[0] = x57 + (-1 * x120) + (-1 * x121 * ((-1 * x57) + x120));
	out[1] = x124 + (-1 * x134) + (-1 * ((-1 * x124) + x134) * x121);
	out[2] = x135 + (-1 * x144) + (-1 * ((-1 * x135) + x144) * x121);
	out[3] = x179 + (-1 * x188) + (-1 * ((-1 * x179) + x188) * x121);
	out[4] = x201 + (-1 * x210) + (-1 * ((-1 * x201) + x210) * x121);
	out[5] = x220 + (-1 * x229) + (-1 * ((-1 * x220) + x229) * x121);
	out[6] = x238 + (-1 * x247) + (-1 * ((-1 * x238) + x247) * x121);
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
	const GEN_FLT x0 = lh_qk * lh_qk;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = lh_qi * lh_qi;
	const GEN_FLT x4 = sqrt((x3 + (lh_qw * lh_qw) + x2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * x2 * x5);
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = obj_qj * obj_qj;
	const GEN_FLT x9 = obj_qi * obj_qi;
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = sqrt((x7 + (obj_qw * obj_qw) + x10));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 + (-1 * (x7 + x8) * x12);
	const GEN_FLT x14 = lh_qw * lh_qk;
	const GEN_FLT x15 = lh_qj * lh_qi;
	const GEN_FLT x16 = (-1 * x14) + x15;
	const GEN_FLT x17 = obj_qw * obj_qk;
	const GEN_FLT x18 = obj_qj * obj_qi;
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 4 * x4 * x11;
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = obj_qw * obj_qj;
	const GEN_FLT x23 = obj_qk * obj_qi;
	const GEN_FLT x24 = (-1 * x22) + x23;
	const GEN_FLT x25 = lh_qw * lh_qj;
	const GEN_FLT x26 = lh_qk * lh_qi;
	const GEN_FLT x27 = x25 + x26;
	const GEN_FLT x28 = x20 * x27;
	const GEN_FLT x29 = (x6 * x13) + (x21 * x16) + (x24 * x28);
	const GEN_FLT x30 = 1 + (-1 * (x1 + x3) * x5);
	const GEN_FLT x31 = 1 + (-1 * x12 * x10);
	const GEN_FLT x32 = obj_qw * obj_qi;
	const GEN_FLT x33 = obj_qk * obj_qj;
	const GEN_FLT x34 = x32 + x33;
	const GEN_FLT x35 = x12 * sensor_y;
	const GEN_FLT x36 = x12 * sensor_x;
	const GEN_FLT x37 = obj_pz + (x31 * sensor_z) + (x34 * x35) + (x36 * x24);
	const GEN_FLT x38 = lh_qw * lh_qi;
	const GEN_FLT x39 = lh_qk * lh_qj;
	const GEN_FLT x40 = x38 + x39;
	const GEN_FLT x41 = (-1 * x32) + x33;
	const GEN_FLT x42 = x12 * sensor_z;
	const GEN_FLT x43 = 1 + (-1 * (x7 + x9) * x12);
	const GEN_FLT x44 = (x41 * x42) + obj_py + (x43 * sensor_y) + (x36 * x19);
	const GEN_FLT x45 = x5 * x44;
	const GEN_FLT x46 = (-1 * x25) + x26;
	const GEN_FLT x47 = x22 + x23;
	const GEN_FLT x48 = (-1 * x17) + x18;
	const GEN_FLT x49 = obj_px + (x42 * x47) + (x48 * x35) + (x13 * sensor_x);
	const GEN_FLT x50 = x5 * x49;
	const GEN_FLT x51 = lh_pz + (x30 * x37) + (x40 * x45) + (x50 * x46);
	const GEN_FLT x52 = x5 * x37;
	const GEN_FLT x53 = lh_px + (x52 * x27) + (x45 * x16) + (x6 * x49);
	const GEN_FLT x54 = x53 * x53;
	const GEN_FLT x55 = pow(x54, -1) * x51;
	const GEN_FLT x56 = pow(x53, -1);
	const GEN_FLT x57 = x5 * x13;
	const GEN_FLT x58 = x30 * x12;
	const GEN_FLT x59 = (x57 * x46) + (x40 * x21) + (x58 * x24);
	const GEN_FLT x60 = (x51 * x51) + x54;
	const GEN_FLT x61 = pow(x60, -1);
	const GEN_FLT x62 = x61 * x54;
	const GEN_FLT x63 = ((x55 * x29) + (-1 * x56 * x59)) * x62;
	const GEN_FLT x64 = (-1 * x38) + x39;
	const GEN_FLT x65 = 1 + (-1 * (x0 + x3) * x5);
	const GEN_FLT x66 = x14 + x15;
	const GEN_FLT x67 = lh_py + (x64 * x52) + (x65 * x44) + (x66 * x50);
	const GEN_FLT x68 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x69 = cos(x68);
	const GEN_FLT x70 = pow(x69, -1);
	const GEN_FLT x71 = x67 * x67;
	const GEN_FLT x72 = x71 + x60;
	const GEN_FLT x73 = x70 * pow(x72, -1.0 / 2.0);
	const GEN_FLT x74 = asin(x73 * x67);
	const GEN_FLT x75 = 8.0108022e-06 * x74;
	const GEN_FLT x76 = -8.0108022e-06 + (-1 * x75);
	const GEN_FLT x77 = 0.0028679863 + (x74 * x76);
	const GEN_FLT x78 = 5.3685255e-06 + (x74 * x77);
	const GEN_FLT x79 = 0.0076069798 + (x78 * x74);
	const GEN_FLT x80 = x79 * x74;
	const GEN_FLT x81 = -8.0108022e-06 + (-1.60216044e-05 * x74);
	const GEN_FLT x82 = (x81 * x74) + x77;
	const GEN_FLT x83 = (x82 * x74) + x78;
	const GEN_FLT x84 = (x83 * x74) + x79;
	const GEN_FLT x85 = x80 + (x84 * x74);
	const GEN_FLT x86 = tan(x68);
	const GEN_FLT x87 = x86 * pow(x60, -1.0 / 2.0);
	const GEN_FLT x88 = -1 * x87 * x67;
	const GEN_FLT x89 = atan2(-1 * x51, x53);
	const GEN_FLT x90 = ogeeMag_1 + (-1 * asin(x88)) + x89;
	const GEN_FLT x91 = curve_1 + (sin(x90) * ogeePhase_1);
	const GEN_FLT x92 = sin(x68);
	const GEN_FLT x93 = x92 * x91;
	const GEN_FLT x94 = (x85 * x93) + x69;
	const GEN_FLT x95 = pow(x94, -1);
	const GEN_FLT x96 = x74 * x74;
	const GEN_FLT x97 = x91 * x96;
	const GEN_FLT x98 = x97 * x95;
	const GEN_FLT x99 = (x79 * x98) + x88;
	const GEN_FLT x100 = pow((1 + (-1 * (x99 * x99))), -1.0 / 2.0);
	const GEN_FLT x101 = 2 * x53;
	const GEN_FLT x102 = 2 * x51;
	const GEN_FLT x103 = (x29 * x101) + (x59 * x102);
	const GEN_FLT x104 = 1.0 / 2.0 * x67;
	const GEN_FLT x105 = x86 * pow(x60, -3.0 / 2.0) * x104;
	const GEN_FLT x106 = x65 * x12;
	const GEN_FLT x107 = x64 * x20;
	const GEN_FLT x108 = (x66 * x57) + (x19 * x106) + (x24 * x107);
	const GEN_FLT x109 = (x103 * x105) + (-1 * x87 * x108);
	const GEN_FLT x110 = pow((1 + (-1 * (x86 * x86) * x71 * x61)), -1.0 / 2.0);
	const GEN_FLT x111 = cos(x90) * ogeePhase_1;
	const GEN_FLT x112 = x111 * (x63 + (-1 * x109 * x110));
	const GEN_FLT x113 = x79 * x96 * x95;
	const GEN_FLT x114 = pow((1 + (-1 * x71 * pow(x72, -1) * pow(x69, -2))), -1.0 / 2.0);
	const GEN_FLT x115 = 2 * x67;
	const GEN_FLT x116 = x70 * pow(x72, -3.0 / 2.0) * x104;
	const GEN_FLT x117 = (-1 * x116 * ((x108 * x115) + x103)) + (x73 * x108);
	const GEN_FLT x118 = x114 * x117;
	const GEN_FLT x119 = 2 * x80 * x91 * x95;
	const GEN_FLT x120 = x85 * x92;
	const GEN_FLT x121 = x76 * x118;
	const GEN_FLT x122 = 2.40324066e-05 * x74;
	const GEN_FLT x123 = (x77 * x118) + (x74 * ((-1 * x75 * x118) + x121));
	const GEN_FLT x124 = (x74 * x123) + (x78 * x118);
	const GEN_FLT x125 = x79 * x114;
	const GEN_FLT x126 = x84 * x114;
	const GEN_FLT x127 = x79 * x97 * pow(x94, -2);
	const GEN_FLT x128 =
		x100 * ((x112 * x113) + (x118 * x119) +
				(-1 * x127 *
				 ((x112 * x120) +
				  (x93 * ((x74 * ((x74 * ((x74 * (x121 + (-1 * x118 * x122) + (x81 * x118))) + (x82 * x118) + x123)) +
								  x124 + (x83 * x118))) +
						  (x117 * x126) + (x74 * x124) + (x117 * x125))))) +
				(x98 * x124) + x109);
	const GEN_FLT x129 = cos(gibPhase_1 + (-1 * asin(x99)) + x89) * gibMag_1;
	const GEN_FLT x130 = x5 * x43;
	const GEN_FLT x131 = x6 * x12;
	const GEN_FLT x132 = (x16 * x130) + (x34 * x28) + (x48 * x131);
	const GEN_FLT x133 = x48 * x20;
	const GEN_FLT x134 = (x46 * x133) + (x40 * x130) + (x58 * x34);
	const GEN_FLT x135 = ((x55 * x132) + (-1 * x56 * x134)) * x62;
	const GEN_FLT x136 = (x101 * x132) + (x102 * x134);
	const GEN_FLT x137 = (x66 * x133) + (x65 * x43) + (x34 * x107);
	const GEN_FLT x138 = (x105 * x136) + (-1 * x87 * x137);
	const GEN_FLT x139 = x135 + (-1 * x110 * x138);
	const GEN_FLT x140 = x111 * x113;
	const GEN_FLT x141 = (-1 * x116 * ((x115 * x137) + x136)) + (x73 * x137);
	const GEN_FLT x142 = x114 * x141;
	const GEN_FLT x143 = x111 * x120;
	const GEN_FLT x144 = x76 * x142;
	const GEN_FLT x145 = (x77 * x142) + (x74 * ((-1 * x75 * x142) + x144));
	const GEN_FLT x146 = (x74 * x145) + (x78 * x142);
	const GEN_FLT x147 =
		x100 * ((x139 * x140) + (x119 * x142) +
				(-1 * x127 *
				 ((x139 * x143) +
				  (x93 * ((x126 * x141) + (x125 * x141) +
						  (x74 * ((x74 * ((x74 * (x144 + (-1 * x122 * x142) + (x81 * x142))) + (x82 * x142) + x145)) +
								  (x83 * x142) + x146)) +
						  (x74 * x146))))) +
				(x98 * x146) + x138);
	const GEN_FLT x148 = x41 * x20;
	const GEN_FLT x149 = x5 * x31;
	const GEN_FLT x150 = (x47 * x131) + (x16 * x148) + (x27 * x149);
	const GEN_FLT x151 = x47 * x20;
	const GEN_FLT x152 = (x46 * x151) + (x40 * x148) + (x30 * x31);
	const GEN_FLT x153 = ((x55 * x150) + (-1 * x56 * x152)) * x62;
	const GEN_FLT x154 = (x101 * x150) + (x102 * x152);
	const GEN_FLT x155 = (x66 * x151) + (x41 * x106) + (x64 * x149);
	const GEN_FLT x156 = (x105 * x154) + (-1 * x87 * x155);
	const GEN_FLT x157 = x153 + (-1 * x110 * x156);
	const GEN_FLT x158 = (-1 * x116 * ((x115 * x155) + x154)) + (x73 * x155);
	const GEN_FLT x159 = x114 * x158;
	const GEN_FLT x160 = x76 * x159;
	const GEN_FLT x161 = (x77 * x159) + (x74 * ((-1 * x75 * x159) + x160));
	const GEN_FLT x162 = (x74 * x161) + (x78 * x159);
	const GEN_FLT x163 =
		x100 * ((x140 * x157) + (x119 * x159) + (x98 * x162) +
				(-1 * x127 *
				 ((x143 * x157) +
				  (x93 * ((x74 * ((x74 * ((x74 * (x160 + (x81 * x159) + (-1 * x122 * x159))) + (x82 * x159) + x161)) +
								  (x83 * x159) + x162)) +
						  (x126 * x158) + (x125 * x158) + (x74 * x162))))) +
				x156);
	out[0] = x63 + (-1 * x128) + (-1 * x129 * ((-1 * x63) + x128));
	out[1] = x135 + (-1 * x147) + (-1 * ((-1 * x135) + x147) * x129);
	out[2] = x153 + (-1 * x163) + (-1 * ((-1 * x153) + x163) * x129);
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
	const GEN_FLT x0 = lh_qj * lh_qj;
	const GEN_FLT x1 = lh_qi * lh_qi;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = lh_qk * lh_qk;
	const GEN_FLT x4 = x3 + x1;
	const GEN_FLT x5 = sqrt((x0 + (lh_qw * lh_qw) + x4));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = x7 + x8;
	const GEN_FLT x10 = obj_qk * obj_qk;
	const GEN_FLT x11 = 2 * sqrt((x10 + (obj_qw * obj_qw) + x9));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 = obj_pz + ((1 + (-1 * x9 * x11)) * sensor_z) + ((x12 + x13) * x14) + (((-1 * x15) + x16) * x17);
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 =
		(((-1 * x12) + x13) * x22) + obj_py + ((1 + (-1 * x11 * (x10 + x8))) * sensor_y) + ((x23 + x24) * x17);
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = (-1 * x27) + x28;
	const GEN_FLT x30 =
		obj_px + ((x15 + x16) * x22) + (((-1 * x23) + x24) * x14) + ((1 + (-1 * x11 * (x10 + x7))) * sensor_x);
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + (x18 * (1 + (-1 * x2 * x6))) + (x21 * x26) + (x31 * x29);
	const GEN_FLT x33 = x27 + x28;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = (-1 * x35) + x36;
	const GEN_FLT x38 = x3 + x0;
	const GEN_FLT x39 = lh_px + (x34 * x33) + (x37 * x26) + (x30 * (1 + (-1 * x6 * x38)));
	const GEN_FLT x40 = x39 * x39;
	const GEN_FLT x41 = (x32 * x32) + x40;
	const GEN_FLT x42 = pow(x41, -1);
	const GEN_FLT x43 = x42 * x32;
	const GEN_FLT x44 = (-1 * x19) + x20;
	const GEN_FLT x45 = x35 + x36;
	const GEN_FLT x46 = lh_py + (x44 * x34) + (x25 * (1 + (-1 * x4 * x6))) + (x45 * x31);
	const GEN_FLT x47 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x48 = cos(x47);
	const GEN_FLT x49 = pow(x48, -1);
	const GEN_FLT x50 = x46 * x46;
	const GEN_FLT x51 = x50 + x41;
	const GEN_FLT x52 = pow(x51, -1.0 / 2.0) * x49;
	const GEN_FLT x53 = asin(x52 * x46);
	const GEN_FLT x54 = 8.0108022e-06 * x53;
	const GEN_FLT x55 = -8.0108022e-06 + (-1 * x54);
	const GEN_FLT x56 = 0.0028679863 + (x53 * x55);
	const GEN_FLT x57 = 5.3685255e-06 + (x53 * x56);
	const GEN_FLT x58 = 0.0076069798 + (x53 * x57);
	const GEN_FLT x59 = x53 * x53;
	const GEN_FLT x60 = tan(x47);
	const GEN_FLT x61 = x60 * pow(x41, -1.0 / 2.0);
	const GEN_FLT x62 = -1 * x61 * x46;
	const GEN_FLT x63 = atan2(-1 * x32, x39);
	const GEN_FLT x64 = ogeeMag_1 + (-1 * asin(x62)) + x63;
	const GEN_FLT x65 = curve_1 + (sin(x64) * ogeePhase_1);
	const GEN_FLT x66 = x53 * x58;
	const GEN_FLT x67 = -8.0108022e-06 + (-1.60216044e-05 * x53);
	const GEN_FLT x68 = (x67 * x53) + x56;
	const GEN_FLT x69 = (x68 * x53) + x57;
	const GEN_FLT x70 = (x69 * x53) + x58;
	const GEN_FLT x71 = x66 + (x70 * x53);
	const GEN_FLT x72 = sin(x47);
	const GEN_FLT x73 = x72 * x65;
	const GEN_FLT x74 = (x71 * x73) + x48;
	const GEN_FLT x75 = pow(x74, -1);
	const GEN_FLT x76 = x75 * x65;
	const GEN_FLT x77 = x76 * x59;
	const GEN_FLT x78 = (x77 * x58) + x62;
	const GEN_FLT x79 = pow((1 + (-1 * (x78 * x78))), -1.0 / 2.0);
	const GEN_FLT x80 = x46 * x39;
	const GEN_FLT x81 = x60 * pow(x41, -3.0 / 2.0);
	const GEN_FLT x82 = x80 * x81;
	const GEN_FLT x83 = pow((1 + (-1 * (x60 * x60) * x50 * x42)), -1.0 / 2.0);
	const GEN_FLT x84 = cos(x64) * ogeePhase_1;
	const GEN_FLT x85 = x84 * (x43 + (-1 * x82 * x83));
	const GEN_FLT x86 = x58 * x59;
	const GEN_FLT x87 = x86 * x75;
	const GEN_FLT x88 = pow(x51, -3.0 / 2.0) * x49;
	const GEN_FLT x89 = 2 * x46;
	const GEN_FLT x90 = pow((1 + (-1 * x50 * pow(x51, -1) * pow(x48, -2))), -1.0 / 2.0);
	const GEN_FLT x91 = x76 * x66;
	const GEN_FLT x92 = x91 * x90;
	const GEN_FLT x93 = x71 * x72;
	const GEN_FLT x94 = x55 * x90;
	const GEN_FLT x95 = x80 * x88;
	const GEN_FLT x96 = -1 * x95 * x94;
	const GEN_FLT x97 = 2.40324066e-05 * x53;
	const GEN_FLT x98 = x90 * x95;
	const GEN_FLT x99 = (-1 * x56 * x98) + (x53 * ((x54 * x98) + x96));
	const GEN_FLT x100 = (x53 * x99) + (-1 * x57 * x98);
	const GEN_FLT x101 = x86 * pow(x74, -2) * x65;
	const GEN_FLT x102 =
		x79 * (x82 + (x85 * x87) + (-1 * x88 * x89 * x92 * x39) +
			   (-1 * x101 *
				((x85 * x93) +
				 (x73 * ((x53 * ((x53 * ((x53 * (x96 + (x98 * x97) + (-1 * x67 * x98))) + (-1 * x68 * x98) + x99)) +
								 (-1 * x69 * x98) + x100)) +
						 (-1 * x70 * x98) + (-1 * x58 * x98) + (x53 * x100))))) +
			   (x77 * x100));
	const GEN_FLT x103 = cos(gibPhase_1 + (-1 * asin(x78)) + x63) * gibMag_1;
	const GEN_FLT x104 = x84 * x87;
	const GEN_FLT x105 = x83 * x61;
	const GEN_FLT x106 = x90 * ((-1 * x88 * x50) + x52);
	const GEN_FLT x107 = 2 * x91;
	const GEN_FLT x108 = x84 * x93;
	const GEN_FLT x109 = x55 * x106;
	const GEN_FLT x110 = (x56 * x106) + (x53 * ((-1 * x54 * x106) + x109));
	const GEN_FLT x111 = (x53 * x110) + (x57 * x106);
	const GEN_FLT x112 =
		x79 * ((x105 * x104) + (x107 * x106) + (-1 * x61) +
			   (-1 * x101 *
				((x108 * x105) +
				 (x73 * ((x53 * ((x53 * ((x53 * (x109 + (-1 * x97 * x106) + (x67 * x106))) + (x68 * x106) + x110)) +
								 (x69 * x106) + x111)) +
						 (x58 * x106) + (x70 * x106) + (x53 * x111))))) +
			   (x77 * x111));
	const GEN_FLT x113 = x42 * x39;
	const GEN_FLT x114 = -1 * x113;
	const GEN_FLT x115 = x81 * x46;
	const GEN_FLT x116 = x32 * x115;
	const GEN_FLT x117 = x114 + (-1 * x83 * x116);
	const GEN_FLT x118 = x88 * x46;
	const GEN_FLT x119 = x32 * x118;
	const GEN_FLT x120 = -1 * x94 * x119;
	const GEN_FLT x121 = x90 * x119;
	const GEN_FLT x122 = (-1 * x56 * x121) + (x53 * ((x54 * x121) + x120));
	const GEN_FLT x123 = (x53 * x122) + (-1 * x57 * x121);
	const GEN_FLT x124 = 2 * x32;
	const GEN_FLT x125 =
		x79 *
		(x116 + (x104 * x117) +
		 (-1 * x101 *
		  ((x108 * x117) +
		   (x73 * ((x53 * ((x53 * ((x53 * (x120 + (x97 * x121) + (-1 * x67 * x121))) + (-1 * x68 * x121) + x122)) +
						   (-1 * x69 * x121) + x123)) +
				   (-1 * x70 * x121) + (-1 * x58 * x121) + (x53 * x123))))) +
		 (-1 * x92 * x118 * x124) + (x77 * x123));
	const GEN_FLT x126 = x30 * x38;
	const GEN_FLT x127 = 2 * pow(x5, -1);
	const GEN_FLT x128 = x127 * lh_qw;
	const GEN_FLT x129 = x37 * x25;
	const GEN_FLT x130 = x26 * lh_qk;
	const GEN_FLT x131 = x33 * x18;
	const GEN_FLT x132 = x34 * lh_qj;
	const GEN_FLT x133 = (-1 * x128 * x126) + (x128 * x129) + (-1 * x130) + (x128 * x131) + x132;
	const GEN_FLT x134 = pow(x40, -1) * x32;
	const GEN_FLT x135 = pow(x39, -1);
	const GEN_FLT x136 = x30 * x29;
	const GEN_FLT x137 = x25 * x21;
	const GEN_FLT x138 = x26 * lh_qi;
	const GEN_FLT x139 = x31 * lh_qj;
	const GEN_FLT x140 = x2 * x127;
	const GEN_FLT x141 = (x128 * x136) + (x128 * x137) + x138 + (-1 * x139) + (-1 * x18 * x140 * lh_qw);
	const GEN_FLT x142 = x40 * x42;
	const GEN_FLT x143 = ((x133 * x134) + (-1 * x135 * x141)) * x142;
	const GEN_FLT x144 = 2 * x39;
	const GEN_FLT x145 = (x133 * x144) + (x124 * x141);
	const GEN_FLT x146 = 1.0 / 2.0 * x115;
	const GEN_FLT x147 = x45 * x30;
	const GEN_FLT x148 = x31 * lh_qk;
	const GEN_FLT x149 = x4 * x25;
	const GEN_FLT x150 = x44 * x18;
	const GEN_FLT x151 = x34 * lh_qi;
	const GEN_FLT x152 = (x128 * x147) + (x128 * x150) + x148 + (-1 * x128 * x149) + (-1 * x151);
	const GEN_FLT x153 = (x146 * x145) + (-1 * x61 * x152);
	const GEN_FLT x154 = x143 + (-1 * x83 * x153);
	const GEN_FLT x155 = 1.0 / 2.0 * x118;
	const GEN_FLT x156 = x90 * ((-1 * x155 * ((x89 * x152) + x145)) + (x52 * x152));
	const GEN_FLT x157 = x55 * x156;
	const GEN_FLT x158 = (x56 * x156) + (x53 * ((-1 * x54 * x156) + x157));
	const GEN_FLT x159 = (x53 * x158) + (x57 * x156);
	const GEN_FLT x160 =
		x79 * ((x104 * x154) + (x107 * x156) +
			   (-1 * x101 *
				((x108 * x154) +
				 (x73 * ((x53 * ((x53 * ((x53 * (x157 + (x67 * x156) + (-1 * x97 * x156))) + (x68 * x156) + x158)) +
								 (x69 * x156) + x159)) +
						 (x70 * x156) + (x58 * x156) + (x53 * x159))))) +
			   (x77 * x159) + x153);
	const GEN_FLT x161 = x127 * lh_qi;
	const GEN_FLT x162 = x26 * lh_qj;
	const GEN_FLT x163 = x34 * lh_qk;
	const GEN_FLT x164 = (-1 * x126 * x161) + (x129 * x161) + x162 + (x161 * x131) + x163;
	const GEN_FLT x165 = x26 * lh_qw;
	const GEN_FLT x166 = 4 * x5;
	const GEN_FLT x167 = -1 * x166 * lh_qi;
	const GEN_FLT x168 = (x161 * x136) + x148 + x165 + (x161 * x137) + (x18 * ((-1 * x140 * lh_qi) + x167));
	const GEN_FLT x169 = ((x164 * x134) + (-1 * x168 * x135)) * x142;
	const GEN_FLT x170 = (x164 * x144) + (x124 * x168);
	const GEN_FLT x171 = x34 * lh_qw;
	const GEN_FLT x172 = (x161 * x147) + x139 + (x25 * ((-1 * x4 * x161) + x167)) + (x161 * x150) + (-1 * x171);
	const GEN_FLT x173 = (x170 * x146) + (-1 * x61 * x172);
	const GEN_FLT x174 = x169 + (-1 * x83 * x173);
	const GEN_FLT x175 = x90 * ((-1 * x155 * ((x89 * x172) + x170)) + (x52 * x172));
	const GEN_FLT x176 = x55 * x175;
	const GEN_FLT x177 = (x56 * x175) + (x53 * ((-1 * x54 * x175) + x176));
	const GEN_FLT x178 = (x53 * x177) + (x57 * x175);
	const GEN_FLT x179 =
		x79 * ((x104 * x174) +
			   (-1 * x101 *
				((x108 * x174) +
				 (x73 * ((x53 * ((x53 * ((x53 * (x176 + (-1 * x97 * x175) + (x67 * x175))) + (x68 * x175) + x177)) +
								 (x69 * x175) + x178)) +
						 (x70 * x175) + (x58 * x175) + (x53 * x178))))) +
			   (x107 * x175) + (x77 * x178) + x173);
	const GEN_FLT x180 = x127 * lh_qj;
	const GEN_FLT x181 = -1 * x166 * lh_qj;
	const GEN_FLT x182 = (x30 * ((-1 * x38 * x180) + x181)) + (x129 * x180) + x138 + (x180 * x131) + x171;
	const GEN_FLT x183 = x31 * lh_qw;
	const GEN_FLT x184 = (x180 * x136) + (-1 * x183) + (x180 * x137) + x130 + (x18 * ((-1 * x140 * lh_qj) + x181));
	const GEN_FLT x185 = ((x182 * x134) + (-1 * x184 * x135)) * x142;
	const GEN_FLT x186 = (x182 * x144) + (x124 * x184);
	const GEN_FLT x187 = x31 * lh_qi;
	const GEN_FLT x188 = (x180 * x147) + x187 + (-1 * x180 * x149) + (x180 * x150) + x163;
	const GEN_FLT x189 = (x186 * x146) + (-1 * x61 * x188);
	const GEN_FLT x190 = x185 + (-1 * x83 * x189);
	const GEN_FLT x191 = x90 * ((-1 * x155 * ((x89 * x188) + x186)) + (x52 * x188));
	const GEN_FLT x192 = x55 * x191;
	const GEN_FLT x193 = (x56 * x191) + (x53 * ((-1 * x54 * x191) + x192));
	const GEN_FLT x194 = (x53 * x193) + (x57 * x191);
	const GEN_FLT x195 =
		x79 * ((x104 * x190) + (x107 * x191) +
			   (-1 * x101 *
				((x108 * x190) +
				 (x73 * ((x53 * ((x53 * ((x53 * (x192 + (-1 * x97 * x191) + (x67 * x191))) + x193 + (x68 * x191))) +
								 (x69 * x191) + x194)) +
						 (x70 * x191) + (x58 * x191) + (x53 * x194))))) +
			   (x77 * x194) + x189);
	const GEN_FLT x196 = x127 * lh_qk;
	const GEN_FLT x197 = -1 * x166 * lh_qk;
	const GEN_FLT x198 = (x30 * ((-1 * x38 * x196) + x197)) + (-1 * x165) + (x129 * x196) + (x196 * x131) + x151;
	const GEN_FLT x199 = (x196 * x136) + (x196 * x137) + x187 + x162 + (-1 * x2 * x18 * x196);
	const GEN_FLT x200 = ((x198 * x134) + (-1 * x199 * x135)) * x142;
	const GEN_FLT x201 = (x198 * x144) + (x124 * x199);
	const GEN_FLT x202 = x183 + (x25 * ((-1 * x4 * x196) + x197)) + (x196 * x147) + (x196 * x150) + x132;
	const GEN_FLT x203 = (x201 * x146) + (-1 * x61 * x202);
	const GEN_FLT x204 = x200 + (-1 * x83 * x203);
	const GEN_FLT x205 = x90 * ((-1 * x155 * ((x89 * x202) + x201)) + (x52 * x202));
	const GEN_FLT x206 = x55 * x205;
	const GEN_FLT x207 = (x56 * x205) + (x53 * ((-1 * x54 * x205) + x206));
	const GEN_FLT x208 = (x53 * x207) + (x57 * x205);
	const GEN_FLT x209 =
		x79 * ((x204 * x104) + (x205 * x107) +
			   (-1 * x101 *
				((x204 * x108) +
				 (x73 * ((x53 * ((x53 * ((x53 * (x206 + (-1 * x97 * x205) + (x67 * x205))) + (x68 * x205) + x207)) +
								 (x69 * x205) + x208)) +
						 (x70 * x205) + (x58 * x205) + (x53 * x208))))) +
			   (x77 * x208) + x203);
	out[0] = x43 + (-1 * x102) + (-1 * x103 * ((-1 * x43) + x102));
	out[1] = (-1 * x112) + (-1 * x103 * x112);
	out[2] = x114 + (-1 * x125) + (-1 * (x113 + x125) * x103);
	out[3] = x143 + (-1 * x160) + (-1 * ((-1 * x143) + x160) * x103);
	out[4] = (-1 * x179) + x169 + (-1 * ((-1 * x169) + x179) * x103);
	out[5] = x185 + (-1 * x195) + (-1 * ((-1 * x185) + x195) * x103);
	out[6] = x200 + (-1 * x209) + (-1 * ((-1 * x200) + x209) * x103);
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
	const GEN_FLT x2 = lh_qw * lh_qi;
	const GEN_FLT x3 = lh_qk * lh_qj;
	const GEN_FLT x4 = obj_qj * obj_qj;
	const GEN_FLT x5 = obj_qi * obj_qi;
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = 2 * sqrt((x7 + (obj_qw * obj_qw) + x6));
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = obj_pz + ((1 + (-1 * x6 * x8)) * sensor_z) + (x11 * (x9 + x10)) + (((-1 * x12) + x13) * x14);
	const GEN_FLT x16 = lh_qi * lh_qi;
	const GEN_FLT x17 = lh_qk * lh_qk;
	const GEN_FLT x18 = lh_qj * lh_qj;
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 2 * sqrt((x16 + (lh_qw * lh_qw) + x19));
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 =
		(x22 * ((-1 * x9) + x10)) + ((x23 + x24) * x14) + obj_py + ((1 + (-1 * (x7 + x5) * x8)) * sensor_y);
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 =
		obj_px + ((x12 + x13) * x22) + (((-1 * x23) + x24) * x11) + ((1 + (-1 * (x7 + x4) * x8)) * sensor_x);
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = lh_py + (((-1 * x2) + x3) * x21) + ((x26 + x27) * x29) + (x25 * (1 + (-1 * (x17 + x16) * x20)));
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = lh_pz + (x15 * (1 + (-1 * (x18 + x16) * x20))) + ((x2 + x3) * x31) + (((-1 * x32) + x33) * x29);
	const GEN_FLT x35 = lh_px + ((x32 + x33) * x21) + (((-1 * x26) + x27) * x31) + (x28 * (1 + (-1 * x20 * x19)));
	const GEN_FLT x36 = (x34 * x34) + (x35 * x35);
	const GEN_FLT x37 = x30 * pow(x36, -1.0 / 2.0);
	const GEN_FLT x38 = -1 * x1 * x37;
	const GEN_FLT x39 = atan2(-1 * x34, x35);
	const GEN_FLT x40 = ogeeMag_1 + (-1 * asin(x38)) + x39;
	const GEN_FLT x41 = sin(x40);
	const GEN_FLT x42 = curve_1 + (x41 * ogeePhase_1);
	const GEN_FLT x43 = cos(x0);
	const GEN_FLT x44 = x30 * x30;
	const GEN_FLT x45 = x44 + x36;
	const GEN_FLT x46 = pow(x45, -1.0 / 2.0) * x30;
	const GEN_FLT x47 = asin(pow(x43, -1) * x46);
	const GEN_FLT x48 = 8.0108022e-06 * x47;
	const GEN_FLT x49 = -8.0108022e-06 + (-1 * x48);
	const GEN_FLT x50 = 0.0028679863 + (x47 * x49);
	const GEN_FLT x51 = 5.3685255e-06 + (x50 * x47);
	const GEN_FLT x52 = 0.0076069798 + (x51 * x47);
	const GEN_FLT x53 = x47 * x47;
	const GEN_FLT x54 = x52 * x47;
	const GEN_FLT x55 = -8.0108022e-06 + (-1.60216044e-05 * x47);
	const GEN_FLT x56 = (x55 * x47) + x50;
	const GEN_FLT x57 = (x56 * x47) + x51;
	const GEN_FLT x58 = (x57 * x47) + x52;
	const GEN_FLT x59 = x54 + (x58 * x47);
	const GEN_FLT x60 = sin(x0);
	const GEN_FLT x61 = x60 * x42;
	const GEN_FLT x62 = x61 * x59;
	const GEN_FLT x63 = x62 + x43;
	const GEN_FLT x64 = pow(x63, -1);
	const GEN_FLT x65 = x64 * x53;
	const GEN_FLT x66 = x65 * x52;
	const GEN_FLT x67 = (x66 * x42) + x38;
	const GEN_FLT x68 = pow((1 + (-1 * (x67 * x67))), -1.0 / 2.0);
	const GEN_FLT x69 = x1 * x1;
	const GEN_FLT x70 = x37 * (1 + x69);
	const GEN_FLT x71 = cos(x40) * ogeePhase_1;
	const GEN_FLT x72 = x71 * x66;
	const GEN_FLT x73 = x70 * pow((1 + (-1 * x69 * x44 * pow(x36, -1))), -1.0 / 2.0);
	const GEN_FLT x74 = pow(x43, -2);
	const GEN_FLT x75 = x74 * x46 * pow((1 + (-1 * x74 * x44 * pow(x45, -1))), -1.0 / 2.0);
	const GEN_FLT x76 = x75 * x60;
	const GEN_FLT x77 = -1 * x76 * x49;
	const GEN_FLT x78 = (-1 * x76 * x50) + (x47 * ((x76 * x48) + x77));
	const GEN_FLT x79 = (x78 * x47) + (-1 * x76 * x51);
	const GEN_FLT x80 = pow(x63, -2) * x53 * x52;
	const GEN_FLT x81 =
		x68 * (x70 + (-1 * x73 * x72) + (x79 * x65 * x42) + (-2 * x75 * x64 * x61 * x54) +
			   (-1 * x80 * x42 *
				(x60 + (-1 * x71 * x73 * x60 * x59) + (-1 * x59 * x42 * x43) +
				 (x61 * ((x47 * ((x47 * ((x47 * (x77 + (-1 * x76 * x55) + (2.40324066e-05 * x76 * x47))) +
										 (-1 * x76 * x56) + x78)) +
								 (-1 * x76 * x57) + x79)) +
						 (-1 * x76 * x58) + (-1 * x76 * x52) + (x79 * x47))))));
	const GEN_FLT x82 = (-1 * gibPhase_1) + asin(x67) + (-1 * x39);
	const GEN_FLT x83 = cos(x82) * gibMag_1;
	const GEN_FLT x84 = x80 * x62;
	const GEN_FLT x85 = (x66 + (-1 * x84)) * x68;
	const GEN_FLT x86 = x68 * (x72 + (-1 * x84 * x71));
	const GEN_FLT x87 = ((x66 * x41) + (-1 * x84 * x41)) * x68;
	out[0] = -1;
	out[1] = (-1 * x81) + (-1 * x81 * x83);
	out[2] = (-1 * x85) + (-1 * x83 * x85);
	out[3] = x83;
	out[4] = -1 * sin(x82);
	out[5] = (-1 * x86) + (-1 * x83 * x86);
	out[6] = (-1 * x87) + (-1 * x83 * x87);
}

/** Applying function <function reproject at 0x7f35a4af8200> */
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
	const GEN_FLT x2 = obj_qj * obj_qj;
	const GEN_FLT x3 = obj_qi * obj_qi;
	const GEN_FLT x4 = x2 + x3;
	const GEN_FLT x5 = obj_qk * obj_qk;
	const GEN_FLT x6 = 2 * sqrt((x5 + (obj_qw * obj_qw) + x4));
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x6 * sensor_y;
	const GEN_FLT x10 = obj_qw * obj_qj;
	const GEN_FLT x11 = obj_qk * obj_qi;
	const GEN_FLT x12 = x6 * sensor_x;
	const GEN_FLT x13 = obj_pz + ((1 + (-1 * x4 * x6)) * sensor_z) + ((x7 + x8) * x9) + (((-1 * x10) + x11) * x12);
	const GEN_FLT x14 = lh_qi * lh_qi;
	const GEN_FLT x15 = lh_qk * lh_qk;
	const GEN_FLT x16 = lh_qj * lh_qj;
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = 2 * sqrt((x14 + (lh_qw * lh_qw) + x17));
	const GEN_FLT x19 = x13 * x18;
	const GEN_FLT x20 = x6 * sensor_z;
	const GEN_FLT x21 = obj_qw * obj_qk;
	const GEN_FLT x22 = obj_qj * obj_qi;
	const GEN_FLT x23 =
		(((-1 * x7) + x8) * x20) + obj_py + ((1 + (-1 * (x5 + x3) * x6)) * sensor_y) + ((x21 + x22) * x12);
	const GEN_FLT x24 = lh_qw * lh_qk;
	const GEN_FLT x25 = lh_qj * lh_qi;
	const GEN_FLT x26 =
		obj_px + ((x10 + x11) * x20) + (((-1 * x21) + x22) * x9) + ((1 + (-1 * (x5 + x2) * x6)) * sensor_x);
	const GEN_FLT x27 = x26 * x18;
	const GEN_FLT x28 = lh_py + ((x24 + x25) * x27) + (((-1 * x0) + x1) * x19) + (x23 * (1 + (-1 * (x15 + x14) * x18)));
	const GEN_FLT x29 = x23 * x18;
	const GEN_FLT x30 = lh_qw * lh_qj;
	const GEN_FLT x31 = lh_qk * lh_qi;
	const GEN_FLT x32 = lh_pz + (x13 * (1 + (-1 * (x16 + x14) * x18))) + ((x0 + x1) * x29) + (((-1 * x30) + x31) * x27);
	const GEN_FLT x33 = -1 * x32;
	const GEN_FLT x34 = x32 * x32;
	const GEN_FLT x35 = lh_px + ((x30 + x31) * x19) + (((-1 * x24) + x25) * x29) + (x26 * (1 + (-1 * x18 * x17)));
	const GEN_FLT x36 = atan2(x35, x33);
	const GEN_FLT x37 = (-1 * asin(x28 * pow((x34 + (x35 * x35)), -1.0 / 2.0) * tilt_0)) + (-1 * phase_0) + (-1 * x36);
	const GEN_FLT x38 =
		(-1 * asin(x35 * pow((x34 + (x28 * x28)), -1.0 / 2.0) * tilt_1)) + (-1 * phase_1) + (-1 * atan2(-1 * x28, x33));
	out[0] = ((atan2(x28, x33) * atan2(x28, x33)) * curve_0) +
			 (-1 * cos(1.5707963267949 + gibPhase_0 + x37) * gibMag_0) + x37;
	out[1] = (-1 * cos(1.5707963267949 + gibPhase_1 + x38) * gibMag_1) + ((x36 * x36) * curve_1) + x38;
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
	const GEN_FLT x0 = lh_qk * lh_qk;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = lh_qi * lh_qi;
	const GEN_FLT x4 = sqrt((x3 + (lh_qw * lh_qw) + x2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * x2 * x5);
	const GEN_FLT x7 = 1 + (-1 * (x1 + x3) * x5);
	const GEN_FLT x8 = obj_qj * obj_qj;
	const GEN_FLT x9 = obj_qi * obj_qi;
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = obj_qk * obj_qk;
	const GEN_FLT x12 = sqrt((x11 + (obj_qw * obj_qw) + x10));
	const GEN_FLT x13 = 2 * x12;
	const GEN_FLT x14 = obj_qw * obj_qi;
	const GEN_FLT x15 = obj_qk * obj_qj;
	const GEN_FLT x16 = x14 + x15;
	const GEN_FLT x17 = x13 * sensor_y;
	const GEN_FLT x18 = obj_qw * obj_qj;
	const GEN_FLT x19 = obj_qk * obj_qi;
	const GEN_FLT x20 = (-1 * x18) + x19;
	const GEN_FLT x21 = x13 * sensor_x;
	const GEN_FLT x22 = obj_pz + ((1 + (-1 * x13 * x10)) * sensor_z) + (x17 * x16) + (x20 * x21);
	const GEN_FLT x23 = (-1 * x14) + x15;
	const GEN_FLT x24 = x13 * sensor_z;
	const GEN_FLT x25 = x11 + x9;
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x26 + x27;
	const GEN_FLT x29 = (x24 * x23) + ((1 + (-1 * x25 * x13)) * sensor_y) + obj_py + (x21 * x28);
	const GEN_FLT x30 = lh_qw * lh_qi;
	const GEN_FLT x31 = lh_qk * lh_qj;
	const GEN_FLT x32 = x30 + x31;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = x18 + x19;
	const GEN_FLT x35 = (-1 * x26) + x27;
	const GEN_FLT x36 = x11 + x8;
	const GEN_FLT x37 = obj_px + (x34 * x24) + (x35 * x17) + ((1 + (-1 * x36 * x13)) * sensor_x);
	const GEN_FLT x38 = lh_qw * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qi;
	const GEN_FLT x40 = (-1 * x38) + x39;
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = lh_pz + (x7 * x22) + (x33 * x29) + (x41 * x37);
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = x42 * x42;
	const GEN_FLT x45 = pow(x44, -1);
	const GEN_FLT x46 = x38 + x39;
	const GEN_FLT x47 = x5 * x46;
	const GEN_FLT x48 = lh_qw * lh_qk;
	const GEN_FLT x49 = lh_qj * lh_qi;
	const GEN_FLT x50 = (-1 * x48) + x49;
	const GEN_FLT x51 = x5 * x50;
	const GEN_FLT x52 = lh_px + (x47 * x22) + (x51 * x29) + (x6 * x37);
	const GEN_FLT x53 = x52 * x45;
	const GEN_FLT x54 = x52 * x52;
	const GEN_FLT x55 = x54 + x44;
	const GEN_FLT x56 = pow(x55, -1);
	const GEN_FLT x57 = x56 * x44;
	const GEN_FLT x58 = x57 * ((-1 * x6 * x43) + (x53 * x41));
	const GEN_FLT x59 = (-1 * x30) + x31;
	const GEN_FLT x60 = x5 * x59;
	const GEN_FLT x61 = 1 + (-1 * (x0 + x3) * x5);
	const GEN_FLT x62 = x48 + x49;
	const GEN_FLT x63 = x5 * x62;
	const GEN_FLT x64 = lh_py + (x60 * x22) + (x61 * x29) + (x63 * x37);
	const GEN_FLT x65 = x64 * x64;
	const GEN_FLT x66 = pow((1 + (-1 * x65 * x56 * (tilt_0 * tilt_0))), -1.0 / 2.0);
	const GEN_FLT x67 = 2 * x52;
	const GEN_FLT x68 = 4 * x4;
	const GEN_FLT x69 = x68 * x42;
	const GEN_FLT x70 = x69 * x40;
	const GEN_FLT x71 = 1.0 / 2.0 * x64 * pow(x55, -3.0 / 2.0) * tilt_0;
	const GEN_FLT x72 = pow(x55, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x73 = (-1 * x58) + (-1 * x66 * ((-1 * x71 * ((x6 * x67) + x70)) + (x72 * x63)));
	const GEN_FLT x74 = -1 * x42;
	const GEN_FLT x75 = atan2(x52, x74);
	const GEN_FLT x76 =
		sin(1.5707963267949 + gibPhase_0 + (-1 * asin(x72 * x64)) + (-1 * phase_0) + (-1 * x75)) * gibMag_0;
	const GEN_FLT x77 = x63 * x43;
	const GEN_FLT x78 = x64 * x45;
	const GEN_FLT x79 = x78 * x41;
	const GEN_FLT x80 = x65 + x44;
	const GEN_FLT x81 = pow(x80, -1);
	const GEN_FLT x82 = x81 * x44;
	const GEN_FLT x83 = 2 * x82 * atan2(x64, x74) * curve_0;
	const GEN_FLT x84 = ((-1 * x51 * x43) + (x53 * x33)) * x57;
	const GEN_FLT x85 = x68 * x52;
	const GEN_FLT x86 = x69 * x32;
	const GEN_FLT x87 = (-1 * x84) + (-1 * x66 * ((-1 * x71 * ((x85 * x50) + x86)) + (x72 * x61)));
	const GEN_FLT x88 = x61 * x43;
	const GEN_FLT x89 = x78 * x33;
	const GEN_FLT x90 = x57 * ((-1 * x43 * x47) + (x7 * x53));
	const GEN_FLT x91 = 2 * x42;
	const GEN_FLT x92 = x7 * x91;
	const GEN_FLT x93 = (-1 * x90) + (-1 * x66 * ((-1 * x71 * ((x85 * x46) + x92)) + (x72 * x60)));
	const GEN_FLT x94 = x60 * x43;
	const GEN_FLT x95 = x7 * x78;
	const GEN_FLT x96 = 2 * pow(x12, -1);
	const GEN_FLT x97 = x96 * x36;
	const GEN_FLT x98 = x97 * sensor_x;
	const GEN_FLT x99 = x96 * obj_qw;
	const GEN_FLT x100 = x35 * sensor_y;
	const GEN_FLT x101 = x17 * obj_qk;
	const GEN_FLT x102 = x34 * sensor_z;
	const GEN_FLT x103 = x24 * obj_qj;
	const GEN_FLT x104 = (-1 * x98 * obj_qw) + (x99 * x100) + (-1 * x101) + (x99 * x102) + x103;
	const GEN_FLT x105 = x99 * sensor_x;
	const GEN_FLT x106 = x96 * x25;
	const GEN_FLT x107 = x106 * sensor_y;
	const GEN_FLT x108 = x21 * obj_qk;
	const GEN_FLT x109 = x23 * sensor_z;
	const GEN_FLT x110 = x24 * obj_qi;
	const GEN_FLT x111 = (x28 * x105) + (-1 * x107 * obj_qw) + x108 + (x99 * x109) + (-1 * x110);
	const GEN_FLT x112 = x21 * obj_qj;
	const GEN_FLT x113 = x16 * sensor_y;
	const GEN_FLT x114 = x17 * obj_qi;
	const GEN_FLT x115 = x96 * x10;
	const GEN_FLT x116 = x115 * sensor_z;
	const GEN_FLT x117 = (-1 * x112) + (x20 * x105) + (x99 * x113) + x114 + (-1 * x116 * obj_qw);
	const GEN_FLT x118 = (x6 * x104) + (x51 * x111) + (x47 * x117);
	const GEN_FLT x119 = (x41 * x104) + (x33 * x111) + (x7 * x117);
	const GEN_FLT x120 = ((-1 * x43 * x118) + (x53 * x119)) * x57;
	const GEN_FLT x121 = x91 * x119;
	const GEN_FLT x122 = (x63 * x104) + (x61 * x111) + (x60 * x117);
	const GEN_FLT x123 = (-1 * x120) + (-1 * x66 * ((-1 * x71 * ((x67 * x118) + x121)) + (x72 * x122)));
	const GEN_FLT x124 = x43 * x122;
	const GEN_FLT x125 = x78 * x119;
	const GEN_FLT x126 = x96 * obj_qi;
	const GEN_FLT x127 = x17 * obj_qj;
	const GEN_FLT x128 = x24 * obj_qk;
	const GEN_FLT x129 = (-1 * x98 * obj_qi) + (x100 * x126) + x127 + (x102 * x126) + x128;
	const GEN_FLT x130 = x28 * sensor_x;
	const GEN_FLT x131 = 4 * x12;
	const GEN_FLT x132 = -1 * x131 * obj_qi;
	const GEN_FLT x133 = x24 * obj_qw;
	const GEN_FLT x134 =
		(x126 * x130) + x112 + (((-1 * x106 * obj_qi) + x132) * sensor_y) + (x109 * x126) + (-1 * x133);
	const GEN_FLT x135 = x5 * x134;
	const GEN_FLT x136 = x20 * sensor_x;
	const GEN_FLT x137 = x96 * x113;
	const GEN_FLT x138 = x17 * obj_qw;
	const GEN_FLT x139 = (x126 * x136) + x108 + (x137 * obj_qi) + (((-1 * x115 * obj_qi) + x132) * sensor_z) + x138;
	const GEN_FLT x140 = (x6 * x129) + (x50 * x135) + (x47 * x139);
	const GEN_FLT x141 = (x41 * x129) + (x32 * x135) + (x7 * x139);
	const GEN_FLT x142 = ((-1 * x43 * x140) + (x53 * x141)) * x57;
	const GEN_FLT x143 = x91 * x141;
	const GEN_FLT x144 = (x63 * x129) + (x61 * x134) + (x60 * x139);
	const GEN_FLT x145 = (-1 * x142) + (-1 * x66 * ((-1 * x71 * ((x67 * x140) + x143)) + (x72 * x144)));
	const GEN_FLT x146 = x43 * x144;
	const GEN_FLT x147 = x78 * x141;
	const GEN_FLT x148 = -1 * x131 * obj_qj;
	const GEN_FLT x149 = x96 * x100;
	const GEN_FLT x150 = x96 * obj_qj;
	const GEN_FLT x151 = (((-1 * x97 * obj_qj) + x148) * sensor_x) + (x149 * obj_qj) + x114 + (x102 * x150) + x133;
	const GEN_FLT x152 = x21 * obj_qi;
	const GEN_FLT x153 = x96 * x109;
	const GEN_FLT x154 = (x130 * x150) + x152 + (-1 * x107 * obj_qj) + (x153 * obj_qj) + x128;
	const GEN_FLT x155 = x21 * obj_qw;
	const GEN_FLT x156 =
		(x136 * x150) + (-1 * x155) + (x137 * obj_qj) + x101 + (((-1 * x115 * obj_qj) + x148) * sensor_z);
	const GEN_FLT x157 = (x6 * x151) + (x51 * x154) + (x47 * x156);
	const GEN_FLT x158 = (x41 * x151) + (x33 * x154) + (x7 * x156);
	const GEN_FLT x159 = ((-1 * x43 * x157) + (x53 * x158)) * x57;
	const GEN_FLT x160 = x91 * x158;
	const GEN_FLT x161 = (x63 * x151) + (x61 * x154) + (x60 * x156);
	const GEN_FLT x162 = (-1 * x159) + (-1 * x66 * ((-1 * x71 * ((x67 * x157) + x160)) + (x72 * x161)));
	const GEN_FLT x163 = x43 * x161;
	const GEN_FLT x164 = x78 * x158;
	const GEN_FLT x165 = -1 * x131 * obj_qk;
	const GEN_FLT x166 = x96 * obj_qk;
	const GEN_FLT x167 =
		(x149 * obj_qk) + (-1 * x138) + (((-1 * x97 * obj_qk) + x165) * sensor_x) + x110 + (x102 * x166);
	const GEN_FLT x168 = (x166 * x130) + x103 + x155 + (((-1 * x106 * obj_qk) + x165) * sensor_y) + (x153 * obj_qk);
	const GEN_FLT x169 = x5 * x168;
	const GEN_FLT x170 = x152 + (x137 * obj_qk) + (x166 * x136) + x127 + (-1 * x116 * obj_qk);
	const GEN_FLT x171 = (x6 * x167) + (x50 * x169) + (x47 * x170);
	const GEN_FLT x172 = (x41 * x167) + (x32 * x169) + (x7 * x170);
	const GEN_FLT x173 = ((-1 * x43 * x171) + (x53 * x172)) * x57;
	const GEN_FLT x174 = x91 * x172;
	const GEN_FLT x175 = (x63 * x167) + (x61 * x168) + (x60 * x170);
	const GEN_FLT x176 = (-1 * x173) + (-1 * x66 * ((-1 * x71 * ((x67 * x171) + x174)) + (x72 * x175)));
	const GEN_FLT x177 = x43 * x175;
	const GEN_FLT x178 = x78 * x172;
	const GEN_FLT x179 = 2 * x75 * curve_1;
	const GEN_FLT x180 = pow((1 + (-1 * x81 * x54 * (tilt_1 * tilt_1))), -1.0 / 2.0);
	const GEN_FLT x181 = pow(x80, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x182 = x64 * x68;
	const GEN_FLT x183 = 1.0 / 2.0 * pow(x80, -3.0 / 2.0) * x52 * tilt_1;
	const GEN_FLT x184 =
		(-1 * (x77 + (-1 * x79)) * x82) + (-1 * x180 * ((x6 * x181) + (-1 * x183 * ((x62 * x182) + x70))));
	const GEN_FLT x185 =
		sin(1.5707963267949 + gibPhase_1 + (-1 * asin(x52 * x181)) + (-1 * phase_1) + (-1 * atan2(-1 * x64, x74))) *
		gibMag_1;
	const GEN_FLT x186 = 2 * x64;
	const GEN_FLT x187 =
		(-1 * (x88 + (-1 * x89)) * x82) + (-1 * x180 * ((x51 * x181) + (-1 * x183 * ((x61 * x186) + x86))));
	const GEN_FLT x188 =
		(-1 * (x94 + (-1 * x95)) * x82) + (-1 * x180 * ((x47 * x181) + (-1 * x183 * ((x59 * x182) + x92))));
	const GEN_FLT x189 =
		(-1 * (x124 + (-1 * x125)) * x82) + (-1 * x180 * ((x118 * x181) + (-1 * x183 * ((x122 * x186) + x121))));
	const GEN_FLT x190 =
		(-1 * (x146 + (-1 * x147)) * x82) + (-1 * x180 * ((x181 * x140) + (-1 * x183 * ((x186 * x144) + x143))));
	const GEN_FLT x191 =
		(-1 * (x163 + (-1 * x164)) * x82) + (-1 * x180 * ((x181 * x157) + (-1 * x183 * ((x161 * x186) + x160))));
	const GEN_FLT x192 =
		(-1 * (x177 + (-1 * x178)) * x82) + (-1 * x180 * ((x171 * x181) + (-1 * x183 * ((x175 * x186) + x174))));
	out[0] = (x73 * x76) + (((-1 * x77) + x79) * x83) + x73;
	out[1] = (x87 * x76) + x87 + (((-1 * x88) + x89) * x83);
	out[2] = (x76 * x93) + (((-1 * x94) + x95) * x83) + x93;
	out[3] = (x76 * x123) + (((-1 * x124) + x125) * x83) + x123;
	out[4] = (x76 * x145) + (((-1 * x146) + x147) * x83) + x145;
	out[5] = (x76 * x162) + (((-1 * x163) + x164) * x83) + x162;
	out[6] = (x76 * x176) + (((-1 * x177) + x178) * x83) + x176;
	out[7] = (x58 * x179) + (x185 * x184) + x184;
	out[8] = (x84 * x179) + (x187 * x185) + x187;
	out[9] = (x90 * x179) + (x188 * x185) + x188;
	out[10] = (x120 * x179) + (x189 * x185) + x189;
	out[11] = (x179 * x142) + (x185 * x190) + x190;
	out[12] = (x179 * x159) + (x185 * x191) + x191;
	out[13] = (x179 * x173) + (x185 * x192) + x192;
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
	const GEN_FLT x0 = lh_qj * lh_qj;
	const GEN_FLT x1 = lh_qi * lh_qi;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x2 + x0;
	const GEN_FLT x4 = sqrt((x1 + (lh_qw * lh_qw) + x3));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * (x0 + x1) * x5);
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = obj_qk * obj_qk;
	const GEN_FLT x10 = x9 + x8;
	const GEN_FLT x11 = sqrt((x7 + (obj_qw * obj_qw) + x10));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 + (-1 * (x7 + x8) * x12);
	const GEN_FLT x14 = obj_qw * obj_qi;
	const GEN_FLT x15 = obj_qk * obj_qj;
	const GEN_FLT x16 = x14 + x15;
	const GEN_FLT x17 = x12 * x16;
	const GEN_FLT x18 = obj_qw * obj_qj;
	const GEN_FLT x19 = obj_qk * obj_qi;
	const GEN_FLT x20 = (-1 * x18) + x19;
	const GEN_FLT x21 = x20 * x12;
	const GEN_FLT x22 = obj_pz + (x13 * sensor_z) + (x17 * sensor_y) + (x21 * sensor_x);
	const GEN_FLT x23 = (-1 * x14) + x15;
	const GEN_FLT x24 = x23 * x12;
	const GEN_FLT x25 = 1 + (-1 * x12 * x10);
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x26 + x27;
	const GEN_FLT x29 = x28 * x12;
	const GEN_FLT x30 = (x24 * sensor_z) + (x29 * sensor_x) + obj_py + (x25 * sensor_y);
	const GEN_FLT x31 = lh_qw * lh_qi;
	const GEN_FLT x32 = lh_qk * lh_qj;
	const GEN_FLT x33 = x31 + x32;
	const GEN_FLT x34 = x5 * x33;
	const GEN_FLT x35 = x18 + x19;
	const GEN_FLT x36 = (-1 * x26) + x27;
	const GEN_FLT x37 = 1 + (-1 * (x9 + x7) * x12);
	const GEN_FLT x38 = obj_px + (x35 * x12 * sensor_z) + (x36 * x12 * sensor_y) + (x37 * sensor_x);
	const GEN_FLT x39 = lh_qw * lh_qj;
	const GEN_FLT x40 = lh_qk * lh_qi;
	const GEN_FLT x41 = (-1 * x39) + x40;
	const GEN_FLT x42 = x5 * x41;
	const GEN_FLT x43 = lh_pz + (x6 * x22) + (x30 * x34) + (x42 * x38);
	const GEN_FLT x44 = pow(x43, -1);
	const GEN_FLT x45 = 1 + (-1 * x3 * x5);
	const GEN_FLT x46 = lh_qw * lh_qk;
	const GEN_FLT x47 = lh_qj * lh_qi;
	const GEN_FLT x48 = (-1 * x46) + x47;
	const GEN_FLT x49 = 4 * x4 * x11;
	const GEN_FLT x50 = x49 * x28;
	const GEN_FLT x51 = x39 + x40;
	const GEN_FLT x52 = x49 * x20;
	const GEN_FLT x53 = (x45 * x37) + (x52 * x51) + (x50 * x48);
	const GEN_FLT x54 = (x42 * x37) + (x50 * x33) + (x6 * x21);
	const GEN_FLT x55 = x43 * x43;
	const GEN_FLT x56 = pow(x55, -1);
	const GEN_FLT x57 = x5 * x22;
	const GEN_FLT x58 = x5 * x48;
	const GEN_FLT x59 = lh_px + (x51 * x57) + (x58 * x30) + (x45 * x38);
	const GEN_FLT x60 = x56 * x59;
	const GEN_FLT x61 = x59 * x59;
	const GEN_FLT x62 = x61 + x55;
	const GEN_FLT x63 = pow(x62, -1);
	const GEN_FLT x64 = x63 * x55;
	const GEN_FLT x65 = ((-1 * x53 * x44) + (x60 * x54)) * x64;
	const GEN_FLT x66 = (-1 * x31) + x32;
	const GEN_FLT x67 = 1 + (-1 * (x2 + x1) * x5);
	const GEN_FLT x68 = x46 + x47;
	const GEN_FLT x69 = x5 * x68;
	const GEN_FLT x70 = lh_py + (x66 * x57) + (x67 * x30) + (x69 * x38);
	const GEN_FLT x71 = x70 * x70;
	const GEN_FLT x72 = pow((1 + (-1 * x71 * x63 * (tilt_0 * tilt_0))), -1.0 / 2.0);
	const GEN_FLT x73 = 2 * x59;
	const GEN_FLT x74 = 2 * x43;
	const GEN_FLT x75 = x74 * x54;
	const GEN_FLT x76 = 1.0 / 2.0 * x70 * pow(x62, -3.0 / 2.0) * tilt_0;
	const GEN_FLT x77 = (x69 * x37) + (x67 * x29) + (x66 * x52);
	const GEN_FLT x78 = pow(x62, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x79 = (-1 * x65) + (-1 * x72 * ((-1 * x76 * ((x73 * x53) + x75)) + (x78 * x77)));
	const GEN_FLT x80 = -1 * x43;
	const GEN_FLT x81 = atan2(x59, x80);
	const GEN_FLT x82 =
		sin(1.5707963267949 + gibPhase_0 + (-1 * asin(x70 * x78)) + (-1 * phase_0) + (-1 * x81)) * gibMag_0;
	const GEN_FLT x83 = x77 * x44;
	const GEN_FLT x84 = x70 * x56;
	const GEN_FLT x85 = x84 * x54;
	const GEN_FLT x86 = x71 + x55;
	const GEN_FLT x87 = pow(x86, -1);
	const GEN_FLT x88 = x87 * x55;
	const GEN_FLT x89 = 2 * x88 * atan2(x70, x80) * curve_0;
	const GEN_FLT x90 = x45 * x12;
	const GEN_FLT x91 = x49 * x16;
	const GEN_FLT x92 = (x58 * x25) + (x90 * x36) + (x51 * x91);
	const GEN_FLT x93 = x49 * x36;
	const GEN_FLT x94 = (x93 * x41) + (x34 * x25) + (x6 * x17);
	const GEN_FLT x95 = ((-1 * x92 * x44) + (x60 * x94)) * x64;
	const GEN_FLT x96 = x74 * x94;
	const GEN_FLT x97 = (x68 * x93) + (x67 * x25) + (x66 * x91);
	const GEN_FLT x98 = (-1 * x95) + (-1 * x72 * ((-1 * x76 * ((x73 * x92) + x96)) + (x78 * x97)));
	const GEN_FLT x99 = x97 * x44;
	const GEN_FLT x100 = x84 * x94;
	const GEN_FLT x101 = x49 * x23;
	const GEN_FLT x102 = x5 * x13;
	const GEN_FLT x103 = (x90 * x35) + (x48 * x101) + (x51 * x102);
	const GEN_FLT x104 = x49 * x35;
	const GEN_FLT x105 = (x41 * x104) + (x33 * x101) + (x6 * x13);
	const GEN_FLT x106 = ((-1 * x44 * x103) + (x60 * x105)) * x64;
	const GEN_FLT x107 = x74 * x105;
	const GEN_FLT x108 = (x68 * x104) + (x67 * x24) + (x66 * x102);
	const GEN_FLT x109 = (-1 * x106) + (-1 * x72 * ((-1 * x76 * ((x73 * x103) + x107)) + (x78 * x108)));
	const GEN_FLT x110 = x44 * x108;
	const GEN_FLT x111 = x84 * x105;
	const GEN_FLT x112 = 2 * x81 * curve_1;
	const GEN_FLT x113 = pow((1 + (-1 * x87 * x61 * (tilt_1 * tilt_1))), -1.0 / 2.0);
	const GEN_FLT x114 = pow(x86, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x115 = 2 * x70;
	const GEN_FLT x116 = 1.0 / 2.0 * pow(x86, -3.0 / 2.0) * x59 * tilt_1;
	const GEN_FLT x117 =
		(-1 * (x83 + (-1 * x85)) * x88) + (-1 * x113 * ((x53 * x114) + (-1 * x116 * ((x77 * x115) + x75))));
	const GEN_FLT x118 =
		sin(1.5707963267949 + gibPhase_1 + (-1 * asin(x59 * x114)) + (-1 * phase_1) + (-1 * atan2(-1 * x70, x80))) *
		gibMag_1;
	const GEN_FLT x119 =
		(-1 * x88 * (x99 + (-1 * x100))) + (-1 * x113 * ((x92 * x114) + (-1 * x116 * ((x97 * x115) + x96))));
	const GEN_FLT x120 =
		(-1 * (x110 + (-1 * x111)) * x88) + (-1 * x113 * ((x103 * x114) + (-1 * x116 * ((x108 * x115) + x107))));
	out[0] = (x82 * x79) + (((-1 * x83) + x85) * x89) + x79;
	out[1] = (x82 * x98) + (x89 * ((-1 * x99) + x100)) + x98;
	out[2] = (x82 * x109) + (((-1 * x110) + x111) * x89) + x109;
	out[3] = (x65 * x112) + (x118 * x117) + x117;
	out[4] = (x95 * x112) + (x118 * x119) + x119;
	out[5] = (x106 * x112) + (x118 * x120) + x120;
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
	const GEN_FLT x0 = lh_qj * lh_qj;
	const GEN_FLT x1 = lh_qi * lh_qi;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = lh_qk * lh_qk;
	const GEN_FLT x4 = x3 + x1;
	const GEN_FLT x5 = sqrt((x0 + (lh_qw * lh_qw) + x4));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = obj_qk * obj_qk;
	const GEN_FLT x10 = x9 + x8;
	const GEN_FLT x11 = 2 * sqrt((x7 + (obj_qw * obj_qw) + x10));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 =
		obj_pz + ((1 + (-1 * (x7 + x8) * x11)) * sensor_z) + ((x12 + x13) * x14) + (((-1 * x15) + x16) * x17);
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = (((-1 * x12) + x13) * x22) + obj_py + ((1 + (-1 * x11 * x10)) * sensor_y) + ((x23 + x24) * x17);
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = (-1 * x27) + x28;
	const GEN_FLT x30 =
		obj_px + ((x15 + x16) * x22) + (((-1 * x23) + x24) * x14) + ((1 + (-1 * (x9 + x7) * x11)) * sensor_x);
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + (x18 * (1 + (-1 * x2 * x6))) + (x21 * x26) + (x31 * x29);
	const GEN_FLT x33 = x27 + x28;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = (-1 * x35) + x36;
	const GEN_FLT x38 = x3 + x0;
	const GEN_FLT x39 = lh_px + (x34 * x33) + (x37 * x26) + (x30 * (1 + (-1 * x6 * x38)));
	const GEN_FLT x40 = x39 * x39;
	const GEN_FLT x41 = x32 * x32;
	const GEN_FLT x42 = x40 + x41;
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = x43 * x32;
	const GEN_FLT x45 = (-1 * x19) + x20;
	const GEN_FLT x46 = x35 + x36;
	const GEN_FLT x47 = lh_py + (x45 * x34) + (x25 * (1 + (-1 * x4 * x6))) + (x46 * x31);
	const GEN_FLT x48 = x47 * x39;
	const GEN_FLT x49 = x47 * x47;
	const GEN_FLT x50 = pow((1 + (-1 * x43 * x49 * (tilt_0 * tilt_0))), -1.0 / 2.0);
	const GEN_FLT x51 = pow(x42, -3.0 / 2.0) * tilt_0;
	const GEN_FLT x52 = x50 * x51;
	const GEN_FLT x53 = x44 + (x52 * x48);
	const GEN_FLT x54 = pow(x42, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x55 = -1 * x32;
	const GEN_FLT x56 = atan2(x39, x55);
	const GEN_FLT x57 =
		sin(1.5707963267949 + gibPhase_0 + (-1 * asin(x54 * x47)) + (-1 * phase_0) + (-1 * x56)) * gibMag_0;
	const GEN_FLT x58 = x50 * x54;
	const GEN_FLT x59 = x49 + x41;
	const GEN_FLT x60 = pow(x59, -1);
	const GEN_FLT x61 = x60 * x32;
	const GEN_FLT x62 = 2 * atan2(x47, x55) * curve_0;
	const GEN_FLT x63 = x43 * x39;
	const GEN_FLT x64 = (-1 * x63) + (x52 * x47 * x32);
	const GEN_FLT x65 = x60 * x47;
	const GEN_FLT x66 = pow(x32, -1);
	const GEN_FLT x67 = 2 * pow(x5, -1);
	const GEN_FLT x68 = x67 * x38;
	const GEN_FLT x69 = x30 * lh_qw;
	const GEN_FLT x70 = x67 * x25;
	const GEN_FLT x71 = x70 * x37;
	const GEN_FLT x72 = x26 * lh_qk;
	const GEN_FLT x73 = x67 * x18;
	const GEN_FLT x74 = x73 * x33;
	const GEN_FLT x75 = x34 * lh_qj;
	const GEN_FLT x76 = (-1 * x68 * x69) + (x71 * lh_qw) + (-1 * x72) + (x74 * lh_qw) + x75;
	const GEN_FLT x77 = x67 * x29;
	const GEN_FLT x78 = x70 * x21;
	const GEN_FLT x79 = x26 * lh_qi;
	const GEN_FLT x80 = x31 * lh_qj;
	const GEN_FLT x81 = x2 * x67;
	const GEN_FLT x82 = x81 * x18;
	const GEN_FLT x83 = (x77 * x69) + (x78 * lh_qw) + (-1 * x82 * lh_qw) + x79 + (-1 * x80);
	const GEN_FLT x84 = pow(x41, -1);
	const GEN_FLT x85 = x84 * x39;
	const GEN_FLT x86 = x41 * x43;
	const GEN_FLT x87 = ((-1 * x76 * x66) + (x83 * x85)) * x86;
	const GEN_FLT x88 = 2 * x39;
	const GEN_FLT x89 = 2 * x32;
	const GEN_FLT x90 = x83 * x89;
	const GEN_FLT x91 = 1.0 / 2.0 * x51 * x47;
	const GEN_FLT x92 = x67 * x46;
	const GEN_FLT x93 = x31 * lh_qk;
	const GEN_FLT x94 = x4 * x67;
	const GEN_FLT x95 = x94 * x25;
	const GEN_FLT x96 = x73 * x45;
	const GEN_FLT x97 = x34 * lh_qi;
	const GEN_FLT x98 = (x69 * x92) + x93 + (-1 * x95 * lh_qw) + (-1 * x97) + (x96 * lh_qw);
	const GEN_FLT x99 = (-1 * x87) + (-1 * x50 * ((-1 * x91 * ((x88 * x76) + x90)) + (x54 * x98)));
	const GEN_FLT x100 = x66 * x98;
	const GEN_FLT x101 = x84 * x47;
	const GEN_FLT x102 = x83 * x101;
	const GEN_FLT x103 = x60 * x41;
	const GEN_FLT x104 = x62 * x103;
	const GEN_FLT x105 = x30 * lh_qi;
	const GEN_FLT x106 = x26 * lh_qj;
	const GEN_FLT x107 = x73 * lh_qi;
	const GEN_FLT x108 = x34 * lh_qk;
	const GEN_FLT x109 = (-1 * x68 * x105) + (x71 * lh_qi) + x106 + (x33 * x107) + x108;
	const GEN_FLT x110 = x26 * lh_qw;
	const GEN_FLT x111 = 4 * x5;
	const GEN_FLT x112 = -1 * x111 * lh_qi;
	const GEN_FLT x113 = (x77 * x105) + x93 + x110 + (x78 * lh_qi) + (x18 * ((-1 * x81 * lh_qi) + x112));
	const GEN_FLT x114 = ((-1 * x66 * x109) + (x85 * x113)) * x86;
	const GEN_FLT x115 = x89 * x113;
	const GEN_FLT x116 = x34 * lh_qw;
	const GEN_FLT x117 = (x92 * x105) + x80 + (x25 * ((-1 * x94 * lh_qi) + x112)) + (x45 * x107) + (-1 * x116);
	const GEN_FLT x118 = (-1 * x114) + (-1 * x50 * ((-1 * x91 * ((x88 * x109) + x115)) + (x54 * x117)));
	const GEN_FLT x119 = x66 * x117;
	const GEN_FLT x120 = x101 * x113;
	const GEN_FLT x121 = -1 * x111 * lh_qj;
	const GEN_FLT x122 = (x30 * ((-1 * x68 * lh_qj) + x121)) + (x71 * lh_qj) + x79 + (x74 * lh_qj) + x116;
	const GEN_FLT x123 = x30 * lh_qj;
	const GEN_FLT x124 = x31 * lh_qw;
	const GEN_FLT x125 = (x77 * x123) + (x18 * ((-1 * x81 * lh_qj) + x121)) + (-1 * x124) + (x78 * lh_qj) + x72;
	const GEN_FLT x126 = ((-1 * x66 * x122) + (x85 * x125)) * x86;
	const GEN_FLT x127 = x89 * x125;
	const GEN_FLT x128 = x31 * lh_qi;
	const GEN_FLT x129 = (x92 * x123) + x128 + (-1 * x95 * lh_qj) + (x96 * lh_qj) + x108;
	const GEN_FLT x130 = (-1 * x126) + (-1 * x50 * ((-1 * x91 * ((x88 * x122) + x127)) + (x54 * x129)));
	const GEN_FLT x131 = x66 * x129;
	const GEN_FLT x132 = x101 * x125;
	const GEN_FLT x133 = -1 * x111 * lh_qk;
	const GEN_FLT x134 = (x30 * ((-1 * x68 * lh_qk) + x133)) + (-1 * x110) + (x71 * lh_qk) + (x74 * lh_qk) + x97;
	const GEN_FLT x135 = x30 * lh_qk;
	const GEN_FLT x136 = (x77 * x135) + x128 + x106 + (x78 * lh_qk) + (-1 * x82 * lh_qk);
	const GEN_FLT x137 = ((-1 * x66 * x134) + (x85 * x136)) * x86;
	const GEN_FLT x138 = x89 * x136;
	const GEN_FLT x139 = x124 + (x25 * ((-1 * x94 * lh_qk) + x133)) + (x92 * x135) + (x96 * lh_qk) + x75;
	const GEN_FLT x140 = (-1 * x137) + (-1 * x50 * ((-1 * x91 * ((x88 * x134) + x138)) + (x54 * x139)));
	const GEN_FLT x141 = x66 * x139;
	const GEN_FLT x142 = x101 * x136;
	const GEN_FLT x143 = pow((1 + (-1 * x60 * x40 * (tilt_1 * tilt_1))), -1.0 / 2.0);
	const GEN_FLT x144 = pow(x59, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x145 = x144 * x143;
	const GEN_FLT x146 = 2 * x56 * curve_1;
	const GEN_FLT x147 =
		sin(1.5707963267949 + gibPhase_1 + (-1 * asin(x39 * x144)) + (-1 * phase_1) + (-1 * atan2(-1 * x47, x55))) *
		gibMag_1;
	const GEN_FLT x148 = pow(x59, -3.0 / 2.0) * tilt_1;
	const GEN_FLT x149 = x143 * x148;
	const GEN_FLT x150 = (-1 * x61) + (x48 * x149);
	const GEN_FLT x151 = x65 + (x32 * x39 * x149);
	const GEN_FLT x152 = 2 * x47;
	const GEN_FLT x153 = 1.0 / 2.0 * x39 * x148;
	const GEN_FLT x154 =
		(-1 * (x100 + (-1 * x102)) * x103) + (-1 * x143 * ((x76 * x144) + (-1 * x153 * ((x98 * x152) + x90))));
	const GEN_FLT x155 =
		(-1 * (x119 + (-1 * x120)) * x103) + (-1 * x143 * ((x109 * x144) + (-1 * x153 * ((x117 * x152) + x115))));
	const GEN_FLT x156 =
		(-1 * (x131 + (-1 * x132)) * x103) + (-1 * x143 * ((x122 * x144) + (-1 * x153 * ((x129 * x152) + x127))));
	const GEN_FLT x157 =
		(-1 * (x141 + (-1 * x142)) * x103) + (-1 * x143 * ((x134 * x144) + (-1 * x153 * ((x139 * x152) + x138))));
	out[0] = (x53 * x57) + x53;
	out[1] = (-1 * x58) + (-1 * x58 * x57) + (-1 * x61 * x62);
	out[2] = (x64 * x57) + (x62 * x65) + x64;
	out[3] = (x57 * x99) + (((-1 * x100) + x102) * x104) + x99;
	out[4] = (x57 * x118) + x118 + (((-1 * x119) + x120) * x104);
	out[5] = (x57 * x130) + (((-1 * x131) + x132) * x104) + x130;
	out[6] = (x57 * x140) + (((-1 * x141) + x142) * x104) + x140;
	out[7] = (-1 * x145) + (-1 * x44 * x146) + (-1 * x145 * x147);
	out[8] = (x147 * x150) + x150;
	out[9] = (x63 * x146) + x151 + (x147 * x151);
	out[10] = (x87 * x146) + (x147 * x154) + x154;
	out[11] = (x114 * x146) + (x147 * x155) + x155;
	out[12] = (x126 * x146) + (x147 * x156) + x156;
	out[13] = (x137 * x146) + (x147 * x157) + x157;
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
	const GEN_FLT x2 = obj_qj * obj_qj;
	const GEN_FLT x3 = obj_qi * obj_qi;
	const GEN_FLT x4 = x2 + x3;
	const GEN_FLT x5 = obj_qk * obj_qk;
	const GEN_FLT x6 = 2 * sqrt((x5 + (obj_qw * obj_qw) + x4));
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x6 * sensor_y;
	const GEN_FLT x10 = obj_qw * obj_qj;
	const GEN_FLT x11 = obj_qk * obj_qi;
	const GEN_FLT x12 = x6 * sensor_x;
	const GEN_FLT x13 = obj_pz + ((1 + (-1 * x4 * x6)) * sensor_z) + ((x7 + x8) * x9) + (((-1 * x10) + x11) * x12);
	const GEN_FLT x14 = lh_qi * lh_qi;
	const GEN_FLT x15 = lh_qk * lh_qk;
	const GEN_FLT x16 = lh_qj * lh_qj;
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = 2 * sqrt((x14 + (lh_qw * lh_qw) + x17));
	const GEN_FLT x19 = x13 * x18;
	const GEN_FLT x20 = x6 * sensor_z;
	const GEN_FLT x21 = obj_qw * obj_qk;
	const GEN_FLT x22 = obj_qj * obj_qi;
	const GEN_FLT x23 =
		(((-1 * x7) + x8) * x20) + obj_py + ((1 + (-1 * (x5 + x3) * x6)) * sensor_y) + ((x21 + x22) * x12);
	const GEN_FLT x24 = lh_qw * lh_qk;
	const GEN_FLT x25 = lh_qj * lh_qi;
	const GEN_FLT x26 =
		obj_px + ((x10 + x11) * x20) + (((-1 * x21) + x22) * x9) + ((1 + (-1 * (x5 + x2) * x6)) * sensor_x);
	const GEN_FLT x27 = x26 * x18;
	const GEN_FLT x28 = lh_py + ((x24 + x25) * x27) + (((-1 * x0) + x1) * x19) + (x23 * (1 + (-1 * (x15 + x14) * x18)));
	const GEN_FLT x29 = x23 * x18;
	const GEN_FLT x30 = lh_qw * lh_qj;
	const GEN_FLT x31 = lh_qk * lh_qi;
	const GEN_FLT x32 = lh_pz + (x13 * (1 + (-1 * (x16 + x14) * x18))) + ((x0 + x1) * x29) + (((-1 * x30) + x31) * x27);
	const GEN_FLT x33 = x32 * x32;
	const GEN_FLT x34 = lh_px + ((x30 + x31) * x19) + (((-1 * x24) + x25) * x29) + (x26 * (1 + (-1 * x18 * x17)));
	const GEN_FLT x35 = x34 * x34;
	const GEN_FLT x36 = x33 + x35;
	const GEN_FLT x37 = pow(x36, -1.0 / 2.0) * x28;
	const GEN_FLT x38 = -1 * x32;
	const GEN_FLT x39 = atan2(x34, x38);
	const GEN_FLT x40 = 1.5707963267949 + gibPhase_0 + (-1 * asin(x37 * tilt_0)) + (-1 * phase_0) + (-1 * x39);
	const GEN_FLT x41 = sin(x40) * gibMag_0;
	const GEN_FLT x42 = x28 * x28;
	const GEN_FLT x43 = x37 * pow((1 + (-1 * x42 * pow(x36, -1) * (tilt_0 * tilt_0))), -1.0 / 2.0);
	const GEN_FLT x44 = x33 + x42;
	const GEN_FLT x45 = pow(x44, -1.0 / 2.0) * x34;
	const GEN_FLT x46 =
		1.5707963267949 + gibPhase_1 + (-1 * asin(x45 * tilt_1)) + (-1 * phase_1) + (-1 * atan2(-1 * x28, x38));
	const GEN_FLT x47 = sin(x46) * gibMag_1;
	const GEN_FLT x48 = x45 * pow((1 + (-1 * pow(x44, -1) * x35 * (tilt_1 * tilt_1))), -1.0 / 2.0);
	out[0] = -1 + (-1 * x41);
	out[1] = (-1 * x41 * x43) + (-1 * x43);
	out[2] = atan2(x28, x38) * atan2(x28, x38);
	out[3] = x41;
	out[4] = -1 * cos(x40);
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
	out[21] = -1 + (-1 * x47);
	out[22] = (-1 * x48) + (-1 * x47 * x48);
	out[23] = x39 * x39;
	out[24] = x47;
	out[25] = -1 * cos(x46);
	out[26] = 0;
	out[27] = 0;
}

/** Applying function <function reproject_axis_x at 0x7f35a4af8290> */
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
	const GEN_FLT x2 = obj_qj * obj_qj;
	const GEN_FLT x3 = obj_qi * obj_qi;
	const GEN_FLT x4 = x2 + x3;
	const GEN_FLT x5 = obj_qk * obj_qk;
	const GEN_FLT x6 = 2 * sqrt((x5 + (obj_qw * obj_qw) + x4));
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x6 * sensor_y;
	const GEN_FLT x10 = obj_qw * obj_qj;
	const GEN_FLT x11 = obj_qk * obj_qi;
	const GEN_FLT x12 = x6 * sensor_x;
	const GEN_FLT x13 = obj_pz + ((1 + (-1 * x4 * x6)) * sensor_z) + ((x7 + x8) * x9) + (((-1 * x10) + x11) * x12);
	const GEN_FLT x14 = lh_qi * lh_qi;
	const GEN_FLT x15 = lh_qk * lh_qk;
	const GEN_FLT x16 = lh_qj * lh_qj;
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = 2 * sqrt((x14 + (lh_qw * lh_qw) + x17));
	const GEN_FLT x19 = x13 * x18;
	const GEN_FLT x20 = x6 * sensor_z;
	const GEN_FLT x21 = obj_qw * obj_qk;
	const GEN_FLT x22 = obj_qj * obj_qi;
	const GEN_FLT x23 =
		(((-1 * x7) + x8) * x20) + obj_py + ((1 + (-1 * (x5 + x3) * x6)) * sensor_y) + ((x21 + x22) * x12);
	const GEN_FLT x24 = lh_qw * lh_qk;
	const GEN_FLT x25 = lh_qj * lh_qi;
	const GEN_FLT x26 =
		obj_px + ((x10 + x11) * x20) + (((-1 * x21) + x22) * x9) + ((1 + (-1 * (x5 + x2) * x6)) * sensor_x);
	const GEN_FLT x27 = x26 * x18;
	const GEN_FLT x28 = lh_py + ((x24 + x25) * x27) + (((-1 * x0) + x1) * x19) + (x23 * (1 + (-1 * (x15 + x14) * x18)));
	const GEN_FLT x29 = x23 * x18;
	const GEN_FLT x30 = lh_qw * lh_qj;
	const GEN_FLT x31 = lh_qk * lh_qi;
	const GEN_FLT x32 = lh_pz + (x13 * (1 + (-1 * (x16 + x14) * x18))) + ((x0 + x1) * x29) + (((-1 * x30) + x31) * x27);
	const GEN_FLT x33 = -1 * x32;
	const GEN_FLT x34 = lh_px + ((x30 + x31) * x19) + (((-1 * x24) + x25) * x29) + (x26 * (1 + (-1 * x18 * x17)));
	const GEN_FLT x35 = (-1 * asin(pow(((x32 * x32) + (x34 * x34)), -1.0 / 2.0) * x28 * tilt_0)) +
						(-1 * atan2(x34, x33)) + (-1 * phase_0);
	out[0] = ((atan2(x28, x33) * atan2(x28, x33)) * curve_0) +
			 (-1 * cos(1.5707963267949 + gibPhase_0 + x35) * gibMag_0) + x35;
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
	const GEN_FLT x0 = lh_qk * lh_qk;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qi * lh_qi;
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt((x1 + (lh_qw * lh_qw) + x3));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * (x0 + x1) * x5);
	const GEN_FLT x7 = 1 + (-1 * (x1 + x2) * x5);
	const GEN_FLT x8 = obj_qj * obj_qj;
	const GEN_FLT x9 = obj_qi * obj_qi;
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = obj_qk * obj_qk;
	const GEN_FLT x12 = x11 + x9;
	const GEN_FLT x13 = sqrt((x8 + (obj_qw * obj_qw) + x12));
	const GEN_FLT x14 = 2 * x13;
	const GEN_FLT x15 = obj_qw * obj_qi;
	const GEN_FLT x16 = obj_qk * obj_qj;
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = x14 * sensor_y;
	const GEN_FLT x19 = obj_qw * obj_qj;
	const GEN_FLT x20 = obj_qk * obj_qi;
	const GEN_FLT x21 = (-1 * x19) + x20;
	const GEN_FLT x22 = x14 * sensor_x;
	const GEN_FLT x23 = obj_pz + ((1 + (-1 * x14 * x10)) * sensor_z) + (x18 * x17) + (x22 * x21);
	const GEN_FLT x24 = (-1 * x15) + x16;
	const GEN_FLT x25 = x14 * sensor_z;
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x26 + x27;
	const GEN_FLT x29 = (x24 * x25) + obj_py + ((1 + (-1 * x14 * x12)) * sensor_y) + (x22 * x28);
	const GEN_FLT x30 = lh_qw * lh_qi;
	const GEN_FLT x31 = lh_qk * lh_qj;
	const GEN_FLT x32 = x30 + x31;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = x19 + x20;
	const GEN_FLT x35 = (-1 * x26) + x27;
	const GEN_FLT x36 = x11 + x8;
	const GEN_FLT x37 = obj_px + (x34 * x25) + (x35 * x18) + ((1 + (-1 * x36 * x14)) * sensor_x);
	const GEN_FLT x38 = lh_qw * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qi;
	const GEN_FLT x40 = (-1 * x38) + x39;
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = lh_pz + (x7 * x23) + (x33 * x29) + (x41 * x37);
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = x42 * x42;
	const GEN_FLT x45 = pow(x44, -1);
	const GEN_FLT x46 = x38 + x39;
	const GEN_FLT x47 = x5 * x23;
	const GEN_FLT x48 = lh_qw * lh_qk;
	const GEN_FLT x49 = lh_qj * lh_qi;
	const GEN_FLT x50 = (-1 * x48) + x49;
	const GEN_FLT x51 = x5 * x50;
	const GEN_FLT x52 = lh_px + (x46 * x47) + (x51 * x29) + (x6 * x37);
	const GEN_FLT x53 = x52 * x45;
	const GEN_FLT x54 = (x52 * x52) + x44;
	const GEN_FLT x55 = pow(x54, -1);
	const GEN_FLT x56 = x55 * x44;
	const GEN_FLT x57 = (-1 * x30) + x31;
	const GEN_FLT x58 = 1 + (-1 * x3 * x5);
	const GEN_FLT x59 = (x48 + x49) * x5;
	const GEN_FLT x60 = lh_py + (x57 * x47) + (x59 * x37) + (x58 * x29);
	const GEN_FLT x61 = x60 * x60;
	const GEN_FLT x62 = pow((1 + (-1 * x61 * x55 * (tilt_0 * tilt_0))), -1.0 / 2.0);
	const GEN_FLT x63 = 2 * x52;
	const GEN_FLT x64 = 4 * x4;
	const GEN_FLT x65 = x64 * x42;
	const GEN_FLT x66 = 1.0 / 2.0 * x60 * pow(x54, -3.0 / 2.0) * tilt_0;
	const GEN_FLT x67 = pow(x54, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x68 = (-1 * x56 * ((-1 * x6 * x43) + (x53 * x41))) +
						(-1 * x62 * ((-1 * x66 * ((x6 * x63) + (x65 * x40))) + (x67 * x59)));
	const GEN_FLT x69 = -1 * x42;
	const GEN_FLT x70 =
		sin(1.5707963267949 + gibPhase_0 + (-1 * asin(x60 * x67)) + (-1 * phase_0) + (-1 * atan2(x52, x69))) * gibMag_0;
	const GEN_FLT x71 = x60 * x45;
	const GEN_FLT x72 = 2 * pow((x61 + x44), -1) * x44 * atan2(x60, x69) * curve_0;
	const GEN_FLT x73 = x64 * x52;
	const GEN_FLT x74 = (-1 * ((-1 * x51 * x43) + (x53 * x33)) * x56) +
						(-1 * x62 * ((-1 * ((x73 * x50) + (x65 * x32)) * x66) + (x67 * x58)));
	const GEN_FLT x75 = x5 * x46;
	const GEN_FLT x76 = 2 * x42;
	const GEN_FLT x77 = x5 * x57;
	const GEN_FLT x78 = (-1 * x56 * ((-1 * x75 * x43) + (x7 * x53))) +
						(-1 * x62 * ((-1 * x66 * ((x73 * x46) + (x7 * x76))) + (x77 * x67)));
	const GEN_FLT x79 = 2 * pow(x13, -1);
	const GEN_FLT x80 = x79 * x36;
	const GEN_FLT x81 = x80 * sensor_x;
	const GEN_FLT x82 = x79 * sensor_y;
	const GEN_FLT x83 = x82 * x35;
	const GEN_FLT x84 = x14 * obj_qk;
	const GEN_FLT x85 = x84 * sensor_y;
	const GEN_FLT x86 = x34 * sensor_z;
	const GEN_FLT x87 = x86 * x79;
	const GEN_FLT x88 = x25 * obj_qj;
	const GEN_FLT x89 = (-1 * x81 * obj_qw) + (x83 * obj_qw) + x88 + (-1 * x85) + (x87 * obj_qw);
	const GEN_FLT x90 = x28 * sensor_x;
	const GEN_FLT x91 = x79 * x90;
	const GEN_FLT x92 = x79 * x12;
	const GEN_FLT x93 = x92 * sensor_y;
	const GEN_FLT x94 = x22 * obj_qk;
	const GEN_FLT x95 = x24 * sensor_z;
	const GEN_FLT x96 = x79 * x95;
	const GEN_FLT x97 = x14 * obj_qi;
	const GEN_FLT x98 = x97 * sensor_z;
	const GEN_FLT x99 = (x91 * obj_qw) + (-1 * x93 * obj_qw) + x94 + (x96 * obj_qw) + (-1 * x98);
	const GEN_FLT x100 = x22 * obj_qj;
	const GEN_FLT x101 = x21 * sensor_x;
	const GEN_FLT x102 = x79 * x101;
	const GEN_FLT x103 = x82 * x17;
	const GEN_FLT x104 = x97 * sensor_y;
	const GEN_FLT x105 = x79 * x10;
	const GEN_FLT x106 = x105 * sensor_z;
	const GEN_FLT x107 = (-1 * x100) + (x102 * obj_qw) + (x103 * obj_qw) + (-1 * x106 * obj_qw) + x104;
	const GEN_FLT x108 = x5 * x107;
	const GEN_FLT x109 = (x6 * x89) + (x51 * x99) + (x46 * x108);
	const GEN_FLT x110 = (x89 * x41) + (x99 * x33) + (x7 * x107);
	const GEN_FLT x111 = (x89 * x59) + (x58 * x99) + (x57 * x108);
	const GEN_FLT x112 = (-1 * ((-1 * x43 * x109) + (x53 * x110)) * x56) +
						 (-1 * x62 * ((-1 * ((x63 * x109) + (x76 * x110)) * x66) + (x67 * x111)));
	const GEN_FLT x113 = x79 * obj_qi;
	const GEN_FLT x114 = x113 * sensor_y;
	const GEN_FLT x115 = x18 * obj_qj;
	const GEN_FLT x116 = x84 * sensor_z;
	const GEN_FLT x117 = (-1 * x81 * obj_qi) + (x35 * x114) + x115 + (x86 * x113) + x116;
	const GEN_FLT x118 = 4 * x13;
	const GEN_FLT x119 = -1 * x118 * obj_qi;
	const GEN_FLT x120 = x14 * obj_qw;
	const GEN_FLT x121 = x120 * sensor_z;
	const GEN_FLT x122 = (x90 * x113) + x100 + (((-1 * x92 * obj_qi) + x119) * sensor_y) + (x95 * x113) + (-1 * x121);
	const GEN_FLT x123 = x120 * sensor_y;
	const GEN_FLT x124 = (x101 * x113) + x94 + (((-1 * x105 * obj_qi) + x119) * sensor_z) + (x17 * x114) + x123;
	const GEN_FLT x125 = (x6 * x117) + (x51 * x122) + (x75 * x124);
	const GEN_FLT x126 = (x41 * x117) + (x33 * x122) + (x7 * x124);
	const GEN_FLT x127 = (x59 * x117) + (x58 * x122) + (x77 * x124);
	const GEN_FLT x128 = (-1 * ((-1 * x43 * x125) + (x53 * x126)) * x56) +
						 (-1 * x62 * ((-1 * ((x63 * x125) + (x76 * x126)) * x66) + (x67 * x127)));
	const GEN_FLT x129 = -1 * x118 * obj_qj;
	const GEN_FLT x130 = x79 * obj_qj;
	const GEN_FLT x131 = (((-1 * x80 * obj_qj) + x129) * sensor_x) + x121 + (x83 * obj_qj) + x104 + (x86 * x130);
	const GEN_FLT x132 = x22 * obj_qi;
	const GEN_FLT x133 = (x91 * obj_qj) + x132 + (-1 * x93 * obj_qj) + (x96 * obj_qj) + x116;
	const GEN_FLT x134 = x22 * obj_qw;
	const GEN_FLT x135 =
		(x101 * x130) + (((-1 * x105 * obj_qj) + x129) * sensor_z) + (-1 * x134) + (x103 * obj_qj) + x85;
	const GEN_FLT x136 = (x6 * x131) + (x51 * x133) + (x75 * x135);
	const GEN_FLT x137 = (x41 * x131) + (x33 * x133) + (x7 * x135);
	const GEN_FLT x138 = (x59 * x131) + (x58 * x133) + (x77 * x135);
	const GEN_FLT x139 = (-1 * ((-1 * x43 * x136) + (x53 * x137)) * x56) +
						 (-1 * x62 * ((-1 * ((x63 * x136) + (x76 * x137)) * x66) + (x67 * x138)));
	const GEN_FLT x140 = -1 * x118 * obj_qk;
	const GEN_FLT x141 =
		(x83 * obj_qk) + (-1 * x123) + (((-1 * x80 * obj_qk) + x140) * sensor_x) + (x87 * obj_qk) + x98;
	const GEN_FLT x142 = (x91 * obj_qk) + x134 + x88 + (((-1 * x92 * obj_qk) + x140) * sensor_y) + (x96 * obj_qk);
	const GEN_FLT x143 = x132 + (x103 * obj_qk) + (x102 * obj_qk) + x115 + (-1 * x106 * obj_qk);
	const GEN_FLT x144 = x5 * x143;
	const GEN_FLT x145 = (x6 * x141) + (x51 * x142) + (x46 * x144);
	const GEN_FLT x146 = (x41 * x141) + (x33 * x142) + (x7 * x143);
	const GEN_FLT x147 = (x59 * x141) + (x58 * x142) + (x57 * x144);
	const GEN_FLT x148 = (-1 * ((-1 * x43 * x145) + (x53 * x146)) * x56) +
						 (-1 * x62 * ((-1 * ((x63 * x145) + (x76 * x146)) * x66) + (x67 * x147)));
	out[0] = (x70 * x68) + (((-1 * x59 * x43) + (x71 * x41)) * x72) + x68;
	out[1] = (x70 * x74) + (((-1 * x58 * x43) + (x71 * x33)) * x72) + x74;
	out[2] = (x70 * x78) + (x72 * ((-1 * x77 * x43) + (x7 * x71))) + x78;
	out[3] = (x70 * x112) + (((-1 * x43 * x111) + (x71 * x110)) * x72) + x112;
	out[4] = (x70 * x128) + (((-1 * x43 * x127) + (x71 * x126)) * x72) + x128;
	out[5] = (x70 * x139) + (((-1 * x43 * x138) + (x71 * x137)) * x72) + x139;
	out[6] = (x70 * x148) + (((-1 * x43 * x147) + (x71 * x146)) * x72) + x148;
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
	const GEN_FLT x0 = lh_qj * lh_qj;
	const GEN_FLT x1 = lh_qi * lh_qi;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = lh_qk * lh_qk;
	const GEN_FLT x4 = sqrt((x3 + (lh_qw * lh_qw) + x2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * x2 * x5);
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = obj_qk * obj_qk;
	const GEN_FLT x10 = x9 + x8;
	const GEN_FLT x11 = sqrt((x7 + (obj_qw * obj_qw) + x10));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 + (-1 * (x7 + x8) * x12);
	const GEN_FLT x14 = obj_qw * obj_qi;
	const GEN_FLT x15 = obj_qk * obj_qj;
	const GEN_FLT x16 = x14 + x15;
	const GEN_FLT x17 = x12 * sensor_y;
	const GEN_FLT x18 = obj_qw * obj_qj;
	const GEN_FLT x19 = obj_qk * obj_qi;
	const GEN_FLT x20 = (-1 * x18) + x19;
	const GEN_FLT x21 = x12 * sensor_x;
	const GEN_FLT x22 = obj_pz + (x13 * sensor_z) + (x17 * x16) + (x20 * x21);
	const GEN_FLT x23 = lh_qw * lh_qi;
	const GEN_FLT x24 = lh_qk * lh_qj;
	const GEN_FLT x25 = x23 + x24;
	const GEN_FLT x26 = (-1 * x14) + x15;
	const GEN_FLT x27 = x12 * sensor_z;
	const GEN_FLT x28 = 1 + (-1 * x12 * x10);
	const GEN_FLT x29 = obj_qw * obj_qk;
	const GEN_FLT x30 = obj_qj * obj_qi;
	const GEN_FLT x31 = x29 + x30;
	const GEN_FLT x32 = (x26 * x27) + obj_py + (x28 * sensor_y) + (x31 * x21);
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = lh_qw * lh_qj;
	const GEN_FLT x35 = lh_qk * lh_qi;
	const GEN_FLT x36 = (-1 * x34) + x35;
	const GEN_FLT x37 = x18 + x19;
	const GEN_FLT x38 = (-1 * x29) + x30;
	const GEN_FLT x39 = 1 + (-1 * (x9 + x7) * x12);
	const GEN_FLT x40 = obj_px + (x37 * x27) + (x38 * x17) + (x39 * sensor_x);
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = lh_pz + (x6 * x22) + (x41 * x36) + (x33 * x25);
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = 1 + (-1 * (x3 + x0) * x5);
	const GEN_FLT x45 = lh_qw * lh_qk;
	const GEN_FLT x46 = lh_qj * lh_qi;
	const GEN_FLT x47 = (-1 * x45) + x46;
	const GEN_FLT x48 = 4 * x4 * x11;
	const GEN_FLT x49 = x48 * x31;
	const GEN_FLT x50 = x34 + x35;
	const GEN_FLT x51 = x48 * x20;
	const GEN_FLT x52 = (x44 * x39) + (x47 * x49) + (x50 * x51);
	const GEN_FLT x53 = x5 * x39;
	const GEN_FLT x54 = x6 * x12;
	const GEN_FLT x55 = (x53 * x36) + (x49 * x25) + (x54 * x20);
	const GEN_FLT x56 = x5 * x22;
	const GEN_FLT x57 = lh_px + (x50 * x56) + (x47 * x33) + (x40 * x44);
	const GEN_FLT x58 = x42 * x42;
	const GEN_FLT x59 = pow(x58, -1);
	const GEN_FLT x60 = x57 * x59;
	const GEN_FLT x61 = (x57 * x57) + x58;
	const GEN_FLT x62 = pow(x61, -1);
	const GEN_FLT x63 = x62 * x58;
	const GEN_FLT x64 = (-1 * x23) + x24;
	const GEN_FLT x65 = 1 + (-1 * (x3 + x1) * x5);
	const GEN_FLT x66 = x45 + x46;
	const GEN_FLT x67 = lh_py + (x64 * x56) + (x65 * x32) + (x66 * x41);
	const GEN_FLT x68 = x67 * x67;
	const GEN_FLT x69 = pow((1 + (-1 * x62 * x68 * (tilt_0 * tilt_0))), -1.0 / 2.0);
	const GEN_FLT x70 = 2 * x57;
	const GEN_FLT x71 = 2 * x42;
	const GEN_FLT x72 = 1.0 / 2.0 * pow(x61, -3.0 / 2.0) * x67 * tilt_0;
	const GEN_FLT x73 = x65 * x12;
	const GEN_FLT x74 = (x66 * x53) + (x73 * x31) + (x64 * x51);
	const GEN_FLT x75 = pow(x61, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x76 = (-1 * ((-1 * x52 * x43) + (x60 * x55)) * x63) +
						(-1 * x69 * ((-1 * ((x70 * x52) + (x71 * x55)) * x72) + (x75 * x74)));
	const GEN_FLT x77 = -1 * x42;
	const GEN_FLT x78 =
		sin(1.5707963267949 + gibPhase_0 + (-1 * asin(x75 * x67)) + (-1 * phase_0) + (-1 * atan2(x57, x77))) * gibMag_0;
	const GEN_FLT x79 = x67 * x59;
	const GEN_FLT x80 = 2 * pow((x68 + x58), -1) * x58 * atan2(x67, x77) * curve_0;
	const GEN_FLT x81 = x5 * x28;
	const GEN_FLT x82 = x44 * x12;
	const GEN_FLT x83 = x48 * x16;
	const GEN_FLT x84 = (x81 * x47) + (x82 * x38) + (x83 * x50);
	const GEN_FLT x85 = x48 * x38;
	const GEN_FLT x86 = (x85 * x36) + (x81 * x25) + (x54 * x16);
	const GEN_FLT x87 = (x85 * x66) + (x65 * x28) + (x83 * x64);
	const GEN_FLT x88 = (-1 * ((-1 * x84 * x43) + (x86 * x60)) * x63) +
						(-1 * x69 * ((-1 * ((x84 * x70) + (x86 * x71)) * x72) + (x87 * x75)));
	const GEN_FLT x89 = x48 * x26;
	const GEN_FLT x90 = x5 * x13;
	const GEN_FLT x91 = (x82 * x37) + (x89 * x47) + (x50 * x90);
	const GEN_FLT x92 = x48 * x37;
	const GEN_FLT x93 = (x92 * x36) + (x89 * x25) + (x6 * x13);
	const GEN_FLT x94 = (x66 * x92) + (x73 * x26) + (x64 * x90);
	const GEN_FLT x95 = (-1 * ((-1 * x91 * x43) + (x60 * x93)) * x63) +
						(-1 * x69 * ((-1 * ((x70 * x91) + (x71 * x93)) * x72) + (x75 * x94)));
	out[0] = (x78 * x76) + (((-1 * x74 * x43) + (x79 * x55)) * x80) + x76;
	out[1] = (x88 * x78) + (((-1 * x87 * x43) + (x86 * x79)) * x80) + x88;
	out[2] = (x78 * x95) + (((-1 * x94 * x43) + (x79 * x93)) * x80) + x95;
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
	const GEN_FLT x0 = lh_qj * lh_qj;
	const GEN_FLT x1 = lh_qi * lh_qi;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = lh_qk * lh_qk;
	const GEN_FLT x4 = x3 + x1;
	const GEN_FLT x5 = sqrt((x0 + (lh_qw * lh_qw) + x4));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = obj_qk * obj_qk;
	const GEN_FLT x10 = x9 + x8;
	const GEN_FLT x11 = 2 * sqrt((x7 + (obj_qw * obj_qw) + x10));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 =
		obj_pz + ((1 + (-1 * (x7 + x8) * x11)) * sensor_z) + ((x12 + x13) * x14) + (((-1 * x15) + x16) * x17);
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = (((-1 * x12) + x13) * x22) + obj_py + ((1 + (-1 * x11 * x10)) * sensor_y) + ((x23 + x24) * x17);
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = (-1 * x27) + x28;
	const GEN_FLT x30 =
		obj_px + ((x15 + x16) * x22) + (((-1 * x23) + x24) * x14) + ((1 + (-1 * (x9 + x7) * x11)) * sensor_x);
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + (x18 * (1 + (-1 * x2 * x6))) + (x21 * x26) + (x31 * x29);
	const GEN_FLT x33 = x27 + x28;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = (-1 * x35) + x36;
	const GEN_FLT x38 = x3 + x0;
	const GEN_FLT x39 = lh_px + (x34 * x33) + (x37 * x26) + (x30 * (1 + (-1 * x6 * x38)));
	const GEN_FLT x40 = x32 * x32;
	const GEN_FLT x41 = (x39 * x39) + x40;
	const GEN_FLT x42 = pow(x41, -1);
	const GEN_FLT x43 = (-1 * x19) + x20;
	const GEN_FLT x44 = x35 + x36;
	const GEN_FLT x45 = lh_py + (x43 * x34) + (x25 * (1 + (-1 * x4 * x6))) + (x44 * x31);
	const GEN_FLT x46 = x45 * x45;
	const GEN_FLT x47 = pow((1 + (-1 * x42 * x46 * (tilt_0 * tilt_0))), -1.0 / 2.0);
	const GEN_FLT x48 = pow(x41, -3.0 / 2.0) * x45 * tilt_0;
	const GEN_FLT x49 = x47 * x48;
	const GEN_FLT x50 = (x42 * x32) + (x49 * x39);
	const GEN_FLT x51 = pow(x41, -1.0 / 2.0) * tilt_0;
	const GEN_FLT x52 = -1 * x32;
	const GEN_FLT x53 =
		sin(1.5707963267949 + gibPhase_0 + (-1 * asin(x51 * x45)) + (-1 * phase_0) + (-1 * atan2(x39, x52))) * gibMag_0;
	const GEN_FLT x54 = x51 * x47;
	const GEN_FLT x55 = 2 * x32;
	const GEN_FLT x56 = pow((x46 + x40), -1) * atan2(x45, x52) * curve_0;
	const GEN_FLT x57 = (-1 * x42 * x39) + (x49 * x32);
	const GEN_FLT x58 = 2 * x56;
	const GEN_FLT x59 = pow(x32, -1);
	const GEN_FLT x60 = 2 * pow(x5, -1);
	const GEN_FLT x61 = x60 * x38;
	const GEN_FLT x62 = x61 * x30;
	const GEN_FLT x63 = x60 * lh_qw;
	const GEN_FLT x64 = x37 * x25;
	const GEN_FLT x65 = x26 * lh_qk;
	const GEN_FLT x66 = x60 * x18;
	const GEN_FLT x67 = x66 * x33;
	const GEN_FLT x68 = x34 * lh_qj;
	const GEN_FLT x69 = (-1 * x62 * lh_qw) + (x63 * x64) + (-1 * x65) + (x67 * lh_qw) + x68;
	const GEN_FLT x70 = x30 * x29;
	const GEN_FLT x71 = x25 * x21;
	const GEN_FLT x72 = x26 * lh_qi;
	const GEN_FLT x73 = x31 * lh_qj;
	const GEN_FLT x74 = x2 * x60;
	const GEN_FLT x75 = x74 * x18;
	const GEN_FLT x76 = (x70 * x63) + (x71 * x63) + x72 + (-1 * x73) + (-1 * x75 * lh_qw);
	const GEN_FLT x77 = pow(x40, -1);
	const GEN_FLT x78 = x77 * x39;
	const GEN_FLT x79 = x40 * x42;
	const GEN_FLT x80 = 2 * x39;
	const GEN_FLT x81 = 1.0 / 2.0 * x48;
	const GEN_FLT x82 = x44 * x30;
	const GEN_FLT x83 = x31 * lh_qk;
	const GEN_FLT x84 = x4 * x60;
	const GEN_FLT x85 = x84 * x25;
	const GEN_FLT x86 = x66 * x43;
	const GEN_FLT x87 = x34 * lh_qi;
	const GEN_FLT x88 = (x82 * x63) + x83 + (-1 * x85 * lh_qw) + (x86 * lh_qw) + (-1 * x87);
	const GEN_FLT x89 = (-1 * ((-1 * x69 * x59) + (x78 * x76)) * x79) +
						(-1 * x47 * ((-1 * ((x80 * x69) + (x76 * x55)) * x81) + (x88 * x51)));
	const GEN_FLT x90 = x77 * x45;
	const GEN_FLT x91 = x58 * x40;
	const GEN_FLT x92 = x60 * lh_qi;
	const GEN_FLT x93 = x26 * lh_qj;
	const GEN_FLT x94 = x34 * lh_qk;
	const GEN_FLT x95 = (-1 * x62 * lh_qi) + x93 + (x64 * x92) + (x67 * lh_qi) + x94;
	const GEN_FLT x96 = x26 * lh_qw;
	const GEN_FLT x97 = 4 * x5;
	const GEN_FLT x98 = -1 * x97 * lh_qi;
	const GEN_FLT x99 = (x70 * x92) + x83 + x96 + (x71 * x92) + (x18 * ((-1 * x74 * lh_qi) + x98));
	const GEN_FLT x100 = x34 * lh_qw;
	const GEN_FLT x101 = (x82 * x92) + x73 + (x86 * lh_qi) + (x25 * ((-1 * x84 * lh_qi) + x98)) + (-1 * x100);
	const GEN_FLT x102 = (-1 * ((-1 * x59 * x95) + (x78 * x99)) * x79) +
						 (-1 * x47 * ((-1 * ((x80 * x95) + (x55 * x99)) * x81) + (x51 * x101)));
	const GEN_FLT x103 = -1 * x97 * lh_qj;
	const GEN_FLT x104 = x60 * lh_qj;
	const GEN_FLT x105 = (x30 * ((-1 * x61 * lh_qj) + x103)) + x100 + (x64 * x104) + x72 + (x67 * lh_qj);
	const GEN_FLT x106 = x31 * lh_qw;
	const GEN_FLT x107 = (x70 * x104) + (-1 * x106) + x65 + (x71 * x104) + (x18 * ((-1 * x74 * lh_qj) + x103));
	const GEN_FLT x108 = x31 * lh_qi;
	const GEN_FLT x109 = (x82 * x104) + x94 + x108 + (-1 * x85 * lh_qj) + (x86 * lh_qj);
	const GEN_FLT x110 = (-1 * ((-1 * x59 * x105) + (x78 * x107)) * x79) +
						 (-1 * x47 * ((-1 * ((x80 * x105) + (x55 * x107)) * x81) + (x51 * x109)));
	const GEN_FLT x111 = -1 * x97 * lh_qk;
	const GEN_FLT x112 = x60 * lh_qk;
	const GEN_FLT x113 = x25 * x112;
	const GEN_FLT x114 = (x30 * ((-1 * x61 * lh_qk) + x111)) + (-1 * x96) + (x37 * x113) + (x67 * lh_qk) + x87;
	const GEN_FLT x115 = (x70 * x112) + (x21 * x113) + x93 + x108 + (-1 * x75 * lh_qk);
	const GEN_FLT x116 = x106 + (x25 * ((-1 * x84 * lh_qk) + x111)) + (x82 * x112) + (x86 * lh_qk) + x68;
	const GEN_FLT x117 = (-1 * ((-1 * x59 * x114) + (x78 * x115)) * x79) +
						 (-1 * x47 * ((-1 * ((x80 * x114) + (x55 * x115)) * x81) + (x51 * x116)));
	out[0] = (x50 * x53) + x50;
	out[1] = (-1 * x54) + (-1 * x54 * x53) + (-1 * x56 * x55);
	out[2] = (x53 * x57) + (x58 * x45) + x57;
	out[3] = (x89 * x53) + (((-1 * x88 * x59) + (x76 * x90)) * x91) + x89;
	out[4] = (x53 * x102) + (x91 * ((-1 * x59 * x101) + (x90 * x99))) + x102;
	out[5] = (x53 * x110) + (((-1 * x59 * x109) + (x90 * x107)) * x91) + x110;
	out[6] = (x53 * x117) + (((-1 * x59 * x116) + (x90 * x115)) * x91) + x117;
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
	const GEN_FLT x2 = obj_qj * obj_qj;
	const GEN_FLT x3 = obj_qi * obj_qi;
	const GEN_FLT x4 = x2 + x3;
	const GEN_FLT x5 = obj_qk * obj_qk;
	const GEN_FLT x6 = 2 * sqrt((x5 + (obj_qw * obj_qw) + x4));
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x6 * sensor_y;
	const GEN_FLT x10 = obj_qw * obj_qj;
	const GEN_FLT x11 = obj_qk * obj_qi;
	const GEN_FLT x12 = x6 * sensor_x;
	const GEN_FLT x13 = obj_pz + ((1 + (-1 * x4 * x6)) * sensor_z) + ((x7 + x8) * x9) + (((-1 * x10) + x11) * x12);
	const GEN_FLT x14 = lh_qi * lh_qi;
	const GEN_FLT x15 = lh_qk * lh_qk;
	const GEN_FLT x16 = lh_qj * lh_qj;
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = 2 * sqrt((x14 + (lh_qw * lh_qw) + x17));
	const GEN_FLT x19 = x13 * x18;
	const GEN_FLT x20 = x6 * sensor_z;
	const GEN_FLT x21 = obj_qw * obj_qk;
	const GEN_FLT x22 = obj_qj * obj_qi;
	const GEN_FLT x23 =
		(((-1 * x7) + x8) * x20) + obj_py + ((1 + (-1 * (x5 + x3) * x6)) * sensor_y) + ((x21 + x22) * x12);
	const GEN_FLT x24 = lh_qw * lh_qk;
	const GEN_FLT x25 = lh_qj * lh_qi;
	const GEN_FLT x26 =
		obj_px + ((x10 + x11) * x20) + (((-1 * x21) + x22) * x9) + ((1 + (-1 * (x5 + x2) * x6)) * sensor_x);
	const GEN_FLT x27 = x26 * x18;
	const GEN_FLT x28 = lh_py + ((x24 + x25) * x27) + (((-1 * x0) + x1) * x19) + (x23 * (1 + (-1 * (x15 + x14) * x18)));
	const GEN_FLT x29 = x23 * x18;
	const GEN_FLT x30 = lh_qw * lh_qj;
	const GEN_FLT x31 = lh_qk * lh_qi;
	const GEN_FLT x32 = lh_pz + (x13 * (1 + (-1 * (x16 + x14) * x18))) + ((x0 + x1) * x29) + (((-1 * x30) + x31) * x27);
	const GEN_FLT x33 = lh_px + ((x30 + x31) * x19) + (((-1 * x24) + x25) * x29) + (x26 * (1 + (-1 * x18 * x17)));
	const GEN_FLT x34 = (x32 * x32) + (x33 * x33);
	const GEN_FLT x35 = pow(x34, -1.0 / 2.0) * x28;
	const GEN_FLT x36 = -1 * x32;
	const GEN_FLT x37 =
		1.5707963267949 + gibPhase_0 + (-1 * asin(x35 * tilt_0)) + (-1 * phase_0) + (-1 * atan2(x33, x36));
	const GEN_FLT x38 = sin(x37) * gibMag_0;
	const GEN_FLT x39 = x35 * pow((1 + (-1 * pow(x34, -1) * (x28 * x28) * (tilt_0 * tilt_0))), -1.0 / 2.0);
	out[0] = -1 + (-1 * x38);
	out[1] = (-1 * x38 * x39) + (-1 * x39);
	out[2] = atan2(x28, x36) * atan2(x28, x36);
	out[3] = x38;
	out[4] = -1 * cos(x37);
	out[5] = 0;
	out[6] = 0;
}

/** Applying function <function reproject_axis_y at 0x7f35a4af8320> */
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
	const GEN_FLT x0 = lh_qj * lh_qj;
	const GEN_FLT x1 = lh_qi * lh_qi;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x2 + x1;
	const GEN_FLT x4 = 2 * sqrt((x0 + (lh_qw * lh_qw) + x3));
	const GEN_FLT x5 = obj_qj * obj_qj;
	const GEN_FLT x6 = obj_qi * obj_qi;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = x7 + x6;
	const GEN_FLT x9 = 2 * sqrt((x5 + (obj_qw * obj_qw) + x8));
	const GEN_FLT x10 = obj_qw * obj_qi;
	const GEN_FLT x11 = obj_qk * obj_qj;
	const GEN_FLT x12 = x9 * sensor_y;
	const GEN_FLT x13 = obj_qw * obj_qj;
	const GEN_FLT x14 = obj_qk * obj_qi;
	const GEN_FLT x15 = x9 * sensor_x;
	const GEN_FLT x16 =
		obj_pz + ((1 + (-1 * (x5 + x6) * x9)) * sensor_z) + ((x10 + x11) * x12) + (((-1 * x13) + x14) * x15);
	const GEN_FLT x17 = lh_qw * lh_qi;
	const GEN_FLT x18 = lh_qk * lh_qj;
	const GEN_FLT x19 = x9 * sensor_z;
	const GEN_FLT x20 = obj_qw * obj_qk;
	const GEN_FLT x21 = obj_qj * obj_qi;
	const GEN_FLT x22 = (((-1 * x10) + x11) * x19) + obj_py + ((1 + (-1 * x8 * x9)) * sensor_y) + ((x20 + x21) * x15);
	const GEN_FLT x23 = x4 * x22;
	const GEN_FLT x24 = lh_qw * lh_qj;
	const GEN_FLT x25 = lh_qk * lh_qi;
	const GEN_FLT x26 =
		obj_px + ((x13 + x14) * x19) + (((-1 * x20) + x21) * x12) + ((1 + (-1 * (x7 + x5) * x9)) * sensor_x);
	const GEN_FLT x27 = x4 * x26;
	const GEN_FLT x28 = lh_pz + ((x17 + x18) * x23) + (x16 * (1 + (-1 * (x0 + x1) * x4))) + (((-1 * x24) + x25) * x27);
	const GEN_FLT x29 = x4 * x16;
	const GEN_FLT x30 = lh_qw * lh_qk;
	const GEN_FLT x31 = lh_qj * lh_qi;
	const GEN_FLT x32 = lh_py + (((-1 * x17) + x18) * x29) + (x22 * (1 + (-1 * x4 * x3))) + ((x30 + x31) * x27);
	const GEN_FLT x33 = lh_px + ((x24 + x25) * x29) + (((-1 * x30) + x31) * x23) + (x26 * (1 + (-1 * (x2 + x0) * x4)));
	const GEN_FLT x34 = -1 * x28;
	const GEN_FLT x35 = (-1 * asin(pow(((x28 * x28) + (x32 * x32)), -1.0 / 2.0) * x33 * tilt_1)) + (-1 * phase_1) +
						(-1 * atan2(-1 * x32, x34));
	out[0] = (-1 * cos(1.5707963267949 + gibPhase_1 + x35) * gibMag_1) +
			 ((atan2(x33, x34) * atan2(x33, x34)) * curve_1) + x35;
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
	const GEN_FLT x0 = lh_qk * lh_qk;
	const GEN_FLT x1 = lh_qj * lh_qj;
	const GEN_FLT x2 = lh_qi * lh_qi;
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt((x1 + (lh_qw * lh_qw) + x3));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * (x0 + x1) * x5);
	const GEN_FLT x7 = 1 + (-1 * (x1 + x2) * x5);
	const GEN_FLT x8 = obj_qj * obj_qj;
	const GEN_FLT x9 = obj_qi * obj_qi;
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = obj_qk * obj_qk;
	const GEN_FLT x12 = x11 + x9;
	const GEN_FLT x13 = sqrt((x8 + (obj_qw * obj_qw) + x12));
	const GEN_FLT x14 = 2 * x13;
	const GEN_FLT x15 = obj_qw * obj_qi;
	const GEN_FLT x16 = obj_qk * obj_qj;
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = x14 * sensor_y;
	const GEN_FLT x19 = obj_qw * obj_qj;
	const GEN_FLT x20 = obj_qk * obj_qi;
	const GEN_FLT x21 = (-1 * x19) + x20;
	const GEN_FLT x22 = x14 * sensor_x;
	const GEN_FLT x23 = obj_pz + ((1 + (-1 * x14 * x10)) * sensor_z) + (x18 * x17) + (x22 * x21);
	const GEN_FLT x24 = (-1 * x15) + x16;
	const GEN_FLT x25 = x14 * sensor_z;
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x26 + x27;
	const GEN_FLT x29 = (x24 * x25) + obj_py + ((1 + (-1 * x14 * x12)) * sensor_y) + (x22 * x28);
	const GEN_FLT x30 = lh_qw * lh_qi;
	const GEN_FLT x31 = lh_qk * lh_qj;
	const GEN_FLT x32 = x30 + x31;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = x19 + x20;
	const GEN_FLT x35 = (-1 * x26) + x27;
	const GEN_FLT x36 = x11 + x8;
	const GEN_FLT x37 = obj_px + (x34 * x25) + (x35 * x18) + ((1 + (-1 * x36 * x14)) * sensor_x);
	const GEN_FLT x38 = lh_qw * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qi;
	const GEN_FLT x40 = (-1 * x38) + x39;
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = lh_pz + (x7 * x23) + (x33 * x29) + (x41 * x37);
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = x42 * x42;
	const GEN_FLT x45 = pow(x44, -1);
	const GEN_FLT x46 = x38 + x39;
	const GEN_FLT x47 = x5 * x23;
	const GEN_FLT x48 = lh_qw * lh_qk;
	const GEN_FLT x49 = lh_qj * lh_qi;
	const GEN_FLT x50 = ((-1 * x48) + x49) * x5;
	const GEN_FLT x51 = lh_px + (x46 * x47) + (x50 * x29) + (x6 * x37);
	const GEN_FLT x52 = x51 * x45;
	const GEN_FLT x53 = -1 * x42;
	const GEN_FLT x54 = x51 * x51;
	const GEN_FLT x55 = 2 * pow((x54 + x44), -1) * x44 * atan2(x51, x53) * curve_1;
	const GEN_FLT x56 = x48 + x49;
	const GEN_FLT x57 = x5 * x56;
	const GEN_FLT x58 = (-1 * x30) + x31;
	const GEN_FLT x59 = 1 + (-1 * x3 * x5);
	const GEN_FLT x60 = lh_py + (x58 * x47) + (x59 * x29) + (x57 * x37);
	const GEN_FLT x61 = 2 * x60;
	const GEN_FLT x62 = x4 * x61 * x45;
	const GEN_FLT x63 = (x60 * x60) + x44;
	const GEN_FLT x64 = pow(x63, -1);
	const GEN_FLT x65 = x64 * x44;
	const GEN_FLT x66 = pow((1 + (-1 * x64 * x54 * (tilt_1 * tilt_1))), -1.0 / 2.0);
	const GEN_FLT x67 = pow(x63, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x68 = 4 * x4;
	const GEN_FLT x69 = x60 * x68;
	const GEN_FLT x70 = x68 * x42;
	const GEN_FLT x71 = 1.0 / 2.0 * pow(x63, -3.0 / 2.0) * x51 * tilt_1;
	const GEN_FLT x72 = (-1 * ((x57 * x43) + (-1 * x62 * x40)) * x65) +
						(-1 * x66 * ((x6 * x67) + (-1 * ((x69 * x56) + (x70 * x40)) * x71)));
	const GEN_FLT x73 =
		sin(1.5707963267949 + gibPhase_1 + (-1 * asin(x67 * x51)) + (-1 * phase_1) + (-1 * atan2(-1 * x60, x53))) *
		gibMag_1;
	const GEN_FLT x74 = (-1 * ((x59 * x43) + (-1 * x62 * x32)) * x65) +
						(-1 * x66 * ((x67 * x50) + (-1 * ((x61 * x59) + (x70 * x32)) * x71)));
	const GEN_FLT x75 = x5 * x43;
	const GEN_FLT x76 = x7 * x45;
	const GEN_FLT x77 = x5 * x46;
	const GEN_FLT x78 = 2 * x42;
	const GEN_FLT x79 = (-1 * ((x75 * x58) + (-1 * x76 * x60)) * x65) +
						(-1 * x66 * ((x77 * x67) + (-1 * x71 * ((x69 * x58) + (x7 * x78)))));
	const GEN_FLT x80 = 2 * pow(x13, -1);
	const GEN_FLT x81 = x80 * x36;
	const GEN_FLT x82 = x81 * sensor_x;
	const GEN_FLT x83 = x35 * sensor_y;
	const GEN_FLT x84 = x80 * x83;
	const GEN_FLT x85 = x18 * obj_qk;
	const GEN_FLT x86 = x80 * obj_qw;
	const GEN_FLT x87 = x34 * sensor_z;
	const GEN_FLT x88 = x25 * obj_qj;
	const GEN_FLT x89 = (-1 * x82 * obj_qw) + (x86 * x87) + x88 + (x84 * obj_qw) + (-1 * x85);
	const GEN_FLT x90 = x28 * sensor_x;
	const GEN_FLT x91 = x80 * x12;
	const GEN_FLT x92 = x91 * sensor_y;
	const GEN_FLT x93 = x22 * obj_qk;
	const GEN_FLT x94 = x24 * sensor_z;
	const GEN_FLT x95 = x25 * obj_qi;
	const GEN_FLT x96 = (x86 * x90) + (-1 * x92 * obj_qw) + (x86 * x94) + (-1 * x95) + x93;
	const GEN_FLT x97 = x22 * obj_qj;
	const GEN_FLT x98 = x21 * sensor_x;
	const GEN_FLT x99 = x17 * sensor_y;
	const GEN_FLT x100 = x80 * x99;
	const GEN_FLT x101 = x18 * obj_qi;
	const GEN_FLT x102 = x80 * x10;
	const GEN_FLT x103 = x102 * sensor_z;
	const GEN_FLT x104 = (-1 * x97) + (x86 * x98) + (x100 * obj_qw) + x101 + (-1 * x103 * obj_qw);
	const GEN_FLT x105 = x5 * x104;
	const GEN_FLT x106 = (x6 * x89) + (x50 * x96) + (x46 * x105);
	const GEN_FLT x107 = (x89 * x41) + (x96 * x33) + (x7 * x104);
	const GEN_FLT x108 = (x89 * x57) + (x59 * x96) + (x58 * x105);
	const GEN_FLT x109 = x60 * x45;
	const GEN_FLT x110 = (-1 * x65 * ((x43 * x108) + (-1 * x109 * x107))) +
						 (-1 * x66 * ((x67 * x106) + (-1 * ((x61 * x108) + (x78 * x107)) * x71)));
	const GEN_FLT x111 = x80 * obj_qi;
	const GEN_FLT x112 = x18 * obj_qj;
	const GEN_FLT x113 = x111 * sensor_z;
	const GEN_FLT x114 = x25 * obj_qk;
	const GEN_FLT x115 = (-1 * x82 * obj_qi) + x114 + (x83 * x111) + x112 + (x34 * x113);
	const GEN_FLT x116 = x111 * sensor_x;
	const GEN_FLT x117 = 4 * x13;
	const GEN_FLT x118 = -1 * x117 * obj_qi;
	const GEN_FLT x119 = x25 * obj_qw;
	const GEN_FLT x120 = (x28 * x116) + x97 + (-1 * x119) + (((-1 * x91 * obj_qi) + x118) * sensor_y) + (x24 * x113);
	const GEN_FLT x121 = x18 * obj_qw;
	const GEN_FLT x122 = (x21 * x116) + (((-1 * x102 * obj_qi) + x118) * sensor_z) + x93 + (x99 * x111) + x121;
	const GEN_FLT x123 = (x6 * x115) + (x50 * x120) + (x77 * x122);
	const GEN_FLT x124 = (x41 * x115) + (x33 * x120) + (x7 * x122);
	const GEN_FLT x125 = x5 * x58;
	const GEN_FLT x126 = (x57 * x115) + (x59 * x120) + (x122 * x125);
	const GEN_FLT x127 = (-1 * x65 * ((x43 * x126) + (-1 * x109 * x124))) +
						 (-1 * x66 * ((x67 * x123) + (-1 * ((x61 * x126) + (x78 * x124)) * x71)));
	const GEN_FLT x128 = -1 * x117 * obj_qj;
	const GEN_FLT x129 = x80 * obj_qj;
	const GEN_FLT x130 = (((-1 * x81 * obj_qj) + x128) * sensor_x) + (x84 * obj_qj) + x101 + (x87 * x129) + x119;
	const GEN_FLT x131 = x22 * obj_qi;
	const GEN_FLT x132 = (x90 * x129) + x131 + (-1 * x92 * obj_qj) + (x94 * x129) + x114;
	const GEN_FLT x133 = x22 * obj_qw;
	const GEN_FLT x134 =
		(x98 * x129) + (-1 * x133) + (x100 * obj_qj) + x85 + (((-1 * x102 * obj_qj) + x128) * sensor_z);
	const GEN_FLT x135 = (x6 * x130) + (x50 * x132) + (x77 * x134);
	const GEN_FLT x136 = (x41 * x130) + (x33 * x132) + (x7 * x134);
	const GEN_FLT x137 = x45 * x136;
	const GEN_FLT x138 = (x57 * x130) + (x59 * x132) + (x125 * x134);
	const GEN_FLT x139 = (-1 * ((x43 * x138) + (-1 * x60 * x137)) * x65) +
						 (-1 * x66 * ((x67 * x135) + (-1 * ((x61 * x138) + (x78 * x136)) * x71)));
	const GEN_FLT x140 = -1 * x117 * obj_qk;
	const GEN_FLT x141 = x80 * obj_qk;
	const GEN_FLT x142 = (x84 * obj_qk) + (-1 * x121) + x95 + (((-1 * x81 * obj_qk) + x140) * sensor_x) + (x87 * x141);
	const GEN_FLT x143 = (x90 * x141) + x133 + (((-1 * x91 * obj_qk) + x140) * sensor_y) + (x94 * x141) + x88;
	const GEN_FLT x144 = x131 + (x100 * obj_qk) + (x98 * x141) + x112 + (-1 * x103 * obj_qk);
	const GEN_FLT x145 = x5 * x144;
	const GEN_FLT x146 = (x6 * x142) + (x50 * x143) + (x46 * x145);
	const GEN_FLT x147 = (x41 * x142) + (x33 * x143) + (x7 * x144);
	const GEN_FLT x148 = x45 * x147;
	const GEN_FLT x149 = (x57 * x142) + (x59 * x143) + (x58 * x145);
	const GEN_FLT x150 = (-1 * ((x43 * x149) + (-1 * x60 * x148)) * x65) +
						 (-1 * x66 * ((x67 * x146) + (-1 * ((x61 * x149) + (x78 * x147)) * x71)));
	out[0] = (x55 * ((-1 * x6 * x43) + (x52 * x41))) + (x73 * x72) + x72;
	out[1] = (((-1 * x50 * x43) + (x52 * x33)) * x55) + (x73 * x74) + x74;
	out[2] = (((-1 * x75 * x46) + (x76 * x51)) * x55) + (x73 * x79) + x79;
	out[3] = (((-1 * x43 * x106) + (x52 * x107)) * x55) + x110 + (x73 * x110);
	out[4] = (((-1 * x43 * x123) + (x52 * x124)) * x55) + (x73 * x127) + x127;
	out[5] = (((-1 * x43 * x135) + (x51 * x137)) * x55) + (x73 * x139) + x139;
	out[6] = (((-1 * x43 * x146) + (x51 * x148)) * x55) + (x73 * x150) + x150;
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
	const GEN_FLT x0 = lh_qj * lh_qj;
	const GEN_FLT x1 = lh_qi * lh_qi;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x2 + x0;
	const GEN_FLT x4 = sqrt((x1 + (lh_qw * lh_qw) + x3));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * (x0 + x1) * x5);
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = x7 + x8;
	const GEN_FLT x10 = obj_qk * obj_qk;
	const GEN_FLT x11 = sqrt((x10 + (obj_qw * obj_qw) + x9));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 + (-1 * x9 * x12);
	const GEN_FLT x14 = obj_qw * obj_qi;
	const GEN_FLT x15 = obj_qk * obj_qj;
	const GEN_FLT x16 = x14 + x15;
	const GEN_FLT x17 = x12 * sensor_y;
	const GEN_FLT x18 = obj_qw * obj_qj;
	const GEN_FLT x19 = obj_qk * obj_qi;
	const GEN_FLT x20 = (-1 * x18) + x19;
	const GEN_FLT x21 = x12 * sensor_x;
	const GEN_FLT x22 = obj_pz + (x13 * sensor_z) + (x17 * x16) + (x20 * x21);
	const GEN_FLT x23 = (-1 * x14) + x15;
	const GEN_FLT x24 = x23 * x12;
	const GEN_FLT x25 = 1 + (-1 * x12 * (x10 + x8));
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x26 + x27;
	const GEN_FLT x29 = (x24 * sensor_z) + obj_py + (x25 * sensor_y) + (x21 * x28);
	const GEN_FLT x30 = lh_qw * lh_qi;
	const GEN_FLT x31 = lh_qk * lh_qj;
	const GEN_FLT x32 = x30 + x31;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = x18 + x19;
	const GEN_FLT x35 = x34 * x12;
	const GEN_FLT x36 = (-1 * x26) + x27;
	const GEN_FLT x37 = 1 + (-1 * x12 * (x10 + x7));
	const GEN_FLT x38 = obj_px + (x35 * sensor_z) + (x36 * x17) + (x37 * sensor_x);
	const GEN_FLT x39 = lh_qw * lh_qj;
	const GEN_FLT x40 = lh_qk * lh_qi;
	const GEN_FLT x41 = (-1 * x39) + x40;
	const GEN_FLT x42 = x5 * x41;
	const GEN_FLT x43 = lh_pz + (x6 * x22) + (x33 * x29) + (x42 * x38);
	const GEN_FLT x44 = pow(x43, -1);
	const GEN_FLT x45 = 1 + (-1 * x3 * x5);
	const GEN_FLT x46 = lh_qw * lh_qk;
	const GEN_FLT x47 = lh_qj * lh_qi;
	const GEN_FLT x48 = (-1 * x46) + x47;
	const GEN_FLT x49 = 4 * x4 * x11;
	const GEN_FLT x50 = x49 * x28;
	const GEN_FLT x51 = x39 + x40;
	const GEN_FLT x52 = x51 * x49;
	const GEN_FLT x53 = (x45 * x37) + (x50 * x48) + (x52 * x20);
	const GEN_FLT x54 = x6 * x12;
	const GEN_FLT x55 = (x42 * x37) + (x50 * x32) + (x54 * x20);
	const GEN_FLT x56 = x43 * x43;
	const GEN_FLT x57 = pow(x56, -1);
	const GEN_FLT x58 = x5 * x22;
	const GEN_FLT x59 = x5 * x48;
	const GEN_FLT x60 = lh_px + (x59 * x29) + (x51 * x58) + (x45 * x38);
	const GEN_FLT x61 = x60 * x57;
	const GEN_FLT x62 = -1 * x43;
	const GEN_FLT x63 = x60 * x60;
	const GEN_FLT x64 = 2 * pow((x63 + x56), -1) * x56 * atan2(x60, x62) * curve_1;
	const GEN_FLT x65 = x46 + x47;
	const GEN_FLT x66 = x5 * x65;
	const GEN_FLT x67 = 1 + (-1 * (x2 + x1) * x5);
	const GEN_FLT x68 = (-1 * x30) + x31;
	const GEN_FLT x69 = x68 * x49;
	const GEN_FLT x70 = (x66 * x37) + (x67 * x28 * x12) + (x69 * x20);
	const GEN_FLT x71 = lh_py + (x68 * x58) + (x67 * x29) + (x66 * x38);
	const GEN_FLT x72 = x71 * x57;
	const GEN_FLT x73 = x56 + (x71 * x71);
	const GEN_FLT x74 = pow(x73, -1);
	const GEN_FLT x75 = x74 * x56;
	const GEN_FLT x76 = pow((1 + (-1 * x74 * x63 * (tilt_1 * tilt_1))), -1.0 / 2.0);
	const GEN_FLT x77 = pow(x73, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x78 = 2 * x71;
	const GEN_FLT x79 = 2 * x43;
	const GEN_FLT x80 = 1.0 / 2.0 * pow(x73, -3.0 / 2.0) * x60 * tilt_1;
	const GEN_FLT x81 = (-1 * ((x70 * x44) + (-1 * x72 * x55)) * x75) +
						(-1 * x76 * ((x77 * x53) + (-1 * ((x70 * x78) + (x79 * x55)) * x80)));
	const GEN_FLT x82 =
		sin(1.5707963267949 + gibPhase_1 + (-1 * asin(x77 * x60)) + (-1 * phase_1) + (-1 * atan2(-1 * x71, x62))) *
		gibMag_1;
	const GEN_FLT x83 = (x59 * x25) + (x45 * x36 * x12) + (x52 * x16);
	const GEN_FLT x84 = x41 * x49;
	const GEN_FLT x85 = (x84 * x36) + (x33 * x25) + (x54 * x16);
	const GEN_FLT x86 = x65 * x49;
	const GEN_FLT x87 = (x86 * x36) + (x67 * x25) + (x69 * x16);
	const GEN_FLT x88 = (-1 * ((x87 * x44) + (-1 * x85 * x72)) * x75) +
						(-1 * x76 * ((x83 * x77) + (-1 * ((x87 * x78) + (x85 * x79)) * x80)));
	const GEN_FLT x89 = x49 * x23;
	const GEN_FLT x90 = x5 * x13;
	const GEN_FLT x91 = (x45 * x35) + (x89 * x48) + (x51 * x90);
	const GEN_FLT x92 = (x84 * x34) + (x89 * x32) + (x6 * x13);
	const GEN_FLT x93 = (x86 * x34) + (x67 * x24) + (x68 * x90);
	const GEN_FLT x94 = (-1 * ((x93 * x44) + (-1 * x72 * x92)) * x75) +
						(-1 * x76 * ((x77 * x91) + (-1 * ((x78 * x93) + (x79 * x92)) * x80)));
	out[0] = (((-1 * x53 * x44) + (x61 * x55)) * x64) + (x81 * x82) + x81;
	out[1] = (((-1 * x83 * x44) + (x85 * x61)) * x64) + (x82 * x88) + x88;
	out[2] = (((-1 * x91 * x44) + (x61 * x92)) * x64) + (x82 * x94) + x94;
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
	const GEN_FLT x0 = lh_qj * lh_qj;
	const GEN_FLT x1 = lh_qi * lh_qi;
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = lh_qk * lh_qk;
	const GEN_FLT x4 = x3 + x0;
	const GEN_FLT x5 = sqrt((x1 + (lh_qw * lh_qw) + x4));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = x7 + x8;
	const GEN_FLT x10 = obj_qk * obj_qk;
	const GEN_FLT x11 = 2 * sqrt((x10 + (obj_qw * obj_qw) + x9));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 = obj_pz + ((1 + (-1 * x9 * x11)) * sensor_z) + ((x12 + x13) * x14) + (((-1 * x15) + x16) * x17);
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 =
		(((-1 * x12) + x13) * x22) + obj_py + ((1 + (-1 * x11 * (x10 + x8))) * sensor_y) + ((x23 + x24) * x17);
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = (-1 * x27) + x28;
	const GEN_FLT x30 =
		obj_px + ((x15 + x16) * x22) + (((-1 * x23) + x24) * x14) + ((1 + (-1 * x11 * (x10 + x7))) * sensor_x);
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + (x18 * (1 + (-1 * x2 * x6))) + (x21 * x26) + (x31 * x29);
	const GEN_FLT x33 = x32 * x32;
	const GEN_FLT x34 = (-1 * x19) + x20;
	const GEN_FLT x35 = x6 * x18;
	const GEN_FLT x36 = x3 + x1;
	const GEN_FLT x37 = lh_qw * lh_qk;
	const GEN_FLT x38 = lh_qj * lh_qi;
	const GEN_FLT x39 = x37 + x38;
	const GEN_FLT x40 = lh_py + (x34 * x35) + (x25 * (1 + (-1 * x6 * x36))) + (x31 * x39);
	const GEN_FLT x41 = x33 + (x40 * x40);
	const GEN_FLT x42 = pow(x41, -1);
	const GEN_FLT x43 = x27 + x28;
	const GEN_FLT x44 = (-1 * x37) + x38;
	const GEN_FLT x45 = lh_px + (x43 * x35) + (x30 * (1 + (-1 * x4 * x6))) + (x44 * x26);
	const GEN_FLT x46 = x45 * x45;
	const GEN_FLT x47 = pow((1 + (-1 * x42 * x46 * (tilt_1 * tilt_1))), -1.0 / 2.0);
	const GEN_FLT x48 = pow(x41, -1.0 / 2.0) * tilt_1;
	const GEN_FLT x49 = x47 * x48;
	const GEN_FLT x50 = 2 * x32;
	const GEN_FLT x51 = -1 * x32;
	const GEN_FLT x52 = pow((x46 + x33), -1) * atan2(x45, x51) * curve_1;
	const GEN_FLT x53 =
		sin(1.5707963267949 + gibPhase_1 + (-1 * asin(x45 * x48)) + (-1 * phase_1) + (-1 * atan2(-1 * x40, x51))) *
		gibMag_1;
	const GEN_FLT x54 = pow(x41, -3.0 / 2.0) * x45 * tilt_1;
	const GEN_FLT x55 = x54 * x47;
	const GEN_FLT x56 = (-1 * x42 * x32) + (x55 * x40);
	const GEN_FLT x57 = 2 * x52;
	const GEN_FLT x58 = (x40 * x42) + (x55 * x32);
	const GEN_FLT x59 = pow(x32, -1);
	const GEN_FLT x60 = 2 * pow(x5, -1);
	const GEN_FLT x61 = x4 * x60;
	const GEN_FLT x62 = x61 * x30;
	const GEN_FLT x63 = x44 * x25;
	const GEN_FLT x64 = x60 * x63;
	const GEN_FLT x65 = x26 * lh_qk;
	const GEN_FLT x66 = x60 * x18;
	const GEN_FLT x67 = x66 * x43;
	const GEN_FLT x68 = x35 * lh_qj;
	const GEN_FLT x69 = (-1 * x62 * lh_qw) + (x64 * lh_qw) + (-1 * x65) + (x67 * lh_qw) + x68;
	const GEN_FLT x70 = x30 * x29;
	const GEN_FLT x71 = x70 * x60;
	const GEN_FLT x72 = x25 * x21;
	const GEN_FLT x73 = x72 * x60;
	const GEN_FLT x74 = x26 * lh_qi;
	const GEN_FLT x75 = x31 * lh_qj;
	const GEN_FLT x76 = x2 * x60;
	const GEN_FLT x77 = x76 * x18;
	const GEN_FLT x78 = (x71 * lh_qw) + x74 + (x73 * lh_qw) + (-1 * x77 * lh_qw) + (-1 * x75);
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
	const GEN_FLT x89 = (x83 * lh_qw) + x84 + (-1 * x86 * lh_qw) + (x87 * lh_qw) + (-1 * x88);
	const GEN_FLT x90 = x79 * x40;
	const GEN_FLT x91 = x42 * x33;
	const GEN_FLT x92 = 2 * x40;
	const GEN_FLT x93 = 1.0 / 2.0 * x54;
	const GEN_FLT x94 = (-1 * ((x89 * x59) + (-1 * x78 * x90)) * x91) +
						(-1 * x47 * ((x69 * x48) + (-1 * ((x89 * x92) + (x78 * x50)) * x93)));
	const GEN_FLT x95 = x26 * lh_qj;
	const GEN_FLT x96 = x35 * lh_qk;
	const GEN_FLT x97 = (-1 * x62 * lh_qi) + (x64 * lh_qi) + x96 + x95 + (x67 * lh_qi);
	const GEN_FLT x98 = x26 * lh_qw;
	const GEN_FLT x99 = 4 * x5;
	const GEN_FLT x100 = -1 * x99 * lh_qi;
	const GEN_FLT x101 = (x71 * lh_qi) + x84 + x98 + (x18 * ((-1 * x76 * lh_qi) + x100)) + (x73 * lh_qi);
	const GEN_FLT x102 = x79 * x101;
	const GEN_FLT x103 = x35 * lh_qw;
	const GEN_FLT x104 = (x83 * lh_qi) + x75 + (x25 * ((-1 * x85 * lh_qi) + x100)) + (-1 * x103) + (x87 * lh_qi);
	const GEN_FLT x105 = (-1 * ((x59 * x104) + (-1 * x40 * x102)) * x91) +
						 (-1 * x47 * ((x97 * x48) + (-1 * ((x92 * x104) + (x50 * x101)) * x93)));
	const GEN_FLT x106 = -1 * x99 * lh_qj;
	const GEN_FLT x107 = x60 * lh_qj;
	const GEN_FLT x108 = (x30 * ((-1 * x61 * lh_qj) + x106)) + (x63 * x107) + x74 + (x67 * lh_qj) + x103;
	const GEN_FLT x109 = x31 * lh_qw;
	const GEN_FLT x110 = (x70 * x107) + (-1 * x109) + (x72 * x107) + x65 + (x18 * ((-1 * x76 * lh_qj) + x106));
	const GEN_FLT x111 = x31 * lh_qi;
	const GEN_FLT x112 = (x83 * lh_qj) + x111 + (-1 * x86 * lh_qj) + (x87 * lh_qj) + x96;
	const GEN_FLT x113 = (-1 * ((x59 * x112) + (-1 * x90 * x110)) * x91) +
						 (-1 * x47 * ((x48 * x108) + (-1 * ((x92 * x112) + (x50 * x110)) * x93)));
	const GEN_FLT x114 = -1 * x99 * lh_qk;
	const GEN_FLT x115 = x60 * lh_qk;
	const GEN_FLT x116 = (x30 * ((-1 * x61 * lh_qk) + x114)) + (-1 * x98) + x88 + (x63 * x115) + (x67 * lh_qk);
	const GEN_FLT x117 = (x70 * x115) + (x72 * x115) + x95 + x111 + (-1 * x77 * lh_qk);
	const GEN_FLT x118 = x109 + (x82 * x115) + (x87 * lh_qk) + (x25 * ((-1 * x85 * lh_qk) + x114)) + x68;
	const GEN_FLT x119 = (-1 * ((x59 * x118) + (-1 * x90 * x117)) * x91) +
						 (-1 * x47 * ((x48 * x116) + (-1 * ((x92 * x118) + (x50 * x117)) * x93)));
	out[0] = (-1 * x49) + (-1 * x50 * x52) + (-1 * x53 * x49);
	out[1] = (x53 * x56) + x56;
	out[2] = (x57 * x45) + (x53 * x58) + x58;
	out[3] = (((-1 * x69 * x59) + (x80 * x78)) * x81) + (x53 * x94) + x94;
	out[4] = (x81 * ((-1 * x59 * x97) + (x45 * x102))) + (x53 * x105) + x105;
	out[5] = (((-1 * x59 * x108) + (x80 * x110)) * x81) + (x53 * x113) + x113;
	out[6] = (((-1 * x59 * x116) + (x80 * x117)) * x81) + (x53 * x119) + x119;
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
	const GEN_FLT x0 = lh_qj * lh_qj;
	const GEN_FLT x1 = lh_qi * lh_qi;
	const GEN_FLT x2 = lh_qk * lh_qk;
	const GEN_FLT x3 = x2 + x1;
	const GEN_FLT x4 = 2 * sqrt((x0 + (lh_qw * lh_qw) + x3));
	const GEN_FLT x5 = obj_qj * obj_qj;
	const GEN_FLT x6 = obj_qi * obj_qi;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = x7 + x6;
	const GEN_FLT x9 = 2 * sqrt((x5 + (obj_qw * obj_qw) + x8));
	const GEN_FLT x10 = obj_qw * obj_qi;
	const GEN_FLT x11 = obj_qk * obj_qj;
	const GEN_FLT x12 = x9 * sensor_y;
	const GEN_FLT x13 = obj_qw * obj_qj;
	const GEN_FLT x14 = obj_qk * obj_qi;
	const GEN_FLT x15 = x9 * sensor_x;
	const GEN_FLT x16 =
		obj_pz + ((1 + (-1 * (x5 + x6) * x9)) * sensor_z) + ((x10 + x11) * x12) + (((-1 * x13) + x14) * x15);
	const GEN_FLT x17 = lh_qw * lh_qi;
	const GEN_FLT x18 = lh_qk * lh_qj;
	const GEN_FLT x19 = x9 * sensor_z;
	const GEN_FLT x20 = obj_qw * obj_qk;
	const GEN_FLT x21 = obj_qj * obj_qi;
	const GEN_FLT x22 = (((-1 * x10) + x11) * x19) + obj_py + ((1 + (-1 * x8 * x9)) * sensor_y) + ((x20 + x21) * x15);
	const GEN_FLT x23 = x4 * x22;
	const GEN_FLT x24 = lh_qw * lh_qj;
	const GEN_FLT x25 = lh_qk * lh_qi;
	const GEN_FLT x26 =
		obj_px + ((x13 + x14) * x19) + (((-1 * x20) + x21) * x12) + ((1 + (-1 * (x7 + x5) * x9)) * sensor_x);
	const GEN_FLT x27 = x4 * x26;
	const GEN_FLT x28 = lh_pz + ((x17 + x18) * x23) + (x16 * (1 + (-1 * (x0 + x1) * x4))) + (((-1 * x24) + x25) * x27);
	const GEN_FLT x29 = x4 * x16;
	const GEN_FLT x30 = lh_qw * lh_qk;
	const GEN_FLT x31 = lh_qj * lh_qi;
	const GEN_FLT x32 = lh_py + (((-1 * x17) + x18) * x29) + (x22 * (1 + (-1 * x4 * x3))) + ((x30 + x31) * x27);
	const GEN_FLT x33 = (x28 * x28) + (x32 * x32);
	const GEN_FLT x34 = lh_px + ((x24 + x25) * x29) + (((-1 * x30) + x31) * x23) + (x26 * (1 + (-1 * (x2 + x0) * x4)));
	const GEN_FLT x35 = x34 * pow(x33, -1.0 / 2.0);
	const GEN_FLT x36 = -1 * x28;
	const GEN_FLT x37 =
		1.5707963267949 + gibPhase_1 + (-1 * asin(x35 * tilt_1)) + (-1 * phase_1) + (-1 * atan2(-1 * x32, x36));
	const GEN_FLT x38 = sin(x37) * gibMag_1;
	const GEN_FLT x39 = x35 * pow((1 + (-1 * (x34 * x34) * pow(x33, -1) * (tilt_1 * tilt_1))), -1.0 / 2.0);
	out[0] = -1 + (-1 * x38);
	out[1] = (-1 * x39) + (-1 * x38 * x39);
	out[2] = atan2(x34, x36) * atan2(x34, x36);
	out[3] = x38;
	out[4] = -1 * cos(x37);
	out[5] = 0;
	out[6] = 0;
}
