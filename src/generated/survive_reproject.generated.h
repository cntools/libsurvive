#pragma once
#include "common.h"
/** Applying function <function reproject_gen2 at 0x7f6253d38a70> */
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 + (-1 * x1);
	const GEN_FLT x3 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x4 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 0;
	const GEN_FLT x5 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						   : 1e-10;
	const GEN_FLT x6 = cos(x5);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 1;
	const GEN_FLT x9 = sin(x5);
	const GEN_FLT x10 = x8 * x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x4 * x7 * x11;
	const GEN_FLT x13 = x9 * x11;
	const GEN_FLT x14 = x8 * x7;
	const GEN_FLT x15 = x4 * x14;
	const GEN_FLT x16 =
		((x15 + (-1 * x13)) * sensor_x) + ((x12 + x10) * sensor_y) + ((x6 + ((x4 * x4) * x7)) * sensor_z) + obj_pz;
	const GEN_FLT x17 = sin(x0);
	const GEN_FLT x18 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 1;
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 0;
	const GEN_FLT x21 = x2 * x3;
	const GEN_FLT x22 = x20 * x21;
	const GEN_FLT x23 = x4 * x9;
	const GEN_FLT x24 = x14 * x11;
	const GEN_FLT x25 =
		((x24 + x23) * sensor_x) + ((x6 + (x7 * (x11 * x11))) * sensor_y) + ((x12 + (-1 * x10)) * sensor_z) + obj_py;
	const GEN_FLT x26 = x20 * x17;
	const GEN_FLT x27 = x21 * x18;
	const GEN_FLT x28 =
		((x6 + ((x8 * x8) * x7)) * sensor_x) + ((x24 + (-1 * x23)) * sensor_y) + ((x15 + x13) * sensor_z) + obj_px;
	const GEN_FLT x29 = ((x27 + (-1 * x26)) * x28) + ((x22 + x19) * x25) + (x16 * (x1 + (x2 * (x3 * x3)))) + lh_pz;
	const GEN_FLT x30 = x3 * x17;
	const GEN_FLT x31 = x2 * x20 * x18;
	const GEN_FLT x32 = (x28 * (x1 + (x2 * (x18 * x18)))) + ((x31 + (-1 * x30)) * x25) + ((x27 + x26) * x16) + lh_px;
	const GEN_FLT x33 = atan2(-1 * x29, x32);
	const GEN_FLT x34 = 0.523598775598299 + tilt_0;
	const GEN_FLT x35 = cos(x34);
	const GEN_FLT x36 = ((x31 + x30) * x28) + (x25 * (x1 + (x2 * (x20 * x20)))) + ((x22 + (-1 * x19)) * x16) + lh_py;
	const GEN_FLT x37 = (x32 * x32) + (x29 * x29);
	const GEN_FLT x38 = x36 * (1. / sqrt(x37 + (x36 * x36)));
	const GEN_FLT x39 = asin((1. / x35) * x38);
	const GEN_FLT x40 = 0.0028679863 + (x39 * (-8.0108022e-06 + (-8.0108022e-06 * x39)));
	const GEN_FLT x41 = 5.3685255e-06 + (x40 * x39);
	const GEN_FLT x42 = 0.0076069798 + (x41 * x39);
	const GEN_FLT x43 = x36 * (1. / sqrt(x37));
	const GEN_FLT x44 = x43 * tan(x34);
	const GEN_FLT x45 = (sin(x33 + (-1 * asin(x44)) + ogeeMag_0) * ogeePhase_0) + curve_0;
	const GEN_FLT x46 =
		(-1 *
		 asin(x44 + (x42 * x45 * (x39 * x39) *
					 (1. / (x35 + (-1 * x45 * sin(x34) *
								   ((x39 * (x42 + (x39 * (x41 + (x39 * (x40 + (x39 * (-8.0108022e-06 +
																					  (-1.60216044e-05 * x39))))))))) +
									(x42 * x39)))))))) +
		x33;
	const GEN_FLT x47 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x48 = -1 * x43 * tan(x47);
	const GEN_FLT x49 = (sin(x33 + (-1 * asin(x48)) + ogeeMag_1) * ogeePhase_1) + curve_1;
	const GEN_FLT x50 = cos(x47);
	const GEN_FLT x51 = asin((1. / x50) * x38);
	const GEN_FLT x52 = 0.0028679863 + (x51 * (-8.0108022e-06 + (-8.0108022e-06 * x51)));
	const GEN_FLT x53 = 5.3685255e-06 + (x52 * x51);
	const GEN_FLT x54 = 0.0076069798 + (x53 * x51);
	const GEN_FLT x55 =
		x33 +
		(-1 *
		 asin(x48 + (x54 * (x51 * x51) * x49 *
					 (1. / (x50 + (x49 * sin(x47) *
								   ((x51 * (x54 + (x51 * (x53 + (x51 * (x52 + (x51 * (-8.0108022e-06 +
																					  (-1.60216044e-05 * x51))))))))) +
									(x54 * x51))))))));
	out[0] = -1.5707963267949 + x46 + (-1 * phase_0) + (sin(x46 + gibPhase_0) * gibMag_0);
	out[1] = -1.5707963267949 + x55 + (sin(x55 + gibPhase_1) * gibMag_1) + (-1 * phase_1);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x5 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x9 + x3;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x20 * x11;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x15 * x11;
	const GEN_FLT x24 = x23 * x16;
	const GEN_FLT x25 =
		((x24 + (-1 * x22)) * sensor_x) + ((x21 + x18) * sensor_y) + ((x14 + (x15 * x12)) * sensor_z) + obj_pz;
	const GEN_FLT x26 = x1 * x4;
	const GEN_FLT x27 = x2 * x8;
	const GEN_FLT x28 = x27 + (-1 * x26);
	const GEN_FLT x29 = x19 * x19;
	const GEN_FLT x30 = x11 * x17;
	const GEN_FLT x31 = x20 * x16;
	const GEN_FLT x32 =
		((x31 + x30) * sensor_x) + ((x14 + (x29 * x15)) * sensor_y) + ((x21 + (-1 * x18)) * sensor_z) + obj_py;
	const GEN_FLT x33 = x5 * x5;
	const GEN_FLT x34 = x6 + (x7 * x33);
	const GEN_FLT x35 = x16 * x16;
	const GEN_FLT x36 =
		((x14 + (x35 * x15)) * sensor_x) + ((x31 + (-1 * x30)) * sensor_y) + ((x24 + x22) * sensor_z) + obj_px;
	const GEN_FLT x37 = (x34 * x36) + (x32 * x28) + (x25 * x10) + lh_px;
	const GEN_FLT x38 = 1. / x37;
	const GEN_FLT x39 = x9 + (-1 * x3);
	const GEN_FLT x40 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x41 = x15 * x16;
	const GEN_FLT x42 = 2 * x41;
	const GEN_FLT x43 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x44 = x43 * x17;
	const GEN_FLT x45 = -1 * x44;
	const GEN_FLT x46 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x47 = x46 * x17;
	const GEN_FLT x48 = x14 * x11;
	const GEN_FLT x49 = x43 * x48;
	const GEN_FLT x50 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x51 = (x40 * x20) + (x43 * x19 * x18) + (x50 * x41);
	const GEN_FLT x52 = x50 * x17;
	const GEN_FLT x53 = x43 * x14;
	const GEN_FLT x54 = x53 * x19;
	const GEN_FLT x55 = x43 * x11;
	const GEN_FLT x56 = (x41 * x46) + (x55 * x18) + (x40 * x23);
	const GEN_FLT x57 = ((x56 + x54 + x52) * sensor_z) + ((x51 + (-1 * x49) + (-1 * x47)) * sensor_y) +
						(((x44 * x35) + x45 + (x40 * x42)) * sensor_x);
	const GEN_FLT x58 = 1 + x57;
	const GEN_FLT x59 = x4 * x4;
	const GEN_FLT x60 = x6 + (x7 * x59);
	const GEN_FLT x61 = x40 * x17;
	const GEN_FLT x62 = x53 * x16;
	const GEN_FLT x63 = (x55 * x22) + (x46 * x20) + (x50 * x23);
	const GEN_FLT x64 = 2 * x23;
	const GEN_FLT x65 = (((x44 * x12) + (x64 * x46) + x45) * sensor_z) + ((x63 + x62 + x61) * sensor_y) +
						((x56 + (-1 * x54) + (-1 * x52)) * sensor_x);
	const GEN_FLT x66 = x60 * x65;
	const GEN_FLT x67 = 2 * x20;
	const GEN_FLT x68 = ((x63 + (-1 * x62) + (-1 * x61)) * sensor_z) + (((x44 * x29) + (x67 * x50) + x45) * sensor_y) +
						((x51 + x49 + x47) * sensor_x);
	const GEN_FLT x69 = x1 * x5;
	const GEN_FLT x70 = x2 * x7;
	const GEN_FLT x71 = x4 * x70;
	const GEN_FLT x72 = x71 + x69;
	const GEN_FLT x73 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x74 = x6 * x73;
	const GEN_FLT x75 = x2 * x74;
	const GEN_FLT x76 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x77 = x1 * x76;
	const GEN_FLT x78 = x1 * x73;
	const GEN_FLT x79 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x80 = x4 * x7;
	const GEN_FLT x81 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x82 = (x8 * x81) + (x80 * x79) + (x4 * x5 * x78);
	const GEN_FLT x83 = -1 * x78;
	const GEN_FLT x84 = x5 * x74;
	const GEN_FLT x85 = x1 * x79;
	const GEN_FLT x86 = x2 * x78;
	const GEN_FLT x87 = (x80 * x76) + (x81 * x70) + (x4 * x86);
	const GEN_FLT x88 = (x32 * (x87 + x85 + x84)) + (x25 * ((2 * x80 * x81) + (x78 * x59) + x83)) +
						(x36 * (x82 + (-1 * x77) + (-1 * x75)));
	const GEN_FLT x89 = x88 + (x72 * x68);
	const GEN_FLT x90 = x89 + x66 + (x58 * x39);
	const GEN_FLT x91 = x68 * x28;
	const GEN_FLT x92 = x4 * x74;
	const GEN_FLT x93 = x1 * x81;
	const GEN_FLT x94 = (x5 * x86) + (x8 * x76) + (x70 * x79);
	const GEN_FLT x95 = (x32 * (x94 + (-1 * x93) + (-1 * x92))) + (x25 * (x82 + x77 + x75)) +
						(x36 * ((2 * x8 * x79) + (x78 * x33) + x83));
	const GEN_FLT x96 = x95 + (x65 * x10);
	const GEN_FLT x97 = x96 + (x58 * x34) + x91;
	const GEN_FLT x98 = x37 * x37;
	const GEN_FLT x99 = (x36 * x39) + (x72 * x32) + (x60 * x25) + lh_pz;
	const GEN_FLT x100 = x99 * (1. / x98);
	const GEN_FLT x101 = x98 + (x99 * x99);
	const GEN_FLT x102 = 1. / x101;
	const GEN_FLT x103 = x98 * x102;
	const GEN_FLT x104 = x103 * ((x97 * x100) + (-1 * x90 * x38));
	const GEN_FLT x105 = x27 + x26;
	const GEN_FLT x106 = x71 + (-1 * x69);
	const GEN_FLT x107 = x65 * x106;
	const GEN_FLT x108 = x2 * x2;
	const GEN_FLT x109 = x6 + (x7 * x108);
	const GEN_FLT x110 = (x25 * (x87 + (-1 * x85) + (-1 * x84))) + (x32 * ((2 * x70 * x76) + (x78 * x108) + x83)) +
						 (x36 * (x94 + x93 + x92));
	const GEN_FLT x111 = x110 + (x68 * x109);
	const GEN_FLT x112 = x111 + x107 + (x58 * x105);
	const GEN_FLT x113 = (x36 * x105) + (x32 * x109) + (x25 * x106) + lh_py;
	const GEN_FLT x114 = 2 * x113;
	const GEN_FLT x115 = 2 * x37;
	const GEN_FLT x116 = 2 * x99;
	const GEN_FLT x117 = (x90 * x116) + (x97 * x115);
	const GEN_FLT x118 = x117 + (x112 * x114);
	const GEN_FLT x119 = 0.523598775598299 + tilt_0;
	const GEN_FLT x120 = cos(x119);
	const GEN_FLT x121 = 1. / x120;
	const GEN_FLT x122 = x113 * x113;
	const GEN_FLT x123 = x101 + x122;
	const GEN_FLT x124 = 1.0 / 2.0 * x113;
	const GEN_FLT x125 = x124 * (1. / (x123 * sqrt(x123)));
	const GEN_FLT x126 = x121 * x125;
	const GEN_FLT x127 = 1. / sqrt(x123);
	const GEN_FLT x128 = x121 * x127;
	const GEN_FLT x129 = (1. / x123) * x122;
	const GEN_FLT x130 = 1. / sqrt(1 + (-1 * (1. / (x120 * x120)) * x129));
	const GEN_FLT x131 = ((x112 * x128) + (-1 * x118 * x126)) * x130;
	const GEN_FLT x132 = 1. / sqrt(x101);
	const GEN_FLT x133 = tan(x119);
	const GEN_FLT x134 = x133 * x132;
	const GEN_FLT x135 = x113 * x134;
	const GEN_FLT x136 = atan2(-1 * x99, x37);
	const GEN_FLT x137 = x136 + (-1 * asin(x135)) + ogeeMag_0;
	const GEN_FLT x138 = (sin(x137) * ogeePhase_0) + curve_0;
	const GEN_FLT x139 = asin(x113 * x128);
	const GEN_FLT x140 = 8.0108022e-06 * x139;
	const GEN_FLT x141 = -8.0108022e-06 + (-1 * x140);
	const GEN_FLT x142 = 0.0028679863 + (x139 * x141);
	const GEN_FLT x143 = 5.3685255e-06 + (x139 * x142);
	const GEN_FLT x144 = 0.0076069798 + (x139 * x143);
	const GEN_FLT x145 = x139 * x144;
	const GEN_FLT x146 = -8.0108022e-06 + (-1.60216044e-05 * x139);
	const GEN_FLT x147 = x142 + (x139 * x146);
	const GEN_FLT x148 = x143 + (x139 * x147);
	const GEN_FLT x149 = x144 + (x139 * x148);
	const GEN_FLT x150 = (x139 * x149) + x145;
	const GEN_FLT x151 = sin(x119);
	const GEN_FLT x152 = x138 * x151;
	const GEN_FLT x153 = x120 + (-1 * x150 * x152);
	const GEN_FLT x154 = 1. / x153;
	const GEN_FLT x155 = 2 * x138 * x145 * x154;
	const GEN_FLT x156 = (1. / (x101 * sqrt(x101))) * x124;
	const GEN_FLT x157 = x133 * x156;
	const GEN_FLT x158 = (x112 * x134) + (-1 * x117 * x157);
	const GEN_FLT x159 = x102 * x122;
	const GEN_FLT x160 = 1. / sqrt(1 + (-1 * (x133 * x133) * x159));
	const GEN_FLT x161 = cos(x137) * ogeePhase_0;
	const GEN_FLT x162 = x161 * ((-1 * x160 * x158) + x104);
	const GEN_FLT x163 = x139 * x139;
	const GEN_FLT x164 = x163 * x144 * x154;
	const GEN_FLT x165 = x131 * x141;
	const GEN_FLT x166 = 2.40324066e-05 * x139;
	const GEN_FLT x167 = (x139 * (x165 + (-1 * x131 * x140))) + (x131 * x142);
	const GEN_FLT x168 = (x167 * x139) + (x131 * x143);
	const GEN_FLT x169 = x150 * x151;
	const GEN_FLT x170 = x163 * x138;
	const GEN_FLT x171 = x170 * x144 * (1. / (x153 * x153));
	const GEN_FLT x172 = x170 * x154;
	const GEN_FLT x173 = x135 + (x172 * x144);
	const GEN_FLT x174 = 1. / sqrt(1 + (-1 * (x173 * x173)));
	const GEN_FLT x175 =
		(-1 * x174 *
		 ((x168 * x172) + x158 +
		  (-1 * x171 *
		   ((-1 * x169 * x162) +
			(-1 * x152 *
			 ((x131 * x144) +
			  (x139 * (x168 + (x139 * (x167 + (x139 * ((x131 * x146) + (-1 * x166 * x131) + x165)) + (x131 * x147))) +
					   (x131 * x148))) +
			  (x168 * x139) + (x131 * x149))))) +
		  (x164 * x162) + (x131 * x155))) +
		x104;
	const GEN_FLT x176 = cos(x136 + (-1 * asin(x173)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x177 = x57 * x39;
	const GEN_FLT x178 = 1 + x68;
	const GEN_FLT x179 = x88 + (x72 * x178) + x66 + x177;
	const GEN_FLT x180 = x57 * x34;
	const GEN_FLT x181 = x96 + (x28 * x178) + x180;
	const GEN_FLT x182 = x103 * ((x100 * x181) + (-1 * x38 * x179));
	const GEN_FLT x183 = x57 * x105;
	const GEN_FLT x184 = (x109 * x178) + x107 + x110 + x183;
	const GEN_FLT x185 = (x116 * x179) + (x115 * x181);
	const GEN_FLT x186 = x185 + (x114 * x184);
	const GEN_FLT x187 = ((x128 * x184) + (-1 * x126 * x186)) * x130;
	const GEN_FLT x188 = x187 * x141;
	const GEN_FLT x189 = (x139 * (x188 + (-1 * x187 * x140))) + (x187 * x142);
	const GEN_FLT x190 = (x189 * x139) + (x187 * x143);
	const GEN_FLT x191 = (x184 * x134) + (-1 * x185 * x157);
	const GEN_FLT x192 = (-1 * x160 * x191) + x182;
	const GEN_FLT x193 = x161 * x169;
	const GEN_FLT x194 = x161 * x164;
	const GEN_FLT x195 =
		(-1 * x174 *
		 (x191 + (x192 * x194) + (x172 * x190) +
		  (-1 * x171 *
		   ((-1 * x192 * x193) +
			(-1 * x152 *
			 ((x190 * x139) + (x187 * x144) +
			  (x139 * (x190 + (x139 * (x189 + (x139 * ((x187 * x146) + (-1 * x166 * x187) + x188)) + (x187 * x147))) +
					   (x187 * x148))) +
			  (x187 * x149))))) +
		  (x187 * x155))) +
		x182;
	const GEN_FLT x196 = 1 + x65;
	const GEN_FLT x197 = x89 + (x60 * x196) + x177;
	const GEN_FLT x198 = x95 + (x10 * x196) + x91 + x180;
	const GEN_FLT x199 = x103 * ((x100 * x198) + (-1 * x38 * x197));
	const GEN_FLT x200 = x111 + (x106 * x196) + x183;
	const GEN_FLT x201 = (x116 * x197) + (x115 * x198);
	const GEN_FLT x202 = x201 + (x200 * x114);
	const GEN_FLT x203 = (x200 * x128) + (-1 * x202 * x126);
	const GEN_FLT x204 = x203 * x130;
	const GEN_FLT x205 = x204 * x141;
	const GEN_FLT x206 = (x139 * (x205 + (-1 * x204 * x140))) + (x204 * x142);
	const GEN_FLT x207 = (x206 * x139) + (x204 * x143);
	const GEN_FLT x208 = x130 * x144;
	const GEN_FLT x209 = (x200 * x134) + (-1 * x201 * x157);
	const GEN_FLT x210 = (-1 * x209 * x160) + x199;
	const GEN_FLT x211 =
		(-1 * x174 *
		 (x209 + (x210 * x194) + (x207 * x172) +
		  (-1 * x171 *
		   ((-1 * x210 * x193) +
			(-1 * x152 *
			 ((x203 * x208) +
			  (x139 * (x207 + (x139 * (x206 + (x139 * ((x204 * x146) + (-1 * x204 * x166) + x205)) + (x204 * x147))) +
					   (x204 * x148))) +
			  (x207 * x139) + (x204 * x149))))) +
		  (x204 * x155))) +
		x199;
	const GEN_FLT x212 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x213 = x17 * x212;
	const GEN_FLT x214 = -1 * x213;
	const GEN_FLT x215 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qi *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x216 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qk *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x217 = x17 * x216;
	const GEN_FLT x218 = x48 * x212;
	const GEN_FLT x219 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qj *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x220 = x19 * x16;
	const GEN_FLT x221 = (x213 * x220) + (x41 * x219) + (x20 * x215);
	const GEN_FLT x222 = x17 * x219;
	const GEN_FLT x223 = x14 * x19;
	const GEN_FLT x224 = x212 * x223;
	const GEN_FLT x225 = x11 * x16;
	const GEN_FLT x226 = (x213 * x225) + (x41 * x216) + (x23 * x215);
	const GEN_FLT x227 = ((x226 + x224 + x222) * sensor_z) + (((-1 * x218) + x221 + (-1 * x217)) * sensor_y) +
						 (((x35 * x213) + (x42 * x215) + x214) * sensor_x);
	const GEN_FLT x228 = x17 * x215;
	const GEN_FLT x229 = x14 * x16;
	const GEN_FLT x230 = x212 * x229;
	const GEN_FLT x231 = x11 * x19;
	const GEN_FLT x232 = (x213 * x231) + (x20 * x216) + (x23 * x219);
	const GEN_FLT x233 = ((x232 + (-1 * x230) + (-1 * x228)) * sensor_z) +
						 (((x29 * x213) + x214 + (x67 * x219)) * sensor_y) + ((x218 + x221 + x217) * sensor_x);
	const GEN_FLT x234 = (((x12 * x213) + (x64 * x216) + x214) * sensor_z) + ((x232 + x230 + x228) * sensor_y) +
						 ((x226 + (-1 * x224) + (-1 * x222)) * sensor_x);
	const GEN_FLT x235 = x88 + (x60 * x234) + (x72 * x233) + (x39 * x227);
	const GEN_FLT x236 = x95 + (x10 * x234) + (x28 * x233) + (x34 * x227);
	const GEN_FLT x237 = x103 * ((x236 * x100) + (-1 * x38 * x235));
	const GEN_FLT x238 = (x234 * x106) + x110 + (x233 * x109) + (x227 * x105);
	const GEN_FLT x239 = (x235 * x116) + (x236 * x115);
	const GEN_FLT x240 = x239 + (x238 * x114);
	const GEN_FLT x241 = x238 * x127;
	const GEN_FLT x242 = (x241 * x121) + (-1 * x240 * x126);
	const GEN_FLT x243 = x242 * x130;
	const GEN_FLT x244 = x243 * x141;
	const GEN_FLT x245 = (x139 * (x244 + (-1 * x243 * x140))) + (x243 * x142);
	const GEN_FLT x246 = (x245 * x139) + (x243 * x143);
	const GEN_FLT x247 = (x238 * x134) + (-1 * x239 * x157);
	const GEN_FLT x248 = (-1 * x247 * x160) + x237;
	const GEN_FLT x249 =
		(-1 * x174 *
		 (x247 + (x248 * x194) + (x246 * x172) +
		  (-1 * x171 *
		   ((-1 * x248 * x193) +
			(-1 * x152 *
			 ((x246 * x139) + (x208 * x242) +
			  (x139 * (x246 + (x139 * (x245 + (x139 * ((x243 * x146) + (-1 * x243 * x166) + x244)) + (x243 * x147))) +
					   (x243 * x148))) +
			  (x243 * x149))))) +
		  (x243 * x155))) +
		x237;
	const GEN_FLT x250 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x251 = x17 * x250;
	const GEN_FLT x252 = -1 * x251;
	const GEN_FLT x253 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qi *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x254 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qk *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x255 = x17 * x254;
	const GEN_FLT x256 = x48 * x250;
	const GEN_FLT x257 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qj *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x258 = (x251 * x220) + (x41 * x257) + (x20 * x253);
	const GEN_FLT x259 = x17 * x257;
	const GEN_FLT x260 = x250 * x223;
	const GEN_FLT x261 = (x41 * x254) + (x251 * x225) + (x23 * x253);
	const GEN_FLT x262 = ((x261 + x260 + x259) * sensor_z) + ((x258 + (-1 * x256) + (-1 * x255)) * sensor_y) +
						 (((x42 * x253) + (x35 * x251) + x252) * sensor_x);
	const GEN_FLT x263 = x17 * x253;
	const GEN_FLT x264 = x250 * x229;
	const GEN_FLT x265 = (x23 * x257) + (x231 * x251) + (x20 * x254);
	const GEN_FLT x266 = ((x265 + (-1 * x264) + (-1 * x263)) * sensor_z) + ((x258 + x256 + x255) * sensor_x) +
						 (((x29 * x251) + (x67 * x257) + x252) * sensor_y);
	const GEN_FLT x267 = (((x12 * x251) + (x64 * x254) + x252) * sensor_z) + ((x265 + x264 + x263) * sensor_y) +
						 ((x261 + (-1 * x260) + (-1 * x259)) * sensor_x);
	const GEN_FLT x268 = x88 + (x60 * x267) + (x72 * x266) + (x39 * x262);
	const GEN_FLT x269 = x95 + (x10 * x267) + (x28 * x266) + (x34 * x262);
	const GEN_FLT x270 = x103 * ((x269 * x100) + (-1 * x38 * x268));
	const GEN_FLT x271 = x110 + (x267 * x106) + (x266 * x109) + (x262 * x105);
	const GEN_FLT x272 = (x268 * x116) + (x269 * x115);
	const GEN_FLT x273 = x272 + (x271 * x114);
	const GEN_FLT x274 = ((x271 * x128) + (-1 * x273 * x126)) * x130;
	const GEN_FLT x275 = x274 * x141;
	const GEN_FLT x276 = (x139 * (x275 + (-1 * x274 * x140))) + (x274 * x142);
	const GEN_FLT x277 = (x276 * x139) + (x274 * x143);
	const GEN_FLT x278 = (x271 * x134) + (-1 * x272 * x157);
	const GEN_FLT x279 = (-1 * x278 * x160) + x270;
	const GEN_FLT x280 =
		(-1 * x174 *
		 (x278 + (x279 * x194) + (x277 * x172) +
		  (-1 * x171 *
		   ((-1 * x279 * x193) +
			(-1 * x152 *
			 ((x274 * x144) +
			  (x139 * (x277 + (x139 * (x276 + (x139 * ((x274 * x146) + (-1 * x274 * x166) + x275)) + (x274 * x147))) +
					   (x274 * x148))) +
			  (x277 * x139) + (x274 * x149))))) +
		  (x274 * x155))) +
		x270;
	const GEN_FLT x281 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x282 = x17 * x281;
	const GEN_FLT x283 = -1 * x282;
	const GEN_FLT x284 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qi *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x285 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qk *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x286 = x17 * x285;
	const GEN_FLT x287 = x48 * x281;
	const GEN_FLT x288 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qj *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x289 = (x282 * x220) + (x41 * x288) + (x20 * x284);
	const GEN_FLT x290 = x17 * x288;
	const GEN_FLT x291 = x281 * x223;
	const GEN_FLT x292 = x11 * x282;
	const GEN_FLT x293 = (x41 * x285) + (x16 * x292) + (x23 * x284);
	const GEN_FLT x294 = ((x293 + x291 + x290) * sensor_z) + ((x289 + (-1 * x287) + (-1 * x286)) * sensor_y) +
						 (((x35 * x282) + (x42 * x284) + x283) * sensor_x);
	const GEN_FLT x295 = x17 * x284;
	const GEN_FLT x296 = x281 * x229;
	const GEN_FLT x297 = (x19 * x292) + (x20 * x285) + (x23 * x288);
	const GEN_FLT x298 = ((x297 + (-1 * x296) + (-1 * x295)) * sensor_z) +
						 ((x283 + (x29 * x282) + (x67 * x288)) * sensor_y) + ((x289 + x287 + x286) * sensor_x);
	const GEN_FLT x299 = ((x297 + x296 + x295) * sensor_y) + (((x12 * x282) + (x64 * x285) + x283) * sensor_z) +
						 ((x293 + (-1 * x291) + (-1 * x290)) * sensor_x);
	const GEN_FLT x300 = x88 + (x60 * x299) + (x72 * x298) + (x39 * x294);
	const GEN_FLT x301 = x95 + (x10 * x299) + (x28 * x298) + (x34 * x294);
	const GEN_FLT x302 = x103 * ((x301 * x100) + (-1 * x38 * x300));
	const GEN_FLT x303 = (x299 * x106) + (x298 * x109) + x110 + (x294 * x105);
	const GEN_FLT x304 = (x300 * x116) + (x301 * x115);
	const GEN_FLT x305 = x304 + (x303 * x114);
	const GEN_FLT x306 = x303 * x127;
	const GEN_FLT x307 = ((x306 * x121) + (-1 * x305 * x126)) * x130;
	const GEN_FLT x308 = (x303 * x134) + (-1 * x304 * x157);
	const GEN_FLT x309 = (-1 * x308 * x160) + x302;
	const GEN_FLT x310 = x307 * x141;
	const GEN_FLT x311 = (x139 * (x310 + (-1 * x307 * x140))) + (x307 * x142);
	const GEN_FLT x312 = (x311 * x139) + (x307 * x143);
	const GEN_FLT x313 =
		(-1 * x174 *
		 ((x312 * x172) +
		  (-1 * x171 *
		   ((-1 * x309 * x193) +
			(-1 * x152 *
			 ((x312 * x139) + (x307 * x144) +
			  (x139 * (x312 + (x139 * (x311 + (x139 * ((x307 * x146) + (-1 * x307 * x166) + x310)) + (x307 * x147))) +
					   (x307 * x148))) +
			  (x307 * x149))))) +
		  x308 + (x309 * x194) + (x307 * x155))) +
		x302;
	const GEN_FLT x314 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x315 = cos(x314);
	const GEN_FLT x316 = 1. / x315;
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
	const GEN_FLT x328 = x136 + (-1 * asin(x327)) + ogeeMag_1;
	const GEN_FLT x329 = (sin(x328) * ogeePhase_1) + curve_1;
	const GEN_FLT x330 = x323 * x318;
	const GEN_FLT x331 = -8.0108022e-06 + (-1.60216044e-05 * x318);
	const GEN_FLT x332 = x321 + (x331 * x318);
	const GEN_FLT x333 = x322 + (x332 * x318);
	const GEN_FLT x334 = x323 + (x333 * x318);
	const GEN_FLT x335 = (x334 * x318) + x330;
	const GEN_FLT x336 = sin(x314);
	const GEN_FLT x337 = x336 * x329;
	const GEN_FLT x338 = x315 + (x337 * x335);
	const GEN_FLT x339 = 1. / x338;
	const GEN_FLT x340 = x339 * x329;
	const GEN_FLT x341 = x324 * x340;
	const GEN_FLT x342 = x327 + (x323 * x341);
	const GEN_FLT x343 = 1. / sqrt(1 + (-1 * (x342 * x342)));
	const GEN_FLT x344 = x325 * x156;
	const GEN_FLT x345 = (-1 * x326 * x112) + (x344 * x117);
	const GEN_FLT x346 = 1. / sqrt(1 + (-1 * (x325 * x325) * x159));
	const GEN_FLT x347 = (-1 * x346 * x345) + x104;
	const GEN_FLT x348 = cos(x328) * ogeePhase_1;
	const GEN_FLT x349 = x323 * x324;
	const GEN_FLT x350 = x339 * x349;
	const GEN_FLT x351 = x350 * x348;
	const GEN_FLT x352 = 1. / sqrt(1 + (-1 * (1. / (x315 * x315)) * x129));
	const GEN_FLT x353 = x316 * x125;
	const GEN_FLT x354 = (x317 * x112) + (-1 * x353 * x118);
	const GEN_FLT x355 = x352 * x354;
	const GEN_FLT x356 = 2 * x330 * x340;
	const GEN_FLT x357 = x355 * x320;
	const GEN_FLT x358 = x352 * x321;
	const GEN_FLT x359 = (x354 * x358) + (x318 * (x357 + (-1 * x355 * x319)));
	const GEN_FLT x360 = (x359 * x318) + (x355 * x322);
	const GEN_FLT x361 = x336 * x335;
	const GEN_FLT x362 = x361 * x348;
	const GEN_FLT x363 = x352 * x334;
	const GEN_FLT x364 = x352 * x333;
	const GEN_FLT x365 = 2.40324066e-05 * x318;
	const GEN_FLT x366 = x352 * x331;
	const GEN_FLT x367 = x352 * x332;
	const GEN_FLT x368 = (1. / (x338 * x338)) * x329 * x349;
	const GEN_FLT x369 =
		(-1 * x343 *
		 (x345 +
		  (-1 * x368 *
		   ((x337 *
			 ((x360 * x318) +
			  (x318 * (x360 + (x318 * (x359 + (x367 * x354) + (x318 * ((x366 * x354) + (-1 * x365 * x355) + x357)))) +
					   (x364 * x354))) +
			  (x355 * x323) + (x363 * x354))) +
			(x362 * x347))) +
		  (x360 * x341) + (x356 * x355) + (x351 * x347))) +
		x104;
	const GEN_FLT x370 = cos((-1 * asin(x342)) + x136 + gibPhase_1) * gibMag_1;
	const GEN_FLT x371 = (-1 * x326 * x184) + (x344 * x185);
	const GEN_FLT x372 = x348 * ((-1 * x371 * x346) + x182);
	const GEN_FLT x373 = (x317 * x184) + (-1 * x353 * x186);
	const GEN_FLT x374 = x373 * x352;
	const GEN_FLT x375 = x374 * x320;
	const GEN_FLT x376 = (x373 * x358) + (x318 * (x375 + (-1 * x374 * x319)));
	const GEN_FLT x377 = (x376 * x318) + (x374 * x322);
	const GEN_FLT x378 =
		(-1 * x343 *
		 (x371 +
		  (-1 * x368 *
		   ((x337 *
			 ((x374 * x323) + (x377 * x318) +
			  (x318 * (x377 + (x318 * ((x373 * x367) + x376 + (x318 * ((x373 * x366) + (-1 * x374 * x365) + x375)))) +
					   (x373 * x364))) +
			  (x374 * x334))) +
			(x372 * x361))) +
		  (x377 * x341) + (x374 * x356) + (x372 * x350))) +
		x182;
	const GEN_FLT x379 = (-1 * x200 * x326) + (x201 * x344);
	const GEN_FLT x380 = (-1 * x379 * x346) + x199;
	const GEN_FLT x381 = (x200 * x317) + (-1 * x202 * x353);
	const GEN_FLT x382 = x352 * x322;
	const GEN_FLT x383 = x381 * x352;
	const GEN_FLT x384 = x352 * x320;
	const GEN_FLT x385 = x384 * x381;
	const GEN_FLT x386 = (x381 * x358) + (x318 * (x385 + (-1 * x383 * x319)));
	const GEN_FLT x387 = (x386 * x318) + (x381 * x382);
	const GEN_FLT x388 = x352 * x323;
	const GEN_FLT x389 =
		(-1 * x343 *
		 (x379 +
		  (-1 * x368 *
		   ((x337 *
			 ((x381 * x388) + (x387 * x318) + (x363 * x381) +
			  (x318 * (x387 + (x318 * (x386 + (x367 * x381) + (x318 * ((x366 * x381) + (-1 * x365 * x383) + x385)))) +
					   (x364 * x381))))) +
			(x362 * x380))) +
		  (x383 * x356) + (x387 * x341) + (x351 * x380))) +
		x199;
	const GEN_FLT x390 = (-1 * x238 * x326) + (x239 * x344);
	const GEN_FLT x391 = (-1 * x390 * x346) + x237;
	const GEN_FLT x392 = (x241 * x316) + (-1 * x240 * x353);
	const GEN_FLT x393 = x392 * x352;
	const GEN_FLT x394 = x392 * x384;
	const GEN_FLT x395 = (x392 * x358) + (x318 * (x394 + (-1 * x393 * x319)));
	const GEN_FLT x396 = (x395 * x318) + (x392 * x382);
	const GEN_FLT x397 =
		(-1 * x343 *
		 (x390 +
		  (-1 * x368 *
		   ((x337 *
			 ((x392 * x388) + (x396 * x318) +
			  (x318 * (x396 + (x318 * (x395 + (x392 * x367) + (x318 * ((x392 * x366) + (-1 * x393 * x365) + x394)))) +
					   (x392 * x364))) +
			  (x392 * x363))) +
			(x391 * x362))) +
		  (x396 * x341) + (x393 * x356) + (x391 * x351))) +
		x237;
	const GEN_FLT x398 = (-1 * x271 * x326) + (x272 * x344);
	const GEN_FLT x399 = (-1 * x398 * x346) + x270;
	const GEN_FLT x400 = (x271 * x317) + (-1 * x273 * x353);
	const GEN_FLT x401 = x400 * x352;
	const GEN_FLT x402 = x400 * x384;
	const GEN_FLT x403 = (x400 * x358) + (x318 * (x402 + (-1 * x401 * x319)));
	const GEN_FLT x404 = (x403 * x318) + (x400 * x382);
	const GEN_FLT x405 =
		(-1 * x343 *
		 (x398 +
		  (-1 * x368 *
		   ((x337 *
			 ((x400 * x388) + (x404 * x318) + (x400 * x363) +
			  (x318 * (x404 + (x318 * (x403 + (x400 * x367) + (x318 * ((x400 * x366) + x402 + (-1 * x401 * x365))))) +
					   (x400 * x364))))) +
			(x399 * x362))) +
		  (x401 * x356) + (x404 * x341) + (x399 * x351))) +
		x270;
	const GEN_FLT x406 = (-1 * x303 * x326) + (x304 * x344);
	const GEN_FLT x407 = (-1 * x406 * x346) + x302;
	const GEN_FLT x408 = (x306 * x316) + (-1 * x353 * x305);
	const GEN_FLT x409 = x408 * x352;
	const GEN_FLT x410 = x409 * x320;
	const GEN_FLT x411 = (x408 * x358) + (x318 * (x410 + (-1 * x409 * x319)));
	const GEN_FLT x412 = (x411 * x318) + (x409 * x322);
	const GEN_FLT x413 =
		(-1 * x343 *
		 (x406 + (x409 * x356) +
		  (-1 * x368 *
		   ((x337 *
			 ((x409 * x323) + (x409 * x334) + (x412 * x318) +
			  (x318 * (x412 + (x318 * ((x408 * x367) + x411 + (x318 * ((x408 * x366) + (-1 * x409 * x365) + x410)))) +
					   (x408 * x364))))) +
			(x407 * x362))) +
		  (x412 * x341) + (x407 * x351))) +
		x302;
	out[0] = x175 + (x176 * x175);
	out[1] = x195 + (x176 * x195);
	out[2] = x211 + (x211 * x176);
	out[3] = x249 + (x249 * x176);
	out[4] = x280 + (x280 * x176);
	out[5] = x313 + (x313 * x176);
	out[6] = x369 + (x369 * x370);
	out[7] = x378 + (x378 * x370);
	out[8] = x389 + (x389 * x370);
	out[9] = x397 + (x397 * x370);
	out[10] = x405 + (x405 * x370);
	out[11] = x413 + (x413 * x370);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x5 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x9 + x3;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = x14 + (x15 * x12);
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x18 = sin(x13);
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x21 * x11;
	const GEN_FLT x23 = x22 + x19;
	const GEN_FLT x24 = x20 * x18;
	const GEN_FLT x25 = x15 * x17;
	const GEN_FLT x26 = x25 * x11;
	const GEN_FLT x27 = x26 + (-1 * x24);
	const GEN_FLT x28 = (x23 * sensor_y) + (x16 * sensor_z) + (x27 * sensor_x) + obj_pz;
	const GEN_FLT x29 = x1 * x4;
	const GEN_FLT x30 = x2 * x8;
	const GEN_FLT x31 = x30 + (-1 * x29);
	const GEN_FLT x32 = x22 + (-1 * x19);
	const GEN_FLT x33 = x20 * x20;
	const GEN_FLT x34 = x14 + (x33 * x15);
	const GEN_FLT x35 = x11 * x18;
	const GEN_FLT x36 = x21 * x17;
	const GEN_FLT x37 = x36 + x35;
	const GEN_FLT x38 = (x37 * sensor_x) + (x34 * sensor_y) + (x32 * sensor_z) + obj_py;
	const GEN_FLT x39 = x5 * x5;
	const GEN_FLT x40 = x6 + (x7 * x39);
	const GEN_FLT x41 = x26 + x24;
	const GEN_FLT x42 = x36 + (-1 * x35);
	const GEN_FLT x43 = x17 * x17;
	const GEN_FLT x44 = x14 + (x43 * x15);
	const GEN_FLT x45 = (x44 * sensor_x) + (x41 * sensor_z) + (x42 * sensor_y) + obj_px;
	const GEN_FLT x46 = (x31 * x38) + (x40 * x45) + (x28 * x10) + lh_px;
	const GEN_FLT x47 = 1. / x46;
	const GEN_FLT x48 = x9 + (-1 * x3);
	const GEN_FLT x49 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x50 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x51 = x50 * x18;
	const GEN_FLT x52 = -1 * x51;
	const GEN_FLT x53 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x54 = x53 * x18;
	const GEN_FLT x55 = x50 * x14;
	const GEN_FLT x56 = x55 * x11;
	const GEN_FLT x57 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x58 = x50 * x19;
	const GEN_FLT x59 = (x49 * x21) + (x58 * x20) + (x57 * x25);
	const GEN_FLT x60 = x57 * x18;
	const GEN_FLT x61 = x55 * x20;
	const GEN_FLT x62 = x15 * x11;
	const GEN_FLT x63 = (x53 * x25) + (x58 * x11) + (x62 * x49);
	const GEN_FLT x64 = ((x63 + x61 + x60) * sensor_z) + ((x59 + (-1 * x56) + (-1 * x54)) * sensor_y) +
						(((x51 * x43) + x52 + (2 * x49 * x25)) * sensor_x);
	const GEN_FLT x65 = x64 + x44;
	const GEN_FLT x66 = x1 * x5;
	const GEN_FLT x67 = x2 * x7;
	const GEN_FLT x68 = x4 * x67;
	const GEN_FLT x69 = x68 + x66;
	const GEN_FLT x70 = x49 * x18;
	const GEN_FLT x71 = x55 * x17;
	const GEN_FLT x72 = (x51 * x20 * x11) + (x53 * x21) + (x62 * x57);
	const GEN_FLT x73 = (((x51 * x33) + (2 * x57 * x21) + x52) * sensor_y) +
						((x72 + (-1 * x71) + (-1 * x70)) * sensor_z) + ((x59 + x56 + x54) * sensor_x);
	const GEN_FLT x74 = x73 + x37;
	const GEN_FLT x75 = x4 * x4;
	const GEN_FLT x76 = x6 + (x7 * x75);
	const GEN_FLT x77 = (((x51 * x12) + (2 * x62 * x53) + x52) * sensor_z) + ((x72 + x71 + x70) * sensor_y) +
						((x63 + (-1 * x61) + (-1 * x60)) * sensor_x);
	const GEN_FLT x78 = x77 + x27;
	const GEN_FLT x79 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x80 = x6 * x79;
	const GEN_FLT x81 = x2 * x80;
	const GEN_FLT x82 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x83 = x1 * x82;
	const GEN_FLT x84 = x1 * x79;
	const GEN_FLT x85 = x5 * x84;
	const GEN_FLT x86 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x87 = x4 * x7;
	const GEN_FLT x88 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x89 = (x8 * x88) + (x86 * x87) + (x4 * x85);
	const GEN_FLT x90 = x5 * x80;
	const GEN_FLT x91 = x1 * x86;
	const GEN_FLT x92 = (x82 * x87) + (x88 * x67) + (x2 * x4 * x84);
	const GEN_FLT x93 = -1 * x84;
	const GEN_FLT x94 = (x28 * ((2 * x88 * x87) + (x84 * x75) + x93)) + (x38 * (x92 + x91 + x90)) +
						(x45 * (x89 + (-1 * x83) + (-1 * x81)));
	const GEN_FLT x95 = x94 + (x78 * x76) + (x74 * x69) + (x65 * x48);
	const GEN_FLT x96 = x4 * x80;
	const GEN_FLT x97 = x1 * x88;
	const GEN_FLT x98 = (x2 * x85) + (x8 * x82) + (x86 * x67);
	const GEN_FLT x99 = (x28 * (x89 + x83 + x81)) + (x38 * (x98 + (-1 * x97) + (-1 * x96))) +
						(x45 * ((2 * x8 * x86) + (x84 * x39) + x93));
	const GEN_FLT x100 = x99 + (x78 * x10) + (x74 * x31) + (x65 * x40);
	const GEN_FLT x101 = x46 * x46;
	const GEN_FLT x102 = (x45 * x48) + (x69 * x38) + (x76 * x28) + lh_pz;
	const GEN_FLT x103 = (1. / x101) * x102;
	const GEN_FLT x104 = x101 + (x102 * x102);
	const GEN_FLT x105 = 1. / x104;
	const GEN_FLT x106 = x101 * x105;
	const GEN_FLT x107 = x106 * ((x100 * x103) + (-1 * x95 * x47));
	const GEN_FLT x108 = 0.523598775598299 + tilt_0;
	const GEN_FLT x109 = cos(x108);
	const GEN_FLT x110 = x68 + (-1 * x66);
	const GEN_FLT x111 = x2 * x2;
	const GEN_FLT x112 = x6 + (x7 * x111);
	const GEN_FLT x113 = x30 + x29;
	const GEN_FLT x114 = (x38 * x112) + (x45 * x113) + (x28 * x110) + lh_py;
	const GEN_FLT x115 = x114 * x114;
	const GEN_FLT x116 = x104 + x115;
	const GEN_FLT x117 = x115 * (1. / x116);
	const GEN_FLT x118 = 1. / sqrt(1 + (-1 * (1. / (x109 * x109)) * x117));
	const GEN_FLT x119 = 1. / x109;
	const GEN_FLT x120 = (x28 * (x92 + (-1 * x91) + (-1 * x90))) + (x38 * ((2 * x82 * x67) + (x84 * x111) + x93)) +
						 (x45 * (x98 + x97 + x96));
	const GEN_FLT x121 = x120 + (x78 * x110) + (x74 * x112) + (x65 * x113);
	const GEN_FLT x122 = 2 * x114;
	const GEN_FLT x123 = 2 * x46;
	const GEN_FLT x124 = 2 * x102;
	const GEN_FLT x125 = (x95 * x124) + (x100 * x123);
	const GEN_FLT x126 = 1.0 / 2.0 * x114;
	const GEN_FLT x127 = (1. / (x116 * sqrt(x116))) * x126;
	const GEN_FLT x128 = x127 * (x125 + (x122 * x121));
	const GEN_FLT x129 = 1. / sqrt(x116);
	const GEN_FLT x130 = x121 * x129;
	const GEN_FLT x131 = ((x119 * x130) + (-1 * x119 * x128)) * x118;
	const GEN_FLT x132 = x114 * x129;
	const GEN_FLT x133 = asin(x119 * x132);
	const GEN_FLT x134 = 8.0108022e-06 * x133;
	const GEN_FLT x135 = -8.0108022e-06 + (-1 * x134);
	const GEN_FLT x136 = 0.0028679863 + (x133 * x135);
	const GEN_FLT x137 = 5.3685255e-06 + (x133 * x136);
	const GEN_FLT x138 = 0.0076069798 + (x133 * x137);
	const GEN_FLT x139 = x133 * x138;
	const GEN_FLT x140 = tan(x108);
	const GEN_FLT x141 = 1. / sqrt(x104);
	const GEN_FLT x142 = x114 * x141;
	const GEN_FLT x143 = x140 * x142;
	const GEN_FLT x144 = atan2(-1 * x102, x46);
	const GEN_FLT x145 = x144 + (-1 * asin(x143)) + ogeeMag_0;
	const GEN_FLT x146 = (sin(x145) * ogeePhase_0) + curve_0;
	const GEN_FLT x147 = -8.0108022e-06 + (-1.60216044e-05 * x133);
	const GEN_FLT x148 = x136 + (x133 * x147);
	const GEN_FLT x149 = x137 + (x133 * x148);
	const GEN_FLT x150 = x138 + (x133 * x149);
	const GEN_FLT x151 = (x133 * x150) + x139;
	const GEN_FLT x152 = sin(x108);
	const GEN_FLT x153 = x146 * x152;
	const GEN_FLT x154 = x109 + (-1 * x151 * x153);
	const GEN_FLT x155 = 1. / x154;
	const GEN_FLT x156 = x146 * x155;
	const GEN_FLT x157 = 2 * x139 * x156;
	const GEN_FLT x158 = x131 * x135;
	const GEN_FLT x159 = 2.40324066e-05 * x133;
	const GEN_FLT x160 = (x133 * (x158 + (-1 * x134 * x131))) + (x131 * x136);
	const GEN_FLT x161 = (x160 * x133) + (x131 * x137);
	const GEN_FLT x162 = x105 * x115;
	const GEN_FLT x163 = 1. / sqrt(1 + (-1 * x162 * (x140 * x140)));
	const GEN_FLT x164 = (1. / (x104 * sqrt(x104))) * x126;
	const GEN_FLT x165 = x125 * x164;
	const GEN_FLT x166 = x121 * x141;
	const GEN_FLT x167 = (x166 * x140) + (-1 * x165 * x140);
	const GEN_FLT x168 = (-1 * x167 * x163) + x107;
	const GEN_FLT x169 = cos(x145) * ogeePhase_0;
	const GEN_FLT x170 = x151 * x152;
	const GEN_FLT x171 = x169 * x170;
	const GEN_FLT x172 = x133 * x133;
	const GEN_FLT x173 = x172 * x138;
	const GEN_FLT x174 = x173 * x146 * (1. / (x154 * x154));
	const GEN_FLT x175 = x172 * x156;
	const GEN_FLT x176 = x173 * x155;
	const GEN_FLT x177 = x169 * x176;
	const GEN_FLT x178 = x143 + (x175 * x138);
	const GEN_FLT x179 = 1. / sqrt(1 + (-1 * (x178 * x178)));
	const GEN_FLT x180 =
		(-1 * x179 *
		 (x167 + (x168 * x177) + (x161 * x175) +
		  (-1 * x174 *
		   ((-1 * x168 * x171) +
			(-1 * x153 *
			 ((x161 * x133) + (x131 * x138) + (x131 * x150) +
			  (x133 * (x161 + (x133 * (x160 + (x133 * ((x131 * x147) + (-1 * x131 * x159) + x158)) + (x131 * x148))) +
					   (x131 * x149))))))) +
		  (x131 * x157))) +
		x107;
	const GEN_FLT x181 = cos(x144 + (-1 * asin(x178)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x182 = x64 + x42;
	const GEN_FLT x183 = x73 + x34;
	const GEN_FLT x184 = x77 + x23;
	const GEN_FLT x185 = x94 + (x76 * x184) + (x69 * x183) + (x48 * x182);
	const GEN_FLT x186 = x99 + (x10 * x184) + (x31 * x183) + (x40 * x182);
	const GEN_FLT x187 = x106 * ((x103 * x186) + (-1 * x47 * x185));
	const GEN_FLT x188 = x120 + (x110 * x184) + (x112 * x183) + (x113 * x182);
	const GEN_FLT x189 = (x124 * x185) + (x123 * x186);
	const GEN_FLT x190 = x127 * (x189 + (x122 * x188));
	const GEN_FLT x191 = x129 * x188;
	const GEN_FLT x192 = ((x119 * x191) + (-1 * x119 * x190)) * x118;
	const GEN_FLT x193 = x192 * x135;
	const GEN_FLT x194 = (x133 * (x193 + (-1 * x192 * x134))) + (x192 * x136);
	const GEN_FLT x195 = (x194 * x133) + (x192 * x137);
	const GEN_FLT x196 = x164 * x189;
	const GEN_FLT x197 = x188 * x141;
	const GEN_FLT x198 = (x197 * x140) + (-1 * x196 * x140);
	const GEN_FLT x199 = (-1 * x163 * x198) + x187;
	const GEN_FLT x200 =
		(-1 * x179 *
		 (x198 + (x177 * x199) + (x175 * x195) +
		  (-1 * x174 *
		   ((-1 * x171 * x199) +
			(-1 * x153 *
			 ((x195 * x133) + (x192 * x138) + (x192 * x150) +
			  (x133 * (x195 + (x133 * (x194 + (x133 * ((x192 * x147) + (-1 * x192 * x159) + x193)) + (x192 * x148))) +
					   (x192 * x149))))))) +
		  (x192 * x157))) +
		x187;
	const GEN_FLT x201 = x64 + x41;
	const GEN_FLT x202 = x73 + x32;
	const GEN_FLT x203 = x77 + x16;
	const GEN_FLT x204 = x94 + (x76 * x203) + (x69 * x202) + (x48 * x201);
	const GEN_FLT x205 = (x10 * x203) + x99 + (x31 * x202) + (x40 * x201);
	const GEN_FLT x206 = x106 * ((x205 * x103) + (-1 * x47 * x204));
	const GEN_FLT x207 = x120 + (x203 * x110) + (x201 * x113) + (x202 * x112);
	const GEN_FLT x208 = (x204 * x124) + (x205 * x123);
	const GEN_FLT x209 = x127 * (x208 + (x207 * x122));
	const GEN_FLT x210 = x207 * x129;
	const GEN_FLT x211 = ((x210 * x119) + (-1 * x209 * x119)) * x118;
	const GEN_FLT x212 = x211 * x135;
	const GEN_FLT x213 = (x133 * (x212 + (-1 * x211 * x134))) + (x211 * x136);
	const GEN_FLT x214 = (x213 * x133) + (x211 * x137);
	const GEN_FLT x215 = x208 * x164;
	const GEN_FLT x216 = x207 * x141;
	const GEN_FLT x217 = (x216 * x140) + (-1 * x215 * x140);
	const GEN_FLT x218 = x169 * ((-1 * x217 * x163) + x206);
	const GEN_FLT x219 =
		(-1 * x179 *
		 (x217 + (x214 * x175) + (x218 * x176) +
		  (-1 * x174 *
		   ((-1 * x218 * x170) +
			(-1 * x153 *
			 ((x214 * x133) + (x211 * x138) + (x211 * x150) +
			  (x133 * (x214 + (x133 * (x213 + (x133 * ((x211 * x147) + (-1 * x211 * x159) + x212)) + (x211 * x148))) +
					   (x211 * x149))))))) +
		  (x211 * x157))) +
		x206;
	const GEN_FLT x220 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x221 = tan(x220);
	const GEN_FLT x222 = 1. / sqrt(1 + (-1 * (x221 * x221) * x162));
	const GEN_FLT x223 = (-1 * x221 * x166) + (x221 * x165);
	const GEN_FLT x224 = (-1 * x223 * x222) + x107;
	const GEN_FLT x225 = -1 * x221 * x142;
	const GEN_FLT x226 = x144 + (-1 * asin(x225)) + ogeeMag_1;
	const GEN_FLT x227 = cos(x226) * ogeePhase_1;
	const GEN_FLT x228 = cos(x220);
	const GEN_FLT x229 = 1. / x228;
	const GEN_FLT x230 = asin(x229 * x132);
	const GEN_FLT x231 = 8.0108022e-06 * x230;
	const GEN_FLT x232 = -8.0108022e-06 + (-1 * x231);
	const GEN_FLT x233 = 0.0028679863 + (x230 * x232);
	const GEN_FLT x234 = 5.3685255e-06 + (x230 * x233);
	const GEN_FLT x235 = 0.0076069798 + (x230 * x234);
	const GEN_FLT x236 = x230 * x235;
	const GEN_FLT x237 = -8.0108022e-06 + (-1.60216044e-05 * x230);
	const GEN_FLT x238 = x233 + (x230 * x237);
	const GEN_FLT x239 = x234 + (x230 * x238);
	const GEN_FLT x240 = x235 + (x230 * x239);
	const GEN_FLT x241 = (x230 * x240) + x236;
	const GEN_FLT x242 = (sin(x226) * ogeePhase_1) + curve_1;
	const GEN_FLT x243 = sin(x220);
	const GEN_FLT x244 = x242 * x243;
	const GEN_FLT x245 = x228 + (x241 * x244);
	const GEN_FLT x246 = 1. / x245;
	const GEN_FLT x247 = x230 * x230;
	const GEN_FLT x248 = x235 * x247;
	const GEN_FLT x249 = x246 * x248;
	const GEN_FLT x250 = x227 * x249;
	const GEN_FLT x251 = 1. / sqrt(1 + (-1 * (1. / (x228 * x228)) * x117));
	const GEN_FLT x252 = (x229 * x130) + (-1 * x229 * x128);
	const GEN_FLT x253 = x252 * x251;
	const GEN_FLT x254 = x232 * x251;
	const GEN_FLT x255 = x252 * x254;
	const GEN_FLT x256 = (x233 * x253) + (x230 * (x255 + (-1 * x231 * x253)));
	const GEN_FLT x257 = (x230 * x256) + (x234 * x253);
	const GEN_FLT x258 = x242 * x246;
	const GEN_FLT x259 = x258 * x247;
	const GEN_FLT x260 = 2 * x236 * x258;
	const GEN_FLT x261 = x241 * x243;
	const GEN_FLT x262 = x261 * x227;
	const GEN_FLT x263 = x239 * x251;
	const GEN_FLT x264 = 2.40324066e-05 * x230;
	const GEN_FLT x265 = x242 * (1. / (x245 * x245)) * x248;
	const GEN_FLT x266 = x225 + (x235 * x259);
	const GEN_FLT x267 = 1. / sqrt(1 + (-1 * (x266 * x266)));
	const GEN_FLT x268 =
		(-1 * x267 *
		 (x223 +
		  (-1 * x265 *
		   ((x244 *
			 ((x235 * x253) + (x230 * x257) +
			  (x230 * (x257 + (x230 * (x256 + (x238 * x253) + (x230 * ((x237 * x253) + (-1 * x264 * x253) + x255)))) +
					   (x263 * x252))) +
			  (x253 * x240))) +
			(x262 * x224))) +
		  (x260 * x253) + (x257 * x259) + (x250 * x224))) +
		x107;
	const GEN_FLT x269 = cos(x144 + (-1 * asin(x266)) + gibPhase_1) * gibMag_1;
	const GEN_FLT x270 = (-1 * x221 * x197) + (x221 * x196);
	const GEN_FLT x271 = (-1 * x270 * x222) + x187;
	const GEN_FLT x272 = (x229 * x191) + (-1 * x229 * x190);
	const GEN_FLT x273 = x272 * x251;
	const GEN_FLT x274 = x272 * x254;
	const GEN_FLT x275 = (x233 * x273) + (x230 * (x274 + (-1 * x231 * x273)));
	const GEN_FLT x276 = (x230 * x275) + (x234 * x273);
	const GEN_FLT x277 =
		(-1 * x267 *
		 (x270 + (x273 * x260) + (x276 * x259) +
		  (-1 * x265 *
		   ((x244 *
			 ((x235 * x273) + (x230 * x276) +
			  (x230 * (x276 + (x230 * (x275 + (x238 * x273) + (x230 * ((x237 * x273) + x274 + (-1 * x273 * x264))))) +
					   (x272 * x263))) +
			  (x273 * x240))) +
			(x271 * x262))) +
		  (x271 * x250))) +
		x187;
	const GEN_FLT x278 = (-1 * x216 * x221) + (x215 * x221);
	const GEN_FLT x279 = x227 * ((-1 * x278 * x222) + x206);
	const GEN_FLT x280 = (x210 * x229) + (-1 * x209 * x229);
	const GEN_FLT x281 = x280 * x251;
	const GEN_FLT x282 = x280 * x254;
	const GEN_FLT x283 = (x233 * x281) + (x230 * (x282 + (-1 * x231 * x281)));
	const GEN_FLT x284 = (x230 * x283) + (x234 * x281);
	const GEN_FLT x285 =
		(-1 * x267 *
		 (x278 +
		  (-1 * x265 *
		   ((x244 *
			 ((x235 * x281) + (x230 * x284) +
			  (x230 * (x284 + (x230 * (x283 + (x238 * x281) + (x230 * ((x237 * x281) + (-1 * x264 * x281) + x282)))) +
					   (x263 * x280))) +
			  (x281 * x240))) +
			(x279 * x261))) +
		  (x260 * x281) + (x284 * x259) + (x279 * x249))) +
		x206;
	out[0] = x180 + (x181 * x180);
	out[1] = x200 + (x200 * x181);
	out[2] = x219 + (x219 * x181);
	out[3] = x268 + (x268 * x269);
	out[4] = x277 + (x277 * x269);
	out[5] = x285 + (x269 * x285);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x9 + x3;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x20 * x11;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x15 * x11;
	const GEN_FLT x24 = x23 * x16;
	const GEN_FLT x25 =
		((x24 + (-1 * x22)) * sensor_x) + ((x21 + x18) * sensor_y) + ((x14 + (x15 * x12)) * sensor_z) + obj_pz;
	const GEN_FLT x26 = x1 * x4;
	const GEN_FLT x27 = x2 * x8;
	const GEN_FLT x28 = x27 + (-1 * x26);
	const GEN_FLT x29 = x19 * x19;
	const GEN_FLT x30 = x11 * x17;
	const GEN_FLT x31 = x20 * x16;
	const GEN_FLT x32 =
		((x31 + x30) * sensor_x) + ((x14 + (x29 * x15)) * sensor_y) + ((x21 + (-1 * x18)) * sensor_z) + obj_py;
	const GEN_FLT x33 = x7 * x7;
	const GEN_FLT x34 = x5 + (x6 * x33);
	const GEN_FLT x35 = x16 * x16;
	const GEN_FLT x36 =
		((x14 + (x35 * x15)) * sensor_x) + ((x31 + (-1 * x30)) * sensor_y) + ((x24 + x22) * sensor_z) + obj_px;
	const GEN_FLT x37 = (x34 * x36) + (x32 * x28) + (x25 * x10) + lh_px;
	const GEN_FLT x38 = 1. / x37;
	const GEN_FLT x39 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x40 = x2 * x5;
	const GEN_FLT x41 = x40 * x39;
	const GEN_FLT x42 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x43 = x1 * x42;
	const GEN_FLT x44 = x1 * x39;
	const GEN_FLT x45 = x4 * x7;
	const GEN_FLT x46 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x47 = x4 * x6;
	const GEN_FLT x48 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x49 = (x8 * x48) + (x46 * x47) + (x44 * x45);
	const GEN_FLT x50 = x5 * x7;
	const GEN_FLT x51 = x50 * x39;
	const GEN_FLT x52 = x1 * x46;
	const GEN_FLT x53 = x3 * x39;
	const GEN_FLT x54 = x2 * x6;
	const GEN_FLT x55 = (x42 * x47) + (x54 * x48) + (x4 * x53);
	const GEN_FLT x56 = -1 * x44;
	const GEN_FLT x57 = x4 * x4;
	const GEN_FLT x58 = 2 * x47;
	const GEN_FLT x59 = x9 + (-1 * x3);
	const GEN_FLT x60 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x61 = x15 * x16;
	const GEN_FLT x62 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x63 = x62 * x17;
	const GEN_FLT x64 = -1 * x63;
	const GEN_FLT x65 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x66 = x65 * x17;
	const GEN_FLT x67 = x62 * x14;
	const GEN_FLT x68 = x67 * x11;
	const GEN_FLT x69 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x70 = x62 * x18;
	const GEN_FLT x71 = (x60 * x20) + (x70 * x19) + (x61 * x69);
	const GEN_FLT x72 = x69 * x17;
	const GEN_FLT x73 = x67 * x19;
	const GEN_FLT x74 = (x61 * x65) + (x70 * x11) + (x60 * x23);
	const GEN_FLT x75 = ((x74 + x73 + x72) * sensor_z) + ((x71 + (-1 * x68) + (-1 * x66)) * sensor_y) +
						(((x63 * x35) + x64 + (2 * x60 * x61)) * sensor_x);
	const GEN_FLT x76 = x60 * x17;
	const GEN_FLT x77 = x67 * x16;
	const GEN_FLT x78 = (x63 * x11 * x19) + (x65 * x20) + (x69 * x23);
	const GEN_FLT x79 = ((x78 + (-1 * x77) + (-1 * x76)) * sensor_z) +
						(((x63 * x29) + (2 * x69 * x20) + x64) * sensor_y) + ((x71 + x68 + x66) * sensor_x);
	const GEN_FLT x80 = x1 * x7;
	const GEN_FLT x81 = x2 * x47;
	const GEN_FLT x82 = x81 + x80;
	const GEN_FLT x83 = x5 + (x6 * x57);
	const GEN_FLT x84 = (((x63 * x12) + (2 * x65 * x23) + x64) * sensor_z) + ((x78 + x77 + x76) * sensor_y) +
						((x74 + (-1 * x73) + (-1 * x72)) * sensor_x);
	const GEN_FLT x85 = (x83 * x84) + (x82 * x79) + (x75 * x59);
	const GEN_FLT x86 = x85 + (x25 * ((x57 * x44) + (x58 * x48) + x56)) + (x32 * (x55 + x52 + x51)) +
						(x36 * (x49 + (-1 * x43) + (-1 * x41)));
	const GEN_FLT x87 = -1 * x86 * x38;
	const GEN_FLT x88 = 2 * x8;
	const GEN_FLT x89 = x4 * x5;
	const GEN_FLT x90 = x89 * x39;
	const GEN_FLT x91 = x1 * x48;
	const GEN_FLT x92 = (x8 * x42) + (x7 * x53) + (x54 * x46);
	const GEN_FLT x93 = (x84 * x10) + (x79 * x28) + (x75 * x34);
	const GEN_FLT x94 = x93 + (x25 * (x49 + x43 + x41)) + (x32 * (x92 + (-1 * x91) + (-1 * x90))) +
						(x36 * ((x88 * x46) + (x44 * x33) + x56));
	const GEN_FLT x95 = 1 + x94;
	const GEN_FLT x96 = x37 * x37;
	const GEN_FLT x97 = (x59 * x36) + (x83 * x25) + (x82 * x32) + lh_pz;
	const GEN_FLT x98 = x97 * (1. / x96);
	const GEN_FLT x99 = x96 + (x97 * x97);
	const GEN_FLT x100 = 1. / x99;
	const GEN_FLT x101 = x96 * x100;
	const GEN_FLT x102 = x101 * ((x98 * x95) + x87);
	const GEN_FLT x103 = x2 * x2;
	const GEN_FLT x104 = 2 * x54;
	const GEN_FLT x105 = x5 + (x6 * x103);
	const GEN_FLT x106 = x27 + x26;
	const GEN_FLT x107 = x81 + (-1 * x80);
	const GEN_FLT x108 = (x84 * x107) + (x75 * x106) + (x79 * x105);
	const GEN_FLT x109 = x108 + (x25 * (x55 + (-1 * x52) + (-1 * x51))) + (x32 * ((x42 * x104) + (x44 * x103) + x56)) +
						 (x36 * (x92 + x91 + x90));
	const GEN_FLT x110 = (x36 * x106) + (x32 * x105) + (x25 * x107) + lh_py;
	const GEN_FLT x111 = 2 * x110;
	const GEN_FLT x112 = x109 * x111;
	const GEN_FLT x113 = 2 * x37;
	const GEN_FLT x114 = 2 * x97;
	const GEN_FLT x115 = x86 * x114;
	const GEN_FLT x116 = x115 + (x95 * x113);
	const GEN_FLT x117 = x116 + x112;
	const GEN_FLT x118 = 0.523598775598299 + tilt_0;
	const GEN_FLT x119 = cos(x118);
	const GEN_FLT x120 = 1. / x119;
	const GEN_FLT x121 = x110 * x110;
	const GEN_FLT x122 = x99 + x121;
	const GEN_FLT x123 = 1.0 / 2.0 * x110;
	const GEN_FLT x124 = x123 * (1. / (x122 * sqrt(x122)));
	const GEN_FLT x125 = x120 * x124;
	const GEN_FLT x126 = 1. / sqrt(x122);
	const GEN_FLT x127 = x109 * x126;
	const GEN_FLT x128 = x120 * x127;
	const GEN_FLT x129 = x128 + (-1 * x117 * x125);
	const GEN_FLT x130 = (1. / x122) * x121;
	const GEN_FLT x131 = 1. / sqrt(1 + (-1 * (1. / (x119 * x119)) * x130));
	const GEN_FLT x132 = tan(x118);
	const GEN_FLT x133 = 1. / sqrt(x99);
	const GEN_FLT x134 = x110 * x133;
	const GEN_FLT x135 = x134 * x132;
	const GEN_FLT x136 = atan2(-1 * x97, x37);
	const GEN_FLT x137 = x136 + (-1 * asin(x135)) + ogeeMag_0;
	const GEN_FLT x138 = (sin(x137) * ogeePhase_0) + curve_0;
	const GEN_FLT x139 = x110 * x126;
	const GEN_FLT x140 = asin(x120 * x139);
	const GEN_FLT x141 = 8.0108022e-06 * x140;
	const GEN_FLT x142 = -8.0108022e-06 + (-1 * x141);
	const GEN_FLT x143 = 0.0028679863 + (x140 * x142);
	const GEN_FLT x144 = 5.3685255e-06 + (x140 * x143);
	const GEN_FLT x145 = 0.0076069798 + (x140 * x144);
	const GEN_FLT x146 = x140 * x145;
	const GEN_FLT x147 = -8.0108022e-06 + (-1.60216044e-05 * x140);
	const GEN_FLT x148 = x143 + (x140 * x147);
	const GEN_FLT x149 = x144 + (x140 * x148);
	const GEN_FLT x150 = x145 + (x140 * x149);
	const GEN_FLT x151 = (x140 * x150) + x146;
	const GEN_FLT x152 = sin(x118);
	const GEN_FLT x153 = x138 * x152;
	const GEN_FLT x154 = x119 + (-1 * x151 * x153);
	const GEN_FLT x155 = 1. / x154;
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
	const GEN_FLT x168 = (x140 * (x162 + (-1 * x129 * x167))) + (x129 * x166);
	const GEN_FLT x169 = x131 * x144;
	const GEN_FLT x170 = (x168 * x140) + (x129 * x169);
	const GEN_FLT x171 = x131 * x145;
	const GEN_FLT x172 = x100 * x121;
	const GEN_FLT x173 = 1. / sqrt(1 + (-1 * x172 * (x132 * x132)));
	const GEN_FLT x174 = (1. / (x99 * sqrt(x99))) * x123;
	const GEN_FLT x175 = x116 * x174;
	const GEN_FLT x176 = x133 * x132;
	const GEN_FLT x177 = x109 * x176;
	const GEN_FLT x178 = x177 + (-1 * x175 * x132);
	const GEN_FLT x179 = (-1 * x178 * x173) + x102;
	const GEN_FLT x180 = cos(x137) * ogeePhase_0;
	const GEN_FLT x181 = x151 * x152;
	const GEN_FLT x182 = x181 * x180;
	const GEN_FLT x183 = x140 * x140;
	const GEN_FLT x184 = x183 * x138;
	const GEN_FLT x185 = x184 * x145 * (1. / (x154 * x154));
	const GEN_FLT x186 = x184 * x155;
	const GEN_FLT x187 = x183 * x145 * x155;
	const GEN_FLT x188 = x180 * x187;
	const GEN_FLT x189 = x135 + (x186 * x145);
	const GEN_FLT x190 = 1. / sqrt(1 + (-1 * (x189 * x189)));
	const GEN_FLT x191 =
		(-1 * x190 *
		 (x178 + (x179 * x188) + (x170 * x186) +
		  (-1 * x185 *
		   ((-1 * x179 * x182) +
			(-1 * x153 *
			 ((x170 * x140) +
			  (x140 * (x170 + (x140 * (x168 + (x140 * ((x129 * x165) + (-1 * x129 * x164) + x162)) + (x129 * x160))) +
					   (x129 * x159))) +
			  (x129 * x171) + (x129 * x158))))) +
		  (x129 * x157))) +
		x102;
	const GEN_FLT x192 = cos(x136 + (-1 * asin(x189)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x193 = x98 * x94;
	const GEN_FLT x194 = x101 * (x193 + x87);
	const GEN_FLT x195 = 1 + x109;
	const GEN_FLT x196 = x94 * x113;
	const GEN_FLT x197 = x115 + x196;
	const GEN_FLT x198 = x197 + (x111 * x195);
	const GEN_FLT x199 = x126 * x195;
	const GEN_FLT x200 = (x120 * x199) + (-1 * x125 * x198);
	const GEN_FLT x201 = x200 * x131;
	const GEN_FLT x202 = x200 * x161;
	const GEN_FLT x203 = (x140 * (x202 + (-1 * x201 * x141))) + (x201 * x143);
	const GEN_FLT x204 = (x203 * x140) + (x201 * x144);
	const GEN_FLT x205 = x174 * x197;
	const GEN_FLT x206 = (x176 * x195) + (-1 * x205 * x132);
	const GEN_FLT x207 = (-1 * x206 * x173) + x194;
	const GEN_FLT x208 =
		(-1 * x190 *
		 (x206 + (x207 * x188) + (x204 * x186) +
		  (-1 * x185 *
		   ((-1 * x207 * x182) +
			(-1 * x153 *
			 ((x204 * x140) + (x200 * x171) +
			  (x140 * (x204 + (x140 * (x203 + (x140 * ((x201 * x147) + (-1 * x201 * x163) + x202)) + (x201 * x148))) +
					   (x201 * x149))) +
			  (x201 * x150))))) +
		  (x201 * x156))) +
		x194;
	const GEN_FLT x209 = 1 + x86;
	const GEN_FLT x210 = x101 * (x193 + (-1 * x38 * x209));
	const GEN_FLT x211 = (x209 * x114) + x196;
	const GEN_FLT x212 = x211 + x112;
	const GEN_FLT x213 = x128 + (-1 * x212 * x125);
	const GEN_FLT x214 = x213 * x131;
	const GEN_FLT x215 = x213 * x161;
	const GEN_FLT x216 = (x140 * (x215 + (-1 * x213 * x167))) + (x213 * x166);
	const GEN_FLT x217 = (x216 * x140) + (x213 * x169);
	const GEN_FLT x218 = x174 * x132;
	const GEN_FLT x219 = x177 + (-1 * x211 * x218);
	const GEN_FLT x220 = (-1 * x219 * x173) + x210;
	const GEN_FLT x221 =
		(-1 * x190 *
		 (x219 + (x217 * x186) + (x220 * x188) +
		  (-1 * x185 *
		   ((-1 * x220 * x182) +
			(-1 * x153 *
			 ((x217 * x140) + (x213 * x171) +
			  (x140 * (x217 + (x140 * (x216 + (x140 * ((x213 * x165) + (-1 * x214 * x163) + x215)) + (x213 * x160))) +
					   (x213 * x159))) +
			  (x213 * x158))))) +
		  (x214 * x156))) +
		x210;
	const GEN_FLT x222 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x223 = x40 * x222;
	const GEN_FLT x224 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qj *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x225 = x1 * x224;
	const GEN_FLT x226 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qi *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x227 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qk *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x228 = x1 * x222;
	const GEN_FLT x229 = x7 * x228;
	const GEN_FLT x230 = (x4 * x229) + (x8 * x227) + (x47 * x226);
	const GEN_FLT x231 = x50 * x222;
	const GEN_FLT x232 = x1 * x226;
	const GEN_FLT x233 = x2 * x4;
	const GEN_FLT x234 = (x47 * x224) + (x54 * x227) + (x233 * x228);
	const GEN_FLT x235 = -1 * x228;
	const GEN_FLT x236 = x85 + (x25 * ((x58 * x227) + (x57 * x228) + x235)) + (x32 * (x234 + x232 + x231)) +
						 (x36 * (x230 + (-1 * x225) + (-1 * x223)));
	const GEN_FLT x237 = x89 * x222;
	const GEN_FLT x238 = x1 * x227;
	const GEN_FLT x239 = (x8 * x224) + (x54 * x226) + (x2 * x229);
	const GEN_FLT x240 = x93 + (x25 * (x230 + x225 + x223)) + (x32 * (x239 + (-1 * x238) + (-1 * x237))) +
						 (x36 * ((x88 * x226) + (x33 * x228) + x235));
	const GEN_FLT x241 = ((x98 * x240) + (-1 * x38 * x236)) * x101;
	const GEN_FLT x242 = x108 + (x36 * (x239 + x238 + x237)) + (x25 * (x234 + (-1 * x232) + (-1 * x231))) +
						 (x32 * ((x224 * x104) + (x228 * x103) + x235));
	const GEN_FLT x243 = (x236 * x114) + (x240 * x113);
	const GEN_FLT x244 = x243 + (x242 * x111);
	const GEN_FLT x245 = x242 * x126;
	const GEN_FLT x246 = (x245 * x120) + (-1 * x244 * x125);
	const GEN_FLT x247 = x246 * x161;
	const GEN_FLT x248 = (x140 * (x247 + (-1 * x246 * x167))) + (x246 * x166);
	const GEN_FLT x249 = (x248 * x140) + (x246 * x169);
	const GEN_FLT x250 = x243 * x174;
	const GEN_FLT x251 = x242 * x133;
	const GEN_FLT x252 = (x251 * x132) + (-1 * x250 * x132);
	const GEN_FLT x253 = (-1 * x252 * x173) + x241;
	const GEN_FLT x254 =
		(-1 * x190 *
		 (x252 + (x253 * x188) +
		  (-1 * x185 *
		   ((-1 * x253 * x182) +
			(-1 * x153 *
			 ((x249 * x140) + (x246 * x171) +
			  (x140 * (x249 + (x140 * (x248 + (x140 * ((x246 * x165) + (-1 * x246 * x164) + x247)) + (x246 * x160))) +
					   (x246 * x159))) +
			  (x246 * x158))))) +
		  (x249 * x186) + (x246 * x157))) +
		x241;
	const GEN_FLT x255 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x256 = x40 * x255;
	const GEN_FLT x257 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qj *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x258 = x1 * x257;
	const GEN_FLT x259 = x1 * x255;
	const GEN_FLT x260 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qi *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x261 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qk *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x262 = (x8 * x261) + (x47 * x260) + (x45 * x259);
	const GEN_FLT x263 = x50 * x255;
	const GEN_FLT x264 = x1 * x260;
	const GEN_FLT x265 = (x47 * x257) + (x54 * x261) + (x233 * x259);
	const GEN_FLT x266 = -1 * x259;
	const GEN_FLT x267 = x85 + (x25 * ((x58 * x261) + (x57 * x259) + x266)) + (x32 * (x265 + x264 + x263)) +
						 (x36 * (x262 + (-1 * x258) + (-1 * x256)));
	const GEN_FLT x268 = x89 * x255;
	const GEN_FLT x269 = x1 * x261;
	const GEN_FLT x270 = x2 * x7;
	const GEN_FLT x271 = (x54 * x260) + (x8 * x257) + (x270 * x259);
	const GEN_FLT x272 = x93 + (x25 * (x262 + x258 + x256)) + (x32 * (x271 + (-1 * x269) + (-1 * x268))) +
						 (x36 * ((x88 * x260) + x266 + (x33 * x259)));
	const GEN_FLT x273 = ((x98 * x272) + (-1 * x38 * x267)) * x101;
	const GEN_FLT x274 = x108 + (x25 * (x265 + (-1 * x264) + (-1 * x263))) +
						 (x32 * ((x257 * x104) + (x259 * x103) + x266)) + (x36 * (x271 + x269 + x268));
	const GEN_FLT x275 = (x267 * x114) + (x272 * x113);
	const GEN_FLT x276 = x124 * (x275 + (x274 * x111));
	const GEN_FLT x277 = x274 * x126;
	const GEN_FLT x278 = (x277 * x120) + (-1 * x276 * x120);
	const GEN_FLT x279 = x278 * x131;
	const GEN_FLT x280 = x278 * x161;
	const GEN_FLT x281 = (x140 * (x280 + (-1 * x279 * x141))) + (x279 * x143);
	const GEN_FLT x282 = (x281 * x140) + (x279 * x144);
	const GEN_FLT x283 = x274 * x133;
	const GEN_FLT x284 = (x283 * x132) + (-1 * x218 * x275);
	const GEN_FLT x285 = (-1 * x284 * x173) + x273;
	const GEN_FLT x286 =
		(-1 * x190 *
		 ((x282 * x186) + x284 + (x285 * x188) +
		  (-1 * x185 *
		   ((-1 * x285 * x182) +
			(-1 * x153 *
			 ((x278 * x171) +
			  (x140 * (x282 + (x140 * (x281 + (x140 * ((x279 * x147) + (-1 * x279 * x163) + x280)) + (x279 * x148))) +
					   (x279 * x149))) +
			  (x282 * x140) + (x279 * x150))))) +
		  (x279 * x156))) +
		x273;
	const GEN_FLT x287 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x288 = x40 * x287;
	const GEN_FLT x289 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qj *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x290 = x1 * x289;
	const GEN_FLT x291 = x1 * x287;
	const GEN_FLT x292 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qi *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x293 = x6 * x292;
	const GEN_FLT x294 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qk *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x295 = (x8 * x294) + (x4 * x293) + (x45 * x291);
	const GEN_FLT x296 = x50 * x287;
	const GEN_FLT x297 = x1 * x292;
	const GEN_FLT x298 = (x47 * x289) + (x54 * x294) + (x233 * x291);
	const GEN_FLT x299 = -1 * x291;
	const GEN_FLT x300 = x85 + (x25 * ((x58 * x294) + (x57 * x291) + x299)) + (x32 * (x298 + x297 + x296)) +
						 (x36 * (x295 + (-1 * x290) + (-1 * x288)));
	const GEN_FLT x301 = x89 * x287;
	const GEN_FLT x302 = x1 * x294;
	const GEN_FLT x303 = (x8 * x289) + (x2 * x293) + (x270 * x291);
	const GEN_FLT x304 = x93 + (x25 * (x295 + x290 + x288)) + (x32 * (x303 + (-1 * x302) + (-1 * x301))) +
						 (x36 * ((2 * x7 * x293) + (x33 * x291) + x299));
	const GEN_FLT x305 = ((x98 * x304) + (-1 * x38 * x300)) * x101;
	const GEN_FLT x306 = x108 + (x25 * (x298 + (-1 * x297) + (-1 * x296))) +
						 (x32 * ((x289 * x104) + (x291 * x103) + x299)) + (x36 * (x303 + x302 + x301));
	const GEN_FLT x307 = (x300 * x114) + (x304 * x113);
	const GEN_FLT x308 = x124 * (x307 + (x306 * x111));
	const GEN_FLT x309 = x306 * x126;
	const GEN_FLT x310 = (x309 * x120) + (-1 * x308 * x120);
	const GEN_FLT x311 = x310 * x161;
	const GEN_FLT x312 = (x140 * (x311 + (-1 * x310 * x167))) + (x310 * x166);
	const GEN_FLT x313 = (x312 * x140) + (x310 * x169);
	const GEN_FLT x314 = x306 * x133;
	const GEN_FLT x315 = (x314 * x132) + (-1 * x218 * x307);
	const GEN_FLT x316 = x180 * ((-1 * x315 * x173) + x305);
	const GEN_FLT x317 =
		(-1 * x190 *
		 ((x313 * x186) + (x316 * x187) +
		  (-1 * x185 *
		   ((-1 * x316 * x181) +
			(-1 * x153 *
			 ((x313 * x140) + (x310 * x171) +
			  (x140 * (x313 + (x140 * (x312 + (x140 * ((x310 * x165) + (-1 * x310 * x164) + x311)) + (x310 * x160))) +
					   (x310 * x159))) +
			  (x310 * x158))))) +
		  x315 + (x310 * x157))) +
		x305;
	const GEN_FLT x318 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x319 = tan(x318);
	const GEN_FLT x320 = 1. / sqrt(1 + (-1 * (x319 * x319) * x172));
	const GEN_FLT x321 = x319 * x133;
	const GEN_FLT x322 = -1 * x321 * x109;
	const GEN_FLT x323 = x322 + (x319 * x175);
	const GEN_FLT x324 = (-1 * x323 * x320) + x102;
	const GEN_FLT x325 = -1 * x319 * x134;
	const GEN_FLT x326 = x136 + (-1 * asin(x325)) + ogeeMag_1;
	const GEN_FLT x327 = cos(x326) * ogeePhase_1;
	const GEN_FLT x328 = cos(x318);
	const GEN_FLT x329 = 1. / x328;
	const GEN_FLT x330 = asin(x329 * x139);
	const GEN_FLT x331 = 8.0108022e-06 * x330;
	const GEN_FLT x332 = -8.0108022e-06 + (-1 * x331);
	const GEN_FLT x333 = 0.0028679863 + (x332 * x330);
	const GEN_FLT x334 = 5.3685255e-06 + (x333 * x330);
	const GEN_FLT x335 = 0.0076069798 + (x334 * x330);
	const GEN_FLT x336 = x330 * x330;
	const GEN_FLT x337 = x335 * x330;
	const GEN_FLT x338 = -8.0108022e-06 + (-1.60216044e-05 * x330);
	const GEN_FLT x339 = x333 + (x338 * x330);
	const GEN_FLT x340 = x334 + (x339 * x330);
	const GEN_FLT x341 = x335 + (x330 * x340);
	const GEN_FLT x342 = (x330 * x341) + x337;
	const GEN_FLT x343 = (sin(x326) * ogeePhase_1) + curve_1;
	const GEN_FLT x344 = sin(x318);
	const GEN_FLT x345 = x344 * x343;
	const GEN_FLT x346 = x328 + (x345 * x342);
	const GEN_FLT x347 = 1. / x346;
	const GEN_FLT x348 = x336 * x347 * x335;
	const GEN_FLT x349 = x327 * x348;
	const GEN_FLT x350 = 1. / sqrt(1 + (-1 * (1. / (x328 * x328)) * x130));
	const GEN_FLT x351 = x329 * x124;
	const GEN_FLT x352 = x329 * x127;
	const GEN_FLT x353 = x352 + (-1 * x351 * x117);
	const GEN_FLT x354 = x350 * x353;
	const GEN_FLT x355 = x354 * x332;
	const GEN_FLT x356 = x350 * x333;
	const GEN_FLT x357 = (x353 * x356) + (x330 * (x355 + (-1 * x354 * x331)));
	const GEN_FLT x358 = (x357 * x330) + (x354 * x334);
	const GEN_FLT x359 = x336 * x343;
	const GEN_FLT x360 = x359 * x347;
	const GEN_FLT x361 = 2 * x337 * x347 * x343;
	const GEN_FLT x362 = x344 * x342;
	const GEN_FLT x363 = x362 * x327;
	const GEN_FLT x364 = 2.40324066e-05 * x330;
	const GEN_FLT x365 = x350 * x339;
	const GEN_FLT x366 = x359 * (1. / (x346 * x346)) * x335;
	const GEN_FLT x367 = x325 + (x360 * x335);
	const GEN_FLT x368 = 1. / sqrt(1 + (-1 * (x367 * x367)));
	const GEN_FLT x369 =
		(-1 * x368 *
		 (x323 +
		  (-1 * x366 *
		   ((x345 *
			 ((x354 * x335) + (x354 * x341) + (x358 * x330) +
			  (x330 * (x358 + (x330 * (x357 + (x365 * x353) + (x330 * ((x354 * x338) + (-1 * x364 * x354) + x355)))) +
					   (x354 * x340))))) +
			(x363 * x324))) +
		  (x361 * x354) + (x360 * x358) + (x324 * x349))) +
		x102;
	const GEN_FLT x370 = cos((-1 * asin(x367)) + x136 + gibPhase_1) * gibMag_1;
	const GEN_FLT x371 = (-1 * x321 * x195) + (x205 * x319);
	const GEN_FLT x372 = (-1 * x371 * x320) + x194;
	const GEN_FLT x373 = (x329 * x199) + (-1 * x351 * x198);
	const GEN_FLT x374 = x373 * x350;
	const GEN_FLT x375 = x374 * x332;
	const GEN_FLT x376 = (x373 * x356) + (x330 * (x375 + (-1 * x374 * x331)));
	const GEN_FLT x377 = (x376 * x330) + (x374 * x334);
	const GEN_FLT x378 =
		(-1 * x368 *
		 (x371 +
		  (-1 * x366 *
		   ((x345 *
			 ((x374 * x335) + (x377 * x330) +
			  (x330 * (x377 + (x330 * (x376 + (x373 * x365) + (x330 * ((x374 * x338) + (-1 * x374 * x364) + x375)))) +
					   (x374 * x340))) +
			  (x374 * x341))) +
			(x372 * x363))) +
		  (x374 * x361) + (x377 * x360) + (x372 * x349))) +
		x194;
	const GEN_FLT x379 = x319 * x174;
	const GEN_FLT x380 = x322 + (x211 * x379);
	const GEN_FLT x381 = (-1 * x380 * x320) + x210;
	const GEN_FLT x382 = x352 + (-1 * x212 * x351);
	const GEN_FLT x383 = x350 * x382;
	const GEN_FLT x384 = x383 * x332;
	const GEN_FLT x385 = (x382 * x356) + (x330 * (x384 + (-1 * x383 * x331)));
	const GEN_FLT x386 = (x385 * x330) + (x383 * x334);
	const GEN_FLT x387 =
		(-1 * x368 *
		 (x380 + (x361 * x383) +
		  (-1 * x366 *
		   ((x345 *
			 ((x386 * x330) + (x383 * x335) +
			  (x330 * (x386 + (x330 * (x385 + (x365 * x382) + (x330 * ((x383 * x338) + (-1 * x364 * x383) + x384)))) +
					   (x383 * x340))) +
			  (x383 * x341))) +
			(x363 * x381))) +
		  (x360 * x386) + (x381 * x349))) +
		x210;
	const GEN_FLT x388 = (-1 * x251 * x319) + (x250 * x319);
	const GEN_FLT x389 = x327 * ((-1 * x388 * x320) + x241);
	const GEN_FLT x390 = (x245 * x329) + (-1 * x244 * x351);
	const GEN_FLT x391 = x390 * x350;
	const GEN_FLT x392 = x391 * x332;
	const GEN_FLT x393 = (x390 * x356) + (x330 * (x392 + (-1 * x391 * x331)));
	const GEN_FLT x394 = (x393 * x330) + (x391 * x334);
	const GEN_FLT x395 =
		(-1 * x368 *
		 (x388 +
		  (-1 * x366 *
		   ((x345 *
			 ((x391 * x335) + (x394 * x330) +
			  (x330 * (x394 + (x330 * (x393 + (x391 * x339) + (x330 * ((x391 * x338) + (-1 * x391 * x364) + x392)))) +
					   (x391 * x340))) +
			  (x391 * x341))) +
			(x362 * x389))) +
		  (x394 * x360) + (x391 * x361) + (x389 * x348))) +
		x241;
	const GEN_FLT x396 = (-1 * x283 * x319) + (x275 * x379);
	const GEN_FLT x397 = (-1 * x396 * x320) + x273;
	const GEN_FLT x398 = (x277 * x329) + (-1 * x276 * x329);
	const GEN_FLT x399 = x398 * x350;
	const GEN_FLT x400 = x399 * x332;
	const GEN_FLT x401 = (x398 * x356) + (x330 * (x400 + (-1 * x399 * x331)));
	const GEN_FLT x402 = (x401 * x330) + (x399 * x334);
	const GEN_FLT x403 =
		(-1 * x368 *
		 (x396 + (x399 * x361) +
		  (-1 * x366 *
		   ((x345 *
			 ((x399 * x341) + (x402 * x330) + (x399 * x335) +
			  (x330 * (x402 + (x330 * ((x399 * x339) + x401 + (x330 * ((x399 * x338) + (-1 * x399 * x364) + x400)))) +
					   (x399 * x340))))) +
			(x397 * x363))) +
		  (x402 * x360) + (x397 * x349))) +
		x273;
	const GEN_FLT x404 = (-1 * x314 * x319) + (x379 * x307);
	const GEN_FLT x405 = (-1 * x404 * x320) + x305;
	const GEN_FLT x406 = (x309 * x329) + (-1 * x308 * x329);
	const GEN_FLT x407 = x406 * x350;
	const GEN_FLT x408 = x407 * x332;
	const GEN_FLT x409 = (x406 * x356) + (x330 * (x408 + (-1 * x407 * x331)));
	const GEN_FLT x410 = (x409 * x330) + (x407 * x334);
	const GEN_FLT x411 =
		(-1 * x368 *
		 (x404 +
		  (-1 * x366 *
		   ((x345 *
			 ((x407 * x335) +
			  (x330 * (x410 + (x330 * (x409 + (x406 * x365) + (x330 * ((x407 * x338) + (-1 * x407 * x364) + x408)))) +
					   (x407 * x340))) +
			  (x410 * x330) + (x407 * x341))) +
			(x405 * x363))) +
		  (x407 * x361) + (x410 * x360) + (x405 * x349))) +
		x305;
	out[0] = x191 + (x191 * x192);
	out[1] = x208 + (x208 * x192);
	out[2] = x221 + (x221 * x192);
	out[3] = x254 + (x254 * x192);
	out[4] = x286 + (x286 * x192);
	out[5] = x317 + (x317 * x192);
	out[6] = x369 + (x369 * x370);
	out[7] = x378 + (x378 * x370);
	out[8] = x387 + (x387 * x370);
	out[9] = x395 + (x395 * x370);
	out[10] = x403 + (x403 * x370);
	out[11] = x411 + (x411 * x370);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x9 + x3;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x20 = x15 * x11;
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x20 * x16;
	const GEN_FLT x24 =
		((x23 + (-1 * x22)) * sensor_x) + ((x21 + x18) * sensor_y) + ((x14 + (x15 * x12)) * sensor_z) + obj_pz;
	const GEN_FLT x25 = x1 * x4;
	const GEN_FLT x26 = x2 * x8;
	const GEN_FLT x27 = x26 + (-1 * x25);
	const GEN_FLT x28 = x19 * x19;
	const GEN_FLT x29 = x11 * x17;
	const GEN_FLT x30 = x15 * x19;
	const GEN_FLT x31 = x30 * x16;
	const GEN_FLT x32 =
		((x31 + x29) * sensor_x) + ((x21 + (-1 * x18)) * sensor_z) + ((x14 + (x28 * x15)) * sensor_y) + obj_py;
	const GEN_FLT x33 = x7 * x7;
	const GEN_FLT x34 = x5 + (x6 * x33);
	const GEN_FLT x35 = x16 * x16;
	const GEN_FLT x36 =
		((x14 + (x35 * x15)) * sensor_x) + ((x31 + (-1 * x29)) * sensor_y) + ((x23 + x22) * sensor_z) + obj_px;
	const GEN_FLT x37 = (x32 * x27) + (x34 * x36) + (x24 * x10) + lh_px;
	const GEN_FLT x38 = x37 * x37;
	const GEN_FLT x39 = x4 * x4;
	const GEN_FLT x40 = x5 + (x6 * x39);
	const GEN_FLT x41 = x1 * x7;
	const GEN_FLT x42 = x4 * x6;
	const GEN_FLT x43 = x2 * x42;
	const GEN_FLT x44 = x43 + x41;
	const GEN_FLT x45 = x9 + (-1 * x3);
	const GEN_FLT x46 = (x45 * x36) + (x44 * x32) + (x40 * x24) + lh_pz;
	const GEN_FLT x47 = x38 + (x46 * x46);
	const GEN_FLT x48 = 1. / x47;
	const GEN_FLT x49 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x50 = x5 * x49;
	const GEN_FLT x51 = x2 * x50;
	const GEN_FLT x52 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x53 = x1 * x52;
	const GEN_FLT x54 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x55 = x6 * x54;
	const GEN_FLT x56 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x57 = (x8 * x56) + (x4 * x55) + (x7 * x49 * x25);
	const GEN_FLT x58 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x59 = x15 * x16;
	const GEN_FLT x60 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x61 = x60 * x17;
	const GEN_FLT x62 = -1 * x61;
	const GEN_FLT x63 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x64 = x63 * x17;
	const GEN_FLT x65 = x60 * x14;
	const GEN_FLT x66 = x65 * x11;
	const GEN_FLT x67 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x68 = x60 * x19;
	const GEN_FLT x69 = (x58 * x30) + (x68 * x18) + (x67 * x59);
	const GEN_FLT x70 = x67 * x17;
	const GEN_FLT x71 = x65 * x19;
	const GEN_FLT x72 = (x63 * x59) + (x60 * x11 * x18) + (x58 * x20);
	const GEN_FLT x73 = ((x72 + x71 + x70) * sensor_z) + ((x69 + (-1 * x66) + (-1 * x64)) * sensor_y) +
						(((x61 * x35) + x62 + (2 * x58 * x59)) * sensor_x);
	const GEN_FLT x74 = x7 * x50;
	const GEN_FLT x75 = x1 * x54;
	const GEN_FLT x76 = x3 * x49;
	const GEN_FLT x77 = x2 * x6;
	const GEN_FLT x78 = (x52 * x42) + (x77 * x56) + (x4 * x76);
	const GEN_FLT x79 = x58 * x17;
	const GEN_FLT x80 = x65 * x16;
	const GEN_FLT x81 = (x68 * x29) + (x63 * x30) + (x67 * x20);
	const GEN_FLT x82 = ((x81 + (-1 * x80) + (-1 * x79)) * sensor_z) +
						(((x61 * x28) + (2 * x67 * x30) + x62) * sensor_y) + ((x69 + x66 + x64) * sensor_x);
	const GEN_FLT x83 = x1 * x49;
	const GEN_FLT x84 = -1 * x83;
	const GEN_FLT x85 = (((x61 * x12) + (2 * x63 * x20) + x62) * sensor_z) + ((x81 + x80 + x79) * sensor_y) +
						((x72 + (-1 * x71) + (-1 * x70)) * sensor_x);
	const GEN_FLT x86 = (x24 * ((2 * x56 * x42) + (x83 * x39) + x84)) + (x82 * x44) + (x85 * x40) +
						(x32 * (x78 + x75 + x74)) + (x73 * x45) + (x36 * (x57 + (-1 * x53) + (-1 * x51)));
	const GEN_FLT x87 = x4 * x50;
	const GEN_FLT x88 = x1 * x56;
	const GEN_FLT x89 = (x7 * x76) + (x8 * x52) + (x2 * x55);
	const GEN_FLT x90 = (x85 * x10) + (x24 * (x57 + x53 + x51)) + (x73 * x34) + (x82 * x27) +
						(x32 * (x89 + (-1 * x88) + (-1 * x87))) + (x36 * ((2 * x8 * x54) + (x83 * x33) + x84));
	const GEN_FLT x91 = x48 * x38 * ((x90 * x46 * (1. / x38)) + (-1 * x86 * (1. / x37)));
	const GEN_FLT x92 = 0.523598775598299 + tilt_0;
	const GEN_FLT x93 = cos(x92);
	const GEN_FLT x94 = 1. / x93;
	const GEN_FLT x95 = x43 + (-1 * x41);
	const GEN_FLT x96 = x2 * x2;
	const GEN_FLT x97 = x5 + (x6 * x96);
	const GEN_FLT x98 = x26 + x25;
	const GEN_FLT x99 = (x98 * x36) + (x97 * x32) + (x95 * x24) + lh_py;
	const GEN_FLT x100 = x99 * x99;
	const GEN_FLT x101 = x47 + x100;
	const GEN_FLT x102 = 1. / sqrt(x101);
	const GEN_FLT x103 = x99 * x102;
	const GEN_FLT x104 = asin(x94 * x103);
	const GEN_FLT x105 = -8.0108022e-06 + (-1.60216044e-05 * x104);
	const GEN_FLT x106 = 8.0108022e-06 * x104;
	const GEN_FLT x107 = -8.0108022e-06 + (-1 * x106);
	const GEN_FLT x108 = 0.0028679863 + (x104 * x107);
	const GEN_FLT x109 = x108 + (x105 * x104);
	const GEN_FLT x110 = 5.3685255e-06 + (x108 * x104);
	const GEN_FLT x111 = x110 + (x109 * x104);
	const GEN_FLT x112 = 0.0076069798 + (x104 * x110);
	const GEN_FLT x113 = x112 + (x104 * x111);
	const GEN_FLT x114 = 1. / (x93 * x93);
	const GEN_FLT x115 = (1. / x101) * x100;
	const GEN_FLT x116 = 1. / sqrt(1 + (-1 * x114 * x115));
	const GEN_FLT x117 = (x24 * (x78 + (-1 * x75) + (-1 * x74))) + (x85 * x95) + (x82 * x97) +
						 (x32 * ((2 * x77 * x52) + (x83 * x96) + x84)) + (x73 * x98) + (x36 * (x89 + x88 + x87));
	const GEN_FLT x118 = (2 * x86 * x46) + (2 * x90 * x37);
	const GEN_FLT x119 = 1.0 / 2.0 * x99;
	const GEN_FLT x120 = (1. / (x101 * sqrt(x101))) * x119 * (x118 + (2 * x99 * x117));
	const GEN_FLT x121 = x102 * x117;
	const GEN_FLT x122 = (x94 * x121) + (-1 * x94 * x120);
	const GEN_FLT x123 = x116 * x122;
	const GEN_FLT x124 = x107 * x123;
	const GEN_FLT x125 = 2.40324066e-05 * x104;
	const GEN_FLT x126 = (x104 * (x124 + (-1 * x106 * x123))) + (x108 * x123);
	const GEN_FLT x127 = x110 * x116;
	const GEN_FLT x128 = (x104 * x126) + (x122 * x127);
	const GEN_FLT x129 = sin(x92);
	const GEN_FLT x130 = tan(x92);
	const GEN_FLT x131 = 1. / sqrt(x47);
	const GEN_FLT x132 = x99 * x131;
	const GEN_FLT x133 = x130 * x132;
	const GEN_FLT x134 = atan2(-1 * x46, x37);
	const GEN_FLT x135 = x134 + (-1 * asin(x133)) + ogeeMag_0;
	const GEN_FLT x136 = sin(x135);
	const GEN_FLT x137 = (x136 * ogeePhase_0) + curve_0;
	const GEN_FLT x138 = x129 * x137;
	const GEN_FLT x139 =
		-1 * x138 *
		((x104 * x128) +
		 (x104 * (x128 + (x104 * (x126 + (x104 * ((x105 * x123) + (-1 * x123 * x125) + x124)) + (x109 * x123))) +
				  (x111 * x123))) +
		 (x112 * x123) + (x113 * x123));
	const GEN_FLT x140 = x104 * x112;
	const GEN_FLT x141 = (x104 * x113) + x140;
	const GEN_FLT x142 = x129 * x141;
	const GEN_FLT x143 = x130 * x130;
	const GEN_FLT x144 = x48 * x100;
	const GEN_FLT x145 = 1. / sqrt(1 + (-1 * x144 * x143));
	const GEN_FLT x146 = (1. / (x47 * sqrt(x47))) * x118 * x119;
	const GEN_FLT x147 = x117 * x131;
	const GEN_FLT x148 = (x130 * x147) + (-1 * x130 * x146);
	const GEN_FLT x149 = (-1 * x145 * x148) + x91;
	const GEN_FLT x150 = cos(x135) * ogeePhase_0;
	const GEN_FLT x151 = x149 * x150;
	const GEN_FLT x152 = x93 + (-1 * x138 * x141);
	const GEN_FLT x153 = x104 * x104;
	const GEN_FLT x154 = x112 * x153;
	const GEN_FLT x155 = x137 * (1. / (x152 * x152)) * x154;
	const GEN_FLT x156 = 1. / x152;
	const GEN_FLT x157 = x154 * x156;
	const GEN_FLT x158 = x137 * x156;
	const GEN_FLT x159 = 2 * x140 * x158;
	const GEN_FLT x160 = x153 * x158;
	const GEN_FLT x161 = x148 + (x128 * x160) + (x123 * x159);
	const GEN_FLT x162 = x133 + (x154 * x158);
	const GEN_FLT x163 = 1. / sqrt(1 + (-1 * (x162 * x162)));
	const GEN_FLT x164 = (-1 * x163 * (x161 + (x151 * x157) + (-1 * x155 * ((-1 * x142 * x151) + x139)))) + x91;
	const GEN_FLT x165 = x134 + (-1 * asin(x162)) + gibPhase_0;
	const GEN_FLT x166 = cos(x165) * gibMag_0;
	const GEN_FLT x167 = x164 + (x166 * x164);
	const GEN_FLT x168 = x122 + (x103 * x114 * x129);
	const GEN_FLT x169 = x116 * x168;
	const GEN_FLT x170 = x107 * x169;
	const GEN_FLT x171 = (x104 * (x170 + (-1 * x106 * x169))) + (x108 * x169);
	const GEN_FLT x172 = (x104 * x171) + (x127 * x168);
	const GEN_FLT x173 = x148 + (x132 * (1 + x143));
	const GEN_FLT x174 = (-1 * x173 * x145) + x91;
	const GEN_FLT x175 = x142 * x150;
	const GEN_FLT x176 = x150 * x157;
	const GEN_FLT x177 =
		(-1 * x163 *
		 (x173 + (x160 * x172) + (x176 * x174) +
		  (-1 * x155 *
		   ((-1 * x174 * x175) +
			(-1 * x138 *
			 ((x112 * x169) + (x104 * x172) +
			  (x104 * (x172 + (x104 * (x171 + (x104 * ((x105 * x169) + (-1 * x125 * x169) + x170)) + (x109 * x169))) +
					   (x111 * x169))) +
			  (x113 * x169))) +
			(-1 * x93 * x137 * x141) + (-1 * x129))) +
		  (x169 * x159))) +
		x91;
	const GEN_FLT x178 = 1 + x151;
	const GEN_FLT x179 = (-1 * x163 * (x161 + (x178 * x157) + (-1 * x155 * ((-1 * x178 * x142) + x139)))) + x91;
	const GEN_FLT x180 = 1 + x149;
	const GEN_FLT x181 = (-1 * x163 * (x161 + (x176 * x180) + (-1 * x155 * ((-1 * x175 * x180) + x139)))) + x91;
	const GEN_FLT x182 = x136 + x151;
	const GEN_FLT x183 = (-1 * x163 * (x161 + (x182 * x157) + (-1 * x155 * ((-1 * x182 * x142) + x139)))) + x91;
	const GEN_FLT x184 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x185 = cos(x184);
	const GEN_FLT x186 = 1. / x185;
	const GEN_FLT x187 = asin(x103 * x186);
	const GEN_FLT x188 = 8.0108022e-06 * x187;
	const GEN_FLT x189 = -8.0108022e-06 + (-1 * x188);
	const GEN_FLT x190 = 0.0028679863 + (x187 * x189);
	const GEN_FLT x191 = 5.3685255e-06 + (x187 * x190);
	const GEN_FLT x192 = 0.0076069798 + (x187 * x191);
	const GEN_FLT x193 = tan(x184);
	const GEN_FLT x194 = -1 * x193 * x132;
	const GEN_FLT x195 = x134 + (-1 * asin(x194)) + ogeeMag_1;
	const GEN_FLT x196 = sin(x195);
	const GEN_FLT x197 = (x196 * ogeePhase_1) + curve_1;
	const GEN_FLT x198 = x187 * x187;
	const GEN_FLT x199 = sin(x184);
	const GEN_FLT x200 = x187 * x192;
	const GEN_FLT x201 = -8.0108022e-06 + (-1.60216044e-05 * x187);
	const GEN_FLT x202 = x190 + (x201 * x187);
	const GEN_FLT x203 = x191 + (x202 * x187);
	const GEN_FLT x204 = x192 + (x203 * x187);
	const GEN_FLT x205 = (x204 * x187) + x200;
	const GEN_FLT x206 = x205 * x199;
	const GEN_FLT x207 = x185 + (x206 * x197);
	const GEN_FLT x208 = 1. / x207;
	const GEN_FLT x209 = x208 * x198;
	const GEN_FLT x210 = x209 * x197;
	const GEN_FLT x211 = x194 + (x210 * x192);
	const GEN_FLT x212 = 1. / sqrt(1 + (-1 * (x211 * x211)));
	const GEN_FLT x213 = x193 * x193;
	const GEN_FLT x214 = 1. / sqrt(1 + (-1 * x213 * x144));
	const GEN_FLT x215 = (-1 * x193 * x147) + (x193 * x146);
	const GEN_FLT x216 = (-1 * x215 * x214) + x91;
	const GEN_FLT x217 = cos(x195) * ogeePhase_1;
	const GEN_FLT x218 = x217 * x216;
	const GEN_FLT x219 = x209 * x192;
	const GEN_FLT x220 = (x121 * x186) + (-1 * x120 * x186);
	const GEN_FLT x221 = 1. / (x185 * x185);
	const GEN_FLT x222 = 1. / sqrt(1 + (-1 * x221 * x115));
	const GEN_FLT x223 = x220 * x222;
	const GEN_FLT x224 = x223 * x189;
	const GEN_FLT x225 = 2.40324066e-05 * x187;
	const GEN_FLT x226 = (x223 * x190) + (x187 * (x224 + (-1 * x223 * x188)));
	const GEN_FLT x227 = (x226 * x187) + (x223 * x191);
	const GEN_FLT x228 = x197 * x199;
	const GEN_FLT x229 =
		x228 * ((x223 * x192) + (x227 * x187) +
				(x187 * (x227 + (x187 * (x226 + (x202 * x223) + (x187 * ((x201 * x223) + (-1 * x223 * x225) + x224)))) +
						 (x203 * x223))) +
				(x204 * x223));
	const GEN_FLT x230 = (1. / (x207 * x207)) * x197 * x198 * x192;
	const GEN_FLT x231 = 2 * x200 * x208 * x197;
	const GEN_FLT x232 = x215 + (x231 * x223) + (x210 * x227);
	const GEN_FLT x233 = (-1 * x212 * (x232 + (-1 * x230 * (x229 + (x218 * x206))) + (x219 * x218))) + x91;
	const GEN_FLT x234 = (-1 * asin(x211)) + x134 + gibPhase_1;
	const GEN_FLT x235 = cos(x234) * gibMag_1;
	const GEN_FLT x236 = x233 + (x233 * x235);
	const GEN_FLT x237 = x215 + (x132 * (1 + x213));
	const GEN_FLT x238 = (-1 * x214 * x237) + x91;
	const GEN_FLT x239 = x219 * x217;
	const GEN_FLT x240 = x222 * (x220 + (-1 * x221 * x103 * x199));
	const GEN_FLT x241 = x240 * x189;
	const GEN_FLT x242 = (x240 * x190) + (x187 * (x241 + (-1 * x240 * x188)));
	const GEN_FLT x243 = (x242 * x187) + (x240 * x191);
	const GEN_FLT x244 = x217 * x206;
	const GEN_FLT x245 =
		(-1 * x212 *
		 (x237 +
		  (-1 * x230 *
		   ((x228 *
			 ((x240 * x192) + (x243 * x187) +
			  (x187 * (x243 + (x187 * (x242 + (x202 * x240) + (x187 * ((x201 * x240) + (-1 * x225 * x240) + x241)))) +
					   (x203 * x240))) +
			  (x204 * x240))) +
			(-1 * x205 * x185 * x197) + x199 + (x238 * x244))) +
		  (x231 * x240) + (x210 * x243) + (x238 * x239))) +
		x91;
	const GEN_FLT x246 = 1 + x218;
	const GEN_FLT x247 = (-1 * x212 * (x232 + (-1 * x230 * (x229 + (x206 * x246))) + (x219 * x246))) + x91;
	const GEN_FLT x248 = 1 + x216;
	const GEN_FLT x249 = (-1 * x212 * (x232 + (-1 * x230 * (x229 + (x244 * x248))) + (x239 * x248))) + x91;
	const GEN_FLT x250 = x196 + x218;
	const GEN_FLT x251 = (-1 * x212 * (x232 + (-1 * x230 * (x229 + (x206 * x250))) + (x219 * x250))) + x91;
	out[0] = -1 + x167;
	out[1] = x177 + (x166 * x177);
	out[2] = x179 + (x166 * x179);
	out[3] = x164 + (x166 * (1 + x164));
	out[4] = x167 + sin(x165);
	out[5] = x181 + (x166 * x181);
	out[6] = x183 + (x166 * x183);
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
	out[22] = x245 + (x235 * x245);
	out[23] = x247 + (x235 * x247);
	out[24] = x233 + (x235 * (1 + x233));
	out[25] = x236 + sin(x234);
	out[26] = x249 + (x235 * x249);
	out[27] = x251 + (x235 * x251);
}

/** Applying function <function reproject_axis_x_gen2 at 0x7f6253d38950> */
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x11 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x12 = cos(x11);
	const GEN_FLT x13 = 1 + (-1 * x12);
	const GEN_FLT x14 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x15 = sin(x11);
	const GEN_FLT x16 = x15 * x14;
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x18 = x13 * x10 * x17;
	const GEN_FLT x19 = x15 * x17;
	const GEN_FLT x20 = x14 * x13;
	const GEN_FLT x21 = x20 * x10;
	const GEN_FLT x22 =
		((x21 + (-1 * x19)) * sensor_x) + ((x18 + x16) * sensor_y) + ((x12 + (x13 * (x10 * x10))) * sensor_z) + obj_pz;
	const GEN_FLT x23 = x15 * x10;
	const GEN_FLT x24 = x20 * x17;
	const GEN_FLT x25 =
		((x24 + x23) * sensor_x) + ((x12 + (x13 * (x17 * x17))) * sensor_y) + ((x18 + (-1 * x16)) * sensor_z) + obj_py;
	const GEN_FLT x26 = x1 * x7;
	const GEN_FLT x27 = x2 * x4 * x6;
	const GEN_FLT x28 =
		((x12 + ((x14 * x14) * x13)) * sensor_x) + ((x24 + (-1 * x23)) * sensor_y) + ((x21 + x19) * sensor_z) + obj_px;
	const GEN_FLT x29 = ((x27 + x26) * x28) + (x25 * (x5 + ((x4 * x4) * x6))) + ((x9 + (-1 * x3)) * x22) + lh_py;
	const GEN_FLT x30 = 0.523598775598299 + tilt_0;
	const GEN_FLT x31 = cos(x30);
	const GEN_FLT x32 = x1 * x4;
	const GEN_FLT x33 = x2 * x8;
	const GEN_FLT x34 = ((x33 + (-1 * x32)) * x28) + ((x9 + x3) * x25) + (x22 * (x5 + (x6 * (x7 * x7)))) + lh_pz;
	const GEN_FLT x35 = (x28 * (x5 + ((x2 * x2) * x6))) + ((x27 + (-1 * x26)) * x25) + ((x33 + x32) * x22) + lh_px;
	const GEN_FLT x36 = (x35 * x35) + (x34 * x34);
	const GEN_FLT x37 = asin((1. / x31) * x29 * (1. / sqrt(x36 + (x29 * x29))));
	const GEN_FLT x38 = 0.0028679863 + (x37 * (-8.0108022e-06 + (-8.0108022e-06 * x37)));
	const GEN_FLT x39 = 5.3685255e-06 + (x38 * x37);
	const GEN_FLT x40 = 0.0076069798 + (x37 * x39);
	const GEN_FLT x41 = (1. / sqrt(x36)) * x29 * tan(x30);
	const GEN_FLT x42 = atan2(-1 * x34, x35);
	const GEN_FLT x43 = (sin(x42 + (-1 * asin(x41)) + ogeeMag_0) * ogeePhase_0) + curve_0;
	const GEN_FLT x44 =
		x42 +
		(-1 *
		 asin(x41 + (x40 * x43 * (x37 * x37) *
					 (1. / (x31 + (-1 * x43 * sin(x30) *
								   ((x37 * (x40 + (x37 * (x39 + (x37 * (x38 + (x37 * (-8.0108022e-06 +
																					  (-1.60216044e-05 * x37))))))))) +
									(x40 * x37))))))));
	out[0] = -1.5707963267949 + x44 + (-1 * phase_0) + (sin(x44 + gibPhase_0) * gibMag_0);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x5 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x9 + x3;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x20 * x11;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x15 * x11;
	const GEN_FLT x24 = x23 * x16;
	const GEN_FLT x25 =
		((x24 + (-1 * x22)) * sensor_x) + ((x21 + x18) * sensor_y) + ((x14 + (x15 * x12)) * sensor_z) + obj_pz;
	const GEN_FLT x26 = x1 * x5;
	const GEN_FLT x27 = x2 * x7;
	const GEN_FLT x28 = x4 * x27;
	const GEN_FLT x29 = x28 + (-1 * x26);
	const GEN_FLT x30 = x19 * x19;
	const GEN_FLT x31 = x11 * x17;
	const GEN_FLT x32 = x20 * x16;
	const GEN_FLT x33 =
		((x14 + (x30 * x15)) * sensor_y) + ((x32 + x31) * sensor_x) + ((x21 + (-1 * x18)) * sensor_z) + obj_py;
	const GEN_FLT x34 = x4 * x4;
	const GEN_FLT x35 = x6 + (x7 * x34);
	const GEN_FLT x36 = x16 * x16;
	const GEN_FLT x37 =
		((x32 + (-1 * x31)) * sensor_y) + ((x24 + x22) * sensor_z) + ((x14 + (x36 * x15)) * sensor_x) + obj_px;
	const GEN_FLT x38 = (x35 * x37) + (x33 * x29) + (x25 * x10) + lh_px;
	const GEN_FLT x39 = 1. / x38;
	const GEN_FLT x40 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x41 = x40 * x17;
	const GEN_FLT x42 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x43 = x42 * x14;
	const GEN_FLT x44 = x43 * x11;
	const GEN_FLT x45 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x46 = x15 * x16;
	const GEN_FLT x47 = x42 * x18;
	const GEN_FLT x48 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x49 = (x48 * x20) + (x47 * x19) + (x45 * x46);
	const GEN_FLT x50 = x42 * x17;
	const GEN_FLT x51 = -1 * x50;
	const GEN_FLT x52 = 2 * x20;
	const GEN_FLT x53 = x48 * x17;
	const GEN_FLT x54 = x43 * x16;
	const GEN_FLT x55 = x11 * x19;
	const GEN_FLT x56 = (x50 * x55) + (x40 * x20) + (x45 * x23);
	const GEN_FLT x57 = ((x56 + (-1 * x54) + (-1 * x53)) * sensor_z) + (((x50 * x30) + (x52 * x45) + x51) * sensor_y) +
						((x49 + x44 + x41) * sensor_x);
	const GEN_FLT x58 = x1 * x4;
	const GEN_FLT x59 = x2 * x8;
	const GEN_FLT x60 = x59 + x58;
	const GEN_FLT x61 = x60 * x57;
	const GEN_FLT x62 = x9 + (-1 * x3);
	const GEN_FLT x63 = 2 * x46;
	const GEN_FLT x64 = x45 * x17;
	const GEN_FLT x65 = x43 * x19;
	const GEN_FLT x66 = (x40 * x46) + (x47 * x11) + (x48 * x23);
	const GEN_FLT x67 = ((x65 + x66 + x64) * sensor_z) + ((x49 + (-1 * x44) + (-1 * x41)) * sensor_y) +
						(((x50 * x36) + x51 + (x63 * x48)) * sensor_x);
	const GEN_FLT x68 = 1 + x67;
	const GEN_FLT x69 = x5 * x5;
	const GEN_FLT x70 = x6 + (x7 * x69);
	const GEN_FLT x71 = 2 * x23;
	const GEN_FLT x72 = (((x50 * x12) + (x71 * x40) + x51) * sensor_z) + ((x56 + x54 + x53) * sensor_y) +
						(((-1 * x65) + x66 + (-1 * x64)) * sensor_x);
	const GEN_FLT x73 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x74 = x6 * x73;
	const GEN_FLT x75 = x4 * x74;
	const GEN_FLT x76 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x77 = x1 * x76;
	const GEN_FLT x78 = x1 * x73;
	const GEN_FLT x79 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x80 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x81 = (x8 * x80) + (x79 * x27) + (x2 * x5 * x78);
	const GEN_FLT x82 = -1 * x78;
	const GEN_FLT x83 = x2 * x74;
	const GEN_FLT x84 = x1 * x80;
	const GEN_FLT x85 = x4 * x78;
	const GEN_FLT x86 = x4 * x7;
	const GEN_FLT x87 = (x86 * x79) + (x8 * x76) + (x5 * x85);
	const GEN_FLT x88 = (x37 * (x87 + (-1 * x84) + (-1 * x83))) + (x25 * ((2 * x8 * x79) + (x78 * x69) + x82)) +
						(x33 * (x81 + x77 + x75));
	const GEN_FLT x89 = x88 + (x70 * x72);
	const GEN_FLT x90 = x89 + (x62 * x68) + x61;
	const GEN_FLT x91 = x72 * x10;
	const GEN_FLT x92 = x5 * x74;
	const GEN_FLT x93 = x1 * x79;
	const GEN_FLT x94 = (x2 * x85) + (x80 * x86) + (x76 * x27);
	const GEN_FLT x95 = (x33 * (x94 + (-1 * x93) + (-1 * x92))) + (x25 * (x87 + x84 + x83)) +
						(x37 * ((2 * x86 * x76) + (x78 * x34) + x82));
	const GEN_FLT x96 = x95 + (x57 * x29);
	const GEN_FLT x97 = x96 + x91 + (x68 * x35);
	const GEN_FLT x98 = x38 * x38;
	const GEN_FLT x99 = (x62 * x37) + (x60 * x33) + (x70 * x25) + lh_pz;
	const GEN_FLT x100 = x99 * (1. / x98);
	const GEN_FLT x101 = x98 + (x99 * x99);
	const GEN_FLT x102 = 1. / x101;
	const GEN_FLT x103 = x98 * x102;
	const GEN_FLT x104 = x103 * ((x97 * x100) + (-1 * x90 * x39));
	const GEN_FLT x105 = x28 + x26;
	const GEN_FLT x106 = x2 * x2;
	const GEN_FLT x107 = x6 + (x7 * x106);
	const GEN_FLT x108 = x57 * x107;
	const GEN_FLT x109 = x59 + (-1 * x58);
	const GEN_FLT x110 = (x37 * (x94 + x93 + x92)) + (x25 * (x81 + (-1 * x77) + (-1 * x75))) +
						 (x33 * ((2 * x80 * x27) + (x78 * x106) + x82));
	const GEN_FLT x111 = x110 + (x72 * x109);
	const GEN_FLT x112 = x108 + x111 + (x68 * x105);
	const GEN_FLT x113 = (x37 * x105) + (x25 * x109) + (x33 * x107) + lh_py;
	const GEN_FLT x114 = 2 * x113;
	const GEN_FLT x115 = 2 * x38;
	const GEN_FLT x116 = 2 * x99;
	const GEN_FLT x117 = (x90 * x116) + (x97 * x115);
	const GEN_FLT x118 = 0.523598775598299 + tilt_0;
	const GEN_FLT x119 = cos(x118);
	const GEN_FLT x120 = 1. / x119;
	const GEN_FLT x121 = x113 * x113;
	const GEN_FLT x122 = x101 + x121;
	const GEN_FLT x123 = 1.0 / 2.0 * x113;
	const GEN_FLT x124 = x120 * x123 * (1. / (x122 * sqrt(x122)));
	const GEN_FLT x125 = x120 * (1. / sqrt(x122));
	const GEN_FLT x126 = 1. / sqrt(1 + (-1 * (1. / (x119 * x119)) * (1. / x122) * x121));
	const GEN_FLT x127 = x126 * ((x112 * x125) + (-1 * x124 * (x117 + (x112 * x114))));
	const GEN_FLT x128 = tan(x118);
	const GEN_FLT x129 = (1. / sqrt(x101)) * x128;
	const GEN_FLT x130 = x113 * x129;
	const GEN_FLT x131 = atan2(-1 * x99, x38);
	const GEN_FLT x132 = x131 + (-1 * asin(x130)) + ogeeMag_0;
	const GEN_FLT x133 = (sin(x132) * ogeePhase_0) + curve_0;
	const GEN_FLT x134 = asin(x113 * x125);
	const GEN_FLT x135 = 8.0108022e-06 * x134;
	const GEN_FLT x136 = -8.0108022e-06 + (-1 * x135);
	const GEN_FLT x137 = 0.0028679863 + (x134 * x136);
	const GEN_FLT x138 = 5.3685255e-06 + (x134 * x137);
	const GEN_FLT x139 = 0.0076069798 + (x134 * x138);
	const GEN_FLT x140 = x134 * x139;
	const GEN_FLT x141 = -8.0108022e-06 + (-1.60216044e-05 * x134);
	const GEN_FLT x142 = x137 + (x134 * x141);
	const GEN_FLT x143 = x138 + (x134 * x142);
	const GEN_FLT x144 = x139 + (x134 * x143);
	const GEN_FLT x145 = (x134 * x144) + x140;
	const GEN_FLT x146 = sin(x118);
	const GEN_FLT x147 = x133 * x146;
	const GEN_FLT x148 = x119 + (-1 * x145 * x147);
	const GEN_FLT x149 = 1. / x148;
	const GEN_FLT x150 = 2 * x133 * x140 * x149;
	const GEN_FLT x151 = (1. / (x101 * sqrt(x101))) * x123 * x128;
	const GEN_FLT x152 = (x112 * x129) + (-1 * x117 * x151);
	const GEN_FLT x153 = 1. / sqrt(1 + (-1 * x102 * x121 * (x128 * x128)));
	const GEN_FLT x154 = (-1 * x152 * x153) + x104;
	const GEN_FLT x155 = cos(x132) * ogeePhase_0;
	const GEN_FLT x156 = x134 * x134;
	const GEN_FLT x157 = x139 * x149 * x156;
	const GEN_FLT x158 = x155 * x157;
	const GEN_FLT x159 = x127 * x136;
	const GEN_FLT x160 = 2.40324066e-05 * x134;
	const GEN_FLT x161 = (x134 * (x159 + (-1 * x127 * x135))) + (x127 * x137);
	const GEN_FLT x162 = (x161 * x134) + (x127 * x138);
	const GEN_FLT x163 = x146 * x145;
	const GEN_FLT x164 = x163 * x155;
	const GEN_FLT x165 = x133 * x156;
	const GEN_FLT x166 = x165 * x139 * (1. / (x148 * x148));
	const GEN_FLT x167 = x165 * x149;
	const GEN_FLT x168 = x130 + (x167 * x139);
	const GEN_FLT x169 = 1. / sqrt(1 + (-1 * (x168 * x168)));
	const GEN_FLT x170 =
		(-1 * x169 *
		 (x152 + (x167 * x162) +
		  (-1 * x166 *
		   ((-1 * x164 * x154) +
			(-1 * x147 *
			 ((x127 * x139) +
			  (x134 * (x162 + (x134 * (x161 + (x134 * ((-1 * x127 * x160) + (x127 * x141) + x159)) + (x127 * x142))) +
					   (x127 * x143))) +
			  (x162 * x134) + (x127 * x144))))) +
		  (x154 * x158) + (x127 * x150))) +
		x104;
	const GEN_FLT x171 = cos(x131 + (-1 * asin(x168)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x172 = x62 * x67;
	const GEN_FLT x173 = 1 + x57;
	const GEN_FLT x174 = x89 + (x60 * x173) + x172;
	const GEN_FLT x175 = x67 * x35;
	const GEN_FLT x176 = x95 + (x29 * x173) + x175 + x91;
	const GEN_FLT x177 = x103 * ((x100 * x176) + (-1 * x39 * x174));
	const GEN_FLT x178 = x67 * x105;
	const GEN_FLT x179 = x111 + (x107 * x173) + x178;
	const GEN_FLT x180 = (x116 * x174) + (x115 * x176);
	const GEN_FLT x181 = (x125 * x179) + (-1 * x124 * (x180 + (x114 * x179)));
	const GEN_FLT x182 = x126 * x181;
	const GEN_FLT x183 = x182 * x136;
	const GEN_FLT x184 = (x134 * (x183 + (-1 * x182 * x135))) + (x182 * x137);
	const GEN_FLT x185 = (x184 * x134) + (x182 * x138);
	const GEN_FLT x186 = x126 * x139;
	const GEN_FLT x187 = (x129 * x179) + (-1 * x180 * x151);
	const GEN_FLT x188 = (-1 * x187 * x153) + x177;
	const GEN_FLT x189 =
		(-1 * x169 *
		 (x187 + (x188 * x158) + (x167 * x185) +
		  (-1 * x166 *
		   ((-1 * x164 * x188) +
			(-1 * x147 *
			 ((x185 * x134) + (x181 * x186) +
			  (x134 * (x185 + (x134 * (x184 + (x134 * ((x182 * x141) + (-1 * x160 * x182) + x183)) + (x182 * x142))) +
					   (x182 * x143))) +
			  (x182 * x144))))) +
		  (x182 * x150))) +
		x177;
	const GEN_FLT x190 = 1 + x72;
	const GEN_FLT x191 = x88 + x61 + (x70 * x190) + x172;
	const GEN_FLT x192 = x96 + (x10 * x190) + x175;
	const GEN_FLT x193 = x103 * ((x100 * x192) + (-1 * x39 * x191));
	const GEN_FLT x194 = x110 + (x109 * x190) + x108 + x178;
	const GEN_FLT x195 = (x116 * x191) + (x115 * x192);
	const GEN_FLT x196 = (x125 * x194) + (-1 * x124 * (x195 + (x114 * x194)));
	const GEN_FLT x197 = x126 * x196;
	const GEN_FLT x198 = x197 * x136;
	const GEN_FLT x199 = (x134 * (x198 + (-1 * x197 * x135))) + (x197 * x137);
	const GEN_FLT x200 = (x199 * x134) + (x197 * x138);
	const GEN_FLT x201 = (x129 * x194) + (-1 * x195 * x151);
	const GEN_FLT x202 = x155 * ((-1 * x201 * x153) + x193);
	const GEN_FLT x203 =
		(-1 * x169 *
		 ((x202 * x157) + (x200 * x167) +
		  (-1 * x166 *
		   ((-1 * x202 * x163) +
			(-1 * x147 *
			 ((x200 * x134) +
			  (x134 * (x200 + (x134 * (x199 + (x134 * ((x197 * x141) + (-1 * x160 * x197) + x198)) + (x197 * x142))) +
					   (x197 * x143))) +
			  (x186 * x196) + (x197 * x144))))) +
		  x201 + (x197 * x150))) +
		x193;
	const GEN_FLT x204 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x205 = x17 * x204;
	const GEN_FLT x206 = -1 * x205;
	const GEN_FLT x207 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qi *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x208 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qk *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x209 = x17 * x208;
	const GEN_FLT x210 = x14 * x11;
	const GEN_FLT x211 = x210 * x204;
	const GEN_FLT x212 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qj *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x213 = x19 * x16;
	const GEN_FLT x214 = (x213 * x205) + (x46 * x212) + (x20 * x207);
	const GEN_FLT x215 = x17 * x212;
	const GEN_FLT x216 = x14 * x19;
	const GEN_FLT x217 = x216 * x204;
	const GEN_FLT x218 = x11 * x16;
	const GEN_FLT x219 = (x218 * x205) + (x46 * x208) + (x23 * x207);
	const GEN_FLT x220 = ((x219 + x217 + x215) * sensor_z) + ((x214 + (-1 * x211) + (-1 * x209)) * sensor_y) +
						 (((x36 * x205) + (x63 * x207) + x206) * sensor_x);
	const GEN_FLT x221 = x17 * x207;
	const GEN_FLT x222 = x14 * x16;
	const GEN_FLT x223 = x204 * x222;
	const GEN_FLT x224 = (x55 * x205) + (x20 * x208) + (x23 * x212);
	const GEN_FLT x225 = ((x224 + (-1 * x223) + (-1 * x221)) * sensor_z) +
						 (((x30 * x205) + x206 + (x52 * x212)) * sensor_y) + ((x214 + x211 + x209) * sensor_x);
	const GEN_FLT x226 = (((x12 * x205) + (x71 * x208) + x206) * sensor_z) + ((x224 + x223 + x221) * sensor_y) +
						 ((x219 + (-1 * x217) + (-1 * x215)) * sensor_x);
	const GEN_FLT x227 = x88 + (x70 * x226) + (x60 * x225) + (x62 * x220);
	const GEN_FLT x228 = x95 + (x10 * x226) + (x29 * x225) + (x35 * x220);
	const GEN_FLT x229 = x103 * ((x228 * x100) + (-1 * x39 * x227));
	const GEN_FLT x230 = x110 + (x226 * x109) + (x225 * x107) + (x220 * x105);
	const GEN_FLT x231 = (x227 * x116) + (x228 * x115);
	const GEN_FLT x232 = (x230 * x125) + (-1 * x124 * (x231 + (x230 * x114)));
	const GEN_FLT x233 = x232 * x126;
	const GEN_FLT x234 = x233 * x136;
	const GEN_FLT x235 = (x134 * (x234 + (-1 * x233 * x135))) + (x233 * x137);
	const GEN_FLT x236 = (x235 * x134) + (x233 * x138);
	const GEN_FLT x237 = (x230 * x129) + (-1 * x231 * x151);
	const GEN_FLT x238 = (-1 * x237 * x153) + x229;
	const GEN_FLT x239 =
		(-1 * x169 *
		 ((x238 * x158) + (x236 * x167) + x237 +
		  (-1 * x166 *
		   ((-1 * x238 * x164) +
			(-1 * x147 *
			 ((x236 * x134) + (x232 * x186) +
			  (x134 * (x236 + (x134 * ((x134 * ((x233 * x141) + (-1 * x233 * x160) + x234)) + x235 + (x233 * x142))) +
					   (x233 * x143))) +
			  (x233 * x144))))) +
		  (x233 * x150))) +
		x229;
	const GEN_FLT x240 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x241 = x17 * x240;
	const GEN_FLT x242 = -1 * x241;
	const GEN_FLT x243 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qi *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x244 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qk *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x245 = x17 * x244;
	const GEN_FLT x246 = x210 * x240;
	const GEN_FLT x247 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qj *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x248 = (x213 * x241) + (x46 * x247) + (x20 * x243);
	const GEN_FLT x249 = x17 * x247;
	const GEN_FLT x250 = x216 * x240;
	const GEN_FLT x251 = (x46 * x244) + (x218 * x241) + (x23 * x243);
	const GEN_FLT x252 = ((x251 + x250 + x249) * sensor_z) + ((x248 + (-1 * x246) + (-1 * x245)) * sensor_y) +
						 (((x36 * x241) + (x63 * x243) + x242) * sensor_x);
	const GEN_FLT x253 = x17 * x243;
	const GEN_FLT x254 = x222 * x240;
	const GEN_FLT x255 = (x23 * x247) + (x55 * x241) + (x20 * x244);
	const GEN_FLT x256 = ((x255 + (-1 * x254) + (-1 * x253)) * sensor_z) + ((x248 + x246 + x245) * sensor_x) +
						 (((x30 * x241) + (x52 * x247) + x242) * sensor_y);
	const GEN_FLT x257 = (((x12 * x241) + (x71 * x244) + x242) * sensor_z) + ((x255 + x254 + x253) * sensor_y) +
						 ((x251 + (-1 * x250) + (-1 * x249)) * sensor_x);
	const GEN_FLT x258 = x88 + (x70 * x257) + (x60 * x256) + (x62 * x252);
	const GEN_FLT x259 = (x10 * x257) + x95 + (x29 * x256) + (x35 * x252);
	const GEN_FLT x260 = x103 * ((x259 * x100) + (-1 * x39 * x258));
	const GEN_FLT x261 = x110 + (x257 * x109) + (x256 * x107) + (x252 * x105);
	const GEN_FLT x262 = (x258 * x116) + (x259 * x115);
	const GEN_FLT x263 = x126 * ((x261 * x125) + (-1 * x124 * (x262 + (x261 * x114))));
	const GEN_FLT x264 = x263 * x136;
	const GEN_FLT x265 = (x134 * (x264 + (-1 * x263 * x135))) + (x263 * x137);
	const GEN_FLT x266 = (x265 * x134) + (x263 * x138);
	const GEN_FLT x267 = (x261 * x129) + (-1 * x262 * x151);
	const GEN_FLT x268 = (-1 * x267 * x153) + x260;
	const GEN_FLT x269 =
		(-1 * x169 *
		 (x267 + (x268 * x158) + (x266 * x167) +
		  (-1 * x166 *
		   ((-1 * x268 * x164) +
			(-1 * x147 *
			 ((x266 * x134) + (x263 * x139) +
			  (x134 * (x266 + (x134 * (x265 + (x134 * ((x263 * x141) + (-1 * x263 * x160) + x264)) + (x263 * x142))) +
					   (x263 * x143))) +
			  (x263 * x144))))) +
		  (x263 * x150))) +
		x260;
	const GEN_FLT x270 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x271 = x17 * x270;
	const GEN_FLT x272 = -1 * x271;
	const GEN_FLT x273 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qi *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x274 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qk *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x275 = x17 * x274;
	const GEN_FLT x276 = x210 * x270;
	const GEN_FLT x277 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qj *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x278 = x15 * x277;
	const GEN_FLT x279 = (x213 * x271) + (x16 * x278) + (x20 * x273);
	const GEN_FLT x280 = x17 * x277;
	const GEN_FLT x281 = x216 * x270;
	const GEN_FLT x282 = (x46 * x274) + (x218 * x271) + (x23 * x273);
	const GEN_FLT x283 = ((x281 + x282 + x280) * sensor_z) + ((x279 + (-1 * x276) + (-1 * x275)) * sensor_y) +
						 (((x36 * x271) + (x63 * x273) + x272) * sensor_x);
	const GEN_FLT x284 = x17 * x273;
	const GEN_FLT x285 = x270 * x222;
	const GEN_FLT x286 = (x55 * x271) + (x20 * x274) + (x11 * x278);
	const GEN_FLT x287 = ((x286 + (-1 * x285) + (-1 * x284)) * sensor_z) +
						 ((x272 + (x30 * x271) + (x52 * x277)) * sensor_y) + ((x279 + x276 + x275) * sensor_x);
	const GEN_FLT x288 = (((x12 * x271) + (x71 * x274) + x272) * sensor_z) + ((x286 + x285 + x284) * sensor_y) +
						 (((-1 * x281) + x282 + (-1 * x280)) * sensor_x);
	const GEN_FLT x289 = x88 + (x70 * x288) + (x60 * x287) + (x62 * x283);
	const GEN_FLT x290 = x95 + (x29 * x287) + (x10 * x288) + (x35 * x283);
	const GEN_FLT x291 = x103 * ((x290 * x100) + (-1 * x39 * x289));
	const GEN_FLT x292 = (x288 * x109) + x110 + (x287 * x107) + (x283 * x105);
	const GEN_FLT x293 = (x289 * x116) + (x290 * x115);
	const GEN_FLT x294 = (x292 * x125) + (-1 * x124 * (x293 + (x292 * x114)));
	const GEN_FLT x295 = x294 * x126;
	const GEN_FLT x296 = (x292 * x129) + (-1 * x293 * x151);
	const GEN_FLT x297 = (-1 * x296 * x153) + x291;
	const GEN_FLT x298 = x295 * x136;
	const GEN_FLT x299 = (x134 * (x298 + (-1 * x295 * x135))) + (x295 * x137);
	const GEN_FLT x300 = (x299 * x134) + (x295 * x138);
	const GEN_FLT x301 =
		(-1 * x169 *
		 (x296 + (x300 * x167) +
		  (-1 * x166 *
		   ((-1 * x297 * x164) +
			(-1 * x147 *
			 ((x300 * x134) + (x294 * x186) +
			  (x134 * (x300 + (x134 * (x299 + (x134 * ((x295 * x141) + (-1 * x295 * x160) + x298)) + (x295 * x142))) +
					   (x295 * x143))) +
			  (x295 * x144))))) +
		  (x297 * x158) + (x295 * x150))) +
		x291;
	out[0] = x170 + (x170 * x171);
	out[1] = x189 + (x171 * x189);
	out[2] = x203 + (x203 * x171);
	out[3] = x239 + (x239 * x171);
	out[4] = x269 + (x269 * x171);
	out[5] = x301 + (x301 * x171);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x9 + x3;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = x14 + (x15 * x12);
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x18 = sin(x13);
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x21 = x15 * x11;
	const GEN_FLT x22 = x20 * x21;
	const GEN_FLT x23 = x22 + x19;
	const GEN_FLT x24 = x20 * x18;
	const GEN_FLT x25 = x21 * x17;
	const GEN_FLT x26 = x25 + (-1 * x24);
	const GEN_FLT x27 = (x26 * sensor_x) + (x23 * sensor_y) + (x16 * sensor_z) + obj_pz;
	const GEN_FLT x28 = x1 * x4;
	const GEN_FLT x29 = x2 * x6;
	const GEN_FLT x30 = x7 * x29;
	const GEN_FLT x31 = x30 + (-1 * x28);
	const GEN_FLT x32 = x22 + (-1 * x19);
	const GEN_FLT x33 = x20 * x20;
	const GEN_FLT x34 = x14 + (x33 * x15);
	const GEN_FLT x35 = x11 * x18;
	const GEN_FLT x36 = x20 * x15;
	const GEN_FLT x37 = x36 * x17;
	const GEN_FLT x38 = x37 + x35;
	const GEN_FLT x39 = (x38 * sensor_x) + (x34 * sensor_y) + (x32 * sensor_z) + obj_py;
	const GEN_FLT x40 = x7 * x7;
	const GEN_FLT x41 = x5 + (x6 * x40);
	const GEN_FLT x42 = x25 + x24;
	const GEN_FLT x43 = x37 + (-1 * x35);
	const GEN_FLT x44 = x17 * x17;
	const GEN_FLT x45 = x14 + (x44 * x15);
	const GEN_FLT x46 = (x45 * sensor_x) + (x43 * sensor_y) + (x42 * sensor_z) + obj_px;
	const GEN_FLT x47 = (x41 * x46) + (x31 * x39) + (x27 * x10) + lh_px;
	const GEN_FLT x48 = 1. / x47;
	const GEN_FLT x49 = x9 + (-1 * x3);
	const GEN_FLT x50 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x51 = x15 * x17;
	const GEN_FLT x52 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x53 = x52 * x18;
	const GEN_FLT x54 = -1 * x53;
	const GEN_FLT x55 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x56 = x55 * x18;
	const GEN_FLT x57 = x52 * x14;
	const GEN_FLT x58 = x57 * x11;
	const GEN_FLT x59 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x60 = x52 * x19;
	const GEN_FLT x61 = (x50 * x36) + (x60 * x20) + (x51 * x59);
	const GEN_FLT x62 = x59 * x18;
	const GEN_FLT x63 = x57 * x20;
	const GEN_FLT x64 = (x51 * x55) + (x60 * x11) + (x50 * x21);
	const GEN_FLT x65 = ((x64 + x63 + x62) * sensor_z) + ((x61 + (-1 * x58) + (-1 * x56)) * sensor_y) +
						(((x53 * x44) + x54 + (2 * x50 * x51)) * sensor_x);
	const GEN_FLT x66 = x65 + x45;
	const GEN_FLT x67 = x1 * x7;
	const GEN_FLT x68 = x4 * x29;
	const GEN_FLT x69 = x68 + x67;
	const GEN_FLT x70 = x50 * x18;
	const GEN_FLT x71 = x57 * x17;
	const GEN_FLT x72 = (x52 * x35 * x20) + (x55 * x36) + (x59 * x21);
	const GEN_FLT x73 = ((x72 + (-1 * x71) + (-1 * x70)) * sensor_z) +
						(((2 * x59 * x36) + (x53 * x33) + x54) * sensor_y) + ((x61 + x58 + x56) * sensor_x);
	const GEN_FLT x74 = x73 + x38;
	const GEN_FLT x75 = x4 * x4;
	const GEN_FLT x76 = x5 + (x6 * x75);
	const GEN_FLT x77 = (((x53 * x12) + (2 * x55 * x21) + x54) * sensor_z) + ((x72 + x71 + x70) * sensor_y) +
						((x64 + (-1 * x63) + (-1 * x62)) * sensor_x);
	const GEN_FLT x78 = x77 + x26;
	const GEN_FLT x79 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x80 = x5 * x79;
	const GEN_FLT x81 = x2 * x80;
	const GEN_FLT x82 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x83 = x1 * x82;
	const GEN_FLT x84 = x7 * x79;
	const GEN_FLT x85 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x86 = x6 * x85;
	const GEN_FLT x87 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x88 = (x8 * x87) + (x4 * x86) + (x84 * x28);
	const GEN_FLT x89 = x7 * x80;
	const GEN_FLT x90 = x1 * x85;
	const GEN_FLT x91 = x4 * x6;
	const GEN_FLT x92 = (x82 * x91) + (x87 * x29) + (x4 * x3 * x79);
	const GEN_FLT x93 = x1 * x79;
	const GEN_FLT x94 = -1 * x93;
	const GEN_FLT x95 = (x27 * ((2 * x87 * x91) + (x75 * x93) + x94)) + (x39 * (x92 + x90 + x89)) +
						(x46 * (x88 + (-1 * x83) + (-1 * x81)));
	const GEN_FLT x96 = x95 + (x78 * x76) + (x74 * x69) + (x66 * x49);
	const GEN_FLT x97 = x4 * x80;
	const GEN_FLT x98 = x1 * x87;
	const GEN_FLT x99 = (x3 * x84) + (x8 * x82) + (x2 * x86);
	const GEN_FLT x100 = (x46 * ((2 * x7 * x86) + (x93 * x40) + x94)) + (x27 * (x88 + x83 + x81)) +
						 (x39 * (x99 + (-1 * x98) + (-1 * x97)));
	const GEN_FLT x101 = (x78 * x10) + (x74 * x31) + x100 + (x66 * x41);
	const GEN_FLT x102 = x47 * x47;
	const GEN_FLT x103 = (x46 * x49) + (x69 * x39) + (x76 * x27) + lh_pz;
	const GEN_FLT x104 = x103 * (1. / x102);
	const GEN_FLT x105 = x102 + (x103 * x103);
	const GEN_FLT x106 = 1. / x105;
	const GEN_FLT x107 = x102 * x106;
	const GEN_FLT x108 = x107 * ((x101 * x104) + (-1 * x96 * x48));
	const GEN_FLT x109 = x30 + x28;
	const GEN_FLT x110 = x2 * x2;
	const GEN_FLT x111 = x5 + (x6 * x110);
	const GEN_FLT x112 = x68 + (-1 * x67);
	const GEN_FLT x113 = (x27 * (x92 + (-1 * x90) + (-1 * x89))) + (x39 * ((2 * x82 * x29) + (x93 * x110) + x94)) +
						 (x46 * (x99 + x98 + x97));
	const GEN_FLT x114 = x113 + (x78 * x112) + (x74 * x111) + (x66 * x109);
	const GEN_FLT x115 = (x46 * x109) + (x27 * x112) + (x39 * x111) + lh_py;
	const GEN_FLT x116 = 2 * x115;
	const GEN_FLT x117 = 2 * x47;
	const GEN_FLT x118 = 2 * x103;
	const GEN_FLT x119 = (x96 * x118) + (x101 * x117);
	const GEN_FLT x120 = 0.523598775598299 + tilt_0;
	const GEN_FLT x121 = cos(x120);
	const GEN_FLT x122 = 1. / x121;
	const GEN_FLT x123 = x115 * x115;
	const GEN_FLT x124 = x105 + x123;
	const GEN_FLT x125 = 1.0 / 2.0 * x115;
	const GEN_FLT x126 = (1. / (x124 * sqrt(x124))) * x122 * x125;
	const GEN_FLT x127 = (1. / sqrt(x124)) * x122;
	const GEN_FLT x128 = 1. / sqrt(1 + (-1 * (1. / x124) * x123 * (1. / (x121 * x121))));
	const GEN_FLT x129 = x128 * ((x114 * x127) + (-1 * x126 * (x119 + (x114 * x116))));
	const GEN_FLT x130 = tan(x120);
	const GEN_FLT x131 = (1. / sqrt(x105)) * x130;
	const GEN_FLT x132 = x115 * x131;
	const GEN_FLT x133 = atan2(-1 * x103, x47);
	const GEN_FLT x134 = x133 + (-1 * asin(x132)) + ogeeMag_0;
	const GEN_FLT x135 = (sin(x134) * ogeePhase_0) + curve_0;
	const GEN_FLT x136 = asin(x115 * x127);
	const GEN_FLT x137 = 8.0108022e-06 * x136;
	const GEN_FLT x138 = -8.0108022e-06 + (-1 * x137);
	const GEN_FLT x139 = 0.0028679863 + (x138 * x136);
	const GEN_FLT x140 = 5.3685255e-06 + (x136 * x139);
	const GEN_FLT x141 = 0.0076069798 + (x136 * x140);
	const GEN_FLT x142 = x136 * x141;
	const GEN_FLT x143 = -8.0108022e-06 + (-1.60216044e-05 * x136);
	const GEN_FLT x144 = x139 + (x136 * x143);
	const GEN_FLT x145 = x140 + (x136 * x144);
	const GEN_FLT x146 = x141 + (x136 * x145);
	const GEN_FLT x147 = (x136 * x146) + x142;
	const GEN_FLT x148 = sin(x120);
	const GEN_FLT x149 = x135 * x148;
	const GEN_FLT x150 = x121 + (-1 * x147 * x149);
	const GEN_FLT x151 = 1. / x150;
	const GEN_FLT x152 = 2 * x135 * x142 * x151;
	const GEN_FLT x153 = x129 * x138;
	const GEN_FLT x154 = 2.40324066e-05 * x136;
	const GEN_FLT x155 = (x136 * (x153 + (-1 * x129 * x137))) + (x129 * x139);
	const GEN_FLT x156 = (x136 * x155) + (x129 * x140);
	const GEN_FLT x157 = 1. / sqrt(1 + (-1 * x106 * x123 * (x130 * x130)));
	const GEN_FLT x158 = (1. / (x105 * sqrt(x105))) * x125 * x130;
	const GEN_FLT x159 = (x114 * x131) + (-1 * x119 * x158);
	const GEN_FLT x160 = (-1 * x157 * x159) + x108;
	const GEN_FLT x161 = cos(x134) * ogeePhase_0;
	const GEN_FLT x162 = x148 * x147;
	const GEN_FLT x163 = x161 * x162;
	const GEN_FLT x164 = x136 * x136;
	const GEN_FLT x165 = x164 * x135;
	const GEN_FLT x166 = x165 * x141 * (1. / (x150 * x150));
	const GEN_FLT x167 = x165 * x151;
	const GEN_FLT x168 = x164 * x141 * x151;
	const GEN_FLT x169 = x161 * x168;
	const GEN_FLT x170 = x132 + (x167 * x141);
	const GEN_FLT x171 = 1. / sqrt(1 + (-1 * (x170 * x170)));
	const GEN_FLT x172 =
		(-1 * x171 *
		 ((x160 * x169) + (x167 * x156) + x159 +
		  (-1 * x166 *
		   ((-1 * x160 * x163) +
			(-1 * x149 *
			 ((x136 * x156) + (x129 * x141) + (x129 * x146) +
			  (x136 * (x156 + (x136 * (x155 + (x136 * ((x129 * x143) + (-1 * x129 * x154) + x153)) + (x129 * x144))) +
					   (x129 * x145))))))) +
		  (x129 * x152))) +
		x108;
	const GEN_FLT x173 = cos(x133 + (-1 * asin(x170)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x174 = x65 + x43;
	const GEN_FLT x175 = x73 + x34;
	const GEN_FLT x176 = x77 + x23;
	const GEN_FLT x177 = x95 + (x76 * x176) + (x69 * x175) + (x49 * x174);
	const GEN_FLT x178 = x100 + (x10 * x176) + (x31 * x175) + (x41 * x174);
	const GEN_FLT x179 = x107 * ((x104 * x178) + (-1 * x48 * x177));
	const GEN_FLT x180 = x113 + (x112 * x176) + (x111 * x175) + (x109 * x174);
	const GEN_FLT x181 = (x118 * x177) + (x117 * x178);
	const GEN_FLT x182 = x128 * ((x127 * x180) + (-1 * x126 * (x181 + (x116 * x180))));
	const GEN_FLT x183 = x182 * x138;
	const GEN_FLT x184 = (x136 * (x183 + (-1 * x182 * x137))) + (x182 * x139);
	const GEN_FLT x185 = (x184 * x136) + (x182 * x140);
	const GEN_FLT x186 = (x180 * x131) + (-1 * x181 * x158);
	const GEN_FLT x187 = (-1 * x186 * x157) + x179;
	const GEN_FLT x188 =
		(-1 * x171 *
		 (x186 + (x169 * x187) +
		  (-1 * x166 *
		   ((-1 * x163 * x187) +
			(-1 * x149 *
			 ((x185 * x136) + (x182 * x141) + (x182 * x146) +
			  (x136 * (x185 + (x136 * (x184 + (x136 * ((x182 * x143) + (-1 * x182 * x154) + x183)) + (x182 * x144))) +
					   (x182 * x145))))))) +
		  (x167 * x185) + (x182 * x152))) +
		x179;
	const GEN_FLT x189 = x65 + x42;
	const GEN_FLT x190 = x73 + x32;
	const GEN_FLT x191 = x77 + x16;
	const GEN_FLT x192 = x95 + (x76 * x191) + (x69 * x190) + (x49 * x189);
	const GEN_FLT x193 = x100 + (x31 * x190) + (x10 * x191) + (x41 * x189);
	const GEN_FLT x194 = x107 * ((x104 * x193) + (-1 * x48 * x192));
	const GEN_FLT x195 = x113 + (x109 * x189) + (x112 * x191) + (x111 * x190);
	const GEN_FLT x196 = (x118 * x192) + (x117 * x193);
	const GEN_FLT x197 = x128 * ((x127 * x195) + (-1 * x126 * (x196 + (x116 * x195))));
	const GEN_FLT x198 = x197 * x138;
	const GEN_FLT x199 = (x136 * (x198 + (-1 * x197 * x137))) + (x197 * x139);
	const GEN_FLT x200 = (x199 * x136) + (x197 * x140);
	const GEN_FLT x201 = (x195 * x131) + (-1 * x196 * x158);
	const GEN_FLT x202 = x161 * ((-1 * x201 * x157) + x194);
	const GEN_FLT x203 =
		(-1 * x171 *
		 ((x200 * x167) + x201 + (x202 * x168) +
		  (-1 * x166 *
		   ((-1 * x202 * x162) +
			(-1 * x149 *
			 ((x197 * x141) + (x197 * x146) + (x200 * x136) +
			  (x136 * (x200 + (x136 * (x199 + (x136 * ((x197 * x143) + (-1 * x197 * x154) + x198)) + (x197 * x144))) +
					   (x197 * x145))))))) +
		  (x197 * x152))) +
		x194;
	out[0] = x172 + (x172 * x173);
	out[1] = x188 + (x173 * x188);
	out[2] = x203 + (x203 * x173);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x9 + x3;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x20 * x11;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x15 * x16;
	const GEN_FLT x24 = x23 * x11;
	const GEN_FLT x25 =
		((x24 + (-1 * x22)) * sensor_x) + ((x21 + x18) * sensor_y) + ((x14 + (x15 * x12)) * sensor_z) + obj_pz;
	const GEN_FLT x26 = x1 * x7;
	const GEN_FLT x27 = x4 * x6;
	const GEN_FLT x28 = x2 * x27;
	const GEN_FLT x29 = x28 + (-1 * x26);
	const GEN_FLT x30 = x19 * x19;
	const GEN_FLT x31 = x11 * x17;
	const GEN_FLT x32 = x20 * x16;
	const GEN_FLT x33 =
		((x14 + (x30 * x15)) * sensor_y) + ((x32 + x31) * sensor_x) + ((x21 + (-1 * x18)) * sensor_z) + obj_py;
	const GEN_FLT x34 = x4 * x4;
	const GEN_FLT x35 = x5 + (x6 * x34);
	const GEN_FLT x36 = x16 * x16;
	const GEN_FLT x37 =
		((x32 + (-1 * x31)) * sensor_y) + ((x24 + x22) * sensor_z) + ((x14 + (x36 * x15)) * sensor_x) + obj_px;
	const GEN_FLT x38 = (x35 * x37) + (x33 * x29) + (x25 * x10) + lh_px;
	const GEN_FLT x39 = 1. / x38;
	const GEN_FLT x40 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = x2 * x41;
	const GEN_FLT x43 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x44 = x1 * x43;
	const GEN_FLT x45 = x1 * x40;
	const GEN_FLT x46 = x4 * x7;
	const GEN_FLT x47 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x48 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x49 = (x48 * x27) + (x8 * x47) + (x45 * x46);
	const GEN_FLT x50 = x4 * x41;
	const GEN_FLT x51 = x1 * x47;
	const GEN_FLT x52 = x2 * x7;
	const GEN_FLT x53 = x2 * x6;
	const GEN_FLT x54 = (x8 * x43) + (x53 * x48) + (x52 * x45);
	const GEN_FLT x55 = -1 * x45;
	const GEN_FLT x56 = x7 * x7;
	const GEN_FLT x57 = 2 * x8;
	const GEN_FLT x58 = x9 + (-1 * x3);
	const GEN_FLT x59 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x60 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x61 = x60 * x17;
	const GEN_FLT x62 = -1 * x61;
	const GEN_FLT x63 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x64 = x63 * x17;
	const GEN_FLT x65 = x60 * x14;
	const GEN_FLT x66 = x65 * x11;
	const GEN_FLT x67 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x68 = x60 * x18;
	const GEN_FLT x69 = (x59 * x20) + (x68 * x19) + (x67 * x23);
	const GEN_FLT x70 = x67 * x17;
	const GEN_FLT x71 = x65 * x19;
	const GEN_FLT x72 = x15 * x11;
	const GEN_FLT x73 = (x63 * x23) + (x68 * x11) + (x72 * x59);
	const GEN_FLT x74 = ((x73 + x71 + x70) * sensor_z) + ((x69 + (-1 * x66) + (-1 * x64)) * sensor_y) +
						(((x61 * x36) + x62 + (2 * x59 * x23)) * sensor_x);
	const GEN_FLT x75 = x59 * x17;
	const GEN_FLT x76 = x65 * x16;
	const GEN_FLT x77 = (x63 * x20) + (x60 * x22 * x11) + (x72 * x67);
	const GEN_FLT x78 = ((x77 + (-1 * x76) + (-1 * x75)) * sensor_z) +
						(((x61 * x30) + (2 * x67 * x20) + x62) * sensor_y) + ((x69 + x66 + x64) * sensor_x);
	const GEN_FLT x79 = x1 * x4;
	const GEN_FLT x80 = x2 * x8;
	const GEN_FLT x81 = x80 + x79;
	const GEN_FLT x82 = x5 + (x6 * x56);
	const GEN_FLT x83 = (((x61 * x12) + (2 * x72 * x63) + x62) * sensor_z) + ((x77 + x76 + x75) * sensor_y) +
						((x73 + (-1 * x71) + (-1 * x70)) * sensor_x);
	const GEN_FLT x84 = (x82 * x83) + (x81 * x78) + (x74 * x58);
	const GEN_FLT x85 = (x25 * ((x57 * x48) + (x56 * x45) + x55)) + x84 + (x33 * (x54 + x51 + x50)) +
						(x37 * (x49 + (-1 * x44) + (-1 * x42)));
	const GEN_FLT x86 = -1 * x85 * x39;
	const GEN_FLT x87 = 2 * x27;
	const GEN_FLT x88 = x7 * x41;
	const GEN_FLT x89 = x1 * x48;
	const GEN_FLT x90 = x2 * x4;
	const GEN_FLT x91 = (x90 * x45) + (x43 * x27) + (x53 * x47);
	const GEN_FLT x92 = (x83 * x10) + (x78 * x29) + (x74 * x35);
	const GEN_FLT x93 = x92 + (x33 * (x91 + (-1 * x89) + (-1 * x88))) + (x25 * (x49 + x44 + x42)) +
						(x37 * ((x87 * x47) + (x45 * x34) + x55));
	const GEN_FLT x94 = 1 + x93;
	const GEN_FLT x95 = x38 * x38;
	const GEN_FLT x96 = (x58 * x37) + (x81 * x33) + (x82 * x25) + lh_pz;
	const GEN_FLT x97 = x96 * (1. / x95);
	const GEN_FLT x98 = x95 + (x96 * x96);
	const GEN_FLT x99 = 1. / x98;
	const GEN_FLT x100 = x99 * x95;
	const GEN_FLT x101 = x100 * ((x97 * x94) + x86);
	const GEN_FLT x102 = x80 + (-1 * x79);
	const GEN_FLT x103 = x2 * x2;
	const GEN_FLT x104 = x5 + (x6 * x103);
	const GEN_FLT x105 = x28 + x26;
	const GEN_FLT x106 = (x37 * x105) + (x33 * x104) + (x25 * x102) + lh_py;
	const GEN_FLT x107 = x106 * x106;
	const GEN_FLT x108 = 0.523598775598299 + tilt_0;
	const GEN_FLT x109 = cos(x108);
	const GEN_FLT x110 = x98 + x107;
	const GEN_FLT x111 = 1. / sqrt(1 + (-1 * (1. / (x109 * x109)) * x107 * (1. / x110)));
	const GEN_FLT x112 = 2 * x53;
	const GEN_FLT x113 = (x83 * x102) + (x78 * x104) + (x74 * x105);
	const GEN_FLT x114 = x113 + (x25 * (x54 + (-1 * x51) + (-1 * x50))) + (x33 * ((x43 * x112) + (x45 * x103) + x55)) +
						 (x37 * (x91 + x89 + x88));
	const GEN_FLT x115 = 2 * x106;
	const GEN_FLT x116 = x114 * x115;
	const GEN_FLT x117 = 2 * x38;
	const GEN_FLT x118 = 2 * x96;
	const GEN_FLT x119 = x85 * x118;
	const GEN_FLT x120 = x119 + (x94 * x117);
	const GEN_FLT x121 = 1. / x109;
	const GEN_FLT x122 = 1.0 / 2.0 * x106;
	const GEN_FLT x123 = (1. / (x110 * sqrt(x110))) * x122 * x121;
	const GEN_FLT x124 = (1. / sqrt(x110)) * x121;
	const GEN_FLT x125 = x114 * x124;
	const GEN_FLT x126 = x125 + (-1 * (x120 + x116) * x123);
	const GEN_FLT x127 = x111 * x126;
	const GEN_FLT x128 = tan(x108);
	const GEN_FLT x129 = (1. / sqrt(x98)) * x128;
	const GEN_FLT x130 = x106 * x129;
	const GEN_FLT x131 = atan2(-1 * x96, x38);
	const GEN_FLT x132 = x131 + (-1 * asin(x130)) + ogeeMag_0;
	const GEN_FLT x133 = (sin(x132) * ogeePhase_0) + curve_0;
	const GEN_FLT x134 = asin(x106 * x124);
	const GEN_FLT x135 = 8.0108022e-06 * x134;
	const GEN_FLT x136 = -8.0108022e-06 + (-1 * x135);
	const GEN_FLT x137 = 0.0028679863 + (x134 * x136);
	const GEN_FLT x138 = 5.3685255e-06 + (x134 * x137);
	const GEN_FLT x139 = 0.0076069798 + (x134 * x138);
	const GEN_FLT x140 = x134 * x139;
	const GEN_FLT x141 = -8.0108022e-06 + (-1.60216044e-05 * x134);
	const GEN_FLT x142 = x137 + (x134 * x141);
	const GEN_FLT x143 = x138 + (x134 * x142);
	const GEN_FLT x144 = x139 + (x134 * x143);
	const GEN_FLT x145 = (x134 * x144) + x140;
	const GEN_FLT x146 = sin(x108);
	const GEN_FLT x147 = x133 * x146;
	const GEN_FLT x148 = x109 + (-1 * x145 * x147);
	const GEN_FLT x149 = 1. / x148;
	const GEN_FLT x150 = 2 * x133 * x140 * x149;
	const GEN_FLT x151 = x111 * x144;
	const GEN_FLT x152 = x111 * x143;
	const GEN_FLT x153 = x111 * x142;
	const GEN_FLT x154 = x111 * x136;
	const GEN_FLT x155 = x126 * x154;
	const GEN_FLT x156 = 2.40324066e-05 * x134;
	const GEN_FLT x157 = x111 * x137;
	const GEN_FLT x158 = (x134 * (x155 + (-1 * x127 * x135))) + (x126 * x157);
	const GEN_FLT x159 = (x134 * x158) + (x127 * x138);
	const GEN_FLT x160 = x111 * x139;
	const GEN_FLT x161 = 1. / sqrt(1 + (-1 * x99 * x107 * (x128 * x128)));
	const GEN_FLT x162 = (1. / (x98 * sqrt(x98))) * x122 * x128;
	const GEN_FLT x163 = x114 * x129;
	const GEN_FLT x164 = x163 + (-1 * x120 * x162);
	const GEN_FLT x165 = (-1 * x161 * x164) + x101;
	const GEN_FLT x166 = cos(x132) * ogeePhase_0;
	const GEN_FLT x167 = x146 * x145;
	const GEN_FLT x168 = x167 * x166;
	const GEN_FLT x169 = x134 * x134;
	const GEN_FLT x170 = x169 * x133;
	const GEN_FLT x171 = x170 * x139 * (1. / (x148 * x148));
	const GEN_FLT x172 = x170 * x149;
	const GEN_FLT x173 = x169 * x139 * x149;
	const GEN_FLT x174 = x166 * x173;
	const GEN_FLT x175 = x130 + (x172 * x139);
	const GEN_FLT x176 = 1. / sqrt(1 + (-1 * (x175 * x175)));
	const GEN_FLT x177 =
		(-1 * x176 *
		 (x164 + (x172 * x159) +
		  (-1 * x171 *
		   ((-1 * x168 * x165) +
			(-1 * x147 *
			 ((x134 * x159) +
			  (x134 * (x159 + (x134 * (x158 + (x134 * ((x127 * x141) + (-1 * x127 * x156) + x155)) + (x126 * x153))) +
					   (x126 * x152))) +
			  (x126 * x160) + (x126 * x151))))) +
		  (x165 * x174) + (x127 * x150))) +
		x101;
	const GEN_FLT x178 = cos(x131 + (-1 * asin(x175)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x179 = x93 * x97;
	const GEN_FLT x180 = x100 * (x179 + x86);
	const GEN_FLT x181 = 1 + x114;
	const GEN_FLT x182 = x93 * x117;
	const GEN_FLT x183 = x119 + x182;
	const GEN_FLT x184 = (x124 * x181) + (-1 * x123 * (x183 + (x115 * x181)));
	const GEN_FLT x185 = x111 * x184;
	const GEN_FLT x186 = x184 * x154;
	const GEN_FLT x187 = (x134 * (x186 + (-1 * x185 * x135))) + (x185 * x137);
	const GEN_FLT x188 = (x187 * x134) + (x185 * x138);
	const GEN_FLT x189 = (x129 * x181) + (-1 * x162 * x183);
	const GEN_FLT x190 = (-1 * x161 * x189) + x180;
	const GEN_FLT x191 =
		(-1 * x176 *
		 (x189 + (x172 * x188) + (x174 * x190) +
		  (-1 * x171 *
		   ((-1 * x168 * x190) +
			(-1 * x147 *
			 ((x188 * x134) + (x160 * x184) +
			  (x134 * (x188 + (x134 * (x187 + (x134 * ((x185 * x141) + (-1 * x185 * x156) + x186)) + (x185 * x142))) +
					   (x185 * x143))) +
			  (x185 * x144))))) +
		  (x185 * x150))) +
		x180;
	const GEN_FLT x192 = 1 + x85;
	const GEN_FLT x193 = x100 * (x179 + (-1 * x39 * x192));
	const GEN_FLT x194 = (x118 * x192) + x182;
	const GEN_FLT x195 = x125 + (-1 * (x194 + x116) * x123);
	const GEN_FLT x196 = x111 * x195;
	const GEN_FLT x197 = x195 * x154;
	const GEN_FLT x198 = (x134 * (x197 + (-1 * x196 * x135))) + (x195 * x157);
	const GEN_FLT x199 = (x198 * x134) + (x196 * x138);
	const GEN_FLT x200 = x163 + (-1 * x162 * x194);
	const GEN_FLT x201 = x166 * ((-1 * x200 * x161) + x193);
	const GEN_FLT x202 =
		(-1 * x176 *
		 (x200 + (x172 * x199) +
		  (-1 * x171 *
		   ((-1 * x201 * x167) +
			(-1 * x147 *
			 ((x199 * x134) +
			  (x134 * (x199 + (x134 * (x198 + (x134 * ((x196 * x141) + (-1 * x196 * x156) + x197)) + (x195 * x153))) +
					   (x195 * x152))) +
			  (x160 * x195) + (x195 * x151))))) +
		  (x201 * x173) + (x196 * x150))) +
		x193;
	const GEN_FLT x203 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x204 = x5 * x203;
	const GEN_FLT x205 = x2 * x204;
	const GEN_FLT x206 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qj *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x207 = x1 * x206;
	const GEN_FLT x208 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qi *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x209 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qk *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x210 = x1 * x203;
	const GEN_FLT x211 = (x46 * x210) + (x27 * x209) + (x8 * x208);
	const GEN_FLT x212 = x4 * x204;
	const GEN_FLT x213 = x1 * x208;
	const GEN_FLT x214 = x2 * x210;
	const GEN_FLT x215 = (x8 * x206) + (x53 * x209) + (x7 * x214);
	const GEN_FLT x216 = -1 * x210;
	const GEN_FLT x217 = (x25 * ((x57 * x209) + (x56 * x210) + x216)) + x84 + (x33 * (x215 + x213 + x212)) +
						 (x37 * (x211 + (-1 * x207) + (-1 * x205)));
	const GEN_FLT x218 = x7 * x204;
	const GEN_FLT x219 = x1 * x209;
	const GEN_FLT x220 = (x27 * x206) + (x53 * x208) + (x4 * x214);
	const GEN_FLT x221 = x92 + (x25 * (x211 + x207 + x205)) + (x33 * ((-1 * x219) + x220 + (-1 * x218))) +
						 (x37 * ((x87 * x208) + (x34 * x210) + x216));
	const GEN_FLT x222 = ((x97 * x221) + (-1 * x39 * x217)) * x100;
	const GEN_FLT x223 = (x37 * (x219 + x220 + x218)) + x113 + (x25 * (x215 + (-1 * x213) + (-1 * x212))) +
						 (x33 * ((x206 * x112) + (x210 * x103) + x216));
	const GEN_FLT x224 = (x217 * x118) + (x221 * x117);
	const GEN_FLT x225 = (x223 * x124) + (-1 * x123 * (x224 + (x223 * x115)));
	const GEN_FLT x226 = x225 * x111;
	const GEN_FLT x227 = x225 * x154;
	const GEN_FLT x228 = (x134 * (x227 + (-1 * x226 * x135))) + (x225 * x157);
	const GEN_FLT x229 = (x228 * x134) + (x226 * x138);
	const GEN_FLT x230 = (x223 * x129) + (-1 * x224 * x162);
	const GEN_FLT x231 = (-1 * x230 * x161) + x222;
	const GEN_FLT x232 =
		(-1 * x176 *
		 ((x229 * x172) + x230 +
		  (-1 * x171 *
		   ((-1 * x231 * x168) +
			(-1 * x147 *
			 ((x229 * x134) + (x225 * x160) +
			  (x134 * (x229 + (x134 * (x228 + (x134 * ((x226 * x141) + (-1 * x226 * x156) + x227)) + (x225 * x153))) +
					   (x225 * x152))) +
			  (x225 * x151))))) +
		  (x231 * x174) + (x226 * x150))) +
		x222;
	const GEN_FLT x233 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x234 = x5 * x233;
	const GEN_FLT x235 = x2 * x234;
	const GEN_FLT x236 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qj *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x237 = x1 * x236;
	const GEN_FLT x238 = x1 * x233;
	const GEN_FLT x239 = x7 * x238;
	const GEN_FLT x240 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qi *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x241 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qk *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x242 = (x27 * x241) + (x8 * x240) + (x4 * x239);
	const GEN_FLT x243 = x4 * x234;
	const GEN_FLT x244 = x1 * x240;
	const GEN_FLT x245 = (x53 * x241) + (x8 * x236) + (x2 * x239);
	const GEN_FLT x246 = -1 * x238;
	const GEN_FLT x247 = x84 + (x25 * ((x57 * x241) + (x56 * x238) + x246)) + (x33 * (x245 + x244 + x243)) +
						 (x37 * (x242 + (-1 * x237) + (-1 * x235)));
	const GEN_FLT x248 = x7 * x234;
	const GEN_FLT x249 = x1 * x241;
	const GEN_FLT x250 = (x53 * x240) + (x27 * x236) + (x90 * x238);
	const GEN_FLT x251 = (x25 * (x242 + x237 + x235)) + x92 + (x33 * (x250 + (-1 * x249) + (-1 * x248))) +
						 (x37 * ((x87 * x240) + x246 + (x34 * x238)));
	const GEN_FLT x252 = ((x97 * x251) + (-1 * x39 * x247)) * x100;
	const GEN_FLT x253 = x113 + (x25 * (x245 + (-1 * x244) + (-1 * x243))) +
						 (x33 * ((x236 * x112) + (x238 * x103) + x246)) + (x37 * (x250 + x249 + x248));
	const GEN_FLT x254 = (x247 * x118) + (x251 * x117);
	const GEN_FLT x255 = x111 * ((x253 * x124) + (-1 * x123 * (x254 + (x253 * x115))));
	const GEN_FLT x256 = x255 * x136;
	const GEN_FLT x257 = (x134 * (x256 + (-1 * x255 * x135))) + (x255 * x137);
	const GEN_FLT x258 = (x257 * x134) + (x255 * x138);
	const GEN_FLT x259 = (x253 * x129) + (-1 * x254 * x162);
	const GEN_FLT x260 = (-1 * x259 * x161) + x252;
	const GEN_FLT x261 =
		(-1 * x176 *
		 (x259 + (x260 * x174) + (x258 * x172) +
		  (-1 * x171 *
		   ((-1 * x260 * x168) +
			(-1 * x147 *
			 ((x258 * x134) + (x255 * x139) +
			  (x134 * (x258 + (x134 * (x257 + (x134 * ((x255 * x141) + (-1 * x255 * x156) + x256)) + (x255 * x142))) +
					   (x255 * x143))) +
			  (x255 * x144))))) +
		  (x255 * x150))) +
		x252;
	const GEN_FLT x262 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x263 = x5 * x262;
	const GEN_FLT x264 = x2 * x263;
	const GEN_FLT x265 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qj *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x266 = x1 * x265;
	const GEN_FLT x267 = x1 * x262;
	const GEN_FLT x268 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qi *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x269 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qk *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x270 = (x8 * x268) + (x27 * x269) + (x46 * x267);
	const GEN_FLT x271 = x4 * x263;
	const GEN_FLT x272 = x1 * x268;
	const GEN_FLT x273 = (x8 * x265) + (x53 * x269) + (x52 * x267);
	const GEN_FLT x274 = -1 * x267;
	const GEN_FLT x275 = x84 + (x25 * ((x57 * x269) + (x56 * x267) + x274)) + (x33 * (x273 + x272 + x271)) +
						 (x37 * (x270 + (-1 * x266) + (-1 * x264)));
	const GEN_FLT x276 = x7 * x263;
	const GEN_FLT x277 = x1 * x269;
	const GEN_FLT x278 = (x27 * x265) + (x53 * x268) + (x90 * x267);
	const GEN_FLT x279 = x92 + (x25 * (x270 + x266 + x264)) + (x33 * (x278 + (-1 * x277) + (-1 * x276))) +
						 (x37 * ((x87 * x268) + (x34 * x267) + x274));
	const GEN_FLT x280 = ((x97 * x279) + (-1 * x39 * x275)) * x100;
	const GEN_FLT x281 = x113 + (x25 * (x273 + (-1 * x272) + (-1 * x271))) +
						 (x33 * ((x267 * x103) + (x265 * x112) + x274)) + (x37 * (x278 + x277 + x276));
	const GEN_FLT x282 = (x275 * x118) + (x279 * x117);
	const GEN_FLT x283 = (x281 * x124) + (-1 * x123 * (x282 + (x281 * x115)));
	const GEN_FLT x284 = x283 * x111;
	const GEN_FLT x285 = x283 * x154;
	const GEN_FLT x286 = (x134 * (x285 + (-1 * x284 * x135))) + (x283 * x157);
	const GEN_FLT x287 = (x286 * x134) + (x284 * x138);
	const GEN_FLT x288 = (x281 * x129) + (-1 * x282 * x162);
	const GEN_FLT x289 = (-1 * x288 * x161) + x280;
	const GEN_FLT x290 =
		(-1 * x176 *
		 (x288 + (x287 * x172) + (x289 * x174) +
		  (-1 * x171 *
		   ((-1 * x289 * x168) +
			(-1 * x147 *
			 ((x287 * x134) + (x283 * x160) +
			  (x134 * ((x134 * (x286 + (x134 * ((x284 * x141) + (-1 * x284 * x156) + x285)) + (x283 * x153))) + x287 +
					   (x283 * x152))) +
			  (x283 * x151))))) +
		  (x284 * x150))) +
		x280;
	out[0] = x177 + (x178 * x177);
	out[1] = x191 + (x178 * x191);
	out[2] = x202 + (x202 * x178);
	out[3] = x232 + (x232 * x178);
	out[4] = x261 + (x261 * x178);
	out[5] = x290 + (x290 * x178);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x5 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x9 + x3;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x20 * x11;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x15 * x16;
	const GEN_FLT x24 = x23 * x11;
	const GEN_FLT x25 =
		((x24 + (-1 * x22)) * sensor_x) + ((x21 + x18) * sensor_y) + ((x14 + (x15 * x12)) * sensor_z) + obj_pz;
	const GEN_FLT x26 = x1 * x5;
	const GEN_FLT x27 = x4 * x7;
	const GEN_FLT x28 = x2 * x27;
	const GEN_FLT x29 = x28 + (-1 * x26);
	const GEN_FLT x30 = x19 * x19;
	const GEN_FLT x31 = x11 * x17;
	const GEN_FLT x32 = x20 * x16;
	const GEN_FLT x33 =
		((x14 + (x30 * x15)) * sensor_y) + ((x32 + x31) * sensor_x) + ((x21 + (-1 * x18)) * sensor_z) + obj_py;
	const GEN_FLT x34 = x4 * x4;
	const GEN_FLT x35 = x6 + (x7 * x34);
	const GEN_FLT x36 = x16 * x16;
	const GEN_FLT x37 =
		((x32 + (-1 * x31)) * sensor_y) + ((x24 + x22) * sensor_z) + ((x14 + (x36 * x15)) * sensor_x) + obj_px;
	const GEN_FLT x38 = (x35 * x37) + (x33 * x29) + (x25 * x10) + lh_px;
	const GEN_FLT x39 = x38 * x38;
	const GEN_FLT x40 = x5 * x5;
	const GEN_FLT x41 = x6 + (x7 * x40);
	const GEN_FLT x42 = x1 * x4;
	const GEN_FLT x43 = x2 * x8;
	const GEN_FLT x44 = x43 + x42;
	const GEN_FLT x45 = x9 + (-1 * x3);
	const GEN_FLT x46 = (x45 * x37) + (x44 * x33) + (x41 * x25) + lh_pz;
	const GEN_FLT x47 = x39 + (x46 * x46);
	const GEN_FLT x48 = 1. / x47;
	const GEN_FLT x49 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x50 = x6 * x49;
	const GEN_FLT x51 = x2 * x50;
	const GEN_FLT x52 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x53 = x1 * x52;
	const GEN_FLT x54 = x4 * x49;
	const GEN_FLT x55 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x56 = x7 * x55;
	const GEN_FLT x57 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x58 = (x57 * x27) + (x5 * x56) + (x54 * x26);
	const GEN_FLT x59 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x60 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x61 = x60 * x17;
	const GEN_FLT x62 = -1 * x61;
	const GEN_FLT x63 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x64 = x63 * x17;
	const GEN_FLT x65 = x60 * x14;
	const GEN_FLT x66 = x65 * x11;
	const GEN_FLT x67 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x68 = x60 * x18;
	const GEN_FLT x69 = (x59 * x20) + (x68 * x19) + (x67 * x23);
	const GEN_FLT x70 = x67 * x17;
	const GEN_FLT x71 = x65 * x19;
	const GEN_FLT x72 = x15 * x11;
	const GEN_FLT x73 = (x63 * x23) + (x68 * x11) + (x72 * x59);
	const GEN_FLT x74 = ((x73 + x71 + x70) * sensor_z) + ((x69 + (-1 * x66) + (-1 * x64)) * sensor_y) +
						(((x61 * x36) + x62 + (2 * x59 * x23)) * sensor_x);
	const GEN_FLT x75 = x4 * x50;
	const GEN_FLT x76 = x1 * x55;
	const GEN_FLT x77 = x2 * x7;
	const GEN_FLT x78 = (x8 * x52) + (x77 * x57) + (x3 * x5 * x49);
	const GEN_FLT x79 = x59 * x17;
	const GEN_FLT x80 = x65 * x16;
	const GEN_FLT x81 = (x60 * x31 * x19) + (x63 * x20) + (x72 * x67);
	const GEN_FLT x82 = ((x81 + (-1 * x80) + (-1 * x79)) * sensor_z) +
						(((x61 * x30) + (2 * x67 * x20) + x62) * sensor_y) + ((x69 + x66 + x64) * sensor_x);
	const GEN_FLT x83 = x1 * x49;
	const GEN_FLT x84 = -1 * x83;
	const GEN_FLT x85 = (((x61 * x12) + (2 * x72 * x63) + x62) * sensor_z) + ((x81 + x80 + x79) * sensor_y) +
						((x73 + (-1 * x71) + (-1 * x70)) * sensor_x);
	const GEN_FLT x86 = (x85 * x41) + (x25 * ((x83 * x40) + (2 * x8 * x57) + x84)) + (x82 * x44) +
						(x33 * (x78 + x76 + x75)) + (x74 * x45) + (x37 * (x58 + (-1 * x53) + (-1 * x51)));
	const GEN_FLT x87 = x5 * x50;
	const GEN_FLT x88 = x1 * x57;
	const GEN_FLT x89 = (x3 * x54) + (x52 * x27) + (x2 * x56);
	const GEN_FLT x90 = (x85 * x10) + (x25 * (x58 + x53 + x51)) + (x82 * x29) +
						(x33 * (x89 + (-1 * x88) + (-1 * x87))) + (x74 * x35) +
						(x37 * ((2 * x4 * x56) + (x83 * x34) + x84));
	const GEN_FLT x91 = x48 * x39 * ((x90 * x46 * (1. / x39)) + (-1 * x86 * (1. / x38)));
	const GEN_FLT x92 = x43 + (-1 * x42);
	const GEN_FLT x93 = x2 * x2;
	const GEN_FLT x94 = x6 + (x7 * x93);
	const GEN_FLT x95 = x28 + x26;
	const GEN_FLT x96 = (x95 * x37) + (x94 * x33) + (x92 * x25) + lh_py;
	const GEN_FLT x97 = 0.523598775598299 + tilt_0;
	const GEN_FLT x98 = cos(x97);
	const GEN_FLT x99 = 1. / x98;
	const GEN_FLT x100 = x96 * x96;
	const GEN_FLT x101 = x47 + x100;
	const GEN_FLT x102 = 1. / sqrt(x101);
	const GEN_FLT x103 = x99 * x102;
	const GEN_FLT x104 = asin(x96 * x103);
	const GEN_FLT x105 = -8.0108022e-06 + (-1.60216044e-05 * x104);
	const GEN_FLT x106 = 8.0108022e-06 * x104;
	const GEN_FLT x107 = -8.0108022e-06 + (-1 * x106);
	const GEN_FLT x108 = 0.0028679863 + (x104 * x107);
	const GEN_FLT x109 = x108 + (x105 * x104);
	const GEN_FLT x110 = 5.3685255e-06 + (x108 * x104);
	const GEN_FLT x111 = x110 + (x109 * x104);
	const GEN_FLT x112 = 0.0076069798 + (x104 * x110);
	const GEN_FLT x113 = x112 + (x104 * x111);
	const GEN_FLT x114 = 1. / (x98 * x98);
	const GEN_FLT x115 = 1. / sqrt(1 + (-1 * (1. / x101) * x100 * x114));
	const GEN_FLT x116 = (x85 * x92) + (x25 * (x78 + (-1 * x76) + (-1 * x75))) + (x82 * x94) +
						 (x33 * ((2 * x77 * x52) + (x83 * x93) + x84)) + (x74 * x95) + (x37 * (x89 + x88 + x87));
	const GEN_FLT x117 = (2 * x86 * x46) + (2 * x90 * x38);
	const GEN_FLT x118 = 1.0 / 2.0 * x96;
	const GEN_FLT x119 = (x103 * x116) + (-1 * x99 * (1. / (x101 * sqrt(x101))) * x118 * (x117 + (2 * x96 * x116)));
	const GEN_FLT x120 = x119 * x115;
	const GEN_FLT x121 = x107 * x120;
	const GEN_FLT x122 = 2.40324066e-05 * x104;
	const GEN_FLT x123 = (x104 * (x121 + (-1 * x106 * x120))) + (x108 * x120);
	const GEN_FLT x124 = (x104 * x123) + (x110 * x120);
	const GEN_FLT x125 = sin(x97);
	const GEN_FLT x126 = tan(x97);
	const GEN_FLT x127 = 1. / sqrt(x47);
	const GEN_FLT x128 = x96 * x127;
	const GEN_FLT x129 = x128 * x126;
	const GEN_FLT x130 = atan2(-1 * x46, x38);
	const GEN_FLT x131 = x130 + (-1 * asin(x129)) + ogeeMag_0;
	const GEN_FLT x132 = sin(x131);
	const GEN_FLT x133 = (x132 * ogeePhase_0) + curve_0;
	const GEN_FLT x134 = x125 * x133;
	const GEN_FLT x135 =
		-1 * x134 *
		((x104 * x124) + (x112 * x120) +
		 (x104 * (x124 + (x104 * (x123 + (x104 * ((x105 * x120) + (-1 * x120 * x122) + x121)) + (x109 * x120))) +
				  (x111 * x120))) +
		 (x113 * x120));
	const GEN_FLT x136 = x104 * x112;
	const GEN_FLT x137 = (x104 * x113) + x136;
	const GEN_FLT x138 = x125 * x137;
	const GEN_FLT x139 = x126 * x126;
	const GEN_FLT x140 = 1. / sqrt(1 + (-1 * x48 * x100 * x139));
	const GEN_FLT x141 = (x116 * x127 * x126) + (-1 * (1. / (x47 * sqrt(x47))) * x118 * x117 * x126);
	const GEN_FLT x142 = (-1 * x140 * x141) + x91;
	const GEN_FLT x143 = cos(x131) * ogeePhase_0;
	const GEN_FLT x144 = x142 * x143;
	const GEN_FLT x145 = x104 * x104;
	const GEN_FLT x146 = x98 + (-1 * x133 * x138);
	const GEN_FLT x147 = x112 * x133 * (1. / (x146 * x146)) * x145;
	const GEN_FLT x148 = 1. / x146;
	const GEN_FLT x149 = x145 * x148;
	const GEN_FLT x150 = x112 * x149;
	const GEN_FLT x151 = 2 * x133 * x136 * x148;
	const GEN_FLT x152 = x133 * x149;
	const GEN_FLT x153 = x141 + (x124 * x152) + (x120 * x151);
	const GEN_FLT x154 = x129 + (x112 * x152);
	const GEN_FLT x155 = 1. / sqrt(1 + (-1 * (x154 * x154)));
	const GEN_FLT x156 = (-1 * x155 * (x153 + (x144 * x150) + (-1 * x147 * ((-1 * x138 * x144) + x135)))) + x91;
	const GEN_FLT x157 = x130 + (-1 * asin(x154)) + gibPhase_0;
	const GEN_FLT x158 = cos(x157) * gibMag_0;
	const GEN_FLT x159 = x156 + (x156 * x158);
	const GEN_FLT x160 = x115 * (x119 + (x96 * x102 * x114 * x125));
	const GEN_FLT x161 = x107 * x160;
	const GEN_FLT x162 = (x104 * (x161 + (-1 * x106 * x160))) + (x108 * x160);
	const GEN_FLT x163 = (x104 * x162) + (x110 * x160);
	const GEN_FLT x164 = x141 + (x128 * (1 + x139));
	const GEN_FLT x165 = (-1 * x164 * x140) + x91;
	const GEN_FLT x166 = x138 * x143;
	const GEN_FLT x167 = x143 * x150;
	const GEN_FLT x168 =
		(-1 * x155 *
		 (x164 + (x163 * x152) + (x167 * x165) +
		  (-1 * x147 *
		   ((-1 * x98 * x133 * x137) + (-1 * x166 * x165) +
			(-1 * x134 *
			 ((x112 * x160) +
			  (x104 * (x163 + (x104 * (x162 + (x104 * ((-1 * x122 * x160) + (x105 * x160) + x161)) + (x109 * x160))) +
					   (x111 * x160))) +
			  (x104 * x163) + (x113 * x160))) +
			(-1 * x125))) +
		  (x160 * x151))) +
		x91;
	const GEN_FLT x169 = 1 + x144;
	const GEN_FLT x170 = (-1 * x155 * (x153 + (x169 * x150) + (-1 * x147 * ((-1 * x169 * x138) + x135)))) + x91;
	const GEN_FLT x171 = 1 + x142;
	const GEN_FLT x172 = (-1 * x155 * (x153 + (x167 * x171) + (-1 * x147 * ((-1 * x166 * x171) + x135)))) + x91;
	const GEN_FLT x173 = x132 + x144;
	const GEN_FLT x174 = (-1 * x155 * (x153 + (x173 * x150) + (-1 * x147 * ((-1 * x173 * x138) + x135)))) + x91;
	out[0] = -1 + x159;
	out[1] = x168 + (x168 * x158);
	out[2] = x170 + (x170 * x158);
	out[3] = x156 + (x158 * (1 + x156));
	out[4] = x159 + sin(x157);
	out[5] = x172 + (x172 * x158);
	out[6] = x174 + (x174 * x158);
}

/** Applying function <function reproject_axis_y_gen2 at 0x7f6253d389e0> */
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 + (-1 * x1);
	const GEN_FLT x3 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x4 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 0;
	const GEN_FLT x5 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						   : 1e-10;
	const GEN_FLT x6 = cos(x5);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 1;
	const GEN_FLT x9 = sin(x5);
	const GEN_FLT x10 = x8 * x9;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x7 * x11;
	const GEN_FLT x13 = x4 * x12;
	const GEN_FLT x14 = x9 * x11;
	const GEN_FLT x15 = x4 * x8 * x7;
	const GEN_FLT x16 =
		((x15 + (-1 * x14)) * sensor_x) + ((x13 + x10) * sensor_y) + ((x6 + ((x4 * x4) * x7)) * sensor_z) + obj_pz;
	const GEN_FLT x17 = sin(x0);
	const GEN_FLT x18 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 1;
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 0;
	const GEN_FLT x21 = x2 * x20;
	const GEN_FLT x22 = x3 * x21;
	const GEN_FLT x23 = x4 * x9;
	const GEN_FLT x24 = x8 * x12;
	const GEN_FLT x25 =
		((x24 + x23) * sensor_x) + ((x6 + (x7 * (x11 * x11))) * sensor_y) + ((x13 + (-1 * x10)) * sensor_z) + obj_py;
	const GEN_FLT x26 = x20 * x17;
	const GEN_FLT x27 = x2 * x3 * x18;
	const GEN_FLT x28 =
		((x6 + ((x8 * x8) * x7)) * sensor_x) + ((x24 + (-1 * x23)) * sensor_y) + ((x15 + x14) * sensor_z) + obj_px;
	const GEN_FLT x29 = ((x27 + (-1 * x26)) * x28) + ((x22 + x19) * x25) + (x16 * (x1 + (x2 * (x3 * x3)))) + lh_pz;
	const GEN_FLT x30 = x3 * x17;
	const GEN_FLT x31 = x21 * x18;
	const GEN_FLT x32 = (x28 * (x1 + (x2 * (x18 * x18)))) + ((x31 + (-1 * x30)) * x25) + ((x27 + x26) * x16) + lh_px;
	const GEN_FLT x33 = atan2(-1 * x29, x32);
	const GEN_FLT x34 = ((x31 + x30) * x28) + (x25 * (x1 + (x2 * (x20 * x20)))) + ((x22 + (-1 * x19)) * x16) + lh_py;
	const GEN_FLT x35 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x36 = (x32 * x32) + (x29 * x29);
	const GEN_FLT x37 = -1 * x34 * (1. / sqrt(x36)) * tan(x35);
	const GEN_FLT x38 = (sin(x33 + (-1 * asin(x37)) + ogeeMag_1) * ogeePhase_1) + curve_1;
	const GEN_FLT x39 = cos(x35);
	const GEN_FLT x40 = asin(x34 * (1. / x39) * (1. / sqrt(x36 + (x34 * x34))));
	const GEN_FLT x41 = 0.0028679863 + (x40 * (-8.0108022e-06 + (-8.0108022e-06 * x40)));
	const GEN_FLT x42 = 5.3685255e-06 + (x40 * x41);
	const GEN_FLT x43 = 0.0076069798 + (x40 * x42);
	const GEN_FLT x44 =
		(-1 *
		 asin(x37 + ((x40 * x40) * x43 * x38 *
					 (1. / (x39 + (x38 * sin(x35) *
								   ((x40 * (x43 + (x40 * (x42 + (x40 * (x41 + (x40 * (-8.0108022e-06 +
																					  (-1.60216044e-05 * x40))))))))) +
									(x40 * x43)))))))) +
		x33;
	out[0] = -1.5707963267949 + x44 + (sin(x44 + gibPhase_1) * gibMag_1) + (-1 * phase_1);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x9 + x3;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x20 = x15 * x11;
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x15 * x16;
	const GEN_FLT x24 = x23 * x11;
	const GEN_FLT x25 =
		((x24 + (-1 * x22)) * sensor_x) + ((x21 + x18) * sensor_y) + ((x14 + (x15 * x12)) * sensor_z) + obj_pz;
	const GEN_FLT x26 = x1 * x7;
	const GEN_FLT x27 = x2 * x6;
	const GEN_FLT x28 = x4 * x27;
	const GEN_FLT x29 = x28 + (-1 * x26);
	const GEN_FLT x30 = x19 * x19;
	const GEN_FLT x31 = x11 * x17;
	const GEN_FLT x32 = x23 * x19;
	const GEN_FLT x33 =
		((x14 + (x30 * x15)) * sensor_y) + ((x32 + x31) * sensor_x) + ((x21 + (-1 * x18)) * sensor_z) + obj_py;
	const GEN_FLT x34 = x4 * x4;
	const GEN_FLT x35 = x5 + (x6 * x34);
	const GEN_FLT x36 = x16 * x16;
	const GEN_FLT x37 =
		((x32 + (-1 * x31)) * sensor_y) + ((x24 + x22) * sensor_z) + ((x14 + (x36 * x15)) * sensor_x) + obj_px;
	const GEN_FLT x38 = (x35 * x37) + (x33 * x29) + (x25 * x10) + lh_px;
	const GEN_FLT x39 = 1. / x38;
	const GEN_FLT x40 = x7 * x7;
	const GEN_FLT x41 = x5 + (x6 * x40);
	const GEN_FLT x42 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x43 = x42 * x17;
	const GEN_FLT x44 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x45 = x44 * x14;
	const GEN_FLT x46 = x45 * x19;
	const GEN_FLT x47 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x48 = x44 * x18;
	const GEN_FLT x49 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x50 = (x49 * x23) + (x48 * x11) + (x47 * x20);
	const GEN_FLT x51 = x47 * x17;
	const GEN_FLT x52 = x45 * x16;
	const GEN_FLT x53 = x15 * x19;
	const GEN_FLT x54 = (x44 * x22 * x11) + (x53 * x49) + (x42 * x20);
	const GEN_FLT x55 = x44 * x17;
	const GEN_FLT x56 = -1 * x55;
	const GEN_FLT x57 = 2 * x20;
	const GEN_FLT x58 = (((x55 * x12) + (x57 * x49) + x56) * sensor_z) + ((x54 + x52 + x51) * sensor_y) +
						((x50 + (-1 * x46) + (-1 * x43)) * sensor_x);
	const GEN_FLT x59 = x58 * x41;
	const GEN_FLT x60 = x49 * x17;
	const GEN_FLT x61 = x14 * x11;
	const GEN_FLT x62 = x61 * x44;
	const GEN_FLT x63 = (x53 * x47) + (x48 * x19) + (x42 * x23);
	const GEN_FLT x64 = 2 * x53;
	const GEN_FLT x65 = ((x54 + (-1 * x52) + (-1 * x51)) * sensor_z) + (((x55 * x30) + (x64 * x42) + x56) * sensor_y) +
						((x63 + x62 + x60) * sensor_x);
	const GEN_FLT x66 = x1 * x4;
	const GEN_FLT x67 = x2 * x8;
	const GEN_FLT x68 = x67 + x66;
	const GEN_FLT x69 = x68 * x65;
	const GEN_FLT x70 = x9 + (-1 * x3);
	const GEN_FLT x71 = 2 * x23;
	const GEN_FLT x72 = ((x50 + x46 + x43) * sensor_z) + ((x63 + (-1 * x62) + (-1 * x60)) * sensor_y) +
						(((x55 * x36) + x56 + (x71 * x47)) * sensor_x);
	const GEN_FLT x73 = 1 + x72;
	const GEN_FLT x74 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x75 = x5 * x74;
	const GEN_FLT x76 = x4 * x75;
	const GEN_FLT x77 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x78 = x1 * x77;
	const GEN_FLT x79 = x1 * x74;
	const GEN_FLT x80 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x81 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x82 = x6 * x81;
	const GEN_FLT x83 = (x7 * x82) + (x80 * x27) + (x2 * x7 * x79);
	const GEN_FLT x84 = -1 * x79;
	const GEN_FLT x85 = x2 * x75;
	const GEN_FLT x86 = x1 * x81;
	const GEN_FLT x87 = x4 * x79;
	const GEN_FLT x88 = x4 * x6;
	const GEN_FLT x89 = (x80 * x88) + (x8 * x77) + (x7 * x87);
	const GEN_FLT x90 = (x37 * (x89 + (-1 * x86) + (-1 * x85))) + (x25 * ((2 * x8 * x80) + (x79 * x40) + x84)) +
						(x33 * (x83 + x78 + x76));
	const GEN_FLT x91 = x90 + (x70 * x73) + x69 + x59;
	const GEN_FLT x92 = x65 * x29;
	const GEN_FLT x93 = x58 * x10;
	const GEN_FLT x94 = x7 * x75;
	const GEN_FLT x95 = x1 * x80;
	const GEN_FLT x96 = (x2 * x87) + (x4 * x82) + (x77 * x27);
	const GEN_FLT x97 = (x37 * ((2 * x88 * x77) + (x79 * x34) + x84)) + (x25 * (x89 + x86 + x85)) +
						(x33 * (x96 + (-1 * x95) + (-1 * x94)));
	const GEN_FLT x98 = x97 + (x73 * x35) + x93 + x92;
	const GEN_FLT x99 = x38 * x38;
	const GEN_FLT x100 = (x70 * x37) + (x68 * x33) + (x41 * x25) + lh_pz;
	const GEN_FLT x101 = (1. / x99) * x100;
	const GEN_FLT x102 = x99 + (x100 * x100);
	const GEN_FLT x103 = 1. / x102;
	const GEN_FLT x104 = x99 * x103;
	const GEN_FLT x105 = x104 * ((x98 * x101) + (-1 * x91 * x39));
	const GEN_FLT x106 = x67 + (-1 * x66);
	const GEN_FLT x107 = x2 * x2;
	const GEN_FLT x108 = x5 + (x6 * x107);
	const GEN_FLT x109 = x28 + x26;
	const GEN_FLT x110 = (x33 * x108) + (x37 * x109) + (x25 * x106) + lh_py;
	const GEN_FLT x111 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x112 = cos(x111);
	const GEN_FLT x113 = 1. / x112;
	const GEN_FLT x114 = x110 * x110;
	const GEN_FLT x115 = x102 + x114;
	const GEN_FLT x116 = x113 * (1. / sqrt(x115));
	const GEN_FLT x117 = asin(x110 * x116);
	const GEN_FLT x118 = 8.0108022e-06 * x117;
	const GEN_FLT x119 = -8.0108022e-06 + (-1 * x118);
	const GEN_FLT x120 = 0.0028679863 + (x119 * x117);
	const GEN_FLT x121 = 5.3685255e-06 + (x117 * x120);
	const GEN_FLT x122 = 0.0076069798 + (x117 * x121);
	const GEN_FLT x123 = x117 * x122;
	const GEN_FLT x124 = -8.0108022e-06 + (-1.60216044e-05 * x117);
	const GEN_FLT x125 = x120 + (x117 * x124);
	const GEN_FLT x126 = x121 + (x117 * x125);
	const GEN_FLT x127 = x122 + (x117 * x126);
	const GEN_FLT x128 = (x117 * x127) + x123;
	const GEN_FLT x129 = tan(x111);
	const GEN_FLT x130 = (1. / sqrt(x102)) * x129;
	const GEN_FLT x131 = -1 * x110 * x130;
	const GEN_FLT x132 = atan2(-1 * x100, x38);
	const GEN_FLT x133 = x132 + (-1 * asin(x131)) + ogeeMag_1;
	const GEN_FLT x134 = (sin(x133) * ogeePhase_1) + curve_1;
	const GEN_FLT x135 = sin(x111);
	const GEN_FLT x136 = x134 * x135;
	const GEN_FLT x137 = x112 + (x128 * x136);
	const GEN_FLT x138 = 1. / x137;
	const GEN_FLT x139 = x117 * x117;
	const GEN_FLT x140 = x134 * x139;
	const GEN_FLT x141 = x138 * x140;
	const GEN_FLT x142 = x131 + (x122 * x141);
	const GEN_FLT x143 = 1. / sqrt(1 + (-1 * (x142 * x142)));
	const GEN_FLT x144 = 2 * x38;
	const GEN_FLT x145 = 2 * x100;
	const GEN_FLT x146 = (x91 * x145) + (x98 * x144);
	const GEN_FLT x147 = 1.0 / 2.0 * x110;
	const GEN_FLT x148 = (1. / (x102 * sqrt(x102))) * x129 * x147;
	const GEN_FLT x149 = x65 * x108;
	const GEN_FLT x150 = (x37 * (x96 + x95 + x94)) + (x25 * (x83 + (-1 * x78) + (-1 * x76))) +
						 (x33 * ((2 * x2 * x82) + (x79 * x107) + x84));
	const GEN_FLT x151 = x150 + (x58 * x106);
	const GEN_FLT x152 = x151 + x149 + (x73 * x109);
	const GEN_FLT x153 = (-1 * x130 * x152) + (x146 * x148);
	const GEN_FLT x154 = 1. / sqrt(1 + (-1 * x103 * x114 * (x129 * x129)));
	const GEN_FLT x155 = cos(x133) * ogeePhase_1;
	const GEN_FLT x156 = x155 * ((-1 * x153 * x154) + x105);
	const GEN_FLT x157 = x122 * x138 * x139;
	const GEN_FLT x158 = 1. / sqrt(1 + (-1 * (1. / (x112 * x112)) * x114 * (1. / x115)));
	const GEN_FLT x159 = 2 * x110;
	const GEN_FLT x160 = x113 * (1. / (x115 * sqrt(x115))) * x147;
	const GEN_FLT x161 = x158 * ((x116 * x152) + (-1 * x160 * (x146 + (x152 * x159))));
	const GEN_FLT x162 = 2 * x123 * x134 * x138;
	const GEN_FLT x163 = x119 * x161;
	const GEN_FLT x164 = (x120 * x161) + (x117 * (x163 + (-1 * x118 * x161)));
	const GEN_FLT x165 = (x117 * x164) + (x121 * x161);
	const GEN_FLT x166 = x128 * x135;
	const GEN_FLT x167 = 2.40324066e-05 * x117;
	const GEN_FLT x168 = x122 * (1. / (x137 * x137)) * x140;
	const GEN_FLT x169 =
		(-1 * x143 *
		 ((-1 * x168 *
		   ((x136 * ((x122 * x161) + (x117 * x165) +
					 (x117 * ((x117 * (x164 + (x125 * x161) + (x117 * ((x124 * x161) + (-1 * x161 * x167) + x163)))) +
							  x165 + (x126 * x161))) +
					 (x127 * x161))) +
			(x166 * x156))) +
		  (x165 * x141) + x153 + (x161 * x162) + (x156 * x157))) +
		x105;
	const GEN_FLT x170 = cos(x132 + (-1 * asin(x142)) + gibPhase_1) * gibMag_1;
	const GEN_FLT x171 = 1 + x65;
	const GEN_FLT x172 = x90 + (x70 * x72);
	const GEN_FLT x173 = x59 + x172 + (x68 * x171);
	const GEN_FLT x174 = x97 + (x72 * x35);
	const GEN_FLT x175 = x174 + x93 + (x29 * x171);
	const GEN_FLT x176 = x104 * ((x101 * x175) + (-1 * x39 * x173));
	const GEN_FLT x177 = (x173 * x145) + (x175 * x144);
	const GEN_FLT x178 = x72 * x109;
	const GEN_FLT x179 = x151 + (x108 * x171) + x178;
	const GEN_FLT x180 = (-1 * x179 * x130) + (x177 * x148);
	const GEN_FLT x181 = (-1 * x180 * x154) + x176;
	const GEN_FLT x182 = x155 * x157;
	const GEN_FLT x183 = (x116 * x179) + (-1 * x160 * (x177 + (x179 * x159)));
	const GEN_FLT x184 = x183 * x158;
	const GEN_FLT x185 = x119 * x184;
	const GEN_FLT x186 = (x120 * x184) + (x117 * (x185 + (-1 * x118 * x184)));
	const GEN_FLT x187 = (x117 * x186) + (x121 * x184);
	const GEN_FLT x188 = x166 * x155;
	const GEN_FLT x189 = x126 * x158;
	const GEN_FLT x190 =
		(-1 * x143 *
		 (x180 +
		  (-1 * x168 *
		   ((x136 *
			 ((x117 * x187) + (x122 * x184) +
			  (x117 * (x187 + (x117 * (x186 + (x125 * x184) + (x117 * ((x124 * x184) + (-1 * x167 * x184) + x185)))) +
					   (x189 * x183))) +
			  (x127 * x184))) +
			(x181 * x188))) +
		  (x187 * x141) + (x162 * x184) + (x181 * x182))) +
		x176;
	const GEN_FLT x191 = 1 + x58;
	const GEN_FLT x192 = (x41 * x191) + x172 + x69;
	const GEN_FLT x193 = x174 + (x10 * x191) + x92;
	const GEN_FLT x194 = x104 * ((x101 * x193) + (-1 * x39 * x192));
	const GEN_FLT x195 = (x192 * x145) + (x193 * x144);
	const GEN_FLT x196 = x150 + (x106 * x191) + x149 + x178;
	const GEN_FLT x197 = (-1 * x196 * x130) + (x195 * x148);
	const GEN_FLT x198 = (-1 * x197 * x154) + x194;
	const GEN_FLT x199 = x158 * ((x116 * x196) + (-1 * x160 * (x195 + (x196 * x159))));
	const GEN_FLT x200 = x119 * x199;
	const GEN_FLT x201 = (x120 * x199) + (x117 * (x200 + (-1 * x118 * x199)));
	const GEN_FLT x202 = (x201 * x117) + (x121 * x199);
	const GEN_FLT x203 =
		(-1 * x143 *
		 ((-1 * x168 *
		   ((x136 *
			 ((x202 * x117) + (x127 * x199) + (x122 * x199) +
			  (x117 * (x202 + (x117 * (x201 + (x125 * x199) + (x117 * ((x124 * x199) + (-1 * x167 * x199) + x200)))) +
					   (x126 * x199))))) +
			(x188 * x198))) +
		  (x162 * x199) + x197 + (x202 * x141) + (x182 * x198))) +
		x194;
	const GEN_FLT x204 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x205 = x17 * x204;
	const GEN_FLT x206 = -1 * x205;
	const GEN_FLT x207 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qi *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x208 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qk *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x209 = x17 * x208;
	const GEN_FLT x210 = x61 * x204;
	const GEN_FLT x211 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qj *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x212 = x16 * x205;
	const GEN_FLT x213 = (x19 * x212) + (x23 * x211) + (x53 * x207);
	const GEN_FLT x214 = x17 * x211;
	const GEN_FLT x215 = x14 * x19;
	const GEN_FLT x216 = x215 * x204;
	const GEN_FLT x217 = (x11 * x212) + (x23 * x208) + (x20 * x207);
	const GEN_FLT x218 = ((x217 + x216 + x214) * sensor_z) + ((x213 + (-1 * x210) + (-1 * x209)) * sensor_y) +
						 (((x36 * x205) + (x71 * x207) + x206) * sensor_x);
	const GEN_FLT x219 = x17 * x207;
	const GEN_FLT x220 = x14 * x16;
	const GEN_FLT x221 = x204 * x220;
	const GEN_FLT x222 = x11 * x19;
	const GEN_FLT x223 = (x205 * x222) + (x53 * x208) + (x20 * x211);
	const GEN_FLT x224 = ((x223 + (-1 * x221) + (-1 * x219)) * sensor_z) +
						 (((x30 * x205) + x206 + (x64 * x211)) * sensor_y) + ((x213 + x210 + x209) * sensor_x);
	const GEN_FLT x225 = (((x12 * x205) + (x57 * x208) + x206) * sensor_z) + ((x223 + x221 + x219) * sensor_y) +
						 ((x217 + (-1 * x216) + (-1 * x214)) * sensor_x);
	const GEN_FLT x226 = x90 + (x41 * x225) + (x68 * x224) + (x70 * x218);
	const GEN_FLT x227 = x97 + (x10 * x225) + (x29 * x224) + (x35 * x218);
	const GEN_FLT x228 = x104 * ((x227 * x101) + (-1 * x39 * x226));
	const GEN_FLT x229 = (x226 * x145) + (x227 * x144);
	const GEN_FLT x230 = x150 + (x225 * x106) + (x224 * x108) + (x218 * x109);
	const GEN_FLT x231 = (-1 * x230 * x130) + (x229 * x148);
	const GEN_FLT x232 = (-1 * x231 * x154) + x228;
	const GEN_FLT x233 = (x230 * x116) + (-1 * x160 * (x229 + (x230 * x159)));
	const GEN_FLT x234 = x233 * x158;
	const GEN_FLT x235 = x234 * x119;
	const GEN_FLT x236 = (x234 * x120) + (x117 * (x235 + (-1 * x234 * x118)));
	const GEN_FLT x237 = (x236 * x117) + (x234 * x121);
	const GEN_FLT x238 =
		(-1 * x143 *
		 (x231 +
		  (-1 * x168 *
		   ((x136 *
			 ((x237 * x117) +
			  (x117 * (x237 + (x117 * (x236 + (x234 * x125) + (x117 * ((-1 * x234 * x167) + (x234 * x124) + x235)))) +
					   (x233 * x189))) +
			  (x234 * x122) + (x234 * x127))) +
			(x232 * x188))) +
		  (x237 * x141) + (x234 * x162) + (x232 * x182))) +
		x228;
	const GEN_FLT x239 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x240 = x17 * x239;
	const GEN_FLT x241 = -1 * x240;
	const GEN_FLT x242 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qi *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x243 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qk *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x244 = x17 * x243;
	const GEN_FLT x245 = x61 * x239;
	const GEN_FLT x246 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qj *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x247 = x19 * x16;
	const GEN_FLT x248 = (x240 * x247) + (x23 * x246) + (x53 * x242);
	const GEN_FLT x249 = x17 * x246;
	const GEN_FLT x250 = x215 * x239;
	const GEN_FLT x251 = x11 * x16;
	const GEN_FLT x252 = (x23 * x243) + (x251 * x240) + (x20 * x242);
	const GEN_FLT x253 = ((x248 + (-1 * x245) + (-1 * x244)) * sensor_y) + ((x252 + x250 + x249) * sensor_z) +
						 (((x36 * x240) + (x71 * x242) + x241) * sensor_x);
	const GEN_FLT x254 = x17 * x242;
	const GEN_FLT x255 = x239 * x220;
	const GEN_FLT x256 = (x20 * x246) + (x222 * x240) + (x53 * x243);
	const GEN_FLT x257 = ((x256 + (-1 * x255) + (-1 * x254)) * sensor_z) + ((x248 + x245 + x244) * sensor_x) +
						 (((x64 * x246) + (x30 * x240) + x241) * sensor_y);
	const GEN_FLT x258 = (((x12 * x240) + (x57 * x243) + x241) * sensor_z) + ((x256 + x255 + x254) * sensor_y) +
						 ((x252 + (-1 * x250) + (-1 * x249)) * sensor_x);
	const GEN_FLT x259 = x90 + (x41 * x258) + (x68 * x257) + (x70 * x253);
	const GEN_FLT x260 = x97 + (x10 * x258) + (x29 * x257) + (x35 * x253);
	const GEN_FLT x261 = x104 * ((x260 * x101) + (-1 * x39 * x259));
	const GEN_FLT x262 = (x259 * x145) + (x260 * x144);
	const GEN_FLT x263 = x150 + (x258 * x106) + (x257 * x108) + (x253 * x109);
	const GEN_FLT x264 = (-1 * x263 * x130) + (x262 * x148);
	const GEN_FLT x265 = (-1 * x264 * x154) + x261;
	const GEN_FLT x266 = (x263 * x116) + (-1 * x160 * (x262 + (x263 * x159)));
	const GEN_FLT x267 = x266 * x158;
	const GEN_FLT x268 = x267 * x119;
	const GEN_FLT x269 = (x267 * x120) + (x117 * (x268 + (-1 * x267 * x118)));
	const GEN_FLT x270 = (x269 * x117) + (x267 * x121);
	const GEN_FLT x271 =
		(-1 * x143 *
		 (x264 +
		  (-1 * x168 *
		   ((x136 *
			 ((x267 * x122) + (x270 * x117) + (x267 * x127) +
			  (x117 * (x270 + (x117 * (x269 + (x267 * x125) + (x117 * (x268 + (x267 * x124) + (-1 * x267 * x167))))) +
					   (x266 * x189))))) +
			(x265 * x188))) +
		  (x267 * x162) + (x270 * x141) + (x265 * x182))) +
		x261;
	const GEN_FLT x272 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x273 = x17 * x272;
	const GEN_FLT x274 = -1 * x273;
	const GEN_FLT x275 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qi *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x276 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qk *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x277 = x17 * x276;
	const GEN_FLT x278 = x61 * x272;
	const GEN_FLT x279 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qj *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x280 = (x273 * x247) + (x23 * x279) + (x53 * x275);
	const GEN_FLT x281 = x17 * x279;
	const GEN_FLT x282 = x215 * x272;
	const GEN_FLT x283 = (x23 * x276) + (x273 * x251) + (x20 * x275);
	const GEN_FLT x284 = ((x282 + x283 + x281) * sensor_z) + ((x280 + (-1 * x278) + (-1 * x277)) * sensor_y) +
						 (((x36 * x273) + (x71 * x275) + x274) * sensor_x);
	const GEN_FLT x285 = x17 * x275;
	const GEN_FLT x286 = x272 * x220;
	const GEN_FLT x287 = (x273 * x222) + (x53 * x276) + (x20 * x279);
	const GEN_FLT x288 = ((x287 + (-1 * x286) + (-1 * x285)) * sensor_z) +
						 (((x30 * x273) + x274 + (x64 * x279)) * sensor_y) + ((x280 + x278 + x277) * sensor_x);
	const GEN_FLT x289 = (((x12 * x273) + (x57 * x276) + x274) * sensor_z) + ((x287 + x286 + x285) * sensor_y) +
						 (((-1 * x282) + x283 + (-1 * x281)) * sensor_x);
	const GEN_FLT x290 = x90 + (x41 * x289) + (x68 * x288) + (x70 * x284);
	const GEN_FLT x291 = x97 + (x10 * x289) + (x29 * x288) + (x35 * x284);
	const GEN_FLT x292 = x104 * ((x291 * x101) + (-1 * x39 * x290));
	const GEN_FLT x293 = (x290 * x145) + (x291 * x144);
	const GEN_FLT x294 = x150 + (x289 * x106) + (x288 * x108) + (x284 * x109);
	const GEN_FLT x295 = (-1 * x294 * x130) + (x293 * x148);
	const GEN_FLT x296 = (-1 * x295 * x154) + x292;
	const GEN_FLT x297 = x158 * ((x294 * x116) + (-1 * x160 * (x293 + (x294 * x159))));
	const GEN_FLT x298 = x297 * x119;
	const GEN_FLT x299 = (x297 * x120) + (x117 * (x298 + (-1 * x297 * x118)));
	const GEN_FLT x300 = (x299 * x117) + (x297 * x121);
	const GEN_FLT x301 =
		(-1 * x143 *
		 (x295 +
		  (-1 * x168 *
		   ((x136 *
			 ((x297 * x122) + (x297 * x127) + (x300 * x117) +
			  (x117 * (x300 + (x117 * (x299 + (x297 * x125) + (x117 * ((x297 * x124) + (-1 * x297 * x167) + x298)))) +
					   (x297 * x126))))) +
			(x296 * x188))) +
		  (x297 * x162) + (x300 * x141) + (x296 * x182))) +
		x292;
	out[0] = x169 + (x169 * x170);
	out[1] = x190 + (x170 * x190);
	out[2] = x203 + (x203 * x170);
	out[3] = x238 + (x238 * x170);
	out[4] = x271 + (x271 * x170);
	out[5] = x301 + (x301 * x170);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x9 + x3;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = x14 + (x15 * x12);
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x18 = sin(x13);
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x21 * x11;
	const GEN_FLT x23 = x22 + x19;
	const GEN_FLT x24 = x20 * x18;
	const GEN_FLT x25 = x15 * x11;
	const GEN_FLT x26 = x25 * x17;
	const GEN_FLT x27 = x26 + (-1 * x24);
	const GEN_FLT x28 = (x23 * sensor_y) + (x16 * sensor_z) + (x27 * sensor_x) + obj_pz;
	const GEN_FLT x29 = x1 * x4;
	const GEN_FLT x30 = x2 * x8;
	const GEN_FLT x31 = x30 + (-1 * x29);
	const GEN_FLT x32 = x22 + (-1 * x19);
	const GEN_FLT x33 = x20 * x20;
	const GEN_FLT x34 = x14 + (x33 * x15);
	const GEN_FLT x35 = x11 * x18;
	const GEN_FLT x36 = x21 * x17;
	const GEN_FLT x37 = x36 + x35;
	const GEN_FLT x38 = (x37 * sensor_x) + (x34 * sensor_y) + (x32 * sensor_z) + obj_py;
	const GEN_FLT x39 = x7 * x7;
	const GEN_FLT x40 = x5 + (x6 * x39);
	const GEN_FLT x41 = x26 + x24;
	const GEN_FLT x42 = x36 + (-1 * x35);
	const GEN_FLT x43 = x17 * x17;
	const GEN_FLT x44 = x14 + (x43 * x15);
	const GEN_FLT x45 = (x44 * sensor_x) + (x41 * sensor_z) + (x42 * sensor_y) + obj_px;
	const GEN_FLT x46 = (x31 * x38) + (x40 * x45) + (x28 * x10) + lh_px;
	const GEN_FLT x47 = 1. / x46;
	const GEN_FLT x48 = x9 + (-1 * x3);
	const GEN_FLT x49 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x50 = x15 * x17;
	const GEN_FLT x51 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x52 = x51 * x18;
	const GEN_FLT x53 = -1 * x52;
	const GEN_FLT x54 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x55 = x54 * x18;
	const GEN_FLT x56 = x51 * x14;
	const GEN_FLT x57 = x56 * x11;
	const GEN_FLT x58 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x59 = x51 * x19;
	const GEN_FLT x60 = (x59 * x20) + (x49 * x21) + (x50 * x58);
	const GEN_FLT x61 = x58 * x18;
	const GEN_FLT x62 = x56 * x20;
	const GEN_FLT x63 = (x50 * x54) + (x59 * x11) + (x49 * x25);
	const GEN_FLT x64 = ((x63 + x62 + x61) * sensor_z) + ((x60 + (-1 * x57) + (-1 * x55)) * sensor_y) +
						(((x52 * x43) + x53 + (2 * x50 * x49)) * sensor_x);
	const GEN_FLT x65 = x64 + x44;
	const GEN_FLT x66 = x1 * x7;
	const GEN_FLT x67 = x2 * x6;
	const GEN_FLT x68 = x4 * x67;
	const GEN_FLT x69 = x68 + x66;
	const GEN_FLT x70 = x49 * x18;
	const GEN_FLT x71 = x56 * x17;
	const GEN_FLT x72 = (x51 * x24 * x11) + (x54 * x21) + (x58 * x25);
	const GEN_FLT x73 = (((x52 * x33) + (2 * x58 * x21) + x53) * sensor_y) +
						((x72 + (-1 * x71) + (-1 * x70)) * sensor_z) + ((x60 + x57 + x55) * sensor_x);
	const GEN_FLT x74 = x73 + x37;
	const GEN_FLT x75 = x4 * x4;
	const GEN_FLT x76 = x5 + (x6 * x75);
	const GEN_FLT x77 = (((x52 * x12) + (2 * x54 * x25) + x53) * sensor_z) + ((x72 + x71 + x70) * sensor_y) +
						((x63 + (-1 * x62) + (-1 * x61)) * sensor_x);
	const GEN_FLT x78 = x77 + x27;
	const GEN_FLT x79 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x80 = x5 * x79;
	const GEN_FLT x81 = x2 * x80;
	const GEN_FLT x82 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x83 = x1 * x82;
	const GEN_FLT x84 = x1 * x79;
	const GEN_FLT x85 = x7 * x84;
	const GEN_FLT x86 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x87 = x4 * x6;
	const GEN_FLT x88 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x89 = (x8 * x88) + (x86 * x87) + (x4 * x85);
	const GEN_FLT x90 = x7 * x80;
	const GEN_FLT x91 = x1 * x86;
	const GEN_FLT x92 = (x82 * x87) + (x88 * x67) + (x2 * x4 * x84);
	const GEN_FLT x93 = -1 * x84;
	const GEN_FLT x94 = (x28 * ((2 * x88 * x87) + (x84 * x75) + x93)) + (x38 * (x92 + x91 + x90)) +
						(x45 * (x89 + (-1 * x83) + (-1 * x81)));
	const GEN_FLT x95 = x94 + (x78 * x76) + (x74 * x69) + (x65 * x48);
	const GEN_FLT x96 = x4 * x80;
	const GEN_FLT x97 = x1 * x88;
	const GEN_FLT x98 = (x2 * x85) + (x8 * x82) + (x86 * x67);
	const GEN_FLT x99 = (x28 * (x89 + x83 + x81)) + (x38 * (x98 + (-1 * x97) + (-1 * x96))) +
						(x45 * ((2 * x8 * x86) + (x84 * x39) + x93));
	const GEN_FLT x100 = x99 + (x78 * x10) + (x74 * x31) + (x65 * x40);
	const GEN_FLT x101 = x46 * x46;
	const GEN_FLT x102 = (x45 * x48) + (x69 * x38) + (x76 * x28) + lh_pz;
	const GEN_FLT x103 = (1. / x101) * x102;
	const GEN_FLT x104 = x101 + (x102 * x102);
	const GEN_FLT x105 = 1. / x104;
	const GEN_FLT x106 = x101 * x105;
	const GEN_FLT x107 = x106 * ((x100 * x103) + (-1 * x95 * x47));
	const GEN_FLT x108 = x68 + (-1 * x66);
	const GEN_FLT x109 = x2 * x2;
	const GEN_FLT x110 = x5 + (x6 * x109);
	const GEN_FLT x111 = x30 + x29;
	const GEN_FLT x112 = (x45 * x111) + (x28 * x108) + (x38 * x110) + lh_py;
	const GEN_FLT x113 = x112 * x112;
	const GEN_FLT x114 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x115 = tan(x114);
	const GEN_FLT x116 = 1. / sqrt(1 + (-1 * x105 * x113 * (x115 * x115)));
	const GEN_FLT x117 = 2 * x46;
	const GEN_FLT x118 = 2 * x102;
	const GEN_FLT x119 = (x95 * x118) + (x100 * x117);
	const GEN_FLT x120 = 1.0 / 2.0 * x112;
	const GEN_FLT x121 = (1. / (x104 * sqrt(x104))) * x115 * x120;
	const GEN_FLT x122 = (x28 * (x92 + (-1 * x91) + (-1 * x90))) + (x38 * ((2 * x82 * x67) + (x84 * x109) + x93)) +
						 (x45 * (x98 + x97 + x96));
	const GEN_FLT x123 = x122 + (x78 * x108) + (x74 * x110) + (x65 * x111);
	const GEN_FLT x124 = (1. / sqrt(x104)) * x115;
	const GEN_FLT x125 = (-1 * x124 * x123) + (x119 * x121);
	const GEN_FLT x126 = (-1 * x116 * x125) + x107;
	const GEN_FLT x127 = -1 * x112 * x124;
	const GEN_FLT x128 = atan2(-1 * x102, x46);
	const GEN_FLT x129 = x128 + (-1 * asin(x127)) + ogeeMag_1;
	const GEN_FLT x130 = cos(x129) * ogeePhase_1;
	const GEN_FLT x131 = cos(x114);
	const GEN_FLT x132 = 1. / x131;
	const GEN_FLT x133 = x104 + x113;
	const GEN_FLT x134 = (1. / sqrt(x133)) * x132;
	const GEN_FLT x135 = asin(x112 * x134);
	const GEN_FLT x136 = 8.0108022e-06 * x135;
	const GEN_FLT x137 = -8.0108022e-06 + (-1 * x136);
	const GEN_FLT x138 = 0.0028679863 + (x137 * x135);
	const GEN_FLT x139 = 5.3685255e-06 + (x138 * x135);
	const GEN_FLT x140 = 0.0076069798 + (x135 * x139);
	const GEN_FLT x141 = x135 * x140;
	const GEN_FLT x142 = -8.0108022e-06 + (-1.60216044e-05 * x135);
	const GEN_FLT x143 = x138 + (x135 * x142);
	const GEN_FLT x144 = x139 + (x135 * x143);
	const GEN_FLT x145 = x140 + (x135 * x144);
	const GEN_FLT x146 = (x135 * x145) + x141;
	const GEN_FLT x147 = (sin(x129) * ogeePhase_1) + curve_1;
	const GEN_FLT x148 = sin(x114);
	const GEN_FLT x149 = x148 * x147;
	const GEN_FLT x150 = x131 + (x146 * x149);
	const GEN_FLT x151 = 1. / x150;
	const GEN_FLT x152 = x135 * x135;
	const GEN_FLT x153 = x140 * x152;
	const GEN_FLT x154 = x151 * x153;
	const GEN_FLT x155 = x130 * x154;
	const GEN_FLT x156 = 2 * x112;
	const GEN_FLT x157 = x120 * (1. / (x133 * sqrt(x133))) * x132;
	const GEN_FLT x158 = 1. / sqrt(1 + (-1 * x113 * (1. / x133) * (1. / (x131 * x131))));
	const GEN_FLT x159 = x158 * ((x123 * x134) + (-1 * x157 * (x119 + (x123 * x156))));
	const GEN_FLT x160 = x137 * x159;
	const GEN_FLT x161 = (x138 * x159) + (x135 * (x160 + (-1 * x136 * x159)));
	const GEN_FLT x162 = (x161 * x135) + (x139 * x159);
	const GEN_FLT x163 = x147 * x151;
	const GEN_FLT x164 = x163 * x152;
	const GEN_FLT x165 = 2 * x163 * x141;
	const GEN_FLT x166 = x146 * x148;
	const GEN_FLT x167 = x166 * x130;
	const GEN_FLT x168 = 2.40324066e-05 * x135;
	const GEN_FLT x169 = x147 * (1. / (x150 * x150)) * x153;
	const GEN_FLT x170 = x127 + (x164 * x140);
	const GEN_FLT x171 = 1. / sqrt(1 + (-1 * (x170 * x170)));
	const GEN_FLT x172 =
		(-1 * x171 *
		 (x125 +
		  (-1 * x169 *
		   ((x149 *
			 ((x162 * x135) +
			  (x135 * (x162 + (x135 * (x161 + (x143 * x159) + (x135 * ((-1 * x168 * x159) + (x142 * x159) + x160)))) +
					   (x144 * x159))) +
			  (x140 * x159) + (x145 * x159))) +
			(x126 * x167))) +
		  (x164 * x162) + (x165 * x159) + (x126 * x155))) +
		x107;
	const GEN_FLT x173 = cos(x128 + (-1 * asin(x170)) + gibPhase_1) * gibMag_1;
	const GEN_FLT x174 = x64 + x42;
	const GEN_FLT x175 = x73 + x34;
	const GEN_FLT x176 = x77 + x23;
	const GEN_FLT x177 = x94 + (x76 * x176) + (x69 * x175) + (x48 * x174);
	const GEN_FLT x178 = x99 + (x10 * x176) + (x31 * x175) + (x40 * x174);
	const GEN_FLT x179 = x106 * ((x103 * x178) + (-1 * x47 * x177));
	const GEN_FLT x180 = (x118 * x177) + (x117 * x178);
	const GEN_FLT x181 = x122 + (x108 * x176) + (x110 * x175) + (x111 * x174);
	const GEN_FLT x182 = (-1 * x124 * x181) + (x121 * x180);
	const GEN_FLT x183 = (-1 * x116 * x182) + x179;
	const GEN_FLT x184 = x158 * ((x181 * x134) + (-1 * x157 * (x180 + (x181 * x156))));
	const GEN_FLT x185 = x184 * x137;
	const GEN_FLT x186 = (x184 * x138) + (x135 * (x185 + (-1 * x184 * x136)));
	const GEN_FLT x187 = (x186 * x135) + (x184 * x139);
	const GEN_FLT x188 =
		(-1 * x171 *
		 ((x165 * x184) + x182 + (x164 * x187) +
		  (-1 * x169 *
		   ((x149 * ((x187 * x135) +
					 (x135 * ((x135 * ((x184 * x143) + x186 + (x135 * ((x184 * x142) + x185 + (-1 * x168 * x184))))) +
							  x187 + (x184 * x144))) +
					 (x184 * x140) + (x184 * x145))) +
			(x167 * x183))) +
		  (x183 * x155))) +
		x179;
	const GEN_FLT x189 = x64 + x41;
	const GEN_FLT x190 = x73 + x32;
	const GEN_FLT x191 = x77 + x16;
	const GEN_FLT x192 = x94 + (x76 * x191) + (x69 * x190) + (x48 * x189);
	const GEN_FLT x193 = x99 + (x10 * x191) + (x31 * x190) + (x40 * x189);
	const GEN_FLT x194 = x106 * ((x103 * x193) + (-1 * x47 * x192));
	const GEN_FLT x195 = (x118 * x192) + (x117 * x193);
	const GEN_FLT x196 = x122 + (x108 * x191) + (x111 * x189) + (x110 * x190);
	const GEN_FLT x197 = (-1 * x124 * x196) + (x121 * x195);
	const GEN_FLT x198 = x130 * ((-1 * x116 * x197) + x194);
	const GEN_FLT x199 = x158 * ((x196 * x134) + (-1 * x157 * (x195 + (x196 * x156))));
	const GEN_FLT x200 = x199 * x137;
	const GEN_FLT x201 = (x199 * x138) + (x135 * (x200 + (-1 * x199 * x136)));
	const GEN_FLT x202 = (x201 * x135) + (x199 * x139);
	const GEN_FLT x203 =
		(-1 * x171 *
		 (x197 +
		  (-1 * x169 *
		   ((x149 *
			 ((x199 * x140) + (x202 * x135) +
			  (x135 * (x202 + (x135 * (x201 + (x199 * x143) + (x135 * ((x199 * x142) + (-1 * x168 * x199) + x200)))) +
					   (x199 * x144))) +
			  (x199 * x145))) +
			(x166 * x198))) +
		  (x165 * x199) + (x202 * x164) + (x198 * x154))) +
		x194;
	out[0] = x172 + (x172 * x173);
	out[1] = x188 + (x173 * x188);
	out[2] = x203 + (x203 * x173);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x9 + x3;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x20 * x11;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x15 * x16;
	const GEN_FLT x24 = x23 * x11;
	const GEN_FLT x25 =
		((x24 + (-1 * x22)) * sensor_x) + ((x21 + x18) * sensor_y) + ((x14 + (x15 * x12)) * sensor_z) + obj_pz;
	const GEN_FLT x26 = x1 * x4;
	const GEN_FLT x27 = x2 * x8;
	const GEN_FLT x28 = x27 + (-1 * x26);
	const GEN_FLT x29 = x19 * x19;
	const GEN_FLT x30 = x11 * x17;
	const GEN_FLT x31 = x20 * x16;
	const GEN_FLT x32 =
		((x31 + x30) * sensor_x) + ((x14 + (x29 * x15)) * sensor_y) + ((x21 + (-1 * x18)) * sensor_z) + obj_py;
	const GEN_FLT x33 = x7 * x7;
	const GEN_FLT x34 = x5 + (x6 * x33);
	const GEN_FLT x35 = x16 * x16;
	const GEN_FLT x36 =
		((x14 + (x35 * x15)) * sensor_x) + ((x31 + (-1 * x30)) * sensor_y) + ((x24 + x22) * sensor_z) + obj_px;
	const GEN_FLT x37 = (x34 * x36) + (x32 * x28) + (x25 * x10) + lh_px;
	const GEN_FLT x38 = 1. / x37;
	const GEN_FLT x39 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x40 = x5 * x39;
	const GEN_FLT x41 = x2 * x40;
	const GEN_FLT x42 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x43 = x1 * x42;
	const GEN_FLT x44 = x1 * x39;
	const GEN_FLT x45 = x4 * x44;
	const GEN_FLT x46 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x47 = x4 * x6;
	const GEN_FLT x48 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x49 = (x8 * x48) + (x46 * x47) + (x7 * x45);
	const GEN_FLT x50 = x5 * x7;
	const GEN_FLT x51 = x50 * x39;
	const GEN_FLT x52 = x1 * x46;
	const GEN_FLT x53 = x2 * x6;
	const GEN_FLT x54 = (x42 * x47) + (x53 * x48) + (x2 * x45);
	const GEN_FLT x55 = -1 * x44;
	const GEN_FLT x56 = x4 * x4;
	const GEN_FLT x57 = 2 * x47;
	const GEN_FLT x58 = x9 + (-1 * x3);
	const GEN_FLT x59 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x60 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x61 = x60 * x17;
	const GEN_FLT x62 = -1 * x61;
	const GEN_FLT x63 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x64 = x63 * x17;
	const GEN_FLT x65 = x60 * x14;
	const GEN_FLT x66 = x65 * x11;
	const GEN_FLT x67 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x68 = x60 * x18;
	const GEN_FLT x69 = (x59 * x20) + (x68 * x19) + (x67 * x23);
	const GEN_FLT x70 = x67 * x17;
	const GEN_FLT x71 = x65 * x19;
	const GEN_FLT x72 = x15 * x11;
	const GEN_FLT x73 = (x63 * x23) + (x68 * x11) + (x72 * x59);
	const GEN_FLT x74 = ((x73 + x71 + x70) * sensor_z) + ((x69 + (-1 * x66) + (-1 * x64)) * sensor_y) +
						(((x61 * x35) + x62 + (2 * x59 * x23)) * sensor_x);
	const GEN_FLT x75 = x59 * x17;
	const GEN_FLT x76 = x65 * x16;
	const GEN_FLT x77 = (x61 * x11 * x19) + (x63 * x20) + (x72 * x67);
	const GEN_FLT x78 = ((x77 + (-1 * x76) + (-1 * x75)) * sensor_z) +
						(((x61 * x29) + (2 * x67 * x20) + x62) * sensor_y) + ((x69 + x66 + x64) * sensor_x);
	const GEN_FLT x79 = x1 * x7;
	const GEN_FLT x80 = x2 * x47;
	const GEN_FLT x81 = x80 + x79;
	const GEN_FLT x82 = x5 + (x6 * x56);
	const GEN_FLT x83 = (((x61 * x12) + (2 * x72 * x63) + x62) * sensor_z) + ((x77 + x76 + x75) * sensor_y) +
						((x73 + (-1 * x71) + (-1 * x70)) * sensor_x);
	const GEN_FLT x84 = (x82 * x83) + (x81 * x78) + (x74 * x58);
	const GEN_FLT x85 = x84 + (x25 * ((x57 * x48) + (x56 * x44) + x55)) + (x32 * (x54 + x52 + x51)) +
						(x36 * (x49 + (-1 * x43) + (-1 * x41)));
	const GEN_FLT x86 = -1 * x85 * x38;
	const GEN_FLT x87 = 2 * x8;
	const GEN_FLT x88 = x4 * x40;
	const GEN_FLT x89 = x1 * x48;
	const GEN_FLT x90 = x2 * x7;
	const GEN_FLT x91 = (x90 * x44) + (x8 * x42) + (x53 * x46);
	const GEN_FLT x92 = (x83 * x10) + (x78 * x28) + (x74 * x34);
	const GEN_FLT x93 = (x25 * (x49 + x43 + x41)) + x92 + (x32 * (x91 + (-1 * x89) + (-1 * x88))) +
						(x36 * ((x87 * x46) + (x44 * x33) + x55));
	const GEN_FLT x94 = 1 + x93;
	const GEN_FLT x95 = x37 * x37;
	const GEN_FLT x96 = (x58 * x36) + (x81 * x32) + (x82 * x25) + lh_pz;
	const GEN_FLT x97 = x96 * (1. / x95);
	const GEN_FLT x98 = x95 + (x96 * x96);
	const GEN_FLT x99 = 1. / x98;
	const GEN_FLT x100 = x99 * x95;
	const GEN_FLT x101 = x100 * ((x97 * x94) + x86);
	const GEN_FLT x102 = x80 + (-1 * x79);
	const GEN_FLT x103 = x2 * x2;
	const GEN_FLT x104 = x5 + (x6 * x103);
	const GEN_FLT x105 = x27 + x26;
	const GEN_FLT x106 = (x36 * x105) + (x32 * x104) + (x25 * x102) + lh_py;
	const GEN_FLT x107 = x106 * x106;
	const GEN_FLT x108 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x109 = tan(x108);
	const GEN_FLT x110 = 1. / sqrt(1 + (-1 * x99 * (x109 * x109) * x107));
	const GEN_FLT x111 = 2 * x37;
	const GEN_FLT x112 = 2 * x96;
	const GEN_FLT x113 = x85 * x112;
	const GEN_FLT x114 = x113 + (x94 * x111);
	const GEN_FLT x115 = 1.0 / 2.0 * x106;
	const GEN_FLT x116 = (1. / (x98 * sqrt(x98))) * x109 * x115;
	const GEN_FLT x117 = 2 * x53;
	const GEN_FLT x118 = (x83 * x102) + (x78 * x104) + (x74 * x105);
	const GEN_FLT x119 = x118 + (x25 * (x54 + (-1 * x52) + (-1 * x51))) + (x32 * ((x42 * x117) + (x44 * x103) + x55)) +
						 (x36 * (x91 + x89 + x88));
	const GEN_FLT x120 = (1. / sqrt(x98)) * x109;
	const GEN_FLT x121 = -1 * x119 * x120;
	const GEN_FLT x122 = x121 + (x114 * x116);
	const GEN_FLT x123 = (-1 * x110 * x122) + x101;
	const GEN_FLT x124 = -1 * x106 * x120;
	const GEN_FLT x125 = atan2(-1 * x96, x37);
	const GEN_FLT x126 = x125 + (-1 * asin(x124)) + ogeeMag_1;
	const GEN_FLT x127 = cos(x126) * ogeePhase_1;
	const GEN_FLT x128 = cos(x108);
	const GEN_FLT x129 = 1. / x128;
	const GEN_FLT x130 = x98 + x107;
	const GEN_FLT x131 = x129 * (1. / sqrt(x130));
	const GEN_FLT x132 = asin(x106 * x131);
	const GEN_FLT x133 = 8.0108022e-06 * x132;
	const GEN_FLT x134 = -8.0108022e-06 + (-1 * x133);
	const GEN_FLT x135 = 0.0028679863 + (x134 * x132);
	const GEN_FLT x136 = 5.3685255e-06 + (x132 * x135);
	const GEN_FLT x137 = 0.0076069798 + (x132 * x136);
	const GEN_FLT x138 = x132 * x132;
	const GEN_FLT x139 = x132 * x137;
	const GEN_FLT x140 = -8.0108022e-06 + (-1.60216044e-05 * x132);
	const GEN_FLT x141 = x135 + (x132 * x140);
	const GEN_FLT x142 = x136 + (x132 * x141);
	const GEN_FLT x143 = x137 + (x132 * x142);
	const GEN_FLT x144 = (x132 * x143) + x139;
	const GEN_FLT x145 = (sin(x126) * ogeePhase_1) + curve_1;
	const GEN_FLT x146 = sin(x108);
	const GEN_FLT x147 = x146 * x145;
	const GEN_FLT x148 = x128 + (x144 * x147);
	const GEN_FLT x149 = 1. / x148;
	const GEN_FLT x150 = x137 * x138 * x149;
	const GEN_FLT x151 = x127 * x150;
	const GEN_FLT x152 = 2 * x106;
	const GEN_FLT x153 = x119 * x152;
	const GEN_FLT x154 = x115 * x129 * (1. / (x130 * sqrt(x130)));
	const GEN_FLT x155 = x119 * x131;
	const GEN_FLT x156 = x155 + (-1 * (x114 + x153) * x154);
	const GEN_FLT x157 = 1. / sqrt(1 + (-1 * x107 * (1. / (x128 * x128)) * (1. / x130)));
	const GEN_FLT x158 = x136 * x157;
	const GEN_FLT x159 = x133 * x157;
	const GEN_FLT x160 = x134 * x157;
	const GEN_FLT x161 = x160 * x156;
	const GEN_FLT x162 = x135 * x157;
	const GEN_FLT x163 = (x162 * x156) + (x132 * (x161 + (-1 * x156 * x159)));
	const GEN_FLT x164 = (x163 * x132) + (x156 * x158);
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
	const GEN_FLT x178 = x165 * x137 * (1. / (x148 * x148));
	const GEN_FLT x179 = x124 + (x166 * x137);
	const GEN_FLT x180 = 1. / sqrt(1 + (-1 * (x179 * x179)));
	const GEN_FLT x181 =
		(-1 * x180 *
		 (x122 +
		  (-1 * x178 *
		   ((x147 *
			 ((x177 * x156) + (x176 * x156) + (x164 * x132) +
			  (x132 * (x164 + (x132 * (x163 + (x175 * x156) + (x132 * ((x174 * x156) + (-1 * x173 * x156) + x161)))) +
					   (x171 * x156))))) +
			(x123 * x170))) +
		  (x168 * x156) + (x166 * x164) + (x123 * x151))) +
		x101;
	const GEN_FLT x182 = cos(x125 + (-1 * asin(x179)) + gibPhase_1) * gibMag_1;
	const GEN_FLT x183 = x93 * x97;
	const GEN_FLT x184 = x100 * (x183 + x86);
	const GEN_FLT x185 = x93 * x111;
	const GEN_FLT x186 = x113 + x185;
	const GEN_FLT x187 = 1 + x119;
	const GEN_FLT x188 = (-1 * x120 * x187) + (x116 * x186);
	const GEN_FLT x189 = (-1 * x110 * x188) + x184;
	const GEN_FLT x190 = (x187 * x131) + (-1 * x154 * (x186 + (x187 * x152)));
	const GEN_FLT x191 = x190 * x157;
	const GEN_FLT x192 = x191 * x134;
	const GEN_FLT x193 = (x191 * x135) + (x132 * (x192 + (-1 * x191 * x133)));
	const GEN_FLT x194 = (x193 * x132) + (x191 * x136);
	const GEN_FLT x195 =
		(-1 * x180 *
		 (x188 +
		  (-1 * x178 *
		   ((x147 *
			 ((x177 * x190) + (x194 * x132) +
			  (x132 * (x194 + (x132 * (x193 + (x191 * x141) + (x132 * ((x191 * x140) + (-1 * x172 * x191) + x192)))) +
					   (x191 * x142))) +
			  (x176 * x190))) +
			(x170 * x189))) +
		  (x167 * x191) + (x166 * x194) + (x189 * x151))) +
		x184;
	const GEN_FLT x196 = 1 + x85;
	const GEN_FLT x197 = x100 * (x183 + (-1 * x38 * x196));
	const GEN_FLT x198 = (x112 * x196) + x185;
	const GEN_FLT x199 = x121 + (x116 * x198);
	const GEN_FLT x200 = (-1 * x110 * x199) + x197;
	const GEN_FLT x201 = x155 + (-1 * (x198 + x153) * x154);
	const GEN_FLT x202 = x201 * x157;
	const GEN_FLT x203 = x202 * x134;
	const GEN_FLT x204 = (x201 * x162) + (x132 * (x203 + (-1 * x202 * x133)));
	const GEN_FLT x205 = (x204 * x132) + (x202 * x136);
	const GEN_FLT x206 =
		(-1 * x180 *
		 (x199 + (x202 * x167) +
		  (-1 * x178 *
		   ((x147 *
			 ((x201 * x177) + (x205 * x132) +
			  (x132 * (x205 + (x132 * (x204 + (x202 * x141) + (x132 * ((x202 * x140) + (-1 * x202 * x172) + x203)))) +
					   (x201 * x171))) +
			  (x201 * x176))) +
			(x200 * x170))) +
		  (x205 * x166) + (x200 * x151))) +
		x197;
	const GEN_FLT x207 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x208 = x5 * x207;
	const GEN_FLT x209 = x2 * x208;
	const GEN_FLT x210 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qj *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x211 = x1 * x210;
	const GEN_FLT x212 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qi *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x213 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qk *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x214 = x1 * x207;
	const GEN_FLT x215 = x7 * x214;
	const GEN_FLT x216 = (x4 * x215) + (x8 * x213) + (x47 * x212);
	const GEN_FLT x217 = x50 * x207;
	const GEN_FLT x218 = x1 * x212;
	const GEN_FLT x219 = x2 * x4;
	const GEN_FLT x220 = (x47 * x210) + (x53 * x213) + (x219 * x214);
	const GEN_FLT x221 = -1 * x214;
	const GEN_FLT x222 = x84 + (x25 * ((x57 * x213) + (x56 * x214) + x221)) + (x32 * (x220 + x218 + x217)) +
						 (x36 * (x216 + (-1 * x211) + (-1 * x209)));
	const GEN_FLT x223 = x4 * x208;
	const GEN_FLT x224 = x1 * x213;
	const GEN_FLT x225 = (x8 * x210) + (x53 * x212) + (x2 * x215);
	const GEN_FLT x226 = x92 + (x25 * (x216 + x211 + x209)) + (x32 * ((-1 * x224) + x225 + (-1 * x223))) +
						 (x36 * ((x33 * x214) + (x87 * x212) + x221));
	const GEN_FLT x227 = ((x97 * x226) + (-1 * x38 * x222)) * x100;
	const GEN_FLT x228 = (x222 * x112) + (x226 * x111);
	const GEN_FLT x229 = x118 + (x36 * (x224 + x225 + x223)) + (x25 * (x220 + (-1 * x218) + (-1 * x217))) +
						 (x32 * ((x210 * x117) + (x214 * x103) + x221));
	const GEN_FLT x230 = (-1 * x229 * x120) + (x228 * x116);
	const GEN_FLT x231 = x127 * ((-1 * x230 * x110) + x227);
	const GEN_FLT x232 = (x229 * x131) + (-1 * x154 * (x228 + (x229 * x152)));
	const GEN_FLT x233 = x232 * x157;
	const GEN_FLT x234 = x233 * x134;
	const GEN_FLT x235 = (x232 * x162) + (x132 * (x234 + (-1 * x233 * x133)));
	const GEN_FLT x236 = (x235 * x132) + (x233 * x136);
	const GEN_FLT x237 =
		(-1 * x180 *
		 (x230 +
		  (-1 * x178 *
		   ((x147 *
			 ((x232 * x177) + (x236 * x132) +
			  (x132 * (x236 + (x132 * (x235 + (x233 * x141) + (x132 * ((-1 * x233 * x172) + (x233 * x140) + x234)))) +
					   (x232 * x171))) +
			  (x232 * x176))) +
			(x231 * x169))) +
		  (x233 * x167) + (x236 * x166) + (x231 * x150))) +
		x227;
	const GEN_FLT x238 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x239 = x5 * x238;
	const GEN_FLT x240 = x2 * x239;
	const GEN_FLT x241 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qj *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x242 = x1 * x241;
	const GEN_FLT x243 = x26 * x238;
	const GEN_FLT x244 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qi *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x245 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qk *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x246 = (x8 * x245) + (x47 * x244) + (x7 * x243);
	const GEN_FLT x247 = x50 * x238;
	const GEN_FLT x248 = x1 * x244;
	const GEN_FLT x249 = (x53 * x245) + (x47 * x241) + (x2 * x243);
	const GEN_FLT x250 = x1 * x238;
	const GEN_FLT x251 = -1 * x250;
	const GEN_FLT x252 = x84 + (x25 * ((x57 * x245) + (x56 * x250) + x251)) + (x32 * (x249 + x248 + x247)) +
						 (x36 * ((-1 * x242) + x246 + (-1 * x240)));
	const GEN_FLT x253 = x4 * x239;
	const GEN_FLT x254 = x1 * x245;
	const GEN_FLT x255 = (x53 * x244) + (x8 * x241) + (x2 * x79 * x238);
	const GEN_FLT x256 = x92 + (x25 * (x242 + x246 + x240)) + (x32 * (x255 + (-1 * x254) + (-1 * x253))) +
						 (x36 * ((x87 * x244) + x251 + (x33 * x250)));
	const GEN_FLT x257 = ((x97 * x256) + (-1 * x38 * x252)) * x100;
	const GEN_FLT x258 = (x252 * x112) + (x256 * x111);
	const GEN_FLT x259 = x118 + (x32 * ((x241 * x117) + (x250 * x103) + x251)) +
						 (x25 * (x249 + (-1 * x248) + (-1 * x247))) + (x36 * (x255 + x254 + x253));
	const GEN_FLT x260 = (-1 * x259 * x120) + (x258 * x116);
	const GEN_FLT x261 = (-1 * x260 * x110) + x257;
	const GEN_FLT x262 = (x259 * x131) + (-1 * x154 * (x258 + (x259 * x152)));
	const GEN_FLT x263 = x262 * x160;
	const GEN_FLT x264 = (x262 * x162) + (x132 * (x263 + (-1 * x262 * x159)));
	const GEN_FLT x265 = (x264 * x132) + (x262 * x158);
	const GEN_FLT x266 =
		(-1 * x180 *
		 (x260 +
		  (-1 * x178 *
		   ((x147 * ((x262 * x176) + (x262 * x177) + (x265 * x132) +
					 (x132 * ((x132 * (x264 + (x262 * x175) + (x132 * ((x262 * x174) + (-1 * x262 * x173) + x263)))) +
							  x265 + (x262 * x171))))) +
			(x261 * x170))) +
		  (x262 * x168) + (x265 * x166) + (x261 * x151))) +
		x257;
	const GEN_FLT x267 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x268 = x5 * x267;
	const GEN_FLT x269 = x2 * x268;
	const GEN_FLT x270 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qj *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x271 = x1 * x270;
	const GEN_FLT x272 = x1 * x267;
	const GEN_FLT x273 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qi *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x274 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qk *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x275 = (x8 * x274) + (x47 * x273) + (x4 * x7 * x272);
	const GEN_FLT x276 = x50 * x267;
	const GEN_FLT x277 = x1 * x273;
	const GEN_FLT x278 = (x47 * x270) + (x53 * x274) + (x219 * x272);
	const GEN_FLT x279 = -1 * x272;
	const GEN_FLT x280 = x84 + (x25 * ((x57 * x274) + (x56 * x272) + x279)) + (x32 * (x278 + x277 + x276)) +
						 (x36 * (x275 + (-1 * x271) + (-1 * x269)));
	const GEN_FLT x281 = x4 * x268;
	const GEN_FLT x282 = x1 * x274;
	const GEN_FLT x283 = (x53 * x273) + (x8 * x270) + (x90 * x272);
	const GEN_FLT x284 = x92 + (x25 * (x275 + x271 + x269)) + (x32 * ((-1 * x282) + x283 + (-1 * x281))) +
						 (x36 * ((x87 * x273) + (x33 * x272) + x279));
	const GEN_FLT x285 = ((x97 * x284) + (-1 * x38 * x280)) * x100;
	const GEN_FLT x286 = (x280 * x112) + (x284 * x111);
	const GEN_FLT x287 = x118 + (x25 * (x278 + (-1 * x277) + (-1 * x276))) +
						 (x32 * ((x270 * x117) + (x272 * x103) + x279)) + (x36 * (x282 + x283 + x281));
	const GEN_FLT x288 = (-1 * x287 * x120) + (x286 * x116);
	const GEN_FLT x289 = (-1 * x288 * x110) + x285;
	const GEN_FLT x290 = (x287 * x131) + (-1 * x154 * (x286 + (x287 * x152)));
	const GEN_FLT x291 = x290 * x157;
	const GEN_FLT x292 = x291 * x134;
	const GEN_FLT x293 = (x291 * x135) + (x132 * (x292 + (-1 * x291 * x133)));
	const GEN_FLT x294 = (x293 * x132) + (x291 * x136);
	const GEN_FLT x295 =
		(-1 * x180 *
		 (x288 +
		  (-1 * x178 *
		   ((x147 *
			 ((x291 * x137) + (x294 * x132) +
			  (x132 * (x294 + (x132 * (x293 + (x291 * x141) + (x132 * ((x291 * x140) + (-1 * x291 * x172) + x292)))) +
					   (x291 * x142))) +
			  (x290 * x176))) +
			(x289 * x170))) +
		  (x291 * x167) + (x294 * x166) + (x289 * x151))) +
		x285;
	out[0] = x181 + (x181 * x182);
	out[1] = x195 + (x182 * x195);
	out[2] = x206 + (x206 * x182);
	out[3] = x237 + (x237 * x182);
	out[4] = x266 + (x266 * x182);
	out[5] = x295 + (x295 * x182);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x9 + x3;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x20 = x15 * x11;
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x20 * x16;
	const GEN_FLT x24 =
		((x23 + (-1 * x22)) * sensor_x) + ((x21 + x18) * sensor_y) + ((x14 + (x15 * x12)) * sensor_z) + obj_pz;
	const GEN_FLT x25 = x1 * x7;
	const GEN_FLT x26 = x4 * x6;
	const GEN_FLT x27 = x2 * x26;
	const GEN_FLT x28 = x27 + (-1 * x25);
	const GEN_FLT x29 = x19 * x19;
	const GEN_FLT x30 = x11 * x17;
	const GEN_FLT x31 = x15 * x19;
	const GEN_FLT x32 = x31 * x16;
	const GEN_FLT x33 =
		((x32 + x30) * sensor_x) + ((x14 + (x29 * x15)) * sensor_y) + ((x21 + (-1 * x18)) * sensor_z) + obj_py;
	const GEN_FLT x34 = x4 * x4;
	const GEN_FLT x35 = x5 + (x6 * x34);
	const GEN_FLT x36 = x16 * x16;
	const GEN_FLT x37 =
		((x32 + (-1 * x30)) * sensor_y) + ((x23 + x22) * sensor_z) + ((x14 + (x36 * x15)) * sensor_x) + obj_px;
	const GEN_FLT x38 = (x33 * x28) + (x35 * x37) + (x24 * x10) + lh_px;
	const GEN_FLT x39 = x38 * x38;
	const GEN_FLT x40 = x7 * x7;
	const GEN_FLT x41 = x5 + (x6 * x40);
	const GEN_FLT x42 = x1 * x4;
	const GEN_FLT x43 = x2 * x8;
	const GEN_FLT x44 = x43 + x42;
	const GEN_FLT x45 = x9 + (-1 * x3);
	const GEN_FLT x46 = (x45 * x37) + (x44 * x33) + (x41 * x24) + lh_pz;
	const GEN_FLT x47 = x39 + (x46 * x46);
	const GEN_FLT x48 = 1. / x47;
	const GEN_FLT x49 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x50 = x5 * x49;
	const GEN_FLT x51 = x2 * x50;
	const GEN_FLT x52 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x53 = x1 * x52;
	const GEN_FLT x54 = x1 * x49;
	const GEN_FLT x55 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x56 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x57 = (x56 * x26) + (x8 * x55) + (x4 * x7 * x54);
	const GEN_FLT x58 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x59 = x15 * x16;
	const GEN_FLT x60 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x61 = x60 * x17;
	const GEN_FLT x62 = -1 * x61;
	const GEN_FLT x63 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x64 = x63 * x17;
	const GEN_FLT x65 = x60 * x14;
	const GEN_FLT x66 = x65 * x11;
	const GEN_FLT x67 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x68 = x60 * x19;
	const GEN_FLT x69 = (x58 * x31) + (x68 * x18) + (x67 * x59);
	const GEN_FLT x70 = x67 * x17;
	const GEN_FLT x71 = x65 * x19;
	const GEN_FLT x72 = (x63 * x59) + (x60 * x11 * x18) + (x58 * x20);
	const GEN_FLT x73 = ((x69 + (-1 * x66) + (-1 * x64)) * sensor_y) + ((x72 + x71 + x70) * sensor_z) +
						(((x61 * x36) + x62 + (2 * x58 * x59)) * sensor_x);
	const GEN_FLT x74 = x4 * x50;
	const GEN_FLT x75 = x1 * x55;
	const GEN_FLT x76 = x3 * x49;
	const GEN_FLT x77 = x2 * x6;
	const GEN_FLT x78 = (x8 * x52) + (x77 * x56) + (x7 * x76);
	const GEN_FLT x79 = x58 * x17;
	const GEN_FLT x80 = x65 * x16;
	const GEN_FLT x81 = (x63 * x31) + (x68 * x30) + (x67 * x20);
	const GEN_FLT x82 = ((x81 + (-1 * x80) + (-1 * x79)) * sensor_z) +
						(((x61 * x29) + (2 * x67 * x31) + x62) * sensor_y) + ((x69 + x66 + x64) * sensor_x);
	const GEN_FLT x83 = -1 * x54;
	const GEN_FLT x84 = (((x61 * x12) + (2 * x63 * x20) + x62) * sensor_z) + ((x81 + x80 + x79) * sensor_y) +
						((x72 + (-1 * x71) + (-1 * x70)) * sensor_x);
	const GEN_FLT x85 = (x24 * ((2 * x8 * x56) + (x54 * x40) + x83)) + (x82 * x44) + (x33 * (x78 + x75 + x74)) +
						(x84 * x41) + (x73 * x45) + (x37 * (x57 + (-1 * x53) + (-1 * x51)));
	const GEN_FLT x86 = x7 * x50;
	const GEN_FLT x87 = x1 * x56;
	const GEN_FLT x88 = (x4 * x76) + (x52 * x26) + (x77 * x55);
	const GEN_FLT x89 = (x84 * x10) + (x24 * (x57 + x53 + x51)) + (x82 * x28) +
						(x33 * (x88 + (-1 * x87) + (-1 * x86))) + (x73 * x35) +
						(x37 * ((2 * x55 * x26) + (x54 * x34) + x83));
	const GEN_FLT x90 = x48 * x39 * ((x89 * x46 * (1. / x39)) + (-1 * x85 * (1. / x38)));
	const GEN_FLT x91 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x92 = tan(x91);
	const GEN_FLT x93 = x43 + (-1 * x42);
	const GEN_FLT x94 = x2 * x2;
	const GEN_FLT x95 = x5 + (x6 * x94);
	const GEN_FLT x96 = x27 + x25;
	const GEN_FLT x97 = (x96 * x37) + (x95 * x33) + (x93 * x24) + lh_py;
	const GEN_FLT x98 = 1. / sqrt(x47);
	const GEN_FLT x99 = x98 * x97;
	const GEN_FLT x100 = -1 * x92 * x99;
	const GEN_FLT x101 = atan2(-1 * x46, x38);
	const GEN_FLT x102 = x101 + (-1 * asin(x100)) + ogeeMag_1;
	const GEN_FLT x103 = sin(x102);
	const GEN_FLT x104 = (x103 * ogeePhase_1) + curve_1;
	const GEN_FLT x105 = sin(x91);
	const GEN_FLT x106 = cos(x91);
	const GEN_FLT x107 = 1. / x106;
	const GEN_FLT x108 = x97 * x97;
	const GEN_FLT x109 = x47 + x108;
	const GEN_FLT x110 = 1. / sqrt(x109);
	const GEN_FLT x111 = x107 * x110;
	const GEN_FLT x112 = asin(x97 * x111);
	const GEN_FLT x113 = 8.0108022e-06 * x112;
	const GEN_FLT x114 = -8.0108022e-06 + (-1 * x113);
	const GEN_FLT x115 = 0.0028679863 + (x112 * x114);
	const GEN_FLT x116 = 5.3685255e-06 + (x112 * x115);
	const GEN_FLT x117 = 0.0076069798 + (x112 * x116);
	const GEN_FLT x118 = x112 * x117;
	const GEN_FLT x119 = -8.0108022e-06 + (-1.60216044e-05 * x112);
	const GEN_FLT x120 = x115 + (x112 * x119);
	const GEN_FLT x121 = x116 + (x112 * x120);
	const GEN_FLT x122 = x117 + (x112 * x121);
	const GEN_FLT x123 = (x112 * x122) + x118;
	const GEN_FLT x124 = x105 * x123;
	const GEN_FLT x125 = x106 + (x104 * x124);
	const GEN_FLT x126 = 1. / x125;
	const GEN_FLT x127 = x104 * x126;
	const GEN_FLT x128 = x112 * x112;
	const GEN_FLT x129 = x117 * x128;
	const GEN_FLT x130 = x100 + (x127 * x129);
	const GEN_FLT x131 = 1. / sqrt(1 + (-1 * (x130 * x130)));
	const GEN_FLT x132 = x92 * x92;
	const GEN_FLT x133 = 1. / sqrt(1 + (-1 * x48 * x108 * x132));
	const GEN_FLT x134 = (2 * x85 * x46) + (2 * x89 * x38);
	const GEN_FLT x135 = 1.0 / 2.0 * x97;
	const GEN_FLT x136 = (x84 * x93) + (x24 * (x78 + (-1 * x75) + (-1 * x74))) + (x82 * x95) + (x73 * x96) +
						 (x33 * ((2 * x77 * x52) + (x54 * x94) + x83)) + (x37 * (x88 + x87 + x86));
	const GEN_FLT x137 = (-1 * x92 * x98 * x136) + (x92 * (1. / (x47 * sqrt(x47))) * x134 * x135);
	const GEN_FLT x138 = (-1 * x133 * x137) + x90;
	const GEN_FLT x139 = cos(x102) * ogeePhase_1;
	const GEN_FLT x140 = x138 * x139;
	const GEN_FLT x141 = x126 * x129;
	const GEN_FLT x142 = (x111 * x136) + (-1 * (1. / (x109 * sqrt(x109))) * x107 * x135 * (x134 + (2 * x97 * x136)));
	const GEN_FLT x143 = 1. / (x106 * x106);
	const GEN_FLT x144 = 1. / sqrt(1 + (-1 * (1. / x109) * x108 * x143));
	const GEN_FLT x145 = x142 * x144;
	const GEN_FLT x146 = x114 * x145;
	const GEN_FLT x147 = 2.40324066e-05 * x112;
	const GEN_FLT x148 = (x115 * x145) + (x112 * (x146 + (-1 * x113 * x145)));
	const GEN_FLT x149 = (x112 * x148) + (x116 * x145);
	const GEN_FLT x150 = x105 * x104;
	const GEN_FLT x151 =
		x150 * ((x117 * x145) + (x112 * x149) +
				(x112 * (x149 + (x112 * (x148 + (x120 * x145) + (x112 * ((x119 * x145) + (-1 * x145 * x147) + x146)))) +
						 (x121 * x145))) +
				(x122 * x145));
	const GEN_FLT x152 = x104 * (1. / (x125 * x125)) * x129;
	const GEN_FLT x153 = x128 * x127;
	const GEN_FLT x154 = 2 * x118 * x127;
	const GEN_FLT x155 = x137 + (x145 * x154) + (x149 * x153);
	const GEN_FLT x156 = (-1 * x131 * (x155 + (-1 * x152 * (x151 + (x124 * x140))) + (x140 * x141))) + x90;
	const GEN_FLT x157 = (-1 * asin(x130)) + x101 + gibPhase_1;
	const GEN_FLT x158 = cos(x157) * gibMag_1;
	const GEN_FLT x159 = x156 + (x156 * x158);
	const GEN_FLT x160 = x137 + (x99 * (1 + x132));
	const GEN_FLT x161 = (-1 * x160 * x133) + x90;
	const GEN_FLT x162 = x139 * x141;
	const GEN_FLT x163 = x144 * (x142 + (-1 * x97 * x105 * x110 * x143));
	const GEN_FLT x164 = x114 * x163;
	const GEN_FLT x165 = (x115 * x163) + (x112 * (x164 + (-1 * x113 * x163)));
	const GEN_FLT x166 = (x112 * x165) + (x116 * x163);
	const GEN_FLT x167 = x124 * x139;
	const GEN_FLT x168 =
		(-1 * x131 *
		 (x160 + (x163 * x154) + (x166 * x153) +
		  (-1 * x152 *
		   ((x150 *
			 ((x117 * x163) + (x112 * x166) +
			  (x112 * (x166 + (x112 * (x165 + (x120 * x163) + (x112 * ((x119 * x163) + (-1 * x163 * x147) + x164)))) +
					   (x121 * x163))) +
			  (x122 * x163))) +
			(-1 * x104 * x106 * x123) + x105 + (x161 * x167))) +
		  (x161 * x162))) +
		x90;
	const GEN_FLT x169 = 1 + x140;
	const GEN_FLT x170 = (-1 * x131 * (x155 + (-1 * x152 * (x151 + (x124 * x169))) + (x169 * x141))) + x90;
	const GEN_FLT x171 = 1 + x138;
	const GEN_FLT x172 = (-1 * x131 * (x155 + (-1 * x152 * (x151 + (x167 * x171))) + (x162 * x171))) + x90;
	const GEN_FLT x173 = x103 + x140;
	const GEN_FLT x174 = (-1 * x131 * (x155 + (-1 * x152 * (x151 + (x124 * x173))) + (x173 * x141))) + x90;
	out[0] = -1 + x159;
	out[1] = x168 + (x168 * x158);
	out[2] = x170 + (x170 * x158);
	out[3] = x156 + (x158 * (1 + x156));
	out[4] = x159 + sin(x157);
	out[5] = x172 + (x172 * x158);
	out[6] = x174 + (x174 * x158);
}

/** Applying function <function reproject at 0x7f6253d38200> */
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x11 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x12 = cos(x11);
	const GEN_FLT x13 = 1 + (-1 * x12);
	const GEN_FLT x14 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x15 = sin(x11);
	const GEN_FLT x16 = x15 * x14;
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x18 = x13 * x10;
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = x15 * x17;
	const GEN_FLT x21 = x14 * x18;
	const GEN_FLT x22 =
		((x21 + (-1 * x20)) * sensor_x) + ((x19 + x16) * sensor_y) + ((x12 + (x13 * (x10 * x10))) * sensor_z) + obj_pz;
	const GEN_FLT x23 = x15 * x10;
	const GEN_FLT x24 = x14 * x13 * x17;
	const GEN_FLT x25 =
		((x24 + x23) * sensor_x) + ((x12 + (x13 * (x17 * x17))) * sensor_y) + ((x19 + (-1 * x16)) * sensor_z) + obj_py;
	const GEN_FLT x26 = x1 * x4;
	const GEN_FLT x27 = x2 * x8;
	const GEN_FLT x28 =
		((x12 + ((x14 * x14) * x13)) * sensor_x) + ((x24 + (-1 * x23)) * sensor_y) + ((x21 + x20) * sensor_z) + obj_px;
	const GEN_FLT x29 = ((x27 + x26) * x28) + ((x9 + (-1 * x3)) * x22) + (x25 * (x5 + (x6 * (x7 * x7)))) + lh_py;
	const GEN_FLT x30 = x1 * x7;
	const GEN_FLT x31 = x2 * x4 * x6;
	const GEN_FLT x32 = ((x31 + (-1 * x30)) * x28) + ((x9 + x3) * x25) + (x22 * (x5 + ((x4 * x4) * x6))) + lh_pz;
	const GEN_FLT x33 = -1 * x32;
	const GEN_FLT x34 = x32 * x32;
	const GEN_FLT x35 = (x28 * (x5 + ((x2 * x2) * x6))) + ((x27 + (-1 * x26)) * x25) + ((x31 + x30) * x22) + lh_px;
	const GEN_FLT x36 = atan2(x35, x33);
	const GEN_FLT x37 = (-1 * x36) + (-1 * phase_0) + (-1 * asin(x29 * (1. / sqrt((x35 * x35) + x34)) * tilt_0));
	const GEN_FLT x38 =
		(-1 * atan2(-1 * x29, x33)) + (-1 * phase_1) + (-1 * asin(x35 * (1. / sqrt((x29 * x29) + x34)) * tilt_1));
	out[0] = x37 + (-1 * cos(1.5707963267949 + x37 + gibPhase_0) * gibMag_0) +
			 ((atan2(x29, x33) * atan2(x29, x33)) * curve_0);
	out[1] = x38 + ((x36 * x36) * curve_1) + (-1 * cos(1.5707963267949 + x38 + gibPhase_1) * gibMag_1);
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
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (-1 * obj_qk *
							  (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										  : 1e-10) *
									 ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										  : 1e-10))) *
							  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
						   : 0;
	const GEN_FLT x1 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						   : 1e-10;
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x5 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 0;
	const GEN_FLT x6 = cos(x1);
	const GEN_FLT x7 = x6 * x5;
	const GEN_FLT x8 = x4 * x7;
	const GEN_FLT x9 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (-1 * obj_qj *
							  (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										  : 1e-10) *
									 ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										  : 1e-10))) *
							  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
						   : 0;
	const GEN_FLT x10 = 1 + (-1 * x6);
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x12 = x11 * x10;
	const GEN_FLT x13 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x14 = x2 * x11;
	const GEN_FLT x15 = x4 * x14;
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x17 = x13 * x10;
	const GEN_FLT x18 = (x17 * x16) + (x15 * x13) + (x9 * x12);
	const GEN_FLT x19 = x2 * x4;
	const GEN_FLT x20 = -1 * x19;
	const GEN_FLT x21 = 2 * x17;
	const GEN_FLT x22 = x13 * x13;
	const GEN_FLT x23 = x2 * x16;
	const GEN_FLT x24 = x4 * x6;
	const GEN_FLT x25 = x24 * x11;
	const GEN_FLT x26 = x5 * x10;
	const GEN_FLT x27 = x2 * x13;
	const GEN_FLT x28 = (x4 * x5 * x27) + (x0 * x17) + (x9 * x26);
	const GEN_FLT x29 = ((x28 + (-1 * x25) + (-1 * x23)) * sensor_z) + (((x22 * x19) + (x9 * x21) + x20) * sensor_y) +
						((x18 + x8 + x3) * sensor_x);
	const GEN_FLT x30 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
							: 1e-10;
	const GEN_FLT x31 = sin(x30);
	const GEN_FLT x32 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 1;
	const GEN_FLT x33 = x32 * x31;
	const GEN_FLT x34 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 0;
	const GEN_FLT x35 = cos(x30);
	const GEN_FLT x36 = 1 + (-1 * x35);
	const GEN_FLT x37 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 0;
	const GEN_FLT x38 = x36 * x37;
	const GEN_FLT x39 = x34 * x38;
	const GEN_FLT x40 = x39 + x33;
	const GEN_FLT x41 = x40 * x29;
	const GEN_FLT x42 = x31 * x34;
	const GEN_FLT x43 = x32 * x36;
	const GEN_FLT x44 = x43 * x37;
	const GEN_FLT x45 = x44 + (-1 * x42);
	const GEN_FLT x46 = 2 * x12;
	const GEN_FLT x47 = x11 * x11;
	const GEN_FLT x48 = x2 * x9;
	const GEN_FLT x49 = x24 * x13;
	const GEN_FLT x50 = (x0 * x12) + (x5 * x15) + (x26 * x16);
	const GEN_FLT x51 = ((x50 + x49 + x48) * sensor_z) + ((x18 + (-1 * x8) + (-1 * x3)) * sensor_y) +
						(((x47 * x19) + x20 + (x46 * x16)) * sensor_x);
	const GEN_FLT x52 = 1 + x51;
	const GEN_FLT x53 = x37 * x37;
	const GEN_FLT x54 = x35 + (x53 * x36);
	const GEN_FLT x55 = 2 * x26;
	const GEN_FLT x56 = x5 * x5;
	const GEN_FLT x57 = (((x56 * x19) + (x0 * x55) + x20) * sensor_z) + ((x28 + x25 + x23) * sensor_y) +
						((x50 + (-1 * x49) + (-1 * x48)) * sensor_x);
	const GEN_FLT x58 = x26 * x11;
	const GEN_FLT x59 = x2 * x5;
	const GEN_FLT x60 = x11 * x17;
	const GEN_FLT x61 =
		((x6 + (x47 * x10)) * sensor_x) + ((x60 + (-1 * x59)) * sensor_y) + ((x58 + x27) * sensor_z) + obj_px;
	const GEN_FLT x62 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x63 = x62 * x35;
	const GEN_FLT x64 = x63 * x34;
	const GEN_FLT x65 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x66 = x65 * x31;
	const GEN_FLT x67 = x62 * x31;
	const GEN_FLT x68 = x67 * x37;
	const GEN_FLT x69 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x70 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x71 = (x70 * x43) + (x69 * x38) + (x68 * x32);
	const GEN_FLT x72 = x26 * x13;
	const GEN_FLT x73 =
		((x72 + x14) * sensor_y) + ((x58 + (-1 * x27)) * sensor_x) + ((x6 + (x56 * x10)) * sensor_z) + obj_pz;
	const GEN_FLT x74 = -1 * x67;
	const GEN_FLT x75 = x63 * x32;
	const GEN_FLT x76 = x69 * x31;
	const GEN_FLT x77 = x34 * x36;
	const GEN_FLT x78 = (x65 * x38) + (x70 * x77) + (x68 * x34);
	const GEN_FLT x79 =
		((x72 + (-1 * x14)) * sensor_z) + ((x60 + x59) * sensor_x) + ((x6 + (x22 * x10)) * sensor_y) + obj_py;
	const GEN_FLT x80 = (x79 * (x78 + x76 + x75)) + (x73 * ((x67 * x53) + (2 * x70 * x38) + x74)) +
						(x61 * (x71 + (-1 * x66) + (-1 * x64)));
	const GEN_FLT x81 = x80 + (x54 * x57);
	const GEN_FLT x82 = x81 + (x52 * x45) + x41;
	const GEN_FLT x83 = x44 + x42;
	const GEN_FLT x84 = x31 * x37;
	const GEN_FLT x85 = x43 * x34;
	const GEN_FLT x86 = x85 + (-1 * x84);
	const GEN_FLT x87 = x32 * x32;
	const GEN_FLT x88 = x35 + (x87 * x36);
	const GEN_FLT x89 = (x86 * x79) + (x88 * x61) + (x83 * x73) + lh_px;
	const GEN_FLT x90 = (x61 * x45) + (x79 * x40) + (x73 * x54) + lh_pz;
	const GEN_FLT x91 = x90 * x90;
	const GEN_FLT x92 = 1. / x91;
	const GEN_FLT x93 = x89 * x92;
	const GEN_FLT x94 = 1. / x90;
	const GEN_FLT x95 = x83 * x57;
	const GEN_FLT x96 = x86 * x29;
	const GEN_FLT x97 = x63 * x37;
	const GEN_FLT x98 = x70 * x31;
	const GEN_FLT x99 = (x67 * x32 * x34) + (x65 * x43) + (x77 * x69);
	const GEN_FLT x100 = (x73 * (x71 + x66 + x64)) + (x79 * (x99 + (-1 * x98) + (-1 * x97))) +
						 (x61 * ((2 * x69 * x43) + (x87 * x67) + x74));
	const GEN_FLT x101 = x100 + (x88 * x52) + x96 + x95;
	const GEN_FLT x102 = x89 * x89;
	const GEN_FLT x103 = x91 + x102;
	const GEN_FLT x104 = 1. / x103;
	const GEN_FLT x105 = x91 * x104;
	const GEN_FLT x106 = x105 * ((-1 * x94 * x101) + (x82 * x93));
	const GEN_FLT x107 = x39 + (-1 * x33);
	const GEN_FLT x108 = x34 * x34;
	const GEN_FLT x109 = x35 + (x36 * x108);
	const GEN_FLT x110 = x85 + x84;
	const GEN_FLT x111 = (x61 * x110) + (x79 * x109) + (x73 * x107) + lh_py;
	const GEN_FLT x112 = x111 * x111;
	const GEN_FLT x113 = 1. / sqrt(1 + (-1 * x104 * x112 * (tilt_0 * tilt_0)));
	const GEN_FLT x114 = 2 * x89;
	const GEN_FLT x115 = 2 * x90;
	const GEN_FLT x116 = x82 * x115;
	const GEN_FLT x117 = 1.0 / 2.0 * (1. / (x103 * sqrt(x103))) * x111 * tilt_0;
	const GEN_FLT x118 = x29 * x109;
	const GEN_FLT x119 = (x61 * (x99 + x98 + x97)) + (x73 * (x78 + (-1 * x76) + (-1 * x75))) +
						 (x79 * ((2 * x77 * x65) + (x67 * x108) + x74));
	const GEN_FLT x120 = x119 + (x57 * x107);
	const GEN_FLT x121 = x120 + x118 + (x52 * x110);
	const GEN_FLT x122 = (1. / sqrt(x103)) * tilt_0;
	const GEN_FLT x123 = (-1 * x113 * ((x122 * x121) + (-1 * x117 * (x116 + (x101 * x114))))) + (-1 * x106);
	const GEN_FLT x124 = -1 * x90;
	const GEN_FLT x125 = atan2(x89, x124);
	const GEN_FLT x126 =
		sin(1.5707963267949 + (-1 * x125) + (-1 * phase_0) + (-1 * asin(x111 * x122)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x127 = x94 * x121;
	const GEN_FLT x128 = x92 * x111;
	const GEN_FLT x129 = x82 * x128;
	const GEN_FLT x130 = x91 + x112;
	const GEN_FLT x131 = 1. / x130;
	const GEN_FLT x132 = x91 * x131;
	const GEN_FLT x133 = 2 * x132 * atan2(x111, x124) * curve_0;
	const GEN_FLT x134 = x51 * x45;
	const GEN_FLT x135 = 1 + x29;
	const GEN_FLT x136 = x81 + (x40 * x135) + x134;
	const GEN_FLT x137 = x100 + (x88 * x51);
	const GEN_FLT x138 = x137 + x95 + (x86 * x135);
	const GEN_FLT x139 = ((-1 * x94 * x138) + (x93 * x136)) * x105;
	const GEN_FLT x140 = x115 * x136;
	const GEN_FLT x141 = x51 * x110;
	const GEN_FLT x142 = x120 + (x109 * x135) + x141;
	const GEN_FLT x143 = (-1 * x113 * ((x122 * x142) + (-1 * x117 * (x140 + (x114 * x138))))) + (-1 * x139);
	const GEN_FLT x144 = x94 * x142;
	const GEN_FLT x145 = x128 * x136;
	const GEN_FLT x146 = 1 + x57;
	const GEN_FLT x147 = x80 + (x54 * x146) + x41 + x134;
	const GEN_FLT x148 = x137 + (x83 * x146) + x96;
	const GEN_FLT x149 = ((-1 * x94 * x148) + (x93 * x147)) * x105;
	const GEN_FLT x150 = x115 * x147;
	const GEN_FLT x151 = x119 + (x107 * x146) + x118 + x141;
	const GEN_FLT x152 = (-1 * x113 * ((x122 * x151) + (-1 * x117 * (x150 + (x114 * x148))))) + (-1 * x149);
	const GEN_FLT x153 = x94 * x151;
	const GEN_FLT x154 = x128 * x147;
	const GEN_FLT x155 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x156 = x2 * x155;
	const GEN_FLT x157 = -1 * x156;
	const GEN_FLT x158 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qi *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x159 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qk *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x160 = x2 * x159;
	const GEN_FLT x161 = x7 * x155;
	const GEN_FLT x162 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qj *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x163 = x13 * x11;
	const GEN_FLT x164 = (x163 * x156) + (x12 * x162) + (x17 * x158);
	const GEN_FLT x165 = x2 * x162;
	const GEN_FLT x166 = x6 * x13;
	const GEN_FLT x167 = x166 * x155;
	const GEN_FLT x168 = x5 * x11;
	const GEN_FLT x169 = (x168 * x156) + (x12 * x159) + (x26 * x158);
	const GEN_FLT x170 = ((x169 + x167 + x165) * sensor_z) + (((-1 * x161) + x164 + (-1 * x160)) * sensor_y) +
						 (((x47 * x156) + (x46 * x158) + x157) * sensor_x);
	const GEN_FLT x171 = x2 * x158;
	const GEN_FLT x172 = x6 * x11;
	const GEN_FLT x173 = x172 * x155;
	const GEN_FLT x174 = x5 * x13;
	const GEN_FLT x175 = (x174 * x156) + (x17 * x159) + (x26 * x162);
	const GEN_FLT x176 = (((-1 * x173) + x175 + (-1 * x171)) * sensor_z) +
						 (((x22 * x156) + x157 + (x21 * x162)) * sensor_y) + ((x161 + x164 + x160) * sensor_x);
	const GEN_FLT x177 = (((x56 * x156) + (x55 * x159) + x157) * sensor_z) + ((x173 + x175 + x171) * sensor_y) +
						 ((x169 + (-1 * x167) + (-1 * x165)) * sensor_x);
	const GEN_FLT x178 = x80 + (x40 * x176) + (x54 * x177) + (x45 * x170);
	const GEN_FLT x179 = x100 + (x83 * x177) + (x86 * x176) + (x88 * x170);
	const GEN_FLT x180 = ((-1 * x94 * x179) + (x93 * x178)) * x105;
	const GEN_FLT x181 = x115 * x178;
	const GEN_FLT x182 = x119 + (x107 * x177) + (x109 * x176) + (x110 * x170);
	const GEN_FLT x183 = (-1 * x113 * ((x122 * x182) + (-1 * x117 * (x181 + (x114 * x179))))) + (-1 * x180);
	const GEN_FLT x184 = x94 * x182;
	const GEN_FLT x185 = x128 * x178;
	const GEN_FLT x186 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x187 = x2 * x186;
	const GEN_FLT x188 = -1 * x187;
	const GEN_FLT x189 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qi *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x190 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qk *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x191 = x2 * x190;
	const GEN_FLT x192 = x7 * x186;
	const GEN_FLT x193 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qj *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x194 = (x163 * x187) + (x12 * x193) + (x17 * x189);
	const GEN_FLT x195 = x2 * x193;
	const GEN_FLT x196 = x166 * x186;
	const GEN_FLT x197 = (x12 * x190) + (x168 * x187) + (x26 * x189);
	const GEN_FLT x198 = ((x197 + x196 + x195) * sensor_z) + ((x194 + (-1 * x192) + (-1 * x191)) * sensor_y) +
						 (((x47 * x187) + (x46 * x189) + x188) * sensor_x);
	const GEN_FLT x199 = x2 * x189;
	const GEN_FLT x200 = x172 * x186;
	const GEN_FLT x201 = (x26 * x193) + (x174 * x187) + (x17 * x190);
	const GEN_FLT x202 = ((x201 + (-1 * x200) + (-1 * x199)) * sensor_z) + ((x194 + x192 + x191) * sensor_x) +
						 (((x22 * x187) + (x21 * x193) + x188) * sensor_y);
	const GEN_FLT x203 = (((x56 * x187) + (x55 * x190) + x188) * sensor_z) + ((x201 + x200 + x199) * sensor_y) +
						 ((x197 + (-1 * x196) + (-1 * x195)) * sensor_x);
	const GEN_FLT x204 = x80 + (x54 * x203) + (x40 * x202) + (x45 * x198);
	const GEN_FLT x205 = (x86 * x202) + x100 + (x83 * x203) + (x88 * x198);
	const GEN_FLT x206 = ((-1 * x94 * x205) + (x93 * x204)) * x105;
	const GEN_FLT x207 = x204 * x115;
	const GEN_FLT x208 = x119 + (x203 * x107) + (x202 * x109) + (x110 * x198);
	const GEN_FLT x209 = (-1 * x113 * ((x208 * x122) + (-1 * x117 * (x207 + (x205 * x114))))) + (-1 * x206);
	const GEN_FLT x210 = x94 * x208;
	const GEN_FLT x211 = x204 * x128;
	const GEN_FLT x212 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x213 = x2 * x212;
	const GEN_FLT x214 = -1 * x213;
	const GEN_FLT x215 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qi *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x216 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qk *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x217 = x2 * x216;
	const GEN_FLT x218 = x7 * x212;
	const GEN_FLT x219 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qj *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x220 = (x213 * x163) + (x12 * x219) + (x17 * x215);
	const GEN_FLT x221 = x2 * x219;
	const GEN_FLT x222 = x212 * x166;
	const GEN_FLT x223 = (x12 * x216) + (x213 * x168) + (x26 * x215);
	const GEN_FLT x224 = ((x223 + x222 + x221) * sensor_z) + ((x220 + (-1 * x218) + (-1 * x217)) * sensor_y) +
						 (((x47 * x213) + (x46 * x215) + x214) * sensor_x);
	const GEN_FLT x225 = x2 * x215;
	const GEN_FLT x226 = x212 * x172;
	const GEN_FLT x227 = (x213 * x174) + (x17 * x216) + (x26 * x219);
	const GEN_FLT x228 = ((x227 + (-1 * x226) + (-1 * x225)) * sensor_z) +
						 (((x22 * x213) + x214 + (x21 * x219)) * sensor_y) + ((x220 + x218 + x217) * sensor_x);
	const GEN_FLT x229 = (((x56 * x213) + (x55 * x216) + x214) * sensor_z) + ((x227 + x226 + x225) * sensor_y) +
						 ((x223 + (-1 * x222) + (-1 * x221)) * sensor_x);
	const GEN_FLT x230 = x80 + (x54 * x229) + (x40 * x228) + (x45 * x224);
	const GEN_FLT x231 = x100 + (x83 * x229) + (x86 * x228) + (x88 * x224);
	const GEN_FLT x232 = ((-1 * x94 * x231) + (x93 * x230)) * x105;
	const GEN_FLT x233 = x230 * x115;
	const GEN_FLT x234 = x119 + (x229 * x107) + (x228 * x109) + (x224 * x110);
	const GEN_FLT x235 = (-1 * x113 * ((x234 * x122) + (-1 * x117 * (x233 + (x231 * x114))))) + (-1 * x232);
	const GEN_FLT x236 = x94 * x234;
	const GEN_FLT x237 = x230 * x128;
	const GEN_FLT x238 = 2 * x125 * curve_1;
	const GEN_FLT x239 = 2 * x111;
	const GEN_FLT x240 = 1.0 / 2.0 * x89 * (1. / (x130 * sqrt(x130))) * tilt_1;
	const GEN_FLT x241 = (1. / sqrt(x130)) * tilt_1;
	const GEN_FLT x242 = 1. / sqrt(1 + (-1 * x102 * x131 * (tilt_1 * tilt_1)));
	const GEN_FLT x243 =
		(-1 * x242 * ((x241 * x101) + (-1 * x240 * (x116 + (x239 * x121))))) + (-1 * ((-1 * x129) + x127) * x132);
	const GEN_FLT x244 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x111, x124)) + (-1 * phase_1) + (-1 * asin(x89 * x241)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x245 =
		(-1 * x242 * ((x241 * x138) + (-1 * x240 * (x140 + (x239 * x142))))) + (-1 * ((-1 * x145) + x144) * x132);
	const GEN_FLT x246 =
		(-1 * x242 * ((x241 * x148) + (-1 * x240 * (x150 + (x239 * x151))))) + (-1 * ((-1 * x154) + x153) * x132);
	const GEN_FLT x247 =
		(-1 * x242 * ((x241 * x179) + (-1 * x240 * (x181 + (x239 * x182))))) + (-1 * ((-1 * x185) + x184) * x132);
	const GEN_FLT x248 =
		(-1 * x242 * ((x205 * x241) + (-1 * x240 * (x207 + (x239 * x208))))) + (-1 * ((-1 * x211) + x210) * x132);
	const GEN_FLT x249 =
		(-1 * x242 * ((x231 * x241) + (-1 * x240 * (x233 + (x234 * x239))))) + (-1 * ((-1 * x237) + x236) * x132);
	out[0] = x123 + ((x129 + (-1 * x127)) * x133) + (x123 * x126);
	out[1] = ((x145 + (-1 * x144)) * x133) + x143 + (x126 * x143);
	out[2] = x152 + ((x154 + (-1 * x153)) * x133) + (x126 * x152);
	out[3] = ((x185 + (-1 * x184)) * x133) + x183 + (x126 * x183);
	out[4] = x209 + ((x211 + (-1 * x210)) * x133) + (x209 * x126);
	out[5] = x235 + ((x237 + (-1 * x236)) * x133) + (x235 * x126);
	out[6] = x243 + (x243 * x244) + (x238 * x106);
	out[7] = x245 + (x244 * x245) + (x238 * x139);
	out[8] = x246 + (x244 * x246) + (x238 * x149);
	out[9] = x247 + (x244 * x247) + (x238 * x180);
	out[10] = x248 + (x244 * x248) + (x238 * x206);
	out[11] = x249 + (x244 * x249) + (x232 * x238);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x5 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x9 + x3;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = x14 + (x15 * x12);
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x18 = sin(x13);
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x21 * x11;
	const GEN_FLT x23 = x22 + x19;
	const GEN_FLT x24 = x20 * x18;
	const GEN_FLT x25 = x15 * x17;
	const GEN_FLT x26 = x25 * x11;
	const GEN_FLT x27 = x26 + (-1 * x24);
	const GEN_FLT x28 = (x23 * sensor_y) + (x16 * sensor_z) + (x27 * sensor_x) + obj_pz;
	const GEN_FLT x29 = x1 * x5;
	const GEN_FLT x30 = x2 * x7;
	const GEN_FLT x31 = x4 * x30;
	const GEN_FLT x32 = x31 + (-1 * x29);
	const GEN_FLT x33 = x22 + (-1 * x19);
	const GEN_FLT x34 = x20 * x20;
	const GEN_FLT x35 = x14 + (x34 * x15);
	const GEN_FLT x36 = x11 * x18;
	const GEN_FLT x37 = x25 * x20;
	const GEN_FLT x38 = x37 + x36;
	const GEN_FLT x39 = (x38 * sensor_x) + (x35 * sensor_y) + (x33 * sensor_z) + obj_py;
	const GEN_FLT x40 = x4 * x4;
	const GEN_FLT x41 = x6 + (x7 * x40);
	const GEN_FLT x42 = x26 + x24;
	const GEN_FLT x43 = x37 + (-1 * x36);
	const GEN_FLT x44 = x17 * x17;
	const GEN_FLT x45 = x14 + (x44 * x15);
	const GEN_FLT x46 = (x45 * sensor_x) + (x43 * sensor_y) + (x42 * sensor_z) + obj_px;
	const GEN_FLT x47 = (x41 * x46) + (x32 * x39) + (x28 * x10) + lh_px;
	const GEN_FLT x48 = x5 * x5;
	const GEN_FLT x49 = x6 + (x7 * x48);
	const GEN_FLT x50 = x1 * x4;
	const GEN_FLT x51 = x5 * x30;
	const GEN_FLT x52 = x51 + x50;
	const GEN_FLT x53 = x9 + (-1 * x3);
	const GEN_FLT x54 = (x53 * x46) + (x52 * x39) + (x49 * x28) + lh_pz;
	const GEN_FLT x55 = x54 * x54;
	const GEN_FLT x56 = 1. / x55;
	const GEN_FLT x57 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x58 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x59 = x58 * x18;
	const GEN_FLT x60 = -1 * x59;
	const GEN_FLT x61 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x62 = x61 * x18;
	const GEN_FLT x63 = x58 * x14;
	const GEN_FLT x64 = x63 * x11;
	const GEN_FLT x65 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x66 = x58 * x19;
	const GEN_FLT x67 = (x57 * x21) + (x66 * x20) + (x65 * x25);
	const GEN_FLT x68 = x65 * x18;
	const GEN_FLT x69 = x63 * x20;
	const GEN_FLT x70 = x15 * x11;
	const GEN_FLT x71 = (x66 * x11) + (x61 * x25) + (x70 * x57);
	const GEN_FLT x72 = ((x71 + x69 + x68) * sensor_z) + ((x67 + (-1 * x64) + (-1 * x62)) * sensor_y) +
						(((x59 * x44) + x60 + (2 * x57 * x25)) * sensor_x);
	const GEN_FLT x73 = x72 + x45;
	const GEN_FLT x74 = x57 * x18;
	const GEN_FLT x75 = x63 * x17;
	const GEN_FLT x76 = (x58 * x36 * x20) + (x61 * x21) + (x70 * x65);
	const GEN_FLT x77 = ((x76 + (-1 * x75) + (-1 * x74)) * sensor_z) +
						(((x59 * x34) + (2 * x65 * x21) + x60) * sensor_y) + ((x67 + x64 + x62) * sensor_x);
	const GEN_FLT x78 = x77 + x38;
	const GEN_FLT x79 = (((x59 * x12) + (2 * x70 * x61) + x60) * sensor_z) + ((x76 + x75 + x74) * sensor_y) +
						((x71 + (-1 * x69) + (-1 * x68)) * sensor_x);
	const GEN_FLT x80 = x79 + x27;
	const GEN_FLT x81 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x82 = x6 * x81;
	const GEN_FLT x83 = x2 * x82;
	const GEN_FLT x84 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x85 = x1 * x84;
	const GEN_FLT x86 = x4 * x81;
	const GEN_FLT x87 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x88 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x89 = x4 * x7;
	const GEN_FLT x90 = (x88 * x89) + (x8 * x87) + (x86 * x29);
	const GEN_FLT x91 = x4 * x82;
	const GEN_FLT x92 = x1 * x87;
	const GEN_FLT x93 = (x8 * x84) + (x88 * x30) + (x3 * x5 * x81);
	const GEN_FLT x94 = x1 * x81;
	const GEN_FLT x95 = -1 * x94;
	const GEN_FLT x96 = (x28 * ((2 * x8 * x88) + (x94 * x48) + x95)) + (x39 * (x93 + x92 + x91)) +
						(x46 * (x90 + (-1 * x85) + (-1 * x83)));
	const GEN_FLT x97 = x96 + (x80 * x49) + (x78 * x52) + (x73 * x53);
	const GEN_FLT x98 = x56 * x97;
	const GEN_FLT x99 = 1. / x54;
	const GEN_FLT x100 = x5 * x82;
	const GEN_FLT x101 = x1 * x88;
	const GEN_FLT x102 = (x3 * x86) + (x89 * x84) + (x87 * x30);
	const GEN_FLT x103 = (x46 * ((2 * x89 * x87) + (x94 * x40) + x95)) + (x28 * (x90 + x85 + x83)) +
						 (x39 * (x102 + (-1 * x101) + (-1 * x100)));
	const GEN_FLT x104 = x103 + (x80 * x10) + (x78 * x32) + (x73 * x41);
	const GEN_FLT x105 = x47 * x47;
	const GEN_FLT x106 = x55 + x105;
	const GEN_FLT x107 = 1. / x106;
	const GEN_FLT x108 = x55 * x107;
	const GEN_FLT x109 = x108 * ((-1 * x99 * x104) + (x98 * x47));
	const GEN_FLT x110 = x51 + (-1 * x50);
	const GEN_FLT x111 = x2 * x2;
	const GEN_FLT x112 = x6 + (x7 * x111);
	const GEN_FLT x113 = x31 + x29;
	const GEN_FLT x114 = (x46 * x113) + (x39 * x112) + (x28 * x110) + lh_py;
	const GEN_FLT x115 = x114 * x114;
	const GEN_FLT x116 = 1. / sqrt(1 + (-1 * x107 * x115 * (tilt_0 * tilt_0)));
	const GEN_FLT x117 = 2 * x47;
	const GEN_FLT x118 = 2 * x54;
	const GEN_FLT x119 = x97 * x118;
	const GEN_FLT x120 = 1.0 / 2.0 * (1. / (x106 * sqrt(x106))) * x114 * tilt_0;
	const GEN_FLT x121 = (x28 * (x93 + (-1 * x92) + (-1 * x91))) + (x39 * ((2 * x84 * x30) + (x94 * x111) + x95)) +
						 (x46 * (x102 + x101 + x100));
	const GEN_FLT x122 = x121 + (x80 * x110) + (x78 * x112) + (x73 * x113);
	const GEN_FLT x123 = (1. / sqrt(x106)) * tilt_0;
	const GEN_FLT x124 = (-1 * x116 * ((x123 * x122) + (-1 * x120 * (x119 + (x104 * x117))))) + (-1 * x109);
	const GEN_FLT x125 = -1 * x54;
	const GEN_FLT x126 = atan2(x47, x125);
	const GEN_FLT x127 =
		sin(1.5707963267949 + (-1 * x126) + (-1 * phase_0) + (-1 * asin(x114 * x123)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x128 = x99 * x122;
	const GEN_FLT x129 = x98 * x114;
	const GEN_FLT x130 = x55 + x115;
	const GEN_FLT x131 = 1. / x130;
	const GEN_FLT x132 = x55 * x131;
	const GEN_FLT x133 = 2 * x132 * atan2(x114, x125) * curve_0;
	const GEN_FLT x134 = x72 + x43;
	const GEN_FLT x135 = x77 + x35;
	const GEN_FLT x136 = x79 + x23;
	const GEN_FLT x137 = x96 + (x49 * x136) + (x52 * x135) + (x53 * x134);
	const GEN_FLT x138 = x56 * x137;
	const GEN_FLT x139 = (x10 * x136) + (x32 * x135) + x103 + (x41 * x134);
	const GEN_FLT x140 = ((-1 * x99 * x139) + (x47 * x138)) * x108;
	const GEN_FLT x141 = x118 * x137;
	const GEN_FLT x142 = x121 + (x110 * x136) + (x112 * x135) + (x113 * x134);
	const GEN_FLT x143 = (-1 * x116 * ((x123 * x142) + (-1 * x120 * (x141 + (x117 * x139))))) + (-1 * x140);
	const GEN_FLT x144 = x99 * x142;
	const GEN_FLT x145 = x114 * x138;
	const GEN_FLT x146 = x72 + x42;
	const GEN_FLT x147 = x77 + x33;
	const GEN_FLT x148 = x79 + x16;
	const GEN_FLT x149 = x96 + (x49 * x148) + (x52 * x147) + (x53 * x146);
	const GEN_FLT x150 = x56 * x149;
	const GEN_FLT x151 = x103 + (x10 * x148) + (x32 * x147) + (x41 * x146);
	const GEN_FLT x152 = ((-1 * x99 * x151) + (x47 * x150)) * x108;
	const GEN_FLT x153 = x118 * x149;
	const GEN_FLT x154 = x121 + (x110 * x148) + (x113 * x146) + (x112 * x147);
	const GEN_FLT x155 = (-1 * x116 * ((x123 * x154) + (-1 * x120 * (x153 + (x117 * x151))))) + (-1 * x152);
	const GEN_FLT x156 = x99 * x154;
	const GEN_FLT x157 = x114 * x150;
	const GEN_FLT x158 = 2 * x126 * curve_1;
	const GEN_FLT x159 = 2 * x114;
	const GEN_FLT x160 = 1.0 / 2.0 * x47 * (1. / (x130 * sqrt(x130))) * tilt_1;
	const GEN_FLT x161 = (1. / sqrt(x130)) * tilt_1;
	const GEN_FLT x162 = 1. / sqrt(1 + (-1 * x105 * x131 * (tilt_1 * tilt_1)));
	const GEN_FLT x163 =
		(-1 * x162 * ((x104 * x161) + (-1 * x160 * (x119 + (x122 * x159))))) + (-1 * ((-1 * x129) + x128) * x132);
	const GEN_FLT x164 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x114, x125)) + (-1 * phase_1) + (-1 * asin(x47 * x161)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x165 =
		(-1 * x162 * ((x161 * x139) + (-1 * x160 * (x141 + (x142 * x159))))) + (-1 * ((-1 * x145) + x144) * x132);
	const GEN_FLT x166 =
		(-1 * x162 * ((x161 * x151) + (-1 * x160 * (x153 + (x154 * x159))))) + (-1 * ((-1 * x157) + x156) * x132);
	out[0] = x124 + ((x129 + (-1 * x128)) * x133) + (x124 * x127);
	out[1] = x143 + ((x145 + (-1 * x144)) * x133) + (x127 * x143);
	out[2] = ((x157 + (-1 * x156)) * x133) + x155 + (x127 * x155);
	out[3] = x163 + (x164 * x163) + (x109 * x158);
	out[4] = x165 + (x165 * x164) + (x140 * x158);
	out[5] = x166 + (x166 * x164) + (x152 * x158);
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
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 0;
	const GEN_FLT x1 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						   : 1e-10;
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 1;
	const GEN_FLT x5 = cos(x1);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x2 * x7;
	const GEN_FLT x11 = x0 * x6;
	const GEN_FLT x12 = x4 * x11;
	const GEN_FLT x13 = x4 * x4;
	const GEN_FLT x14 =
		((x5 + (x6 * x13)) * sensor_x) + ((x12 + (-1 * x10)) * sensor_y) + ((x9 + x3) * sensor_z) + obj_px;
	const GEN_FLT x15 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 0;
	const GEN_FLT x16 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x17 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
							: 1e-10;
	const GEN_FLT x18 = cos(x17);
	const GEN_FLT x19 = x18 * x16;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = sin(x17);
	const GEN_FLT x22 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 0;
	const GEN_FLT x25 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 1;
	const GEN_FLT x26 = x21 * x16;
	const GEN_FLT x27 = x25 * x26;
	const GEN_FLT x28 = 1 + (-1 * x18);
	const GEN_FLT x29 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x30 = x28 * x29;
	const GEN_FLT x31 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x32 = x25 * x28;
	const GEN_FLT x33 = (x32 * x31) + (x30 * x24) + (x24 * x27);
	const GEN_FLT x34 = x25 * x19;
	const GEN_FLT x35 = x21 * x29;
	const GEN_FLT x36 = x24 * x15;
	const GEN_FLT x37 = x28 * x15;
	const GEN_FLT x38 = x24 * x28;
	const GEN_FLT x39 = (x38 * x22) + (x31 * x37) + (x36 * x26);
	const GEN_FLT x40 = x2 * x4;
	const GEN_FLT x41 = x7 * x11;
	const GEN_FLT x42 = x0 * x0;
	const GEN_FLT x43 =
		((x12 + x10) * sensor_x) + ((x5 + (x6 * x42)) * sensor_y) + ((x41 + (-1 * x40)) * sensor_z) + obj_py;
	const GEN_FLT x44 = x7 * x7;
	const GEN_FLT x45 =
		((x9 + (-1 * x3)) * sensor_x) + ((x41 + x40) * sensor_y) + ((x5 + (x6 * x44)) * sensor_z) + obj_pz;
	const GEN_FLT x46 = -1 * x26;
	const GEN_FLT x47 = x24 * x24;
	const GEN_FLT x48 = 2 * x38;
	const GEN_FLT x49 = x21 * x15;
	const GEN_FLT x50 = x32 * x24;
	const GEN_FLT x51 = x50 + (-1 * x49);
	const GEN_FLT x52 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x53 = x4 * x6;
	const GEN_FLT x54 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x55 = x2 * x54;
	const GEN_FLT x56 = -1 * x55;
	const GEN_FLT x57 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x58 = x2 * x57;
	const GEN_FLT x59 = x5 * x54;
	const GEN_FLT x60 = x7 * x59;
	const GEN_FLT x61 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x62 = x54 * x40;
	const GEN_FLT x63 = (x52 * x11) + (x0 * x62) + (x61 * x53);
	const GEN_FLT x64 = x2 * x61;
	const GEN_FLT x65 = x0 * x59;
	const GEN_FLT x66 = (x53 * x57) + (x7 * x62) + (x8 * x52);
	const GEN_FLT x67 = ((x65 + x66 + x64) * sensor_z) + ((x63 + (-1 * x60) + (-1 * x58)) * sensor_y) +
						(((x55 * x13) + x56 + (2 * x53 * x52)) * sensor_x);
	const GEN_FLT x68 = x2 * x52;
	const GEN_FLT x69 = x4 * x59;
	const GEN_FLT x70 = (x0 * x7 * x55) + (x57 * x11) + (x8 * x61);
	const GEN_FLT x71 = (((-1 * x69) + x70 + (-1 * x68)) * sensor_z) +
						(((x55 * x42) + (2 * x61 * x11) + x56) * sensor_y) + ((x63 + x60 + x58) * sensor_x);
	const GEN_FLT x72 = x25 * x21;
	const GEN_FLT x73 = x37 * x24;
	const GEN_FLT x74 = x73 + x72;
	const GEN_FLT x75 = x18 + (x47 * x28);
	const GEN_FLT x76 = (((2 * x8 * x57) + (x55 * x44) + x56) * sensor_z) + ((x69 + x70 + x68) * sensor_y) +
						(((-1 * x65) + x66 + (-1 * x64)) * sensor_x);
	const GEN_FLT x77 = (x75 * x76) + (x71 * x74) + (x67 * x51);
	const GEN_FLT x78 = x77 + (x45 * ((x48 * x31) + (x47 * x26) + x46)) + (x43 * (x39 + x35 + x34)) +
						(x14 * (x33 + (-1 * x23) + (-1 * x20)));
	const GEN_FLT x79 = x50 + x49;
	const GEN_FLT x80 = x24 * x21;
	const GEN_FLT x81 = x32 * x15;
	const GEN_FLT x82 = x81 + (-1 * x80);
	const GEN_FLT x83 = x25 * x25;
	const GEN_FLT x84 = x18 + (x83 * x28);
	const GEN_FLT x85 = (x84 * x14) + (x82 * x43) + (x79 * x45) + lh_px;
	const GEN_FLT x86 = (x74 * x43) + (x51 * x14) + (x75 * x45) + lh_pz;
	const GEN_FLT x87 = x86 * x86;
	const GEN_FLT x88 = 1. / x87;
	const GEN_FLT x89 = x88 * x85;
	const GEN_FLT x90 = x89 * x78;
	const GEN_FLT x91 = 1. / x86;
	const GEN_FLT x92 = 2 * x32;
	const GEN_FLT x93 = x24 * x19;
	const GEN_FLT x94 = x31 * x21;
	const GEN_FLT x95 = (x27 * x15) + (x32 * x22) + (x30 * x15);
	const GEN_FLT x96 = (x79 * x76) + (x82 * x71) + (x84 * x67);
	const GEN_FLT x97 = (x45 * (x33 + x23 + x20)) + x96 + (x43 * (x95 + (-1 * x94) + (-1 * x93))) +
						(x14 * ((x92 * x29) + (x83 * x26) + x46));
	const GEN_FLT x98 = 1 + x97;
	const GEN_FLT x99 = x85 * x85;
	const GEN_FLT x100 = x87 + x99;
	const GEN_FLT x101 = 1. / x100;
	const GEN_FLT x102 = x87 * x101;
	const GEN_FLT x103 = x102 * ((-1 * x91 * x98) + x90);
	const GEN_FLT x104 = 2 * x85;
	const GEN_FLT x105 = 2 * x86;
	const GEN_FLT x106 = x78 * x105;
	const GEN_FLT x107 = x73 + (-1 * x72);
	const GEN_FLT x108 = x15 * x15;
	const GEN_FLT x109 = x18 + (x28 * x108);
	const GEN_FLT x110 = x81 + x80;
	const GEN_FLT x111 = (x14 * x110) + (x45 * x107) + (x43 * x109) + lh_py;
	const GEN_FLT x112 = 1.0 / 2.0 * (1. / (x100 * sqrt(x100))) * x111 * tilt_0;
	const GEN_FLT x113 = 2 * x37;
	const GEN_FLT x114 = (x76 * x107) + (x71 * x109) + (x67 * x110);
	const GEN_FLT x115 = x114 + (x45 * (x39 + (-1 * x35) + (-1 * x34))) + (x43 * ((x22 * x113) + (x26 * x108) + x46)) +
						 (x14 * (x95 + x94 + x93));
	const GEN_FLT x116 = (1. / sqrt(x100)) * tilt_0;
	const GEN_FLT x117 = x115 * x116;
	const GEN_FLT x118 = x111 * x111;
	const GEN_FLT x119 = 1. / sqrt(1 + (-1 * x101 * x118 * (tilt_0 * tilt_0)));
	const GEN_FLT x120 = (-1 * x119 * (x117 + (-1 * x112 * (x106 + (x98 * x104))))) + (-1 * x103);
	const GEN_FLT x121 = -1 * x86;
	const GEN_FLT x122 = atan2(x85, x121);
	const GEN_FLT x123 =
		sin(1.5707963267949 + (-1 * x122) + (-1 * phase_0) + (-1 * asin(x111 * x116)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x124 = x91 * x115;
	const GEN_FLT x125 = -1 * x124;
	const GEN_FLT x126 = x88 * x111;
	const GEN_FLT x127 = x78 * x126;
	const GEN_FLT x128 = x87 + x118;
	const GEN_FLT x129 = 1. / x128;
	const GEN_FLT x130 = x87 * x129;
	const GEN_FLT x131 = 2 * x130 * atan2(x111, x121) * curve_0;
	const GEN_FLT x132 = -1 * x91 * x97;
	const GEN_FLT x133 = x102 * (x132 + x90);
	const GEN_FLT x134 = x97 * x104;
	const GEN_FLT x135 = 1 + x115;
	const GEN_FLT x136 = (-1 * x119 * ((x116 * x135) + (-1 * (x106 + x134) * x112))) + (-1 * x133);
	const GEN_FLT x137 = x91 * x135;
	const GEN_FLT x138 = 1 + x78;
	const GEN_FLT x139 = x102 * (x132 + (x89 * x138));
	const GEN_FLT x140 = x105 * x138;
	const GEN_FLT x141 = (-1 * x119 * (x117 + (-1 * (x140 + x134) * x112))) + (-1 * x139);
	const GEN_FLT x142 = x126 * x138;
	const GEN_FLT x143 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x144 = x18 * x143;
	const GEN_FLT x145 = x15 * x144;
	const GEN_FLT x146 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qj *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x147 = x21 * x146;
	const GEN_FLT x148 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qi *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x149 = x28 * x148;
	const GEN_FLT x150 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qk *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x151 = x21 * x143;
	const GEN_FLT x152 = x25 * x151;
	const GEN_FLT x153 = (x24 * x152) + (x32 * x150) + (x24 * x149);
	const GEN_FLT x154 = x25 * x144;
	const GEN_FLT x155 = x21 * x148;
	const GEN_FLT x156 = (x38 * x146) + (x37 * x150) + (x36 * x151);
	const GEN_FLT x157 = -1 * x151;
	const GEN_FLT x158 = (x45 * ((x48 * x150) + (x47 * x151) + x157)) + x77 + (x43 * (x156 + x155 + x154)) +
						 (x14 * (x153 + (-1 * x147) + (-1 * x145)));
	const GEN_FLT x159 = x24 * x144;
	const GEN_FLT x160 = x21 * x150;
	const GEN_FLT x161 = (x32 * x146) + (x15 * x149) + (x15 * x152);
	const GEN_FLT x162 = x96 + (x45 * (x153 + x147 + x145)) + (x43 * (x161 + (-1 * x160) + (-1 * x159))) +
						 (x14 * ((x92 * x148) + (x83 * x151) + x157));
	const GEN_FLT x163 = ((-1 * x91 * x162) + (x89 * x158)) * x102;
	const GEN_FLT x164 = x105 * x158;
	const GEN_FLT x165 = x114 + (x14 * (x161 + x160 + x159)) + (x45 * (x156 + (-1 * x155) + (-1 * x154))) +
						 (x43 * ((x113 * x146) + (x108 * x151) + x157));
	const GEN_FLT x166 = (-1 * x119 * ((x116 * x165) + (-1 * x112 * (x164 + (x104 * x162))))) + (-1 * x163);
	const GEN_FLT x167 = x91 * x165;
	const GEN_FLT x168 = x126 * x158;
	const GEN_FLT x169 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x170 = x18 * x169;
	const GEN_FLT x171 = x15 * x170;
	const GEN_FLT x172 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qj *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x173 = x21 * x172;
	const GEN_FLT x174 = x21 * x169;
	const GEN_FLT x175 = x25 * x174;
	const GEN_FLT x176 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qi *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x177 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qk *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x178 = (x32 * x177) + (x38 * x176) + (x24 * x175);
	const GEN_FLT x179 = x25 * x170;
	const GEN_FLT x180 = x21 * x176;
	const GEN_FLT x181 = (x38 * x172) + (x37 * x177) + (x36 * x174);
	const GEN_FLT x182 = -1 * x174;
	const GEN_FLT x183 = x77 + (x45 * ((x48 * x177) + (x47 * x174) + x182)) + (x43 * (x181 + x180 + x179)) +
						 (x14 * (x178 + (-1 * x173) + (-1 * x171)));
	const GEN_FLT x184 = x24 * x170;
	const GEN_FLT x185 = x21 * x177;
	const GEN_FLT x186 = (x37 * x176) + (x32 * x172) + (x15 * x175);
	const GEN_FLT x187 = x96 + (x43 * (x186 + (-1 * x185) + (-1 * x184))) + (x45 * (x178 + x173 + x171)) +
						 (x14 * ((x92 * x176) + x182 + (x83 * x174)));
	const GEN_FLT x188 = ((-1 * x91 * x187) + (x89 * x183)) * x102;
	const GEN_FLT x189 = x105 * x183;
	const GEN_FLT x190 = x114 + (x45 * (x181 + (-1 * x180) + (-1 * x179))) +
						 (x43 * ((x113 * x172) + (x108 * x174) + x182)) + (x14 * (x186 + x185 + x184));
	const GEN_FLT x191 = (-1 * x119 * ((x116 * x190) + (-1 * x112 * (x189 + (x104 * x187))))) + (-1 * x188);
	const GEN_FLT x192 = x91 * x190;
	const GEN_FLT x193 = x126 * x183;
	const GEN_FLT x194 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x195 = x18 * x194;
	const GEN_FLT x196 = x15 * x195;
	const GEN_FLT x197 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qj *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x198 = x21 * x197;
	const GEN_FLT x199 = x21 * x194;
	const GEN_FLT x200 = x25 * x199;
	const GEN_FLT x201 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qi *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x202 = x28 * x201;
	const GEN_FLT x203 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qk *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x204 = (x32 * x203) + (x24 * x202) + (x24 * x200);
	const GEN_FLT x205 = x25 * x195;
	const GEN_FLT x206 = x21 * x201;
	const GEN_FLT x207 = (x38 * x197) + (x37 * x203) + (x36 * x199);
	const GEN_FLT x208 = -1 * x199;
	const GEN_FLT x209 = x77 + (x45 * ((x48 * x203) + (x47 * x199) + x208)) + (x43 * (x207 + x206 + x205)) +
						 (x14 * (x204 + (-1 * x198) + (-1 * x196)));
	const GEN_FLT x210 = x24 * x195;
	const GEN_FLT x211 = x21 * x203;
	const GEN_FLT x212 = (x32 * x197) + (x15 * x202) + (x15 * x200);
	const GEN_FLT x213 = (x45 * (x204 + x198 + x196)) + x96 + (x43 * (x212 + (-1 * x211) + (-1 * x210))) +
						 (x14 * ((2 * x25 * x202) + (x83 * x199) + x208));
	const GEN_FLT x214 = ((-1 * x91 * x213) + (x89 * x209)) * x102;
	const GEN_FLT x215 = x209 * x105;
	const GEN_FLT x216 = (x45 * (x207 + (-1 * x206) + (-1 * x205))) + x114 +
						 (x43 * ((x113 * x197) + (x108 * x199) + x208)) + (x14 * (x212 + x211 + x210));
	const GEN_FLT x217 = (-1 * x119 * ((x216 * x116) + (-1 * x112 * (x215 + (x213 * x104))))) + (-1 * x214);
	const GEN_FLT x218 = x91 * x216;
	const GEN_FLT x219 = x209 * x126;
	const GEN_FLT x220 = 2 * x122 * curve_1;
	const GEN_FLT x221 = -1 * x127;
	const GEN_FLT x222 = 2 * x111;
	const GEN_FLT x223 = x222 * x115;
	const GEN_FLT x224 = 1.0 / 2.0 * x85 * (1. / (x128 * sqrt(x128))) * tilt_1;
	const GEN_FLT x225 = (1. / sqrt(x128)) * tilt_1;
	const GEN_FLT x226 = 1. / sqrt(1 + (-1 * x99 * x129 * (tilt_1 * tilt_1)));
	const GEN_FLT x227 = (-1 * x226 * ((x98 * x225) + (-1 * (x106 + x223) * x224))) + (-1 * (x221 + x124) * x130);
	const GEN_FLT x228 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x111, x121)) + (-1 * phase_1) + (-1 * asin(x85 * x225)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x229 = x97 * x225;
	const GEN_FLT x230 = (-1 * x226 * (x229 + (-1 * x224 * (x106 + (x222 * x135))))) + (-1 * (x221 + x137) * x130);
	const GEN_FLT x231 = (-1 * x226 * (x229 + (-1 * (x140 + x223) * x224))) + (-1 * ((-1 * x142) + x124) * x130);
	const GEN_FLT x232 =
		(-1 * x226 * ((x225 * x162) + (-1 * x224 * (x164 + (x222 * x165))))) + (-1 * ((-1 * x168) + x167) * x130);
	const GEN_FLT x233 =
		(-1 * x226 * ((x225 * x187) + (-1 * x224 * (x189 + (x222 * x190))))) + (-1 * ((-1 * x193) + x192) * x130);
	const GEN_FLT x234 =
		(-1 * x226 * ((x213 * x225) + (-1 * x224 * (x215 + (x216 * x222))))) + (-1 * ((-1 * x219) + x218) * x130);
	out[0] = x120 + ((x127 + x125) * x131) + (x120 * x123);
	out[1] = x136 + ((x127 + (-1 * x137)) * x131) + (x123 * x136);
	out[2] = x141 + ((x142 + x125) * x131) + (x123 * x141);
	out[3] = ((x168 + (-1 * x167)) * x131) + x166 + (x123 * x166);
	out[4] = x191 + ((x193 + (-1 * x192)) * x131) + (x123 * x191);
	out[5] = x217 + ((x219 + (-1 * x218)) * x131) + (x217 * x123);
	out[6] = x227 + (x227 * x228) + (x220 * x103);
	out[7] = x230 + (x230 * x228) + (x220 * x133);
	out[8] = x231 + (x231 * x228) + (x220 * x139);
	out[9] = x232 + (x232 * x228) + (x220 * x163);
	out[10] = x233 + (x233 * x228) + (x220 * x188);
	out[11] = x234 + (x234 * x228) + (x214 * x220);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 + (-1 * x1);
	const GEN_FLT x3 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x4 = x3 * x3;
	const GEN_FLT x5 = x1 + (x2 * x4);
	const GEN_FLT x6 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 0;
	const GEN_FLT x7 = x6 * x6;
	const GEN_FLT x8 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						   : 1e-10;
	const GEN_FLT x9 = cos(x8);
	const GEN_FLT x10 = 1 + (-1 * x9);
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x12 = sin(x8);
	const GEN_FLT x13 = x12 * x11;
	const GEN_FLT x14 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x15 = x14 * x10;
	const GEN_FLT x16 = x6 * x15;
	const GEN_FLT x17 = x14 * x12;
	const GEN_FLT x18 = x11 * x10;
	const GEN_FLT x19 = x6 * x18;
	const GEN_FLT x20 =
		((x19 + (-1 * x17)) * sensor_x) + ((x16 + x13) * sensor_y) + ((x9 + (x7 * x10)) * sensor_z) + obj_pz;
	const GEN_FLT x21 = sin(x0);
	const GEN_FLT x22 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 1;
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 0;
	const GEN_FLT x25 = x2 * x3;
	const GEN_FLT x26 = x24 * x25;
	const GEN_FLT x27 = x26 + x23;
	const GEN_FLT x28 = x14 * x14;
	const GEN_FLT x29 = x6 * x12;
	const GEN_FLT x30 = x15 * x11;
	const GEN_FLT x31 =
		((x30 + x29) * sensor_x) + ((x16 + (-1 * x13)) * sensor_z) + ((x9 + (x28 * x10)) * sensor_y) + obj_py;
	const GEN_FLT x32 = x24 * x21;
	const GEN_FLT x33 = x25 * x22;
	const GEN_FLT x34 = x33 + (-1 * x32);
	const GEN_FLT x35 = x11 * x11;
	const GEN_FLT x36 =
		((x9 + (x35 * x10)) * sensor_x) + ((x30 + (-1 * x29)) * sensor_y) + ((x19 + x17) * sensor_z) + obj_px;
	const GEN_FLT x37 = (x34 * x36) + (x31 * x27) + (x5 * x20) + lh_pz;
	const GEN_FLT x38 = 1. / x37;
	const GEN_FLT x39 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x40 = x1 * x39;
	const GEN_FLT x41 = x3 * x40;
	const GEN_FLT x42 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x43 = x42 * x21;
	const GEN_FLT x44 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x45 = x2 * x24;
	const GEN_FLT x46 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x47 = x2 * x22;
	const GEN_FLT x48 = x32 * x39;
	const GEN_FLT x49 = (x48 * x22) + (x46 * x47) + (x44 * x45);
	const GEN_FLT x50 = x3 * x21;
	const GEN_FLT x51 = x45 * x22;
	const GEN_FLT x52 = x51 + x50;
	const GEN_FLT x53 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x54 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x55 = x54 * x12;
	const GEN_FLT x56 = -1 * x55;
	const GEN_FLT x57 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x58 = x57 * x12;
	const GEN_FLT x59 = x9 * x54;
	const GEN_FLT x60 = x6 * x59;
	const GEN_FLT x61 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x62 = x54 * x13;
	const GEN_FLT x63 = (x53 * x15) + (x62 * x14) + (x61 * x18);
	const GEN_FLT x64 = x61 * x12;
	const GEN_FLT x65 = x59 * x14;
	const GEN_FLT x66 = x6 * x10;
	const GEN_FLT x67 = (x57 * x18) + (x6 * x62) + (x66 * x53);
	const GEN_FLT x68 = ((x67 + x65 + x64) * sensor_z) + ((x63 + (-1 * x60) + (-1 * x58)) * sensor_y) +
						(((x55 * x35) + x56 + (2 * x53 * x18)) * sensor_x);
	const GEN_FLT x69 = x39 * x21;
	const GEN_FLT x70 = -1 * x69;
	const GEN_FLT x71 = x24 * x24;
	const GEN_FLT x72 = x53 * x12;
	const GEN_FLT x73 = x59 * x11;
	const GEN_FLT x74 = (x6 * x55 * x14) + (x57 * x15) + (x61 * x66);
	const GEN_FLT x75 = ((x74 + (-1 * x73) + (-1 * x72)) * sensor_z) +
						(((x55 * x28) + (2 * x61 * x15) + x56) * sensor_y) + ((x63 + x60 + x58) * sensor_x);
	const GEN_FLT x76 = x1 + (x2 * x71);
	const GEN_FLT x77 = x40 * x22;
	const GEN_FLT x78 = x44 * x21;
	const GEN_FLT x79 = (x46 * x25) + (x42 * x45) + (x3 * x48);
	const GEN_FLT x80 = x26 + (-1 * x23);
	const GEN_FLT x81 = (((x7 * x55) + (2 * x66 * x57) + x56) * sensor_z) + ((x74 + x73 + x72) * sensor_y) +
						((x67 + (-1 * x65) + (-1 * x64)) * sensor_x);
	const GEN_FLT x82 = (x80 * x81) + (x75 * x76) + (x20 * (x79 + (-1 * x78) + (-1 * x77))) + (x68 * x52) +
						(x31 * ((2 * x45 * x46) + (x71 * x69) + x70)) + (x36 * (x49 + x43 + x41));
	const GEN_FLT x83 = x82 * x38;
	const GEN_FLT x84 = (x52 * x36) + (x76 * x31) + (x80 * x20) + lh_py;
	const GEN_FLT x85 = x37 * x37;
	const GEN_FLT x86 = x40 * x24;
	const GEN_FLT x87 = x46 * x21;
	const GEN_FLT x88 = (x42 * x47) + (x44 * x25) + (x3 * x69 * x22);
	const GEN_FLT x89 = (x20 * ((x4 * x69) + (2 * x42 * x25) + x70)) + (x75 * x27) + (x5 * x81) + (x68 * x34) +
						(x31 * (x79 + x78 + x77)) + (x36 * (x88 + (-1 * x87) + (-1 * x86)));
	const GEN_FLT x90 = x89 * (1. / x85);
	const GEN_FLT x91 = x84 * x90;
	const GEN_FLT x92 = -1 * x37;
	const GEN_FLT x93 = atan2(x84, x92);
	const GEN_FLT x94 = x84 * x84;
	const GEN_FLT x95 = x85 + x94;
	const GEN_FLT x96 = 1. / x95;
	const GEN_FLT x97 = x85 * x96;
	const GEN_FLT x98 = 2 * (x91 + (-1 * x83)) * x93 * x97 * curve_0;
	const GEN_FLT x99 = x33 + x32;
	const GEN_FLT x100 = x51 + (-1 * x50);
	const GEN_FLT x101 = x22 * x22;
	const GEN_FLT x102 = x1 + (x2 * x101);
	const GEN_FLT x103 = (x36 * x102) + (x31 * x100) + (x99 * x20) + lh_px;
	const GEN_FLT x104 = (x81 * x99) + (x20 * (x88 + x87 + x86)) + (x75 * x100) +
						 (x31 * (x49 + (-1 * x43) + (-1 * x41))) + (x68 * x102) +
						 (x36 * ((2 * x44 * x47) + (x69 * x101) + x70));
	const GEN_FLT x105 = x103 * x103;
	const GEN_FLT x106 = x85 + x105;
	const GEN_FLT x107 = 1. / x106;
	const GEN_FLT x108 = ((-1 * x38 * x104) + (x90 * x103)) * x85 * x107;
	const GEN_FLT x109 = -1 * x108;
	const GEN_FLT x110 = 1. / sqrt(1 + (-1 * x94 * x107 * (tilt_0 * tilt_0)));
	const GEN_FLT x111 = 2 * x89 * x37;
	const GEN_FLT x112 = 1. / sqrt(x106);
	const GEN_FLT x113 =
		(x82 * x112 * tilt_0) + (-1.0 / 2.0 * x84 * (1. / (x106 * sqrt(x106))) * (x111 + (2 * x103 * x104)) * tilt_0);
	const GEN_FLT x114 = (-1 * x110 * x113) + x109;
	const GEN_FLT x115 = -1 + x114;
	const GEN_FLT x116 = x84 * x112;
	const GEN_FLT x117 = atan2(x103, x92);
	const GEN_FLT x118 = 1.5707963267949 + (-1 * x117) + (-1 * phase_0) + (-1 * asin(x116 * tilt_0)) + gibPhase_0;
	const GEN_FLT x119 = sin(x118) * gibMag_0;
	const GEN_FLT x120 = (-1 * (x113 + x116) * x110) + x109;
	const GEN_FLT x121 = x114 + x98;
	const GEN_FLT x122 = x121 + (x119 * x114);
	const GEN_FLT x123 = -1 * ((-1 * x91) + x83) * x97;
	const GEN_FLT x124 = 1. / sqrt(x95);
	const GEN_FLT x125 =
		(x104 * x124 * tilt_1) + (-1.0 / 2.0 * (1. / (x95 * sqrt(x95))) * x103 * tilt_1 * (x111 + (2 * x82 * x84)));
	const GEN_FLT x126 = 1. / sqrt(1 + (-1 * x96 * x105 * (tilt_1 * tilt_1)));
	const GEN_FLT x127 = (-1 * x126 * x125) + x123;
	const GEN_FLT x128 = x103 * x124;
	const GEN_FLT x129 =
		1.5707963267949 + (-1 * atan2(-1 * x84, x92)) + (-1 * phase_1) + (-1 * asin(x128 * tilt_1)) + gibPhase_1;
	const GEN_FLT x130 = sin(x129) * gibMag_1;
	const GEN_FLT x131 = 2 * x108 * x117 * curve_1;
	const GEN_FLT x132 = x127 + x131;
	const GEN_FLT x133 = x132 + (x127 * x130);
	const GEN_FLT x134 = -1 + x127;
	const GEN_FLT x135 = (-1 * (x125 + x128) * x126) + x123;
	out[0] = x115 + (x119 * x115) + x98;
	out[1] = x120 + (x119 * x120) + x98;
	out[2] = x122 + (x93 * x93);
	out[3] = x121 + (x119 * (1 + x114));
	out[4] = x122 + (-1 * cos(x118));
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
	out[21] = x134 + (x130 * x134) + x131;
	out[22] = x135 + (x130 * x135) + x131;
	out[23] = x133 + (x117 * x117);
	out[24] = x132 + (x130 * (1 + x127));
	out[25] = x133 + (-1 * cos(x129));
	out[26] = x133;
	out[27] = x133;
}

/** Applying function <function reproject_axis_x at 0x7f6253d38290> */
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x11 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x12 = cos(x11);
	const GEN_FLT x13 = 1 + (-1 * x12);
	const GEN_FLT x14 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x15 = sin(x11);
	const GEN_FLT x16 = x15 * x14;
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x18 = x13 * x17;
	const GEN_FLT x19 = x10 * x18;
	const GEN_FLT x20 = x15 * x17;
	const GEN_FLT x21 = x14 * x13 * x10;
	const GEN_FLT x22 =
		((x21 + (-1 * x20)) * sensor_x) + ((x19 + x16) * sensor_y) + ((x12 + (x13 * (x10 * x10))) * sensor_z) + obj_pz;
	const GEN_FLT x23 = x15 * x10;
	const GEN_FLT x24 = x14 * x18;
	const GEN_FLT x25 =
		((x24 + x23) * sensor_x) + ((x12 + (x13 * (x17 * x17))) * sensor_y) + ((x19 + (-1 * x16)) * sensor_z) + obj_py;
	const GEN_FLT x26 = x1 * x4;
	const GEN_FLT x27 = x2 * x8;
	const GEN_FLT x28 =
		((x12 + ((x14 * x14) * x13)) * sensor_x) + ((x24 + (-1 * x23)) * sensor_y) + ((x21 + x20) * sensor_z) + obj_px;
	const GEN_FLT x29 = ((x27 + x26) * x28) + ((x9 + (-1 * x3)) * x22) + (x25 * (x5 + (x6 * (x7 * x7)))) + lh_py;
	const GEN_FLT x30 = x1 * x7;
	const GEN_FLT x31 = x2 * x4 * x6;
	const GEN_FLT x32 = ((x31 + (-1 * x30)) * x28) + ((x9 + x3) * x25) + (x22 * (x5 + ((x4 * x4) * x6))) + lh_pz;
	const GEN_FLT x33 = -1 * x32;
	const GEN_FLT x34 = (x28 * (x5 + ((x2 * x2) * x6))) + ((x27 + (-1 * x26)) * x25) + ((x31 + x30) * x22) + lh_px;
	const GEN_FLT x35 =
		(-1 * atan2(x34, x33)) + (-1 * phase_0) + (-1 * asin((1. / sqrt((x34 * x34) + (x32 * x32))) * x29 * tilt_0));
	out[0] = x35 + (-1 * cos(1.5707963267949 + x35 + gibPhase_0) * gibMag_0) +
			 ((atan2(x29, x33) * atan2(x29, x33)) * curve_0);
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
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (-1 * obj_qk *
							  (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										  : 1e-10) *
									 ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										  : 1e-10))) *
							  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
						   : 0;
	const GEN_FLT x1 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						   : 1e-10;
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 0;
	const GEN_FLT x5 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x6 = cos(x1);
	const GEN_FLT x7 = x6 * x5;
	const GEN_FLT x8 = x4 * x7;
	const GEN_FLT x9 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (-1 * obj_qj *
							  (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										  : 1e-10) *
									 ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										  : 1e-10))) *
							  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
						   : 0;
	const GEN_FLT x10 = 1 + (-1 * x6);
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x12 = x11 * x10;
	const GEN_FLT x13 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x14 = x2 * x11;
	const GEN_FLT x15 = x14 * x13;
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x17 = x13 * x10;
	const GEN_FLT x18 = (x17 * x16) + (x5 * x15) + (x9 * x12);
	const GEN_FLT x19 = x2 * x5;
	const GEN_FLT x20 = -1 * x19;
	const GEN_FLT x21 = 2 * x17;
	const GEN_FLT x22 = x13 * x13;
	const GEN_FLT x23 = x2 * x16;
	const GEN_FLT x24 = x7 * x11;
	const GEN_FLT x25 = x4 * x10;
	const GEN_FLT x26 = x4 * x13;
	const GEN_FLT x27 = (x26 * x19) + (x0 * x17) + (x9 * x25);
	const GEN_FLT x28 = ((x27 + (-1 * x24) + (-1 * x23)) * sensor_z) + (((x22 * x19) + (x9 * x21) + x20) * sensor_y) +
						((x18 + x8 + x3) * sensor_x);
	const GEN_FLT x29 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
							: 1e-10;
	const GEN_FLT x30 = sin(x29);
	const GEN_FLT x31 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 1;
	const GEN_FLT x32 = x30 * x31;
	const GEN_FLT x33 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 0;
	const GEN_FLT x34 = cos(x29);
	const GEN_FLT x35 = 1 + (-1 * x34);
	const GEN_FLT x36 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 0;
	const GEN_FLT x37 = x36 * x35;
	const GEN_FLT x38 = x33 * x37;
	const GEN_FLT x39 = x38 + x32;
	const GEN_FLT x40 = x39 * x28;
	const GEN_FLT x41 = x33 * x33;
	const GEN_FLT x42 = x34 + (x41 * x35);
	const GEN_FLT x43 = x2 * x9;
	const GEN_FLT x44 = x7 * x13;
	const GEN_FLT x45 = x4 * x14;
	const GEN_FLT x46 = (x5 * x45) + (x0 * x12) + (x25 * x16);
	const GEN_FLT x47 = 2 * x25;
	const GEN_FLT x48 = x4 * x4;
	const GEN_FLT x49 = ((x27 + x24 + x23) * sensor_y) + (((x48 * x19) + (x0 * x47) + x20) * sensor_z) +
						((x46 + (-1 * x44) + (-1 * x43)) * sensor_x);
	const GEN_FLT x50 = x42 * x49;
	const GEN_FLT x51 = x30 * x36;
	const GEN_FLT x52 = x33 * x35;
	const GEN_FLT x53 = x52 * x31;
	const GEN_FLT x54 = x53 + (-1 * x51);
	const GEN_FLT x55 = 2 * x12;
	const GEN_FLT x56 = x11 * x11;
	const GEN_FLT x57 = ((x46 + x44 + x43) * sensor_z) + ((x18 + (-1 * x8) + (-1 * x3)) * sensor_y) +
						(((x56 * x19) + x20 + (x55 * x16)) * sensor_x);
	const GEN_FLT x58 = 1 + x57;
	const GEN_FLT x59 = x2 * x13;
	const GEN_FLT x60 = x25 * x11;
	const GEN_FLT x61 = x2 * x4;
	const GEN_FLT x62 = x11 * x17;
	const GEN_FLT x63 =
		((x62 + (-1 * x61)) * sensor_y) + ((x60 + x59) * sensor_z) + ((x6 + (x56 * x10)) * sensor_x) + obj_px;
	const GEN_FLT x64 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x65 = x64 * x34;
	const GEN_FLT x66 = x65 * x36;
	const GEN_FLT x67 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x68 = x67 * x30;
	const GEN_FLT x69 = x64 * x30;
	const GEN_FLT x70 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x71 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x72 = x31 * x35;
	const GEN_FLT x73 = (x71 * x72) + (x70 * x52) + (x69 * x31 * x33);
	const GEN_FLT x74 = x65 * x31;
	const GEN_FLT x75 = x70 * x30;
	const GEN_FLT x76 = x69 * x36;
	const GEN_FLT x77 = (x67 * x52) + (x71 * x37) + (x76 * x33);
	const GEN_FLT x78 = x25 * x13;
	const GEN_FLT x79 =
		((x62 + x61) * sensor_x) + ((x78 + (-1 * x14)) * sensor_z) + ((x6 + (x22 * x10)) * sensor_y) + obj_py;
	const GEN_FLT x80 =
		((x60 + (-1 * x59)) * sensor_x) + ((x78 + x14) * sensor_y) + ((x6 + (x48 * x10)) * sensor_z) + obj_pz;
	const GEN_FLT x81 = -1 * x69;
	const GEN_FLT x82 = (x80 * ((2 * x71 * x52) + (x69 * x41) + x81)) + (x79 * (x77 + x75 + x74)) +
						(x63 * (x73 + (-1 * x68) + (-1 * x66)));
	const GEN_FLT x83 = x82 + (x54 * x58) + x50 + x40;
	const GEN_FLT x84 = x53 + x51;
	const GEN_FLT x85 = x30 * x33;
	const GEN_FLT x86 = x31 * x37;
	const GEN_FLT x87 = x86 + (-1 * x85);
	const GEN_FLT x88 = x31 * x31;
	const GEN_FLT x89 = x34 + (x88 * x35);
	const GEN_FLT x90 = (x89 * x63) + (x87 * x79) + (x80 * x84) + lh_px;
	const GEN_FLT x91 = (x63 * x54) + (x79 * x39) + (x80 * x42) + lh_pz;
	const GEN_FLT x92 = x91 * x91;
	const GEN_FLT x93 = 1. / x92;
	const GEN_FLT x94 = x93 * x90;
	const GEN_FLT x95 = 1. / x91;
	const GEN_FLT x96 = x84 * x49;
	const GEN_FLT x97 = x65 * x33;
	const GEN_FLT x98 = x71 * x30;
	const GEN_FLT x99 = (x76 * x31) + (x72 * x67) + (x70 * x37);
	const GEN_FLT x100 = (x80 * (x73 + x68 + x66)) + (x79 * (x99 + (-1 * x98) + (-1 * x97))) +
						 (x63 * ((2 * x70 * x72) + (x88 * x69) + x81));
	const GEN_FLT x101 = x100 + (x87 * x28);
	const GEN_FLT x102 = x101 + x96 + (x89 * x58);
	const GEN_FLT x103 = x92 + (x90 * x90);
	const GEN_FLT x104 = 1. / x103;
	const GEN_FLT x105 = x92 * x104;
	const GEN_FLT x106 = x38 + (-1 * x32);
	const GEN_FLT x107 = x36 * x36;
	const GEN_FLT x108 = x34 + (x35 * x107);
	const GEN_FLT x109 = x86 + x85;
	const GEN_FLT x110 = (x63 * x109) + (x79 * x108) + (x80 * x106) + lh_py;
	const GEN_FLT x111 = x110 * x110;
	const GEN_FLT x112 = 1. / sqrt(1 + (-1 * x104 * x111 * (tilt_0 * tilt_0)));
	const GEN_FLT x113 = 2 * x90;
	const GEN_FLT x114 = 2 * x91;
	const GEN_FLT x115 = 1.0 / 2.0 * (1. / (x103 * sqrt(x103))) * x110 * tilt_0;
	const GEN_FLT x116 = x28 * x108;
	const GEN_FLT x117 = (x80 * (x77 + (-1 * x75) + (-1 * x74))) + (x63 * (x99 + x98 + x97)) +
						 (x79 * ((2 * x67 * x37) + (x69 * x107) + x81));
	const GEN_FLT x118 = x117 + (x49 * x106);
	const GEN_FLT x119 = x118 + x116 + (x58 * x109);
	const GEN_FLT x120 = (1. / sqrt(x103)) * tilt_0;
	const GEN_FLT x121 = (-1 * x112 * ((x119 * x120) + (-1 * x115 * ((x83 * x114) + (x102 * x113))))) +
						 (-1 * x105 * ((-1 * x95 * x102) + (x83 * x94)));
	const GEN_FLT x122 = -1 * x91;
	const GEN_FLT x123 =
		sin(1.5707963267949 + (-1 * atan2(x90, x122)) + (-1 * phase_0) + (-1 * asin(x110 * x120)) + gibPhase_0) *
		gibMag_0;
	const GEN_FLT x124 = x93 * x110;
	const GEN_FLT x125 = 2 * x92 * (1. / (x92 + x111)) * atan2(x110, x122) * curve_0;
	const GEN_FLT x126 = 1 + x28;
	const GEN_FLT x127 = x82 + (x54 * x57);
	const GEN_FLT x128 = x127 + x50 + (x39 * x126);
	const GEN_FLT x129 = x89 * x57;
	const GEN_FLT x130 = x100 + (x87 * x126) + x129 + x96;
	const GEN_FLT x131 = x57 * x109;
	const GEN_FLT x132 = x118 + (x108 * x126) + x131;
	const GEN_FLT x133 = (-1 * x112 * ((x120 * x132) + (-1 * ((x114 * x128) + (x113 * x130)) * x115))) +
						 (-1 * ((-1 * x95 * x130) + (x94 * x128)) * x105);
	const GEN_FLT x134 = 1 + x49;
	const GEN_FLT x135 = x127 + (x42 * x134) + x40;
	const GEN_FLT x136 = x101 + (x84 * x134) + x129;
	const GEN_FLT x137 = x117 + (x106 * x134) + x116 + x131;
	const GEN_FLT x138 = (-1 * x112 * ((x120 * x137) + (-1 * ((x114 * x135) + (x113 * x136)) * x115))) +
						 (-1 * ((-1 * x95 * x136) + (x94 * x135)) * x105);
	const GEN_FLT x139 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x140 = x2 * x139;
	const GEN_FLT x141 = -1 * x140;
	const GEN_FLT x142 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qi *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x143 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qk *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x144 = x2 * x143;
	const GEN_FLT x145 = x4 * x6;
	const GEN_FLT x146 = x139 * x145;
	const GEN_FLT x147 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qj *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x148 = x13 * x11;
	const GEN_FLT x149 = (x140 * x148) + (x12 * x147) + (x17 * x142);
	const GEN_FLT x150 = x2 * x147;
	const GEN_FLT x151 = x6 * x13;
	const GEN_FLT x152 = x139 * x151;
	const GEN_FLT x153 = x4 * x11;
	const GEN_FLT x154 = (x140 * x153) + (x12 * x143) + (x25 * x142);
	const GEN_FLT x155 = ((x154 + x152 + x150) * sensor_z) + ((x149 + (-1 * x146) + (-1 * x144)) * sensor_y) +
						 (((x56 * x140) + (x55 * x142) + x141) * sensor_x);
	const GEN_FLT x156 = x2 * x142;
	const GEN_FLT x157 = x6 * x11;
	const GEN_FLT x158 = x139 * x157;
	const GEN_FLT x159 = (x26 * x140) + (x17 * x143) + (x25 * x147);
	const GEN_FLT x160 = ((x159 + (-1 * x158) + (-1 * x156)) * sensor_z) +
						 (((x22 * x140) + x141 + (x21 * x147)) * sensor_y) + ((x149 + x146 + x144) * sensor_x);
	const GEN_FLT x161 = (((x48 * x140) + (x47 * x143) + x141) * sensor_z) + ((x159 + x158 + x156) * sensor_y) +
						 ((x154 + (-1 * x152) + (-1 * x150)) * sensor_x);
	const GEN_FLT x162 = x82 + (x39 * x160) + (x42 * x161) + (x54 * x155);
	const GEN_FLT x163 = x100 + (x84 * x161) + (x87 * x160) + (x89 * x155);
	const GEN_FLT x164 = x117 + (x106 * x161) + (x108 * x160) + (x109 * x155);
	const GEN_FLT x165 = (-1 * x112 * ((x120 * x164) + (-1 * ((x114 * x162) + (x113 * x163)) * x115))) +
						 (-1 * ((-1 * x95 * x163) + (x94 * x162)) * x105);
	const GEN_FLT x166 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x167 = x2 * x166;
	const GEN_FLT x168 = -1 * x167;
	const GEN_FLT x169 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qi *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x170 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qk *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x171 = x2 * x170;
	const GEN_FLT x172 = x166 * x145;
	const GEN_FLT x173 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qj *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x174 = (x167 * x148) + (x12 * x173) + (x17 * x169);
	const GEN_FLT x175 = x2 * x173;
	const GEN_FLT x176 = x166 * x151;
	const GEN_FLT x177 = (x12 * x170) + (x167 * x153) + (x25 * x169);
	const GEN_FLT x178 = ((x177 + x176 + x175) * sensor_z) + ((x174 + (-1 * x172) + (-1 * x171)) * sensor_y) +
						 (((x56 * x167) + (x55 * x169) + x168) * sensor_x);
	const GEN_FLT x179 = x2 * x169;
	const GEN_FLT x180 = x166 * x157;
	const GEN_FLT x181 = (x25 * x173) + (x26 * x167) + (x17 * x170);
	const GEN_FLT x182 = ((x181 + (-1 * x180) + (-1 * x179)) * sensor_z) + ((x174 + x172 + x171) * sensor_x) +
						 (((x22 * x167) + (x21 * x173) + x168) * sensor_y);
	const GEN_FLT x183 = (((x48 * x167) + (x47 * x170) + x168) * sensor_z) + ((x181 + x180 + x179) * sensor_y) +
						 ((x177 + (-1 * x176) + (-1 * x175)) * sensor_x);
	const GEN_FLT x184 = (x42 * x183) + x82 + (x39 * x182) + (x54 * x178);
	const GEN_FLT x185 = (x84 * x183) + x100 + (x87 * x182) + (x89 * x178);
	const GEN_FLT x186 = x117 + (x106 * x183) + (x108 * x182) + (x109 * x178);
	const GEN_FLT x187 = (-1 * x112 * ((x120 * x186) + (-1 * ((x114 * x184) + (x113 * x185)) * x115))) +
						 (-1 * ((-1 * x95 * x185) + (x94 * x184)) * x105);
	const GEN_FLT x188 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x189 = x2 * x188;
	const GEN_FLT x190 = -1 * x189;
	const GEN_FLT x191 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qi *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x192 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qk *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x193 = x2 * x192;
	const GEN_FLT x194 = x188 * x145;
	const GEN_FLT x195 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qj *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x196 = (x15 * x188) + (x12 * x195) + (x17 * x191);
	const GEN_FLT x197 = x2 * x195;
	const GEN_FLT x198 = x188 * x151;
	const GEN_FLT x199 = (x12 * x192) + (x45 * x188) + (x25 * x191);
	const GEN_FLT x200 = ((x199 + x198 + x197) * sensor_z) + ((x196 + (-1 * x194) + (-1 * x193)) * sensor_y) +
						 (((x56 * x189) + (x55 * x191) + x190) * sensor_x);
	const GEN_FLT x201 = x2 * x191;
	const GEN_FLT x202 = x188 * x157;
	const GEN_FLT x203 = (x26 * x189) + (x17 * x192) + (x25 * x195);
	const GEN_FLT x204 = ((x203 + (-1 * x202) + (-1 * x201)) * sensor_z) +
						 (((x22 * x189) + x190 + (x21 * x195)) * sensor_y) + ((x196 + x194 + x193) * sensor_x);
	const GEN_FLT x205 = (((x48 * x189) + (x47 * x192) + x190) * sensor_z) + ((x203 + x202 + x201) * sensor_y) +
						 ((x199 + (-1 * x198) + (-1 * x197)) * sensor_x);
	const GEN_FLT x206 = x82 + (x39 * x204) + (x42 * x205) + (x54 * x200);
	const GEN_FLT x207 = x100 + (x84 * x205) + (x87 * x204) + (x89 * x200);
	const GEN_FLT x208 = x117 + (x205 * x106) + (x204 * x108) + (x200 * x109);
	const GEN_FLT x209 = (-1 * x112 * ((x208 * x120) + (-1 * ((x206 * x114) + (x207 * x113)) * x115))) +
						 (-1 * ((-1 * x95 * x207) + (x94 * x206)) * x105);
	out[0] = x121 + (((x83 * x124) + (-1 * x95 * x119)) * x125) + (x123 * x121);
	out[1] = x133 + (x125 * ((x124 * x128) + (-1 * x95 * x132))) + (x123 * x133);
	out[2] = x138 + (x125 * ((x124 * x135) + (-1 * x95 * x137))) + (x123 * x138);
	out[3] = x165 + (x125 * ((x124 * x162) + (-1 * x95 * x164))) + (x123 * x165);
	out[4] = x187 + (x125 * ((x124 * x184) + (-1 * x95 * x186))) + (x123 * x187);
	out[5] = (x125 * ((x206 * x124) + (-1 * x95 * x208))) + x209 + (x209 * x123);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x9 + x3;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = x14 + (x15 * x12);
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x18 = sin(x13);
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x21 * x11;
	const GEN_FLT x23 = x22 + x19;
	const GEN_FLT x24 = x20 * x18;
	const GEN_FLT x25 = x15 * x17;
	const GEN_FLT x26 = x25 * x11;
	const GEN_FLT x27 = x26 + (-1 * x24);
	const GEN_FLT x28 = (x23 * sensor_y) + (x16 * sensor_z) + (x27 * sensor_x) + obj_pz;
	const GEN_FLT x29 = x1 * x7;
	const GEN_FLT x30 = x2 * x6;
	const GEN_FLT x31 = x4 * x30;
	const GEN_FLT x32 = x31 + (-1 * x29);
	const GEN_FLT x33 = x22 + (-1 * x19);
	const GEN_FLT x34 = x20 * x20;
	const GEN_FLT x35 = x14 + (x34 * x15);
	const GEN_FLT x36 = x11 * x18;
	const GEN_FLT x37 = x21 * x17;
	const GEN_FLT x38 = x37 + x36;
	const GEN_FLT x39 = (x38 * sensor_x) + (x35 * sensor_y) + (x33 * sensor_z) + obj_py;
	const GEN_FLT x40 = x4 * x4;
	const GEN_FLT x41 = x5 + (x6 * x40);
	const GEN_FLT x42 = x26 + x24;
	const GEN_FLT x43 = x37 + (-1 * x36);
	const GEN_FLT x44 = x17 * x17;
	const GEN_FLT x45 = x14 + (x44 * x15);
	const GEN_FLT x46 = (x45 * sensor_x) + (x43 * sensor_y) + (x42 * sensor_z) + obj_px;
	const GEN_FLT x47 = (x41 * x46) + (x32 * x39) + (x28 * x10) + lh_px;
	const GEN_FLT x48 = x7 * x7;
	const GEN_FLT x49 = x5 + (x6 * x48);
	const GEN_FLT x50 = x1 * x4;
	const GEN_FLT x51 = x2 * x8;
	const GEN_FLT x52 = x51 + x50;
	const GEN_FLT x53 = x9 + (-1 * x3);
	const GEN_FLT x54 = (x53 * x46) + (x52 * x39) + (x49 * x28) + lh_pz;
	const GEN_FLT x55 = x54 * x54;
	const GEN_FLT x56 = 1. / x55;
	const GEN_FLT x57 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x58 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x59 = x58 * x18;
	const GEN_FLT x60 = -1 * x59;
	const GEN_FLT x61 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x62 = x61 * x18;
	const GEN_FLT x63 = x58 * x14;
	const GEN_FLT x64 = x63 * x11;
	const GEN_FLT x65 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x66 = x58 * x19;
	const GEN_FLT x67 = (x57 * x21) + (x66 * x20) + (x65 * x25);
	const GEN_FLT x68 = x65 * x18;
	const GEN_FLT x69 = x63 * x20;
	const GEN_FLT x70 = x15 * x11;
	const GEN_FLT x71 = (x66 * x11) + (x61 * x25) + (x70 * x57);
	const GEN_FLT x72 = ((x71 + x69 + x68) * sensor_z) + ((x67 + (-1 * x64) + (-1 * x62)) * sensor_y) +
						(((x59 * x44) + x60 + (2 * x57 * x25)) * sensor_x);
	const GEN_FLT x73 = x72 + x45;
	const GEN_FLT x74 = x57 * x18;
	const GEN_FLT x75 = x63 * x17;
	const GEN_FLT x76 = (x59 * x20 * x11) + (x61 * x21) + (x70 * x65);
	const GEN_FLT x77 = ((x76 + (-1 * x75) + (-1 * x74)) * sensor_z) +
						(((x59 * x34) + (2 * x65 * x21) + x60) * sensor_y) + ((x67 + x64 + x62) * sensor_x);
	const GEN_FLT x78 = x77 + x38;
	const GEN_FLT x79 = (((x59 * x12) + (2 * x70 * x61) + x60) * sensor_z) + ((x76 + x75 + x74) * sensor_y) +
						((x71 + (-1 * x69) + (-1 * x68)) * sensor_x);
	const GEN_FLT x80 = x79 + x27;
	const GEN_FLT x81 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x82 = x5 * x81;
	const GEN_FLT x83 = x2 * x82;
	const GEN_FLT x84 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x85 = x1 * x84;
	const GEN_FLT x86 = x1 * x81;
	const GEN_FLT x87 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x88 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x89 = x4 * x6;
	const GEN_FLT x90 = (x88 * x89) + (x8 * x87) + (x4 * x7 * x86);
	const GEN_FLT x91 = x4 * x82;
	const GEN_FLT x92 = x1 * x87;
	const GEN_FLT x93 = x2 * x86;
	const GEN_FLT x94 = (x8 * x84) + (x88 * x30) + (x7 * x93);
	const GEN_FLT x95 = -1 * x86;
	const GEN_FLT x96 = (x39 * (x94 + x92 + x91)) + (x28 * ((2 * x8 * x88) + (x86 * x48) + x95)) +
						(x46 * (x90 + (-1 * x85) + (-1 * x83)));
	const GEN_FLT x97 = x96 + (x80 * x49) + (x78 * x52) + (x73 * x53);
	const GEN_FLT x98 = x56 * x97;
	const GEN_FLT x99 = 1. / x54;
	const GEN_FLT x100 = x7 * x82;
	const GEN_FLT x101 = x1 * x88;
	const GEN_FLT x102 = (x4 * x93) + (x89 * x84) + (x87 * x30);
	const GEN_FLT x103 = (x28 * (x90 + x85 + x83)) + (x39 * (x102 + (-1 * x101) + (-1 * x100))) +
						 (x46 * ((2 * x89 * x87) + (x86 * x40) + x95));
	const GEN_FLT x104 = x103 + (x80 * x10) + (x78 * x32) + (x73 * x41);
	const GEN_FLT x105 = x55 + (x47 * x47);
	const GEN_FLT x106 = 1. / x105;
	const GEN_FLT x107 = x55 * x106;
	const GEN_FLT x108 = x51 + (-1 * x50);
	const GEN_FLT x109 = x2 * x2;
	const GEN_FLT x110 = x5 + (x6 * x109);
	const GEN_FLT x111 = x31 + x29;
	const GEN_FLT x112 = (x46 * x111) + (x39 * x110) + (x28 * x108) + lh_py;
	const GEN_FLT x113 = x112 * x112;
	const GEN_FLT x114 = 1. / sqrt(1 + (-1 * x106 * x113 * (tilt_0 * tilt_0)));
	const GEN_FLT x115 = 2 * x47;
	const GEN_FLT x116 = 2 * x54;
	const GEN_FLT x117 = 1.0 / 2.0 * (1. / (x105 * sqrt(x105))) * x112 * tilt_0;
	const GEN_FLT x118 = (x39 * ((2 * x84 * x30) + (x86 * x109) + x95)) + (x28 * (x94 + (-1 * x92) + (-1 * x91))) +
						 (x46 * (x102 + x101 + x100));
	const GEN_FLT x119 = x118 + (x80 * x108) + (x78 * x110) + (x73 * x111);
	const GEN_FLT x120 = (1. / sqrt(x105)) * tilt_0;
	const GEN_FLT x121 = (-1 * x114 * ((x119 * x120) + (-1 * x117 * ((x97 * x116) + (x104 * x115))))) +
						 (-1 * x107 * ((-1 * x99 * x104) + (x98 * x47)));
	const GEN_FLT x122 = -1 * x54;
	const GEN_FLT x123 =
		sin(1.5707963267949 + (-1 * atan2(x47, x122)) + (-1 * phase_0) + (-1 * asin(x112 * x120)) + gibPhase_0) *
		gibMag_0;
	const GEN_FLT x124 = 2 * x55 * (1. / (x55 + x113)) * atan2(x112, x122) * curve_0;
	const GEN_FLT x125 = x72 + x43;
	const GEN_FLT x126 = x77 + x35;
	const GEN_FLT x127 = x79 + x23;
	const GEN_FLT x128 = (x49 * x127) + x96 + (x52 * x126) + (x53 * x125);
	const GEN_FLT x129 = x56 * x128;
	const GEN_FLT x130 = x103 + (x10 * x127) + (x32 * x126) + (x41 * x125);
	const GEN_FLT x131 = (x108 * x127) + x118 + (x110 * x126) + (x111 * x125);
	const GEN_FLT x132 = (-1 * x114 * ((x120 * x131) + (-1 * ((x116 * x128) + (x115 * x130)) * x117))) +
						 (-1 * ((-1 * x99 * x130) + (x47 * x129)) * x107);
	const GEN_FLT x133 = x72 + x42;
	const GEN_FLT x134 = x77 + x33;
	const GEN_FLT x135 = x79 + x16;
	const GEN_FLT x136 = (x49 * x135) + (x52 * x134) + x96 + (x53 * x133);
	const GEN_FLT x137 = x56 * x136;
	const GEN_FLT x138 = x103 + (x32 * x134) + (x10 * x135) + (x41 * x133);
	const GEN_FLT x139 = (x108 * x135) + x118 + (x111 * x133) + (x110 * x134);
	const GEN_FLT x140 = (-1 * x114 * ((x120 * x139) + (-1 * ((x116 * x136) + (x115 * x138)) * x117))) +
						 (-1 * ((-1 * x99 * x138) + (x47 * x137)) * x107);
	out[0] = x121 + (((x98 * x112) + (-1 * x99 * x119)) * x124) + (x123 * x121);
	out[1] = x132 + (x124 * ((x112 * x129) + (-1 * x99 * x131))) + (x123 * x132);
	out[2] = (x124 * ((x112 * x137) + (-1 * x99 * x139))) + x140 + (x123 * x140);
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
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 0;
	const GEN_FLT x1 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						   : 1e-10;
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 0;
	const GEN_FLT x5 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
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
		((x6 + (x7 * x13)) * sensor_x) + ((x12 + (-1 * x10)) * sensor_y) + ((x9 + x3) * sensor_z) + obj_px;
	const GEN_FLT x15 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 0;
	const GEN_FLT x16 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x17 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
							: 1e-10;
	const GEN_FLT x18 = cos(x17);
	const GEN_FLT x19 = x18 * x16;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = sin(x17);
	const GEN_FLT x22 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 1;
	const GEN_FLT x25 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 0;
	const GEN_FLT x26 = x25 * x21;
	const GEN_FLT x27 = x24 * x26;
	const GEN_FLT x28 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x29 = 1 + (-1 * x18);
	const GEN_FLT x30 = x25 * x29;
	const GEN_FLT x31 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x32 = x24 * x29;
	const GEN_FLT x33 = (x32 * x31) + (x30 * x28) + (x27 * x16);
	const GEN_FLT x34 = x24 * x19;
	const GEN_FLT x35 = x21 * x28;
	const GEN_FLT x36 = x21 * x15;
	const GEN_FLT x37 = x36 * x25;
	const GEN_FLT x38 = x29 * x15;
	const GEN_FLT x39 = (x30 * x22) + (x31 * x38) + (x37 * x16);
	const GEN_FLT x40 = x2 * x5;
	const GEN_FLT x41 = x4 * x11;
	const GEN_FLT x42 = x0 * x0;
	const GEN_FLT x43 =
		((x6 + (x7 * x42)) * sensor_y) + ((x12 + x10) * sensor_x) + ((x41 + (-1 * x40)) * sensor_z) + obj_py;
	const GEN_FLT x44 = x4 * x4;
	const GEN_FLT x45 =
		((x9 + (-1 * x3)) * sensor_x) + ((x41 + x40) * sensor_y) + ((x6 + (x7 * x44)) * sensor_z) + obj_pz;
	const GEN_FLT x46 = x21 * x16;
	const GEN_FLT x47 = -1 * x46;
	const GEN_FLT x48 = x25 * x25;
	const GEN_FLT x49 = 2 * x30;
	const GEN_FLT x50 = x30 * x24;
	const GEN_FLT x51 = x50 + (-1 * x36);
	const GEN_FLT x52 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x53 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x54 = x2 * x53;
	const GEN_FLT x55 = -1 * x54;
	const GEN_FLT x56 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x57 = x2 * x56;
	const GEN_FLT x58 = x6 * x53;
	const GEN_FLT x59 = x4 * x58;
	const GEN_FLT x60 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x61 = x53 * x40;
	const GEN_FLT x62 = (x52 * x11) + (x0 * x61) + (x8 * x60);
	const GEN_FLT x63 = x2 * x60;
	const GEN_FLT x64 = x0 * x58;
	const GEN_FLT x65 = x4 * x7;
	const GEN_FLT x66 = (x8 * x56) + (x4 * x61) + (x65 * x52);
	const GEN_FLT x67 = ((x66 + x64 + x63) * sensor_z) + ((x62 + (-1 * x59) + (-1 * x57)) * sensor_y) +
						(((x54 * x13) + x55 + (2 * x8 * x52)) * sensor_x);
	const GEN_FLT x68 = x2 * x52;
	const GEN_FLT x69 = x5 * x58;
	const GEN_FLT x70 = (x0 * x53 * x10) + (x56 * x11) + (x60 * x65);
	const GEN_FLT x71 = (((-1 * x69) + x70 + (-1 * x68)) * sensor_z) +
						(((x54 * x42) + (2 * x60 * x11) + x55) * sensor_y) + ((x62 + x59 + x57) * sensor_x);
	const GEN_FLT x72 = x24 * x21;
	const GEN_FLT x73 = x30 * x15;
	const GEN_FLT x74 = x73 + x72;
	const GEN_FLT x75 = x18 + (x48 * x29);
	const GEN_FLT x76 = (((x54 * x44) + (2 * x65 * x56) + x55) * sensor_z) + ((x69 + x70 + x68) * sensor_y) +
						((x66 + (-1 * x64) + (-1 * x63)) * sensor_x);
	const GEN_FLT x77 = (x75 * x76) + (x71 * x74) + (x67 * x51);
	const GEN_FLT x78 = x77 + (x45 * ((x46 * x48) + (x49 * x31) + x47)) + (x43 * (x39 + x35 + x34)) +
						(x14 * (x33 + (-1 * x23) + (-1 * x20)));
	const GEN_FLT x79 = (x74 * x43) + (x51 * x14) + (x75 * x45) + lh_pz;
	const GEN_FLT x80 = x79 * x79;
	const GEN_FLT x81 = 1. / x80;
	const GEN_FLT x82 = x50 + x36;
	const GEN_FLT x83 = x38 * x24;
	const GEN_FLT x84 = x83 + (-1 * x26);
	const GEN_FLT x85 = x24 * x24;
	const GEN_FLT x86 = x18 + (x85 * x29);
	const GEN_FLT x87 = (x86 * x14) + (x84 * x43) + (x82 * x45) + lh_px;
	const GEN_FLT x88 = x81 * x87;
	const GEN_FLT x89 = x88 * x78;
	const GEN_FLT x90 = 1. / x79;
	const GEN_FLT x91 = 2 * x32;
	const GEN_FLT x92 = x25 * x18;
	const GEN_FLT x93 = x92 * x16;
	const GEN_FLT x94 = x31 * x21;
	const GEN_FLT x95 = x36 * x24;
	const GEN_FLT x96 = (x95 * x16) + (x32 * x22) + (x38 * x28);
	const GEN_FLT x97 = (x82 * x76) + (x84 * x71) + (x86 * x67);
	const GEN_FLT x98 = x97 + (x45 * (x33 + x23 + x20)) + (x43 * (x96 + (-1 * x94) + (-1 * x93))) +
						(x14 * ((x91 * x28) + (x85 * x46) + x47));
	const GEN_FLT x99 = 1 + x98;
	const GEN_FLT x100 = x80 + (x87 * x87);
	const GEN_FLT x101 = 1. / x100;
	const GEN_FLT x102 = x80 * x101;
	const GEN_FLT x103 = 2 * x87;
	const GEN_FLT x104 = 2 * x79;
	const GEN_FLT x105 = x78 * x104;
	const GEN_FLT x106 = x73 + (-1 * x72);
	const GEN_FLT x107 = x15 * x15;
	const GEN_FLT x108 = x18 + (x29 * x107);
	const GEN_FLT x109 = x83 + x26;
	const GEN_FLT x110 = (x43 * x108) + (x45 * x106) + (x14 * x109) + lh_py;
	const GEN_FLT x111 = 1.0 / 2.0 * (1. / (x100 * sqrt(x100))) * x110 * tilt_0;
	const GEN_FLT x112 = 2 * x38;
	const GEN_FLT x113 = (x76 * x106) + (x67 * x109) + (x71 * x108);
	const GEN_FLT x114 = x113 + (x43 * ((x22 * x112) + (x46 * x107) + x47)) + (x45 * (x39 + (-1 * x35) + (-1 * x34))) +
						 (x14 * (x96 + x94 + x93));
	const GEN_FLT x115 = (1. / sqrt(x100)) * tilt_0;
	const GEN_FLT x116 = x114 * x115;
	const GEN_FLT x117 = x110 * x110;
	const GEN_FLT x118 = 1. / sqrt(1 + (-1 * x101 * x117 * (tilt_0 * tilt_0)));
	const GEN_FLT x119 =
		(-1 * x118 * (x116 + (-1 * x111 * (x105 + (x99 * x103))))) + (-1 * x102 * ((-1 * x90 * x99) + x89));
	const GEN_FLT x120 = -1 * x79;
	const GEN_FLT x121 =
		sin(1.5707963267949 + (-1 * atan2(x87, x120)) + (-1 * phase_0) + (-1 * asin(x110 * x115)) + gibPhase_0) *
		gibMag_0;
	const GEN_FLT x122 = -1 * x90 * x114;
	const GEN_FLT x123 = x81 * x110;
	const GEN_FLT x124 = x78 * x123;
	const GEN_FLT x125 = 2 * x80 * (1. / (x80 + x117)) * atan2(x110, x120) * curve_0;
	const GEN_FLT x126 = -1 * x90 * x98;
	const GEN_FLT x127 = x98 * x103;
	const GEN_FLT x128 = 1 + x114;
	const GEN_FLT x129 = (-1 * x118 * ((x115 * x128) + (-1 * (x105 + x127) * x111))) + (-1 * x102 * (x126 + x89));
	const GEN_FLT x130 = 1 + x78;
	const GEN_FLT x131 =
		(-1 * x118 * (x116 + (-1 * x111 * ((x104 * x130) + x127)))) + (-1 * x102 * (x126 + (x88 * x130)));
	const GEN_FLT x132 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x133 = x18 * x132;
	const GEN_FLT x134 = x15 * x133;
	const GEN_FLT x135 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qj *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x136 = x21 * x135;
	const GEN_FLT x137 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qi *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x138 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qk *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x139 = (x27 * x132) + (x32 * x138) + (x30 * x137);
	const GEN_FLT x140 = x24 * x133;
	const GEN_FLT x141 = x21 * x137;
	const GEN_FLT x142 = x36 * x132;
	const GEN_FLT x143 = (x30 * x135) + (x38 * x138) + (x25 * x142);
	const GEN_FLT x144 = x21 * x132;
	const GEN_FLT x145 = -1 * x144;
	const GEN_FLT x146 = x77 + (x45 * ((x49 * x138) + (x48 * x144) + x145)) + (x43 * (x143 + x141 + x140)) +
						 (x14 * (x139 + (-1 * x136) + (-1 * x134)));
	const GEN_FLT x147 = x81 * x146;
	const GEN_FLT x148 = x92 * x132;
	const GEN_FLT x149 = x21 * x138;
	const GEN_FLT x150 = (x32 * x135) + (x38 * x137) + (x24 * x142);
	const GEN_FLT x151 = x97 + (x45 * (x139 + x136 + x134)) + (x43 * (x150 + (-1 * x149) + (-1 * x148))) +
						 (x14 * ((x91 * x137) + (x85 * x144) + x145));
	const GEN_FLT x152 = x113 + (x14 * (x150 + x149 + x148)) + (x45 * (x143 + (-1 * x141) + (-1 * x140))) +
						 (x43 * ((x112 * x135) + (x107 * x144) + x145));
	const GEN_FLT x153 = (-1 * x118 * ((x115 * x152) + (-1 * ((x104 * x146) + (x103 * x151)) * x111))) +
						 (-1 * ((-1 * x90 * x151) + (x87 * x147)) * x102);
	const GEN_FLT x154 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x155 = x15 * x18;
	const GEN_FLT x156 = x154 * x155;
	const GEN_FLT x157 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qj *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x158 = x21 * x157;
	const GEN_FLT x159 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qi *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x160 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qk *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x161 = (x30 * x159) + (x32 * x160) + (x27 * x154);
	const GEN_FLT x162 = x24 * x18;
	const GEN_FLT x163 = x162 * x154;
	const GEN_FLT x164 = x21 * x159;
	const GEN_FLT x165 = (x30 * x157) + (x38 * x160) + (x37 * x154);
	const GEN_FLT x166 = x21 * x154;
	const GEN_FLT x167 = -1 * x166;
	const GEN_FLT x168 = x77 + (x43 * (x165 + x164 + x163)) + (x45 * ((x49 * x160) + (x48 * x166) + x167)) +
						 (x14 * (x161 + (-1 * x158) + (-1 * x156)));
	const GEN_FLT x169 = x81 * x168;
	const GEN_FLT x170 = x92 * x154;
	const GEN_FLT x171 = x21 * x160;
	const GEN_FLT x172 = (x38 * x159) + (x32 * x157) + (x95 * x154);
	const GEN_FLT x173 = x97 + (x43 * (x172 + (-1 * x171) + (-1 * x170))) + (x45 * (x161 + x158 + x156)) +
						 (x14 * ((x91 * x159) + x167 + (x85 * x166)));
	const GEN_FLT x174 = x113 + (x45 * (x165 + (-1 * x164) + (-1 * x163))) +
						 (x43 * ((x112 * x157) + (x107 * x166) + x167)) + (x14 * (x172 + x171 + x170));
	const GEN_FLT x175 = (-1 * x118 * ((x115 * x174) + (-1 * ((x104 * x168) + (x103 * x173)) * x111))) +
						 (-1 * ((-1 * x90 * x173) + (x87 * x169)) * x102);
	const GEN_FLT x176 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x177 = x176 * x155;
	const GEN_FLT x178 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qj *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x179 = x21 * x178;
	const GEN_FLT x180 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qi *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x181 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qk *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x182 = (x32 * x181) + (x30 * x180) + (x27 * x176);
	const GEN_FLT x183 = x162 * x176;
	const GEN_FLT x184 = x21 * x180;
	const GEN_FLT x185 = (x30 * x178) + (x38 * x181) + (x37 * x176);
	const GEN_FLT x186 = x21 * x176;
	const GEN_FLT x187 = -1 * x186;
	const GEN_FLT x188 = x77 + (x45 * ((x48 * x186) + (x49 * x181) + x187)) + (x43 * (x185 + x184 + x183)) +
						 (x14 * ((-1 * x179) + x182 + (-1 * x177)));
	const GEN_FLT x189 = x92 * x176;
	const GEN_FLT x190 = x21 * x181;
	const GEN_FLT x191 = (x32 * x178) + (x38 * x180) + (x95 * x176);
	const GEN_FLT x192 = x97 + (x45 * (x179 + x182 + x177)) + (x43 * (x191 + (-1 * x190) + (-1 * x189))) +
						 (x14 * ((x91 * x180) + (x85 * x186) + x187));
	const GEN_FLT x193 = x113 + (x45 * (x185 + (-1 * x184) + (-1 * x183))) +
						 (x43 * ((x112 * x178) + (x107 * x186) + x187)) + (x14 * (x191 + x190 + x189));
	const GEN_FLT x194 = (-1 * x118 * ((x115 * x193) + (-1 * ((x104 * x188) + (x103 * x192)) * x111))) +
						 (-1 * ((-1 * x90 * x192) + (x88 * x188)) * x102);
	out[0] = x119 + ((x124 + x122) * x125) + (x119 * x121);
	out[1] = x129 + (x125 * (x124 + (-1 * x90 * x128))) + (x121 * x129);
	out[2] = x131 + (x125 * ((x123 * x130) + x122)) + (x121 * x131);
	out[3] = x153 + (x125 * ((x110 * x147) + (-1 * x90 * x152))) + (x121 * x153);
	out[4] = (x125 * ((x110 * x169) + (-1 * x90 * x174))) + x175 + (x121 * x175);
	out[5] = x194 + (x125 * ((x123 * x188) + (-1 * x90 * x193))) + (x121 * x194);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 + (-1 * x1);
	const GEN_FLT x3 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x4 = x3 * x3;
	const GEN_FLT x5 = x1 + (x2 * x4);
	const GEN_FLT x6 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 0;
	const GEN_FLT x7 = x6 * x6;
	const GEN_FLT x8 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						   : 1e-10;
	const GEN_FLT x9 = cos(x8);
	const GEN_FLT x10 = 1 + (-1 * x9);
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x12 = sin(x8);
	const GEN_FLT x13 = x12 * x11;
	const GEN_FLT x14 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x15 = x14 * x10;
	const GEN_FLT x16 = x6 * x15;
	const GEN_FLT x17 = x14 * x12;
	const GEN_FLT x18 = x6 * x10;
	const GEN_FLT x19 = x11 * x18;
	const GEN_FLT x20 =
		((x19 + (-1 * x17)) * sensor_x) + ((x16 + x13) * sensor_y) + ((x9 + (x7 * x10)) * sensor_z) + obj_pz;
	const GEN_FLT x21 = sin(x0);
	const GEN_FLT x22 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 1;
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 0;
	const GEN_FLT x25 = x2 * x3;
	const GEN_FLT x26 = x24 * x25;
	const GEN_FLT x27 = x26 + x23;
	const GEN_FLT x28 = x14 * x14;
	const GEN_FLT x29 = x6 * x12;
	const GEN_FLT x30 = x15 * x11;
	const GEN_FLT x31 =
		((x30 + x29) * sensor_x) + ((x16 + (-1 * x13)) * sensor_z) + ((x9 + (x28 * x10)) * sensor_y) + obj_py;
	const GEN_FLT x32 = x24 * x21;
	const GEN_FLT x33 = x25 * x22;
	const GEN_FLT x34 = x33 + (-1 * x32);
	const GEN_FLT x35 = x11 * x11;
	const GEN_FLT x36 =
		((x9 + (x35 * x10)) * sensor_x) + ((x30 + (-1 * x29)) * sensor_y) + ((x19 + x17) * sensor_z) + obj_px;
	const GEN_FLT x37 = (x34 * x36) + (x31 * x27) + (x5 * x20) + lh_pz;
	const GEN_FLT x38 = x37 * x37;
	const GEN_FLT x39 = x26 + (-1 * x23);
	const GEN_FLT x40 = x24 * x24;
	const GEN_FLT x41 = x1 + (x2 * x40);
	const GEN_FLT x42 = x3 * x21;
	const GEN_FLT x43 = x2 * x22;
	const GEN_FLT x44 = x43 * x24;
	const GEN_FLT x45 = x44 + x42;
	const GEN_FLT x46 = (x45 * x36) + (x41 * x31) + (x39 * x20) + lh_py;
	const GEN_FLT x47 = x46 * x46;
	const GEN_FLT x48 = 1. / x37;
	const GEN_FLT x49 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x50 = x1 * x49;
	const GEN_FLT x51 = x3 * x50;
	const GEN_FLT x52 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x53 = x52 * x21;
	const GEN_FLT x54 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x55 = x2 * x54;
	const GEN_FLT x56 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x57 = x49 * x24;
	const GEN_FLT x58 = (x57 * x23) + (x56 * x43) + (x55 * x24);
	const GEN_FLT x59 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x60 = x11 * x10;
	const GEN_FLT x61 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x62 = x61 * x12;
	const GEN_FLT x63 = -1 * x62;
	const GEN_FLT x64 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x65 = x64 * x12;
	const GEN_FLT x66 = x9 * x61;
	const GEN_FLT x67 = x6 * x66;
	const GEN_FLT x68 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x69 = x61 * x13;
	const GEN_FLT x70 = (x59 * x15) + (x69 * x14) + (x60 * x68);
	const GEN_FLT x71 = x68 * x12;
	const GEN_FLT x72 = x66 * x14;
	const GEN_FLT x73 = (x60 * x64) + (x6 * x69) + (x59 * x18);
	const GEN_FLT x74 = ((x73 + x72 + x71) * sensor_z) + ((x70 + (-1 * x67) + (-1 * x65)) * sensor_y) +
						(((x62 * x35) + x63 + (2 * x60 * x59)) * sensor_x);
	const GEN_FLT x75 = x49 * x21;
	const GEN_FLT x76 = -1 * x75;
	const GEN_FLT x77 = x2 * x24;
	const GEN_FLT x78 = x59 * x12;
	const GEN_FLT x79 = x66 * x11;
	const GEN_FLT x80 = (x61 * x29 * x14) + (x64 * x15) + (x68 * x18);
	const GEN_FLT x81 = ((x80 + (-1 * x79) + (-1 * x78)) * sensor_z) +
						(((x62 * x28) + (2 * x68 * x15) + x63) * sensor_y) + ((x70 + x67 + x65) * sensor_x);
	const GEN_FLT x82 = x50 * x22;
	const GEN_FLT x83 = x54 * x21;
	const GEN_FLT x84 = (x56 * x25) + (x77 * x52) + (x57 * x42);
	const GEN_FLT x85 = (((x7 * x62) + (2 * x64 * x18) + x63) * sensor_z) + ((x80 + x79 + x78) * sensor_y) +
						((x73 + (-1 * x72) + (-1 * x71)) * sensor_x);
	const GEN_FLT x86 = (x85 * x39) + (x20 * (x84 + (-1 * x83) + (-1 * x82))) + (x81 * x41) +
						(x31 * ((2 * x77 * x56) + (x75 * x40) + x76)) + (x74 * x45) + (x36 * (x58 + x53 + x51));
	const GEN_FLT x87 = x50 * x24;
	const GEN_FLT x88 = x56 * x21;
	const GEN_FLT x89 = (x52 * x43) + (x3 * x55) + (x3 * x49 * x23);
	const GEN_FLT x90 = (x5 * x85) + (x20 * ((2 * x52 * x25) + (x4 * x75) + x76)) + (x31 * (x84 + x83 + x82)) +
						(x74 * x34) + (x81 * x27) + (x36 * (x89 + (-1 * x88) + (-1 * x87)));
	const GEN_FLT x91 = x90 * (1. / x38);
	const GEN_FLT x92 = -1 * x37;
	const GEN_FLT x93 = atan2(x46, x92);
	const GEN_FLT x94 = 2 * (1. / (x38 + x47)) * ((x91 * x46) + (-1 * x86 * x48)) * x93 * x38 * curve_0;
	const GEN_FLT x95 = x33 + x32;
	const GEN_FLT x96 = x44 + (-1 * x42);
	const GEN_FLT x97 = x22 * x22;
	const GEN_FLT x98 = x1 + (x2 * x97);
	const GEN_FLT x99 = (x98 * x36) + (x96 * x31) + (x95 * x20) + lh_px;
	const GEN_FLT x100 = (x85 * x95) + (x20 * (x89 + x88 + x87)) + (x31 * (x58 + (-1 * x53) + (-1 * x51))) +
						 (x81 * x96) + (x74 * x98) + (x36 * ((2 * x55 * x22) + (x75 * x97) + x76));
	const GEN_FLT x101 = x38 + (x99 * x99);
	const GEN_FLT x102 = 1. / x101;
	const GEN_FLT x103 = -1 * x38 * x102 * ((-1 * x48 * x100) + (x91 * x99));
	const GEN_FLT x104 = 1. / sqrt(1 + (-1 * x47 * x102 * (tilt_0 * tilt_0)));
	const GEN_FLT x105 = 1. / sqrt(x101);
	const GEN_FLT x106 = (x86 * x105 * tilt_0) + (-1.0 / 2.0 * x46 * (1. / (x101 * sqrt(x101))) * tilt_0 *
												  ((2 * x90 * x37) + (2 * x99 * x100)));
	const GEN_FLT x107 = (-1 * x104 * x106) + x103;
	const GEN_FLT x108 = -1 + x107;
	const GEN_FLT x109 = x46 * x105;
	const GEN_FLT x110 =
		1.5707963267949 + (-1 * atan2(x99, x92)) + (-1 * phase_0) + (-1 * asin(x109 * tilt_0)) + gibPhase_0;
	const GEN_FLT x111 = sin(x110) * gibMag_0;
	const GEN_FLT x112 = (-1 * (x106 + x109) * x104) + x103;
	const GEN_FLT x113 = x107 + x94;
	const GEN_FLT x114 = x113 + (x107 * x111);
	out[0] = x108 + (x108 * x111) + x94;
	out[1] = x112 + (x111 * x112) + x94;
	out[2] = x114 + (x93 * x93);
	out[3] = x113 + (x111 * (1 + x107));
	out[4] = x114 + (-1 * cos(x110));
	out[5] = x114;
	out[6] = x114;
}

/** Applying function <function reproject_axis_y at 0x7f6253d38320> */
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = cos(x0);
	const GEN_FLT x5 = 1 + (-1 * x4);
	const GEN_FLT x6 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x7 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x8 = x6 * x5 * x7;
	const GEN_FLT x9 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 0;
	const GEN_FLT x10 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x11 = cos(x10);
	const GEN_FLT x12 = 1 + (-1 * x11);
	const GEN_FLT x13 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x14 = sin(x10);
	const GEN_FLT x15 = x14 * x13;
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x17 = x12 * x16;
	const GEN_FLT x18 = x9 * x17;
	const GEN_FLT x19 = x14 * x16;
	const GEN_FLT x20 = x9 * x13 * x12;
	const GEN_FLT x21 =
		((x20 + (-1 * x19)) * sensor_x) + ((x18 + x15) * sensor_y) + ((x11 + ((x9 * x9) * x12)) * sensor_z) + obj_pz;
	const GEN_FLT x22 = x1 * x7;
	const GEN_FLT x23 = x2 * x5;
	const GEN_FLT x24 = x6 * x23;
	const GEN_FLT x25 = x9 * x14;
	const GEN_FLT x26 = x13 * x17;
	const GEN_FLT x27 =
		((x26 + x25) * sensor_x) + ((x11 + (x12 * (x16 * x16))) * sensor_y) + ((x18 + (-1 * x15)) * sensor_z) + obj_py;
	const GEN_FLT x28 =
		((x26 + (-1 * x25)) * sensor_y) + ((x11 + ((x13 * x13) * x12)) * sensor_x) + ((x20 + x19) * sensor_z) + obj_px;
	const GEN_FLT x29 = (x28 * (x4 + ((x6 * x6) * x5))) + ((x24 + (-1 * x22)) * x27) + ((x8 + x3) * x21) + lh_px;
	const GEN_FLT x30 = x1 * x6;
	const GEN_FLT x31 = x7 * x23;
	const GEN_FLT x32 = ((x8 + (-1 * x3)) * x28) + (x21 * (x4 + (x5 * (x7 * x7)))) + ((x31 + x30) * x27) + lh_pz;
	const GEN_FLT x33 = ((x24 + x22) * x28) + (x27 * (x4 + ((x2 * x2) * x5))) + ((x31 + (-1 * x30)) * x21) + lh_py;
	const GEN_FLT x34 = -1 * x32;
	const GEN_FLT x35 = (-1 * atan2(-1 * x33, x34)) + (-1 * phase_1) +
						(-1 * asin((1. / sqrt((x33 * x33) + (x32 * x32))) * x29 * tilt_1));
	out[0] = x35 + ((atan2(x29, x34) * atan2(x29, x34)) * curve_1) +
			 (-1 * cos(1.5707963267949 + x35 + gibPhase_1) * gibMag_1);
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
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (-1 * obj_qk *
							  (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										  : 1e-10) *
									 ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										  : 1e-10))) *
							  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
						   : 0;
	const GEN_FLT x1 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						   : 1e-10;
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 0;
	const GEN_FLT x5 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x6 = cos(x1);
	const GEN_FLT x7 = x6 * x5;
	const GEN_FLT x8 = x4 * x7;
	const GEN_FLT x9 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (-1 * obj_qj *
							  (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										  : 1e-10) *
									 ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										  : 1e-10))) *
							  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
						   : 0;
	const GEN_FLT x10 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x11 = 1 + (-1 * x6);
	const GEN_FLT x12 = x11 * x10;
	const GEN_FLT x13 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x14 = x2 * x10;
	const GEN_FLT x15 = x14 * x13;
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x17 = x13 * x11;
	const GEN_FLT x18 = (x17 * x16) + (x5 * x15) + (x9 * x12);
	const GEN_FLT x19 = x2 * x5;
	const GEN_FLT x20 = -1 * x19;
	const GEN_FLT x21 = 2 * x17;
	const GEN_FLT x22 = x13 * x13;
	const GEN_FLT x23 = x2 * x16;
	const GEN_FLT x24 = x6 * x10;
	const GEN_FLT x25 = x5 * x24;
	const GEN_FLT x26 = x4 * x11;
	const GEN_FLT x27 = x2 * x4;
	const GEN_FLT x28 = (x5 * x27 * x13) + (x0 * x17) + (x9 * x26);
	const GEN_FLT x29 = ((x28 + (-1 * x25) + (-1 * x23)) * sensor_z) + (((x22 * x19) + (x9 * x21) + x20) * sensor_y) +
						((x18 + x8 + x3) * sensor_x);
	const GEN_FLT x30 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
							: 1e-10;
	const GEN_FLT x31 = sin(x30);
	const GEN_FLT x32 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 1;
	const GEN_FLT x33 = x32 * x31;
	const GEN_FLT x34 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 0;
	const GEN_FLT x35 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 0;
	const GEN_FLT x36 = cos(x30);
	const GEN_FLT x37 = 1 + (-1 * x36);
	const GEN_FLT x38 = x35 * x37;
	const GEN_FLT x39 = x34 * x38;
	const GEN_FLT x40 = x39 + x33;
	const GEN_FLT x41 = x40 * x29;
	const GEN_FLT x42 = x31 * x34;
	const GEN_FLT x43 = x32 * x37;
	const GEN_FLT x44 = x43 * x35;
	const GEN_FLT x45 = x44 + (-1 * x42);
	const GEN_FLT x46 = 2 * x12;
	const GEN_FLT x47 = x10 * x10;
	const GEN_FLT x48 = x2 * x9;
	const GEN_FLT x49 = x7 * x13;
	const GEN_FLT x50 = x4 * x14;
	const GEN_FLT x51 = (x0 * x12) + (x5 * x50) + (x26 * x16);
	const GEN_FLT x52 = ((x18 + (-1 * x8) + (-1 * x3)) * sensor_y) + ((x51 + x49 + x48) * sensor_z) +
						(((x47 * x19) + x20 + (x46 * x16)) * sensor_x);
	const GEN_FLT x53 = 1 + x52;
	const GEN_FLT x54 = x35 * x35;
	const GEN_FLT x55 = x36 + (x54 * x37);
	const GEN_FLT x56 = 2 * x26;
	const GEN_FLT x57 = x4 * x4;
	const GEN_FLT x58 = (((x57 * x19) + (x0 * x56) + x20) * sensor_z) + ((x28 + x25 + x23) * sensor_y) +
						((x51 + (-1 * x49) + (-1 * x48)) * sensor_x);
	const GEN_FLT x59 = x2 * x13;
	const GEN_FLT x60 = x26 * x10;
	const GEN_FLT x61 = x10 * x17;
	const GEN_FLT x62 =
		((x6 + (x47 * x11)) * sensor_x) + ((x61 + (-1 * x27)) * sensor_y) + ((x60 + x59) * sensor_z) + obj_px;
	const GEN_FLT x63 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x64 = x63 * x36;
	const GEN_FLT x65 = x64 * x34;
	const GEN_FLT x66 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x67 = x66 * x31;
	const GEN_FLT x68 = x31 * x35;
	const GEN_FLT x69 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x70 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x71 = (x70 * x43) + (x69 * x38) + (x63 * x68 * x32);
	const GEN_FLT x72 = x64 * x32;
	const GEN_FLT x73 = x69 * x31;
	const GEN_FLT x74 = x63 * x42;
	const GEN_FLT x75 = x34 * x37;
	const GEN_FLT x76 = (x66 * x38) + (x70 * x75) + (x74 * x35);
	const GEN_FLT x77 = x26 * x13;
	const GEN_FLT x78 =
		((x61 + x27) * sensor_x) + ((x6 + (x22 * x11)) * sensor_y) + ((x77 + (-1 * x14)) * sensor_z) + obj_py;
	const GEN_FLT x79 =
		((x60 + (-1 * x59)) * sensor_x) + ((x77 + x14) * sensor_y) + ((x6 + (x57 * x11)) * sensor_z) + obj_pz;
	const GEN_FLT x80 = x63 * x31;
	const GEN_FLT x81 = -1 * x80;
	const GEN_FLT x82 = (x79 * ((2 * x70 * x38) + (x80 * x54) + x81)) + (x78 * (x76 + x73 + x72)) +
						(x62 * (x71 + (-1 * x67) + (-1 * x65)));
	const GEN_FLT x83 = x82 + (x58 * x55);
	const GEN_FLT x84 = x83 + (x53 * x45) + x41;
	const GEN_FLT x85 = x44 + x42;
	const GEN_FLT x86 = x43 * x34;
	const GEN_FLT x87 = x86 + (-1 * x68);
	const GEN_FLT x88 = x32 * x32;
	const GEN_FLT x89 = x36 + (x88 * x37);
	const GEN_FLT x90 = (x89 * x62) + (x87 * x78) + (x85 * x79) + lh_px;
	const GEN_FLT x91 = (x62 * x45) + (x78 * x40) + (x79 * x55) + lh_pz;
	const GEN_FLT x92 = x91 * x91;
	const GEN_FLT x93 = 1. / x92;
	const GEN_FLT x94 = x93 * x90;
	const GEN_FLT x95 = 1. / x91;
	const GEN_FLT x96 = x87 * x29;
	const GEN_FLT x97 = x64 * x35;
	const GEN_FLT x98 = x70 * x31;
	const GEN_FLT x99 = (x74 * x32) + (x66 * x43) + (x75 * x69);
	const GEN_FLT x100 = (x79 * (x71 + x67 + x65)) + (x78 * (x99 + (-1 * x98) + (-1 * x97))) +
						 (x62 * ((2 * x69 * x43) + (x80 * x88) + x81));
	const GEN_FLT x101 = x100 + (x85 * x58);
	const GEN_FLT x102 = x101 + (x89 * x53) + x96;
	const GEN_FLT x103 = x90 * x90;
	const GEN_FLT x104 = -1 * x91;
	const GEN_FLT x105 = 2 * x92 * atan2(x90, x104) * (1. / (x92 + x103)) * curve_1;
	const GEN_FLT x106 = x86 + x68;
	const GEN_FLT x107 = x34 * x34;
	const GEN_FLT x108 = x36 + (x37 * x107);
	const GEN_FLT x109 = x29 * x108;
	const GEN_FLT x110 = x39 + (-1 * x33);
	const GEN_FLT x111 = (x79 * (x76 + (-1 * x73) + (-1 * x72))) + (x78 * ((2 * x75 * x66) + (x80 * x107) + x81)) +
						 (x62 * (x99 + x98 + x97));
	const GEN_FLT x112 = x111 + (x58 * x110);
	const GEN_FLT x113 = x112 + x109 + (x53 * x106);
	const GEN_FLT x114 = (x62 * x106) + (x78 * x108) + (x79 * x110) + lh_py;
	const GEN_FLT x115 = x93 * x114;
	const GEN_FLT x116 = x92 + (x114 * x114);
	const GEN_FLT x117 = 1. / x116;
	const GEN_FLT x118 = x92 * x117;
	const GEN_FLT x119 = 2 * x114;
	const GEN_FLT x120 = 2 * x91;
	const GEN_FLT x121 = 1.0 / 2.0 * x90 * (1. / (x116 * sqrt(x116))) * tilt_1;
	const GEN_FLT x122 = (1. / sqrt(x116)) * tilt_1;
	const GEN_FLT x123 = 1. / sqrt(1 + (-1 * x103 * x117 * (tilt_1 * tilt_1)));
	const GEN_FLT x124 = (-1 * x123 * ((x102 * x122) + (-1 * x121 * ((x84 * x120) + (x113 * x119))))) +
						 (-1 * ((-1 * x84 * x115) + (x95 * x113)) * x118);
	const GEN_FLT x125 =
		sin(1.5707963267949 + (-1 * phase_1) + (-1 * asin(x90 * x122)) + (-1 * atan2(-1 * x114, x104)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x126 = x52 * x45;
	const GEN_FLT x127 = 1 + x29;
	const GEN_FLT x128 = x83 + (x40 * x127) + x126;
	const GEN_FLT x129 = x89 * x52;
	const GEN_FLT x130 = x101 + (x87 * x127) + x129;
	const GEN_FLT x131 = x52 * x106;
	const GEN_FLT x132 = x112 + (x108 * x127) + x131;
	const GEN_FLT x133 = (-1 * x123 * ((x122 * x130) + (-1 * ((x120 * x128) + (x119 * x132)) * x121))) +
						 (-1 * x118 * ((-1 * x115 * x128) + (x95 * x132)));
	const GEN_FLT x134 = 1 + x58;
	const GEN_FLT x135 = (x55 * x134) + x82 + x41 + x126;
	const GEN_FLT x136 = x100 + (x85 * x134) + x129 + x96;
	const GEN_FLT x137 = x111 + (x110 * x134) + x109 + x131;
	const GEN_FLT x138 = (-1 * x123 * ((x122 * x136) + (-1 * ((x120 * x135) + (x119 * x137)) * x121))) +
						 (-1 * x118 * ((-1 * x115 * x135) + (x95 * x137)));
	const GEN_FLT x139 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x140 = x2 * x139;
	const GEN_FLT x141 = -1 * x140;
	const GEN_FLT x142 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qi *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x143 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qk *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x144 = x2 * x143;
	const GEN_FLT x145 = x6 * x139;
	const GEN_FLT x146 = x4 * x145;
	const GEN_FLT x147 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qj *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qi * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x148 = (x15 * x139) + (x12 * x147) + (x17 * x142);
	const GEN_FLT x149 = x2 * x147;
	const GEN_FLT x150 = x13 * x145;
	const GEN_FLT x151 = (x50 * x139) + (x12 * x143) + (x26 * x142);
	const GEN_FLT x152 = ((x151 + x150 + x149) * sensor_z) + ((x148 + (-1 * x146) + (-1 * x144)) * sensor_y) +
						 (((x47 * x140) + (x46 * x142) + x141) * sensor_x);
	const GEN_FLT x153 = x2 * x142;
	const GEN_FLT x154 = x24 * x139;
	const GEN_FLT x155 = x4 * x13;
	const GEN_FLT x156 = (x140 * x155) + (x17 * x143) + (x26 * x147);
	const GEN_FLT x157 = ((x156 + (-1 * x154) + (-1 * x153)) * sensor_z) +
						 (((x22 * x140) + x141 + (x21 * x147)) * sensor_y) + ((x148 + x146 + x144) * sensor_x);
	const GEN_FLT x158 = (((x57 * x140) + (x56 * x143) + x141) * sensor_z) + ((x156 + x154 + x153) * sensor_y) +
						 ((x151 + (-1 * x150) + (-1 * x149)) * sensor_x);
	const GEN_FLT x159 = x82 + (x55 * x158) + (x40 * x157) + (x45 * x152);
	const GEN_FLT x160 = x100 + (x85 * x158) + (x87 * x157) + (x89 * x152);
	const GEN_FLT x161 = x111 + (x110 * x158) + (x108 * x157) + (x106 * x152);
	const GEN_FLT x162 = (-1 * x123 * ((x122 * x160) + (-1 * ((x120 * x159) + (x119 * x161)) * x121))) +
						 (-1 * x118 * ((-1 * x115 * x159) + (x95 * x161)));
	const GEN_FLT x163 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x164 = x2 * x163;
	const GEN_FLT x165 = -1 * x164;
	const GEN_FLT x166 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qi *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x167 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qk *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x168 = x2 * x167;
	const GEN_FLT x169 = x6 * x163;
	const GEN_FLT x170 = x4 * x169;
	const GEN_FLT x171 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qj *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qj * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x172 = (x15 * x163) + (x12 * x171) + (x17 * x166);
	const GEN_FLT x173 = x2 * x171;
	const GEN_FLT x174 = x13 * x169;
	const GEN_FLT x175 = (x12 * x167) + (x50 * x163) + (x26 * x166);
	const GEN_FLT x176 = ((x175 + x174 + x173) * sensor_z) + ((x172 + (-1 * x170) + (-1 * x168)) * sensor_y) +
						 (((x47 * x164) + (x46 * x166) + x165) * sensor_x);
	const GEN_FLT x177 = x2 * x166;
	const GEN_FLT x178 = x10 * x169;
	const GEN_FLT x179 = (x26 * x171) + (x164 * x155) + (x17 * x167);
	const GEN_FLT x180 = ((x179 + (-1 * x178) + (-1 * x177)) * sensor_z) + ((x172 + x170 + x168) * sensor_x) +
						 (((x22 * x164) + (x21 * x171) + x165) * sensor_y);
	const GEN_FLT x181 = (((x57 * x164) + (x56 * x167) + x165) * sensor_z) + ((x179 + x178 + x177) * sensor_y) +
						 ((x175 + (-1 * x174) + (-1 * x173)) * sensor_x);
	const GEN_FLT x182 = x82 + (x55 * x181) + (x40 * x180) + (x45 * x176);
	const GEN_FLT x183 = x100 + (x85 * x181) + (x87 * x180) + (x89 * x176);
	const GEN_FLT x184 = x111 + (x110 * x181) + (x108 * x180) + (x106 * x176);
	const GEN_FLT x185 = (-1 * x123 * ((x122 * x183) + (-1 * ((x120 * x182) + (x119 * x184)) * x121))) +
						 (-1 * x118 * ((-1 * x115 * x182) + (x95 * x184)));
	const GEN_FLT x186 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
							 : 0;
	const GEN_FLT x187 = x2 * x186;
	const GEN_FLT x188 = -1 * x187;
	const GEN_FLT x189 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qi *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x190 =
		(0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
				  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
				  : 1e-10))
			? ((-1 * obj_qk *
				((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
					 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
					 : 0) *
				(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10) *
					   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10)))) +
			   (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						  : 1e-10)))
			: 0;
	const GEN_FLT x191 = x2 * x190;
	const GEN_FLT x192 = x6 * x186;
	const GEN_FLT x193 = x4 * x192;
	const GEN_FLT x194 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								   : 1e-10))
							 ? (-1 * obj_qj *
								((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
									 ? (obj_qk * (1. / sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))))
									 : 0) *
								(1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10) *
									   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
											? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
											: 1e-10))))
							 : 0;
	const GEN_FLT x195 = (x12 * x194) + (x15 * x186) + (x17 * x189);
	const GEN_FLT x196 = x2 * x194;
	const GEN_FLT x197 = x13 * x192;
	const GEN_FLT x198 = (x12 * x190) + (x50 * x186) + (x26 * x189);
	const GEN_FLT x199 = ((x198 + x197 + x196) * sensor_z) + (((-1 * x193) + x195 + (-1 * x191)) * sensor_y) +
						 (((x47 * x187) + (x46 * x189) + x188) * sensor_x);
	const GEN_FLT x200 = x2 * x189;
	const GEN_FLT x201 = x24 * x186;
	const GEN_FLT x202 = (x187 * x155) + (x17 * x190) + (x26 * x194);
	const GEN_FLT x203 = ((x202 + (-1 * x201) + (-1 * x200)) * sensor_z) +
						 (((x22 * x187) + x188 + (x21 * x194)) * sensor_y) + ((x193 + x195 + x191) * sensor_x);
	const GEN_FLT x204 = (((x57 * x187) + (x56 * x190) + x188) * sensor_z) + ((x202 + x201 + x200) * sensor_y) +
						 ((x198 + (-1 * x197) + (-1 * x196)) * sensor_x);
	const GEN_FLT x205 = x82 + (x55 * x204) + (x40 * x203) + (x45 * x199);
	const GEN_FLT x206 = x100 + (x85 * x204) + (x87 * x203) + (x89 * x199);
	const GEN_FLT x207 = x111 + (x204 * x110) + (x203 * x108) + (x106 * x199);
	const GEN_FLT x208 = (-1 * x123 * ((x206 * x122) + (-1 * ((x205 * x120) + (x207 * x119)) * x121))) +
						 (-1 * x118 * ((-1 * x205 * x115) + (x95 * x207)));
	out[0] = x124 + (x124 * x125) + (x105 * ((-1 * x95 * x102) + (x84 * x94)));
	out[1] = x133 + (x125 * x133) + (((-1 * x95 * x130) + (x94 * x128)) * x105);
	out[2] = x138 + (x125 * x138) + (((-1 * x95 * x136) + (x94 * x135)) * x105);
	out[3] = x162 + (x125 * x162) + (((-1 * x95 * x160) + (x94 * x159)) * x105);
	out[4] = x185 + (x125 * x185) + (((-1 * x95 * x183) + (x94 * x182)) * x105);
	out[5] = x208 + (x208 * x125) + (((-1 * x95 * x206) + (x94 * x205)) * x105);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x5 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x9 + x3;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = x14 + (x15 * x12);
	const GEN_FLT x17 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x18 = sin(x13);
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x21 * x11;
	const GEN_FLT x23 = x22 + x19;
	const GEN_FLT x24 = x20 * x18;
	const GEN_FLT x25 = x15 * x11;
	const GEN_FLT x26 = x25 * x17;
	const GEN_FLT x27 = x26 + (-1 * x24);
	const GEN_FLT x28 = (x23 * sensor_y) + (x16 * sensor_z) + (x27 * sensor_x) + obj_pz;
	const GEN_FLT x29 = x1 * x4;
	const GEN_FLT x30 = x2 * x8;
	const GEN_FLT x31 = x30 + (-1 * x29);
	const GEN_FLT x32 = x22 + (-1 * x19);
	const GEN_FLT x33 = x20 * x20;
	const GEN_FLT x34 = x14 + (x33 * x15);
	const GEN_FLT x35 = x11 * x18;
	const GEN_FLT x36 = x21 * x17;
	const GEN_FLT x37 = x36 + x35;
	const GEN_FLT x38 = (x37 * sensor_x) + (x34 * sensor_y) + (x32 * sensor_z) + obj_py;
	const GEN_FLT x39 = x5 * x5;
	const GEN_FLT x40 = x6 + (x7 * x39);
	const GEN_FLT x41 = x26 + x24;
	const GEN_FLT x42 = x36 + (-1 * x35);
	const GEN_FLT x43 = x17 * x17;
	const GEN_FLT x44 = x14 + (x43 * x15);
	const GEN_FLT x45 = (x44 * sensor_x) + (x41 * sensor_z) + (x42 * sensor_y) + obj_px;
	const GEN_FLT x46 = (x31 * x38) + (x40 * x45) + (x28 * x10) + lh_px;
	const GEN_FLT x47 = x4 * x4;
	const GEN_FLT x48 = x6 + (x7 * x47);
	const GEN_FLT x49 = x1 * x5;
	const GEN_FLT x50 = x2 * x7;
	const GEN_FLT x51 = x4 * x50;
	const GEN_FLT x52 = x51 + x49;
	const GEN_FLT x53 = x9 + (-1 * x3);
	const GEN_FLT x54 = (x53 * x45) + (x52 * x38) + (x48 * x28) + lh_pz;
	const GEN_FLT x55 = x54 * x54;
	const GEN_FLT x56 = 1. / x55;
	const GEN_FLT x57 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x58 = x15 * x17;
	const GEN_FLT x59 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x60 = x59 * x18;
	const GEN_FLT x61 = -1 * x60;
	const GEN_FLT x62 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x63 = x62 * x18;
	const GEN_FLT x64 = x59 * x14;
	const GEN_FLT x65 = x64 * x11;
	const GEN_FLT x66 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x67 = x59 * x20;
	const GEN_FLT x68 = (x57 * x21) + (x67 * x19) + (x66 * x58);
	const GEN_FLT x69 = x66 * x18;
	const GEN_FLT x70 = x64 * x20;
	const GEN_FLT x71 = (x62 * x58) + (x59 * x11 * x19) + (x57 * x25);
	const GEN_FLT x72 = ((x71 + x70 + x69) * sensor_z) + ((x68 + (-1 * x65) + (-1 * x63)) * sensor_y) +
						(((x60 * x43) + x61 + (2 * x58 * x57)) * sensor_x);
	const GEN_FLT x73 = x72 + x44;
	const GEN_FLT x74 = x57 * x18;
	const GEN_FLT x75 = x64 * x17;
	const GEN_FLT x76 = (x67 * x35) + (x62 * x21) + (x66 * x25);
	const GEN_FLT x77 = ((x76 + (-1 * x75) + (-1 * x74)) * sensor_z) +
						(((x60 * x33) + (2 * x66 * x21) + x61) * sensor_y) + ((x68 + x65 + x63) * sensor_x);
	const GEN_FLT x78 = x77 + x37;
	const GEN_FLT x79 = (((x60 * x12) + (2 * x62 * x25) + x61) * sensor_z) + ((x76 + x75 + x74) * sensor_y) +
						((x71 + (-1 * x70) + (-1 * x69)) * sensor_x);
	const GEN_FLT x80 = x79 + x27;
	const GEN_FLT x81 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x82 = x6 * x81;
	const GEN_FLT x83 = x2 * x82;
	const GEN_FLT x84 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x85 = x1 * x84;
	const GEN_FLT x86 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x87 = x7 * x86;
	const GEN_FLT x88 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x89 = (x8 * x88) + (x4 * x87) + (x4 * x81 * x49);
	const GEN_FLT x90 = x5 * x82;
	const GEN_FLT x91 = x1 * x86;
	const GEN_FLT x92 = x2 * x81;
	const GEN_FLT x93 = x4 * x7;
	const GEN_FLT x94 = (x84 * x93) + (x88 * x50) + (x92 * x29);
	const GEN_FLT x95 = x1 * x81;
	const GEN_FLT x96 = -1 * x95;
	const GEN_FLT x97 = (x28 * ((2 * x88 * x93) + (x95 * x47) + x96)) + (x38 * (x94 + x91 + x90)) +
						(x45 * ((-1 * x85) + x89 + (-1 * x83)));
	const GEN_FLT x98 = x97 + (x80 * x48) + (x78 * x52) + (x73 * x53);
	const GEN_FLT x99 = x56 * x98;
	const GEN_FLT x100 = 1. / x54;
	const GEN_FLT x101 = x4 * x82;
	const GEN_FLT x102 = x1 * x88;
	const GEN_FLT x103 = (x92 * x49) + (x8 * x84) + (x2 * x87);
	const GEN_FLT x104 = (x28 * (x85 + x89 + x83)) + (x38 * (x103 + (-1 * x102) + (-1 * x101))) +
						 (x45 * ((2 * x5 * x87) + (x95 * x39) + x96));
	const GEN_FLT x105 = x104 + (x80 * x10) + (x78 * x31) + (x73 * x40);
	const GEN_FLT x106 = x46 * x46;
	const GEN_FLT x107 = -1 * x54;
	const GEN_FLT x108 = 2 * x55 * atan2(x46, x107) * (1. / (x55 + x106)) * curve_1;
	const GEN_FLT x109 = x30 + x29;
	const GEN_FLT x110 = x2 * x2;
	const GEN_FLT x111 = x6 + (x7 * x110);
	const GEN_FLT x112 = x51 + (-1 * x49);
	const GEN_FLT x113 = (x28 * (x94 + (-1 * x91) + (-1 * x90))) + (x38 * ((x95 * x110) + (2 * x84 * x50) + x96)) +
						 (x45 * (x103 + x102 + x101));
	const GEN_FLT x114 = x113 + (x80 * x112) + (x78 * x111) + (x73 * x109);
	const GEN_FLT x115 = (x45 * x109) + (x38 * x111) + (x28 * x112) + lh_py;
	const GEN_FLT x116 = x55 + (x115 * x115);
	const GEN_FLT x117 = 1. / x116;
	const GEN_FLT x118 = x55 * x117;
	const GEN_FLT x119 = 2 * x115;
	const GEN_FLT x120 = 2 * x54;
	const GEN_FLT x121 = 1.0 / 2.0 * x46 * (1. / (x116 * sqrt(x116))) * tilt_1;
	const GEN_FLT x122 = (1. / sqrt(x116)) * tilt_1;
	const GEN_FLT x123 = 1. / sqrt(1 + (-1 * x106 * x117 * (tilt_1 * tilt_1)));
	const GEN_FLT x124 = (-1 * x123 * ((x105 * x122) + (-1 * x121 * ((x98 * x120) + (x119 * x114))))) +
						 (-1 * x118 * ((-1 * x99 * x115) + (x100 * x114)));
	const GEN_FLT x125 =
		sin(1.5707963267949 + (-1 * phase_1) + (-1 * atan2(-1 * x115, x107)) + (-1 * asin(x46 * x122)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x126 = x72 + x42;
	const GEN_FLT x127 = x77 + x34;
	const GEN_FLT x128 = x79 + x23;
	const GEN_FLT x129 = x97 + (x48 * x128) + (x52 * x127) + (x53 * x126);
	const GEN_FLT x130 = x56 * x129;
	const GEN_FLT x131 = x104 + (x10 * x128) + (x31 * x127) + (x40 * x126);
	const GEN_FLT x132 = x113 + (x112 * x128) + (x111 * x127) + (x109 * x126);
	const GEN_FLT x133 = (-1 * x123 * ((x122 * x131) + (-1 * ((x120 * x129) + (x119 * x132)) * x121))) +
						 (-1 * ((-1 * x115 * x130) + (x100 * x132)) * x118);
	const GEN_FLT x134 = x72 + x41;
	const GEN_FLT x135 = x77 + x32;
	const GEN_FLT x136 = x79 + x16;
	const GEN_FLT x137 = x97 + (x52 * x135) + (x48 * x136) + (x53 * x134);
	const GEN_FLT x138 = x56 * x137;
	const GEN_FLT x139 = x104 + (x10 * x136) + (x31 * x135) + (x40 * x134);
	const GEN_FLT x140 = (x112 * x136) + (x109 * x134) + x113 + (x111 * x135);
	const GEN_FLT x141 = (-1 * x123 * ((x122 * x139) + (-1 * ((x120 * x137) + (x119 * x140)) * x121))) +
						 (-1 * ((-1 * x115 * x138) + (x100 * x140)) * x118);
	out[0] = x124 + (x124 * x125) + (x108 * ((-1 * x100 * x105) + (x99 * x46)));
	out[1] = x133 + (x125 * x133) + (x108 * ((-1 * x100 * x131) + (x46 * x130)));
	out[2] = x141 + (x125 * x141) + (x108 * ((-1 * x100 * x139) + (x46 * x138)));
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
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 0;
	const GEN_FLT x1 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
						   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
						   : 1e-10;
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 1;
	const GEN_FLT x5 = cos(x1);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								 ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								 : 1e-10))
						   ? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												  : 1e-10)))
						   : 0;
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x2 * x7;
	const GEN_FLT x11 = x0 * x6;
	const GEN_FLT x12 = x4 * x11;
	const GEN_FLT x13 = x4 * x4;
	const GEN_FLT x14 =
		((x5 + (x6 * x13)) * sensor_x) + ((x12 + (-1 * x10)) * sensor_y) + ((x9 + x3) * sensor_z) + obj_px;
	const GEN_FLT x15 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 0;
	const GEN_FLT x16 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x17 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
							: 1e-10;
	const GEN_FLT x18 = cos(x17);
	const GEN_FLT x19 = x18 * x16;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = sin(x17);
	const GEN_FLT x22 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 1;
	const GEN_FLT x25 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												  : 1e-10)))
							: 0;
	const GEN_FLT x26 = x25 * x21;
	const GEN_FLT x27 = x24 * x26;
	const GEN_FLT x28 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x29 = 1 + (-1 * x18);
	const GEN_FLT x30 = x25 * x29;
	const GEN_FLT x31 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x32 = x24 * x29;
	const GEN_FLT x33 = (x32 * x31) + (x30 * x28) + (x27 * x16);
	const GEN_FLT x34 = x24 * x18;
	const GEN_FLT x35 = x34 * x16;
	const GEN_FLT x36 = x21 * x28;
	const GEN_FLT x37 = x26 * x15;
	const GEN_FLT x38 = x29 * x15;
	const GEN_FLT x39 = (x30 * x22) + (x31 * x38) + (x37 * x16);
	const GEN_FLT x40 = x2 * x4;
	const GEN_FLT x41 = x7 * x11;
	const GEN_FLT x42 = x0 * x0;
	const GEN_FLT x43 =
		((x12 + x10) * sensor_x) + ((x5 + (x6 * x42)) * sensor_y) + ((x41 + (-1 * x40)) * sensor_z) + obj_py;
	const GEN_FLT x44 = x7 * x7;
	const GEN_FLT x45 =
		((x9 + (-1 * x3)) * sensor_x) + ((x41 + x40) * sensor_y) + ((x5 + (x6 * x44)) * sensor_z) + obj_pz;
	const GEN_FLT x46 = x21 * x16;
	const GEN_FLT x47 = -1 * x46;
	const GEN_FLT x48 = x25 * x25;
	const GEN_FLT x49 = 2 * x30;
	const GEN_FLT x50 = x21 * x15;
	const GEN_FLT x51 = x32 * x25;
	const GEN_FLT x52 = x51 + (-1 * x50);
	const GEN_FLT x53 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x54 = x4 * x6;
	const GEN_FLT x55 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x56 = x2 * x55;
	const GEN_FLT x57 = -1 * x56;
	const GEN_FLT x58 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x59 = x2 * x58;
	const GEN_FLT x60 = x5 * x55;
	const GEN_FLT x61 = x7 * x60;
	const GEN_FLT x62 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x63 = x55 * x40;
	const GEN_FLT x64 = (x53 * x11) + (x0 * x63) + (x62 * x54);
	const GEN_FLT x65 = x2 * x62;
	const GEN_FLT x66 = x0 * x60;
	const GEN_FLT x67 = (x54 * x58) + (x7 * x63) + (x8 * x53);
	const GEN_FLT x68 = ((x67 + x66 + x65) * sensor_z) + ((x64 + (-1 * x61) + (-1 * x59)) * sensor_y) +
						(((x56 * x13) + x57 + (2 * x54 * x53)) * sensor_x);
	const GEN_FLT x69 = x2 * x53;
	const GEN_FLT x70 = x4 * x60;
	const GEN_FLT x71 = (x0 * x7 * x56) + (x58 * x11) + (x8 * x62);
	const GEN_FLT x72 = ((x71 + (-1 * x70) + (-1 * x69)) * sensor_z) +
						(((x56 * x42) + (2 * x62 * x11) + x57) * sensor_y) + ((x64 + x61 + x59) * sensor_x);
	const GEN_FLT x73 = x24 * x21;
	const GEN_FLT x74 = x38 * x25;
	const GEN_FLT x75 = x74 + x73;
	const GEN_FLT x76 = x18 + (x48 * x29);
	const GEN_FLT x77 = (((x56 * x44) + (2 * x8 * x58) + x57) * sensor_z) + ((x71 + x70 + x69) * sensor_y) +
						((x67 + (-1 * x66) + (-1 * x65)) * sensor_x);
	const GEN_FLT x78 = (x77 * x76) + (x72 * x75) + (x68 * x52);
	const GEN_FLT x79 = x78 + (x45 * ((x46 * x48) + (x49 * x31) + x47)) + (x43 * (x39 + x36 + x35)) +
						(x14 * (x33 + (-1 * x23) + (-1 * x20)));
	const GEN_FLT x80 = (x52 * x14) + (x75 * x43) + (x76 * x45) + lh_pz;
	const GEN_FLT x81 = x80 * x80;
	const GEN_FLT x82 = 1. / x81;
	const GEN_FLT x83 = x51 + x50;
	const GEN_FLT x84 = x32 * x15;
	const GEN_FLT x85 = x84 + (-1 * x26);
	const GEN_FLT x86 = x24 * x24;
	const GEN_FLT x87 = x18 + (x86 * x29);
	const GEN_FLT x88 = (x87 * x14) + (x83 * x45) + (x85 * x43) + lh_px;
	const GEN_FLT x89 = x82 * x88;
	const GEN_FLT x90 = x89 * x79;
	const GEN_FLT x91 = 1. / x80;
	const GEN_FLT x92 = 2 * x32;
	const GEN_FLT x93 = x25 * x19;
	const GEN_FLT x94 = x31 * x21;
	const GEN_FLT x95 = x73 * x15;
	const GEN_FLT x96 = (x95 * x16) + (x32 * x22) + (x38 * x28);
	const GEN_FLT x97 = (x83 * x77) + (x85 * x72) + (x87 * x68);
	const GEN_FLT x98 = x97 + (x45 * (x33 + x23 + x20)) + (x43 * (x96 + (-1 * x94) + (-1 * x93))) +
						(x14 * ((x86 * x46) + (x92 * x28) + x47));
	const GEN_FLT x99 = 1 + x98;
	const GEN_FLT x100 = x88 * x88;
	const GEN_FLT x101 = -1 * x80;
	const GEN_FLT x102 = 2 * x81 * atan2(x88, x101) * (1. / (x81 + x100)) * curve_1;
	const GEN_FLT x103 = x15 * x15;
	const GEN_FLT x104 = 2 * x38;
	const GEN_FLT x105 = x84 + x26;
	const GEN_FLT x106 = x18 + (x29 * x103);
	const GEN_FLT x107 = x74 + (-1 * x73);
	const GEN_FLT x108 = (x77 * x107) + (x72 * x106) + (x68 * x105);
	const GEN_FLT x109 = x108 + (x43 * ((x22 * x104) + (x46 * x103) + x47)) + (x45 * (x39 + (-1 * x36) + (-1 * x35))) +
						 (x14 * (x96 + x94 + x93));
	const GEN_FLT x110 = x91 * x109;
	const GEN_FLT x111 = (x14 * x105) + (x43 * x106) + (x45 * x107) + lh_py;
	const GEN_FLT x112 = x82 * x111;
	const GEN_FLT x113 = -1 * x79 * x112;
	const GEN_FLT x114 = x81 + (x111 * x111);
	const GEN_FLT x115 = 1. / x114;
	const GEN_FLT x116 = x81 * x115;
	const GEN_FLT x117 = 2 * x111;
	const GEN_FLT x118 = x109 * x117;
	const GEN_FLT x119 = 2 * x80;
	const GEN_FLT x120 = x79 * x119;
	const GEN_FLT x121 = 1.0 / 2.0 * x88 * (1. / (x114 * sqrt(x114))) * tilt_1;
	const GEN_FLT x122 = (1. / sqrt(x114)) * tilt_1;
	const GEN_FLT x123 = 1. / sqrt(1 + (-1 * x100 * x115 * (tilt_1 * tilt_1)));
	const GEN_FLT x124 = (-1 * x123 * ((x99 * x122) + (-1 * (x120 + x118) * x121))) + (-1 * (x113 + x110) * x116);
	const GEN_FLT x125 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x111, x101)) + (-1 * phase_1) + (-1 * asin(x88 * x122)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x126 = -1 * x91 * x98;
	const GEN_FLT x127 = 1 + x109;
	const GEN_FLT x128 = x98 * x122;
	const GEN_FLT x129 =
		(-1 * x123 * (x128 + (-1 * x121 * (x120 + (x117 * x127))))) + (-1 * x116 * (x113 + (x91 * x127)));
	const GEN_FLT x130 = 1 + x79;
	const GEN_FLT x131 =
		(-1 * x123 * (x128 + (-1 * x121 * ((x119 * x130) + x118)))) + (-1 * x116 * ((-1 * x112 * x130) + x110));
	const GEN_FLT x132 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x133 = x18 * x132;
	const GEN_FLT x134 = x15 * x133;
	const GEN_FLT x135 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qj *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x136 = x21 * x135;
	const GEN_FLT x137 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qi *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x138 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qk *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qi * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x139 = (x27 * x132) + (x32 * x138) + (x30 * x137);
	const GEN_FLT x140 = x34 * x132;
	const GEN_FLT x141 = x21 * x137;
	const GEN_FLT x142 = (x30 * x135) + (x38 * x138) + (x37 * x132);
	const GEN_FLT x143 = x21 * x132;
	const GEN_FLT x144 = -1 * x143;
	const GEN_FLT x145 = (x45 * ((x49 * x138) + (x48 * x143) + x144)) + (x43 * (x142 + x141 + x140)) + x78 +
						 (x14 * (x139 + (-1 * x136) + (-1 * x134)));
	const GEN_FLT x146 = x25 * x133;
	const GEN_FLT x147 = x21 * x138;
	const GEN_FLT x148 = (x32 * x135) + (x38 * x137) + (x95 * x132);
	const GEN_FLT x149 = x97 + (x45 * (x139 + x136 + x134)) + (x43 * (x148 + (-1 * x147) + (-1 * x146))) +
						 (x14 * ((x92 * x137) + (x86 * x143) + x144));
	const GEN_FLT x150 = x108 + (x14 * (x148 + x147 + x146)) + (x45 * (x142 + (-1 * x141) + (-1 * x140))) +
						 (x43 * ((x104 * x135) + (x103 * x143) + x144));
	const GEN_FLT x151 = (-1 * x123 * ((x122 * x149) + (-1 * ((x119 * x145) + (x117 * x150)) * x121))) +
						 (-1 * x116 * ((-1 * x112 * x145) + (x91 * x150)));
	const GEN_FLT x152 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x153 = x15 * x18;
	const GEN_FLT x154 = x152 * x153;
	const GEN_FLT x155 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qj *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x156 = x21 * x155;
	const GEN_FLT x157 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qi *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x158 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qk *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qj * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x159 = (x32 * x158) + (x30 * x157) + (x27 * x152);
	const GEN_FLT x160 = x34 * x152;
	const GEN_FLT x161 = x21 * x157;
	const GEN_FLT x162 = x15 * x152;
	const GEN_FLT x163 = (x30 * x155) + (x38 * x158) + (x26 * x162);
	const GEN_FLT x164 = x21 * x152;
	const GEN_FLT x165 = -1 * x164;
	const GEN_FLT x166 = x78 + (x45 * ((x49 * x158) + (x48 * x164) + x165)) + (x43 * (x163 + x161 + x160)) +
						 (x14 * (x159 + (-1 * x156) + (-1 * x154)));
	const GEN_FLT x167 = x25 * x18;
	const GEN_FLT x168 = x167 * x152;
	const GEN_FLT x169 = x21 * x158;
	const GEN_FLT x170 = (x38 * x157) + (x32 * x155) + (x73 * x162);
	const GEN_FLT x171 = x97 + (x45 * (x159 + x156 + x154)) + (x43 * (x170 + (-1 * x169) + (-1 * x168))) +
						 (x14 * ((x92 * x157) + x165 + (x86 * x164)));
	const GEN_FLT x172 = (x45 * (x163 + (-1 * x161) + (-1 * x160))) + (x43 * ((x104 * x155) + (x103 * x164) + x165)) +
						 x108 + (x14 * (x170 + x169 + x168));
	const GEN_FLT x173 = (-1 * x123 * ((x122 * x171) + (-1 * ((x119 * x166) + (x117 * x172)) * x121))) +
						 (-1 * x116 * ((-1 * x112 * x166) + (x91 * x172)));
	const GEN_FLT x174 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
							 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
							 : 0;
	const GEN_FLT x175 = x174 * x153;
	const GEN_FLT x176 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qj *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x177 = x21 * x176;
	const GEN_FLT x178 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? (-1 * lh_qi *
								(1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10) *
									   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											: 1e-10))) *
								((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									 ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									 : 0))
							 : 0;
	const GEN_FLT x179 = x29 * x178;
	const GEN_FLT x180 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								   : 1e-10))
							 ? ((-1 * lh_qk *
								 (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10) *
										((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
											 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
											 : 1e-10))) *
								 ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
									  ? (lh_qk * (1. / sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))))
									  : 0)) +
								(1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10)))
							 : 0;
	const GEN_FLT x181 = (x32 * x180) + (x25 * x179) + (x27 * x174);
	const GEN_FLT x182 = x34 * x174;
	const GEN_FLT x183 = x21 * x178;
	const GEN_FLT x184 = (x30 * x176) + (x38 * x180) + (x37 * x174);
	const GEN_FLT x185 = x21 * x174;
	const GEN_FLT x186 = -1 * x185;
	const GEN_FLT x187 = x78 + (x45 * ((x49 * x180) + (x48 * x185) + x186)) + (x43 * (x184 + x183 + x182)) +
						 (x14 * (x181 + (-1 * x177) + (-1 * x175)));
	const GEN_FLT x188 = x167 * x174;
	const GEN_FLT x189 = x21 * x180;
	const GEN_FLT x190 = (x32 * x176) + (x15 * x179) + (x95 * x174);
	const GEN_FLT x191 = (x45 * (x181 + x177 + x175)) + (x43 * (x190 + (-1 * x189) + (-1 * x188))) + x97 +
						 (x14 * ((x92 * x178) + (x86 * x185) + x186));
	const GEN_FLT x192 = x108 + (x45 * (x184 + (-1 * x183) + (-1 * x182))) +
						 (x43 * ((x104 * x176) + (x103 * x185) + x186)) + (x14 * (x190 + x189 + x188));
	const GEN_FLT x193 = (-1 * x123 * ((x122 * x191) + (-1 * ((x119 * x187) + (x117 * x192)) * x121))) +
						 (-1 * x116 * ((-1 * x112 * x187) + (x91 * x192)));
	out[0] = x124 + (x124 * x125) + (x102 * ((-1 * x91 * x99) + x90));
	out[1] = x129 + (x125 * x129) + (x102 * (x126 + x90));
	out[2] = x131 + (x125 * x131) + (x102 * (x126 + (x89 * x130)));
	out[3] = x151 + (x125 * x151) + (((-1 * x91 * x149) + (x89 * x145)) * x102);
	out[4] = x173 + (x125 * x173) + (((-1 * x91 * x171) + (x89 * x166)) * x102);
	out[5] = x193 + (x125 * x193) + (((-1 * x91 * x191) + (x89 * x187)) * x102);
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
						   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
						   : 1e-10;
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qj * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qk * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 0;
	const GEN_FLT x5 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								 : 1e-10))
						   ? (lh_qi * (1. / ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
												 ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
												 : 1e-10)))
						   : 1;
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = x9 + x3;
	const GEN_FLT x11 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qk * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
							? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
							: 1e-10;
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = 1 + (-1 * x14);
	const GEN_FLT x16 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qi * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 1;
	const GEN_FLT x17 = sin(x13);
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (obj_qj * (1. / ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
												   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
												   : 1e-10)))
							: 0;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x20 * x11;
	const GEN_FLT x22 = x19 * x17;
	const GEN_FLT x23 = x15 * x16;
	const GEN_FLT x24 = x23 * x11;
	const GEN_FLT x25 =
		((x24 + (-1 * x22)) * sensor_x) + ((x21 + x18) * sensor_y) + ((x14 + (x15 * x12)) * sensor_z) + obj_pz;
	const GEN_FLT x26 = x1 * x4;
	const GEN_FLT x27 = x2 * x7;
	const GEN_FLT x28 = x5 * x27;
	const GEN_FLT x29 = x28 + (-1 * x26);
	const GEN_FLT x30 = x19 * x19;
	const GEN_FLT x31 = x11 * x17;
	const GEN_FLT x32 = x20 * x16;
	const GEN_FLT x33 =
		((x14 + (x30 * x15)) * sensor_y) + ((x32 + x31) * sensor_x) + ((x21 + (-1 * x18)) * sensor_z) + obj_py;
	const GEN_FLT x34 = x5 * x5;
	const GEN_FLT x35 = x6 + (x7 * x34);
	const GEN_FLT x36 = x16 * x16;
	const GEN_FLT x37 =
		((x32 + (-1 * x31)) * sensor_y) + ((x24 + x22) * sensor_z) + ((x14 + (x36 * x15)) * sensor_x) + obj_px;
	const GEN_FLT x38 = (x35 * x37) + (x33 * x29) + (x25 * x10) + lh_px;
	const GEN_FLT x39 = x4 * x4;
	const GEN_FLT x40 = x6 + (x7 * x39);
	const GEN_FLT x41 = x1 * x5;
	const GEN_FLT x42 = x4 * x27;
	const GEN_FLT x43 = x42 + x41;
	const GEN_FLT x44 = x9 + (-1 * x3);
	const GEN_FLT x45 = (x44 * x37) + (x43 * x33) + (x40 * x25) + lh_pz;
	const GEN_FLT x46 = x45 * x45;
	const GEN_FLT x47 = (1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0;
	const GEN_FLT x48 = x6 * x47;
	const GEN_FLT x49 = x2 * x48;
	const GEN_FLT x50 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qj *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x51 = x1 * x50;
	const GEN_FLT x52 = x41 * x47;
	const GEN_FLT x53 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qi *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x54 = x4 * x7;
	const GEN_FLT x55 = (0 < ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
								  ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
								  : 1e-10))
							? (-1 * lh_qk *
							   (1. / (((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10) *
									  ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi)))
										   ? sqrt((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))
										   : 1e-10))) *
							   ((1e-20 < ((lh_qk * lh_qk) + (lh_qj * lh_qj) + (lh_qi * lh_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x56 = (x8 * x55) + (x54 * x53) + (x4 * x52);
	const GEN_FLT x57 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qi *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x58 = (1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0;
	const GEN_FLT x59 = x58 * x17;
	const GEN_FLT x60 = -1 * x59;
	const GEN_FLT x61 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qk *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x62 = x61 * x17;
	const GEN_FLT x63 = x58 * x14;
	const GEN_FLT x64 = x63 * x11;
	const GEN_FLT x65 = (0 < ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
								  ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
								  : 1e-10))
							? (-1 * obj_qj *
							   (1. / (((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10) *
									  ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi)))
										   ? sqrt((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))
										   : 1e-10))) *
							   ((1e-20 < ((obj_qk * obj_qk) + (obj_qj * obj_qj) + (obj_qi * obj_qi))) ? 0 : 0))
							: 0;
	const GEN_FLT x66 = x58 * x19;
	const GEN_FLT x67 = (x57 * x20) + (x66 * x18) + (x65 * x23);
	const GEN_FLT x68 = x65 * x17;
	const GEN_FLT x69 = x63 * x19;
	const GEN_FLT x70 = x15 * x11;
	const GEN_FLT x71 = (x61 * x23) + (x58 * x11 * x18) + (x70 * x57);
	const GEN_FLT x72 = ((x71 + x69 + x68) * sensor_z) + ((x67 + (-1 * x64) + (-1 * x62)) * sensor_y) +
						((x60 + (x59 * x36) + (2 * x57 * x23)) * sensor_x);
	const GEN_FLT x73 = x5 * x48;
	const GEN_FLT x74 = x1 * x53;
	const GEN_FLT x75 = (x50 * x54) + (x55 * x27) + (x2 * x47 * x26);
	const GEN_FLT x76 = x57 * x17;
	const GEN_FLT x77 = x63 * x16;
	const GEN_FLT x78 = (x66 * x31) + (x61 * x20) + (x70 * x65);
	const GEN_FLT x79 = ((x78 + (-1 * x77) + (-1 * x76)) * sensor_z) +
						(((x59 * x30) + (2 * x65 * x20) + x60) * sensor_y) + ((x67 + x64 + x62) * sensor_x);
	const GEN_FLT x80 = x1 * x47;
	const GEN_FLT x81 = -1 * x80;
	const GEN_FLT x82 = (((x59 * x12) + (2 * x70 * x61) + x60) * sensor_z) + ((x78 + x77 + x76) * sensor_y) +
						((x71 + (-1 * x69) + (-1 * x68)) * sensor_x);
	const GEN_FLT x83 = (x82 * x40) + (x25 * ((2 * x54 * x55) + (x80 * x39) + x81)) + (x33 * (x75 + x74 + x73)) +
						(x79 * x43) + (x72 * x44) + (x37 * (x56 + (-1 * x51) + (-1 * x49)));
	const GEN_FLT x84 = x83 * (1. / x46);
	const GEN_FLT x85 = 1. / x45;
	const GEN_FLT x86 = x4 * x48;
	const GEN_FLT x87 = x1 * x55;
	const GEN_FLT x88 = (x2 * x52) + (x8 * x50) + (x53 * x27);
	const GEN_FLT x89 = (x82 * x10) + (x79 * x29) + (x33 * (x88 + (-1 * x87) + (-1 * x86))) +
						(x25 * (x56 + x51 + x49)) + (x72 * x35) + (x37 * ((2 * x8 * x53) + (x80 * x34) + x81));
	const GEN_FLT x90 = x38 * x38;
	const GEN_FLT x91 = -1 * x45;
	const GEN_FLT x92 = atan2(x38, x91);
	const GEN_FLT x93 = 2 * (1. / (x46 + x90)) * ((-1 * x89 * x85) + (x84 * x38)) * x92 * x46 * curve_1;
	const GEN_FLT x94 = x42 + (-1 * x41);
	const GEN_FLT x95 = x2 * x2;
	const GEN_FLT x96 = x6 + (x7 * x95);
	const GEN_FLT x97 = x28 + x26;
	const GEN_FLT x98 = (x97 * x37) + (x96 * x33) + (x94 * x25) + lh_py;
	const GEN_FLT x99 = x46 + (x98 * x98);
	const GEN_FLT x100 = 1. / x99;
	const GEN_FLT x101 = (x25 * (x75 + (-1 * x74) + (-1 * x73))) + (x79 * x96) +
						 (x33 * ((2 * x50 * x27) + (x80 * x95) + x81)) + (x82 * x94) + (x72 * x97) +
						 (x37 * (x88 + x87 + x86));
	const GEN_FLT x102 = -1 * x46 * x100 * ((-1 * x84 * x98) + (x85 * x101));
	const GEN_FLT x103 = 1. / sqrt(x99);
	const GEN_FLT x104 = (x89 * x103 * tilt_1) +
						 (-1.0 / 2.0 * (1. / (x99 * sqrt(x99))) * x38 * tilt_1 * ((2 * x83 * x45) + (2 * x98 * x101)));
	const GEN_FLT x105 = 1. / sqrt(1 + (-1 * x90 * x100 * (tilt_1 * tilt_1)));
	const GEN_FLT x106 = (-1 * x105 * x104) + x102;
	const GEN_FLT x107 = -1 + x106;
	const GEN_FLT x108 = x38 * x103;
	const GEN_FLT x109 =
		1.5707963267949 + (-1 * atan2(-1 * x98, x91)) + (-1 * phase_1) + (-1 * asin(x108 * tilt_1)) + gibPhase_1;
	const GEN_FLT x110 = sin(x109) * gibMag_1;
	const GEN_FLT x111 = (-1 * (x104 + x108) * x105) + x102;
	const GEN_FLT x112 = x106 + x93;
	const GEN_FLT x113 = x112 + (x106 * x110);
	out[0] = x107 + (x107 * x110) + x93;
	out[1] = x111 + (x110 * x111) + x93;
	out[2] = x113 + (x92 * x92);
	out[3] = x112 + (x110 * (1 + x106));
	out[4] = x113 + (-1 * cos(x109));
	out[5] = x113;
	out[6] = x113;
}

/** Applying function <function reproject_gen2 at 0x7f6253d38a70> */
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
	const GEN_FLT x6 = x5 + x4;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = 2 * sqrt(x6 + (obj_qw * obj_qw) + x7);
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = ((x13 + (-1 * x12)) * x14) + (x11 * (x10 + x9)) + ((1 + (-1 * x6 * x8)) * sensor_z) + obj_pz;
	const GEN_FLT x16 = lh_qi * lh_qi;
	const GEN_FLT x17 = lh_qk * lh_qk;
	const GEN_FLT x18 = lh_qj * lh_qj;
	const GEN_FLT x19 = x18 + x17;
	const GEN_FLT x20 = 2 * sqrt(x19 + (lh_qw * lh_qw) + x16);
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 =
		((1 + (-1 * (x5 + x7) * x8)) * sensor_y) + obj_py + ((x24 + x23) * x14) + (x22 * (x10 + (-1 * x9)));
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 =
		((1 + (-1 * (x4 + x7) * x8)) * sensor_x) + ((x24 + (-1 * x23)) * x11) + ((x13 + x12) * x22) + obj_px;
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = (x25 * (1 + (-1 * (x16 + x17) * x20))) + ((x27 + x26) * x29) + ((x3 + (-1 * x2)) * x21) + lh_py;
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = ((x33 + (-1 * x32)) * x29) + ((x3 + x2) * x31) + (x15 * (1 + (-1 * (x16 + x18) * x20))) + lh_pz;
	const GEN_FLT x35 = (x28 * (1 + (-1 * x20 * x19))) + ((x27 + (-1 * x26)) * x31) + ((x33 + x32) * x21) + lh_px;
	const GEN_FLT x36 = (x35 * x35) + (x34 * x34);
	const GEN_FLT x37 = x30 * (1. / sqrt(x36 + (x30 * x30)));
	const GEN_FLT x38 = asin((1. / x1) * x37);
	const GEN_FLT x39 = 0.0028679863 + (x38 * (-8.0108022e-06 + (-8.0108022e-06 * x38)));
	const GEN_FLT x40 = 5.3685255e-06 + (x38 * x39);
	const GEN_FLT x41 = 0.0076069798 + (x40 * x38);
	const GEN_FLT x42 = x30 * (1. / sqrt(x36));
	const GEN_FLT x43 = tan(x0) * x42;
	const GEN_FLT x44 = atan2(-1 * x34, x35);
	const GEN_FLT x45 = (sin(x44 + (-1 * asin(x43)) + ogeeMag_0) * ogeePhase_0) + curve_0;
	const GEN_FLT x46 = asin(
		x43 +
		(x41 * x45 * (x38 * x38) *
		 (1. /
		  (x1 + (-1 * sin(x0) * x45 *
				 ((x38 * (x41 + (x38 * (x40 + (x38 * (x39 + (x38 * (-8.0108022e-06 + (-1.60216044e-05 * x38))))))))) +
				  (x41 * x38)))))));
	const GEN_FLT x47 = -1 * x44;
	const GEN_FLT x48 = -1.5707963267949 + x44;
	const GEN_FLT x49 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x50 = -1 * x42 * tan(x49);
	const GEN_FLT x51 = (sin(x44 + (-1 * asin(x50)) + ogeeMag_1) * ogeePhase_1) + curve_1;
	const GEN_FLT x52 = cos(x49);
	const GEN_FLT x53 = asin((1. / x52) * x37);
	const GEN_FLT x54 = 0.0028679863 + (x53 * (-8.0108022e-06 + (-8.0108022e-06 * x53)));
	const GEN_FLT x55 = 5.3685255e-06 + (x54 * x53);
	const GEN_FLT x56 = 0.0076069798 + (x53 * x55);
	const GEN_FLT x57 = asin(
		x50 +
		((x53 * x53) * x51 * x56 *
		 (1. /
		  (x52 + (x51 * sin(x49) *
				  ((x53 * (x56 + (x53 * (x55 + (x53 * (x54 + (x53 * (-8.0108022e-06 + (-1.60216044e-05 * x53))))))))) +
				   (x53 * x56)))))));
	out[0] = x48 + (-1 * x46) + (-1 * sin(x47 + x46 + (-1 * gibPhase_0)) * gibMag_0) + (-1 * phase_0);
	out[1] = x48 + (-1 * x57) + (-1 * sin(x57 + x47 + (-1 * gibPhase_1)) * gibMag_1) + (-1 * phase_1);
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
	const GEN_FLT x2 = x1 + (-1 * x0);
	const GEN_FLT x3 = obj_qj * obj_qj;
	const GEN_FLT x4 = obj_qi * obj_qi;
	const GEN_FLT x5 = x4 + x3;
	const GEN_FLT x6 = obj_qk * obj_qk;
	const GEN_FLT x7 = x4 + x6;
	const GEN_FLT x8 = sqrt(x7 + (obj_qw * obj_qw) + x3);
	const GEN_FLT x9 = 2 * x8;
	const GEN_FLT x10 = obj_qw * obj_qi;
	const GEN_FLT x11 = obj_qk * obj_qj;
	const GEN_FLT x12 = x11 + x10;
	const GEN_FLT x13 = x9 * sensor_y;
	const GEN_FLT x14 = obj_qw * obj_qj;
	const GEN_FLT x15 = obj_qk * obj_qi;
	const GEN_FLT x16 = x15 + (-1 * x14);
	const GEN_FLT x17 = x9 * sensor_x;
	const GEN_FLT x18 = (x17 * x16) + (x13 * x12) + ((1 + (-1 * x5 * x9)) * sensor_z) + obj_pz;
	const GEN_FLT x19 = lh_qj * lh_qj;
	const GEN_FLT x20 = lh_qk * lh_qk;
	const GEN_FLT x21 = lh_qi * lh_qi;
	const GEN_FLT x22 = x21 + x20;
	const GEN_FLT x23 = sqrt(x22 + (lh_qw * lh_qw) + x19);
	const GEN_FLT x24 = 2 * x23;
	const GEN_FLT x25 = x24 * x18;
	const GEN_FLT x26 = 1 + (-1 * x24 * x22);
	const GEN_FLT x27 = x11 + (-1 * x10);
	const GEN_FLT x28 = x9 * sensor_z;
	const GEN_FLT x29 = obj_qw * obj_qk;
	const GEN_FLT x30 = obj_qj * obj_qi;
	const GEN_FLT x31 = x30 + x29;
	const GEN_FLT x32 = (x31 * x17) + ((1 + (-1 * x7 * x9)) * sensor_y) + obj_py + (x28 * x27);
	const GEN_FLT x33 = x15 + x14;
	const GEN_FLT x34 = x30 + (-1 * x29);
	const GEN_FLT x35 = x3 + x6;
	const GEN_FLT x36 = ((1 + (-1 * x9 * x35)) * sensor_x) + (x34 * x13) + (x33 * x28) + obj_px;
	const GEN_FLT x37 = lh_qw * lh_qk;
	const GEN_FLT x38 = lh_qj * lh_qi;
	const GEN_FLT x39 = x38 + x37;
	const GEN_FLT x40 = x39 * x24;
	const GEN_FLT x41 = (x40 * x36) + (x32 * x26) + (x2 * x25) + lh_py;
	const GEN_FLT x42 = x41 * x41;
	const GEN_FLT x43 = 1 + (-1 * (x21 + x19) * x24);
	const GEN_FLT x44 = x1 + x0;
	const GEN_FLT x45 = x44 * x24;
	const GEN_FLT x46 = lh_qw * lh_qj;
	const GEN_FLT x47 = lh_qk * lh_qi;
	const GEN_FLT x48 = x47 + (-1 * x46);
	const GEN_FLT x49 = x48 * x24;
	const GEN_FLT x50 = (x49 * x36) + (x45 * x32) + (x43 * x18) + lh_pz;
	const GEN_FLT x51 = x47 + x46;
	const GEN_FLT x52 = x38 + (-1 * x37);
	const GEN_FLT x53 = x52 * x24;
	const GEN_FLT x54 = 1 + (-1 * (x19 + x20) * x24);
	const GEN_FLT x55 = (x54 * x36) + (x53 * x32) + (x51 * x25) + lh_px;
	const GEN_FLT x56 = x55 * x55;
	const GEN_FLT x57 = x56 + (x50 * x50);
	const GEN_FLT x58 = x57 + x42;
	const GEN_FLT x59 = 1. / sqrt(x58);
	const GEN_FLT x60 = 0.523598775598299 + tilt_0;
	const GEN_FLT x61 = cos(x60);
	const GEN_FLT x62 = 1. / x61;
	const GEN_FLT x63 = x62 * x59;
	const GEN_FLT x64 = asin(x63 * x41);
	const GEN_FLT x65 = 8.0108022e-06 * x64;
	const GEN_FLT x66 = -8.0108022e-06 + (-1 * x65);
	const GEN_FLT x67 = 0.0028679863 + (x64 * x66);
	const GEN_FLT x68 = 5.3685255e-06 + (x64 * x67);
	const GEN_FLT x69 = 0.0076069798 + (x64 * x68);
	const GEN_FLT x70 = x64 * x69;
	const GEN_FLT x71 = -8.0108022e-06 + (-1.60216044e-05 * x64);
	const GEN_FLT x72 = x67 + (x71 * x64);
	const GEN_FLT x73 = x68 + (x72 * x64);
	const GEN_FLT x74 = x69 + (x73 * x64);
	const GEN_FLT x75 = (x74 * x64) + x70;
	const GEN_FLT x76 = sin(x60);
	const GEN_FLT x77 = tan(x60);
	const GEN_FLT x78 = 1. / sqrt(x57);
	const GEN_FLT x79 = x78 * x41;
	const GEN_FLT x80 = x79 * x77;
	const GEN_FLT x81 = atan2(-1 * x50, x55);
	const GEN_FLT x82 = x81 + (-1 * asin(x80)) + ogeeMag_0;
	const GEN_FLT x83 = (sin(x82) * ogeePhase_0) + curve_0;
	const GEN_FLT x84 = x83 * x76;
	const GEN_FLT x85 = x61 + (-1 * x84 * x75);
	const GEN_FLT x86 = 1. / x85;
	const GEN_FLT x87 = x64 * x64;
	const GEN_FLT x88 = x83 * x87;
	const GEN_FLT x89 = x88 * x86;
	const GEN_FLT x90 = x80 + (x89 * x69);
	const GEN_FLT x91 = 1. / sqrt(1 + (-1 * (x90 * x90)));
	const GEN_FLT x92 = (1. / x58) * x42;
	const GEN_FLT x93 = 1. / sqrt(1 + (-1 * (1. / (x61 * x61)) * x92));
	const GEN_FLT x94 = 4 * x23;
	const GEN_FLT x95 = x94 * x41;
	const GEN_FLT x96 = 2 * x55;
	const GEN_FLT x97 = x50 * x94;
	const GEN_FLT x98 = (x97 * x48) + (x54 * x96);
	const GEN_FLT x99 = x98 + (x95 * x39);
	const GEN_FLT x100 = 1.0 / 2.0 * x41;
	const GEN_FLT x101 = (1. / (x58 * sqrt(x58))) * x100;
	const GEN_FLT x102 = x62 * x101;
	const GEN_FLT x103 = (x63 * x40) + (-1 * x99 * x102);
	const GEN_FLT x104 = x93 * x103;
	const GEN_FLT x105 = x66 * x104;
	const GEN_FLT x106 = (x67 * x104) + (x64 * (x105 + (-1 * x65 * x104)));
	const GEN_FLT x107 = (x64 * x106) + (x68 * x104);
	const GEN_FLT x108 = 1. / x56;
	const GEN_FLT x109 = x50 * x108;
	const GEN_FLT x110 = 1. / x55;
	const GEN_FLT x111 = 1. / x57;
	const GEN_FLT x112 = x56 * x111;
	const GEN_FLT x113 = ((-1 * x49 * x110) + (x54 * x109)) * x112;
	const GEN_FLT x114 = x42 * x111;
	const GEN_FLT x115 = 1. / sqrt(1 + (-1 * (x77 * x77) * x114));
	const GEN_FLT x116 = (1. / (x57 * sqrt(x57))) * x100;
	const GEN_FLT x117 = x77 * x116;
	const GEN_FLT x118 = x78 * x40;
	const GEN_FLT x119 = (x77 * x118) + (-1 * x98 * x117);
	const GEN_FLT x120 = (-1 * x119 * x115) + x113;
	const GEN_FLT x121 = cos(x82) * ogeePhase_0;
	const GEN_FLT x122 = x75 * x76;
	const GEN_FLT x123 = x122 * x121;
	const GEN_FLT x124 = 2.40324066e-05 * x64;
	const GEN_FLT x125 = x73 * x93;
	const GEN_FLT x126 = x88 * (1. / (x85 * x85)) * x69;
	const GEN_FLT x127 = x86 * x87 * x69;
	const GEN_FLT x128 = x121 * x127;
	const GEN_FLT x129 = 2 * x83 * x86 * x70;
	const GEN_FLT x130 =
		x91 * (x119 + (x104 * x129) + (x120 * x128) +
			   (-1 * x126 *
				((-1 * x84 *
				  ((x69 * x104) + (x64 * x107) + (x74 * x104) +
				   (x64 * (x107 + (x103 * x125) +
						   (x64 * ((x72 * x104) + x106 + (x64 * ((x71 * x104) + (-1 * x104 * x124) + x105)))))))) +
				 (-1 * x120 * x123))) +
			   (x89 * x107));
	const GEN_FLT x131 = -1 * x113;
	const GEN_FLT x132 = -1 * x81;
	const GEN_FLT x133 = cos(x132 + asin(x90) + (-1 * gibPhase_0)) * gibMag_0;
	const GEN_FLT x134 = 2 * x50;
	const GEN_FLT x135 = x23 * x108 * x134;
	const GEN_FLT x136 = ((-1 * x45 * x110) + (x52 * x135)) * x112;
	const GEN_FLT x137 = 2 * x41;
	const GEN_FLT x138 = x55 * x94;
	const GEN_FLT x139 = (x97 * x44) + (x52 * x138);
	const GEN_FLT x140 = x139 + (x26 * x137);
	const GEN_FLT x141 = (x63 * x26) + (-1 * x102 * x140);
	const GEN_FLT x142 = x93 * x141;
	const GEN_FLT x143 = x66 * x142;
	const GEN_FLT x144 = (x67 * x142) + (x64 * (x143 + (-1 * x65 * x142)));
	const GEN_FLT x145 = (x64 * x144) + (x68 * x142);
	const GEN_FLT x146 = x116 * x139;
	const GEN_FLT x147 = x78 * x26;
	const GEN_FLT x148 = (x77 * x147) + (-1 * x77 * x146);
	const GEN_FLT x149 = x121 * ((-1 * x115 * x148) + x136);
	const GEN_FLT x150 =
		x91 * (x148 + (x127 * x149) + (x129 * x142) +
			   (-1 * x126 *
				((-1 * x84 *
				  ((x69 * x142) + (x64 * x145) + (x74 * x142) +
				   (x64 * (x145 + (x125 * x141) +
						   (x64 * (x144 + (x72 * x142) + (x64 * ((x71 * x142) + x143 + (-1 * x124 * x142))))))))) +
				 (-1 * x122 * x149))) +
			   (x89 * x145));
	const GEN_FLT x151 = -1 * x136;
	const GEN_FLT x152 = ((-1 * x43 * x110) + (x51 * x135)) * x112;
	const GEN_FLT x153 = (x43 * x134) + (x51 * x138);
	const GEN_FLT x154 = x101 * (x153 + (x2 * x95));
	const GEN_FLT x155 = x2 * x24;
	const GEN_FLT x156 = (x63 * x155) + (-1 * x62 * x154);
	const GEN_FLT x157 = x93 * x156;
	const GEN_FLT x158 = x66 * x157;
	const GEN_FLT x159 = (x67 * x157) + (x64 * (x158 + (-1 * x65 * x157)));
	const GEN_FLT x160 = (x64 * x159) + (x68 * x157);
	const GEN_FLT x161 = x78 * x155;
	const GEN_FLT x162 = (x77 * x161) + (-1 * x117 * x153);
	const GEN_FLT x163 = (-1 * x115 * x162) + x152;
	const GEN_FLT x164 =
		x91 * (x162 + (x129 * x157) +
			   (-1 * x126 *
				((-1 * x84 *
				  ((x64 * x160) +
				   (x64 * (x160 + (x125 * x156) +
						   (x64 * (x159 + (x72 * x157) + (x64 * ((x71 * x157) + (-1 * x124 * x157) + x158)))))) +
				   (x69 * x157) + (x74 * x157))) +
				 (-1 * x123 * x163))) +
			   (x128 * x163) + (x89 * x160));
	const GEN_FLT x165 = -1 * x152;
	const GEN_FLT x166 = 2 * (1. / x8);
	const GEN_FLT x167 = x35 * x166;
	const GEN_FLT x168 = x167 * sensor_x;
	const GEN_FLT x169 = x166 * obj_qw;
	const GEN_FLT x170 = x34 * sensor_y;
	const GEN_FLT x171 = x13 * obj_qk;
	const GEN_FLT x172 = x33 * sensor_z;
	const GEN_FLT x173 = x28 * obj_qj;
	const GEN_FLT x174 = x173 + (x169 * x172) + (-1 * x171) + (x169 * x170) + (-1 * x168 * obj_qw);
	const GEN_FLT x175 = x31 * sensor_x;
	const GEN_FLT x176 = x7 * x166;
	const GEN_FLT x177 = x176 * sensor_y;
	const GEN_FLT x178 = x17 * obj_qk;
	const GEN_FLT x179 = x27 * sensor_z;
	const GEN_FLT x180 = x28 * obj_qi;
	const GEN_FLT x181 = (-1 * x180) + (x169 * x179) + x178 + (-1 * x177 * obj_qw) + (x169 * x175);
	const GEN_FLT x182 = x17 * obj_qj;
	const GEN_FLT x183 = x16 * sensor_x;
	const GEN_FLT x184 = x12 * sensor_y;
	const GEN_FLT x185 = x166 * x184;
	const GEN_FLT x186 = x13 * obj_qi;
	const GEN_FLT x187 = x5 * x166;
	const GEN_FLT x188 = x187 * sensor_z;
	const GEN_FLT x189 = (-1 * x188 * obj_qw) + x186 + (x185 * obj_qw) + (x169 * x183) + (-1 * x182);
	const GEN_FLT x190 = (x189 * x155) + (x26 * x181) + (x40 * x174);
	const GEN_FLT x191 = x51 * x24;
	const GEN_FLT x192 = (x189 * x191) + (x53 * x181) + (x54 * x174);
	const GEN_FLT x193 = (x43 * x189) + (x45 * x181) + (x49 * x174);
	const GEN_FLT x194 = (x193 * x134) + (x96 * x192);
	const GEN_FLT x195 = x194 + (x190 * x137);
	const GEN_FLT x196 = (x63 * x190) + (-1 * x102 * x195);
	const GEN_FLT x197 = x93 * x196;
	const GEN_FLT x198 = x66 * x197;
	const GEN_FLT x199 = (x67 * x197) + (x64 * (x198 + (-1 * x65 * x197)));
	const GEN_FLT x200 = (x64 * x199) + (x68 * x197);
	const GEN_FLT x201 = ((-1 * x110 * x193) + (x109 * x192)) * x112;
	const GEN_FLT x202 = x78 * x190;
	const GEN_FLT x203 = (x77 * x202) + (-1 * x117 * x194);
	const GEN_FLT x204 = (-1 * x203 * x115) + x201;
	const GEN_FLT x205 =
		x91 * ((x129 * x197) + x203 + (x204 * x128) +
			   (-1 * x126 *
				((-1 * x84 *
				  ((x69 * x197) + (x64 * x200) + (x74 * x197) +
				   (x64 * (x200 + (x125 * x196) +
						   (x64 * (x199 + (x72 * x197) + (x64 * ((x71 * x197) + (-1 * x124 * x197) + x198)))))))) +
				 (-1 * x204 * x123))) +
			   (x89 * x200));
	const GEN_FLT x206 = -1 * x201;
	const GEN_FLT x207 = x166 * obj_qi;
	const GEN_FLT x208 = x13 * obj_qj;
	const GEN_FLT x209 = x28 * obj_qk;
	const GEN_FLT x210 = x209 + (x207 * x172) + x208 + (x207 * x170) + (-1 * x168 * obj_qi);
	const GEN_FLT x211 = 4 * x8;
	const GEN_FLT x212 = -1 * x211 * obj_qi;
	const GEN_FLT x213 = x28 * obj_qw;
	const GEN_FLT x214 =
		(-1 * x213) + (x207 * x179) + ((x212 + (-1 * x176 * obj_qi)) * sensor_y) + x182 + (x207 * x175);
	const GEN_FLT x215 = x13 * obj_qw;
	const GEN_FLT x216 = ((x212 + (-1 * x187 * obj_qi)) * sensor_z) + x215 + (x185 * obj_qi) + x178 + (x207 * x183);
	const GEN_FLT x217 = (x216 * x191) + (x53 * x214) + (x54 * x210);
	const GEN_FLT x218 = (x43 * x216) + (x45 * x214) + (x49 * x210);
	const GEN_FLT x219 = ((-1 * x218 * x110) + (x217 * x109)) * x112;
	const GEN_FLT x220 = (x216 * x155) + (x26 * x214) + (x40 * x210);
	const GEN_FLT x221 = (x218 * x134) + (x96 * x217);
	const GEN_FLT x222 = x221 + (x220 * x137);
	const GEN_FLT x223 = (x63 * x220) + (-1 * x222 * x102);
	const GEN_FLT x224 = x93 * x223;
	const GEN_FLT x225 = x66 * x224;
	const GEN_FLT x226 = (x67 * x224) + (x64 * (x225 + (-1 * x65 * x224)));
	const GEN_FLT x227 = (x64 * x226) + (x68 * x224);
	const GEN_FLT x228 = x78 * x220;
	const GEN_FLT x229 = (x77 * x228) + (-1 * x221 * x117);
	const GEN_FLT x230 = (-1 * x229 * x115) + x219;
	const GEN_FLT x231 =
		x91 * (x229 + (x224 * x129) + (x230 * x128) +
			   (-1 * x126 *
				((-1 * x84 *
				  ((x69 * x224) + (x64 * x227) + (x74 * x224) +
				   (x64 * (x227 + (x223 * x125) +
						   (x64 * (x226 + (x72 * x224) + (x64 * ((x71 * x224) + (-1 * x224 * x124) + x225)))))))) +
				 (-1 * x230 * x123))) +
			   (x89 * x227));
	const GEN_FLT x232 = -1 * x219;
	const GEN_FLT x233 = -1 * x211 * obj_qj;
	const GEN_FLT x234 = x166 * obj_qj;
	const GEN_FLT x235 = x213 + (x234 * x172) + x186 + (x234 * x170) + ((x233 + (-1 * x167 * obj_qj)) * sensor_x);
	const GEN_FLT x236 = x17 * obj_qi;
	const GEN_FLT x237 = x209 + (x234 * x179) + (-1 * x177 * obj_qj) + x236 + (x234 * x175);
	const GEN_FLT x238 = x17 * obj_qw;
	const GEN_FLT x239 =
		((x233 + (-1 * x187 * obj_qj)) * sensor_z) + (x185 * obj_qj) + (-1 * x238) + x171 + (x234 * x183);
	const GEN_FLT x240 = (x239 * x191) + (x53 * x237) + (x54 * x235);
	const GEN_FLT x241 = (x43 * x239) + (x45 * x237) + (x49 * x235);
	const GEN_FLT x242 = ((-1 * x241 * x110) + (x240 * x109)) * x112;
	const GEN_FLT x243 = (x239 * x155) + (x26 * x237) + (x40 * x235);
	const GEN_FLT x244 = (x241 * x134) + (x96 * x240);
	const GEN_FLT x245 = x101 * (x244 + (x243 * x137));
	const GEN_FLT x246 = x59 * x243;
	const GEN_FLT x247 = (x62 * x246) + (-1 * x62 * x245);
	const GEN_FLT x248 = x93 * x247;
	const GEN_FLT x249 = x66 * x248;
	const GEN_FLT x250 = (x67 * x248) + (x64 * (x249 + (-1 * x65 * x248)));
	const GEN_FLT x251 = (x64 * x250) + (x68 * x248);
	const GEN_FLT x252 = x78 * x243;
	const GEN_FLT x253 = (x77 * x252) + (-1 * x244 * x117);
	const GEN_FLT x254 = (-1 * x253 * x115) + x242;
	const GEN_FLT x255 =
		x91 * ((x254 * x128) + x253 + (x248 * x129) +
			   (-1 * x126 *
				((-1 * x84 *
				  ((x74 * x248) + (x69 * x248) + (x64 * x251) +
				   (x64 * (x251 + (x247 * x125) +
						   (x64 * (x250 + (x72 * x248) + (x64 * ((x71 * x248) + (-1 * x248 * x124) + x249)))))))) +
				 (-1 * x254 * x123))) +
			   (x89 * x251));
	const GEN_FLT x256 = -1 * x242;
	const GEN_FLT x257 = x166 * obj_qk;
	const GEN_FLT x258 = -1 * x211 * obj_qk;
	const GEN_FLT x259 =
		x180 + (x257 * x172) + ((x258 + (-1 * x167 * obj_qk)) * sensor_x) + (-1 * x215) + (x257 * x170);
	const GEN_FLT x260 = x173 + (x257 * x179) + ((x258 + (-1 * x176 * obj_qk)) * sensor_y) + x238 + (x257 * x175);
	const GEN_FLT x261 = x208 + (x257 * x183) + (x257 * x184) + (-1 * x188 * obj_qk) + x236;
	const GEN_FLT x262 = (x261 * x191) + (x53 * x260) + (x54 * x259);
	const GEN_FLT x263 = (x43 * x261) + (x45 * x260) + (x49 * x259);
	const GEN_FLT x264 = ((-1 * x263 * x110) + (x262 * x109)) * x112;
	const GEN_FLT x265 = (x261 * x155) + (x26 * x260) + (x40 * x259);
	const GEN_FLT x266 = (x263 * x134) + (x96 * x262);
	const GEN_FLT x267 = x266 + (x265 * x137);
	const GEN_FLT x268 = (x63 * x265) + (-1 * x267 * x102);
	const GEN_FLT x269 = x93 * x268;
	const GEN_FLT x270 = x66 * x269;
	const GEN_FLT x271 = (x67 * x269) + (x64 * (x270 + (-1 * x65 * x269)));
	const GEN_FLT x272 = (x64 * x271) + (x68 * x269);
	const GEN_FLT x273 = x78 * x265;
	const GEN_FLT x274 = (x77 * x273) + (-1 * x266 * x117);
	const GEN_FLT x275 = (-1 * x274 * x115) + x264;
	const GEN_FLT x276 =
		x91 * (x274 + (x269 * x129) + (x275 * x128) +
			   (-1 * x126 *
				((-1 * x84 *
				  ((x69 * x269) + (x64 * x272) + (x74 * x269) +
				   (x64 * (x272 + (x268 * x125) +
						   (x64 * ((x72 * x269) + x271 + (x64 * ((-1 * x269 * x124) + (x71 * x269) + x270)))))))) +
				 (-1 * x275 * x123))) +
			   (x89 * x272));
	const GEN_FLT x277 = -1 * x264;
	const GEN_FLT x278 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x279 = cos(x278);
	const GEN_FLT x280 = 1. / x279;
	const GEN_FLT x281 = x59 * x280;
	const GEN_FLT x282 = asin(x41 * x281);
	const GEN_FLT x283 = 8.0108022e-06 * x282;
	const GEN_FLT x284 = -8.0108022e-06 + (-1 * x283);
	const GEN_FLT x285 = 0.0028679863 + (x282 * x284);
	const GEN_FLT x286 = 5.3685255e-06 + (x282 * x285);
	const GEN_FLT x287 = 0.0076069798 + (x286 * x282);
	const GEN_FLT x288 = x287 * x282;
	const GEN_FLT x289 = -8.0108022e-06 + (-1.60216044e-05 * x282);
	const GEN_FLT x290 = x285 + (x289 * x282);
	const GEN_FLT x291 = x286 + (x290 * x282);
	const GEN_FLT x292 = x287 + (x291 * x282);
	const GEN_FLT x293 = (x292 * x282) + x288;
	const GEN_FLT x294 = tan(x278);
	const GEN_FLT x295 = -1 * x79 * x294;
	const GEN_FLT x296 = x81 + (-1 * asin(x295)) + ogeeMag_1;
	const GEN_FLT x297 = (sin(x296) * ogeePhase_1) + curve_1;
	const GEN_FLT x298 = sin(x278);
	const GEN_FLT x299 = x297 * x298;
	const GEN_FLT x300 = x279 + (x299 * x293);
	const GEN_FLT x301 = 1. / x300;
	const GEN_FLT x302 = x282 * x282;
	const GEN_FLT x303 = x297 * x302;
	const GEN_FLT x304 = x301 * x303;
	const GEN_FLT x305 = x295 + (x287 * x304);
	const GEN_FLT x306 = 1. / sqrt(1 + (-1 * (x305 * x305)));
	const GEN_FLT x307 = 1. / sqrt(1 + (-1 * (x294 * x294) * x114));
	const GEN_FLT x308 = x294 * x116;
	const GEN_FLT x309 = (-1 * x294 * x118) + (x98 * x308);
	const GEN_FLT x310 = (-1 * x309 * x307) + x113;
	const GEN_FLT x311 = cos(x296) * ogeePhase_1;
	const GEN_FLT x312 = x287 * x301 * x302;
	const GEN_FLT x313 = x312 * x311;
	const GEN_FLT x314 = 1. / sqrt(1 + (-1 * x92 * (1. / (x279 * x279))));
	const GEN_FLT x315 = x280 * x101;
	const GEN_FLT x316 = (x40 * x281) + (-1 * x99 * x315);
	const GEN_FLT x317 = x314 * x316;
	const GEN_FLT x318 = 2 * x297 * x288 * x301;
	const GEN_FLT x319 = x298 * x293;
	const GEN_FLT x320 = x311 * x319;
	const GEN_FLT x321 = x284 * x317;
	const GEN_FLT x322 = 2.40324066e-05 * x282;
	const GEN_FLT x323 = (x282 * (x321 + (-1 * x283 * x317))) + (x285 * x317);
	const GEN_FLT x324 = x286 * x314;
	const GEN_FLT x325 = (x324 * x316) + (x282 * x323);
	const GEN_FLT x326 = x287 * x314;
	const GEN_FLT x327 = x287 * (1. / (x300 * x300)) * x303;
	const GEN_FLT x328 =
		x306 *
		((x304 * x325) + x309 +
		 (-1 * x327 *
		  ((x299 *
			((x282 * x325) + (x326 * x316) + (x292 * x317) +
			 (x282 * (x325 + (x291 * x317) +
					  (x282 * (x323 + (x290 * x317) + (x282 * ((x289 * x317) + (-1 * x322 * x317) + x321)))))))) +
		   (x310 * x320))) +
		 (x317 * x318) + (x313 * x310));
	const GEN_FLT x329 = cos(x132 + asin(x305) + (-1 * gibPhase_1)) * gibMag_1;
	const GEN_FLT x330 = (-1 * x294 * x147) + (x294 * x146);
	const GEN_FLT x331 = (-1 * x307 * x330) + x136;
	const GEN_FLT x332 = (x26 * x281) + (-1 * x315 * x140);
	const GEN_FLT x333 = x314 * x332;
	const GEN_FLT x334 = x284 * x333;
	const GEN_FLT x335 = (x282 * (x334 + (-1 * x283 * x333))) + (x285 * x333);
	const GEN_FLT x336 = (x324 * x332) + (x282 * x335);
	const GEN_FLT x337 =
		x306 *
		((x333 * x318) + x330 + (x304 * x336) +
		 (-1 * x327 *
		  ((x299 *
			((x282 * x336) + (x326 * x332) + (x292 * x333) +
			 (x282 * (x336 + (x291 * x333) +
					  (x282 * (x335 + (x290 * x333) + (x282 * ((x289 * x333) + x334 + (-1 * x322 * x333))))))))) +
		   (x320 * x331))) +
		 (x313 * x331));
	const GEN_FLT x338 = (-1 * x294 * x161) + (x308 * x153);
	const GEN_FLT x339 = x311 * ((-1 * x307 * x338) + x152);
	const GEN_FLT x340 = (x281 * x155) + (-1 * x280 * x154);
	const GEN_FLT x341 = x314 * x340;
	const GEN_FLT x342 = x284 * x341;
	const GEN_FLT x343 = (x282 * (x342 + (-1 * x283 * x341))) + (x285 * x341);
	const GEN_FLT x344 = (x324 * x340) + (x282 * x343);
	const GEN_FLT x345 =
		x306 *
		(x338 +
		 (-1 * x327 *
		  ((x299 *
			((x282 * x344) + (x326 * x340) + (x292 * x341) +
			 (x282 * (x344 + (x291 * x341) +
					  (x282 * (x343 + (x290 * x341) + (x282 * ((x289 * x341) + (-1 * x322 * x341) + x342)))))))) +
		   (x339 * x319))) +
		 (x318 * x341) + (x304 * x344) + (x339 * x312));
	const GEN_FLT x346 = (-1 * x202 * x294) + (x308 * x194);
	const GEN_FLT x347 = (-1 * x307 * x346) + x201;
	const GEN_FLT x348 = ((x281 * x190) + (-1 * x315 * x195)) * x314;
	const GEN_FLT x349 = x284 * x348;
	const GEN_FLT x350 = (x282 * (x349 + (-1 * x283 * x348))) + (x285 * x348);
	const GEN_FLT x351 = (x286 * x348) + (x282 * x350);
	const GEN_FLT x352 =
		x306 *
		(x346 +
		 (-1 * x327 *
		  ((x299 *
			((x282 * x351) + (x292 * x348) + (x287 * x348) +
			 (x282 * (x351 + (x291 * x348) +
					  (x282 * (x350 + (x290 * x348) + (x282 * ((x289 * x348) + (-1 * x322 * x348) + x349)))))))) +
		   (x320 * x347))) +
		 (x348 * x318) + (x351 * x304) + (x313 * x347));
	const GEN_FLT x353 = (-1 * x294 * x228) + (x221 * x308);
	const GEN_FLT x354 = (-1 * x353 * x307) + x219;
	const GEN_FLT x355 = (x281 * x220) + (-1 * x222 * x315);
	const GEN_FLT x356 = x355 * x314;
	const GEN_FLT x357 = x284 * x356;
	const GEN_FLT x358 = (x282 * (x357 + (-1 * x283 * x356))) + (x285 * x356);
	const GEN_FLT x359 = (x355 * x324) + (x282 * x358);
	const GEN_FLT x360 =
		x306 *
		(x353 + (x359 * x304) +
		 (-1 * x327 *
		  ((x299 * ((x282 * x359) +
					(x282 * (x359 + (x291 * x356) +
							 (x282 * ((x290 * x356) + x358 + (x282 * ((x289 * x356) + (-1 * x356 * x322) + x357)))))) +
					(x287 * x356) + (x292 * x356))) +
		   (x354 * x320))) +
		 (x356 * x318) + (x354 * x313));
	const GEN_FLT x361 = (-1 * x294 * x252) + (x244 * x308);
	const GEN_FLT x362 = (-1 * x361 * x307) + x242;
	const GEN_FLT x363 = (x280 * x246) + (-1 * x280 * x245);
	const GEN_FLT x364 = x363 * x314;
	const GEN_FLT x365 = x284 * x364;
	const GEN_FLT x366 = (x282 * (x365 + (-1 * x283 * x364))) + (x285 * x364);
	const GEN_FLT x367 = (x363 * x324) + (x282 * x366);
	const GEN_FLT x368 =
		x306 *
		(x361 + (x367 * x304) +
		 (-1 * x327 *
		  ((x299 *
			((x282 * x367) + (x363 * x326) + (x292 * x364) +
			 (x282 * (x367 + (x291 * x364) +
					  (x282 * (x366 + (x290 * x364) + (x282 * ((x289 * x364) + (-1 * x364 * x322) + x365)))))))) +
		   (x362 * x320))) +
		 (x364 * x318) + (x362 * x313));
	const GEN_FLT x369 = (-1 * x273 * x294) + (x266 * x308);
	const GEN_FLT x370 = (-1 * x369 * x307) + x264;
	const GEN_FLT x371 = ((x265 * x281) + (-1 * x267 * x315)) * x314;
	const GEN_FLT x372 = x284 * x371;
	const GEN_FLT x373 = (x282 * (x372 + (-1 * x283 * x371))) + (x285 * x371);
	const GEN_FLT x374 = (x286 * x371) + (x282 * x373);
	const GEN_FLT x375 =
		x306 *
		((x374 * x304) + x369 +
		 (-1 * x327 *
		  ((x299 *
			((x282 * x374) + (x287 * x371) + (x292 * x371) +
			 (x282 * (x374 + (x291 * x371) +
					  (x282 * (x373 + (x290 * x371) + (x282 * ((x289 * x371) + (-1 * x371 * x322) + x372)))))))) +
		   (x370 * x320))) +
		 (x371 * x318) + (x370 * x313));
	out[0] = (-1 * (x130 + x131) * x133) + x113 + (-1 * x130);
	out[1] = (-1 * (x150 + x151) * x133) + (-1 * x150) + x136;
	out[2] = (-1 * (x164 + x165) * x133) + (-1 * x164) + x152;
	out[3] = (-1 * (x205 + x206) * x133) + x201 + (-1 * x205);
	out[4] = (-1 * (x231 + x232) * x133) + (-1 * x231) + x219;
	out[5] = (-1 * (x255 + x256) * x133) + (-1 * x255) + x242;
	out[6] = (-1 * x276) + (-1 * (x276 + x277) * x133) + x264;
	out[7] = (-1 * (x328 + x131) * x329) + (-1 * x328) + x113;
	out[8] = (-1 * (x337 + x151) * x329) + (-1 * x337) + x136;
	out[9] = (-1 * (x345 + x165) * x329) + (-1 * x345) + x152;
	out[10] = (-1 * (x352 + x206) * x329) + (-1 * x352) + x201;
	out[11] = (-1 * (x360 + x232) * x329) + (-1 * x360) + x219;
	out[12] = (-1 * (x368 + x256) * x329) + (-1 * x368) + x242;
	out[13] = (-1 * (x375 + x277) * x329) + (-1 * x375) + x264;
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
	const GEN_FLT x3 = x2 + x0;
	const GEN_FLT x4 = sqrt(x3 + (lh_qw * lh_qw) + x1);
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * (x1 + x0) * x5);
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = obj_qj * obj_qj;
	const GEN_FLT x9 = obj_qi * obj_qi;
	const GEN_FLT x10 = x9 + x7;
	const GEN_FLT x11 = sqrt(x10 + (obj_qw * obj_qw) + x8);
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 + (-1 * (x8 + x7) * x12);
	const GEN_FLT x14 = lh_qw * lh_qk;
	const GEN_FLT x15 = lh_qj * lh_qi;
	const GEN_FLT x16 = x15 + (-1 * x14);
	const GEN_FLT x17 = obj_qw * obj_qk;
	const GEN_FLT x18 = obj_qj * obj_qi;
	const GEN_FLT x19 = x18 + x17;
	const GEN_FLT x20 = 4 * x4 * x11;
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = obj_qw * obj_qj;
	const GEN_FLT x23 = obj_qk * obj_qi;
	const GEN_FLT x24 = x23 + (-1 * x22);
	const GEN_FLT x25 = lh_qw * lh_qj;
	const GEN_FLT x26 = lh_qk * lh_qi;
	const GEN_FLT x27 = x26 + x25;
	const GEN_FLT x28 = x20 * x27;
	const GEN_FLT x29 = (x24 * x28) + (x21 * x16) + (x6 * x13);
	const GEN_FLT x30 = 1 + (-1 * (x2 + x1) * x5);
	const GEN_FLT x31 = 1 + (-1 * (x9 + x8) * x12);
	const GEN_FLT x32 = obj_qw * obj_qi;
	const GEN_FLT x33 = obj_qk * obj_qj;
	const GEN_FLT x34 = x33 + x32;
	const GEN_FLT x35 = x12 * sensor_y;
	const GEN_FLT x36 = x12 * sensor_x;
	const GEN_FLT x37 = (x36 * x24) + (x34 * x35) + (x31 * sensor_z) + obj_pz;
	const GEN_FLT x38 = lh_qw * lh_qi;
	const GEN_FLT x39 = lh_qk * lh_qj;
	const GEN_FLT x40 = x39 + x38;
	const GEN_FLT x41 = x33 + (-1 * x32);
	const GEN_FLT x42 = x12 * sensor_z;
	const GEN_FLT x43 = 1 + (-1 * x12 * x10);
	const GEN_FLT x44 = (x36 * x19) + (x43 * sensor_y) + obj_py + (x41 * x42);
	const GEN_FLT x45 = x5 * x44;
	const GEN_FLT x46 = x23 + x22;
	const GEN_FLT x47 = x18 + (-1 * x17);
	const GEN_FLT x48 = (x13 * sensor_x) + (x47 * x35) + (x42 * x46) + obj_px;
	const GEN_FLT x49 = x26 + (-1 * x25);
	const GEN_FLT x50 = x5 * x49;
	const GEN_FLT x51 = (x50 * x48) + (x40 * x45) + (x30 * x37) + lh_pz;
	const GEN_FLT x52 = x5 * x37;
	const GEN_FLT x53 = (x6 * x48) + (x45 * x16) + (x52 * x27) + lh_px;
	const GEN_FLT x54 = x53 * x53;
	const GEN_FLT x55 = (1. / x54) * x51;
	const GEN_FLT x56 = 1. / x53;
	const GEN_FLT x57 = x30 * x12;
	const GEN_FLT x58 = (x57 * x24) + (x40 * x21) + (x50 * x13);
	const GEN_FLT x59 = x54 + (x51 * x51);
	const GEN_FLT x60 = 1. / x59;
	const GEN_FLT x61 = x60 * x54;
	const GEN_FLT x62 = ((-1 * x58 * x56) + (x55 * x29)) * x61;
	const GEN_FLT x63 = 0.523598775598299 + tilt_0;
	const GEN_FLT x64 = cos(x63);
	const GEN_FLT x65 = 1. / x64;
	const GEN_FLT x66 = x39 + (-1 * x38);
	const GEN_FLT x67 = 1 + (-1 * x3 * x5);
	const GEN_FLT x68 = x15 + x14;
	const GEN_FLT x69 = x5 * x68;
	const GEN_FLT x70 = (x67 * x44) + (x69 * x48) + (x66 * x52) + lh_py;
	const GEN_FLT x71 = x70 * x70;
	const GEN_FLT x72 = x59 + x71;
	const GEN_FLT x73 = 1. / sqrt(x72);
	const GEN_FLT x74 = x70 * x73;
	const GEN_FLT x75 = asin(x74 * x65);
	const GEN_FLT x76 = 8.0108022e-06 * x75;
	const GEN_FLT x77 = -8.0108022e-06 + (-1 * x76);
	const GEN_FLT x78 = 0.0028679863 + (x75 * x77);
	const GEN_FLT x79 = 5.3685255e-06 + (x78 * x75);
	const GEN_FLT x80 = 0.0076069798 + (x79 * x75);
	const GEN_FLT x81 = x80 * x75;
	const GEN_FLT x82 = -8.0108022e-06 + (-1.60216044e-05 * x75);
	const GEN_FLT x83 = x78 + (x82 * x75);
	const GEN_FLT x84 = x79 + (x83 * x75);
	const GEN_FLT x85 = x80 + (x84 * x75);
	const GEN_FLT x86 = (x85 * x75) + x81;
	const GEN_FLT x87 = sin(x63);
	const GEN_FLT x88 = tan(x63);
	const GEN_FLT x89 = 1. / sqrt(x59);
	const GEN_FLT x90 = x88 * x89;
	const GEN_FLT x91 = x70 * x90;
	const GEN_FLT x92 = atan2(-1 * x51, x53);
	const GEN_FLT x93 = x92 + (-1 * asin(x91)) + ogeeMag_0;
	const GEN_FLT x94 = (sin(x93) * ogeePhase_0) + curve_0;
	const GEN_FLT x95 = x87 * x94;
	const GEN_FLT x96 = x64 + (-1 * x86 * x95);
	const GEN_FLT x97 = 1. / x96;
	const GEN_FLT x98 = x75 * x75;
	const GEN_FLT x99 = x98 * x94;
	const GEN_FLT x100 = x99 * x97;
	const GEN_FLT x101 = x91 + (x80 * x100);
	const GEN_FLT x102 = 1. / sqrt(1 + (-1 * (x101 * x101)));
	const GEN_FLT x103 = x71 * (1. / x72);
	const GEN_FLT x104 = 1. / sqrt(1 + (-1 * (1. / (x64 * x64)) * x103));
	const GEN_FLT x105 = x67 * x12;
	const GEN_FLT x106 = x66 * x20;
	const GEN_FLT x107 = (x24 * x106) + (x19 * x105) + (x69 * x13);
	const GEN_FLT x108 = 2 * x70;
	const GEN_FLT x109 = 2 * x53;
	const GEN_FLT x110 = 2 * x51;
	const GEN_FLT x111 = (x58 * x110) + (x29 * x109);
	const GEN_FLT x112 = 1.0 / 2.0 * x70;
	const GEN_FLT x113 = (1. / (x72 * sqrt(x72))) * x112;
	const GEN_FLT x114 = x113 * (x111 + (x108 * x107));
	const GEN_FLT x115 = x73 * x107;
	const GEN_FLT x116 = ((x65 * x115) + (-1 * x65 * x114)) * x104;
	const GEN_FLT x117 = x77 * x116;
	const GEN_FLT x118 = (x78 * x116) + (x75 * (x117 + (-1 * x76 * x116)));
	const GEN_FLT x119 = (x75 * x118) + (x79 * x116);
	const GEN_FLT x120 = 2 * x81 * x97 * x94;
	const GEN_FLT x121 = (1. / (x59 * sqrt(x59))) * x112;
	const GEN_FLT x122 = x88 * x121;
	const GEN_FLT x123 = (x90 * x107) + (-1 * x111 * x122);
	const GEN_FLT x124 = x71 * x60;
	const GEN_FLT x125 = 1. / sqrt(1 + (-1 * (x88 * x88) * x124));
	const GEN_FLT x126 = cos(x93) * ogeePhase_0;
	const GEN_FLT x127 = x126 * ((-1 * x123 * x125) + x62);
	const GEN_FLT x128 = x86 * x87;
	const GEN_FLT x129 = 2.40324066e-05 * x75;
	const GEN_FLT x130 = x80 * x99 * (1. / (x96 * x96));
	const GEN_FLT x131 = x80 * x98 * x97;
	const GEN_FLT x132 =
		x102 * ((x127 * x131) + x123 +
				(-1 * x130 *
				 ((-1 * x95 *
				   ((x80 * x116) + (x75 * x119) + (x85 * x116) +
					(x75 * (x119 + (x84 * x116) +
							(x75 * (x118 + (x83 * x116) + (x75 * ((x82 * x116) + (-1 * x116 * x129) + x117)))))))) +
				  (-1 * x128 * x127))) +
				(x116 * x120) + (x100 * x119));
	const GEN_FLT x133 = -1 * x62;
	const GEN_FLT x134 = -1 * x92;
	const GEN_FLT x135 = cos(x134 + asin(x101) + (-1 * gibPhase_0)) * gibMag_0;
	const GEN_FLT x136 = x5 * x43;
	const GEN_FLT x137 = x6 * x12;
	const GEN_FLT x138 = (x34 * x28) + (x47 * x137) + (x16 * x136);
	const GEN_FLT x139 = x47 * x20;
	const GEN_FLT x140 = (x57 * x34) + (x40 * x136) + (x49 * x139);
	const GEN_FLT x141 = ((-1 * x56 * x140) + (x55 * x138)) * x61;
	const GEN_FLT x142 = (x34 * x106) + (x67 * x43) + (x68 * x139);
	const GEN_FLT x143 = (x110 * x140) + (x109 * x138);
	const GEN_FLT x144 = x143 + (x108 * x142);
	const GEN_FLT x145 = x65 * x113;
	const GEN_FLT x146 = x73 * x142;
	const GEN_FLT x147 = x104 * ((x65 * x146) + (-1 * x144 * x145));
	const GEN_FLT x148 = x77 * x147;
	const GEN_FLT x149 = (x78 * x147) + (x75 * (x148 + (-1 * x76 * x147)));
	const GEN_FLT x150 = (x75 * x149) + (x79 * x147);
	const GEN_FLT x151 = x89 * x142;
	const GEN_FLT x152 = (x88 * x151) + (-1 * x122 * x143);
	const GEN_FLT x153 = (-1 * x125 * x152) + x141;
	const GEN_FLT x154 = x128 * x126;
	const GEN_FLT x155 = x126 * x131;
	const GEN_FLT x156 =
		x102 * (x152 + (x120 * x147) +
				(-1 * x130 *
				 ((-1 * x95 *
				   ((x80 * x147) + (x75 * x150) + (x85 * x147) +
					(x75 * (x150 + (x84 * x147) +
							(x75 * (x149 + (x83 * x147) + (x75 * ((x82 * x147) + (-1 * x129 * x147) + x148)))))))) +
				  (-1 * x153 * x154))) +
				(x153 * x155) + (x100 * x150));
	const GEN_FLT x157 = -1 * x141;
	const GEN_FLT x158 = x41 * x20;
	const GEN_FLT x159 = x5 * x31;
	const GEN_FLT x160 = (x27 * x159) + (x16 * x158) + (x46 * x137);
	const GEN_FLT x161 = x46 * x20;
	const GEN_FLT x162 = (x30 * x31) + (x40 * x158) + (x49 * x161);
	const GEN_FLT x163 = ((-1 * x56 * x162) + (x55 * x160)) * x61;
	const GEN_FLT x164 = (x66 * x159) + (x41 * x105) + (x68 * x161);
	const GEN_FLT x165 = (x110 * x162) + (x109 * x160);
	const GEN_FLT x166 = x165 + (x108 * x164);
	const GEN_FLT x167 = x73 * x164;
	const GEN_FLT x168 = x104 * ((x65 * x167) + (-1 * x166 * x145));
	const GEN_FLT x169 = x77 * x168;
	const GEN_FLT x170 = (x78 * x168) + (x75 * (x169 + (-1 * x76 * x168)));
	const GEN_FLT x171 = (x75 * x170) + (x79 * x168);
	const GEN_FLT x172 = x89 * x164;
	const GEN_FLT x173 = (x88 * x172) + (-1 * x122 * x165);
	const GEN_FLT x174 = (-1 * x125 * x173) + x163;
	const GEN_FLT x175 =
		x102 * (x173 + (x174 * x155) +
				(-1 * x130 *
				 ((-1 * x95 *
				   ((x80 * x168) + (x75 * x171) + (x85 * x168) +
					(x75 * (x171 + (x84 * x168) +
							(x75 * (x170 + (x83 * x168) + (x75 * ((x82 * x168) + (-1 * x129 * x168) + x169)))))))) +
				  (-1 * x174 * x154))) +
				(x120 * x168) + (x100 * x171));
	const GEN_FLT x176 = -1 * x163;
	const GEN_FLT x177 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x178 = cos(x177);
	const GEN_FLT x179 = 1. / x178;
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
	const GEN_FLT x190 = x92 + (-1 * asin(x189)) + ogeeMag_1;
	const GEN_FLT x191 = (sin(x190) * ogeePhase_1) + curve_1;
	const GEN_FLT x192 = x180 * x185;
	const GEN_FLT x193 = -8.0108022e-06 + (-1.60216044e-05 * x180);
	const GEN_FLT x194 = x183 + (x180 * x193);
	const GEN_FLT x195 = x184 + (x180 * x194);
	const GEN_FLT x196 = x185 + (x180 * x195);
	const GEN_FLT x197 = (x180 * x196) + x192;
	const GEN_FLT x198 = sin(x177);
	const GEN_FLT x199 = x191 * x198;
	const GEN_FLT x200 = x178 + (x197 * x199);
	const GEN_FLT x201 = 1. / x200;
	const GEN_FLT x202 = x201 * x191;
	const GEN_FLT x203 = x202 * x186;
	const GEN_FLT x204 = x189 + (x203 * x185);
	const GEN_FLT x205 = 1. / sqrt(1 + (-1 * (x204 * x204)));
	const GEN_FLT x206 = x121 * x187;
	const GEN_FLT x207 = (-1 * x107 * x188) + (x206 * x111);
	const GEN_FLT x208 = 1. / sqrt(1 + (-1 * x124 * (x187 * x187)));
	const GEN_FLT x209 = cos(x190) * ogeePhase_1;
	const GEN_FLT x210 = x209 * ((-1 * x208 * x207) + x62);
	const GEN_FLT x211 = x186 * x185;
	const GEN_FLT x212 = x211 * x201;
	const GEN_FLT x213 = 1. / sqrt(1 + (-1 * x103 * (1. / (x178 * x178))));
	const GEN_FLT x214 = (x115 * x179) + (-1 * x114 * x179);
	const GEN_FLT x215 = x213 * x214;
	const GEN_FLT x216 = 2 * x202 * x192;
	const GEN_FLT x217 = x197 * x198;
	const GEN_FLT x218 = x215 * x182;
	const GEN_FLT x219 = 2.40324066e-05 * x180;
	const GEN_FLT x220 = (x180 * (x218 + (-1 * x215 * x181))) + (x215 * x183);
	const GEN_FLT x221 = (x215 * x184) + (x220 * x180);
	const GEN_FLT x222 = x213 * x185;
	const GEN_FLT x223 = x213 * x196;
	const GEN_FLT x224 = x211 * (1. / (x200 * x200)) * x191;
	const GEN_FLT x225 =
		x205 *
		((-1 * x224 *
		  ((x199 *
			((x221 * x180) + (x214 * x223) + (x214 * x222) +
			 (x180 * ((x215 * x195) + x221 +
					  (x180 * (x220 + (x215 * x194) + (x180 * ((x215 * x193) + (-1 * x219 * x215) + x218)))))))) +
		   (x210 * x217))) +
		 (x215 * x216) + x207 + (x203 * x221) + (x210 * x212));
	const GEN_FLT x226 = cos(asin(x204) + x134 + (-1 * gibPhase_1)) * gibMag_1;
	const GEN_FLT x227 = (-1 * x187 * x151) + (x206 * x143);
	const GEN_FLT x228 = (-1 * x208 * x227) + x141;
	const GEN_FLT x229 = x212 * x209;
	const GEN_FLT x230 = x113 * x179;
	const GEN_FLT x231 = (x179 * x146) + (-1 * x230 * x144);
	const GEN_FLT x232 = x213 * x231;
	const GEN_FLT x233 = x217 * x209;
	const GEN_FLT x234 = x232 * x182;
	const GEN_FLT x235 = (x180 * (x234 + (-1 * x232 * x181))) + (x232 * x183);
	const GEN_FLT x236 = (x232 * x184) + (x235 * x180);
	const GEN_FLT x237 =
		x205 *
		(x227 + (x236 * x203) +
		 (-1 * x224 *
		  ((x199 * ((x236 * x180) +
					(x180 * (x236 + (x232 * x195) +
							 (x180 * (x235 + (x232 * x194) + (x180 * ((x232 * x193) + (-1 * x219 * x232) + x234)))))) +
					(x231 * x222) + (x231 * x223))) +
		   (x233 * x228))) +
		 (x216 * x232) + (x228 * x229));
	const GEN_FLT x238 = (-1 * x172 * x187) + (x206 * x165);
	const GEN_FLT x239 = (-1 * x238 * x208) + x163;
	const GEN_FLT x240 = (x167 * x179) + (-1 * x230 * x166);
	const GEN_FLT x241 = x213 * x240;
	const GEN_FLT x242 = x241 * x182;
	const GEN_FLT x243 = (x180 * (x242 + (-1 * x241 * x181))) + (x241 * x183);
	const GEN_FLT x244 = (x241 * x184) + (x243 * x180);
	const GEN_FLT x245 =
		x205 *
		((-1 * x224 *
		  ((x199 *
			((x244 * x180) + (x222 * x240) + (x223 * x240) +
			 (x180 * (x244 + (x241 * x195) +
					  (x180 * (x243 + (x241 * x194) + (x180 * ((x241 * x193) + (-1 * x219 * x241) + x242)))))))) +
		   (x233 * x239))) +
		 x238 + (x203 * x244) + (x216 * x241) + (x239 * x229));
	out[0] = (-1 * (x132 + x133) * x135) + (-1 * x132) + x62;
	out[1] = (-1 * (x156 + x157) * x135) + (-1 * x156) + x141;
	out[2] = (-1 * (x175 + x176) * x135) + (-1 * x175) + x163;
	out[3] = (-1 * (x225 + x133) * x226) + (-1 * x225) + x62;
	out[4] = (-1 * (x237 + x157) * x226) + (-1 * x237) + x141;
	out[5] = (-1 * (x245 + x176) * x226) + (-1 * x245) + x163;
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
	const GEN_FLT x2 = x1 + x0;
	const GEN_FLT x3 = lh_qk * lh_qk;
	const GEN_FLT x4 = x0 + x3;
	const GEN_FLT x5 = sqrt(x4 + (lh_qw * lh_qw) + x1);
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = obj_qk * obj_qk;
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = 2 * sqrt(x10 + (obj_qw * obj_qw) + x7);
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 =
		((x16 + (-1 * x15)) * x17) + ((x13 + x12) * x14) + ((1 + (-1 * (x8 + x7) * x11)) * sensor_z) + obj_pz;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x20 + x19;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = ((x24 + x23) * x17) + ((1 + (-1 * x11 * x10)) * sensor_y) + obj_py + ((x13 + (-1 * x12)) * x22);
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = x28 + (-1 * x27);
	const GEN_FLT x30 =
		((1 + (-1 * (x7 + x9) * x11)) * sensor_x) + ((x24 + (-1 * x23)) * x14) + ((x16 + x15) * x22) + obj_px;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = (x31 * x29) + (x21 * x26) + (x18 * (1 + (-1 * x2 * x6))) + lh_pz;
	const GEN_FLT x33 = x28 + x27;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = x36 + (-1 * x35);
	const GEN_FLT x38 = (x30 * (1 + (-1 * x4 * x6))) + (x37 * x26) + (x34 * x33) + lh_px;
	const GEN_FLT x39 = x38 * x38;
	const GEN_FLT x40 = x39 + (x32 * x32);
	const GEN_FLT x41 = 1. / x40;
	const GEN_FLT x42 = x41 * x32;
	const GEN_FLT x43 = x20 + (-1 * x19);
	const GEN_FLT x44 = x1 + x3;
	const GEN_FLT x45 = x36 + x35;
	const GEN_FLT x46 = (x45 * x31) + (x25 * (1 + (-1 * x6 * x44))) + (x43 * x34) + lh_py;
	const GEN_FLT x47 = 0.523598775598299 + tilt_0;
	const GEN_FLT x48 = cos(x47);
	const GEN_FLT x49 = 1. / x48;
	const GEN_FLT x50 = x46 * x46;
	const GEN_FLT x51 = x40 + x50;
	const GEN_FLT x52 = 1. / sqrt(x51);
	const GEN_FLT x53 = x52 * x49;
	const GEN_FLT x54 = asin(x53 * x46);
	const GEN_FLT x55 = 8.0108022e-06 * x54;
	const GEN_FLT x56 = -8.0108022e-06 + (-1 * x55);
	const GEN_FLT x57 = 0.0028679863 + (x54 * x56);
	const GEN_FLT x58 = 5.3685255e-06 + (x54 * x57);
	const GEN_FLT x59 = 0.0076069798 + (x54 * x58);
	const GEN_FLT x60 = x54 * x59;
	const GEN_FLT x61 = -8.0108022e-06 + (-1.60216044e-05 * x54);
	const GEN_FLT x62 = x57 + (x61 * x54);
	const GEN_FLT x63 = x58 + (x62 * x54);
	const GEN_FLT x64 = x59 + (x63 * x54);
	const GEN_FLT x65 = (x64 * x54) + x60;
	const GEN_FLT x66 = sin(x47);
	const GEN_FLT x67 = tan(x47);
	const GEN_FLT x68 = 1. / sqrt(x40);
	const GEN_FLT x69 = x67 * x68;
	const GEN_FLT x70 = x69 * x46;
	const GEN_FLT x71 = atan2(-1 * x32, x38);
	const GEN_FLT x72 = x71 + (-1 * asin(x70)) + ogeeMag_0;
	const GEN_FLT x73 = (sin(x72) * ogeePhase_0) + curve_0;
	const GEN_FLT x74 = x73 * x66;
	const GEN_FLT x75 = x48 + (-1 * x74 * x65);
	const GEN_FLT x76 = 1. / x75;
	const GEN_FLT x77 = x54 * x54;
	const GEN_FLT x78 = x73 * x77;
	const GEN_FLT x79 = x78 * x76;
	const GEN_FLT x80 = x70 + (x79 * x59);
	const GEN_FLT x81 = 1. / sqrt(1 + (-1 * (x80 * x80)));
	const GEN_FLT x82 = (1. / (x40 * sqrt(x40))) * x46;
	const GEN_FLT x83 = x82 * x38;
	const GEN_FLT x84 = x83 * x67;
	const GEN_FLT x85 = x50 * (1. / x51);
	const GEN_FLT x86 = 1. / sqrt(1 + (-1 * x85 * (1. / (x48 * x48))));
	const GEN_FLT x87 = 1. / (x51 * sqrt(x51));
	const GEN_FLT x88 = x87 * x46;
	const GEN_FLT x89 = x88 * x38;
	const GEN_FLT x90 = x89 * x49;
	const GEN_FLT x91 = x86 * x90;
	const GEN_FLT x92 = -1 * x56 * x91;
	const GEN_FLT x93 = x86 * x57;
	const GEN_FLT x94 = (-1 * x93 * x90) + (x54 * (x92 + (x55 * x91)));
	const GEN_FLT x95 = (x54 * x94) + (-1 * x58 * x91);
	const GEN_FLT x96 = x50 * x41;
	const GEN_FLT x97 = 1. / sqrt(1 + (-1 * (x67 * x67) * x96));
	const GEN_FLT x98 = (x84 * x97) + x42;
	const GEN_FLT x99 = cos(x72) * ogeePhase_0;
	const GEN_FLT x100 = x65 * x66;
	const GEN_FLT x101 = x99 * x100;
	const GEN_FLT x102 = 2.40324066e-05 * x54;
	const GEN_FLT x103 = x86 * x63;
	const GEN_FLT x104 = x86 * x64;
	const GEN_FLT x105 = x86 * x59;
	const GEN_FLT x106 = x78 * (1. / (x75 * x75)) * x59;
	const GEN_FLT x107 = x77 * x76 * x59;
	const GEN_FLT x108 = x99 * x107;
	const GEN_FLT x109 = 2 * x38;
	const GEN_FLT x110 = x88 * x109;
	const GEN_FLT x111 = x73 * x76 * x60;
	const GEN_FLT x112 = x86 * x49 * x111;
	const GEN_FLT x113 =
		x81 * ((-1 * x110 * x112) + (x98 * x108) +
			   (-1 * x106 *
				((-1 * x74 *
				  ((-1 * x90 * x105) + (x54 * x95) + (-1 * x90 * x104) +
				   (x54 * (x95 + (-1 * x90 * x103) +
						   (x54 * (x94 + (-1 * x62 * x91) + (x54 * ((-1 * x61 * x91) + (x91 * x102) + x92)))))))) +
				 (-1 * x98 * x101))) +
			   (x79 * x95) + (-1 * x84));
	const GEN_FLT x114 = -1 * x42;
	const GEN_FLT x115 = -1 * x71;
	const GEN_FLT x116 = cos(x115 + asin(x80) + (-1 * gibPhase_0)) * gibMag_0;
	const GEN_FLT x117 = x87 * x50;
	const GEN_FLT x118 = x53 + (-1 * x49 * x117);
	const GEN_FLT x119 = x86 * x118;
	const GEN_FLT x120 = x56 * x119;
	const GEN_FLT x121 = (x93 * x118) + (x54 * (x120 + (-1 * x55 * x119)));
	const GEN_FLT x122 = (x54 * x121) + (x58 * x119);
	const GEN_FLT x123 = x69 * x97;
	const GEN_FLT x124 = 2 * x111;
	const GEN_FLT x125 =
		x81 * ((x119 * x124) + (-1 * x108 * x123) + (x79 * x122) +
			   (-1 * x106 *
				((-1 * x74 *
				  ((x105 * x118) + (x54 * x122) + (x104 * x118) +
				   (x54 * (x122 + (x103 * x118) +
						   (x54 * (x121 + (x62 * x119) + (x54 * ((x61 * x119) + (-1 * x102 * x119) + x120)))))))) +
				 (x101 * x123))) +
			   x69);
	const GEN_FLT x126 = x41 * x38;
	const GEN_FLT x127 = -1 * x126;
	const GEN_FLT x128 = x88 * x32;
	const GEN_FLT x129 = x49 * x128;
	const GEN_FLT x130 = x86 * x129;
	const GEN_FLT x131 = -1 * x56 * x130;
	const GEN_FLT x132 = (-1 * x93 * x129) + (x54 * (x131 + (x55 * x130)));
	const GEN_FLT x133 = (x54 * x132) + (-1 * x58 * x130);
	const GEN_FLT x134 = x82 * x32;
	const GEN_FLT x135 = x67 * x134;
	const GEN_FLT x136 = (x97 * x135) + x127;
	const GEN_FLT x137 = 2 * x32;
	const GEN_FLT x138 = x88 * x137;
	const GEN_FLT x139 =
		x81 * ((-1 * x112 * x138) + (-1 * x135) + (x108 * x136) +
			   (-1 * x106 *
				((-1 * x74 *
				  ((-1 * x105 * x129) + (x54 * x133) +
				   (x54 * (x133 + (-1 * x103 * x129) +
						   (x54 * (x132 + (-1 * x62 * x130) + (x54 * ((-1 * x61 * x130) + (x102 * x130) + x131)))))) +
				   (-1 * x104 * x129))) +
				 (-1 * x101 * x136))) +
			   (x79 * x133));
	const GEN_FLT x140 = 2 * (1. / x5);
	const GEN_FLT x141 = x4 * x140;
	const GEN_FLT x142 = x30 * x141;
	const GEN_FLT x143 = x140 * lh_qw;
	const GEN_FLT x144 = x25 * x143;
	const GEN_FLT x145 = x26 * lh_qk;
	const GEN_FLT x146 = x33 * x18;
	const GEN_FLT x147 = x34 * lh_qj;
	const GEN_FLT x148 = x147 + (x143 * x146) + (-1 * x145) + (x37 * x144) + (-1 * x142 * lh_qw);
	const GEN_FLT x149 = x32 * (1. / x39);
	const GEN_FLT x150 = 1. / x38;
	const GEN_FLT x151 = x30 * x29;
	const GEN_FLT x152 = x26 * lh_qi;
	const GEN_FLT x153 = x31 * lh_qj;
	const GEN_FLT x154 = x2 * x140;
	const GEN_FLT x155 = (-1 * x18 * x154 * lh_qw) + (-1 * x153) + x152 + (x21 * x144) + (x143 * x151);
	const GEN_FLT x156 = x41 * x39;
	const GEN_FLT x157 = ((-1 * x150 * x155) + (x148 * x149)) * x156;
	const GEN_FLT x158 = x45 * x30;
	const GEN_FLT x159 = x31 * lh_qk;
	const GEN_FLT x160 = x44 * x140;
	const GEN_FLT x161 = x25 * x160;
	const GEN_FLT x162 = x43 * x18;
	const GEN_FLT x163 = x34 * lh_qi;
	const GEN_FLT x164 = (-1 * x163) + (x162 * x143) + (-1 * x161 * lh_qw) + x159 + (x143 * x158);
	const GEN_FLT x165 = 2 * x46;
	const GEN_FLT x166 = (x137 * x155) + (x109 * x148);
	const GEN_FLT x167 = 1.0 / 2.0 * x88;
	const GEN_FLT x168 = x167 * (x166 + (x165 * x164));
	const GEN_FLT x169 = (x53 * x164) + (-1 * x49 * x168);
	const GEN_FLT x170 = x86 * x169;
	const GEN_FLT x171 = x56 * x170;
	const GEN_FLT x172 = (x93 * x169) + (x54 * (x171 + (-1 * x55 * x170)));
	const GEN_FLT x173 = (x54 * x172) + (x58 * x170);
	const GEN_FLT x174 = 1.0 / 2.0 * x82;
	const GEN_FLT x175 = x166 * x174;
	const GEN_FLT x176 = (x69 * x164) + (-1 * x67 * x175);
	const GEN_FLT x177 = (-1 * x97 * x176) + x157;
	const GEN_FLT x178 =
		x81 * (x176 +
			   (-1 * x106 *
				((-1 * x74 *
				  ((x105 * x169) + (x104 * x169) + (x54 * x173) +
				   (x54 * (x173 + (x103 * x169) +
						   (x54 * ((x62 * x170) + x172 + (x54 * ((x61 * x170) + (-1 * x102 * x170) + x171)))))))) +
				 (-1 * x101 * x177))) +
			   (x124 * x170) + (x108 * x177) + (x79 * x173));
	const GEN_FLT x179 = -1 * x157;
	const GEN_FLT x180 = x140 * lh_qi;
	const GEN_FLT x181 = x37 * x25;
	const GEN_FLT x182 = x26 * lh_qj;
	const GEN_FLT x183 = x34 * lh_qk;
	const GEN_FLT x184 = x183 + (x180 * x146) + x182 + (x181 * x180) + (-1 * x142 * lh_qi);
	const GEN_FLT x185 = x26 * lh_qw;
	const GEN_FLT x186 = x25 * x21;
	const GEN_FLT x187 = 4 * x5;
	const GEN_FLT x188 = -1 * x187 * lh_qi;
	const GEN_FLT x189 = (x180 * x186) + (x18 * (x188 + (-1 * x154 * lh_qi))) + x185 + x159 + (x180 * x151);
	const GEN_FLT x190 = ((-1 * x189 * x150) + (x184 * x149)) * x156;
	const GEN_FLT x191 = x34 * lh_qw;
	const GEN_FLT x192 = (-1 * x191) + (x162 * x180) + (x25 * (x188 + (-1 * x160 * lh_qi))) + x153 + (x180 * x158);
	const GEN_FLT x193 = (x189 * x137) + (x109 * x184);
	const GEN_FLT x194 = x167 * (x193 + (x165 * x192));
	const GEN_FLT x195 = ((x53 * x192) + (-1 * x49 * x194)) * x86;
	const GEN_FLT x196 = x56 * x195;
	const GEN_FLT x197 = (x57 * x195) + (x54 * (x196 + (-1 * x55 * x195)));
	const GEN_FLT x198 = (x54 * x197) + (x58 * x195);
	const GEN_FLT x199 = x174 * x193;
	const GEN_FLT x200 = (x69 * x192) + (-1 * x67 * x199);
	const GEN_FLT x201 = x99 * ((-1 * x97 * x200) + x190);
	const GEN_FLT x202 =
		x81 * (x200 + (x201 * x107) +
			   (-1 * x106 *
				((-1 * x74 *
				  ((x59 * x195) + (x54 * x198) + (x64 * x195) +
				   (x54 * (x198 + (x63 * x195) +
						   (x54 * (x197 + (x62 * x195) + (x54 * ((x61 * x195) + (-1 * x102 * x195) + x196)))))))) +
				 (-1 * x201 * x100))) +
			   (x124 * x195) + (x79 * x198));
	const GEN_FLT x203 = -1 * x190;
	const GEN_FLT x204 = -1 * x187 * lh_qj;
	const GEN_FLT x205 = x140 * lh_qj;
	const GEN_FLT x206 = x191 + (x205 * x146) + x152 + (x205 * x181) + (x30 * (x204 + (-1 * x141 * lh_qj)));
	const GEN_FLT x207 = x140 * x151;
	const GEN_FLT x208 = x31 * lh_qw;
	const GEN_FLT x209 = x145 + (x18 * (x204 + (-1 * x154 * lh_qj))) + (x205 * x186) + (-1 * x208) + (x207 * lh_qj);
	const GEN_FLT x210 = ((-1 * x209 * x150) + (x206 * x149)) * x156;
	const GEN_FLT x211 = x31 * lh_qi;
	const GEN_FLT x212 = x162 * x140;
	const GEN_FLT x213 = (x212 * lh_qj) + x183 + (-1 * x161 * lh_qj) + x211 + (x205 * x158);
	const GEN_FLT x214 = (x209 * x137) + (x206 * x109);
	const GEN_FLT x215 = x167 * (x214 + (x213 * x165));
	const GEN_FLT x216 = (x53 * x213) + (-1 * x49 * x215);
	const GEN_FLT x217 = x86 * x216;
	const GEN_FLT x218 = x56 * x217;
	const GEN_FLT x219 = (x93 * x216) + (x54 * (x218 + (-1 * x55 * x217)));
	const GEN_FLT x220 = (x54 * x219) + (x58 * x217);
	const GEN_FLT x221 = x214 * x174;
	const GEN_FLT x222 = (x69 * x213) + (-1 * x67 * x221);
	const GEN_FLT x223 = (-1 * x97 * x222) + x210;
	const GEN_FLT x224 =
		x81 * ((x223 * x108) + (x217 * x124) +
			   (-1 * x106 *
				((-1 * x74 *
				  ((x59 * x217) + (x54 * x220) + (x216 * x104) +
				   (x54 * (x220 + (x216 * x103) +
						   (x54 * (x219 + (x62 * x217) + (x54 * ((-1 * x217 * x102) + (x61 * x217) + x218)))))))) +
				 (-1 * x223 * x101))) +
			   x222 + (x79 * x220));
	const GEN_FLT x225 = -1 * x210;
	const GEN_FLT x226 = -1 * x187 * lh_qk;
	const GEN_FLT x227 = x140 * lh_qk;
	const GEN_FLT x228 = x18 * lh_qk;
	const GEN_FLT x229 =
		x163 + (x33 * x228 * x140) + (x227 * x181) + (-1 * x185) + (x30 * (x226 + (-1 * x141 * lh_qk)));
	const GEN_FLT x230 = x182 + (-1 * x228 * x154) + x211 + (x227 * x186) + (x207 * lh_qk);
	const GEN_FLT x231 = ((-1 * x230 * x150) + (x229 * x149)) * x156;
	const GEN_FLT x232 = (x212 * lh_qk) + x147 + (x227 * x158) + (x25 * (x226 + (-1 * x160 * lh_qk))) + x208;
	const GEN_FLT x233 = (x230 * x137) + (x229 * x109);
	const GEN_FLT x234 = x167 * (x233 + (x232 * x165));
	const GEN_FLT x235 = (x53 * x232) + (-1 * x49 * x234);
	const GEN_FLT x236 = x86 * x235;
	const GEN_FLT x237 = x56 * x236;
	const GEN_FLT x238 = (x93 * x235) + (x54 * (x237 + (-1 * x55 * x236)));
	const GEN_FLT x239 = (x54 * x238) + (x58 * x236);
	const GEN_FLT x240 = x233 * x174;
	const GEN_FLT x241 = (x69 * x232) + (-1 * x67 * x240);
	const GEN_FLT x242 = (-1 * x97 * x241) + x231;
	const GEN_FLT x243 =
		x81 * (x241 + (x236 * x124) +
			   (-1 * x106 *
				((-1 * x74 *
				  ((x59 * x236) + (x54 * x239) + (x64 * x236) +
				   (x54 * (x239 + (x235 * x103) +
						   (x54 * (x238 + (x62 * x236) + (x54 * ((x61 * x236) + (-1 * x236 * x102) + x237)))))))) +
				 (-1 * x242 * x101))) +
			   (x242 * x108) + (x79 * x239));
	const GEN_FLT x244 = -1 * x231;
	const GEN_FLT x245 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x246 = cos(x245);
	const GEN_FLT x247 = 1. / x246;
	const GEN_FLT x248 = x52 * x247;
	const GEN_FLT x249 = asin(x46 * x248);
	const GEN_FLT x250 = 8.0108022e-06 * x249;
	const GEN_FLT x251 = -8.0108022e-06 + (-1 * x250);
	const GEN_FLT x252 = 0.0028679863 + (x251 * x249);
	const GEN_FLT x253 = 5.3685255e-06 + (x252 * x249);
	const GEN_FLT x254 = 0.0076069798 + (x253 * x249);
	const GEN_FLT x255 = x254 * x249;
	const GEN_FLT x256 = -8.0108022e-06 + (-1.60216044e-05 * x249);
	const GEN_FLT x257 = x252 + (x256 * x249);
	const GEN_FLT x258 = x253 + (x257 * x249);
	const GEN_FLT x259 = x254 + (x258 * x249);
	const GEN_FLT x260 = (x259 * x249) + x255;
	const GEN_FLT x261 = tan(x245);
	const GEN_FLT x262 = x68 * x261;
	const GEN_FLT x263 = -1 * x46 * x262;
	const GEN_FLT x264 = x71 + (-1 * asin(x263)) + ogeeMag_1;
	const GEN_FLT x265 = (sin(x264) * ogeePhase_1) + curve_1;
	const GEN_FLT x266 = sin(x245);
	const GEN_FLT x267 = x266 * x265;
	const GEN_FLT x268 = x246 + (x267 * x260);
	const GEN_FLT x269 = 1. / x268;
	const GEN_FLT x270 = x249 * x249;
	const GEN_FLT x271 = x270 * x265;
	const GEN_FLT x272 = x271 * x269;
	const GEN_FLT x273 = x263 + (x272 * x254);
	const GEN_FLT x274 = 1. / sqrt(1 + (-1 * (x273 * x273)));
	const GEN_FLT x275 = x83 * x261;
	const GEN_FLT x276 = 1. / sqrt(1 + (-1 * x96 * (x261 * x261)));
	const GEN_FLT x277 = cos(x264) * ogeePhase_1;
	const GEN_FLT x278 = x277 * ((-1 * x276 * x275) + x42);
	const GEN_FLT x279 = x270 * x269 * x254;
	const GEN_FLT x280 = 1. / sqrt(1 + (-1 * x85 * (1. / (x246 * x246))));
	const GEN_FLT x281 = x269 * x265 * x255;
	const GEN_FLT x282 = x280 * x281 * x247;
	const GEN_FLT x283 = x266 * x260;
	const GEN_FLT x284 = x89 * x247;
	const GEN_FLT x285 = x284 * x280;
	const GEN_FLT x286 = -1 * x285 * x251;
	const GEN_FLT x287 = 2.40324066e-05 * x249;
	const GEN_FLT x288 = x280 * x252;
	const GEN_FLT x289 = (x249 * (x286 + (x285 * x250))) + (-1 * x288 * x284);
	const GEN_FLT x290 = (-1 * x285 * x253) + (x289 * x249);
	const GEN_FLT x291 = x271 * (1. / (x268 * x268)) * x254;
	const GEN_FLT x292 =
		x274 *
		((x272 * x290) +
		 (-1 * x291 *
		  ((x267 *
			((x290 * x249) + (-1 * x285 * x254) + (-1 * x285 * x259) +
			 (x249 * (x290 + (-1 * x285 * x258) +
					  (x249 * (x289 + (-1 * x285 * x257) + (x249 * ((-1 * x285 * x256) + (x287 * x285) + x286)))))))) +
		   (x278 * x283))) +
		 (-1 * x282 * x110) + (x279 * x278) + x275);
	const GEN_FLT x293 = cos(x115 + asin(x273) + (-1 * gibPhase_1)) * gibMag_1;
	const GEN_FLT x294 = x279 * x277;
	const GEN_FLT x295 = x276 * x262;
	const GEN_FLT x296 = x248 + (-1 * x247 * x117);
	const GEN_FLT x297 = x296 * x280;
	const GEN_FLT x298 = 2 * x281;
	const GEN_FLT x299 = x277 * x283;
	const GEN_FLT x300 = x297 * x251;
	const GEN_FLT x301 = (x249 * (x300 + (-1 * x297 * x250))) + (x296 * x288);
	const GEN_FLT x302 = (x297 * x253) + (x249 * x301);
	const GEN_FLT x303 =
		x274 *
		((x272 * x302) +
		 (-1 * x291 *
		  ((x267 *
			((x249 * x302) + (x297 * x259) + (x297 * x254) +
			 (x249 * (x302 + (x297 * x258) +
					  (x249 * (x301 + (x297 * x257) + (x249 * ((x297 * x256) + (-1 * x297 * x287) + x300)))))))) +
		   (x299 * x295))) +
		 (-1 * x262) + (x297 * x298) + (x295 * x294));
	const GEN_FLT x304 = x261 * x134;
	const GEN_FLT x305 = (-1 * x276 * x304) + x127;
	const GEN_FLT x306 = x247 * x128;
	const GEN_FLT x307 = x280 * x306;
	const GEN_FLT x308 = -1 * x251 * x307;
	const GEN_FLT x309 = (x249 * (x308 + (x250 * x307))) + (-1 * x288 * x306);
	const GEN_FLT x310 = (-1 * x253 * x307) + (x249 * x309);
	const GEN_FLT x311 =
		x274 *
		((x272 * x310) +
		 (-1 * x291 *
		  ((x267 *
			((x249 * x310) + (-1 * x254 * x307) + (-1 * x259 * x307) +
			 (x249 * (x310 + (-1 * x258 * x307) +
					  (x249 * (x309 + (-1 * x257 * x307) + (x249 * ((-1 * x256 * x307) + (x287 * x307) + x308)))))))) +
		   (x299 * x305))) +
		 (x294 * x305) + (-1 * x282 * x138) + x304);
	const GEN_FLT x312 = (-1 * x262 * x164) + (x261 * x175);
	const GEN_FLT x313 = (-1 * x276 * x312) + x157;
	const GEN_FLT x314 = (x248 * x164) + (-1 * x247 * x168);
	const GEN_FLT x315 = x280 * x314;
	const GEN_FLT x316 = x251 * x315;
	const GEN_FLT x317 = (x249 * (x316 + (-1 * x250 * x315))) + (x288 * x314);
	const GEN_FLT x318 = (x253 * x315) + (x249 * x317);
	const GEN_FLT x319 =
		x274 *
		(x312 + (x272 * x318) +
		 (-1 * x291 *
		  ((x267 *
			((x254 * x315) + (x259 * x315) + (x249 * x318) +
			 (x249 * (x318 + (x258 * x315) +
					  (x249 * ((x257 * x315) + x317 + (x249 * ((x256 * x315) + (-1 * x287 * x315) + x316)))))))) +
		   (x299 * x313))) +
		 (x298 * x315) + (x294 * x313));
	const GEN_FLT x320 = (-1 * x262 * x192) + (x261 * x199);
	const GEN_FLT x321 = (-1 * x276 * x320) + x190;
	const GEN_FLT x322 = (x248 * x192) + (-1 * x247 * x194);
	const GEN_FLT x323 = x280 * x322;
	const GEN_FLT x324 = x251 * x323;
	const GEN_FLT x325 = (x249 * (x324 + (-1 * x250 * x323))) + (x288 * x322);
	const GEN_FLT x326 = (x253 * x323) + (x249 * x325);
	const GEN_FLT x327 =
		x274 *
		(x320 + (x272 * x326) +
		 (-1 * x291 *
		  ((x267 *
			((x249 * x326) + (x254 * x323) + (x259 * x323) +
			 (x249 * (x326 + (x258 * x323) +
					  (x249 * (x325 + (x257 * x323) + (x249 * ((x256 * x323) + (-1 * x287 * x323) + x324)))))))) +
		   (x299 * x321))) +
		 (x298 * x323) + (x294 * x321));
	const GEN_FLT x328 = (-1 * x213 * x262) + (x261 * x221);
	const GEN_FLT x329 = (-1 * x276 * x328) + x210;
	const GEN_FLT x330 = ((x213 * x248) + (-1 * x215 * x247)) * x280;
	const GEN_FLT x331 = x251 * x330;
	const GEN_FLT x332 = (x249 * (x331 + (-1 * x250 * x330))) + (x252 * x330);
	const GEN_FLT x333 = (x253 * x330) + (x249 * x332);
	const GEN_FLT x334 =
		x274 *
		((x272 * x333) +
		 (-1 * x291 *
		  ((x267 *
			((x249 * x333) + (x254 * x330) + (x259 * x330) +
			 (x249 * (x333 + (x258 * x330) +
					  (x249 * (x332 + (x257 * x330) + (x249 * ((x256 * x330) + (-1 * x287 * x330) + x331)))))))) +
		   (x299 * x329))) +
		 x328 + (x298 * x330) + (x294 * x329));
	const GEN_FLT x335 = (-1 * x232 * x262) + (x261 * x240);
	const GEN_FLT x336 = (-1 * x276 * x335) + x231;
	const GEN_FLT x337 = ((x232 * x248) + (-1 * x234 * x247)) * x280;
	const GEN_FLT x338 = x251 * x337;
	const GEN_FLT x339 = (x249 * (x338 + (-1 * x250 * x337))) + (x252 * x337);
	const GEN_FLT x340 = (x253 * x337) + (x249 * x339);
	const GEN_FLT x341 =
		x274 *
		(x335 +
		 (-1 * x291 *
		  ((x267 *
			((x249 * x340) + (x254 * x337) + (x259 * x337) +
			 (x249 * (x340 + (x258 * x337) +
					  (x249 * (x339 + (x257 * x337) + (x249 * ((x256 * x337) + (-1 * x287 * x337) + x338)))))))) +
		   (x299 * x336))) +
		 (x272 * x340) + (x298 * x337) + (x294 * x336));
	out[0] = (-1 * (x113 + x114) * x116) + (-1 * x113) + x42;
	out[1] = (-1 * x116 * x125) + (-1 * x125);
	out[2] = (-1 * (x139 + x126) * x116) + (-1 * x139) + x127;
	out[3] = (-1 * (x178 + x179) * x116) + (-1 * x178) + x157;
	out[4] = (-1 * (x202 + x203) * x116) + (-1 * x202) + x190;
	out[5] = (-1 * x224) + (-1 * (x224 + x225) * x116) + x210;
	out[6] = (-1 * x243) + (-1 * (x243 + x244) * x116) + x231;
	out[7] = (-1 * (x292 + x114) * x293) + (-1 * x292) + x42;
	out[8] = (-1 * x293 * x303) + (-1 * x303);
	out[9] = (-1 * (x311 + x126) * x293) + (-1 * x311) + x127;
	out[10] = (-1 * (x319 + x179) * x293) + (-1 * x319) + x157;
	out[11] = (-1 * (x327 + x203) * x293) + x190 + (-1 * x327);
	out[12] = (-1 * (x334 + x225) * x293) + (-1 * x334) + x210;
	out[13] = (-1 * (x341 + x244) * x293) + (-1 * x341) + x231;
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
	const GEN_FLT x6 = x5 + x4;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = 2 * sqrt(x6 + (obj_qw * obj_qw) + x7);
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = ((x13 + (-1 * x12)) * x14) + (x11 * (x10 + x9)) + ((1 + (-1 * x6 * x8)) * sensor_z) + obj_pz;
	const GEN_FLT x16 = lh_qi * lh_qi;
	const GEN_FLT x17 = lh_qk * lh_qk;
	const GEN_FLT x18 = lh_qj * lh_qj;
	const GEN_FLT x19 = x18 + x17;
	const GEN_FLT x20 = 2 * sqrt(x19 + (lh_qw * lh_qw) + x16);
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 =
		((1 + (-1 * (x5 + x7) * x8)) * sensor_y) + obj_py + ((x24 + x23) * x14) + (x22 * (x10 + (-1 * x9)));
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 =
		((1 + (-1 * (x4 + x7) * x8)) * sensor_x) + ((x24 + (-1 * x23)) * x11) + ((x13 + x12) * x22) + obj_px;
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = (x25 * (1 + (-1 * (x16 + x17) * x20))) + ((x27 + x26) * x29) + ((x3 + (-1 * x2)) * x21) + lh_py;
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = ((x33 + (-1 * x32)) * x29) + ((x3 + x2) * x31) + (x15 * (1 + (-1 * (x16 + x18) * x20))) + lh_pz;
	const GEN_FLT x35 = (x28 * (1 + (-1 * x20 * x19))) + ((x27 + (-1 * x26)) * x31) + ((x33 + x32) * x21) + lh_px;
	const GEN_FLT x36 = (x35 * x35) + (x34 * x34);
	const GEN_FLT x37 = x30 * (1. / sqrt(x36));
	const GEN_FLT x38 = x1 * x37;
	const GEN_FLT x39 = atan2(-1 * x34, x35);
	const GEN_FLT x40 = x39 + (-1 * asin(x38)) + ogeeMag_0;
	const GEN_FLT x41 = sin(x40);
	const GEN_FLT x42 = (x41 * ogeePhase_0) + curve_0;
	const GEN_FLT x43 = cos(x0);
	const GEN_FLT x44 = x30 * x30;
	const GEN_FLT x45 = x36 + x44;
	const GEN_FLT x46 = (1. / sqrt(x45)) * x30;
	const GEN_FLT x47 = asin((1. / x43) * x46);
	const GEN_FLT x48 = 8.0108022e-06 * x47;
	const GEN_FLT x49 = -8.0108022e-06 + (-1 * x48);
	const GEN_FLT x50 = 0.0028679863 + (x47 * x49);
	const GEN_FLT x51 = 5.3685255e-06 + (x50 * x47);
	const GEN_FLT x52 = 0.0076069798 + (x51 * x47);
	const GEN_FLT x53 = sin(x0);
	const GEN_FLT x54 = x52 * x47;
	const GEN_FLT x55 = -8.0108022e-06 + (-1.60216044e-05 * x47);
	const GEN_FLT x56 = x50 + (x55 * x47);
	const GEN_FLT x57 = x51 + (x56 * x47);
	const GEN_FLT x58 = x52 + (x57 * x47);
	const GEN_FLT x59 = (x58 * x47) + x54;
	const GEN_FLT x60 = x59 * x42;
	const GEN_FLT x61 = x60 * x53;
	const GEN_FLT x62 = x43 + (-1 * x61);
	const GEN_FLT x63 = 1. / x62;
	const GEN_FLT x64 = x47 * x47;
	const GEN_FLT x65 = x63 * x64;
	const GEN_FLT x66 = x65 * x52;
	const GEN_FLT x67 = x38 + (x66 * x42);
	const GEN_FLT x68 = 1. / sqrt(1 + (-1 * (x67 * x67)));
	const GEN_FLT x69 = x1 * x1;
	const GEN_FLT x70 = x37 * (1 + x69);
	const GEN_FLT x71 = 1. / (x43 * x43);
	const GEN_FLT x72 = x44 * (1. / x45);
	const GEN_FLT x73 = x71 * x46 * (1. / sqrt(1 + (-1 * x71 * x72)));
	const GEN_FLT x74 = x73 * x53;
	const GEN_FLT x75 = x74 * x49;
	const GEN_FLT x76 = (x74 * x50) + (x47 * (x75 + (-1 * x74 * x48)));
	const GEN_FLT x77 = (x76 * x47) + (x74 * x51);
	const GEN_FLT x78 = cos(x40) * ogeePhase_0;
	const GEN_FLT x79 = x44 * (1. / x36);
	const GEN_FLT x80 = x70 * (1. / sqrt(1 + (-1 * x79 * x69)));
	const GEN_FLT x81 = x53 * x42;
	const GEN_FLT x82 = x64 * (1. / (x62 * x62)) * x52;
	const GEN_FLT x83 = x78 * x66;
	const GEN_FLT x84 =
		x68 *
		((2 * x81 * x73 * x63 * x54) + (-1 * x80 * x83) +
		 (-1 * x82 * x42 *
		  ((-1 * x81 *
			((x74 * x52) + (x77 * x47) + (x74 * x58) +
			 (x47 * (x77 + (x74 * x57) +
					 (x47 * (x76 + (x74 * x56) + (x47 * ((x74 * x55) + (-2.40324066e-05 * x74 * x47) + x75)))))))) +
		   (x80 * x78 * x53 * x59) + (-1 * x60 * x43) + (-1 * x53))) +
		 (x77 * x65 * x42) + x70);
	const GEN_FLT x85 = -1 * x39;
	const GEN_FLT x86 = x85 + asin(x67) + (-1 * gibPhase_0);
	const GEN_FLT x87 = cos(x86) * gibMag_0;
	const GEN_FLT x88 = x82 * x61;
	const GEN_FLT x89 = (x66 + x88) * x68;
	const GEN_FLT x90 = x68 * (x83 + (x88 * x78));
	const GEN_FLT x91 = ((x66 * x41) + (x88 * x41)) * x68;
	const GEN_FLT x92 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x93 = tan(x92);
	const GEN_FLT x94 = -1 * x93 * x37;
	const GEN_FLT x95 = x39 + (-1 * asin(x94)) + ogeeMag_1;
	const GEN_FLT x96 = sin(x95);
	const GEN_FLT x97 = (x96 * ogeePhase_1) + curve_1;
	const GEN_FLT x98 = cos(x92);
	const GEN_FLT x99 = asin((1. / x98) * x46);
	const GEN_FLT x100 = 8.0108022e-06 * x99;
	const GEN_FLT x101 = -8.0108022e-06 + (-1 * x100);
	const GEN_FLT x102 = 0.0028679863 + (x99 * x101);
	const GEN_FLT x103 = 5.3685255e-06 + (x99 * x102);
	const GEN_FLT x104 = 0.0076069798 + (x99 * x103);
	const GEN_FLT x105 = x99 * x99;
	const GEN_FLT x106 = x99 * x104;
	const GEN_FLT x107 = -8.0108022e-06 + (-1.60216044e-05 * x99);
	const GEN_FLT x108 = x102 + (x99 * x107);
	const GEN_FLT x109 = x103 + (x99 * x108);
	const GEN_FLT x110 = x104 + (x99 * x109);
	const GEN_FLT x111 = (x99 * x110) + x106;
	const GEN_FLT x112 = sin(x92);
	const GEN_FLT x113 = x97 * x112;
	const GEN_FLT x114 = x111 * x113;
	const GEN_FLT x115 = x98 + x114;
	const GEN_FLT x116 = 1. / x115;
	const GEN_FLT x117 = x105 * x116;
	const GEN_FLT x118 = x104 * x117;
	const GEN_FLT x119 = x94 + (x97 * x118);
	const GEN_FLT x120 = 1. / sqrt(1 + (-1 * (x119 * x119)));
	const GEN_FLT x121 = x93 * x93;
	const GEN_FLT x122 = x37 * (1 + x121);
	const GEN_FLT x123 = cos(x95) * ogeePhase_1;
	const GEN_FLT x124 = x118 * x123;
	const GEN_FLT x125 = x122 * (1. / sqrt(1 + (-1 * x79 * x121)));
	const GEN_FLT x126 = 1. / (x98 * x98);
	const GEN_FLT x127 = x46 * x126 * (1. / sqrt(1 + (-1 * x72 * x126)));
	const GEN_FLT x128 = x112 * x127;
	const GEN_FLT x129 = -1 * x101 * x128;
	const GEN_FLT x130 = (x99 * (x129 + (x100 * x128))) + (-1 * x102 * x128);
	const GEN_FLT x131 = (-1 * x103 * x128) + (x99 * x130);
	const GEN_FLT x132 = x105 * x104 * (1. / (x115 * x115));
	const GEN_FLT x133 =
		x120 * ((-2 * x106 * x113 * x116 * x127) + (x97 * x117 * x131) + (-1 * x124 * x125) +
				(-1 * x97 * x132 *
				 ((x113 * ((x99 * x131) + (-1 * x104 * x128) + (-1 * x110 * x128) +
						   (x99 * (x131 + (-1 * x109 * x128) +
								   (x99 * (x130 + (-1 * x108 * x128) +
										   (x99 * ((-1 * x107 * x128) + (2.40324066e-05 * x99 * x128) + x129)))))))) +
				  (-1 * x98 * x97 * x111) + (-1 * x111 * x112 * x123 * x125) + x112)) +
				x122);
	const GEN_FLT x134 = x85 + asin(x119) + (-1 * gibPhase_1);
	const GEN_FLT x135 = cos(x134) * gibMag_1;
	const GEN_FLT x136 = x114 * x132;
	const GEN_FLT x137 = ((-1 * x136) + x118) * x120;
	const GEN_FLT x138 = x120 * ((-1 * x123 * x136) + x124);
	const GEN_FLT x139 = ((-1 * x96 * x136) + (x96 * x118)) * x120;
	out[0] = -1;
	out[1] = (-1 * x84 * x87) + (-1 * x84);
	out[2] = (-1 * x89 * x87) + (-1 * x89);
	out[3] = x87;
	out[4] = -1 * sin(x86);
	out[5] = (-1 * x87 * x90) + (-1 * x90);
	out[6] = (-1 * x87 * x91) + (-1 * x91);
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
	out[22] = (-1 * x133 * x135) + (-1 * x133);
	out[23] = (-1 * x137 * x135) + (-1 * x137);
	out[24] = x135;
	out[25] = -1 * sin(x134);
	out[26] = (-1 * x138 * x135) + (-1 * x138);
	out[27] = (-1 * x135 * x139) + (-1 * x139);
}

/** Applying function <function reproject_axis_x_gen2 at 0x7f6253d38950> */
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
	const GEN_FLT x7 = x5 + x6;
	const GEN_FLT x8 = 2 * sqrt(x7 + (obj_qw * obj_qw) + x4);
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 =
		((x13 + (-1 * x12)) * x14) + (x11 * (x10 + x9)) + ((1 + (-1 * (x5 + x4) * x8)) * sensor_z) + obj_pz;
	const GEN_FLT x16 = lh_qi * lh_qi;
	const GEN_FLT x17 = lh_qk * lh_qk;
	const GEN_FLT x18 = lh_qj * lh_qj;
	const GEN_FLT x19 = x18 + x17;
	const GEN_FLT x20 = 2 * sqrt(x19 + (lh_qw * lh_qw) + x16);
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = ((1 + (-1 * x8 * x7)) * sensor_y) + obj_py + ((x24 + x23) * x14) + (x22 * (x10 + (-1 * x9)));
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 =
		((1 + (-1 * (x4 + x6) * x8)) * sensor_x) + ((x24 + (-1 * x23)) * x11) + ((x13 + x12) * x22) + obj_px;
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = (x25 * (1 + (-1 * (x16 + x17) * x20))) + ((x27 + x26) * x29) + ((x3 + (-1 * x2)) * x21) + lh_py;
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = ((x33 + (-1 * x32)) * x29) + ((x3 + x2) * x31) + (x15 * (1 + (-1 * (x16 + x18) * x20))) + lh_pz;
	const GEN_FLT x35 = (x28 * (1 + (-1 * x20 * x19))) + ((x27 + (-1 * x26)) * x31) + ((x33 + x32) * x21) + lh_px;
	const GEN_FLT x36 = (x35 * x35) + (x34 * x34);
	const GEN_FLT x37 = asin((1. / x1) * x30 * (1. / sqrt(x36 + (x30 * x30))));
	const GEN_FLT x38 = 0.0028679863 + (x37 * (-8.0108022e-06 + (-8.0108022e-06 * x37)));
	const GEN_FLT x39 = 5.3685255e-06 + (x38 * x37);
	const GEN_FLT x40 = 0.0076069798 + (x37 * x39);
	const GEN_FLT x41 = tan(x0) * x30 * (1. / sqrt(x36));
	const GEN_FLT x42 = atan2(-1 * x34, x35);
	const GEN_FLT x43 = (sin(x42 + (-1 * asin(x41)) + ogeeMag_0) * ogeePhase_0) + curve_0;
	const GEN_FLT x44 = asin(
		x41 +
		(x40 * x43 * (x37 * x37) *
		 (1. /
		  (x1 + (-1 * sin(x0) * x43 *
				 ((x37 * (x40 + (x37 * (x39 + (x37 * (x38 + (x37 * (-8.0108022e-06 + (-1.60216044e-05 * x37))))))))) +
				  (x40 * x37)))))));
	out[0] = -1.5707963267949 + x42 + (-1 * x44) + (sin(x42 + (-1 * x44) + gibPhase_0) * gibMag_0) + (-1 * phase_0);
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
	const GEN_FLT x2 = x1 + x0;
	const GEN_FLT x3 = obj_qk * obj_qk;
	const GEN_FLT x4 = x1 + x3;
	const GEN_FLT x5 = sqrt(x4 + (obj_qw * obj_qw) + x0);
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x8 + x7;
	const GEN_FLT x10 = x6 * sensor_y;
	const GEN_FLT x11 = obj_qw * obj_qj;
	const GEN_FLT x12 = obj_qk * obj_qi;
	const GEN_FLT x13 = x12 + (-1 * x11);
	const GEN_FLT x14 = x6 * sensor_x;
	const GEN_FLT x15 = (x14 * x13) + (x9 * x10) + ((1 + (-1 * x2 * x6)) * sensor_z) + obj_pz;
	const GEN_FLT x16 = lh_qw * lh_qi;
	const GEN_FLT x17 = lh_qk * lh_qj;
	const GEN_FLT x18 = x17 + (-1 * x16);
	const GEN_FLT x19 = lh_qj * lh_qj;
	const GEN_FLT x20 = lh_qk * lh_qk;
	const GEN_FLT x21 = lh_qi * lh_qi;
	const GEN_FLT x22 = x21 + x20;
	const GEN_FLT x23 = sqrt(x22 + (lh_qw * lh_qw) + x19);
	const GEN_FLT x24 = 2 * x23;
	const GEN_FLT x25 = x24 * x18;
	const GEN_FLT x26 = 1 + (-1 * x24 * x22);
	const GEN_FLT x27 = x8 + (-1 * x7);
	const GEN_FLT x28 = x6 * sensor_z;
	const GEN_FLT x29 = obj_qw * obj_qk;
	const GEN_FLT x30 = obj_qj * obj_qi;
	const GEN_FLT x31 = x30 + x29;
	const GEN_FLT x32 = (x31 * x14) + ((1 + (-1 * x4 * x6)) * sensor_y) + obj_py + (x28 * x27);
	const GEN_FLT x33 = x12 + x11;
	const GEN_FLT x34 = x30 + (-1 * x29);
	const GEN_FLT x35 = x0 + x3;
	const GEN_FLT x36 = ((1 + (-1 * x6 * x35)) * sensor_x) + (x34 * x10) + (x33 * x28) + obj_px;
	const GEN_FLT x37 = lh_qw * lh_qk;
	const GEN_FLT x38 = lh_qj * lh_qi;
	const GEN_FLT x39 = x38 + x37;
	const GEN_FLT x40 = x39 * x24;
	const GEN_FLT x41 = (x40 * x36) + (x32 * x26) + (x25 * x15) + lh_py;
	const GEN_FLT x42 = 0.523598775598299 + tilt_0;
	const GEN_FLT x43 = cos(x42);
	const GEN_FLT x44 = 1. / x43;
	const GEN_FLT x45 = x41 * x41;
	const GEN_FLT x46 = 1 + (-1 * (x21 + x19) * x24);
	const GEN_FLT x47 = x17 + x16;
	const GEN_FLT x48 = x47 * x24;
	const GEN_FLT x49 = lh_qw * lh_qj;
	const GEN_FLT x50 = lh_qk * lh_qi;
	const GEN_FLT x51 = x50 + (-1 * x49);
	const GEN_FLT x52 = x51 * x24;
	const GEN_FLT x53 = (x52 * x36) + (x48 * x32) + (x46 * x15) + lh_pz;
	const GEN_FLT x54 = x50 + x49;
	const GEN_FLT x55 = x54 * x24;
	const GEN_FLT x56 = x38 + (-1 * x37);
	const GEN_FLT x57 = x56 * x24;
	const GEN_FLT x58 = 1 + (-1 * (x19 + x20) * x24);
	const GEN_FLT x59 = (x58 * x36) + (x57 * x32) + (x55 * x15) + lh_px;
	const GEN_FLT x60 = x59 * x59;
	const GEN_FLT x61 = x60 + (x53 * x53);
	const GEN_FLT x62 = x61 + x45;
	const GEN_FLT x63 = (1. / sqrt(x62)) * x44;
	const GEN_FLT x64 = asin(x63 * x41);
	const GEN_FLT x65 = 8.0108022e-06 * x64;
	const GEN_FLT x66 = -8.0108022e-06 + (-1 * x65);
	const GEN_FLT x67 = 0.0028679863 + (x64 * x66);
	const GEN_FLT x68 = 5.3685255e-06 + (x64 * x67);
	const GEN_FLT x69 = 0.0076069798 + (x64 * x68);
	const GEN_FLT x70 = x64 * x69;
	const GEN_FLT x71 = -8.0108022e-06 + (-1.60216044e-05 * x64);
	const GEN_FLT x72 = x67 + (x71 * x64);
	const GEN_FLT x73 = x68 + (x72 * x64);
	const GEN_FLT x74 = x69 + (x73 * x64);
	const GEN_FLT x75 = (x74 * x64) + x70;
	const GEN_FLT x76 = sin(x42);
	const GEN_FLT x77 = tan(x42);
	const GEN_FLT x78 = x77 * (1. / sqrt(x61));
	const GEN_FLT x79 = x78 * x41;
	const GEN_FLT x80 = atan2(-1 * x53, x59);
	const GEN_FLT x81 = x80 + (-1 * asin(x79)) + ogeeMag_0;
	const GEN_FLT x82 = (sin(x81) * ogeePhase_0) + curve_0;
	const GEN_FLT x83 = x82 * x76;
	const GEN_FLT x84 = x43 + (-1 * x83 * x75);
	const GEN_FLT x85 = 1. / x84;
	const GEN_FLT x86 = x64 * x64;
	const GEN_FLT x87 = x82 * x86;
	const GEN_FLT x88 = x85 * x87;
	const GEN_FLT x89 = x79 + (x88 * x69);
	const GEN_FLT x90 = 1. / sqrt(1 + (-1 * (x89 * x89)));
	const GEN_FLT x91 = 1. / sqrt(1 + (-1 * (1. / x62) * (1. / (x43 * x43)) * x45));
	const GEN_FLT x92 = 4 * x23;
	const GEN_FLT x93 = x92 * x41;
	const GEN_FLT x94 = 2 * x59;
	const GEN_FLT x95 = x53 * x92;
	const GEN_FLT x96 = (x51 * x95) + (x58 * x94);
	const GEN_FLT x97 = 1.0 / 2.0 * x41;
	const GEN_FLT x98 = (1. / (x62 * sqrt(x62))) * x97 * x44;
	const GEN_FLT x99 = (x63 * x40) + (-1 * x98 * (x96 + (x93 * x39)));
	const GEN_FLT x100 = x91 * x99;
	const GEN_FLT x101 = x66 * x100;
	const GEN_FLT x102 = x67 * x91;
	const GEN_FLT x103 = (x99 * x102) + (x64 * (x101 + (-1 * x65 * x100)));
	const GEN_FLT x104 = (x64 * x103) + (x68 * x100);
	const GEN_FLT x105 = 1. / x60;
	const GEN_FLT x106 = x53 * x105;
	const GEN_FLT x107 = 1. / x59;
	const GEN_FLT x108 = 1. / x61;
	const GEN_FLT x109 = x60 * x108;
	const GEN_FLT x110 = ((-1 * x52 * x107) + (x58 * x106)) * x109;
	const GEN_FLT x111 = 1. / sqrt(1 + (-1 * (x77 * x77) * x45 * x108));
	const GEN_FLT x112 = x77 * (1. / (x61 * sqrt(x61))) * x97;
	const GEN_FLT x113 = (x78 * x40) + (-1 * x96 * x112);
	const GEN_FLT x114 = (-1 * x111 * x113) + x110;
	const GEN_FLT x115 = cos(x81) * ogeePhase_0;
	const GEN_FLT x116 = x75 * x76;
	const GEN_FLT x117 = x115 * x116;
	const GEN_FLT x118 = 2.40324066e-05 * x64;
	const GEN_FLT x119 = (1. / (x84 * x84)) * x87 * x69;
	const GEN_FLT x120 = x85 * x86 * x69;
	const GEN_FLT x121 = x115 * x120;
	const GEN_FLT x122 = 2 * x82 * x85 * x70;
	const GEN_FLT x123 =
		x90 * (x113 + (x114 * x121) +
			   (-1 * x119 *
				((-1 * x83 *
				  ((x69 * x100) + (x64 * x104) + (x74 * x100) +
				   (x64 * (x104 + (x73 * x100) +
						   (x64 * (x103 + (x72 * x100) + (x64 * ((x71 * x100) + (-1 * x100 * x118) + x101)))))))) +
				 (-1 * x114 * x117))) +
			   (x100 * x122) + (x88 * x104));
	const GEN_FLT x124 = cos(x80 + (-1 * asin(x89)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x125 = 2 * x53;
	const GEN_FLT x126 = x23 * x105 * x125;
	const GEN_FLT x127 = ((-1 * x48 * x107) + (x56 * x126)) * x109;
	const GEN_FLT x128 = 2 * x41;
	const GEN_FLT x129 = x59 * x92;
	const GEN_FLT x130 = (x95 * x47) + (x56 * x129);
	const GEN_FLT x131 = (x63 * x26) + (-1 * x98 * (x130 + (x26 * x128)));
	const GEN_FLT x132 = x91 * x131;
	const GEN_FLT x133 = x66 * x132;
	const GEN_FLT x134 = (x102 * x131) + (x64 * (x133 + (-1 * x65 * x132)));
	const GEN_FLT x135 = (x64 * x134) + (x68 * x132);
	const GEN_FLT x136 = (x78 * x26) + (-1 * x112 * x130);
	const GEN_FLT x137 = x115 * ((-1 * x111 * x136) + x127);
	const GEN_FLT x138 =
		x90 * (x136 + (x120 * x137) + (x122 * x132) +
			   (-1 * x119 *
				((-1 * x83 *
				  ((x69 * x132) + (x64 * x135) + (x74 * x132) +
				   (x64 * (x135 + (x73 * x132) +
						   (x64 * (x134 + (x72 * x132) + (x64 * ((x71 * x132) + x133 + (-1 * x118 * x132))))))))) +
				 (-1 * x116 * x137))) +
			   (x88 * x135));
	const GEN_FLT x139 = ((-1 * x46 * x107) + (x54 * x126)) * x109;
	const GEN_FLT x140 = (x46 * x125) + (x54 * x129);
	const GEN_FLT x141 = (x63 * x25) + (-1 * x98 * (x140 + (x93 * x18)));
	const GEN_FLT x142 = x91 * x141;
	const GEN_FLT x143 = x66 * x142;
	const GEN_FLT x144 = (x102 * x141) + (x64 * (x143 + (-1 * x65 * x142)));
	const GEN_FLT x145 = (x64 * x144) + (x68 * x142);
	const GEN_FLT x146 = (x78 * x25) + (-1 * x112 * x140);
	const GEN_FLT x147 = (-1 * x111 * x146) + x139;
	const GEN_FLT x148 =
		x90 * (x146 + (x122 * x142) + (x121 * x147) +
			   (-1 * x119 *
				((-1 * x83 *
				  ((x69 * x142) + (x64 * x145) +
				   (x64 * (x145 + (x73 * x142) +
						   (x64 * (x144 + (x72 * x142) + (x64 * ((x71 * x142) + (-1 * x118 * x142) + x143)))))) +
				   (x74 * x142))) +
				 (-1 * x117 * x147))) +
			   (x88 * x145));
	const GEN_FLT x149 = 2 * (1. / x5);
	const GEN_FLT x150 = x149 * obj_qw;
	const GEN_FLT x151 = x35 * sensor_x;
	const GEN_FLT x152 = x34 * sensor_y;
	const GEN_FLT x153 = x10 * obj_qk;
	const GEN_FLT x154 = x33 * sensor_z;
	const GEN_FLT x155 = x28 * obj_qj;
	const GEN_FLT x156 = x155 + (x150 * x154) + (-1 * x153) + (x150 * x152) + (-1 * x150 * x151);
	const GEN_FLT x157 = x31 * sensor_x;
	const GEN_FLT x158 = x4 * sensor_y;
	const GEN_FLT x159 = x14 * obj_qk;
	const GEN_FLT x160 = x27 * sensor_z;
	const GEN_FLT x161 = x28 * obj_qi;
	const GEN_FLT x162 = (-1 * x161) + (x160 * x150) + x159 + (-1 * x150 * x158) + (x150 * x157);
	const GEN_FLT x163 = x14 * obj_qj;
	const GEN_FLT x164 = x13 * sensor_x;
	const GEN_FLT x165 = x9 * sensor_y;
	const GEN_FLT x166 = x10 * obj_qi;
	const GEN_FLT x167 = x2 * sensor_z;
	const GEN_FLT x168 = x166 + (-1 * x167 * x150) + (x165 * x150) + (x164 * x150) + (-1 * x163);
	const GEN_FLT x169 = (x25 * x168) + (x26 * x162) + (x40 * x156);
	const GEN_FLT x170 = (x55 * x168) + (x57 * x162) + (x58 * x156);
	const GEN_FLT x171 = (x46 * x168) + (x48 * x162) + (x52 * x156);
	const GEN_FLT x172 = (x125 * x171) + (x94 * x170);
	const GEN_FLT x173 = (x63 * x169) + (-1 * x98 * (x172 + (x128 * x169)));
	const GEN_FLT x174 = x91 * x173;
	const GEN_FLT x175 = x66 * x174;
	const GEN_FLT x176 = (x102 * x173) + (x64 * (x175 + (-1 * x65 * x174)));
	const GEN_FLT x177 = (x64 * x176) + (x68 * x174);
	const GEN_FLT x178 = ((-1 * x107 * x171) + (x106 * x170)) * x109;
	const GEN_FLT x179 = (x78 * x169) + (-1 * x112 * x172);
	const GEN_FLT x180 = (-1 * x111 * x179) + x178;
	const GEN_FLT x181 =
		x90 * (x179 + (x122 * x174) + (x121 * x180) +
			   (-1 * x119 *
				((-1 * x83 *
				  ((x69 * x174) + (x64 * x177) + (x74 * x174) +
				   (x64 * (x177 + (x73 * x174) +
						   (x64 * (x176 + (x72 * x174) + (x64 * ((x71 * x174) + (-1 * x118 * x174) + x175)))))))) +
				 (-1 * x117 * x180))) +
			   (x88 * x177));
	const GEN_FLT x182 = x149 * obj_qi;
	const GEN_FLT x183 = x10 * obj_qj;
	const GEN_FLT x184 = x28 * obj_qk;
	const GEN_FLT x185 = x184 + (x182 * x154) + x183 + (x182 * x152) + (-1 * x182 * x151);
	const GEN_FLT x186 = 4 * x5;
	const GEN_FLT x187 = -1 * x186 * obj_qi;
	const GEN_FLT x188 = x28 * obj_qw;
	const GEN_FLT x189 = (x160 * x182) + (-1 * x188) + ((x187 + (-1 * x4 * x182)) * sensor_y) + x163 + (x182 * x157);
	const GEN_FLT x190 = x10 * obj_qw;
	const GEN_FLT x191 = ((x187 + (-1 * x2 * x182)) * sensor_z) + x190 + (x165 * x182) + x159 + (x164 * x182);
	const GEN_FLT x192 = (x57 * x189) + (x55 * x191) + (x58 * x185);
	const GEN_FLT x193 = (x46 * x191) + (x48 * x189) + (x52 * x185);
	const GEN_FLT x194 = ((-1 * x107 * x193) + (x106 * x192)) * x109;
	const GEN_FLT x195 = (x25 * x191) + (x26 * x189) + (x40 * x185);
	const GEN_FLT x196 = (x125 * x193) + (x94 * x192);
	const GEN_FLT x197 = x91 * ((x63 * x195) + (-1 * x98 * (x196 + (x128 * x195))));
	const GEN_FLT x198 = x66 * x197;
	const GEN_FLT x199 = (x67 * x197) + (x64 * (x198 + (-1 * x65 * x197)));
	const GEN_FLT x200 = (x64 * x199) + (x68 * x197);
	const GEN_FLT x201 = (x78 * x195) + (-1 * x112 * x196);
	const GEN_FLT x202 = (-1 * x201 * x111) + x194;
	const GEN_FLT x203 =
		x90 * (x201 + (x122 * x197) + (x202 * x121) +
			   (-1 * x119 *
				((-1 * x83 *
				  ((x69 * x197) + (x74 * x197) + (x64 * x200) +
				   (x64 * (x200 + (x73 * x197) +
						   (x64 * (x199 + (x72 * x197) + (x64 * ((x71 * x197) + (-1 * x118 * x197) + x198)))))))) +
				 (-1 * x202 * x117))) +
			   (x88 * x200));
	const GEN_FLT x204 = x149 * obj_qj;
	const GEN_FLT x205 = -1 * x186 * obj_qj;
	const GEN_FLT x206 = (x204 * x154) + x166 + (x204 * x152) + x188 + ((x205 + (-1 * x35 * x204)) * sensor_x);
	const GEN_FLT x207 = x14 * obj_qi;
	const GEN_FLT x208 = x184 + (x204 * x160) + (-1 * x204 * x158) + x207 + (x204 * x157);
	const GEN_FLT x209 = x14 * obj_qw;
	const GEN_FLT x210 = ((x205 + (-1 * x2 * x204)) * sensor_z) + x153 + (x204 * x165) + (-1 * x209) + (x204 * x164);
	const GEN_FLT x211 = (x55 * x210) + (x57 * x208) + (x58 * x206);
	const GEN_FLT x212 = (x46 * x210) + (x48 * x208) + (x52 * x206);
	const GEN_FLT x213 = ((-1 * x212 * x107) + (x211 * x106)) * x109;
	const GEN_FLT x214 = (x25 * x210) + (x26 * x208) + (x40 * x206);
	const GEN_FLT x215 = (x212 * x125) + (x94 * x211);
	const GEN_FLT x216 = x91 * ((x63 * x214) + (-1 * x98 * (x215 + (x214 * x128))));
	const GEN_FLT x217 = x66 * x216;
	const GEN_FLT x218 = (x67 * x216) + (x64 * (x217 + (-1 * x65 * x216)));
	const GEN_FLT x219 = (x64 * x218) + (x68 * x216);
	const GEN_FLT x220 = (x78 * x214) + (-1 * x215 * x112);
	const GEN_FLT x221 = (-1 * x220 * x111) + x213;
	const GEN_FLT x222 =
		x90 * (x220 + (x221 * x121) + (x216 * x122) +
			   (-1 * x119 *
				((-1 * x83 *
				  ((x69 * x216) + (x64 * x219) + (x74 * x216) +
				   (x64 * (x219 + (x73 * x216) +
						   (x64 * (x218 + (x72 * x216) + (x64 * ((x71 * x216) + (-1 * x216 * x118) + x217)))))))) +
				 (-1 * x221 * x117))) +
			   (x88 * x219));
	const GEN_FLT x223 = x149 * obj_qk;
	const GEN_FLT x224 = -1 * x186 * obj_qk;
	const GEN_FLT x225 = x161 + (x223 * x154) + ((x224 + (-1 * x35 * x223)) * sensor_x) + (-1 * x190) + (x223 * x152);
	const GEN_FLT x226 = ((x224 + (-1 * x4 * x223)) * sensor_y) + x155 + (x223 * x160) + x209 + (x223 * x157);
	const GEN_FLT x227 = x183 + (-1 * x223 * x167) + (x223 * x164) + (x223 * x165) + x207;
	const GEN_FLT x228 = (x55 * x227) + (x57 * x226) + (x58 * x225);
	const GEN_FLT x229 = (x46 * x227) + (x48 * x226) + (x52 * x225);
	const GEN_FLT x230 = ((-1 * x229 * x107) + (x228 * x106)) * x109;
	const GEN_FLT x231 = (x26 * x226) + (x25 * x227) + (x40 * x225);
	const GEN_FLT x232 = (x229 * x125) + (x94 * x228);
	const GEN_FLT x233 = x91 * ((x63 * x231) + (-1 * x98 * (x232 + (x231 * x128))));
	const GEN_FLT x234 = x66 * x233;
	const GEN_FLT x235 = (x67 * x233) + (x64 * (x234 + (-1 * x65 * x233)));
	const GEN_FLT x236 = (x64 * x235) + (x68 * x233);
	const GEN_FLT x237 = (x78 * x231) + (-1 * x232 * x112);
	const GEN_FLT x238 = (-1 * x237 * x111) + x230;
	const GEN_FLT x239 =
		x90 * (x237 + (x233 * x122) + (x238 * x121) +
			   (-1 * x119 *
				((-1 * x83 *
				  ((x69 * x233) + (x64 * x236) + (x74 * x233) +
				   (x64 * (x236 + (x73 * x233) +
						   (x64 * (x235 + (x72 * x233) + (x64 * ((x71 * x233) + (-1 * x233 * x118) + x234)))))))) +
				 (-1 * x238 * x117))) +
			   (x88 * x236));
	out[0] = (-1 * (x123 + (-1 * x110)) * x124) + x110 + (-1 * x123);
	out[1] = (-1 * (x138 + (-1 * x127)) * x124) + (-1 * x138) + x127;
	out[2] = (-1 * (x148 + (-1 * x139)) * x124) + (-1 * x148) + x139;
	out[3] = (-1 * (x181 + (-1 * x178)) * x124) + x178 + (-1 * x181);
	out[4] = (-1 * (x203 + (-1 * x194)) * x124) + (-1 * x203) + x194;
	out[5] = (-1 * (x222 + (-1 * x213)) * x124) + (-1 * x222) + x213;
	out[6] = (-1 * x239) + (-1 * (x239 + (-1 * x230)) * x124) + x230;
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
	const GEN_FLT x3 = x2 + x0;
	const GEN_FLT x4 = sqrt(x3 + (lh_qw * lh_qw) + x1);
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * (x1 + x0) * x5);
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = obj_qj * obj_qj;
	const GEN_FLT x9 = obj_qi * obj_qi;
	const GEN_FLT x10 = x9 + x7;
	const GEN_FLT x11 = sqrt(x10 + (obj_qw * obj_qw) + x8);
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 + (-1 * (x8 + x7) * x12);
	const GEN_FLT x14 = lh_qw * lh_qk;
	const GEN_FLT x15 = lh_qj * lh_qi;
	const GEN_FLT x16 = x15 + (-1 * x14);
	const GEN_FLT x17 = obj_qw * obj_qk;
	const GEN_FLT x18 = obj_qj * obj_qi;
	const GEN_FLT x19 = x18 + x17;
	const GEN_FLT x20 = 4 * x4 * x11;
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = lh_qw * lh_qj;
	const GEN_FLT x23 = lh_qk * lh_qi;
	const GEN_FLT x24 = x23 + x22;
	const GEN_FLT x25 = obj_qw * obj_qj;
	const GEN_FLT x26 = obj_qk * obj_qi;
	const GEN_FLT x27 = x26 + (-1 * x25);
	const GEN_FLT x28 = x20 * x27;
	const GEN_FLT x29 = (x24 * x28) + (x21 * x16) + (x6 * x13);
	const GEN_FLT x30 = 1 + (-1 * (x2 + x1) * x5);
	const GEN_FLT x31 = 1 + (-1 * (x9 + x8) * x12);
	const GEN_FLT x32 = obj_qw * obj_qi;
	const GEN_FLT x33 = obj_qk * obj_qj;
	const GEN_FLT x34 = x33 + x32;
	const GEN_FLT x35 = x34 * x12;
	const GEN_FLT x36 = x27 * x12;
	const GEN_FLT x37 = (x36 * sensor_x) + (x35 * sensor_y) + (x31 * sensor_z) + obj_pz;
	const GEN_FLT x38 = lh_qw * lh_qi;
	const GEN_FLT x39 = lh_qk * lh_qj;
	const GEN_FLT x40 = x39 + x38;
	const GEN_FLT x41 = x33 + (-1 * x32);
	const GEN_FLT x42 = x41 * x12;
	const GEN_FLT x43 = 1 + (-1 * x12 * x10);
	const GEN_FLT x44 = x12 * x19;
	const GEN_FLT x45 = (x44 * sensor_x) + (x43 * sensor_y) + obj_py + (x42 * sensor_z);
	const GEN_FLT x46 = x5 * x45;
	const GEN_FLT x47 = x23 + (-1 * x22);
	const GEN_FLT x48 = x26 + x25;
	const GEN_FLT x49 = x18 + (-1 * x17);
	const GEN_FLT x50 = (x13 * sensor_x) + (x49 * x12 * sensor_y) + (x48 * x12 * sensor_z) + obj_px;
	const GEN_FLT x51 = x5 * x50;
	const GEN_FLT x52 = (x51 * x47) + (x40 * x46) + (x30 * x37) + lh_pz;
	const GEN_FLT x53 = x5 * x37;
	const GEN_FLT x54 = (x6 * x50) + (x46 * x16) + (x53 * x24) + lh_px;
	const GEN_FLT x55 = x54 * x54;
	const GEN_FLT x56 = x52 * (1. / x55);
	const GEN_FLT x57 = 1. / x54;
	const GEN_FLT x58 = x5 * x13;
	const GEN_FLT x59 = (x30 * x36) + (x40 * x21) + (x58 * x47);
	const GEN_FLT x60 = x55 + (x52 * x52);
	const GEN_FLT x61 = 1. / x60;
	const GEN_FLT x62 = x61 * x55;
	const GEN_FLT x63 = ((-1 * x57 * x59) + (x56 * x29)) * x62;
	const GEN_FLT x64 = x39 + (-1 * x38);
	const GEN_FLT x65 = 1 + (-1 * x3 * x5);
	const GEN_FLT x66 = x15 + x14;
	const GEN_FLT x67 = (x66 * x51) + (x65 * x45) + (x64 * x53) + lh_py;
	const GEN_FLT x68 = 0.523598775598299 + tilt_0;
	const GEN_FLT x69 = cos(x68);
	const GEN_FLT x70 = 1. / x69;
	const GEN_FLT x71 = x67 * x67;
	const GEN_FLT x72 = x60 + x71;
	const GEN_FLT x73 = x70 * (1. / sqrt(x72));
	const GEN_FLT x74 = asin(x73 * x67);
	const GEN_FLT x75 = 8.0108022e-06 * x74;
	const GEN_FLT x76 = -8.0108022e-06 + (-1 * x75);
	const GEN_FLT x77 = 0.0028679863 + (x74 * x76);
	const GEN_FLT x78 = 5.3685255e-06 + (x74 * x77);
	const GEN_FLT x79 = 0.0076069798 + (x78 * x74);
	const GEN_FLT x80 = x79 * x74;
	const GEN_FLT x81 = -8.0108022e-06 + (-1.60216044e-05 * x74);
	const GEN_FLT x82 = x77 + (x81 * x74);
	const GEN_FLT x83 = x78 + (x82 * x74);
	const GEN_FLT x84 = x79 + (x83 * x74);
	const GEN_FLT x85 = (x84 * x74) + x80;
	const GEN_FLT x86 = sin(x68);
	const GEN_FLT x87 = tan(x68);
	const GEN_FLT x88 = x87 * (1. / sqrt(x60));
	const GEN_FLT x89 = x88 * x67;
	const GEN_FLT x90 = atan2(-1 * x52, x54);
	const GEN_FLT x91 = x90 + (-1 * asin(x89)) + ogeeMag_0;
	const GEN_FLT x92 = (sin(x91) * ogeePhase_0) + curve_0;
	const GEN_FLT x93 = x86 * x92;
	const GEN_FLT x94 = x69 + (-1 * x85 * x93);
	const GEN_FLT x95 = 1. / x94;
	const GEN_FLT x96 = x74 * x74;
	const GEN_FLT x97 = x92 * x96;
	const GEN_FLT x98 = x97 * x95;
	const GEN_FLT x99 = x89 + (x79 * x98);
	const GEN_FLT x100 = 1. / sqrt(1 + (-1 * (x99 * x99)));
	const GEN_FLT x101 = 1. / sqrt(1 + (-1 * x71 * (1. / x72) * (1. / (x69 * x69))));
	const GEN_FLT x102 = (x64 * x28) + (x65 * x44) + (x66 * x58);
	const GEN_FLT x103 = 2 * x67;
	const GEN_FLT x104 = 2 * x54;
	const GEN_FLT x105 = 2 * x52;
	const GEN_FLT x106 = (x59 * x105) + (x29 * x104);
	const GEN_FLT x107 = 1.0 / 2.0 * x67;
	const GEN_FLT x108 = x70 * (1. / (x72 * sqrt(x72))) * x107;
	const GEN_FLT x109 = x101 * ((x73 * x102) + (-1 * x108 * (x106 + (x103 * x102))));
	const GEN_FLT x110 = x76 * x109;
	const GEN_FLT x111 = (x77 * x109) + (x74 * (x110 + (-1 * x75 * x109)));
	const GEN_FLT x112 = (x74 * x111) + (x78 * x109);
	const GEN_FLT x113 = 2 * x80 * x92 * x95;
	const GEN_FLT x114 = x87 * (1. / (x60 * sqrt(x60))) * x107;
	const GEN_FLT x115 = (x88 * x102) + (-1 * x106 * x114);
	const GEN_FLT x116 = 1. / sqrt(1 + (-1 * (x87 * x87) * x71 * x61));
	const GEN_FLT x117 = cos(x91) * ogeePhase_0;
	const GEN_FLT x118 = x117 * ((-1 * x115 * x116) + x63);
	const GEN_FLT x119 = x85 * x86;
	const GEN_FLT x120 = 2.40324066e-05 * x74;
	const GEN_FLT x121 = x79 * x97 * (1. / (x94 * x94));
	const GEN_FLT x122 = x79 * x96 * x95;
	const GEN_FLT x123 =
		x100 * (x115 + (x118 * x122) +
				(-1 * x121 *
				 ((-1 * x93 *
				   ((x79 * x109) + (x74 * x112) + (x84 * x109) +
					(x74 * (x112 + (x83 * x109) +
							(x74 * ((x82 * x109) + x111 + (x74 * ((x81 * x109) + (-1 * x109 * x120) + x110)))))))) +
				  (-1 * x118 * x119))) +
				(x109 * x113) + (x98 * x112));
	const GEN_FLT x124 = cos(x90 + (-1 * asin(x99)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x125 = x5 * x43;
	const GEN_FLT x126 = x6 * x12;
	const GEN_FLT x127 = x34 * x20;
	const GEN_FLT x128 = (x24 * x127) + (x49 * x126) + (x16 * x125);
	const GEN_FLT x129 = x49 * x20;
	const GEN_FLT x130 = (x30 * x35) + (x40 * x125) + (x47 * x129);
	const GEN_FLT x131 = ((-1 * x57 * x130) + (x56 * x128)) * x62;
	const GEN_FLT x132 = (x64 * x127) + (x65 * x43) + (x66 * x129);
	const GEN_FLT x133 = (x105 * x130) + (x104 * x128);
	const GEN_FLT x134 = x101 * ((x73 * x132) + (-1 * x108 * (x133 + (x103 * x132))));
	const GEN_FLT x135 = x76 * x134;
	const GEN_FLT x136 = (x77 * x134) + (x74 * (x135 + (-1 * x75 * x134)));
	const GEN_FLT x137 = (x74 * x136) + (x78 * x134);
	const GEN_FLT x138 = (x88 * x132) + (-1 * x114 * x133);
	const GEN_FLT x139 = (-1 * x116 * x138) + x131;
	const GEN_FLT x140 = x119 * x117;
	const GEN_FLT x141 = x117 * x122;
	const GEN_FLT x142 =
		x100 * (x138 + (x139 * x141) +
				(-1 * x121 *
				 ((-1 * x93 *
				   ((x79 * x134) + (x74 * x137) + (x84 * x134) +
					(x74 * (x137 + (x83 * x134) +
							(x74 * (x136 + (x82 * x134) + (x74 * ((x81 * x134) + (-1 * x120 * x134) + x135)))))))) +
				  (-1 * x139 * x140))) +
				(x113 * x134) + (x98 * x137));
	const GEN_FLT x143 = x41 * x20;
	const GEN_FLT x144 = x5 * x31;
	const GEN_FLT x145 = (x24 * x144) + (x16 * x143) + (x48 * x126);
	const GEN_FLT x146 = x48 * x20;
	const GEN_FLT x147 = (x30 * x31) + (x40 * x143) + (x47 * x146);
	const GEN_FLT x148 = ((-1 * x57 * x147) + (x56 * x145)) * x62;
	const GEN_FLT x149 = (x64 * x144) + (x65 * x42) + (x66 * x146);
	const GEN_FLT x150 = (x105 * x147) + (x104 * x145);
	const GEN_FLT x151 = x101 * ((x73 * x149) + (-1 * x108 * (x150 + (x103 * x149))));
	const GEN_FLT x152 = x76 * x151;
	const GEN_FLT x153 = (x77 * x151) + (x74 * (x152 + (-1 * x75 * x151)));
	const GEN_FLT x154 = (x74 * x153) + (x78 * x151);
	const GEN_FLT x155 = (x88 * x149) + (-1 * x114 * x150);
	const GEN_FLT x156 = (-1 * x116 * x155) + x148;
	const GEN_FLT x157 =
		x100 * (x155 + (x113 * x151) + (x141 * x156) +
				(-1 * x121 *
				 ((-1 * x93 *
				   ((x79 * x151) + (x74 * x154) + (x84 * x151) +
					(x74 * (x154 + (x83 * x151) +
							(x74 * (x153 + (x82 * x151) + (x74 * ((x81 * x151) + (-1 * x120 * x151) + x152)))))))) +
				  (-1 * x140 * x156))) +
				(x98 * x154));
	out[0] = (-1 * x124 * (x123 + (-1 * x63))) + (-1 * x123) + x63;
	out[1] = (-1 * (x142 + (-1 * x131)) * x124) + (-1 * x142) + x131;
	out[2] = (-1 * x157) + (-1 * (x157 + (-1 * x148)) * x124) + x148;
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
	const GEN_FLT x2 = x1 + x0;
	const GEN_FLT x3 = lh_qk * lh_qk;
	const GEN_FLT x4 = x1 + x3;
	const GEN_FLT x5 = sqrt(x4 + (lh_qw * lh_qw) + x0);
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = obj_qk * obj_qk;
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = 2 * sqrt(x10 + (obj_qw * obj_qw) + x7);
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 =
		((x16 + (-1 * x15)) * x17) + ((x13 + x12) * x14) + ((1 + (-1 * (x8 + x7) * x11)) * sensor_z) + obj_pz;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x20 + x19;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = ((x24 + x23) * x17) + ((1 + (-1 * x11 * x10)) * sensor_y) + obj_py + ((x13 + (-1 * x12)) * x22);
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = x28 + (-1 * x27);
	const GEN_FLT x30 =
		((1 + (-1 * (x7 + x9) * x11)) * sensor_x) + ((x24 + (-1 * x23)) * x14) + ((x16 + x15) * x22) + obj_px;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = (x31 * x29) + (x21 * x26) + (x18 * (1 + (-1 * x2 * x6))) + lh_pz;
	const GEN_FLT x33 = x28 + x27;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = x36 + (-1 * x35);
	const GEN_FLT x38 = x0 + x3;
	const GEN_FLT x39 = (x30 * (1 + (-1 * x6 * x38))) + (x37 * x26) + (x34 * x33) + lh_px;
	const GEN_FLT x40 = x39 * x39;
	const GEN_FLT x41 = x40 + (x32 * x32);
	const GEN_FLT x42 = 1. / x41;
	const GEN_FLT x43 = x42 * x32;
	const GEN_FLT x44 = x20 + (-1 * x19);
	const GEN_FLT x45 = x36 + x35;
	const GEN_FLT x46 = (x45 * x31) + (x25 * (1 + (-1 * x4 * x6))) + (x44 * x34) + lh_py;
	const GEN_FLT x47 = 0.523598775598299 + tilt_0;
	const GEN_FLT x48 = cos(x47);
	const GEN_FLT x49 = 1. / x48;
	const GEN_FLT x50 = x46 * x46;
	const GEN_FLT x51 = x41 + x50;
	const GEN_FLT x52 = (1. / sqrt(x51)) * x49;
	const GEN_FLT x53 = asin(x52 * x46);
	const GEN_FLT x54 = 8.0108022e-06 * x53;
	const GEN_FLT x55 = -8.0108022e-06 + (-1 * x54);
	const GEN_FLT x56 = 0.0028679863 + (x53 * x55);
	const GEN_FLT x57 = 5.3685255e-06 + (x53 * x56);
	const GEN_FLT x58 = 0.0076069798 + (x53 * x57);
	const GEN_FLT x59 = x53 * x58;
	const GEN_FLT x60 = -8.0108022e-06 + (-1.60216044e-05 * x53);
	const GEN_FLT x61 = x56 + (x60 * x53);
	const GEN_FLT x62 = x57 + (x61 * x53);
	const GEN_FLT x63 = x58 + (x62 * x53);
	const GEN_FLT x64 = (x63 * x53) + x59;
	const GEN_FLT x65 = sin(x47);
	const GEN_FLT x66 = tan(x47);
	const GEN_FLT x67 = x66 * (1. / sqrt(x41));
	const GEN_FLT x68 = x67 * x46;
	const GEN_FLT x69 = atan2(-1 * x32, x39);
	const GEN_FLT x70 = x69 + (-1 * asin(x68)) + ogeeMag_0;
	const GEN_FLT x71 = (sin(x70) * ogeePhase_0) + curve_0;
	const GEN_FLT x72 = x71 * x65;
	const GEN_FLT x73 = x48 + (-1 * x72 * x64);
	const GEN_FLT x74 = 1. / x73;
	const GEN_FLT x75 = x53 * x53;
	const GEN_FLT x76 = x71 * x75;
	const GEN_FLT x77 = x74 * x76;
	const GEN_FLT x78 = x68 + (x77 * x58);
	const GEN_FLT x79 = 1. / sqrt(1 + (-1 * (x78 * x78)));
	const GEN_FLT x80 = x66 * (1. / (x41 * sqrt(x41)));
	const GEN_FLT x81 = x80 * x46;
	const GEN_FLT x82 = x81 * x39;
	const GEN_FLT x83 = 1. / sqrt(1 + (-1 * x50 * (1. / x51) * (1. / (x48 * x48))));
	const GEN_FLT x84 = (1. / (x51 * sqrt(x51))) * x49;
	const GEN_FLT x85 = x84 * x46;
	const GEN_FLT x86 = x85 * x39;
	const GEN_FLT x87 = x83 * x86;
	const GEN_FLT x88 = -1 * x87 * x55;
	const GEN_FLT x89 = (-1 * x87 * x56) + (x53 * (x88 + (x87 * x54)));
	const GEN_FLT x90 = (x89 * x53) + (-1 * x87 * x57);
	const GEN_FLT x91 = 1. / sqrt(1 + (-1 * (x66 * x66) * x50 * x42));
	const GEN_FLT x92 = cos(x70) * ogeePhase_0;
	const GEN_FLT x93 = x92 * ((x82 * x91) + x43);
	const GEN_FLT x94 = x64 * x65;
	const GEN_FLT x95 = 2.40324066e-05 * x53;
	const GEN_FLT x96 = x83 * x60;
	const GEN_FLT x97 = x83 * x62;
	const GEN_FLT x98 = x83 * x58;
	const GEN_FLT x99 = (1. / (x73 * x73)) * x76 * x58;
	const GEN_FLT x100 = x75 * x74 * x58;
	const GEN_FLT x101 = 2 * x39;
	const GEN_FLT x102 = x71 * x74 * x59;
	const GEN_FLT x103 = x83 * x85 * x102;
	const GEN_FLT x104 =
		x79 * ((-1 * x101 * x103) + (x93 * x100) + (x77 * x90) +
			   (-1 * x99 *
				((-1 * x72 *
				  ((x53 * x90) + (-1 * x87 * x63) + (-1 * x86 * x98) +
				   (x53 * (x90 + (-1 * x86 * x97) +
						   (x53 * (x89 + (-1 * x87 * x61) + (x53 * ((-1 * x86 * x96) + (x87 * x95) + x88)))))))) +
				 (-1 * x93 * x94))) +
			   (-1 * x82));
	const GEN_FLT x105 = cos(x69 + (-1 * asin(x78)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x106 = x83 * (x52 + (-1 * x84 * x50));
	const GEN_FLT x107 = x55 * x106;
	const GEN_FLT x108 = (x56 * x106) + (x53 * (x107 + (-1 * x54 * x106)));
	const GEN_FLT x109 = (x53 * x108) + (x57 * x106);
	const GEN_FLT x110 = x92 * x94;
	const GEN_FLT x111 = x67 * x91;
	const GEN_FLT x112 = x92 * x100;
	const GEN_FLT x113 = 2 * x102;
	const GEN_FLT x114 =
		x79 * ((x106 * x113) + (-1 * x111 * x112) +
			   (-1 * x99 *
				((-1 * x72 *
				  ((x58 * x106) + (x53 * x109) + (x63 * x106) +
				   (x53 * (x109 + (x62 * x106) +
						   (x53 * (x108 + (x61 * x106) + (x53 * ((x60 * x106) + (-1 * x95 * x106) + x107)))))))) +
				 (x110 * x111))) +
			   (x77 * x109) + x67);
	const GEN_FLT x115 = x42 * x39;
	const GEN_FLT x116 = -1 * x115;
	const GEN_FLT x117 = x46 * x32;
	const GEN_FLT x118 = x84 * x117;
	const GEN_FLT x119 = x83 * x118;
	const GEN_FLT x120 = -1 * x55 * x119;
	const GEN_FLT x121 = (-1 * x56 * x119) + (x53 * (x120 + (x54 * x119)));
	const GEN_FLT x122 = (x53 * x121) + (-1 * x57 * x119);
	const GEN_FLT x123 = x80 * x117;
	const GEN_FLT x124 = (x91 * x123) + x116;
	const GEN_FLT x125 = 2 * x32;
	const GEN_FLT x126 =
		x79 * ((-1 * x103 * x125) + (x112 * x124) +
			   (-1 * x99 *
				((-1 * x72 *
				  ((-1 * x98 * x118) + (x53 * x122) +
				   (x53 * (x122 + (-1 * x97 * x118) +
						   (x53 * (x121 + (-1 * x61 * x119) + (x53 * ((-1 * x96 * x118) + (x95 * x119) + x120)))))) +
				   (-1 * x63 * x119))) +
				 (-1 * x110 * x124))) +
			   (-1 * x123) + (x77 * x122));
	const GEN_FLT x127 = 2 * (1. / x5);
	const GEN_FLT x128 = x38 * x127;
	const GEN_FLT x129 = x30 * x128;
	const GEN_FLT x130 = x127 * lh_qw;
	const GEN_FLT x131 = x37 * x25;
	const GEN_FLT x132 = x26 * lh_qk;
	const GEN_FLT x133 = x33 * x18;
	const GEN_FLT x134 = x34 * lh_qj;
	const GEN_FLT x135 = x134 + (x130 * x133) + (-1 * x132) + (x130 * x131) + (-1 * x129 * lh_qw);
	const GEN_FLT x136 = (1. / x40) * x32;
	const GEN_FLT x137 = 1. / x39;
	const GEN_FLT x138 = x30 * x29;
	const GEN_FLT x139 = x25 * x21;
	const GEN_FLT x140 = x26 * lh_qi;
	const GEN_FLT x141 = x31 * lh_qj;
	const GEN_FLT x142 = x2 * x127;
	const GEN_FLT x143 = x18 * x142;
	const GEN_FLT x144 = (-1 * x143 * lh_qw) + (-1 * x141) + x140 + (x130 * x139) + (x130 * x138);
	const GEN_FLT x145 = x40 * x42;
	const GEN_FLT x146 = ((-1 * x137 * x144) + (x135 * x136)) * x145;
	const GEN_FLT x147 = x45 * x30;
	const GEN_FLT x148 = x127 * x147;
	const GEN_FLT x149 = x31 * lh_qk;
	const GEN_FLT x150 = x4 * x127;
	const GEN_FLT x151 = x25 * x150;
	const GEN_FLT x152 = x44 * x18;
	const GEN_FLT x153 = x34 * lh_qi;
	const GEN_FLT x154 = (-1 * x153) + (x130 * x152) + (-1 * x151 * lh_qw) + x149 + (x148 * lh_qw);
	const GEN_FLT x155 = 2 * x46;
	const GEN_FLT x156 = (x125 * x144) + (x101 * x135);
	const GEN_FLT x157 = 1.0 / 2.0 * x85;
	const GEN_FLT x158 = (x52 * x154) + (-1 * x157 * (x156 + (x154 * x155)));
	const GEN_FLT x159 = x83 * x158;
	const GEN_FLT x160 = x55 * x159;
	const GEN_FLT x161 = (x56 * x159) + (x53 * (x160 + (-1 * x54 * x159)));
	const GEN_FLT x162 = (x53 * x161) + (x57 * x159);
	const GEN_FLT x163 = 1.0 / 2.0 * x81;
	const GEN_FLT x164 = (x67 * x154) + (-1 * x163 * x156);
	const GEN_FLT x165 = (-1 * x91 * x164) + x146;
	const GEN_FLT x166 =
		x79 * ((-1 * x99 *
				((-1 * x72 *
				  ((x98 * x158) + (x63 * x159) + (x53 * x162) +
				   (x53 * (x162 + (x97 * x158) +
						   (x53 * (x161 + (x61 * x159) + (x53 * ((x60 * x159) + (-1 * x95 * x159) + x160)))))))) +
				 (-1 * x110 * x165))) +
			   (x113 * x159) + (x112 * x165) + x164 + (x77 * x162));
	const GEN_FLT x167 = x127 * lh_qi;
	const GEN_FLT x168 = x26 * lh_qj;
	const GEN_FLT x169 = x34 * lh_qk;
	const GEN_FLT x170 = x169 + (x167 * x133) + x168 + (x167 * x131) + (-1 * x129 * lh_qi);
	const GEN_FLT x171 = x26 * lh_qw;
	const GEN_FLT x172 = 4 * x5;
	const GEN_FLT x173 = -1 * x172 * lh_qi;
	const GEN_FLT x174 = (x18 * (x173 + (-1 * x142 * lh_qi))) + (x167 * x139) + x149 + x171 + (x167 * x138);
	const GEN_FLT x175 = ((-1 * x174 * x137) + (x170 * x136)) * x145;
	const GEN_FLT x176 = x34 * lh_qw;
	const GEN_FLT x177 = (x167 * x152) + (-1 * x176) + x141 + (x25 * (x173 + (-1 * x150 * lh_qi))) + (x148 * lh_qi);
	const GEN_FLT x178 = (x125 * x174) + (x101 * x170);
	const GEN_FLT x179 = (x52 * x177) + (-1 * x157 * (x178 + (x177 * x155)));
	const GEN_FLT x180 = x83 * x179;
	const GEN_FLT x181 = x55 * x180;
	const GEN_FLT x182 = (x56 * x180) + (x53 * (x181 + (-1 * x54 * x180)));
	const GEN_FLT x183 = (x53 * x182) + (x57 * x180);
	const GEN_FLT x184 = (x67 * x177) + (-1 * x163 * x178);
	const GEN_FLT x185 = (-1 * x91 * x184) + x175;
	const GEN_FLT x186 =
		x79 * (x184 + (x112 * x185) +
			   (-1 * x99 *
				((-1 * x72 *
				  ((x98 * x179) + (x53 * x183) + (x63 * x180) +
				   (x53 * (x183 + (x97 * x179) +
						   (x53 * (x182 + (x61 * x180) + (x53 * ((x60 * x180) + (-1 * x95 * x180) + x181)))))))) +
				 (-1 * x110 * x185))) +
			   (x113 * x180) + (x77 * x183));
	const GEN_FLT x187 = -1 * x172 * lh_qj;
	const GEN_FLT x188 = x127 * lh_qj;
	const GEN_FLT x189 = x176 + (x188 * x133) + x140 + (x188 * x131) + (x30 * (x187 + (-1 * x128 * lh_qj)));
	const GEN_FLT x190 = x31 * lh_qw;
	const GEN_FLT x191 = x132 + (x188 * x139) + (-1 * x190) + (x18 * (x187 + (-1 * x142 * lh_qj))) + (x188 * x138);
	const GEN_FLT x192 = ((-1 * x191 * x137) + (x189 * x136)) * x145;
	const GEN_FLT x193 = x31 * lh_qi;
	const GEN_FLT x194 = (x188 * x152) + (-1 * x151 * lh_qj) + x193 + x169 + (x148 * lh_qj);
	const GEN_FLT x195 = (x125 * x191) + (x101 * x189);
	const GEN_FLT x196 = (x52 * x194) + (-1 * x157 * (x195 + (x194 * x155)));
	const GEN_FLT x197 = x83 * x196;
	const GEN_FLT x198 = x55 * x197;
	const GEN_FLT x199 = (x56 * x197) + (x53 * (x198 + (-1 * x54 * x197)));
	const GEN_FLT x200 = (x53 * x199) + (x57 * x197);
	const GEN_FLT x201 = (x67 * x194) + (-1 * x163 * x195);
	const GEN_FLT x202 = (-1 * x91 * x201) + x192;
	const GEN_FLT x203 =
		x79 * (x201 + (x202 * x112) + (x113 * x197) +
			   (-1 * x99 *
				((-1 * x72 *
				  ((x98 * x196) + (x53 * x200) + (x63 * x197) +
				   (x53 * ((x97 * x196) + x200 +
						   (x53 * ((x61 * x197) + x199 + (x53 * ((x60 * x197) + (-1 * x95 * x197) + x198)))))))) +
				 (-1 * x202 * x110))) +
			   (x77 * x200));
	const GEN_FLT x204 = -1 * x172 * lh_qk;
	const GEN_FLT x205 = x127 * lh_qk;
	const GEN_FLT x206 = x18 * x205;
	const GEN_FLT x207 = x153 + (x33 * x206) + (x205 * x131) + (-1 * x171) + (x30 * (x204 + (-1 * x128 * lh_qk)));
	const GEN_FLT x208 = x168 + x193 + (-1 * x143 * lh_qk) + (x205 * x139) + (x205 * x138);
	const GEN_FLT x209 = ((-1 * x208 * x137) + (x207 * x136)) * x145;
	const GEN_FLT x210 = x134 + (x44 * x206) + (x205 * x147) + (x25 * (x204 + (-1 * x150 * lh_qk))) + x190;
	const GEN_FLT x211 = (x208 * x125) + (x207 * x101);
	const GEN_FLT x212 = (x52 * x210) + (-1 * x157 * (x211 + (x210 * x155)));
	const GEN_FLT x213 = x83 * x212;
	const GEN_FLT x214 = x55 * x213;
	const GEN_FLT x215 = (x56 * x213) + (x53 * (x214 + (-1 * x54 * x213)));
	const GEN_FLT x216 = (x53 * x215) + (x57 * x213);
	const GEN_FLT x217 = (x67 * x210) + (-1 * x211 * x163);
	const GEN_FLT x218 = (-1 * x91 * x217) + x209;
	const GEN_FLT x219 =
		x79 * ((x213 * x113) + x217 + (x218 * x112) +
			   (-1 * x99 *
				((-1 * x72 *
				  ((x98 * x212) + (x53 * x216) + (x63 * x213) +
				   (x53 * (x216 + (x97 * x212) +
						   (x53 * (x215 + (x61 * x213) + (x53 * ((x60 * x213) + (-1 * x95 * x213) + x214)))))))) +
				 (-1 * x218 * x110))) +
			   (x77 * x216));
	out[0] = (-1 * x105 * (x104 + (-1 * x43))) + (-1 * x104) + x43;
	out[1] = (-1 * x105 * x114) + (-1 * x114);
	out[2] = (-1 * (x126 + x115) * x105) + (-1 * x126) + x116;
	out[3] = (-1 * (x166 + (-1 * x146)) * x105) + (-1 * x166) + x146;
	out[4] = (-1 * (x186 + (-1 * x175)) * x105) + (-1 * x186) + x175;
	out[5] = (-1 * (x203 + (-1 * x192)) * x105) + (-1 * x203) + x192;
	out[6] = (-1 * (x219 + (-1 * x209)) * x105) + (-1 * x219) + x209;
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
	const GEN_FLT x6 = x5 + x4;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = 2 * sqrt(x6 + (obj_qw * obj_qw) + x7);
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = ((x13 + (-1 * x12)) * x14) + (x11 * (x10 + x9)) + ((1 + (-1 * x6 * x8)) * sensor_z) + obj_pz;
	const GEN_FLT x16 = lh_qi * lh_qi;
	const GEN_FLT x17 = lh_qk * lh_qk;
	const GEN_FLT x18 = lh_qj * lh_qj;
	const GEN_FLT x19 = x18 + x17;
	const GEN_FLT x20 = 2 * sqrt(x19 + (lh_qw * lh_qw) + x16);
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 =
		((1 + (-1 * (x5 + x7) * x8)) * sensor_y) + obj_py + ((x24 + x23) * x14) + (x22 * (x10 + (-1 * x9)));
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 =
		((1 + (-1 * (x4 + x7) * x8)) * sensor_x) + ((x24 + (-1 * x23)) * x11) + ((x13 + x12) * x22) + obj_px;
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = (x25 * (1 + (-1 * (x16 + x17) * x20))) + ((x27 + x26) * x29) + ((x3 + (-1 * x2)) * x21) + lh_py;
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = ((x33 + (-1 * x32)) * x29) + ((x3 + x2) * x31) + (x15 * (1 + (-1 * (x16 + x18) * x20))) + lh_pz;
	const GEN_FLT x35 = (x28 * (1 + (-1 * x20 * x19))) + ((x27 + (-1 * x26)) * x31) + ((x33 + x32) * x21) + lh_px;
	const GEN_FLT x36 = (x35 * x35) + (x34 * x34);
	const GEN_FLT x37 = x30 * (1. / sqrt(x36));
	const GEN_FLT x38 = x1 * x37;
	const GEN_FLT x39 = atan2(-1 * x34, x35);
	const GEN_FLT x40 = x39 + (-1 * asin(x38)) + ogeeMag_0;
	const GEN_FLT x41 = sin(x40);
	const GEN_FLT x42 = (x41 * ogeePhase_0) + curve_0;
	const GEN_FLT x43 = cos(x0);
	const GEN_FLT x44 = x30 * x30;
	const GEN_FLT x45 = x36 + x44;
	const GEN_FLT x46 = (1. / sqrt(x45)) * x30;
	const GEN_FLT x47 = asin((1. / x43) * x46);
	const GEN_FLT x48 = 8.0108022e-06 * x47;
	const GEN_FLT x49 = -8.0108022e-06 + (-1 * x48);
	const GEN_FLT x50 = 0.0028679863 + (x47 * x49);
	const GEN_FLT x51 = 5.3685255e-06 + (x50 * x47);
	const GEN_FLT x52 = 0.0076069798 + (x51 * x47);
	const GEN_FLT x53 = sin(x0);
	const GEN_FLT x54 = x52 * x47;
	const GEN_FLT x55 = -8.0108022e-06 + (-1.60216044e-05 * x47);
	const GEN_FLT x56 = x50 + (x55 * x47);
	const GEN_FLT x57 = x51 + (x56 * x47);
	const GEN_FLT x58 = x52 + (x57 * x47);
	const GEN_FLT x59 = (x58 * x47) + x54;
	const GEN_FLT x60 = x59 * x42;
	const GEN_FLT x61 = x60 * x53;
	const GEN_FLT x62 = x43 + (-1 * x61);
	const GEN_FLT x63 = 1. / x62;
	const GEN_FLT x64 = x47 * x47;
	const GEN_FLT x65 = x63 * x64;
	const GEN_FLT x66 = x65 * x52;
	const GEN_FLT x67 = x38 + (x66 * x42);
	const GEN_FLT x68 = 1. / sqrt(1 + (-1 * (x67 * x67)));
	const GEN_FLT x69 = x1 * x1;
	const GEN_FLT x70 = x37 * (1 + x69);
	const GEN_FLT x71 = 1. / (x43 * x43);
	const GEN_FLT x72 = x71 * x46 * (1. / sqrt(1 + (-1 * x71 * x44 * (1. / x45))));
	const GEN_FLT x73 = x72 * x53;
	const GEN_FLT x74 = x73 * x49;
	const GEN_FLT x75 = (x73 * x50) + (x47 * (x74 + (-1 * x73 * x48)));
	const GEN_FLT x76 = (x75 * x47) + (x73 * x51);
	const GEN_FLT x77 = cos(x40) * ogeePhase_0;
	const GEN_FLT x78 = x70 * (1. / sqrt(1 + (-1 * x69 * x44 * (1. / x36))));
	const GEN_FLT x79 = x53 * x42;
	const GEN_FLT x80 = x64 * (1. / (x62 * x62)) * x52;
	const GEN_FLT x81 = x77 * x66;
	const GEN_FLT x82 =
		x68 *
		((2 * x72 * x79 * x63 * x54) + (-1 * x81 * x78) +
		 (-1 * x80 * x42 *
		  ((-1 * x79 *
			((x73 * x52) + (x76 * x47) + (x73 * x58) +
			 (x47 * (x76 + (x73 * x57) +
					 (x47 * (x75 + (x73 * x56) + (x47 * ((x73 * x55) + (-2.40324066e-05 * x73 * x47) + x74)))))))) +
		   (x78 * x77 * x53 * x59) + (-1 * x60 * x43) + (-1 * x53))) +
		 (x76 * x65 * x42) + x70);
	const GEN_FLT x83 = (-1 * x39) + asin(x67) + (-1 * gibPhase_0);
	const GEN_FLT x84 = cos(x83) * gibMag_0;
	const GEN_FLT x85 = x80 * x61;
	const GEN_FLT x86 = (x66 + x85) * x68;
	const GEN_FLT x87 = x68 * (x81 + (x85 * x77));
	const GEN_FLT x88 = ((x66 * x41) + (x85 * x41)) * x68;
	out[0] = -1;
	out[1] = (-1 * x82 * x84) + (-1 * x82);
	out[2] = (-1 * x84 * x86) + (-1 * x86);
	out[3] = x84;
	out[4] = -1 * sin(x83);
	out[5] = (-1 * x84 * x87) + (-1 * x87);
	out[6] = (-1 * x88 * x84) + (-1 * x88);
}

/** Applying function <function reproject_axis_y_gen2 at 0x7f6253d389e0> */
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
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = 2 * sqrt(x6 + (obj_qw * obj_qw) + x3);
	const GEN_FLT x8 = obj_qw * obj_qi;
	const GEN_FLT x9 = obj_qk * obj_qj;
	const GEN_FLT x10 = x7 * sensor_y;
	const GEN_FLT x11 = obj_qw * obj_qj;
	const GEN_FLT x12 = obj_qk * obj_qi;
	const GEN_FLT x13 = x7 * sensor_x;
	const GEN_FLT x14 =
		((x12 + (-1 * x11)) * x13) + ((x9 + x8) * x10) + ((1 + (-1 * (x4 + x3) * x7)) * sensor_z) + obj_pz;
	const GEN_FLT x15 = lh_qk * lh_qk;
	const GEN_FLT x16 = lh_qj * lh_qj;
	const GEN_FLT x17 = lh_qi * lh_qi;
	const GEN_FLT x18 = x17 + x16;
	const GEN_FLT x19 = 2 * sqrt(x18 + (lh_qw * lh_qw) + x15);
	const GEN_FLT x20 = x14 * x19;
	const GEN_FLT x21 = x7 * sensor_z;
	const GEN_FLT x22 = obj_qw * obj_qk;
	const GEN_FLT x23 = obj_qj * obj_qi;
	const GEN_FLT x24 = ((x23 + x22) * x13) + ((1 + (-1 * x6 * x7)) * sensor_y) + obj_py + ((x9 + (-1 * x8)) * x21);
	const GEN_FLT x25 = lh_qw * lh_qk;
	const GEN_FLT x26 = lh_qj * lh_qi;
	const GEN_FLT x27 =
		((1 + (-1 * (x3 + x5) * x7)) * sensor_x) + ((x12 + x11) * x21) + ((x23 + (-1 * x22)) * x10) + obj_px;
	const GEN_FLT x28 = x27 * x19;
	const GEN_FLT x29 = ((x26 + x25) * x28) + (x24 * (1 + (-1 * (x17 + x15) * x19))) + ((x2 + (-1 * x1)) * x20) + lh_py;
	const GEN_FLT x30 = x24 * x19;
	const GEN_FLT x31 = lh_qw * lh_qj;
	const GEN_FLT x32 = lh_qk * lh_qi;
	const GEN_FLT x33 = ((x2 + x1) * x30) + ((x32 + (-1 * x31)) * x28) + (x14 * (1 + (-1 * x19 * x18))) + lh_pz;
	const GEN_FLT x34 =
		(x27 * (1 + (-1 * (x16 + x15) * x19))) + ((x32 + x31) * x20) + ((x26 + (-1 * x25)) * x30) + lh_px;
	const GEN_FLT x35 = (x34 * x34) + (x33 * x33);
	const GEN_FLT x36 = -1 * tan(x0) * (1. / sqrt(x35)) * x29;
	const GEN_FLT x37 = atan2(-1 * x33, x34);
	const GEN_FLT x38 = (sin(x37 + (-1 * asin(x36)) + ogeeMag_1) * ogeePhase_1) + curve_1;
	const GEN_FLT x39 = cos(x0);
	const GEN_FLT x40 = asin((1. / x39) * x29 * (1. / sqrt(x35 + (x29 * x29))));
	const GEN_FLT x41 = 0.0028679863 + (x40 * (-8.0108022e-06 + (-8.0108022e-06 * x40)));
	const GEN_FLT x42 = 5.3685255e-06 + (x40 * x41);
	const GEN_FLT x43 = 0.0076069798 + (x40 * x42);
	const GEN_FLT x44 = asin(
		x36 +
		((x40 * x40) * x43 * x38 *
		 (1. /
		  (x39 + (sin(x0) * x38 *
				  ((x40 * (x43 + (x40 * (x42 + (x40 * (x41 + (x40 * (-8.0108022e-06 + (-1.60216044e-05 * x40))))))))) +
				   (x40 * x43)))))));
	out[0] = -1.5707963267949 + (-1 * x44) + x37 + (-1 * sin((-1 * x37) + x44 + (-1 * gibPhase_1)) * gibMag_1) +
			 (-1 * phase_1);
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
	const GEN_FLT x2 = x1 + x0;
	const GEN_FLT x3 = lh_qi * lh_qi;
	const GEN_FLT x4 = sqrt(x2 + (lh_qw * lh_qw) + x3);
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * x2 * x5);
	const GEN_FLT x7 = 1 + (-1 * (x3 + x1) * x5);
	const GEN_FLT x8 = obj_qj * obj_qj;
	const GEN_FLT x9 = obj_qi * obj_qi;
	const GEN_FLT x10 = x9 + x8;
	const GEN_FLT x11 = obj_qk * obj_qk;
	const GEN_FLT x12 = x8 + x11;
	const GEN_FLT x13 = sqrt(x12 + (obj_qw * obj_qw) + x9);
	const GEN_FLT x14 = 2 * x13;
	const GEN_FLT x15 = obj_qw * obj_qi;
	const GEN_FLT x16 = obj_qk * obj_qj;
	const GEN_FLT x17 = x16 + x15;
	const GEN_FLT x18 = x14 * sensor_y;
	const GEN_FLT x19 = obj_qw * obj_qj;
	const GEN_FLT x20 = obj_qk * obj_qi;
	const GEN_FLT x21 = x20 + (-1 * x19);
	const GEN_FLT x22 = x14 * sensor_x;
	const GEN_FLT x23 = (x22 * x21) + (x18 * x17) + ((1 + (-1 * x14 * x10)) * sensor_z) + obj_pz;
	const GEN_FLT x24 = x16 + (-1 * x15);
	const GEN_FLT x25 = x14 * sensor_z;
	const GEN_FLT x26 = x9 + x11;
	const GEN_FLT x27 = obj_qw * obj_qk;
	const GEN_FLT x28 = obj_qj * obj_qi;
	const GEN_FLT x29 = x28 + x27;
	const GEN_FLT x30 = (x22 * x29) + ((1 + (-1 * x26 * x14)) * sensor_y) + obj_py + (x24 * x25);
	const GEN_FLT x31 = lh_qw * lh_qi;
	const GEN_FLT x32 = lh_qk * lh_qj;
	const GEN_FLT x33 = x32 + x31;
	const GEN_FLT x34 = x5 * x33;
	const GEN_FLT x35 = x20 + x19;
	const GEN_FLT x36 = x28 + (-1 * x27);
	const GEN_FLT x37 = ((1 + (-1 * x14 * x12)) * sensor_x) + (x36 * x18) + (x35 * x25) + obj_px;
	const GEN_FLT x38 = lh_qw * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qi;
	const GEN_FLT x40 = x39 + (-1 * x38);
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = (x41 * x37) + (x30 * x34) + (x7 * x23) + lh_pz;
	const GEN_FLT x43 = x39 + x38;
	const GEN_FLT x44 = x5 * x23;
	const GEN_FLT x45 = lh_qw * lh_qk;
	const GEN_FLT x46 = lh_qj * lh_qi;
	const GEN_FLT x47 = x46 + (-1 * x45);
	const GEN_FLT x48 = x5 * x47;
	const GEN_FLT x49 = (x6 * x37) + (x43 * x44) + (x48 * x30) + lh_px;
	const GEN_FLT x50 = x49 * x49;
	const GEN_FLT x51 = 1. / x50;
	const GEN_FLT x52 = x51 * x42;
	const GEN_FLT x53 = 1. / x49;
	const GEN_FLT x54 = x50 + (x42 * x42);
	const GEN_FLT x55 = 1. / x54;
	const GEN_FLT x56 = x50 * x55;
	const GEN_FLT x57 = x56 * ((-1 * x53 * x41) + (x6 * x52));
	const GEN_FLT x58 = x32 + (-1 * x31);
	const GEN_FLT x59 = 1 + (-1 * (x3 + x0) * x5);
	const GEN_FLT x60 = x46 + x45;
	const GEN_FLT x61 = x5 * x60;
	const GEN_FLT x62 = (x61 * x37) + (x59 * x30) + (x58 * x44) + lh_py;
	const GEN_FLT x63 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x64 = cos(x63);
	const GEN_FLT x65 = 1. / x64;
	const GEN_FLT x66 = x62 * x62;
	const GEN_FLT x67 = x54 + x66;
	const GEN_FLT x68 = (1. / sqrt(x67)) * x65;
	const GEN_FLT x69 = asin(x62 * x68);
	const GEN_FLT x70 = 8.0108022e-06 * x69;
	const GEN_FLT x71 = -8.0108022e-06 + (-1 * x70);
	const GEN_FLT x72 = 0.0028679863 + (x71 * x69);
	const GEN_FLT x73 = 5.3685255e-06 + (x72 * x69);
	const GEN_FLT x74 = 0.0076069798 + (x73 * x69);
	const GEN_FLT x75 = x74 * x69;
	const GEN_FLT x76 = -8.0108022e-06 + (-1.60216044e-05 * x69);
	const GEN_FLT x77 = x72 + (x76 * x69);
	const GEN_FLT x78 = x73 + (x77 * x69);
	const GEN_FLT x79 = x74 + (x78 * x69);
	const GEN_FLT x80 = (x79 * x69) + x75;
	const GEN_FLT x81 = tan(x63);
	const GEN_FLT x82 = x81 * (1. / sqrt(x54));
	const GEN_FLT x83 = -1 * x82 * x62;
	const GEN_FLT x84 = atan2(-1 * x42, x49);
	const GEN_FLT x85 = x84 + (-1 * asin(x83)) + ogeeMag_1;
	const GEN_FLT x86 = (sin(x85) * ogeePhase_1) + curve_1;
	const GEN_FLT x87 = sin(x63);
	const GEN_FLT x88 = x86 * x87;
	const GEN_FLT x89 = x64 + (x80 * x88);
	const GEN_FLT x90 = 1. / x89;
	const GEN_FLT x91 = x69 * x69;
	const GEN_FLT x92 = x86 * x91;
	const GEN_FLT x93 = x92 * x90;
	const GEN_FLT x94 = x83 + (x74 * x93);
	const GEN_FLT x95 = 1. / sqrt(1 + (-1 * (x94 * x94)));
	const GEN_FLT x96 = 1. / sqrt(1 + (-1 * (x81 * x81) * x66 * x55));
	const GEN_FLT x97 = 2 * x49;
	const GEN_FLT x98 = 4 * x4;
	const GEN_FLT x99 = x98 * x42;
	const GEN_FLT x100 = (x99 * x40) + (x6 * x97);
	const GEN_FLT x101 = 1.0 / 2.0 * x62;
	const GEN_FLT x102 = x81 * (1. / (x54 * sqrt(x54))) * x101;
	const GEN_FLT x103 = (-1 * x82 * x61) + (x100 * x102);
	const GEN_FLT x104 = (-1 * x96 * x103) + x57;
	const GEN_FLT x105 = cos(x85) * ogeePhase_1;
	const GEN_FLT x106 = x74 * x91 * x90;
	const GEN_FLT x107 = x105 * x106;
	const GEN_FLT x108 = 1. / sqrt(1 + (-1 * (1. / (x64 * x64)) * (1. / x67) * x66));
	const GEN_FLT x109 = x62 * x98;
	const GEN_FLT x110 = (1. / (x67 * sqrt(x67))) * x65 * x101;
	const GEN_FLT x111 = x108 * ((x61 * x68) + (-1 * x110 * (x100 + (x60 * x109))));
	const GEN_FLT x112 = 2 * x86 * x75 * x90;
	const GEN_FLT x113 = x80 * x87;
	const GEN_FLT x114 = x105 * x113;
	const GEN_FLT x115 = x71 * x111;
	const GEN_FLT x116 = 2.40324066e-05 * x69;
	const GEN_FLT x117 = (x69 * (x115 + (-1 * x70 * x111))) + (x72 * x111);
	const GEN_FLT x118 = (x73 * x111) + (x69 * x117);
	const GEN_FLT x119 = (1. / (x89 * x89)) * x74 * x92;
	const GEN_FLT x120 =
		x95 *
		(x103 + (x93 * x118) +
		 (-1 * x119 *
		  ((x88 * ((x69 * x118) + (x74 * x111) + (x79 * x111) +
				   (x69 * ((x78 * x111) + x118 +
						   (x69 * (x117 + (x77 * x111) + (x69 * ((x76 * x111) + (-1 * x111 * x116) + x115)))))))) +
		   (x104 * x114))) +
		 (x111 * x112) + (x104 * x107));
	const GEN_FLT x121 = cos(x84 + (-1 * asin(x94)) + gibPhase_1) * gibMag_1;
	const GEN_FLT x122 = 2 * x42;
	const GEN_FLT x123 = x4 * x51 * x122;
	const GEN_FLT x124 = x56 * ((-1 * x53 * x34) + (x47 * x123));
	const GEN_FLT x125 = x98 * x49;
	const GEN_FLT x126 = (x99 * x33) + (x47 * x125);
	const GEN_FLT x127 = (-1 * x82 * x59) + (x102 * x126);
	const GEN_FLT x128 = x105 * ((-1 * x96 * x127) + x124);
	const GEN_FLT x129 = 2 * x62;
	const GEN_FLT x130 = x108 * ((x68 * x59) + (-1 * x110 * (x126 + (x59 * x129))));
	const GEN_FLT x131 = x71 * x130;
	const GEN_FLT x132 = (x69 * (x131 + (-1 * x70 * x130))) + (x72 * x130);
	const GEN_FLT x133 = (x73 * x130) + (x69 * x132);
	const GEN_FLT x134 =
		x95 *
		((x93 * x133) + x127 + (x112 * x130) +
		 (-1 * x119 *
		  ((x88 * ((x69 * x133) + (x74 * x130) + (x79 * x130) +
				   (x69 * (x133 + (x78 * x130) +
						   (x69 * (x132 + (x77 * x130) + (x69 * ((x76 * x130) + x131 + (-1 * x116 * x130))))))))) +
		   (x113 * x128))) +
		 (x106 * x128));
	const GEN_FLT x135 = x56 * ((-1 * x7 * x53) + (x43 * x123));
	const GEN_FLT x136 = (x7 * x122) + (x43 * x125);
	const GEN_FLT x137 = x5 * x58;
	const GEN_FLT x138 = (-1 * x82 * x137) + (x102 * x136);
	const GEN_FLT x139 = (-1 * x96 * x138) + x135;
	const GEN_FLT x140 = x108 * ((x68 * x137) + (-1 * x110 * (x136 + (x58 * x109))));
	const GEN_FLT x141 = x71 * x140;
	const GEN_FLT x142 = (x69 * (x141 + (-1 * x70 * x140))) + (x72 * x140);
	const GEN_FLT x143 = (x73 * x140) + (x69 * x142);
	const GEN_FLT x144 =
		x95 *
		(x138 + (x93 * x143) +
		 (-1 * x119 *
		  ((x88 * ((x69 * x143) + (x74 * x140) + (x79 * x140) +
				   (x69 * ((x78 * x140) + x143 +
						   (x69 * (x142 + (x77 * x140) + (x69 * ((x76 * x140) + (-1 * x116 * x140) + x141)))))))) +
		   (x114 * x139))) +
		 (x112 * x140) + (x107 * x139));
	const GEN_FLT x145 = 2 * (1. / x13);
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
	const GEN_FLT x156 = (x153 * obj_qw) + (-1 * x151) + x155 + (x149 * obj_qw) + (-1 * x147 * obj_qw);
	const GEN_FLT x157 = x29 * sensor_x;
	const GEN_FLT x158 = x145 * x157;
	const GEN_FLT x159 = x26 * x145;
	const GEN_FLT x160 = x159 * sensor_y;
	const GEN_FLT x161 = x22 * obj_qk;
	const GEN_FLT x162 = x24 * sensor_z;
	const GEN_FLT x163 = x162 * x145;
	const GEN_FLT x164 = x14 * obj_qi;
	const GEN_FLT x165 = x164 * sensor_z;
	const GEN_FLT x166 = (-1 * x165) + (x163 * obj_qw) + x161 + (-1 * x160 * obj_qw) + (x158 * obj_qw);
	const GEN_FLT x167 = x154 * sensor_x;
	const GEN_FLT x168 = x21 * sensor_x;
	const GEN_FLT x169 = x168 * x145;
	const GEN_FLT x170 = x17 * sensor_y;
	const GEN_FLT x171 = x170 * x145;
	const GEN_FLT x172 = x164 * sensor_y;
	const GEN_FLT x173 = x10 * x145;
	const GEN_FLT x174 = x173 * sensor_z;
	const GEN_FLT x175 = (-1 * x174 * obj_qw) + x172 + (x171 * obj_qw) + (x169 * obj_qw) + (-1 * x167);
	const GEN_FLT x176 = x5 * x43;
	const GEN_FLT x177 = (x176 * x175) + (x48 * x166) + (x6 * x156);
	const GEN_FLT x178 = (x7 * x175) + (x34 * x166) + (x41 * x156);
	const GEN_FLT x179 = ((-1 * x53 * x178) + (x52 * x177)) * x56;
	const GEN_FLT x180 = (x122 * x178) + (x97 * x177);
	const GEN_FLT x181 = (x175 * x137) + (x59 * x166) + (x61 * x156);
	const GEN_FLT x182 = (-1 * x82 * x181) + (x102 * x180);
	const GEN_FLT x183 = (-1 * x96 * x182) + x179;
	const GEN_FLT x184 = x108 * ((x68 * x181) + (-1 * x110 * (x180 + (x129 * x181))));
	const GEN_FLT x185 = x71 * x184;
	const GEN_FLT x186 = (x69 * (x185 + (-1 * x70 * x184))) + (x72 * x184);
	const GEN_FLT x187 = (x73 * x184) + (x69 * x186);
	const GEN_FLT x188 =
		x95 *
		(x182 +
		 (-1 * x119 *
		  ((x88 * ((x74 * x184) + (x79 * x184) + (x69 * x187) +
				   (x69 * (x187 + (x78 * x184) +
						   (x69 * (x186 + (x77 * x184) + (x69 * ((x76 * x184) + (-1 * x116 * x184) + x185)))))))) +
		   (x114 * x183))) +
		 (x112 * x184) + (x93 * x187) + (x107 * x183));
	const GEN_FLT x189 = x154 * sensor_y;
	const GEN_FLT x190 = x150 * sensor_z;
	const GEN_FLT x191 = x190 + (x153 * obj_qi) + x189 + (x149 * obj_qi) + (-1 * x147 * obj_qi);
	const GEN_FLT x192 = 4 * x13;
	const GEN_FLT x193 = -1 * x192 * obj_qi;
	const GEN_FLT x194 = x14 * obj_qw;
	const GEN_FLT x195 = x194 * sensor_z;
	const GEN_FLT x196 =
		(-1 * x195) + ((x193 + (-1 * x159 * obj_qi)) * sensor_y) + (x163 * obj_qi) + x167 + (x158 * obj_qi);
	const GEN_FLT x197 = x194 * sensor_y;
	const GEN_FLT x198 = ((x193 + (-1 * x173 * obj_qi)) * sensor_z) + (x171 * obj_qi) + x161 + x197 + (x169 * obj_qi);
	const GEN_FLT x199 = (x176 * x198) + (x48 * x196) + (x6 * x191);
	const GEN_FLT x200 = (x7 * x198) + (x34 * x196) + (x41 * x191);
	const GEN_FLT x201 = ((-1 * x53 * x200) + (x52 * x199)) * x56;
	const GEN_FLT x202 = (x200 * x122) + (x97 * x199);
	const GEN_FLT x203 = (x198 * x137) + (x59 * x196) + (x61 * x191);
	const GEN_FLT x204 = (-1 * x82 * x203) + (x202 * x102);
	const GEN_FLT x205 = (-1 * x96 * x204) + x201;
	const GEN_FLT x206 = x108 * ((x68 * x203) + (-1 * x110 * (x202 + (x203 * x129))));
	const GEN_FLT x207 = x71 * x206;
	const GEN_FLT x208 = (x69 * (x207 + (-1 * x70 * x206))) + (x72 * x206);
	const GEN_FLT x209 = (x73 * x206) + (x69 * x208);
	const GEN_FLT x210 =
		x95 * (x204 + (x93 * x209) +
			   (-1 * x119 *
				((x88 * ((x69 * (x209 + (x78 * x206) +
								 (x69 * (x208 + (x77 * x206) + (x69 * ((x76 * x206) + (-1 * x206 * x116) + x207)))))) +
						 (x69 * x209) + (x74 * x206) + (x79 * x206))) +
				 (x205 * x114))) +
			   (x206 * x112) + (x205 * x107));
	const GEN_FLT x211 = -1 * x192 * obj_qj;
	const GEN_FLT x212 = x145 * obj_qj;
	const GEN_FLT x213 = (x212 * x152) + x172 + x195 + (x212 * x148) + ((x211 + (-1 * x146 * obj_qj)) * sensor_x);
	const GEN_FLT x214 = x22 * obj_qi;
	const GEN_FLT x215 = x190 + (x212 * x162) + (-1 * x160 * obj_qj) + x214 + (x212 * x157);
	const GEN_FLT x216 = x22 * obj_qw;
	const GEN_FLT x217 =
		((x211 + (-1 * x173 * obj_qj)) * sensor_z) + x151 + (x212 * x170) + (-1 * x216) + (x212 * x168);
	const GEN_FLT x218 = (x217 * x176) + (x48 * x215) + (x6 * x213);
	const GEN_FLT x219 = (x7 * x217) + (x34 * x215) + (x41 * x213);
	const GEN_FLT x220 = ((-1 * x53 * x219) + (x52 * x218)) * x56;
	const GEN_FLT x221 = (x219 * x122) + (x97 * x218);
	const GEN_FLT x222 = (x217 * x137) + (x59 * x215) + (x61 * x213);
	const GEN_FLT x223 = (-1 * x82 * x222) + (x221 * x102);
	const GEN_FLT x224 = (-1 * x96 * x223) + x220;
	const GEN_FLT x225 = x108 * ((x68 * x222) + (-1 * x110 * (x221 + (x222 * x129))));
	const GEN_FLT x226 = x71 * x225;
	const GEN_FLT x227 = (x69 * (x226 + (-1 * x70 * x225))) + (x72 * x225);
	const GEN_FLT x228 = (x73 * x225) + (x69 * x227);
	const GEN_FLT x229 =
		x95 *
		((x93 * x228) +
		 (-1 * x119 *
		  ((x88 * ((x69 * x228) + (x74 * x225) + (x79 * x225) +
				   (x69 * (x228 + (x78 * x225) +
						   (x69 * (x227 + (x77 * x225) + (x69 * ((x76 * x225) + (-1 * x225 * x116) + x226)))))))) +
		   (x224 * x114))) +
		 (x225 * x112) + x223 + (x224 * x107));
	const GEN_FLT x230 = x145 * obj_qk;
	const GEN_FLT x231 = -1 * x192 * obj_qk;
	const GEN_FLT x232 =
		x165 + (x230 * x152) + ((x231 + (-1 * x146 * obj_qk)) * sensor_x) + (-1 * x197) + (x230 * x148);
	const GEN_FLT x233 = x155 + (x163 * obj_qk) + ((x231 + (-1 * x159 * obj_qk)) * sensor_y) + x216 + (x230 * x157);
	const GEN_FLT x234 = x5 * x233;
	const GEN_FLT x235 = (-1 * x174 * obj_qk) + x189 + (x230 * x168) + (x230 * x170) + x214;
	const GEN_FLT x236 = (x235 * x176) + (x47 * x234) + (x6 * x232);
	const GEN_FLT x237 = (x33 * x234) + (x7 * x235) + (x41 * x232);
	const GEN_FLT x238 = ((-1 * x53 * x237) + (x52 * x236)) * x56;
	const GEN_FLT x239 = (x237 * x122) + (x97 * x236);
	const GEN_FLT x240 = (x235 * x137) + (x59 * x233) + (x61 * x232);
	const GEN_FLT x241 = (-1 * x82 * x240) + (x239 * x102);
	const GEN_FLT x242 = (-1 * x96 * x241) + x238;
	const GEN_FLT x243 = x108 * ((x68 * x240) + (-1 * x110 * (x239 + (x240 * x129))));
	const GEN_FLT x244 = x71 * x243;
	const GEN_FLT x245 = (x69 * (x244 + (-1 * x70 * x243))) + (x72 * x243);
	const GEN_FLT x246 = (x73 * x243) + (x69 * x245);
	const GEN_FLT x247 =
		x95 *
		(x241 + (x93 * x246) +
		 (-1 * x119 *
		  ((x88 * ((x69 * x246) + (x74 * x243) + (x79 * x243) +
				   (x69 * (x246 + (x78 * x243) +
						   (x69 * (x245 + (x77 * x243) + (x69 * ((x76 * x243) + (-1 * x243 * x116) + x244)))))))) +
		   (x242 * x114))) +
		 (x243 * x112) + (x242 * x107));
	out[0] = (-1 * x121 * (x120 + (-1 * x57))) + (-1 * x120) + x57;
	out[1] = (-1 * (x134 + (-1 * x124)) * x121) + (-1 * x134) + x124;
	out[2] = (-1 * (x144 + (-1 * x135)) * x121) + (-1 * x144) + x135;
	out[3] = (-1 * (x188 + (-1 * x179)) * x121) + (-1 * x188) + x179;
	out[4] = (-1 * (x210 + (-1 * x201)) * x121) + (-1 * x210) + x201;
	out[5] = (-1 * (x229 + (-1 * x220)) * x121) + (-1 * x229) + x220;
	out[6] = (-1 * (x247 + (-1 * x238)) * x121) + (-1 * x247) + x238;
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
	const GEN_FLT x2 = x1 + x0;
	const GEN_FLT x3 = lh_qi * lh_qi;
	const GEN_FLT x4 = sqrt(x2 + (lh_qw * lh_qw) + x3);
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * x2 * x5);
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = obj_qj * obj_qj;
	const GEN_FLT x9 = obj_qi * obj_qi;
	const GEN_FLT x10 = x9 + x8;
	const GEN_FLT x11 = sqrt(x10 + (obj_qw * obj_qw) + x7);
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 + (-1 * (x8 + x7) * x12);
	const GEN_FLT x14 = lh_qw * lh_qk;
	const GEN_FLT x15 = lh_qj * lh_qi;
	const GEN_FLT x16 = x15 + (-1 * x14);
	const GEN_FLT x17 = obj_qw * obj_qk;
	const GEN_FLT x18 = obj_qj * obj_qi;
	const GEN_FLT x19 = x18 + x17;
	const GEN_FLT x20 = 4 * x4 * x11;
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = obj_qw * obj_qj;
	const GEN_FLT x23 = obj_qk * obj_qi;
	const GEN_FLT x24 = x23 + (-1 * x22);
	const GEN_FLT x25 = lh_qw * lh_qj;
	const GEN_FLT x26 = lh_qk * lh_qi;
	const GEN_FLT x27 = x26 + x25;
	const GEN_FLT x28 = x20 * x27;
	const GEN_FLT x29 = (x24 * x28) + (x21 * x16) + (x6 * x13);
	const GEN_FLT x30 = 1 + (-1 * (x3 + x1) * x5);
	const GEN_FLT x31 = 1 + (-1 * x12 * x10);
	const GEN_FLT x32 = obj_qw * obj_qi;
	const GEN_FLT x33 = obj_qk * obj_qj;
	const GEN_FLT x34 = x33 + x32;
	const GEN_FLT x35 = x12 * sensor_y;
	const GEN_FLT x36 = x12 * sensor_x;
	const GEN_FLT x37 = (x36 * x24) + (x34 * x35) + (x31 * sensor_z) + obj_pz;
	const GEN_FLT x38 = lh_qw * lh_qi;
	const GEN_FLT x39 = lh_qk * lh_qj;
	const GEN_FLT x40 = x39 + x38;
	const GEN_FLT x41 = x33 + (-1 * x32);
	const GEN_FLT x42 = x12 * sensor_z;
	const GEN_FLT x43 = 1 + (-1 * (x9 + x7) * x12);
	const GEN_FLT x44 = (x36 * x19) + (x43 * sensor_y) + obj_py + (x41 * x42);
	const GEN_FLT x45 = x5 * x44;
	const GEN_FLT x46 = x26 + (-1 * x25);
	const GEN_FLT x47 = x23 + x22;
	const GEN_FLT x48 = x18 + (-1 * x17);
	const GEN_FLT x49 = (x13 * sensor_x) + (x48 * x35) + (x42 * x47) + obj_px;
	const GEN_FLT x50 = x5 * x49;
	const GEN_FLT x51 = (x50 * x46) + (x40 * x45) + (x30 * x37) + lh_pz;
	const GEN_FLT x52 = x5 * x37;
	const GEN_FLT x53 = (x6 * x49) + (x45 * x16) + (x52 * x27) + lh_px;
	const GEN_FLT x54 = x53 * x53;
	const GEN_FLT x55 = (1. / x54) * x51;
	const GEN_FLT x56 = 1. / x53;
	const GEN_FLT x57 = x5 * x13;
	const GEN_FLT x58 = x30 * x12;
	const GEN_FLT x59 = (x58 * x24) + (x40 * x21) + (x57 * x46);
	const GEN_FLT x60 = x54 + (x51 * x51);
	const GEN_FLT x61 = 1. / x60;
	const GEN_FLT x62 = x61 * x54;
	const GEN_FLT x63 = ((-1 * x56 * x59) + (x55 * x29)) * x62;
	const GEN_FLT x64 = x39 + (-1 * x38);
	const GEN_FLT x65 = 1 + (-1 * (x3 + x0) * x5);
	const GEN_FLT x66 = x15 + x14;
	const GEN_FLT x67 = (x66 * x50) + (x65 * x44) + (x64 * x52) + lh_py;
	const GEN_FLT x68 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x69 = cos(x68);
	const GEN_FLT x70 = 1. / x69;
	const GEN_FLT x71 = x67 * x67;
	const GEN_FLT x72 = x60 + x71;
	const GEN_FLT x73 = x70 * (1. / sqrt(x72));
	const GEN_FLT x74 = asin(x73 * x67);
	const GEN_FLT x75 = 8.0108022e-06 * x74;
	const GEN_FLT x76 = -8.0108022e-06 + (-1 * x75);
	const GEN_FLT x77 = 0.0028679863 + (x74 * x76);
	const GEN_FLT x78 = 5.3685255e-06 + (x74 * x77);
	const GEN_FLT x79 = 0.0076069798 + (x78 * x74);
	const GEN_FLT x80 = x79 * x74;
	const GEN_FLT x81 = -8.0108022e-06 + (-1.60216044e-05 * x74);
	const GEN_FLT x82 = x77 + (x81 * x74);
	const GEN_FLT x83 = x78 + (x82 * x74);
	const GEN_FLT x84 = x79 + (x83 * x74);
	const GEN_FLT x85 = (x84 * x74) + x80;
	const GEN_FLT x86 = tan(x68);
	const GEN_FLT x87 = x86 * (1. / sqrt(x60));
	const GEN_FLT x88 = -1 * x87 * x67;
	const GEN_FLT x89 = atan2(-1 * x51, x53);
	const GEN_FLT x90 = x89 + (-1 * asin(x88)) + ogeeMag_1;
	const GEN_FLT x91 = (sin(x90) * ogeePhase_1) + curve_1;
	const GEN_FLT x92 = sin(x68);
	const GEN_FLT x93 = x92 * x91;
	const GEN_FLT x94 = x69 + (x85 * x93);
	const GEN_FLT x95 = 1. / x94;
	const GEN_FLT x96 = x74 * x74;
	const GEN_FLT x97 = x91 * x96;
	const GEN_FLT x98 = x97 * x95;
	const GEN_FLT x99 = x88 + (x79 * x98);
	const GEN_FLT x100 = 1. / sqrt(1 + (-1 * (x99 * x99)));
	const GEN_FLT x101 = 2 * x53;
	const GEN_FLT x102 = 2 * x51;
	const GEN_FLT x103 = (x59 * x102) + (x29 * x101);
	const GEN_FLT x104 = 1.0 / 2.0 * x67;
	const GEN_FLT x105 = x86 * (1. / (x60 * sqrt(x60))) * x104;
	const GEN_FLT x106 = x65 * x12;
	const GEN_FLT x107 = x64 * x20;
	const GEN_FLT x108 = (x24 * x107) + (x19 * x106) + (x66 * x57);
	const GEN_FLT x109 = (-1 * x87 * x108) + (x103 * x105);
	const GEN_FLT x110 = 1. / sqrt(1 + (-1 * (x86 * x86) * x71 * x61));
	const GEN_FLT x111 = cos(x90) * ogeePhase_1;
	const GEN_FLT x112 = x111 * ((-1 * x109 * x110) + x63);
	const GEN_FLT x113 = x79 * x96 * x95;
	const GEN_FLT x114 = 1. / sqrt(1 + (-1 * x71 * (1. / x72) * (1. / (x69 * x69))));
	const GEN_FLT x115 = 2 * x67;
	const GEN_FLT x116 = x70 * (1. / (x72 * sqrt(x72))) * x104;
	const GEN_FLT x117 = (x73 * x108) + (-1 * x116 * (x103 + (x108 * x115)));
	const GEN_FLT x118 = x114 * x117;
	const GEN_FLT x119 = 2 * x80 * x91 * x95;
	const GEN_FLT x120 = x85 * x92;
	const GEN_FLT x121 = x76 * x118;
	const GEN_FLT x122 = 2.40324066e-05 * x74;
	const GEN_FLT x123 = (x74 * (x121 + (-1 * x75 * x118))) + (x77 * x118);
	const GEN_FLT x124 = (x78 * x118) + (x74 * x123);
	const GEN_FLT x125 = x79 * x114;
	const GEN_FLT x126 = x84 * x114;
	const GEN_FLT x127 = x79 * x97 * (1. / (x94 * x94));
	const GEN_FLT x128 =
		x100 *
		(x109 + (x98 * x124) +
		 (-1 * x127 *
		  ((x93 * ((x117 * x125) + (x74 * x124) + (x117 * x126) +
				   (x74 * ((x83 * x118) + x124 +
						   (x74 * (x123 + (x82 * x118) + (x74 * ((x81 * x118) + (-1 * x118 * x122) + x121)))))))) +
		   (x112 * x120))) +
		 (x118 * x119) + (x112 * x113));
	const GEN_FLT x129 = cos(x89 + (-1 * asin(x99)) + gibPhase_1) * gibMag_1;
	const GEN_FLT x130 = x5 * x43;
	const GEN_FLT x131 = x6 * x12;
	const GEN_FLT x132 = (x48 * x131) + (x34 * x28) + (x16 * x130);
	const GEN_FLT x133 = x48 * x20;
	const GEN_FLT x134 = (x58 * x34) + (x40 * x130) + (x46 * x133);
	const GEN_FLT x135 = ((-1 * x56 * x134) + (x55 * x132)) * x62;
	const GEN_FLT x136 = (x102 * x134) + (x101 * x132);
	const GEN_FLT x137 = (x34 * x107) + (x65 * x43) + (x66 * x133);
	const GEN_FLT x138 = (-1 * x87 * x137) + (x105 * x136);
	const GEN_FLT x139 = (-1 * x110 * x138) + x135;
	const GEN_FLT x140 = x111 * x113;
	const GEN_FLT x141 = (x73 * x137) + (-1 * x116 * (x136 + (x115 * x137)));
	const GEN_FLT x142 = x114 * x141;
	const GEN_FLT x143 = x111 * x120;
	const GEN_FLT x144 = x76 * x142;
	const GEN_FLT x145 = (x74 * (x144 + (-1 * x75 * x142))) + (x77 * x142);
	const GEN_FLT x146 = (x78 * x142) + (x74 * x145);
	const GEN_FLT x147 =
		x100 * (x138 + (x98 * x146) +
				(-1 * x127 *
				 ((x93 * ((x74 * x146) +
						  (x74 * (x146 + (x83 * x142) +
								  (x74 * (x145 + (x82 * x142) + (x74 * ((x81 * x142) + (-1 * x122 * x142) + x144)))))) +
						  (x125 * x141) + (x126 * x141))) +
				  (x139 * x143))) +
				(x119 * x142) + (x139 * x140));
	const GEN_FLT x148 = x41 * x20;
	const GEN_FLT x149 = x5 * x31;
	const GEN_FLT x150 = (x27 * x149) + (x16 * x148) + (x47 * x131);
	const GEN_FLT x151 = x47 * x20;
	const GEN_FLT x152 = (x30 * x31) + (x40 * x148) + (x46 * x151);
	const GEN_FLT x153 = ((-1 * x56 * x152) + (x55 * x150)) * x62;
	const GEN_FLT x154 = (x102 * x152) + (x101 * x150);
	const GEN_FLT x155 = (x64 * x149) + (x41 * x106) + (x66 * x151);
	const GEN_FLT x156 = (-1 * x87 * x155) + (x105 * x154);
	const GEN_FLT x157 = (-1 * x110 * x156) + x153;
	const GEN_FLT x158 = (x73 * x155) + (-1 * x116 * (x154 + (x115 * x155)));
	const GEN_FLT x159 = x114 * x158;
	const GEN_FLT x160 = x76 * x159;
	const GEN_FLT x161 = (x74 * (x160 + (-1 * x75 * x159))) + (x77 * x159);
	const GEN_FLT x162 = (x78 * x159) + (x74 * x161);
	const GEN_FLT x163 =
		x100 *
		(x156 +
		 (-1 * x127 *
		  ((x93 * ((x74 * x162) + (x125 * x158) + (x126 * x158) +
				   (x74 * (x162 + (x83 * x159) +
						   (x74 * (x161 + (x82 * x159) + (x74 * ((-1 * x122 * x159) + (x81 * x159) + x160)))))))) +
		   (x143 * x157))) +
		 (x98 * x162) + (x119 * x159) + (x140 * x157));
	out[0] = (-1 * x129 * (x128 + (-1 * x63))) + (-1 * x128) + x63;
	out[1] = (-1 * (x147 + (-1 * x135)) * x129) + (-1 * x147) + x135;
	out[2] = (-1 * (x163 + (-1 * x153)) * x129) + (-1 * x163) + x153;
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
	const GEN_FLT x2 = x1 + x0;
	const GEN_FLT x3 = lh_qk * lh_qk;
	const GEN_FLT x4 = x1 + x3;
	const GEN_FLT x5 = sqrt(x4 + (lh_qw * lh_qw) + x0);
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = x8 + x7;
	const GEN_FLT x10 = obj_qk * obj_qk;
	const GEN_FLT x11 = 2 * sqrt(x9 + (obj_qw * obj_qw) + x10);
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 = ((x16 + (-1 * x15)) * x17) + ((x13 + x12) * x14) + ((1 + (-1 * x9 * x11)) * sensor_z) + obj_pz;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x20 + x19;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 =
		((x24 + x23) * x17) + ((1 + (-1 * x11 * (x8 + x10))) * sensor_y) + obj_py + ((x13 + (-1 * x12)) * x22);
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = x28 + (-1 * x27);
	const GEN_FLT x30 =
		((1 + (-1 * x11 * (x7 + x10))) * sensor_x) + ((x24 + (-1 * x23)) * x14) + ((x16 + x15) * x22) + obj_px;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = (x31 * x29) + (x21 * x26) + (x18 * (1 + (-1 * x2 * x6))) + lh_pz;
	const GEN_FLT x33 = x28 + x27;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = x36 + (-1 * x35);
	const GEN_FLT x38 = x0 + x3;
	const GEN_FLT x39 = (x30 * (1 + (-1 * x6 * x38))) + (x37 * x26) + (x34 * x33) + lh_px;
	const GEN_FLT x40 = x39 * x39;
	const GEN_FLT x41 = x40 + (x32 * x32);
	const GEN_FLT x42 = 1. / x41;
	const GEN_FLT x43 = x42 * x32;
	const GEN_FLT x44 = x20 + (-1 * x19);
	const GEN_FLT x45 = x36 + x35;
	const GEN_FLT x46 = (x45 * x31) + (x25 * (1 + (-1 * x4 * x6))) + (x44 * x34) + lh_py;
	const GEN_FLT x47 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x48 = cos(x47);
	const GEN_FLT x49 = 1. / x48;
	const GEN_FLT x50 = x46 * x46;
	const GEN_FLT x51 = x41 + x50;
	const GEN_FLT x52 = (1. / sqrt(x51)) * x49;
	const GEN_FLT x53 = asin(x52 * x46);
	const GEN_FLT x54 = 8.0108022e-06 * x53;
	const GEN_FLT x55 = -8.0108022e-06 + (-1 * x54);
	const GEN_FLT x56 = 0.0028679863 + (x53 * x55);
	const GEN_FLT x57 = 5.3685255e-06 + (x53 * x56);
	const GEN_FLT x58 = 0.0076069798 + (x53 * x57);
	const GEN_FLT x59 = x53 * x53;
	const GEN_FLT x60 = tan(x47);
	const GEN_FLT x61 = x60 * (1. / sqrt(x41));
	const GEN_FLT x62 = -1 * x61 * x46;
	const GEN_FLT x63 = atan2(-1 * x32, x39);
	const GEN_FLT x64 = x63 + (-1 * asin(x62)) + ogeeMag_1;
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
	const GEN_FLT x75 = 1. / x74;
	const GEN_FLT x76 = x75 * x65;
	const GEN_FLT x77 = x76 * x59;
	const GEN_FLT x78 = x62 + (x77 * x58);
	const GEN_FLT x79 = 1. / sqrt(1 + (-1 * (x78 * x78)));
	const GEN_FLT x80 = x46 * x39;
	const GEN_FLT x81 = x60 * (1. / (x41 * sqrt(x41)));
	const GEN_FLT x82 = x80 * x81;
	const GEN_FLT x83 = 1. / sqrt(1 + (-1 * (x60 * x60) * x50 * x42));
	const GEN_FLT x84 = cos(x64) * ogeePhase_1;
	const GEN_FLT x85 = x84 * ((-1 * x82 * x83) + x43);
	const GEN_FLT x86 = x58 * x59;
	const GEN_FLT x87 = x86 * x75;
	const GEN_FLT x88 = (1. / (x51 * sqrt(x51))) * x49;
	const GEN_FLT x89 = 2 * x46;
	const GEN_FLT x90 = 1. / sqrt(1 + (-1 * x50 * (1. / x51) * (1. / (x48 * x48))));
	const GEN_FLT x91 = x76 * x66;
	const GEN_FLT x92 = x91 * x90;
	const GEN_FLT x93 = x71 * x72;
	const GEN_FLT x94 = x55 * x90;
	const GEN_FLT x95 = x80 * x88;
	const GEN_FLT x96 = -1 * x95 * x94;
	const GEN_FLT x97 = 2.40324066e-05 * x53;
	const GEN_FLT x98 = x90 * x95;
	const GEN_FLT x99 = (x53 * (x96 + (x54 * x98))) + (-1 * x56 * x98);
	const GEN_FLT x100 = (-1 * x57 * x98) + (x53 * x99);
	const GEN_FLT x101 = x86 * (1. / (x74 * x74)) * x65;
	const GEN_FLT x102 =
		x79 * ((x77 * x100) +
			   (-1 * x101 *
				((x73 * ((x53 * x100) + (-1 * x58 * x98) + (-1 * x70 * x98) +
						 (x53 * (x100 + (-1 * x69 * x98) +
								 (x53 * (x99 + (-1 * x68 * x98) + (x53 * ((-1 * x67 * x98) + (x98 * x97) + x96)))))))) +
				 (x85 * x93))) +
			   (-1 * x88 * x89 * x92 * x39) + (x85 * x87) + x82);
	const GEN_FLT x103 = cos(x63 + (-1 * asin(x78)) + gibPhase_1) * gibMag_1;
	const GEN_FLT x104 = x84 * x87;
	const GEN_FLT x105 = x83 * x61;
	const GEN_FLT x106 = x90 * (x52 + (-1 * x88 * x50));
	const GEN_FLT x107 = 2 * x91;
	const GEN_FLT x108 = x84 * x93;
	const GEN_FLT x109 = x55 * x106;
	const GEN_FLT x110 = (x53 * (x109 + (-1 * x54 * x106))) + (x56 * x106);
	const GEN_FLT x111 = (x57 * x106) + (x53 * x110);
	const GEN_FLT x112 =
		x79 * ((x77 * x111) +
			   (-1 * x101 *
				((x73 * ((x53 * x111) + (x70 * x106) + (x58 * x106) +
						 (x53 * (x111 + (x69 * x106) +
								 (x53 * (x110 + (x68 * x106) + (x53 * ((x67 * x106) + (-1 * x97 * x106) + x109)))))))) +
				 (x108 * x105))) +
			   (-1 * x61) + (x107 * x106) + (x105 * x104));
	const GEN_FLT x113 = x42 * x39;
	const GEN_FLT x114 = -1 * x113;
	const GEN_FLT x115 = x81 * x46;
	const GEN_FLT x116 = x32 * x115;
	const GEN_FLT x117 = (-1 * x83 * x116) + x114;
	const GEN_FLT x118 = x88 * x46;
	const GEN_FLT x119 = x32 * x118;
	const GEN_FLT x120 = -1 * x94 * x119;
	const GEN_FLT x121 = x90 * x119;
	const GEN_FLT x122 = (x53 * (x120 + (x54 * x121))) + (-1 * x56 * x121);
	const GEN_FLT x123 = (-1 * x57 * x121) + (x53 * x122);
	const GEN_FLT x124 = 2 * x32;
	const GEN_FLT x125 =
		x79 *
		((x77 * x123) + (-1 * x92 * x118 * x124) +
		 (-1 * x101 *
		  ((x73 * ((x53 * x123) + (-1 * x58 * x121) + (-1 * x70 * x121) +
				   (x53 * (x123 + (-1 * x69 * x121) +
						   (x53 * (x122 + (-1 * x68 * x121) + (x53 * ((-1 * x67 * x121) + (x97 * x121) + x120)))))))) +
		   (x108 * x117))) +
		 (x104 * x117) + x116);
	const GEN_FLT x126 = x30 * x38;
	const GEN_FLT x127 = 2 * (1. / x5);
	const GEN_FLT x128 = x127 * lh_qw;
	const GEN_FLT x129 = x37 * x25;
	const GEN_FLT x130 = x26 * lh_qk;
	const GEN_FLT x131 = x33 * x18;
	const GEN_FLT x132 = x34 * lh_qj;
	const GEN_FLT x133 = x132 + (x128 * x131) + (-1 * x130) + (x128 * x129) + (-1 * x128 * x126);
	const GEN_FLT x134 = (1. / x40) * x32;
	const GEN_FLT x135 = 1. / x39;
	const GEN_FLT x136 = x30 * x29;
	const GEN_FLT x137 = x25 * x21;
	const GEN_FLT x138 = x26 * lh_qi;
	const GEN_FLT x139 = x31 * lh_qj;
	const GEN_FLT x140 = x2 * x127;
	const GEN_FLT x141 = (-1 * x18 * x140 * lh_qw) + (-1 * x139) + x138 + (x128 * x137) + (x128 * x136);
	const GEN_FLT x142 = x40 * x42;
	const GEN_FLT x143 = ((-1 * x135 * x141) + (x133 * x134)) * x142;
	const GEN_FLT x144 = 2 * x39;
	const GEN_FLT x145 = (x124 * x141) + (x133 * x144);
	const GEN_FLT x146 = 1.0 / 2.0 * x115;
	const GEN_FLT x147 = x45 * x30;
	const GEN_FLT x148 = x31 * lh_qk;
	const GEN_FLT x149 = x4 * x25;
	const GEN_FLT x150 = x44 * x18;
	const GEN_FLT x151 = x34 * lh_qi;
	const GEN_FLT x152 = (-1 * x151) + (-1 * x128 * x149) + x148 + (x128 * x150) + (x128 * x147);
	const GEN_FLT x153 = (-1 * x61 * x152) + (x146 * x145);
	const GEN_FLT x154 = (-1 * x83 * x153) + x143;
	const GEN_FLT x155 = 1.0 / 2.0 * x118;
	const GEN_FLT x156 = x90 * ((x52 * x152) + (-1 * x155 * (x145 + (x89 * x152))));
	const GEN_FLT x157 = x55 * x156;
	const GEN_FLT x158 = (x53 * (x157 + (-1 * x54 * x156))) + (x56 * x156);
	const GEN_FLT x159 = (x57 * x156) + (x53 * x158);
	const GEN_FLT x160 =
		x79 * (x153 + (x77 * x159) +
			   (-1 * x101 *
				((x73 * ((x53 * x159) + (x58 * x156) + (x70 * x156) +
						 (x53 * (x159 + (x69 * x156) +
								 (x53 * (x158 + (x68 * x156) + (x53 * ((-1 * x97 * x156) + (x67 * x156) + x157)))))))) +
				 (x108 * x154))) +
			   (x107 * x156) + (x104 * x154));
	const GEN_FLT x161 = x127 * lh_qi;
	const GEN_FLT x162 = x26 * lh_qj;
	const GEN_FLT x163 = x34 * lh_qk;
	const GEN_FLT x164 = x163 + (x161 * x131) + x162 + (x129 * x161) + (-1 * x126 * x161);
	const GEN_FLT x165 = x26 * lh_qw;
	const GEN_FLT x166 = 4 * x5;
	const GEN_FLT x167 = -1 * x166 * lh_qi;
	const GEN_FLT x168 = (x18 * (x167 + (-1 * x140 * lh_qi))) + (x161 * x137) + x165 + x148 + (x161 * x136);
	const GEN_FLT x169 = ((-1 * x168 * x135) + (x164 * x134)) * x142;
	const GEN_FLT x170 = (x124 * x168) + (x164 * x144);
	const GEN_FLT x171 = x34 * lh_qw;
	const GEN_FLT x172 = (-1 * x171) + (x161 * x150) + (x25 * (x167 + (-1 * x4 * x161))) + x139 + (x161 * x147);
	const GEN_FLT x173 = (-1 * x61 * x172) + (x170 * x146);
	const GEN_FLT x174 = (-1 * x83 * x173) + x169;
	const GEN_FLT x175 = x90 * ((x52 * x172) + (-1 * x155 * (x170 + (x89 * x172))));
	const GEN_FLT x176 = x55 * x175;
	const GEN_FLT x177 = (x53 * (x176 + (-1 * x54 * x175))) + (x56 * x175);
	const GEN_FLT x178 = (x57 * x175) + (x53 * x177);
	const GEN_FLT x179 =
		x79 * (x173 + (x77 * x178) + (x107 * x175) +
			   (-1 * x101 *
				((x73 * ((x53 * x178) + (x58 * x175) + (x70 * x175) +
						 (x53 * (x178 + (x69 * x175) +
								 (x53 * (x177 + (x68 * x175) + (x53 * ((x67 * x175) + (-1 * x97 * x175) + x176)))))))) +
				 (x108 * x174))) +
			   (x104 * x174));
	const GEN_FLT x180 = x127 * lh_qj;
	const GEN_FLT x181 = -1 * x166 * lh_qj;
	const GEN_FLT x182 = x171 + (x180 * x131) + x138 + (x129 * x180) + (x30 * (x181 + (-1 * x38 * x180)));
	const GEN_FLT x183 = x31 * lh_qw;
	const GEN_FLT x184 = (x18 * (x181 + (-1 * x140 * lh_qj))) + x130 + (x180 * x137) + (-1 * x183) + (x180 * x136);
	const GEN_FLT x185 = ((-1 * x184 * x135) + (x182 * x134)) * x142;
	const GEN_FLT x186 = (x124 * x184) + (x182 * x144);
	const GEN_FLT x187 = x31 * lh_qi;
	const GEN_FLT x188 = x163 + (x180 * x150) + (-1 * x180 * x149) + x187 + (x180 * x147);
	const GEN_FLT x189 = (-1 * x61 * x188) + (x186 * x146);
	const GEN_FLT x190 = (-1 * x83 * x189) + x185;
	const GEN_FLT x191 = x90 * ((x52 * x188) + (-1 * x155 * (x186 + (x89 * x188))));
	const GEN_FLT x192 = x55 * x191;
	const GEN_FLT x193 = (x53 * (x192 + (-1 * x54 * x191))) + (x56 * x191);
	const GEN_FLT x194 = (x57 * x191) + (x53 * x193);
	const GEN_FLT x195 =
		x79 * (x189 + (x77 * x194) +
			   (-1 * x101 *
				((x73 * ((x53 * x194) + (x58 * x191) + (x70 * x191) +
						 (x53 * (x194 + (x69 * x191) +
								 (x53 * ((x68 * x191) + x193 + (x53 * ((x67 * x191) + (-1 * x97 * x191) + x192)))))))) +
				 (x108 * x190))) +
			   (x107 * x191) + (x104 * x190));
	const GEN_FLT x196 = x127 * lh_qk;
	const GEN_FLT x197 = -1 * x166 * lh_qk;
	const GEN_FLT x198 = x151 + (x196 * x131) + (x129 * x196) + (-1 * x165) + (x30 * (x197 + (-1 * x38 * x196)));
	const GEN_FLT x199 = (-1 * x2 * x18 * x196) + x162 + x187 + (x196 * x137) + (x196 * x136);
	const GEN_FLT x200 = ((-1 * x199 * x135) + (x198 * x134)) * x142;
	const GEN_FLT x201 = (x124 * x199) + (x198 * x144);
	const GEN_FLT x202 = x132 + (x196 * x150) + (x196 * x147) + (x25 * (x197 + (-1 * x4 * x196))) + x183;
	const GEN_FLT x203 = (-1 * x61 * x202) + (x201 * x146);
	const GEN_FLT x204 = (-1 * x83 * x203) + x200;
	const GEN_FLT x205 = x90 * ((x52 * x202) + (-1 * x155 * (x201 + (x89 * x202))));
	const GEN_FLT x206 = x55 * x205;
	const GEN_FLT x207 = (x53 * (x206 + (-1 * x54 * x205))) + (x56 * x205);
	const GEN_FLT x208 = (x57 * x205) + (x53 * x207);
	const GEN_FLT x209 =
		x79 * (x203 + (x77 * x208) +
			   (-1 * x101 *
				((x73 * ((x53 * x208) + (x58 * x205) + (x70 * x205) +
						 (x53 * (x208 + (x69 * x205) +
								 (x53 * (x207 + (x68 * x205) + (x53 * ((x67 * x205) + (-1 * x97 * x205) + x206)))))))) +
				 (x204 * x108))) +
			   (x205 * x107) + (x204 * x104));
	out[0] = (-1 * x103 * (x102 + (-1 * x43))) + (-1 * x102) + x43;
	out[1] = (-1 * x103 * x112) + (-1 * x112);
	out[2] = (-1 * (x125 + x113) * x103) + (-1 * x125) + x114;
	out[3] = (-1 * (x160 + (-1 * x143)) * x103) + (-1 * x160) + x143;
	out[4] = (-1 * (x179 + (-1 * x169)) * x103) + x169 + (-1 * x179);
	out[5] = (-1 * (x195 + (-1 * x185)) * x103) + (-1 * x195) + x185;
	out[6] = (-1 * (x209 + (-1 * x200)) * x103) + (-1 * x209) + x200;
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
	const GEN_FLT x6 = x5 + x4;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = 2 * sqrt(x6 + (obj_qw * obj_qw) + x7);
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = ((x13 + (-1 * x12)) * x14) + (x11 * (x10 + x9)) + ((1 + (-1 * x6 * x8)) * sensor_z) + obj_pz;
	const GEN_FLT x16 = lh_qi * lh_qi;
	const GEN_FLT x17 = lh_qk * lh_qk;
	const GEN_FLT x18 = lh_qj * lh_qj;
	const GEN_FLT x19 = x18 + x17;
	const GEN_FLT x20 = 2 * sqrt(x19 + (lh_qw * lh_qw) + x16);
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 =
		((1 + (-1 * (x5 + x7) * x8)) * sensor_y) + obj_py + ((x24 + x23) * x14) + (x22 * (x10 + (-1 * x9)));
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 =
		((1 + (-1 * (x4 + x7) * x8)) * sensor_x) + ((x24 + (-1 * x23)) * x11) + ((x13 + x12) * x22) + obj_px;
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = (x25 * (1 + (-1 * (x16 + x17) * x20))) + ((x27 + x26) * x29) + ((x3 + (-1 * x2)) * x21) + lh_py;
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = ((x33 + (-1 * x32)) * x29) + ((x3 + x2) * x31) + (x15 * (1 + (-1 * (x16 + x18) * x20))) + lh_pz;
	const GEN_FLT x35 = (x28 * (1 + (-1 * x20 * x19))) + ((x27 + (-1 * x26)) * x31) + ((x33 + x32) * x21) + lh_px;
	const GEN_FLT x36 = (x35 * x35) + (x34 * x34);
	const GEN_FLT x37 = x30 * (1. / sqrt(x36));
	const GEN_FLT x38 = -1 * x1 * x37;
	const GEN_FLT x39 = atan2(-1 * x34, x35);
	const GEN_FLT x40 = x39 + (-1 * asin(x38)) + ogeeMag_1;
	const GEN_FLT x41 = sin(x40);
	const GEN_FLT x42 = (x41 * ogeePhase_1) + curve_1;
	const GEN_FLT x43 = cos(x0);
	const GEN_FLT x44 = x30 * x30;
	const GEN_FLT x45 = x36 + x44;
	const GEN_FLT x46 = (1. / sqrt(x45)) * x30;
	const GEN_FLT x47 = asin((1. / x43) * x46);
	const GEN_FLT x48 = 8.0108022e-06 * x47;
	const GEN_FLT x49 = -8.0108022e-06 + (-1 * x48);
	const GEN_FLT x50 = 0.0028679863 + (x47 * x49);
	const GEN_FLT x51 = 5.3685255e-06 + (x50 * x47);
	const GEN_FLT x52 = 0.0076069798 + (x51 * x47);
	const GEN_FLT x53 = x47 * x47;
	const GEN_FLT x54 = x52 * x47;
	const GEN_FLT x55 = -8.0108022e-06 + (-1.60216044e-05 * x47);
	const GEN_FLT x56 = x50 + (x55 * x47);
	const GEN_FLT x57 = x51 + (x56 * x47);
	const GEN_FLT x58 = x52 + (x57 * x47);
	const GEN_FLT x59 = (x58 * x47) + x54;
	const GEN_FLT x60 = sin(x0);
	const GEN_FLT x61 = x60 * x42;
	const GEN_FLT x62 = x61 * x59;
	const GEN_FLT x63 = x43 + x62;
	const GEN_FLT x64 = 1. / x63;
	const GEN_FLT x65 = x64 * x53;
	const GEN_FLT x66 = x65 * x52;
	const GEN_FLT x67 = x38 + (x66 * x42);
	const GEN_FLT x68 = 1. / sqrt(1 + (-1 * (x67 * x67)));
	const GEN_FLT x69 = x1 * x1;
	const GEN_FLT x70 = x37 * (1 + x69);
	const GEN_FLT x71 = cos(x40) * ogeePhase_1;
	const GEN_FLT x72 = x71 * x66;
	const GEN_FLT x73 = x70 * (1. / sqrt(1 + (-1 * x69 * x44 * (1. / x36))));
	const GEN_FLT x74 = 1. / (x43 * x43);
	const GEN_FLT x75 = x74 * x46 * (1. / sqrt(1 + (-1 * x74 * x44 * (1. / x45))));
	const GEN_FLT x76 = x75 * x60;
	const GEN_FLT x77 = -1 * x76 * x49;
	const GEN_FLT x78 = (x47 * (x77 + (x76 * x48))) + (-1 * x76 * x50);
	const GEN_FLT x79 = (-1 * x76 * x51) + (x78 * x47);
	const GEN_FLT x80 = (1. / (x63 * x63)) * x53 * x52;
	const GEN_FLT x81 =
		x68 * ((-1 * x80 * x42 *
				((x61 * ((x79 * x47) + (-1 * x76 * x52) + (-1 * x76 * x58) +
						 (x47 * (x79 + (-1 * x76 * x57) +
								 (x47 * (x78 + (-1 * x76 * x56) +
										 (x47 * ((2.40324066e-05 * x76 * x47) + (-1 * x76 * x55) + x77)))))))) +
				 (-1 * x59 * x42 * x43) + (-1 * x71 * x73 * x60 * x59) + x60)) +
			   (-2 * x75 * x64 * x61 * x54) + (x79 * x65 * x42) + (-1 * x73 * x72) + x70);
	const GEN_FLT x82 = (-1 * x39) + asin(x67) + (-1 * gibPhase_1);
	const GEN_FLT x83 = cos(x82) * gibMag_1;
	const GEN_FLT x84 = x80 * x62;
	const GEN_FLT x85 = ((-1 * x84) + x66) * x68;
	const GEN_FLT x86 = x68 * ((-1 * x84 * x71) + x72);
	const GEN_FLT x87 = ((-1 * x84 * x41) + (x66 * x41)) * x68;
	out[0] = -1;
	out[1] = (-1 * x81 * x83) + (-1 * x81);
	out[2] = (-1 * x83 * x85) + (-1 * x85);
	out[3] = x83;
	out[4] = -1 * sin(x82);
	out[5] = (-1 * x83 * x86) + (-1 * x86);
	out[6] = (-1 * x83 * x87) + (-1 * x87);
}

/** Applying function <function reproject at 0x7f6253d38200> */
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
	const GEN_FLT x4 = x3 + x2;
	const GEN_FLT x5 = obj_qk * obj_qk;
	const GEN_FLT x6 = 2 * sqrt(x4 + (obj_qw * obj_qw) + x5);
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x6 * sensor_y;
	const GEN_FLT x10 = obj_qw * obj_qj;
	const GEN_FLT x11 = obj_qk * obj_qi;
	const GEN_FLT x12 = x6 * sensor_x;
	const GEN_FLT x13 = ((x11 + (-1 * x10)) * x12) + ((x8 + x7) * x9) + ((1 + (-1 * x4 * x6)) * sensor_z) + obj_pz;
	const GEN_FLT x14 = lh_qi * lh_qi;
	const GEN_FLT x15 = lh_qk * lh_qk;
	const GEN_FLT x16 = lh_qj * lh_qj;
	const GEN_FLT x17 = x16 + x15;
	const GEN_FLT x18 = 2 * sqrt(x17 + (lh_qw * lh_qw) + x14);
	const GEN_FLT x19 = x13 * x18;
	const GEN_FLT x20 = x6 * sensor_z;
	const GEN_FLT x21 = obj_qw * obj_qk;
	const GEN_FLT x22 = obj_qj * obj_qi;
	const GEN_FLT x23 =
		((x22 + x21) * x12) + ((1 + (-1 * (x3 + x5) * x6)) * sensor_y) + obj_py + ((x8 + (-1 * x7)) * x20);
	const GEN_FLT x24 = lh_qw * lh_qk;
	const GEN_FLT x25 = lh_qj * lh_qi;
	const GEN_FLT x26 =
		((1 + (-1 * (x2 + x5) * x6)) * sensor_x) + ((x22 + (-1 * x21)) * x9) + ((x11 + x10) * x20) + obj_px;
	const GEN_FLT x27 = x26 * x18;
	const GEN_FLT x28 = (x23 * (1 + (-1 * (x14 + x15) * x18))) + ((x1 + (-1 * x0)) * x19) + ((x25 + x24) * x27) + lh_py;
	const GEN_FLT x29 = x23 * x18;
	const GEN_FLT x30 = lh_qw * lh_qj;
	const GEN_FLT x31 = lh_qk * lh_qi;
	const GEN_FLT x32 = ((x31 + (-1 * x30)) * x27) + ((x1 + x0) * x29) + (x13 * (1 + (-1 * (x14 + x16) * x18))) + lh_pz;
	const GEN_FLT x33 = -1 * x32;
	const GEN_FLT x34 = x32 * x32;
	const GEN_FLT x35 = (x26 * (1 + (-1 * x18 * x17))) + ((x25 + (-1 * x24)) * x29) + ((x31 + x30) * x19) + lh_px;
	const GEN_FLT x36 = atan2(x35, x33);
	const GEN_FLT x37 = (-1 * x36) + (-1 * phase_0) + (-1 * asin(x28 * (1. / sqrt((x35 * x35) + x34)) * tilt_0));
	const GEN_FLT x38 =
		(-1 * atan2(-1 * x28, x33)) + (-1 * phase_1) + (-1 * asin(x35 * (1. / sqrt((x28 * x28) + x34)) * tilt_1));
	out[0] = x37 + (-1 * cos(1.5707963267949 + x37 + gibPhase_0) * gibMag_0) +
			 ((atan2(x28, x33) * atan2(x28, x33)) * curve_0);
	out[1] = x38 + ((x36 * x36) * curve_1) + (-1 * cos(1.5707963267949 + x38 + gibPhase_1) * gibMag_1);
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
	const GEN_FLT x2 = x1 + x0;
	const GEN_FLT x3 = lh_qi * lh_qi;
	const GEN_FLT x4 = sqrt(x2 + (lh_qw * lh_qw) + x3);
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * x2 * x5);
	const GEN_FLT x7 = 1 + (-1 * (x3 + x1) * x5);
	const GEN_FLT x8 = obj_qj * obj_qj;
	const GEN_FLT x9 = obj_qi * obj_qi;
	const GEN_FLT x10 = x9 + x8;
	const GEN_FLT x11 = obj_qk * obj_qk;
	const GEN_FLT x12 = sqrt(x10 + (obj_qw * obj_qw) + x11);
	const GEN_FLT x13 = 2 * x12;
	const GEN_FLT x14 = obj_qw * obj_qi;
	const GEN_FLT x15 = obj_qk * obj_qj;
	const GEN_FLT x16 = x15 + x14;
	const GEN_FLT x17 = x13 * sensor_y;
	const GEN_FLT x18 = obj_qw * obj_qj;
	const GEN_FLT x19 = obj_qk * obj_qi;
	const GEN_FLT x20 = x19 + (-1 * x18);
	const GEN_FLT x21 = x13 * sensor_x;
	const GEN_FLT x22 = (x20 * x21) + (x17 * x16) + ((1 + (-1 * x13 * x10)) * sensor_z) + obj_pz;
	const GEN_FLT x23 = x15 + (-1 * x14);
	const GEN_FLT x24 = x13 * sensor_z;
	const GEN_FLT x25 = x9 + x11;
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x27 + x26;
	const GEN_FLT x29 = (x21 * x28) + obj_py + ((1 + (-1 * x25 * x13)) * sensor_y) + (x24 * x23);
	const GEN_FLT x30 = lh_qw * lh_qi;
	const GEN_FLT x31 = lh_qk * lh_qj;
	const GEN_FLT x32 = x31 + x30;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = x19 + x18;
	const GEN_FLT x35 = x27 + (-1 * x26);
	const GEN_FLT x36 = x8 + x11;
	const GEN_FLT x37 = ((1 + (-1 * x36 * x13)) * sensor_x) + (x35 * x17) + (x34 * x24) + obj_px;
	const GEN_FLT x38 = lh_qw * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qi;
	const GEN_FLT x40 = x39 + (-1 * x38);
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = (x41 * x37) + (x33 * x29) + (x7 * x22) + lh_pz;
	const GEN_FLT x43 = 1. / x42;
	const GEN_FLT x44 = x42 * x42;
	const GEN_FLT x45 = 1. / x44;
	const GEN_FLT x46 = x39 + x38;
	const GEN_FLT x47 = x5 * x46;
	const GEN_FLT x48 = lh_qw * lh_qk;
	const GEN_FLT x49 = lh_qj * lh_qi;
	const GEN_FLT x50 = x49 + (-1 * x48);
	const GEN_FLT x51 = x5 * x50;
	const GEN_FLT x52 = (x6 * x37) + (x51 * x29) + (x47 * x22) + lh_px;
	const GEN_FLT x53 = x52 * x45;
	const GEN_FLT x54 = x52 * x52;
	const GEN_FLT x55 = x44 + x54;
	const GEN_FLT x56 = 1. / x55;
	const GEN_FLT x57 = x56 * x44;
	const GEN_FLT x58 = x57 * ((x53 * x41) + (-1 * x6 * x43));
	const GEN_FLT x59 = x31 + (-1 * x30);
	const GEN_FLT x60 = x5 * x59;
	const GEN_FLT x61 = 1 + (-1 * (x3 + x0) * x5);
	const GEN_FLT x62 = x49 + x48;
	const GEN_FLT x63 = x5 * x62;
	const GEN_FLT x64 = (x63 * x37) + (x61 * x29) + (x60 * x22) + lh_py;
	const GEN_FLT x65 = x64 * x64;
	const GEN_FLT x66 = 1. / sqrt(1 + (-1 * x65 * x56 * (tilt_0 * tilt_0)));
	const GEN_FLT x67 = 2 * x52;
	const GEN_FLT x68 = 4 * x4;
	const GEN_FLT x69 = x68 * x42;
	const GEN_FLT x70 = x69 * x40;
	const GEN_FLT x71 = 1.0 / 2.0 * x64 * (1. / (x55 * sqrt(x55))) * tilt_0;
	const GEN_FLT x72 = (1. / sqrt(x55)) * tilt_0;
	const GEN_FLT x73 = (-1 * x66 * ((x72 * x63) + (-1 * x71 * (x70 + (x6 * x67))))) + (-1 * x58);
	const GEN_FLT x74 = -1 * x42;
	const GEN_FLT x75 = atan2(x52, x74);
	const GEN_FLT x76 =
		sin(1.5707963267949 + (-1 * x75) + (-1 * phase_0) + (-1 * asin(x72 * x64)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x77 = x63 * x43;
	const GEN_FLT x78 = x64 * x45;
	const GEN_FLT x79 = x78 * x41;
	const GEN_FLT x80 = x44 + x65;
	const GEN_FLT x81 = 1. / x80;
	const GEN_FLT x82 = x81 * x44;
	const GEN_FLT x83 = 2 * x82 * atan2(x64, x74) * curve_0;
	const GEN_FLT x84 = ((x53 * x33) + (-1 * x51 * x43)) * x57;
	const GEN_FLT x85 = x68 * x52;
	const GEN_FLT x86 = x69 * x32;
	const GEN_FLT x87 = (-1 * x66 * ((x72 * x61) + (-1 * x71 * (x86 + (x85 * x50))))) + (-1 * x84);
	const GEN_FLT x88 = x61 * x43;
	const GEN_FLT x89 = x78 * x33;
	const GEN_FLT x90 = x57 * ((x7 * x53) + (-1 * x43 * x47));
	const GEN_FLT x91 = 2 * x42;
	const GEN_FLT x92 = x7 * x91;
	const GEN_FLT x93 = (-1 * x66 * ((x72 * x60) + (-1 * x71 * (x92 + (x85 * x46))))) + (-1 * x90);
	const GEN_FLT x94 = x60 * x43;
	const GEN_FLT x95 = x7 * x78;
	const GEN_FLT x96 = 2 * (1. / x12);
	const GEN_FLT x97 = x96 * x36;
	const GEN_FLT x98 = x97 * sensor_x;
	const GEN_FLT x99 = x96 * obj_qw;
	const GEN_FLT x100 = x35 * sensor_y;
	const GEN_FLT x101 = x17 * obj_qk;
	const GEN_FLT x102 = x34 * sensor_z;
	const GEN_FLT x103 = x24 * obj_qj;
	const GEN_FLT x104 = x103 + (x99 * x102) + (-1 * x101) + (x99 * x100) + (-1 * x98 * obj_qw);
	const GEN_FLT x105 = x99 * sensor_x;
	const GEN_FLT x106 = x96 * x25;
	const GEN_FLT x107 = x106 * sensor_y;
	const GEN_FLT x108 = x21 * obj_qk;
	const GEN_FLT x109 = x23 * sensor_z;
	const GEN_FLT x110 = x24 * obj_qi;
	const GEN_FLT x111 = (-1 * x110) + (x99 * x109) + x108 + (-1 * x107 * obj_qw) + (x28 * x105);
	const GEN_FLT x112 = x21 * obj_qj;
	const GEN_FLT x113 = x16 * sensor_y;
	const GEN_FLT x114 = x17 * obj_qi;
	const GEN_FLT x115 = x96 * x10;
	const GEN_FLT x116 = x115 * sensor_z;
	const GEN_FLT x117 = (-1 * x116 * obj_qw) + x114 + (x99 * x113) + (x20 * x105) + (-1 * x112);
	const GEN_FLT x118 = (x47 * x117) + (x51 * x111) + (x6 * x104);
	const GEN_FLT x119 = (x7 * x117) + (x33 * x111) + (x41 * x104);
	const GEN_FLT x120 = ((x53 * x119) + (-1 * x43 * x118)) * x57;
	const GEN_FLT x121 = x91 * x119;
	const GEN_FLT x122 = (x60 * x117) + (x61 * x111) + (x63 * x104);
	const GEN_FLT x123 = (-1 * x66 * ((x72 * x122) + (-1 * x71 * (x121 + (x67 * x118))))) + (-1 * x120);
	const GEN_FLT x124 = x43 * x122;
	const GEN_FLT x125 = x78 * x119;
	const GEN_FLT x126 = x96 * obj_qi;
	const GEN_FLT x127 = x17 * obj_qj;
	const GEN_FLT x128 = x24 * obj_qk;
	const GEN_FLT x129 = x128 + (x102 * x126) + x127 + (x100 * x126) + (-1 * x98 * obj_qi);
	const GEN_FLT x130 = x28 * sensor_x;
	const GEN_FLT x131 = 4 * x12;
	const GEN_FLT x132 = -1 * x131 * obj_qi;
	const GEN_FLT x133 = x24 * obj_qw;
	const GEN_FLT x134 =
		(-1 * x133) + (x109 * x126) + ((x132 + (-1 * x106 * obj_qi)) * sensor_y) + x112 + (x126 * x130);
	const GEN_FLT x135 = x5 * x134;
	const GEN_FLT x136 = x20 * sensor_x;
	const GEN_FLT x137 = x96 * x113;
	const GEN_FLT x138 = x17 * obj_qw;
	const GEN_FLT x139 = x138 + ((x132 + (-1 * x115 * obj_qi)) * sensor_z) + (x137 * obj_qi) + x108 + (x126 * x136);
	const GEN_FLT x140 = (x47 * x139) + (x50 * x135) + (x6 * x129);
	const GEN_FLT x141 = (x7 * x139) + (x32 * x135) + (x41 * x129);
	const GEN_FLT x142 = ((x53 * x141) + (-1 * x43 * x140)) * x57;
	const GEN_FLT x143 = x91 * x141;
	const GEN_FLT x144 = (x60 * x139) + (x61 * x134) + (x63 * x129);
	const GEN_FLT x145 = (-1 * x66 * ((x72 * x144) + (-1 * x71 * (x143 + (x67 * x140))))) + (-1 * x142);
	const GEN_FLT x146 = x43 * x144;
	const GEN_FLT x147 = x78 * x141;
	const GEN_FLT x148 = -1 * x131 * obj_qj;
	const GEN_FLT x149 = x96 * x100;
	const GEN_FLT x150 = x96 * obj_qj;
	const GEN_FLT x151 = x133 + (x102 * x150) + x114 + (x149 * obj_qj) + ((x148 + (-1 * x97 * obj_qj)) * sensor_x);
	const GEN_FLT x152 = x21 * obj_qi;
	const GEN_FLT x153 = x96 * x109;
	const GEN_FLT x154 = x128 + (x153 * obj_qj) + (-1 * x107 * obj_qj) + x152 + (x130 * x150);
	const GEN_FLT x155 = x21 * obj_qw;
	const GEN_FLT x156 =
		((x148 + (-1 * x115 * obj_qj)) * sensor_z) + x101 + (x137 * obj_qj) + (-1 * x155) + (x136 * x150);
	const GEN_FLT x157 = (x47 * x156) + (x51 * x154) + (x6 * x151);
	const GEN_FLT x158 = (x7 * x156) + (x33 * x154) + (x41 * x151);
	const GEN_FLT x159 = ((x53 * x158) + (-1 * x43 * x157)) * x57;
	const GEN_FLT x160 = x91 * x158;
	const GEN_FLT x161 = (x60 * x156) + (x61 * x154) + (x63 * x151);
	const GEN_FLT x162 = (-1 * x66 * ((x72 * x161) + (-1 * x71 * (x160 + (x67 * x157))))) + (-1 * x159);
	const GEN_FLT x163 = x43 * x161;
	const GEN_FLT x164 = x78 * x158;
	const GEN_FLT x165 = -1 * x131 * obj_qk;
	const GEN_FLT x166 = x96 * obj_qk;
	const GEN_FLT x167 =
		(x102 * x166) + x110 + ((x165 + (-1 * x97 * obj_qk)) * sensor_x) + (-1 * x138) + (x149 * obj_qk);
	const GEN_FLT x168 = (x153 * obj_qk) + ((x165 + (-1 * x106 * obj_qk)) * sensor_y) + x155 + x103 + (x166 * x130);
	const GEN_FLT x169 = x5 * x168;
	const GEN_FLT x170 = (-1 * x116 * obj_qk) + x127 + (x166 * x136) + (x137 * obj_qk) + x152;
	const GEN_FLT x171 = (x47 * x170) + (x50 * x169) + (x6 * x167);
	const GEN_FLT x172 = (x7 * x170) + (x32 * x169) + (x41 * x167);
	const GEN_FLT x173 = ((x53 * x172) + (-1 * x43 * x171)) * x57;
	const GEN_FLT x174 = x91 * x172;
	const GEN_FLT x175 = (x60 * x170) + (x61 * x168) + (x63 * x167);
	const GEN_FLT x176 = (-1 * x66 * ((x72 * x175) + (-1 * x71 * (x174 + (x67 * x171))))) + (-1 * x173);
	const GEN_FLT x177 = x43 * x175;
	const GEN_FLT x178 = x78 * x172;
	const GEN_FLT x179 = 2 * x75 * curve_1;
	const GEN_FLT x180 = 1. / sqrt(1 + (-1 * x81 * x54 * (tilt_1 * tilt_1)));
	const GEN_FLT x181 = (1. / sqrt(x80)) * tilt_1;
	const GEN_FLT x182 = x64 * x68;
	const GEN_FLT x183 = 1.0 / 2.0 * (1. / (x80 * sqrt(x80))) * x52 * tilt_1;
	const GEN_FLT x184 =
		(-1 * x180 * ((-1 * x183 * (x70 + (x62 * x182))) + (x6 * x181))) + (-1 * ((-1 * x79) + x77) * x82);
	const GEN_FLT x185 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x64, x74)) + (-1 * phase_1) + (-1 * asin(x52 * x181)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x186 = 2 * x64;
	const GEN_FLT x187 =
		(-1 * x180 * ((-1 * x183 * (x86 + (x61 * x186))) + (x51 * x181))) + (-1 * ((-1 * x89) + x88) * x82);
	const GEN_FLT x188 =
		(-1 * x180 * ((-1 * x183 * (x92 + (x59 * x182))) + (x47 * x181))) + (-1 * ((-1 * x95) + x94) * x82);
	const GEN_FLT x189 =
		(-1 * x180 * ((-1 * x183 * (x121 + (x122 * x186))) + (x118 * x181))) + (-1 * ((-1 * x125) + x124) * x82);
	const GEN_FLT x190 =
		(-1 * x180 * ((-1 * x183 * (x143 + (x186 * x144))) + (x181 * x140))) + (-1 * ((-1 * x147) + x146) * x82);
	const GEN_FLT x191 =
		(-1 * x180 * ((-1 * x183 * (x160 + (x161 * x186))) + (x181 * x157))) + (-1 * ((-1 * x164) + x163) * x82);
	const GEN_FLT x192 =
		(-1 * x180 * ((-1 * x183 * (x174 + (x175 * x186))) + (x171 * x181))) + (-1 * ((-1 * x178) + x177) * x82);
	out[0] = x73 + ((x79 + (-1 * x77)) * x83) + (x73 * x76);
	out[1] = ((x89 + (-1 * x88)) * x83) + x87 + (x87 * x76);
	out[2] = x93 + ((x95 + (-1 * x94)) * x83) + (x76 * x93);
	out[3] = x123 + ((x125 + (-1 * x124)) * x83) + (x76 * x123);
	out[4] = x145 + ((x147 + (-1 * x146)) * x83) + (x76 * x145);
	out[5] = x162 + ((x164 + (-1 * x163)) * x83) + (x76 * x162);
	out[6] = x176 + ((x178 + (-1 * x177)) * x83) + (x76 * x176);
	out[7] = x184 + (x185 * x184) + (x58 * x179);
	out[8] = x187 + (x187 * x185) + (x84 * x179);
	out[9] = x188 + (x188 * x185) + (x90 * x179);
	out[10] = x189 + (x189 * x185) + (x120 * x179);
	out[11] = x190 + (x185 * x190) + (x179 * x142);
	out[12] = x191 + (x185 * x191) + (x179 * x159);
	out[13] = x192 + (x185 * x192) + (x179 * x173);
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
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt(x3 + (lh_qw * lh_qw) + x1);
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * (x1 + x0) * x5);
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = obj_qk * obj_qk;
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = sqrt(x10 + (obj_qw * obj_qw) + x7);
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 + (-1 * (x8 + x7) * x12);
	const GEN_FLT x14 = obj_qw * obj_qi;
	const GEN_FLT x15 = obj_qk * obj_qj;
	const GEN_FLT x16 = x15 + x14;
	const GEN_FLT x17 = x12 * x16;
	const GEN_FLT x18 = obj_qw * obj_qj;
	const GEN_FLT x19 = obj_qk * obj_qi;
	const GEN_FLT x20 = x19 + (-1 * x18);
	const GEN_FLT x21 = x20 * x12;
	const GEN_FLT x22 = (x21 * sensor_x) + (x17 * sensor_y) + (x13 * sensor_z) + obj_pz;
	const GEN_FLT x23 = x15 + (-1 * x14);
	const GEN_FLT x24 = x23 * x12;
	const GEN_FLT x25 = 1 + (-1 * x12 * x10);
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x27 + x26;
	const GEN_FLT x29 = x28 * x12;
	const GEN_FLT x30 = (x25 * sensor_y) + obj_py + (x29 * sensor_x) + (x24 * sensor_z);
	const GEN_FLT x31 = lh_qw * lh_qi;
	const GEN_FLT x32 = lh_qk * lh_qj;
	const GEN_FLT x33 = x32 + x31;
	const GEN_FLT x34 = x5 * x33;
	const GEN_FLT x35 = x19 + x18;
	const GEN_FLT x36 = x27 + (-1 * x26);
	const GEN_FLT x37 = 1 + (-1 * (x7 + x9) * x12);
	const GEN_FLT x38 = (x37 * sensor_x) + (x36 * x12 * sensor_y) + (x35 * x12 * sensor_z) + obj_px;
	const GEN_FLT x39 = lh_qw * lh_qj;
	const GEN_FLT x40 = lh_qk * lh_qi;
	const GEN_FLT x41 = x40 + (-1 * x39);
	const GEN_FLT x42 = x5 * x41;
	const GEN_FLT x43 = (x42 * x38) + (x30 * x34) + (x6 * x22) + lh_pz;
	const GEN_FLT x44 = 1. / x43;
	const GEN_FLT x45 = 1 + (-1 * x3 * x5);
	const GEN_FLT x46 = lh_qw * lh_qk;
	const GEN_FLT x47 = lh_qj * lh_qi;
	const GEN_FLT x48 = x47 + (-1 * x46);
	const GEN_FLT x49 = 4 * x4 * x11;
	const GEN_FLT x50 = x49 * x28;
	const GEN_FLT x51 = x40 + x39;
	const GEN_FLT x52 = x49 * x20;
	const GEN_FLT x53 = (x50 * x48) + (x52 * x51) + (x45 * x37);
	const GEN_FLT x54 = (x6 * x21) + (x50 * x33) + (x42 * x37);
	const GEN_FLT x55 = x43 * x43;
	const GEN_FLT x56 = 1. / x55;
	const GEN_FLT x57 = x5 * x22;
	const GEN_FLT x58 = x5 * x48;
	const GEN_FLT x59 = (x45 * x38) + (x58 * x30) + (x51 * x57) + lh_px;
	const GEN_FLT x60 = x56 * x59;
	const GEN_FLT x61 = x59 * x59;
	const GEN_FLT x62 = x55 + x61;
	const GEN_FLT x63 = 1. / x62;
	const GEN_FLT x64 = x63 * x55;
	const GEN_FLT x65 = ((x60 * x54) + (-1 * x53 * x44)) * x64;
	const GEN_FLT x66 = x32 + (-1 * x31);
	const GEN_FLT x67 = 1 + (-1 * (x1 + x2) * x5);
	const GEN_FLT x68 = x47 + x46;
	const GEN_FLT x69 = x5 * x68;
	const GEN_FLT x70 = (x69 * x38) + (x67 * x30) + (x66 * x57) + lh_py;
	const GEN_FLT x71 = x70 * x70;
	const GEN_FLT x72 = 1. / sqrt(1 + (-1 * x71 * x63 * (tilt_0 * tilt_0)));
	const GEN_FLT x73 = 2 * x59;
	const GEN_FLT x74 = 2 * x43;
	const GEN_FLT x75 = x74 * x54;
	const GEN_FLT x76 = 1.0 / 2.0 * x70 * (1. / (x62 * sqrt(x62))) * tilt_0;
	const GEN_FLT x77 = (x66 * x52) + (x67 * x29) + (x69 * x37);
	const GEN_FLT x78 = (1. / sqrt(x62)) * tilt_0;
	const GEN_FLT x79 = (-1 * x72 * ((x78 * x77) + (-1 * x76 * (x75 + (x73 * x53))))) + (-1 * x65);
	const GEN_FLT x80 = -1 * x43;
	const GEN_FLT x81 = atan2(x59, x80);
	const GEN_FLT x82 =
		sin(1.5707963267949 + (-1 * x81) + (-1 * phase_0) + (-1 * asin(x70 * x78)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x83 = x77 * x44;
	const GEN_FLT x84 = x70 * x56;
	const GEN_FLT x85 = x84 * x54;
	const GEN_FLT x86 = x55 + x71;
	const GEN_FLT x87 = 1. / x86;
	const GEN_FLT x88 = x87 * x55;
	const GEN_FLT x89 = 2 * x88 * atan2(x70, x80) * curve_0;
	const GEN_FLT x90 = x45 * x12;
	const GEN_FLT x91 = x49 * x16;
	const GEN_FLT x92 = (x51 * x91) + (x90 * x36) + (x58 * x25);
	const GEN_FLT x93 = x49 * x36;
	const GEN_FLT x94 = (x6 * x17) + (x34 * x25) + (x93 * x41);
	const GEN_FLT x95 = ((x60 * x94) + (-1 * x92 * x44)) * x64;
	const GEN_FLT x96 = x74 * x94;
	const GEN_FLT x97 = (x66 * x91) + (x67 * x25) + (x68 * x93);
	const GEN_FLT x98 = (-1 * x72 * ((x78 * x97) + (-1 * x76 * (x96 + (x73 * x92))))) + (-1 * x95);
	const GEN_FLT x99 = x97 * x44;
	const GEN_FLT x100 = x84 * x94;
	const GEN_FLT x101 = x49 * x23;
	const GEN_FLT x102 = x5 * x13;
	const GEN_FLT x103 = (x51 * x102) + (x48 * x101) + (x90 * x35);
	const GEN_FLT x104 = x49 * x35;
	const GEN_FLT x105 = (x6 * x13) + (x33 * x101) + (x41 * x104);
	const GEN_FLT x106 = ((x60 * x105) + (-1 * x44 * x103)) * x64;
	const GEN_FLT x107 = x74 * x105;
	const GEN_FLT x108 = (x66 * x102) + (x67 * x24) + (x68 * x104);
	const GEN_FLT x109 = (-1 * x72 * ((x78 * x108) + (-1 * x76 * (x107 + (x73 * x103))))) + (-1 * x106);
	const GEN_FLT x110 = x44 * x108;
	const GEN_FLT x111 = x84 * x105;
	const GEN_FLT x112 = 2 * x81 * curve_1;
	const GEN_FLT x113 = 1. / sqrt(1 + (-1 * x87 * x61 * (tilt_1 * tilt_1)));
	const GEN_FLT x114 = (1. / sqrt(x86)) * tilt_1;
	const GEN_FLT x115 = 2 * x70;
	const GEN_FLT x116 = 1.0 / 2.0 * (1. / (x86 * sqrt(x86))) * x59 * tilt_1;
	const GEN_FLT x117 =
		(-1 * x113 * ((-1 * x116 * (x75 + (x77 * x115))) + (x53 * x114))) + (-1 * ((-1 * x85) + x83) * x88);
	const GEN_FLT x118 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x70, x80)) + (-1 * phase_1) + (-1 * asin(x59 * x114)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x119 =
		(-1 * x113 * ((-1 * x116 * (x96 + (x97 * x115))) + (x92 * x114))) + (-1 * x88 * ((-1 * x100) + x99));
	const GEN_FLT x120 =
		(-1 * x113 * ((-1 * x116 * (x107 + (x108 * x115))) + (x103 * x114))) + (-1 * ((-1 * x111) + x110) * x88);
	out[0] = x79 + ((x85 + (-1 * x83)) * x89) + (x82 * x79);
	out[1] = x98 + (x89 * (x100 + (-1 * x99))) + (x82 * x98);
	out[2] = x109 + ((x111 + (-1 * x110)) * x89) + (x82 * x109);
	out[3] = x117 + (x118 * x117) + (x65 * x112);
	out[4] = x119 + (x118 * x119) + (x95 * x112);
	out[5] = x120 + (x118 * x120) + (x106 * x112);
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
	const GEN_FLT x2 = x1 + x0;
	const GEN_FLT x3 = lh_qk * lh_qk;
	const GEN_FLT x4 = x1 + x3;
	const GEN_FLT x5 = sqrt(x4 + (lh_qw * lh_qw) + x0);
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = obj_qk * obj_qk;
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = 2 * sqrt(x10 + (obj_qw * obj_qw) + x7);
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 =
		((x16 + (-1 * x15)) * x17) + ((x13 + x12) * x14) + ((1 + (-1 * (x8 + x7) * x11)) * sensor_z) + obj_pz;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x20 + x19;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = ((x24 + x23) * x17) + ((1 + (-1 * x11 * x10)) * sensor_y) + obj_py + ((x13 + (-1 * x12)) * x22);
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = x28 + (-1 * x27);
	const GEN_FLT x30 =
		((1 + (-1 * (x7 + x9) * x11)) * sensor_x) + ((x24 + (-1 * x23)) * x14) + ((x16 + x15) * x22) + obj_px;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = (x31 * x29) + (x21 * x26) + (x18 * (1 + (-1 * x2 * x6))) + lh_pz;
	const GEN_FLT x33 = x28 + x27;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = x36 + (-1 * x35);
	const GEN_FLT x38 = x0 + x3;
	const GEN_FLT x39 = (x30 * (1 + (-1 * x6 * x38))) + (x37 * x26) + (x34 * x33) + lh_px;
	const GEN_FLT x40 = x39 * x39;
	const GEN_FLT x41 = x32 * x32;
	const GEN_FLT x42 = x41 + x40;
	const GEN_FLT x43 = 1. / x42;
	const GEN_FLT x44 = x43 * x32;
	const GEN_FLT x45 = x20 + (-1 * x19);
	const GEN_FLT x46 = x36 + x35;
	const GEN_FLT x47 = (x46 * x31) + (x25 * (1 + (-1 * x4 * x6))) + (x45 * x34) + lh_py;
	const GEN_FLT x48 = x47 * x39;
	const GEN_FLT x49 = x47 * x47;
	const GEN_FLT x50 = 1. / sqrt(1 + (-1 * x43 * x49 * (tilt_0 * tilt_0)));
	const GEN_FLT x51 = (1. / (x42 * sqrt(x42))) * tilt_0;
	const GEN_FLT x52 = x50 * x51;
	const GEN_FLT x53 = (x52 * x48) + x44;
	const GEN_FLT x54 = (1. / sqrt(x42)) * tilt_0;
	const GEN_FLT x55 = -1 * x32;
	const GEN_FLT x56 = atan2(x39, x55);
	const GEN_FLT x57 =
		sin(1.5707963267949 + (-1 * x56) + (-1 * phase_0) + (-1 * asin(x54 * x47)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x58 = x50 * x54;
	const GEN_FLT x59 = x41 + x49;
	const GEN_FLT x60 = 1. / x59;
	const GEN_FLT x61 = x60 * x32;
	const GEN_FLT x62 = 2 * atan2(x47, x55) * curve_0;
	const GEN_FLT x63 = x43 * x39;
	const GEN_FLT x64 = (x52 * x47 * x32) + (-1 * x63);
	const GEN_FLT x65 = x60 * x47;
	const GEN_FLT x66 = 1. / x32;
	const GEN_FLT x67 = 2 * (1. / x5);
	const GEN_FLT x68 = x67 * x38;
	const GEN_FLT x69 = x30 * lh_qw;
	const GEN_FLT x70 = x67 * x25;
	const GEN_FLT x71 = x70 * x37;
	const GEN_FLT x72 = x26 * lh_qk;
	const GEN_FLT x73 = x67 * x18;
	const GEN_FLT x74 = x73 * x33;
	const GEN_FLT x75 = x34 * lh_qj;
	const GEN_FLT x76 = x75 + (x74 * lh_qw) + (-1 * x72) + (x71 * lh_qw) + (-1 * x68 * x69);
	const GEN_FLT x77 = x67 * x29;
	const GEN_FLT x78 = x70 * x21;
	const GEN_FLT x79 = x26 * lh_qi;
	const GEN_FLT x80 = x31 * lh_qj;
	const GEN_FLT x81 = x2 * x67;
	const GEN_FLT x82 = x81 * x18;
	const GEN_FLT x83 = (-1 * x80) + x79 + (-1 * x82 * lh_qw) + (x78 * lh_qw) + (x77 * x69);
	const GEN_FLT x84 = 1. / x41;
	const GEN_FLT x85 = x84 * x39;
	const GEN_FLT x86 = x41 * x43;
	const GEN_FLT x87 = ((x83 * x85) + (-1 * x76 * x66)) * x86;
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
	const GEN_FLT x98 = (x96 * lh_qw) + (-1 * x97) + (-1 * x95 * lh_qw) + x93 + (x69 * x92);
	const GEN_FLT x99 = (-1 * x50 * ((x54 * x98) + (-1 * x91 * (x90 + (x88 * x76))))) + (-1 * x87);
	const GEN_FLT x100 = x66 * x98;
	const GEN_FLT x101 = x84 * x47;
	const GEN_FLT x102 = x83 * x101;
	const GEN_FLT x103 = x60 * x41;
	const GEN_FLT x104 = x62 * x103;
	const GEN_FLT x105 = x30 * lh_qi;
	const GEN_FLT x106 = x26 * lh_qj;
	const GEN_FLT x107 = x73 * lh_qi;
	const GEN_FLT x108 = x34 * lh_qk;
	const GEN_FLT x109 = x108 + (x33 * x107) + x106 + (x71 * lh_qi) + (-1 * x68 * x105);
	const GEN_FLT x110 = x26 * lh_qw;
	const GEN_FLT x111 = 4 * x5;
	const GEN_FLT x112 = -1 * x111 * lh_qi;
	const GEN_FLT x113 = (x18 * (x112 + (-1 * x81 * lh_qi))) + (x78 * lh_qi) + x110 + x93 + (x77 * x105);
	const GEN_FLT x114 = ((x85 * x113) + (-1 * x66 * x109)) * x86;
	const GEN_FLT x115 = x89 * x113;
	const GEN_FLT x116 = x34 * lh_qw;
	const GEN_FLT x117 = (-1 * x116) + (x45 * x107) + (x25 * (x112 + (-1 * x94 * lh_qi))) + x80 + (x92 * x105);
	const GEN_FLT x118 = (-1 * x50 * ((x54 * x117) + (-1 * x91 * (x115 + (x88 * x109))))) + (-1 * x114);
	const GEN_FLT x119 = x66 * x117;
	const GEN_FLT x120 = x101 * x113;
	const GEN_FLT x121 = -1 * x111 * lh_qj;
	const GEN_FLT x122 = x116 + (x74 * lh_qj) + x79 + (x71 * lh_qj) + (x30 * (x121 + (-1 * x68 * lh_qj)));
	const GEN_FLT x123 = x30 * lh_qj;
	const GEN_FLT x124 = x31 * lh_qw;
	const GEN_FLT x125 = x72 + (x78 * lh_qj) + (-1 * x124) + (x18 * (x121 + (-1 * x81 * lh_qj))) + (x77 * x123);
	const GEN_FLT x126 = ((x85 * x125) + (-1 * x66 * x122)) * x86;
	const GEN_FLT x127 = x89 * x125;
	const GEN_FLT x128 = x31 * lh_qi;
	const GEN_FLT x129 = x108 + (x96 * lh_qj) + (-1 * x95 * lh_qj) + x128 + (x92 * x123);
	const GEN_FLT x130 = (-1 * x50 * ((x54 * x129) + (-1 * x91 * (x127 + (x88 * x122))))) + (-1 * x126);
	const GEN_FLT x131 = x66 * x129;
	const GEN_FLT x132 = x101 * x125;
	const GEN_FLT x133 = -1 * x111 * lh_qk;
	const GEN_FLT x134 = x97 + (x74 * lh_qk) + (x71 * lh_qk) + (-1 * x110) + (x30 * (x133 + (-1 * x68 * lh_qk)));
	const GEN_FLT x135 = x30 * lh_qk;
	const GEN_FLT x136 = (-1 * x82 * lh_qk) + (x78 * lh_qk) + x106 + x128 + (x77 * x135);
	const GEN_FLT x137 = ((x85 * x136) + (-1 * x66 * x134)) * x86;
	const GEN_FLT x138 = x89 * x136;
	const GEN_FLT x139 = x75 + (x96 * lh_qk) + (x92 * x135) + (x25 * (x133 + (-1 * x94 * lh_qk))) + x124;
	const GEN_FLT x140 = (-1 * x50 * ((x54 * x139) + (-1 * x91 * (x138 + (x88 * x134))))) + (-1 * x137);
	const GEN_FLT x141 = x66 * x139;
	const GEN_FLT x142 = x101 * x136;
	const GEN_FLT x143 = 1. / sqrt(1 + (-1 * x60 * x40 * (tilt_1 * tilt_1)));
	const GEN_FLT x144 = (1. / sqrt(x59)) * tilt_1;
	const GEN_FLT x145 = x144 * x143;
	const GEN_FLT x146 = 2 * x56 * curve_1;
	const GEN_FLT x147 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x47, x55)) + (-1 * phase_1) + (-1 * asin(x39 * x144)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x148 = (1. / (x59 * sqrt(x59))) * tilt_1;
	const GEN_FLT x149 = x143 * x148;
	const GEN_FLT x150 = (x48 * x149) + (-1 * x61);
	const GEN_FLT x151 = (x32 * x39 * x149) + x65;
	const GEN_FLT x152 = 2 * x47;
	const GEN_FLT x153 = 1.0 / 2.0 * x39 * x148;
	const GEN_FLT x154 =
		(-1 * x143 * ((-1 * x153 * (x90 + (x98 * x152))) + (x76 * x144))) + (-1 * ((-1 * x102) + x100) * x103);
	const GEN_FLT x155 =
		(-1 * x143 * ((-1 * x153 * (x115 + (x117 * x152))) + (x109 * x144))) + (-1 * ((-1 * x120) + x119) * x103);
	const GEN_FLT x156 =
		(-1 * x143 * ((-1 * x153 * (x127 + (x129 * x152))) + (x122 * x144))) + (-1 * ((-1 * x132) + x131) * x103);
	const GEN_FLT x157 =
		(-1 * x143 * ((-1 * x153 * (x138 + (x139 * x152))) + (x134 * x144))) + (-1 * ((-1 * x142) + x141) * x103);
	out[0] = x53 + (x53 * x57);
	out[1] = (-1 * x61 * x62) + (-1 * x58 * x57) + (-1 * x58);
	out[2] = x64 + (x62 * x65) + (x64 * x57);
	out[3] = x99 + ((x102 + (-1 * x100)) * x104) + (x57 * x99);
	out[4] = ((x120 + (-1 * x119)) * x104) + x118 + (x57 * x118);
	out[5] = x130 + ((x132 + (-1 * x131)) * x104) + (x57 * x130);
	out[6] = x140 + ((x142 + (-1 * x141)) * x104) + (x57 * x140);
	out[7] = (-1 * x145 * x147) + (-1 * x44 * x146) + (-1 * x145);
	out[8] = x150 + (x147 * x150);
	out[9] = (x147 * x151) + x151 + (x63 * x146);
	out[10] = x154 + (x147 * x154) + (x87 * x146);
	out[11] = x155 + (x147 * x155) + (x114 * x146);
	out[12] = x156 + (x147 * x156) + (x126 * x146);
	out[13] = x157 + (x147 * x157) + (x137 * x146);
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
	const GEN_FLT x4 = x3 + x2;
	const GEN_FLT x5 = obj_qk * obj_qk;
	const GEN_FLT x6 = 2 * sqrt(x4 + (obj_qw * obj_qw) + x5);
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x6 * sensor_y;
	const GEN_FLT x10 = obj_qw * obj_qj;
	const GEN_FLT x11 = obj_qk * obj_qi;
	const GEN_FLT x12 = x6 * sensor_x;
	const GEN_FLT x13 = ((x11 + (-1 * x10)) * x12) + ((x8 + x7) * x9) + ((1 + (-1 * x4 * x6)) * sensor_z) + obj_pz;
	const GEN_FLT x14 = lh_qi * lh_qi;
	const GEN_FLT x15 = lh_qk * lh_qk;
	const GEN_FLT x16 = lh_qj * lh_qj;
	const GEN_FLT x17 = x16 + x15;
	const GEN_FLT x18 = 2 * sqrt(x17 + (lh_qw * lh_qw) + x14);
	const GEN_FLT x19 = x13 * x18;
	const GEN_FLT x20 = x6 * sensor_z;
	const GEN_FLT x21 = obj_qw * obj_qk;
	const GEN_FLT x22 = obj_qj * obj_qi;
	const GEN_FLT x23 =
		((x22 + x21) * x12) + ((1 + (-1 * (x3 + x5) * x6)) * sensor_y) + obj_py + ((x8 + (-1 * x7)) * x20);
	const GEN_FLT x24 = lh_qw * lh_qk;
	const GEN_FLT x25 = lh_qj * lh_qi;
	const GEN_FLT x26 =
		((1 + (-1 * (x2 + x5) * x6)) * sensor_x) + ((x22 + (-1 * x21)) * x9) + ((x11 + x10) * x20) + obj_px;
	const GEN_FLT x27 = x26 * x18;
	const GEN_FLT x28 = (x23 * (1 + (-1 * (x14 + x15) * x18))) + ((x1 + (-1 * x0)) * x19) + ((x25 + x24) * x27) + lh_py;
	const GEN_FLT x29 = x23 * x18;
	const GEN_FLT x30 = lh_qw * lh_qj;
	const GEN_FLT x31 = lh_qk * lh_qi;
	const GEN_FLT x32 = ((x31 + (-1 * x30)) * x27) + ((x1 + x0) * x29) + (x13 * (1 + (-1 * (x14 + x16) * x18))) + lh_pz;
	const GEN_FLT x33 = x32 * x32;
	const GEN_FLT x34 = (x26 * (1 + (-1 * x18 * x17))) + ((x25 + (-1 * x24)) * x29) + ((x31 + x30) * x19) + lh_px;
	const GEN_FLT x35 = x34 * x34;
	const GEN_FLT x36 = x35 + x33;
	const GEN_FLT x37 = (1. / sqrt(x36)) * x28;
	const GEN_FLT x38 = -1 * x32;
	const GEN_FLT x39 = atan2(x34, x38);
	const GEN_FLT x40 = 1.5707963267949 + (-1 * x39) + (-1 * phase_0) + (-1 * asin(x37 * tilt_0)) + gibPhase_0;
	const GEN_FLT x41 = sin(x40) * gibMag_0;
	const GEN_FLT x42 = x28 * x28;
	const GEN_FLT x43 = x37 * (1. / sqrt(1 + (-1 * x42 * (1. / x36) * (tilt_0 * tilt_0))));
	const GEN_FLT x44 = x42 + x33;
	const GEN_FLT x45 = (1. / sqrt(x44)) * x34;
	const GEN_FLT x46 =
		1.5707963267949 + (-1 * atan2(-1 * x28, x38)) + (-1 * phase_1) + (-1 * asin(x45 * tilt_1)) + gibPhase_1;
	const GEN_FLT x47 = sin(x46) * gibMag_1;
	const GEN_FLT x48 = x45 * (1. / sqrt(1 + (-1 * (1. / x44) * x35 * (tilt_1 * tilt_1))));
	out[0] = -1 + (-1 * x41);
	out[1] = (-1 * x43) + (-1 * x41 * x43);
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
	out[22] = (-1 * x47 * x48) + (-1 * x48);
	out[23] = x39 * x39;
	out[24] = x47;
	out[25] = -1 * cos(x46);
	out[26] = 0;
	out[27] = 0;
}

/** Applying function <function reproject_axis_x at 0x7f6253d38290> */
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
	const GEN_FLT x4 = x3 + x2;
	const GEN_FLT x5 = obj_qk * obj_qk;
	const GEN_FLT x6 = 2 * sqrt(x4 + (obj_qw * obj_qw) + x5);
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x6 * sensor_y;
	const GEN_FLT x10 = obj_qw * obj_qj;
	const GEN_FLT x11 = obj_qk * obj_qi;
	const GEN_FLT x12 = x6 * sensor_x;
	const GEN_FLT x13 = ((x11 + (-1 * x10)) * x12) + ((x8 + x7) * x9) + ((1 + (-1 * x4 * x6)) * sensor_z) + obj_pz;
	const GEN_FLT x14 = lh_qi * lh_qi;
	const GEN_FLT x15 = lh_qk * lh_qk;
	const GEN_FLT x16 = lh_qj * lh_qj;
	const GEN_FLT x17 = x16 + x15;
	const GEN_FLT x18 = 2 * sqrt(x17 + (lh_qw * lh_qw) + x14);
	const GEN_FLT x19 = x13 * x18;
	const GEN_FLT x20 = x6 * sensor_z;
	const GEN_FLT x21 = obj_qw * obj_qk;
	const GEN_FLT x22 = obj_qj * obj_qi;
	const GEN_FLT x23 =
		((x22 + x21) * x12) + ((1 + (-1 * (x3 + x5) * x6)) * sensor_y) + obj_py + ((x8 + (-1 * x7)) * x20);
	const GEN_FLT x24 = lh_qw * lh_qk;
	const GEN_FLT x25 = lh_qj * lh_qi;
	const GEN_FLT x26 =
		((1 + (-1 * (x2 + x5) * x6)) * sensor_x) + ((x22 + (-1 * x21)) * x9) + ((x11 + x10) * x20) + obj_px;
	const GEN_FLT x27 = x26 * x18;
	const GEN_FLT x28 = (x23 * (1 + (-1 * (x14 + x15) * x18))) + ((x1 + (-1 * x0)) * x19) + ((x25 + x24) * x27) + lh_py;
	const GEN_FLT x29 = x23 * x18;
	const GEN_FLT x30 = lh_qw * lh_qj;
	const GEN_FLT x31 = lh_qk * lh_qi;
	const GEN_FLT x32 = ((x31 + (-1 * x30)) * x27) + ((x1 + x0) * x29) + (x13 * (1 + (-1 * (x14 + x16) * x18))) + lh_pz;
	const GEN_FLT x33 = -1 * x32;
	const GEN_FLT x34 = (x26 * (1 + (-1 * x18 * x17))) + ((x25 + (-1 * x24)) * x29) + ((x31 + x30) * x19) + lh_px;
	const GEN_FLT x35 =
		(-1 * phase_0) + (-1 * atan2(x34, x33)) + (-1 * asin((1. / sqrt((x34 * x34) + (x32 * x32))) * x28 * tilt_0));
	out[0] = x35 + (-1 * cos(1.5707963267949 + x35 + gibPhase_0) * gibMag_0) +
			 ((atan2(x28, x33) * atan2(x28, x33)) * curve_0);
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
	const GEN_FLT x3 = x2 + x0;
	const GEN_FLT x4 = sqrt(x3 + (lh_qw * lh_qw) + x1);
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * (x1 + x0) * x5);
	const GEN_FLT x7 = 1 + (-1 * (x2 + x1) * x5);
	const GEN_FLT x8 = obj_qj * obj_qj;
	const GEN_FLT x9 = obj_qi * obj_qi;
	const GEN_FLT x10 = x9 + x8;
	const GEN_FLT x11 = obj_qk * obj_qk;
	const GEN_FLT x12 = x9 + x11;
	const GEN_FLT x13 = sqrt(x12 + (obj_qw * obj_qw) + x8);
	const GEN_FLT x14 = 2 * x13;
	const GEN_FLT x15 = obj_qw * obj_qi;
	const GEN_FLT x16 = obj_qk * obj_qj;
	const GEN_FLT x17 = x16 + x15;
	const GEN_FLT x18 = x14 * sensor_y;
	const GEN_FLT x19 = obj_qw * obj_qj;
	const GEN_FLT x20 = obj_qk * obj_qi;
	const GEN_FLT x21 = x20 + (-1 * x19);
	const GEN_FLT x22 = x14 * sensor_x;
	const GEN_FLT x23 = (x22 * x21) + (x18 * x17) + ((1 + (-1 * x14 * x10)) * sensor_z) + obj_pz;
	const GEN_FLT x24 = x16 + (-1 * x15);
	const GEN_FLT x25 = x14 * sensor_z;
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x27 + x26;
	const GEN_FLT x29 = (x22 * x28) + ((1 + (-1 * x14 * x12)) * sensor_y) + obj_py + (x24 * x25);
	const GEN_FLT x30 = lh_qw * lh_qi;
	const GEN_FLT x31 = lh_qk * lh_qj;
	const GEN_FLT x32 = x31 + x30;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = x20 + x19;
	const GEN_FLT x35 = x27 + (-1 * x26);
	const GEN_FLT x36 = x8 + x11;
	const GEN_FLT x37 = ((1 + (-1 * x36 * x14)) * sensor_x) + (x35 * x18) + (x34 * x25) + obj_px;
	const GEN_FLT x38 = lh_qw * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qi;
	const GEN_FLT x40 = x39 + (-1 * x38);
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = (x41 * x37) + (x33 * x29) + (x7 * x23) + lh_pz;
	const GEN_FLT x43 = 1. / x42;
	const GEN_FLT x44 = x42 * x42;
	const GEN_FLT x45 = 1. / x44;
	const GEN_FLT x46 = x39 + x38;
	const GEN_FLT x47 = x5 * x23;
	const GEN_FLT x48 = lh_qw * lh_qk;
	const GEN_FLT x49 = lh_qj * lh_qi;
	const GEN_FLT x50 = x49 + (-1 * x48);
	const GEN_FLT x51 = x5 * x50;
	const GEN_FLT x52 = (x6 * x37) + (x51 * x29) + (x46 * x47) + lh_px;
	const GEN_FLT x53 = x52 * x45;
	const GEN_FLT x54 = x44 + (x52 * x52);
	const GEN_FLT x55 = 1. / x54;
	const GEN_FLT x56 = x55 * x44;
	const GEN_FLT x57 = x31 + (-1 * x30);
	const GEN_FLT x58 = 1 + (-1 * x3 * x5);
	const GEN_FLT x59 = (x49 + x48) * x5;
	const GEN_FLT x60 = (x58 * x29) + (x59 * x37) + (x57 * x47) + lh_py;
	const GEN_FLT x61 = x60 * x60;
	const GEN_FLT x62 = 1. / sqrt(1 + (-1 * x61 * x55 * (tilt_0 * tilt_0)));
	const GEN_FLT x63 = 2 * x52;
	const GEN_FLT x64 = 4 * x4;
	const GEN_FLT x65 = x64 * x42;
	const GEN_FLT x66 = 1.0 / 2.0 * x60 * (1. / (x54 * sqrt(x54))) * tilt_0;
	const GEN_FLT x67 = (1. / sqrt(x54)) * tilt_0;
	const GEN_FLT x68 = (-1 * x62 * ((x67 * x59) + (-1 * x66 * ((x65 * x40) + (x6 * x63))))) +
						(-1 * x56 * ((x53 * x41) + (-1 * x6 * x43)));
	const GEN_FLT x69 = -1 * x42;
	const GEN_FLT x70 =
		sin(1.5707963267949 + (-1 * atan2(x52, x69)) + (-1 * phase_0) + (-1 * asin(x60 * x67)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x71 = x60 * x45;
	const GEN_FLT x72 = 2 * (1. / (x44 + x61)) * x44 * atan2(x60, x69) * curve_0;
	const GEN_FLT x73 = x64 * x52;
	const GEN_FLT x74 = (-1 * x62 * ((x67 * x58) + (-1 * ((x65 * x32) + (x73 * x50)) * x66))) +
						(-1 * ((x53 * x33) + (-1 * x51 * x43)) * x56);
	const GEN_FLT x75 = x5 * x46;
	const GEN_FLT x76 = 2 * x42;
	const GEN_FLT x77 = x5 * x57;
	const GEN_FLT x78 = (-1 * x62 * ((x77 * x67) + (-1 * x66 * ((x7 * x76) + (x73 * x46))))) +
						(-1 * x56 * ((x7 * x53) + (-1 * x75 * x43)));
	const GEN_FLT x79 = 2 * (1. / x13);
	const GEN_FLT x80 = x79 * x36;
	const GEN_FLT x81 = x80 * sensor_x;
	const GEN_FLT x82 = x79 * sensor_y;
	const GEN_FLT x83 = x82 * x35;
	const GEN_FLT x84 = x14 * obj_qk;
	const GEN_FLT x85 = x84 * sensor_y;
	const GEN_FLT x86 = x34 * sensor_z;
	const GEN_FLT x87 = x86 * x79;
	const GEN_FLT x88 = x25 * obj_qj;
	const GEN_FLT x89 = (x87 * obj_qw) + (-1 * x85) + x88 + (x83 * obj_qw) + (-1 * x81 * obj_qw);
	const GEN_FLT x90 = x28 * sensor_x;
	const GEN_FLT x91 = x79 * x90;
	const GEN_FLT x92 = x79 * x12;
	const GEN_FLT x93 = x92 * sensor_y;
	const GEN_FLT x94 = x22 * obj_qk;
	const GEN_FLT x95 = x24 * sensor_z;
	const GEN_FLT x96 = x79 * x95;
	const GEN_FLT x97 = x14 * obj_qi;
	const GEN_FLT x98 = x97 * sensor_z;
	const GEN_FLT x99 = (-1 * x98) + (x96 * obj_qw) + x94 + (-1 * x93 * obj_qw) + (x91 * obj_qw);
	const GEN_FLT x100 = x22 * obj_qj;
	const GEN_FLT x101 = x21 * sensor_x;
	const GEN_FLT x102 = x79 * x101;
	const GEN_FLT x103 = x82 * x17;
	const GEN_FLT x104 = x97 * sensor_y;
	const GEN_FLT x105 = x79 * x10;
	const GEN_FLT x106 = x105 * sensor_z;
	const GEN_FLT x107 = x104 + (-1 * x106 * obj_qw) + (x103 * obj_qw) + (x102 * obj_qw) + (-1 * x100);
	const GEN_FLT x108 = x5 * x107;
	const GEN_FLT x109 = (x46 * x108) + (x51 * x99) + (x6 * x89);
	const GEN_FLT x110 = (x7 * x107) + (x99 * x33) + (x89 * x41);
	const GEN_FLT x111 = (x57 * x108) + (x58 * x99) + (x89 * x59);
	const GEN_FLT x112 = (-1 * x62 * ((x67 * x111) + (-1 * ((x76 * x110) + (x63 * x109)) * x66))) +
						 (-1 * ((x53 * x110) + (-1 * x43 * x109)) * x56);
	const GEN_FLT x113 = x79 * obj_qi;
	const GEN_FLT x114 = x113 * sensor_y;
	const GEN_FLT x115 = x18 * obj_qj;
	const GEN_FLT x116 = x84 * sensor_z;
	const GEN_FLT x117 = x116 + (x86 * x113) + x115 + (x35 * x114) + (-1 * x81 * obj_qi);
	const GEN_FLT x118 = 4 * x13;
	const GEN_FLT x119 = -1 * x118 * obj_qi;
	const GEN_FLT x120 = x14 * obj_qw;
	const GEN_FLT x121 = x120 * sensor_z;
	const GEN_FLT x122 = (-1 * x121) + (x95 * x113) + ((x119 + (-1 * x92 * obj_qi)) * sensor_y) + x100 + (x90 * x113);
	const GEN_FLT x123 = x120 * sensor_y;
	const GEN_FLT x124 = x123 + (x17 * x114) + ((x119 + (-1 * x105 * obj_qi)) * sensor_z) + x94 + (x101 * x113);
	const GEN_FLT x125 = (x75 * x124) + (x51 * x122) + (x6 * x117);
	const GEN_FLT x126 = (x7 * x124) + (x33 * x122) + (x41 * x117);
	const GEN_FLT x127 = (x77 * x124) + (x58 * x122) + (x59 * x117);
	const GEN_FLT x128 = (-1 * x62 * ((x67 * x127) + (-1 * ((x76 * x126) + (x63 * x125)) * x66))) +
						 (-1 * ((x53 * x126) + (-1 * x43 * x125)) * x56);
	const GEN_FLT x129 = -1 * x118 * obj_qj;
	const GEN_FLT x130 = x79 * obj_qj;
	const GEN_FLT x131 = (x86 * x130) + x104 + (x83 * obj_qj) + x121 + ((x129 + (-1 * x80 * obj_qj)) * sensor_x);
	const GEN_FLT x132 = x22 * obj_qi;
	const GEN_FLT x133 = x116 + (x96 * obj_qj) + (-1 * x93 * obj_qj) + x132 + (x91 * obj_qj);
	const GEN_FLT x134 = x22 * obj_qw;
	const GEN_FLT x135 =
		x85 + (x103 * obj_qj) + (-1 * x134) + ((x129 + (-1 * x105 * obj_qj)) * sensor_z) + (x101 * x130);
	const GEN_FLT x136 = (x75 * x135) + (x51 * x133) + (x6 * x131);
	const GEN_FLT x137 = (x7 * x135) + (x33 * x133) + (x41 * x131);
	const GEN_FLT x138 = (x77 * x135) + (x58 * x133) + (x59 * x131);
	const GEN_FLT x139 = (-1 * x62 * ((x67 * x138) + (-1 * ((x76 * x137) + (x63 * x136)) * x66))) +
						 (-1 * ((x53 * x137) + (-1 * x43 * x136)) * x56);
	const GEN_FLT x140 = -1 * x118 * obj_qk;
	const GEN_FLT x141 =
		x98 + (x87 * obj_qk) + ((x140 + (-1 * x80 * obj_qk)) * sensor_x) + (-1 * x123) + (x83 * obj_qk);
	const GEN_FLT x142 = (x96 * obj_qk) + ((x140 + (-1 * x92 * obj_qk)) * sensor_y) + x88 + x134 + (x91 * obj_qk);
	const GEN_FLT x143 = (-1 * x106 * obj_qk) + x115 + (x102 * obj_qk) + (x103 * obj_qk) + x132;
	const GEN_FLT x144 = x5 * x143;
	const GEN_FLT x145 = (x46 * x144) + (x51 * x142) + (x6 * x141);
	const GEN_FLT x146 = (x7 * x143) + (x33 * x142) + (x41 * x141);
	const GEN_FLT x147 = (x57 * x144) + (x58 * x142) + (x59 * x141);
	const GEN_FLT x148 = (-1 * x62 * ((x67 * x147) + (-1 * ((x76 * x146) + (x63 * x145)) * x66))) +
						 (-1 * ((x53 * x146) + (-1 * x43 * x145)) * x56);
	out[0] = x68 + (((x71 * x41) + (-1 * x59 * x43)) * x72) + (x70 * x68);
	out[1] = x74 + (((x71 * x33) + (-1 * x58 * x43)) * x72) + (x70 * x74);
	out[2] = x78 + (x72 * ((x7 * x71) + (-1 * x77 * x43))) + (x70 * x78);
	out[3] = x112 + (((x71 * x110) + (-1 * x43 * x111)) * x72) + (x70 * x112);
	out[4] = x128 + (((x71 * x126) + (-1 * x43 * x127)) * x72) + (x70 * x128);
	out[5] = x139 + (((x71 * x137) + (-1 * x43 * x138)) * x72) + (x70 * x139);
	out[6] = x148 + (((x71 * x146) + (-1 * x43 * x147)) * x72) + (x70 * x148);
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
	const GEN_FLT x2 = x1 + x0;
	const GEN_FLT x3 = lh_qk * lh_qk;
	const GEN_FLT x4 = sqrt(x2 + (lh_qw * lh_qw) + x3);
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * x2 * x5);
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = obj_qk * obj_qk;
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = sqrt(x10 + (obj_qw * obj_qw) + x7);
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 + (-1 * (x8 + x7) * x12);
	const GEN_FLT x14 = obj_qw * obj_qi;
	const GEN_FLT x15 = obj_qk * obj_qj;
	const GEN_FLT x16 = x15 + x14;
	const GEN_FLT x17 = x12 * sensor_y;
	const GEN_FLT x18 = obj_qw * obj_qj;
	const GEN_FLT x19 = obj_qk * obj_qi;
	const GEN_FLT x20 = x19 + (-1 * x18);
	const GEN_FLT x21 = x12 * sensor_x;
	const GEN_FLT x22 = (x20 * x21) + (x17 * x16) + (x13 * sensor_z) + obj_pz;
	const GEN_FLT x23 = lh_qw * lh_qi;
	const GEN_FLT x24 = lh_qk * lh_qj;
	const GEN_FLT x25 = x24 + x23;
	const GEN_FLT x26 = x15 + (-1 * x14);
	const GEN_FLT x27 = x12 * sensor_z;
	const GEN_FLT x28 = 1 + (-1 * x12 * x10);
	const GEN_FLT x29 = obj_qw * obj_qk;
	const GEN_FLT x30 = obj_qj * obj_qi;
	const GEN_FLT x31 = x30 + x29;
	const GEN_FLT x32 = (x31 * x21) + (x28 * sensor_y) + obj_py + (x26 * x27);
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = lh_qw * lh_qj;
	const GEN_FLT x35 = lh_qk * lh_qi;
	const GEN_FLT x36 = x35 + (-1 * x34);
	const GEN_FLT x37 = x19 + x18;
	const GEN_FLT x38 = x30 + (-1 * x29);
	const GEN_FLT x39 = 1 + (-1 * (x7 + x9) * x12);
	const GEN_FLT x40 = (x39 * sensor_x) + (x38 * x17) + (x37 * x27) + obj_px;
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = (x33 * x25) + (x41 * x36) + (x6 * x22) + lh_pz;
	const GEN_FLT x43 = 1. / x42;
	const GEN_FLT x44 = 1 + (-1 * (x0 + x3) * x5);
	const GEN_FLT x45 = lh_qw * lh_qk;
	const GEN_FLT x46 = lh_qj * lh_qi;
	const GEN_FLT x47 = x46 + (-1 * x45);
	const GEN_FLT x48 = 4 * x4 * x11;
	const GEN_FLT x49 = x48 * x31;
	const GEN_FLT x50 = x35 + x34;
	const GEN_FLT x51 = x48 * x20;
	const GEN_FLT x52 = (x50 * x51) + (x47 * x49) + (x44 * x39);
	const GEN_FLT x53 = x5 * x39;
	const GEN_FLT x54 = x6 * x12;
	const GEN_FLT x55 = (x54 * x20) + (x49 * x25) + (x53 * x36);
	const GEN_FLT x56 = x5 * x22;
	const GEN_FLT x57 = (x40 * x44) + (x47 * x33) + (x50 * x56) + lh_px;
	const GEN_FLT x58 = x42 * x42;
	const GEN_FLT x59 = 1. / x58;
	const GEN_FLT x60 = x57 * x59;
	const GEN_FLT x61 = x58 + (x57 * x57);
	const GEN_FLT x62 = 1. / x61;
	const GEN_FLT x63 = x62 * x58;
	const GEN_FLT x64 = x24 + (-1 * x23);
	const GEN_FLT x65 = 1 + (-1 * (x1 + x3) * x5);
	const GEN_FLT x66 = x46 + x45;
	const GEN_FLT x67 = (x66 * x41) + (x65 * x32) + (x64 * x56) + lh_py;
	const GEN_FLT x68 = x67 * x67;
	const GEN_FLT x69 = 1. / sqrt(1 + (-1 * x62 * x68 * (tilt_0 * tilt_0)));
	const GEN_FLT x70 = 2 * x57;
	const GEN_FLT x71 = 2 * x42;
	const GEN_FLT x72 = 1.0 / 2.0 * (1. / (x61 * sqrt(x61))) * x67 * tilt_0;
	const GEN_FLT x73 = x65 * x12;
	const GEN_FLT x74 = (x64 * x51) + (x73 * x31) + (x66 * x53);
	const GEN_FLT x75 = (1. / sqrt(x61)) * tilt_0;
	const GEN_FLT x76 = (-1 * x69 * ((x75 * x74) + (-1 * ((x71 * x55) + (x70 * x52)) * x72))) +
						(-1 * ((x60 * x55) + (-1 * x52 * x43)) * x63);
	const GEN_FLT x77 = -1 * x42;
	const GEN_FLT x78 =
		sin(1.5707963267949 + (-1 * atan2(x57, x77)) + (-1 * phase_0) + (-1 * asin(x75 * x67)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x79 = x67 * x59;
	const GEN_FLT x80 = 2 * (1. / (x58 + x68)) * x58 * atan2(x67, x77) * curve_0;
	const GEN_FLT x81 = x5 * x28;
	const GEN_FLT x82 = x44 * x12;
	const GEN_FLT x83 = x48 * x16;
	const GEN_FLT x84 = (x83 * x50) + (x82 * x38) + (x81 * x47);
	const GEN_FLT x85 = x48 * x38;
	const GEN_FLT x86 = (x54 * x16) + (x81 * x25) + (x85 * x36);
	const GEN_FLT x87 = (x83 * x64) + (x65 * x28) + (x85 * x66);
	const GEN_FLT x88 = (-1 * x69 * ((x87 * x75) + (-1 * ((x86 * x71) + (x84 * x70)) * x72))) +
						(-1 * ((x86 * x60) + (-1 * x84 * x43)) * x63);
	const GEN_FLT x89 = x48 * x26;
	const GEN_FLT x90 = x5 * x13;
	const GEN_FLT x91 = (x50 * x90) + (x89 * x47) + (x82 * x37);
	const GEN_FLT x92 = x48 * x37;
	const GEN_FLT x93 = (x6 * x13) + (x89 * x25) + (x92 * x36);
	const GEN_FLT x94 = (x64 * x90) + (x73 * x26) + (x66 * x92);
	const GEN_FLT x95 = (-1 * x69 * ((x75 * x94) + (-1 * ((x71 * x93) + (x70 * x91)) * x72))) +
						(-1 * ((x60 * x93) + (-1 * x91 * x43)) * x63);
	out[0] = x76 + (((x79 * x55) + (-1 * x74 * x43)) * x80) + (x78 * x76);
	out[1] = x88 + (((x86 * x79) + (-1 * x87 * x43)) * x80) + (x88 * x78);
	out[2] = x95 + (((x79 * x93) + (-1 * x94 * x43)) * x80) + (x78 * x95);
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
	const GEN_FLT x2 = x1 + x0;
	const GEN_FLT x3 = lh_qk * lh_qk;
	const GEN_FLT x4 = x1 + x3;
	const GEN_FLT x5 = sqrt(x4 + (lh_qw * lh_qw) + x0);
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = obj_qk * obj_qk;
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = 2 * sqrt(x10 + (obj_qw * obj_qw) + x7);
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 =
		((x16 + (-1 * x15)) * x17) + ((x13 + x12) * x14) + ((1 + (-1 * (x8 + x7) * x11)) * sensor_z) + obj_pz;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x20 + x19;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = ((x24 + x23) * x17) + ((1 + (-1 * x11 * x10)) * sensor_y) + obj_py + ((x13 + (-1 * x12)) * x22);
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = x28 + (-1 * x27);
	const GEN_FLT x30 =
		((1 + (-1 * (x7 + x9) * x11)) * sensor_x) + ((x24 + (-1 * x23)) * x14) + ((x16 + x15) * x22) + obj_px;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = (x31 * x29) + (x21 * x26) + (x18 * (1 + (-1 * x2 * x6))) + lh_pz;
	const GEN_FLT x33 = x28 + x27;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = x36 + (-1 * x35);
	const GEN_FLT x38 = x0 + x3;
	const GEN_FLT x39 = (x30 * (1 + (-1 * x6 * x38))) + (x37 * x26) + (x34 * x33) + lh_px;
	const GEN_FLT x40 = x32 * x32;
	const GEN_FLT x41 = x40 + (x39 * x39);
	const GEN_FLT x42 = 1. / x41;
	const GEN_FLT x43 = x20 + (-1 * x19);
	const GEN_FLT x44 = x36 + x35;
	const GEN_FLT x45 = (x44 * x31) + (x25 * (1 + (-1 * x4 * x6))) + (x43 * x34) + lh_py;
	const GEN_FLT x46 = x45 * x45;
	const GEN_FLT x47 = 1. / sqrt(1 + (-1 * x42 * x46 * (tilt_0 * tilt_0)));
	const GEN_FLT x48 = (1. / (x41 * sqrt(x41))) * x45 * tilt_0;
	const GEN_FLT x49 = x47 * x48;
	const GEN_FLT x50 = (x49 * x39) + (x42 * x32);
	const GEN_FLT x51 = (1. / sqrt(x41)) * tilt_0;
	const GEN_FLT x52 = -1 * x32;
	const GEN_FLT x53 =
		sin(1.5707963267949 + (-1 * atan2(x39, x52)) + (-1 * phase_0) + (-1 * asin(x51 * x45)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x54 = x51 * x47;
	const GEN_FLT x55 = 2 * x32;
	const GEN_FLT x56 = (1. / (x40 + x46)) * atan2(x45, x52) * curve_0;
	const GEN_FLT x57 = (x49 * x32) + (-1 * x42 * x39);
	const GEN_FLT x58 = 2 * x56;
	const GEN_FLT x59 = 1. / x32;
	const GEN_FLT x60 = 2 * (1. / x5);
	const GEN_FLT x61 = x60 * x38;
	const GEN_FLT x62 = x61 * x30;
	const GEN_FLT x63 = x60 * lh_qw;
	const GEN_FLT x64 = x37 * x25;
	const GEN_FLT x65 = x26 * lh_qk;
	const GEN_FLT x66 = x60 * x18;
	const GEN_FLT x67 = x66 * x33;
	const GEN_FLT x68 = x34 * lh_qj;
	const GEN_FLT x69 = x68 + (x67 * lh_qw) + (-1 * x65) + (x63 * x64) + (-1 * x62 * lh_qw);
	const GEN_FLT x70 = x30 * x29;
	const GEN_FLT x71 = x25 * x21;
	const GEN_FLT x72 = x26 * lh_qi;
	const GEN_FLT x73 = x31 * lh_qj;
	const GEN_FLT x74 = x2 * x60;
	const GEN_FLT x75 = x74 * x18;
	const GEN_FLT x76 = (-1 * x75 * lh_qw) + (-1 * x73) + x72 + (x71 * x63) + (x70 * x63);
	const GEN_FLT x77 = 1. / x40;
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
	const GEN_FLT x88 = (-1 * x87) + (x86 * lh_qw) + (-1 * x85 * lh_qw) + x83 + (x82 * x63);
	const GEN_FLT x89 = (-1 * x47 * ((x88 * x51) + (-1 * ((x76 * x55) + (x80 * x69)) * x81))) +
						(-1 * ((x78 * x76) + (-1 * x69 * x59)) * x79);
	const GEN_FLT x90 = x77 * x45;
	const GEN_FLT x91 = x58 * x40;
	const GEN_FLT x92 = x60 * lh_qi;
	const GEN_FLT x93 = x26 * lh_qj;
	const GEN_FLT x94 = x34 * lh_qk;
	const GEN_FLT x95 = x94 + (x67 * lh_qi) + (x64 * x92) + x93 + (-1 * x62 * lh_qi);
	const GEN_FLT x96 = x26 * lh_qw;
	const GEN_FLT x97 = 4 * x5;
	const GEN_FLT x98 = -1 * x97 * lh_qi;
	const GEN_FLT x99 = (x18 * (x98 + (-1 * x74 * lh_qi))) + (x71 * x92) + x96 + x83 + (x70 * x92);
	const GEN_FLT x100 = x34 * lh_qw;
	const GEN_FLT x101 = (-1 * x100) + (x25 * (x98 + (-1 * x84 * lh_qi))) + (x86 * lh_qi) + x73 + (x82 * x92);
	const GEN_FLT x102 = (-1 * x47 * ((x51 * x101) + (-1 * ((x55 * x99) + (x80 * x95)) * x81))) +
						 (-1 * ((x78 * x99) + (-1 * x59 * x95)) * x79);
	const GEN_FLT x103 = -1 * x97 * lh_qj;
	const GEN_FLT x104 = x60 * lh_qj;
	const GEN_FLT x105 = (x67 * lh_qj) + x72 + (x64 * x104) + x100 + (x30 * (x103 + (-1 * x61 * lh_qj)));
	const GEN_FLT x106 = x31 * lh_qw;
	const GEN_FLT x107 = (x18 * (x103 + (-1 * x74 * lh_qj))) + (x71 * x104) + x65 + (-1 * x106) + (x70 * x104);
	const GEN_FLT x108 = x31 * lh_qi;
	const GEN_FLT x109 = (x86 * lh_qj) + (-1 * x85 * lh_qj) + x108 + x94 + (x82 * x104);
	const GEN_FLT x110 = (-1 * x47 * ((x51 * x109) + (-1 * ((x55 * x107) + (x80 * x105)) * x81))) +
						 (-1 * ((x78 * x107) + (-1 * x59 * x105)) * x79);
	const GEN_FLT x111 = -1 * x97 * lh_qk;
	const GEN_FLT x112 = x60 * lh_qk;
	const GEN_FLT x113 = x25 * x112;
	const GEN_FLT x114 = x87 + (x67 * lh_qk) + (x37 * x113) + (-1 * x96) + (x30 * (x111 + (-1 * x61 * lh_qk)));
	const GEN_FLT x115 = (-1 * x75 * lh_qk) + x108 + x93 + (x21 * x113) + (x70 * x112);
	const GEN_FLT x116 = x68 + (x86 * lh_qk) + (x82 * x112) + (x25 * (x111 + (-1 * x84 * lh_qk))) + x106;
	const GEN_FLT x117 = (-1 * x47 * ((x51 * x116) + (-1 * ((x55 * x115) + (x80 * x114)) * x81))) +
						 (-1 * ((x78 * x115) + (-1 * x59 * x114)) * x79);
	out[0] = x50 + (x50 * x53);
	out[1] = (-1 * x56 * x55) + (-1 * x54 * x53) + (-1 * x54);
	out[2] = x57 + (x58 * x45) + (x53 * x57);
	out[3] = x89 + (((x76 * x90) + (-1 * x88 * x59)) * x91) + (x89 * x53);
	out[4] = x102 + (x91 * ((x90 * x99) + (-1 * x59 * x101))) + (x53 * x102);
	out[5] = x110 + (((x90 * x107) + (-1 * x59 * x109)) * x91) + (x53 * x110);
	out[6] = x117 + (((x90 * x115) + (-1 * x59 * x116)) * x91) + (x53 * x117);
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
	const GEN_FLT x4 = x3 + x2;
	const GEN_FLT x5 = obj_qk * obj_qk;
	const GEN_FLT x6 = 2 * sqrt(x4 + (obj_qw * obj_qw) + x5);
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x6 * sensor_y;
	const GEN_FLT x10 = obj_qw * obj_qj;
	const GEN_FLT x11 = obj_qk * obj_qi;
	const GEN_FLT x12 = x6 * sensor_x;
	const GEN_FLT x13 = ((x11 + (-1 * x10)) * x12) + ((x8 + x7) * x9) + ((1 + (-1 * x4 * x6)) * sensor_z) + obj_pz;
	const GEN_FLT x14 = lh_qi * lh_qi;
	const GEN_FLT x15 = lh_qk * lh_qk;
	const GEN_FLT x16 = lh_qj * lh_qj;
	const GEN_FLT x17 = x16 + x15;
	const GEN_FLT x18 = 2 * sqrt(x17 + (lh_qw * lh_qw) + x14);
	const GEN_FLT x19 = x13 * x18;
	const GEN_FLT x20 = x6 * sensor_z;
	const GEN_FLT x21 = obj_qw * obj_qk;
	const GEN_FLT x22 = obj_qj * obj_qi;
	const GEN_FLT x23 =
		((x22 + x21) * x12) + ((1 + (-1 * (x3 + x5) * x6)) * sensor_y) + obj_py + ((x8 + (-1 * x7)) * x20);
	const GEN_FLT x24 = lh_qw * lh_qk;
	const GEN_FLT x25 = lh_qj * lh_qi;
	const GEN_FLT x26 =
		((1 + (-1 * (x2 + x5) * x6)) * sensor_x) + ((x22 + (-1 * x21)) * x9) + ((x11 + x10) * x20) + obj_px;
	const GEN_FLT x27 = x26 * x18;
	const GEN_FLT x28 = (x23 * (1 + (-1 * (x14 + x15) * x18))) + ((x1 + (-1 * x0)) * x19) + ((x25 + x24) * x27) + lh_py;
	const GEN_FLT x29 = x23 * x18;
	const GEN_FLT x30 = lh_qw * lh_qj;
	const GEN_FLT x31 = lh_qk * lh_qi;
	const GEN_FLT x32 = ((x31 + (-1 * x30)) * x27) + ((x1 + x0) * x29) + (x13 * (1 + (-1 * (x14 + x16) * x18))) + lh_pz;
	const GEN_FLT x33 = (x26 * (1 + (-1 * x18 * x17))) + ((x25 + (-1 * x24)) * x29) + ((x31 + x30) * x19) + lh_px;
	const GEN_FLT x34 = (x33 * x33) + (x32 * x32);
	const GEN_FLT x35 = (1. / sqrt(x34)) * x28;
	const GEN_FLT x36 = -1 * x32;
	const GEN_FLT x37 =
		1.5707963267949 + (-1 * atan2(x33, x36)) + (-1 * phase_0) + (-1 * asin(x35 * tilt_0)) + gibPhase_0;
	const GEN_FLT x38 = sin(x37) * gibMag_0;
	const GEN_FLT x39 = x35 * (1. / sqrt(1 + (-1 * (1. / x34) * (x28 * x28) * (tilt_0 * tilt_0))));
	out[0] = -1 + (-1 * x38);
	out[1] = (-1 * x39) + (-1 * x38 * x39);
	out[2] = atan2(x28, x36) * atan2(x28, x36);
	out[3] = x38;
	out[4] = -1 * cos(x37);
	out[5] = 0;
	out[6] = 0;
}

/** Applying function <function reproject_axis_y at 0x7f6253d38320> */
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
	const GEN_FLT x3 = x1 + x2;
	const GEN_FLT x4 = 2 * sqrt(x3 + (lh_qw * lh_qw) + x0);
	const GEN_FLT x5 = obj_qj * obj_qj;
	const GEN_FLT x6 = obj_qi * obj_qi;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = x6 + x7;
	const GEN_FLT x9 = 2 * sqrt(x8 + (obj_qw * obj_qw) + x5);
	const GEN_FLT x10 = obj_qw * obj_qi;
	const GEN_FLT x11 = obj_qk * obj_qj;
	const GEN_FLT x12 = x9 * sensor_y;
	const GEN_FLT x13 = obj_qw * obj_qj;
	const GEN_FLT x14 = obj_qk * obj_qi;
	const GEN_FLT x15 = x9 * sensor_x;
	const GEN_FLT x16 =
		((x14 + (-1 * x13)) * x15) + ((x11 + x10) * x12) + ((1 + (-1 * (x6 + x5) * x9)) * sensor_z) + obj_pz;
	const GEN_FLT x17 = lh_qw * lh_qi;
	const GEN_FLT x18 = lh_qk * lh_qj;
	const GEN_FLT x19 = x9 * sensor_z;
	const GEN_FLT x20 = obj_qw * obj_qk;
	const GEN_FLT x21 = obj_qj * obj_qi;
	const GEN_FLT x22 = ((x21 + x20) * x15) + ((1 + (-1 * x8 * x9)) * sensor_y) + obj_py + ((x11 + (-1 * x10)) * x19);
	const GEN_FLT x23 = x4 * x22;
	const GEN_FLT x24 = lh_qw * lh_qj;
	const GEN_FLT x25 = lh_qk * lh_qi;
	const GEN_FLT x26 =
		((1 + (-1 * (x5 + x7) * x9)) * sensor_x) + ((x21 + (-1 * x20)) * x12) + ((x14 + x13) * x19) + obj_px;
	const GEN_FLT x27 = x4 * x26;
	const GEN_FLT x28 = ((x25 + (-1 * x24)) * x27) + (x16 * (1 + (-1 * (x1 + x0) * x4))) + ((x18 + x17) * x23) + lh_pz;
	const GEN_FLT x29 = x4 * x16;
	const GEN_FLT x30 = lh_qw * lh_qk;
	const GEN_FLT x31 = lh_qj * lh_qi;
	const GEN_FLT x32 = ((x31 + x30) * x27) + (x22 * (1 + (-1 * x4 * x3))) + ((x18 + (-1 * x17)) * x29) + lh_py;
	const GEN_FLT x33 = (x26 * (1 + (-1 * (x0 + x2) * x4))) + ((x31 + (-1 * x30)) * x23) + ((x25 + x24) * x29) + lh_px;
	const GEN_FLT x34 = -1 * x28;
	const GEN_FLT x35 = (-1 * atan2(-1 * x32, x34)) + (-1 * phase_1) +
						(-1 * asin((1. / sqrt((x32 * x32) + (x28 * x28))) * x33 * tilt_1));
	out[0] = x35 + ((atan2(x33, x34) * atan2(x33, x34)) * curve_1) +
			 (-1 * cos(1.5707963267949 + x35 + gibPhase_1) * gibMag_1);
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
	const GEN_FLT x3 = x2 + x0;
	const GEN_FLT x4 = sqrt(x3 + (lh_qw * lh_qw) + x1);
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * (x1 + x0) * x5);
	const GEN_FLT x7 = 1 + (-1 * (x2 + x1) * x5);
	const GEN_FLT x8 = obj_qj * obj_qj;
	const GEN_FLT x9 = obj_qi * obj_qi;
	const GEN_FLT x10 = x9 + x8;
	const GEN_FLT x11 = obj_qk * obj_qk;
	const GEN_FLT x12 = x9 + x11;
	const GEN_FLT x13 = sqrt(x12 + (obj_qw * obj_qw) + x8);
	const GEN_FLT x14 = 2 * x13;
	const GEN_FLT x15 = obj_qw * obj_qi;
	const GEN_FLT x16 = obj_qk * obj_qj;
	const GEN_FLT x17 = x16 + x15;
	const GEN_FLT x18 = x14 * sensor_y;
	const GEN_FLT x19 = obj_qw * obj_qj;
	const GEN_FLT x20 = obj_qk * obj_qi;
	const GEN_FLT x21 = x20 + (-1 * x19);
	const GEN_FLT x22 = x14 * sensor_x;
	const GEN_FLT x23 = (x22 * x21) + (x18 * x17) + ((1 + (-1 * x14 * x10)) * sensor_z) + obj_pz;
	const GEN_FLT x24 = x16 + (-1 * x15);
	const GEN_FLT x25 = x14 * sensor_z;
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x27 + x26;
	const GEN_FLT x29 = (x22 * x28) + ((1 + (-1 * x14 * x12)) * sensor_y) + obj_py + (x24 * x25);
	const GEN_FLT x30 = lh_qw * lh_qi;
	const GEN_FLT x31 = lh_qk * lh_qj;
	const GEN_FLT x32 = x31 + x30;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = x20 + x19;
	const GEN_FLT x35 = x27 + (-1 * x26);
	const GEN_FLT x36 = x8 + x11;
	const GEN_FLT x37 = ((1 + (-1 * x36 * x14)) * sensor_x) + (x35 * x18) + (x34 * x25) + obj_px;
	const GEN_FLT x38 = lh_qw * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qi;
	const GEN_FLT x40 = x39 + (-1 * x38);
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = (x41 * x37) + (x33 * x29) + (x7 * x23) + lh_pz;
	const GEN_FLT x43 = 1. / x42;
	const GEN_FLT x44 = x42 * x42;
	const GEN_FLT x45 = 1. / x44;
	const GEN_FLT x46 = x39 + x38;
	const GEN_FLT x47 = x5 * x23;
	const GEN_FLT x48 = lh_qw * lh_qk;
	const GEN_FLT x49 = lh_qj * lh_qi;
	const GEN_FLT x50 = (x49 + (-1 * x48)) * x5;
	const GEN_FLT x51 = (x6 * x37) + (x50 * x29) + (x46 * x47) + lh_px;
	const GEN_FLT x52 = x51 * x45;
	const GEN_FLT x53 = -1 * x42;
	const GEN_FLT x54 = x51 * x51;
	const GEN_FLT x55 = 2 * (1. / (x44 + x54)) * x44 * atan2(x51, x53) * curve_1;
	const GEN_FLT x56 = x49 + x48;
	const GEN_FLT x57 = x5 * x56;
	const GEN_FLT x58 = x31 + (-1 * x30);
	const GEN_FLT x59 = 1 + (-1 * x3 * x5);
	const GEN_FLT x60 = (x57 * x37) + (x59 * x29) + (x58 * x47) + lh_py;
	const GEN_FLT x61 = 2 * x60;
	const GEN_FLT x62 = x4 * x61 * x45;
	const GEN_FLT x63 = x44 + (x60 * x60);
	const GEN_FLT x64 = 1. / x63;
	const GEN_FLT x65 = x64 * x44;
	const GEN_FLT x66 = 1. / sqrt(1 + (-1 * x64 * x54 * (tilt_1 * tilt_1)));
	const GEN_FLT x67 = (1. / sqrt(x63)) * tilt_1;
	const GEN_FLT x68 = 4 * x4;
	const GEN_FLT x69 = x60 * x68;
	const GEN_FLT x70 = x68 * x42;
	const GEN_FLT x71 = 1.0 / 2.0 * (1. / (x63 * sqrt(x63))) * x51 * tilt_1;
	const GEN_FLT x72 = (-1 * x66 * ((-1 * ((x70 * x40) + (x69 * x56)) * x71) + (x6 * x67))) +
						(-1 * ((-1 * x62 * x40) + (x57 * x43)) * x65);
	const GEN_FLT x73 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x60, x53)) + (-1 * phase_1) + (-1 * asin(x67 * x51)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x74 = (-1 * x66 * ((-1 * ((x70 * x32) + (x61 * x59)) * x71) + (x67 * x50))) +
						(-1 * ((-1 * x62 * x32) + (x59 * x43)) * x65);
	const GEN_FLT x75 = x5 * x43;
	const GEN_FLT x76 = x7 * x45;
	const GEN_FLT x77 = x5 * x46;
	const GEN_FLT x78 = 2 * x42;
	const GEN_FLT x79 = (-1 * x66 * ((-1 * x71 * ((x7 * x78) + (x69 * x58))) + (x77 * x67))) +
						(-1 * ((-1 * x76 * x60) + (x75 * x58)) * x65);
	const GEN_FLT x80 = 2 * (1. / x13);
	const GEN_FLT x81 = x80 * x36;
	const GEN_FLT x82 = x81 * sensor_x;
	const GEN_FLT x83 = x35 * sensor_y;
	const GEN_FLT x84 = x80 * x83;
	const GEN_FLT x85 = x18 * obj_qk;
	const GEN_FLT x86 = x80 * obj_qw;
	const GEN_FLT x87 = x34 * sensor_z;
	const GEN_FLT x88 = x25 * obj_qj;
	const GEN_FLT x89 = (-1 * x85) + (x84 * obj_qw) + x88 + (x86 * x87) + (-1 * x82 * obj_qw);
	const GEN_FLT x90 = x28 * sensor_x;
	const GEN_FLT x91 = x80 * x12;
	const GEN_FLT x92 = x91 * sensor_y;
	const GEN_FLT x93 = x22 * obj_qk;
	const GEN_FLT x94 = x24 * sensor_z;
	const GEN_FLT x95 = x25 * obj_qi;
	const GEN_FLT x96 = x93 + (-1 * x95) + (x86 * x94) + (-1 * x92 * obj_qw) + (x86 * x90);
	const GEN_FLT x97 = x22 * obj_qj;
	const GEN_FLT x98 = x21 * sensor_x;
	const GEN_FLT x99 = x17 * sensor_y;
	const GEN_FLT x100 = x80 * x99;
	const GEN_FLT x101 = x18 * obj_qi;
	const GEN_FLT x102 = x80 * x10;
	const GEN_FLT x103 = x102 * sensor_z;
	const GEN_FLT x104 = (-1 * x103 * obj_qw) + x101 + (x100 * obj_qw) + (x86 * x98) + (-1 * x97);
	const GEN_FLT x105 = x5 * x104;
	const GEN_FLT x106 = (x46 * x105) + (x50 * x96) + (x6 * x89);
	const GEN_FLT x107 = (x7 * x104) + (x96 * x33) + (x89 * x41);
	const GEN_FLT x108 = (x58 * x105) + (x59 * x96) + (x89 * x57);
	const GEN_FLT x109 = x60 * x45;
	const GEN_FLT x110 = (-1 * x66 * ((-1 * ((x78 * x107) + (x61 * x108)) * x71) + (x67 * x106))) +
						 (-1 * x65 * ((-1 * x109 * x107) + (x43 * x108)));
	const GEN_FLT x111 = x80 * obj_qi;
	const GEN_FLT x112 = x18 * obj_qj;
	const GEN_FLT x113 = x111 * sensor_z;
	const GEN_FLT x114 = x25 * obj_qk;
	const GEN_FLT x115 = (x34 * x113) + x112 + (x83 * x111) + x114 + (-1 * x82 * obj_qi);
	const GEN_FLT x116 = x111 * sensor_x;
	const GEN_FLT x117 = 4 * x13;
	const GEN_FLT x118 = -1 * x117 * obj_qi;
	const GEN_FLT x119 = x25 * obj_qw;
	const GEN_FLT x120 = (x24 * x113) + ((x118 + (-1 * x91 * obj_qi)) * sensor_y) + (-1 * x119) + x97 + (x28 * x116);
	const GEN_FLT x121 = x18 * obj_qw;
	const GEN_FLT x122 = x121 + (x99 * x111) + x93 + ((x118 + (-1 * x102 * obj_qi)) * sensor_z) + (x21 * x116);
	const GEN_FLT x123 = (x77 * x122) + (x50 * x120) + (x6 * x115);
	const GEN_FLT x124 = (x7 * x122) + (x33 * x120) + (x41 * x115);
	const GEN_FLT x125 = x5 * x58;
	const GEN_FLT x126 = (x122 * x125) + (x59 * x120) + (x57 * x115);
	const GEN_FLT x127 = (-1 * x66 * ((-1 * ((x78 * x124) + (x61 * x126)) * x71) + (x67 * x123))) +
						 (-1 * x65 * ((-1 * x109 * x124) + (x43 * x126)));
	const GEN_FLT x128 = -1 * x117 * obj_qj;
	const GEN_FLT x129 = x80 * obj_qj;
	const GEN_FLT x130 = x119 + (x87 * x129) + x101 + (x84 * obj_qj) + ((x128 + (-1 * x81 * obj_qj)) * sensor_x);
	const GEN_FLT x131 = x22 * obj_qi;
	const GEN_FLT x132 = x114 + (x94 * x129) + (-1 * x92 * obj_qj) + x131 + (x90 * x129);
	const GEN_FLT x133 = x22 * obj_qw;
	const GEN_FLT x134 =
		((x128 + (-1 * x102 * obj_qj)) * sensor_z) + x85 + (x100 * obj_qj) + (-1 * x133) + (x98 * x129);
	const GEN_FLT x135 = (x77 * x134) + (x50 * x132) + (x6 * x130);
	const GEN_FLT x136 = (x7 * x134) + (x33 * x132) + (x41 * x130);
	const GEN_FLT x137 = x45 * x136;
	const GEN_FLT x138 = (x125 * x134) + (x59 * x132) + (x57 * x130);
	const GEN_FLT x139 = (-1 * x66 * ((-1 * ((x78 * x136) + (x61 * x138)) * x71) + (x67 * x135))) +
						 (-1 * ((-1 * x60 * x137) + (x43 * x138)) * x65);
	const GEN_FLT x140 = -1 * x117 * obj_qk;
	const GEN_FLT x141 = x80 * obj_qk;
	const GEN_FLT x142 = (x87 * x141) + ((x140 + (-1 * x81 * obj_qk)) * sensor_x) + x95 + (-1 * x121) + (x84 * obj_qk);
	const GEN_FLT x143 = x88 + (x94 * x141) + ((x140 + (-1 * x91 * obj_qk)) * sensor_y) + x133 + (x90 * x141);
	const GEN_FLT x144 = (-1 * x103 * obj_qk) + x112 + (x98 * x141) + (x100 * obj_qk) + x131;
	const GEN_FLT x145 = x5 * x144;
	const GEN_FLT x146 = (x46 * x145) + (x50 * x143) + (x6 * x142);
	const GEN_FLT x147 = (x7 * x144) + (x33 * x143) + (x41 * x142);
	const GEN_FLT x148 = x45 * x147;
	const GEN_FLT x149 = (x58 * x145) + (x59 * x143) + (x57 * x142);
	const GEN_FLT x150 = (-1 * x66 * ((-1 * ((x78 * x147) + (x61 * x149)) * x71) + (x67 * x146))) +
						 (-1 * ((-1 * x60 * x148) + (x43 * x149)) * x65);
	out[0] = x72 + (x73 * x72) + (x55 * ((x52 * x41) + (-1 * x6 * x43)));
	out[1] = x74 + (x73 * x74) + (((x52 * x33) + (-1 * x50 * x43)) * x55);
	out[2] = x79 + (x73 * x79) + (((x76 * x51) + (-1 * x75 * x46)) * x55);
	out[3] = (x73 * x110) + x110 + (((x52 * x107) + (-1 * x43 * x106)) * x55);
	out[4] = x127 + (x73 * x127) + (((x52 * x124) + (-1 * x43 * x123)) * x55);
	out[5] = x139 + (x73 * x139) + (((x51 * x137) + (-1 * x43 * x135)) * x55);
	out[6] = x150 + (x73 * x150) + (((x51 * x148) + (-1 * x43 * x146)) * x55);
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
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt(x3 + (lh_qw * lh_qw) + x1);
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 + (-1 * (x1 + x0) * x5);
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = x8 + x7;
	const GEN_FLT x10 = obj_qk * obj_qk;
	const GEN_FLT x11 = sqrt(x9 + (obj_qw * obj_qw) + x10);
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 + (-1 * x9 * x12);
	const GEN_FLT x14 = obj_qw * obj_qi;
	const GEN_FLT x15 = obj_qk * obj_qj;
	const GEN_FLT x16 = x15 + x14;
	const GEN_FLT x17 = x12 * sensor_y;
	const GEN_FLT x18 = obj_qw * obj_qj;
	const GEN_FLT x19 = obj_qk * obj_qi;
	const GEN_FLT x20 = x19 + (-1 * x18);
	const GEN_FLT x21 = x12 * sensor_x;
	const GEN_FLT x22 = (x20 * x21) + (x17 * x16) + (x13 * sensor_z) + obj_pz;
	const GEN_FLT x23 = x15 + (-1 * x14);
	const GEN_FLT x24 = x23 * x12;
	const GEN_FLT x25 = 1 + (-1 * x12 * (x8 + x10));
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x27 + x26;
	const GEN_FLT x29 = (x21 * x28) + (x25 * sensor_y) + obj_py + (x24 * sensor_z);
	const GEN_FLT x30 = lh_qw * lh_qi;
	const GEN_FLT x31 = lh_qk * lh_qj;
	const GEN_FLT x32 = x31 + x30;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = x19 + x18;
	const GEN_FLT x35 = x34 * x12;
	const GEN_FLT x36 = x27 + (-1 * x26);
	const GEN_FLT x37 = 1 + (-1 * x12 * (x7 + x10));
	const GEN_FLT x38 = (x37 * sensor_x) + (x36 * x17) + (x35 * sensor_z) + obj_px;
	const GEN_FLT x39 = lh_qw * lh_qj;
	const GEN_FLT x40 = lh_qk * lh_qi;
	const GEN_FLT x41 = x40 + (-1 * x39);
	const GEN_FLT x42 = x5 * x41;
	const GEN_FLT x43 = (x42 * x38) + (x33 * x29) + (x6 * x22) + lh_pz;
	const GEN_FLT x44 = 1. / x43;
	const GEN_FLT x45 = 1 + (-1 * x3 * x5);
	const GEN_FLT x46 = lh_qw * lh_qk;
	const GEN_FLT x47 = lh_qj * lh_qi;
	const GEN_FLT x48 = x47 + (-1 * x46);
	const GEN_FLT x49 = 4 * x4 * x11;
	const GEN_FLT x50 = x49 * x28;
	const GEN_FLT x51 = x40 + x39;
	const GEN_FLT x52 = x51 * x49;
	const GEN_FLT x53 = (x52 * x20) + (x50 * x48) + (x45 * x37);
	const GEN_FLT x54 = x6 * x12;
	const GEN_FLT x55 = (x54 * x20) + (x50 * x32) + (x42 * x37);
	const GEN_FLT x56 = x43 * x43;
	const GEN_FLT x57 = 1. / x56;
	const GEN_FLT x58 = x5 * x22;
	const GEN_FLT x59 = x5 * x48;
	const GEN_FLT x60 = (x45 * x38) + (x51 * x58) + (x59 * x29) + lh_px;
	const GEN_FLT x61 = x60 * x57;
	const GEN_FLT x62 = -1 * x43;
	const GEN_FLT x63 = x60 * x60;
	const GEN_FLT x64 = 2 * (1. / (x56 + x63)) * x56 * atan2(x60, x62) * curve_1;
	const GEN_FLT x65 = x47 + x46;
	const GEN_FLT x66 = x5 * x65;
	const GEN_FLT x67 = 1 + (-1 * (x1 + x2) * x5);
	const GEN_FLT x68 = x31 + (-1 * x30);
	const GEN_FLT x69 = x68 * x49;
	const GEN_FLT x70 = (x69 * x20) + (x67 * x28 * x12) + (x66 * x37);
	const GEN_FLT x71 = (x66 * x38) + (x67 * x29) + (x68 * x58) + lh_py;
	const GEN_FLT x72 = x71 * x57;
	const GEN_FLT x73 = (x71 * x71) + x56;
	const GEN_FLT x74 = 1. / x73;
	const GEN_FLT x75 = x74 * x56;
	const GEN_FLT x76 = 1. / sqrt(1 + (-1 * x74 * x63 * (tilt_1 * tilt_1)));
	const GEN_FLT x77 = (1. / sqrt(x73)) * tilt_1;
	const GEN_FLT x78 = 2 * x71;
	const GEN_FLT x79 = 2 * x43;
	const GEN_FLT x80 = 1.0 / 2.0 * (1. / (x73 * sqrt(x73))) * x60 * tilt_1;
	const GEN_FLT x81 = (-1 * x76 * ((-1 * ((x79 * x55) + (x70 * x78)) * x80) + (x77 * x53))) +
						(-1 * ((-1 * x72 * x55) + (x70 * x44)) * x75);
	const GEN_FLT x82 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x71, x62)) + (-1 * phase_1) + (-1 * asin(x77 * x60)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x83 = (x52 * x16) + (x45 * x36 * x12) + (x59 * x25);
	const GEN_FLT x84 = x41 * x49;
	const GEN_FLT x85 = (x54 * x16) + (x33 * x25) + (x84 * x36);
	const GEN_FLT x86 = x65 * x49;
	const GEN_FLT x87 = (x69 * x16) + (x67 * x25) + (x86 * x36);
	const GEN_FLT x88 = (-1 * x76 * ((-1 * ((x85 * x79) + (x87 * x78)) * x80) + (x83 * x77))) +
						(-1 * ((-1 * x85 * x72) + (x87 * x44)) * x75);
	const GEN_FLT x89 = x49 * x23;
	const GEN_FLT x90 = x5 * x13;
	const GEN_FLT x91 = (x51 * x90) + (x89 * x48) + (x45 * x35);
	const GEN_FLT x92 = (x6 * x13) + (x89 * x32) + (x84 * x34);
	const GEN_FLT x93 = (x68 * x90) + (x67 * x24) + (x86 * x34);
	const GEN_FLT x94 = (-1 * x76 * ((-1 * ((x79 * x92) + (x78 * x93)) * x80) + (x77 * x91))) +
						(-1 * ((-1 * x72 * x92) + (x93 * x44)) * x75);
	out[0] = x81 + (x81 * x82) + (((x61 * x55) + (-1 * x53 * x44)) * x64);
	out[1] = x88 + (x82 * x88) + (((x85 * x61) + (-1 * x83 * x44)) * x64);
	out[2] = x94 + (x82 * x94) + (((x61 * x92) + (-1 * x91 * x44)) * x64);
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
	const GEN_FLT x2 = x1 + x0;
	const GEN_FLT x3 = lh_qk * lh_qk;
	const GEN_FLT x4 = x0 + x3;
	const GEN_FLT x5 = sqrt(x4 + (lh_qw * lh_qw) + x1);
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = obj_qj * obj_qj;
	const GEN_FLT x8 = obj_qi * obj_qi;
	const GEN_FLT x9 = x8 + x7;
	const GEN_FLT x10 = obj_qk * obj_qk;
	const GEN_FLT x11 = 2 * sqrt(x9 + (obj_qw * obj_qw) + x10);
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 = ((x16 + (-1 * x15)) * x17) + ((x13 + x12) * x14) + ((1 + (-1 * x9 * x11)) * sensor_z) + obj_pz;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x20 + x19;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 =
		((x24 + x23) * x17) + ((1 + (-1 * x11 * (x8 + x10))) * sensor_y) + obj_py + ((x13 + (-1 * x12)) * x22);
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = x28 + (-1 * x27);
	const GEN_FLT x30 =
		((1 + (-1 * x11 * (x7 + x10))) * sensor_x) + ((x24 + (-1 * x23)) * x14) + ((x16 + x15) * x22) + obj_px;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = (x31 * x29) + (x21 * x26) + (x18 * (1 + (-1 * x2 * x6))) + lh_pz;
	const GEN_FLT x33 = x32 * x32;
	const GEN_FLT x34 = x20 + (-1 * x19);
	const GEN_FLT x35 = x6 * x18;
	const GEN_FLT x36 = x1 + x3;
	const GEN_FLT x37 = lh_qw * lh_qk;
	const GEN_FLT x38 = lh_qj * lh_qi;
	const GEN_FLT x39 = x38 + x37;
	const GEN_FLT x40 = (x31 * x39) + (x25 * (1 + (-1 * x6 * x36))) + (x34 * x35) + lh_py;
	const GEN_FLT x41 = (x40 * x40) + x33;
	const GEN_FLT x42 = 1. / x41;
	const GEN_FLT x43 = x28 + x27;
	const GEN_FLT x44 = x38 + (-1 * x37);
	const GEN_FLT x45 = (x44 * x26) + (x30 * (1 + (-1 * x4 * x6))) + (x43 * x35) + lh_px;
	const GEN_FLT x46 = x45 * x45;
	const GEN_FLT x47 = 1. / sqrt(1 + (-1 * x42 * x46 * (tilt_1 * tilt_1)));
	const GEN_FLT x48 = (1. / sqrt(x41)) * tilt_1;
	const GEN_FLT x49 = x47 * x48;
	const GEN_FLT x50 = 2 * x32;
	const GEN_FLT x51 = -1 * x32;
	const GEN_FLT x52 = (1. / (x33 + x46)) * atan2(x45, x51) * curve_1;
	const GEN_FLT x53 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x40, x51)) + (-1 * phase_1) + (-1 * asin(x45 * x48)) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x54 = (1. / (x41 * sqrt(x41))) * x45 * tilt_1;
	const GEN_FLT x55 = x54 * x47;
	const GEN_FLT x56 = (x55 * x40) + (-1 * x42 * x32);
	const GEN_FLT x57 = 2 * x52;
	const GEN_FLT x58 = (x55 * x32) + (x40 * x42);
	const GEN_FLT x59 = 1. / x32;
	const GEN_FLT x60 = 2 * (1. / x5);
	const GEN_FLT x61 = x4 * x60;
	const GEN_FLT x62 = x61 * x30;
	const GEN_FLT x63 = x44 * x25;
	const GEN_FLT x64 = x60 * x63;
	const GEN_FLT x65 = x26 * lh_qk;
	const GEN_FLT x66 = x60 * x18;
	const GEN_FLT x67 = x66 * x43;
	const GEN_FLT x68 = x35 * lh_qj;
	const GEN_FLT x69 = x68 + (x67 * lh_qw) + (-1 * x65) + (x64 * lh_qw) + (-1 * x62 * lh_qw);
	const GEN_FLT x70 = x30 * x29;
	const GEN_FLT x71 = x70 * x60;
	const GEN_FLT x72 = x25 * x21;
	const GEN_FLT x73 = x72 * x60;
	const GEN_FLT x74 = x26 * lh_qi;
	const GEN_FLT x75 = x31 * lh_qj;
	const GEN_FLT x76 = x2 * x60;
	const GEN_FLT x77 = x76 * x18;
	const GEN_FLT x78 = (-1 * x75) + (-1 * x77 * lh_qw) + (x73 * lh_qw) + x74 + (x71 * lh_qw);
	const GEN_FLT x79 = 1. / x33;
	const GEN_FLT x80 = x79 * x45;
	const GEN_FLT x81 = x57 * x33;
	const GEN_FLT x82 = x30 * x39;
	const GEN_FLT x83 = x82 * x60;
	const GEN_FLT x84 = x31 * lh_qk;
	const GEN_FLT x85 = x60 * x36;
	const GEN_FLT x86 = x85 * x25;
	const GEN_FLT x87 = x66 * x34;
	const GEN_FLT x88 = x35 * lh_qi;
	const GEN_FLT x89 = (-1 * x88) + (x87 * lh_qw) + (-1 * x86 * lh_qw) + x84 + (x83 * lh_qw);
	const GEN_FLT x90 = x79 * x40;
	const GEN_FLT x91 = x42 * x33;
	const GEN_FLT x92 = 2 * x40;
	const GEN_FLT x93 = 1.0 / 2.0 * x54;
	const GEN_FLT x94 = (-1 * x47 * ((-1 * ((x78 * x50) + (x89 * x92)) * x93) + (x69 * x48))) +
						(-1 * ((-1 * x78 * x90) + (x89 * x59)) * x91);
	const GEN_FLT x95 = x26 * lh_qj;
	const GEN_FLT x96 = x35 * lh_qk;
	const GEN_FLT x97 = (x67 * lh_qi) + x95 + x96 + (x64 * lh_qi) + (-1 * x62 * lh_qi);
	const GEN_FLT x98 = x26 * lh_qw;
	const GEN_FLT x99 = 4 * x5;
	const GEN_FLT x100 = -1 * x99 * lh_qi;
	const GEN_FLT x101 = (x73 * lh_qi) + (x18 * (x100 + (-1 * x76 * lh_qi))) + x98 + x84 + (x71 * lh_qi);
	const GEN_FLT x102 = x79 * x101;
	const GEN_FLT x103 = x35 * lh_qw;
	const GEN_FLT x104 = (x87 * lh_qi) + (-1 * x103) + (x25 * (x100 + (-1 * x85 * lh_qi))) + x75 + (x83 * lh_qi);
	const GEN_FLT x105 = (-1 * x47 * ((-1 * ((x50 * x101) + (x92 * x104)) * x93) + (x97 * x48))) +
						 (-1 * ((-1 * x40 * x102) + (x59 * x104)) * x91);
	const GEN_FLT x106 = -1 * x99 * lh_qj;
	const GEN_FLT x107 = x60 * lh_qj;
	const GEN_FLT x108 = x103 + (x67 * lh_qj) + x74 + (x63 * x107) + (x30 * (x106 + (-1 * x61 * lh_qj)));
	const GEN_FLT x109 = x31 * lh_qw;
	const GEN_FLT x110 = (x18 * (x106 + (-1 * x76 * lh_qj))) + x65 + (x72 * x107) + (-1 * x109) + (x70 * x107);
	const GEN_FLT x111 = x31 * lh_qi;
	const GEN_FLT x112 = x96 + (x87 * lh_qj) + (-1 * x86 * lh_qj) + x111 + (x83 * lh_qj);
	const GEN_FLT x113 = (-1 * x47 * ((-1 * ((x50 * x110) + (x92 * x112)) * x93) + (x48 * x108))) +
						 (-1 * ((-1 * x90 * x110) + (x59 * x112)) * x91);
	const GEN_FLT x114 = -1 * x99 * lh_qk;
	const GEN_FLT x115 = x60 * lh_qk;
	const GEN_FLT x116 = (x67 * lh_qk) + (x63 * x115) + x88 + (-1 * x98) + (x30 * (x114 + (-1 * x61 * lh_qk)));
	const GEN_FLT x117 = (-1 * x77 * lh_qk) + x111 + x95 + (x72 * x115) + (x70 * x115);
	const GEN_FLT x118 = x68 + (x25 * (x114 + (-1 * x85 * lh_qk))) + (x87 * lh_qk) + (x82 * x115) + x109;
	const GEN_FLT x119 = (-1 * x47 * ((-1 * ((x50 * x117) + (x92 * x118)) * x93) + (x48 * x116))) +
						 (-1 * ((-1 * x90 * x117) + (x59 * x118)) * x91);
	out[0] = (-1 * x53 * x49) + (-1 * x50 * x52) + (-1 * x49);
	out[1] = x56 + (x53 * x56);
	out[2] = x58 + (x53 * x58) + (x57 * x45);
	out[3] = x94 + (x53 * x94) + (((x80 * x78) + (-1 * x69 * x59)) * x81);
	out[4] = x105 + (x53 * x105) + (x81 * ((x45 * x102) + (-1 * x59 * x97)));
	out[5] = x113 + (x53 * x113) + (((x80 * x110) + (-1 * x59 * x108)) * x81);
	out[6] = x119 + (x53 * x119) + (((x80 * x117) + (-1 * x59 * x116)) * x81);
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
	const GEN_FLT x3 = x1 + x2;
	const GEN_FLT x4 = 2 * sqrt(x3 + (lh_qw * lh_qw) + x0);
	const GEN_FLT x5 = obj_qj * obj_qj;
	const GEN_FLT x6 = obj_qi * obj_qi;
	const GEN_FLT x7 = obj_qk * obj_qk;
	const GEN_FLT x8 = x6 + x7;
	const GEN_FLT x9 = 2 * sqrt(x8 + (obj_qw * obj_qw) + x5);
	const GEN_FLT x10 = obj_qw * obj_qi;
	const GEN_FLT x11 = obj_qk * obj_qj;
	const GEN_FLT x12 = x9 * sensor_y;
	const GEN_FLT x13 = obj_qw * obj_qj;
	const GEN_FLT x14 = obj_qk * obj_qi;
	const GEN_FLT x15 = x9 * sensor_x;
	const GEN_FLT x16 =
		((x14 + (-1 * x13)) * x15) + ((x11 + x10) * x12) + ((1 + (-1 * (x6 + x5) * x9)) * sensor_z) + obj_pz;
	const GEN_FLT x17 = lh_qw * lh_qi;
	const GEN_FLT x18 = lh_qk * lh_qj;
	const GEN_FLT x19 = x9 * sensor_z;
	const GEN_FLT x20 = obj_qw * obj_qk;
	const GEN_FLT x21 = obj_qj * obj_qi;
	const GEN_FLT x22 = ((x21 + x20) * x15) + ((1 + (-1 * x8 * x9)) * sensor_y) + obj_py + ((x11 + (-1 * x10)) * x19);
	const GEN_FLT x23 = x4 * x22;
	const GEN_FLT x24 = lh_qw * lh_qj;
	const GEN_FLT x25 = lh_qk * lh_qi;
	const GEN_FLT x26 =
		((1 + (-1 * (x5 + x7) * x9)) * sensor_x) + ((x21 + (-1 * x20)) * x12) + ((x14 + x13) * x19) + obj_px;
	const GEN_FLT x27 = x4 * x26;
	const GEN_FLT x28 = ((x25 + (-1 * x24)) * x27) + (x16 * (1 + (-1 * (x1 + x0) * x4))) + ((x18 + x17) * x23) + lh_pz;
	const GEN_FLT x29 = x4 * x16;
	const GEN_FLT x30 = lh_qw * lh_qk;
	const GEN_FLT x31 = lh_qj * lh_qi;
	const GEN_FLT x32 = ((x31 + x30) * x27) + (x22 * (1 + (-1 * x4 * x3))) + ((x18 + (-1 * x17)) * x29) + lh_py;
	const GEN_FLT x33 = (x32 * x32) + (x28 * x28);
	const GEN_FLT x34 = (x26 * (1 + (-1 * (x0 + x2) * x4))) + ((x31 + (-1 * x30)) * x23) + ((x25 + x24) * x29) + lh_px;
	const GEN_FLT x35 = x34 * (1. / sqrt(x33));
	const GEN_FLT x36 = -1 * x28;
	const GEN_FLT x37 =
		1.5707963267949 + (-1 * atan2(-1 * x32, x36)) + (-1 * phase_1) + (-1 * asin(x35 * tilt_1)) + gibPhase_1;
	const GEN_FLT x38 = sin(x37) * gibMag_1;
	const GEN_FLT x39 = x35 * (1. / sqrt(1 + (-1 * (x34 * x34) * (1. / x33) * (tilt_1 * tilt_1))));
	out[0] = -1 + (-1 * x38);
	out[1] = (-1 * x38 * x39) + (-1 * x39);
	out[2] = atan2(x34, x36) * atan2(x34, x36);
	out[3] = x38;
	out[4] = -1 * cos(x37);
	out[5] = 0;
	out[6] = 0;
}
