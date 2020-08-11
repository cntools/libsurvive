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
	return -1.5707963267949 + x44 + (-1 * phase_0) + (sin(x44 + gibPhase_0) * gibMag_0);
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
	return -1.5707963267949 + x44 + (sin(x44 + gibPhase_1) * gibMag_1) + (-1 * phase_1);
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
	return x35 + (-1 * cos(1.5707963267949 + x35 + gibPhase_0) * gibMag_0) +
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
	return x35 + ((atan2(x29, x34) * atan2(x29, x34)) * curve_1) +
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
	const GEN_FLT x12 = (x11 * x11) + (x9 * x9);
	const GEN_FLT x13 = x5 + (2 * ((x7 * lh_qk) + (-1 * x10 * lh_qi))) + lh_py;
	const GEN_FLT x14 = x13 * (1. / sqrt(x12));
	const GEN_FLT x15 = tan(x0) * x14;
	const GEN_FLT x16 = atan2(-1 * x9, x11);
	const GEN_FLT x17 = (sin(x16 + (-1 * asin(x15)) + ogeeMag_0) * ogeePhase_0) + curve_0;
	const GEN_FLT x18 = cos(x0);
	const GEN_FLT x19 = x13 * (1. / sqrt(x12 + (x13 * x13)));
	const GEN_FLT x20 = asin(x19 * (1. / x18));
	const GEN_FLT x21 = 0.0028679863 + (x20 * (-8.0108022e-06 + (-8.0108022e-06 * x20)));
	const GEN_FLT x22 = 5.3685255e-06 + (x20 * x21);
	const GEN_FLT x23 = 0.0076069798 + (x22 * x20);
	const GEN_FLT x24 = asin(
		x15 +
		(x23 * (x20 * x20) * x17 *
		 (1. /
		  (x18 + (-1 * sin(x0) * x17 *
				  ((x20 * (x23 + (x20 * (x22 + (x20 * (x21 + (x20 * (-8.0108022e-06 + (-1.60216044e-05 * x20))))))))) +
				   (x23 * x20)))))));
	const GEN_FLT x25 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x26 = cos(x25);
	const GEN_FLT x27 = asin((1. / x26) * x19);
	const GEN_FLT x28 = 0.0028679863 + (x27 * (-8.0108022e-06 + (-8.0108022e-06 * x27)));
	const GEN_FLT x29 = 5.3685255e-06 + (x28 * x27);
	const GEN_FLT x30 = 0.0076069798 + (x29 * x27);
	const GEN_FLT x31 = -1 * x14 * tan(x25);
	const GEN_FLT x32 = (sin(x16 + (-1 * asin(x31)) + ogeeMag_1) * ogeePhase_1) + curve_1;
	const GEN_FLT x33 =
		(-1 *
		 asin(x31 + (x30 * x32 * (x27 * x27) *
					 (1. / (x26 + (x32 * sin(x25) *
								   ((x27 * (x30 + (x27 * (x29 + (x27 * (x28 + (x27 * (-8.0108022e-06 +
																					  (-1.60216044e-05 * x27))))))))) +
									(x30 * x27)))))))) +
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
	const GEN_FLT x23 = 1. / x22;
	const GEN_FLT x24 = -2 * (lh_qk * lh_qk);
	const GEN_FLT x25 = -2 * (lh_qj * lh_qj);
	const GEN_FLT x26 = 1 + x25 + x24;
	const GEN_FLT x27 = x22 * x22;
	const GEN_FLT x28 = (x18 * lh_qj) + (-1 * x13 * lh_qk) + (x19 * lh_qw);
	const GEN_FLT x29 = x18 + (2 * ((x20 * lh_qi) + (-1 * x28 * lh_qj))) + lh_pz;
	const GEN_FLT x30 = x29 * (1. / x27);
	const GEN_FLT x31 = x27 + (x29 * x29);
	const GEN_FLT x32 = 1. / x31;
	const GEN_FLT x33 = x32 * x27;
	const GEN_FLT x34 = x33 * ((x30 * x26) + (-1 * x4 * x23));
	const GEN_FLT x35 = 0.523598775598299 + tilt_0;
	const GEN_FLT x36 = cos(x35);
	const GEN_FLT x37 = 1. / x36;
	const GEN_FLT x38 = x13 + (2 * ((x28 * lh_qk) + (-1 * x21 * lh_qi))) + lh_py;
	const GEN_FLT x39 = x38 * x38;
	const GEN_FLT x40 = x31 + x39;
	const GEN_FLT x41 = 1. / sqrt(x40);
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
	const GEN_FLT x56 = 1. / sqrt(x31);
	const GEN_FLT x57 = tan(x35);
	const GEN_FLT x58 = x57 * x56;
	const GEN_FLT x59 = x58 * x38;
	const GEN_FLT x60 = atan2(-1 * x29, x22);
	const GEN_FLT x61 = x60 + (-1 * asin(x59)) + ogeeMag_0;
	const GEN_FLT x62 = (sin(x61) * ogeePhase_0) + curve_0;
	const GEN_FLT x63 = x62 * x55;
	const GEN_FLT x64 = x36 + (-1 * x63 * x54);
	const GEN_FLT x65 = 1. / x64;
	const GEN_FLT x66 = x43 * x43;
	const GEN_FLT x67 = x62 * x66;
	const GEN_FLT x68 = x67 * x65;
	const GEN_FLT x69 = x59 + (x68 * x48);
	const GEN_FLT x70 = 1. / sqrt(1 + (-1 * (x69 * x69)));
	const GEN_FLT x71 = x32 * x39;
	const GEN_FLT x72 = 1. / sqrt(1 + (-1 * x71 * (x57 * x57)));
	const GEN_FLT x73 = x0 * lh_qi;
	const GEN_FLT x74 = x2 * lh_qw;
	const GEN_FLT x75 = x74 + x73;
	const GEN_FLT x76 = x75 * x56;
	const GEN_FLT x77 = 2 * x22;
	const GEN_FLT x78 = 2 * x29;
	const GEN_FLT x79 = (x4 * x78) + (x77 * x26);
	const GEN_FLT x80 = 1.0 / 2.0 * x38;
	const GEN_FLT x81 = x80 * (1. / (x31 * sqrt(x31)));
	const GEN_FLT x82 = x81 * x57;
	const GEN_FLT x83 = (-1 * x82 * x79) + (x76 * x57);
	const GEN_FLT x84 = cos(x61) * ogeePhase_0;
	const GEN_FLT x85 = x84 * ((-1 * x83 * x72) + x34);
	const GEN_FLT x86 = x65 * x66 * x48;
	const GEN_FLT x87 = 2 * x38;
	const GEN_FLT x88 = x79 + (x87 * x75);
	const GEN_FLT x89 = x80 * (1. / (x40 * sqrt(x40)));
	const GEN_FLT x90 = x89 * x37;
	const GEN_FLT x91 = x75 * x41;
	const GEN_FLT x92 = (x91 * x37) + (-1 * x88 * x90);
	const GEN_FLT x93 = (1. / x40) * x39;
	const GEN_FLT x94 = 1. / sqrt(1 + (-1 * x93 * (1. / (x36 * x36))));
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
	const GEN_FLT x110 = (1. / (x64 * x64)) * x67 * x48;
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
	const GEN_FLT x121 = 1 + (-2 * (lh_qi * lh_qi));
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
	const GEN_FLT x255 = 1. / x254;
	const GEN_FLT x256 = asin(x42 * x255);
	const GEN_FLT x257 = 8.0108022e-06 * x256;
	const GEN_FLT x258 = 1. / sqrt(1 + (-1 * x93 * (1. / (x254 * x254))));
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
	const GEN_FLT x286 = 1. / x285;
	const GEN_FLT x287 = x256 * x256;
	const GEN_FLT x288 = x287 * x283;
	const GEN_FLT x289 = x286 * x288;
	const GEN_FLT x290 = 1. / sqrt(1 + (-1 * x71 * (x279 * x279)));
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
	const GEN_FLT x304 = x271 * x288 * (1. / (x285 * x285));
	const GEN_FLT x305 = x281 + (x271 * x289);
	const GEN_FLT x306 = 1. / sqrt(1 + (-1 * (x305 * x305)));
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
	const GEN_FLT x8 = -2 * (obj_qk * obj_qk);
	const GEN_FLT x9 = 1 + (-2 * (obj_qj * obj_qj));
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
	const GEN_FLT x25 = 1. / x24;
	const GEN_FLT x26 = 2 * lh_qk;
	const GEN_FLT x27 = (x4 * lh_qw) + (-1 * x10 * lh_qj) + (x7 * lh_qi);
	const GEN_FLT x28 = x10 + (x27 * x12) + (-1 * x26 * x13);
	const GEN_FLT x29 = x24 * x24;
	const GEN_FLT x30 = (x20 * lh_qj) + (-1 * x18 * lh_qk) + (x21 * lh_qw);
	const GEN_FLT x31 = x20 + (2 * ((x22 * lh_qi) + (-1 * x30 * lh_qj))) + lh_pz;
	const GEN_FLT x32 = x31 * (1. / x29);
	const GEN_FLT x33 = x29 + (x31 * x31);
	const GEN_FLT x34 = 1. / x33;
	const GEN_FLT x35 = x34 * x29;
	const GEN_FLT x36 = ((x32 * x28) + (-1 * x25 * x15)) * x35;
	const GEN_FLT x37 = 0.523598775598299 + tilt_0;
	const GEN_FLT x38 = cos(x37);
	const GEN_FLT x39 = 1. / x38;
	const GEN_FLT x40 = x18 + (2 * ((x30 * lh_qk) + (-1 * x23 * lh_qi))) + lh_py;
	const GEN_FLT x41 = x40 * x40;
	const GEN_FLT x42 = x33 + x41;
	const GEN_FLT x43 = 1. / sqrt(x42);
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
	const GEN_FLT x59 = 1. / sqrt(x33);
	const GEN_FLT x60 = x59 * x40;
	const GEN_FLT x61 = x60 * x58;
	const GEN_FLT x62 = atan2(-1 * x31, x24);
	const GEN_FLT x63 = x62 + (-1 * asin(x61)) + ogeeMag_0;
	const GEN_FLT x64 = (sin(x63) * ogeePhase_0) + curve_0;
	const GEN_FLT x65 = x64 * x57;
	const GEN_FLT x66 = x38 + (-1 * x65 * x56);
	const GEN_FLT x67 = 1. / x66;
	const GEN_FLT x68 = x45 * x45;
	const GEN_FLT x69 = x64 * x68;
	const GEN_FLT x70 = x67 * x69;
	const GEN_FLT x71 = x61 + (x70 * x50);
	const GEN_FLT x72 = 1. / sqrt(1 + (-1 * (x71 * x71)));
	const GEN_FLT x73 = x7 + (x26 * x11) + (-1 * x27 * x14);
	const GEN_FLT x74 = x73 * x59;
	const GEN_FLT x75 = 2 * x24;
	const GEN_FLT x76 = 2 * x31;
	const GEN_FLT x77 = (x76 * x15) + (x75 * x28);
	const GEN_FLT x78 = 1.0 / 2.0 * x40;
	const GEN_FLT x79 = x78 * (1. / (x33 * sqrt(x33)));
	const GEN_FLT x80 = x79 * x58;
	const GEN_FLT x81 = (-1 * x80 * x77) + (x74 * x58);
	const GEN_FLT x82 = x41 * x34;
	const GEN_FLT x83 = 1. / sqrt(1 + (-1 * x82 * (x58 * x58)));
	const GEN_FLT x84 = (-1 * x81 * x83) + x36;
	const GEN_FLT x85 = cos(x63) * ogeePhase_0;
	const GEN_FLT x86 = x67 * x68 * x50;
	const GEN_FLT x87 = x85 * x86;
	const GEN_FLT x88 = x41 * (1. / x42);
	const GEN_FLT x89 = 1. / sqrt(1 + (-1 * x88 * (1. / (x38 * x38))));
	const GEN_FLT x90 = 2 * x40;
	const GEN_FLT x91 = x78 * (1. / (x42 * sqrt(x42)));
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
	const GEN_FLT x103 = (1. / (x66 * x66)) * x69 * x50;
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
	const GEN_FLT x110 = -2 * (obj_qi * obj_qi);
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
	const GEN_FLT x156 = 1. / x155;
	const GEN_FLT x157 = asin(x44 * x156);
	const GEN_FLT x158 = 8.0108022e-06 * x157;
	const GEN_FLT x159 = 1. / sqrt(1 + (-1 * x88 * (1. / (x155 * x155))));
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
	const GEN_FLT x181 = 1. / x180;
	const GEN_FLT x182 = x157 * x157;
	const GEN_FLT x183 = x178 * x182;
	const GEN_FLT x184 = x181 * x183;
	const GEN_FLT x185 = x79 * x175;
	const GEN_FLT x186 = (x77 * x185) + (-1 * x74 * x175);
	const GEN_FLT x187 = 1. / sqrt(1 + (-1 * x82 * (x175 * x175)));
	const GEN_FLT x188 = (-1 * x187 * x186) + x36;
	const GEN_FLT x189 = cos(x177) * ogeePhase_1;
	const GEN_FLT x190 = x167 * x181 * x182;
	const GEN_FLT x191 = x189 * x190;
	const GEN_FLT x192 = 2 * x168 * x178 * x181;
	const GEN_FLT x193 = 2.40324066e-05 * x157;
	const GEN_FLT x194 = x174 * x173;
	const GEN_FLT x195 = x189 * x194;
	const GEN_FLT x196 = x167 * (1. / (x180 * x180)) * x183;
	const GEN_FLT x197 = x176 + (x167 * x184);
	const GEN_FLT x198 = 1. / sqrt(1 + (-1 * (x197 * x197)));
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
	const GEN_FLT x23 = x22 * x22;
	const GEN_FLT x24 = x23 + (x17 * x17);
	const GEN_FLT x25 = 1. / x24;
	const GEN_FLT x26 = x25 * x17;
	const GEN_FLT x27 = x7 + (2 * ((x12 * lh_qk) + (-1 * x21 * lh_qi))) + lh_py;
	const GEN_FLT x28 = 0.523598775598299 + tilt_0;
	const GEN_FLT x29 = cos(x28);
	const GEN_FLT x30 = 1. / x29;
	const GEN_FLT x31 = x27 * x27;
	const GEN_FLT x32 = x24 + x31;
	const GEN_FLT x33 = 1. / sqrt(x32);
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
	const GEN_FLT x48 = 1. / sqrt(x24);
	const GEN_FLT x49 = tan(x28);
	const GEN_FLT x50 = x48 * x49;
	const GEN_FLT x51 = x50 * x27;
	const GEN_FLT x52 = atan2(-1 * x17, x22);
	const GEN_FLT x53 = x52 + (-1 * asin(x51)) + ogeeMag_0;
	const GEN_FLT x54 = (sin(x53) * ogeePhase_0) + curve_0;
	const GEN_FLT x55 = x54 * x47;
	const GEN_FLT x56 = x29 + (-1 * x55 * x46);
	const GEN_FLT x57 = 1. / x56;
	const GEN_FLT x58 = x35 * x35;
	const GEN_FLT x59 = x54 * x58;
	const GEN_FLT x60 = x57 * x59;
	const GEN_FLT x61 = x51 + (x60 * x40);
	const GEN_FLT x62 = 1. / sqrt(1 + (-1 * (x61 * x61)));
	const GEN_FLT x63 = (1. / (x24 * sqrt(x24))) * x27;
	const GEN_FLT x64 = x63 * x22;
	const GEN_FLT x65 = x64 * x49;
	const GEN_FLT x66 = x31 * x25;
	const GEN_FLT x67 = 1. / sqrt(1 + (-1 * x66 * (x49 * x49)));
	const GEN_FLT x68 = (x67 * x65) + x26;
	const GEN_FLT x69 = cos(x53) * ogeePhase_0;
	const GEN_FLT x70 = x58 * x57 * x40;
	const GEN_FLT x71 = x70 * x69;
	const GEN_FLT x72 = 1. / (x32 * sqrt(x32));
	const GEN_FLT x73 = x72 * x27;
	const GEN_FLT x74 = x73 * x22;
	const GEN_FLT x75 = (1. / x32) * x31;
	const GEN_FLT x76 = 1. / sqrt(1 + (-1 * x75 * (1. / (x29 * x29))));
	const GEN_FLT x77 = x76 * x30;
	const GEN_FLT x78 = x74 * x77;
	const GEN_FLT x79 = -1 * x78 * x37;
	const GEN_FLT x80 = (-1 * x78 * x38) + (x35 * (x79 + (x78 * x36)));
	const GEN_FLT x81 = (x80 * x35) + (-1 * x78 * x39);
	const GEN_FLT x82 = 2.40324066e-05 * x35;
	const GEN_FLT x83 = x46 * x47;
	const GEN_FLT x84 = x83 * x69;
	const GEN_FLT x85 = (1. / (x56 * x56)) * x59 * x40;
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
	const GEN_FLT x113 = 1. / x22;
	const GEN_FLT x114 = 2 * x19;
	const GEN_FLT x115 = (2 * x20) + (-1 * x114);
	const GEN_FLT x116 = 2 * x8;
	const GEN_FLT x117 = (2 * x11) + (-1 * x116);
	const GEN_FLT x118 = (1. / x23) * x17;
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
	const GEN_FLT x189 = 1. / x188;
	const GEN_FLT x190 = x33 * x189;
	const GEN_FLT x191 = asin(x27 * x190);
	const GEN_FLT x192 = 8.0108022e-06 * x191;
	const GEN_FLT x193 = 1. / sqrt(1 + (-1 * x75 * (1. / (x188 * x188))));
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
	const GEN_FLT x216 = 1. / x215;
	const GEN_FLT x217 = x191 * x191;
	const GEN_FLT x218 = x213 * x217;
	const GEN_FLT x219 = x218 * x216;
	const GEN_FLT x220 = 1. / sqrt(1 + (-1 * x66 * (x186 * x186)));
	const GEN_FLT x221 = (-1 * x220 * x187) + x26;
	const GEN_FLT x222 = cos(x212) * ogeePhase_1;
	const GEN_FLT x223 = x217 * x216 * x202;
	const GEN_FLT x224 = x223 * x222;
	const GEN_FLT x225 = x213 * x216 * x203;
	const GEN_FLT x226 = x225 * x194;
	const GEN_FLT x227 = 2.40324066e-05 * x191;
	const GEN_FLT x228 = x208 * x209;
	const GEN_FLT x229 = x222 * x228;
	const GEN_FLT x230 = x218 * (1. / (x215 * x215)) * x202;
	const GEN_FLT x231 = x211 + (x219 * x202);
	const GEN_FLT x232 = 1. / sqrt(1 + (-1 * (x231 * x231)));
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
	const GEN_FLT x14 = (x13 * x13) + (x12 * x12);
	const GEN_FLT x15 = (1. / sqrt(x14)) * x10;
	const GEN_FLT x16 = x1 * x15;
	const GEN_FLT x17 = atan2(-1 * x12, x13);
	const GEN_FLT x18 = x17 + (-1 * asin(x16)) + ogeeMag_0;
	const GEN_FLT x19 = sin(x18);
	const GEN_FLT x20 = (x19 * ogeePhase_0) + curve_0;
	const GEN_FLT x21 = cos(x0);
	const GEN_FLT x22 = x10 * x10;
	const GEN_FLT x23 = x14 + x22;
	const GEN_FLT x24 = (1. / sqrt(x23)) * x10;
	const GEN_FLT x25 = asin(x24 * (1. / x21));
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
	const GEN_FLT x41 = 1. / x40;
	const GEN_FLT x42 = x25 * x25;
	const GEN_FLT x43 = x41 * x42;
	const GEN_FLT x44 = x43 * x30;
	const GEN_FLT x45 = x16 + (x44 * x20);
	const GEN_FLT x46 = 1. / sqrt(1 + (-1 * (x45 * x45)));
	const GEN_FLT x47 = x1 * x1;
	const GEN_FLT x48 = x15 * (1 + x47);
	const GEN_FLT x49 = cos(x18) * ogeePhase_0;
	const GEN_FLT x50 = x44 * x49;
	const GEN_FLT x51 = x22 * (1. / x14);
	const GEN_FLT x52 = x48 * (1. / sqrt(1 + (-1 * x51 * x47)));
	const GEN_FLT x53 = 1. / (x21 * x21);
	const GEN_FLT x54 = x22 * (1. / x23);
	const GEN_FLT x55 = x53 * x24 * (1. / sqrt(1 + (-1 * x54 * x53)));
	const GEN_FLT x56 = x55 * x37;
	const GEN_FLT x57 = x56 * x27;
	const GEN_FLT x58 = (x56 * x28) + (x25 * (x57 + (-1 * x56 * x26)));
	const GEN_FLT x59 = (x58 * x25) + (x56 * x29);
	const GEN_FLT x60 = (1. / (x40 * x40)) * x42 * x30;
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
	const GEN_FLT x70 = x69 * x69;
	const GEN_FLT x71 = x15 * (1 + x70);
	const GEN_FLT x72 = cos(x68);
	const GEN_FLT x73 = asin((1. / x72) * x24);
	const GEN_FLT x74 = 8.0108022e-06 * x73;
	const GEN_FLT x75 = sin(x68);
	const GEN_FLT x76 = 1. / (x72 * x72);
	const GEN_FLT x77 = x76 * x24 * (1. / sqrt(1 + (-1 * x76 * x54)));
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
	const GEN_FLT x89 = x73 * x73;
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
	const GEN_FLT x100 = 1. / x99;
	const GEN_FLT x101 = x89 * x100;
	const GEN_FLT x102 = x90 * x101;
	const GEN_FLT x103 = cos(x86) * ogeePhase_1;
	const GEN_FLT x104 = x103 * x102;
	const GEN_FLT x105 = x71 * (1. / sqrt(1 + (-1 * x70 * x51)));
	const GEN_FLT x106 = x88 * x75;
	const GEN_FLT x107 = x89 * x90 * (1. / (x99 * x99));
	const GEN_FLT x108 = x85 + (x88 * x102);
	const GEN_FLT x109 = 1. / sqrt(1 + (-1 * (x108 * x108)));
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
	const GEN_FLT x11 = (x10 * x10) + (x8 * x8);
	const GEN_FLT x12 = 0.523598775598299 + tilt_0;
	const GEN_FLT x13 = (2 * ((x6 * lh_qk) + (-1 * x9 * lh_qi))) + x4 + lh_py;
	const GEN_FLT x14 = x13 * (1. / sqrt(x11)) * tan(x12);
	const GEN_FLT x15 = atan2(-1 * x8, x10);
	const GEN_FLT x16 = (sin(x15 + (-1 * asin(x14)) + ogeeMag_0) * ogeePhase_0) + curve_0;
	const GEN_FLT x17 = cos(x12);
	const GEN_FLT x18 = asin(x13 * (1. / x17) * (1. / sqrt(x11 + (x13 * x13))));
	const GEN_FLT x19 = 0.0028679863 + (x18 * (-8.0108022e-06 + (-8.0108022e-06 * x18)));
	const GEN_FLT x20 = 5.3685255e-06 + (x19 * x18);
	const GEN_FLT x21 = 0.0076069798 + (x20 * x18);
	const GEN_FLT x22 = asin(
		x14 +
		(x21 * (x18 * x18) * x16 *
		 (1. /
		  (x17 + (-1 * x16 * sin(x12) *
				  ((x18 * (x21 + (x18 * (x20 + (x18 * (x19 + (x18 * (-8.0108022e-06 + (-1.60216044e-05 * x18))))))))) +
				   (x21 * x18)))))));
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
	const GEN_FLT x23 = 1. / x22;
	const GEN_FLT x24 = -2 * (lh_qk * lh_qk);
	const GEN_FLT x25 = -2 * (lh_qj * lh_qj);
	const GEN_FLT x26 = 1 + x25 + x24;
	const GEN_FLT x27 = (x18 * lh_qj) + (-1 * x13 * lh_qk) + (x19 * lh_qw);
	const GEN_FLT x28 = x18 + (2 * ((x20 * lh_qi) + (-1 * x27 * lh_qj))) + lh_pz;
	const GEN_FLT x29 = x22 * x22;
	const GEN_FLT x30 = x28 * (1. / x29);
	const GEN_FLT x31 = x29 + (x28 * x28);
	const GEN_FLT x32 = 1. / x31;
	const GEN_FLT x33 = x32 * x29;
	const GEN_FLT x34 = x33 * ((x30 * x26) + (-1 * x4 * x23));
	const GEN_FLT x35 = x13 + (2 * ((x27 * lh_qk) + (-1 * x21 * lh_qi))) + lh_py;
	const GEN_FLT x36 = 0.523598775598299 + tilt_0;
	const GEN_FLT x37 = cos(x36);
	const GEN_FLT x38 = 1. / x37;
	const GEN_FLT x39 = x35 * x35;
	const GEN_FLT x40 = x31 + x39;
	const GEN_FLT x41 = (1. / sqrt(x40)) * x38;
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
	const GEN_FLT x56 = x55 * (1. / sqrt(x31));
	const GEN_FLT x57 = x56 * x35;
	const GEN_FLT x58 = atan2(-1 * x28, x22);
	const GEN_FLT x59 = x58 + (-1 * asin(x57)) + ogeeMag_0;
	const GEN_FLT x60 = (sin(x59) * ogeePhase_0) + curve_0;
	const GEN_FLT x61 = x60 * x54;
	const GEN_FLT x62 = x37 + (-1 * x61 * x53);
	const GEN_FLT x63 = 1. / x62;
	const GEN_FLT x64 = x42 * x42;
	const GEN_FLT x65 = x60 * x64;
	const GEN_FLT x66 = x63 * x65;
	const GEN_FLT x67 = x57 + (x66 * x47);
	const GEN_FLT x68 = 1. / sqrt(1 + (-1 * (x67 * x67)));
	const GEN_FLT x69 = 1. / sqrt(1 + (-1 * (x55 * x55) * x32 * x39));
	const GEN_FLT x70 = 2 * lh_qi;
	const GEN_FLT x71 = x70 * lh_qj;
	const GEN_FLT x72 = x0 * lh_qk;
	const GEN_FLT x73 = x72 + x71;
	const GEN_FLT x74 = 2 * x22;
	const GEN_FLT x75 = 2 * x28;
	const GEN_FLT x76 = (x4 * x75) + (x74 * x26);
	const GEN_FLT x77 = 1.0 / 2.0 * x35;
	const GEN_FLT x78 = x77 * x55 * (1. / (x31 * sqrt(x31)));
	const GEN_FLT x79 = (-1 * x78 * x76) + (x73 * x56);
	const GEN_FLT x80 = (-1 * x79 * x69) + x34;
	const GEN_FLT x81 = cos(x59) * ogeePhase_0;
	const GEN_FLT x82 = x63 * x64 * x47;
	const GEN_FLT x83 = x81 * x82;
	const GEN_FLT x84 = 1. / sqrt(1 + (-1 * (1. / x40) * (1. / (x37 * x37)) * x39));
	const GEN_FLT x85 = 2 * x35;
	const GEN_FLT x86 = x77 * (1. / (x40 * sqrt(x40))) * x38;
	const GEN_FLT x87 = x84 * ((x73 * x41) + (-1 * x86 * (x76 + (x85 * x73))));
	const GEN_FLT x88 = x87 * x44;
	const GEN_FLT x89 = (x87 * x45) + (x42 * (x88 + (-1 * x87 * x43)));
	const GEN_FLT x90 = (x89 * x42) + (x87 * x46);
	const GEN_FLT x91 = 2.40324066e-05 * x42;
	const GEN_FLT x92 = x54 * x53;
	const GEN_FLT x93 = x81 * x92;
	const GEN_FLT x94 = (1. / (x62 * x62)) * x65 * x47;
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
	const GEN_FLT x103 = 1 + (-2 * (lh_qi * lh_qi));
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
	const GEN_FLT x8 = -2 * (obj_qk * obj_qk);
	const GEN_FLT x9 = 1 + (-2 * (obj_qj * obj_qj));
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
	const GEN_FLT x23 = 1. / x22;
	const GEN_FLT x24 = 2 * ((x4 * lh_qw) + (-1 * x10 * lh_qj) + (x7 * lh_qi));
	const GEN_FLT x25 = (x24 * lh_qj) + x10 + (-1 * x12 * lh_qk);
	const GEN_FLT x26 = (x18 * lh_qj) + (-1 * x16 * lh_qk) + (x19 * lh_qw);
	const GEN_FLT x27 = x18 + (2 * ((x20 * lh_qi) + (-1 * x26 * lh_qj))) + lh_pz;
	const GEN_FLT x28 = x22 * x22;
	const GEN_FLT x29 = (1. / x28) * x27;
	const GEN_FLT x30 = x28 + (x27 * x27);
	const GEN_FLT x31 = 1. / x30;
	const GEN_FLT x32 = x31 * x28;
	const GEN_FLT x33 = ((x25 * x29) + (-1 * x23 * x13)) * x32;
	const GEN_FLT x34 = (2 * ((x26 * lh_qk) + (-1 * x21 * lh_qi))) + x16 + lh_py;
	const GEN_FLT x35 = 0.523598775598299 + tilt_0;
	const GEN_FLT x36 = cos(x35);
	const GEN_FLT x37 = 1. / x36;
	const GEN_FLT x38 = x34 * x34;
	const GEN_FLT x39 = x30 + x38;
	const GEN_FLT x40 = x37 * (1. / sqrt(x39));
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
	const GEN_FLT x55 = x54 * (1. / sqrt(x30));
	const GEN_FLT x56 = x55 * x34;
	const GEN_FLT x57 = atan2(-1 * x27, x22);
	const GEN_FLT x58 = x57 + (-1 * asin(x56)) + ogeeMag_0;
	const GEN_FLT x59 = (sin(x58) * ogeePhase_0) + curve_0;
	const GEN_FLT x60 = x53 * x59;
	const GEN_FLT x61 = x36 + (-1 * x60 * x52);
	const GEN_FLT x62 = 1. / x61;
	const GEN_FLT x63 = x41 * x41;
	const GEN_FLT x64 = x63 * x59;
	const GEN_FLT x65 = x64 * x62;
	const GEN_FLT x66 = x56 + (x65 * x46);
	const GEN_FLT x67 = 1. / sqrt(1 + (-1 * (x66 * x66)));
	const GEN_FLT x68 = x7 + (x11 * lh_qk) + (-1 * x24 * lh_qi);
	const GEN_FLT x69 = 2 * x22;
	const GEN_FLT x70 = 2 * x27;
	const GEN_FLT x71 = (x70 * x13) + (x69 * x25);
	const GEN_FLT x72 = 1.0 / 2.0 * x34;
	const GEN_FLT x73 = x72 * x54 * (1. / (x30 * sqrt(x30)));
	const GEN_FLT x74 = (-1 * x71 * x73) + (x68 * x55);
	const GEN_FLT x75 = 1. / sqrt(1 + (-1 * (x54 * x54) * x31 * x38));
	const GEN_FLT x76 = (-1 * x75 * x74) + x33;
	const GEN_FLT x77 = cos(x58) * ogeePhase_0;
	const GEN_FLT x78 = x63 * x62 * x46;
	const GEN_FLT x79 = x78 * x77;
	const GEN_FLT x80 = 1. / sqrt(1 + (-1 * (1. / (x36 * x36)) * x38 * (1. / x39)));
	const GEN_FLT x81 = 2 * x34;
	const GEN_FLT x82 = x72 * x37 * (1. / (x39 * sqrt(x39)));
	const GEN_FLT x83 = x80 * ((x68 * x40) + (-1 * x82 * (x71 + (x81 * x68))));
	const GEN_FLT x84 = x83 * x43;
	const GEN_FLT x85 = (x83 * x44) + (x41 * (x84 + (-1 * x83 * x42)));
	const GEN_FLT x86 = (x85 * x41) + (x83 * x45);
	const GEN_FLT x87 = 2.40324066e-05 * x41;
	const GEN_FLT x88 = x53 * x52;
	const GEN_FLT x89 = x88 * x77;
	const GEN_FLT x90 = x64 * (1. / (x61 * x61)) * x46;
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
	const GEN_FLT x97 = -2 * (obj_qi * obj_qi);
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
	const GEN_FLT x23 = x22 * x22;
	const GEN_FLT x24 = x23 + (x17 * x17);
	const GEN_FLT x25 = 1. / x24;
	const GEN_FLT x26 = x25 * x17;
	const GEN_FLT x27 = x7 + (2 * ((x12 * lh_qk) + (-1 * x21 * lh_qi))) + lh_py;
	const GEN_FLT x28 = 0.523598775598299 + tilt_0;
	const GEN_FLT x29 = cos(x28);
	const GEN_FLT x30 = 1. / x29;
	const GEN_FLT x31 = x27 * x27;
	const GEN_FLT x32 = x24 + x31;
	const GEN_FLT x33 = x30 * (1. / sqrt(x32));
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
	const GEN_FLT x48 = x47 * (1. / sqrt(x24));
	const GEN_FLT x49 = x48 * x27;
	const GEN_FLT x50 = atan2(-1 * x17, x22);
	const GEN_FLT x51 = x50 + (-1 * asin(x49)) + ogeeMag_0;
	const GEN_FLT x52 = (sin(x51) * ogeePhase_0) + curve_0;
	const GEN_FLT x53 = x52 * x46;
	const GEN_FLT x54 = x29 + (-1 * x53 * x45);
	const GEN_FLT x55 = 1. / x54;
	const GEN_FLT x56 = x34 * x34;
	const GEN_FLT x57 = x52 * x56;
	const GEN_FLT x58 = x57 * x55;
	const GEN_FLT x59 = x49 + (x58 * x39);
	const GEN_FLT x60 = 1. / sqrt(1 + (-1 * (x59 * x59)));
	const GEN_FLT x61 = x22 * x27;
	const GEN_FLT x62 = x47 * (1. / (x24 * sqrt(x24)));
	const GEN_FLT x63 = x61 * x62;
	const GEN_FLT x64 = 1. / sqrt(1 + (-1 * (x47 * x47) * x31 * x25));
	const GEN_FLT x65 = (x63 * x64) + x26;
	const GEN_FLT x66 = cos(x51) * ogeePhase_0;
	const GEN_FLT x67 = x56 * x55 * x39;
	const GEN_FLT x68 = x67 * x66;
	const GEN_FLT x69 = 1. / sqrt(1 + (-1 * (1. / x32) * x31 * (1. / (x29 * x29))));
	const GEN_FLT x70 = x30 * (1. / (x32 * sqrt(x32)));
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
	const GEN_FLT x81 = (1. / (x54 * x54)) * x57 * x39;
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
	const GEN_FLT x107 = 1. / x22;
	const GEN_FLT x108 = 2 * x19;
	const GEN_FLT x109 = (2 * x20) + (-1 * x108);
	const GEN_FLT x110 = 2 * x8;
	const GEN_FLT x111 = (2 * x11) + (-1 * x110);
	const GEN_FLT x112 = (1. / x23) * x17;
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
	const GEN_FLT x13 = (x12 * x12) + (x10 * x10);
	const GEN_FLT x14 = x6 + (2 * ((x8 * lh_qk) + (-1 * x11 * lh_qi))) + lh_py;
	const GEN_FLT x15 = x14 * (1. / sqrt(x13));
	const GEN_FLT x16 = x1 * x15;
	const GEN_FLT x17 = atan2(-1 * x10, x12);
	const GEN_FLT x18 = x17 + (-1 * asin(x16)) + ogeeMag_0;
	const GEN_FLT x19 = sin(x18);
	const GEN_FLT x20 = (x19 * ogeePhase_0) + curve_0;
	const GEN_FLT x21 = cos(x0);
	const GEN_FLT x22 = x14 * x14;
	const GEN_FLT x23 = x13 + x22;
	const GEN_FLT x24 = (1. / sqrt(x23)) * x14;
	const GEN_FLT x25 = asin(x24 * (1. / x21));
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
	const GEN_FLT x41 = 1. / x40;
	const GEN_FLT x42 = x25 * x25;
	const GEN_FLT x43 = x41 * x42;
	const GEN_FLT x44 = x43 * x30;
	const GEN_FLT x45 = x16 + (x44 * x20);
	const GEN_FLT x46 = 1. / sqrt(1 + (-1 * (x45 * x45)));
	const GEN_FLT x47 = x1 * x1;
	const GEN_FLT x48 = x15 * (1 + x47);
	const GEN_FLT x49 = cos(x18) * ogeePhase_0;
	const GEN_FLT x50 = x44 * x49;
	const GEN_FLT x51 = x48 * (1. / sqrt(1 + (-1 * x47 * x22 * (1. / x13))));
	const GEN_FLT x52 = 1. / (x21 * x21);
	const GEN_FLT x53 = x52 * x24 * (1. / sqrt(1 + (-1 * x52 * x22 * (1. / x23))));
	const GEN_FLT x54 = x53 * x31;
	const GEN_FLT x55 = x54 * x27;
	const GEN_FLT x56 = (x54 * x28) + (x25 * (x55 + (-1 * x54 * x26)));
	const GEN_FLT x57 = (x56 * x25) + (x54 * x29);
	const GEN_FLT x58 = x31 * x20;
	const GEN_FLT x59 = (1. / (x40 * x40)) * x42 * x30;
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
	const GEN_FLT x14 = (x13 * x13) + (x12 * x12);
	const GEN_FLT x15 = asin((1. / x1) * x10 * (1. / sqrt(x14 + (x10 * x10))));
	const GEN_FLT x16 = 0.0028679863 + (x15 * (-8.0108022e-06 + (-8.0108022e-06 * x15)));
	const GEN_FLT x17 = 5.3685255e-06 + (x15 * x16);
	const GEN_FLT x18 = 0.0076069798 + (x15 * x17);
	const GEN_FLT x19 = -1 * tan(x0) * (1. / sqrt(x14)) * x10;
	const GEN_FLT x20 = atan2(-1 * x12, x13);
	const GEN_FLT x21 = (sin(x20 + (-1 * asin(x19)) + ogeeMag_1) * ogeePhase_1) + curve_1;
	const GEN_FLT x22 =
		x20 +
		(-1 *
		 asin(x19 + (x21 * (x15 * x15) * x18 *
					 (1. / (x1 + (sin(x0) * x21 *
								  ((x15 * (x18 + (x15 * (x17 + (x15 * (x16 + (x15 * (-8.0108022e-06 +
																					 (-1.60216044e-05 * x15))))))))) +
								   (x15 * x18))))))));
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
	const GEN_FLT x23 = 1. / x22;
	const GEN_FLT x24 = -2 * (lh_qk * lh_qk);
	const GEN_FLT x25 = -2 * (lh_qj * lh_qj);
	const GEN_FLT x26 = 1 + x25 + x24;
	const GEN_FLT x27 = (x18 * lh_qj) + (-1 * x13 * lh_qk) + (x19 * lh_qw);
	const GEN_FLT x28 = x18 + (2 * ((x20 * lh_qi) + (-1 * x27 * lh_qj))) + lh_pz;
	const GEN_FLT x29 = x22 * x22;
	const GEN_FLT x30 = x28 * (1. / x29);
	const GEN_FLT x31 = x29 + (x28 * x28);
	const GEN_FLT x32 = 1. / x31;
	const GEN_FLT x33 = x32 * x29;
	const GEN_FLT x34 = x33 * ((x30 * x26) + (-1 * x4 * x23));
	const GEN_FLT x35 = x13 + (2 * ((x27 * lh_qk) + (-1 * x21 * lh_qi))) + lh_py;
	const GEN_FLT x36 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x37 = cos(x36);
	const GEN_FLT x38 = 1. / x37;
	const GEN_FLT x39 = x35 * x35;
	const GEN_FLT x40 = x31 + x39;
	const GEN_FLT x41 = (1. / sqrt(x40)) * x38;
	const GEN_FLT x42 = asin(x41 * x35);
	const GEN_FLT x43 = 8.0108022e-06 * x42;
	const GEN_FLT x44 = 1. / sqrt(1 + (-1 * (1. / x40) * (1. / (x37 * x37)) * x39));
	const GEN_FLT x45 = 2 * lh_qj;
	const GEN_FLT x46 = x45 * lh_qi;
	const GEN_FLT x47 = x2 * lh_qw;
	const GEN_FLT x48 = x47 + x46;
	const GEN_FLT x49 = 2 * x35;
	const GEN_FLT x50 = 2 * x22;
	const GEN_FLT x51 = 2 * x28;
	const GEN_FLT x52 = (x4 * x51) + (x50 * x26);
	const GEN_FLT x53 = 1.0 / 2.0 * x35;
	const GEN_FLT x54 = x53 * (1. / (x40 * sqrt(x40))) * x38;
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
	const GEN_FLT x71 = x70 * (1. / sqrt(x31));
	const GEN_FLT x72 = -1 * x71 * x35;
	const GEN_FLT x73 = atan2(-1 * x28, x22);
	const GEN_FLT x74 = x73 + (-1 * asin(x72)) + ogeeMag_1;
	const GEN_FLT x75 = (sin(x74) * ogeePhase_1) + curve_1;
	const GEN_FLT x76 = x75 * x69;
	const GEN_FLT x77 = x37 + (x76 * x68);
	const GEN_FLT x78 = 1. / x77;
	const GEN_FLT x79 = x42 * x42;
	const GEN_FLT x80 = x79 * x75;
	const GEN_FLT x81 = x80 * x78;
	const GEN_FLT x82 = 1. / sqrt(1 + (-1 * (x70 * x70) * x32 * x39));
	const GEN_FLT x83 = x70 * x53 * (1. / (x31 * sqrt(x31)));
	const GEN_FLT x84 = (x83 * x52) + (-1 * x71 * x48);
	const GEN_FLT x85 = (-1 * x82 * x84) + x34;
	const GEN_FLT x86 = cos(x74) * ogeePhase_1;
	const GEN_FLT x87 = x79 * x78 * x62;
	const GEN_FLT x88 = x86 * x87;
	const GEN_FLT x89 = 2 * x78 * x75 * x63;
	const GEN_FLT x90 = 2.40324066e-05 * x42;
	const GEN_FLT x91 = x68 * x69;
	const GEN_FLT x92 = x86 * x91;
	const GEN_FLT x93 = x80 * (1. / (x77 * x77)) * x62;
	const GEN_FLT x94 = x72 + (x81 * x62);
	const GEN_FLT x95 = 1. / sqrt(1 + (-1 * (x94 * x94)));
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
	const GEN_FLT x103 = 1 + (-2 * (lh_qi * lh_qi));
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
	const GEN_FLT x8 = -2 * (obj_qj * obj_qj);
	const GEN_FLT x9 = 1 + (-2 * (obj_qk * obj_qk));
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
	const GEN_FLT x25 = 1. / x24;
	const GEN_FLT x26 = 2 * lh_qk;
	const GEN_FLT x27 = (x4 * lh_qw) + (-1 * x10 * lh_qj) + (x7 * lh_qi);
	const GEN_FLT x28 = x10 + (x27 * x12) + (-1 * x26 * x13);
	const GEN_FLT x29 = (x20 * lh_qj) + (-1 * x18 * lh_qk) + (x21 * lh_qw);
	const GEN_FLT x30 = x20 + (2 * ((x22 * lh_qi) + (-1 * x29 * lh_qj))) + lh_pz;
	const GEN_FLT x31 = x24 * x24;
	const GEN_FLT x32 = x30 * (1. / x31);
	const GEN_FLT x33 = x31 + (x30 * x30);
	const GEN_FLT x34 = 1. / x33;
	const GEN_FLT x35 = x31 * x34;
	const GEN_FLT x36 = ((x32 * x28) + (-1 * x25 * x15)) * x35;
	const GEN_FLT x37 = x18 + (2 * ((x29 * lh_qk) + (-1 * x23 * lh_qi))) + lh_py;
	const GEN_FLT x38 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x39 = cos(x38);
	const GEN_FLT x40 = 1. / x39;
	const GEN_FLT x41 = x37 * x37;
	const GEN_FLT x42 = x33 + x41;
	const GEN_FLT x43 = x40 * (1. / sqrt(x42));
	const GEN_FLT x44 = asin(x43 * x37);
	const GEN_FLT x45 = 8.0108022e-06 * x44;
	const GEN_FLT x46 = 1. / sqrt(1 + (-1 * x41 * (1. / x42) * (1. / (x39 * x39))));
	const GEN_FLT x47 = x7 + (x26 * x11) + (-1 * x27 * x14);
	const GEN_FLT x48 = 2 * x37;
	const GEN_FLT x49 = 2 * x24;
	const GEN_FLT x50 = 2 * x30;
	const GEN_FLT x51 = (x50 * x15) + (x49 * x28);
	const GEN_FLT x52 = 1.0 / 2.0 * x37;
	const GEN_FLT x53 = x52 * x40 * (1. / (x42 * sqrt(x42)));
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
	const GEN_FLT x72 = x71 * (1. / sqrt(x33));
	const GEN_FLT x73 = -1 * x72 * x37;
	const GEN_FLT x74 = atan2(-1 * x30, x24);
	const GEN_FLT x75 = x74 + (-1 * asin(x73)) + ogeeMag_1;
	const GEN_FLT x76 = (sin(x75) * ogeePhase_1) + curve_1;
	const GEN_FLT x77 = x70 * x76;
	const GEN_FLT x78 = x39 + (x77 * x69);
	const GEN_FLT x79 = 1. / x78;
	const GEN_FLT x80 = x44 * x44;
	const GEN_FLT x81 = x80 * x76;
	const GEN_FLT x82 = x81 * x79;
	const GEN_FLT x83 = x71 * x52 * (1. / (x33 * sqrt(x33)));
	const GEN_FLT x84 = (x83 * x51) + (-1 * x72 * x47);
	const GEN_FLT x85 = 1. / sqrt(1 + (-1 * (x71 * x71) * x41 * x34));
	const GEN_FLT x86 = (-1 * x84 * x85) + x36;
	const GEN_FLT x87 = cos(x75) * ogeePhase_1;
	const GEN_FLT x88 = x80 * x79 * x63;
	const GEN_FLT x89 = x88 * x87;
	const GEN_FLT x90 = 2 * x79 * x76 * x64;
	const GEN_FLT x91 = 2.40324066e-05 * x44;
	const GEN_FLT x92 = x70 * x69;
	const GEN_FLT x93 = x87 * x92;
	const GEN_FLT x94 = x81 * (1. / (x78 * x78)) * x63;
	const GEN_FLT x95 = x73 + (x82 * x63);
	const GEN_FLT x96 = 1. / sqrt(1 + (-1 * (x95 * x95)));
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
	const GEN_FLT x102 = -2 * (obj_qi * obj_qi);
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
	const GEN_FLT x23 = x22 * x22;
	const GEN_FLT x24 = x23 + (x17 * x17);
	const GEN_FLT x25 = 1. / x24;
	const GEN_FLT x26 = x25 * x17;
	const GEN_FLT x27 = x7 + (2 * ((x12 * lh_qk) + (-1 * x21 * lh_qi))) + lh_py;
	const GEN_FLT x28 = x22 * x27;
	const GEN_FLT x29 = 0.523598775598299 + (-1 * tilt_1);
	const GEN_FLT x30 = tan(x29);
	const GEN_FLT x31 = x30 * (1. / (x24 * sqrt(x24)));
	const GEN_FLT x32 = x31 * x28;
	const GEN_FLT x33 = cos(x29);
	const GEN_FLT x34 = 1. / x33;
	const GEN_FLT x35 = x27 * x27;
	const GEN_FLT x36 = x24 + x35;
	const GEN_FLT x37 = x34 * (1. / sqrt(x36));
	const GEN_FLT x38 = asin(x37 * x27);
	const GEN_FLT x39 = 8.0108022e-06 * x38;
	const GEN_FLT x40 = 1. / sqrt(1 + (-1 * (1. / (x33 * x33)) * (1. / x36) * x35));
	const GEN_FLT x41 = x34 * (1. / (x36 * sqrt(x36)));
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
	const GEN_FLT x59 = x30 * (1. / sqrt(x24));
	const GEN_FLT x60 = -1 * x59 * x27;
	const GEN_FLT x61 = atan2(-1 * x17, x22);
	const GEN_FLT x62 = x61 + (-1 * asin(x60)) + ogeeMag_1;
	const GEN_FLT x63 = (sin(x62) * ogeePhase_1) + curve_1;
	const GEN_FLT x64 = x63 * x58;
	const GEN_FLT x65 = x33 + (x64 * x57);
	const GEN_FLT x66 = 1. / x65;
	const GEN_FLT x67 = x38 * x38;
	const GEN_FLT x68 = x63 * x67;
	const GEN_FLT x69 = x68 * x66;
	const GEN_FLT x70 = 1. / sqrt(1 + (-1 * (x30 * x30) * x35 * x25));
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
	const GEN_FLT x83 = x68 * (1. / (x65 * x65)) * x51;
	const GEN_FLT x84 = x60 + (x69 * x51);
	const GEN_FLT x85 = 1. / sqrt(1 + (-1 * (x84 * x84)));
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
	const GEN_FLT x107 = 1. / x22;
	const GEN_FLT x108 = 2 * x19;
	const GEN_FLT x109 = (2 * x20) + (-1 * x108);
	const GEN_FLT x110 = 2 * x8;
	const GEN_FLT x111 = (2 * x11) + (-1 * x110);
	const GEN_FLT x112 = (1. / x23) * x17;
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
	const GEN_FLT x2 = x1 * x1;
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
	const GEN_FLT x15 = (x14 * x14) + (x13 * x13);
	const GEN_FLT x16 = (1. / sqrt(x15)) * x11;
	const GEN_FLT x17 = (1 + x2) * x16;
	const GEN_FLT x18 = cos(x0);
	const GEN_FLT x19 = x11 * x11;
	const GEN_FLT x20 = x15 + x19;
	const GEN_FLT x21 = (1. / sqrt(x20)) * x11;
	const GEN_FLT x22 = asin(x21 * (1. / x18));
	const GEN_FLT x23 = 8.0108022e-06 * x22;
	const GEN_FLT x24 = sin(x0);
	const GEN_FLT x25 = 1. / (x18 * x18);
	const GEN_FLT x26 = x25 * x21 * (1. / sqrt(1 + (-1 * x25 * (1. / x20) * x19)));
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
	const GEN_FLT x39 = x22 * x22;
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
	const GEN_FLT x50 = 1. / x49;
	const GEN_FLT x51 = x50 * x39;
	const GEN_FLT x52 = x51 * x40;
	const GEN_FLT x53 = cos(x36) * ogeePhase_1;
	const GEN_FLT x54 = x53 * x52;
	const GEN_FLT x55 = x17 * (1. / sqrt(1 + (-1 * x2 * (1. / x15) * x19)));
	const GEN_FLT x56 = x40 * (1. / (x49 * x49)) * x39;
	const GEN_FLT x57 = x34 + (x52 * x38);
	const GEN_FLT x58 = 1. / sqrt(1 + (-1 * (x57 * x57)));
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
	const GEN_FLT x12 = x10 * x10;
	const GEN_FLT x13 = x4 + (2 * ((x6 * lh_qj) + (-1 * x9 * lh_qk))) + lh_px;
	const GEN_FLT x14 = atan2(x13, x11);
	const GEN_FLT x15 = (-1 * x14) + (-1 * phase_0) + (-1 * asin(x8 * (1. / sqrt((x13 * x13) + x12)) * tilt_0));
	const GEN_FLT x16 =
		(-1 * atan2(-1 * x8, x11)) + (-1 * asin(x13 * (1. / sqrt((x8 * x8) + x12)) * tilt_1)) + (-1 * phase_1);
	out[0] =
		x15 + (-1 * cos(1.5707963267949 + x15 + gibPhase_0) * gibMag_0) + ((atan2(x8, x11) * atan2(x8, x11)) * curve_0);
	out[1] = x16 + ((x14 * x14) * curve_1) + (-1 * cos(1.5707963267949 + x16 + gibPhase_1) * gibMag_1);
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
	const GEN_FLT x25 = x24 * x24;
	const GEN_FLT x26 = 1. / x25;
	const GEN_FLT x27 = x22 * x26;
	const GEN_FLT x28 = -2 * (lh_qk * lh_qk);
	const GEN_FLT x29 = -2 * (lh_qj * lh_qj);
	const GEN_FLT x30 = 1 + x29 + x28;
	const GEN_FLT x31 = 1. / x24;
	const GEN_FLT x32 = x22 * x22;
	const GEN_FLT x33 = x25 + x32;
	const GEN_FLT x34 = 1. / x33;
	const GEN_FLT x35 = x34 * x25;
	const GEN_FLT x36 = x35 * ((-1 * x30 * x31) + (x4 * x27));
	const GEN_FLT x37 = x0 * lh_qi;
	const GEN_FLT x38 = 2 * lh_qw;
	const GEN_FLT x39 = x38 * lh_qk;
	const GEN_FLT x40 = x39 + x37;
	const GEN_FLT x41 = (1. / sqrt(x33)) * tilt_0;
	const GEN_FLT x42 = 2 * x22;
	const GEN_FLT x43 = 2 * x24;
	const GEN_FLT x44 = x4 * x43;
	const GEN_FLT x45 = x13 + (2 * ((x23 * lh_qk) + (-1 * x21 * lh_qi))) + lh_py;
	const GEN_FLT x46 = 1.0 / 2.0 * x45 * (1. / (x33 * sqrt(x33))) * tilt_0;
	const GEN_FLT x47 = x45 * x45;
	const GEN_FLT x48 = 1. / sqrt(1 + (-1 * x47 * x34 * (tilt_0 * tilt_0)));
	const GEN_FLT x49 = (-1 * x48 * ((-1 * x46 * (x44 + (x42 * x30))) + (x40 * x41))) + (-1 * x36);
	const GEN_FLT x50 = -1 * x24;
	const GEN_FLT x51 = atan2(x22, x50);
	const GEN_FLT x52 =
		sin(1.5707963267949 + (-1 * x51) + (-1 * phase_0) + (-1 * asin(x41 * x45)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x53 = x45 * x26;
	const GEN_FLT x54 = x4 * x53;
	const GEN_FLT x55 = x40 * x31;
	const GEN_FLT x56 = x25 + x47;
	const GEN_FLT x57 = 1. / x56;
	const GEN_FLT x58 = x57 * x25;
	const GEN_FLT x59 = 2 * x58 * atan2(x45, x50) * curve_0;
	const GEN_FLT x60 = x0 * lh_qk;
	const GEN_FLT x61 = x38 * lh_qi;
	const GEN_FLT x62 = x61 + x60;
	const GEN_FLT x63 = x37 + (-1 * x39);
	const GEN_FLT x64 = ((-1 * x63 * x31) + (x62 * x27)) * x35;
	const GEN_FLT x65 = 1 + (-2 * (lh_qi * lh_qi));
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
	const GEN_FLT x150 = 1. / sqrt(1 + (-1 * x57 * x32 * (tilt_1 * tilt_1)));
	const GEN_FLT x151 = (1. / sqrt(x56)) * tilt_1;
	const GEN_FLT x152 = 2 * x45;
	const GEN_FLT x153 = 1.0 / 2.0 * (1. / (x56 * sqrt(x56))) * x22 * tilt_1;
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
	const GEN_FLT x8 = -2 * (obj_qk * obj_qk);
	const GEN_FLT x9 = -2 * (obj_qj * obj_qj);
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
	const GEN_FLT x25 = x24 * x24;
	const GEN_FLT x26 = 1. / x25;
	const GEN_FLT x27 = (-1 * x18 * lh_qj) + (x20 * lh_qi) + (x21 * lh_qw);
	const GEN_FLT x28 = x18 + (2 * ((x27 * lh_qj) + (-1 * x23 * lh_qk))) + lh_px;
	const GEN_FLT x29 = x28 * x26;
	const GEN_FLT x30 = 2 * lh_qk;
	const GEN_FLT x31 = (x4 * lh_qw) + (-1 * x10 * lh_qj) + (x7 * lh_qi);
	const GEN_FLT x32 = x10 + (x31 * x12) + (-1 * x30 * x13);
	const GEN_FLT x33 = 1. / x24;
	const GEN_FLT x34 = x28 * x28;
	const GEN_FLT x35 = x25 + x34;
	const GEN_FLT x36 = 1. / x35;
	const GEN_FLT x37 = x36 * x25;
	const GEN_FLT x38 = ((-1 * x32 * x33) + (x29 * x15)) * x37;
	const GEN_FLT x39 = x20 + (2 * ((x22 * lh_qk) + (-1 * x27 * lh_qi))) + lh_py;
	const GEN_FLT x40 = x39 * x39;
	const GEN_FLT x41 = 1. / sqrt(1 + (-1 * x40 * x36 * (tilt_0 * tilt_0)));
	const GEN_FLT x42 = x7 + (x30 * x11) + (-1 * x31 * x14);
	const GEN_FLT x43 = (1. / sqrt(x35)) * tilt_0;
	const GEN_FLT x44 = 2 * x28;
	const GEN_FLT x45 = 2 * x24;
	const GEN_FLT x46 = x45 * x15;
	const GEN_FLT x47 = 1.0 / 2.0 * (1. / (x35 * sqrt(x35))) * x39 * tilt_0;
	const GEN_FLT x48 = (-1 * x41 * ((-1 * x47 * (x46 + (x44 * x32))) + (x42 * x43))) + (-1 * x38);
	const GEN_FLT x49 = -1 * x24;
	const GEN_FLT x50 = atan2(x28, x49);
	const GEN_FLT x51 =
		sin(1.5707963267949 + (-1 * x50) + (-1 * phase_0) + (-1 * asin(x43 * x39)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x52 = x39 * x26;
	const GEN_FLT x53 = x52 * x15;
	const GEN_FLT x54 = x42 * x33;
	const GEN_FLT x55 = x25 + x40;
	const GEN_FLT x56 = 1. / x55;
	const GEN_FLT x57 = x56 * x25;
	const GEN_FLT x58 = 2 * x57 * atan2(x39, x49) * curve_0;
	const GEN_FLT x59 = x2 * obj_qj;
	const GEN_FLT x60 = 2 * obj_qw * obj_qi;
	const GEN_FLT x61 = x60 + x59;
	const GEN_FLT x62 = 1 + (-2 * (obj_qi * obj_qi));
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
	const GEN_FLT x91 = 1. / sqrt(1 + (-1 * x56 * x34 * (tilt_1 * tilt_1)));
	const GEN_FLT x92 = (1. / sqrt(x55)) * tilt_1;
	const GEN_FLT x93 = 2 * x39;
	const GEN_FLT x94 = 1.0 / 2.0 * (1. / (x55 * sqrt(x55))) * x28 * tilt_1;
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
	const GEN_FLT x18 = x17 * x17;
	const GEN_FLT x19 = x10 * lh_qw;
	const GEN_FLT x20 = x3 * lh_qk;
	const GEN_FLT x21 = x7 * lh_qj;
	const GEN_FLT x22 = x21 + (-1 * x20) + x19;
	const GEN_FLT x23 = x7 + (2 * ((x12 * lh_qi) + (-1 * x22 * lh_qj))) + lh_pz;
	const GEN_FLT x24 = x23 * x23;
	const GEN_FLT x25 = x24 + x18;
	const GEN_FLT x26 = 1. / x25;
	const GEN_FLT x27 = x23 * x26;
	const GEN_FLT x28 = x3 + (2 * ((x22 * lh_qk) + (-1 * x16 * lh_qi))) + lh_py;
	const GEN_FLT x29 = x28 * x28;
	const GEN_FLT x30 = 1. / sqrt(1 + (-1 * x29 * x26 * (tilt_0 * tilt_0)));
	const GEN_FLT x31 = (1. / (x25 * sqrt(x25))) * x28 * tilt_0;
	const GEN_FLT x32 = x30 * x31;
	const GEN_FLT x33 = (x32 * x17) + x27;
	const GEN_FLT x34 = (1. / sqrt(x25)) * tilt_0;
	const GEN_FLT x35 = -1 * x23;
	const GEN_FLT x36 = atan2(x17, x35);
	const GEN_FLT x37 =
		sin(1.5707963267949 + (-1 * x36) + (-1 * phase_0) + (-1 * asin(x34 * x28)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x38 = x30 * x34;
	const GEN_FLT x39 = x24 + x29;
	const GEN_FLT x40 = 1. / x39;
	const GEN_FLT x41 = x40 * x23;
	const GEN_FLT x42 = 2 * atan2(x28, x35) * curve_0;
	const GEN_FLT x43 = x26 * x17;
	const GEN_FLT x44 = (x32 * x23) + (-1 * x43);
	const GEN_FLT x45 = x40 * x28;
	const GEN_FLT x46 = 2 * x14;
	const GEN_FLT x47 = (2 * x15) + (-1 * x46);
	const GEN_FLT x48 = 1. / x24;
	const GEN_FLT x49 = x48 * x17;
	const GEN_FLT x50 = 2 * x20;
	const GEN_FLT x51 = (2 * x21) + (-1 * x50);
	const GEN_FLT x52 = 1. / x23;
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
	const GEN_FLT x100 = 1. / sqrt(1 + (-1 * x40 * x18 * (tilt_1 * tilt_1)));
	const GEN_FLT x101 = (1. / sqrt(x39)) * tilt_1;
	const GEN_FLT x102 = x101 * x100;
	const GEN_FLT x103 = 2 * x36 * curve_1;
	const GEN_FLT x104 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x28, x35)) + (-1 * asin(x17 * x101)) + (-1 * phase_1) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x105 = (1. / (x39 * sqrt(x39))) * x17 * tilt_1;
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
	const GEN_FLT x9 = x8 * x8;
	const GEN_FLT x10 = (x4 * lh_qi) + (-1 * x2 * lh_qj) + (x5 * lh_qw);
	const GEN_FLT x11 = x2 + (2 * ((x10 * lh_qj) + (-1 * x7 * lh_qk))) + lh_px;
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = x12 + x9;
	const GEN_FLT x14 = (2 * ((x6 * lh_qk) + (-1 * x10 * lh_qi))) + x4 + lh_py;
	const GEN_FLT x15 = x14 * (1. / sqrt(x13));
	const GEN_FLT x16 = -1 * x8;
	const GEN_FLT x17 = atan2(x11, x16);
	const GEN_FLT x18 = 1.5707963267949 + (-1 * x17) + (-1 * phase_0) + (-1 * asin(x15 * tilt_0)) + gibPhase_0;
	const GEN_FLT x19 = sin(x18) * gibMag_0;
	const GEN_FLT x20 = x14 * x14;
	const GEN_FLT x21 = x15 * (1. / sqrt(1 + (-1 * x20 * (1. / x13) * (tilt_0 * tilt_0))));
	const GEN_FLT x22 = x20 + x9;
	const GEN_FLT x23 = (1. / sqrt(x22)) * x11;
	const GEN_FLT x24 =
		1.5707963267949 + (-1 * atan2(-1 * x14, x16)) + (-1 * asin(x23 * tilt_1)) + (-1 * phase_1) + gibPhase_1;
	const GEN_FLT x25 = sin(x24) * gibMag_1;
	const GEN_FLT x26 = x23 * (1. / sqrt(1 + (-1 * (1. / x22) * x12 * (tilt_1 * tilt_1))));
	out[0] = -1 + (-1 * x19);
	out[1] = (-1 * x21) + (-1 * x21 * x19);
	out[2] = atan2(x14, x16) * atan2(x14, x16);
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
	out[23] = x17 * x17;
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
	const GEN_FLT x13 =
		(-1 * atan2(x12, x11)) + (-1 * phase_0) + (-1 * asin((1. / sqrt((x12 * x12) + (x10 * x10))) * x8 * tilt_0));
	return x13 + (-1 * cos(1.5707963267949 + x13 + gibPhase_0) * gibMag_0) +
		   ((atan2(x8, x11) * atan2(x8, x11)) * curve_0);
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
	const GEN_FLT x23 = x22 * x22;
	const GEN_FLT x24 = 1. / x23;
	const GEN_FLT x25 = (x18 * lh_qi) + (-1 * x13 * lh_qj) + (x19 * lh_qw);
	const GEN_FLT x26 = x13 + (2 * ((x25 * lh_qj) + (-1 * x21 * lh_qk))) + lh_px;
	const GEN_FLT x27 = x24 * x26;
	const GEN_FLT x28 = -2 * (lh_qk * lh_qk);
	const GEN_FLT x29 = -2 * (lh_qj * lh_qj);
	const GEN_FLT x30 = 1 + x29 + x28;
	const GEN_FLT x31 = 1. / x22;
	const GEN_FLT x32 = x23 + (x26 * x26);
	const GEN_FLT x33 = 1. / x32;
	const GEN_FLT x34 = x33 * x23;
	const GEN_FLT x35 = 2 * lh_qi;
	const GEN_FLT x36 = x35 * lh_qj;
	const GEN_FLT x37 = x2 * lh_qw;
	const GEN_FLT x38 = x37 + x36;
	const GEN_FLT x39 = (1. / sqrt(x32)) * tilt_0;
	const GEN_FLT x40 = 2 * x26;
	const GEN_FLT x41 = 2 * x22;
	const GEN_FLT x42 = x18 + (2 * ((x20 * lh_qk) + (-1 * x25 * lh_qi))) + lh_py;
	const GEN_FLT x43 = 1.0 / 2.0 * x42 * (1. / (x32 * sqrt(x32))) * tilt_0;
	const GEN_FLT x44 = x42 * x42;
	const GEN_FLT x45 = 1. / sqrt(1 + (-1 * x44 * x33 * (tilt_0 * tilt_0)));
	const GEN_FLT x46 = (-1 * x45 * ((-1 * x43 * ((x4 * x41) + (x40 * x30))) + (x38 * x39))) +
						(-1 * x34 * ((-1 * x30 * x31) + (x4 * x27)));
	const GEN_FLT x47 = -1 * x22;
	const GEN_FLT x48 =
		sin(1.5707963267949 + (-1 * atan2(x26, x47)) + (-1 * phase_0) + (-1 * asin(x42 * x39)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x49 = x42 * x24;
	const GEN_FLT x50 = 2 * (1. / (x23 + x44)) * x23 * atan2(x42, x47) * curve_0;
	const GEN_FLT x51 = x2 * lh_qj;
	const GEN_FLT x52 = x35 * lh_qw;
	const GEN_FLT x53 = x52 + x51;
	const GEN_FLT x54 = x53 * x24;
	const GEN_FLT x55 = x36 + (-1 * x37);
	const GEN_FLT x56 = 1 + (-2 * (lh_qi * lh_qi));
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
	const GEN_FLT x17 = -2 * (obj_qj * obj_qj);
	const GEN_FLT x18 = 1 + (-2 * (obj_qk * obj_qk));
	const GEN_FLT x19 = x18 + x17;
	const GEN_FLT x20 = (x19 * lh_qw) + (-1 * x16 * lh_qk) + (x13 * lh_qj);
	const GEN_FLT x21 = 2 * lh_qj;
	const GEN_FLT x22 = (x16 * lh_qw) + (-1 * x13 * lh_qi) + (x19 * lh_qk);
	const GEN_FLT x23 = 2 * lh_qi;
	const GEN_FLT x24 = x13 + (x22 * x23) + (-1 * x20 * x21);
	const GEN_FLT x25 = (x4 * lh_qj) + (-1 * x2 * lh_qk) + (x5 * lh_qw);
	const GEN_FLT x26 = x4 + (2 * ((x6 * lh_qi) + (-1 * x25 * lh_qj))) + lh_pz;
	const GEN_FLT x27 = x26 * x26;
	const GEN_FLT x28 = 1. / x27;
	const GEN_FLT x29 = x24 * x28;
	const GEN_FLT x30 = 2 * lh_qk;
	const GEN_FLT x31 = (x13 * lh_qw) + (-1 * x19 * lh_qj) + (x16 * lh_qi);
	const GEN_FLT x32 = x19 + (x31 * x21) + (-1 * x30 * x22);
	const GEN_FLT x33 = 1. / x26;
	const GEN_FLT x34 = x27 + (x8 * x8);
	const GEN_FLT x35 = 1. / x34;
	const GEN_FLT x36 = x35 * x27;
	const GEN_FLT x37 = x2 + (2 * ((x25 * lh_qk) + (-1 * x7 * lh_qi))) + lh_py;
	const GEN_FLT x38 = x37 * x37;
	const GEN_FLT x39 = 1. / sqrt(1 + (-1 * x35 * x38 * (tilt_0 * tilt_0)));
	const GEN_FLT x40 = x16 + (x30 * x20) + (-1 * x31 * x23);
	const GEN_FLT x41 = (1. / sqrt(x34)) * tilt_0;
	const GEN_FLT x42 = 2 * x8;
	const GEN_FLT x43 = 2 * x26;
	const GEN_FLT x44 = 1.0 / 2.0 * (1. / (x34 * sqrt(x34))) * x37 * tilt_0;
	const GEN_FLT x45 = (-1 * x39 * ((-1 * ((x43 * x24) + (x42 * x32)) * x44) + (x40 * x41))) +
						(-1 * x36 * ((-1 * x32 * x33) + (x8 * x29)));
	const GEN_FLT x46 = -1 * x26;
	const GEN_FLT x47 =
		sin(1.5707963267949 + (-1 * atan2(x8, x46)) + (-1 * phase_0) + (-1 * asin(x41 * x37)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x48 = 2 * (1. / (x27 + x38)) * x27 * atan2(x37, x46) * curve_0;
	const GEN_FLT x49 = 2 * obj_qk * obj_qj;
	const GEN_FLT x50 = x9 * obj_qi;
	const GEN_FLT x51 = x50 + x49;
	const GEN_FLT x52 = -2 * (obj_qi * obj_qi);
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
	const GEN_FLT x23 = x22 * x22;
	const GEN_FLT x24 = x23 + (x17 * x17);
	const GEN_FLT x25 = 1. / x24;
	const GEN_FLT x26 = x3 + (2 * ((x21 * lh_qk) + (-1 * x16 * lh_qi))) + lh_py;
	const GEN_FLT x27 = x26 * x26;
	const GEN_FLT x28 = 1. / sqrt(1 + (-1 * x25 * x27 * (tilt_0 * tilt_0)));
	const GEN_FLT x29 = (1. / (x24 * sqrt(x24))) * x26 * tilt_0;
	const GEN_FLT x30 = x28 * x29;
	const GEN_FLT x31 = (x30 * x17) + (x25 * x22);
	const GEN_FLT x32 = (1. / sqrt(x24)) * tilt_0;
	const GEN_FLT x33 = -1 * x22;
	const GEN_FLT x34 =
		sin(1.5707963267949 + (-1 * atan2(x17, x33)) + (-1 * phase_0) + (-1 * asin(x32 * x26)) + gibPhase_0) * gibMag_0;
	const GEN_FLT x35 = x32 * x28;
	const GEN_FLT x36 = 2 * x22;
	const GEN_FLT x37 = (1. / (x23 + x27)) * atan2(x26, x33) * curve_0;
	const GEN_FLT x38 = (x30 * x22) + (-1 * x25 * x17);
	const GEN_FLT x39 = 2 * x37;
	const GEN_FLT x40 = 2 * x14;
	const GEN_FLT x41 = (2 * x15) + (-1 * x40);
	const GEN_FLT x42 = 1. / x23;
	const GEN_FLT x43 = x42 * x17;
	const GEN_FLT x44 = 2 * x19;
	const GEN_FLT x45 = (2 * x20) + (-1 * x44);
	const GEN_FLT x46 = 1. / x22;
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
	const GEN_FLT x11 = (x10 * x10) + (x8 * x8);
	const GEN_FLT x12 = (2 * ((x6 * lh_qk) + (-1 * x9 * lh_qi))) + x4 + lh_py;
	const GEN_FLT x13 = x12 * (1. / sqrt(x11));
	const GEN_FLT x14 = -1 * x8;
	const GEN_FLT x15 =
		1.5707963267949 + (-1 * atan2(x10, x14)) + (-1 * phase_0) + (-1 * asin(x13 * tilt_0)) + gibPhase_0;
	const GEN_FLT x16 = sin(x15) * gibMag_0;
	const GEN_FLT x17 = x13 * (1. / sqrt(1 + (-1 * (x12 * x12) * (1. / x11) * (tilt_0 * tilt_0))));
	out[0] = -1 + (-1 * x16);
	out[1] = (-1 * x17) + (-1 * x17 * x16);
	out[2] = atan2(x12, x14) * atan2(x12, x14);
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
	const GEN_FLT x13 =
		(-1 * atan2(-1 * x10, x12)) + (-1 * asin(x11 * (1. / sqrt((x10 * x10) + (x8 * x8))) * tilt_1)) + (-1 * phase_1);
	return x13 + ((atan2(x11, x12) * atan2(x11, x12)) * curve_1) +
		   (-1 * cos(1.5707963267949 + x13 + gibPhase_1) * gibMag_1);
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
	const GEN_FLT x25 = x24 * x24;
	const GEN_FLT x26 = 1. / x25;
	const GEN_FLT x27 = x22 * x26;
	const GEN_FLT x28 = -2 * (lh_qk * lh_qk);
	const GEN_FLT x29 = -2 * (lh_qj * lh_qj);
	const GEN_FLT x30 = 1 + x29 + x28;
	const GEN_FLT x31 = 1. / x24;
	const GEN_FLT x32 = x22 * x22;
	const GEN_FLT x33 = -1 * x24;
	const GEN_FLT x34 = 2 * (1. / (x25 + x32)) * x25 * atan2(x22, x33) * curve_1;
	const GEN_FLT x35 = x13 + (2 * ((x23 * lh_qk) + (-1 * x21 * lh_qi))) + lh_py;
	const GEN_FLT x36 = x35 * x26;
	const GEN_FLT x37 = x0 * lh_qi;
	const GEN_FLT x38 = x2 * lh_qw;
	const GEN_FLT x39 = x38 + x37;
	const GEN_FLT x40 = x25 + (x35 * x35);
	const GEN_FLT x41 = 1. / x40;
	const GEN_FLT x42 = x41 * x25;
	const GEN_FLT x43 = 1. / sqrt(1 + (-1 * x41 * x32 * (tilt_1 * tilt_1)));
	const GEN_FLT x44 = (1. / sqrt(x40)) * tilt_1;
	const GEN_FLT x45 = 2 * x35;
	const GEN_FLT x46 = 2 * x24;
	const GEN_FLT x47 = 1.0 / 2.0 * (1. / (x40 * sqrt(x40))) * x22 * tilt_1;
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
	const GEN_FLT x55 = 1 + (-2 * (lh_qi * lh_qi));
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
	const GEN_FLT x17 = -2 * (obj_qk * obj_qk);
	const GEN_FLT x18 = 1 + (-2 * (obj_qj * obj_qj));
	const GEN_FLT x19 = x18 + x17;
	const GEN_FLT x20 = (x19 * lh_qw) + (-1 * x16 * lh_qk) + (x13 * lh_qj);
	const GEN_FLT x21 = 2 * lh_qj;
	const GEN_FLT x22 = (x16 * lh_qw) + (-1 * x13 * lh_qi) + (x19 * lh_qk);
	const GEN_FLT x23 = 2 * lh_qi;
	const GEN_FLT x24 = x13 + (x22 * x23) + (-1 * x20 * x21);
	const GEN_FLT x25 = (x4 * lh_qj) + (-1 * x2 * lh_qk) + (x5 * lh_qw);
	const GEN_FLT x26 = x4 + (2 * ((x6 * lh_qi) + (-1 * x25 * lh_qj))) + lh_pz;
	const GEN_FLT x27 = x26 * x26;
	const GEN_FLT x28 = 1. / x27;
	const GEN_FLT x29 = x24 * x28;
	const GEN_FLT x30 = 2 * lh_qk;
	const GEN_FLT x31 = (x13 * lh_qw) + (-1 * x19 * lh_qj) + (x16 * lh_qi);
	const GEN_FLT x32 = x19 + (x31 * x21) + (-1 * x30 * x22);
	const GEN_FLT x33 = 1. / x26;
	const GEN_FLT x34 = x8 * x8;
	const GEN_FLT x35 = -1 * x26;
	const GEN_FLT x36 = 2 * (1. / (x27 + x34)) * x27 * atan2(x8, x35) * curve_1;
	const GEN_FLT x37 = x2 + (2 * ((x25 * lh_qk) + (-1 * x7 * lh_qi))) + lh_py;
	const GEN_FLT x38 = x16 + (x30 * x20) + (-1 * x31 * x23);
	const GEN_FLT x39 = x27 + (x37 * x37);
	const GEN_FLT x40 = 1. / x39;
	const GEN_FLT x41 = x40 * x27;
	const GEN_FLT x42 = 1. / sqrt(1 + (-1 * x40 * x34 * (tilt_1 * tilt_1)));
	const GEN_FLT x43 = (1. / sqrt(x39)) * tilt_1;
	const GEN_FLT x44 = 2 * x37;
	const GEN_FLT x45 = 2 * x26;
	const GEN_FLT x46 = 1.0 / 2.0 * x8 * (1. / (x39 * sqrt(x39))) * tilt_1;
	const GEN_FLT x47 = (-1 * x42 * ((-1 * ((x45 * x24) + (x44 * x38)) * x46) + (x43 * x32))) +
						(-1 * ((x33 * x38) + (-1 * x37 * x29)) * x41);
	const GEN_FLT x48 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x37, x35)) + (-1 * asin(x8 * x43)) + (-1 * phase_1) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x49 = 2 * obj_qk * obj_qj;
	const GEN_FLT x50 = x9 * obj_qi;
	const GEN_FLT x51 = x50 + x49;
	const GEN_FLT x52 = -2 * (obj_qi * obj_qi);
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
	const GEN_FLT x18 = x17 * x17;
	const GEN_FLT x19 = x10 * lh_qw;
	const GEN_FLT x20 = x3 * lh_qj;
	const GEN_FLT x21 = x7 * lh_qi;
	const GEN_FLT x22 = x21 + (-1 * x20) + x19;
	const GEN_FLT x23 = x7 + (2 * ((x12 * lh_qk) + (-1 * x22 * lh_qi))) + lh_py;
	const GEN_FLT x24 = (x23 * x23) + x18;
	const GEN_FLT x25 = 1. / x24;
	const GEN_FLT x26 = x3 + (2 * ((x22 * lh_qj) + (-1 * x16 * lh_qk))) + lh_px;
	const GEN_FLT x27 = x26 * x26;
	const GEN_FLT x28 = 1. / sqrt(1 + (-1 * x25 * x27 * (tilt_1 * tilt_1)));
	const GEN_FLT x29 = (1. / sqrt(x24)) * tilt_1;
	const GEN_FLT x30 = x28 * x29;
	const GEN_FLT x31 = 2 * x17;
	const GEN_FLT x32 = -1 * x17;
	const GEN_FLT x33 = (1. / (x18 + x27)) * atan2(x26, x32) * curve_1;
	const GEN_FLT x34 =
		sin(1.5707963267949 + (-1 * atan2(-1 * x23, x32)) + (-1 * asin(x29 * x26)) + (-1 * phase_1) + gibPhase_1) *
		gibMag_1;
	const GEN_FLT x35 = (1. / (x24 * sqrt(x24))) * x26 * tilt_1;
	const GEN_FLT x36 = x35 * x28;
	const GEN_FLT x37 = (x36 * x23) + (-1 * x25 * x17);
	const GEN_FLT x38 = 2 * x33;
	const GEN_FLT x39 = (x36 * x17) + (x25 * x23);
	const GEN_FLT x40 = 2 * x20;
	const GEN_FLT x41 = (2 * x21) + (-1 * x40);
	const GEN_FLT x42 = 1. / x18;
	const GEN_FLT x43 = x42 * x26;
	const GEN_FLT x44 = 2 * x8;
	const GEN_FLT x45 = (2 * x11) + (-1 * x44);
	const GEN_FLT x46 = 1. / x17;
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
	const GEN_FLT x11 = (x10 * x10) + (x8 * x8);
	const GEN_FLT x12 = x2 + (2 * ((x9 * lh_qj) + (-1 * x7 * lh_qk))) + lh_px;
	const GEN_FLT x13 = x12 * (1. / sqrt(x11));
	const GEN_FLT x14 = -1 * x8;
	const GEN_FLT x15 =
		1.5707963267949 + (-1 * atan2(-1 * x10, x14)) + (-1 * asin(x13 * tilt_1)) + (-1 * phase_1) + gibPhase_1;
	const GEN_FLT x16 = sin(x15) * gibMag_1;
	const GEN_FLT x17 = x13 * (1. / sqrt(1 + (-1 * (x12 * x12) * (1. / x11) * (tilt_1 * tilt_1))));
	out[0] = -1 + (-1 * x16);
	out[1] = (-1 * x17 * x16) + (-1 * x17);
	out[2] = atan2(x12, x14) * atan2(x12, x14);
	out[3] = x16;
	out[4] = -1 * cos(x15);
	out[5] = 0;
	out[6] = 0;
}
