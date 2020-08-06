#pragma once
#include "common.h"
/** Applying function <function reproject_gen2 at 0x7effc26e3b90> */
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
	const GEN_FLT x0 = 0.523598775598299 + tilt_0;
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x3 = sin(x2);
	const GEN_FLT x4 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x5 = x4 * x3;
	const GEN_FLT x6 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x7 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x8 = cos(x2);
	const GEN_FLT x9 = 1 - x8;
	const GEN_FLT x10 = x7 * x9;
	const GEN_FLT x11 = x6 * x10;
	const GEN_FLT x12 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1e-10));
	const GEN_FLT x13 = cos(x12);
	const GEN_FLT x14 = 1 - x13;
	const GEN_FLT x15 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x16 = sin(x12);
	const GEN_FLT x17 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x20 = x15 * x14 * x19;
	const GEN_FLT x21 = x19 * x16;
	const GEN_FLT x22 = x14 * x17;
	const GEN_FLT x23 = x22 * x15;
	const GEN_FLT x24 =
		obj_pz + (x13 + pow(x15, 2) * x14) * sensor_z + (x18 + x20) * sensor_y + (-x21 + x23) * sensor_x;
	const GEN_FLT x25 = x15 * x16;
	const GEN_FLT x26 = x22 * x19;
	const GEN_FLT x27 =
		obj_py + (x13 + x14 * pow(x19, 2)) * sensor_y + (-x18 + x20) * sensor_z + (x25 + x26) * sensor_x;
	const GEN_FLT x28 =
		obj_px + (x13 + x14 * pow(x17, 2)) * sensor_x + (x21 + x23) * sensor_z + (-x25 + x26) * sensor_y;
	const GEN_FLT x29 = x3 * x6;
	const GEN_FLT x30 = x4 * x10;
	const GEN_FLT x31 = lh_py + x24 * (x11 - x5) + x27 * (x8 + pow(x7, 2) * x9) + (x29 + x30) * x28;
	const GEN_FLT x32 = x3 * x7;
	const GEN_FLT x33 = x4 * x6 * x9;
	const GEN_FLT x34 = lh_pz + x24 * (x8 + pow(x6, 2) * x9) + x27 * (x11 + x5) + (-x32 + x33) * x28;
	const GEN_FLT x35 = lh_px + x28 * (x8 + pow(x4, 2) * x9) + (-x29 + x30) * x27 + (x32 + x33) * x24;
	const GEN_FLT x36 = pow(x34, 2) + pow(x35, 2);
	const GEN_FLT x37 = x31 / sqrt(x36 + pow(x31, 2));
	const GEN_FLT x38 = asin(x37 / x1);
	const GEN_FLT x39 = 0.0028679863 + x38 * (-8.0108022e-06 - 8.0108022e-06 * x38);
	const GEN_FLT x40 = 5.3685255e-06 + x38 * x39;
	const GEN_FLT x41 = 0.0076069798 + x40 * x38;
	const GEN_FLT x42 = x31 / sqrt(x36);
	const GEN_FLT x43 = tan(x0) * x42;
	const GEN_FLT x44 = atan2(-x34, x35);
	const GEN_FLT x45 = curve_0 + sin(ogeeMag_0 + x44 - asin(x43)) * ogeePhase_0;
	const GEN_FLT x46 = asin(
		x43 + x41 * x45 * pow(x38, 2) /
				  (x1 - sin(x0) * x45 *
							(x38 * (x41 + x38 * (x40 + x38 * (x39 + x38 * (-8.0108022e-06 - 1.60216044e-05 * x38)))) +
							 x41 * x38)));
	const GEN_FLT x47 = -x44;
	const GEN_FLT x48 = -1.5707963267949 + x44;
	const GEN_FLT x49 = 0.523598775598299 - tilt_1;
	const GEN_FLT x50 = cos(x49);
	const GEN_FLT x51 = asin(x37 / x50);
	const GEN_FLT x52 = 0.0028679863 + x51 * (-8.0108022e-06 - 8.0108022e-06 * x51);
	const GEN_FLT x53 = 5.3685255e-06 + x52 * x51;
	const GEN_FLT x54 = 0.0076069798 + x53 * x51;
	const GEN_FLT x55 = -x42 * tan(x49);
	const GEN_FLT x56 = curve_1 + sin(ogeeMag_1 + x44 - asin(x55)) * ogeePhase_1;
	const GEN_FLT x57 = asin(
		x55 + x54 * pow(x51, 2) * x56 /
				  (x50 + x56 * sin(x49) *
							 (x51 * (x54 + x51 * (x53 + x51 * (x52 + x51 * (-8.0108022e-06 - 1.60216044e-05 * x51)))) +
							  x54 * x51)));
	out[0] = -phase_0 - x46 + x48 - sin(-gibPhase_0 + x46 + x47) * gibMag_0;
	out[1] = -phase_1 + x48 - x57 - sin(-gibPhase_1 + x47 + x57) * gibMag_1;
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 - x1;
	const GEN_FLT x3 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x4 = pow(x3, 2);
	const GEN_FLT x5 = x1 + x2 * x4;
	const GEN_FLT x6 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x7 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1e-10));
	const GEN_FLT x8 = sin(x7);
	const GEN_FLT x9 = x6 * x8;
	const GEN_FLT x10 = -x9;
	const GEN_FLT x11 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x12 = pow(x11, 2);
	const GEN_FLT x13 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x14 = cos(x7);
	const GEN_FLT x15 = 1 - x14;
	const GEN_FLT x16 = x15 * x11;
	const GEN_FLT x17 = 2 * x16;
	const GEN_FLT x18 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x19 = x6 * x14;
	const GEN_FLT x20 = x19 * x18;
	const GEN_FLT x21 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x22 = x8 * x21;
	const GEN_FLT x23 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x24 = x8 * x11;
	const GEN_FLT x25 = x24 * x23;
	const GEN_FLT x26 = x23 * x15;
	const GEN_FLT x27 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x28 = x27 * x15;
	const GEN_FLT x29 = x26 * x13 + x28 * x11 + x6 * x25;
	const GEN_FLT x30 = x23 * x19;
	const GEN_FLT x31 = x8 * x27;
	const GEN_FLT x32 = x24 * x18;
	const GEN_FLT x33 = x21 * x15;
	const GEN_FLT x34 = x15 * x18;
	const GEN_FLT x35 = x33 * x11 + x34 * x13 + x6 * x32;
	const GEN_FLT x36 =
		(x10 + x13 * x17 + x9 * x12) * sensor_x + (-x20 - x22 + x29) * sensor_y + (x30 + x31 + x35) * sensor_z;
	const GEN_FLT x37 = 1 + x36;
	const GEN_FLT x38 = sin(x0);
	const GEN_FLT x39 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x40 = x38 * x39;
	const GEN_FLT x41 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x42 = x2 * x41;
	const GEN_FLT x43 = x3 * x42;
	const GEN_FLT x44 = -x40 + x43;
	const GEN_FLT x45 = x11 * x19;
	const GEN_FLT x46 = x8 * x13;
	const GEN_FLT x47 = x8 * x18;
	const GEN_FLT x48 = x47 * x23;
	const GEN_FLT x49 = x28 * x18 + x33 * x23 + x6 * x48;
	const GEN_FLT x50 = pow(x23, 2);
	const GEN_FLT x51 =
		(x10 + 2 * x23 * x28 + x9 * x50) * sensor_y + (x20 + x22 + x29) * sensor_x + (-x45 - x46 + x49) * sensor_z;
	const GEN_FLT x52 = x51 * x44;
	const GEN_FLT x53 = pow(x18, 2);
	const GEN_FLT x54 = 2 * x18;
	const GEN_FLT x55 =
		(x10 + x54 * x33 + x9 * x53) * sensor_z + (-x30 - x31 + x35) * sensor_x + (x45 + x46 + x49) * sensor_y;
	const GEN_FLT x56 = x41 * x38;
	const GEN_FLT x57 = x2 * x39;
	const GEN_FLT x58 = x3 * x57;
	const GEN_FLT x59 = x56 + x58;
	const GEN_FLT x60 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x61 = x60 * x38;
	const GEN_FLT x62 = -x61;
	const GEN_FLT x63 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x64 = x8 * x23;
	const GEN_FLT x65 = x34 * x11;
	const GEN_FLT x66 = x26 * x11;
	const GEN_FLT x67 = obj_px + (x14 + x15 * x12) * sensor_x + (x64 + x65) * sensor_z + (-x47 + x66) * sensor_y;
	const GEN_FLT x68 = x26 * x18;
	const GEN_FLT x69 = obj_py + (x14 + x50 * x15) * sensor_y + (x47 + x66) * sensor_x + (-x24 + x68) * sensor_z;
	const GEN_FLT x70 = x1 * x60;
	const GEN_FLT x71 = x70 * x39;
	const GEN_FLT x72 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x73 = x72 * x38;
	const GEN_FLT x74 = x3 * x61;
	const GEN_FLT x75 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x76 = x2 * x75;
	const GEN_FLT x77 = x3 * x76 + x63 * x42 + x74 * x41;
	const GEN_FLT x78 = obj_pz + (x14 + x53 * x15) * sensor_z + (-x64 + x65) * sensor_x + (x24 + x68) * sensor_y;
	const GEN_FLT x79 = x70 * x41;
	const GEN_FLT x80 = x75 * x38;
	const GEN_FLT x81 = x2 * x72;
	const GEN_FLT x82 = x3 * x81 + x63 * x57 + x74 * x39;
	const GEN_FLT x83 = x67 * (x62 + x4 * x61 + 2 * x2 * x3 * x63) + x69 * (-x71 - x73 + x77) + x78 * (x79 + x80 + x82);
	const GEN_FLT x84 = x83 + x55 * x59;
	const GEN_FLT x85 = x52 + x84 + x5 * x37;
	const GEN_FLT x86 = pow(x39, 2);
	const GEN_FLT x87 = x1 + x2 * x86;
	const GEN_FLT x88 = x3 * x38;
	const GEN_FLT x89 = x42 * x39;
	const GEN_FLT x90 = x88 + x89;
	const GEN_FLT x91 = -x56 + x58;
	const GEN_FLT x92 = lh_pz + x67 * x91 + x69 * x90 + x87 * x78;
	const GEN_FLT x93 = lh_px + x5 * x67 + x69 * x44 + x78 * x59;
	const GEN_FLT x94 = pow(x93, 2);
	const GEN_FLT x95 = x92 / x94;
	const GEN_FLT x96 = x51 * x90;
	const GEN_FLT x97 = x3 * x70;
	const GEN_FLT x98 = x63 * x38;
	const GEN_FLT x99 = x76 * x39 + x81 * x41 + x61 * x41 * x39;
	const GEN_FLT x100 = x67 * (-x79 - x80 + x82) + x69 * (x97 + x98 + x99) + x78 * (x62 + 2 * x81 * x39 + x86 * x61);
	const GEN_FLT x101 = x100 + x87 * x55;
	const GEN_FLT x102 = x101 + x96 + x91 * x37;
	const GEN_FLT x103 = pow(x93, -1);
	const GEN_FLT x104 = x94 + pow(x92, 2);
	const GEN_FLT x105 = pow(x104, -1);
	const GEN_FLT x106 = x94 * x105;
	const GEN_FLT x107 = x106 * (-x103 * x102 + x85 * x95);
	const GEN_FLT x108 = pow(x41, 2);
	const GEN_FLT x109 = x1 + x2 * x108;
	const GEN_FLT x110 = x51 * x109;
	const GEN_FLT x111 = -x88 + x89;
	const GEN_FLT x112 = x55 * x111;
	const GEN_FLT x113 = x40 + x43;
	const GEN_FLT x114 = x67 * (x71 + x73 + x77) + x69 * (x62 + x61 * x108 + 2 * x76 * x41) + x78 * (-x97 - x98 + x99);
	const GEN_FLT x115 = x110 + x112 + x114 + x37 * x113;
	const GEN_FLT x116 = lh_py + x67 * x113 + x69 * x109 + x78 * x111;
	const GEN_FLT x117 = pow(x116, 2);
	const GEN_FLT x118 = x104 + x117;
	const GEN_FLT x119 = pow(x118, -1.0 / 2.0);
	const GEN_FLT x120 = 0.523598775598299 + tilt_0;
	const GEN_FLT x121 = cos(x120);
	const GEN_FLT x122 = pow(x121, -1);
	const GEN_FLT x123 = x119 * x122;
	const GEN_FLT x124 = 2 * x116;
	const GEN_FLT x125 = 2 * x93;
	const GEN_FLT x126 = 2 * x92;
	const GEN_FLT x127 = x102 * x126 + x85 * x125;
	const GEN_FLT x128 = (1.0 / 2.0) * x116;
	const GEN_FLT x129 = x128 / pow(x118, 3.0 / 2.0);
	const GEN_FLT x130 = x129 * (x127 + x115 * x124);
	const GEN_FLT x131 = x115 * x123 - x122 * x130;
	const GEN_FLT x132 = x119 * x116;
	const GEN_FLT x133 = asin(x122 * x132);
	const GEN_FLT x134 = -8.0108022e-06 - 1.60216044e-05 * x133;
	const GEN_FLT x135 = 8.0108022e-06 * x133;
	const GEN_FLT x136 = -8.0108022e-06 - x135;
	const GEN_FLT x137 = 0.0028679863 + x133 * x136;
	const GEN_FLT x138 = x137 + x133 * x134;
	const GEN_FLT x139 = 5.3685255e-06 + x133 * x137;
	const GEN_FLT x140 = x139 + x133 * x138;
	const GEN_FLT x141 = 0.0076069798 + x133 * x139;
	const GEN_FLT x142 = x141 + x133 * x140;
	const GEN_FLT x143 = x117 / x118;
	const GEN_FLT x144 = pow(1 - x143 / pow(x121, 2), -1.0 / 2.0);
	const GEN_FLT x145 = x142 * x144;
	const GEN_FLT x146 = x131 * x144;
	const GEN_FLT x147 = x136 * x146;
	const GEN_FLT x148 = 2.40324066e-05 * x133;
	const GEN_FLT x149 = x133 * (x147 - x135 * x146) + x137 * x146;
	const GEN_FLT x150 = x133 * x149 + x139 * x146;
	const GEN_FLT x151 = sin(x120);
	const GEN_FLT x152 = tan(x120);
	const GEN_FLT x153 = pow(x104, -1.0 / 2.0);
	const GEN_FLT x154 = x116 * x153;
	const GEN_FLT x155 = x152 * x154;
	const GEN_FLT x156 = atan2(-x92, x93);
	const GEN_FLT x157 = ogeeMag_0 + x156 - asin(x155);
	const GEN_FLT x158 = curve_0 + sin(x157) * ogeePhase_0;
	const GEN_FLT x159 = x151 * x158;
	const GEN_FLT x160 = x105 * x117;
	const GEN_FLT x161 = pow(1 - x160 * pow(x152, 2), -1.0 / 2.0);
	const GEN_FLT x162 = x128 / pow(x104, 3.0 / 2.0);
	const GEN_FLT x163 = x162 * x152;
	const GEN_FLT x164 = x115 * x153;
	const GEN_FLT x165 = -x127 * x163 + x164 * x152;
	const GEN_FLT x166 = x107 - x161 * x165;
	const GEN_FLT x167 = cos(x157) * ogeePhase_0;
	const GEN_FLT x168 = x133 * x141;
	const GEN_FLT x169 = x168 + x133 * x142;
	const GEN_FLT x170 = x169 * x151;
	const GEN_FLT x171 = x167 * x170;
	const GEN_FLT x172 = x121 - x169 * x159;
	const GEN_FLT x173 = pow(x133, 2);
	const GEN_FLT x174 = x173 * x158;
	const GEN_FLT x175 = x174 * x141 / pow(x172, 2);
	const GEN_FLT x176 = pow(x172, -1);
	const GEN_FLT x177 = 2 * x168 * x176 * x158;
	const GEN_FLT x178 = x176 * x174;
	const GEN_FLT x179 = x176 * x173 * x141;
	const GEN_FLT x180 = x167 * x179;
	const GEN_FLT x181 = x155 + x178 * x141;
	const GEN_FLT x182 = pow(1 - pow(x181, 2), -1.0 / 2.0);
	const GEN_FLT x183 =
		x182 *
		(x165 + x166 * x180 -
		 x175 * (-x159 * (x131 * x145 + x133 * x150 +
						  x133 * (x150 + x133 * (x149 + x133 * (x147 + x134 * x146 - x146 * x148) + x138 * x146) +
								  x140 * x146) +
						  x141 * x146) -
				 x166 * x171) +
		 x177 * x146 + x178 * x150);
	const GEN_FLT x184 = -x107;
	const GEN_FLT x185 = -x156;
	const GEN_FLT x186 = cos(-gibPhase_0 + x185 + asin(x181)) * gibMag_0;
	const GEN_FLT x187 = x5 * x36;
	const GEN_FLT x188 = 1 + x51;
	const GEN_FLT x189 = x187 + x84 + x44 * x188;
	const GEN_FLT x190 = x91 * x36;
	const GEN_FLT x191 = x101 + x190 + x90 * x188;
	const GEN_FLT x192 = x106 * (-x103 * x191 + x95 * x189);
	const GEN_FLT x193 = x114 + x36 * x113;
	const GEN_FLT x194 = x112 + x193 + x109 * x188;
	const GEN_FLT x195 = x125 * x189 + x126 * x191;
	const GEN_FLT x196 = x195 + x124 * x194;
	const GEN_FLT x197 = x122 * x129;
	const GEN_FLT x198 = (x123 * x194 - x196 * x197) * x144;
	const GEN_FLT x199 = x198 * x136;
	const GEN_FLT x200 = x133 * (x199 - x198 * x135) + x198 * x137;
	const GEN_FLT x201 = x198 * x139 + x200 * x133;
	const GEN_FLT x202 = x162 * x195;
	const GEN_FLT x203 = x194 * x153;
	const GEN_FLT x204 = -x202 * x152 + x203 * x152;
	const GEN_FLT x205 = x167 * (x192 - x204 * x161);
	const GEN_FLT x206 =
		x182 *
		(x204 -
		 x175 * (-x159 * (x133 * (x201 + x133 * (x200 + x133 * (x199 + x198 * x134 - x198 * x148) + x198 * x138) +
								  x198 * x140) +
						  x198 * x141 + x198 * x142 + x201 * x133) -
				 x205 * x170) +
		 x177 * x198 + x201 * x178 + x205 * x179);
	const GEN_FLT x207 = -x192;
	const GEN_FLT x208 = 1 + x55;
	const GEN_FLT x209 = x187 + x52 + x83 + x59 * x208;
	const GEN_FLT x210 = x100 + x190 + x96 + x87 * x208;
	const GEN_FLT x211 = x106 * (-x210 * x103 + x95 * x209);
	const GEN_FLT x212 = x110 + x193 + x208 * x111;
	const GEN_FLT x213 = x209 * x125 + x210 * x126;
	const GEN_FLT x214 = x213 + x212 * x124;
	const GEN_FLT x215 = x212 * x123 - x214 * x197;
	const GEN_FLT x216 = x215 * x144;
	const GEN_FLT x217 = x216 * x136;
	const GEN_FLT x218 = x133 * (x217 - x216 * x135) + x216 * x137;
	const GEN_FLT x219 = x139 * x144;
	const GEN_FLT x220 = x218 * x133 + x219 * x215;
	const GEN_FLT x221 = x212 * x153;
	const GEN_FLT x222 = -x213 * x163 + x221 * x152;
	const GEN_FLT x223 = x211 - x222 * x161;
	const GEN_FLT x224 =
		x182 *
		(x222 -
		 x175 * (-x159 * (x133 * (x220 + x133 * (x218 + x133 * (x217 + x216 * x134 - x216 * x148) + x216 * x138) +
								  x216 * x140) +
						  x215 * x145 + x216 * x141 + x220 * x133) -
				 x223 * x171) +
		 x216 * x177 + x220 * x178 + x223 * x180);
	const GEN_FLT x225 = -x211;
	const GEN_FLT x226 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x227 = x8 * x226;
	const GEN_FLT x228 = -x227;
	const GEN_FLT x229 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x230 = x14 * x226;
	const GEN_FLT x231 = x18 * x230;
	const GEN_FLT x232 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x233 = x8 * x232;
	const GEN_FLT x234 = x23 * x11;
	const GEN_FLT x235 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x236 = x16 * x235 + x234 * x227 + x26 * x229;
	const GEN_FLT x237 = x23 * x230;
	const GEN_FLT x238 = x8 * x235;
	const GEN_FLT x239 = x18 * x227;
	const GEN_FLT x240 = x11 * x239 + x16 * x232 + x34 * x229;
	const GEN_FLT x241 = (x228 + x12 * x227 + x17 * x229) * sensor_x + (-x231 - x233 + x236) * sensor_y +
						 (x237 + x238 + x240) * sensor_z;
	const GEN_FLT x242 = x11 * x230;
	const GEN_FLT x243 = x8 * x229;
	const GEN_FLT x244 = x23 * x239 + x26 * x232 + x34 * x235;
	const GEN_FLT x245 = 2 * x26;
	const GEN_FLT x246 = sensor_y * (x228 + x235 * x245 + x50 * x227) + (x231 + x233 + x236) * sensor_x +
						 (-x242 - x243 + x244) * sensor_z;
	const GEN_FLT x247 = 2 * x34;
	const GEN_FLT x248 = sensor_z * (x228 + x232 * x247 + x53 * x227) + (-x237 - x238 + x240) * sensor_x +
						 (x242 + x243 + x244) * sensor_y;
	const GEN_FLT x249 = x114 + x241 * x113 + x246 * x109 + x248 * x111;
	const GEN_FLT x250 = x249 * x119;
	const GEN_FLT x251 = x83 + x44 * x246 + x5 * x241 + x59 * x248;
	const GEN_FLT x252 = x100 + x87 * x248 + x90 * x246 + x91 * x241;
	const GEN_FLT x253 = x251 * x125 + x252 * x126;
	const GEN_FLT x254 = x129 * (x253 + x249 * x124);
	const GEN_FLT x255 = x250 * x122 - x254 * x122;
	const GEN_FLT x256 = x255 * x144;
	const GEN_FLT x257 = x256 * x136;
	const GEN_FLT x258 = x133 * (x257 - x256 * x135) + x256 * x137;
	const GEN_FLT x259 = x219 * x255 + x258 * x133;
	const GEN_FLT x260 = x106 * (-x252 * x103 + x95 * x251);
	const GEN_FLT x261 = x249 * x153;
	const GEN_FLT x262 = -x253 * x163 + x261 * x152;
	const GEN_FLT x263 = x260 - x262 * x161;
	const GEN_FLT x264 =
		x182 *
		(x262 -
		 x175 * (-x159 * (x133 * (x259 + x133 * (x258 + x133 * (x257 + x256 * x134 - x256 * x148) + x256 * x138) +
								  x256 * x140) +
						  x255 * x145 + x256 * x141 + x259 * x133) -
				 x263 * x171) +
		 x256 * x177 + x259 * x178 + x263 * x180);
	const GEN_FLT x265 = -x260;
	const GEN_FLT x266 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x267 = x8 * x266;
	const GEN_FLT x268 = -x267;
	const GEN_FLT x269 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x270 = x14 * x266;
	const GEN_FLT x271 = x18 * x270;
	const GEN_FLT x272 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x273 = x8 * x272;
	const GEN_FLT x274 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x275 = x16 * x274 + x234 * x267 + x26 * x269;
	const GEN_FLT x276 = x23 * x270;
	const GEN_FLT x277 = x8 * x274;
	const GEN_FLT x278 = x18 * x267;
	const GEN_FLT x279 = x11 * x278 + x16 * x272 + x34 * x269;
	const GEN_FLT x280 = (x268 + x12 * x267 + x17 * x269) * sensor_x + (-x271 - x273 + x275) * sensor_y +
						 (x276 + x277 + x279) * sensor_z;
	const GEN_FLT x281 = x11 * x270;
	const GEN_FLT x282 = x8 * x269;
	const GEN_FLT x283 = x23 * x278 + x26 * x272 + x34 * x274;
	const GEN_FLT x284 = sensor_y * (x268 + x274 * x245 + x50 * x267) + (x271 + x273 + x275) * sensor_x +
						 (-x281 - x282 + x283) * sensor_z;
	const GEN_FLT x285 = sensor_z * (x268 + x272 * x247 + x53 * x267) + (-x276 - x277 + x279) * sensor_x +
						 (x281 + x282 + x283) * sensor_y;
	const GEN_FLT x286 = x83 + x44 * x284 + x5 * x280 + x59 * x285;
	const GEN_FLT x287 = x100 + x87 * x285 + x90 * x284 + x91 * x280;
	const GEN_FLT x288 = x106 * (-x287 * x103 + x95 * x286);
	const GEN_FLT x289 = x114 + x280 * x113 + x284 * x109 + x285 * x111;
	const GEN_FLT x290 = x289 * x119;
	const GEN_FLT x291 = x286 * x125 + x287 * x126;
	const GEN_FLT x292 = x291 + x289 * x124;
	const GEN_FLT x293 = x290 * x122 - x292 * x197;
	const GEN_FLT x294 = x293 * x144;
	const GEN_FLT x295 = x294 * x136;
	const GEN_FLT x296 = x133 * (x295 - x294 * x135) + x294 * x137;
	const GEN_FLT x297 = x294 * x139 + x296 * x133;
	const GEN_FLT x298 = x289 * x153;
	const GEN_FLT x299 = -x291 * x163 + x298 * x152;
	const GEN_FLT x300 = x288 - x299 * x161;
	const GEN_FLT x301 =
		x182 *
		(x299 -
		 x175 * (-x159 * (x133 * (x297 + x133 * (x296 + x133 * (x295 + x294 * x134 - x294 * x148) + x294 * x138) +
								  x294 * x140) +
						  x293 * x145 + x294 * x141 + x297 * x133) -
				 x300 * x171) +
		 x294 * x177 + x297 * x178 + x300 * x180);
	const GEN_FLT x302 = -x288;
	const GEN_FLT x303 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x304 = x14 * x303;
	const GEN_FLT x305 = x18 * x304;
	const GEN_FLT x306 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x307 = x8 * x306;
	const GEN_FLT x308 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x309 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x310 = x16 * x309 + x25 * x303 + x26 * x308;
	const GEN_FLT x311 = x11 * x304;
	const GEN_FLT x312 = x8 * x308;
	const GEN_FLT x313 = x15 * x306;
	const GEN_FLT x314 = x23 * x313 + x34 * x309 + x48 * x303;
	const GEN_FLT x315 = x8 * x303;
	const GEN_FLT x316 = -x315;
	const GEN_FLT x317 = sensor_y * (x316 + x245 * x309 + x50 * x315) + (x305 + x307 + x310) * sensor_x +
						 (-x311 - x312 + x314) * sensor_z;
	const GEN_FLT x318 = x23 * x304;
	const GEN_FLT x319 = x8 * x309;
	const GEN_FLT x320 = x11 * x313 + x32 * x303 + x34 * x308;
	const GEN_FLT x321 = (-x305 - x307 + x310) * sensor_y + (x316 + x12 * x315 + x17 * x308) * sensor_x +
						 (x318 + x319 + x320) * sensor_z;
	const GEN_FLT x322 = (x311 + x312 + x314) * sensor_y + (x316 + x53 * x315 + x54 * x313) * sensor_z +
						 (-x318 - x319 + x320) * sensor_x;
	const GEN_FLT x323 = x83 + x44 * x317 + x5 * x321 + x59 * x322;
	const GEN_FLT x324 = x100 + x87 * x322 + x90 * x317 + x91 * x321;
	const GEN_FLT x325 = x106 * (-x324 * x103 + x95 * x323);
	const GEN_FLT x326 = x114 + x317 * x109 + x321 * x113 + x322 * x111;
	const GEN_FLT x327 = x323 * x125 + x324 * x126;
	const GEN_FLT x328 = x129 * (x327 + x326 * x124);
	const GEN_FLT x329 = x326 * x123 - x328 * x122;
	const GEN_FLT x330 = x329 * x144;
	const GEN_FLT x331 = x330 * x136;
	const GEN_FLT x332 = x133 * (x331 - x330 * x135) + x330 * x137;
	const GEN_FLT x333 = x330 * x139 + x332 * x133;
	const GEN_FLT x334 = x327 * x162;
	const GEN_FLT x335 = x326 * x153;
	const GEN_FLT x336 = -x334 * x152 + x335 * x152;
	const GEN_FLT x337 = x325 - x336 * x161;
	const GEN_FLT x338 =
		x182 *
		(x336 -
		 x175 * (-x159 * (x133 * (x333 + x133 * (x332 + x133 * (x331 + x330 * x134 - x330 * x148) + x330 * x138) +
								  x330 * x140) +
						  x329 * x145 + x330 * x141 + x333 * x133) -
				 x337 * x171) +
		 x330 * x177 + x333 * x178 + x337 * x180);
	const GEN_FLT x339 = -x325;
	const GEN_FLT x340 = 0.523598775598299 - tilt_1;
	const GEN_FLT x341 = cos(x340);
	const GEN_FLT x342 = pow(x341, -1);
	const GEN_FLT x343 = asin(x342 * x132);
	const GEN_FLT x344 = 8.0108022e-06 * x343;
	const GEN_FLT x345 = -8.0108022e-06 - x344;
	const GEN_FLT x346 = 0.0028679863 + x345 * x343;
	const GEN_FLT x347 = 5.3685255e-06 + x346 * x343;
	const GEN_FLT x348 = 0.0076069798 + x347 * x343;
	const GEN_FLT x349 = pow(x343, 2);
	const GEN_FLT x350 = tan(x340);
	const GEN_FLT x351 = -x350 * x154;
	const GEN_FLT x352 = ogeeMag_1 + x156 - asin(x351);
	const GEN_FLT x353 = curve_1 + sin(x352) * ogeePhase_1;
	const GEN_FLT x354 = x348 * x343;
	const GEN_FLT x355 = -8.0108022e-06 - 1.60216044e-05 * x343;
	const GEN_FLT x356 = x346 + x355 * x343;
	const GEN_FLT x357 = x347 + x356 * x343;
	const GEN_FLT x358 = x348 + x357 * x343;
	const GEN_FLT x359 = x354 + x358 * x343;
	const GEN_FLT x360 = sin(x340);
	const GEN_FLT x361 = x360 * x353;
	const GEN_FLT x362 = x341 + x361 * x359;
	const GEN_FLT x363 = pow(x362, -1);
	const GEN_FLT x364 = x363 * x353;
	const GEN_FLT x365 = x364 * x349;
	const GEN_FLT x366 = x351 + x365 * x348;
	const GEN_FLT x367 = pow(1 - pow(x366, 2), -1.0 / 2.0);
	const GEN_FLT x368 = x342 * x119;
	const GEN_FLT x369 = -x342 * x130 + x368 * x115;
	const GEN_FLT x370 = pow(1 - x143 / pow(x341, 2), -1.0 / 2.0);
	const GEN_FLT x371 = x370 * x346;
	const GEN_FLT x372 = x369 * x370;
	const GEN_FLT x373 = x370 * x345;
	const GEN_FLT x374 = x373 * x369;
	const GEN_FLT x375 = x343 * (x374 - x372 * x344) + x371 * x369;
	const GEN_FLT x376 = x370 * x347;
	const GEN_FLT x377 = x375 * x343 + x376 * x369;
	const GEN_FLT x378 = 2 * x364 * x354;
	const GEN_FLT x379 = x370 * x358;
	const GEN_FLT x380 = x370 * x357;
	const GEN_FLT x381 = x370 * x356;
	const GEN_FLT x382 = 2.40324066e-05 * x343;
	const GEN_FLT x383 = x370 * x355;
	const GEN_FLT x384 = x370 * x348;
	const GEN_FLT x385 = pow(1 - pow(x350, 2) * x160, -1.0 / 2.0);
	const GEN_FLT x386 = x350 * x162;
	const GEN_FLT x387 = -x350 * x164 + x386 * x127;
	const GEN_FLT x388 = x107 - x387 * x385;
	const GEN_FLT x389 = cos(x352) * ogeePhase_1;
	const GEN_FLT x390 = x360 * x359;
	const GEN_FLT x391 = x390 * x389;
	const GEN_FLT x392 = x348 * x349;
	const GEN_FLT x393 = x392 * x353 / pow(x362, 2);
	const GEN_FLT x394 = x392 * x363;
	const GEN_FLT x395 = x394 * x389;
	const GEN_FLT x396 =
		x367 * (x387 + x372 * x378 + x377 * x365 -
				x393 * (x361 * (x343 * (x377 + x343 * (x375 + x343 * (x374 + x369 * x383 - x372 * x382) + x369 * x381) +
										x369 * x380) +
								x369 * x384 + x377 * x343 + x379 * x369) +
						x391 * x388) +
				x395 * x388);
	const GEN_FLT x397 = cos(-gibPhase_1 + x185 + asin(x366)) * gibMag_1;
	const GEN_FLT x398 = x342 * x129;
	const GEN_FLT x399 = x368 * x194 - x398 * x196;
	const GEN_FLT x400 = x399 * x370;
	const GEN_FLT x401 = x373 * x399;
	const GEN_FLT x402 = x343 * (x401 - x400 * x344) + x371 * x399;
	const GEN_FLT x403 = x376 * x399 + x402 * x343;
	const GEN_FLT x404 = x202 * x350 - x203 * x350;
	const GEN_FLT x405 = x192 - x404 * x385;
	const GEN_FLT x406 =
		x367 * (x404 -
				x393 * (x361 * (x343 * (x403 + x343 * (x402 + x343 * (x401 + x399 * x383 - x400 * x382) + x399 * x381) +
										x399 * x380) +
								x379 * x399 + x399 * x384 + x403 * x343) +
						x405 * x391) +
				x400 * x378 + x403 * x365 + x405 * x395);
	const GEN_FLT x407 = x212 * x368 - x214 * x398;
	const GEN_FLT x408 = x407 * x370;
	const GEN_FLT x409 = x407 * x373;
	const GEN_FLT x410 = x343 * (x409 - x408 * x344) + x407 * x371;
	const GEN_FLT x411 = x407 * x376 + x410 * x343;
	const GEN_FLT x412 = x213 * x386 - x221 * x350;
	const GEN_FLT x413 = x211 - x412 * x385;
	const GEN_FLT x414 =
		x367 * (x412 -
				x393 * (x361 * (x343 * (x411 + x343 * (x410 + x343 * (x409 + x407 * x383 - x408 * x382) + x407 * x381) +
										x407 * x380) +
								x407 * x379 + x407 * x384 + x411 * x343) +
						x413 * x391) +
				x408 * x378 + x411 * x365 + x413 * x395);
	const GEN_FLT x415 = x250 * x342 - x254 * x342;
	const GEN_FLT x416 = x415 * x370;
	const GEN_FLT x417 = x415 * x373;
	const GEN_FLT x418 = x343 * (x417 - x416 * x344) + x416 * x346;
	const GEN_FLT x419 = x415 * x376 + x418 * x343;
	const GEN_FLT x420 = x253 * x386 - x261 * x350;
	const GEN_FLT x421 = x260 - x420 * x385;
	const GEN_FLT x422 =
		x367 * (x420 -
				x393 * (x361 * (x343 * (x419 + x343 * (x418 + x343 * (x417 + x416 * x355 - x416 * x382) + x415 * x381) +
										x415 * x380) +
								x416 * x348 + x416 * x358 + x419 * x343) +
						x421 * x391) +
				x416 * x378 + x419 * x365 + x421 * x395);
	const GEN_FLT x423 = x290 * x342 - x292 * x398;
	const GEN_FLT x424 = x423 * x370;
	const GEN_FLT x425 = x423 * x373;
	const GEN_FLT x426 = x343 * (x425 - x424 * x344) + x424 * x346;
	const GEN_FLT x427 = x423 * x376 + x426 * x343;
	const GEN_FLT x428 = x291 * x386 - x298 * x350;
	const GEN_FLT x429 = x288 - x428 * x385;
	const GEN_FLT x430 =
		x367 * (x428 -
				x393 * (x361 * (x343 * (x427 + x343 * (x426 + x343 * (x425 + x423 * x383 - x424 * x382) + x423 * x381) +
										x423 * x380) +
								x423 * x384 + x424 * x358 + x427 * x343) +
						x429 * x391) +
				x424 * x378 + x427 * x365 + x429 * x395);
	const GEN_FLT x431 = -x328 * x342 + x368 * x326;
	const GEN_FLT x432 = x431 * x370;
	const GEN_FLT x433 = x432 * x345;
	const GEN_FLT x434 = x343 * (x433 - x432 * x344) + x432 * x346;
	const GEN_FLT x435 = x431 * x376 + x434 * x343;
	const GEN_FLT x436 = x350 * x334 - x350 * x335;
	const GEN_FLT x437 = x389 * (x325 - x436 * x385);
	const GEN_FLT x438 =
		x367 * (x436 -
				x393 * (x361 * (x343 * (x435 + x343 * (x434 + x343 * (x433 + x432 * x355 - x432 * x382) + x431 * x381) +
										x431 * x380) +
								x432 * x348 + x432 * x358 + x435 * x343) +
						x437 * x390) +
				x432 * x378 + x435 * x365 + x437 * x394);
	out[0] = x107 - x183 - (x183 + x184) * x186;
	out[1] = x192 - x206 - (x206 + x207) * x186;
	out[2] = x211 - x224 - (x224 + x225) * x186;
	out[3] = x260 - x264 - (x264 + x265) * x186;
	out[4] = x288 - x301 - (x301 + x302) * x186;
	out[5] = x325 - x338 - (x338 + x339) * x186;
	out[6] = x107 - x396 - (x184 + x396) * x397;
	out[7] = x192 - x406 - (x207 + x406) * x397;
	out[8] = x211 - x414 - (x225 + x414) * x397;
	out[9] = x260 - x422 - (x265 + x422) * x397;
	out[10] = x288 - x430 - (x302 + x430) * x397;
	out[11] = x325 - x438 - (x339 + x438) * x397;
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 - x1;
	const GEN_FLT x3 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x4 = pow(x3, 2);
	const GEN_FLT x5 = x1 + x2 * x4;
	const GEN_FLT x6 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1e-10));
	const GEN_FLT x7 = cos(x6);
	const GEN_FLT x8 = 1 - x7;
	const GEN_FLT x9 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								  : (1e-10)))
							? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											 : (1e-10)))
							: (1));
	const GEN_FLT x10 = pow(x9, 2);
	const GEN_FLT x11 = x7 + x8 * x10;
	const GEN_FLT x12 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x13 = sin(x6);
	const GEN_FLT x14 = x13 * x12;
	const GEN_FLT x15 = -x14;
	const GEN_FLT x16 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x17 = x8 * x9;
	const GEN_FLT x18 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x19 = x7 * x12;
	const GEN_FLT x20 = x19 * x18;
	const GEN_FLT x21 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x22 = x21 * x13;
	const GEN_FLT x23 = x9 * x13;
	const GEN_FLT x24 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x25 = x24 * x12;
	const GEN_FLT x26 = x8 * x24;
	const GEN_FLT x27 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x28 = x25 * x23 + x26 * x16 + x27 * x17;
	const GEN_FLT x29 = x24 * x19;
	const GEN_FLT x30 = x27 * x13;
	const GEN_FLT x31 = x8 * x18;
	const GEN_FLT x32 = x21 * x17 + x31 * x16 + x23 * x12 * x18;
	const GEN_FLT x33 =
		(x15 + x14 * x10 + 2 * x17 * x16) * sensor_x + (-x20 - x22 + x28) * sensor_y + (x29 + x30 + x32) * sensor_z;
	const GEN_FLT x34 = x11 + x33;
	const GEN_FLT x35 = sin(x0);
	const GEN_FLT x36 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x37 = x36 * x35;
	const GEN_FLT x38 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x39 = x2 * x38;
	const GEN_FLT x40 = x3 * x39;
	const GEN_FLT x41 = -x37 + x40;
	const GEN_FLT x42 = x13 * x18;
	const GEN_FLT x43 = x24 * x17;
	const GEN_FLT x44 = x42 + x43;
	const GEN_FLT x45 = x9 * x19;
	const GEN_FLT x46 = x13 * x16;
	const GEN_FLT x47 = x21 * x26 + x31 * x27 + x42 * x25;
	const GEN_FLT x48 = pow(x24, 2);
	const GEN_FLT x49 =
		(x15 + 2 * x26 * x27 + x48 * x14) * sensor_y + (x20 + x22 + x28) * sensor_x + (-x45 - x46 + x47) * sensor_z;
	const GEN_FLT x50 = x44 + x49;
	const GEN_FLT x51 = x24 * x13;
	const GEN_FLT x52 = x18 * x17;
	const GEN_FLT x53 = -x51 + x52;
	const GEN_FLT x54 = pow(x18, 2);
	const GEN_FLT x55 =
		(x15 + 2 * x31 * x21 + x54 * x14) * sensor_z + (-x29 - x30 + x32) * sensor_x + (x45 + x46 + x47) * sensor_y;
	const GEN_FLT x56 = x53 + x55;
	const GEN_FLT x57 = x35 * x38;
	const GEN_FLT x58 = x2 * x3;
	const GEN_FLT x59 = x58 * x36;
	const GEN_FLT x60 = x57 + x59;
	const GEN_FLT x61 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x62 = x61 * x35;
	const GEN_FLT x63 = -x62;
	const GEN_FLT x64 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x65 = x51 + x52;
	const GEN_FLT x66 = -x42 + x43;
	const GEN_FLT x67 = obj_px + x11 * sensor_x + x65 * sensor_z + x66 * sensor_y;
	const GEN_FLT x68 = x7 + x8 * x48;
	const GEN_FLT x69 = x31 * x24;
	const GEN_FLT x70 = -x23 + x69;
	const GEN_FLT x71 = obj_py + x44 * sensor_x + x68 * sensor_y + x70 * sensor_z;
	const GEN_FLT x72 = x1 * x61;
	const GEN_FLT x73 = x72 * x36;
	const GEN_FLT x74 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x75 = x74 * x35;
	const GEN_FLT x76 = x3 * x62;
	const GEN_FLT x77 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x78 = x2 * x77;
	const GEN_FLT x79 = x3 * x78 + x64 * x39 + x76 * x38;
	const GEN_FLT x80 = x7 + x8 * x54;
	const GEN_FLT x81 = x23 + x69;
	const GEN_FLT x82 = obj_pz + x53 * sensor_x + x80 * sensor_z + x81 * sensor_y;
	const GEN_FLT x83 = x72 * x38;
	const GEN_FLT x84 = x77 * x35;
	const GEN_FLT x85 = x2 * x36;
	const GEN_FLT x86 = x74 * x58 + x76 * x36 + x85 * x64;
	const GEN_FLT x87 = x67 * (x63 + x4 * x62 + 2 * x64 * x58) + x71 * (-x73 - x75 + x79) + x82 * (x83 + x84 + x86);
	const GEN_FLT x88 = x87 + x5 * x34 + x50 * x41 + x60 * x56;
	const GEN_FLT x89 = pow(x36, 2);
	const GEN_FLT x90 = x1 + x2 * x89;
	const GEN_FLT x91 = x3 * x35;
	const GEN_FLT x92 = x36 * x39;
	const GEN_FLT x93 = x91 + x92;
	const GEN_FLT x94 = -x57 + x59;
	const GEN_FLT x95 = lh_pz + x67 * x94 + x71 * x93 + x82 * x90;
	const GEN_FLT x96 = lh_px + x5 * x67 + x71 * x41 + x82 * x60;
	const GEN_FLT x97 = pow(x96, 2);
	const GEN_FLT x98 = x95 / x97;
	const GEN_FLT x99 = pow(x96, -1);
	const GEN_FLT x100 = x3 * x72;
	const GEN_FLT x101 = x64 * x35;
	const GEN_FLT x102 = x74 * x39 + x78 * x36 + x62 * x36 * x38;
	const GEN_FLT x103 =
		x67 * (-x83 - x84 + x86) + x71 * (x100 + x101 + x102) + x82 * (x63 + 2 * x85 * x74 + x89 * x62);
	const GEN_FLT x104 = x103 + x50 * x93 + x56 * x90 + x94 * x34;
	const GEN_FLT x105 = x97 + pow(x95, 2);
	const GEN_FLT x106 = pow(x105, -1);
	const GEN_FLT x107 = x97 * x106;
	const GEN_FLT x108 = x107 * (x88 * x98 - x99 * x104);
	const GEN_FLT x109 = 0.523598775598299 + tilt_0;
	const GEN_FLT x110 = cos(x109);
	const GEN_FLT x111 = pow(x110, -1);
	const GEN_FLT x112 = -x91 + x92;
	const GEN_FLT x113 = pow(x38, 2);
	const GEN_FLT x114 = x1 + x2 * x113;
	const GEN_FLT x115 = x37 + x40;
	const GEN_FLT x116 = lh_py + x67 * x115 + x71 * x114 + x82 * x112;
	const GEN_FLT x117 = pow(x116, 2);
	const GEN_FLT x118 = x105 + x117;
	const GEN_FLT x119 = pow(x118, -1.0 / 2.0);
	const GEN_FLT x120 = x119 * x116;
	const GEN_FLT x121 = asin(x111 * x120);
	const GEN_FLT x122 = -8.0108022e-06 - 1.60216044e-05 * x121;
	const GEN_FLT x123 = 8.0108022e-06 * x121;
	const GEN_FLT x124 = -8.0108022e-06 - x123;
	const GEN_FLT x125 = 0.0028679863 + x124 * x121;
	const GEN_FLT x126 = x125 + x122 * x121;
	const GEN_FLT x127 = 5.3685255e-06 + x121 * x125;
	const GEN_FLT x128 = x127 + x121 * x126;
	const GEN_FLT x129 = 0.0076069798 + x121 * x127;
	const GEN_FLT x130 = x129 + x121 * x128;
	const GEN_FLT x131 = x117 / x118;
	const GEN_FLT x132 = pow(1 - x131 / pow(x110, 2), -1.0 / 2.0);
	const GEN_FLT x133 =
		x67 * (x73 + x75 + x79) + x71 * (x63 + x62 * x113 + 2 * x78 * x38) + x82 * (-x100 - x101 + x102);
	const GEN_FLT x134 = x133 + x34 * x115 + x50 * x114 + x56 * x112;
	const GEN_FLT x135 = x119 * x134;
	const GEN_FLT x136 = 2 * x116;
	const GEN_FLT x137 = 2 * x96;
	const GEN_FLT x138 = 2 * x95;
	const GEN_FLT x139 = x104 * x138 + x88 * x137;
	const GEN_FLT x140 = x139 + x134 * x136;
	const GEN_FLT x141 = (1.0 / 2.0) * x116;
	const GEN_FLT x142 = x141 / pow(x118, 3.0 / 2.0);
	const GEN_FLT x143 = x111 * x142;
	const GEN_FLT x144 = (x111 * x135 - x140 * x143) * x132;
	const GEN_FLT x145 = x124 * x144;
	const GEN_FLT x146 = 2.40324066e-05 * x121;
	const GEN_FLT x147 = x121 * (x145 - x123 * x144) + x125 * x144;
	const GEN_FLT x148 = x121 * x147 + x127 * x144;
	const GEN_FLT x149 = sin(x109);
	const GEN_FLT x150 = tan(x109);
	const GEN_FLT x151 = pow(x105, -1.0 / 2.0);
	const GEN_FLT x152 = x116 * x151;
	const GEN_FLT x153 = x150 * x152;
	const GEN_FLT x154 = atan2(-x95, x96);
	const GEN_FLT x155 = ogeeMag_0 + x154 - asin(x153);
	const GEN_FLT x156 = curve_0 + sin(x155) * ogeePhase_0;
	const GEN_FLT x157 = x149 * x156;
	const GEN_FLT x158 = x141 / pow(x105, 3.0 / 2.0);
	const GEN_FLT x159 = x139 * x158;
	const GEN_FLT x160 = x134 * x151;
	const GEN_FLT x161 = -x150 * x159 + x160 * x150;
	const GEN_FLT x162 = x106 * x117;
	const GEN_FLT x163 = pow(1 - x162 * pow(x150, 2), -1.0 / 2.0);
	const GEN_FLT x164 = cos(x155) * ogeePhase_0;
	const GEN_FLT x165 = x164 * (x108 - x161 * x163);
	const GEN_FLT x166 = x121 * x129;
	const GEN_FLT x167 = x166 + x121 * x130;
	const GEN_FLT x168 = x167 * x149;
	const GEN_FLT x169 = x110 - x167 * x157;
	const GEN_FLT x170 = pow(x121, 2);
	const GEN_FLT x171 = x170 * x156;
	const GEN_FLT x172 = x129 * x171 / pow(x169, 2);
	const GEN_FLT x173 = pow(x169, -1);
	const GEN_FLT x174 = 2 * x166 * x173 * x156;
	const GEN_FLT x175 = x171 * x173;
	const GEN_FLT x176 = x129 * x170 * x173;
	const GEN_FLT x177 = x153 + x129 * x175;
	const GEN_FLT x178 = pow(1 - pow(x177, 2), -1.0 / 2.0);
	const GEN_FLT x179 =
		x178 *
		(x161 + x165 * x176 -
		 x172 * (-x157 * (x121 * x148 +
						  x121 * (x148 + x121 * (x147 + x121 * (x145 + x122 * x144 - x144 * x146) + x126 * x144) +
								  x128 * x144) +
						  x129 * x144 + x130 * x144) -
				 x168 * x165) +
		 x174 * x144 + x175 * x148);
	const GEN_FLT x180 = -x108;
	const GEN_FLT x181 = -x154;
	const GEN_FLT x182 = cos(-gibPhase_0 + x181 + asin(x177)) * gibMag_0;
	const GEN_FLT x183 = x49 + x68;
	const GEN_FLT x184 = x33 + x66;
	const GEN_FLT x185 = x55 + x81;
	const GEN_FLT x186 = x87 + x41 * x183 + x5 * x184 + x60 * x185;
	const GEN_FLT x187 = x103 + x90 * x185 + x93 * x183 + x94 * x184;
	const GEN_FLT x188 = (x98 * x186 - x99 * x187) * x107;
	const GEN_FLT x189 = x133 + x112 * x185 + x114 * x183 + x115 * x184;
	const GEN_FLT x190 = x119 * x189;
	const GEN_FLT x191 = x186 * x137 + x187 * x138;
	const GEN_FLT x192 = x191 + x189 * x136;
	const GEN_FLT x193 = (x111 * x190 - x192 * x143) * x132;
	const GEN_FLT x194 = x124 * x193;
	const GEN_FLT x195 = x121 * (x194 - x123 * x193) + x125 * x193;
	const GEN_FLT x196 = x121 * x195 + x127 * x193;
	const GEN_FLT x197 = x191 * x158;
	const GEN_FLT x198 = x189 * x151;
	const GEN_FLT x199 = -x197 * x150 + x198 * x150;
	const GEN_FLT x200 = x188 - x163 * x199;
	const GEN_FLT x201 = x168 * x164;
	const GEN_FLT x202 = x164 * x176;
	const GEN_FLT x203 =
		x178 *
		(x199 -
		 x172 * (-x157 * (x121 * x196 +
						  x121 * (x196 + x121 * (x195 + x121 * (x194 + x122 * x193 - x193 * x146) + x126 * x193) +
								  x128 * x193) +
						  x129 * x193 + x193 * x130) -
				 x200 * x201) +
		 x174 * x193 + x175 * x196 + x200 * x202);
	const GEN_FLT x204 = -x188;
	const GEN_FLT x205 = x33 + x65;
	const GEN_FLT x206 = x49 + x70;
	const GEN_FLT x207 = x55 + x80;
	const GEN_FLT x208 = x87 + x41 * x206 + x5 * x205 + x60 * x207;
	const GEN_FLT x209 = x103 + x90 * x207 + x93 * x206 + x94 * x205;
	const GEN_FLT x210 = (x98 * x208 - x99 * x209) * x107;
	const GEN_FLT x211 = x133 + x205 * x115 + x206 * x114 + x207 * x112;
	const GEN_FLT x212 = x211 * x119;
	const GEN_FLT x213 = x208 * x137 + x209 * x138;
	const GEN_FLT x214 = x213 + x211 * x136;
	const GEN_FLT x215 = (x212 * x111 - x214 * x143) * x132;
	const GEN_FLT x216 = x215 * x124;
	const GEN_FLT x217 = x121 * (x216 - x215 * x123) + x215 * x125;
	const GEN_FLT x218 = x215 * x127 + x217 * x121;
	const GEN_FLT x219 = x213 * x158;
	const GEN_FLT x220 = x211 * x151;
	const GEN_FLT x221 = -x219 * x150 + x220 * x150;
	const GEN_FLT x222 = x210 - x221 * x163;
	const GEN_FLT x223 =
		x178 *
		(x221 -
		 x172 * (-x157 * (x121 * (x218 + x121 * (x217 + x121 * (x216 + x215 * x122 - x215 * x146) + x215 * x126) +
								  x215 * x128) +
						  x215 * x129 + x215 * x130 + x218 * x121) -
				 x201 * x222) +
		 x202 * x222 + x215 * x174 + x218 * x175);
	const GEN_FLT x224 = -x210;
	const GEN_FLT x225 = 0.523598775598299 - tilt_1;
	const GEN_FLT x226 = cos(x225);
	const GEN_FLT x227 = pow(x226, -1);
	const GEN_FLT x228 = asin(x227 * x120);
	const GEN_FLT x229 = 8.0108022e-06 * x228;
	const GEN_FLT x230 = -8.0108022e-06 - x229;
	const GEN_FLT x231 = 0.0028679863 + x230 * x228;
	const GEN_FLT x232 = 5.3685255e-06 + x231 * x228;
	const GEN_FLT x233 = 0.0076069798 + x232 * x228;
	const GEN_FLT x234 = pow(x228, 2);
	const GEN_FLT x235 = tan(x225);
	const GEN_FLT x236 = -x235 * x152;
	const GEN_FLT x237 = ogeeMag_1 + x154 - asin(x236);
	const GEN_FLT x238 = curve_1 + sin(x237) * ogeePhase_1;
	const GEN_FLT x239 = x233 * x228;
	const GEN_FLT x240 = -8.0108022e-06 - 1.60216044e-05 * x228;
	const GEN_FLT x241 = x231 + x228 * x240;
	const GEN_FLT x242 = x232 + x228 * x241;
	const GEN_FLT x243 = x233 + x228 * x242;
	const GEN_FLT x244 = x239 + x228 * x243;
	const GEN_FLT x245 = sin(x225);
	const GEN_FLT x246 = x238 * x245;
	const GEN_FLT x247 = x226 + x244 * x246;
	const GEN_FLT x248 = pow(x247, -1);
	const GEN_FLT x249 = x238 * x248;
	const GEN_FLT x250 = x234 * x249;
	const GEN_FLT x251 = x236 + x233 * x250;
	const GEN_FLT x252 = pow(1 - pow(x251, 2), -1.0 / 2.0);
	const GEN_FLT x253 = pow(1 - x131 / pow(x226, 2), -1.0 / 2.0);
	const GEN_FLT x254 = x227 * x142;
	const GEN_FLT x255 = (x227 * x135 - x254 * x140) * x253;
	const GEN_FLT x256 = x230 * x255;
	const GEN_FLT x257 = x228 * (x256 - x255 * x229) + x231 * x255;
	const GEN_FLT x258 = x232 * x255 + x257 * x228;
	const GEN_FLT x259 = 2 * x239 * x249;
	const GEN_FLT x260 = x235 * x159 - x235 * x160;
	const GEN_FLT x261 = pow(1 - pow(x235, 2) * x162, -1.0 / 2.0);
	const GEN_FLT x262 = x108 - x260 * x261;
	const GEN_FLT x263 = cos(x237) * ogeePhase_1;
	const GEN_FLT x264 = x234 * x233;
	const GEN_FLT x265 = x264 * x248;
	const GEN_FLT x266 = x265 * x263;
	const GEN_FLT x267 = 2.40324066e-05 * x228;
	const GEN_FLT x268 = x244 * x245;
	const GEN_FLT x269 = x268 * x263;
	const GEN_FLT x270 = x238 * x264 / pow(x247, 2);
	const GEN_FLT x271 =
		x252 * (x260 + x250 * x258 + x255 * x259 + x266 * x262 -
				x270 * (x246 * (x228 * (x258 + x228 * (x257 + x228 * (x256 + x255 * x240 - x267 * x255) + x255 * x241) +
										x255 * x242) +
								x233 * x255 + x255 * x243 + x258 * x228) +
						x269 * x262));
	const GEN_FLT x272 = cos(-gibPhase_1 + x181 + asin(x251)) * gibMag_1;
	const GEN_FLT x273 = (x227 * x190 - x254 * x192) * x253;
	const GEN_FLT x274 = x230 * x273;
	const GEN_FLT x275 = x228 * (x274 - x273 * x229) + x231 * x273;
	const GEN_FLT x276 = x232 * x273 + x275 * x228;
	const GEN_FLT x277 = x235 * x197 - x235 * x198;
	const GEN_FLT x278 = x188 - x277 * x261;
	const GEN_FLT x279 =
		x252 * (x277 -
				x270 * (x246 * (x228 * (x276 + x228 * (x275 + x228 * (x274 + x273 * x240 - x273 * x267) + x273 * x241) +
										x273 * x242) +
								x233 * x273 + x273 * x243 + x276 * x228) +
						x278 * x269) +
				x273 * x259 + x276 * x250 + x278 * x266);
	const GEN_FLT x280 = (x212 * x227 - x214 * x254) * x253;
	const GEN_FLT x281 = x230 * x280;
	const GEN_FLT x282 = x228 * (x281 - x280 * x229) + x231 * x280;
	const GEN_FLT x283 = x232 * x280 + x282 * x228;
	const GEN_FLT x284 = x219 * x235 - x235 * x220;
	const GEN_FLT x285 = x263 * (x210 - x261 * x284);
	const GEN_FLT x286 =
		x252 * (x284 + x265 * x285 -
				x270 * (x246 * (x228 * (x283 + x228 * (x282 + x228 * (x281 - x267 * x280 + x280 * x240) + x280 * x241) +
										x280 * x242) +
								x233 * x280 + x280 * x243 + x283 * x228) +
						x268 * x285) +
				x280 * x259 + x283 * x250);
	out[0] = x108 - x179 - (x179 + x180) * x182;
	out[1] = x188 - x203 - (x203 + x204) * x182;
	out[2] = x210 - x223 - (x223 + x224) * x182;
	out[3] = x108 - x271 - (x180 + x271) * x272;
	out[4] = x188 - x279 - (x204 + x279) * x272;
	out[5] = x210 - x286 - (x224 + x286) * x272;
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = -x3;
	const GEN_FLT x5 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x6 = pow(x5, 2);
	const GEN_FLT x7 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
							   pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										: (1e-10)),
								   2))
							: (0));
	const GEN_FLT x8 = cos(x0);
	const GEN_FLT x9 = 1 - x8;
	const GEN_FLT x10 = x5 * x9;
	const GEN_FLT x11 = 2 * x10;
	const GEN_FLT x12 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1e-10));
	const GEN_FLT x13 = sin(x12);
	const GEN_FLT x14 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x15 = x14 * x13;
	const GEN_FLT x16 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x17 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x18 = cos(x12);
	const GEN_FLT x19 = 1 - x18;
	const GEN_FLT x20 = x19 * x17;
	const GEN_FLT x21 = x20 * x16;
	const GEN_FLT x22 = x13 * x16;
	const GEN_FLT x23 = x20 * x14;
	const GEN_FLT x24 = pow(x17, 2);
	const GEN_FLT x25 = obj_px + (x18 + x24 * x19) * sensor_x + (x15 + x21) * sensor_z + (-x22 + x23) * sensor_y;
	const GEN_FLT x26 = pow(x14, 2);
	const GEN_FLT x27 = x13 * x17;
	const GEN_FLT x28 = x14 * x19;
	const GEN_FLT x29 = x28 * x16;
	const GEN_FLT x30 = obj_py + (x18 + x26 * x19) * sensor_y + (x22 + x23) * sensor_x + (-x27 + x29) * sensor_z;
	const GEN_FLT x31 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x32 = x2 * x8;
	const GEN_FLT x33 = x32 * x31;
	const GEN_FLT x34 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x35 = x1 * x34;
	const GEN_FLT x36 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x37 = x3 * x5;
	const GEN_FLT x38 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x39 = x9 * x38;
	const GEN_FLT x40 = x9 * x36;
	const GEN_FLT x41 = x36 * x37 + x5 * x39 + x7 * x40;
	const GEN_FLT x42 = pow(x16, 2);
	const GEN_FLT x43 = obj_pz + (x18 + x42 * x19) * sensor_z + (-x15 + x21) * sensor_x + (x27 + x29) * sensor_y;
	const GEN_FLT x44 = x32 * x36;
	const GEN_FLT x45 = x1 * x38;
	const GEN_FLT x46 = x9 * x31;
	const GEN_FLT x47 = x31 * x37 + x34 * x10 + x7 * x46;
	const GEN_FLT x48 = x8 + x6 * x9;
	const GEN_FLT x49 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x50 = x49 * x13;
	const GEN_FLT x51 = -x50;
	const GEN_FLT x52 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x53 = x49 * x18;
	const GEN_FLT x54 = x53 * x16;
	const GEN_FLT x55 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x56 = x55 * x13;
	const GEN_FLT x57 = x49 * x17;
	const GEN_FLT x58 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x59 = x52 * x28 + x57 * x15 + x58 * x20;
	const GEN_FLT x60 = x53 * x14;
	const GEN_FLT x61 = x58 * x13;
	const GEN_FLT x62 = x19 * x16;
	const GEN_FLT x63 = x55 * x20 + x57 * x22 + x62 * x52;
	const GEN_FLT x64 =
		(x51 + x50 * x24 + 2 * x52 * x20) * sensor_x + (-x54 - x56 + x59) * sensor_y + (x60 + x61 + x63) * sensor_z;
	const GEN_FLT x65 = x1 * x31;
	const GEN_FLT x66 = x5 * x40;
	const GEN_FLT x67 = -x65 + x66;
	const GEN_FLT x68 = x53 * x17;
	const GEN_FLT x69 = x52 * x13;
	const GEN_FLT x70 = x55 * x28 + x62 * x58 + x49 * x15 * x16;
	const GEN_FLT x71 =
		(x51 + x50 * x26 + 2 * x58 * x28) * sensor_y + (x54 + x56 + x59) * sensor_x + (-x68 - x69 + x70) * sensor_z;
	const GEN_FLT x72 =
		(x51 + x50 * x42 + 2 * x62 * x55) * sensor_z + (-x60 - x61 + x63) * sensor_x + (x68 + x69 + x70) * sensor_y;
	const GEN_FLT x73 = x1 * x36;
	const GEN_FLT x74 = x31 * x10;
	const GEN_FLT x75 = x73 + x74;
	const GEN_FLT x76 = x64 * x48 + x71 * x67 + x72 * x75;
	const GEN_FLT x77 = x76 + x25 * (x4 + x3 * x6 + x7 * x11) + x30 * (-x33 - x35 + x41) + x43 * (x44 + x45 + x47);
	const GEN_FLT x78 = 1 + x77;
	const GEN_FLT x79 = pow(x31, 2);
	const GEN_FLT x80 = x8 + x9 * x79;
	const GEN_FLT x81 = x1 * x5;
	const GEN_FLT x82 = x40 * x31;
	const GEN_FLT x83 = x81 + x82;
	const GEN_FLT x84 = -x73 + x74;
	const GEN_FLT x85 = lh_pz + x80 * x43 + x83 * x30 + x84 * x25;
	const GEN_FLT x86 = lh_px + x48 * x25 + x67 * x30 + x75 * x43;
	const GEN_FLT x87 = pow(x86, 2);
	const GEN_FLT x88 = x85 / x87;
	const GEN_FLT x89 = x5 * x32;
	const GEN_FLT x90 = x1 * x7;
	const GEN_FLT x91 = x31 * x39 + x40 * x34 + x3 * x31 * x36;
	const GEN_FLT x92 = 2 * x46;
	const GEN_FLT x93 = x80 * x72 + x83 * x71 + x84 * x64;
	const GEN_FLT x94 = x93 + x25 * (-x44 - x45 + x47) + x30 * (x89 + x90 + x91) + x43 * (x4 + x3 * x79 + x92 * x34);
	const GEN_FLT x95 = pow(x86, -1);
	const GEN_FLT x96 = -x95 * x94;
	const GEN_FLT x97 = x87 + pow(x85, 2);
	const GEN_FLT x98 = pow(x97, -1);
	const GEN_FLT x99 = x87 * x98;
	const GEN_FLT x100 = x99 * (x96 + x88 * x78);
	const GEN_FLT x101 = 0.523598775598299 + tilt_0;
	const GEN_FLT x102 = cos(x101);
	const GEN_FLT x103 = pow(x102, -1);
	const GEN_FLT x104 = -x81 + x82;
	const GEN_FLT x105 = pow(x36, 2);
	const GEN_FLT x106 = x8 + x9 * x105;
	const GEN_FLT x107 = x65 + x66;
	const GEN_FLT x108 = lh_py + x25 * x107 + x30 * x106 + x43 * x104;
	const GEN_FLT x109 = pow(x108, 2);
	const GEN_FLT x110 = x109 + x97;
	const GEN_FLT x111 = pow(x110, -1.0 / 2.0);
	const GEN_FLT x112 = x108 * x111;
	const GEN_FLT x113 = asin(x103 * x112);
	const GEN_FLT x114 = -8.0108022e-06 - 1.60216044e-05 * x113;
	const GEN_FLT x115 = 8.0108022e-06 * x113;
	const GEN_FLT x116 = -8.0108022e-06 - x115;
	const GEN_FLT x117 = 0.0028679863 + x113 * x116;
	const GEN_FLT x118 = x117 + x113 * x114;
	const GEN_FLT x119 = 5.3685255e-06 + x113 * x117;
	const GEN_FLT x120 = x119 + x113 * x118;
	const GEN_FLT x121 = 0.0076069798 + x113 * x119;
	const GEN_FLT x122 = x121 + x113 * x120;
	const GEN_FLT x123 = x109 / x110;
	const GEN_FLT x124 = pow(1 - x123 / pow(x102, 2), -1.0 / 2.0);
	const GEN_FLT x125 = 2 * x36;
	const GEN_FLT x126 = x64 * x107 + x71 * x106 + x72 * x104;
	const GEN_FLT x127 =
		x126 + x25 * (x33 + x35 + x41) + x30 * (x4 + x3 * x105 + x39 * x125) + x43 * (-x89 - x90 + x91);
	const GEN_FLT x128 = x111 * x127;
	const GEN_FLT x129 = x103 * x128;
	const GEN_FLT x130 = 2 * x108;
	const GEN_FLT x131 = x127 * x130;
	const GEN_FLT x132 = 2 * x86;
	const GEN_FLT x133 = 2 * x85;
	const GEN_FLT x134 = x94 * x133;
	const GEN_FLT x135 = x134 + x78 * x132;
	const GEN_FLT x136 = (1.0 / 2.0) * x108;
	const GEN_FLT x137 = x136 / pow(x110, 3.0 / 2.0);
	const GEN_FLT x138 = (x131 + x135) * x137;
	const GEN_FLT x139 = x129 - x103 * x138;
	const GEN_FLT x140 = x124 * x139;
	const GEN_FLT x141 = x116 * x140;
	const GEN_FLT x142 = 2.40324066e-05 * x113;
	const GEN_FLT x143 = x114 * x124;
	const GEN_FLT x144 = x113 * (x141 - x115 * x140) + x117 * x140;
	const GEN_FLT x145 = x113 * x144 + x119 * x140;
	const GEN_FLT x146 = sin(x101);
	const GEN_FLT x147 = tan(x101);
	const GEN_FLT x148 = pow(x97, -1.0 / 2.0);
	const GEN_FLT x149 = x108 * x148;
	const GEN_FLT x150 = x147 * x149;
	const GEN_FLT x151 = atan2(-x85, x86);
	const GEN_FLT x152 = ogeeMag_0 + x151 - asin(x150);
	const GEN_FLT x153 = curve_0 + sin(x152) * ogeePhase_0;
	const GEN_FLT x154 = x146 * x153;
	const GEN_FLT x155 = x98 * x109;
	const GEN_FLT x156 = pow(1 - pow(x147, 2) * x155, -1.0 / 2.0);
	const GEN_FLT x157 = x136 / pow(x97, 3.0 / 2.0);
	const GEN_FLT x158 = x147 * x157;
	const GEN_FLT x159 = x127 * x148;
	const GEN_FLT x160 = x147 * x159;
	const GEN_FLT x161 = x160 - x135 * x158;
	const GEN_FLT x162 = x100 - x161 * x156;
	const GEN_FLT x163 = cos(x152) * ogeePhase_0;
	const GEN_FLT x164 = x113 * x121;
	const GEN_FLT x165 = x164 + x113 * x122;
	const GEN_FLT x166 = x165 * x146;
	const GEN_FLT x167 = x166 * x163;
	const GEN_FLT x168 = x102 - x165 * x154;
	const GEN_FLT x169 = pow(x113, 2);
	const GEN_FLT x170 = x169 * x153;
	const GEN_FLT x171 = x121 * x170 / pow(x168, 2);
	const GEN_FLT x172 = pow(x168, -1);
	const GEN_FLT x173 = 2 * x164 * x172 * x153;
	const GEN_FLT x174 = x170 * x172;
	const GEN_FLT x175 = x121 * x169 * x172;
	const GEN_FLT x176 = x163 * x175;
	const GEN_FLT x177 = x150 + x121 * x174;
	const GEN_FLT x178 = pow(1 - pow(x177, 2), -1.0 / 2.0);
	const GEN_FLT x179 =
		x178 *
		(x161 + x162 * x176 -
		 x171 * (-x154 * (x113 * x145 +
						  x113 * (x145 + x113 * (x144 + x113 * (x141 + x139 * x143 - x140 * x142) + x118 * x140) +
								  x120 * x140) +
						  x121 * x140 + x122 * x140) -
				 x167 * x162) +
		 x173 * x140 + x174 * x145);
	const GEN_FLT x180 = -x100;
	const GEN_FLT x181 = -x151;
	const GEN_FLT x182 = cos(-gibPhase_0 + x181 + asin(x177)) * gibMag_0;
	const GEN_FLT x183 = x88 * x77;
	const GEN_FLT x184 = x99 * (x183 + x96);
	const GEN_FLT x185 = 1 + x127;
	const GEN_FLT x186 = x111 * x185;
	const GEN_FLT x187 = x77 * x132;
	const GEN_FLT x188 = x134 + x187;
	const GEN_FLT x189 = x137 * (x188 + x185 * x130);
	const GEN_FLT x190 = x103 * x186 - x103 * x189;
	const GEN_FLT x191 = x124 * x190;
	const GEN_FLT x192 = x116 * x124;
	const GEN_FLT x193 = x190 * x192;
	const GEN_FLT x194 = x113 * (x193 - x115 * x191) + x117 * x191;
	const GEN_FLT x195 = x113 * x194 + x119 * x191;
	const GEN_FLT x196 = x185 * x148;
	const GEN_FLT x197 = -x188 * x158 + x196 * x147;
	const GEN_FLT x198 = x184 - x197 * x156;
	const GEN_FLT x199 =
		x178 *
		(x197 -
		 x171 * (-x154 * (x113 * x195 +
						  x113 * (x195 + x113 * (x194 + x113 * (x193 + x190 * x143 - x191 * x142) + x118 * x191) +
								  x120 * x191) +
						  x121 * x191 + x122 * x191) -
				 x167 * x198) +
		 x173 * x191 + x174 * x195 + x176 * x198);
	const GEN_FLT x200 = -x184;
	const GEN_FLT x201 = 1 + x94;
	const GEN_FLT x202 = x99 * (x183 - x95 * x201);
	const GEN_FLT x203 = x187 + x201 * x133;
	const GEN_FLT x204 = (x131 + x203) * x137;
	const GEN_FLT x205 = x129 - x204 * x103;
	const GEN_FLT x206 = x205 * x124;
	const GEN_FLT x207 = x205 * x192;
	const GEN_FLT x208 = x113 * (x207 - x206 * x115) + x206 * x117;
	const GEN_FLT x209 = x206 * x119 + x208 * x113;
	const GEN_FLT x210 = x160 - x203 * x158;
	const GEN_FLT x211 = x163 * (x202 - x210 * x156);
	const GEN_FLT x212 =
		x178 *
		(x210 -
		 x171 * (-x154 * (x113 * (x209 + x113 * (x208 + x113 * (x207 + x205 * x143 - x206 * x142) + x206 * x118) +
								  x206 * x120) +
						  x206 * x121 + x206 * x122 + x209 * x113) -
				 x211 * x166) +
		 x206 * x173 + x209 * x174 + x211 * x175);
	const GEN_FLT x213 = -x202;
	const GEN_FLT x214 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x215 = x1 * x214;
	const GEN_FLT x216 = -x215;
	const GEN_FLT x217 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x218 = x8 * x214;
	const GEN_FLT x219 = x31 * x218;
	const GEN_FLT x220 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x221 = x1 * x220;
	const GEN_FLT x222 = x36 * x214;
	const GEN_FLT x223 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x224 = x10 * x223 + x40 * x217 + x81 * x222;
	const GEN_FLT x225 = x36 * x218;
	const GEN_FLT x226 = x1 * x223;
	const GEN_FLT x227 = x5 * x65;
	const GEN_FLT x228 = x10 * x220 + x214 * x227 + x46 * x217;
	const GEN_FLT x229 =
		x76 + x25 * (x216 + x11 * x217 + x6 * x215) + x30 * (-x219 - x221 + x224) + x43 * (x225 + x226 + x228);
	const GEN_FLT x230 = x5 * x218;
	const GEN_FLT x231 = x1 * x217;
	const GEN_FLT x232 = x40 * x220 + x46 * x223 + x65 * x222;
	const GEN_FLT x233 =
		x93 + x25 * (-x225 - x226 + x228) + x30 * (x230 + x231 + x232) + x43 * (x216 + x79 * x215 + x92 * x220);
	const GEN_FLT x234 = (x88 * x229 - x95 * x233) * x99;
	const GEN_FLT x235 = 2 * x40;
	const GEN_FLT x236 =
		x126 + x25 * (x219 + x221 + x224) + x30 * (x216 + x215 * x105 + x235 * x223) + x43 * (-x230 - x231 + x232);
	const GEN_FLT x237 = x236 * x111;
	const GEN_FLT x238 = x229 * x132 + x233 * x133;
	const GEN_FLT x239 = x137 * (x238 + x236 * x130);
	const GEN_FLT x240 = x237 * x103 - x239 * x103;
	const GEN_FLT x241 = x240 * x124;
	const GEN_FLT x242 = x240 * x192;
	const GEN_FLT x243 = x113 * (x242 - x241 * x115) + x241 * x117;
	const GEN_FLT x244 = x241 * x119 + x243 * x113;
	const GEN_FLT x245 = x236 * x148;
	const GEN_FLT x246 = -x238 * x158 + x245 * x147;
	const GEN_FLT x247 = x234 - x246 * x156;
	const GEN_FLT x248 =
		x178 *
		(x246 -
		 x171 * (-x154 * (x113 * (x244 + x113 * (x243 + x113 * (x242 + x240 * x143 - x241 * x142) + x241 * x118) +
								  x241 * x120) +
						  x241 * x121 + x241 * x122 + x244 * x113) -
				 x247 * x167) +
		 x241 * x173 + x244 * x174 + x247 * x176);
	const GEN_FLT x249 = -x234;
	const GEN_FLT x250 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x251 = x1 * x250;
	const GEN_FLT x252 = -x251;
	const GEN_FLT x253 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x254 = x9 * x253;
	const GEN_FLT x255 = x8 * x250;
	const GEN_FLT x256 = x31 * x255;
	const GEN_FLT x257 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x258 = x1 * x257;
	const GEN_FLT x259 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x260 = x9 * x259;
	const GEN_FLT x261 = x36 * x250;
	const GEN_FLT x262 = x36 * x254 + x5 * x260 + x81 * x261;
	const GEN_FLT x263 = x36 * x255;
	const GEN_FLT x264 = x1 * x259;
	const GEN_FLT x265 = x10 * x257 + x250 * x227 + x31 * x254;
	const GEN_FLT x266 =
		x76 + x25 * (x252 + 2 * x5 * x254 + x6 * x251) + x30 * (-x256 - x258 + x262) + x43 * (x263 + x264 + x265);
	const GEN_FLT x267 = x5 * x255;
	const GEN_FLT x268 = x1 * x253;
	const GEN_FLT x269 = x31 * x260 + x40 * x257 + x65 * x261;
	const GEN_FLT x270 =
		x93 + x25 * (-x263 - x264 + x265) + x30 * (x267 + x268 + x269) + x43 * (x252 + x79 * x251 + x92 * x257);
	const GEN_FLT x271 = (x88 * x266 - x95 * x270) * x99;
	const GEN_FLT x272 =
		x126 + x25 * (x256 + x258 + x262) + x30 * (x252 + x251 * x105 + x260 * x125) + x43 * (-x267 - x268 + x269);
	const GEN_FLT x273 = x272 * x111;
	const GEN_FLT x274 = x266 * x132 + x270 * x133;
	const GEN_FLT x275 = x137 * (x274 + x272 * x130);
	const GEN_FLT x276 = (x273 * x103 - x275 * x103) * x124;
	const GEN_FLT x277 = x276 * x116;
	const GEN_FLT x278 = x113 * (x277 - x276 * x115) + x276 * x117;
	const GEN_FLT x279 = x276 * x119 + x278 * x113;
	const GEN_FLT x280 = x272 * x148;
	const GEN_FLT x281 = -x274 * x158 + x280 * x147;
	const GEN_FLT x282 = x271 - x281 * x156;
	const GEN_FLT x283 =
		x178 *
		(x281 -
		 x171 * (-x154 * (x113 * (x279 + x113 * (x278 + x113 * (x277 + x276 * x114 - x276 * x142) + x276 * x118) +
								  x276 * x120) +
						  x276 * x121 + x276 * x122 + x279 * x113) -
				 x282 * x167) +
		 x276 * x173 + x279 * x174 + x282 * x176);
	const GEN_FLT x284 = -x271;
	const GEN_FLT x285 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x286 = x1 * x285;
	const GEN_FLT x287 = -x286;
	const GEN_FLT x288 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x289 = x8 * x285;
	const GEN_FLT x290 = x31 * x289;
	const GEN_FLT x291 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x292 = x1 * x291;
	const GEN_FLT x293 = x36 * x285;
	const GEN_FLT x294 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x295 = x10 * x294 + x40 * x288 + x81 * x293;
	const GEN_FLT x296 = x36 * x289;
	const GEN_FLT x297 = x1 * x294;
	const GEN_FLT x298 = x10 * x291 + x285 * x227 + x46 * x288;
	const GEN_FLT x299 =
		x76 + x25 * (x287 + x11 * x288 + x6 * x286) + x30 * (-x290 - x292 + x295) + x43 * (x296 + x297 + x298);
	const GEN_FLT x300 = x5 * x289;
	const GEN_FLT x301 = x1 * x288;
	const GEN_FLT x302 = x40 * x291 + x46 * x294 + x65 * x293;
	const GEN_FLT x303 =
		x93 + x25 * (-x296 - x297 + x298) + x30 * (x300 + x301 + x302) + x43 * (x287 + x79 * x286 + x92 * x291);
	const GEN_FLT x304 = (x88 * x299 - x95 * x303) * x99;
	const GEN_FLT x305 =
		x126 + x25 * (x290 + x292 + x295) + x30 * (x287 + x235 * x294 + x286 * x105) + x43 * (-x300 - x301 + x302);
	const GEN_FLT x306 = x305 * x111;
	const GEN_FLT x307 = x299 * x132 + x303 * x133;
	const GEN_FLT x308 = x137 * (x307 + x305 * x130);
	const GEN_FLT x309 = (x306 * x103 - x308 * x103) * x124;
	const GEN_FLT x310 = x309 * x116;
	const GEN_FLT x311 = x113 * (x310 - x309 * x115) + x309 * x117;
	const GEN_FLT x312 = x309 * x119 + x311 * x113;
	const GEN_FLT x313 = x305 * x148;
	const GEN_FLT x314 = -x307 * x158 + x313 * x147;
	const GEN_FLT x315 = x304 - x314 * x156;
	const GEN_FLT x316 =
		x178 *
		(x314 -
		 x171 * (-x154 * (x113 * (x312 + x113 * (x311 + x113 * (x310 + x309 * x114 - x309 * x142) + x309 * x118) +
								  x309 * x120) +
						  x309 * x121 + x309 * x122 + x312 * x113) -
				 x315 * x167) +
		 x309 * x173 + x312 * x174 + x315 * x176);
	const GEN_FLT x317 = -x304;
	const GEN_FLT x318 = 0.523598775598299 - tilt_1;
	const GEN_FLT x319 = cos(x318);
	const GEN_FLT x320 = pow(x319, -1);
	const GEN_FLT x321 = asin(x320 * x112);
	const GEN_FLT x322 = 8.0108022e-06 * x321;
	const GEN_FLT x323 = -8.0108022e-06 - x322;
	const GEN_FLT x324 = 0.0028679863 + x323 * x321;
	const GEN_FLT x325 = 5.3685255e-06 + x324 * x321;
	const GEN_FLT x326 = 0.0076069798 + x325 * x321;
	const GEN_FLT x327 = x326 * x321;
	const GEN_FLT x328 = -8.0108022e-06 - 1.60216044e-05 * x321;
	const GEN_FLT x329 = x324 + x328 * x321;
	const GEN_FLT x330 = x325 + x329 * x321;
	const GEN_FLT x331 = x326 + x321 * x330;
	const GEN_FLT x332 = x327 + x321 * x331;
	const GEN_FLT x333 = sin(x318);
	const GEN_FLT x334 = tan(x318);
	const GEN_FLT x335 = -x334 * x149;
	const GEN_FLT x336 = ogeeMag_1 + x151 - asin(x335);
	const GEN_FLT x337 = curve_1 + sin(x336) * ogeePhase_1;
	const GEN_FLT x338 = x337 * x333;
	const GEN_FLT x339 = x319 + x338 * x332;
	const GEN_FLT x340 = pow(x339, -1);
	const GEN_FLT x341 = pow(x321, 2);
	const GEN_FLT x342 = x337 * x341;
	const GEN_FLT x343 = x342 * x340;
	const GEN_FLT x344 = x335 + x326 * x343;
	const GEN_FLT x345 = pow(1 - pow(x344, 2), -1.0 / 2.0);
	const GEN_FLT x346 = pow(1 - x123 / pow(x319, 2), -1.0 / 2.0);
	const GEN_FLT x347 = x320 * x128;
	const GEN_FLT x348 = x347 - x320 * x138;
	const GEN_FLT x349 = x348 * x346;
	const GEN_FLT x350 = x323 * x349;
	const GEN_FLT x351 = x321 * (x350 - x322 * x349) + x324 * x349;
	const GEN_FLT x352 = x325 * x349 + x351 * x321;
	const GEN_FLT x353 = 2 * x337 * x327 * x340;
	const GEN_FLT x354 = pow(1 - pow(x334, 2) * x155, -1.0 / 2.0);
	const GEN_FLT x355 = x334 * x157;
	const GEN_FLT x356 = -x334 * x159;
	const GEN_FLT x357 = x356 + x355 * x135;
	const GEN_FLT x358 = cos(x336) * ogeePhase_1;
	const GEN_FLT x359 = x358 * (x100 - x354 * x357);
	const GEN_FLT x360 = x326 * x340 * x341;
	const GEN_FLT x361 = x346 * x330;
	const GEN_FLT x362 = 2.40324066e-05 * x321;
	const GEN_FLT x363 = x333 * x332;
	const GEN_FLT x364 = x326 * x342 / pow(x339, 2);
	const GEN_FLT x365 =
		x345 * (x357 + x352 * x343 + x353 * x349 + x360 * x359 -
				x364 * (x338 * (x321 * (x352 + x321 * (x351 + x321 * (x350 + x328 * x349 - x362 * x349) + x329 * x349) +
										x361 * x348) +
								x326 * x349 + x349 * x331 + x352 * x321) +
						x363 * x359));
	const GEN_FLT x366 = cos(-gibPhase_1 + x181 + asin(x344)) * gibMag_1;
	const GEN_FLT x367 = x320 * x186 - x320 * x189;
	const GEN_FLT x368 = x367 * x346;
	const GEN_FLT x369 = x368 * x323;
	const GEN_FLT x370 = x321 * (x369 - x368 * x322) + x368 * x324;
	const GEN_FLT x371 = x368 * x325 + x370 * x321;
	const GEN_FLT x372 = x328 * x346;
	const GEN_FLT x373 = -x334 * x196 + x355 * x188;
	const GEN_FLT x374 = x184 - x373 * x354;
	const GEN_FLT x375 = x363 * x358;
	const GEN_FLT x376 = x360 * x358;
	const GEN_FLT x377 =
		x345 * (x373 -
				x364 * (x338 * (x321 * (x371 + x321 * (x370 + x321 * (x369 - x362 * x368 + x372 * x367) + x368 * x329) +
										x361 * x367) +
								x368 * x326 + x368 * x331 + x371 * x321) +
						x374 * x375) +
				x368 * x353 + x371 * x343 + x374 * x376);
	const GEN_FLT x378 = x347 - x204 * x320;
	const GEN_FLT x379 = x378 * x346;
	const GEN_FLT x380 = x379 * x323;
	const GEN_FLT x381 = x321 * (x380 - x379 * x322) + x379 * x324;
	const GEN_FLT x382 = x379 * x325 + x381 * x321;
	const GEN_FLT x383 = x356 + x203 * x355;
	const GEN_FLT x384 = x202 - x383 * x354;
	const GEN_FLT x385 =
		x345 * (x383 -
				x364 * (x338 * (x321 * (x382 + x321 * (x381 + x321 * (x380 + x372 * x378 - x379 * x362) + x379 * x329) +
										x378 * x361) +
								x379 * x326 + x379 * x331 + x382 * x321) +
						x375 * x384) +
				x376 * x384 + x379 * x353 + x382 * x343);
	const GEN_FLT x386 = x237 * x320 - x239 * x320;
	const GEN_FLT x387 = x386 * x346;
	const GEN_FLT x388 = x387 * x323;
	const GEN_FLT x389 = x321 * (x388 - x387 * x322) + x387 * x324;
	const GEN_FLT x390 = x387 * x325 + x389 * x321;
	const GEN_FLT x391 = x238 * x355 - x245 * x334;
	const GEN_FLT x392 = x234 - x391 * x354;
	const GEN_FLT x393 =
		x345 * (x391 -
				x364 * (x338 * (x321 * (x390 + x321 * (x389 + x321 * (x388 - x362 * x387 + x372 * x386) + x387 * x329) +
										x361 * x386) +
								x387 * x326 + x387 * x331 + x390 * x321) +
						x375 * x392) +
				x376 * x392 + x387 * x353 + x390 * x343);
	const GEN_FLT x394 = x273 * x320 - x275 * x320;
	const GEN_FLT x395 = x394 * x346;
	const GEN_FLT x396 = x395 * x323;
	const GEN_FLT x397 = x321 * (x396 - x395 * x322) + x395 * x324;
	const GEN_FLT x398 = x395 * x325 + x397 * x321;
	const GEN_FLT x399 = x274 * x355 - x280 * x334;
	const GEN_FLT x400 = x271 - x399 * x354;
	const GEN_FLT x401 =
		x345 * (x399 -
				x364 * (x338 * (x321 * (x398 + x321 * (x397 + x321 * (x396 + x372 * x394 - x395 * x362) + x395 * x329) +
										x394 * x361) +
								x395 * x326 + x395 * x331 + x398 * x321) +
						x400 * x375) +
				x395 * x353 + x398 * x343 + x400 * x376);
	const GEN_FLT x402 = x306 * x320 - x308 * x320;
	const GEN_FLT x403 = x402 * x346;
	const GEN_FLT x404 = x403 * x323;
	const GEN_FLT x405 = x321 * (x404 - x403 * x322) + x403 * x324;
	const GEN_FLT x406 = x403 * x325 + x405 * x321;
	const GEN_FLT x407 = -x313 * x334 + x355 * x307;
	const GEN_FLT x408 = x304 - x407 * x354;
	const GEN_FLT x409 =
		x345 * (x407 -
				x364 * (x338 * (x321 * (x406 + x321 * (x405 + x321 * (x404 + x402 * x372 - x403 * x362) + x403 * x329) +
										x402 * x361) +
								x403 * x326 + x403 * x331 + x406 * x321) +
						x408 * x375) +
				x403 * x353 + x406 * x343 + x408 * x376);
	out[0] = x100 - x179 - (x179 + x180) * x182;
	out[1] = x184 - x199 - (x199 + x200) * x182;
	out[2] = x202 - x212 - (x212 + x213) * x182;
	out[3] = x234 - x248 - (x248 + x249) * x182;
	out[4] = x271 - x283 - (x283 + x284) * x182;
	out[5] = x304 - x316 - (x316 + x317) * x182;
	out[6] = x100 - x365 - (x180 + x365) * x366;
	out[7] = x184 - x377 - (x200 + x377) * x366;
	out[8] = x202 - x385 - (x213 + x385) * x366;
	out[9] = x234 - x393 - (x249 + x393) * x366;
	out[10] = x271 - x401 - (x284 + x401) * x366;
	out[11] = x304 - x409 - (x317 + x409) * x366;
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = -x3;
	const GEN_FLT x5 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x6 = pow(x5, 2);
	const GEN_FLT x7 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
							   pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										: (1e-10)),
								   2))
							: (0));
	const GEN_FLT x8 = cos(x0);
	const GEN_FLT x9 = 1 - x8;
	const GEN_FLT x10 = x5 * x9;
	const GEN_FLT x11 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1e-10));
	const GEN_FLT x12 = sin(x11);
	const GEN_FLT x13 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x14 = x13 * x12;
	const GEN_FLT x15 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x16 = cos(x11);
	const GEN_FLT x17 = 1 - x16;
	const GEN_FLT x18 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x15 * x12;
	const GEN_FLT x22 = x13 * x17;
	const GEN_FLT x23 = x22 * x18;
	const GEN_FLT x24 = pow(x18, 2);
	const GEN_FLT x25 = obj_px + (x16 + x24 * x17) * sensor_x + (x14 + x20) * sensor_z + (-x21 + x23) * sensor_y;
	const GEN_FLT x26 = x8 + x6 * x9;
	const GEN_FLT x27 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x28 = x27 * x12;
	const GEN_FLT x29 = -x28;
	const GEN_FLT x30 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x31 = x27 * x16;
	const GEN_FLT x32 = x31 * x15;
	const GEN_FLT x33 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x34 = x33 * x12;
	const GEN_FLT x35 = x12 * x18;
	const GEN_FLT x36 = x27 * x13;
	const GEN_FLT x37 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x38 = x37 * x17;
	const GEN_FLT x39 = x30 * x22 + x36 * x35 + x38 * x18;
	const GEN_FLT x40 = x31 * x13;
	const GEN_FLT x41 = x37 * x12;
	const GEN_FLT x42 = x15 * x17;
	const GEN_FLT x43 = x33 * x19 + x42 * x30 + x35 * x27 * x15;
	const GEN_FLT x44 =
		(x29 + x24 * x28 + 2 * x30 * x19) * sensor_x + (-x32 - x34 + x39) * sensor_y + (x40 + x41 + x43) * sensor_z;
	const GEN_FLT x45 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x46 = x1 * x45;
	const GEN_FLT x47 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x48 = x47 * x10;
	const GEN_FLT x49 = -x46 + x48;
	const GEN_FLT x50 = x31 * x18;
	const GEN_FLT x51 = x30 * x12;
	const GEN_FLT x52 = x33 * x22 + x36 * x21 + x38 * x15;
	const GEN_FLT x53 = pow(x13, 2);
	const GEN_FLT x54 =
		(x29 + 2 * x37 * x22 + x53 * x28) * sensor_y + (x32 + x34 + x39) * sensor_x + (-x50 - x51 + x52) * sensor_z;
	const GEN_FLT x55 = x22 * x15;
	const GEN_FLT x56 = obj_py + (x16 + x53 * x17) * sensor_y + (x21 + x23) * sensor_x + (-x35 + x55) * sensor_z;
	const GEN_FLT x57 = x2 * x8;
	const GEN_FLT x58 = x57 * x45;
	const GEN_FLT x59 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x60 = x1 * x59;
	const GEN_FLT x61 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x62 = x9 * x61;
	const GEN_FLT x63 = x9 * x47;
	const GEN_FLT x64 = x5 * x62 + x7 * x63 + x3 * x5 * x47;
	const GEN_FLT x65 = pow(x15, 2);
	const GEN_FLT x66 = obj_pz + (x16 + x65 * x17) * sensor_z + (-x14 + x20) * sensor_x + (x35 + x55) * sensor_y;
	const GEN_FLT x67 = x57 * x47;
	const GEN_FLT x68 = x1 * x61;
	const GEN_FLT x69 = x9 * x45;
	const GEN_FLT x70 = x3 * x45;
	const GEN_FLT x71 = x5 * x70 + x59 * x10 + x7 * x69;
	const GEN_FLT x72 =
		(x29 + 2 * x42 * x33 + x65 * x28) * sensor_z + (-x40 - x41 + x43) * sensor_x + (x50 + x51 + x52) * sensor_y;
	const GEN_FLT x73 = x1 * x47;
	const GEN_FLT x74 = x45 * x10;
	const GEN_FLT x75 = x73 + x74;
	const GEN_FLT x76 = x25 * (x4 + x3 * x6 + 2 * x7 * x10) + x44 * x26 + x54 * x49 + x56 * (-x58 - x60 + x64) +
						x66 * (x67 + x68 + x71) + x72 * x75;
	const GEN_FLT x77 = pow(x45, 2);
	const GEN_FLT x78 = x8 + x9 * x77;
	const GEN_FLT x79 = x1 * x5;
	const GEN_FLT x80 = x63 * x45;
	const GEN_FLT x81 = x79 + x80;
	const GEN_FLT x82 = -x73 + x74;
	const GEN_FLT x83 = lh_pz + x78 * x66 + x81 * x56 + x82 * x25;
	const GEN_FLT x84 = lh_px + x25 * x26 + x56 * x49 + x75 * x66;
	const GEN_FLT x85 = pow(x84, 2);
	const GEN_FLT x86 = x5 * x57;
	const GEN_FLT x87 = x1 * x7;
	const GEN_FLT x88 = x62 * x45 + x63 * x59 + x70 * x47;
	const GEN_FLT x89 = x25 * (-x67 - x68 + x71) + x56 * (x86 + x87 + x88) + x66 * (x4 + x3 * x77 + 2 * x69 * x59) +
						x72 * x78 + x81 * x54 + x82 * x44;
	const GEN_FLT x90 = x85 + pow(x83, 2);
	const GEN_FLT x91 = pow(x90, -1);
	const GEN_FLT x92 = x85 * x91 * (-x89 / x84 + x83 * x76 / x85);
	const GEN_FLT x93 = -x92;
	const GEN_FLT x94 = 0.523598775598299 + tilt_0;
	const GEN_FLT x95 = cos(x94);
	const GEN_FLT x96 = pow(x95, -1);
	const GEN_FLT x97 = -x79 + x80;
	const GEN_FLT x98 = pow(x47, 2);
	const GEN_FLT x99 = x8 + x9 * x98;
	const GEN_FLT x100 = x46 + x48;
	const GEN_FLT x101 = lh_py + x25 * x100 + x56 * x99 + x66 * x97;
	const GEN_FLT x102 = pow(x101, 2);
	const GEN_FLT x103 = x102 + x90;
	const GEN_FLT x104 = pow(x103, -1.0 / 2.0);
	const GEN_FLT x105 = x101 * x104;
	const GEN_FLT x106 = asin(x96 * x105);
	const GEN_FLT x107 = -8.0108022e-06 - 1.60216044e-05 * x106;
	const GEN_FLT x108 = 8.0108022e-06 * x106;
	const GEN_FLT x109 = -8.0108022e-06 - x108;
	const GEN_FLT x110 = 0.0028679863 + x109 * x106;
	const GEN_FLT x111 = x110 + x107 * x106;
	const GEN_FLT x112 = 5.3685255e-06 + x106 * x110;
	const GEN_FLT x113 = x112 + x106 * x111;
	const GEN_FLT x114 = pow(x95, -2);
	const GEN_FLT x115 = x102 / x103;
	const GEN_FLT x116 = pow(1 - x114 * x115, -1.0 / 2.0);
	const GEN_FLT x117 = x25 * (x58 + x60 + x64) + x44 * x100 + x54 * x99 + x56 * (x4 + x3 * x98 + 2 * x62 * x47) +
						 x66 * (-x86 - x87 + x88) + x72 * x97;
	const GEN_FLT x118 = x104 * x117;
	const GEN_FLT x119 = 2 * x83 * x89 + 2 * x84 * x76;
	const GEN_FLT x120 = (1.0 / 2.0) * x101;
	const GEN_FLT x121 = x120 * (x119 + 2 * x101 * x117) / pow(x103, 3.0 / 2.0);
	const GEN_FLT x122 = x96 * x118 - x96 * x121;
	const GEN_FLT x123 = x116 * x122;
	const GEN_FLT x124 = x109 * x123;
	const GEN_FLT x125 = 2.40324066e-05 * x106;
	const GEN_FLT x126 = x106 * (x124 - x108 * x123) + x110 * x123;
	const GEN_FLT x127 = x106 * x126 + x112 * x123;
	const GEN_FLT x128 = 0.0076069798 + x106 * x112;
	const GEN_FLT x129 = x128 + x106 * x113;
	const GEN_FLT x130 = sin(x94);
	const GEN_FLT x131 = tan(x94);
	const GEN_FLT x132 = pow(x90, -1.0 / 2.0);
	const GEN_FLT x133 = x101 * x132;
	const GEN_FLT x134 = x133 * x131;
	const GEN_FLT x135 = atan2(-x83, x84);
	const GEN_FLT x136 = ogeeMag_0 + x135 - asin(x134);
	const GEN_FLT x137 = sin(x136);
	const GEN_FLT x138 = curve_0 + x137 * ogeePhase_0;
	const GEN_FLT x139 = x130 * x138;
	const GEN_FLT x140 =
		-x139 * (x106 * x127 +
				 x106 * (x127 + x106 * (x126 + x106 * (x124 + x107 * x123 - x123 * x125) + x111 * x123) + x113 * x123) +
				 x123 * x128 + x123 * x129);
	const GEN_FLT x141 = x106 * x128;
	const GEN_FLT x142 = x141 + x106 * x129;
	const GEN_FLT x143 = x130 * x142;
	const GEN_FLT x144 = pow(x131, 2);
	const GEN_FLT x145 = x91 * x102;
	const GEN_FLT x146 = pow(1 - x144 * x145, -1.0 / 2.0);
	const GEN_FLT x147 = x119 * x120 / pow(x90, 3.0 / 2.0);
	const GEN_FLT x148 = x117 * x132;
	const GEN_FLT x149 = -x131 * x147 + x131 * x148;
	const GEN_FLT x150 = x92 - x146 * x149;
	const GEN_FLT x151 = cos(x136) * ogeePhase_0;
	const GEN_FLT x152 = x150 * x151;
	const GEN_FLT x153 = pow(x106, 2);
	const GEN_FLT x154 = x95 - x138 * x143;
	const GEN_FLT x155 = x128 * x138 * x153 / pow(x154, 2);
	const GEN_FLT x156 = pow(x154, -1);
	const GEN_FLT x157 = x153 * x156;
	const GEN_FLT x158 = x128 * x157;
	const GEN_FLT x159 = 2 * x138 * x141 * x156;
	const GEN_FLT x160 = x138 * x157;
	const GEN_FLT x161 = x149 + x123 * x159 + x127 * x160;
	const GEN_FLT x162 = x134 + x138 * x158;
	const GEN_FLT x163 = pow(1 - pow(x162, 2), -1.0 / 2.0);
	const GEN_FLT x164 = x163 * (x161 + x152 * x158 - x155 * (x140 - x143 * x152));
	const GEN_FLT x165 = x164 + x93;
	const GEN_FLT x166 = -x135;
	const GEN_FLT x167 = -gibPhase_0 + x166 + asin(x162);
	const GEN_FLT x168 = cos(x167) * gibMag_0;
	const GEN_FLT x169 = -x164 + x92;
	const GEN_FLT x170 = x169 - x168 * x165;
	const GEN_FLT x171 = x116 * (x122 + x105 * x114 * x130);
	const GEN_FLT x172 = x109 * x171;
	const GEN_FLT x173 = x106 * (x172 - x108 * x171) + x110 * x171;
	const GEN_FLT x174 = x106 * x173 + x112 * x171;
	const GEN_FLT x175 = x149 + x133 * (1 + x144);
	const GEN_FLT x176 = x92 - x175 * x146;
	const GEN_FLT x177 = x143 * x151;
	const GEN_FLT x178 = x151 * x158;
	const GEN_FLT x179 =
		x163 * (x175 -
				x155 * (-x130 -
						x139 * (x106 * x174 +
								x106 * (x174 + x106 * (x173 + x106 * (x172 + x107 * x171 - x125 * x171) + x111 * x171) +
										x113 * x171) +
								x128 * x171 + x129 * x171) -
						x176 * x177 - x95 * x138 * x142) +
				x160 * x174 + x171 * x159 + x178 * x176);
	const GEN_FLT x180 = 1 + x152;
	const GEN_FLT x181 = x163 * (x161 - x155 * (x140 - x180 * x143) + x180 * x158);
	const GEN_FLT x182 = 1 + x150;
	const GEN_FLT x183 = x163 * (x161 - x155 * (x140 - x177 * x182) + x178 * x182);
	const GEN_FLT x184 = x137 + x152;
	const GEN_FLT x185 = x163 * (x161 - x155 * (x140 - x184 * x143) + x184 * x158);
	const GEN_FLT x186 = 0.523598775598299 - tilt_1;
	const GEN_FLT x187 = cos(x186);
	const GEN_FLT x188 = pow(x187, -1);
	const GEN_FLT x189 = asin(x105 * x188);
	const GEN_FLT x190 = 8.0108022e-06 * x189;
	const GEN_FLT x191 = -8.0108022e-06 - x190;
	const GEN_FLT x192 = 0.0028679863 + x189 * x191;
	const GEN_FLT x193 = 5.3685255e-06 + x189 * x192;
	const GEN_FLT x194 = 0.0076069798 + x189 * x193;
	const GEN_FLT x195 = pow(x189, 2);
	const GEN_FLT x196 = tan(x186);
	const GEN_FLT x197 = -x196 * x133;
	const GEN_FLT x198 = ogeeMag_1 + x135 - asin(x197);
	const GEN_FLT x199 = sin(x198);
	const GEN_FLT x200 = curve_1 + x199 * ogeePhase_1;
	const GEN_FLT x201 = sin(x186);
	const GEN_FLT x202 = x189 * x194;
	const GEN_FLT x203 = -8.0108022e-06 - 1.60216044e-05 * x189;
	const GEN_FLT x204 = x192 + x203 * x189;
	const GEN_FLT x205 = x193 + x204 * x189;
	const GEN_FLT x206 = x194 + x205 * x189;
	const GEN_FLT x207 = x202 + x206 * x189;
	const GEN_FLT x208 = x201 * x207;
	const GEN_FLT x209 = x187 + x200 * x208;
	const GEN_FLT x210 = pow(x209, -1);
	const GEN_FLT x211 = x210 * x200;
	const GEN_FLT x212 = x211 * x195;
	const GEN_FLT x213 = x197 + x212 * x194;
	const GEN_FLT x214 = pow(1 - pow(x213, 2), -1.0 / 2.0);
	const GEN_FLT x215 = pow(x187, -2);
	const GEN_FLT x216 = pow(1 - x215 * x115, -1.0 / 2.0);
	const GEN_FLT x217 = x118 * x188 - x121 * x188;
	const GEN_FLT x218 = x217 * x216;
	const GEN_FLT x219 = x218 * x191;
	const GEN_FLT x220 = 2.40324066e-05 * x189;
	const GEN_FLT x221 = x189 * (x219 - x218 * x190) + x218 * x192;
	const GEN_FLT x222 = x218 * x193 + x221 * x189;
	const GEN_FLT x223 = x200 * x201;
	const GEN_FLT x224 =
		x223 * (x189 * (x222 + x189 * (x221 + x189 * (x219 + x218 * x203 - x218 * x220) + x218 * x204) + x218 * x205) +
				x218 * x194 + x218 * x206 + x222 * x189);
	const GEN_FLT x225 = pow(x196, 2);
	const GEN_FLT x226 = pow(1 - x225 * x145, -1.0 / 2.0);
	const GEN_FLT x227 = x196 * x147 - x196 * x148;
	const GEN_FLT x228 = x92 - x227 * x226;
	const GEN_FLT x229 = cos(x198) * ogeePhase_1;
	const GEN_FLT x230 = x228 * x229;
	const GEN_FLT x231 = x194 * x195;
	const GEN_FLT x232 = x231 * x200 / pow(x209, 2);
	const GEN_FLT x233 = x210 * x231;
	const GEN_FLT x234 = 2 * x211 * x202;
	const GEN_FLT x235 = x227 + x212 * x222 + x218 * x234;
	const GEN_FLT x236 = x214 * (x235 + x230 * x233 - x232 * (x224 + x230 * x208));
	const GEN_FLT x237 = x236 + x93;
	const GEN_FLT x238 = -gibPhase_1 + x166 + asin(x213);
	const GEN_FLT x239 = cos(x238) * gibMag_1;
	const GEN_FLT x240 = -x236 + x92;
	const GEN_FLT x241 = x240 - x237 * x239;
	const GEN_FLT x242 = x216 * (x217 - x215 * x201 * x105);
	const GEN_FLT x243 = x242 * x191;
	const GEN_FLT x244 = x189 * (x243 - x242 * x190) + x242 * x192;
	const GEN_FLT x245 = x242 * x193 + x244 * x189;
	const GEN_FLT x246 = x227 + x133 * (1 + x225);
	const GEN_FLT x247 = x92 - x226 * x246;
	const GEN_FLT x248 = x208 * x229;
	const GEN_FLT x249 = x233 * x229;
	const GEN_FLT x250 =
		x214 * (x246 + x212 * x245 -
				x232 * (x201 +
						x223 * (x189 * (x245 + x189 * (x244 + x189 * (x243 + x203 * x242 - x220 * x242) + x204 * x242) +
										x205 * x242) +
								x206 * x242 + x242 * x194 + x245 * x189) +
						x247 * x248 - x200 * x207 * x187) +
				x234 * x242 + x247 * x249);
	const GEN_FLT x251 = 1 + x230;
	const GEN_FLT x252 = x214 * (x235 - x232 * (x224 + x208 * x251) + x233 * x251);
	const GEN_FLT x253 = 1 + x228;
	const GEN_FLT x254 = x214 * (x235 - x232 * (x224 + x253 * x248) + x253 * x249);
	const GEN_FLT x255 = x199 + x230;
	const GEN_FLT x256 = x214 * (x235 - x232 * (x224 + x208 * x255) + x233 * x255);
	out[0] = -1 + x170;
	out[1] = -x179 + x92 - x168 * (x179 + x93);
	out[2] = -x181 + x92 - x168 * (x181 + x93);
	out[3] = x169 - x168 * (-1 + x165);
	out[4] = x170 - sin(x167);
	out[5] = -x183 + x92 - x168 * (x183 + x93);
	out[6] = -x185 + x92 - x168 * (x185 + x93);
	out[7] = x170;
	out[8] = x170;
	out[9] = x170;
	out[10] = x170;
	out[11] = x170;
	out[12] = x170;
	out[13] = x170;
	out[14] = x241;
	out[15] = x241;
	out[16] = x241;
	out[17] = x241;
	out[18] = x241;
	out[19] = x241;
	out[20] = x241;
	out[21] = -1 + x241;
	out[22] = -x250 + x92 - x239 * (x250 + x93);
	out[23] = -x252 + x92 - x239 * (x252 + x93);
	out[24] = x240 - x239 * (-1 + x237);
	out[25] = x241 - sin(x238);
	out[26] = -x254 + x92 - x239 * (x254 + x93);
	out[27] = -x256 + x92 - x239 * (x256 + x93);
}

/** Applying function <function reproject_axis_x_gen2 at 0x7effc26e3a70> */
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
	const GEN_FLT x0 = 0.523598775598299 + tilt_0;
	const GEN_FLT x1 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x4 = x2 * x3;
	const GEN_FLT x5 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x6 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x7 = cos(x1);
	const GEN_FLT x8 = 1 - x7;
	const GEN_FLT x9 = x6 * x8;
	const GEN_FLT x10 = x5 * x9;
	const GEN_FLT x11 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1e-10));
	const GEN_FLT x12 = cos(x11);
	const GEN_FLT x13 = 1 - x12;
	const GEN_FLT x14 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x15 = sin(x11);
	const GEN_FLT x16 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x17 = x15 * x16;
	const GEN_FLT x18 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x19 = x14 * x13 * x18;
	const GEN_FLT x20 = x15 * x18;
	const GEN_FLT x21 = x13 * x16;
	const GEN_FLT x22 = x21 * x14;
	const GEN_FLT x23 =
		obj_pz + (x12 + pow(x14, 2) * x13) * sensor_z + (x17 + x19) * sensor_y + (-x20 + x22) * sensor_x;
	const GEN_FLT x24 = x15 * x14;
	const GEN_FLT x25 = x21 * x18;
	const GEN_FLT x26 =
		obj_py + (x12 + x13 * pow(x18, 2)) * sensor_y + (-x17 + x19) * sensor_z + (x24 + x25) * sensor_x;
	const GEN_FLT x27 =
		obj_px + (x12 + x13 * pow(x16, 2)) * sensor_x + (x20 + x22) * sensor_z + (-x24 + x25) * sensor_y;
	const GEN_FLT x28 = x2 * x5;
	const GEN_FLT x29 = x3 * x9;
	const GEN_FLT x30 = lh_py + x23 * (x10 - x4) + x26 * (x7 + pow(x6, 2) * x8) + (x28 + x29) * x27;
	const GEN_FLT x31 = x2 * x6;
	const GEN_FLT x32 = x3 * x5 * x8;
	const GEN_FLT x33 = lh_pz + x23 * (x7 + pow(x5, 2) * x8) + x26 * (x10 + x4) + (-x31 + x32) * x27;
	const GEN_FLT x34 = lh_px + x27 * (x7 + pow(x3, 2) * x8) + (-x28 + x29) * x26 + (x31 + x32) * x23;
	const GEN_FLT x35 = pow(x33, 2) + pow(x34, 2);
	const GEN_FLT x36 = cos(x0);
	const GEN_FLT x37 = asin(x30 / (x36 * sqrt(x35 + pow(x30, 2))));
	const GEN_FLT x38 = 0.0028679863 + x37 * (-8.0108022e-06 - 8.0108022e-06 * x37);
	const GEN_FLT x39 = 5.3685255e-06 + x38 * x37;
	const GEN_FLT x40 = 0.0076069798 + x37 * x39;
	const GEN_FLT x41 = tan(x0) * x30 / sqrt(x35);
	const GEN_FLT x42 = atan2(-x33, x34);
	const GEN_FLT x43 = curve_0 + sin(ogeeMag_0 + x42 - asin(x41)) * ogeePhase_0;
	const GEN_FLT x44 = asin(
		x41 + x40 * x43 * pow(x37, 2) /
				  (x36 - sin(x0) * x43 *
							 (x37 * (x40 + x37 * (x39 + x37 * (x38 + x37 * (-8.0108022e-06 - 1.60216044e-05 * x37)))) +
							  x40 * x37)));
	out[0] = -1.5707963267949 - phase_0 + x42 - x44 + sin(gibPhase_0 + x42 - x44) * gibMag_0;
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 - x1;
	const GEN_FLT x3 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x4 = pow(x3, 2);
	const GEN_FLT x5 = x1 + x2 * x4;
	const GEN_FLT x6 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x7 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1e-10));
	const GEN_FLT x8 = sin(x7);
	const GEN_FLT x9 = x6 * x8;
	const GEN_FLT x10 = -x9;
	const GEN_FLT x11 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x12 = pow(x11, 2);
	const GEN_FLT x13 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x14 = cos(x7);
	const GEN_FLT x15 = 1 - x14;
	const GEN_FLT x16 = x15 * x11;
	const GEN_FLT x17 = 2 * x16;
	const GEN_FLT x18 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x19 = x6 * x14;
	const GEN_FLT x20 = x19 * x18;
	const GEN_FLT x21 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x22 = x8 * x21;
	const GEN_FLT x23 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x24 = x8 * x11;
	const GEN_FLT x25 = x24 * x23;
	const GEN_FLT x26 = x23 * x15;
	const GEN_FLT x27 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x28 = x26 * x13 + x27 * x16 + x6 * x25;
	const GEN_FLT x29 = x23 * x19;
	const GEN_FLT x30 = x8 * x27;
	const GEN_FLT x31 = x24 * x18;
	const GEN_FLT x32 = x15 * x18;
	const GEN_FLT x33 = x21 * x16 + x32 * x13 + x6 * x31;
	const GEN_FLT x34 =
		(x10 + x13 * x17 + x9 * x12) * sensor_x + (-x20 - x22 + x28) * sensor_y + (x29 + x30 + x33) * sensor_z;
	const GEN_FLT x35 = 1 + x34;
	const GEN_FLT x36 = x11 * x19;
	const GEN_FLT x37 = x8 * x13;
	const GEN_FLT x38 = x8 * x18;
	const GEN_FLT x39 = x38 * x23;
	const GEN_FLT x40 = x21 * x26 + x32 * x27 + x6 * x39;
	const GEN_FLT x41 = pow(x18, 2);
	const GEN_FLT x42 = 2 * x32;
	const GEN_FLT x43 =
		(x10 + x42 * x21 + x9 * x41) * sensor_z + (-x29 - x30 + x33) * sensor_x + (x36 + x37 + x40) * sensor_y;
	const GEN_FLT x44 = sin(x0);
	const GEN_FLT x45 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x46 = x44 * x45;
	const GEN_FLT x47 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x48 = x2 * x3;
	const GEN_FLT x49 = x47 * x48;
	const GEN_FLT x50 = x46 + x49;
	const GEN_FLT x51 = x50 * x43;
	const GEN_FLT x52 = x44 * x47;
	const GEN_FLT x53 = x45 * x48;
	const GEN_FLT x54 = -x52 + x53;
	const GEN_FLT x55 = pow(x23, 2);
	const GEN_FLT x56 = 2 * x26;
	const GEN_FLT x57 =
		(x10 + x56 * x27 + x9 * x55) * sensor_y + (x20 + x22 + x28) * sensor_x + (-x36 - x37 + x40) * sensor_z;
	const GEN_FLT x58 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x59 = x58 * x44;
	const GEN_FLT x60 = -x59;
	const GEN_FLT x61 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x62 = x8 * x23;
	const GEN_FLT x63 = x32 * x11;
	const GEN_FLT x64 = x26 * x11;
	const GEN_FLT x65 = obj_px + (x14 + x15 * x12) * sensor_x + (x62 + x63) * sensor_z + (-x38 + x64) * sensor_y;
	const GEN_FLT x66 = x26 * x18;
	const GEN_FLT x67 = obj_py + (x14 + x55 * x15) * sensor_y + (x38 + x64) * sensor_x + (-x24 + x66) * sensor_z;
	const GEN_FLT x68 = x1 * x58;
	const GEN_FLT x69 = x68 * x47;
	const GEN_FLT x70 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x71 = x70 * x44;
	const GEN_FLT x72 = x3 * x59;
	const GEN_FLT x73 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x74 = x2 * x45;
	const GEN_FLT x75 = x72 * x45 + x73 * x48 + x74 * x61;
	const GEN_FLT x76 = obj_pz + (x14 + x41 * x15) * sensor_z + (-x62 + x63) * sensor_x + (x24 + x66) * sensor_y;
	const GEN_FLT x77 = x68 * x45;
	const GEN_FLT x78 = x73 * x44;
	const GEN_FLT x79 = x2 * x47;
	const GEN_FLT x80 = x70 * x48 + x72 * x47 + x79 * x61;
	const GEN_FLT x81 = x65 * (x60 + x4 * x59 + 2 * x61 * x48) + x67 * (-x69 - x71 + x75) + x76 * (x77 + x78 + x80);
	const GEN_FLT x82 = x81 + x54 * x57;
	const GEN_FLT x83 = x51 + x82 + x5 * x35;
	const GEN_FLT x84 = pow(x47, 2);
	const GEN_FLT x85 = x1 + x2 * x84;
	const GEN_FLT x86 = x3 * x44;
	const GEN_FLT x87 = x74 * x47;
	const GEN_FLT x88 = x86 + x87;
	const GEN_FLT x89 = -x46 + x49;
	const GEN_FLT x90 = lh_pz + x85 * x76 + x88 * x67 + x89 * x65;
	const GEN_FLT x91 = lh_px + x5 * x65 + x67 * x54 + x76 * x50;
	const GEN_FLT x92 = pow(x91, 2);
	const GEN_FLT x93 = x90 / x92;
	const GEN_FLT x94 = x88 * x57;
	const GEN_FLT x95 = x3 * x68;
	const GEN_FLT x96 = x61 * x44;
	const GEN_FLT x97 = x70 * x74 + x73 * x79 + x59 * x45 * x47;
	const GEN_FLT x98 = x65 * (-x77 - x78 + x80) + x67 * (x95 + x96 + x97) + x76 * (x60 + 2 * x70 * x79 + x84 * x59);
	const GEN_FLT x99 = x98 + x85 * x43;
	const GEN_FLT x100 = x94 + x99 + x89 * x35;
	const GEN_FLT x101 = pow(x91, -1);
	const GEN_FLT x102 = x92 + pow(x90, 2);
	const GEN_FLT x103 = pow(x102, -1);
	const GEN_FLT x104 = x92 * x103;
	const GEN_FLT x105 = x104 * (-x101 * x100 + x83 * x93);
	const GEN_FLT x106 = x52 + x53;
	const GEN_FLT x107 = -x86 + x87;
	const GEN_FLT x108 = x43 * x107;
	const GEN_FLT x109 = pow(x45, 2);
	const GEN_FLT x110 = x1 + x2 * x109;
	const GEN_FLT x111 = x65 * (x69 + x71 + x75) + x67 * (x60 + x59 * x109 + 2 * x73 * x74) + x76 * (-x95 - x96 + x97);
	const GEN_FLT x112 = x111 + x57 * x110;
	const GEN_FLT x113 = x108 + x112 + x35 * x106;
	const GEN_FLT x114 = lh_py + x65 * x106 + x67 * x110 + x76 * x107;
	const GEN_FLT x115 = pow(x114, 2);
	const GEN_FLT x116 = x102 + x115;
	const GEN_FLT x117 = 0.523598775598299 + tilt_0;
	const GEN_FLT x118 = cos(x117);
	const GEN_FLT x119 = pow(x118, -1);
	const GEN_FLT x120 = x119 / sqrt(x116);
	const GEN_FLT x121 = 2 * x114;
	const GEN_FLT x122 = 2 * x91;
	const GEN_FLT x123 = 2 * x90;
	const GEN_FLT x124 = x100 * x123 + x83 * x122;
	const GEN_FLT x125 = (1.0 / 2.0) * x114;
	const GEN_FLT x126 = x119 * x125 / pow(x116, 3.0 / 2.0);
	const GEN_FLT x127 = x113 * x120 - x126 * (x124 + x113 * x121);
	const GEN_FLT x128 = asin(x114 * x120);
	const GEN_FLT x129 = -8.0108022e-06 - 1.60216044e-05 * x128;
	const GEN_FLT x130 = 8.0108022e-06 * x128;
	const GEN_FLT x131 = -8.0108022e-06 - x130;
	const GEN_FLT x132 = 0.0028679863 + x128 * x131;
	const GEN_FLT x133 = x132 + x128 * x129;
	const GEN_FLT x134 = 5.3685255e-06 + x128 * x132;
	const GEN_FLT x135 = x134 + x128 * x133;
	const GEN_FLT x136 = 0.0076069798 + x128 * x134;
	const GEN_FLT x137 = x136 + x128 * x135;
	const GEN_FLT x138 = pow(1 - x115 / (pow(x118, 2) * x116), -1.0 / 2.0);
	const GEN_FLT x139 = x137 * x138;
	const GEN_FLT x140 = x127 * x138;
	const GEN_FLT x141 = x131 * x140;
	const GEN_FLT x142 = 2.40324066e-05 * x128;
	const GEN_FLT x143 = x128 * (x141 - x130 * x140) + x132 * x140;
	const GEN_FLT x144 = x128 * x143 + x134 * x140;
	const GEN_FLT x145 = x138 * x136;
	const GEN_FLT x146 = sin(x117);
	const GEN_FLT x147 = tan(x117);
	const GEN_FLT x148 = x147 / sqrt(x102);
	const GEN_FLT x149 = x114 * x148;
	const GEN_FLT x150 = atan2(-x90, x91);
	const GEN_FLT x151 = ogeeMag_0 + x150 - asin(x149);
	const GEN_FLT x152 = curve_0 + sin(x151) * ogeePhase_0;
	const GEN_FLT x153 = x146 * x152;
	const GEN_FLT x154 = pow(1 - x103 * x115 * pow(x147, 2), -1.0 / 2.0);
	const GEN_FLT x155 = x125 * x147 / pow(x102, 3.0 / 2.0);
	const GEN_FLT x156 = x113 * x148 - x124 * x155;
	const GEN_FLT x157 = x105 - x154 * x156;
	const GEN_FLT x158 = cos(x151) * ogeePhase_0;
	const GEN_FLT x159 = x128 * x136;
	const GEN_FLT x160 = x159 + x128 * x137;
	const GEN_FLT x161 = x160 * x146;
	const GEN_FLT x162 = x161 * x158;
	const GEN_FLT x163 = x118 - x160 * x153;
	const GEN_FLT x164 = pow(x128, 2);
	const GEN_FLT x165 = x164 * x136;
	const GEN_FLT x166 = x165 * x152 / pow(x163, 2);
	const GEN_FLT x167 = pow(x163, -1);
	const GEN_FLT x168 = x167 * x152;
	const GEN_FLT x169 = 2 * x168 * x159;
	const GEN_FLT x170 = x168 * x164;
	const GEN_FLT x171 = x167 * x165;
	const GEN_FLT x172 = x171 * x158;
	const GEN_FLT x173 = x149 + x170 * x136;
	const GEN_FLT x174 = pow(1 - pow(x173, 2), -1.0 / 2.0);
	const GEN_FLT x175 =
		x174 *
		(x156 -
		 x166 * (-x153 * (x127 * x139 + x127 * x145 + x128 * x144 +
						  x128 * (x144 + x128 * (x143 + x128 * (x141 + x129 * x140 - x140 * x142) + x133 * x140) +
								  x135 * x140)) -
				 x162 * x157) +
		 x169 * x140 + x170 * x144 + x172 * x157);
	const GEN_FLT x176 = cos(gibPhase_0 + x150 - asin(x173)) * gibMag_0;
	const GEN_FLT x177 = x5 * x34;
	const GEN_FLT x178 = 1 + x57;
	const GEN_FLT x179 = x177 + x51 + x81 + x54 * x178;
	const GEN_FLT x180 = x89 * x34;
	const GEN_FLT x181 = x180 + x99 + x88 * x178;
	const GEN_FLT x182 = x104 * (-x101 * x181 + x93 * x179);
	const GEN_FLT x183 = x34 * x106;
	const GEN_FLT x184 = x108 + x111 + x183 + x110 * x178;
	const GEN_FLT x185 = x122 * x179 + x123 * x181;
	const GEN_FLT x186 = x138 * (x120 * x184 - x126 * (x185 + x121 * x184));
	const GEN_FLT x187 = x186 * x131;
	const GEN_FLT x188 = x128 * (x187 - x186 * x130) + x186 * x132;
	const GEN_FLT x189 = x128 * x188 + x186 * x134;
	const GEN_FLT x190 = x184 * x148 - x185 * x155;
	const GEN_FLT x191 = x158 * (x182 - x190 * x154);
	const GEN_FLT x192 =
		x174 *
		(x190 -
		 x166 * (-x153 * (x128 * x189 +
						  x128 * (x189 + x128 * (x188 + x128 * (x187 + x129 * x186 - x186 * x142) + x186 * x133) +
								  x186 * x135) +
						  x186 * x136 + x186 * x137) -
				 x161 * x191) +
		 x169 * x186 + x170 * x189 + x171 * x191);
	const GEN_FLT x193 = 1 + x43;
	const GEN_FLT x194 = x177 + x82 + x50 * x193;
	const GEN_FLT x195 = x180 + x94 + x98 + x85 * x193;
	const GEN_FLT x196 = x104 * (-x101 * x195 + x93 * x194);
	const GEN_FLT x197 = x112 + x183 + x107 * x193;
	const GEN_FLT x198 = x122 * x194 + x123 * x195;
	const GEN_FLT x199 = x120 * x197 - x126 * (x198 + x121 * x197);
	const GEN_FLT x200 = x199 * x138;
	const GEN_FLT x201 = x200 * x131;
	const GEN_FLT x202 = x128 * (x201 - x200 * x130) + x200 * x132;
	const GEN_FLT x203 = x200 * x134 + x202 * x128;
	const GEN_FLT x204 = x197 * x148 - x198 * x155;
	const GEN_FLT x205 = x196 - x204 * x154;
	const GEN_FLT x206 =
		x174 *
		(x204 -
		 x166 * (-x153 * (x128 * (x203 + x128 * (x202 + x128 * (x201 + x200 * x129 - x200 * x142) + x200 * x133) +
								  x200 * x135) +
						  x199 * x139 + x200 * x136 + x203 * x128) -
				 x205 * x162) +
		 x200 * x169 + x203 * x170 + x205 * x172);
	const GEN_FLT x207 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x208 = x8 * x207;
	const GEN_FLT x209 = -x208;
	const GEN_FLT x210 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x211 = x14 * x207;
	const GEN_FLT x212 = x18 * x211;
	const GEN_FLT x213 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x214 = x8 * x213;
	const GEN_FLT x215 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x216 = x16 * x215 + x25 * x207 + x26 * x210;
	const GEN_FLT x217 = x23 * x211;
	const GEN_FLT x218 = x8 * x215;
	const GEN_FLT x219 = x16 * x213 + x31 * x207 + x32 * x210;
	const GEN_FLT x220 = (x209 + x12 * x208 + x17 * x210) * sensor_x + (-x212 - x214 + x216) * sensor_y +
						 (x217 + x218 + x219) * sensor_z;
	const GEN_FLT x221 = x11 * x211;
	const GEN_FLT x222 = x8 * x210;
	const GEN_FLT x223 = x26 * x213 + x32 * x215 + x39 * x207;
	const GEN_FLT x224 = (x209 + x55 * x208 + x56 * x215) * sensor_y + (x212 + x214 + x216) * sensor_x +
						 (-x221 - x222 + x223) * sensor_z;
	const GEN_FLT x225 = (x209 + x41 * x208 + x42 * x213) * sensor_z + (-x217 - x218 + x219) * sensor_x +
						 (x221 + x222 + x223) * sensor_y;
	const GEN_FLT x226 = x111 + x220 * x106 + x224 * x110 + x225 * x107;
	const GEN_FLT x227 = x81 + x5 * x220 + x50 * x225 + x54 * x224;
	const GEN_FLT x228 = x98 + x85 * x225 + x88 * x224 + x89 * x220;
	const GEN_FLT x229 = x227 * x122 + x228 * x123;
	const GEN_FLT x230 = -x126 * (x229 + x226 * x121) + x226 * x120;
	const GEN_FLT x231 = x230 * x138;
	const GEN_FLT x232 = x231 * x131;
	const GEN_FLT x233 = x128 * (x232 - x231 * x130) + x231 * x132;
	const GEN_FLT x234 = x231 * x134 + x233 * x128;
	const GEN_FLT x235 = x104 * (-x228 * x101 + x93 * x227);
	const GEN_FLT x236 = x226 * x148 - x229 * x155;
	const GEN_FLT x237 = x235 - x236 * x154;
	const GEN_FLT x238 =
		x174 *
		(x236 -
		 x166 * (-x153 * (x128 * (x234 + x128 * (x233 + x128 * (x232 + x231 * x129 - x231 * x142) + x231 * x133) +
								  x231 * x135) +
						  x230 * x139 + x230 * x145 + x234 * x128) -
				 x237 * x162) +
		 x231 * x169 + x234 * x170 + x237 * x172);
	const GEN_FLT x239 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x240 = x8 * x239;
	const GEN_FLT x241 = -x240;
	const GEN_FLT x242 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x243 = x14 * x239;
	const GEN_FLT x244 = x18 * x243;
	const GEN_FLT x245 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x246 = x8 * x245;
	const GEN_FLT x247 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x248 = x15 * x247;
	const GEN_FLT x249 = x11 * x248 + x26 * x242 + x23 * x11 * x240;
	const GEN_FLT x250 = x23 * x243;
	const GEN_FLT x251 = x8 * x247;
	const GEN_FLT x252 = x18 * x240;
	const GEN_FLT x253 = x11 * x252 + x16 * x245 + x32 * x242;
	const GEN_FLT x254 = (x241 + x12 * x240 + x17 * x242) * sensor_x + (-x244 - x246 + x249) * sensor_y +
						 (x250 + x251 + x253) * sensor_z;
	const GEN_FLT x255 = x11 * x243;
	const GEN_FLT x256 = x8 * x242;
	const GEN_FLT x257 = x18 * x248 + x23 * x252 + x26 * x245;
	const GEN_FLT x258 = (x241 + 2 * x23 * x248 + x55 * x240) * sensor_y + (x244 + x246 + x249) * sensor_x +
						 (-x255 - x256 + x257) * sensor_z;
	const GEN_FLT x259 = (x241 + x41 * x240 + x42 * x245) * sensor_z + (-x250 - x251 + x253) * sensor_x +
						 (x255 + x256 + x257) * sensor_y;
	const GEN_FLT x260 = x81 + x5 * x254 + x50 * x259 + x54 * x258;
	const GEN_FLT x261 = x98 + x85 * x259 + x88 * x258 + x89 * x254;
	const GEN_FLT x262 = x104 * (-x261 * x101 + x93 * x260);
	const GEN_FLT x263 = x111 + x254 * x106 + x258 * x110 + x259 * x107;
	const GEN_FLT x264 = x260 * x122 + x261 * x123;
	const GEN_FLT x265 = -x126 * (x264 + x263 * x121) + x263 * x120;
	const GEN_FLT x266 = x265 * x138;
	const GEN_FLT x267 = x266 * x131;
	const GEN_FLT x268 = x128 * (x267 - x266 * x130) + x266 * x132;
	const GEN_FLT x269 = x266 * x134 + x268 * x128;
	const GEN_FLT x270 = x263 * x148 - x264 * x155;
	const GEN_FLT x271 = x262 - x270 * x154;
	const GEN_FLT x272 =
		x174 *
		(x270 -
		 x166 * (-x153 * (x128 * (x269 + x128 * (x268 + x128 * (x267 + x266 * x129 - x266 * x142) + x266 * x133) +
								  x266 * x135) +
						  x265 * x139 + x266 * x136 + x269 * x128) -
				 x271 * x162) +
		 x266 * x169 + x269 * x170 + x271 * x172);
	const GEN_FLT x273 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x274 = x14 * x273;
	const GEN_FLT x275 = x18 * x274;
	const GEN_FLT x276 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x277 = x8 * x276;
	const GEN_FLT x278 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x279 = x15 * x278;
	const GEN_FLT x280 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x281 = x16 * x280 + x23 * x279 + x25 * x273;
	const GEN_FLT x282 = x11 * x274;
	const GEN_FLT x283 = x8 * x278;
	const GEN_FLT x284 = x26 * x276 + x32 * x280 + x39 * x273;
	const GEN_FLT x285 = x8 * x273;
	const GEN_FLT x286 = -x285;
	const GEN_FLT x287 = (x275 + x277 + x281) * sensor_x + (-x282 - x283 + x284) * sensor_z +
						 (x286 + x55 * x285 + x56 * x280) * sensor_y;
	const GEN_FLT x288 = x23 * x274;
	const GEN_FLT x289 = x8 * x280;
	const GEN_FLT x290 = x16 * x276 + x18 * x279 + x31 * x273;
	const GEN_FLT x291 = (-x275 - x277 + x281) * sensor_y + (x286 + 2 * x11 * x279 + x12 * x285) * sensor_x +
						 (x288 + x289 + x290) * sensor_z;
	const GEN_FLT x292 = (x282 + x283 + x284) * sensor_y + (x286 + x41 * x285 + x42 * x276) * sensor_z +
						 (-x288 - x289 + x290) * sensor_x;
	const GEN_FLT x293 = x81 + x5 * x291 + x50 * x292 + x54 * x287;
	const GEN_FLT x294 = x98 + x85 * x292 + x88 * x287 + x89 * x291;
	const GEN_FLT x295 = x104 * (-x294 * x101 + x93 * x293);
	const GEN_FLT x296 = x111 + x287 * x110 + x291 * x106 + x292 * x107;
	const GEN_FLT x297 = x293 * x122 + x294 * x123;
	const GEN_FLT x298 = -x126 * (x297 + x296 * x121) + x296 * x120;
	const GEN_FLT x299 = x298 * x138;
	const GEN_FLT x300 = x299 * x131;
	const GEN_FLT x301 = x128 * (x300 - x299 * x130) + x299 * x132;
	const GEN_FLT x302 = x299 * x134 + x301 * x128;
	const GEN_FLT x303 = x296 * x148 - x297 * x155;
	const GEN_FLT x304 = x295 - x303 * x154;
	const GEN_FLT x305 =
		x174 *
		(x303 -
		 x166 * (-x153 * (x128 * (x302 + x128 * (x301 + x128 * (x300 + x299 * x129 - x299 * x142) + x299 * x133) +
								  x299 * x135) +
						  x298 * x139 + x298 * x145 + x302 * x128) -
				 x304 * x162) +
		 x299 * x169 + x302 * x170 + x304 * x172);
	out[0] = x105 - x175 - (-x105 + x175) * x176;
	out[1] = x182 - x192 - (-x182 + x192) * x176;
	out[2] = x196 - x206 - (-x196 + x206) * x176;
	out[3] = x235 - x238 - (-x235 + x238) * x176;
	out[4] = x262 - x272 - (-x262 + x272) * x176;
	out[5] = x295 - x305 - (-x295 + x305) * x176;
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 - x1;
	const GEN_FLT x3 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x4 = pow(x3, 2);
	const GEN_FLT x5 = x1 + x2 * x4;
	const GEN_FLT x6 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1e-10));
	const GEN_FLT x7 = cos(x6);
	const GEN_FLT x8 = 1 - x7;
	const GEN_FLT x9 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								  : (1e-10)))
							? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											 : (1e-10)))
							: (1));
	const GEN_FLT x10 = pow(x9, 2);
	const GEN_FLT x11 = x7 + x8 * x10;
	const GEN_FLT x12 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x13 = sin(x6);
	const GEN_FLT x14 = x13 * x12;
	const GEN_FLT x15 = -x14;
	const GEN_FLT x16 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x17 = x8 * x9;
	const GEN_FLT x18 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x19 = x7 * x12;
	const GEN_FLT x20 = x19 * x18;
	const GEN_FLT x21 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x22 = x21 * x13;
	const GEN_FLT x23 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x24 = x23 * x13;
	const GEN_FLT x25 = x9 * x12;
	const GEN_FLT x26 = x8 * x16;
	const GEN_FLT x27 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x28 = x8 * x27;
	const GEN_FLT x29 = x23 * x26 + x24 * x25 + x9 * x28;
	const GEN_FLT x30 = x23 * x19;
	const GEN_FLT x31 = x27 * x13;
	const GEN_FLT x32 = x13 * x18;
	const GEN_FLT x33 = x21 * x17 + x26 * x18 + x32 * x25;
	const GEN_FLT x34 =
		(x15 + x14 * x10 + 2 * x17 * x16) * sensor_x + (-x20 - x22 + x29) * sensor_y + (x30 + x31 + x33) * sensor_z;
	const GEN_FLT x35 = x11 + x34;
	const GEN_FLT x36 = sin(x0);
	const GEN_FLT x37 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x38 = x36 * x37;
	const GEN_FLT x39 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x40 = x3 * x39;
	const GEN_FLT x41 = x2 * x40;
	const GEN_FLT x42 = -x38 + x41;
	const GEN_FLT x43 = x23 * x17;
	const GEN_FLT x44 = x32 + x43;
	const GEN_FLT x45 = x9 * x19;
	const GEN_FLT x46 = x13 * x16;
	const GEN_FLT x47 = x8 * x23;
	const GEN_FLT x48 = x28 * x18 + x47 * x21 + x24 * x12 * x18;
	const GEN_FLT x49 = pow(x23, 2);
	const GEN_FLT x50 =
		(x15 + 2 * x23 * x28 + x49 * x14) * sensor_y + (x20 + x22 + x29) * sensor_x + (-x45 - x46 + x48) * sensor_z;
	const GEN_FLT x51 = x44 + x50;
	const GEN_FLT x52 = x18 * x17;
	const GEN_FLT x53 = -x24 + x52;
	const GEN_FLT x54 = pow(x18, 2);
	const GEN_FLT x55 = (x15 + x54 * x14 + 2 * x8 * x21 * x18) * sensor_z + (-x30 - x31 + x33) * sensor_x +
						(x45 + x46 + x48) * sensor_y;
	const GEN_FLT x56 = x53 + x55;
	const GEN_FLT x57 = x36 * x39;
	const GEN_FLT x58 = x2 * x37;
	const GEN_FLT x59 = x3 * x58;
	const GEN_FLT x60 = x57 + x59;
	const GEN_FLT x61 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x62 = x61 * x36;
	const GEN_FLT x63 = -x62;
	const GEN_FLT x64 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x65 = x2 * x64;
	const GEN_FLT x66 = x24 + x52;
	const GEN_FLT x67 = -x32 + x43;
	const GEN_FLT x68 = obj_px + x11 * sensor_x + x66 * sensor_z + x67 * sensor_y;
	const GEN_FLT x69 = x7 + x8 * x49;
	const GEN_FLT x70 = x9 * x13;
	const GEN_FLT x71 = x47 * x18;
	const GEN_FLT x72 = -x70 + x71;
	const GEN_FLT x73 = obj_py + x44 * sensor_x + x69 * sensor_y + x72 * sensor_z;
	const GEN_FLT x74 = x1 * x61;
	const GEN_FLT x75 = x74 * x37;
	const GEN_FLT x76 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x77 = x76 * x36;
	const GEN_FLT x78 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x79 = x2 * x78;
	const GEN_FLT x80 = x3 * x79 + x62 * x40 + x65 * x39;
	const GEN_FLT x81 = x7 + x8 * x54;
	const GEN_FLT x82 = x70 + x71;
	const GEN_FLT x83 = obj_pz + x53 * sensor_x + x81 * sensor_z + x82 * sensor_y;
	const GEN_FLT x84 = x74 * x39;
	const GEN_FLT x85 = x78 * x36;
	const GEN_FLT x86 = x62 * x37;
	const GEN_FLT x87 = x2 * x76;
	const GEN_FLT x88 = x3 * x86 + x3 * x87 + x65 * x37;
	const GEN_FLT x89 = x68 * (x63 + 2 * x3 * x65 + x4 * x62) + x73 * (-x75 - x77 + x80) + x83 * (x84 + x85 + x88);
	const GEN_FLT x90 = x89 + x5 * x35 + x51 * x42 + x60 * x56;
	const GEN_FLT x91 = pow(x37, 2);
	const GEN_FLT x92 = x1 + x2 * x91;
	const GEN_FLT x93 = x3 * x36;
	const GEN_FLT x94 = x58 * x39;
	const GEN_FLT x95 = x93 + x94;
	const GEN_FLT x96 = -x57 + x59;
	const GEN_FLT x97 = lh_pz + x68 * x96 + x73 * x95 + x83 * x92;
	const GEN_FLT x98 = lh_px + x5 * x68 + x73 * x42 + x83 * x60;
	const GEN_FLT x99 = pow(x98, 2);
	const GEN_FLT x100 = x97 / x99;
	const GEN_FLT x101 = pow(x98, -1);
	const GEN_FLT x102 = x3 * x74;
	const GEN_FLT x103 = x64 * x36;
	const GEN_FLT x104 = x79 * x37 + x86 * x39 + x87 * x39;
	const GEN_FLT x105 =
		x68 * (-x84 - x85 + x88) + x73 * (x102 + x103 + x104) + x83 * (x63 + x62 * x91 + 2 * x87 * x37);
	const GEN_FLT x106 = x105 + x51 * x95 + x56 * x92 + x96 * x35;
	const GEN_FLT x107 = x99 + pow(x97, 2);
	const GEN_FLT x108 = pow(x107, -1);
	const GEN_FLT x109 = x99 * x108;
	const GEN_FLT x110 = x109 * (-x101 * x106 + x90 * x100);
	const GEN_FLT x111 = -x93 + x94;
	const GEN_FLT x112 = pow(x39, 2);
	const GEN_FLT x113 = x1 + x2 * x112;
	const GEN_FLT x114 = x38 + x41;
	const GEN_FLT x115 = lh_py + x68 * x114 + x73 * x113 + x83 * x111;
	const GEN_FLT x116 = pow(x115, 2);
	const GEN_FLT x117 = x107 + x116;
	const GEN_FLT x118 = 0.523598775598299 + tilt_0;
	const GEN_FLT x119 = cos(x118);
	const GEN_FLT x120 = pow(x119, -1);
	const GEN_FLT x121 = x120 / sqrt(x117);
	const GEN_FLT x122 = asin(x115 * x121);
	const GEN_FLT x123 = -8.0108022e-06 - 1.60216044e-05 * x122;
	const GEN_FLT x124 = 8.0108022e-06 * x122;
	const GEN_FLT x125 = -8.0108022e-06 - x124;
	const GEN_FLT x126 = 0.0028679863 + x122 * x125;
	const GEN_FLT x127 = x126 + x123 * x122;
	const GEN_FLT x128 = 5.3685255e-06 + x122 * x126;
	const GEN_FLT x129 = x128 + x122 * x127;
	const GEN_FLT x130 = 0.0076069798 + x122 * x128;
	const GEN_FLT x131 = x130 + x122 * x129;
	const GEN_FLT x132 = pow(1 - x116 / (pow(x119, 2) * x117), -1.0 / 2.0);
	const GEN_FLT x133 =
		x68 * (x75 + x77 + x80) + x73 * (x63 + x62 * x112 + 2 * x79 * x39) + x83 * (-x102 - x103 + x104);
	const GEN_FLT x134 = x133 + x35 * x114 + x51 * x113 + x56 * x111;
	const GEN_FLT x135 = 2 * x115;
	const GEN_FLT x136 = 2 * x98;
	const GEN_FLT x137 = 2 * x97;
	const GEN_FLT x138 = x106 * x137 + x90 * x136;
	const GEN_FLT x139 = (1.0 / 2.0) * x115;
	const GEN_FLT x140 = x120 * x139 / pow(x117, 3.0 / 2.0);
	const GEN_FLT x141 = x121 * x134 - x140 * (x138 + x134 * x135);
	const GEN_FLT x142 = x132 * x141;
	const GEN_FLT x143 = x125 * x142;
	const GEN_FLT x144 = 2.40324066e-05 * x122;
	const GEN_FLT x145 = x122 * (x143 - x124 * x142) + x126 * x142;
	const GEN_FLT x146 = x128 * x132;
	const GEN_FLT x147 = x122 * x145 + x141 * x146;
	const GEN_FLT x148 = sin(x118);
	const GEN_FLT x149 = tan(x118);
	const GEN_FLT x150 = x149 / sqrt(x107);
	const GEN_FLT x151 = x115 * x150;
	const GEN_FLT x152 = atan2(-x97, x98);
	const GEN_FLT x153 = ogeeMag_0 + x152 - asin(x151);
	const GEN_FLT x154 = curve_0 + sin(x153) * ogeePhase_0;
	const GEN_FLT x155 = x148 * x154;
	const GEN_FLT x156 = x139 * x149 / pow(x107, 3.0 / 2.0);
	const GEN_FLT x157 = x134 * x150 - x138 * x156;
	const GEN_FLT x158 = pow(1 - x108 * x116 * pow(x149, 2), -1.0 / 2.0);
	const GEN_FLT x159 = x110 - x157 * x158;
	const GEN_FLT x160 = cos(x153) * ogeePhase_0;
	const GEN_FLT x161 = x122 * x130;
	const GEN_FLT x162 = x161 + x122 * x131;
	const GEN_FLT x163 = x162 * x148;
	const GEN_FLT x164 = x160 * x163;
	const GEN_FLT x165 = x119 - x162 * x155;
	const GEN_FLT x166 = pow(x122, 2);
	const GEN_FLT x167 = x166 * x154;
	const GEN_FLT x168 = x167 * x130 / pow(x165, 2);
	const GEN_FLT x169 = pow(x165, -1);
	const GEN_FLT x170 = 2 * x161 * x169 * x154;
	const GEN_FLT x171 = x169 * x167;
	const GEN_FLT x172 = x169 * x166 * x130;
	const GEN_FLT x173 = x160 * x172;
	const GEN_FLT x174 = x151 + x171 * x130;
	const GEN_FLT x175 = pow(1 - pow(x174, 2), -1.0 / 2.0);
	const GEN_FLT x176 =
		x175 *
		(x157 -
		 x168 * (-x155 * (x122 * x147 +
						  x122 * (x147 + x122 * (x145 + x122 * (x143 + x123 * x142 - x142 * x144) + x127 * x142) +
								  x129 * x142) +
						  x130 * x142 + x131 * x142) -
				 x164 * x159) +
		 x170 * x142 + x171 * x147 + x173 * x159);
	const GEN_FLT x177 = cos(gibPhase_0 + x152 - asin(x174)) * gibMag_0;
	const GEN_FLT x178 = x50 + x69;
	const GEN_FLT x179 = x34 + x67;
	const GEN_FLT x180 = x55 + x82;
	const GEN_FLT x181 = x89 + x42 * x178 + x5 * x179 + x60 * x180;
	const GEN_FLT x182 = x105 + x92 * x180 + x95 * x178 + x96 * x179;
	const GEN_FLT x183 = (x100 * x181 - x101 * x182) * x109;
	const GEN_FLT x184 = x133 + x111 * x180 + x113 * x178 + x114 * x179;
	const GEN_FLT x185 = x181 * x136 + x182 * x137;
	const GEN_FLT x186 = x121 * x184 - x140 * (x185 + x184 * x135);
	const GEN_FLT x187 = x186 * x132;
	const GEN_FLT x188 = x125 * x187;
	const GEN_FLT x189 = x122 * (x188 - x124 * x187) + x126 * x187;
	const GEN_FLT x190 = x122 * x189 + x186 * x146;
	const GEN_FLT x191 = x184 * x150 - x185 * x156;
	const GEN_FLT x192 = x160 * (x183 - x191 * x158);
	const GEN_FLT x193 =
		x175 *
		(x191 -
		 x168 * (-x155 * (x122 * x190 +
						  x122 * (x190 + x122 * (x189 + x122 * (x188 + x123 * x187 - x187 * x144) + x127 * x187) +
								  x129 * x187) +
						  x187 * x130 + x187 * x131) -
				 x163 * x192) +
		 x170 * x187 + x171 * x190 + x172 * x192);
	const GEN_FLT x194 = x34 + x66;
	const GEN_FLT x195 = x50 + x72;
	const GEN_FLT x196 = x55 + x81;
	const GEN_FLT x197 = x89 + x42 * x195 + x5 * x194 + x60 * x196;
	const GEN_FLT x198 = x105 + x92 * x196 + x95 * x195 + x96 * x194;
	const GEN_FLT x199 = (x100 * x197 - x101 * x198) * x109;
	const GEN_FLT x200 = x133 + x111 * x196 + x113 * x195 + x114 * x194;
	const GEN_FLT x201 = x197 * x136 + x198 * x137;
	const GEN_FLT x202 = -x140 * (x201 + x200 * x135) + x200 * x121;
	const GEN_FLT x203 = x202 * x132;
	const GEN_FLT x204 = x203 * x125;
	const GEN_FLT x205 = x122 * (x204 - x203 * x124) + x203 * x126;
	const GEN_FLT x206 = x202 * x146 + x205 * x122;
	const GEN_FLT x207 = x200 * x150 - x201 * x156;
	const GEN_FLT x208 = x199 - x207 * x158;
	const GEN_FLT x209 =
		x175 *
		(x207 -
		 x168 * (-x155 * (x122 * (x206 + x122 * (x205 + x122 * (x204 + x203 * x123 - x203 * x144) + x203 * x127) +
								  x203 * x129) +
						  x203 * x130 + x203 * x131 + x206 * x122) -
				 x208 * x164) +
		 x203 * x170 + x206 * x171 + x208 * x173);
	out[0] = x110 - x176 - (-x110 + x176) * x177;
	out[1] = x183 - x193 - (-x183 + x193) * x177;
	out[2] = x199 - x209 - (-x199 + x209) * x177;
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = -x3;
	const GEN_FLT x5 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x6 = pow(x5, 2);
	const GEN_FLT x7 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
							   pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										: (1e-10)),
								   2))
							: (0));
	const GEN_FLT x8 = cos(x0);
	const GEN_FLT x9 = 1 - x8;
	const GEN_FLT x10 = x5 * x9;
	const GEN_FLT x11 = 2 * x10;
	const GEN_FLT x12 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1e-10));
	const GEN_FLT x13 = sin(x12);
	const GEN_FLT x14 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x15 = x14 * x13;
	const GEN_FLT x16 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x17 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x18 = cos(x12);
	const GEN_FLT x19 = 1 - x18;
	const GEN_FLT x20 = x19 * x17;
	const GEN_FLT x21 = x20 * x16;
	const GEN_FLT x22 = x13 * x17;
	const GEN_FLT x23 = x14 * x19;
	const GEN_FLT x24 = x23 * x16;
	const GEN_FLT x25 = pow(x16, 2);
	const GEN_FLT x26 = obj_px + (x18 + x25 * x19) * sensor_x + (x15 + x21) * sensor_z + (-x22 + x24) * sensor_y;
	const GEN_FLT x27 = pow(x14, 2);
	const GEN_FLT x28 = x13 * x16;
	const GEN_FLT x29 = x23 * x17;
	const GEN_FLT x30 = obj_py + (x18 + x27 * x19) * sensor_y + (x22 + x24) * sensor_x + (-x28 + x29) * sensor_z;
	const GEN_FLT x31 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x32 = x2 * x8;
	const GEN_FLT x33 = x32 * x31;
	const GEN_FLT x34 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x35 = x1 * x34;
	const GEN_FLT x36 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x37 = x5 * x36;
	const GEN_FLT x38 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x39 = x9 * x36;
	const GEN_FLT x40 = x3 * x37 + x38 * x10 + x7 * x39;
	const GEN_FLT x41 = pow(x17, 2);
	const GEN_FLT x42 = obj_pz + (x18 + x41 * x19) * sensor_z + (-x15 + x21) * sensor_x + (x28 + x29) * sensor_y;
	const GEN_FLT x43 = x32 * x36;
	const GEN_FLT x44 = x1 * x38;
	const GEN_FLT x45 = x9 * x31;
	const GEN_FLT x46 = x5 * x31;
	const GEN_FLT x47 = x3 * x46 + x34 * x10 + x7 * x45;
	const GEN_FLT x48 = x8 + x6 * x9;
	const GEN_FLT x49 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x50 = x49 * x13;
	const GEN_FLT x51 = -x50;
	const GEN_FLT x52 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x53 = x49 * x18;
	const GEN_FLT x54 = x53 * x17;
	const GEN_FLT x55 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x56 = x55 * x13;
	const GEN_FLT x57 = x49 * x16;
	const GEN_FLT x58 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x59 = x58 * x19;
	const GEN_FLT x60 = x52 * x23 + x57 * x15 + x59 * x16;
	const GEN_FLT x61 = x53 * x14;
	const GEN_FLT x62 = x58 * x13;
	const GEN_FLT x63 = x55 * x19;
	const GEN_FLT x64 = x52 * x20 + x57 * x22 + x63 * x16;
	const GEN_FLT x65 = (x51 + x50 * x25 + 2 * x52 * x19 * x16) * sensor_x + (-x54 - x56 + x60) * sensor_y +
						(x61 + x62 + x64) * sensor_z;
	const GEN_FLT x66 = x1 * x31;
	const GEN_FLT x67 = x36 * x10;
	const GEN_FLT x68 = -x66 + x67;
	const GEN_FLT x69 = x53 * x16;
	const GEN_FLT x70 = x52 * x13;
	const GEN_FLT x71 = x59 * x17 + x63 * x14 + x49 * x15 * x17;
	const GEN_FLT x72 =
		(x51 + x50 * x27 + 2 * x59 * x14) * sensor_y + (x54 + x56 + x60) * sensor_x + (-x69 - x70 + x71) * sensor_z;
	const GEN_FLT x73 =
		(x51 + x50 * x41 + 2 * x63 * x17) * sensor_z + (-x61 - x62 + x64) * sensor_x + (x69 + x70 + x71) * sensor_y;
	const GEN_FLT x74 = x1 * x36;
	const GEN_FLT x75 = x5 * x45;
	const GEN_FLT x76 = x74 + x75;
	const GEN_FLT x77 = x65 * x48 + x72 * x68 + x73 * x76;
	const GEN_FLT x78 = x77 + x26 * (x4 + x3 * x6 + x7 * x11) + x30 * (-x33 - x35 + x40) + x42 * (x43 + x44 + x47);
	const GEN_FLT x79 = 1 + x78;
	const GEN_FLT x80 = pow(x31, 2);
	const GEN_FLT x81 = x8 + x9 * x80;
	const GEN_FLT x82 = x1 * x5;
	const GEN_FLT x83 = x45 * x36;
	const GEN_FLT x84 = x82 + x83;
	const GEN_FLT x85 = -x74 + x75;
	const GEN_FLT x86 = lh_pz + x81 * x42 + x84 * x30 + x85 * x26;
	const GEN_FLT x87 = lh_px + x48 * x26 + x68 * x30 + x76 * x42;
	const GEN_FLT x88 = pow(x87, 2);
	const GEN_FLT x89 = x86 / x88;
	const GEN_FLT x90 = x5 * x32;
	const GEN_FLT x91 = x1 * x7;
	const GEN_FLT x92 = x31 * x36;
	const GEN_FLT x93 = x3 * x92 + x34 * x39 + x45 * x38;
	const GEN_FLT x94 = 2 * x45;
	const GEN_FLT x95 = x81 * x73 + x84 * x72 + x85 * x65;
	const GEN_FLT x96 = x95 + x26 * (-x43 - x44 + x47) + x30 * (x90 + x91 + x93) + x42 * (x4 + x3 * x80 + x94 * x34);
	const GEN_FLT x97 = pow(x87, -1);
	const GEN_FLT x98 = -x97 * x96;
	const GEN_FLT x99 = x88 + pow(x86, 2);
	const GEN_FLT x100 = pow(x99, -1);
	const GEN_FLT x101 = x88 * x100;
	const GEN_FLT x102 = x101 * (x98 + x89 * x79);
	const GEN_FLT x103 = -x82 + x83;
	const GEN_FLT x104 = pow(x36, 2);
	const GEN_FLT x105 = x8 + x9 * x104;
	const GEN_FLT x106 = x66 + x67;
	const GEN_FLT x107 = lh_py + x26 * x106 + x30 * x105 + x42 * x103;
	const GEN_FLT x108 = pow(x107, 2);
	const GEN_FLT x109 = x108 + x99;
	const GEN_FLT x110 = 0.523598775598299 + tilt_0;
	const GEN_FLT x111 = cos(x110);
	const GEN_FLT x112 = pow(x111, -1);
	const GEN_FLT x113 = x112 / sqrt(x109);
	const GEN_FLT x114 = asin(x107 * x113);
	const GEN_FLT x115 = -8.0108022e-06 - 1.60216044e-05 * x114;
	const GEN_FLT x116 = 8.0108022e-06 * x114;
	const GEN_FLT x117 = -8.0108022e-06 - x116;
	const GEN_FLT x118 = 0.0028679863 + x114 * x117;
	const GEN_FLT x119 = x118 + x114 * x115;
	const GEN_FLT x120 = 5.3685255e-06 + x118 * x114;
	const GEN_FLT x121 = x120 + x119 * x114;
	const GEN_FLT x122 = 0.0076069798 + x114 * x120;
	const GEN_FLT x123 = x122 + x114 * x121;
	const GEN_FLT x124 = pow(1 - x108 / (x109 * pow(x111, 2)), -1.0 / 2.0);
	const GEN_FLT x125 = 2 * x39;
	const GEN_FLT x126 = x65 * x106 + x72 * x105 + x73 * x103;
	const GEN_FLT x127 =
		x126 + x26 * (x33 + x35 + x40) + x30 * (x4 + x3 * x104 + x38 * x125) + x42 * (-x90 - x91 + x93);
	const GEN_FLT x128 = x113 * x127;
	const GEN_FLT x129 = 2 * x107;
	const GEN_FLT x130 = x127 * x129;
	const GEN_FLT x131 = 2 * x87;
	const GEN_FLT x132 = 2 * x86;
	const GEN_FLT x133 = x96 * x132;
	const GEN_FLT x134 = x133 + x79 * x131;
	const GEN_FLT x135 = (1.0 / 2.0) * x107;
	const GEN_FLT x136 = x112 * x135 / pow(x109, 3.0 / 2.0);
	const GEN_FLT x137 = x124 * (x128 - (x130 + x134) * x136);
	const GEN_FLT x138 = x117 * x137;
	const GEN_FLT x139 = 2.40324066e-05 * x114;
	const GEN_FLT x140 = x114 * (x138 - x116 * x137) + x118 * x137;
	const GEN_FLT x141 = x114 * x140 + x120 * x137;
	const GEN_FLT x142 = sin(x110);
	const GEN_FLT x143 = tan(x110);
	const GEN_FLT x144 = x143 / sqrt(x99);
	const GEN_FLT x145 = x107 * x144;
	const GEN_FLT x146 = atan2(-x86, x87);
	const GEN_FLT x147 = ogeeMag_0 + x146 - asin(x145);
	const GEN_FLT x148 = curve_0 + sin(x147) * ogeePhase_0;
	const GEN_FLT x149 = x142 * x148;
	const GEN_FLT x150 = pow(1 - x100 * x108 * pow(x143, 2), -1.0 / 2.0);
	const GEN_FLT x151 = x135 * x143 / pow(x99, 3.0 / 2.0);
	const GEN_FLT x152 = x127 * x144;
	const GEN_FLT x153 = x152 - x134 * x151;
	const GEN_FLT x154 = x102 - x150 * x153;
	const GEN_FLT x155 = cos(x147) * ogeePhase_0;
	const GEN_FLT x156 = x114 * x122;
	const GEN_FLT x157 = x156 + x114 * x123;
	const GEN_FLT x158 = x142 * x157;
	const GEN_FLT x159 = x155 * x158;
	const GEN_FLT x160 = x111 - x149 * x157;
	const GEN_FLT x161 = pow(x114, 2);
	const GEN_FLT x162 = x161 * x148;
	const GEN_FLT x163 = x122 * x162 / pow(x160, 2);
	const GEN_FLT x164 = pow(x160, -1);
	const GEN_FLT x165 = 2 * x164 * x148 * x156;
	const GEN_FLT x166 = x164 * x162;
	const GEN_FLT x167 = x122 * x161 * x164;
	const GEN_FLT x168 = x167 * x155;
	const GEN_FLT x169 = x145 + x122 * x166;
	const GEN_FLT x170 = pow(1 - pow(x169, 2), -1.0 / 2.0);
	const GEN_FLT x171 =
		x170 *
		(x153 -
		 x163 * (-x149 * (x114 * x141 +
						  x114 * (x141 + x114 * (x140 + x114 * (x138 + x115 * x137 - x137 * x139) + x119 * x137) +
								  x121 * x137) +
						  x122 * x137 + x123 * x137) -
				 x154 * x159) +
		 x165 * x137 + x166 * x141 + x168 * x154);
	const GEN_FLT x172 = cos(gibPhase_0 + x146 - asin(x169)) * gibMag_0;
	const GEN_FLT x173 = x89 * x78;
	const GEN_FLT x174 = x101 * (x173 + x98);
	const GEN_FLT x175 = 1 + x127;
	const GEN_FLT x176 = x78 * x131;
	const GEN_FLT x177 = x133 + x176;
	const GEN_FLT x178 = x124 * (x113 * x175 - x136 * (x177 + x129 * x175));
	const GEN_FLT x179 = x117 * x178;
	const GEN_FLT x180 = x114 * (x179 - x116 * x178) + x118 * x178;
	const GEN_FLT x181 = x114 * x180 + x120 * x178;
	const GEN_FLT x182 = x175 * x144 - x177 * x151;
	const GEN_FLT x183 = x155 * (x174 - x182 * x150);
	const GEN_FLT x184 =
		x170 *
		(x182 -
		 x163 * (-x149 * (x114 * x181 +
						  x114 * (x181 + x114 * (x180 + x114 * (x179 + x115 * x178 - x178 * x139) + x119 * x178) +
								  x121 * x178) +
						  x122 * x178 + x123 * x178) -
				 x183 * x158) +
		 x165 * x178 + x166 * x181 + x167 * x183);
	const GEN_FLT x185 = 1 + x96;
	const GEN_FLT x186 = x101 * (x173 - x97 * x185);
	const GEN_FLT x187 = x176 + x185 * x132;
	const GEN_FLT x188 = x124 * (x128 - (x130 + x187) * x136);
	const GEN_FLT x189 = x117 * x188;
	const GEN_FLT x190 = x114 * (x189 - x116 * x188) + x118 * x188;
	const GEN_FLT x191 = x114 * x190 + x120 * x188;
	const GEN_FLT x192 = x152 - x187 * x151;
	const GEN_FLT x193 = x186 - x192 * x150;
	const GEN_FLT x194 =
		x170 *
		(x192 -
		 x163 * (-x149 * (x114 * x191 +
						  x114 * (x191 + x114 * (x190 + x114 * (x189 + x115 * x188 - x188 * x139) + x119 * x188) +
								  x121 * x188) +
						  x122 * x188 + x123 * x188) -
				 x193 * x159) +
		 x165 * x188 + x166 * x191 + x168 * x193);
	const GEN_FLT x195 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x196 = x1 * x195;
	const GEN_FLT x197 = -x196;
	const GEN_FLT x198 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x199 = x8 * x195;
	const GEN_FLT x200 = x31 * x199;
	const GEN_FLT x201 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x202 = x1 * x201;
	const GEN_FLT x203 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x204 = x10 * x203 + x37 * x196 + x39 * x198;
	const GEN_FLT x205 = x36 * x199;
	const GEN_FLT x206 = x1 * x203;
	const GEN_FLT x207 = x10 * x201 + x45 * x198 + x46 * x196;
	const GEN_FLT x208 =
		x77 + x26 * (x197 + x11 * x198 + x6 * x196) + x30 * (-x200 - x202 + x204) + x42 * (x205 + x206 + x207);
	const GEN_FLT x209 = x5 * x199;
	const GEN_FLT x210 = x1 * x198;
	const GEN_FLT x211 = x39 * x201 + x45 * x203 + x92 * x196;
	const GEN_FLT x212 =
		x95 + x26 * (-x205 - x206 + x207) + x30 * (x209 + x210 + x211) + x42 * (x197 + x80 * x196 + x94 * x201);
	const GEN_FLT x213 = (x89 * x208 - x97 * x212) * x101;
	const GEN_FLT x214 =
		x126 + x26 * (x200 + x202 + x204) + x30 * (x197 + x104 * x196 + x203 * x125) + x42 * (-x209 - x210 + x211);
	const GEN_FLT x215 = x208 * x131 + x212 * x132;
	const GEN_FLT x216 = x124 * (-x136 * (x215 + x214 * x129) + x214 * x113);
	const GEN_FLT x217 = x216 * x117;
	const GEN_FLT x218 = x114 * (x217 - x216 * x116) + x216 * x118;
	const GEN_FLT x219 = x216 * x120 + x218 * x114;
	const GEN_FLT x220 = x214 * x144 - x215 * x151;
	const GEN_FLT x221 = x213 - x220 * x150;
	const GEN_FLT x222 =
		x170 *
		(x220 -
		 x163 * (-x149 * (x114 * (x219 + x114 * (x218 + x114 * (x217 + x216 * x115 - x216 * x139) + x216 * x119) +
								  x216 * x121) +
						  x216 * x122 + x216 * x123 + x219 * x114) -
				 x221 * x159) +
		 x216 * x165 + x219 * x166 + x221 * x168);
	const GEN_FLT x223 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x224 = x1 * x223;
	const GEN_FLT x225 = -x224;
	const GEN_FLT x226 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x227 = x8 * x223;
	const GEN_FLT x228 = x31 * x227;
	const GEN_FLT x229 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x230 = x1 * x229;
	const GEN_FLT x231 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x232 = x10 * x231 + x37 * x224 + x39 * x226;
	const GEN_FLT x233 = x36 * x227;
	const GEN_FLT x234 = x1 * x231;
	const GEN_FLT x235 = x10 * x229 + x45 * x226 + x46 * x224;
	const GEN_FLT x236 =
		x77 + x26 * (x225 + x11 * x226 + x6 * x224) + x30 * (-x228 - x230 + x232) + x42 * (x233 + x234 + x235);
	const GEN_FLT x237 = x5 * x227;
	const GEN_FLT x238 = x1 * x226;
	const GEN_FLT x239 = x39 * x229 + x45 * x231 + x92 * x224;
	const GEN_FLT x240 =
		x95 + x26 * (-x233 - x234 + x235) + x30 * (x237 + x238 + x239) + x42 * (x225 + x80 * x224 + x94 * x229);
	const GEN_FLT x241 = (x89 * x236 - x97 * x240) * x101;
	const GEN_FLT x242 =
		x126 + x26 * (x228 + x230 + x232) + x30 * (x225 + x224 * x104 + x231 * x125) + x42 * (-x237 - x238 + x239);
	const GEN_FLT x243 = x236 * x131 + x240 * x132;
	const GEN_FLT x244 = x124 * (-x136 * (x243 + x242 * x129) + x242 * x113);
	const GEN_FLT x245 = x244 * x117;
	const GEN_FLT x246 = x114 * (x245 - x244 * x116) + x244 * x118;
	const GEN_FLT x247 = x244 * x120 + x246 * x114;
	const GEN_FLT x248 = x242 * x144 - x243 * x151;
	const GEN_FLT x249 = x241 - x248 * x150;
	const GEN_FLT x250 =
		x170 *
		(x248 -
		 x163 * (-x149 * (x114 * (x247 + x114 * (x246 + x114 * (x245 + x244 * x115 - x244 * x139) + x244 * x119) +
								  x244 * x121) +
						  x244 * x122 + x244 * x123 + x247 * x114) -
				 x249 * x159) +
		 x244 * x165 + x247 * x166 + x249 * x168);
	const GEN_FLT x251 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x252 = x1 * x251;
	const GEN_FLT x253 = -x252;
	const GEN_FLT x254 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x255 = x8 * x251;
	const GEN_FLT x256 = x31 * x255;
	const GEN_FLT x257 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x258 = x1 * x257;
	const GEN_FLT x259 = x36 * x252;
	const GEN_FLT x260 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x261 = x10 * x260 + x39 * x254 + x5 * x259;
	const GEN_FLT x262 = x36 * x255;
	const GEN_FLT x263 = x1 * x260;
	const GEN_FLT x264 = x10 * x257 + x45 * x254 + x46 * x252;
	const GEN_FLT x265 =
		x77 + x26 * (x253 + x11 * x254 + x6 * x252) + x30 * (-x256 - x258 + x261) + x42 * (x262 + x263 + x264);
	const GEN_FLT x266 = x5 * x255;
	const GEN_FLT x267 = x1 * x254;
	const GEN_FLT x268 = x31 * x259 + x39 * x257 + x45 * x260;
	const GEN_FLT x269 =
		x95 + x26 * (-x262 - x263 + x264) + x30 * (x266 + x267 + x268) + x42 * (x253 + x80 * x252 + x94 * x257);
	const GEN_FLT x270 = (x89 * x265 - x97 * x269) * x101;
	const GEN_FLT x271 =
		x126 + x26 * (x256 + x258 + x261) + x30 * (x253 + x252 * x104 + x260 * x125) + x42 * (-x266 - x267 + x268);
	const GEN_FLT x272 = x265 * x131 + x269 * x132;
	const GEN_FLT x273 = x124 * (-x136 * (x272 + x271 * x129) + x271 * x113);
	const GEN_FLT x274 = x273 * x117;
	const GEN_FLT x275 = x114 * (x274 - x273 * x116) + x273 * x118;
	const GEN_FLT x276 = x273 * x120 + x275 * x114;
	const GEN_FLT x277 = x271 * x144 - x272 * x151;
	const GEN_FLT x278 = x270 - x277 * x150;
	const GEN_FLT x279 =
		x170 *
		(x277 -
		 x163 * (-x149 * (x114 * (x276 + x114 * (x275 + x114 * (x274 + x273 * x115 - x273 * x139) + x273 * x119) +
								  x273 * x121) +
						  x273 * x122 + x273 * x123 + x276 * x114) -
				 x278 * x159) +
		 x273 * x165 + x276 * x166 + x278 * x168);
	out[0] = x102 - x171 - (-x102 + x171) * x172;
	out[1] = x174 - x184 - (-x174 + x184) * x172;
	out[2] = x186 - x194 - (-x186 + x194) * x172;
	out[3] = x213 - x222 - (-x213 + x222) * x172;
	out[4] = x241 - x250 - (-x241 + x250) * x172;
	out[5] = x270 - x279 - (-x270 + x279) * x172;
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = -x3;
	const GEN_FLT x5 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x6 = pow(x5, 2);
	const GEN_FLT x7 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
							   pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										: (1e-10)),
								   2))
							: (0));
	const GEN_FLT x8 = cos(x0);
	const GEN_FLT x9 = 1 - x8;
	const GEN_FLT x10 = x5 * x9;
	const GEN_FLT x11 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1e-10));
	const GEN_FLT x12 = sin(x11);
	const GEN_FLT x13 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x14 = x13 * x12;
	const GEN_FLT x15 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x16 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x17 = cos(x11);
	const GEN_FLT x18 = 1 - x17;
	const GEN_FLT x19 = x18 * x16;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x15 * x12;
	const GEN_FLT x22 = x13 * x19;
	const GEN_FLT x23 = pow(x16, 2);
	const GEN_FLT x24 = obj_px + (x17 + x23 * x18) * sensor_x + (x14 + x20) * sensor_z + (-x21 + x22) * sensor_y;
	const GEN_FLT x25 = x8 + x6 * x9;
	const GEN_FLT x26 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x27 = x26 * x12;
	const GEN_FLT x28 = -x27;
	const GEN_FLT x29 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x30 = x26 * x17;
	const GEN_FLT x31 = x30 * x15;
	const GEN_FLT x32 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x33 = x32 * x12;
	const GEN_FLT x34 = x26 * x14;
	const GEN_FLT x35 = x13 * x18;
	const GEN_FLT x36 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x37 = x36 * x18;
	const GEN_FLT x38 = x34 * x16 + x35 * x29 + x37 * x16;
	const GEN_FLT x39 = x30 * x13;
	const GEN_FLT x40 = x36 * x12;
	const GEN_FLT x41 = x15 * x18;
	const GEN_FLT x42 = x32 * x19 + x41 * x29 + x21 * x26 * x16;
	const GEN_FLT x43 =
		(x28 + x23 * x27 + 2 * x29 * x19) * sensor_x + (-x31 - x33 + x38) * sensor_y + (x39 + x40 + x42) * sensor_z;
	const GEN_FLT x44 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x45 = x1 * x44;
	const GEN_FLT x46 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x47 = x9 * x46;
	const GEN_FLT x48 = x5 * x47;
	const GEN_FLT x49 = -x45 + x48;
	const GEN_FLT x50 = x30 * x16;
	const GEN_FLT x51 = x29 * x12;
	const GEN_FLT x52 = x32 * x35 + x34 * x15 + x37 * x15;
	const GEN_FLT x53 = pow(x13, 2);
	const GEN_FLT x54 =
		(x28 + 2 * x37 * x13 + x53 * x27) * sensor_y + (x31 + x33 + x38) * sensor_x + (-x50 - x51 + x52) * sensor_z;
	const GEN_FLT x55 = x12 * x16;
	const GEN_FLT x56 = x35 * x15;
	const GEN_FLT x57 = obj_py + (x17 + x53 * x18) * sensor_y + (x21 + x22) * sensor_x + (-x55 + x56) * sensor_z;
	const GEN_FLT x58 = x2 * x8;
	const GEN_FLT x59 = x58 * x44;
	const GEN_FLT x60 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x61 = x1 * x60;
	const GEN_FLT x62 = x3 * x5;
	const GEN_FLT x63 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x64 = x9 * x63;
	const GEN_FLT x65 = x5 * x64 + x62 * x46 + x7 * x47;
	const GEN_FLT x66 = pow(x15, 2);
	const GEN_FLT x67 = obj_pz + (x17 + x66 * x18) * sensor_z + (-x14 + x20) * sensor_x + (x55 + x56) * sensor_y;
	const GEN_FLT x68 = x58 * x46;
	const GEN_FLT x69 = x1 * x63;
	const GEN_FLT x70 = x9 * x44;
	const GEN_FLT x71 = x60 * x10 + x62 * x44 + x7 * x70;
	const GEN_FLT x72 =
		(x28 + 2 * x41 * x32 + x66 * x27) * sensor_z + (-x39 - x40 + x42) * sensor_x + (x50 + x51 + x52) * sensor_y;
	const GEN_FLT x73 = x1 * x46;
	const GEN_FLT x74 = x44 * x10;
	const GEN_FLT x75 = x73 + x74;
	const GEN_FLT x76 = x24 * (x4 + x3 * x6 + 2 * x7 * x10) + x43 * x25 + x54 * x49 + x57 * (-x59 - x61 + x65) +
						x67 * (x68 + x69 + x71) + x72 * x75;
	const GEN_FLT x77 = pow(x44, 2);
	const GEN_FLT x78 = x8 + x9 * x77;
	const GEN_FLT x79 = x1 * x5;
	const GEN_FLT x80 = x44 * x47;
	const GEN_FLT x81 = x79 + x80;
	const GEN_FLT x82 = -x73 + x74;
	const GEN_FLT x83 = lh_pz + x78 * x67 + x81 * x57 + x82 * x24;
	const GEN_FLT x84 = lh_px + x24 * x25 + x57 * x49 + x75 * x67;
	const GEN_FLT x85 = pow(x84, 2);
	const GEN_FLT x86 = x5 * x58;
	const GEN_FLT x87 = x1 * x7;
	const GEN_FLT x88 = x60 * x47 + x64 * x44 + x3 * x44 * x46;
	const GEN_FLT x89 = x24 * (-x68 - x69 + x71) + x57 * (x86 + x87 + x88) + x67 * (x4 + x3 * x77 + 2 * x70 * x60) +
						x72 * x78 + x81 * x54 + x82 * x43;
	const GEN_FLT x90 = x85 + pow(x83, 2);
	const GEN_FLT x91 = pow(x90, -1);
	const GEN_FLT x92 = x85 * x91 * (-x89 / x84 + x83 * x76 / x85);
	const GEN_FLT x93 = -x92;
	const GEN_FLT x94 = -x79 + x80;
	const GEN_FLT x95 = pow(x46, 2);
	const GEN_FLT x96 = x8 + x9 * x95;
	const GEN_FLT x97 = x45 + x48;
	const GEN_FLT x98 = lh_py + x57 * x96 + x67 * x94 + x97 * x24;
	const GEN_FLT x99 = pow(x98, 2);
	const GEN_FLT x100 = x90 + x99;
	const GEN_FLT x101 = pow(x100, -1.0 / 2.0);
	const GEN_FLT x102 = 0.523598775598299 + tilt_0;
	const GEN_FLT x103 = cos(x102);
	const GEN_FLT x104 = pow(x103, -1);
	const GEN_FLT x105 = x101 * x104;
	const GEN_FLT x106 = asin(x98 * x105);
	const GEN_FLT x107 = -8.0108022e-06 - 1.60216044e-05 * x106;
	const GEN_FLT x108 = 8.0108022e-06 * x106;
	const GEN_FLT x109 = -8.0108022e-06 - x108;
	const GEN_FLT x110 = 0.0028679863 + x109 * x106;
	const GEN_FLT x111 = x110 + x107 * x106;
	const GEN_FLT x112 = 5.3685255e-06 + x106 * x110;
	const GEN_FLT x113 = x112 + x106 * x111;
	const GEN_FLT x114 = pow(x103, -2);
	const GEN_FLT x115 = pow(1 - x99 * x114 / x100, -1.0 / 2.0);
	const GEN_FLT x116 = x24 * (x59 + x61 + x65) + x54 * x96 + x57 * (x4 + x3 * x95 + 2 * x64 * x46) +
						 x67 * (-x86 - x87 + x88) + x72 * x94 + x97 * x43;
	const GEN_FLT x117 = 2 * x83 * x89 + 2 * x84 * x76;
	const GEN_FLT x118 = (1.0 / 2.0) * x98;
	const GEN_FLT x119 = x105 * x116 - x104 * x118 * (x117 + 2 * x98 * x116) / pow(x100, 3.0 / 2.0);
	const GEN_FLT x120 = x119 * x115;
	const GEN_FLT x121 = x109 * x120;
	const GEN_FLT x122 = 2.40324066e-05 * x106;
	const GEN_FLT x123 = x106 * (x121 - x108 * x120) + x110 * x120;
	const GEN_FLT x124 = x106 * x123 + x112 * x120;
	const GEN_FLT x125 = 0.0076069798 + x106 * x112;
	const GEN_FLT x126 = x125 + x106 * x113;
	const GEN_FLT x127 = sin(x102);
	const GEN_FLT x128 = tan(x102);
	const GEN_FLT x129 = pow(x90, -1.0 / 2.0);
	const GEN_FLT x130 = x98 * x129;
	const GEN_FLT x131 = x128 * x130;
	const GEN_FLT x132 = atan2(-x83, x84);
	const GEN_FLT x133 = ogeeMag_0 + x132 - asin(x131);
	const GEN_FLT x134 = sin(x133);
	const GEN_FLT x135 = curve_0 + x134 * ogeePhase_0;
	const GEN_FLT x136 = x127 * x135;
	const GEN_FLT x137 =
		-x136 * (x106 * x124 +
				 x106 * (x124 + x106 * (x123 + x106 * (x121 + x107 * x120 - x120 * x122) + x111 * x120) + x113 * x120) +
				 x120 * x125 + x120 * x126);
	const GEN_FLT x138 = x106 * x125;
	const GEN_FLT x139 = x138 + x106 * x126;
	const GEN_FLT x140 = x127 * x139;
	const GEN_FLT x141 = pow(x128, 2);
	const GEN_FLT x142 = pow(1 - x91 * x99 * x141, -1.0 / 2.0);
	const GEN_FLT x143 = x116 * x128 * x129 - x118 * x117 * x128 / pow(x90, 3.0 / 2.0);
	const GEN_FLT x144 = x92 - x142 * x143;
	const GEN_FLT x145 = cos(x133) * ogeePhase_0;
	const GEN_FLT x146 = x144 * x145;
	const GEN_FLT x147 = pow(x106, 2);
	const GEN_FLT x148 = x103 - x135 * x140;
	const GEN_FLT x149 = x125 * x135 * x147 / pow(x148, 2);
	const GEN_FLT x150 = pow(x148, -1);
	const GEN_FLT x151 = x147 * x150;
	const GEN_FLT x152 = x125 * x151;
	const GEN_FLT x153 = 2 * x138 * x135 * x150;
	const GEN_FLT x154 = x135 * x151;
	const GEN_FLT x155 = x143 + x120 * x153 + x124 * x154;
	const GEN_FLT x156 = x131 + x125 * x154;
	const GEN_FLT x157 = pow(1 - pow(x156, 2), -1.0 / 2.0);
	const GEN_FLT x158 = x157 * (x155 + x146 * x152 - x149 * (x137 - x140 * x146));
	const GEN_FLT x159 = x158 + x93;
	const GEN_FLT x160 = -gibPhase_0 - x132 + asin(x156);
	const GEN_FLT x161 = cos(x160) * gibMag_0;
	const GEN_FLT x162 = -x158 + x92;
	const GEN_FLT x163 = x162 - x161 * x159;
	const GEN_FLT x164 = x115 * (x119 + x98 * x101 * x114 * x127);
	const GEN_FLT x165 = x109 * x164;
	const GEN_FLT x166 = x106 * (x165 - x108 * x164) + x110 * x164;
	const GEN_FLT x167 = x106 * x166 + x112 * x164;
	const GEN_FLT x168 = x143 + x130 * (1 + x141);
	const GEN_FLT x169 = x92 - x168 * x142;
	const GEN_FLT x170 = x140 * x145;
	const GEN_FLT x171 = x145 * x152;
	const GEN_FLT x172 =
		x157 * (x168 -
				x149 * (-x127 -
						x136 * (x106 * x167 +
								x106 * (x167 + x106 * (x166 + x106 * (x165 + x107 * x164 - x122 * x164) + x111 * x164) +
										x113 * x164) +
								x125 * x164 + x126 * x164) -
						x169 * x170 - x103 * x135 * x139) +
				x164 * x153 + x167 * x154 + x169 * x171);
	const GEN_FLT x173 = 1 + x146;
	const GEN_FLT x174 = x157 * (x155 - x149 * (x137 - x173 * x140) + x173 * x152);
	const GEN_FLT x175 = 1 + x144;
	const GEN_FLT x176 = x157 * (x155 - x149 * (x137 - x170 * x175) + x171 * x175);
	const GEN_FLT x177 = x134 + x146;
	const GEN_FLT x178 = x157 * (x155 - x149 * (x137 - x177 * x140) + x177 * x152);
	out[0] = -1 + x163;
	out[1] = -x172 + x92 - x161 * (x172 + x93);
	out[2] = -x174 + x92 - x161 * (x174 + x93);
	out[3] = x162 - x161 * (-1 + x159);
	out[4] = x163 - sin(x160);
	out[5] = -x176 + x92 - x161 * (x176 + x93);
	out[6] = -x178 + x92 - x161 * (x178 + x93);
}

/** Applying function <function reproject_axis_y_gen2 at 0x7effc26e3b00> */
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x5 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 - x6;
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1e-10));
	const GEN_FLT x11 = cos(x10);
	const GEN_FLT x12 = 1 - x11;
	const GEN_FLT x13 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x14 = sin(x10);
	const GEN_FLT x15 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x16 = x15 * x14;
	const GEN_FLT x17 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x18 = x13 * x12 * x17;
	const GEN_FLT x19 = x14 * x17;
	const GEN_FLT x20 = x15 * x12;
	const GEN_FLT x21 = x20 * x13;
	const GEN_FLT x22 =
		obj_pz + (x11 + pow(x13, 2) * x12) * sensor_z + (x16 + x18) * sensor_y + (-x19 + x21) * sensor_x;
	const GEN_FLT x23 = x14 * x13;
	const GEN_FLT x24 = x20 * x17;
	const GEN_FLT x25 =
		obj_py + (x11 + x12 * pow(x17, 2)) * sensor_y + (-x16 + x18) * sensor_z + (x23 + x24) * sensor_x;
	const GEN_FLT x26 =
		obj_px + (x11 + pow(x15, 2) * x12) * sensor_x + (x19 + x21) * sensor_z + (-x23 + x24) * sensor_y;
	const GEN_FLT x27 = x1 * x4;
	const GEN_FLT x28 = x2 * x8;
	const GEN_FLT x29 = lh_py + x25 * (x6 + pow(x5, 2) * x7) + (x27 + x28) * x26 + (-x3 + x9) * x22;
	const GEN_FLT x30 = x1 * x5;
	const GEN_FLT x31 = x2 * x4 * x7;
	const GEN_FLT x32 = lh_pz + x22 * (x6 + pow(x4, 2) * x7) + (x3 + x9) * x25 + (-x30 + x31) * x26;
	const GEN_FLT x33 = lh_px + x26 * (x6 + pow(x2, 2) * x7) + (-x27 + x28) * x25 + (x30 + x31) * x22;
	const GEN_FLT x34 = pow(x32, 2) + pow(x33, 2);
	const GEN_FLT x35 = 0.523598775598299 - tilt_1;
	const GEN_FLT x36 = cos(x35);
	const GEN_FLT x37 = asin(x29 / (x36 * sqrt(x34 + pow(x29, 2))));
	const GEN_FLT x38 = 0.0028679863 + x37 * (-8.0108022e-06 - 8.0108022e-06 * x37);
	const GEN_FLT x39 = 5.3685255e-06 + x38 * x37;
	const GEN_FLT x40 = 0.0076069798 + x37 * x39;
	const GEN_FLT x41 = -x29 * tan(x35) / sqrt(x34);
	const GEN_FLT x42 = atan2(-x32, x33);
	const GEN_FLT x43 = curve_1 + sin(ogeeMag_1 + x42 - asin(x41)) * ogeePhase_1;
	const GEN_FLT x44 = asin(
		x41 + x40 * x43 * pow(x37, 2) /
				  (x36 + x43 * sin(x35) *
							 (x37 * (x40 + x37 * (x39 + x37 * (x38 + x37 * (-8.0108022e-06 - 1.60216044e-05 * x37)))) +
							  x40 * x37)));
	out[0] = -1.5707963267949 - phase_1 + x42 - x44 + sin(gibPhase_1 + x42 - x44) * gibMag_1;
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 - x1;
	const GEN_FLT x3 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x4 = pow(x3, 2);
	const GEN_FLT x5 = x1 + x2 * x4;
	const GEN_FLT x6 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x7 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1e-10));
	const GEN_FLT x8 = sin(x7);
	const GEN_FLT x9 = x6 * x8;
	const GEN_FLT x10 = -x9;
	const GEN_FLT x11 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x12 = pow(x11, 2);
	const GEN_FLT x13 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x14 = cos(x7);
	const GEN_FLT x15 = 1 - x14;
	const GEN_FLT x16 = x15 * x11;
	const GEN_FLT x17 = 2 * x16;
	const GEN_FLT x18 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x19 = x6 * x14;
	const GEN_FLT x20 = x19 * x18;
	const GEN_FLT x21 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x22 = x8 * x21;
	const GEN_FLT x23 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x24 = x8 * x23;
	const GEN_FLT x25 = x24 * x11;
	const GEN_FLT x26 = x23 * x15;
	const GEN_FLT x27 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x28 = x26 * x13 + x27 * x16 + x6 * x25;
	const GEN_FLT x29 = x23 * x14;
	const GEN_FLT x30 = x6 * x29;
	const GEN_FLT x31 = x8 * x27;
	const GEN_FLT x32 = x8 * x18;
	const GEN_FLT x33 = x32 * x11;
	const GEN_FLT x34 = x15 * x18;
	const GEN_FLT x35 = x21 * x16 + x34 * x13 + x6 * x33;
	const GEN_FLT x36 =
		(x10 + x13 * x17 + x9 * x12) * sensor_x + (-x20 - x22 + x28) * sensor_y + (x30 + x31 + x35) * sensor_z;
	const GEN_FLT x37 = 1 + x36;
	const GEN_FLT x38 = x11 * x19;
	const GEN_FLT x39 = x8 * x13;
	const GEN_FLT x40 = x24 * x18;
	const GEN_FLT x41 = x21 * x26 + x34 * x27 + x6 * x40;
	const GEN_FLT x42 = pow(x18, 2);
	const GEN_FLT x43 = 2 * x34;
	const GEN_FLT x44 =
		(x10 + x43 * x21 + x9 * x42) * sensor_z + (-x30 - x31 + x35) * sensor_x + (x38 + x39 + x41) * sensor_y;
	const GEN_FLT x45 = sin(x0);
	const GEN_FLT x46 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x47 = x45 * x46;
	const GEN_FLT x48 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x49 = x2 * x48;
	const GEN_FLT x50 = x3 * x49;
	const GEN_FLT x51 = x47 + x50;
	const GEN_FLT x52 = x51 * x44;
	const GEN_FLT x53 = x45 * x48;
	const GEN_FLT x54 = x2 * x46;
	const GEN_FLT x55 = x3 * x54;
	const GEN_FLT x56 = -x53 + x55;
	const GEN_FLT x57 = pow(x23, 2);
	const GEN_FLT x58 = 2 * x26;
	const GEN_FLT x59 =
		(x10 + x58 * x27 + x9 * x57) * sensor_y + (x20 + x22 + x28) * sensor_x + (-x38 - x39 + x41) * sensor_z;
	const GEN_FLT x60 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x61 = x60 * x45;
	const GEN_FLT x62 = -x61;
	const GEN_FLT x63 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x64 = x18 * x16;
	const GEN_FLT x65 = x23 * x16;
	const GEN_FLT x66 = obj_px + (x14 + x15 * x12) * sensor_x + (x24 + x64) * sensor_z + (-x32 + x65) * sensor_y;
	const GEN_FLT x67 = x8 * x11;
	const GEN_FLT x68 = x26 * x18;
	const GEN_FLT x69 = obj_py + (x14 + x57 * x15) * sensor_y + (x32 + x65) * sensor_x + (-x67 + x68) * sensor_z;
	const GEN_FLT x70 = x1 * x60;
	const GEN_FLT x71 = x70 * x48;
	const GEN_FLT x72 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x73 = x72 * x45;
	const GEN_FLT x74 = x3 * x61;
	const GEN_FLT x75 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x76 = x2 * x75;
	const GEN_FLT x77 = x3 * x76 + x63 * x54 + x74 * x46;
	const GEN_FLT x78 = obj_pz + (x14 + x42 * x15) * sensor_z + (-x24 + x64) * sensor_x + (x67 + x68) * sensor_y;
	const GEN_FLT x79 = x70 * x46;
	const GEN_FLT x80 = x75 * x45;
	const GEN_FLT x81 = x2 * x72;
	const GEN_FLT x82 = x3 * x81 + x63 * x49 + x74 * x48;
	const GEN_FLT x83 = x66 * (x62 + x4 * x61 + 2 * x2 * x3 * x63) + x69 * (-x71 - x73 + x77) + x78 * (x79 + x80 + x82);
	const GEN_FLT x84 = x83 + x56 * x59;
	const GEN_FLT x85 = x52 + x84 + x5 * x37;
	const GEN_FLT x86 = pow(x48, 2);
	const GEN_FLT x87 = x1 + x2 * x86;
	const GEN_FLT x88 = x3 * x45;
	const GEN_FLT x89 = x46 * x49;
	const GEN_FLT x90 = x88 + x89;
	const GEN_FLT x91 = -x47 + x50;
	const GEN_FLT x92 = lh_pz + x66 * x91 + x69 * x90 + x87 * x78;
	const GEN_FLT x93 = lh_px + x5 * x66 + x69 * x56 + x78 * x51;
	const GEN_FLT x94 = pow(x93, 2);
	const GEN_FLT x95 = x92 / x94;
	const GEN_FLT x96 = x59 * x90;
	const GEN_FLT x97 = x3 * x70;
	const GEN_FLT x98 = x63 * x45;
	const GEN_FLT x99 = x76 * x48 + x81 * x46 + x61 * x46 * x48;
	const GEN_FLT x100 = x66 * (-x79 - x80 + x82) + x69 * (x97 + x98 + x99) + x78 * (x62 + 2 * x81 * x48 + x86 * x61);
	const GEN_FLT x101 = x100 + x87 * x44;
	const GEN_FLT x102 = x101 + x96 + x91 * x37;
	const GEN_FLT x103 = pow(x93, -1);
	const GEN_FLT x104 = x94 + pow(x92, 2);
	const GEN_FLT x105 = pow(x104, -1);
	const GEN_FLT x106 = x94 * x105;
	const GEN_FLT x107 = x106 * (-x103 * x102 + x85 * x95);
	const GEN_FLT x108 = -x88 + x89;
	const GEN_FLT x109 = pow(x46, 2);
	const GEN_FLT x110 = x1 + x2 * x109;
	const GEN_FLT x111 = x53 + x55;
	const GEN_FLT x112 = lh_py + x66 * x111 + x69 * x110 + x78 * x108;
	const GEN_FLT x113 = pow(x112, 2);
	const GEN_FLT x114 = x104 + x113;
	const GEN_FLT x115 = 0.523598775598299 - tilt_1;
	const GEN_FLT x116 = cos(x115);
	const GEN_FLT x117 = pow(x116, -1);
	const GEN_FLT x118 = x117 / sqrt(x114);
	const GEN_FLT x119 = asin(x112 * x118);
	const GEN_FLT x120 = 8.0108022e-06 * x119;
	const GEN_FLT x121 = -8.0108022e-06 - x120;
	const GEN_FLT x122 = 0.0028679863 + x119 * x121;
	const GEN_FLT x123 = 5.3685255e-06 + x119 * x122;
	const GEN_FLT x124 = 0.0076069798 + x119 * x123;
	const GEN_FLT x125 = x119 * x124;
	const GEN_FLT x126 = -8.0108022e-06 - 1.60216044e-05 * x119;
	const GEN_FLT x127 = x122 + x119 * x126;
	const GEN_FLT x128 = x123 + x119 * x127;
	const GEN_FLT x129 = x124 + x119 * x128;
	const GEN_FLT x130 = x125 + x119 * x129;
	const GEN_FLT x131 = sin(x115);
	const GEN_FLT x132 = tan(x115);
	const GEN_FLT x133 = x132 / sqrt(x104);
	const GEN_FLT x134 = -x112 * x133;
	const GEN_FLT x135 = atan2(-x92, x93);
	const GEN_FLT x136 = ogeeMag_1 + x135 - asin(x134);
	const GEN_FLT x137 = curve_1 + sin(x136) * ogeePhase_1;
	const GEN_FLT x138 = x131 * x137;
	const GEN_FLT x139 = x116 + x130 * x138;
	const GEN_FLT x140 = pow(x139, -1);
	const GEN_FLT x141 = pow(x119, 2);
	const GEN_FLT x142 = x137 * x141;
	const GEN_FLT x143 = x140 * x142;
	const GEN_FLT x144 = x134 + x124 * x143;
	const GEN_FLT x145 = pow(1 - pow(x144, 2), -1.0 / 2.0);
	const GEN_FLT x146 = pow(1 - x113 / (x114 * pow(x116, 2)), -1.0 / 2.0);
	const GEN_FLT x147 = x59 * x110;
	const GEN_FLT x148 = x66 * (x71 + x73 + x77) + x69 * (x62 + x61 * x109 + 2 * x76 * x46) + x78 * (-x97 - x98 + x99);
	const GEN_FLT x149 = x148 + x44 * x108;
	const GEN_FLT x150 = x147 + x149 + x37 * x111;
	const GEN_FLT x151 = 2 * x112;
	const GEN_FLT x152 = 2 * x93;
	const GEN_FLT x153 = 2 * x92;
	const GEN_FLT x154 = x102 * x153 + x85 * x152;
	const GEN_FLT x155 = (1.0 / 2.0) * x112;
	const GEN_FLT x156 = x117 * x155 / pow(x114, 3.0 / 2.0);
	const GEN_FLT x157 = x118 * x150 - x156 * (x154 + x150 * x151);
	const GEN_FLT x158 = x146 * x157;
	const GEN_FLT x159 = x121 * x158;
	const GEN_FLT x160 = x119 * (x159 - x120 * x158) + x122 * x158;
	const GEN_FLT x161 = x123 * x146;
	const GEN_FLT x162 = x119 * x160 + x161 * x157;
	const GEN_FLT x163 = 2 * x125 * x137 * x140;
	const GEN_FLT x164 = x128 * x146;
	const GEN_FLT x165 = x127 * x146;
	const GEN_FLT x166 = 2.40324066e-05 * x119;
	const GEN_FLT x167 = x126 * x146;
	const GEN_FLT x168 = x124 * x146;
	const GEN_FLT x169 = pow(1 - x105 * x113 * pow(x132, 2), -1.0 / 2.0);
	const GEN_FLT x170 = x132 * x155 / pow(x104, 3.0 / 2.0);
	const GEN_FLT x171 = -x133 * x150 + x170 * x154;
	const GEN_FLT x172 = x107 - x169 * x171;
	const GEN_FLT x173 = cos(x136) * ogeePhase_1;
	const GEN_FLT x174 = x130 * x131;
	const GEN_FLT x175 = x174 * x173;
	const GEN_FLT x176 = x124 * x142 / pow(x139, 2);
	const GEN_FLT x177 = x124 * x140 * x141;
	const GEN_FLT x178 = x177 * x173;
	const GEN_FLT x179 =
		x145 * (x171 + x162 * x143 + x163 * x158 -
				x176 * (x138 * (x119 * x162 +
								x119 * (x162 + x119 * (x160 + x119 * (x159 - x166 * x158 + x167 * x157) + x165 * x157) +
										x164 * x157) +
								x129 * x158 + x168 * x157) +
						x175 * x172) +
				x178 * x172);
	const GEN_FLT x180 = cos(gibPhase_1 + x135 - asin(x144)) * gibMag_1;
	const GEN_FLT x181 = x5 * x36;
	const GEN_FLT x182 = 1 + x59;
	const GEN_FLT x183 = x181 + x52 + x83 + x56 * x182;
	const GEN_FLT x184 = x91 * x36;
	const GEN_FLT x185 = x101 + x184 + x90 * x182;
	const GEN_FLT x186 = x106 * (-x103 * x185 + x95 * x183);
	const GEN_FLT x187 = x36 * x111;
	const GEN_FLT x188 = x149 + x187 + x110 * x182;
	const GEN_FLT x189 = x183 * x152 + x185 * x153;
	const GEN_FLT x190 = x118 * x188 - x156 * (x189 + x188 * x151);
	const GEN_FLT x191 = x190 * x146;
	const GEN_FLT x192 = x121 * x191;
	const GEN_FLT x193 = x119 * (x192 - x120 * x191) + x122 * x191;
	const GEN_FLT x194 = x119 * x193 + x161 * x190;
	const GEN_FLT x195 = x170 * x189 - x188 * x133;
	const GEN_FLT x196 = x186 - x169 * x195;
	const GEN_FLT x197 =
		x145 * (x195 + x163 * x191 -
				x176 * (x138 * (x119 * x194 +
								x119 * (x194 + x119 * (x193 + x119 * (x192 - x166 * x191 + x167 * x190) + x165 * x190) +
										x164 * x190) +
								x129 * x191 + x168 * x190) +
						x175 * x196) +
				x178 * x196 + x194 * x143);
	const GEN_FLT x198 = 1 + x44;
	const GEN_FLT x199 = x181 + x84 + x51 * x198;
	const GEN_FLT x200 = x100 + x184 + x96 + x87 * x198;
	const GEN_FLT x201 = x106 * (-x200 * x103 + x95 * x199);
	const GEN_FLT x202 = x147 + x148 + x187 + x108 * x198;
	const GEN_FLT x203 = x199 * x152 + x200 * x153;
	const GEN_FLT x204 = -x156 * (x203 + x202 * x151) + x202 * x118;
	const GEN_FLT x205 = x204 * x146;
	const GEN_FLT x206 = x205 * x121;
	const GEN_FLT x207 = x119 * (x206 - x205 * x120) + x205 * x122;
	const GEN_FLT x208 = x204 * x161 + x207 * x119;
	const GEN_FLT x209 = -x202 * x133 + x203 * x170;
	const GEN_FLT x210 = x201 - x209 * x169;
	const GEN_FLT x211 =
		x145 * (x209 -
				x176 * (x138 * (x119 * (x208 + x119 * (x207 + x119 * (x206 + x204 * x167 - x205 * x166) + x204 * x165) +
										x204 * x164) +
								x204 * x168 + x205 * x129 + x208 * x119) +
						x210 * x175) +
				x205 * x163 + x208 * x143 + x210 * x178);
	const GEN_FLT x212 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x213 = x8 * x212;
	const GEN_FLT x214 = -x213;
	const GEN_FLT x215 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x216 = x14 * x212;
	const GEN_FLT x217 = x18 * x216;
	const GEN_FLT x218 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x219 = x8 * x218;
	const GEN_FLT x220 = x23 * x213;
	const GEN_FLT x221 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x222 = x11 * x220 + x16 * x221 + x26 * x215;
	const GEN_FLT x223 = x29 * x212;
	const GEN_FLT x224 = x8 * x221;
	const GEN_FLT x225 = x16 * x218 + x34 * x215 + x11 * x18 * x213;
	const GEN_FLT x226 = (x214 + x12 * x213 + x17 * x215) * sensor_x + (-x217 - x219 + x222) * sensor_y +
						 (x223 + x224 + x225) * sensor_z;
	const GEN_FLT x227 = x11 * x216;
	const GEN_FLT x228 = x8 * x215;
	const GEN_FLT x229 = x18 * x220 + x26 * x218 + x34 * x221;
	const GEN_FLT x230 = (x214 + x57 * x213 + x58 * x221) * sensor_y + (x217 + x219 + x222) * sensor_x +
						 (-x227 - x228 + x229) * sensor_z;
	const GEN_FLT x231 = (x214 + x42 * x213 + x43 * x218) * sensor_z + (-x223 - x224 + x225) * sensor_x +
						 (x227 + x228 + x229) * sensor_y;
	const GEN_FLT x232 = x83 + x5 * x226 + x51 * x231 + x56 * x230;
	const GEN_FLT x233 = x100 + x87 * x231 + x90 * x230 + x91 * x226;
	const GEN_FLT x234 = x106 * (-x233 * x103 + x95 * x232);
	const GEN_FLT x235 = x148 + x226 * x111 + x230 * x110 + x231 * x108;
	const GEN_FLT x236 = x232 * x152 + x233 * x153;
	const GEN_FLT x237 = x146 * (-x156 * (x236 + x235 * x151) + x235 * x118);
	const GEN_FLT x238 = x237 * x121;
	const GEN_FLT x239 = x119 * (x238 - x237 * x120) + x237 * x122;
	const GEN_FLT x240 = x237 * x123 + x239 * x119;
	const GEN_FLT x241 = -x235 * x133 + x236 * x170;
	const GEN_FLT x242 = x234 - x241 * x169;
	const GEN_FLT x243 =
		x145 * (x241 -
				x176 * (x138 * (x119 * (x240 + x119 * (x239 + x119 * (x238 + x237 * x126 - x237 * x166) + x237 * x127) +
										x237 * x128) +
								x237 * x124 + x237 * x129 + x240 * x119) +
						x242 * x175) +
				x237 * x163 + x240 * x143 + x242 * x178);
	const GEN_FLT x244 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x245 = x8 * x244;
	const GEN_FLT x246 = -x245;
	const GEN_FLT x247 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x248 = x14 * x244;
	const GEN_FLT x249 = x18 * x248;
	const GEN_FLT x250 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x251 = x8 * x250;
	const GEN_FLT x252 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x253 = x16 * x252 + x25 * x244 + x26 * x247;
	const GEN_FLT x254 = x23 * x248;
	const GEN_FLT x255 = x8 * x252;
	const GEN_FLT x256 = x16 * x250 + x33 * x244 + x34 * x247;
	const GEN_FLT x257 = (x246 + x12 * x245 + x17 * x247) * sensor_x + (-x249 - x251 + x253) * sensor_y +
						 (x254 + x255 + x256) * sensor_z;
	const GEN_FLT x258 = x11 * x248;
	const GEN_FLT x259 = x8 * x247;
	const GEN_FLT x260 = x26 * x250 + x34 * x252 + x40 * x244;
	const GEN_FLT x261 = (x246 + x57 * x245 + x58 * x252) * sensor_y + (x249 + x251 + x253) * sensor_x +
						 (-x258 - x259 + x260) * sensor_z;
	const GEN_FLT x262 = (x246 + x42 * x245 + x43 * x250) * sensor_z + (-x254 - x255 + x256) * sensor_x +
						 (x258 + x259 + x260) * sensor_y;
	const GEN_FLT x263 = x83 + x5 * x257 + x51 * x262 + x56 * x261;
	const GEN_FLT x264 = x100 + x87 * x262 + x90 * x261 + x91 * x257;
	const GEN_FLT x265 = x106 * (-x264 * x103 + x95 * x263);
	const GEN_FLT x266 = x148 + x257 * x111 + x261 * x110 + x262 * x108;
	const GEN_FLT x267 = x263 * x152 + x264 * x153;
	const GEN_FLT x268 = x146 * (-x156 * (x267 + x266 * x151) + x266 * x118);
	const GEN_FLT x269 = x268 * x121;
	const GEN_FLT x270 = x119 * (x269 - x268 * x120) + x268 * x122;
	const GEN_FLT x271 = x268 * x123 + x270 * x119;
	const GEN_FLT x272 = -x266 * x133 + x267 * x170;
	const GEN_FLT x273 = x173 * (x265 - x272 * x169);
	const GEN_FLT x274 =
		x145 * (x272 -
				x176 * (x138 * (x119 * (x271 + x119 * (x270 + x119 * (x269 + x268 * x126 - x268 * x166) + x268 * x127) +
										x268 * x128) +
								x268 * x124 + x268 * x129 + x271 * x119) +
						x273 * x174) +
				x268 * x163 + x271 * x143 + x273 * x177);
	const GEN_FLT x275 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x276 = x14 * x275;
	const GEN_FLT x277 = x18 * x276;
	const GEN_FLT x278 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x279 = x8 * x278;
	const GEN_FLT x280 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x281 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x282 = x16 * x281 + x25 * x275 + x26 * x280;
	const GEN_FLT x283 = x11 * x276;
	const GEN_FLT x284 = x8 * x280;
	const GEN_FLT x285 = x26 * x278 + x34 * x281 + x40 * x275;
	const GEN_FLT x286 = x8 * x275;
	const GEN_FLT x287 = -x286;
	const GEN_FLT x288 = (x277 + x279 + x282) * sensor_x + (-x283 - x284 + x285) * sensor_z +
						 (x287 + x57 * x286 + x58 * x281) * sensor_y;
	const GEN_FLT x289 = x23 * x276;
	const GEN_FLT x290 = x8 * x281;
	const GEN_FLT x291 = x16 * x278 + x33 * x275 + x34 * x280;
	const GEN_FLT x292 = (-x277 - x279 + x282) * sensor_y + (x287 + x12 * x286 + x17 * x280) * sensor_x +
						 (x289 + x290 + x291) * sensor_z;
	const GEN_FLT x293 = (x283 + x284 + x285) * sensor_y + (x287 + x42 * x286 + x43 * x278) * sensor_z +
						 (-x289 - x290 + x291) * sensor_x;
	const GEN_FLT x294 = x83 + x5 * x292 + x51 * x293 + x56 * x288;
	const GEN_FLT x295 = x100 + x87 * x293 + x90 * x288 + x91 * x292;
	const GEN_FLT x296 = x106 * (-x295 * x103 + x95 * x294);
	const GEN_FLT x297 = x148 + x288 * x110 + x292 * x111 + x293 * x108;
	const GEN_FLT x298 = x294 * x152 + x295 * x153;
	const GEN_FLT x299 = -x156 * (x298 + x297 * x151) + x297 * x118;
	const GEN_FLT x300 = x299 * x146;
	const GEN_FLT x301 = x300 * x121;
	const GEN_FLT x302 = x119 * (x301 - x300 * x120) + x300 * x122;
	const GEN_FLT x303 = x299 * x161 + x302 * x119;
	const GEN_FLT x304 = -x297 * x133 + x298 * x170;
	const GEN_FLT x305 = x296 - x304 * x169;
	const GEN_FLT x306 =
		x145 * (x304 -
				x176 * (x138 * (x119 * (x303 + x119 * (x302 + x119 * (x301 + x299 * x167 - x300 * x166) + x299 * x165) +
										x300 * x128) +
								x300 * x124 + x300 * x129 + x303 * x119) +
						x305 * x175) +
				x300 * x163 + x303 * x143 + x305 * x178);
	out[0] = x107 - x179 - (-x107 + x179) * x180;
	out[1] = x186 - x197 - (-x186 + x197) * x180;
	out[2] = x201 - x211 - (-x201 + x211) * x180;
	out[3] = x234 - x243 - (-x234 + x243) * x180;
	out[4] = x265 - x274 - (-x265 + x274) * x180;
	out[5] = x296 - x306 - (-x296 + x306) * x180;
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 - x1;
	const GEN_FLT x3 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x4 = pow(x3, 2);
	const GEN_FLT x5 = x1 + x2 * x4;
	const GEN_FLT x6 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1e-10));
	const GEN_FLT x7 = cos(x6);
	const GEN_FLT x8 = 1 - x7;
	const GEN_FLT x9 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								  : (1e-10)))
							? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											 : (1e-10)))
							: (1));
	const GEN_FLT x10 = pow(x9, 2);
	const GEN_FLT x11 = x7 + x8 * x10;
	const GEN_FLT x12 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x13 = sin(x6);
	const GEN_FLT x14 = x13 * x12;
	const GEN_FLT x15 = -x14;
	const GEN_FLT x16 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x17 = x8 * x9;
	const GEN_FLT x18 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x19 = x7 * x12;
	const GEN_FLT x20 = x19 * x18;
	const GEN_FLT x21 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x22 = x21 * x13;
	const GEN_FLT x23 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x24 = x23 * x13;
	const GEN_FLT x25 = x9 * x12;
	const GEN_FLT x26 = x8 * x23;
	const GEN_FLT x27 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x28 = x24 * x25 + x26 * x16 + x27 * x17;
	const GEN_FLT x29 = x23 * x19;
	const GEN_FLT x30 = x27 * x13;
	const GEN_FLT x31 = x13 * x18;
	const GEN_FLT x32 = x8 * x18;
	const GEN_FLT x33 = x21 * x17 + x31 * x25 + x32 * x16;
	const GEN_FLT x34 =
		(x15 + x14 * x10 + 2 * x17 * x16) * sensor_x + (-x20 - x22 + x28) * sensor_y + (x29 + x30 + x33) * sensor_z;
	const GEN_FLT x35 = x11 + x34;
	const GEN_FLT x36 = sin(x0);
	const GEN_FLT x37 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x38 = x36 * x37;
	const GEN_FLT x39 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x40 = x2 * x3;
	const GEN_FLT x41 = x40 * x39;
	const GEN_FLT x42 = -x38 + x41;
	const GEN_FLT x43 = x23 * x17;
	const GEN_FLT x44 = x31 + x43;
	const GEN_FLT x45 = x9 * x19;
	const GEN_FLT x46 = x13 * x16;
	const GEN_FLT x47 = x21 * x26 + x32 * x27 + x24 * x12 * x18;
	const GEN_FLT x48 = pow(x23, 2);
	const GEN_FLT x49 =
		(x15 + 2 * x26 * x27 + x48 * x14) * sensor_y + (x20 + x22 + x28) * sensor_x + (-x45 - x46 + x47) * sensor_z;
	const GEN_FLT x50 = x44 + x49;
	const GEN_FLT x51 = x18 * x17;
	const GEN_FLT x52 = -x24 + x51;
	const GEN_FLT x53 = pow(x18, 2);
	const GEN_FLT x54 =
		(x15 + 2 * x32 * x21 + x53 * x14) * sensor_z + (-x29 - x30 + x33) * sensor_x + (x45 + x46 + x47) * sensor_y;
	const GEN_FLT x55 = x52 + x54;
	const GEN_FLT x56 = x36 * x39;
	const GEN_FLT x57 = x40 * x37;
	const GEN_FLT x58 = x56 + x57;
	const GEN_FLT x59 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x60 = x59 * x36;
	const GEN_FLT x61 = -x60;
	const GEN_FLT x62 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x63 = x24 + x51;
	const GEN_FLT x64 = -x31 + x43;
	const GEN_FLT x65 = obj_px + x11 * sensor_x + x63 * sensor_z + x64 * sensor_y;
	const GEN_FLT x66 = x7 + x8 * x48;
	const GEN_FLT x67 = x9 * x13;
	const GEN_FLT x68 = x32 * x23;
	const GEN_FLT x69 = -x67 + x68;
	const GEN_FLT x70 = obj_py + x44 * sensor_x + x66 * sensor_y + x69 * sensor_z;
	const GEN_FLT x71 = x1 * x59;
	const GEN_FLT x72 = x71 * x37;
	const GEN_FLT x73 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x74 = x73 * x36;
	const GEN_FLT x75 = x3 * x60;
	const GEN_FLT x76 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x77 = x2 * x76;
	const GEN_FLT x78 = x2 * x39;
	const GEN_FLT x79 = x3 * x77 + x75 * x39 + x78 * x62;
	const GEN_FLT x80 = x7 + x8 * x53;
	const GEN_FLT x81 = x67 + x68;
	const GEN_FLT x82 = obj_pz + x52 * sensor_x + x80 * sensor_z + x81 * sensor_y;
	const GEN_FLT x83 = x71 * x39;
	const GEN_FLT x84 = x76 * x36;
	const GEN_FLT x85 = x2 * x37;
	const GEN_FLT x86 = x73 * x40 + x75 * x37 + x85 * x62;
	const GEN_FLT x87 = x65 * (x61 + x4 * x60 + 2 * x62 * x40) + x70 * (-x72 - x74 + x79) + x82 * (x83 + x84 + x86);
	const GEN_FLT x88 = x87 + x5 * x35 + x50 * x42 + x58 * x55;
	const GEN_FLT x89 = pow(x37, 2);
	const GEN_FLT x90 = x1 + x2 * x89;
	const GEN_FLT x91 = x3 * x36;
	const GEN_FLT x92 = x78 * x37;
	const GEN_FLT x93 = x91 + x92;
	const GEN_FLT x94 = -x56 + x57;
	const GEN_FLT x95 = lh_pz + x65 * x94 + x70 * x93 + x82 * x90;
	const GEN_FLT x96 = lh_px + x5 * x65 + x70 * x42 + x82 * x58;
	const GEN_FLT x97 = pow(x96, 2);
	const GEN_FLT x98 = x95 / x97;
	const GEN_FLT x99 = pow(x96, -1);
	const GEN_FLT x100 = x3 * x71;
	const GEN_FLT x101 = x62 * x36;
	const GEN_FLT x102 = x73 * x78 + x77 * x37 + x60 * x37 * x39;
	const GEN_FLT x103 =
		x65 * (-x83 - x84 + x86) + x70 * (x100 + x101 + x102) + x82 * (x61 + 2 * x85 * x73 + x89 * x60);
	const GEN_FLT x104 = x103 + x50 * x93 + x55 * x90 + x94 * x35;
	const GEN_FLT x105 = x97 + pow(x95, 2);
	const GEN_FLT x106 = pow(x105, -1);
	const GEN_FLT x107 = x97 * x106;
	const GEN_FLT x108 = x107 * (x88 * x98 - x99 * x104);
	const GEN_FLT x109 = -x91 + x92;
	const GEN_FLT x110 = pow(x39, 2);
	const GEN_FLT x111 = x1 + x2 * x110;
	const GEN_FLT x112 = x38 + x41;
	const GEN_FLT x113 = lh_py + x65 * x112 + x70 * x111 + x82 * x109;
	const GEN_FLT x114 = pow(x113, 2);
	const GEN_FLT x115 = x105 + x114;
	const GEN_FLT x116 = 0.523598775598299 - tilt_1;
	const GEN_FLT x117 = cos(x116);
	const GEN_FLT x118 = pow(x117, -1);
	const GEN_FLT x119 = x118 / sqrt(x115);
	const GEN_FLT x120 = asin(x113 * x119);
	const GEN_FLT x121 = 8.0108022e-06 * x120;
	const GEN_FLT x122 = -8.0108022e-06 - x121;
	const GEN_FLT x123 = 0.0028679863 + x120 * x122;
	const GEN_FLT x124 = 5.3685255e-06 + x120 * x123;
	const GEN_FLT x125 = 0.0076069798 + x120 * x124;
	const GEN_FLT x126 = x120 * x125;
	const GEN_FLT x127 = -8.0108022e-06 - 1.60216044e-05 * x120;
	const GEN_FLT x128 = x123 + x120 * x127;
	const GEN_FLT x129 = x124 + x120 * x128;
	const GEN_FLT x130 = x125 + x120 * x129;
	const GEN_FLT x131 = x126 + x120 * x130;
	const GEN_FLT x132 = sin(x116);
	const GEN_FLT x133 = tan(x116);
	const GEN_FLT x134 = x133 / sqrt(x105);
	const GEN_FLT x135 = -x113 * x134;
	const GEN_FLT x136 = atan2(-x95, x96);
	const GEN_FLT x137 = ogeeMag_1 + x136 - asin(x135);
	const GEN_FLT x138 = curve_1 + sin(x137) * ogeePhase_1;
	const GEN_FLT x139 = x132 * x138;
	const GEN_FLT x140 = x117 + x131 * x139;
	const GEN_FLT x141 = pow(x140, -1);
	const GEN_FLT x142 = pow(x120, 2);
	const GEN_FLT x143 = x138 * x142;
	const GEN_FLT x144 = x141 * x143;
	const GEN_FLT x145 = x135 + x125 * x144;
	const GEN_FLT x146 = pow(1 - pow(x145, 2), -1.0 / 2.0);
	const GEN_FLT x147 = pow(1 - x114 / (x115 * pow(x117, 2)), -1.0 / 2.0);
	const GEN_FLT x148 =
		x65 * (x72 + x74 + x79) + x70 * (x61 + x60 * x110 + 2 * x77 * x39) + x82 * (-x100 - x101 + x102);
	const GEN_FLT x149 = x148 + x35 * x112 + x50 * x111 + x55 * x109;
	const GEN_FLT x150 = 2 * x113;
	const GEN_FLT x151 = 2 * x96;
	const GEN_FLT x152 = 2 * x95;
	const GEN_FLT x153 = x104 * x152 + x88 * x151;
	const GEN_FLT x154 = (1.0 / 2.0) * x113;
	const GEN_FLT x155 = x118 * x154 / pow(x115, 3.0 / 2.0);
	const GEN_FLT x156 = x147 * (x119 * x149 - x155 * (x153 + x149 * x150));
	const GEN_FLT x157 = x122 * x156;
	const GEN_FLT x158 = x120 * (x157 - x121 * x156) + x123 * x156;
	const GEN_FLT x159 = x120 * x158 + x124 * x156;
	const GEN_FLT x160 = 2 * x126 * x138 * x141;
	const GEN_FLT x161 = x133 * x154 / pow(x105, 3.0 / 2.0);
	const GEN_FLT x162 = -x134 * x149 + x161 * x153;
	const GEN_FLT x163 = pow(1 - x106 * x114 * pow(x133, 2), -1.0 / 2.0);
	const GEN_FLT x164 = cos(x137) * ogeePhase_1;
	const GEN_FLT x165 = x164 * (x108 - x163 * x162);
	const GEN_FLT x166 = x125 * x142 * x141;
	const GEN_FLT x167 = 2.40324066e-05 * x120;
	const GEN_FLT x168 = x131 * x132;
	const GEN_FLT x169 = x125 * x143 / pow(x140, 2);
	const GEN_FLT x170 =
		x146 * (x162 + x144 * x159 + x160 * x156 + x166 * x165 -
				x169 * (x139 * (x120 * x159 +
								x120 * (x159 + x120 * (x158 + x120 * (x157 + x127 * x156 - x167 * x156) + x128 * x156) +
										x129 * x156) +
								x125 * x156 + x130 * x156) +
						x168 * x165));
	const GEN_FLT x171 = cos(gibPhase_1 + x136 - asin(x145)) * gibMag_1;
	const GEN_FLT x172 = x49 + x66;
	const GEN_FLT x173 = x34 + x64;
	const GEN_FLT x174 = x54 + x81;
	const GEN_FLT x175 = x87 + x42 * x172 + x5 * x173 + x58 * x174;
	const GEN_FLT x176 = x103 + x90 * x174 + x93 * x172 + x94 * x173;
	const GEN_FLT x177 = (x98 * x175 - x99 * x176) * x107;
	const GEN_FLT x178 = x148 + x109 * x174 + x111 * x172 + x112 * x173;
	const GEN_FLT x179 = x175 * x151 + x176 * x152;
	const GEN_FLT x180 = x147 * (x119 * x178 - x155 * (x179 + x178 * x150));
	const GEN_FLT x181 = x122 * x180;
	const GEN_FLT x182 = x120 * (x181 - x121 * x180) + x123 * x180;
	const GEN_FLT x183 = x120 * x182 + x124 * x180;
	const GEN_FLT x184 = x161 * x179 - x178 * x134;
	const GEN_FLT x185 = x177 - x163 * x184;
	const GEN_FLT x186 = x168 * x164;
	const GEN_FLT x187 = x166 * x164;
	const GEN_FLT x188 =
		x146 * (x184 + x160 * x180 -
				x169 * (x139 * (x120 * x183 +
								x120 * (x183 + x120 * (x182 + x120 * (x181 + x127 * x180 - x167 * x180) + x128 * x180) +
										x129 * x180) +
								x125 * x180 + x180 * x130) +
						x186 * x185) +
				x183 * x144 + x187 * x185);
	const GEN_FLT x189 = x34 + x63;
	const GEN_FLT x190 = x49 + x69;
	const GEN_FLT x191 = x54 + x80;
	const GEN_FLT x192 = x87 + x42 * x190 + x5 * x189 + x58 * x191;
	const GEN_FLT x193 = x103 + x90 * x191 + x93 * x190 + x94 * x189;
	const GEN_FLT x194 = (x98 * x192 - x99 * x193) * x107;
	const GEN_FLT x195 = x148 + x109 * x191 + x111 * x190 + x112 * x189;
	const GEN_FLT x196 = x192 * x151 + x193 * x152;
	const GEN_FLT x197 = x147 * (x119 * x195 - x155 * (x196 + x195 * x150));
	const GEN_FLT x198 = x122 * x197;
	const GEN_FLT x199 = x120 * (x198 - x121 * x197) + x123 * x197;
	const GEN_FLT x200 = x120 * x199 + x124 * x197;
	const GEN_FLT x201 = x161 * x196 - x195 * x134;
	const GEN_FLT x202 = x194 - x201 * x163;
	const GEN_FLT x203 =
		x146 * (x201 + x160 * x197 -
				x169 * (x139 * (x120 * (x200 + x120 * (x199 + x120 * (x198 + x127 * x197 - x167 * x197) + x128 * x197) +
										x129 * x197) +
								x125 * x197 + x197 * x130 + x200 * x120) +
						x202 * x186) +
				x200 * x144 + x202 * x187);
	out[0] = x108 - x170 - (-x108 + x170) * x171;
	out[1] = x177 - x188 - (-x177 + x188) * x171;
	out[2] = x194 - x203 - (-x194 + x203) * x171;
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = -x3;
	const GEN_FLT x5 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x6 = pow(x5, 2);
	const GEN_FLT x7 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
							   pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										: (1e-10)),
								   2))
							: (0));
	const GEN_FLT x8 = cos(x0);
	const GEN_FLT x9 = 1 - x8;
	const GEN_FLT x10 = x5 * x9;
	const GEN_FLT x11 = 2 * x10;
	const GEN_FLT x12 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1e-10));
	const GEN_FLT x13 = sin(x12);
	const GEN_FLT x14 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x15 = x14 * x13;
	const GEN_FLT x16 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x17 = cos(x12);
	const GEN_FLT x18 = 1 - x17;
	const GEN_FLT x19 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x20 = x19 * x18;
	const GEN_FLT x21 = x20 * x16;
	const GEN_FLT x22 = x13 * x16;
	const GEN_FLT x23 = x14 * x18;
	const GEN_FLT x24 = x23 * x19;
	const GEN_FLT x25 = pow(x19, 2);
	const GEN_FLT x26 = obj_px + (x17 + x25 * x18) * sensor_x + (x15 + x21) * sensor_z + (-x22 + x24) * sensor_y;
	const GEN_FLT x27 = pow(x14, 2);
	const GEN_FLT x28 = x13 * x19;
	const GEN_FLT x29 = x23 * x16;
	const GEN_FLT x30 = obj_py + (x17 + x27 * x18) * sensor_y + (x22 + x24) * sensor_x + (-x28 + x29) * sensor_z;
	const GEN_FLT x31 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x32 = x8 * x31;
	const GEN_FLT x33 = x2 * x32;
	const GEN_FLT x34 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x35 = x1 * x34;
	const GEN_FLT x36 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x37 = x3 * x5;
	const GEN_FLT x38 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x39 = x9 * x38;
	const GEN_FLT x40 = x9 * x36;
	const GEN_FLT x41 = x36 * x37 + x5 * x39 + x7 * x40;
	const GEN_FLT x42 = pow(x16, 2);
	const GEN_FLT x43 = obj_pz + (x17 + x42 * x18) * sensor_z + (-x15 + x21) * sensor_x + (x28 + x29) * sensor_y;
	const GEN_FLT x44 = x8 * x36;
	const GEN_FLT x45 = x2 * x44;
	const GEN_FLT x46 = x1 * x38;
	const GEN_FLT x47 = x9 * x31;
	const GEN_FLT x48 = x31 * x37 + x34 * x10 + x7 * x47;
	const GEN_FLT x49 = x8 + x6 * x9;
	const GEN_FLT x50 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x51 = x50 * x13;
	const GEN_FLT x52 = -x51;
	const GEN_FLT x53 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x54 = x50 * x17;
	const GEN_FLT x55 = x54 * x16;
	const GEN_FLT x56 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x57 = x56 * x13;
	const GEN_FLT x58 = x50 * x19;
	const GEN_FLT x59 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x60 = x53 * x23 + x58 * x15 + x59 * x20;
	const GEN_FLT x61 = x54 * x14;
	const GEN_FLT x62 = x59 * x13;
	const GEN_FLT x63 = x18 * x16;
	const GEN_FLT x64 = x56 * x20 + x58 * x22 + x63 * x53;
	const GEN_FLT x65 =
		(x52 + x51 * x25 + 2 * x53 * x20) * sensor_x + (-x55 - x57 + x60) * sensor_y + (x61 + x62 + x64) * sensor_z;
	const GEN_FLT x66 = x1 * x31;
	const GEN_FLT x67 = x5 * x40;
	const GEN_FLT x68 = -x66 + x67;
	const GEN_FLT x69 = x54 * x19;
	const GEN_FLT x70 = x53 * x13;
	const GEN_FLT x71 = x56 * x23 + x63 * x59 + x50 * x15 * x16;
	const GEN_FLT x72 =
		(x52 + x51 * x27 + 2 * x59 * x23) * sensor_y + (x55 + x57 + x60) * sensor_x + (-x69 - x70 + x71) * sensor_z;
	const GEN_FLT x73 =
		(x52 + x51 * x42 + 2 * x63 * x56) * sensor_z + (-x61 - x62 + x64) * sensor_x + (x69 + x70 + x71) * sensor_y;
	const GEN_FLT x74 = x1 * x36;
	const GEN_FLT x75 = x31 * x10;
	const GEN_FLT x76 = x74 + x75;
	const GEN_FLT x77 = x65 * x49 + x72 * x68 + x73 * x76;
	const GEN_FLT x78 = x77 + x26 * (x4 + x3 * x6 + x7 * x11) + x30 * (-x33 - x35 + x41) + x43 * (x45 + x46 + x48);
	const GEN_FLT x79 = 1 + x78;
	const GEN_FLT x80 = pow(x31, 2);
	const GEN_FLT x81 = x8 + x9 * x80;
	const GEN_FLT x82 = x1 * x5;
	const GEN_FLT x83 = x40 * x31;
	const GEN_FLT x84 = x82 + x83;
	const GEN_FLT x85 = -x74 + x75;
	const GEN_FLT x86 = lh_pz + x81 * x43 + x84 * x30 + x85 * x26;
	const GEN_FLT x87 = lh_px + x49 * x26 + x68 * x30 + x76 * x43;
	const GEN_FLT x88 = pow(x87, 2);
	const GEN_FLT x89 = x86 / x88;
	const GEN_FLT x90 = x5 * x8;
	const GEN_FLT x91 = x2 * x90;
	const GEN_FLT x92 = x1 * x7;
	const GEN_FLT x93 = x31 * x39 + x40 * x34 + x3 * x31 * x36;
	const GEN_FLT x94 = 2 * x47;
	const GEN_FLT x95 = x81 * x73 + x84 * x72 + x85 * x65;
	const GEN_FLT x96 = x95 + x26 * (-x45 - x46 + x48) + x30 * (x91 + x92 + x93) + x43 * (x4 + x3 * x80 + x94 * x34);
	const GEN_FLT x97 = pow(x87, -1);
	const GEN_FLT x98 = -x97 * x96;
	const GEN_FLT x99 = x88 + pow(x86, 2);
	const GEN_FLT x100 = pow(x99, -1);
	const GEN_FLT x101 = x88 * x100;
	const GEN_FLT x102 = x101 * (x98 + x89 * x79);
	const GEN_FLT x103 = -x82 + x83;
	const GEN_FLT x104 = pow(x36, 2);
	const GEN_FLT x105 = x8 + x9 * x104;
	const GEN_FLT x106 = x66 + x67;
	const GEN_FLT x107 = lh_py + x26 * x106 + x30 * x105 + x43 * x103;
	const GEN_FLT x108 = pow(x107, 2);
	const GEN_FLT x109 = x108 + x99;
	const GEN_FLT x110 = 0.523598775598299 - tilt_1;
	const GEN_FLT x111 = cos(x110);
	const GEN_FLT x112 = pow(x111, -1);
	const GEN_FLT x113 = x112 / sqrt(x109);
	const GEN_FLT x114 = asin(x107 * x113);
	const GEN_FLT x115 = 8.0108022e-06 * x114;
	const GEN_FLT x116 = -8.0108022e-06 - x115;
	const GEN_FLT x117 = 0.0028679863 + x114 * x116;
	const GEN_FLT x118 = 5.3685255e-06 + x114 * x117;
	const GEN_FLT x119 = 0.0076069798 + x118 * x114;
	const GEN_FLT x120 = x119 * x114;
	const GEN_FLT x121 = -8.0108022e-06 - 1.60216044e-05 * x114;
	const GEN_FLT x122 = x117 + x114 * x121;
	const GEN_FLT x123 = x118 + x114 * x122;
	const GEN_FLT x124 = x119 + x114 * x123;
	const GEN_FLT x125 = x120 + x114 * x124;
	const GEN_FLT x126 = sin(x110);
	const GEN_FLT x127 = tan(x110);
	const GEN_FLT x128 = x127 / sqrt(x99);
	const GEN_FLT x129 = -x107 * x128;
	const GEN_FLT x130 = atan2(-x86, x87);
	const GEN_FLT x131 = ogeeMag_1 + x130 - asin(x129);
	const GEN_FLT x132 = curve_1 + sin(x131) * ogeePhase_1;
	const GEN_FLT x133 = x126 * x132;
	const GEN_FLT x134 = x111 + x125 * x133;
	const GEN_FLT x135 = pow(x134, -1);
	const GEN_FLT x136 = pow(x114, 2);
	const GEN_FLT x137 = x132 * x136;
	const GEN_FLT x138 = x137 * x135;
	const GEN_FLT x139 = x129 + x119 * x138;
	const GEN_FLT x140 = pow(1 - pow(x139, 2), -1.0 / 2.0);
	const GEN_FLT x141 = pow(1 - x108 / (x109 * pow(x111, 2)), -1.0 / 2.0);
	const GEN_FLT x142 = 2 * x36;
	const GEN_FLT x143 = x65 * x106 + x72 * x105 + x73 * x103;
	const GEN_FLT x144 =
		x143 + x26 * (x33 + x35 + x41) + x30 * (x4 + x3 * x104 + x39 * x142) + x43 * (-x91 - x92 + x93);
	const GEN_FLT x145 = x113 * x144;
	const GEN_FLT x146 = 2 * x107;
	const GEN_FLT x147 = x144 * x146;
	const GEN_FLT x148 = 2 * x87;
	const GEN_FLT x149 = 2 * x86;
	const GEN_FLT x150 = x96 * x149;
	const GEN_FLT x151 = x150 + x79 * x148;
	const GEN_FLT x152 = (1.0 / 2.0) * x107;
	const GEN_FLT x153 = x112 * x152 / pow(x109, 3.0 / 2.0);
	const GEN_FLT x154 = x145 - (x147 + x151) * x153;
	const GEN_FLT x155 = x141 * x154;
	const GEN_FLT x156 = x116 * x155;
	const GEN_FLT x157 = x114 * (x156 - x115 * x155) + x117 * x155;
	const GEN_FLT x158 = x118 * x141;
	const GEN_FLT x159 = x114 * x157 + x154 * x158;
	const GEN_FLT x160 = 2 * x120 * x132 * x135;
	const GEN_FLT x161 = pow(1 - x100 * x108 * pow(x127, 2), -1.0 / 2.0);
	const GEN_FLT x162 = x127 * x152 / pow(x99, 3.0 / 2.0);
	const GEN_FLT x163 = -x128 * x144;
	const GEN_FLT x164 = x163 + x162 * x151;
	const GEN_FLT x165 = cos(x131) * ogeePhase_1;
	const GEN_FLT x166 = x165 * (x102 - x161 * x164);
	const GEN_FLT x167 = x119 * x135 * x136;
	const GEN_FLT x168 = 2.40324066e-05 * x114;
	const GEN_FLT x169 = x121 * x141;
	const GEN_FLT x170 = x126 * x125;
	const GEN_FLT x171 = x119 * x137 / pow(x134, 2);
	const GEN_FLT x172 =
		x140 * (x164 + x138 * x159 + x160 * x155 + x167 * x166 -
				x171 * (x133 * (x114 * x159 +
								x114 * (x159 + x114 * (x157 + x114 * (x156 - x168 * x155 + x169 * x154) + x122 * x155) +
										x123 * x155) +
								x119 * x155 + x124 * x155) +
						x166 * x170));
	const GEN_FLT x173 = cos(gibPhase_1 + x130 - asin(x139)) * gibMag_1;
	const GEN_FLT x174 = x89 * x78;
	const GEN_FLT x175 = x101 * (x174 + x98);
	const GEN_FLT x176 = 1 + x144;
	const GEN_FLT x177 = x78 * x148;
	const GEN_FLT x178 = x150 + x177;
	const GEN_FLT x179 = x141 * (x113 * x176 - x153 * (x178 + x176 * x146));
	const GEN_FLT x180 = x116 * x179;
	const GEN_FLT x181 = x114 * (x180 - x115 * x179) + x117 * x179;
	const GEN_FLT x182 = x114 * x181 + x118 * x179;
	const GEN_FLT x183 = -x128 * x176 + x162 * x178;
	const GEN_FLT x184 = x175 - x161 * x183;
	const GEN_FLT x185 = x165 * x170;
	const GEN_FLT x186 = x167 * x165;
	const GEN_FLT x187 =
		x140 * (x183 + x160 * x179 -
				x171 * (x133 * (x114 * x182 +
								x114 * (x182 + x114 * (x181 + x114 * (x180 + x121 * x179 - x168 * x179) + x122 * x179) +
										x123 * x179) +
								x119 * x179 + x124 * x179) +
						x185 * x184) +
				x182 * x138 + x186 * x184);
	const GEN_FLT x188 = 1 + x96;
	const GEN_FLT x189 = x101 * (x174 - x97 * x188);
	const GEN_FLT x190 = x177 + x188 * x149;
	const GEN_FLT x191 = x145 - (x147 + x190) * x153;
	const GEN_FLT x192 = x191 * x141;
	const GEN_FLT x193 = x116 * x192;
	const GEN_FLT x194 = x114 * (x193 - x115 * x192) + x117 * x192;
	const GEN_FLT x195 = x114 * x194 + x191 * x158;
	const GEN_FLT x196 = x163 + x162 * x190;
	const GEN_FLT x197 = x189 - x161 * x196;
	const GEN_FLT x198 =
		x140 * (x196 + x160 * x192 -
				x171 * (x133 * (x114 * x195 +
								x114 * (x195 + x114 * (x194 + x114 * (x193 - x168 * x192 + x169 * x191) + x122 * x192) +
										x123 * x192) +
								x119 * x192 + x124 * x192) +
						x185 * x197) +
				x186 * x197 + x195 * x138);
	const GEN_FLT x199 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x200 = x1 * x199;
	const GEN_FLT x201 = -x200;
	const GEN_FLT x202 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x203 = x32 * x199;
	const GEN_FLT x204 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x205 = x1 * x204;
	const GEN_FLT x206 = x74 * x199;
	const GEN_FLT x207 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x208 = x10 * x207 + x40 * x202 + x5 * x206;
	const GEN_FLT x209 = x44 * x199;
	const GEN_FLT x210 = x1 * x207;
	const GEN_FLT x211 = x82 * x31;
	const GEN_FLT x212 = x10 * x204 + x211 * x199 + x47 * x202;
	const GEN_FLT x213 =
		x77 + x26 * (x201 + x11 * x202 + x6 * x200) + x30 * (-x203 - x205 + x208) + x43 * (x209 + x210 + x212);
	const GEN_FLT x214 = x90 * x199;
	const GEN_FLT x215 = x1 * x202;
	const GEN_FLT x216 = x31 * x206 + x40 * x204 + x47 * x207;
	const GEN_FLT x217 =
		x95 + x26 * (-x209 - x210 + x212) + x30 * (x214 + x215 + x216) + x43 * (x201 + x80 * x200 + x94 * x204);
	const GEN_FLT x218 = (x89 * x213 - x97 * x217) * x101;
	const GEN_FLT x219 = 2 * x40;
	const GEN_FLT x220 =
		x143 + x26 * (x203 + x205 + x208) + x30 * (x201 + x200 * x104 + x219 * x207) + x43 * (-x214 - x215 + x216);
	const GEN_FLT x221 = x213 * x148 + x217 * x149;
	const GEN_FLT x222 = -x153 * (x221 + x220 * x146) + x220 * x113;
	const GEN_FLT x223 = x222 * x141;
	const GEN_FLT x224 = x223 * x116;
	const GEN_FLT x225 = x114 * (x224 - x223 * x115) + x223 * x117;
	const GEN_FLT x226 = x222 * x158 + x225 * x114;
	const GEN_FLT x227 = -x220 * x128 + x221 * x162;
	const GEN_FLT x228 = x218 - x227 * x161;
	const GEN_FLT x229 =
		x140 * (x227 -
				x171 * (x133 * (x114 * (x226 + x114 * (x225 + x114 * (x224 + x222 * x169 - x223 * x168) + x223 * x122) +
										x223 * x123) +
								x223 * x119 + x223 * x124 + x226 * x114) +
						x228 * x185) +
				x223 * x160 + x226 * x138 + x228 * x186);
	const GEN_FLT x230 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x231 = x1 * x230;
	const GEN_FLT x232 = -x231;
	const GEN_FLT x233 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x234 = x9 * x233;
	const GEN_FLT x235 = x32 * x230;
	const GEN_FLT x236 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x237 = x1 * x236;
	const GEN_FLT x238 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x239 = x9 * x238;
	const GEN_FLT x240 = x74 * x230;
	const GEN_FLT x241 = x36 * x234 + x5 * x239 + x5 * x240;
	const GEN_FLT x242 = x44 * x230;
	const GEN_FLT x243 = x1 * x238;
	const GEN_FLT x244 = x10 * x236 + x211 * x230 + x31 * x234;
	const GEN_FLT x245 =
		x77 + x26 * (x232 + 2 * x5 * x234 + x6 * x231) + x30 * (-x235 - x237 + x241) + x43 * (x242 + x243 + x244);
	const GEN_FLT x246 = x90 * x230;
	const GEN_FLT x247 = x1 * x233;
	const GEN_FLT x248 = x31 * x239 + x31 * x240 + x40 * x236;
	const GEN_FLT x249 =
		x95 + x26 * (-x242 - x243 + x244) + x30 * (x246 + x247 + x248) + x43 * (x232 + x80 * x231 + x94 * x236);
	const GEN_FLT x250 = (x89 * x245 - x97 * x249) * x101;
	const GEN_FLT x251 =
		x143 + x26 * (x235 + x237 + x241) + x30 * (x232 + x231 * x104 + x239 * x142) + x43 * (-x246 - x247 + x248);
	const GEN_FLT x252 = x245 * x148 + x249 * x149;
	const GEN_FLT x253 = x141 * (-x153 * (x252 + x251 * x146) + x251 * x113);
	const GEN_FLT x254 = x253 * x116;
	const GEN_FLT x255 = x114 * (x254 - x253 * x115) + x253 * x117;
	const GEN_FLT x256 = x253 * x118 + x255 * x114;
	const GEN_FLT x257 = -x251 * x128 + x252 * x162;
	const GEN_FLT x258 = x250 - x257 * x161;
	const GEN_FLT x259 =
		x140 * (x257 -
				x171 * (x133 * (x114 * (x256 + x114 * (x255 + x114 * (x254 + x253 * x121 - x253 * x168) + x253 * x122) +
										x253 * x123) +
								x253 * x119 + x253 * x124 + x256 * x114) +
						x258 * x185) +
				x253 * x160 + x256 * x138 + x258 * x186);
	const GEN_FLT x260 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x261 = x1 * x260;
	const GEN_FLT x262 = -x261;
	const GEN_FLT x263 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x264 = x32 * x260;
	const GEN_FLT x265 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x266 = x1 * x265;
	const GEN_FLT x267 = x74 * x260;
	const GEN_FLT x268 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x269 = x10 * x268 + x40 * x263 + x5 * x267;
	const GEN_FLT x270 = x44 * x260;
	const GEN_FLT x271 = x1 * x268;
	const GEN_FLT x272 = x10 * x265 + x211 * x260 + x47 * x263;
	const GEN_FLT x273 =
		x77 + x26 * (x262 + x11 * x263 + x6 * x261) + x30 * (-x264 - x266 + x269) + x43 * (x270 + x271 + x272);
	const GEN_FLT x274 = x90 * x260;
	const GEN_FLT x275 = x1 * x263;
	const GEN_FLT x276 = x31 * x267 + x40 * x265 + x47 * x268;
	const GEN_FLT x277 =
		x95 + x26 * (-x270 - x271 + x272) + x30 * (x274 + x275 + x276) + x43 * (x262 + x80 * x261 + x94 * x265);
	const GEN_FLT x278 = (x89 * x273 - x97 * x277) * x101;
	const GEN_FLT x279 =
		x143 + x26 * (x264 + x266 + x269) + x30 * (x262 + x219 * x268 + x261 * x104) + x43 * (-x274 - x275 + x276);
	const GEN_FLT x280 = x273 * x148 + x277 * x149;
	const GEN_FLT x281 = x141 * (-x153 * (x280 + x279 * x146) + x279 * x113);
	const GEN_FLT x282 = x281 * x116;
	const GEN_FLT x283 = x114 * (x282 - x281 * x115) + x281 * x117;
	const GEN_FLT x284 = x281 * x118 + x283 * x114;
	const GEN_FLT x285 = -x279 * x128 + x280 * x162;
	const GEN_FLT x286 = x278 - x285 * x161;
	const GEN_FLT x287 =
		x140 * (x285 -
				x171 * (x133 * (x114 * (x284 + x114 * (x283 + x114 * (x282 + x281 * x121 - x281 * x168) + x281 * x122) +
										x281 * x123) +
								x281 * x119 + x281 * x124 + x284 * x114) +
						x286 * x185) +
				x281 * x160 + x284 * x138 + x286 * x186);
	out[0] = x102 - x172 - (-x102 + x172) * x173;
	out[1] = x175 - x187 - (-x175 + x187) * x173;
	out[2] = x189 - x198 - (-x189 + x198) * x173;
	out[3] = x218 - x229 - (-x218 + x229) * x173;
	out[4] = x250 - x259 - (-x250 + x259) * x173;
	out[5] = x278 - x287 - (-x278 + x287) * x173;
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = -x3;
	const GEN_FLT x5 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x6 = pow(x5, 2);
	const GEN_FLT x7 = cos(x0);
	const GEN_FLT x8 = 1 - x7;
	const GEN_FLT x9 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
							   pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										: (1e-10)),
								   2))
							: (0));
	const GEN_FLT x10 = x8 * x9;
	const GEN_FLT x11 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1e-10));
	const GEN_FLT x12 = sin(x11);
	const GEN_FLT x13 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x14 = x13 * x12;
	const GEN_FLT x15 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x16 = cos(x11);
	const GEN_FLT x17 = 1 - x16;
	const GEN_FLT x18 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = x15 * x19;
	const GEN_FLT x21 = x15 * x12;
	const GEN_FLT x22 = x13 * x19;
	const GEN_FLT x23 = pow(x18, 2);
	const GEN_FLT x24 = obj_px + (x16 + x23 * x17) * sensor_x + (x14 + x20) * sensor_z + (-x21 + x22) * sensor_y;
	const GEN_FLT x25 = x7 + x6 * x8;
	const GEN_FLT x26 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x27 = x26 * x12;
	const GEN_FLT x28 = -x27;
	const GEN_FLT x29 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x30 = x26 * x16;
	const GEN_FLT x31 = x30 * x15;
	const GEN_FLT x32 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x33 = x32 * x12;
	const GEN_FLT x34 = x12 * x18;
	const GEN_FLT x35 = x26 * x13;
	const GEN_FLT x36 = x13 * x17;
	const GEN_FLT x37 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x38 = x34 * x35 + x36 * x29 + x37 * x19;
	const GEN_FLT x39 = x30 * x13;
	const GEN_FLT x40 = x37 * x12;
	const GEN_FLT x41 = x15 * x17;
	const GEN_FLT x42 = x32 * x19 + x41 * x29 + x34 * x26 * x15;
	const GEN_FLT x43 =
		(x28 + x23 * x27 + 2 * x29 * x19) * sensor_x + (-x31 - x33 + x38) * sensor_y + (x39 + x40 + x42) * sensor_z;
	const GEN_FLT x44 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x45 = x1 * x44;
	const GEN_FLT x46 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x47 = x5 * x8;
	const GEN_FLT x48 = x46 * x47;
	const GEN_FLT x49 = -x45 + x48;
	const GEN_FLT x50 = x30 * x18;
	const GEN_FLT x51 = x29 * x12;
	const GEN_FLT x52 = x32 * x36 + x35 * x21 + x41 * x37;
	const GEN_FLT x53 = pow(x13, 2);
	const GEN_FLT x54 =
		(x28 + 2 * x36 * x37 + x53 * x27) * sensor_y + (x31 + x33 + x38) * sensor_x + (-x50 - x51 + x52) * sensor_z;
	const GEN_FLT x55 = x41 * x13;
	const GEN_FLT x56 = obj_py + (x16 + x53 * x17) * sensor_y + (x21 + x22) * sensor_x + (-x34 + x55) * sensor_z;
	const GEN_FLT x57 = x2 * x7;
	const GEN_FLT x58 = x57 * x44;
	const GEN_FLT x59 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x60 = x1 * x59;
	const GEN_FLT x61 = x3 * x5;
	const GEN_FLT x62 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x63 = x8 * x62;
	const GEN_FLT x64 = x46 * x10 + x5 * x63 + x61 * x46;
	const GEN_FLT x65 = pow(x15, 2);
	const GEN_FLT x66 = obj_pz + (x16 + x65 * x17) * sensor_z + (-x14 + x20) * sensor_x + (x34 + x55) * sensor_y;
	const GEN_FLT x67 = x57 * x46;
	const GEN_FLT x68 = x1 * x62;
	const GEN_FLT x69 = x8 * x59;
	const GEN_FLT x70 = x44 * x10 + x5 * x69 + x61 * x44;
	const GEN_FLT x71 =
		(x28 + 2 * x41 * x32 + x65 * x27) * sensor_z + (-x39 - x40 + x42) * sensor_x + (x50 + x51 + x52) * sensor_y;
	const GEN_FLT x72 = x1 * x46;
	const GEN_FLT x73 = x44 * x47;
	const GEN_FLT x74 = x72 + x73;
	const GEN_FLT x75 = x24 * (x4 + x3 * x6 + 2 * x5 * x10) + x43 * x25 + x54 * x49 + x56 * (-x58 - x60 + x64) +
						x66 * (x67 + x68 + x70) + x71 * x74;
	const GEN_FLT x76 = pow(x44, 2);
	const GEN_FLT x77 = x7 + x8 * x76;
	const GEN_FLT x78 = x1 * x5;
	const GEN_FLT x79 = x44 * x46;
	const GEN_FLT x80 = x8 * x79;
	const GEN_FLT x81 = x78 + x80;
	const GEN_FLT x82 = -x72 + x73;
	const GEN_FLT x83 = lh_pz + x77 * x66 + x81 * x56 + x82 * x24;
	const GEN_FLT x84 = lh_px + x24 * x25 + x56 * x49 + x74 * x66;
	const GEN_FLT x85 = pow(x84, 2);
	const GEN_FLT x86 = x5 * x57;
	const GEN_FLT x87 = x1 * x9;
	const GEN_FLT x88 = x3 * x79 + x63 * x44 + x69 * x46;
	const GEN_FLT x89 = x24 * (-x67 - x68 + x70) + x56 * (x86 + x87 + x88) + x66 * (x4 + x3 * x76 + 2 * x69 * x44) +
						x71 * x77 + x81 * x54 + x82 * x43;
	const GEN_FLT x90 = x85 + pow(x83, 2);
	const GEN_FLT x91 = pow(x90, -1);
	const GEN_FLT x92 = x85 * x91 * (-x89 / x84 + x83 * x75 / x85);
	const GEN_FLT x93 = -x92;
	const GEN_FLT x94 = 0.523598775598299 - tilt_1;
	const GEN_FLT x95 = tan(x94);
	const GEN_FLT x96 = -x78 + x80;
	const GEN_FLT x97 = pow(x46, 2);
	const GEN_FLT x98 = x7 + x8 * x97;
	const GEN_FLT x99 = x45 + x48;
	const GEN_FLT x100 = lh_py + x56 * x98 + x66 * x96 + x99 * x24;
	const GEN_FLT x101 = pow(x90, -1.0 / 2.0);
	const GEN_FLT x102 = x101 * x100;
	const GEN_FLT x103 = -x95 * x102;
	const GEN_FLT x104 = atan2(-x83, x84);
	const GEN_FLT x105 = ogeeMag_1 + x104 - asin(x103);
	const GEN_FLT x106 = sin(x105);
	const GEN_FLT x107 = curve_1 + x106 * ogeePhase_1;
	const GEN_FLT x108 = pow(x100, 2);
	const GEN_FLT x109 = x108 + x90;
	const GEN_FLT x110 = pow(x109, -1.0 / 2.0);
	const GEN_FLT x111 = cos(x94);
	const GEN_FLT x112 = pow(x111, -1);
	const GEN_FLT x113 = x110 * x112;
	const GEN_FLT x114 = asin(x100 * x113);
	const GEN_FLT x115 = 8.0108022e-06 * x114;
	const GEN_FLT x116 = -8.0108022e-06 - x115;
	const GEN_FLT x117 = 0.0028679863 + x114 * x116;
	const GEN_FLT x118 = 5.3685255e-06 + x114 * x117;
	const GEN_FLT x119 = 0.0076069798 + x118 * x114;
	const GEN_FLT x120 = pow(x114, 2);
	const GEN_FLT x121 = sin(x94);
	const GEN_FLT x122 = x119 * x114;
	const GEN_FLT x123 = -8.0108022e-06 - 1.60216044e-05 * x114;
	const GEN_FLT x124 = x117 + x114 * x123;
	const GEN_FLT x125 = x118 + x114 * x124;
	const GEN_FLT x126 = x119 + x114 * x125;
	const GEN_FLT x127 = x122 + x114 * x126;
	const GEN_FLT x128 = x121 * x127;
	const GEN_FLT x129 = x111 + x107 * x128;
	const GEN_FLT x130 = pow(x129, -1);
	const GEN_FLT x131 = x120 * x130;
	const GEN_FLT x132 = x119 * x131;
	const GEN_FLT x133 = x103 + x107 * x132;
	const GEN_FLT x134 = pow(1 - pow(x133, 2), -1.0 / 2.0);
	const GEN_FLT x135 = pow(x111, -2);
	const GEN_FLT x136 = pow(1 - x108 * x135 / x109, -1.0 / 2.0);
	const GEN_FLT x137 = x24 * (x58 + x60 + x64) + x54 * x98 + x56 * (x4 + x3 * x97 + 2 * x63 * x46) +
						 x66 * (-x86 - x87 + x88) + x71 * x96 + x99 * x43;
	const GEN_FLT x138 = 2 * x83 * x89 + 2 * x84 * x75;
	const GEN_FLT x139 = (1.0 / 2.0) * x100;
	const GEN_FLT x140 = x113 * x137 - x112 * x139 * (x138 + 2 * x100 * x137) / pow(x109, 3.0 / 2.0);
	const GEN_FLT x141 = x136 * x140;
	const GEN_FLT x142 = x116 * x141;
	const GEN_FLT x143 = 2.40324066e-05 * x114;
	const GEN_FLT x144 = x114 * (x142 - x115 * x141) + x117 * x141;
	const GEN_FLT x145 = x114 * x144 + x118 * x141;
	const GEN_FLT x146 = x107 * x121;
	const GEN_FLT x147 =
		x146 * (x114 * x145 +
				x114 * (x145 + x114 * (x144 + x114 * (x142 + x123 * x141 - x141 * x143) + x124 * x141) + x125 * x141) +
				x119 * x141 + x126 * x141);
	const GEN_FLT x148 = pow(x95, 2);
	const GEN_FLT x149 = pow(1 - x91 * x108 * x148, -1.0 / 2.0);
	const GEN_FLT x150 = -x95 * x101 * x137 + x95 * x138 * x139 / pow(x90, 3.0 / 2.0);
	const GEN_FLT x151 = x92 - x149 * x150;
	const GEN_FLT x152 = cos(x105) * ogeePhase_1;
	const GEN_FLT x153 = x151 * x152;
	const GEN_FLT x154 = x107 * x119 * x120 / pow(x129, 2);
	const GEN_FLT x155 = x107 * x131;
	const GEN_FLT x156 = 2 * x107 * x122 * x130;
	const GEN_FLT x157 = x150 + x141 * x156 + x145 * x155;
	const GEN_FLT x158 = x134 * (x157 + x132 * x153 - x154 * (x147 + x128 * x153));
	const GEN_FLT x159 = x158 + x93;
	const GEN_FLT x160 = -gibPhase_1 - x104 + asin(x133);
	const GEN_FLT x161 = cos(x160) * gibMag_1;
	const GEN_FLT x162 = -x158 + x92;
	const GEN_FLT x163 = x162 - x161 * x159;
	const GEN_FLT x164 = x136 * (x140 - x100 * x110 * x121 * x135);
	const GEN_FLT x165 = x116 * x164;
	const GEN_FLT x166 = x114 * (x165 - x115 * x164) + x117 * x164;
	const GEN_FLT x167 = x114 * x166 + x118 * x164;
	const GEN_FLT x168 = x150 + x102 * (1 + x148);
	const GEN_FLT x169 = x92 - x168 * x149;
	const GEN_FLT x170 = x128 * x152;
	const GEN_FLT x171 = x132 * x152;
	const GEN_FLT x172 =
		x134 * (x168 -
				x154 * (x121 +
						x146 * (x114 * x167 +
								x114 * (x167 + x114 * (x166 + x114 * (x165 + x123 * x164 - x164 * x143) + x124 * x164) +
										x125 * x164) +
								x119 * x164 + x126 * x164) +
						x169 * x170 - x107 * x111 * x127) +
				x164 * x156 + x167 * x155 + x169 * x171);
	const GEN_FLT x173 = 1 + x153;
	const GEN_FLT x174 = x134 * (x157 - x154 * (x147 + x128 * x173) + x173 * x132);
	const GEN_FLT x175 = 1 + x151;
	const GEN_FLT x176 = x134 * (x157 - x154 * (x147 + x170 * x175) + x171 * x175);
	const GEN_FLT x177 = x106 + x153;
	const GEN_FLT x178 = x134 * (x157 - x154 * (x147 + x128 * x177) + x177 * x132);
	out[0] = -1 + x163;
	out[1] = -x172 + x92 - x161 * (x172 + x93);
	out[2] = -x174 + x92 - x161 * (x174 + x93);
	out[3] = x162 - x161 * (-1 + x159);
	out[4] = x163 - sin(x160);
	out[5] = -x176 + x92 - x161 * (x176 + x93);
	out[6] = -x178 + x92 - x161 * (x178 + x93);
}

/** Applying function <function reproject at 0x7effc26e3320> */
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x5 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 - x6;
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1e-10));
	const GEN_FLT x11 = cos(x10);
	const GEN_FLT x12 = 1 - x11;
	const GEN_FLT x13 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x14 = sin(x10);
	const GEN_FLT x15 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x16 = x15 * x14;
	const GEN_FLT x17 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x18 = x13 * x12 * x17;
	const GEN_FLT x19 = x14 * x17;
	const GEN_FLT x20 = x15 * x12;
	const GEN_FLT x21 = x20 * x13;
	const GEN_FLT x22 =
		obj_pz + (x11 + pow(x13, 2) * x12) * sensor_z + (x16 + x18) * sensor_y + (-x19 + x21) * sensor_x;
	const GEN_FLT x23 = x14 * x13;
	const GEN_FLT x24 = x20 * x17;
	const GEN_FLT x25 =
		obj_py + (x11 + x12 * pow(x17, 2)) * sensor_y + (-x16 + x18) * sensor_z + (x23 + x24) * sensor_x;
	const GEN_FLT x26 =
		obj_px + (x11 + pow(x15, 2) * x12) * sensor_x + (x19 + x21) * sensor_z + (-x23 + x24) * sensor_y;
	const GEN_FLT x27 = x1 * x4;
	const GEN_FLT x28 = x2 * x8;
	const GEN_FLT x29 = lh_py + x25 * (x6 + pow(x5, 2) * x7) + (x27 + x28) * x26 + (-x3 + x9) * x22;
	const GEN_FLT x30 = x1 * x5;
	const GEN_FLT x31 = x2 * x4 * x7;
	const GEN_FLT x32 = lh_pz + x22 * (x6 + pow(x4, 2) * x7) + (x3 + x9) * x25 + (-x30 + x31) * x26;
	const GEN_FLT x33 = -x32;
	const GEN_FLT x34 = pow(x32, 2);
	const GEN_FLT x35 = lh_px + x26 * (x6 + pow(x2, 2) * x7) + (-x27 + x28) * x25 + (x30 + x31) * x22;
	const GEN_FLT x36 = atan2(x35, x33);
	const GEN_FLT x37 = -phase_0 - x36 - asin(x29 * tilt_0 / sqrt(x34 + pow(x35, 2)));
	const GEN_FLT x38 = -phase_1 - asin(x35 * tilt_1 / sqrt(x34 + pow(x29, 2))) - atan2(-x29, x33);
	out[0] = x37 - cos(1.5707963267949 + gibPhase_0 + x37) * gibMag_0 + pow(atan2(x29, x33), 2) * curve_0;
	out[1] = x38 + pow(x36, 2) * curve_1 - cos(1.5707963267949 + gibPhase_1 + x38) * gibMag_1;
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x5 = cos(x0);
	const GEN_FLT x6 = 1 - x5;
	const GEN_FLT x7 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = -x3 + x9;
	const GEN_FLT x11 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x12 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x13 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1e-10));
	const GEN_FLT x14 = cos(x13);
	const GEN_FLT x15 = x14 * x12;
	const GEN_FLT x16 = x15 * x11;
	const GEN_FLT x17 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x18 = sin(x13);
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x21 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x22 = x21 * x18;
	const GEN_FLT x23 = x22 * x20;
	const GEN_FLT x24 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x25 = 1 - x14;
	const GEN_FLT x26 = x25 * x21;
	const GEN_FLT x27 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x28 = x25 * x27;
	const GEN_FLT x29 = x20 * x28 + x23 * x12 + x24 * x26;
	const GEN_FLT x30 = x20 * x15;
	const GEN_FLT x31 = x24 * x18;
	const GEN_FLT x32 = x22 * x11;
	const GEN_FLT x33 = x25 * x17;
	const GEN_FLT x34 = x28 * x11 + x32 * x12 + x33 * x21;
	const GEN_FLT x35 = x12 * x18;
	const GEN_FLT x36 = -x35;
	const GEN_FLT x37 = pow(x21, 2);
	const GEN_FLT x38 =
		(x16 + x19 + x29) * sensor_x + (-x30 - x31 + x34) * sensor_z + (x36 + 2 * x21 * x28 + x35 * x37) * sensor_y;
	const GEN_FLT x39 = x38 * x10;
	const GEN_FLT x40 = x21 * x15;
	const GEN_FLT x41 = x27 * x18;
	const GEN_FLT x42 = x20 * x18;
	const GEN_FLT x43 = x25 * x11;
	const GEN_FLT x44 = x33 * x20 + x43 * x24 + x42 * x12 * x11;
	const GEN_FLT x45 = pow(x11, 2);
	const GEN_FLT x46 = 2 * x11;
	const GEN_FLT x47 =
		(x30 + x31 + x34) * sensor_y + (x36 + x45 * x35 + x46 * x33) * sensor_z + (-x40 - x41 + x44) * sensor_x;
	const GEN_FLT x48 = x1 * x7;
	const GEN_FLT x49 = x4 * x6;
	const GEN_FLT x50 = x2 * x49;
	const GEN_FLT x51 = x48 + x50;
	const GEN_FLT x52 = x51 * x47;
	const GEN_FLT x53 = pow(x4, 2);
	const GEN_FLT x54 = x5 + x6 * x53;
	const GEN_FLT x55 = pow(x20, 2);
	const GEN_FLT x56 = x25 * x20;
	const GEN_FLT x57 = 2 * x56;
	const GEN_FLT x58 =
		(-x16 - x19 + x29) * sensor_y + (x36 + x55 * x35 + x57 * x24) * sensor_x + (x40 + x41 + x44) * sensor_z;
	const GEN_FLT x59 = 1 + x58;
	const GEN_FLT x60 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x61 = x1 * x60;
	const GEN_FLT x62 = -x61;
	const GEN_FLT x63 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x64 = x56 * x11;
	const GEN_FLT x65 = x11 * x18;
	const GEN_FLT x66 = x56 * x21;
	const GEN_FLT x67 = obj_px + (x14 + x55 * x25) * sensor_x + (x22 + x64) * sensor_z + (-x65 + x66) * sensor_y;
	const GEN_FLT x68 = x43 * x21;
	const GEN_FLT x69 = obj_py + (x14 + x37 * x25) * sensor_y + (x65 + x66) * sensor_x + (-x42 + x68) * sensor_z;
	const GEN_FLT x70 = x5 * x60;
	const GEN_FLT x71 = x2 * x70;
	const GEN_FLT x72 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x73 = x1 * x72;
	const GEN_FLT x74 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x75 = x6 * x74;
	const GEN_FLT x76 = x4 * x75 + x8 * x63 + x4 * x7 * x61;
	const GEN_FLT x77 = obj_pz + (x14 + x45 * x25) * sensor_z + (-x22 + x64) * sensor_x + (x42 + x68) * sensor_y;
	const GEN_FLT x78 = x7 * x70;
	const GEN_FLT x79 = x1 * x74;
	const GEN_FLT x80 = x2 * x6;
	const GEN_FLT x81 = x2 * x61;
	const GEN_FLT x82 = x4 * x81 + x72 * x49 + x80 * x63;
	const GEN_FLT x83 = x67 * (x62 + x61 * x53 + 2 * x63 * x49) + x69 * (-x71 - x73 + x76) + x77 * (x78 + x79 + x82);
	const GEN_FLT x84 = x39 + x52 + x83 + x54 * x59;
	const GEN_FLT x85 = pow(x2, 2);
	const GEN_FLT x86 = x5 + x6 * x85;
	const GEN_FLT x87 = x1 * x4;
	const GEN_FLT x88 = x2 * x8;
	const GEN_FLT x89 = x87 + x88;
	const GEN_FLT x90 = -x48 + x50;
	const GEN_FLT x91 = lh_pz + x67 * x90 + x86 * x77 + x89 * x69;
	const GEN_FLT x92 = pow(x91, -1);
	const GEN_FLT x93 = x89 * x38;
	const GEN_FLT x94 = x4 * x70;
	const GEN_FLT x95 = x1 * x63;
	const GEN_FLT x96 = x2 * x75 + x7 * x81 + x8 * x72;
	const GEN_FLT x97 = x67 * (-x78 - x79 + x82) + x69 * (x94 + x95 + x96) + x77 * (x62 + 2 * x80 * x72 + x85 * x61);
	const GEN_FLT x98 = x97 + x86 * x47;
	const GEN_FLT x99 = x93 + x98 + x59 * x90;
	const GEN_FLT x100 = lh_px + x67 * x54 + x69 * x10 + x77 * x51;
	const GEN_FLT x101 = pow(x91, 2);
	const GEN_FLT x102 = pow(x101, -1);
	const GEN_FLT x103 = x100 * x102;
	const GEN_FLT x104 = pow(x100, 2);
	const GEN_FLT x105 = x101 + x104;
	const GEN_FLT x106 = pow(x105, -1);
	const GEN_FLT x107 = x101 * x106;
	const GEN_FLT x108 = x107 * (-x84 * x92 + x99 * x103);
	const GEN_FLT x109 = -x87 + x88;
	const GEN_FLT x110 = pow(x7, 2);
	const GEN_FLT x111 = x5 + x6 * x110;
	const GEN_FLT x112 = x3 + x9;
	const GEN_FLT x113 = lh_py + x67 * x112 + x69 * x111 + x77 * x109;
	const GEN_FLT x114 = pow(x113, 2);
	const GEN_FLT x115 = pow(1 - x106 * x114 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x116 = 2 * x100;
	const GEN_FLT x117 = 2 * x91;
	const GEN_FLT x118 = x99 * x117;
	const GEN_FLT x119 = (1.0 / 2.0) * x113 * tilt_0 / pow(x105, 3.0 / 2.0);
	const GEN_FLT x120 = x38 * x111;
	const GEN_FLT x121 = x47 * x109;
	const GEN_FLT x122 = x67 * (x71 + x73 + x76) + x69 * (x62 + x61 * x110 + 2 * x7 * x75) + x77 * (-x94 - x95 + x96);
	const GEN_FLT x123 = x120 + x121 + x122 + x59 * x112;
	const GEN_FLT x124 = tilt_0 / sqrt(x105);
	const GEN_FLT x125 = -x108 - x115 * (-x119 * (x118 + x84 * x116) + x124 * x123);
	const GEN_FLT x126 = -x91;
	const GEN_FLT x127 = atan2(x100, x126);
	const GEN_FLT x128 = sin(1.5707963267949 + gibPhase_0 - phase_0 - x127 - asin(x113 * x124)) * gibMag_0;
	const GEN_FLT x129 = x92 * x123;
	const GEN_FLT x130 = x102 * x113;
	const GEN_FLT x131 = x99 * x130;
	const GEN_FLT x132 = x101 + x114;
	const GEN_FLT x133 = pow(x132, -1);
	const GEN_FLT x134 = x101 * x133;
	const GEN_FLT x135 = 2 * x134 * atan2(x113, x126) * curve_0;
	const GEN_FLT x136 = 1 + x38;
	const GEN_FLT x137 = x83 + x54 * x58;
	const GEN_FLT x138 = x137 + x52 + x10 * x136;
	const GEN_FLT x139 = x58 * x90;
	const GEN_FLT x140 = x139 + x98 + x89 * x136;
	const GEN_FLT x141 = x107 * (x103 * x140 - x92 * x138);
	const GEN_FLT x142 = x117 * x140;
	const GEN_FLT x143 = x122 + x58 * x112;
	const GEN_FLT x144 = x121 + x143 + x111 * x136;
	const GEN_FLT x145 = -x141 - x115 * (-x119 * (x142 + x116 * x138) + x124 * x144);
	const GEN_FLT x146 = x92 * x144;
	const GEN_FLT x147 = x130 * x140;
	const GEN_FLT x148 = 1 + x47;
	const GEN_FLT x149 = x137 + x39 + x51 * x148;
	const GEN_FLT x150 = x139 + x93 + x97 + x86 * x148;
	const GEN_FLT x151 = x107 * (x103 * x150 - x92 * x149);
	const GEN_FLT x152 = x117 * x150;
	const GEN_FLT x153 = x120 + x143 + x109 * x148;
	const GEN_FLT x154 = -x151 - x115 * (-x119 * (x152 + x116 * x149) + x124 * x153);
	const GEN_FLT x155 = x92 * x153;
	const GEN_FLT x156 = x130 * x150;
	const GEN_FLT x157 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x158 = x18 * x157;
	const GEN_FLT x159 = -x158;
	const GEN_FLT x160 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x161 = x25 * x160;
	const GEN_FLT x162 = x14 * x157;
	const GEN_FLT x163 = x11 * x162;
	const GEN_FLT x164 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x165 = x18 * x164;
	const GEN_FLT x166 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x167 = x21 * x161 + x23 * x157 + x56 * x166;
	const GEN_FLT x168 = x21 * x162;
	const GEN_FLT x169 = x18 * x166;
	const GEN_FLT x170 = x20 * x11;
	const GEN_FLT x171 = x11 * x161 + x170 * x158 + x56 * x164;
	const GEN_FLT x172 = (x159 + 2 * x20 * x161 + x55 * x158) * sensor_x + (-x163 - x165 + x167) * sensor_y +
						 (x168 + x169 + x171) * sensor_z;
	const GEN_FLT x173 = x20 * x162;
	const GEN_FLT x174 = x18 * x160;
	const GEN_FLT x175 = x26 * x164 + x32 * x157 + x43 * x166;
	const GEN_FLT x176 = 2 * x26;
	const GEN_FLT x177 = sensor_y * (x159 + x166 * x176 + x37 * x158) + (x163 + x165 + x167) * sensor_x +
						 (-x173 - x174 + x175) * sensor_z;
	const GEN_FLT x178 = 2 * x43;
	const GEN_FLT x179 = sensor_z * (x159 + x164 * x178 + x45 * x158) + (-x168 - x169 + x171) * sensor_x +
						 (x173 + x174 + x175) * sensor_y;
	const GEN_FLT x180 = x83 + x10 * x177 + x51 * x179 + x54 * x172;
	const GEN_FLT x181 = x97 + x86 * x179 + x89 * x177 + x90 * x172;
	const GEN_FLT x182 = x107 * (x103 * x181 - x92 * x180);
	const GEN_FLT x183 = x117 * x181;
	const GEN_FLT x184 = x122 + x109 * x179 + x111 * x177 + x112 * x172;
	const GEN_FLT x185 = -x182 - x115 * (-x119 * (x183 + x116 * x180) + x124 * x184);
	const GEN_FLT x186 = x92 * x184;
	const GEN_FLT x187 = x181 * x130;
	const GEN_FLT x188 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x189 = x18 * x188;
	const GEN_FLT x190 = -x189;
	const GEN_FLT x191 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x192 = x14 * x188;
	const GEN_FLT x193 = x11 * x192;
	const GEN_FLT x194 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x195 = x18 * x194;
	const GEN_FLT x196 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x197 = x23 * x188 + x26 * x191 + x56 * x196;
	const GEN_FLT x198 = x21 * x192;
	const GEN_FLT x199 = x18 * x196;
	const GEN_FLT x200 = x170 * x189 + x43 * x191 + x56 * x194;
	const GEN_FLT x201 = (x190 + x55 * x189 + x57 * x191) * sensor_x + (-x193 - x195 + x197) * sensor_y +
						 (x198 + x199 + x200) * sensor_z;
	const GEN_FLT x202 = x20 * x192;
	const GEN_FLT x203 = x18 * x191;
	const GEN_FLT x204 = x26 * x194 + x32 * x188 + x43 * x196;
	const GEN_FLT x205 = sensor_y * (x190 + x176 * x196 + x37 * x189) + (x193 + x195 + x197) * sensor_x +
						 (-x202 - x203 + x204) * sensor_z;
	const GEN_FLT x206 = sensor_z * (x190 + x178 * x194 + x45 * x189) + (-x198 - x199 + x200) * sensor_x +
						 (x202 + x203 + x204) * sensor_y;
	const GEN_FLT x207 = x83 + x10 * x205 + x51 * x206 + x54 * x201;
	const GEN_FLT x208 = x97 + x86 * x206 + x89 * x205 + x90 * x201;
	const GEN_FLT x209 = x208 * x102;
	const GEN_FLT x210 = x107 * (x209 * x100 - x92 * x207);
	const GEN_FLT x211 = x208 * x117;
	const GEN_FLT x212 = x122 + x201 * x112 + x205 * x111 + x206 * x109;
	const GEN_FLT x213 = -x210 - x115 * (-x119 * (x211 + x207 * x116) + x212 * x124);
	const GEN_FLT x214 = x92 * x212;
	const GEN_FLT x215 = x209 * x113;
	const GEN_FLT x216 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x217 = x14 * x216;
	const GEN_FLT x218 = x11 * x217;
	const GEN_FLT x219 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x220 = x18 * x219;
	const GEN_FLT x221 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x222 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x223 = x23 * x216 + x26 * x221 + x56 * x222;
	const GEN_FLT x224 = x20 * x217;
	const GEN_FLT x225 = x18 * x221;
	const GEN_FLT x226 = x25 * x219;
	const GEN_FLT x227 = x21 * x226 + x32 * x216 + x43 * x222;
	const GEN_FLT x228 = x18 * x216;
	const GEN_FLT x229 = -x228;
	const GEN_FLT x230 = sensor_y * (x229 + x222 * x176 + x37 * x228) + (x218 + x220 + x223) * sensor_x +
						 (-x224 - x225 + x227) * sensor_z;
	const GEN_FLT x231 = x21 * x217;
	const GEN_FLT x232 = x18 * x222;
	const GEN_FLT x233 = x20 * x226 + x228 * x170 + x43 * x221;
	const GEN_FLT x234 = (-x218 - x220 + x223) * sensor_y + (x229 + x55 * x228 + x57 * x221) * sensor_x +
						 (x231 + x232 + x233) * sensor_z;
	const GEN_FLT x235 = (x224 + x225 + x227) * sensor_y + (x229 + x45 * x228 + x46 * x226) * sensor_z +
						 (-x231 - x232 + x233) * sensor_x;
	const GEN_FLT x236 = x83 + x10 * x230 + x51 * x235 + x54 * x234;
	const GEN_FLT x237 = x97 + x86 * x235 + x89 * x230 + x90 * x234;
	const GEN_FLT x238 = x237 * x102;
	const GEN_FLT x239 = x107 * (x238 * x100 - x92 * x236);
	const GEN_FLT x240 = x237 * x117;
	const GEN_FLT x241 = x122 + x230 * x111 + x234 * x112 + x235 * x109;
	const GEN_FLT x242 = -x239 - x115 * (-x119 * (x240 + x236 * x116) + x241 * x124);
	const GEN_FLT x243 = x92 * x241;
	const GEN_FLT x244 = x238 * x113;
	const GEN_FLT x245 = pow(1 - x104 * x133 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x246 = 2 * x113;
	const GEN_FLT x247 = (1.0 / 2.0) * x100 * tilt_1 / pow(x132, 3.0 / 2.0);
	const GEN_FLT x248 = tilt_1 / sqrt(x132);
	const GEN_FLT x249 = -x245 * (-x247 * (x118 + x246 * x123) + x84 * x248) - (x129 - x131) * x134;
	const GEN_FLT x250 =
		sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x248 * x100) - atan2(-x113, x126)) * gibMag_1;
	const GEN_FLT x251 = 2 * x127 * curve_1;
	const GEN_FLT x252 = -x245 * (-x247 * (x142 + x246 * x144) + x248 * x138) - (x146 - x147) * x134;
	const GEN_FLT x253 = -x245 * (-x247 * (x152 + x246 * x153) + x248 * x149) - (x155 - x156) * x134;
	const GEN_FLT x254 = -x245 * (-x247 * (x183 + x246 * x184) + x248 * x180) - (x186 - x187) * x134;
	const GEN_FLT x255 = -x245 * (x207 * x248 - x247 * (x211 + x212 * x246)) - (x214 - x215) * x134;
	const GEN_FLT x256 = -x245 * (x236 * x248 - x247 * (x240 + x241 * x246)) - (x243 - x244) * x134;
	out[0] = x125 + x128 * x125 + (-x129 + x131) * x135;
	out[1] = x145 + x128 * x145 + (-x146 + x147) * x135;
	out[2] = x154 + x128 * x154 + (-x155 + x156) * x135;
	out[3] = x185 + x128 * x185 + (-x186 + x187) * x135;
	out[4] = x213 + x213 * x128 + (-x214 + x215) * x135;
	out[5] = x242 + x242 * x128 + (-x243 + x244) * x135;
	out[6] = x249 + x250 * x249 + x251 * x108;
	out[7] = x252 + x250 * x252 + x251 * x141;
	out[8] = x253 + x250 * x253 + x251 * x151;
	out[9] = x254 + x250 * x254 + x251 * x182;
	out[10] = x255 + x210 * x251 + x250 * x255;
	out[11] = x256 + x239 * x251 + x250 * x256;
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 - x1;
	const GEN_FLT x3 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x4 = pow(x3, 2);
	const GEN_FLT x5 = x1 + x2 * x4;
	const GEN_FLT x6 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1e-10));
	const GEN_FLT x7 = cos(x6);
	const GEN_FLT x8 = 1 - x7;
	const GEN_FLT x9 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								  : (1e-10)))
							? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											 : (1e-10)))
							: (0));
	const GEN_FLT x10 = pow(x9, 2);
	const GEN_FLT x11 = x7 + x8 * x10;
	const GEN_FLT x12 = sin(x6);
	const GEN_FLT x13 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x14 = x13 * x12;
	const GEN_FLT x15 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x16 = x8 * x15;
	const GEN_FLT x17 = x9 * x16;
	const GEN_FLT x18 = x14 + x17;
	const GEN_FLT x19 = x15 * x12;
	const GEN_FLT x20 = x8 * x9;
	const GEN_FLT x21 = x20 * x13;
	const GEN_FLT x22 = -x19 + x21;
	const GEN_FLT x23 = obj_pz + x11 * sensor_z + x18 * sensor_y + x22 * sensor_x;
	const GEN_FLT x24 = sin(x0);
	const GEN_FLT x25 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (1));
	const GEN_FLT x26 = x24 * x25;
	const GEN_FLT x27 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x28 = x2 * x27;
	const GEN_FLT x29 = x3 * x28;
	const GEN_FLT x30 = x26 + x29;
	const GEN_FLT x31 = pow(x15, 2);
	const GEN_FLT x32 = x7 + x8 * x31;
	const GEN_FLT x33 = -x14 + x17;
	const GEN_FLT x34 = x9 * x12;
	const GEN_FLT x35 = x13 * x16;
	const GEN_FLT x36 = x34 + x35;
	const GEN_FLT x37 = obj_py + x32 * sensor_y + x33 * sensor_z + x36 * sensor_x;
	const GEN_FLT x38 = x24 * x27;
	const GEN_FLT x39 = x2 * x25;
	const GEN_FLT x40 = x3 * x39;
	const GEN_FLT x41 = -x38 + x40;
	const GEN_FLT x42 = x19 + x21;
	const GEN_FLT x43 = -x34 + x35;
	const GEN_FLT x44 = pow(x13, 2);
	const GEN_FLT x45 = x7 + x8 * x44;
	const GEN_FLT x46 = obj_px + x42 * sensor_z + x43 * sensor_y + x45 * sensor_x;
	const GEN_FLT x47 = lh_pz + x30 * x37 + x41 * x46 + x5 * x23;
	const GEN_FLT x48 = pow(x47, -1);
	const GEN_FLT x49 = pow(x25, 2);
	const GEN_FLT x50 = x1 + x2 * x49;
	const GEN_FLT x51 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x52 = x51 * x12;
	const GEN_FLT x53 = -x52;
	const GEN_FLT x54 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x55 = x8 * x13;
	const GEN_FLT x56 = x7 * x51;
	const GEN_FLT x57 = x9 * x56;
	const GEN_FLT x58 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x59 = x58 * x12;
	const GEN_FLT x60 = x51 * x19;
	const GEN_FLT x61 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x62 = x8 * x61;
	const GEN_FLT x63 = x54 * x16 + x60 * x13 + x62 * x13;
	const GEN_FLT x64 = x56 * x15;
	const GEN_FLT x65 = x61 * x12;
	const GEN_FLT x66 = x54 * x20 + x58 * x55 + x51 * x34 * x13;
	const GEN_FLT x67 =
		(x53 + x52 * x44 + 2 * x54 * x55) * sensor_x + (-x57 - x59 + x63) * sensor_y + (x64 + x65 + x66) * sensor_z;
	const GEN_FLT x68 = x45 + x67;
	const GEN_FLT x69 = x3 * x24;
	const GEN_FLT x70 = x25 * x28;
	const GEN_FLT x71 = -x69 + x70;
	const GEN_FLT x72 = x56 * x13;
	const GEN_FLT x73 = x54 * x12;
	const GEN_FLT x74 = x58 * x16 + x9 * x60 + x9 * x62;
	const GEN_FLT x75 =
		(x53 + x52 * x31 + 2 * x61 * x16) * sensor_y + (x57 + x59 + x63) * sensor_x + (-x72 - x73 + x74) * sensor_z;
	const GEN_FLT x76 = x36 + x75;
	const GEN_FLT x77 =
		(x53 + x52 * x10 + 2 * x58 * x20) * sensor_z + (-x64 - x65 + x66) * sensor_x + (x72 + x73 + x74) * sensor_y;
	const GEN_FLT x78 = x22 + x77;
	const GEN_FLT x79 = x38 + x40;
	const GEN_FLT x80 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x81 = x80 * x24;
	const GEN_FLT x82 = -x81;
	const GEN_FLT x83 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x84 = x1 * x80;
	const GEN_FLT x85 = x3 * x84;
	const GEN_FLT x86 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x87 = x86 * x24;
	const GEN_FLT x88 = x81 * x25;
	const GEN_FLT x89 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x90 = x2 * x89;
	const GEN_FLT x91 = x83 * x28 + x88 * x27 + x90 * x25;
	const GEN_FLT x92 = x84 * x27;
	const GEN_FLT x93 = x89 * x24;
	const GEN_FLT x94 = x2 * x3;
	const GEN_FLT x95 = x3 * x88 + x83 * x94 + x86 * x39;
	const GEN_FLT x96 = x23 * (x92 + x93 + x95) + x37 * (-x85 - x87 + x91) + x46 * (x82 + x81 * x49 + 2 * x83 * x39);
	const GEN_FLT x97 = x96 + x68 * x50 + x71 * x76 + x79 * x78;
	const GEN_FLT x98 = lh_px + x50 * x46 + x71 * x37 + x79 * x23;
	const GEN_FLT x99 = pow(x47, 2);
	const GEN_FLT x100 = pow(x99, -1);
	const GEN_FLT x101 = x84 * x25;
	const GEN_FLT x102 = x83 * x24;
	const GEN_FLT x103 = x3 * x90 + x86 * x28 + x3 * x81 * x27;
	const GEN_FLT x104 = x23 * (x82 + x4 * x81 + 2 * x86 * x94) + x37 * (x101 + x102 + x103) + x46 * (-x92 - x93 + x95);
	const GEN_FLT x105 = x104 + x5 * x78 + x68 * x41 + x76 * x30;
	const GEN_FLT x106 = x100 * x105;
	const GEN_FLT x107 = pow(x98, 2);
	const GEN_FLT x108 = x107 + x99;
	const GEN_FLT x109 = pow(x108, -1);
	const GEN_FLT x110 = x99 * x109;
	const GEN_FLT x111 = x110 * (-x97 * x48 + x98 * x106);
	const GEN_FLT x112 = -x26 + x29;
	const GEN_FLT x113 = pow(x27, 2);
	const GEN_FLT x114 = x1 + x2 * x113;
	const GEN_FLT x115 = x69 + x70;
	const GEN_FLT x116 = lh_py + x23 * x112 + x37 * x114 + x46 * x115;
	const GEN_FLT x117 = pow(x116, 2);
	const GEN_FLT x118 = pow(1 - x109 * x117 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x119 = 2 * x98;
	const GEN_FLT x120 = 2 * x47;
	const GEN_FLT x121 = x105 * x120;
	const GEN_FLT x122 = (1.0 / 2.0) * x116 * tilt_0 / pow(x108, 3.0 / 2.0);
	const GEN_FLT x123 =
		x23 * (-x101 - x102 + x103) + x37 * (x82 + x81 * x113 + 2 * x89 * x28) + x46 * (x85 + x87 + x91);
	const GEN_FLT x124 = x123 + x68 * x115 + x76 * x114 + x78 * x112;
	const GEN_FLT x125 = tilt_0 / sqrt(x108);
	const GEN_FLT x126 = -x111 - x118 * (-x122 * (x121 + x97 * x119) + x124 * x125);
	const GEN_FLT x127 = -x47;
	const GEN_FLT x128 = atan2(x98, x127);
	const GEN_FLT x129 = sin(1.5707963267949 + gibPhase_0 - phase_0 - x128 - asin(x116 * x125)) * gibMag_0;
	const GEN_FLT x130 = x48 * x124;
	const GEN_FLT x131 = x106 * x116;
	const GEN_FLT x132 = x117 + x99;
	const GEN_FLT x133 = pow(x132, -1);
	const GEN_FLT x134 = x99 * x133;
	const GEN_FLT x135 = 2 * x134 * atan2(x116, x127) * curve_0;
	const GEN_FLT x136 = x32 + x75;
	const GEN_FLT x137 = x43 + x67;
	const GEN_FLT x138 = x18 + x77;
	const GEN_FLT x139 = x96 + x50 * x137 + x71 * x136 + x79 * x138;
	const GEN_FLT x140 = x104 + x30 * x136 + x41 * x137 + x5 * x138;
	const GEN_FLT x141 = x98 * x100;
	const GEN_FLT x142 = x110 * (x140 * x141 - x48 * x139);
	const GEN_FLT x143 = x120 * x140;
	const GEN_FLT x144 = x123 + x112 * x138 + x114 * x136 + x115 * x137;
	const GEN_FLT x145 = -x142 - x118 * (-x122 * (x143 + x119 * x139) + x125 * x144);
	const GEN_FLT x146 = x48 * x144;
	const GEN_FLT x147 = x100 * x116;
	const GEN_FLT x148 = x140 * x147;
	const GEN_FLT x149 = x42 + x67;
	const GEN_FLT x150 = x33 + x75;
	const GEN_FLT x151 = x11 + x77;
	const GEN_FLT x152 = x96 + x50 * x149 + x71 * x150 + x79 * x151;
	const GEN_FLT x153 = x104 + x30 * x150 + x41 * x149 + x5 * x151;
	const GEN_FLT x154 = x110 * (x141 * x153 - x48 * x152);
	const GEN_FLT x155 = x120 * x153;
	const GEN_FLT x156 = x123 + x112 * x151 + x114 * x150 + x115 * x149;
	const GEN_FLT x157 = -x154 - x118 * (-x122 * (x155 + x119 * x152) + x125 * x156);
	const GEN_FLT x158 = x48 * x156;
	const GEN_FLT x159 = x147 * x153;
	const GEN_FLT x160 = 2 * x116;
	const GEN_FLT x161 = (1.0 / 2.0) * x98 * tilt_1 / pow(x132, 3.0 / 2.0);
	const GEN_FLT x162 = tilt_1 / sqrt(x132);
	const GEN_FLT x163 = pow(1 - x107 * x133 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x164 = -x163 * (-x161 * (x121 + x124 * x160) + x97 * x162) - (x130 - x131) * x134;
	const GEN_FLT x165 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x98 * x162) - atan2(-x116, x127)) * gibMag_1;
	const GEN_FLT x166 = 2 * x128 * curve_1;
	const GEN_FLT x167 = -x163 * (-x161 * (x143 + x160 * x144) + x162 * x139) - (x146 - x148) * x134;
	const GEN_FLT x168 = -x163 * (-x161 * (x155 + x160 * x156) + x162 * x152) - (x158 - x159) * x134;
	out[0] = x126 + x126 * x129 + (-x130 + x131) * x135;
	out[1] = x145 + x129 * x145 + (-x146 + x148) * x135;
	out[2] = x157 + x129 * x157 + (-x158 + x159) * x135;
	out[3] = x164 + x111 * x166 + x165 * x164;
	out[4] = x167 + x166 * x142 + x167 * x165;
	out[5] = x168 + x166 * x154 + x168 * x165;
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = -x3;
	const GEN_FLT x5 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x6 = pow(x5, 2);
	const GEN_FLT x7 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
							   pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										: (1e-10)),
								   2))
							: (0));
	const GEN_FLT x8 = cos(x0);
	const GEN_FLT x9 = 1 - x8;
	const GEN_FLT x10 = x5 * x9;
	const GEN_FLT x11 = 2 * x10;
	const GEN_FLT x12 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1e-10));
	const GEN_FLT x13 = sin(x12);
	const GEN_FLT x14 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x15 = x14 * x13;
	const GEN_FLT x16 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x17 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x18 = cos(x12);
	const GEN_FLT x19 = 1 - x18;
	const GEN_FLT x20 = x19 * x17;
	const GEN_FLT x21 = x20 * x16;
	const GEN_FLT x22 = x13 * x17;
	const GEN_FLT x23 = x14 * x19;
	const GEN_FLT x24 = x23 * x16;
	const GEN_FLT x25 = pow(x16, 2);
	const GEN_FLT x26 = obj_px + (x18 + x25 * x19) * sensor_x + (x15 + x21) * sensor_z + (-x22 + x24) * sensor_y;
	const GEN_FLT x27 = pow(x14, 2);
	const GEN_FLT x28 = x13 * x16;
	const GEN_FLT x29 = x23 * x17;
	const GEN_FLT x30 = obj_py + (x18 + x27 * x19) * sensor_y + (x22 + x24) * sensor_x + (-x28 + x29) * sensor_z;
	const GEN_FLT x31 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x32 = x2 * x8;
	const GEN_FLT x33 = x32 * x31;
	const GEN_FLT x34 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x35 = x1 * x34;
	const GEN_FLT x36 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x37 = x5 * x36;
	const GEN_FLT x38 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x39 = x9 * x38;
	const GEN_FLT x40 = x9 * x36;
	const GEN_FLT x41 = x3 * x37 + x5 * x39 + x7 * x40;
	const GEN_FLT x42 = pow(x17, 2);
	const GEN_FLT x43 = obj_pz + (x18 + x42 * x19) * sensor_z + (-x15 + x21) * sensor_x + (x28 + x29) * sensor_y;
	const GEN_FLT x44 = x32 * x36;
	const GEN_FLT x45 = x1 * x38;
	const GEN_FLT x46 = x9 * x31;
	const GEN_FLT x47 = x5 * x31;
	const GEN_FLT x48 = x3 * x47 + x34 * x10 + x7 * x46;
	const GEN_FLT x49 = x8 + x6 * x9;
	const GEN_FLT x50 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x51 = x50 * x13;
	const GEN_FLT x52 = -x51;
	const GEN_FLT x53 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x54 = x50 * x18;
	const GEN_FLT x55 = x54 * x17;
	const GEN_FLT x56 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x57 = x56 * x13;
	const GEN_FLT x58 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x59 = x58 * x19;
	const GEN_FLT x60 = x53 * x23 + x59 * x16 + x50 * x15 * x16;
	const GEN_FLT x61 = x54 * x14;
	const GEN_FLT x62 = x58 * x13;
	const GEN_FLT x63 = x50 * x22;
	const GEN_FLT x64 = x56 * x19;
	const GEN_FLT x65 = x53 * x20 + x63 * x16 + x64 * x16;
	const GEN_FLT x66 = (x52 + x51 * x25 + 2 * x53 * x19 * x16) * sensor_x + (-x55 - x57 + x60) * sensor_y +
						(x61 + x62 + x65) * sensor_z;
	const GEN_FLT x67 = x1 * x31;
	const GEN_FLT x68 = x36 * x10;
	const GEN_FLT x69 = -x67 + x68;
	const GEN_FLT x70 = x54 * x16;
	const GEN_FLT x71 = x53 * x13;
	const GEN_FLT x72 = x59 * x17 + x63 * x14 + x64 * x14;
	const GEN_FLT x73 =
		(x52 + x51 * x27 + 2 * x59 * x14) * sensor_y + (x55 + x57 + x60) * sensor_x + (-x70 - x71 + x72) * sensor_z;
	const GEN_FLT x74 =
		(x52 + x51 * x42 + 2 * x64 * x17) * sensor_z + (-x61 - x62 + x65) * sensor_x + (x70 + x71 + x72) * sensor_y;
	const GEN_FLT x75 = x1 * x36;
	const GEN_FLT x76 = x31 * x10;
	const GEN_FLT x77 = x75 + x76;
	const GEN_FLT x78 = x66 * x49 + x73 * x69 + x74 * x77;
	const GEN_FLT x79 = x78 + x26 * (x4 + x3 * x6 + x7 * x11) + x30 * (-x33 - x35 + x41) + x43 * (x44 + x45 + x48);
	const GEN_FLT x80 = 1 + x79;
	const GEN_FLT x81 = pow(x31, 2);
	const GEN_FLT x82 = x8 + x9 * x81;
	const GEN_FLT x83 = x1 * x5;
	const GEN_FLT x84 = x46 * x36;
	const GEN_FLT x85 = x83 + x84;
	const GEN_FLT x86 = -x75 + x76;
	const GEN_FLT x87 = lh_pz + x82 * x43 + x85 * x30 + x86 * x26;
	const GEN_FLT x88 = pow(x87, -1);
	const GEN_FLT x89 = x5 * x32;
	const GEN_FLT x90 = x1 * x7;
	const GEN_FLT x91 = x31 * x36;
	const GEN_FLT x92 = x3 * x91 + x31 * x39 + x40 * x34;
	const GEN_FLT x93 = 2 * x46;
	const GEN_FLT x94 = x82 * x74 + x85 * x73 + x86 * x66;
	const GEN_FLT x95 = x94 + x26 * (-x44 - x45 + x48) + x30 * (x89 + x90 + x92) + x43 * (x4 + x3 * x81 + x93 * x34);
	const GEN_FLT x96 = lh_px + x49 * x26 + x69 * x30 + x77 * x43;
	const GEN_FLT x97 = pow(x87, 2);
	const GEN_FLT x98 = pow(x97, -1);
	const GEN_FLT x99 = x98 * x96;
	const GEN_FLT x100 = x99 * x95;
	const GEN_FLT x101 = pow(x96, 2);
	const GEN_FLT x102 = x101 + x97;
	const GEN_FLT x103 = pow(x102, -1);
	const GEN_FLT x104 = x97 * x103;
	const GEN_FLT x105 = x104 * (x100 - x80 * x88);
	const GEN_FLT x106 = -x83 + x84;
	const GEN_FLT x107 = pow(x36, 2);
	const GEN_FLT x108 = x8 + x9 * x107;
	const GEN_FLT x109 = x67 + x68;
	const GEN_FLT x110 = lh_py + x26 * x109 + x30 * x108 + x43 * x106;
	const GEN_FLT x111 = pow(x110, 2);
	const GEN_FLT x112 = pow(1 - x103 * x111 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x113 = 2 * x96;
	const GEN_FLT x114 = 2 * x87;
	const GEN_FLT x115 = x95 * x114;
	const GEN_FLT x116 = (1.0 / 2.0) * x110 * tilt_0 / pow(x102, 3.0 / 2.0);
	const GEN_FLT x117 = 2 * x36;
	const GEN_FLT x118 = x66 * x109 + x73 * x108 + x74 * x106;
	const GEN_FLT x119 =
		x118 + x26 * (x33 + x35 + x41) + x30 * (x4 + x3 * x107 + x39 * x117) + x43 * (-x89 - x90 + x92);
	const GEN_FLT x120 = tilt_0 / sqrt(x102);
	const GEN_FLT x121 = x119 * x120;
	const GEN_FLT x122 = -x105 - x112 * (x121 - x116 * (x115 + x80 * x113));
	const GEN_FLT x123 = -x87;
	const GEN_FLT x124 = atan2(x96, x123);
	const GEN_FLT x125 = sin(1.5707963267949 + gibPhase_0 - phase_0 - x124 - asin(x110 * x120)) * gibMag_0;
	const GEN_FLT x126 = x88 * x119;
	const GEN_FLT x127 = -x126;
	const GEN_FLT x128 = x98 * x110;
	const GEN_FLT x129 = x95 * x128;
	const GEN_FLT x130 = x111 + x97;
	const GEN_FLT x131 = pow(x130, -1);
	const GEN_FLT x132 = x97 * x131;
	const GEN_FLT x133 = 2 * x132 * atan2(x110, x123) * curve_0;
	const GEN_FLT x134 = -x88 * x79;
	const GEN_FLT x135 = (x100 + x134) * x104;
	const GEN_FLT x136 = x79 * x113;
	const GEN_FLT x137 = 1 + x119;
	const GEN_FLT x138 = -x135 - x112 * (x120 * x137 - (x115 + x136) * x116);
	const GEN_FLT x139 = x88 * x137;
	const GEN_FLT x140 = 1 + x95;
	const GEN_FLT x141 = x104 * (x134 + x99 * x140);
	const GEN_FLT x142 = x114 * x140;
	const GEN_FLT x143 = -x141 - x112 * (x121 - (x136 + x142) * x116);
	const GEN_FLT x144 = x128 * x140;
	const GEN_FLT x145 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x146 = x1 * x145;
	const GEN_FLT x147 = -x146;
	const GEN_FLT x148 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x149 = x8 * x145;
	const GEN_FLT x150 = x31 * x149;
	const GEN_FLT x151 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x152 = x1 * x151;
	const GEN_FLT x153 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x154 = x10 * x153 + x37 * x146 + x40 * x148;
	const GEN_FLT x155 = x36 * x149;
	const GEN_FLT x156 = x1 * x153;
	const GEN_FLT x157 = x10 * x151 + x46 * x148 + x47 * x146;
	const GEN_FLT x158 =
		x78 + x26 * (x147 + x11 * x148 + x6 * x146) + x30 * (-x150 - x152 + x154) + x43 * (x155 + x156 + x157);
	const GEN_FLT x159 = x5 * x149;
	const GEN_FLT x160 = x1 * x148;
	const GEN_FLT x161 = x40 * x151 + x46 * x153 + x91 * x146;
	const GEN_FLT x162 =
		x94 + x26 * (-x155 - x156 + x157) + x30 * (x159 + x160 + x161) + x43 * (x147 + x81 * x146 + x93 * x151);
	const GEN_FLT x163 = (-x88 * x158 + x99 * x162) * x104;
	const GEN_FLT x164 = x114 * x162;
	const GEN_FLT x165 = 2 * x40;
	const GEN_FLT x166 =
		x118 + x26 * (x150 + x152 + x154) + x30 * (x147 + x107 * x146 + x165 * x153) + x43 * (-x159 - x160 + x161);
	const GEN_FLT x167 = -x163 - x112 * (-x116 * (x164 + x113 * x158) + x120 * x166);
	const GEN_FLT x168 = x88 * x166;
	const GEN_FLT x169 = x128 * x162;
	const GEN_FLT x170 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x171 = x1 * x170;
	const GEN_FLT x172 = -x171;
	const GEN_FLT x173 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x174 = x9 * x173;
	const GEN_FLT x175 = x8 * x170;
	const GEN_FLT x176 = x31 * x175;
	const GEN_FLT x177 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x178 = x1 * x177;
	const GEN_FLT x179 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x180 = x9 * x179;
	const GEN_FLT x181 = x36 * x174 + x37 * x171 + x5 * x180;
	const GEN_FLT x182 = x36 * x175;
	const GEN_FLT x183 = x1 * x179;
	const GEN_FLT x184 = x9 * x177;
	const GEN_FLT x185 = x31 * x174 + x47 * x171 + x5 * x184;
	const GEN_FLT x186 =
		x78 + x26 * (x172 + 2 * x5 * x174 + x6 * x171) + x30 * (-x176 - x178 + x181) + x43 * (x182 + x183 + x185);
	const GEN_FLT x187 = x5 * x175;
	const GEN_FLT x188 = x1 * x173;
	const GEN_FLT x189 = x31 * x180 + x36 * x184 + x91 * x171;
	const GEN_FLT x190 =
		x94 + x26 * (-x182 - x183 + x185) + x30 * (x187 + x188 + x189) + x43 * (x172 + 2 * x31 * x184 + x81 * x171);
	const GEN_FLT x191 = (-x88 * x186 + x99 * x190) * x104;
	const GEN_FLT x192 = x114 * x190;
	const GEN_FLT x193 =
		x118 + x26 * (x176 + x178 + x181) + x30 * (x172 + x107 * x171 + x117 * x180) + x43 * (-x187 - x188 + x189);
	const GEN_FLT x194 = -x191 - x112 * (-x116 * (x192 + x113 * x186) + x120 * x193);
	const GEN_FLT x195 = x88 * x193;
	const GEN_FLT x196 = x128 * x190;
	const GEN_FLT x197 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x198 = x1 * x197;
	const GEN_FLT x199 = -x198;
	const GEN_FLT x200 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x201 = x8 * x197;
	const GEN_FLT x202 = x31 * x201;
	const GEN_FLT x203 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x204 = x1 * x203;
	const GEN_FLT x205 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x206 = x10 * x205 + x37 * x198 + x40 * x200;
	const GEN_FLT x207 = x36 * x201;
	const GEN_FLT x208 = x1 * x205;
	const GEN_FLT x209 = x10 * x203 + x46 * x200 + x47 * x198;
	const GEN_FLT x210 =
		x78 + x26 * (x199 + x11 * x200 + x6 * x198) + x30 * (-x202 - x204 + x206) + x43 * (x207 + x208 + x209);
	const GEN_FLT x211 = x5 * x201;
	const GEN_FLT x212 = x1 * x200;
	const GEN_FLT x213 = x40 * x203 + x46 * x205 + x91 * x198;
	const GEN_FLT x214 =
		x94 + x26 * (-x207 - x208 + x209) + x30 * (x211 + x212 + x213) + x43 * (x199 + x81 * x198 + x93 * x203);
	const GEN_FLT x215 = (-x88 * x210 + x99 * x214) * x104;
	const GEN_FLT x216 = x214 * x114;
	const GEN_FLT x217 =
		x118 + x26 * (x202 + x204 + x206) + x30 * (x199 + x107 * x198 + x205 * x165) + x43 * (-x211 - x212 + x213);
	const GEN_FLT x218 = -x215 - x112 * (-x116 * (x216 + x210 * x113) + x217 * x120);
	const GEN_FLT x219 = x88 * x217;
	const GEN_FLT x220 = x214 * x128;
	const GEN_FLT x221 = -x129;
	const GEN_FLT x222 = pow(1 - x101 * x131 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x223 = 2 * x110;
	const GEN_FLT x224 = x223 * x119;
	const GEN_FLT x225 = (1.0 / 2.0) * x96 * tilt_1 / pow(x130, 3.0 / 2.0);
	const GEN_FLT x226 = tilt_1 / sqrt(x130);
	const GEN_FLT x227 = -x222 * (x80 * x226 - (x115 + x224) * x225) - (x126 + x221) * x132;
	const GEN_FLT x228 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x96 * x226) - atan2(-x110, x123)) * gibMag_1;
	const GEN_FLT x229 = 2 * x124 * curve_1;
	const GEN_FLT x230 = x79 * x226;
	const GEN_FLT x231 = -x222 * (x230 - x225 * (x115 + x223 * x137)) - (x139 + x221) * x132;
	const GEN_FLT x232 = -x222 * (x230 - (x142 + x224) * x225) - (x126 - x144) * x132;
	const GEN_FLT x233 = -x222 * (-x225 * (x164 + x223 * x166) + x226 * x158) - (x168 - x169) * x132;
	const GEN_FLT x234 = -x222 * (-x225 * (x192 + x223 * x193) + x226 * x186) - (x195 - x196) * x132;
	const GEN_FLT x235 = -x222 * (x210 * x226 - x225 * (x216 + x217 * x223)) - (x219 - x220) * x132;
	out[0] = x122 + x122 * x125 + (x127 + x129) * x133;
	out[1] = x138 + x125 * x138 + (x129 - x139) * x133;
	out[2] = x143 + x125 * x143 + (x127 + x144) * x133;
	out[3] = x167 + x125 * x167 + (-x168 + x169) * x133;
	out[4] = x194 + x125 * x194 + (-x195 + x196) * x133;
	out[5] = x218 + x218 * x125 + (-x219 + x220) * x133;
	out[6] = x227 + x227 * x228 + x229 * x105;
	out[7] = x231 + x229 * x135 + x231 * x228;
	out[8] = x232 + x229 * x141 + x232 * x228;
	out[9] = x233 + x229 * x163 + x233 * x228;
	out[10] = x234 + x229 * x191 + x234 * x228;
	out[11] = x235 + x215 * x229 + x235 * x228;
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
	const GEN_FLT x0 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x1 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1e-10));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = -x3;
	const GEN_FLT x5 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								  : (1e-10)))
							? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											 : (1e-10)))
							: (1));
	const GEN_FLT x6 = pow(x5, 2);
	const GEN_FLT x7 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								  : (1e-10)))
							? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
							   pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										: (1e-10)),
								   2))
							: (0));
	const GEN_FLT x8 = cos(x1);
	const GEN_FLT x9 = 1 - x8;
	const GEN_FLT x10 = x5 * x9;
	const GEN_FLT x11 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x12 = x0 * x8;
	const GEN_FLT x13 = x12 * x11;
	const GEN_FLT x14 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x15 = x2 * x14;
	const GEN_FLT x16 = x2 * x5;
	const GEN_FLT x17 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x18 = x0 * x17;
	const GEN_FLT x19 = x7 * x9;
	const GEN_FLT x20 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x21 = x18 * x16 + x19 * x17 + x20 * x10;
	const GEN_FLT x22 = x12 * x17;
	const GEN_FLT x23 = x2 * x20;
	const GEN_FLT x24 = x11 * x19 + x14 * x10 + x0 * x11 * x16;
	const GEN_FLT x25 =
		(-x13 - x15 + x21) * sensor_y + (x22 + x23 + x24) * sensor_z + (x4 + x3 * x6 + 2 * x7 * x10) * sensor_x;
	const GEN_FLT x26 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x27 = sin(x26);
	const GEN_FLT x28 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x29 = x28 * x27;
	const GEN_FLT x30 = cos(x26);
	const GEN_FLT x31 = 1 - x30;
	const GEN_FLT x32 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x33 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (1));
	const GEN_FLT x34 = x32 * x31 * x33;
	const GEN_FLT x35 = x29 + x34;
	const GEN_FLT x36 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x37 = x36 * x27;
	const GEN_FLT x38 = -x37;
	const GEN_FLT x39 = pow(x32, 2);
	const GEN_FLT x40 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x41 = x40 * x31;
	const GEN_FLT x42 = pow(x17, 2);
	const GEN_FLT x43 = x9 * x11;
	const GEN_FLT x44 = x43 * x17;
	const GEN_FLT x45 = x2 * x11;
	const GEN_FLT x46 = x10 * x17;
	const GEN_FLT x47 = obj_py + (-x16 + x44) * sensor_z + (x45 + x46) * sensor_x + (x8 + x9 * x42) * sensor_y;
	const GEN_FLT x48 = x30 + x31 * x39;
	const GEN_FLT x49 = x5 * x12;
	const GEN_FLT x50 = x2 * x7;
	const GEN_FLT x51 = x9 * x17;
	const GEN_FLT x52 = x43 * x20 + x45 * x18 + x51 * x14;
	const GEN_FLT x53 =
		(x13 + x15 + x21) * sensor_x + (x4 + x3 * x42 + 2 * x51 * x20) * sensor_y + (-x49 - x50 + x52) * sensor_z;
	const GEN_FLT x54 = x30 * x36;
	const GEN_FLT x55 = x54 * x33;
	const GEN_FLT x56 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x57 = x56 * x27;
	const GEN_FLT x58 = x32 * x27;
	const GEN_FLT x59 = x58 * x36;
	const GEN_FLT x60 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x61 = x60 * x31;
	const GEN_FLT x62 = x41 * x28 + x59 * x28 + x61 * x32;
	const GEN_FLT x63 = pow(x11, 2);
	const GEN_FLT x64 = x2 * x17;
	const GEN_FLT x65 = x11 * x10;
	const GEN_FLT x66 = obj_pz + (x16 + x44) * sensor_y + (-x64 + x65) * sensor_x + (x8 + x9 * x63) * sensor_z;
	const GEN_FLT x67 = obj_px + (-x45 + x46) * sensor_y + (x64 + x65) * sensor_z + (x8 + x6 * x9) * sensor_x;
	const GEN_FLT x68 = x54 * x28;
	const GEN_FLT x69 = x60 * x27;
	const GEN_FLT x70 = x56 * x31;
	const GEN_FLT x71 = x41 * x33 + x59 * x33 + x70 * x32;
	const GEN_FLT x72 =
		(-x22 - x23 + x24) * sensor_x + (x4 + x3 * x63 + 2 * x43 * x14) * sensor_z + (x49 + x50 + x52) * sensor_y;
	const GEN_FLT x73 = x33 * x27;
	const GEN_FLT x74 = x31 * x28;
	const GEN_FLT x75 = x74 * x32;
	const GEN_FLT x76 = -x73 + x75;
	const GEN_FLT x77 = x35 * x25 + x47 * (x38 + x37 * x39 + 2 * x41 * x32) + x53 * x48 + x66 * (-x55 - x57 + x62) +
						x67 * (x68 + x69 + x71) + x72 * x76;
	const GEN_FLT x78 = pow(x28, 2);
	const GEN_FLT x79 = x30 + x78 * x31;
	const GEN_FLT x80 = x73 + x75;
	const GEN_FLT x81 = x74 * x33;
	const GEN_FLT x82 = -x58 + x81;
	const GEN_FLT x83 = lh_pz + x79 * x66 + x80 * x47 + x82 * x67;
	const GEN_FLT x84 = pow(x83, -1);
	const GEN_FLT x85 = x84 * x77;
	const GEN_FLT x86 = lh_py + x47 * x48 + x67 * x35 + x76 * x66;
	const GEN_FLT x87 = x54 * x32;
	const GEN_FLT x88 = x40 * x27;
	const GEN_FLT x89 = x61 * x33 + x70 * x28 + x73 * x36 * x28;
	const GEN_FLT x90 = x47 * (x55 + x57 + x62) + x66 * (x38 + 2 * x61 * x28 + x78 * x37) + x67 * (-x87 - x88 + x89) +
						x72 * x79 + x80 * x53 + x82 * x25;
	const GEN_FLT x91 = pow(x83, 2);
	const GEN_FLT x92 = x90 / x91;
	const GEN_FLT x93 = x86 * x92;
	const GEN_FLT x94 = -x83;
	const GEN_FLT x95 = atan2(x86, x94);
	const GEN_FLT x96 = pow(x86, 2);
	const GEN_FLT x97 = x91 + x96;
	const GEN_FLT x98 = pow(x97, -1);
	const GEN_FLT x99 = x91 * x98;
	const GEN_FLT x100 = 2 * (-x85 + x93) * x99 * x95 * curve_0;
	const GEN_FLT x101 = x58 + x81;
	const GEN_FLT x102 = -x29 + x34;
	const GEN_FLT x103 = pow(x33, 2);
	const GEN_FLT x104 = x30 + x31 * x103;
	const GEN_FLT x105 = lh_px + x47 * x102 + x66 * x101 + x67 * x104;
	const GEN_FLT x106 = pow(x105, 2);
	const GEN_FLT x107 = x106 + x91;
	const GEN_FLT x108 = pow(x107, -1);
	const GEN_FLT x109 = x25 * x104 + x47 * (-x68 - x69 + x71) + x53 * x102 + x66 * (x87 + x88 + x89) +
						 x67 * (x38 + x37 * x103 + 2 * x70 * x33) + x72 * x101;
	const GEN_FLT x110 = (-x84 * x109 + x92 * x105) * x91 * x108;
	const GEN_FLT x111 = -x110;
	const GEN_FLT x112 = pow(1 - x96 * x108 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x113 = 2 * x83 * x90;
	const GEN_FLT x114 = pow(x107, -1.0 / 2.0);
	const GEN_FLT x115 =
		x77 * x114 * tilt_0 + (-1.0 / 2.0) * x86 * (x113 + 2 * x109 * x105) * tilt_0 / pow(x107, 3.0 / 2.0);
	const GEN_FLT x116 = x111 - x112 * x115;
	const GEN_FLT x117 = -1 + x116;
	const GEN_FLT x118 = x86 * x114;
	const GEN_FLT x119 = atan2(x105, x94);
	const GEN_FLT x120 = 1.5707963267949 + gibPhase_0 - phase_0 - x119 - asin(x118 * tilt_0);
	const GEN_FLT x121 = sin(x120) * gibMag_0;
	const GEN_FLT x122 = x111 - (x115 + x118) * x112;
	const GEN_FLT x123 = x100 + x116;
	const GEN_FLT x124 = x123 + x116 * x121;
	const GEN_FLT x125 = -(x85 - x93) * x99;
	const GEN_FLT x126 = pow(1 - x98 * x106 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x127 = pow(x97, -1.0 / 2.0);
	const GEN_FLT x128 =
		x109 * x127 * tilt_1 + (-1.0 / 2.0) * x105 * tilt_1 * (x113 + 2 * x86 * x77) / pow(x97, 3.0 / 2.0);
	const GEN_FLT x129 = x125 - x128 * x126;
	const GEN_FLT x130 = x105 * x127;
	const GEN_FLT x131 = 1.5707963267949 + gibPhase_1 - phase_1 - asin(x130 * tilt_1) - atan2(-x86, x94);
	const GEN_FLT x132 = sin(x131) * gibMag_1;
	const GEN_FLT x133 = 2 * x110 * x119 * curve_1;
	const GEN_FLT x134 = x129 + x133;
	const GEN_FLT x135 = x134 + x129 * x132;
	const GEN_FLT x136 = -1 + x129;
	const GEN_FLT x137 = x125 - (x128 + x130) * x126;
	out[0] = x100 + x117 + x117 * x121;
	out[1] = x100 + x122 + x122 * x121;
	out[2] = x124 + pow(x95, 2);
	out[3] = x123 + x121 * (1 + x116);
	out[4] = x124 - cos(x120);
	out[5] = x124;
	out[6] = x124;
	out[7] = x124;
	out[8] = x124;
	out[9] = x124;
	out[10] = x124;
	out[11] = x124;
	out[12] = x124;
	out[13] = x124;
	out[14] = x135;
	out[15] = x135;
	out[16] = x135;
	out[17] = x135;
	out[18] = x135;
	out[19] = x135;
	out[20] = x135;
	out[21] = x133 + x136 + x132 * x136;
	out[22] = x133 + x137 + x132 * x137;
	out[23] = x135 + pow(x119, 2);
	out[24] = x134 + x132 * (1 + x129);
	out[25] = x135 - cos(x131);
	out[26] = x135;
	out[27] = x135;
}

/** Applying function <function reproject_axis_x at 0x7effc26e33b0> */
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x5 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 - x6;
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1e-10));
	const GEN_FLT x11 = cos(x10);
	const GEN_FLT x12 = 1 - x11;
	const GEN_FLT x13 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x14 = sin(x10);
	const GEN_FLT x15 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x16 = x15 * x14;
	const GEN_FLT x17 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x18 = x13 * x12;
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = x14 * x17;
	const GEN_FLT x21 = x15 * x18;
	const GEN_FLT x22 =
		obj_pz + (x11 + pow(x13, 2) * x12) * sensor_z + (x16 + x19) * sensor_y + (-x20 + x21) * sensor_x;
	const GEN_FLT x23 = x14 * x13;
	const GEN_FLT x24 = x15 * x12 * x17;
	const GEN_FLT x25 =
		obj_py + (x11 + x12 * pow(x17, 2)) * sensor_y + (-x16 + x19) * sensor_z + (x23 + x24) * sensor_x;
	const GEN_FLT x26 =
		obj_px + (x11 + pow(x15, 2) * x12) * sensor_x + (x20 + x21) * sensor_z + (-x23 + x24) * sensor_y;
	const GEN_FLT x27 = x1 * x4;
	const GEN_FLT x28 = x2 * x8;
	const GEN_FLT x29 = lh_py + x25 * (x6 + pow(x5, 2) * x7) + (x27 + x28) * x26 + (-x3 + x9) * x22;
	const GEN_FLT x30 = x1 * x5;
	const GEN_FLT x31 = x2 * x4 * x7;
	const GEN_FLT x32 = lh_pz + x22 * (x6 + pow(x4, 2) * x7) + (x3 + x9) * x25 + (-x30 + x31) * x26;
	const GEN_FLT x33 = -x32;
	const GEN_FLT x34 = lh_px + x26 * (x6 + pow(x2, 2) * x7) + (-x27 + x28) * x25 + (x30 + x31) * x22;
	const GEN_FLT x35 = -phase_0 - asin(x29 * tilt_0 / sqrt(pow(x32, 2) + pow(x34, 2))) - atan2(x34, x33);
	out[0] = x35 - cos(1.5707963267949 + gibPhase_0 + x35) * gibMag_0 + pow(atan2(x29, x33), 2) * curve_0;
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 - x1;
	const GEN_FLT x3 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x4 = pow(x3, 2);
	const GEN_FLT x5 = x1 + x2 * x4;
	const GEN_FLT x6 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x7 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1e-10));
	const GEN_FLT x8 = sin(x7);
	const GEN_FLT x9 = x6 * x8;
	const GEN_FLT x10 = -x9;
	const GEN_FLT x11 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x12 = pow(x11, 2);
	const GEN_FLT x13 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x14 = cos(x7);
	const GEN_FLT x15 = 1 - x14;
	const GEN_FLT x16 = x15 * x11;
	const GEN_FLT x17 = 2 * x16;
	const GEN_FLT x18 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x19 = x6 * x14;
	const GEN_FLT x20 = x19 * x18;
	const GEN_FLT x21 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x22 = x8 * x21;
	const GEN_FLT x23 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x24 = x8 * x11;
	const GEN_FLT x25 = x24 * x23;
	const GEN_FLT x26 = x23 * x15;
	const GEN_FLT x27 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x28 = x26 * x13 + x27 * x16 + x6 * x25;
	const GEN_FLT x29 = x23 * x19;
	const GEN_FLT x30 = x8 * x27;
	const GEN_FLT x31 = x24 * x18;
	const GEN_FLT x32 = x15 * x18;
	const GEN_FLT x33 = x21 * x16 + x32 * x13 + x6 * x31;
	const GEN_FLT x34 =
		(x10 + x13 * x17 + x9 * x12) * sensor_x + (-x20 - x22 + x28) * sensor_y + (x29 + x30 + x33) * sensor_z;
	const GEN_FLT x35 = 1 + x34;
	const GEN_FLT x36 = sin(x0);
	const GEN_FLT x37 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x38 = x36 * x37;
	const GEN_FLT x39 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x40 = x2 * x39;
	const GEN_FLT x41 = x3 * x40;
	const GEN_FLT x42 = -x38 + x41;
	const GEN_FLT x43 = x11 * x19;
	const GEN_FLT x44 = x8 * x13;
	const GEN_FLT x45 = x8 * x18;
	const GEN_FLT x46 = x45 * x23;
	const GEN_FLT x47 = x21 * x26 + x32 * x27 + x6 * x46;
	const GEN_FLT x48 = pow(x23, 2);
	const GEN_FLT x49 = 2 * x26;
	const GEN_FLT x50 =
		(x10 + x49 * x27 + x9 * x48) * sensor_y + (x20 + x22 + x28) * sensor_x + (-x43 - x44 + x47) * sensor_z;
	const GEN_FLT x51 = x50 * x42;
	const GEN_FLT x52 = pow(x18, 2);
	const GEN_FLT x53 = 2 * x32;
	const GEN_FLT x54 =
		(x10 + x53 * x21 + x9 * x52) * sensor_z + (-x29 - x30 + x33) * sensor_x + (x43 + x44 + x47) * sensor_y;
	const GEN_FLT x55 = x36 * x39;
	const GEN_FLT x56 = x2 * x37;
	const GEN_FLT x57 = x3 * x56;
	const GEN_FLT x58 = x55 + x57;
	const GEN_FLT x59 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x60 = x59 * x36;
	const GEN_FLT x61 = -x60;
	const GEN_FLT x62 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x63 = x8 * x23;
	const GEN_FLT x64 = x18 * x16;
	const GEN_FLT x65 = x23 * x16;
	const GEN_FLT x66 = obj_px + (x14 + x15 * x12) * sensor_x + (x63 + x64) * sensor_z + (-x45 + x65) * sensor_y;
	const GEN_FLT x67 = x32 * x23;
	const GEN_FLT x68 = obj_py + (x14 + x48 * x15) * sensor_y + (x45 + x65) * sensor_x + (-x24 + x67) * sensor_z;
	const GEN_FLT x69 = x1 * x59;
	const GEN_FLT x70 = x69 * x37;
	const GEN_FLT x71 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x72 = x71 * x36;
	const GEN_FLT x73 = x3 * x60;
	const GEN_FLT x74 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x75 = x2 * x74;
	const GEN_FLT x76 = x3 * x75 + x62 * x40 + x73 * x39;
	const GEN_FLT x77 = obj_pz + (x14 + x52 * x15) * sensor_z + (-x63 + x64) * sensor_x + (x24 + x67) * sensor_y;
	const GEN_FLT x78 = x69 * x39;
	const GEN_FLT x79 = x74 * x36;
	const GEN_FLT x80 = x2 * x71;
	const GEN_FLT x81 = x3 * x80 + x62 * x56 + x73 * x37;
	const GEN_FLT x82 = x66 * (x61 + x4 * x60 + 2 * x2 * x3 * x62) + x68 * (-x70 - x72 + x76) + x77 * (x78 + x79 + x81);
	const GEN_FLT x83 = x82 + x54 * x58;
	const GEN_FLT x84 = x51 + x83 + x5 * x35;
	const GEN_FLT x85 = pow(x37, 2);
	const GEN_FLT x86 = x1 + x2 * x85;
	const GEN_FLT x87 = x3 * x36;
	const GEN_FLT x88 = x40 * x37;
	const GEN_FLT x89 = x87 + x88;
	const GEN_FLT x90 = -x55 + x57;
	const GEN_FLT x91 = lh_pz + x66 * x90 + x86 * x77 + x89 * x68;
	const GEN_FLT x92 = pow(x91, -1);
	const GEN_FLT x93 = x86 * x54;
	const GEN_FLT x94 = x3 * x69;
	const GEN_FLT x95 = x62 * x36;
	const GEN_FLT x96 = x75 * x37 + x80 * x39 + x60 * x37 * x39;
	const GEN_FLT x97 = x66 * (-x78 - x79 + x81) + x68 * (x94 + x95 + x96) + x77 * (x61 + 2 * x80 * x37 + x85 * x60);
	const GEN_FLT x98 = x97 + x89 * x50;
	const GEN_FLT x99 = x93 + x98 + x90 * x35;
	const GEN_FLT x100 = lh_px + x5 * x66 + x68 * x42 + x77 * x58;
	const GEN_FLT x101 = pow(x91, 2);
	const GEN_FLT x102 = pow(x101, -1);
	const GEN_FLT x103 = x100 * x102;
	const GEN_FLT x104 = x101 + pow(x100, 2);
	const GEN_FLT x105 = pow(x104, -1);
	const GEN_FLT x106 = x101 * x105;
	const GEN_FLT x107 = -x87 + x88;
	const GEN_FLT x108 = pow(x39, 2);
	const GEN_FLT x109 = x1 + x2 * x108;
	const GEN_FLT x110 = x38 + x41;
	const GEN_FLT x111 = lh_py + x66 * x110 + x68 * x109 + x77 * x107;
	const GEN_FLT x112 = pow(x111, 2);
	const GEN_FLT x113 = pow(1 - x105 * x112 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x114 = 2 * x100;
	const GEN_FLT x115 = 2 * x91;
	const GEN_FLT x116 = (1.0 / 2.0) * x111 * tilt_0 / pow(x104, 3.0 / 2.0);
	const GEN_FLT x117 = x50 * x109;
	const GEN_FLT x118 = x54 * x107;
	const GEN_FLT x119 = x66 * (x70 + x72 + x76) + x68 * (x61 + x60 * x108 + 2 * x75 * x39) + x77 * (-x94 - x95 + x96);
	const GEN_FLT x120 = x117 + x118 + x119 + x35 * x110;
	const GEN_FLT x121 = tilt_0 / sqrt(x104);
	const GEN_FLT x122 = -x106 * (-x84 * x92 + x99 * x103) - x113 * (x120 * x121 - (x84 * x114 + x99 * x115) * x116);
	const GEN_FLT x123 = -x91;
	const GEN_FLT x124 = sin(1.5707963267949 + gibPhase_0 - phase_0 - asin(x111 * x121) - atan2(x100, x123)) * gibMag_0;
	const GEN_FLT x125 = x102 * x111;
	const GEN_FLT x126 = 2 * x101 * atan2(x111, x123) * curve_0 / (x101 + x112);
	const GEN_FLT x127 = x5 * x34;
	const GEN_FLT x128 = 1 + x50;
	const GEN_FLT x129 = x127 + x83 + x42 * x128;
	const GEN_FLT x130 = x90 * x34;
	const GEN_FLT x131 = x130 + x93 + x97 + x89 * x128;
	const GEN_FLT x132 = x119 + x34 * x110;
	const GEN_FLT x133 = x118 + x132 + x109 * x128;
	const GEN_FLT x134 = -x106 * (x103 * x131 - x92 * x129) - x113 * (x121 * x133 - (x114 * x129 + x115 * x131) * x116);
	const GEN_FLT x135 = 1 + x54;
	const GEN_FLT x136 = x127 + x51 + x82 + x58 * x135;
	const GEN_FLT x137 = x130 + x98 + x86 * x135;
	const GEN_FLT x138 = x117 + x132 + x107 * x135;
	const GEN_FLT x139 = -x106 * (x103 * x137 - x92 * x136) - x113 * (x121 * x138 - (x114 * x136 + x115 * x137) * x116);
	const GEN_FLT x140 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x141 = x8 * x140;
	const GEN_FLT x142 = -x141;
	const GEN_FLT x143 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x144 = x14 * x140;
	const GEN_FLT x145 = x18 * x144;
	const GEN_FLT x146 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x147 = x8 * x146;
	const GEN_FLT x148 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x149 = x16 * x148 + x25 * x140 + x26 * x143;
	const GEN_FLT x150 = x23 * x144;
	const GEN_FLT x151 = x8 * x148;
	const GEN_FLT x152 = x16 * x146 + x31 * x140 + x32 * x143;
	const GEN_FLT x153 = (x142 + x12 * x141 + x17 * x143) * sensor_x + (-x145 - x147 + x149) * sensor_y +
						 (x150 + x151 + x152) * sensor_z;
	const GEN_FLT x154 = x11 * x144;
	const GEN_FLT x155 = x8 * x143;
	const GEN_FLT x156 = x26 * x146 + x32 * x148 + x46 * x140;
	const GEN_FLT x157 = (x142 + x48 * x141 + x49 * x148) * sensor_y + (x145 + x147 + x149) * sensor_x +
						 (-x154 - x155 + x156) * sensor_z;
	const GEN_FLT x158 = (x142 + x52 * x141 + x53 * x146) * sensor_z + (-x150 - x151 + x152) * sensor_x +
						 (x154 + x155 + x156) * sensor_y;
	const GEN_FLT x159 = x82 + x42 * x157 + x5 * x153 + x58 * x158;
	const GEN_FLT x160 = x97 + x86 * x158 + x89 * x157 + x90 * x153;
	const GEN_FLT x161 = x119 + x107 * x158 + x109 * x157 + x110 * x153;
	const GEN_FLT x162 = -x106 * (x103 * x160 - x92 * x159) - x113 * (x121 * x161 - (x114 * x159 + x115 * x160) * x116);
	const GEN_FLT x163 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x164 = x8 * x163;
	const GEN_FLT x165 = -x164;
	const GEN_FLT x166 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x167 = x14 * x163;
	const GEN_FLT x168 = x18 * x167;
	const GEN_FLT x169 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x170 = x8 * x169;
	const GEN_FLT x171 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x172 = x16 * x171 + x25 * x163 + x26 * x166;
	const GEN_FLT x173 = x23 * x167;
	const GEN_FLT x174 = x8 * x171;
	const GEN_FLT x175 = x16 * x169 + x31 * x163 + x32 * x166;
	const GEN_FLT x176 = (x165 + x12 * x164 + x17 * x166) * sensor_x + (-x168 - x170 + x172) * sensor_y +
						 (x173 + x174 + x175) * sensor_z;
	const GEN_FLT x177 = x11 * x167;
	const GEN_FLT x178 = x8 * x166;
	const GEN_FLT x179 = x26 * x169 + x32 * x171 + x46 * x163;
	const GEN_FLT x180 = (x165 + x48 * x164 + x49 * x171) * sensor_y + (x168 + x170 + x172) * sensor_x +
						 (-x177 - x178 + x179) * sensor_z;
	const GEN_FLT x181 = (x165 + x52 * x164 + x53 * x169) * sensor_z + (-x173 - x174 + x175) * sensor_x +
						 (x177 + x178 + x179) * sensor_y;
	const GEN_FLT x182 = x82 + x42 * x180 + x5 * x176 + x58 * x181;
	const GEN_FLT x183 = x97 + x86 * x181 + x89 * x180 + x90 * x176;
	const GEN_FLT x184 = x102 * x183;
	const GEN_FLT x185 = x119 + x107 * x181 + x109 * x180 + x110 * x176;
	const GEN_FLT x186 = -x106 * (x100 * x184 - x92 * x182) - x113 * (x121 * x185 - (x114 * x182 + x115 * x183) * x116);
	const GEN_FLT x187 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x188 = x14 * x187;
	const GEN_FLT x189 = x18 * x188;
	const GEN_FLT x190 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x191 = x8 * x190;
	const GEN_FLT x192 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x193 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x194 = x16 * x193 + x25 * x187 + x26 * x192;
	const GEN_FLT x195 = x11 * x188;
	const GEN_FLT x196 = x8 * x192;
	const GEN_FLT x197 = x26 * x190 + x32 * x193 + x46 * x187;
	const GEN_FLT x198 = x8 * x187;
	const GEN_FLT x199 = -x198;
	const GEN_FLT x200 = (x189 + x191 + x194) * sensor_x + (-x195 - x196 + x197) * sensor_z +
						 (x199 + x48 * x198 + x49 * x193) * sensor_y;
	const GEN_FLT x201 = x23 * x188;
	const GEN_FLT x202 = x8 * x193;
	const GEN_FLT x203 = x16 * x190 + x31 * x187 + x32 * x192;
	const GEN_FLT x204 = (-x189 - x191 + x194) * sensor_y + (x199 + x12 * x198 + x17 * x192) * sensor_x +
						 (x201 + x202 + x203) * sensor_z;
	const GEN_FLT x205 = (x195 + x196 + x197) * sensor_y + (x199 + x52 * x198 + x53 * x190) * sensor_z +
						 (-x201 - x202 + x203) * sensor_x;
	const GEN_FLT x206 = x82 + x42 * x200 + x5 * x204 + x58 * x205;
	const GEN_FLT x207 = x97 + x86 * x205 + x89 * x200 + x90 * x204;
	const GEN_FLT x208 = x207 * x102;
	const GEN_FLT x209 = x119 + x200 * x109 + x204 * x110 + x205 * x107;
	const GEN_FLT x210 = -x106 * (x208 * x100 - x92 * x206) - x113 * (x209 * x121 - (x206 * x114 + x207 * x115) * x116);
	out[0] = x122 + x124 * x122 + (-x92 * x120 + x99 * x125) * x126;
	out[1] = x134 + x124 * x134 + x126 * (x125 * x131 - x92 * x133);
	out[2] = x139 + x124 * x139 + x126 * (x125 * x137 - x92 * x138);
	out[3] = x162 + x124 * x162 + x126 * (x125 * x160 - x92 * x161);
	out[4] = x186 + x124 * x186 + x126 * (x111 * x184 - x92 * x185);
	out[5] = x210 + x126 * (x208 * x111 - x92 * x209) + x210 * x124;
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 - x1;
	const GEN_FLT x3 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x4 = pow(x3, 2);
	const GEN_FLT x5 = x1 + x2 * x4;
	const GEN_FLT x6 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1e-10));
	const GEN_FLT x7 = cos(x6);
	const GEN_FLT x8 = 1 - x7;
	const GEN_FLT x9 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								  : (1e-10)))
							? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											 : (1e-10)))
							: (0));
	const GEN_FLT x10 = pow(x9, 2);
	const GEN_FLT x11 = x7 + x8 * x10;
	const GEN_FLT x12 = sin(x6);
	const GEN_FLT x13 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x14 = x13 * x12;
	const GEN_FLT x15 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x16 = x8 * x9;
	const GEN_FLT x17 = x15 * x16;
	const GEN_FLT x18 = x14 + x17;
	const GEN_FLT x19 = x15 * x12;
	const GEN_FLT x20 = x8 * x13;
	const GEN_FLT x21 = x9 * x20;
	const GEN_FLT x22 = -x19 + x21;
	const GEN_FLT x23 = obj_pz + x11 * sensor_z + x18 * sensor_y + x22 * sensor_x;
	const GEN_FLT x24 = sin(x0);
	const GEN_FLT x25 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (1));
	const GEN_FLT x26 = x24 * x25;
	const GEN_FLT x27 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x28 = x2 * x3;
	const GEN_FLT x29 = x28 * x27;
	const GEN_FLT x30 = x26 + x29;
	const GEN_FLT x31 = pow(x15, 2);
	const GEN_FLT x32 = x7 + x8 * x31;
	const GEN_FLT x33 = -x14 + x17;
	const GEN_FLT x34 = x9 * x12;
	const GEN_FLT x35 = x20 * x15;
	const GEN_FLT x36 = x34 + x35;
	const GEN_FLT x37 = obj_py + x32 * sensor_y + x33 * sensor_z + x36 * sensor_x;
	const GEN_FLT x38 = x24 * x27;
	const GEN_FLT x39 = x25 * x28;
	const GEN_FLT x40 = -x38 + x39;
	const GEN_FLT x41 = x19 + x21;
	const GEN_FLT x42 = -x34 + x35;
	const GEN_FLT x43 = pow(x13, 2);
	const GEN_FLT x44 = x7 + x8 * x43;
	const GEN_FLT x45 = obj_px + x41 * sensor_z + x42 * sensor_y + x44 * sensor_x;
	const GEN_FLT x46 = lh_pz + x30 * x37 + x40 * x45 + x5 * x23;
	const GEN_FLT x47 = pow(x46, -1);
	const GEN_FLT x48 = pow(x25, 2);
	const GEN_FLT x49 = x1 + x2 * x48;
	const GEN_FLT x50 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x51 = x50 * x12;
	const GEN_FLT x52 = -x51;
	const GEN_FLT x53 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x54 = x7 * x50;
	const GEN_FLT x55 = x9 * x54;
	const GEN_FLT x56 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x57 = x56 * x12;
	const GEN_FLT x58 = x50 * x14;
	const GEN_FLT x59 = x8 * x15;
	const GEN_FLT x60 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x61 = x53 * x59 + x58 * x15 + x60 * x20;
	const GEN_FLT x62 = x54 * x15;
	const GEN_FLT x63 = x60 * x12;
	const GEN_FLT x64 = x53 * x16 + x56 * x20 + x9 * x58;
	const GEN_FLT x65 =
		(x52 + x51 * x43 + 2 * x53 * x20) * sensor_x + (-x55 - x57 + x61) * sensor_y + (x62 + x63 + x64) * sensor_z;
	const GEN_FLT x66 = x44 + x65;
	const GEN_FLT x67 = x3 * x24;
	const GEN_FLT x68 = x2 * x25;
	const GEN_FLT x69 = x68 * x27;
	const GEN_FLT x70 = -x67 + x69;
	const GEN_FLT x71 = x54 * x13;
	const GEN_FLT x72 = x53 * x12;
	const GEN_FLT x73 = x56 * x59 + x60 * x16 + x50 * x34 * x15;
	const GEN_FLT x74 =
		(x52 + x51 * x31 + 2 * x60 * x59) * sensor_y + (x55 + x57 + x61) * sensor_x + (-x71 - x72 + x73) * sensor_z;
	const GEN_FLT x75 = x36 + x74;
	const GEN_FLT x76 =
		(x52 + x51 * x10 + 2 * x56 * x16) * sensor_z + (-x62 - x63 + x64) * sensor_x + (x71 + x72 + x73) * sensor_y;
	const GEN_FLT x77 = x22 + x76;
	const GEN_FLT x78 = x38 + x39;
	const GEN_FLT x79 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x80 = x79 * x24;
	const GEN_FLT x81 = -x80;
	const GEN_FLT x82 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x83 = x1 * x79;
	const GEN_FLT x84 = x3 * x83;
	const GEN_FLT x85 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x86 = x85 * x24;
	const GEN_FLT x87 = x80 * x25;
	const GEN_FLT x88 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x89 = x2 * x88;
	const GEN_FLT x90 = x2 * x27;
	const GEN_FLT x91 = x82 * x90 + x87 * x27 + x89 * x25;
	const GEN_FLT x92 = x83 * x27;
	const GEN_FLT x93 = x88 * x24;
	const GEN_FLT x94 = x3 * x87 + x82 * x28 + x85 * x68;
	const GEN_FLT x95 = x23 * (x92 + x93 + x94) + x37 * (-x84 - x86 + x91) + x45 * (x81 + x80 * x48 + 2 * x82 * x68);
	const GEN_FLT x96 = x95 + x66 * x49 + x70 * x75 + x78 * x77;
	const GEN_FLT x97 = x83 * x25;
	const GEN_FLT x98 = x82 * x24;
	const GEN_FLT x99 = x3 * x89 + x85 * x90 + x3 * x80 * x27;
	const GEN_FLT x100 = x23 * (x81 + x4 * x80 + 2 * x85 * x28) + x37 * (x97 + x98 + x99) + x45 * (-x92 - x93 + x94);
	const GEN_FLT x101 = x100 + x5 * x77 + x66 * x40 + x75 * x30;
	const GEN_FLT x102 = pow(x46, 2);
	const GEN_FLT x103 = pow(x102, -1);
	const GEN_FLT x104 = lh_px + x45 * x49 + x70 * x37 + x78 * x23;
	const GEN_FLT x105 = x103 * x104;
	const GEN_FLT x106 = x102 + pow(x104, 2);
	const GEN_FLT x107 = pow(x106, -1);
	const GEN_FLT x108 = x102 * x107;
	const GEN_FLT x109 = -x26 + x29;
	const GEN_FLT x110 = pow(x27, 2);
	const GEN_FLT x111 = x1 + x2 * x110;
	const GEN_FLT x112 = x67 + x69;
	const GEN_FLT x113 = lh_py + x23 * x109 + x37 * x111 + x45 * x112;
	const GEN_FLT x114 = pow(x113, 2);
	const GEN_FLT x115 = pow(1 - x107 * x114 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x116 = 2 * x104;
	const GEN_FLT x117 = 2 * x46;
	const GEN_FLT x118 = (1.0 / 2.0) * x113 * tilt_0 / pow(x106, 3.0 / 2.0);
	const GEN_FLT x119 = x23 * (-x97 - x98 + x99) + x37 * (x81 + x80 * x110 + 2 * x89 * x27) + x45 * (x84 + x86 + x91);
	const GEN_FLT x120 = x119 + x66 * x112 + x75 * x111 + x77 * x109;
	const GEN_FLT x121 = tilt_0 / sqrt(x106);
	const GEN_FLT x122 = -x108 * (x101 * x105 - x96 * x47) - x115 * (-x118 * (x101 * x117 + x96 * x116) + x120 * x121);
	const GEN_FLT x123 = -x46;
	const GEN_FLT x124 = sin(1.5707963267949 + gibPhase_0 - phase_0 - asin(x113 * x121) - atan2(x104, x123)) * gibMag_0;
	const GEN_FLT x125 = x103 * x113;
	const GEN_FLT x126 = 2 * x102 * atan2(x113, x123) * curve_0 / (x102 + x114);
	const GEN_FLT x127 = x32 + x74;
	const GEN_FLT x128 = x42 + x65;
	const GEN_FLT x129 = x18 + x76;
	const GEN_FLT x130 = x95 + x49 * x128 + x70 * x127 + x78 * x129;
	const GEN_FLT x131 = x100 + x30 * x127 + x40 * x128 + x5 * x129;
	const GEN_FLT x132 = x119 + x109 * x129 + x111 * x127 + x112 * x128;
	const GEN_FLT x133 = -x108 * (x105 * x131 - x47 * x130) - x115 * (x121 * x132 - (x116 * x130 + x117 * x131) * x118);
	const GEN_FLT x134 = x41 + x65;
	const GEN_FLT x135 = x33 + x74;
	const GEN_FLT x136 = x11 + x76;
	const GEN_FLT x137 = x95 + x49 * x134 + x70 * x135 + x78 * x136;
	const GEN_FLT x138 = x100 + x30 * x135 + x40 * x134 + x5 * x136;
	const GEN_FLT x139 = x119 + x109 * x136 + x111 * x135 + x112 * x134;
	const GEN_FLT x140 = -x108 * (x105 * x138 - x47 * x137) - x115 * (x121 * x139 - (x116 * x137 + x117 * x138) * x118);
	out[0] = x122 + x124 * x122 + x126 * (x101 * x125 - x47 * x120);
	out[1] = x133 + x124 * x133 + x126 * (x125 * x131 - x47 * x132);
	out[2] = x140 + x124 * x140 + x126 * (x125 * x138 - x47 * x139);
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = -x3;
	const GEN_FLT x5 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x6 = pow(x5, 2);
	const GEN_FLT x7 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
							   pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										: (1e-10)),
								   2))
							: (0));
	const GEN_FLT x8 = cos(x0);
	const GEN_FLT x9 = 1 - x8;
	const GEN_FLT x10 = x5 * x9;
	const GEN_FLT x11 = 2 * x10;
	const GEN_FLT x12 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1e-10));
	const GEN_FLT x13 = sin(x12);
	const GEN_FLT x14 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x15 = x14 * x13;
	const GEN_FLT x16 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x17 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x18 = cos(x12);
	const GEN_FLT x19 = 1 - x18;
	const GEN_FLT x20 = x19 * x17;
	const GEN_FLT x21 = x20 * x16;
	const GEN_FLT x22 = x13 * x16;
	const GEN_FLT x23 = x20 * x14;
	const GEN_FLT x24 = pow(x17, 2);
	const GEN_FLT x25 = obj_px + (x18 + x24 * x19) * sensor_x + (x15 + x21) * sensor_z + (-x22 + x23) * sensor_y;
	const GEN_FLT x26 = pow(x14, 2);
	const GEN_FLT x27 = x13 * x17;
	const GEN_FLT x28 = x19 * x16;
	const GEN_FLT x29 = x28 * x14;
	const GEN_FLT x30 = obj_py + (x18 + x26 * x19) * sensor_y + (x22 + x23) * sensor_x + (-x27 + x29) * sensor_z;
	const GEN_FLT x31 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x32 = x2 * x8;
	const GEN_FLT x33 = x32 * x31;
	const GEN_FLT x34 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x35 = x1 * x34;
	const GEN_FLT x36 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x37 = x3 * x5;
	const GEN_FLT x38 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x39 = x9 * x38;
	const GEN_FLT x40 = x9 * x36;
	const GEN_FLT x41 = x36 * x37 + x5 * x39 + x7 * x40;
	const GEN_FLT x42 = pow(x16, 2);
	const GEN_FLT x43 = obj_pz + (x18 + x42 * x19) * sensor_z + (-x15 + x21) * sensor_x + (x27 + x29) * sensor_y;
	const GEN_FLT x44 = x32 * x36;
	const GEN_FLT x45 = x1 * x38;
	const GEN_FLT x46 = x9 * x31;
	const GEN_FLT x47 = x31 * x37 + x34 * x10 + x7 * x46;
	const GEN_FLT x48 = x8 + x6 * x9;
	const GEN_FLT x49 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x50 = x49 * x13;
	const GEN_FLT x51 = -x50;
	const GEN_FLT x52 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x53 = x49 * x18;
	const GEN_FLT x54 = x53 * x16;
	const GEN_FLT x55 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x56 = x55 * x13;
	const GEN_FLT x57 = x49 * x15;
	const GEN_FLT x58 = x14 * x19;
	const GEN_FLT x59 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x60 = x59 * x19;
	const GEN_FLT x61 = x52 * x58 + x57 * x17 + x60 * x17;
	const GEN_FLT x62 = x53 * x14;
	const GEN_FLT x63 = x59 * x13;
	const GEN_FLT x64 = x52 * x28 + x55 * x20 + x49 * x22 * x17;
	const GEN_FLT x65 =
		(x51 + x50 * x24 + 2 * x52 * x20) * sensor_x + (-x54 - x56 + x61) * sensor_y + (x62 + x63 + x64) * sensor_z;
	const GEN_FLT x66 = x1 * x31;
	const GEN_FLT x67 = x5 * x40;
	const GEN_FLT x68 = -x66 + x67;
	const GEN_FLT x69 = x53 * x17;
	const GEN_FLT x70 = x52 * x13;
	const GEN_FLT x71 = x57 * x16 + x58 * x55 + x60 * x16;
	const GEN_FLT x72 =
		(x51 + x50 * x26 + 2 * x60 * x14) * sensor_y + (x54 + x56 + x61) * sensor_x + (-x69 - x70 + x71) * sensor_z;
	const GEN_FLT x73 =
		(x51 + x50 * x42 + 2 * x55 * x28) * sensor_z + (-x62 - x63 + x64) * sensor_x + (x69 + x70 + x71) * sensor_y;
	const GEN_FLT x74 = x1 * x36;
	const GEN_FLT x75 = x31 * x10;
	const GEN_FLT x76 = x74 + x75;
	const GEN_FLT x77 = x65 * x48 + x72 * x68 + x73 * x76;
	const GEN_FLT x78 = x77 + x25 * (x4 + x3 * x6 + x7 * x11) + x30 * (-x33 - x35 + x41) + x43 * (x44 + x45 + x47);
	const GEN_FLT x79 = 1 + x78;
	const GEN_FLT x80 = pow(x31, 2);
	const GEN_FLT x81 = x8 + x9 * x80;
	const GEN_FLT x82 = x1 * x5;
	const GEN_FLT x83 = x40 * x31;
	const GEN_FLT x84 = x82 + x83;
	const GEN_FLT x85 = -x74 + x75;
	const GEN_FLT x86 = lh_pz + x81 * x43 + x84 * x30 + x85 * x25;
	const GEN_FLT x87 = pow(x86, -1);
	const GEN_FLT x88 = x5 * x32;
	const GEN_FLT x89 = x1 * x7;
	const GEN_FLT x90 = x31 * x39 + x40 * x34 + x3 * x31 * x36;
	const GEN_FLT x91 = 2 * x46;
	const GEN_FLT x92 = x81 * x73 + x84 * x72 + x85 * x65;
	const GEN_FLT x93 = x92 + x25 * (-x44 - x45 + x47) + x30 * (x88 + x89 + x90) + x43 * (x4 + x3 * x80 + x91 * x34);
	const GEN_FLT x94 = lh_px + x48 * x25 + x68 * x30 + x76 * x43;
	const GEN_FLT x95 = pow(x86, 2);
	const GEN_FLT x96 = pow(x95, -1);
	const GEN_FLT x97 = x96 * x94;
	const GEN_FLT x98 = x93 * x97;
	const GEN_FLT x99 = x95 + pow(x94, 2);
	const GEN_FLT x100 = pow(x99, -1);
	const GEN_FLT x101 = x95 * x100;
	const GEN_FLT x102 = -x82 + x83;
	const GEN_FLT x103 = pow(x36, 2);
	const GEN_FLT x104 = x8 + x9 * x103;
	const GEN_FLT x105 = x66 + x67;
	const GEN_FLT x106 = lh_py + x25 * x105 + x30 * x104 + x43 * x102;
	const GEN_FLT x107 = pow(x106, 2);
	const GEN_FLT x108 = pow(1 - x100 * x107 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x109 = 2 * x94;
	const GEN_FLT x110 = 2 * x86;
	const GEN_FLT x111 = x93 * x110;
	const GEN_FLT x112 = (1.0 / 2.0) * x106 * tilt_0 / pow(x99, 3.0 / 2.0);
	const GEN_FLT x113 = x65 * x105 + x72 * x104 + x73 * x102;
	const GEN_FLT x114 =
		x113 + x25 * (x33 + x35 + x41) + x30 * (x4 + x3 * x103 + 2 * x36 * x39) + x43 * (-x88 - x89 + x90);
	const GEN_FLT x115 = tilt_0 / sqrt(x99);
	const GEN_FLT x116 = x114 * x115;
	const GEN_FLT x117 = -x101 * (x98 - x87 * x79) - x108 * (x116 - x112 * (x111 + x79 * x109));
	const GEN_FLT x118 = -x86;
	const GEN_FLT x119 = sin(1.5707963267949 + gibPhase_0 - phase_0 - asin(x106 * x115) - atan2(x94, x118)) * gibMag_0;
	const GEN_FLT x120 = -x87 * x114;
	const GEN_FLT x121 = x96 * x106;
	const GEN_FLT x122 = x93 * x121;
	const GEN_FLT x123 = 2 * x95 * atan2(x106, x118) * curve_0 / (x107 + x95);
	const GEN_FLT x124 = -x87 * x78;
	const GEN_FLT x125 = x78 * x109;
	const GEN_FLT x126 = 1 + x114;
	const GEN_FLT x127 = -x101 * (x124 + x98) - x108 * (x115 * x126 - (x111 + x125) * x112);
	const GEN_FLT x128 = 1 + x93;
	const GEN_FLT x129 = -x101 * (x124 + x97 * x128) - x108 * (x116 - x112 * (x125 + x110 * x128));
	const GEN_FLT x130 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x131 = x1 * x130;
	const GEN_FLT x132 = -x131;
	const GEN_FLT x133 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x134 = x8 * x130;
	const GEN_FLT x135 = x31 * x134;
	const GEN_FLT x136 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x137 = x1 * x136;
	const GEN_FLT x138 = x74 * x130;
	const GEN_FLT x139 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x140 = x10 * x139 + x40 * x133 + x5 * x138;
	const GEN_FLT x141 = x36 * x134;
	const GEN_FLT x142 = x1 * x139;
	const GEN_FLT x143 = x82 * x31;
	const GEN_FLT x144 = x10 * x136 + x130 * x143 + x46 * x133;
	const GEN_FLT x145 =
		x77 + x25 * (x132 + x11 * x133 + x6 * x131) + x30 * (-x135 - x137 + x140) + x43 * (x141 + x142 + x144);
	const GEN_FLT x146 = x5 * x134;
	const GEN_FLT x147 = x1 * x133;
	const GEN_FLT x148 = x31 * x138 + x40 * x136 + x46 * x139;
	const GEN_FLT x149 =
		x92 + x25 * (-x141 - x142 + x144) + x30 * (x146 + x147 + x148) + x43 * (x132 + x80 * x131 + x91 * x136);
	const GEN_FLT x150 = 2 * x40;
	const GEN_FLT x151 =
		x113 + x25 * (x135 + x137 + x140) + x30 * (x132 + x103 * x131 + x139 * x150) + x43 * (-x146 - x147 + x148);
	const GEN_FLT x152 = -x108 * (x115 * x151 - (x109 * x145 + x110 * x149) * x112) - (-x87 * x145 + x97 * x149) * x101;
	const GEN_FLT x153 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x154 = x1 * x153;
	const GEN_FLT x155 = -x154;
	const GEN_FLT x156 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x157 = x9 * x156;
	const GEN_FLT x158 = x8 * x153;
	const GEN_FLT x159 = x31 * x158;
	const GEN_FLT x160 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x161 = x1 * x160;
	const GEN_FLT x162 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x163 = x74 * x153;
	const GEN_FLT x164 = x10 * x162 + x36 * x157 + x5 * x163;
	const GEN_FLT x165 = x36 * x158;
	const GEN_FLT x166 = x1 * x162;
	const GEN_FLT x167 = x9 * x160;
	const GEN_FLT x168 = x143 * x153 + x31 * x157 + x5 * x167;
	const GEN_FLT x169 =
		x77 + x25 * (x155 + 2 * x5 * x157 + x6 * x154) + x30 * (-x159 - x161 + x164) + x43 * (x165 + x166 + x168);
	const GEN_FLT x170 = x5 * x158;
	const GEN_FLT x171 = x1 * x156;
	const GEN_FLT x172 = x31 * x163 + x36 * x167 + x46 * x162;
	const GEN_FLT x173 =
		x92 + x25 * (-x165 - x166 + x168) + x30 * (x170 + x171 + x172) + x43 * (x155 + 2 * x31 * x167 + x80 * x154);
	const GEN_FLT x174 =
		x113 + x25 * (x159 + x161 + x164) + x30 * (x155 + x103 * x154 + x162 * x150) + x43 * (-x170 - x171 + x172);
	const GEN_FLT x175 = -x108 * (x115 * x174 - (x109 * x169 + x110 * x173) * x112) - (-x87 * x169 + x97 * x173) * x101;
	const GEN_FLT x176 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x177 = x1 * x176;
	const GEN_FLT x178 = -x177;
	const GEN_FLT x179 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x180 = x8 * x176;
	const GEN_FLT x181 = x31 * x180;
	const GEN_FLT x182 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x183 = x1 * x182;
	const GEN_FLT x184 = x74 * x176;
	const GEN_FLT x185 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x186 = x10 * x185 + x40 * x179 + x5 * x184;
	const GEN_FLT x187 = x36 * x180;
	const GEN_FLT x188 = x1 * x185;
	const GEN_FLT x189 = x10 * x182 + x176 * x143 + x46 * x179;
	const GEN_FLT x190 =
		x77 + x25 * (x178 + x11 * x179 + x6 * x177) + x30 * (-x181 - x183 + x186) + x43 * (x187 + x188 + x189);
	const GEN_FLT x191 = x5 * x180;
	const GEN_FLT x192 = x1 * x179;
	const GEN_FLT x193 = x31 * x184 + x40 * x182 + x46 * x185;
	const GEN_FLT x194 =
		x92 + x25 * (-x187 - x188 + x189) + x30 * (x191 + x192 + x193) + x43 * (x178 + x80 * x177 + x91 * x182);
	const GEN_FLT x195 =
		x113 + x25 * (x181 + x183 + x186) + x30 * (x178 + x103 * x177 + x185 * x150) + x43 * (-x191 - x192 + x193);
	const GEN_FLT x196 = -x108 * (x115 * x195 - (x109 * x190 + x110 * x194) * x112) - (-x87 * x190 + x97 * x194) * x101;
	out[0] = x117 + x119 * x117 + (x120 + x122) * x123;
	out[1] = x127 + x119 * x127 + x123 * (x122 - x87 * x126);
	out[2] = x129 + x119 * x129 + x123 * (x120 + x121 * x128);
	out[3] = x152 + x119 * x152 + x123 * (x121 * x149 - x87 * x151);
	out[4] = x175 + x119 * x175 + x123 * (x121 * x173 - x87 * x174);
	out[5] = x196 + x119 * x196 + x123 * (x121 * x194 - x87 * x195);
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 - x1;
	const GEN_FLT x3 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x4 = pow(x3, 2);
	const GEN_FLT x5 = x1 + x2 * x4;
	const GEN_FLT x6 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1e-10));
	const GEN_FLT x7 = cos(x6);
	const GEN_FLT x8 = 1 - x7;
	const GEN_FLT x9 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								  : (1e-10)))
							? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											 : (1e-10)))
							: (0));
	const GEN_FLT x10 = pow(x9, 2);
	const GEN_FLT x11 = sin(x6);
	const GEN_FLT x12 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x13 = x12 * x11;
	const GEN_FLT x14 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x15 = x8 * x9;
	const GEN_FLT x16 = x15 * x14;
	const GEN_FLT x17 = x14 * x11;
	const GEN_FLT x18 = x8 * x12;
	const GEN_FLT x19 = x9 * x18;
	const GEN_FLT x20 = obj_pz + (x13 + x16) * sensor_y + (-x17 + x19) * sensor_x + (x7 + x8 * x10) * sensor_z;
	const GEN_FLT x21 = sin(x0);
	const GEN_FLT x22 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (1));
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x25 = x2 * x24;
	const GEN_FLT x26 = x3 * x25;
	const GEN_FLT x27 = x23 + x26;
	const GEN_FLT x28 = pow(x14, 2);
	const GEN_FLT x29 = x9 * x11;
	const GEN_FLT x30 = x14 * x18;
	const GEN_FLT x31 = obj_py + (-x13 + x16) * sensor_z + (x29 + x30) * sensor_x + (x7 + x8 * x28) * sensor_y;
	const GEN_FLT x32 = x24 * x21;
	const GEN_FLT x33 = x2 * x3;
	const GEN_FLT x34 = x33 * x22;
	const GEN_FLT x35 = -x32 + x34;
	const GEN_FLT x36 = pow(x12, 2);
	const GEN_FLT x37 = obj_px + (x17 + x19) * sensor_z + (-x29 + x30) * sensor_y + (x7 + x8 * x36) * sensor_x;
	const GEN_FLT x38 = lh_pz + x31 * x27 + x35 * x37 + x5 * x20;
	const GEN_FLT x39 = pow(x38, 2);
	const GEN_FLT x40 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x41 = x40 * x11;
	const GEN_FLT x42 = -x41;
	const GEN_FLT x43 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x44 = x7 * x40;
	const GEN_FLT x45 = x9 * x44;
	const GEN_FLT x46 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x47 = x46 * x11;
	const GEN_FLT x48 = x8 * x14;
	const GEN_FLT x49 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x50 = x43 * x48 + x49 * x18 + x41 * x14 * x12;
	const GEN_FLT x51 = x44 * x14;
	const GEN_FLT x52 = x49 * x11;
	const GEN_FLT x53 = x9 * x41;
	const GEN_FLT x54 = x43 * x15 + x46 * x18 + x53 * x12;
	const GEN_FLT x55 =
		(x42 + x41 * x36 + 2 * x43 * x18) * sensor_x + (-x45 - x47 + x50) * sensor_y + (x51 + x52 + x54) * sensor_z;
	const GEN_FLT x56 = x3 * x21;
	const GEN_FLT x57 = x25 * x22;
	const GEN_FLT x58 = x56 + x57;
	const GEN_FLT x59 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x60 = x59 * x21;
	const GEN_FLT x61 = -x60;
	const GEN_FLT x62 = pow(x24, 2);
	const GEN_FLT x63 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x64 = x2 * x63;
	const GEN_FLT x65 = x1 + x2 * x62;
	const GEN_FLT x66 = x44 * x12;
	const GEN_FLT x67 = x43 * x11;
	const GEN_FLT x68 = x46 * x48 + x49 * x15 + x53 * x14;
	const GEN_FLT x69 =
		(x42 + x41 * x28 + 2 * x48 * x49) * sensor_y + (x45 + x47 + x50) * sensor_x + (-x66 - x67 + x68) * sensor_z;
	const GEN_FLT x70 = x1 * x59;
	const GEN_FLT x71 = x70 * x22;
	const GEN_FLT x72 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x73 = x72 * x21;
	const GEN_FLT x74 = x59 * x24;
	const GEN_FLT x75 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x76 = x3 * x64 + x74 * x56 + x75 * x25;
	const GEN_FLT x77 = x3 * x70;
	const GEN_FLT x78 = x75 * x21;
	const GEN_FLT x79 = x64 * x22 + x72 * x25 + x74 * x23;
	const GEN_FLT x80 =
		(x42 + x41 * x10 + 2 * x46 * x15) * sensor_z + (-x51 - x52 + x54) * sensor_x + (x66 + x67 + x68) * sensor_y;
	const GEN_FLT x81 = -x23 + x26;
	const GEN_FLT x82 = x20 * (-x71 - x73 + x76) + x31 * (x61 + x60 * x62 + 2 * x64 * x24) + x37 * (x77 + x78 + x79) +
						x58 * x55 + x65 * x69 + x80 * x81;
	const GEN_FLT x83 = pow(x38, -1);
	const GEN_FLT x84 = lh_py + x58 * x37 + x65 * x31 + x81 * x20;
	const GEN_FLT x85 = x70 * x24;
	const GEN_FLT x86 = x63 * x21;
	const GEN_FLT x87 = x2 * x22;
	const GEN_FLT x88 = x72 * x33 + x87 * x75 + x56 * x59 * x22;
	const GEN_FLT x89 = x20 * (x61 + x4 * x60 + 2 * x75 * x33) + x31 * (x71 + x73 + x76) + x37 * (-x85 - x86 + x88) +
						x5 * x80 + x55 * x35 + x69 * x27;
	const GEN_FLT x90 = x89 / x39;
	const GEN_FLT x91 = pow(x84, 2);
	const GEN_FLT x92 = -x38;
	const GEN_FLT x93 = atan2(x84, x92);
	const GEN_FLT x94 = 2 * (-x82 * x83 + x84 * x90) * x93 * x39 * curve_0 / (x39 + x91);
	const GEN_FLT x95 = x32 + x34;
	const GEN_FLT x96 = -x56 + x57;
	const GEN_FLT x97 = pow(x22, 2);
	const GEN_FLT x98 = x1 + x2 * x97;
	const GEN_FLT x99 = lh_px + x95 * x20 + x96 * x31 + x98 * x37;
	const GEN_FLT x100 = x39 + pow(x99, 2);
	const GEN_FLT x101 = pow(x100, -1);
	const GEN_FLT x102 = x20 * (x85 + x86 + x88) + x31 * (-x77 - x78 + x79) + x37 * (x61 + x60 * x97 + 2 * x87 * x72) +
						 x55 * x98 + x69 * x96 + x80 * x95;
	const GEN_FLT x103 = -x39 * x101 * (-x83 * x102 + x90 * x99);
	const GEN_FLT x104 = pow(1 - x91 * x101 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x105 = pow(x100, -1.0 / 2.0);
	const GEN_FLT x106 =
		x82 * x105 * tilt_0 + (-1.0 / 2.0) * x84 * tilt_0 * (2 * x89 * x38 + 2 * x99 * x102) / pow(x100, 3.0 / 2.0);
	const GEN_FLT x107 = x103 - x104 * x106;
	const GEN_FLT x108 = -1 + x107;
	const GEN_FLT x109 = x84 * x105;
	const GEN_FLT x110 = 1.5707963267949 + gibPhase_0 - phase_0 - asin(x109 * tilt_0) - atan2(x99, x92);
	const GEN_FLT x111 = sin(x110) * gibMag_0;
	const GEN_FLT x112 = x103 - (x106 + x109) * x104;
	const GEN_FLT x113 = x107 + x94;
	const GEN_FLT x114 = x113 + x107 * x111;
	out[0] = x108 + x94 + x108 * x111;
	out[1] = x112 + x94 + x111 * x112;
	out[2] = x114 + pow(x93, 2);
	out[3] = x113 + x111 * (1 + x107);
	out[4] = x114 - cos(x110);
	out[5] = x114;
	out[6] = x114;
}

/** Applying function <function reproject_axis_y at 0x7effc26e3440> */
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x5 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (1));
	const GEN_FLT x6 = cos(x0);
	const GEN_FLT x7 = 1 - x6;
	const GEN_FLT x8 = x5 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1e-10));
	const GEN_FLT x11 = cos(x10);
	const GEN_FLT x12 = 1 - x11;
	const GEN_FLT x13 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x14 = sin(x10);
	const GEN_FLT x15 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x16 = x15 * x14;
	const GEN_FLT x17 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x18 = x13 * x12;
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = x14 * x17;
	const GEN_FLT x21 = x15 * x18;
	const GEN_FLT x22 =
		obj_pz + (x11 + pow(x13, 2) * x12) * sensor_z + (x16 + x19) * sensor_y + (-x20 + x21) * sensor_x;
	const GEN_FLT x23 = x1 * x4;
	const GEN_FLT x24 = x2 * x8;
	const GEN_FLT x25 = x14 * x13;
	const GEN_FLT x26 = x15 * x12 * x17;
	const GEN_FLT x27 =
		obj_py + (x11 + x12 * pow(x17, 2)) * sensor_y + (-x16 + x19) * sensor_z + (x25 + x26) * sensor_x;
	const GEN_FLT x28 =
		obj_px + (x11 + pow(x15, 2) * x12) * sensor_x + (x20 + x21) * sensor_z + (-x25 + x26) * sensor_y;
	const GEN_FLT x29 = lh_px + x28 * (x6 + pow(x5, 2) * x7) + (-x23 + x24) * x27 + (x3 + x9) * x22;
	const GEN_FLT x30 = x1 * x5;
	const GEN_FLT x31 = x2 * x4 * x7;
	const GEN_FLT x32 = lh_pz + x22 * (x6 + pow(x4, 2) * x7) + (-x3 + x9) * x28 + (x30 + x31) * x27;
	const GEN_FLT x33 = -x32;
	const GEN_FLT x34 = lh_py + x27 * (x6 + pow(x2, 2) * x7) + (x23 + x24) * x28 + (-x30 + x31) * x22;
	const GEN_FLT x35 = -phase_1 - asin(x29 * tilt_1 / sqrt(pow(x32, 2) + pow(x34, 2))) - atan2(-x34, x33);
	out[0] = x35 - cos(1.5707963267949 + gibPhase_1 + x35) * gibMag_1 + pow(atan2(x29, x33), 2) * curve_1;
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
	const GEN_FLT x0 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x1 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1e-10));
	const GEN_FLT x2 = sin(x1);
	const GEN_FLT x3 = x0 * x2;
	const GEN_FLT x4 = -x3;
	const GEN_FLT x5 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								  : (1e-10)))
							? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											 : (1e-10)))
							: (1));
	const GEN_FLT x6 = pow(x5, 2);
	const GEN_FLT x7 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								  : (1e-10)))
							? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
							   pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										: (1e-10)),
								   2))
							: (0));
	const GEN_FLT x8 = cos(x1);
	const GEN_FLT x9 = 1 - x8;
	const GEN_FLT x10 = x5 * x9;
	const GEN_FLT x11 = 2 * x10;
	const GEN_FLT x12 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x13 = x0 * x8;
	const GEN_FLT x14 = x13 * x12;
	const GEN_FLT x15 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x16 = x2 * x15;
	const GEN_FLT x17 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x18 = x2 * x17;
	const GEN_FLT x19 = x5 * x18;
	const GEN_FLT x20 = x9 * x17;
	const GEN_FLT x21 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x22 = x0 * x19 + x21 * x10 + x7 * x20;
	const GEN_FLT x23 = x13 * x17;
	const GEN_FLT x24 = x2 * x21;
	const GEN_FLT x25 = x2 * x12;
	const GEN_FLT x26 = x5 * x25;
	const GEN_FLT x27 = x9 * x12;
	const GEN_FLT x28 = x0 * x26 + x15 * x10 + x7 * x27;
	const GEN_FLT x29 =
		(-x14 - x16 + x22) * sensor_y + (x23 + x24 + x28) * sensor_z + (x4 + x3 * x6 + x7 * x11) * sensor_x;
	const GEN_FLT x30 = 1 + x29;
	const GEN_FLT x31 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x32 = sin(x31);
	const GEN_FLT x33 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x34 = x32 * x33;
	const GEN_FLT x35 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (1));
	const GEN_FLT x36 = cos(x31);
	const GEN_FLT x37 = 1 - x36;
	const GEN_FLT x38 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x39 = x38 * x37;
	const GEN_FLT x40 = x35 * x39;
	const GEN_FLT x41 = x34 + x40;
	const GEN_FLT x42 = pow(x38, 2);
	const GEN_FLT x43 = x36 + x42 * x37;
	const GEN_FLT x44 = x5 * x13;
	const GEN_FLT x45 = x2 * x7;
	const GEN_FLT x46 = x25 * x17;
	const GEN_FLT x47 = x0 * x46 + x20 * x15 + x21 * x27;
	const GEN_FLT x48 = pow(x17, 2);
	const GEN_FLT x49 = 2 * x20;
	const GEN_FLT x50 =
		(x14 + x16 + x22) * sensor_x + (x4 + x3 * x48 + x49 * x21) * sensor_y + (-x44 - x45 + x47) * sensor_z;
	const GEN_FLT x51 = x50 * x43;
	const GEN_FLT x52 = pow(x12, 2);
	const GEN_FLT x53 = 2 * x27;
	const GEN_FLT x54 =
		(-x23 - x24 + x28) * sensor_x + (x4 + x3 * x52 + x53 * x15) * sensor_z + (x44 + x45 + x47) * sensor_y;
	const GEN_FLT x55 = x32 * x35;
	const GEN_FLT x56 = x33 * x39;
	const GEN_FLT x57 = -x55 + x56;
	const GEN_FLT x58 = x5 * x27;
	const GEN_FLT x59 = x5 * x20;
	const GEN_FLT x60 = obj_px + (x18 + x58) * sensor_z + (-x25 + x59) * sensor_y + (x8 + x6 * x9) * sensor_x;
	const GEN_FLT x61 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x62 = x61 * x36;
	const GEN_FLT x63 = x62 * x33;
	const GEN_FLT x64 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x65 = x64 * x32;
	const GEN_FLT x66 = x61 * x32;
	const GEN_FLT x67 = x66 * x38;
	const GEN_FLT x68 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x69 = x68 * x37;
	const GEN_FLT x70 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x71 = x70 * x37;
	const GEN_FLT x72 = x67 * x35 + x69 * x35 + x71 * x38;
	const GEN_FLT x73 = -x66;
	const GEN_FLT x74 = x2 * x5;
	const GEN_FLT x75 = x20 * x12;
	const GEN_FLT x76 = obj_py + (x25 + x59) * sensor_x + (-x74 + x75) * sensor_z + (x8 + x9 * x48) * sensor_y;
	const GEN_FLT x77 = x62 * x35;
	const GEN_FLT x78 = x70 * x32;
	const GEN_FLT x79 = x64 * x37;
	const GEN_FLT x80 = x67 * x33 + x69 * x33 + x79 * x38;
	const GEN_FLT x81 = obj_pz + (-x18 + x58) * sensor_x + (x74 + x75) * sensor_y + (x8 + x9 * x52) * sensor_z;
	const GEN_FLT x82 = x60 * (x63 + x65 + x72) + x76 * (x73 + x66 * x42 + 2 * x69 * x38) + x81 * (-x77 - x78 + x80);
	const GEN_FLT x83 = x82 + x54 * x57;
	const GEN_FLT x84 = x51 + x83 + x41 * x30;
	const GEN_FLT x85 = pow(x33, 2);
	const GEN_FLT x86 = x36 + x85 * x37;
	const GEN_FLT x87 = x55 + x56;
	const GEN_FLT x88 = x32 * x38;
	const GEN_FLT x89 = x33 * x35;
	const GEN_FLT x90 = x89 * x37;
	const GEN_FLT x91 = -x88 + x90;
	const GEN_FLT x92 = lh_pz + x60 * x91 + x81 * x86 + x87 * x76;
	const GEN_FLT x93 = pow(x92, -1);
	const GEN_FLT x94 = x86 * x54;
	const GEN_FLT x95 = x87 * x50;
	const GEN_FLT x96 = x62 * x38;
	const GEN_FLT x97 = x68 * x32;
	const GEN_FLT x98 = x71 * x33 + x79 * x35 + x89 * x66;
	const GEN_FLT x99 = x60 * (-x96 - x97 + x98) + x76 * (x77 + x78 + x80) + x81 * (x73 + 2 * x79 * x33 + x85 * x66);
	const GEN_FLT x100 = x94 + x95 + x99 + x91 * x30;
	const GEN_FLT x101 = pow(x92, 2);
	const GEN_FLT x102 = pow(x101, -1);
	const GEN_FLT x103 = lh_py + x60 * x41 + x76 * x43 + x81 * x57;
	const GEN_FLT x104 = x103 * x102;
	const GEN_FLT x105 = x101 + pow(x103, 2);
	const GEN_FLT x106 = pow(x105, -1);
	const GEN_FLT x107 = x101 * x106;
	const GEN_FLT x108 = x88 + x90;
	const GEN_FLT x109 = -x34 + x40;
	const GEN_FLT x110 = pow(x35, 2);
	const GEN_FLT x111 = x36 + x37 * x110;
	const GEN_FLT x112 = lh_px + x60 * x111 + x76 * x109 + x81 * x108;
	const GEN_FLT x113 = pow(x112, 2);
	const GEN_FLT x114 = pow(1 - x106 * x113 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x115 = 2 * x103;
	const GEN_FLT x116 = 2 * x92;
	const GEN_FLT x117 = (1.0 / 2.0) * x112 * tilt_1 / pow(x105, 3.0 / 2.0);
	const GEN_FLT x118 = x54 * x108;
	const GEN_FLT x119 = x60 * (x73 + x66 * x110 + 2 * x71 * x35) + x76 * (-x63 - x65 + x72) + x81 * (x96 + x97 + x98);
	const GEN_FLT x120 = x119 + x50 * x109;
	const GEN_FLT x121 = x118 + x120 + x30 * x111;
	const GEN_FLT x122 = tilt_1 / sqrt(x105);
	const GEN_FLT x123 = -x107 * (-x100 * x104 + x84 * x93) - x114 * (-x117 * (x100 * x116 + x84 * x115) + x122 * x121);
	const GEN_FLT x124 = -x92;
	const GEN_FLT x125 =
		sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x112 * x122) - atan2(-x103, x124)) * gibMag_1;
	const GEN_FLT x126 = x102 * x112;
	const GEN_FLT x127 = 2 * x101 * atan2(x112, x124) * curve_1 / (x101 + x113);
	const GEN_FLT x128 = x41 * x29;
	const GEN_FLT x129 = 1 + x50;
	const GEN_FLT x130 = x128 + x83 + x43 * x129;
	const GEN_FLT x131 = x99 + x91 * x29;
	const GEN_FLT x132 = x131 + x94 + x87 * x129;
	const GEN_FLT x133 = x29 * x111;
	const GEN_FLT x134 = x118 + x119 + x133 + x109 * x129;
	const GEN_FLT x135 =
		-x107 * (-x104 * x132 + x93 * x130) - x114 * (x122 * x134 - (x115 * x130 + x116 * x132) * x117);
	const GEN_FLT x136 = 1 + x54;
	const GEN_FLT x137 = x128 + x51 + x82 + x57 * x136;
	const GEN_FLT x138 = x131 + x95 + x86 * x136;
	const GEN_FLT x139 = x120 + x133 + x108 * x136;
	const GEN_FLT x140 =
		-x107 * (-x104 * x138 + x93 * x137) - x114 * (x122 * x139 - (x115 * x137 + x116 * x138) * x117);
	const GEN_FLT x141 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x142 = x2 * x141;
	const GEN_FLT x143 = -x142;
	const GEN_FLT x144 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x145 = x8 * x141;
	const GEN_FLT x146 = x12 * x145;
	const GEN_FLT x147 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x148 = x2 * x147;
	const GEN_FLT x149 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qi / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x150 = x10 * x149 + x19 * x141 + x20 * x144;
	const GEN_FLT x151 = x17 * x145;
	const GEN_FLT x152 = x2 * x149;
	const GEN_FLT x153 = x25 * x141;
	const GEN_FLT x154 = x10 * x147 + x27 * x144 + x5 * x153;
	const GEN_FLT x155 =
		(x143 + x11 * x144 + x6 * x142) * sensor_x + (-x146 - x148 + x150) * sensor_y + (x151 + x152 + x154) * sensor_z;
	const GEN_FLT x156 = x5 * x145;
	const GEN_FLT x157 = x2 * x144;
	const GEN_FLT x158 = x17 * x153 + x20 * x147 + x27 * x149;
	const GEN_FLT x159 = (x143 + x48 * x142 + x49 * x149) * sensor_y + (x146 + x148 + x150) * sensor_x +
						 (-x156 - x157 + x158) * sensor_z;
	const GEN_FLT x160 = (x143 + x52 * x142 + x53 * x147) * sensor_z + (-x151 - x152 + x154) * sensor_x +
						 (x156 + x157 + x158) * sensor_y;
	const GEN_FLT x161 = x82 + x41 * x155 + x43 * x159 + x57 * x160;
	const GEN_FLT x162 = x99 + x86 * x160 + x87 * x159 + x91 * x155;
	const GEN_FLT x163 = x119 + x108 * x160 + x109 * x159 + x111 * x155;
	const GEN_FLT x164 =
		-x107 * (-x104 * x162 + x93 * x161) - x114 * (x122 * x163 - (x115 * x161 + x116 * x162) * x117);
	const GEN_FLT x165 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x166 = x8 * x165;
	const GEN_FLT x167 = x12 * x166;
	const GEN_FLT x168 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x169 = x2 * x168;
	const GEN_FLT x170 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x171 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qj / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x172 = x9 * x171;
	const GEN_FLT x173 = x19 * x165 + x20 * x170 + x5 * x172;
	const GEN_FLT x174 = x5 * x166;
	const GEN_FLT x175 = x2 * x170;
	const GEN_FLT x176 = x12 * x172 + x20 * x168 + x46 * x165;
	const GEN_FLT x177 = x2 * x165;
	const GEN_FLT x178 = -x177;
	const GEN_FLT x179 = (x167 + x169 + x173) * sensor_x + (-x174 - x175 + x176) * sensor_z +
						 (x178 + x48 * x177 + x49 * x171) * sensor_y;
	const GEN_FLT x180 = x17 * x166;
	const GEN_FLT x181 = x2 * x171;
	const GEN_FLT x182 = x10 * x168 + x26 * x165 + x27 * x170;
	const GEN_FLT x183 =
		(-x167 - x169 + x173) * sensor_y + (x178 + x11 * x170 + x6 * x177) * sensor_x + (x180 + x181 + x182) * sensor_z;
	const GEN_FLT x184 = (x174 + x175 + x176) * sensor_y + (x178 + x52 * x177 + x53 * x168) * sensor_z +
						 (-x180 - x181 + x182) * sensor_x;
	const GEN_FLT x185 = x82 + x41 * x183 + x43 * x179 + x57 * x184;
	const GEN_FLT x186 = x99 + x86 * x184 + x87 * x179 + x91 * x183;
	const GEN_FLT x187 = x119 + x108 * x184 + x109 * x179 + x111 * x183;
	const GEN_FLT x188 =
		-x107 * (-x104 * x186 + x93 * x185) - x114 * (x122 * x187 - (x115 * x185 + x116 * x186) * x117);
	const GEN_FLT x189 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							  : (0));
	const GEN_FLT x190 = x2 * x189;
	const GEN_FLT x191 = -x190;
	const GEN_FLT x192 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qi *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x193 = x8 * x189;
	const GEN_FLT x194 = x12 * x193;
	const GEN_FLT x195 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qk *
									 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x196 = x2 * x195;
	const GEN_FLT x197 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									: (1e-10)))
							  ? (-obj_qj *
								 ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
									  ? (obj_qk / sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x198 = x10 * x197 + x19 * x189 + x20 * x192;
	const GEN_FLT x199 = x17 * x193;
	const GEN_FLT x200 = x2 * x197;
	const GEN_FLT x201 = x10 * x195 + x26 * x189 + x27 * x192;
	const GEN_FLT x202 =
		(x191 + x11 * x192 + x6 * x190) * sensor_x + (-x194 - x196 + x198) * sensor_y + (x199 + x200 + x201) * sensor_z;
	const GEN_FLT x203 = x5 * x193;
	const GEN_FLT x204 = x2 * x192;
	const GEN_FLT x205 = x20 * x195 + x27 * x197 + x46 * x189;
	const GEN_FLT x206 = (x191 + x48 * x190 + x49 * x197) * sensor_y + (x194 + x196 + x198) * sensor_x +
						 (-x203 - x204 + x205) * sensor_z;
	const GEN_FLT x207 = (x191 + x52 * x190 + x53 * x195) * sensor_z + (-x199 - x200 + x201) * sensor_x +
						 (x203 + x204 + x205) * sensor_y;
	const GEN_FLT x208 = x82 + x41 * x202 + x43 * x206 + x57 * x207;
	const GEN_FLT x209 = x99 + x86 * x207 + x87 * x206 + x91 * x202;
	const GEN_FLT x210 = x119 + x202 * x111 + x206 * x109 + x207 * x108;
	const GEN_FLT x211 =
		-x107 * (-x209 * x104 + x93 * x208) - x114 * (x210 * x122 - (x208 * x115 + x209 * x116) * x117);
	out[0] = x123 + x123 * x125 + x127 * (x100 * x126 - x93 * x121);
	out[1] = x135 + x125 * x135 + x127 * (x126 * x132 - x93 * x134);
	out[2] = x140 + x125 * x140 + x127 * (x126 * x138 - x93 * x139);
	out[3] = x164 + x125 * x164 + x127 * (x126 * x162 - x93 * x163);
	out[4] = x188 + x125 * x188 + x127 * (x126 * x186 - x93 * x187);
	out[5] = x211 + x127 * (x209 * x126 - x93 * x210) + x211 * x125;
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
	const GEN_FLT x0 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1e-10));
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 - x1;
	const GEN_FLT x3 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								  : (1e-10)))
							? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											 : (1e-10)))
							: (1));
	const GEN_FLT x4 = pow(x3, 2);
	const GEN_FLT x5 = x1 + x2 * x4;
	const GEN_FLT x6 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x7 = sin(x0);
	const GEN_FLT x8 = x6 * x7;
	const GEN_FLT x9 = -x8;
	const GEN_FLT x10 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x11 = x2 * x3;
	const GEN_FLT x12 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x13 = x1 * x6;
	const GEN_FLT x14 = x13 * x12;
	const GEN_FLT x15 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x16 = x7 * x15;
	const GEN_FLT x17 = x3 * x7;
	const GEN_FLT x18 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x19 = x6 * x18;
	const GEN_FLT x20 = x2 * x18;
	const GEN_FLT x21 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x22 = x2 * x21;
	const GEN_FLT x23 = x19 * x17 + x20 * x10 + x3 * x22;
	const GEN_FLT x24 = x13 * x18;
	const GEN_FLT x25 = x7 * x21;
	const GEN_FLT x26 = x2 * x12;
	const GEN_FLT x27 = x15 * x11 + x26 * x10 + x6 * x12 * x17;
	const GEN_FLT x28 =
		(-x14 - x16 + x23) * sensor_y + (x24 + x25 + x27) * sensor_z + (x9 + 2 * x11 * x10 + x4 * x8) * sensor_x;
	const GEN_FLT x29 = x28 + x5;
	const GEN_FLT x30 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x31 = sin(x30);
	const GEN_FLT x32 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x33 = x32 * x31;
	const GEN_FLT x34 = cos(x30);
	const GEN_FLT x35 = 1 - x34;
	const GEN_FLT x36 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x37 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (1));
	const GEN_FLT x38 = x36 * x35 * x37;
	const GEN_FLT x39 = x33 + x38;
	const GEN_FLT x40 = pow(x36, 2);
	const GEN_FLT x41 = x34 + x40 * x35;
	const GEN_FLT x42 = x7 * x12;
	const GEN_FLT x43 = x3 * x20;
	const GEN_FLT x44 = x42 + x43;
	const GEN_FLT x45 = x3 * x13;
	const GEN_FLT x46 = x7 * x10;
	const GEN_FLT x47 = x20 * x15 + x22 * x12 + x42 * x19;
	const GEN_FLT x48 = pow(x18, 2);
	const GEN_FLT x49 =
		(x14 + x16 + x23) * sensor_x + (-x45 - x46 + x47) * sensor_z + (x9 + 2 * x20 * x21 + x8 * x48) * sensor_y;
	const GEN_FLT x50 = x44 + x49;
	const GEN_FLT x51 = x7 * x18;
	const GEN_FLT x52 = x3 * x26;
	const GEN_FLT x53 = -x51 + x52;
	const GEN_FLT x54 = pow(x12, 2);
	const GEN_FLT x55 =
		(-x24 - x25 + x27) * sensor_x + (x45 + x46 + x47) * sensor_y + (x9 + 2 * x26 * x15 + x8 * x54) * sensor_z;
	const GEN_FLT x56 = x53 + x55;
	const GEN_FLT x57 = x31 * x37;
	const GEN_FLT x58 = x32 * x35;
	const GEN_FLT x59 = x58 * x36;
	const GEN_FLT x60 = -x57 + x59;
	const GEN_FLT x61 = x51 + x52;
	const GEN_FLT x62 = -x42 + x43;
	const GEN_FLT x63 = obj_px + x5 * sensor_x + x61 * sensor_z + x62 * sensor_y;
	const GEN_FLT x64 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x65 = x64 * x34;
	const GEN_FLT x66 = x65 * x32;
	const GEN_FLT x67 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x68 = x67 * x31;
	const GEN_FLT x69 = x31 * x36;
	const GEN_FLT x70 = x64 * x69;
	const GEN_FLT x71 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x72 = x71 * x35;
	const GEN_FLT x73 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x74 = x73 * x35;
	const GEN_FLT x75 = x70 * x37 + x72 * x37 + x74 * x36;
	const GEN_FLT x76 = x65 * x37;
	const GEN_FLT x77 = x73 * x31;
	const GEN_FLT x78 = x67 * x35;
	const GEN_FLT x79 = x70 * x32 + x72 * x32 + x78 * x36;
	const GEN_FLT x80 = x1 + x2 * x54;
	const GEN_FLT x81 = x20 * x12;
	const GEN_FLT x82 = x17 + x81;
	const GEN_FLT x83 = obj_pz + x53 * sensor_x + x80 * sensor_z + x82 * sensor_y;
	const GEN_FLT x84 = x64 * x31;
	const GEN_FLT x85 = -x84;
	const GEN_FLT x86 = x1 + x2 * x48;
	const GEN_FLT x87 = -x17 + x81;
	const GEN_FLT x88 = obj_py + x44 * sensor_x + x86 * sensor_y + x87 * sensor_z;
	const GEN_FLT x89 = x63 * (x66 + x68 + x75) + x83 * (-x76 - x77 + x79) + x88 * (x85 + 2 * x72 * x36 + x84 * x40);
	const GEN_FLT x90 = x89 + x39 * x29 + x50 * x41 + x60 * x56;
	const GEN_FLT x91 = pow(x32, 2);
	const GEN_FLT x92 = x34 + x91 * x35;
	const GEN_FLT x93 = x57 + x59;
	const GEN_FLT x94 = x58 * x37;
	const GEN_FLT x95 = -x69 + x94;
	const GEN_FLT x96 = lh_pz + x63 * x95 + x83 * x92 + x88 * x93;
	const GEN_FLT x97 = pow(x96, -1);
	const GEN_FLT x98 = x65 * x36;
	const GEN_FLT x99 = x71 * x31;
	const GEN_FLT x100 = x74 * x32 + x78 * x37 + x64 * x57 * x32;
	const GEN_FLT x101 = x63 * (x100 - x98 - x99) + x83 * (x85 + 2 * x78 * x32 + x84 * x91) + x88 * (x76 + x77 + x79);
	const GEN_FLT x102 = x101 + x50 * x93 + x56 * x92 + x95 * x29;
	const GEN_FLT x103 = pow(x96, 2);
	const GEN_FLT x104 = pow(x103, -1);
	const GEN_FLT x105 = lh_py + x63 * x39 + x83 * x60 + x88 * x41;
	const GEN_FLT x106 = x105 * x104;
	const GEN_FLT x107 = x103 + pow(x105, 2);
	const GEN_FLT x108 = pow(x107, -1);
	const GEN_FLT x109 = x103 * x108;
	const GEN_FLT x110 = 2 * x105;
	const GEN_FLT x111 = 2 * x96;
	const GEN_FLT x112 = x69 + x94;
	const GEN_FLT x113 = -x33 + x38;
	const GEN_FLT x114 = pow(x37, 2);
	const GEN_FLT x115 = x34 + x35 * x114;
	const GEN_FLT x116 = lh_px + x63 * x115 + x83 * x112 + x88 * x113;
	const GEN_FLT x117 = (1.0 / 2.0) * x116 * tilt_1 / pow(x107, 3.0 / 2.0);
	const GEN_FLT x118 = x63 * (x85 + 2 * x74 * x37 + x84 * x114) + x83 * (x100 + x98 + x99) + x88 * (-x66 - x68 + x75);
	const GEN_FLT x119 = x118 + x29 * x115 + x50 * x113 + x56 * x112;
	const GEN_FLT x120 = tilt_1 / sqrt(x107);
	const GEN_FLT x121 = pow(x116, 2);
	const GEN_FLT x122 = pow(1 - x108 * x121 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x123 = -x109 * (-x102 * x106 + x90 * x97) - x122 * (-x117 * (x102 * x111 + x90 * x110) + x119 * x120);
	const GEN_FLT x124 = -x96;
	const GEN_FLT x125 =
		sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x116 * x120) - atan2(-x105, x124)) * gibMag_1;
	const GEN_FLT x126 = x104 * x116;
	const GEN_FLT x127 = 2 * x103 * atan2(x116, x124) * curve_1 / (x103 + x121);
	const GEN_FLT x128 = x28 + x62;
	const GEN_FLT x129 = x49 + x86;
	const GEN_FLT x130 = x55 + x82;
	const GEN_FLT x131 = x89 + x39 * x128 + x41 * x129 + x60 * x130;
	const GEN_FLT x132 = x101 + x92 * x130 + x93 * x129 + x95 * x128;
	const GEN_FLT x133 = x118 + x112 * x130 + x113 * x129 + x115 * x128;
	const GEN_FLT x134 =
		-x109 * (-x106 * x132 + x97 * x131) - x122 * (x120 * x133 - (x110 * x131 + x111 * x132) * x117);
	const GEN_FLT x135 = x28 + x61;
	const GEN_FLT x136 = x49 + x87;
	const GEN_FLT x137 = x55 + x80;
	const GEN_FLT x138 = x89 + x39 * x135 + x41 * x136 + x60 * x137;
	const GEN_FLT x139 = x101 + x92 * x137 + x93 * x136 + x95 * x135;
	const GEN_FLT x140 = x118 + x112 * x137 + x113 * x136 + x115 * x135;
	const GEN_FLT x141 =
		-x109 * (-x106 * x139 + x97 * x138) - x122 * (x120 * x140 - (x110 * x138 + x111 * x139) * x117);
	out[0] = x123 + x123 * x125 + x127 * (x102 * x126 - x97 * x119);
	out[1] = x134 + x125 * x134 + x127 * (x126 * x132 - x97 * x133);
	out[2] = x141 + x125 * x141 + x127 * (x126 * x139 - x97 * x140);
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = sin(x0);
	const GEN_FLT x2 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = -x3;
	const GEN_FLT x5 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x6 = pow(x5, 2);
	const GEN_FLT x7 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
							   pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										: (1e-10)),
								   2))
							: (0));
	const GEN_FLT x8 = cos(x0);
	const GEN_FLT x9 = 1 - x8;
	const GEN_FLT x10 = x5 * x9;
	const GEN_FLT x11 = 2 * x10;
	const GEN_FLT x12 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							 : (1e-10));
	const GEN_FLT x13 = cos(x12);
	const GEN_FLT x14 = 1 - x13;
	const GEN_FLT x15 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x16 = pow(x15, 2);
	const GEN_FLT x17 = sin(x12);
	const GEN_FLT x18 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x21 = x15 * x14;
	const GEN_FLT x22 = x20 * x21;
	const GEN_FLT x23 = x20 * x17;
	const GEN_FLT x24 = x14 * x18;
	const GEN_FLT x25 = x24 * x15;
	const GEN_FLT x26 = obj_py + (x13 + x14 * x16) * sensor_y + (-x19 + x22) * sensor_z + (x23 + x25) * sensor_x;
	const GEN_FLT x27 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (1));
	const GEN_FLT x28 = x2 * x8;
	const GEN_FLT x29 = x28 * x27;
	const GEN_FLT x30 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x31 = x1 * x30;
	const GEN_FLT x32 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x35 = x9 * x32;
	const GEN_FLT x36 = x3 * x33 + x34 * x10 + x7 * x35;
	const GEN_FLT x37 = pow(x20, 2);
	const GEN_FLT x38 = x15 * x17;
	const GEN_FLT x39 = x24 * x20;
	const GEN_FLT x40 = obj_pz + (x13 + x37 * x14) * sensor_z + (x19 + x22) * sensor_y + (-x38 + x39) * sensor_x;
	const GEN_FLT x41 = pow(x18, 2);
	const GEN_FLT x42 = obj_px + (x13 + x41 * x14) * sensor_x + (-x23 + x25) * sensor_y + (x38 + x39) * sensor_z;
	const GEN_FLT x43 = x8 * x32;
	const GEN_FLT x44 = x2 * x43;
	const GEN_FLT x45 = x1 * x34;
	const GEN_FLT x46 = x5 * x27;
	const GEN_FLT x47 = x9 * x27;
	const GEN_FLT x48 = x3 * x46 + x30 * x10 + x7 * x47;
	const GEN_FLT x49 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x50 = x49 * x17;
	const GEN_FLT x51 = -x50;
	const GEN_FLT x52 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x53 = x49 * x13;
	const GEN_FLT x54 = x53 * x20;
	const GEN_FLT x55 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x56 = x55 * x17;
	const GEN_FLT x57 = x49 * x18;
	const GEN_FLT x58 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x59 = x52 * x21 + x57 * x38 + x58 * x24;
	const GEN_FLT x60 = x53 * x15;
	const GEN_FLT x61 = x58 * x17;
	const GEN_FLT x62 = x20 * x14;
	const GEN_FLT x63 = x55 * x24 + x57 * x23 + x62 * x52;
	const GEN_FLT x64 =
		(x51 + x50 * x41 + 2 * x52 * x24) * sensor_x + (-x54 - x56 + x59) * sensor_y + (x60 + x61 + x63) * sensor_z;
	const GEN_FLT x65 = x1 * x32;
	const GEN_FLT x66 = x5 * x47;
	const GEN_FLT x67 = x65 + x66;
	const GEN_FLT x68 = x8 + x6 * x9;
	const GEN_FLT x69 = x53 * x18;
	const GEN_FLT x70 = x52 * x17;
	const GEN_FLT x71 = x55 * x21 + x62 * x58 + x49 * x38 * x20;
	const GEN_FLT x72 =
		(x51 + x50 * x16 + 2 * x58 * x21) * sensor_y + (x54 + x56 + x59) * sensor_x + (-x69 - x70 + x71) * sensor_z;
	const GEN_FLT x73 =
		(x51 + x50 * x37 + 2 * x62 * x55) * sensor_z + (-x60 - x61 + x63) * sensor_x + (x69 + x70 + x71) * sensor_y;
	const GEN_FLT x74 = x1 * x27;
	const GEN_FLT x75 = x5 * x35;
	const GEN_FLT x76 = -x74 + x75;
	const GEN_FLT x77 = x64 * x67 + x72 * x68 + x73 * x76;
	const GEN_FLT x78 = x77 + x26 * (x4 + x3 * x6 + x7 * x11) + x40 * (-x29 - x31 + x36) + x42 * (x44 + x45 + x48);
	const GEN_FLT x79 = pow(x32, 2);
	const GEN_FLT x80 = x8 + x9 * x79;
	const GEN_FLT x81 = x74 + x75;
	const GEN_FLT x82 = x1 * x5;
	const GEN_FLT x83 = x47 * x32;
	const GEN_FLT x84 = -x82 + x83;
	const GEN_FLT x85 = lh_pz + x80 * x40 + x81 * x26 + x84 * x42;
	const GEN_FLT x86 = pow(x85, -1);
	const GEN_FLT x87 = x86 * x78;
	const GEN_FLT x88 = x5 * x28;
	const GEN_FLT x89 = x1 * x7;
	const GEN_FLT x90 = x32 * x27;
	const GEN_FLT x91 = x3 * x90 + x30 * x35 + x47 * x34;
	const GEN_FLT x92 = 2 * x35;
	const GEN_FLT x93 = x80 * x73 + x81 * x72 + x84 * x64;
	const GEN_FLT x94 = x93 + x26 * (x29 + x31 + x36) + x40 * (x4 + x3 * x79 + x92 * x34) + x42 * (-x88 - x89 + x91);
	const GEN_FLT x95 = pow(x85, 2);
	const GEN_FLT x96 = pow(x95, -1);
	const GEN_FLT x97 = lh_py + x67 * x42 + x68 * x26 + x76 * x40;
	const GEN_FLT x98 = x97 * x96;
	const GEN_FLT x99 = -x98 * x94;
	const GEN_FLT x100 = x95 + pow(x97, 2);
	const GEN_FLT x101 = pow(x100, -1);
	const GEN_FLT x102 = x95 * x101;
	const GEN_FLT x103 = x82 + x83;
	const GEN_FLT x104 = -x65 + x66;
	const GEN_FLT x105 = pow(x27, 2);
	const GEN_FLT x106 = x8 + x9 * x105;
	const GEN_FLT x107 = lh_px + x26 * x104 + x40 * x103 + x42 * x106;
	const GEN_FLT x108 = pow(x107, 2);
	const GEN_FLT x109 = pow(1 - x101 * x108 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x110 = 2 * x97;
	const GEN_FLT x111 = x78 * x110;
	const GEN_FLT x112 = 2 * x85;
	const GEN_FLT x113 = x94 * x112;
	const GEN_FLT x114 = (1.0 / 2.0) * x107 * tilt_1 / pow(x100, 3.0 / 2.0);
	const GEN_FLT x115 = 2 * x47;
	const GEN_FLT x116 = x64 * x106 + x72 * x104 + x73 * x103;
	const GEN_FLT x117 =
		x116 + x26 * (-x44 - x45 + x48) + x40 * (x88 + x89 + x91) + x42 * (x4 + x3 * x105 + x30 * x115);
	const GEN_FLT x118 = 1 + x117;
	const GEN_FLT x119 = tilt_1 / sqrt(x100);
	const GEN_FLT x120 = -x109 * (x118 * x119 - (x111 + x113) * x114) - (x87 + x99) * x102;
	const GEN_FLT x121 = -x85;
	const GEN_FLT x122 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x107 * x119) - atan2(-x97, x121)) * gibMag_1;
	const GEN_FLT x123 = x96 * x107;
	const GEN_FLT x124 = x94 * x123;
	const GEN_FLT x125 = 2 * x95 * atan2(x107, x121) * curve_1 / (x108 + x95);
	const GEN_FLT x126 = 1 + x78;
	const GEN_FLT x127 = x119 * x117;
	const GEN_FLT x128 = -x102 * (x99 + x86 * x126) - x109 * (x127 - x114 * (x113 + x110 * x126));
	const GEN_FLT x129 = -x86 * x117;
	const GEN_FLT x130 = 1 + x94;
	const GEN_FLT x131 = -x102 * (x87 - x98 * x130) - x109 * (x127 - x114 * (x111 + x112 * x130));
	const GEN_FLT x132 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x133 = x43 * x132;
	const GEN_FLT x134 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x135 = x1 * x134;
	const GEN_FLT x136 = x1 * x132;
	const GEN_FLT x137 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x138 = x9 * x137;
	const GEN_FLT x139 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qi / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x140 = x10 * x139 + x27 * x138 + x46 * x136;
	const GEN_FLT x141 = -x136;
	const GEN_FLT x142 = x8 * x132;
	const GEN_FLT x143 = x27 * x142;
	const GEN_FLT x144 = x1 * x139;
	const GEN_FLT x145 = x10 * x134 + x32 * x138 + x33 * x136;
	const GEN_FLT x146 =
		x77 + x26 * (x141 + 2 * x5 * x138 + x6 * x136) + x40 * (-x143 - x144 + x145) + x42 * (x133 + x135 + x140);
	const GEN_FLT x147 = x5 * x142;
	const GEN_FLT x148 = x1 * x137;
	const GEN_FLT x149 = x35 * x139 + x47 * x134 + x90 * x136;
	const GEN_FLT x150 =
		x93 + x26 * (x143 + x144 + x145) + x40 * (x141 + x79 * x136 + x92 * x134) + x42 * (-x147 - x148 + x149);
	const GEN_FLT x151 =
		x116 + x26 * (-x133 - x135 + x140) + x40 * (x147 + x148 + x149) + x42 * (x141 + x105 * x136 + x115 * x139);
	const GEN_FLT x152 = -x109 * (x119 * x151 - (x110 * x146 + x112 * x150) * x114) - (x86 * x146 - x98 * x150) * x102;
	const GEN_FLT x153 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x154 = x43 * x153;
	const GEN_FLT x155 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x156 = x1 * x155;
	const GEN_FLT x157 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x158 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qj / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x159 = x9 * x158;
	const GEN_FLT x160 = x1 * x153;
	const GEN_FLT x161 = x46 * x160 + x47 * x157 + x5 * x159;
	const GEN_FLT x162 = -x160;
	const GEN_FLT x163 = x8 * x153;
	const GEN_FLT x164 = x27 * x163;
	const GEN_FLT x165 = x1 * x158;
	const GEN_FLT x166 = x10 * x155 + x33 * x160 + x35 * x157;
	const GEN_FLT x167 =
		x77 + x26 * (x162 + x11 * x157 + x6 * x160) + x40 * (-x164 - x165 + x166) + x42 * (x154 + x156 + x161);
	const GEN_FLT x168 = x5 * x163;
	const GEN_FLT x169 = x1 * x157;
	const GEN_FLT x170 = x32 * x159 + x47 * x155 + x90 * x160;
	const GEN_FLT x171 =
		x93 + x26 * (x164 + x165 + x166) + x40 * (x162 + x79 * x160 + x92 * x155) + x42 * (-x168 - x169 + x170);
	const GEN_FLT x172 =
		x116 + x26 * (-x154 - x156 + x161) + x40 * (x168 + x169 + x170) + x42 * (x162 + x105 * x160 + x115 * x158);
	const GEN_FLT x173 = -x109 * (x119 * x172 - (x110 * x167 + x112 * x171) * x114) - (x86 * x167 - x98 * x171) * x102;
	const GEN_FLT x174 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
							  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
							  : (0));
	const GEN_FLT x175 = x43 * x174;
	const GEN_FLT x176 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qk *
									 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (0)) /
									 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											  : (1e-10)),
										 2) +
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 -1))
							  : (0));
	const GEN_FLT x177 = x1 * x176;
	const GEN_FLT x178 = x1 * x174;
	const GEN_FLT x179 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qj *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x180 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									: (1e-10)))
							  ? (-lh_qi *
								 ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
									  ? (lh_qk / sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
									  : (0)) /
								 pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										  : (1e-10)),
									 2))
							  : (0));
	const GEN_FLT x181 = x10 * x180 + x46 * x178 + x47 * x179;
	const GEN_FLT x182 = -x178;
	const GEN_FLT x183 = x8 * x174;
	const GEN_FLT x184 = x27 * x183;
	const GEN_FLT x185 = x1 * x180;
	const GEN_FLT x186 = x10 * x176 + x33 * x178 + x35 * x179;
	const GEN_FLT x187 =
		x77 + x26 * (x182 + x11 * x179 + x6 * x178) + x40 * (-x184 - x185 + x186) + x42 * (x175 + x177 + x181);
	const GEN_FLT x188 = x5 * x183;
	const GEN_FLT x189 = x1 * x179;
	const GEN_FLT x190 = x35 * x180 + x47 * x176 + x90 * x178;
	const GEN_FLT x191 =
		x93 + x26 * (x184 + x185 + x186) + x40 * (x182 + x79 * x178 + x92 * x176) + x42 * (-x188 - x189 + x190);
	const GEN_FLT x192 =
		x116 + x26 * (-x175 - x177 + x181) + x40 * (x188 + x189 + x190) + x42 * (x182 + x105 * x178 + x115 * x180);
	const GEN_FLT x193 = -x109 * (x119 * x192 - (x110 * x187 + x112 * x191) * x114) - (x86 * x187 - x98 * x191) * x102;
	out[0] = x120 + x120 * x122 + x125 * (x124 - x86 * x118);
	out[1] = x128 + x122 * x128 + (x124 + x129) * x125;
	out[2] = x131 + x122 * x131 + x125 * (x129 + x123 * x130);
	out[3] = x152 + x122 * x152 + x125 * (x123 * x150 - x86 * x151);
	out[4] = x173 + x122 * x173 + x125 * (x123 * x171 - x86 * x172);
	out[5] = x193 + x122 * x193 + x125 * (x123 * x191 - x86 * x192);
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
	const GEN_FLT x0 =
		((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
															 : (1e-10));
	const GEN_FLT x1 = cos(x0);
	const GEN_FLT x2 = 1 - x1;
	const GEN_FLT x3 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								  ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								  : (1e-10)))
							? (lh_qk / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											: (1e-10)))
							: (0));
	const GEN_FLT x4 = pow(x3, 2);
	const GEN_FLT x5 = x1 + x2 * x4;
	const GEN_FLT x6 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
							? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
							: (1e-10));
	const GEN_FLT x7 = cos(x6);
	const GEN_FLT x8 = 1 - x7;
	const GEN_FLT x9 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								  : (1e-10)))
							? (obj_qk / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											 : (1e-10)))
							: (0));
	const GEN_FLT x10 = pow(x9, 2);
	const GEN_FLT x11 = sin(x6);
	const GEN_FLT x12 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qi / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (1));
	const GEN_FLT x13 = x12 * x11;
	const GEN_FLT x14 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (obj_qj / ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
											  ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
											  : (1e-10)))
							 : (0));
	const GEN_FLT x15 = x8 * x9;
	const GEN_FLT x16 = x15 * x14;
	const GEN_FLT x17 = x14 * x11;
	const GEN_FLT x18 = x8 * x12;
	const GEN_FLT x19 = x9 * x18;
	const GEN_FLT x20 = obj_pz + (x13 + x16) * sensor_y + (-x17 + x19) * sensor_x + (x7 + x8 * x10) * sensor_z;
	const GEN_FLT x21 = sin(x0);
	const GEN_FLT x22 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qi / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (1));
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (lh_qj / ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
											 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
											 : (1e-10)))
							 : (0));
	const GEN_FLT x25 = x2 * x3;
	const GEN_FLT x26 = x24 * x25;
	const GEN_FLT x27 = x23 + x26;
	const GEN_FLT x28 = pow(x14, 2);
	const GEN_FLT x29 = x9 * x11;
	const GEN_FLT x30 = x14 * x18;
	const GEN_FLT x31 = obj_py + (-x13 + x16) * sensor_z + (x29 + x30) * sensor_x + (x7 + x8 * x28) * sensor_y;
	const GEN_FLT x32 = x24 * x21;
	const GEN_FLT x33 = x25 * x22;
	const GEN_FLT x34 = -x32 + x33;
	const GEN_FLT x35 = pow(x12, 2);
	const GEN_FLT x36 = obj_px + (x17 + x19) * sensor_z + (-x29 + x30) * sensor_y + (x7 + x8 * x35) * sensor_x;
	const GEN_FLT x37 = lh_pz + x31 * x27 + x34 * x36 + x5 * x20;
	const GEN_FLT x38 = pow(x37, 2);
	const GEN_FLT x39 = x32 + x33;
	const GEN_FLT x40 = x3 * x21;
	const GEN_FLT x41 = x2 * x24;
	const GEN_FLT x42 = x41 * x22;
	const GEN_FLT x43 = -x40 + x42;
	const GEN_FLT x44 = pow(x22, 2);
	const GEN_FLT x45 = x1 + x2 * x44;
	const GEN_FLT x46 = lh_px + x39 * x20 + x43 * x31 + x45 * x36;
	const GEN_FLT x47 = -x37;
	const GEN_FLT x48 = atan2(x46, x47);
	const GEN_FLT x49 = pow(x46, 2);
	const GEN_FLT x50 = ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0));
	const GEN_FLT x51 = x50 * x21;
	const GEN_FLT x52 = -x51;
	const GEN_FLT x53 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qi * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x54 = x2 * x22;
	const GEN_FLT x55 = ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0));
	const GEN_FLT x56 = x55 * x11;
	const GEN_FLT x57 = -x56;
	const GEN_FLT x58 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qi * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x59 = x7 * x55;
	const GEN_FLT x60 = x9 * x59;
	const GEN_FLT x61 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qk * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x62 = x61 * x11;
	const GEN_FLT x63 = x55 * x14;
	const GEN_FLT x64 = x8 * x14;
	const GEN_FLT x65 = ((0 < ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
								   ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
								   : (1e-10)))
							 ? (-obj_qj * ((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2))
										 ? (sqrt(pow(obj_qi, 2) + pow(obj_qj, 2) + pow(obj_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x66 = x63 * x13 + x64 * x58 + x65 * x18;
	const GEN_FLT x67 = x59 * x14;
	const GEN_FLT x68 = x65 * x11;
	const GEN_FLT x69 = x58 * x15 + x61 * x18 + x9 * x55 * x13;
	const GEN_FLT x70 =
		(x57 + x56 * x35 + 2 * x58 * x18) * sensor_x + (-x60 - x62 + x66) * sensor_y + (x67 + x68 + x69) * sensor_z;
	const GEN_FLT x71 = x59 * x12;
	const GEN_FLT x72 = x58 * x11;
	const GEN_FLT x73 = x63 * x29 + x64 * x61 + x65 * x15;
	const GEN_FLT x74 =
		(x57 + x56 * x28 + 2 * x64 * x65) * sensor_y + (x60 + x62 + x66) * sensor_x + (-x71 - x72 + x73) * sensor_z;
	const GEN_FLT x75 = x1 * x50;
	const GEN_FLT x76 = x3 * x75;
	const GEN_FLT x77 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qk * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x78 = x77 * x21;
	const GEN_FLT x79 = ((0 < ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
								   ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
								   : (1e-10)))
							 ? (-lh_qj * ((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)) ? (0) : (0)) /
								pow(((0 < pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2))
										 ? (sqrt(pow(lh_qi, 2) + pow(lh_qj, 2) + pow(lh_qk, 2)))
										 : (1e-10)),
									2))
							 : (0));
	const GEN_FLT x80 = x2 * x79;
	const GEN_FLT x81 = x53 * x41 + x80 * x22 + x51 * x24 * x22;
	const GEN_FLT x82 = x75 * x24;
	const GEN_FLT x83 = x79 * x21;
	const GEN_FLT x84 = x50 * x40;
	const GEN_FLT x85 = x53 * x25 + x77 * x54 + x84 * x22;
	const GEN_FLT x86 =
		(x57 + x56 * x10 + 2 * x61 * x15) * sensor_z + (-x67 - x68 + x69) * sensor_x + (x71 + x72 + x73) * sensor_y;
	const GEN_FLT x87 = x20 * (x82 + x83 + x85) + x31 * (-x76 - x78 + x81) + x36 * (x52 + x51 * x44 + 2 * x54 * x53) +
						x70 * x45 + x74 * x43 + x86 * x39;
	const GEN_FLT x88 = pow(x37, -1);
	const GEN_FLT x89 = x75 * x22;
	const GEN_FLT x90 = x53 * x21;
	const GEN_FLT x91 = x3 * x80 + x77 * x41 + x84 * x24;
	const GEN_FLT x92 = x20 * (x52 + x4 * x51 + 2 * x77 * x25) + x31 * (x89 + x90 + x91) + x36 * (-x82 - x83 + x85) +
						x5 * x86 + x70 * x34 + x74 * x27;
	const GEN_FLT x93 = x92 / x38;
	const GEN_FLT x94 = 2 * (-x88 * x87 + x93 * x46) * x48 * x38 * curve_1 / (x38 + x49);
	const GEN_FLT x95 = x40 + x42;
	const GEN_FLT x96 = pow(x24, 2);
	const GEN_FLT x97 = x1 + x2 * x96;
	const GEN_FLT x98 = -x23 + x26;
	const GEN_FLT x99 = x20 * (-x89 - x90 + x91) + x31 * (x52 + x51 * x96 + 2 * x80 * x24) + x36 * (x76 + x78 + x81) +
						x70 * x95 + x74 * x97 + x86 * x98;
	const GEN_FLT x100 = lh_py + x95 * x36 + x97 * x31 + x98 * x20;
	const GEN_FLT x101 = x38 + pow(x100, 2);
	const GEN_FLT x102 = pow(x101, -1);
	const GEN_FLT x103 = -x38 * x102 * (x88 * x99 - x93 * x100);
	const GEN_FLT x104 = pow(1 - x49 * x102 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x105 = pow(x101, -1.0 / 2.0);
	const GEN_FLT x106 =
		x87 * x105 * tilt_1 + (-1.0 / 2.0) * x46 * tilt_1 * (2 * x92 * x37 + 2 * x99 * x100) / pow(x101, 3.0 / 2.0);
	const GEN_FLT x107 = x103 - x104 * x106;
	const GEN_FLT x108 = -1 + x107;
	const GEN_FLT x109 = x46 * x105;
	const GEN_FLT x110 = 1.5707963267949 + gibPhase_1 - phase_1 - asin(x109 * tilt_1) - atan2(-x100, x47);
	const GEN_FLT x111 = sin(x110) * gibMag_1;
	const GEN_FLT x112 = x103 - (x106 + x109) * x104;
	const GEN_FLT x113 = x107 + x94;
	const GEN_FLT x114 = x113 + x107 * x111;
	out[0] = x108 + x94 + x108 * x111;
	out[1] = x112 + x94 + x111 * x112;
	out[2] = x114 + pow(x48, 2);
	out[3] = x113 + x111 * (1 + x107);
	out[4] = x114 - cos(x110);
	out[5] = x114;
	out[6] = x114;
}

/** Applying function <function reproject_gen2 at 0x7effc26e3b90> */
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
	const GEN_FLT x4 = pow(obj_qj, 2);
	const GEN_FLT x5 = pow(obj_qi, 2);
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = 2 * sqrt(x6 + x7 + pow(obj_qw, 2));
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = obj_pz + x11 * (x10 + x9) + (1 - x6 * x8) * sensor_z + (-x12 + x13) * x14;
	const GEN_FLT x16 = pow(lh_qi, 2);
	const GEN_FLT x17 = pow(lh_qk, 2);
	const GEN_FLT x18 = pow(lh_qj, 2);
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 2 * sqrt(x16 + x19 + pow(lh_qw, 2));
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + x22 * (x10 - x9) + (1 - (x5 + x7) * x8) * sensor_y + (x23 + x24) * x14;
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 = obj_px + (1 - (x4 + x7) * x8) * sensor_x + (x12 + x13) * x22 + (-x23 + x24) * x11;
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = lh_py + x25 * (1 - (x16 + x17) * x20) + (-x2 + x3) * x21 + (x26 + x27) * x29;
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = lh_pz + x15 * (1 - (x16 + x18) * x20) + (x2 + x3) * x31 + (-x32 + x33) * x29;
	const GEN_FLT x35 = lh_px + x28 * (1 - x20 * x19) + (-x26 + x27) * x31 + (x32 + x33) * x21;
	const GEN_FLT x36 = pow(x34, 2) + pow(x35, 2);
	const GEN_FLT x37 = x30 / sqrt(x36 + pow(x30, 2));
	const GEN_FLT x38 = asin(x37 / x1);
	const GEN_FLT x39 = 0.0028679863 + x38 * (-8.0108022e-06 - 8.0108022e-06 * x38);
	const GEN_FLT x40 = 5.3685255e-06 + x38 * x39;
	const GEN_FLT x41 = 0.0076069798 + x40 * x38;
	const GEN_FLT x42 = x30 / sqrt(x36);
	const GEN_FLT x43 = tan(x0) * x42;
	const GEN_FLT x44 = atan2(-x34, x35);
	const GEN_FLT x45 = curve_0 + sin(ogeeMag_0 + x44 - asin(x43)) * ogeePhase_0;
	const GEN_FLT x46 = asin(
		x43 + x41 * x45 * pow(x38, 2) /
				  (x1 - sin(x0) * x45 *
							(x38 * (x41 + x38 * (x40 + x38 * (x39 + x38 * (-8.0108022e-06 - 1.60216044e-05 * x38)))) +
							 x41 * x38)));
	const GEN_FLT x47 = -x44;
	const GEN_FLT x48 = -1.5707963267949 + x44;
	const GEN_FLT x49 = 0.523598775598299 - tilt_1;
	const GEN_FLT x50 = -x42 * tan(x49);
	const GEN_FLT x51 = curve_1 + sin(ogeeMag_1 + x44 - asin(x50)) * ogeePhase_1;
	const GEN_FLT x52 = cos(x49);
	const GEN_FLT x53 = asin(x37 / x52);
	const GEN_FLT x54 = 0.0028679863 + x53 * (-8.0108022e-06 - 8.0108022e-06 * x53);
	const GEN_FLT x55 = 5.3685255e-06 + x54 * x53;
	const GEN_FLT x56 = 0.0076069798 + x53 * x55;
	const GEN_FLT x57 =
		asin(x50 +
			 pow(x53, 2) * x51 * x56 /
				 (x52 + x51 * sin(x49) *
							(x53 * x56 +
							 x53 * (x56 + x53 * (x55 + x53 * (x54 + x53 * (-8.0108022e-06 - 1.60216044e-05 * x53)))))));
	out[0] = -phase_0 - x46 + x48 - sin(-gibPhase_0 + x46 + x47) * gibMag_0;
	out[1] = -phase_1 + x48 - x57 - sin(-gibPhase_1 + x47 + x57) * gibMag_1;
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
	const GEN_FLT x2 = -x0 + x1;
	const GEN_FLT x3 = pow(obj_qj, 2);
	const GEN_FLT x4 = pow(obj_qi, 2);
	const GEN_FLT x5 = x3 + x4;
	const GEN_FLT x6 = pow(obj_qk, 2);
	const GEN_FLT x7 = x4 + x6;
	const GEN_FLT x8 = sqrt(x3 + x7 + pow(obj_qw, 2));
	const GEN_FLT x9 = 2 * x8;
	const GEN_FLT x10 = obj_qw * obj_qi;
	const GEN_FLT x11 = obj_qk * obj_qj;
	const GEN_FLT x12 = x10 + x11;
	const GEN_FLT x13 = x9 * sensor_y;
	const GEN_FLT x14 = obj_qw * obj_qj;
	const GEN_FLT x15 = obj_qk * obj_qi;
	const GEN_FLT x16 = -x14 + x15;
	const GEN_FLT x17 = x9 * sensor_x;
	const GEN_FLT x18 = obj_pz + x13 * x12 + x17 * x16 + (1 - x5 * x9) * sensor_z;
	const GEN_FLT x19 = pow(lh_qj, 2);
	const GEN_FLT x20 = pow(lh_qk, 2);
	const GEN_FLT x21 = pow(lh_qi, 2);
	const GEN_FLT x22 = x20 + x21;
	const GEN_FLT x23 = sqrt(x19 + x22 + pow(lh_qw, 2));
	const GEN_FLT x24 = 2 * x23;
	const GEN_FLT x25 = x24 * x18;
	const GEN_FLT x26 = 1 - x24 * x22;
	const GEN_FLT x27 = -x10 + x11;
	const GEN_FLT x28 = x9 * sensor_z;
	const GEN_FLT x29 = obj_qw * obj_qk;
	const GEN_FLT x30 = obj_qj * obj_qi;
	const GEN_FLT x31 = x29 + x30;
	const GEN_FLT x32 = obj_py + x28 * x27 + x31 * x17 + (1 - x7 * x9) * sensor_y;
	const GEN_FLT x33 = x14 + x15;
	const GEN_FLT x34 = -x29 + x30;
	const GEN_FLT x35 = x3 + x6;
	const GEN_FLT x36 = obj_px + x33 * x28 + x34 * x13 + (1 - x9 * x35) * sensor_x;
	const GEN_FLT x37 = lh_qw * lh_qk;
	const GEN_FLT x38 = lh_qj * lh_qi;
	const GEN_FLT x39 = x37 + x38;
	const GEN_FLT x40 = x39 * x24;
	const GEN_FLT x41 = lh_py + x2 * x25 + x32 * x26 + x40 * x36;
	const GEN_FLT x42 = pow(x41, 2);
	const GEN_FLT x43 = 1 - (x19 + x21) * x24;
	const GEN_FLT x44 = x0 + x1;
	const GEN_FLT x45 = x44 * x24;
	const GEN_FLT x46 = lh_qw * lh_qj;
	const GEN_FLT x47 = lh_qk * lh_qi;
	const GEN_FLT x48 = -x46 + x47;
	const GEN_FLT x49 = x48 * x24;
	const GEN_FLT x50 = lh_pz + x43 * x18 + x45 * x32 + x49 * x36;
	const GEN_FLT x51 = x46 + x47;
	const GEN_FLT x52 = -x37 + x38;
	const GEN_FLT x53 = x52 * x24;
	const GEN_FLT x54 = 1 - (x19 + x20) * x24;
	const GEN_FLT x55 = lh_px + x51 * x25 + x53 * x32 + x54 * x36;
	const GEN_FLT x56 = pow(x55, 2);
	const GEN_FLT x57 = x56 + pow(x50, 2);
	const GEN_FLT x58 = x42 + x57;
	const GEN_FLT x59 = pow(x58, -1.0 / 2.0);
	const GEN_FLT x60 = 0.523598775598299 + tilt_0;
	const GEN_FLT x61 = cos(x60);
	const GEN_FLT x62 = pow(x61, -1);
	const GEN_FLT x63 = x62 * x59;
	const GEN_FLT x64 = asin(x63 * x41);
	const GEN_FLT x65 = 8.0108022e-06 * x64;
	const GEN_FLT x66 = -8.0108022e-06 - x65;
	const GEN_FLT x67 = 0.0028679863 + x64 * x66;
	const GEN_FLT x68 = 5.3685255e-06 + x64 * x67;
	const GEN_FLT x69 = 0.0076069798 + x64 * x68;
	const GEN_FLT x70 = x64 * x69;
	const GEN_FLT x71 = -8.0108022e-06 - 1.60216044e-05 * x64;
	const GEN_FLT x72 = x67 + x71 * x64;
	const GEN_FLT x73 = x68 + x72 * x64;
	const GEN_FLT x74 = x69 + x73 * x64;
	const GEN_FLT x75 = x70 + x74 * x64;
	const GEN_FLT x76 = sin(x60);
	const GEN_FLT x77 = tan(x60);
	const GEN_FLT x78 = pow(x57, -1.0 / 2.0);
	const GEN_FLT x79 = x78 * x41;
	const GEN_FLT x80 = x79 * x77;
	const GEN_FLT x81 = atan2(-x50, x55);
	const GEN_FLT x82 = ogeeMag_0 + x81 - asin(x80);
	const GEN_FLT x83 = curve_0 + sin(x82) * ogeePhase_0;
	const GEN_FLT x84 = x83 * x76;
	const GEN_FLT x85 = x61 - x84 * x75;
	const GEN_FLT x86 = pow(x85, -1);
	const GEN_FLT x87 = pow(x64, 2);
	const GEN_FLT x88 = x83 * x87;
	const GEN_FLT x89 = x88 * x86;
	const GEN_FLT x90 = x80 + x89 * x69;
	const GEN_FLT x91 = pow(1 - pow(x90, 2), -1.0 / 2.0);
	const GEN_FLT x92 = x42 / x58;
	const GEN_FLT x93 = pow(1 - x92 / pow(x61, 2), -1.0 / 2.0);
	const GEN_FLT x94 = 4 * x23;
	const GEN_FLT x95 = x94 * x41;
	const GEN_FLT x96 = 2 * x55;
	const GEN_FLT x97 = x50 * x94;
	const GEN_FLT x98 = x54 * x96 + x97 * x48;
	const GEN_FLT x99 = x98 + x95 * x39;
	const GEN_FLT x100 = (1.0 / 2.0) * x41;
	const GEN_FLT x101 = x100 / pow(x58, 3.0 / 2.0);
	const GEN_FLT x102 = x62 * x101;
	const GEN_FLT x103 = x63 * x40 - x99 * x102;
	const GEN_FLT x104 = x93 * x103;
	const GEN_FLT x105 = x66 * x104;
	const GEN_FLT x106 = x64 * (x105 - x65 * x104) + x67 * x104;
	const GEN_FLT x107 = x64 * x106 + x68 * x104;
	const GEN_FLT x108 = pow(x56, -1);
	const GEN_FLT x109 = x50 * x108;
	const GEN_FLT x110 = pow(x55, -1);
	const GEN_FLT x111 = pow(x57, -1);
	const GEN_FLT x112 = x56 * x111;
	const GEN_FLT x113 = (-x49 * x110 + x54 * x109) * x112;
	const GEN_FLT x114 = x42 * x111;
	const GEN_FLT x115 = pow(1 - pow(x77, 2) * x114, -1.0 / 2.0);
	const GEN_FLT x116 = x100 / pow(x57, 3.0 / 2.0);
	const GEN_FLT x117 = x77 * x116;
	const GEN_FLT x118 = x78 * x40;
	const GEN_FLT x119 = x77 * x118 - x98 * x117;
	const GEN_FLT x120 = x113 - x119 * x115;
	const GEN_FLT x121 = cos(x82) * ogeePhase_0;
	const GEN_FLT x122 = x75 * x76;
	const GEN_FLT x123 = x122 * x121;
	const GEN_FLT x124 = 2.40324066e-05 * x64;
	const GEN_FLT x125 = x73 * x93;
	const GEN_FLT x126 = x88 * x69 / pow(x85, 2);
	const GEN_FLT x127 = x86 * x87 * x69;
	const GEN_FLT x128 = x121 * x127;
	const GEN_FLT x129 = 2 * x83 * x86 * x70;
	const GEN_FLT x130 =
		x91 *
		(x119 + x104 * x129 + x120 * x128 -
		 x126 * (-x120 * x123 - x84 * (x64 * x107 +
									   x64 * (x107 + x103 * x125 +
											  x64 * (x106 + x64 * (x105 - x104 * x124 + x71 * x104) + x72 * x104)) +
									   x69 * x104 + x74 * x104)) +
		 x89 * x107);
	const GEN_FLT x131 = -x113;
	const GEN_FLT x132 = -x81;
	const GEN_FLT x133 = cos(-gibPhase_0 + x132 + asin(x90)) * gibMag_0;
	const GEN_FLT x134 = 2 * x50;
	const GEN_FLT x135 = x23 * x108 * x134;
	const GEN_FLT x136 = (-x45 * x110 + x52 * x135) * x112;
	const GEN_FLT x137 = 2 * x41;
	const GEN_FLT x138 = x55 * x94;
	const GEN_FLT x139 = x52 * x138 + x97 * x44;
	const GEN_FLT x140 = x139 + x26 * x137;
	const GEN_FLT x141 = -x102 * x140 + x63 * x26;
	const GEN_FLT x142 = x93 * x141;
	const GEN_FLT x143 = x66 * x142;
	const GEN_FLT x144 = x64 * (x143 - x65 * x142) + x67 * x142;
	const GEN_FLT x145 = x64 * x144 + x68 * x142;
	const GEN_FLT x146 = x116 * x139;
	const GEN_FLT x147 = x78 * x26;
	const GEN_FLT x148 = -x77 * x146 + x77 * x147;
	const GEN_FLT x149 = x121 * (x136 - x115 * x148);
	const GEN_FLT x150 =
		x91 *
		(x148 -
		 x126 * (-x122 * x149 - x84 * (x64 * x145 +
									   x64 * (x145 + x125 * x141 +
											  x64 * (x144 + x64 * (x143 - x124 * x142 + x71 * x142) + x72 * x142)) +
									   x69 * x142 + x74 * x142)) +
		 x127 * x149 + x129 * x142 + x89 * x145);
	const GEN_FLT x151 = -x136;
	const GEN_FLT x152 = (-x43 * x110 + x51 * x135) * x112;
	const GEN_FLT x153 = x43 * x134 + x51 * x138;
	const GEN_FLT x154 = x101 * (x153 + x2 * x95);
	const GEN_FLT x155 = x2 * x24;
	const GEN_FLT x156 = -x62 * x154 + x63 * x155;
	const GEN_FLT x157 = x93 * x156;
	const GEN_FLT x158 = x66 * x157;
	const GEN_FLT x159 = x64 * (x158 - x65 * x157) + x67 * x157;
	const GEN_FLT x160 = x64 * x159 + x68 * x157;
	const GEN_FLT x161 = x78 * x155;
	const GEN_FLT x162 = -x117 * x153 + x77 * x161;
	const GEN_FLT x163 = x152 - x115 * x162;
	const GEN_FLT x164 =
		x91 *
		(x162 -
		 x126 * (-x123 * x163 - x84 * (x64 * x160 +
									   x64 * (x160 + x125 * x156 +
											  x64 * (x159 + x64 * (x158 - x124 * x157 + x71 * x157) + x72 * x157)) +
									   x69 * x157 + x74 * x157)) +
		 x128 * x163 + x129 * x157 + x89 * x160);
	const GEN_FLT x165 = -x152;
	const GEN_FLT x166 = 2 / x8;
	const GEN_FLT x167 = x35 * x166;
	const GEN_FLT x168 = x167 * sensor_x;
	const GEN_FLT x169 = x166 * obj_qw;
	const GEN_FLT x170 = x34 * sensor_y;
	const GEN_FLT x171 = x13 * obj_qk;
	const GEN_FLT x172 = x33 * sensor_z;
	const GEN_FLT x173 = x28 * obj_qj;
	const GEN_FLT x174 = -x171 + x173 - x168 * obj_qw + x169 * x170 + x169 * x172;
	const GEN_FLT x175 = x31 * sensor_x;
	const GEN_FLT x176 = x7 * x166;
	const GEN_FLT x177 = x176 * sensor_y;
	const GEN_FLT x178 = x17 * obj_qk;
	const GEN_FLT x179 = x27 * sensor_z;
	const GEN_FLT x180 = x28 * obj_qi;
	const GEN_FLT x181 = x178 - x180 + x169 * x175 + x169 * x179 - x177 * obj_qw;
	const GEN_FLT x182 = x17 * obj_qj;
	const GEN_FLT x183 = x16 * sensor_x;
	const GEN_FLT x184 = x12 * sensor_y;
	const GEN_FLT x185 = x166 * x184;
	const GEN_FLT x186 = x13 * obj_qi;
	const GEN_FLT x187 = x5 * x166;
	const GEN_FLT x188 = x187 * sensor_z;
	const GEN_FLT x189 = -x182 + x186 + x169 * x183 + x185 * obj_qw - x188 * obj_qw;
	const GEN_FLT x190 = x189 * x155 + x26 * x181 + x40 * x174;
	const GEN_FLT x191 = x51 * x24;
	const GEN_FLT x192 = x189 * x191 + x53 * x181 + x54 * x174;
	const GEN_FLT x193 = x43 * x189 + x45 * x181 + x49 * x174;
	const GEN_FLT x194 = x193 * x134 + x96 * x192;
	const GEN_FLT x195 = x194 + x190 * x137;
	const GEN_FLT x196 = -x102 * x195 + x63 * x190;
	const GEN_FLT x197 = x93 * x196;
	const GEN_FLT x198 = x66 * x197;
	const GEN_FLT x199 = x64 * (x198 - x65 * x197) + x67 * x197;
	const GEN_FLT x200 = x64 * x199 + x68 * x197;
	const GEN_FLT x201 = (x109 * x192 - x110 * x193) * x112;
	const GEN_FLT x202 = x78 * x190;
	const GEN_FLT x203 = -x117 * x194 + x77 * x202;
	const GEN_FLT x204 = x201 - x203 * x115;
	const GEN_FLT x205 =
		x91 *
		(x203 -
		 x126 * (-x204 * x123 - x84 * (x64 * x200 +
									   x64 * (x200 + x125 * x196 +
											  x64 * (x199 + x64 * (x198 - x124 * x197 + x71 * x197) + x72 * x197)) +
									   x69 * x197 + x74 * x197)) +
		 x129 * x197 + x204 * x128 + x89 * x200);
	const GEN_FLT x206 = -x201;
	const GEN_FLT x207 = x166 * obj_qi;
	const GEN_FLT x208 = x13 * obj_qj;
	const GEN_FLT x209 = x28 * obj_qk;
	const GEN_FLT x210 = x208 + x209 - x168 * obj_qi + x207 * x170 + x207 * x172;
	const GEN_FLT x211 = 4 * x8;
	const GEN_FLT x212 = -x211 * obj_qi;
	const GEN_FLT x213 = x28 * obj_qw;
	const GEN_FLT x214 = x182 - x213 + x207 * x175 + x207 * x179 + (x212 - x176 * obj_qi) * sensor_y;
	const GEN_FLT x215 = x13 * obj_qw;
	const GEN_FLT x216 = x178 + x215 + x185 * obj_qi + x207 * x183 + (x212 - x187 * obj_qi) * sensor_z;
	const GEN_FLT x217 = x216 * x191 + x53 * x214 + x54 * x210;
	const GEN_FLT x218 = x43 * x216 + x45 * x214 + x49 * x210;
	const GEN_FLT x219 = (x217 * x109 - x218 * x110) * x112;
	const GEN_FLT x220 = x216 * x155 + x26 * x214 + x40 * x210;
	const GEN_FLT x221 = x218 * x134 + x96 * x217;
	const GEN_FLT x222 = x221 + x220 * x137;
	const GEN_FLT x223 = -x222 * x102 + x63 * x220;
	const GEN_FLT x224 = x93 * x223;
	const GEN_FLT x225 = x66 * x224;
	const GEN_FLT x226 = x64 * (x225 - x65 * x224) + x67 * x224;
	const GEN_FLT x227 = x64 * x226 + x68 * x224;
	const GEN_FLT x228 = x78 * x220;
	const GEN_FLT x229 = -x221 * x117 + x77 * x228;
	const GEN_FLT x230 = x219 - x229 * x115;
	const GEN_FLT x231 =
		x91 *
		(x229 -
		 x126 * (-x230 * x123 - x84 * (x64 * x227 +
									   x64 * (x227 + x223 * x125 +
											  x64 * (x226 + x64 * (x225 - x224 * x124 + x71 * x224) + x72 * x224)) +
									   x69 * x224 + x74 * x224)) +
		 x224 * x129 + x230 * x128 + x89 * x227);
	const GEN_FLT x232 = -x219;
	const GEN_FLT x233 = -x211 * obj_qj;
	const GEN_FLT x234 = x166 * obj_qj;
	const GEN_FLT x235 = x186 + x213 + x234 * x170 + x234 * x172 + (x233 - x167 * obj_qj) * sensor_x;
	const GEN_FLT x236 = x17 * obj_qi;
	const GEN_FLT x237 = x209 + x236 - x177 * obj_qj + x234 * x175 + x234 * x179;
	const GEN_FLT x238 = x17 * obj_qw;
	const GEN_FLT x239 = x171 - x238 + x185 * obj_qj + x234 * x183 + (x233 - x187 * obj_qj) * sensor_z;
	const GEN_FLT x240 = x239 * x191 + x53 * x237 + x54 * x235;
	const GEN_FLT x241 = x43 * x239 + x45 * x237 + x49 * x235;
	const GEN_FLT x242 = (x240 * x109 - x241 * x110) * x112;
	const GEN_FLT x243 = x239 * x155 + x26 * x237 + x40 * x235;
	const GEN_FLT x244 = x241 * x134 + x96 * x240;
	const GEN_FLT x245 = x101 * (x244 + x243 * x137);
	const GEN_FLT x246 = x59 * x243;
	const GEN_FLT x247 = -x62 * x245 + x62 * x246;
	const GEN_FLT x248 = x93 * x247;
	const GEN_FLT x249 = x66 * x248;
	const GEN_FLT x250 = x64 * (x249 - x65 * x248) + x67 * x248;
	const GEN_FLT x251 = x64 * x250 + x68 * x248;
	const GEN_FLT x252 = x78 * x243;
	const GEN_FLT x253 = -x244 * x117 + x77 * x252;
	const GEN_FLT x254 = x242 - x253 * x115;
	const GEN_FLT x255 =
		x91 *
		(x253 -
		 x126 * (-x254 * x123 - x84 * (x64 * x251 +
									   x64 * (x251 + x247 * x125 +
											  x64 * (x250 + x64 * (x249 - x248 * x124 + x71 * x248) + x72 * x248)) +
									   x69 * x248 + x74 * x248)) +
		 x248 * x129 + x254 * x128 + x89 * x251);
	const GEN_FLT x256 = -x242;
	const GEN_FLT x257 = x166 * obj_qk;
	const GEN_FLT x258 = -x211 * obj_qk;
	const GEN_FLT x259 = x180 - x215 + x257 * x170 + x257 * x172 + (x258 - x167 * obj_qk) * sensor_x;
	const GEN_FLT x260 = x173 + x238 + x257 * x175 + x257 * x179 + (x258 - x176 * obj_qk) * sensor_y;
	const GEN_FLT x261 = x208 + x236 - x188 * obj_qk + x257 * x183 + x257 * x184;
	const GEN_FLT x262 = x261 * x191 + x53 * x260 + x54 * x259;
	const GEN_FLT x263 = x43 * x261 + x45 * x260 + x49 * x259;
	const GEN_FLT x264 = (x262 * x109 - x263 * x110) * x112;
	const GEN_FLT x265 = x26 * x260 + x261 * x155 + x40 * x259;
	const GEN_FLT x266 = x263 * x134 + x96 * x262;
	const GEN_FLT x267 = x266 + x265 * x137;
	const GEN_FLT x268 = -x267 * x102 + x63 * x265;
	const GEN_FLT x269 = x93 * x268;
	const GEN_FLT x270 = x66 * x269;
	const GEN_FLT x271 = x64 * (x270 - x65 * x269) + x67 * x269;
	const GEN_FLT x272 = x64 * x271 + x68 * x269;
	const GEN_FLT x273 = x78 * x265;
	const GEN_FLT x274 = -x266 * x117 + x77 * x273;
	const GEN_FLT x275 = x264 - x274 * x115;
	const GEN_FLT x276 =
		x91 *
		(x274 -
		 x126 * (-x275 * x123 - x84 * (x64 * x272 +
									   x64 * (x272 + x268 * x125 +
											  x64 * (x271 + x64 * (x270 - x269 * x124 + x71 * x269) + x72 * x269)) +
									   x69 * x269 + x74 * x269)) +
		 x269 * x129 + x275 * x128 + x89 * x272);
	const GEN_FLT x277 = -x264;
	const GEN_FLT x278 = 0.523598775598299 - tilt_1;
	const GEN_FLT x279 = cos(x278);
	const GEN_FLT x280 = pow(x279, -1);
	const GEN_FLT x281 = x59 * x280;
	const GEN_FLT x282 = asin(x41 * x281);
	const GEN_FLT x283 = 8.0108022e-06 * x282;
	const GEN_FLT x284 = -8.0108022e-06 - x283;
	const GEN_FLT x285 = 0.0028679863 + x282 * x284;
	const GEN_FLT x286 = 5.3685255e-06 + x282 * x285;
	const GEN_FLT x287 = 0.0076069798 + x286 * x282;
	const GEN_FLT x288 = x287 * x282;
	const GEN_FLT x289 = -8.0108022e-06 - 1.60216044e-05 * x282;
	const GEN_FLT x290 = x285 + x289 * x282;
	const GEN_FLT x291 = x286 + x290 * x282;
	const GEN_FLT x292 = x287 + x291 * x282;
	const GEN_FLT x293 = x288 + x292 * x282;
	const GEN_FLT x294 = tan(x278);
	const GEN_FLT x295 = -x79 * x294;
	const GEN_FLT x296 = ogeeMag_1 + x81 - asin(x295);
	const GEN_FLT x297 = curve_1 + sin(x296) * ogeePhase_1;
	const GEN_FLT x298 = sin(x278);
	const GEN_FLT x299 = x297 * x298;
	const GEN_FLT x300 = x279 + x299 * x293;
	const GEN_FLT x301 = pow(x300, -1);
	const GEN_FLT x302 = pow(x282, 2);
	const GEN_FLT x303 = x297 * x302;
	const GEN_FLT x304 = x301 * x303;
	const GEN_FLT x305 = x295 + x287 * x304;
	const GEN_FLT x306 = pow(1 - pow(x305, 2), -1.0 / 2.0);
	const GEN_FLT x307 = pow(1 - pow(x294, 2) * x114, -1.0 / 2.0);
	const GEN_FLT x308 = x294 * x116;
	const GEN_FLT x309 = -x294 * x118 + x98 * x308;
	const GEN_FLT x310 = x113 - x309 * x307;
	const GEN_FLT x311 = cos(x296) * ogeePhase_1;
	const GEN_FLT x312 = x287 * x301 * x302;
	const GEN_FLT x313 = x312 * x311;
	const GEN_FLT x314 = pow(1 - x92 / pow(x279, 2), -1.0 / 2.0);
	const GEN_FLT x315 = x280 * x101;
	const GEN_FLT x316 = x40 * x281 - x99 * x315;
	const GEN_FLT x317 = x314 * x316;
	const GEN_FLT x318 = 2 * x297 * x288 * x301;
	const GEN_FLT x319 = x298 * x293;
	const GEN_FLT x320 = x311 * x319;
	const GEN_FLT x321 = x284 * x317;
	const GEN_FLT x322 = 2.40324066e-05 * x282;
	const GEN_FLT x323 = x282 * (x321 - x283 * x317) + x285 * x317;
	const GEN_FLT x324 = x286 * x314;
	const GEN_FLT x325 = x282 * x323 + x324 * x316;
	const GEN_FLT x326 = x287 * x314;
	const GEN_FLT x327 = x287 * x303 / pow(x300, 2);
	const GEN_FLT x328 =
		x306 * (x309 + x304 * x325 + x313 * x310 + x317 * x318 -
				x327 * (x299 * (x282 * x325 +
								x282 * (x325 + x282 * (x323 + x282 * (x321 + x289 * x317 - x322 * x317) + x290 * x317) +
										x291 * x317) +
								x292 * x317 + x326 * x316) +
						x310 * x320));
	const GEN_FLT x329 = cos(-gibPhase_1 + x132 + asin(x305)) * gibMag_1;
	const GEN_FLT x330 = x294 * x146 - x294 * x147;
	const GEN_FLT x331 = x136 - x307 * x330;
	const GEN_FLT x332 = x26 * x281 - x315 * x140;
	const GEN_FLT x333 = x314 * x332;
	const GEN_FLT x334 = x284 * x333;
	const GEN_FLT x335 = x282 * (x334 - x283 * x333) + x285 * x333;
	const GEN_FLT x336 = x282 * x335 + x324 * x332;
	const GEN_FLT x337 =
		x306 * (x330 + x304 * x336 + x313 * x331 -
				x327 * (x299 * (x282 * x336 +
								x282 * (x336 + x282 * (x335 + x282 * (x334 + x289 * x333 - x322 * x333) + x290 * x333) +
										x291 * x333) +
								x292 * x333 + x326 * x332) +
						x320 * x331) +
				x333 * x318);
	const GEN_FLT x338 = -x294 * x161 + x308 * x153;
	const GEN_FLT x339 = x311 * (x152 - x307 * x338);
	const GEN_FLT x340 = -x280 * x154 + x281 * x155;
	const GEN_FLT x341 = x314 * x340;
	const GEN_FLT x342 = x284 * x341;
	const GEN_FLT x343 = x282 * (x342 - x283 * x341) + x285 * x341;
	const GEN_FLT x344 = x282 * x343 + x324 * x340;
	const GEN_FLT x345 =
		x306 * (x338 + x304 * x344 + x318 * x341 -
				x327 * (x299 * (x282 * x344 +
								x282 * (x344 + x282 * (x343 + x282 * (x342 + x289 * x341 - x322 * x341) + x290 * x341) +
										x291 * x341) +
								x292 * x341 + x326 * x340) +
						x339 * x319) +
				x339 * x312);
	const GEN_FLT x346 = -x202 * x294 + x308 * x194;
	const GEN_FLT x347 = x201 - x307 * x346;
	const GEN_FLT x348 = (x281 * x190 - x315 * x195) * x314;
	const GEN_FLT x349 = x284 * x348;
	const GEN_FLT x350 = x282 * (x349 - x283 * x348) + x285 * x348;
	const GEN_FLT x351 = x282 * x350 + x286 * x348;
	const GEN_FLT x352 =
		x306 * (x346 + x313 * x347 -
				x327 * (x299 * (x282 * x351 +
								x282 * (x351 + x282 * (x350 + x282 * (x349 + x289 * x348 - x322 * x348) + x290 * x348) +
										x291 * x348) +
								x287 * x348 + x292 * x348) +
						x320 * x347) +
				x348 * x318 + x351 * x304);
	const GEN_FLT x353 = x221 * x308 - x294 * x228;
	const GEN_FLT x354 = x219 - x353 * x307;
	const GEN_FLT x355 = -x222 * x315 + x281 * x220;
	const GEN_FLT x356 = x355 * x314;
	const GEN_FLT x357 = x284 * x356;
	const GEN_FLT x358 = x282 * (x357 - x283 * x356) + x285 * x356;
	const GEN_FLT x359 = x282 * x358 + x355 * x324;
	const GEN_FLT x360 =
		x306 * (x353 -
				x327 * (x299 * (x282 * x359 +
								x282 * (x359 + x282 * (x358 + x282 * (x357 + x289 * x356 - x356 * x322) + x290 * x356) +
										x291 * x356) +
								x287 * x356 + x292 * x356) +
						x354 * x320) +
				x354 * x313 + x356 * x318 + x359 * x304);
	const GEN_FLT x361 = x244 * x308 - x294 * x252;
	const GEN_FLT x362 = x242 - x361 * x307;
	const GEN_FLT x363 = -x280 * x245 + x280 * x246;
	const GEN_FLT x364 = x363 * x314;
	const GEN_FLT x365 = x284 * x364;
	const GEN_FLT x366 = x282 * (x365 - x283 * x364) + x285 * x364;
	const GEN_FLT x367 = x282 * x366 + x363 * x324;
	const GEN_FLT x368 =
		x306 * (x361 -
				x327 * (x299 * (x282 * x367 +
								x282 * (x367 + x282 * (x366 + x282 * (x365 + x289 * x364 - x364 * x322) + x290 * x364) +
										x291 * x364) +
								x292 * x364 + x363 * x326) +
						x362 * x320) +
				x362 * x313 + x364 * x318 + x367 * x304);
	const GEN_FLT x369 = x266 * x308 - x273 * x294;
	const GEN_FLT x370 = x264 - x369 * x307;
	const GEN_FLT x371 = (x265 * x281 - x267 * x315) * x314;
	const GEN_FLT x372 = x284 * x371;
	const GEN_FLT x373 = x282 * (x372 - x283 * x371) + x285 * x371;
	const GEN_FLT x374 = x282 * x373 + x286 * x371;
	const GEN_FLT x375 =
		x306 * (x369 -
				x327 * (x299 * (x282 * x374 +
								x282 * (x374 + x282 * (x373 + x282 * (x372 + x289 * x371 - x371 * x322) + x290 * x371) +
										x291 * x371) +
								x287 * x371 + x292 * x371) +
						x370 * x320) +
				x370 * x313 + x371 * x318 + x374 * x304);
	out[0] = x113 - x130 - (x130 + x131) * x133;
	out[1] = x136 - x150 - (x150 + x151) * x133;
	out[2] = x152 - x164 - (x164 + x165) * x133;
	out[3] = x201 - x205 - (x205 + x206) * x133;
	out[4] = x219 - x231 - (x231 + x232) * x133;
	out[5] = x242 - x255 - (x255 + x256) * x133;
	out[6] = x264 - x276 - (x276 + x277) * x133;
	out[7] = x113 - x328 - (x131 + x328) * x329;
	out[8] = x136 - x337 - (x151 + x337) * x329;
	out[9] = x152 - x345 - (x165 + x345) * x329;
	out[10] = x201 - x352 - (x206 + x352) * x329;
	out[11] = x219 - x360 - (x232 + x360) * x329;
	out[12] = x242 - x368 - (x256 + x368) * x329;
	out[13] = x264 - x375 - (x277 + x375) * x329;
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt(x1 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - (x0 + x1) * x5;
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = pow(obj_qj, 2);
	const GEN_FLT x9 = pow(obj_qi, 2);
	const GEN_FLT x10 = x7 + x9;
	const GEN_FLT x11 = sqrt(x10 + x8 + pow(obj_qw, 2));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 - (x7 + x8) * x12;
	const GEN_FLT x14 = lh_qw * lh_qk;
	const GEN_FLT x15 = lh_qj * lh_qi;
	const GEN_FLT x16 = -x14 + x15;
	const GEN_FLT x17 = obj_qw * obj_qk;
	const GEN_FLT x18 = obj_qj * obj_qi;
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 4 * x4 * x11;
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = obj_qw * obj_qj;
	const GEN_FLT x23 = obj_qk * obj_qi;
	const GEN_FLT x24 = -x22 + x23;
	const GEN_FLT x25 = lh_qw * lh_qj;
	const GEN_FLT x26 = lh_qk * lh_qi;
	const GEN_FLT x27 = x25 + x26;
	const GEN_FLT x28 = x20 * x27;
	const GEN_FLT x29 = x21 * x16 + x24 * x28 + x6 * x13;
	const GEN_FLT x30 = 1 - (x1 + x2) * x5;
	const GEN_FLT x31 = 1 - (x8 + x9) * x12;
	const GEN_FLT x32 = obj_qw * obj_qi;
	const GEN_FLT x33 = obj_qk * obj_qj;
	const GEN_FLT x34 = x32 + x33;
	const GEN_FLT x35 = x12 * sensor_y;
	const GEN_FLT x36 = x12 * sensor_x;
	const GEN_FLT x37 = obj_pz + x31 * sensor_z + x34 * x35 + x36 * x24;
	const GEN_FLT x38 = lh_qw * lh_qi;
	const GEN_FLT x39 = lh_qk * lh_qj;
	const GEN_FLT x40 = x38 + x39;
	const GEN_FLT x41 = -x32 + x33;
	const GEN_FLT x42 = x12 * sensor_z;
	const GEN_FLT x43 = 1 - x12 * x10;
	const GEN_FLT x44 = obj_py + x36 * x19 + x41 * x42 + x43 * sensor_y;
	const GEN_FLT x45 = x5 * x44;
	const GEN_FLT x46 = x22 + x23;
	const GEN_FLT x47 = -x17 + x18;
	const GEN_FLT x48 = obj_px + x13 * sensor_x + x42 * x46 + x47 * x35;
	const GEN_FLT x49 = -x25 + x26;
	const GEN_FLT x50 = x5 * x49;
	const GEN_FLT x51 = lh_pz + x30 * x37 + x40 * x45 + x50 * x48;
	const GEN_FLT x52 = x5 * x37;
	const GEN_FLT x53 = lh_px + x45 * x16 + x52 * x27 + x6 * x48;
	const GEN_FLT x54 = pow(x53, 2);
	const GEN_FLT x55 = x51 / x54;
	const GEN_FLT x56 = pow(x53, -1);
	const GEN_FLT x57 = x30 * x12;
	const GEN_FLT x58 = x40 * x21 + x50 * x13 + x57 * x24;
	const GEN_FLT x59 = x54 + pow(x51, 2);
	const GEN_FLT x60 = pow(x59, -1);
	const GEN_FLT x61 = x60 * x54;
	const GEN_FLT x62 = (x55 * x29 - x58 * x56) * x61;
	const GEN_FLT x63 = 0.523598775598299 + tilt_0;
	const GEN_FLT x64 = cos(x63);
	const GEN_FLT x65 = pow(x64, -1);
	const GEN_FLT x66 = -x38 + x39;
	const GEN_FLT x67 = 1 - x3 * x5;
	const GEN_FLT x68 = x14 + x15;
	const GEN_FLT x69 = x5 * x68;
	const GEN_FLT x70 = lh_py + x66 * x52 + x67 * x44 + x69 * x48;
	const GEN_FLT x71 = pow(x70, 2);
	const GEN_FLT x72 = x59 + x71;
	const GEN_FLT x73 = pow(x72, -1.0 / 2.0);
	const GEN_FLT x74 = x70 * x73;
	const GEN_FLT x75 = asin(x74 * x65);
	const GEN_FLT x76 = 8.0108022e-06 * x75;
	const GEN_FLT x77 = -8.0108022e-06 - x76;
	const GEN_FLT x78 = 0.0028679863 + x75 * x77;
	const GEN_FLT x79 = 5.3685255e-06 + x78 * x75;
	const GEN_FLT x80 = 0.0076069798 + x79 * x75;
	const GEN_FLT x81 = x80 * x75;
	const GEN_FLT x82 = -8.0108022e-06 - 1.60216044e-05 * x75;
	const GEN_FLT x83 = x78 + x82 * x75;
	const GEN_FLT x84 = x79 + x83 * x75;
	const GEN_FLT x85 = x80 + x84 * x75;
	const GEN_FLT x86 = x81 + x85 * x75;
	const GEN_FLT x87 = sin(x63);
	const GEN_FLT x88 = tan(x63);
	const GEN_FLT x89 = pow(x59, -1.0 / 2.0);
	const GEN_FLT x90 = x88 * x89;
	const GEN_FLT x91 = x70 * x90;
	const GEN_FLT x92 = atan2(-x51, x53);
	const GEN_FLT x93 = ogeeMag_0 + x92 - asin(x91);
	const GEN_FLT x94 = curve_0 + sin(x93) * ogeePhase_0;
	const GEN_FLT x95 = x87 * x94;
	const GEN_FLT x96 = x64 - x86 * x95;
	const GEN_FLT x97 = pow(x96, -1);
	const GEN_FLT x98 = pow(x75, 2);
	const GEN_FLT x99 = x98 * x94;
	const GEN_FLT x100 = x99 * x97;
	const GEN_FLT x101 = x91 + x80 * x100;
	const GEN_FLT x102 = pow(1 - pow(x101, 2), -1.0 / 2.0);
	const GEN_FLT x103 = x71 / x72;
	const GEN_FLT x104 = pow(1 - x103 / pow(x64, 2), -1.0 / 2.0);
	const GEN_FLT x105 = x67 * x12;
	const GEN_FLT x106 = x66 * x20;
	const GEN_FLT x107 = x19 * x105 + x24 * x106 + x69 * x13;
	const GEN_FLT x108 = 2 * x70;
	const GEN_FLT x109 = 2 * x53;
	const GEN_FLT x110 = 2 * x51;
	const GEN_FLT x111 = x29 * x109 + x58 * x110;
	const GEN_FLT x112 = (1.0 / 2.0) * x70;
	const GEN_FLT x113 = x112 / pow(x72, 3.0 / 2.0);
	const GEN_FLT x114 = x113 * (x111 + x108 * x107);
	const GEN_FLT x115 = x73 * x107;
	const GEN_FLT x116 = (-x65 * x114 + x65 * x115) * x104;
	const GEN_FLT x117 = x77 * x116;
	const GEN_FLT x118 = x75 * (x117 - x76 * x116) + x78 * x116;
	const GEN_FLT x119 = x75 * x118 + x79 * x116;
	const GEN_FLT x120 = 2 * x81 * x97 * x94;
	const GEN_FLT x121 = x112 / pow(x59, 3.0 / 2.0);
	const GEN_FLT x122 = x88 * x121;
	const GEN_FLT x123 = -x111 * x122 + x90 * x107;
	const GEN_FLT x124 = x71 * x60;
	const GEN_FLT x125 = pow(1 - pow(x88, 2) * x124, -1.0 / 2.0);
	const GEN_FLT x126 = cos(x93) * ogeePhase_0;
	const GEN_FLT x127 = x126 * (x62 - x123 * x125);
	const GEN_FLT x128 = x86 * x87;
	const GEN_FLT x129 = 2.40324066e-05 * x75;
	const GEN_FLT x130 = x80 * x99 / pow(x96, 2);
	const GEN_FLT x131 = x80 * x98 * x97;
	const GEN_FLT x132 =
		x102 *
		(x123 + x100 * x119 + x116 * x120 + x127 * x131 -
		 x130 *
			 (-x128 * x127 -
			  x95 * (x75 * x119 +
					 x75 * (x119 + x75 * (x118 + x75 * (x117 - x116 * x129 + x82 * x116) + x83 * x116) + x84 * x116) +
					 x80 * x116 + x85 * x116)));
	const GEN_FLT x133 = -x62;
	const GEN_FLT x134 = -x92;
	const GEN_FLT x135 = cos(-gibPhase_0 + x134 + asin(x101)) * gibMag_0;
	const GEN_FLT x136 = x5 * x43;
	const GEN_FLT x137 = x6 * x12;
	const GEN_FLT x138 = x16 * x136 + x34 * x28 + x47 * x137;
	const GEN_FLT x139 = x47 * x20;
	const GEN_FLT x140 = x40 * x136 + x49 * x139 + x57 * x34;
	const GEN_FLT x141 = (x55 * x138 - x56 * x140) * x61;
	const GEN_FLT x142 = x34 * x106 + x67 * x43 + x68 * x139;
	const GEN_FLT x143 = x109 * x138 + x110 * x140;
	const GEN_FLT x144 = x143 + x108 * x142;
	const GEN_FLT x145 = x65 * x113;
	const GEN_FLT x146 = x73 * x142;
	const GEN_FLT x147 = x104 * (-x144 * x145 + x65 * x146);
	const GEN_FLT x148 = x77 * x147;
	const GEN_FLT x149 = x75 * (x148 - x76 * x147) + x78 * x147;
	const GEN_FLT x150 = x75 * x149 + x79 * x147;
	const GEN_FLT x151 = x89 * x142;
	const GEN_FLT x152 = -x122 * x143 + x88 * x151;
	const GEN_FLT x153 = x141 - x125 * x152;
	const GEN_FLT x154 = x128 * x126;
	const GEN_FLT x155 = x126 * x131;
	const GEN_FLT x156 =
		x102 *
		(x152 + x100 * x150 + x120 * x147 -
		 x130 *
			 (-x153 * x154 -
			  x95 * (x75 * x150 +
					 x75 * (x150 + x75 * (x149 + x75 * (x148 - x129 * x147 + x82 * x147) + x83 * x147) + x84 * x147) +
					 x80 * x147 + x85 * x147)) +
		 x153 * x155);
	const GEN_FLT x157 = -x141;
	const GEN_FLT x158 = x41 * x20;
	const GEN_FLT x159 = x5 * x31;
	const GEN_FLT x160 = x16 * x158 + x27 * x159 + x46 * x137;
	const GEN_FLT x161 = x46 * x20;
	const GEN_FLT x162 = x30 * x31 + x40 * x158 + x49 * x161;
	const GEN_FLT x163 = (x55 * x160 - x56 * x162) * x61;
	const GEN_FLT x164 = x41 * x105 + x66 * x159 + x68 * x161;
	const GEN_FLT x165 = x109 * x160 + x110 * x162;
	const GEN_FLT x166 = x165 + x108 * x164;
	const GEN_FLT x167 = x73 * x164;
	const GEN_FLT x168 = x104 * (-x166 * x145 + x65 * x167);
	const GEN_FLT x169 = x77 * x168;
	const GEN_FLT x170 = x75 * (x169 - x76 * x168) + x78 * x168;
	const GEN_FLT x171 = x75 * x170 + x79 * x168;
	const GEN_FLT x172 = x89 * x164;
	const GEN_FLT x173 = -x122 * x165 + x88 * x172;
	const GEN_FLT x174 = x163 - x125 * x173;
	const GEN_FLT x175 =
		x102 *
		(x173 + x100 * x171 + x120 * x168 -
		 x130 *
			 (-x174 * x154 -
			  x95 * (x75 * x171 +
					 x75 * (x171 + x75 * (x170 + x75 * (x169 - x129 * x168 + x82 * x168) + x83 * x168) + x84 * x168) +
					 x80 * x168 + x85 * x168)) +
		 x174 * x155);
	const GEN_FLT x176 = -x163;
	const GEN_FLT x177 = 0.523598775598299 - tilt_1;
	const GEN_FLT x178 = cos(x177);
	const GEN_FLT x179 = pow(x178, -1);
	const GEN_FLT x180 = asin(x74 * x179);
	const GEN_FLT x181 = 8.0108022e-06 * x180;
	const GEN_FLT x182 = -8.0108022e-06 - x181;
	const GEN_FLT x183 = 0.0028679863 + x180 * x182;
	const GEN_FLT x184 = 5.3685255e-06 + x180 * x183;
	const GEN_FLT x185 = 0.0076069798 + x180 * x184;
	const GEN_FLT x186 = pow(x180, 2);
	const GEN_FLT x187 = tan(x177);
	const GEN_FLT x188 = x89 * x187;
	const GEN_FLT x189 = -x70 * x188;
	const GEN_FLT x190 = ogeeMag_1 + x92 - asin(x189);
	const GEN_FLT x191 = curve_1 + sin(x190) * ogeePhase_1;
	const GEN_FLT x192 = x180 * x185;
	const GEN_FLT x193 = -8.0108022e-06 - 1.60216044e-05 * x180;
	const GEN_FLT x194 = x183 + x180 * x193;
	const GEN_FLT x195 = x184 + x180 * x194;
	const GEN_FLT x196 = x185 + x180 * x195;
	const GEN_FLT x197 = x192 + x180 * x196;
	const GEN_FLT x198 = sin(x177);
	const GEN_FLT x199 = x191 * x198;
	const GEN_FLT x200 = x178 + x197 * x199;
	const GEN_FLT x201 = pow(x200, -1);
	const GEN_FLT x202 = x201 * x191;
	const GEN_FLT x203 = x202 * x186;
	const GEN_FLT x204 = x189 + x203 * x185;
	const GEN_FLT x205 = pow(1 - pow(x204, 2), -1.0 / 2.0);
	const GEN_FLT x206 = x121 * x187;
	const GEN_FLT x207 = -x107 * x188 + x206 * x111;
	const GEN_FLT x208 = pow(1 - x124 * pow(x187, 2), -1.0 / 2.0);
	const GEN_FLT x209 = cos(x190) * ogeePhase_1;
	const GEN_FLT x210 = x209 * (x62 - x208 * x207);
	const GEN_FLT x211 = x186 * x185;
	const GEN_FLT x212 = x211 * x201;
	const GEN_FLT x213 = pow(1 - x103 / pow(x178, 2), -1.0 / 2.0);
	const GEN_FLT x214 = -x114 * x179 + x115 * x179;
	const GEN_FLT x215 = x213 * x214;
	const GEN_FLT x216 = 2 * x202 * x192;
	const GEN_FLT x217 = x197 * x198;
	const GEN_FLT x218 = x215 * x182;
	const GEN_FLT x219 = 2.40324066e-05 * x180;
	const GEN_FLT x220 = x180 * (x218 - x215 * x181) + x215 * x183;
	const GEN_FLT x221 = x215 * x184 + x220 * x180;
	const GEN_FLT x222 = x213 * x185;
	const GEN_FLT x223 = x213 * x196;
	const GEN_FLT x224 = x211 * x191 / pow(x200, 2);
	const GEN_FLT x225 =
		x205 * (x207 + x203 * x221 + x210 * x212 + x215 * x216 -
				x224 * (x199 * (x180 * (x221 + x180 * (x220 + x180 * (x218 + x215 * x193 - x219 * x215) + x215 * x194) +
										x215 * x195) +
								x214 * x222 + x214 * x223 + x221 * x180) +
						x210 * x217));
	const GEN_FLT x226 = cos(-gibPhase_1 + x134 + asin(x204)) * gibMag_1;
	const GEN_FLT x227 = -x187 * x151 + x206 * x143;
	const GEN_FLT x228 = x141 - x208 * x227;
	const GEN_FLT x229 = x212 * x209;
	const GEN_FLT x230 = x113 * x179;
	const GEN_FLT x231 = x179 * x146 - x230 * x144;
	const GEN_FLT x232 = x213 * x231;
	const GEN_FLT x233 = x217 * x209;
	const GEN_FLT x234 = x232 * x182;
	const GEN_FLT x235 = x180 * (x234 - x232 * x181) + x232 * x183;
	const GEN_FLT x236 = x232 * x184 + x235 * x180;
	const GEN_FLT x237 =
		x205 * (x227 + x216 * x232 -
				x224 * (x199 * (x180 * (x236 + x180 * (x235 + x180 * (x234 - x219 * x232 + x232 * x193) + x232 * x194) +
										x232 * x195) +
								x231 * x222 + x231 * x223 + x236 * x180) +
						x233 * x228) +
				x228 * x229 + x236 * x203);
	const GEN_FLT x238 = -x172 * x187 + x206 * x165;
	const GEN_FLT x239 = x163 - x238 * x208;
	const GEN_FLT x240 = x167 * x179 - x230 * x166;
	const GEN_FLT x241 = x213 * x240;
	const GEN_FLT x242 = x241 * x182;
	const GEN_FLT x243 = x180 * (x242 - x241 * x181) + x241 * x183;
	const GEN_FLT x244 = x241 * x184 + x243 * x180;
	const GEN_FLT x245 =
		x205 * (x238 + x203 * x244 + x216 * x241 -
				x224 * (x199 * (x180 * (x244 + x180 * (x243 + x180 * (x242 - x219 * x241 + x241 * x193) + x241 * x194) +
										x241 * x195) +
								x222 * x240 + x223 * x240 + x244 * x180) +
						x233 * x239) +
				x239 * x229);
	out[0] = -x132 + x62 - (x132 + x133) * x135;
	out[1] = x141 - x156 - (x156 + x157) * x135;
	out[2] = x163 - x175 - (x175 + x176) * x135;
	out[3] = -x225 + x62 - (x133 + x225) * x226;
	out[4] = x141 - x237 - (x157 + x237) * x226;
	out[5] = x163 - x245 - (x176 + x245) * x226;
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
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qk, 2);
	const GEN_FLT x4 = x0 + x3;
	const GEN_FLT x5 = sqrt(x1 + x4 + pow(lh_qw, 2));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = pow(obj_qk, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = 2 * sqrt(x10 + x7 + pow(obj_qw, 2));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 = obj_pz + (1 - (x7 + x8) * x11) * sensor_z + (x12 + x13) * x14 + (-x15 + x16) * x17;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + (1 - x11 * x10) * sensor_y + (-x12 + x13) * x22 + (x23 + x24) * x17;
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = -x27 + x28;
	const GEN_FLT x30 = obj_px + (1 - (x7 + x9) * x11) * sensor_x + (x15 + x16) * x22 + (-x23 + x24) * x14;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + x18 * (1 - x2 * x6) + x21 * x26 + x31 * x29;
	const GEN_FLT x33 = x27 + x28;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = -x35 + x36;
	const GEN_FLT x38 = lh_px + x30 * (1 - x4 * x6) + x34 * x33 + x37 * x26;
	const GEN_FLT x39 = pow(x38, 2);
	const GEN_FLT x40 = x39 + pow(x32, 2);
	const GEN_FLT x41 = pow(x40, -1);
	const GEN_FLT x42 = x41 * x32;
	const GEN_FLT x43 = -x19 + x20;
	const GEN_FLT x44 = x1 + x3;
	const GEN_FLT x45 = x35 + x36;
	const GEN_FLT x46 = lh_py + x25 * (1 - x6 * x44) + x43 * x34 + x45 * x31;
	const GEN_FLT x47 = 0.523598775598299 + tilt_0;
	const GEN_FLT x48 = cos(x47);
	const GEN_FLT x49 = pow(x48, -1);
	const GEN_FLT x50 = pow(x46, 2);
	const GEN_FLT x51 = x40 + x50;
	const GEN_FLT x52 = pow(x51, -1.0 / 2.0);
	const GEN_FLT x53 = x52 * x49;
	const GEN_FLT x54 = asin(x53 * x46);
	const GEN_FLT x55 = 8.0108022e-06 * x54;
	const GEN_FLT x56 = -8.0108022e-06 - x55;
	const GEN_FLT x57 = 0.0028679863 + x54 * x56;
	const GEN_FLT x58 = 5.3685255e-06 + x54 * x57;
	const GEN_FLT x59 = 0.0076069798 + x54 * x58;
	const GEN_FLT x60 = x54 * x59;
	const GEN_FLT x61 = -8.0108022e-06 - 1.60216044e-05 * x54;
	const GEN_FLT x62 = x57 + x61 * x54;
	const GEN_FLT x63 = x58 + x62 * x54;
	const GEN_FLT x64 = x59 + x63 * x54;
	const GEN_FLT x65 = x60 + x64 * x54;
	const GEN_FLT x66 = sin(x47);
	const GEN_FLT x67 = tan(x47);
	const GEN_FLT x68 = pow(x40, -1.0 / 2.0);
	const GEN_FLT x69 = x67 * x68;
	const GEN_FLT x70 = x69 * x46;
	const GEN_FLT x71 = atan2(-x32, x38);
	const GEN_FLT x72 = ogeeMag_0 + x71 - asin(x70);
	const GEN_FLT x73 = curve_0 + sin(x72) * ogeePhase_0;
	const GEN_FLT x74 = x73 * x66;
	const GEN_FLT x75 = x48 - x74 * x65;
	const GEN_FLT x76 = pow(x75, -1);
	const GEN_FLT x77 = pow(x54, 2);
	const GEN_FLT x78 = x73 * x77;
	const GEN_FLT x79 = x78 * x76;
	const GEN_FLT x80 = x70 + x79 * x59;
	const GEN_FLT x81 = pow(1 - pow(x80, 2), -1.0 / 2.0);
	const GEN_FLT x82 = x46 / pow(x40, 3.0 / 2.0);
	const GEN_FLT x83 = x82 * x38;
	const GEN_FLT x84 = x83 * x67;
	const GEN_FLT x85 = x50 / x51;
	const GEN_FLT x86 = pow(1 - x85 / pow(x48, 2), -1.0 / 2.0);
	const GEN_FLT x87 = pow(x51, -3.0 / 2.0);
	const GEN_FLT x88 = x87 * x46;
	const GEN_FLT x89 = x88 * x38;
	const GEN_FLT x90 = x89 * x49;
	const GEN_FLT x91 = x86 * x90;
	const GEN_FLT x92 = -x56 * x91;
	const GEN_FLT x93 = x86 * x57;
	const GEN_FLT x94 = x54 * (x92 + x55 * x91) - x93 * x90;
	const GEN_FLT x95 = x54 * x94 - x58 * x91;
	const GEN_FLT x96 = x50 * x41;
	const GEN_FLT x97 = pow(1 - pow(x67, 2) * x96, -1.0 / 2.0);
	const GEN_FLT x98 = x42 + x84 * x97;
	const GEN_FLT x99 = cos(x72) * ogeePhase_0;
	const GEN_FLT x100 = x65 * x66;
	const GEN_FLT x101 = x99 * x100;
	const GEN_FLT x102 = 2.40324066e-05 * x54;
	const GEN_FLT x103 = x86 * x63;
	const GEN_FLT x104 = x86 * x64;
	const GEN_FLT x105 = x86 * x59;
	const GEN_FLT x106 = x78 * x59 / pow(x75, 2);
	const GEN_FLT x107 = x77 * x76 * x59;
	const GEN_FLT x108 = x99 * x107;
	const GEN_FLT x109 = 2 * x38;
	const GEN_FLT x110 = x88 * x109;
	const GEN_FLT x111 = x73 * x76 * x60;
	const GEN_FLT x112 = x86 * x49 * x111;
	const GEN_FLT x113 =
		x81 *
		(-x84 -
		 x106 * (-x74 * (x54 * x95 +
						 x54 * (x95 + x54 * (x94 + x54 * (x92 - x61 * x91 + x91 * x102) - x62 * x91) - x90 * x103) -
						 x90 * x104 - x90 * x105) -
				 x98 * x101) -
		 x110 * x112 + x79 * x95 + x98 * x108);
	const GEN_FLT x114 = -x42;
	const GEN_FLT x115 = -x71;
	const GEN_FLT x116 = cos(-gibPhase_0 + x115 + asin(x80)) * gibMag_0;
	const GEN_FLT x117 = x87 * x50;
	const GEN_FLT x118 = x53 - x49 * x117;
	const GEN_FLT x119 = x86 * x118;
	const GEN_FLT x120 = x56 * x119;
	const GEN_FLT x121 = x54 * (x120 - x55 * x119) + x93 * x118;
	const GEN_FLT x122 = x54 * x121 + x58 * x119;
	const GEN_FLT x123 = x69 * x97;
	const GEN_FLT x124 = 2 * x111;
	const GEN_FLT x125 =
		x81 *
		(x69 -
		 x106 * (x101 * x123 - x74 * (x104 * x118 + x105 * x118 + x54 * x122 +
									  x54 * (x122 + x103 * x118 +
											 x54 * (x121 + x54 * (x120 - x102 * x119 + x61 * x119) + x62 * x119)))) -
		 x108 * x123 + x119 * x124 + x79 * x122);
	const GEN_FLT x126 = x41 * x38;
	const GEN_FLT x127 = -x126;
	const GEN_FLT x128 = x88 * x32;
	const GEN_FLT x129 = x49 * x128;
	const GEN_FLT x130 = x86 * x129;
	const GEN_FLT x131 = -x56 * x130;
	const GEN_FLT x132 = x54 * (x131 + x55 * x130) - x93 * x129;
	const GEN_FLT x133 = x54 * x132 - x58 * x130;
	const GEN_FLT x134 = x82 * x32;
	const GEN_FLT x135 = x67 * x134;
	const GEN_FLT x136 = x127 + x97 * x135;
	const GEN_FLT x137 = 2 * x32;
	const GEN_FLT x138 = x88 * x137;
	const GEN_FLT x139 =
		x81 *
		(-x135 -
		 x106 * (-x101 * x136 - x74 * (-x104 * x129 - x105 * x129 + x54 * x133 +
									   x54 * (x133 - x103 * x129 +
											  x54 * (x132 + x54 * (x131 + x102 * x130 - x61 * x130) - x62 * x130)))) +
		 x108 * x136 - x112 * x138 + x79 * x133);
	const GEN_FLT x140 = 2 / x5;
	const GEN_FLT x141 = x4 * x140;
	const GEN_FLT x142 = x30 * x141;
	const GEN_FLT x143 = x140 * lh_qw;
	const GEN_FLT x144 = x25 * x143;
	const GEN_FLT x145 = x26 * lh_qk;
	const GEN_FLT x146 = x33 * x18;
	const GEN_FLT x147 = x34 * lh_qj;
	const GEN_FLT x148 = -x145 + x147 - x142 * lh_qw + x143 * x146 + x37 * x144;
	const GEN_FLT x149 = x32 / x39;
	const GEN_FLT x150 = pow(x38, -1);
	const GEN_FLT x151 = x30 * x29;
	const GEN_FLT x152 = x26 * lh_qi;
	const GEN_FLT x153 = x31 * lh_qj;
	const GEN_FLT x154 = x2 * x140;
	const GEN_FLT x155 = x152 - x153 + x143 * x151 + x21 * x144 - x18 * x154 * lh_qw;
	const GEN_FLT x156 = x41 * x39;
	const GEN_FLT x157 = (x148 * x149 - x150 * x155) * x156;
	const GEN_FLT x158 = x45 * x30;
	const GEN_FLT x159 = x31 * lh_qk;
	const GEN_FLT x160 = x44 * x140;
	const GEN_FLT x161 = x25 * x160;
	const GEN_FLT x162 = x43 * x18;
	const GEN_FLT x163 = x34 * lh_qi;
	const GEN_FLT x164 = x159 - x163 + x143 * x158 - x161 * lh_qw + x162 * x143;
	const GEN_FLT x165 = 2 * x46;
	const GEN_FLT x166 = x109 * x148 + x137 * x155;
	const GEN_FLT x167 = (1.0 / 2.0) * x88;
	const GEN_FLT x168 = x167 * (x166 + x165 * x164);
	const GEN_FLT x169 = -x49 * x168 + x53 * x164;
	const GEN_FLT x170 = x86 * x169;
	const GEN_FLT x171 = x56 * x170;
	const GEN_FLT x172 = x54 * (x171 - x55 * x170) + x93 * x169;
	const GEN_FLT x173 = x54 * x172 + x58 * x170;
	const GEN_FLT x174 = (1.0 / 2.0) * x82;
	const GEN_FLT x175 = x166 * x174;
	const GEN_FLT x176 = -x67 * x175 + x69 * x164;
	const GEN_FLT x177 = x157 - x97 * x176;
	const GEN_FLT x178 =
		x81 *
		(x176 -
		 x106 * (-x101 * x177 - x74 * (x104 * x169 + x105 * x169 + x54 * x173 +
									   x54 * (x173 + x103 * x169 +
											  x54 * (x172 + x54 * (x171 - x102 * x170 + x61 * x170) + x62 * x170)))) +
		 x108 * x177 + x124 * x170 + x79 * x173);
	const GEN_FLT x179 = -x157;
	const GEN_FLT x180 = x140 * lh_qi;
	const GEN_FLT x181 = x37 * x25;
	const GEN_FLT x182 = x26 * lh_qj;
	const GEN_FLT x183 = x34 * lh_qk;
	const GEN_FLT x184 = x182 + x183 - x142 * lh_qi + x180 * x146 + x181 * x180;
	const GEN_FLT x185 = x26 * lh_qw;
	const GEN_FLT x186 = x25 * x21;
	const GEN_FLT x187 = 4 * x5;
	const GEN_FLT x188 = -x187 * lh_qi;
	const GEN_FLT x189 = x159 + x185 + x18 * (x188 - x154 * lh_qi) + x180 * x151 + x180 * x186;
	const GEN_FLT x190 = (x184 * x149 - x189 * x150) * x156;
	const GEN_FLT x191 = x34 * lh_qw;
	const GEN_FLT x192 = x153 - x191 + x162 * x180 + x180 * x158 + x25 * (x188 - x160 * lh_qi);
	const GEN_FLT x193 = x109 * x184 + x189 * x137;
	const GEN_FLT x194 = x167 * (x193 + x165 * x192);
	const GEN_FLT x195 = (-x49 * x194 + x53 * x192) * x86;
	const GEN_FLT x196 = x56 * x195;
	const GEN_FLT x197 = x54 * (x196 - x55 * x195) + x57 * x195;
	const GEN_FLT x198 = x54 * x197 + x58 * x195;
	const GEN_FLT x199 = x174 * x193;
	const GEN_FLT x200 = -x67 * x199 + x69 * x192;
	const GEN_FLT x201 = x99 * (x190 - x97 * x200);
	const GEN_FLT x202 =
		x81 *
		(x200 -
		 x106 *
			 (-x201 * x100 -
			  x74 * (x54 * x198 +
					 x54 * (x198 + x54 * (x197 + x54 * (x196 - x102 * x195 + x61 * x195) + x62 * x195) + x63 * x195) +
					 x59 * x195 + x64 * x195)) +
		 x124 * x195 + x201 * x107 + x79 * x198);
	const GEN_FLT x203 = -x190;
	const GEN_FLT x204 = -x187 * lh_qj;
	const GEN_FLT x205 = x140 * lh_qj;
	const GEN_FLT x206 = x152 + x191 + x205 * x146 + x205 * x181 + x30 * (x204 - x141 * lh_qj);
	const GEN_FLT x207 = x140 * x151;
	const GEN_FLT x208 = x31 * lh_qw;
	const GEN_FLT x209 = x145 - x208 + x18 * (x204 - x154 * lh_qj) + x205 * x186 + x207 * lh_qj;
	const GEN_FLT x210 = (x206 * x149 - x209 * x150) * x156;
	const GEN_FLT x211 = x31 * lh_qi;
	const GEN_FLT x212 = x162 * x140;
	const GEN_FLT x213 = x183 + x211 - x161 * lh_qj + x205 * x158 + x212 * lh_qj;
	const GEN_FLT x214 = x206 * x109 + x209 * x137;
	const GEN_FLT x215 = x167 * (x214 + x213 * x165);
	const GEN_FLT x216 = -x49 * x215 + x53 * x213;
	const GEN_FLT x217 = x86 * x216;
	const GEN_FLT x218 = x56 * x217;
	const GEN_FLT x219 = x54 * (x218 - x55 * x217) + x93 * x216;
	const GEN_FLT x220 = x54 * x219 + x58 * x217;
	const GEN_FLT x221 = x214 * x174;
	const GEN_FLT x222 = -x67 * x221 + x69 * x213;
	const GEN_FLT x223 = x210 - x97 * x222;
	const GEN_FLT x224 =
		x81 *
		(x222 -
		 x106 * (-x223 * x101 - x74 * (x216 * x104 + x54 * x220 +
									   x54 * (x220 + x216 * x103 +
											  x54 * (x219 + x54 * (x218 - x217 * x102 + x61 * x217) + x62 * x217)) +
									   x59 * x217)) +
		 x217 * x124 + x223 * x108 + x79 * x220);
	const GEN_FLT x225 = -x210;
	const GEN_FLT x226 = -x187 * lh_qk;
	const GEN_FLT x227 = x140 * lh_qk;
	const GEN_FLT x228 = x18 * lh_qk;
	const GEN_FLT x229 = x163 - x185 + x227 * x181 + x30 * (x226 - x141 * lh_qk) + x33 * x228 * x140;
	const GEN_FLT x230 = x182 + x211 + x207 * lh_qk + x227 * x186 - x228 * x154;
	const GEN_FLT x231 = (x229 * x149 - x230 * x150) * x156;
	const GEN_FLT x232 = x147 + x208 + x212 * lh_qk + x227 * x158 + x25 * (x226 - x160 * lh_qk);
	const GEN_FLT x233 = x229 * x109 + x230 * x137;
	const GEN_FLT x234 = x167 * (x233 + x232 * x165);
	const GEN_FLT x235 = -x49 * x234 + x53 * x232;
	const GEN_FLT x236 = x86 * x235;
	const GEN_FLT x237 = x56 * x236;
	const GEN_FLT x238 = x54 * (x237 - x55 * x236) + x93 * x235;
	const GEN_FLT x239 = x54 * x238 + x58 * x236;
	const GEN_FLT x240 = x233 * x174;
	const GEN_FLT x241 = -x67 * x240 + x69 * x232;
	const GEN_FLT x242 = x231 - x97 * x241;
	const GEN_FLT x243 =
		x81 *
		(x241 -
		 x106 * (-x242 * x101 - x74 * (x54 * x239 +
									   x54 * (x239 + x235 * x103 +
											  x54 * (x238 + x54 * (x237 - x236 * x102 + x61 * x236) + x62 * x236)) +
									   x59 * x236 + x64 * x236)) +
		 x236 * x124 + x242 * x108 + x79 * x239);
	const GEN_FLT x244 = -x231;
	const GEN_FLT x245 = 0.523598775598299 - tilt_1;
	const GEN_FLT x246 = cos(x245);
	const GEN_FLT x247 = pow(x246, -1);
	const GEN_FLT x248 = x52 * x247;
	const GEN_FLT x249 = asin(x46 * x248);
	const GEN_FLT x250 = 8.0108022e-06 * x249;
	const GEN_FLT x251 = -8.0108022e-06 - x250;
	const GEN_FLT x252 = 0.0028679863 + x251 * x249;
	const GEN_FLT x253 = 5.3685255e-06 + x252 * x249;
	const GEN_FLT x254 = 0.0076069798 + x253 * x249;
	const GEN_FLT x255 = x254 * x249;
	const GEN_FLT x256 = -8.0108022e-06 - 1.60216044e-05 * x249;
	const GEN_FLT x257 = x252 + x256 * x249;
	const GEN_FLT x258 = x253 + x257 * x249;
	const GEN_FLT x259 = x254 + x258 * x249;
	const GEN_FLT x260 = x255 + x259 * x249;
	const GEN_FLT x261 = tan(x245);
	const GEN_FLT x262 = x68 * x261;
	const GEN_FLT x263 = -x46 * x262;
	const GEN_FLT x264 = ogeeMag_1 + x71 - asin(x263);
	const GEN_FLT x265 = curve_1 + sin(x264) * ogeePhase_1;
	const GEN_FLT x266 = sin(x245);
	const GEN_FLT x267 = x266 * x265;
	const GEN_FLT x268 = x246 + x267 * x260;
	const GEN_FLT x269 = pow(x268, -1);
	const GEN_FLT x270 = pow(x249, 2);
	const GEN_FLT x271 = x270 * x265;
	const GEN_FLT x272 = x271 * x269;
	const GEN_FLT x273 = x263 + x272 * x254;
	const GEN_FLT x274 = pow(1 - pow(x273, 2), -1.0 / 2.0);
	const GEN_FLT x275 = x83 * x261;
	const GEN_FLT x276 = pow(1 - x96 * pow(x261, 2), -1.0 / 2.0);
	const GEN_FLT x277 = cos(x264) * ogeePhase_1;
	const GEN_FLT x278 = x277 * (x42 - x276 * x275);
	const GEN_FLT x279 = x270 * x269 * x254;
	const GEN_FLT x280 = pow(1 - x85 / pow(x246, 2), -1.0 / 2.0);
	const GEN_FLT x281 = x269 * x265 * x255;
	const GEN_FLT x282 = x280 * x281 * x247;
	const GEN_FLT x283 = x266 * x260;
	const GEN_FLT x284 = x89 * x247;
	const GEN_FLT x285 = x284 * x280;
	const GEN_FLT x286 = -x285 * x251;
	const GEN_FLT x287 = 2.40324066e-05 * x249;
	const GEN_FLT x288 = x280 * x252;
	const GEN_FLT x289 = x249 * (x286 + x285 * x250) - x288 * x284;
	const GEN_FLT x290 = -x285 * x253 + x289 * x249;
	const GEN_FLT x291 = x271 * x254 / pow(x268, 2);
	const GEN_FLT x292 =
		x274 * (x275 + x272 * x290 + x279 * x278 - x282 * x110 -
				x291 * (x267 * (x249 * (x290 + x249 * (x289 + x249 * (x286 - x285 * x256 + x287 * x285) - x285 * x257) -
										x285 * x258) -
								x285 * x254 - x285 * x259 + x290 * x249) +
						x278 * x283));
	const GEN_FLT x293 = cos(-gibPhase_1 + x115 + asin(x273)) * gibMag_1;
	const GEN_FLT x294 = x279 * x277;
	const GEN_FLT x295 = x276 * x262;
	const GEN_FLT x296 = x248 - x247 * x117;
	const GEN_FLT x297 = x296 * x280;
	const GEN_FLT x298 = 2 * x281;
	const GEN_FLT x299 = x277 * x283;
	const GEN_FLT x300 = x297 * x251;
	const GEN_FLT x301 = x249 * (x300 - x297 * x250) + x296 * x288;
	const GEN_FLT x302 = x249 * x301 + x297 * x253;
	const GEN_FLT x303 =
		x274 * (-x262 + x272 * x302 -
				x291 * (x267 * (x249 * x302 +
								x249 * (x302 + x249 * (x301 + x249 * (x300 + x297 * x256 - x297 * x287) + x297 * x257) +
										x297 * x258) +
								x297 * x254 + x297 * x259) +
						x299 * x295) +
				x295 * x294 + x297 * x298);
	const GEN_FLT x304 = x261 * x134;
	const GEN_FLT x305 = x127 - x276 * x304;
	const GEN_FLT x306 = x247 * x128;
	const GEN_FLT x307 = x280 * x306;
	const GEN_FLT x308 = -x251 * x307;
	const GEN_FLT x309 = x249 * (x308 + x250 * x307) - x288 * x306;
	const GEN_FLT x310 = x249 * x309 - x253 * x307;
	const GEN_FLT x311 =
		x274 * (x304 + x272 * x310 - x282 * x138 -
				x291 * (x267 * (x249 * x310 +
								x249 * (x310 + x249 * (x309 + x249 * (x308 - x256 * x307 + x287 * x307) - x257 * x307) -
										x258 * x307) -
								x254 * x307 - x259 * x307) +
						x299 * x305) +
				x294 * x305);
	const GEN_FLT x312 = x261 * x175 - x262 * x164;
	const GEN_FLT x313 = x157 - x276 * x312;
	const GEN_FLT x314 = -x247 * x168 + x248 * x164;
	const GEN_FLT x315 = x280 * x314;
	const GEN_FLT x316 = x251 * x315;
	const GEN_FLT x317 = x249 * (x316 - x250 * x315) + x288 * x314;
	const GEN_FLT x318 = x249 * x317 + x253 * x315;
	const GEN_FLT x319 =
		x274 * (x312 + x272 * x318 -
				x291 * (x267 * (x249 * x318 +
								x249 * (x318 + x249 * (x317 + x249 * (x316 + x256 * x315 - x287 * x315) + x257 * x315) +
										x258 * x315) +
								x254 * x315 + x259 * x315) +
						x299 * x313) +
				x294 * x313 + x298 * x315);
	const GEN_FLT x320 = x261 * x199 - x262 * x192;
	const GEN_FLT x321 = x190 - x276 * x320;
	const GEN_FLT x322 = -x247 * x194 + x248 * x192;
	const GEN_FLT x323 = x280 * x322;
	const GEN_FLT x324 = x251 * x323;
	const GEN_FLT x325 = x249 * (x324 - x250 * x323) + x288 * x322;
	const GEN_FLT x326 = x249 * x325 + x253 * x323;
	const GEN_FLT x327 =
		x274 * (x320 + x272 * x326 -
				x291 * (x267 * (x249 * x326 +
								x249 * (x326 + x249 * (x325 + x249 * (x324 + x256 * x323 - x287 * x323) + x257 * x323) +
										x258 * x323) +
								x254 * x323 + x259 * x323) +
						x299 * x321) +
				x294 * x321 + x298 * x323);
	const GEN_FLT x328 = -x213 * x262 + x261 * x221;
	const GEN_FLT x329 = x210 - x276 * x328;
	const GEN_FLT x330 = (x213 * x248 - x215 * x247) * x280;
	const GEN_FLT x331 = x251 * x330;
	const GEN_FLT x332 = x249 * (x331 - x250 * x330) + x252 * x330;
	const GEN_FLT x333 = x249 * x332 + x253 * x330;
	const GEN_FLT x334 =
		x274 * (x328 + x272 * x333 -
				x291 * (x267 * (x249 * x333 +
								x249 * (x333 + x249 * (x332 + x249 * (x331 + x256 * x330 - x287 * x330) + x257 * x330) +
										x258 * x330) +
								x254 * x330 + x259 * x330) +
						x299 * x329) +
				x294 * x329 + x298 * x330);
	const GEN_FLT x335 = -x232 * x262 + x261 * x240;
	const GEN_FLT x336 = x231 - x276 * x335;
	const GEN_FLT x337 = (x232 * x248 - x234 * x247) * x280;
	const GEN_FLT x338 = x251 * x337;
	const GEN_FLT x339 = x249 * (x338 - x250 * x337) + x252 * x337;
	const GEN_FLT x340 = x249 * x339 + x253 * x337;
	const GEN_FLT x341 =
		x274 * (x335 + x272 * x340 -
				x291 * (x267 * (x249 * x340 +
								x249 * (x340 + x249 * (x339 + x249 * (x338 + x256 * x337 - x287 * x337) + x257 * x337) +
										x258 * x337) +
								x254 * x337 + x259 * x337) +
						x299 * x336) +
				x294 * x336 + x298 * x337);
	out[0] = -x113 + x42 - (x113 + x114) * x116;
	out[1] = -x125 - x116 * x125;
	out[2] = x127 - x139 - (x126 + x139) * x116;
	out[3] = x157 - x178 - (x178 + x179) * x116;
	out[4] = x190 - x202 - (x202 + x203) * x116;
	out[5] = x210 - x224 - (x224 + x225) * x116;
	out[6] = x231 - x243 - (x243 + x244) * x116;
	out[7] = -x292 + x42 - (x114 + x292) * x293;
	out[8] = -x303 - x293 * x303;
	out[9] = x127 - x311 - (x126 + x311) * x293;
	out[10] = x157 - x319 - (x179 + x319) * x293;
	out[11] = x190 - x327 - (x203 + x327) * x293;
	out[12] = x210 - x334 - (x225 + x334) * x293;
	out[13] = x231 - x341 - (x244 + x341) * x293;
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
	const GEN_FLT x4 = pow(obj_qj, 2);
	const GEN_FLT x5 = pow(obj_qi, 2);
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = 2 * sqrt(x6 + x7 + pow(obj_qw, 2));
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = obj_pz + x11 * (x10 + x9) + (1 - x6 * x8) * sensor_z + (-x12 + x13) * x14;
	const GEN_FLT x16 = pow(lh_qi, 2);
	const GEN_FLT x17 = pow(lh_qk, 2);
	const GEN_FLT x18 = pow(lh_qj, 2);
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 2 * sqrt(x16 + x19 + pow(lh_qw, 2));
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + x22 * (x10 - x9) + (1 - (x5 + x7) * x8) * sensor_y + (x23 + x24) * x14;
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 = obj_px + (1 - (x4 + x7) * x8) * sensor_x + (x12 + x13) * x22 + (-x23 + x24) * x11;
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = lh_py + x25 * (1 - (x16 + x17) * x20) + (-x2 + x3) * x21 + (x26 + x27) * x29;
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = lh_pz + x15 * (1 - (x16 + x18) * x20) + (x2 + x3) * x31 + (-x32 + x33) * x29;
	const GEN_FLT x35 = lh_px + x28 * (1 - x20 * x19) + (-x26 + x27) * x31 + (x32 + x33) * x21;
	const GEN_FLT x36 = pow(x34, 2) + pow(x35, 2);
	const GEN_FLT x37 = x30 / sqrt(x36);
	const GEN_FLT x38 = x1 * x37;
	const GEN_FLT x39 = atan2(-x34, x35);
	const GEN_FLT x40 = ogeeMag_0 + x39 - asin(x38);
	const GEN_FLT x41 = sin(x40);
	const GEN_FLT x42 = curve_0 + x41 * ogeePhase_0;
	const GEN_FLT x43 = cos(x0);
	const GEN_FLT x44 = pow(x30, 2);
	const GEN_FLT x45 = x36 + x44;
	const GEN_FLT x46 = x30 / sqrt(x45);
	const GEN_FLT x47 = asin(x46 / x43);
	const GEN_FLT x48 = 8.0108022e-06 * x47;
	const GEN_FLT x49 = -8.0108022e-06 - x48;
	const GEN_FLT x50 = 0.0028679863 + x47 * x49;
	const GEN_FLT x51 = 5.3685255e-06 + x50 * x47;
	const GEN_FLT x52 = 0.0076069798 + x51 * x47;
	const GEN_FLT x53 = sin(x0);
	const GEN_FLT x54 = x52 * x47;
	const GEN_FLT x55 = -8.0108022e-06 - 1.60216044e-05 * x47;
	const GEN_FLT x56 = x50 + x55 * x47;
	const GEN_FLT x57 = x51 + x56 * x47;
	const GEN_FLT x58 = x52 + x57 * x47;
	const GEN_FLT x59 = x54 + x58 * x47;
	const GEN_FLT x60 = x59 * x42;
	const GEN_FLT x61 = x60 * x53;
	const GEN_FLT x62 = x43 - x61;
	const GEN_FLT x63 = pow(x62, -1);
	const GEN_FLT x64 = pow(x47, 2);
	const GEN_FLT x65 = x63 * x64;
	const GEN_FLT x66 = x65 * x52;
	const GEN_FLT x67 = x38 + x66 * x42;
	const GEN_FLT x68 = pow(1 - pow(x67, 2), -1.0 / 2.0);
	const GEN_FLT x69 = pow(x1, 2);
	const GEN_FLT x70 = x37 * (1 + x69);
	const GEN_FLT x71 = pow(x43, -2);
	const GEN_FLT x72 = x44 / x45;
	const GEN_FLT x73 = x71 * x46 / sqrt(1 - x71 * x72);
	const GEN_FLT x74 = x73 * x53;
	const GEN_FLT x75 = x74 * x49;
	const GEN_FLT x76 = x47 * (x75 - x74 * x48) + x74 * x50;
	const GEN_FLT x77 = x74 * x51 + x76 * x47;
	const GEN_FLT x78 = cos(x40) * ogeePhase_0;
	const GEN_FLT x79 = x44 / x36;
	const GEN_FLT x80 = x70 / sqrt(1 - x79 * x69);
	const GEN_FLT x81 = x53 * x42;
	const GEN_FLT x82 = x64 * x52 / pow(x62, 2);
	const GEN_FLT x83 = x78 * x66;
	const GEN_FLT x84 =
		x68 * (x70 - x80 * x83 + x77 * x65 * x42 -
			   x82 * x42 *
				   (-x53 - x60 * x43 -
					x81 * (x47 * (x77 + x47 * (x76 + x47 * (x75 - 2.40324066e-05 * x74 * x47 + x74 * x55) + x74 * x56) +
								  x74 * x57) +
						   x74 * x52 + x74 * x58 + x77 * x47) +
					x80 * x78 * x53 * x59) +
			   2 * x81 * x73 * x63 * x54);
	const GEN_FLT x85 = -x39;
	const GEN_FLT x86 = -gibPhase_0 + x85 + asin(x67);
	const GEN_FLT x87 = cos(x86) * gibMag_0;
	const GEN_FLT x88 = x82 * x61;
	const GEN_FLT x89 = (x66 + x88) * x68;
	const GEN_FLT x90 = x68 * (x83 + x88 * x78);
	const GEN_FLT x91 = (x66 * x41 + x88 * x41) * x68;
	const GEN_FLT x92 = 0.523598775598299 - tilt_1;
	const GEN_FLT x93 = tan(x92);
	const GEN_FLT x94 = -x93 * x37;
	const GEN_FLT x95 = ogeeMag_1 + x39 - asin(x94);
	const GEN_FLT x96 = sin(x95);
	const GEN_FLT x97 = curve_1 + x96 * ogeePhase_1;
	const GEN_FLT x98 = cos(x92);
	const GEN_FLT x99 = asin(x46 / x98);
	const GEN_FLT x100 = 8.0108022e-06 * x99;
	const GEN_FLT x101 = -8.0108022e-06 - x100;
	const GEN_FLT x102 = 0.0028679863 + x99 * x101;
	const GEN_FLT x103 = 5.3685255e-06 + x99 * x102;
	const GEN_FLT x104 = 0.0076069798 + x99 * x103;
	const GEN_FLT x105 = pow(x99, 2);
	const GEN_FLT x106 = x99 * x104;
	const GEN_FLT x107 = -8.0108022e-06 - 1.60216044e-05 * x99;
	const GEN_FLT x108 = x102 + x99 * x107;
	const GEN_FLT x109 = x103 + x99 * x108;
	const GEN_FLT x110 = x104 + x99 * x109;
	const GEN_FLT x111 = x106 + x99 * x110;
	const GEN_FLT x112 = sin(x92);
	const GEN_FLT x113 = x97 * x112;
	const GEN_FLT x114 = x111 * x113;
	const GEN_FLT x115 = x114 + x98;
	const GEN_FLT x116 = pow(x115, -1);
	const GEN_FLT x117 = x105 * x116;
	const GEN_FLT x118 = x104 * x117;
	const GEN_FLT x119 = x94 + x97 * x118;
	const GEN_FLT x120 = pow(1 - pow(x119, 2), -1.0 / 2.0);
	const GEN_FLT x121 = pow(x93, 2);
	const GEN_FLT x122 = x37 * (1 + x121);
	const GEN_FLT x123 = cos(x95) * ogeePhase_1;
	const GEN_FLT x124 = x118 * x123;
	const GEN_FLT x125 = x122 / sqrt(1 - x79 * x121);
	const GEN_FLT x126 = pow(x98, -2);
	const GEN_FLT x127 = x46 * x126 / sqrt(1 - x72 * x126);
	const GEN_FLT x128 = x112 * x127;
	const GEN_FLT x129 = -x101 * x128;
	const GEN_FLT x130 = -x102 * x128 + x99 * (x129 + x100 * x128);
	const GEN_FLT x131 = -x103 * x128 + x99 * x130;
	const GEN_FLT x132 = x105 * x104 / pow(x115, 2);
	const GEN_FLT x133 =
		x120 *
		(x122 - x124 * x125 + x97 * x117 * x131 -
		 x97 * x132 *
			 (x112 +
			  x113 * (-x104 * x128 - x110 * x128 + x99 * x131 +
					  x99 * (x131 - x109 * x128 +
							 x99 * (x130 - x108 * x128 + x99 * (x129 - x107 * x128 + 2.40324066e-05 * x99 * x128)))) -
			  x98 * x97 * x111 - x111 * x112 * x123 * x125) -
		 2 * x106 * x113 * x116 * x127);
	const GEN_FLT x134 = -gibPhase_1 + x85 + asin(x119);
	const GEN_FLT x135 = cos(x134) * gibMag_1;
	const GEN_FLT x136 = x114 * x132;
	const GEN_FLT x137 = (x118 - x136) * x120;
	const GEN_FLT x138 = x120 * (x124 - x123 * x136);
	const GEN_FLT x139 = (x96 * x118 - x96 * x136) * x120;
	out[0] = -1;
	out[1] = -x84 - x84 * x87;
	out[2] = -x89 - x89 * x87;
	out[3] = x87;
	out[4] = -sin(x86);
	out[5] = -x90 - x87 * x90;
	out[6] = -x91 - x87 * x91;
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
	out[22] = -x133 - x133 * x135;
	out[23] = -x137 - x137 * x135;
	out[24] = x135;
	out[25] = -sin(x134);
	out[26] = -x138 - x138 * x135;
	out[27] = -x139 - x135 * x139;
}

/** Applying function <function reproject_axis_x_gen2 at 0x7effc26e3a70> */
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
	const GEN_FLT x4 = pow(obj_qj, 2);
	const GEN_FLT x5 = pow(obj_qi, 2);
	const GEN_FLT x6 = pow(obj_qk, 2);
	const GEN_FLT x7 = x5 + x6;
	const GEN_FLT x8 = 2 * sqrt(x4 + x7 + pow(obj_qw, 2));
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = obj_pz + x11 * (x10 + x9) + (1 - (x4 + x5) * x8) * sensor_z + (-x12 + x13) * x14;
	const GEN_FLT x16 = pow(lh_qi, 2);
	const GEN_FLT x17 = pow(lh_qk, 2);
	const GEN_FLT x18 = pow(lh_qj, 2);
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 2 * sqrt(x16 + x19 + pow(lh_qw, 2));
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + x22 * (x10 - x9) + (1 - x8 * x7) * sensor_y + (x23 + x24) * x14;
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 = obj_px + (1 - (x4 + x6) * x8) * sensor_x + (x12 + x13) * x22 + (-x23 + x24) * x11;
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = lh_py + x25 * (1 - (x16 + x17) * x20) + (-x2 + x3) * x21 + (x26 + x27) * x29;
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = lh_pz + x15 * (1 - (x16 + x18) * x20) + (x2 + x3) * x31 + (-x32 + x33) * x29;
	const GEN_FLT x35 = lh_px + x28 * (1 - x20 * x19) + (-x26 + x27) * x31 + (x32 + x33) * x21;
	const GEN_FLT x36 = pow(x34, 2) + pow(x35, 2);
	const GEN_FLT x37 = asin(x30 / (x1 * sqrt(x36 + pow(x30, 2))));
	const GEN_FLT x38 = 0.0028679863 + x37 * (-8.0108022e-06 - 8.0108022e-06 * x37);
	const GEN_FLT x39 = 5.3685255e-06 + x38 * x37;
	const GEN_FLT x40 = 0.0076069798 + x37 * x39;
	const GEN_FLT x41 = tan(x0) * x30 / sqrt(x36);
	const GEN_FLT x42 = atan2(-x34, x35);
	const GEN_FLT x43 = curve_0 + sin(ogeeMag_0 + x42 - asin(x41)) * ogeePhase_0;
	const GEN_FLT x44 = asin(
		x41 + x40 * x43 * pow(x37, 2) /
				  (x1 - sin(x0) * x43 *
							(x37 * (x40 + x37 * (x39 + x37 * (x38 + x37 * (-8.0108022e-06 - 1.60216044e-05 * x37)))) +
							 x40 * x37)));
	out[0] = -1.5707963267949 - phase_0 + x42 - x44 + sin(gibPhase_0 + x42 - x44) * gibMag_0;
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
	const GEN_FLT x0 = pow(obj_qj, 2);
	const GEN_FLT x1 = pow(obj_qi, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(obj_qk, 2);
	const GEN_FLT x4 = x1 + x3;
	const GEN_FLT x5 = sqrt(x0 + x4 + pow(obj_qw, 2));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x7 + x8;
	const GEN_FLT x10 = x6 * sensor_y;
	const GEN_FLT x11 = obj_qw * obj_qj;
	const GEN_FLT x12 = obj_qk * obj_qi;
	const GEN_FLT x13 = -x11 + x12;
	const GEN_FLT x14 = x6 * sensor_x;
	const GEN_FLT x15 = obj_pz + x14 * x13 + x9 * x10 + (1 - x2 * x6) * sensor_z;
	const GEN_FLT x16 = lh_qw * lh_qi;
	const GEN_FLT x17 = lh_qk * lh_qj;
	const GEN_FLT x18 = -x16 + x17;
	const GEN_FLT x19 = pow(lh_qj, 2);
	const GEN_FLT x20 = pow(lh_qk, 2);
	const GEN_FLT x21 = pow(lh_qi, 2);
	const GEN_FLT x22 = x20 + x21;
	const GEN_FLT x23 = sqrt(x19 + x22 + pow(lh_qw, 2));
	const GEN_FLT x24 = 2 * x23;
	const GEN_FLT x25 = x24 * x18;
	const GEN_FLT x26 = 1 - x24 * x22;
	const GEN_FLT x27 = -x7 + x8;
	const GEN_FLT x28 = x6 * sensor_z;
	const GEN_FLT x29 = obj_qw * obj_qk;
	const GEN_FLT x30 = obj_qj * obj_qi;
	const GEN_FLT x31 = x29 + x30;
	const GEN_FLT x32 = obj_py + x28 * x27 + x31 * x14 + (1 - x4 * x6) * sensor_y;
	const GEN_FLT x33 = x11 + x12;
	const GEN_FLT x34 = -x29 + x30;
	const GEN_FLT x35 = x0 + x3;
	const GEN_FLT x36 = obj_px + x33 * x28 + x34 * x10 + (1 - x6 * x35) * sensor_x;
	const GEN_FLT x37 = lh_qw * lh_qk;
	const GEN_FLT x38 = lh_qj * lh_qi;
	const GEN_FLT x39 = x37 + x38;
	const GEN_FLT x40 = x39 * x24;
	const GEN_FLT x41 = lh_py + x25 * x15 + x32 * x26 + x40 * x36;
	const GEN_FLT x42 = 0.523598775598299 + tilt_0;
	const GEN_FLT x43 = cos(x42);
	const GEN_FLT x44 = pow(x43, -1);
	const GEN_FLT x45 = pow(x41, 2);
	const GEN_FLT x46 = 1 - (x19 + x21) * x24;
	const GEN_FLT x47 = x16 + x17;
	const GEN_FLT x48 = x47 * x24;
	const GEN_FLT x49 = lh_qw * lh_qj;
	const GEN_FLT x50 = lh_qk * lh_qi;
	const GEN_FLT x51 = -x49 + x50;
	const GEN_FLT x52 = x51 * x24;
	const GEN_FLT x53 = lh_pz + x46 * x15 + x48 * x32 + x52 * x36;
	const GEN_FLT x54 = x49 + x50;
	const GEN_FLT x55 = x54 * x24;
	const GEN_FLT x56 = -x37 + x38;
	const GEN_FLT x57 = x56 * x24;
	const GEN_FLT x58 = 1 - (x19 + x20) * x24;
	const GEN_FLT x59 = lh_px + x55 * x15 + x57 * x32 + x58 * x36;
	const GEN_FLT x60 = pow(x59, 2);
	const GEN_FLT x61 = x60 + pow(x53, 2);
	const GEN_FLT x62 = x45 + x61;
	const GEN_FLT x63 = x44 / sqrt(x62);
	const GEN_FLT x64 = asin(x63 * x41);
	const GEN_FLT x65 = 8.0108022e-06 * x64;
	const GEN_FLT x66 = -8.0108022e-06 - x65;
	const GEN_FLT x67 = 0.0028679863 + x64 * x66;
	const GEN_FLT x68 = 5.3685255e-06 + x64 * x67;
	const GEN_FLT x69 = 0.0076069798 + x64 * x68;
	const GEN_FLT x70 = x64 * x69;
	const GEN_FLT x71 = -8.0108022e-06 - 1.60216044e-05 * x64;
	const GEN_FLT x72 = x67 + x71 * x64;
	const GEN_FLT x73 = x68 + x72 * x64;
	const GEN_FLT x74 = x69 + x73 * x64;
	const GEN_FLT x75 = x70 + x74 * x64;
	const GEN_FLT x76 = sin(x42);
	const GEN_FLT x77 = tan(x42);
	const GEN_FLT x78 = x77 / sqrt(x61);
	const GEN_FLT x79 = x78 * x41;
	const GEN_FLT x80 = atan2(-x53, x59);
	const GEN_FLT x81 = ogeeMag_0 + x80 - asin(x79);
	const GEN_FLT x82 = curve_0 + sin(x81) * ogeePhase_0;
	const GEN_FLT x83 = x82 * x76;
	const GEN_FLT x84 = x43 - x83 * x75;
	const GEN_FLT x85 = pow(x84, -1);
	const GEN_FLT x86 = pow(x64, 2);
	const GEN_FLT x87 = x82 * x86;
	const GEN_FLT x88 = x85 * x87;
	const GEN_FLT x89 = x79 + x88 * x69;
	const GEN_FLT x90 = pow(1 - pow(x89, 2), -1.0 / 2.0);
	const GEN_FLT x91 = pow(1 - x45 / (x62 * pow(x43, 2)), -1.0 / 2.0);
	const GEN_FLT x92 = 4 * x23;
	const GEN_FLT x93 = x92 * x41;
	const GEN_FLT x94 = 2 * x59;
	const GEN_FLT x95 = x53 * x92;
	const GEN_FLT x96 = x51 * x95 + x58 * x94;
	const GEN_FLT x97 = (1.0 / 2.0) * x41;
	const GEN_FLT x98 = x97 * x44 / pow(x62, 3.0 / 2.0);
	const GEN_FLT x99 = x63 * x40 - x98 * (x96 + x93 * x39);
	const GEN_FLT x100 = x91 * x99;
	const GEN_FLT x101 = x66 * x100;
	const GEN_FLT x102 = x67 * x91;
	const GEN_FLT x103 = x64 * (x101 - x65 * x100) + x99 * x102;
	const GEN_FLT x104 = x64 * x103 + x68 * x100;
	const GEN_FLT x105 = pow(x60, -1);
	const GEN_FLT x106 = x53 * x105;
	const GEN_FLT x107 = pow(x59, -1);
	const GEN_FLT x108 = pow(x61, -1);
	const GEN_FLT x109 = x60 * x108;
	const GEN_FLT x110 = (-x52 * x107 + x58 * x106) * x109;
	const GEN_FLT x111 = pow(1 - pow(x77, 2) * x45 * x108, -1.0 / 2.0);
	const GEN_FLT x112 = x77 * x97 / pow(x61, 3.0 / 2.0);
	const GEN_FLT x113 = x78 * x40 - x96 * x112;
	const GEN_FLT x114 = x110 - x111 * x113;
	const GEN_FLT x115 = cos(x81) * ogeePhase_0;
	const GEN_FLT x116 = x75 * x76;
	const GEN_FLT x117 = x115 * x116;
	const GEN_FLT x118 = 2.40324066e-05 * x64;
	const GEN_FLT x119 = x87 * x69 / pow(x84, 2);
	const GEN_FLT x120 = x85 * x86 * x69;
	const GEN_FLT x121 = x115 * x120;
	const GEN_FLT x122 = 2 * x82 * x85 * x70;
	const GEN_FLT x123 =
		x90 *
		(x113 + x100 * x122 + x114 * x121 -
		 x119 *
			 (-x114 * x117 -
			  x83 * (x64 * x104 +
					 x64 * (x104 + x64 * (x103 + x64 * (x101 - x100 * x118 + x71 * x100) + x72 * x100) + x73 * x100) +
					 x69 * x100 + x74 * x100)) +
		 x88 * x104);
	const GEN_FLT x124 = cos(gibPhase_0 + x80 - asin(x89)) * gibMag_0;
	const GEN_FLT x125 = 2 * x53;
	const GEN_FLT x126 = x23 * x105 * x125;
	const GEN_FLT x127 = (-x48 * x107 + x56 * x126) * x109;
	const GEN_FLT x128 = 2 * x41;
	const GEN_FLT x129 = x59 * x92;
	const GEN_FLT x130 = x56 * x129 + x95 * x47;
	const GEN_FLT x131 = x63 * x26 - x98 * (x130 + x26 * x128);
	const GEN_FLT x132 = x91 * x131;
	const GEN_FLT x133 = x66 * x132;
	const GEN_FLT x134 = x102 * x131 + x64 * (x133 - x65 * x132);
	const GEN_FLT x135 = x64 * x134 + x68 * x132;
	const GEN_FLT x136 = -x112 * x130 + x78 * x26;
	const GEN_FLT x137 = x115 * (x127 - x111 * x136);
	const GEN_FLT x138 =
		x90 *
		(x136 -
		 x119 *
			 (-x116 * x137 -
			  x83 * (x64 * x135 +
					 x64 * (x135 + x64 * (x134 + x64 * (x133 - x118 * x132 + x71 * x132) + x72 * x132) + x73 * x132) +
					 x69 * x132 + x74 * x132)) +
		 x120 * x137 + x122 * x132 + x88 * x135);
	const GEN_FLT x139 = (-x46 * x107 + x54 * x126) * x109;
	const GEN_FLT x140 = x46 * x125 + x54 * x129;
	const GEN_FLT x141 = x63 * x25 - x98 * (x140 + x93 * x18);
	const GEN_FLT x142 = x91 * x141;
	const GEN_FLT x143 = x66 * x142;
	const GEN_FLT x144 = x102 * x141 + x64 * (x143 - x65 * x142);
	const GEN_FLT x145 = x64 * x144 + x68 * x142;
	const GEN_FLT x146 = -x112 * x140 + x78 * x25;
	const GEN_FLT x147 = x139 - x111 * x146;
	const GEN_FLT x148 =
		x90 *
		(x146 -
		 x119 *
			 (-x117 * x147 -
			  x83 * (x64 * x145 +
					 x64 * (x145 + x64 * (x144 + x64 * (x143 - x118 * x142 + x71 * x142) + x72 * x142) + x73 * x142) +
					 x69 * x142 + x74 * x142)) +
		 x121 * x147 + x122 * x142 + x88 * x145);
	const GEN_FLT x149 = 2 / x5;
	const GEN_FLT x150 = x149 * obj_qw;
	const GEN_FLT x151 = x35 * sensor_x;
	const GEN_FLT x152 = x34 * sensor_y;
	const GEN_FLT x153 = x10 * obj_qk;
	const GEN_FLT x154 = x33 * sensor_z;
	const GEN_FLT x155 = x28 * obj_qj;
	const GEN_FLT x156 = -x153 + x155 - x150 * x151 + x150 * x152 + x150 * x154;
	const GEN_FLT x157 = x31 * sensor_x;
	const GEN_FLT x158 = x4 * sensor_y;
	const GEN_FLT x159 = x14 * obj_qk;
	const GEN_FLT x160 = x27 * sensor_z;
	const GEN_FLT x161 = x28 * obj_qi;
	const GEN_FLT x162 = x159 - x161 + x150 * x157 - x150 * x158 + x160 * x150;
	const GEN_FLT x163 = x14 * obj_qj;
	const GEN_FLT x164 = x13 * sensor_x;
	const GEN_FLT x165 = x9 * sensor_y;
	const GEN_FLT x166 = x10 * obj_qi;
	const GEN_FLT x167 = x2 * sensor_z;
	const GEN_FLT x168 = -x163 + x166 + x164 * x150 + x165 * x150 - x167 * x150;
	const GEN_FLT x169 = x25 * x168 + x26 * x162 + x40 * x156;
	const GEN_FLT x170 = x55 * x168 + x57 * x162 + x58 * x156;
	const GEN_FLT x171 = x46 * x168 + x48 * x162 + x52 * x156;
	const GEN_FLT x172 = x125 * x171 + x94 * x170;
	const GEN_FLT x173 = x63 * x169 - x98 * (x172 + x128 * x169);
	const GEN_FLT x174 = x91 * x173;
	const GEN_FLT x175 = x66 * x174;
	const GEN_FLT x176 = x102 * x173 + x64 * (x175 - x65 * x174);
	const GEN_FLT x177 = x64 * x176 + x68 * x174;
	const GEN_FLT x178 = (x106 * x170 - x107 * x171) * x109;
	const GEN_FLT x179 = -x112 * x172 + x78 * x169;
	const GEN_FLT x180 = x178 - x111 * x179;
	const GEN_FLT x181 =
		x90 *
		(x179 -
		 x119 *
			 (-x117 * x180 -
			  x83 * (x64 * x177 +
					 x64 * (x177 + x64 * (x176 + x64 * (x175 - x118 * x174 + x71 * x174) + x72 * x174) + x73 * x174) +
					 x69 * x174 + x74 * x174)) +
		 x121 * x180 + x122 * x174 + x88 * x177);
	const GEN_FLT x182 = x149 * obj_qi;
	const GEN_FLT x183 = x10 * obj_qj;
	const GEN_FLT x184 = x28 * obj_qk;
	const GEN_FLT x185 = x183 + x184 - x182 * x151 + x182 * x152 + x182 * x154;
	const GEN_FLT x186 = 4 * x5;
	const GEN_FLT x187 = -x186 * obj_qi;
	const GEN_FLT x188 = x28 * obj_qw;
	const GEN_FLT x189 = x163 - x188 + x160 * x182 + x182 * x157 + (x187 - x4 * x182) * sensor_y;
	const GEN_FLT x190 = x10 * obj_qw;
	const GEN_FLT x191 = x159 + x190 + x164 * x182 + x165 * x182 + (x187 - x2 * x182) * sensor_z;
	const GEN_FLT x192 = x55 * x191 + x57 * x189 + x58 * x185;
	const GEN_FLT x193 = x46 * x191 + x48 * x189 + x52 * x185;
	const GEN_FLT x194 = (x106 * x192 - x107 * x193) * x109;
	const GEN_FLT x195 = x25 * x191 + x26 * x189 + x40 * x185;
	const GEN_FLT x196 = x125 * x193 + x94 * x192;
	const GEN_FLT x197 = x91 * (x63 * x195 - x98 * (x196 + x128 * x195));
	const GEN_FLT x198 = x66 * x197;
	const GEN_FLT x199 = x64 * (x198 - x65 * x197) + x67 * x197;
	const GEN_FLT x200 = x64 * x199 + x68 * x197;
	const GEN_FLT x201 = -x112 * x196 + x78 * x195;
	const GEN_FLT x202 = x194 - x201 * x111;
	const GEN_FLT x203 =
		x90 *
		(x201 -
		 x119 *
			 (-x202 * x117 -
			  x83 * (x64 * x200 +
					 x64 * (x200 + x64 * (x199 + x64 * (x198 - x118 * x197 + x71 * x197) + x72 * x197) + x73 * x197) +
					 x69 * x197 + x74 * x197)) +
		 x122 * x197 + x202 * x121 + x88 * x200);
	const GEN_FLT x204 = x149 * obj_qj;
	const GEN_FLT x205 = -x186 * obj_qj;
	const GEN_FLT x206 = x166 + x188 + x204 * x152 + x204 * x154 + (x205 - x35 * x204) * sensor_x;
	const GEN_FLT x207 = x14 * obj_qi;
	const GEN_FLT x208 = x184 + x207 + x204 * x157 - x204 * x158 + x204 * x160;
	const GEN_FLT x209 = x14 * obj_qw;
	const GEN_FLT x210 = x153 - x209 + x204 * x164 + x204 * x165 + (x205 - x2 * x204) * sensor_z;
	const GEN_FLT x211 = x55 * x210 + x57 * x208 + x58 * x206;
	const GEN_FLT x212 = x46 * x210 + x48 * x208 + x52 * x206;
	const GEN_FLT x213 = (x211 * x106 - x212 * x107) * x109;
	const GEN_FLT x214 = x25 * x210 + x26 * x208 + x40 * x206;
	const GEN_FLT x215 = x212 * x125 + x94 * x211;
	const GEN_FLT x216 = x91 * (x63 * x214 - x98 * (x215 + x214 * x128));
	const GEN_FLT x217 = x66 * x216;
	const GEN_FLT x218 = x64 * (x217 - x65 * x216) + x67 * x216;
	const GEN_FLT x219 = x64 * x218 + x68 * x216;
	const GEN_FLT x220 = -x215 * x112 + x78 * x214;
	const GEN_FLT x221 = x213 - x220 * x111;
	const GEN_FLT x222 =
		x90 *
		(x220 -
		 x119 *
			 (-x221 * x117 -
			  x83 * (x64 * x219 +
					 x64 * (x219 + x64 * (x218 + x64 * (x217 - x216 * x118 + x71 * x216) + x72 * x216) + x73 * x216) +
					 x69 * x216 + x74 * x216)) +
		 x216 * x122 + x221 * x121 + x88 * x219);
	const GEN_FLT x223 = x149 * obj_qk;
	const GEN_FLT x224 = -x186 * obj_qk;
	const GEN_FLT x225 = x161 - x190 + x223 * x152 + x223 * x154 + (x224 - x35 * x223) * sensor_x;
	const GEN_FLT x226 = x155 + x209 + x223 * x157 + x223 * x160 + (x224 - x4 * x223) * sensor_y;
	const GEN_FLT x227 = x183 + x207 + x223 * x164 + x223 * x165 - x223 * x167;
	const GEN_FLT x228 = x55 * x227 + x57 * x226 + x58 * x225;
	const GEN_FLT x229 = x46 * x227 + x48 * x226 + x52 * x225;
	const GEN_FLT x230 = (x228 * x106 - x229 * x107) * x109;
	const GEN_FLT x231 = x25 * x227 + x26 * x226 + x40 * x225;
	const GEN_FLT x232 = x229 * x125 + x94 * x228;
	const GEN_FLT x233 = x91 * (x63 * x231 - x98 * (x232 + x231 * x128));
	const GEN_FLT x234 = x66 * x233;
	const GEN_FLT x235 = x64 * (x234 - x65 * x233) + x67 * x233;
	const GEN_FLT x236 = x64 * x235 + x68 * x233;
	const GEN_FLT x237 = -x232 * x112 + x78 * x231;
	const GEN_FLT x238 = x230 - x237 * x111;
	const GEN_FLT x239 =
		x90 *
		(x237 -
		 x119 *
			 (-x238 * x117 -
			  x83 * (x64 * x236 +
					 x64 * (x236 + x64 * (x235 + x64 * (x234 - x233 * x118 + x71 * x233) + x72 * x233) + x73 * x233) +
					 x69 * x233 + x74 * x233)) +
		 x233 * x122 + x238 * x121 + x88 * x236);
	out[0] = x110 - x123 - (-x110 + x123) * x124;
	out[1] = x127 - x138 - (-x127 + x138) * x124;
	out[2] = x139 - x148 - (-x139 + x148) * x124;
	out[3] = x178 - x181 - (-x178 + x181) * x124;
	out[4] = x194 - x203 - (-x194 + x203) * x124;
	out[5] = x213 - x222 - (-x213 + x222) * x124;
	out[6] = x230 - x239 - (-x230 + x239) * x124;
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt(x1 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - (x0 + x1) * x5;
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = pow(obj_qj, 2);
	const GEN_FLT x9 = pow(obj_qi, 2);
	const GEN_FLT x10 = x7 + x9;
	const GEN_FLT x11 = sqrt(x10 + x8 + pow(obj_qw, 2));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 - (x7 + x8) * x12;
	const GEN_FLT x14 = lh_qw * lh_qk;
	const GEN_FLT x15 = lh_qj * lh_qi;
	const GEN_FLT x16 = -x14 + x15;
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
	const GEN_FLT x27 = -x25 + x26;
	const GEN_FLT x28 = x20 * x27;
	const GEN_FLT x29 = x21 * x16 + x24 * x28 + x6 * x13;
	const GEN_FLT x30 = 1 - (x1 + x2) * x5;
	const GEN_FLT x31 = 1 - (x8 + x9) * x12;
	const GEN_FLT x32 = obj_qw * obj_qi;
	const GEN_FLT x33 = obj_qk * obj_qj;
	const GEN_FLT x34 = x32 + x33;
	const GEN_FLT x35 = x34 * x12;
	const GEN_FLT x36 = x27 * x12;
	const GEN_FLT x37 = obj_pz + x31 * sensor_z + x35 * sensor_y + x36 * sensor_x;
	const GEN_FLT x38 = lh_qw * lh_qi;
	const GEN_FLT x39 = lh_qk * lh_qj;
	const GEN_FLT x40 = x38 + x39;
	const GEN_FLT x41 = -x32 + x33;
	const GEN_FLT x42 = x41 * x12;
	const GEN_FLT x43 = 1 - x12 * x10;
	const GEN_FLT x44 = x12 * x19;
	const GEN_FLT x45 = obj_py + x42 * sensor_z + x43 * sensor_y + x44 * sensor_x;
	const GEN_FLT x46 = x5 * x45;
	const GEN_FLT x47 = -x22 + x23;
	const GEN_FLT x48 = x25 + x26;
	const GEN_FLT x49 = -x17 + x18;
	const GEN_FLT x50 = obj_px + x13 * sensor_x + x48 * x12 * sensor_z + x49 * x12 * sensor_y;
	const GEN_FLT x51 = x5 * x50;
	const GEN_FLT x52 = lh_pz + x30 * x37 + x40 * x46 + x51 * x47;
	const GEN_FLT x53 = x5 * x37;
	const GEN_FLT x54 = lh_px + x46 * x16 + x53 * x24 + x6 * x50;
	const GEN_FLT x55 = pow(x54, 2);
	const GEN_FLT x56 = x52 / x55;
	const GEN_FLT x57 = pow(x54, -1);
	const GEN_FLT x58 = x5 * x13;
	const GEN_FLT x59 = x30 * x36 + x40 * x21 + x58 * x47;
	const GEN_FLT x60 = x55 + pow(x52, 2);
	const GEN_FLT x61 = pow(x60, -1);
	const GEN_FLT x62 = x61 * x55;
	const GEN_FLT x63 = (x56 * x29 - x57 * x59) * x62;
	const GEN_FLT x64 = -x38 + x39;
	const GEN_FLT x65 = 1 - x3 * x5;
	const GEN_FLT x66 = x14 + x15;
	const GEN_FLT x67 = lh_py + x64 * x53 + x65 * x45 + x66 * x51;
	const GEN_FLT x68 = 0.523598775598299 + tilt_0;
	const GEN_FLT x69 = cos(x68);
	const GEN_FLT x70 = pow(x69, -1);
	const GEN_FLT x71 = pow(x67, 2);
	const GEN_FLT x72 = x60 + x71;
	const GEN_FLT x73 = x70 / sqrt(x72);
	const GEN_FLT x74 = asin(x73 * x67);
	const GEN_FLT x75 = 8.0108022e-06 * x74;
	const GEN_FLT x76 = -8.0108022e-06 - x75;
	const GEN_FLT x77 = 0.0028679863 + x74 * x76;
	const GEN_FLT x78 = 5.3685255e-06 + x74 * x77;
	const GEN_FLT x79 = 0.0076069798 + x78 * x74;
	const GEN_FLT x80 = x79 * x74;
	const GEN_FLT x81 = -8.0108022e-06 - 1.60216044e-05 * x74;
	const GEN_FLT x82 = x77 + x81 * x74;
	const GEN_FLT x83 = x78 + x82 * x74;
	const GEN_FLT x84 = x79 + x83 * x74;
	const GEN_FLT x85 = x80 + x84 * x74;
	const GEN_FLT x86 = sin(x68);
	const GEN_FLT x87 = tan(x68);
	const GEN_FLT x88 = x87 / sqrt(x60);
	const GEN_FLT x89 = x88 * x67;
	const GEN_FLT x90 = atan2(-x52, x54);
	const GEN_FLT x91 = ogeeMag_0 + x90 - asin(x89);
	const GEN_FLT x92 = curve_0 + sin(x91) * ogeePhase_0;
	const GEN_FLT x93 = x86 * x92;
	const GEN_FLT x94 = x69 - x85 * x93;
	const GEN_FLT x95 = pow(x94, -1);
	const GEN_FLT x96 = pow(x74, 2);
	const GEN_FLT x97 = x92 * x96;
	const GEN_FLT x98 = x97 * x95;
	const GEN_FLT x99 = x89 + x79 * x98;
	const GEN_FLT x100 = pow(1 - pow(x99, 2), -1.0 / 2.0);
	const GEN_FLT x101 = pow(1 - x71 / (x72 * pow(x69, 2)), -1.0 / 2.0);
	const GEN_FLT x102 = x64 * x28 + x65 * x44 + x66 * x58;
	const GEN_FLT x103 = 2 * x67;
	const GEN_FLT x104 = 2 * x54;
	const GEN_FLT x105 = 2 * x52;
	const GEN_FLT x106 = x29 * x104 + x59 * x105;
	const GEN_FLT x107 = (1.0 / 2.0) * x67;
	const GEN_FLT x108 = x70 * x107 / pow(x72, 3.0 / 2.0);
	const GEN_FLT x109 = x101 * (-x108 * (x106 + x103 * x102) + x73 * x102);
	const GEN_FLT x110 = x76 * x109;
	const GEN_FLT x111 = x74 * (x110 - x75 * x109) + x77 * x109;
	const GEN_FLT x112 = x74 * x111 + x78 * x109;
	const GEN_FLT x113 = 2 * x80 * x92 * x95;
	const GEN_FLT x114 = x87 * x107 / pow(x60, 3.0 / 2.0);
	const GEN_FLT x115 = -x106 * x114 + x88 * x102;
	const GEN_FLT x116 = pow(1 - pow(x87, 2) * x71 * x61, -1.0 / 2.0);
	const GEN_FLT x117 = cos(x91) * ogeePhase_0;
	const GEN_FLT x118 = x117 * (x63 - x115 * x116);
	const GEN_FLT x119 = x85 * x86;
	const GEN_FLT x120 = 2.40324066e-05 * x74;
	const GEN_FLT x121 = x79 * x97 / pow(x94, 2);
	const GEN_FLT x122 = x79 * x96 * x95;
	const GEN_FLT x123 =
		x100 *
		(x115 + x109 * x113 + x118 * x122 -
		 x121 *
			 (-x118 * x119 -
			  x93 * (x74 * x112 +
					 x74 * (x112 + x74 * (x111 + x74 * (x110 - x109 * x120 + x81 * x109) + x82 * x109) + x83 * x109) +
					 x79 * x109 + x84 * x109)) +
		 x98 * x112);
	const GEN_FLT x124 = cos(gibPhase_0 + x90 - asin(x99)) * gibMag_0;
	const GEN_FLT x125 = x5 * x43;
	const GEN_FLT x126 = x6 * x12;
	const GEN_FLT x127 = x34 * x20;
	const GEN_FLT x128 = x16 * x125 + x24 * x127 + x49 * x126;
	const GEN_FLT x129 = x49 * x20;
	const GEN_FLT x130 = x30 * x35 + x40 * x125 + x47 * x129;
	const GEN_FLT x131 = (x56 * x128 - x57 * x130) * x62;
	const GEN_FLT x132 = x64 * x127 + x65 * x43 + x66 * x129;
	const GEN_FLT x133 = x104 * x128 + x105 * x130;
	const GEN_FLT x134 = x101 * (-x108 * (x133 + x103 * x132) + x73 * x132);
	const GEN_FLT x135 = x76 * x134;
	const GEN_FLT x136 = x74 * (x135 - x75 * x134) + x77 * x134;
	const GEN_FLT x137 = x74 * x136 + x78 * x134;
	const GEN_FLT x138 = -x114 * x133 + x88 * x132;
	const GEN_FLT x139 = x131 - x116 * x138;
	const GEN_FLT x140 = x119 * x117;
	const GEN_FLT x141 = x117 * x122;
	const GEN_FLT x142 =
		x100 *
		(x138 + x113 * x134 -
		 x121 *
			 (-x139 * x140 -
			  x93 * (x74 * x137 +
					 x74 * (x137 + x74 * (x136 + x74 * (x135 - x120 * x134 + x81 * x134) + x82 * x134) + x83 * x134) +
					 x79 * x134 + x84 * x134)) +
		 x139 * x141 + x98 * x137);
	const GEN_FLT x143 = x41 * x20;
	const GEN_FLT x144 = x5 * x31;
	const GEN_FLT x145 = x16 * x143 + x24 * x144 + x48 * x126;
	const GEN_FLT x146 = x48 * x20;
	const GEN_FLT x147 = x30 * x31 + x40 * x143 + x47 * x146;
	const GEN_FLT x148 = (x56 * x145 - x57 * x147) * x62;
	const GEN_FLT x149 = x64 * x144 + x65 * x42 + x66 * x146;
	const GEN_FLT x150 = x104 * x145 + x105 * x147;
	const GEN_FLT x151 = x101 * (-x108 * (x150 + x103 * x149) + x73 * x149);
	const GEN_FLT x152 = x76 * x151;
	const GEN_FLT x153 = x74 * (x152 - x75 * x151) + x77 * x151;
	const GEN_FLT x154 = x74 * x153 + x78 * x151;
	const GEN_FLT x155 = -x114 * x150 + x88 * x149;
	const GEN_FLT x156 = x148 - x116 * x155;
	const GEN_FLT x157 =
		x100 *
		(x155 + x113 * x151 -
		 x121 *
			 (-x140 * x156 -
			  x93 * (x74 * x154 +
					 x74 * (x154 + x74 * (x153 + x74 * (x152 - x120 * x151 + x81 * x151) + x82 * x151) + x83 * x151) +
					 x79 * x151 + x84 * x151)) +
		 x141 * x156 + x98 * x154);
	out[0] = -x123 + x63 - x124 * (x123 - x63);
	out[1] = x131 - x142 - (-x131 + x142) * x124;
	out[2] = x148 - x157 - (-x148 + x157) * x124;
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
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qk, 2);
	const GEN_FLT x4 = x1 + x3;
	const GEN_FLT x5 = sqrt(x0 + x4 + pow(lh_qw, 2));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = pow(obj_qk, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = 2 * sqrt(x10 + x7 + pow(obj_qw, 2));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 = obj_pz + (1 - (x7 + x8) * x11) * sensor_z + (x12 + x13) * x14 + (-x15 + x16) * x17;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + (1 - x11 * x10) * sensor_y + (-x12 + x13) * x22 + (x23 + x24) * x17;
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = -x27 + x28;
	const GEN_FLT x30 = obj_px + (1 - (x7 + x9) * x11) * sensor_x + (x15 + x16) * x22 + (-x23 + x24) * x14;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + x18 * (1 - x2 * x6) + x21 * x26 + x31 * x29;
	const GEN_FLT x33 = x27 + x28;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = -x35 + x36;
	const GEN_FLT x38 = x0 + x3;
	const GEN_FLT x39 = lh_px + x30 * (1 - x6 * x38) + x34 * x33 + x37 * x26;
	const GEN_FLT x40 = pow(x39, 2);
	const GEN_FLT x41 = x40 + pow(x32, 2);
	const GEN_FLT x42 = pow(x41, -1);
	const GEN_FLT x43 = x42 * x32;
	const GEN_FLT x44 = -x19 + x20;
	const GEN_FLT x45 = x35 + x36;
	const GEN_FLT x46 = lh_py + x25 * (1 - x4 * x6) + x44 * x34 + x45 * x31;
	const GEN_FLT x47 = 0.523598775598299 + tilt_0;
	const GEN_FLT x48 = cos(x47);
	const GEN_FLT x49 = pow(x48, -1);
	const GEN_FLT x50 = pow(x46, 2);
	const GEN_FLT x51 = x41 + x50;
	const GEN_FLT x52 = x49 / sqrt(x51);
	const GEN_FLT x53 = asin(x52 * x46);
	const GEN_FLT x54 = 8.0108022e-06 * x53;
	const GEN_FLT x55 = -8.0108022e-06 - x54;
	const GEN_FLT x56 = 0.0028679863 + x53 * x55;
	const GEN_FLT x57 = 5.3685255e-06 + x53 * x56;
	const GEN_FLT x58 = 0.0076069798 + x53 * x57;
	const GEN_FLT x59 = x53 * x58;
	const GEN_FLT x60 = -8.0108022e-06 - 1.60216044e-05 * x53;
	const GEN_FLT x61 = x56 + x60 * x53;
	const GEN_FLT x62 = x57 + x61 * x53;
	const GEN_FLT x63 = x58 + x62 * x53;
	const GEN_FLT x64 = x59 + x63 * x53;
	const GEN_FLT x65 = sin(x47);
	const GEN_FLT x66 = tan(x47);
	const GEN_FLT x67 = x66 / sqrt(x41);
	const GEN_FLT x68 = x67 * x46;
	const GEN_FLT x69 = atan2(-x32, x39);
	const GEN_FLT x70 = ogeeMag_0 + x69 - asin(x68);
	const GEN_FLT x71 = curve_0 + sin(x70) * ogeePhase_0;
	const GEN_FLT x72 = x71 * x65;
	const GEN_FLT x73 = x48 - x72 * x64;
	const GEN_FLT x74 = pow(x73, -1);
	const GEN_FLT x75 = pow(x53, 2);
	const GEN_FLT x76 = x71 * x75;
	const GEN_FLT x77 = x74 * x76;
	const GEN_FLT x78 = x68 + x77 * x58;
	const GEN_FLT x79 = pow(1 - pow(x78, 2), -1.0 / 2.0);
	const GEN_FLT x80 = x66 / pow(x41, 3.0 / 2.0);
	const GEN_FLT x81 = x80 * x46;
	const GEN_FLT x82 = x81 * x39;
	const GEN_FLT x83 = pow(1 - x50 / (x51 * pow(x48, 2)), -1.0 / 2.0);
	const GEN_FLT x84 = x49 / pow(x51, 3.0 / 2.0);
	const GEN_FLT x85 = x84 * x46;
	const GEN_FLT x86 = x85 * x39;
	const GEN_FLT x87 = x83 * x86;
	const GEN_FLT x88 = -x87 * x55;
	const GEN_FLT x89 = x53 * (x88 + x87 * x54) - x87 * x56;
	const GEN_FLT x90 = -x87 * x57 + x89 * x53;
	const GEN_FLT x91 = pow(1 - pow(x66, 2) * x50 * x42, -1.0 / 2.0);
	const GEN_FLT x92 = cos(x70) * ogeePhase_0;
	const GEN_FLT x93 = x92 * (x43 + x82 * x91);
	const GEN_FLT x94 = x64 * x65;
	const GEN_FLT x95 = 2.40324066e-05 * x53;
	const GEN_FLT x96 = x83 * x60;
	const GEN_FLT x97 = x83 * x62;
	const GEN_FLT x98 = x83 * x58;
	const GEN_FLT x99 = x76 * x58 / pow(x73, 2);
	const GEN_FLT x100 = x75 * x74 * x58;
	const GEN_FLT x101 = 2 * x39;
	const GEN_FLT x102 = x71 * x74 * x59;
	const GEN_FLT x103 = x83 * x85 * x102;
	const GEN_FLT x104 =
		x79 * (-x82 - x101 * x103 + x77 * x90 + x93 * x100 -
			   x99 * (-x72 * (x53 * x90 +
							  x53 * (x90 + x53 * (x89 + x53 * (x88 - x86 * x96 + x87 * x95) - x87 * x61) - x86 * x97) -
							  x86 * x98 - x87 * x63) -
					  x93 * x94));
	const GEN_FLT x105 = cos(gibPhase_0 + x69 - asin(x78)) * gibMag_0;
	const GEN_FLT x106 = x83 * (x52 - x84 * x50);
	const GEN_FLT x107 = x55 * x106;
	const GEN_FLT x108 = x53 * (x107 - x54 * x106) + x56 * x106;
	const GEN_FLT x109 = x53 * x108 + x57 * x106;
	const GEN_FLT x110 = x92 * x94;
	const GEN_FLT x111 = x67 * x91;
	const GEN_FLT x112 = x92 * x100;
	const GEN_FLT x113 = 2 * x102;
	const GEN_FLT x114 =
		x79 *
		(x67 + x106 * x113 - x111 * x112 + x77 * x109 -
		 x99 * (x110 * x111 -
				x72 * (x53 * x109 +
					   x53 * (x109 + x53 * (x108 + x53 * (x107 + x60 * x106 - x95 * x106) + x61 * x106) + x62 * x106) +
					   x58 * x106 + x63 * x106)));
	const GEN_FLT x115 = x42 * x39;
	const GEN_FLT x116 = -x115;
	const GEN_FLT x117 = x46 * x32;
	const GEN_FLT x118 = x84 * x117;
	const GEN_FLT x119 = x83 * x118;
	const GEN_FLT x120 = -x55 * x119;
	const GEN_FLT x121 = x53 * (x120 + x54 * x119) - x56 * x119;
	const GEN_FLT x122 = x53 * x121 - x57 * x119;
	const GEN_FLT x123 = x80 * x117;
	const GEN_FLT x124 = x116 + x91 * x123;
	const GEN_FLT x125 = 2 * x32;
	const GEN_FLT x126 =
		x79 *
		(-x123 - x103 * x125 + x112 * x124 + x77 * x122 -
		 x99 * (-x110 * x124 -
				x72 * (x53 * x122 +
					   x53 * (x122 + x53 * (x121 + x53 * (x120 + x95 * x119 - x96 * x118) - x61 * x119) - x97 * x118) -
					   x63 * x119 - x98 * x118)));
	const GEN_FLT x127 = 2 / x5;
	const GEN_FLT x128 = x38 * x127;
	const GEN_FLT x129 = x30 * x128;
	const GEN_FLT x130 = x127 * lh_qw;
	const GEN_FLT x131 = x37 * x25;
	const GEN_FLT x132 = x26 * lh_qk;
	const GEN_FLT x133 = x33 * x18;
	const GEN_FLT x134 = x34 * lh_qj;
	const GEN_FLT x135 = -x132 + x134 - x129 * lh_qw + x130 * x131 + x130 * x133;
	const GEN_FLT x136 = x32 / x40;
	const GEN_FLT x137 = pow(x39, -1);
	const GEN_FLT x138 = x30 * x29;
	const GEN_FLT x139 = x25 * x21;
	const GEN_FLT x140 = x26 * lh_qi;
	const GEN_FLT x141 = x31 * lh_qj;
	const GEN_FLT x142 = x2 * x127;
	const GEN_FLT x143 = x18 * x142;
	const GEN_FLT x144 = x140 - x141 + x130 * x138 + x130 * x139 - x143 * lh_qw;
	const GEN_FLT x145 = x40 * x42;
	const GEN_FLT x146 = (x135 * x136 - x137 * x144) * x145;
	const GEN_FLT x147 = x45 * x30;
	const GEN_FLT x148 = x127 * x147;
	const GEN_FLT x149 = x31 * lh_qk;
	const GEN_FLT x150 = x4 * x127;
	const GEN_FLT x151 = x25 * x150;
	const GEN_FLT x152 = x44 * x18;
	const GEN_FLT x153 = x34 * lh_qi;
	const GEN_FLT x154 = x149 - x153 + x130 * x152 + x148 * lh_qw - x151 * lh_qw;
	const GEN_FLT x155 = 2 * x46;
	const GEN_FLT x156 = x101 * x135 + x125 * x144;
	const GEN_FLT x157 = (1.0 / 2.0) * x85;
	const GEN_FLT x158 = -x157 * (x156 + x154 * x155) + x52 * x154;
	const GEN_FLT x159 = x83 * x158;
	const GEN_FLT x160 = x55 * x159;
	const GEN_FLT x161 = x53 * (x160 - x54 * x159) + x56 * x159;
	const GEN_FLT x162 = x53 * x161 + x57 * x159;
	const GEN_FLT x163 = (1.0 / 2.0) * x81;
	const GEN_FLT x164 = -x163 * x156 + x67 * x154;
	const GEN_FLT x165 = x146 - x91 * x164;
	const GEN_FLT x166 =
		x79 *
		(x164 + x112 * x165 + x113 * x159 + x77 * x162 -
		 x99 * (-x110 * x165 -
				x72 * (x53 * x162 +
					   x53 * (x162 + x53 * (x161 + x53 * (x160 + x60 * x159 - x95 * x159) + x61 * x159) + x97 * x158) +
					   x63 * x159 + x98 * x158)));
	const GEN_FLT x167 = x127 * lh_qi;
	const GEN_FLT x168 = x26 * lh_qj;
	const GEN_FLT x169 = x34 * lh_qk;
	const GEN_FLT x170 = x168 + x169 - x129 * lh_qi + x167 * x131 + x167 * x133;
	const GEN_FLT x171 = x26 * lh_qw;
	const GEN_FLT x172 = 4 * x5;
	const GEN_FLT x173 = -x172 * lh_qi;
	const GEN_FLT x174 = x149 + x171 + x167 * x138 + x167 * x139 + x18 * (x173 - x142 * lh_qi);
	const GEN_FLT x175 = (x170 * x136 - x174 * x137) * x145;
	const GEN_FLT x176 = x34 * lh_qw;
	const GEN_FLT x177 = x141 - x176 + x148 * lh_qi + x167 * x152 + x25 * (x173 - x150 * lh_qi);
	const GEN_FLT x178 = x101 * x170 + x125 * x174;
	const GEN_FLT x179 = -x157 * (x178 + x177 * x155) + x52 * x177;
	const GEN_FLT x180 = x83 * x179;
	const GEN_FLT x181 = x55 * x180;
	const GEN_FLT x182 = x53 * (x181 - x54 * x180) + x56 * x180;
	const GEN_FLT x183 = x53 * x182 + x57 * x180;
	const GEN_FLT x184 = -x163 * x178 + x67 * x177;
	const GEN_FLT x185 = x175 - x91 * x184;
	const GEN_FLT x186 =
		x79 *
		(x184 + x112 * x185 + x113 * x180 + x77 * x183 -
		 x99 * (-x110 * x185 -
				x72 * (x53 * x183 +
					   x53 * (x183 + x53 * (x182 + x53 * (x181 + x60 * x180 - x95 * x180) + x61 * x180) + x97 * x179) +
					   x63 * x180 + x98 * x179)));
	const GEN_FLT x187 = -x172 * lh_qj;
	const GEN_FLT x188 = x127 * lh_qj;
	const GEN_FLT x189 = x140 + x176 + x188 * x131 + x188 * x133 + x30 * (x187 - x128 * lh_qj);
	const GEN_FLT x190 = x31 * lh_qw;
	const GEN_FLT x191 = x132 - x190 + x18 * (x187 - x142 * lh_qj) + x188 * x138 + x188 * x139;
	const GEN_FLT x192 = (x189 * x136 - x191 * x137) * x145;
	const GEN_FLT x193 = x31 * lh_qi;
	const GEN_FLT x194 = x169 + x193 + x148 * lh_qj - x151 * lh_qj + x188 * x152;
	const GEN_FLT x195 = x101 * x189 + x125 * x191;
	const GEN_FLT x196 = -x157 * (x195 + x194 * x155) + x52 * x194;
	const GEN_FLT x197 = x83 * x196;
	const GEN_FLT x198 = x55 * x197;
	const GEN_FLT x199 = x53 * (x198 - x54 * x197) + x56 * x197;
	const GEN_FLT x200 = x53 * x199 + x57 * x197;
	const GEN_FLT x201 = -x163 * x195 + x67 * x194;
	const GEN_FLT x202 = x192 - x91 * x201;
	const GEN_FLT x203 =
		x79 *
		(x201 + x113 * x197 + x202 * x112 + x77 * x200 -
		 x99 * (-x202 * x110 -
				x72 * (x53 * x200 +
					   x53 * (x200 + x53 * (x199 + x53 * (x198 + x60 * x197 - x95 * x197) + x61 * x197) + x97 * x196) +
					   x63 * x197 + x98 * x196)));
	const GEN_FLT x204 = -x172 * lh_qk;
	const GEN_FLT x205 = x127 * lh_qk;
	const GEN_FLT x206 = x18 * x205;
	const GEN_FLT x207 = x153 - x171 + x205 * x131 + x30 * (x204 - x128 * lh_qk) + x33 * x206;
	const GEN_FLT x208 = x168 + x193 - x143 * lh_qk + x205 * x138 + x205 * x139;
	const GEN_FLT x209 = (x207 * x136 - x208 * x137) * x145;
	const GEN_FLT x210 = x134 + x190 + x205 * x147 + x25 * (x204 - x150 * lh_qk) + x44 * x206;
	const GEN_FLT x211 = x207 * x101 + x208 * x125;
	const GEN_FLT x212 = -x157 * (x211 + x210 * x155) + x52 * x210;
	const GEN_FLT x213 = x83 * x212;
	const GEN_FLT x214 = x55 * x213;
	const GEN_FLT x215 = x53 * (x214 - x54 * x213) + x56 * x213;
	const GEN_FLT x216 = x53 * x215 + x57 * x213;
	const GEN_FLT x217 = -x211 * x163 + x67 * x210;
	const GEN_FLT x218 = x209 - x91 * x217;
	const GEN_FLT x219 =
		x79 *
		(x217 + x213 * x113 + x218 * x112 + x77 * x216 -
		 x99 * (-x218 * x110 -
				x72 * (x53 * x216 +
					   x53 * (x216 + x53 * (x215 + x53 * (x214 + x60 * x213 - x95 * x213) + x61 * x213) + x97 * x212) +
					   x63 * x213 + x98 * x212)));
	out[0] = -x104 + x43 - x105 * (x104 - x43);
	out[1] = -x114 - x105 * x114;
	out[2] = x116 - x126 - (x115 + x126) * x105;
	out[3] = x146 - x166 - (-x146 + x166) * x105;
	out[4] = x175 - x186 - (-x175 + x186) * x105;
	out[5] = x192 - x203 - (-x192 + x203) * x105;
	out[6] = x209 - x219 - (-x209 + x219) * x105;
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
	const GEN_FLT x4 = pow(obj_qj, 2);
	const GEN_FLT x5 = pow(obj_qi, 2);
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = 2 * sqrt(x6 + x7 + pow(obj_qw, 2));
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = obj_pz + x11 * (x10 + x9) + (1 - x6 * x8) * sensor_z + (-x12 + x13) * x14;
	const GEN_FLT x16 = pow(lh_qi, 2);
	const GEN_FLT x17 = pow(lh_qk, 2);
	const GEN_FLT x18 = pow(lh_qj, 2);
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 2 * sqrt(x16 + x19 + pow(lh_qw, 2));
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + x22 * (x10 - x9) + (1 - (x5 + x7) * x8) * sensor_y + (x23 + x24) * x14;
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 = obj_px + (1 - (x4 + x7) * x8) * sensor_x + (x12 + x13) * x22 + (-x23 + x24) * x11;
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = lh_py + x25 * (1 - (x16 + x17) * x20) + (-x2 + x3) * x21 + (x26 + x27) * x29;
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = lh_pz + x15 * (1 - (x16 + x18) * x20) + (x2 + x3) * x31 + (-x32 + x33) * x29;
	const GEN_FLT x35 = lh_px + x28 * (1 - x20 * x19) + (-x26 + x27) * x31 + (x32 + x33) * x21;
	const GEN_FLT x36 = pow(x34, 2) + pow(x35, 2);
	const GEN_FLT x37 = x30 / sqrt(x36);
	const GEN_FLT x38 = x1 * x37;
	const GEN_FLT x39 = atan2(-x34, x35);
	const GEN_FLT x40 = ogeeMag_0 + x39 - asin(x38);
	const GEN_FLT x41 = sin(x40);
	const GEN_FLT x42 = curve_0 + x41 * ogeePhase_0;
	const GEN_FLT x43 = cos(x0);
	const GEN_FLT x44 = pow(x30, 2);
	const GEN_FLT x45 = x36 + x44;
	const GEN_FLT x46 = x30 / sqrt(x45);
	const GEN_FLT x47 = asin(x46 / x43);
	const GEN_FLT x48 = 8.0108022e-06 * x47;
	const GEN_FLT x49 = -8.0108022e-06 - x48;
	const GEN_FLT x50 = 0.0028679863 + x47 * x49;
	const GEN_FLT x51 = 5.3685255e-06 + x50 * x47;
	const GEN_FLT x52 = 0.0076069798 + x51 * x47;
	const GEN_FLT x53 = sin(x0);
	const GEN_FLT x54 = x52 * x47;
	const GEN_FLT x55 = -8.0108022e-06 - 1.60216044e-05 * x47;
	const GEN_FLT x56 = x50 + x55 * x47;
	const GEN_FLT x57 = x51 + x56 * x47;
	const GEN_FLT x58 = x52 + x57 * x47;
	const GEN_FLT x59 = x54 + x58 * x47;
	const GEN_FLT x60 = x59 * x42;
	const GEN_FLT x61 = x60 * x53;
	const GEN_FLT x62 = x43 - x61;
	const GEN_FLT x63 = pow(x62, -1);
	const GEN_FLT x64 = pow(x47, 2);
	const GEN_FLT x65 = x63 * x64;
	const GEN_FLT x66 = x65 * x52;
	const GEN_FLT x67 = x38 + x66 * x42;
	const GEN_FLT x68 = pow(1 - pow(x67, 2), -1.0 / 2.0);
	const GEN_FLT x69 = pow(x1, 2);
	const GEN_FLT x70 = x37 * (1 + x69);
	const GEN_FLT x71 = pow(x43, -2);
	const GEN_FLT x72 = x71 * x46 / sqrt(1 - x71 * x44 / x45);
	const GEN_FLT x73 = x72 * x53;
	const GEN_FLT x74 = x73 * x49;
	const GEN_FLT x75 = x47 * (x74 - x73 * x48) + x73 * x50;
	const GEN_FLT x76 = x73 * x51 + x75 * x47;
	const GEN_FLT x77 = cos(x40) * ogeePhase_0;
	const GEN_FLT x78 = x70 / sqrt(1 - x69 * x44 / x36);
	const GEN_FLT x79 = x53 * x42;
	const GEN_FLT x80 = x64 * x52 / pow(x62, 2);
	const GEN_FLT x81 = x77 * x66;
	const GEN_FLT x82 =
		x68 * (x70 - x81 * x78 + x76 * x65 * x42 -
			   x80 * x42 *
				   (-x53 - x60 * x43 -
					x79 * (x47 * (x76 + x47 * (x75 + x47 * (x74 - 2.40324066e-05 * x73 * x47 + x73 * x55) + x73 * x56) +
								  x73 * x57) +
						   x73 * x52 + x73 * x58 + x76 * x47) +
					x78 * x77 * x53 * x59) +
			   2 * x72 * x79 * x63 * x54);
	const GEN_FLT x83 = -gibPhase_0 - x39 + asin(x67);
	const GEN_FLT x84 = cos(x83) * gibMag_0;
	const GEN_FLT x85 = x80 * x61;
	const GEN_FLT x86 = (x66 + x85) * x68;
	const GEN_FLT x87 = x68 * (x81 + x85 * x77);
	const GEN_FLT x88 = (x66 * x41 + x85 * x41) * x68;
	out[0] = -1;
	out[1] = -x82 - x82 * x84;
	out[2] = -x86 - x84 * x86;
	out[3] = x84;
	out[4] = -sin(x83);
	out[5] = -x87 - x84 * x87;
	out[6] = -x88 - x88 * x84;
}

/** Applying function <function reproject_axis_y_gen2 at 0x7effc26e3b00> */
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
	const GEN_FLT x0 = 0.523598775598299 - tilt_1;
	const GEN_FLT x1 = lh_qw * lh_qi;
	const GEN_FLT x2 = lh_qk * lh_qj;
	const GEN_FLT x3 = pow(obj_qj, 2);
	const GEN_FLT x4 = pow(obj_qi, 2);
	const GEN_FLT x5 = pow(obj_qk, 2);
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = 2 * sqrt(x3 + x6 + pow(obj_qw, 2));
	const GEN_FLT x8 = obj_qw * obj_qi;
	const GEN_FLT x9 = obj_qk * obj_qj;
	const GEN_FLT x10 = x7 * sensor_y;
	const GEN_FLT x11 = obj_qw * obj_qj;
	const GEN_FLT x12 = obj_qk * obj_qi;
	const GEN_FLT x13 = x7 * sensor_x;
	const GEN_FLT x14 = obj_pz + (1 - (x3 + x4) * x7) * sensor_z + (-x11 + x12) * x13 + (x8 + x9) * x10;
	const GEN_FLT x15 = pow(lh_qk, 2);
	const GEN_FLT x16 = pow(lh_qj, 2);
	const GEN_FLT x17 = pow(lh_qi, 2);
	const GEN_FLT x18 = x16 + x17;
	const GEN_FLT x19 = 2 * sqrt(x15 + x18 + pow(lh_qw, 2));
	const GEN_FLT x20 = x14 * x19;
	const GEN_FLT x21 = x7 * sensor_z;
	const GEN_FLT x22 = obj_qw * obj_qk;
	const GEN_FLT x23 = obj_qj * obj_qi;
	const GEN_FLT x24 = obj_py + (1 - x6 * x7) * sensor_y + (x22 + x23) * x13 + (-x8 + x9) * x21;
	const GEN_FLT x25 = lh_qw * lh_qk;
	const GEN_FLT x26 = lh_qj * lh_qi;
	const GEN_FLT x27 = obj_px + (1 - (x3 + x5) * x7) * sensor_x + (x11 + x12) * x21 + (-x22 + x23) * x10;
	const GEN_FLT x28 = x27 * x19;
	const GEN_FLT x29 = lh_py + x24 * (1 - (x15 + x17) * x19) + (-x1 + x2) * x20 + (x25 + x26) * x28;
	const GEN_FLT x30 = x24 * x19;
	const GEN_FLT x31 = lh_qw * lh_qj;
	const GEN_FLT x32 = lh_qk * lh_qi;
	const GEN_FLT x33 = lh_pz + x14 * (1 - x19 * x18) + (x1 + x2) * x30 + (-x31 + x32) * x28;
	const GEN_FLT x34 = lh_px + x27 * (1 - (x15 + x16) * x19) + (-x25 + x26) * x30 + (x31 + x32) * x20;
	const GEN_FLT x35 = pow(x33, 2) + pow(x34, 2);
	const GEN_FLT x36 = -tan(x0) * x29 / sqrt(x35);
	const GEN_FLT x37 = atan2(-x33, x34);
	const GEN_FLT x38 = curve_1 + sin(ogeeMag_1 + x37 - asin(x36)) * ogeePhase_1;
	const GEN_FLT x39 = cos(x0);
	const GEN_FLT x40 = asin(x29 / (x39 * sqrt(x35 + pow(x29, 2))));
	const GEN_FLT x41 = 0.0028679863 + x40 * (-8.0108022e-06 - 8.0108022e-06 * x40);
	const GEN_FLT x42 = 5.3685255e-06 + x40 * x41;
	const GEN_FLT x43 = 0.0076069798 + x40 * x42;
	const GEN_FLT x44 =
		asin(x36 +
			 pow(x40, 2) * x43 * x38 /
				 (x39 + sin(x0) * x38 *
							(x40 * x43 +
							 x40 * (x43 + x40 * (x42 + x40 * (x41 + x40 * (-8.0108022e-06 - 1.60216044e-05 * x40)))))));
	out[0] = -1.5707963267949 - phase_1 + x37 - x44 - sin(-gibPhase_1 - x37 + x44) * gibMag_1;
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qi, 2);
	const GEN_FLT x4 = sqrt(x2 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - x2 * x5;
	const GEN_FLT x7 = 1 - (x1 + x3) * x5;
	const GEN_FLT x8 = pow(obj_qj, 2);
	const GEN_FLT x9 = pow(obj_qi, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = x11 + x8;
	const GEN_FLT x13 = sqrt(x12 + x9 + pow(obj_qw, 2));
	const GEN_FLT x14 = 2 * x13;
	const GEN_FLT x15 = obj_qw * obj_qi;
	const GEN_FLT x16 = obj_qk * obj_qj;
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = x14 * sensor_y;
	const GEN_FLT x19 = obj_qw * obj_qj;
	const GEN_FLT x20 = obj_qk * obj_qi;
	const GEN_FLT x21 = -x19 + x20;
	const GEN_FLT x22 = x14 * sensor_x;
	const GEN_FLT x23 = obj_pz + x18 * x17 + x22 * x21 + (1 - x14 * x10) * sensor_z;
	const GEN_FLT x24 = -x15 + x16;
	const GEN_FLT x25 = x14 * sensor_z;
	const GEN_FLT x26 = x11 + x9;
	const GEN_FLT x27 = obj_qw * obj_qk;
	const GEN_FLT x28 = obj_qj * obj_qi;
	const GEN_FLT x29 = x27 + x28;
	const GEN_FLT x30 = obj_py + x22 * x29 + x24 * x25 + (1 - x26 * x14) * sensor_y;
	const GEN_FLT x31 = lh_qw * lh_qi;
	const GEN_FLT x32 = lh_qk * lh_qj;
	const GEN_FLT x33 = x31 + x32;
	const GEN_FLT x34 = x5 * x33;
	const GEN_FLT x35 = x19 + x20;
	const GEN_FLT x36 = -x27 + x28;
	const GEN_FLT x37 = obj_px + x35 * x25 + x36 * x18 + (1 - x14 * x12) * sensor_x;
	const GEN_FLT x38 = lh_qw * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qi;
	const GEN_FLT x40 = -x38 + x39;
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = lh_pz + x30 * x34 + x41 * x37 + x7 * x23;
	const GEN_FLT x43 = x38 + x39;
	const GEN_FLT x44 = x5 * x23;
	const GEN_FLT x45 = lh_qw * lh_qk;
	const GEN_FLT x46 = lh_qj * lh_qi;
	const GEN_FLT x47 = -x45 + x46;
	const GEN_FLT x48 = x5 * x47;
	const GEN_FLT x49 = lh_px + x43 * x44 + x48 * x30 + x6 * x37;
	const GEN_FLT x50 = pow(x49, 2);
	const GEN_FLT x51 = pow(x50, -1);
	const GEN_FLT x52 = x51 * x42;
	const GEN_FLT x53 = pow(x49, -1);
	const GEN_FLT x54 = x50 + pow(x42, 2);
	const GEN_FLT x55 = pow(x54, -1);
	const GEN_FLT x56 = x50 * x55;
	const GEN_FLT x57 = x56 * (-x53 * x41 + x6 * x52);
	const GEN_FLT x58 = -x31 + x32;
	const GEN_FLT x59 = 1 - (x0 + x3) * x5;
	const GEN_FLT x60 = x45 + x46;
	const GEN_FLT x61 = x5 * x60;
	const GEN_FLT x62 = lh_py + x58 * x44 + x59 * x30 + x61 * x37;
	const GEN_FLT x63 = 0.523598775598299 - tilt_1;
	const GEN_FLT x64 = cos(x63);
	const GEN_FLT x65 = pow(x64, -1);
	const GEN_FLT x66 = pow(x62, 2);
	const GEN_FLT x67 = x54 + x66;
	const GEN_FLT x68 = x65 / sqrt(x67);
	const GEN_FLT x69 = asin(x62 * x68);
	const GEN_FLT x70 = 8.0108022e-06 * x69;
	const GEN_FLT x71 = -8.0108022e-06 - x70;
	const GEN_FLT x72 = 0.0028679863 + x71 * x69;
	const GEN_FLT x73 = 5.3685255e-06 + x72 * x69;
	const GEN_FLT x74 = 0.0076069798 + x73 * x69;
	const GEN_FLT x75 = x74 * x69;
	const GEN_FLT x76 = -8.0108022e-06 - 1.60216044e-05 * x69;
	const GEN_FLT x77 = x72 + x76 * x69;
	const GEN_FLT x78 = x73 + x77 * x69;
	const GEN_FLT x79 = x74 + x78 * x69;
	const GEN_FLT x80 = x75 + x79 * x69;
	const GEN_FLT x81 = tan(x63);
	const GEN_FLT x82 = x81 / sqrt(x54);
	const GEN_FLT x83 = -x82 * x62;
	const GEN_FLT x84 = atan2(-x42, x49);
	const GEN_FLT x85 = ogeeMag_1 + x84 - asin(x83);
	const GEN_FLT x86 = curve_1 + sin(x85) * ogeePhase_1;
	const GEN_FLT x87 = sin(x63);
	const GEN_FLT x88 = x86 * x87;
	const GEN_FLT x89 = x64 + x80 * x88;
	const GEN_FLT x90 = pow(x89, -1);
	const GEN_FLT x91 = pow(x69, 2);
	const GEN_FLT x92 = x86 * x91;
	const GEN_FLT x93 = x92 * x90;
	const GEN_FLT x94 = x83 + x74 * x93;
	const GEN_FLT x95 = pow(1 - pow(x94, 2), -1.0 / 2.0);
	const GEN_FLT x96 = pow(1 - pow(x81, 2) * x66 * x55, -1.0 / 2.0);
	const GEN_FLT x97 = 2 * x49;
	const GEN_FLT x98 = 4 * x4;
	const GEN_FLT x99 = x98 * x42;
	const GEN_FLT x100 = x6 * x97 + x99 * x40;
	const GEN_FLT x101 = (1.0 / 2.0) * x62;
	const GEN_FLT x102 = x81 * x101 / pow(x54, 3.0 / 2.0);
	const GEN_FLT x103 = x100 * x102 - x82 * x61;
	const GEN_FLT x104 = x57 - x96 * x103;
	const GEN_FLT x105 = cos(x85) * ogeePhase_1;
	const GEN_FLT x106 = x74 * x91 * x90;
	const GEN_FLT x107 = x105 * x106;
	const GEN_FLT x108 = pow(1 - x66 / (pow(x64, 2) * x67), -1.0 / 2.0);
	const GEN_FLT x109 = x62 * x98;
	const GEN_FLT x110 = x65 * x101 / pow(x67, 3.0 / 2.0);
	const GEN_FLT x111 = x108 * (-x110 * (x100 + x60 * x109) + x61 * x68);
	const GEN_FLT x112 = 2 * x86 * x75 * x90;
	const GEN_FLT x113 = x80 * x87;
	const GEN_FLT x114 = x105 * x113;
	const GEN_FLT x115 = x71 * x111;
	const GEN_FLT x116 = 2.40324066e-05 * x69;
	const GEN_FLT x117 = x69 * (x115 - x70 * x111) + x72 * x111;
	const GEN_FLT x118 = x69 * x117 + x73 * x111;
	const GEN_FLT x119 = x74 * x92 / pow(x89, 2);
	const GEN_FLT x120 =
		x95 *
		(x103 + x104 * x107 + x111 * x112 -
		 x119 *
			 (x104 * x114 +
			  x88 * (x69 * x118 +
					 x69 * (x118 + x69 * (x117 + x69 * (x115 - x111 * x116 + x76 * x111) + x77 * x111) + x78 * x111) +
					 x74 * x111 + x79 * x111)) +
		 x93 * x118);
	const GEN_FLT x121 = cos(gibPhase_1 + x84 - asin(x94)) * gibMag_1;
	const GEN_FLT x122 = 2 * x42;
	const GEN_FLT x123 = x4 * x51 * x122;
	const GEN_FLT x124 = x56 * (x47 * x123 - x53 * x34);
	const GEN_FLT x125 = x98 * x49;
	const GEN_FLT x126 = x47 * x125 + x99 * x33;
	const GEN_FLT x127 = x102 * x126 - x82 * x59;
	const GEN_FLT x128 = x105 * (x124 - x96 * x127);
	const GEN_FLT x129 = 2 * x62;
	const GEN_FLT x130 = x108 * (-x110 * (x126 + x59 * x129) + x68 * x59);
	const GEN_FLT x131 = x71 * x130;
	const GEN_FLT x132 = x69 * (x131 - x70 * x130) + x72 * x130;
	const GEN_FLT x133 = x69 * x132 + x73 * x130;
	const GEN_FLT x134 =
		x95 *
		(x127 + x106 * x128 + x112 * x130 -
		 x119 *
			 (x113 * x128 +
			  x88 * (x69 * x133 +
					 x69 * (x133 + x69 * (x132 + x69 * (x131 - x116 * x130 + x76 * x130) + x77 * x130) + x78 * x130) +
					 x74 * x130 + x79 * x130)) +
		 x93 * x133);
	const GEN_FLT x135 = x56 * (x43 * x123 - x7 * x53);
	const GEN_FLT x136 = x43 * x125 + x7 * x122;
	const GEN_FLT x137 = x5 * x58;
	const GEN_FLT x138 = x102 * x136 - x82 * x137;
	const GEN_FLT x139 = x135 - x96 * x138;
	const GEN_FLT x140 = x108 * (-x110 * (x136 + x58 * x109) + x68 * x137);
	const GEN_FLT x141 = x71 * x140;
	const GEN_FLT x142 = x69 * (x141 - x70 * x140) + x72 * x140;
	const GEN_FLT x143 = x69 * x142 + x73 * x140;
	const GEN_FLT x144 =
		x95 *
		(x138 + x107 * x139 + x112 * x140 -
		 x119 *
			 (x114 * x139 +
			  x88 * (x69 * x143 +
					 x69 * (x143 + x69 * (x142 + x69 * (x141 - x116 * x140 + x76 * x140) + x77 * x140) + x78 * x140) +
					 x74 * x140 + x79 * x140)) +
		 x93 * x143);
	const GEN_FLT x145 = 2 / x13;
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
	const GEN_FLT x156 = -x151 + x155 - x147 * obj_qw + x149 * obj_qw + x153 * obj_qw;
	const GEN_FLT x157 = x29 * sensor_x;
	const GEN_FLT x158 = x145 * x157;
	const GEN_FLT x159 = x26 * x145;
	const GEN_FLT x160 = x159 * sensor_y;
	const GEN_FLT x161 = x22 * obj_qk;
	const GEN_FLT x162 = x24 * sensor_z;
	const GEN_FLT x163 = x162 * x145;
	const GEN_FLT x164 = x14 * obj_qi;
	const GEN_FLT x165 = x164 * sensor_z;
	const GEN_FLT x166 = x161 - x165 + x158 * obj_qw - x160 * obj_qw + x163 * obj_qw;
	const GEN_FLT x167 = x154 * sensor_x;
	const GEN_FLT x168 = x21 * sensor_x;
	const GEN_FLT x169 = x168 * x145;
	const GEN_FLT x170 = x17 * sensor_y;
	const GEN_FLT x171 = x170 * x145;
	const GEN_FLT x172 = x164 * sensor_y;
	const GEN_FLT x173 = x10 * x145;
	const GEN_FLT x174 = x173 * sensor_z;
	const GEN_FLT x175 = -x167 + x172 + x169 * obj_qw + x171 * obj_qw - x174 * obj_qw;
	const GEN_FLT x176 = x5 * x43;
	const GEN_FLT x177 = x176 * x175 + x48 * x166 + x6 * x156;
	const GEN_FLT x178 = x34 * x166 + x41 * x156 + x7 * x175;
	const GEN_FLT x179 = (x52 * x177 - x53 * x178) * x56;
	const GEN_FLT x180 = x122 * x178 + x97 * x177;
	const GEN_FLT x181 = x175 * x137 + x59 * x166 + x61 * x156;
	const GEN_FLT x182 = x102 * x180 - x82 * x181;
	const GEN_FLT x183 = x179 - x96 * x182;
	const GEN_FLT x184 = x108 * (-x110 * (x180 + x129 * x181) + x68 * x181);
	const GEN_FLT x185 = x71 * x184;
	const GEN_FLT x186 = x69 * (x185 - x70 * x184) + x72 * x184;
	const GEN_FLT x187 = x69 * x186 + x73 * x184;
	const GEN_FLT x188 =
		x95 *
		(x182 + x107 * x183 + x112 * x184 -
		 x119 *
			 (x114 * x183 +
			  x88 * (x69 * x187 +
					 x69 * (x187 + x69 * (x186 + x69 * (x185 - x116 * x184 + x76 * x184) + x77 * x184) + x78 * x184) +
					 x74 * x184 + x79 * x184)) +
		 x93 * x187);
	const GEN_FLT x189 = x154 * sensor_y;
	const GEN_FLT x190 = x150 * sensor_z;
	const GEN_FLT x191 = x189 + x190 - x147 * obj_qi + x149 * obj_qi + x153 * obj_qi;
	const GEN_FLT x192 = 4 * x13;
	const GEN_FLT x193 = -x192 * obj_qi;
	const GEN_FLT x194 = x14 * obj_qw;
	const GEN_FLT x195 = x194 * sensor_z;
	const GEN_FLT x196 = x167 - x195 + x158 * obj_qi + x163 * obj_qi + (x193 - x159 * obj_qi) * sensor_y;
	const GEN_FLT x197 = x194 * sensor_y;
	const GEN_FLT x198 = x161 + x197 + x169 * obj_qi + x171 * obj_qi + (x193 - x173 * obj_qi) * sensor_z;
	const GEN_FLT x199 = x176 * x198 + x48 * x196 + x6 * x191;
	const GEN_FLT x200 = x34 * x196 + x41 * x191 + x7 * x198;
	const GEN_FLT x201 = (x52 * x199 - x53 * x200) * x56;
	const GEN_FLT x202 = x200 * x122 + x97 * x199;
	const GEN_FLT x203 = x198 * x137 + x59 * x196 + x61 * x191;
	const GEN_FLT x204 = x202 * x102 - x82 * x203;
	const GEN_FLT x205 = x201 - x96 * x204;
	const GEN_FLT x206 = x108 * (-x110 * (x202 + x203 * x129) + x68 * x203);
	const GEN_FLT x207 = x71 * x206;
	const GEN_FLT x208 = x69 * (x207 - x70 * x206) + x72 * x206;
	const GEN_FLT x209 = x69 * x208 + x73 * x206;
	const GEN_FLT x210 =
		x95 *
		(x204 -
		 x119 *
			 (x205 * x114 +
			  x88 * (x69 * x209 +
					 x69 * (x209 + x69 * (x208 + x69 * (x207 - x206 * x116 + x76 * x206) + x77 * x206) + x78 * x206) +
					 x74 * x206 + x79 * x206)) +
		 x205 * x107 + x206 * x112 + x93 * x209);
	const GEN_FLT x211 = -x192 * obj_qj;
	const GEN_FLT x212 = x145 * obj_qj;
	const GEN_FLT x213 = x172 + x195 + x212 * x148 + x212 * x152 + (x211 - x146 * obj_qj) * sensor_x;
	const GEN_FLT x214 = x22 * obj_qi;
	const GEN_FLT x215 = x190 + x214 - x160 * obj_qj + x212 * x157 + x212 * x162;
	const GEN_FLT x216 = x22 * obj_qw;
	const GEN_FLT x217 = x151 - x216 + x212 * x168 + x212 * x170 + (x211 - x173 * obj_qj) * sensor_z;
	const GEN_FLT x218 = x217 * x176 + x48 * x215 + x6 * x213;
	const GEN_FLT x219 = x34 * x215 + x41 * x213 + x7 * x217;
	const GEN_FLT x220 = (x52 * x218 - x53 * x219) * x56;
	const GEN_FLT x221 = x219 * x122 + x97 * x218;
	const GEN_FLT x222 = x217 * x137 + x59 * x215 + x61 * x213;
	const GEN_FLT x223 = x221 * x102 - x82 * x222;
	const GEN_FLT x224 = x220 - x96 * x223;
	const GEN_FLT x225 = x108 * (-x110 * (x221 + x222 * x129) + x68 * x222);
	const GEN_FLT x226 = x71 * x225;
	const GEN_FLT x227 = x69 * (x226 - x70 * x225) + x72 * x225;
	const GEN_FLT x228 = x69 * x227 + x73 * x225;
	const GEN_FLT x229 =
		x95 *
		(x223 -
		 x119 *
			 (x224 * x114 +
			  x88 * (x69 * x228 +
					 x69 * (x228 + x69 * (x227 + x69 * (x226 - x225 * x116 + x76 * x225) + x77 * x225) + x78 * x225) +
					 x74 * x225 + x79 * x225)) +
		 x224 * x107 + x225 * x112 + x93 * x228);
	const GEN_FLT x230 = x145 * obj_qk;
	const GEN_FLT x231 = -x192 * obj_qk;
	const GEN_FLT x232 = x165 - x197 + x230 * x148 + x230 * x152 + (x231 - x146 * obj_qk) * sensor_x;
	const GEN_FLT x233 = x155 + x216 + x163 * obj_qk + x230 * x157 + (x231 - x159 * obj_qk) * sensor_y;
	const GEN_FLT x234 = x5 * x233;
	const GEN_FLT x235 = x189 + x214 - x174 * obj_qk + x230 * x168 + x230 * x170;
	const GEN_FLT x236 = x235 * x176 + x47 * x234 + x6 * x232;
	const GEN_FLT x237 = x33 * x234 + x41 * x232 + x7 * x235;
	const GEN_FLT x238 = (x52 * x236 - x53 * x237) * x56;
	const GEN_FLT x239 = x237 * x122 + x97 * x236;
	const GEN_FLT x240 = x235 * x137 + x59 * x233 + x61 * x232;
	const GEN_FLT x241 = x239 * x102 - x82 * x240;
	const GEN_FLT x242 = x238 - x96 * x241;
	const GEN_FLT x243 = x108 * (-x110 * (x239 + x240 * x129) + x68 * x240);
	const GEN_FLT x244 = x71 * x243;
	const GEN_FLT x245 = x69 * (x244 - x70 * x243) + x72 * x243;
	const GEN_FLT x246 = x69 * x245 + x73 * x243;
	const GEN_FLT x247 =
		x95 *
		(x241 -
		 x119 *
			 (x242 * x114 +
			  x88 * (x69 * x246 +
					 x69 * (x246 + x69 * (x245 + x69 * (x244 - x243 * x116 + x76 * x243) + x77 * x243) + x78 * x243) +
					 x74 * x243 + x79 * x243)) +
		 x242 * x107 + x243 * x112 + x93 * x246);
	out[0] = -x120 + x57 - x121 * (x120 - x57);
	out[1] = x124 - x134 - (-x124 + x134) * x121;
	out[2] = x135 - x144 - (-x135 + x144) * x121;
	out[3] = x179 - x188 - (-x179 + x188) * x121;
	out[4] = x201 - x210 - (-x201 + x210) * x121;
	out[5] = x220 - x229 - (-x220 + x229) * x121;
	out[6] = x238 - x247 - (-x238 + x247) * x121;
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qi, 2);
	const GEN_FLT x4 = sqrt(x2 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - x2 * x5;
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = pow(obj_qj, 2);
	const GEN_FLT x9 = pow(obj_qi, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = sqrt(x10 + x7 + pow(obj_qw, 2));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 - (x7 + x8) * x12;
	const GEN_FLT x14 = lh_qw * lh_qk;
	const GEN_FLT x15 = lh_qj * lh_qi;
	const GEN_FLT x16 = -x14 + x15;
	const GEN_FLT x17 = obj_qw * obj_qk;
	const GEN_FLT x18 = obj_qj * obj_qi;
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 4 * x4 * x11;
	const GEN_FLT x21 = x20 * x19;
	const GEN_FLT x22 = obj_qw * obj_qj;
	const GEN_FLT x23 = obj_qk * obj_qi;
	const GEN_FLT x24 = -x22 + x23;
	const GEN_FLT x25 = lh_qw * lh_qj;
	const GEN_FLT x26 = lh_qk * lh_qi;
	const GEN_FLT x27 = x25 + x26;
	const GEN_FLT x28 = x20 * x27;
	const GEN_FLT x29 = x21 * x16 + x24 * x28 + x6 * x13;
	const GEN_FLT x30 = 1 - (x1 + x3) * x5;
	const GEN_FLT x31 = 1 - x12 * x10;
	const GEN_FLT x32 = obj_qw * obj_qi;
	const GEN_FLT x33 = obj_qk * obj_qj;
	const GEN_FLT x34 = x32 + x33;
	const GEN_FLT x35 = x12 * sensor_y;
	const GEN_FLT x36 = x12 * sensor_x;
	const GEN_FLT x37 = obj_pz + x31 * sensor_z + x34 * x35 + x36 * x24;
	const GEN_FLT x38 = lh_qw * lh_qi;
	const GEN_FLT x39 = lh_qk * lh_qj;
	const GEN_FLT x40 = x38 + x39;
	const GEN_FLT x41 = -x32 + x33;
	const GEN_FLT x42 = x12 * sensor_z;
	const GEN_FLT x43 = 1 - (x7 + x9) * x12;
	const GEN_FLT x44 = obj_py + x36 * x19 + x41 * x42 + x43 * sensor_y;
	const GEN_FLT x45 = x5 * x44;
	const GEN_FLT x46 = -x25 + x26;
	const GEN_FLT x47 = x22 + x23;
	const GEN_FLT x48 = -x17 + x18;
	const GEN_FLT x49 = obj_px + x13 * sensor_x + x42 * x47 + x48 * x35;
	const GEN_FLT x50 = x5 * x49;
	const GEN_FLT x51 = lh_pz + x30 * x37 + x40 * x45 + x50 * x46;
	const GEN_FLT x52 = x5 * x37;
	const GEN_FLT x53 = lh_px + x45 * x16 + x52 * x27 + x6 * x49;
	const GEN_FLT x54 = pow(x53, 2);
	const GEN_FLT x55 = x51 / x54;
	const GEN_FLT x56 = pow(x53, -1);
	const GEN_FLT x57 = x5 * x13;
	const GEN_FLT x58 = x30 * x12;
	const GEN_FLT x59 = x40 * x21 + x57 * x46 + x58 * x24;
	const GEN_FLT x60 = x54 + pow(x51, 2);
	const GEN_FLT x61 = pow(x60, -1);
	const GEN_FLT x62 = x61 * x54;
	const GEN_FLT x63 = (x55 * x29 - x56 * x59) * x62;
	const GEN_FLT x64 = -x38 + x39;
	const GEN_FLT x65 = 1 - (x0 + x3) * x5;
	const GEN_FLT x66 = x14 + x15;
	const GEN_FLT x67 = lh_py + x64 * x52 + x65 * x44 + x66 * x50;
	const GEN_FLT x68 = 0.523598775598299 - tilt_1;
	const GEN_FLT x69 = cos(x68);
	const GEN_FLT x70 = pow(x69, -1);
	const GEN_FLT x71 = pow(x67, 2);
	const GEN_FLT x72 = x60 + x71;
	const GEN_FLT x73 = x70 / sqrt(x72);
	const GEN_FLT x74 = asin(x73 * x67);
	const GEN_FLT x75 = 8.0108022e-06 * x74;
	const GEN_FLT x76 = -8.0108022e-06 - x75;
	const GEN_FLT x77 = 0.0028679863 + x74 * x76;
	const GEN_FLT x78 = 5.3685255e-06 + x74 * x77;
	const GEN_FLT x79 = 0.0076069798 + x78 * x74;
	const GEN_FLT x80 = x79 * x74;
	const GEN_FLT x81 = -8.0108022e-06 - 1.60216044e-05 * x74;
	const GEN_FLT x82 = x77 + x81 * x74;
	const GEN_FLT x83 = x78 + x82 * x74;
	const GEN_FLT x84 = x79 + x83 * x74;
	const GEN_FLT x85 = x80 + x84 * x74;
	const GEN_FLT x86 = tan(x68);
	const GEN_FLT x87 = x86 / sqrt(x60);
	const GEN_FLT x88 = -x87 * x67;
	const GEN_FLT x89 = atan2(-x51, x53);
	const GEN_FLT x90 = ogeeMag_1 + x89 - asin(x88);
	const GEN_FLT x91 = curve_1 + sin(x90) * ogeePhase_1;
	const GEN_FLT x92 = sin(x68);
	const GEN_FLT x93 = x92 * x91;
	const GEN_FLT x94 = x69 + x85 * x93;
	const GEN_FLT x95 = pow(x94, -1);
	const GEN_FLT x96 = pow(x74, 2);
	const GEN_FLT x97 = x91 * x96;
	const GEN_FLT x98 = x97 * x95;
	const GEN_FLT x99 = x88 + x79 * x98;
	const GEN_FLT x100 = pow(1 - pow(x99, 2), -1.0 / 2.0);
	const GEN_FLT x101 = 2 * x53;
	const GEN_FLT x102 = 2 * x51;
	const GEN_FLT x103 = x29 * x101 + x59 * x102;
	const GEN_FLT x104 = (1.0 / 2.0) * x67;
	const GEN_FLT x105 = x86 * x104 / pow(x60, 3.0 / 2.0);
	const GEN_FLT x106 = x65 * x12;
	const GEN_FLT x107 = x64 * x20;
	const GEN_FLT x108 = x19 * x106 + x24 * x107 + x66 * x57;
	const GEN_FLT x109 = x103 * x105 - x87 * x108;
	const GEN_FLT x110 = pow(1 - pow(x86, 2) * x71 * x61, -1.0 / 2.0);
	const GEN_FLT x111 = cos(x90) * ogeePhase_1;
	const GEN_FLT x112 = x111 * (x63 - x109 * x110);
	const GEN_FLT x113 = x79 * x96 * x95;
	const GEN_FLT x114 = pow(1 - x71 / (x72 * pow(x69, 2)), -1.0 / 2.0);
	const GEN_FLT x115 = 2 * x67;
	const GEN_FLT x116 = x70 * x104 / pow(x72, 3.0 / 2.0);
	const GEN_FLT x117 = -x116 * (x103 + x108 * x115) + x73 * x108;
	const GEN_FLT x118 = x114 * x117;
	const GEN_FLT x119 = 2 * x80 * x91 * x95;
	const GEN_FLT x120 = x85 * x92;
	const GEN_FLT x121 = x76 * x118;
	const GEN_FLT x122 = 2.40324066e-05 * x74;
	const GEN_FLT x123 = x74 * (x121 - x75 * x118) + x77 * x118;
	const GEN_FLT x124 = x74 * x123 + x78 * x118;
	const GEN_FLT x125 = x79 * x114;
	const GEN_FLT x126 = x84 * x114;
	const GEN_FLT x127 = x79 * x97 / pow(x94, 2);
	const GEN_FLT x128 =
		x100 *
		(x109 + x112 * x113 + x118 * x119 -
		 x127 *
			 (x112 * x120 +
			  x93 * (x117 * x125 + x117 * x126 + x74 * x124 +
					 x74 * (x124 + x74 * (x123 + x74 * (x121 - x118 * x122 + x81 * x118) + x82 * x118) + x83 * x118))) +
		 x98 * x124);
	const GEN_FLT x129 = cos(gibPhase_1 + x89 - asin(x99)) * gibMag_1;
	const GEN_FLT x130 = x5 * x43;
	const GEN_FLT x131 = x6 * x12;
	const GEN_FLT x132 = x16 * x130 + x34 * x28 + x48 * x131;
	const GEN_FLT x133 = x48 * x20;
	const GEN_FLT x134 = x40 * x130 + x46 * x133 + x58 * x34;
	const GEN_FLT x135 = (x55 * x132 - x56 * x134) * x62;
	const GEN_FLT x136 = x101 * x132 + x102 * x134;
	const GEN_FLT x137 = x34 * x107 + x65 * x43 + x66 * x133;
	const GEN_FLT x138 = x105 * x136 - x87 * x137;
	const GEN_FLT x139 = x135 - x110 * x138;
	const GEN_FLT x140 = x111 * x113;
	const GEN_FLT x141 = -x116 * (x136 + x115 * x137) + x73 * x137;
	const GEN_FLT x142 = x114 * x141;
	const GEN_FLT x143 = x111 * x120;
	const GEN_FLT x144 = x76 * x142;
	const GEN_FLT x145 = x74 * (x144 - x75 * x142) + x77 * x142;
	const GEN_FLT x146 = x74 * x145 + x78 * x142;
	const GEN_FLT x147 =
		x100 *
		(x138 + x119 * x142 -
		 x127 *
			 (x139 * x143 +
			  x93 * (x125 * x141 + x126 * x141 + x74 * x146 +
					 x74 * (x146 + x74 * (x145 + x74 * (x144 - x122 * x142 + x81 * x142) + x82 * x142) + x83 * x142))) +
		 x139 * x140 + x98 * x146);
	const GEN_FLT x148 = x41 * x20;
	const GEN_FLT x149 = x5 * x31;
	const GEN_FLT x150 = x16 * x148 + x27 * x149 + x47 * x131;
	const GEN_FLT x151 = x47 * x20;
	const GEN_FLT x152 = x30 * x31 + x40 * x148 + x46 * x151;
	const GEN_FLT x153 = (x55 * x150 - x56 * x152) * x62;
	const GEN_FLT x154 = x101 * x150 + x102 * x152;
	const GEN_FLT x155 = x41 * x106 + x64 * x149 + x66 * x151;
	const GEN_FLT x156 = x105 * x154 - x87 * x155;
	const GEN_FLT x157 = x153 - x110 * x156;
	const GEN_FLT x158 = -x116 * (x154 + x115 * x155) + x73 * x155;
	const GEN_FLT x159 = x114 * x158;
	const GEN_FLT x160 = x76 * x159;
	const GEN_FLT x161 = x74 * (x160 - x75 * x159) + x77 * x159;
	const GEN_FLT x162 = x74 * x161 + x78 * x159;
	const GEN_FLT x163 =
		x100 *
		(x156 + x119 * x159 -
		 x127 *
			 (x143 * x157 +
			  x93 * (x125 * x158 + x126 * x158 + x74 * x162 +
					 x74 * (x162 + x74 * (x161 + x74 * (x160 - x122 * x159 + x81 * x159) + x82 * x159) + x83 * x159))) +
		 x140 * x157 + x98 * x162);
	out[0] = -x128 + x63 - x129 * (x128 - x63);
	out[1] = x135 - x147 - (-x135 + x147) * x129;
	out[2] = x153 - x163 - (-x153 + x163) * x129;
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
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qk, 2);
	const GEN_FLT x4 = x1 + x3;
	const GEN_FLT x5 = sqrt(x0 + x4 + pow(lh_qw, 2));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = x7 + x8;
	const GEN_FLT x10 = pow(obj_qk, 2);
	const GEN_FLT x11 = 2 * sqrt(x10 + x9 + pow(obj_qw, 2));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 = obj_pz + (1 - x9 * x11) * sensor_z + (x12 + x13) * x14 + (-x15 + x16) * x17;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + (1 - x11 * (x10 + x8)) * sensor_y + (-x12 + x13) * x22 + (x23 + x24) * x17;
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = -x27 + x28;
	const GEN_FLT x30 = obj_px + (1 - x11 * (x10 + x7)) * sensor_x + (x15 + x16) * x22 + (-x23 + x24) * x14;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + x18 * (1 - x2 * x6) + x21 * x26 + x31 * x29;
	const GEN_FLT x33 = x27 + x28;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = -x35 + x36;
	const GEN_FLT x38 = x0 + x3;
	const GEN_FLT x39 = lh_px + x30 * (1 - x6 * x38) + x34 * x33 + x37 * x26;
	const GEN_FLT x40 = pow(x39, 2);
	const GEN_FLT x41 = x40 + pow(x32, 2);
	const GEN_FLT x42 = pow(x41, -1);
	const GEN_FLT x43 = x42 * x32;
	const GEN_FLT x44 = -x19 + x20;
	const GEN_FLT x45 = x35 + x36;
	const GEN_FLT x46 = lh_py + x25 * (1 - x4 * x6) + x44 * x34 + x45 * x31;
	const GEN_FLT x47 = 0.523598775598299 - tilt_1;
	const GEN_FLT x48 = cos(x47);
	const GEN_FLT x49 = pow(x48, -1);
	const GEN_FLT x50 = pow(x46, 2);
	const GEN_FLT x51 = x41 + x50;
	const GEN_FLT x52 = x49 / sqrt(x51);
	const GEN_FLT x53 = asin(x52 * x46);
	const GEN_FLT x54 = 8.0108022e-06 * x53;
	const GEN_FLT x55 = -8.0108022e-06 - x54;
	const GEN_FLT x56 = 0.0028679863 + x53 * x55;
	const GEN_FLT x57 = 5.3685255e-06 + x53 * x56;
	const GEN_FLT x58 = 0.0076069798 + x53 * x57;
	const GEN_FLT x59 = pow(x53, 2);
	const GEN_FLT x60 = tan(x47);
	const GEN_FLT x61 = x60 / sqrt(x41);
	const GEN_FLT x62 = -x61 * x46;
	const GEN_FLT x63 = atan2(-x32, x39);
	const GEN_FLT x64 = ogeeMag_1 + x63 - asin(x62);
	const GEN_FLT x65 = curve_1 + sin(x64) * ogeePhase_1;
	const GEN_FLT x66 = x53 * x58;
	const GEN_FLT x67 = -8.0108022e-06 - 1.60216044e-05 * x53;
	const GEN_FLT x68 = x56 + x67 * x53;
	const GEN_FLT x69 = x57 + x68 * x53;
	const GEN_FLT x70 = x58 + x69 * x53;
	const GEN_FLT x71 = x66 + x70 * x53;
	const GEN_FLT x72 = sin(x47);
	const GEN_FLT x73 = x72 * x65;
	const GEN_FLT x74 = x48 + x71 * x73;
	const GEN_FLT x75 = pow(x74, -1);
	const GEN_FLT x76 = x75 * x65;
	const GEN_FLT x77 = x76 * x59;
	const GEN_FLT x78 = x62 + x77 * x58;
	const GEN_FLT x79 = pow(1 - pow(x78, 2), -1.0 / 2.0);
	const GEN_FLT x80 = x46 * x39;
	const GEN_FLT x81 = x60 / pow(x41, 3.0 / 2.0);
	const GEN_FLT x82 = x80 * x81;
	const GEN_FLT x83 = pow(1 - pow(x60, 2) * x50 * x42, -1.0 / 2.0);
	const GEN_FLT x84 = cos(x64) * ogeePhase_1;
	const GEN_FLT x85 = x84 * (x43 - x82 * x83);
	const GEN_FLT x86 = x58 * x59;
	const GEN_FLT x87 = x86 * x75;
	const GEN_FLT x88 = x49 / pow(x51, 3.0 / 2.0);
	const GEN_FLT x89 = 2 * x46;
	const GEN_FLT x90 = pow(1 - x50 / (x51 * pow(x48, 2)), -1.0 / 2.0);
	const GEN_FLT x91 = x76 * x66;
	const GEN_FLT x92 = x91 * x90;
	const GEN_FLT x93 = x71 * x72;
	const GEN_FLT x94 = x55 * x90;
	const GEN_FLT x95 = x80 * x88;
	const GEN_FLT x96 = -x95 * x94;
	const GEN_FLT x97 = 2.40324066e-05 * x53;
	const GEN_FLT x98 = x90 * x95;
	const GEN_FLT x99 = x53 * (x96 + x54 * x98) - x56 * x98;
	const GEN_FLT x100 = x53 * x99 - x57 * x98;
	const GEN_FLT x101 = x86 * x65 / pow(x74, 2);
	const GEN_FLT x102 =
		x79 * (x82 -
			   x101 * (x73 * (x53 * x100 +
							  x53 * (x100 + x53 * (x99 + x53 * (x96 - x67 * x98 + x98 * x97) - x68 * x98) - x69 * x98) -
							  x58 * x98 - x70 * x98) +
					   x85 * x93) +
			   x77 * x100 + x85 * x87 - x88 * x89 * x92 * x39);
	const GEN_FLT x103 = cos(gibPhase_1 + x63 - asin(x78)) * gibMag_1;
	const GEN_FLT x104 = x84 * x87;
	const GEN_FLT x105 = x83 * x61;
	const GEN_FLT x106 = x90 * (x52 - x88 * x50);
	const GEN_FLT x107 = 2 * x91;
	const GEN_FLT x108 = x84 * x93;
	const GEN_FLT x109 = x55 * x106;
	const GEN_FLT x110 = x53 * (x109 - x54 * x106) + x56 * x106;
	const GEN_FLT x111 = x53 * x110 + x57 * x106;
	const GEN_FLT x112 =
		x79 *
		(-x61 -
		 x101 * (x108 * x105 +
				 x73 * (x53 * x111 +
						x53 * (x111 + x53 * (x110 + x53 * (x109 + x67 * x106 - x97 * x106) + x68 * x106) + x69 * x106) +
						x58 * x106 + x70 * x106)) +
		 x105 * x104 + x107 * x106 + x77 * x111);
	const GEN_FLT x113 = x42 * x39;
	const GEN_FLT x114 = -x113;
	const GEN_FLT x115 = x81 * x46;
	const GEN_FLT x116 = x32 * x115;
	const GEN_FLT x117 = x114 - x83 * x116;
	const GEN_FLT x118 = x88 * x46;
	const GEN_FLT x119 = x32 * x118;
	const GEN_FLT x120 = -x94 * x119;
	const GEN_FLT x121 = x90 * x119;
	const GEN_FLT x122 = x53 * (x120 + x54 * x121) - x56 * x121;
	const GEN_FLT x123 = x53 * x122 - x57 * x121;
	const GEN_FLT x124 = 2 * x32;
	const GEN_FLT x125 =
		x79 *
		(x116 -
		 x101 * (x108 * x117 +
				 x73 * (x53 * x123 +
						x53 * (x123 + x53 * (x122 + x53 * (x120 - x67 * x121 + x97 * x121) - x68 * x121) - x69 * x121) -
						x58 * x121 - x70 * x121)) +
		 x104 * x117 + x77 * x123 - x92 * x118 * x124);
	const GEN_FLT x126 = x30 * x38;
	const GEN_FLT x127 = 2 / x5;
	const GEN_FLT x128 = x127 * lh_qw;
	const GEN_FLT x129 = x37 * x25;
	const GEN_FLT x130 = x26 * lh_qk;
	const GEN_FLT x131 = x33 * x18;
	const GEN_FLT x132 = x34 * lh_qj;
	const GEN_FLT x133 = -x130 + x132 - x128 * x126 + x128 * x129 + x128 * x131;
	const GEN_FLT x134 = x32 / x40;
	const GEN_FLT x135 = pow(x39, -1);
	const GEN_FLT x136 = x30 * x29;
	const GEN_FLT x137 = x25 * x21;
	const GEN_FLT x138 = x26 * lh_qi;
	const GEN_FLT x139 = x31 * lh_qj;
	const GEN_FLT x140 = x2 * x127;
	const GEN_FLT x141 = x138 - x139 + x128 * x136 + x128 * x137 - x18 * x140 * lh_qw;
	const GEN_FLT x142 = x40 * x42;
	const GEN_FLT x143 = (x133 * x134 - x135 * x141) * x142;
	const GEN_FLT x144 = 2 * x39;
	const GEN_FLT x145 = x124 * x141 + x133 * x144;
	const GEN_FLT x146 = (1.0 / 2.0) * x115;
	const GEN_FLT x147 = x45 * x30;
	const GEN_FLT x148 = x31 * lh_qk;
	const GEN_FLT x149 = x4 * x25;
	const GEN_FLT x150 = x44 * x18;
	const GEN_FLT x151 = x34 * lh_qi;
	const GEN_FLT x152 = x148 - x151 + x128 * x147 - x128 * x149 + x128 * x150;
	const GEN_FLT x153 = x146 * x145 - x61 * x152;
	const GEN_FLT x154 = x143 - x83 * x153;
	const GEN_FLT x155 = (1.0 / 2.0) * x118;
	const GEN_FLT x156 = x90 * (-x155 * (x145 + x89 * x152) + x52 * x152);
	const GEN_FLT x157 = x55 * x156;
	const GEN_FLT x158 = x53 * (x157 - x54 * x156) + x56 * x156;
	const GEN_FLT x159 = x53 * x158 + x57 * x156;
	const GEN_FLT x160 =
		x79 *
		(x153 -
		 x101 * (x108 * x154 +
				 x73 * (x53 * x159 +
						x53 * (x159 + x53 * (x158 + x53 * (x157 + x67 * x156 - x97 * x156) + x68 * x156) + x69 * x156) +
						x58 * x156 + x70 * x156)) +
		 x104 * x154 + x107 * x156 + x77 * x159);
	const GEN_FLT x161 = x127 * lh_qi;
	const GEN_FLT x162 = x26 * lh_qj;
	const GEN_FLT x163 = x34 * lh_qk;
	const GEN_FLT x164 = x162 + x163 - x126 * x161 + x129 * x161 + x161 * x131;
	const GEN_FLT x165 = x26 * lh_qw;
	const GEN_FLT x166 = 4 * x5;
	const GEN_FLT x167 = -x166 * lh_qi;
	const GEN_FLT x168 = x148 + x165 + x161 * x136 + x161 * x137 + x18 * (x167 - x140 * lh_qi);
	const GEN_FLT x169 = (x164 * x134 - x168 * x135) * x142;
	const GEN_FLT x170 = x124 * x168 + x164 * x144;
	const GEN_FLT x171 = x34 * lh_qw;
	const GEN_FLT x172 = x139 - x171 + x161 * x147 + x161 * x150 + x25 * (x167 - x4 * x161);
	const GEN_FLT x173 = x170 * x146 - x61 * x172;
	const GEN_FLT x174 = x169 - x83 * x173;
	const GEN_FLT x175 = x90 * (-x155 * (x170 + x89 * x172) + x52 * x172);
	const GEN_FLT x176 = x55 * x175;
	const GEN_FLT x177 = x53 * (x176 - x54 * x175) + x56 * x175;
	const GEN_FLT x178 = x53 * x177 + x57 * x175;
	const GEN_FLT x179 =
		x79 *
		(x173 -
		 x101 * (x108 * x174 +
				 x73 * (x53 * x178 +
						x53 * (x178 + x53 * (x177 + x53 * (x176 + x67 * x175 - x97 * x175) + x68 * x175) + x69 * x175) +
						x58 * x175 + x70 * x175)) +
		 x104 * x174 + x107 * x175 + x77 * x178);
	const GEN_FLT x180 = x127 * lh_qj;
	const GEN_FLT x181 = -x166 * lh_qj;
	const GEN_FLT x182 = x138 + x171 + x129 * x180 + x180 * x131 + x30 * (x181 - x38 * x180);
	const GEN_FLT x183 = x31 * lh_qw;
	const GEN_FLT x184 = x130 - x183 + x18 * (x181 - x140 * lh_qj) + x180 * x136 + x180 * x137;
	const GEN_FLT x185 = (x182 * x134 - x184 * x135) * x142;
	const GEN_FLT x186 = x124 * x184 + x182 * x144;
	const GEN_FLT x187 = x31 * lh_qi;
	const GEN_FLT x188 = x163 + x187 + x180 * x147 - x180 * x149 + x180 * x150;
	const GEN_FLT x189 = x186 * x146 - x61 * x188;
	const GEN_FLT x190 = x185 - x83 * x189;
	const GEN_FLT x191 = x90 * (-x155 * (x186 + x89 * x188) + x52 * x188);
	const GEN_FLT x192 = x55 * x191;
	const GEN_FLT x193 = x53 * (x192 - x54 * x191) + x56 * x191;
	const GEN_FLT x194 = x53 * x193 + x57 * x191;
	const GEN_FLT x195 =
		x79 *
		(x189 -
		 x101 * (x108 * x190 +
				 x73 * (x53 * x194 +
						x53 * (x194 + x53 * (x193 + x53 * (x192 + x67 * x191 - x97 * x191) + x68 * x191) + x69 * x191) +
						x58 * x191 + x70 * x191)) +
		 x104 * x190 + x107 * x191 + x77 * x194);
	const GEN_FLT x196 = x127 * lh_qk;
	const GEN_FLT x197 = -x166 * lh_qk;
	const GEN_FLT x198 = x151 - x165 + x129 * x196 + x196 * x131 + x30 * (x197 - x38 * x196);
	const GEN_FLT x199 = x162 + x187 + x196 * x136 + x196 * x137 - x2 * x18 * x196;
	const GEN_FLT x200 = (x198 * x134 - x199 * x135) * x142;
	const GEN_FLT x201 = x124 * x199 + x198 * x144;
	const GEN_FLT x202 = x132 + x183 + x196 * x147 + x196 * x150 + x25 * (x197 - x4 * x196);
	const GEN_FLT x203 = x201 * x146 - x61 * x202;
	const GEN_FLT x204 = x200 - x83 * x203;
	const GEN_FLT x205 = x90 * (-x155 * (x201 + x89 * x202) + x52 * x202);
	const GEN_FLT x206 = x55 * x205;
	const GEN_FLT x207 = x53 * (x206 - x54 * x205) + x56 * x205;
	const GEN_FLT x208 = x53 * x207 + x57 * x205;
	const GEN_FLT x209 =
		x79 *
		(x203 -
		 x101 * (x204 * x108 +
				 x73 * (x53 * x208 +
						x53 * (x208 + x53 * (x207 + x53 * (x206 + x67 * x205 - x97 * x205) + x68 * x205) + x69 * x205) +
						x58 * x205 + x70 * x205)) +
		 x204 * x104 + x205 * x107 + x77 * x208);
	out[0] = -x102 + x43 - x103 * (x102 - x43);
	out[1] = -x112 - x103 * x112;
	out[2] = x114 - x125 - (x113 + x125) * x103;
	out[3] = x143 - x160 - (-x143 + x160) * x103;
	out[4] = x169 - x179 - (-x169 + x179) * x103;
	out[5] = x185 - x195 - (-x185 + x195) * x103;
	out[6] = x200 - x209 - (-x200 + x209) * x103;
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
	const GEN_FLT x0 = 0.523598775598299 - tilt_1;
	const GEN_FLT x1 = tan(x0);
	const GEN_FLT x2 = lh_qw * lh_qi;
	const GEN_FLT x3 = lh_qk * lh_qj;
	const GEN_FLT x4 = pow(obj_qj, 2);
	const GEN_FLT x5 = pow(obj_qi, 2);
	const GEN_FLT x6 = x4 + x5;
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = 2 * sqrt(x6 + x7 + pow(obj_qw, 2));
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	const GEN_FLT x11 = x8 * sensor_y;
	const GEN_FLT x12 = obj_qw * obj_qj;
	const GEN_FLT x13 = obj_qk * obj_qi;
	const GEN_FLT x14 = x8 * sensor_x;
	const GEN_FLT x15 = obj_pz + x11 * (x10 + x9) + (1 - x6 * x8) * sensor_z + (-x12 + x13) * x14;
	const GEN_FLT x16 = pow(lh_qi, 2);
	const GEN_FLT x17 = pow(lh_qk, 2);
	const GEN_FLT x18 = pow(lh_qj, 2);
	const GEN_FLT x19 = x17 + x18;
	const GEN_FLT x20 = 2 * sqrt(x16 + x19 + pow(lh_qw, 2));
	const GEN_FLT x21 = x20 * x15;
	const GEN_FLT x22 = x8 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + x22 * (x10 - x9) + (1 - (x5 + x7) * x8) * sensor_y + (x23 + x24) * x14;
	const GEN_FLT x26 = lh_qw * lh_qk;
	const GEN_FLT x27 = lh_qj * lh_qi;
	const GEN_FLT x28 = obj_px + (1 - (x4 + x7) * x8) * sensor_x + (x12 + x13) * x22 + (-x23 + x24) * x11;
	const GEN_FLT x29 = x20 * x28;
	const GEN_FLT x30 = lh_py + x25 * (1 - (x16 + x17) * x20) + (-x2 + x3) * x21 + (x26 + x27) * x29;
	const GEN_FLT x31 = x25 * x20;
	const GEN_FLT x32 = lh_qw * lh_qj;
	const GEN_FLT x33 = lh_qk * lh_qi;
	const GEN_FLT x34 = lh_pz + x15 * (1 - (x16 + x18) * x20) + (x2 + x3) * x31 + (-x32 + x33) * x29;
	const GEN_FLT x35 = lh_px + x28 * (1 - x20 * x19) + (-x26 + x27) * x31 + (x32 + x33) * x21;
	const GEN_FLT x36 = pow(x34, 2) + pow(x35, 2);
	const GEN_FLT x37 = x30 / sqrt(x36);
	const GEN_FLT x38 = -x1 * x37;
	const GEN_FLT x39 = atan2(-x34, x35);
	const GEN_FLT x40 = ogeeMag_1 + x39 - asin(x38);
	const GEN_FLT x41 = sin(x40);
	const GEN_FLT x42 = curve_1 + x41 * ogeePhase_1;
	const GEN_FLT x43 = cos(x0);
	const GEN_FLT x44 = pow(x30, 2);
	const GEN_FLT x45 = x36 + x44;
	const GEN_FLT x46 = x30 / sqrt(x45);
	const GEN_FLT x47 = asin(x46 / x43);
	const GEN_FLT x48 = 8.0108022e-06 * x47;
	const GEN_FLT x49 = -8.0108022e-06 - x48;
	const GEN_FLT x50 = 0.0028679863 + x47 * x49;
	const GEN_FLT x51 = 5.3685255e-06 + x50 * x47;
	const GEN_FLT x52 = 0.0076069798 + x51 * x47;
	const GEN_FLT x53 = pow(x47, 2);
	const GEN_FLT x54 = x52 * x47;
	const GEN_FLT x55 = -8.0108022e-06 - 1.60216044e-05 * x47;
	const GEN_FLT x56 = x50 + x55 * x47;
	const GEN_FLT x57 = x51 + x56 * x47;
	const GEN_FLT x58 = x52 + x57 * x47;
	const GEN_FLT x59 = x54 + x58 * x47;
	const GEN_FLT x60 = sin(x0);
	const GEN_FLT x61 = x60 * x42;
	const GEN_FLT x62 = x61 * x59;
	const GEN_FLT x63 = x43 + x62;
	const GEN_FLT x64 = pow(x63, -1);
	const GEN_FLT x65 = x64 * x53;
	const GEN_FLT x66 = x65 * x52;
	const GEN_FLT x67 = x38 + x66 * x42;
	const GEN_FLT x68 = pow(1 - pow(x67, 2), -1.0 / 2.0);
	const GEN_FLT x69 = pow(x1, 2);
	const GEN_FLT x70 = x37 * (1 + x69);
	const GEN_FLT x71 = cos(x40) * ogeePhase_1;
	const GEN_FLT x72 = x71 * x66;
	const GEN_FLT x73 = x70 / sqrt(1 - x69 * x44 / x36);
	const GEN_FLT x74 = pow(x43, -2);
	const GEN_FLT x75 = x74 * x46 / sqrt(1 - x74 * x44 / x45);
	const GEN_FLT x76 = x75 * x60;
	const GEN_FLT x77 = -x76 * x49;
	const GEN_FLT x78 = x47 * (x77 + x76 * x48) - x76 * x50;
	const GEN_FLT x79 = -x76 * x51 + x78 * x47;
	const GEN_FLT x80 = x53 * x52 / pow(x63, 2);
	const GEN_FLT x81 =
		x68 * (x70 - x73 * x72 + x79 * x65 * x42 -
			   x80 * x42 *
				   (x60 +
					x61 * (x47 * (x79 + x47 * (x78 + x47 * (x77 + 2.40324066e-05 * x76 * x47 - x76 * x55) - x76 * x56) -
								  x76 * x57) -
						   x76 * x52 - x76 * x58 + x79 * x47) -
					x59 * x42 * x43 - x71 * x73 * x60 * x59) -
			   2 * x75 * x64 * x61 * x54);
	const GEN_FLT x82 = -gibPhase_1 - x39 + asin(x67);
	const GEN_FLT x83 = cos(x82) * gibMag_1;
	const GEN_FLT x84 = x80 * x62;
	const GEN_FLT x85 = (x66 - x84) * x68;
	const GEN_FLT x86 = x68 * (x72 - x84 * x71);
	const GEN_FLT x87 = (x66 * x41 - x84 * x41) * x68;
	out[0] = -1;
	out[1] = -x81 - x81 * x83;
	out[2] = -x85 - x83 * x85;
	out[3] = x83;
	out[4] = -sin(x82);
	out[5] = -x86 - x83 * x86;
	out[6] = -x87 - x83 * x87;
}

/** Applying function <function reproject at 0x7effc26e3320> */
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
	const GEN_FLT x2 = pow(obj_qj, 2);
	const GEN_FLT x3 = pow(obj_qi, 2);
	const GEN_FLT x4 = x2 + x3;
	const GEN_FLT x5 = pow(obj_qk, 2);
	const GEN_FLT x6 = 2 * sqrt(x4 + x5 + pow(obj_qw, 2));
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x6 * sensor_y;
	const GEN_FLT x10 = obj_qw * obj_qj;
	const GEN_FLT x11 = obj_qk * obj_qi;
	const GEN_FLT x12 = x6 * sensor_x;
	const GEN_FLT x13 = obj_pz + (1 - x4 * x6) * sensor_z + (-x10 + x11) * x12 + (x7 + x8) * x9;
	const GEN_FLT x14 = pow(lh_qi, 2);
	const GEN_FLT x15 = pow(lh_qk, 2);
	const GEN_FLT x16 = pow(lh_qj, 2);
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = 2 * sqrt(x14 + x17 + pow(lh_qw, 2));
	const GEN_FLT x19 = x13 * x18;
	const GEN_FLT x20 = x6 * sensor_z;
	const GEN_FLT x21 = obj_qw * obj_qk;
	const GEN_FLT x22 = obj_qj * obj_qi;
	const GEN_FLT x23 = obj_py + (1 - (x3 + x5) * x6) * sensor_y + (x21 + x22) * x12 + (-x7 + x8) * x20;
	const GEN_FLT x24 = lh_qw * lh_qk;
	const GEN_FLT x25 = lh_qj * lh_qi;
	const GEN_FLT x26 = obj_px + (1 - (x2 + x5) * x6) * sensor_x + (x10 + x11) * x20 + (-x21 + x22) * x9;
	const GEN_FLT x27 = x26 * x18;
	const GEN_FLT x28 = lh_py + x23 * (1 - (x14 + x15) * x18) + (-x0 + x1) * x19 + (x24 + x25) * x27;
	const GEN_FLT x29 = x23 * x18;
	const GEN_FLT x30 = lh_qw * lh_qj;
	const GEN_FLT x31 = lh_qk * lh_qi;
	const GEN_FLT x32 = lh_pz + x13 * (1 - (x14 + x16) * x18) + (x0 + x1) * x29 + (-x30 + x31) * x27;
	const GEN_FLT x33 = -x32;
	const GEN_FLT x34 = pow(x32, 2);
	const GEN_FLT x35 = lh_px + x26 * (1 - x18 * x17) + (-x24 + x25) * x29 + (x30 + x31) * x19;
	const GEN_FLT x36 = atan2(x35, x33);
	const GEN_FLT x37 = -phase_0 - x36 - asin(x28 * tilt_0 / sqrt(x34 + pow(x35, 2)));
	const GEN_FLT x38 = -phase_1 - asin(x35 * tilt_1 / sqrt(x34 + pow(x28, 2))) - atan2(-x28, x33);
	out[0] = x37 - cos(1.5707963267949 + gibPhase_0 + x37) * gibMag_0 + pow(atan2(x28, x33), 2) * curve_0;
	out[1] = x38 + pow(x36, 2) * curve_1 - cos(1.5707963267949 + gibPhase_1 + x38) * gibMag_1;
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qi, 2);
	const GEN_FLT x4 = sqrt(x2 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - x2 * x5;
	const GEN_FLT x7 = 1 - (x1 + x3) * x5;
	const GEN_FLT x8 = pow(obj_qj, 2);
	const GEN_FLT x9 = pow(obj_qi, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = sqrt(x10 + x11 + pow(obj_qw, 2));
	const GEN_FLT x13 = 2 * x12;
	const GEN_FLT x14 = obj_qw * obj_qi;
	const GEN_FLT x15 = obj_qk * obj_qj;
	const GEN_FLT x16 = x14 + x15;
	const GEN_FLT x17 = x13 * sensor_y;
	const GEN_FLT x18 = obj_qw * obj_qj;
	const GEN_FLT x19 = obj_qk * obj_qi;
	const GEN_FLT x20 = -x18 + x19;
	const GEN_FLT x21 = x13 * sensor_x;
	const GEN_FLT x22 = obj_pz + x17 * x16 + x20 * x21 + (1 - x13 * x10) * sensor_z;
	const GEN_FLT x23 = -x14 + x15;
	const GEN_FLT x24 = x13 * sensor_z;
	const GEN_FLT x25 = x11 + x9;
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x26 + x27;
	const GEN_FLT x29 = obj_py + x21 * x28 + x24 * x23 + (1 - x25 * x13) * sensor_y;
	const GEN_FLT x30 = lh_qw * lh_qi;
	const GEN_FLT x31 = lh_qk * lh_qj;
	const GEN_FLT x32 = x30 + x31;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = x18 + x19;
	const GEN_FLT x35 = -x26 + x27;
	const GEN_FLT x36 = x11 + x8;
	const GEN_FLT x37 = obj_px + x34 * x24 + x35 * x17 + (1 - x36 * x13) * sensor_x;
	const GEN_FLT x38 = lh_qw * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qi;
	const GEN_FLT x40 = -x38 + x39;
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = lh_pz + x33 * x29 + x41 * x37 + x7 * x22;
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = pow(x42, 2);
	const GEN_FLT x45 = pow(x44, -1);
	const GEN_FLT x46 = x38 + x39;
	const GEN_FLT x47 = x5 * x46;
	const GEN_FLT x48 = lh_qw * lh_qk;
	const GEN_FLT x49 = lh_qj * lh_qi;
	const GEN_FLT x50 = -x48 + x49;
	const GEN_FLT x51 = x5 * x50;
	const GEN_FLT x52 = lh_px + x47 * x22 + x51 * x29 + x6 * x37;
	const GEN_FLT x53 = x52 * x45;
	const GEN_FLT x54 = pow(x52, 2);
	const GEN_FLT x55 = x44 + x54;
	const GEN_FLT x56 = pow(x55, -1);
	const GEN_FLT x57 = x56 * x44;
	const GEN_FLT x58 = x57 * (x53 * x41 - x6 * x43);
	const GEN_FLT x59 = -x30 + x31;
	const GEN_FLT x60 = x5 * x59;
	const GEN_FLT x61 = 1 - (x0 + x3) * x5;
	const GEN_FLT x62 = x48 + x49;
	const GEN_FLT x63 = x5 * x62;
	const GEN_FLT x64 = lh_py + x60 * x22 + x61 * x29 + x63 * x37;
	const GEN_FLT x65 = pow(x64, 2);
	const GEN_FLT x66 = pow(1 - x65 * x56 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x67 = 2 * x52;
	const GEN_FLT x68 = 4 * x4;
	const GEN_FLT x69 = x68 * x42;
	const GEN_FLT x70 = x69 * x40;
	const GEN_FLT x71 = (1.0 / 2.0) * x64 * tilt_0 / pow(x55, 3.0 / 2.0);
	const GEN_FLT x72 = tilt_0 / sqrt(x55);
	const GEN_FLT x73 = -x58 - x66 * (-x71 * (x70 + x6 * x67) + x72 * x63);
	const GEN_FLT x74 = -x42;
	const GEN_FLT x75 = atan2(x52, x74);
	const GEN_FLT x76 = sin(1.5707963267949 + gibPhase_0 - phase_0 - x75 - asin(x72 * x64)) * gibMag_0;
	const GEN_FLT x77 = x63 * x43;
	const GEN_FLT x78 = x64 * x45;
	const GEN_FLT x79 = x78 * x41;
	const GEN_FLT x80 = x44 + x65;
	const GEN_FLT x81 = pow(x80, -1);
	const GEN_FLT x82 = x81 * x44;
	const GEN_FLT x83 = 2 * x82 * atan2(x64, x74) * curve_0;
	const GEN_FLT x84 = (-x51 * x43 + x53 * x33) * x57;
	const GEN_FLT x85 = x68 * x52;
	const GEN_FLT x86 = x69 * x32;
	const GEN_FLT x87 = -x84 - x66 * (-x71 * (x86 + x85 * x50) + x72 * x61);
	const GEN_FLT x88 = x61 * x43;
	const GEN_FLT x89 = x78 * x33;
	const GEN_FLT x90 = x57 * (-x43 * x47 + x7 * x53);
	const GEN_FLT x91 = 2 * x42;
	const GEN_FLT x92 = x7 * x91;
	const GEN_FLT x93 = -x90 - x66 * (-x71 * (x92 + x85 * x46) + x72 * x60);
	const GEN_FLT x94 = x60 * x43;
	const GEN_FLT x95 = x7 * x78;
	const GEN_FLT x96 = 2 / x12;
	const GEN_FLT x97 = x96 * x36;
	const GEN_FLT x98 = x97 * sensor_x;
	const GEN_FLT x99 = x96 * obj_qw;
	const GEN_FLT x100 = x35 * sensor_y;
	const GEN_FLT x101 = x17 * obj_qk;
	const GEN_FLT x102 = x34 * sensor_z;
	const GEN_FLT x103 = x24 * obj_qj;
	const GEN_FLT x104 = -x101 + x103 - x98 * obj_qw + x99 * x100 + x99 * x102;
	const GEN_FLT x105 = x99 * sensor_x;
	const GEN_FLT x106 = x96 * x25;
	const GEN_FLT x107 = x106 * sensor_y;
	const GEN_FLT x108 = x21 * obj_qk;
	const GEN_FLT x109 = x23 * sensor_z;
	const GEN_FLT x110 = x24 * obj_qi;
	const GEN_FLT x111 = x108 - x110 - x107 * obj_qw + x28 * x105 + x99 * x109;
	const GEN_FLT x112 = x21 * obj_qj;
	const GEN_FLT x113 = x16 * sensor_y;
	const GEN_FLT x114 = x17 * obj_qi;
	const GEN_FLT x115 = x96 * x10;
	const GEN_FLT x116 = x115 * sensor_z;
	const GEN_FLT x117 = -x112 + x114 - x116 * obj_qw + x20 * x105 + x99 * x113;
	const GEN_FLT x118 = x47 * x117 + x51 * x111 + x6 * x104;
	const GEN_FLT x119 = x33 * x111 + x41 * x104 + x7 * x117;
	const GEN_FLT x120 = (-x43 * x118 + x53 * x119) * x57;
	const GEN_FLT x121 = x91 * x119;
	const GEN_FLT x122 = x60 * x117 + x61 * x111 + x63 * x104;
	const GEN_FLT x123 = -x120 - x66 * (-x71 * (x121 + x67 * x118) + x72 * x122);
	const GEN_FLT x124 = x43 * x122;
	const GEN_FLT x125 = x78 * x119;
	const GEN_FLT x126 = x96 * obj_qi;
	const GEN_FLT x127 = x17 * obj_qj;
	const GEN_FLT x128 = x24 * obj_qk;
	const GEN_FLT x129 = x127 + x128 + x100 * x126 + x102 * x126 - x98 * obj_qi;
	const GEN_FLT x130 = x28 * sensor_x;
	const GEN_FLT x131 = 4 * x12;
	const GEN_FLT x132 = -x131 * obj_qi;
	const GEN_FLT x133 = x24 * obj_qw;
	const GEN_FLT x134 = x112 - x133 + x109 * x126 + x126 * x130 + (x132 - x106 * obj_qi) * sensor_y;
	const GEN_FLT x135 = x5 * x134;
	const GEN_FLT x136 = x20 * sensor_x;
	const GEN_FLT x137 = x96 * x113;
	const GEN_FLT x138 = x17 * obj_qw;
	const GEN_FLT x139 = x108 + x138 + x126 * x136 + x137 * obj_qi + (x132 - x115 * obj_qi) * sensor_z;
	const GEN_FLT x140 = x47 * x139 + x50 * x135 + x6 * x129;
	const GEN_FLT x141 = x32 * x135 + x41 * x129 + x7 * x139;
	const GEN_FLT x142 = (-x43 * x140 + x53 * x141) * x57;
	const GEN_FLT x143 = x91 * x141;
	const GEN_FLT x144 = x60 * x139 + x61 * x134 + x63 * x129;
	const GEN_FLT x145 = -x142 - x66 * (-x71 * (x143 + x67 * x140) + x72 * x144);
	const GEN_FLT x146 = x43 * x144;
	const GEN_FLT x147 = x78 * x141;
	const GEN_FLT x148 = -x131 * obj_qj;
	const GEN_FLT x149 = x96 * x100;
	const GEN_FLT x150 = x96 * obj_qj;
	const GEN_FLT x151 = x114 + x133 + x102 * x150 + x149 * obj_qj + (x148 - x97 * obj_qj) * sensor_x;
	const GEN_FLT x152 = x21 * obj_qi;
	const GEN_FLT x153 = x96 * x109;
	const GEN_FLT x154 = x128 + x152 - x107 * obj_qj + x130 * x150 + x153 * obj_qj;
	const GEN_FLT x155 = x21 * obj_qw;
	const GEN_FLT x156 = x101 - x155 + x136 * x150 + x137 * obj_qj + (x148 - x115 * obj_qj) * sensor_z;
	const GEN_FLT x157 = x47 * x156 + x51 * x154 + x6 * x151;
	const GEN_FLT x158 = x33 * x154 + x41 * x151 + x7 * x156;
	const GEN_FLT x159 = (-x43 * x157 + x53 * x158) * x57;
	const GEN_FLT x160 = x91 * x158;
	const GEN_FLT x161 = x60 * x156 + x61 * x154 + x63 * x151;
	const GEN_FLT x162 = -x159 - x66 * (-x71 * (x160 + x67 * x157) + x72 * x161);
	const GEN_FLT x163 = x43 * x161;
	const GEN_FLT x164 = x78 * x158;
	const GEN_FLT x165 = -x131 * obj_qk;
	const GEN_FLT x166 = x96 * obj_qk;
	const GEN_FLT x167 = x110 - x138 + x102 * x166 + x149 * obj_qk + (x165 - x97 * obj_qk) * sensor_x;
	const GEN_FLT x168 = x103 + x155 + x153 * obj_qk + x166 * x130 + (x165 - x106 * obj_qk) * sensor_y;
	const GEN_FLT x169 = x5 * x168;
	const GEN_FLT x170 = x127 + x152 - x116 * obj_qk + x137 * obj_qk + x166 * x136;
	const GEN_FLT x171 = x47 * x170 + x50 * x169 + x6 * x167;
	const GEN_FLT x172 = x32 * x169 + x41 * x167 + x7 * x170;
	const GEN_FLT x173 = (-x43 * x171 + x53 * x172) * x57;
	const GEN_FLT x174 = x91 * x172;
	const GEN_FLT x175 = x60 * x170 + x61 * x168 + x63 * x167;
	const GEN_FLT x176 = -x173 - x66 * (-x71 * (x174 + x67 * x171) + x72 * x175);
	const GEN_FLT x177 = x43 * x175;
	const GEN_FLT x178 = x78 * x172;
	const GEN_FLT x179 = 2 * x75 * curve_1;
	const GEN_FLT x180 = pow(1 - x81 * x54 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x181 = tilt_1 / sqrt(x80);
	const GEN_FLT x182 = x64 * x68;
	const GEN_FLT x183 = (1.0 / 2.0) * x52 * tilt_1 / pow(x80, 3.0 / 2.0);
	const GEN_FLT x184 = -x180 * (-x183 * (x70 + x62 * x182) + x6 * x181) - (x77 - x79) * x82;
	const GEN_FLT x185 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x52 * x181) - atan2(-x64, x74)) * gibMag_1;
	const GEN_FLT x186 = 2 * x64;
	const GEN_FLT x187 = -x180 * (-x183 * (x86 + x61 * x186) + x51 * x181) - (x88 - x89) * x82;
	const GEN_FLT x188 = -x180 * (-x183 * (x92 + x59 * x182) + x47 * x181) - (x94 - x95) * x82;
	const GEN_FLT x189 = -x180 * (x118 * x181 - x183 * (x121 + x122 * x186)) - (x124 - x125) * x82;
	const GEN_FLT x190 = -x180 * (x181 * x140 - x183 * (x143 + x186 * x144)) - (x146 - x147) * x82;
	const GEN_FLT x191 = -x180 * (x181 * x157 - x183 * (x160 + x161 * x186)) - (x163 - x164) * x82;
	const GEN_FLT x192 = -x180 * (x171 * x181 - x183 * (x174 + x175 * x186)) - (x177 - x178) * x82;
	out[0] = x73 + x73 * x76 + (-x77 + x79) * x83;
	out[1] = x87 + x87 * x76 + (-x88 + x89) * x83;
	out[2] = x93 + x76 * x93 + (-x94 + x95) * x83;
	out[3] = x123 + x76 * x123 + (-x124 + x125) * x83;
	out[4] = x145 + x76 * x145 + (-x146 + x147) * x83;
	out[5] = x162 + x76 * x162 + (-x163 + x164) * x83;
	out[6] = x176 + x76 * x176 + (-x177 + x178) * x83;
	out[7] = x184 + x185 * x184 + x58 * x179;
	out[8] = x187 + x187 * x185 + x84 * x179;
	out[9] = x188 + x188 * x185 + x90 * x179;
	out[10] = x189 + x120 * x179 + x189 * x185;
	out[11] = x190 + x179 * x142 + x185 * x190;
	out[12] = x191 + x179 * x159 + x185 * x191;
	out[13] = x192 + x179 * x173 + x185 * x192;
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
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = pow(lh_qk, 2);
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt(x1 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - (x0 + x1) * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = pow(obj_qk, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = sqrt(x10 + x7 + pow(obj_qw, 2));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 - (x7 + x8) * x12;
	const GEN_FLT x14 = obj_qw * obj_qi;
	const GEN_FLT x15 = obj_qk * obj_qj;
	const GEN_FLT x16 = x14 + x15;
	const GEN_FLT x17 = x12 * x16;
	const GEN_FLT x18 = obj_qw * obj_qj;
	const GEN_FLT x19 = obj_qk * obj_qi;
	const GEN_FLT x20 = -x18 + x19;
	const GEN_FLT x21 = x20 * x12;
	const GEN_FLT x22 = obj_pz + x13 * sensor_z + x17 * sensor_y + x21 * sensor_x;
	const GEN_FLT x23 = -x14 + x15;
	const GEN_FLT x24 = x23 * x12;
	const GEN_FLT x25 = 1 - x12 * x10;
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x26 + x27;
	const GEN_FLT x29 = x28 * x12;
	const GEN_FLT x30 = obj_py + x24 * sensor_z + x25 * sensor_y + x29 * sensor_x;
	const GEN_FLT x31 = lh_qw * lh_qi;
	const GEN_FLT x32 = lh_qk * lh_qj;
	const GEN_FLT x33 = x31 + x32;
	const GEN_FLT x34 = x5 * x33;
	const GEN_FLT x35 = x18 + x19;
	const GEN_FLT x36 = -x26 + x27;
	const GEN_FLT x37 = 1 - (x7 + x9) * x12;
	const GEN_FLT x38 = obj_px + x37 * sensor_x + x35 * x12 * sensor_z + x36 * x12 * sensor_y;
	const GEN_FLT x39 = lh_qw * lh_qj;
	const GEN_FLT x40 = lh_qk * lh_qi;
	const GEN_FLT x41 = -x39 + x40;
	const GEN_FLT x42 = x5 * x41;
	const GEN_FLT x43 = lh_pz + x30 * x34 + x42 * x38 + x6 * x22;
	const GEN_FLT x44 = pow(x43, -1);
	const GEN_FLT x45 = 1 - x3 * x5;
	const GEN_FLT x46 = lh_qw * lh_qk;
	const GEN_FLT x47 = lh_qj * lh_qi;
	const GEN_FLT x48 = -x46 + x47;
	const GEN_FLT x49 = 4 * x4 * x11;
	const GEN_FLT x50 = x49 * x28;
	const GEN_FLT x51 = x39 + x40;
	const GEN_FLT x52 = x49 * x20;
	const GEN_FLT x53 = x45 * x37 + x50 * x48 + x52 * x51;
	const GEN_FLT x54 = x42 * x37 + x50 * x33 + x6 * x21;
	const GEN_FLT x55 = pow(x43, 2);
	const GEN_FLT x56 = pow(x55, -1);
	const GEN_FLT x57 = x5 * x22;
	const GEN_FLT x58 = x5 * x48;
	const GEN_FLT x59 = lh_px + x45 * x38 + x51 * x57 + x58 * x30;
	const GEN_FLT x60 = x56 * x59;
	const GEN_FLT x61 = pow(x59, 2);
	const GEN_FLT x62 = x55 + x61;
	const GEN_FLT x63 = pow(x62, -1);
	const GEN_FLT x64 = x63 * x55;
	const GEN_FLT x65 = (-x53 * x44 + x60 * x54) * x64;
	const GEN_FLT x66 = -x31 + x32;
	const GEN_FLT x67 = 1 - (x1 + x2) * x5;
	const GEN_FLT x68 = x46 + x47;
	const GEN_FLT x69 = x5 * x68;
	const GEN_FLT x70 = lh_py + x66 * x57 + x67 * x30 + x69 * x38;
	const GEN_FLT x71 = pow(x70, 2);
	const GEN_FLT x72 = pow(1 - x71 * x63 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x73 = 2 * x59;
	const GEN_FLT x74 = 2 * x43;
	const GEN_FLT x75 = x74 * x54;
	const GEN_FLT x76 = (1.0 / 2.0) * x70 * tilt_0 / pow(x62, 3.0 / 2.0);
	const GEN_FLT x77 = x66 * x52 + x67 * x29 + x69 * x37;
	const GEN_FLT x78 = tilt_0 / sqrt(x62);
	const GEN_FLT x79 = -x65 - x72 * (-x76 * (x75 + x73 * x53) + x78 * x77);
	const GEN_FLT x80 = -x43;
	const GEN_FLT x81 = atan2(x59, x80);
	const GEN_FLT x82 = sin(1.5707963267949 + gibPhase_0 - phase_0 - x81 - asin(x70 * x78)) * gibMag_0;
	const GEN_FLT x83 = x77 * x44;
	const GEN_FLT x84 = x70 * x56;
	const GEN_FLT x85 = x84 * x54;
	const GEN_FLT x86 = x55 + x71;
	const GEN_FLT x87 = pow(x86, -1);
	const GEN_FLT x88 = x87 * x55;
	const GEN_FLT x89 = 2 * x88 * atan2(x70, x80) * curve_0;
	const GEN_FLT x90 = x45 * x12;
	const GEN_FLT x91 = x49 * x16;
	const GEN_FLT x92 = x51 * x91 + x58 * x25 + x90 * x36;
	const GEN_FLT x93 = x49 * x36;
	const GEN_FLT x94 = x34 * x25 + x6 * x17 + x93 * x41;
	const GEN_FLT x95 = (x60 * x94 - x92 * x44) * x64;
	const GEN_FLT x96 = x74 * x94;
	const GEN_FLT x97 = x66 * x91 + x67 * x25 + x68 * x93;
	const GEN_FLT x98 = -x95 - x72 * (-x76 * (x96 + x73 * x92) + x78 * x97);
	const GEN_FLT x99 = x97 * x44;
	const GEN_FLT x100 = x84 * x94;
	const GEN_FLT x101 = x49 * x23;
	const GEN_FLT x102 = x5 * x13;
	const GEN_FLT x103 = x48 * x101 + x51 * x102 + x90 * x35;
	const GEN_FLT x104 = x49 * x35;
	const GEN_FLT x105 = x33 * x101 + x41 * x104 + x6 * x13;
	const GEN_FLT x106 = (-x44 * x103 + x60 * x105) * x64;
	const GEN_FLT x107 = x74 * x105;
	const GEN_FLT x108 = x66 * x102 + x67 * x24 + x68 * x104;
	const GEN_FLT x109 = -x106 - x72 * (-x76 * (x107 + x73 * x103) + x78 * x108);
	const GEN_FLT x110 = x44 * x108;
	const GEN_FLT x111 = x84 * x105;
	const GEN_FLT x112 = 2 * x81 * curve_1;
	const GEN_FLT x113 = pow(1 - x87 * x61 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x114 = tilt_1 / sqrt(x86);
	const GEN_FLT x115 = 2 * x70;
	const GEN_FLT x116 = (1.0 / 2.0) * x59 * tilt_1 / pow(x86, 3.0 / 2.0);
	const GEN_FLT x117 = -x113 * (-x116 * (x75 + x77 * x115) + x53 * x114) - (x83 - x85) * x88;
	const GEN_FLT x118 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x59 * x114) - atan2(-x70, x80)) * gibMag_1;
	const GEN_FLT x119 = -x113 * (-x116 * (x96 + x97 * x115) + x92 * x114) - x88 * (-x100 + x99);
	const GEN_FLT x120 = -x113 * (x103 * x114 - x116 * (x107 + x108 * x115)) - (x110 - x111) * x88;
	out[0] = x79 + x82 * x79 + (-x83 + x85) * x89;
	out[1] = x98 + x82 * x98 + x89 * (x100 - x99);
	out[2] = x109 + x82 * x109 + (-x110 + x111) * x89;
	out[3] = x117 + x118 * x117 + x65 * x112;
	out[4] = x119 + x118 * x119 + x95 * x112;
	out[5] = x120 + x106 * x112 + x118 * x120;
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
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qk, 2);
	const GEN_FLT x4 = x1 + x3;
	const GEN_FLT x5 = sqrt(x0 + x4 + pow(lh_qw, 2));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = pow(obj_qk, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = 2 * sqrt(x10 + x7 + pow(obj_qw, 2));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 = obj_pz + (1 - (x7 + x8) * x11) * sensor_z + (x12 + x13) * x14 + (-x15 + x16) * x17;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + (1 - x11 * x10) * sensor_y + (-x12 + x13) * x22 + (x23 + x24) * x17;
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = -x27 + x28;
	const GEN_FLT x30 = obj_px + (1 - (x7 + x9) * x11) * sensor_x + (x15 + x16) * x22 + (-x23 + x24) * x14;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + x18 * (1 - x2 * x6) + x21 * x26 + x31 * x29;
	const GEN_FLT x33 = x27 + x28;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = -x35 + x36;
	const GEN_FLT x38 = x0 + x3;
	const GEN_FLT x39 = lh_px + x30 * (1 - x6 * x38) + x34 * x33 + x37 * x26;
	const GEN_FLT x40 = pow(x39, 2);
	const GEN_FLT x41 = pow(x32, 2);
	const GEN_FLT x42 = x40 + x41;
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = x43 * x32;
	const GEN_FLT x45 = -x19 + x20;
	const GEN_FLT x46 = x35 + x36;
	const GEN_FLT x47 = lh_py + x25 * (1 - x4 * x6) + x45 * x34 + x46 * x31;
	const GEN_FLT x48 = x47 * x39;
	const GEN_FLT x49 = pow(x47, 2);
	const GEN_FLT x50 = pow(1 - x43 * x49 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x51 = tilt_0 / pow(x42, 3.0 / 2.0);
	const GEN_FLT x52 = x50 * x51;
	const GEN_FLT x53 = x44 + x52 * x48;
	const GEN_FLT x54 = tilt_0 / sqrt(x42);
	const GEN_FLT x55 = -x32;
	const GEN_FLT x56 = atan2(x39, x55);
	const GEN_FLT x57 = sin(1.5707963267949 + gibPhase_0 - phase_0 - x56 - asin(x54 * x47)) * gibMag_0;
	const GEN_FLT x58 = x50 * x54;
	const GEN_FLT x59 = x41 + x49;
	const GEN_FLT x60 = pow(x59, -1);
	const GEN_FLT x61 = x60 * x32;
	const GEN_FLT x62 = 2 * atan2(x47, x55) * curve_0;
	const GEN_FLT x63 = x43 * x39;
	const GEN_FLT x64 = -x63 + x52 * x47 * x32;
	const GEN_FLT x65 = x60 * x47;
	const GEN_FLT x66 = pow(x32, -1);
	const GEN_FLT x67 = 2 / x5;
	const GEN_FLT x68 = x67 * x38;
	const GEN_FLT x69 = x30 * lh_qw;
	const GEN_FLT x70 = x67 * x25;
	const GEN_FLT x71 = x70 * x37;
	const GEN_FLT x72 = x26 * lh_qk;
	const GEN_FLT x73 = x67 * x18;
	const GEN_FLT x74 = x73 * x33;
	const GEN_FLT x75 = x34 * lh_qj;
	const GEN_FLT x76 = -x72 + x75 - x68 * x69 + x71 * lh_qw + x74 * lh_qw;
	const GEN_FLT x77 = x67 * x29;
	const GEN_FLT x78 = x70 * x21;
	const GEN_FLT x79 = x26 * lh_qi;
	const GEN_FLT x80 = x31 * lh_qj;
	const GEN_FLT x81 = x2 * x67;
	const GEN_FLT x82 = x81 * x18;
	const GEN_FLT x83 = x79 - x80 + x77 * x69 + x78 * lh_qw - x82 * lh_qw;
	const GEN_FLT x84 = pow(x41, -1);
	const GEN_FLT x85 = x84 * x39;
	const GEN_FLT x86 = x41 * x43;
	const GEN_FLT x87 = (-x76 * x66 + x83 * x85) * x86;
	const GEN_FLT x88 = 2 * x39;
	const GEN_FLT x89 = 2 * x32;
	const GEN_FLT x90 = x83 * x89;
	const GEN_FLT x91 = (1.0 / 2.0) * x51 * x47;
	const GEN_FLT x92 = x67 * x46;
	const GEN_FLT x93 = x31 * lh_qk;
	const GEN_FLT x94 = x4 * x67;
	const GEN_FLT x95 = x94 * x25;
	const GEN_FLT x96 = x73 * x45;
	const GEN_FLT x97 = x34 * lh_qi;
	const GEN_FLT x98 = x93 - x97 + x69 * x92 - x95 * lh_qw + x96 * lh_qw;
	const GEN_FLT x99 = -x87 - x50 * (x54 * x98 - x91 * (x90 + x88 * x76));
	const GEN_FLT x100 = x66 * x98;
	const GEN_FLT x101 = x84 * x47;
	const GEN_FLT x102 = x83 * x101;
	const GEN_FLT x103 = x60 * x41;
	const GEN_FLT x104 = x62 * x103;
	const GEN_FLT x105 = x30 * lh_qi;
	const GEN_FLT x106 = x26 * lh_qj;
	const GEN_FLT x107 = x73 * lh_qi;
	const GEN_FLT x108 = x34 * lh_qk;
	const GEN_FLT x109 = x106 + x108 + x33 * x107 - x68 * x105 + x71 * lh_qi;
	const GEN_FLT x110 = x26 * lh_qw;
	const GEN_FLT x111 = 4 * x5;
	const GEN_FLT x112 = -x111 * lh_qi;
	const GEN_FLT x113 = x110 + x93 + x18 * (x112 - x81 * lh_qi) + x77 * x105 + x78 * lh_qi;
	const GEN_FLT x114 = (-x66 * x109 + x85 * x113) * x86;
	const GEN_FLT x115 = x89 * x113;
	const GEN_FLT x116 = x34 * lh_qw;
	const GEN_FLT x117 = -x116 + x80 + x25 * (x112 - x94 * lh_qi) + x45 * x107 + x92 * x105;
	const GEN_FLT x118 = -x114 - x50 * (x54 * x117 - x91 * (x115 + x88 * x109));
	const GEN_FLT x119 = x66 * x117;
	const GEN_FLT x120 = x101 * x113;
	const GEN_FLT x121 = -x111 * lh_qj;
	const GEN_FLT x122 = x116 + x79 + x30 * (x121 - x68 * lh_qj) + x71 * lh_qj + x74 * lh_qj;
	const GEN_FLT x123 = x30 * lh_qj;
	const GEN_FLT x124 = x31 * lh_qw;
	const GEN_FLT x125 = -x124 + x72 + x18 * (x121 - x81 * lh_qj) + x77 * x123 + x78 * lh_qj;
	const GEN_FLT x126 = (-x66 * x122 + x85 * x125) * x86;
	const GEN_FLT x127 = x89 * x125;
	const GEN_FLT x128 = x31 * lh_qi;
	const GEN_FLT x129 = x108 + x128 + x92 * x123 - x95 * lh_qj + x96 * lh_qj;
	const GEN_FLT x130 = -x126 - x50 * (x54 * x129 - x91 * (x127 + x88 * x122));
	const GEN_FLT x131 = x66 * x129;
	const GEN_FLT x132 = x101 * x125;
	const GEN_FLT x133 = -x111 * lh_qk;
	const GEN_FLT x134 = -x110 + x97 + x30 * (x133 - x68 * lh_qk) + x71 * lh_qk + x74 * lh_qk;
	const GEN_FLT x135 = x30 * lh_qk;
	const GEN_FLT x136 = x106 + x128 + x77 * x135 + x78 * lh_qk - x82 * lh_qk;
	const GEN_FLT x137 = (-x66 * x134 + x85 * x136) * x86;
	const GEN_FLT x138 = x89 * x136;
	const GEN_FLT x139 = x124 + x75 + x25 * (x133 - x94 * lh_qk) + x92 * x135 + x96 * lh_qk;
	const GEN_FLT x140 = -x137 - x50 * (x54 * x139 - x91 * (x138 + x88 * x134));
	const GEN_FLT x141 = x66 * x139;
	const GEN_FLT x142 = x101 * x136;
	const GEN_FLT x143 = pow(1 - x60 * x40 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x144 = tilt_1 / sqrt(x59);
	const GEN_FLT x145 = x144 * x143;
	const GEN_FLT x146 = 2 * x56 * curve_1;
	const GEN_FLT x147 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x39 * x144) - atan2(-x47, x55)) * gibMag_1;
	const GEN_FLT x148 = tilt_1 / pow(x59, 3.0 / 2.0);
	const GEN_FLT x149 = x143 * x148;
	const GEN_FLT x150 = -x61 + x48 * x149;
	const GEN_FLT x151 = x65 + x32 * x39 * x149;
	const GEN_FLT x152 = 2 * x47;
	const GEN_FLT x153 = (1.0 / 2.0) * x39 * x148;
	const GEN_FLT x154 = -x143 * (-x153 * (x90 + x98 * x152) + x76 * x144) - (x100 - x102) * x103;
	const GEN_FLT x155 = -x143 * (x109 * x144 - x153 * (x115 + x117 * x152)) - (x119 - x120) * x103;
	const GEN_FLT x156 = -x143 * (x122 * x144 - x153 * (x127 + x129 * x152)) - (x131 - x132) * x103;
	const GEN_FLT x157 = -x143 * (x134 * x144 - x153 * (x138 + x139 * x152)) - (x141 - x142) * x103;
	out[0] = x53 + x53 * x57;
	out[1] = -x58 - x58 * x57 - x61 * x62;
	out[2] = x64 + x62 * x65 + x64 * x57;
	out[3] = x99 + x57 * x99 + (-x100 + x102) * x104;
	out[4] = x118 + x57 * x118 + (-x119 + x120) * x104;
	out[5] = x130 + x57 * x130 + (-x131 + x132) * x104;
	out[6] = x140 + x57 * x140 + (-x141 + x142) * x104;
	out[7] = -x145 - x145 * x147 - x44 * x146;
	out[8] = x150 + x147 * x150;
	out[9] = x151 + x147 * x151 + x63 * x146;
	out[10] = x154 + x147 * x154 + x87 * x146;
	out[11] = x155 + x114 * x146 + x147 * x155;
	out[12] = x156 + x126 * x146 + x147 * x156;
	out[13] = x157 + x137 * x146 + x147 * x157;
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
	const GEN_FLT x2 = pow(obj_qj, 2);
	const GEN_FLT x3 = pow(obj_qi, 2);
	const GEN_FLT x4 = x2 + x3;
	const GEN_FLT x5 = pow(obj_qk, 2);
	const GEN_FLT x6 = 2 * sqrt(x4 + x5 + pow(obj_qw, 2));
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x6 * sensor_y;
	const GEN_FLT x10 = obj_qw * obj_qj;
	const GEN_FLT x11 = obj_qk * obj_qi;
	const GEN_FLT x12 = x6 * sensor_x;
	const GEN_FLT x13 = obj_pz + (1 - x4 * x6) * sensor_z + (-x10 + x11) * x12 + (x7 + x8) * x9;
	const GEN_FLT x14 = pow(lh_qi, 2);
	const GEN_FLT x15 = pow(lh_qk, 2);
	const GEN_FLT x16 = pow(lh_qj, 2);
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = 2 * sqrt(x14 + x17 + pow(lh_qw, 2));
	const GEN_FLT x19 = x13 * x18;
	const GEN_FLT x20 = x6 * sensor_z;
	const GEN_FLT x21 = obj_qw * obj_qk;
	const GEN_FLT x22 = obj_qj * obj_qi;
	const GEN_FLT x23 = obj_py + (1 - (x3 + x5) * x6) * sensor_y + (x21 + x22) * x12 + (-x7 + x8) * x20;
	const GEN_FLT x24 = lh_qw * lh_qk;
	const GEN_FLT x25 = lh_qj * lh_qi;
	const GEN_FLT x26 = obj_px + (1 - (x2 + x5) * x6) * sensor_x + (x10 + x11) * x20 + (-x21 + x22) * x9;
	const GEN_FLT x27 = x26 * x18;
	const GEN_FLT x28 = lh_py + x23 * (1 - (x14 + x15) * x18) + (-x0 + x1) * x19 + (x24 + x25) * x27;
	const GEN_FLT x29 = x23 * x18;
	const GEN_FLT x30 = lh_qw * lh_qj;
	const GEN_FLT x31 = lh_qk * lh_qi;
	const GEN_FLT x32 = lh_pz + x13 * (1 - (x14 + x16) * x18) + (x0 + x1) * x29 + (-x30 + x31) * x27;
	const GEN_FLT x33 = pow(x32, 2);
	const GEN_FLT x34 = lh_px + x26 * (1 - x18 * x17) + (-x24 + x25) * x29 + (x30 + x31) * x19;
	const GEN_FLT x35 = pow(x34, 2);
	const GEN_FLT x36 = x33 + x35;
	const GEN_FLT x37 = x28 / sqrt(x36);
	const GEN_FLT x38 = -x32;
	const GEN_FLT x39 = atan2(x34, x38);
	const GEN_FLT x40 = 1.5707963267949 + gibPhase_0 - phase_0 - x39 - asin(x37 * tilt_0);
	const GEN_FLT x41 = sin(x40) * gibMag_0;
	const GEN_FLT x42 = pow(x28, 2);
	const GEN_FLT x43 = x37 / sqrt(1 - x42 * pow(tilt_0, 2) / x36);
	const GEN_FLT x44 = x33 + x42;
	const GEN_FLT x45 = x34 / sqrt(x44);
	const GEN_FLT x46 = 1.5707963267949 + gibPhase_1 - phase_1 - asin(x45 * tilt_1) - atan2(-x28, x38);
	const GEN_FLT x47 = sin(x46) * gibMag_1;
	const GEN_FLT x48 = x45 / sqrt(1 - x35 * pow(tilt_1, 2) / x44);
	out[0] = -1 - x41;
	out[1] = -x43 - x41 * x43;
	out[2] = pow(atan2(x28, x38), 2);
	out[3] = x41;
	out[4] = -cos(x40);
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
	out[21] = -1 - x47;
	out[22] = -x48 - x47 * x48;
	out[23] = pow(x39, 2);
	out[24] = x47;
	out[25] = -cos(x46);
	out[26] = 0;
	out[27] = 0;
}

/** Applying function <function reproject_axis_x at 0x7effc26e33b0> */
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
	const GEN_FLT x2 = pow(obj_qj, 2);
	const GEN_FLT x3 = pow(obj_qi, 2);
	const GEN_FLT x4 = x2 + x3;
	const GEN_FLT x5 = pow(obj_qk, 2);
	const GEN_FLT x6 = 2 * sqrt(x4 + x5 + pow(obj_qw, 2));
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x6 * sensor_y;
	const GEN_FLT x10 = obj_qw * obj_qj;
	const GEN_FLT x11 = obj_qk * obj_qi;
	const GEN_FLT x12 = x6 * sensor_x;
	const GEN_FLT x13 = obj_pz + (1 - x4 * x6) * sensor_z + (-x10 + x11) * x12 + (x7 + x8) * x9;
	const GEN_FLT x14 = pow(lh_qi, 2);
	const GEN_FLT x15 = pow(lh_qk, 2);
	const GEN_FLT x16 = pow(lh_qj, 2);
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = 2 * sqrt(x14 + x17 + pow(lh_qw, 2));
	const GEN_FLT x19 = x13 * x18;
	const GEN_FLT x20 = x6 * sensor_z;
	const GEN_FLT x21 = obj_qw * obj_qk;
	const GEN_FLT x22 = obj_qj * obj_qi;
	const GEN_FLT x23 = obj_py + (1 - (x3 + x5) * x6) * sensor_y + (x21 + x22) * x12 + (-x7 + x8) * x20;
	const GEN_FLT x24 = lh_qw * lh_qk;
	const GEN_FLT x25 = lh_qj * lh_qi;
	const GEN_FLT x26 = obj_px + (1 - (x2 + x5) * x6) * sensor_x + (x10 + x11) * x20 + (-x21 + x22) * x9;
	const GEN_FLT x27 = x26 * x18;
	const GEN_FLT x28 = lh_py + x23 * (1 - (x14 + x15) * x18) + (-x0 + x1) * x19 + (x24 + x25) * x27;
	const GEN_FLT x29 = x23 * x18;
	const GEN_FLT x30 = lh_qw * lh_qj;
	const GEN_FLT x31 = lh_qk * lh_qi;
	const GEN_FLT x32 = lh_pz + x13 * (1 - (x14 + x16) * x18) + (x0 + x1) * x29 + (-x30 + x31) * x27;
	const GEN_FLT x33 = -x32;
	const GEN_FLT x34 = lh_px + x26 * (1 - x18 * x17) + (-x24 + x25) * x29 + (x30 + x31) * x19;
	const GEN_FLT x35 = -phase_0 - asin(x28 * tilt_0 / sqrt(pow(x32, 2) + pow(x34, 2))) - atan2(x34, x33);
	out[0] = x35 - cos(1.5707963267949 + gibPhase_0 + x35) * gibMag_0 + pow(atan2(x28, x33), 2) * curve_0;
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt(x1 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - (x0 + x1) * x5;
	const GEN_FLT x7 = 1 - (x1 + x2) * x5;
	const GEN_FLT x8 = pow(obj_qj, 2);
	const GEN_FLT x9 = pow(obj_qi, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = x11 + x9;
	const GEN_FLT x13 = sqrt(x12 + x8 + pow(obj_qw, 2));
	const GEN_FLT x14 = 2 * x13;
	const GEN_FLT x15 = obj_qw * obj_qi;
	const GEN_FLT x16 = obj_qk * obj_qj;
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = x14 * sensor_y;
	const GEN_FLT x19 = obj_qw * obj_qj;
	const GEN_FLT x20 = obj_qk * obj_qi;
	const GEN_FLT x21 = -x19 + x20;
	const GEN_FLT x22 = x14 * sensor_x;
	const GEN_FLT x23 = obj_pz + x18 * x17 + x22 * x21 + (1 - x14 * x10) * sensor_z;
	const GEN_FLT x24 = -x15 + x16;
	const GEN_FLT x25 = x14 * sensor_z;
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x26 + x27;
	const GEN_FLT x29 = obj_py + x22 * x28 + x24 * x25 + (1 - x14 * x12) * sensor_y;
	const GEN_FLT x30 = lh_qw * lh_qi;
	const GEN_FLT x31 = lh_qk * lh_qj;
	const GEN_FLT x32 = x30 + x31;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = x19 + x20;
	const GEN_FLT x35 = -x26 + x27;
	const GEN_FLT x36 = x11 + x8;
	const GEN_FLT x37 = obj_px + x34 * x25 + x35 * x18 + (1 - x36 * x14) * sensor_x;
	const GEN_FLT x38 = lh_qw * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qi;
	const GEN_FLT x40 = -x38 + x39;
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = lh_pz + x33 * x29 + x41 * x37 + x7 * x23;
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = pow(x42, 2);
	const GEN_FLT x45 = pow(x44, -1);
	const GEN_FLT x46 = x38 + x39;
	const GEN_FLT x47 = x5 * x23;
	const GEN_FLT x48 = lh_qw * lh_qk;
	const GEN_FLT x49 = lh_qj * lh_qi;
	const GEN_FLT x50 = -x48 + x49;
	const GEN_FLT x51 = x5 * x50;
	const GEN_FLT x52 = lh_px + x46 * x47 + x51 * x29 + x6 * x37;
	const GEN_FLT x53 = x52 * x45;
	const GEN_FLT x54 = x44 + pow(x52, 2);
	const GEN_FLT x55 = pow(x54, -1);
	const GEN_FLT x56 = x55 * x44;
	const GEN_FLT x57 = -x30 + x31;
	const GEN_FLT x58 = 1 - x3 * x5;
	const GEN_FLT x59 = (x48 + x49) * x5;
	const GEN_FLT x60 = lh_py + x57 * x47 + x58 * x29 + x59 * x37;
	const GEN_FLT x61 = pow(x60, 2);
	const GEN_FLT x62 = pow(1 - x61 * x55 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x63 = 2 * x52;
	const GEN_FLT x64 = 4 * x4;
	const GEN_FLT x65 = x64 * x42;
	const GEN_FLT x66 = (1.0 / 2.0) * x60 * tilt_0 / pow(x54, 3.0 / 2.0);
	const GEN_FLT x67 = tilt_0 / sqrt(x54);
	const GEN_FLT x68 = -x56 * (x53 * x41 - x6 * x43) - x62 * (-x66 * (x6 * x63 + x65 * x40) + x67 * x59);
	const GEN_FLT x69 = -x42;
	const GEN_FLT x70 = sin(1.5707963267949 + gibPhase_0 - phase_0 - asin(x60 * x67) - atan2(x52, x69)) * gibMag_0;
	const GEN_FLT x71 = x60 * x45;
	const GEN_FLT x72 = 2 * x44 * atan2(x60, x69) * curve_0 / (x44 + x61);
	const GEN_FLT x73 = x64 * x52;
	const GEN_FLT x74 = -x62 * (x67 * x58 - (x65 * x32 + x73 * x50) * x66) - (-x51 * x43 + x53 * x33) * x56;
	const GEN_FLT x75 = x5 * x46;
	const GEN_FLT x76 = 2 * x42;
	const GEN_FLT x77 = x5 * x57;
	const GEN_FLT x78 = -x56 * (x7 * x53 - x75 * x43) - x62 * (-x66 * (x7 * x76 + x73 * x46) + x77 * x67);
	const GEN_FLT x79 = 2 / x13;
	const GEN_FLT x80 = x79 * x36;
	const GEN_FLT x81 = x80 * sensor_x;
	const GEN_FLT x82 = x79 * sensor_y;
	const GEN_FLT x83 = x82 * x35;
	const GEN_FLT x84 = x14 * obj_qk;
	const GEN_FLT x85 = x84 * sensor_y;
	const GEN_FLT x86 = x34 * sensor_z;
	const GEN_FLT x87 = x86 * x79;
	const GEN_FLT x88 = x25 * obj_qj;
	const GEN_FLT x89 = -x85 + x88 - x81 * obj_qw + x83 * obj_qw + x87 * obj_qw;
	const GEN_FLT x90 = x28 * sensor_x;
	const GEN_FLT x91 = x79 * x90;
	const GEN_FLT x92 = x79 * x12;
	const GEN_FLT x93 = x92 * sensor_y;
	const GEN_FLT x94 = x22 * obj_qk;
	const GEN_FLT x95 = x24 * sensor_z;
	const GEN_FLT x96 = x79 * x95;
	const GEN_FLT x97 = x14 * obj_qi;
	const GEN_FLT x98 = x97 * sensor_z;
	const GEN_FLT x99 = x94 - x98 + x91 * obj_qw - x93 * obj_qw + x96 * obj_qw;
	const GEN_FLT x100 = x22 * obj_qj;
	const GEN_FLT x101 = x21 * sensor_x;
	const GEN_FLT x102 = x79 * x101;
	const GEN_FLT x103 = x82 * x17;
	const GEN_FLT x104 = x97 * sensor_y;
	const GEN_FLT x105 = x79 * x10;
	const GEN_FLT x106 = x105 * sensor_z;
	const GEN_FLT x107 = -x100 + x104 + x102 * obj_qw + x103 * obj_qw - x106 * obj_qw;
	const GEN_FLT x108 = x5 * x107;
	const GEN_FLT x109 = x46 * x108 + x51 * x99 + x6 * x89;
	const GEN_FLT x110 = x7 * x107 + x89 * x41 + x99 * x33;
	const GEN_FLT x111 = x57 * x108 + x58 * x99 + x89 * x59;
	const GEN_FLT x112 = -x62 * (x67 * x111 - (x63 * x109 + x76 * x110) * x66) - (-x43 * x109 + x53 * x110) * x56;
	const GEN_FLT x113 = x79 * obj_qi;
	const GEN_FLT x114 = x113 * sensor_y;
	const GEN_FLT x115 = x18 * obj_qj;
	const GEN_FLT x116 = x84 * sensor_z;
	const GEN_FLT x117 = x115 + x116 + x35 * x114 - x81 * obj_qi + x86 * x113;
	const GEN_FLT x118 = 4 * x13;
	const GEN_FLT x119 = -x118 * obj_qi;
	const GEN_FLT x120 = x14 * obj_qw;
	const GEN_FLT x121 = x120 * sensor_z;
	const GEN_FLT x122 = x100 - x121 + x90 * x113 + x95 * x113 + (x119 - x92 * obj_qi) * sensor_y;
	const GEN_FLT x123 = x120 * sensor_y;
	const GEN_FLT x124 = x123 + x94 + x101 * x113 + x17 * x114 + (x119 - x105 * obj_qi) * sensor_z;
	const GEN_FLT x125 = x51 * x122 + x6 * x117 + x75 * x124;
	const GEN_FLT x126 = x33 * x122 + x41 * x117 + x7 * x124;
	const GEN_FLT x127 = x58 * x122 + x59 * x117 + x77 * x124;
	const GEN_FLT x128 = -x62 * (x67 * x127 - (x63 * x125 + x76 * x126) * x66) - (-x43 * x125 + x53 * x126) * x56;
	const GEN_FLT x129 = -x118 * obj_qj;
	const GEN_FLT x130 = x79 * obj_qj;
	const GEN_FLT x131 = x104 + x121 + x83 * obj_qj + x86 * x130 + (x129 - x80 * obj_qj) * sensor_x;
	const GEN_FLT x132 = x22 * obj_qi;
	const GEN_FLT x133 = x116 + x132 + x91 * obj_qj - x93 * obj_qj + x96 * obj_qj;
	const GEN_FLT x134 = x22 * obj_qw;
	const GEN_FLT x135 = -x134 + x85 + x101 * x130 + x103 * obj_qj + (x129 - x105 * obj_qj) * sensor_z;
	const GEN_FLT x136 = x51 * x133 + x6 * x131 + x75 * x135;
	const GEN_FLT x137 = x33 * x133 + x41 * x131 + x7 * x135;
	const GEN_FLT x138 = x58 * x133 + x59 * x131 + x77 * x135;
	const GEN_FLT x139 = -x62 * (x67 * x138 - (x63 * x136 + x76 * x137) * x66) - (-x43 * x136 + x53 * x137) * x56;
	const GEN_FLT x140 = -x118 * obj_qk;
	const GEN_FLT x141 = -x123 + x98 + x83 * obj_qk + x87 * obj_qk + (x140 - x80 * obj_qk) * sensor_x;
	const GEN_FLT x142 = x134 + x88 + x91 * obj_qk + x96 * obj_qk + (x140 - x92 * obj_qk) * sensor_y;
	const GEN_FLT x143 = x115 + x132 + x102 * obj_qk + x103 * obj_qk - x106 * obj_qk;
	const GEN_FLT x144 = x5 * x143;
	const GEN_FLT x145 = x46 * x144 + x51 * x142 + x6 * x141;
	const GEN_FLT x146 = x33 * x142 + x41 * x141 + x7 * x143;
	const GEN_FLT x147 = x57 * x144 + x58 * x142 + x59 * x141;
	const GEN_FLT x148 = -x62 * (x67 * x147 - (x63 * x145 + x76 * x146) * x66) - (-x43 * x145 + x53 * x146) * x56;
	out[0] = x68 + x70 * x68 + (-x59 * x43 + x71 * x41) * x72;
	out[1] = x74 + x70 * x74 + (-x58 * x43 + x71 * x33) * x72;
	out[2] = x78 + x70 * x78 + x72 * (x7 * x71 - x77 * x43);
	out[3] = x112 + x70 * x112 + (-x43 * x111 + x71 * x110) * x72;
	out[4] = x128 + x70 * x128 + (-x43 * x127 + x71 * x126) * x72;
	out[5] = x139 + x70 * x139 + (-x43 * x138 + x71 * x137) * x72;
	out[6] = x148 + x70 * x148 + (-x43 * x147 + x71 * x146) * x72;
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
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qk, 2);
	const GEN_FLT x4 = sqrt(x2 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - x2 * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = pow(obj_qk, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = sqrt(x10 + x7 + pow(obj_qw, 2));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 - (x7 + x8) * x12;
	const GEN_FLT x14 = obj_qw * obj_qi;
	const GEN_FLT x15 = obj_qk * obj_qj;
	const GEN_FLT x16 = x14 + x15;
	const GEN_FLT x17 = x12 * sensor_y;
	const GEN_FLT x18 = obj_qw * obj_qj;
	const GEN_FLT x19 = obj_qk * obj_qi;
	const GEN_FLT x20 = -x18 + x19;
	const GEN_FLT x21 = x12 * sensor_x;
	const GEN_FLT x22 = obj_pz + x13 * sensor_z + x17 * x16 + x20 * x21;
	const GEN_FLT x23 = lh_qw * lh_qi;
	const GEN_FLT x24 = lh_qk * lh_qj;
	const GEN_FLT x25 = x23 + x24;
	const GEN_FLT x26 = -x14 + x15;
	const GEN_FLT x27 = x12 * sensor_z;
	const GEN_FLT x28 = 1 - x12 * x10;
	const GEN_FLT x29 = obj_qw * obj_qk;
	const GEN_FLT x30 = obj_qj * obj_qi;
	const GEN_FLT x31 = x29 + x30;
	const GEN_FLT x32 = obj_py + x26 * x27 + x28 * sensor_y + x31 * x21;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = lh_qw * lh_qj;
	const GEN_FLT x35 = lh_qk * lh_qi;
	const GEN_FLT x36 = -x34 + x35;
	const GEN_FLT x37 = x18 + x19;
	const GEN_FLT x38 = -x29 + x30;
	const GEN_FLT x39 = 1 - (x7 + x9) * x12;
	const GEN_FLT x40 = obj_px + x37 * x27 + x38 * x17 + x39 * sensor_x;
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = lh_pz + x33 * x25 + x41 * x36 + x6 * x22;
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = 1 - (x0 + x3) * x5;
	const GEN_FLT x45 = lh_qw * lh_qk;
	const GEN_FLT x46 = lh_qj * lh_qi;
	const GEN_FLT x47 = -x45 + x46;
	const GEN_FLT x48 = 4 * x4 * x11;
	const GEN_FLT x49 = x48 * x31;
	const GEN_FLT x50 = x34 + x35;
	const GEN_FLT x51 = x48 * x20;
	const GEN_FLT x52 = x44 * x39 + x47 * x49 + x50 * x51;
	const GEN_FLT x53 = x5 * x39;
	const GEN_FLT x54 = x6 * x12;
	const GEN_FLT x55 = x49 * x25 + x53 * x36 + x54 * x20;
	const GEN_FLT x56 = x5 * x22;
	const GEN_FLT x57 = lh_px + x40 * x44 + x47 * x33 + x50 * x56;
	const GEN_FLT x58 = pow(x42, 2);
	const GEN_FLT x59 = pow(x58, -1);
	const GEN_FLT x60 = x57 * x59;
	const GEN_FLT x61 = x58 + pow(x57, 2);
	const GEN_FLT x62 = pow(x61, -1);
	const GEN_FLT x63 = x62 * x58;
	const GEN_FLT x64 = -x23 + x24;
	const GEN_FLT x65 = 1 - (x1 + x3) * x5;
	const GEN_FLT x66 = x45 + x46;
	const GEN_FLT x67 = lh_py + x64 * x56 + x65 * x32 + x66 * x41;
	const GEN_FLT x68 = pow(x67, 2);
	const GEN_FLT x69 = pow(1 - x62 * x68 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x70 = 2 * x57;
	const GEN_FLT x71 = 2 * x42;
	const GEN_FLT x72 = (1.0 / 2.0) * x67 * tilt_0 / pow(x61, 3.0 / 2.0);
	const GEN_FLT x73 = x65 * x12;
	const GEN_FLT x74 = x64 * x51 + x66 * x53 + x73 * x31;
	const GEN_FLT x75 = tilt_0 / sqrt(x61);
	const GEN_FLT x76 = -x69 * (x75 * x74 - (x70 * x52 + x71 * x55) * x72) - (-x52 * x43 + x60 * x55) * x63;
	const GEN_FLT x77 = -x42;
	const GEN_FLT x78 = sin(1.5707963267949 + gibPhase_0 - phase_0 - asin(x75 * x67) - atan2(x57, x77)) * gibMag_0;
	const GEN_FLT x79 = x67 * x59;
	const GEN_FLT x80 = 2 * x58 * atan2(x67, x77) * curve_0 / (x58 + x68);
	const GEN_FLT x81 = x5 * x28;
	const GEN_FLT x82 = x44 * x12;
	const GEN_FLT x83 = x48 * x16;
	const GEN_FLT x84 = x81 * x47 + x82 * x38 + x83 * x50;
	const GEN_FLT x85 = x48 * x38;
	const GEN_FLT x86 = x54 * x16 + x81 * x25 + x85 * x36;
	const GEN_FLT x87 = x65 * x28 + x83 * x64 + x85 * x66;
	const GEN_FLT x88 = -x69 * (x87 * x75 - (x84 * x70 + x86 * x71) * x72) - (-x84 * x43 + x86 * x60) * x63;
	const GEN_FLT x89 = x48 * x26;
	const GEN_FLT x90 = x5 * x13;
	const GEN_FLT x91 = x50 * x90 + x82 * x37 + x89 * x47;
	const GEN_FLT x92 = x48 * x37;
	const GEN_FLT x93 = x6 * x13 + x89 * x25 + x92 * x36;
	const GEN_FLT x94 = x64 * x90 + x66 * x92 + x73 * x26;
	const GEN_FLT x95 = -x69 * (x75 * x94 - (x70 * x91 + x71 * x93) * x72) - (x60 * x93 - x91 * x43) * x63;
	out[0] = x76 + x78 * x76 + (-x74 * x43 + x79 * x55) * x80;
	out[1] = x88 + x88 * x78 + (x86 * x79 - x87 * x43) * x80;
	out[2] = x95 + x78 * x95 + (x79 * x93 - x94 * x43) * x80;
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
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qk, 2);
	const GEN_FLT x4 = x1 + x3;
	const GEN_FLT x5 = sqrt(x0 + x4 + pow(lh_qw, 2));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = pow(obj_qk, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = 2 * sqrt(x10 + x7 + pow(obj_qw, 2));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 = obj_pz + (1 - (x7 + x8) * x11) * sensor_z + (x12 + x13) * x14 + (-x15 + x16) * x17;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + (1 - x11 * x10) * sensor_y + (-x12 + x13) * x22 + (x23 + x24) * x17;
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = -x27 + x28;
	const GEN_FLT x30 = obj_px + (1 - (x7 + x9) * x11) * sensor_x + (x15 + x16) * x22 + (-x23 + x24) * x14;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + x18 * (1 - x2 * x6) + x21 * x26 + x31 * x29;
	const GEN_FLT x33 = x27 + x28;
	const GEN_FLT x34 = x6 * x18;
	const GEN_FLT x35 = lh_qw * lh_qk;
	const GEN_FLT x36 = lh_qj * lh_qi;
	const GEN_FLT x37 = -x35 + x36;
	const GEN_FLT x38 = x0 + x3;
	const GEN_FLT x39 = lh_px + x30 * (1 - x6 * x38) + x34 * x33 + x37 * x26;
	const GEN_FLT x40 = pow(x32, 2);
	const GEN_FLT x41 = x40 + pow(x39, 2);
	const GEN_FLT x42 = pow(x41, -1);
	const GEN_FLT x43 = -x19 + x20;
	const GEN_FLT x44 = x35 + x36;
	const GEN_FLT x45 = lh_py + x25 * (1 - x4 * x6) + x43 * x34 + x44 * x31;
	const GEN_FLT x46 = pow(x45, 2);
	const GEN_FLT x47 = pow(1 - x42 * x46 * pow(tilt_0, 2), -1.0 / 2.0);
	const GEN_FLT x48 = x45 * tilt_0 / pow(x41, 3.0 / 2.0);
	const GEN_FLT x49 = x47 * x48;
	const GEN_FLT x50 = x42 * x32 + x49 * x39;
	const GEN_FLT x51 = tilt_0 / sqrt(x41);
	const GEN_FLT x52 = -x32;
	const GEN_FLT x53 = sin(1.5707963267949 + gibPhase_0 - phase_0 - asin(x51 * x45) - atan2(x39, x52)) * gibMag_0;
	const GEN_FLT x54 = x51 * x47;
	const GEN_FLT x55 = 2 * x32;
	const GEN_FLT x56 = atan2(x45, x52) * curve_0 / (x40 + x46);
	const GEN_FLT x57 = -x42 * x39 + x49 * x32;
	const GEN_FLT x58 = 2 * x56;
	const GEN_FLT x59 = pow(x32, -1);
	const GEN_FLT x60 = 2 / x5;
	const GEN_FLT x61 = x60 * x38;
	const GEN_FLT x62 = x61 * x30;
	const GEN_FLT x63 = x60 * lh_qw;
	const GEN_FLT x64 = x37 * x25;
	const GEN_FLT x65 = x26 * lh_qk;
	const GEN_FLT x66 = x60 * x18;
	const GEN_FLT x67 = x66 * x33;
	const GEN_FLT x68 = x34 * lh_qj;
	const GEN_FLT x69 = -x65 + x68 - x62 * lh_qw + x63 * x64 + x67 * lh_qw;
	const GEN_FLT x70 = x30 * x29;
	const GEN_FLT x71 = x25 * x21;
	const GEN_FLT x72 = x26 * lh_qi;
	const GEN_FLT x73 = x31 * lh_qj;
	const GEN_FLT x74 = x2 * x60;
	const GEN_FLT x75 = x74 * x18;
	const GEN_FLT x76 = x72 - x73 + x70 * x63 + x71 * x63 - x75 * lh_qw;
	const GEN_FLT x77 = pow(x40, -1);
	const GEN_FLT x78 = x77 * x39;
	const GEN_FLT x79 = x40 * x42;
	const GEN_FLT x80 = 2 * x39;
	const GEN_FLT x81 = (1.0 / 2.0) * x48;
	const GEN_FLT x82 = x44 * x30;
	const GEN_FLT x83 = x31 * lh_qk;
	const GEN_FLT x84 = x4 * x60;
	const GEN_FLT x85 = x84 * x25;
	const GEN_FLT x86 = x66 * x43;
	const GEN_FLT x87 = x34 * lh_qi;
	const GEN_FLT x88 = x83 - x87 + x82 * x63 - x85 * lh_qw + x86 * lh_qw;
	const GEN_FLT x89 = -x47 * (x88 * x51 - (x76 * x55 + x80 * x69) * x81) - (-x69 * x59 + x78 * x76) * x79;
	const GEN_FLT x90 = x77 * x45;
	const GEN_FLT x91 = x58 * x40;
	const GEN_FLT x92 = x60 * lh_qi;
	const GEN_FLT x93 = x26 * lh_qj;
	const GEN_FLT x94 = x34 * lh_qk;
	const GEN_FLT x95 = x93 + x94 - x62 * lh_qi + x64 * x92 + x67 * lh_qi;
	const GEN_FLT x96 = x26 * lh_qw;
	const GEN_FLT x97 = 4 * x5;
	const GEN_FLT x98 = -x97 * lh_qi;
	const GEN_FLT x99 = x83 + x96 + x18 * (x98 - x74 * lh_qi) + x70 * x92 + x71 * x92;
	const GEN_FLT x100 = x34 * lh_qw;
	const GEN_FLT x101 = -x100 + x73 + x25 * (x98 - x84 * lh_qi) + x82 * x92 + x86 * lh_qi;
	const GEN_FLT x102 = -x47 * (x51 * x101 - (x55 * x99 + x80 * x95) * x81) - (-x59 * x95 + x78 * x99) * x79;
	const GEN_FLT x103 = -x97 * lh_qj;
	const GEN_FLT x104 = x60 * lh_qj;
	const GEN_FLT x105 = x100 + x72 + x30 * (x103 - x61 * lh_qj) + x64 * x104 + x67 * lh_qj;
	const GEN_FLT x106 = x31 * lh_qw;
	const GEN_FLT x107 = -x106 + x65 + x18 * (x103 - x74 * lh_qj) + x70 * x104 + x71 * x104;
	const GEN_FLT x108 = x31 * lh_qi;
	const GEN_FLT x109 = x108 + x94 + x82 * x104 - x85 * lh_qj + x86 * lh_qj;
	const GEN_FLT x110 = -x47 * (x51 * x109 - (x55 * x107 + x80 * x105) * x81) - (-x59 * x105 + x78 * x107) * x79;
	const GEN_FLT x111 = -x97 * lh_qk;
	const GEN_FLT x112 = x60 * lh_qk;
	const GEN_FLT x113 = x25 * x112;
	const GEN_FLT x114 = x87 - x96 + x30 * (x111 - x61 * lh_qk) + x37 * x113 + x67 * lh_qk;
	const GEN_FLT x115 = x108 + x93 + x21 * x113 + x70 * x112 - x75 * lh_qk;
	const GEN_FLT x116 = x106 + x68 + x25 * (x111 - x84 * lh_qk) + x82 * x112 + x86 * lh_qk;
	const GEN_FLT x117 = -x47 * (x51 * x116 - (x55 * x115 + x80 * x114) * x81) - (-x59 * x114 + x78 * x115) * x79;
	out[0] = x50 + x50 * x53;
	out[1] = -x54 - x54 * x53 - x56 * x55;
	out[2] = x57 + x53 * x57 + x58 * x45;
	out[3] = x89 + x89 * x53 + (x76 * x90 - x88 * x59) * x91;
	out[4] = x102 + x53 * x102 + x91 * (-x59 * x101 + x90 * x99);
	out[5] = x110 + x53 * x110 + (-x59 * x109 + x90 * x107) * x91;
	out[6] = x117 + x53 * x117 + (-x59 * x116 + x90 * x115) * x91;
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
	const GEN_FLT x2 = pow(obj_qj, 2);
	const GEN_FLT x3 = pow(obj_qi, 2);
	const GEN_FLT x4 = x2 + x3;
	const GEN_FLT x5 = pow(obj_qk, 2);
	const GEN_FLT x6 = 2 * sqrt(x4 + x5 + pow(obj_qw, 2));
	const GEN_FLT x7 = obj_qw * obj_qi;
	const GEN_FLT x8 = obj_qk * obj_qj;
	const GEN_FLT x9 = x6 * sensor_y;
	const GEN_FLT x10 = obj_qw * obj_qj;
	const GEN_FLT x11 = obj_qk * obj_qi;
	const GEN_FLT x12 = x6 * sensor_x;
	const GEN_FLT x13 = obj_pz + (1 - x4 * x6) * sensor_z + (-x10 + x11) * x12 + (x7 + x8) * x9;
	const GEN_FLT x14 = pow(lh_qi, 2);
	const GEN_FLT x15 = pow(lh_qk, 2);
	const GEN_FLT x16 = pow(lh_qj, 2);
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = 2 * sqrt(x14 + x17 + pow(lh_qw, 2));
	const GEN_FLT x19 = x13 * x18;
	const GEN_FLT x20 = x6 * sensor_z;
	const GEN_FLT x21 = obj_qw * obj_qk;
	const GEN_FLT x22 = obj_qj * obj_qi;
	const GEN_FLT x23 = obj_py + (1 - (x3 + x5) * x6) * sensor_y + (x21 + x22) * x12 + (-x7 + x8) * x20;
	const GEN_FLT x24 = lh_qw * lh_qk;
	const GEN_FLT x25 = lh_qj * lh_qi;
	const GEN_FLT x26 = obj_px + (1 - (x2 + x5) * x6) * sensor_x + (x10 + x11) * x20 + (-x21 + x22) * x9;
	const GEN_FLT x27 = x26 * x18;
	const GEN_FLT x28 = lh_py + x23 * (1 - (x14 + x15) * x18) + (-x0 + x1) * x19 + (x24 + x25) * x27;
	const GEN_FLT x29 = x23 * x18;
	const GEN_FLT x30 = lh_qw * lh_qj;
	const GEN_FLT x31 = lh_qk * lh_qi;
	const GEN_FLT x32 = lh_pz + x13 * (1 - (x14 + x16) * x18) + (x0 + x1) * x29 + (-x30 + x31) * x27;
	const GEN_FLT x33 = lh_px + x26 * (1 - x18 * x17) + (-x24 + x25) * x29 + (x30 + x31) * x19;
	const GEN_FLT x34 = pow(x32, 2) + pow(x33, 2);
	const GEN_FLT x35 = x28 / sqrt(x34);
	const GEN_FLT x36 = -x32;
	const GEN_FLT x37 = 1.5707963267949 + gibPhase_0 - phase_0 - asin(x35 * tilt_0) - atan2(x33, x36);
	const GEN_FLT x38 = sin(x37) * gibMag_0;
	const GEN_FLT x39 = x35 / sqrt(1 - pow(x28, 2) * pow(tilt_0, 2) / x34);
	out[0] = -1 - x38;
	out[1] = -x39 - x38 * x39;
	out[2] = pow(atan2(x28, x36), 2);
	out[3] = x38;
	out[4] = -cos(x37);
	out[5] = 0;
	out[6] = 0;
}

/** Applying function <function reproject_axis_y at 0x7effc26e3440> */
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
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = pow(lh_qk, 2);
	const GEN_FLT x3 = x1 + x2;
	const GEN_FLT x4 = 2 * sqrt(x0 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = pow(obj_qj, 2);
	const GEN_FLT x6 = pow(obj_qi, 2);
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = x6 + x7;
	const GEN_FLT x9 = 2 * sqrt(x5 + x8 + pow(obj_qw, 2));
	const GEN_FLT x10 = obj_qw * obj_qi;
	const GEN_FLT x11 = obj_qk * obj_qj;
	const GEN_FLT x12 = x9 * sensor_y;
	const GEN_FLT x13 = obj_qw * obj_qj;
	const GEN_FLT x14 = obj_qk * obj_qi;
	const GEN_FLT x15 = x9 * sensor_x;
	const GEN_FLT x16 = obj_pz + (1 - (x5 + x6) * x9) * sensor_z + (x10 + x11) * x12 + (-x13 + x14) * x15;
	const GEN_FLT x17 = lh_qw * lh_qi;
	const GEN_FLT x18 = lh_qk * lh_qj;
	const GEN_FLT x19 = x9 * sensor_z;
	const GEN_FLT x20 = obj_qw * obj_qk;
	const GEN_FLT x21 = obj_qj * obj_qi;
	const GEN_FLT x22 = obj_py + (1 - x8 * x9) * sensor_y + (-x10 + x11) * x19 + (x20 + x21) * x15;
	const GEN_FLT x23 = x4 * x22;
	const GEN_FLT x24 = lh_qw * lh_qj;
	const GEN_FLT x25 = lh_qk * lh_qi;
	const GEN_FLT x26 = obj_px + (1 - (x5 + x7) * x9) * sensor_x + (x13 + x14) * x19 + (-x20 + x21) * x12;
	const GEN_FLT x27 = x4 * x26;
	const GEN_FLT x28 = lh_pz + x16 * (1 - (x0 + x1) * x4) + (x17 + x18) * x23 + (-x24 + x25) * x27;
	const GEN_FLT x29 = x4 * x16;
	const GEN_FLT x30 = lh_qw * lh_qk;
	const GEN_FLT x31 = lh_qj * lh_qi;
	const GEN_FLT x32 = lh_py + x22 * (1 - x4 * x3) + (-x17 + x18) * x29 + (x30 + x31) * x27;
	const GEN_FLT x33 = lh_px + x26 * (1 - (x0 + x2) * x4) + (x24 + x25) * x29 + (-x30 + x31) * x23;
	const GEN_FLT x34 = -x28;
	const GEN_FLT x35 = -phase_1 - asin(x33 * tilt_1 / sqrt(pow(x28, 2) + pow(x32, 2))) - atan2(-x32, x34);
	out[0] = x35 - cos(1.5707963267949 + gibPhase_1 + x35) * gibMag_1 + pow(atan2(x33, x34), 2) * curve_1;
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
	const GEN_FLT x0 = pow(lh_qk, 2);
	const GEN_FLT x1 = pow(lh_qj, 2);
	const GEN_FLT x2 = pow(lh_qi, 2);
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt(x1 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - (x0 + x1) * x5;
	const GEN_FLT x7 = 1 - (x1 + x2) * x5;
	const GEN_FLT x8 = pow(obj_qj, 2);
	const GEN_FLT x9 = pow(obj_qi, 2);
	const GEN_FLT x10 = x8 + x9;
	const GEN_FLT x11 = pow(obj_qk, 2);
	const GEN_FLT x12 = x11 + x9;
	const GEN_FLT x13 = sqrt(x12 + x8 + pow(obj_qw, 2));
	const GEN_FLT x14 = 2 * x13;
	const GEN_FLT x15 = obj_qw * obj_qi;
	const GEN_FLT x16 = obj_qk * obj_qj;
	const GEN_FLT x17 = x15 + x16;
	const GEN_FLT x18 = x14 * sensor_y;
	const GEN_FLT x19 = obj_qw * obj_qj;
	const GEN_FLT x20 = obj_qk * obj_qi;
	const GEN_FLT x21 = -x19 + x20;
	const GEN_FLT x22 = x14 * sensor_x;
	const GEN_FLT x23 = obj_pz + x18 * x17 + x22 * x21 + (1 - x14 * x10) * sensor_z;
	const GEN_FLT x24 = -x15 + x16;
	const GEN_FLT x25 = x14 * sensor_z;
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x26 + x27;
	const GEN_FLT x29 = obj_py + x22 * x28 + x24 * x25 + (1 - x14 * x12) * sensor_y;
	const GEN_FLT x30 = lh_qw * lh_qi;
	const GEN_FLT x31 = lh_qk * lh_qj;
	const GEN_FLT x32 = x30 + x31;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = x19 + x20;
	const GEN_FLT x35 = -x26 + x27;
	const GEN_FLT x36 = x11 + x8;
	const GEN_FLT x37 = obj_px + x34 * x25 + x35 * x18 + (1 - x36 * x14) * sensor_x;
	const GEN_FLT x38 = lh_qw * lh_qj;
	const GEN_FLT x39 = lh_qk * lh_qi;
	const GEN_FLT x40 = -x38 + x39;
	const GEN_FLT x41 = x5 * x40;
	const GEN_FLT x42 = lh_pz + x33 * x29 + x41 * x37 + x7 * x23;
	const GEN_FLT x43 = pow(x42, -1);
	const GEN_FLT x44 = pow(x42, 2);
	const GEN_FLT x45 = pow(x44, -1);
	const GEN_FLT x46 = x38 + x39;
	const GEN_FLT x47 = x5 * x23;
	const GEN_FLT x48 = lh_qw * lh_qk;
	const GEN_FLT x49 = lh_qj * lh_qi;
	const GEN_FLT x50 = (-x48 + x49) * x5;
	const GEN_FLT x51 = lh_px + x46 * x47 + x50 * x29 + x6 * x37;
	const GEN_FLT x52 = x51 * x45;
	const GEN_FLT x53 = -x42;
	const GEN_FLT x54 = pow(x51, 2);
	const GEN_FLT x55 = 2 * x44 * atan2(x51, x53) * curve_1 / (x44 + x54);
	const GEN_FLT x56 = x48 + x49;
	const GEN_FLT x57 = x5 * x56;
	const GEN_FLT x58 = -x30 + x31;
	const GEN_FLT x59 = 1 - x3 * x5;
	const GEN_FLT x60 = lh_py + x57 * x37 + x58 * x47 + x59 * x29;
	const GEN_FLT x61 = 2 * x60;
	const GEN_FLT x62 = x4 * x61 * x45;
	const GEN_FLT x63 = x44 + pow(x60, 2);
	const GEN_FLT x64 = pow(x63, -1);
	const GEN_FLT x65 = x64 * x44;
	const GEN_FLT x66 = pow(1 - x64 * x54 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x67 = tilt_1 / sqrt(x63);
	const GEN_FLT x68 = 4 * x4;
	const GEN_FLT x69 = x60 * x68;
	const GEN_FLT x70 = x68 * x42;
	const GEN_FLT x71 = (1.0 / 2.0) * x51 * tilt_1 / pow(x63, 3.0 / 2.0);
	const GEN_FLT x72 = -x66 * (x6 * x67 - (x69 * x56 + x70 * x40) * x71) - (x57 * x43 - x62 * x40) * x65;
	const GEN_FLT x73 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x67 * x51) - atan2(-x60, x53)) * gibMag_1;
	const GEN_FLT x74 = -x66 * (x67 * x50 - (x61 * x59 + x70 * x32) * x71) - (x59 * x43 - x62 * x32) * x65;
	const GEN_FLT x75 = x5 * x43;
	const GEN_FLT x76 = x7 * x45;
	const GEN_FLT x77 = x5 * x46;
	const GEN_FLT x78 = 2 * x42;
	const GEN_FLT x79 = -x66 * (-x71 * (x69 * x58 + x7 * x78) + x77 * x67) - (x75 * x58 - x76 * x60) * x65;
	const GEN_FLT x80 = 2 / x13;
	const GEN_FLT x81 = x80 * x36;
	const GEN_FLT x82 = x81 * sensor_x;
	const GEN_FLT x83 = x35 * sensor_y;
	const GEN_FLT x84 = x80 * x83;
	const GEN_FLT x85 = x18 * obj_qk;
	const GEN_FLT x86 = x80 * obj_qw;
	const GEN_FLT x87 = x34 * sensor_z;
	const GEN_FLT x88 = x25 * obj_qj;
	const GEN_FLT x89 = -x85 + x88 - x82 * obj_qw + x84 * obj_qw + x86 * x87;
	const GEN_FLT x90 = x28 * sensor_x;
	const GEN_FLT x91 = x80 * x12;
	const GEN_FLT x92 = x91 * sensor_y;
	const GEN_FLT x93 = x22 * obj_qk;
	const GEN_FLT x94 = x24 * sensor_z;
	const GEN_FLT x95 = x25 * obj_qi;
	const GEN_FLT x96 = x93 - x95 + x86 * x90 + x86 * x94 - x92 * obj_qw;
	const GEN_FLT x97 = x22 * obj_qj;
	const GEN_FLT x98 = x21 * sensor_x;
	const GEN_FLT x99 = x17 * sensor_y;
	const GEN_FLT x100 = x80 * x99;
	const GEN_FLT x101 = x18 * obj_qi;
	const GEN_FLT x102 = x80 * x10;
	const GEN_FLT x103 = x102 * sensor_z;
	const GEN_FLT x104 = x101 - x97 + x100 * obj_qw - x103 * obj_qw + x86 * x98;
	const GEN_FLT x105 = x5 * x104;
	const GEN_FLT x106 = x46 * x105 + x50 * x96 + x6 * x89;
	const GEN_FLT x107 = x7 * x104 + x89 * x41 + x96 * x33;
	const GEN_FLT x108 = x58 * x105 + x59 * x96 + x89 * x57;
	const GEN_FLT x109 = x60 * x45;
	const GEN_FLT x110 = -x65 * (-x109 * x107 + x43 * x108) - x66 * (x67 * x106 - (x61 * x108 + x78 * x107) * x71);
	const GEN_FLT x111 = x80 * obj_qi;
	const GEN_FLT x112 = x18 * obj_qj;
	const GEN_FLT x113 = x111 * sensor_z;
	const GEN_FLT x114 = x25 * obj_qk;
	const GEN_FLT x115 = x112 + x114 + x34 * x113 - x82 * obj_qi + x83 * x111;
	const GEN_FLT x116 = x111 * sensor_x;
	const GEN_FLT x117 = 4 * x13;
	const GEN_FLT x118 = -x117 * obj_qi;
	const GEN_FLT x119 = x25 * obj_qw;
	const GEN_FLT x120 = -x119 + x97 + x24 * x113 + x28 * x116 + (x118 - x91 * obj_qi) * sensor_y;
	const GEN_FLT x121 = x18 * obj_qw;
	const GEN_FLT x122 = x121 + x93 + x21 * x116 + x99 * x111 + (x118 - x102 * obj_qi) * sensor_z;
	const GEN_FLT x123 = x50 * x120 + x6 * x115 + x77 * x122;
	const GEN_FLT x124 = x33 * x120 + x41 * x115 + x7 * x122;
	const GEN_FLT x125 = x5 * x58;
	const GEN_FLT x126 = x122 * x125 + x57 * x115 + x59 * x120;
	const GEN_FLT x127 = -x65 * (-x109 * x124 + x43 * x126) - x66 * (x67 * x123 - (x61 * x126 + x78 * x124) * x71);
	const GEN_FLT x128 = -x117 * obj_qj;
	const GEN_FLT x129 = x80 * obj_qj;
	const GEN_FLT x130 = x101 + x119 + x84 * obj_qj + x87 * x129 + (x128 - x81 * obj_qj) * sensor_x;
	const GEN_FLT x131 = x22 * obj_qi;
	const GEN_FLT x132 = x114 + x131 + x90 * x129 - x92 * obj_qj + x94 * x129;
	const GEN_FLT x133 = x22 * obj_qw;
	const GEN_FLT x134 = -x133 + x85 + x100 * obj_qj + x98 * x129 + (x128 - x102 * obj_qj) * sensor_z;
	const GEN_FLT x135 = x50 * x132 + x6 * x130 + x77 * x134;
	const GEN_FLT x136 = x33 * x132 + x41 * x130 + x7 * x134;
	const GEN_FLT x137 = x45 * x136;
	const GEN_FLT x138 = x125 * x134 + x57 * x130 + x59 * x132;
	const GEN_FLT x139 = -x66 * (x67 * x135 - (x61 * x138 + x78 * x136) * x71) - (x43 * x138 - x60 * x137) * x65;
	const GEN_FLT x140 = -x117 * obj_qk;
	const GEN_FLT x141 = x80 * obj_qk;
	const GEN_FLT x142 = -x121 + x95 + x84 * obj_qk + x87 * x141 + (x140 - x81 * obj_qk) * sensor_x;
	const GEN_FLT x143 = x133 + x88 + x90 * x141 + x94 * x141 + (x140 - x91 * obj_qk) * sensor_y;
	const GEN_FLT x144 = x112 + x131 + x100 * obj_qk - x103 * obj_qk + x98 * x141;
	const GEN_FLT x145 = x5 * x144;
	const GEN_FLT x146 = x46 * x145 + x50 * x143 + x6 * x142;
	const GEN_FLT x147 = x33 * x143 + x41 * x142 + x7 * x144;
	const GEN_FLT x148 = x45 * x147;
	const GEN_FLT x149 = x57 * x142 + x58 * x145 + x59 * x143;
	const GEN_FLT x150 = -x66 * (x67 * x146 - (x61 * x149 + x78 * x147) * x71) - (x43 * x149 - x60 * x148) * x65;
	out[0] = x72 + x55 * (x52 * x41 - x6 * x43) + x73 * x72;
	out[1] = x74 + x73 * x74 + (-x50 * x43 + x52 * x33) * x55;
	out[2] = x79 + x73 * x79 + (-x75 * x46 + x76 * x51) * x55;
	out[3] = x110 + x73 * x110 + (-x43 * x106 + x52 * x107) * x55;
	out[4] = x127 + x73 * x127 + (-x43 * x123 + x52 * x124) * x55;
	out[5] = x139 + x73 * x139 + (-x43 * x135 + x51 * x137) * x55;
	out[6] = x150 + x73 * x150 + (-x43 * x146 + x51 * x148) * x55;
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
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = pow(lh_qk, 2);
	const GEN_FLT x3 = x0 + x2;
	const GEN_FLT x4 = sqrt(x1 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = 1 - (x0 + x1) * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = x7 + x8;
	const GEN_FLT x10 = pow(obj_qk, 2);
	const GEN_FLT x11 = sqrt(x10 + x9 + pow(obj_qw, 2));
	const GEN_FLT x12 = 2 * x11;
	const GEN_FLT x13 = 1 - x9 * x12;
	const GEN_FLT x14 = obj_qw * obj_qi;
	const GEN_FLT x15 = obj_qk * obj_qj;
	const GEN_FLT x16 = x14 + x15;
	const GEN_FLT x17 = x12 * sensor_y;
	const GEN_FLT x18 = obj_qw * obj_qj;
	const GEN_FLT x19 = obj_qk * obj_qi;
	const GEN_FLT x20 = -x18 + x19;
	const GEN_FLT x21 = x12 * sensor_x;
	const GEN_FLT x22 = obj_pz + x13 * sensor_z + x17 * x16 + x20 * x21;
	const GEN_FLT x23 = -x14 + x15;
	const GEN_FLT x24 = x23 * x12;
	const GEN_FLT x25 = 1 - x12 * (x10 + x8);
	const GEN_FLT x26 = obj_qw * obj_qk;
	const GEN_FLT x27 = obj_qj * obj_qi;
	const GEN_FLT x28 = x26 + x27;
	const GEN_FLT x29 = obj_py + x21 * x28 + x24 * sensor_z + x25 * sensor_y;
	const GEN_FLT x30 = lh_qw * lh_qi;
	const GEN_FLT x31 = lh_qk * lh_qj;
	const GEN_FLT x32 = x30 + x31;
	const GEN_FLT x33 = x5 * x32;
	const GEN_FLT x34 = x18 + x19;
	const GEN_FLT x35 = x34 * x12;
	const GEN_FLT x36 = -x26 + x27;
	const GEN_FLT x37 = 1 - x12 * (x10 + x7);
	const GEN_FLT x38 = obj_px + x35 * sensor_z + x36 * x17 + x37 * sensor_x;
	const GEN_FLT x39 = lh_qw * lh_qj;
	const GEN_FLT x40 = lh_qk * lh_qi;
	const GEN_FLT x41 = -x39 + x40;
	const GEN_FLT x42 = x5 * x41;
	const GEN_FLT x43 = lh_pz + x33 * x29 + x42 * x38 + x6 * x22;
	const GEN_FLT x44 = pow(x43, -1);
	const GEN_FLT x45 = 1 - x3 * x5;
	const GEN_FLT x46 = lh_qw * lh_qk;
	const GEN_FLT x47 = lh_qj * lh_qi;
	const GEN_FLT x48 = -x46 + x47;
	const GEN_FLT x49 = 4 * x4 * x11;
	const GEN_FLT x50 = x49 * x28;
	const GEN_FLT x51 = x39 + x40;
	const GEN_FLT x52 = x51 * x49;
	const GEN_FLT x53 = x45 * x37 + x50 * x48 + x52 * x20;
	const GEN_FLT x54 = x6 * x12;
	const GEN_FLT x55 = x42 * x37 + x50 * x32 + x54 * x20;
	const GEN_FLT x56 = pow(x43, 2);
	const GEN_FLT x57 = pow(x56, -1);
	const GEN_FLT x58 = x5 * x22;
	const GEN_FLT x59 = x5 * x48;
	const GEN_FLT x60 = lh_px + x45 * x38 + x51 * x58 + x59 * x29;
	const GEN_FLT x61 = x60 * x57;
	const GEN_FLT x62 = -x43;
	const GEN_FLT x63 = pow(x60, 2);
	const GEN_FLT x64 = 2 * x56 * atan2(x60, x62) * curve_1 / (x56 + x63);
	const GEN_FLT x65 = x46 + x47;
	const GEN_FLT x66 = x5 * x65;
	const GEN_FLT x67 = 1 - (x1 + x2) * x5;
	const GEN_FLT x68 = -x30 + x31;
	const GEN_FLT x69 = x68 * x49;
	const GEN_FLT x70 = x66 * x37 + x69 * x20 + x67 * x28 * x12;
	const GEN_FLT x71 = lh_py + x66 * x38 + x67 * x29 + x68 * x58;
	const GEN_FLT x72 = x71 * x57;
	const GEN_FLT x73 = x56 + pow(x71, 2);
	const GEN_FLT x74 = pow(x73, -1);
	const GEN_FLT x75 = x74 * x56;
	const GEN_FLT x76 = pow(1 - x74 * x63 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x77 = tilt_1 / sqrt(x73);
	const GEN_FLT x78 = 2 * x71;
	const GEN_FLT x79 = 2 * x43;
	const GEN_FLT x80 = (1.0 / 2.0) * x60 * tilt_1 / pow(x73, 3.0 / 2.0);
	const GEN_FLT x81 = -x76 * (x77 * x53 - (x70 * x78 + x79 * x55) * x80) - (x70 * x44 - x72 * x55) * x75;
	const GEN_FLT x82 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x77 * x60) - atan2(-x71, x62)) * gibMag_1;
	const GEN_FLT x83 = x52 * x16 + x59 * x25 + x45 * x36 * x12;
	const GEN_FLT x84 = x41 * x49;
	const GEN_FLT x85 = x33 * x25 + x54 * x16 + x84 * x36;
	const GEN_FLT x86 = x65 * x49;
	const GEN_FLT x87 = x67 * x25 + x69 * x16 + x86 * x36;
	const GEN_FLT x88 = -x76 * (x83 * x77 - (x85 * x79 + x87 * x78) * x80) - (-x85 * x72 + x87 * x44) * x75;
	const GEN_FLT x89 = x49 * x23;
	const GEN_FLT x90 = x5 * x13;
	const GEN_FLT x91 = x45 * x35 + x51 * x90 + x89 * x48;
	const GEN_FLT x92 = x6 * x13 + x84 * x34 + x89 * x32;
	const GEN_FLT x93 = x67 * x24 + x68 * x90 + x86 * x34;
	const GEN_FLT x94 = -x76 * (x77 * x91 - (x78 * x93 + x79 * x92) * x80) - (-x72 * x92 + x93 * x44) * x75;
	out[0] = x81 + x81 * x82 + (-x53 * x44 + x61 * x55) * x64;
	out[1] = x88 + x82 * x88 + (-x83 * x44 + x85 * x61) * x64;
	out[2] = x94 + x82 * x94 + (x61 * x92 - x91 * x44) * x64;
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
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = x0 + x1;
	const GEN_FLT x3 = pow(lh_qk, 2);
	const GEN_FLT x4 = x0 + x3;
	const GEN_FLT x5 = sqrt(x1 + x4 + pow(lh_qw, 2));
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = pow(obj_qj, 2);
	const GEN_FLT x8 = pow(obj_qi, 2);
	const GEN_FLT x9 = x7 + x8;
	const GEN_FLT x10 = pow(obj_qk, 2);
	const GEN_FLT x11 = 2 * sqrt(x10 + x9 + pow(obj_qw, 2));
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	const GEN_FLT x14 = x11 * sensor_y;
	const GEN_FLT x15 = obj_qw * obj_qj;
	const GEN_FLT x16 = obj_qk * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 = obj_pz + (1 - x9 * x11) * sensor_z + (x12 + x13) * x14 + (-x15 + x16) * x17;
	const GEN_FLT x19 = lh_qw * lh_qi;
	const GEN_FLT x20 = lh_qk * lh_qj;
	const GEN_FLT x21 = x19 + x20;
	const GEN_FLT x22 = x11 * sensor_z;
	const GEN_FLT x23 = obj_qw * obj_qk;
	const GEN_FLT x24 = obj_qj * obj_qi;
	const GEN_FLT x25 = obj_py + (1 - x11 * (x10 + x8)) * sensor_y + (-x12 + x13) * x22 + (x23 + x24) * x17;
	const GEN_FLT x26 = x6 * x25;
	const GEN_FLT x27 = lh_qw * lh_qj;
	const GEN_FLT x28 = lh_qk * lh_qi;
	const GEN_FLT x29 = -x27 + x28;
	const GEN_FLT x30 = obj_px + (1 - x11 * (x10 + x7)) * sensor_x + (x15 + x16) * x22 + (-x23 + x24) * x14;
	const GEN_FLT x31 = x6 * x30;
	const GEN_FLT x32 = lh_pz + x18 * (1 - x2 * x6) + x21 * x26 + x31 * x29;
	const GEN_FLT x33 = pow(x32, 2);
	const GEN_FLT x34 = -x19 + x20;
	const GEN_FLT x35 = x6 * x18;
	const GEN_FLT x36 = x1 + x3;
	const GEN_FLT x37 = lh_qw * lh_qk;
	const GEN_FLT x38 = lh_qj * lh_qi;
	const GEN_FLT x39 = x37 + x38;
	const GEN_FLT x40 = lh_py + x25 * (1 - x6 * x36) + x31 * x39 + x34 * x35;
	const GEN_FLT x41 = x33 + pow(x40, 2);
	const GEN_FLT x42 = pow(x41, -1);
	const GEN_FLT x43 = x27 + x28;
	const GEN_FLT x44 = -x37 + x38;
	const GEN_FLT x45 = lh_px + x30 * (1 - x4 * x6) + x43 * x35 + x44 * x26;
	const GEN_FLT x46 = pow(x45, 2);
	const GEN_FLT x47 = pow(1 - x42 * x46 * pow(tilt_1, 2), -1.0 / 2.0);
	const GEN_FLT x48 = tilt_1 / sqrt(x41);
	const GEN_FLT x49 = x47 * x48;
	const GEN_FLT x50 = 2 * x32;
	const GEN_FLT x51 = -x32;
	const GEN_FLT x52 = atan2(x45, x51) * curve_1 / (x33 + x46);
	const GEN_FLT x53 = sin(1.5707963267949 + gibPhase_1 - phase_1 - asin(x45 * x48) - atan2(-x40, x51)) * gibMag_1;
	const GEN_FLT x54 = x45 * tilt_1 / pow(x41, 3.0 / 2.0);
	const GEN_FLT x55 = x54 * x47;
	const GEN_FLT x56 = -x42 * x32 + x55 * x40;
	const GEN_FLT x57 = 2 * x52;
	const GEN_FLT x58 = x40 * x42 + x55 * x32;
	const GEN_FLT x59 = pow(x32, -1);
	const GEN_FLT x60 = 2 / x5;
	const GEN_FLT x61 = x4 * x60;
	const GEN_FLT x62 = x61 * x30;
	const GEN_FLT x63 = x44 * x25;
	const GEN_FLT x64 = x60 * x63;
	const GEN_FLT x65 = x26 * lh_qk;
	const GEN_FLT x66 = x60 * x18;
	const GEN_FLT x67 = x66 * x43;
	const GEN_FLT x68 = x35 * lh_qj;
	const GEN_FLT x69 = -x65 + x68 - x62 * lh_qw + x64 * lh_qw + x67 * lh_qw;
	const GEN_FLT x70 = x30 * x29;
	const GEN_FLT x71 = x70 * x60;
	const GEN_FLT x72 = x25 * x21;
	const GEN_FLT x73 = x72 * x60;
	const GEN_FLT x74 = x26 * lh_qi;
	const GEN_FLT x75 = x31 * lh_qj;
	const GEN_FLT x76 = x2 * x60;
	const GEN_FLT x77 = x76 * x18;
	const GEN_FLT x78 = x74 - x75 + x71 * lh_qw + x73 * lh_qw - x77 * lh_qw;
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
	const GEN_FLT x89 = x84 - x88 + x83 * lh_qw - x86 * lh_qw + x87 * lh_qw;
	const GEN_FLT x90 = x79 * x40;
	const GEN_FLT x91 = x42 * x33;
	const GEN_FLT x92 = 2 * x40;
	const GEN_FLT x93 = (1.0 / 2.0) * x54;
	const GEN_FLT x94 = -x47 * (x69 * x48 - (x78 * x50 + x89 * x92) * x93) - (-x78 * x90 + x89 * x59) * x91;
	const GEN_FLT x95 = x26 * lh_qj;
	const GEN_FLT x96 = x35 * lh_qk;
	const GEN_FLT x97 = x95 + x96 - x62 * lh_qi + x64 * lh_qi + x67 * lh_qi;
	const GEN_FLT x98 = x26 * lh_qw;
	const GEN_FLT x99 = 4 * x5;
	const GEN_FLT x100 = -x99 * lh_qi;
	const GEN_FLT x101 = x84 + x98 + x18 * (x100 - x76 * lh_qi) + x71 * lh_qi + x73 * lh_qi;
	const GEN_FLT x102 = x79 * x101;
	const GEN_FLT x103 = x35 * lh_qw;
	const GEN_FLT x104 = -x103 + x75 + x25 * (x100 - x85 * lh_qi) + x83 * lh_qi + x87 * lh_qi;
	const GEN_FLT x105 = -x47 * (x97 * x48 - (x50 * x101 + x92 * x104) * x93) - (-x40 * x102 + x59 * x104) * x91;
	const GEN_FLT x106 = -x99 * lh_qj;
	const GEN_FLT x107 = x60 * lh_qj;
	const GEN_FLT x108 = x103 + x74 + x30 * (x106 - x61 * lh_qj) + x63 * x107 + x67 * lh_qj;
	const GEN_FLT x109 = x31 * lh_qw;
	const GEN_FLT x110 = -x109 + x65 + x18 * (x106 - x76 * lh_qj) + x70 * x107 + x72 * x107;
	const GEN_FLT x111 = x31 * lh_qi;
	const GEN_FLT x112 = x111 + x96 + x83 * lh_qj - x86 * lh_qj + x87 * lh_qj;
	const GEN_FLT x113 = -x47 * (x48 * x108 - (x50 * x110 + x92 * x112) * x93) - (x59 * x112 - x90 * x110) * x91;
	const GEN_FLT x114 = -x99 * lh_qk;
	const GEN_FLT x115 = x60 * lh_qk;
	const GEN_FLT x116 = x88 - x98 + x30 * (x114 - x61 * lh_qk) + x63 * x115 + x67 * lh_qk;
	const GEN_FLT x117 = x111 + x95 + x70 * x115 + x72 * x115 - x77 * lh_qk;
	const GEN_FLT x118 = x109 + x68 + x25 * (x114 - x85 * lh_qk) + x82 * x115 + x87 * lh_qk;
	const GEN_FLT x119 = -x47 * (x48 * x116 - (x50 * x117 + x92 * x118) * x93) - (x59 * x118 - x90 * x117) * x91;
	out[0] = -x49 - x50 * x52 - x53 * x49;
	out[1] = x56 + x53 * x56;
	out[2] = x58 + x53 * x58 + x57 * x45;
	out[3] = x94 + x53 * x94 + (-x69 * x59 + x80 * x78) * x81;
	out[4] = x105 + x53 * x105 + x81 * (x45 * x102 - x59 * x97);
	out[5] = x113 + x53 * x113 + (-x59 * x108 + x80 * x110) * x81;
	out[6] = x119 + x53 * x119 + (-x59 * x116 + x80 * x117) * x81;
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
	const GEN_FLT x0 = pow(lh_qj, 2);
	const GEN_FLT x1 = pow(lh_qi, 2);
	const GEN_FLT x2 = pow(lh_qk, 2);
	const GEN_FLT x3 = x1 + x2;
	const GEN_FLT x4 = 2 * sqrt(x0 + x3 + pow(lh_qw, 2));
	const GEN_FLT x5 = pow(obj_qj, 2);
	const GEN_FLT x6 = pow(obj_qi, 2);
	const GEN_FLT x7 = pow(obj_qk, 2);
	const GEN_FLT x8 = x6 + x7;
	const GEN_FLT x9 = 2 * sqrt(x5 + x8 + pow(obj_qw, 2));
	const GEN_FLT x10 = obj_qw * obj_qi;
	const GEN_FLT x11 = obj_qk * obj_qj;
	const GEN_FLT x12 = x9 * sensor_y;
	const GEN_FLT x13 = obj_qw * obj_qj;
	const GEN_FLT x14 = obj_qk * obj_qi;
	const GEN_FLT x15 = x9 * sensor_x;
	const GEN_FLT x16 = obj_pz + (1 - (x5 + x6) * x9) * sensor_z + (x10 + x11) * x12 + (-x13 + x14) * x15;
	const GEN_FLT x17 = lh_qw * lh_qi;
	const GEN_FLT x18 = lh_qk * lh_qj;
	const GEN_FLT x19 = x9 * sensor_z;
	const GEN_FLT x20 = obj_qw * obj_qk;
	const GEN_FLT x21 = obj_qj * obj_qi;
	const GEN_FLT x22 = obj_py + (1 - x8 * x9) * sensor_y + (-x10 + x11) * x19 + (x20 + x21) * x15;
	const GEN_FLT x23 = x4 * x22;
	const GEN_FLT x24 = lh_qw * lh_qj;
	const GEN_FLT x25 = lh_qk * lh_qi;
	const GEN_FLT x26 = obj_px + (1 - (x5 + x7) * x9) * sensor_x + (x13 + x14) * x19 + (-x20 + x21) * x12;
	const GEN_FLT x27 = x4 * x26;
	const GEN_FLT x28 = lh_pz + x16 * (1 - (x0 + x1) * x4) + (x17 + x18) * x23 + (-x24 + x25) * x27;
	const GEN_FLT x29 = x4 * x16;
	const GEN_FLT x30 = lh_qw * lh_qk;
	const GEN_FLT x31 = lh_qj * lh_qi;
	const GEN_FLT x32 = lh_py + x22 * (1 - x4 * x3) + (-x17 + x18) * x29 + (x30 + x31) * x27;
	const GEN_FLT x33 = pow(x28, 2) + pow(x32, 2);
	const GEN_FLT x34 = lh_px + x26 * (1 - (x0 + x2) * x4) + (x24 + x25) * x29 + (-x30 + x31) * x23;
	const GEN_FLT x35 = x34 / sqrt(x33);
	const GEN_FLT x36 = -x28;
	const GEN_FLT x37 = 1.5707963267949 + gibPhase_1 - phase_1 - asin(x35 * tilt_1) - atan2(-x32, x36);
	const GEN_FLT x38 = sin(x37) * gibMag_1;
	const GEN_FLT x39 = x35 / sqrt(1 - pow(x34, 2) * pow(tilt_1, 2) / x33);
	out[0] = -1 - x38;
	out[1] = -x39 - x38 * x39;
	out[2] = pow(atan2(x34, x36), 2);
	out[3] = x38;
	out[4] = -cos(x37);
	out[5] = 0;
	out[6] = 0;
}
