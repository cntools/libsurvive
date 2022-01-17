#include "survive_reproject.generated.h"
static inline void gen_apply_axisangle_pose_to_pt(FLT *out, const LinmathAxisAnglePose *obj_p_axisangle,
												  const FLT *sensor_pt) {
	const GEN_FLT obj_px = (*obj_p_axisangle).Pos[0];
	const GEN_FLT obj_py = (*obj_p_axisangle).Pos[1];
	const GEN_FLT obj_pz = (*obj_p_axisangle).Pos[2];
	const GEN_FLT obj_qi = (*obj_p_axisangle).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p_axisangle).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p_axisangle).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = obj_qi * obj_qi;
	const GEN_FLT x1 = obj_qk * obj_qk;
	const GEN_FLT x2 = obj_qj * obj_qj;
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = (1. / x3) * (1 + (-1 * x5));
	const GEN_FLT x7 = (1. / x4) * sin(x4);
	const GEN_FLT x8 = x7 * obj_qj;
	const GEN_FLT x9 = x6 * obj_qi;
	const GEN_FLT x10 = x9 * obj_qk;
	const GEN_FLT x11 = x7 * obj_qk;
	const GEN_FLT x12 = x9 * obj_qj;
	const GEN_FLT x13 = x7 * obj_qi;
	const GEN_FLT x14 = x6 * obj_qk * obj_qj;
	out[0] = ((x12 + (-1 * x11)) * sensor_y) + ((x10 + x8) * sensor_z) + ((x5 + (x0 * x6)) * sensor_x) + obj_px;
	out[1] = ((x5 + (x2 * x6)) * sensor_y) + ((x12 + x11) * sensor_x) + obj_py + ((x14 + (-1 * x13)) * sensor_z);
	out[2] = ((x14 + x13) * sensor_y) + ((x10 + (-1 * x8)) * sensor_x) + obj_pz + ((x5 + (x1 * x6)) * sensor_z);
}

// Jacobian of apply_axisangle_pose_to_pt wrt [obj_px, obj_py, obj_pz, obj_qi, obj_qj, obj_qk]
static inline void gen_apply_axisangle_pose_to_pt_jac_obj_p_axisangle(FLT *out,
																	  const LinmathAxisAnglePose *obj_p_axisangle,
																	  const FLT *sensor_pt) {
	const GEN_FLT obj_px = (*obj_p_axisangle).Pos[0];
	const GEN_FLT obj_py = (*obj_p_axisangle).Pos[1];
	const GEN_FLT obj_pz = (*obj_p_axisangle).Pos[2];
	const GEN_FLT obj_qi = (*obj_p_axisangle).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p_axisangle).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p_axisangle).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = obj_qk * obj_qk;
	const GEN_FLT x1 = obj_qi * obj_qi;
	const GEN_FLT x2 = obj_qj * obj_qj;
	const GEN_FLT x3 = 1e-10 + x2 + x0 + x1;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = sin(x4);
	const GEN_FLT x6 = (1. / x4) * x5;
	const GEN_FLT x7 = -1 * x6 * obj_qi;
	const GEN_FLT x8 = obj_qi * obj_qi * obj_qi;
	const GEN_FLT x9 = cos(x4);
	const GEN_FLT x10 = 1 + (-1 * x9);
	const GEN_FLT x11 = 2 * (1. / (x3 * x3)) * x10;
	const GEN_FLT x12 = (1. / (x3 * sqrt(x3))) * x5;
	const GEN_FLT x13 = 1. / x3;
	const GEN_FLT x14 = x13 * x10;
	const GEN_FLT x15 = x14 * obj_qi;
	const GEN_FLT x16 = x14 * obj_qj;
	const GEN_FLT x17 = x1 * x11;
	const GEN_FLT x18 = x1 * x12;
	const GEN_FLT x19 = (x18 * obj_qj) + (-1 * x17 * obj_qj);
	const GEN_FLT x20 = x19 + x16;
	const GEN_FLT x21 = obj_qk * obj_qi;
	const GEN_FLT x22 = x21 * x12;
	const GEN_FLT x23 = x9 * x13;
	const GEN_FLT x24 = x23 * x21;
	const GEN_FLT x25 = (-1 * x24) + x22;
	const GEN_FLT x26 = x14 * obj_qk;
	const GEN_FLT x27 = (x18 * obj_qk) + (-1 * x17 * obj_qk);
	const GEN_FLT x28 = x27 + x26;
	const GEN_FLT x29 = obj_qj * obj_qi;
	const GEN_FLT x30 = x29 * x12;
	const GEN_FLT x31 = x23 * x29;
	const GEN_FLT x32 = x31 + (-1 * x30);
	const GEN_FLT x33 = x11 * obj_qi;
	const GEN_FLT x34 = x2 * x12;
	const GEN_FLT x35 = (x34 * obj_qi) + (-1 * x2 * x33);
	const GEN_FLT x36 = x35 + x15;
	const GEN_FLT x37 = obj_qk * obj_qj;
	const GEN_FLT x38 = x37 * x12;
	const GEN_FLT x39 = x37 * x23;
	const GEN_FLT x40 = (-1 * x39) + x38;
	const GEN_FLT x41 = -1 * x6 * obj_qj;
	const GEN_FLT x42 = x2 * x23;
	const GEN_FLT x43 = (x38 * obj_qi) + (-1 * x33 * x37);
	const GEN_FLT x44 = x43 + x6;
	const GEN_FLT x45 = x0 * x12;
	const GEN_FLT x46 = x0 * x23;
	const GEN_FLT x47 = x43 + (-1 * x6);
	const GEN_FLT x48 = -1 * x6 * obj_qk;
	const GEN_FLT x49 = x39 + (-1 * x38);
	const GEN_FLT x50 = (x45 * obj_qi) + (-1 * x0 * x33);
	const GEN_FLT x51 = x50 + x15;
	const GEN_FLT x52 = x1 * x23;
	const GEN_FLT x53 = x24 + (-1 * x22);
	const GEN_FLT x54 = obj_qj * obj_qj * obj_qj;
	const GEN_FLT x55 = (-1 * x31) + x30;
	const GEN_FLT x56 = (x34 * obj_qk) + (-1 * x2 * x11 * obj_qk);
	const GEN_FLT x57 = x56 + x26;
	const GEN_FLT x58 = (x45 * obj_qj) + (-1 * x0 * x11 * obj_qj);
	const GEN_FLT x59 = x58 + x16;
	const GEN_FLT x60 = obj_qk * obj_qk * obj_qk;
	out[0] = 1;
	out[1] = 0;
	out[2] = 0;
	out[3] = ((x32 + x28) * sensor_z) + (((2 * x15) + (x8 * x12) + x7 + (-1 * x8 * x11)) * sensor_x) +
			 ((x25 + x20) * sensor_y);
	out[4] = ((x44 + x42 + (-1 * x34)) * sensor_z) + ((x40 + x36) * sensor_y) + ((x19 + x41) * sensor_x);
	out[5] = ((x51 + x49) * sensor_z) + ((x47 + x45 + (-1 * x46)) * sensor_y) + ((x27 + x48) * sensor_x);
	out[6] = 0;
	out[7] = 1;
	out[8] = 0;
	out[9] = ((x53 + x20) * sensor_x) + ((x35 + x7) * sensor_y) + ((x47 + (-1 * x52) + x18) * sensor_z);
	out[10] = ((x49 + x36) * sensor_x) + (((2 * x16) + (x54 * x12) + x41 + (-1 * x54 * x11)) * sensor_y) +
			  ((x57 + x55) * sensor_z);
	out[11] = ((x44 + (-1 * x45) + x46) * sensor_x) + ((x56 + x48) * sensor_y) + ((x59 + x25) * sensor_z);
	out[12] = 0;
	out[13] = 0;
	out[14] = 1;
	out[15] = ((x50 + x7) * sensor_z) + ((x55 + x28) * sensor_x) + ((x44 + x52 + (-1 * x18)) * sensor_y);
	out[16] = ((x57 + x32) * sensor_y) + ((x47 + (-1 * x42) + x34) * sensor_x) + ((x58 + x41) * sensor_z);
	out[17] = ((x51 + x40) * sensor_x) + ((x59 + x53) * sensor_y) +
			  (((2 * x26) + (-1 * x60 * x11) + x48 + (x60 * x12)) * sensor_z);
}

// Jacobian of apply_axisangle_pose_to_pt wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_apply_axisangle_pose_to_pt_jac_sensor_pt(FLT *out, const LinmathAxisAnglePose *obj_p_axisangle,
																const FLT *sensor_pt) {
	const GEN_FLT obj_px = (*obj_p_axisangle).Pos[0];
	const GEN_FLT obj_py = (*obj_p_axisangle).Pos[1];
	const GEN_FLT obj_pz = (*obj_p_axisangle).Pos[2];
	const GEN_FLT obj_qi = (*obj_p_axisangle).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p_axisangle).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p_axisangle).AxisAngleRot[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = obj_qi * obj_qi;
	const GEN_FLT x1 = obj_qk * obj_qk;
	const GEN_FLT x2 = obj_qj * obj_qj;
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = (1. / x3) * (1 + (-1 * x5));
	const GEN_FLT x7 = (1. / x4) * sin(x4);
	const GEN_FLT x8 = x7 * obj_qk;
	const GEN_FLT x9 = x6 * obj_qi;
	const GEN_FLT x10 = x9 * obj_qj;
	const GEN_FLT x11 = x7 * obj_qj;
	const GEN_FLT x12 = x9 * obj_qk;
	const GEN_FLT x13 = x7 * obj_qi;
	const GEN_FLT x14 = x6 * obj_qk * obj_qj;
	out[0] = x5 + (x0 * x6);
	out[1] = x10 + (-1 * x8);
	out[2] = x12 + x11;
	out[3] = x10 + x8;
	out[4] = x5 + (x2 * x6);
	out[5] = x14 + (-1 * x13);
	out[6] = x12 + (-1 * x11);
	out[7] = x14 + x13;
	out[8] = x5 + (x1 * x6);
}

static inline void gen_apply_pose_to_pt(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt) {
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
	const GEN_FLT x0 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x1 = (-1 * obj_qj * sensor_x) + (obj_qw * sensor_z) + (obj_qi * sensor_y);
	const GEN_FLT x2 = (obj_qw * sensor_x) + (-1 * obj_qk * sensor_y) + (obj_qj * sensor_z);
	out[0] = (2 * ((x1 * obj_qj) + (-1 * x0 * obj_qk))) + obj_px + sensor_x;
	out[1] = (2 * ((x2 * obj_qk) + (-1 * x1 * obj_qi))) + obj_py + sensor_y;
	out[2] = (2 * ((x0 * obj_qi) + (-1 * x2 * obj_qj))) + obj_pz + sensor_z;
}

// Jacobian of apply_pose_to_pt wrt [obj_px, obj_py, obj_pz, obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_apply_pose_to_pt_jac_obj_p(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt) {
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
	const GEN_FLT x0 = 2 * sensor_y;
	const GEN_FLT x1 = x0 * obj_qk;
	const GEN_FLT x2 = 2 * obj_qj;
	const GEN_FLT x3 = x2 * sensor_z;
	const GEN_FLT x4 = 2 * obj_qk;
	const GEN_FLT x5 = x4 * sensor_z;
	const GEN_FLT x6 = x0 * obj_qj;
	const GEN_FLT x7 = 4 * sensor_x;
	const GEN_FLT x8 = 2 * obj_qw;
	const GEN_FLT x9 = x8 * sensor_z;
	const GEN_FLT x10 = x0 * obj_qi;
	const GEN_FLT x11 = 2 * obj_qi;
	const GEN_FLT x12 = x11 * sensor_z;
	const GEN_FLT x13 = x0 * obj_qw;
	const GEN_FLT x14 = x4 * sensor_x;
	const GEN_FLT x15 = x2 * sensor_x;
	const GEN_FLT x16 = 4 * obj_qi;
	const GEN_FLT x17 = x11 * sensor_x;
	const GEN_FLT x18 = x8 * sensor_x;
	out[0] = 1;
	out[1] = 0;
	out[2] = 0;
	out[3] = x3 + (-1 * x1);
	out[4] = x6 + x5;
	out[5] = (-1 * x7 * obj_qj) + x10 + x9;
	out[6] = (-1 * x13) + (-1 * x7 * obj_qk) + x12;
	out[7] = 0;
	out[8] = 1;
	out[9] = 0;
	out[10] = x14 + (-1 * x12);
	out[11] = (-1 * x16 * sensor_y) + x15 + (-1 * x9);
	out[12] = x5 + x17;
	out[13] = x3 + x18 + (-4 * obj_qk * sensor_y);
	out[14] = 0;
	out[15] = 0;
	out[16] = 1;
	out[17] = x10 + (-1 * x15);
	out[18] = x13 + x14 + (-1 * x16 * sensor_z);
	out[19] = (-4 * obj_qj * sensor_z) + (-1 * x18) + x1;
	out[20] = x17 + x6;
}

// Jacobian of apply_pose_to_pt wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_apply_pose_to_pt_jac_sensor_pt(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt) {
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
	const GEN_FLT x0 = -2 * (obj_qk * obj_qk);
	const GEN_FLT x1 = 1 + (-2 * (obj_qj * obj_qj));
	const GEN_FLT x2 = 2 * obj_qw;
	const GEN_FLT x3 = x2 * obj_qk;
	const GEN_FLT x4 = 2 * obj_qi;
	const GEN_FLT x5 = x4 * obj_qj;
	const GEN_FLT x6 = x4 * obj_qk;
	const GEN_FLT x7 = x2 * obj_qj;
	const GEN_FLT x8 = -2 * (obj_qi * obj_qi);
	const GEN_FLT x9 = x2 * obj_qi;
	const GEN_FLT x10 = 2 * obj_qk * obj_qj;
	out[0] = x1 + x0;
	out[1] = x5 + (-1 * x3);
	out[2] = x7 + x6;
	out[3] = x3 + x5;
	out[4] = 1 + x0 + x8;
	out[5] = x10 + (-1 * x9);
	out[6] = x6 + (-1 * x7);
	out[7] = x9 + x10;
	out[8] = x1 + x8;
}

static inline void gen_axisangle2quat(FLT *out, const FLT *axis_angle) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT x0 = aa_z * aa_z;
	const GEN_FLT x1 = aa_x * aa_x;
	const GEN_FLT x2 = aa_y * aa_y;
	const GEN_FLT x3 = 1e-10 + x2 + x0 + x1;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = 0.5 * x4;
	const GEN_FLT x6 = cos(x5);
	const GEN_FLT x7 = sin(x5);
	const GEN_FLT x8 = (1. / x3) * (x7 * x7);
	const GEN_FLT x9 = 1. / sqrt(1e-11 + (x6 * x6) + (x1 * x8) + (x0 * x8) + (x2 * x8));
	const GEN_FLT x10 = (1. / x4) * x7 * x9;
	out[0] = x6 * x9;
	out[1] = x10 * aa_x;
	out[2] = x10 * aa_y;
	out[3] = x10 * aa_z;
}

// Jacobian of axisangle2quat wrt [aa_x, aa_y, aa_z]
static inline void gen_axisangle2quat_jac_axis_angle(FLT *out, const FLT *axis_angle) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT x0 = aa_z * aa_z;
	const GEN_FLT x1 = aa_x * aa_x;
	const GEN_FLT x2 = aa_y * aa_y;
	const GEN_FLT x3 = 1e-10 + x2 + x0 + x1;
	const GEN_FLT x4 = 1. / x3;
	const GEN_FLT x5 = sqrt(x3);
	const GEN_FLT x6 = 0.5 * x5;
	const GEN_FLT x7 = sin(x6);
	const GEN_FLT x8 = x7 * x7;
	const GEN_FLT x9 = x4 * x8;
	const GEN_FLT x10 = 2 * x9;
	const GEN_FLT x11 = 2 * (1. / (x3 * x3)) * x8;
	const GEN_FLT x12 = x11 * aa_x;
	const GEN_FLT x13 = aa_x * aa_x * aa_x;
	const GEN_FLT x14 = 1. / (x3 * sqrt(x3));
	const GEN_FLT x15 = cos(x6);
	const GEN_FLT x16 = 1.0 * x7 * x15;
	const GEN_FLT x17 = x14 * x16;
	const GEN_FLT x18 = 1. / x5;
	const GEN_FLT x19 = x18 * x16;
	const GEN_FLT x20 = x17 * aa_x;
	const GEN_FLT x21 = (-1 * x2 * x12) + (-1 * x13 * x11) + (-1 * x0 * x12) + (x10 * aa_x) + (x2 * x20) +
						(-1 * x19 * aa_x) + (x0 * x20) + (x13 * x17);
	const GEN_FLT x22 = 1e-11 + (x1 * x9) + (x15 * x15) + (x0 * x9) + (x2 * x9);
	const GEN_FLT x23 = 1.0 / 2.0 * (1. / (x22 * sqrt(x22)));
	const GEN_FLT x24 = x23 * x15;
	const GEN_FLT x25 = 1. / sqrt(x22);
	const GEN_FLT x26 = x7 * x25;
	const GEN_FLT x27 = x26 * x18;
	const GEN_FLT x28 = 0.5 * x27;
	const GEN_FLT x29 = x1 * x11;
	const GEN_FLT x30 = x1 * x17;
	const GEN_FLT x31 = x0 * aa_y;
	const GEN_FLT x32 = aa_y * aa_y * aa_y;
	const GEN_FLT x33 = (x32 * x17) + (x30 * aa_y) + (x10 * aa_y) + (-1 * x29 * aa_y) + (-1 * x32 * x11) +
						(-1 * x19 * aa_y) + (x31 * x17) + (-1 * x31 * x11);
	const GEN_FLT x34 = aa_z * aa_z * aa_z;
	const GEN_FLT x35 = x2 * aa_z;
	const GEN_FLT x36 = (x35 * x17) + (-1 * x34 * x11) + (-1 * x29 * aa_z) + (-1 * x35 * x11) + (x10 * aa_z) +
						(-1 * x19 * aa_z) + (x30 * aa_z) + (x34 * x17);
	const GEN_FLT x37 = 0.5 * x4 * x25 * x15;
	const GEN_FLT x38 = x7 * x23 * x18;
	const GEN_FLT x39 = x38 * aa_x;
	const GEN_FLT x40 = x26 * x14;
	const GEN_FLT x41 = x40 * aa_y;
	const GEN_FLT x42 = x37 * aa_y;
	const GEN_FLT x43 = (x42 * aa_x) + (-1 * x41 * aa_x);
	const GEN_FLT x44 = aa_x * aa_z;
	const GEN_FLT x45 = (x44 * x37) + (-1 * x40 * x44);
	const GEN_FLT x46 = x38 * aa_y;
	const GEN_FLT x47 = (x42 * aa_z) + (-1 * x41 * aa_z);
	const GEN_FLT x48 = x38 * aa_z;
	out[0] = (-1 * x28 * aa_x) + (-1 * x24 * x21);
	out[1] = (-1 * x28 * aa_y) + (-1 * x33 * x24);
	out[2] = (-1 * x28 * aa_z) + (-1 * x36 * x24);
	out[3] = (-1 * x1 * x40) + (x1 * x37) + (-1 * x39 * x21) + x27;
	out[4] = x43 + (-1 * x33 * x39);
	out[5] = x45 + (-1 * x36 * x39);
	out[6] = x43 + (-1 * x46 * x21);
	out[7] = (-1 * x2 * x40) + x27 + (-1 * x46 * x33) + (x2 * x37);
	out[8] = x47 + (-1 * x46 * x36);
	out[9] = x45 + (-1 * x48 * x21);
	out[10] = x47 + (-1 * x48 * x33);
	out[11] = (-1 * x48 * x36) + (-1 * x0 * x40) + (x0 * x37) + x27;
}

static inline FLT gen_axisanglemagnitude(const FLT *axis_angle) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];

	return sqrt(1e-10 + (aa_y * aa_y) + (aa_z * aa_z) + (aa_x * aa_x));
}

// Jacobian of axisanglemagnitude wrt [aa_x, aa_y, aa_z]
static inline void gen_axisanglemagnitude_jac_axis_angle(FLT *out, const FLT *axis_angle) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT x0 = 1. / sqrt(1e-10 + (aa_y * aa_y) + (aa_z * aa_z) + (aa_x * aa_x));
	out[0] = x0 * aa_x;
	out[1] = x0 * aa_y;
	out[2] = x0 * aa_z;
}

static inline void gen_axisanglenormalize(FLT *out, const FLT *axis_angle) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT x0 = 1. / sqrt(1e-10 + (aa_y * aa_y) + (aa_z * aa_z) + (aa_x * aa_x));
	out[0] = x0 * aa_x;
	out[1] = x0 * aa_y;
	out[2] = x0 * aa_z;
}

// Jacobian of axisanglenormalize wrt [aa_x, aa_y, aa_z]
static inline void gen_axisanglenormalize_jac_axis_angle(FLT *out, const FLT *axis_angle) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT x0 = aa_x * aa_x;
	const GEN_FLT x1 = aa_z * aa_z;
	const GEN_FLT x2 = aa_y * aa_y;
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = 1. / (x3 * sqrt(x3));
	const GEN_FLT x5 = 1. / sqrt(x3);
	const GEN_FLT x6 = x4 * aa_x;
	const GEN_FLT x7 = -1 * x6 * aa_y;
	const GEN_FLT x8 = -1 * x6 * aa_z;
	const GEN_FLT x9 = -1 * x4 * aa_y * aa_z;
	out[0] = x5 + (-1 * x0 * x4);
	out[1] = x7;
	out[2] = x8;
	out[3] = x7;
	out[4] = x5 + (-1 * x2 * x4);
	out[5] = x9;
	out[6] = x8;
	out[7] = x9;
	out[8] = x5 + (-1 * x1 * x4);
}

static inline void gen_axisanglerotatevector(FLT *out, const FLT *axis_angle, const FLT *sensor_pt) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = aa_z * aa_z;
	const GEN_FLT x1 = aa_x * aa_x;
	const GEN_FLT x2 = aa_y * aa_y;
	const GEN_FLT x3 = 1e-10 + x2 + x0 + x1;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = (1. / x4) * sin(x4);
	const GEN_FLT x6 = x5 * aa_y;
	const GEN_FLT x7 = cos(x4);
	const GEN_FLT x8 = (1. / x3) * (1 + (-1 * x7));
	const GEN_FLT x9 = x8 * aa_z;
	const GEN_FLT x10 = x9 * aa_x;
	const GEN_FLT x11 = x5 * aa_z;
	const GEN_FLT x12 = x8 * aa_x * aa_y;
	const GEN_FLT x13 = x5 * aa_x;
	const GEN_FLT x14 = x9 * aa_y;
	out[0] = ((x12 + (-1 * x11)) * sensor_y) + ((x10 + x6) * sensor_z) + ((x7 + (x1 * x8)) * sensor_x);
	out[1] = ((x14 + (-1 * x13)) * sensor_z) + ((x7 + (x2 * x8)) * sensor_y) + ((x12 + x11) * sensor_x);
	out[2] = ((x14 + x13) * sensor_y) + ((x7 + (x0 * x8)) * sensor_z) + ((x10 + (-1 * x6)) * sensor_x);
}

// Jacobian of axisanglerotatevector wrt [aa_x, aa_y, aa_z]
static inline void gen_axisanglerotatevector_jac_axis_angle(FLT *out, const FLT *axis_angle, const FLT *sensor_pt) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = aa_z * aa_z;
	const GEN_FLT x1 = aa_x * aa_x;
	const GEN_FLT x2 = aa_y * aa_y;
	const GEN_FLT x3 = 1e-10 + x2 + x0 + x1;
	const GEN_FLT x4 = 1. / x3;
	const GEN_FLT x5 = sqrt(x3);
	const GEN_FLT x6 = cos(x5);
	const GEN_FLT x7 = 1 + (-1 * x6);
	const GEN_FLT x8 = x4 * x7;
	const GEN_FLT x9 = x8 * aa_y;
	const GEN_FLT x10 = sin(x5);
	const GEN_FLT x11 = (1. / (x3 * sqrt(x3))) * x10;
	const GEN_FLT x12 = x1 * x11;
	const GEN_FLT x13 = 2 * (1. / (x3 * x3)) * x7;
	const GEN_FLT x14 = x1 * x13;
	const GEN_FLT x15 = (-1 * x14 * aa_y) + (x12 * aa_y);
	const GEN_FLT x16 = x15 + x9;
	const GEN_FLT x17 = x4 * x6;
	const GEN_FLT x18 = aa_x * aa_z;
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = x11 * x18;
	const GEN_FLT x21 = x20 + (-1 * x19);
	const GEN_FLT x22 = x8 * aa_z;
	const GEN_FLT x23 = (-1 * x14 * aa_z) + (x12 * aa_z);
	const GEN_FLT x24 = x23 + x22;
	const GEN_FLT x25 = x17 * aa_y;
	const GEN_FLT x26 = x25 * aa_x;
	const GEN_FLT x27 = x11 * aa_y;
	const GEN_FLT x28 = x27 * aa_x;
	const GEN_FLT x29 = (-1 * x28) + x26;
	const GEN_FLT x30 = (1. / x5) * x10;
	const GEN_FLT x31 = -1 * x30 * aa_x;
	const GEN_FLT x32 = x8 * aa_x;
	const GEN_FLT x33 = aa_x * aa_x * aa_x;
	const GEN_FLT x34 = x2 * x11;
	const GEN_FLT x35 = x2 * x13;
	const GEN_FLT x36 = (-1 * x35 * aa_x) + (x34 * aa_x);
	const GEN_FLT x37 = x36 + x32;
	const GEN_FLT x38 = x25 * aa_z;
	const GEN_FLT x39 = x27 * aa_z;
	const GEN_FLT x40 = x39 + (-1 * x38);
	const GEN_FLT x41 = x2 * x17;
	const GEN_FLT x42 = (-1 * x13 * x18 * aa_y) + (x27 * x18);
	const GEN_FLT x43 = x42 + x30;
	const GEN_FLT x44 = -1 * x30 * aa_y;
	const GEN_FLT x45 = x0 * x11;
	const GEN_FLT x46 = x0 * x17;
	const GEN_FLT x47 = x42 + (-1 * x30);
	const GEN_FLT x48 = (-1 * x39) + x38;
	const GEN_FLT x49 = x0 * x13;
	const GEN_FLT x50 = (-1 * x49 * aa_x) + (x45 * aa_x);
	const GEN_FLT x51 = x50 + x32;
	const GEN_FLT x52 = -1 * x30 * aa_z;
	const GEN_FLT x53 = x1 * x17;
	const GEN_FLT x54 = (-1 * x20) + x19;
	const GEN_FLT x55 = aa_y * aa_y * aa_y;
	const GEN_FLT x56 = x28 + (-1 * x26);
	const GEN_FLT x57 = (-1 * x35 * aa_z) + (x34 * aa_z);
	const GEN_FLT x58 = x57 + x22;
	const GEN_FLT x59 = (-1 * x49 * aa_y) + (x45 * aa_y);
	const GEN_FLT x60 = x59 + x9;
	const GEN_FLT x61 = aa_z * aa_z * aa_z;
	out[0] = ((x21 + x16) * sensor_y) + (((-1 * x33 * x13) + x31 + (x33 * x11) + (2 * x32)) * sensor_x) +
			 ((x29 + x24) * sensor_z);
	out[1] = ((x15 + x44) * sensor_x) + ((x40 + x37) * sensor_y) + ((x43 + (-1 * x34) + x41) * sensor_z);
	out[2] = ((x23 + x52) * sensor_x) + ((x47 + x45 + (-1 * x46)) * sensor_y) + ((x51 + x48) * sensor_z);
	out[3] = ((x54 + x16) * sensor_x) + ((x36 + x31) * sensor_y) + ((x47 + x12 + (-1 * x53)) * sensor_z);
	out[4] = ((x48 + x37) * sensor_x) + (((2 * x9) + (x55 * x11) + x44 + (-1 * x55 * x13)) * sensor_y) +
			 ((x58 + x56) * sensor_z);
	out[5] = ((x43 + (-1 * x45) + x46) * sensor_x) + ((x57 + x52) * sensor_y) + ((x60 + x21) * sensor_z);
	out[6] = ((x43 + (-1 * x12) + x53) * sensor_y) + ((x56 + x24) * sensor_x) + ((x50 + x31) * sensor_z);
	out[7] = ((x47 + x34 + (-1 * x41)) * sensor_x) + ((x58 + x29) * sensor_y) + ((x59 + x44) * sensor_z);
	out[8] = ((x51 + x40) * sensor_x) + ((x60 + x54) * sensor_y) +
			 (((2 * x22) + x52 + (-1 * x61 * x13) + (x61 * x11)) * sensor_z);
}

// Jacobian of axisanglerotatevector wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_axisanglerotatevector_jac_sensor_pt(FLT *out, const FLT *axis_angle, const FLT *sensor_pt) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = aa_x * aa_x;
	const GEN_FLT x1 = aa_z * aa_z;
	const GEN_FLT x2 = aa_y * aa_y;
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = (1. / x3) * (1 + (-1 * x5));
	const GEN_FLT x7 = (1. / x4) * sin(x4);
	const GEN_FLT x8 = x7 * aa_z;
	const GEN_FLT x9 = x6 * aa_x * aa_y;
	const GEN_FLT x10 = x7 * aa_y;
	const GEN_FLT x11 = x6 * aa_z;
	const GEN_FLT x12 = x11 * aa_x;
	const GEN_FLT x13 = x7 * aa_x;
	const GEN_FLT x14 = x11 * aa_y;
	out[0] = x5 + (x0 * x6);
	out[1] = x9 + (-1 * x8);
	out[2] = x12 + x10;
	out[3] = x9 + x8;
	out[4] = x5 + (x2 * x6);
	out[5] = x14 + (-1 * x13);
	out[6] = x12 + (-1 * x10);
	out[7] = x14 + x13;
	out[8] = x5 + (x1 * x6);
}

static inline void gen_axisanglerotationmatrix(FLT *out, const FLT *axis_angle) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT x0 = aa_x * aa_x;
	const GEN_FLT x1 = aa_z * aa_z;
	const GEN_FLT x2 = aa_y * aa_y;
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = (1. / x3) * (1 + (-1 * x5));
	const GEN_FLT x7 = (1. / x4) * sin(x4);
	const GEN_FLT x8 = x7 * aa_z;
	const GEN_FLT x9 = x6 * aa_x * aa_y;
	const GEN_FLT x10 = x7 * aa_y;
	const GEN_FLT x11 = x6 * aa_z;
	const GEN_FLT x12 = x11 * aa_x;
	const GEN_FLT x13 = x7 * aa_x;
	const GEN_FLT x14 = x11 * aa_y;
	out[0] = x5 + (x0 * x6);
	out[1] = x9 + (-1 * x8);
	out[2] = x12 + x10;
	out[3] = x9 + x8;
	out[4] = x5 + (x2 * x6);
	out[5] = x14 + (-1 * x13);
	out[6] = x12 + (-1 * x10);
	out[7] = x14 + x13;
	out[8] = x5 + (x1 * x6);
}

// Jacobian of axisanglerotationmatrix wrt [aa_x, aa_y, aa_z]
static inline void gen_axisanglerotationmatrix_jac_axis_angle(FLT *out, const FLT *axis_angle) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT x0 = aa_z * aa_z;
	const GEN_FLT x1 = aa_x * aa_x;
	const GEN_FLT x2 = aa_y * aa_y;
	const GEN_FLT x3 = 1e-10 + x2 + x0 + x1;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = sin(x4);
	const GEN_FLT x6 = (1. / x4) * x5;
	const GEN_FLT x7 = 1. / x3;
	const GEN_FLT x8 = cos(x4);
	const GEN_FLT x9 = 1 + (-1 * x8);
	const GEN_FLT x10 = x7 * x9;
	const GEN_FLT x11 = x10 * aa_x;
	const GEN_FLT x12 = aa_x * aa_x * aa_x;
	const GEN_FLT x13 = 2 * (1. / (x3 * x3)) * x9;
	const GEN_FLT x14 = (1. / (x3 * sqrt(x3))) * x5;
	const GEN_FLT x15 = x14 * aa_y;
	const GEN_FLT x16 = x13 * aa_y;
	const GEN_FLT x17 = (-1 * x1 * x16) + (x1 * x15);
	const GEN_FLT x18 = x1 * aa_z;
	const GEN_FLT x19 = (-1 * x13 * x18) + (x14 * x18);
	const GEN_FLT x20 = x8 * x7;
	const GEN_FLT x21 = aa_x * aa_z;
	const GEN_FLT x22 = x2 * x14;
	const GEN_FLT x23 = x20 * aa_y;
	const GEN_FLT x24 = x23 * aa_z;
	const GEN_FLT x25 = x13 * aa_x;
	const GEN_FLT x26 = x15 * aa_z;
	const GEN_FLT x27 = x0 * x14;
	const GEN_FLT x28 = (x21 * x15) + (-1 * x21 * x16);
	out[0] = (-1 * x13 * x12) + (-1 * x6 * aa_x) + (x14 * x12) + (2 * x11);
	out[1] = x17 + (-1 * x6 * aa_y);
	out[2] = x19 + (-1 * x6 * aa_z);
	out[3] = x17 + (x21 * x14) + (-1 * x20 * x21) + (x10 * aa_y);
	out[4] = x26 + (-1 * x2 * x25) + (-1 * x24) + x11 + (x22 * aa_x);
	out[5] = (-1 * x6) + x27 + x28 + (-1 * x0 * x20);
	out[6] = x19 + (-1 * x15 * aa_x) + (x10 * aa_z) + (x23 * aa_x);
	out[7] = x6 + x28 + (-1 * x22) + (x2 * x20);
	out[8] = x24 + x11 + (-1 * x0 * x25) + (-1 * x26) + (x27 * aa_x);
}

static inline void gen_invert_pose(FLT *out, const SurvivePose *obj_p) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT x0 = 1. / sqrt(1e-11 + (obj_qw * obj_qw) + (obj_qi * obj_qi) + (obj_qj * obj_qj) + (obj_qk * obj_qk));
	const GEN_FLT x1 = x0 * obj_qw;
	const GEN_FLT x2 = x0 * obj_qk;
	const GEN_FLT x3 = x0 * obj_qi;
	const GEN_FLT x4 = (x3 * obj_pz) + (x1 * obj_py) + (-1 * x2 * obj_px);
	const GEN_FLT x5 = x0 * obj_qj;
	const GEN_FLT x6 = (x5 * obj_px) + (x1 * obj_pz) + (-1 * x3 * obj_py);
	const GEN_FLT x7 = (x2 * obj_py) + (-1 * x5 * obj_pz) + (x1 * obj_px);
	out[0] = -1 * (obj_px + (2 * ((-1 * x6 * x5) + (x2 * x4))));
	out[1] = -1 * (obj_py + (2 * ((-1 * x2 * x7) + (x3 * x6))));
	out[2] = -1 * (obj_pz + (2 * ((-1 * x4 * x3) + (x5 * x7))));
	out[3] = x1;
	out[4] = -1 * x3;
	out[5] = -1 * x5;
	out[6] = -1 * x2;
}

// Jacobian of invert_pose wrt [obj_px, obj_py, obj_pz, obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_invert_pose_jac_obj_p(FLT *out, const SurvivePose *obj_p) {
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT x0 = obj_qk * obj_qk;
	const GEN_FLT x1 = obj_qw * obj_qw;
	const GEN_FLT x2 = obj_qj * obj_qj;
	const GEN_FLT x3 = obj_qi * obj_qi;
	const GEN_FLT x4 = 1e-11 + x2 + x1 + x3 + x0;
	const GEN_FLT x5 = 2 * (1. / x4);
	const GEN_FLT x6 = -1 * x0 * x5;
	const GEN_FLT x7 = 1 + (-1 * x2 * x5);
	const GEN_FLT x8 = x5 * obj_qw;
	const GEN_FLT x9 = x8 * obj_qk;
	const GEN_FLT x10 = x5 * obj_qi;
	const GEN_FLT x11 = x10 * obj_qj;
	const GEN_FLT x12 = x10 * obj_qk;
	const GEN_FLT x13 = x8 * obj_qj;
	const GEN_FLT x14 = 1. / (x4 * sqrt(x4));
	const GEN_FLT x15 = x14 * obj_qw;
	const GEN_FLT x16 = x15 * obj_qk;
	const GEN_FLT x17 = 1. / sqrt(x4);
	const GEN_FLT x18 = x17 * obj_py;
	const GEN_FLT x19 = x17 * obj_px;
	const GEN_FLT x20 = x17 * obj_pz;
	const GEN_FLT x21 = (x20 * obj_qi) + (x18 * obj_qw) + (-1 * x19 * obj_qk);
	const GEN_FLT x22 = 2 * x21;
	const GEN_FLT x23 = x16 * obj_px;
	const GEN_FLT x24 = x15 * obj_qi;
	const GEN_FLT x25 = -1 * x24 * obj_pz;
	const GEN_FLT x26 = x1 * x14;
	const GEN_FLT x27 = x18 + (-1 * x26 * obj_py) + x23 + x25;
	const GEN_FLT x28 = 2 * x17;
	const GEN_FLT x29 = x28 * obj_qk;
	const GEN_FLT x30 = x15 * obj_qj;
	const GEN_FLT x31 = (x19 * obj_qj) + (x20 * obj_qw) + (-1 * x18 * obj_qi);
	const GEN_FLT x32 = 2 * x31;
	const GEN_FLT x33 = -1 * x30 * obj_px;
	const GEN_FLT x34 = x24 * obj_py;
	const GEN_FLT x35 = (-1 * x26 * obj_pz) + x33 + x20 + x34;
	const GEN_FLT x36 = x28 * obj_qj;
	const GEN_FLT x37 = x14 * obj_qi;
	const GEN_FLT x38 = x37 * obj_qk;
	const GEN_FLT x39 = x38 * x22;
	const GEN_FLT x40 = x3 * x14;
	const GEN_FLT x41 = (x38 * obj_px) + (-1 * x34) + x20 + (-1 * x40 * obj_pz);
	const GEN_FLT x42 = x37 * obj_qj;
	const GEN_FLT x43 = x42 * x32;
	const GEN_FLT x44 = x25 + (x40 * obj_py) + (-1 * x42 * obj_px) + (-1 * x18);
	const GEN_FLT x45 = obj_qk * obj_qj;
	const GEN_FLT x46 = x45 * x14;
	const GEN_FLT x47 = x2 * x14;
	const GEN_FLT x48 = x30 * obj_pz;
	const GEN_FLT x49 = (x42 * obj_py) + (-1 * x48) + x19 + (-1 * x47 * obj_px);
	const GEN_FLT x50 = x42 * obj_pz;
	const GEN_FLT x51 = x46 * obj_px;
	const GEN_FLT x52 = x51 + (-1 * x50) + (-1 * x30 * obj_py);
	const GEN_FLT x53 = x31 * x28;
	const GEN_FLT x54 = -1 * x16 * obj_py;
	const GEN_FLT x55 = x0 * x14;
	const GEN_FLT x56 = x54 + (-1 * x38 * obj_pz) + (x55 * obj_px) + (-1 * x19);
	const GEN_FLT x57 = x21 * x28;
	const GEN_FLT x58 = x38 * obj_py;
	const GEN_FLT x59 = x58 + (-1 * x51) + (-1 * x16 * obj_pz);
	const GEN_FLT x60 = -1 * x3 * x5;
	const GEN_FLT x61 = x10 * obj_qw;
	const GEN_FLT x62 = x5 * x45;
	const GEN_FLT x63 = (-1 * x26 * obj_px) + x54 + x48 + x19;
	const GEN_FLT x64 = x28 * obj_qi;
	const GEN_FLT x65 = (x18 * obj_qk) + (-1 * x20 * obj_qj) + (x19 * obj_qw);
	const GEN_FLT x66 = 2 * x65;
	const GEN_FLT x67 = (-1 * x24 * obj_px) + (-1 * x58) + x50;
	const GEN_FLT x68 = x66 * x46;
	const GEN_FLT x69 = (-1 * x20) + (x47 * obj_pz) + (-1 * x46 * obj_py) + x33;
	const GEN_FLT x70 = x65 * x28;
	const GEN_FLT x71 = (-1 * x23) + (x46 * obj_pz) + x18 + (-1 * x55 * obj_py);
	const GEN_FLT x72 = -1 * x17;
	out[0] = -1 * (x7 + x6);
	out[1] = -1 * (x11 + x9);
	out[2] = -1 * ((-1 * x13) + x12);
	out[3] = -1 * ((-1 * x36 * x35) + (x30 * x32) + (-1 * x22 * x16) + (x29 * x27));
	out[4] = -1 * ((-1 * x44 * x36) + x43 + (-1 * x39) + (x41 * x29));
	out[5] = -1 * ((-1 * x53) + (x52 * x29) + (-1 * x46 * x22) + (x47 * x32) + (-1 * x49 * x36));
	out[6] = -1 * ((-1 * x59 * x36) + x57 + (x56 * x29) + (x46 * x32) + (-1 * x55 * x22));
	out[7] = -1 * ((-1 * x9) + x11);
	out[8] = -1 * (1 + x6 + x60);
	out[9] = -1 * (x62 + x61);
	out[10] = -1 * ((x66 * x16) + (x64 * x35) + (-1 * x32 * x24) + (-1 * x63 * x29));
	out[11] = -1 * ((-1 * x40 * x32) + (-1 * x67 * x29) + x53 + (x66 * x38) + (x64 * x44));
	out[12] = -1 * ((-1 * x69 * x29) + (-1 * x43) + (x64 * x49) + x68);
	out[13] = -1 * ((-1 * x71 * x29) + (-1 * x70) + (x66 * x55) + (-1 * x32 * x38) + (x64 * x59));
	out[14] = -1 * (x12 + x13);
	out[15] = -1 * ((-1 * x61) + x62);
	out[16] = -1 * (x7 + x60);
	out[17] = -1 * ((x24 * x22) + (-1 * x64 * x27) + (-1 * x66 * x30) + (x63 * x36));
	out[18] = -1 * ((x67 * x36) + (-1 * x64 * x41) + (-1 * x66 * x42) + (x40 * x22) + (-1 * x57));
	out[19] = -1 * ((-1 * x64 * x52) + (x42 * x22) + (x69 * x36) + x70 + (-1 * x66 * x47));
	out[20] = -1 * ((-1 * x64 * x56) + x39 + (-1 * x68) + (x71 * x36));
	out[21] = 0;
	out[22] = 0;
	out[23] = 0;
	out[24] = x17 + (-1 * x26);
	out[25] = -1 * x24;
	out[26] = -1 * x30;
	out[27] = -1 * x16;
	out[28] = 0;
	out[29] = 0;
	out[30] = 0;
	out[31] = x24;
	out[32] = x72 + x40;
	out[33] = x42;
	out[34] = x38;
	out[35] = 0;
	out[36] = 0;
	out[37] = 0;
	out[38] = x30;
	out[39] = x42;
	out[40] = x72 + x47;
	out[41] = x46;
	out[42] = 0;
	out[43] = 0;
	out[44] = 0;
	out[45] = x16;
	out[46] = x38;
	out[47] = x46;
	out[48] = x72 + x55;
}

static inline void gen_quatgetreciprocal(FLT *out, const FLT *q) {
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];

	out[0] = obj_qw;
	out[1] = -1 * obj_qi;
	out[2] = -1 * obj_qj;
	out[3] = -1 * obj_qk;
}

// Jacobian of quatgetreciprocal wrt [obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_quatgetreciprocal_jac_q(FLT *out, const FLT *q) {
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];

	out[0] = 1;
	out[1] = 0;
	out[2] = 0;
	out[3] = 0;
	out[4] = 0;
	out[5] = -1;
	out[6] = 0;
	out[7] = 0;
	out[8] = 0;
	out[9] = 0;
	out[10] = -1;
	out[11] = 0;
	out[12] = 0;
	out[13] = 0;
	out[14] = 0;
	out[15] = -1;
}

static inline FLT gen_quatmagnitude(const FLT *q) {
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];

	return sqrt(1e-11 + (obj_qw * obj_qw) + (obj_qi * obj_qi) + (obj_qj * obj_qj) + (obj_qk * obj_qk));
}

// Jacobian of quatmagnitude wrt [obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_quatmagnitude_jac_q(FLT *out, const FLT *q) {
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];
	const GEN_FLT x0 = 1. / sqrt(1e-11 + (obj_qw * obj_qw) + (obj_qi * obj_qi) + (obj_qj * obj_qj) + (obj_qk * obj_qk));
	out[0] = x0 * obj_qw;
	out[1] = x0 * obj_qi;
	out[2] = x0 * obj_qj;
	out[3] = x0 * obj_qk;
}

static inline void gen_quatnormalize(FLT *out, const FLT *q) {
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];
	const GEN_FLT x0 = 1. / sqrt(1e-11 + (obj_qw * obj_qw) + (obj_qi * obj_qi) + (obj_qj * obj_qj) + (obj_qk * obj_qk));
	out[0] = x0 * obj_qw;
	out[1] = x0 * obj_qi;
	out[2] = x0 * obj_qj;
	out[3] = x0 * obj_qk;
}

// Jacobian of quatnormalize wrt [obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_quatnormalize_jac_q(FLT *out, const FLT *q) {
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];
	const GEN_FLT x0 = obj_qw * obj_qw;
	const GEN_FLT x1 = obj_qk * obj_qk;
	const GEN_FLT x2 = obj_qj * obj_qj;
	const GEN_FLT x3 = obj_qi * obj_qi;
	const GEN_FLT x4 = 1e-11 + x2 + x3 + x0 + x1;
	const GEN_FLT x5 = 1. / (x4 * sqrt(x4));
	const GEN_FLT x6 = 1. / sqrt(x4);
	const GEN_FLT x7 = x5 * obj_qi;
	const GEN_FLT x8 = -1 * x7 * obj_qw;
	const GEN_FLT x9 = -1 * x5 * obj_qw * obj_qj;
	const GEN_FLT x10 = x5 * obj_qk;
	const GEN_FLT x11 = -1 * x10 * obj_qw;
	const GEN_FLT x12 = -1 * x7 * obj_qj;
	const GEN_FLT x13 = -1 * x10 * obj_qi;
	const GEN_FLT x14 = -1 * x10 * obj_qj;
	out[0] = x6 + (-1 * x0 * x5);
	out[1] = x8;
	out[2] = x9;
	out[3] = x11;
	out[4] = x8;
	out[5] = x6 + (-1 * x3 * x5);
	out[6] = x12;
	out[7] = x13;
	out[8] = x9;
	out[9] = x12;
	out[10] = x6 + (-1 * x2 * x5);
	out[11] = x14;
	out[12] = x11;
	out[13] = x13;
	out[14] = x14;
	out[15] = x6 + (-1 * x1 * x5);
}

static inline void gen_quatrotatevector(FLT *out, const FLT *q, const FLT *pt) {
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];
	const GEN_FLT pt_x = pt[0];
	const GEN_FLT pt_y = pt[1];
	const GEN_FLT pt_z = pt[2];
	const GEN_FLT x0 = (-1 * pt_z * obj_qi) + (pt_y * obj_qw) + (pt_x * obj_qk);
	const GEN_FLT x1 = (-1 * pt_x * obj_qj) + (pt_z * obj_qw) + (pt_y * obj_qi);
	const GEN_FLT x2 = (pt_z * obj_qj) + (-1 * pt_y * obj_qk) + (pt_x * obj_qw);
	out[0] = pt_x + (2 * ((x1 * obj_qj) + (-1 * x0 * obj_qk)));
	out[1] = pt_y + (2 * ((x2 * obj_qk) + (-1 * x1 * obj_qi)));
	out[2] = pt_z + (2 * ((x0 * obj_qi) + (-1 * x2 * obj_qj)));
}

// Jacobian of quatrotatevector wrt [obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_quatrotatevector_jac_q(FLT *out, const FLT *q, const FLT *pt) {
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];
	const GEN_FLT pt_x = pt[0];
	const GEN_FLT pt_y = pt[1];
	const GEN_FLT pt_z = pt[2];
	const GEN_FLT x0 = 2 * pt_y;
	const GEN_FLT x1 = x0 * obj_qk;
	const GEN_FLT x2 = 2 * pt_z;
	const GEN_FLT x3 = x2 * obj_qj;
	const GEN_FLT x4 = x2 * obj_qk;
	const GEN_FLT x5 = x0 * obj_qj;
	const GEN_FLT x6 = pt_x * obj_qj;
	const GEN_FLT x7 = 2 * obj_qw;
	const GEN_FLT x8 = x7 * pt_z;
	const GEN_FLT x9 = 2 * obj_qi;
	const GEN_FLT x10 = x9 * pt_y;
	const GEN_FLT x11 = x7 * pt_y;
	const GEN_FLT x12 = pt_x * obj_qk;
	const GEN_FLT x13 = x9 * pt_z;
	const GEN_FLT x14 = 2 * x12;
	const GEN_FLT x15 = 2 * x6;
	const GEN_FLT x16 = 4 * pt_y;
	const GEN_FLT x17 = x9 * pt_x;
	const GEN_FLT x18 = x7 * pt_x;
	const GEN_FLT x19 = 4 * pt_z;
	out[0] = x3 + (-1 * x1);
	out[1] = x5 + x4;
	out[2] = x10 + (-4 * x6) + x8;
	out[3] = x13 + (-1 * x11) + (-4 * x12);
	out[4] = x14 + (-1 * x13);
	out[5] = (-1 * x16 * obj_qi) + x15 + (-1 * x8);
	out[6] = x4 + x17;
	out[7] = x18 + x3 + (-1 * x16 * obj_qk);
	out[8] = x10 + (-1 * x15);
	out[9] = x11 + (-1 * x19 * obj_qi) + x14;
	out[10] = (-1 * x18) + (-1 * x19 * obj_qj) + x1;
	out[11] = x17 + x5;
}

// Jacobian of quatrotatevector wrt [pt_x, pt_y, pt_z]
static inline void gen_quatrotatevector_jac_pt(FLT *out, const FLT *q, const FLT *pt) {
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];
	const GEN_FLT pt_x = pt[0];
	const GEN_FLT pt_y = pt[1];
	const GEN_FLT pt_z = pt[2];
	const GEN_FLT x0 = -2 * (obj_qk * obj_qk);
	const GEN_FLT x1 = 1 + (-2 * (obj_qj * obj_qj));
	const GEN_FLT x2 = 2 * obj_qw;
	const GEN_FLT x3 = x2 * obj_qk;
	const GEN_FLT x4 = 2 * obj_qi;
	const GEN_FLT x5 = x4 * obj_qj;
	const GEN_FLT x6 = x4 * obj_qk;
	const GEN_FLT x7 = x2 * obj_qj;
	const GEN_FLT x8 = -2 * (obj_qi * obj_qi);
	const GEN_FLT x9 = x2 * obj_qi;
	const GEN_FLT x10 = 2 * obj_qk * obj_qj;
	out[0] = x1 + x0;
	out[1] = x5 + (-1 * x3);
	out[2] = x7 + x6;
	out[3] = x3 + x5;
	out[4] = 1 + x0 + x8;
	out[5] = x10 + (-1 * x9);
	out[6] = x6 + (-1 * x7);
	out[7] = x9 + x10;
	out[8] = x1 + x8;
}

static inline void gen_quatrotatevector2(FLT *out, const FLT *q, const FLT *sensor_pt) {
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = obj_qw * obj_qj;
	const GEN_FLT x1 = obj_qk * obj_qi;
	const GEN_FLT x2 = obj_qk * obj_qk;
	const GEN_FLT x3 = obj_qj * obj_qj;
	const GEN_FLT x4 = obj_qi * obj_qi;
	const GEN_FLT x5 = x4 + x3;
	const GEN_FLT x6 = 2 * sqrt(1e-11 + x5 + x2 + (obj_qw * obj_qw));
	const GEN_FLT x7 = x6 * sensor_z;
	const GEN_FLT x8 = obj_qw * obj_qk;
	const GEN_FLT x9 = obj_qj * obj_qi;
	const GEN_FLT x10 = x6 * sensor_y;
	const GEN_FLT x11 = x6 * sensor_x;
	const GEN_FLT x12 = obj_qw * obj_qi;
	const GEN_FLT x13 = obj_qk * obj_qj;
	out[0] = ((x9 + (-1 * x8)) * x10) + ((x1 + x0) * x7) + ((1 + (-1 * (x3 + x2) * x6)) * sensor_x);
	out[1] = ((x9 + x8) * x11) + ((1 + (-1 * (x4 + x2) * x6)) * sensor_y) + ((x13 + (-1 * x12)) * x7);
	out[2] = ((x13 + x12) * x10) + ((1 + (-1 * x6 * x5)) * sensor_z) + ((x1 + (-1 * x0)) * x11);
}

// Jacobian of quatrotatevector2 wrt [obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_quatrotatevector2_jac_q(FLT *out, const FLT *q, const FLT *sensor_pt) {
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = obj_qi * obj_qi;
	const GEN_FLT x1 = obj_qk * obj_qk;
	const GEN_FLT x2 = obj_qj * obj_qj;
	const GEN_FLT x3 = x2 + x1;
	const GEN_FLT x4 = sqrt(1e-11 + x3 + x0 + (obj_qw * obj_qw));
	const GEN_FLT x5 = 2 * x4;
	const GEN_FLT x6 = x5 * obj_qk;
	const GEN_FLT x7 = x6 * sensor_y;
	const GEN_FLT x8 = obj_qw * sensor_y;
	const GEN_FLT x9 = obj_qw * obj_qk;
	const GEN_FLT x10 = obj_qj * obj_qi;
	const GEN_FLT x11 = x10 + (-1 * x9);
	const GEN_FLT x12 = 2 * (1. / x4);
	const GEN_FLT x13 = x12 * x11;
	const GEN_FLT x14 = x3 * x12;
	const GEN_FLT x15 = x14 * sensor_x;
	const GEN_FLT x16 = obj_qw * obj_qj;
	const GEN_FLT x17 = obj_qk * obj_qi;
	const GEN_FLT x18 = (x17 + x16) * sensor_z;
	const GEN_FLT x19 = x12 * x18;
	const GEN_FLT x20 = x5 * obj_qj;
	const GEN_FLT x21 = x20 * sensor_z;
	const GEN_FLT x22 = x20 * sensor_y;
	const GEN_FLT x23 = x12 * obj_qi;
	const GEN_FLT x24 = x6 * sensor_z;
	const GEN_FLT x25 = 4 * x4;
	const GEN_FLT x26 = -1 * x25 * obj_qj;
	const GEN_FLT x27 = x13 * sensor_y;
	const GEN_FLT x28 = x5 * obj_qi;
	const GEN_FLT x29 = x28 * sensor_y;
	const GEN_FLT x30 = x5 * obj_qw;
	const GEN_FLT x31 = x30 * sensor_z;
	const GEN_FLT x32 = x30 * sensor_y;
	const GEN_FLT x33 = -1 * x25 * obj_qk;
	const GEN_FLT x34 = x28 * sensor_z;
	const GEN_FLT x35 = (x0 + x1) * x12;
	const GEN_FLT x36 = obj_qw * sensor_z;
	const GEN_FLT x37 = obj_qw * obj_qi;
	const GEN_FLT x38 = obj_qk * obj_qj;
	const GEN_FLT x39 = x38 + (-1 * x37);
	const GEN_FLT x40 = x39 * x12;
	const GEN_FLT x41 = (x10 + x9) * sensor_x;
	const GEN_FLT x42 = x41 * x12;
	const GEN_FLT x43 = x6 * sensor_x;
	const GEN_FLT x44 = -1 * x25 * obj_qi;
	const GEN_FLT x45 = x20 * sensor_x;
	const GEN_FLT x46 = x40 * sensor_z;
	const GEN_FLT x47 = x28 * sensor_x;
	const GEN_FLT x48 = x30 * sensor_x;
	const GEN_FLT x49 = (x38 + x37) * sensor_y;
	const GEN_FLT x50 = x49 * x12;
	const GEN_FLT x51 = (x0 + x2) * x12;
	const GEN_FLT x52 = (x17 + (-1 * x16)) * sensor_x;
	const GEN_FLT x53 = x52 * x12;
	out[0] = x21 + (x19 * obj_qw) + (-1 * x7) + (-1 * x15 * obj_qw) + (x8 * x13);
	out[1] = x24 + (x23 * x18) + (-1 * x15 * obj_qi) + x22 + (x23 * x11 * sensor_y);
	out[2] = x31 + x29 + (x19 * obj_qj) + (((-1 * x14 * obj_qj) + x26) * sensor_x) + (x27 * obj_qj);
	out[3] = x34 + (x19 * obj_qk) + (((-1 * x14 * obj_qk) + x33) * sensor_x) + (-1 * x32) + (x27 * obj_qk);
	out[4] = x43 + (-1 * x34) + (x42 * obj_qw) + (-1 * x8 * x35) + (x40 * x36);
	out[5] = (x41 * x23) + (x39 * x23 * sensor_z) + x45 + (((-1 * x35 * obj_qi) + x44) * sensor_y) + (-1 * x31);
	out[6] = (x42 * obj_qj) + x47 + (x46 * obj_qj) + (-1 * x35 * obj_qj * sensor_y) + x24;
	out[7] = x48 + x21 + (x42 * obj_qk) + (((-1 * x35 * obj_qk) + x33) * sensor_y) + (x46 * obj_qk);
	out[8] = (x53 * obj_qw) + (-1 * x51 * x36) + (x50 * obj_qw) + x29 + (-1 * x45);
	out[9] = (x52 * x23) + (((-1 * x51 * obj_qi) + x44) * sensor_z) + x32 + (x49 * x23) + x43;
	out[10] = x7 + (((-1 * x51 * obj_qj) + x26) * sensor_z) + (x50 * obj_qj) + (x53 * obj_qj) + (-1 * x48);
	out[11] = (x53 * obj_qk) + (-1 * x51 * obj_qk * sensor_z) + x47 + x22 + (x50 * obj_qk);
}

// Jacobian of quatrotatevector2 wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_quatrotatevector2_jac_sensor_pt(FLT *out, const FLT *q, const FLT *sensor_pt) {
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = obj_qk * obj_qk;
	const GEN_FLT x1 = obj_qj * obj_qj;
	const GEN_FLT x2 = obj_qi * obj_qi;
	const GEN_FLT x3 = x2 + x1;
	const GEN_FLT x4 = 2 * sqrt(1e-11 + x3 + x0 + (obj_qw * obj_qw));
	const GEN_FLT x5 = obj_qw * obj_qk;
	const GEN_FLT x6 = obj_qj * obj_qi;
	const GEN_FLT x7 = obj_qw * obj_qj;
	const GEN_FLT x8 = obj_qk * obj_qi;
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	out[0] = 1 + (-1 * (x1 + x0) * x4);
	out[1] = (x6 + (-1 * x5)) * x4;
	out[2] = (x8 + x7) * x4;
	out[3] = (x6 + x5) * x4;
	out[4] = 1 + (-1 * (x2 + x0) * x4);
	out[5] = x4 * (x10 + (-1 * x9));
	out[6] = (x8 + (-1 * x7)) * x4;
	out[7] = x4 * (x10 + x9);
	out[8] = 1 + (-1 * x4 * x3);
}

static inline void gen_quatrotationmatrix(FLT *out, const FLT *q) {
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];
	const GEN_FLT x0 = obj_qk * obj_qk;
	const GEN_FLT x1 = obj_qj * obj_qj;
	const GEN_FLT x2 = obj_qi * obj_qi;
	const GEN_FLT x3 = x2 + x1;
	const GEN_FLT x4 = 2 * sqrt(1e-11 + x3 + x0 + (obj_qw * obj_qw));
	const GEN_FLT x5 = obj_qw * obj_qk;
	const GEN_FLT x6 = obj_qj * obj_qi;
	const GEN_FLT x7 = obj_qw * obj_qj;
	const GEN_FLT x8 = obj_qk * obj_qi;
	const GEN_FLT x9 = obj_qw * obj_qi;
	const GEN_FLT x10 = obj_qk * obj_qj;
	out[0] = 1 + (-1 * (x1 + x0) * x4);
	out[1] = (x6 + (-1 * x5)) * x4;
	out[2] = (x8 + x7) * x4;
	out[3] = (x6 + x5) * x4;
	out[4] = 1 + (-1 * (x2 + x0) * x4);
	out[5] = x4 * (x10 + (-1 * x9));
	out[6] = (x8 + (-1 * x7)) * x4;
	out[7] = x4 * (x10 + x9);
	out[8] = 1 + (-1 * x4 * x3);
}

// Jacobian of quatrotationmatrix wrt [obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_quatrotationmatrix_jac_q(FLT *out, const FLT *q) {
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];
	const GEN_FLT x0 = (obj_qj * obj_qj) + (obj_qk * obj_qk);
	const GEN_FLT x1 = sqrt(1e-11 + x0 + (obj_qw * obj_qw) + (obj_qi * obj_qi));
	const GEN_FLT x2 = 2 * (1. / x1);
	const GEN_FLT x3 = x2 * obj_qw;
	const GEN_FLT x4 = x0 * x2;
	const GEN_FLT x5 = 4 * x1;
	const GEN_FLT x6 = (obj_qj * obj_qi) + (-1 * obj_qw * obj_qk);
	const GEN_FLT x7 = 2 * x1;
	const GEN_FLT x8 = x7 * obj_qk;
	const GEN_FLT x9 = x2 * x6;
	const GEN_FLT x10 = x7 * obj_qj;
	const GEN_FLT x11 = x7 * obj_qi;
	const GEN_FLT x12 = x7 * obj_qw;
	const GEN_FLT x13 = (obj_qk * obj_qi) + (obj_qw * obj_qj);
	const GEN_FLT x14 = x2 * x13;
	out[0] = -1 * x0 * x3;
	out[1] = -1 * x4 * obj_qi;
	out[2] = (-1 * x4 * obj_qj) + (-1 * x5 * obj_qj);
	out[3] = (-1 * x4 * obj_qk) + (-1 * x5 * obj_qk);
	out[4] = (-1 * x8) + (x3 * x6);
	out[5] = x10 + (x9 * obj_qi);
	out[6] = x11 + (x9 * obj_qj);
	out[7] = (-1 * x12) + (x9 * obj_qk);
	out[8] = x10 + (x3 * x13);
	out[9] = x8 + (x14 * obj_qi);
	out[10] = x12 + (x14 * obj_qj);
	out[11] = x11 + (x14 * obj_qk);
}

static inline void gen_quat2axisangle(FLT *out, const FLT *q) {
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];
	const GEN_FLT x0 = sqrt(1e-10 + (obj_qj * obj_qj) + (obj_qk * obj_qk) + (obj_qi * obj_qi));
	const GEN_FLT x1 = 2 * (1. / x0) * atan2(x0, obj_qw);
	out[0] = x1 * obj_qi;
	out[1] = x1 * obj_qj;
	out[2] = x1 * obj_qk;
}

// Jacobian of quat2axisangle wrt [obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_quat2axisangle_jac_q(FLT *out, const FLT *q) {
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];
	const GEN_FLT x0 = obj_qk * obj_qk;
	const GEN_FLT x1 = obj_qi * obj_qi;
	const GEN_FLT x2 = obj_qj * obj_qj;
	const GEN_FLT x3 = 1e-10 + x2 + x0 + x1;
	const GEN_FLT x4 = 2 * (1. / (x3 + (obj_qw * obj_qw)));
	const GEN_FLT x5 = sqrt(x3);
	const GEN_FLT x6 = 2 * atan2(x5, obj_qw);
	const GEN_FLT x7 = (1. / (x3 * sqrt(x3))) * x6;
	const GEN_FLT x8 = x6 * (1. / x5);
	const GEN_FLT x9 = (1. / x3) * obj_qw;
	const GEN_FLT x10 = x4 * x9;
	const GEN_FLT x11 = x7 * obj_qj;
	const GEN_FLT x12 = x4 * obj_qj;
	const GEN_FLT x13 = (x9 * x12 * obj_qi) + (-1 * x11 * obj_qi);
	const GEN_FLT x14 = x4 * obj_qk;
	const GEN_FLT x15 = x9 * x14;
	const GEN_FLT x16 = (x15 * obj_qi) + (-1 * x7 * obj_qk * obj_qi);
	const GEN_FLT x17 = (x15 * obj_qj) + (-1 * x11 * obj_qk);
	out[0] = -1 * x4 * obj_qi;
	out[1] = (x1 * x10) + (-1 * x1 * x7) + x8;
	out[2] = x13;
	out[3] = x16;
	out[4] = -1 * x12;
	out[5] = x13;
	out[6] = (x2 * x10) + x8 + (-1 * x2 * x7);
	out[7] = x17;
	out[8] = -1 * x14;
	out[9] = x16;
	out[10] = x17;
	out[11] = (x0 * x10) + (-1 * x0 * x7) + x8;
}

static inline void gen_sensor_to_world(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
									   const SurvivePose *lh_p) {
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
	const GEN_FLT x0 = (-1 * obj_qj * sensor_x) + (obj_qw * sensor_z) + (obj_qi * sensor_y);
	const GEN_FLT x1 = (obj_qw * sensor_x) + (-1 * obj_qk * sensor_y) + (obj_qj * sensor_z);
	const GEN_FLT x2 = (2 * ((x1 * obj_qk) + (-1 * x0 * obj_qi))) + obj_py + sensor_y;
	const GEN_FLT x3 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x4 = (2 * ((x0 * obj_qj) + (-1 * x3 * obj_qk))) + obj_px + sensor_x;
	const GEN_FLT x5 = (2 * ((x3 * obj_qi) + (-1 * x1 * obj_qj))) + obj_pz + sensor_z;
	const GEN_FLT x6 = (-1 * x5 * lh_qi) + (x2 * lh_qw) + (x4 * lh_qk);
	const GEN_FLT x7 = (-1 * x4 * lh_qj) + (x5 * lh_qw) + (x2 * lh_qi);
	const GEN_FLT x8 = (-1 * x2 * lh_qk) + (x4 * lh_qw) + (x5 * lh_qj);
	out[0] = x4 + lh_px + (2 * ((x7 * lh_qj) + (-1 * x6 * lh_qk)));
	out[1] = x2 + lh_py + (2 * ((x8 * lh_qk) + (-1 * x7 * lh_qi)));
	out[2] = x5 + lh_pz + (2 * ((x6 * lh_qi) + (-1 * x8 * lh_qj)));
}

// Jacobian of sensor_to_world wrt [obj_px, obj_py, obj_pz, obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_sensor_to_world_jac_obj_p(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
												 const SurvivePose *lh_p) {
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
	const GEN_FLT x0 = -2 * (lh_qk * lh_qk);
	const GEN_FLT x1 = 1 + (-2 * (lh_qj * lh_qj));
	const GEN_FLT x2 = 2 * lh_qk;
	const GEN_FLT x3 = x2 * lh_qw;
	const GEN_FLT x4 = 2 * lh_qi;
	const GEN_FLT x5 = x4 * lh_qj;
	const GEN_FLT x6 = x2 * lh_qi;
	const GEN_FLT x7 = 2 * lh_qj;
	const GEN_FLT x8 = x7 * lh_qw;
	const GEN_FLT x9 = 2 * sensor_x;
	const GEN_FLT x10 = x9 * obj_qj;
	const GEN_FLT x11 = 2 * obj_qi;
	const GEN_FLT x12 = x11 * sensor_y;
	const GEN_FLT x13 = x12 + (-1 * x10);
	const GEN_FLT x14 = 2 * obj_qk;
	const GEN_FLT x15 = x14 * sensor_y;
	const GEN_FLT x16 = 2 * obj_qj;
	const GEN_FLT x17 = x16 * sensor_z;
	const GEN_FLT x18 = x17 + (-1 * x15);
	const GEN_FLT x19 = x11 * sensor_z;
	const GEN_FLT x20 = x9 * obj_qk;
	const GEN_FLT x21 = x20 + (-1 * x19);
	const GEN_FLT x22 = (x21 * lh_qi) + (x13 * lh_qw) + (-1 * x18 * lh_qj);
	const GEN_FLT x23 = (x18 * lh_qk) + (-1 * x13 * lh_qi) + (x21 * lh_qw);
	const GEN_FLT x24 = 4 * obj_qi;
	const GEN_FLT x25 = 2 * obj_qw;
	const GEN_FLT x26 = x25 * sensor_y;
	const GEN_FLT x27 = x26 + x20 + (-1 * x24 * sensor_z);
	const GEN_FLT x28 = x14 * sensor_z;
	const GEN_FLT x29 = x16 * sensor_y;
	const GEN_FLT x30 = x29 + x28;
	const GEN_FLT x31 = x25 * sensor_z;
	const GEN_FLT x32 = (-1 * x24 * sensor_y) + x10 + (-1 * x31);
	const GEN_FLT x33 = 2 * ((x32 * lh_qi) + (x27 * lh_qw) + (-1 * x30 * lh_qj));
	const GEN_FLT x34 = (-1 * x27 * lh_qi) + (x30 * lh_qk) + (x32 * lh_qw);
	const GEN_FLT x35 = 4 * obj_qj;
	const GEN_FLT x36 = x12 + (-1 * x35 * sensor_x) + x31;
	const GEN_FLT x37 = x25 * sensor_x;
	const GEN_FLT x38 = (-1 * x35 * sensor_z) + (-1 * x37) + x15;
	const GEN_FLT x39 = x9 * obj_qi;
	const GEN_FLT x40 = x28 + x39;
	const GEN_FLT x41 = (x40 * lh_qi) + (-1 * x36 * lh_qj) + (x38 * lh_qw);
	const GEN_FLT x42 = (x36 * lh_qk) + (-1 * x38 * lh_qi) + (x40 * lh_qw);
	const GEN_FLT x43 = 4 * obj_qk;
	const GEN_FLT x44 = (-1 * x26) + (-1 * x43 * sensor_x) + x19;
	const GEN_FLT x45 = x39 + x29;
	const GEN_FLT x46 = x17 + x37 + (-1 * x43 * sensor_y);
	const GEN_FLT x47 = 2 * ((x46 * lh_qi) + (-1 * x44 * lh_qj) + (x45 * lh_qw));
	const GEN_FLT x48 = (x44 * lh_qk) + (x46 * lh_qw) + (-1 * x45 * lh_qi);
	const GEN_FLT x49 = -2 * (lh_qi * lh_qi);
	const GEN_FLT x50 = x4 * lh_qw;
	const GEN_FLT x51 = x2 * lh_qj;
	const GEN_FLT x52 = (x13 * lh_qj) + (-1 * x21 * lh_qk) + (x18 * lh_qw);
	const GEN_FLT x53 = (x27 * lh_qj) + (-1 * x32 * lh_qk) + (x30 * lh_qw);
	const GEN_FLT x54 = (x38 * lh_qj) + (-1 * x40 * lh_qk) + (x36 * lh_qw);
	const GEN_FLT x55 = 2 * ((x45 * lh_qj) + (-1 * x46 * lh_qk) + (x44 * lh_qw));
	out[0] = x1 + x0;
	out[1] = x5 + (-1 * x3);
	out[2] = x8 + x6;
	out[3] = (x7 * x22) + x18 + (-1 * x2 * x23);
	out[4] = x30 + (x33 * lh_qj) + (-1 * x2 * x34);
	out[5] = x36 + (x7 * x41) + (-1 * x2 * x42);
	out[6] = x44 + (x47 * lh_qj) + (-1 * x2 * x48);
	out[7] = x3 + x5;
	out[8] = 1 + x0 + x49;
	out[9] = x51 + (-1 * x50);
	out[10] = x21 + (x2 * x52) + (-1 * x4 * x22);
	out[11] = x32 + (x2 * x53) + (-1 * x33 * lh_qi);
	out[12] = (x2 * x54) + x40 + (-1 * x4 * x41);
	out[13] = x46 + (x55 * lh_qk) + (-1 * x47 * lh_qi);
	out[14] = x6 + (-1 * x8);
	out[15] = x50 + x51;
	out[16] = x1 + x49;
	out[17] = x13 + (x4 * x23) + (-1 * x7 * x52);
	out[18] = x27 + (x4 * x34) + (-1 * x7 * x53);
	out[19] = x38 + (x4 * x42) + (-1 * x7 * x54);
	out[20] = x45 + (x4 * x48) + (-1 * x55 * lh_qj);
}

// Jacobian of sensor_to_world wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_sensor_to_world_jac_sensor_pt(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
													 const SurvivePose *lh_p) {
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
	const GEN_FLT x0 = -2 * (obj_qj * obj_qj);
	const GEN_FLT x1 = 1 + (-2 * (obj_qk * obj_qk));
	const GEN_FLT x2 = x1 + x0;
	const GEN_FLT x3 = 2 * obj_qw;
	const GEN_FLT x4 = x3 * obj_qj;
	const GEN_FLT x5 = 2 * obj_qk;
	const GEN_FLT x6 = x5 * obj_qi;
	const GEN_FLT x7 = x6 + (-1 * x4);
	const GEN_FLT x8 = 2 * obj_qj * obj_qi;
	const GEN_FLT x9 = x5 * obj_qw;
	const GEN_FLT x10 = x9 + x8;
	const GEN_FLT x11 = 2 * ((x10 * lh_qi) + (-1 * x2 * lh_qj) + (x7 * lh_qw));
	const GEN_FLT x12 = (x2 * lh_qk) + (-1 * x7 * lh_qi) + (x10 * lh_qw);
	const GEN_FLT x13 = 2 * lh_qk;
	const GEN_FLT x14 = x8 + (-1 * x9);
	const GEN_FLT x15 = x5 * obj_qj;
	const GEN_FLT x16 = x3 * obj_qi;
	const GEN_FLT x17 = x16 + x15;
	const GEN_FLT x18 = -2 * (obj_qi * obj_qi);
	const GEN_FLT x19 = x1 + x18;
	const GEN_FLT x20 = (x19 * lh_qi) + (-1 * x14 * lh_qj) + (x17 * lh_qw);
	const GEN_FLT x21 = 2 * lh_qj;
	const GEN_FLT x22 = (x14 * lh_qk) + (-1 * x17 * lh_qi) + (x19 * lh_qw);
	const GEN_FLT x23 = x4 + x6;
	const GEN_FLT x24 = 1 + x18 + x0;
	const GEN_FLT x25 = x15 + (-1 * x16);
	const GEN_FLT x26 = (-1 * x23 * lh_qj) + (x25 * lh_qi) + (x24 * lh_qw);
	const GEN_FLT x27 = (x23 * lh_qk) + (-1 * x24 * lh_qi) + (x25 * lh_qw);
	const GEN_FLT x28 = (x7 * lh_qj) + (x2 * lh_qw) + (-1 * x10 * lh_qk);
	const GEN_FLT x29 = (x17 * lh_qj) + (x14 * lh_qw) + (-1 * x19 * lh_qk);
	const GEN_FLT x30 = 2 * lh_qi;
	const GEN_FLT x31 = (-1 * x25 * lh_qk) + (x24 * lh_qj) + (x23 * lh_qw);
	out[0] = x2 + (x11 * lh_qj) + (-1 * x13 * x12);
	out[1] = x14 + (x20 * x21) + (-1 * x22 * x13);
	out[2] = x23 + (x21 * x26) + (-1 * x27 * x13);
	out[3] = x10 + (x28 * x13) + (-1 * x11 * lh_qi);
	out[4] = x19 + (x29 * x13) + (-1 * x30 * x20);
	out[5] = x25 + (x31 * x13) + (-1 * x30 * x26);
	out[6] = x7 + (x30 * x12) + (-1 * x21 * x28);
	out[7] = x17 + (x30 * x22) + (-1 * x21 * x29);
	out[8] = x24 + (x30 * x27) + (-1 * x31 * x21);
}

// Jacobian of sensor_to_world wrt [lh_px, lh_py, lh_pz, lh_qw, lh_qi, lh_qj, lh_qk]
static inline void gen_sensor_to_world_jac_lh_p(FLT *out, const SurvivePose *obj_p, const FLT *sensor_pt,
												const SurvivePose *lh_p) {
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
	const GEN_FLT x0 = (-1 * obj_qj * sensor_x) + (obj_qw * sensor_z) + (obj_qi * sensor_y);
	const GEN_FLT x1 = (obj_qw * sensor_x) + (-1 * obj_qk * sensor_y) + (obj_qj * sensor_z);
	const GEN_FLT x2 = 2 * ((x1 * obj_qk) + (-1 * x0 * obj_qi));
	const GEN_FLT x3 = x2 + obj_py + sensor_y;
	const GEN_FLT x4 = 2 * lh_qk;
	const GEN_FLT x5 = x4 * x3;
	const GEN_FLT x6 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x7 = 2 * ((x6 * obj_qi) + (-1 * x1 * obj_qj));
	const GEN_FLT x8 = x7 + obj_pz + sensor_z;
	const GEN_FLT x9 = x8 * lh_qj;
	const GEN_FLT x10 = (2 * x9) + (-1 * x5);
	const GEN_FLT x11 = (-1 * obj_pz) + (-1 * sensor_z) + (-1 * x7);
	const GEN_FLT x12 = 2 * lh_qj;
	const GEN_FLT x13 = 2 * ((x0 * obj_qj) + (-1 * x6 * obj_qk));
	const GEN_FLT x14 = (-1 * obj_px) + (-1 * sensor_x) + (-1 * x13);
	const GEN_FLT x15 = 2 * lh_qw;
	const GEN_FLT x16 = x8 * x15;
	const GEN_FLT x17 = obj_px + sensor_x + x13;
	const GEN_FLT x18 = x12 * x17;
	const GEN_FLT x19 = 2 * lh_qi;
	const GEN_FLT x20 = (x3 * x19) + (-1 * x18);
	const GEN_FLT x21 = x3 * x15;
	const GEN_FLT x22 = x8 * x19;
	const GEN_FLT x23 = x17 * lh_qk;
	const GEN_FLT x24 = (2 * x23) + (-1 * x22);
	const GEN_FLT x25 = x15 * x17;
	const GEN_FLT x26 = (-1 * sensor_y) + (-1 * obj_py) + (-1 * x2);
	out[0] = 1;
	out[1] = 0;
	out[2] = 0;
	out[3] = x10;
	out[4] = (x3 * x12) + (-1 * x4 * x11);
	out[5] = (x14 * x12) + x20 + x16;
	out[6] = (-4 * x23) + (-1 * x21) + x22;
	out[7] = 0;
	out[8] = 1;
	out[9] = 0;
	out[10] = x24;
	out[11] = (-4 * x3 * lh_qi) + x18 + (-1 * x16);
	out[12] = (x4 * x8) + (-1 * x14 * x19);
	out[13] = x25 + x10 + (x4 * x26);
	out[14] = 0;
	out[15] = 0;
	out[16] = 1;
	out[17] = x20;
	out[18] = x24 + x21 + (x11 * x19);
	out[19] = (-4 * x9) + (-1 * x25) + x5;
	out[20] = (x19 * x17) + (-1 * x26 * x12);
}

static inline void gen_cross(FLT *out, const FLT *sensor_pt, const FLT *axis_angle) {
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];

	out[0] = (aa_z * sensor_y) + (-1 * aa_y * sensor_z);
	out[1] = (aa_x * sensor_z) + (-1 * aa_z * sensor_x);
	out[2] = (aa_y * sensor_x) + (-1 * aa_x * sensor_y);
}

// Jacobian of cross wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_cross_jac_sensor_pt(FLT *out, const FLT *sensor_pt, const FLT *axis_angle) {
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];

	out[0] = 0;
	out[1] = aa_z;
	out[2] = -1 * aa_y;
	out[3] = -1 * aa_z;
	out[4] = 0;
	out[5] = aa_x;
	out[6] = aa_y;
	out[7] = -1 * aa_x;
	out[8] = 0;
}

// Jacobian of cross wrt [aa_x, aa_y, aa_z]
static inline void gen_cross_jac_axis_angle(FLT *out, const FLT *sensor_pt, const FLT *axis_angle) {
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];

	out[0] = 0;
	out[1] = -1 * sensor_z;
	out[2] = sensor_y;
	out[3] = sensor_z;
	out[4] = 0;
	out[5] = -1 * sensor_x;
	out[6] = -1 * sensor_y;
	out[7] = sensor_x;
	out[8] = 0;
}

static inline FLT gen_obj2world_aa_up_err(const FLT *axis_angle, const FLT *sensor_pt) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = aa_z * aa_z;
	const GEN_FLT x1 = 1e-10 + (aa_y * aa_y) + x0 + (aa_x * aa_x);
	const GEN_FLT x2 = sqrt(x1);
	const GEN_FLT x3 = cos(x2);
	const GEN_FLT x4 = (1. / x1) * (1 + (-1 * x3));
	const GEN_FLT x5 = (1. / x2) * sin(x2);
	const GEN_FLT x6 = x4 * aa_z;
	return 1 + (-1 * ((((x6 * aa_y) + (x5 * aa_x)) * sensor_y) + ((x3 + (x0 * x4)) * sensor_z) +
					  (((x6 * aa_x) + (-1 * x5 * aa_y)) * sensor_x)));
}

// Jacobian of obj2world_aa_up_err wrt [aa_x, aa_y, aa_z]
static inline void gen_obj2world_aa_up_err_jac_axis_angle(FLT *out, const FLT *axis_angle, const FLT *sensor_pt) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = aa_x * aa_x;
	const GEN_FLT x1 = aa_z * aa_z;
	const GEN_FLT x2 = aa_y * aa_y;
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = 2 * (1. / (x3 * x3)) * x6;
	const GEN_FLT x8 = x7 * aa_z;
	const GEN_FLT x9 = 1. / x3;
	const GEN_FLT x10 = x6 * x9;
	const GEN_FLT x11 = x10 * aa_z;
	const GEN_FLT x12 = x5 * x9;
	const GEN_FLT x13 = x12 * aa_x;
	const GEN_FLT x14 = x13 * aa_y;
	const GEN_FLT x15 = sin(x4);
	const GEN_FLT x16 = (1. / (x3 * sqrt(x3))) * x15;
	const GEN_FLT x17 = x0 * x16;
	const GEN_FLT x18 = x16 * aa_x;
	const GEN_FLT x19 = x18 * aa_y;
	const GEN_FLT x20 = (1. / x4) * x15;
	const GEN_FLT x21 = aa_y * aa_z;
	const GEN_FLT x22 = x7 * aa_x;
	const GEN_FLT x23 = (x21 * x18) + (-1 * x22 * x21);
	const GEN_FLT x24 = (-1 * x1 * x22) + (x1 * x18);
	const GEN_FLT x25 = x2 * x16;
	const GEN_FLT x26 = x1 * aa_y;
	const GEN_FLT x27 = (-1 * x7 * x26) + (x26 * x16);
	const GEN_FLT x28 = aa_z * aa_z * aa_z;
	out[0] = (-1 * (x24 + (-1 * x20 * aa_x)) * sensor_z) +
			 (-1 * (x19 + (x17 * aa_z) + (-1 * x14) + (-1 * x0 * x8) + x11) * sensor_x) +
			 (-1 * (x20 + x23 + (x0 * x12) + (-1 * x17)) * sensor_y);
	out[1] = (-1 * ((-1 * x20) + x25 + x23 + (-1 * x2 * x12)) * sensor_x) +
			 (-1 * (x27 + (-1 * x20 * aa_y)) * sensor_z) +
			 (-1 * ((-1 * x19) + (x25 * aa_z) + x14 + (-1 * x2 * x8) + x11) * sensor_y);
	out[2] = (-1 * ((-1 * x7 * x28) + (-1 * x20 * aa_z) + (2 * x11) + (x28 * x16)) * sensor_z) +
			 (-1 * (x24 + (x21 * x16) + (x10 * aa_x) + (-1 * x21 * x12)) * sensor_x) +
			 (-1 * (x27 + (x13 * aa_z) + (-1 * x18 * aa_z) + (x10 * aa_y)) * sensor_y);
}

// Jacobian of obj2world_aa_up_err wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_obj2world_aa_up_err_jac_sensor_pt(FLT *out, const FLT *axis_angle, const FLT *sensor_pt) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = aa_z * aa_z;
	const GEN_FLT x1 = 1e-10 + (aa_y * aa_y) + x0 + (aa_x * aa_x);
	const GEN_FLT x2 = sqrt(x1);
	const GEN_FLT x3 = -1 * cos(x2);
	const GEN_FLT x4 = (1. / x1) * (1 + x3);
	const GEN_FLT x5 = x4 * aa_z;
	const GEN_FLT x6 = (1. / x2) * sin(x2);
	out[0] = (x6 * aa_y) + (-1 * x5 * aa_x);
	out[1] = (-1 * x6 * aa_x) + (-1 * x5 * aa_y);
	out[2] = (-1 * x0 * x4) + x3;
}

static inline FLT gen_obj2world_up_err(const FLT *q1, const FLT *sensor_pt) {
	const GEN_FLT obj_qw = q1[0];
	const GEN_FLT obj_qi = q1[1];
	const GEN_FLT obj_qj = q1[2];
	const GEN_FLT obj_qk = q1[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];

	return 1 + (-1 * (sensor_z +
					  (2 * ((obj_qi * ((obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y))) +
							(-1 * obj_qj * ((obj_qw * sensor_x) + (-1 * obj_qk * sensor_y) + (obj_qj * sensor_z)))))));
}

// Jacobian of obj2world_up_err wrt [obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_obj2world_up_err_jac_q1(FLT *out, const FLT *q1, const FLT *sensor_pt) {
	const GEN_FLT obj_qw = q1[0];
	const GEN_FLT obj_qi = q1[1];
	const GEN_FLT obj_qj = q1[2];
	const GEN_FLT obj_qk = q1[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = 2 * obj_qi;
	const GEN_FLT x1 = 2 * obj_qj;
	const GEN_FLT x2 = 2 * obj_qk;
	const GEN_FLT x3 = 2 * obj_qw;
	const GEN_FLT x4 = 4 * sensor_z;
	out[0] = (x1 * sensor_x) + (-1 * x0 * sensor_y);
	out[1] = (x4 * obj_qi) + (-1 * x2 * sensor_x) + (-1 * x3 * sensor_y);
	out[2] = (-1 * x2 * sensor_y) + (x3 * sensor_x) + (x4 * obj_qj);
	out[3] = (-1 * x1 * sensor_y) + (-1 * x0 * sensor_x);
}

// Jacobian of obj2world_up_err wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_obj2world_up_err_jac_sensor_pt(FLT *out, const FLT *q1, const FLT *sensor_pt) {
	const GEN_FLT obj_qw = q1[0];
	const GEN_FLT obj_qi = q1[1];
	const GEN_FLT obj_qj = q1[2];
	const GEN_FLT obj_qk = q1[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = 2 * obj_qk;
	const GEN_FLT x1 = 2 * obj_qw;
	out[0] = (x1 * obj_qj) + (-1 * x0 * obj_qi);
	out[1] = (-1 * x0 * obj_qj) + (-1 * x1 * obj_qi);
	out[2] = -1 + (2 * (obj_qj * obj_qj)) + (2 * (obj_qi * obj_qi));
}

static inline FLT gen_world2lh_aa_up_err(const FLT *axis_angle, const FLT *sensor_pt) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = aa_z * aa_z;
	const GEN_FLT x1 = 1e-10 + (aa_y * aa_y) + x0 + (aa_x * aa_x);
	const GEN_FLT x2 = sqrt(x1);
	const GEN_FLT x3 = cos(x2);
	const GEN_FLT x4 = (1. / x1) * (1 + (-1 * x3));
	const GEN_FLT x5 = (1. / x2) * sin(x2);
	const GEN_FLT x6 = x4 * aa_z;
	return 1 + (-1 * ((((x6 * aa_y) + (-1 * x5 * aa_x)) * sensor_y) + ((x3 + (x0 * x4)) * sensor_z) +
					  (((x6 * aa_x) + (x5 * aa_y)) * sensor_x)));
}

// Jacobian of world2lh_aa_up_err wrt [aa_x, aa_y, aa_z]
static inline void gen_world2lh_aa_up_err_jac_axis_angle(FLT *out, const FLT *axis_angle, const FLT *sensor_pt) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = aa_x * aa_x;
	const GEN_FLT x1 = aa_z * aa_z;
	const GEN_FLT x2 = aa_y * aa_y;
	const GEN_FLT x3 = 1e-10 + x2 + x1 + x0;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = cos(x4);
	const GEN_FLT x6 = 1 + (-1 * x5);
	const GEN_FLT x7 = 2 * (1. / (x3 * x3)) * x6;
	const GEN_FLT x8 = x7 * aa_z;
	const GEN_FLT x9 = 1. / x3;
	const GEN_FLT x10 = x6 * x9;
	const GEN_FLT x11 = x10 * aa_z;
	const GEN_FLT x12 = x5 * x9;
	const GEN_FLT x13 = x12 * aa_x;
	const GEN_FLT x14 = x13 * aa_y;
	const GEN_FLT x15 = sin(x4);
	const GEN_FLT x16 = (1. / (x3 * sqrt(x3))) * x15;
	const GEN_FLT x17 = x0 * x16;
	const GEN_FLT x18 = x16 * aa_x;
	const GEN_FLT x19 = x18 * aa_y;
	const GEN_FLT x20 = (1. / x4) * x15;
	const GEN_FLT x21 = aa_y * aa_z;
	const GEN_FLT x22 = x7 * aa_x;
	const GEN_FLT x23 = (x21 * x18) + (-1 * x22 * x21);
	const GEN_FLT x24 = (-1 * x1 * x22) + (x1 * x18);
	const GEN_FLT x25 = x2 * x16;
	const GEN_FLT x26 = x1 * aa_y;
	const GEN_FLT x27 = (-1 * x7 * x26) + (x26 * x16);
	const GEN_FLT x28 = aa_z * aa_z * aa_z;
	out[0] = (-1 * (x24 + (-1 * x20 * aa_x)) * sensor_z) +
			 (-1 * ((-1 * x19) + (x17 * aa_z) + x14 + (-1 * x0 * x8) + x11) * sensor_x) +
			 (-1 * ((-1 * x20) + x23 + (-1 * x0 * x12) + x17) * sensor_y);
	out[1] = (-1 * (x27 + (-1 * x20 * aa_y)) * sensor_z) + (-1 * (x20 + (-1 * x25) + x23 + (x2 * x12)) * sensor_x) +
			 (-1 * (x19 + (x25 * aa_z) + (-1 * x14) + (-1 * x2 * x8) + x11) * sensor_y);
	out[2] = (-1 * ((-1 * x7 * x28) + (-1 * x20 * aa_z) + (2 * x11) + (x28 * x16)) * sensor_z) +
			 (-1 * (x24 + (-1 * x21 * x16) + (x10 * aa_x) + (x21 * x12)) * sensor_x) +
			 (-1 * (x27 + (-1 * x13 * aa_z) + (x18 * aa_z) + (x10 * aa_y)) * sensor_y);
}

// Jacobian of world2lh_aa_up_err wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_world2lh_aa_up_err_jac_sensor_pt(FLT *out, const FLT *axis_angle, const FLT *sensor_pt) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = aa_z * aa_z;
	const GEN_FLT x1 = 1e-10 + (aa_y * aa_y) + x0 + (aa_x * aa_x);
	const GEN_FLT x2 = sqrt(x1);
	const GEN_FLT x3 = -1 * cos(x2);
	const GEN_FLT x4 = (1. / x1) * (1 + x3);
	const GEN_FLT x5 = x4 * aa_z;
	const GEN_FLT x6 = (1. / x2) * sin(x2);
	out[0] = (-1 * x6 * aa_y) + (-1 * x5 * aa_x);
	out[1] = (x6 * aa_x) + (-1 * x5 * aa_y);
	out[2] = (-1 * x0 * x4) + x3;
}

static inline FLT gen_world2lh_up_err(const FLT *q1, const FLT *sensor_pt) {
	const GEN_FLT obj_qw = q1[0];
	const GEN_FLT obj_qi = q1[1];
	const GEN_FLT obj_qj = q1[2];
	const GEN_FLT obj_qk = q1[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];

	return 1 + (-1 * (sensor_z +
					  (2 * ((obj_qi * ((obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y))) +
							(-1 * obj_qj * ((obj_qw * sensor_x) + (-1 * obj_qk * sensor_y) + (obj_qj * sensor_z)))))));
}

// Jacobian of world2lh_up_err wrt [obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_world2lh_up_err_jac_q1(FLT *out, const FLT *q1, const FLT *sensor_pt) {
	const GEN_FLT obj_qw = q1[0];
	const GEN_FLT obj_qi = q1[1];
	const GEN_FLT obj_qj = q1[2];
	const GEN_FLT obj_qk = q1[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = 2 * obj_qi;
	const GEN_FLT x1 = 2 * obj_qj;
	const GEN_FLT x2 = 2 * obj_qk;
	const GEN_FLT x3 = 2 * obj_qw;
	const GEN_FLT x4 = 4 * sensor_z;
	out[0] = (x1 * sensor_x) + (-1 * x0 * sensor_y);
	out[1] = (x4 * obj_qi) + (-1 * x2 * sensor_x) + (-1 * x3 * sensor_y);
	out[2] = (-1 * x2 * sensor_y) + (x3 * sensor_x) + (x4 * obj_qj);
	out[3] = (-1 * x1 * sensor_y) + (-1 * x0 * sensor_x);
}

// Jacobian of world2lh_up_err wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_world2lh_up_err_jac_sensor_pt(FLT *out, const FLT *q1, const FLT *sensor_pt) {
	const GEN_FLT obj_qw = q1[0];
	const GEN_FLT obj_qi = q1[1];
	const GEN_FLT obj_qj = q1[2];
	const GEN_FLT obj_qk = q1[3];
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT x0 = 2 * obj_qk;
	const GEN_FLT x1 = 2 * obj_qw;
	out[0] = (x1 * obj_qj) + (-1 * x0 * obj_qi);
	out[1] = (-1 * x0 * obj_qj) + (-1 * x1 * obj_qi);
	out[2] = -1 + (2 * (obj_qj * obj_qj)) + (2 * (obj_qi * obj_qi));
}

static inline void gen_apply_ang_velocity(FLT *out, const FLT *axis_angle, const FLT time, const FLT *q) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];
	const GEN_FLT x0 = time * time;
	const GEN_FLT x1 = x0 * (aa_z * aa_z);
	const GEN_FLT x2 = x0 * (aa_x * aa_x);
	const GEN_FLT x3 = x0 * (aa_y * aa_y);
	const GEN_FLT x4 = 1e-10 + x3 + x1 + x2;
	const GEN_FLT x5 = sqrt(x4);
	const GEN_FLT x6 = 0.5 * x5;
	const GEN_FLT x7 = sin(x6);
	const GEN_FLT x8 = (1. / x4) * (x7 * x7);
	const GEN_FLT x9 = cos(x6);
	const GEN_FLT x10 = 1. / sqrt(1e-11 + (x2 * x8) + (x9 * x9) + (x1 * x8) + (x3 * x8));
	const GEN_FLT x11 = (1. / x5) * x7 * x10 * time;
	const GEN_FLT x12 = x11 * obj_qj;
	const GEN_FLT x13 = x9 * x10;
	const GEN_FLT x14 = x11 * aa_z;
	const GEN_FLT x15 = x11 * aa_x;
	const GEN_FLT x16 = x11 * aa_y;
	out[0] = (-1 * x12 * aa_y) + (-1 * x15 * obj_qi) + (-1 * x14 * obj_qk) + (x13 * obj_qw);
	out[1] = (x15 * obj_qw) + (x13 * obj_qi) + (-1 * x12 * aa_z) + (x16 * obj_qk);
	out[2] = (-1 * x15 * obj_qk) + (x16 * obj_qw) + (x14 * obj_qi) + (x13 * obj_qj);
	out[3] = (x12 * aa_x) + (x13 * obj_qk) + (x14 * obj_qw) + (-1 * x16 * obj_qi);
}

// Jacobian of apply_ang_velocity wrt [aa_x, aa_y, aa_z]
static inline void gen_apply_ang_velocity_jac_axis_angle(FLT *out, const FLT *axis_angle, const FLT time,
														 const FLT *q) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];
	const GEN_FLT x0 = aa_x * aa_x;
	const GEN_FLT x1 = time * time;
	const GEN_FLT x2 = aa_z * aa_z;
	const GEN_FLT x3 = x2 * x1;
	const GEN_FLT x4 = x0 * x1;
	const GEN_FLT x5 = aa_y * aa_y;
	const GEN_FLT x6 = x1 * x5;
	const GEN_FLT x7 = 1e-10 + x6 + x3 + x4;
	const GEN_FLT x8 = 1. / (x7 * sqrt(x7));
	const GEN_FLT x9 = time * time * time;
	const GEN_FLT x10 = x8 * x9;
	const GEN_FLT x11 = x0 * x10;
	const GEN_FLT x12 = sqrt(x7);
	const GEN_FLT x13 = 0.5 * x12;
	const GEN_FLT x14 = sin(x13);
	const GEN_FLT x15 = 1. / x7;
	const GEN_FLT x16 = x14 * x14;
	const GEN_FLT x17 = x15 * x16;
	const GEN_FLT x18 = cos(x13);
	const GEN_FLT x19 = 1e-11 + (x18 * x18) + (x4 * x17) + (x3 * x17) + (x6 * x17);
	const GEN_FLT x20 = 1. / sqrt(x19);
	const GEN_FLT x21 = x20 * x14;
	const GEN_FLT x22 = x21 * obj_qi;
	const GEN_FLT x23 = x1 * aa_x;
	const GEN_FLT x24 = 2 * x17;
	const GEN_FLT x25 = time * time * time * time;
	const GEN_FLT x26 = x25 * (aa_x * aa_x * aa_x);
	const GEN_FLT x27 = 2 * (1. / (x7 * x7)) * x16;
	const GEN_FLT x28 = 1.0 * x14 * x18;
	const GEN_FLT x29 = x8 * x28;
	const GEN_FLT x30 = x25 * x27;
	const GEN_FLT x31 = x2 * x30;
	const GEN_FLT x32 = x25 * x29;
	const GEN_FLT x33 = x2 * x32;
	const GEN_FLT x34 = x5 * x32;
	const GEN_FLT x35 = x5 * x30;
	const GEN_FLT x36 = 1. / x12;
	const GEN_FLT x37 = x36 * x28;
	const GEN_FLT x38 = 1.0 / 2.0 * (1. / (x19 * sqrt(x19)));
	const GEN_FLT x39 = ((-1 * x35 * aa_x) + (-1 * x26 * x27) + (-1 * x37 * x23) + (x24 * x23) + (-1 * x31 * aa_x) +
						 (x34 * aa_x) + (x29 * x26) + (x33 * aa_x)) *
						x38;
	const GEN_FLT x40 = x39 * x18;
	const GEN_FLT x41 = x21 * obj_qw;
	const GEN_FLT x42 = 0.5 * x36;
	const GEN_FLT x43 = x42 * x23;
	const GEN_FLT x44 = x36 * time;
	const GEN_FLT x45 = x44 * x14;
	const GEN_FLT x46 = x45 * x39;
	const GEN_FLT x47 = x46 * aa_y;
	const GEN_FLT x48 = x46 * aa_z;
	const GEN_FLT x49 = x44 * x21;
	const GEN_FLT x50 = x49 * obj_qi;
	const GEN_FLT x51 = -1 * x50;
	const GEN_FLT x52 = x18 * obj_qj;
	const GEN_FLT x53 = 0.5 * x9 * x20 * x15;
	const GEN_FLT x54 = x53 * aa_x;
	const GEN_FLT x55 = x54 * aa_y;
	const GEN_FLT x56 = x52 * x55;
	const GEN_FLT x57 = x21 * obj_qj;
	const GEN_FLT x58 = x10 * aa_x;
	const GEN_FLT x59 = x58 * aa_y;
	const GEN_FLT x60 = x57 * x59;
	const GEN_FLT x61 = x46 * aa_x;
	const GEN_FLT x62 = x18 * obj_qi;
	const GEN_FLT x63 = x0 * x53;
	const GEN_FLT x64 = x21 * obj_qk;
	const GEN_FLT x65 = x58 * aa_z;
	const GEN_FLT x66 = x18 * obj_qk;
	const GEN_FLT x67 = x54 * aa_z;
	const GEN_FLT x68 = (-1 * x67 * x66) + (x64 * x65);
	const GEN_FLT x69 = x57 * x44;
	const GEN_FLT x70 = -1 * x69;
	const GEN_FLT x71 = x0 * x30;
	const GEN_FLT x72 = x0 * x32;
	const GEN_FLT x73 = x1 * x37;
	const GEN_FLT x74 = x1 * x24;
	const GEN_FLT x75 = aa_y * aa_y * aa_y;
	const GEN_FLT x76 = ((x75 * x32) + (-1 * x75 * x30) + (x33 * aa_y) + (x72 * aa_y) + (-1 * x71 * aa_y) +
						 (-1 * x31 * aa_y) + (-1 * x73 * aa_y) + (x74 * aa_y)) *
						x38;
	const GEN_FLT x77 = x76 * x18;
	const GEN_FLT x78 = x76 * x45;
	const GEN_FLT x79 = x78 * obj_qj;
	const GEN_FLT x80 = x64 * aa_y;
	const GEN_FLT x81 = x10 * aa_z;
	const GEN_FLT x82 = x80 * x81;
	const GEN_FLT x83 = x78 * aa_z;
	const GEN_FLT x84 = x53 * aa_y * aa_z;
	const GEN_FLT x85 = x84 * x66;
	const GEN_FLT x86 = x5 * x53;
	const GEN_FLT x87 = x1 * x42;
	const GEN_FLT x88 = x87 * x41;
	const GEN_FLT x89 = x5 * x10;
	const GEN_FLT x90 = x78 * aa_x;
	const GEN_FLT x91 = (x59 * x22) + (-1 * x62 * x55);
	const GEN_FLT x92 = x62 * x67;
	const GEN_FLT x93 = aa_z * aa_z * aa_z;
	const GEN_FLT x94 = ((-1 * x93 * x30) + (-1 * x35 * aa_z) + (x74 * aa_z) + (-1 * x73 * aa_z) + (x72 * aa_z) +
						 (-1 * x71 * aa_z) + (x34 * aa_z) + (x93 * x32)) *
						x38;
	const GEN_FLT x95 = x94 * x45;
	const GEN_FLT x96 = x95 * aa_y;
	const GEN_FLT x97 = x94 * x18;
	const GEN_FLT x98 = x49 * obj_qk;
	const GEN_FLT x99 = -1 * x98;
	const GEN_FLT x100 = x95 * aa_x;
	const GEN_FLT x101 = x95 * aa_z;
	const GEN_FLT x102 = x2 * x53;
	const GEN_FLT x103 = x2 * x10;
	const GEN_FLT x104 = x21 * x103;
	const GEN_FLT x105 = x65 * x22;
	const GEN_FLT x106 = x81 * aa_y;
	const GEN_FLT x107 = (-1 * x84 * x52) + (x57 * x106);
	const GEN_FLT x108 = x67 * x52;
	const GEN_FLT x109 = x49 * obj_qw;
	const GEN_FLT x110 = x80 * x58;
	const GEN_FLT x111 = x65 * x57;
	const GEN_FLT x112 = x66 * x55;
	const GEN_FLT x113 = x18 * obj_qw;
	const GEN_FLT x114 = x78 * aa_y;
	const GEN_FLT x115 = x86 * x18;
	const GEN_FLT x116 = x54 * x113;
	const GEN_FLT x117 = (x116 * aa_y) + (-1 * x59 * x41);
	const GEN_FLT x118 = x22 * aa_z;
	const GEN_FLT x119 = (x116 * aa_z) + (-1 * x65 * x41);
	const GEN_FLT x120 = x57 * x42;
	const GEN_FLT x121 = x1 * x120;
	const GEN_FLT x122 = x84 * x62;
	const GEN_FLT x123 = x10 * x118 * aa_y;
	const GEN_FLT x124 = (-1 * x41 * x106) + (x84 * x113);
	const GEN_FLT x125 = aa_z * obj_qw;
	out[0] = (x61 * obj_qi) + x60 + (-1 * x56) + x51 + (-1 * x40 * obj_qw) + x68 + (-1 * x63 * x62) + (x47 * obj_qj) +
			 (x22 * x11) + (x48 * obj_qk) + (-1 * x41 * x43);
	out[1] = (x90 * obj_qi) + (x89 * x57) + (-1 * x88 * aa_y) + (-1 * x85) + x91 + x70 + (x79 * aa_y) +
			 (-1 * x77 * obj_qw) + x82 + (-1 * x86 * x52) + (x83 * obj_qk);
	out[2] = x105 + (-1 * x66 * x102) + (x101 * obj_qk) + (x100 * obj_qi) + (-1 * x88 * aa_z) + (-1 * x92) +
			 (x104 * obj_qk) + (-1 * x97 * obj_qw) + x99 + x107 + (x96 * obj_qj);
	out[3] = (x63 * x113) + x109 + (x48 * obj_qj) + (-1 * x108) + (-1 * x47 * obj_qk) + (-1 * x62 * x39) +
			 (-1 * x43 * x22) + (-1 * x110) + x111 + (-1 * x41 * x11) + (-1 * x61 * obj_qw) + x112;
	out[4] = x117 + (-1 * x90 * obj_qw) + (-1 * x114 * obj_qk) + x98 + x107 + (-1 * x89 * x64) +
			 (-1 * x87 * x22 * aa_y) + (-1 * x76 * x62) + (x115 * obj_qk) + (x79 * aa_z);
	out[5] = x119 + (-1 * x87 * x118) + (-1 * x96 * obj_qk) + (-1 * x62 * x94) + (x57 * x103) + (-1 * x82) + x70 +
			 (-1 * x52 * x102) + (-1 * x100 * obj_qw) + (x101 * obj_qj) + x85;
	out[6] = (-1 * x47 * obj_qw) + (-1 * x63 * x66) + (x61 * obj_qk) + (x64 * x11) + (-1 * x52 * x39) + x99 + x117 +
			 x92 + (-1 * x48 * obj_qi) + (-1 * x23 * x120) + (-1 * x105);
	out[7] = (x90 * obj_qk) + (-1 * x112) + (x115 * obj_qw) + (-1 * x77 * obj_qj) + x122 + x109 + (-1 * x114 * obj_qw) +
			 (-1 * x83 * obj_qi) + (-1 * x121 * aa_y) + (-1 * x123) + x110 + (-1 * x89 * x41);
	out[8] = x68 + (-1 * x96 * obj_qw) + (x100 * obj_qk) + x124 + (-1 * x101 * obj_qi) + (-1 * x104 * obj_qi) +
			 (-1 * x97 * obj_qj) + x50 + (x62 * x102) + (-1 * x121 * aa_z);
	out[9] = x91 + x119 + (x63 * x52) + (-1 * x61 * obj_qj) + (-1 * x57 * x11) + (x47 * obj_qi) + x69 +
			 (-1 * x46 * x125) + (-1 * x64 * x43) + (-1 * x40 * obj_qk);
	out[10] = (x114 * obj_qi) + (-1 * x79 * aa_x) + (-1 * x60) + (-1 * x80 * x87) + x56 + (-1 * x77 * obj_qk) +
			  (-1 * x86 * x62) + x51 + x124 + (x89 * x22) + (-1 * x78 * x125);
	out[11] = (-1 * x100 * obj_qj) + (-1 * x87 * x64 * aa_z) + x108 + (-1 * x95 * x125) + (x96 * obj_qi) + (-1 * x122) +
			  (-1 * x97 * obj_qk) + (-1 * x111) + x109 + (-1 * x41 * x103) + (x102 * x113) + x123;
}

// Jacobian of apply_ang_velocity wrt [time]
static inline void gen_apply_ang_velocity_jac_time(FLT *out, const FLT *axis_angle, const FLT time, const FLT *q) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];
	const GEN_FLT x0 = time * time;
	const GEN_FLT x1 = aa_z * aa_z;
	const GEN_FLT x2 = x0 * x1;
	const GEN_FLT x3 = aa_x * aa_x;
	const GEN_FLT x4 = x0 * x3;
	const GEN_FLT x5 = aa_y * aa_y;
	const GEN_FLT x6 = x0 * x5;
	const GEN_FLT x7 = 1e-10 + x6 + x2 + x4;
	const GEN_FLT x8 = 1. / x7;
	const GEN_FLT x9 = sqrt(x7);
	const GEN_FLT x10 = 0.5 * x9;
	const GEN_FLT x11 = sin(x10);
	const GEN_FLT x12 = x11 * x11;
	const GEN_FLT x13 = x8 * x12;
	const GEN_FLT x14 = cos(x10);
	const GEN_FLT x15 = 1e-11 + (x4 * x13) + (x14 * x14) + (x2 * x13) + (x6 * x13);
	const GEN_FLT x16 = 1. / sqrt(x15);
	const GEN_FLT x17 = time * obj_qi;
	const GEN_FLT x18 = x17 * x16;
	const GEN_FLT x19 = x18 * aa_x;
	const GEN_FLT x20 = 2 * time;
	const GEN_FLT x21 = x5 * x20;
	const GEN_FLT x22 = x1 * x20;
	const GEN_FLT x23 = x3 * x20;
	const GEN_FLT x24 = x23 + x21 + x22;
	const GEN_FLT x25 = (1. / (x7 * sqrt(x7))) * x11;
	const GEN_FLT x26 = x24 * x25;
	const GEN_FLT x27 = x24 * x14;
	const GEN_FLT x28 = x8 * x27;
	const GEN_FLT x29 = 0.25 * x28;
	const GEN_FLT x30 = 0.5 * x27;
	const GEN_FLT x31 = x30 * x25;
	const GEN_FLT x32 = (1. / x9) * x11;
	const GEN_FLT x33 = (1. / (x7 * x7)) * x24 * x12;
	const GEN_FLT x34 = ((x22 * x13) + (x23 * x13) + (-1 * x6 * x33) + (-1 * x30 * x32) + (x4 * x31) + (-1 * x2 * x33) +
						 (x21 * x13) + (-1 * x4 * x33) + (x6 * x31) + (x2 * x31)) *
						(1. / (x15 * sqrt(x15)));
	const GEN_FLT x35 = x34 * x14;
	const GEN_FLT x36 = 1.0 / 2.0 * x35;
	const GEN_FLT x37 = x32 * x34 * time;
	const GEN_FLT x38 = x37 * aa_y;
	const GEN_FLT x39 = 1.0 / 2.0 * obj_qj;
	const GEN_FLT x40 = x32 * x16;
	const GEN_FLT x41 = x40 * aa_x;
	const GEN_FLT x42 = x40 * aa_y;
	const GEN_FLT x43 = x16 * time;
	const GEN_FLT x44 = x43 * aa_y;
	const GEN_FLT x45 = x29 * obj_qj;
	const GEN_FLT x46 = x44 * x26;
	const GEN_FLT x47 = 1.0 / 2.0 * obj_qk;
	const GEN_FLT x48 = x37 * aa_z;
	const GEN_FLT x49 = x29 * obj_qk;
	const GEN_FLT x50 = x43 * aa_z;
	const GEN_FLT x51 = 0.25 * obj_qw;
	const GEN_FLT x52 = x40 * x24;
	const GEN_FLT x53 = x40 * aa_z;
	const GEN_FLT x54 = x50 * x26;
	const GEN_FLT x55 = x32 * x34 * x17;
	const GEN_FLT x56 = x43 * aa_x;
	const GEN_FLT x57 = 1.0 / 2.0 * obj_qw;
	const GEN_FLT x58 = x57 * x26;
	const GEN_FLT x59 = x51 * x28;
	const GEN_FLT x60 = x59 * x43;
	const GEN_FLT x61 = x37 * aa_x;
	const GEN_FLT x62 = 0.25 * x52;
	const GEN_FLT x63 = x56 * x26;
	const GEN_FLT x64 = 1.0 / 2.0 * aa_z;
	const GEN_FLT x65 = x26 * x18;
	const GEN_FLT x66 = x29 * x18;
	const GEN_FLT x67 = 1.0 / 2.0 * aa_y;
	out[0] = (x54 * x47) + (-1 * x42 * obj_qj) + (1.0 / 2.0 * x55 * aa_x) + (-1 * x29 * x19) + (-1 * x36 * obj_qw) +
			 (1.0 / 2.0 * x26 * x19) + (x47 * x48) + (x38 * x39) + (-1 * x44 * x45) + (x46 * x39) + (-1 * x50 * x49) +
			 (-1 * x53 * obj_qk) + (-1 * x41 * obj_qi) + (-1 * x52 * x51);
	out[1] = (-1 * x62 * obj_qi) + (-1 * x50 * x45) + (-1 * x61 * x57) + (x44 * x49) + (x54 * x39) + (x42 * obj_qk) +
			 (-1 * x53 * obj_qj) + (-1 * x46 * x47) + (x48 * x39) + (x41 * obj_qw) + (-1 * x47 * x38) +
			 (-1 * x36 * obj_qi) + (-1 * x58 * x56) + (x60 * aa_x);
	out[2] = (x61 * x47) + (-1 * x62 * obj_qj) + (x59 * x44) + (-1 * x41 * obj_qk) + (-1 * x36 * obj_qj) +
			 (-1 * x64 * x65) + (x53 * obj_qi) + (-1 * x58 * x44) + (x63 * x47) + (-1 * x57 * x38) + (-1 * x64 * x55) +
			 (x42 * obj_qw) + (x66 * aa_z) + (-1 * x56 * x49);
	out[3] = (-1 * x62 * obj_qk) + (x67 * x65) + (x67 * x55) + (x56 * x45) + (x60 * aa_z) + (-1 * x54 * x57) +
			 (x41 * obj_qj) + (x53 * obj_qw) + (-1 * x63 * x39) + (-1 * x57 * x48) + (-1 * x42 * obj_qi) +
			 (-1 * x66 * aa_y) + (-1 * x47 * x35) + (-1 * x61 * x39);
}

// Jacobian of apply_ang_velocity wrt [obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_apply_ang_velocity_jac_q(FLT *out, const FLT *axis_angle, const FLT time, const FLT *q) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT obj_qw = q[0];
	const GEN_FLT obj_qi = q[1];
	const GEN_FLT obj_qj = q[2];
	const GEN_FLT obj_qk = q[3];
	const GEN_FLT x0 = time * time;
	const GEN_FLT x1 = x0 * (aa_z * aa_z);
	const GEN_FLT x2 = x0 * (aa_x * aa_x);
	const GEN_FLT x3 = x0 * (aa_y * aa_y);
	const GEN_FLT x4 = 1e-10 + x3 + x1 + x2;
	const GEN_FLT x5 = sqrt(x4);
	const GEN_FLT x6 = 0.5 * x5;
	const GEN_FLT x7 = cos(x6);
	const GEN_FLT x8 = sin(x6);
	const GEN_FLT x9 = (1. / x4) * (x8 * x8);
	const GEN_FLT x10 = 1. / sqrt(1e-11 + (x2 * x9) + (x1 * x9) + (x7 * x7) + (x3 * x9));
	const GEN_FLT x11 = x7 * x10;
	const GEN_FLT x12 = (1. / x5) * x8 * x10 * time;
	const GEN_FLT x13 = x12 * aa_x;
	const GEN_FLT x14 = -1 * x13;
	const GEN_FLT x15 = x12 * aa_y;
	const GEN_FLT x16 = -1 * x15;
	const GEN_FLT x17 = x12 * aa_z;
	const GEN_FLT x18 = -1 * x17;
	out[0] = x11;
	out[1] = x14;
	out[2] = x16;
	out[3] = x18;
	out[4] = x13;
	out[5] = x11;
	out[6] = x18;
	out[7] = x15;
	out[8] = x15;
	out[9] = x17;
	out[10] = x11;
	out[11] = x14;
	out[12] = x17;
	out[13] = x16;
	out[14] = x13;
	out[15] = x11;
}

static inline void gen_apply_ang_velocity_aa(FLT *out, const FLT *axis_angle, const FLT time, const FLT *axis_angle2) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT aa2_x = axis_angle2[0];
	const GEN_FLT aa2_y = axis_angle2[1];
	const GEN_FLT aa2_z = axis_angle2[2];
	const GEN_FLT x0 = sqrt(1e-10 + (aa2_y * aa2_y) + (aa2_z * aa2_z) + (aa2_x * aa2_x));
	const GEN_FLT x1 = 1.0 / 2.0 * x0;
	const GEN_FLT x2 = (1. / x0) * sin(x1);
	const GEN_FLT x3 = time * time;
	const GEN_FLT x4 = sqrt(1e-10 + (x3 * (aa_y * aa_y)) + (x3 * (aa_z * aa_z)) + (x3 * (aa_x * aa_x)));
	const GEN_FLT x5 = 1.0 / 2.0 * x4;
	const GEN_FLT x6 = (1. / x4) * sin(x5) * time;
	const GEN_FLT x7 = x2 * x6;
	const GEN_FLT x8 = x7 * aa_z;
	const GEN_FLT x9 = x7 * aa_y;
	const GEN_FLT x10 = cos(x5);
	const GEN_FLT x11 = x2 * x10;
	const GEN_FLT x12 = cos(x1);
	const GEN_FLT x13 = x6 * x12;
	const GEN_FLT x14 = (x11 * aa2_x) + (-1 * x8 * aa2_y) + (x13 * aa_x) + (x9 * aa2_z);
	const GEN_FLT x15 = x7 * aa_x;
	const GEN_FLT x16 = (x11 * aa2_z) + (x13 * aa_z) + (x15 * aa2_y) + (-1 * x9 * aa2_x);
	const GEN_FLT x17 = (x13 * aa_y) + (x11 * aa2_y) + (x8 * aa2_x) + (-1 * x15 * aa2_z);
	const GEN_FLT x18 = 2 * (1. / sqrt(1e-10 + (x17 * x17) + (x16 * x16) + (x14 * x14))) *
						acos((x12 * x10) + (-1 * ((x9 * aa2_y) + (x8 * aa2_z) + (x15 * aa2_x))));
	out[0] = x14 * x18;
	out[1] = x18 * x17;
	out[2] = x18 * x16;
}

// Jacobian of apply_ang_velocity_aa wrt [aa_x, aa_y, aa_z]
static inline void gen_apply_ang_velocity_aa_jac_axis_angle(FLT *out, const FLT *axis_angle, const FLT time,
															const FLT *axis_angle2) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT aa2_x = axis_angle2[0];
	const GEN_FLT aa2_y = axis_angle2[1];
	const GEN_FLT aa2_z = axis_angle2[2];
	const GEN_FLT x0 = time * time;
	const GEN_FLT x1 = aa_z * aa_z;
	const GEN_FLT x2 = aa_x * aa_x;
	const GEN_FLT x3 = aa_y * aa_y;
	const GEN_FLT x4 = 1e-10 + (x0 * x3) + (x0 * x1) + (x0 * x2);
	const GEN_FLT x5 = sqrt(x4);
	const GEN_FLT x6 = 1.0 / 2.0 * x5;
	const GEN_FLT x7 = sin(x6);
	const GEN_FLT x8 = sqrt(1e-10 + (aa2_y * aa2_y) + (aa2_z * aa2_z) + (aa2_x * aa2_x));
	const GEN_FLT x9 = 1.0 / 2.0 * x8;
	const GEN_FLT x10 = (1. / x8) * sin(x9);
	const GEN_FLT x11 = x7 * x10;
	const GEN_FLT x12 = x11 * aa2_z;
	const GEN_FLT x13 = 1. / x5;
	const GEN_FLT x14 = x13 * time;
	const GEN_FLT x15 = x14 * x12;
	const GEN_FLT x16 = -1 * x15;
	const GEN_FLT x17 = x11 * aa2_y;
	const GEN_FLT x18 = x0 * x13;
	const GEN_FLT x19 = x18 * x17;
	const GEN_FLT x20 = 1.0 / 2.0 * aa_x;
	const GEN_FLT x21 = cos(x6);
	const GEN_FLT x22 = x21 * x10;
	const GEN_FLT x23 = x22 * aa2_z;
	const GEN_FLT x24 = 1. / x4;
	const GEN_FLT x25 = 1.0 / 2.0 * x24;
	const GEN_FLT x26 = x25 * x23;
	const GEN_FLT x27 = time * time * time;
	const GEN_FLT x28 = x2 * x27;
	const GEN_FLT x29 = x22 * aa2_x;
	const GEN_FLT x30 = x25 * x29;
	const GEN_FLT x31 = x27 * aa_z;
	const GEN_FLT x32 = x31 * aa_x;
	const GEN_FLT x33 = x30 * x32;
	const GEN_FLT x34 = x11 * aa2_x;
	const GEN_FLT x35 = 1. / (x4 * sqrt(x4));
	const GEN_FLT x36 = x35 * x27;
	const GEN_FLT x37 = aa_x * aa_z;
	const GEN_FLT x38 = x36 * x37;
	const GEN_FLT x39 = x34 * x38;
	const GEN_FLT x40 = x2 * x36;
	const GEN_FLT x41 = cos(x9);
	const GEN_FLT x42 = x7 * x41;
	const GEN_FLT x43 = x42 * x35;
	const GEN_FLT x44 = x43 * x27;
	const GEN_FLT x45 = aa_x * aa_y;
	const GEN_FLT x46 = 1.0 / 2.0 * aa_y;
	const GEN_FLT x47 = x41 * x21;
	const GEN_FLT x48 = x47 * x24;
	const GEN_FLT x49 = x46 * x48;
	const GEN_FLT x50 = x27 * aa_x;
	const GEN_FLT x51 = (x50 * x49) + (-1 * x44 * x45);
	const GEN_FLT x52 = x51 + (x40 * x12) + (-1 * x20 * x19) + x16 + (-1 * x28 * x26) + x33 + (-1 * x39);
	const GEN_FLT x53 = x14 * x11;
	const GEN_FLT x54 = x53 * aa2_x;
	const GEN_FLT x55 = x22 * aa2_y;
	const GEN_FLT x56 = x42 * x13;
	const GEN_FLT x57 = x56 * time;
	const GEN_FLT x58 = (x57 * aa_y) + (x54 * aa_z) + x55 + (-1 * x15 * aa_x);
	const GEN_FLT x59 = 2 * x58;
	const GEN_FLT x60 = x53 * aa2_y;
	const GEN_FLT x61 = x55 * x25;
	const GEN_FLT x62 = x12 * x18;
	const GEN_FLT x63 = x46 * x24;
	const GEN_FLT x64 = x63 * x29;
	const GEN_FLT x65 = x45 * x36;
	const GEN_FLT x66 = (x65 * x34) + (-1 * x64 * x50);
	const GEN_FLT x67 = x44 * aa_z;
	const GEN_FLT x68 = 1.0 / 2.0 * x48;
	const GEN_FLT x69 = x68 * x27;
	const GEN_FLT x70 = (x69 * x37) + (-1 * x67 * aa_x);
	const GEN_FLT x71 = x70 + (x61 * x28) + (-1 * x40 * x17) + x66 + x60 + (-1 * x62 * x20);
	const GEN_FLT x72 = (x57 * aa_z) + x23 + (x60 * aa_x) + (-1 * x54 * aa_y);
	const GEN_FLT x73 = 2 * x72;
	const GEN_FLT x74 = x38 * x17;
	const GEN_FLT x75 = x61 * x32;
	const GEN_FLT x76 = x63 * x23;
	const GEN_FLT x77 = x76 * x50;
	const GEN_FLT x78 = x36 * x12;
	const GEN_FLT x79 = x78 * x45;
	const GEN_FLT x80 = x34 * x18;
	const GEN_FLT x81 = 1.0 / 2.0 * x80;
	const GEN_FLT x82 = (-1 * x79) + x77 + (x2 * x69) + (-1 * x2 * x44) + x57 + (-1 * x81 * aa_x) + (-1 * x75) + x74;
	const GEN_FLT x83 = (-1 * x60 * aa_z) + (x57 * aa_x) + x29 + (x15 * aa_y);
	const GEN_FLT x84 = 2 * x83;
	const GEN_FLT x85 = (x82 * x84) + (x52 * x59) + (x71 * x73);
	const GEN_FLT x86 = x47 + (-1 * ((x60 * aa_y) + (x15 * aa_z) + (x54 * aa_x)));
	const GEN_FLT x87 = acos(x86);
	const GEN_FLT x88 = 1e-10 + (x58 * x58) + (x72 * x72) + (x83 * x83);
	const GEN_FLT x89 = (1. / (x88 * sqrt(x88))) * x87;
	const GEN_FLT x90 = x83 * x89;
	const GEN_FLT x91 = 1. / sqrt(x88);
	const GEN_FLT x92 = 2 * x87 * x91;
	const GEN_FLT x93 = -1 * x54;
	const GEN_FLT x94 = x0 * x56;
	const GEN_FLT x95 = 1.0 / 2.0 * x94;
	const GEN_FLT x96 = x65 * x17;
	const GEN_FLT x97 = x63 * x55;
	const GEN_FLT x98 = x50 * x97;
	const GEN_FLT x99 = (x78 * x37) + (-1 * x32 * x26);
	const GEN_FLT x100 = x99 + (-1 * x30 * x28) + (-1 * x95 * aa_x) + x93 + (x40 * x34) + (-1 * x98) + x96;
	const GEN_FLT x101 = x91 * (1. / sqrt(1 + (-1 * (x86 * x86))));
	const GEN_FLT x102 = x84 * x101;
	const GEN_FLT x103 = x3 * x27;
	const GEN_FLT x104 = x3 * x36;
	const GEN_FLT x105 = x11 * x104;
	const GEN_FLT x106 = (-1 * x67 * aa_y) + (x49 * x31);
	const GEN_FLT x107 = x106 + (x105 * aa2_x) + (-1 * x96) + (-1 * x62 * x46) + (-1 * x30 * x103) + x98 + x93;
	const GEN_FLT x108 = aa_y * aa_z;
	const GEN_FLT x109 = x36 * x108;
	const GEN_FLT x110 = x34 * x109;
	const GEN_FLT x111 = x64 * x31;
	const GEN_FLT x112 = (-1 * x77) + (x3 * x69) + (-1 * x3 * x44) + x111 + (-1 * x46 * x19) + x57 + x79 + (-1 * x110);
	const GEN_FLT x113 = (x17 * x109) + (-1 * x97 * x31);
	const GEN_FLT x114 = x51 + x15 + (x26 * x103) + (-1 * x80 * x46) + (-1 * x12 * x104) + x113;
	const GEN_FLT x115 = (x84 * x114) + (x73 * x107) + (x59 * x112);
	const GEN_FLT x116 = -1 * x60;
	const GEN_FLT x117 = x78 * x108;
	const GEN_FLT x118 = x76 * x31;
	const GEN_FLT x119 = x66 + (-1 * x118) + (x105 * aa2_y) + (-1 * x61 * x103) + x116 + (-1 * x94 * x46) + x117;
	const GEN_FLT x120 = x1 * x27;
	const GEN_FLT x121 = x35 * x120;
	const GEN_FLT x122 = (-1 * x117) + (-1 * x81 * aa_z) + (x17 * x121) + x116 + x70 + (-1 * x61 * x120) + x118;
	const GEN_FLT x123 = 1.0 / 2.0 * aa_z;
	const GEN_FLT x124 = x99 + (-1 * x34 * x121) + x54 + (-1 * x19 * x123) + (x30 * x120) + x106;
	const GEN_FLT x125 =
		(-1 * x111) + (x68 * x120) + (-1 * x62 * x123) + x57 + x110 + (-1 * x43 * x120) + x75 + (-1 * x74);
	const GEN_FLT x126 = x89 * ((x59 * x124) + (x84 * x122) + (x73 * x125));
	const GEN_FLT x127 = (x12 * x121) + (-1 * x26 * x120) + x16 + (-1 * x95 * aa_z) + x113 + (-1 * x33) + x39;
	const GEN_FLT x128 = x101 * x127;
	const GEN_FLT x129 = x89 * x58;
	const GEN_FLT x130 = x59 * x101;
	const GEN_FLT x131 = x89 * x72;
	const GEN_FLT x132 = x73 * x101;
	out[0] = (-1 * x100 * x102) + (-1 * x85 * x90) + (x82 * x92);
	out[1] = (-1 * x102 * x119) + (-1 * x90 * x115) + (x92 * x114);
	out[2] = (-1 * x84 * x128) + (x92 * x122) + (-1 * x83 * x126);
	out[3] = (-1 * x100 * x130) + (-1 * x85 * x129) + (x52 * x92);
	out[4] = (-1 * x119 * x130) + (-1 * x115 * x129) + (x92 * x112);
	out[5] = (-1 * x59 * x128) + (-1 * x58 * x126) + (x92 * x124);
	out[6] = (-1 * x100 * x132) + (-1 * x85 * x131) + (x71 * x92);
	out[7] = (-1 * x119 * x132) + (x92 * x107) + (-1 * x115 * x131);
	out[8] = (-1 * x127 * x132) + (-1 * x72 * x126) + (x92 * x125);
}

// Jacobian of apply_ang_velocity_aa wrt [time]
static inline void gen_apply_ang_velocity_aa_jac_time(FLT *out, const FLT *axis_angle, const FLT time,
													  const FLT *axis_angle2) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT aa2_x = axis_angle2[0];
	const GEN_FLT aa2_y = axis_angle2[1];
	const GEN_FLT aa2_z = axis_angle2[2];
	const GEN_FLT x0 = sqrt(1e-10 + (aa2_y * aa2_y) + (aa2_z * aa2_z) + (aa2_x * aa2_x));
	const GEN_FLT x1 = 1.0 / 2.0 * x0;
	const GEN_FLT x2 = (1. / x0) * sin(x1);
	const GEN_FLT x3 = time * time;
	const GEN_FLT x4 = aa_z * aa_z;
	const GEN_FLT x5 = aa_x * aa_x;
	const GEN_FLT x6 = aa_y * aa_y;
	const GEN_FLT x7 = 1e-10 + (x3 * x6) + (x4 * x3) + (x3 * x5);
	const GEN_FLT x8 = sqrt(x7);
	const GEN_FLT x9 = 1.0 / 2.0 * x8;
	const GEN_FLT x10 = sin(x9);
	const GEN_FLT x11 = (1. / x8) * x10;
	const GEN_FLT x12 = x2 * x11;
	const GEN_FLT x13 = x12 * aa2_y;
	const GEN_FLT x14 = x13 * aa_z;
	const GEN_FLT x15 = x12 * aa2_z;
	const GEN_FLT x16 = x15 * aa_y;
	const GEN_FLT x17 = cos(x9);
	const GEN_FLT x18 = x2 * x17;
	const GEN_FLT x19 = x18 * aa2_x;
	const GEN_FLT x20 = cos(x1);
	const GEN_FLT x21 = x20 * x11;
	const GEN_FLT x22 = x21 * aa_x;
	const GEN_FLT x23 = (x22 * time) + x19 + (-1 * x14 * time) + (x16 * time);
	const GEN_FLT x24 = x15 * aa_z;
	const GEN_FLT x25 = x12 * aa2_x;
	const GEN_FLT x26 = x25 * aa_x;
	const GEN_FLT x27 = x13 * aa_y;
	const GEN_FLT x28 = x20 * x17;
	const GEN_FLT x29 = x28 + (-1 * ((x27 * time) + (x24 * time) + (x26 * time)));
	const GEN_FLT x30 = acos(x29);
	const GEN_FLT x31 = x25 * aa_z;
	const GEN_FLT x32 = x15 * aa_x;
	const GEN_FLT x33 = x18 * aa2_y;
	const GEN_FLT x34 = x21 * aa_y;
	const GEN_FLT x35 = (x34 * time) + x33 + (x31 * time) + (-1 * x32 * time);
	const GEN_FLT x36 = 2 * time;
	const GEN_FLT x37 = (x6 * x36) + (x5 * x36) + (x4 * x36);
	const GEN_FLT x38 = 1.0 / 2.0 * (1. / (x7 * sqrt(x7))) * x37 * x10;
	const GEN_FLT x39 = x38 * x20;
	const GEN_FLT x40 = x39 * time;
	const GEN_FLT x41 = 1.0 / 4.0 * x37;
	const GEN_FLT x42 = (1. / x7) * x41;
	const GEN_FLT x43 = x42 * x28;
	const GEN_FLT x44 = x43 * time;
	const GEN_FLT x45 = time * aa_x;
	const GEN_FLT x46 = x2 * x38;
	const GEN_FLT x47 = x46 * aa2_z;
	const GEN_FLT x48 = x18 * aa2_z;
	const GEN_FLT x49 = x42 * x48;
	const GEN_FLT x50 = time * aa_z;
	const GEN_FLT x51 = x42 * x19;
	const GEN_FLT x52 = x46 * aa2_x;
	const GEN_FLT x53 = 2 * ((-1 * x50 * x52) + (x50 * x51) + (-1 * x32) + (x44 * aa_y) + (-1 * x40 * aa_y) +
							 (-1 * x41 * x13) + x34 + (x45 * x47) + (-1 * x45 * x49) + x31);
	const GEN_FLT x54 = x13 * aa_x;
	const GEN_FLT x55 = x25 * aa_y;
	const GEN_FLT x56 = x21 * aa_z;
	const GEN_FLT x57 = (x56 * time) + x48 + (x54 * time) + (-1 * x55 * time);
	const GEN_FLT x58 = time * aa_y;
	const GEN_FLT x59 = x42 * x33;
	const GEN_FLT x60 = x46 * aa2_y;
	const GEN_FLT x61 = 2 * ((-1 * x55) + (-1 * x60 * x45) + (-1 * x51 * x58) + (x59 * x45) + (x50 * x43) +
							 (x52 * x58) + x54 + x56 + (-1 * x50 * x39) + (-1 * x41 * x15));
	const GEN_FLT x62 = 2 * ((-1 * x50 * x59) + (x60 * x50) + (x44 * aa_x) + x16 + x22 + (-1 * x41 * x25) +
							 (x58 * x49) + (-1 * x58 * x47) + (-1 * x14) + (-1 * x40 * aa_x));
	const GEN_FLT x63 = 1e-10 + (x35 * x35) + (x57 * x57) + (x23 * x23);
	const GEN_FLT x64 = (1. / (x63 * sqrt(x63))) * x30 * ((x62 * x23) + (x53 * x35) + (x61 * x57));
	const GEN_FLT x65 = 1. / sqrt(x63);
	const GEN_FLT x66 = x65 * x30;
	const GEN_FLT x67 = 2 * x65 * (1. / sqrt(1 + (-1 * (x29 * x29)))) *
						((-1 * x50 * x49) + (-1 * x24) + (-1 * x27) + (-1 * x26) + (-1 * x41 * x21) + (x60 * x58) +
						 (-1 * x58 * x59) + (-1 * x51 * x45) + (x52 * x45) + (x50 * x47));
	out[0] = (-1 * x67 * x23) + (-1 * x64 * x23) + (x62 * x66);
	out[1] = (-1 * x67 * x35) + (-1 * x64 * x35) + (x66 * x53);
	out[2] = (-1 * x64 * x57) + (-1 * x67 * x57) + (x61 * x66);
}

// Jacobian of apply_ang_velocity_aa wrt [aa2_x, aa2_y, aa2_z]
static inline void gen_apply_ang_velocity_aa_jac_axis_angle2(FLT *out, const FLT *axis_angle, const FLT time,
															 const FLT *axis_angle2) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT aa2_x = axis_angle2[0];
	const GEN_FLT aa2_y = axis_angle2[1];
	const GEN_FLT aa2_z = axis_angle2[2];
	const GEN_FLT x0 = aa2_z * aa2_z;
	const GEN_FLT x1 = aa2_x * aa2_x;
	const GEN_FLT x2 = aa2_y * aa2_y;
	const GEN_FLT x3 = 1e-10 + x2 + x0 + x1;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = 1. / x4;
	const GEN_FLT x6 = 1.0 / 2.0 * x4;
	const GEN_FLT x7 = sin(x6);
	const GEN_FLT x8 = time * time;
	const GEN_FLT x9 = sqrt(1e-10 + (x8 * (aa_y * aa_y)) + (x8 * (aa_z * aa_z)) + (x8 * (aa_x * aa_x)));
	const GEN_FLT x10 = 1.0 / 2.0 * x9;
	const GEN_FLT x11 = (1. / x9) * sin(x10) * time;
	const GEN_FLT x12 = x7 * x11;
	const GEN_FLT x13 = x5 * x12;
	const GEN_FLT x14 = x13 * aa_z;
	const GEN_FLT x15 = x14 * aa2_y;
	const GEN_FLT x16 = x13 * aa_y;
	const GEN_FLT x17 = x16 * aa2_z;
	const GEN_FLT x18 = cos(x10);
	const GEN_FLT x19 = x7 * x18;
	const GEN_FLT x20 = x5 * x19;
	const GEN_FLT x21 = x20 * aa2_x;
	const GEN_FLT x22 = cos(x6);
	const GEN_FLT x23 = x22 * x11;
	const GEN_FLT x24 = x23 * aa_x;
	const GEN_FLT x25 = x24 + x21 + (-1 * x15) + x17;
	const GEN_FLT x26 = x16 * aa2_x;
	const GEN_FLT x27 = x23 * aa_z;
	const GEN_FLT x28 = 1.0 / 2.0 * (1. / x3);
	const GEN_FLT x29 = x1 * x28;
	const GEN_FLT x30 = 1. / (x3 * sqrt(x3));
	const GEN_FLT x31 = x30 * x12;
	const GEN_FLT x32 = x31 * aa_z;
	const GEN_FLT x33 = x22 * x18;
	const GEN_FLT x34 = x33 * x28;
	const GEN_FLT x35 = aa2_y * aa2_x;
	const GEN_FLT x36 = x30 * x19;
	const GEN_FLT x37 = (-1 * x36 * x35) + (x34 * x35);
	const GEN_FLT x38 = x31 * aa_x;
	const GEN_FLT x39 = aa2_z * aa2_x;
	const GEN_FLT x40 = x39 * x28;
	const GEN_FLT x41 = (-1 * x40 * x24) + (x38 * x39);
	const GEN_FLT x42 = x41 + x14 + x37 + (-1.0 / 2.0 * x26) + (x29 * x27) + (-1 * x1 * x32);
	const GEN_FLT x43 = x14 * aa2_x;
	const GEN_FLT x44 = x13 * aa_x;
	const GEN_FLT x45 = x44 * aa2_z;
	const GEN_FLT x46 = x20 * aa2_y;
	const GEN_FLT x47 = x23 * aa_y;
	const GEN_FLT x48 = x46 + x47 + x43 + (-1 * x45);
	const GEN_FLT x49 = 2 * x48;
	const GEN_FLT x50 = x35 * x38;
	const GEN_FLT x51 = x24 * x28;
	const GEN_FLT x52 = x51 * x35;
	const GEN_FLT x53 = x1 * x31;
	const GEN_FLT x54 = -1 * x16;
	const GEN_FLT x55 = x34 * aa2_z;
	const GEN_FLT x56 = x36 * aa2_z;
	const GEN_FLT x57 = (-1 * x56 * aa2_x) + (x55 * aa2_x);
	const GEN_FLT x58 = x57 + (-1 * x47 * x29) + (-1 * x50) + (-1.0 / 2.0 * x43) + x52 + (x53 * aa_y) + x54;
	const GEN_FLT x59 = x44 * aa2_y;
	const GEN_FLT x60 = x20 * aa2_z;
	const GEN_FLT x61 = x27 + x60 + x59 + (-1 * x26);
	const GEN_FLT x62 = 2 * x61;
	const GEN_FLT x63 = x44 * aa2_x;
	const GEN_FLT x64 = x47 * x28;
	const GEN_FLT x65 = x64 * x39;
	const GEN_FLT x66 = x28 * x27;
	const GEN_FLT x67 = x66 * x35;
	const GEN_FLT x68 = x32 * x35;
	const GEN_FLT x69 = x31 * aa_y;
	const GEN_FLT x70 = x69 * x39;
	const GEN_FLT x71 = (-1 * x70) + x68 + (-1 * x67) + (x1 * x34) + x20 + (-1.0 / 2.0 * x63) + x65 + (-1 * x1 * x36);
	const GEN_FLT x72 = 2 * x25;
	const GEN_FLT x73 = (x42 * x49) + (x71 * x72) + (x62 * x58);
	const GEN_FLT x74 = x14 * aa2_z;
	const GEN_FLT x75 = x16 * aa2_y;
	const GEN_FLT x76 = x33 + (-1 * (x74 + x75 + x63));
	const GEN_FLT x77 = acos(x76);
	const GEN_FLT x78 = 1e-10 + (x48 * x48) + (x61 * x61) + (x25 * x25);
	const GEN_FLT x79 = (1. / (x78 * sqrt(x78))) * x77;
	const GEN_FLT x80 = x73 * x79;
	const GEN_FLT x81 = 1. / sqrt(x78);
	const GEN_FLT x82 = 2 * x81 * x77;
	const GEN_FLT x83 = x40 * x27;
	const GEN_FLT x84 = -1 * x44;
	const GEN_FLT x85 = x32 * x39;
	const GEN_FLT x86 = (-1 * x64 * x35) + (x69 * x35);
	const GEN_FLT x87 = x86 + (-1 * x24 * x29) + x85 + (-1.0 / 2.0 * x21) + (-1 * x83) + (x53 * aa_x) + x84;
	const GEN_FLT x88 = x81 * (1. / sqrt(1 + (-1 * (x76 * x76))));
	const GEN_FLT x89 = x88 * x87;
	const GEN_FLT x90 = aa2_z * aa2_y;
	const GEN_FLT x91 = x51 * x90;
	const GEN_FLT x92 = x90 * x38;
	const GEN_FLT x93 = (-1 * x2 * x36) + (-1.0 / 2.0 * x75) + x20 + (x2 * x34) + (-1 * x68) + (-1 * x91) + x92 + x67;
	const GEN_FLT x94 = x2 * x28;
	const GEN_FLT x95 = (-1 * x56 * aa2_y) + (x55 * aa2_y);
	const GEN_FLT x96 = x95 + x44 + (-1.0 / 2.0 * x15) + (-1 * x2 * x38) + x86 + (x94 * x24);
	const GEN_FLT x97 = -1 * x14;
	const GEN_FLT x98 = x64 * x90;
	const GEN_FLT x99 = x69 * x90;
	const GEN_FLT x100 = (-1.0 / 2.0 * x59) + x97 + (x2 * x32) + x98 + x37 + (-1 * x94 * x27) + (-1 * x99);
	const GEN_FLT x101 = (x72 * x100) + (x93 * x49) + (x62 * x96);
	const GEN_FLT x102 = x79 * x25;
	const GEN_FLT x103 = (x90 * x32) + (-1 * x66 * x90);
	const GEN_FLT x104 = x103 + (-1 * x52) + (x2 * x69) + x54 + (-1.0 / 2.0 * x46) + x50 + (-1 * x2 * x64);
	const GEN_FLT x105 = x88 * x104;
	const GEN_FLT x106 = (-1 * x0 * x36) + (-1 * x65) + x70 + x91 + (-1.0 / 2.0 * x74) + x20 + (x0 * x34) + (-1 * x92);
	const GEN_FLT x107 = x0 * x28;
	const GEN_FLT x108 = (-1 * x24 * x107) + x84 + (-1.0 / 2.0 * x17) + x83 + x95 + (x0 * x38) + (-1 * x85);
	const GEN_FLT x109 = x103 + x16 + (x0 * x64) + (-1.0 / 2.0 * x45) + x57 + (-1 * x0 * x69);
	const GEN_FLT x110 = (x72 * x109) + (x62 * x106) + (x49 * x108);
	const GEN_FLT x111 = x88 * ((x0 * x32) + (-1.0 / 2.0 * x60) + x99 + x97 + (-1 * x27 * x107) + x41 + (-1 * x98));
	const GEN_FLT x112 = x79 * x48;
	const GEN_FLT x113 = x88 * x62;
	const GEN_FLT x114 = x79 * x61;
	out[0] = (-1 * x80 * x25) + (-1 * x89 * x72) + (x82 * x71);
	out[1] = (-1 * x72 * x105) + (-1 * x101 * x102) + (x82 * x100);
	out[2] = (-1 * x72 * x111) + (-1 * x102 * x110) + (x82 * x109);
	out[3] = (-1 * x89 * x49) + (x82 * x42) + (-1 * x73 * x112);
	out[4] = (-1 * x49 * x105) + (x82 * x93) + (-1 * x101 * x112);
	out[5] = (-1 * x49 * x111) + (-1 * x110 * x112) + (x82 * x108);
	out[6] = (-1 * x87 * x113) + (x82 * x58) + (-1 * x80 * x61);
	out[7] = (-1 * x104 * x113) + (-1 * x101 * x114) + (x82 * x96);
	out[8] = (-1 * x62 * x111) + (-1 * x110 * x114) + (x82 * x106);
}

static inline void gen_axisanglecompose(FLT *out, const FLT *axis_angle, const FLT *axis_angle2) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT aa2_x = axis_angle2[0];
	const GEN_FLT aa2_y = axis_angle2[1];
	const GEN_FLT aa2_z = axis_angle2[2];
	const GEN_FLT x0 = sqrt(1e-10 + (aa_y * aa_y) + (aa_z * aa_z) + (aa_x * aa_x));
	const GEN_FLT x1 = 1.0 / 2.0 * x0;
	const GEN_FLT x2 = (1. / x0) * sin(x1);
	const GEN_FLT x3 = sqrt(1e-10 + (aa2_y * aa2_y) + (aa2_z * aa2_z) + (aa2_x * aa2_x));
	const GEN_FLT x4 = 1.0 / 2.0 * x3;
	const GEN_FLT x5 = (1. / x3) * sin(x4);
	const GEN_FLT x6 = x2 * x5;
	const GEN_FLT x7 = x6 * aa2_y;
	const GEN_FLT x8 = x6 * aa_y;
	const GEN_FLT x9 = cos(x1);
	const GEN_FLT x10 = x5 * x9;
	const GEN_FLT x11 = cos(x4);
	const GEN_FLT x12 = x2 * x11;
	const GEN_FLT x13 = (x12 * aa_x) + (x10 * aa2_x) + (-1 * x7 * aa_z) + (x8 * aa2_z);
	const GEN_FLT x14 = x6 * aa2_z;
	const GEN_FLT x15 = x6 * aa2_x;
	const GEN_FLT x16 = (x12 * aa_z) + (x7 * aa_x) + (x10 * aa2_z) + (-1 * x8 * aa2_x);
	const GEN_FLT x17 = (x12 * aa_y) + (x15 * aa_z) + (x10 * aa2_y) + (-1 * x14 * aa_x);
	const GEN_FLT x18 = 2 * (1. / sqrt(1e-10 + (x16 * x16) + (x17 * x17) + (x13 * x13))) *
						acos((x9 * x11) + (-1 * ((x7 * aa_y) + (x14 * aa_z) + (x15 * aa_x))));
	out[0] = x13 * x18;
	out[1] = x18 * x17;
	out[2] = x18 * x16;
}

// Jacobian of axisanglecompose wrt [aa_x, aa_y, aa_z]
static inline void gen_axisanglecompose_jac_axis_angle(FLT *out, const FLT *axis_angle, const FLT *axis_angle2) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT aa2_x = axis_angle2[0];
	const GEN_FLT aa2_y = axis_angle2[1];
	const GEN_FLT aa2_z = axis_angle2[2];
	const GEN_FLT x0 = aa_z * aa_z;
	const GEN_FLT x1 = aa_x * aa_x;
	const GEN_FLT x2 = aa_y * aa_y;
	const GEN_FLT x3 = 1e-10 + x2 + x0 + x1;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = 1. / x4;
	const GEN_FLT x6 = 1.0 / 2.0 * x4;
	const GEN_FLT x7 = sin(x6);
	const GEN_FLT x8 = sqrt(1e-10 + (aa2_y * aa2_y) + (aa2_z * aa2_z) + (aa2_x * aa2_x));
	const GEN_FLT x9 = 1.0 / 2.0 * x8;
	const GEN_FLT x10 = (1. / x8) * sin(x9);
	const GEN_FLT x11 = x7 * x10;
	const GEN_FLT x12 = x5 * x11;
	const GEN_FLT x13 = x12 * aa2_y;
	const GEN_FLT x14 = x13 * aa_z;
	const GEN_FLT x15 = x12 * aa2_z;
	const GEN_FLT x16 = x15 * aa_y;
	const GEN_FLT x17 = cos(x6);
	const GEN_FLT x18 = x10 * x17;
	const GEN_FLT x19 = x18 * aa2_x;
	const GEN_FLT x20 = cos(x9);
	const GEN_FLT x21 = x7 * x20;
	const GEN_FLT x22 = x5 * x21;
	const GEN_FLT x23 = x22 * aa_x;
	const GEN_FLT x24 = x23 + x19 + (-1 * x14) + x16;
	const GEN_FLT x25 = -1 * x15;
	const GEN_FLT x26 = 1. / (x3 * sqrt(x3));
	const GEN_FLT x27 = x26 * x11;
	const GEN_FLT x28 = x27 * aa2_x;
	const GEN_FLT x29 = aa_x * aa_z;
	const GEN_FLT x30 = x28 * x29;
	const GEN_FLT x31 = x27 * aa2_z;
	const GEN_FLT x32 = 1.0 / 2.0 * (1. / x3);
	const GEN_FLT x33 = x32 * x29;
	const GEN_FLT x34 = x33 * x19;
	const GEN_FLT x35 = x18 * aa2_z;
	const GEN_FLT x36 = x1 * x32;
	const GEN_FLT x37 = x13 * aa_x;
	const GEN_FLT x38 = x20 * x17;
	const GEN_FLT x39 = x32 * x38;
	const GEN_FLT x40 = x39 * aa_x;
	const GEN_FLT x41 = x21 * x26;
	const GEN_FLT x42 = x41 * aa_x;
	const GEN_FLT x43 = (-1 * x42 * aa_y) + (x40 * aa_y);
	const GEN_FLT x44 = (-1.0 / 2.0 * x37) + (-1 * x30) + (-1 * x36 * x35) + x25 + x43 + (x1 * x31) + x34;
	const GEN_FLT x45 = x12 * aa2_x;
	const GEN_FLT x46 = x45 * aa_z;
	const GEN_FLT x47 = x15 * aa_x;
	const GEN_FLT x48 = x18 * aa2_y;
	const GEN_FLT x49 = x22 * aa_y;
	const GEN_FLT x50 = x49 + x48 + x46 + (-1 * x47);
	const GEN_FLT x51 = 2 * x50;
	const GEN_FLT x52 = x27 * aa2_y;
	const GEN_FLT x53 = (x40 * aa_z) + (-1 * x42 * aa_z);
	const GEN_FLT x54 = aa_x * aa_y;
	const GEN_FLT x55 = x54 * x32;
	const GEN_FLT x56 = (x54 * x28) + (-1 * x55 * x19);
	const GEN_FLT x57 = x56 + (-1 * x1 * x52) + x13 + (-1.0 / 2.0 * x47) + (x48 * x36) + x53;
	const GEN_FLT x58 = x45 * aa_y;
	const GEN_FLT x59 = x22 * aa_z;
	const GEN_FLT x60 = x59 + x35 + x37 + (-1 * x58);
	const GEN_FLT x61 = 2 * x60;
	const GEN_FLT x62 = x52 * x29;
	const GEN_FLT x63 = x48 * x33;
	const GEN_FLT x64 = x45 * aa_x;
	const GEN_FLT x65 = x55 * x35;
	const GEN_FLT x66 = x54 * x31;
	const GEN_FLT x67 = (-1 * x66) + (-1.0 / 2.0 * x64) + x22 + (x1 * x39) + x62 + x65 + (-1 * x1 * x41) + (-1 * x63);
	const GEN_FLT x68 = 2 * x24;
	const GEN_FLT x69 = x15 * aa_z;
	const GEN_FLT x70 = x13 * aa_y;
	const GEN_FLT x71 = x38 + (-1 * (x70 + x69 + x64));
	const GEN_FLT x72 = acos(x71);
	const GEN_FLT x73 = 1e-10 + (x60 * x60) + (x50 * x50) + (x24 * x24);
	const GEN_FLT x74 = (1. / (x73 * sqrt(x73))) * x72;
	const GEN_FLT x75 = x74 * ((x67 * x68) + (x51 * x44) + (x61 * x57));
	const GEN_FLT x76 = 1. / sqrt(x73);
	const GEN_FLT x77 = 2 * x72 * x76;
	const GEN_FLT x78 = x54 * x52;
	const GEN_FLT x79 = x55 * x48;
	const GEN_FLT x80 = -1 * x45;
	const GEN_FLT x81 = (-1 * x33 * x35) + (x31 * x29);
	const GEN_FLT x82 = x76 * (1. / sqrt(1 + (-1 * (x71 * x71))));
	const GEN_FLT x83 = x82 * ((x1 * x28) + (-1.0 / 2.0 * x23) + x81 + x78 + (-1 * x36 * x19) + x80 + (-1 * x79));
	const GEN_FLT x84 = aa_y * aa_z;
	const GEN_FLT x85 = x84 * x28;
	const GEN_FLT x86 = x84 * x32;
	const GEN_FLT x87 = x86 * x19;
	const GEN_FLT x88 = x66 + (-1 * x65) + (-1.0 / 2.0 * x70) + x22 + (x2 * x39) + x87 + (-1 * x2 * x41) + (-1 * x85);
	const GEN_FLT x89 = x2 * x32;
	const GEN_FLT x90 = (x84 * x39) + (-1 * x84 * x41);
	const GEN_FLT x91 = (-1 * x78) + (x2 * x28) + (-1.0 / 2.0 * x16) + x90 + x80 + x79 + (-1 * x89 * x19);
	const GEN_FLT x92 = (x84 * x52) + (-1 * x86 * x48);
	const GEN_FLT x93 = x15 + (-1.0 / 2.0 * x58) + x43 + (-1 * x2 * x31) + (x89 * x35) + x92;
	const GEN_FLT x94 = x74 * ((x68 * x93) + (x88 * x51) + (x61 * x91));
	const GEN_FLT x95 = -1 * x13;
	const GEN_FLT x96 = x84 * x31;
	const GEN_FLT x97 = x86 * x35;
	const GEN_FLT x98 = x56 + (-1 * x97) + x95 + (-1.0 / 2.0 * x49) + x96 + (-1 * x89 * x48) + (x2 * x52);
	const GEN_FLT x99 = x82 * x68;
	const GEN_FLT x100 = x0 * x32;
	const GEN_FLT x101 = x90 + x45 + (x19 * x100) + (-1 * x0 * x28) + (-1.0 / 2.0 * x14) + x81;
	const GEN_FLT x102 = (-1 * x87) + x85 + (x0 * x39) + (-1 * x0 * x41) + (-1.0 / 2.0 * x69) + x22 + (-1 * x62) + x63;
	const GEN_FLT x103 = x53 + (-1 * x96) + (x0 * x52) + (-1 * x48 * x100) + x97 + (-1.0 / 2.0 * x46) + x95;
	const GEN_FLT x104 = x74 * ((x68 * x103) + (x51 * x101) + (x61 * x102));
	const GEN_FLT x105 = (x0 * x31) + x92 + x30 + (-1 * x34) + (-1 * x35 * x100) + (-1.0 / 2.0 * x59) + x25;
	const GEN_FLT x106 = x82 * x51;
	const GEN_FLT x107 = x82 * x61;
	out[0] = (-1 * x75 * x24) + (-1 * x83 * x68) + (x77 * x67);
	out[1] = (-1 * x99 * x98) + (-1 * x94 * x24) + (x77 * x93);
	out[2] = (-1 * x99 * x105) + (-1 * x24 * x104) + (x77 * x103);
	out[3] = (x77 * x44) + (-1 * x75 * x50) + (-1 * x83 * x51);
	out[4] = (-1 * x50 * x94) + (x88 * x77) + (-1 * x98 * x106);
	out[5] = (-1 * x50 * x104) + (x77 * x101) + (-1 * x105 * x106);
	out[6] = (-1 * x83 * x61) + (x77 * x57) + (-1 * x75 * x60);
	out[7] = (-1 * x98 * x107) + (-1 * x60 * x94) + (x77 * x91);
	out[8] = (-1 * x105 * x107) + (x77 * x102) + (-1 * x60 * x104);
}

// Jacobian of axisanglecompose wrt [aa2_x, aa2_y, aa2_z]
static inline void gen_axisanglecompose_jac_axis_angle2(FLT *out, const FLT *axis_angle, const FLT *axis_angle2) {
	const GEN_FLT aa_x = axis_angle[0];
	const GEN_FLT aa_y = axis_angle[1];
	const GEN_FLT aa_z = axis_angle[2];
	const GEN_FLT aa2_x = axis_angle2[0];
	const GEN_FLT aa2_y = axis_angle2[1];
	const GEN_FLT aa2_z = axis_angle2[2];
	const GEN_FLT x0 = aa2_z * aa2_z;
	const GEN_FLT x1 = aa2_x * aa2_x;
	const GEN_FLT x2 = aa2_y * aa2_y;
	const GEN_FLT x3 = 1e-10 + x2 + x0 + x1;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = 1. / x4;
	const GEN_FLT x6 = 1.0 / 2.0 * x4;
	const GEN_FLT x7 = sin(x6);
	const GEN_FLT x8 = sqrt(1e-10 + (aa_y * aa_y) + (aa_z * aa_z) + (aa_x * aa_x));
	const GEN_FLT x9 = 1.0 / 2.0 * x8;
	const GEN_FLT x10 = (1. / x8) * sin(x9);
	const GEN_FLT x11 = x7 * x10;
	const GEN_FLT x12 = x5 * x11;
	const GEN_FLT x13 = x12 * aa_y;
	const GEN_FLT x14 = x13 * aa2_x;
	const GEN_FLT x15 = x12 * aa_z;
	const GEN_FLT x16 = 1. / (x3 * sqrt(x3));
	const GEN_FLT x17 = x11 * x16;
	const GEN_FLT x18 = x17 * aa_z;
	const GEN_FLT x19 = cos(x6);
	const GEN_FLT x20 = x10 * x19;
	const GEN_FLT x21 = x20 * aa_z;
	const GEN_FLT x22 = 1.0 / 2.0 * (1. / x3);
	const GEN_FLT x23 = x22 * x21;
	const GEN_FLT x24 = aa2_z * aa2_x;
	const GEN_FLT x25 = x17 * aa_x;
	const GEN_FLT x26 = x20 * aa_x;
	const GEN_FLT x27 = x22 * x26;
	const GEN_FLT x28 = (-1 * x24 * x27) + (x24 * x25);
	const GEN_FLT x29 = cos(x9);
	const GEN_FLT x30 = x29 * x19;
	const GEN_FLT x31 = x30 * x22;
	const GEN_FLT x32 = x31 * aa2_y;
	const GEN_FLT x33 = x7 * x29;
	const GEN_FLT x34 = x33 * x16;
	const GEN_FLT x35 = x34 * aa2_y;
	const GEN_FLT x36 = (-1 * x35 * aa2_x) + (x32 * aa2_x);
	const GEN_FLT x37 = x36 + x15 + (x1 * x23) + (-1.0 / 2.0 * x14) + (-1 * x1 * x18) + x28;
	const GEN_FLT x38 = x15 * aa2_x;
	const GEN_FLT x39 = x12 * aa_x;
	const GEN_FLT x40 = x39 * aa2_z;
	const GEN_FLT x41 = x5 * x33;
	const GEN_FLT x42 = x41 * aa2_y;
	const GEN_FLT x43 = x20 * aa_y;
	const GEN_FLT x44 = x43 + x42 + x38 + (-1 * x40);
	const GEN_FLT x45 = 2 * x44;
	const GEN_FLT x46 = -1 * x13;
	const GEN_FLT x47 = aa2_y * aa2_x;
	const GEN_FLT x48 = x47 * x27;
	const GEN_FLT x49 = x25 * aa2_y;
	const GEN_FLT x50 = x49 * aa2_x;
	const GEN_FLT x51 = x43 * x22;
	const GEN_FLT x52 = x17 * aa_y;
	const GEN_FLT x53 = (x31 * x24) + (-1 * x34 * x24);
	const GEN_FLT x54 = x53 + (x1 * x52) + (-1.0 / 2.0 * x38) + x46 + x48 + (-1 * x50) + (-1 * x1 * x51);
	const GEN_FLT x55 = x39 * aa2_y;
	const GEN_FLT x56 = x41 * aa2_z;
	const GEN_FLT x57 = x56 + x21 + x55 + (-1 * x14);
	const GEN_FLT x58 = 2 * x57;
	const GEN_FLT x59 = x39 * aa2_x;
	const GEN_FLT x60 = x47 * x23;
	const GEN_FLT x61 = x18 * aa2_y;
	const GEN_FLT x62 = x61 * aa2_x;
	const GEN_FLT x63 = x52 * x24;
	const GEN_FLT x64 = x51 * x24;
	const GEN_FLT x65 = x64 + (-1 * x63) + (x1 * x31) + x41 + (-1 * x1 * x34) + (-1 * x60) + (-1.0 / 2.0 * x59) + x62;
	const GEN_FLT x66 = x15 * aa2_y;
	const GEN_FLT x67 = x13 * aa2_z;
	const GEN_FLT x68 = x41 * aa2_x;
	const GEN_FLT x69 = x26 + x68 + (-1 * x66) + x67;
	const GEN_FLT x70 = 2 * x69;
	const GEN_FLT x71 = (x70 * x65) + (x45 * x37) + (x54 * x58);
	const GEN_FLT x72 = x15 * aa2_z;
	const GEN_FLT x73 = x13 * aa2_y;
	const GEN_FLT x74 = x30 + (-1 * (x73 + x72 + x59));
	const GEN_FLT x75 = acos(x74);
	const GEN_FLT x76 = 1e-10 + (x57 * x57) + (x44 * x44) + (x69 * x69);
	const GEN_FLT x77 = x75 * (1. / (x76 * sqrt(x76)));
	const GEN_FLT x78 = x77 * x69;
	const GEN_FLT x79 = 1. / sqrt(x76);
	const GEN_FLT x80 = 2 * x79 * x75;
	const GEN_FLT x81 = -1 * x39;
	const GEN_FLT x82 = x24 * x18;
	const GEN_FLT x83 = x24 * x23;
	const GEN_FLT x84 = x52 * aa2_y;
	const GEN_FLT x85 = (-1 * x51 * x47) + (x84 * aa2_x);
	const GEN_FLT x86 = x85 + (x1 * x25) + (-1.0 / 2.0 * x68) + (-1 * x83) + x81 + (-1 * x1 * x27) + x82;
	const GEN_FLT x87 = x79 * (1. / sqrt(1 + (-1 * (x74 * x74))));
	const GEN_FLT x88 = x86 * x87;
	const GEN_FLT x89 = x49 * aa2_z;
	const GEN_FLT x90 = aa2_z * aa2_y;
	const GEN_FLT x91 = x90 * x27;
	const GEN_FLT x92 = (-1 * x91) + (-1 * x2 * x34) + x41 + (x2 * x31) + (-1.0 / 2.0 * x73) + x60 + (-1 * x62) + x89;
	const GEN_FLT x93 = x2 * x17;
	const GEN_FLT x94 = (-1 * x35 * aa2_z) + (x32 * aa2_z);
	const GEN_FLT x95 = x85 + (-1.0 / 2.0 * x66) + (-1 * x93 * aa_x) + (x2 * x27) + x39 + x94;
	const GEN_FLT x96 = -1 * x15;
	const GEN_FLT x97 = x84 * aa2_z;
	const GEN_FLT x98 = x51 * x90;
	const GEN_FLT x99 = x36 + x98 + (x93 * aa_z) + x96 + (-1.0 / 2.0 * x55) + (-1 * x2 * x23) + (-1 * x97);
	const GEN_FLT x100 = (x70 * x99) + (x92 * x45) + (x58 * x95);
	const GEN_FLT x101 = (x61 * aa2_z) + (-1 * x90 * x23);
	const GEN_FLT x102 = x101 + (-1 * x48) + (-1.0 / 2.0 * x42) + (-1 * x2 * x51) + x46 + x50 + (x93 * aa_y);
	const GEN_FLT x103 = x87 * x102;
	const GEN_FLT x104 = x83 + (-1.0 / 2.0 * x67) + (-1 * x82) + x81 + (x0 * x25) + x94 + (-1 * x0 * x27);
	const GEN_FLT x105 = (-1 * x0 * x34) + x91 + x41 + (x0 * x31) + x63 + (-1 * x89) + (-1.0 / 2.0 * x72) + (-1 * x64);
	const GEN_FLT x106 = x53 + (-1 * x0 * x52) + (x0 * x51) + (-1.0 / 2.0 * x40) + x13 + x101;
	const GEN_FLT x107 = (x70 * x106) + (x45 * x104) + (x58 * x105);
	const GEN_FLT x108 = (x0 * x18) + x96 + (-1 * x0 * x23) + (-1.0 / 2.0 * x56) + x28 + x97 + (-1 * x98);
	const GEN_FLT x109 = x87 * x108;
	const GEN_FLT x110 = x77 * x44;
	const GEN_FLT x111 = x77 * x57;
	const GEN_FLT x112 = x87 * x58;
	out[0] = (-1 * x88 * x70) + (-1 * x71 * x78) + (x80 * x65);
	out[1] = (-1 * x78 * x100) + (-1 * x70 * x103) + (x80 * x99);
	out[2] = (-1 * x70 * x109) + (-1 * x78 * x107) + (x80 * x106);
	out[3] = (-1 * x71 * x110) + (x80 * x37) + (-1 * x88 * x45);
	out[4] = (-1 * x100 * x110) + (x80 * x92) + (-1 * x45 * x103);
	out[5] = (-1 * x107 * x110) + (x80 * x104) + (-1 * x45 * x109);
	out[6] = (-1 * x86 * x112) + (-1 * x71 * x111) + (x80 * x54);
	out[7] = (-1 * x102 * x112) + (-1 * x100 * x111) + (x80 * x95);
	out[8] = (-1 * x108 * x112) + (-1 * x107 * x111) + (x80 * x105);
}

static inline void gen_scale_sensor_pt(FLT *out, const FLT *sensor_pt, const SurvivePose *obj_p, const FLT scale) {
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT x0 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x1 = (-1 * obj_qj * sensor_x) + (obj_qw * sensor_z) + (obj_qi * sensor_y);
	const GEN_FLT x2 = scale * ((2 * ((x1 * obj_qj) + (-1 * x0 * obj_qk))) + obj_px + sensor_x);
	const GEN_FLT x3 = 1. / sqrt(1e-11 + (obj_qw * obj_qw) + (obj_qi * obj_qi) + (obj_qj * obj_qj) + (obj_qk * obj_qk));
	const GEN_FLT x4 = x3 * obj_py;
	const GEN_FLT x5 = x3 * obj_qk;
	const GEN_FLT x6 = x3 * obj_qi;
	const GEN_FLT x7 = (x6 * obj_pz) + (x4 * obj_qw) + (-1 * x5 * obj_px);
	const GEN_FLT x8 = x3 * obj_qw;
	const GEN_FLT x9 = x3 * obj_qj;
	const GEN_FLT x10 = x3 * ((x9 * obj_px) + (x8 * obj_pz) + (-1 * x6 * obj_py));
	const GEN_FLT x11 = (obj_qw * sensor_x) + (-1 * obj_qk * sensor_y) + (obj_qj * sensor_z);
	const GEN_FLT x12 = scale * ((2 * ((x11 * obj_qk) + (-1 * x1 * obj_qi))) + obj_py + sensor_y);
	const GEN_FLT x13 = scale * ((2 * ((x0 * obj_qi) + (-1 * x11 * obj_qj))) + obj_pz + sensor_z);
	const GEN_FLT x14 = (x6 * x13) + (x8 * x12) + (-1 * x2 * x5);
	const GEN_FLT x15 = (x8 * x13) + (x2 * x9) + (-1 * x6 * x12);
	const GEN_FLT x16 = x3 * ((x4 * obj_qk) + (-1 * x9 * obj_pz) + (x8 * obj_px));
	const GEN_FLT x17 = (x5 * x12) + (x2 * x8) + (-1 * x9 * x13);
	out[0] = (2 * ((-1 * x9 * x15) + (x5 * x14))) + x2 + (-1 * (obj_px + (2 * ((-1 * x10 * obj_qj) + (x5 * x7)))));
	out[1] =
		(2 * ((-1 * x5 * x17) + (x6 * x15))) + (-1 * (obj_py + (2 * ((-1 * x16 * obj_qk) + (x10 * obj_qi))))) + x12;
	out[2] = (2 * ((-1 * x6 * x14) + (x9 * x17))) + (-1 * (obj_pz + (2 * ((-1 * x6 * x7) + (x16 * obj_qj))))) + x13;
}

// Jacobian of scale_sensor_pt wrt [sensor_x, sensor_y, sensor_z]
static inline void gen_scale_sensor_pt_jac_sensor_pt(FLT *out, const FLT *sensor_pt, const SurvivePose *obj_p,
													 const FLT scale) {
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT x0 = 2 * obj_qj;
	const GEN_FLT x1 = x0 * obj_qw;
	const GEN_FLT x2 = 2 * obj_qk;
	const GEN_FLT x3 = x2 * obj_qi;
	const GEN_FLT x4 = (x3 + (-1 * x1)) * scale;
	const GEN_FLT x5 = obj_qk * obj_qk;
	const GEN_FLT x6 = obj_qj * obj_qj;
	const GEN_FLT x7 = obj_qi * obj_qi;
	const GEN_FLT x8 = 1. / sqrt(1e-11 + x7 + x6 + (obj_qw * obj_qw) + x5);
	const GEN_FLT x9 = x8 * obj_qi;
	const GEN_FLT x10 = x0 * obj_qi;
	const GEN_FLT x11 = x2 * obj_qw;
	const GEN_FLT x12 = (x11 + x10) * scale;
	const GEN_FLT x13 = x8 * x12;
	const GEN_FLT x14 = -2 * x5;
	const GEN_FLT x15 = -2 * x6;
	const GEN_FLT x16 = (1 + x15 + x14) * scale;
	const GEN_FLT x17 = x8 * obj_qk;
	const GEN_FLT x18 = (-1 * x17 * x16) + (x4 * x9) + (x13 * obj_qw);
	const GEN_FLT x19 = x2 * x8;
	const GEN_FLT x20 = x8 * obj_qj;
	const GEN_FLT x21 = x8 * obj_qw;
	const GEN_FLT x22 = (-1 * x13 * obj_qi) + (x20 * x16) + (x4 * x21);
	const GEN_FLT x23 = x0 * x8;
	const GEN_FLT x24 = x2 * obj_qj;
	const GEN_FLT x25 = 2 * obj_qi;
	const GEN_FLT x26 = x25 * obj_qw;
	const GEN_FLT x27 = (x26 + x24) * scale;
	const GEN_FLT x28 = x8 * x27;
	const GEN_FLT x29 = 1 + (-2 * x7);
	const GEN_FLT x30 = (x29 + x14) * scale;
	const GEN_FLT x31 = (x10 + (-1 * x11)) * scale;
	const GEN_FLT x32 = x8 * x31;
	const GEN_FLT x33 = (-1 * x32 * obj_qk) + (x28 * obj_qi) + (x30 * x21);
	const GEN_FLT x34 = (-1 * x9 * x30) + (x32 * obj_qj) + (x28 * obj_qw);
	const GEN_FLT x35 = (x29 + x15) * scale;
	const GEN_FLT x36 = (x24 + (-1 * x26)) * scale;
	const GEN_FLT x37 = x8 * x36;
	const GEN_FLT x38 = (x1 + x3) * scale;
	const GEN_FLT x39 = (-1 * x38 * x17) + (x9 * x35) + (x37 * obj_qw);
	const GEN_FLT x40 = (-1 * x37 * obj_qi) + (x38 * x20) + (x35 * x21);
	const GEN_FLT x41 = x8 * x25;
	const GEN_FLT x42 = (-1 * x4 * x20) + (x13 * obj_qk) + (x21 * x16);
	const GEN_FLT x43 = (-1 * x28 * obj_qj) + (x30 * x17) + (x32 * obj_qw);
	const GEN_FLT x44 = (-1 * x35 * x20) + (x37 * obj_qk) + (x38 * x21);
	out[0] = (x19 * x18) + (-1 * x22 * x23) + x16;
	out[1] = (x33 * x19) + (-1 * x34 * x23) + x31;
	out[2] = (-1 * x40 * x23) + (x39 * x19) + x38;
	out[3] = (x41 * x22) + (-1 * x42 * x19) + x12;
	out[4] = (-1 * x43 * x19) + (x41 * x34) + x30;
	out[5] = (x40 * x41) + (-1 * x44 * x19) + x36;
	out[6] = (-1 * x41 * x18) + (x42 * x23) + x4;
	out[7] = (-1 * x41 * x33) + (x43 * x23) + x27;
	out[8] = (-1 * x41 * x39) + (x44 * x23) + x35;
}

// Jacobian of scale_sensor_pt wrt [obj_px, obj_py, obj_pz, obj_qw, obj_qi, obj_qj, obj_qk]
static inline void gen_scale_sensor_pt_jac_obj_p(FLT *out, const FLT *sensor_pt, const SurvivePose *obj_p,
												 const FLT scale) {
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT x0 = obj_qj * obj_qj;
	const GEN_FLT x1 = obj_qw * obj_qw;
	const GEN_FLT x2 = obj_qk * obj_qk;
	const GEN_FLT x3 = obj_qi * obj_qi;
	const GEN_FLT x4 = 1e-11 + x3 + x0 + x1 + x2;
	const GEN_FLT x5 = 1. / x4;
	const GEN_FLT x6 = 2 * x5;
	const GEN_FLT x7 = x0 * x6;
	const GEN_FLT x8 = -1 + x7 + scale + (-1 * x7 * scale);
	const GEN_FLT x9 = x2 * x6;
	const GEN_FLT x10 = x9 + (-1 * x9 * scale);
	const GEN_FLT x11 = 2 * obj_qk;
	const GEN_FLT x12 = x5 * x11;
	const GEN_FLT x13 = x12 * obj_qw;
	const GEN_FLT x14 = x13 * scale;
	const GEN_FLT x15 = 2 * obj_qj;
	const GEN_FLT x16 = x5 * x15;
	const GEN_FLT x17 = x16 * obj_qi;
	const GEN_FLT x18 = (-1 * x17) + (x17 * scale);
	const GEN_FLT x19 = x16 * obj_qw;
	const GEN_FLT x20 = x19 * scale;
	const GEN_FLT x21 = x12 * obj_qi;
	const GEN_FLT x22 = (-1 * x21) + (x21 * scale);
	const GEN_FLT x23 = obj_qw * sensor_x;
	const GEN_FLT x24 = obj_qj * sensor_z;
	const GEN_FLT x25 = obj_qk * sensor_y;
	const GEN_FLT x26 = (-1 * x25) + x23 + x24;
	const GEN_FLT x27 = obj_qk * sensor_x;
	const GEN_FLT x28 = obj_qw * sensor_y;
	const GEN_FLT x29 = obj_qi * sensor_z;
	const GEN_FLT x30 = (-1 * x29) + x27 + x28;
	const GEN_FLT x31 = (2 * ((x30 * obj_qi) + (-1 * x26 * obj_qj))) + obj_pz + sensor_z;
	const GEN_FLT x32 = 1. / (x4 * sqrt(x4));
	const GEN_FLT x33 = x32 * scale;
	const GEN_FLT x34 = x31 * x33;
	const GEN_FLT x35 = obj_qw * obj_qi;
	const GEN_FLT x36 = -1 * x34 * x35;
	const GEN_FLT x37 = 2 * x29;
	const GEN_FLT x38 = 2 * x27;
	const GEN_FLT x39 = (x38 + (-1 * x37)) * scale;
	const GEN_FLT x40 = 1. / sqrt(x4);
	const GEN_FLT x41 = x40 * obj_qw;
	const GEN_FLT x42 = obj_qw * sensor_z;
	const GEN_FLT x43 = obj_qi * sensor_y;
	const GEN_FLT x44 = obj_qj * sensor_x;
	const GEN_FLT x45 = (-1 * x44) + x42 + x43;
	const GEN_FLT x46 = (2 * ((x26 * obj_qk) + (-1 * x45 * obj_qi))) + obj_py + sensor_y;
	const GEN_FLT x47 = x40 * scale;
	const GEN_FLT x48 = x46 * x47;
	const GEN_FLT x49 = 2 * x44;
	const GEN_FLT x50 = 2 * x43;
	const GEN_FLT x51 = (x50 + (-1 * x49)) * scale;
	const GEN_FLT x52 = x40 * obj_qi;
	const GEN_FLT x53 = x46 * x33;
	const GEN_FLT x54 = 2 * x25;
	const GEN_FLT x55 = 2 * x24;
	const GEN_FLT x56 = (x55 + (-1 * x54)) * scale;
	const GEN_FLT x57 = x40 * obj_qk;
	const GEN_FLT x58 = (2 * ((x45 * obj_qj) + (-1 * x30 * obj_qk))) + obj_px + sensor_x;
	const GEN_FLT x59 = x58 * x33;
	const GEN_FLT x60 = x59 * obj_qk;
	const GEN_FLT x61 = x60 * obj_qw;
	const GEN_FLT x62 = 2 * x40;
	const GEN_FLT x63 = x62 * ((-1 * x57 * x56) + x61 + (x41 * x39) + (-1 * x1 * x53) + x36 + (x52 * x51) + x48);
	const GEN_FLT x64 = x47 * x31;
	const GEN_FLT x65 = x58 * x47;
	const GEN_FLT x66 = (x65 * obj_qj) + (x64 * obj_qw) + (-1 * x48 * obj_qi);
	const GEN_FLT x67 = x32 * x15;
	const GEN_FLT x68 = x67 * obj_qw;
	const GEN_FLT x69 = obj_qw * obj_qj;
	const GEN_FLT x70 = -1 * x69 * x59;
	const GEN_FLT x71 = x40 * obj_qj;
	const GEN_FLT x72 = x53 * x35;
	const GEN_FLT x73 = x62 * ((-1 * x52 * x39) + (x71 * x56) + x70 + (x51 * x41) + x72 + x64 + (-1 * x1 * x34));
	const GEN_FLT x74 = (x64 * obj_qi) + (x48 * obj_qw) + (-1 * x65 * obj_qk);
	const GEN_FLT x75 = x32 * x11;
	const GEN_FLT x76 = x75 * obj_qw;
	const GEN_FLT x77 = x40 * obj_py;
	const GEN_FLT x78 = x40 * obj_px;
	const GEN_FLT x79 = x40 * obj_pz;
	const GEN_FLT x80 = (x79 * obj_qi) + (x77 * obj_qw) + (-1 * x78 * obj_qk);
	const GEN_FLT x81 = x80 * obj_qw;
	const GEN_FLT x82 = (x79 * obj_qw) + (x78 * obj_qj) + (-1 * x77 * obj_qi);
	const GEN_FLT x83 = x32 * obj_px;
	const GEN_FLT x84 = x83 * obj_qk;
	const GEN_FLT x85 = x84 * obj_qw;
	const GEN_FLT x86 = x32 * obj_pz;
	const GEN_FLT x87 = x86 * obj_qw;
	const GEN_FLT x88 = -1 * x87 * obj_qi;
	const GEN_FLT x89 = x32 * obj_py;
	const GEN_FLT x90 = (-1 * x1 * x89) + x77 + x85 + x88;
	const GEN_FLT x91 = x62 * obj_qk;
	const GEN_FLT x92 = x83 * obj_qw;
	const GEN_FLT x93 = -1 * x92 * obj_qj;
	const GEN_FLT x94 = x89 * obj_qw;
	const GEN_FLT x95 = x94 * obj_qi;
	const GEN_FLT x96 = x62 * (x79 + (-1 * x1 * x86) + x93 + x95);
	const GEN_FLT x97 = 2 * x28;
	const GEN_FLT x98 = (x97 + x38 + (-4 * x29)) * scale;
	const GEN_FLT x99 = x11 * sensor_z;
	const GEN_FLT x100 = x15 * sensor_y;
	const GEN_FLT x101 = (x100 + x99) * scale;
	const GEN_FLT x102 = 2 * x42;
	const GEN_FLT x103 = scale * ((-4 * x43) + x49 + (-1 * x102));
	const GEN_FLT x104 =
		x62 * ((x52 * x98) + (-1 * x3 * x34) + x64 + (-1 * x72) + (-1 * x57 * x101) + (x60 * obj_qi) + (x41 * x103));
	const GEN_FLT x105 = x67 * obj_qi;
	const GEN_FLT x106 = x66 * x105;
	const GEN_FLT x107 = x75 * obj_qi;
	const GEN_FLT x108 = x80 * x107;
	const GEN_FLT x109 = x40 * x101;
	const GEN_FLT x110 = obj_qj * obj_qi;
	const GEN_FLT x111 =
		x62 * (x36 + (x109 * obj_qj) + (-1 * x52 * x103) + (x98 * x41) + (-1 * x59 * x110) + (x3 * x53) + (-1 * x48));
	const GEN_FLT x112 = x82 * x105;
	const GEN_FLT x113 = x74 * x107;
	const GEN_FLT x114 = (x3 * x89) + (-1 * x83 * x110) + x88 + (-1 * x77);
	const GEN_FLT x115 = x62 * obj_qj;
	const GEN_FLT x116 = x62 * ((x84 * obj_qi) + (-1 * x95) + x79 + (-1 * x3 * x86));
	const GEN_FLT x117 = x34 * x110;
	const GEN_FLT x118 = 2 * obj_qi;
	const GEN_FLT x119 = x118 * sensor_x;
	const GEN_FLT x120 = (x99 + x119) * scale;
	const GEN_FLT x121 = x40 * x120;
	const GEN_FLT x122 = scale * ((-4 * x44) + x50 + x102);
	const GEN_FLT x123 = obj_qk * obj_qj;
	const GEN_FLT x124 = x59 * x123;
	const GEN_FLT x125 = 2 * x23;
	const GEN_FLT x126 = scale * ((-4 * x24) + (-1 * x125) + x54);
	const GEN_FLT x127 = (-1 * x117) + (-1 * x57 * x122) + (x52 * x126) + (-1 * x69 * x53) + x124 + (x121 * obj_qw);
	const GEN_FLT x128 = x69 * x34;
	const GEN_FLT x129 =
		(x71 * x122) + (-1 * x121 * obj_qi) + (x41 * x126) + (-1 * x0 * x59) + (x53 * x110) + x65 + (-1 * x128);
	const GEN_FLT x130 = x75 * obj_qj;
	const GEN_FLT x131 = 2 * x32;
	const GEN_FLT x132 = x66 * x131;
	const GEN_FLT x133 = x62 * x66;
	const GEN_FLT x134 = x87 * obj_qj;
	const GEN_FLT x135 = (x89 * x110) + x78 + (-1 * x134) + (-1 * x0 * x83);
	const GEN_FLT x136 = x82 * x62;
	const GEN_FLT x137 = x82 * x131;
	const GEN_FLT x138 = x86 * x110;
	const GEN_FLT x139 = x84 * obj_qj;
	const GEN_FLT x140 = x62 * (x139 + (-1 * x138) + (-1 * x94 * obj_qj));
	const GEN_FLT x141 = x74 * x62;
	const GEN_FLT x142 = x74 * x131;
	const GEN_FLT x143 = (x119 + x100) * scale;
	const GEN_FLT x144 = x34 * obj_qk;
	const GEN_FLT x145 = scale * (x55 + x125 + (-4 * x25));
	const GEN_FLT x146 = x53 * obj_qk;
	const GEN_FLT x147 = x146 * obj_qi;
	const GEN_FLT x148 = ((-1 * x97) + (-4 * x27) + x37) * scale;
	const GEN_FLT x149 = (-1 * x144 * obj_qw) + (-1 * x124) + (x41 * x143) + x147 + (x71 * x148) + (-1 * x52 * x145);
	const GEN_FLT x150 = -1 * x146 * obj_qw;
	const GEN_FLT x151 =
		x62 * ((x2 * x59) + (x52 * x143) + (x41 * x145) + (-1 * x57 * x148) + x150 + (-1 * x144 * obj_qi) + (-1 * x65));
	const GEN_FLT x152 = x89 * obj_qk;
	const GEN_FLT x153 = x152 * obj_qi;
	const GEN_FLT x154 = x62 * ((-1 * x139) + x153 + (-1 * x87 * obj_qk));
	const GEN_FLT x155 = x86 * obj_qk;
	const GEN_FLT x156 = -1 * x94 * obj_qk;
	const GEN_FLT x157 = (x2 * x83) + x156 + (-1 * x155 * obj_qi) + (-1 * x78);
	const GEN_FLT x158 = x80 * x62;
	const GEN_FLT x159 = x80 * x131;
	const GEN_FLT x160 = x3 * x6;
	const GEN_FLT x161 = x160 + (-1 * x160 * scale);
	const GEN_FLT x162 = x6 * x35;
	const GEN_FLT x163 = x162 * scale;
	const GEN_FLT x164 = x12 * obj_qj;
	const GEN_FLT x165 = (-1 * x164) + (x164 * scale);
	const GEN_FLT x166 = (x48 * obj_qk) + (x65 * obj_qw) + (-1 * x64 * obj_qj);
	const GEN_FLT x167 = x75 * x166;
	const GEN_FLT x168 = (x56 * x41) + x128 + x65 + (x57 * x39) + (-1 * x1 * x59) + (-1 * x71 * x51) + x150;
	const GEN_FLT x169 = x32 * x118;
	const GEN_FLT x170 = x169 * obj_qw;
	const GEN_FLT x171 = x134 + (-1 * x1 * x83) + x156 + x78;
	const GEN_FLT x172 = (x77 * obj_qk) + (-1 * x79 * obj_qj) + (x78 * obj_qw);
	const GEN_FLT x173 = x166 * obj_qi;
	const GEN_FLT x174 =
		x62 * (x117 + (-1 * x59 * x35) + (-1 * x147) + (x57 * x103) + (-1 * x71 * x98) + (x109 * obj_qw));
	const GEN_FLT x175 = (-1 * x92 * obj_qi) + (-1 * x153) + x138;
	const GEN_FLT x176 = x62 * obj_qi;
	const GEN_FLT x177 = x167 * obj_qj;
	const GEN_FLT x178 = x172 * x130;
	const GEN_FLT x179 =
		(-1 * x64) + (-1 * x53 * x123) + (x57 * x120) + x70 + (x0 * x34) + (-1 * x71 * x126) + (x41 * x122);
	const GEN_FLT x180 = x62 * ((x0 * x86) + (-1 * x152 * obj_qj) + (-1 * x79) + x93);
	const GEN_FLT x181 = x166 * x131;
	const GEN_FLT x182 = x62 * x166;
	const GEN_FLT x183 = x62 * x172;
	const GEN_FLT x184 =
		(-1 * x61) + x48 + (-1 * x2 * x53) + (x57 * x145) + (x34 * x123) + (x41 * x148) + (-1 * x71 * x143);
	const GEN_FLT x185 = x172 * x131;
	const GEN_FLT x186 = (-1 * x85) + x77 + (x155 * obj_qj) + (-1 * x2 * x89);
	out[0] = x10 + x8;
	out[1] = x18 + x14 + (-1 * x13);
	out[2] = x22 + (-1 * x20) + x19;
	out[3] = (-1 * x82 * x68) + (x81 * x75) + (x68 * x66) + (x63 * obj_qk) + (x96 * obj_qj) + (-1 * x73 * obj_qj) +
			 (-1 * x74 * x76) + (-1 * x91 * x90) + x56;
	out[4] = (x114 * x115) + (-1 * x113) + (-1 * x116 * obj_qk) + x106 + (x104 * obj_qk) + x108 + (-1 * x112) + x101 +
			 (-1 * x111 * obj_qj);
	out[5] = x122 + x136 + (-1 * x140 * obj_qk) + (-1 * x133) + (-1 * x115 * x129) + (x80 * x130) + (x115 * x135) +
			 (x91 * x127) + (-1 * x74 * x130) + (-1 * x0 * x137) + (x0 * x132);
	out[6] = x148 + (x154 * obj_qj) + (-1 * x82 * x130) + (-1 * x2 * x142) + (x2 * x159) + x141 + (x151 * obj_qk) +
			 (-1 * x158) + (-1 * x91 * x157) + (-1 * x115 * x149) + (x66 * x130);
	out[7] = x18 + (-1 * x14) + x13;
	out[8] = -1 + x161 + scale + x10;
	out[9] = x165 + (-1 * x162) + x163;
	out[10] = (-1 * x96 * obj_qi) + (x82 * x170) + (x167 * obj_qw) + (x73 * obj_qi) + x39 + (-1 * x91 * x168) +
			  (-1 * x76 * x172) + (x91 * x171) + (-1 * x66 * x170);
	out[11] = x103 + (-1 * x136) + (-1 * x3 * x132) + (-1 * x114 * x176) + x133 + (x3 * x137) + (-1 * x174 * obj_qk) +
			  (-1 * x107 * x172) + (x91 * x175) + (x75 * x173) + (x111 * obj_qi);
	out[12] = (-1 * x176 * x135) + (x180 * obj_qk) + x177 + (-1 * x106) + x112 + (x129 * x176) + (-1 * x91 * x179) +
			  x120 + (-1 * x178);
	out[13] = (x91 * x186) + (x82 * x107) + (-1 * x66 * x107) + x145 + (-1 * x2 * x185) + x183 + (x176 * x149) +
			  (-1 * x91 * x184) + (-1 * x154 * obj_qi) + (x2 * x181) + (-1 * x182);
	out[14] = x22 + x20 + (-1 * x19);
	out[15] = x165 + (-1 * x163) + x162;
	out[16] = x161 + x8;
	out[17] = x51 + (x68 * x172) + (-1 * x81 * x169) + (-1 * x68 * x166) + (x115 * x168) + (-1 * x63 * obj_qi) +
			  (x90 * x176) + (x74 * x170) + (-1 * x115 * x171);
	out[18] = (x116 * obj_qi) + (-1 * x141) + (-1 * x115 * x175) + x158 + (x105 * x172) + (-1 * x67 * x173) +
			  (x174 * obj_qj) + (-1 * x3 * x159) + (x3 * x142) + x98 + (-1 * x104 * obj_qi);
	out[19] = (-1 * x80 * x105) + (-1 * x180 * obj_qj) + (x140 * obj_qi) + (x74 * x105) + (-1 * x0 * x181) +
			  (x115 * x179) + (x0 * x185) + x126 + x182 + (-1 * x127 * x176) + (-1 * x183);
	out[20] = x143 + (x176 * x157) + x113 + (-1 * x177) + (-1 * x115 * x186) + (x115 * x184) + (-1 * x151 * obj_qi) +
			  x178 + (-1 * x108);
}

// Jacobian of scale_sensor_pt wrt [scale]
static inline void gen_scale_sensor_pt_jac_scale(FLT *out, const FLT *sensor_pt, const SurvivePose *obj_p,
												 const FLT scale) {
	const GEN_FLT sensor_x = sensor_pt[0];
	const GEN_FLT sensor_y = sensor_pt[1];
	const GEN_FLT sensor_z = sensor_pt[2];
	const GEN_FLT obj_px = (*obj_p).Pos[0];
	const GEN_FLT obj_py = (*obj_p).Pos[1];
	const GEN_FLT obj_pz = (*obj_p).Pos[2];
	const GEN_FLT obj_qw = (*obj_p).Rot[0];
	const GEN_FLT obj_qi = (*obj_p).Rot[1];
	const GEN_FLT obj_qj = (*obj_p).Rot[2];
	const GEN_FLT obj_qk = (*obj_p).Rot[3];
	const GEN_FLT x0 = 1. / sqrt(1e-11 + (obj_qw * obj_qw) + (obj_qi * obj_qi) + (obj_qj * obj_qj) + (obj_qk * obj_qk));
	const GEN_FLT x1 = (obj_qw * sensor_x) + (-1 * obj_qk * sensor_y) + (obj_qj * sensor_z);
	const GEN_FLT x2 = (obj_qk * sensor_x) + (-1 * obj_qi * sensor_z) + (obj_qw * sensor_y);
	const GEN_FLT x3 = obj_pz + (2 * ((x2 * obj_qi) + (-1 * x1 * obj_qj))) + sensor_z;
	const GEN_FLT x4 = x0 * x3;
	const GEN_FLT x5 = (-1 * obj_qj * sensor_x) + (obj_qw * sensor_z) + (obj_qi * sensor_y);
	const GEN_FLT x6 = (2 * ((x1 * obj_qk) + (-1 * x5 * obj_qi))) + obj_py + sensor_y;
	const GEN_FLT x7 = x0 * x6;
	const GEN_FLT x8 = (2 * ((x5 * obj_qj) + (-1 * x2 * obj_qk))) + obj_px + sensor_x;
	const GEN_FLT x9 = x0 * x8;
	const GEN_FLT x10 = 2 * x0;
	const GEN_FLT x11 = x10 * ((-1 * x9 * obj_qk) + (x4 * obj_qi) + (x7 * obj_qw));
	const GEN_FLT x12 = x10 * ((-1 * x7 * obj_qi) + (x9 * obj_qj) + (x4 * obj_qw));
	const GEN_FLT x13 = x10 * ((-1 * x4 * obj_qj) + (x9 * obj_qw) + (x7 * obj_qk));
	out[0] = x8 + (x11 * obj_qk) + (-1 * x12 * obj_qj);
	out[1] = x6 + (-1 * x13 * obj_qk) + (x12 * obj_qi);
	out[2] = x3 + (x13 * obj_qj) + (-1 * x11 * obj_qi);
}
