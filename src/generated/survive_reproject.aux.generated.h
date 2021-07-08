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

static inline void gen_axisangle2pose(FLT *out, const LinmathAxisAnglePose *obj_p_axisangle) {
	const GEN_FLT obj_px = (*obj_p_axisangle).Pos[0];
	const GEN_FLT obj_py = (*obj_p_axisangle).Pos[1];
	const GEN_FLT obj_pz = (*obj_p_axisangle).Pos[2];
	const GEN_FLT obj_qi = (*obj_p_axisangle).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p_axisangle).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p_axisangle).AxisAngleRot[2];
	const GEN_FLT x0 = obj_qk * obj_qk;
	const GEN_FLT x1 = obj_qi * obj_qi;
	const GEN_FLT x2 = obj_qj * obj_qj;
	const GEN_FLT x3 = 1e-10 + x2 + x0 + x1;
	const GEN_FLT x4 = sqrt(x3);
	const GEN_FLT x5 = 0.5 * x4;
	const GEN_FLT x6 = sin(x5);
	const GEN_FLT x7 = (1. / x3) * (x6 * x6);
	const GEN_FLT x8 = cos(x5);
	const GEN_FLT x9 = 1. / sqrt(1e-11 + (x1 * x7) + (x2 * x7) + (x0 * x7) + (x8 * x8));
	const GEN_FLT x10 = (1. / x4) * x6 * x9;
	out[0] = obj_px;
	out[1] = obj_py;
	out[2] = obj_pz;
	out[3] = x8 * x9;
	out[4] = x10 * obj_qi;
	out[5] = x10 * obj_qj;
	out[6] = x10 * obj_qk;
}

// Jacobian of axisangle2pose wrt [obj_px, obj_py, obj_pz, obj_qi, obj_qj, obj_qk]
static inline void gen_axisangle2pose_jac_obj_p_axisangle(FLT *out, const LinmathAxisAnglePose *obj_p_axisangle) {
	const GEN_FLT obj_px = (*obj_p_axisangle).Pos[0];
	const GEN_FLT obj_py = (*obj_p_axisangle).Pos[1];
	const GEN_FLT obj_pz = (*obj_p_axisangle).Pos[2];
	const GEN_FLT obj_qi = (*obj_p_axisangle).AxisAngleRot[0];
	const GEN_FLT obj_qj = (*obj_p_axisangle).AxisAngleRot[1];
	const GEN_FLT obj_qk = (*obj_p_axisangle).AxisAngleRot[2];

	out[0] = 1;
	out[1] = 0;
	out[2] = 0;
	out[3] = 0;
	out[4] = 0;
	out[5] = 0;
	out[6] = 0;
	out[7] = 1;
	out[8] = 0;
	out[9] = 0;
	out[10] = 0;
	out[11] = 0;
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

	out[0] = obj_qw;
	out[1] = -1 * obj_qi;
	out[2] = -1 * obj_qj;
	out[3] = -1 * obj_qk;
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

	out[0] = 0;
	out[1] = 0;
	out[2] = 0;
	out[3] = 1;
	out[4] = 0;
	out[5] = 0;
	out[6] = 0;
	out[7] = 0;
	out[8] = 0;
	out[9] = 0;
	out[10] = 0;
	out[11] = -1;
	out[12] = 0;
	out[13] = 0;
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
