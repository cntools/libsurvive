#include "barycentric_svd.h"
#include "math.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include "survive.h"
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <cnmatrix/cn_matrix.h>
#include <malloc.h>
#endif

static void bc_svd_choose_control_points(bc_svd *self) {
	// Take C0 as the reference points centroid:
	self->setup.control_points[0][0] = self->setup.control_points[0][1] = self->setup.control_points[0][2] = 0;
	size_t valid_points = 0;
	for (int i = 0; i < self->setup.obj_cnt; i++)
		for (int j = 0; j < 3; j++) {
			self->setup.control_points[0][j] += self->setup.obj_pts[i][j];
		}

	for (int j = 0; j < 3; j++)
		self->setup.control_points[0][j] /= self->setup.obj_cnt;

	// Note: The original ePNP paper / implementation has a sophisticated PCA scheme for control points which is
	// supposed to work better with SFM problems. The scheme can fail pretty hard with vive devices though. The sensor
	// placement for the HMD is relatively planar and so the control points conform to that plane, and with spotty data
	// you get degenerate cases. So here we give it a relatively random rotation but centered and it seems pretty robust
	// overall
	FLT R[9];
	LinmathQuat q = {1, 1, 1, 1};
	quatnormalize(q, q);
	quattomatrix33(R, q);
	CN_CREATE_STACK_MAT(UCt, 3, 3);

	cn_copy_in_row_major(&UCt, R, 3);

	for (int i = 1; i < 4; i++) {
		FLT k = sqrt(1. / (FLT)self->setup.obj_cnt);
		for (int j = 0; j < 3; j++) {
			FLT uct_val = cnMatrixGet(&UCt, i - 1, j);
#ifndef CN_MATRIX_IS_COL_MAJOR
			assert(uct_val == CN_FLT_PTR(&UCt)[3 * (i - 1) + j]);
#endif
			self->setup.control_points[i][j] = self->setup.control_points[0][j] + k * uct_val;
		}
	}

	CN_FREE_STACK_MAT(UCt);
}

static void bc_svd_compute_barycentric_coordinates(bc_svd *self) {
	FLT cc[3 * 3] = {0}, cc_inv[3 * 3] = {0};
	CnMat CC = cnMat(3, 3, cc);
	CnMat CC_inv = cnMat(3, 3, cc_inv);

	for (int i = 0; i < 3; i++)
		for (int j = 1; j < 4; j++) {
			cnMatrixSet(&CC, i, j - 1, self->setup.control_points[j][i] - self->setup.control_points[0][i]);
		}

	cnInvert(&CC, &CC_inv, 1);

	FLT *ci = cc_inv;
	for (int i = 0; i < self->setup.obj_cnt; i++) {
		const FLT *pi = self->setup.obj_pts[i];
		FLT *a = self->setup.alphas[i];

		for (int j = 0; j < 3; j++)
			a[1 + j] = cnMatrixGet(&CC_inv, j, 0) * (pi[0] - self->setup.control_points[0][0]) +
					   cnMatrixGet(&CC_inv, j, 1) * (pi[1] - self->setup.control_points[0][1]) +
					   cnMatrixGet(&CC_inv, j, 2) * (pi[2] - self->setup.control_points[0][2]);
		a[0] = 1.0f - a[1] - a[2] - a[3];
	}
}

void bc_svd_bc_svd(bc_svd *self, void *user, bc_svd_fill_M_fn fillFn, const LinmathPoint3d *obj_pts, size_t obj_cnt) {
	*self = (bc_svd){0};

	self->setup.user = user;
	self->setup.fillFn = fillFn;

	self->setup.obj_cnt = obj_cnt;
	self->setup.obj_pts = obj_pts;

	self->setup.alphas = SV_CALLOC_N(obj_cnt, sizeof(self->setup.alphas[0]));
	self->object_pts_in_camera = SV_CALLOC_N(obj_cnt, sizeof(self->setup.alphas[0]));

	bc_svd_choose_control_points(self);
	bc_svd_compute_barycentric_coordinates(self);
}

void bc_svd_dtor(bc_svd *self) {
	free(self->setup.alphas);
	free(self->object_pts_in_camera);
	free(self->meas);
}

static FLT bc_svd_compute_R_and_t(bc_svd *self, const CnMat *ut, const FLT *betas, FLT R[3][3], FLT t[3]);

FLT dot(const FLT *v1, const FLT *v2) { return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]; }

FLT dist2(const FLT *p1, const FLT *p2) {
	return (p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]) + (p1[2] - p2[2]) * (p1[2] - p2[2]);
}

void bc_svd_compute_rho(bc_svd *self, FLT *rho) {
	rho[0] = dist2(self->setup.control_points[0], self->setup.control_points[1]);
	rho[1] = dist2(self->setup.control_points[0], self->setup.control_points[2]);
	rho[2] = dist2(self->setup.control_points[0], self->setup.control_points[3]);
	rho[3] = dist2(self->setup.control_points[1], self->setup.control_points[2]);
	rho[4] = dist2(self->setup.control_points[1], self->setup.control_points[3]);
	rho[5] = dist2(self->setup.control_points[2], self->setup.control_points[3]);
}

void bc_svd_reset_correspondences(bc_svd *self) { self->meas_cnt = 0; }

void bc_svd_add_single_correspondence(bc_svd *self, size_t idx, int axis, FLT angle) {
	if (isnan(angle))
		return;

	if (self->meas_space <= self->meas_cnt) {
		self->meas_space = self->meas_space * 2 + 1;
		self->meas = SV_REALLOC(self->meas, sizeof(self->meas[0]) * self->meas_space);
	}

	assert(idx < self->setup.obj_cnt);
	self->meas[self->meas_cnt] = (bc_svd_meas_t){.angle = angle, .axis = axis, .obj_idx = idx};

	self->meas_cnt++;
}
void bc_svd_add_correspondence(bc_svd *self, size_t idx, FLT u, FLT v) {
	for (int i = 0; i < 2; i++) {
		FLT angle = i == 0 ? u : v;
		bc_svd_add_single_correspondence(self, idx, i, angle);
	}
}

void bc_svd_fill_M(bc_svd *self, CnMat *_M, const int row, const FLT *as, int axis, FLT angle) {
	FLT *M = CN_RAW_PTR(_M) + row * 12;

	FLT eq[3] = {NAN, NAN, NAN};
	self->setup.fillFn(self->setup.user, eq, axis, angle);

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			cnMatrixSet(_M, row, i * 3 + j, eq[j] * as[i]);
			assert(isfinite(cnMatrixGet(_M, row, i * 3 + j)));
		}
	}
}

static void bc_svd_compute_ccs(bc_svd *self, const FLT *betas, const CnMat *ut) {
	for (int i = 0; i < 4; i++)
		self->control_points_in_camera[i][0] = self->control_points_in_camera[i][1] =
			self->control_points_in_camera[i][2] = 0.0f;

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++)
			for (int k = 0; k < 3; k++) {
				FLT val = cnMatrixGet(ut, (11 - i), 3 * j + k);
				self->control_points_in_camera[j][k] += betas[i] * val;
			}
	}
}

void bc_svd_compute_pcs(bc_svd *self) {
	for (int i = 0; i < self->setup.obj_cnt; i++) {
		FLT *a = self->setup.alphas[i];
		FLT *pc = self->object_pts_in_camera[i];

		for (int j = 0; j < 3; j++)
			pc[j] = a[0] * self->control_points_in_camera[0][j] + a[1] * self->control_points_in_camera[1][j] +
					a[2] * self->control_points_in_camera[2][j] + a[3] * self->control_points_in_camera[3][j];
	}
}

static void bc_svd_compute_L_6x10(bc_svd *self, const CnMat *ut, CnMat *L_6x10) {
	FLT dv[4][6][3];

	for (int i = 0; i < 4; i++) {
		int a = 0, b = 1;
		for (int j = 0; j < 6; j++) {

			for (int h = 0; h < 3; h++) {
				dv[i][j][h] = cnMatrixGet(ut, 11 - i, 3 * a + h) - cnMatrixGet(ut, 11 - i, 3 * b + h);
			}

			b++;
			if (b > 3) {
				a++;
				b = a + 1;
			}
		}
	}

	for (int i = 0; i < 6; i++) {
		*cnMatrixPtr(L_6x10, i, 0) = dot(dv[0][i], dv[0][i]);
		*cnMatrixPtr(L_6x10, i, 1) = 2.0f * dot(dv[0][i], dv[1][i]);
		*cnMatrixPtr(L_6x10, i, 2) = dot(dv[1][i], dv[1][i]);
		*cnMatrixPtr(L_6x10, i, 3) = 2.0f * dot(dv[0][i], dv[2][i]);
		*cnMatrixPtr(L_6x10, i, 4) = 2.0f * dot(dv[1][i], dv[2][i]);
		*cnMatrixPtr(L_6x10, i, 5) = dot(dv[2][i], dv[2][i]);
		*cnMatrixPtr(L_6x10, i, 6) = 2.0f * dot(dv[0][i], dv[3][i]);
		*cnMatrixPtr(L_6x10, i, 7) = 2.0f * dot(dv[1][i], dv[3][i]);
		*cnMatrixPtr(L_6x10, i, 8) = 2.0f * dot(dv[2][i], dv[3][i]);
		*cnMatrixPtr(L_6x10, i, 9) = dot(dv[3][i], dv[3][i]);
	}
}

void find_betas_approx_1(const CnMat *L_6x10, const CnMat *Rho, FLT *betas) {
	FLT l_6x4[6 * 4], b4[4];
	CnMat L_6x4 = cnMat(6, 4, l_6x4);
	CnMat B4 = cnMat(4, 1, b4);

	for (int i = 0; i < 6; i++) {
		cnMatrixSet(&L_6x4, i, 0, cnMatrixGet(L_6x10, i, 0));
		cnMatrixSet(&L_6x4, i, 1, cnMatrixGet(L_6x10, i, 1));
		cnMatrixSet(&L_6x4, i, 2, cnMatrixGet(L_6x10, i, 3));
		cnMatrixSet(&L_6x4, i, 3, cnMatrixGet(L_6x10, i, 6));
	}

	cnSolve(&L_6x4, Rho, &B4, CN_INVERT_METHOD_SVD);

	if (b4[0] < 0) {
		betas[0] = sqrt(-b4[0]);
		betas[1] = -b4[1] / betas[0];
		betas[2] = -b4[2] / betas[0];
		betas[3] = -b4[3] / betas[0];
	} else {
		betas[0] = sqrt(b4[0]);
		betas[1] = b4[1] / betas[0];
		betas[2] = b4[2] / betas[0];
		betas[3] = b4[3] / betas[0];
	}
}

static void compute_A_and_b_gauss_newton(const CnMat *L_6x10, const FLT *rho, FLT betas[4], CnMat *A, CnMat *b) {
	for (int i = 0; i < 6; i++) {
		*cnMatrixPtr(A, i, 0) = 2 * cnMatrixGet(L_6x10, i, 0) * betas[0] + cnMatrixGet(L_6x10, i, 1) * betas[1] +
								cnMatrixGet(L_6x10, i, 3) * betas[2] + cnMatrixGet(L_6x10, i, 6) * betas[3];
		*cnMatrixPtr(A, i, 1) = cnMatrixGet(L_6x10, i, 1) * betas[0] + 2 * cnMatrixGet(L_6x10, i, 2) * betas[1] +
								cnMatrixGet(L_6x10, i, 4) * betas[2] + cnMatrixGet(L_6x10, i, 7) * betas[3];
		*cnMatrixPtr(A, i, 2) = cnMatrixGet(L_6x10, i, 3) * betas[0] + cnMatrixGet(L_6x10, i, 4) * betas[1] +
								2 * cnMatrixGet(L_6x10, i, 5) * betas[2] + cnMatrixGet(L_6x10, i, 8) * betas[3];
		*cnMatrixPtr(A, i, 3) = cnMatrixGet(L_6x10, i, 6) * betas[0] + cnMatrixGet(L_6x10, i, 7) * betas[1] +
								cnMatrixGet(L_6x10, i, 8) * betas[2] + 2 * cnMatrixGet(L_6x10, i, 9) * betas[3];

		cnMatrixSet(
			b, i, 0,
			rho[i] -
				(cnMatrixGet(L_6x10, i, 0) * betas[0] * betas[0] + cnMatrixGet(L_6x10, i, 1) * betas[0] * betas[1] +
				 cnMatrixGet(L_6x10, i, 2) * betas[1] * betas[1] + cnMatrixGet(L_6x10, i, 3) * betas[0] * betas[2] +
				 cnMatrixGet(L_6x10, i, 4) * betas[1] * betas[2] + cnMatrixGet(L_6x10, i, 5) * betas[2] * betas[2] +
				 cnMatrixGet(L_6x10, i, 6) * betas[0] * betas[3] + cnMatrixGet(L_6x10, i, 7) * betas[1] * betas[3] +
				 cnMatrixGet(L_6x10, i, 8) * betas[2] * betas[3] + cnMatrixGet(L_6x10, i, 9) * betas[3] * betas[3]));
	}
}

static void gauss_newton(const CnMat *L_6x10, const CnMat *Rho, FLT betas[4]) {
	const int iterations_number = 5;

	FLT x[4] = {0};
	CN_CREATE_STACK_MAT(A, 6, 4);
	CN_CREATE_STACK_MAT(B, 6, 1);
	CnMat X = cnMat(4, 1, x);

	for (int k = 0; k < iterations_number; k++) {
		compute_A_and_b_gauss_newton(L_6x10, cn_as_const_vector(Rho), betas, &A, &B);
		cnSolve(&A, &B, &X, CN_INVERT_METHOD_QR);
		for (int i = 0; i < 4; i++)
			betas[i] += x[i];
	}
	CN_FREE_STACK_MAT(B);
	CN_FREE_STACK_MAT(A);
}

static void find_betas_approx_2(const CnMat *L_6x10, const CnMat *Rho, FLT *betas) {
	FLT b3[3];
	CN_CREATE_STACK_MAT(L_6x3, 6, 3);
	CnMat B3 = cnMat(3, 1, b3);

	for (int i = 0; i < 6; i++) {
		cnMatrixSet(&L_6x3, i, 0, cnMatrixGet(L_6x10, i, 0));
		cnMatrixSet(&L_6x3, i, 1, cnMatrixGet(L_6x10, i, 1));
		cnMatrixSet(&L_6x3, i, 2, cnMatrixGet(L_6x10, i, 2));
	}

	cnSolve(&L_6x3, Rho, &B3, CN_INVERT_METHOD_SVD);

	if (b3[0] < 0) {
		betas[0] = sqrt(-b3[0]);
		betas[1] = (b3[2] < 0) ? sqrt(-b3[2]) : 0.0;
	} else {
		betas[0] = sqrt(b3[0]);
		betas[1] = (b3[2] > 0) ? sqrt(b3[2]) : 0.0;
	}

	if (b3[1] < 0)
		betas[0] = -betas[0];

	betas[2] = 0.0;
	betas[3] = 0.0;
	CN_FREE_STACK_MAT(L_6x3);
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_3 = [B11 B12 B22 B13 B23                    ]

static void bc_svd_find_betas_approx_3(bc_svd *self, const CnMat *L_6x10, const CnMat *Rho, FLT *betas) {
	FLT l_6x5[6 * 5], b5[5];
	CnMat L_6x5 = cnMat(6, 5, l_6x5);
	CnMat B5 = cnMat(5, 1, b5);

	for (int i = 0; i < 6; i++) {
		cnMatrixSet(&L_6x5, i, 0, cnMatrixGet(L_6x10, i, 0));
		cnMatrixSet(&L_6x5, i, 1, cnMatrixGet(L_6x10, i, 1));
		cnMatrixSet(&L_6x5, i, 2, cnMatrixGet(L_6x10, i, 2));
		cnMatrixSet(&L_6x5, i, 3, cnMatrixGet(L_6x10, i, 3));
		cnMatrixSet(&L_6x5, i, 4, cnMatrixGet(L_6x10, i, 4));
	}

	cnSolve(&L_6x5, Rho, &B5, CN_INVERT_METHOD_SVD);

	if (b5[0] < 0) {
		betas[0] = sqrt(-b5[0]);
		betas[1] = (b5[2] < 0) ? sqrt(-b5[2]) : 0.0;
	} else {
		betas[0] = sqrt(b5[0]);
		betas[1] = (b5[2] > 0) ? sqrt(b5[2]) : 0.0;
	}
	if (b5[1] < 0)
		betas[0] = -betas[0];
	betas[2] = b5[3] / betas[0];
	betas[3] = 0.0;
}

static void copy_R_and_t(const FLT R_src[3][3], const FLT t_src[3], FLT R_dst[3][3], FLT t_dst[3]) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			R_dst[i][j] = R_src[i][j];
		t_dst[i] = t_src[i];
	}
}

FLT bc_svd_compute_pose(bc_svd *self, FLT R[3][3], FLT t[3]) {
	FLT Betas[4][4] = {0}, rep_errors[3] = {0};
	FLT Rs[4][3][3] = {0}, ts[4][3] = {0};
	int N = 0;

	CN_CREATE_STACK_MAT(M, self->meas_cnt, 12);
	bool colCovered[12] = { 0 };
	bool has_axis[2] = {false, false};
	for (int i = 0; i < self->meas_cnt; i++) {
		size_t obj_pt_idx = self->meas[i].obj_idx;
		const bc_svd_meas_t *meas = &self->meas[i];
		bc_svd_fill_M(self, &M, i, self->setup.alphas[obj_pt_idx], meas->axis, meas->angle);
		has_axis[meas->axis] = true;

		for (int j = 0; j < 12; j++) {
			FLT v = cnMatrixGet(&M, i, j);
			assert(isfinite(v));
			if (v != 0.0)
				colCovered[j] = true;
		}
	}

	// Gen2 can technically solve with just one axis but it's very very very noisey
	if (has_axis[0] == false || has_axis[1] == false) {
		return -1;
	}

	for (int j = 0; j < 12; j++) {
		if (colCovered[j] == false)
			return -1;
	}

	CN_CREATE_STACK_MAT(MtM, 12, 12);
	CN_CREATE_STACK_MAT(D, 12, 1);
	CN_CREATE_STACK_MAT(Ut, 12, 12);

	cnMulTransposed(&M, &MtM, 1, 0, 1);

	cnSVD(&MtM, &D, &Ut, 0, CN_SVD_MODIFY_A | CN_SVD_U_T);

	FLT l_6x10[6 * 10], rho[6];
	CnMat L_6x10 = cnMat(6, 10, l_6x10);
	CnMat Rho = cnMat(6, 1, rho);

	bc_svd_compute_L_6x10(self, &Ut, &L_6x10);

	bc_svd_compute_rho(self, rho);

	find_betas_approx_1(&L_6x10, &Rho, Betas[1]);
	gauss_newton(&L_6x10, &Rho, Betas[1]);
	rep_errors[0] = bc_svd_compute_R_and_t(self, &Ut, Betas[1], Rs[1], ts[1]);

	find_betas_approx_2(&L_6x10, &Rho, Betas[2]);
	gauss_newton(&L_6x10, &Rho, Betas[2]);
	rep_errors[1] = bc_svd_compute_R_and_t(self, &Ut, Betas[2], Rs[2], ts[2]);

	bc_svd_find_betas_approx_3(self, &L_6x10, &Rho, Betas[3]);
	gauss_newton(&L_6x10, &Rho, Betas[3]);
	rep_errors[2] = bc_svd_compute_R_and_t(self, &Ut, Betas[3], Rs[3], ts[3]);

	N = 0;
	if (rep_errors[1] < rep_errors[0])
		N = 1;
	if (rep_errors[2] < rep_errors[N])
		N = 2;

	// printf("BCSVD %f %f %f (%d)\n", rep_errors[0], rep_errors[1], rep_errors[2], N);

	copy_R_and_t(Rs[N + 1], ts[N + 1], R, t);

	CN_FREE_STACK_MAT(Ut);
	CN_FREE_STACK_MAT(D);
	CN_FREE_STACK_MAT(MtM);
	CN_FREE_STACK_MAT(M);

	return rep_errors[N];
}

static FLT bc_svd_reprojection_error(bc_svd *self, const FLT R[3][3], const FLT t[3]) {
	FLT sum2 = 0.0;

	for (int i = 0; i < self->meas_cnt; i++) {
		size_t obj_idx = self->meas[i].obj_idx;
		assert(obj_idx < self->setup.obj_cnt);
		const FLT *pw = self->setup.obj_pts[obj_idx];
		FLT Xc = dot(R[0], pw) + t[0];
		FLT Yc = dot(R[1], pw) + t[1];
		FLT Zc = dot(R[2], pw) + t[2];

		FLT eq[3];

		self->setup.fillFn(self->setup.user, eq, self->meas[i].axis, self->meas[i].angle);
		FLT rerr = eq[0] * Xc + eq[1] * Yc + eq[2] * Zc;
		sum2 += rerr * rerr;
	}

	return sqrt(sum2) / self->meas_cnt;
}

void bc_svd_estimate_R_and_t(bc_svd *self, FLT R[3][3], FLT t[3]) {
	FLT pc0[3], pw0[3];

	pc0[0] = pc0[1] = pc0[2] = 0.0;
	pw0[0] = pw0[1] = pw0[2] = 0.0;

	for (int i = 0; i < self->setup.obj_cnt; i++) {
		const FLT *pc = self->object_pts_in_camera[i];
		const FLT *pw = self->setup.obj_pts[i];

		for (int j = 0; j < 3; j++) {
			pc0[j] += pc[j];
			pw0[j] += pw[j];
		}
	}
	for (int j = 0; j < 3; j++) {
		pc0[j] /= self->setup.obj_cnt;
		pw0[j] /= self->setup.obj_cnt;
	}

	FLT abt[3 * 3] = {0}, abt_d[3] = {0}, abt_u[3 * 3] = {0}, abt_v[3 * 3] = {0};
	CnMat ABt = cnMat(3, 3, abt);
	CnMat ABt_D = cnMat(3, 1, abt_d);
	CnMat ABt_U = cnMat(3, 3, abt_u);
	CnMat ABt_V = cnMat(3, 3, abt_v);

	cnSetZero(&ABt);

	for (int i = 0; i < self->setup.obj_cnt; i++) {
		FLT *pc = self->object_pts_in_camera[i];
		const FLT *pw = self->setup.obj_pts[i];

		for (int j = 0; j < 3; j++) {
			abt[3 * j] += (pc[j] - pc0[j]) * (pw[0] - pw0[0]);
			abt[3 * j + 1] += (pc[j] - pc0[j]) * (pw[1] - pw0[1]);
			abt[3 * j + 2] += (pc[j] - pc0[j]) * (pw[2] - pw0[2]);
		}
	}

#ifdef CN_MATRIX_IS_COL_MAJOR
	cnTranspose(&ABt, &ABt);
#endif

	cnSVD(&ABt, &ABt_D, &ABt_U, &ABt_V, CN_SVD_MODIFY_A);

#ifdef CN_MATRIX_IS_COL_MAJOR
	cnTranspose(&ABt, &ABt);
	cnTranspose(&ABt_U, &ABt_U);
	cnTranspose(&ABt_V, &ABt_V);
#endif

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R[i][j] = dot(abt_u + 3 * i, abt_v + 3 * j);

	const FLT det = R[0][0] * R[1][1] * R[2][2] + R[0][1] * R[1][2] * R[2][0] + R[0][2] * R[1][0] * R[2][1] -
					R[0][2] * R[1][1] * R[2][0] - R[0][1] * R[1][0] * R[2][2] - R[0][0] * R[1][2] * R[2][1];

	if (det < 0) {
		R[2][0] = -R[2][0];
		R[2][1] = -R[2][1];
		R[2][2] = -R[2][2];
	}

	t[0] = pc0[0] - dot(R[0], pw0);
	t[1] = pc0[1] - dot(R[1], pw0);
	t[2] = pc0[2] - dot(R[2], pw0);
}

void bc_svd_solve_for_sign(bc_svd *self) {
	if (self->object_pts_in_camera[0][2] < 0.0) {
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 3; j++)
				self->control_points_in_camera[i][j] = -self->control_points_in_camera[i][j];

		for (int i = 0; i < self->setup.obj_cnt; i++) {
			self->object_pts_in_camera[i][0] = -self->object_pts_in_camera[i][0];
			self->object_pts_in_camera[i][1] = -self->object_pts_in_camera[i][1];
			self->object_pts_in_camera[i][2] = -self->object_pts_in_camera[i][2];
		}
	}
}

static FLT bc_svd_compute_R_and_t(bc_svd *self, const CnMat *ut, const FLT *betas, FLT R[3][3], FLT t[3]) {
	bc_svd_compute_ccs(self, betas, ut);
	bc_svd_compute_pcs(self);

	bc_svd_solve_for_sign(self);

	bc_svd_estimate_R_and_t(self, R, t);

	return bc_svd_reprojection_error(self, R, t);
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_1 = [B11 B12     B13         B14]

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_2 = [B11 B12 B22                            ]

void mat_to_quat(const FLT R[3][3], FLT q[4]) {
	FLT tr = R[0][0] + R[1][1] + R[2][2];
	FLT n4;

	if (tr > 0.0f) {
		q[0] = R[1][2] - R[2][1];
		q[1] = R[2][0] - R[0][2];
		q[2] = R[0][1] - R[1][0];
		q[3] = tr + 1.0f;
		n4 = q[3];
	} else if ((R[0][0] > R[1][1]) && (R[0][0] > R[2][2])) {
		q[0] = 1.0f + R[0][0] - R[1][1] - R[2][2];
		q[1] = R[1][0] + R[0][1];
		q[2] = R[2][0] + R[0][2];
		q[3] = R[1][2] - R[2][1];
		n4 = q[0];
	} else if (R[1][1] > R[2][2]) {
		q[0] = R[1][0] + R[0][1];
		q[1] = 1.0f + R[1][1] - R[0][0] - R[2][2];
		q[2] = R[2][1] + R[1][2];
		q[3] = R[2][0] - R[0][2];
		n4 = q[1];
	} else {
		q[0] = R[2][0] + R[0][2];
		q[1] = R[2][1] + R[1][2];
		q[2] = 1.0f + R[2][2] - R[0][0] - R[1][1];
		q[3] = R[0][1] - R[1][0];
		n4 = q[2];
	}
	FLT scale = 0.5f / (sqrt(n4));

	q[0] *= scale;
	q[1] *= scale;
	q[2] *= scale;
	q[3] *= scale;
}

void relative_error(FLT *rot_err, FLT *transl_err, const FLT Rtrue[3][3], const FLT ttrue[3], const FLT Rest[3][3],
					const FLT test[3]) {
	FLT qtrue[4], qest[4];

	mat_to_quat(Rtrue, qtrue);
	mat_to_quat(Rest, qest);

	FLT rot_err1 = sqrt((qtrue[0] - qest[0]) * (qtrue[0] - qest[0]) + (qtrue[1] - qest[1]) * (qtrue[1] - qest[1]) +
						(qtrue[2] - qest[2]) * (qtrue[2] - qest[2]) + (qtrue[3] - qest[3]) * (qtrue[3] - qest[3])) /
				   sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

	FLT rot_err2 = sqrt((qtrue[0] + qest[0]) * (qtrue[0] + qest[0]) + (qtrue[1] + qest[1]) * (qtrue[1] + qest[1]) +
						(qtrue[2] + qest[2]) * (qtrue[2] + qest[2]) + (qtrue[3] + qest[3]) * (qtrue[3] + qest[3])) /
				   sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

	*rot_err = fmin(rot_err1, rot_err2);

	*transl_err = sqrt((ttrue[0] - test[0]) * (ttrue[0] - test[0]) + (ttrue[1] - test[1]) * (ttrue[1] - test[1]) +
					   (ttrue[2] - test[2]) * (ttrue[2] - test[2])) /
				  sqrt(ttrue[0] * ttrue[0] + ttrue[1] * ttrue[1] + ttrue[2] * ttrue[2]);
}
