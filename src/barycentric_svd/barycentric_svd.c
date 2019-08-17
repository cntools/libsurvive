#include "barycentric_svd.h"
#include "math.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"

static void bc_svd_choose_control_points(bc_svd *self) {
	// Take C0 as the reference points centroid:
	self->setup.control_points[0][0] = self->setup.control_points[0][1] = self->setup.control_points[0][2] = 0;
	for (int i = 0; i < self->setup.obj_cnt; i++)
		for (int j = 0; j < 3; j++)
			self->setup.control_points[0][j] += self->setup.obj_pts[i][j];

	for (int j = 0; j < 3; j++)
		self->setup.control_points[0][j] /= self->setup.obj_cnt;

	// Take C1, C2, and C3 from PCA on the reference points:
	CvMat *PW0 = cvCreateMat(self->setup.obj_cnt, 3, CV_64F);

	double pw0tpw0[3 * 3] = {0}, dc[3], uct[3 * 3];
	CvMat PW0tPW0 = cvMat(3, 3, CV_64F, pw0tpw0);
	CvMat DC = cvMat(3, 1, CV_64F, dc);
	CvMat UCt = cvMat(3, 3, CV_64F, uct);

	for (int i = 0; i < self->setup.obj_cnt; i++)
		for (int j = 0; j < 3; j++)
			PW0->data.db[3 * i + j] = self->setup.obj_pts[i][j] - self->setup.control_points[0][j];

	cvMulTransposed(PW0, &PW0tPW0, 1, 0, 1);

	cvSVD(&PW0tPW0, &DC, &UCt, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);
	assert(UCt.data.db == uct);

	cvReleaseMat(&PW0);

	for (int i = 1; i < 4; i++) {
		double k = sqrt(dc[i - 1] / self->setup.obj_cnt);
		for (int j = 0; j < 3; j++)
			self->setup.control_points[i][j] = self->setup.control_points[0][j] + k * uct[3 * (i - 1) + j];
	}
}

static void bc_svd_compute_barycentric_coordinates(bc_svd *self) {
	double cc[3 * 3], cc_inv[3 * 3];
	CvMat CC = cvMat(3, 3, CV_64F, cc);
	CvMat CC_inv = cvMat(3, 3, CV_64F, cc_inv);

	for (int i = 0; i < 3; i++)
		for (int j = 1; j < 4; j++)
			cc[3 * i + j - 1] = self->setup.control_points[j][i] - self->setup.control_points[0][i];

	cvInvert(&CC, &CC_inv, 1);

	double *ci = cc_inv;
	for (int i = 0; i < self->setup.obj_cnt; i++) {
		const double *pi = self->setup.obj_pts[i];
		double *a = self->setup.alphas[i];

		for (int j = 0; j < 3; j++)
			a[1 + j] = ci[3 * j] * (pi[0] - self->setup.control_points[0][0]) +
					   ci[3 * j + 1] * (pi[1] - self->setup.control_points[0][1]) +
					   ci[3 * j + 2] * (pi[2] - self->setup.control_points[0][2]);
		a[0] = 1.0f - a[1] - a[2] - a[3];
	}
}

void bc_svd_bc_svd(bc_svd *self, void *user, bc_svd_fill_M_fn fillFn, const LinmathPoint3d *obj_pts, size_t obj_cnt) {
	*self = (bc_svd){0};

	self->setup.user = user;
	self->setup.fillFn = fillFn;

	self->setup.obj_cnt = obj_cnt;
	self->setup.obj_pts = obj_pts;
	self->setup.alphas = calloc(obj_cnt, sizeof(self->setup.alphas[0]));
	self->object_pts_in_camera = calloc(obj_cnt, sizeof(self->setup.alphas[0]));

	bc_svd_choose_control_points(self);
	bc_svd_compute_barycentric_coordinates(self);
}

void bc_svd_dtor(bc_svd *self) {
	free(self->setup.alphas);
	free(self->object_pts_in_camera);
	free(self->meas);
}

double bc_svd_compute_R_and_t(bc_svd *self, const double *ut, const double *betas, double R[3][3], double t[3]);

double dot(const double *v1, const double *v2) { return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]; }

double dist2(const double *p1, const double *p2) {
	return (p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]) + (p1[2] - p2[2]) * (p1[2] - p2[2]);
}

void bc_svd_compute_rho(bc_svd *self, double *rho) {
	rho[0] = dist2(self->setup.control_points[0], self->setup.control_points[1]);
	rho[1] = dist2(self->setup.control_points[0], self->setup.control_points[2]);
	rho[2] = dist2(self->setup.control_points[0], self->setup.control_points[3]);
	rho[3] = dist2(self->setup.control_points[1], self->setup.control_points[2]);
	rho[4] = dist2(self->setup.control_points[1], self->setup.control_points[3]);
	rho[5] = dist2(self->setup.control_points[2], self->setup.control_points[3]);
}

void bc_svd_reset_correspondences(bc_svd *self) { self->meas_cnt = 0; }

void bc_svd_add_correspondence(bc_svd *self, size_t idx, double u, double v) {
	if (isnan(u) && isnan(v)) {
		return;
	}

	bool only_pairs = false;
	if (only_pairs && (isnan(u) || isnan(v))) {
		return;
	}

	for (int i = 0; i < 2; i++) {
		FLT angle = i == 0 ? u : v;
		if (isnan(angle))
			continue;

		if (self->meas_space <= self->meas_cnt) {
			self->meas_space = self->meas_space * 2 + 1;
			self->meas = realloc(self->meas, sizeof(self->meas[0]) * self->meas_space);
		}

		self->meas[self->meas_cnt] = (bc_svd_meas_t){.angle = angle, .axis = i, .obj_idx = idx};

		self->meas_cnt++;
	}
}

void bc_svd_fill_M(bc_svd *self, CvMat *_M, const int row, const double *as, int axis, double angle) {
	double *M = _M->data.db + row * 12;

	double eq[3] = {NAN, NAN, NAN};
	self->setup.fillFn(self->setup.user, eq, axis, angle);

	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 3; j++) {
			M[i * 3 + j] = eq[j] * as[i];
		}
	}
}

void bc_svd_compute_ccs(bc_svd *self, const double *betas, const double *ut) {
	for (int i = 0; i < 4; i++)
		self->control_points_in_camera[i][0] = self->control_points_in_camera[i][1] =
			self->control_points_in_camera[i][2] = 0.0f;

	for (int i = 0; i < 4; i++) {
		const double *v = ut + 12 * (11 - i);
		for (int j = 0; j < 4; j++)
			for (int k = 0; k < 3; k++)
				self->control_points_in_camera[j][k] += betas[i] * v[3 * j + k];
	}
}

void bc_svd_compute_pcs(bc_svd *self) {
	for (int i = 0; i < self->setup.obj_cnt; i++) {
		double *a = self->setup.alphas[i];
		double *pc = self->object_pts_in_camera[i];

		for (int j = 0; j < 3; j++)
			pc[j] = a[0] * self->control_points_in_camera[0][j] + a[1] * self->control_points_in_camera[1][j] +
					a[2] * self->control_points_in_camera[2][j] + a[3] * self->control_points_in_camera[3][j];
	}
}

void bc_svd_compute_L_6x10(bc_svd *self, const double *ut, double *l_6x10) {
	const double *v[4];

	v[0] = ut + 12 * 11;
	v[1] = ut + 12 * 10;
	v[2] = ut + 12 * 9;
	v[3] = ut + 12 * 8;

	double dv[4][6][3];

	for (int i = 0; i < 4; i++) {
		int a = 0, b = 1;
		for (int j = 0; j < 6; j++) {
			dv[i][j][0] = v[i][3 * a] - v[i][3 * b];
			dv[i][j][1] = v[i][3 * a + 1] - v[i][3 * b + 1];
			dv[i][j][2] = v[i][3 * a + 2] - v[i][3 * b + 2];

			b++;
			if (b > 3) {
				a++;
				b = a + 1;
			}
		}
	}

	for (int i = 0; i < 6; i++) {
		double *row = l_6x10 + 10 * i;

		row[0] = dot(dv[0][i], dv[0][i]);
		row[1] = 2.0f * dot(dv[0][i], dv[1][i]);
		row[2] = dot(dv[1][i], dv[1][i]);
		row[3] = 2.0f * dot(dv[0][i], dv[2][i]);
		row[4] = 2.0f * dot(dv[1][i], dv[2][i]);
		row[5] = dot(dv[2][i], dv[2][i]);
		row[6] = 2.0f * dot(dv[0][i], dv[3][i]);
		row[7] = 2.0f * dot(dv[1][i], dv[3][i]);
		row[8] = 2.0f * dot(dv[2][i], dv[3][i]);
		row[9] = dot(dv[3][i], dv[3][i]);
	}
}

void find_betas_approx_1(const CvMat *L_6x10, const CvMat *Rho, double *betas) {
	double l_6x4[6 * 4], b4[4];
	CvMat L_6x4 = cvMat(6, 4, CV_64F, l_6x4);
	CvMat B4 = cvMat(4, 1, CV_64F, b4);

	for (int i = 0; i < 6; i++) {
		cvmSet(&L_6x4, i, 0, cvmGet(L_6x10, i, 0));
		cvmSet(&L_6x4, i, 1, cvmGet(L_6x10, i, 1));
		cvmSet(&L_6x4, i, 2, cvmGet(L_6x10, i, 3));
		cvmSet(&L_6x4, i, 3, cvmGet(L_6x10, i, 6));
	}

	cvSolve(&L_6x4, Rho, &B4, CV_SVD);

	assert(B4.data.db == b4);

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

void compute_A_and_b_gauss_newton(const double *l_6x10, const double *rho, double betas[4], CvMat *A, CvMat *b) {
	for (int i = 0; i < 6; i++) {
		const double *rowL = l_6x10 + i * 10;
		double *rowA = A->data.db + i * 4;

		rowA[0] = 2 * rowL[0] * betas[0] + rowL[1] * betas[1] + rowL[3] * betas[2] + rowL[6] * betas[3];
		rowA[1] = rowL[1] * betas[0] + 2 * rowL[2] * betas[1] + rowL[4] * betas[2] + rowL[7] * betas[3];
		rowA[2] = rowL[3] * betas[0] + rowL[4] * betas[1] + 2 * rowL[5] * betas[2] + rowL[8] * betas[3];
		rowA[3] = rowL[6] * betas[0] + rowL[7] * betas[1] + rowL[8] * betas[2] + 2 * rowL[9] * betas[3];

		cvmSet(b, i, 0,
			   rho[i] - (rowL[0] * betas[0] * betas[0] + rowL[1] * betas[0] * betas[1] + rowL[2] * betas[1] * betas[1] +
						 rowL[3] * betas[0] * betas[2] + rowL[4] * betas[1] * betas[2] + rowL[5] * betas[2] * betas[2] +
						 rowL[6] * betas[0] * betas[3] + rowL[7] * betas[1] * betas[3] + rowL[8] * betas[2] * betas[3] +
						 rowL[9] * betas[3] * betas[3]));
	}
}

void qr_solve(CvMat *A, CvMat *b, CvMat *X) {
	static int max_nr = 0;
	static double *A1, *A2;

	const int nr = A->rows;
	const int nc = A->cols;

	if (max_nr != 0 && max_nr < nr) {
		free(A1);
		free(A2);
	}
	if (max_nr < nr) {
		max_nr = nr;
		A1 = malloc(sizeof(double) * nr);
		A2 = malloc(sizeof(double) * nr);
	}

	double *pA = A->data.db, *ppAkk = pA;
	for (int k = 0; k < nc; k++) {
		double *ppAik = ppAkk, eta = fabs(*ppAik);
		for (int i = k + 1; i < nr; i++) {
			double elt = fabs(*ppAik);
			if (eta < elt)
				eta = elt;
			ppAik += nc;
		}

		if (eta == 0) {
			A1[k] = A2[k] = 0.0;
			// cerr << "God damnit, A is singular, this shouldn't happen." << endl;
			return;
		} else {
			double *ppAik = ppAkk, sum = 0.0, inv_eta = 1. / eta;
			for (int i = k; i < nr; i++) {
				*ppAik *= inv_eta;
				sum += *ppAik * *ppAik;
				ppAik += nc;
			}
			double sigma = sqrt(sum);
			if (*ppAkk < 0)
				sigma = -sigma;
			*ppAkk += sigma;
			A1[k] = sigma * *ppAkk;
			A2[k] = -eta * sigma;
			for (int j = k + 1; j < nc; j++) {
				double *ppAik = ppAkk, sum = 0;
				for (int i = k; i < nr; i++) {
					sum += *ppAik * ppAik[j - k];
					ppAik += nc;
				}
				double tau = sum / A1[k];
				ppAik = ppAkk;
				for (int i = k; i < nr; i++) {
					ppAik[j - k] -= tau * *ppAik;
					ppAik += nc;
				}
			}
		}
		ppAkk += nc + 1;
	}

	// b <- Qt b
	double *ppAjj = pA, *pb = b->data.db;
	for (int j = 0; j < nc; j++) {
		double *ppAij = ppAjj, tau = 0;
		for (int i = j; i < nr; i++) {
			tau += *ppAij * pb[i];
			ppAij += nc;
		}
		tau /= A1[j];
		ppAij = ppAjj;
		for (int i = j; i < nr; i++) {
			pb[i] -= tau * *ppAij;
			ppAij += nc;
		}
		ppAjj += nc + 1;
	}

	// X = R-1 b
	double *pX = X->data.db;
	pX[nc - 1] = pb[nc - 1] / A2[nc - 1];
	for (int i = nc - 2; i >= 0; i--) {
		double *ppAij = pA + i * nc + (i + 1), sum = 0;

		for (int j = i + 1; j < nc; j++) {
			sum += *ppAij * pX[j];
			ppAij++;
		}
		pX[i] = (pb[i] - sum) / A2[i];
	}
}

void gauss_newton(const CvMat *L_6x10, const CvMat *Rho, double betas[4]) {
	const int iterations_number = 5;

	double a[6 * 4], b[6], x[4];
	CvMat A = cvMat(6, 4, CV_64F, a);
	CvMat B = cvMat(6, 1, CV_64F, b);
	CvMat X = cvMat(4, 1, CV_64F, x);

	for (int k = 0; k < iterations_number; k++) {
		compute_A_and_b_gauss_newton(L_6x10->data.db, Rho->data.db, betas, &A, &B);
		qr_solve(&A, &B, &X);

		for (int i = 0; i < 4; i++)
			betas[i] += x[i];
	}
}

void find_betas_approx_2(const CvMat *L_6x10, const CvMat *Rho, double *betas) {
	double l_6x3[6 * 3], b3[3];
	CvMat L_6x3 = cvMat(6, 3, CV_64F, l_6x3);
	CvMat B3 = cvMat(3, 1, CV_64F, b3);

	for (int i = 0; i < 6; i++) {
		cvmSet(&L_6x3, i, 0, cvmGet(L_6x10, i, 0));
		cvmSet(&L_6x3, i, 1, cvmGet(L_6x10, i, 1));
		cvmSet(&L_6x3, i, 2, cvmGet(L_6x10, i, 2));
	}

	cvSolve(&L_6x3, Rho, &B3, CV_SVD);

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
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_3 = [B11 B12 B22 B13 B23                    ]

void bc_svd_find_betas_approx_3(bc_svd *self, const CvMat *L_6x10, const CvMat *Rho, double *betas) {
	double l_6x5[6 * 5], b5[5];
	CvMat L_6x5 = cvMat(6, 5, CV_64F, l_6x5);
	CvMat B5 = cvMat(5, 1, CV_64F, b5);

	for (int i = 0; i < 6; i++) {
		cvmSet(&L_6x5, i, 0, cvmGet(L_6x10, i, 0));
		cvmSet(&L_6x5, i, 1, cvmGet(L_6x10, i, 1));
		cvmSet(&L_6x5, i, 2, cvmGet(L_6x10, i, 2));
		cvmSet(&L_6x5, i, 3, cvmGet(L_6x10, i, 3));
		cvmSet(&L_6x5, i, 4, cvmGet(L_6x10, i, 4));
	}

	cvSolve(&L_6x5, Rho, &B5, CV_SVD);

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

void copy_R_and_t(const double R_src[3][3], const double t_src[3], double R_dst[3][3], double t_dst[3]) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++)
			R_dst[i][j] = R_src[i][j];
		t_dst[i] = t_src[i];
	}
}

#ifdef USE_DOUBLE
#define SURVIVE_CV_F CV_64F
#else
#define SURVIVE_CV_F CV_32F
#endif

#define CREATE_STACK_MAT(name, rows, cols)                                                                             \
	FLT *_##name = alloca(rows * cols * sizeof(FLT));                                                                  \
	CvMat name = cvMat(rows, cols, SURVIVE_CV_F, _##name);

double bc_svd_compute_pose(bc_svd *self, double R[3][3], double t[3]) {
	CREATE_STACK_MAT(M, self->meas_cnt, 12);
	bool colCovered[12] = { 0 };
	for (int i = 0; i < self->meas_cnt; i++) {
		size_t obj_pt_idx = self->meas[i].obj_idx;
		bc_svd_fill_M(self, &M, i, self->setup.alphas[obj_pt_idx], self->meas[i].axis, self->meas[i].angle);

		double *_M = M.data.db + i * 12;
		for (int j = 0; j < 12; j++) {
			if (_M[j] != 0.0)
				colCovered[j] = true;
		}
	}

	for (int j = 0; j < 12; j++) {
		if (colCovered[j] == false)
			return -1;
	}

	double mtm[12 * 12], d[12], ut[12 * 12];
	CvMat MtM = cvMat(12, 12, CV_64F, mtm);
	CvMat D = cvMat(12, 1, CV_64F, d);
	CvMat Ut = cvMat(12, 12, CV_64F, ut);

	cvMulTransposed(&M, &MtM, 1, 0, 1);

	cvSVD(&MtM, &D, &Ut, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);

	assert(Ut.data.db == ut);

	double l_6x10[6 * 10], rho[6];
	CvMat L_6x10 = cvMat(6, 10, CV_64F, l_6x10);
	CvMat Rho = cvMat(6, 1, CV_64F, rho);

	bc_svd_compute_L_6x10(self, ut, l_6x10);

	bc_svd_compute_rho(self, rho);

	double Betas[4][4] = { 0 }, rep_errors[4] = { 0 };
	double Rs[4][3][3] = { 0 }, ts[4][3] = { 0 };

	find_betas_approx_1(&L_6x10, &Rho, Betas[1]);
	gauss_newton(&L_6x10, &Rho, Betas[1]);

	rep_errors[1] = bc_svd_compute_R_and_t(self, ut, Betas[1], Rs[1], ts[1]);

	find_betas_approx_2(&L_6x10, &Rho, Betas[2]);
	gauss_newton(&L_6x10, &Rho, Betas[2]);
	rep_errors[2] = bc_svd_compute_R_and_t(self, ut, Betas[2], Rs[2], ts[2]);

	bc_svd_find_betas_approx_3(self, &L_6x10, &Rho, Betas[3]);
	gauss_newton(&L_6x10, &Rho, Betas[3]);
	rep_errors[3] = bc_svd_compute_R_and_t(self, ut, Betas[3], Rs[3], ts[3]);

	int N = 1;
	if (rep_errors[2] < rep_errors[1])
		N = 2;
	if (rep_errors[3] < rep_errors[N])
		N = 3;

	copy_R_and_t(Rs[N], ts[N], R, t);

	return rep_errors[N];
}

static double bc_svd_reprojection_error(bc_svd *self, const double R[3][3], const double t[3]) {
	double sum2 = 0.0;

	for (int i = 0; i < self->meas_cnt; i++) {
		size_t obj_idx = self->meas[i].obj_idx;
		const double *pw = self->setup.obj_pts[obj_idx];
		double Xc = dot(R[0], pw) + t[0];
		double Yc = dot(R[1], pw) + t[1];
		double Zc = dot(R[2], pw) + t[2];

		double eq[3];

		self->setup.fillFn(self->setup.user, eq, self->meas[i].axis, self->meas[i].angle);
		double rerr = eq[0] * Xc + eq[1] * Yc + eq[2] * Zc;
		sum2 += rerr * rerr;
	}

	return sqrt(sum2) / self->meas_cnt;
}

void bc_svd_estimate_R_and_t(bc_svd *self, double R[3][3], double t[3]) {
	double pc0[3], pw0[3];

	pc0[0] = pc0[1] = pc0[2] = 0.0;
	pw0[0] = pw0[1] = pw0[2] = 0.0;

	for (int i = 0; i < self->setup.obj_cnt; i++) {
		const double *pc = self->object_pts_in_camera[i];
		const double *pw = self->setup.obj_pts[i];

		for (int j = 0; j < 3; j++) {
			pc0[j] += pc[j];
			pw0[j] += pw[j];
		}
	}
	for (int j = 0; j < 3; j++) {
		pc0[j] /= self->setup.obj_cnt;
		pw0[j] /= self->setup.obj_cnt;
	}

	double abt[3 * 3], abt_d[3], abt_u[3 * 3], abt_v[3 * 3];
	CvMat ABt = cvMat(3, 3, CV_64F, abt);
	CvMat ABt_D = cvMat(3, 1, CV_64F, abt_d);
	CvMat ABt_U = cvMat(3, 3, CV_64F, abt_u);
	CvMat ABt_V = cvMat(3, 3, CV_64F, abt_v);

	cvSetZero(&ABt);

	for (int i = 0; i < self->setup.obj_cnt; i++) {
		double *pc = self->object_pts_in_camera[i];
		const double *pw = self->setup.obj_pts[i];

		for (int j = 0; j < 3; j++) {
			abt[3 * j] += (pc[j] - pc0[j]) * (pw[0] - pw0[0]);
			abt[3 * j + 1] += (pc[j] - pc0[j]) * (pw[1] - pw0[1]);
			abt[3 * j + 2] += (pc[j] - pc0[j]) * (pw[2] - pw0[2]);
		}
	}

	cvSVD(&ABt, &ABt_D, &ABt_U, &ABt_V, CV_SVD_MODIFY_A);

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			R[i][j] = dot(abt_u + 3 * i, abt_v + 3 * j);

	const double det = R[0][0] * R[1][1] * R[2][2] + R[0][1] * R[1][2] * R[2][0] + R[0][2] * R[1][0] * R[2][1] -
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

void print_pose(const double R[3][3], const double t[3]) {
	for (unsigned i = 0; i < 3; i++) {
		for (unsigned j = 0; j < 3; j++) {
			printf("%g ", R[i][j]);
		}
		printf("%g ", t[i]);
		printf("\n");
	}
	printf("\n");
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

double bc_svd_compute_R_and_t(bc_svd *self, const double *ut, const double *betas, double R[3][3], double t[3]) {
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

void mat_to_quat(const double R[3][3], double q[4]) {
	double tr = R[0][0] + R[1][1] + R[2][2];
	double n4;

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
	double scale = 0.5f / (sqrt(n4));

	q[0] *= scale;
	q[1] *= scale;
	q[2] *= scale;
	q[3] *= scale;
}

void relative_error(double *rot_err, double *transl_err, const double Rtrue[3][3], const double ttrue[3],
					const double Rest[3][3], const double test[3]) {
	double qtrue[4], qest[4];

	mat_to_quat(Rtrue, qtrue);
	mat_to_quat(Rest, qest);

	double rot_err1 = sqrt((qtrue[0] - qest[0]) * (qtrue[0] - qest[0]) + (qtrue[1] - qest[1]) * (qtrue[1] - qest[1]) +
						   (qtrue[2] - qest[2]) * (qtrue[2] - qest[2]) + (qtrue[3] - qest[3]) * (qtrue[3] - qest[3])) /
					  sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

	double rot_err2 = sqrt((qtrue[0] + qest[0]) * (qtrue[0] + qest[0]) + (qtrue[1] + qest[1]) * (qtrue[1] + qest[1]) +
						   (qtrue[2] + qest[2]) * (qtrue[2] + qest[2]) + (qtrue[3] + qest[3]) * (qtrue[3] + qest[3])) /
					  sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

	*rot_err = fmin(rot_err1, rot_err2);

	*transl_err = sqrt((ttrue[0] - test[0]) * (ttrue[0] - test[0]) + (ttrue[1] - test[1]) * (ttrue[1] - test[1]) +
					   (ttrue[2] - test[2]) * (ttrue[2] - test[2])) /
				  sqrt(ttrue[0] * ttrue[0] + ttrue[1] * ttrue[1] + ttrue[2] * ttrue[2]);
}
