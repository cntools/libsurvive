// Copyright (c) 2009, V. Lepetit, EPFL
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// The views and conclusions contained in the software and documentation are those
// of the authors and should not be interpreted as representing official policies,
//   either expressed or implied, of the FreeBSD Project.
#include "epnp.h"
#include "math.h"
#include "stdbool.h"
#include "stdio.h"
#include "stdlib.h"
#include <survive.h>

#pragma GCC diagnostic ignored "-Wpedantic"

void print_mat(const CvMat *M) {
	if (!M) {
		printf("null\n");
		return;
	}
	printf("%d x %d:\n", M->rows, M->cols);
	for (unsigned i = 0; i < M->rows; i++) {
		for (unsigned j = 0; j < M->cols; j++) {
			printf("%.17g, ", cvmGet(M, i, j));
		}
		printf("\n");
	}
	printf("\n");
}

void epnp_epnp(epnp *self) {
	self->maximum_number_of_correspondences = 0;
	self->number_of_correspondences = 0;

	self->obj_pts = 0;
	self->meas = 0;
	self->alphas = 0;
	self->object_pts_in_camera = 0;
}

void epnp_dtor(epnp *self) {
	free(self->obj_pts);
	free(self->meas);
	free(self->alphas);
	free(self->object_pts_in_camera);
}
double epnp_compute_R_and_t(epnp *self, const double *ut, const double *betas, double R[3][3], double t[3]);

double dot(const double *v1, const double *v2) { return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]; }

double dist2(const double *p1, const double *p2) {
	return (p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]) + (p1[2] - p2[2]) * (p1[2] - p2[2]);
}

void epnp_compute_rho(epnp *self, double *rho) {
	rho[0] = dist2(self->control_points[0], self->control_points[1]);
	rho[1] = dist2(self->control_points[0], self->control_points[2]);
	rho[2] = dist2(self->control_points[0], self->control_points[3]);
	rho[3] = dist2(self->control_points[1], self->control_points[2]);
	rho[4] = dist2(self->control_points[1], self->control_points[3]);
	rho[5] = dist2(self->control_points[2], self->control_points[3]);

	CvMat cws = cvMat(4, 3, CV_64F, self->control_points);
	CvMat ccs = cvMat(4, 3, CV_64F, self->control_points_in_camera);
	CvMat pws = cvMat(self->maximum_number_of_correspondences, 3, CV_64F, self->obj_pts);
}

void epnp_set_internal_parameters(epnp *self, double uc, double vc, double fu, double fv) {
	self->uc = uc;
	self->vc = vc;
	self->fu = fu;
	self->fv = fv;
}

void epnp_set_maximum_number_of_correspondences(epnp *self, int n) {
	if (self->maximum_number_of_correspondences < n) {
		if (self->obj_pts != 0)
			free(self->obj_pts);
		if (self->meas != 0)
			free(self->meas);
		if (self->alphas != 0)
			free(self->alphas);
		if (self->object_pts_in_camera != 0)
			free(self->object_pts_in_camera);

		self->maximum_number_of_correspondences = n;
		self->obj_pts = SV_CALLOC(sizeof(double), 3 * self->maximum_number_of_correspondences);
		self->meas = SV_CALLOC(sizeof(double), 2 * self->maximum_number_of_correspondences);
		self->alphas = SV_CALLOC(sizeof(double), 4 * self->maximum_number_of_correspondences);
		self->object_pts_in_camera = SV_CALLOC(sizeof(double), 3 * self->maximum_number_of_correspondences);
	}
}

void epnp_reset_correspondences(epnp *self) { self->number_of_correspondences = 0; }

void epnp_add_correspondence(epnp *self, double X, double Y, double Z, double u, double v) {
	self->obj_pts[3 * self->number_of_correspondences] = X;
	self->obj_pts[3 * self->number_of_correspondences + 1] = Y;
	self->obj_pts[3 * self->number_of_correspondences + 2] = Z;

	self->meas[2 * self->number_of_correspondences] = u;
	self->meas[2 * self->number_of_correspondences + 1] = v;

	self->number_of_correspondences++;
}

void epnp_choose_control_points(epnp *self) {
	// Take C0 as the reference points centroid:
	self->control_points[0][0] = self->control_points[0][1] = self->control_points[0][2] = 0;
	for (int i = 0; i < self->number_of_correspondences; i++)
		for (int j = 0; j < 3; j++)
			self->control_points[0][j] += self->obj_pts[3 * i + j];

	for (int j = 0; j < 3; j++)
		self->control_points[0][j] /= self->number_of_correspondences;

	// Take C1, C2, and C3 from PCA on the reference points:
	CvMat *PW0 = cvCreateMat(self->number_of_correspondences, 3, CV_64F);

	double pw0tpw0[3 * 3] = {0}, dc[3], uct[3 * 3];
	CvMat PW0tPW0 = cvMat(3, 3, CV_64F, pw0tpw0);
	CvMat DC = cvMat(3, 1, CV_64F, dc);
	CvMat UCt = cvMat(3, 3, CV_64F, uct);

	for (int i = 0; i < self->number_of_correspondences; i++)
		for (int j = 0; j < 3; j++)
			PW0->data.db[3 * i + j] = self->obj_pts[3 * i + j] - self->control_points[0][j];

	cvMulTransposed(PW0, &PW0tPW0, 1, 0, 1);

	cvSVD(&PW0tPW0, &DC, &UCt, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);
	assert(UCt.data.db == uct);

	cvReleaseMat(&PW0);

	for (int i = 1; i < 4; i++) {
		double k = sqrt(dc[i - 1] / self->number_of_correspondences);
		for (int j = 0; j < 3; j++)
			self->control_points[i][j] = self->control_points[0][j] + k * uct[3 * (i - 1) + j];
	}
}

void epnp_compute_barycentric_coordinates(epnp *self) {
	double cc[3 * 3], cc_inv[3 * 3];
	CvMat CC = cvMat(3, 3, CV_64F, cc);
	CvMat CC_inv = cvMat(3, 3, CV_64F, cc_inv);

	for (int i = 0; i < 3; i++)
		for (int j = 1; j < 4; j++)
			cc[3 * i + j - 1] = self->control_points[j][i] - self->control_points[0][i];

	cvInvert(&CC, &CC_inv, 1);

	double *ci = cc_inv;
	for (int i = 0; i < self->number_of_correspondences; i++) {
		double *pi = self->obj_pts + 3 * i;
		double *a = self->alphas + 4 * i;

		for (int j = 0; j < 3; j++)
			a[1 + j] = ci[3 * j] * (pi[0] - self->control_points[0][0]) +
					   ci[3 * j + 1] * (pi[1] - self->control_points[0][1]) +
					   ci[3 * j + 2] * (pi[2] - self->control_points[0][2]);
		a[0] = 1.0f - a[1] - a[2] - a[3];
	}
}

void epnp_fill_M(epnp *self, CvMat *M, const int row, const double *as, const double u, const double v) {
	double *M1 = M->data.db + row * 12;
	double *M2 = M1 + 12;

	for (int i = 0; i < 4; i++) {
		M1[3 * i] = as[i] * self->fu;
		M1[3 * i + 1] = 0.0;
		M1[3 * i + 2] = as[i] * (self->uc - u);

		M2[3 * i] = 0.0;
		M2[3 * i + 1] = as[i] * self->fv;
		M2[3 * i + 2] = as[i] * (self->vc - v);
	}
}

void epnp_compute_ccs(epnp *self, const double *betas, const double *ut) {
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

void epnp_compute_pcs(epnp *self) {
	for (int i = 0; i < self->number_of_correspondences; i++) {
		double *a = self->alphas + 4 * i;
		double *pc = self->object_pts_in_camera + 3 * i;

		for (int j = 0; j < 3; j++)
			pc[j] = a[0] * self->control_points_in_camera[0][j] + a[1] * self->control_points_in_camera[1][j] +
					a[2] * self->control_points_in_camera[2][j] + a[3] * self->control_points_in_camera[3][j];
	}
}

void epnp_compute_L_6x10(epnp *self, const double *ut, double *l_6x10) {
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
		A1 = SV_MALLOC(sizeof(double) * nr);
		A2 = SV_MALLOC(sizeof(double) * nr);
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

void epnp_find_betas_approx_3(epnp *self, const CvMat *L_6x10, const CvMat *Rho, double *betas) {
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

double epnp_compute_pose(epnp *self, double R[3][3], double t[3]) {
	epnp_choose_control_points(self);
	epnp_compute_barycentric_coordinates(self);

	CvMat *M = cvCreateMat(2 * self->number_of_correspondences, 12, CV_64F);

	for (int i = 0; i < self->number_of_correspondences; i++)
		epnp_fill_M(self, M, 2 * i, self->alphas + 4 * i, self->meas[2 * i], self->meas[2 * i + 1]);

	double mtm[12 * 12], d[12], ut[12 * 12];
	CvMat MtM = cvMat(12, 12, CV_64F, mtm);
	CvMat D = cvMat(12, 1, CV_64F, d);
	CvMat Ut = cvMat(12, 12, CV_64F, ut);

	cvMulTransposed(M, &MtM, 1, 0, 1);

	cvSVD(&MtM, &D, &Ut, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);
	cvReleaseMat(&M);

	/*  double gt[] = {0.907567, -0.00916941, -0.0637565, -0.239863, 0.00224965, 0.0225974, -0.239574, 0.00209046,
	   0.0176213, -0.237255, 0.00251711, -0.0108157,
		   0.00910763, 0.909518, 0.0026331, -0.00232957, -0.239824, 0.00409253, -0.00243169, -0.239934, 0.00316978,
	   -0.0024403, -0.239909, -0.0016784,
		   -0.00657473, -0.00182409, -0.118455, -0.418384, -0.0208829, -0.00537926, -0.341435, -0.198683, 0.0639791,
	   0.777439, 0.211238, 0.0351144,
		   -0.000558729, -0.00120335, 0.0410987, 0.435735, 0.470224, -0.0117729, -0.330236, -0.651751, 0.0877612,
	   -0.112846, 0.179057, -0.0293607,
		   0.000207011, -0.000114796, 1.30348e-05, -0.150349, -0.396757, -0.0336814, 0.362168, -0.332794, -0.0038853,
	   -0.215378, 0.728371, 3.59307e-05,
		   -0.000236456, 3.59257e-05, -0.00240085, -0.516359, 0.533741, 8.75851e-05, 0.550447, -0.29792, -0.00101687,
	   -0.0338867, -0.235687, -0.00652534,
		   0.367037, -0.0382166, -0.268689, 0.518886, -0.0415839, 0.198992, 0.504361, -0.0564282, 0.00252704, 0.456381,
	   -0.0480543, 0.11356,
		   -0.0477438, -0.404345, -0.0789953, -0.0475805, -0.514161, -0.108317, -0.0431554, -0.498573, 0.134824,
	   -0.0719115, -0.52184, 0.0593704,
		   0.172473, -0.0624523, 0.798148, 0.0821341, -0.0877883, -0.120482, 0.105865, -0.083816, -0.254253, 0.24317,
	   -0.056877, -0.393827,
		   -0.0555454, -0.0526344, 0.0122309, -0.0649974, -0.0336308, 0.479865, -0.117645, -0.135477, -0.783616,
	   -0.0585432, -0.034449, 0.327881,
		   0.0797424, 0.032575, 0.168567, 0.0597489, 0.0568341, -0.66392, 0.0387932, 0.0297936, -0.142108, 0.0542191,
	   0.0221337, 0.700399,
		   -0.00310509, 0.000734298, -0.485965, 0.0476647, 0.0218702, -0.51114, -0.00347318, -0.0252922, -0.520376,
	   0.00830308, -0.0120006, -0.477658 };
		   for(int i = 0;i < 144;i++) ut[i] = gt[i];*/
	assert(Ut.data.db == ut);

	double l_6x10[6 * 10], rho[6];
	CvMat L_6x10 = cvMat(6, 10, CV_64F, l_6x10);
	CvMat Rho = cvMat(6, 1, CV_64F, rho);

	epnp_compute_L_6x10(self, ut, l_6x10);

	epnp_compute_rho(self, rho);

	double Betas[4][4] = {0}, rep_errors[4] = {0};
	double Rs[4][3][3] = {0}, ts[4][3] = {0};

	find_betas_approx_1(&L_6x10, &Rho, Betas[1]);
	gauss_newton(&L_6x10, &Rho, Betas[1]);

	rep_errors[1] = epnp_compute_R_and_t(self, ut, Betas[1], Rs[1], ts[1]);

	find_betas_approx_2(&L_6x10, &Rho, Betas[2]);
	gauss_newton(&L_6x10, &Rho, Betas[2]);
	rep_errors[2] = epnp_compute_R_and_t(self, ut, Betas[2], Rs[2], ts[2]);

	epnp_find_betas_approx_3(self, &L_6x10, &Rho, Betas[3]);
	gauss_newton(&L_6x10, &Rho, Betas[3]);
	rep_errors[3] = epnp_compute_R_and_t(self, ut, Betas[3], Rs[3], ts[3]);

	int N = 1;
	if (rep_errors[2] < rep_errors[1])
		N = 2;
	if (rep_errors[3] < rep_errors[N])
		N = 3;

	copy_R_and_t(Rs[N], ts[N], R, t);

	return rep_errors[N];
}

double epnp_reprojection_error(epnp *self, const double R[3][3], const double t[3]) {
	double sum2 = 0.0;

	for (int i = 0; i < self->number_of_correspondences; i++) {
		double *pw = self->obj_pts + 3 * i;
		double Xc = dot(R[0], pw) + t[0];
		double Yc = dot(R[1], pw) + t[1];
		double inv_Zc = 1.0 / (dot(R[2], pw) + t[2]);
		double ue = self->uc + self->fu * Xc * inv_Zc;
		double ve = self->vc + self->fv * Yc * inv_Zc;
		double u = self->meas[2 * i], v = self->meas[2 * i + 1];

		sum2 += sqrt((u - ue) * (u - ue) + (v - ve) * (v - ve));
	}

	return sum2 / self->number_of_correspondences;
}

void epnp_estimate_R_and_t(epnp *self, double R[3][3], double t[3]) {
	double pc0[3], pw0[3];

	pc0[0] = pc0[1] = pc0[2] = 0.0;
	pw0[0] = pw0[1] = pw0[2] = 0.0;

	for (int i = 0; i < self->number_of_correspondences; i++) {
		const double *pc = self->object_pts_in_camera + 3 * i;
		const double *pw = self->obj_pts + 3 * i;

		for (int j = 0; j < 3; j++) {
			pc0[j] += pc[j];
			pw0[j] += pw[j];
		}
	}
	for (int j = 0; j < 3; j++) {
		pc0[j] /= self->number_of_correspondences;
		pw0[j] /= self->number_of_correspondences;
	}

	double abt[3 * 3], abt_d[3], abt_u[3 * 3], abt_v[3 * 3];
	CvMat ABt = cvMat(3, 3, CV_64F, abt);
	CvMat ABt_D = cvMat(3, 1, CV_64F, abt_d);
	CvMat ABt_U = cvMat(3, 3, CV_64F, abt_u);
	CvMat ABt_V = cvMat(3, 3, CV_64F, abt_v);

	cvSetZero(&ABt);

	for (int i = 0; i < self->number_of_correspondences; i++) {
		double *pc = self->object_pts_in_camera + 3 * i;
		double *pw = self->obj_pts + 3 * i;

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

void epnp_solve_for_sign(epnp *self) {
	if (self->object_pts_in_camera[2] < 0.0) {
		for (int i = 0; i < 4; i++)
			for (int j = 0; j < 3; j++)
				self->control_points_in_camera[i][j] = -self->control_points_in_camera[i][j];

		for (int i = 0; i < self->number_of_correspondences; i++) {
			self->object_pts_in_camera[3 * i] = -self->object_pts_in_camera[3 * i];
			self->object_pts_in_camera[3 * i + 1] = -self->object_pts_in_camera[3 * i + 1];
			self->object_pts_in_camera[3 * i + 2] = -self->object_pts_in_camera[3 * i + 2];
		}
	}
}

double epnp_compute_R_and_t(epnp *self, const double *ut, const double *betas, double R[3][3], double t[3]) {
	epnp_compute_ccs(self, betas, ut);
	epnp_compute_pcs(self);

	epnp_solve_for_sign(self);

	epnp_estimate_R_and_t(self, R, t);

	return epnp_reprojection_error(self, R, t);
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
