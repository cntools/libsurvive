#include "survive_kalman.h"
#include <malloc.h>
#include <memory.h>
#include <minimal_opencv.h>

#ifdef USE_DOUBLE
#define SURVIVE_CV_F CV_64F
#else
#define SURVIVE_CV_F CV_32F
#endif

#define CREATE_STACK_MAT(name, rows, cols)                                                                             \
	FLT *_##name = alloca(rows * cols * sizeof(FLT));                                                                  \
	CvMat name = cvMat(rows, cols, SURVIVE_CV_F, _##name);

static void kalman_linear_predict(FLT t, const survive_kalman_state_t *k, const CvMat *x_t0_t0, CvMat *x_t0_t1) {
	int state_cnt = k->info.state_cnt;
	CREATE_STACK_MAT(F, state_cnt, state_cnt);
	k->info.F_fn(t, _F);

	// X_k|k-1 = F * X_k-1|k-1
	cvGEMM(&F, x_t0_t0, 1, 0, 0, x_t0_t1, 0);
}

void survive_kalman_init(survive_kalman_t *k, size_t state_cnt, F_fn_t F, const FLT *Q_per_sec, FLT *P) {
	memset(k, 0, sizeof(*k));

	k->state_cnt = (int)state_cnt;
	k->F_fn = F;
	k->Q_per_sec = Q_per_sec;
	k->P = P;

	if (!k->P) {
		k->P_is_heap = true;
		k->P = SV_CALLOC(1, sizeof(FLT) * state_cnt * state_cnt);
	}

	k->P[0] = 1e10;
	k->Predict_fn = kalman_linear_predict;
}

void survive_kalman_free(survive_kalman_t *k) {
	if (k->P_is_heap)
		free(k->P);
	k->P = 0;
}

void survive_kalman_state_init(survive_kalman_state_t *k, size_t state_cnt, F_fn_t F, const FLT *Q_per_sec, FLT *P,
							   size_t *dims, FLT *state) {
	memset(k, 0, sizeof(*k));
	survive_kalman_init(&k->info, state_cnt, F, Q_per_sec, P);

	for (int i = 0; i < state_cnt; i++) {
		k->dimension_cnt[i] = dims[i];
		if (dims[i] > k->max_dim_cnt)
			k->max_dim_cnt = dims[i];
	}

	k->state = state;

	if (!k->state) {
		k->State_is_heap = true;
		k->state = SV_CALLOC(1, sizeof(FLT) * k->max_dim_cnt * k->info.state_cnt);
	}
}

void survive_kalman_state_free(survive_kalman_state_t *k) {
	survive_kalman_free(&k->info);
	if (k->State_is_heap)
		free(k->state);
	k->state = 0;
}
void print_mat(const CvMat *M) {
	if (!M) {
		printf("null\n");
		return;
	}
	printf("%d x %d:\n", M->rows, M->cols);
	for (unsigned i = 0; i < M->rows; i++) {
		for (unsigned j = 0; j < M->cols; j++) {
			printf("%+.17g,\t", cvmGet(M, i, j));
		}
		printf("\n");
	}
	printf("\n");
}

void survive_kalman_predict_covariance(FLT t, survive_kalman_t *k) {
	int dims = k->state_cnt;

	CREATE_STACK_MAT(F, dims, dims);
	CREATE_STACK_MAT(tmp, dims, dims);

	k->F_fn(t, _F);

	CvMat Pk1_k1 = cvMat(dims, dims, SURVIVE_CV_F, (void *)k->P);
	const CvMat Q_per_sec = cvMat(dims, dims, SURVIVE_CV_F, (void *)k->Q_per_sec);

	// k->P = F_K * k->P * F_K^T + Q*t
	cvGEMM(&Pk1_k1, &F, 1, 0, 0, &tmp, CV_GEMM_B_T);
	cvGEMM(&F, &tmp, 1, &Q_per_sec, t, &Pk1_k1, 0);
}

void survive_kalman_update_covariance(FLT t, survive_kalman_t *k, FLT *_K, const FLT *_H, FLT _R) {
	int dims = k->state_cnt;

	CvMat Pk_k = cvMat(dims, dims, SURVIVE_CV_F, k->P);
	CvMat K = cvMat(dims, 1, SURVIVE_CV_F, _K);
	const CvMat H = cvMat(1, dims, SURVIVE_CV_F, (void *)_H);

	CREATE_STACK_MAT(Pk_k1Ht, dims, 1);

	// Pk_k1Ht = P_k|k-1 * H^T
	cvGEMM(&Pk_k, &H, 1, 0, 0, &Pk_k1Ht, CV_GEMM_B_T);
	FLT _S;
	CvMat S = cvMat(1, 1, SURVIVE_CV_F, &_S);
	CvMat R = cvMat(1, 1, SURVIVE_CV_F, &_R);

	// S = H * P_k|k-1 * H^T + R
	cvGEMM(&H, &Pk_k1Ht, 1, &R, 1, &S, 0);

	FLT _Si = _S == 0 ? 1. : (1. / _S);

	// K = P_k|k-1*H^T*S^-1
	for (int i = 0; i < dims; i++)
		_K[i] = _Pk_k1Ht[i] * _Si;

	// Apparently cvEye isn't a thing!?
	CREATE_STACK_MAT(eye, dims, dims);
	memset(_eye, 0, sizeof(FLT) * dims * dims);
	for (int i = 0; i < dims; i++)
		_eye[i * dims + i] = 1.;

	CREATE_STACK_MAT(ikh, dims, dims);

	// ikh = (I - K * H)
	cvGEMM(&K, &H, -1, &eye, 1, &ikh, 0);

	// cvGEMM does not like the same addresses for src and destination...
	CREATE_STACK_MAT(tmp, dims, dims);
	cvCopy(&Pk_k, &tmp, 0);

	// P_k|k = (I - K * H) * P_k|k-1
	cvGEMM(&ikh, &tmp, 1, 0, 0, &Pk_k, 0);
}

void survive_kalman_predict_update_covariance(FLT t, survive_kalman_t *k, FLT *K, const FLT *H, FLT R) {
	survive_kalman_predict_covariance(t, k);
	survive_kalman_update_covariance(t, k, K, H, R);
}

static inline void survive_kalman_predict(FLT t, survive_kalman_state_t *k, const CvMat *x_t0_t0, CvMat *x_t0_t1) {
	// X_k|k-1 = Predict(X_K-1|k-1)
	k->info.Predict_fn(t, k, x_t0_t0, x_t0_t1);
}

void survive_kalman_predict_update_state_extended(FLT t, survive_kalman_state_t *k, const FLT *z, const FLT *_H,
												  Update_fn_t updateFn, FLT R) {
	int state_cnt = k->info.state_cnt;
	CREATE_STACK_MAT(K, state_cnt, 1);

	const CvMat H = cvMat(1, state_cnt, SURVIVE_CV_F, (void *)_H);

	// Run predict / update; filling in K
	survive_kalman_predict_update_covariance(t, &k->info, _K, _H, R);

	// To avoid an unneeded copy, x1 here is both X_k-1|k-1 and X_k|k.
	// x is X_k|k-1
	CvMat x1 = cvMat(state_cnt, k->max_dim_cnt, SURVIVE_CV_F, k->state);

	CREATE_STACK_MAT(x2, state_cnt, k->max_dim_cnt);
	survive_kalman_predict(t, k, &x1, &x2);

	updateFn(t, k, &H, &K, &x2, &x1, z);
}

static void linear_update(FLT t, survive_kalman_state_t *k, const CvMat *H, const CvMat *K, const CvMat *x_t0,
						  CvMat *x_t1, const FLT *z) {
	CvMat Z = cvMat(1, k->max_dim_cnt, SURVIVE_CV_F, (void *)(z));

	CREATE_STACK_MAT(y, 1, k->max_dim_cnt);
	// y = Z - H * X_K|k-1
	cvGEMM(H, x_t0, -1, &Z, 1, &y, 0);

	// X_k|k = X_k|k-1 + K * y
	cvGEMM(K, &y, 1, x_t0, 1, x_t1, 0);
}

void survive_kalman_predict_update_state(FLT t, survive_kalman_state_t *k, const FLT *z, const FLT *_H, FLT R) {
	survive_kalman_predict_update_state_extended(t, k, z, _H, linear_update, R);
}

void survive_kalman_predict_state(FLT t, const survive_kalman_state_t *k, size_t index, FLT *_out) {
	int state_cnt = k->info.state_cnt;
	CREATE_STACK_MAT(F, state_cnt, state_cnt);
	(void)F;

	k->info.F_fn(t, _F);

	CREATE_STACK_MAT(tmpOut, 1, k->max_dim_cnt);
	CvMat x = cvMat(state_cnt, k->max_dim_cnt, SURVIVE_CV_F, k->state);

	// Truncate F so that is only the 'index'nth row. This avoids unneeded multiplications / copies.
	CvMat FTrunc = cvMat(1, state_cnt, SURVIVE_CV_F, _F + state_cnt * index);
	cvGEMM(&FTrunc, &x, 1, 0, 0, &tmpOut, 0);

	memcpy(_out, _tmpOut, sizeof(FLT) * k->dimension_cnt[index]);
}
