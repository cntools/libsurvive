#include "survive_kalman.h"
#include <malloc.h>
#include <memory.h>
#include <minimal_opencv.h>

void survive_kalman_init(survive_kalman_t *k, size_t dims, F_fn_t F, const FLT *Q_per_sec, FLT *P) {
	memset(k, 0, sizeof(*k));

	k->dims = dims;
	k->F_fn = F;
	k->Q_per_sec = Q_per_sec;
	k->P = P;

	if (!k->P) {
		k->P_is_heap = true;
		k->P = calloc(1, sizeof(FLT) * dims * dims);
	}

	k->P[0] = 1e10;
}

void survive_kalman_state_init(survive_kalman_state_t *k, size_t dims, F_fn_t F, const FLT *Q_per_sec, FLT *P,
							   size_t state_size, FLT *state) {
	memset(k, 0, sizeof(*k));
	survive_kalman_init(&k->info, dims, F, Q_per_sec, P);

	k->state_size = state_size;
	k->state = state;

	if (!k->state) {
		k->State_is_heap = true;
		k->state = calloc(1, sizeof(FLT) * state_size * k->info.dims);
	}
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

#ifdef USE_DOUBLE
#define SURVIVE_CV_F CV_64F
#else
#define SURVIVE_CV_F CV_32F
#endif

void survive_kalman_predict(FLT t, survive_kalman_t *k) {
	size_t dims = k->dims;

	FLT *_F = alloca(dims * dims * sizeof(FLT));
	k->F_fn(t, _F);
	const CvMat F = cvMat(dims, dims, SURVIVE_CV_F, _F);

	CvMat Pk1_k1 = cvMat(dims, dims, SURVIVE_CV_F, (void *)k->P);
	const CvMat Q_per_sec = cvMat(dims, dims, SURVIVE_CV_F, (void *)k->Q_per_sec);

	FLT *tmpd = alloca(dims * dims * sizeof(FLT));
	CvMat tmp = cvMat(dims, dims, SURVIVE_CV_F, tmpd);
	cvGEMM(&Pk1_k1, &F, 1, 0, 0, &tmp, GEMM_2_T);

	cvGEMM(&F, &tmp, 1, &Q_per_sec, t, &Pk1_k1, 0);
}

void survive_kalman_update(FLT t, survive_kalman_t *k, FLT *_K, const FLT *_H, FLT _R) {
	size_t dims = k->dims;

	CvMat Pk_k = cvMat(dims, dims, SURVIVE_CV_F, k->P);
	CvMat K = cvMat(dims, 1, SURVIVE_CV_F, _K);
	const CvMat H = cvMat(1, dims, SURVIVE_CV_F, (void *)_H);

	FLT *_Pk_k1H = alloca(dims * sizeof(FLT));
	CvMat Pk_k1Ht = cvMat(dims, 1, SURVIVE_CV_F, _Pk_k1H);

	cvGEMM(&Pk_k, &H, 1, 0, 0, &Pk_k1Ht, GEMM_2_T);
	FLT _S;
	CvMat S = cvMat(1, 1, SURVIVE_CV_F, &_S);
	FLT _Si;
	CvMat Si = cvMat(1, 1, SURVIVE_CV_F, &_Si);

	CvMat R = cvMat(1, 1, SURVIVE_CV_F, &_R);
	cvGEMM(&H, &Pk_k1Ht, 1, &R, 1, &S, 0);

	_Si = _S == 0 ? 1. : (1. / _S);

	cvCopyTo(&Pk_k1Ht, &K);
	for (int i = 0; i < dims; i++)
		_K[i] = _K[i] * _Si;

	size_t dimsxdims_size = dims * dims * sizeof(FLT);
	FLT *_eye = alloca(dimsxdims_size);
	memset(_eye, 0, dimsxdims_size);
	for (int i = 0; i < dims; i++)
		_eye[i * dims + i] = 1.;
	CvMat eye = cvMat(dims, dims, SURVIVE_CV_F, _eye);

	FLT *_ikh = alloca(dimsxdims_size);
	CvMat ikh = cvMat(dims, dims, SURVIVE_CV_F, _ikh);
	cvGEMM(&K, &H, -1, &eye, 1, &ikh, 0);

	// print_mat(&ikh);

	FLT *tmpd = alloca(dimsxdims_size);
	CvMat tmp = cvMat(dims, dims, SURVIVE_CV_F, tmpd);
	cvCopyTo(&Pk_k, &tmp);

	cvGEMM(&ikh, &tmp, 1, 0, 0, &Pk_k, 0);

	/*
		printf("!ikh:\n");
		print_mat(&ikh);
		printf("!K:\n");
		print_mat(&K);
		printf("!Pk_k:\n");
		print_mat(&Pk_k);
	*/
}

void survive_kalman_predict_update(FLT t, survive_kalman_t *k, FLT *K, const FLT *H, FLT R) {
	survive_kalman_predict(t, k);
	survive_kalman_update(t, k, K, H, R);
}

void survive_kalman_predict_update_state(FLT t, survive_kalman_state_t *k, const FLT *z, const FLT *_H, FLT R) {
	FLT *_K = alloca(sizeof(FLT) * k->info.dims);
	CvMat K = cvMat(k->info.dims, 1, SURVIVE_CV_F, _K);

	survive_kalman_predict_update(t, &k->info, _K, _H, R);
	const CvMat H = cvMat(1, k->info.dims, SURVIVE_CV_F, (void *)_H);
	// print_mat(&K);

	int dims = k->info.dims;
	FLT *_F = alloca(dims * dims * sizeof(FLT));
	k->info.F_fn(t, _F);
	const CvMat F = cvMat(k->info.dims, k->info.dims, SURVIVE_CV_F, _F);

	for (int i = 0; i < k->state_size; i++) {
		CvMat x1 = cvMat(k->info.dims, 1, SURVIVE_CV_F, k->state + k->info.dims * i);
		FLT *_x = alloca(k->info.dims * sizeof(FLT));
		CvMat x = cvMat(k->info.dims, 1, SURVIVE_CV_F, _x);
		cvGEMM(&F, &x1, 1, 0, 0, &x, 0);

		FLT _y;
		CvMat y = cvMat(1, 1, SURVIVE_CV_F, &_y);
		CvMat Z = cvMat(1, 1, SURVIVE_CV_F, (void *)(z + i));
		cvGEMM(&H, &x, -1, &Z, 1, &y, 0);

		cvGEMM(&K, &y, 1, &x, 1, &x1, 0);

		// printf("%d:\n", i);
		// print_mat(&x1);
	}

	// CvMat X = cvMat(k->info.dims, k->state_size, SURVIVE_CV_F, k->state);
	// print_mat(&X);
}

void survive_kalman_get_state(FLT t, const survive_kalman_state_t *k, size_t index, FLT *out) {
	int dims = k->info.dims;
	FLT *_F = alloca(dims * dims * sizeof(FLT));
	k->info.F_fn(t, _F);
	const CvMat F = cvMat(k->info.dims, k->info.dims, SURVIVE_CV_F, _F);

	FLT *_x = alloca(k->info.dims * sizeof(FLT));
	CvMat x = cvMat(k->info.dims, 1, SURVIVE_CV_F, _x);

	for (int i = 0; i < k->state_size; i++) {
		CvMat x1 = cvMat(k->info.dims, 1, SURVIVE_CV_F, k->state + k->info.dims * i);
		cvGEMM(&F, &x1, 1, 0, 0, &x, 0);
		out[i] = _x[index];
	}
}
