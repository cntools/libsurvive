#include "survive_kalman.h"
#include <malloc.h>
#include <memory.h>
#include <minimal_opencv.h>

#include "math.h"

#define KALMAN_LOG_LEVEL 1000

#define CREATE_STACK_MAT(name, rows, cols)                                                                             \
	FLT *_##name = alloca(rows * cols * sizeof(FLT));                                                                  \
	CvMat name = cvMat(rows, cols, SURVIVE_CV_F, _##name);

#define SV_KALMAN_VERBOSE(lvl, fmt, ...)                                                                               \
	{                                                                                                                  \
		if (log_level >= lvl) {                                                                                        \
			fprintf(stderr, fmt "\n", __VA_ARGS__);                                                                    \
		}                                                                                                              \
	}
static inline void mat_eye_diag(CvMat *m, const FLT *v) {
	for (int i = 0; i < m->rows; i++) {
		for (int j = 0; j < m->cols; j++) {
			CV_FLT_PTR(m)[j * m->cols + i] = i == j ? v[i] : 0.;
		}
	}
}
static inline void mat_eye(CvMat *m, FLT v) {
	for (int i = 0; i < m->rows; i++) {
		for (int j = 0; j < m->cols; j++) {
			CV_FLT_PTR(m)[j * m->cols + i] = i == j ? v : 0.;
		}
	}
}

static int log_level = 0;
void survive_kalman_set_logging_level(int v) { log_level = v; }

static void sv_print_mat_v(int ll, const char *name, const CvMat *M, bool newlines) {
	if (log_level < ll) {
		return;
	}
	char term = newlines ? '\n' : ' ';
	if (!M) {
		fprintf(stderr, "null%c", term);
		return;
	}
	fprintf(stderr, "%s %d x %d:%c", name, M->rows, M->cols, term);
	for (unsigned i = 0; i < M->rows; i++) {
		for (unsigned j = 0; j < M->cols; j++) {
			fprintf(stderr, "%+8.8f,\t", cvmGet(M, i, j));
		}
		if (newlines)
			fprintf(stderr, "\n");
	}
	fprintf(stderr, "\n");
}
static void sv_print_mat(const char *name, const CvMat *M, bool newlines) {
	sv_print_mat_v(KALMAN_LOG_LEVEL, name, M, newlines);
}

static void kalman_linear_predict(FLT t, const survive_kalman_state_t *k, const CvMat *x_t0_t0, CvMat *x_t0_t1) {
	int state_cnt = k->info.state_cnt;
	CREATE_STACK_MAT(F, state_cnt, state_cnt);
	k->info.F_fn(t, _F, x_t0_t0);

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

	for (int i = 0; i < state_cnt; i++) {
		k->P[i * state_cnt + i] = 0;
	}
	k->Predict_fn = kalman_linear_predict;
}

void survive_kalman_free(survive_kalman_t *k) {
	if (k->P_is_heap)
		free(k->P);
	k->P = 0;
}

void survive_kalman_state_init(survive_kalman_state_t *k, size_t state_cnt, F_fn_t F, const FLT *Q_per_sec, FLT *P,
							   FLT *state) {
	memset(k, 0, sizeof(*k));
	survive_kalman_init(&k->info, state_cnt, F, Q_per_sec, P);

	k->state = state;

	if (!k->state) {
		k->State_is_heap = true;
		k->state = SV_CALLOC(1, sizeof(FLT) * k->info.state_cnt);
	}
}

void survive_kalman_state_free(survive_kalman_state_t *k) {
	survive_kalman_free(&k->info);
	if (k->State_is_heap)
		free(k->state);
	k->state = 0;
}

void survive_kalman_predict_covariance(FLT t, const CvMat *F, survive_kalman_t *k) {
	int dims = k->state_cnt;

	CREATE_STACK_MAT(tmp, dims, dims);

	CvMat Pk1_k1 = cvMat(dims, dims, SURVIVE_CV_F, (void *)k->P);
	sv_print_mat("Pk-1_k-1", &Pk1_k1, 1);
	const CvMat Q_per_sec = cvMat(dims, dims, SURVIVE_CV_F, (void *)k->Q_per_sec);

	// tmp = k->P * F^T
	cvGEMM(&Pk1_k1, F, 1, 0, 0, &tmp, CV_GEMM_B_T);

	// k->P = F * tmp + Q * t
	cvGEMM(F, &tmp, 1, &Q_per_sec, t, &Pk1_k1, 0);

	if (log_level > KALMAN_LOG_LEVEL) {
		SV_KALMAN_VERBOSE(110, "T: %f", t);
		sv_print_mat("Q", &Q_per_sec, 1);
		sv_print_mat("F", F, 1);
		sv_print_mat("tmp", &tmp, 1);
		sv_print_mat("Pk1_k-1", &Pk1_k1, 1);
	}
}

void survive_kalman_update_covariance(survive_kalman_t *k, survive_kalman_gain_matrix *K,
									  const survive_kalman_measurement_matrix *H, const FLT *Rv) {
	int dims = k->state_cnt;

	CvMat Pk_k = cvMat(dims, dims, SURVIVE_CV_F, k->P);

	CREATE_STACK_MAT(Pk_k1Ht, dims, H->rows);

	// Pk_k1Ht = P_k|k-1 * H^T
	cvGEMM(&Pk_k, H, 1, 0, 0, &Pk_k1Ht, CV_GEMM_B_T);
	CREATE_STACK_MAT(S, H->rows, H->rows);
	CREATE_STACK_MAT(R, H->rows, H->rows);

	mat_eye_diag(&R, Rv);

	sv_print_mat("H", H, 1);
	sv_print_mat("R", &R, 1);

	// S = H * P_k|k-1 * H^T + R
	cvGEMM(H, &Pk_k1Ht, 1, &R, 1, &S, 0);

	CREATE_STACK_MAT(iS, H->rows, H->rows);
	cvInvert(&S, &iS, DECOMP_LU);

	sv_print_mat("Pk_k1Ht", &Pk_k1Ht, 1);
	sv_print_mat("S", &S, 1);
	sv_print_mat("iS", &iS, 1);

	cvGEMM(&Pk_k1Ht, &iS, 1, 0, 0, K, 0);

	// Apparently cvEye isn't a thing!?
	CREATE_STACK_MAT(eye, dims, dims);
	mat_eye(&eye, 1);

	CREATE_STACK_MAT(ikh, dims, dims);

	// ikh = (I - K * H)
	cvGEMM(K, H, -1, &eye, 1, &ikh, 0);

	// cvGEMM does not like the same addresses for src and destination...
	CREATE_STACK_MAT(tmp, dims, dims);
	cvCopy(&Pk_k, &tmp, 0);

	// P_k|k = (I - K * H) * P_k|k-1
	cvGEMM(&ikh, &tmp, 1, 0, 0, &Pk_k, 0);

	if (log_level > KALMAN_LOG_LEVEL) {
		fprintf(stderr, "INFO gain\t");
		sv_print_mat("K", K, true);

		fprintf(stderr, "INFO new Pk_k\t");
		sv_print_mat("Pk_k", &Pk_k, true);
	}

	if (log_level >= 110) {
		CREATE_STACK_MAT(ones, K->cols, 1);
		CREATE_STACK_MAT(gain_vector, K->rows, 1);
		for (int i = 0; i < K->cols; i++)
			_ones[i] = 1;
		cvGEMM(K, &ones, 1, 0, 0, &gain_vector, 0);
		sv_print_mat_v(110, "Info: Gains ", &gain_vector, 0);
	}
}

void survive_kalman_predict_update_covariance(FLT t, survive_kalman_t *k, survive_kalman_gain_matrix *K, const CvMat *F,
											  const survive_kalman_measurement_matrix *H, const FLT *R) {
	survive_kalman_predict_covariance(t, F, k);
	survive_kalman_update_covariance(k, K, H, R);
}

static inline void survive_kalman_predict(FLT t, survive_kalman_state_t *k, const CvMat *x_t0_t0, CvMat *x_t0_t1) {
	// X_k|k-1 = Predict(X_K-1|k-1)
	if (log_level > KALMAN_LOG_LEVEL) {
		fprintf(stderr, "INFO kalman_predict from ");
		sv_print_mat("x_t0_t0", x_t0_t0, false);
		fprintf(stderr, "\n");
	}
	k->info.Predict_fn(t - k->t, k, x_t0_t0, x_t0_t1);
	if (log_level > KALMAN_LOG_LEVEL) {
		fprintf(stderr, "INFO kalman_predict to ");
		sv_print_mat("x_t0_t1", x_t0_t1, false);
		fprintf(stderr, "\n");
	}
}

static void linear_update(FLT t, survive_kalman_state_t *k, const CvMat *y, const CvMat *K, const CvMat *x_t0,
						  CvMat *x_t1) {
	//// y = Z - H * X_K|k-1
	// cvGEMM(H, x_t0, -1, &Z, 1, &y, 0);

	if (log_level > KALMAN_LOG_LEVEL) {
		fprintf(stderr, "INFO linear_update t=%f dt=%f ", t, t - k->t);
		sv_print_mat("y", y, false);
		fprintf(stderr, "\n");
		CREATE_STACK_MAT(tmp, x_t1->rows, x_t1->cols);
		cvGEMM(K, y, 1, 0, 0, &tmp, 0);
		sv_print_mat("K*y", &tmp, false);
		fprintf(stderr, "\n");
	}

	assert(x_t0 != x_t1);
	// X_k|k = X_k|k-1 + K * y
	cvGEMM(K, y, 1, x_t0, 1, x_t1, 0);
}

void survive_kalman_predict_update_state_extended(FLT t, survive_kalman_state_t *k, const struct CvMat *Z, const FLT *R,
												  Map_to_obs Hfn, void *user) {
	int state_cnt = k->info.state_cnt;
	struct CvMat *H = 0;

	CREATE_STACK_MAT(K, state_cnt, Z->rows);
	CREATE_STACK_MAT(y, Z->rows, Z->cols);

	// To avoid an unneeded copy, x1 here is both X_k-1|k-1 and X_k|k.
	// x is X_k|k-1
	CvMat x1 = cvMat(state_cnt, 1, SURVIVE_CV_F, k->state);

	// Predict x
	CREATE_STACK_MAT(x2, state_cnt, 1);
	survive_kalman_predict(t, k, &x1, &x2);

	CREATE_STACK_MAT(HStorage, Z->rows, state_cnt);
	if (Hfn) {
		// typedef void (*Map_to_obs)(void* user, FLT t, const struct CvMat * Z, const struct CvMat *x_t, struct CvMat*
		// h_x_t, struct CvMat* H_k);
		Hfn(user, t - k->t, Z, &x2, &y, &HStorage);
		H = &HStorage;
	} else {
		H = (struct CvMat *)user;
		cvGEMM(H, &x2, -1, Z, 1, &y, 0);
	}

	if (log_level > KALMAN_LOG_LEVEL) {
		fprintf(stderr, "INFO kalman_predict_update_state_extended t=%f dt=%f ", t, t - k->t);
		sv_print_mat("Z", Z, false);
		fprintf(stderr, "\n");
	}

	CREATE_STACK_MAT(F, state_cnt, state_cnt);
	k->info.F_fn(t - k->t, _F, &x1);

	// Run predict
	survive_kalman_predict_covariance(t - k->t, &F, &k->info);

	// Run update; filling in K
	survive_kalman_update_covariance(&k->info, &K, H, R);

	linear_update(t, k, &y, &K, &x2, &x1);

	if (log_level > KALMAN_LOG_LEVEL) {
		fprintf(stderr, "INFO kalman_update to    ");
		sv_print_mat("x1", &x1, false);
		fprintf(stderr, "\n");
	}

	k->t = t;
}

void linear_measurement(void *user, FLT t, const struct CvMat *Z, const struct CvMat *x_t, struct CvMat *h_x_t,
						struct CvMat *H_k) {
	const survive_kalman_measurement_matrix *H;
}

void survive_kalman_predict_update_state(FLT t, survive_kalman_state_t *k, const struct CvMat *Z,
										 const survive_kalman_measurement_matrix *H, const FLT *R) {
	// survive_kalman_predict_update_state_extended(FLT t, survive_kalman_state_t *k, const struct CvMat *Z, const FLT*
	// R, Map_to_obs Hfn, void* user) {
	survive_kalman_predict_update_state_extended(t, k, Z, R, 0, (void *)H);
}

void survive_kalman_predict_state(FLT t, const survive_kalman_state_t *k, size_t start_index, size_t end_index,
								  FLT *_out) {
	CREATE_STACK_MAT(tmpOut, k->info.state_cnt, 1);
	CvMat x = cvMat(k->info.state_cnt, 1, SURVIVE_CV_F, k->state);

	FLT dt = t == 0. ? 0 : t - k->t;
	FLT *copyFrom = k->state;
	if (t > 0) {
		k->info.Predict_fn(dt, k, &x, &tmpOut);
		copyFrom = _tmpOut;
	}
	memcpy(_out, copyFrom + start_index, (end_index - start_index) * sizeof(FLT));
}
void survive_kalman_set_P(survive_kalman_state_t *k, const FLT *p) {
	CvMat P = cvMat(k->info.state_cnt, k->info.state_cnt, SURVIVE_CV_F, k->info.P);
	mat_eye_diag(&P, p);
	sv_print_mat_v(110, "P", &P, true);
}
