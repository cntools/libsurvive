#include "survive_kalman.h"
#include <malloc.h>
#include <memory.h>
#include <minimal_opencv.h>

#include "math.h"

typedef struct CvMat survive_kalman_gain_matrix;

#define KALMAN_LOG_LEVEL 1000

#define CREATE_STACK_MAT(name, rows, cols)                                                                             \
	FLT *_##name = alloca(rows * cols * sizeof(FLT));                                                                  \
	CvMat name = cvMat(rows, cols, SURVIVE_CV_F, _##name);

#define SV_KALMAN_VERBOSE(lvl, fmt, ...)                                                                               \
	{                                                                                                                  \
		if (log_level >= lvl) {                                                                                        \
			fprintf(stdout, fmt "\n", __VA_ARGS__);                                                                    \
		}                                                                                                              \
	}
static inline void mat_eye_diag_cov(CvMat *m, const FLT *v) {
	for (int i = 0; i < m->rows; i++) {
		for (int j = 0; j < m->cols; j++) {
			CV_FLT_PTR(m)[j * m->cols + i] = i == j ? (v[i] >= 0 ? v[i] : 1e5) : 0.;
		}
	}
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
		fprintf(stdout, "null%c", term);
		return;
	}
	fprintf(stdout, "%s %d x %d:%c", name, M->rows, M->cols, term);
	for (unsigned i = 0; i < M->rows; i++) {
		for (unsigned j = 0; j < M->cols; j++) {
			FLT v = cvmGet(M, i, j);
			if (v == 0)
				fprintf(stdout, "         0,\t");
			else
				fprintf(stdout, "%+5.2e,\t", v);
		}
		if (newlines)
			fprintf(stdout, "\n");
	}
	fprintf(stdout, "\n");
}
static void sv_print_mat(const char *name, const CvMat *M, bool newlines) {
	sv_print_mat_v(KALMAN_LOG_LEVEL, name, M, newlines);
}

static void kalman_linear_predict(FLT t, const survive_kalman_state_t *k, const CvMat *x_t0_t0, CvMat *x_t0_t1) {
	int state_cnt = k->state_cnt;
	CREATE_STACK_MAT(F, state_cnt, state_cnt);
	k->F_fn(t, _F, x_t0_t0);

	// X_k|k-1 = F * X_k-1|k-1
	cvGEMM(&F, x_t0_t0, 1, 0, 0, x_t0_t1, 0);
}

void user_is_q(void *user, FLT t, const struct CvMat *x, FLT *Q_out) {
	const FLT *q = (const FLT *)user;
	scalend(Q_out, q, t, x->rows * x->rows);
}

SURVIVE_EXPORT void survive_kalman_state_reset(survive_kalman_state_t *k) {
	k->t = 0;
	memset(k->P, 0, k->state_cnt * k->state_cnt * sizeof(FLT));
}

void survive_kalman_state_init(survive_kalman_state_t *k, size_t state_cnt, kalman_transition_fn_t F,
							   kalman_process_noise_fn_t q_fn, void *user, FLT *state) {
	memset(k, 0, sizeof(*k));

	k->state_cnt = (int)state_cnt;
	k->F_fn = F;
	k->Q_fn = q_fn ? q_fn : user_is_q;

	k->P = SV_CALLOC(1, sizeof(FLT) * state_cnt * state_cnt);

	k->Predict_fn = kalman_linear_predict;
	k->user = user;

	k->state = state;

	if (!k->state) {
		k->State_is_heap = true;
		k->state = SV_CALLOC(1, sizeof(FLT) * k->state_cnt);
	}
}

void survive_kalman_state_free(survive_kalman_state_t *k) {
	free(k->P);
	k->P = 0;

	if (k->State_is_heap)
		free(k->state);
	k->state = 0;
}

void survive_kalman_predict_covariance(FLT t, const CvMat *F, const CvMat *x, survive_kalman_state_t *k) {
	int dims = k->state_cnt;

	CvMat Pk1_k1 = cvMat(dims, dims, SURVIVE_CV_F, (void *)k->P);
	sv_print_mat("Pk-1_k-1", &Pk1_k1, 1);
	CREATE_STACK_MAT(Q, dims, dims);
	k->Q_fn(k->user, t, x, _Q);

	// k->P = F * k->P * F^T + Q
	mulBABt(&Pk1_k1, F, 1, &Q, 1, &Pk1_k1);

	if (log_level >= KALMAN_LOG_LEVEL) {
		SV_KALMAN_VERBOSE(110, "T: %f", t);
		sv_print_mat("Q", &Q, 1);
		sv_print_mat("F", F, 1);
		sv_print_mat("Pk1_k-1", &Pk1_k1, 1);
	}
}

static void survive_kalman_update_covariance(survive_kalman_state_t *k, survive_kalman_gain_matrix *K,
											 const struct CvMat *H, const CvMat *R) {
	int dims = k->state_cnt;

	CvMat Pk_k = cvMat(dims, dims, SURVIVE_CV_F, k->P);

	CREATE_STACK_MAT(Pk_k1Ht, dims, H->rows);

	// Pk_k1Ht = P_k|k-1 * H^T
	cvGEMM(&Pk_k, H, 1, 0, 0, &Pk_k1Ht, CV_GEMM_B_T);
	CREATE_STACK_MAT(S, H->rows, H->rows);

	sv_print_mat("H", H, 1);
	sv_print_mat("R", R, 1);

	// S = H * P_k|k-1 * H^T
	cvGEMM(H, &Pk_k1Ht, 1, R, 1, &S, 0);

	sv_print_mat("Pk_k1Ht", &Pk_k1Ht, 1);
	sv_print_mat("S", &S, 1);

	CREATE_STACK_MAT(iS, H->rows, H->rows);
	cvInvert(&S, &iS, DECOMP_LU);

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

	if (log_level >= KALMAN_LOG_LEVEL) {
		fprintf(stdout, "INFO gain\t");
		sv_print_mat("K", K, true);

		sv_print_mat("ikh", &ikh, true);

		fprintf(stdout, "INFO new Pk_k\t");
		sv_print_mat("Pk_k", &Pk_k, true);
	}
}

static inline void survive_kalman_predict(FLT t, survive_kalman_state_t *k, const CvMat *x_t0_t0, CvMat *x_t0_t1) {
	// X_k|k-1 = Predict(X_K-1|k-1)
	if (log_level > KALMAN_LOG_LEVEL) {
		fprintf(stdout, "INFO kalman_predict from ");
		sv_print_mat("x_t0_t0", x_t0_t0, false);
	}
	if (t == k->t) {
		cvCopy(x_t0_t0, x_t0_t1, 0);
	} else {
		k->Predict_fn(t - k->t, k, x_t0_t0, x_t0_t1);
	}
	if (log_level > KALMAN_LOG_LEVEL) {
		fprintf(stdout, "INFO kalman_predict to ");
		sv_print_mat("x_t0_t1", x_t0_t1, false);
	}
}

static void linear_update(FLT dt, survive_kalman_state_t *k, const CvMat *y, const CvMat *K, const CvMat *x_t0,
						  CvMat *x_t1) {
	//// y = Z - H * X_K|k-1
	// cvGEMM(H, x_t0, -1, &Z, 1, &y, 0);

	if (log_level > KALMAN_LOG_LEVEL) {
		fprintf(stdout, "INFO linear_update dt=%f ", dt);
		sv_print_mat("y", y, false);

		CREATE_STACK_MAT(tmp, x_t1->rows, x_t1->cols);
		cvGEMM(K, y, 1, 0, 0, &tmp, 0);
		sv_print_mat("K*y", &tmp, false);
	}

	assert(x_t0 != x_t1);
	// X_k|k = X_k|k-1 + K * y
	cvGEMM(K, y, 1, x_t0, 1, x_t1, 0);
}

static CvMat *survive_kalman_find_residual(FLT dt, survive_kalman_state_t *k, kalman_measurement_model_fn_t Hfn,
										   void *user, const struct CvMat *Z, const struct CvMat *x, CvMat *y,
										   CvMat *H) {
	for (int i = 0; i < H->rows * H->cols; i++)
		CV_FLT_PTR(H)[i] = INFINITY;
	CvMat *rtn = 0;
	if (Hfn) {
		// typedef void (*kalman_measurement_model_fn_t)(void* user, FLT t, const struct CvMat * Z, const struct CvMat
		// *x_t, struct CvMat* h_x_t, struct CvMat* H_k);
		bool okay = Hfn(user, Z, x, y, H);
		if (okay == false) {
			return 0;
		}
		sv_print_mat_v(600, "Hk", H, true);
		rtn = H;
	} else {
		rtn = (struct CvMat *)user;
		cvGEMM(rtn, x, -1, Z, 1, y, 0);
	}
	for (int i = 0; i < H->rows * H->cols; i++)
		assert(isfinite(CV_FLT_PTR(rtn)[i]));
	return rtn;
}

static FLT survive_kalman_predict_update_state_extended_adaptive_internal(FLT t, survive_kalman_state_t *k,
																		  const struct CvMat *Z, FLT *Rv,
																		  kalman_measurement_model_fn_t Hfn, void *user,
																		  bool adaptive) {
	int state_cnt = k->state_cnt;
	struct CvMat *H = 0;
	FLT dt = t - k->t;

	// Anything coming in this soon is liable to spike stuff since dt is so small
	if (dt < 1e-5) {
		dt = 0;
		t = k->t;
	}

	CREATE_STACK_MAT(Pm, state_cnt, state_cnt);
	// Adaptive update happens on the covariance matrix prior; so save it.
	if (adaptive)
		memcpy(_Pm, k->P, sizeof(FLT) * state_cnt * state_cnt);

	CREATE_STACK_MAT(y, Z->rows, Z->cols);

	// To avoid an unneeded copy, x1 here is both X_k-1|k-1 and X_k|k.
	// x is X_k|k-1
	CvMat x1 = cvMat(state_cnt, 1, SURVIVE_CV_F, k->state);

	// Predict x
	CREATE_STACK_MAT(x2, state_cnt, 1);
	survive_kalman_predict(t, k, &x1, &x2);

	CREATE_STACK_MAT(HStorage, Z->rows, state_cnt);
	H = survive_kalman_find_residual(dt, k, Hfn, user, Z, &x2, &y, &HStorage);

	if (log_level > KALMAN_LOG_LEVEL) {
		fprintf(stdout, "INFO kalman_predict_update_state_extended t=%f dt=%f ", t, dt);
		sv_print_mat("Z", Z, false);
		fprintf(stdout, "\n");
	}

	if (dt > 0) {
		CREATE_STACK_MAT(F, state_cnt, state_cnt);
		for (int i = 0; i < state_cnt * state_cnt; i++)
			_F[i] = NAN;

		k->F_fn(dt, _F, &x1);
		for (int i = 0; i < F.rows * F.cols; i++)
			assert(isfinite(_F[i]));

		// Run predict
		survive_kalman_predict_covariance(dt, &F, &x2, k);
	}

	// Run update; filling in K
	CREATE_STACK_MAT(K, state_cnt, Z->rows);
	FLT *Rs = adaptive ? Rv : alloca(Z->rows * Z->rows * sizeof(FLT));
	CvMat R = cvMat(Z->rows, Z->rows, SURVIVE_CV_F, Rs);
	if (!adaptive) {
		mat_eye_diag(&R, Rv);
	}

	survive_kalman_update_covariance(k, &K, H, &R);

	linear_update(dt, k, &y, &K, &x2, &x1);

	if (log_level > KALMAN_LOG_LEVEL) {
		fprintf(stdout, "INFO kalman_update to    ");
		sv_print_mat("x1", &x1, false);
	}

	if (adaptive) {
		// https://arxiv.org/pdf/1702.00884.pdf
		CREATE_STACK_MAT(PostHStorage, Z->rows, state_cnt);
		CREATE_STACK_MAT(HPkHt, Z->rows, Z->rows);
		CREATE_STACK_MAT(yyt, Z->rows, Z->rows);

		CvMat *PostH = survive_kalman_find_residual(dt, k, Hfn, user, Z, &x1, &y, &PostHStorage);
		cvMulTransposed(&y, &yyt, false, 0, 1);

		CREATE_STACK_MAT(Pk_k1Ht, state_cnt, H->rows);

		cvGEMM(&Pm, PostH, 1, 0, 0, &Pk_k1Ht, CV_GEMM_B_T);
		cvGEMM(PostH, &Pk_k1Ht, 1, 0, 0, &HPkHt, 0);

		sv_print_mat_v(200, "PostH", PostH, true);
		sv_print_mat_v(200, "PkHt", &Pk_k1Ht, true);
		sv_print_mat_v(200, "HpkHt", &HPkHt, true);
		sv_print_mat_v(200, "yyt", &yyt, true);

		FLT a = .3;
		FLT b = 1 - a;
		for (int i = 0; i < Z->rows; i++) {
			for (int j = 0; j < Z->rows; j++) {
				size_t idx = i + j * Z->rows;

				// HPkHt should in theory have positive diagonal but
				// rounding errors can push it over. Absolute value of it here
				// to preserve a positive diaganol in R.
				FLT HpkH = i == j ? fabs(_HPkHt[idx]) : _HPkHt[idx];
				Rs[idx] = a * Rs[idx] + b * (_yyt[idx] + HpkH);
			}
		}

		sv_print_mat_v(200, "Adaptive R", &R, true);
	}

	k->t = t;

	return normnd(_y, y.rows * y.cols);
}

FLT survive_kalman_predict_update_state_extended(FLT t, survive_kalman_state_t *k, const struct CvMat *Z, const FLT *R,
												 kalman_measurement_model_fn_t Hfn, void *user, bool adaptive) {
	return survive_kalman_predict_update_state_extended_adaptive_internal(t, k, Z, (FLT *)R, Hfn, user, adaptive);
}

FLT survive_kalman_predict_update_state(FLT t, survive_kalman_state_t *k, const struct CvMat *Z, const struct CvMat *H,
										const FLT *R, bool adaptive) {
	return survive_kalman_predict_update_state_extended(t, k, Z, R, 0, (void *)H, adaptive);
}

void survive_kalman_predict_state(FLT t, const survive_kalman_state_t *k, size_t start_index, size_t end_index,
								  FLT *_out) {
	CREATE_STACK_MAT(tmpOut, k->state_cnt, 1);
	CvMat x = cvMat(k->state_cnt, 1, SURVIVE_CV_F, k->state);

	FLT dt = t == 0. ? 0 : t - k->t;
	FLT *copyFrom = k->state;
	if (dt > 0) {
		k->Predict_fn(dt, k, &x, &tmpOut);
		copyFrom = _tmpOut;
	}
	memcpy(_out, copyFrom + start_index, (end_index - start_index) * sizeof(FLT));
}
void survive_kalman_set_P(survive_kalman_state_t *k, const FLT *p) {
	CvMat P = cvMat(k->state_cnt, k->state_cnt, SURVIVE_CV_F, k->P);
	mat_eye_diag(&P, p);
}
