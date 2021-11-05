#include "survive_kalman.h"
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h>
#endif
#include <memory.h>
#include <sv_matrix.h>

#include "math.h"

typedef struct SvMat survive_kalman_gain_matrix;

#define KALMAN_LOG_LEVEL 1000

#define SV_KALMAN_VERBOSE(lvl, fmt, ...)                                                                               \
	{                                                                                                                  \
		if (k->log_level >= lvl) {                                                                                     \
			fprintf(stdout, fmt "\n", __VA_ARGS__);                                                                    \
		}                                                                                                              \
	}

void survive_kalman_set_logging_level(survive_kalman_state_t *k, int v) { k->log_level = v; }

static inline FLT mul_at_ib_a(const struct SvMat *A, const struct SvMat *B) {
	FLT rtn = 0;
	SV_CREATE_STACK_MAT(V, 1, 1);
	SV_CREATE_STACK_MAT(iB, B->rows, B->cols);
	svInvert(B, &iB, SV_INVERT_METHOD_SVD);

	SV_CREATE_STACK_MAT(AtiB, A->cols, iB.cols);
	svGEMM(A, &iB, 1, 0, 0, &AtiB, SV_GEMM_FLAG_A_T);
	svGEMM(&AtiB, A, 1, 0, 0, &V, 0);

	rtn = V.data[0];
	SV_FREE_STACK_MAT(AtiB);
	SV_FREE_STACK_MAT(iB);
	SV_FREE_STACK_MAT(V);

	return rtn;
}

static inline FLT mul_at_b_a(const struct SvMat *A, const struct SvMat *B) {
	SV_CREATE_STACK_MAT(V, 1, 1);
	SV_CREATE_STACK_MAT(AtiB, A->cols, B->cols);
	svGEMM(A, B, 1, 0, 0, &AtiB, SV_GEMM_FLAG_A_T);
	svGEMM(&AtiB, A, 1, 0, 0, &V, 0);
	assert(V.data[0] >= 0);
	return V.data[0];
}

static void sv_print_mat_v(const survive_kalman_state_t *k, int ll, const char *name, const SvMat *M, bool newlines) {
	if (k->log_level < ll) {
		return;
	}
	char term = newlines ? '\n' : ' ';
	if (!M) {
		fprintf(stdout, "null%c", term);
		return;
	}
	fprintf(stdout, "%8s %2d x %2d:%c", name, M->rows, M->cols, term);
	FLT scale = sv_sum(M);
	for (unsigned i = 0; i < M->rows; i++) {
		for (unsigned j = 0; j < M->cols; j++) {
			FLT v = svMatrixGet(M, i, j);
			if (v == 0)
				fprintf(stdout, "         0,\t");
			else
				fprintf(stdout, "%+7.7e,\t", v);
		}
		if (newlines && M->cols > 1)
			fprintf(stdout, "\n");
	}
	fprintf(stdout, "\n");
}
static void sv_print_mat(survive_kalman_state_t *k, const char *name, const SvMat *M, bool newlines) {
	sv_print_mat_v(k, KALMAN_LOG_LEVEL, name, M, newlines);
}

static void kalman_linear_predict(FLT t, const survive_kalman_state_t *k, const SvMat *x_t0_t0, SvMat *x_t0_t1) {
	int state_cnt = k->state_cnt;
	SV_CREATE_STACK_MAT(F, state_cnt, state_cnt);
	k->F_fn(t, &F, x_t0_t0);

	// X_k|k-1 = F * X_k-1|k-1
	svGEMM(&F, x_t0_t0, 1, 0, 0, x_t0_t1, 0);
	SV_FREE_STACK_MAT(F);
}

void user_is_q(void *user, FLT t, const struct SvMat *x, SvMat *Q_out) {
	const SvMat *q = (const SvMat *)user;
	scalend(SV_FLT_PTR(Q_out), SV_FLT_PTR(q), t, x->rows * x->rows);
}

SURVIVE_EXPORT void survive_kalman_state_reset(survive_kalman_state_t *k) {
	k->t = 0;
	sv_set_zero(&k->P);

	k->Q_fn(k->user, 1e2, &k->state, &k->P);
	sv_print_mat(k, "initial Pk_k", &k->P, true);
}

static void transition_is_identity(FLT t, struct SvMat *f_out, const struct SvMat *x0) {
	sv_eye(f_out, 0);
}
SURVIVE_EXPORT void survive_kalman_meas_model_init(survive_kalman_state_t *k, const char *name,
												   survive_kalman_meas_model_t *mk, kalman_measurement_model_fn_t Hfn) {
	memset(mk, 0, sizeof(*mk));
	mk->k = k;
	mk->name = name;
	mk->Hfn = Hfn;
	mk->term_criteria = (struct term_criteria_t){.max_iterations = 5, .xtol = 1e-2, .minimum_step = .05};
}

void survive_kalman_state_init(survive_kalman_state_t *k, size_t state_cnt, kalman_transition_fn_t F,
							   kalman_process_noise_fn_t q_fn, void *user, FLT *state) {
	memset(k, 0, sizeof(*k));

	k->state_cnt = (int)state_cnt;
	k->F_fn = F ? F : transition_is_identity;
	k->Q_fn = q_fn ? q_fn : user_is_q;

	k->P = svMat(k->state_cnt, k->state_cnt, 0);

	k->Predict_fn = kalman_linear_predict;
	k->user = user;

	if (!state) {
		k->State_is_heap = true;
		state = SV_CALLOC(sizeof(FLT) * k->state_cnt);
	}

	k->state = svMat(k->state_cnt, 1, state);
}

void survive_kalman_state_free(survive_kalman_state_t *k) {
	free(k->P.data);
	k->P.data = 0;

	if (k->State_is_heap)
		free(SV_FLT_PTR(&k->state));
	k->state.data = 0;
}

void survive_kalman_predict_covariance(FLT t, const SvMat *F, const SvMat *x, survive_kalman_state_t *k) {
	int dims = k->state_cnt;

	SvMat *Pk1_k1 = &k->P;
	sv_print_mat(k, "Pk-1_k-1", Pk1_k1, 1);
	SV_CREATE_STACK_MAT(Q, dims, dims);
	k->Q_fn(k->user, t, x, &Q);

	// k->P = F * k->P * F^T + Q
	matrix_ABAt_add(Pk1_k1, F, Pk1_k1, &Q);

	if (k->log_level >= KALMAN_LOG_LEVEL) {
		SV_KALMAN_VERBOSE(110, "T: %f", t);
		sv_print_mat(k, "Q", &Q, 1);
		sv_print_mat(k, "F", F, 1);
		sv_print_mat(k, "Pk1_k-1", Pk1_k1, 1);
	}
	SV_FREE_STACK_MAT(Q);
}
static void survive_kalman_find_k(survive_kalman_state_t *k, survive_kalman_gain_matrix *K, const struct SvMat *H,
								  const SvMat *R) {
	int dims = k->state_cnt;

	const SvMat *Pk_k = &k->P;

	SV_CREATE_STACK_MAT(Pk_k1Ht, dims, H->rows);

	// Pk_k1Ht = P_k|k-1 * H^T
	svGEMM(Pk_k, H, 1, 0, 0, &Pk_k1Ht, SV_GEMM_FLAG_B_T);
	SV_CREATE_STACK_MAT(S, H->rows, H->rows);

	sv_print_mat(k, "H", H, 1);
	sv_print_mat(k, "R", R, 1);

	// S = H * P_k|k-1 * H^T + R
	svGEMM(H, &Pk_k1Ht, 1, R, 1, &S, 0);
	assert(sv_is_finite(&S));

	sv_print_mat(k, "Pk_k1Ht", &Pk_k1Ht, 1);
	sv_print_mat(k, "S", &S, 1);

	SV_CREATE_STACK_MAT(iS, H->rows, H->rows);

	FLT diag = 0, non_diag = 0;
	for (int i = 0; i < H->rows; i++) {
		for (int j = 0; j < H->rows; j++) {
			if (i == j) {
				diag += fabs(_S[i + j * H->rows]);
				_iS[i + j * H->rows] = 1. / _S[i + j * H->rows];
			} else {
				non_diag += fabs(_S[i + j * H->rows]);
				_iS[i + j * H->rows] = 0;
			}
		}
	}

	if (diag == 0 || non_diag / diag > 1e-5) {
		svInvert(&S, &iS, SV_INVERT_METHOD_LU);
	}
	assert(sv_is_finite(&iS));
	sv_print_mat(k, "iS", &iS, 1);

	// K = Pk_k1Ht * iS
	svGEMM(&Pk_k1Ht, &iS, 1, 0, 0, K, 0);
	sv_print_mat(k, "K", K, 1);
}

static void survive_kalman_update_covariance(survive_kalman_state_t *k, const survive_kalman_gain_matrix *K,
											 const struct SvMat *H) {
	int dims = k->state_cnt;
	// Apparently cvEye isn't a thing!?
	SV_CREATE_STACK_MAT(eye, dims, dims);
	sv_set_diag_val(&eye, 1);

	SV_CREATE_STACK_MAT(ikh, dims, dims);

	// ikh = (I - K * H)
	svGEMM(K, H, -1, &eye, 1, &ikh, 0);

	// cvGEMM does not like the same addresses for src and destination...
	SvMat *Pk_k = &k->P;
	SV_CREATE_STACK_MAT(tmp, dims, dims);
	svCopy(Pk_k, &tmp, 0);

	// P_k|k = (I - K * H) * P_k|k-1
	svGEMM(&ikh, &tmp, 1, 0, 0, Pk_k, 0);
	assert(sv_is_finite(Pk_k));
	if (k->log_level >= KALMAN_LOG_LEVEL) {
		fprintf(stdout, "INFO gain\t");
		sv_print_mat(k, "K", K, true);

		sv_print_mat(k, "ikh", &ikh, true);

		fprintf(stdout, "INFO new Pk_k\t");
		sv_print_mat(k, "Pk_k", Pk_k, true);
	}
	SV_FREE_STACK_MAT(tmp);
	SV_FREE_STACK_MAT(ikh);
	SV_FREE_STACK_MAT(eye);
}

static inline void survive_kalman_predict(FLT t, survive_kalman_state_t *k, const SvMat *x_t0_t0, SvMat *x_t0_t1) {
	// X_k|k-1 = Predict(X_K-1|k-1)
	if (k->log_level > KALMAN_LOG_LEVEL) {
		fprintf(stdout, "INFO kalman_predict from ");
		sv_print_mat(k, "x_t0_t0", x_t0_t0, false);
	}
	assert(sv_as_const_vector(x_t0_t0) != sv_as_const_vector(x_t0_t1));
	if (t == k->t) {
		svCopy(x_t0_t0, x_t0_t1, 0);
	} else {
		assert(t > k->t);
		k->Predict_fn(t - k->t, k, x_t0_t0, x_t0_t1);
	}
	if (k->log_level > KALMAN_LOG_LEVEL) {
		fprintf(stdout, "INFO kalman_predict to   ");
		sv_print_mat(k, "x_t0_t1", x_t0_t1, false);
	}
	if (k->datalog) {
		SV_CREATE_STACK_MAT(tmp, x_t0_t0->rows, x_t0_t1->cols);
		sv_elementwise_subtract(&tmp, x_t0_t1, x_t0_t0);
		k->datalog(k, "predict_diff", sv_as_const_vector(&tmp), tmp.rows * tmp.cols);
	}
}

static bool numeric_jacobian(survive_kalman_state_t *k, kalman_measurement_model_fn_t Hfn, void *user, const struct SvMat *Z, const struct SvMat *x, SvMat *H) {
    SV_CREATE_STACK_MAT(y1, Z->rows, Z->cols);
    SV_CREATE_STACK_MAT(y2, Z->rows, Z->cols);
    SV_CREATE_STACK_MAT(temp_x, x->rows, x->cols);
    sv_matrix_copy(&temp_x, x);

    FLT* xa = sv_as_vector(&temp_x);
    for(int i = 0;i < k->state_cnt;i++) {
		FLT s = xa[i] == 0 ? 1e-5 : linmath_max(1e-10, fabs(xa[i] * 1e-8));
		xa[i] += s;
        if(!Hfn(user, Z, &temp_x, &y1, 0))
            return false;

        xa[i] -= 2*s;
        if(!Hfn(user, Z, &temp_x, &y2, 0))
            return false;

        for(int j = 0;j < H->rows;j++) {
            svMatrixSet(H, j, i, (_y2[j] - _y1[j])/2./s);
        }
    }

    return true;
}

static SvMat *survive_kalman_find_residual(survive_kalman_meas_model_t *mk, void *user, const struct SvMat *Z,
										   const struct SvMat *x, SvMat *y, SvMat *H) {
	survive_kalman_state_t *k = mk->k;
	kalman_measurement_model_fn_t Hfn = mk->Hfn;

	sv_set_constant(H, INFINITY);

	SvMat *rtn = 0;
	if (Hfn) {
        bool okay = Hfn(user, Z, x, y, H);
		if (okay == false) {
			return 0;
		}
		// k->debug_jacobian = 1;
		if (mk->debug_jacobian) {
			SV_CREATE_STACK_MAT(H_calc, H->rows, H->cols);

			numeric_jacobian(k, Hfn, user, Z, x, &H_calc);
			fprintf(stderr, "FJAC DEBUG BEGIN %s %d\n", mk->name, Z->rows);

			for (int j = 0; j < H->cols; j++) {
				fprintf(stderr, "FJAC PARM %d\n", j);
				for (int i = 0; i < H->rows; i++) {

					FLT deriv_u = svMatrixGet(H, i, j);
					FLT deriv_n = svMatrixGet(&H_calc, i, j);
					FLT diff_abs = fabs(deriv_n - deriv_u);
					FLT diff_rel = diff_abs / (deriv_n + deriv_u);

					if (diff_abs > 1e-2 && diff_rel > 1e-2) {
						fprintf(stderr, "%2d %+7.7f %+7.7f %+7.7f %+7.7f %+7.7f \n", i, sv_as_vector(y)[i], deriv_u,
								deriv_n, diff_abs, diff_rel);
					}
				}
			}
			fprintf(stderr, "FJAC DEBUG END\n");
			sv_matrix_copy(H, &H_calc);
		}
		rtn = H;
	} else {
		rtn = (struct SvMat *)user;
		svGEMM(rtn, x, -1, Z, 1, y, 0);
	}
	assert(sv_is_finite(rtn));

	return rtn;
}
static inline enum survive_kalman_update_extended_termination_reason
survive_kalman_termination_criteria(survive_kalman_state_t *k, const struct term_criteria_t *term_criteria,
									FLT initial_error, FLT error, FLT alpha, FLT last_error) {
	FLT minimum_step = term_criteria->minimum_step > 0 ? term_criteria->minimum_step : .01;
	if (alpha == 0 || alpha < minimum_step) {
		return survive_kalman_update_extended_termination_reason_step;
	}
	if (error == 0) {
		return survive_kalman_update_extended_termination_reason_xtol;
	}

	if (isfinite(last_error) && error < last_error && last_error - error < term_criteria->xtol * error) {
		return survive_kalman_update_extended_termination_reason_xtol;
	}
	return survive_kalman_update_extended_termination_reason_none;
}

SURVIVE_EXPORT FLT calculate_v(const struct SvMat *y, const struct SvMat *xDiff, const struct SvMat *iR,
							   const struct SvMat *iP, FLT *meas_part, FLT *delta_part) {
	if (delta_part == 0) {
		return *meas_part = .5 * mul_at_b_a(y, iR);
	}
	*meas_part = .5 * mul_at_b_a(y, iR);
	*delta_part = .5 * mul_at_b_a(xDiff, iP);
	return .5 * (*meas_part + *delta_part);
}

const char *survive_kalman_update_extended_termination_reason_to_str(
	enum survive_kalman_update_extended_termination_reason reason) {
	switch (reason) {
	case survive_kalman_update_extended_termination_reason_none:
		return "none";
	case survive_kalman_update_extended_termination_reason_maxiter:
		return "maxiter";
	case survive_kalman_update_extended_termination_reason_invalid_jacobian:
		return "invalid_jac";
	case survive_kalman_update_extended_termination_reason_xtol:
		return "xtol";
	case survive_kalman_update_extended_termination_reason_MAX:
		return "MAX";
	case survive_kalman_update_extended_termination_reason_step:
		return "step";
	default:
		return "";
	}
}

// Extended Kalman Filter Modifications Based on an Optimization View Point
// https://www.diva-portal.org/smash/get/diva2:844060/FULLTEXT01.pdf
// Note that in this document, 'y' is the measurement; which we refer to as 'Z'
// throughout this code and so 'y - h(x)' from the paper is 'y' in code.
// The main driver in this document is V(x) which is defined as:
// r(x) = [ R^-.5 * y; P^-.5 * (x_t-1 * x) ]
// V(X) = r'(x) * r(x) / 2
// V(X) = 1/2 * (R^-.5 * y)' * R^-.5 * y + (P^-.5 * (x_t-1 * x))'*P^-.5 * (x_t-1 * x)
// Then owing to the identity (AB)' = B'A', and also to the fact that R and P are symmetric and so R' = R; P' = P:
// V(X) = 1/2 * (y' * (R^-.5)' * R^-.5 * y + (x_t-1 * x)' * (P^-.5)'* P^-.5 * (x_t-1 * x))
// V(X) = 1/2 * (y' * (R^-.5) * R^-.5 * y + (x_t-1 * x)' * (P^-.5)* P^-.5 * (x_t-1 * x))
// V(X) = 1/2 * (y' * (R^-1) * y + (x_t-1 * x)' * (P^-1) * (x_t-1 * x))

// Similarly, we need dV(X)/dX -- ΔV(X) --
// ΔV(X) = -[ R^-.5 * H; P^-.5]' * r(x)
// ΔV(X) = -[ R^-.5 * H; P^-.5]' * [ R^-.5 * y; P^-.5 * (x_t-1 * x) ]
// ΔV(X) =  -(R^-.5 * H) * (R^-.5 * y) - P^-.5 * (P^-.5 * (x_t-1 * x))
// ΔV(X) =  H' * R^-1 * y - P^-1 * (x_t-1 * x)
// The point of all of this is that we don't need to ever explicitly calculate R^-.5 / P^-.5; just the inverses

static FLT survive_kalman_run_iterations(survive_kalman_state_t *k, const struct SvMat *Z, const struct SvMat *R,
										 survive_kalman_meas_model_t *mk, void *user, const SvMat *x_k_k1, SvMat *K,
										 SvMat *H, SvMat *x_k_k, struct survive_kalman_update_extended_stats_t *stats) {
	int state_cnt = k->state_cnt;
	int meas_cnt = Z->rows;

	SV_CREATE_STACK_MAT(y, meas_cnt, 1);
	SV_CREATE_STACK_MAT(x_i, state_cnt, 1);
	SV_CREATE_STACK_MAT(x_i_best, state_cnt, 1);

	SV_CREATE_STACK_MAT(iR, meas_cnt, meas_cnt);
	svInvert(R, &iR, SV_INVERT_METHOD_LU);
	// sv_print_mat_v(k, 100, "iR", &iR, true);

	SV_CREATE_STACK_MAT(iP, state_cnt, state_cnt);
	svInvert(&k->P, &iP, SV_INVERT_METHOD_SVD);
	// sv_print_mat_v(k, 100, "iP", &iP, true);

	assert(sv_is_finite(&iP));
	assert(sv_is_finite(&iR));

	enum survive_kalman_update_extended_termination_reason stop_reason =
		survive_kalman_update_extended_termination_reason_none;
	FLT error = INFINITY, last_error = INFINITY;
	FLT initial_error = 0;

	sv_matrix_copy(&x_i, x_k_k1);
	int iter;
	int max_iter = mk->term_criteria.max_iterations;
	FLT meas_part, delta_part;
	SV_CREATE_STACK_MAT(Hxdiff, Z->rows, 1);
	SV_CREATE_STACK_MAT(x_update, state_cnt, 1);
	SV_CREATE_STACK_MAT(xn, x_i.rows, x_i.cols);
	SV_CREATE_STACK_MAT(x_diff, state_cnt, 1);
	SV_CREATE_STACK_MAT(iRy, meas_cnt, 1);
	SV_CREATE_STACK_MAT(iPdx, state_cnt, 1);
	SV_CREATE_STACK_MAT(dVt, state_cnt, 1);

	for (iter = 0; iter < max_iter; iter++) {
		// Find the residual y and possibly also the jacobian H. The user could have passed one in as 'user', or given
		// us a map function which will calculate it.
		struct SvMat *HR = survive_kalman_find_residual(mk, user, Z, &x_i, &y, H);
		if (stats) {
			stats->fevals++;
			stats->hevals++;
		}

		// If the measurement jacobian isn't calculable, the best we can do is just bail.
		if (HR == 0) {
			stop_reason = survive_kalman_update_extended_termination_reason_invalid_jacobian;
			error = INFINITY;
			break;
		}
		last_error = error;

		svSub(&x_diff, x_k_k1, &x_i);
		error = calculate_v(&y, &x_diff, &iR, &iP, &meas_part, iter == 0 ? 0 : &delta_part);
		assert(error >= 0);

		svGEMM(&iR, &y, 1, 0, 0, &iRy, 0);
		svGEMM(&iP, &x_diff, 1, 0, 0, &iPdx, 0);

		svGEMM(H, &iRy, -1, &iPdx, -1, &dVt, SV_GEMM_FLAG_A_T);

		if (iter == 0) {
			initial_error = error;
			if (stats) {
				stats->orignorm_meas += meas_part;
			}
		}

		// Run update; filling in K
		survive_kalman_find_k(k, K, H, R);

		if ((stop_reason =
				 survive_kalman_termination_criteria(k, &mk->term_criteria, initial_error, error, 1, last_error)) >
			survive_kalman_update_extended_termination_reason_none) {
			goto end_of_loop;
		}

		svGEMM(H, &x_diff, 1, 0, 0, &Hxdiff, 0);

		svSub(&y, &y, &Hxdiff);
		svGEMM(K, &y, 1, &x_diff, 1, &x_update, 0);

		FLT m = dotnd(dVt.data, x_update.data, state_cnt);
		m = linmath_min(-1e-8, m);
		FLT c = .5, tau = .5;
		FLT fa = 0, fa_best = error, a_best = 0;
		FLT scale = 1.;

		FLT min_step = mk->term_criteria.minimum_step == 0 ? .05 : mk->term_criteria.minimum_step;

		bool exit_condition = false;
		while (!exit_condition) {
			exit_condition = true;
			svAddScaled(&xn, &x_update, scale, &x_i, 1);
			mk->Hfn(user, Z, &xn, &y, 0);
			if (stats) {
				stats->fevals++;
			}

			svSub(&x_diff, x_k_k1, &xn);

			fa = calculate_v(&y, &x_diff, &iR, &iP, &meas_part, &delta_part);
			if (k->log_level >= 1000) {
				fprintf(stdout, "%3f: %7.7f ", scale, fa);
				sv_print_mat_v(k, 1000, "at x", &xn, false);
			}
			assert(fa >= 0);

			if (fa >= error + scale * m * c) {
				exit_condition = false;
				if (fa_best > fa) {
					fa_best = fa;
					a_best = scale;
				}
				scale = tau * scale;

				if (scale <= min_step) {
					error = fa_best;
					scale = a_best;
					break;
				}
			}
		}

		if (stats && stats->total_stats) {
			stats->total_stats->step_cnt++;
			stats->total_stats->step_acc += scale;
		}
		svAddScaled(&x_i, &x_i, 1, &x_update, scale);

		if (k->normalize_fn) {
			k->normalize_fn(k->user, &x_i);
		}
		assert(sv_is_finite(&x_i));

	end_of_loop:
		if (k->log_level > 1000) {
			fprintf(stdout, "%3d: %7.7f / %7.7f (%f, %f, %f) ", iter, initial_error, error, scale, m,
					svNorm(&x_update));
			sv_print_mat_v(k, 1000, "new x", &x_i, false);
		}
		if ((stop_reason =
				 survive_kalman_termination_criteria(k, &mk->term_criteria, initial_error, error, scale, last_error)) >
			survive_kalman_update_extended_termination_reason_none) {
			break;
		}
	}
	if (stop_reason == survive_kalman_update_extended_termination_reason_none)
		stop_reason = survive_kalman_update_extended_termination_reason_maxiter;
	bool isFailure = error > initial_error || isinf(error);
	if (stats) {
		stats->iterations = iter;
		stats->orignorm = initial_error;
		stats->bestnorm = error;
		stats->stop_reason = stop_reason;
		stats->bestnorm_meas = meas_part;
		stats->bestnorm_delta = delta_part;
		if (stats->total_stats) {
			stats->total_stats->total_runs++;
			stats->total_stats->total_failures += isFailure;
			stats->total_stats->orignorm_acc += initial_error;
			stats->total_stats->bestnorm_acc += error;
			stats->total_stats->stop_reason_counts[stop_reason]++;
			stats->total_stats->total_fevals += stats->fevals;
			stats->total_stats->total_hevals += stats->hevals;
			stats->total_stats->total_iterations += stats->iterations;
			stats->total_stats->bestnorm_meas_acc += stats->bestnorm_meas;
			stats->total_stats->bestnorm_delta_acc += stats->bestnorm_delta;
			stats->total_stats->orignorm_meas_acc += stats->orignorm_meas;
		}
	}

	if (isFailure) {
		initial_error = -1;
	} else {
		assert(sv_is_finite(H));
		assert(sv_is_finite(K));
		sv_matrix_copy(x_k_k, &x_i);
	}

	SV_FREE_STACK_MAT(Hxdiff);
	SV_FREE_STACK_MAT(x_update);
	SV_FREE_STACK_MAT(xn);
	SV_FREE_STACK_MAT(x_diff);
	SV_FREE_STACK_MAT(iRy);
	SV_FREE_STACK_MAT(iPdx);
	SV_FREE_STACK_MAT(dVt);
	SV_FREE_STACK_MAT(y);
	SV_FREE_STACK_MAT(x_i);
	SV_FREE_STACK_MAT(x_i_best);

	SV_FREE_STACK_MAT(iR);
	SV_FREE_STACK_MAT(iP);

	return initial_error;
}
static FLT survive_kalman_predict_update_state_extended_adaptive_internal(
	FLT t, survive_kalman_state_t *k, void *user, const struct SvMat *Z, FLT *Rv, survive_kalman_meas_model_t *mk,
	struct survive_kalman_update_extended_stats_t *stats) {

	kalman_measurement_model_fn_t Hfn = mk->Hfn;
	bool adaptive = mk->adaptive;

	int state_cnt = k->state_cnt;

	FLT dt = t - k->t;
    FLT result = 0;

    // Setup the R matrix.
    FLT *Rs = adaptive ? Rv : alloca(Z->rows * Z->rows * sizeof(FLT));
    SvMat R = svMat(Z->rows, Z->rows, Rs);
    if (!adaptive) {
        sv_set_diag(&R, Rv);
        if (k->datalog) {
            k->datalog(k, "R", Rv, Z->rows);
        }
    }

    // Anything coming in this soon is liable to spike stuff since dt is so small
    if (dt < 1e-5) {
        dt = 0;
        t = k->t;
    }

    SV_CREATE_STACK_MAT(Pm, state_cnt, state_cnt);
    // Adaptive update happens on the covariance matrix prior; so save it.
	if (adaptive) {
		sv_matrix_copy(&Pm, &k->P);
	}

	SvMat *x_k_k = &k->state;
	SV_CREATE_STACK_MAT(x_k1_k1, state_cnt, 1);
	sv_matrix_copy(&x_k1_k1, x_k_k);

	// Run prediction steps -- gets new state, and covariance matrix based on time delta
	SV_CREATE_STACK_MAT(x_k_k1, state_cnt, 1);

	survive_kalman_predict(t, k, &x_k1_k1, &x_k_k1);
	if (dt > 0) {
        SV_CREATE_STACK_MAT(F, state_cnt, state_cnt);
        sv_set_constant(&F, NAN);

		k->F_fn(dt, &F, &x_k1_k1);
		assert(sv_is_finite(&F));

        // Run predict
		survive_kalman_predict_covariance(dt, &F, &x_k_k1, k);
		SV_FREE_STACK_MAT(F);
	}

	if (k->log_level > KALMAN_LOG_LEVEL) {
		fprintf(stdout, "INFO kalman_predict_update_state_extended t=%f dt=%f ", t, dt);
		sv_print_mat(k, "Z", Z, false);
		fprintf(stdout, "\n");
	}

	SV_CREATE_STACK_MAT(K, state_cnt, Z->rows);
	SV_CREATE_STACK_MAT(HStorage, Z->rows, state_cnt);
	struct SvMat *H = &HStorage;
	if (mk->term_criteria.max_iterations > 1) {
		result = survive_kalman_run_iterations(k, Z, &R, mk, user, &x_k_k1, &K, H, x_k_k, stats);
		if (result < 0)
			return result;
	} else {
		SV_CREATE_STACK_MAT(y, Z->rows, Z->cols);
		H = survive_kalman_find_residual(mk, user, Z, &x_k_k1, &y, H);

		if (H == 0) {
			return -1;
		}

		// Run update; filling in K
		survive_kalman_find_k(k, &K, H, &R);

		// Calculate the next state
		svGEMM(&K, &y, 1, &x_k_k1, 1, x_k_k, 0);
		result = normnd2(_y, y.rows * y.cols);
	}

	assert(sv_is_finite(H));
	assert(sv_is_finite(&K));

	if (k->log_level > KALMAN_LOG_LEVEL) {
		fprintf(stdout, "INFO kalman_update to    ");
		sv_print_mat(k, "x1", x_k_k, false);
	}

	survive_kalman_update_covariance(k, &K, H);

	if (adaptive) {
		// https://arxiv.org/pdf/1702.00884.pdf
		SV_CREATE_STACK_MAT(y, Z->rows, Z->cols);
		SV_CREATE_STACK_MAT(PostHStorage, Z->rows, state_cnt);
		SV_CREATE_STACK_MAT(HPkHt, Z->rows, Z->rows);
		SV_CREATE_STACK_MAT(yyt, Z->rows, Z->rows);

		SvMat *PostH = survive_kalman_find_residual(mk, user, Z, &x_k1_k1, &y, &PostHStorage);
		svMulTransposed(&y, &yyt, false, 0, 1);

		SV_CREATE_STACK_MAT(Pk_k1Ht, state_cnt, H->rows);

		svGEMM(&Pm, PostH, 1, 0, 0, &Pk_k1Ht, SV_GEMM_FLAG_B_T);
		svGEMM(PostH, &Pk_k1Ht, 1, 0, 0, &HPkHt, 0);

		sv_print_mat_v(k, 200, "PostH", PostH, true);
		sv_print_mat_v(k, 200, "PkHt", &Pk_k1Ht, true);
		sv_print_mat_v(k, 200, "HpkHt", &HPkHt, true);
		sv_print_mat_v(k, 200, "yyt", &yyt, true);

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

		sv_print_mat_v(k, 200, "Adaptive R", &R, true);

		SV_FREE_STACK_MAT(Pk_k1Ht);
		SV_FREE_STACK_MAT(yyt);
		SV_FREE_STACK_MAT(HPkHt);
		SV_FREE_STACK_MAT(PostHStorage);
	}

	k->t = t;

	SV_FREE_STACK_MAT(K);
	SV_FREE_STACK_MAT(HStorage);
	SV_FREE_STACK_MAT(x_k_k1);
	SV_FREE_STACK_MAT(Pm);

	return result;
}

FLT survive_kalman_meas_model_predict_update_stats(FLT t, struct survive_kalman_meas_model *mk, void *user,
												   const struct SvMat *Z, const FLT *R,
												   struct survive_kalman_update_extended_stats_t *stats) {
	return survive_kalman_predict_update_state_extended_adaptive_internal(t, mk->k, user, Z, (FLT *)R, mk, stats);
}

FLT survive_kalman_meas_model_predict_update(FLT t, struct survive_kalman_meas_model *mk, void *user,
											 const struct SvMat *Z, const FLT *R) {
	struct survive_kalman_update_extended_stats_t stats = {.total_stats = &mk->stats};
	return survive_kalman_predict_update_state_extended_adaptive_internal(t, mk->k, user, Z, (FLT *)R, mk, &stats);
}

/*
FLT survive_kalman_predict_update_state_extended(FLT t, survive_kalman_state_t *k, const struct SvMat *Z, const FLT *R,
												 const survive_kalman_update_extended_params_t *extended_params,
												 struct survive_kalman_update_extended_stats_t *stats) {
	return survive_kalman_predict_update_state_extended_adaptive_internal(t, k, Z, (FLT *)R, extended_params, stats);
}

FLT survive_kalman_predict_update_state(FLT t, survive_kalman_state_t *k, const struct SvMat *Z, const struct SvMat *H,
										const FLT *R, bool adaptive) {
	survive_kalman_update_extended_params_t params = {.user = (void *)H, .adapative = adaptive};
	return survive_kalman_predict_update_state_extended(t, k, Z, R, &params, 0);
}
*/
FLT survive_kalman_predict_update_state(FLT t, survive_kalman_state_t *k, const struct SvMat *Z, const struct SvMat *H,
										const FLT *R, bool adaptive) {
	survive_kalman_meas_model_t mk = {.adaptive = adaptive, .k = k};
	return survive_kalman_meas_model_predict_update(t, &mk, (void *)H, Z, R);
}

void survive_kalman_predict_state(FLT t, const survive_kalman_state_t *k, size_t start_index, size_t end_index,
								  FLT *_out) {
	SV_CREATE_STACK_MAT(tmpOut, k->state_cnt, 1);
	const SvMat *x = &k->state;

	FLT dt = t == 0. ? 0 : t - k->t;
	const FLT *copyFrom = sv_as_const_vector(&k->state);
	if (dt > 0) {
		k->Predict_fn(dt, k, x, &tmpOut);
		copyFrom = _tmpOut;
	}
	assert(_out != copyFrom);
	memcpy(_out, copyFrom + start_index, (end_index - start_index) * sizeof(FLT));
	SV_FREE_STACK_MAT(tmpOut);
}
void survive_kalman_set_P(survive_kalman_state_t *k, const FLT *p) { sv_set_diag(&k->P, p); }
