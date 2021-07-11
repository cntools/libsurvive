#ifndef _SURVIVE_KALMAN_H
#define _SURVIVE_KALMAN_H

#include "survive.h"
#include "sv_matrix.h"

/**
 * This file contains a generic kalman implementation. survive_kalman_tracker.h/c fills in the actual lighthouse
 * data model.
 *
 * This implementation tries to use the same nomenclature as:
 * https://en.wikipedia.org/wiki/Kalman_filter#Underlying_dynamical_system_model and
 * https://en.wikipedia.org/wiki/Extended_Kalman_filter.
 *
 * This implementation supports both nonlinear prediction models and nonlinear measurement models. Each phase
 * incorporates a time delta to approximate a continous model.
 *
 * Adaptive functionality:
 *
 * https://arxiv.org/pdf/1702.00884.pdf
 *
 * The R matrix should be initialized to reasonable values on the first all and then is updated based on the residual
 * error -- higher error generates higher variance values:
 *
 * R_k = a * R_k-1 + (1 - a) * (e*e^t + H * P_k-1 * H^t)
 *
 * a is set to .3 for this implementation.

 */

struct survive_kalman_state_s;

// Generates the transition matrix F
typedef void (*kalman_transition_fn_t)(FLT dt, struct SvMat *f_out, const struct SvMat *x0);

// Given state x0 and time delta; gives the new state x1. For a linear model, this is just x1 = F * x0
typedef void (*kalman_predict_fn_t)(FLT dt, const struct survive_kalman_state_s *k, const struct SvMat *x0,
									struct SvMat *x1);

// Given time and current state, generate the process noise Q_k.
typedef void (*kalman_process_noise_fn_t)(void *user, FLT dt, const struct SvMat *x, struct SvMat *Q_out);

// Given a measurement Z, and state X_t, generates both the y difference term and the H jacobian term.
typedef bool (*kalman_measurement_model_fn_t)(void *user, const struct SvMat *Z, const struct SvMat *x_t,
											  struct SvMat *y, struct SvMat *H_k);

typedef struct survive_kalman_state_s {
	// The number of states stored. For instance, something that tracked position and velocity would have 6 states --
	// [x, y, z, vx, vy, vz]
	int state_cnt;

	void *user;

	kalman_predict_fn_t Predict_fn;
	kalman_transition_fn_t F_fn;
	kalman_process_noise_fn_t Q_fn;

	// Store the current covariance matrix (state_cnt x state_cnt)
	struct SvMat P;

	// Actual state matrix and whether its stored on the heap. Make no assumptions about how this matrix is organized.
	// it is always size of state_cnt*sizeof(FLT) though.
	bool State_is_heap;
	struct SvMat state;

	// Current time
	FLT t;

	void *datalog_user;
	void (*datalog)(struct survive_kalman_state_s *state, const char *name, const FLT *v, size_t length);
} survive_kalman_state_t;

/**
 * Predict the state at a given delta; doesn't update the covariance matrix
 * @param t delta time
 * @param k kalman state info
 * @param index Which state vector to pull out
 * @param out Pre allocated output buffer.
 */
SURVIVE_EXPORT void survive_kalman_predict_state(FLT t, const survive_kalman_state_t *k, size_t start_index,
												 size_t end_index, FLT *out);

/**
 * Run predict and update, updating the state matrix. This is for purely linear measurement models.
 *
 * @param t absolute time
 * @param k kalman state info
 * @param z measurement -- SvMat of n x 1
 * @param H Input observation model -- SvMat of n x state_cnt
 * @param R Observation noise -- The diagonal of the measurement covariance matrix; length n
 * @param adapative Whether or not R is an adaptive matrix. When true, R should be a full n x n matrix.
 *
 */
SURVIVE_EXPORT FLT survive_kalman_predict_update_state(FLT t, survive_kalman_state_t *k, const struct SvMat *Z,
													   const struct SvMat *H, const FLT *R, bool adaptive);

/**
 * Run predict and update, updating the state matrix. This is for non-linear measurement models.
 *
 * @param t absolute time
 * @param k kalman state info
 * @param z measurement -- SvMat of n x 1
 * @param R Observation noise -- The diagonal of the measurement covariance matrix; length n
 * @param H Input observation model -- SvMat of n x state_cnt
 * @param Hfn Observation function that gives both the residual vector and the jacobian associated with it.
 * @param adapative Whether or not R is an adaptive matrix. When true, R should be a full n x n matrix.
 *
 * @returns Returns the average residual error
 */
SURVIVE_EXPORT FLT survive_kalman_predict_update_state_extended(FLT t, survive_kalman_state_t *k, const struct SvMat *Z,
																const FLT *R, kalman_measurement_model_fn_t Hfn,
																void *user, bool adapative);

/**
 * Initialize a kalman state object
 * @param k object to initialize
 * @param state_cnt Length of state vector
 * @param F Transition function
 * @param q_fn Noise function
 * @param user pointer to give to user functions
 * @param state Optional state. Pass 0 to malloc one. Otherwise should point to a vector of at least state_cnt FLTs.
 *
 * @returns Returns the average residual error
 */
SURVIVE_EXPORT void survive_kalman_state_init(survive_kalman_state_t *k, size_t state_cnt, kalman_transition_fn_t F,
											  kalman_process_noise_fn_t q_fn, void *user, FLT *state);

SURVIVE_EXPORT void survive_kalman_state_reset(survive_kalman_state_t *k);

SURVIVE_EXPORT void survive_kalman_state_free(survive_kalman_state_t *k);
SURVIVE_EXPORT void survive_kalman_set_P(survive_kalman_state_t *k, const FLT *d);
SURVIVE_EXPORT void survive_kalman_set_logging_level(int verbosity);
#endif
