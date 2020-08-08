#ifndef _SURVIVE_KALMAN_H
#define _SURVIVE_KALMAN_H

#include "survive.h"

struct survive_kalman_state_s;
struct CvMat;

typedef void (*F_fn_t)(FLT t, FLT *f_out, const struct CvMat *x0);
typedef void (*Predict_fn_t)(FLT t, const struct survive_kalman_state_s *k, const struct CvMat *x0, struct CvMat *x1);
typedef void (*Update_fn_t)(FLT t, struct survive_kalman_state_s *k, const struct CvMat *H, const struct CvMat *K,
							const struct CvMat *x_t0, struct CvMat *x_t1, const FLT *z);
typedef void (*Map_to_obs)(void *user, FLT t, const struct CvMat *Z, const struct CvMat *x_t, struct CvMat *y,
						   struct CvMat *H_k);

/**
 * https://en.wikipedia.org/wiki/Kalman_filter#Underlying_dynamical_system_model
 *
 * The kalman filter is organized into two logical parts -- one which just propagates and stores the covariance matrix
 * and another which actually manipulates state.
 *
 */
typedef struct survive_kalman_s {
	// The number of states stored. For instance, something that tracked position and velocity would have 6 states --
	// [x, y, z, vx, vy, vz]
	int state_cnt;

	// f(x_k-1|k-1, uk) so that X_k|k-1 = f(x_k-1|k-1, uk)
	Predict_fn_t Predict_fn;

	// F is assumed to be time varying; so instead of having F we have a function ptr which generates the transition
	// matrix of size state_cnt x state_cnt.
	// Should be df/dx. In the linear case, this function isn't dependent on x.
	F_fn_t F_fn;

	// Added covariance per sec is time varying; but is a constant matrix that is multiplied by delta T. Process noise
	// for these models will always have a time component essentially.
	const FLT *Q_per_sec;

	// Store the actual P matrix (state_cnt x state_cnt). A pointer can be passed in as storage; otherwise its on the
	// heap.
	bool P_is_heap;
	FLT *P;
} survive_kalman_t;

typedef struct survive_kalman_state_s {
	survive_kalman_t info;

	// Actual state matrix and whether its stored on the heap. Make no assumptions about how this matrix is organized.
	// it is always size of dimension_cnt*state_cnt*sizeof(FLT) though.
	bool State_is_heap;
	FLT *state;

	// Current time
	FLT t;
} survive_kalman_state_t;

typedef struct survive_kalman_meas_def {
	survive_kalman_state_t state;

	Map_to_obs map;

} survive_kalman_measurment_setup_s;

typedef struct CvMat survive_kalman_measurement_matrix;
typedef struct CvMat survive_kalman_gain_matrix;

// Given a measurement an H array (size of 1xstate_cnt), and an observation variance R, propogate the variance matrix.
// This also generates the gain matrix K.
/**
 * Run 'predict' and 'update' phase
 * @param t delta time
 * @param k kalman info
 * @param K Output kalman gain matrix
 * @param F State transition matrix
 * @param H Input observation model -- maps measurement to state space
 * @param R Observation noise
 */
SURVIVE_EXPORT void survive_kalman_predict_update_covariance(FLT t, survive_kalman_t *k, survive_kalman_gain_matrix *K,
															 const struct CvMat *F,
															 const survive_kalman_measurement_matrix *H, const FLT *R);

// Given a measurement array z (size of 1xdimension_cnt), an H array (size of 1xstate_cnt), and an observation variance
// R, propogate the entire thing -- variance matrix and state
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
 * Run predict and update, also updating the state matrix.
 * @param t delta time
 * @param k kalman state info
 * @param z measurement -- array of dimension_cnt size
 * @param H Input observation model
 * @param R Observation noise
 */
SURVIVE_EXPORT void survive_kalman_predict_update_state(FLT t, survive_kalman_state_t *k, const struct CvMat *Z,
														const survive_kalman_measurement_matrix *H, const FLT *R);
SURVIVE_EXPORT void survive_kalman_predict_update_state_extended(FLT t, survive_kalman_state_t *k,
																 const struct CvMat *Z, const FLT *R, Map_to_obs Hfn,
																 void *user);

SURVIVE_EXPORT void survive_kalman_init(survive_kalman_t *k, size_t state_cnt, F_fn_t F, const FLT *Q_per_sec, FLT *P);
SURVIVE_EXPORT void survive_kalman_free(survive_kalman_t *k);
SURVIVE_EXPORT void survive_kalman_state_init(survive_kalman_state_t *k, size_t state_cnt, F_fn_t F,
											  const FLT *Q_per_sec, FLT *P, FLT *state);
SURVIVE_EXPORT void survive_kalman_state_free(survive_kalman_state_t *k);
SURVIVE_EXPORT void survive_kalman_set_P(survive_kalman_state_t *k, const FLT *d);
SURVIVE_EXPORT void survive_kalman_set_logging_level(int verbosity);
#endif
