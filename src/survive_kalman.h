#ifndef _SURVIVE_KALMAN_H
#define _SURVIVE_KALMAN_H

#include "survive.h"

struct survive_kalman_state_s;
struct CvMat;

typedef void (*F_fn_t)(FLT t, FLT *f_out);
typedef void (*Predict_fn_t)(FLT t, const struct survive_kalman_state_s *k, const struct CvMat *x0, struct CvMat *x1);
typedef void (*Update_fn_t)(FLT t, struct survive_kalman_state_s *k, const struct CvMat *H, const struct CvMat *K,
							const struct CvMat *x_t0, struct CvMat *x_t1, const FLT *z);
typedef void (*Map_to_obs)(FLT *z_out, const FLT *f_in);

/**
 * https://en.wikipedia.org/wiki/Kalman_filter#Underlying_dynamical_system_model
 *
 * The kalman filter is organized into two logical parts -- one which just propagates and stores the covariance matrix
 * and another which actually manipulates state.
 *
 * This is done to introduce a simplifying assumption -- our state space has multi-dimensional vectors; but each
 * dimension is independent and all dimensions have the same variance against other states.
 *
 * As an example, to track angular position in 3 space we might have a state space of [ angular_pos, angular_vel ],
 * each of which is a 3 vector -- XYZ. Any measurement we take is going to have the same variance for XYZ, so all
 * of XYZ can be represented by the same covariance matrix. This means instead of doing calculations on 6x6 covariance
 * matrix, its one 3x3.
 */
typedef struct survive_kalman_s {
	// The number of states stored. For instance, something that tracked position and velocity would have 2 states.
	int state_cnt;

	// F is assumed to be time varying; so instead of having F we have a function ptr which generates the transition
	// matrix of size state_cnt x state_cnt.
	F_fn_t F_fn;

	Predict_fn_t Predict_fn;
	Map_to_obs Map_fn;

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

	// Number of dimensions each state has. In theory this doesn't need to be the same for all states; but this
	// implementation assumes it.
	int dimension_cnt[4];
	int max_dim_cnt;

	// Actual state matrix and whether its stored on the heap. Make no assumptions about how this matrix is organized.
	// it is always size of dimension_cnt*state_cnt*sizeof(FLT) though.
	bool State_is_heap;
	FLT *state;
} survive_kalman_state_t;

// Given a measurement an H array (size of 1xstate_cnt), and an observation variance R, propogate the variance matrix.
// This also generates the gain matrix K.
/**
 * Run 'predict' and 'update' phase
 * @param t delta time
 * @param k kalman info
 * @param K Output kalman gain matrix
 * @param H Input observation model -- maps measurement to state space
 * @param R Observation noise
 */
void survive_kalman_predict_update_covariance(FLT t, survive_kalman_t *k, FLT *K, const FLT *H, FLT R);

// Given a measurement array z (size of 1xdimension_cnt), an H array (size of 1xstate_cnt), and an observation variance
// R, propogate the entire thing -- variance matrix and state
/**
 * Predict the state at a given delta; doesn't update the covariance matrix
 * @param t delta time
 * @param k kalman state info
 * @param index Which state vector to pull out
 * @param out Pre allocated output buffer.
 */
void survive_kalman_predict_state(FLT t, const survive_kalman_state_t *k, size_t index, FLT *out);

/**
 * Run predict and update, also updating the state matrix.
 * @param t delta time
 * @param k kalman state info
 * @param z measurement -- array of dimension_cnt size
 * @param H Input observation model
 * @param R Observation noise
 */
void survive_kalman_predict_update_state(FLT t, survive_kalman_state_t *k, const FLT *z, const FLT *H, FLT R);
void survive_kalman_predict_update_state_extended(FLT t, survive_kalman_state_t *k, const FLT *z, const FLT *H,
												  Update_fn_t updateFn, FLT R);

void survive_kalman_init(survive_kalman_t *k, size_t state_cnt, F_fn_t F, const FLT *Q_per_sec, FLT *P);
void survive_kalman_free(survive_kalman_t *k);
void survive_kalman_state_init(survive_kalman_state_t *k, size_t state_cnt, F_fn_t F, const FLT *Q_per_sec, FLT *P,
							   size_t *dims, FLT *state);
void survive_kalman_state_free(survive_kalman_state_t *k);

#endif