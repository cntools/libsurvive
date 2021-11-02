#include "../src/survive_kalman.h"
#include "test_case.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sv_matrix.h>

// Generates the transition matrix F
static void transition(FLT dt, struct SvMat *f_out, const struct SvMat *x0) {
	sv_set_zero(f_out);
	sv_set_diag_val(f_out, 1);
}

static void process_noise(void *user, FLT dt, const struct SvMat *x, struct SvMat *Q_out) {
	sv_set_zero(Q_out);
	sv_set_diag_val(Q_out, .1);
}

static FLT measurement_model(const LinmathPoint2d sensor_pos, const LinmathPoint2d state) {
	return atan2(state[1] - sensor_pos[1], state[0] - sensor_pos[0]);
	// return atan2(state[0] - sensor_pos[0], state[1] - sensor_pos[1]);
}

static bool toy_measurement_model(void *user, const struct SvMat *Z, const struct SvMat *x_t, struct SvMat *y,
								  struct SvMat *H_k) {
	const LinmathPoint2d *sensors = user;

	for (int i = 0; i < Z->rows; i++) {
		if (y) {
			y->data[i] = Z->data[i] - measurement_model(sensors[i], x_t->data);
		}
		if (H_k) {
			FLT dx = x_t->data[0] - sensors[i][0];
			FLT dy = x_t->data[1] - sensors[i][1];
			FLT scale = 1. / ((dx * dx) + (dy * dy));
			svMatrixSet(H_k, i, 0, scale * -dy);
			svMatrixSet(H_k, i, 1, scale * dx);
		}
	}

	return true;
}

void run_standard_experiment(LinmathPoint2d X_out, FLT *P, const survive_kalman_update_extended_params_t *params_in,
							 int time_steps) {
	LinmathPoint2d true_state = {1.5, 1.5};
	FLT X[2] = {0.5, 0.1};
	LinmathPoint2d sensors[2] = {{0, 0}, {1.5, 0}};
	survive_kalman_state_t state = {};
	survive_kalman_state_init(&state, 2, transition, process_noise, 0, X);
	//    state.debug_jacobian = true;
	state.log_level = 101;
	sv_set_diag_val(&state.P, .1);

	SV_CREATE_STACK_MAT(Z, 2, 1);
	FLT Rv = LINMATHPI * LINMATHPI * 1e-5;
	FLT R[] = {Rv, Rv};

	for (int i = 0; i < 2; i++) {
		Z.data[i] = measurement_model(sensors[i], true_state);
	}

	survive_kalman_update_extended_params_t params = *params_in;
	params.Hfn = toy_measurement_model;
	params.user = sensors;

	for (int i = 0; i < time_steps; i++) {
		survive_kalman_predict_update_state_extended(1, &state, &Z, R, &params);
		printf("%3d: %7.6f %7.6f\n", i, X[0], X[1]);
	}

	memcpy(P, state.P.data, sizeof(FLT) * 4);
	memcpy(X_out, X, sizeof(FLT) * 2);
	survive_kalman_state_free(&state);
}

// https://www.diva-portal.org/smash/get/diva2:844060/FULLTEXT01.pdf

TEST(Kalman, EKFTest) {
	/**
	 * These values are not the ones shown in the chart in the paper, but it's unclear if that chart is with noise or
	 * without noise in the measurement. Results reproduced in octave:

	  x0 = [.5, .1]
	  P0 = [.1 0; 0 .1]
	  Q = [ .1 0; 0 .1]
	  x_t = [1.5 1.5]
	  S0 = [ 0, 0]
	  S1 = [ 1.5, 0]
	  y0 = atan2(x_t(2) - S0(2), x_t(1) - S0(1))
	  y1 = atan2(x_t(2) - S1(2), x_t(1) - S1(1))
	  y = [y0; y1]
	  H0 = [-(x0(2) - S0(2)) (x0(1) - S0(1))] / ( (x0(2) - S0(2))^2 + (x0(1) - S0(1))^2)
	  H1 = [-(x0(2) - S1(2)) (x0(1) - S1(1))] / ( (x0(2) - S1(2))^2 + (x0(1) - S1(1))^2)
	  H = [H0; H1]
	  h0 = atan2(x0(2) - S0(2), x0(1) - S0(1))
	  h1 = atan2(x0(2) - S1(2), x0(1) - S1(1))
	  hx = [h0;h1]
	  rv = pi*pi * 10^-5
	  R = [rv 0; 0 rv]
	  K0 = P0_1 * H' * (H * P0_1 * H' + R)^-1

	  x1 = x0' + K0 * (y - hx)
	  P1 = (eye(2) - K0 * H) * P0_1

	 */
	FLT expected_X[2] = {4.4049048331227372, 1.1884307714294169};
	FLT expected_P[4] = {0.0014050624776344668, 0.00019267084936229752, 0.0001926708493623336, 4.751355804986091e-05};

	FLT X[2];
	FLT P[4];
	survive_kalman_update_extended_params_t params = {};
	run_standard_experiment(X, P, &params, 1);

	LinmathPoint2d true_state = {1.5, 1.5};
	FLT error = distnd(X, true_state, 2);

	ASSERT_DOUBLE_ARRAY_EQ(2, X, expected_X);
	ASSERT_DOUBLE_ARRAY_EQ(4, P, expected_P);

	return 0;
}

TEST(Kalman, IEKFTest) {
	FLT X[2];
	FLT P[4];
	survive_kalman_update_extended_params_t params = {.term_criteria = {.max_iterations = 10, .error_tol = INFINITY}};
	run_standard_experiment(X, P, &params, 1);

	LinmathPoint2d true_state = {1.5, 1.5};
	FLT error = distnd(X, true_state, 2);
	ASSERT_GT(.17, error);

	return 0;
}
