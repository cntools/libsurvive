#ifndef _SURVIVE_KALMAN_H
#define _SURVIVE_KALMAN_H

#include "survive.h"

typedef void (*F_fn_t)(FLT t, FLT *f_out);

typedef struct {
	int dims;
	F_fn_t F_fn;
	const FLT *Q_per_sec;

	bool P_is_heap;
	FLT *P;
} survive_kalman_t;

typedef struct {
	size_t state_size;
	survive_kalman_t info;

	bool State_is_heap;
	FLT *state;
} survive_kalman_state_t;

void survive_kalman_predict(FLT t, survive_kalman_t *k);
void survive_kalman_update(FLT t, survive_kalman_t *k, FLT *_K, const FLT *_H, FLT _R);
void survive_kalman_predict_update(FLT t, survive_kalman_t *k, FLT *K, const FLT *H, FLT R);

void survive_kalman_predict_update_state(FLT t, survive_kalman_state_t *k, const FLT *z, const FLT *H, FLT R);

void survive_kalman_init(survive_kalman_t *k, size_t dims, F_fn_t F, const FLT *Q_per_sec, FLT *P);
void survive_kalman_state_init(survive_kalman_state_t *k, size_t dims, F_fn_t F, const FLT *Q_per_sec, FLT *P,
							   size_t state_size, FLT *state);
void survive_kalman_get_state(FLT t, const survive_kalman_state_t *k, size_t index, FLT *out);
#endif