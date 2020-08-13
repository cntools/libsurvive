#include "linmath.h"

struct variance_measure {
	size_t size;
	FLT sum[16];
	FLT sumSq[16];
	size_t n;
};

static inline void variance_measure_add(struct variance_measure *meas, const FLT *d) {
	if (meas->size == 0)
		meas->size = 1;

	meas->n++;
	addnd(meas->sum, meas->sum, d, meas->size);
	for (int i = 0; i < meas->size; i++) {
		assert(isfinite(d[i]));
		meas->sumSq[i] += d[i] * d[i];
	}
}

static inline void variance_measure_reset(struct variance_measure *meas) {
	meas->n = 0;
	memset(meas->sum, 0, sizeof(FLT) * meas->size);
	memset(meas->sumSq, 0, sizeof(FLT) * meas->size);
}
static inline void variance_measure_calc(struct variance_measure *meas, FLT *d) {
	if (meas->n == 0) {
		memset(d, 0, meas->n * sizeof(FLT));
		return;
	}

	for (int i = 0; i < meas->size; i++) {
		d[i] = (meas->sumSq[i] - (meas->sum[i] * meas->sum[i]) / meas->n) / meas->n;
	}
}