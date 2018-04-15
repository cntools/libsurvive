#ifndef _fileutil_h_
#define _fileutil_h_

#include <pthread.h>
#include "os_generic.h"

void LoadLighthousePos(
	const char *path, 
	float *x, float *y, float *z,
	float *qi, float *qj, float *qk, float *qreal);


// first 32 are hmd, next 24 wm0 next 24 wm1
#define NUM_HMD   80
#define NUM_SWEEP 4
#define SWEEP_LX  0
#define SWEEP_LY  1
#define SWEEP_RX  2
#define SWEEP_RY  3
void LoadHmdProcessedDataAngles(
	const char *path,
	double angle[NUM_SWEEP][NUM_HMD]);


extern og_mutex_t  read_mutex;
extern og_thread_t read_thread;
extern double read_hmdAngles[NUM_SWEEP][NUM_HMD];
extern int read_hmdAngleViewed[NUM_SWEEP][NUM_HMD];
extern int read_frameno;
void *ThreadReadHmtAngles(void *junk);


#endif // __fileutil_h_


