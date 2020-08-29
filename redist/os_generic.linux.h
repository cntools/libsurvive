#ifndef OS_GENERIC_DONT_INCLUDE_DIRECTLY
#error Do not include this file directly
#endif

#include <pthread.h>
#include <semaphore.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>

OSG_INLINE void OGSleep(int is) { sleep(is); }

OSG_INLINE int OGUSleep(int ius) {
	struct timespec sleep = {0};
	sleep.tv_nsec = (ius % 1000000) * 1000;
	sleep.tv_sec = ius / 1000000;
	return nanosleep(&sleep, 0);
}

OSG_INLINE uint64_t OGGetAbsoluteTimeMS() {
	struct timeval tv;

	gettimeofday(&tv, NULL);
	return (((uint64_t)tv.tv_sec) * 1000) + (tv.tv_usec / 1000);
}

OSG_INLINE double OGGetAbsoluteTime() {
	struct timeval tv;
	gettimeofday(&tv, 0);
	return ((double)tv.tv_usec) / 1000000. + (tv.tv_sec);
}

OSG_INLINE double OGGetFileTime(const char *file) {
	struct stat buff;

	int r = stat(file, &buff);

	if (r < 0) {
		return -1;
	}

	return buff.st_mtime;
}

OSG_INLINE og_thread_t OGCreateThread(void *(routine)(void *), const char *name, void *parameter) {
	pthread_t *ret = (pthread_t *)malloc(sizeof(pthread_t));
	int r = pthread_create(ret, 0, routine, parameter);
	if (r) {
		free(ret);
		return 0;
	}
#ifdef _GNU_SOURCE
	pthread_setname_np(*ret, name);
#endif
	return (og_thread_t)ret;
}

OSG_INLINE void *OGJoinThread(og_thread_t ot) {
	void *retval;
	if (!ot) {
		return 0;
	}
	pthread_join(*(pthread_t *)ot, &retval);
	free(ot);
	return retval;
}

OSG_INLINE void OGCancelThread(og_thread_t ot) {
#ifndef ANDROID
	if (!ot) {
		return;
	}
	pthread_cancel(*(pthread_t *)ot);
	free(ot);
#endif
}

static inline void _OGHandlePosixError(const char *msg, int err) {
	if (err != 0) {
		fprintf(stderr, "%s: %s (%d)\n", msg, strerror(err), err);
		abort();
	}
}

OSG_INLINE og_mutex_t OGCreateMutex() {
	pthread_mutexattr_t mta;
	og_mutex_t r = malloc(sizeof(pthread_mutex_t));

	pthread_mutexattr_init(&mta);
	pthread_mutexattr_settype(&mta, PTHREAD_MUTEX_RECURSIVE);

	_OGHandlePosixError("OGCreateMutex", pthread_mutex_init((pthread_mutex_t *)r, &mta));
	return r;
}

OSG_INLINE void OGLockMutex(og_mutex_t om) {
	if (!om) {
		return;
	}
	_OGHandlePosixError("OGLockMutex", pthread_mutex_lock((pthread_mutex_t *)om));
}

OSG_INLINE void OGUnlockMutex(og_mutex_t om) {
	if (!om) {
		return;
	}
	_OGHandlePosixError("OGUnlockMutex", pthread_mutex_unlock((pthread_mutex_t *)om));
}

OSG_INLINE void OGDeleteMutex(og_mutex_t om) {
	if (!om) {
		return;
	}

	pthread_mutex_destroy((pthread_mutex_t *)om);
	free(om);
}

OSG_INLINE og_sema_t OGCreateSema() {
	sem_t *sem = (sem_t *)malloc(sizeof(sem_t));
	sem_init(sem, 0, 0);
	return (og_sema_t)sem;
}

OSG_INLINE int OGGetSema(og_sema_t os) {
	int valp;
	sem_getvalue((sem_t *)os, &valp);
	return valp;
}

OSG_INLINE void OGLockSema(og_sema_t os) { sem_wait((sem_t *)os); }

OSG_INLINE void OGUnlockSema(og_sema_t os) { sem_post((sem_t *)os); }

OSG_INLINE void OGDeleteSema(og_sema_t os) {
	sem_destroy((sem_t *)os);
	free(os);
}

OSG_INLINE void OGSignalCond(og_cv_t cv) {
	_OGHandlePosixError("OGSignalCond", pthread_cond_signal((pthread_cond_t *)cv));
}
OSG_INLINE void OGBroadcastCond(og_cv_t cv) {
	_OGHandlePosixError("OGBroadcastCond", pthread_cond_broadcast((pthread_cond_t *)cv));
}
OSG_INLINE void OGWaitCond(og_cv_t cv, og_mutex_t m) {
	_OGHandlePosixError("OGWaitCond", pthread_cond_wait((pthread_cond_t *)cv, (pthread_mutex_t *)m));
}

OSG_INLINE void OGDeleteConditionVariable(og_cv_t cv) {
	pthread_cond_destroy((pthread_cond_t *)cv);
	free(cv);
}

OSG_INLINE og_cv_t OGCreateConditionVariable() {
	pthread_cond_t *cv = (pthread_cond_t *)malloc(sizeof(pthread_cond_t));
	int status = pthread_cond_init(cv, 0);
	if (status != 0) {
		free(cv);
		return 0;
	}

	return cv;
}

