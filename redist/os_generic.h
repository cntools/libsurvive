#pragma once
/*
	"osgeneric" Generic, platform independent tool for the following operations:

	Delay functions:
		void OGSleep( int is );
		void OGUSleep( int ius );

	Getting current time (may be time from program start, boot, or epoc)
		double OGGetAbsoluteTime();
		double OGGetFileTime( const char * file );

	Thread functions
		og_thread_t OGCreateThread( void * (routine)( void * ), const char* name, void * parameter );
		void * OGJoinThread( og_thread_t ot );
		void OGCancelThread( og_thread_t ot );

	Mutex functions, used for protecting data structures.
		 (recursive on platforms where available.)
		og_mutex_t OGCreateMutex();
		void OGLockMutex( og_mutex_t om );
		void OGUnlockMutex( og_mutex_t om );
		void OGDeleteMutex( og_mutex_t om );

//Always a semaphore (not recursive)
// og_sema_t OGCreateSema(); //Create a semaphore, comes locked initially.  NOTE: Max count is 32767
//  void OGLockSema( og_sema_t os );
//  int OGGetSema( og_sema_t os );  //if <0 there was a failure.
//  void OGUnlockSema( og_sema_t os );
//  void OGDeleteSema( og_sema_t os );



   Copyright (c) 2011-2012,2013,2016,2018 <>< Charles Lohr
	This file may be licensed under the MIT/x11 license or the NewBSD license.

   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of this file.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
   FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
   IN THE SOFTWARE.

	Date Stamp: 2018-03-25: Switched to header-only format.
*/

#define OSG_INLINE static inline

// Threads and Mutices
typedef void *og_thread_t;
typedef void *og_mutex_t;
typedef void *og_sema_t;
typedef void *og_cv_t;

#include "stdbool.h"
#include "stdint.h"
#include "stdlib.h"

#ifdef __cplusplus
extern "C" {
#endif

OSG_INLINE void OGSleep(int is);
OSG_INLINE int OGUSleep(int ius);
OSG_INLINE double OGGetAbsoluteTime();

static inline double OGStartTimeS() {
	static double start_time_s = 0;
	if (start_time_s == 0.)
		start_time_s = OGGetAbsoluteTime();
	return start_time_s;
}

static inline double OGRelativeTime() { return OGGetAbsoluteTime() - OGStartTimeS(); }

OSG_INLINE uint64_t OGGetAbsoluteTimeUS();
OSG_INLINE uint64_t OGGetAbsoluteTimeMS() { return (OGGetAbsoluteTimeUS() + 999) / 1000; }

OSG_INLINE og_thread_t OGCreateThread(void *(routine)(void *), const char *name, void *parameter);

OSG_INLINE void *OGJoinThread(og_thread_t ot);

OSG_INLINE void OGCancelThread(og_thread_t ot);

OSG_INLINE og_mutex_t OGCreateMutex();

OSG_INLINE void OGLockMutex(og_mutex_t om);

OSG_INLINE void OGUnlockMutex(og_mutex_t om);

OSG_INLINE void OGDeleteMutex(og_mutex_t om);

OSG_INLINE og_sema_t OGCreateSema();

OSG_INLINE void OGLockSema(og_sema_t os);

OSG_INLINE void OGUnlockSema(og_sema_t os);

OSG_INLINE void OGDeleteSema(og_sema_t os);

OSG_INLINE void OGSignalCond(og_cv_t cv);
OSG_INLINE void OGBroadcastCond(og_cv_t cv);
OSG_INLINE void OGWaitCond(og_cv_t cv, og_mutex_t m);

OSG_INLINE void OGDeleteConditionVariable(og_cv_t cv);

OSG_INLINE og_cv_t OGCreateConditionVariable();  

#if defined(WIN32) || defined(WINDOWS) || defined(_WIN32)
#define USE_WINDOWS
#endif

#define OS_GENERIC_DONT_INCLUDE_DIRECTLY
#ifdef USE_WINDOWS
#include "os_generic.windows.h"
#else
#include "os_generic.unix.h"
#endif

#undef OS_GENERIC_DONT_INCLUDE_DIRECTLY
  
#ifdef __cplusplus
}
#endif
