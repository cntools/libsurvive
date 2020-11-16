#ifndef OS_GENERIC_DONT_INCLUDE_DIRECTLY
#error Do not include this file directly
#endif

#include <stdint.h>
#include <windows.h>

OSG_INLINE void OGSleep(int is) { Sleep(is * 1000); }

OSG_INLINE int OGUSleep(int ius) {
	Sleep(ius / 1000);
	return 0;
}

OSG_INLINE uint64_t OGGetAbsoluteTimeUS() {
	static LARGE_INTEGER lpf;
	LARGE_INTEGER li;

	if (!lpf.QuadPart) {
		QueryPerformanceFrequency(&lpf);
	}

	QueryPerformanceCounter(&li);
	return li.QuadPart * 1000 * 1000 / lpf.QuadPart;
}

OSG_INLINE double OGGetAbsoluteTime() {
	static LARGE_INTEGER lpf;
	LARGE_INTEGER li;

	if (!lpf.QuadPart) {
		QueryPerformanceFrequency(&lpf);
	}

	QueryPerformanceCounter(&li);
	return (double)li.QuadPart / (double)lpf.QuadPart;
}

OSG_INLINE double OGGetFileTime(const char *file) {
	FILETIME ft;

	HANDLE h = CreateFile(file, GENERIC_READ, FILE_SHARE_READ, NULL, OPEN_EXISTING, 0, NULL);

	if (h == INVALID_HANDLE_VALUE)
		return -1;

	GetFileTime(h, 0, 0, &ft);

	CloseHandle(h);

	return ft.dwHighDateTime + ft.dwLowDateTime;
}

OSG_INLINE og_thread_t OGCreateThread(void *(routine)(void *), const char *name, void *parameter) {
	return (og_thread_t)CreateThread(0, 0, (LPTHREAD_START_ROUTINE)routine, parameter, 0, 0);
}

OSG_INLINE void *OGJoinThread(og_thread_t ot) {
	WaitForSingleObject(ot, INFINITE);
	CloseHandle(ot);
	return 0;
}

OSG_INLINE void OGCancelThread(og_thread_t ot) { CloseHandle(ot); }

OSG_INLINE og_mutex_t OGCreateMutex() {
	CRITICAL_SECTION *mutex = (CRITICAL_SECTION *)malloc(sizeof(CRITICAL_SECTION));
	InitializeCriticalSection(mutex);
	return mutex;
}

OSG_INLINE void OGLockMutex(og_mutex_t om) { EnterCriticalSection((CRITICAL_SECTION*)om); }

OSG_INLINE void OGUnlockMutex(og_mutex_t om) { LeaveCriticalSection((CRITICAL_SECTION*)om); }

OSG_INLINE void OGDeleteMutex(og_mutex_t om) { free(om); }

OSG_INLINE og_sema_t OGCreateSema() {
	HANDLE sem = CreateSemaphore(0, 0, 32767, 0);
	return (og_sema_t)sem;
}

OSG_INLINE int OGGetSema(og_sema_t os) {
	typedef LONG NTSTATUS;
	HANDLE sem = (HANDLE)os;
	typedef NTSTATUS(NTAPI * _NtQuerySemaphore)(
		HANDLE SemaphoreHandle, DWORD SemaphoreInformationClass, /* Would be SEMAPHORE_INFORMATION_CLASS */
		PVOID SemaphoreInformation,								 /* but this is to much to dump here     */
		ULONG SemaphoreInformationLength, PULONG ReturnLength OPTIONAL);

	typedef struct _SEMAPHORE_BASIC_INFORMATION {
		ULONG CurrentCount;
		ULONG MaximumCount;
	} SEMAPHORE_BASIC_INFORMATION;

	static _NtQuerySemaphore NtQuerySemaphore;
	SEMAPHORE_BASIC_INFORMATION BasicInfo;
	NTSTATUS Status;

	if (!NtQuerySemaphore) {
		NtQuerySemaphore = (_NtQuerySemaphore)GetProcAddress(GetModuleHandle("ntdll.dll"), "NtQuerySemaphore");
		if (!NtQuerySemaphore) {
			return -1;
		}
	}

	Status =
		NtQuerySemaphore(sem, 0 /*SemaphoreBasicInformation*/, &BasicInfo, sizeof(SEMAPHORE_BASIC_INFORMATION), NULL);

	if (Status == ERROR_SUCCESS) {
		return BasicInfo.CurrentCount;
	}

	return -2;
}

OSG_INLINE void OGLockSema(og_sema_t os) { WaitForSingleObject((HANDLE)os, INFINITE); }

OSG_INLINE void OGUnlockSema(og_sema_t os) { ReleaseSemaphore((HANDLE)os, 1, 0); }

OSG_INLINE void OGDeleteSema(og_sema_t os) { CloseHandle(os); }

OSG_INLINE void OGSignalCond(og_cv_t cv) { WakeConditionVariable((PCONDITION_VARIABLE)cv); }
OSG_INLINE void OGBroadcastCond(og_cv_t cv) { WakeAllConditionVariable((PCONDITION_VARIABLE)cv); }
OSG_INLINE void OGWaitCond(og_cv_t cv, og_mutex_t m) {
	SleepConditionVariableCS((PCONDITION_VARIABLE)cv, (CRITICAL_SECTION *)m, INFINITE);
}

OSG_INLINE bool OGWaitCondTimeout(og_cv_t cv, og_mutex_t m, int ms) {
	return SleepConditionVariableCS((PCONDITION_VARIABLE)cv, (CRITICAL_SECTION *)m, ms) != 0;
}

OSG_INLINE void OGDeleteConditionVariable(og_cv_t cv) { free(cv); }

OSG_INLINE og_cv_t OGCreateConditionVariable() {
	CONDITION_VARIABLE *cv = (CONDITION_VARIABLE *)malloc(sizeof(CONDITION_VARIABLE));
	InitializeConditionVariable(cv);
	return cv;
}

