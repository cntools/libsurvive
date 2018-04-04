#include "survive_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

	struct SurviveAsyncContext;
	typedef struct SurviveAsyncContext SurviveAsyncContext;

	SURVIVE_EXPORT SurviveAsyncContext *survive_async_init(int argc, char *const *argv);
	SURVIVE_EXPORT void survive_async_close(SurviveAsyncContext* actx);

	SURVIVE_EXPORT void survive_async_start_thread(SurviveAsyncContext* actx);
	SURVIVE_EXPORT int survive_async_stop_thread(SurviveAsyncContext* actx);
	SURVIVE_EXPORT bool survive_async_is_running(SurviveAsyncContext* actx);

	struct SurviveAsyncObject;
	typedef struct SurviveAsyncObject SurviveAsyncObject;

	SURVIVE_EXPORT const SurviveAsyncObject* survive_async_get_first_tracked(SurviveAsyncContext* actx);
	SURVIVE_EXPORT const SurviveAsyncObject* survive_async_get_next_tracked(SurviveAsyncContext* actx, const SurviveAsyncObject* curr);
	SURVIVE_EXPORT const SurviveAsyncObject* survive_async_get_next_updated(SurviveAsyncContext* actx);

	SURVIVE_EXPORT survive_timecode survive_async_object_get_latest_pose(const SurviveAsyncObject *sao,
																		 SurvivePose *OUTPUT);
	SURVIVE_EXPORT const char *survive_async_object_name(const SurviveAsyncObject *sao);

#ifdef __cplusplus
}
#endif
