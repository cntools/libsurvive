#include "survive_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

	struct SurviveAsyncContext;
	typedef struct SurviveAsyncContext SurviveAsyncContext;

	/***
	 * Initialize a new instance of an async context -- mirrors survive_init
	 * @return Pointer to the async context
	 */
	SURVIVE_EXPORT SurviveAsyncContext *survive_async_init(int argc, char *const *argv);

	/**
	 * Close all devices and free all memory associated with the given context
	 */
	SURVIVE_EXPORT void survive_async_close(SurviveAsyncContext* actx);

	/**
	 * Start the background thread which processes various inputs and produces deliverable data like position.
	 */
	SURVIVE_EXPORT void survive_async_start_thread(SurviveAsyncContext* actx);

	/**
	 * @return true iff the background thread is still running
	 */
	SURVIVE_EXPORT bool survive_async_is_running(SurviveAsyncContext *actx);

	struct SurviveAsyncObject;
	typedef struct SurviveAsyncObject SurviveAsyncObject;

	/**
	 * Get the first known object. Note that this also includes lighthouses
	 */
	SURVIVE_EXPORT const SurviveAsyncObject *survive_async_get_first_object(SurviveAsyncContext *actx);
	/**
	 * Get the next known object from a current one.
	 */
	SURVIVE_EXPORT const SurviveAsyncObject *survive_async_get_next_object(SurviveAsyncContext *actx,
																		   const SurviveAsyncObject *curr);

	/**
	 * Gets the next object which has been updated since we last looked at it with this function
	 */
	SURVIVE_EXPORT const SurviveAsyncObject *survive_async_get_next_updated(SurviveAsyncContext *actx);

	/**
	 * Gets the pose of a given object
	 */
	SURVIVE_EXPORT survive_timecode survive_async_object_get_latest_pose(const SurviveAsyncObject *sao,
																		 SurvivePose *pose);

	/**
	 * Gets the null terminated name of the object.
	 */
	SURVIVE_EXPORT const char *survive_async_object_name(const SurviveAsyncObject *sao);

#ifdef __cplusplus
}
#endif
