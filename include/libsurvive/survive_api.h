#include "survive_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct SurviveSimpleContext;
typedef struct SurviveSimpleContext SurviveSimpleContext;

/***
 * Initialize a new instance of an simple context -- mirrors survive_init
 * @return Pointer to the simple context
 */
SURVIVE_EXPORT SurviveSimpleContext *survive_simple_init(int argc, char *const *argv);

/**
 * Close all devices and free all memory associated with the given context
 */
SURVIVE_EXPORT void survive_simple_close(SurviveSimpleContext *actx);

/**
 * Start the background thread which processes various inputs and produces deliverable data like position.
 */
SURVIVE_EXPORT void survive_simple_start_thread(SurviveSimpleContext *actx);

/**
 * @return true iff the background thread is still running
 */
SURVIVE_EXPORT bool survive_simple_is_running(SurviveSimpleContext *actx);

struct SurviveSimpleObject;
typedef struct SurviveSimpleObject SurviveSimpleObject;

/**
 * Get the first known object. Note that this also includes lighthouses
 */
SURVIVE_EXPORT const SurviveSimpleObject *survive_simple_get_first_object(SurviveSimpleContext *actx);
/**
 * Get the next known object from a current one.
 */
SURVIVE_EXPORT const SurviveSimpleObject *survive_simple_get_next_object(SurviveSimpleContext *actx,
																		 const SurviveSimpleObject *curr);

SURVIVE_EXPORT int survive_simple_get_object_count(SurviveSimpleContext *actx);

/**
 * Gets the next object which has been updated since we last looked at it with this function
 */
SURVIVE_EXPORT const SurviveSimpleObject *survive_simple_get_next_updated(SurviveSimpleContext *actx);

/**
 * Gets the pose of a given object
 */
SURVIVE_EXPORT survive_timecode survive_simple_object_get_latest_pose(const SurviveSimpleObject *sao,
																	  SurvivePose *pose);

/**
 * Gets the velocity of a given object
 */
SURVIVE_EXPORT survive_timecode survive_simple_object_get_latest_velocity(const SurviveSimpleObject *sao,
																		  SurviveVelocity *pose);

/**
 * Gets the null terminated name of the object.
 */
SURVIVE_EXPORT const char *survive_simple_object_name(const SurviveSimpleObject *sao);

/**
 * Gets the null terminated serial number of the object.
 */
SURVIVE_EXPORT const char *survive_simple_serial_number(const SurviveSimpleObject *sao);

// The functions in this ifdef allow the possibility of breaking encapsulation and should be regarded as a relatively
// unstable interface. If you have a use case which requires access to these functions, it means there should likely
// be more safe accessor functions. Please file an issue at https://github.com/cnlohr/libsurvive/issues or write a PR
// and we'll try to include it in the safer API
#ifdef SURVIVE_ENABLE_FULL_API
/**
 * Gets the underlying SurviveContext
 */
SURVIVE_EXPORT SurviveContext *survive_simple_get_ctx(SurviveSimpleContext *actx);
/**
 * Gets the underlying SurviveObject. Lighthouses and external objects return null.
 */
SURVIVE_EXPORT SurviveObject *survive_simple_get_survive_object(const SurviveSimpleObject *sao);
/**
 * Gets the underlying BSD structure for a lighthouse. Non lighthouse objects return null.
 */
SURVIVE_EXPORT BaseStationData *survive_simple_get_bsd(const SurviveSimpleObject *sao);

SURVIVE_EXPORT void survive_simple_lock(SurviveSimpleContext *actx);
SURVIVE_EXPORT void survive_simple_unlock(SurviveSimpleContext *actx);
#endif

#ifdef __cplusplus
}
#endif
