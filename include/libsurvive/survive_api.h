#pragma once

#include "survive_types.h"
#include <stdbool.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

struct SurviveSimpleContext;
typedef struct SurviveSimpleContext SurviveSimpleContext;

enum SurviveSimpleObject_type {
	SurviveSimpleObject_UNKNOWN,
	SurviveSimpleObject_LIGHTHOUSE,
	SurviveSimpleObject_HMD,
	SurviveSimpleObject_OBJECT,
	SurviveSimpleObject_EXTERNAL
};

typedef SurviveObjectSubtype SurviveSimpleSubobject_type;

struct SurviveSimpleObject;
typedef struct SurviveSimpleObject SurviveSimpleObject;
typedef void (*SurviveSimpleLogFn)(struct SurviveSimpleContext *ctx, SurviveLogLevel logLevel, const char *msg);

enum SurviveSimpleEventType { SurviveSimpleEventType_None = 0, SurviveSimpleEventType_ButtonEvent };

struct SurviveSimpleEvent;
typedef struct SurviveSimpleEvent SurviveSimpleEvent;

#define SURVIVE_MAX_AXIS_COUNT 8
typedef struct SurviveSimpleButtonEvent {
	SurviveSimpleObject *object;
	enum SurviveInputEvent event_type;
	enum SurviveButton button_id;

	uint8_t axis_count;
	enum SurviveAxis axis_ids[SURVIVE_MAX_AXIS_COUNT];
	SurviveAxisVal_t axis_val[SURVIVE_MAX_AXIS_COUNT];
} SurviveSimpleButtonEvent;

/***
 * Initialize a new instance of an simple context -- mirrors survive_init
 * @return Pointer to the simple context
 */
SURVIVE_EXPORT SurviveSimpleContext *survive_simple_init(int argc, char *const *argv);
SURVIVE_EXPORT void survive_simple_set_user(SurviveSimpleContext *, void *user);
SURVIVE_EXPORT void *survive_simple_get_user(SurviveSimpleContext *);
SURVIVE_EXPORT SurviveSimpleContext *survive_simple_init_with_logger(int argc, char *const *argv,
																	 SurviveSimpleLogFn fn);

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

/**
 * Get the first known object. Note that this also includes lighthouses
 */
SURVIVE_EXPORT const SurviveSimpleObject *survive_simple_get_first_object(SurviveSimpleContext *actx);
/**
 * Get the next known object from a current one.
 */
SURVIVE_EXPORT const SurviveSimpleObject *survive_simple_get_next_object(SurviveSimpleContext *actx,
																		 const SurviveSimpleObject *curr);
/**
 * Get a known object from a name.
 */
SURVIVE_EXPORT SurviveSimpleObject *survive_simple_get_object(SurviveSimpleContext *actx, const char *name);

SURVIVE_EXPORT size_t survive_simple_get_object_count(SurviveSimpleContext *actx);

/**
 * Gets the next object which has been updated since we last looked at it with this function
 */
SURVIVE_EXPORT const SurviveSimpleObject *survive_simple_get_next_updated(SurviveSimpleContext *actx);

/**
 * Gets the pose of a given object
 * @return Time in seconds since epoch of the pose
 */
SURVIVE_EXPORT FLT survive_simple_object_get_latest_pose(const SurviveSimpleObject *sao, SurvivePose *pose);

/**
 * Gets the velocity of a given object
 * @return Time in seconds since epoch of the velocity
 */
SURVIVE_EXPORT FLT survive_simple_object_get_latest_velocity(const SurviveSimpleObject *sao, SurviveVelocity *pose);

/**
 * Gets the null terminated name of the object.
 */
SURVIVE_EXPORT const char *survive_simple_object_name(const SurviveSimpleObject *sao);

/**
 * Gets the null terminated serial number of the object.
 */
SURVIVE_EXPORT const char *survive_simple_serial_number(const SurviveSimpleObject *sao);

/***
 * Block waiting for any kind of update from either locations or buttons
 * @return returns whether or not we are still running
 */
SURVIVE_EXPORT bool survive_simple_wait_for_update(SurviveSimpleContext *actx);
/**
 * Gets the next system event if there is one. Can return an event with NONE type.
 */
SURVIVE_EXPORT enum SurviveSimpleEventType survive_simple_next_event(SurviveSimpleContext *actx,
																	 SurviveSimpleEvent *event);

SURVIVE_EXPORT int survive_simple_object_haptic(struct SurviveSimpleObject *sao, FLT frequency, FLT amplitude,
												FLT time_s);
SURVIVE_EXPORT enum SurviveSimpleObject_type survive_simple_object_get_type(const struct SurviveSimpleObject *sao);
SURVIVE_EXPORT SurviveAxisVal_t survive_simple_object_get_input_axis(const struct SurviveSimpleObject *sao,
																	 enum SurviveAxis axis);
SURVIVE_EXPORT SurviveSimpleSubobject_type survive_simple_object_get_subtype(const struct SurviveSimpleObject *sao);
/**
 * Given an event with the type of 'button', it returns the internal ButtonEvent structure. If the type isn't a button,
 * it returns null.
 */
SURVIVE_EXPORT const SurviveSimpleButtonEvent *survive_simple_get_button_event(const SurviveSimpleEvent *event);

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
#define SURVIVE_ENCAPSULATE_DECORATOR(n) n
#else

// We purposefully obfuscate access to members marked like this. Go through function calls to get at these members, as
// there is -- at the very least -- more checks there.
//
// If you use the SURVIVE_ENABLE_FULL_API functionality you can get direct access, at the risk of compatibility issues
// in future releases
#define SURVIVE_ENCAPSULATE_DECORATOR(n) __private_##n
#endif

struct SurviveSimpleEvent {
	enum SurviveSimpleEventType event_type;
	SurviveSimpleButtonEvent SURVIVE_ENCAPSULATE_DECORATOR(button_event);
};

#ifdef __cplusplus
}

#endif
