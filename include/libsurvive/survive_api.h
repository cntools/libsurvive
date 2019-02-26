#include "survive_types.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct SurviveExternalObject {
	SurvivePose pose;
};

struct SurviveLighthouseData {
	int lighthouse;
	char serial_number[16];
};

struct SurviveSimpleObject {
	struct SurviveSimpleContext *actx;

	enum SurviveSimpleObject_type {
		SurviveSimpleObject_LIGHTHOUSE,
		SurviveSimpleObject_OBJECT,
		SurviveSimpleObject_EXTERNAL
	} type;

	union {
		struct SurviveLighthouseData lh;
		struct SurviveObject *so;
		struct SurviveExternalObject seo;
	} data;

	char name[32];
	bool has_update;
};

struct SurviveSimpleContext {
	SurviveContext* ctx; 
	
	bool running;
	og_thread_t thread;
	og_mutex_t poll_mutex;

	size_t external_object_ct;
	struct SurviveSimpleObject *external_objects;

	size_t object_ct;
	struct SurviveSimpleObject objects[];
};

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
 * Gets the next object which has been updated since we last looked at it with this function
 */
SURVIVE_EXPORT const SurviveSimpleObject *survive_simple_get_next_updated(SurviveSimpleContext *actx);

/**
 * Gets the pose of a given object
 */
SURVIVE_EXPORT survive_timecode survive_simple_object_get_latest_pose(const SurviveSimpleObject *sao,
																	  SurvivePose *pose);

/**
 * Gets the null terminated name of the object.
 */
SURVIVE_EXPORT const char *survive_simple_object_name(const SurviveSimpleObject *sao);

/**
 * Gets the null terminated serial number of the object.
 */
SURVIVE_EXPORT const char *survive_simple_serial_number(const SurviveSimpleObject *sao);

#ifdef __cplusplus
}
#endif
