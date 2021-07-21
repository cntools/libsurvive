#ifndef _LSPOSER_H
#define _LSPOSER_H

#include "survive_types.h"
#include <os_generic.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum PoserType_t {
	POSERDATA_NONE = 0,
	POSERDATA_IMU,
	POSERDATA_LIGHT, // Single lighting event.

	POSERDATA_DISASSOCIATE, // If you get this, it doesn't contain data.  It just tells you to please disassociate from
							// the current SurviveObject and delete your poserdata.
	POSERDATA_SYNC,			// Sync pulse.
	POSERDATA_LIGHT_GEN2,	// Gen2 lighting event.
	POSERDATA_SYNC_GEN2,	// Gen2 sync pulse

	POSERDATA_GLOBAL_SCENES
} PoserType;

typedef void (*poser_pose_func)(SurviveObject *so, uint32_t lighthouse, const SurvivePose *pose, void *user);
typedef void (*poser_lighthouse_pose_func)(SurviveObject *so, uint8_t lighthouse, SurvivePose *lighthouse_pose,
										   SurvivePose *object_pose, void *user);

typedef struct
{
	PoserType pt;
	survive_long_timecode timecode; // In object-local ticks.
	poser_pose_func poseproc;
	poser_lighthouse_pose_func lighthouseposeproc;
	void *userdata;
} PoserData;

SURVIVE_EXPORT int32_t PoserData_size(const PoserData *poser_data);

/**
 * Meant to be used by individual posers to report back their findings on the pose of an object back to the invoker of
 * the call.
 *
 * @param poser_data the data pointer passed into the poser function invocation
 * @param so The survive object which we are giving a solution for.
 * @param pose The actual object pose. This is in world space, not in LH space. It must represent a transformation from
 * object space of the SO to global space.
 */
SURVIVE_EXPORT void PoserData_poser_pose_func(PoserData *poser_data, SurviveObject *so, const SurvivePose *pose, FLT reprojError);

/**
 * Meant to be used by individual posers to report back their findings on the pose of an object back to the invoker of
 * the call.
 *
 * @param poser_data the data pointer passed into the poser function invocation
 * @param so The survive object which we are giving a solution for.
 * @param lighthouse @deprecated The lighthouse which observed that position. Make it -1 if it was a combination of
 * lighthouses. Will be removed in the future.
 * @param pose The actual object pose. This is in world space, not in LH space. It must represent a transformation from
 * object space of the SO to global space.
 */
SURVIVE_EXPORT void PoserData_poser_pose_func_with_velocity(PoserData *poser_data, SurviveObject *so,
															const SurvivePose *pose, const SurviveVelocity *velocity);

/**
 * Meant to be used by individual posers to report back their findings on the pose of a lighthouse.
 *
 * Note that you are free to assume the position of the lighthouse and solve for the object or vice versa. Most solvers
 * assume that the object is at 0,0,0 but this isn't a hard requirement.
 *
 * @param poser_data the data pointer passed into the poser function invocation
 * @param so The survive object which gave us the info for the solution
 * @param lighthouse The lighthouse which to solve for
 * @param lighthouse_pose This is the assumed or derived position of the given lighthouse.
 * @param object_pose This is the assumed or derived position of the tracked object.
 */
SURVIVE_EXPORT void PoserData_lighthouse_pose_func(PoserData *poser_data, SurviveObject *so, uint8_t lighthouse,
												   SurvivePose *lighthouse_pose, FLT var, SurvivePose *object_pose);
SURVIVE_EXPORT void PoserData_normalize_scene(SurviveContext *ctx, SurvivePose *lighthouse_pose,
											  uint32_t lighthouse_count, SurvivePose *object_pose);
SURVIVE_EXPORT void PoserData_lighthouse_poses_func(PoserData *poser_data, SurviveObject *so,
													SurvivePose *lighthouse_pose, FLT *variances,
													uint32_t lighthouse_count, SurvivePose *object_pose);
SURVIVE_EXPORT int8_t survive_get_reference_bsd(SurviveContext *ctx, SurvivePose *lighthouse_pose,
												uint32_t lighthouse_count);

SURVIVE_EXPORT FLT survive_lighthouse_adjust_confidence(SurviveContext *ctx, uint8_t bsd_idx, FLT delta);
SURVIVE_EXPORT FLT survive_adjust_confidence(SurviveObject *so, FLT delta);

typedef struct PoserDataIMU {
	PoserData hdr;
	uint8_t datamask;  //0 = accel present, 1 = gyro present, 2 = mag present.
	FLT accel[3];
	FLT gyro[3];
	FLT mag[3];
} PoserDataIMU;

typedef struct PoserDataLight {
	PoserData hdr;
	int sensor_id;
	int lh;             //Lighthouse making this sweep
	FLT angle;			//In radians from center of lighthouse.

	bool assume_current_pose; // Don't solve for object pose; use OutPoseIMU for LH solving
	bool no_lighthouse_solve; // Don't solve for LH positions
} PoserDataLight;

typedef struct PoserDataLightGen1 {
	PoserDataLight common;

	int acode;  // OOTX Code associated with this sweep. bit 1 indicates vertical(1) or horizontal(0) sweep
	FLT length; // In seconds
} PoserDataLightGen1;

typedef struct PoserDataLightGen2 {
	PoserDataLight common;

	int8_t plane;
	uint32_t sync;
} PoserDataLightGen2;

SURVIVE_EXPORT int PoserDataLight_axis(const struct PoserDataLight *pdl);
typedef struct {
	FLT value;
	uint8_t lh;
	uint8_t sensor_idx;
	uint8_t axis;
} PoserDataGlobalSceneMeasurement;

struct PoserDataGlobalScene {
	struct SurviveObject *so;
	SurvivePose pose;
	LinmathPoint3d accel;
	size_t meas_cnt;
	PoserDataGlobalSceneMeasurement *meas;
};

typedef struct PoserDataGlobalScenes {
	PoserData hdr;

	SurvivePose *world2lhs;
	size_t scenes_cnt;
	struct PoserDataGlobalScene *scenes;
} PoserDataGlobalScenes;

union PoserDataAll {
	PoserData pd;
	PoserDataLight pdl1;
	PoserDataLightGen2 pdlg2;
	PoserDataIMU pdimu;
	PoserDataGlobalScenes pdgs;
};

struct SurviveSensorActivations_s;

//When you write your posers, use the following definition, and register with REGISTER_LINKTIME.
typedef int (*PoserCB)(SurviveObject *so, void **user, PoserData *pd);

#define SURVIVE_POSER_INVOKE(so, poserData) survive_poser_invoke(so, (PoserData *)poserData, sizeof(*poserData));
void survive_poser_invoke(SurviveObject *so, PoserData *poserData, size_t poserDataSize);

struct survive_threaded_poser;
struct survive_threaded_poser *survive_create_threaded_poser(SurviveObject *so, PoserCB innerPoser);
int survive_threaded_poser_fn(SurviveObject *so, void **user, PoserData *pd);

#ifdef __cplusplus
};
#endif

#endif
