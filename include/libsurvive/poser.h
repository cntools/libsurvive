#ifndef _LSPOSER_H
#define _LSPOSER_H

#include "survive_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum PoserType_t {
	POSERDATA_NONE = 0,
	POSERDATA_IMU,
	POSERDATA_LIGHT,		// Single lighting event.
	POSERDATA_FULL_SCENE,   // Full, statified X, Y sweep data for both lighthouses.
	POSERDATA_DISASSOCIATE, // If you get this, it doesn't contain data.  It just tells you to please disassociate from
							// the current SurviveObject and delete your poserdata.
	POSERDATA_SYNC, // Sync pulse.
} PoserType;

typedef void (*poser_pose_func)(SurviveObject *so, uint32_t lighthouse, SurvivePose *pose, void *user);
typedef void (*poser_lighthouse_pose_func)(SurviveObject *so, uint8_t lighthouse, SurvivePose *lighthouse_pose,
										   SurvivePose *object_pose, void *user);

typedef struct
{
	PoserType pt;
	poser_pose_func poseproc;
	poser_lighthouse_pose_func lighthouseposeproc;
	void *userdata;
} PoserData;

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
SURVIVE_EXPORT void PoserData_poser_pose_func(PoserData *poser_data, SurviveObject *so, SurvivePose *pose);

/**
 * Meant to be used by individual posers to report back their findings on the pose of a lighthouse.
 *
 * Note that you are free to assume the position of the lighthouse and solve for the object or vice versa. Most solvers
 * assume that the object is at 0,0,0 but this isn't a hard requirement.
 *
 * @param poser_data the data pointer passed into the poser function invocation
 * @param so The survive object which gave us the info for the solution
 * @param lighthouse The lighthouse which to solve for
 * @param arb2world For use when solving for both ligthhouse positions. For the first invocation of this function,
 * pass in a zero-inited SurvivePose. This function will set that to a relative transform to normalize the space.
 * pass the same pose in again for the second lighthouse to get accurate results.
 * @param lighthouse_pose This is the assumed or derived position of the given lighthouse.
 * @param object_pose This is the assumed or derived position of the tracked object.
 */
SURVIVE_EXPORT void PoserData_lighthouse_pose_func(PoserData *poser_data, SurviveObject *so, uint8_t lighthouse,
									/* OUTPARAM */ SurvivePose *arb2world, SurvivePose *lighthouse_pose,
									SurvivePose *object_pose);

typedef struct PoserDataIMU {
	PoserData hdr;
	uint8_t datamask;  //0 = accel present, 1 = gyro present, 2 = mag present.
	FLT accel[3];
	FLT gyro[3];
	FLT mag[3];
	uint32_t timecode; //In object-local ticks.
} PoserDataIMU;

typedef struct PoserDataLight {
	PoserData hdr;
	int sensor_id;
	int acode;			//OOTX Code associated with this sweep. bit 1 indicates vertical(1) or horizontal(0) sweep
	int lh;             //Lighthouse making this sweep
	uint32_t timecode;  //In object-local ticks.
	FLT length;			//In seconds
	FLT angle;			//In radians from center of lighthouse.
} PoserDataLight;

typedef struct
{
	PoserData hdr;

	//If "lengths[...]" < 0, means not a valid piece of sweep information.
	FLT  lengths[SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2];
	FLT  angles [SENSORS_PER_OBJECT][NUM_LIGHTHOUSES][2];  //2 Axes  (Angles in LH space)

	PoserDataIMU lastimu;
} PoserDataFullScene;

struct SurviveSensorActivations_s;
SURVIVE_EXPORT void PoserDataFullScene2Activations(const PoserDataFullScene *pdfs, struct SurviveSensorActivations_s *activations);

//When you write your posers, use the following definition, and register with REGISTER_LINKTIME.
typedef int (*PoserCB)( SurviveObject * so, PoserData * pd );


#ifdef __cplusplus
};
#endif

#endif
