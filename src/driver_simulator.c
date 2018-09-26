// All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL
// or LGPL licenses.

#include "math.h"
#include "os_generic.h"
#include "survive_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <survive.h>
#include <survive_reproject.h>

STATIC_CONFIG_ITEM(Simulator_DRIVER_ENABLE, "use-simulator", 'i', "Load a Simulator driver for testing.", 0);
STATIC_CONFIG_ITEM(Simulator_TIME, "simulator-time", 'f', "Seconds to run simulator for.", 0.0);

struct SurviveDriverSimulator {
	SurviveContext *ctx;
	SurviveObject *so;

	SurvivePose position;
	SurvivePose velocity;

	FLT time_last_imu;
	FLT time_last_light;
	FLT time_last_gt;
	FLT time_last_iterate;

	FLT timestart;
	int acode;
};
typedef struct SurviveDriverSimulator SurviveDriverSimulator;

static int Simulator_poll(struct SurviveContext *ctx, void *_driver) {
	SurviveDriverSimulator *driver = _driver;
	FLT timestamp = OGGetAbsoluteTime();

	FLT time_between_imu = 1. / driver->so->imu_freq;
	FLT time_between_pulses = 0.00833333333;
	FLT time_between_gt = time_between_pulses / 5.;

	FLT t = (timestamp - driver->timestart);

	// SurvivePose accel = {.Pos = {cos(t * 3) * 4, cos(t * 2) * 3, cos(t * 4) * 2},
	//					 .Rot = {10 + cos(t) * 2, cos(t), sin(t), (cos(t) + sin(t))}};

	SurvivePose accel = {.Rot = {10 + cos(t) * 2, cos(t), sin(t), (cos(t) + sin(t))}};

	LinmathVec3d attractors[] = {{1, 1, 1}, {-1, 0, 1}, {0, -1, .5}};

	for (int i = 0; i < sizeof(attractors) / sizeof(LinmathVec3d); i++) {
		LinmathVec3d acc;
		sub3d(acc, attractors[i], driver->position.Pos);
		FLT r = norm3d(acc);
		scale3d(acc, acc, 1. / r / r);
		add3d(accel.Pos, accel.Pos, acc);
	}

	// scale3d(accel.Pos, accel.Pos, 0);
	// scale3d(accel.Rot + 1, accel.Rot + 1, 0);
	quatnormalize(accel.Rot, accel.Rot);
	// quatrotatevector(accel.Pos, accel.Rot, accel.Pos);
	survive_timecode timecode = (survive_timecode)round(timestamp * 48000000.);

	if (timestamp > time_between_imu + driver->time_last_imu) {
		// ( SurviveObject * so, int mask, FLT * accelgyro, survive_timecode timecode, int id );
		FLT accelgyro[9] = {0, 0, 9.8066, // Acc
							0, 0, 0,	  // Gyro
							0, 0, 0};	 // Mag

		add3d(accelgyro, accelgyro, accel.Pos);
		scale3d(accelgyro, accelgyro, 1 / 9.8066);
		quatrotatevector(accelgyro, driver->position.Rot, accelgyro);

		LinmathQuat rotVel;
		quatrotateabout(rotVel, driver->position.Rot, driver->velocity.Rot);
		quattoeuler(accelgyro + 3, rotVel);

		ctx->imuproc(driver->so, 3, accelgyro, timecode, 0);

		driver->time_last_imu = timestamp;
	}

	if (timestamp > time_between_pulses + driver->time_last_light) {
		int lh = driver->acode >> 1;
		for (int idx = 0; idx < driver->so->sensor_ct; idx++) {
			FLT *pt = driver->so->sensor_locations + idx * 3;

			LinmathVec3d ptInWorld;
			ApplyPoseToPoint(ptInWorld, &driver->position, pt);

			SurvivePose world2lh = InvertPoseRtn(&ctx->bsd[lh].Pose);
			LinmathPoint3d ptInLh;
			ApplyPoseToPoint(ptInLh, &world2lh, ptInWorld);

			SurviveAngleReading ang;
			if (ptInLh[2] < 0) {
				survive_reproject_xy(ctx->bsd[lh].fcal, ptInLh, ang);

				// SurviveObject * so, int sensor_id, int acode, survive_timecode timecode, FLT length, FLT angle,
				// uint32_t lh);
				int acode = (lh << 2) + (driver->acode & 1);
				ctx->angleproc(driver->so, idx, acode, timecode, .006, ang[driver->acode & 1], lh);
			}
		}
		// SurviveObject * so, int sensor_id, int acode, int timeinsweep, survive_timecode timecode, survive_timecode
		// length, uint32_t lighthouse);
		int acode = (lh << 2) + (driver->acode & 1);
		ctx->lightproc(driver->so, -1, acode, 0, timecode, 100, lh);
		driver->acode = (driver->acode + 1) % 4;
		driver->time_last_light = timestamp;
	}

	if (timestamp > time_between_gt + driver->time_last_gt) {
		survive_default_external_pose_process(ctx, "Sim_GT", &driver->position);
		driver->time_last_gt = timestamp;
	}

	if (driver->time_last_iterate == 0) {
		driver->time_last_iterate = timestamp;
		driver->timestart = timestamp;
		return 0;
	}
	FLT time_diff = timestamp - driver->time_last_iterate;
	driver->time_last_iterate = timestamp;

	SurvivePose velGain;
	scale3d(velGain.Pos, accel.Pos, time_diff);
	quatmultiplyrotation(velGain.Rot, accel.Rot, time_diff);

	add3d(driver->velocity.Pos, driver->velocity.Pos, velGain.Pos);
	quatrotateabout(driver->velocity.Rot, driver->velocity.Rot, velGain.Rot);

	SurvivePose posGain;
	scale3d(posGain.Pos, driver->velocity.Pos, time_diff);
	quatmultiplyrotation(posGain.Rot, driver->velocity.Rot, time_diff);

	add3d(driver->position.Pos, driver->position.Pos, posGain.Pos);
	quatrotateabout(driver->position.Rot, driver->position.Rot, posGain.Rot);

	FLT time = survive_configf(ctx, "simulator-time", SC_GET, 0);
	if (timestamp - driver->timestart > time && time > 0)
		return 1;

	return 0;
}

int DriverRegSimulator(SurviveContext *ctx) {
	int enable = survive_configi(ctx, "use-simulator", SC_GET, 1);
	if (!enable)
		return 0;

	SurviveDriverSimulator *sp = calloc(1, sizeof(SurviveDriverSimulator));
	sp->ctx = ctx;

	sp->position.Rot[0] = sp->velocity.Rot[0] = 1;

	SV_INFO("Setting up Simulator driver.");

	// Create a new SurviveObject...
	SurviveObject *device = calloc(1, sizeof(SurviveObject));
	device->ctx = ctx;
	device->driver = sp;
	memcpy(device->codename, "SM0", 4);
	memcpy(device->drivername, "SIM", 4);
	device->sensor_ct = 20;
	device->sensor_locations = malloc(device->sensor_ct * sizeof(FLT) * 3);
	device->sensor_normals = malloc(device->sensor_ct * sizeof(FLT) * 3);

	device->head2imu.Rot[0] = 1;
	device->head2trackref.Rot[0] = 1;
	device->imu2trackref.Rot[0] = 1;

	FLT r = .25;
	srand(42);
	for (int i = 0; i < device->sensor_ct; i++) {
		FLT azi = rand();
		FLT pol = rand();
		FLT *normals = device->sensor_normals + i * 3;
		FLT *locations = device->sensor_locations + i * 3;
		normals[0] = locations[0] = r * cos(azi) * sin(pol);
		normals[1] = locations[1] = r * sin(azi) * sin(pol);
		normals[2] = locations[2] = r * cos(pol);
	}

	device->timebase_hz = 48000000;
	device->imu_freq = 1000.0f;

	sp->so = device;
	survive_add_object(ctx, device);
	survive_add_driver(ctx, sp, Simulator_poll, 0, 0);
	return 0;
}

REGISTER_LINKTIME(DriverRegSimulator);
