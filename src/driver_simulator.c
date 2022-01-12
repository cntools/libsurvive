// All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL
// or LGPL licenses.

#include "math.h"
#include "os_generic.h"
#include "survive_config.h"
#include "survive_default_devices.h"
#include "survive_reproject_gen2.h"
#include "survive_str.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <survive.h>
#include <survive_reproject.h>

#include "survive_recording.h"
#include "variance.h"

STATIC_CONFIG_ITEM(Simulator_DRIVER_ENABLE, "simulator", 'b', "Load a Simulator driver for testing.", 0)

typedef struct SurviveDriverSimulatorLHState {
	FLT last_eval_time;

	FLT period_s;
	FLT start_time;
} SurviveDriverSimulatorLHState;

typedef SurviveVelocity SurviveAcceleration;
struct SurviveDriverSimulator {
	int lh_version;
	SurviveContext *ctx;
	SurviveObject *so;

	SurviveDriverSimulatorLHState lhstates[NUM_GEN2_LIGHTHOUSES];
	BaseStationData bsd[NUM_GEN2_LIGHTHOUSES];

	SurvivePose position;
	SurviveVelocity velocity;
	SurviveAcceleration accel;

	FLT time_last_imu;
	FLT time_last_light;
	FLT time_last_iterate;

	FLT noise_scale;
	FLT sensor_var;
	FLT sensor_droprate;
	FLT init_time;

	FLT scale_error;
	FLT timestart;
	FLT current_timestamp;
	int acode;

	FLT gyro_bias[3];
	FLT gyro_bias_scale;
	FLT gyro_var;
	FLT sensor_jitter;
	FLT acc_var;
	int show_gt_device_cfg;

	struct variance_measure pose_variance;

	pose_process_func pose_fn;
	lighthouse_pose_process_func lh_fn;

	struct {
		FLT runtime;
		FLT obj_radius;
		FLT fcal_noise;
		FLT time_factor;
		int obj_sensors;
		int attractors;

		FLT lh_duty_cycle;
		int report_in_imu;
	} settings;
};
typedef struct SurviveDriverSimulator SurviveDriverSimulator;

// clang-format off
STRUCT_CONFIG_SECTION(SurviveDriverSimulator)
    STRUCT_CONFIG_ITEM("simulator-attractors",  "Number on gravity attractors in simulation", 3, t->settings.attractors)
    STRUCT_CONFIG_ITEM("simulator-obj-sensors",  "Number on sensors on the simulated object", 20, t->settings.obj_sensors)
    STRUCT_CONFIG_ITEM("simulator-fcal-noise",  "Noise to apply to BSD fcal parameters", 1e-3, t->settings.fcal_noise)
    STRUCT_CONFIG_ITEM("simulator-init-time", "Init time -- object wont move for this long", 2., t->init_time)
    STRUCT_CONFIG_ITEM("simulator-gyro-noise", "Variance of noise to apply to gyro", 1e-3, t->gyro_var)
    STRUCT_CONFIG_ITEM("simulator-acc-noise", "Variance of noise to apply to accelerometer", 1e-4, t->acc_var)
    STRUCT_CONFIG_ITEM("simulator-gyro-bias", "Scale of bias to apply to gyro", 1e-1, t->gyro_bias_scale)

    STRUCT_CONFIG_ITEM("simulator-sensor-time-jitter", "Variance of time jitter to apply to light sensors", 1e-2, t->sensor_jitter)
    STRUCT_CONFIG_ITEM("simulator-show-gt","0: No GT device, 1: Show GT device, 2: Only GT device", 1, t->show_gt_device_cfg)
    STRUCT_CONFIG_ITEM("simulator-sensor-noise", "Variance of noise to apply to light sensors", 1e-4, t->sensor_var)
    STRUCT_CONFIG_ITEM("simulator-obj-radius", "Radius of the simulated object", 0.05, t->settings.obj_radius)
    STRUCT_CONFIG_ITEM("simulator-time", "Seconds to run simulator for.", 0.0, t->settings.runtime)
    STRUCT_CONFIG_ITEM("simulator-sensor-droprate", "Chance to drop a sensor reading", .2, t->sensor_droprate)
    STRUCT_CONFIG_ITEM("simulator-noise-scale", "", 1., t->noise_scale)
    STRUCT_CONFIG_ITEM("simulator-lh-gen", "Lighthouse generation", 1, t->lh_version)

    STRUCT_CONFIG_ITEM("simulator-lh-duty-cycle", "Duty cycle of lighthouses", 1., t->settings.lh_duty_cycle)
    STRUCT_EXISTING_CONFIG_ITEM("report-in-imu",t->settings.report_in_imu)
END_STRUCT_CONFIG_SECTION(SurviveDriverSimulator)
// clang-format on

static double timestamp_in_s() {
	static double start_time_s = 0;
	if (start_time_s == 0.)
		start_time_s = OGGetAbsoluteTime();
	return OGGetAbsoluteTime() - start_time_s;
}

static FLT lighthouse_lasttime_of_angle(SurviveDriverSimulator *driver, int lh, FLT timestamp, FLT angle) {
	SurviveDriverSimulatorLHState *lhs = &driver->lhstates[lh];
	return timestamp - fmod(timestamp - lhs->start_time, lhs->period_s) + angle / (2 * LINMATHPI) * lhs->period_s;
}
static FLT lighthouse_sync_time(SurviveDriverSimulator *driver, int lh, FLT timestamp) {
	return lighthouse_lasttime_of_angle(driver, lh, timestamp, 0);
}
FLT lighthouse_angle(SurviveDriverSimulator *driver, int lh, FLT timestamp) {
	SurviveDriverSimulatorLHState *lhs = &driver->lhstates[lh];

	FLT angle = fmod(timestamp - lhs->start_time, lhs->period_s) / lhs->period_s * 2. * LINMATHPI;
	return angle;
}
static bool lighthouse_sensor_angle(SurviveDriverSimulator *driver, int lh, size_t idx, SurviveAngleReading ang) {
	SurviveContext *ctx = driver->ctx;
	LinmathVec3d pt;
	copy3d(pt, driver->so->sensor_locations + idx * 3);

	SurvivePose imu2trackref = driver->so->imu2trackref;
	SurvivePose trackref2imu = InvertPoseRtn(&imu2trackref);

	ApplyPoseToPoint(pt, &imu2trackref, pt);
	scale3d(pt, pt, driver->scale_error);
	ApplyPoseToPoint(pt, &trackref2imu, pt);

	if (driver->settings.lh_duty_cycle < 1) {
		FLT t = driver->current_timestamp + driver->lhstates[lh].start_time;
		if (fmod(t, 1) > driver->settings.lh_duty_cycle) {
			return false;
		}
	}

	LinmathVec3d ptInWorld;
	LinmathVec3d normalInWorld;
	ApplyPoseToPoint(ptInWorld, &driver->position, pt);
	SurvivePose world2lh = InvertPoseRtn(&driver->bsd[lh].Pose);
	LinmathPoint3d ptInLh;
	ApplyPoseToPoint(ptInLh, &world2lh, ptInWorld);

	if (ptInLh[2] < 0) {
		LinmathVec3d dirLh;
		normalize3d(dirLh, ptInLh);
		scale3d(dirLh, dirLh, -1);

		quatrotatevector(normalInWorld, driver->position.Rot, driver->so->sensor_normals + idx * 3);

		LinmathVec3d normalInLh;
		quatrotatevector(normalInLh, world2lh.Rot, normalInWorld);

		FLT facingness = dot3d(normalInLh, dirLh);
		if (facingness > 0 && linmath_rand(0, 1.) > driver->sensor_droprate * driver->noise_scale) {
			if (driver->lh_version == 0) {
				survive_reproject_xy(driver->bsd[lh].fcal, ptInLh, ang);
				for (int i = 0; i < 2; i++) {
					if (fabs(ang[i]) > 1.04)
						return false;
				}
			} else {
				survive_reproject_xy_gen2(driver->bsd[lh].fcal, ptInLh, ang);
				ang[0] += 2 * LINMATHPI / 3.;
				ang[1] += 4 * LINMATHPI / 3.;
			}

			for (int i = 0; i < 2; i++) {
				ang[i] += linmath_normrand(0, driver->sensor_var * driver->noise_scale);
			}
			return true;
		}
	}
	return false;
}

struct lh_event {
	FLT time;
	uint8_t lh;
	int idx;
};

static int event_compare(const void *p, const void *q) {
	const struct lh_event *x = (const struct lh_event *)p;
	const struct lh_event *y = (const struct lh_event *)q;

	return x->time > y->time;
}
static size_t run_lighthouse_v2(SurviveDriverSimulator *driver, int lh, FLT timestamp, struct lh_event *events) {
	SurviveContext *ctx = driver->ctx;
	SurviveDriverSimulatorLHState *lhs = &driver->lhstates[lh];

	size_t evt_idx = 0;

	FLT sync_time = lighthouse_sync_time(driver, lh, timestamp);

	if (sync_time >= lhs->last_eval_time && sync_time <= timestamp) {
		// fprintf(stderr, "Sync %d %f %f\n", lh, sync_time, timestamp);
		events[evt_idx].time = sync_time;
		events[evt_idx].lh = lh;
		events[evt_idx++].idx = -1;
	}

	for (size_t idx = 0; idx < driver->so->sensor_ct; idx++) {
		SurviveAngleReading ang;

		if (lighthouse_sensor_angle(driver, lh, idx, ang)) {
			for (int axis = 0; axis < 2; axis++) {
				FLT angle_time = lighthouse_lasttime_of_angle(driver, lh, timestamp, ang[axis]);
				if (angle_time >= lhs->last_eval_time && angle_time <= timestamp) {
					events[evt_idx].time = angle_time;
					events[evt_idx].lh = lh;
					events[evt_idx++].idx = idx;
				}
			}
		}
	}

	lhs->last_eval_time = timestamp;

	return evt_idx;
}
static void run_lighthouse_v1(SurviveDriverSimulator *driver, int lh, FLT timestamp) {
	SurviveContext *ctx = driver->ctx;
	survive_timecode timecode = (survive_timecode)round(timestamp * 48000000.);

	if (lh >= ctx->activeLighthouses || driver->bsd[lh].PositionSet == false) {
		driver->acode = (driver->acode + 1) % 4;
	} else {
		for (int idx = 0; idx < driver->so->sensor_ct; idx++) {
			SurviveAngleReading ang = {0};
			if (lighthouse_sensor_angle(driver, lh, idx, ang)) {
				if (driver->lh_version == 0) {
					int acode = (lh << 2) + (driver->acode & 1);
					SURVIVE_INVOKE_HOOK_SO(angle, driver->so, idx, acode, timecode, .006, ang[driver->acode & 1], lh);
				} else {
					SURVIVE_INVOKE_HOOK_SO(sweep_angle, driver->so, driver->bsd[lh].mode, idx, timecode,
										   driver->acode & 1, ang[driver->acode & 1]);
				}
			}
		}

		if (driver->lh_version == 0) {
			int acode = (lh << 2) + (driver->acode & 1);
			SURVIVE_INVOKE_HOOK_SO(light, driver->so, -3, acode, 0, timecode, 100, lh);
			driver->acode = (driver->acode + 1) % 4;
		} else {
			SURVIVE_INVOKE_HOOK_SO(sync, driver->so, driver->bsd[lh].mode, timecode, false, false);
			driver->acode = (driver->acode + 1) % 4;
		}
	}
}

static bool run_imu(struct SurviveContext *ctx, SurviveDriverSimulator *driver, double timestamp,
					double time_between_imu, survive_long_timecode timecode) {
	bool update_gt = false;
	if (timestamp > time_between_imu + driver->time_last_imu) {
		update_gt = true;
		// ( SurviveObject * so, int mask, FLT * accelgyro, survive_timecode timecode, int id );
		FLT accelgyro[9] = {0, 0, 0,  // Acc
							0, 0, 0,  // Gyro
							0, 0, 0}; // Mag

		add3d(accelgyro, accelgyro, driver->accel.Pos);
		scale3d(accelgyro, accelgyro, 1. / 9.80665);

		SV_VERBOSE(200, "(Gt)Acc\t\t" Point3_format "\t%f", LINMATH_VEC3_EXPAND(accelgyro), norm3d(accelgyro));
		accelgyro[2] += 1;

		SurvivePose ip = InvertPoseRtn(&driver->position);
		LinmathQuat q;
		quatgetconjugate(q, driver->position.Rot);
		quatrotatevector(accelgyro, q, accelgyro);
		quatrotatevector(accelgyro + 3, q, driver->velocity.AxisAngleRot);
		add3d(accelgyro + 3, accelgyro + 3, driver->gyro_bias);

		for (int i = 0; i < 3; i++) {
			accelgyro[i] += linmath_normrand(0, driver->acc_var * driver->noise_scale);
			accelgyro[i + 3] += linmath_normrand(0, driver->gyro_var * driver->noise_scale);
		}

		SV_VERBOSE(200, "Ang: " Point3_format, LINMATH_VEC3_EXPAND(driver->velocity.AxisAngleRot));
		SV_VERBOSE(200, "GT: " SurvivePose_format " %f", SURVIVE_POSE_EXPAND(driver->position),
				   quatmagnitude(driver->position.Rot));
		if (driver->show_gt_device_cfg != 2) {
			SURVIVE_INVOKE_HOOK_SO(imu, driver->so, 3, accelgyro, timecode, 0);
		}

		for (int i = 0; i < 3; i++) {
			driver->gyro_bias[i] += linmath_normrand(0, driver->gyro_bias_scale * driver->noise_scale) * .001;
		}
		driver->time_last_imu = timestamp - 1e-10;
	}
	return update_gt;
}
bool run_light(const struct SurviveContext *ctx, SurviveDriverSimulator *driver, double timestamp,
			   double time_between_pulses) {
	bool update_gt = false;
	if (driver->show_gt_device_cfg == 2) {
		return false;
	}

	if (driver->lh_version == 0) {
		if (timestamp > time_between_pulses + driver->time_last_light) {
			update_gt = true;
			int lh = driver->acode >> 1;

			run_lighthouse_v1(driver, lh, timestamp);
			driver->time_last_light = timestamp;
		}
	} else {
		struct lh_event events[NUM_GEN2_LIGHTHOUSES * (SENSORS_PER_OBJECT + 1)];
		size_t evt_idx = 0;
		for (int i = 0; i < ctx->activeLighthouses; i++) {
			evt_idx += run_lighthouse_v2(driver, i, timestamp, events + evt_idx);
		}

		qsort(events, evt_idx, sizeof *events, event_compare);

		for (size_t i = 0; i < evt_idx; i++) {
			survive_timecode timecode = (survive_timecode)round(events[i].time * 48000000.);
			uint8_t lh = events[i].lh;
			if (events[i].idx == -1) {
				SURVIVE_INVOKE_HOOK_SO(sync, driver->so, driver->bsd[lh].mode, timecode, 0, 0);
			} else {
				SURVIVE_INVOKE_HOOK_SO(sweep, driver->so, driver->bsd[lh].mode, events[i].idx, timecode, 0);
			}
		}
	}
	return update_gt;
}
static void propagate_state(SurviveDriverSimulator *driver, double time_diff) {
	SurviveVelocity velGain;
	scale3d(velGain.Pos, driver->accel.Pos, time_diff);
	scale3d(velGain.AxisAngleRot, driver->accel.AxisAngleRot, time_diff);

	add3d(driver->velocity.Pos, driver->velocity.Pos, velGain.Pos);
	add3d(driver->velocity.AxisAngleRot, velGain.AxisAngleRot, driver->velocity.AxisAngleRot);

	SurviveVelocity posGain;
	scale3d(posGain.Pos, driver->velocity.Pos, time_diff);
	add3d(driver->position.Pos, driver->position.Pos, posGain.Pos);

	survive_apply_ang_velocity(driver->position.Rot, driver->velocity.AxisAngleRot, time_diff, driver->position.Rot);
}
static void update_gt_device(struct SurviveContext *ctx, const SurviveDriverSimulator *driver) {
	if (driver->show_gt_device_cfg == 0)
		return;

	SurvivePose head2world = driver->position;
	if (!driver->settings.report_in_imu) {
		ApplyPoseToPose(&head2world, &driver->position, &driver->so->head2imu);
	}

	survive_default_external_pose_process(ctx, "Sim_GT", &head2world);
	survive_default_external_velocity_process(ctx, "Sim_GT", &driver->velocity);
	survive_recording_write_to_output(ctx->recptr, "%s FULL_STATE " Point16_format "\n", "Sim_GT",
									  SURVIVE_POSE_EXPAND(head2world), SURVIVE_VELOCITY_EXPAND(driver->velocity),
									  LINMATH_VEC3_EXPAND(&driver->accel.Pos[0]));
}
void apply_attractors(struct SurviveContext *ctx, SurviveDriverSimulator *driver) {
	SurviveVelocity accel = {0};

	FLT time_since_init = driver->current_timestamp - driver->init_time;
	FLT s = 1.;

	LinmathVec3d attractors[] = {{1, 1, 1}, {-1, 0, 1}, {0, -1, .5}};

	int attractor_cnt = driver->settings.attractors;
	if (attractor_cnt > (int)(sizeof(attractors) / sizeof(LinmathVec3d))) {
		attractor_cnt = sizeof(attractors) / sizeof(LinmathVec3d);
	}

	static bool reported = false;

	for (int i = 0; i < attractor_cnt; i++) {
		LinmathVec3d acc;
		sub3d(acc, attractors[i], driver->position.Pos);
		FLT r = norm3d(acc);
		scale3d(acc, acc, s / r / r);
		if (r < .1) {
			scale3d(acc, acc, -1);
		}
		add3d(accel.Pos, accel.Pos, acc);
		if (reported == false && ctx->recptr) {
			survive_recording_write_to_output(ctx->recptr, "SPHERE attractor_%d %f %d " Point3_format "\n", i, .05,
											  0x00FF00, LINMATH_VEC3_EXPAND(attractors[i]));
		}
	}
	reported = true;

	if (attractor_cnt == 0) {
		// accel.Pos[0] = 1 * cos(timestamp);
	}

	for (int i = 0; i < 3; i++) {
		// accel.AxisAngleRot[i] += (((rand() / (FLT)RAND_MAX) - .5) * 10);
		// accel.AxisAngleRot[i] = cos(timestamp);
	}

	memcpy(&driver->accel, &accel, sizeof(accel));
}
static void apply_initial_position(SurviveDriverSimulator *driver) {
	FLT up[] = {0, 0, 1};
	FLT ones[] = {1, -1, 1};
	quatfrom2vectors(driver->position.Rot, up, ones);
	for (int i = 0; i < 3; i++)
		driver->position.Pos[i] = 0;
}

static void apply_initial_velocity(SurviveDriverSimulator *sp) {
	SurviveContext *ctx = sp->ctx;

	int attractor_cnt = sp->settings.attractors;
	if (attractor_cnt >= 0) {
		sp->velocity.AxisAngleRot[0] = sp->velocity.AxisAngleRot[1] = sp->velocity.AxisAngleRot[2] = 1.;
	}

	if (attractor_cnt == 1) {
		for (int i = 0; i < 3; i++)
			sp->velocity.Pos[i] = 2. * rand() / RAND_MAX - 1;
	}
}

static int Simulator_poll(struct SurviveContext *ctx, void *_driver) {
	SurviveDriverSimulator *driver = _driver;
	static FLT last_time = 0;
	FLT realtime = timestamp_in_s();

	FLT timefactor = driver->settings.time_factor;
	FLT timestep = .01;

	while (last_time != 0 && last_time + timefactor * timestep > realtime) {
		survive_release_ctx_lock(ctx);
		OGUSleep((timefactor * timestep + realtime - last_time) * 1e6);
		survive_get_ctx_lock(ctx);
		realtime = timestamp_in_s();
	}
	last_time = realtime;

	bool wasIniting = driver->current_timestamp < driver->init_time;
	FLT timestamp = (driver->current_timestamp += timestep);
	FLT time_between_imu = 1. / driver->so->imu_freq;
	FLT time_between_pulses = 0.00833333333;
	FLT time_between_gt = time_between_imu;
	bool isIniting = timestamp < driver->init_time || driver->init_time < 0;

	bool update_gt = false;

	FLT t = (timestamp - driver->timestart);

	if (wasIniting == true && isIniting == false) {
		apply_initial_velocity(driver);
	}

	if (isIniting == false) {
		apply_attractors(ctx, driver);
		for (int i = 0; i < 3; i++) {
			driver->accel.AxisAngleRot[i] += linmath_rand(-1e-1, 1e-1);
		}
	}

	survive_long_timecode timecode = (survive_long_timecode)round(timestamp * 48000000.);

	update_gt |= run_imu(ctx, driver, timestamp, time_between_imu, timecode);
	update_gt |= run_light(ctx, driver, timestamp, time_between_pulses);

	if (update_gt) {
		update_gt_device(ctx, driver);
	}

	if (driver->time_last_iterate == 0) {
		driver->time_last_iterate = timestamp;
		driver->timestart = timestamp;
		return 0;
	}
	FLT time_diff = timestamp - driver->time_last_iterate;
	// SV_INFO("%.013f", time_diff);
	driver->time_last_iterate = timestamp;

	propagate_state(driver, time_diff);

	FLT time = driver->settings.runtime;
	if (timestamp - driver->timestart > time && time > 0) {
		SV_INFO("Simulation finished after %f seconds", realtime);
		return 1;
	}

	return 0;
}

const BaseStationData simulated_bsd[5] = {
	{.PositionSet = 1,
	 .BaseStationID = 0,
	 .Pose = {.Pos = {-3, 0, 1}, .Rot = {-0.70710678118, 0, 0.70710678118, 0}},
	 .mode = 0,
	 .OOTXSet = 1},
	{.PositionSet = 1,
	 .BaseStationID = 1,
	 .Pose = {.Pos = {3, 0, 1}, .Rot = {0.70710678118, 0, 0.70710678118, 0}},
	 .mode = 1,
	 .OOTXSet = 1},
	{.PositionSet = 1,
	 .BaseStationID = 1,
	 .Pose = {.Pos = {0, 3, 1}, .Rot = {0.70710678118, -0.70710678118, 0, 0}},
	 .mode = 2,
	 .OOTXSet = 1},
	{.PositionSet = 1,
	 .BaseStationID = 1,
	 .Pose = {.Pos = {0, -3, 1}, .Rot = {0.70710678118, 0.70710678118, 0, 0}},
	 .mode = 3,
	 .OOTXSet = 1},
	{.PositionSet = 1, .BaseStationID = 1, .Pose = {.Pos = {0, 0, 6}, .Rot = {1, 0, 0, 0}}, .mode = 4, .OOTXSet = 1},
};

static void simulation_lh_compare(SurviveContext *ctx, uint8_t lighthouse, const SurvivePose *lighthouse_pose) {
	const SurviveDriverSimulator *driver = survive_get_driver(ctx, Simulator_poll);

	SV_VERBOSE(50, "Simulation LH%d position " SurvivePose_format "\t", lighthouse,
			   SURVIVE_POSE_EXPAND(driver->bsd[lighthouse].Pose));
	SV_VERBOSE(50, "Found      LH%d position " SurvivePose_format "\t", lighthouse,
			   SURVIVE_POSE_EXPAND(*lighthouse_pose));

	driver->lh_fn(ctx, lighthouse, lighthouse_pose);
}

static void simulation_compare(SurviveObject *so, survive_long_timecode timecode, const SurvivePose *imupose) {
	survive_default_imupose_process(so, timecode, imupose);
	if (strcmp(so->drivername, "SIM") != 0) {
		return;
	}
	SurviveContext *ctx = so->ctx;
	SurviveDriverSimulator *driver = so->driver;

	SurvivePose p = InvertPoseRtn(&driver->position);
	ApplyPoseToPose(&p, &p, &so->OutPoseIMU);

	FLT error[7] = {0};
	FLT verror[6] = {0};
	FLT aerror[3] = {0};
	subnd(error, driver->position.Pos, so->OutPoseIMU.Pos, 3);

	for (int i = 0; i < 4; i++)
		error[i + 3] = driver->position.Rot[i] * (driver->position.Rot[0] > 0 ? 1 : -1) -
					   so->OutPoseIMU.Rot[i] * (so->OutPoseIMU.Rot[0] > 0 ? 1 : -1);

	subnd(verror, driver->velocity.Pos, so->velocity.Pos, 6);
	subnd(aerror, driver->accel.Pos, so->acceleration, 3);
	variance_measure_add(&driver->pose_variance, error);

	FLT var[7];
	variance_measure_calc(&driver->pose_variance, var);
	SV_VERBOSE(110, "\tSimulation pose error     " Point7_format, LINMATH_VEC7_EXPAND(var));
	SV_VERBOSE(110, "\tSimulation velocity error " Point6_format, LINMATH_VEC6_EXPAND(verror));
	SV_VERBOSE(110, "\tSimulation acc error      " Point3_format, LINMATH_VEC3_EXPAND(aerror));
	bool pos_unsync = norm3d(p.Pos) > .1 || norm3d(p.Rot + 1) > .2;
	bool unsync = pos_unsync || normnd(verror, 6) > .1;
	if (unsync || ctx->log_level >= 500) {
		SV_VERBOSE(200, "Simulation diff:\t%+f\t%+f\t" SurvivePose_format, norm3d(p.Pos), norm3d(p.Rot + 1),
				   SURVIVE_POSE_EXPAND(p));

		SV_VERBOSE(200, "Simulation position " SurvivePose_format "\t", SURVIVE_POSE_EXPAND(driver->position));
		SV_VERBOSE(200, "Simulation velocity " SurviveVel_format "\t", SURVIVE_VELOCITY_EXPAND(driver->velocity));
		SV_VERBOSE(200, "Simulation acceleration " Point3_format "\t", LINMATH_VEC3_EXPAND(driver->accel.Pos));
		SV_VERBOSE(200, "Simulation bias         " Point3_format "\t", LINMATH_VEC3_EXPAND(driver->gyro_bias));

		SV_VERBOSE(200, "Object     position " SurvivePose_format "\t", SURVIVE_POSE_EXPAND(so->OutPoseIMU));
		SV_VERBOSE(200, "Object     velocity " SurviveVel_format "\t", SURVIVE_VELOCITY_EXPAND(so->velocity));
		if (unsync || pos_unsync) {
			SV_VERBOSE(200, "Simulation unsync");
		}
	}
}

static int simulator_close(struct SurviveContext *ctx, void *_driver) {
	SurviveDriverSimulator *driver = _driver;

	FLT var[7];
	variance_measure_calc(&driver->pose_variance, var);
	SV_VERBOSE(5, "Simulation info");
	SV_VERBOSE(5, "\tError         " Point7_format, LINMATH_VEC7_EXPAND(var));
	SV_VERBOSE(5, "\tTracker bias  " Point3_format, LINMATH_VEC3_EXPAND(driver->gyro_bias));

	SurviveDriverSimulator_detach_config(ctx, driver);
	free(driver);
	return 0;
}

cstring generate_simulated_object(FLT r, size_t sensor_ct) {
	cstring cfg = {0};
	cstring loc = {0}, nor_buf = {0};

	char buffer[1024] = {0};

	for (int i = 0; i < sensor_ct; i++) {
		FLT azi = rand();
		FLT pol = rand();
		LinmathVec3d normals, locations;
		normals[0] = locations[0] = r * cos(azi) * sin(pol);
		normals[1] = locations[1] = r * sin(azi) * sin(pol);
		normals[2] = locations[2] = fabs(r * cos(pol));
		normalize3d(normals, normals);

		snprintf(buffer, sizeof(buffer), "[%f, %f, %f],\n", locations[0], locations[1], locations[2]);
		str_append(&loc, buffer);

		snprintf(buffer, sizeof(buffer), "[%f, %f, %f],\n", normals[0], normals[1], normals[2]);
		str_append(&nor_buf, buffer);
	}
	nor_buf.d[nor_buf.length - 2] = 0;
	loc.d[loc.length - 2] = 0;

	FLT trackref_from_head[7] = {0, 0, 0, 1};
	FLT trackref_from_imu[7] = {0, 0, 0, 1};

	for (int i = 0; i < 7; i++) {
		trackref_from_head[i] = .1 * (trackref_from_head[i] / RAND_MAX - .5);
		trackref_from_imu[i] = .1 * (trackref_from_imu[i] / RAND_MAX - .5);
	}

	quatnormalize(trackref_from_head, trackref_from_head);
	quatnormalize(trackref_from_imu, trackref_from_imu);

	snprintf(buffer, sizeof(buffer),
			 "\"trackref_from_head\": [%f, %f, %f, %f, %f, %f, %f], \n"
			 "\"trackref_from_imu\": [%f, %f, %f, %f, %f, %f, %f], \n",
			 trackref_from_head[0], trackref_from_head[1], trackref_from_head[2], trackref_from_head[3],
			 trackref_from_head[4], trackref_from_head[5], trackref_from_head[6], trackref_from_imu[0],
			 trackref_from_imu[1], trackref_from_imu[2], trackref_from_imu[3], trackref_from_imu[4],
			 trackref_from_imu[5], trackref_from_imu[6]);

	str_append(&cfg, "{\n");
	str_append(&cfg, buffer);
	str_append(&cfg, "     \"lighthouse_config\": {\n");
	str_append(&cfg, "          \"modelNormals\": [\n");
	str_append(&cfg, nor_buf.d);
	str_free(&nor_buf);

	str_append(&cfg, "          ],\n");
	str_append(&cfg, "          \"modelPoints\": [\n");
	str_append(&cfg, loc.d);
	str_free(&loc);

	str_append(&cfg, "          ]\n");
	str_append(&cfg, "     }\n");
	str_append(&cfg, "}\n");

	return cfg;
}

SURVIVE_EXPORT SurviveObject *survive_create_simulation_device(SurviveContext *ctx, SurviveDriverSimulator *driver,
															   const char *device_name) {
	SurviveObject *device = survive_create_device(ctx, "SIM", driver, device_name, 0);
	device->sensor_ct = driver->settings.obj_sensors;

	device->head2imu.Rot[0] = 1;
	device->head2trackref.Rot[0] = 1;
	device->imu2trackref.Rot[0] = 1;

	FLT r = driver->settings.obj_radius;

	cstring cfg = generate_simulated_object(r, device->sensor_ct);

	SURVIVE_INVOKE_HOOK_SO(config, device, cfg.d, strlen(cfg.d));
	device->object_type = SURVIVE_OBJECT_TYPE_CONTROLLER;
	device->object_subtype = SURVIVE_OBJECT_SUBTYPE_WAND;
	device->timebase_hz = 48000000;
	device->imu_freq = 1000.0f;

	return device;
}

int DriverRegSimulator(SurviveContext *ctx) {
	SurviveDriverSimulator *sp = SV_CALLOC(sizeof(SurviveDriverSimulator));
	sp->ctx = ctx;
	ctx->poll_min_time_ms = 0;

	apply_initial_position(sp);

	SV_INFO("Setting up Simulator driver.");

	SurviveDriverSimulator_attach_config(ctx, sp);
	sp->settings.time_factor = linmath_max(survive_configf(ctx, "time-factor", SC_GET, 1.), .00001);

	sp->pose_variance.size = 7;

	for (int i = 0; i < 3; i++)
		sp->gyro_bias[i] = linmath_normrand(0, sp->gyro_bias_scale * sp->noise_scale);
	sp->scale_error = .97; // linmath_normrand(1, .05);
	int use_lh2 = sp->lh_version == 2;
	int max_lighthouses = use_lh2 ? 16 : 2;
	// Create a new SurviveObject...
	SurviveObject *device = survive_create_simulation_device(ctx, sp, "SM0");

	srand(42);

	FLT freq_per_channel[NUM_GEN2_LIGHTHOUSES] = {
		50.0521, 50.1567, 50.3673, 50.5796, 50.6864, 50.9014, 51.0096, 51.1182,
		51.2273, 51.6685, 52.2307, 52.6894, 52.9217, 53.2741, 53.7514, 54.1150,
	};

	if (ctx->activeLighthouses > max_lighthouses) {
		ctx->activeLighthouses = max_lighthouses;
	}
	for (int i = 0; i < ctx->activeLighthouses; i++) {
		sp->bsd[i] = ctx->bsd[i];
		if (!ctx->bsd[i].PositionSet) {
			sp->bsd[i].Pose = simulated_bsd[i].Pose;
		}

		ctx->bsd_map[ctx->bsd[i].mode] = i;

		sp->lhstates[i].start_time = (rand() / (FLT)RAND_MAX);

		assert(ctx->bsd[i].mode < NUM_GEN2_LIGHTHOUSES);

		sp->lhstates[i].period_s = 1. / freq_per_channel[ctx->bsd[i].mode];
	}

	BaseStationCal fcalNoise = {.phase = .1,
								.tilt = .002,
								.curve = .005,
								.gibpha = LINMATHPI,
								.gibmag = .01,
								.ogeephase = LINMATHPI,
								.ogeemag = .25};

	if (ctx->activeLighthouses == 0) {
		int lh_count = sizeof(simulated_bsd) / sizeof(simulated_bsd[0]);
		if (!use_lh2)
			lh_count = 2;

		for (int i = 0; i < lh_count; i++) {
			ctx->bsd[i] = simulated_bsd[i];

			for (int axis = 0; axis < 2; axis++) {
				for (int cal_idx = 0; cal_idx < sizeof(fcalNoise) / sizeof(FLT); cal_idx++) {
					((FLT *)(&ctx->bsd[i].fcal[axis]))[cal_idx] =
						linmath_rand(-((FLT *)&fcalNoise)[cal_idx], ((FLT *)&fcalNoise)[cal_idx]);
				}
			}
			ctx->activeLighthouses++;

			ctx->bsd_map[ctx->bsd[i].mode] = i;
			sp->lhstates[i].start_time = (rand() / (FLT)RAND_MAX);
			sp->lhstates[i].period_s = 1. / freq_per_channel[ctx->bsd[i].mode];

			sp->bsd[i] = ctx->bsd[i];
		}
	}

	FLT fcal_noise = sp->settings.fcal_noise;
	for (int i = 0; i < ctx->activeLighthouses; i++) {
		for (int axis = 0; axis < 2; axis++) {
			for (int cal_idx = 0; cal_idx < sizeof(fcalNoise) / sizeof(FLT); cal_idx++) {
				((FLT *)(&ctx->bsd[i].fcal[axis]))[cal_idx] +=
					fcal_noise * linmath_rand(-((FLT *)&fcalNoise)[cal_idx], ((FLT *)&fcalNoise)[cal_idx]);
			}
		}
	}

	// ctx->bsd[0].Pose = sp->bsd[0].Pose;
	// ctx->bsd[0].PositionSet = 1;

	sp->so = device;
	survive_add_object(ctx, device);
	sp->lh_version = use_lh2 ? 1 : 0;
	ctx->lh_version = sp->lh_version;
	ctx->lh_version_configed = ctx->lh_version;

	if (use_lh2) {
		survive_notify_gen2(device, "Simulator setup for lh2");
	} else {
		survive_notify_gen1(device, "Simulator setup for lh1");
	}

	sp->pose_fn = survive_install_imupose_fn(ctx, simulation_compare);
	sp->lh_fn = survive_install_lighthouse_pose_fn(ctx, simulation_lh_compare);
	survive_add_driver(ctx, sp, Simulator_poll, simulator_close);
	return 0;
}

REGISTER_LINKTIME(DriverRegSimulator)
