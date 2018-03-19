#ifndef USE_DOUBLE
#define FLT double
#define USE_DOUBLE
#endif

#include <sba/sba.h>

#include "poser.h"
#include <survive.h>

#include "assert.h"
#include "linmath.h"
#include "string.h"
#include "survive_config.h"
#include "survive_reproject.h"

#include "math.h"

typedef struct {
	survive_calibration_config calibration_config;
	PoserData *pdfs;
	SurviveObject *so;
} sba_context;

void metric_function(int j, int i, double *aj, double *xij, void *adata) {
	sba_context *ctx = (sba_context *)(adata);
	SurviveObject *so = ctx->so;

	SurvivePose obj2world = so->OutPose;
	FLT sensorInWorld[3] = {};
	ApplyPoseToPoint(sensorInWorld, obj2world.Pos, &so->sensor_locations[i * 3]);
	survive_reproject_from_pose_with_config(so->ctx, &ctx->calibration_config, j, (SurvivePose *)aj, sensorInWorld,
											xij);
}

size_t construct_input(const SurviveObject *so, PoserDataFullScene *pdfs, char *vmask, double *meas) {
	size_t measCount = 0;
	size_t size = so->sensor_ct * NUM_LIGHTHOUSES; // One set per lighthouse
	for (size_t sensor = 0; sensor < so->sensor_ct; sensor++) {
		for (size_t lh = 0; lh < 2; lh++) {
			FLT *l = pdfs->lengths[sensor][lh];
			if (l[0] < 0 || l[1] < 0) {
				vmask[sensor * NUM_LIGHTHOUSES + lh] = 0;
				continue;
			}

			double *angles = pdfs->angles[sensor][lh];
			vmask[sensor * NUM_LIGHTHOUSES + lh] = 1;

			meas[measCount++] = angles[0];
			meas[measCount++] = angles[1];
		}
	}
	return measCount;
}

size_t construct_input_from_scene(const SurviveObject *so, PoserDataLight *pdl, SurviveSensorActivations *scene,
								  char *vmask, double *meas) {
	size_t rtn = 0;

	for (size_t sensor = 0; sensor < so->sensor_ct; sensor++) {
		for (size_t lh = 0; lh < 2; lh++) {
			if (SurviveSensorActivations_isPairValid(scene, SurviveSensorActivations_default_tolerance, pdl->timecode,
													 sensor, lh)) {
				double *a = scene->angles[sensor][lh];
				vmask[sensor * NUM_LIGHTHOUSES + lh] = 1;
				meas[rtn++] = a[0];
				meas[rtn++] = a[1];
			} else {
				vmask[sensor * NUM_LIGHTHOUSES + lh] = 0;
			}
		}
	}
	return rtn;
}

void sba_set_cameras(SurviveObject *so, uint8_t lighthouse, SurvivePose *pose, void *user) {
	SurvivePose *poses = (SurvivePose *)(user);
	poses[lighthouse] = *pose;
}

typedef struct {
	bool hasInfo;
	SurvivePose poses;
} sba_set_position_t;

void sba_set_position(SurviveObject *so, uint8_t lighthouse, SurvivePose *new_pose, void *_user) {
	sba_set_position_t *user = _user;
	assert(user->hasInfo == false);
	user->hasInfo = 1;
	user->poses = *new_pose;
}
void *GetDriver(const char *name);

void str_metric_function(int j, int i, double *bi, double *xij, void *adata) {
	SurvivePose obj = *(SurvivePose *)bi;
	int sensor_idx = j >> 1;
	int lh = j & 1;

	sba_context *ctx = (sba_context *)(adata);
	SurviveObject *so = ctx->so;

	assert(lh < 2);
	assert(sensor_idx < so->sensor_ct);

	quatnormalize(obj.Rot, obj.Rot);
	FLT xyz[3];
	ApplyPoseToPoint(xyz, obj.Pos, &so->sensor_locations[sensor_idx * 3]);

	// std::cerr << "Processing " << sensor_idx << ", " << lh << std::endl;
	SurvivePose *camera = &so->ctx->bsd[lh].Pose;
	survive_reproject_from_pose_with_config(so->ctx, &ctx->calibration_config, lh, camera, xyz, xij);
}

// Optimizes for object assuming given LH
static double run_sba_find_3d_structure(survive_calibration_config options, PoserDataLight *pdl, SurviveObject *so,
										SurviveSensorActivations *scene, int max_iterations /* = 50*/,
										double max_reproj_error /* = 0.005*/) {
	double *covx = 0;

	char *vmask = malloc(sizeof(char) * so->sensor_ct * NUM_LIGHTHOUSES);
	double *meas = malloc(sizeof(double) * 2 * so->sensor_ct * NUM_LIGHTHOUSES);
	size_t meas_size = construct_input_from_scene(so, pdl, scene, vmask, meas);

	static int failure_count = 500;
	if (so->ctx->bsd[0].PositionSet == 0 || so->ctx->bsd[1].PositionSet == 0 || meas_size < 7) {
		if (so->ctx->bsd[0].PositionSet && so->ctx->bsd[1].PositionSet && failure_count++ == 500) {
			SurviveContext *ctx = so->ctx;
			SV_INFO("Can't solve for position with just %lu measurements", meas_size);
			failure_count = 0;
		}
		free(vmask);
		free(meas);
		return -1;
	}
	failure_count = 0;

	SurvivePose soLocation = so->OutPose;
	bool currentPositionValid = quatmagnitude(&soLocation.Rot[0]) != 0;

	{
		const char *subposer = config_read_str(so->ctx->global_config_values, "SBASeedPoser", "PoserEPNP");
		PoserCB driver = (PoserCB)GetDriver(subposer);
		SurviveContext *ctx = so->ctx;
		if (driver) {
			PoserData hdr = pdl->hdr;
			memset(&pdl->hdr, 0, sizeof(pdl->hdr)); // Clear callback functions
			pdl->hdr.pt = hdr.pt;
			pdl->hdr.rawposeproc = sba_set_position;

			sba_set_position_t locations = {};
			pdl->hdr.userdata = &locations;
			driver(so, &pdl->hdr);
			pdl->hdr = hdr;

			if (locations.hasInfo == false) {
				free(vmask);
				free(meas);

				return -1;
			} else if (locations.hasInfo) {
				soLocation = locations.poses;
			}
		} else {
			SV_INFO("Not using a seed poser for SBA; results will likely be way off");
		}
	}

	double opts[SBA_OPTSSZ] = {};
	double info[SBA_INFOSZ] = {};

	sba_context ctx = {options, &pdl->hdr, so};

	opts[0] = SBA_INIT_MU;
	opts[1] = SBA_STOP_THRESH;
	opts[2] = SBA_STOP_THRESH;
	opts[3] = SBA_STOP_THRESH;
	opts[3] = SBA_STOP_THRESH; // max_reproj_error * meas.size();
	opts[4] = 0.0;

	int status = sba_str_levmar(1, // Number of 3d points
								0, // Number of 3d points to fix in spot
								NUM_LIGHTHOUSES * so->sensor_ct, vmask,
								soLocation.Pos, // Reads as the full pose though
								7,				// pnp -- SurvivePose
								meas,			// x* -- measurement data
								0,				// cov data
								2,				// mnp -- 2 points per image
								str_metric_function,
								0,				// jacobia of metric_func
								&ctx,			// user data
								max_iterations, // Max iterations
								0,				// verbosity
								opts,			// options
								info);			// info

	if (status > 0) {
		quatnormalize(soLocation.Rot, soLocation.Rot);
		PoserData_poser_raw_pose_func(&pdl->hdr, so, 1, &soLocation);
	}

	{
		SurviveContext *ctx = so->ctx;
		// Docs say info[0] should be divided by meas; I don't buy it really...
		static int cnt = 0;
		if (cnt++ > 1000 || meas_size < 8) {
			SV_INFO("%f original reproj error for %lu meas", (info[0] / meas_size * 2), meas_size);
			SV_INFO("%f cur reproj error", (info[1] / meas_size * 2));
			cnt = 0;
		}
	}

	free(vmask);
	free(meas);

	return info[1] / meas_size * 2;
}

// Optimizes for LH position assuming object is posed at 0
static double run_sba(survive_calibration_config options, PoserDataFullScene *pdfs, SurviveObject *so,
					  int max_iterations /* = 50*/, double max_reproj_error /* = 0.005*/) {
	double *covx = 0;

	char *vmask = malloc(sizeof(char) * so->sensor_ct * NUM_LIGHTHOUSES);
	double *meas = malloc(sizeof(double) * 2 * so->sensor_ct * NUM_LIGHTHOUSES);
	size_t meas_size = construct_input(so, pdfs, vmask, meas);

	SurvivePose camera_params[2] = {so->ctx->bsd[0].Pose, so->ctx->bsd[1].Pose};

	if (true || so->ctx->bsd[0].PositionSet == 0 || so->ctx->bsd[1].PositionSet == 0) {
		const char *subposer = config_read_str(so->ctx->global_config_values, "SBASeedPoser", "PoserEPNP");
		PoserCB driver = (PoserCB)GetDriver(subposer);
		SurviveContext *ctx = so->ctx;
		if (driver) {
			SV_INFO("Using %s seed poser for SBA", subposer);
			PoserData hdr = pdfs->hdr;
			memset(&pdfs->hdr, 0, sizeof(pdfs->hdr)); // Clear callback functions
			pdfs->hdr.pt = hdr.pt;
			pdfs->hdr.lighthouseposeproc = sba_set_cameras;
			pdfs->hdr.userdata = camera_params;
			driver(so, &pdfs->hdr);
			pdfs->hdr = hdr;
		} else {
			SV_INFO("Not using a seed poser for SBA; results will likely be way off");
			for (int i = 0; i < 2; i++) {
				so->ctx->bsd[i].Pose = (SurvivePose){};
				so->ctx->bsd[i].Pose.Rot[0] = 1.;
			}
		}
		// opencv_solver_poser_cb(so, (PoserData *)pdfs);
		// PoserCharlesSlow(so, (PoserData *)pdfs);
	}

	double opts[SBA_OPTSSZ] = {};
	double info[SBA_INFOSZ] = {};

	sba_context ctx = {options, &pdfs->hdr, so};

	opts[0] = SBA_INIT_MU;
	opts[1] = SBA_STOP_THRESH;
	opts[2] = SBA_STOP_THRESH;
	opts[3] = SBA_STOP_THRESH;
	opts[3] = SBA_STOP_THRESH; // max_reproj_error * meas.size();
	opts[4] = 0.0;

	int status = sba_mot_levmar(so->sensor_ct,						  // number of 3d points
								NUM_LIGHTHOUSES,					  // Number of cameras -- 2 lighthouses
								0,									  // Number of cameras to not modify
								vmask,								  // boolean vis mask
								(double *)&camera_params[0],		  // camera parameters
								sizeof(SurvivePose) / sizeof(double), // The number of floats that are in a camera param
								meas,								  // 2d points for 3d objs
								covx, // covariance of measurement. Null sets to identity
								2,	// 2 points per image
								metric_function,
								0,				// jacobia of metric_func
								&ctx,			// user data
								max_iterations, // Max iterations
								0,				// verbosity
								opts,			// options
								info);			// info

	if (status >= 0) {
		PoserData_lighthouse_pose_func(&pdfs->hdr, so, 0, &camera_params[0]);
		PoserData_lighthouse_pose_func(&pdfs->hdr, so, 1, &camera_params[1]);
	}
	// Docs say info[0] should be divided by meas; I don't buy it really...
	// std::cerr << info[0] / meas.size() * 2 << " original reproj error" << std::endl;

	free(vmask);
	free(meas);

	{
		SurviveContext *ctx = so->ctx;
		// Docs say info[0] should be divided by meas; I don't buy it really...
		SV_INFO("%f original reproj error for %lu meas", (info[0] / meas_size * 2), meas_size);
		SV_INFO("%f cur reproj error", (info[1] / meas_size * 2));
	}

	return info[1] / meas_size * 2;
}

int PoserSBA(SurviveObject *so, PoserData *pd) {
	switch (pd->pt) {
	case POSERDATA_LIGHT: {
		SurviveSensorActivations *scene = &so->activations;

		PoserDataLight *lightData = (PoserDataLight *)pd;

		survive_calibration_config config = *survive_calibration_default_config();
		FLT error = run_sba_find_3d_structure(config, lightData, so, scene, 50, .5);
		return 0;
	}
	case POSERDATA_FULL_SCENE: {
		PoserDataFullScene *pdfs = (PoserDataFullScene *)(pd);
		survive_calibration_config config = *survive_calibration_default_config();
		// std::cerr << "Running sba with " << config << std::endl;
		double error = run_sba(config, pdfs, so, 50, .005);
		// std::cerr << "Average reproj error: " << error << std::endl;
		return 0;
	}
	default: {
		const char *subposer = config_read_str(so->ctx->global_config_values, "SBASeedPoser", "PoserEPNP");
		PoserCB driver = (PoserCB)GetDriver(subposer);
		if (driver) {
			return driver(so, pd);
		}
		break;
	}
	}
	return -1;
}

REGISTER_LINKTIME(PoserSBA);
