#include <libsurvive/survive.h>

#include <libsurvive/survive_optimizer.h>
#include <survive_reproject_gen2.h>

SURVIVE_EXPORT SurviveObject *survive_create_simulation_device(SurviveContext *ctx, void *driver,
															   const char *device_name);

static bool lighthouse_sensor_angle(SurviveObject *so, int lh, size_t idx, SurviveAngleReading ang, FLT sensor_var) {
	SurviveContext *ctx = so->ctx;
	FLT *pt = so->sensor_locations + idx * 3;

	LinmathVec3d ptInWorld;
	LinmathVec3d normalInWorld;
	ApplyPoseToPoint(ptInWorld, &so->OutPoseIMU, pt);
	SurvivePose world2lh = InvertPoseRtn(&ctx->bsd[lh].Pose);
	LinmathPoint3d ptInLh;
	ApplyPoseToPoint(ptInLh, &world2lh, ptInWorld);

	if (ptInLh[2] < 0) {
		LinmathVec3d dirLh;
		normalize3d(dirLh, ptInLh);
		scale3d(dirLh, dirLh, -1);

		quatrotatevector(normalInWorld, so->OutPoseIMU.Rot, so->sensor_normals + idx * 3);

		LinmathVec3d normalInLh;
		quatrotatevector(normalInLh, world2lh.Rot, normalInWorld);

		FLT facingness = dot3d(normalInLh, dirLh);
		if (facingness > 0) {
			if (ctx->lh_version == 0) {
				survive_reproject_xy(ctx->bsd[lh].fcal, ptInLh, ang);
			} else {
				survive_reproject_xy_gen2(ctx->bsd[lh].fcal, ptInLh, ang);
			}

			for (int i = 0; i < 2; i++) {
				ang[i] += linmath_normrand(0, sensor_var);
			}
			return true;
		}
	}
	return false;
}

void report_pose(SurviveContext *ctx, size_t idx, SurvivePose *pose, uint32_t rgb) {
	char buffer[32] = {0};
	snprintf(buffer, 31, "EST%d", (int)idx);

	// if(norm3d(pose->Pos) < 5)
	// survive_default_external_pose_process(ctx, buffer, pose);

	printf("%f SPHERE %s %f %d " Point3_format "\n", 0., buffer, .05, rgb, LINMATH_VEC3_EXPAND(pose->Pos));
}

void iteration_cb(struct survive_optimizer *opt_ctx, int m, int n, FLT *p, FLT *deviates, FLT **derivs) {
	SurvivePose *estimate = survive_optimizer_get_pose(opt_ctx);
	size_t idx = *(size_t *)opt_ctx->user;
	char buffer[32] = {0};
	snprintf(buffer, 31, "EST%d", (int)idx);

	SurvivePose pose = *estimate;
	quatfromaxisangle(pose.Rot, pose.Rot, norm3d(pose.Rot));
	if (norm3d(pose.Pos) < 5)
		report_pose(opt_ctx->sos[0]->ctx, idx, &pose, 0xff);
}
int main(int argc, char **argv) {
	SurviveContext *ctx = survive_init(argc, argv);
	survive_startup(ctx);

	ctx->activeLighthouses = 1;

	SurviveObject *device = survive_create_simulation_device(ctx, 0, "SM0");
	SurvivePose camera[] = {
		{.Pos = {-3, 0, 1}, .Rot = {-0.70710678118, 0, 0.70710678118, 0}},
		{.Pos = {0, 3, 1}, .Rot = {0.70710678118, -0.70710678118, 0, 0}},
	};

	size_t cam_cnt = 2;
	for (int i = 0; i < cam_cnt; i++) {
		ctx->bsd[i].Pose = camera[i];
		ctx->bsd[i].PositionSet = ctx->bsd[i].OOTXSet = 1;
		survive_default_lighthouse_pose_process(ctx, i, &camera[i]);
	}

	SurvivePose object = {.Rot = {1.}};
	survive_default_pose_process(device, 0, &object);

	survive_optimizer mpfitctx = {
		.reprojectModel = &survive_reproject_gen2_model,
		.poseLength = 1,
		.cameraLength = cam_cnt,
		.measurementsCnt = device->sensor_ct * 2 * cam_cnt,
		//.iteration_cb = iteration_cb
	};

	SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(mpfitctx, device);

	survive_optimizer_setup_cameras(&mpfitctx, ctx, true, true, true);

	survive_optimizer_setup_pose(&mpfitctx, &object, false, true);

	for (int lh = 0; lh < cam_cnt; lh++) {
		for (int i = 0; i < device->sensor_ct; i++) {
			SurviveAngleReading ang;
			if (lighthouse_sensor_angle(device, lh, i, ang, 0)) {
				for (int j = 0; j < 2; j++) {
					survive_optimizer_measurement *meas =
						survive_optimizer_emplace_meas(&mpfitctx, survive_optimizer_measurement_type_light);
					meas->light.sensor_idx = i;
					meas->light.object = 0;
					meas->light.axis = j;
					meas->light.lh = lh;
					meas->light.value = ang[j];
					meas->variance = 1;
				}
			}
		}
	}
	mpfitctx.cfg = survive_optimizer_precise_config();

	size_t idx = 0;
	mpfitctx.user = &idx;
	FLT inc = .25;
	FLT limits = 5;
	for (FLT xx = -limits; xx <= limits; xx += inc) {
		for (FLT yy = -limits; yy <= limits; yy += inc) {
			for (FLT zz = 0; zz <= limits; zz += inc) {
				idx++;
				struct mp_result_struct result = {0};
				SurvivePose *p = survive_optimizer_get_pose(&mpfitctx);
				p->Pos[0] = xx;
				p->Pos[1] = yy;
				p->Pos[2] = zz;
				quatcopy(p->Rot, LinmathQuat_Identity);

				SurvivePose orig = *p;

				survive_optimizer_run(&mpfitctx, &result, 0);

				if (norm3d(p->Pos) < .01) {
					// report_pose(ctx, idx, &orig, 0xFF00);
					// report_pose(ctx, idx, p);
				} else {
					report_pose(ctx, -idx, &orig, 0xFF00);
					report_pose(ctx, idx, p, 0xFF0000);
				}
			}
		}
	}

	return 0;
}
