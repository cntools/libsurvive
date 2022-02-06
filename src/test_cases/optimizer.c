#include "survive.h"

#include "../generated/kalman_kinematics.gen.h"
#include "survive_optimizer.h"
#include "test_case.h"

survive_optimizer_settings settings = {
	.optimize_scale_threshold = -1,
};

static SurvivePose run(survive_optimizer* mpfitctx, SurviveKalmanModel* mdl, const FLT* points, size_t points_cnt, mp_result* result, CnMat* R, const SurviveVelocity* vel) {
	mpfitctx->ptsLength = points_cnt;
	SURVIVE_OPTIMIZER_SETUP_STACK_BUFFERS(*mpfitctx, 0);

	quatnormalize(mdl->Pose.Rot, mdl->Pose.Rot);
	SurvivePose lh_pose = {
		.Pos = { 0, 0, -5 },
		.Rot = { 1 }
	};
	SurvivePose ilh = InvertPoseRtn(&lh_pose);

	BaseStationCal bcal[2] = { 0 };

	SurvivePose orig = mdl->Pose;//{ .Rot = { 1 } };
	survive_optimizer_setup_pose(mpfitctx, &orig, false, 1);
	survive_optimizer_setup_camera(mpfitctx, 0, &ilh, true, 1);
	survive_optimizer_parameter * pt_params = survive_optimizer_emplace_params(mpfitctx, survive_optimizer_parameter_obj_points, points_cnt);
	memcpy(pt_params->p, points, points_cnt * sizeof(FLT) * 3);

	survive_optimizer_parameter * bsd_params = survive_optimizer_emplace_params(mpfitctx, survive_optimizer_parameter_camera_parameters, 1);
	memset(bsd_params->p, 0, bsd_params->size * sizeof(FLT));


	if(!mpfitctx->disableVelocity){
		SurviveVelocity *velocity = survive_optimizer_get_velocity(mpfitctx);
		*velocity = vel ? *vel : mdl->Velocity;
	}

	FLT t = 0;
	for(int i = 0;i < 2;i++) {
		t += .1;
		SurviveKalmanModel tmp = {0};
		SurviveKalmanModelPredict(&tmp, .1, mdl);
		*mdl = tmp;
		quatnormalize(mdl->Pose.Rot, mdl->Pose.Rot);
		for(int j = 0;j < points_cnt;j++) {
			survive_optimizer_measurement *meas =
				survive_optimizer_emplace_meas(mpfitctx, survive_optimizer_measurement_type_light);
			meas->variance = 1e-4;
			meas->time = t;
			meas->light.sensor_idx = j;
			meas->light.axis = i & 1;

			FLT out[2];
			survive_reproject_full(bcal, &lh_pose, &mdl->Pose, &points[j*3], out);
			meas->light.value = out[i & 1];
		}
	}
	mpfitctx->timecode = t;
	SurvivePose *opt_pose = survive_optimizer_get_pose(mpfitctx);
	//*opt_pose = mdl.Pose;

	survive_optimizer_run(mpfitctx, result, R);
	return *opt_pose;
}

FLT points[] = {
	-.1, -.1, 0,
	-.1, +.1, 0,
	+.1, -.1, 0,
	+.1, +.1, 0,
	0, 0, .1,
	0, 0, -.1
};


static SurvivePose poseDiff(const SurvivePose *a, const SurvivePose *b) {
	if (quatiszero(a->Rot) && quatiszero(b->Rot)) {
		return (SurvivePose) { 0 };
	}

	SurvivePose iB = InvertPoseRtn(b);
	SurvivePose nearId;
	ApplyPoseToPose(&nearId, a, &iB);
	return nearId;
}

survive_optimizer default_optimizer() {
	survive_optimizer mpfitctx = {
		.settings = &settings,
		.reprojectModel = &survive_reproject_gen1_model,
		.poseLength = 1,
		.cameraLength = 1,
		.objectUpVectorVariance = -1,
		.disableVelocity = true,
		.cfg = survive_optimizer_precise_config(),
		.user = 0
	};
	return mpfitctx;
}

bool verify_R(CnMat* R, SurvivePose* gt, SurvivePose* modeled) {
	FLT v[7];
	cn_get_diag(R, v, 7);
	for(int i = 0;i < 7;i++) v[i] = sqrt(v[i]);

	SurvivePose d = poseDiff(gt, modeled);
	d.Rot[0] = fabs(1 - fabs(d.Rot[0]));
	FLT* dd = &d.Pos[0];
	for(int i = 0;i < 7;i++) {
		if (fabs(dd[i]) > (10 * v[i] + 1e-6))
			return false;
	}

	return true;
}

TEST(Optimizer, Simple) {
	survive_optimizer mpfitctx = default_optimizer();

	SurviveKalmanModel mdl = {.Pose = {.Rot = {1, 1, 1, 1}},
							  .Velocity = {.Pos = {0, 0, .1}, .AxisAngleRot = {0, 0, .1}},
							  .IMUBias = {
								  .IMUCorrection = {1},
								  .AccScale = 1,
							  }};
	CN_CREATE_STACK_MAT(R, 7, 7);

	mp_result results = {0};
	SurvivePose output = run(&mpfitctx, &mdl, points, SURVIVE_ARRAY_SIZE(points) / 3, &results, &R, 0);

	assert(results.bestnorm < 1e1);
	assert(verify_R(&R, &mdl.Pose, &output));

	return  0;
}

TEST(Optimizer, Velocity) {
	survive_optimizer mpfitctx = default_optimizer();
	mpfitctx.disableVelocity = false;

	SurviveKalmanModel mdl = {.Pose = {.Rot = {1, 1, 1, 1}},
							  .Velocity = {.Pos = {0, 0, .1}, .AxisAngleRot = {0, 0, .1}},
							  .IMUBias = {
								  .IMUCorrection = {1},
								  .AccScale = 1,
							  }};
	CN_CREATE_STACK_MAT(R, 7, 7);

	mp_result results = {0};
	SurvivePose output = run(&mpfitctx, &mdl, points, SURVIVE_ARRAY_SIZE(points) / 3, &results, &R, 0);
	assert(results.bestnorm < 1e-5);

	assert(verify_R(&R, &mdl.Pose, &output));
	return  0;
}


FLT side_points[] = {
	-1e-3, -.1, 0,
	-1e-3, +.1, 0,
	+1e-3, -.1, 0,
	+1e-3, +.1, 0,
	0, 0, .1,
	0, 0, 0
};


TEST(Optimizer, SimpleSide) {
	survive_optimizer mpfitctx = default_optimizer();

	SurviveKalmanModel mdl = {.Pose = {.Rot = {1, 1, 1, 1}},
							  .Velocity = {.Pos = {0, 0, .1}, .AxisAngleRot = {0, 0, .1}},
							  .IMUBias = {
								  .IMUCorrection = {1},
								  .AccScale = 1,
							  }};
	CN_CREATE_STACK_MAT(R, 7, 7);

	mp_result results = {0};
	SurvivePose output = run(&mpfitctx, &mdl, side_points, SURVIVE_ARRAY_SIZE(side_points) / 3, &results, &R, 0);

	//assert(verify_R(&R, &mdl.Pose, &output));

	return  0;
}


TEST(Optimizer, VelocitySide) {
	survive_optimizer mpfitctx = default_optimizer();
	mpfitctx.disableVelocity = false;

	SurviveKalmanModel mdl = {.Pose = {.Rot = {1, 1, 1, 1}},
							  .Velocity = {.Pos = {0, 0, .1}, .AxisAngleRot = {0, 0, .1}},
							  .IMUBias = {
								  .IMUCorrection = {1},
								  .AccScale = 1,
							  }};
	CN_CREATE_STACK_MAT(R, 7, 7);

	SurviveVelocity velocity = { .Pos = { 0, 0, -.1 }, .AxisAngleRot = { 0, 0, -.1 } };
	mp_result results = {0};
	SurvivePose output = run(&mpfitctx, &mdl, side_points, SURVIVE_ARRAY_SIZE(side_points) / 3, &results, &R, &velocity);
	SurviveVelocity v1 = *survive_optimizer_get_velocity(&mpfitctx);
	// assert(verify_R(&R, &mdl.Pose, &output));

	return 0;
}

FLT under_specified[] = {
	-.1, -.5, 0, +.1, -.5, 0, 0, .5, 0,
};

TEST(Optimizer, SimpleUnderspecced) {
	survive_optimizer mpfitctx = default_optimizer();

	SurviveKalmanModel mdl = {.Pose = {.Rot = {1, 0, 1, 0}},
							  //.Velocity = { .Pos = { 0, 0, .1}, .AxisAngleRot = {0, 0, .1}},
							  .IMUBias = {
								  .IMUCorrection = {1},
								  .AccScale = 1,
							  }};
	CN_CREATE_STACK_MAT(R, 7, 7);

	mp_result results = {0};
	SurvivePose output =
		run(&mpfitctx, &mdl, under_specified, SURVIVE_ARRAY_SIZE(under_specified) / 3, &results, &R, 0);

	assert(verify_R(&R, &mdl.Pose, &output));

	return  0;
}
