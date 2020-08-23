#include <stdio.h>
#include <string.h>
#define SURVIVE_ENABLE_FULL_API

#include <complex.h>
#include <malloc.h>
#include <math.h>
#include <os_generic.h>
#include <survive.h>
#include <survive_api.h>

static void diff(FLT *out, const SurvivePose *a, const SurvivePose *b) {
	SurvivePose iB = InvertPoseRtn(b);
	SurvivePose nearId;
	ApplyPoseToPose(&nearId, a, &iB);
	out[0] = (1 - fabs(nearId.Rot[0]));
	out[1] = norm3d(nearId.Pos);
}

SurviveSimpleObject *get_test_version(SurviveSimpleContext *actx, const char *name) {
	if (strncmp(name, "replay_", strlen("replay_")) == 0) {
		return survive_simple_get_object(actx, name + strlen("replay_"));
	}
	return 0;
}

struct replay_ctx {
	int mismatched;
	pose_process_func pose_fn;
	external_pose_process_func external_pose_fn;
};

static bool check(SurviveSimpleContext *actx, const SurviveSimpleObject *sao, FLT max_pos_error, FLT max_rot_error) {
	FLT err[2] = {0};

	SurvivePose pose = {0};
	survive_simple_object_get_latest_pose(sao, &pose);

	SurviveSimpleObject *test_sao = get_test_version(actx, survive_simple_object_name(sao));
	if (test_sao == 0)
		return true;

	SurvivePose compare_pose = {0};
	survive_simple_object_get_latest_pose(test_sao, &compare_pose);

	if (quatiszero(compare_pose.Rot))
		return true;

	diff(err, &pose, &compare_pose);

	if (err[1] > max_pos_error || err[0] > max_rot_error) {
		fprintf(stderr, "%s: " SurvivePose_format "\n", survive_simple_object_name(sao), pose.Pos[0], pose.Pos[1],
				pose.Pos[2], pose.Rot[0], pose.Rot[1], pose.Rot[2], pose.Rot[3]);

		fprintf(stderr, "       %s: " SurvivePose_format " %f\t%f\n", survive_simple_object_name(test_sao),
				compare_pose.Pos[0], compare_pose.Pos[1], compare_pose.Pos[2], compare_pose.Rot[0], compare_pose.Rot[1],
				compare_pose.Rot[2], compare_pose.Rot[3], err[0], err[1]);

		fprintf(stderr, "TEST FAILED, %s deviates too much -- %f %f\n", survive_simple_object_name(sao), err[0],
				err[1]);
		return false;
	}
	return true;
}

#ifdef USE_FLOAT
FLT max_pos_error = .08, max_rot_error = .005;
#else
FLT max_pos_error = .08, max_rot_error = .001;
#endif

static void external_pose_fn(SurviveContext *ctx, const char *name, const SurvivePose *pose) {
	SurviveSimpleContext *actx = ctx->user_ptr;
	struct replay_ctx *rctx = survive_simple_get_user(actx);
	rctx->external_pose_fn(ctx, name, pose);

	struct SurviveSimpleObject *sao = survive_simple_get_object(actx, name);

	if (rctx->mismatched == 0 && !check(actx, sao, max_pos_error * 10., max_rot_error * 10.)) {
		rctx->mismatched++;
	}
}

static int test_path(const char *filename, int main_argc, char **main_argv) {
	int rtn = 0;

	char configPath[FILENAME_MAX] = {0};
	sprintf(configPath, "%s.json", filename);

	char *argv[] = {
		"",
		"--init-configfile",
		configPath,
		"--playback-replay-pose",
		"--playback",
		(char *)filename,
		"--playback-factor",
		"0",
		"--v",
		"100",
	};
	int argc = sizeof(argv) / sizeof(argv[0]);

	fprintf(stderr, "Test with: './src/test_cases/test_replays '%s'", filename);
	for (int i = 0; i < main_argc; i++) {
		fprintf(stderr, " %s", main_argv[i]);
	}
	fprintf(stderr, "'\n");

	fprintf(stderr, "Run  with: './survive-cli");
	for (int i = 0; i < sizeof(argv) / sizeof(argv[0]); i++) {
		fprintf(stderr, " %s", argv[i]);
	}
	fprintf(stderr, "'\n");
	int total_argc = argc + main_argc;
	char **total_argv = alloca(sizeof(char *) * total_argc);
	for (int i = 0; i < argc; i++) {
		total_argv[i] = argv[i];
	}
	for (int i = 0; i < main_argc; i++) {
		total_argv[i + argc] = main_argv[i];
	}

	struct replay_ctx rctx = {0};
	SurviveSimpleContext *actx = survive_simple_init(total_argc, total_argv);
	if (actx == 0) {
		return -1;
	}
	survive_simple_set_user(actx, &rctx);

	SurviveContext *ctx = survive_simple_get_ctx(actx);
	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		ctx->bsd[i].PositionSet = false;
	}

	SurvivePose originalLH[NUM_GEN2_LIGHTHOUSES] = {0};

	fprintf(stderr, "Ground truth LH poses:\n");
	uint32_t ref_lh = 0;
	for (int i = 0; i < ctx->activeLighthouses; i++) {
		SurvivePose pose = ctx->bsd[i].Pose;
		originalLH[i] = pose;
		ctx->bsd[i].PositionSet = 0;
		ctx->bsd[i].Pose = LinmathPose_Identity;
		fprintf(stderr, " LH%2d (%08x): " SurvivePose_format "\n", i, ctx->bsd[i].BaseStationID, pose.Pos[0],
				pose.Pos[1], pose.Pos[2], pose.Rot[0], pose.Rot[1], pose.Rot[2], pose.Rot[3]);
		if (fabs(pose.Pos[0]) < 1e-10) {
			ref_lh = ctx->bsd[i].BaseStationID;
		}
	}

	SV_INFO("Setting %08x as reference base station", ref_lh);
	survive_configi(ctx, "reference-basestation", SC_SET, ref_lh);

	SV_WARN(" =============== Starting thread =============== ")
	survive_simple_start_thread(actx);

	// rctx.pose_fn = survive_install_pose_fn(ctx, pose_fn);
	rctx.external_pose_fn = survive_install_external_pose_fn(ctx, external_pose_fn);

	while (survive_simple_is_running(actx)) {
		OGUSleep(10000);
	}

	for (const SurviveSimpleObject *it = survive_simple_get_first_object(actx); it != 0;
		 it = survive_simple_get_next_object(actx, it)) {
		SurvivePose pose;
		const char *name = survive_simple_object_name(it);

		survive_simple_object_get_latest_pose(it, &pose);

		if (!check(actx, it, max_pos_error, max_rot_error)) {
			rctx.mismatched++;
		}
	}

	for (int i = 0; i < ctx->activeLighthouses; i++) {
		SurvivePose pose = originalLH[i];
		fprintf(stderr, " LH%2d (%08x): " SurvivePose_format "\n", i, ctx->bsd[i].BaseStationID,
				SURVIVE_POSE_EXPAND(ctx->bsd[i].Pose));
		FLT err[2] = {0};
		diff(err, &pose, &ctx->bsd[i].Pose);
		fprintf(stderr, "                  " SurvivePose_format "\terr: %f %f\n", pose.Pos[0], pose.Pos[1], pose.Pos[2],
				pose.Rot[0], pose.Rot[1], pose.Rot[2], pose.Rot[3], err[0], err[1]);

		if (err[1] > max_pos_error || err[0] > max_rot_error) {
			fprintf(stderr, "TEST FAILED, LH%d deviates too much -- %f %f\n", i, err[0], err[1]);
			rctx.mismatched++;
		}

		if (ctx->bsd[i].OOTXSet == false || ctx->bsd[i].PositionSet == false) {
			fprintf(stderr, "TEST FAILED, LH%d was not solved for either ootx or position: %d %d\n", i,
					ctx->bsd[i].OOTXSet, ctx->bsd[i].PositionSet);
			rctx.mismatched++;
		}
	}

	if (ctx->currentError != 0) {
		fprintf(stderr, "TEST FAILED, survive ctx had error -- %d\n", ctx->currentError);
		rtn = -1;
	}

	survive_simple_close(actx);
	if (rctx.mismatched > 0)
		return -2;

	return rtn;
}

int main(int argc, char **argv) { return test_path(argv[1], argc - 2, argv + 2); }
