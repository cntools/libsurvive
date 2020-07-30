#include <stdio.h>
#include <string.h>
#define SURVIVE_ENABLE_FULL_API

#include <complex.h>
#include <malloc.h>
#include <math.h>
#include <os_generic.h>
#include <survive.h>
#include <survive_api.h>

static void diff(double *out, const SurvivePose *a, const SurvivePose *b) {
	SurvivePose iB = InvertPoseRtn(b);
	SurvivePose nearId;
	ApplyPoseToPose(&nearId, a, &iB);
	out[0] = (1 - fabs(nearId.Rot[0]));
	out[1] = norm3d(nearId.Pos);
}

static int test_path(const char *name, int main_argc, char **main_argv) {
	int rtn = 0;
	double max_pos_error = .08, max_rot_error = .001;
	char configPath[FILENAME_MAX] = {0};
	sprintf(configPath, "%s.json", name);

	char *playbackFlag = strstr(name, "pcap") ? "--usbmon-playback" : "--playback";

	char *argv[] = {"",
					"--init-configfile",
					configPath,
					"--playback-replay-pose",
					playbackFlag,
					(char *)name,
					"--playback-factor",
					"0",
					"--v",
					"100"};
	int argc = sizeof(argv) / sizeof(argv[0]);

	fprintf(stderr, "Run with: './survive-cli");
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

	SurviveSimpleContext *actx = survive_simple_init(total_argc, total_argv);
	SurviveContext *ctx = survive_simple_get_ctx(actx);
	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		ctx->bsd[i].PositionSet = false;
	}

	SurvivePose originalLH[NUM_GEN2_LIGHTHOUSES] = {0};

	printf("Ground truth LH poses:\n");
	for (int i = 0; i < ctx->activeLighthouses; i++) {
		SurvivePose pose = ctx->bsd[i].Pose;
		originalLH[i] = pose;
		ctx->bsd[i].PositionSet = 0;
		ctx->bsd[i].Pose = LinmathPose_Identity;
		printf(" LH%2d: " SurvivePose_format "\n", i, pose.Pos[0], pose.Pos[1], pose.Pos[2], pose.Rot[0], pose.Rot[1],
			   pose.Rot[2], pose.Rot[3]);
	}

	survive_simple_start_thread(actx);

	while (survive_simple_is_running(actx)) {
		OGUSleep(10000);
	}

	for (const SurviveSimpleObject *it = survive_simple_get_first_object(actx); it != 0;
		 it = survive_simple_get_next_object(actx, it)) {
		SurvivePose pose;
		const char *name = survive_simple_object_name(it);
		uint32_t timecode = survive_simple_object_get_latest_pose(it, &pose);

		if (strncmp(name, "replay_", strlen("replay_")) == 0) {
			printf("%s: " SurvivePose_format "\n", survive_simple_object_name(it), pose.Pos[0], pose.Pos[1],
				   pose.Pos[2], pose.Rot[0], pose.Rot[1], pose.Rot[2], pose.Rot[3]);

			for (const SurviveSimpleObject *it2 = survive_simple_get_first_object(actx); it2 != 0;
				 it2 = survive_simple_get_next_object(actx, it2)) {

				const char *name2 = survive_simple_object_name(it2);
				if (strcmp(name2, name + strlen("replay_")) == 0) {
					SurvivePose pose2;
					survive_simple_object_get_latest_pose(it2, &pose2);
					double err[2] = {0};
					diff(err, &pose, &pose2);
					printf("       %s: " SurvivePose_format " %f\t%f\n", survive_simple_object_name(it2), pose2.Pos[0],
						   pose2.Pos[1], pose2.Pos[2], pose2.Rot[0], pose2.Rot[1], pose2.Rot[2], pose2.Rot[3], err[0],
						   err[1]);
					if (err[1] > max_pos_error || err[0] > max_rot_error) {
						fprintf(stderr, "TEST FAILED, %s deviates too much -- %f %f\n", survive_simple_object_name(it2),
								err[0], err[1]);
						rtn = -1;
					}
				}
			}
		}
	}

	for (int i = 0; i < ctx->activeLighthouses; i++) {
		SurvivePose pose = originalLH[i];
		printf(" LH%2d: " SurvivePose_format "\n", i, SURVIVE_POSE_EXPAND(ctx->bsd[i].Pose));
		double err[2] = {0};
		diff(err, &pose, &ctx->bsd[i].Pose);
		printf("       " SurvivePose_format "\terr: %f %f\n", pose.Pos[0], pose.Pos[1], pose.Pos[2], pose.Rot[0],
			   pose.Rot[1], pose.Rot[2], pose.Rot[3], err[0], err[1]);

		if (err[1] > max_pos_error || err[0] > max_rot_error) {
			fprintf(stderr, "TEST FAILED, LH%d deviates too much -- %f %f\n", i, err[0], err[1]);
			rtn = -1;
		}

		if (ctx->bsd[i].OOTXSet == false || ctx->bsd[i].PositionSet == false) {
			fprintf(stderr, "TEST FAILED, LH%d was not solved for either ootx or position: %d %d\n", i,
					ctx->bsd[i].OOTXSet, ctx->bsd[i].PositionSet);
			rtn = -1;
		}
	}

	if (ctx->currentError != 0) {
		fprintf(stderr, "TEST FAILED, survive ctx had error -- %d\n", ctx->currentError);
		rtn = -1;
	}

	survive_simple_close(actx);
	return rtn;
}

int main(int argc, char **argv) { return test_path(argv[1], argc - 2, argv + 2); }
