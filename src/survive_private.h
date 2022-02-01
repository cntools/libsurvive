#pragma once

struct SurviveExternalPose {
	char name[32];
	SurvivePose pose;
};

struct SurviveContext_private {
	og_sema_t poll_sema;
	survive_run_time_fn runTimeFn;
	void *runTimeFnUser;
	double lastRunTime;

	double callbackStatsTimeBetween;
	double lastCallbackStats;

	struct SurviveExternalPose ExternalPoses[16];
	SurvivePose external2world;
};