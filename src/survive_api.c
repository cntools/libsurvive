#include "survive_api.h"
#include "inttypes.h"
#include "os_generic.h"
#include "stdio.h"
#include "survive.h"

struct SurviveSimpleObject {
	struct SurviveSimpleContext *actx;

	enum SurviveSimpleObject_type { SurviveSimpleObject_LIGHTHOUSE, SurviveSimpleObject_OBJECT } type;

	union {
		int lighthouse;
		struct SurviveObject* so;
	} data;

	char name[32];
	bool has_update;
};

struct SurviveSimpleContext {
	SurviveContext* ctx; 
	
	bool running;
	og_thread_t thread;
	og_mutex_t poll_mutex;

	size_t object_ct;
	struct SurviveSimpleObject objects[];
};

static void pose_fn(SurviveObject *so, uint32_t timecode, SurvivePose *pose) {
	struct SurviveSimpleContext *actx = so->ctx->user_ptr;
	OGLockMutex(actx->poll_mutex);
	survive_default_raw_pose_process(so, timecode, pose);

	intptr_t idx = (intptr_t)so->user_ptr;
	actx->objects[idx].has_update = true;
	OGUnlockMutex(actx->poll_mutex);
}
static void lh_fn(SurviveContext *ctx, uint8_t lighthouse, SurvivePose *lighthouse_pose,
	SurvivePose *object_pose) {
	struct SurviveSimpleContext *actx = ctx->user_ptr;
	OGLockMutex(actx->poll_mutex);
	survive_default_lighthouse_pose_process(ctx, lighthouse, lighthouse_pose, object_pose);

	actx->objects[lighthouse].has_update = true;

	OGUnlockMutex(actx->poll_mutex);
}

struct SurviveSimpleContext *survive_simple_init(int argc, char *const *argv) {
	SurviveContext* ctx = survive_init(argc, argv);
	if (ctx == 0)
		return 0;

	survive_startup(ctx);

	int object_ct = ctx->activeLighthouses + ctx->objs_ct;
	struct SurviveSimpleContext *actx =
		calloc(1, sizeof(struct SurviveSimpleContext) + sizeof(struct SurviveSimpleObject) * object_ct);
	actx->object_ct = object_ct;
	actx->ctx = ctx;
	actx->poll_mutex = OGCreateMutex();
	ctx->user_ptr = actx;
	intptr_t i = 0;
	for (i = 0; i < ctx->activeLighthouses; i++) {
		struct SurviveSimpleObject *obj = &actx->objects[i];
		obj->data.lighthouse = i;
		obj->type = SurviveSimpleObject_LIGHTHOUSE;
		obj->actx = actx;
		obj->has_update = ctx->bsd[i].PositionSet;
		snprintf(obj->name, 32, "LH%" PRIdPTR, i);
	}
	for (; i < object_ct; i++) {
		struct SurviveSimpleObject *obj = &actx->objects[i];
		int so_idx = i - ctx->activeLighthouses;
		obj->data.so = ctx->objs[so_idx];
		obj->type = SurviveSimpleObject_OBJECT;
		obj->actx = actx;
		obj->data.so->user_ptr = (void*)i;
		snprintf(obj->name, 32, "%s", obj->data.so->codename);
	}

	survive_install_pose_fn(ctx, pose_fn);
	survive_install_lighthouse_pose_fn(ctx, lh_fn);
	return actx;
}

int survive_simple_stop_thread(struct SurviveSimpleContext *actx) {
	actx->running = false;
	intptr_t error = (intptr_t)OGJoinThread(actx->thread);
	if (error != 0) {
		SurviveContext *ctx = actx->ctx;
		SV_INFO("Warning: Loop exited with error %" PRIdPTR, error);
	}
	return error;
}

void survive_simple_close(struct SurviveSimpleContext *actx) {
	if (actx->running) {
		survive_simple_stop_thread(actx);
	}

	survive_close(actx->ctx);
}

static inline void *__simple_thread(void *_actx) {
	struct SurviveSimpleContext *actx = _actx;
	intptr_t error = 0;
	while (actx->running && error == 0) {
		error = survive_poll(actx->ctx);
	}
	actx->running = false;
	return (void*)error; 
}
bool survive_simple_is_running(struct SurviveSimpleContext *actx) { return actx->running; }
void survive_simple_start_thread(struct SurviveSimpleContext *actx) {
	actx->running = true;
	actx->thread = OGCreateThread(__simple_thread, actx);
}

const struct SurviveSimpleObject *survive_simple_get_next_object(struct SurviveSimpleContext *actx,
																 const struct SurviveSimpleObject *curr) {
	const struct SurviveSimpleObject *next = curr + 1;
	if (next >= actx->objects + actx->object_ct)
		return 0;
	return next;
}

const struct SurviveSimpleObject *survive_simple_get_first_object(struct SurviveSimpleContext *actx) {
	return actx->objects;
}

const struct SurviveSimpleObject *survive_simple_get_next_updated(struct SurviveSimpleContext *actx) {
	for (int i = 0; i < actx->object_ct; i++) {
		if (actx->objects[i].has_update) {
			actx->objects[i].has_update = false;
			return &actx->objects[i];
		}
	}
	return 0;
}

uint32_t survive_simple_object_get_latest_pose(const struct SurviveSimpleObject *sao, SurvivePose *pose) {
	uint32_t timecode = 0;
	OGLockMutex(sao->actx->poll_mutex);

	switch (sao->type) {
	case SurviveSimpleObject_LIGHTHOUSE: {
		if(pose)
			*pose = sao->actx->ctx->bsd[sao->data.lighthouse].Pose;
		break;
	}
	case SurviveSimpleObject_OBJECT:
		if(pose) 
			*pose = sao->data.so->OutPose;
		timecode = sao->data.so->OutPose_timecode;
		break;
	}

	OGUnlockMutex(sao->actx->poll_mutex);
	return timecode;
}

const char *survive_simple_object_name(const SurviveSimpleObject *sao) { return sao->name; }
