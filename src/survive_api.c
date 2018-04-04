#include "survive_api.h"
#include "os_generic.h"
#include "survive.h"
#include "stdio.h"

struct SurviveAsyncObject {
	struct SurviveAsyncContext* actx;
	
	enum SurviveAsyncObject_type {
		SurviveAsyncObject_LIGHTHOUSE,
		SurviveAsyncObject_OBJECT
	} type;

	union {
		int lighthosue;
		struct SurviveObject* so;
	} data;

	char name[32];
	bool has_update;
};

struct SurviveAsyncContext {
	SurviveContext* ctx; 
	
	bool running;
	og_thread_t thread;
	og_mutex_t poll_mutex;

	size_t object_ct;
	struct SurviveAsyncObject objects[]; 
};

static void pose_fn(SurviveObject *so, uint32_t timecode, SurvivePose *pose) {
	survive_default_raw_pose_process(so, timecode, pose);	

	struct SurviveAsyncContext* actx = so->ctx->user_ptr;
	int idx = (int)so->user_ptr;
	actx->objects[idx].has_update = true;
}
static void lh_fn(SurviveContext *ctx, uint8_t lighthouse, SurvivePose *lighthouse_pose,
	SurvivePose *object_pose) {
	survive_default_lighthouse_pose_process(ctx, lighthouse, lighthouse_pose, object_pose);

	struct SurviveAsyncContext* actx = ctx->user_ptr;
	actx->objects[lighthouse].has_update = true;
}

struct SurviveAsyncContext *survive_async_init(int argc, char *const *argv) {
	SurviveContext* ctx = survive_init(argc, argv);
	if (ctx == 0)
		return 0;

	survive_startup(ctx);

	int object_ct = ctx->activeLighthouses + ctx->objs_ct;
	struct SurviveAsyncContext * actx = calloc(1, sizeof(struct SurviveAsyncContext) + sizeof(struct SurviveAsyncObject) * object_ct );
	actx->object_ct = object_ct;
	actx->ctx = ctx;
	actx->poll_mutex = OGCreateMutex();
	ctx->user_ptr = actx;
	int i = 0;
	for (i = 0; i < ctx->activeLighthouses; i++) {
		struct SurviveAsyncObject* obj = &actx->objects[i];
		obj->data.lighthosue = i;
		obj->type = SurviveAsyncObject_LIGHTHOUSE;
		obj->actx = actx;
		obj->has_update = ctx->bsd[i].PositionSet;
		snprintf(obj->name, 32, "LH%d", i);
	}
	for (; i < object_ct; i++) {
		struct SurviveAsyncObject* obj = &actx->objects[i];
		int so_idx = i - ctx->activeLighthouses;
		obj->data.so = ctx->objs[so_idx];
		obj->type = SurviveAsyncObject_OBJECT;
		obj->actx = actx;
		obj->data.so->user_ptr = (void*)i;
		snprintf(obj->name, 32, "%s", obj->data.so->codename);
	}

	survive_install_pose_fn(ctx, pose_fn);
	survive_install_lighthouse_pose_fn(ctx, lh_fn);
	return actx;
}

void survive_async_close(struct SurviveAsyncContext* actx) {
	if (actx->running) {
		survive_async_stop_thread(actx);
	}

	survive_close(actx->ctx);
}

static inline void* __async_thread(void* _actx) {
	struct SurviveAsyncContext* actx = _actx; 
	int error = 0;
	while (actx->running && error == 0) {
		OGLockMutex(actx->poll_mutex);
		error = survive_poll(actx->ctx);
		OGUnlockMutex(actx->poll_mutex); 
	}
	actx->running = false;
	return (void*)error; 
}
bool survive_async_is_running(struct SurviveAsyncContext* actx) {
	return actx->running;
}
void survive_async_start_thread(struct SurviveAsyncContext* actx) {
	actx->running = true;
	actx->thread = OGCreateThread(__async_thread, actx);
}
int survive_async_stop_thread(struct SurviveAsyncContext* actx) {
	actx->running = false; 
	int error = (int)OGJoinThread(actx->thread);
	if (error != 0) {
		SurviveContext* ctx = actx->ctx;
		SV_INFO("Warning: Loope exited with error %d", error);
	}
	return error;
}

const struct SurviveAsyncObject* survive_async_get_next_tracked(struct SurviveAsyncContext* actx, const struct SurviveAsyncObject* curr) {
	const struct SurviveAsyncObject* next = curr + 1; 
	if (next >= actx->objects + actx->object_ct)
		return 0;
	return next;
}

const struct SurviveAsyncObject* survive_async_get_first_tracked(struct SurviveAsyncContext* actx) {
	return actx->objects;
}

const struct SurviveAsyncObject* survive_async_get_next_updated(struct SurviveAsyncContext* actx) {
	for (int i = 0; i < actx->object_ct; i++) {
		if (actx->objects[i].has_update) {
			actx->objects[i].has_update = false;
			return &actx->objects[i];
		}
	}
	return 0;
}

uint32_t survive_async_object_get_latest_pose(const struct SurviveAsyncObject* sao, SurvivePose* pose) {
	uint32_t timecode = 0;
	OGLockMutex(sao->actx->poll_mutex);

	switch (sao->type) {
	case SurviveAsyncObject_LIGHTHOUSE: {
		if(pose)
			*pose = sao->actx->ctx->bsd[sao->data.lighthosue].Pose;
		break;
	}
	case SurviveAsyncObject_OBJECT:
		if(pose) 
			*pose = sao->data.so->OutPose;
		timecode = sao->data.so->OutPose_timecode;
		break;
	}

	OGUnlockMutex(sao->actx->poll_mutex);
	return timecode;
}

const char * survive_async_object_name(const SurviveAsyncObject * sao)
{
	return sao->name;
}
