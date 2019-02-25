#include "survive_api.h"
#include "inttypes.h"
#include "os_generic.h"
#include "stdio.h"
#include "string.h"
#include "survive.h"

struct SurviveExternalObject {
	SurvivePose pose;
};

struct SurviveLighthouseData {
	int lighthouse;
	char serial_number[16];
};

struct SurviveSimpleObject {
	struct SurviveSimpleContext *actx;

	enum SurviveSimpleObject_type {
		SurviveSimpleObject_LIGHTHOUSE,
		SurviveSimpleObject_OBJECT,
		SurviveSimpleObject_EXTERNAL
	} type;

	union {
		struct SurviveLighthouseData lh;
		struct SurviveObject *so;
		struct SurviveExternalObject seo;
	} data;

	char name[32];
	bool has_update;
};

struct SurviveSimpleContext {
	SurviveContext* ctx; 
	
	bool running;
	og_thread_t thread;
	og_mutex_t poll_mutex;

	size_t external_object_ct;
	struct SurviveSimpleObject *external_objects;

	size_t object_ct;
	struct SurviveSimpleObject objects[];
};

SurviveSimpleObject *find_or_create_external(struct SurviveSimpleContext *actx, const char *name) {
	for (int i = 0; i < actx->external_object_ct; i++) {
		struct SurviveSimpleObject *so = &actx->external_objects[i];
		if (strncmp(name, so->name, 32) == 0) {
			return so;
		}
	}

	actx->external_objects =
		realloc(actx->external_objects, sizeof(struct SurviveSimpleObject) * (actx->external_object_ct + 1));
	struct SurviveSimpleObject *so = &actx->external_objects[actx->external_object_ct++];
	memset(so, 0, sizeof(struct SurviveSimpleObject));
	so->type = SurviveSimpleObject_EXTERNAL;
	so->actx = actx;
	strncpy(so->name, name, 32);
	return so;
}

static void external_pose_fn(SurviveContext *ctx, const char *name, const SurvivePose *pose) {
	struct SurviveSimpleContext *actx = ctx->user_ptr;
	OGLockMutex(actx->poll_mutex);
	survive_default_external_pose_process(ctx, name, pose);

	struct SurviveSimpleObject *so = find_or_create_external(actx, name);
	so->has_update = true;
	so->data.seo.pose = *pose;
	OGUnlockMutex(actx->poll_mutex);
}
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
		obj->data.lh.lighthouse = i;
		obj->type = SurviveSimpleObject_LIGHTHOUSE;
		obj->actx = actx;
		obj->has_update = ctx->bsd[i].PositionSet;
		snprintf(obj->name, 32, "LH%" PRIdPTR, i);
		snprintf(obj->data.lh.serial_number, 16, "LHB-%X", ctx->bsd[i].BaseStationID);
	}
	for (; i < object_ct; i++) {
		struct SurviveSimpleObject *obj = &actx->objects[i];
		int so_idx = i - ctx->activeLighthouses;
		obj->data.so = ctx->objs[so_idx];
		obj->type = SurviveSimpleObject_OBJECT;
		obj->actx = actx;
		obj->data.so->user_ptr = (void*)i;
		strncpy(obj->name, obj->data.so->codename, sizeof(obj->name));
	}

	survive_install_pose_fn(ctx, pose_fn);
	survive_install_external_pose_fn(ctx, external_pose_fn);
	survive_install_external_pose_fn(ctx, external_pose_fn);
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
	for (int i = 0; i < actx->external_object_ct; i++) {
		if (actx->external_objects[i].has_update) {
			actx->external_objects[i].has_update = false;
			return &actx->external_objects[i];
		}
	}
	return 0;
}

uint32_t survive_simple_object_get_latest_pose(const struct SurviveSimpleObject *sao, SurvivePose *pose) {
	uint32_t timecode = 0;
	OGLockMutex(sao->actx->poll_mutex);

	switch (sao->type) {
	case SurviveSimpleObject_LIGHTHOUSE: {
		if (pose)
			*pose = sao->actx->ctx->bsd[sao->data.lh.lighthouse].Pose;
		break;
	}
	case SurviveSimpleObject_OBJECT:
		if (pose)
			*pose = sao->data.so->OutPose;
		timecode = sao->data.so->OutPose_timecode;
		break;
	case SurviveSimpleObject_EXTERNAL:
		if (pose)
			*pose = sao->data.seo.pose;
		break;

	default: {
		SurviveContext *ctx = sao->actx->ctx;
		SV_ERROR("Invalid object type %d", sao->type);
	}
	}

	OGUnlockMutex(sao->actx->poll_mutex);
	return timecode;
}

const char *survive_simple_object_name(const SurviveSimpleObject *sao) { return sao->name; }
const char *survive_simple_serial_number(const SurviveSimpleObject *sao) {
	switch (sao->type) {
	case SurviveSimpleObject_LIGHTHOUSE: {
		return sao->data.lh.serial_number;
	}
	case SurviveSimpleObject_OBJECT:
		return sao->data.so->serial_number;
	case SurviveSimpleObject_EXTERNAL:
	default:
		return "";
	}
}
