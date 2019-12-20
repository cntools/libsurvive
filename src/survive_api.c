#define SURVIVE_ENABLE_FULL_API
#include <assert.h>
#include <survive_api.h>

#include "survive_api.h"
#include "inttypes.h"
#include "os_generic.h"
#include "stdio.h"
#include "string.h"
#include "survive.h"

struct SurviveExternalObject {
	SurvivePose pose;
	SurviveVelocity velocity;
};

struct SurviveLighthouseData {
	int lighthouse;
	char serial_number[16];
};

struct SurviveSimpleObject {
	struct SurviveSimpleContext *actx;

	enum SurviveSimpleObject_type type;

	union {
		struct SurviveLighthouseData lh;
		struct SurviveObject *so;
		struct SurviveExternalObject seo;
	} data;

	char name[32];
	bool has_update;

	SurviveSimpleObject *next;
};

struct SurviveSimpleObjectList {
	size_t cnt;
	SurviveSimpleObject *head, *tail;
};

#define MAX_EVENT_SIZE 64
struct SurviveSimpleContext {
	SurviveContext *ctx;
	SurviveSimpleLogFn log_fn;

	bool running;
	og_thread_t thread;
	og_mutex_t poll_mutex;

	size_t events_cnt;
	size_t event_next_write;
	struct SurviveSimpleEvent events[MAX_EVENT_SIZE];

	struct SurviveSimpleObjectList objects;
};

static void insert_into_event_buffer(SurviveSimpleContext *actx, const SurviveSimpleEvent *event) {
	bool buffer_full = actx->events_cnt == MAX_EVENT_SIZE;

	actx->events[actx->event_next_write] = *event;
	actx->event_next_write = (actx->event_next_write + 1) % MAX_EVENT_SIZE;
	if (!buffer_full)
		actx->events_cnt++;
}

static bool pop_from_event_buffer(SurviveSimpleContext *actx, SurviveSimpleEvent *event) {
	if (actx->events_cnt == 0)
		return false;

	size_t read_idx = (MAX_EVENT_SIZE + actx->event_next_write - actx->events_cnt) % MAX_EVENT_SIZE;
	*event = actx->events[read_idx];

	// Mark read events as invalid and assert if they are about to be passed on. This means there is a logic bug in the
	// circular buffer
	assert(event->event_type != -1);
	actx->events[read_idx].event_type = -1;

	actx->events_cnt--;
	return true;
}

static void SurviveSimpleObjectList_add(struct SurviveSimpleObjectList *list, SurviveSimpleObject *so) {
	list->cnt++;
	if (list->head == 0) {
		list->head = so;
	} else {
		assert(list->tail);
		list->tail->next = so;
	}

	list->tail = so;
}

static SurviveSimpleObject *find_or_create_external(SurviveSimpleContext *actx, const char *name) {
	for (struct SurviveSimpleObject *so = actx->objects.head; so; so = so->next) {
		if (strncmp(name, so->name, 32) == 0) {
			return so;
		}
	}

	SurviveSimpleObject *so = SV_CALLOC(1, sizeof(struct SurviveSimpleObject));
	so->type = SurviveSimpleObject_EXTERNAL;
	so->actx = actx;
	strncpy(so->name, name, sizeof(so->name) - 1);
	SurviveSimpleObjectList_add(&actx->objects, so);

	return so;
}

static void external_velocity_fn(SurviveContext *ctx, const char *name, const SurviveVelocity *velocity) {
	SurviveSimpleContext *actx = ctx->user_ptr;
	OGLockMutex(actx->poll_mutex);
	survive_default_external_velocity_process(ctx, name, velocity);

	SurviveSimpleObject *so = find_or_create_external(actx, name);
	so->has_update = true;
	so->data.seo.velocity = *velocity;
	OGUnlockMutex(actx->poll_mutex);
}

static void external_pose_fn(SurviveContext *ctx, const char *name, const SurvivePose *pose) {
	SurviveSimpleContext *actx = ctx->user_ptr;
	OGLockMutex(actx->poll_mutex);
	survive_default_external_pose_process(ctx, name, pose);

	SurviveSimpleObject *so = find_or_create_external(actx, name);
	so->has_update = true;
	so->data.seo.pose = *pose;
	OGUnlockMutex(actx->poll_mutex);
}
static void pose_fn(SurviveObject *so, uint32_t timecode, SurvivePose *pose) {
	SurviveSimpleContext *actx = so->ctx->user_ptr;
	OGLockMutex(actx->poll_mutex);
	survive_default_pose_process(so, timecode, pose);

	struct SurviveSimpleObject *sao = so->user_ptr;
	sao->has_update = true;
	OGUnlockMutex(actx->poll_mutex);
}
static void lh_fn(SurviveContext *ctx, uint8_t lighthouse, SurvivePose *lighthouse_pose,
	SurvivePose *object_pose) {
	SurviveSimpleContext *actx = ctx->user_ptr;
	OGLockMutex(actx->poll_mutex);
	survive_default_lighthouse_pose_process(ctx, lighthouse, lighthouse_pose, object_pose);

	struct SurviveSimpleObject *sao = ctx->bsd[lighthouse].user_ptr;
	sao->has_update = true;

	OGUnlockMutex(actx->poll_mutex);
}

static void button_fn(SurviveObject *so, uint8_t eventType, uint8_t buttonId, uint8_t axis1Id, uint16_t axis1Val,
					  uint8_t axis2Id, uint16_t axis2Val) {
	SurviveSimpleContext *actx = so->ctx->user_ptr;
	OGLockMutex(actx->poll_mutex);
	survive_default_button_process(so, eventType, buttonId, axis1Id, axis1Val, axis2Id, axis2Val);
	struct SurviveSimpleObject *sao = so->user_ptr;

	SurviveSimpleEvent event = {.event_type = SurviveSimpleEventType_ButtonEvent,
								.button_event = {.object = sao,
												 .event_type = eventType,
												 .button_id = buttonId,
												 .axis_count = 2,
												 .axis_ids = {axis1Id, axis2Id},
												 .axis_val = {axis1Val, axis2Val}}};

	insert_into_event_buffer(actx, &event);

	OGUnlockMutex(actx->poll_mutex);
}

SURVIVE_EXPORT SurviveSimpleContext *survive_simple_init(int argc, char *const *argv) {
	return survive_simple_init_with_logger(argc, argv, 0);
}

static void simple_log_fn(SurviveContext *ctx, SurviveLogLevel logLevel, const char *msg) {
	SurviveSimpleContext *actx = ctx->user_ptr;
	if (actx == 0 || actx->log_fn == 0) {
		survive_default_log_process(ctx, logLevel, msg);
		return;
	}

	actx->log_fn(actx, logLevel, msg);
}

static void new_object_fn(SurviveObject *so) {
	SurviveSimpleContext *actx = so->ctx->user_ptr;

	SurviveSimpleObject *obj = SV_CALLOC(1, sizeof(struct SurviveSimpleObject));
	obj->data.so = so;
	obj->type = SurviveSimpleObject_OBJECT;
	obj->actx = actx;
	obj->data.so->user_ptr = (void *)obj;
	strncpy(obj->name, obj->data.so->codename, sizeof(obj->name));

	SurviveSimpleObjectList_add(&actx->objects, obj);
}

SURVIVE_EXPORT SurviveSimpleContext *survive_simple_init_with_logger(int argc, char *const *argv,
																	 SurviveSimpleLogFn fn) {
	SurviveSimpleContext *actx = SV_CALLOC(1, sizeof(SurviveSimpleContext));
	actx->log_fn = fn;

	SurviveContext *ctx = survive_init_with_logger(argc, argv, actx, simple_log_fn);
	survive_install_new_object_fn(ctx, new_object_fn);

	if (ctx == 0) {
		free(actx);
		return 0;
	}
	ctx->user_ptr = actx;
	survive_startup(ctx);

	actx->ctx = ctx;
	actx->poll_mutex = OGCreateMutex();

	intptr_t i = 0;
	for (i = 0; i < ctx->activeLighthouses; i++) {
		SurviveSimpleObject *obj = SV_CALLOC(1, sizeof(struct SurviveSimpleObject));
		obj->data.lh.lighthouse = i;
		obj->type = SurviveSimpleObject_LIGHTHOUSE;
		obj->actx = actx;
		obj->has_update = ctx->bsd[i].PositionSet;
		ctx->bsd[i].user_ptr = obj;
		snprintf(obj->name, 32, "LH%" PRIdPTR, i);
		snprintf(obj->data.lh.serial_number, 16, "LHB-%X", ctx->bsd[i].BaseStationID);
		SurviveSimpleObjectList_add(&actx->objects, obj);
	}

	survive_install_pose_fn(ctx, pose_fn);
	survive_install_external_pose_fn(ctx, external_pose_fn);
	survive_install_external_velocity_fn(ctx, external_velocity_fn);
	survive_install_button_fn(ctx, button_fn);
	survive_install_lighthouse_pose_fn(ctx, lh_fn);
	return actx;
}

int survive_simple_stop_thread(SurviveSimpleContext *actx) {
	actx->running = false;
	intptr_t error = (intptr_t)OGJoinThread(actx->thread);
	if (error != 0) {
		SurviveContext *ctx = actx->ctx;
		SV_INFO("Warning: Loop exited with error %" PRIdPTR, error);
	}
	return error;
}

void survive_simple_close(SurviveSimpleContext *actx) {
	if (actx->running) {
		survive_simple_stop_thread(actx);
	}

	survive_close(actx->ctx);
}

static inline void *__simple_thread(void *_actx) {
	SurviveSimpleContext *actx = _actx;
	intptr_t error = 0;
	while (actx->running && error == 0) {
		error = survive_poll(actx->ctx);
	}
	actx->running = false;
	return (void*)error; 
}
bool survive_simple_is_running(SurviveSimpleContext *actx) { return actx->running; }
void survive_simple_start_thread(SurviveSimpleContext *actx) {
	actx->running = true;
	actx->thread = OGCreateThread(__simple_thread, actx);
	OGNameThread(actx->thread, "survive simple api");
}

const SurviveSimpleObject *survive_simple_get_next_object(SurviveSimpleContext *actx, const SurviveSimpleObject *curr) {
	return curr->next;
}

const SurviveSimpleObject *survive_simple_get_first_object(SurviveSimpleContext *actx) { return actx->objects.head; }

size_t survive_simple_get_object_count(SurviveSimpleContext *actx) { return actx->objects.cnt; }

const SurviveSimpleObject *survive_simple_get_next_updated(SurviveSimpleContext *actx) {
	for (struct SurviveSimpleObject *n = actx->objects.head; n; n = n->next) {
		if (n->has_update) {
			n->has_update = false;
			return n;
		}
	}
	return 0;
}

survive_timecode survive_simple_object_get_latest_velocity(const SurviveSimpleObject *sao, SurviveVelocity *velocity) {
	uint32_t timecode = 0;
	OGLockMutex(sao->actx->poll_mutex);

	switch (sao->type) {
	case SurviveSimpleObject_LIGHTHOUSE:
		if (velocity)
			*velocity = (SurviveVelocity){ 0 };
		break;
	case SurviveSimpleObject_OBJECT:
		if (velocity)
			*velocity = sao->data.so->velocity;
		timecode = sao->data.so->velocity_timecode;
		break;
	case SurviveSimpleObject_EXTERNAL:
		if (velocity)
			*velocity = sao->data.seo.velocity;
		break;

	default: {
		SurviveContext *ctx = sao->actx->ctx;
		SV_GENERAL_ERROR("Invalid object type %d", sao->type);
	}
	}

	OGUnlockMutex(sao->actx->poll_mutex);
	return timecode;
}

uint32_t survive_simple_object_get_latest_pose(const SurviveSimpleObject *sao, SurvivePose *pose) {
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
		SV_GENERAL_ERROR("Invalid object type %d", sao->type);
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

void survive_simple_lock(SurviveSimpleContext *actx) { OGLockMutex(actx->poll_mutex); }

void survive_simple_unlock(SurviveSimpleContext *actx) { OGUnlockMutex(actx->poll_mutex); }

SurviveContext *survive_simple_get_ctx(SurviveSimpleContext *actx) { return actx->ctx; }

SurviveObject *survive_simple_get_survive_object(const SurviveSimpleObject *sao) {
	switch (sao->type) {
	case SurviveSimpleObject_OBJECT:
		return sao->data.so;
	default:
		return NULL;
	}
}

BaseStationData *survive_simple_get_bsd(const SurviveSimpleObject *sao) {
	switch (sao->type) {
	case SurviveSimpleObject_LIGHTHOUSE:
		return &sao->actx->ctx->bsd[sao->data.lh.lighthouse];
	default:
		return NULL;
	}
}

const SurviveSimpleButtonEvent *survive_simple_get_button_event(const SurviveSimpleEvent *event) {
	if (event->event_type == SurviveSimpleEventType_ButtonEvent)
		return &event->button_event;
	return 0;
}

enum SurviveSimpleEventType survive_simple_next_event(SurviveSimpleContext *actx, SurviveSimpleEvent *event) {
	event->event_type = SurviveSimpleEventType_None;
	OGLockMutex(actx->poll_mutex);
	pop_from_event_buffer(actx, event);
	OGUnlockMutex(actx->poll_mutex);
	return event->event_type;
}

enum SurviveSimpleObject_type survive_simple_object_get_type(const struct SurviveSimpleObject *sao) {
	return sao->type;
}