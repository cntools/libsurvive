#include "survive_async_optimizer.h"

static void run_buffer(survive_async_optimizer *self, uint8_t idx) {
	struct mp_result_struct results = {0};
	self->active_buffer = idx;
	OGUnlockMutex(self->active_buffer_lock);
	self->completed++;
	int status = survive_optimizer_run(&self->buffers[idx].optimizer, &results, 0);
	if (self->cb) {
		self->cb(&self->buffers[idx], status, &results);
	}
	OGLockMutex(self->active_buffer_lock);
	self->buffer_ready[idx] = false;
	self->active_buffer = -1;
}

static void *async_thread(void *param) {
	survive_async_optimizer *self = param;
	OGLockMutex(self->active_buffer_lock);
	while (self->cb) {
		for (uint8_t i = 0; i < 2; i++) {
			if (self->buffer_ready[i]) {
				run_buffer(self, i);
			}
		}

		OGWaitCond(self->job_available, self->active_buffer_lock);
	}

	OGUnlockMutex(self->active_buffer_lock);
	return 0;
}

struct survive_async_optimizer *survive_async_optimizer_init(struct survive_async_optimizer *self,
															 survive_async_optimizer_cb cb) {
	self->cb = cb;
	self->active_buffer = -1;
	self->active_buffer_lock = OGCreateMutex();
	self->job_available = OGCreateConditionVariable();

	self->async_thread = OGCreateThread(async_thread, "async optimizer", self);
	return self;
}

survive_async_optimizer_buffer *survive_async_optimizer_alloc_optimizer(struct survive_async_optimizer *self) {
	OGLockMutex(self->active_buffer_lock);
	survive_async_optimizer_buffer *rtn = 0;
	if (self->active_buffer == 0) {
		rtn = &self->buffers[1];
		self->buffer_ready[1] = false;
	} else {
		rtn = &self->buffers[0];
		self->buffer_ready[0] = false;
	}
	self->submitted++;
	OGUnlockMutex(self->active_buffer_lock);
	return rtn;
}

void survive_async_optimizer_run(struct survive_async_optimizer *self, survive_async_optimizer_buffer *opt) {
	OGLockMutex(self->active_buffer_lock);
	uint8_t idx = opt == &self->buffers[0] ? 0 : 1;
	self->buffer_ready[idx] = true;
	OGSignalCond(self->job_available);
	OGUnlockMutex(self->active_buffer_lock);
}

void survive_async_free(struct survive_async_optimizer *self) {
	if (self == 0) {
		return;
	}

	OGLockMutex(self->active_buffer_lock);
	self->cb = 0;
	OGSignalCond(self->job_available);
	OGUnlockMutex(self->active_buffer_lock);

	OGJoinThread(self->async_thread);

	OGDeleteConditionVariable(self->job_available);
	OGDeleteMutex(self->active_buffer_lock);

	for (int i = 0; i < 2; i++) {
		SURVIVE_OPTIMIZER_CLEANUP_HEAP_BUFFERS(self->buffers[i].optimizer);
		free(self->buffers[i].user);
	}

	free(self);
}
