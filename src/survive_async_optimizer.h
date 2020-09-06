#pragma once

#include <survive_optimizer.h>
#include <survive_types.h>

typedef struct survive_async_optimizer_buffer {
	survive_optimizer optimizer;
	void *user;
} survive_async_optimizer_buffer;

typedef void (*survive_async_optimizer_cb)(struct survive_async_optimizer_buffer *buffer, int return_code,
										   struct mp_result_struct *result);

typedef struct survive_async_optimizer {
	survive_async_optimizer_cb cb;
	void *user;

	og_thread_t async_thread;

	int8_t active_buffer;
	bool buffer_ready[2];
	struct survive_async_optimizer_buffer buffers[2];
	og_mutex_t active_buffer_lock;

	og_cv_t job_available;

	size_t submitted;
	size_t completed;
} survive_async_optimizer;

SURVIVE_EXPORT struct survive_async_optimizer *survive_async_optimizer_init(struct survive_async_optimizer *self,
																			survive_async_optimizer_cb cb);
SURVIVE_EXPORT void survive_async_free(struct survive_async_optimizer *optimizer);

SURVIVE_EXPORT survive_async_optimizer_buffer *
survive_async_optimizer_alloc_optimizer(struct survive_async_optimizer *optimizer);
SURVIVE_EXPORT void survive_async_optimizer_run(struct survive_async_optimizer *optimizer,
												survive_async_optimizer_buffer *);
