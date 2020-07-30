#pragma once

#include <survive_optimizer.h>
#include <survive_types.h>

struct survive_async_optimizer;

typedef struct survive_async_optimizer_buffer {
	survive_optimizer optimizer;
	void *user;
} survive_async_optimizer_buffer;

typedef void (*survive_async_optimizer_cb)(struct survive_async_optimizer_buffer *buffer, int return_code,
										   struct mp_result_struct *result);

SURVIVE_EXPORT struct survive_async_optimizer *survive_async_init(survive_async_optimizer_cb cb);
SURVIVE_EXPORT void survive_async_free(struct survive_async_optimizer *optimizer);

SURVIVE_EXPORT survive_async_optimizer_buffer *
survive_async_optimizer_alloc_optimizer(struct survive_async_optimizer *optimizer);
SURVIVE_EXPORT void survive_async_optimizer_run(struct survive_async_optimizer *optimizer,
												survive_async_optimizer_buffer *);
