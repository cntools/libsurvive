#pragma once
#include "survive_types.h"
#include <stdlib.h>

typedef struct cstring {
	char *d;
	size_t length;
	size_t size;
} cstring;

SURVIVE_EXPORT void str_ensure_size(cstring *str, size_t s);
SURVIVE_EXPORT char *str_increase_by(cstring *str, size_t len);
SURVIVE_EXPORT void str_append(cstring *str, const char *add);
SURVIVE_EXPORT int str_append_printf(cstring *str, const char *format, ...);
SURVIVE_EXPORT void str_free(cstring *str);
SURVIVE_EXPORT void str_clear(cstring *str);
SURVIVE_EXPORT void str_append_n(cstring* cstr, const char* buffer, size_t len);
