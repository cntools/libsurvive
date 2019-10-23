#ifndef SURVIVE_STR_H
#define SURVIVE_STR_H

#include <stdlib.h>

typedef struct cstring {
    char* d;
    size_t length;
    size_t size;
} cstring;

void str_ensure_size(cstring* str, size_t s);
char* str_increase_by(cstring* str, size_t len);
void str_append(cstring* str, const char* add);
int str_append_printf(cstring* str, const char *format, ...);
void str_free(cstring* str);
#endif