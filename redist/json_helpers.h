// (C) 2017 <>< Joshua Allen, Under MIT/x11 License.

#ifndef JSON_HELPERS_H
#define JSON_HELPERS_H

#include <stdint.h>

void json_write_float_array(FILE* f, const char* tag, float* v, uint8_t count);
void json_write_double_array(FILE* f, const char* tag, double* v, uint8_t count);
void json_write_uint32(FILE* f, const char* tag, uint32_t v);
void json_write_float(FILE* f, const char* tag, float v);
void json_write_str(FILE* f, const char* tag, const char* v);

void json_load_file(const char* path);
extern void (*json_begin_object)(char* tag);
extern void (*json_end_object)();
extern void (*json_tag_value)(char* tag, char** values, uint16_t count);


#endif