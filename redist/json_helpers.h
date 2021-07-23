// (C) 2017 <>< Joshua Allen, Under MIT/x11 License.

#ifndef JSON_HELPERS_H
#define JSON_HELPERS_H

#include "survive_types.h"
#include <jsmn.h>
#include <stdint.h>
#include <stdio.h>

void json_write_float_array(FILE* f, const char* tag, float* v, uint8_t count);
void json_write_double_array(FILE* f, const char* tag, double* v, uint8_t count);
void json_write_uint32(FILE* f, const char* tag, uint32_t v);
void json_write_float(FILE* f, const char* tag, float v);
void json_write_str(FILE* f, const char* tag, const char* v);

int parse_float_array_in_place(char *str, const jsmntok_t *token, FLT *values, uint8_t count);
int parse_float_array(char *str, const jsmntok_t *token, FLT **values, uint8_t count);

int parse_int_array_in_place(char *str, const jsmntok_t *token, int *values, uint8_t count);
int parse_int_array(char *str, const jsmntok_t *token, int **values, uint8_t count);

struct json_stack_entry_s;

bool json_has_ancestor_tag(const char *tag, struct json_stack_entry_s *obj);
const char *json_stack_value(struct json_stack_entry_s *obj);
const char *json_stack_tag(struct json_stack_entry_s *obj);
int json_stack_index(struct json_stack_entry_s *obj);
struct json_callbacks {
	void *user;
	void (*json_begin_object)(struct json_callbacks *cb, struct json_stack_entry_s *obj);
	void (*json_end_object)(struct json_callbacks *cb, struct json_stack_entry_s *obj);

	void (*json_begin_array)(struct json_callbacks *cb, struct json_stack_entry_s *array);
	void (*json_end_array)(struct json_callbacks *cb, struct json_stack_entry_s *array);

	void (*json_tag_value)(struct json_callbacks *cb, struct json_stack_entry_s *array);
};
int json_run_callbacks(struct json_callbacks *cbs, const char *json_string, int length);
void json_load_file(struct json_callbacks *cbs, const char *path);

#endif
