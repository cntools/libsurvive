// (C) 2017 <>< Joshua Allen, Under MIT/x11 License.
#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "json_helpers.h"
#include <jsmn.h>
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h>
#endif

#ifdef _MSC_VER
#include <stdarg.h>

// Windows doesn't provide asprintf, so we need to make it.
int asprintf(char **strp, const char *fmt, ...)
{
	char* buff = NULL;
	va_list listPointer;
	va_start( listPointer, fmt );

	size_t lenNeeded = _vscprintf(fmt, listPointer) + 1; // add one for a terminating null

	if (lenNeeded > 1)
	{
		buff = (char*)malloc(lenNeeded);
		if (buff)
		{
			int bytesWritten = _vsnprintf(buff, lenNeeded, fmt, listPointer);
			if (bytesWritten < 0)			
			{
				free(buff);
				buff = NULL;
			}
		}
	}

	if (strp)
	{
		*strp = buff;
	}
	return (int)lenNeeded;
}

#endif

void json_write_float_array(FILE* f, const char* tag, float* v, uint8_t count) {
	uint8_t i = 0;
	char * str1 = NULL;
	char * str2 = NULL;
	if( asprintf(&str1,"\"%s\":[", tag) < 0 ) goto giveup;

	for (i=0;i<count;++i) {
		if ( (i+1) < count) {
			if (asprintf(&str2, "%s\"%.12f\"", str1, v[i]) < 0)
				goto giveup;
		} else {
			if (asprintf(&str2, "%s\"%.12f\",", str1, v[i]) < 0)
				goto giveup;
		}
		free(str1);
		str1=str2;
		str2=NULL;
	}
	if( asprintf(&str2, "%s]", str1) < 0 ) goto giveup;
	fputs(str2,f);
giveup:
	if( str1 ) free(str1);
	if( str2 ) free(str2);
}

void json_write_double_array(FILE* f, const char* tag, double* v, uint8_t count) {
	uint8_t i = 0;
	char * str1 = NULL;
	char * str2 = NULL;
	if( asprintf(&str1,"\"%s\":[", tag) < 0 ) goto giveup;

	for (i=0;i<count;++i) {
		if (i<(count-1)) {
			if (asprintf(&str2, "%s\"%.12f\",", str1, v[i]) < 0)
				goto giveup;
		} else {
			if (asprintf(&str2, "%s\"%.12f\"", str1, v[i]) < 0)
				goto giveup;
		}
		free(str1);
		str1=str2;
		str2=NULL;
	}
	if( asprintf(&str2, "%s]", str1) < 0 ) goto giveup;
	fputs(str2,f);
giveup:
	if( str1 ) free(str1);
	if( str2 ) free(str2);
}

void json_write_uint32(FILE* f, const char* tag, uint32_t v) {
	fprintf(f, "\"%s\":\"%"PRIu32"\"", tag, v);
}

void json_write_float(FILE *f, const char *tag, float v) { fprintf(f, "\"%s\":\"%.12f\"", tag, v); }

void json_write_str(FILE* f, const char* tag, const char* v) {
	fprintf(f, "\"%s\":\"%s\"", tag, v);
}

uint32_t JSON_STRING_LEN;

char* load_file_to_mem(const char* path) {
	FILE * f = fopen( path, "r" );
	if (f==NULL) return NULL;
	fseek( f, 0, SEEK_END );
	int len = ftell( f );
	fseek( f, 0, SEEK_SET );
	char * JSON_STRING = malloc( len + 1);
	memset(JSON_STRING,0,len+1);
	size_t i = fread(JSON_STRING, len, 1, f);
	(void)i; // Ignore return value.
	fclose( f );
	return JSON_STRING;
}

typedef struct json_stack_entry_s {
	const char *data;
	struct json_stack_entry_s *previous;
	int index;
	jsmntok_t *token, *key;
} stack_entry_t;

static const char *json_value(const char *d, jsmntok_t *t) {
	if (t == 0)
		return "";
	return d + t->start;
}

bool json_has_ancestor_tag(const char *tag, struct json_stack_entry_s *obj) {
	if (!obj)
		return false;
	const char *tag_value = json_stack_tag(obj);
	if (tag_value && strcmp(tag, tag_value) == 0)
		return true;
	return json_has_ancestor_tag(tag, obj->previous);
}
int json_stack_index(struct json_stack_entry_s *obj) { return obj->index; }
const char *json_stack_tag(struct json_stack_entry_s *obj) { return json_value(obj->data, obj->key); }
const char *json_stack_value(struct json_stack_entry_s *obj) { return json_value(obj->data, obj->token); }

#define INVOKE_CB(cb)                                                                                                  \
	if (cb)                                                                                                            \
	cb

static int process_jsontok(struct json_callbacks *cbs, const char *d, stack_entry_t *stack, jsmntok_t *t, int count) {
	int i, j, k;
	if (stack) {
		stack->data = d;
		stack->token = t;
	}

	if (count == 0) {
		return 0;
	}
	if (t->type == JSMN_PRIMITIVE) {
		INVOKE_CB(cbs->json_tag_value)(cbs, stack);
		return 1;
	} else if (t->type == JSMN_STRING) {
		INVOKE_CB(cbs->json_tag_value)(cbs, stack);
		return 1;
	} else if (t->type == JSMN_OBJECT) {
		j = 0;
		INVOKE_CB(cbs->json_begin_object)(cbs, stack);

		for (i = 0; i < t->size; i++) {
			stack_entry_t entry = {.key = t + 1 + j, .index = i, .previous = stack};
			// print_stack_spot(d, &entry);
			// Skip the key
			j++;
			// Read value
			j += process_jsontok(cbs, d, &entry, t + 1 + j, count - j);
		}
		INVOKE_CB(cbs->json_end_object)(cbs, stack);
		return j + 1;
	} else if (t->type == JSMN_ARRAY) {
		INVOKE_CB(cbs->json_begin_array)(cbs, stack);
		j = 0;
		for (i = 0; i < t->size; i++) {
			stack_entry_t entry = {.index = i, .previous = stack};
			j += process_jsontok(cbs, d, &entry, t + 1 + j, count - j);
		}
		INVOKE_CB(cbs->json_end_array)(cbs, stack);
		return j + 1;
	}
	return 0;
}

int json_run_callbacks(struct json_callbacks *cbs, const char *_JSON_STRING, int JSON_STRING_LEN) {
	jsmn_parser parser;
	jsmn_init(&parser);
	int32_t items = jsmn_parse(&parser, _JSON_STRING, JSON_STRING_LEN);
	if (items < 0) {
		return items;
	}
	char *JSON_STRING = malloc(JSON_STRING_LEN);
	memcpy(JSON_STRING, _JSON_STRING, JSON_STRING_LEN);

	jsmntok_t *tokens = parser.token_pool;
	for (int i = 0; i < items; i++) {
		JSON_STRING[tokens[i].end] = 0;
	}

	int16_t children = -1;
	bool noGlobal = parser.token_pool->type != JSMN_ARRAY && parser.token_pool->type != JSMN_OBJECT;
	if (noGlobal) {
		for (int j = 0; j < items;) {
			stack_entry_t entry = {.key = tokens + j};
			j++;
			j += process_jsontok(cbs, JSON_STRING, &entry, tokens + j, items - j);
		}
	} else {
		process_jsontok(cbs, JSON_STRING, 0, parser.token_pool, items);
	}
	jsmn_free(&parser);
	free(JSON_STRING);
	return items;
}
void json_load_file(struct json_callbacks *cbs, const char *path) {
	uint32_t i = 0;

	char* JSON_STRING = load_file_to_mem(path);
	if (JSON_STRING==NULL) return;

	JSON_STRING_LEN = (uint32_t)strlen(JSON_STRING);

	json_run_callbacks(cbs, JSON_STRING, JSON_STRING_LEN);

	free(JSON_STRING);
}

int parse_float_array_in_place(char *str, const jsmntok_t *token, FLT *f, uint8_t count) {
	for (int i = 0; i < count; ++i) {
		char* end = str + token->end;
		char* s = str+token->start;

		#ifdef USE_DOUBLE
		f[i] = strtod(s, &end);
		#else
		f[i] = strtof(s, &end);
		#endif

		if (s == end) {
			return 0; //not a float
		}
		token++;
	}


	return count;
}
int parse_float_array(char *str, const jsmntok_t *token, FLT **f, uint8_t count) {
	if (count == 0)
		return 0;

	if (*f != NULL)
		free(*f);
	*f = malloc(sizeof(FLT) * count);

	int rtn = parse_float_array_in_place(str, token, *f, count);
	if (rtn == 0) {
		free(*f);
		*f = 0;
	}

	return count;
}

int parse_int_array_in_place(char *str, const jsmntok_t *token, int *f, uint8_t count) {
	for (int i = 0; i < count; ++i) {
		char *end = str + token->end;
		char *s = str + token->start;

		f[i] = atoi(s);

		if (s == end) {
			return 0; // not a float
		}
		token++;
	}

	return count;
}
int parse_int_array(char *str, const jsmntok_t *token, int **f, uint8_t count) {
	if (count == 0)
		return 0;

	if (*f != NULL)
		free(*f);
	*f = malloc(sizeof(int) * count);

	int rtn = parse_int_array_in_place(str, token, *f, count);
	if (rtn == 0) {
		free(*f);
		*f = 0;
	}

	return count;
}
