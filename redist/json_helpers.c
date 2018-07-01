// (C) 2017 <>< Joshua Allen, Under MIT/x11 License.

#define _GNU_SOURCE

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
			if( asprintf(&str2, "%s\"%f\"", str1,v[i]) < 0 ) goto giveup;
		} else {
			if( asprintf(&str2, "%s\"%f\",", str1,v[i]) < 0 ) goto giveup;
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
			if( asprintf(&str2, "%s\"%f\",", str1,v[i]) < 0 ) goto giveup;
		} else {
			if( asprintf(&str2, "%s\"%f\"", str1,v[i]) < 0 ) goto giveup;
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
	fprintf(f, "\"%s\":\"%d\"", tag, v);
}

void json_write_float(FILE* f, const char* tag, float v) {
	fprintf(f, "\"%s\":\"%f\"", tag, v);
}

void json_write_str(FILE* f, const char* tag, const char* v) {
	fprintf(f, "\"%s\":\"%s\"", tag, v);
}




void (*json_begin_object)(char* tag) = NULL;
void (*json_end_object)() = NULL;
void (*json_tag_value)(char* tag, char** values, uint8_t count) = NULL;

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

static char* substr(const char* str, uint32_t start, uint32_t end, uint32_t npos) {
	uint32_t l = end-start+1;

	if (end<=start || end>=npos) return NULL;

	char* x = malloc(l);
	memcpy(x,str+start,l);
	x[l-1] = '\0';
	return x;
}

static uint16_t json_load_array(const char* JSON_STRING, jsmntok_t* tokens, uint16_t size, char* tag) {
	jsmntok_t* t = tokens;
	uint16_t i = 0;

	char** values;
	values = alloca(sizeof(char*) * size);

	for (i=0;i<size;++i) {
		t = tokens+i;
		values[i] = substr(JSON_STRING, t->start, t->end, JSON_STRING_LEN);
	}

	if (json_tag_value != NULL) json_tag_value(tag, values, (uint8_t)i);

	for (i=0;i<size;++i) free(values[i]);

	return size;
}

void json_load_file(const char* path) {
	uint32_t i = 0;

	char* JSON_STRING = load_file_to_mem(path);
	if (JSON_STRING==NULL) return;

	JSON_STRING_LEN = (uint32_t)strlen(JSON_STRING);

	jsmn_parser parser;
	jsmn_init(&parser);

	int32_t items = jsmn_parse(&parser, JSON_STRING, JSON_STRING_LEN, NULL, 0);
	if (items < 0) return;
	jsmntok_t* tokens = malloc(items * sizeof(jsmntok_t));

	jsmn_init(&parser);
	items = jsmn_parse(&parser, JSON_STRING, JSON_STRING_LEN, tokens, items);

	int16_t children = -1;

	for (i=0; i<(unsigned int)items; i+=2)
	{
		//increment i on each successful tag + values combination, not individual tokens
		jsmntok_t* tag_t = tokens+i;
		jsmntok_t* value_t = tokens+i+1;

		char* tag = substr(JSON_STRING, tag_t->start, tag_t->end, JSON_STRING_LEN);
		char* value = substr(JSON_STRING, value_t->start, value_t->end, JSON_STRING_LEN);

		if (value_t->type == JSMN_ARRAY) {
			i += json_load_array(JSON_STRING, tokens+i+2,value_t->size, tag); //look at array children
		} else if (value_t->type == JSMN_OBJECT) {
			if (json_begin_object != NULL) json_begin_object(tag);
			children = (int16_t)(value_t->size +1); //+1 to account for this loop where we are not yed parsing children
//			i += decode_jsmn_object(JSON_STRING, tokens+i+2,value_t->size);
		}
		else {
			if (json_tag_value != NULL) json_tag_value(tag, &value, 1);
		}

		if (children>=0) children--;
		if (children == 0) {
			children = -1;
			if (json_end_object!=NULL) json_end_object();
		}

//		printf("%d %s \n", value_t->type, tag);

		free(tag);
		free(value);
	}

	free(tokens);
	free(JSON_STRING);
}

int parse_float_array_in_place(char *str, jsmntok_t *token, FLT *f, uint8_t count) {
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
int parse_float_array(char *str, jsmntok_t *token, FLT **f, uint8_t count) {
	uint16_t i = 0;

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
