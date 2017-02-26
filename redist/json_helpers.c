// (C) 2017 <>< Joshua Allen, Under MIT/x11 License.

#define _GNU_SOURCE

#include <stdio.h>
#include <string.h>
#include "json_helpers.h"

void json_write_float_array(FILE* f, const char* tag, float* v, uint8_t count) {
	uint8_t i = 0;
	char * str1 = NULL;
	char * str2 = NULL;
	asprintf(&str1,"\"%s\":[", tag);

	for (i=0;i<count;++i) {
		if (i<(count-1)) {
			asprintf(&str2, "%s\"%f\",", str1,v[i]);
		} else {
			asprintf(&str2, "%s\"%f\"", str1,v[i]);
		}
		free(str1);
		str1=str2;
		str2=NULL;
	}
	asprintf(&str2, "%s]\n", str1,v[i]);
	fputs(str2,f);
	free(str1);
	free(str2);
}

void json_write_double_array(FILE* f, const char* tag, double* v, uint8_t count) {
	uint8_t i = 0;
	char * str1 = NULL;
	char * str2 = NULL;
	asprintf(&str1,"\"%s\":[", tag);

	for (i=0;i<count;++i) {
		if (i<(count-1)) {
			asprintf(&str2, "%s\"%f\",", str1,v[i]);
		} else {
			asprintf(&str2, "%s\"%f\"", str1,v[i]);
		}
		free(str1);
		str1=str2;
		str2=NULL;
	}
	asprintf(&str2, "%s]\n", str1,v[i]);
	fputs(str2,f);
	free(str1);
	free(str2);
}

void json_write_uint32(FILE* f, const char* tag, uint32_t v) {
	fprintf(f, "\"%s\":\"%d\"\n", tag, v);
}

void json_write_float(FILE* f, const char* tag, float v) {
	fprintf(f, "\"%s\":\"%f\"\n", tag, v);
}

void json_write_str(FILE* f, const char* tag, const char* v) {
	fprintf(f, "\"%s\":\"%s\"\n", tag, v);
}