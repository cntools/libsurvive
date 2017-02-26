// (C) 2017 <>< Joshua Allen, Under MIT/x11 License.
#include <string.h>
#include <assert.h>
#include "survive_config.h"
#include <json_helpers.h>

#define MAX_CONFIG_ENTRIES 100

config_val config_values[MAX_CONFIG_ENTRIES];
static uint16_t used_entries = 0;

static FILE *config_file = NULL;
const FLT* config_set_float_a(const char *tag, const FLT* values, uint8_t count);

void config_init() {
	uint16_t i = 0;
	for (i=0;i<MAX_CONFIG_ENTRIES;++i) {
		config_values[i].data = NULL;
		config_values[i].tag = NULL;
		config_values[i].type = CONFIG_UNKNOWN;
		config_values[i].elements = 0;
	}

	used_entries = 0;
}

void write_float(char* tag, FLT x) {
	fprintf(config_file, "\"%s\":\"%f\"\n", tag, x);
}

void set_float_a(char* tag, FLT *x, uint8_t count) {
	uint8_t i = 0;
	char t[100];
		printf("set float\n",t,x[i]);
	for (i=0;i<count;++i) {
		sprintf(t,"%s%d",tag,i);
		printf("%s:%f\n",t,x[i]);
		config_set_float(t,x[i]);
	}
}
/*
void set_float_a2(char* tag, char **x, uint8_t count) {
	uint8_t i = 0;
	char t[100];

	if (*x == NULL) {
		*x = (float*)malloc(count*sizeof(float));
	} else {
		*x = (float*)realloc(*x, count);
	}

	memcpy(x,)
	strcpy(*x,src);


	for (i=0;i<count;++i) {
		sprintf(t,"%s%d",tag,i);
		printf("%s:%f\n",t,x[i]);
		config_set_float(t,x[i]);
	}
}
*/
void set_uint32(char* tag, uint32_t x) {
//	fprintf(config_file, "\"%s\":\"%d\"\n", tag, x);

}

void config_open(const char* path, const char* mode) {
	config_file = fopen(path, mode);
}

void config_close() {
	fclose(config_file);
}
/*
void config_write_lighthouse(struct BaseStationData* bsd, uint8_t length) {
	uint8_t i = 0;

	for (i=0;i<length; ++i) {
		write_uint32("id", bsd[i].BaseStationID);
		write_float_a("position", bsd[i].Position, 3);
		write_float_a("quaternion", bsd[i].Quaternion, 4);
		write_float_a("quaternion", bsd[i].Quaternion, 4);
		write_float_a("fcalphase", bsd[i].fcalphase, 2);
		write_float_a("fcaltilt", bsd[i].fcaltilt,2);
		write_float_a("fcalcurve", bsd[i].fcalcurve,2);
		write_float_a("fcalgibpha", bsd[i].fcalgibpha,2);
		write_float_a("fcalgibmag", bsd[i].fcalgibmag,2);
	}
}
*/
void config_set_lighthouse(struct BaseStationData* bsd, uint8_t idx) {
	config_set_uint32("index", idx);
	config_set_uint32("id", bsd->BaseStationID);
	config_set_float_a("position", bsd->Position, 3);
	config_set_float_a("quaternion", bsd->Quaternion, 4);
	config_set_float_a("fcalphase", bsd->fcalphase, 2);
	config_set_float_a("fcaltilt", bsd->fcaltilt,2);
	config_set_float_a("fcalcurve", bsd->fcalcurve,2);
	config_set_float_a("fcalgibpha", bsd->fcalgibpha,2);
	config_set_float_a("fcalgibmag", bsd->fcalgibmag,2);
}

void sstrcpy(char** dest, const char *src) {
	uint32_t len = strlen(src)+1;
	if (*dest == NULL) {
		*dest = (char*)malloc(len);
	} else {
		*dest = (char*)realloc(*dest, len);
	}
	strcpy(*dest,src);
}

config_val* find_config_entry(const char *tag) {
	uint16_t i = 0;
	for (i=0;i<used_entries;++i) {
		if ( strcmp(config_values[i].tag, tag) == 0 ) {
			return config_values+i;
		}
	}
	return NULL;
}

const char* config_read_str(const char *tag, const char *value, const char *def_str) {
	config_val *cv = find_config_entry(tag);

	if (cv != NULL) return cv->data;

	assert(used_entries<MAX_CONFIG_ENTRIES);

	used_entries++;
	sstrcpy(&(cv->tag), tag);
	sstrcpy(&(cv->data), def_str);
	cv->type = CONFIG_STRING;

	return cv->data;
}

uint32_t config_read_uint32(const char *tag, const uint32_t value, const uint32_t def) {
	config_val *cv = find_config_entry(tag);

	if (cv != NULL) return cv->numeric.i;

	assert(used_entries<MAX_CONFIG_ENTRIES);

	used_entries++;
	sstrcpy(&(cv->tag), tag);
	cv->numeric.i = def;
	cv->type = CONFIG_UINT32;

	return cv->numeric.i;
}

FLT config_read_float(const char *tag, const FLT value, const FLT def) {
	config_val *cv = find_config_entry(tag);

	if (cv != NULL) return cv->numeric.f;

	assert(used_entries<MAX_CONFIG_ENTRIES);

	used_entries++;
	sstrcpy(&(cv->tag), tag);
	cv->numeric.f = def;
	cv->type = CONFIG_FLOAT;

	return cv->numeric.f;
}

config_val* next_unused_val() {
	config_val *cv = config_values+used_entries;
	assert(used_entries<MAX_CONFIG_ENTRIES);
	used_entries++;
	return cv;
}

const char* config_set_str(const char *tag, const char* value) {
	config_val *cv = find_config_entry(tag);
	if (cv == NULL) cv = next_unused_val();

	sstrcpy(&(cv->tag), tag);
	sstrcpy(&(cv->data), value);
	cv->type = CONFIG_STRING;

	return value;
}

const uint32_t config_set_uint32(const char *tag, const uint32_t value) {
	config_val *cv = find_config_entry(tag);
	if (cv == NULL) cv = next_unused_val();

	sstrcpy(&(cv->tag), tag);
	cv->numeric.i = value;
	cv->type = CONFIG_UINT32;

	return value;
}

const FLT config_set_float(const char *tag, const FLT value) {
	config_val *cv = find_config_entry(tag);
	if (cv == NULL) cv = next_unused_val();

	sstrcpy(&(cv->tag), tag);
	cv->numeric.f = value;
	cv->type = CONFIG_FLOAT;

	return value;
}

const FLT* config_set_float_a(const char *tag, const FLT* values, uint8_t count) {
	config_val *cv = find_config_entry(tag);
	if (cv == NULL) cv = next_unused_val();

	sstrcpy(&(cv->tag), tag);

	if (cv->data == NULL) {
		cv->data = (char*)malloc(sizeof(FLT)*count);
	}
	else {
		cv->data = (char*)realloc(cv->data, sizeof(FLT)*count);
	}
	printf("float array\n");

	memcpy(cv->data,values,sizeof(FLT)*count);
	cv->type = CONFIG_FLOAT_ARRAY;
	cv->elements = count;

	return values;
}

void _json_write_float_array(FILE* f, const char* tag, FLT* v, uint8_t count) {
	json_write_double_array(f,tag,v,count);
}

void config_save(const char* path) {
	uint16_t i = 0;

	FILE* f = fopen(path, "w");

	for (i=0;i<=used_entries;++i) {
		if (config_values[i].type == CONFIG_FLOAT) {
			json_write_float(f, config_values[i].tag, config_values[i].numeric.f);
		} else if (config_values[i].type == CONFIG_UINT32) {
			json_write_uint32(f, config_values[i].tag, config_values[i].numeric.i);
		} else if (config_values[i].type == CONFIG_STRING) {
			json_write_str(f, config_values[i].tag, config_values[i].data);
		} else if (config_values[i].type == CONFIG_FLOAT_ARRAY) {
			_json_write_float_array(f, config_values[i].tag, (FLT*)config_values[i].data, config_values[i].elements);
		}
	};

	fclose(f);
}

