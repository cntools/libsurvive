// (C) 2017 <>< Joshua Allen, Under MIT/x11 License.

#include <string.h>
#include <assert.h>
#include "survive_config.h"

#define MAX_CONFIG_ENTRIES 100

config_val config_values[MAX_CONFIG_ENTRIES];
static uint16_t used_entries = 0;

static FILE *config_file = NULL;

void config_init() {
	uint16_t i = 0;
	for (i=0;i<MAX_CONFIG_ENTRIES;++i) {
		config_values[i].str = NULL;
		config_values[i].tag = NULL;
		config_values[i].type = CONFIG_UNKNOWN;
	}

	used_entries = 0;
}

void write_float(char* tag, FLT x) {
	fprintf(config_file, "\"%s\":\"%f\"\n", tag, x);
}

void write_float_a(char* tag, FLT *x, uint8_t count) {
	uint8_t i = 0;
	char idx[4];
	for (i=0;i<count;++i) {
		sprintf(idx,"%d",i);
		fprintf(config_file, "\"%s%s\":\"%f\"\n", tag,idx, x);
	}
	fprintf(config_file, "\"%s\":\"%f\"\n", tag, x);
}

void write_uint32(char* tag, uint32_t x) {
	fprintf(config_file, "\"%s\":\"%d\"\n", tag, x);
}

void config_open(const char* path, const char* mode) {
	config_file = fopen(path, mode);
}

void config_close() {
	fclose(config_file);
}

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

void sstrcpy(char* dest, const char *src) {
	uint32_t len = strlen(src)+1;
	if (dest == NULL) {
		dest = (char*)malloc(len);
	} else {
		dest = (char*)realloc(dest, len);
	}
	strcpy(dest,src);
}

const char* config_read_str(const char *tag, const char *value, const char *def_str) {
	uint16_t i = 0;
	for (i=0;i<used_entries;++i) {
		if ( strcmp(config_values[i].tag, tag) == 0 ) {
			return config_values[i].str;
		}
	}
	assert(used_entries<MAX_CONFIG_ENTRIES);

	i = used_entries++;
	sstrcpy(config_values[i].tag, tag);
	sstrcpy(config_values[i].str, def_str);
	config_values[i].type = CONFIG_STRING;

	return config_values[i].str;
}

uint32_t config_read_uint32(const char *tag, const uint32_t value, const uint32_t def) {
	uint16_t i = 0;
	for (i=0;i<used_entries;++i) {
		if ( strcmp(config_values[i].tag, tag) == 0 ) {
			return config_values[i].numeric.i;
		}
	}
	assert(used_entries<MAX_CONFIG_ENTRIES);

	i = used_entries++;
	sstrcpy(config_values[i].tag, tag);
	config_values[i].numeric.i = def;
	config_values[i].type = CONFIG_UINT32;

	return config_values[i].numeric.i;
}

FLT config_read_float(const char *tag, const FLT value, const FLT def) {
	uint16_t i = 0;
	for (i=0;i<used_entries;++i) {
		if ( strcmp(config_values[i].tag, tag) == 0 ) {
			return config_values[i].numeric.f;
		}
	}
	assert(used_entries<MAX_CONFIG_ENTRIES);

	i = used_entries++;
	sstrcpy(config_values[i].tag, tag);
	config_values[i].numeric.f = def;
	config_values[i].type = CONFIG_FLOAT;

	return config_values[i].numeric.f;
}

const char* config_set_str(const char *tag, const char* value) {
	uint16_t i = 0;

	assert(used_entries<MAX_CONFIG_ENTRIES);

	i = used_entries++;
	sstrcpy(config_values[i].tag, tag);
	sstrcpy(config_values[i].str, value);
	config_values[i].type = CONFIG_STRING;

	return value;
}

const uint32_t config_set_uint32(const char *tag, const uint32_t value) {
	uint16_t i = 0;

	assert(used_entries<MAX_CONFIG_ENTRIES);

	i = used_entries++;
	sstrcpy(config_values[i].tag, tag);
	config_values[i].numeric.i = value;
	config_values[i].type = CONFIG_UINT32;

	return value;
}

const FLT config_set_float(const char *tag, const FLT value) {
	uint16_t i = 0;

	assert(used_entries<MAX_CONFIG_ENTRIES);

	i = used_entries++;
	sstrcpy(config_values[i].tag, tag);
	config_values[i].numeric.f = value;
	config_values[i].type = CONFIG_FLOAT;

	return value;
}
void config_save(const char* path) {
	uint16_t i = 0;

	FILE* f = fopen(path, "w");

	for (i=0;i<=used_entries;++i) {
		if (config_values[i].type == CONFIG_FLOAT) {
			fprintf(f, "\"%s\":\"%F\"\n", config_values[i].tag, config_values[i].numeric.f);
		} else if (config_values[i].type == CONFIG_UINT32) {
			fprintf(f, "\"%s\":\"%d\"\n", config_values[i].tag, config_values[i].numeric.i);
		} else if (config_values[i].type == CONFIG_STRING) {
			fprintf(f, "\"%s\":\"%s\"\n", config_values[i].tag, config_values[i].str);
		}
	};

	fclose(f);
}

