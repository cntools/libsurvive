// (C) 2017 <>< Joshua Allen, Under MIT/x11 License.


#ifndef _SURVIVE_CONFIG_H
#define _SURVIVE_CONFIG_H

#include "survive_internal.h"

typedef enum {
	CONFIG_UNKNOWN = 0,
	CONFIG_FLOAT = 1,
	CONFIG_UINT32 = 2,
	CONFIG_STRING = 3,
	CONFIG_FLOAT_ARRAY = 4,
} cval_type;


typedef struct {
	char *tag;
	cval_type type;
	union {
		uint32_t i;
		FLT f;
	} numeric;
	char *data;
	uint32_t elements;
} config_entry;

typedef struct {
	config_entry *config_entries;
	uint16_t	used_entries;
	uint16_t	max_entries;
} config_group;

extern config_group global_config_values;
extern config_group lh_config[2]; //lighthouse configs


void config_init();
//void config_open(const char* path, const char* mode);
void config_read(const char* path);
//void config_write_lighthouse(struct BaseStationData* bsd, uint8_t length);
void config_set_lighthouse(struct BaseStationData* bsd, uint8_t idx);

void config_save(const char* path);
const FLT config_set_float(config_group *cg, const char *tag, const FLT value);
const uint32_t config_set_uint32(config_group *cg, const char *tag, const uint32_t value);
const char* config_set_str(config_group *cg, const char *tag, const char* value);

FLT config_read_float(config_group *cg, const char *tag, const FLT def);
uint16_t config_read_float_array(config_group *cg, const char *tag, FLT** values, const FLT* def, uint16_t count);
uint32_t config_read_uint32(config_group *cg, const char *tag, const uint32_t def);
const char* config_read_str(config_group *cg, const char *tag, const char *def);

#endif