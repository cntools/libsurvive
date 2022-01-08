// (C) 2017 <>< Joshua Allen, Under MIT/x11 License.
//
// This header is for handling internal parameter values.  Most accesses should be done through functions like survive_config
//

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

struct update_list_t_s
{
	void * value;
	struct update_list_t_s * next;
}; 

typedef struct update_list_t_s update_list_t;


typedef struct {
	char *tag;
	cval_type type;
	union {
		uint32_t i;
		FLT f;
	} numeric;
	char *data;
	uint32_t elements;

	update_list_t * update_list;
} config_entry;

typedef struct config_group {
	config_entry *config_entries;
	uint16_t	used_entries;
	uint16_t	max_entries;
	og_mutex_t write_lock;
	SurviveContext * ctx;
} config_group;

//extern config_group global_config_values;
//extern config_group lh_config[2]; //lighthouse configs

void init_config_group(config_group *cg, uint8_t count, SurviveContext * ctx);
void destroy_config_group(config_group* cg);

//void config_init();
//void config_open(const char* path, const char* mode);

//void config_write_lighthouse(struct BaseStationData* bsd, uint8_t length);
void config_set_lighthouse(config_group* lh_config, BaseStationData* bsd, uint8_t idx);
bool config_read_lighthouse(config_group *lh_config, BaseStationData *bsd, uint8_t idx);

void config_read(SurviveContext* sctx, const char* path);
void config_save(SurviveContext *ctx);

FLT config_set_float(config_group *cg, const char *tag, FLT value);
uint32_t config_set_uint32(config_group *cg, const char *tag, uint32_t value);
const char* config_set_str(config_group *cg, const char *tag, const char* value);

//These functions look for a parameter in a specific group, and then chose the best to return. If the parameter does not exist, default will be written.
FLT config_read_float(config_group *cg, const char *tag, FLT def);
uint16_t config_read_float_array(config_group *cg, const char *tag, FLT* values, const FLT* def, uint8_t count);
uint32_t config_read_uint32(config_group *cg, const char *tag, uint32_t def);
const char* config_read_str(config_group *cg, const char *tag, const char *def);

//These are for the internal non-function configuration system.
void survive_config_bind_variable( char vt, const char * name, const char * description, ... );
void survive_print_known_configs( SurviveContext * ctx, int verbose );
void survive_config_populate_ctx( SurviveContext * ctx );
int survive_print_help_for_parameter(SurviveContext *ctx, const char *tomap);


#endif
