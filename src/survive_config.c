// (C) 2017 <>< Joshua Allen, Under MIT/x11 License.
#include <string.h>
#include <assert.h>
#include "survive_config.h"
#include <json_helpers.h>
#if defined(__FreeBSD__) || defined(__APPLE__)
#include <stdlib.h>
#else
#include <malloc.h> //for alloca
#endif
#include <errno.h>

//#define MAX_CONFIG_ENTRIES 100
//#define MAX_LIGHTHOUSES 2

//config_group global_config_values;
//config_group lh_config[MAX_LIGHTHOUSES]; //lighthouse configs

//static uint16_t used_entries = 0;

//static FILE *config_file = NULL;
const FLT* config_set_float_a(config_group *cg, const char *tag, const FLT* values, uint8_t count);

void init_config_entry(config_entry* ce) {
	ce->data = NULL;
	ce->tag = NULL;
	ce->type = CONFIG_UNKNOWN;
	ce->elements = 0;
}

void destroy_config_entry(config_entry* ce) {
	if (ce->tag!=NULL) { free(ce->tag); ce->tag=NULL; }
	if (ce->data!=NULL) { free(ce->data); ce->data=NULL; }
}

void init_config_group(config_group *cg, uint8_t count) {
	uint16_t i = 0;
	cg->used_entries = 0;
	cg->max_entries = count;
	cg->config_entries = NULL;

	if (count==0) return;

	cg->config_entries = malloc(count*sizeof(config_entry));

	for (i=0;i<count;++i) {
		init_config_entry(cg->config_entries+i);
	}
}

void destroy_config_group(config_group* cg) {
	uint16_t i = 0;
	if (cg->config_entries==NULL) return;

	for (i=0;i<cg->max_entries;++i) {
		destroy_config_entry(cg->config_entries+i);
	}

	free(cg->config_entries);
}

void resize_config_group(config_group *cg, uint16_t count) {
	uint16_t i = 0;

	if (count > cg->max_entries) {
		config_entry* ptr = realloc(cg->config_entries, sizeof(config_entry)*count);
		assert(ptr!=NULL);

		cg->config_entries = ptr;

		for (i=cg->max_entries;i<count;++i) {
			init_config_entry(cg->config_entries+i);
		}

		cg->max_entries = count;
	}
}

/*
void config_init() {
	uint16_t i = 0;
	init_config_group(&global_config_values, MAX_CONFIG_ENTRIES);
	for(i=0;i<MAX_LIGHTHOUSES;i++) {
		init_config_group(lh_config+i, 9);
	}
}
*/

void config_read_lighthouse(config_group* lh_config, BaseStationData* bsd, uint8_t idx) {
	config_group *cg = lh_config + idx;
	uint8_t found = 0;
	for (int i = 0; i < NUM_LIGHTHOUSES; i++)
	{
		uint32_t tmpIdx = 0xffffffff;
		cg = lh_config + idx;

		tmpIdx = config_read_uint32(cg, "index", 0xffffffff);

		if (tmpIdx == idx && i == idx) // assumes that lighthouses are stored in the config in order.
		{
			found = 1;
			break;
		}
	}

//	assert(found); // throw an assertion if we didn't find it...  Is this good?  not necessarily?
	if (!found)
	{
		return;
	}


	FLT defaults[7] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

	bsd->BaseStationID = config_read_uint32(cg, "id", 0);
	config_read_float_array(cg, "pose", &bsd->Pose.Pos[0], defaults, 7);
	config_read_float_array(cg, "fcalphase", bsd->fcalphase, defaults, 2);
	config_read_float_array(cg, "fcaltilt", bsd->fcaltilt, defaults, 2);
	config_read_float_array(cg, "fcalcurve", bsd->fcalcurve, defaults, 2);
	config_read_float_array(cg, "fcalgibpha", bsd->fcalgibpha, defaults, 2);
	config_read_float_array(cg, "fcalgibmag", bsd->fcalgibmag, defaults, 2);
	bsd->PositionSet = config_read_uint32(cg, "PositionSet", 0);
}


void config_set_lighthouse(config_group* lh_config, BaseStationData* bsd, uint8_t idx) {
	config_group *cg = lh_config+idx;
	config_set_uint32(cg,"index", idx);
	config_set_uint32(cg,"id", bsd->BaseStationID);
	config_set_float_a(cg,"pose", &bsd->Pose.Pos[0], 7);
	config_set_float_a(cg,"fcalphase", bsd->fcalphase, 2);
	config_set_float_a(cg,"fcaltilt", bsd->fcaltilt,2);
	config_set_float_a(cg,"fcalcurve", bsd->fcalcurve,2);
	config_set_float_a(cg,"fcalgibpha", bsd->fcalgibpha,2);
	config_set_float_a(cg,"fcalgibmag", bsd->fcalgibmag,2);
	config_set_uint32(cg, "PositionSet", bsd->PositionSet);
}

void sstrcpy(char** dest, const char *src) {
	uint32_t len = (uint32_t)strlen(src)+1;
	assert(dest!=NULL);

	char* ptr = (char*)realloc(*dest, len); //acts like malloc if dest==NULL
	assert(ptr!=NULL);
	*dest = ptr;

	strcpy(*dest,src);
}

config_entry* find_config_entry(config_group *cg, const char *tag) {
	uint16_t i = 0;
	for (i=0;i < cg->used_entries;++i) {
		if ( strcmp(cg->config_entries[i].tag, tag) == 0 ) {
			return cg->config_entries+i;
		}
	}
	return NULL;
}

const char* config_read_str(config_group *cg, const char *tag, const char *def) {
	config_entry *cv = find_config_entry(cg, tag);

	if (cv != NULL) return cv->data;

	return config_set_str(cg,tag,def);
}

uint32_t config_read_uint32(config_group *cg, const char *tag, const uint32_t def) {
	config_entry *cv = find_config_entry(cg, tag);

	if (cv != NULL) return cv->numeric.i;

	return config_set_uint32(cg, tag, def);
}

FLT config_read_float(config_group *cg, const char *tag, const FLT def) {
	config_entry *cv = find_config_entry(cg, tag);

	if (cv != NULL) return cv->numeric.f;

	return config_set_float(cg, tag, def);
}

// TODO: Do something better than this:
#define CFG_MIN(x,y) ((x) < (y)? (x): (y))


uint16_t config_read_float_array(config_group *cg, const char *tag, FLT* values, const FLT* def, uint8_t count) {
	config_entry *cv = find_config_entry(cg, tag);

	if (cv != NULL) {
		for (unsigned int i=0; i < CFG_MIN(count, cv->elements); i++)
		{
			values[i] = ((double*)cv->data)[i];
		}
		return cv->elements;
	}

	if (def == NULL) return 0;

	config_set_float_a(cg, tag, def, count);
	for (int i = 0; i < count; i++)
	{
		values[i] = def[i];
	}
	return count;
}

config_entry* next_unused_entry(config_group *cg) {
	config_entry *cv = NULL;
//	assert(cg->used_entries < cg->max_entries);

	if (cg->used_entries >= cg->max_entries) resize_config_group(cg, cg->max_entries + 10);

	cv = cg->config_entries + cg->used_entries;

	cg->used_entries++;
	return cv;
}

const char* config_set_str(config_group *cg, const char *tag, const char* value) {
	config_entry *cv = find_config_entry(cg, tag);
	if (cv == NULL) cv = next_unused_entry(cg);

	sstrcpy(&(cv->tag), tag);

	if (NULL != value){
		sstrcpy(&(cv->data), value);
	}
	else {
		sstrcpy(&(cv->data), "");
	}
	cv->type = CONFIG_STRING;

	return value;
}

const uint32_t config_set_uint32(config_group *cg, const char *tag, const uint32_t value) {
	config_entry *cv = find_config_entry(cg, tag);
	if (cv == NULL) cv = next_unused_entry(cg);

	sstrcpy(&(cv->tag), tag);
	cv->numeric.i = value;
	cv->type = CONFIG_UINT32;

	return value;
}

const FLT config_set_float(config_group *cg, const char *tag, const FLT value) {
	config_entry *cv = find_config_entry(cg, tag);
	if (cv == NULL) cv = next_unused_entry(cg);

	sstrcpy(&(cv->tag), tag);
	cv->numeric.f = value;
	cv->type = CONFIG_FLOAT;

	return value;
}

const FLT* config_set_float_a(config_group *cg, const char *tag, const FLT* values, uint8_t count) {
	config_entry *cv = find_config_entry(cg, tag);
	if (cv == NULL) cv = next_unused_entry(cg);

	sstrcpy(&(cv->tag), tag);

	char* ptr = (char*)realloc(cv->data, sizeof(FLT)*count);
	assert(ptr!=NULL);
	cv->data = ptr;

	memcpy(cv->data,values,sizeof(FLT)*count);
	cv->type = CONFIG_FLOAT_ARRAY;
	cv->elements = count;

	return values;
}

void _json_write_float_array(FILE* f, const char* tag, FLT* v, uint8_t count) {
	#ifdef USE_DOUBLE
		json_write_double_array(f,tag,v,count);
	#else
		json_write_float_array(f,tag,v,count);
	#endif
}

void write_config_group(FILE* f, config_group *cg, char *tag) {
	uint16_t i = 0;

	if (tag != NULL) {
		fprintf(f, "\"%s\":{\n", tag);
	}

	for (i=0;i < cg->used_entries;++i) {
		if (cg->config_entries[i].type == CONFIG_FLOAT) {
			json_write_float(f, cg->config_entries[i].tag, (float)cg->config_entries[i].numeric.f);
		} else if (cg->config_entries[i].type == CONFIG_UINT32) {
			json_write_uint32(f, cg->config_entries[i].tag, cg->config_entries[i].numeric.i);
		} else if (cg->config_entries[i].type == CONFIG_STRING) {
			json_write_str(f, cg->config_entries[i].tag, cg->config_entries[i].data);
		} else if (cg->config_entries[i].type == CONFIG_FLOAT_ARRAY) {
			_json_write_float_array(f, cg->config_entries[i].tag, (FLT*)cg->config_entries[i].data, cg->config_entries[i].elements);
		}
		if ((i+1) < cg->used_entries) fprintf(f,",");
		fprintf(f,"\n");
	};

	if (tag != NULL) {
		fprintf(f,"}\n");
	}
}

//struct SurviveContext;
SurviveContext* survive_context;

void config_save(SurviveContext* sctx, const char* path) {
	uint16_t i = 0;

	FILE* f = fopen(path, "w");

	write_config_group(f,sctx->global_config_values, NULL);
	write_config_group(f,sctx->lh_config, "lighthouse0");
	write_config_group(f,sctx->lh_config+1, "lighthouse1");

	fclose(f);
}

void print_json_value(char* tag, char** values, uint16_t count) {
	uint16_t i = 0;
	for (i=0;i<count; ++i) {
		printf("%s:%s \n", tag, values[i]);
	}
}

config_group* cg_stack[10]; //handle 10 nested objects deep
uint8_t cg_stack_head = 0;

void handle_config_group(char* tag) {
	cg_stack_head++;
	if (strcmp("lighthouse0",tag) == 0) {
		cg_stack[cg_stack_head] = survive_context->lh_config;
	} else if (strcmp("lighthouse1",tag) == 0) {
		cg_stack[cg_stack_head] = survive_context->lh_config+1;
	} else {
		cg_stack[cg_stack_head] = survive_context->global_config_values;
	}
}

void pop_config_group() {
	cg_stack_head--;
}


int parse_floats(char* tag, char** values, uint8_t count) {
	uint16_t i = 0;
	FLT *f;
	f = alloca(sizeof(FLT) * count);
	char* end = NULL;
	config_group* cg = cg_stack[cg_stack_head];

	for(i=0;i<count;++i) {

		#ifdef USE_DOUBLE
		f[i] = strtod(values[i], &end);
		#else
		f[i] = strtof(values[i], &end);
		#endif

//		if (values[i] == end) return 0; //not a float
		if (*end != '\0') return 0; //not an integer
	}

	if (count>1) {
		config_set_float_a(cg, tag, f, count);
	}
	else {
		config_set_float(cg, tag, f[0]);
	}

	return 1;
}

int parse_uint32(char* tag, char** values, uint16_t count) {
	uint16_t i = 0;
	FLT *l;
	l = alloca(sizeof(FLT) * count);
	char* end = NULL;
	config_group* cg = cg_stack[cg_stack_head];

/*
	//look for non numeric values
	for(end=values[0];*end!='\0';++end) {
		if ((*end<48) || (*end>57)) return 0;
	}

	end=NULL;
*/
	for(i=0;i<count;++i) {
		l[i] = strtol(values[i], &end, 10);
//		if (values[i] == end) return 0; //not an integer
		if (*end != '\0') return 0; //not an integer
	}

//	if (count>1)
//		config_set_uint32_array(cg, tag, f, count);
//	else
		config_set_uint32(cg, tag, (uint32_t)l[0]);

	return 1;
}

void handle_tag_value(char* tag, char** values, uint8_t count) {

	//Uncomment for more debugging of input configuration.
	//print_json_value(tag,values,count);

	config_group* cg = cg_stack[cg_stack_head];

	if (NULL != *values){	
	if (parse_uint32(tag,values,count) > 0) return; //parse integers first, stricter rules

	if (parse_floats(tag,values,count) > 0) return;
	}

	//should probably also handle string arrays
	config_set_str(cg,tag,values[0]);
//	else if (count>1) config_set_str
}

void config_read(SurviveContext* sctx, const char* path) {
	survive_context = sctx;

	json_begin_object = handle_config_group;
	json_end_object = pop_config_group;
	json_tag_value = handle_tag_value;

	cg_stack[0] = sctx->global_config_values;

	json_load_file(path);

	json_begin_object = NULL;
	json_end_object = NULL;
	json_tag_value = NULL;
}

