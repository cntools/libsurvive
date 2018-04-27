// (C) 2017 <>< Joshua Allen, Under MIT/x11 License.
#include "survive_config.h"
#include <assert.h>
#include <json_helpers.h>
#include <string.h>
#if defined(__FreeBSD__) || defined(__APPLE__)
#include <stdlib.h>
#else
#include <malloc.h> //for alloca
#endif
#include "math.h"
#include <errno.h>
#include <stdarg.h>

//Static-time registration system.

struct static_conf_t
{
	union
	{
		FLT f;
		int i;
		char * s;
	} data_default;
	const char * name;
	const char * description;
	char type;
};

static struct static_conf_t static_configs[MAX_SHORTHAND_CONFIGS];

void survive_config_bind_variable( char vt, int * variable, const char * name, const char * description, ... )
{
	va_list ap;
	va_start(ap, description); 
	int i;
	struct static_conf_t * config;
	for( i = 0; i < MAX_SHORTHAND_CONFIGS; i++ )
	{
		config = &static_configs[i];
		if( !config->name || strcmp( config->name, name ) == 0 ) break;
	}
	if( i == MAX_SHORTHAND_CONFIGS )
	{
		fprintf( stderr, "Fatal: Too many static configuration items. Please recompile with a higher MAX_STATIC_CONFIGS\n" );
		exit( -1 );
	}
	if( !config->description ) config->description = description;
	if( !config->name ) config->name = name;
	if( config->type && config->type != vt )
	{
		fprintf( stderr, "Fatal: Internal error on variable %s.  Types disagree.\n", name ); 
		exit( -2 );
	}
	config->type = vt;
	switch( vt )
	{
	case 'i': config->data_default.i = va_arg(ap, int); break;
	case 'f': config->data_default.f = va_arg(ap, FLT); break;
	case 's': config->data_default.s = va_arg(ap, char *); break;
	default:
		fprintf( stderr, "Fatal: Internal error on variable %s.  Unknown type %c\n", name, vt );
	}
	*variable = i;
}

void survive_config_populate_ctx( SurviveContext * ctx )
{
	int i;
	struct static_conf_t * config;
	for( i = 0; i < MAX_SHORTHAND_CONFIGS; i++ )
	{
		config = &static_configs[i];
		switch( config->type )
		{
		case 'i': ctx->shorthand_configs[i].i = config->data_default.i;
		case 'f': ctx->shorthand_configs[i].f = config->data_default.f;
		case 's': ctx->shorthand_configs[i].s = config->data_default.s;
		}
	}
}

void survive_print_known_configs( SurviveContext * ctx )
{
	int i;
	struct static_conf_t * config;
	for( i = 0; i < MAX_SHORTHAND_CONFIGS; i++ )
	{
		config = &static_configs[i];
		if( !config->name ) break;
		switch( config->type )
		{
		case 'i':	printf( "%10d %20s  %s\n", config->data_default.i, config->name, config->description ); break;
		case 'f':	printf( "%10f %20s  %s\n", config->data_default.f, config->name, config->description ); break;
		case 's':	printf( "%10s %20s  %s\n", config->data_default.s, config->name, config->description ); break;
		}
	}

	//XXX TODO: Maybe this should print out the actual config values after being updated from the rest of the config system?
	//struct config_group *global_config_values;
	//struct config_group	*temporary_config_values; // Set per-session, from command-line. Not saved but override global_config_values
}






const FLT *config_set_float_a(config_group *cg, const char *tag, const FLT *values, uint8_t count);

void init_config_entry(config_entry *ce) {
	ce->data = NULL;
	ce->tag = NULL;
	ce->type = CONFIG_UNKNOWN;
	ce->elements = 0;
	ce->shorthand_place = -1;
}

void destroy_config_entry(config_entry *ce) {
	if (ce->tag != NULL) {
		free(ce->tag);
		ce->tag = NULL;
	}
	if (ce->data != NULL) {
		free(ce->data);
		ce->data = NULL;
	}
}

void init_config_group(config_group *cg, uint8_t count, SurviveContext * ctx) {
	uint16_t i = 0;
	cg->used_entries = 0;
	cg->max_entries = count;
	cg->config_entries = NULL;
	cg->ctx = ctx;

	if (count == 0)
		return;

	cg->config_entries = malloc(count * sizeof(config_entry));

	for (i = 0; i < count; ++i) {
		init_config_entry(cg->config_entries + i);
	}
}

void destroy_config_group(config_group *cg) {
	uint16_t i = 0;
	if (cg->config_entries == NULL)
		return;

	for (i = 0; i < cg->max_entries; ++i) {
		destroy_config_entry(cg->config_entries + i);
	}

	free(cg->config_entries);
}

void resize_config_group(config_group *cg, uint16_t count) {
	uint16_t i = 0;

	if (count > cg->max_entries) {
		config_entry *ptr = realloc(cg->config_entries, sizeof(config_entry) * count);
		assert(ptr != NULL);

		cg->config_entries = ptr;

		for (i = cg->max_entries; i < count; ++i) {
			init_config_entry(cg->config_entries + i);
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

void config_read_lighthouse(config_group *lh_config, BaseStationData *bsd, uint8_t idx) {
	config_group *cg = lh_config + idx;
	uint8_t found = 0;
	for (int i = 0; i < NUM_LIGHTHOUSES; i++) {
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
	if (!found) {
		return;
	}

	FLT defaults[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	bsd->BaseStationID = config_read_uint32(cg, "id", 0);
	bsd->mode = config_read_uint32(cg, "mode", 0);
	config_read_float_array(cg, "pose", &bsd->Pose.Pos[0], defaults, 7);
	config_read_float_array(cg, "fcalphase", bsd->fcal.phase, defaults, 2);
	config_read_float_array(cg, "fcaltilt", bsd->fcal.tilt, defaults, 2);
	config_read_float_array(cg, "fcalcurve", bsd->fcal.curve, defaults, 2);
	config_read_float_array(cg, "fcalgibpha", bsd->fcal.gibpha, defaults, 2);
	config_read_float_array(cg, "fcalgibmag", bsd->fcal.gibmag, defaults, 2);
	bsd->PositionSet = config_read_uint32(cg, "PositionSet", 0);
}

void config_set_lighthouse(config_group *lh_config, BaseStationData *bsd, uint8_t idx) {
	config_group *cg = lh_config + idx;
	config_set_uint32(cg, "index", idx);
	config_set_uint32(cg, "id", bsd->BaseStationID);
	config_set_uint32(cg, "mode", bsd->mode);
	config_set_float_a(cg, "pose", &bsd->Pose.Pos[0], 7);
	config_set_float_a(cg, "fcalphase", bsd->fcal.phase, 2);
	config_set_float_a(cg, "fcaltilt", bsd->fcal.tilt, 2);
	config_set_float_a(cg, "fcalcurve", bsd->fcal.curve, 2);
	config_set_float_a(cg, "fcalgibpha", bsd->fcal.gibpha, 2);
	config_set_float_a(cg, "fcalgibmag", bsd->fcal.gibmag, 2);
	config_set_uint32(cg, "PositionSet", bsd->PositionSet);
}

void sstrcpy(char **dest, const char *src) {
	uint32_t len = (uint32_t)strlen(src) + 1;
	assert(dest != NULL);

	char *ptr = (char *)realloc(*dest, len); // acts like malloc if dest==NULL
	assert(ptr != NULL);
	*dest = ptr;

	strcpy(*dest, src);
}

config_entry *find_config_entry(config_group *cg, const char *tag) {
	uint16_t i = 0;
	for (i = 0; i < cg->used_entries; ++i) {
		if (strcmp(cg->config_entries[i].tag, tag) == 0) {
			return cg->config_entries + i;
		}
	}
	return NULL;
}

const char *config_read_str(config_group *cg, const char *tag, const char *def) {
	config_entry *cv = find_config_entry(cg, tag);

	if (cv != NULL)
		return cv->data;

	return config_set_str(cg, tag, def);
}

uint32_t config_read_uint32(config_group *cg, const char *tag, const uint32_t def) {
	config_entry *cv = find_config_entry(cg, tag);

	if (cv != NULL)
		return cv->numeric.i;

	return config_set_uint32(cg, tag, def);
}

FLT config_read_float(config_group *cg, const char *tag, const FLT def) {
	config_entry *cv = find_config_entry(cg, tag);

	if (cv != NULL)
		return cv->numeric.f;

	return config_set_float(cg, tag, def);
}

// TODO: Do something better than this:
#define CFG_MIN(x, y) ((x) < (y) ? (x) : (y))

uint16_t config_read_float_array(config_group *cg, const char *tag, FLT *values, const FLT *def, uint8_t count) {
	config_entry *cv = find_config_entry(cg, tag);

	if (cv != NULL) {
		for (unsigned int i = 0; i < CFG_MIN(count, cv->elements); i++) {
			values[i] = ((double *)cv->data)[i];
		}
		return cv->elements;
	}

	if (def == NULL)
		return 0;

	config_set_float_a(cg, tag, def, count);
	for (int i = 0; i < count; i++) {
		values[i] = def[i];
	}
	return count;
}

config_entry *next_unused_entry(config_group *cg, const char * tag) {
	config_entry *cv = NULL;
	//	assert(cg->used_entries < cg->max_entries);

	if (cg->used_entries >= cg->max_entries)
		resize_config_group(cg, cg->max_entries + 10);

	cv = cg->config_entries + cg->used_entries;

	cg->used_entries++;


	int i;
	struct static_conf_t * config;
	for( i = 0; i < MAX_SHORTHAND_CONFIGS; i++ )
	{
		config = &static_configs[i];
		if( !config->name ) break;
		if( strcmp( config->name, tag ) == 0 )
		{
			cv->shorthand_place = i;
			break;
		}
	}

	return cv;
}

const char *config_set_str(config_group *cg, const char *tag, const char *value) {
	config_entry *cv = find_config_entry(cg, tag);
	if (cv == NULL)
		cv = next_unused_entry(cg,tag);

	sstrcpy(&(cv->tag), tag);

	if (NULL != value) {
		sstrcpy(&(cv->data), value);
	} else {
		sstrcpy(&(cv->data), "");
	}
	cv->type = CONFIG_STRING;

	if( cv->shorthand_place >= 0 )	cg->ctx->shorthand_configs[cv->shorthand_place].s = value;

	return value;
}

const uint32_t config_set_uint32(config_group *cg, const char *tag, const uint32_t value) {
	config_entry *cv = find_config_entry(cg, tag);
	if (cv == NULL)
		cv = next_unused_entry(cg,tag);

	sstrcpy(&(cv->tag), tag);
	cv->numeric.i = value;
	cv->type = CONFIG_UINT32;

	if( cv->shorthand_place >= 0 )	cg->ctx->shorthand_configs[cv->shorthand_place].i = value;

	return value;
}

const FLT config_set_float(config_group *cg, const char *tag, const FLT value) {
	config_entry *cv = find_config_entry(cg, tag);
	if (cv == NULL)
		cv = next_unused_entry(cg,tag);

	sstrcpy(&(cv->tag), tag);
	cv->numeric.f = value;
	cv->type = CONFIG_FLOAT;

	if( cv->shorthand_place >= 0 )	cg->ctx->shorthand_configs[cv->shorthand_place].f = value;

	return value;
}

const FLT *config_set_float_a(config_group *cg, const char *tag, const FLT *values, uint8_t count) {
	config_entry *cv = find_config_entry(cg, tag);
	if (cv == NULL)
		cv = next_unused_entry(cg,tag);

	sstrcpy(&(cv->tag), tag);

	char *ptr = (char *)realloc(cv->data, sizeof(FLT) * count);
	assert(ptr != NULL);
	cv->data = ptr;

	memcpy(cv->data, values, sizeof(FLT) * count);
	cv->type = CONFIG_FLOAT_ARRAY;
	cv->elements = count;

	return values;
}

void _json_write_float_array(FILE *f, const char *tag, FLT *v, uint8_t count) {
#ifdef USE_DOUBLE
	json_write_double_array(f, tag, v, count);
#else
	json_write_float_array(f, tag, v, count);
#endif
}

void write_config_group(FILE *f, config_group *cg, char *tag) {
	uint16_t i = 0;

	if (tag != NULL) {
		fprintf(f, "\"%s\":{\n", tag);
	}

	for (i = 0; i < cg->used_entries; ++i) {
		if (cg->config_entries[i].type == CONFIG_FLOAT) {
			json_write_float(f, cg->config_entries[i].tag, (float)cg->config_entries[i].numeric.f);
		} else if (cg->config_entries[i].type == CONFIG_UINT32) {
			json_write_uint32(f, cg->config_entries[i].tag, cg->config_entries[i].numeric.i);
		} else if (cg->config_entries[i].type == CONFIG_STRING) {
			json_write_str(f, cg->config_entries[i].tag, cg->config_entries[i].data);
		} else if (cg->config_entries[i].type == CONFIG_FLOAT_ARRAY) {
			_json_write_float_array(f, cg->config_entries[i].tag, (FLT *)cg->config_entries[i].data,
									cg->config_entries[i].elements);
		}
		if ((i + 1) < cg->used_entries)
			fprintf(f, ",");
		fprintf(f, "\n");
	};

	if (tag != NULL) {
		fprintf(f, "}\n");
	}
}

// struct SurviveContext;
SurviveContext *survive_context;

void config_save(SurviveContext *sctx, const char *path) {
	uint16_t i = 0;

	FILE *f = fopen(path, "w");

	write_config_group(f, sctx->global_config_values, NULL);
	write_config_group(f, sctx->lh_config, "lighthouse0");
	write_config_group(f, sctx->lh_config + 1, "lighthouse1");

	fclose(f);
}

void print_json_value(char *tag, char **values, uint16_t count) {
	uint16_t i = 0;
	for (i = 0; i < count; ++i) {
		printf("%s:%s \n", tag, values[i]);
	}
}

config_group *cg_stack[10]; // handle 10 nested objects deep
uint8_t cg_stack_head = 0;

void handle_config_group(char *tag) {
	cg_stack_head++;
	if (strcmp("lighthouse0", tag) == 0) {
		cg_stack[cg_stack_head] = survive_context->lh_config;
	} else if (strcmp("lighthouse1", tag) == 0) {
		cg_stack[cg_stack_head] = survive_context->lh_config + 1;
	} else {
		cg_stack[cg_stack_head] = survive_context->global_config_values;
	}
}

void pop_config_group() { cg_stack_head--; }

int parse_floats(char *tag, char **values, uint8_t count) {
	uint16_t i = 0;
	FLT *f;
	f = alloca(sizeof(FLT) * count);
	char *end = NULL;
	config_group *cg = cg_stack[cg_stack_head];

	for (i = 0; i < count; ++i) {

#ifdef USE_DOUBLE
		f[i] = strtod(values[i], &end);
#else
		f[i] = strtof(values[i], &end);
#endif

		//		if (values[i] == end) return 0; //not a float
		if (*end != '\0')
			return 0; // not an integer
	}

	if (count > 1) {
		config_set_float_a(cg, tag, f, count);
	} else {
		config_set_float(cg, tag, f[0]);
	}

	return 1;
}

int parse_uint32(char *tag, char **values, uint16_t count) {
	uint16_t i = 0;
	FLT *l;
	l = alloca(sizeof(FLT) * count);
	char *end = NULL;
	config_group *cg = cg_stack[cg_stack_head];

	/*
		//look for non numeric values
		for(end=values[0];*end!='\0';++end) {
			if ((*end<48) || (*end>57)) return 0;
		}

		end=NULL;
	*/
	for (i = 0; i < count; ++i) {
		l[i] = strtol(values[i], &end, 10);
		//		if (values[i] == end) return 0; //not an integer
		if (*end != '\0')
			return 0; // not an integer
	}

	//	if (count>1)
	//		config_set_uint32_array(cg, tag, f, count);
	//	else
	config_set_uint32(cg, tag, (uint32_t)l[0]);

	return 1;
}

void handle_tag_value(char *tag, char **values, uint8_t count) {

	// Uncomment for more debugging of input configuration.
	// print_json_value(tag,values,count);

	config_group *cg = cg_stack[cg_stack_head];

	if (NULL != *values) {
		if (parse_uint32(tag, values, count) > 0)
			return; // parse integers first, stricter rules

		if (parse_floats(tag, values, count) > 0)
			return;
	}

	// should probably also handle string arrays
	config_set_str(cg, tag, values[0]);
	//	else if (count>1) config_set_str
}

void config_read(SurviveContext *sctx, const char *path) {
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

static config_entry *sc_search(SurviveContext *ctx, const char *tag) {
	config_entry *cv = find_config_entry(ctx->temporary_config_values, tag);
	if (!cv) {
		cv = find_config_entry(ctx->global_config_values, tag);
	}
	return cv;
}

static FLT config_entry_as_FLT(config_entry *entry) {
	switch (entry->type) {
	case CONFIG_FLOAT:
		return (uint32_t)roundf((float)entry->numeric.f);
	case CONFIG_UINT32:
		return (FLT)entry->numeric.i;
	case CONFIG_STRING:
		return (FLT)atof(entry->data);
	case CONFIG_FLOAT_ARRAY:
	case CONFIG_UNKNOWN:
		break;
	}
	return 0;
}

static uint32_t config_entry_as_uint32_t(config_entry *entry) {
	switch (entry->type) {
	case CONFIG_FLOAT:
		return (uint32_t)roundf((float)entry->numeric.f);
	case CONFIG_UINT32:
		return entry->numeric.i;
	case CONFIG_STRING:
		return (uint32_t)atoi(entry->data);
	case CONFIG_FLOAT_ARRAY:
	case CONFIG_UNKNOWN:
		break;
	}
	return 0;
}

FLT survive_configf(SurviveContext *ctx, const char *tag, char flags, FLT def) {
	if (!(flags & SC_OVERRIDE)) {
		config_entry *cv = sc_search(ctx, tag);
		if (cv) {
			return config_entry_as_FLT(cv);
		}
	}

	// If override is flagged, or, we can't find the variable, ,continue on.
	if (flags & SC_SETCONFIG) {
		config_set_float(ctx->temporary_config_values, tag, def);
		config_set_float(ctx->global_config_values, tag, def);
	} else if (flags & SC_SET) {
		config_set_float(ctx->temporary_config_values, tag, def);
	}

	return def;
}

uint32_t survive_configi(SurviveContext *ctx, const char *tag, char flags, uint32_t def) {
	if (!(flags & SC_OVERRIDE)) {
		config_entry *cv = sc_search(ctx, tag);
		if (cv) {
			return config_entry_as_uint32_t(cv);
		}
	}

	// If override is flagged, or, we can't find the variable, ,continue on.
	if (flags & SC_SETCONFIG) {
		config_set_uint32(ctx->temporary_config_values, tag, def);
		config_set_uint32(ctx->global_config_values, tag, def);
	} else if (flags & SC_SET) {
		config_set_uint32(ctx->temporary_config_values, tag, def);
	}

	return def;
}

const char *survive_configs(SurviveContext *ctx, const char *tag, char flags, const char *def) {
	if (!(flags & SC_OVERRIDE)) {
		config_entry *cv = sc_search(ctx, tag);
		if (cv)
			return cv->data;
	}

	// If override is flagged, or, we can't find the variable, ,continue on.
	if (flags & SC_SETCONFIG) {
		config_set_str(ctx->temporary_config_values, tag, def);
		config_set_str(ctx->global_config_values, tag, def);
	} else if (flags & SC_SET) {
		config_set_str(ctx->temporary_config_values, tag, def);
	}

	return def;
}


