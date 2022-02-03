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

#include "ctype.h"
#include "math.h"
#include <errno.h>
#include <stdarg.h>
#include <sys/stat.h>

//Static-time registration system.

struct static_conf_t
{
	union
	{
		FLT f;
		int i;
		char * s;
		bool b;
	} data_default;
	const char * name;
	const char * description;
	char type;

	struct static_conf_t *next;
};

static struct static_conf_t *head = 0;
static struct static_conf_t *tail = 0;

static struct static_conf_t *find_static_conf_t(const char *name) {
	struct static_conf_t *curr = head;
	while (curr) {
		if (strcmp(curr->name, name) == 0)
			break;
		curr = curr->next;
	}

	return curr;
}

static struct static_conf_t *find_or_create_conf_t(const char *name) {
	struct static_conf_t *curr = find_static_conf_t(name);

	if (curr) {
		return curr;
	}

	curr = SV_CALLOC(sizeof(struct static_conf_t));
	if (tail)
		tail->next = curr;
	if (head == 0)
		head = curr;

	tail = curr;
	return curr;
}

void survive_config_bind_variable( char vt, const char * name, const char * description, ... )
{
	va_list ap;
	va_start(ap, description);

	struct static_conf_t *existing = find_static_conf_t(name);
	assert(existing == 0);
	struct static_conf_t *config = find_or_create_conf_t(name);

	if( !config->description ) config->description = description;
	if( !config->name ) config->name = name;
	if( config->type && config->type != vt )
	{
		fprintf(stderr, "Fatal: Internal error on variable %s.  Types disagree [%c/%c].\n", name, config->type, vt);
		exit( -2 );
	}
	config->type = vt;

	switch( vt )
	{
	case 'i': config->data_default.i = va_arg(ap, int); break;
	case 'b': config->data_default.b = va_arg(ap, int); break;
	case 'f':
		config->data_default.f = va_arg(ap, double);
		break;
	case 's': config->data_default.s = va_arg(ap, char *); break;
	default:
		fprintf( stderr, "Fatal: Internal error on variable %s.  Unknown type %c\n", name, vt );
	}

	uint32_t marker = va_arg(ap, uint32_t);
	if (marker != 0xcafebeef) {
		fprintf(stderr, "Fatal: Internal error on variable %s.\n", name);
		fprintf(stderr,
				"This is happening because the default value doesn't have the same type as the indicated type.\n");
		fprintf(stderr, "Note that 'f' types MUST be seen as float/double type to the compiler; ie '1.' and not '1'\n");
		exit(-2);
	}
	va_end(ap);
}

SURVIVE_EXPORT void survive_config_bind_variablei(const char *name, const char *description, int def) {
	survive_config_bind_variable('i', name, description, def, 0xcafebeef);
}
SURVIVE_EXPORT void survive_config_bind_variablef(const char *name, const char *description, FLT def) {
	survive_config_bind_variable('f', name, description, def, 0xcafebeef);
}
SURVIVE_EXPORT void survive_config_bind_variableb(const char *name, const char *description, bool def) {
    survive_config_bind_variable('b', name, description, def, 0xcafebeef);
}

int survive_print_help_for_parameter(SurviveContext *ctx, const char *tomap) {
	for (struct static_conf_t *config = head; config; config = config->next) {
		if (strcmp(config->name, tomap) == 0) {
			char val[128];
			survive_config_as_str(ctx, val, 128, config->name, "");
			char sthelp[160];
			snprintf(sthelp, 159, "    %s: %s \t\tdefault: %s\t\t(%c)", config->name, config->description, val,
					 config->type);

			fprintf(stderr, "\0337\033[1A\033[1000D\033[K%s\0338", sthelp);
			return 1;
		}
	}
	return 0;
}

#define USAGE_FORMAT " --%-40s"
static const char *USAGE_FORMAT_BOOL = "%15d    ";
static const char *USAGE_FORMAT_INT = "%15d    ";
static const char *USAGE_FORMAT_FLOAT = "%15f    ";
static const char *USAGE_FORMAT_SMALL_FLOAT = "%15e    ";
static const char *USAGE_FORMAT_STRING = "%15s    ";

void survive_config_entry_to_str(config_entry *config, char *stobuf, size_t len) {
	switch (config->type) {
	case CONFIG_UINT32:
		snprintf(stobuf, 127, USAGE_FORMAT_INT, config->numeric.i);
		break;
	case CONFIG_FLOAT: {
		FLT df = config->numeric.f;
		if (fabs(df) > .001 || df == 0) {
			snprintf(stobuf, 127, USAGE_FORMAT_FLOAT, df);
		} else {
			snprintf(stobuf, 127, USAGE_FORMAT_SMALL_FLOAT, df);
		}
	} break;
	case CONFIG_STRING:
		snprintf(stobuf, 127, USAGE_FORMAT_STRING, config->data);
		break;
	case CONFIG_FLOAT_ARRAY:
		snprintf(stobuf, 127, "[FA] %25s  %s\n", config->tag, "");
		break;
	default:
		assert("Invalid config item" && false);
	}
}
void survive_default_to_str(struct static_conf_t *config, char *stobuf, size_t len) {
	switch (config->type) {
	case 'b':
		snprintf(stobuf, 127, USAGE_FORMAT_BOOL, config->data_default.b);
		break;
	case 'i':
		snprintf(stobuf, 127, USAGE_FORMAT_INT, config->data_default.i);
		break;
	case 'f': {
		FLT df = config->data_default.f;
		if (fabs(df) > .001 || df == 0) {
			snprintf(stobuf, 127, USAGE_FORMAT_FLOAT, df);
		} else {
			snprintf(stobuf, 127, USAGE_FORMAT_SMALL_FLOAT, df);
		}
		break;
	}
	case 's':
		snprintf(stobuf, 127, USAGE_FORMAT_STRING, config->data_default.s);
		break;
	case 'a':
		snprintf(stobuf, 127, "[FA] %25s  %s\n", config->name, config->description);
		break;
	default:
		assert("Invalid config item" && false);
	}
}

void survive_config_iterate(SurviveContext *ctx, survive_config_iterate_fn fn, void *user) {
	char buffer[128];
	for (struct static_conf_t *config = head; config; config = config->next) {
		survive_default_to_str(config, buffer, 128);
		fn(ctx, config->name, config->type, config->description, buffer, user);
	}
}

static const char *survive_config_type_to_str(cval_type type) {
	switch (type) {
	case CONFIG_FLOAT:
		return ":float";
	case CONFIG_UINT32:
		return ":int";
	case CONFIG_STRING:
		return ":string";
	}
	return ".";
}
static const char *type_to_desc(char config_type) {
	switch (config_type) {
	case 'f':
		return ":float";
	case 'i':
		return ":int";
	case 's':
		return ":string";
	case 'b':
		return ":bool";
	}
	return ".";
}

static void PrintConfigGroup(config_group * grp, const char ** chkval, int * cvs, int verbose )
{
	int i, j;
	for( i = 0; i < grp->used_entries; i++ )
	{
		config_entry * ce = &grp->config_entries[i];
		for( j = 0; j < *cvs; j++ )
		{
			if( strcmp( chkval[j], ce->tag ) == 0 ) break;
		}
		if( j == *cvs )
		{
			if( verbose )
			{
				char stobuf[128];
				survive_config_entry_to_str(ce, stobuf, 128);

				const char *description = "";
				for (struct static_conf_t *config = head; config; config = config->next) {
					if (strcmp(config->name, ce->tag) == 0) {
						description = config->description;
					}
				}
				printf(USAGE_FORMAT "%s %-12s     %s\n", ce->tag, stobuf, survive_config_type_to_str(ce->type),
					   description);
			}
			else
			{
				printf( "--%s ", ce->tag );
			}
			chkval[*cvs] = ce->tag;
			(*cvs)++;
		}
	}
}


void survive_print_known_configs( SurviveContext * ctx, int verbose )
{
	int i;

	const char * checked_values[256];
	int cvs = 0;
	memset( checked_values, 0, sizeof( checked_values ) );
	PrintConfigGroup( ctx->temporary_config_values, checked_values, &cvs, verbose );
	PrintConfigGroup( ctx->global_config_values, checked_values, &cvs, verbose );

	for (struct static_conf_t *config = head; config; config = config->next) {
		for( i = 0; i < cvs; i++ )
		{
			if (strcmp(config->name, checked_values[i]) == 0)
				break;
		}
		if( i == cvs )
		{
			const char * name = config->name;

			if( verbose )
			{
				char stobuf[128];
				survive_default_to_str(config, stobuf, 128);

				const char *type_desc = type_to_desc(config->type);
				printf(USAGE_FORMAT "%s %-12s     %s\n", name, stobuf, type_desc, config->description);
			}
			else
			{
				printf( "--%s ", name );

				// Also display '--no-*' formatted options. We only do this for 'i' types since that is how we represent
				// flags internally; and it'd be sorta useless to have them show up for default-off options. If the
				// default is not 1, it's either not a flag or already off.
				if ((config->type == 'i' && config->data_default.i == 1) ||
                    (config->type == 'b' && config->data_default.b == 1))
					printf("--no-%s ", name);
			}
		}
	}
}

const FLT *config_set_float_a(config_group *cg, const char *tag, const FLT *values, uint8_t count);

void init_config_entry(config_entry *ce) {
	ce->data = NULL;
	ce->tag = NULL;
	ce->type = CONFIG_UNKNOWN;
	ce->elements = 0;
	ce->update_list = 0;
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
	cg->write_lock = OGCreateMutex();

	cg->used_entries = 0;
	cg->max_entries = count;
	cg->config_entries = NULL;
	cg->ctx = ctx;

	if (count == 0)
		return;

	cg->config_entries = SV_MALLOC(count * sizeof(config_entry));

	for (i = 0; i < count; ++i) {
		init_config_entry(cg->config_entries + i);
	}
}

void destroy_config_group(config_group *cg) {
	uint16_t i = 0;
	if (cg->config_entries == NULL)
		return;

	cg->used_entries = 0;
	for (i = 0; i < cg->max_entries; ++i) {
		destroy_config_entry(cg->config_entries + i);
	}
	OGDeleteMutex(cg->write_lock);
	free(cg->config_entries);
}

void resize_config_group(config_group *cg, uint16_t count) {
	uint16_t i = 0;

	if (count > cg->max_entries) {
		config_entry *ptr = SV_REALLOC(cg->config_entries, sizeof(config_entry) * count);
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

bool config_read_lighthouse(config_group *lh_config, BaseStationData *bsd, uint8_t idx) {
	config_group *cg = lh_config + idx;
	uint8_t found = 0;
	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
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
		return false;
	}

	FLT defaults[7] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	bsd->BaseStationID = config_read_uint32(cg, "id", 0);
	bsd->mode = config_read_uint32(cg, "mode", 0);

	config_read_float_array(cg, "pose", &bsd->Pose.Pos[0], defaults, 7);
	config_read_float_array(cg, "variance", &bsd->variance.Pos[0], defaults, 6);

	FLT accel[3] = {0};
	config_read_float_array(cg, "accel", accel, defaults, 3);
	for (int i = 0; i < 3; i++)
		bsd->accel[i] = accel[i];

	FLT cal[sizeof(bsd->fcal)] = { 0 };
	config_read_float_array(cg, "fcalphase", cal, defaults, 2);
	config_read_float_array(cg, "fcaltilt", cal + 2, defaults, 2);
	config_read_float_array(cg, "fcalcurve", cal + 4, defaults, 2);
	config_read_float_array(cg, "fcalgibpha", cal + 6, defaults, 2);
	config_read_float_array(cg, "fcalgibmag", cal + 8, defaults, 2);
	config_read_float_array(cg, "fcalogeephase", cal + 10, defaults, 2);
	config_read_float_array(cg, "fcalogeemag", cal + 12, defaults, 2);

	for (size_t i = 0; i < 2; i++) {
		bsd->fcal[i].phase = cal[0 + i];
		bsd->fcal[i].tilt = cal[2 + i];
		bsd->fcal[i].curve = cal[4 + i];
		bsd->fcal[i].gibpha = cal[6 + i];
		bsd->fcal[i].gibmag = cal[8 + i];
		bsd->fcal[i].ogeephase = cal[10 + i];
		bsd->fcal[i].ogeemag = cal[12 + i];
	}

	bsd->OOTXSet = config_read_uint32(cg, "OOTXSet", 0);
	bsd->PositionSet = config_read_uint32(cg, "PositionSet", 0);
	return true;
}

void config_set_lighthouse(config_group *lh_config, BaseStationData *bsd, uint8_t idx) {
	config_group *cg = lh_config + idx;
	config_set_uint32(cg, "index", idx);
	
	config_set_uint32(cg, "id", bsd->BaseStationID);
	config_set_uint32(cg, "mode", bsd->mode);
	config_set_float_a(cg, "pose", &bsd->Pose.Pos[0], 7);
	config_set_float_a(cg, "variance", &bsd->variance.Pos[0], 6);
	config_set_uint32(cg, "unlock_count", bsd->sys_unlock_count);
	FLT accel[] = {bsd->accel[0], bsd->accel[1], bsd->accel[2]};
	config_set_float_a(cg, "accel", accel, 3);
	if (!quatiszero(bsd->Pose.Rot))
		quatnormalize(bsd->Pose.Rot, bsd->Pose.Rot);

	FLT cal[sizeof(bsd->fcal)] = { 0 };

	for (size_t i = 0; i < 2; i++) {
		cal[0 + i] = bsd->fcal[i].phase;
		cal[2 + i] = bsd->fcal[i].tilt;
		cal[4 + i] = bsd->fcal[i].curve;
		cal[6 + i] = bsd->fcal[i].gibpha;
		cal[8 + i] = bsd->fcal[i].gibmag;
		cal[10 + i] = bsd->fcal[i].ogeephase;
		cal[12 + i] = bsd->fcal[i].ogeemag;
	}

	config_set_float_a(cg, "fcalphase", cal, 2);
	config_set_float_a(cg, "fcaltilt", cal + 2, 2);
	config_set_float_a(cg, "fcalcurve", cal + 4, 2);
	config_set_float_a(cg, "fcalgibpha", cal + 6, 2);
	config_set_float_a(cg, "fcalgibmag", cal + 8, 2);
	config_set_float_a(cg, "fcalogeephase", cal + 10, 2);
	config_set_float_a(cg, "fcalogeemag", cal + 12, 2);

	config_set_uint32(cg, "OOTXSet", bsd->OOTXSet);
	config_set_uint32(cg, "PositionSet", bsd->PositionSet);
}

void sstrcpy(char **dest, const char *src) {
	uint32_t len = (uint32_t)strlen(src) + 1;
	assert(dest != NULL);

	char *ptr = (char *)SV_REALLOC(*dest, len); // acts like SV_MALLOC if dest==NULL
	assert(ptr != NULL);
	*dest = ptr;

	strcpy(*dest, src);
	// printf("%s -> %s\r\n", *dest, src);
}

static void config_group_lock(config_group *cg) {
	if (cg == 0)
		return;

	OGLockMutex(cg->write_lock);
}

static void config_group_unlock(config_group *cg) {
	if (cg == 0)
		return;

	OGUnlockMutex(cg->write_lock);
}

config_entry *find_config_entry(config_group *cg, const char *tag) {
	if (cg == NULL || tag == NULL) {
		return NULL;
	}

	OGLockMutex(cg->write_lock);
	uint16_t i = 0;
	for (i = 0; i < cg->used_entries; ++i) {
		if (strcmp(cg->config_entries[i].tag, tag) == 0) {
			config_group_unlock(cg);
			return cg->config_entries + i;
		}
	}
	config_group_unlock(cg);
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
			values[i] = ((FLT *)cv->data)[i];
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
	if (cg == NULL)
		return NULL;

	if (cg->used_entries >= cg->max_entries)
		resize_config_group(cg, cg->max_entries + 10);

	cv = cg->config_entries + cg->used_entries;

	cg->used_entries++;

	return cv;
}

const char *config_set_str(config_group *cg, const char *tag, const char *value) {
	if (cg == 0) {
		return 0;
	}
	OGLockMutex(cg->write_lock);

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

	update_list_t * t = cv->update_list;
	while( t ) { *((const char **)t->value) = value; t = t->next; }
	config_group_unlock(cg);

	return value;
}

uint32_t config_set_uint32(config_group *cg, const char *tag, const uint32_t value) {
	config_group_lock(cg);
	config_entry *cv = find_config_entry(cg, tag);
	if (cv == NULL)
		cv = next_unused_entry(cg,tag);

	if (cv == NULL) {
		config_group_unlock(cg);
		return value;
	}

	sstrcpy(&(cv->tag), tag);
	cv->numeric.i = value;
	cv->type = CONFIG_UINT32;

	update_list_t * t = cv->update_list;
	while( t ) { *((uint32_t*)t->value) = value; t = t->next; }
	config_group_unlock(cg);

	return value;
}

FLT config_set_float(config_group *cg, const char *tag, const FLT value) {
	if (cg == NULL)
		return value;

	config_group_lock(cg);
	config_entry *cv = find_config_entry(cg, tag);
	if (cv == NULL)
		cv = next_unused_entry(cg,tag);

	sstrcpy(&(cv->tag), tag);
	cv->numeric.f = value;
	cv->type = CONFIG_FLOAT;

	
	update_list_t * t = cv->update_list;
	while( t ) { *((FLT*)t->value) = value; t = t->next; }
	config_group_unlock(cg);

	return value;
}

const FLT *config_set_float_a(config_group *cg, const char *tag, const FLT *values, uint8_t count) {
	config_group_lock(cg);
	config_entry *cv = find_config_entry(cg, tag);
	if (cv == NULL)
		cv = next_unused_entry(cg,tag);

	sstrcpy(&(cv->tag), tag);

	char *ptr = (char *)SV_REALLOC(cv->data, sizeof(FLT) * count);
	assert(ptr != NULL);
	cv->data = ptr;

	memcpy(cv->data, values, sizeof(FLT) * count);
	cv->type = CONFIG_FLOAT_ARRAY;
	cv->elements = count;

	config_group_unlock(cg);
	return values;
}

void _json_write_float_array(FILE *f, const char *tag, FLT *v, uint8_t count) {
#ifdef CN_USE_DOUBLE
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

const char *survive_config_file_path(struct SurviveContext *ctx, char *path) {
	const char *configpath = survive_config_file_name(ctx);

	if (isalpha(configpath[0])) {
		size_t idx = 0;

#ifdef SURVIVE_CONFIG_PATH
		idx = snprintf(path, FILENAME_MAX, SURVIVE_CONFIG_PATH);
#else
		const char *home = 0;

		if (home = getenv("XDG_CONFIG_HOME")) {
			idx = snprintf(path, FILENAME_MAX, "%s/libsurvive", home);
		} else if (home = getenv("HOME")) {
			idx = snprintf(path, FILENAME_MAX, "%s/.config/libsurvive", home);
		} else if (home = getenv("LOCALAPPDATA")) {
			idx = snprintf(path, FILENAME_MAX, "%s/libsurvive", home);
		}
#endif

#ifdef _WIN32
		_mkdir(path);
#elif __linux__
		mkdir(path, 0755);
#endif
		snprintf(path + idx, FILENAME_MAX - idx, "/%s", configpath);
	} else {
		char pathname[512] = {0};
		strncpy(pathname, configpath, sizeof(pathname) - 1);
		int i;
		for (i = strlen(pathname); i > 0 && pathname[i] != '/'; i--)
			;
		if (i > 0) {
			pathname[i] = 0;
#ifdef _WIN32
			_mkdir(path);
#elif __linux__
			mkdir(pathname, 0755);
#endif
		}
		strncpy(path, configpath, FILENAME_MAX);
	}
	return path;
}

// struct SurviveContext
SurviveContext *survive_context;
void config_save(SurviveContext *ctx) {
	char path[FILENAME_MAX] = "";
	survive_config_file_path(ctx, path);

	FILE *f = fopen(path, "w");

	if (f == 0) {
		static bool warnedOnce = false;
		if (!warnedOnce && strcmp(path, "/dev/null") != 0) {
			SV_WARN("Could not open '%.512s' for writing; settings and calibration will not persist. This typically "
					"happens if the path doesn't exist or root owns the file.",
					path);
			warnedOnce = true;
		}
		return;
	}

	write_config_group(f, ctx->global_config_values, NULL);

	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		if (ctx->bsd[i].OOTXSet) {
			char name[128] = { 0 };
			snprintf(name, 128, "lighthouse%d", i);
			write_config_group(f, ctx->lh_config + i, name);
		}
	}

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

void handle_config_group(struct json_callbacks *cbs, struct json_stack_entry_s *obj) {
	cg_stack_head++;
	int lh_idx;

	int lhMatch = sscanf(json_stack_tag(obj), "lighthouse%d", &lh_idx);
	if (lhMatch == 1) {
		cg_stack[cg_stack_head] = survive_context->lh_config + lh_idx;
	} else {
		cg_stack[cg_stack_head] = survive_context->global_config_values;
	}
}

void pop_config_group(struct json_callbacks *cbs, struct json_stack_entry_s *obj) { cg_stack_head--; }

int parse_floats(const char *tag, const char **values, uint8_t count) {
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

int parse_uint32(const char *tag, const char **values, uint16_t count) {
	uint16_t i = 0;
	uint32_t *l = alloca(sizeof(uint32_t) * count);
	char *end = NULL;
	config_group *cg = cg_stack[cg_stack_head];

	for (i = 0; i < count; ++i) {
		l[i] = strtoul(values[i], &end, 10);
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

static size_t array_size = 0;
static const char **array_data = 0;
void handle_array_start(struct json_callbacks *cb, struct json_stack_entry_s *array) { array_size = 1; }
void handle_array_end(struct json_callbacks *cb, struct json_stack_entry_s *array) {
	const char *tag = json_stack_tag(array);
	if (NULL != array_data && NULL != *array_data) {
		if (parse_uint32(tag, array_data, array_size - 1) == 0) {
			// parse integers first, stricter rules
			parse_floats(tag, array_data, array_size - 1);
		}
	}

	array_size = 0;
}

void handle_tag_value(struct json_callbacks *cbs, struct json_stack_entry_s *array) {
	const char *tag = json_stack_tag(array);
	const char *value = json_stack_value(array);

	if (array_size > 0) {
		array_data = realloc(array_data, sizeof(char *) * array_size);
		array_data[array_size++ - 1] = value;
		return;
	}
	char **values = 0;
	uint8_t count = 0;
	// Uncomment for more debugging of input configuration.
	// print_json_value(tag,values,count);

	if (parse_uint32(tag, &value, 1) > 0)
		return; // parse integers first, stricter rules

	if (parse_floats(tag, &value, 1) > 0)
		return;

	config_group *cg = cg_stack[cg_stack_head];

	// should probably also handle string arrays
	config_set_str(cg, tag, value);
	//	else if (count>1) config_set_str
}

void config_read(SurviveContext *sctx, const char *init_path) {
	survive_context = sctx;

	char path[FILENAME_MAX] = "";
	if (init_path) {
		strncpy(path, init_path, FILENAME_MAX - 1);
	} else {
		survive_config_file_path(sctx, path);
	}
	cg_stack[0] = sctx->global_config_values;
	struct json_callbacks cbs = {.json_begin_object = handle_config_group,
								 .json_end_object = pop_config_group,
								 .json_tag_value = handle_tag_value,
								 .json_begin_array = handle_array_start,
								 .json_end_array = handle_array_end};
	json_load_file(&cbs, path);
}

static config_entry *sc_search(SurviveContext *ctx, const char *tag) {
	if (ctx == 0) {
		return 0;
	}

	config_entry *cv = find_config_entry(ctx->temporary_config_values, tag);
	if (!cv) {
		cv = find_config_entry(ctx->global_config_values, tag);
	}
	return cv;
}

static FLT config_entry_as_FLT(config_entry *entry) {
	switch (entry->type) {
	case CONFIG_FLOAT:
		return entry->numeric.f;
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
		return (uint32_t)strtol(entry->data, 0, 0);
	case CONFIG_FLOAT_ARRAY:
	case CONFIG_UNKNOWN:
		break;
	}
	return 0;
}

SURVIVE_EXPORT char survive_config_type(SurviveContext *ctx, const char *tag) {
	config_entry *entry = sc_search(ctx, tag);
	if (entry == 0) {
		return entry->type;
	}

	return 0;
}

SURVIVE_EXPORT void survive_config_as_str(SurviveContext *ctx, char *output, size_t n, const char *tag,
										  const char *def) {
	if (n == 0 || output == 0)
		return;

	struct static_conf_t *sc = find_static_conf_t(tag);
	if (sc) {
		switch (sc->type) {
		case 'b':
            snprintf(output, n, "%d", survive_configb(ctx, tag, SC_GET, sc->data_default.b));
            break;
		case 'f': {
			FLT df = survive_configf(ctx, tag, SC_GET, (float)sc->data_default.f);
			if (fabs(df) > .001 || df == 0) {
				snprintf(output, n, "%f", df);
			} else {
				snprintf(output, n, "%e", df);
			}
			break;
		}
		case 'i':
			snprintf(output, n, "%i", survive_configi(ctx, tag, SC_GET, sc->data_default.i));
			break;
		case 's':
			snprintf(output, n, "%s", survive_configs(ctx, tag, SC_GET, sc->data_default.s));
			break;
		}
		return;
	}

	config_entry *entry = sc_search(ctx, tag);
	if (entry == 0) {
		if (def == 0) {
			output[0] = 0;
		} else {
			strncpy(output, def, n);
		}
		return;
	}

	switch (entry->type) {
	case CONFIG_FLOAT:
		snprintf(output, n, "%f", (float)entry->numeric.f);
		break;
	case CONFIG_UINT32:
		snprintf(output, n, "%i", entry->numeric.i);
		break;
	case CONFIG_STRING:
		snprintf(output, n, "%s", entry->data);
		break;
	case CONFIG_FLOAT_ARRAY:

		snprintf(output, n, "%s", entry->data);
	case CONFIG_UNKNOWN:
		break;
	}
}

bool survive_config_is_set(SurviveContext *ctx, const char *tag) {
	config_entry *cv = sc_search(ctx, tag);
	return cv != 0;
}

FLT survive_configf(SurviveContext *ctx, const char *tag, char flags, FLT def) {
	if (!(flags & SC_OVERRIDE)) {
		config_entry *cv = sc_search(ctx, tag);
		if (cv) {
			return config_entry_as_FLT(cv);
		}
	}


	int i;
	if( !(flags & SC_OVERRIDE) )
	{
		for (struct static_conf_t *config = head; config; config = config->next) {
			if (strcmp(tag, config->name) == 0) {
				def = config->data_default.f;
			}
		}
	}

	if (ctx) {
		// If override is flagged, or, we can't find the variable, ,continue on.
		if (flags & SC_SETCONFIG) {
			config_set_float(ctx->temporary_config_values, tag, def);
			config_set_float(ctx->global_config_values, tag, def);
		} else if (flags & SC_SET) {
			config_set_float(ctx->temporary_config_values, tag, def);
		}
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

	uint32_t statictimedef = def;
	int i;
	if( !(flags & SC_OVERRIDE) )
	{
		for (struct static_conf_t *config = head; config; config = config->next) {
			if (strcmp(tag, config->name) == 0) {
				def = config->data_default.i;
			}
		}
	}

	if (ctx) {
		// If override is flagged, or, we can't find the variable, ,continue on.
		if (flags & SC_SETCONFIG) {
			config_set_uint32(ctx->temporary_config_values, tag, def);
			config_set_uint32(ctx->global_config_values, tag, def);
		} else if (flags & SC_SET) {
			config_set_uint32(ctx->temporary_config_values, tag, def);
		}
	}

	return def;
}

SURVIVE_EXPORT bool survive_configb(SurviveContext *ctx, const char *tag, char flags, bool def) {
    if (!(flags & SC_OVERRIDE)) {
        config_entry *cv = sc_search(ctx, tag);
        if (cv) {
            return config_entry_as_uint32_t(cv);
        }
    }

    uint32_t statictimedef = def;
    int i;
    if( !(flags & SC_OVERRIDE) )
    {
        for (struct static_conf_t *config = head; config; config = config->next) {
            if (strcmp(tag, config->name) == 0) {
                def = config->data_default.i;
            }
        }
    }

    if (ctx) {
        // If override is flagged, or, we can't find the variable, ,continue on.
        if (flags & SC_SETCONFIG) {
            config_set_uint32(ctx->temporary_config_values, tag, def);
            config_set_uint32(ctx->global_config_values, tag, def);
        } else if (flags & SC_SET) {
            config_set_uint32(ctx->temporary_config_values, tag, def);
        }
    }

    return def;
}

const char *survive_configs(SurviveContext *ctx, const char *tag, char flags, const char *def) {
	if(ctx == 0)
		return def;

	if (!(flags & SC_OVERRIDE)) {
		config_entry *cv = sc_search(ctx, tag);
		if (cv)
			return cv->data;
	}

	int i;
	char foundtype = 0;
	const char * founddata = def;
	for (struct static_conf_t *config = head; config; config = config->next) {
		if (!config)
			break;
		if (strcmp(tag, config->name) == 0) {
			founddata = config->data_default.s;
			foundtype = config->type;
			if( !(flags & SC_OVERRIDE) )
			{
				def = founddata;
			}
		}
	}

	if( !foundtype || foundtype == 's' )
	{
		// If override is flagged, or, we can't find the variable, ,continue on.
		if (flags & SC_SETCONFIG) {
			config_set_str(ctx->temporary_config_values, tag, def);
			config_set_str(ctx->global_config_values, tag, def);
		} else if (flags & SC_SET) {
			config_set_str(ctx->temporary_config_values, tag, def);
		} else {
			return founddata;
		}
	}
	else if( foundtype == 'i' )
	{
		survive_configi(ctx, tag, flags, atoi(def ? def : "0"));
	} else if (foundtype == 'b') {
		survive_configb(ctx, tag, flags, atoi(def ? def : "0"));
	} else if (foundtype == 'f') {
		survive_configf( ctx, tag, flags, atof( def ) );
	}

	return def;
}

SURVIVE_EXPORT void survive_attach_config(SurviveContext *ctx, const char *tag, void * var, char type )
{
	if (ctx == 0)
		return;

	config_entry *cv = sc_search(ctx, tag);
	if( !cv )
	{
		//Check to see if there is a static config with this value.
		if( type == 'i' )
		{
			*((int*)var) = survive_configi( ctx, tag, SC_SET, 0);
		}
        if( type == 'b' )
        {
            *((bool*)var) = survive_configb( ctx, tag, SC_SET, 0);
        }
		if( type == 'f' )
		{
			*((FLT*)var) = survive_configf( ctx, tag, SC_SET, 0);
		}
		if( type == 's' )
		{
			const char * cv = survive_configs( ctx, tag, SC_SET, 0);
			memcpy( var, cv, strlen(cv) );
		}
		cv = sc_search(ctx, tag);
		if (!cv && ctx) {
			SV_GENERAL_ERROR("Configuration item %s not initialized.\n", tag);
			return;
		}
	}

	{
		update_list_t **ul = &cv->update_list;
		while (*ul) {
			if ((*ul)->value == var)
				return;
			ul = &((*ul)->next);
		}

		update_list_t *t = *ul = SV_CALLOC(sizeof(update_list_t));
		t->next = 0;
		t->value = var;
	}

	switch (type) {
	    case 'b':
            *((bool *)var) = survive_configb(ctx, tag, SC_GET, 0);
			SV_VERBOSE(110, "\t%s: %i", tag, *((bool *)var));
			break;
	case 'i':
		*((int *)var) = survive_configi(ctx, tag, SC_GET, 0);
		SV_VERBOSE(110, "\t%s: %i", tag, *((int *)var));
		break;
	case 'f':
		*((FLT *)var) = survive_configf(ctx, tag, SC_GET, 0);
		SV_VERBOSE(110, "\t%s: %+f", tag, *((FLT *)var));
		break;
	case 's': {
		const char *cv = survive_configs(ctx, tag, SC_SET, 0);
		strcpy(var, cv);
		SV_VERBOSE(110, "\t%s: '%s'", tag, cv);
		break;
	}
	default:
		SV_GENERAL_ERROR("Unhandled config type '%c'.\n", type);
	}
}

SURVIVE_EXPORT void survive_attach_configi(SurviveContext *ctx, const char *tag, int32_t *var) {
	survive_attach_config( ctx, tag, var, 'i' );
}
SURVIVE_EXPORT void survive_attach_configb(SurviveContext *ctx, const char *tag, bool *var) {
    survive_attach_config( ctx, tag, var, 'b' );
}

SURVIVE_EXPORT void survive_attach_configf(SurviveContext *ctx, const char *tag, FLT * var )
{
	survive_attach_config( ctx, tag, var, 'f' );
}

SURVIVE_EXPORT void survive_attach_configs(SurviveContext *ctx, const char *tag, char * var )
{
	survive_attach_config( ctx, tag, var, 's' );
}

SURVIVE_EXPORT void survive_detach_config(SurviveContext *ctx, const char *tag, void * var )
{
	if (ctx == 0) {
		return;
	}

	config_entry *cv = sc_search(ctx, tag);
	if( !cv )
	{
		SV_GENERAL_ERROR("Configuration item %s not initialized.\n", tag);
		return;
	}

	// printf("Tag: %s %p\n", tag, var);
	// print_ul(&cv->update_list);
	bool somethingFreed = false;
	update_list_t ** ul = &cv->update_list;
	update_list_t **lul = ul;
	while( *ul )
	{
		if( (*ul)->value == var )
		{
			update_list_t * v = *ul;
			v->value = 0;
			(*lul)->next = v->next;
			*ul = v->next;
			free(v);
			somethingFreed = true;
		} else {
			ul = &((*ul)->next);
		}
		lul = ul;
	}

	// print_ul(&cv->update_list);
	if (somethingFreed == false) {
		SV_WARN("Found no config item to detach %s", tag);
	}
}
