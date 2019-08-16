// Copyright 2016 <>< C. N. Lohr, FULLY Under MIT/x11 License.
// All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include "survive_internal.h"
#include <assert.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <survive.h>

#include "os_generic.h"
#include "survive_config.h"
#include "survive_default_devices.h"
#include "survive_playback.h"

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef __APPLE__
#define z_const const
#endif

STATIC_CONFIG_ITEM(SURVIVE_VERBOSE, "v", 'i', "Verbosity level", 0);
STATIC_CONFIG_ITEM( BLACKLIST_DEVS, "blacklist-devs", 's', "List any devs (or substrings of devs) to blacklist.", "-" );
STATIC_CONFIG_ITEM( CONFIG_FILE, "configfile", 's', "Default configuration file", "config.json" );
STATIC_CONFIG_ITEM(INIT_CONFIG_FILE, "init-configfile", 's', "Initial configuration file", 0);
STATIC_CONFIG_ITEM( CONFIG_D_CALI, "disable-calibrate", 'i', "Enables or disables calibration", 0 );
STATIC_CONFIG_ITEM( CONFIG_F_CALI, "force-calibrate", 'i', "Forces calibration even if one exists.", 0 );
STATIC_CONFIG_ITEM(CONFIG_LIGHTHOUSE_COUNT, "lighthousecount", 'i', "How many lighthouses to look for.", 2);
STATIC_CONFIG_ITEM(LIGHTHOUSE_GEN, "lighthouse-gen", 'i',
				   "Which lighthouse gen to use -- 1 for LH1, 2 for LH2, 0 (default) for auto-detect", 0);

#ifdef WIN32
#define RUNTIME_SYMNUM
#endif

#ifdef RUNTIME_SYMNUM
#include <symbol_enumerator.h>
static int did_runtime_symnum;
int SymnumCheck(const char *path, const char *name, void *location, long size) {
	if (strncmp(name, "REGISTER", 8) == 0) {
		typedef void (*sf)();
		sf fn = (sf)location;
		fn();
	}
	return 0;
}

#endif

static void reset_stderr(FILE *f) {
#ifndef _WIN32
	fprintf(f, "\033[0m");
#else
	HANDLE   hConsole = GetStdHandle(STD_ERROR_HANDLE);
	SetConsoleTextAttribute(hConsole, 7);
#endif
}

static void set_stderr_color(FILE *f, int c) {
#ifndef _WIN32
	fprintf(f, "\033[0;31m");
#else
	HANDLE   hConsole = GetStdHandle(STD_ERROR_HANDLE);
	SetConsoleTextAttribute(hConsole, c == 1 ? FOREGROUND_RED : (FOREGROUND_INTENSITY | FOREGROUND_RED));
#endif
}

static void survive_default_report_error_process(struct SurviveContext *ctx, SurviveError errorCode) {
	ctx->currentError = errorCode;
}

static void survive_default_error(struct SurviveContext *ctx, SurviveError errorCode, const char *fault) {
	set_stderr_color(ctx->log_target, 2);
	fprintf(ctx->log_target, "Error %d: %s\n", errorCode, fault);
	reset_stderr(ctx->log_target);
	fflush(ctx->log_target);
}

static void survive_default_info(struct SurviveContext *ctx, const char *fault) {
	survive_recording_info_process(ctx, fault);
	fprintf(ctx->log_target, "Info: %s\n", fault);
	fflush(ctx->log_target);
}

static void survive_default_warn(struct SurviveContext *ctx, const char *fault) {
	survive_recording_info_process(ctx, fault);
	set_stderr_color(ctx->log_target, 1);
	fprintf(ctx->log_target, "Warning: %s\n", fault);
	reset_stderr(ctx->log_target);
	fflush(ctx->log_target);
}

void survive_default_log_process(struct SurviveContext *ctx, SurviveLogLevel ll, const char *fault) {
	switch (ll) {
	case SURVIVE_LOG_LEVEL_ERROR:
		survive_default_error(ctx, ctx->currentError, fault);
		return;
	case SURVIVE_LOG_LEVEL_WARNING:
		survive_default_warn(ctx, fault);
		return;
	case SURVIVE_LOG_LEVEL_INFO:
		survive_default_info(ctx, fault);
		return;
	}
}

static void *button_servicer(void *context) {
	SurviveContext *ctx = (SurviveContext *)context;

	while (1) {
		OGLockSema(ctx->buttonQueue.buttonservicesem);

		if (ctx->state != SURVIVE_RUNNING) {
			// we're shutting down.  Close.
			return NULL;
		}

		ButtonQueueEntry *entry = &(ctx->buttonQueue.entry[ctx->buttonQueue.nextReadIndex]);
		if (entry->isPopulated == 0) {
			// should never happen.  indicates failure of code pushing stuff onto
			// the buttonQueue
			// if it does happen, it will kill all future button input
			printf("ERROR: Unpopulated ButtonQueueEntry! NextReadIndex=%d\n", ctx->buttonQueue.nextReadIndex);
			return NULL;
		}

		// printf("ButtonEntry: eventType:%x, buttonId:%d, axis1:%d, axis1Val:%8.8x, axis2:%d, axis2Val:%8.8x\n",
		//	entry->eventType,
		//	entry->buttonId,
		//	entry->axis1Id,
		//	entry->axis1Val,
		//	entry->axis2Id,
		//	entry->axis2Val);

		button_process_func butt_func = ctx->buttonproc;
		if (butt_func) {
			butt_func(entry->so, entry->eventType, entry->buttonId, entry->axis1Id, entry->axis1Val, entry->axis2Id,
					  entry->axis2Val);
		}

		ctx->buttonQueue.nextReadIndex++;
		if (ctx->buttonQueue.nextReadIndex >= BUTTON_QUEUE_MAX_LEN) {
			ctx->buttonQueue.nextReadIndex = 0;
		}
	};
	return NULL;
}

void survive_verify_FLT_size(uint32_t user_size) {
	if (sizeof(FLT) != user_size) {
		fprintf(stderr, "FLT type incompatible; the shared library libsurvive has FLT size %lu vs user program %u\n",
				(unsigned long)sizeof(FLT), user_size);
		fprintf(stderr, "Add '#define FLT %s' before including survive.h or recompile the shared library with the "
						"appropriate flag. \n",
				sizeof(FLT) == sizeof(double) ? "double" : "float");
		exit(-1);
	}
}

static void PrintMatchingDrivers( const char * prefix, const char * matchingparam )
{
	int i = 0;
	char stringmatch[128];
	snprintf( stringmatch, 127, "%s%s", prefix, matchingparam?matchingparam:"" );
	const char * DriverName;
	while ((DriverName = GetDriverNameMatching(stringmatch, i++))) 
	{
		printf( "%s ", DriverName+strlen(prefix) );
	}
}

SURVIVE_EXPORT int8_t survive_get_bsd_idx(SurviveContext *ctx, survive_channel channel) {
	assert(channel >= 0 && channel < 16);

	int8_t i = ctx->bsd_map[channel];
	if (i != -1)
		return i;

	for (i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		if (ctx->bsd[i].mode == 0xFF) {
			ctx->bsd[i] = (BaseStationData){0};
			ctx->bsd[i].mode = channel;
			SV_INFO("Adding lighthouse ch %d", channel);
			if (ctx->activeLighthouses < i + 1) {
				ctx->activeLighthouses = i + 1;
			}
			return ctx->bsd_map[channel] = i;
		}
	}

	assert(false);
	return -1;
}

SurviveContext *survive_init_internal(int argc, char *const *argv, void *userData, log_process_func log_func) {
	int i;

	survive_load_plugins(0);

#ifdef RUNTIME_SYMNUM
	if (!did_runtime_symnum) {
		EnumerateSymbols(SymnumCheck);
		did_runtime_symnum = 1;
	}
#endif

	SurviveContext *ctx = calloc(1, sizeof(SurviveContext));
	ctx->user_ptr = userData;

	for (int i = 0; i < NUM_GEN2_LIGHTHOUSES; i++) {
		ctx->bsd[i].mode = -1;
		ctx->bsd_map[i] = -1;
	}
	ctx->state = SURVIVE_STOPPED;

#define SURVIVE_HOOK_PROCESS_DEF(hook) survive_install_##hook##_fn(ctx, 0);
#define SURVIVE_HOOK_FEEDBACK_DEF(hook) survive_install_##hook##_fn(ctx, 0);
#include "survive_hooks.h"

	survive_install_log_fn(ctx, log_func);

	ctx->global_config_values = malloc(sizeof(config_group));
	ctx->temporary_config_values = malloc(sizeof(config_group));
	ctx->lh_config = malloc(sizeof(config_group) * NUM_GEN2_LIGHTHOUSES);

	// initdata
	init_config_group(ctx->global_config_values, 30, ctx);
	init_config_group(ctx->temporary_config_values, 30, ctx);
	for (i = 0; i < NUM_GEN2_LIGHTHOUSES; i++)
		init_config_group(&ctx->lh_config[i], 10, ctx);

	// Process command-line parameters.
	char *const *av = argv + 1;
	char *const *argvend = argv + argc;
	int list_for_autocomplete = 0;
	const char * autocomplete_match[3] = { 0, 0, 0};
	int showhelp = 0;
	for (; av < argvend; av++) {
		if ((*av)[0] != '-')
			showhelp = 1;
		else {
			const char *vartoupdate = 0;

			switch ((*av)[1]) {
			case '-':
				vartoupdate = &(*av)[2];
				break;
			case 'h':
				showhelp = 1;
				break;
			case 'p':
				vartoupdate = "defaultposer";
				break;
			case 'l':
				vartoupdate = "lighthousecount";
				break;
			case 'm':
				if( av + 1 < argvend ) autocomplete_match[0] = *(av+1);
				if( av + 2 < argvend ) autocomplete_match[1] = *(av+2);
				if( av + 3 < argvend ) autocomplete_match[2] = *(av+3);
				list_for_autocomplete = 1;
				av = argvend; //Eject immediately after processing -m
				break;
			case 'c':
				vartoupdate = "configfile";
				break;
			default:
				fprintf(ctx->log_target, "Error: unknown parameter %s\n", *av);
				showhelp = 1;
			}

			if (vartoupdate) {
				const char *name = *av + 2; // Skip the '--';
				bool isLast = av + 1 == argvend;
				bool nextIsOption = !isLast && av[1][0] == '-';
				if (nextIsOption) {
					int n = atoi(*(av + 1));
					nextIsOption = n == 0;
				}
				bool flagArgument = isLast || nextIsOption;

				if ( strcmp( name, "help" ) == 0 )
					showhelp = 1;
				else if (flagArgument) {
					bool value = strncmp("no-", name, 3) != 0;
					if (value == 0) {
						name += 3; // Skip "no-"
					}
					survive_configi(ctx, name, SC_OVERRIDE | SC_SET, value);
				} else {
					const char *value = *(av + 1);
					survive_configs(ctx, name, SC_OVERRIDE | SC_SET, value);
					av++;
				}
			}
		}
	}

	const char *log_file = survive_configs(ctx, "log", SC_GET, 0);
	ctx->log_target = log_file ? fopen(log_file, "w") : stderr;

	const char *config_prefix_fields[] = {"playback", 0};
	for (const char **name = config_prefix_fields; *name; name++) {
		if (!survive_config_is_set(ctx, "configfile") && survive_config_is_set(ctx, *name)) {
			char configfile[256] = { 0 };
			const char *recordname = survive_configs(ctx, *name, SC_GET, "");

			const char *end = recordname + strlen(recordname);
			while (end != recordname && !(*end == '/' || *end == '\\'))
				end--;
			if (*end == '/' || *end == '\\')
				end++;

			if (end[0] != 0) {
				recordname = end;
				snprintf(configfile, 256, "%s.json", recordname);
				survive_configs(ctx, "configfile", SC_SET | SC_OVERRIDE, configfile);
				SV_INFO("Config file is %s", configfile);
			}
		}
	}

	ctx->log_level = survive_configi(ctx, "v", SC_SETCONFIG, 0);
	ctx->lh_version = -1;
	ctx->lh_version_forced = survive_configi(ctx, "lighthouse-gen", SC_SETCONFIG, 0) - 1;

	const char *init_config = survive_configs(ctx, "init-configfile", SC_GET, 0);
	;
	if (init_config == 0) {
		init_config = survive_configs(ctx, "configfile", SC_GET, "config.json");
	} else {
		SV_INFO("Initial config file is %s", init_config);
	}
	config_read(ctx, init_config);
	ctx->activeLighthouses = survive_configi(ctx, "lighthousecount", SC_SETCONFIG, 2);

	for (int i = 0; i < ctx->activeLighthouses; i++) {
		config_read_lighthouse(ctx->lh_config, &(ctx->bsd[i]), i);
		if (ctx->bsd[i].mode >= 0 && ctx->bsd[i].mode < 16)
			ctx->bsd_map[ctx->bsd[i].mode] = i;
		SV_VERBOSE(50, "Adding LH %d mode: %d id: %08x", i, ctx->bsd[i].mode, ctx->bsd[i].BaseStationID);
	}

	if( list_for_autocomplete )
	{
		const char * lastparam = (autocomplete_match[2]==0)?autocomplete_match[1]:autocomplete_match[2];
		const char * matchingparam = (autocomplete_match[2]==0)?0:autocomplete_match[1];
		//First see if any of the parameters perfectly match a config item, if so print some help.
		//fprintf( stderr, "!!! %s !!! %s !!!\n", lastparam, matchingparam );

		const char * checkconfig = matchingparam;
		if( matchingparam == 0 ) checkconfig = lastparam;

		if( checkconfig && strlen( checkconfig ) > 2 && survive_print_help_for_parameter( checkconfig+2 ) )
		{
			exit(0);
		}

		if (lastparam && strstr(lastparam, "poser"))
			PrintMatchingDrivers("Poser", matchingparam);
		else if (lastparam && strstr(lastparam, "disambiguator"))
			PrintMatchingDrivers("Disambiguator", matchingparam);
		else
		{
			printf( "-h -m -p -l -c " );
			survive_print_known_configs( ctx, 0 );
		}
		printf( "\n" );
		exit( 0 );
	}

	if( showhelp )
	{
		// Can't use SV_GENERAL_ERROR here since we don't have a context to send to yet.
		fprintf(stderr, "Available flags:\n");
		fprintf(stderr, " -h                      - shows help.\n");
		fprintf(stderr, " -m                      - list parameters, for autocomplete.\n");
		fprintf(stderr, " -p [poser]              - use a specific defaultposer.\n");
		fprintf(stderr, " -l [lighthouse count]   - use a specific number of lighthoses.\n");
		fprintf(stderr, " -c [config file]        - set config file\n\n");

		survive_print_known_configs( ctx, 1 );
		return 0;
	}

	return ctx;
}

survive_timecode survive_timecode_difference(survive_timecode most_recent, survive_timecode least_recent) {
	survive_timecode diff = 0;
	if (most_recent > least_recent) {
		diff = most_recent - least_recent;
	} else {
		diff = least_recent - most_recent;
	}

	if (diff > 0xFFFFFFFF / 2)
		return 0xFFFFFFFF - diff;
	return diff;
}

void *GetDriverByConfig(SurviveContext *ctx, const char *name, const char *configname, const char *configdef) {
	const char *Preferred = survive_configs(ctx, configname, SC_SETCONFIG, configdef);
	const char *DriverName = 0;
	const char *picked = 0;
	int i = 0;
	void *func = 0;
	int prefixLen = strlen(name);

	SV_VERBOSE(1, "Available %ss:", name);
	while ((DriverName = GetDriverNameMatching(name, i++))) {
		void *p = GetDriver(DriverName);

		bool match = strcmp(DriverName, Preferred) == 0 || strcmp(DriverName + prefixLen, Preferred) == 0;
		SV_VERBOSE(1, "\t%c%s", match ? '*' : ' ', DriverName + prefixLen);
		if (!func || match) {
			func = p;
			picked = (DriverName + prefixLen);
		}
	}
	if (!func) {
		SV_ERROR(SURVIVE_ERROR_INVALID_CONFIG, "Error.  Cannot find any valid %s.", name);
		return 0;
	}

	SV_VERBOSE(1, "Totals %d %ss.", i - 1, name);
	SV_VERBOSE(1, "Using '%s' for %s", picked, configname);

	return func;
}
static inline bool callDriver(SurviveContext* ctx, const char* DriverName, char* buffer) {
	DeviceDriver dd = GetDriver(DriverName);
	int r = dd(ctx);
	if (r < 0) {
		SV_WARN("Driver %s reports status %d", DriverName + strlen("DriverReg"), r);
		return false;
	}
	else {
		strcat(buffer, DriverName + strlen("DriverReg"));
		strcat(buffer, ", ");
	}
	return true; 
}
int survive_startup(SurviveContext *ctx) {
	int r = 0;
	int i = 0;
	ctx->state = SURVIVE_RUNNING;

	survive_install_recording(ctx);

	// initialize the button queue
	memset(&(ctx->buttonQueue), 0, sizeof(ctx->buttonQueue));
	ctx->buttonQueue.buttonservicesem = OGCreateSema();

	// start the thread to process button data
	ctx->buttonservicethread = OGCreateThread(button_servicer, ctx);

	PoserCB PreferredPoserCB = GetDriverByConfig(ctx, "Poser", "defaultposer", "MPFIT");
	ctx->lightcapproc = GetDriverByConfig(ctx, "Disambiguator", "disambiguator", "StateBased");

	const char *DriverName;

	i = 0;

	int loadedDrivers = 0;

	char buffer[1024] = "Loaded drivers: ";
	while ((DriverName = GetDriverNameMatching("DriverReg", i++))) {
		char driverNameSuffix[256] = { 0 };
		char* driverNameSuffix_p = driverNameSuffix;
		for (const char* c = DriverName + strlen("DriverReg");*c;c++) {
			*driverNameSuffix_p++ = tolower(*c);
		}
		
		int enabled = survive_config_is_set(ctx, driverNameSuffix);
		if (enabled && callDriver(ctx, DriverName, buffer)) {
			loadedDrivers++;
		}
	}

	if (ctx->currentError != SURVIVE_OK) {
		return ctx->currentError;
	}

	// Load the vive driver by default, even if not enabled as a flag
	if (loadedDrivers == 0 && callDriver(ctx, "DriverRegHTCVive", buffer)) {
		loadedDrivers++;
	}

	if (ctx->currentError != SURVIVE_OK) {
		return ctx->currentError;
	}

	buffer[strlen(buffer) - 2] = 0;
	SV_INFO("%s", buffer);

	// Apply poser to objects.
	for (i = 0; i < ctx->objs_ct; i++) {
		ctx->objs[i]->PoserFn = PreferredPoserCB;
	}

	// saving the config extra to make sure that the user has a config file they can change.
	config_save(ctx, survive_configs(ctx, "configfile", SC_GET, "config.json"));

	int calibrateMandatory = survive_configi(ctx, "force-calibrate", SC_GET, 0);
	if (calibrateMandatory) {
		SV_INFO("Force calibrate flag set -- clearing position on all lighthouses");
		for (int i = 0; i < ctx->activeLighthouses; i++) {
			ctx->bsd[i].PositionSet = 0;
		}
	}

	// If lighthouse positions are known, broadcast them
	for (int i = 0; i < ctx->activeLighthouses; i++) {
		if (ctx->bsd[i].PositionSet) {
			ctx->lighthouse_poseproc(ctx, i, &ctx->bsd[i].Pose, 0);
		}
	}

	if (ctx->objs_ct == 0 && ctx->driver_ct == 0) {
		SV_ERROR(SURVIVE_ERROR_NO_TRACKABLE_OBJECTS, "No trackable objects provided and no drivers are registered.");
	}

	return 0;
}

#define SURVIVE_HOOK_FN_DEF(hook)                                                                                      \
	SURVIVE_EXPORT void survive_install_##hook##_fn(SurviveContext *ctx, hook##_func fbp) {                            \
		ctx->hook##function = fbp ? fbp : survive_default_##hook;                                                      \
	}
#define SURVIVE_HOOK_PROCESS_DEF(hook)                                                                                 \
	SURVIVE_EXPORT void survive_install_##hook##_fn(SurviveContext *ctx, hook##_process_func fbp) {                    \
		ctx->hook##proc = fbp ? fbp : survive_default_##hook##_process;                                                \
	}
#define SURVIVE_HOOK_FEEDBACK_DEF(hook)                                                                                \
	SURVIVE_EXPORT void survive_install_##hook##_fn(SurviveContext *ctx, hook##_feedback_func fbp) {                   \
		ctx->hook##function = fbp ? fbp : survive_default_##hook;                                                      \
	}

#include "survive_hooks.h"

void survive_default_new_object_process(SurviveObject *so) {}
int survive_add_object(SurviveContext *ctx, SurviveObject *obj) {
	SV_INFO("Adding tracked object %s from %s", obj->codename, obj->drivername);
	int oldct = ctx->objs_ct;
	ctx->objs = realloc(ctx->objs, sizeof(SurviveObject *) * (oldct + 1));
	ctx->objs[oldct] = obj;
	ctx->objs_ct = oldct + 1;

	ctx->new_objectproc(obj);
	PoserCB PreferredPoserCB = GetDriverByConfig(ctx, "Poser", "defaultposer", "MPFIT");
	obj->PoserFn = PreferredPoserCB;

	return 0;
}

void survive_remove_object(SurviveContext *ctx, SurviveObject *obj) {
	int obj_idx = 0;
	for (obj_idx = 0; obj_idx < ctx->objs_ct; obj_idx++) {
		if (ctx->objs[obj_idx] == obj)
			break;
	}

	if (obj_idx == ctx->objs_ct) {
		SV_INFO("Warning: Tried to remove un-added object %p(%s)", obj, obj->codename);
		return;
	}

	// Swap the last item into this items slot; this assumes order doesn't matter in this list
	if (obj_idx != ctx->objs_ct - 1) {
		ctx->objs[obj_idx] = ctx->objs[ctx->objs_ct - 1];
	}

	ctx->objs_ct--;

	// Blank out the spot; but this is only really necessary for diagnostic reasons -- presumably no one will ever read
	// past the end of the list
	ctx->objs[ctx->objs_ct] = 0;

	SV_INFO("Removing tracked object %s from %s", obj->codename, obj->drivername);
	free(obj);
}
void survive_add_driver(SurviveContext *ctx, void *payload, DeviceDriverCb poll, DeviceDriverCb close,
						DeviceDriverMagicCb magic) {
	int oldct = ctx->driver_ct;
	ctx->drivers = realloc(ctx->drivers, sizeof(void *) * (oldct + 1));
	ctx->driverpolls = realloc(ctx->driverpolls, sizeof(DeviceDriverCb *) * (oldct + 1));
	ctx->drivercloses = realloc(ctx->drivercloses, sizeof(DeviceDriverCb *) * (oldct + 1));
	ctx->drivermagics = realloc(ctx->drivermagics, sizeof(DeviceDriverMagicCb *) * (oldct + 1));
	ctx->drivers[oldct] = payload;
	ctx->driverpolls[oldct] = poll;
	ctx->drivercloses[oldct] = close;
	ctx->drivermagics[oldct] = magic;
	ctx->driver_ct = oldct + 1;
}

int survive_send_magic(SurviveContext *ctx, int magic_code, void *data, int datalen) {
	int oldct = ctx->driver_ct;
	int i;
	for (i = 0; i < oldct; i++) {
		if (ctx->drivermagics[i]) {
			ctx->drivermagics[i](ctx, ctx->drivers[i], magic_code, data, datalen);
		}
	}
	return 0;
}

int survive_haptic(SurviveObject *so, uint8_t reserved, uint16_t pulseHigh, uint16_t pulseLow, uint16_t repeatCount) {
	if (NULL == so || NULL == so->haptic) {
		return -404;
	}

	return so->haptic(so, reserved, pulseHigh, pulseLow, repeatCount);
}

void survive_close(SurviveContext *ctx) {
	const char *DriverName;
	int r = 0;

	ctx->state = SURVIVE_CLOSING;

	// unlock/ post to button service semaphore so the thread can kill itself
	OGUnlockSema(ctx->buttonQueue.buttonservicesem);
	OGJoinThread(ctx->buttonservicethread);
	OGDeleteSema(ctx->buttonQueue.buttonservicesem);

	while ((DriverName = GetDriverNameMatching("DriverUnreg", r++))) {
		DeviceDriver dd = GetDriver(DriverName);
		SV_INFO("De-registering driver %s (%p)", DriverName, dd);
		r = dd(ctx);
		SV_INFO("Driver %s reports status %d", DriverName, r);
	}

	int oldct = ctx->driver_ct;
	int i;

	for (i = 0; i < ctx->objs_ct; i++) {
		PoserData pd;
		pd.pt = POSERDATA_DISASSOCIATE;
		if (ctx->objs[i]->PoserFn)
			ctx->objs[i]->PoserFn(ctx->objs[i], &pd);
	}

	for (i = 0; i < oldct; i++) {
		if (ctx->drivercloses[i]) {
			ctx->drivercloses[i](ctx, ctx->drivers[i]);
		} else {
			free(ctx->drivers[i]);
		}
	}

	config_save(ctx, survive_configs(ctx, "configfile", SC_GET, "config.json"));

	destroy_config_group(ctx->global_config_values);
	destroy_config_group(ctx->temporary_config_values);

	for (int lh = 0; lh < NUM_GEN2_LIGHTHOUSES; lh++)
		destroy_config_group(ctx->lh_config + lh);

	for (i = 0; i < ctx->objs_ct; i++) {
		survive_destroy_device(ctx->objs[i]);
	}

	free(ctx->objs);
	free(ctx->drivers);
	free(ctx->driverpolls);
	free(ctx->drivermagics);
	free(ctx->drivercloses);
	free(ctx->global_config_values);
	free(ctx->temporary_config_values);
	free(ctx->lh_config);
	free(ctx->calptr);
	free(ctx->recptr);

	free(ctx);
}

int survive_poll(struct SurviveContext *ctx) {
	int i, r;
	if (ctx->state == SURVIVE_STOPPED) {
		r = survive_startup(ctx);
		if (r) {
			return r;
		}
	}

	if (ctx->currentError != SURVIVE_OK) {
		return ctx->currentError;
	}

	int oldct = ctx->driver_ct;

	for (i = 0; i < oldct; i++) {
		r = ctx->driverpolls[i](ctx, ctx->drivers[i]);
		if (r) {
			SV_WARN("Driver reported %d", r);
			return r;
		}
	}

	return 0;
}

struct SurviveObject *survive_get_so_by_name(struct SurviveContext *ctx, const char *name) {
	int i;
	for (i = 0; i < ctx->objs_ct; i++) {
		if (strcmp(ctx->objs[i]->codename, name) == 0)
			return ctx->objs[i];
	}
	return 0;
}

#ifdef NOZLIB

#include <puff.h>

int survive_simple_inflate(struct SurviveContext *ctx, const char *input, int inlen, char *output, int outlen) {
	// Tricky: we actually get 2 bytes of data on the front.  I don't know what it's for. 0x78 0x9c - puff doesn't deal
	// with it well.
	unsigned long ol = outlen;
	unsigned long il = inlen - 2;
	int ret = puff(output, &ol, input + 2, &il);
	if (ret == 0)
		return ol;
	else {
		SV_INFO("puff returned error code %d\n", ret);
		return -5;
	}
}

#else

#include <zlib.h>

int survive_simple_inflate(struct SurviveContext *ctx, const uint8_t *input, int inlen, uint8_t *output, int outlen) {
	z_stream zs; // Zlib stream.  May only be used by configuration at beginning and by USB thread periodically.
	memset(&zs, 0, sizeof(zs));
	inflateInit(&zs); /// Consider checking error

	// XXX: Todo: If we find that this is not useful past the beginning (nix this here and move into the configuration
	// getter)
	zs.avail_in = inlen;
	zs.next_in = (z_const Bytef *)input;
	zs.avail_out = outlen;
	zs.next_out = output;

	int errorCode = inflate(&zs, Z_FINISH);
	if (errorCode != Z_STREAM_END) {
		SV_WARN("survive_simple_inflate could not inflate: %d (stream written to 'libz_error.stream')", errorCode);

		char fstname[128] = "libz_error.stream";
		FILE *f = fopen(fstname, "wb");
		fwrite(input, inlen, 1, f);
		fclose(f);

		return -1;
	}
	int len = zs.total_out;
	inflateEnd(&zs);
	return len;
}

#endif

const SurvivePose *survive_object_last_imu2world(const SurviveObject *so) { return &so->OutPoseIMU; }
const char *survive_object_codename(SurviveObject *so) { return so->codename; }

const char *survive_object_drivername(SurviveObject *so) { return so->drivername; }
const int8_t survive_object_charge(SurviveObject *so) { return so->charge; }
const bool survive_object_charging(SurviveObject *so) { return so->charging; }

const SurvivePose *survive_object_pose(SurviveObject *so) { return &so->OutPose; }

int8_t survive_object_sensor_ct(SurviveObject *so) { return so->sensor_ct; }
const FLT *survive_object_sensor_locations(SurviveObject *so) { return so->sensor_locations; }
const FLT *survive_object_sensor_normals(SurviveObject *so) { return so->sensor_normals; }

inline void survive_find_ang_velocity(SurviveAngularVelocity out, FLT tdiff, const LinmathQuat from,
									  const LinmathQuat to) {
	LinmathQuat vDiff = {1.};
	// quatfind(vDiff, comparison_pose.Rot.v, tracker->last_pose.Rot.v);
	quatfind(vDiff, from, to);

	quattoaxisanglemag(out, vDiff);
	scale3d(out, out, 1. / tdiff);
}
inline void survive_apply_ang_velocity(LinmathQuat out, const SurviveAngularVelocity v, FLT t, const LinmathQuat t0) {
	SurviveAngularVelocity vel;
	scale3d(vel, v, t);

	LinmathQuat rot_change;
	quatfromaxisanglemag(rot_change, vel);
	quatrotateabout(out, rot_change, t0);
}
