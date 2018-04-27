// Copyright 2016 <>< C. N. Lohr, FULLY Under MIT/x11 License.
// All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include "survive_internal.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <survive.h>

#include "os_generic.h"
#include "survive_config.h"
#include "survive_default_devices.h"
#include "survive_playback.h"

#ifdef __APPLE__
#define z_const const
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

static void survivefault(struct SurviveContext *ctx, const char *fault) {
	fprintf(stderr, "Error: %s\n", fault);
	exit(-1);
}

static void survivenote(struct SurviveContext *ctx, const char *fault) {
	survive_recording_info_process(ctx, fault);
	fprintf(stderr, "Info: %s\n", fault);
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

SurviveContext *survive_init_internal(int argc, char *const *argv) {
	int i;

#ifdef RUNTIME_SYMNUM
	if (!did_runtime_symnum) {
		EnumerateSymbols(SymnumCheck);
		did_runtime_symnum = 1;
	}
#endif
#ifdef MANUAL_REGISTRATION
	// note: this manual registration is currently only in use on builds using Visual Studio.

	static int did_manual_driver_registration = 0;
	if (did_manual_driver_registration == 0) {
#define MANUAL_DRIVER_REGISTRATION(func)                                                                               \
	int func(SurviveObject *so, PoserData *pd);                                                                        \
	RegisterDriver(#func, &func);

		MANUAL_DRIVER_REGISTRATION(PoserCharlesSlow)
		MANUAL_DRIVER_REGISTRATION(PoserDaveOrtho)
		MANUAL_DRIVER_REGISTRATION(PoserDummy)
		MANUAL_DRIVER_REGISTRATION(PoserEPNP)
		MANUAL_DRIVER_REGISTRATION(PoserSBA)
		MANUAL_DRIVER_REGISTRATION(PoserCharlesRefine)
		MANUAL_DRIVER_REGISTRATION(PoserMPFIT)

		MANUAL_DRIVER_REGISTRATION(DriverRegHTCVive)
		MANUAL_DRIVER_REGISTRATION(DriverRegPlayback)

		MANUAL_DRIVER_REGISTRATION(DisambiguatorCharles)
		MANUAL_DRIVER_REGISTRATION(DisambiguatorStateBased)
		MANUAL_DRIVER_REGISTRATION(DisambiguatorTurvey)
		did_manual_driver_registration = 1;
	}
#endif

	SurviveContext *ctx = calloc(1, sizeof(SurviveContext));

	ctx->state = SURVIVE_STOPPED;

	survive_config_populate_ctx( ctx );

	ctx->global_config_values = malloc(sizeof(config_group));
	ctx->temporary_config_values = malloc(sizeof(config_group));
	ctx->lh_config = malloc(sizeof(config_group) * NUM_LIGHTHOUSES);

	// initdata
	init_config_group(ctx->global_config_values, 10, ctx);
	init_config_group(ctx->temporary_config_values, 20, ctx);
	for( i = 0; i < NUM_LIGHTHOUSES; i++ )
			init_config_group(&ctx->lh_config[i], 10, ctx);

	// Process command-line parameters.
	char *const *av = argv + 1;
	char *const *argvend = argv + argc;
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
			case 'c':
				vartoupdate = "configfile";
				break;
			default:
				fprintf(stderr, "Error: unknown parameter %s\n", *av);
				showhelp = 1;
			}

			if (vartoupdate) {
				const char *name = *av + 2; // Skip the '--';
				bool flagArgument = (av + 1 == argvend) || av[1][0] == '-';

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
	if (showhelp) {
		// Can't use SV_ERROR here since we don't have a context to send to yet.
		fprintf(stderr, "libsurvive - usage:\n");
		fprintf(stderr, " --[parameter] [value]   - sets parameter\n");
		fprintf(stderr, " -h                      - shows help.\n");
		fprintf(stderr, " -p [poser]              - use a specific defaultposer.\n");
		fprintf(stderr, " -l [lighthouse count]   - use a specific number of lighthoses.\n");
		fprintf(stderr, " -c [config file]        - set config file\n");
		fprintf(stderr, " --record [log file]     - Write all events to the given record file.\n");
		fprintf(stderr, " --playback [log file]   - Read events from the given file instead of USB devices.\n");
		fprintf(stderr, " --playback-factor [f]   - Time factor of playback -- 1 is run at the same timing as "
						"original, 0 is run as fast as possible.\n");
	}

	config_read(ctx, survive_configs(ctx, "configfile", SC_GET, "config.json"));
	ctx->activeLighthouses = survive_configi(ctx, "lighthousecount", SC_SETCONFIG, 2);
	config_read_lighthouse(ctx->lh_config, &(ctx->bsd[0]), 0);
	config_read_lighthouse(ctx->lh_config, &(ctx->bsd[1]), 1);

	if( showhelp )
	{
		survive_print_known_configs( ctx );
		return 0;
	}

	ctx->faultfunction = survivefault;
	ctx->notefunction = survivenote;
	ctx->lightproc = survive_default_light_process;
	ctx->imuproc = survive_default_imu_process;
	ctx->angleproc = survive_default_angle_process;
	ctx->lighthouseposeproc = survive_default_lighthouse_pose_process;
	ctx->configfunction = survive_default_htc_config_process;
	ctx->poseproc = survive_default_raw_pose_process;

	ctx->calibration_config = survive_calibration_config_ctor();
	ctx->calibration_config.use_flag = (enum SurviveCalFlag)survive_configi(ctx, "bsd-cal", SC_GET, SVCal_All);

	return ctx;
}

survive_timecode survive_timecode_difference(survive_timecode most_recent, survive_timecode least_recent) {
	uint32_t diff = 0;
	if (most_recent > least_recent) {
		diff = most_recent - least_recent;
	} else {
		diff = least_recent - most_recent;
	}

	if (diff > 0xFFFFFFFF / 2)
		return 0x7FFFFFFF / 2 - diff;
	return diff;
}

void *GetDriverByConfig(SurviveContext *ctx, const char *name, const char *configname, const char *configdef,
						int verbose) {
	const char *Preferred = survive_configs(ctx, configname, SC_SETCONFIG, configdef);
	const char *DriverName = 0;
	const char *picked = 0;
	int i = 0;
	void *func = 0;
	int prefixLen = strlen(name);

	if (verbose > 1)
		SV_INFO("Available %ss:", name);
	while ((DriverName = GetDriverNameMatching(name, i++))) {
		void *p = GetDriver(DriverName);

		bool match = strcmp(DriverName, Preferred) == 0 || strcmp(DriverName + prefixLen, Preferred) == 0;
		if (verbose > 1)
			SV_INFO("\t%c%s", match ? '*' : ' ', DriverName + prefixLen);
		if (!func || match) {
			func = p;
			picked = (DriverName + prefixLen);
		}
	}
	if (!func) {
		SV_ERROR("Error.  Cannot find any valid %s.", name);
	}
	if (verbose > 1)
		SV_INFO("Totals %d %ss.", i - 1, name);
	if (verbose > 0)
		SV_INFO("Using '%s' for %s", picked, configname);

	return func;
}

int survive_startup(SurviveContext *ctx) {
	int r = 0;
	int i = 0;

	survive_install_recording(ctx);

	// initialize the button queue
	memset(&(ctx->buttonQueue), 0, sizeof(ctx->buttonQueue));
	ctx->buttonQueue.buttonservicesem = OGCreateSema();

	// start the thread to process button data
	ctx->buttonservicethread = OGCreateThread(button_servicer, ctx);

	PoserCB PreferredPoserCB = GetDriverByConfig(ctx, "Poser", "defaultposer", "SBA", 2);
	ctx->lightcapfunction = GetDriverByConfig(ctx, "Disambiguator", "disambiguator", "Turvey", 2);

	const char *DriverName;

	i = 0;

	while ((DriverName = GetDriverNameMatching("DriverReg", i++))) {
		DeviceDriver dd = GetDriver(DriverName);
		SV_INFO("Loading driver %s (%p) (%d)", DriverName, dd, i);
		r = dd(ctx);
		SV_INFO("Driver %s reports status %d", DriverName, r);
	}

	// Apply poser to objects.
	for (i = 0; i < ctx->objs_ct; i++) {
		ctx->objs[i]->PoserFn = PreferredPoserCB;
	}

	// saving the config extra to make sure that the user has a config file they can change.
	config_save(ctx, survive_configs(ctx, "configfile", SC_GET, "config.json"));

	ctx->state = SURVIVE_RUNNING;

	int calibrateMandatory = survive_configi(ctx, "calibrate", SC_GET, 0);
	int calibrateForbidden = survive_configi(ctx, "calibrate", SC_GET, 1) == 0;
	if (calibrateMandatory && calibrateForbidden) {
		SV_INFO("Contradictory settings --calibrate and --no-calibrate specified. Switching to normal behavior.");
		calibrateMandatory = calibrateForbidden = 0;
	}

	if (!calibrateForbidden) {
		bool isCalibrated = true;
		for (int i = 0; i < ctx->activeLighthouses; i++) {
			isCalibrated &= ctx->bsd[i].PositionSet;
		}

		if (!isCalibrated) {
			SV_INFO("Uncalibrated configuration detected. Attaching calibration. Please don't move tracked objects for "
					"the duration of calibration. Pass '--no-calibrate' to skip calibration");
		} else {
			SV_INFO("Calibration requested. Previous calibration will be overwritten.");
		}

		bool doCalibrate = isCalibrated == false || calibrateMandatory;

		if (doCalibrate) {
			survive_cal_install(ctx);
		}
	}

	// If lighthouse positions are known, broadcast them
	for (int i = 0; i < ctx->activeLighthouses; i++) {
		if (ctx->bsd[i].PositionSet) {
			ctx->lighthouseposeproc(ctx, i, &ctx->bsd[i].Pose, 0);
		}
	}

	if( ctx->objs_ct == 0 )
	{
		SV_ERROR( "Fatal error: No trackable objects found in any calibrators." );
	}

	return 0;
}

void survive_install_info_fn(SurviveContext *ctx, text_feedback_func fbp) {
	if (fbp)
		ctx->notefunction = fbp;
	else
		ctx->notefunction = survivenote;
}

void survive_install_htc_config_fn(SurviveContext *ctx, htc_config_func fbp) {
	if (fbp)
		ctx->configfunction = fbp;
	else
		ctx->configfunction = survive_default_htc_config_process;
}

void survive_install_error_fn(SurviveContext *ctx, text_feedback_func fbp) {
	if (fbp)
		ctx->faultfunction = fbp;
	else
		ctx->faultfunction = survivefault;
}

void survive_install_light_fn(SurviveContext *ctx, light_process_func fbp) {
	if (fbp)
		ctx->lightproc = fbp;
	else
		ctx->lightproc = survive_default_light_process;
}

void survive_install_imu_fn(SurviveContext *ctx, imu_process_func fbp) {
	if (fbp)
		ctx->imuproc = fbp;
	else
		ctx->imuproc = survive_default_imu_process;
}

void survive_install_angle_fn(SurviveContext *ctx, angle_process_func fbp) {
	if (fbp)
		ctx->angleproc = fbp;
	else
		ctx->angleproc = survive_default_angle_process;
}

void survive_install_button_fn(SurviveContext *ctx, button_process_func fbp) {
	if (fbp)
		ctx->buttonproc = fbp;
	else
		ctx->buttonproc = survive_default_button_process;
}

void survive_install_pose_fn(SurviveContext *ctx, pose_func fbp) {
	if (fbp)
		ctx->poseproc = fbp;
	else
		ctx->poseproc = survive_default_raw_pose_process;
}

void survive_install_lighthouse_pose_fn(SurviveContext *ctx, lighthouse_pose_func fbp) {
	if (fbp)
		ctx->lighthouseposeproc = fbp;
	else
		ctx->lighthouseposeproc = survive_default_lighthouse_pose_process;
}

int survive_add_object(SurviveContext *ctx, SurviveObject *obj) {
	int oldct = ctx->objs_ct;
	ctx->objs = realloc(ctx->objs, sizeof(SurviveObject *) * (oldct + 1));
	ctx->objs[oldct] = obj;
	ctx->objs_ct = oldct + 1;
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
		ctx->drivercloses[i](ctx, ctx->drivers[i]);
	}

	config_save(ctx, survive_configs(ctx, "configfile", SC_GET, "config.json"));

	destroy_config_group(ctx->global_config_values);
	destroy_config_group(ctx->temporary_config_values);
	destroy_config_group(ctx->lh_config);

	for (i = 0; i < ctx->objs_ct; i++) {
		free(ctx->objs[i]->sensor_locations);
		free(ctx->objs[i]->sensor_normals);
		free(ctx->objs[i]);
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
		if (r)
			return r;
	}

	int oldct = ctx->driver_ct;

	for (i = 0; i < oldct; i++) {
		r = ctx->driverpolls[i](ctx, ctx->drivers[i]);
		if (r)
			return r;
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

	if (inflate(&zs, Z_FINISH) != Z_STREAM_END) {
		SV_INFO("survive_simple_inflate could not inflate.");
		return -1;
	}
	int len = zs.total_out;
	inflateEnd(&zs);
	return len;
}

#endif

const char *survive_object_codename(SurviveObject *so) { return so->codename; }

const char *survive_object_drivername(SurviveObject *so) { return so->drivername; }
const int8_t survive_object_charge(SurviveObject *so) { return so->charge; }
const bool survive_object_charging(SurviveObject *so) { return so->charging; }

const SurvivePose *survive_object_pose(SurviveObject *so) { return &so->OutPose; }

int8_t survive_object_sensor_ct(SurviveObject *so) { return so->sensor_ct; }
const FLT *survive_object_sensor_locations(SurviveObject *so) { return so->sensor_locations; }
const FLT *survive_object_sensor_normals(SurviveObject *so) { return so->sensor_normals; }
