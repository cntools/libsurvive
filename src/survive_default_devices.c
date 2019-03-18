#include "survive_default_devices.h"
#include "assert.h"
#include "json_helpers.h"
#include <jsmn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define HMD_IMU_HZ 1000.0f
#define VIVE_DEFAULT_IMU_HZ 250.0f

SurviveObject *survive_create_device(SurviveContext *ctx, const char *driver_name, void *driver,
									 const char *device_name, haptic_func fn) {
	SurviveObject *device = calloc(1, sizeof(SurviveObject));

	device->ctx = ctx;
	device->driver = driver;
	memcpy(device->codename, device_name, strlen(device_name));
	memcpy(device->drivername, driver_name, strlen(driver_name));

	device->timebase_hz = 48000000;
	device->imu_freq = VIVE_DEFAULT_IMU_HZ;
	device->haptic = fn;

	device->imu2trackref.Rot[0] = 1.;
	device->head2trackref.Rot[0] = 1.;

	for (int i = 0; i < 3; i++) {
		device->gyro_scale[i] = device->acc_scale[i] = 1.0;
	}

	return device;
}

SurviveObject *survive_create_hmd(SurviveContext *ctx, const char *driver_name,
								  void *driver) {
	return survive_create_device(ctx, driver_name, driver, "HMD", 0);
}

SurviveObject *survive_create_wm0(SurviveContext *ctx, const char *driver_name,
								  void *driver, haptic_func fn) {
	return survive_create_device(ctx, driver_name, driver, "WM0", fn);
}
SurviveObject *survive_create_wm1(SurviveContext *ctx, const char *driver_name,
								  void *driver, haptic_func fn) {
	return survive_create_device(ctx, driver_name, driver, "WM1", fn);
}
SurviveObject *survive_create_tr0(SurviveContext *ctx, const char *driver_name,
								  void *driver) {
	return survive_create_device(ctx, driver_name, driver, "TR0", 0);
}
SurviveObject *survive_create_tr1(SurviveContext *ctx, const char *driver_name,
								  void *driver) {
	return survive_create_device(ctx, driver_name, driver, "TR1", 0);
}
SurviveObject *survive_create_ww0(SurviveContext *ctx, const char *driver_name,
								  void *driver) {
	return survive_create_device(ctx, driver_name, driver, "WW0", 0);
}

static int jsoneq(const char *json, const jsmntok_t *tok, const char *s) {
	if (tok && tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
		strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
		return 0;
	}
	return -1;
}

static int ParsePoints(SurviveContext *ctx, SurviveObject *so, char *ct0conf, FLT **floats_out, const jsmntok_t *t) {
	int k;
	int pts = t[1].size;
	const jsmntok_t *tk;

	so->sensor_ct = 0;
	*floats_out = malloc(sizeof(**floats_out) * 32 * 3);

	for (k = 0; k < pts; k++) {
		tk = &t[2 + k * 4];

		int m;
		for (m = 0; m < 3; m++) {
			char ctt[128];

			tk++;
			int elemlen = tk->end - tk->start;

			if (tk->type != 4 || elemlen > sizeof(ctt) - 1) {
				SV_GENERAL_ERROR("Parse error in JSON\n");
				return 1;
			}

			memcpy(ctt, ct0conf + tk->start, elemlen);
			ctt[elemlen] = 0;
			FLT f = atof(ctt);
			int id = so->sensor_ct * 3 + m;
			(*floats_out)[id] = f;
		}
		so->sensor_ct++;
	}
	return 0;
}

static void vive_json_pose_to_survive_pose(const FLT *values, SurvivePose *pose) {
	for (int i = 0; i < 3; i++) {
		pose->Pos[i] = values[4 + i];
		pose->Rot[1 + i] = values[i];
	}
	pose->Rot[0] = values[3];
}

typedef struct stack_entry_s {
	struct stack_entry_s *previous;
	jsmntok_t *key;
} stack_entry_t;

typedef struct {
	FLT position[3];
	FLT plus_x[3];
	FLT plus_z[3];
} vive_pose_t;

int solve_vive_pose(SurvivePose *pose, const vive_pose_t *vpose) {
	if (vpose->plus_x[0] == 0.0 && vpose->plus_x[1] == 0.0 && vpose->plus_x[2] == 0.0)
		return 0;

	if (vpose->plus_z[0] == 0.0 && vpose->plus_z[1] == 0.0 && vpose->plus_z[2] == 0.0)
		return 0;

	copy3d(pose->Pos, vpose->position);

	LinmathVec3d plus_y;
	cross3d(plus_y, vpose->plus_z, vpose->plus_x);
	FLT m[] = {vpose->plus_x[0], plus_y[0],		   vpose->plus_z[0], vpose->plus_x[1], plus_y[1],
			   vpose->plus_z[1], vpose->plus_x[2], plus_y[2],		 vpose->plus_z[2]};

	quatfrommatrix33(pose->Rot, m);

	// Double check the math above; I'm 95% sure its right but kabsch tells us for sure; but is expensive. Remove this
	// when it's 100% clear its correct
	FLT axis[] = {1, 0, 0, 0, 0, 1};
	LinmathQuat rAssert;
	KabschCentered(rAssert, axis, vpose->plus_x, 2);

	LinmathQuat diff;
	quatfind(diff, rAssert, pose->Rot);
	assert(norm3d(diff + 1) < .001);

	return 1;
}

typedef struct {
	SurviveObject *so;
	vive_pose_t imu_pose;
	vive_pose_t head;
	int device_class;
} scratch_space_t;

static scratch_space_t scratch_space_init(SurviveObject *so) { return (scratch_space_t){.so = so}; }

static bool parse_ctx_sensitive_vive_pose_t(char *ct0conf, stack_entry_t *stack, const char *field_name,
											vive_pose_t *output) {
	if (stack->previous && jsoneq(ct0conf, stack->previous->key, field_name) == 0) {
		struct field {
			const char *name;
			FLT *vals;
		};

		struct field imufields[] = {
			{"plus_x", output->plus_x}, {"plus_z", output->plus_z}, {"position", output->position}};

		const jsmntok_t *tk = stack->key;
		for (int i = 0; i < sizeof(imufields) / sizeof(struct field); i++) {
			if (jsoneq(ct0conf, tk, imufields[i].name) == 0) {
				int32_t count = (tk + 1)->size;
				assert(count == 3);
				if (count == 3) {
					parse_float_array_in_place(ct0conf, tk + 2, imufields[i].vals, count);
				}
				break;
			}
		}

		return true;
	}

	return false;
}

static int process_jsonarray(scratch_space_t *scratch, char *ct0conf, stack_entry_t *stack) {
	SurviveObject *so = scratch->so;
	jsmntok_t const *const tk = stack->key;
	SurviveContext *ctx = so->ctx;

	/// CONTEXT FREE FIELDS
	if (jsoneq(ct0conf, tk, "modelPoints") == 0) {
		if (ParsePoints(ctx, so, ct0conf, &so->sensor_locations, tk)) {
			return -1;
		}
	} else if (jsoneq(ct0conf, tk, "modelNormals") == 0) {
		if (ParsePoints(ctx, so, ct0conf, &so->sensor_normals, tk)) {
			return -1;
		}
	} else if (jsoneq(ct0conf, tk, "channelMap") == 0) {
		int32_t count = (tk + 1)->size;
		int *values = NULL;
		if (parse_int_array(ct0conf, tk + 2, &values, count)) {
			int max_port = 0;
			for (int i = 0; i < count; i++) {
				max_port = max_port > values[i] ? max_port : values[i];
			}

			max_port++;
			assert(max_port <= 32);
			max_port = 32;
			so->channel_map = malloc(sizeof(int) * max_port);
			for (int i = 0; i < max_port; i++)
				so->channel_map[i] = -1;

			for (int i = 0; i < count; i++) {
				so->channel_map[values[i]] = i;
			}
		}
		free(values);
	} else if (jsoneq(ct0conf, tk, "acc_bias") == 0) {
		int32_t count = (tk + 1)->size;
		if (parse_float_array_in_place(ct0conf, tk + 2, so->acc_bias, count) <= 0) {
			SV_WARN("Could not parse acc_bias");
		}
	} else if (jsoneq(ct0conf, tk, "acc_scale") == 0) {
		int32_t count = (tk + 1)->size;
		if (parse_float_array_in_place(ct0conf, tk + 2, so->acc_scale, count) <= 0) {
			SV_WARN("Could not parse acc_scale");
		}
	} else if (jsoneq(ct0conf, tk, "gyro_bias") == 0) {
		int32_t count = (tk + 1)->size;
		if (parse_float_array_in_place(ct0conf, tk + 2, so->gyro_bias, count) <= 0) {
			SV_WARN("Could not parse gyro_bias");
		}
	} else if (jsoneq(ct0conf, tk, "gyro_scale") == 0) {
		int32_t count = (tk + 1)->size;
		if (parse_float_array_in_place(ct0conf, tk + 2, so->gyro_scale, count) <= 0) {
			SV_WARN("Could not parse gyro_scale");
		}
	} else if (jsoneq(ct0conf, tk, "trackref_from_imu") == 0) {
		int32_t count = (tk + 1)->size;
		if (count == 7) {
			FLT *values = NULL;
			if (parse_float_array(ct0conf, tk + 2, &values, count) > 0) {
				vive_json_pose_to_survive_pose(values, &so->imu2trackref);
				free(values);
			}
		}
	} else if (jsoneq(ct0conf, tk, "trackref_from_head") == 0) {
		int32_t count = (tk + 1)->size;
		if (count == 7) {
			FLT *values = NULL;
			if (parse_float_array(ct0conf, tk + 2, &values, count) > 0) {
				vive_json_pose_to_survive_pose(values, &so->head2trackref);
				free(values);
			}
		}
	}

	/// Context sensitive fields
	else {
		parse_ctx_sensitive_vive_pose_t(ct0conf, stack, "imu", &scratch->imu_pose);
		parse_ctx_sensitive_vive_pose_t(ct0conf, stack, "head", &scratch->head);
	}

	return 0;
}

static int process_jsontok(scratch_space_t *scratch, char *d, stack_entry_t *stack, jsmntok_t *t, int count) {
	int i, j, k;
	assert(count >= 0);
	if (count == 0) {
		return 0;
	}
	if (t->type == JSMN_PRIMITIVE) {
		return 1;
	} else if (t->type == JSMN_STRING) {
		if (stack && stack->key) {
			if (jsoneq(d, stack->key, "device_class") == 0) {
				if (strncmp("controller", d + t->start, t->end - t->start) == 0) {
					scratch->device_class = 1;
				} else if (strncmp("hmd", d + t->start, t->end - t->start) == 0) {
					scratch->device_class = 2;
				}
			} else if (jsoneq(d, stack->key, "device_serial_number") == 0) {
				int size = sizeof(scratch->so->serial_number);
				if (size > t->end - t->start)
					size = t->end - t->start;

				memcpy(scratch->so->serial_number, d + t->start, size);
			}
		}

		return 1;
	} else if (t->type == JSMN_OBJECT) {
		stack_entry_t entry;
		entry.previous = stack;
		j = 0;
		for (i = 0; i < t->size; i++) {
			entry.key = t + 1 + j;
			// print_stack_spot(d, &entry);
			// Skip the key
			j++;
			// Read value
			j += process_jsontok(scratch, d, &entry, t + 1 + j, count - j);
		}
		return j + 1;
	} else if (t->type == JSMN_ARRAY) {
		process_jsonarray(scratch, d, stack);
		j = 0;
		for (i = 0; i < t->size; i++) {
			j += process_jsontok(scratch, d, stack, t + 1 + j, count - j);
		}
		return j + 1;
	}
	return 0;
}

int survive_load_htc_config_format(SurviveObject *so, char *ct0conf, int len) {
	if (len == 0)
		return -1;

	SurviveContext *ctx = so->ctx;
	// From JSMN example.
	jsmn_parser p;
	jsmntok_t t[4096];
	jsmn_init(&p);
	int i;
	int r = jsmn_parse(&p, ct0conf, len, t, sizeof(t) / sizeof(t[0]));
	if (r < 0) {
		SV_INFO("Failed to parse JSON in HMD configuration: %d\n", r);
		return -1;
	}
	if (r < 1 || t[0].type != JSMN_OBJECT) {
		SV_INFO("Object expected in HMD configuration\n");
		return -2;
	}

	scratch_space_t scratch = scratch_space_init(so);
	process_jsontok(&scratch, ct0conf, 0, t, r);

	solve_vive_pose(&so->imu2trackref, &scratch.imu_pose);
	solve_vive_pose(&so->head2trackref, &scratch.head);

	SurvivePose trackref2imu = InvertPoseRtn(&so->imu2trackref);
	// InvertPose(&so->head2trackref, &so->head2trackref);

	for (int i = 0; i < so->sensor_ct; i++) {
		ApplyPoseToPoint(&so->sensor_locations[i * 3], &trackref2imu, &so->sensor_locations[i * 3]);
		quatrotatevector(&so->sensor_normals[i * 3], trackref2imu.Rot, &so->sensor_normals[i * 3]);
	}

	ApplyPoseToPose(&so->head2imu, &trackref2imu, &so->head2trackref);

	// Handle device-specific sacling.
	if (strcmp(so->codename, "HMD") == 0 || scratch.device_class == 2) {
		SV_INFO("%s is treated as HMD device", so->codename);
		scale3d(so->acc_scale, so->acc_scale, 1. / 8192.0);
		scale3d(so->acc_bias, so->acc_bias, 1. / 1000.0); // Odd but seems right.

		so->imu_freq = HMD_IMU_HZ;

		FLT deg_per_sec = 500;
		scale3d(so->gyro_scale, so->gyro_scale, deg_per_sec / (1 << 15) * LINMATHPI / 180.);
	} else if (memcmp(so->codename, "WM", 2) == 0) {
		scale3d(so->acc_scale, so->acc_scale, 2. / 8192.0);
		scale3d(so->acc_bias, so->acc_bias, 1. / 1000.); // Need to verify.

		FLT deg_per_sec = 2000;
		scale3d(so->gyro_scale, so->gyro_scale, deg_per_sec / (1 << 15) * LINMATHPI / 180.);

		int j;
		for (j = 0; j < so->sensor_ct; j++) {
			so->sensor_locations[j * 3 + 0] *= 1.0;
		}

	} else // Verified on WW, Need to verify on Tracker.
	{
		// 1G for accelerometer, from MPU6500 datasheet
		// this can change if the firmware changes the sensitivity.
		// When coming off of USB, these values are in units of .5g -JB
		scale3d(so->acc_scale, so->acc_scale, 2. / 8192.0);

		// If any other device, we know we at least need this.
		// I deeply suspect bias is in milligravities -JB
		scale3d(so->acc_bias, so->acc_bias, 1. / 1000.);

		// From datasheet, can be 250, 500, 1000, 2000 deg/s range over 16 bits
		FLT deg_per_sec = 2000;
		scale3d(so->gyro_scale, so->gyro_scale, deg_per_sec / (1 << 15) * LINMATHPI / 180.);
		// scale3d(so->gyro_scale, so->gyro_scale, 3.14159 / 1800. / 1.8);
	}

	char fname[64];

	sprintf(fname, "calinfo/%s_points.csv", so->codename);
	FILE *f = fopen(fname, "w");
	int j;
	if(f) {
	  for (j = 0; j < so->sensor_ct; j++) {
	    fprintf(f, "%f %f %f\n", so->sensor_locations[j * 3 + 0],
		    so->sensor_locations[j * 3 + 1],
		    so->sensor_locations[j * 3 + 2]);
	  }
	  fclose(f);
	}

	if(f) {
	  sprintf(fname, "calinfo/%s_normals.csv", so->codename);
	  f = fopen(fname, "w");
	  for (j = 0; j < so->sensor_ct; j++) {
	    fprintf(f, "%f %f %f\n", so->sensor_normals[j * 3 + 0],
		    so->sensor_normals[j * 3 + 1], so->sensor_normals[j * 3 + 2]);
	  }
	  fclose(f);
	}

	return 0;
}
