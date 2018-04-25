#include "survive_default_devices.h"
#include <jsmn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "json_helpers.h"

static SurviveObject *
survive_create_device(SurviveContext *ctx, const char *driver_name,
					  void *driver, const char *device_name, haptic_func fn) {
	SurviveObject *device = calloc(1, sizeof(SurviveObject));

	device->ctx = ctx;
	device->driver = driver;
	memcpy(device->codename, device_name, strlen(device_name));
	memcpy(device->drivername, driver_name, strlen(driver_name));

	device->timebase_hz = 48000000;
	device->imu_freq = 250.f;
	device->haptic = fn;

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
SurviveObject *survive_create_ww0(SurviveContext *ctx, const char *driver_name,
								  void *driver) {
	return survive_create_device(ctx, driver_name, driver, "WW0", 0);
}

static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
	if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
		strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
		return 0;
	}
	return -1;
}
static int ParsePoints(SurviveContext *ctx, SurviveObject *so, char *ct0conf,
					   FLT **floats_out, jsmntok_t *t, int i) {
	int k;
	int pts = t[i + 1].size;
	jsmntok_t *tk;

	so->sensor_ct = 0;
	*floats_out = malloc(sizeof(**floats_out) * 32 * 3);

	for (k = 0; k < pts; k++) {
		tk = &t[i + 2 + k * 4];

		int m;
		for (m = 0; m < 3; m++) {
			char ctt[128];

			tk++;
			int elemlen = tk->end - tk->start;

			if (tk->type != 4 || elemlen > sizeof(ctt) - 1) {
				SV_ERROR("Parse error in JSON\n");
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

	for (i = 1; i < r; i++) {
		jsmntok_t *tk = &t[i];

		char ctxo[100];
		int ilen = tk->end - tk->start;
		if (ilen > 99)
			ilen = 99;
		memcpy(ctxo, ct0conf + tk->start, ilen);
		ctxo[ilen] = 0;

		//				printf( "%d / %d / %d / %d %s %d\n", tk->type, tk->start,
		//tk->end, tk->size, ctxo, jsoneq(ct0conf, &t[i], "modelPoints") );
		//				printf( "%.*s\n", ilen, ct0conf + tk->start );

		if (jsoneq(ct0conf, tk, "modelPoints") == 0) {
			if (ParsePoints(ctx, so, ct0conf, &so->sensor_locations, t, i)) {
				break;
			}
		}
		if (jsoneq(ct0conf, tk, "modelNormals") == 0) {
			if (ParsePoints(ctx, so, ct0conf, &so->sensor_normals, t, i)) {
				break;
			}
		}

		if (jsoneq(ct0conf, tk, "acc_bias") == 0) {
			int32_t count = (tk + 1)->size;
			FLT *values = NULL;
			if (parse_float_array(ct0conf, tk + 2, &values, count) > 0) {
				so->acc_bias = values;
			}
		}
		if (jsoneq(ct0conf, tk, "acc_scale") == 0) {
			int32_t count = (tk + 1)->size;
			FLT *values = NULL;
			if (parse_float_array(ct0conf, tk + 2, &values, count) > 0) {
				so->acc_scale = values;
			}
		}

		if (jsoneq(ct0conf, tk, "gyro_bias") == 0) {
			int32_t count = (tk + 1)->size;
			FLT *values = NULL;
			if (parse_float_array(ct0conf, tk + 2, &values, count) > 0) {
				so->gyro_bias = values;
			}
		}
		if (jsoneq(ct0conf, tk, "gyro_scale") == 0) {
			int32_t count = (tk + 1)->size;
			FLT *values = NULL;
			if (parse_float_array(ct0conf, tk + 2, &values, count) > 0) {
				so->gyro_scale = values;
			}
		}
	}

	// Handle device-specific sacling.
	if (strcmp(so->codename, "HMD") == 0) {
		if (so->acc_scale) {
			scale3d(so->acc_scale, so->acc_scale, 1. / 8192.0);
		}
		if (so->acc_bias)
			scale3d(so->acc_bias, so->acc_bias, 2. / 1000.); // Odd but seems right.
		if (so->gyro_scale) {
			FLT deg_per_sec = 500;
			scale3d(so->gyro_scale, so->gyro_scale, deg_per_sec / (1 << 15) * LINMATHPI / 180.);
			// scale3d(so->gyro_scale, so->gyro_scale, 0.000065665);
		}

		so->imu_freq = 1000.f;
	} else if (memcmp(so->codename, "WM", 2) == 0) {
		if (so->acc_scale)
			scale3d(so->acc_scale, so->acc_scale, 2. / 8192.0);
		if (so->acc_bias)
			scale3d(so->acc_bias, so->acc_bias, 2. / 1000.); // Need to verify.

		FLT deg_per_sec = 2000;
		if (so->gyro_scale)
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
		if (so->acc_scale)
			scale3d(so->acc_scale, so->acc_scale, 2. / 8192.0);

		// If any other device, we know we at least need this.
		// I deeply suspect bias is in milligravities -JB
		if (so->acc_bias)
			scale3d(so->acc_bias, so->acc_bias, 1. / 1000.);

		// From datasheet, can be 250, 500, 1000, 2000 deg/s range over 16 bits
		FLT deg_per_sec = 2000;
		if (so->gyro_scale)
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
