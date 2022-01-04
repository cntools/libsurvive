#include <survive.h>

static volatile int keepRunning = 1;

#ifdef __linux__

#include <assert.h>
#include <cnmatrix/cn_matrix.h>
#include <math.h>
#include <signal.h>
#include <stdlib.h>
#include <xdo.h>

struct ControllerInfo {
	SurviveObject *so;
	LinmathVec3d pointVector;
	LinmathVec3d pointVectorSum;
	int pointVectorSumCnt;
	bool triggerIn;
};

struct ControllerInfo infos[32] = {0};
struct ControllerInfo *getControllerInfo(SurviveObject *so) {
	int i = 0;
	for (i = 0; i < sizeof(infos) / sizeof(infos[0]); i++) {
		if (infos[i].so == so)
			return &infos[i];
		if (infos[i].so == 0)
			break;
	}

	infos[i].so = so;
	infos[i].pointVector[2] = -1;
	infos[i].pointVector[1] = -1;
	normalize3d(infos[i].pointVector, infos[i].pointVector);

	add3d(infos[i].pointVectorSum, infos[i].pointVectorSum, infos[i].pointVector);
	infos[i].pointVectorSumCnt++;

	SurviveContext *ctx = so->ctx;
	SV_INFO("Using " Point3_format " as point vector for %s", LINMATH_VEC3_EXPAND(infos[i].pointVector), so->codename);

	return &infos[i];
}

void add_pointVector(struct ControllerInfo *info, const LinmathPoint3d additional) {
	add3d(info->pointVectorSum, additional, info->pointVectorSum);
	info->pointVectorSumCnt++;
	scale3d(info->pointVector, info->pointVectorSum, 1. / info->pointVectorSumCnt);
}

struct LinmathPlane planeFromPoints(FLT *pts, size_t len) {
	struct LinmathPlane rtn = {0};
	if (len < 3)
		return rtn;

	LinmathVec3d sum = {0};
	for (int i = 0; i < len; i++) {
		add3d(sum, sum, pts + (i * 3));
	}
	LinmathVec3d centroid = {0};
	scale3d(centroid, sum, 1. / len);

	FLT xx = 0, xy = 0, xz = 0, yy = 0, yz = 0, zz = 0;
	for (int i = 0; i < len; i++) {
		LinmathVec3d r;
		sub3d(r, pts + i * 3, centroid);
		xx += r[0] * r[0];
		xy += r[0] * r[1];
		xz += r[0] * r[2];
		yy += r[1] * r[1];
		yz += r[1] * r[2];
		zz += r[2] * r[2];
	}

	FLT det_x = yy * zz - yz * yz;
	FLT det_y = xx * zz - xz * xz;
	FLT det_z = xx * yy - xy * xy;
	FLT det_max = det_x;
	if (det_y > det_max)
		det_max = det_y;
	if (det_z > det_max)
		det_max = det_z;

	if (det_max <= 0)
		return rtn;

	LinmathVec3d norm;
	if (det_max == det_x) {
		norm[0] = det_x;
		norm[1] = xz * yz - xy * zz;
		norm[2] = xy * yz - xz * yy;
	} else if (det_max == det_y) {
		norm[0] = xz * yz - xy * zz;
		norm[1] = det_y;
		norm[2] = xy * xz - yz * xx;
	} else {
		norm[0] = xy * yz - xz * yy;
		norm[1] = xy * xz - yz * xx;
		norm[2] = det_z;
	}

	copy3d(rtn.normal, norm);
	copy3d(rtn.origin, centroid);
	return rtn;
}

struct LinmathLine3d getRay(SurviveObject *so) {
	struct LinmathLine3d ray = {0};
	LinmathVec3d origin = {0};
	struct ControllerInfo *ci = getControllerInfo(so);
	ApplyPoseToPoint(ray.a, &so->OutPose, origin);
	ApplyPoseToPoint(ray.b, &so->OutPose, ci->pointVector);
	return ray;
}

struct ScreenInfo {
	LinmathVec3d corners[4]; // TL TR BR BL

	size_t total_samples_cnt;
	size_t samples_cnt[4];
	struct LinmathLine3d samples[4][128];

	LinmathPose tx2screen;
	FLT scale2screen;

	struct LinmathPlane plane;
};

void copyLine(struct LinmathLine3d *dst, const struct LinmathLine3d *src) {
	copy3d(dst->a, src->a);
	copy3d(dst->b, src->b);
}

struct ScreenInfo screen = {
	.corners = {
		0
		/* { +9.770404e-01,   +1.447184e+00,   +1.532997e+00},
		 { +1.843338e+00,   +5.526268e-01,   +1.539447e+00},
		 { +1.843338e+00,   +5.526268e-01,   +7.500533e-01},//{ +1.714258e+00,   +8.190169e-01,   +5.637074e-01},
		 {+9.867645e-01,   +1.422196e+00 ,  +7.500533e-01},*/
	}};

void write_calibration(const char *fn) {
	FILE *f = fopen(fn, "w");
	for (int i = 0; i < sizeof(infos) / sizeof(infos[0]); i++) {
		if (infos[i].so == 0)
			break;
		fprintf(f, "Device: %s " Point3_format "\n", infos[i].so->codename, LINMATH_VEC3_EXPAND(infos[i].pointVector));
	}
	fprintf(f, "\n");

	for (int i = 0; i < 4; i++) {
		fprintf(f, Point3_format "\n", LINMATH_VEC3_EXPAND(screen.corners[i]));
	}
	fclose(f);
}

void load_calibration(SurviveContext *ctx, const char *fn) {
	FILE *f = fopen(fn, "r");
	for (int i = 0; i < sizeof(infos) / sizeof(infos[0]); i++) {
		char devname[10];
		int read = fscanf(f, "Device: %s " Point3_sformat "\n", devname, &infos[i].pointVector[0],
						  &infos[i].pointVector[1], &infos[i].pointVector[2]);
		if (read < 4)
			break;

		infos[i].so = survive_get_so_by_name(ctx, devname);
		if (infos[i].so == 0)
			i--;
	}

	for (int i = 0; i < 4; i++) {
		int corners =
			fscanf(f, Point3_sformat "\n", &screen.corners[i][0], &screen.corners[i][1], &screen.corners[i][2]);
		assert(corners == 3);
	}
	fclose(f);
}

void add_sample(SurviveContext *ctx, struct ScreenInfo *info, struct LinmathLine3d *ray) {
	if (info->total_samples_cnt < 8) {
		int target = info->total_samples_cnt % 4;

		// SV_INFO("Integrating for %d " Point6_format, target, LINMATH_VEC6_EXPAND(ray->a));
		copyLine(&info->samples[target][info->samples_cnt[target]], ray);
		info->samples_cnt[target]++;

		for (int i = info->total_samples_cnt; i < 4; i++)
			linmath_pt_along_line(info->corners[i], ray, 2.5);

		info->total_samples_cnt++;
		if (info->total_samples_cnt > 4) {
			SV_INFO("Solving for %d %d", target, (int)info->samples_cnt[target]);
			linmath_find_best_intersection(info->corners[target], info->samples[target], info->samples_cnt[target]);
		}
	} else {
		FLT md = 1e5;
		int best_i = 0;
		for (int i = 0; i < 4; i++) {
			FLT t;
			FLT d = linmath_point_distance_from_line(ray, info->corners[i], &t);
			SV_INFO("corner %3d %3.6f %3.6f", i, t, d);
			if (d < md) {
				md = d;
				best_i = i;
			}
		}

		copyLine(&info->samples[best_i][info->samples_cnt[best_i]], ray);
		info->total_samples_cnt++;
		info->samples_cnt[best_i]++;

		linmath_find_best_intersection(info->corners[best_i], info->samples[best_i], info->samples_cnt[best_i]);
	}

	LinmathPoint3d m3out;
	linmath_pt_along_line(m3out, ray, 3);
	if (ctx->recptr) {
		fprintf(stdout, "%0.6f POLY ray%d " Point3_format " " Point3_format "\n", OGRelativeTime(),
				(int)info->total_samples_cnt, LINMATH_VEC3_EXPAND(ray->a), LINMATH_VEC3_EXPAND(m3out));

		fprintf(stdout, "%0.6f POLY screen " Point3_format " " Point3_format " " Point3_format " " Point3_format "\n",
				OGRelativeTime(), LINMATH_VEC3_EXPAND(info->corners[0]), LINMATH_VEC3_EXPAND(info->corners[1]),
				LINMATH_VEC3_EXPAND(info->corners[2]), LINMATH_VEC3_EXPAND(info->corners[3]));
	}

	write_calibration("vive_mouse.cfg");
}

void findScreenTx(struct ScreenInfo *info) {
	size_t w = 3840;
	size_t h = 2160;
	LinmathVec3d screenCorners[4] = {{0, 0, 0}, {w, 0, 0}, {w, h, 0}, {0, h, 0}};

	KabschScaled(&info->tx2screen, &info->scale2screen, (const FLT *)screenCorners, (const FLT *)info->corners, 4);

	info->plane = planeFromPoints((FLT *)info->corners, 4);
}

void findPlaneRayIntersection(LinmathPoint3d pt, const struct LinmathPlane *p, const struct LinmathLine3d *ray) {
	LinmathPoint3d rayDirection;
	linmath_get_line_dir(rayDirection, ray);
	FLT denom = dot3d(p->normal, rayDirection);

	if (fabs(denom) > 1e-5) {
		LinmathPoint3d diff;
		sub3d(diff, p->origin, ray->a);
		FLT t = dot3d(diff, p->normal) / denom;
		linmath_pt_along_line(pt, ray, t);
		return;
	}
}

size_t corners_idx = 0; // 4;
void findScreenCoordinates(LinmathPoint2d xy, struct ScreenInfo *info, struct LinmathLine3d *ray, bool output) {
	if (info->scale2screen == 0) {
		findScreenTx(info);
	}

	LinmathPoint3d intersection;
	findPlaneRayIntersection(intersection, &info->plane, ray);

	struct LinmathLine3d tl_tr = {0};
	copy3d(tl_tr.a, info->corners[0]);
	copy3d(tl_tr.b, info->corners[1]);
	struct LinmathLine3d tl_bl;
	copy3d(tl_bl.a, info->corners[0]);
	copy3d(tl_bl.b, info->corners[3]);
	FLT px, py;
	linmath_point_distance_from_line(&tl_tr, intersection, &px);
	linmath_point_distance_from_line(&tl_bl, intersection, &py);

	size_t w = 3840;
	size_t h = 2160;
	xy[0] = px * w;
	xy[1] = py * h;
}

xdo_t *xdo = 0;
static void pose_fn(SurviveObject *so, survive_long_timecode timecode, const SurvivePose *pose) {
	survive_default_pose_process(so, timecode, pose);
	struct LinmathLine3d r = getRay(so);

	LinmathPoint3d m3out;
	linmath_pt_along_line(m3out, &r, 3);
	if (so->ctx->recptr) {
		fprintf(stdout, "%0.6f POLY pointer " Point3_format " " Point3_format "\n", OGRelativeTime(),
				LINMATH_VEC3_EXPAND(r.a), LINMATH_VEC3_EXPAND(m3out));
		fflush(stdout);
	}

	if (so->touchmask & (1 << SURVIVE_BUTTON_TRACKPAD)) {
		LinmathPoint2d xy;
		findScreenCoordinates(xy, &screen, &r, 0);
		if (xdo != 0) {
			xdo_move_mouse(xdo, xy[0], xy[1], 0);
		}
		// SV_INFO("%s xy %8.4f\t%8.4f", so->codename, xy[0], xy[1]);
	}
}
static void button_fn(SurviveObject *so, enum SurviveInputEvent eventType, enum SurviveButton buttonId,
					  const enum SurviveAxis *axisIds, const SurviveAxisVal_t *axisVals) {
	survive_default_button_process(so, eventType, buttonId, axisIds, axisVals);
	SurviveContext *ctx = so->ctx;

	struct ControllerInfo *ci = getControllerInfo(so);
	SurviveAxisVal_t v = so->axis[SURVIVE_AXIS_TRIGGER];
	if (quatiszero(so->OutPose.Rot)) {
		return;
	}
	struct LinmathLine3d r = getRay(so);

	if (buttonId == SURVIVE_BUTTON_TRIGGER && eventType == SURVIVE_INPUT_EVENT_BUTTON_DOWN) {
		LinmathPoint2d xy;
		findScreenCoordinates(xy, &screen, &r, 1);

		if (so->buttonmask & (1 << SURVIVE_BUTTON_B)) {
			LinmathPoint3d hit;
			FLT dmin = 10000.0;
			for (int i = 0; i < ctx->activeLighthouses; i++) {
				FLT t;
				FLT d = linmath_point_distance_from_line(&r, ctx->bsd[i].Pose.Pos, &t);
				if (d < dmin) {
					dmin = d;
					copy3d(hit, ctx->bsd[i].Pose.Pos);
				}
				SV_INFO("LH %3d %3.6f %3.6f", i, t, d);
			}

			if (!quatiszero(so->OutPose.Rot)) {
				SurvivePose world2so = InvertPoseRtn(&so->OutPose);
				LinmathPoint3d v = {0};
				ApplyPoseToPoint(v, &world2so, hit);
				normalize3d(v, v);
				add_pointVector(ci, v);
				SV_INFO("Using " Point3_format " as point vector for %s", LINMATH_VEC3_EXPAND(v), so->codename);
				SV_INFO("Using " Point3_format " as point vector for %s", LINMATH_VEC3_EXPAND(ci->pointVector),
						so->codename);
				write_calibration("vive_mouse.cfg");
			}
		}
		if (so->buttonmask & (1 << SURVIVE_BUTTON_A)) {
			add_sample(ctx, &screen, &r);
		} else {
			if (xdo != 0) {
				xdo_move_mouse(xdo, xy[0], xy[1], 0);
				xdo_click_window(xdo, CURRENTWINDOW, 1);
				// xdo_move_mouse(xdo, -10, -10, 0);
			}
		}
	}
}

void intHandler(int dummy) {
	if (keepRunning == 0)
		exit(-1);
	keepRunning = 0;
}

#endif

int main(int argc, char **argv) {
#ifdef __linux__
	signal(SIGINT, intHandler);
	signal(SIGTERM, intHandler);
	signal(SIGKILL, intHandler);
#endif

	xdo = xdo_new(0);

	SurviveContext *ctx = survive_init(argc, argv);
	if (ctx == 0) // implies -help or similiar
		return 0;

	survive_startup(ctx);
	survive_install_button_fn(ctx, button_fn);
	survive_install_pose_fn(ctx, pose_fn);

	load_calibration(ctx, "vive_mouse.cfg");

	if (ctx->recptr) {
		fprintf(stdout, "%0.6f POLY screen " Point3_format " " Point3_format " " Point3_format " " Point3_format "\n",
				OGRelativeTime(), LINMATH_VEC3_EXPAND(screen.corners[0]), LINMATH_VEC3_EXPAND(screen.corners[1]),
				LINMATH_VEC3_EXPAND(screen.corners[2]), LINMATH_VEC3_EXPAND(screen.corners[3]));
	}

	while (keepRunning && survive_poll(ctx) == 0) {
	}

	survive_close(ctx);
	return 0;
}
