// This code is mostly adapted from the equivalent bc_svd code
#pragma once

#ifndef WITH_OPENCV
#include "../../redist/minimal_opencv.h"
#else
#include <opencv/cv.h>
#endif

#include "../redist/linmath.h"

typedef double LinmathPoint4d[4];

typedef void (*bc_svd_fill_M_fn)(void *user, double *eq, int axis, double angle);

typedef struct {
	size_t obj_cnt;
	const LinmathPoint3d *obj_pts;
	LinmathPoint4d *alphas;

	LinmathPoint3d control_points[4];

	bc_svd_fill_M_fn fillFn;
	void *user;
} bc_svd_setup;

typedef struct {
	int obj_idx;
	int axis;
	FLT angle;
} bc_svd_meas_t;

typedef struct {
	bc_svd_setup setup;

	size_t meas_space, meas_cnt;
	bc_svd_meas_t *meas; // [meas_cnt]

	LinmathPoint3d *object_pts_in_camera; // [obj_cnt]
	LinmathPoint3d control_points_in_camera[4];
} bc_svd;

void bc_svd_bc_svd(bc_svd *self, void *user, bc_svd_fill_M_fn fillFn, const LinmathPoint3d *obj_pts, size_t obj_cnt);
void bc_svd_dtor(bc_svd *self);

void bc_svd_reset_correspondences(bc_svd *self);
void bc_svd_add_correspondence(bc_svd *self, size_t idx, double u, double v);

double bc_svd_compute_pose(bc_svd *self, double R[3][3], double t[3]);
void relative_error(double *rot_err, double *transl_err, const double Rtrue[3][3], const double ttrue[3],
					const double Rest[3][3], const double test[3]);
void bc_svd_print_pose(bc_svd *self, const double R[3][3], const double t[3]);
void print_pose(const double R[3][3], const double t[3]);
