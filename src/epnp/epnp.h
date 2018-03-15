// Copyright (c) 2009, V. Lepetit, EPFL
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

// The views and conclusions contained in the software and documentation are those
// of the authors and should not be interpreted as representing official policies,
//   either expressed or implied, of the FreeBSD Project.

#ifndef epnp_h
#define epnp_h

#ifndef WITH_OPENCV
#include "opencv_shim.h"
#else
#include <opencv/cv.h>
#endif

typedef struct {

	double uc, vc, fu, fv;

	double *pws, *us, *alphas, *pcs;
	int maximum_number_of_correspondences;
	int number_of_correspondences;

	double cws[4][3], ccs[4][3];
	double cws_determinant;
} epnp;

void epnp_dtor(epnp *self);
void epnp_set_internal_parameters(epnp *self, double uc, double vc, double fu, double fv);
void epnp_set_maximum_number_of_correspondences(epnp *self, int n);
void epnp_reset_correspondences(epnp *self);
void epnp_add_correspondence(epnp *self, double X, double Y, double Z, double u, double v);
double epnp_compute_pose(epnp *self, double R[3][3], double t[3]);
void relative_error(double *rot_err, double *transl_err, const double Rtrue[3][3], const double ttrue[3],
					const double Rest[3][3], const double test[3]);
void epnp_print_pose(epnp *self, const double R[3][3], const double t[3]);
double epnp_reprojection_error(epnp *self, const double R[3][3], const double t[3]);
void print_pose(const double R[3][3], const double t[3]);

#endif
