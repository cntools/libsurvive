#include "string.h"
#include <libsurvive/survive.h>
#include <libsurvive/survive_reproject.h>
#include <libsurvive/survive_reproject_gen2.h>
#include <math.h>
#include <os_generic.h>

#include "minimal_opencv.h"

#include "test_case.h"

#include "../generated/survive_imu.generated.h"
#include "../generated/survive_reproject.generated.h"

#ifdef HAVE_AUX_GENERATED
#include "../generated/survive_reproject.aux.generated.h"
#endif

static void rot_predict_quat(FLT t, const void *k, const CvMat *f_in, CvMat *f_out) {
	(void)k;

	const FLT *rot = CV_FLT_PTR(f_in);
	const FLT *vel = CV_FLT_PTR(f_in) + 4;
	copy3d(CV_FLT_PTR(f_out) + 4, vel);

	survive_apply_ang_velocity(CV_FLT_PTR(f_out), vel, t, rot);
}

typedef struct survive_calibration_config {
	FLT phase_scale, tilt_scale, curve_scale, gib_scale;
} survive_calibration_config;

static const survive_calibration_config default_config = {
	.phase_scale = 1., .tilt_scale = 1. / 10., .curve_scale = 1. / 10., .gib_scale = -1. / 10.};

double next_rand(double mx) { return (float)rand() / (float)(RAND_MAX / mx) - mx / 2.; }

void random_quat(LinmathQuat rtn) {
	const LinmathEulerAngle euler = {next_rand(2 * M_PI), next_rand(2 * M_PI), next_rand(2 * M_PI)};
	quatfromeuler(rtn, euler);
}

void random_axis_angle(LinmathAxisAngle a) {
	for (int i = 0; i < 3; i++) {
		a[i] = next_rand(2 * M_PI) - M_PI;
	}
}

SurvivePose random_pose() {
	const LinmathEulerAngle euler = {next_rand(2 * M_PI), next_rand(2 * M_PI), next_rand(2 * M_PI)};
	SurvivePose rtn = {.Pos = {next_rand(10), next_rand(10), next_rand(10)}};
	quatfromeuler(rtn.Rot, euler);
	return rtn;
}

LinmathAxisAnglePose random_pose_axisangle() {
	LinmathAxisAnglePose rtn = {.Pos = {next_rand(10), next_rand(10), next_rand(10)},
								.AxisAngleRot = {next_rand(2 * M_PI), next_rand(2 * M_PI), next_rand(2 * M_PI)}};
	return rtn;
}

void random_point(FLT *out) {
	out[0] = next_rand(1);
	out[1] = next_rand(1);
	out[2] = next_rand(1);
}

void print_pose(const SurvivePose *pose) {
	printf("[%f %f %f] [%f %f %f %f]\n", pose->Pos[0], pose->Pos[1], pose->Pos[2], pose->Rot[0], pose->Rot[1],
		   pose->Rot[2], pose->Rot[3]);
}

void print_point(const FLT *Pos) {
	printf("[%f %f %f]\n", Pos[0], Pos[1], Pos[2]);
}

#ifdef HAVE_AUX_GENERATED
void check_rotate_vector() {
	SurvivePose obj = random_pose();
	FLT pt[3];
	random_point(pt);

	int cycles = 1000;
	FLT gen_out[3], out[3];
	double start, stop;
	start = OGGetAbsoluteTime();
	for (int i = 0; i < cycles; i++) {
		gen_quatrotatevector(gen_out, obj.Rot, pt);
	}
	stop = OGGetAbsoluteTime();
	printf("gen: %f %f %f (%f)\n", gen_out[0], gen_out[1], gen_out[2], stop - start);

	start = OGGetAbsoluteTime();
	for (int i = 0; i < cycles; i++) {
		quatrotatevector(out, obj.Rot, pt);
	}
	stop = OGGetAbsoluteTime();

	printf("%f %f %f (%f)\n", out[0], out[1], out[2], stop - start);
}

void check_invert() {
	SurvivePose obj = random_pose();
	SurvivePose gen_inv, inv;
	gen_invert_pose(gen_inv.Pos, &obj);
	InvertPose(&inv, &obj);

	print_pose(&gen_inv);
	print_pose(&inv);
}
#endif
TEST(Generated, Reproject) {
	SurvivePose obj = random_pose();
	LinmathVec3d pt;
	random_point(pt);
	SurvivePose world2lh = random_pose();

	BaseStationData bsd = { 0 };
	// for (int i = 0; i < 10; i++)
	//*((FLT *)&bsd.fcal[0].phase + i) = next_rand(1);

	FLT out_pt[3] = {0}, out_pt_gen[3] = { 0 };
	int cycles = 50000000;

	double start_gen = OGGetAbsoluteTime();
	for (int i = 0; i < cycles; i++) {
		obj.Pos[0] += .001;

		gen_reproject(out_pt_gen, &obj, pt, &world2lh, bsd.fcal);
	}
	double stop_gen = OGGetAbsoluteTime();
	printf("Generated: %f %f (%f)\n", out_pt_gen[0], out_pt_gen[1], stop_gen - start_gen);

	double start_reproject = OGGetAbsoluteTime();
	for (int i = 0; i < cycles; i++) {
		survive_reproject_full(bsd.fcal, &world2lh, &obj, pt, out_pt);
	}
	double stop_reproject = OGGetAbsoluteTime();

	printf("Non-gen:   %f %f (%f)\n", out_pt[0], out_pt[1], stop_reproject - start_reproject);
	return dist3d(out_pt_gen, out_pt) < 1e-5 ? 0 : -1;
}

TEST(Generated, reproject_gen2) {
	SurvivePose obj = random_pose();
	LinmathVec3d pt;
	random_point(pt);
	SurvivePose world2lh = random_pose();

	BaseStationData bsd = { 0 };
	// for (int i = 0; i < 10; i++)
	//*((FLT *)&bsd.fcal[0].phase + i) = next_rand(1);

	FLT out_pt[3] = {0}, out_pt_gen[3] = {0};
	int cycles = 10000000;

	double start_gen = OGGetAbsoluteTime();
	for (int i = 0; i < cycles; i++) {
		gen_reproject_gen2(out_pt_gen, &obj, pt, &world2lh, bsd.fcal);
	}
	double stop_gen = OGGetAbsoluteTime();
	printf("generated    : %f %f (%f)\n", out_pt_gen[0], out_pt_gen[1], stop_gen - start_gen);

	double start_reproject = OGGetAbsoluteTime();
	for (int i = 0; i < cycles; i++) {
		survive_reproject_full_gen2(bsd.fcal, &world2lh, &obj, pt, out_pt);
	}
	double stop_reproject = OGGetAbsoluteTime();

	printf("non-generated: %f %f (%f)\n", out_pt[0], out_pt[1], stop_reproject - start_reproject);
	return dist3d(out_pt_gen, out_pt) < 1e-5 ? 0 : -1;
}

TEST(Generated, reproject_gen2_vals) {
	BaseStationData bsd = { 0 };
	double cal[] = {-0.047119140625, 0, 0.15478515625, 2.369140625, -0.00440216064453125, 0.4765625, -0.1766357421875};

	bsd.fcal[0].phase = 0;
	bsd.fcal[0].tilt = -0.047119140625;
	bsd.fcal[0].curve = 0.15478515625;
	bsd.fcal[0].gibpha = 2.369140625;
	bsd.fcal[0].gibmag = -0.00440216064453125;
	bsd.fcal[0].ogeephase = 0.4765625;
	bsd.fcal[0].ogeemag = -0.1766357421875;

	LinmathPoint3d xyz = {0.37831748940152643, -0.29826620924843278, -3.0530035758130878};
	double ang = survive_reproject_axis_x_gen2(&bsd.fcal[0], xyz);
	ang += M_PI * 2. * (0 + 1.) / 3.;
	printf("%.16f\n", ang);
	return fabs(ang - 2.024090911337) < 1e-5 ? 0 : -1;
}


#ifdef HAVE_AUX_GENERATED
void check_apply_ang_velocity() {
	LinmathQuat qo, qi;
	random_quat(qi);
	LinmathAxisAngle v;
	random_axis_angle(v);
	FLT t = next_rand(5);
	survive_apply_ang_velocity(qo, v, t, qi);

	LinmathQuat qo2;
	gen_apply_ang_velocity(qo2, v, t, qi);

	printf("Lib: " Quat_format "\n", LINMATH_QUAT_EXPAND(qo));
	printf("Gen: " Quat_format "\n", LINMATH_QUAT_EXPAND(qo2));
}
#endif

extern void rot_predict_quat(FLT t, const void *k, const CvMat *f_in, CvMat *f_out);

TEST(Generated, rot_predict_quat) {
	FLT _mi[7] = { 0 };
	FLT _mo1[7] = { 0 };
	FLT _mo2[7] = { 0 };
	CvMat mi = cvMat(7, 1, CV_64F, _mi);
	CvMat mo1 = cvMat(7, 1, CV_64F, _mo1);
	CvMat mo2 = cvMat(7, 1, CV_64F, _mo2);

	FLT t = next_rand(5);

	random_quat(_mi);
	random_axis_angle(_mi + 4);

	rot_predict_quat(t, 0, &mi, &mo1);

	gen_imu_rot_f(_mo2, t, _mi);

	printf("Lib: " SurvivePose_format "\n", LINMATH_QUAT_EXPAND(_mo1), LINMATH_VEC3_EXPAND(_mo1 + 4));
	printf("Gen: " SurvivePose_format "\t"
		   "\n",
		   LINMATH_QUAT_EXPAND(_mo2), LINMATH_VEC3_EXPAND(_mo2 + 4));

	FLT err = 0;
	for(int i = 0;i < 7;i++)
		err += fabs((_mo1[i] - _mo2[i]) * (_mo1[i] - _mo2[i]));
	return err > 1e-5 ? -1 : 0;
}

/*
void check_jacobian_axisangle() {
	LinmathAxisAnglePose obj2world = random_pose_axisangle();
	// SurvivePose obj2world = { 0 };
	// memset(obj2world.Rot, 0, sizeof(FLT) * 3);

	LinmathVec3d pt;
	random_point(pt);

	LinmathAxisAnglePose world2lh = random_pose_axisangle();
	// memset(world2lh.Rot, 0, sizeof(FLT) * 4);
	// world2lh.Rot[1] = 1.;
	// SurvivePose lh = { 0 }; lh.Rot[0] = 1.;

	survive_calibration_config config;
	BaseStationData bsd = { 0 };
	for (int i = 0; i < 10; i++)
		*((FLT *)&bsd.fcal[0].phase + i) = next_rand(0.5);

	FLT out_jac[14] = {0};
	survive_reproject_full_jac_obj_pose_axisangle(out_jac, &obj2world, pt, &world2lh, bsd.fcal);

	FLT comp_jac[14] = {0};
	FLT out_pt[2] = {0};

	for (int i = 0; i < 6; i++) {
		FLT out[2] = { 0 };

		double H = 1e-10;
		for (int n = 0; n < 2; n++) {
			LinmathAxisAnglePose p = obj2world;
			int s = n == 0 ? 1 : -1;
			if (i < 3)
				p.Pos[i] += s * H;
			else {
				H = 2.5e-10;
				p.Rot[i - 3] += s * H;
			}

			// quatnormalize(p.Rot, p.Rot);

			SurvivePose world2lhq = AxisAnglePose2Pose(&world2lh);
			SurvivePose pq = AxisAnglePose2Pose(&p);
			survive_reproject_full(bsd.fcal, &world2lhq, &pq, pt, n == 0 ? out : out_pt);
		}

		comp_jac[i] = (out[0] - out_pt[0]) / (2. * H);
		comp_jac[i + 6] = (out[1] - out_pt[1]) / (2. * H);
	}

	for (int j = 0; j < 2; j++) {
		for (int i = 0; i < 6; i++) {
			printf("%+.08f ", out_jac[i + j * 6]);
		}
		printf("\n");
	}
	printf("\n");
	for (int j = 0; j < 2; j++) {
		for (int i = 0; i < 6; i++) {
			printf("%+.08f ", comp_jac[i + j * 6]);
		}
		printf("\n");
	}
	printf("\n");
	for (int j = 0; j < 2; j++) {
		for (int i = 0; i < 6; i++) {
			printf("%+.08f ", comp_jac[i + j * 6] - out_jac[i + j * 6]);
		}
		printf("\n");
	}

	out_pt[0] = out_pt[1] = 0;
}
*/

TEST(Generated, Speed) {
	SurvivePose obj2world = random_pose();

	memset(obj2world.Rot, 0, sizeof(FLT) * 4);
	obj2world.Rot[1] = 1.;

	LinmathVec3d pt;
	random_point(pt);

	SurvivePose world2lh = random_pose();
	// memset(world2lh.Rot, 0, sizeof(FLT) * 4);
	// world2lh.Rot[1] = 1.;
	// SurvivePose lh = { 0 }; lh.Rot[0] = 1.;

	survive_calibration_config config;
	BaseStationData bsd = { 0 };
	for (int i = 0; i < 10; i++)
		*((FLT *)&bsd.fcal[0].phase + i) = next_rand(0.5);

	FLT out_jac[14] = {0};

	for (int i = 0; i < 20000000; i++) {
		// survive_reproject_full_jac_obj_pose
		gen_reproject_jac_obj_p(out_jac, &obj2world, pt, &world2lh, bsd.fcal);
	}

	return 0;
}

TEST(Generated, jacobian) {
	SurvivePose obj2world = random_pose();
	// SurvivePose obj2world = { 0 };
	memset(obj2world.Rot, 0, sizeof(FLT) * 4);
	obj2world.Rot[1] = 1.;

	LinmathVec3d pt;
	random_point(pt);

	SurvivePose world2lh = random_pose();
	// memset(world2lh.Rot, 0, sizeof(FLT) * 4);
	// world2lh.Rot[1] = 1.;
	// SurvivePose lh = { 0 }; lh.Rot[0] = 1.;

	survive_calibration_config config;
	BaseStationData bsd = { 0 };
	for (int i = 0; i < 10; i++)
		*((FLT *)&bsd.fcal[0].phase + i) = next_rand(0.5);

	FLT out_jac[14] = {0};
	// survive_reproject_full_jac_obj_pose(out_jac, &obj2world, pt, &world2lh, bsd.fcal);
	gen_reproject_jac_obj_p(out_jac, &obj2world, pt, &world2lh, bsd.fcal);
	// survive_reproject_full_jac_obj_pose(out_jac, &obj2world, pt, &world2lh, bsd.fcal);

	FLT comp_jac[14] = {0};
	FLT out_pt[2] = {0};

	for(int i = 0;i < 7;i++) {
	  FLT out[2] = { 0 };

	  double H = 1e-10;
	  for (int n = 0; n < 2; n++) {
		  SurvivePose p = obj2world;
		  int s = n == 0 ? 1 : -1;
		  if (i < 3)
			  p.Pos[i] += s * H;
		  else {
			  H = 2.5e-10;
			  p.Rot[i - 3] += s * H;
		  }

		  // quatnormalize(p.Rot, p.Rot);

		  survive_reproject_full(bsd.fcal, &world2lh, &p, pt, n == 0 ? out : out_pt);
	  }

	  comp_jac[i] = (out[0] - out_pt[0]) / (2. * H);
	  comp_jac[i + 7] = (out[1] - out_pt[1]) / (2. * H);
	}


	for(int j = 0;j < 2;j++) {
	  for(int i = 0;i < 7;i++) {
		  printf("%+.08f ", out_jac[i + j * 7]);
	  }
	  printf("\n");
	}
	printf("\n");
	for(int j = 0;j < 2;j++) {
	  for(int i = 0;i < 7;i++) {
		  printf("%+.08f ", comp_jac[i + j * 7]);
	  }
	  printf("\n");
	}
	printf("\n");
	bool fail = false;
	for(int j = 0;j < 2;j++) {
	  for(int i = 0;i < 7;i++) {
		  printf("%+.08f ", comp_jac[i + j * 7] - out_jac[i + j * 7]);
		  fail |= fabs(comp_jac[i + j * 7] - out_jac[i + j * 7]) > 1e-1;
	  }
	  printf("\n");
	}
	
	out_pt[0] = out_pt[1] = 0;
	return fail ? -1 : 0;
}

TEST(Generated, jacobian_gen2) {
	SurvivePose obj2world = random_pose();
	// SurvivePose obj2world = { 0 };
	memset(obj2world.Rot, 0, sizeof(FLT) * 4);
	obj2world.Rot[1] = 1.;

	LinmathVec3d pt;
	random_point(pt);

	SurvivePose world2lh = random_pose();
	// memset(world2lh.Rot, 0, sizeof(FLT) * 4);
	// world2lh.Rot[1] = 1.;
	// SurvivePose lh = { 0 }; lh.Rot[0] = 1.;

	survive_calibration_config config;
	BaseStationData bsd = { 0 };
	for (int i = 0; i < 10; i++)
		*((FLT *)&bsd.fcal[0].phase + i) = next_rand(0.5);

	FLT out_jac[14] = {0};
	// survive_reproject_full_jac_obj_pose_gen2(out_jac, &obj2world, pt, &world2lh, bsd.fcal);
	gen_reproject_gen2_jac_obj_p(out_jac, &obj2world, pt, &world2lh, bsd.fcal);

	FLT comp_jac[14] = {0};
	FLT out_pt[2] = {0};

	for (int i = 0; i < 7; i++) {
		FLT out[2] = { 0 };

		double H = 1e-10;
		for (int n = 0; n < 2; n++) {
			SurvivePose p = obj2world;
			int s = n == 0 ? 1 : -1;
			if (i < 3)
				p.Pos[i] += s * H;
			else {
				H = 2.5e-10;
				p.Rot[i - 3] += s * H;
			}

			// quatnormalize(p.Rot, p.Rot);
			gen_reproject_gen2(n == 0 ? out : out_pt, &p, pt, &world2lh, bsd.fcal);
			// survive_reproject_full_gen2(bsd.fcal, &world2lh, &p, pt, n == 0 ? out : out_pt);
		}

		comp_jac[i] = (out[0] - out_pt[0]) / (2. * H);
		comp_jac[i + 7] = (out[1] - out_pt[1]) / (2. * H);
	}

	for (int j = 0; j < 2; j++) {
		for (int i = 0; i < 7; i++) {
			printf("%+.08f ", out_jac[i + j * 7]);
		}
		printf("\n");
	}
	printf("\n");
	for (int j = 0; j < 2; j++) {
		for (int i = 0; i < 7; i++) {
			printf("%+.08f ", comp_jac[i + j * 7]);
		}
		printf("\n");
	}
	printf("\n");
	bool fail = false;
	for (int j = 0; j < 2; j++) {
		for (int i = 0; i < 7; i++) {
			printf("%+.08f ", comp_jac[i + j * 7] - out_jac[i + j * 7]);
			fail |= fabs(comp_jac[i + j * 7] - out_jac[i + j * 7]) > 1e-1;
		}
		printf("\n");
	}

	return fail ? 0 : -1;
}

#ifdef HAVE_AUX_GENERATED
void check_apply_pose() {
	SurvivePose obj = random_pose();
	LinmathVec3d pt, out, gen_out;
	random_point(pt);

	gen_apply_pose_to_pt(out, &obj, pt);
	ApplyPoseToPoint(gen_out, &obj, pt);

	print_point(out);
	print_point(gen_out);	
}
#endif
