#include "survive_reproject.full.generated.h"
#include <libsurvive/survive.h>
#include <libsurvive/survive_reproject.h>
#include <math.h>
#include <os_generic.h>

typedef struct survive_calibration_config {
	FLT phase_scale, tilt_scale, curve_scale, gib_scale;
} survive_calibration_config;

static const survive_calibration_config default_config = {
	.phase_scale = 1., .tilt_scale = 1. / 10., .curve_scale = 1. / 10., .gib_scale = -1. / 10.};

void gen_survive_reproject_full(FLT *out, const SurvivePose *obj_pose, const LinmathVec3d obj_pt,
								const SurvivePose *lh2world, const BaseStationCal *bcal) {
	FLT phase_scale = default_config.phase_scale;
	FLT phase_0 = bcal[0].phase;
	FLT phase_1 = bcal[1].phase;

	FLT tilt_scale = default_config.tilt_scale;
	FLT tilt_0 = bcal[0].tilt;
	FLT tilt_1 = bcal[1].tilt;

	FLT curve_scale = default_config.curve_scale;
	FLT curve_0 = bcal[0].curve;
	FLT curve_1 = bcal[1].curve;

	FLT gib_scale = default_config.gib_scale;
	FLT gibPhase_0 = bcal[0].gibpha;
	FLT gibPhase_1 = bcal[1].gibpha;
	FLT gibMag_0 = bcal[0].gibmag;
	FLT gibMag_1 = bcal[1].gibmag;

	gen_reproject(out, obj_pose->Pos, obj_pt, lh2world->Pos, phase_scale, phase_0, phase_1, tilt_scale, tilt_0, tilt_1,
				  curve_scale, curve_0, curve_1, gib_scale, gibPhase_0, gibPhase_1, gibMag_0, gibMag_1);
}

double next_rand(double mx) { return (float)rand() / (float)(RAND_MAX / mx) - mx / 2.; }

SurvivePose random_pose() {
  const LinmathEulerAngle euler = { next_rand(2 * M_PI), next_rand(2 * M_PI), next_rand(2 * M_PI) };
  SurvivePose rtn = {.Pos = {next_rand(10), next_rand(10), next_rand(10)} };
  quatfromeuler(rtn.Rot, euler);
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

void check_rotate_vector() {
	SurvivePose obj = random_pose();
	FLT pt[3];
	random_point(pt);

	int cycles = 1000000000;
	FLT gen_out[3], out[3];
	double start, stop;
	start = OGGetAbsoluteTime();
	for (int i = 0; i < cycles; i++) {
		gen_quat_rotate_vector(gen_out, obj.Rot, pt);
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
	gen_invert_pose(gen_inv.Pos, obj.Pos);
	InvertPose(&inv, &obj);

	print_pose(&gen_inv);
	print_pose(&inv);
}

void check_reproject() {
	printf("Checking reprojection...\n");
	SurvivePose obj = random_pose();
	LinmathVec3d pt;
	random_point(pt);
	SurvivePose lh2world = random_pose();

	BaseStationData bsd;
	for (int i = 0; i < 10; i++)
		*((FLT *)&bsd.fcal[0].phase + i) = next_rand(1);

	FLT out_pt[2] = {0};
	int cycles = 1000000;

	double start_gen = OGGetAbsoluteTime();
	for (int i = 0; i < cycles; i++) {
		gen_survive_reproject_full(out_pt, &obj, pt, &lh2world, bsd.fcal);
	}
	double stop_gen = OGGetAbsoluteTime();
	printf("gen: %f %f (%f)\n", out_pt[0], out_pt[1], stop_gen - start_gen);

	double start_reproject = OGGetAbsoluteTime();
	for (int i = 0; i < cycles; i++)
		survive_reproject_full(bsd.fcal, &lh2world, &obj, pt, out_pt);
	double stop_reproject = OGGetAbsoluteTime();

	printf("%f %f (%f)\n", out_pt[0], out_pt[1], stop_reproject - start_reproject);
	out_pt[0] = out_pt[1] = 0;
}

void check_jacobian() {
  SurvivePose obj = random_pose();
  //SurvivePose obj = {};
  //obj.Rot[0] = 1.;
  //obj.Rot[1] = obj.Rot[2] = obj.Rot[3] = 0.0;
  
	LinmathVec3d pt;
	random_point(pt);

	SurvivePose lh2world = random_pose();
	//SurvivePose lh = {}; lh.Rot[0] = 1.;

	survive_calibration_config config;
	BaseStationData bsd = {};
	//for (int i = 0; i < 10; i++)
	//*((FLT *)&bsd.fcal[0].phase + i) = next_rand(0.5);

	FLT out_jac[14] = {0};
	survive_reproject_full_jac_obj_pose(out_jac, &obj, pt, &lh2world, bsd.fcal);

	FLT comp_jac[14] = {0};
	FLT out_pt[2] = {0};

	survive_reproject_full(bsd.fcal, &lh2world, &obj, pt, out_pt);
	for(int i = 0;i < 7;i++) {
	  FLT out[2] = {};
	  SurvivePose p = obj;

	  double H = 9e-9;
	  if(i < 3)
	    p.Pos[i] += H;
	  else {
	    H = 2.5e-10;
	    p.Rot[i - 3] += H;
	  }

	  //quatnormalize(p.Rot, p.Rot);

	  survive_reproject_full(bsd.fcal, &lh2world, &p, pt, out);
	  comp_jac[i] = (out[0] - out_pt[0]) / H;
	  comp_jac[i + 7] = (out[1] - out_pt[1]) / H;
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
	for(int j = 0;j < 2;j++) {
	  for(int i = 0;i < 7;i++) {
		  printf("%+.08f ", comp_jac[i + j * 7] - out_jac[i + j * 7]);
	  }
	  printf("\n");
	}
	
	out_pt[0] = out_pt[1] = 0;  
}

void check_apply_pose() {
	SurvivePose obj = random_pose();
	LinmathVec3d pt, out, gen_out;
	random_point(pt);

	gen_apply_pose(out, obj.Pos, pt);
	ApplyPoseToPoint(gen_out, &obj, pt);

	print_point(out);
	print_point(gen_out);	
}

int main() {
  check_apply_pose();
	check_jacobian();
	check_rotate_vector();
	check_invert();
	check_reproject();
	
	return 0;
}
