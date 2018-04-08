#include "survive_reproject.full.generated.h"
#include <libsurvive/survive.h>
#include <libsurvive/survive_reproject.h>
#include <math.h>
#include <os_generic.h>


void gen_survive_reproject_full(FLT *out, const SurvivePose *obj_pose, const LinmathVec3d obj_pt,
								const SurvivePose *lh2world, const BaseStationData *bsd,
								const survive_calibration_config *config) {
	FLT phase_scale = config->use_flag & SVCal_Phase ? config->phase_scale : 0.;
	FLT phase_0 = bsd->fcal.phase[0];
	FLT phase_1 = bsd->fcal.phase[1];

	FLT tilt_scale = config->use_flag & SVCal_Tilt ? config->tilt_scale : 0.;
	FLT tilt_0 = bsd->fcal.tilt[0];
	FLT tilt_1 = bsd->fcal.tilt[1];

	FLT curve_scale = config->use_flag & SVCal_Curve ? config->curve_scale : 0.;
	FLT curve_0 = bsd->fcal.curve[0];
	FLT curve_1 = bsd->fcal.curve[1];

	FLT gib_scale = config->use_flag & SVCal_Gib ? config->gib_scale : 0;
	FLT gibPhase_0 = bsd->fcal.gibpha[0];
	FLT gibPhase_1 = bsd->fcal.gibpha[1];
	FLT gibMag_0 = bsd->fcal.gibmag[0];
	FLT gibMag_1 = bsd->fcal.gibmag[1];

	gen_reproject(out, obj_pose->Pos, obj_pt, lh2world->Pos, phase_scale, phase_0, phase_1, tilt_scale, tilt_0, tilt_1,
				  curve_scale, curve_0, curve_1, gib_scale, gibPhase_0, gibPhase_1, gibMag_0, gibMag_1);
}

double next_rand(double mx) { return (float)rand() / (float)(RAND_MAX / mx) - mx / 2.; }

SurvivePose random_pose() {
	SurvivePose rtn = {.Pos = {next_rand(10), next_rand(10), next_rand(10)},
					   .Rot = {next_rand(1), next_rand(1), next_rand(1), next_rand(1)}};

	quatnormalize(rtn.Rot, rtn.Rot);
	return rtn;
}

void random_point(FLT *out) {
	out[0] = next_rand(10);
	out[1] = next_rand(10);
	out[2] = next_rand(10);
}

void print_pose(const SurvivePose *pose) {
	printf("[%f %f %f] [%f %f %f %f]\n", pose->Pos[0], pose->Pos[1], pose->Pos[2], pose->Rot[0], pose->Rot[1],
		   pose->Rot[2], pose->Rot[3]);
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
	SurvivePose obj = random_pose();
	LinmathVec3d pt;
	random_point(pt);
	SurvivePose lh = random_pose();

	survive_calibration_config config;
	BaseStationData bsd;
	for (int i = 0; i < 10; i++)
		*((FLT *)&bsd.fcal.phase[0] + i) = next_rand(1);

	for (int i = 0; i < 4; i++)
		*((FLT *)&config.phase_scale + i) = next_rand(1);

	config.use_flag = (enum SurviveCalFlag)0xff;
	FLT out_pt[2] = {0};
	int cycles = 10000000;

	double start_gen = OGGetAbsoluteTime();
	for (int i = 0; i < cycles; i++) {
		gen_survive_reproject_full(out_pt, &obj, pt, &lh, &bsd, &config);
	}
	double stop_gen = OGGetAbsoluteTime();
	printf("gen: %f %f (%f)\n", out_pt[0], out_pt[1], stop_gen - start_gen);

	double start_reproject = OGGetAbsoluteTime();
	for (int i = 0; i < cycles; i++)
		survive_reproject_full(out_pt, &obj, pt, &lh, &bsd, &config);
	double stop_reproject = OGGetAbsoluteTime();

	printf("%f %f (%f)\n", out_pt[0], out_pt[1], stop_reproject - start_reproject);
	out_pt[0] = out_pt[1] = 0;
}

int main() {
	check_rotate_vector();
	check_invert();
	check_reproject();

	return 0;
}
