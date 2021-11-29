#include "string.h"
#include <fenv.h>
#include <libsurvive/survive.h>
#include <libsurvive/survive_reproject.h>
#include <libsurvive/survive_reproject_gen2.h>
#include <math.h>
#include <os_generic.h>

#include "sv_matrix.h"

#include "test_case.h"

#include "../generated/survive_imu.generated.h"
#include "../generated/survive_reproject.generated.h"

//#ifdef HAVE_AUX_GENERATED
#include "../generated/survive_reproject.aux.generated.h"
//#endif

#ifndef M_PI
#define M_PI LINMATHPI
#endif

#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include "malloc.h"
#endif

#define STACK_ALLOC(nmembers) (FLT *)alloca(nmembers * sizeof(FLT))

static void rot_predict_quat(FLT t, const void *k, const SvMat *f_in, SvMat *f_out) {
	(void)k;

	const FLT *rot = SV_FLT_PTR(f_in);
	const FLT *vel = SV_FLT_PTR(f_in) + 4;
	copy3d(SV_FLT_PTR(f_out) + 4, vel);

	survive_apply_ang_velocity(SV_FLT_PTR(f_out), vel, t, rot);
}

typedef struct survive_calibration_config {
	FLT phase_scale, tilt_scale, curve_scale, gib_scale;
} survive_calibration_config;

static const survive_calibration_config default_config = {
	.phase_scale = 1., .tilt_scale = 1. / 10., .curve_scale = 1. / 10., .gib_scale = -1. / 10.};

static double next_rand(double mx) { return (float)rand() / (float)(RAND_MAX / mx) - mx / 2.; }

static size_t random_quat(LinmathQuat rtn) {
	if (rtn) {
		const LinmathEulerAngle euler = {next_rand(2 * M_PI), next_rand(2 * M_PI), next_rand(2 * M_PI)};
		quatfromeuler(rtn, euler);
		// quatcopy(rtn, LinmathQuat_Identity);
	}
	return sizeof(FLT) * 7;
}

static void random_axis_angle(LinmathAxisAngle a) {
	for (int i = 0; i < 3; i++) {
		a[i] = next_rand(2 * M_PI) - M_PI;
	}
}

typedef size_t (*generate_input)(FLT *output);
typedef void (*general_fn)(FLT *output, const FLT *input);

static void print_array(const char *label, const FLT *a, size_t t, size_t columns) {
	if (label)
		TEST_PRINTF("%32s: \t", label);
	for (int i = 0; i < t; i++) {
		if ((fabs(a[i]) > .000001 && fabs(a[i]) < 1e4))
			TEST_PRINTF("%+6.6lf"
						"\t",
						(double)a[i]);
		else if (isnan(a[i])) {
			TEST_PRINTF("%6snan\t", "");
		} else if (a[i] == 0) {
			TEST_PRINTF("        0\t");
		} else {
			TEST_PRINTF("%+2.3le\t", (double)a[i]);
		}
		if (columns != 0 && i % columns == (columns - 1)) {
			TEST_PRINTF("\n%32s  \t", "");
		}
	}
	TEST_PRINTF("\n");
}
static FLT diff_array(FLT *out, const FLT *a, const FLT *b, size_t len) {
	FLT rtn = 0;
	for (int i = 0; i < len; i++) {
		rtn += (a[i] - b[i]) * (a[i] - b[i]);
		if (out)
			out[i] = fabs(a[i] - b[i]);
	}
	return sqrt(rtn) / (FLT)len;
}
static FLT print_diff_array(const char *label, const FLT *a, const FLT *b, size_t len, size_t columns) {
	FLT *array = STACK_ALLOC(len);
	FLT rtn = diff_array(array, a, b, len);
	print_array(label, array, len, columns);
	return rtn;
}

static FLT test_gen_jacobian_function(const char *name, generate_input input_fn, general_fn nongen, general_fn gen,
									  general_fn generated_jacobian, size_t outputs, size_t jac_start_idx,
									  size_t jac_length) {
	size_t inputs = input_fn(0) / sizeof(FLT);
	FLT *output_gen = STACK_ALLOC(outputs * jac_length), *output = STACK_ALLOC(outputs * jac_length);
	bool failed = false;

	FLT *input = STACK_ALLOC(inputs);
	for (int n = 0; n < inputs; n++) {
		input[n] = NAN;
	}
	input_fn(input);

	for (int n = 0; n < outputs * jac_length; n++) {
		output_gen[n] = NAN;
	}
	generated_jacobian(output_gen, input);

	FLT *out = STACK_ALLOC(outputs);
	FLT *out_pt = STACK_ALLOC(outputs);
	FLT *input_copy = STACK_ALLOC(inputs);
	FLT *gen_output = STACK_ALLOC(outputs);

	const int M = 10;
	// This thing is to maintain compatibility with VS C compiler -- it chokes on FLT D[outputs][M][M];
	FLT ***D = alloca(outputs * sizeof(FLT **));
	for (int i = 0; i < outputs; i++) {
		D[i] = alloca(M * sizeof(FLT *));
		for (int j = 0; j < M; j++) {
			D[i][j] = alloca(M * sizeof(FLT));
		}
	}

	for (int i = 0; i < jac_length; i++) {
		for (int d = 0; d < M; d++) {
			for (int m = 0; m < M; m++) {
				for (int n = 0; n < outputs; n++) {
					D[n][m][d] = NAN;
				}
			}
		}
		for (int d = 0; d < M; d++) {
			FLT H = 2;
			for (int m = d; m < M; m++) {

				if (d == 0) {
					for (int n = 0; n < 2; n++) {
						memcpy(input_copy, input, sizeof(FLT) * inputs);

						int s = n == 0 ? 1 : -1;
						input_copy[jac_start_idx + i] += s * H;
						// print_array("Input", input_copy, inputs, 0);
						if (nongen) {
							nongen(n == 0 ? out : out_pt, input_copy);
							gen(gen_output, input_copy);

							FLT err = diff_array(0, gen_output, n == 0 ? out : out_pt, outputs);
							if (err > 1e-5) {
								failed = true;
								TEST_PRINTF("Gen/nongen mismatch\n");
							}
						} else {
							gen(n == 0 ? out : out_pt, input_copy);
						}
						// print_array("Output", n == 0 ? out : out_pt, outputs, 0);
					}

					for (int n = 0; n < outputs; n++) {
						D[n][m][d] = (out[n] - out_pt[n]) / (2.) / H;
					}

					H = H / 2.;
				} else {
					for (int n = 0; n < outputs; n++) {
						D[n][m][d] = (pow(4., d) * D[n][m][d - 1] - D[n][m - 1][d - 1]) / (pow(4., d) - 1);
					}
				}
			}
		}

		for (int n = 0; n < outputs; n++) {
			// TEST_PRINTF("For input %d output %d: \n", i, n);
			output[i + n * jac_length] = D[n][M - 1][M - 1];
			// print_array("D", (double*)D[n], M*M, M);
		}
	}

	if (failed) {
		TEST_PRINTF("Testing generated jacobian %s\n", name);
		print_array("inputs", input, inputs, 0);

		print_array("gen jacobian outputs", output_gen, outputs * jac_length, jac_length);
		print_array("jacobian outputs", output, outputs * jac_length, jac_length);

		FLT err = print_diff_array("Differences", output, output_gen, outputs * jac_length, jac_length);
		TEST_PRINTF("SSE: %f\n", err);
		return err;
	}
	return 0;
}

typedef struct gen_function_jacobian_def {
	const char *suffix;
	general_fn jacobian;
	size_t jacobian_start_idx;
	size_t jacobian_length;
} gen_function_jacobian_def;
typedef struct gen_function_def {
	const char *name;
	general_fn generated;
	general_fn check;
	generate_input generate_inputs;
	size_t outputs;
	const struct gen_function_jacobian_def jacobians[16];
} gen_function_def;

static double run_cycles(general_fn runme, const FLT *inputs, FLT *outputs) {
	FLT runtime = 1;
	size_t cycles = 0;

	double start_gen = OGGetAbsoluteTime();
	double stop_gen = 0;
	do {
		runme(outputs, inputs);
		cycles++;
		stop_gen = OGGetAbsoluteTime();
	} while (start_gen + runtime > stop_gen);

	return cycles / (stop_gen - start_gen);
}

static FLT test_gen_function(const char *name, generate_input input_fn, general_fn nongen, general_fn generated,
							 size_t outputs) {
	if (!nongen) {
		printf("Testing generated %-32s \n", name);
		return 0;
	}

	FLT *output_gen = STACK_ALLOC(outputs), *output = STACK_ALLOC(outputs);

	size_t inputs = input_fn(0) / sizeof(FLT);

	FLT *input = STACK_ALLOC(inputs);
	input_fn(input);

	for (int i = 0; i < 1000; i++) {
		input_fn(input);
		generated(output_gen, input);
		nongen(output, input);

		FLT err = diff_array(0, output, output_gen, outputs);
		TEST_PRINTF("%s match\n", name);
		if (err > 1e-5) {
			TEST_PRINTF("%s eval mismatch: \n", name);
			print_array("inputs", input, inputs, 0);
			print_array("gen outputs", output, outputs, 0);
			print_array("outputs", output_gen, outputs, 0);

			FLT err = print_diff_array("Differences", output, output_gen, outputs, 0);
			TEST_PRINTF("Difference: %f\n", err);
		}
	}

	input_fn(input);
	FLT gen_hz = run_cycles(generated, input, output_gen);
	FLT hz = run_cycles(nongen, input, output);

	printf("Testing generated %-32s gen: %8.2fkz nongen: %8.2fkz\n", name, gen_hz / 1000., hz / 1000.);
	TEST_PRINTF("Testing generated %-32s gen: %8.2fkz nongen: %8.2fkz\n", name, gen_hz / 1000., hz / 1000.);
	print_array("inputs", input, inputs, 0);
	print_array("gen outputs", output, outputs, 0);
	print_array("outputs", output_gen, outputs, 0);

	FLT err = print_diff_array("Differences", output, output_gen, outputs, 0);
	TEST_PRINTF("Difference: %f\n", err);

	return err;
}

static int test_gen_function_def(const gen_function_def *def) {
	bool failed = false;
	FLT err = test_gen_function(def->name, def->generate_inputs, def->check, def->generated, def->outputs);
	failed |= err > 1e-5;
	for (int i = 0; i < 16 && def->jacobians[i].jacobian; i++) {
		const gen_function_jacobian_def *jdef = &def->jacobians[i];
		char name[64] = {0};
		strcat(name, def->name);
		strcat(name, "_");
		strcat(name, jdef->suffix);
		for (int j = 0; j < 100; j++) {
			failed |= test_gen_jacobian_function(name, def->generate_inputs, def->check, def->generated, jdef->jacobian,
												 def->outputs, jdef->jacobian_start_idx, jdef->jacobian_length) > 1e-5;
		}
	}
	return failed ? -1 : 0;
}

static size_t random_quat_quat(FLT *output) {
	if (output) {
		random_quat(output);
		random_quat(output + 4);
	}
	return sizeof(FLT) * 8;
}

static void general_gen_quatrotateabout(FLT *out, const FLT *input) { gen_quatrotateabout(out, input, input + 4); }
static void general_quatrotateabout(FLT *out, const FLT *input) { quatrotateabout(out, input, input + 4); }
static void general_gen_quatrotateabout_jac_q1(FLT *out, const FLT *input) {
	gen_quatrotateabout_jac_q1(out, input, input + 4);
}
static void general_gen_quatrotateabout_jac_q2(FLT *out, const FLT *input) {
	gen_quatrotateabout_jac_q2(out, input, input + 4);
}

gen_function_def quatrotateabout_def = {
	.name = "quatrotateabout",
	.generated = general_gen_quatrotateabout,
	.check = general_quatrotateabout,
	.generate_inputs = random_quat_quat,
	.outputs = 4,
	.jacobians = {{.suffix = "q1", .jacobian = general_gen_quatrotateabout_jac_q1, .jacobian_length = 4},
				  {.suffix = "q2",
				   .jacobian = general_gen_quatrotateabout_jac_q2,
				   .jacobian_length = 4,
				   .jacobian_start_idx = 4}}};

TEST(Generated, quatrotateabout) { return test_gen_function_def(&quatrotateabout_def); }

static void general_quatrotatevector(FLT *out, const FLT *input) { quatrotatevector(out, input, input + 4); }
static void general_gen_quatrotatevector(FLT *out, const FLT *input) { gen_quatrotatevector(out, input, input + 4); }
static void general_gen_quatrotatevector_jac_q(FLT *out, const FLT *input) {
	gen_quatrotatevector_jac_q(out, input, input + 4);
}
static void general_gen_quatrotatevector_jac_pt(FLT *out, const FLT *input) {
	gen_quatrotatevector_jac_pt(out, input, input + 4);
}

static size_t random_quat_vec3(FLT *output) {
	if (output) {
		random_quat(output);
		random_axis_angle(output + 4);
	}
	return sizeof(FLT) * 7;
}

gen_function_def quatrotatevector_def = {
	.name = "quatrotatevector",
	.generated = general_gen_quatrotatevector,
	.check = general_quatrotatevector,
	.generate_inputs = random_quat_vec3,
	.outputs = 3,
	.jacobians = {{.suffix = "q", .jacobian = general_gen_quatrotatevector_jac_q, .jacobian_length = 4},
				  {.suffix = "pt",
				   .jacobian = general_gen_quatrotatevector_jac_pt,
				   .jacobian_length = 3,
				   .jacobian_start_idx = 4}}};

TEST(Generated, quatrotatevector) { return test_gen_function_def(&quatrotatevector_def); }

SurvivePose random_pose() {
	const LinmathEulerAngle euler = {next_rand(2 * M_PI), next_rand(2 * M_PI), next_rand(2 * M_PI)};
	SurvivePose rtn = {.Pos = {next_rand(10), next_rand(10), next_rand(10)}};
	quatfromeuler(rtn.Rot, euler);
	return rtn;
}

void random_pose_into(FLT *out) {
	SurvivePose rtn = random_pose();
	memcpy(out, rtn.Pos, sizeof(SurvivePose));
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

void random_fcal(BaseStationCal *fcal) {
	fcal->curve = next_rand(0.5);
	fcal->gibmag = next_rand(0.5);
	fcal->gibpha = next_rand(0.5);
	fcal->ogeemag = next_rand(0.5);
	fcal->ogeephase = next_rand(0.5);
	fcal->phase = next_rand(0.5);
	fcal->tilt = next_rand(0.5);
}

size_t random_kalman_model(FLT *out) {
	if (out != 0) {
		SurviveKalmanModel m = {
			.Pose = random_pose(),
		};
		random_point(m.Acc);
		random_point(m.Velocity.Pos);
		random_point(m.Velocity.AxisAngleRot);
		memcpy(out, &m, sizeof(SurviveKalmanModel));
	}
	return sizeof(SurviveKalmanModel);
}

static void general_imu_predict(FLT *out, const FLT *_input) {
	SurviveKalmanModel *input = (SurviveKalmanModel *)_input;
	gen_imu_predict(out, input);
}

static void general_gen_imu_predict_jac_kalman_model(FLT *out, const FLT *_input) {
	SurviveKalmanModel *input = (SurviveKalmanModel *)_input;
	gen_imu_predict_jac_kalman_model(out, input);
}

static void imu_predict_gyro(FLT *out, SurviveKalmanModel *m) {
	/*
	 * def imu_predict_gyro(kalman_model):
		rot = quatgetreciprocal(quatnormalize(kalman_model.Pose.Rot))
		rotv = quatrotatevector(rot, kalman_model.Velocity.Rot)
		return [rotv[0] + kalman_model.GyroBias[0],
				rotv[1] + kalman_model.GyroBias[1],
				rotv[2] + kalman_model.GyroBias[2]
		]
	 */
	LinmathQuat w2o;
	quatgetreciprocal(w2o, m->Pose.Rot);
	quatrotatevector(out, w2o, m->Velocity.AxisAngleRot);
	add3d(out, out, m->GyroBias);
}
static void imu_predict_up(FLT *out, SurviveKalmanModel *m) {
	/*
	 *     g = 9.80665
		G = [ kalman_model.Acc[0]/g, kalman_model.Acc[1]/g, 1 + kalman_model.Acc[2]/g]
		rot = quatgetreciprocal(quatnormalize(kalman_model.Pose.Rot))
		return quatrotatevector(rot, G)

	 */
	FLT g = 9.80665;
	FLT accInWorld[3] = {0, 0, 1};
	FLT accInG[3];
	scale3d(accInG, m->Acc, 1. / g);
	add3d(accInWorld, accInWorld, accInG);

	LinmathQuat w2o;
	quatgetreciprocal(w2o, m->Pose.Rot);

	quatrotatevector(out, w2o, accInWorld);
	for (int i = 0; i < 3; i++)
		out[i] += m->AccBias[i];
}

static void imu_predict(FLT *out, const FLT *m) {
	/*
	def imu_predict(kalman_model):
	return [*imu_predict_up(kalman_model), *imu_predict_gyro(kalman_model)]
	*/
	imu_predict_up(out, (SurviveKalmanModel *)m);
	imu_predict_gyro(out + 3, (SurviveKalmanModel *)m);
}

gen_function_def imu_predict_def = {.name = "imu_predict",
									.generated = general_imu_predict,
									.generate_inputs = random_kalman_model,
									.check = imu_predict,
									.outputs = 6,
									.jacobians = {
										{.suffix = "kalman_model",
										 .jacobian = general_gen_imu_predict_jac_kalman_model,
										 .jacobian_length = sizeof(SurviveKalmanModel) / sizeof(FLT)},
									}};

TEST(Generated, imu_predict) {
	SurviveKalmanModel m = {.Pose = {.Rot = {1}}, .Acc = {0, 0, 9.80665}};

	{
		FLT imu[6] = {0};
		FLT imu_gt[6] = {0, 0, 2, 0, 0, 0};
		imu_predict_up(imu, &m);
		ASSERT_DOUBLE_ARRAY_EQ(6, imu, imu_gt);
	}
	{
		m.Acc[2] = 0;
		FLT imu[6] = {0};
		FLT imu_gt[6] = {0, 0, 1, 0, 0, 0};
		imu_predict_up(imu, &m);
		ASSERT_DOUBLE_ARRAY_EQ(6, imu, imu_gt);
	}
	{
		m.Acc[2] = -9.80665;
		FLT imu[6] = {0};
		FLT imu_gt[6] = {0, 0, 0, 0, 0, 0};
		imu_predict_up(imu, &m);
		ASSERT_DOUBLE_ARRAY_EQ(6, imu, imu_gt);
	}
	return test_gen_function_def(&imu_predict_def);
}

void print_pose(const SurvivePose *pose) {
	TEST_PRINTF("[%f %f %f] [%f %f %f %f]\n", pose->Pos[0], pose->Pos[1], pose->Pos[2], pose->Rot[0], pose->Rot[1],
				pose->Rot[2], pose->Rot[3]);
}

void print_point(const FLT *Pos) { TEST_PRINTF("[%f %f %f]\n", Pos[0], Pos[1], Pos[2]); }

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
	TEST_PRINTF("%.16f\n", ang);
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

	TEST_PRINTF("Lib: " Quat_format "\n", LINMATH_QUAT_EXPAND(qo));
	TEST_PRINTF("Gen: " Quat_format "\n", LINMATH_QUAT_EXPAND(qo2));
}
#endif

extern void rot_predict_quat(FLT t, const void *k, const SvMat *f_in, SvMat *f_out);

TEST(Generated, imu_predict_up) {
	SurviveKalmanModel model = {.Pose = {.Rot = {1}}};
	FLT accel[3] = {linmath_rand(-1, 1), linmath_rand(-1, 1), linmath_rand(-1, 1)};
	normalize3d(accel, accel);

	LinmathVec3d up = {0, 0, 1};
	quatfrom2vectors(model.Pose.Rot, accel, up);

	FLT accel_out[3];
	gen_imu_predict_up(accel_out, &model); // gen_imu_predict_up(accel_out, &model);

	ASSERT_DOUBLE_ARRAY_EQ(3, accel, accel_out);
	return 0;
}

TEST(Generated, rot_predict_quat) {
	FLT _mi[7] = { 0 };
	FLT _mo1[7] = { 0 };
	FLT _mo2[7] = { 0 };
	SvMat mi = svMat(7, 1, _mi);
	SvMat mo1 = svMat(7, 1, _mo1);
	SvMat mo2 = svMat(7, 1, _mo2);

	FLT t = next_rand(5);

	random_quat(_mi);
	random_axis_angle(_mi + 4);

	rot_predict_quat(t, 0, &mi, &mo1);

	gen_imu_rot_f(_mo2, t, _mi);

	TEST_PRINTF("Lib: " SurvivePose_format "\n", LINMATH_QUAT_EXPAND(_mo1), LINMATH_VEC3_EXPAND(_mo1 + 4));
	TEST_PRINTF("Gen: " SurvivePose_format "\t"
				"\n",
				LINMATH_QUAT_EXPAND(_mo2), LINMATH_VEC3_EXPAND(_mo2 + 4));

	FLT err = 0;
	for(int i = 0;i < 7;i++)
		err += fabs((_mo1[i] - _mo2[i]) * (_mo1[i] - _mo2[i]));
	return err > 1e-5 ? -1 : 0;
}

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

	for (int i = 0; i < 200000; i++) {
		// survive_reproject_full_jac_obj_pose
		gen_reproject_jac_obj_p(out_jac, &obj2world, pt, &world2lh, bsd.fcal);
	}

	return 0;
}

struct reproject_input {
	SurvivePose p;
	BaseStationCal fcal[2];
	SurvivePose world2lh;
	LinmathPoint3d pt;
};

size_t generate_reproject_input(FLT *out) {
	if (out != 0) {
		struct reproject_input *s = (struct reproject_input *)out;
		s->p = random_pose();
		// s->p.Rot[1] = .1;
		// quatnormalize(s->p.Rot, s->p.Rot);
		s->world2lh = random_pose();
		random_point(s->pt);
		random_fcal(s->fcal);
		random_fcal(s->fcal + 1);
	}
	return sizeof(struct reproject_input);
}

struct reproject_input_axisangle {
	LinmathAxisAnglePose p;
	BaseStationCal fcal[2];
	LinmathAxisAnglePose  world2lh;
	LinmathPoint3d pt;
};
size_t generate_pose(FLT* out) {
	if (out != 0) {
		SurvivePose * p = (SurvivePose *)out;
		*p = random_pose();
	}
	return sizeof(SurvivePose);
}
size_t generate_reproject_input_axisangle(FLT *out) {
	if (out != 0) {
		struct reproject_input _s;
		generate_reproject_input((FLT*)&_s);

		struct reproject_input_axisangle *s = (struct reproject_input_axisangle *)out;
		memcpy(s->pt, _s.pt, sizeof(s->pt));
		memcpy(s->fcal, _s.fcal, sizeof(s->fcal));
		memcpy(s->p.Pos, _s.p.Pos, sizeof(s->p.Pos));
		memcpy(s->world2lh.Pos, _s.p.Pos, sizeof(s->world2lh.Pos));
		quattoaxisanglemag(s->p.AxisAngleRot, _s.p.Rot);
		quattoaxisanglemag(s->world2lh.AxisAngleRot, _s.world2lh.Rot);

	}
	return sizeof(struct reproject_input_axisangle);
}


static void general_gen_reproject_x_gen2(FLT *out, const FLT *_input) {
	struct reproject_input *input = (struct reproject_input *)_input;
	*out = gen_reproject_axis_x_gen2(&input->p, input->pt, &input->world2lh, input->fcal);
}
static void general_gen_reproject_y_gen2(FLT *out, const FLT *_input) {
	struct reproject_input *input = (struct reproject_input *)_input;
	*out = gen_reproject_axis_y_gen2(&input->p, input->pt, &input->world2lh, input->fcal + 1);
}

static void general_gen_reproject_x(FLT *out, const FLT *_input) {
	struct reproject_input *input = (struct reproject_input *)_input;
	*out = gen_reproject_axis_x(&input->p, input->pt, &input->world2lh, input->fcal);
}
static void general_gen_reproject_y(FLT *out, const FLT *_input) {
	struct reproject_input *input = (struct reproject_input *)_input;
	*out = gen_reproject_axis_y(&input->p, input->pt, &input->world2lh, input->fcal + 1);
}

static void general_gen_reproject_x_gen2_jac_obj(FLT *out, const FLT *_input) {
	struct reproject_input *input = (struct reproject_input *)_input;
	gen_reproject_axis_x_gen2_jac_obj_p(out, &input->p, input->pt, &input->world2lh, input->fcal);
}
static void general_gen_reproject_y_gen2_jac_obj(FLT *out, const FLT *_input) {
	struct reproject_input *input = (struct reproject_input *)_input;
	gen_reproject_axis_y_gen2_jac_obj_p(out, &input->p, input->pt, &input->world2lh, input->fcal + 1);
}
static void general_gen_reproject_x_jac_obj(FLT *out, const FLT *_input) {
	struct reproject_input *input = (struct reproject_input *)_input;
	gen_reproject_axis_x_jac_obj_p(out, &input->p, input->pt, &input->world2lh, input->fcal);
}
static void general_gen_reproject_y_jac_obj(FLT *out, const FLT *_input) {
	struct reproject_input *input = (struct reproject_input *)_input;
	gen_reproject_axis_y_jac_obj_p(out, &input->p, input->pt, &input->world2lh, input->fcal + 1);
}
static void general_reproject_x_gen2(FLT *out, const FLT *_input) {
	struct reproject_input *input = (struct reproject_input *)_input;

	LinmathVec3d world_pt;
	ApplyPoseToPoint(world_pt, &input->p, input->pt);

	LinmathPoint3d t_pt;
	ApplyPoseToPoint(t_pt, &input->world2lh, world_pt);

	*out = survive_reproject_axis_x_gen2(input->fcal, t_pt);
}
static void general_reproject_y_gen2(FLT *out, const FLT *_input) {
	struct reproject_input *input = (struct reproject_input *)_input;

	LinmathVec3d world_pt;
	ApplyPoseToPoint(world_pt, &input->p, input->pt);

	LinmathPoint3d t_pt;
	ApplyPoseToPoint(t_pt, &input->world2lh, world_pt);

	*out = survive_reproject_axis_y_gen2(input->fcal, t_pt);
}
static void general_reproject_x(FLT *out, const FLT *_input) {
	struct reproject_input *input = (struct reproject_input *)_input;

	LinmathVec3d world_pt;
	ApplyPoseToPoint(world_pt, &input->p, input->pt);

	LinmathPoint3d t_pt;
	ApplyPoseToPoint(t_pt, &input->world2lh, world_pt);

	*out = survive_reproject_axis_x(input->fcal, t_pt);
}
static void general_reproject_y(FLT *out, const FLT *_input) {
	struct reproject_input *input = (struct reproject_input *)_input;

	LinmathVec3d world_pt;
	ApplyPoseToPoint(world_pt, &input->p, input->pt);

	LinmathPoint3d t_pt;
	ApplyPoseToPoint(t_pt, &input->world2lh, world_pt);

	*out = survive_reproject_axis_y(input->fcal, t_pt);
}
static void general_gen_reproject(FLT *out, const FLT *_input) {
	// static inline void gen_reproject(FLT* out, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose*
	// lh_p, const BaseStationCal* bsd) {
	struct reproject_input *input = (struct reproject_input *)_input;
	gen_reproject(out, &input->p, input->pt, &input->world2lh, input->fcal);
}
static void general_gen_reproject_gen2(FLT *out, const FLT *_input) {
	// static inline void gen_reproject(FLT* out, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose*
	// lh_p, const BaseStationCal* bsd) {
	struct reproject_input *input = (struct reproject_input *)_input;
	gen_reproject_gen2(out, &input->p, input->pt, &input->world2lh, input->fcal);
}

void general_gen_reproject_jac_obj(FLT *out, const FLT *_input) {
	// static inline void gen_reproject(FLT* out, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose*
	// lh_p, const BaseStationCal* bsd) {
	struct reproject_input *input = (struct reproject_input *)_input;
	gen_reproject_jac_obj_p(out, &input->p, input->pt, &input->world2lh, input->fcal);
}
void general_gen_reproject_gen2_jac_obj(FLT *out, const FLT *_input) {
	// static inline void gen_reproject(FLT* out, const SurvivePose* obj_p, const FLT* sensor_pt, const SurvivePose*
	// lh_p, const BaseStationCal* bsd) {
	struct reproject_input *input = (struct reproject_input *)_input;
	gen_reproject_gen2_jac_obj_p(out, &input->p, input->pt, &input->world2lh, input->fcal);
}

static void general_reproject(FLT *out, const FLT *_input) {
	struct reproject_input *input = (struct reproject_input *)_input;
	survive_reproject_full(input->fcal, &input->world2lh, &input->p, input->pt, out);
}
static void general_reproject_gen2(FLT *out, const FLT *_input) {
	struct reproject_input *input = (struct reproject_input *)_input;
	survive_reproject_full_gen2(input->fcal, &input->world2lh, &input->p, input->pt, out);
}

gen_function_def reproject_def = {
	.name = "reproject",
	.generated = general_gen_reproject,
	.check = general_reproject,
	.generate_inputs = generate_reproject_input,
	.outputs = 2,
	.jacobians = {{.suffix = "obj", .jacobian = general_gen_reproject_jac_obj, .jacobian_length = 7}}};

TEST(Generated, reproject) { return test_gen_function_def(&reproject_def); }

gen_function_def reproject_gen2_def = {
	.name = "reproject_gen2",
	.generated = general_gen_reproject_gen2,
	.check = general_reproject_gen2,
	.generate_inputs = generate_reproject_input,
	.outputs = 2,
	.jacobians = {{.suffix = "obj", .jacobian = general_gen_reproject_gen2_jac_obj, .jacobian_length = 7}}};

TEST(Generated, reproject_gen2) {
	FLT inputs[] = {+4.532471, +1.504555, +2.278860, +0.553159, +0.464419, +0.401467, -0.563165, +0.164194,
					+0.124270, +0.005944, -0.246393, -0.103026, +0.180054, +0.215506, -0.161558, +0.108294,
					+0.211122, +0.026800, +0.238683, +0.117266, -0.091412, +2.128406, +3.859190, -4.102889,
					+0.281242, -0.174546, -0.361296, -0.871723, +0.375631, +0.393990, -0.180330};
	FLT out[2], out_gen[2];

	general_gen_reproject_y_gen2(out_gen, inputs);
	general_reproject_y_gen2(out, inputs);

	return test_gen_function_def(&reproject_gen2_def);
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

gen_function_def reproject_axis_x_gen2_def = {
	.name = "reproject_axis_x_gen2",
	.generated = general_gen_reproject_x_gen2,
	.check = general_reproject_x_gen2,
	.generate_inputs = generate_reproject_input,
	.outputs = 1,
	.jacobians = {
		{.suffix = "obj", .jacobian = general_gen_reproject_x_gen2_jac_obj, .jacobian_length = 7},
	}};

TEST(Generated, reproject_axis_x_gen2) { return test_gen_function_def(&reproject_axis_x_gen2_def); }

gen_function_def reproject_axis_y_gen2_def = {
	.name = "reproject_axis_y_gen2",
	.generated = general_gen_reproject_y_gen2,
	.check = general_reproject_y_gen2,
	.generate_inputs = generate_reproject_input,
	.outputs = 1,
	.jacobians = {
		{.suffix = "obj", .jacobian = general_gen_reproject_y_gen2_jac_obj, .jacobian_length = 7},
	}};

TEST(Generated, reproject_axis_y_gen2) { return test_gen_function_def(&reproject_axis_y_gen2_def); }

gen_function_def reproject_axis_x_def = {
	.name = "reproject_axis_x",
	.generated = general_gen_reproject_x,
	.check = general_reproject_x,
	.generate_inputs = generate_reproject_input,
	.outputs = 1,
	.jacobians = {
		{.suffix = "obj", .jacobian = general_gen_reproject_x_jac_obj, .jacobian_length = 7},
	}};

TEST(Generated, reproject_axis_x) { return test_gen_function_def(&reproject_axis_x_def); }

gen_function_def reproject_axis_y_def = {
	.name = "reproject_axis_y",
	.generated = general_gen_reproject_y,
	.check = general_reproject_y,
	.generate_inputs = generate_reproject_input,
	.outputs = 1,
	.jacobians = {
		{.suffix = "obj", .jacobian = general_gen_reproject_y_jac_obj, .jacobian_length = 7},
	}};

TEST(Generated, reproject_axis_y) { return test_gen_function_def(&reproject_axis_y_def); }


static void general_gen_reproject_xy(FLT *out, const FLT *_input) {
	struct reproject_input *input = (struct reproject_input *)_input;
	gen_reproject_xy(out, input->fcal, input->pt);
}
static void general_reproject_xy(FLT *out, const FLT *_input) {
	struct reproject_input *input = (struct reproject_input *)_input;
	survive_reproject_xy(input->fcal, input->pt, out);
}

static void general_gen_reproject_xy_jac_sensor_pt(FLT *out, const FLT *_input) {
	struct reproject_input *input = (struct reproject_input *)_input;
	gen_reproject_xy_jac_sensor_pt(out, input->fcal, input->pt);
}

gen_function_def reproject_xy_def = {
	.name = "reproject_xy",
	.generated = general_gen_reproject_xy,
	.check = general_reproject_xy,
	.generate_inputs = generate_reproject_input,
	.outputs = 2,
	.jacobians = {
		{.suffix = "sensor_pt",
		 .jacobian_start_idx = offsetof(struct reproject_input, pt) / sizeof(FLT),
		 .jacobian = general_gen_reproject_xy_jac_sensor_pt, .jacobian_length = 3
		},
	}};

TEST(Generated, reproject_xy) { return test_gen_function_def(&reproject_xy_def); }

static void general_reproject_axisangle(FLT *out, const FLT *_input) {
	struct reproject_input_axisangle *input = (struct reproject_input_axisangle *)_input;
	survive_reproject_full_axisangle(input->fcal, &input->world2lh, &input->p, input->pt, out);
}

static void general_gen_reproject_xy_axisangle(FLT *out, const FLT *_input) {
	struct reproject_input_axisangle *input = (struct reproject_input_axisangle *)_input;
	gen_reproject_axis_angle(out, &input->p, input->pt, &input->world2lh, input->fcal);
}
static void general_gen_reproject_xy_jac_lh_axis_angle(FLT *out, const FLT *_input) {
	struct reproject_input_axisangle *input = (struct reproject_input_axisangle *)_input;
	gen_reproject_jac_lh_p_axis_angle(out, &input->p, input->pt, &input->world2lh, input->fcal);
}
static void general_gen_reproject_xy_jac_obj_axis_angle(FLT *out, const FLT *_input) {
	struct reproject_input_axisangle *input = (struct reproject_input_axisangle *)_input;
	gen_reproject_jac_obj_p_axis_angle(out, &input->p, input->pt, &input->world2lh, input->fcal);
}

gen_function_def reproject_axis_angle = {
	.name = "reproject_axis_angle",
	.generated = general_gen_reproject_xy_axisangle,
	.check = general_reproject_axisangle,
	.generate_inputs = generate_reproject_input_axisangle,
	.outputs = 2,
	.jacobians = {
		{.suffix = "lh", .jacobian = general_gen_reproject_xy_jac_lh_axis_angle,
		 .jacobian_start_idx = offsetof(struct reproject_input_axisangle, world2lh) / sizeof(FLT),
		 .jacobian_length = 7},
		{.suffix = "obj", .jacobian = general_gen_reproject_xy_jac_obj_axis_angle,
			.jacobian_start_idx = offsetof(struct reproject_input_axisangle, p) / sizeof(FLT),
			.jacobian_length = 7},
	}};

TEST(Generated, reproject_axisangle) { return test_gen_function_def(&reproject_axis_angle); }


void general_invert_pose(FLT* out, const FLT* in) {
	SurvivePose* p = (SurvivePose *)in;
	InvertPose((LinmathPose *)out, p);
}
void general_gen_invert_pose(FLT* out, const FLT* in) {
	SurvivePose* p = (SurvivePose *)in;
	gen_invert_pose(out, p);
}
void general_gen_invert_pose_jac_obj_p(FLT* out, const FLT* in) {
	SurvivePose* p = (SurvivePose *)in;
	gen_invert_pose_jac_obj_p(out, p);
}


gen_function_def invert_pose_def = {
	.name = "invert_pose",
	.generated = general_gen_invert_pose,
	.check = general_invert_pose,
	.generate_inputs = generate_pose,
	.outputs = 7,
	.jacobians = {
		{.suffix = "obj", .jacobian = general_gen_invert_pose_jac_obj_p, .jacobian_length = 7},
	}};

TEST(Generated, invert_pose) { return test_gen_function_def(&invert_pose_def); }
