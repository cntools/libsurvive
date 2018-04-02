#include <assert.h>
#include <libsurvive/survive.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

uint32_t current_timecode;

void light_process(SurviveObject *so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length,
				   uint32_t lighthouse) {
	current_timecode = timecode;
	survive_default_light_process(so, sensor_id, acode, timeinsweep, timecode, length, lighthouse);
}

void raw_pose_process(SurviveObject *so, uint8_t _lh, SurvivePose *pose2w) {
	survive_default_raw_pose_process(so, _lh, pose2w);

	/*printf("Pose: ");
	for(int i = 0;i < 7;i++) {
	  printf("%f, ", pose2w->Pos[i]);
	};
	printf("\n");
	*/
	for (int lh = 0; lh < 2; lh++) {
		SurvivePose pose2lh = {};
		SurvivePose w2lh = {};
		InvertPose(&w2lh, &so->ctx->bsd[lh].Pose);
		ApplyPoseToPose(&pose2lh, &w2lh, pose2w);

		SurviveSensorActivations *scene = &so->activations;
		for (size_t sensor_idx = 0; sensor_idx < so->sensor_ct; sensor_idx++) {
			if (SurviveSensorActivations_isPairValid(scene, SurviveSensorActivations_default_tolerance / 2,
													 current_timecode, sensor_idx, lh)) {
				uint32_t *lengths = scene->lengths[sensor_idx][lh];

				const FLT *sensor_location = so->sensor_locations + 3 * sensor_idx;
				const FLT *sensor_normals = so->sensor_normals + 3 * sensor_idx;

				LinmathPoint3d sensor = {}, sensorN = {};

				scale3d(sensorN, sensor_normals, .01);
				add3d(sensorN, sensor_location, sensor_normals);

				/*
				printf("\n");
				printf("%f, %f, %f\n", sensor_location[0], sensor_location[1], sensor_location[2]);
				printf("%f, %f, %f\n", sensorN[0], sensorN[1], sensorN[2]);

				FLT m[4][4];
				PoseToMatrix((FLT*)m, &pose2lh);

				for(int i = 0;i < 7;i++) {
			  printf("%f, ", pose2lh.Pos[i]);
				};
				printf("\n");
				for(int i = 0;i < 4;i++) {
			  for(int j = 0;j < 4;j++) {
				printf("%f, ", m[i][j]);
			  }
			  printf("\n");
				}
				printf("\n");
				*/
				ApplyPoseToPoint(sensor, &pose2lh, sensor_location);
				ApplyPoseToPoint(sensorN, &pose2lh, sensorN);

				// printf("%f, %f, %f\n", sensor[0], sensor[1], sensor[2]);
				// printf("%f, %f, %f\n", sensorN[0], sensorN[1], sensorN[2]);
				FLT dist = magnitude3d(sensor);
				LinmathVec3d zout = {0, 0, -dist};
				LinmathQuat r;
				quatfrom2vectors(r, sensor, zout);
				quatrotatevector(sensorN, r, sensorN);
				sub3d(sensorN, sensorN, zout);

				FLT dot = dot3d(sensor, sensorN);
				if (dist > 20 || dist < 0.25)
					continue;

				FLT angs[2] = {};
				for (int axis = 0; axis < 2; axis++) {
					angs[axis] = cos(atan2(fabs(sensorN[axis ? 0 : 1]), sensorN[2]));
				}
				FLT area = angs[0] * angs[1];

				printf("%u\t%u\t%lu\t%f\t%f\t%f\t%f\t%u\t%u\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", current_timecode, lh,
					   sensor_idx, dist, angs[0], angs[1], area, lengths[0], lengths[1], dot, sensor[0], sensor[1],
					   sensor[2], sensorN[0], sensorN[1], sensorN[2]);

				// assert(angs[0] >= 0);
			}
		}
	}
}

int main(int argc, char **argv) {
	SurviveContext *ctx = survive_init(argc, argv);
	if (ctx == 0) // implies -help or similiar
		return 0;

	printf("timecode\tlh\tsensor_idx\tdistance\tnorm_x\tnorm_y\tarea\tlength_x\tlength_y\tdot\tx\ty\tz\tnx\tny\tnz\n");

	survive_startup(ctx);

	survive_install_raw_pose_fn(ctx, raw_pose_process);
	survive_install_light_fn(ctx, light_process);
	while (survive_poll(ctx) == 0) {
	}

	survive_close(ctx);
	return 0;
}
