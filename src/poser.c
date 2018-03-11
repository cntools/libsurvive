#include <stdint.h>

#include "poser.h"
#include "survive_internal.h"

void PoserData_poser_raw_pose_func(PoserData *poser_data, SurviveObject *so, uint8_t lighthouse, FLT *pose) {
	if (poser_data->rawposeproc) {
		poser_data->rawposeproc(so, lighthouse, pose, poser_data->userdata);
	} else {
		so->ctx->rawposeproc(so, lighthouse, pose);
	}
}

void PoserData_lighthouse_pose_func(PoserData *poser_data, SurviveObject *so, uint8_t lighthouse, SurvivePose *pose) {
	if (poser_data->lighthouseposeproc) {
		poser_data->lighthouseposeproc(so, lighthouse, pose, poser_data->userdata);
	} else {
		so->ctx->lighthouseposeproc(so->ctx, lighthouse, pose);
	}
}
