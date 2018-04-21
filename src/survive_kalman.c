
#include <stdio.h>
#include "survive_kalman.h"

void survive_kpose_predict(const survive_kpose_t *kpose, survive_timecode when, SurvivePose *out) {
    for(int i = 0;i < 3;i++)
        out->Pos[i] = kpose->state.pose.Pos[i] +
                kpose->state.velocity[i] * (survive_timecode_difference(when, kpose->timecode) / 48000000.);
}

void survive_kpose_init(survive_kpose_t *kpose, survive_timecode timecode, const SurvivePose *pose,
                                  const double *pos_variance, FLT rot_variance) {
    kpose->variance.rot_variance = rot_variance;
    copy3d(kpose->variance.pos_variance, pos_variance);
    kpose->state.pose = *pose;

    kpose->process_variance_per_second.rot_variance = .01;

    FLT avg_acc_per_sec = 0.5;
    FLT avg_vel_per_sec = 0.5;
    for(int i = 0;i < 3;i++) {
        kpose->process_variance_per_second.vel_variance[i] = avg_acc_per_sec;
        kpose->process_variance_per_second.pos_variance[i] = avg_vel_per_sec;
    }

    kpose->timecode = timecode;

    fprintf(stderr, "Init!\n");

}

static inline void kalman_update(FLT* R, FLT* X, FLT P, FLT X_p) {
    FLT gain = *R / (*R + P);
    *X = *X * (1. - gain) + X_p * gain;
    *R = *R * (1. - gain);// + P * gain;

    //fprintf(stderr, "Gain %f\n", gain);
}

static inline void update(survive_kpose_t *kpose, survive_timecode when) {
    survive_kpose_predict(kpose, when, &kpose->state.pose);
    FLT diff_seconds = (survive_timecode_difference(when, kpose->timecode) / 48000000.);
    for(int i = 0;i < 3;i++) {
        kpose->variance.pos_variance[i] += kpose->process_variance_per_second.pos_variance[i] * diff_seconds;
    }
    kpose->variance.rot_variance += kpose->process_variance_per_second.rot_variance * diff_seconds;
    kpose->timecode = when;
}

void survive_kpose_integrate_pose(survive_kpose_t *kpose, survive_timecode timecode, const SurvivePose *pose,
                                  const double *pos_variance, FLT rot_variance) {
    if (kpose->variance.rot_variance == 0.0) {
        survive_kpose_init(kpose, timecode, pose, pos_variance, rot_variance);
        return;
    }

    update(kpose, timecode);

    FLT rot_gain = kpose->variance.rot_variance / (kpose->variance.rot_variance + rot_variance);
    LinmathQuat new_rot;
    //quatslerp(new_rot, pose->Rot, kpose->state.pose.Rot, 1. - rot_gain);
    for(int i = 0;i < 4;i++)
        kpose->state.pose.Rot[i] = (1. - rot_gain) * kpose->state.pose.Rot[i] + rot_gain * pose->Rot[i];
    quatnormalize(kpose->state.pose.Rot,kpose->state.pose.Rot);
    //quatcopy(kpose->state.pose.Rot, new_rot);
    kpose->variance.rot_variance = kpose->variance.rot_variance * (1. - rot_gain);// + rot_variance * rot_gain;

    for(int i = 0;i < 3;i++) {
        kalman_update(kpose->variance.pos_variance + i, kpose->state.pose.Pos + i, pos_variance[i], pose->Pos[i]);
    }

}
