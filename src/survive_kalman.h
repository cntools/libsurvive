
#include <linmath.h>
#include <survive_types.h>

typedef struct {
    SurvivePose pose;
    LinmathVec3d velocity;
} survive_kalman_pose_state_t ;

typedef struct {
    LinmathVec3d pos_variance;
    FLT rot_variance;
    LinmathVec3d vel_variance;
} survive_kalman_pose_var_t ;

typedef struct {
    survive_kalman_pose_state_t state;
    survive_kalman_pose_var_t variance;

    survive_timecode timecode;
    survive_kalman_pose_var_t process_variance_per_second;
} survive_kpose_t;


void survive_kpose_predict(const survive_kpose_t* kpose, survive_timecode when, SurvivePose* out);

void survive_kpose_integrate_pose(survive_kpose_t* kpose, survive_timecode timecode,
                                  const SurvivePose* pose,
                                  const LinmathVec3d pos_variance, FLT rot_variance);

void survive_kpose_integrate_pose_velocity(survive_kpose_t* kpose, survive_timecode timecode,
                                           const SurvivePose* pose, const LinmathVec3d vel,
                                           const LinmathVec3d pos_variance, FLT rot_variance, const LinmathVec3d vel_var);
