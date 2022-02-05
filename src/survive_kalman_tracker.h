#ifndef _SURVIVE_IMU_H
#define _SURVIVE_IMU_H

#include <cnkalman/kalman.h>

#include "poser.h"
#include "survive.h"

#include "survive_types.h"
#include <stdbool.h>
#include <stdint.h>
#include "variance.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef PoserDataGlobalSceneMeasurement LightInfo;

struct pid_t {
	FLT err;
	FLT integration;

	FLT Kp, Ki, Kd;
};

struct SurviveKalmanTracker_Params {
	FLT process_weight_jerk, process_weight_acc, process_weight_vel, process_weight_pos;
	FLT process_weight_ang_velocity, process_weight_rotation;
	FLT process_weight_acc_bias;
	FLT process_weight_gyro_bias;
	FLT initial_acc_scale_variance;
	FLT initial_acc_bias_variance, initial_gyro_bias_variance;
	FLT initial_gyro_variance;
	FLT initial_variance_imu_correction;
};

/**
 * The kalman model as it pertains to LH tracking has a state space like so:
 *
 * @SurviveKalmanModel:
 * [ SurvivePose Pose, SurviveVelocity Vel, Point3d Acc, Point3d GyroBias ]
 *
 * for a 19 dimension state space.
 *
 * This model has three types of observations:
 * * IMU data - @survive_kalman_tracker_integrate_imu
 * * Raw light data - @survive_kalman_tracker_integrate_light
 * * Poser data - @survive_kalman_tracker_integrate_observation
 *
 * IMU data and raw light data can drift over time; but the poser data input is
 * assumed to be noisy but not drift in time.
 */
typedef struct SurviveKalmanTracker {
	SurviveObject *so;

	FLT acc_norm_penalty;
	FLT acc_var;
	FLT gyro_var;
	FLT obs_cov_scale;
	FLT obs_pos_var;
	FLT obs_rot_var;
	bool obs_axisangle_model;
	FLT light_var;

	int light_batchsize;

	FLT last_light_time, last_report_time, first_report_time;
	FLT first_imu_time, last_imu_time;
	FLT min_report_time;
	int report_covariance_cnt;
	FLT report_sampled_cloud;

	bool minimize_state_space, use_error_state;
	bool use_raw_obs;
	bool show_raw_obs;

	FLT joint_lightcap_ratio;
	int joint_min_sensor_cnt;
	int lightcap_min_sensor_cnt;

	FLT light_threshold_var, report_threshold_var, light_error_threshold;
	FLT zvu_stationary_var;
	FLT zvu_no_light_time;
	FLT zvu_no_light_var;
	int noise_model;
	FLT zvu_moving_var;
	int32_t light_required_obs;
	int32_t report_ignore_start;
	int32_t report_ignore_start_cnt;

	struct SurviveKalmanTracker_Params params;

	// Kalman state is layed out as SurviveKalmanModel
	cnkalman_state_t model, imu_bias_model;
	cnkalman_meas_model_t obs_model, lightcap_model, imu_model, zvu_model, joint_model;

	const char* datalog_tag;

	struct {
		uint32_t late_imu_dropped;
		uint32_t late_light_dropped;

		FLT imu_total_error;
		size_t imu_count;
		FLT lightcap_total_error;
		size_t lightcap_count;

		FLT lightcap_error_by_lh[NUM_GEN2_LIGHTHOUSES];
		size_t lightcap_count_by_lh[NUM_GEN2_LIGHTHOUSES];

		FLT obs_total_error;
		size_t obs_count;

		size_t reported_poses, dropped_poses;
		FLT dropped_var[sizeof(SurviveKalmanModel) / sizeof(FLT)];
		FLT reported_var[sizeof(SurviveKalmanModel) / sizeof(FLT)];

		FLT acc_norm;
		FLT stationary_acc_norm;
		size_t stationary_imu_count;

		size_t no_light_imu_count;

		uint32_t joint_model_dropped;
		uint32_t joint_model_sensor_cnt_sum;
		uint32_t lightcap_model_dropped;
		uint32_t lightcap_model_sensor_cnt_sum;
	} stats;

	FLT imu_residuals;
	FLT light_residuals_all;
	FLT light_residuals[NUM_GEN2_LIGHTHOUSES];

	FLT Obs_R[7 * 7];
	FLT IMU_R[6 * 6];
	FLT Lightcap_R;

	FLT lightcap_max_error;
	int light_rampin_length;
	bool use_error_for_lh_pos;

	LightInfo savedLight[32];
	uint32_t savedLight_idx;

	SurviveKalmanModel state, previous_state, reported_state_variance;
	SurviveKalmanModel process_variance;

	size_t state_variance_count;

	struct variance_tracker imu_variance, pose_variance;
	struct variance_tracker light_variance[NUM_GEN2_LIGHTHOUSES][SENSORS_PER_OBJECT][2];
} SurviveKalmanTracker;

SURVIVE_EXPORT SurviveVelocity survive_kalman_tracker_velocity(const SurviveKalmanTracker *tracker);
SURVIVE_EXPORT bool survive_kalman_tracker_predict_variance(const SurviveKalmanTracker *tracker, FLT time, CnMat* P);
SURVIVE_EXPORT void survive_kalman_tracker_predict(const SurviveKalmanTracker *tracker, FLT time, SurvivePose *out);
SURVIVE_EXPORT void survive_kalman_tracker_init(SurviveKalmanTracker *tracker, SurviveObject *so);
SURVIVE_EXPORT void survive_kalman_tracker_free(SurviveKalmanTracker *tracker);
SURVIVE_EXPORT void survive_kalman_tracker_integrate_imu(SurviveKalmanTracker *tracker, PoserDataIMU *data);
SURVIVE_EXPORT void survive_kalman_tracker_integrate_light(SurviveKalmanTracker *tracker, PoserDataLight *data);

SURVIVE_EXPORT void survive_kalman_tracker_integrate_observation(PoserData *pd, SurviveKalmanTracker *tracker,
																 const SurvivePose *pose, const struct CnMat *R);
SURVIVE_EXPORT void survive_kalman_tracker_report_state(PoserData *pd, SurviveKalmanTracker *tracker);
SURVIVE_EXPORT void survive_kalman_tracker_lost_tracking(SurviveKalmanTracker *tracker, bool allowLHReset);

SURVIVE_EXPORT void survive_kalman_tracker_predict_jac(FLT dt, const struct cnkalman_state_s *k, const struct CnMat *x0,
													   struct CnMat *x1, struct CnMat *f_out);
SURVIVE_EXPORT void survive_kalman_tracker_process_noise(const struct SurviveKalmanTracker_Params *params,
														 bool errorState, FLT t, const CnMat *x, struct CnMat *q_out);
SURVIVE_EXPORT bool survive_kalman_tracker_imu_measurement_model(void *user, const struct CnMat *Z,
																 const struct CnMat *x_t, struct CnMat *y,
																 struct CnMat *H_k);
SURVIVE_EXPORT void survive_kalman_tracker_correct_imu(SurviveKalmanTracker *tracker, LinmathVec3d out, const LinmathVec3d accel);

#define MEAS_MDL_CONFIG(prefix, x, default_iterations, default_max_error)                                              \
	STRUCT_NAMED_CONFIG_SECTION(prefix##_##x, cnkalman_meas_model_t)                                                   \
	STRUCT_CONFIG_ITEM("kalman-" #prefix "-" #x "-adaptive", "Use adaptive covariance for " #x, 0, t->adaptive)        \
	STRUCT_CONFIG_ITEM("kalman-" #prefix "-" #x "-max-error", "Max tolerable initial error " #x, default_max_error,    \
					   t->term_criteria.max_error)                                                                     \
	STRUCT_CONFIG_ITEM("kalman-" #prefix "-" #x "-iterations", "Max iterations for " #x, default_iterations,           \
					   t->term_criteria.max_iterations)                                                                \
	STRUCT_CONFIG_ITEM("kalman-" #prefix "-" #x "-step-size", "Step size for " #x ".", -1, t->numeric_step_size)       \
	STRUCT_CONFIG_ITEM("kalman-" #prefix "-" #x "-error-state-model",                                                  \
					   "Use error state model jacobian if available " #x, true, t->error_state_model)                  \
	END_STRUCT_CONFIG_SECTION(cnkalman_meas_model_t)

#ifdef __cplusplus
};
#endif

#endif
