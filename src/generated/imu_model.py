from common_math import *
from survive_types import *

import cnkalman.codegen as cg

@cg.generate_code()
def SurviveIMUBiasModelToErrorModel(x1: SurviveIMUBiasModel, x0: SurviveIMUBiasModel):
    return SurviveIMUBiasErrorModel(
        AccScale=x1.AccScale - x0.AccScale,
        IMUCorrection=GenerateQuatErrorModel(x1.IMUCorrection, x0.IMUCorrection),
        AccBias=x1.AccBias - x0.AccBias,
        GyroBias=x1.GyroBias - x0.GyroBias
    )

@cg.generate_code()
def SurviveIMUBiasModelAddErrorModel(x0: SurviveIMUBiasModel, error_state: SurviveIMUBiasErrorModel):
    return SurviveIMUBiasModel(
        AccScale=x0.AccScale + error_state.AccScale,
        IMUCorrection=GenerateQuatModel(x0.IMUCorrection, error_state.IMUCorrection),
        AccBias=x0.AccBias + error_state.AccBias,
        GyroBias=x0.GyroBias + error_state.GyroBias
    )


def imu_predict_up(kalman_model: SurviveKalmanModel):
    g = 9.80665
    acc_scale = kalman_model.IMUBias.AccScale
    acc_bias = kalman_model.IMUBias.AccBias
    G = [kalman_model.Acc[0] / g, kalman_model.Acc[1] / g, 1 + kalman_model.Acc[2] / g]
    rot = quatrotateabout(quatgetreciprocal(quatnormalize(kalman_model.Pose.Rot)),
                          quatnormalize(kalman_model.IMUBias.IMUCorrection))
    GinObj = quatrotatevector(rot, G)
    return [
        acc_scale[0] * GinObj[0] + acc_bias[0],
        acc_scale[1] * GinObj[1] + acc_bias[1],
        acc_scale[2] * GinObj[2] + acc_bias[2]
    ]


def imu_predict_gyro(kalman_model: SurviveKalmanModel):
    rot = quatrotateabout(quatgetreciprocal(quatnormalize(kalman_model.Pose.Rot)),
                          quatnormalize(kalman_model.IMUBias.IMUCorrection))
    rotv = quatrotatevector(rot, kalman_model.Velocity.AxisAngleRot)
    return [rotv[0] + kalman_model.IMUBias.GyroBias[0],
            rotv[1] + kalman_model.IMUBias.GyroBias[1],
            rotv[2] + kalman_model.IMUBias.GyroBias[2]
            ]
