import math

import cnkalman.codegen as cg
import lighthouse_gen1
import lighthouse_gen2
from common_math import *

from imu_model import SurviveIMUBiasModelAddErrorModel, SurviveIMUBiasModelToErrorModel, imu_predict_up, \
    imu_predict_gyro

from survive_types import *

@cg.generate_code()
def SurvivePoseToErrorModel(x1: SurvivePose, x0: SurvivePose):
    return SurviveAxisAnglePose(
            Pos=x1.Pos - x0.Pos,
            AxisAngleRot=GenerateQuatErrorModel(x1.Rot, x0.Rot)
    )

@cg.generate_code()
def SurvivePoseAddErrorModel(x0: SurvivePose, error_state: SurviveAxisAnglePose):
    return SurvivePose(
        Pos=x0.Pos + error_state.Pos,
        Rot=GenerateQuatModel(x0.Rot, error_state.AxisAngleRot)
    )

@cg.generate_code()
def SurvivePoseToErrorModelExact(x1: SurvivePose, x0: SurvivePose):
    return SurviveAxisAnglePose(
        Pos=x1.Pos - x0.Pos,
        AxisAngleRot=GenerateQuatErrorModel(x1.Rot, x0.Rot)
    )

@cg.generate_code()
def SurvivePoseAddErrorModelExact(x0: SurvivePose, error_state: SurviveAxisAnglePose):
    return SurvivePose(
        Pos=x0.Pos + error_state.Pos,
        Rot=GenerateQuatModel(x0.Rot, error_state.AxisAngleRot)
    )

@cg.generate_code()
def SurviveKalmanModelToErrorModel(x1: SurviveKalmanModel, x0: SurviveKalmanModel):
    return SurviveKalmanErrorModel(
        Pose=SurvivePoseToErrorModel(x1.Pose, x0.Pose),
        Velocity=SurviveAxisAnglePose(
            Pos=x1.Velocity.Pos - x0.Velocity.Pos,
            AxisAngleRot=x1.Velocity.AxisAngleRot-x0.Velocity.AxisAngleRot
        ),
        Acc=x1.Acc - x0.Acc,
        IMUBias=SurviveIMUBiasModelToErrorModel(x1.IMUBias, x0.IMUBias)
    )

@cg.generate_code()
def SurviveKalmanModelAddErrorModel(x0: SurviveKalmanModel, error_state: SurviveKalmanErrorModel):
    return SurviveKalmanModel(
        Pose=SurvivePoseAddErrorModel(x0.Pose, error_state.Pose),
        Velocity=SurviveAxisAnglePose(
            Pos=x0.Velocity.Pos + error_state.Velocity.Pos,
            AxisAngleRot=x0.Velocity.AxisAngleRot + error_state.Velocity.AxisAngleRot
        ),
        Acc=x0.Acc + error_state.Acc,
        IMUBias=SurviveIMUBiasModelAddErrorModel(x0.IMUBias, error_state.IMUBias)
    )

@cg.generate_code()
def SurviveKalmanModelPredict(t, kalman_model : SurviveKalmanModel):
    obj_p = kalman_model.Pose
    obj_v = kalman_model.Velocity
    obj_acc = kalman_model.Acc

    pos = obj_p.Pos
    vpos = obj_v.Pos
    new_rot = apply_ang_velocity(obj_v.AxisAngleRot, t, obj_p.Rot)

    return SurviveKalmanModel(
        Pose=SurvivePose(Pos=pos + vpos * t + obj_acc * (Abs(t) * t / 2), Rot=new_rot),
        Velocity=SurviveAxisAnglePose(vpos + obj_acc * t, obj_v.AxisAngleRot),
        Acc=kalman_model.Acc,
        IMUBias=kalman_model.IMUBias
    )


@cg.generate_code(x0 = 3)
def AxisAngleFlip(x0):
    mag = axisanglemagnitude(x0)
    return x0 * ((mag - 2.*math.pi) / mag)

@cg.generate_code()
def SurviveKalmanModelErrorPredict(t, x0 : SurviveKalmanModel, error_model : SurviveKalmanErrorModel):
    x1 = SurviveKalmanModelAddErrorModel(x0, error_model)
    x2 = SurviveKalmanModelPredict(t, x1)
    return SurviveKalmanModelToErrorModel(x2, x0)

def SurviveObsErrorModel(x0: SurviveKalmanModel, Z: SurviveAxisAnglePose, flag):
    x0aa = SurviveAxisAnglePose(x0.Pose.Pos, quat2axisangle(x0.Pose.Rot))
    return SurviveAxisAnglePose(x0aa.Pos - Z.Pos,
                                AxisAngleFlip(x0aa.AxisAngleRot - Z.AxisAngleRot) if flag else (x0aa.AxisAngleRot - Z.AxisAngleRot))

@cg.generate_code()
def SurviveObsErrorModelNoFlip(x0: SurviveKalmanModel, Z: SurviveAxisAnglePose):
    return SurviveObsErrorModel(x0, Z, False)

@cg.generate_code()
def SurviveObsErrorModelFlip(x0: SurviveKalmanModel, Z: SurviveAxisAnglePose):
    return SurviveObsErrorModel(x0, Z, True)

@cg.generate_code()
def SurviveObsErrorStateErrorModelNoFlip(x0: SurviveKalmanModel, err: SurviveKalmanErrorModel, Z : SurviveAxisAnglePose):
    x1 = SurviveKalmanModelAddErrorModel(x0, err)
    return SurviveObsErrorModel(x1, Z, False)

@cg.generate_code()
def SurviveObsErrorStateErrorModelFlip(x0: SurviveKalmanModel, err: SurviveKalmanErrorModel, Z : SurviveAxisAnglePose):
    x1 = SurviveKalmanModelAddErrorModel(x0, err)
    return SurviveObsErrorModel(x1, Z, True)

def SurviveKalmanModel_LightMeas(dt : float, fn, x0: SurviveKalmanModel, sensor_pt: list, lh_p : SurvivePose,  bsc0 : BaseStationCal):
    x1 = SurviveKalmanModelPredict(dt, x0)
    return fn(x1.Pose, sensor_pt, lh_p, bsc0)

def SurviveKalmanErrorModel_LightMeas(dt : float, fn, x0: SurviveKalmanModel, error_model: SurviveKalmanErrorModel, sensor_pt: list, lh_p : SurvivePose,  bsc0 : BaseStationCal):
    x1 = SurviveKalmanModelAddErrorModel(x0, error_model)
    x2 = SurviveKalmanModelPredict(dt, x1)
    return fn(x2.Pose, sensor_pt, lh_p, bsc0)

@cg.generate_code()
def SurviveKalmanModel_LightMeas_x_gen1(dt : float, x0: SurviveKalmanModel, sensor_pt: list, lh_p : SurvivePose,  bsc0 : BaseStationCal):
    return SurviveKalmanModel_LightMeas(dt, lighthouse_gen1.reproject_axis_x, x0, sensor_pt, lh_p, bsc0)

@cg.generate_code()
def SurviveKalmanModel_LightMeas_y_gen1(dt : float, x0: SurviveKalmanModel, sensor_pt: list, lh_p : SurvivePose,  bsc0 : BaseStationCal):
    return SurviveKalmanModel_LightMeas(dt, lighthouse_gen1.reproject_axis_y, x0, sensor_pt, lh_p, bsc0)

@cg.generate_code()
def SurviveKalmanModel_LightMeas_x_gen2(dt : float, x0: SurviveKalmanModel, sensor_pt: list, lh_p : SurvivePose,  bsc0 : BaseStationCal):
    return SurviveKalmanModel_LightMeas(dt, lighthouse_gen2.reproject_axis_x_gen2, x0, sensor_pt, lh_p, bsc0)

@cg.generate_code()
def SurviveKalmanModel_LightMeas_y_gen2(dt : float, x0: SurviveKalmanModel, sensor_pt: list, lh_p : SurvivePose,  bsc0 : BaseStationCal):
    return SurviveKalmanModel_LightMeas(dt, lighthouse_gen2.reproject_axis_y_gen2, x0, sensor_pt, lh_p, bsc0)

@cg.generate_code()
def SurviveKalmanErrorModel_LightMeas_x_gen1(dt : float, x0: SurviveKalmanModel, error_model: SurviveKalmanErrorModel, sensor_pt: list, lh_p : SurvivePose,  bsc0 : BaseStationCal):
    return SurviveKalmanErrorModel_LightMeas(dt, lighthouse_gen1.reproject_axis_x, x0, error_model, sensor_pt, lh_p, bsc0)

@cg.generate_code()
def SurviveKalmanErrorModel_LightMeas_y_gen1(dt : float, x0: SurviveKalmanModel, error_model: SurviveKalmanErrorModel, sensor_pt: list, lh_p : SurvivePose,  bsc0 : BaseStationCal):
    return SurviveKalmanErrorModel_LightMeas(dt, lighthouse_gen1.reproject_axis_y, x0, error_model, sensor_pt, lh_p, bsc0)

@cg.generate_code()
def SurviveKalmanErrorModel_LightMeas_x_gen2(dt : float, x0: SurviveKalmanModel, error_model: SurviveKalmanErrorModel, sensor_pt: list, lh_p : SurvivePose,  bsc0 : BaseStationCal):
    return SurviveKalmanErrorModel_LightMeas(dt, lighthouse_gen2.reproject_axis_x_gen2, x0, error_model, sensor_pt, lh_p, bsc0)

@cg.generate_code()
def SurviveKalmanErrorModel_LightMeas_y_gen2(dt : float, x0: SurviveKalmanModel, error_model: SurviveKalmanErrorModel, sensor_pt: list, lh_p : SurvivePose,  bsc0 : BaseStationCal):
    return SurviveKalmanErrorModel_LightMeas(dt, lighthouse_gen2.reproject_axis_y_gen2, x0, error_model, sensor_pt, lh_p, bsc0)

@cg.generate_code()
def IMUMeasurementModel(model: SurviveKalmanModel):
    return [*imu_predict_up(model), *imu_predict_gyro(model)]


@cg.generate_code()
def IMUMeasurementErrorModel(x0: SurviveKalmanModel, error_model: SurviveKalmanErrorModel):
    x1 = SurviveKalmanModelAddErrorModel(x0, error_model)
    return [*imu_predict_up(x1), *imu_predict_gyro(x1)]
