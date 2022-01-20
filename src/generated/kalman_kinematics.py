import math

import cnkalman.codegen as cg
import lighthouse_gen1
import lighthouse_gen2
from common_math import *
from survive_types import *


def quattoeuler(q):
    return [
        atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])),
        asin(2 * (q[0] * q[2] - q[3] * q[1])),
        atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3]))
    ]



def quatfromeuler(euler):
    X = euler[0] / 2.0
    Y = euler[1] / 2.0
    Z = euler[2] / 2.0

    cx = cos(X)
    sx = sin(X)
    cy = cos(Y)
    sy = sin(Y)
    cz = cos(Z)
    sz = sin(Z)

    # Correct according to
    # http://en.wikipedia.org/wiki/Conversion_between_MQuaternions_and_Euler_angles
    return quatnormalize([
        cx * cy * cz + sx * sy * sz,
        sx * cy * cz - cx * sy * sz,
        cx * sy * cz + sx * cy * sz,
        cx * cy * sz - sx * sy * cz
    ])

def axisanglemagnitude(axis_angle):
    qw, qi, qj = axis_angle
    mag = qw * qw + qi * qi + qj * qj
    return sqrt(mag + 1e-10)

def quat2axisangle(q):
    qw, qi, qj, qk = q
    mag = sqrt(qi*qi+qj*qj+qk*qk + 1e-10)
    angle = 2 * atan2(mag, q[0])
    return [q[1] * angle / mag, \
           q[2] * angle / mag, \
           q[3] * angle / mag]

def axisangle2quat(axis_angle):
    mag = axisanglemagnitude(axis_angle)

    x = axis_angle[0] / mag
    y = axis_angle[1] / mag
    z = axis_angle[2] / mag

    sn = sin(mag / 2.0)
    return quatnormalize([cos(mag / 2.0), sn * x, sn * y, sn * z])

@cg.generate_code(axis_angle = 3)
def axisangle2euler(axis_angle):
    return quattoeuler(axisangle2quat(axis_angle))


def apply_ang_velocity(axis_angle, time, q):
    qi, qj, qk = axis_angle
    q1 = axisangle2quat((qi * time, qj * time, qk * time))
    if len(q) == 3:
        return quatrotateabout(q1, axisangle2quat(q))
    return quatrotateabout(q1, q)

# error_state = x0' * x1
@cg.generate_code(x1 = 4, x0 = 4)
def GenerateQuatErrorModel(x1, x0):
    return quattoeuler(quatrotateabout(quatgetreciprocal(x0), x1))

# x1 = x0 * error_state
@cg.generate_code(x0 = 4, error_state = 3)
def GenerateQuatModel(x0, error_state):
    return quatrotateabout(x0, quatfromeuler(error_state))

# error_state = x0' * x1
@cg.generate_code(x1 = 4, x0 = 4)
def GenerateQuatErrorModelApprox(x1, x0):
    return quattoeuler(quatrotateabout(quatgetreciprocal(x0), x1))

# x1 = x0 * error_state
@cg.generate_code(x0 = 4, error_state = 3)
def GenerateQuatModelApprox(x0, error_state):
    a,b,c = error_state
    return quatnormalize(quatrotateabout(x0, [1, a / 2., b / 2., c / 2.]))

@cg.generate_code()
def SurviveKalmanModelToErrorModel(x1: SurviveKalmanModel, x0: SurviveKalmanModel):
    return SurviveKalmanErrorModel(
        Pose=SurviveAxisAnglePose(
            Pos=x1.Pose.Pos - x0.Pose.Pos,
            AxisAngleRot=GenerateQuatErrorModelApprox(x1.Pose.Rot, x0.Pose.Rot)
        ),
        Velocity=SurviveAxisAnglePose(
            Pos=x1.Velocity.Pos - x0.Velocity.Pos,
            AxisAngleRot=x1.Velocity.AxisAngleRot-x0.Velocity.AxisAngleRot
        ),
        Acc=x1.Acc - x0.Acc,
        AccScale=x1.AccScale - x0.AccScale,
        IMUCorrection=x1.IMUCorrection -x0.IMUCorrection,
        AccBias=x1.AccBias -x0.AccBias,
        GyroBias=x1.GyroBias- x0.GyroBias,
    )

@cg.generate_code()
def SurviveKalmanModelAddErrorModel(x0: SurviveKalmanModel, error_state: SurviveKalmanErrorModel):
    return SurviveKalmanModel(
        Pose=SurvivePose(
            Pos=x0.Pose.Pos + error_state.Pose.Pos,
            Rot=GenerateQuatModelApprox(x0.Pose.Rot, error_state.Pose.AxisAngleRot)
        ),
        Velocity=SurviveAxisAnglePose(
            Pos=x0.Velocity.Pos + error_state.Velocity.Pos,
            AxisAngleRot=x0.Velocity.AxisAngleRot + error_state.Velocity.AxisAngleRot
        ),
        Acc=x0.Acc + error_state.Acc,
        AccScale=x0.AccScale + error_state.AccScale,
        IMUCorrection=x0.IMUCorrection + error_state.IMUCorrection,
        AccBias=x0.AccBias + error_state.AccBias,
        GyroBias=x0.GyroBias + error_state.GyroBias,
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
        AccScale=kalman_model.AccScale,
        IMUCorrection=kalman_model.IMUCorrection,
        AccBias=kalman_model.AccBias,
        GyroBias=kalman_model.GyroBias
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
