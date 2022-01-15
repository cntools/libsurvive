from dataclasses import dataclass

from symengine import atan2, asin, cos, sin, tan, sqrt, Matrix
import cnkalman.codegen as cg
import numpy as np

def quatmagnitude(q):
    qw, qi, qj, qk = q
    return sqrt(qw * qw + qi * qi + qj * qj + qk * qk)

def quatnormalize(q):
    qw, qi, qj, qk = q
    mag = quatmagnitude(q)
    return [qw / mag, qi / mag, qj / mag, qk / mag]

def quatgetreciprocal(q):
    return [q[0], -q[1], -q[2], -q[3]]

# q1 * q2
def quatrotateabout(q1, q2):
    return Matrix([(q1[0] * q2[0]) - (q1[1] * q2[1]) - (q1[2] * q2[2]) - (q1[3] * q2[3]),
                      (q1[0] * q2[1]) + (q1[1] * q2[0]) + (q1[2] * q2[3]) - (q1[3] * q2[2]),
                      (q1[0] * q2[2]) - (q1[1] * q2[3]) + (q1[2] * q2[0]) + (q1[3] * q2[1]),
                      (q1[0] * q2[3]) + (q1[1] * q2[2]) - (q1[2] * q2[1]) + (q1[3] * q2[0])])

def quatfind(q1, q2):
    return quatrotateabout(q2, quatgetreciprocal(q1))

@dataclass
class SurvivePose:
    Pos: np.array = np.array((0, 0, 0))
    Rot: np.array = np.array((1, 0, 0, 0))

@dataclass
class SurviveAxisAnglePose:
    Pos: np.array = np.array((0., 0., 0))
    AxisAngleRot: np.array = np.array((0, 0, 0))

@dataclass
class SurviveKalmanModel:
    Pose: SurvivePose
    Velocity: SurviveAxisAnglePose
    Acc: np.array = np.array((0., 0., 0))
    AccScale: float = 1
    IMUCorrection: np.array = np.array((1, 0., 0., 0))
    AccBias: np.array = np.array((0., 0., 0))
    GyroBias: np.array = np.array((0., 0., 0))

@dataclass
class SurviveKalmanErrorModel:
    Pose: SurviveAxisAnglePose
    Velocity: SurviveAxisAnglePose
    Acc: np.array = np.array((0., 0., 0))
    AccScale: float = 1
    IMUCorrection: np.array = (1, 0., 0., 0)
    AccBias: np.array = np.array((0., 0., 0))
    GyroBias: np.array = np.array((0., 0., 0))

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

def axisangle2quat(axis_angle):
    mag = axisanglemagnitude(axis_angle)

    x = axis_angle[0] / mag
    y = axis_angle[1] / mag
    z = axis_angle[2] / mag

    sn = sin(mag / 2.0)
    return quatnormalize([cos(mag / 2.0), sn * x, sn * y, sn * z])


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
    return quatrotateabout(x0, [1, a / 2., b / 2., c / 2.])

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
        Pose=SurvivePose(Pos=pos + vpos * t + obj_acc * (t * t / 2), Rot=new_rot),
        Velocity=SurviveAxisAnglePose(vpos + obj_acc * t, obj_v.AxisAngleRot),
        Acc=kalman_model.Acc,
        AccScale=kalman_model.AccScale,
        IMUCorrection=kalman_model.IMUCorrection,
        AccBias=kalman_model.AccBias,
        GyroBias=kalman_model.GyroBias
    )

@cg.generate_code()
def SurviveKalmanModelErrorPredict(t, x0 : SurviveKalmanModel, error_model : SurviveKalmanErrorModel):
    x1 = SurviveKalmanModelAddErrorModel(x0, error_model)
    x2 = SurviveKalmanModelPredict(t, x1)
    return SurviveKalmanModelToErrorModel(x2, x0)

q0 = [0, 1, 0, 0]
q1 = [.447, .8944, 0, 0]
error_model = GenerateQuatErrorModel(q1, q0)
rq1 = GenerateQuatModel(q0, error_model)
print(error_model, rq1.transpose())
print(quatrotateabout(quatgetreciprocal(q0), q1).transpose())