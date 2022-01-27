import sympy
import cnkalman.codegen as cg
from cnkalman.codegen import atan2, sqrt, cos, sin, Matrix, Pow, Mul, asin, Abs, tan

from survive_types import *

import symengine

def simple_neg(x):
    if isinstance(x, sympy.Expr):
        return sympy.Mul(x, -1, evaluate=False)
    if isinstance(x, symengine.Expr):
        return symengine.Mul(x, -1, evaluate=False)
    return -x

def cross(sensor_pt, axis_angle):
    a = sensor_pt
    b = axis_angle
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]

def quatrotatevector(q, pt):
    tmp = cross(q[1:], pt)
    for i in range(3):
        tmp[i] += pt[i] * q[0]
    tmp2 = cross(q[1:], tmp)

    return [
        pt[0] + 2 * tmp2[0],
        pt[1] + 2 * tmp2[1],
        pt[2] + 2 * tmp2[2],
        ]

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

def add3d(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]

def apply_pose_to_pt(obj_p, sensor_pt):
    px, py, pz = obj_p.Pos
    #return quatrotatevector(obj_p.Rot, sensor_pt) + sp.Matrix((px, py, pz))
    return add3d(quatrotatevector(obj_p.Rot, sensor_pt), obj_p.Pos)

@cg.generate_code()
def apply_pose_to_pose(lhs : SurvivePose, rhs: SurvivePose):
    return SurvivePose(
        Pos=apply_pose_to_pt(lhs, rhs.Pos),
        Rot=quatrotateabout(lhs.Rot, rhs.Rot)
    )

def axisanglemagnitude(axis_angle):
    qw, qi, qj = axis_angle
    mag = qw * qw + qi * qi + qj * qj
    return sqrt(mag + 1e-10)

def axisanglerotationmatrix(axis_angle):
    R = axisanglemagnitude(axis_angle)

    x = axis_angle[0] / R
    y = axis_angle[1] / R
    z = axis_angle[2] / R

    csr = cos(R)
    one_minus_csr = (1 - csr)
    snr = sin(R)

    return Matrix(
        [[csr + x * x * (1 - csr), x * y * one_minus_csr - z * snr, x * z * one_minus_csr + y * snr],
         [y * x * one_minus_csr + z * snr, csr + y * y * one_minus_csr, y * z * one_minus_csr - x * snr],
         [z * x * one_minus_csr - y * snr, z * y * one_minus_csr + x * snr, csr + z * z * one_minus_csr]])

def axisanglerotatevector(axis_angle, sensor_pt):
    x, y, z = sensor_pt
    return axisanglerotationmatrix(axis_angle) * sp.Matrix((x, y, z))

def apply_axisangle_pose_to_pt(obj_p_axisangle, sensor_pt):
    px, py, pz = obj_p_axisangle.Pos
    return (axisanglerotatevector(obj_p_axisangle.Rot, sensor_pt) + sp.Matrix((px, py, pz)))

def sensor_to_world(obj_p, sensor_pt, lh_p):
    if len(obj_p.Rot) == 4:
        return apply_pose_to_pt(lh_p, apply_pose_to_pt(obj_p, sensor_pt))
    return apply_axisangle_pose_to_pt(lh_p, apply_axisangle_pose_to_pt(obj_p, sensor_pt))


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
def GenerateQuatErrorModelExact(x1, x0):
    return quattoeuler(quatrotateabout(quatgetreciprocal(x0), x1))

# x1 = x0 * error_state
@cg.generate_code(x0 = 4, error_state = 3)
def GenerateQuatModelExact(x0, error_state):
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


# error_state = x0' * x1
@cg.generate_code(x1 = 4, x0 = 4)
def GenerateQuatErrorModel(x1, x0):
    return GenerateQuatErrorModelApprox(x1, x0)

# x1 = x0 * error_state
@cg.generate_code(x0 = 4, error_state = 3)
def GenerateQuatModel(x0, error_state):
    return GenerateQuatModelApprox(x0, error_state)
