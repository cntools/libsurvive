import math
import sys
import types
import itertools
import inspect

# import symengine as se
from collections import defaultdict

from collections.abc import Iterable

import symengine as sp
from symengine import sqrt, atan2, tan, asin, cos, Pow, sin, Piecewise, Symbol, cse
from sympy import evaluate, Atom

phase_0, phase_1 = sp.symbols('phase_0, phase_1')
tilt_0, tilt_1 = sp.symbols('tilt_0, tilt_1')
curve_0, curve_1 = sp.symbols('curve_0, curve_1')
gibPhase_0, gibPhase_1 = sp.symbols('gibPhase_0, gibPhase_1')
gibMag_0, gibMag_1 = sp.symbols('gibMag_0, gibMag_1')
ogeePhase_0, ogeePhase_1 = sp.symbols('ogeePhase_0, ogeePhase_1')
ogeeMag_0, ogeeMag_1 = sp.symbols('ogeeMag_0, ogeeMag_1')


class SurviveType:
    pass

class BaseStationCal(SurviveType):
    def __init__(self, cal):
        (self.phase, self.tilt,
         self.curve,
         self.gibpha,
         self.gibmag,
         self.ogeephase,
         self.ogeemag) = cal

def bsd():
    return [bsc0(), bsc1()]


def bsc0():
    return BaseStationCal((phase_0, tilt_0, curve_0, gibPhase_0, gibMag_0, ogeeMag_0, ogeePhase_0))


def bsc1():
    return BaseStationCal((phase_1, tilt_1, curve_1, gibPhase_1, gibMag_1, ogeeMag_1, ogeePhase_1))


obj_qw, obj_qi, obj_qj, obj_qk = sp.symbols('obj_qw,obj_qi,obj_qj,obj_qk')
obj_px, obj_py, obj_pz = sp.symbols('obj_px,obj_py,obj_pz')

lh_qw, lh_qi, lh_qj, lh_qk = sp.symbols('lh_qw,lh_qi,lh_qj,lh_qk')
lh_px, lh_py, lh_pz = sp.symbols('lh_px,lh_py,lh_pz')

sensor_x, sensor_y, sensor_z = sp.symbols('sensor_x,sensor_y,sensor_z')

axis = sp.symbols('axis')

def axis_angle():
    return sp.symbols('aa_x, aa_y, aa_z')

obj_rot = (obj_qw, obj_qi, obj_qj, obj_qk)

axis_angle_mode = False


class SurvivePose(SurviveType):
    def __init__(self, p, r):
        self.Pos = p
        self.Rot = r


class LinmathAxisAnglePose(SurviveType):
    def __init__(self, p, r):
        self.Pos = p
        self.AxisAngleRot = r

    @property
    def Rot(self):
        return self.AxisAngleRot


def obj_p():
    if axis_angle_mode:
        return obj_p_axisangle()
    # return ((obj_px, obj_py, obj_pz), (obj_qw, obj_qi, obj_qj, obj_qk))
    return SurvivePose((obj_px, obj_py, obj_pz), (obj_qw, obj_qi, obj_qj, obj_qk))


def obj_p_axisangle():
    return LinmathAxisAnglePose((obj_px, obj_py, obj_pz), (obj_qi, obj_qj, obj_qk))


def lh_p():
    if axis_angle_mode:
        return lh_p_axisangle()

    return SurvivePose((lh_px, lh_py, lh_pz), (lh_qw, lh_qi, lh_qj, lh_qk))


def lh_p_axisangle():
    return LinmathAxisAnglePose((lh_px, lh_py, lh_pz), (lh_qi, lh_qj, lh_qk))


def sensor_pt():
    return (sensor_x, sensor_y, sensor_z)


def q():
    return (obj_qw, obj_qi, obj_qj, obj_qk)


def quatnormalize(q):
    qw, qi, qj, qk = q
    mag = quatmagnitude(q);
    return [qw / mag, qi / mag, qj / mag, qk / mag]


def axisanglenormalize(axis_angle):
    qi, qj, qk = axis_angle
    mag = axisanglemagnitude(axis_angle)
    return [qi / mag, qj / mag, qk / mag]


def quatmagnitude(q):
    qw, qi, qj, qk = q
    return sqrt(qw * qw + qi * qi + qj * qj + qk * qk)


def quatrotationmatrix(q):
    qw, qi, qj, qk = q
    s = quatmagnitude(q)
    return sp.Matrix(
        [[1 - 2 * s * (qj * qj + qk * qk), 2 * s * (qi * qj - qk * qw), 2 * s * (qi * qk + qj * qw)],
         [2 * s * (qi * qj + qk * qw), 1 - 2 * s * (qi * qi + qk * qk), 2 * s * (qj * qk - qi * qw)],
         [2 * s * (qi * qk - qj * qw), 2 * s * (qj * qk + qi * qw), 1 - 2 * s * (qi * qi + qj * qj)]
         ])


def axisanglemagnitude(axis_angle):
    qw, qi, qj = axis_angle
    return sp.sqrt(qw * qw + qi * qi + qj * qj)


def axisanglerotationmatrix(axis_angle):
    R = axisanglemagnitude(axis_angle)

    x = Piecewise((axis_angle[0] / R, R > 0), (1, True))
    y = Piecewise((axis_angle[1] / R, R > 0), (0, True))
    z = Piecewise((axis_angle[2] / R, R > 0), (0, True))

    csr = sp.cos(R)
    one_minus_csr = (1 - csr)
    snr = sp.sin(R)

    return sp.Matrix(
        [[csr + x * x * (1 - csr), x * y * one_minus_csr - z * snr, x * z * one_minus_csr + y * snr],
         [y * x * one_minus_csr + z * snr, csr + y * y * one_minus_csr, y * z * one_minus_csr - x * snr],
         [z * x * one_minus_csr - y * snr, z * y * one_minus_csr + x * snr, csr + z * z * one_minus_csr]])


def quatrotatevector(q, sensor_pt):
    x, y, z = sensor_pt
    return quatrotationmatrix(q) * sp.Matrix((x, y, z))


def axisanglerotatevector(axis_angle, sensor_pt):
    x, y, z = sensor_pt
    return axisanglerotationmatrix(axis_angle) * sp.Matrix((x, y, z))


def quatgetreciprocal(q):
    return [q[0], -q[1], -q[2], -q[3]]


def apply_axisangle_pose_to_pt(obj_p_axisangle, sensor_pt):
    px, py, pz = obj_p_axisangle.Pos
    return (axisanglerotatevector(obj_p_axisangle.Rot, sensor_pt) + sp.Matrix((px, py, pz)))


def invert_pose(obj_p):
    r = quatgetreciprocal(obj_p.Rot)
    return (-1 * quatrotatevector(r, obj_p.Pos), r)


def axisangle2quat(axis_angle):
    qi, qj, qk = axis_angle
    mag = axisanglemagnitude(axis_angle)
    v = [qi / mag, qj / mag, qk / mag]

    sn = sin(mag / 2.0)
    return quatnormalize([cos(mag / 2.0), sn * v[0], sn * v[1], sn * v[2]])


def axisangle2pose(obj_p_axisangle):
    return obj_p_axisangle.Pos, axisangle2quat(obj_p_axisangle.Rot)


def apply_pose_to_pt(obj_p, sensor_pt):
    px, py, pz = obj_p.Pos
    return quatrotatevector(obj_p.Rot, sensor_pt) + sp.Matrix((px, py, pz))


def sensor_to_world(obj_p, sensor_pt, lh_p):
    if len(obj_p.Rot) == 4:
        return apply_pose_to_pt(lh_p, apply_pose_to_pt(obj_p, sensor_pt))
    return apply_axisangle_pose_to_pt(lh_p, apply_axisangle_pose_to_pt(obj_p, sensor_pt))


def simple_neg(x):
    if isinstance(x, sp.Expr):
        return sp.Mul(x, -1, evaluate=False)
    return -x


generate = [
    apply_axisangle_pose_to_pt,
    apply_pose_to_pt,
    axisangle2pose,
    axisangle2quat,
    axisanglemagnitude,
    axisanglenormalize,
    axisanglerotatevector,
    axisanglerotationmatrix,
    invert_pose,
    quatgetreciprocal,
    quatmagnitude,
    quatnormalize,
    quatrotatevector,
    quatrotationmatrix,
    sensor_to_world
]
