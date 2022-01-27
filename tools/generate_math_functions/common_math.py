import symengine as sp
from symengine import sqrt, cos, sin, Piecewise, atan2, acos
import sympy

import math

phase_0, phase_1 = sp.symbols('phase_0, phase_1')
tilt_0, tilt_1 = sp.symbols('tilt_0, tilt_1')
curve_0, curve_1 = sp.symbols('curve_0, curve_1')
gibPhase_0, gibPhase_1 = sp.symbols('gibPhase_0, gibPhase_1')
gibMag_0, gibMag_1 = sp.symbols('gibMag_0, gibMag_1')
ogeePhase_0, ogeePhase_1 = sp.symbols('ogeePhase_0, ogeePhase_1')
ogeeMag_0, ogeeMag_1 = sp.symbols('ogeeMag_0, ogeeMag_1')


def time():
    return sp.symbols('time')

def scale():
    return sp.symbols('scale')


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

def axis_angle2():
    return sp.symbols('aa2_x, aa2_y, aa2_z')


obj_rot = (obj_qw, obj_qi, obj_qj, obj_qk)

axis_angle_mode = False


class SurvivePose(SurviveType):
    def __init__(self, p, r):
        self.Pos = p
        self.Rot = r


def invert_pose_typed(obj_p):
    r = quatgetreciprocal(quatnormalize(obj_p.Rot))
    X,Y,Z = quatrotatevector(r, obj_p.Pos)
    return SurvivePose([-X,-Y,-Z], r)

class LinmathAxisAnglePose(SurviveType):
    def __init__(self, p, r):
        self.Pos = p
        self.AxisAngleRot = r

    @property
    def Rot(self):
        return self.AxisAngleRot

class SurviveKalmanModel(SurviveType):
    def __init__(self, p, v, a, accB, b, imuCorrection, accScale):
        self.Pose = p
        self.Velocity = v
        self.Acc = a
        self.AccScale = accScale
        self.IMUCorrection = imuCorrection
        self.AccBias = accB
        self.GyroBias = b

def gyro_bias():
    return sp.symbols('gbx, gby, gbz')

def imu_correction():
    return sp.symbols('imu_w, imu_x, imu_y, imu_z')
def acc_scale():
    return sp.symbols('acc_scale')

def kalman_model():
    return SurviveKalmanModel(obj_p(), obj_v(), obj_acc(), obj_acc_bias(), gyro_bias(), imu_correction(), acc_scale())

def obj_p():
    if axis_angle_mode:
        return obj_p_axisangle()
    # return ((obj_px, obj_py, obj_pz), (obj_qw, obj_qi, obj_qj, obj_qk))
    return SurvivePose((obj_px, obj_py, obj_pz), (obj_qw, obj_qi, obj_qj, obj_qk))

def rotation_phi(t, q):
    qw, qx, qy, qz = q
    return sympy.MutableDenseMatrix([
        [1, 0, 0, 0, -qx*t/2, -qy*t/2, -qz*t/2],
        [0, 1, 0, 0,  qw*t/2, -qz*t/2,  qy*t/2],
        [0, 0, 1, 0,  qz*t/2,  qw*t/2, -qx*t/2],
        [0, 0, 0, 1, -qy*t/2,  qx*t/2,  qw*t/2],
        [0, 0, 0, 0, 1, 0, 0],
        [0, 0, 0, 0, 0, 1, 0],
        [0, 0, 0, 0, 0, 0, 1]])

def cov(dt, q, sw):
    t = sympy.symbols('t')
    Q = sympy.MutableDenseMatrix([
        [0] * 7,
        [0] * 7,
        [0] * 7,
        [0] * 7,
        [0, 0, 0, 0, sw, 0, 0],
        [0, 0, 0, 0, 0, sw, 0],
        [0, 0, 0, 0, 0, 0, sw],
    ])
    phi = rotation_phi(t, q)
    return sympy.integrate(phi * Q * phi.transpose(), (t, 0, dt))

def obj_p_axisangle():
    return LinmathAxisAnglePose((obj_px, obj_py, obj_pz), (obj_qi, obj_qj, obj_qk))

def obj_v():
    return LinmathAxisAnglePose(sp.symbols('vx, vy, vz'), sp.symbols('avx, avy, avz'))

def obj_acc():
    return sp.symbols('acc_x, acc_y, acc_z')

def obj_acc_bias():
    return sp.symbols('acc_biasx, acc_biasy, acc_biasz')

def lh_p():
    if axis_angle_mode:
        return lh_p_axisangle()

    return SurvivePose((lh_px, lh_py, lh_pz), (lh_qw, lh_qi, lh_qj, lh_qk))


def lh_p_axisangle():
    return LinmathAxisAnglePose((lh_px, lh_py, lh_pz), (lh_qi, lh_qj, lh_qk))


def sensor_pt():
    return (sensor_x, sensor_y, sensor_z)
def pt():
    return sp.symbols('pt_x, pt_y, pt_z')

def cross(sensor_pt, axis_angle):
    a = sensor_pt
    b = axis_angle
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]

def q():
    return (obj_qw, obj_qi, obj_qj, obj_qk)

def q1():
    return q()

def q2():
    return sp.symbols('q1_w, q1_x, q1_y, q1_z')

def quatnormalize(q):
    qw, qi, qj, qk = q
    mag = quatmagnitude(q)
    return [qw / mag, qi / mag, qj / mag, qk / mag]


def axisanglenormalize(axis_angle):
    qi, qj, qk = axis_angle
    mag = axisanglemagnitude(axis_angle)
    return [qi / mag, qj / mag, qk / mag]


def quatmagnitude(q):
    qw, qi, qj, qk = q
    return sqrt(qw * qw + qi * qi + qj * qj + qk * qk + 1e-11)


def quatrotationmatrix(q):
    qw, qi, qj, qk = q
    s = quatmagnitude(q)
    return sp.Matrix(
        [[1 - 2 * s * (qj * qj + qk * qk), 2 * s * (qi * qj - qk * qw), 2 * s * (qi * qk + qj * qw)],
         [2 * s * (qi * qj + qk * qw), 1 - 2 * s * (qi * qi + qk * qk), 2 * s * (qj * qk - qi * qw)],
         [2 * s * (qi * qk - qj * qw), 2 * s * (qj * qk + qi * qw), 1 - 2 * s * (qi * qi + qj * qj)]
         ])


def quatrotateabout(q1, q2):
    return sp.Matrix([(q1[0] * q2[0]) - (q1[1] * q2[1]) - (q1[2] * q2[2]) - (q1[3] * q2[3]),
            (q1[0] * q2[1]) + (q1[1] * q2[0]) + (q1[2] * q2[3]) - (q1[3] * q2[2]),
            (q1[0] * q2[2]) - (q1[1] * q2[3]) + (q1[2] * q2[0]) + (q1[3] * q2[1]),
            (q1[0] * q2[3]) + (q1[1] * q2[2]) - (q1[2] * q2[1]) + (q1[3] * q2[0])])

def quatrotatevector2(q, sensor_pt):
    x, y, z = sensor_pt
    return quatrotationmatrix(q) * sp.Matrix((x, y, z))

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

def quatrotatevector3(q, sensor_pt):
    x, y, z = sensor_pt
    pc = [0, x, y, z]
    qc = quatgetreciprocal(q)
    return quatrotateabout(quatrotateabout(q, pc), qc)[1:]
    #return quatrotationmatrix(q) * sp.Matrix((x, y, z))

def quatfind(q1, q2):
    return quatrotateabout(q2, quatgetreciprocal(q1))

def axisanglemagnitude(axis_angle):
    qw, qi, qj = axis_angle
    mag = qw * qw + qi * qi + qj * qj
    return sp.sqrt(mag + 1e-10)

def dot3d(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

def cross3d(a, b):
    return sp.Matrix([
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ])


def axisanglecompose(axis_angle, axis_angle2):
    a = axisanglemagnitude(axis_angle)
    ah = axisanglenormalize(axis_angle)

    b = axisanglemagnitude(axis_angle2)
    bh = axisanglenormalize(axis_angle2)

    sina = sin(a / 2)
    asina = [ah[0] * sina, ah[1] * sina, ah[2] * sina]

    sinb = sin(b / 2)
    bsinb = [bh[0] * sinb, bh[1] * sinb, bh[2] * sinb]

    c = 2 * acos(cos(a / 2)*cos(b / 2) - dot3d(asina, bsinb))
    d = axisanglenormalize(cos(a / 2) * sp.Matrix(bsinb) + cos(b / 2) * sp.Matrix(asina) + cross3d(asina, bsinb))
    return sp.Matrix([
            c * d[0],
            c * d[1],
            c * d[2]
        ])

def axisanglerotationmatrix(axis_angle):
    R = axisanglemagnitude(axis_angle)

    x = axis_angle[0] / R
    y = axis_angle[1] / R
    z = axis_angle[2] / R

    csr = sp.cos(R)
    one_minus_csr = (1 - csr)
    snr = sp.sin(R)

    return sp.Matrix(
        [[csr + x * x * (1 - csr), x * y * one_minus_csr - z * snr, x * z * one_minus_csr + y * snr],
         [y * x * one_minus_csr + z * snr, csr + y * y * one_minus_csr, y * z * one_minus_csr - x * snr],
         [z * x * one_minus_csr - y * snr, z * y * one_minus_csr + x * snr, csr + z * z * one_minus_csr]])

def axisanglerotatevector(axis_angle, sensor_pt):
    x, y, z = sensor_pt
    return axisanglerotationmatrix(axis_angle) * sp.Matrix((x, y, z))

def obj2world_aa_up_err(axis_angle, sensor_pt):
    out = axisanglerotatevector(axis_angle, sensor_pt)
    return 1 - out[2]

def obj2world_up_err(q1, sensor_pt):
    out = quatrotatevector(q1, sensor_pt)
    return 1 - out[2]

def world2lh_aa_up_err(axis_angle, sensor_pt):
    [ax,ay,az] = axis_angle
    out = axisanglerotatevector([-ax,-ay,-az], sensor_pt)
    return 1 - out[2]

def world2lh_up_err(q1, sensor_pt):
    out = quatrotatevector(q1, sensor_pt)
    return 1 - out[2]

def invertaxisanglerotatevector(axis_angle, sensor_pt):
    x, y, z = sensor_pt
    ax, ay, az = axis_angle
    return axisanglerotationmatrix([-ax,-ay,-az]) * sp.Matrix((x, y, z))


def quatgetreciprocal(q):
    return [q[0], -q[1], -q[2], -q[3]]

def apply_axisangle_pose_to_pt(obj_p_axisangle, sensor_pt):
    px, py, pz = obj_p_axisangle.Pos
    return (axisanglerotatevector(obj_p_axisangle.Rot, sensor_pt) + sp.Matrix((px, py, pz)))


def invert_pose(obj_p):
    r = quatgetreciprocal(quatnormalize(obj_p.Rot))
    X,Y,Z = quatrotatevector(r, obj_p.Pos)
    return sp.Matrix([-X,-Y,-Z, *r])

def quat2axisangle(q):
    qw, qi, qj, qk = q
    mag = sqrt(qi*qi+qj*qj+qk*qk + 1e-10)
    angle = 2 * atan2(mag, q[0])
    return q[1] * angle / mag,\
           q[2] * angle / mag,\
           q[3] * angle / mag

def axisangle2quat(axis_angle):
    mag = axisanglemagnitude(axis_angle)

    x = axis_angle[0] / mag
    y = axis_angle[1] / mag
    z = axis_angle[2] / mag

    sn = sin(mag / 2.0)
    return quatnormalize([cos(mag / 2.0), sn * x, sn * y, sn * z])


def axisangle2pose(obj_p_axisangle):
    return *obj_p_axisangle.Pos, *axisangle2quat(obj_p_axisangle.Rot)

def add3d(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]

def apply_pose_to_pt(obj_p, sensor_pt):
    px, py, pz = obj_p.Pos
    #return quatrotatevector(obj_p.Rot, sensor_pt) + sp.Matrix((px, py, pz))
    return add3d(quatrotatevector(obj_p.Rot, sensor_pt), obj_p.Pos)


def sensor_to_world(obj_p, sensor_pt, lh_p):
    if len(obj_p.Rot) == 4:
        return apply_pose_to_pt(lh_p, apply_pose_to_pt(obj_p, sensor_pt))
    return apply_axisangle_pose_to_pt(lh_p, apply_axisangle_pose_to_pt(obj_p, sensor_pt))

def scale_sensor_pt(sensor_pt, obj_p, scale):
    sensor_pt = apply_pose_to_pt(obj_p, sensor_pt)
    sensor_pt = [sensor_pt[0] * scale, sensor_pt[1] * scale, sensor_pt[2] * scale]
    return apply_pose_to_pt(invert_pose_typed(obj_p), sensor_pt)

def apply_ang_velocity(axis_angle, time, q):
    qi, qj, qk = axis_angle
    q1 = axisangle2quat((qi * time, qj * time, qk * time))
    if len(q) == 3:
        return quatrotateabout(q1, axisangle2quat(q))
    return quatrotateabout(q1, q)

def apply_ang_velocity_aa(axis_angle, time, axis_angle2):
    qi, qj, qk = axis_angle
    q1 = (qi * time, qj * time, qk * time)
    #q1 = (qi * time, qj * time, qk * time)
    #return axisanglerotatevector(q1, axis_angle2)
    return axisanglecompose(q1, axis_angle2)

def apply_kinematics(pos, time, vel, acc=None):
    if acc is None:
        acc = [0, 0, 0]

    return [
        pos[0] + time * vel[0] + acc[0] / 2 * time * time,
        pos[1] + time * vel[1] + acc[1] / 2 * time * time,
        pos[2] + time * vel[2] + acc[2] / 2 * time * time,
    ]

def simple_neg(x):
    if isinstance(x, sp.Expr):
        return sp.Mul(x, -1, evaluate=False)
    return -x


def imu_rot():
    return (*q(), *axis_angle())


def imu_rot_aa():
    return (*axis_angle(), *(lh_qi, lh_qj, lh_qk))

def up_in_obj():
    return list(sp.symbols('obj_up_x, obj_up_y, obj_up_z'))


generate = [
    apply_axisangle_pose_to_pt,
    apply_pose_to_pt,
    #axisangle2pose,
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
    quatrotatevector2,
    quatrotationmatrix,
    quat2axisangle,
    sensor_to_world,
    cross,
    obj2world_aa_up_err,
    obj2world_up_err,
    world2lh_aa_up_err,
    world2lh_up_err,
    apply_ang_velocity,
    apply_ang_velocity_aa,

    axisanglecompose,
    scale_sensor_pt
]
