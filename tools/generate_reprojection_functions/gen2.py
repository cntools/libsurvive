from common_math import *

import math

from symengine import sqrt, cos, sin, Piecewise, atan2, tan, asin

def calc_cal_series(s):
    f = [-8.0108022e-06, 0.0028679863, 5.3685255000000001e-06, 0.0076069798000000001, 0, 0]

    m = f[0]
    a = 0
    for i in range(0, 6):
        a = a * s + m
        m = m * s + f[i]
    return m, a


def reproject_axis_gen2(X, Y, Z, axis, cal):
    #(phase_cal, tilt_cal, curve_cal, gibPhase_cal, gibMag_cal, ogeePhase_cal, ogeeMag_cal) = cal

    B = atan2(Z, X)

    Ydeg = cal.tilt + (-1 if axis else 1) * math.pi / 6.
    tanA = tan(Ydeg)
    normXZ = sqrt(X * X + Z * Z)

    asinArg = tanA * Y / normXZ

    sinYdeg = sin(Ydeg)
    cosYdeg = cos(Ydeg)

    sinPart = sin(B - asin(asinArg) + cal.ogeephase) * cal.ogeemag

    normXYZ = sqrt(X * X + Y * Y + Z * Z)

    modAsinArg = Y / normXYZ / cosYdeg

    asinOut = asin(modAsinArg)

    mod, acc = calc_cal_series(asinOut)

    BcalCurved = sinPart + cal.curve
    asinArg2 = asinArg + mod * BcalCurved / (cosYdeg - acc * BcalCurved * sinYdeg)

    asinOut2 = asin(asinArg2)
    sinOut2 = sin(B - asinOut2 + cal.gibpha)

    return B - asinOut2 + sinOut2 * cal.gibmag - cal.phase - math.pi / 2.

def reproject_axis_x_gen2(obj_p, sensor_pt, lh_p, bsc0):
    XYZ = sensor_to_world(obj_p, sensor_pt, lh_p)
    return reproject_axis_gen2(XYZ[0], XYZ[1], -XYZ[2], 0, bsc0)

def reproject_axis_y_gen2(obj_p, sensor_pt, lh_p, bsc1):
    XYZ = sensor_to_world(obj_p, sensor_pt, lh_p)
    return reproject_axis_gen2(XYZ[0], XYZ[1], -XYZ[2], 1, bsc1)


def reproject_gen2(obj_p, sensor_pt, lh_p, bsd):
    (cal0, cal1) = bsd
    return sp.Matrix((
        reproject_axis_x_gen2(obj_p, sensor_pt, lh_p, cal0),
        reproject_axis_y_gen2(obj_p, sensor_pt, lh_p, cal1)
    ))


generate = [
    reproject_gen2,
    reproject_axis_x_gen2,
    reproject_axis_y_gen2,
]