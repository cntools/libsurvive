import math
from common_math import *
import cnkalman.codegen as cg
from survive_types import *

def reproject_axis(axis_value, other_axis_value, Z, cal):
    # We do this weirdness to only have to calculate atan2(X, Z) and atan2(Y, Z); never atan2(Z, -X) et al
    if isinstance(axis_value, symengine.Mul) and (axis_value.args[1] == -1):
        ang = math.pi / 2. + atan2(axis_value.args[0], Z)
    else:
        ang = math.pi / 2. - atan2(axis_value, Z)

    mag = sqrt(axis_value * axis_value + Z * Z)
    ang -= cal.phase
    asin_arg = cal.tilt * other_axis_value / mag
    ang -= asin(asin_arg)
    ang -= cos(cal.gibpha + ang) * cal.gibmag
    ang += cal.curve * atan2(other_axis_value, Z) * atan2(other_axis_value, Z)

    return ang - math.pi / 2.


def reproject(obj_p, sensor_pt, lh_p, bsd):
    XYZ = sensor_to_world(obj_p, sensor_pt, lh_p)
    cal0, cal1 = bsd
    return Matrix((
        reproject_axis(XYZ[0], XYZ[1], -XYZ[2], cal0),
        reproject_axis(simple_neg(XYZ[1]), XYZ[0], -XYZ[2], cal1)
    ))

@cg.generate_code()
def reproject_xy(cal0 : BaseStationCal, cal1 : BaseStationCal, sensor_pt : list):
    XYZ = sensor_pt
    return Matrix((
        reproject_axis(XYZ[0], XYZ[1], -XYZ[2], cal0),
        reproject_axis(simple_neg(XYZ[1]), XYZ[0], -XYZ[2], cal1)
    ))

@cg.generate_code()
def reproject_axis_x(obj_p : SurvivePose, sensor_pt : list, lh_p : SurvivePose, bsc0 : BaseStationCal):
    XYZ = sensor_to_world(obj_p, sensor_pt, lh_p)
    return reproject_axis(XYZ[0], XYZ[1], -XYZ[2], bsc0)

@cg.generate_code()
def reproject_axis_y(obj_p : SurvivePose, sensor_pt : list, lh_p : SurvivePose, bsc1 : BaseStationCal):
    XYZ = sensor_to_world(obj_p, sensor_pt, lh_p)
    return reproject_axis(simple_neg(XYZ[1]), XYZ[0], -XYZ[2], bsc1)
