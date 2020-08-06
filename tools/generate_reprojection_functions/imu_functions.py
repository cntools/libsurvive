import sys

import symengine as sp
import common_math
from codegen import *

def imu_rot_f(time, imu_rot):
    rot = imu_rot[0:4]
    rotv = imu_rot[4:7]

    return sp.Matrix([*apply_ang_velocity(rotv, time, rot), *rotv ])

def imu_rot_f_aa(time, imu_rot_aa):
    rot = imu_rot_aa[0:3]
    rotv = imu_rot_aa[3:6]
    q = apply_ang_velocity(rotv, time, rot)
    ret = (*quat2axisangle(q), *rotv)
    return sp.Matrix(ret)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--aux":
        pass
    else:
        print("#pragma once")
        print("#include \"common.h\"")

        for f in [imu_rot_f, imu_rot_f_aa]:
            generate_ccode(f)
            j = generate_jacobians(f, transpose=True)
            #print("// ", j)

