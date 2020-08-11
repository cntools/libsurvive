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

def imu_predict_up(imu_rot):
    G = [ 0, 0, 1]
    rot = quatgetreciprocal(quatnormalize(imu_rot[0:4]))
    return quatrotatevector(rot, G)

def imu_predict_gyro(imu_rot):
    rot = quatgetreciprocal(quatnormalize(imu_rot[0:4]))
    rotv = imu_rot[4:7]
    return quatrotatevector(rot, rotv)

def imu_predict(imu_rot):
    return [*imu_predict_up(imu_rot), *imu_predict_gyro(imu_rot)]

def imu_correct_up(mu, imu_rot, up_in_obj):
    rot = imu_rot[0:4]
    rotv = imu_rot[4:7]

    up = quatrotatevector(rot, up_in_obj)

    G = [ 0, 0, 1]
    N = cross(up, G)
    angle = atan2(sqrt(up[0] * up[0] + up[1] * up[1]), up[2])
    for i in range(3):
        N[i] = N[i] * angle * mu / 2.
    qc = axisangle2quat(N)
    return sp.Matrix([*quatrotateabout(qc, rot), *rotv])

import numdifftools
import numpy as np

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--aux":
        pass
    else:
        print("#pragma once")
        print("#include \"common.h\"")
        print("// clang-format off")
        def f(h):
            print(h, +0.680413 - h)
            q1 = np.array([h[0],	+0.164668,	+0.708549,	-0.088767,	+0.898978,	+0.182254,	+0.002019,	+0.398268])
            print(q1)
            rtn = np.array(list(map(float, quatrotateabout(q1[0:4],q1[4:8]))))
            print(rtn)
            return rtn

        #print(numdifftools.Jacobian(f)(+0.680413))

        for f in [quatrotatevector, imu_rot_f, imu_rot_f_aa, imu_correct_up, imu_predict_up, quatrotateabout, imu_predict, imu_predict_gyro]:
            generate_ccode(f)
            j = generate_jacobians(f, transpose= f == imu_rot_f)


            ##print(j)

