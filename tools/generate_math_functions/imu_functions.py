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

def quatrotate_small(q, axis_angle):
    a,b,c = axis_angle
    return quatrotateabout(q, [1, a/2,b/2,c/2])

def imu_predict_up(kalman_model):
    g = 9.80665
    acc_scale = kalman_model.AccScale
    acc_bias = kalman_model.AccBias
    G = [ kalman_model.Acc[0]/g, kalman_model.Acc[1]/g, 1 + kalman_model.Acc[2]/g]
    rot = quatrotateabout(quatgetreciprocal(quatnormalize(kalman_model.Pose.Rot)), quatnormalize(kalman_model.IMUCorrection))
    GinObj = quatrotatevector(rot, G)
    return [
        acc_scale * GinObj[0] + acc_bias[0],
        acc_scale * GinObj[1] + acc_bias[1],
        acc_scale * GinObj[2] + acc_bias[2]
    ]

def imu_predict_gyro(kalman_model):
    rot = quatrotateabout(quatgetreciprocal(quatnormalize(kalman_model.Pose.Rot)), quatnormalize(kalman_model.IMUCorrection))
    rotv = quatrotatevector(rot, kalman_model.Velocity.Rot)
    return [rotv[0] + kalman_model.GyroBias[0],
            rotv[1] + kalman_model.GyroBias[1],
            rotv[2] + kalman_model.GyroBias[2]
    ]


def imu_predict(kalman_model):
    return [*imu_predict_up(kalman_model), *imu_predict_gyro(kalman_model)]

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

def kalman_model_predict(t, kalman_model):
    obj_p = kalman_model.Pose
    obj_v = kalman_model.Velocity
    obj_acc = kalman_model.Acc

    pos = obj_p.Pos
    vpos = obj_v.Pos
    new_pos = [
        pos[0] + vpos[0] * t + obj_acc[0] * t * t / 2,
        pos[1] + vpos[1] * t + obj_acc[1] * t * t / 2,
        pos[2] + vpos[2] * t + obj_acc[2] * t * t / 2
    ]
    new_rot = apply_ang_velocity(obj_v.Rot, t, obj_p.Rot)

    new_vpos = [
        vpos[0] + obj_acc[0] * t,
        vpos[1] + obj_acc[1] * t,
        vpos[2] + obj_acc[2] * t,
    ]
    return [ *new_pos, *new_rot, *new_vpos, *obj_v.Rot, *obj_acc,
             kalman_model.AccScale, *kalman_model.IMUCorrection,
             *kalman_model.AccBias, *kalman_model.GyroBias ]

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--aux":
        pass
    else:
        print("#pragma once")
        print("#include \"common.h\"")
        print("// clang-format off")

        generate_code_and_jacobians(imu_rot_f, transpose=True)
        generate_code_and_jacobians(kalman_model_predict)
        #generate_code_and_jacobians(invert_pose)
        for f in [imu_rot_f_aa, imu_correct_up, imu_predict_up, quatrotateabout,
                  imu_predict, imu_predict_gyro, quatfind, quatrotate_small]:

            if f in common_math.generate:
                sys.stderr.write(f, "!!!!!\n")
            generate_ccode(f)
            j = generate_jacobians(f, transpose=f == imu_rot_f)

