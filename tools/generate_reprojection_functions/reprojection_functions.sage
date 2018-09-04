# -*- python -*-

from sympy.utilities.codegen import codegen
from sympy.printing import print_ccode
from sympy import cse, sqrt, sin, pprint, ccode
import types
import sys

obj_qw,obj_qi,obj_qj,obj_qk=var('obj_qw,obj_qi,obj_qj,obj_qk')
obj_px,obj_py,obj_pz=var('obj_px,obj_py,obj_pz')

lh_qw,lh_qi,lh_qj,lh_qk=var('lh_qw,lh_qi,lh_qj,lh_qk')
lh_px,lh_py,lh_pz=var('lh_px,lh_py,lh_pz')

sensor_x,sensor_y,sensor_z=var('sensor_x,sensor_y,sensor_z')

axis=var('axis')
phase_scale=var('phase_scale')
tilt_scale=var('tilt_scale')
curve_scale=var('curve_scale')
gib_scale=var('gib_scale')

phase_0,phase_1=var('phase_0, phase_1')
tilt_0,tilt_1=var('tilt_0, tilt_1')
curve_0,curve_1=var('curve_0, curve_1')
gibPhase_0,gibPhase_1=var('gibPhase_0, gibPhase_1')
gibMag_0,gibMag_1=var('gibMag_0, gibMag_1')

def quatmagnitude(q):
    qw,qi,qj,qk = q    
    return sqrt(qw*qw+qi*qi+qj*qj+qk*qk)

def quatrotationmatrix(q):
    qw,qi,qj,qk = q
    s = quatmagnitude(q)
    return matrix(SR,
                  [ [ 1 - 2 * s * (qj*qj + qk*qk), 2 * s*(qi*qj - qk*qw), 2*s*(qi*qk + qj*qw)],
                    [ 2*s*(qi*qj + qk*qw), 1 - 2*s*(qi*qi+qk*qk), 2*s*(qj*qk-qi*qw)],
                    [ 2*s*(qi*qk-qj*qw), 2*s*(qj*qk+qi*qw), 1-2*s*(qi*qi+qj*qj)]
                    ])

def quatrotatevector(q, pt):
    qw,qi,qj,qk = q
    x,y,z = pt
    return quatrotationmatrix(q) * vector((x,y,z))

def quatgetreciprocal(q):
    return [ q[0], -q[1], -q[2], -q[3] ]

def apply_pose_to_pt(p, pt):
    px,py,pz = p[0]
    return quatrotatevector(p[1], pt) + vector((px,py,pz))

def invert_pose(p):
    r = quatgetreciprocal(p[1])
    return ( -1 * quatrotatevector(r, p[0]), r)

def reproject_axis(X, Y, Z,
                   phase_scale, phase_cal,
                   tilt_scale, tilt_cal,
                   curve_scale, curve_cal,
                   gib_scale, gibPhase_cal, gibMag_cal):
    y = Y / Z
    ang = atan2(X, Z)

    return ang - phase_scale * phase_cal \
           - tan(tilt_scale * tilt_cal) * y \
           - curve_scale * curve_cal * y * y \
           - gib_scale * sin(gibPhase_cal + ang) * gibMag_cal


def reproject_axis_x(p, pt, lh_p,
                     phase_scale, phase_cal,
                     tilt_scale, tilt_cal,
                     curve_scale, curve_cal,
                     gib_scale, gibPhase_cal, gibMag_cal):

    pt_in_world = apply_pose_to_pt(p, pt)
    XYZ = apply_pose_to_pt( invert_pose(lh_p), pt_in_world)

    return reproject_axis(-XYZ[0], XYZ[1], -XYZ[2],
                          phase_scale, phase_cal,
                          tilt_scale, tilt_cal,
                          curve_scale, curve_cal,
                          gib_scale, gibPhase_cal, gibMag_cal)

def reproject_axis_y(p, pt, lh_p,
                     phase_scale, phase_cal,
                     tilt_scale, tilt_cal,
                     curve_scale, curve_cal,
                     gib_scale, gibPhase_cal, gibMag_cal):
    pt_in_world = apply_pose_to_pt(p, pt)
    XYZ = apply_pose_to_pt( invert_pose(lh_p), pt_in_world)

    return reproject_axis(XYZ[1], -XYZ[0], -XYZ[2],
                          phase_scale, phase_cal,
                          tilt_scale, tilt_cal,
                          curve_scale, curve_cal,
                          gib_scale, gibPhase_cal, gibMag_cal)

def reproject(p, pt, lh_p,
              phase_scale, phase_0, phase_1,
              tilt_scale, tilt_0, tilt_1,
              curve_scale, curve_0, curve_1,
              gib_scale, gibPhase_0, gibPhase_1, gibMag_0, gibMag_1):
    pt_in_world = apply_pose_to_pt(p, pt)
    XYZ = apply_pose_to_pt( invert_pose(lh_p), pt_in_world)

    return vector((
            reproject_axis(-XYZ[0], XYZ[1], -XYZ[2], phase_scale, phase_0, tilt_scale, tilt_0, curve_scale, curve_0, gib_scale, gibPhase_0, gibMag_0),
            reproject_axis(XYZ[1], -XYZ[0], -XYZ[2], phase_scale, phase_1, tilt_scale, tilt_1, curve_scale, curve_1, gib_scale, gibPhase_1, gibMag_1)
        ))

obj_rot = (obj_qw,obj_qi,obj_qj,obj_qk)
obj_p = ((obj_px, obj_py, obj_pz), (obj_qw,obj_qi,obj_qj,obj_qk))

lh_p = ((lh_px, lh_py, lh_pz), (lh_qw,lh_qi,lh_qj,lh_qk))
sensor_pt = (sensor_x,sensor_y,sensor_z)

def flatten_args(bla):
    output = []
    for item in bla:
        output += flatten_args(item) if hasattr (item, "__iter__") else [item]
    return output

def generate_ccode(name, args, expressions):
    flatten = []
    if isinstance(expressions, types.FunctionType):
        print("/** Applying function %s */" % str(expressions))
        expressions = expressions(*args)

    if not hasattr(expressions, "_sympy_"):
        for col in expressions:
            if hasattr(col, '_sympy_'):
                flatten.append(col._sympy_())
            else:
                for cell in col:
                    flatten.append(cell._sympy_())
    else:
        flatten.append(expressions._sympy_())

    cse_output = cse( flatten )
    cnt = 0
    arg_str = lambda (idx, a): ("const FLT *%s" % str(flatten_args(a)[0]).split('_', 1)[0] ) if isinstance(a, tuple) else ("FLT " + str(a))
    print("static inline void gen_%s(FLT* out, %s) {" % (name, ", ".join( map(arg_str, enumerate(args)) )))

    # Unroll struct types
    for idx, a in enumerate(args):
        if isinstance(a, tuple):
            name = str(flatten_args(a)[0]).split('_', 1)[0]
            for v in flatten_args(a):
                print("\tFLT %s = *(%s++);" % (str(v), name))

    for item in cse_output[0]:
        if isinstance(item, tuple):
            print("\tFLT %s = %s;" % (ccode(item[0]), ccode(item[1])))
        else:
            print("/** %s */" % item)
            
    for item in cse_output[1]:            
        print("\t*(out++) = %s;" % ccode(item))
    print "}"
    print ""
    
#print(min_form)

print(" // NOTE: Auto-generated code; see tools/generate_reprojection_functions ")
print("#include <math.h>")

reproject_params = (obj_p, sensor_pt, lh_p, phase_scale, phase_0, phase_1,
                    tilt_scale, tilt_0, tilt_1,
                    curve_scale, curve_0, curve_1,
                    gib_scale, gibPhase_0, gibPhase_1, gibMag_0, gibMag_1)

reproject_axis_params = (obj_p, sensor_pt, lh_p, phase_scale, phase_0,
                    tilt_scale, tilt_0,
                    curve_scale, curve_0,
                    gib_scale, gibPhase_0, gibMag_0)

if len(sys.argv) > 1 and sys.argv[1] == "--full":
    generate_ccode("quat_rotate_vector", [obj_rot, sensor_pt], quatrotatevector)
    generate_ccode("invert_pose", [obj_p], invert_pose)
    generate_ccode("reproject", reproject_params, reproject)
    generate_ccode("apply_pose", [obj_p, sensor_pt], apply_pose_to_pt)

generate_ccode("reproject_jac_obj_p", reproject_params, jacobian(reproject(*reproject_params), (obj_px, obj_py, obj_pz, obj_qw,obj_qi,obj_qj,obj_qk)))
generate_ccode("reproject_axis_x_jac_obj_p", reproject_axis_params , jacobian(reproject_axis_x(*reproject_axis_params ), (obj_px, obj_py, obj_pz, obj_qw,obj_qi,obj_qj,obj_qk) ))
generate_ccode("reproject_axis_y_jac_obj_p", reproject_axis_params , jacobian(reproject_axis_y(*reproject_axis_params ), (obj_px, obj_py, obj_pz, obj_qw,obj_qi,obj_qj,obj_qk) ))
