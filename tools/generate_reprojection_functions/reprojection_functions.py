# -*- python -*-

from sympy.utilities.codegen import codegen
from sympy.printing import print_ccode
from sympy import cse, sqrt, pprint, ccode,atan2,tan,asin,cos
import math
#from sympy import *
import sympy as sp
import types
import sys

obj_qw,obj_qi,obj_qj,obj_qk=sp.symbols('obj_qw,obj_qi,obj_qj,obj_qk')
obj_px,obj_py,obj_pz=sp.symbols('obj_px,obj_py,obj_pz')

lh_qw,lh_qi,lh_qj,lh_qk=sp.symbols('lh_qw,lh_qi,lh_qj,lh_qk')
lh_px,lh_py,lh_pz=sp.symbols('lh_px,lh_py,lh_pz')

sensor_x,sensor_y,sensor_z=sp.symbols('sensor_x,sensor_y,sensor_z')

axis=sp.symbols('axis')

phase_0,phase_1=sp.symbols('phase_0, phase_1')
tilt_0,tilt_1=sp.symbols('tilt_0, tilt_1')
curve_0,curve_1=sp.symbols('curve_0, curve_1')
gibPhase_0,gibPhase_1=sp.symbols('gibPhase_0, gibPhase_1')
gibMag_0,gibMag_1=sp.symbols('gibMag_0, gibMag_1')

def quatnormalize(q):
    qw,qi,qj,qk = q
    mag = quatmagnitude(q);
    return [qw/mag, qi/mag, qj/mag, qk / mag]

def axisanglenormalize(a):
    qi,qj,qk = a
    mag = axisanglemagnitude(a);
    return [qi/mag, qj/mag, qk / mag]


def quatmagnitude(q):
    qw,qi,qj,qk = q
    return sqrt(qw*qw+qi*qi+qj*qj+qk*qk)

def quatrotationmatrix(q):
    qw,qi,qj,qk = q
    s = quatmagnitude(q)
    return sp.Matrix(
                  [ [ 1 - 2 * s * (qj*qj + qk*qk), 2 * s*(qi*qj - qk*qw), 2*s*(qi*qk + qj*qw)],
                    [ 2*s*(qi*qj + qk*qw), 1 - 2*s*(qi*qi+qk*qk), 2*s*(qj*qk-qi*qw)],
                    [ 2*s*(qi*qk-qj*qw), 2*s*(qj*qk+qi*qw), 1-2*s*(qi*qi+qj*qj)]
                    ])


def axisanglemagnitude(q):
    qw,qi,qj = q
    return sp.sqrt(qw*qw+qi*qi+qj*qj)

def axisanglerotationmatrix(a):
    R=axisanglemagnitude(a)
    x,y,z=[a[0]/R,a[1]/R,a[2]/R]
    csr=sp.cos(R)
    one_minus_csr=(1-csr)
    snr=sp.sin(R)
    
    return sp.Matrix(
                  [ [csr+x*x*(1-csr), x * y *one_minus_csr-z*snr,x*z*one_minus_csr+y*snr],
                    [y*x*one_minus_csr+z*snr, csr+y*y*one_minus_csr,y*z*one_minus_csr-x*snr],
                    [z*x*one_minus_csr-y*snr, z*y*one_minus_csr+x*snr,csr+z*z*one_minus_csr] ])
    
def quatrotatevector(q, pt):
    x,y,z = pt
    return quatrotationmatrix(q) * sp.Matrix((x,y,z))

def axisanglerotatevector(q, pt):
    x,y,z = pt
    return (axisanglerotationmatrix(q) * sp.Matrix((x,y,z)))

def quatgetreciprocal(q):
    return [ q[0], -q[1], -q[2], -q[3] ]

def apply_pose_to_pt(p, pt):
    px,py,pz = p[0]
    return quatrotatevector(p[1], pt) + sp.Matrix((px,py,pz))

def apply_axisangle_pose_to_pt(p, pt):
    px,py,pz = p[0]
    return (axisanglerotatevector(p[1], pt) + sp.Matrix((px,py,pz)))

def invert_pose(p):
    r = quatgetreciprocal(p[1])
    return ( -1 * quatrotatevector(r, p[0]), r)

def axisangle2quat(axis):
    qi,qj,qk = axis
    mag = axisanglemagnitude(axis);
    v = [qi/mag, qj/mag, qk / mag]

    sn = sin(mag / 2.0);
    return quatnormalize([cos(mag / 2.0), sn * v[0], sn * v[1], sn * v[2] ]);

def axisangle2pose(p):
    return p[0], axisangle2quat(p[1])

def reproject_axis(axis_value, other_axis_value, Z,
                       phase,
                       tilt,
                       curve,
                       gibPhase, gibMag):
    ang = atan2(Z, axis_value)

    mag = sqrt(axis_value * axis_value  + Z * Z);
    ang -= phase;
    asin_arg = tan(tilt) * other_axis_value / mag;
    ang -= asin(asin_arg);
    ang -= cos(gibPhase + ang) * gibMag;
    ang += curve * atan2(other_axis_value, Z) * atan2(other_axis_value, Z);

    return ang - math.pi / 2.

def reproject_axis_x(p, pt, lh_p,
                     phase_cal,
                     tilt_cal,
                     curve_cal,
                     gibPhase_cal, gibMag_cal):
    XYZ = apply_pose_to_pt( lh_p, apply_pose_to_pt(p, pt) )

    return reproject_axis(XYZ[0], XYZ[1], -XYZ[2],
                          phase_cal,
                          tilt_cal,
                          curve_cal,
                          gibPhase_cal, gibMag_cal)

def reproject_axis_y(p, pt, lh_p,
                     phase_cal,
                     tilt_cal,
                     curve_cal,
                     gibPhase_cal, gibMag_cal):
    XYZ = apply_pose_to_pt( lh_p, apply_pose_to_pt(p, pt) )

    return reproject_axis(-XYZ[1], XYZ[0], -XYZ[2],
                          phase_cal,
                          tilt_cal,
                          curve_cal,
                          gibPhase_cal, gibMag_cal)

def reproject(p, pt, lh_p,
              phase_0, phase_1,
              tilt_0, tilt_1,
              curve_0, curve_1,
              gibPhase_0, gibPhase_1, gibMag_0, gibMag_1):
    XYZ = apply_pose_to_pt( lh_p, apply_pose_to_pt(p, pt) )

    return sp.Matrix((
            reproject_axis(XYZ[0], XYZ[1], -XYZ[2], phase_0, tilt_0, curve_0, gibPhase_0, gibMag_0),
            reproject_axis(-XYZ[1], XYZ[0], -XYZ[2], phase_1, tilt_1, curve_1, gibPhase_1, gibMag_1)
        ))


def reproject_axis_x_axisangle(p_axisangle, pt, lh_p_axisangle,
                     phase_cal,
                     tilt_cal,
                     curve_cal,
                     gibPhase_cal, gibMag_cal):
    XYZ = apply_axisangle_pose_to_pt(lh_p_axisangle, apply_axisangle_pose_to_pt(p_axisangle, pt))

    return reproject_axis(XYZ[0], XYZ[1], -XYZ[2],
                          phase_cal,
                          tilt_cal,
                          curve_cal,
                          gibPhase_cal, gibMag_cal)

def reproject_axis_y_axisangle(p_axisangle, pt, lh_p_axisangle,
                     phase_cal,
                     tilt_cal,
                     curve_cal,
                     gibPhase_cal, gibMag_cal):
    XYZ = apply_axisangle_pose_to_pt(lh_p_axisangle, apply_axisangle_pose_to_pt(p_axisangle, pt))

    return reproject_axis(-XYZ[1], XYZ[0], -XYZ[2],
                          phase_cal,
                          tilt_cal,
                          curve_cal,
                          gibPhase_cal, gibMag_cal)

def reproject_axisangle(p_axisangle, pt, lh_p_axisangle,
              phase_0, phase_1,
              tilt_0, tilt_1,
              curve_0, curve_1,
              gibPhase_0, gibPhase_1, gibMag_0, gibMag_1):
    XYZ = apply_axisangle_pose_to_pt(lh_p_axisangle, apply_axisangle_pose_to_pt(p_axisangle, pt))

    return sp.Matrix((
            reproject_axis(XYZ[0], XYZ[1], -XYZ[2], phase_0, tilt_0, curve_0, gibPhase_0, gibMag_0),
            reproject_axis(-XYZ[1], XYZ[0], -XYZ[2], phase_1, tilt_1, curve_1, gibPhase_1, gibMag_1)
        ))

obj_rot = (obj_qw,obj_qi,obj_qj,obj_qk)
obj_p = ((obj_px, obj_py, obj_pz), (obj_qw,obj_qi,obj_qj,obj_qk))

obj_p_axisangle = ((obj_px, obj_py, obj_pz), (obj_qi,obj_qj,obj_qk))

lh_p = ((lh_px, lh_py, lh_pz), (lh_qw,lh_qi,lh_qj,lh_qk))
lh_p_axisangle = ((lh_px, lh_py, lh_pz), (lh_qi,lh_qj,lh_qk))

sensor_pt = (sensor_x,sensor_y,sensor_z)

def flatten_args(bla):
    output = []
    for item in bla:
        output += flatten_args(item) if hasattr (item, "__iter__") else [item]
    return output

def try_simplify(expr):
    #if hasattr(expr, "simplify_full"):
    #print("")    
    #print(expr)
    return expr
    #return expr._sympy_()

def make_sympy(expressions):
    flatten=[]
    if hasattr(expressions, "atoms"):
        return [expressions]

    if not hasattr(expressions, "_sympy_"):
        for col in expressions:
            if hasattr(col, '_sympy_'):
                flatten.append(try_simplify(col))
            else:
                for cell in col:
                    flatten.append(try_simplify(cell))
    else:
        flatten.append(try_simplify(expressions))
    return flatten
        
def generate_ccode(name, args, expressions):
    sys.stderr.write("Writing out %s\n" % name)

    if isinstance(expressions, types.FunctionType):
        print("/** Applying function %s */" % str(expressions))
        expressions = expressions(*args)

    flatten = make_sympy(expressions)

    sys.stderr.write("Running CSE\n")
    cse_output = sp.cse( flatten, order='none' )
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
            print("\tFLT %s = %s;" % (sp.ccode(item[0]), sp.ccode(item[1])))
        else:
            print("/** %s */" % item)
            
    for item in cse_output[1]:
        if hasattr (item, "tolist"):
            for item1 in sum(item.tolist(),[]):
                print("\t*(out++) = %s;" % sp.ccode(item1))
        else:
            print("\t*(out++) = %s;" % sp.ccode(item))
    print "}"
    print ""
    
#print(min_form)

print(" // NOTE: Auto-generated code; see tools/generate_reprojection_functions ")
print("#include <math.h>")
print("#include <linmath.h>")
print("#include <complex.h>")
print("double __safe_asin(double x) { return asin( linmath_max(-1., linmath_min(1., x)) ); }")
print("double __safe_pow(double x, double y) { return creal(cpow(x, y));}")
print("#define pow __safe_pow")
print("#define asin __safe_asin")
reproject_params = (obj_p, sensor_pt, lh_p, phase_0, phase_1,
                    tilt_0, tilt_1,
                    curve_0, curve_1,
                    gibPhase_0, gibPhase_1, gibMag_0, gibMag_1)

reproject_axisangle_params = (obj_p_axisangle, sensor_pt, lh_p_axisangle, phase_0, phase_1,
                    tilt_0, tilt_1,
                    curve_0, curve_1,
                    gibPhase_0, gibPhase_1, gibMag_0, gibMag_1)

reproject_axis_params = (obj_p, sensor_pt, lh_p, phase_0,
                    tilt_0,
                    curve_0,
                    gibPhase_0, gibMag_0)


reproject_axisangle_axis_params = (obj_p_axisangle, sensor_pt, lh_p_axisangle, phase_0,
                         tilt_0,
                         curve_0,
                         gibPhase_0, gibMag_0)

if len(sys.argv) > 1 and sys.argv[1] == "--full":
    generate_ccode("apply_axisangle_pose_to_pt", [obj_p_axisangle, sensor_pt], apply_axisangle_pose_to_pt)
    generate_ccode("reproject_axisangle", reproject_axisangle_params, reproject_axisangle)        
    generate_ccode("quat_rotate_vector", [obj_rot, sensor_pt], quatrotatevector)
    generate_ccode("invert_pose", [obj_p], invert_pose)
    generate_ccode("reproject", reproject_params, reproject)
    generate_ccode("apply_pose", [obj_p, sensor_pt], apply_pose_to_pt)

def jacobian(v, of):
    if hasattr(v, 'jacobian'):
        return v.jacobian(of)
    return sp.Matrix([v]).jacobian(of)
    
generate_ccode("reproject_axis_x_jac_obj_p_axisangle", reproject_axisangle_axis_params ,
               jacobian(reproject_axis_x_axisangle(*reproject_axisangle_axis_params), ( (obj_px, obj_py, obj_pz, obj_qi,obj_qj,obj_qk) )))
generate_ccode("reproject_axis_y_jac_obj_p_axisangle", reproject_axisangle_axis_params , jacobian(reproject_axis_y_axisangle(*reproject_axisangle_axis_params ), (obj_px, obj_py, obj_pz, obj_qi,obj_qj,obj_qk) ))
generate_ccode("reproject_jac_obj_p_axisangle", reproject_axisangle_params, jacobian(reproject_axisangle(*reproject_axisangle_params), (obj_px, obj_py, obj_pz, obj_qi,obj_qj,obj_qk)))

generate_ccode("reproject_jac_obj_p", reproject_params, jacobian(reproject(*reproject_params), (obj_px, obj_py, obj_pz, obj_qw,obj_qi,obj_qj,obj_qk)))
generate_ccode("reproject_axis_x_jac_obj_p", reproject_axis_params , jacobian(reproject_axis_x(*reproject_axis_params ), (obj_px, obj_py, obj_pz, obj_qw,obj_qi,obj_qj,obj_qk) ))
generate_ccode("reproject_axis_y_jac_obj_p", reproject_axis_params , jacobian(reproject_axis_y(*reproject_axis_params ), (obj_px, obj_py, obj_pz, obj_qw,obj_qi,obj_qj,obj_qk) ))
