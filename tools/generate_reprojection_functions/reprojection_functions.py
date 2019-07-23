# -*- python -*-

import math
import sys
import types

import sympy as sp
from sympy import sqrt, atan2, tan, asin, cos, evaluate, Pow

obj_qw, obj_qi, obj_qj, obj_qk = sp.symbols('obj_qw,obj_qi,obj_qj,obj_qk')
obj_px, obj_py, obj_pz = sp.symbols('obj_px,obj_py,obj_pz')

lh_qw, lh_qi, lh_qj, lh_qk = sp.symbols('lh_qw,lh_qi,lh_qj,lh_qk')
lh_px, lh_py, lh_pz = sp.symbols('lh_px,lh_py,lh_pz')

sensor_x, sensor_y, sensor_z = sp.symbols('sensor_x,sensor_y,sensor_z')

axis = sp.symbols('axis')

phase_0, phase_1 = sp.symbols('phase_0, phase_1')
tilt_0, tilt_1 = sp.symbols('tilt_0, tilt_1')
curve_0, curve_1 = sp.symbols('curve_0, curve_1')
gibPhase_0, gibPhase_1 = sp.symbols('gibPhase_0, gibPhase_1')
gibMag_0, gibMag_1 = sp.symbols('gibMag_0, gibMag_1')


def quatnormalize(q):
    qw, qi, qj, qk = q
    mag = quatmagnitude(q);
    return [qw / mag, qi / mag, qj / mag, qk / mag]


def axisanglenormalize(a):
    qi, qj, qk = a
    mag = axisanglemagnitude(a);
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


def axisanglemagnitude(q):
    qw, qi, qj = q
    return sp.sqrt(qw * qw + qi * qi + qj * qj)


def axisanglerotationmatrix(a):
    R = axisanglemagnitude(a)
    x, y, z = [a[0] / R, a[1] / R, a[2] / R]
    csr = sp.cos(R)
    one_minus_csr = (1 - csr)
    snr = sp.sin(R)

    return sp.Matrix(
        [[csr + x * x * (1 - csr), x * y * one_minus_csr - z * snr, x * z * one_minus_csr + y * snr],
         [y * x * one_minus_csr + z * snr, csr + y * y * one_minus_csr, y * z * one_minus_csr - x * snr],
         [z * x * one_minus_csr - y * snr, z * y * one_minus_csr + x * snr, csr + z * z * one_minus_csr]])


def quatrotatevector(q, pt):
    x, y, z = pt
    return quatrotationmatrix(q) * sp.Matrix((x, y, z))


def axisanglerotatevector(q, pt):
    x, y, z = pt
    return axisanglerotationmatrix(q) * sp.Matrix((x, y, z))


def quatgetreciprocal(q):
    return [q[0], -q[1], -q[2], -q[3]]


def apply_pose_to_pt(p, pt):
    px, py, pz = p[0]
    return quatrotatevector(p[1], pt) + sp.Matrix((px, py, pz))


def apply_axisangle_pose_to_pt(p, pt):
    px, py, pz = p[0]
    return (axisanglerotatevector(p[1], pt) + sp.Matrix((px, py, pz)))


def invert_pose(p):
    r = quatgetreciprocal(p[1])
    return (-1 * quatrotatevector(r, p[0]), r)


def axisangle2quat(axis):
    qi, qj, qk = axis
    mag = axisanglemagnitude(axis);
    v = [qi / mag, qj / mag, qk / mag]

    sn = sin(mag / 2.0);
    return quatnormalize([cos(mag / 2.0), sn * v[0], sn * v[1], sn * v[2]]);


def axisangle2pose(p):
    return p[0], axisangle2quat(p[1])


def reproject_axis(axis_value, other_axis_value, Z,
                   phase,
                   tilt,
                   curve,
                   gibPhase, gibMag):

    # We do this weirdness to only have to calculate atan2(X, Z) and atan2(Y, Z); never atan2(Z, -X) et al
    if isinstance(axis_value, sp.Mul) and (axis_value.args[1] == -1):
        ang = math.pi / 2. + atan2(axis_value.args[0], Z)
    else:
        ang = math.pi / 2. - atan2(axis_value, Z)

    mag = sqrt(axis_value * axis_value + Z * Z)
    ang -= phase
    asin_arg = tilt * other_axis_value / mag
    ang -= asin(asin_arg)
    ang -= cos(gibPhase + ang) * gibMag
    ang += curve * atan2(other_axis_value, Z) * atan2(other_axis_value, Z)

    return ang - math.pi / 2.

def reproject_axis_x_gen2(p, pt, lh_p,
                     phase_cal,
                     tilt_cal,
                     curve_cal,
                     gibPhase_cal, gibMag_cal):
    XYZ = apply_pose_to_pt(lh_p, apply_pose_to_pt(p, pt))

    X = XYZ[0]
    Y = XYZ[1]
    Z = XYZ[2]
    tan30 = 0.57735026919
    B = atan2(X, -Z)
    A = asin(tan30 * Y / sqrt(X * X + Z * Z))

    return (-A) - B - phase_cal
             
def reproject_axis_y_gen2(p, pt, lh_p,
                     phase_cal,
                     tilt_cal,
                     curve_cal,
                     gibPhase_cal, gibMag_cal):
    XYZ = apply_pose_to_pt(lh_p, apply_pose_to_pt(p, pt))

    X = XYZ[0]
    Y = XYZ[1]
    Z = XYZ[2]
    tan30 = 0.57735026919
    B = atan2(X, -Z)
    A = asin(tan30 * Y / sqrt(X * X + Z * Z))

    return A - B - phase_cal

def reproject_gen2(p, pt, lh_p,
              phase_0, phase_1,
              tilt_0, tilt_1,
              curve_0, curve_1,
              gibPhase_0, gibPhase_1, gibMag_0, gibMag_1):

    return sp.Matrix((
        reproject_axis_x_gen2(p, pt, lh_p, phase_0, tilt_0, curve_0, gibPhase_0, gibMag_0),
        reproject_axis_y_gen2(p, pt, lh_p, phase_1, tilt_1, curve_1, gibPhase_1, gibMag_1)
    ))

def reproject_axis_x(p, pt, lh_p,
                     phase_cal,
                     tilt_cal,
                     curve_cal,
                     gibPhase_cal, gibMag_cal):
    XYZ = apply_pose_to_pt(lh_p, apply_pose_to_pt(p, pt))

    return reproject_axis(XYZ[0], XYZ[1], -XYZ[2],
                          phase_cal,
                          tilt_cal,
                          curve_cal,
                          gibPhase_cal, gibMag_cal)

def simple_neg(x):
    if isinstance(x, sp.Expr):
        return sp.Mul(x, -1, evaluate=False)
    return -x

def reproject_axis_y(p, pt, lh_p,
                     phase_cal,
                     tilt_cal,
                     curve_cal,
                     gibPhase_cal, gibMag_cal):
    XYZ = apply_pose_to_pt(lh_p, apply_pose_to_pt(p, pt))


    return reproject_axis(simple_neg(XYZ[1]), XYZ[0], -XYZ[2],
                          phase_cal,
                          tilt_cal,
                          curve_cal,
                          gibPhase_cal, gibMag_cal)


def reproject(p, pt, lh_p,
              phase_0, phase_1,
              tilt_0, tilt_1,
              curve_0, curve_1,
              gibPhase_0, gibPhase_1, gibMag_0, gibMag_1):
    XYZ = apply_pose_to_pt(lh_p, apply_pose_to_pt(p, pt))

    return sp.Matrix((
        reproject_axis(XYZ[0], XYZ[1], -XYZ[2], phase_0, tilt_0, curve_0, gibPhase_0, gibMag_0),
        reproject_axis(simple_neg(XYZ[1]), XYZ[0], -XYZ[2], phase_1, tilt_1, curve_1, gibPhase_1, gibMag_1)
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


obj_rot = (obj_qw, obj_qi, obj_qj, obj_qk)
obj_p = ((obj_px, obj_py, obj_pz), (obj_qw, obj_qi, obj_qj, obj_qk))

obj_p_axisangle = ((obj_px, obj_py, obj_pz), (obj_qi, obj_qj, obj_qk))

lh_p = ((lh_px, lh_py, lh_pz), (lh_qw, lh_qi, lh_qj, lh_qk))
lh_p_axisangle = ((lh_px, lh_py, lh_pz), (lh_qi, lh_qj, lh_qk))

sensor_pt = (sensor_x, sensor_y, sensor_z)


def flatten_args(bla):
    output = []
    for item in bla:
        output += flatten_args(item) if hasattr(item, "__iter__") else [item]
    return output


def make_sympy(expressions):
    flatten = []
    if hasattr(expressions, "atoms"):
        return [expressions]

    if not hasattr(expressions, "_sympy_"):
        for col in expressions:
            if hasattr(col, '_sympy_'):
                flatten.append(col)
            else:
                for cell in col:
                    flatten.append(cell)
    else:
        flatten.append(expressions)
    return flatten


def c_filter(item):
    if isinstance(item, sp.Atom):
        return item

    with evaluate(False):
        rtn = item.__class__(*map(c_filter, item.args))

        # powers of 2 and 3 should just be shown as x*x*x in C; it's faster to skip the function call
        if item.__class__ == Pow and item.args[1] == 2:
            rtn = item.args[0] * item.args[0]
        elif item.__class__ == Pow and item.args[1] == 3:
            rtn = item.args[0] * item.args[0] * item.args[0]
        if item.__class__ == Pow and item.args[1] == -2:
            rtn = 1 / (item.args[0] * item.args[0])
        elif item.__class__ == Pow and item.args[1] == -3:
            rtn = 1 / (item.args[0] * item.args[0] * item.args[0])

        return rtn


def generate_ccode(name, args, expressions):
    sys.stderr.write("Writing out %s\n" % name)

    if isinstance(expressions, types.FunctionType):
        print("/** Applying function %s */" % str(expressions))
        expressions = expressions(*args)

    flatten = make_sympy(expressions)

    sys.stderr.write("Running CSE\n")
    cse_output = sp.cse(flatten)

    def arg_str((_, a)):
        if isinstance(a, tuple):
            return "const FLT *%s" % str(flatten_args(a)[0]).split('_', 1)[0]
        else:
            return "const FLT " + str(a)

    print("static inline void gen_%s(FLT* out, %s) {" % (name, ", ".join(map(arg_str, enumerate(args)))))

    # Unroll struct types
    for idx, a in enumerate(args):
        if isinstance(a, tuple):
            name = str(flatten_args(a)[0]).split('_', 1)[0]
            for v in flatten_args(a):
                print("\tconst GEN_FLT %s = *(%s++);" % (str(v), name))

    print("\n".join(
        map(lambda item: "\tconst GEN_FLT %s = %s;" % (sp.ccode(item[0]), sp.ccode(c_filter(item[1]))), cse_output[0])))

    for item in cse_output[1]:
        if hasattr(item, "tolist"):
            for item1 in sum(item.tolist(), []):
                print("\t*(out++) = %s;" % sp.ccode(c_filter(item1)))
        else:
            print("\t*(out++) = %s;" % sp.ccode(c_filter(item)))
    print("}")
    print("")


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


def jacobian(v, of):
    if hasattr(v, 'jacobian'):
        return v.jacobian(of)
    return sp.Matrix([v]).jacobian(of)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--aux":
        print("#include \"survive_reproject.generated.h\"")
        #generate_ccode("apply_axisangle_pose_to_pt", [obj_p_axisangle, sensor_pt], apply_axisangle_pose_to_pt)
        #generate_ccode("reproject_axisangle", reproject_axisangle_params, reproject_axisangle)
        generate_ccode("quat_rotate_vector", [obj_rot, sensor_pt], quatrotatevector)
        generate_ccode("invert_pose", [obj_p], invert_pose)
        generate_ccode("reproject", reproject_params, reproject)
        generate_ccode("reproject_gen2", reproject_params, reproject_gen2)        
        generate_ccode("apply_pose", [obj_p, sensor_pt], apply_pose_to_pt)
    else:
        print("// NOTE: Auto-generated code; see tools/generate_reprojection_functions")
        print("#include <linmath.h>")
        print("#include <math.h>")
        print("static inline double __safe_asin(double x) { return asin(linmath_enforce_range(x, -1, 1)); }")
        print("#define asin __safe_asin")
        print('#ifndef WIN32')
        print("#include <complex.h>")
        print("static inline double __safe_pow(double x, double y) { return x >= 0 ? pow(x, y) : creal(cpow(x, y)); }")
        print("#define pow __safe_pow")
        print('#endif')
        print('#define GEN_FLT FLT')
        # generate_ccode("reproject_axis_x_jac_obj_p_axisangle", reproject_axisangle_axis_params ,jacobian(reproject_axis_x_axisangle(*reproject_axisangle_axis_params), ( (obj_px, obj_py, obj_pz, obj_qi,obj_qj,obj_qk) )))
        # generate_ccode("reproject_axis_y_jac_obj_p_axisangle", reproject_axisangle_axis_params , jacobian(reproject_axis_y_axisangle(*reproject_axisangle_axis_params ), (obj_px, obj_py, obj_pz, obj_qi,obj_qj,obj_qk) ))
        # generate_ccode("reproject_jac_obj_p_axisangle", reproject_axisangle_params, jacobian(reproject_axisangle(*reproject_axisangle_params), (obj_px, obj_py, obj_pz, obj_qi,obj_qj,obj_qk)))

        generate_ccode("reproject_jac_obj_p", reproject_params,
                       jacobian(reproject(*reproject_params), (obj_px, obj_py, obj_pz, obj_qw, obj_qi, obj_qj, obj_qk)))
        generate_ccode("reproject_axis_x_jac_obj_p", reproject_axis_params,
                       jacobian(reproject_axis_x(*reproject_axis_params),
                                (obj_px, obj_py, obj_pz, obj_qw, obj_qi, obj_qj, obj_qk)))
        generate_ccode("reproject_axis_y_jac_obj_p", reproject_axis_params,
                       jacobian(reproject_axis_y(*reproject_axis_params),
                                (obj_px, obj_py, obj_pz, obj_qw, obj_qi, obj_qj, obj_qk)))

        generate_ccode("reproject_jac_obj_p_gen2", reproject_params,
                       jacobian(reproject_gen2(*reproject_params), (obj_px, obj_py, obj_pz, obj_qw, obj_qi, obj_qj, obj_qk)))
        generate_ccode("reproject_axis_x_jac_obj_p_gen2", reproject_axis_params,
                       jacobian(reproject_axis_x_gen2(*reproject_axis_params),
                                (obj_px, obj_py, obj_pz, obj_qw, obj_qi, obj_qj, obj_qk)))
        generate_ccode("reproject_axis_y_jac_obj_p_gen2", reproject_axis_params,
                       jacobian(reproject_axis_y_gen2(*reproject_axis_params),
                                (obj_px, obj_py, obj_pz, obj_qw, obj_qi, obj_qj, obj_qk)))
