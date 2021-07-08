import inspect
import sys
import types
from collections.abc import Iterable

from symengine import Pow, cse, Mul
from sympy import evaluate

import sympy

from gen2 import *


def flatten_args(bla, prefix=''):
    output = []
    if hasattr(bla, "__iter__"):
        for i, item in enumerate(bla):
            output += flatten_args(item, prefix=prefix+'['+str(i)+']') if hasattr(item, "__iter__") or isinstance(item, SurviveType) else [(prefix+"["+str(i)+"]", item)]
    elif hasattr(bla, "__dict__"):
        for k,vall in bla.__dict__.items():
            if hasattr(vall, "__iter__") or isinstance(vall, SurviveType):
                for ik, iv in flatten_args(vall, prefix=prefix+'.'+k):
                    output.append((ik, iv))
            else:
                output.append((prefix + "." + k, vall))

    return output


def make_sympy(expressions):
    flatten = []

    if type(expressions) == sp.MutableDenseMatrix:
        return expressions

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

def expand_pow(x):
    pass

def number(x):
    if x.is_Number:
        return float(x)
    return None

def clean_parens(txt):
    if txt[0] == '(' and txt[-1] == ')':
        cnt = 0
        for c in txt[1:-1]:
            if c == '(':
                cnt += 1
            if c == ')':
                cnt -= 1
            if cnt < 0:
                return txt
        return clean_parens(txt[1:-1])
    return txt

def ccode_wrapper(item, depth = 0):
    #return sp.ccode(item)

    if item.is_Atom:
        if item == True:
            return "true"
        if item == False:
            return "false"
        return sp.ccode(item)

    newargs = list(map(lambda x: ccode_wrapper(x, depth+1), item.args))

    infixes = {
        Mul: '*',
        sp.Add: '+',
        sp.GreaterThan: '>=',
        sp.StrictGreaterThan: '>',
        sp.LessThan: '<=',
        sp.StrictLessThan: '<'
    }

    if item.__class__ == Pow:
        # Basically it's always faster to never call pow
        if item.args[1].is_Number and abs(item.args[1]) < 20:
            invert = item.args[1] < 0
            num, den = abs(item.args[1]).get_num_den()

            if den == 1 or den == 2:
                mul_cnt = num if den == 1 else (num - 1) / 2
                muls = [newargs[0]] * int(mul_cnt)
                if den == 2:
                    muls.append("sqrt(" + clean_parens(newargs[0]) + ")")
                v = " * ".join(muls)
                if len(muls) > 1:
                    v = "(" + v + ")"
                if invert:
                    v = "(1. / " + v + ")"
                return v
        return "pow(%s, %s)" % tuple(newargs)
    elif item.__class__ in infixes:
        return "(" + (" " + infixes[item.__class__] + " ").join(newargs) + ")"
    elif item.__class__ == sp.Piecewise:
        if item.args[1] == True:
            return newargs[0]
        if item.args[1] == False:
            return newargs[2]
        return "(%s ? %s : %s)" % (newargs[1], newargs[0], newargs[2])

    return item.__class__.__name__ + "(" + ", ".join(map(clean_parens, newargs)) + ")"
    #raise Exception("Unhandled type " + item.__class__.__name__)

def ccode(item):
    return clean_parens(ccode_wrapper(item))

def get_argument(n):
    if n in globals():
        return globals()[n]
    return sympy.symbols(n)

def get_name(a):
    if type(a) == list:
        return "_".join(map(get_name, a))
    if hasattr(a, '__name__'):
        return a.__name__
    return str(a)

def generate_ccode(func, name=None, args=None, suffix = None):
    if callable(func):
        name = func.__name__
        args = [get_argument(n) for n in inspect.getfullargspec(func).args]

    if suffix is not None:
        name = name + "_" + suffix

    sys.stderr.write("Writing out %s\n" % name)

    if isinstance(func, types.FunctionType):
        func = func(*map_arg(args))

    flatten = make_sympy(func)

    sys.stderr.write("Running CSE\n")
    singular_return = len(flatten) == 1

    cse_output = cse(sp.Matrix(flatten))

    def get_type(a):
        if callable(a):
            return get_type(a())
        if hasattr(a, "__iter__"):
            ty = get_type(a[0])
            if ty[-1] != "*":
                ty += "*"
            return ty
        if isinstance(a, SurviveType):
            return a.__class__.__name__ + "*"
        return "FLT"

    def arg_str(arg):
        a = arg[1]
        return "const %s %s" % (get_type(a), get_name(a))

    if singular_return:
        print("static inline FLT gen_%s(%s) {" % (name, ", ".join(map(arg_str, enumerate(args)))))
    else:
        print("static inline void gen_%s(FLT* out, %s) {" % (name, ", ".join(map(arg_str, enumerate(args)))))

    # Unroll struct types
    for idx, a in enumerate(args):
        if callable(a):
            name = get_name(a)
            for k, v in flatten_args(a()):
                print("\tconst GEN_FLT %s = %s%s;" % (str(v), "(*"+name+")" if isinstance(a(), SurviveType) else name, k))

    print("\n".join(
        map(lambda item: "\tconst GEN_FLT %s = %s;" % (
            sp.ccode(item[0]), ccode(item[1]).replace("\n", " ").replace("\t", " ")), cse_output[0])))

    output_idx = 0
    for item in cse_output[1]:
        if hasattr(item, "tolist"):
            for item1 in sum(item.tolist(), []):
                print("\tout[%d] = %s;" % (output_idx, ccode(item1).replace("\n", " ").replace("\t", " ")))
                output_idx += 1
        else:
            if singular_return:
                print("\treturn %s;" % (ccode(item).replace("\n", " ").replace("\t", " ")))
            else:
                print("\tout[%d] = %s;" % (output_idx, ccode(item).replace("\n", " ").replace("\t", " ")))
            output_idx += 1
    print("}")
    print("")


def jacobian(v, of):
    if hasattr(v, 'jacobian'):
        return v.jacobian(sp.Matrix(of))
    return sp.Matrix([v]).jacobian(sp.Matrix(of))

def map_arg(arg):
    if callable(arg):
        return map_arg(arg())
    elif isinstance(arg, list):
        return list(map(map_arg, arg))
    elif isinstance(arg, tuple):
        return tuple(map(map_arg, arg))
    return arg

def flat_values(a):
    if isinstance(a, Iterable):
        return sum([flat_values(it) for it in a], [])
    if hasattr(a, '__dict__'):
        return flat_values(a.__dict__.values())
    return [a]

def generate_jacobians(func, suffix=None,transpose=False,jac_all=False, jac_over=None):
    rtn = {}

    func_args = [get_argument(n) for n in inspect.getfullargspec(func).args]
    jac_of = {}
    if jac_over is not None:
        jac_of[get_name(jac_over)] = flat_values(map_arg(jac_over))
    else:
        jac_of.update({get_name(arg): flat_values(map_arg(arg)) for arg in func_args})

    if jac_all:
        jac_of['all'] = sum(list(jac_of.values()), [])

    feval = (func(*map_arg(func_args)))

    for name, jac_value in jac_of.items():
        fname = func.__name__  + '_jac_' + name
        if type(feval) == list or type(feval) == tuple:
            feval = sp.MutableDenseMatrix(feval)
        this_jac = jacobian(feval, jac_value)
        if transpose:
            this_jac = this_jac.transpose()
        print("// Jacobian of", func.__name__, "wrt", jac_value)
        generate_ccode(this_jac, fname, func_args, suffix=suffix)
        rtn[fname] = this_jac
    return rtn

def generate_code_and_jacobians(f,transpose=False, jac_over=None):
    generate_ccode(f)
    generate_jacobians(f, transpose=transpose,jac_over=jac_over)