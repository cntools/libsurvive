
import math
import sys
import types
import itertools
import inspect

#import symengine as se
from collections import defaultdict

from collections.abc import Iterable

import symengine as sp
from symengine import sqrt, atan2, tan, asin, cos, Pow, sin, Piecewise, Symbol, cse
from sympy import evaluate, Atom

from common_math import *
from gen1 import *
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
    if item.is_Atom or isinstance(item, Piecewise):
        return item

    with evaluate(False):
        newargs = list(map(c_filter, item.args))
        rtn = item.__class__(*newargs)

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

def generate_ccode(func, name=None, args=None, suffix = None):
    if callable(func):
        name = func.__name__
        args = [globals()[n] for n in inspect.getfullargspec(func).args]

    if suffix is not None:
        name = name + "_" + suffix

    sys.stderr.write("Writing out %s\n" % name)

    if isinstance(func, types.FunctionType):
        print("/** Applying function %s */" % str(func))
        func = func(*map_arg(args))

    flatten = make_sympy(func)

    sys.stderr.write("Running CSE\n")
    cse_output = cse(sp.Matrix(flatten))

    def get_type(a):
        if callable(a):
            return get_type(a())
        if hasattr(a, "__iter__"):
            return get_type(a[0])
        if isinstance(a, SurviveType):
            return a.__class__.__name__
        return "FLT"

    def get_name(a):
        if hasattr(a, '__name__'):
            return a.__name__
        return str(a)

    def arg_str(arg):
        a = arg[1]
        return "const %s *%s" % (get_type(a), get_name(a))

    print("static inline void gen_%s(FLT* out, %s) {" % (name, ", ".join(map(arg_str, enumerate(args)))))

    # Unroll struct types
    for idx, a in enumerate(args):
        if callable(a):
            name = get_name(a)
            for k, v in flatten_args(a()):
                print("\tconst GEN_FLT %s = %s%s;" % (str(v), "(*"+name+")" if isinstance(a(), SurviveType) else name, k))

    print("\n".join(
        map(lambda item: "\tconst GEN_FLT %s = %s;" % (
            sp.ccode(item[0]), sp.ccode(c_filter(item[1])).replace("\n", " ").replace("\t", " ")), cse_output[0])))

    for item in cse_output[1]:
        if hasattr(item, "tolist"):
            for item1 in sum(item.tolist(), []):
                print("\t*(out++) = %s;" % sp.ccode(c_filter(item1)).replace("\n", " ").replace("\t", " "))
        else:
            print("\t*(out++) = %s;" % sp.ccode(c_filter(item)).replace("\n", " ").replace("\t", " "))
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

def generate_jacobians(func, suffix=None):
    func_args = [globals()[n] for n in inspect.getfullargspec(func).args]
    jac_of = {arg.__name__: flat_values(map_arg(arg)) for arg in func_args}

    feval = (func(*map_arg(func_args)))

    for name, jac_value in jac_of.items():
        fname = func.__name__  + '_jac_' + name
        this_jac = jacobian(feval, jac_value)
        print("// Jacobian of", func.__name__, "wrt", jac_value)
        generate_ccode(this_jac, fname, func_args, suffix=suffix)
