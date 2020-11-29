r"""Wrapper for poser.h

Generated with:
./run.py /home/justin/source/oss/libsurvive/include/libsurvive/poser.h /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h /home/justin/source/oss/libsurvive/include/libsurvive/survive.h /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject_gen2.h /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h -I/home/justin/source/oss/libsurvive/_skbuild/linux-x86_64-3.7/cmake-install/include -I/home/justin/source/oss/libsurvive/redist -I/home/justin/source/oss/libsurvive/include/libsurvive -I/home/justin/source/oss/libsurvive/include --custom-library-loader --no-macros -L/home/justin/source/oss/libsurvive/_skbuild/linux-x86_64-3.7/cmake-build -lsurvive --strip-prefix=survive_ -P Survive -o /home/justin/source/oss/libsurvive/bindings/python/pysurvive/pysurvive_generated.py

Do not modify this file.
"""

__docformat__ = "restructuredtext"

# Begin preamble for Python v(3, 2)

import ctypes, os, sys
from ctypes import *

_int_types = (c_int16, c_int32)
if hasattr(ctypes, "c_int64"):
    # Some builds of ctypes apparently do not have c_int64
    # defined; it's a pretty good bet that these builds do not
    # have 64-bit pointers.
    _int_types += (c_int64,)
for t in _int_types:
    if sizeof(t) == sizeof(c_size_t):
        c_ptrdiff_t = t
del t
del _int_types


class UserString:
    def __init__(self, seq):
        if isinstance(seq, bytes):
            self.data = seq
        elif isinstance(seq, UserString):
            self.data = seq.data[:]
        else:
            self.data = str(seq).encode()

    def __bytes__(self):
        return self.data

    def __str__(self):
        return self.data.decode()

    def __repr__(self):
        return repr(self.data)

    def __int__(self):
        return int(self.data.decode())

    def __long__(self):
        return int(self.data.decode())

    def __float__(self):
        return float(self.data.decode())

    def __complex__(self):
        return complex(self.data.decode())

    def __hash__(self):
        return hash(self.data)

    def __cmp__(self, string):
        if isinstance(string, UserString):
            return cmp(self.data, string.data)
        else:
            return cmp(self.data, string)

    def __le__(self, string):
        if isinstance(string, UserString):
            return self.data <= string.data
        else:
            return self.data <= string

    def __lt__(self, string):
        if isinstance(string, UserString):
            return self.data < string.data
        else:
            return self.data < string

    def __ge__(self, string):
        if isinstance(string, UserString):
            return self.data >= string.data
        else:
            return self.data >= string

    def __gt__(self, string):
        if isinstance(string, UserString):
            return self.data > string.data
        else:
            return self.data > string

    def __eq__(self, string):
        if isinstance(string, UserString):
            return self.data == string.data
        else:
            return self.data == string

    def __ne__(self, string):
        if isinstance(string, UserString):
            return self.data != string.data
        else:
            return self.data != string

    def __contains__(self, char):
        return char in self.data

    def __len__(self):
        return len(self.data)

    def __getitem__(self, index):
        return self.__class__(self.data[index])

    def __getslice__(self, start, end):
        start = max(start, 0)
        end = max(end, 0)
        return self.__class__(self.data[start:end])

    def __add__(self, other):
        if isinstance(other, UserString):
            return self.__class__(self.data + other.data)
        elif isinstance(other, bytes):
            return self.__class__(self.data + other)
        else:
            return self.__class__(self.data + str(other).encode())

    def __radd__(self, other):
        if isinstance(other, bytes):
            return self.__class__(other + self.data)
        else:
            return self.__class__(str(other).encode() + self.data)

    def __mul__(self, n):
        return self.__class__(self.data * n)

    __rmul__ = __mul__

    def __mod__(self, args):
        return self.__class__(self.data % args)

    # the following methods are defined in alphabetical order:
    def capitalize(self):
        return self.__class__(self.data.capitalize())

    def center(self, width, *args):
        return self.__class__(self.data.center(width, *args))

    def count(self, sub, start=0, end=sys.maxsize):
        return self.data.count(sub, start, end)

    def decode(self, encoding=None, errors=None):  # XXX improve this?
        if encoding:
            if errors:
                return self.__class__(self.data.decode(encoding, errors))
            else:
                return self.__class__(self.data.decode(encoding))
        else:
            return self.__class__(self.data.decode())

    def encode(self, encoding=None, errors=None):  # XXX improve this?
        if encoding:
            if errors:
                return self.__class__(self.data.encode(encoding, errors))
            else:
                return self.__class__(self.data.encode(encoding))
        else:
            return self.__class__(self.data.encode())

    def endswith(self, suffix, start=0, end=sys.maxsize):
        return self.data.endswith(suffix, start, end)

    def expandtabs(self, tabsize=8):
        return self.__class__(self.data.expandtabs(tabsize))

    def find(self, sub, start=0, end=sys.maxsize):
        return self.data.find(sub, start, end)

    def index(self, sub, start=0, end=sys.maxsize):
        return self.data.index(sub, start, end)

    def isalpha(self):
        return self.data.isalpha()

    def isalnum(self):
        return self.data.isalnum()

    def isdecimal(self):
        return self.data.isdecimal()

    def isdigit(self):
        return self.data.isdigit()

    def islower(self):
        return self.data.islower()

    def isnumeric(self):
        return self.data.isnumeric()

    def isspace(self):
        return self.data.isspace()

    def istitle(self):
        return self.data.istitle()

    def isupper(self):
        return self.data.isupper()

    def join(self, seq):
        return self.data.join(seq)

    def ljust(self, width, *args):
        return self.__class__(self.data.ljust(width, *args))

    def lower(self):
        return self.__class__(self.data.lower())

    def lstrip(self, chars=None):
        return self.__class__(self.data.lstrip(chars))

    def partition(self, sep):
        return self.data.partition(sep)

    def replace(self, old, new, maxsplit=-1):
        return self.__class__(self.data.replace(old, new, maxsplit))

    def rfind(self, sub, start=0, end=sys.maxsize):
        return self.data.rfind(sub, start, end)

    def rindex(self, sub, start=0, end=sys.maxsize):
        return self.data.rindex(sub, start, end)

    def rjust(self, width, *args):
        return self.__class__(self.data.rjust(width, *args))

    def rpartition(self, sep):
        return self.data.rpartition(sep)

    def rstrip(self, chars=None):
        return self.__class__(self.data.rstrip(chars))

    def split(self, sep=None, maxsplit=-1):
        return self.data.split(sep, maxsplit)

    def rsplit(self, sep=None, maxsplit=-1):
        return self.data.rsplit(sep, maxsplit)

    def splitlines(self, keepends=0):
        return self.data.splitlines(keepends)

    def startswith(self, prefix, start=0, end=sys.maxsize):
        return self.data.startswith(prefix, start, end)

    def strip(self, chars=None):
        return self.__class__(self.data.strip(chars))

    def swapcase(self):
        return self.__class__(self.data.swapcase())

    def title(self):
        return self.__class__(self.data.title())

    def translate(self, *args):
        return self.__class__(self.data.translate(*args))

    def upper(self):
        return self.__class__(self.data.upper())

    def zfill(self, width):
        return self.__class__(self.data.zfill(width))


class MutableString(UserString):
    """mutable string objects

    Python strings are immutable objects.  This has the advantage, that
    strings may be used as dictionary keys.  If this property isn't needed
    and you insist on changing string values in place instead, you may cheat
    and use MutableString.

    But the purpose of this class is an educational one: to prevent
    people from inventing their own mutable string class derived
    from UserString and than forget thereby to remove (override) the
    __hash__ method inherited from UserString.  This would lead to
    errors that would be very hard to track down.

    A faster and better solution is to rewrite your program using lists."""

    def __init__(self, string=""):
        self.data = string

    def __hash__(self):
        raise TypeError("unhashable type (it is mutable)")

    def __setitem__(self, index, sub):
        if index < 0:
            index += len(self.data)
        if index < 0 or index >= len(self.data):
            raise IndexError
        self.data = self.data[:index] + sub + self.data[index + 1 :]

    def __delitem__(self, index):
        if index < 0:
            index += len(self.data)
        if index < 0 or index >= len(self.data):
            raise IndexError
        self.data = self.data[:index] + self.data[index + 1 :]

    def __setslice__(self, start, end, sub):
        start = max(start, 0)
        end = max(end, 0)
        if isinstance(sub, UserString):
            self.data = self.data[:start] + sub.data + self.data[end:]
        elif isinstance(sub, bytes):
            self.data = self.data[:start] + sub + self.data[end:]
        else:
            self.data = self.data[:start] + str(sub).encode() + self.data[end:]

    def __delslice__(self, start, end):
        start = max(start, 0)
        end = max(end, 0)
        self.data = self.data[:start] + self.data[end:]

    def immutable(self):
        return UserString(self.data)

    def __iadd__(self, other):
        if isinstance(other, UserString):
            self.data += other.data
        elif isinstance(other, bytes):
            self.data += other
        else:
            self.data += str(other).encode()
        return self

    def __imul__(self, n):
        self.data *= n
        return self


class String(MutableString, Union):

    _fields_ = [("raw", POINTER(c_char)), ("data", c_char_p)]

    def __init__(self, obj=b""):
        if isinstance(obj, (bytes, UserString)):
            self.data = bytes(obj)
        else:
            self.raw = obj

    def __len__(self):
        return self.data and len(self.data) or 0

    def from_param(cls, obj):
        # Convert None or 0
        if obj is None or obj == 0:
            return cls(POINTER(c_char)())

        # Convert from String
        elif isinstance(obj, String):
            return obj

        # Convert from bytes
        elif isinstance(obj, bytes):
            return cls(obj)

        # Convert from str
        elif isinstance(obj, str):
            return cls(obj.encode())

        # Convert from c_char_p
        elif isinstance(obj, c_char_p):
            return obj

        # Convert from POINTER(c_char)
        elif isinstance(obj, POINTER(c_char)):
            return obj

        # Convert from raw pointer
        elif isinstance(obj, int):
            return cls(cast(obj, POINTER(c_char)))

        # Convert from c_char array
        elif isinstance(obj, c_char * len(obj)):
            return obj

        # Convert from object
        else:
            return String.from_param(obj._as_parameter_)

    from_param = classmethod(from_param)


def ReturnString(obj, func=None, arguments=None):
    return String.from_param(obj)


# As of ctypes 1.0, ctypes does not support custom error-checking
# functions on callbacks, nor does it support custom datatypes on
# callbacks, so we must ensure that all callbacks return
# primitive datatypes.
#
# Non-primitive return values wrapped with UNCHECKED won't be
# typechecked, and will be converted to c_void_p.
def UNCHECKED(type):
    if hasattr(type, "_type_") and isinstance(type._type_, str) and type._type_ != "P":
        return type
    else:
        return c_void_p


# ctypes doesn't have direct support for variadic functions, so we have to write
# our own wrapper class
class _variadic_function(object):
    def __init__(self, func, restype, argtypes, errcheck):
        self.func = func
        self.func.restype = restype
        self.argtypes = argtypes
        if errcheck:
            self.func.errcheck = errcheck

    def _as_parameter_(self):
        # So we can pass this variadic function as a function pointer
        return self.func

    def __call__(self, *args):
        fixed_args = []
        i = 0
        for argtype in self.argtypes:
            # Typecheck what we can
            fixed_args.append(argtype.from_param(args[i]))
            i += 1
        return self.func(*fixed_args + list(args[i:]))


def ord_if_char(value):
    """
    Simple helper used for casts to simple builtin types:  if the argument is a
    string type, it will be converted to it's ordinal value.

    This function will raise an exception if the argument is string with more
    than one characters.
    """
    return ord(value) if (isinstance(value, bytes) or isinstance(value, str)) else value

# End preamble

from . import CustomLibraryLoader
load_library = CustomLibraryLoader.load_library

# Begin libraries
_libs = {}
_libs["survive"] = load_library("survive")

# 1 libraries
# End libraries

# No modules

__off_t = c_long# /usr/include/x86_64-linux-gnu/bits/types.h: 152

__off64_t = c_long# /usr/include/x86_64-linux-gnu/bits/types.h: 153

# /usr/include/x86_64-linux-gnu/bits/types/struct_FILE.h: 49
class struct__IO_FILE(Structure):
    pass

FILE = struct__IO_FILE# /usr/include/x86_64-linux-gnu/bits/types/FILE.h: 7

# /usr/include/x86_64-linux-gnu/bits/types/struct_FILE.h: 36
class struct__IO_marker(Structure):
    pass

# /usr/include/x86_64-linux-gnu/bits/types/struct_FILE.h: 37
class struct__IO_codecvt(Structure):
    pass

# /usr/include/x86_64-linux-gnu/bits/types/struct_FILE.h: 38
class struct__IO_wide_data(Structure):
    pass

_IO_lock_t = None# /usr/include/x86_64-linux-gnu/bits/types/struct_FILE.h: 43

struct__IO_FILE.__slots__ = [
    '_flags',
    '_IO_read_ptr',
    '_IO_read_end',
    '_IO_read_base',
    '_IO_write_base',
    '_IO_write_ptr',
    '_IO_write_end',
    '_IO_buf_base',
    '_IO_buf_end',
    '_IO_save_base',
    '_IO_backup_base',
    '_IO_save_end',
    '_markers',
    '_chain',
    '_fileno',
    '_flags2',
    '_old_offset',
    '_cur_column',
    '_vtable_offset',
    '_shortbuf',
    '_lock',
    '_offset',
    '_codecvt',
    '_wide_data',
    '_freeres_list',
    '_freeres_buf',
    '__pad5',
    '_mode',
    '_unused2',
]
struct__IO_FILE._fields_ = [
    ('_flags', c_int),
    ('_IO_read_ptr', String),
    ('_IO_read_end', String),
    ('_IO_read_base', String),
    ('_IO_write_base', String),
    ('_IO_write_ptr', String),
    ('_IO_write_end', String),
    ('_IO_buf_base', String),
    ('_IO_buf_end', String),
    ('_IO_save_base', String),
    ('_IO_backup_base', String),
    ('_IO_save_end', String),
    ('_markers', POINTER(struct__IO_marker)),
    ('_chain', POINTER(struct__IO_FILE)),
    ('_fileno', c_int),
    ('_flags2', c_int),
    ('_old_offset', __off_t),
    ('_cur_column', c_ushort),
    ('_vtable_offset', c_char),
    ('_shortbuf', c_char * int(1)),
    ('_lock', POINTER(_IO_lock_t)),
    ('_offset', __off64_t),
    ('_codecvt', POINTER(struct__IO_codecvt)),
    ('_wide_data', POINTER(struct__IO_wide_data)),
    ('_freeres_list', POINTER(struct__IO_FILE)),
    ('_freeres_buf', POINTER(None)),
    ('__pad5', c_size_t),
    ('_mode', c_int),
    ('_unused2', c_char * int((((15 * sizeof(c_int)) - (4 * sizeof(POINTER(None)))) - sizeof(c_size_t)))),
]

LinmathQuat = c_double * int(4)# /home/justin/source/oss/libsurvive/redist/linmath.h: 80

LinmathPoint3d = c_double * int(3)# /home/justin/source/oss/libsurvive/redist/linmath.h: 82

LinmathVec3d = c_double * int(3)# /home/justin/source/oss/libsurvive/redist/linmath.h: 83

LinmathAxisAngle = c_double * int(3)# /home/justin/source/oss/libsurvive/redist/linmath.h: 85

LinmathAxisAngleMag = c_double * int(4)# /home/justin/source/oss/libsurvive/redist/linmath.h: 86

# /home/justin/source/oss/libsurvive/redist/linmath.h: 96
class struct_LinmathPose(Structure):
    pass

struct_LinmathPose.__slots__ = [
    'Pos',
    'Rot',
]
struct_LinmathPose._fields_ = [
    ('Pos', LinmathPoint3d),
    ('Rot', LinmathQuat),
]

LinmathPose = struct_LinmathPose# /home/justin/source/oss/libsurvive/redist/linmath.h: 96

# /home/justin/source/oss/libsurvive/redist/linmath.h: 101
class struct_LinmathAxisAnglePose(Structure):
    pass

struct_LinmathAxisAnglePose.__slots__ = [
    'Pos',
    'AxisAngleRot',
]
struct_LinmathAxisAnglePose._fields_ = [
    ('Pos', LinmathPoint3d),
    ('AxisAngleRot', LinmathAxisAngle),
]

LinmathAxisAnglePose = struct_LinmathAxisAnglePose# /home/justin/source/oss/libsurvive/redist/linmath.h: 101

SurvivePose = LinmathPose# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 98

SurviveAngularVelocity = LinmathAxisAngleMag# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 99

SurviveVelocity = LinmathAxisAnglePose# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 100

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 107
class struct_survive_kalman_model_t(Structure):
    pass

struct_survive_kalman_model_t.__slots__ = [
    'Pose',
    'Velocity',
    'Acc',
    'GyroBias',
]
struct_survive_kalman_model_t._fields_ = [
    ('Pose', SurvivePose),
    ('Velocity', SurviveVelocity),
    ('Acc', LinmathVec3d),
    ('GyroBias', LinmathVec3d),
]

SurviveKalmanModel = struct_survive_kalman_model_t# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 107

enum_SurviveInputEvent = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 118

SURVIVE_INPUT_EVENT_NONE = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 118

SURVIVE_INPUT_EVENT_BUTTON_FLAG = 2# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 118

SURVIVE_INPUT_EVENT_BUTTON_DOWN = 3# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 118

SURVIVE_INPUT_EVENT_BUTTON_UP = 2# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 118

SURVIVE_INPUT_EVENT_TOUCH_FLAG = 4# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 118

SURVIVE_INPUT_EVENT_TOUCH_DOWN = 5# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 118

SURVIVE_INPUT_EVENT_TOUCH_UP = 4# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 118

SURVIVE_INPUT_EVENT_AXIS_CHANGED = 8# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 118

enum_SurviveButton = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 129

SURVIVE_BUTTON_UNKNOWN = 255# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 129

SURVIVE_BUTTON_TRIGGER = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 129

SURVIVE_BUTTON_TRACKPAD = 1# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 129

SURVIVE_BUTTON_THUMBSTICK = 2# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 129

SURVIVE_BUTTON_SYSTEM = 3# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 129

SURVIVE_BUTTON_A = 4# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 129

SURVIVE_BUTTON_B = 5# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 129

SURVIVE_BUTTON_MENU = 6# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 129

SURVIVE_BUTTON_GRIP = 7# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 129

SURVIVE_BUTTON_ON_FACE = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 129

enum_SurviveAxis = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 144

SURVIVE_AXIS_UNKNOWN = 255# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 144

SURVIVE_AXIS_TRIGGER = 1# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 144

SURVIVE_AXIS_TRACKPAD_X = 2# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 144

SURVIVE_AXIS_TRACKPAD_Y = 3# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 144

SURVIVE_AXIS_MIDDLE_FINGER_PROXIMITY = 4# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 144

SURVIVE_AXIS_RING_FINGER_PROXIMITY = 5# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 144

SURVIVE_AXIS_PINKY_FINGER_PROXIMITY = 6# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 144

SURVIVE_AXIS_TRIGGER_FINGER_PROXIMITY = 7# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 144

SURVIVE_AXIS_GRIP_FORCE = 8# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 144

SURVIVE_AXIS_TRACKPAD_FORCE = 9# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 144

SURVIVE_AXIS_JOYSTICK_X = 10# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 144

SURVIVE_AXIS_JOYSTICK_Y = 11# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 144

SURVIVE_AXIS_IPD = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 144

SURVIVE_AXIS_FACE_PROXIMITY = 1# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 144

SurviveAxisVal_t = c_float# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 161

enum_anon_25 = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 168

SURVIVE_OBJECT_TYPE_UNKNOWN = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 168

SURVIVE_OBJECT_TYPE_HMD = (SURVIVE_OBJECT_TYPE_UNKNOWN + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 168

SURVIVE_OBJECT_TYPE_CONTROLLER = (SURVIVE_OBJECT_TYPE_HMD + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 168

SURVIVE_OBJECT_TYPE_OTHER = (SURVIVE_OBJECT_TYPE_CONTROLLER + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 168

SurviveObjectType = enum_anon_25# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 168

enum_anon_26 = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 180

SURVIVE_OBJECT_SUBTYPE_GENERIC = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 180

SURVIVE_OBJECT_SUBTYPE_VIVE_HMD = (SURVIVE_OBJECT_SUBTYPE_GENERIC + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 180

SURVIVE_OBJECT_SUBTYPE_INDEX_HMD = (SURVIVE_OBJECT_SUBTYPE_VIVE_HMD + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 180

SURVIVE_OBJECT_SUBTYPE_WAND = (SURVIVE_OBJECT_SUBTYPE_INDEX_HMD + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 180

SURVIVE_OBJECT_SUBTYPE_KNUCKLES_R = (SURVIVE_OBJECT_SUBTYPE_WAND + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 180

SURVIVE_OBJECT_SUBTYPE_KNUCKLES_L = (SURVIVE_OBJECT_SUBTYPE_KNUCKLES_R + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 180

SURVIVE_OBJECT_SUBTYPE_TRACKER = (SURVIVE_OBJECT_SUBTYPE_KNUCKLES_L + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 180

SURVIVE_OBJECT_SUBTYPE_TRACKER_GEN2 = (SURVIVE_OBJECT_SUBTYPE_TRACKER + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 180

SURVIVE_OBJECT_SUBTYPE_COUNT = (SURVIVE_OBJECT_SUBTYPE_TRACKER_GEN2 + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 180

SurviveObjectSubtype = enum_anon_26# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 180

survive_timecode = c_uint32# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 182

survive_long_timecode = c_uint64# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 183

survive_channel = c_uint8# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 186

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 188
if _libs["survive"].has("survive_timecode_difference", "cdecl"):
    survive_timecode_difference = _libs["survive"].get("survive_timecode_difference", "cdecl")
    survive_timecode_difference.argtypes = [survive_timecode, survive_timecode]
    survive_timecode_difference.restype = survive_timecode

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 103
class struct_SurviveObject(Structure):
    pass

SurviveObject = struct_SurviveObject# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 190

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 287
class struct_SurviveContext(Structure):
    pass

SurviveContext = struct_SurviveContext# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 191

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 226
class struct_BaseStationData(Structure):
    pass

BaseStationData = struct_BaseStationData# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 192

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 193
class struct_SurviveCalData(Structure):
    pass

SurviveCalData = struct_SurviveCalData# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 193

enum_anon_27 = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

SURVIVE_OK = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

SURVIVE_ERROR_GENERAL = (-1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

SURVIVE_ERROR_NO_TRACKABLE_OBJECTS = (-2)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

SURVIVE_ERROR_HARWARE_FAULT = (-3)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

SURVIVE_ERROR_INVALID_CONFIG = (-4)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

SurviveError = enum_anon_27# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

enum_anon_28 = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 207

SURVIVE_LOG_LEVEL_ERROR = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 207

SURVIVE_LOG_LEVEL_WARNING = 1# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 207

SURVIVE_LOG_LEVEL_INFO = 2# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 207

SurviveLogLevel = enum_anon_28# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 207

survive_driver_fn = CFUNCTYPE(UNCHECKED(None), )# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 209

datalog_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), String, POINTER(c_double), c_size_t)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 211

printf_process_func = CFUNCTYPE(UNCHECKED(c_int), POINTER(SurviveContext), String)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 212

log_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveContext), SurviveLogLevel, String)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 213

report_error_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveContext), SurviveError)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 214

config_process_func = CFUNCTYPE(UNCHECKED(c_int), POINTER(SurviveObject), String, c_int)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 216

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 226
class struct_anon_29(Structure):
    pass

struct_anon_29.__slots__ = [
    'sensor_id',
    'length',
    'timestamp',
]
struct_anon_29._fields_ = [
    ('sensor_id', c_uint8),
    ('length', c_uint16),
    ('timestamp', c_uint32),
]

LightcapElement = struct_anon_29# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 226

gen_detected_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), c_int)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 232

lightcap_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), POINTER(LightcapElement))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 238

light_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), c_int, c_int, c_int, survive_timecode, survive_timecode, c_uint32)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 244

ootx_received_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(struct_SurviveContext), c_uint8)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 247

light_pulse_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), c_int, c_int, survive_timecode, c_double, c_uint32)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 249

angle_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), c_int, c_int, survive_timecode, c_double, c_double, c_uint32)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 255

sync_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), survive_channel, survive_timecode, c_bool, c_bool)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 263

sweep_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), survive_channel, c_int, survive_timecode, c_bool)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 269

sweep_angle_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), survive_channel, c_int, survive_timecode, c_int8, c_double)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 275

raw_imu_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), c_int, POINTER(c_double), survive_timecode, c_int)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 282

imu_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), c_int, POINTER(c_double), survive_timecode, c_int)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 290

button_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), enum_SurviveInputEvent, enum_SurviveButton, POINTER(enum_SurviveAxis), POINTER(SurviveAxisVal_t))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 295

pose_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), survive_long_timecode, POINTER(SurvivePose))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 301

imupose_process_func = pose_process_func# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 306

velocity_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), survive_long_timecode, POINTER(SurviveVelocity))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 311

external_pose_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveContext), String, POINTER(SurvivePose))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 317

external_velocity_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveContext), String, POINTER(SurviveVelocity))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 318

lighthouse_pose_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveContext), c_uint8, POINTER(SurvivePose))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 323

new_object_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 328

haptic_func = CFUNCTYPE(UNCHECKED(c_int), POINTER(SurviveObject), c_double, c_double, c_double)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 331

DeviceDriver = CFUNCTYPE(UNCHECKED(c_int), POINTER(SurviveContext))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 335

enum_SurviveDeviceDriverReturn = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 340

SURVIVE_DRIVER_NORMAL = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 340

SURVIVE_DRIVER_ERROR = (-1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 340

SURVIVE_DRIVER_PASSIVE = 1# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 340

SurviveDeviceDriverReturn = enum_SurviveDeviceDriverReturn# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 340

DeviceDriverCb = CFUNCTYPE(UNCHECKED(c_int), POINTER(struct_SurviveContext), POINTER(None))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 342

DeviceDriverMagicCb = CFUNCTYPE(UNCHECKED(c_int), POINTER(struct_SurviveContext), POINTER(None), c_int, POINTER(None), c_int)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 343

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 345
if _libs["survive"].has("SurviveInputEventStr", "cdecl"):
    SurviveInputEventStr = _libs["survive"].get("SurviveInputEventStr", "cdecl")
    SurviveInputEventStr.argtypes = [enum_SurviveInputEvent]
    SurviveInputEventStr.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 346
if _libs["survive"].has("SurviveButtonsStr", "cdecl"):
    SurviveButtonsStr = _libs["survive"].get("SurviveButtonsStr", "cdecl")
    SurviveButtonsStr.argtypes = [SurviveObjectSubtype, enum_SurviveButton]
    SurviveButtonsStr.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 347
if _libs["survive"].has("SurviveAxisStr", "cdecl"):
    SurviveAxisStr = _libs["survive"].get("SurviveAxisStr", "cdecl")
    SurviveAxisStr.argtypes = [SurviveObjectSubtype, enum_SurviveAxis]
    SurviveAxisStr.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 349
if _libs["survive"].has("SurviveObjectTypeStr", "cdecl"):
    SurviveObjectTypeStr = _libs["survive"].get("SurviveObjectTypeStr", "cdecl")
    SurviveObjectTypeStr.argtypes = [SurviveObjectType]
    SurviveObjectTypeStr.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 350
if _libs["survive"].has("SurviveObjectSubtypeStr", "cdecl"):
    SurviveObjectSubtypeStr = _libs["survive"].get("SurviveObjectSubtypeStr", "cdecl")
    SurviveObjectSubtypeStr.argtypes = [SurviveObjectSubtype]
    SurviveObjectSubtypeStr.restype = c_char_p

enum_PoserType_t = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 23

POSERDATA_NONE = 0# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 23

POSERDATA_IMU = (POSERDATA_NONE + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 23

POSERDATA_LIGHT = (POSERDATA_IMU + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 23

POSERDATA_DISASSOCIATE = (POSERDATA_LIGHT + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 23

POSERDATA_SYNC = (POSERDATA_DISASSOCIATE + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 23

POSERDATA_LIGHT_GEN2 = (POSERDATA_SYNC + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 23

POSERDATA_SYNC_GEN2 = (POSERDATA_LIGHT_GEN2 + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 23

POSERDATA_GLOBAL_SCENES = (POSERDATA_SYNC_GEN2 + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 23

PoserType = enum_PoserType_t# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 23

poser_pose_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), c_uint32, POINTER(SurvivePose), POINTER(None))# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 25

poser_lighthouse_pose_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), c_uint8, POINTER(SurvivePose), POINTER(SurvivePose), POINTER(None))# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 26

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 36
class struct_anon_47(Structure):
    pass

struct_anon_47.__slots__ = [
    'pt',
    'timecode',
    'poseproc',
    'lighthouseposeproc',
    'userdata',
]
struct_anon_47._fields_ = [
    ('pt', PoserType),
    ('timecode', survive_long_timecode),
    ('poseproc', poser_pose_func),
    ('lighthouseposeproc', poser_lighthouse_pose_func),
    ('userdata', POINTER(None)),
]

PoserData = struct_anon_47# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 36

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 38
if _libs["survive"].has("PoserData_size", "cdecl"):
    PoserData_size = _libs["survive"].get("PoserData_size", "cdecl")
    PoserData_size.argtypes = [POINTER(PoserData)]
    PoserData_size.restype = c_int32

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 51
if _libs["survive"].has("PoserData_poser_pose_func", "cdecl"):
    PoserData_poser_pose_func = _libs["survive"].get("PoserData_poser_pose_func", "cdecl")
    PoserData_poser_pose_func.argtypes = [POINTER(PoserData), POINTER(SurviveObject), POINTER(SurvivePose)]
    PoserData_poser_pose_func.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 64
if _libs["survive"].has("PoserData_poser_pose_func_with_velocity", "cdecl"):
    PoserData_poser_pose_func_with_velocity = _libs["survive"].get("PoserData_poser_pose_func_with_velocity", "cdecl")
    PoserData_poser_pose_func_with_velocity.argtypes = [POINTER(PoserData), POINTER(SurviveObject), POINTER(SurvivePose), POINTER(SurviveVelocity)]
    PoserData_poser_pose_func_with_velocity.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 79
if _libs["survive"].has("PoserData_lighthouse_pose_func", "cdecl"):
    PoserData_lighthouse_pose_func = _libs["survive"].get("PoserData_lighthouse_pose_func", "cdecl")
    PoserData_lighthouse_pose_func.argtypes = [POINTER(PoserData), POINTER(SurviveObject), c_uint8, POINTER(SurvivePose), POINTER(SurvivePose)]
    PoserData_lighthouse_pose_func.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 81
if _libs["survive"].has("PoserData_lighthouse_poses_func", "cdecl"):
    PoserData_lighthouse_poses_func = _libs["survive"].get("PoserData_lighthouse_poses_func", "cdecl")
    PoserData_lighthouse_poses_func.argtypes = [POINTER(PoserData), POINTER(SurviveObject), POINTER(SurvivePose), c_uint32, POINTER(SurvivePose)]
    PoserData_lighthouse_poses_func.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 84
if _libs["survive"].has("survive_get_reference_bsd", "cdecl"):
    survive_get_reference_bsd = _libs["survive"].get("survive_get_reference_bsd", "cdecl")
    survive_get_reference_bsd.argtypes = [POINTER(SurviveContext), POINTER(SurvivePose), c_uint32]
    survive_get_reference_bsd.restype = c_int8

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 87
if _libs["survive"].has("survive_lighthouse_adjust_confidence", "cdecl"):
    survive_lighthouse_adjust_confidence = _libs["survive"].get("survive_lighthouse_adjust_confidence", "cdecl")
    survive_lighthouse_adjust_confidence.argtypes = [POINTER(SurviveContext), c_uint8, c_double]
    survive_lighthouse_adjust_confidence.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 88
if _libs["survive"].has("survive_adjust_confidence", "cdecl"):
    survive_adjust_confidence = _libs["survive"].get("survive_adjust_confidence", "cdecl")
    survive_adjust_confidence.argtypes = [POINTER(SurviveObject), c_double]
    survive_adjust_confidence.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 96
class struct_PoserDataIMU(Structure):
    pass

struct_PoserDataIMU.__slots__ = [
    'hdr',
    'datamask',
    'accel',
    'gyro',
    'mag',
]
struct_PoserDataIMU._fields_ = [
    ('hdr', PoserData),
    ('datamask', c_uint8),
    ('accel', c_double * int(3)),
    ('gyro', c_double * int(3)),
    ('mag', c_double * int(3)),
]

PoserDataIMU = struct_PoserDataIMU# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 96

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 106
class struct_PoserDataLight(Structure):
    pass

struct_PoserDataLight.__slots__ = [
    'hdr',
    'sensor_id',
    'lh',
    'angle',
    'assume_current_pose',
    'no_lighthouse_solve',
]
struct_PoserDataLight._fields_ = [
    ('hdr', PoserData),
    ('sensor_id', c_int),
    ('lh', c_int),
    ('angle', c_double),
    ('assume_current_pose', c_bool),
    ('no_lighthouse_solve', c_bool),
]

PoserDataLight = struct_PoserDataLight# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 106

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 113
class struct_PoserDataLightGen1(Structure):
    pass

struct_PoserDataLightGen1.__slots__ = [
    'common',
    'acode',
    'length',
]
struct_PoserDataLightGen1._fields_ = [
    ('common', PoserDataLight),
    ('acode', c_int),
    ('length', c_double),
]

PoserDataLightGen1 = struct_PoserDataLightGen1# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 113

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 120
class struct_PoserDataLightGen2(Structure):
    pass

struct_PoserDataLightGen2.__slots__ = [
    'common',
    'plane',
    'sync',
]
struct_PoserDataLightGen2._fields_ = [
    ('common', PoserDataLight),
    ('plane', c_int8),
    ('sync', c_uint32),
]

PoserDataLightGen2 = struct_PoserDataLightGen2# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 120

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 127
class struct_anon_48(Structure):
    pass

struct_anon_48.__slots__ = [
    'value',
    'lh',
    'sensor_idx',
    'axis',
]
struct_anon_48._fields_ = [
    ('value', c_double),
    ('lh', c_uint8),
    ('sensor_idx', c_uint8),
    ('axis', c_uint8),
]

PoserDataGlobalSceneMeasurement = struct_anon_48# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 127

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 129
class struct_PoserDataGlobalScene(Structure):
    pass

struct_PoserDataGlobalScene.__slots__ = [
    'so',
    'pose',
    'accel',
    'meas_cnt',
    'meas',
]
struct_PoserDataGlobalScene._fields_ = [
    ('so', POINTER(struct_SurviveObject)),
    ('pose', SurvivePose),
    ('accel', LinmathPoint3d),
    ('meas_cnt', c_size_t),
    ('meas', POINTER(PoserDataGlobalSceneMeasurement)),
]

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 143
class struct_PoserDataGlobalScenes(Structure):
    pass

struct_PoserDataGlobalScenes.__slots__ = [
    'hdr',
    'world2lhs',
    'scenes_cnt',
    'scenes',
]
struct_PoserDataGlobalScenes._fields_ = [
    ('hdr', PoserData),
    ('world2lhs', POINTER(SurvivePose)),
    ('scenes_cnt', c_size_t),
    ('scenes', POINTER(struct_PoserDataGlobalScene)),
]

PoserDataGlobalScenes = struct_PoserDataGlobalScenes# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 143

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 145
class union_PoserDataAll(Union):
    pass

union_PoserDataAll.__slots__ = [
    'pd',
    'pdl1',
    'pdlg2',
    'pdimu',
    'pdgs',
]
union_PoserDataAll._fields_ = [
    ('pd', PoserData),
    ('pdl1', PoserDataLight),
    ('pdlg2', PoserDataLightGen2),
    ('pdimu', PoserDataIMU),
    ('pdgs', PoserDataGlobalScenes),
]

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 27
class struct_SurviveSensorActivations_s(Structure):
    pass

PoserCB = CFUNCTYPE(UNCHECKED(c_int), POINTER(SurviveObject), POINTER(POINTER(None)), POINTER(PoserData))# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 156

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 159
for _lib in _libs.values():
    if not _lib.has("survive_poser_invoke", "cdecl"):
        continue
    survive_poser_invoke = _lib.get("survive_poser_invoke", "cdecl")
    survive_poser_invoke.argtypes = [POINTER(SurviveObject), POINTER(PoserData), c_size_t]
    survive_poser_invoke.restype = None
    break

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 161
class struct_survive_threaded_poser(Structure):
    pass

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 162
for _lib in _libs.values():
    if not _lib.has("survive_create_threaded_poser", "cdecl"):
        continue
    survive_create_threaded_poser = _lib.get("survive_create_threaded_poser", "cdecl")
    survive_create_threaded_poser.argtypes = [POINTER(SurviveObject), PoserCB]
    survive_create_threaded_poser.restype = POINTER(struct_survive_threaded_poser)
    break

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 163
for _lib in _libs.values():
    if not _lib.has("survive_threaded_poser_fn", "cdecl"):
        continue
    survive_threaded_poser_fn = _lib.get("survive_threaded_poser_fn", "cdecl")
    survive_threaded_poser_fn.argtypes = [POINTER(SurviveObject), POINTER(POINTER(None)), POINTER(PoserData)]
    survive_threaded_poser_fn.restype = c_int
    break

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 11
class struct_SurviveSimpleContext(Structure):
    pass

SurviveSimpleContext = struct_SurviveSimpleContext# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 12

enum_SurviveSimpleObject_type = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 14

SurviveSimpleObject_UNKNOWN = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 14

SurviveSimpleObject_LIGHTHOUSE = (SurviveSimpleObject_UNKNOWN + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 14

SurviveSimpleObject_HMD = (SurviveSimpleObject_LIGHTHOUSE + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 14

SurviveSimpleObject_OBJECT = (SurviveSimpleObject_HMD + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 14

SurviveSimpleObject_EXTERNAL = (SurviveSimpleObject_OBJECT + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 14

SurviveSimpleSubobject_type = SurviveObjectSubtype# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 22

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 24
class struct_SurviveSimpleObject(Structure):
    pass

SurviveSimpleObject = struct_SurviveSimpleObject# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 25

SurviveSimpleLogFn = CFUNCTYPE(UNCHECKED(None), POINTER(struct_SurviveSimpleContext), SurviveLogLevel, String)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 26

enum_SurviveSimpleEventType = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 28

SurviveSimpleEventType_None = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 28

SurviveSimpleEventType_ButtonEvent = (SurviveSimpleEventType_None + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 28

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 166
class struct_SurviveSimpleEvent(Structure):
    pass

SurviveSimpleEvent = struct_SurviveSimpleEvent# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 31

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 42
class struct_SurviveSimpleButtonEvent(Structure):
    pass

struct_SurviveSimpleButtonEvent.__slots__ = [
    'object',
    'event_type',
    'button_id',
    'axis_count',
    'axis_ids',
    'axis_val',
]
struct_SurviveSimpleButtonEvent._fields_ = [
    ('object', POINTER(SurviveSimpleObject)),
    ('event_type', enum_SurviveInputEvent),
    ('button_id', enum_SurviveButton),
    ('axis_count', c_uint8),
    ('axis_ids', enum_SurviveAxis * int(8)),
    ('axis_val', SurviveAxisVal_t * int(8)),
]

SurviveSimpleButtonEvent = struct_SurviveSimpleButtonEvent# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 42

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 48
if _libs["survive"].has("survive_simple_init", "cdecl"):
    survive_simple_init = _libs["survive"].get("survive_simple_init", "cdecl")
    survive_simple_init.argtypes = [c_int, POINTER(POINTER(c_char))]
    survive_simple_init.restype = POINTER(SurviveSimpleContext)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 49
if _libs["survive"].has("survive_simple_set_user", "cdecl"):
    survive_simple_set_user = _libs["survive"].get("survive_simple_set_user", "cdecl")
    survive_simple_set_user.argtypes = [POINTER(SurviveSimpleContext), POINTER(None)]
    survive_simple_set_user.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 50
if _libs["survive"].has("survive_simple_get_user", "cdecl"):
    survive_simple_get_user = _libs["survive"].get("survive_simple_get_user", "cdecl")
    survive_simple_get_user.argtypes = [POINTER(SurviveSimpleContext)]
    survive_simple_get_user.restype = POINTER(c_ubyte)
    survive_simple_get_user.errcheck = lambda v,*a : cast(v, c_void_p)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 51
if _libs["survive"].has("survive_simple_init_with_logger", "cdecl"):
    survive_simple_init_with_logger = _libs["survive"].get("survive_simple_init_with_logger", "cdecl")
    survive_simple_init_with_logger.argtypes = [c_int, POINTER(POINTER(c_char)), SurviveSimpleLogFn]
    survive_simple_init_with_logger.restype = POINTER(SurviveSimpleContext)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 57
if _libs["survive"].has("survive_simple_close", "cdecl"):
    survive_simple_close = _libs["survive"].get("survive_simple_close", "cdecl")
    survive_simple_close.argtypes = [POINTER(SurviveSimpleContext)]
    survive_simple_close.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 62
if _libs["survive"].has("survive_simple_start_thread", "cdecl"):
    survive_simple_start_thread = _libs["survive"].get("survive_simple_start_thread", "cdecl")
    survive_simple_start_thread.argtypes = [POINTER(SurviveSimpleContext)]
    survive_simple_start_thread.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 67
if _libs["survive"].has("survive_simple_is_running", "cdecl"):
    survive_simple_is_running = _libs["survive"].get("survive_simple_is_running", "cdecl")
    survive_simple_is_running.argtypes = [POINTER(SurviveSimpleContext)]
    survive_simple_is_running.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 72
if _libs["survive"].has("survive_simple_get_first_object", "cdecl"):
    survive_simple_get_first_object = _libs["survive"].get("survive_simple_get_first_object", "cdecl")
    survive_simple_get_first_object.argtypes = [POINTER(SurviveSimpleContext)]
    survive_simple_get_first_object.restype = POINTER(SurviveSimpleObject)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 76
if _libs["survive"].has("survive_simple_get_next_object", "cdecl"):
    survive_simple_get_next_object = _libs["survive"].get("survive_simple_get_next_object", "cdecl")
    survive_simple_get_next_object.argtypes = [POINTER(SurviveSimpleContext), POINTER(SurviveSimpleObject)]
    survive_simple_get_next_object.restype = POINTER(SurviveSimpleObject)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 81
if _libs["survive"].has("survive_simple_get_object", "cdecl"):
    survive_simple_get_object = _libs["survive"].get("survive_simple_get_object", "cdecl")
    survive_simple_get_object.argtypes = [POINTER(SurviveSimpleContext), String]
    survive_simple_get_object.restype = POINTER(SurviveSimpleObject)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 83
if _libs["survive"].has("survive_simple_get_object_count", "cdecl"):
    survive_simple_get_object_count = _libs["survive"].get("survive_simple_get_object_count", "cdecl")
    survive_simple_get_object_count.argtypes = [POINTER(SurviveSimpleContext)]
    survive_simple_get_object_count.restype = c_size_t

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 88
if _libs["survive"].has("survive_simple_get_next_updated", "cdecl"):
    survive_simple_get_next_updated = _libs["survive"].get("survive_simple_get_next_updated", "cdecl")
    survive_simple_get_next_updated.argtypes = [POINTER(SurviveSimpleContext)]
    survive_simple_get_next_updated.restype = POINTER(SurviveSimpleObject)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 94
if _libs["survive"].has("survive_simple_object_get_latest_pose", "cdecl"):
    survive_simple_object_get_latest_pose = _libs["survive"].get("survive_simple_object_get_latest_pose", "cdecl")
    survive_simple_object_get_latest_pose.argtypes = [POINTER(SurviveSimpleObject), POINTER(SurvivePose)]
    survive_simple_object_get_latest_pose.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 100
if _libs["survive"].has("survive_simple_object_get_latest_velocity", "cdecl"):
    survive_simple_object_get_latest_velocity = _libs["survive"].get("survive_simple_object_get_latest_velocity", "cdecl")
    survive_simple_object_get_latest_velocity.argtypes = [POINTER(SurviveSimpleObject), POINTER(SurviveVelocity)]
    survive_simple_object_get_latest_velocity.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 105
if _libs["survive"].has("survive_simple_object_name", "cdecl"):
    survive_simple_object_name = _libs["survive"].get("survive_simple_object_name", "cdecl")
    survive_simple_object_name.argtypes = [POINTER(SurviveSimpleObject)]
    survive_simple_object_name.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 110
if _libs["survive"].has("survive_simple_serial_number", "cdecl"):
    survive_simple_serial_number = _libs["survive"].get("survive_simple_serial_number", "cdecl")
    survive_simple_serial_number.argtypes = [POINTER(SurviveSimpleObject)]
    survive_simple_serial_number.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 116
if _libs["survive"].has("survive_simple_wait_for_update", "cdecl"):
    survive_simple_wait_for_update = _libs["survive"].get("survive_simple_wait_for_update", "cdecl")
    survive_simple_wait_for_update.argtypes = [POINTER(SurviveSimpleContext)]
    survive_simple_wait_for_update.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 120
if _libs["survive"].has("survive_simple_next_event", "cdecl"):
    survive_simple_next_event = _libs["survive"].get("survive_simple_next_event", "cdecl")
    survive_simple_next_event.argtypes = [POINTER(SurviveSimpleContext), POINTER(SurviveSimpleEvent)]
    survive_simple_next_event.restype = enum_SurviveSimpleEventType

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 123
if _libs["survive"].has("survive_simple_object_haptic", "cdecl"):
    survive_simple_object_haptic = _libs["survive"].get("survive_simple_object_haptic", "cdecl")
    survive_simple_object_haptic.argtypes = [POINTER(struct_SurviveSimpleObject), c_double, c_double, c_double]
    survive_simple_object_haptic.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 125
if _libs["survive"].has("survive_simple_object_get_type", "cdecl"):
    survive_simple_object_get_type = _libs["survive"].get("survive_simple_object_get_type", "cdecl")
    survive_simple_object_get_type.argtypes = [POINTER(struct_SurviveSimpleObject)]
    survive_simple_object_get_type.restype = enum_SurviveSimpleObject_type

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 126
if _libs["survive"].has("survive_simple_object_get_input_axis", "cdecl"):
    survive_simple_object_get_input_axis = _libs["survive"].get("survive_simple_object_get_input_axis", "cdecl")
    survive_simple_object_get_input_axis.argtypes = [POINTER(struct_SurviveSimpleObject), enum_SurviveAxis]
    survive_simple_object_get_input_axis.restype = SurviveAxisVal_t

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 128
if _libs["survive"].has("survive_simple_object_get_subtype", "cdecl"):
    survive_simple_object_get_subtype = _libs["survive"].get("survive_simple_object_get_subtype", "cdecl")
    survive_simple_object_get_subtype.argtypes = [POINTER(struct_SurviveSimpleObject)]
    survive_simple_object_get_subtype.restype = SurviveSimpleSubobject_type

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 133
if _libs["survive"].has("survive_simple_get_button_event", "cdecl"):
    survive_simple_get_button_event = _libs["survive"].get("survive_simple_get_button_event", "cdecl")
    survive_simple_get_button_event.argtypes = [POINTER(SurviveSimpleEvent)]
    survive_simple_get_button_event.restype = POINTER(SurviveSimpleButtonEvent)

struct_SurviveSimpleEvent.__slots__ = [
    'event_type',
    '__private_button_event',
]
struct_SurviveSimpleEvent._fields_ = [
    ('event_type', enum_SurviveSimpleEventType),
    ('__private_button_event', SurviveSimpleButtonEvent),
]

struct_SurviveSensorActivations_s.__slots__ = [
    'so',
    'lh_gen',
    'angles',
    'angles_center',
    'angles_center_cnt',
    'timecode',
    'lengths',
    'imu_init_cnt',
    'last_imu',
    'last_light',
    'last_light_change',
    'last_movement',
    'runtime_offset',
    'accel',
    'gyro',
    'mag',
]
struct_SurviveSensorActivations_s._fields_ = [
    ('so', POINTER(SurviveObject)),
    ('lh_gen', c_int),
    ('angles', ((c_double * int(2)) * int(16)) * int(32)),
    ('angles_center', (c_double * int(2)) * int(16)),
    ('angles_center_cnt', (c_int * int(2)) * int(16)),
    ('timecode', ((survive_long_timecode * int(2)) * int(16)) * int(32)),
    ('lengths', ((survive_timecode * int(2)) * int(2)) * int(32)),
    ('imu_init_cnt', c_size_t),
    ('last_imu', survive_long_timecode),
    ('last_light', survive_long_timecode),
    ('last_light_change', survive_long_timecode),
    ('last_movement', survive_long_timecode),
    ('runtime_offset', c_double),
    ('accel', c_double * int(3)),
    ('gyro', c_double * int(3)),
    ('mag', c_double * int(3)),
]

SurviveSensorActivations = struct_SurviveSensorActivations_s# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 52

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 57
if _libs["survive"].has("SurviveSensorActivations_reset", "cdecl"):
    SurviveSensorActivations_reset = _libs["survive"].get("SurviveSensorActivations_reset", "cdecl")
    SurviveSensorActivations_reset.argtypes = [POINTER(SurviveSensorActivations)]
    SurviveSensorActivations_reset.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 58
if _libs["survive"].has("SurviveSensorActivations_ctor", "cdecl"):
    SurviveSensorActivations_ctor = _libs["survive"].get("SurviveSensorActivations_ctor", "cdecl")
    SurviveSensorActivations_ctor.argtypes = [POINTER(SurviveObject), POINTER(SurviveSensorActivations)]
    SurviveSensorActivations_ctor.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 59
if _libs["survive"].has("SurviveSensorActivations_long_timecode_imu", "cdecl"):
    SurviveSensorActivations_long_timecode_imu = _libs["survive"].get("SurviveSensorActivations_long_timecode_imu", "cdecl")
    SurviveSensorActivations_long_timecode_imu.argtypes = [POINTER(SurviveSensorActivations), survive_timecode]
    SurviveSensorActivations_long_timecode_imu.restype = survive_long_timecode

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 60
if _libs["survive"].has("SurviveSensorActivations_long_timecode_light", "cdecl"):
    SurviveSensorActivations_long_timecode_light = _libs["survive"].get("SurviveSensorActivations_long_timecode_light", "cdecl")
    SurviveSensorActivations_long_timecode_light.argtypes = [POINTER(SurviveSensorActivations), survive_timecode]
    SurviveSensorActivations_long_timecode_light.restype = survive_long_timecode

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 65
if _libs["survive"].has("SurviveSensorActivations_difference", "cdecl"):
    SurviveSensorActivations_difference = _libs["survive"].get("SurviveSensorActivations_difference", "cdecl")
    SurviveSensorActivations_difference.argtypes = [POINTER(SurviveSensorActivations), POINTER(SurviveSensorActivations)]
    SurviveSensorActivations_difference.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 67
if _libs["survive"].has("SurviveSensorActivations_add", "cdecl"):
    SurviveSensorActivations_add = _libs["survive"].get("SurviveSensorActivations_add", "cdecl")
    SurviveSensorActivations_add.argtypes = [POINTER(SurviveSensorActivations), POINTER(struct_PoserDataLightGen1)]
    SurviveSensorActivations_add.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 68
if _libs["survive"].has("SurviveSensorActivations_add_gen2", "cdecl"):
    SurviveSensorActivations_add_gen2 = _libs["survive"].get("SurviveSensorActivations_add_gen2", "cdecl")
    SurviveSensorActivations_add_gen2.argtypes = [POINTER(SurviveSensorActivations), POINTER(struct_PoserDataLightGen2)]
    SurviveSensorActivations_add_gen2.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 71
if _libs["survive"].has("SurviveSensorActivations_register_runtime", "cdecl"):
    SurviveSensorActivations_register_runtime = _libs["survive"].get("SurviveSensorActivations_register_runtime", "cdecl")
    SurviveSensorActivations_register_runtime.argtypes = [POINTER(SurviveSensorActivations), survive_long_timecode, c_uint64]
    SurviveSensorActivations_register_runtime.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 73
if _libs["survive"].has("SurviveSensorActivations_runtime", "cdecl"):
    SurviveSensorActivations_runtime = _libs["survive"].get("SurviveSensorActivations_runtime", "cdecl")
    SurviveSensorActivations_runtime.argtypes = [POINTER(SurviveSensorActivations), survive_long_timecode]
    SurviveSensorActivations_runtime.restype = c_uint64

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 74
if _libs["survive"].has("SurviveSensorActivations_add_imu", "cdecl"):
    SurviveSensorActivations_add_imu = _libs["survive"].get("SurviveSensorActivations_add_imu", "cdecl")
    SurviveSensorActivations_add_imu.argtypes = [POINTER(SurviveSensorActivations), POINTER(struct_PoserDataIMU)]
    SurviveSensorActivations_add_imu.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 80
if _libs["survive"].has("SurviveSensorActivations_isReadingValid", "cdecl"):
    SurviveSensorActivations_isReadingValid = _libs["survive"].get("SurviveSensorActivations_isReadingValid", "cdecl")
    SurviveSensorActivations_isReadingValid.argtypes = [POINTER(SurviveSensorActivations), survive_long_timecode, c_uint32, c_int, c_int]
    SurviveSensorActivations_isReadingValid.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 88
if _libs["survive"].has("SurviveSensorActivations_isPairValid", "cdecl"):
    SurviveSensorActivations_isPairValid = _libs["survive"].get("SurviveSensorActivations_isPairValid", "cdecl")
    SurviveSensorActivations_isPairValid.argtypes = [POINTER(SurviveSensorActivations), survive_timecode, survive_timecode, c_uint32, c_int]
    SurviveSensorActivations_isPairValid.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 94
if _libs["survive"].has("SurviveSensorActivations_stationary_time", "cdecl"):
    SurviveSensorActivations_stationary_time = _libs["survive"].get("SurviveSensorActivations_stationary_time", "cdecl")
    SurviveSensorActivations_stationary_time.argtypes = [POINTER(SurviveSensorActivations)]
    SurviveSensorActivations_stationary_time.restype = survive_long_timecode

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 95
if _libs["survive"].has("SurviveSensorActivations_last_time", "cdecl"):
    SurviveSensorActivations_last_time = _libs["survive"].get("SurviveSensorActivations_last_time", "cdecl")
    SurviveSensorActivations_last_time.argtypes = [POINTER(SurviveSensorActivations)]
    SurviveSensorActivations_last_time.restype = survive_long_timecode

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 101
try:
    SurviveSensorActivations_default_tolerance = (survive_timecode).in_dll(_libs["survive"], "SurviveSensorActivations_default_tolerance")
except:
    pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 185
class struct_SurviveKalmanTracker(Structure):
    pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 187
class struct_anon_49(Structure):
    pass

struct_anon_49.__slots__ = [
    'syncs',
    'skipped_syncs',
    'bad_syncs',
    'hit_from_lhs',
    'rejected_data',
    'dropped_light',
    'sync_resets',
    'extent_hits',
    'extent_misses',
    'naive_hits',
    'min_extent',
    'max_extent',
]
struct_anon_49._fields_ = [
    ('syncs', c_uint32 * int(16)),
    ('skipped_syncs', c_uint32 * int(16)),
    ('bad_syncs', c_uint32 * int(16)),
    ('hit_from_lhs', c_uint32 * int(16)),
    ('rejected_data', c_uint32 * int(16)),
    ('dropped_light', c_uint32 * int(16)),
    ('sync_resets', c_uint32 * int(16)),
    ('extent_hits', c_uint32),
    ('extent_misses', c_uint32),
    ('naive_hits', c_uint32),
    ('min_extent', c_double),
    ('max_extent', c_double),
]

struct_SurviveObject.__slots__ = [
    'ctx',
    'codename',
    'drivername',
    'serial_number',
    'driver',
    'object_type',
    'object_subtype',
    'buttonmask',
    'touchmask',
    'axis',
    'charge',
    'charging',
    'ison',
    'additional_flags',
    'PoseConfidence',
    'OutPose',
    'OutPoseIMU',
    'poseConfidence',
    'OutPose_timecode',
    'velocity',
    'velocity_timecode',
    'FromLHPose',
    'PoserFnData',
    'sensor_ct',
    'channel_map',
    'has_sensor_locations',
    'sensor_locations',
    'sensor_normals',
    'timebase_hz',
    'disambiguator_data',
    'oldcode',
    'last_time_between_sync',
    'last_sync_time',
    'sync_count',
    'imu_freq',
    'head2trackref',
    'imu2trackref',
    'head2imu',
    'acc_bias',
    'acc_scale',
    'gyro_bias',
    'gyro_scale',
    'haptic',
    'activations',
    'user_ptr',
    'conf',
    'conf_cnt',
    'tracker',
    'stats',
]
struct_SurviveObject._fields_ = [
    ('ctx', POINTER(SurviveContext)),
    ('codename', c_char * int(4)),
    ('drivername', c_char * int(8)),
    ('serial_number', c_char * int(16)),
    ('driver', POINTER(None)),
    ('object_type', SurviveObjectType),
    ('object_subtype', SurviveObjectSubtype),
    ('buttonmask', c_uint32),
    ('touchmask', c_uint32),
    ('axis', SurviveAxisVal_t * int(16)),
    ('charge', c_int8),
    ('charging', c_int8, 1),
    ('ison', c_uint8, 1),
    ('additional_flags', c_int8, 6),
    ('PoseConfidence', c_double),
    ('OutPose', SurvivePose),
    ('OutPoseIMU', SurvivePose),
    ('poseConfidence', c_double),
    ('OutPose_timecode', survive_long_timecode),
    ('velocity', SurviveVelocity),
    ('velocity_timecode', survive_long_timecode),
    ('FromLHPose', SurvivePose * int(16)),
    ('PoserFnData', POINTER(None)),
    ('sensor_ct', c_int8),
    ('channel_map', POINTER(c_int)),
    ('has_sensor_locations', c_bool),
    ('sensor_locations', POINTER(c_double)),
    ('sensor_normals', POINTER(c_double)),
    ('timebase_hz', c_int32),
    ('disambiguator_data', POINTER(None)),
    ('oldcode', c_int8),
    ('last_time_between_sync', survive_timecode * int(16)),
    ('last_sync_time', survive_timecode * int(16)),
    ('sync_count', survive_timecode * int(16)),
    ('imu_freq', c_double),
    ('head2trackref', SurvivePose),
    ('imu2trackref', SurvivePose),
    ('head2imu', SurvivePose),
    ('acc_bias', c_double * int(3)),
    ('acc_scale', c_double * int(3)),
    ('gyro_bias', c_double * int(3)),
    ('gyro_scale', c_double * int(3)),
    ('haptic', haptic_func),
    ('activations', SurviveSensorActivations),
    ('user_ptr', POINTER(None)),
    ('conf', String),
    ('conf_cnt', c_size_t),
    ('tracker', POINTER(struct_SurviveKalmanTracker)),
    ('stats', struct_anon_49),
]

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 202
if _libs["survive"].has("survive_object_codename", "cdecl"):
    survive_object_codename = _libs["survive"].get("survive_object_codename", "cdecl")
    survive_object_codename.argtypes = [POINTER(SurviveObject)]
    survive_object_codename.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 203
if _libs["survive"].has("survive_object_last_imu2world", "cdecl"):
    survive_object_last_imu2world = _libs["survive"].get("survive_object_last_imu2world", "cdecl")
    survive_object_last_imu2world.argtypes = [POINTER(SurviveObject)]
    survive_object_last_imu2world.restype = POINTER(SurvivePose)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 204
if _libs["survive"].has("survive_object_drivername", "cdecl"):
    survive_object_drivername = _libs["survive"].get("survive_object_drivername", "cdecl")
    survive_object_drivername.argtypes = [POINTER(SurviveObject)]
    survive_object_drivername.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 205
if _libs["survive"].has("survive_object_charge", "cdecl"):
    survive_object_charge = _libs["survive"].get("survive_object_charge", "cdecl")
    survive_object_charge.argtypes = [POINTER(SurviveObject)]
    survive_object_charge.restype = c_int8

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 206
if _libs["survive"].has("survive_object_charging", "cdecl"):
    survive_object_charging = _libs["survive"].get("survive_object_charging", "cdecl")
    survive_object_charging.argtypes = [POINTER(SurviveObject)]
    survive_object_charging.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 208
if _libs["survive"].has("survive_object_pose", "cdecl"):
    survive_object_pose = _libs["survive"].get("survive_object_pose", "cdecl")
    survive_object_pose.argtypes = [POINTER(SurviveObject)]
    survive_object_pose.restype = POINTER(SurvivePose)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 210
if _libs["survive"].has("survive_object_sensor_ct", "cdecl"):
    survive_object_sensor_ct = _libs["survive"].get("survive_object_sensor_ct", "cdecl")
    survive_object_sensor_ct.argtypes = [POINTER(SurviveObject)]
    survive_object_sensor_ct.restype = c_int8

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 211
if _libs["survive"].has("survive_object_sensor_locations", "cdecl"):
    survive_object_sensor_locations = _libs["survive"].get("survive_object_sensor_locations", "cdecl")
    survive_object_sensor_locations.argtypes = [POINTER(SurviveObject)]
    survive_object_sensor_locations.restype = POINTER(c_double)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 212
if _libs["survive"].has("survive_object_sensor_normals", "cdecl"):
    survive_object_sensor_normals = _libs["survive"].get("survive_object_sensor_normals", "cdecl")
    survive_object_sensor_normals.argtypes = [POINTER(SurviveObject)]
    survive_object_sensor_normals.restype = POINTER(c_double)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 224
class struct_BaseStationCal(Structure):
    pass

struct_BaseStationCal.__slots__ = [
    'phase',
    'tilt',
    'curve',
    'gibpha',
    'gibmag',
    'ogeephase',
    'ogeemag',
]
struct_BaseStationCal._fields_ = [
    ('phase', c_double),
    ('tilt', c_double),
    ('curve', c_double),
    ('gibpha', c_double),
    ('gibmag', c_double),
    ('ogeephase', c_double),
    ('ogeemag', c_double),
]

BaseStationCal = struct_BaseStationCal# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 224

struct_BaseStationData.__slots__ = [
    'PositionSet',
    'Pose',
    'OOTXSet',
    'BaseStationID',
    'fcal',
    'accel',
    'mode',
    'confidence',
    'ootx_data',
    'user_ptr',
]
struct_BaseStationData._fields_ = [
    ('PositionSet', c_uint8, 1),
    ('Pose', SurvivePose),
    ('OOTXSet', c_uint8, 1),
    ('BaseStationID', c_uint32),
    ('fcal', BaseStationCal * int(2)),
    ('accel', c_int8 * int(3)),
    ('mode', c_uint8),
    ('confidence', c_double),
    ('ootx_data', POINTER(None)),
    ('user_ptr', POINTER(None)),
]

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 246
class struct_config_group(Structure):
    pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 263
class struct_anon_50(Structure):
    pass

struct_anon_50.__slots__ = [
    'isPopulated',
    'eventType',
    'buttonId',
    'ids',
    'axisValues',
    'so',
]
struct_anon_50._fields_ = [
    ('isPopulated', c_uint8),
    ('eventType', enum_SurviveInputEvent),
    ('buttonId', enum_SurviveButton),
    ('ids', enum_SurviveAxis * int(16)),
    ('axisValues', SurviveAxisVal_t * int(16)),
    ('so', POINTER(SurviveObject)),
]

ButtonQueueEntry = struct_anon_50# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 263

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 272
class struct_anon_51(Structure):
    pass

struct_anon_51.__slots__ = [
    'nextReadIndex',
    'nextWriteIndex',
    'buttonservicesem',
    'entry',
    'processed_events',
]
struct_anon_51._fields_ = [
    ('nextReadIndex', c_uint8),
    ('nextWriteIndex', c_uint8),
    ('buttonservicesem', POINTER(None)),
    ('entry', ButtonQueueEntry * int(32)),
    ('processed_events', c_size_t),
]

ButtonQueue = struct_anon_51# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 272

enum_anon_52 = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 274

SURVIVE_STOPPED = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 274

SURVIVE_RUNNING = (SURVIVE_STOPPED + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 274

SURVIVE_CLOSING = (SURVIVE_RUNNING + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 274

SURVIVE_STATE_MAX = (SURVIVE_CLOSING + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 274

SurviveState = enum_anon_52# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 274

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 276
class struct_SurviveRecordingData(Structure):
    pass

enum_SurviveCalFlag = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 278

SVCal_None = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 278

SVCal_Phase = 1# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 278

SVCal_Tilt = 2# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 278

SVCal_Curve = 4# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 278

SVCal_Gib = 8# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 278

SVCal_All = (((SVCal_Gib | SVCal_Curve) | SVCal_Tilt) | SVCal_Phase)# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 278

struct_SurviveContext.__slots__ = [
    'lh_version_configed',
    'lh_version_forced',
    'lh_version',
    'new_objectproc',
    'printfproc',
    'logproc',
    'report_errorproc',
    'configproc',
    'gen_detectedproc',
    'ootx_receivedproc',
    'lightcapproc',
    'lightproc',
    'light_pulseproc',
    'angleproc',
    'syncproc',
    'sweepproc',
    'sweep_angleproc',
    'raw_imuproc',
    'imuproc',
    'buttonproc',
    'imuposeproc',
    'poseproc',
    'velocityproc',
    'external_poseproc',
    'external_velocityproc',
    'lighthouse_poseproc',
    'datalogproc',
    'activeLighthouses',
    'bsd',
    'bsd_map',
    'disambiguator_data',
    'recptr',
    'objs',
    'objs_ct',
    'PoserFn',
    'drivers',
    'driverpolls',
    'drivercloses',
    'driver_ct',
    'state',
    'currentError',
    'buttonservicethread',
    'buttonQueue',
    'user_ptr',
    'log_level',
    'log_target',
    'poll_min_time_ms',
    'global_config_values',
    'lh_config',
    'temporary_config_values',
    'private_members',
]
struct_SurviveContext._fields_ = [
    ('lh_version_configed', c_int),
    ('lh_version_forced', c_int),
    ('lh_version', c_int),
    ('new_objectproc', new_object_process_func),
    ('printfproc', printf_process_func),
    ('logproc', log_process_func),
    ('report_errorproc', report_error_process_func),
    ('configproc', config_process_func),
    ('gen_detectedproc', gen_detected_process_func),
    ('ootx_receivedproc', ootx_received_process_func),
    ('lightcapproc', lightcap_process_func),
    ('lightproc', light_process_func),
    ('light_pulseproc', light_pulse_process_func),
    ('angleproc', angle_process_func),
    ('syncproc', sync_process_func),
    ('sweepproc', sweep_process_func),
    ('sweep_angleproc', sweep_angle_process_func),
    ('raw_imuproc', raw_imu_process_func),
    ('imuproc', imu_process_func),
    ('buttonproc', button_process_func),
    ('imuposeproc', imupose_process_func),
    ('poseproc', pose_process_func),
    ('velocityproc', velocity_process_func),
    ('external_poseproc', external_pose_process_func),
    ('external_velocityproc', external_velocity_process_func),
    ('lighthouse_poseproc', lighthouse_pose_process_func),
    ('datalogproc', datalog_process_func),
    ('activeLighthouses', c_int),
    ('bsd', BaseStationData * int(16)),
    ('bsd_map', c_int8 * int(16)),
    ('disambiguator_data', POINTER(None)),
    ('recptr', POINTER(struct_SurviveRecordingData)),
    ('objs', POINTER(POINTER(SurviveObject))),
    ('objs_ct', c_int),
    ('PoserFn', PoserCB),
    ('drivers', POINTER(POINTER(None))),
    ('driverpolls', POINTER(DeviceDriverCb)),
    ('drivercloses', POINTER(DeviceDriverCb)),
    ('driver_ct', c_int),
    ('state', SurviveState),
    ('currentError', SurviveError),
    ('buttonservicethread', POINTER(None)),
    ('buttonQueue', ButtonQueue),
    ('user_ptr', POINTER(None)),
    ('log_level', c_int),
    ('log_target', POINTER(FILE)),
    ('poll_min_time_ms', c_size_t),
    ('global_config_values', POINTER(struct_config_group)),
    ('lh_config', POINTER(struct_config_group)),
    ('temporary_config_values', POINTER(struct_config_group)),
    ('private_members', POINTER(None)),
]

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 333
if _libs["survive"].has("survive_verify_FLT_size", "cdecl"):
    survive_verify_FLT_size = _libs["survive"].get("survive_verify_FLT_size", "cdecl")
    survive_verify_FLT_size.argtypes = [c_uint32]
    survive_verify_FLT_size.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 336
if _libs["survive"].has("survive_init_internal", "cdecl"):
    survive_init_internal = _libs["survive"].get("survive_init_internal", "cdecl")
    survive_init_internal.argtypes = [c_int, POINTER(POINTER(c_char)), POINTER(None), log_process_func]
    survive_init_internal.restype = POINTER(SurviveContext)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 10
if _libs["survive"].has("survive_install_new_object_fn", "cdecl"):
    survive_install_new_object_fn = _libs["survive"].get("survive_install_new_object_fn", "cdecl")
    survive_install_new_object_fn.argtypes = [POINTER(SurviveContext), new_object_process_func]
    survive_install_new_object_fn.restype = new_object_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 11
if _libs["survive"].has("survive_install_printf_fn", "cdecl"):
    survive_install_printf_fn = _libs["survive"].get("survive_install_printf_fn", "cdecl")
    survive_install_printf_fn.argtypes = [POINTER(SurviveContext), printf_process_func]
    survive_install_printf_fn.restype = printf_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 12
if _libs["survive"].has("survive_install_log_fn", "cdecl"):
    survive_install_log_fn = _libs["survive"].get("survive_install_log_fn", "cdecl")
    survive_install_log_fn.argtypes = [POINTER(SurviveContext), log_process_func]
    survive_install_log_fn.restype = log_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 13
if _libs["survive"].has("survive_install_report_error_fn", "cdecl"):
    survive_install_report_error_fn = _libs["survive"].get("survive_install_report_error_fn", "cdecl")
    survive_install_report_error_fn.argtypes = [POINTER(SurviveContext), report_error_process_func]
    survive_install_report_error_fn.restype = report_error_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 15
if _libs["survive"].has("survive_install_config_fn", "cdecl"):
    survive_install_config_fn = _libs["survive"].get("survive_install_config_fn", "cdecl")
    survive_install_config_fn.argtypes = [POINTER(SurviveContext), config_process_func]
    survive_install_config_fn.restype = config_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 16
if _libs["survive"].has("survive_install_gen_detected_fn", "cdecl"):
    survive_install_gen_detected_fn = _libs["survive"].get("survive_install_gen_detected_fn", "cdecl")
    survive_install_gen_detected_fn.argtypes = [POINTER(SurviveContext), gen_detected_process_func]
    survive_install_gen_detected_fn.restype = gen_detected_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 17
if _libs["survive"].has("survive_install_ootx_received_fn", "cdecl"):
    survive_install_ootx_received_fn = _libs["survive"].get("survive_install_ootx_received_fn", "cdecl")
    survive_install_ootx_received_fn.argtypes = [POINTER(SurviveContext), ootx_received_process_func]
    survive_install_ootx_received_fn.restype = ootx_received_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 20
if _libs["survive"].has("survive_install_lightcap_fn", "cdecl"):
    survive_install_lightcap_fn = _libs["survive"].get("survive_install_lightcap_fn", "cdecl")
    survive_install_lightcap_fn.argtypes = [POINTER(SurviveContext), lightcap_process_func]
    survive_install_lightcap_fn.restype = lightcap_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 21
if _libs["survive"].has("survive_install_light_fn", "cdecl"):
    survive_install_light_fn = _libs["survive"].get("survive_install_light_fn", "cdecl")
    survive_install_light_fn.argtypes = [POINTER(SurviveContext), light_process_func]
    survive_install_light_fn.restype = light_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 22
if _libs["survive"].has("survive_install_light_pulse_fn", "cdecl"):
    survive_install_light_pulse_fn = _libs["survive"].get("survive_install_light_pulse_fn", "cdecl")
    survive_install_light_pulse_fn.argtypes = [POINTER(SurviveContext), light_pulse_process_func]
    survive_install_light_pulse_fn.restype = light_pulse_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 23
if _libs["survive"].has("survive_install_angle_fn", "cdecl"):
    survive_install_angle_fn = _libs["survive"].get("survive_install_angle_fn", "cdecl")
    survive_install_angle_fn.argtypes = [POINTER(SurviveContext), angle_process_func]
    survive_install_angle_fn.restype = angle_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 26
if _libs["survive"].has("survive_install_sync_fn", "cdecl"):
    survive_install_sync_fn = _libs["survive"].get("survive_install_sync_fn", "cdecl")
    survive_install_sync_fn.argtypes = [POINTER(SurviveContext), sync_process_func]
    survive_install_sync_fn.restype = sync_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 27
if _libs["survive"].has("survive_install_sweep_fn", "cdecl"):
    survive_install_sweep_fn = _libs["survive"].get("survive_install_sweep_fn", "cdecl")
    survive_install_sweep_fn.argtypes = [POINTER(SurviveContext), sweep_process_func]
    survive_install_sweep_fn.restype = sweep_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 28
if _libs["survive"].has("survive_install_sweep_angle_fn", "cdecl"):
    survive_install_sweep_angle_fn = _libs["survive"].get("survive_install_sweep_angle_fn", "cdecl")
    survive_install_sweep_angle_fn.argtypes = [POINTER(SurviveContext), sweep_angle_process_func]
    survive_install_sweep_angle_fn.restype = sweep_angle_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 30
if _libs["survive"].has("survive_install_raw_imu_fn", "cdecl"):
    survive_install_raw_imu_fn = _libs["survive"].get("survive_install_raw_imu_fn", "cdecl")
    survive_install_raw_imu_fn.argtypes = [POINTER(SurviveContext), raw_imu_process_func]
    survive_install_raw_imu_fn.restype = raw_imu_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 31
if _libs["survive"].has("survive_install_imu_fn", "cdecl"):
    survive_install_imu_fn = _libs["survive"].get("survive_install_imu_fn", "cdecl")
    survive_install_imu_fn.argtypes = [POINTER(SurviveContext), imu_process_func]
    survive_install_imu_fn.restype = imu_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 32
if _libs["survive"].has("survive_install_button_fn", "cdecl"):
    survive_install_button_fn = _libs["survive"].get("survive_install_button_fn", "cdecl")
    survive_install_button_fn.argtypes = [POINTER(SurviveContext), button_process_func]
    survive_install_button_fn.restype = button_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 34
if _libs["survive"].has("survive_install_imupose_fn", "cdecl"):
    survive_install_imupose_fn = _libs["survive"].get("survive_install_imupose_fn", "cdecl")
    survive_install_imupose_fn.argtypes = [POINTER(SurviveContext), imupose_process_func]
    survive_install_imupose_fn.restype = imupose_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 35
if _libs["survive"].has("survive_install_pose_fn", "cdecl"):
    survive_install_pose_fn = _libs["survive"].get("survive_install_pose_fn", "cdecl")
    survive_install_pose_fn.argtypes = [POINTER(SurviveContext), pose_process_func]
    survive_install_pose_fn.restype = pose_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 36
if _libs["survive"].has("survive_install_velocity_fn", "cdecl"):
    survive_install_velocity_fn = _libs["survive"].get("survive_install_velocity_fn", "cdecl")
    survive_install_velocity_fn.argtypes = [POINTER(SurviveContext), velocity_process_func]
    survive_install_velocity_fn.restype = velocity_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 38
if _libs["survive"].has("survive_install_external_pose_fn", "cdecl"):
    survive_install_external_pose_fn = _libs["survive"].get("survive_install_external_pose_fn", "cdecl")
    survive_install_external_pose_fn.argtypes = [POINTER(SurviveContext), external_pose_process_func]
    survive_install_external_pose_fn.restype = external_pose_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 39
if _libs["survive"].has("survive_install_external_velocity_fn", "cdecl"):
    survive_install_external_velocity_fn = _libs["survive"].get("survive_install_external_velocity_fn", "cdecl")
    survive_install_external_velocity_fn.argtypes = [POINTER(SurviveContext), external_velocity_process_func]
    survive_install_external_velocity_fn.restype = external_velocity_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 40
if _libs["survive"].has("survive_install_lighthouse_pose_fn", "cdecl"):
    survive_install_lighthouse_pose_fn = _libs["survive"].get("survive_install_lighthouse_pose_fn", "cdecl")
    survive_install_lighthouse_pose_fn.argtypes = [POINTER(SurviveContext), lighthouse_pose_process_func]
    survive_install_lighthouse_pose_fn.restype = lighthouse_pose_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 42
if _libs["survive"].has("survive_install_datalog_fn", "cdecl"):
    survive_install_datalog_fn = _libs["survive"].get("survive_install_datalog_fn", "cdecl")
    survive_install_datalog_fn.argtypes = [POINTER(SurviveContext), datalog_process_func]
    survive_install_datalog_fn.restype = datalog_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 371
if _libs["survive"].has("survive_startup", "cdecl"):
    survive_startup = _libs["survive"].get("survive_startup", "cdecl")
    survive_startup.argtypes = [POINTER(SurviveContext)]
    survive_startup.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 372
if _libs["survive"].has("survive_poll", "cdecl"):
    survive_poll = _libs["survive"].get("survive_poll", "cdecl")
    survive_poll.argtypes = [POINTER(SurviveContext)]
    survive_poll.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 373
if _libs["survive"].has("survive_close", "cdecl"):
    survive_close = _libs["survive"].get("survive_close", "cdecl")
    survive_close.argtypes = [POINTER(SurviveContext)]
    survive_close.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 374
if _libs["survive"].has("survive_get_ctx_lock", "cdecl"):
    survive_get_ctx_lock = _libs["survive"].get("survive_get_ctx_lock", "cdecl")
    survive_get_ctx_lock.argtypes = [POINTER(SurviveContext)]
    survive_get_ctx_lock.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 375
if _libs["survive"].has("survive_release_ctx_lock", "cdecl"):
    survive_release_ctx_lock = _libs["survive"].get("survive_release_ctx_lock", "cdecl")
    survive_release_ctx_lock.argtypes = [POINTER(SurviveContext)]
    survive_release_ctx_lock.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 377
if _libs["survive"].has("survive_get_so_by_name", "cdecl"):
    survive_get_so_by_name = _libs["survive"].get("survive_get_so_by_name", "cdecl")
    survive_get_so_by_name.argtypes = [POINTER(SurviveContext), String]
    survive_get_so_by_name.restype = POINTER(SurviveObject)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 380
if _libs["survive"].has("survive_simple_inflate", "cdecl"):
    survive_simple_inflate = _libs["survive"].get("survive_simple_inflate", "cdecl")
    survive_simple_inflate.argtypes = [POINTER(SurviveContext), POINTER(c_uint8), c_int, POINTER(c_uint8), c_int]
    survive_simple_inflate.restype = c_int

enum_survive_config_flags = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 383

SC_GET = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 383

SC_SET = 1# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 383

SC_OVERRIDE = 2# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 383

SC_SETCONFIG = 4# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 383

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 390
if _libs["survive"].has("survive_config_is_set", "cdecl"):
    survive_config_is_set = _libs["survive"].get("survive_config_is_set", "cdecl")
    survive_config_is_set.argtypes = [POINTER(SurviveContext), String]
    survive_config_is_set.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 391
if _libs["survive"].has("survive_configf", "cdecl"):
    survive_configf = _libs["survive"].get("survive_configf", "cdecl")
    survive_configf.argtypes = [POINTER(SurviveContext), String, c_char, c_double]
    survive_configf.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 392
if _libs["survive"].has("survive_configi", "cdecl"):
    survive_configi = _libs["survive"].get("survive_configi", "cdecl")
    survive_configi.argtypes = [POINTER(SurviveContext), String, c_char, c_uint32]
    survive_configi.restype = c_uint32

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 393
if _libs["survive"].has("survive_config_type", "cdecl"):
    survive_config_type = _libs["survive"].get("survive_config_type", "cdecl")
    survive_config_type.argtypes = [POINTER(SurviveContext), String]
    survive_config_type.restype = c_char

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 394
if _libs["survive"].has("survive_config_as_str", "cdecl"):
    survive_config_as_str = _libs["survive"].get("survive_config_as_str", "cdecl")
    survive_config_as_str.argtypes = [POINTER(SurviveContext), String, c_size_t, String, String]
    survive_config_as_str.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 397
if _libs["survive"].has("survive_configs", "cdecl"):
    survive_configs = _libs["survive"].get("survive_configs", "cdecl")
    survive_configs.argtypes = [POINTER(SurviveContext), String, c_char, String]
    survive_configs.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 399
if _libs["survive"].has("survive_attach_configi", "cdecl"):
    survive_attach_configi = _libs["survive"].get("survive_attach_configi", "cdecl")
    survive_attach_configi.argtypes = [POINTER(SurviveContext), String, POINTER(c_int32)]
    survive_attach_configi.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 400
if _libs["survive"].has("survive_attach_configf", "cdecl"):
    survive_attach_configf = _libs["survive"].get("survive_attach_configf", "cdecl")
    survive_attach_configf.argtypes = [POINTER(SurviveContext), String, POINTER(c_double)]
    survive_attach_configf.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 401
if _libs["survive"].has("survive_attach_configs", "cdecl"):
    survive_attach_configs = _libs["survive"].get("survive_attach_configs", "cdecl")
    survive_attach_configs.argtypes = [POINTER(SurviveContext), String, String]
    survive_attach_configs.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 402
if _libs["survive"].has("survive_detach_config", "cdecl"):
    survive_detach_config = _libs["survive"].get("survive_detach_config", "cdecl")
    survive_detach_config.argtypes = [POINTER(SurviveContext), String, POINTER(None)]
    survive_detach_config.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 404
if _libs["survive"].has("survive_get_bsd_idx", "cdecl"):
    survive_get_bsd_idx = _libs["survive"].get("survive_get_bsd_idx", "cdecl")
    survive_get_bsd_idx.argtypes = [POINTER(SurviveContext), survive_channel]
    survive_get_bsd_idx.restype = c_int8

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 411
if _libs["survive"].has("survive_config_bind_variable", "cdecl"):
    _func = _libs["survive"].get("survive_config_bind_variable", "cdecl")
    _restype = None
    _errcheck = None
    _argtypes = [c_char, String, String]
    survive_config_bind_variable = _variadic_function(_func,_restype,_argtypes,_errcheck)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 415
for _lib in _libs.values():
    if not _lib.has("survive_cal_get_status", "cdecl"):
        continue
    survive_cal_get_status = _lib.get("survive_cal_get_status", "cdecl")
    survive_cal_get_status.argtypes = [POINTER(SurviveContext), String, c_int]
    survive_cal_get_status.restype = c_int
    break

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 418
if _libs["survive"].has("survive_haptic", "cdecl"):
    survive_haptic = _libs["survive"].get("survive_haptic", "cdecl")
    survive_haptic.argtypes = [POINTER(SurviveObject), c_double, c_double, c_double]
    survive_haptic.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 419
if _libs["survive"].has("survive_ootx_free_decoder_context", "cdecl"):
    survive_ootx_free_decoder_context = _libs["survive"].get("survive_ootx_free_decoder_context", "cdecl")
    survive_ootx_free_decoder_context.argtypes = [POINTER(struct_SurviveContext), c_int]
    survive_ootx_free_decoder_context.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 420
if _libs["survive"].has("survive_find_ang_velocity", "cdecl"):
    survive_find_ang_velocity = _libs["survive"].get("survive_find_ang_velocity", "cdecl")
    survive_find_ang_velocity.argtypes = [SurviveAngularVelocity, c_double, LinmathQuat, LinmathQuat]
    survive_find_ang_velocity.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 422
if _libs["survive"].has("survive_apply_ang_velocity", "cdecl"):
    survive_apply_ang_velocity = _libs["survive"].get("survive_apply_ang_velocity", "cdecl")
    survive_apply_ang_velocity.argtypes = [LinmathQuat, SurviveAngularVelocity, c_double, LinmathQuat]
    survive_apply_ang_velocity.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 426
if _libs["survive"].has("survive_default_ootx_received_process", "cdecl"):
    survive_default_ootx_received_process = _libs["survive"].get("survive_default_ootx_received_process", "cdecl")
    survive_default_ootx_received_process.argtypes = [POINTER(struct_SurviveContext), c_uint8]
    survive_default_ootx_received_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 428
if _libs["survive"].has("survive_default_printf_process", "cdecl"):
    _func = _libs["survive"].get("survive_default_printf_process", "cdecl")
    _restype = c_int
    _errcheck = None
    _argtypes = [POINTER(struct_SurviveContext), String]
    survive_default_printf_process = _variadic_function(_func,_restype,_argtypes,_errcheck)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 429
if _libs["survive"].has("survive_default_log_process", "cdecl"):
    survive_default_log_process = _libs["survive"].get("survive_default_log_process", "cdecl")
    survive_default_log_process.argtypes = [POINTER(struct_SurviveContext), SurviveLogLevel, String]
    survive_default_log_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 430
if _libs["survive"].has("survive_default_lightcap_process", "cdecl"):
    survive_default_lightcap_process = _libs["survive"].get("survive_default_lightcap_process", "cdecl")
    survive_default_lightcap_process.argtypes = [POINTER(SurviveObject), POINTER(LightcapElement)]
    survive_default_lightcap_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 431
if _libs["survive"].has("survive_default_light_process", "cdecl"):
    survive_default_light_process = _libs["survive"].get("survive_default_light_process", "cdecl")
    survive_default_light_process.argtypes = [POINTER(SurviveObject), c_int, c_int, c_int, survive_timecode, survive_timecode, c_uint32]
    survive_default_light_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 433
if _libs["survive"].has("survive_default_raw_imu_process", "cdecl"):
    survive_default_raw_imu_process = _libs["survive"].get("survive_default_raw_imu_process", "cdecl")
    survive_default_raw_imu_process.argtypes = [POINTER(SurviveObject), c_int, POINTER(c_double), survive_timecode, c_int]
    survive_default_raw_imu_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 435
if _libs["survive"].has("survive_default_imu_process", "cdecl"):
    survive_default_imu_process = _libs["survive"].get("survive_default_imu_process", "cdecl")
    survive_default_imu_process.argtypes = [POINTER(SurviveObject), c_int, POINTER(c_double), survive_timecode, c_int]
    survive_default_imu_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 436
if _libs["survive"].has("survive_default_angle_process", "cdecl"):
    survive_default_angle_process = _libs["survive"].get("survive_default_angle_process", "cdecl")
    survive_default_angle_process.argtypes = [POINTER(SurviveObject), c_int, c_int, survive_timecode, c_double, c_double, c_uint32]
    survive_default_angle_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 439
if _libs["survive"].has("survive_default_light_pulse_process", "cdecl"):
    survive_default_light_pulse_process = _libs["survive"].get("survive_default_light_pulse_process", "cdecl")
    survive_default_light_pulse_process.argtypes = [POINTER(SurviveObject), c_int, c_int, survive_timecode, c_double, c_uint32]
    survive_default_light_pulse_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 441
if _libs["survive"].has("survive_default_sync_process", "cdecl"):
    survive_default_sync_process = _libs["survive"].get("survive_default_sync_process", "cdecl")
    survive_default_sync_process.argtypes = [POINTER(SurviveObject), survive_channel, survive_timecode, c_bool, c_bool]
    survive_default_sync_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 443
if _libs["survive"].has("survive_default_sweep_process", "cdecl"):
    survive_default_sweep_process = _libs["survive"].get("survive_default_sweep_process", "cdecl")
    survive_default_sweep_process.argtypes = [POINTER(SurviveObject), survive_channel, c_int, survive_timecode, c_bool]
    survive_default_sweep_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 445
if _libs["survive"].has("survive_default_sweep_angle_process", "cdecl"):
    survive_default_sweep_angle_process = _libs["survive"].get("survive_default_sweep_angle_process", "cdecl")
    survive_default_sweep_angle_process.argtypes = [POINTER(SurviveObject), survive_channel, c_int, survive_timecode, c_int8, c_double]
    survive_default_sweep_angle_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 447
if _libs["survive"].has("survive_default_button_process", "cdecl"):
    survive_default_button_process = _libs["survive"].get("survive_default_button_process", "cdecl")
    survive_default_button_process.argtypes = [POINTER(SurviveObject), enum_SurviveInputEvent, enum_SurviveButton, POINTER(enum_SurviveAxis), POINTER(SurviveAxisVal_t)]
    survive_default_button_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 450
if _libs["survive"].has("survive_default_imupose_process", "cdecl"):
    survive_default_imupose_process = _libs["survive"].get("survive_default_imupose_process", "cdecl")
    survive_default_imupose_process.argtypes = [POINTER(SurviveObject), survive_long_timecode, POINTER(SurvivePose)]
    survive_default_imupose_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 452
if _libs["survive"].has("survive_default_pose_process", "cdecl"):
    survive_default_pose_process = _libs["survive"].get("survive_default_pose_process", "cdecl")
    survive_default_pose_process.argtypes = [POINTER(SurviveObject), survive_long_timecode, POINTER(SurvivePose)]
    survive_default_pose_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 454
if _libs["survive"].has("survive_default_velocity_process", "cdecl"):
    survive_default_velocity_process = _libs["survive"].get("survive_default_velocity_process", "cdecl")
    survive_default_velocity_process.argtypes = [POINTER(SurviveObject), survive_long_timecode, POINTER(SurviveVelocity)]
    survive_default_velocity_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 456
if _libs["survive"].has("survive_default_external_pose_process", "cdecl"):
    survive_default_external_pose_process = _libs["survive"].get("survive_default_external_pose_process", "cdecl")
    survive_default_external_pose_process.argtypes = [POINTER(SurviveContext), String, POINTER(SurvivePose)]
    survive_default_external_pose_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 458
if _libs["survive"].has("survive_default_external_velocity_process", "cdecl"):
    survive_default_external_velocity_process = _libs["survive"].get("survive_default_external_velocity_process", "cdecl")
    survive_default_external_velocity_process.argtypes = [POINTER(SurviveContext), String, POINTER(SurviveVelocity)]
    survive_default_external_velocity_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 460
if _libs["survive"].has("survive_default_lighthouse_pose_process", "cdecl"):
    survive_default_lighthouse_pose_process = _libs["survive"].get("survive_default_lighthouse_pose_process", "cdecl")
    survive_default_lighthouse_pose_process.argtypes = [POINTER(SurviveContext), c_uint8, POINTER(SurvivePose)]
    survive_default_lighthouse_pose_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 462
if _libs["survive"].has("survive_default_config_process", "cdecl"):
    survive_default_config_process = _libs["survive"].get("survive_default_config_process", "cdecl")
    survive_default_config_process.argtypes = [POINTER(SurviveObject), String, c_int]
    survive_default_config_process.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 463
if _libs["survive"].has("survive_default_gen_detected_process", "cdecl"):
    survive_default_gen_detected_process = _libs["survive"].get("survive_default_gen_detected_process", "cdecl")
    survive_default_gen_detected_process.argtypes = [POINTER(SurviveObject), c_int]
    survive_default_gen_detected_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 464
if _libs["survive"].has("survive_default_new_object_process", "cdecl"):
    survive_default_new_object_process = _libs["survive"].get("survive_default_new_object_process", "cdecl")
    survive_default_new_object_process.argtypes = [POINTER(SurviveObject)]
    survive_default_new_object_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 465
if _libs["survive"].has("survive_run_time", "cdecl"):
    survive_run_time = _libs["survive"].get("survive_run_time", "cdecl")
    survive_run_time.argtypes = [POINTER(SurviveContext)]
    survive_run_time.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 467
if _libs["survive"].has("survive_input_event_count", "cdecl"):
    survive_input_event_count = _libs["survive"].get("survive_input_event_count", "cdecl")
    survive_input_event_count.argtypes = [POINTER(SurviveContext)]
    survive_input_event_count.restype = c_size_t

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 470
if _libs["survive"].has("RegisterDriver", "cdecl"):
    RegisterDriver = _libs["survive"].get("RegisterDriver", "cdecl")
    RegisterDriver.argtypes = [String, survive_driver_fn]
    RegisterDriver.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 471
if _libs["survive"].has("RegisterPoserDriver", "cdecl"):
    RegisterPoserDriver = _libs["survive"].get("RegisterPoserDriver", "cdecl")
    RegisterPoserDriver.argtypes = [String, PoserCB]
    RegisterPoserDriver.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 494
if _libs["survive"].has("survive_add_object", "cdecl"):
    survive_add_object = _libs["survive"].get("survive_add_object", "cdecl")
    survive_add_object.argtypes = [POINTER(SurviveContext), POINTER(SurviveObject)]
    survive_add_object.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 495
if _libs["survive"].has("survive_remove_object", "cdecl"):
    survive_remove_object = _libs["survive"].get("survive_remove_object", "cdecl")
    survive_remove_object.argtypes = [POINTER(SurviveContext), POINTER(SurviveObject)]
    survive_remove_object.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 496
if _libs["survive"].has("survive_get_driver", "cdecl"):
    survive_get_driver = _libs["survive"].get("survive_get_driver", "cdecl")
    survive_get_driver.argtypes = [POINTER(SurviveContext), DeviceDriverCb]
    survive_get_driver.restype = POINTER(c_ubyte)
    survive_get_driver.errcheck = lambda v,*a : cast(v, c_void_p)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 497
if _libs["survive"].has("survive_get_driver_by_closefn", "cdecl"):
    survive_get_driver_by_closefn = _libs["survive"].get("survive_get_driver_by_closefn", "cdecl")
    survive_get_driver_by_closefn.argtypes = [POINTER(SurviveContext), DeviceDriverCb]
    survive_get_driver_by_closefn.restype = POINTER(c_ubyte)
    survive_get_driver_by_closefn.errcheck = lambda v,*a : cast(v, c_void_p)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 498
if _libs["survive"].has("survive_add_driver", "cdecl"):
    survive_add_driver = _libs["survive"].get("survive_add_driver", "cdecl")
    survive_add_driver.argtypes = [POINTER(SurviveContext), POINTER(None), DeviceDriverCb, DeviceDriverCb]
    survive_add_driver.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 500
if _libs["survive"].has("survive_add_threaded_driver", "cdecl"):
    survive_add_threaded_driver = _libs["survive"].get("survive_add_threaded_driver", "cdecl")
    survive_add_threaded_driver.argtypes = [POINTER(SurviveContext), POINTER(None), String, CFUNCTYPE(UNCHECKED(POINTER(c_ubyte)), POINTER(None)), DeviceDriverCb]
    survive_add_threaded_driver.restype = POINTER(c_bool)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 502
if _libs["survive"].has("survive_export_config", "cdecl"):
    survive_export_config = _libs["survive"].get("survive_export_config", "cdecl")
    survive_export_config.argtypes = [POINTER(SurviveObject)]
    if sizeof(c_int) == sizeof(c_void_p):
        survive_export_config.restype = ReturnString
    else:
        survive_export_config.restype = String
        survive_export_config.errcheck = ReturnString

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 505
if _libs["survive"].has("survive_map_sensor_id", "cdecl"):
    survive_map_sensor_id = _libs["survive"].get("survive_map_sensor_id", "cdecl")
    survive_map_sensor_id.argtypes = [POINTER(SurviveObject), c_uint8]
    survive_map_sensor_id.restype = c_uint8

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 506
if _libs["survive"].has("handle_lightcap", "cdecl"):
    handle_lightcap = _libs["survive"].get("handle_lightcap", "cdecl")
    handle_lightcap.argtypes = [POINTER(SurviveObject), POINTER(LightcapElement)]
    handle_lightcap.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 574
for _lib in _libs.values():
    try:
        ctx = (POINTER(struct_SurviveContext)).in_dll(_lib, "ctx")
        break
    except:
        pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 575
for _lib in _libs.values():
    try:
        stbuff = (c_char * int(1024)).in_dll(_lib, "stbuff")
        break
    except:
        pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 586
for _lib in _libs.values():
    try:
        ctx = (POINTER(struct_SurviveContext)).in_dll(_lib, "ctx")
        break
    except:
        pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 587
for _lib in _libs.values():
    try:
        stbuff = (c_char * int(1024)).in_dll(_lib, "stbuff")
        break
    except:
        pass

SurviveAngleReading = c_double * int(2)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 22

survive_reproject_axis_fn_t = CFUNCTYPE(UNCHECKED(c_double), POINTER(BaseStationCal), POINTER(c_double))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 24

survive_reproject_xy_fn_t = CFUNCTYPE(UNCHECKED(None), POINTER(BaseStationCal), LinmathVec3d, POINTER(c_double))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 25

survive_reproject_full_xy_fn_t = CFUNCTYPE(UNCHECKED(c_double), POINTER(SurvivePose), LinmathVec3d, POINTER(SurvivePose), POINTER(BaseStationCal))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 27

survive_reproject_axis_jacob_fn_t = CFUNCTYPE(UNCHECKED(None), POINTER(c_double), POINTER(SurvivePose), LinmathPoint3d, POINTER(SurvivePose), POINTER(BaseStationCal))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 31

survive_reproject_full_jac_obj_pose_fn_t = CFUNCTYPE(UNCHECKED(None), POINTER(c_double), POINTER(SurvivePose), LinmathVec3d, POINTER(SurvivePose), POINTER(BaseStationCal))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 33

survive_reproject_full_jac_lh_pose_fn_t = survive_reproject_full_jac_obj_pose_fn_t# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 37

survive_reproject_axis_jacob_lh_pose_fn_t = survive_reproject_axis_jacob_fn_t# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 38

survive_reproject_axisangle_axis_jacob_fn_t = CFUNCTYPE(UNCHECKED(None), POINTER(c_double), POINTER(LinmathAxisAnglePose), LinmathPoint3d, POINTER(LinmathAxisAnglePose), POINTER(BaseStationCal))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 40

survive_reproject_axisangle_full_jac_obj_pose_fn_t = CFUNCTYPE(UNCHECKED(None), POINTER(c_double), POINTER(LinmathAxisAnglePose), LinmathVec3d, POINTER(LinmathAxisAnglePose), POINTER(BaseStationCal))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 43

survive_reproject_axisangle_full_jac_lh_pose_fn_t = survive_reproject_axisangle_axis_jacob_fn_t# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 48

survive_reproject_axisangle_axis_jacob_lh_pose_fn_t = survive_reproject_axisangle_full_jac_obj_pose_fn_t# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 49

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 67
class struct_survive_reproject_model_t(Structure):
    pass

struct_survive_reproject_model_t.__slots__ = [
    'reprojectXY',
    'reprojectAxisFn',
    'reprojectAxisFullFn',
    'reprojectFullJacObjPose',
    'reprojectAxisJacobFn',
    'reprojectFullJacLhPose',
    'reprojectAxisJacobLhPoseFn',
    'reprojectAxisAngleFullJacObjPose',
    'reprojectAxisAngleAxisJacobFn',
    'reprojectAxisAngleFullJacLhPose',
    'reprojectAxisAngleAxisJacobLhPoseFn',
]
struct_survive_reproject_model_t._fields_ = [
    ('reprojectXY', survive_reproject_xy_fn_t),
    ('reprojectAxisFn', survive_reproject_axis_fn_t * int(2)),
    ('reprojectAxisFullFn', survive_reproject_full_xy_fn_t * int(2)),
    ('reprojectFullJacObjPose', survive_reproject_full_jac_obj_pose_fn_t),
    ('reprojectAxisJacobFn', survive_reproject_axis_jacob_fn_t * int(2)),
    ('reprojectFullJacLhPose', survive_reproject_full_jac_lh_pose_fn_t),
    ('reprojectAxisJacobLhPoseFn', survive_reproject_axis_jacob_lh_pose_fn_t * int(2)),
    ('reprojectAxisAngleFullJacObjPose', survive_reproject_axisangle_full_jac_obj_pose_fn_t),
    ('reprojectAxisAngleAxisJacobFn', survive_reproject_axisangle_axis_jacob_fn_t * int(2)),
    ('reprojectAxisAngleFullJacLhPose', survive_reproject_axisangle_full_jac_lh_pose_fn_t),
    ('reprojectAxisAngleAxisJacobLhPoseFn', survive_reproject_axisangle_axis_jacob_lh_pose_fn_t * int(2)),
]

survive_reproject_model_t = struct_survive_reproject_model_t# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 67

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 69
try:
    survive_reproject_model = (survive_reproject_model_t).in_dll(_libs["survive"], "survive_reproject_model")
except:
    pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 71
if _libs["survive"].has("survive_reproject_axis_x", "cdecl"):
    survive_reproject_axis_x = _libs["survive"].get("survive_reproject_axis_x", "cdecl")
    survive_reproject_axis_x.argtypes = [POINTER(BaseStationCal), LinmathVec3d]
    survive_reproject_axis_x.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 72
if _libs["survive"].has("survive_reproject_axis_y", "cdecl"):
    survive_reproject_axis_y = _libs["survive"].get("survive_reproject_axis_y", "cdecl")
    survive_reproject_axis_y.argtypes = [POINTER(BaseStationCal), LinmathVec3d]
    survive_reproject_axis_y.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 74
if _libs["survive"].has("survive_reproject_xy", "cdecl"):
    survive_reproject_xy = _libs["survive"].get("survive_reproject_xy", "cdecl")
    survive_reproject_xy.argtypes = [POINTER(BaseStationCal), LinmathVec3d, SurviveAngleReading]
    survive_reproject_xy.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 75
if _libs["survive"].has("survive_reproject_from_pose", "cdecl"):
    survive_reproject_from_pose = _libs["survive"].get("survive_reproject_from_pose", "cdecl")
    survive_reproject_from_pose.argtypes = [POINTER(SurviveContext), c_int, POINTER(SurvivePose), LinmathVec3d, SurviveAngleReading]
    survive_reproject_from_pose.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 78
for _lib in _libs.values():
    if not _lib.has("survive_reproject_full_jac_obj_pose", "cdecl"):
        continue
    survive_reproject_full_jac_obj_pose = _lib.get("survive_reproject_full_jac_obj_pose", "cdecl")
    survive_reproject_full_jac_obj_pose.argtypes = [SurviveAngleReading, POINTER(SurvivePose), LinmathVec3d, POINTER(SurvivePose), POINTER(BaseStationCal)]
    survive_reproject_full_jac_obj_pose.restype = None
    break

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 82
for _lib in _libs.values():
    if not _lib.has("survive_reproject_full_x_jac_obj_pose", "cdecl"):
        continue
    survive_reproject_full_x_jac_obj_pose = _lib.get("survive_reproject_full_x_jac_obj_pose", "cdecl")
    survive_reproject_full_x_jac_obj_pose.argtypes = [SurviveAngleReading, POINTER(SurvivePose), LinmathVec3d, POINTER(SurvivePose), POINTER(BaseStationCal)]
    survive_reproject_full_x_jac_obj_pose.restype = None
    break

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 86
for _lib in _libs.values():
    if not _lib.has("survive_reproject_full_y_jac_obj_pose", "cdecl"):
        continue
    survive_reproject_full_y_jac_obj_pose = _lib.get("survive_reproject_full_y_jac_obj_pose", "cdecl")
    survive_reproject_full_y_jac_obj_pose.argtypes = [SurviveAngleReading, POINTER(SurvivePose), LinmathVec3d, POINTER(SurvivePose), POINTER(BaseStationCal)]
    survive_reproject_full_y_jac_obj_pose.restype = None
    break

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 90
if _libs["survive"].has("survive_reproject_full", "cdecl"):
    survive_reproject_full = _libs["survive"].get("survive_reproject_full", "cdecl")
    survive_reproject_full.argtypes = [POINTER(BaseStationCal), POINTER(SurvivePose), POINTER(SurvivePose), LinmathVec3d, SurviveAngleReading]
    survive_reproject_full.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 101
if _libs["survive"].has("survive_reproject_from_pose_with_bcal", "cdecl"):
    survive_reproject_from_pose_with_bcal = _libs["survive"].get("survive_reproject_from_pose_with_bcal", "cdecl")
    survive_reproject_from_pose_with_bcal.argtypes = [POINTER(BaseStationCal), POINTER(SurvivePose), LinmathVec3d, SurviveAngleReading]
    survive_reproject_from_pose_with_bcal.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 104
if _libs["survive"].has("survive_reproject", "cdecl"):
    survive_reproject = _libs["survive"].get("survive_reproject", "cdecl")
    survive_reproject.argtypes = [POINTER(SurviveContext), c_int, LinmathVec3d, SurviveAngleReading]
    survive_reproject.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 112
if _libs["survive"].has("survive_apply_bsd_calibration", "cdecl"):
    survive_apply_bsd_calibration = _libs["survive"].get("survive_apply_bsd_calibration", "cdecl")
    survive_apply_bsd_calibration.argtypes = [POINTER(SurviveContext), c_int, SurviveAngleReading, SurviveAngleReading]
    survive_apply_bsd_calibration.restype = None

# /home/justin/source/oss/libsurvive/redist/mpfit/mpfit.h: 37
class struct_mp_par_struct(Structure):
    pass

struct_mp_par_struct.__slots__ = [
    'fixed',
    'limited',
    'limits',
    'parname',
    'step',
    'relstep',
    'side',
    'deriv_debug',
    'deriv_reltol',
    'deriv_abstol',
]
struct_mp_par_struct._fields_ = [
    ('fixed', c_int),
    ('limited', c_int * int(2)),
    ('limits', c_double * int(2)),
    ('parname', String),
    ('step', c_double),
    ('relstep', c_double),
    ('side', c_int),
    ('deriv_debug', c_int),
    ('deriv_reltol', c_double),
    ('deriv_abstol', c_double),
]

mp_iterproc = CFUNCTYPE(UNCHECKED(None), )# /home/justin/source/oss/libsurvive/redist/mpfit/mpfit.h: 70

# /home/justin/source/oss/libsurvive/redist/mpfit/mpfit.h: 73
class struct_mp_config_struct(Structure):
    pass

struct_mp_config_struct.__slots__ = [
    'ftol',
    'xtol',
    'gtol',
    'epsfcn',
    'stepfactor',
    'covtol',
    'maxiter',
    'maxfev',
    'nprint',
    'douserscale',
    'nofinitecheck',
    'iterproc',
    'normtol',
]
struct_mp_config_struct._fields_ = [
    ('ftol', c_double),
    ('xtol', c_double),
    ('gtol', c_double),
    ('epsfcn', c_double),
    ('stepfactor', c_double),
    ('covtol', c_double),
    ('maxiter', c_int),
    ('maxfev', c_int),
    ('nprint', c_int),
    ('douserscale', c_int),
    ('nofinitecheck', c_int),
    ('iterproc', mp_iterproc),
    ('normtol', c_double),
]

# /home/justin/source/oss/libsurvive/redist/mpfit/mpfit.h: 105
class struct_mp_result_struct(Structure):
    pass

struct_mp_result_struct.__slots__ = [
    'bestnorm',
    'orignorm',
    'niter',
    'nfev',
    'status',
    'npar',
    'nfree',
    'npegged',
    'nfunc',
    'resid',
    'xerror',
    'covar',
    'version',
]
struct_mp_result_struct._fields_ = [
    ('bestnorm', c_double),
    ('orignorm', c_double),
    ('niter', c_int),
    ('nfev', c_int),
    ('status', c_int),
    ('npar', c_int),
    ('nfree', c_int),
    ('npegged', c_int),
    ('nfunc', c_int),
    ('resid', POINTER(c_double)),
    ('xerror', POINTER(c_double)),
    ('covar', POINTER(c_double)),
    ('version', c_char * int(20)),
]

mp_config = struct_mp_config_struct# /home/justin/source/oss/libsurvive/redist/mpfit/mpfit.h: 128

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 23
class struct_anon_54(Structure):
    pass

struct_anon_54.__slots__ = [
    'value',
    'variance',
    'lh',
    'sensor_idx',
    'axis',
    'object',
    'invalid',
]
struct_anon_54._fields_ = [
    ('value', c_double),
    ('variance', c_double),
    ('lh', c_uint8),
    ('sensor_idx', c_uint8),
    ('axis', c_uint8),
    ('object', c_int),
    ('invalid', c_bool),
]

survive_optimizer_measurement = struct_anon_54# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 23

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 49
class struct_anon_55(Structure):
    pass

struct_anon_55.__slots__ = [
    'total_meas_cnt',
    'total_lh_cnt',
    'dropped_meas_cnt',
    'dropped_lh_cnt',
]
struct_anon_55._fields_ = [
    ('total_meas_cnt', c_uint32),
    ('total_lh_cnt', c_uint32),
    ('dropped_meas_cnt', c_uint32),
    ('dropped_lh_cnt', c_uint32),
]

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 28
class struct_survive_optimizer(Structure):
    pass

struct_survive_optimizer.__slots__ = [
    'reprojectModel',
    'sos',
    'measurements',
    'measurementsCnt',
    'current_bias',
    'initialPose',
    'parameters',
    'parameters_info',
    'poseLength',
    'cameraLength',
    'ptsLength',
    'nofilter',
    'cfg',
    'needsFiltering',
    'stats',
    'user',
    'iteration_cb',
]
struct_survive_optimizer._fields_ = [
    ('reprojectModel', POINTER(survive_reproject_model_t)),
    ('sos', POINTER(POINTER(SurviveObject))),
    ('measurements', POINTER(survive_optimizer_measurement)),
    ('measurementsCnt', c_size_t),
    ('current_bias', c_double),
    ('initialPose', SurvivePose),
    ('parameters', POINTER(c_double)),
    ('parameters_info', POINTER(struct_mp_par_struct)),
    ('poseLength', c_int),
    ('cameraLength', c_int),
    ('ptsLength', c_int),
    ('nofilter', c_bool),
    ('cfg', POINTER(mp_config)),
    ('needsFiltering', c_bool),
    ('stats', struct_anon_55),
    ('user', POINTER(None)),
    ('iteration_cb', CFUNCTYPE(UNCHECKED(None), POINTER(struct_survive_optimizer), c_int, c_int, POINTER(c_double), POINTER(c_double), POINTER(POINTER(c_double)))),
]

survive_optimizer = struct_survive_optimizer# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 58

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 88
if _libs["survive"].has("survive_optimizer_realloc", "cdecl"):
    survive_optimizer_realloc = _libs["survive"].get("survive_optimizer_realloc", "cdecl")
    survive_optimizer_realloc.argtypes = [POINTER(None), c_size_t]
    survive_optimizer_realloc.restype = POINTER(c_ubyte)
    survive_optimizer_realloc.errcheck = lambda v,*a : cast(v, c_void_p)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 90
if _libs["survive"].has("survive_optimizer_get_parameters_count", "cdecl"):
    survive_optimizer_get_parameters_count = _libs["survive"].get("survive_optimizer_get_parameters_count", "cdecl")
    survive_optimizer_get_parameters_count.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_parameters_count.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 92
if _libs["survive"].has("survive_optimizer_get_total_buffer_size", "cdecl"):
    survive_optimizer_get_total_buffer_size = _libs["survive"].get("survive_optimizer_get_total_buffer_size", "cdecl")
    survive_optimizer_get_total_buffer_size.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_total_buffer_size.restype = c_size_t

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 94
if _libs["survive"].has("survive_optimizer_setup_buffers", "cdecl"):
    survive_optimizer_setup_buffers = _libs["survive"].get("survive_optimizer_setup_buffers", "cdecl")
    survive_optimizer_setup_buffers.argtypes = [POINTER(survive_optimizer), POINTER(None), POINTER(None), POINTER(None), POINTER(None)]
    survive_optimizer_setup_buffers.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 98
if _libs["survive"].has("survive_optimizer_get_pose", "cdecl"):
    survive_optimizer_get_pose = _libs["survive"].get("survive_optimizer_get_pose", "cdecl")
    survive_optimizer_get_pose.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_pose.restype = POINTER(SurvivePose)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 100
if _libs["survive"].has("survive_optimizer_get_camera_index", "cdecl"):
    survive_optimizer_get_camera_index = _libs["survive"].get("survive_optimizer_get_camera_index", "cdecl")
    survive_optimizer_get_camera_index.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_camera_index.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 102
if _libs["survive"].has("survive_optimizer_get_camera", "cdecl"):
    survive_optimizer_get_camera = _libs["survive"].get("survive_optimizer_get_camera", "cdecl")
    survive_optimizer_get_camera.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_camera.restype = POINTER(SurvivePose)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 104
if _libs["survive"].has("survive_optimizer_get_calibration_index", "cdecl"):
    survive_optimizer_get_calibration_index = _libs["survive"].get("survive_optimizer_get_calibration_index", "cdecl")
    survive_optimizer_get_calibration_index.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_calibration_index.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 106
if _libs["survive"].has("survive_optimizer_get_calibration", "cdecl"):
    survive_optimizer_get_calibration = _libs["survive"].get("survive_optimizer_get_calibration", "cdecl")
    survive_optimizer_get_calibration.argtypes = [POINTER(survive_optimizer), c_int]
    survive_optimizer_get_calibration.restype = POINTER(BaseStationCal)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 108
if _libs["survive"].has("survive_optimizer_get_sensors_index", "cdecl"):
    survive_optimizer_get_sensors_index = _libs["survive"].get("survive_optimizer_get_sensors_index", "cdecl")
    survive_optimizer_get_sensors_index.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_sensors_index.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 110
if _libs["survive"].has("survive_optimizer_get_sensors", "cdecl"):
    survive_optimizer_get_sensors = _libs["survive"].get("survive_optimizer_get_sensors", "cdecl")
    survive_optimizer_get_sensors.argtypes = [POINTER(survive_optimizer), c_size_t]
    survive_optimizer_get_sensors.restype = POINTER(c_double)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 112
if _libs["survive"].has("survive_optimizer_setup_pose_n", "cdecl"):
    survive_optimizer_setup_pose_n = _libs["survive"].get("survive_optimizer_setup_pose_n", "cdecl")
    survive_optimizer_setup_pose_n.argtypes = [POINTER(survive_optimizer), POINTER(SurvivePose), c_size_t, c_bool, c_int]
    survive_optimizer_setup_pose_n.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 115
if _libs["survive"].has("survive_optimizer_fix_camera", "cdecl"):
    survive_optimizer_fix_camera = _libs["survive"].get("survive_optimizer_fix_camera", "cdecl")
    survive_optimizer_fix_camera.argtypes = [POINTER(survive_optimizer), c_int]
    survive_optimizer_fix_camera.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 117
if _libs["survive"].has("survive_optimizer_setup_pose", "cdecl"):
    survive_optimizer_setup_pose = _libs["survive"].get("survive_optimizer_setup_pose", "cdecl")
    survive_optimizer_setup_pose.argtypes = [POINTER(survive_optimizer), POINTER(SurvivePose), c_bool, c_int]
    survive_optimizer_setup_pose.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 120
if _libs["survive"].has("survive_optimizer_setup_camera", "cdecl"):
    survive_optimizer_setup_camera = _libs["survive"].get("survive_optimizer_setup_camera", "cdecl")
    survive_optimizer_setup_camera.argtypes = [POINTER(survive_optimizer), c_int8, POINTER(SurvivePose), c_bool, c_int]
    survive_optimizer_setup_camera.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 123
if _libs["survive"].has("survive_optimizer_setup_cameras", "cdecl"):
    survive_optimizer_setup_cameras = _libs["survive"].get("survive_optimizer_setup_cameras", "cdecl")
    survive_optimizer_setup_cameras.argtypes = [POINTER(survive_optimizer), POINTER(SurviveContext), c_bool, c_int]
    survive_optimizer_setup_cameras.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 126
if _libs["survive"].has("survive_optimizer_error", "cdecl"):
    survive_optimizer_error = _libs["survive"].get("survive_optimizer_error", "cdecl")
    survive_optimizer_error.argtypes = [c_int]
    survive_optimizer_error.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 128
if _libs["survive"].has("survive_optimizer_run", "cdecl"):
    survive_optimizer_run = _libs["survive"].get("survive_optimizer_run", "cdecl")
    survive_optimizer_run.argtypes = [POINTER(survive_optimizer), POINTER(struct_mp_result_struct)]
    survive_optimizer_run.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 130
if _libs["survive"].has("survive_optimizer_set_reproject_model", "cdecl"):
    survive_optimizer_set_reproject_model = _libs["survive"].get("survive_optimizer_set_reproject_model", "cdecl")
    survive_optimizer_set_reproject_model.argtypes = [POINTER(survive_optimizer), POINTER(survive_reproject_model_t)]
    survive_optimizer_set_reproject_model.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 133
if _libs["survive"].has("survive_optimizer_serialize", "cdecl"):
    survive_optimizer_serialize = _libs["survive"].get("survive_optimizer_serialize", "cdecl")
    survive_optimizer_serialize.argtypes = [POINTER(survive_optimizer), String]
    survive_optimizer_serialize.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 135
if _libs["survive"].has("survive_optimizer_load", "cdecl"):
    survive_optimizer_load = _libs["survive"].get("survive_optimizer_load", "cdecl")
    survive_optimizer_load.argtypes = [String]
    survive_optimizer_load.restype = POINTER(survive_optimizer)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 137
if _libs["survive"].has("survive_optimizer_current_norm", "cdecl"):
    survive_optimizer_current_norm = _libs["survive"].get("survive_optimizer_current_norm", "cdecl")
    survive_optimizer_current_norm.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_current_norm.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 139
if _libs["survive"].has("survive_optimizer_precise_config", "cdecl"):
    survive_optimizer_precise_config = _libs["survive"].get("survive_optimizer_precise_config", "cdecl")
    survive_optimizer_precise_config.argtypes = []
    survive_optimizer_precise_config.restype = POINTER(mp_config)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 141
if _libs["survive"].has("survive_optimizer_nonfixed_cnt", "cdecl"):
    survive_optimizer_nonfixed_cnt = _libs["survive"].get("survive_optimizer_nonfixed_cnt", "cdecl")
    survive_optimizer_nonfixed_cnt.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_nonfixed_cnt.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 143
if _libs["survive"].has("survive_optimizer_get_nonfixed", "cdecl"):
    survive_optimizer_get_nonfixed = _libs["survive"].get("survive_optimizer_get_nonfixed", "cdecl")
    survive_optimizer_get_nonfixed.argtypes = [POINTER(survive_optimizer), POINTER(c_double)]
    survive_optimizer_get_nonfixed.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 144
if _libs["survive"].has("survive_optimizer_set_nonfixed", "cdecl"):
    survive_optimizer_set_nonfixed = _libs["survive"].get("survive_optimizer_set_nonfixed", "cdecl")
    survive_optimizer_set_nonfixed.argtypes = [POINTER(survive_optimizer), POINTER(c_double)]
    survive_optimizer_set_nonfixed.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject_gen2.h: 23
if _libs["survive"].has("survive_reproject_axis_x_gen2", "cdecl"):
    survive_reproject_axis_x_gen2 = _libs["survive"].get("survive_reproject_axis_x_gen2", "cdecl")
    survive_reproject_axis_x_gen2.argtypes = [POINTER(BaseStationCal), LinmathVec3d]
    survive_reproject_axis_x_gen2.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject_gen2.h: 24
if _libs["survive"].has("survive_reproject_axis_y_gen2", "cdecl"):
    survive_reproject_axis_y_gen2 = _libs["survive"].get("survive_reproject_axis_y_gen2", "cdecl")
    survive_reproject_axis_y_gen2.argtypes = [POINTER(BaseStationCal), LinmathVec3d]
    survive_reproject_axis_y_gen2.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject_gen2.h: 26
if _libs["survive"].has("survive_reproject_xy_gen2", "cdecl"):
    survive_reproject_xy_gen2 = _libs["survive"].get("survive_reproject_xy_gen2", "cdecl")
    survive_reproject_xy_gen2.argtypes = [POINTER(BaseStationCal), LinmathVec3d, SurviveAngleReading]
    survive_reproject_xy_gen2.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject_gen2.h: 28
if _libs["survive"].has("survive_reproject_from_pose_gen2", "cdecl"):
    survive_reproject_from_pose_gen2 = _libs["survive"].get("survive_reproject_from_pose_gen2", "cdecl")
    survive_reproject_from_pose_gen2.argtypes = [POINTER(SurviveContext), c_int, POINTER(SurvivePose), LinmathVec3d, SurviveAngleReading]
    survive_reproject_from_pose_gen2.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject_gen2.h: 32
if _libs["survive"].has("survive_reproject_gen2", "cdecl"):
    survive_reproject_gen2 = _libs["survive"].get("survive_reproject_gen2", "cdecl")
    survive_reproject_gen2.argtypes = [POINTER(SurviveContext), c_int, LinmathVec3d, SurviveAngleReading]
    survive_reproject_gen2.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject_gen2.h: 35
for _lib in _libs.values():
    if not _lib.has("survive_reproject_full_jac_obj_pose_gen2", "cdecl"):
        continue
    survive_reproject_full_jac_obj_pose_gen2 = _lib.get("survive_reproject_full_jac_obj_pose_gen2", "cdecl")
    survive_reproject_full_jac_obj_pose_gen2.argtypes = [SurviveAngleReading, POINTER(SurvivePose), LinmathVec3d, POINTER(SurvivePose), POINTER(BaseStationCal)]
    survive_reproject_full_jac_obj_pose_gen2.restype = None
    break

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject_gen2.h: 39
if _libs["survive"].has("survive_reproject_full_gen2", "cdecl"):
    survive_reproject_full_gen2 = _libs["survive"].get("survive_reproject_full_gen2", "cdecl")
    survive_reproject_full_gen2.argtypes = [POINTER(BaseStationCal), POINTER(SurvivePose), POINTER(SurvivePose), LinmathVec3d, SurviveAngleReading]
    survive_reproject_full_gen2.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject_gen2.h: 42
try:
    survive_reproject_gen2_model = (survive_reproject_model_t).in_dll(_libs["survive"], "survive_reproject_gen2_model")
except:
    pass

survive_kalman_model_t = struct_survive_kalman_model_t# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 107

SurviveObject = struct_SurviveObject# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 103

SurviveContext = struct_SurviveContext# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 287

BaseStationData = struct_BaseStationData# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 226

SurviveCalData = struct_SurviveCalData# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 193

PoserDataIMU = struct_PoserDataIMU# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 96

PoserDataLight = struct_PoserDataLight# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 106

PoserDataLightGen1 = struct_PoserDataLightGen1# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 113

PoserDataLightGen2 = struct_PoserDataLightGen2# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 120

PoserDataGlobalScene = struct_PoserDataGlobalScene# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 129

PoserDataGlobalScenes = struct_PoserDataGlobalScenes# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 143

PoserDataAll = union_PoserDataAll# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 145

SurviveSensorActivations_s = struct_SurviveSensorActivations_s# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 27

survive_threaded_poser = struct_survive_threaded_poser# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 161

SurviveSimpleContext = struct_SurviveSimpleContext# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 11

SurviveSimpleObject = struct_SurviveSimpleObject# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 24

SurviveSimpleEvent = struct_SurviveSimpleEvent# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 166

SurviveSimpleButtonEvent = struct_SurviveSimpleButtonEvent# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 42

SurviveKalmanTracker = struct_SurviveKalmanTracker# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 185

BaseStationCal = struct_BaseStationCal# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 224

config_group = struct_config_group# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 246

SurviveRecordingData = struct_SurviveRecordingData# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 276

survive_reproject_model_t = struct_survive_reproject_model_t# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 67

survive_optimizer = struct_survive_optimizer# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 28

# No inserted files

# Begin prefix-stripping

# Strip prefixes from all symbols following regular expression:
# (survive_|Survive)

import re as __re_module

__strip_expr = __re_module.compile('(survive_|Survive)')
for __k, __v in globals().copy().items():
    __m = __strip_expr.match(__k)
    if __m:
        globals()[__k[__m.end():]] = __v
        # remove symbol with prefix(?)
        # globals().pop(__k)
del __re_module, __k, __v, __m, __strip_expr

# End prefix-stripping

