r"""Wrapper for poser.h

Generated with:
/home/justin/.local/bin/ctypesgen /home/justin/source/oss/libsurvive/include/libsurvive/poser.h /home/justin/source/oss/libsurvive/include/libsurvive/survive.h /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject_gen2.h /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h -I/home/justin/source/oss/libsurvive/redist -I/home/justin/source/oss/libsurvive/include/libsurvive -I/home/justin/source/oss/libsurvive/include --no-macros -L/home/justin/source/oss/libsurvive/bin -lsurvive --strip-prefix=survive_ -P Survive -o /home/justin/source/oss/libsurvive/bin/../bindings/python/pysurvive/pysurvive_generated.py

Do not modify this file.
"""

__docformat__ = "restructuredtext"

# Begin preamble for Python

import ctypes
import sys
from ctypes import *  # noqa: F401, F403

_int_types = (ctypes.c_int16, ctypes.c_int32)
if hasattr(ctypes, "c_int64"):
    # Some builds of ctypes apparently do not have ctypes.c_int64
    # defined; it's a pretty good bet that these builds do not
    # have 64-bit pointers.
    _int_types += (ctypes.c_int64,)
for t in _int_types:
    if ctypes.sizeof(t) == ctypes.sizeof(ctypes.c_size_t):
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


class String(MutableString, ctypes.Union):

    _fields_ = [("raw", ctypes.POINTER(ctypes.c_char)), ("data", ctypes.c_char_p)]

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
            return cls(ctypes.POINTER(ctypes.c_char)())

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
        elif isinstance(obj, ctypes.c_char_p):
            return obj

        # Convert from POINTER(ctypes.c_char)
        elif isinstance(obj, ctypes.POINTER(ctypes.c_char)):
            return obj

        # Convert from raw pointer
        elif isinstance(obj, int):
            return cls(ctypes.cast(obj, ctypes.POINTER(ctypes.c_char)))

        # Convert from ctypes.c_char array
        elif isinstance(obj, ctypes.c_char * len(obj)):
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
# typechecked, and will be converted to ctypes.c_void_p.
def UNCHECKED(type):
    if hasattr(type, "_type_") and isinstance(type._type_, str) and type._type_ != "P":
        return type
    else:
        return ctypes.c_void_p


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

_libs = {}
_libdirs = ['/home/justin/source/oss/libsurvive/bin']

# Begin loader

"""
Load libraries - appropriately for all our supported platforms
"""
# ----------------------------------------------------------------------------
# Copyright (c) 2008 David James
# Copyright (c) 2006-2008 Alex Holkner
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
#  * Neither the name of pyglet nor the names of its
#    contributors may be used to endorse or promote products
#    derived from this software without specific prior written
#    permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
# ----------------------------------------------------------------------------

import ctypes
import ctypes.util
import glob
import os.path
import platform
import re
import sys


def _environ_path(name):
    """Split an environment variable into a path-like list elements"""
    if name in os.environ:
        return os.environ[name].split(":")
    return []


class LibraryLoader:
    """
    A base class For loading of libraries ;-)
    Subclasses load libraries for specific platforms.
    """

    # library names formatted specifically for platforms
    name_formats = ["%s"]

    class Lookup:
        """Looking up calling conventions for a platform"""

        mode = ctypes.DEFAULT_MODE

        def __init__(self, path):
            super(LibraryLoader.Lookup, self).__init__()
            self.access = dict(cdecl=ctypes.CDLL(path, self.mode))

        def get(self, name, calling_convention="cdecl"):
            """Return the given name according to the selected calling convention"""
            if calling_convention not in self.access:
                raise LookupError(
                    "Unknown calling convention '{}' for function '{}'".format(
                        calling_convention, name
                    )
                )
            return getattr(self.access[calling_convention], name)

        def has(self, name, calling_convention="cdecl"):
            """Return True if this given calling convention finds the given 'name'"""
            if calling_convention not in self.access:
                return False
            return hasattr(self.access[calling_convention], name)

        def __getattr__(self, name):
            return getattr(self.access["cdecl"], name)

    def __init__(self):
        self.other_dirs = []

    def __call__(self, libname):
        """Given the name of a library, load it."""
        paths = self.getpaths(libname)

        for path in paths:
            # noinspection PyBroadException
            try:
                return self.Lookup(path)
            except Exception:  # pylint: disable=broad-except
                pass

        raise ImportError("Could not load %s." % libname)

    def getpaths(self, libname):
        """Return a list of paths where the library might be found."""
        if os.path.isabs(libname):
            yield libname
        else:
            # search through a prioritized series of locations for the library

            # we first search any specific directories identified by user
            for dir_i in self.other_dirs:
                for fmt in self.name_formats:
                    # dir_i should be absolute already
                    yield os.path.join(dir_i, fmt % libname)

            # check if this code is even stored in a physical file
            try:
                this_file = __file__
            except NameError:
                this_file = None

            # then we search the directory where the generated python interface is stored
            if this_file is not None:
                for fmt in self.name_formats:
                    yield os.path.abspath(os.path.join(os.path.dirname(__file__), fmt % libname))

            # now, use the ctypes tools to try to find the library
            for fmt in self.name_formats:
                path = ctypes.util.find_library(fmt % libname)
                if path:
                    yield path

            # then we search all paths identified as platform-specific lib paths
            for path in self.getplatformpaths(libname):
                yield path

            # Finally, we'll try the users current working directory
            for fmt in self.name_formats:
                yield os.path.abspath(os.path.join(os.path.curdir, fmt % libname))

    def getplatformpaths(self, _libname):  # pylint: disable=no-self-use
        """Return all the library paths available in this platform"""
        return []


# Darwin (Mac OS X)


class DarwinLibraryLoader(LibraryLoader):
    """Library loader for MacOS"""

    name_formats = [
        "lib%s.dylib",
        "lib%s.so",
        "lib%s.bundle",
        "%s.dylib",
        "%s.so",
        "%s.bundle",
        "%s",
    ]

    class Lookup(LibraryLoader.Lookup):
        """
        Looking up library files for this platform (Darwin aka MacOS)
        """

        # Darwin requires dlopen to be called with mode RTLD_GLOBAL instead
        # of the default RTLD_LOCAL.  Without this, you end up with
        # libraries not being loadable, resulting in "Symbol not found"
        # errors
        mode = ctypes.RTLD_GLOBAL

    def getplatformpaths(self, libname):
        if os.path.pathsep in libname:
            names = [libname]
        else:
            names = [fmt % libname for fmt in self.name_formats]

        for directory in self.getdirs(libname):
            for name in names:
                yield os.path.join(directory, name)

    @staticmethod
    def getdirs(libname):
        """Implements the dylib search as specified in Apple documentation:

        http://developer.apple.com/documentation/DeveloperTools/Conceptual/
            DynamicLibraries/Articles/DynamicLibraryUsageGuidelines.html

        Before commencing the standard search, the method first checks
        the bundle's ``Frameworks`` directory if the application is running
        within a bundle (OS X .app).
        """

        dyld_fallback_library_path = _environ_path("DYLD_FALLBACK_LIBRARY_PATH")
        if not dyld_fallback_library_path:
            dyld_fallback_library_path = [
                os.path.expanduser("~/lib"),
                "/usr/local/lib",
                "/usr/lib",
            ]

        dirs = []

        if "/" in libname:
            dirs.extend(_environ_path("DYLD_LIBRARY_PATH"))
        else:
            dirs.extend(_environ_path("LD_LIBRARY_PATH"))
            dirs.extend(_environ_path("DYLD_LIBRARY_PATH"))
            dirs.extend(_environ_path("LD_RUN_PATH"))

        if hasattr(sys, "frozen") and getattr(sys, "frozen") == "macosx_app":
            dirs.append(os.path.join(os.environ["RESOURCEPATH"], "..", "Frameworks"))

        dirs.extend(dyld_fallback_library_path)

        return dirs


# Posix


class PosixLibraryLoader(LibraryLoader):
    """Library loader for POSIX-like systems (including Linux)"""

    _ld_so_cache = None

    _include = re.compile(r"^\s*include\s+(?P<pattern>.*)")

    name_formats = ["lib%s.so", "%s.so", "%s"]

    class _Directories(dict):
        """Deal with directories"""

        def __init__(self):
            dict.__init__(self)
            self.order = 0

        def add(self, directory):
            """Add a directory to our current set of directories"""
            if len(directory) > 1:
                directory = directory.rstrip(os.path.sep)
            # only adds and updates order if exists and not already in set
            if not os.path.exists(directory):
                return
            order = self.setdefault(directory, self.order)
            if order == self.order:
                self.order += 1

        def extend(self, directories):
            """Add a list of directories to our set"""
            for a_dir in directories:
                self.add(a_dir)

        def ordered(self):
            """Sort the list of directories"""
            return (i[0] for i in sorted(self.items(), key=lambda d: d[1]))

    def _get_ld_so_conf_dirs(self, conf, dirs):
        """
        Recursive function to help parse all ld.so.conf files, including proper
        handling of the `include` directive.
        """

        try:
            with open(conf) as fileobj:
                for dirname in fileobj:
                    dirname = dirname.strip()
                    if not dirname:
                        continue

                    match = self._include.match(dirname)
                    if not match:
                        dirs.add(dirname)
                    else:
                        for dir2 in glob.glob(match.group("pattern")):
                            self._get_ld_so_conf_dirs(dir2, dirs)
        except IOError:
            pass

    def _create_ld_so_cache(self):
        # Recreate search path followed by ld.so.  This is going to be
        # slow to build, and incorrect (ld.so uses ld.so.cache, which may
        # not be up-to-date).  Used only as fallback for distros without
        # /sbin/ldconfig.
        #
        # We assume the DT_RPATH and DT_RUNPATH binary sections are omitted.

        directories = self._Directories()
        for name in (
            "LD_LIBRARY_PATH",
            "SHLIB_PATH",  # HP-UX
            "LIBPATH",  # OS/2, AIX
            "LIBRARY_PATH",  # BE/OS
        ):
            if name in os.environ:
                directories.extend(os.environ[name].split(os.pathsep))

        self._get_ld_so_conf_dirs("/etc/ld.so.conf", directories)

        bitage = platform.architecture()[0]

        unix_lib_dirs_list = []
        if bitage.startswith("64"):
            # prefer 64 bit if that is our arch
            unix_lib_dirs_list += ["/lib64", "/usr/lib64"]

        # must include standard libs, since those paths are also used by 64 bit
        # installs
        unix_lib_dirs_list += ["/lib", "/usr/lib"]
        if sys.platform.startswith("linux"):
            # Try and support multiarch work in Ubuntu
            # https://wiki.ubuntu.com/MultiarchSpec
            if bitage.startswith("32"):
                # Assume Intel/AMD x86 compat
                unix_lib_dirs_list += ["/lib/i386-linux-gnu", "/usr/lib/i386-linux-gnu"]
            elif bitage.startswith("64"):
                # Assume Intel/AMD x86 compatible
                unix_lib_dirs_list += [
                    "/lib/x86_64-linux-gnu",
                    "/usr/lib/x86_64-linux-gnu",
                ]
            else:
                # guess...
                unix_lib_dirs_list += glob.glob("/lib/*linux-gnu")
        directories.extend(unix_lib_dirs_list)

        cache = {}
        lib_re = re.compile(r"lib(.*)\.s[ol]")
        # ext_re = re.compile(r"\.s[ol]$")
        for our_dir in directories.ordered():
            try:
                for path in glob.glob("%s/*.s[ol]*" % our_dir):
                    file = os.path.basename(path)

                    # Index by filename
                    cache_i = cache.setdefault(file, set())
                    cache_i.add(path)

                    # Index by library name
                    match = lib_re.match(file)
                    if match:
                        library = match.group(1)
                        cache_i = cache.setdefault(library, set())
                        cache_i.add(path)
            except OSError:
                pass

        self._ld_so_cache = cache

    def getplatformpaths(self, libname):
        if self._ld_so_cache is None:
            self._create_ld_so_cache()

        result = self._ld_so_cache.get(libname, set())
        for i in result:
            # we iterate through all found paths for library, since we may have
            # actually found multiple architectures or other library types that
            # may not load
            yield i


# Windows


class WindowsLibraryLoader(LibraryLoader):
    """Library loader for Microsoft Windows"""

    name_formats = ["%s.dll", "lib%s.dll", "%slib.dll", "%s"]

    class Lookup(LibraryLoader.Lookup):
        """Lookup class for Windows libraries..."""

        def __init__(self, path):
            super(WindowsLibraryLoader.Lookup, self).__init__(path)
            self.access["stdcall"] = ctypes.windll.LoadLibrary(path)


# Platform switching

# If your value of sys.platform does not appear in this dict, please contact
# the Ctypesgen maintainers.

loaderclass = {
    "darwin": DarwinLibraryLoader,
    "cygwin": WindowsLibraryLoader,
    "win32": WindowsLibraryLoader,
    "msys": WindowsLibraryLoader,
}

load_library = loaderclass.get(sys.platform, PosixLibraryLoader)()


def add_library_search_dirs(other_dirs):
    """
    Add libraries to search paths.
    If library paths are relative, convert them to absolute with respect to this
    file's directory
    """
    for path in other_dirs:
        if not os.path.isabs(path):
            path = os.path.abspath(path)
        load_library.other_dirs.append(path)


del loaderclass


# Begin libraries
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

# /usr/local/include/cnmatrix/cn_matrix.h: 54
class struct_CnMat(Structure):
    pass

struct_CnMat.__slots__ = [
    'step',
    'data',
    'rows',
    'cols',
]
struct_CnMat._fields_ = [
    ('step', c_int),
    ('data', POINTER(c_double)),
    ('rows', c_int),
    ('cols', c_int),
]

LinmathQuat = c_double * int(4)# /home/justin/source/oss/libsurvive/redist/linmath.h: 48

LinmathPoint3d = c_double * int(3)# /home/justin/source/oss/libsurvive/redist/linmath.h: 50

LinmathVec3d = c_double * int(3)# /home/justin/source/oss/libsurvive/redist/linmath.h: 51

LinmathAxisAngle = c_double * int(3)# /home/justin/source/oss/libsurvive/redist/linmath.h: 53

LinmathAxisAngleMag = c_double * int(3)# /home/justin/source/oss/libsurvive/redist/linmath.h: 54

# /home/justin/source/oss/libsurvive/redist/linmath.h: 64
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

LinmathPose = struct_LinmathPose# /home/justin/source/oss/libsurvive/redist/linmath.h: 64

# /home/justin/source/oss/libsurvive/redist/linmath.h: 69
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

LinmathAxisAnglePose = struct_LinmathAxisAnglePose# /home/justin/source/oss/libsurvive/redist/linmath.h: 69

SurvivePose = LinmathPose# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 114

SurviveAngularVelocity = LinmathAxisAngleMag# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 115

SurviveVelocity = LinmathAxisAnglePose# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 116

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 126
class struct_survive_kalman_model_t(Structure):
    pass

struct_survive_kalman_model_t.__slots__ = [
    'Pose',
    'Velocity',
    'Acc',
    'AccScale',
    'IMUCorrection',
    'AccBias',
    'GyroBias',
]
struct_survive_kalman_model_t._fields_ = [
    ('Pose', SurvivePose),
    ('Velocity', SurviveVelocity),
    ('Acc', LinmathVec3d),
    ('AccScale', c_double),
    ('IMUCorrection', LinmathQuat),
    ('AccBias', LinmathVec3d),
    ('GyroBias', LinmathVec3d),
]

SurviveKalmanModel = struct_survive_kalman_model_t# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 126

enum_SurviveInputEvent = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 137

SURVIVE_INPUT_EVENT_NONE = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 137

SURVIVE_INPUT_EVENT_BUTTON_FLAG = 2# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 137

SURVIVE_INPUT_EVENT_BUTTON_DOWN = 3# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 137

SURVIVE_INPUT_EVENT_BUTTON_UP = 2# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 137

SURVIVE_INPUT_EVENT_TOUCH_FLAG = 4# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 137

SURVIVE_INPUT_EVENT_TOUCH_DOWN = 5# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 137

SURVIVE_INPUT_EVENT_TOUCH_UP = 4# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 137

SURVIVE_INPUT_EVENT_AXIS_CHANGED = 8# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 137

enum_SurviveButton = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 148

SURVIVE_BUTTON_UNKNOWN = 255# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 148

SURVIVE_BUTTON_TRIGGER = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 148

SURVIVE_BUTTON_TRACKPAD = 1# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 148

SURVIVE_BUTTON_THUMBSTICK = 2# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 148

SURVIVE_BUTTON_SYSTEM = 3# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 148

SURVIVE_BUTTON_A = 4# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 148

SURVIVE_BUTTON_B = 5# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 148

SURVIVE_BUTTON_MENU = 6# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 148

SURVIVE_BUTTON_GRIP = 7# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 148

SURVIVE_BUTTON_ON_FACE = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 148

SURVIVE_BUTTON_MAX = 16# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 148

enum_SurviveAxis = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 165

SURVIVE_AXIS_UNKNOWN = 255# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 165

SURVIVE_AXIS_TRIGGER = 1# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 165

SURVIVE_AXIS_TRACKPAD_X = 2# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 165

SURVIVE_AXIS_TRACKPAD_Y = 3# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 165

SURVIVE_AXIS_MIDDLE_FINGER_PROXIMITY = 4# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 165

SURVIVE_AXIS_RING_FINGER_PROXIMITY = 5# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 165

SURVIVE_AXIS_PINKY_FINGER_PROXIMITY = 6# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 165

SURVIVE_AXIS_TRIGGER_FINGER_PROXIMITY = 7# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 165

SURVIVE_AXIS_GRIP_FORCE = 8# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 165

SURVIVE_AXIS_TRACKPAD_FORCE = 9# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 165

SURVIVE_AXIS_JOYSTICK_X = 10# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 165

SURVIVE_AXIS_JOYSTICK_Y = 11# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 165

SURVIVE_AXIS_IPD = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 165

SURVIVE_AXIS_FACE_PROXIMITY = 1# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 165

SurviveAxisVal_t = c_double# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 182

enum_anon_26 = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 189

SURVIVE_OBJECT_TYPE_UNKNOWN = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 189

SURVIVE_OBJECT_TYPE_HMD = (SURVIVE_OBJECT_TYPE_UNKNOWN + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 189

SURVIVE_OBJECT_TYPE_CONTROLLER = (SURVIVE_OBJECT_TYPE_HMD + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 189

SURVIVE_OBJECT_TYPE_OTHER = (SURVIVE_OBJECT_TYPE_CONTROLLER + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 189

SurviveObjectType = enum_anon_26# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 189

enum_anon_27 = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

SURVIVE_OBJECT_SUBTYPE_GENERIC = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

SURVIVE_OBJECT_SUBTYPE_VIVE_HMD = (SURVIVE_OBJECT_SUBTYPE_GENERIC + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

SURVIVE_OBJECT_SUBTYPE_INDEX_HMD = (SURVIVE_OBJECT_SUBTYPE_VIVE_HMD + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

SURVIVE_OBJECT_SUBTYPE_WAND = (SURVIVE_OBJECT_SUBTYPE_INDEX_HMD + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

SURVIVE_OBJECT_SUBTYPE_KNUCKLES_R = (SURVIVE_OBJECT_SUBTYPE_WAND + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

SURVIVE_OBJECT_SUBTYPE_KNUCKLES_L = (SURVIVE_OBJECT_SUBTYPE_KNUCKLES_R + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

SURVIVE_OBJECT_SUBTYPE_TRACKER = (SURVIVE_OBJECT_SUBTYPE_KNUCKLES_L + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

SURVIVE_OBJECT_SUBTYPE_TRACKER_GEN2 = (SURVIVE_OBJECT_SUBTYPE_TRACKER + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

SURVIVE_OBJECT_SUBTYPE_COUNT = (SURVIVE_OBJECT_SUBTYPE_TRACKER_GEN2 + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

SurviveObjectSubtype = enum_anon_27# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 201

survive_timecode = c_uint32# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 203

survive_long_timecode = c_uint64# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 204

survive_channel = c_uint8# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 207

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 209
if _libs["survive"].has("survive_timecode_difference", "cdecl"):
    survive_timecode_difference = _libs["survive"].get("survive_timecode_difference", "cdecl")
    survive_timecode_difference.argtypes = [survive_timecode, survive_timecode]
    survive_timecode_difference.restype = survive_timecode

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 128
class struct_SurviveObject(Structure):
    pass

SurviveObject = struct_SurviveObject# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 211

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 321
class struct_SurviveContext(Structure):
    pass

SurviveContext = struct_SurviveContext# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 212

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 256
class struct_BaseStationData(Structure):
    pass

BaseStationData = struct_BaseStationData# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 213

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 214
class struct_SurviveCalData(Structure):
    pass

SurviveCalData = struct_SurviveCalData# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 214

enum_anon_28 = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 222

SURVIVE_OK = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 222

SURVIVE_ERROR_GENERAL = (-1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 222

SURVIVE_ERROR_NO_TRACKABLE_OBJECTS = (-2)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 222

SURVIVE_ERROR_HARWARE_FAULT = (-3)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 222

SURVIVE_ERROR_INVALID_CONFIG = (-4)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 222

SurviveError = enum_anon_28# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 222

enum_anon_29 = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 228

SURVIVE_LOG_LEVEL_ERROR = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 228

SURVIVE_LOG_LEVEL_WARNING = 1# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 228

SURVIVE_LOG_LEVEL_INFO = 2# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 228

SurviveLogLevel = enum_anon_29# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 228

survive_driver_fn = CFUNCTYPE(UNCHECKED(None), )# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 230

datalog_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), String, POINTER(c_double), c_size_t)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 232

disconnect_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 233

printf_process_func = CFUNCTYPE(UNCHECKED(c_int), POINTER(SurviveContext), String)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 234

log_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveContext), SurviveLogLevel, String)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 235

report_error_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveContext), SurviveError)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 236

config_process_func = CFUNCTYPE(UNCHECKED(c_int), POINTER(SurviveObject), String, c_int)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 238

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 248
class struct_anon_30(Structure):
    pass

struct_anon_30.__slots__ = [
    'sensor_id',
    'length',
    'timestamp',
]
struct_anon_30._fields_ = [
    ('sensor_id', c_uint8),
    ('length', c_uint16),
    ('timestamp', c_uint32),
]

LightcapElement = struct_anon_30# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 248

gen_detected_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), c_int)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 254

lightcap_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), POINTER(LightcapElement))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 260

light_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), c_int, c_int, c_int, survive_timecode, survive_timecode, c_uint32)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 266

ootx_received_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(struct_SurviveContext), c_uint8)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 269

light_pulse_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), c_int, c_int, survive_timecode, c_double, c_uint32)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 271

angle_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), c_int, c_int, survive_timecode, c_double, c_double, c_uint32)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 277

sync_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), survive_channel, survive_timecode, c_bool, c_bool)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 285

sweep_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), survive_channel, c_int, survive_timecode, c_bool)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 291

sweep_angle_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), survive_channel, c_int, survive_timecode, c_int8, c_double)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 297

raw_imu_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), c_int, POINTER(c_double), survive_timecode, c_int)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 304

imu_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), c_int, POINTER(c_double), survive_timecode, c_int)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 313

button_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), enum_SurviveInputEvent, enum_SurviveButton, POINTER(enum_SurviveAxis), POINTER(SurviveAxisVal_t))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 318

pose_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), survive_long_timecode, POINTER(SurvivePose))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 324

imupose_process_func = pose_process_func# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 329

velocity_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject), survive_long_timecode, POINTER(SurviveVelocity))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 334

external_pose_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveContext), String, POINTER(SurvivePose))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 340

external_velocity_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveContext), String, POINTER(SurviveVelocity))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 341

lighthouse_pose_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveContext), c_uint8, POINTER(SurvivePose))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 346

new_object_process_func = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveObject))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 351

haptic_func = CFUNCTYPE(UNCHECKED(c_int), POINTER(SurviveObject), c_double, c_double, c_double)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 354

DeviceDriver = CFUNCTYPE(UNCHECKED(c_int), POINTER(SurviveContext))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 358

enum_SurviveDeviceDriverReturn = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 363

SURVIVE_DRIVER_NORMAL = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 363

SURVIVE_DRIVER_ERROR = (-1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 363

SURVIVE_DRIVER_PASSIVE = 1# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 363

SurviveDeviceDriverReturn = enum_SurviveDeviceDriverReturn# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 363

DeviceDriverCb = CFUNCTYPE(UNCHECKED(c_int), POINTER(struct_SurviveContext), POINTER(None))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 365

DeviceDriverMagicCb = CFUNCTYPE(UNCHECKED(c_int), POINTER(struct_SurviveContext), POINTER(None), c_int, POINTER(None), c_int)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 366

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 368
if _libs["survive"].has("SurviveInputEventStr", "cdecl"):
    SurviveInputEventStr = _libs["survive"].get("SurviveInputEventStr", "cdecl")
    SurviveInputEventStr.argtypes = [enum_SurviveInputEvent]
    SurviveInputEventStr.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 369
if _libs["survive"].has("SurviveButtonsStr", "cdecl"):
    SurviveButtonsStr = _libs["survive"].get("SurviveButtonsStr", "cdecl")
    SurviveButtonsStr.argtypes = [SurviveObjectSubtype, enum_SurviveButton]
    SurviveButtonsStr.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 370
if _libs["survive"].has("SurviveAxisStr", "cdecl"):
    SurviveAxisStr = _libs["survive"].get("SurviveAxisStr", "cdecl")
    SurviveAxisStr.argtypes = [SurviveObjectSubtype, enum_SurviveAxis]
    SurviveAxisStr.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 372
if _libs["survive"].has("SurviveObjectTypeStr", "cdecl"):
    SurviveObjectTypeStr = _libs["survive"].get("SurviveObjectTypeStr", "cdecl")
    SurviveObjectTypeStr.argtypes = [SurviveObjectType]
    SurviveObjectTypeStr.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 373
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

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 49
if _libs["survive"].has("PoserData_poser_pose_func", "cdecl"):
    PoserData_poser_pose_func = _libs["survive"].get("PoserData_poser_pose_func", "cdecl")
    PoserData_poser_pose_func.argtypes = [POINTER(PoserData), POINTER(SurviveObject), POINTER(SurvivePose), c_double, POINTER(struct_CnMat)]
    PoserData_poser_pose_func.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 63
if _libs["survive"].has("PoserData_poser_pose_func_with_velocity", "cdecl"):
    PoserData_poser_pose_func_with_velocity = _libs["survive"].get("PoserData_poser_pose_func_with_velocity", "cdecl")
    PoserData_poser_pose_func_with_velocity.argtypes = [POINTER(PoserData), POINTER(SurviveObject), POINTER(SurvivePose), POINTER(SurviveVelocity)]
    PoserData_poser_pose_func_with_velocity.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 78
if _libs["survive"].has("PoserData_lighthouse_pose_func", "cdecl"):
    PoserData_lighthouse_pose_func = _libs["survive"].get("PoserData_lighthouse_pose_func", "cdecl")
    PoserData_lighthouse_pose_func.argtypes = [POINTER(PoserData), POINTER(SurviveObject), c_uint8, POINTER(SurvivePose), c_double, POINTER(SurvivePose)]
    PoserData_lighthouse_pose_func.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 80
if _libs["survive"].has("PoserData_normalize_scene", "cdecl"):
    PoserData_normalize_scene = _libs["survive"].get("PoserData_normalize_scene", "cdecl")
    PoserData_normalize_scene.argtypes = [POINTER(SurviveContext), POINTER(SurvivePose), c_uint32, POINTER(SurvivePose)]
    PoserData_normalize_scene.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 82
if _libs["survive"].has("PoserData_lighthouse_poses_func", "cdecl"):
    PoserData_lighthouse_poses_func = _libs["survive"].get("PoserData_lighthouse_poses_func", "cdecl")
    PoserData_lighthouse_poses_func.argtypes = [POINTER(PoserData), POINTER(SurviveObject), POINTER(SurvivePose), POINTER(struct_CnMat), c_uint32, POINTER(SurvivePose)]
    PoserData_lighthouse_poses_func.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 85
if _libs["survive"].has("survive_get_reference_bsd", "cdecl"):
    survive_get_reference_bsd = _libs["survive"].get("survive_get_reference_bsd", "cdecl")
    survive_get_reference_bsd.argtypes = [POINTER(SurviveContext), POINTER(SurvivePose), c_uint32]
    survive_get_reference_bsd.restype = c_int8

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 88
if _libs["survive"].has("survive_lighthouse_adjust_confidence", "cdecl"):
    survive_lighthouse_adjust_confidence = _libs["survive"].get("survive_lighthouse_adjust_confidence", "cdecl")
    survive_lighthouse_adjust_confidence.argtypes = [POINTER(SurviveContext), c_uint8, c_double]
    survive_lighthouse_adjust_confidence.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 89
if _libs["survive"].has("survive_adjust_confidence", "cdecl"):
    survive_adjust_confidence = _libs["survive"].get("survive_adjust_confidence", "cdecl")
    survive_adjust_confidence.argtypes = [POINTER(SurviveObject), c_double]
    survive_adjust_confidence.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 97
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

PoserDataIMU = struct_PoserDataIMU# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 97

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 107
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

PoserDataLight = struct_PoserDataLight# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 107

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 114
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

PoserDataLightGen1 = struct_PoserDataLightGen1# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 114

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 121
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

PoserDataLightGen2 = struct_PoserDataLightGen2# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 121

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 123
if _libs["survive"].has("PoserDataLight_axis", "cdecl"):
    PoserDataLight_axis = _libs["survive"].get("PoserDataLight_axis", "cdecl")
    PoserDataLight_axis.argtypes = [POINTER(struct_PoserDataLight)]
    PoserDataLight_axis.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 129
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

PoserDataGlobalSceneMeasurement = struct_anon_48# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 129

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 131
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

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 145
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

PoserDataGlobalScenes = struct_PoserDataGlobalScenes# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 145

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 147
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

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 26
class struct_SurviveSensorActivations_s(Structure):
    pass

PoserCB = CFUNCTYPE(UNCHECKED(c_int), POINTER(SurviveObject), POINTER(POINTER(None)), POINTER(PoserData))# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 158

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 161
for _lib in _libs.values():
    if not _lib.has("survive_poser_invoke", "cdecl"):
        continue
    survive_poser_invoke = _lib.get("survive_poser_invoke", "cdecl")
    survive_poser_invoke.argtypes = [POINTER(SurviveObject), POINTER(PoserData), c_size_t]
    survive_poser_invoke.restype = None
    break

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 163
class struct_survive_threaded_poser(Structure):
    pass

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 164
for _lib in _libs.values():
    if not _lib.has("survive_create_threaded_poser", "cdecl"):
        continue
    survive_create_threaded_poser = _lib.get("survive_create_threaded_poser", "cdecl")
    survive_create_threaded_poser.argtypes = [POINTER(SurviveObject), PoserCB]
    survive_create_threaded_poser.restype = POINTER(struct_survive_threaded_poser)
    break

# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 165
for _lib in _libs.values():
    if not _lib.has("survive_threaded_poser_fn", "cdecl"):
        continue
    survive_threaded_poser_fn = _lib.get("survive_threaded_poser_fn", "cdecl")
    survive_threaded_poser_fn.argtypes = [POINTER(SurviveObject), POINTER(POINTER(None)), POINTER(PoserData)]
    survive_threaded_poser_fn.restype = c_int
    break

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 58
class struct_SurviveSensorActivations_params(Structure):
    pass

struct_SurviveSensorActivations_params.__slots__ = [
    'moveThresholdGyro',
    'moveThresholdAcc',
    'moveThresholdAng',
    'filterLightChange',
    'filterOutlierCriteria',
    'filterVarianceMin',
]
struct_SurviveSensorActivations_params._fields_ = [
    ('moveThresholdGyro', c_double),
    ('moveThresholdAcc', c_double),
    ('moveThresholdAng', c_double),
    ('filterLightChange', c_double),
    ('filterOutlierCriteria', c_double),
    ('filterVarianceMin', c_double),
]

struct_SurviveSensorActivations_s.__slots__ = [
    'so',
    'lh_gen',
    'angles',
    'angles_center_x',
    'angles_center_dev',
    'angles_center_cnt',
    'raw_angles',
    'raw_timecode',
    'timecode',
    'lengths',
    'hits',
    'imu_init_cnt',
    'last_imu',
    'last_light',
    'last_light_change',
    'last_movement',
    'runtime_offset',
    'accel',
    'last_accel',
    'gyro',
    'mag',
    'params',
]
struct_SurviveSensorActivations_s._fields_ = [
    ('so', POINTER(SurviveObject)),
    ('lh_gen', c_int),
    ('angles', ((c_double * int(2)) * int(16)) * int(32)),
    ('angles_center_x', (c_double * int(2)) * int(16)),
    ('angles_center_dev', (c_double * int(2)) * int(16)),
    ('angles_center_cnt', (c_int * int(2)) * int(16)),
    ('raw_angles', ((c_double * int(2)) * int(16)) * int(32)),
    ('raw_timecode', ((survive_long_timecode * int(2)) * int(16)) * int(32)),
    ('timecode', ((survive_long_timecode * int(2)) * int(16)) * int(32)),
    ('lengths', ((survive_timecode * int(2)) * int(2)) * int(32)),
    ('hits', ((survive_long_timecode * int(2)) * int(16)) * int(32)),
    ('imu_init_cnt', c_size_t),
    ('last_imu', survive_long_timecode),
    ('last_light', survive_long_timecode),
    ('last_light_change', survive_long_timecode),
    ('last_movement', survive_long_timecode),
    ('runtime_offset', c_double),
    ('accel', c_double * int(3)),
    ('last_accel', c_double * int(3)),
    ('gyro', c_double * int(3)),
    ('mag', c_double * int(3)),
    ('params', struct_SurviveSensorActivations_params),
]

SurviveSensorActivations = struct_SurviveSensorActivations_s# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 66

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 71
if _libs["survive"].has("SurviveSensorActivations_reset", "cdecl"):
    SurviveSensorActivations_reset = _libs["survive"].get("SurviveSensorActivations_reset", "cdecl")
    SurviveSensorActivations_reset.argtypes = [POINTER(SurviveSensorActivations)]
    SurviveSensorActivations_reset.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 72
if _libs["survive"].has("SurviveSensorActivations_ctor", "cdecl"):
    SurviveSensorActivations_ctor = _libs["survive"].get("SurviveSensorActivations_ctor", "cdecl")
    SurviveSensorActivations_ctor.argtypes = [POINTER(SurviveObject), POINTER(SurviveSensorActivations)]
    SurviveSensorActivations_ctor.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 73
if _libs["survive"].has("SurviveSensorActivations_dtor", "cdecl"):
    SurviveSensorActivations_dtor = _libs["survive"].get("SurviveSensorActivations_dtor", "cdecl")
    SurviveSensorActivations_dtor.argtypes = [POINTER(SurviveObject)]
    SurviveSensorActivations_dtor.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 74
if _libs["survive"].has("SurviveSensorActivations_long_timecode_imu", "cdecl"):
    SurviveSensorActivations_long_timecode_imu = _libs["survive"].get("SurviveSensorActivations_long_timecode_imu", "cdecl")
    SurviveSensorActivations_long_timecode_imu.argtypes = [POINTER(SurviveSensorActivations), survive_timecode]
    SurviveSensorActivations_long_timecode_imu.restype = survive_long_timecode

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 75
if _libs["survive"].has("SurviveSensorActivations_long_timecode_light", "cdecl"):
    SurviveSensorActivations_long_timecode_light = _libs["survive"].get("SurviveSensorActivations_long_timecode_light", "cdecl")
    SurviveSensorActivations_long_timecode_light.argtypes = [POINTER(SurviveSensorActivations), survive_timecode]
    SurviveSensorActivations_long_timecode_light.restype = survive_long_timecode

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 80
if _libs["survive"].has("SurviveSensorActivations_difference", "cdecl"):
    SurviveSensorActivations_difference = _libs["survive"].get("SurviveSensorActivations_difference", "cdecl")
    SurviveSensorActivations_difference.argtypes = [POINTER(SurviveSensorActivations), POINTER(SurviveSensorActivations)]
    SurviveSensorActivations_difference.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 82
if _libs["survive"].has("SurviveSensorActivations_add_sync", "cdecl"):
    SurviveSensorActivations_add_sync = _libs["survive"].get("SurviveSensorActivations_add_sync", "cdecl")
    SurviveSensorActivations_add_sync.argtypes = [POINTER(SurviveSensorActivations), POINTER(struct_PoserDataLight)]
    SurviveSensorActivations_add_sync.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 83
if _libs["survive"].has("SurviveSensorActivations_add", "cdecl"):
    SurviveSensorActivations_add = _libs["survive"].get("SurviveSensorActivations_add", "cdecl")
    SurviveSensorActivations_add.argtypes = [POINTER(SurviveSensorActivations), POINTER(struct_PoserDataLightGen1)]
    SurviveSensorActivations_add.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 84
if _libs["survive"].has("SurviveSensorActivations_add_gen2", "cdecl"):
    SurviveSensorActivations_add_gen2 = _libs["survive"].get("SurviveSensorActivations_add_gen2", "cdecl")
    SurviveSensorActivations_add_gen2.argtypes = [POINTER(SurviveSensorActivations), POINTER(struct_PoserDataLightGen2)]
    SurviveSensorActivations_add_gen2.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 86
if _libs["survive"].has("SurviveSensorActivations_valid_counts", "cdecl"):
    SurviveSensorActivations_valid_counts = _libs["survive"].get("SurviveSensorActivations_valid_counts", "cdecl")
    SurviveSensorActivations_valid_counts.argtypes = [POINTER(SurviveSensorActivations), survive_long_timecode, POINTER(c_uint32), POINTER(c_uint32), POINTER(c_uint32), POINTER(c_size_t)]
    SurviveSensorActivations_valid_counts.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 90
if _libs["survive"].has("SurviveSensorActivations_register_runtime", "cdecl"):
    SurviveSensorActivations_register_runtime = _libs["survive"].get("SurviveSensorActivations_register_runtime", "cdecl")
    SurviveSensorActivations_register_runtime.argtypes = [POINTER(SurviveSensorActivations), survive_long_timecode, c_uint64]
    SurviveSensorActivations_register_runtime.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 92
if _libs["survive"].has("SurviveSensorActivations_runtime", "cdecl"):
    SurviveSensorActivations_runtime = _libs["survive"].get("SurviveSensorActivations_runtime", "cdecl")
    SurviveSensorActivations_runtime.argtypes = [POINTER(SurviveSensorActivations), survive_long_timecode]
    SurviveSensorActivations_runtime.restype = c_uint64

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 93
if _libs["survive"].has("SurviveSensorActivations_add_imu", "cdecl"):
    SurviveSensorActivations_add_imu = _libs["survive"].get("SurviveSensorActivations_add_imu", "cdecl")
    SurviveSensorActivations_add_imu.argtypes = [POINTER(SurviveSensorActivations), POINTER(struct_PoserDataIMU)]
    SurviveSensorActivations_add_imu.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 99
if _libs["survive"].has("SurviveSensorActivations_is_reading_valid", "cdecl"):
    SurviveSensorActivations_is_reading_valid = _libs["survive"].get("SurviveSensorActivations_is_reading_valid", "cdecl")
    SurviveSensorActivations_is_reading_valid.argtypes = [POINTER(SurviveSensorActivations), survive_long_timecode, c_uint32, c_int, c_int]
    SurviveSensorActivations_is_reading_valid.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 103
if _libs["survive"].has("SurviveSensorActivations_time_since_last_reading", "cdecl"):
    SurviveSensorActivations_time_since_last_reading = _libs["survive"].get("SurviveSensorActivations_time_since_last_reading", "cdecl")
    SurviveSensorActivations_time_since_last_reading.argtypes = [POINTER(SurviveSensorActivations), c_uint32, c_int, c_int]
    SurviveSensorActivations_time_since_last_reading.restype = survive_long_timecode

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 106
if _libs["survive"].has("SurviveSensorActivations_last_reading", "cdecl"):
    SurviveSensorActivations_last_reading = _libs["survive"].get("SurviveSensorActivations_last_reading", "cdecl")
    SurviveSensorActivations_last_reading.argtypes = [POINTER(SurviveSensorActivations), c_uint32, c_int, c_int]
    SurviveSensorActivations_last_reading.restype = survive_long_timecode

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 113
if _libs["survive"].has("SurviveSensorActivations_isPairValid", "cdecl"):
    SurviveSensorActivations_isPairValid = _libs["survive"].get("SurviveSensorActivations_isPairValid", "cdecl")
    SurviveSensorActivations_isPairValid.argtypes = [POINTER(SurviveSensorActivations), survive_timecode, survive_timecode, c_uint32, c_int]
    SurviveSensorActivations_isPairValid.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 119
if _libs["survive"].has("SurviveSensorActivations_stationary_time", "cdecl"):
    SurviveSensorActivations_stationary_time = _libs["survive"].get("SurviveSensorActivations_stationary_time", "cdecl")
    SurviveSensorActivations_stationary_time.argtypes = [POINTER(SurviveSensorActivations)]
    SurviveSensorActivations_stationary_time.restype = survive_long_timecode

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 120
if _libs["survive"].has("SurviveSensorActivations_last_time", "cdecl"):
    SurviveSensorActivations_last_time = _libs["survive"].get("SurviveSensorActivations_last_time", "cdecl")
    SurviveSensorActivations_last_time.argtypes = [POINTER(SurviveSensorActivations)]
    SurviveSensorActivations_last_time.restype = survive_long_timecode

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 126
try:
    SurviveSensorActivations_default_tolerance = (survive_timecode).in_dll(_libs["survive"], "SurviveSensorActivations_default_tolerance")
except:
    pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 211
class struct_SurviveKalmanTracker(Structure):
    pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 213
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
    'raw_acc_scale',
    'raw_gyro_scale',
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
    'acceleration',
    'sensor_scale',
    'sensor_scale_var',
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
    ('charging', c_uint8, 1),
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
    ('raw_acc_scale', c_double),
    ('raw_gyro_scale', c_double),
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
    ('acceleration', LinmathPoint3d),
    ('sensor_scale', c_double),
    ('sensor_scale_var', c_double),
]

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 232
if _libs["survive"].has("survive_object_codename", "cdecl"):
    survive_object_codename = _libs["survive"].get("survive_object_codename", "cdecl")
    survive_object_codename.argtypes = [POINTER(SurviveObject)]
    survive_object_codename.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 233
if _libs["survive"].has("survive_object_last_imu2world", "cdecl"):
    survive_object_last_imu2world = _libs["survive"].get("survive_object_last_imu2world", "cdecl")
    survive_object_last_imu2world.argtypes = [POINTER(SurviveObject)]
    survive_object_last_imu2world.restype = POINTER(SurvivePose)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 234
if _libs["survive"].has("survive_object_drivername", "cdecl"):
    survive_object_drivername = _libs["survive"].get("survive_object_drivername", "cdecl")
    survive_object_drivername.argtypes = [POINTER(SurviveObject)]
    survive_object_drivername.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 235
if _libs["survive"].has("survive_object_charge", "cdecl"):
    survive_object_charge = _libs["survive"].get("survive_object_charge", "cdecl")
    survive_object_charge.argtypes = [POINTER(SurviveObject)]
    survive_object_charge.restype = c_int8

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 236
if _libs["survive"].has("survive_object_charging", "cdecl"):
    survive_object_charging = _libs["survive"].get("survive_object_charging", "cdecl")
    survive_object_charging.argtypes = [POINTER(SurviveObject)]
    survive_object_charging.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 238
if _libs["survive"].has("survive_object_pose", "cdecl"):
    survive_object_pose = _libs["survive"].get("survive_object_pose", "cdecl")
    survive_object_pose.argtypes = [POINTER(SurviveObject)]
    survive_object_pose.restype = POINTER(SurvivePose)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 240
if _libs["survive"].has("survive_object_sensor_ct", "cdecl"):
    survive_object_sensor_ct = _libs["survive"].get("survive_object_sensor_ct", "cdecl")
    survive_object_sensor_ct.argtypes = [POINTER(SurviveObject)]
    survive_object_sensor_ct.restype = c_int8

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 241
if _libs["survive"].has("survive_object_sensor_locations", "cdecl"):
    survive_object_sensor_locations = _libs["survive"].get("survive_object_sensor_locations", "cdecl")
    survive_object_sensor_locations.argtypes = [POINTER(SurviveObject)]
    survive_object_sensor_locations.restype = POINTER(c_double)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 242
if _libs["survive"].has("survive_object_sensor_normals", "cdecl"):
    survive_object_sensor_normals = _libs["survive"].get("survive_object_sensor_normals", "cdecl")
    survive_object_sensor_normals.argtypes = [POINTER(SurviveObject)]
    survive_object_sensor_normals.restype = POINTER(c_double)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 254
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

BaseStationCal = struct_BaseStationCal# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 254

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 277
class struct_SurviveKalmanLighthouse(Structure):
    pass

struct_BaseStationData.__slots__ = [
    'PositionSet',
    'Pose',
    'OOTXSet',
    'BaseStationID',
    'fcal',
    'sys_unlock_count',
    'accel',
    'mode',
    'confidence',
    'ootx_data',
    'user_ptr',
    'disable',
    'OOTXChecked',
    'tracker',
]
struct_BaseStationData._fields_ = [
    ('PositionSet', c_uint8, 1),
    ('Pose', SurvivePose),
    ('OOTXSet', c_uint8, 1),
    ('BaseStationID', c_uint32),
    ('fcal', BaseStationCal * int(2)),
    ('sys_unlock_count', c_uint8),
    ('accel', LinmathPoint3d),
    ('mode', c_uint8),
    ('confidence', c_double),
    ('ootx_data', POINTER(None)),
    ('user_ptr', POINTER(None)),
    ('disable', c_bool),
    ('OOTXChecked', c_uint8, 1),
    ('tracker', POINTER(struct_SurviveKalmanLighthouse)),
]

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 280
class struct_config_group(Structure):
    pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 297
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

ButtonQueueEntry = struct_anon_50# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 297

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 306
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

ButtonQueue = struct_anon_51# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 306

enum_anon_52 = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 308

SURVIVE_STOPPED = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 308

SURVIVE_RUNNING = (SURVIVE_STOPPED + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 308

SURVIVE_CLOSING = (SURVIVE_RUNNING + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 308

SURVIVE_STATE_MAX = (SURVIVE_CLOSING + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 308

SurviveState = enum_anon_52# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 308

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 310
class struct_SurviveRecordingData(Structure):
    pass

enum_SurviveCalFlag = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 312

SVCal_None = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 312

SVCal_Phase = 1# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 312

SVCal_Tilt = 2# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 312

SVCal_Curve = 4# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 312

SVCal_Gib = 8# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 312

SVCal_All = (((SVCal_Gib | SVCal_Curve) | SVCal_Tilt) | SVCal_Phase)# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 312

struct_SurviveContext.__slots__ = [
    'lh_version_configed',
    'lh_version_forced',
    'lh_version',
    'new_objectproc',
    'disconnectproc',
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
    'new_object_call_time',
    'new_object_call_cnt',
    'new_object_call_over_cnt',
    'new_object_max_call_time',
    'disconnect_call_time',
    'disconnect_call_cnt',
    'disconnect_call_over_cnt',
    'disconnect_max_call_time',
    'printf_call_time',
    'printf_call_cnt',
    'printf_call_over_cnt',
    'printf_max_call_time',
    'log_call_time',
    'log_call_cnt',
    'log_call_over_cnt',
    'log_max_call_time',
    'report_error_call_time',
    'report_error_call_cnt',
    'report_error_call_over_cnt',
    'report_error_max_call_time',
    'config_call_time',
    'config_call_cnt',
    'config_call_over_cnt',
    'config_max_call_time',
    'gen_detected_call_time',
    'gen_detected_call_cnt',
    'gen_detected_call_over_cnt',
    'gen_detected_max_call_time',
    'ootx_received_call_time',
    'ootx_received_call_cnt',
    'ootx_received_call_over_cnt',
    'ootx_received_max_call_time',
    'lightcap_call_time',
    'lightcap_call_cnt',
    'lightcap_call_over_cnt',
    'lightcap_max_call_time',
    'light_call_time',
    'light_call_cnt',
    'light_call_over_cnt',
    'light_max_call_time',
    'light_pulse_call_time',
    'light_pulse_call_cnt',
    'light_pulse_call_over_cnt',
    'light_pulse_max_call_time',
    'angle_call_time',
    'angle_call_cnt',
    'angle_call_over_cnt',
    'angle_max_call_time',
    'sync_call_time',
    'sync_call_cnt',
    'sync_call_over_cnt',
    'sync_max_call_time',
    'sweep_call_time',
    'sweep_call_cnt',
    'sweep_call_over_cnt',
    'sweep_max_call_time',
    'sweep_angle_call_time',
    'sweep_angle_call_cnt',
    'sweep_angle_call_over_cnt',
    'sweep_angle_max_call_time',
    'raw_imu_call_time',
    'raw_imu_call_cnt',
    'raw_imu_call_over_cnt',
    'raw_imu_max_call_time',
    'imu_call_time',
    'imu_call_cnt',
    'imu_call_over_cnt',
    'imu_max_call_time',
    'button_call_time',
    'button_call_cnt',
    'button_call_over_cnt',
    'button_max_call_time',
    'imupose_call_time',
    'imupose_call_cnt',
    'imupose_call_over_cnt',
    'imupose_max_call_time',
    'pose_call_time',
    'pose_call_cnt',
    'pose_call_over_cnt',
    'pose_max_call_time',
    'velocity_call_time',
    'velocity_call_cnt',
    'velocity_call_over_cnt',
    'velocity_max_call_time',
    'external_pose_call_time',
    'external_pose_call_cnt',
    'external_pose_call_over_cnt',
    'external_pose_max_call_time',
    'external_velocity_call_time',
    'external_velocity_call_cnt',
    'external_velocity_call_over_cnt',
    'external_velocity_max_call_time',
    'lighthouse_pose_call_time',
    'lighthouse_pose_call_cnt',
    'lighthouse_pose_call_over_cnt',
    'lighthouse_pose_max_call_time',
    'datalog_call_time',
    'datalog_call_cnt',
    'datalog_call_over_cnt',
    'datalog_max_call_time',
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
    'request_floor_set',
]
struct_SurviveContext._fields_ = [
    ('lh_version_configed', c_int),
    ('lh_version_forced', c_int),
    ('lh_version', c_int),
    ('new_objectproc', new_object_process_func),
    ('disconnectproc', disconnect_process_func),
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
    ('new_object_call_time', c_double),
    ('new_object_call_cnt', c_uint32),
    ('new_object_call_over_cnt', c_uint32),
    ('new_object_max_call_time', c_double),
    ('disconnect_call_time', c_double),
    ('disconnect_call_cnt', c_uint32),
    ('disconnect_call_over_cnt', c_uint32),
    ('disconnect_max_call_time', c_double),
    ('printf_call_time', c_double),
    ('printf_call_cnt', c_uint32),
    ('printf_call_over_cnt', c_uint32),
    ('printf_max_call_time', c_double),
    ('log_call_time', c_double),
    ('log_call_cnt', c_uint32),
    ('log_call_over_cnt', c_uint32),
    ('log_max_call_time', c_double),
    ('report_error_call_time', c_double),
    ('report_error_call_cnt', c_uint32),
    ('report_error_call_over_cnt', c_uint32),
    ('report_error_max_call_time', c_double),
    ('config_call_time', c_double),
    ('config_call_cnt', c_uint32),
    ('config_call_over_cnt', c_uint32),
    ('config_max_call_time', c_double),
    ('gen_detected_call_time', c_double),
    ('gen_detected_call_cnt', c_uint32),
    ('gen_detected_call_over_cnt', c_uint32),
    ('gen_detected_max_call_time', c_double),
    ('ootx_received_call_time', c_double),
    ('ootx_received_call_cnt', c_uint32),
    ('ootx_received_call_over_cnt', c_uint32),
    ('ootx_received_max_call_time', c_double),
    ('lightcap_call_time', c_double),
    ('lightcap_call_cnt', c_uint32),
    ('lightcap_call_over_cnt', c_uint32),
    ('lightcap_max_call_time', c_double),
    ('light_call_time', c_double),
    ('light_call_cnt', c_uint32),
    ('light_call_over_cnt', c_uint32),
    ('light_max_call_time', c_double),
    ('light_pulse_call_time', c_double),
    ('light_pulse_call_cnt', c_uint32),
    ('light_pulse_call_over_cnt', c_uint32),
    ('light_pulse_max_call_time', c_double),
    ('angle_call_time', c_double),
    ('angle_call_cnt', c_uint32),
    ('angle_call_over_cnt', c_uint32),
    ('angle_max_call_time', c_double),
    ('sync_call_time', c_double),
    ('sync_call_cnt', c_uint32),
    ('sync_call_over_cnt', c_uint32),
    ('sync_max_call_time', c_double),
    ('sweep_call_time', c_double),
    ('sweep_call_cnt', c_uint32),
    ('sweep_call_over_cnt', c_uint32),
    ('sweep_max_call_time', c_double),
    ('sweep_angle_call_time', c_double),
    ('sweep_angle_call_cnt', c_uint32),
    ('sweep_angle_call_over_cnt', c_uint32),
    ('sweep_angle_max_call_time', c_double),
    ('raw_imu_call_time', c_double),
    ('raw_imu_call_cnt', c_uint32),
    ('raw_imu_call_over_cnt', c_uint32),
    ('raw_imu_max_call_time', c_double),
    ('imu_call_time', c_double),
    ('imu_call_cnt', c_uint32),
    ('imu_call_over_cnt', c_uint32),
    ('imu_max_call_time', c_double),
    ('button_call_time', c_double),
    ('button_call_cnt', c_uint32),
    ('button_call_over_cnt', c_uint32),
    ('button_max_call_time', c_double),
    ('imupose_call_time', c_double),
    ('imupose_call_cnt', c_uint32),
    ('imupose_call_over_cnt', c_uint32),
    ('imupose_max_call_time', c_double),
    ('pose_call_time', c_double),
    ('pose_call_cnt', c_uint32),
    ('pose_call_over_cnt', c_uint32),
    ('pose_max_call_time', c_double),
    ('velocity_call_time', c_double),
    ('velocity_call_cnt', c_uint32),
    ('velocity_call_over_cnt', c_uint32),
    ('velocity_max_call_time', c_double),
    ('external_pose_call_time', c_double),
    ('external_pose_call_cnt', c_uint32),
    ('external_pose_call_over_cnt', c_uint32),
    ('external_pose_max_call_time', c_double),
    ('external_velocity_call_time', c_double),
    ('external_velocity_call_cnt', c_uint32),
    ('external_velocity_call_over_cnt', c_uint32),
    ('external_velocity_max_call_time', c_double),
    ('lighthouse_pose_call_time', c_double),
    ('lighthouse_pose_call_cnt', c_uint32),
    ('lighthouse_pose_call_over_cnt', c_uint32),
    ('lighthouse_pose_max_call_time', c_double),
    ('datalog_call_time', c_double),
    ('datalog_call_cnt', c_uint32),
    ('datalog_call_over_cnt', c_uint32),
    ('datalog_max_call_time', c_double),
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
    ('request_floor_set', c_bool),
]

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 375
if _libs["survive"].has("survive_verify_FLT_size", "cdecl"):
    survive_verify_FLT_size = _libs["survive"].get("survive_verify_FLT_size", "cdecl")
    survive_verify_FLT_size.argtypes = [c_uint32]
    survive_verify_FLT_size.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 378
if _libs["survive"].has("survive_init_plugins", "cdecl"):
    survive_init_plugins = _libs["survive"].get("survive_init_plugins", "cdecl")
    survive_init_plugins.argtypes = []
    survive_init_plugins.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 379
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
if _libs["survive"].has("survive_install_disconnect_fn", "cdecl"):
    survive_install_disconnect_fn = _libs["survive"].get("survive_install_disconnect_fn", "cdecl")
    survive_install_disconnect_fn.argtypes = [POINTER(SurviveContext), disconnect_process_func]
    survive_install_disconnect_fn.restype = disconnect_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 12
if _libs["survive"].has("survive_install_printf_fn", "cdecl"):
    survive_install_printf_fn = _libs["survive"].get("survive_install_printf_fn", "cdecl")
    survive_install_printf_fn.argtypes = [POINTER(SurviveContext), printf_process_func]
    survive_install_printf_fn.restype = printf_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 13
if _libs["survive"].has("survive_install_log_fn", "cdecl"):
    survive_install_log_fn = _libs["survive"].get("survive_install_log_fn", "cdecl")
    survive_install_log_fn.argtypes = [POINTER(SurviveContext), log_process_func]
    survive_install_log_fn.restype = log_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 14
if _libs["survive"].has("survive_install_report_error_fn", "cdecl"):
    survive_install_report_error_fn = _libs["survive"].get("survive_install_report_error_fn", "cdecl")
    survive_install_report_error_fn.argtypes = [POINTER(SurviveContext), report_error_process_func]
    survive_install_report_error_fn.restype = report_error_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 16
if _libs["survive"].has("survive_install_config_fn", "cdecl"):
    survive_install_config_fn = _libs["survive"].get("survive_install_config_fn", "cdecl")
    survive_install_config_fn.argtypes = [POINTER(SurviveContext), config_process_func]
    survive_install_config_fn.restype = config_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 17
if _libs["survive"].has("survive_install_gen_detected_fn", "cdecl"):
    survive_install_gen_detected_fn = _libs["survive"].get("survive_install_gen_detected_fn", "cdecl")
    survive_install_gen_detected_fn.argtypes = [POINTER(SurviveContext), gen_detected_process_func]
    survive_install_gen_detected_fn.restype = gen_detected_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 18
if _libs["survive"].has("survive_install_ootx_received_fn", "cdecl"):
    survive_install_ootx_received_fn = _libs["survive"].get("survive_install_ootx_received_fn", "cdecl")
    survive_install_ootx_received_fn.argtypes = [POINTER(SurviveContext), ootx_received_process_func]
    survive_install_ootx_received_fn.restype = ootx_received_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 21
if _libs["survive"].has("survive_install_lightcap_fn", "cdecl"):
    survive_install_lightcap_fn = _libs["survive"].get("survive_install_lightcap_fn", "cdecl")
    survive_install_lightcap_fn.argtypes = [POINTER(SurviveContext), lightcap_process_func]
    survive_install_lightcap_fn.restype = lightcap_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 22
if _libs["survive"].has("survive_install_light_fn", "cdecl"):
    survive_install_light_fn = _libs["survive"].get("survive_install_light_fn", "cdecl")
    survive_install_light_fn.argtypes = [POINTER(SurviveContext), light_process_func]
    survive_install_light_fn.restype = light_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 23
if _libs["survive"].has("survive_install_light_pulse_fn", "cdecl"):
    survive_install_light_pulse_fn = _libs["survive"].get("survive_install_light_pulse_fn", "cdecl")
    survive_install_light_pulse_fn.argtypes = [POINTER(SurviveContext), light_pulse_process_func]
    survive_install_light_pulse_fn.restype = light_pulse_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 24
if _libs["survive"].has("survive_install_angle_fn", "cdecl"):
    survive_install_angle_fn = _libs["survive"].get("survive_install_angle_fn", "cdecl")
    survive_install_angle_fn.argtypes = [POINTER(SurviveContext), angle_process_func]
    survive_install_angle_fn.restype = angle_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 27
if _libs["survive"].has("survive_install_sync_fn", "cdecl"):
    survive_install_sync_fn = _libs["survive"].get("survive_install_sync_fn", "cdecl")
    survive_install_sync_fn.argtypes = [POINTER(SurviveContext), sync_process_func]
    survive_install_sync_fn.restype = sync_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 28
if _libs["survive"].has("survive_install_sweep_fn", "cdecl"):
    survive_install_sweep_fn = _libs["survive"].get("survive_install_sweep_fn", "cdecl")
    survive_install_sweep_fn.argtypes = [POINTER(SurviveContext), sweep_process_func]
    survive_install_sweep_fn.restype = sweep_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 29
if _libs["survive"].has("survive_install_sweep_angle_fn", "cdecl"):
    survive_install_sweep_angle_fn = _libs["survive"].get("survive_install_sweep_angle_fn", "cdecl")
    survive_install_sweep_angle_fn.argtypes = [POINTER(SurviveContext), sweep_angle_process_func]
    survive_install_sweep_angle_fn.restype = sweep_angle_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 31
if _libs["survive"].has("survive_install_raw_imu_fn", "cdecl"):
    survive_install_raw_imu_fn = _libs["survive"].get("survive_install_raw_imu_fn", "cdecl")
    survive_install_raw_imu_fn.argtypes = [POINTER(SurviveContext), raw_imu_process_func]
    survive_install_raw_imu_fn.restype = raw_imu_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 32
if _libs["survive"].has("survive_install_imu_fn", "cdecl"):
    survive_install_imu_fn = _libs["survive"].get("survive_install_imu_fn", "cdecl")
    survive_install_imu_fn.argtypes = [POINTER(SurviveContext), imu_process_func]
    survive_install_imu_fn.restype = imu_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 33
if _libs["survive"].has("survive_install_button_fn", "cdecl"):
    survive_install_button_fn = _libs["survive"].get("survive_install_button_fn", "cdecl")
    survive_install_button_fn.argtypes = [POINTER(SurviveContext), button_process_func]
    survive_install_button_fn.restype = button_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 35
if _libs["survive"].has("survive_install_imupose_fn", "cdecl"):
    survive_install_imupose_fn = _libs["survive"].get("survive_install_imupose_fn", "cdecl")
    survive_install_imupose_fn.argtypes = [POINTER(SurviveContext), imupose_process_func]
    survive_install_imupose_fn.restype = imupose_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 36
if _libs["survive"].has("survive_install_pose_fn", "cdecl"):
    survive_install_pose_fn = _libs["survive"].get("survive_install_pose_fn", "cdecl")
    survive_install_pose_fn.argtypes = [POINTER(SurviveContext), pose_process_func]
    survive_install_pose_fn.restype = pose_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 37
if _libs["survive"].has("survive_install_velocity_fn", "cdecl"):
    survive_install_velocity_fn = _libs["survive"].get("survive_install_velocity_fn", "cdecl")
    survive_install_velocity_fn.argtypes = [POINTER(SurviveContext), velocity_process_func]
    survive_install_velocity_fn.restype = velocity_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 39
if _libs["survive"].has("survive_install_external_pose_fn", "cdecl"):
    survive_install_external_pose_fn = _libs["survive"].get("survive_install_external_pose_fn", "cdecl")
    survive_install_external_pose_fn.argtypes = [POINTER(SurviveContext), external_pose_process_func]
    survive_install_external_pose_fn.restype = external_pose_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 40
if _libs["survive"].has("survive_install_external_velocity_fn", "cdecl"):
    survive_install_external_velocity_fn = _libs["survive"].get("survive_install_external_velocity_fn", "cdecl")
    survive_install_external_velocity_fn.argtypes = [POINTER(SurviveContext), external_velocity_process_func]
    survive_install_external_velocity_fn.restype = external_velocity_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 41
if _libs["survive"].has("survive_install_lighthouse_pose_fn", "cdecl"):
    survive_install_lighthouse_pose_fn = _libs["survive"].get("survive_install_lighthouse_pose_fn", "cdecl")
    survive_install_lighthouse_pose_fn.argtypes = [POINTER(SurviveContext), lighthouse_pose_process_func]
    survive_install_lighthouse_pose_fn.restype = lighthouse_pose_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_hooks.h: 43
if _libs["survive"].has("survive_install_datalog_fn", "cdecl"):
    survive_install_datalog_fn = _libs["survive"].get("survive_install_datalog_fn", "cdecl")
    survive_install_datalog_fn.argtypes = [POINTER(SurviveContext), datalog_process_func]
    survive_install_datalog_fn.restype = datalog_process_func

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 415
if _libs["survive"].has("survive_startup", "cdecl"):
    survive_startup = _libs["survive"].get("survive_startup", "cdecl")
    survive_startup.argtypes = [POINTER(SurviveContext)]
    survive_startup.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 416
if _libs["survive"].has("survive_poll", "cdecl"):
    survive_poll = _libs["survive"].get("survive_poll", "cdecl")
    survive_poll.argtypes = [POINTER(SurviveContext)]
    survive_poll.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 417
if _libs["survive"].has("survive_close", "cdecl"):
    survive_close = _libs["survive"].get("survive_close", "cdecl")
    survive_close.argtypes = [POINTER(SurviveContext)]
    survive_close.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 418
if _libs["survive"].has("survive_get_ctx_lock", "cdecl"):
    survive_get_ctx_lock = _libs["survive"].get("survive_get_ctx_lock", "cdecl")
    survive_get_ctx_lock.argtypes = [POINTER(SurviveContext)]
    survive_get_ctx_lock.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 419
if _libs["survive"].has("survive_release_ctx_lock", "cdecl"):
    survive_release_ctx_lock = _libs["survive"].get("survive_release_ctx_lock", "cdecl")
    survive_release_ctx_lock.argtypes = [POINTER(SurviveContext)]
    survive_release_ctx_lock.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 421
if _libs["survive"].has("survive_build_tag", "cdecl"):
    survive_build_tag = _libs["survive"].get("survive_build_tag", "cdecl")
    survive_build_tag.argtypes = []
    survive_build_tag.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 423
if _libs["survive"].has("survive_get_so_by_name", "cdecl"):
    survive_get_so_by_name = _libs["survive"].get("survive_get_so_by_name", "cdecl")
    survive_get_so_by_name.argtypes = [POINTER(SurviveContext), String]
    survive_get_so_by_name.restype = POINTER(SurviveObject)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 426
if _libs["survive"].has("survive_simple_inflate", "cdecl"):
    survive_simple_inflate = _libs["survive"].get("survive_simple_inflate", "cdecl")
    survive_simple_inflate.argtypes = [POINTER(SurviveContext), POINTER(c_uint8), c_int, POINTER(c_uint8), c_int]
    survive_simple_inflate.restype = c_int

enum_survive_config_flags = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 429

SC_GET = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 429

SC_SET = 1# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 429

SC_OVERRIDE = 2# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 429

SC_SETCONFIG = 4# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 429

survive_config_iterate_fn = CFUNCTYPE(UNCHECKED(None), POINTER(SurviveContext), String, c_uint8, String, String, POINTER(None))# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 436

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 438
if _libs["survive"].has("survive_config_iterate", "cdecl"):
    survive_config_iterate = _libs["survive"].get("survive_config_iterate", "cdecl")
    survive_config_iterate.argtypes = [POINTER(SurviveContext), survive_config_iterate_fn, POINTER(None)]
    survive_config_iterate.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 439
if _libs["survive"].has("survive_config_is_set", "cdecl"):
    survive_config_is_set = _libs["survive"].get("survive_config_is_set", "cdecl")
    survive_config_is_set.argtypes = [POINTER(SurviveContext), String]
    survive_config_is_set.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 440
if _libs["survive"].has("survive_configf", "cdecl"):
    survive_configf = _libs["survive"].get("survive_configf", "cdecl")
    survive_configf.argtypes = [POINTER(SurviveContext), String, c_char, c_double]
    survive_configf.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 441
if _libs["survive"].has("survive_configb", "cdecl"):
    survive_configb = _libs["survive"].get("survive_configb", "cdecl")
    survive_configb.argtypes = [POINTER(SurviveContext), String, c_char, c_bool]
    survive_configb.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 442
if _libs["survive"].has("survive_configi", "cdecl"):
    survive_configi = _libs["survive"].get("survive_configi", "cdecl")
    survive_configi.argtypes = [POINTER(SurviveContext), String, c_char, c_uint32]
    survive_configi.restype = c_uint32

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 443
if _libs["survive"].has("survive_config_type", "cdecl"):
    survive_config_type = _libs["survive"].get("survive_config_type", "cdecl")
    survive_config_type.argtypes = [POINTER(SurviveContext), String]
    survive_config_type.restype = c_char

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 444
if _libs["survive"].has("survive_config_as_str", "cdecl"):
    survive_config_as_str = _libs["survive"].get("survive_config_as_str", "cdecl")
    survive_config_as_str.argtypes = [POINTER(SurviveContext), String, c_size_t, String, String]
    survive_config_as_str.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 447
if _libs["survive"].has("survive_configs", "cdecl"):
    survive_configs = _libs["survive"].get("survive_configs", "cdecl")
    survive_configs.argtypes = [POINTER(SurviveContext), String, c_char, String]
    survive_configs.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 449
if _libs["survive"].has("survive_attach_config", "cdecl"):
    survive_attach_config = _libs["survive"].get("survive_attach_config", "cdecl")
    survive_attach_config.argtypes = [POINTER(SurviveContext), String, POINTER(None), c_char]
    survive_attach_config.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 450
if _libs["survive"].has("survive_attach_configi", "cdecl"):
    survive_attach_configi = _libs["survive"].get("survive_attach_configi", "cdecl")
    survive_attach_configi.argtypes = [POINTER(SurviveContext), String, POINTER(c_int32)]
    survive_attach_configi.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 451
if _libs["survive"].has("survive_attach_configf", "cdecl"):
    survive_attach_configf = _libs["survive"].get("survive_attach_configf", "cdecl")
    survive_attach_configf.argtypes = [POINTER(SurviveContext), String, POINTER(c_double)]
    survive_attach_configf.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 452
if _libs["survive"].has("survive_attach_configs", "cdecl"):
    survive_attach_configs = _libs["survive"].get("survive_attach_configs", "cdecl")
    survive_attach_configs.argtypes = [POINTER(SurviveContext), String, String]
    survive_attach_configs.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 453
if _libs["survive"].has("survive_attach_configb", "cdecl"):
    survive_attach_configb = _libs["survive"].get("survive_attach_configb", "cdecl")
    survive_attach_configb.argtypes = [POINTER(SurviveContext), String, POINTER(c_bool)]
    survive_attach_configb.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 476
if _libs["survive"].has("survive_detach_config", "cdecl"):
    survive_detach_config = _libs["survive"].get("survive_detach_config", "cdecl")
    survive_detach_config.argtypes = [POINTER(SurviveContext), String, POINTER(None)]
    survive_detach_config.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 478
if _libs["survive"].has("survive_get_bsd_idx", "cdecl"):
    survive_get_bsd_idx = _libs["survive"].get("survive_get_bsd_idx", "cdecl")
    survive_get_bsd_idx.argtypes = [POINTER(SurviveContext), survive_channel]
    survive_get_bsd_idx.restype = c_int8

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 542
if _libs["survive"].has("survive_config_bind_variable", "cdecl"):
    _func = _libs["survive"].get("survive_config_bind_variable", "cdecl")
    _restype = None
    _errcheck = None
    _argtypes = [c_char, String, String]
    survive_config_bind_variable = _variadic_function(_func,_restype,_argtypes,_errcheck)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 543
if _libs["survive"].has("survive_config_bind_variablei", "cdecl"):
    survive_config_bind_variablei = _libs["survive"].get("survive_config_bind_variablei", "cdecl")
    survive_config_bind_variablei.argtypes = [String, String, c_int]
    survive_config_bind_variablei.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 544
if _libs["survive"].has("survive_config_bind_variableb", "cdecl"):
    survive_config_bind_variableb = _libs["survive"].get("survive_config_bind_variableb", "cdecl")
    survive_config_bind_variableb.argtypes = [String, String, c_bool]
    survive_config_bind_variableb.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 545
if _libs["survive"].has("survive_config_bind_variablef", "cdecl"):
    survive_config_bind_variablef = _libs["survive"].get("survive_config_bind_variablef", "cdecl")
    survive_config_bind_variablef.argtypes = [String, String, c_double]
    survive_config_bind_variablef.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 548
for _lib in _libs.values():
    if not _lib.has("survive_cal_get_status", "cdecl"):
        continue
    survive_cal_get_status = _lib.get("survive_cal_get_status", "cdecl")
    survive_cal_get_status.argtypes = [POINTER(SurviveContext), String, c_int]
    survive_cal_get_status.restype = c_int
    break

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 551
if _libs["survive"].has("survive_haptic", "cdecl"):
    survive_haptic = _libs["survive"].get("survive_haptic", "cdecl")
    survive_haptic.argtypes = [POINTER(SurviveObject), c_double, c_double, c_double]
    survive_haptic.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 552
if _libs["survive"].has("survive_ootx_free_decoder_context", "cdecl"):
    survive_ootx_free_decoder_context = _libs["survive"].get("survive_ootx_free_decoder_context", "cdecl")
    survive_ootx_free_decoder_context.argtypes = [POINTER(struct_SurviveContext), c_int]
    survive_ootx_free_decoder_context.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 553
if _libs["survive"].has("survive_find_ang_velocity", "cdecl"):
    survive_find_ang_velocity = _libs["survive"].get("survive_find_ang_velocity", "cdecl")
    survive_find_ang_velocity.argtypes = [SurviveAngularVelocity, c_double, LinmathQuat, LinmathQuat]
    survive_find_ang_velocity.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 555
if _libs["survive"].has("survive_apply_ang_velocity", "cdecl"):
    survive_apply_ang_velocity = _libs["survive"].get("survive_apply_ang_velocity", "cdecl")
    survive_apply_ang_velocity.argtypes = [LinmathQuat, SurviveAngularVelocity, c_double, LinmathQuat]
    survive_apply_ang_velocity.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 557
if _libs["survive"].has("survive_apply_ang_velocity_aa", "cdecl"):
    survive_apply_ang_velocity_aa = _libs["survive"].get("survive_apply_ang_velocity_aa", "cdecl")
    survive_apply_ang_velocity_aa.argtypes = [LinmathAxisAngle, SurviveAngularVelocity, c_double, LinmathAxisAngle]
    survive_apply_ang_velocity_aa.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 561
if _libs["survive"].has("survive_default_ootx_received_process", "cdecl"):
    survive_default_ootx_received_process = _libs["survive"].get("survive_default_ootx_received_process", "cdecl")
    survive_default_ootx_received_process.argtypes = [POINTER(struct_SurviveContext), c_uint8]
    survive_default_ootx_received_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 563
if _libs["survive"].has("survive_default_disconnect_process", "cdecl"):
    survive_default_disconnect_process = _libs["survive"].get("survive_default_disconnect_process", "cdecl")
    survive_default_disconnect_process.argtypes = [POINTER(struct_SurviveObject)]
    survive_default_disconnect_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 564
if _libs["survive"].has("survive_default_printf_process", "cdecl"):
    _func = _libs["survive"].get("survive_default_printf_process", "cdecl")
    _restype = c_int
    _errcheck = None
    _argtypes = [POINTER(struct_SurviveContext), String]
    survive_default_printf_process = _variadic_function(_func,_restype,_argtypes,_errcheck)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 565
if _libs["survive"].has("survive_default_log_process", "cdecl"):
    survive_default_log_process = _libs["survive"].get("survive_default_log_process", "cdecl")
    survive_default_log_process.argtypes = [POINTER(struct_SurviveContext), SurviveLogLevel, String]
    survive_default_log_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 566
if _libs["survive"].has("survive_default_lightcap_process", "cdecl"):
    survive_default_lightcap_process = _libs["survive"].get("survive_default_lightcap_process", "cdecl")
    survive_default_lightcap_process.argtypes = [POINTER(SurviveObject), POINTER(LightcapElement)]
    survive_default_lightcap_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 567
if _libs["survive"].has("survive_default_light_process", "cdecl"):
    survive_default_light_process = _libs["survive"].get("survive_default_light_process", "cdecl")
    survive_default_light_process.argtypes = [POINTER(SurviveObject), c_int, c_int, c_int, survive_timecode, survive_timecode, c_uint32]
    survive_default_light_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 569
if _libs["survive"].has("survive_default_raw_imu_process", "cdecl"):
    survive_default_raw_imu_process = _libs["survive"].get("survive_default_raw_imu_process", "cdecl")
    survive_default_raw_imu_process.argtypes = [POINTER(SurviveObject), c_int, POINTER(c_double), survive_timecode, c_int]
    survive_default_raw_imu_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 571
if _libs["survive"].has("survive_default_set_imu_scale_modes", "cdecl"):
    survive_default_set_imu_scale_modes = _libs["survive"].get("survive_default_set_imu_scale_modes", "cdecl")
    survive_default_set_imu_scale_modes.argtypes = [POINTER(SurviveObject), c_int, c_int]
    survive_default_set_imu_scale_modes.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 572
if _libs["survive"].has("survive_default_imu_process", "cdecl"):
    survive_default_imu_process = _libs["survive"].get("survive_default_imu_process", "cdecl")
    survive_default_imu_process.argtypes = [POINTER(SurviveObject), c_int, POINTER(c_double), survive_timecode, c_int]
    survive_default_imu_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 574
if _libs["survive"].has("survive_default_angle_process", "cdecl"):
    survive_default_angle_process = _libs["survive"].get("survive_default_angle_process", "cdecl")
    survive_default_angle_process.argtypes = [POINTER(SurviveObject), c_int, c_int, survive_timecode, c_double, c_double, c_uint32]
    survive_default_angle_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 577
if _libs["survive"].has("survive_default_light_pulse_process", "cdecl"):
    survive_default_light_pulse_process = _libs["survive"].get("survive_default_light_pulse_process", "cdecl")
    survive_default_light_pulse_process.argtypes = [POINTER(SurviveObject), c_int, c_int, survive_timecode, c_double, c_uint32]
    survive_default_light_pulse_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 579
if _libs["survive"].has("survive_default_sync_process", "cdecl"):
    survive_default_sync_process = _libs["survive"].get("survive_default_sync_process", "cdecl")
    survive_default_sync_process.argtypes = [POINTER(SurviveObject), survive_channel, survive_timecode, c_bool, c_bool]
    survive_default_sync_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 581
if _libs["survive"].has("survive_default_sweep_process", "cdecl"):
    survive_default_sweep_process = _libs["survive"].get("survive_default_sweep_process", "cdecl")
    survive_default_sweep_process.argtypes = [POINTER(SurviveObject), survive_channel, c_int, survive_timecode, c_bool]
    survive_default_sweep_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 583
if _libs["survive"].has("survive_default_sweep_angle_process", "cdecl"):
    survive_default_sweep_angle_process = _libs["survive"].get("survive_default_sweep_angle_process", "cdecl")
    survive_default_sweep_angle_process.argtypes = [POINTER(SurviveObject), survive_channel, c_int, survive_timecode, c_int8, c_double]
    survive_default_sweep_angle_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 585
if _libs["survive"].has("survive_default_button_process", "cdecl"):
    survive_default_button_process = _libs["survive"].get("survive_default_button_process", "cdecl")
    survive_default_button_process.argtypes = [POINTER(SurviveObject), enum_SurviveInputEvent, enum_SurviveButton, POINTER(enum_SurviveAxis), POINTER(SurviveAxisVal_t)]
    survive_default_button_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 588
if _libs["survive"].has("survive_default_imupose_process", "cdecl"):
    survive_default_imupose_process = _libs["survive"].get("survive_default_imupose_process", "cdecl")
    survive_default_imupose_process.argtypes = [POINTER(SurviveObject), survive_long_timecode, POINTER(SurvivePose)]
    survive_default_imupose_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 590
if _libs["survive"].has("survive_default_pose_process", "cdecl"):
    survive_default_pose_process = _libs["survive"].get("survive_default_pose_process", "cdecl")
    survive_default_pose_process.argtypes = [POINTER(SurviveObject), survive_long_timecode, POINTER(SurvivePose)]
    survive_default_pose_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 592
if _libs["survive"].has("survive_default_velocity_process", "cdecl"):
    survive_default_velocity_process = _libs["survive"].get("survive_default_velocity_process", "cdecl")
    survive_default_velocity_process.argtypes = [POINTER(SurviveObject), survive_long_timecode, POINTER(SurviveVelocity)]
    survive_default_velocity_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 594
if _libs["survive"].has("survive_default_external_pose_process", "cdecl"):
    survive_default_external_pose_process = _libs["survive"].get("survive_default_external_pose_process", "cdecl")
    survive_default_external_pose_process.argtypes = [POINTER(SurviveContext), String, POINTER(SurvivePose)]
    survive_default_external_pose_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 596
if _libs["survive"].has("survive_default_external_velocity_process", "cdecl"):
    survive_default_external_velocity_process = _libs["survive"].get("survive_default_external_velocity_process", "cdecl")
    survive_default_external_velocity_process.argtypes = [POINTER(SurviveContext), String, POINTER(SurviveVelocity)]
    survive_default_external_velocity_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 598
if _libs["survive"].has("survive_default_lighthouse_pose_process", "cdecl"):
    survive_default_lighthouse_pose_process = _libs["survive"].get("survive_default_lighthouse_pose_process", "cdecl")
    survive_default_lighthouse_pose_process.argtypes = [POINTER(SurviveContext), c_uint8, POINTER(SurvivePose)]
    survive_default_lighthouse_pose_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 600
if _libs["survive"].has("survive_default_config_process", "cdecl"):
    survive_default_config_process = _libs["survive"].get("survive_default_config_process", "cdecl")
    survive_default_config_process.argtypes = [POINTER(SurviveObject), String, c_int]
    survive_default_config_process.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 601
if _libs["survive"].has("survive_default_gen_detected_process", "cdecl"):
    survive_default_gen_detected_process = _libs["survive"].get("survive_default_gen_detected_process", "cdecl")
    survive_default_gen_detected_process.argtypes = [POINTER(SurviveObject), c_int]
    survive_default_gen_detected_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 602
if _libs["survive"].has("survive_default_new_object_process", "cdecl"):
    survive_default_new_object_process = _libs["survive"].get("survive_default_new_object_process", "cdecl")
    survive_default_new_object_process.argtypes = [POINTER(SurviveObject)]
    survive_default_new_object_process.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 603
if _libs["survive"].has("survive_run_time", "cdecl"):
    survive_run_time = _libs["survive"].get("survive_run_time", "cdecl")
    survive_run_time.argtypes = [POINTER(SurviveContext)]
    survive_run_time.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 604
if _libs["survive"].has("survive_run_time_since_epoch", "cdecl"):
    survive_run_time_since_epoch = _libs["survive"].get("survive_run_time_since_epoch", "cdecl")
    survive_run_time_since_epoch.argtypes = [POINTER(SurviveContext)]
    survive_run_time_since_epoch.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 606
if _libs["survive"].has("survive_input_event_count", "cdecl"):
    survive_input_event_count = _libs["survive"].get("survive_input_event_count", "cdecl")
    survive_input_event_count.argtypes = [POINTER(SurviveContext)]
    survive_input_event_count.restype = c_size_t

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 609
if _libs["survive"].has("RegisterDriver", "cdecl"):
    RegisterDriver = _libs["survive"].get("RegisterDriver", "cdecl")
    RegisterDriver.argtypes = [String, survive_driver_fn]
    RegisterDriver.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 610
if _libs["survive"].has("RegisterPoserDriver", "cdecl"):
    RegisterPoserDriver = _libs["survive"].get("RegisterPoserDriver", "cdecl")
    RegisterPoserDriver.argtypes = [String, PoserCB]
    RegisterPoserDriver.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 633
if _libs["survive"].has("survive_add_object", "cdecl"):
    survive_add_object = _libs["survive"].get("survive_add_object", "cdecl")
    survive_add_object.argtypes = [POINTER(SurviveContext), POINTER(SurviveObject)]
    survive_add_object.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 634
if _libs["survive"].has("survive_remove_object", "cdecl"):
    survive_remove_object = _libs["survive"].get("survive_remove_object", "cdecl")
    survive_remove_object.argtypes = [POINTER(SurviveContext), POINTER(SurviveObject)]
    survive_remove_object.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 635
if _libs["survive"].has("survive_get_driver", "cdecl"):
    survive_get_driver = _libs["survive"].get("survive_get_driver", "cdecl")
    survive_get_driver.argtypes = [POINTER(SurviveContext), DeviceDriverCb]
    survive_get_driver.restype = POINTER(c_ubyte)
    survive_get_driver.errcheck = lambda v,*a : cast(v, c_void_p)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 636
if _libs["survive"].has("survive_get_driver_by_closefn", "cdecl"):
    survive_get_driver_by_closefn = _libs["survive"].get("survive_get_driver_by_closefn", "cdecl")
    survive_get_driver_by_closefn.argtypes = [POINTER(SurviveContext), DeviceDriverCb]
    survive_get_driver_by_closefn.restype = POINTER(c_ubyte)
    survive_get_driver_by_closefn.errcheck = lambda v,*a : cast(v, c_void_p)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 637
if _libs["survive"].has("survive_add_driver", "cdecl"):
    survive_add_driver = _libs["survive"].get("survive_add_driver", "cdecl")
    survive_add_driver.argtypes = [POINTER(SurviveContext), POINTER(None), DeviceDriverCb, DeviceDriverCb]
    survive_add_driver.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 639
if _libs["survive"].has("survive_add_threaded_driver", "cdecl"):
    survive_add_threaded_driver = _libs["survive"].get("survive_add_threaded_driver", "cdecl")
    survive_add_threaded_driver.argtypes = [POINTER(SurviveContext), POINTER(None), String, CFUNCTYPE(UNCHECKED(POINTER(c_ubyte)), POINTER(None)), DeviceDriverCb]
    survive_add_threaded_driver.restype = POINTER(c_bool)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 641
if _libs["survive"].has("survive_export_config", "cdecl"):
    survive_export_config = _libs["survive"].get("survive_export_config", "cdecl")
    survive_export_config.argtypes = [POINTER(SurviveObject)]
    if sizeof(c_int) == sizeof(c_void_p):
        survive_export_config.restype = ReturnString
    else:
        survive_export_config.restype = String
        survive_export_config.errcheck = ReturnString

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 642
if _libs["survive"].has("survive_reset_lighthouse_positions", "cdecl"):
    survive_reset_lighthouse_positions = _libs["survive"].get("survive_reset_lighthouse_positions", "cdecl")
    survive_reset_lighthouse_positions.argtypes = [POINTER(SurviveContext)]
    survive_reset_lighthouse_positions.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 643
if _libs["survive"].has("survive_reset_lighthouse_position", "cdecl"):
    survive_reset_lighthouse_position = _libs["survive"].get("survive_reset_lighthouse_position", "cdecl")
    survive_reset_lighthouse_position.argtypes = [POINTER(SurviveContext), c_int]
    survive_reset_lighthouse_position.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 646
if _libs["survive"].has("survive_map_sensor_id", "cdecl"):
    survive_map_sensor_id = _libs["survive"].get("survive_map_sensor_id", "cdecl")
    survive_map_sensor_id.argtypes = [POINTER(SurviveObject), c_uint8]
    survive_map_sensor_id.restype = c_uint8

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 647
if _libs["survive"].has("handle_lightcap", "cdecl"):
    handle_lightcap = _libs["survive"].get("handle_lightcap", "cdecl")
    handle_lightcap.argtypes = [POINTER(SurviveObject), POINTER(LightcapElement)]
    handle_lightcap.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 649
if _libs["survive"].has("survive_colorize", "cdecl"):
    survive_colorize = _libs["survive"].get("survive_colorize", "cdecl")
    survive_colorize.argtypes = [String]
    survive_colorize.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 650
if _libs["survive"].has("survive_colorize_codename", "cdecl"):
    survive_colorize_codename = _libs["survive"].get("survive_colorize_codename", "cdecl")
    survive_colorize_codename.argtypes = [POINTER(SurviveObject)]
    survive_colorize_codename.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 651
if _libs["survive"].has("survive_hash", "cdecl"):
    survive_hash = _libs["survive"].get("survive_hash", "cdecl")
    survive_hash.argtypes = [POINTER(c_uint8), c_size_t]
    survive_hash.restype = c_uint32

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 652
if _libs["survive"].has("survive_hash_str", "cdecl"):
    survive_hash_str = _libs["survive"].get("survive_hash_str", "cdecl")
    survive_hash_str.argtypes = [String]
    survive_hash_str.restype = c_uint32

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 725
for _lib in _libs.values():
    try:
        ctx = (POINTER(struct_SurviveContext)).in_dll(_lib, "ctx")
        break
    except:
        pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 726
for _lib in _libs.values():
    try:
        stbuff = (c_char * int(1024)).in_dll(_lib, "stbuff")
        break
    except:
        pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 726
for _lib in _libs.values():
    try:
        start_time = (c_double).in_dll(_lib, "start_time")
        break
    except:
        pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 727
for _lib in _libs.values():
    try:
        start_time = (c_double).in_dll(_lib, "start_time")
        break
    except:
        pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 737
for _lib in _libs.values():
    try:
        ctx = (POINTER(struct_SurviveContext)).in_dll(_lib, "ctx")
        break
    except:
        pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 738
for _lib in _libs.values():
    try:
        stbuff = (c_char * int(1024)).in_dll(_lib, "stbuff")
        break
    except:
        pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 738
for _lib in _libs.values():
    try:
        start_time = (c_double).in_dll(_lib, "start_time")
        break
    except:
        pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 739
for _lib in _libs.values():
    try:
        start_time = (c_double).in_dll(_lib, "start_time")
        break
    except:
        pass

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

SurviveSimpleEventType_ButtonEvent = 1# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 28

SurviveSimpleEventType_ConfigEvent = 2# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 28

SurviveSimpleEventType_PoseUpdateEvent = 3# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 28

SurviveSimpleEventType_Shutdown = 4# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 28

SurviveSimpleEventType_DeviceAdded = 5# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 28

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 238
class struct_SurviveSimpleEvent(Structure):
    pass

SurviveSimpleEvent = struct_SurviveSimpleEvent# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 38

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 50
class struct_SurviveSimpleButtonEvent(Structure):
    pass

struct_SurviveSimpleButtonEvent.__slots__ = [
    'time',
    'object',
    'event_type',
    'button_id',
    'axis_count',
    'axis_ids',
    'axis_val',
]
struct_SurviveSimpleButtonEvent._fields_ = [
    ('time', c_double),
    ('object', POINTER(SurviveSimpleObject)),
    ('event_type', enum_SurviveInputEvent),
    ('button_id', enum_SurviveButton),
    ('axis_count', c_uint8),
    ('axis_ids', enum_SurviveAxis * int(8)),
    ('axis_val', SurviveAxisVal_t * int(8)),
]

SurviveSimpleButtonEvent = struct_SurviveSimpleButtonEvent# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 50

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 56
class struct_SurviveSimpleConfigEvent(Structure):
    pass

struct_SurviveSimpleConfigEvent.__slots__ = [
    'time',
    'object',
    'cfg',
]
struct_SurviveSimpleConfigEvent._fields_ = [
    ('time', c_double),
    ('object', POINTER(SurviveSimpleObject)),
    ('cfg', String),
]

SurviveSimpleConfigEvent = struct_SurviveSimpleConfigEvent# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 56

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 63
class struct_SurviveSimplePoseUpdatedEvent(Structure):
    pass

struct_SurviveSimplePoseUpdatedEvent.__slots__ = [
    'time',
    'object',
    'pose',
    'velocity',
]
struct_SurviveSimplePoseUpdatedEvent._fields_ = [
    ('time', c_double),
    ('object', POINTER(SurviveSimpleObject)),
    ('pose', SurvivePose),
    ('velocity', SurviveVelocity),
]

SurviveSimplePoseUpdatedEvent = struct_SurviveSimplePoseUpdatedEvent# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 63

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 68
class struct_SurviveSimpleObjectEvent(Structure):
    pass

struct_SurviveSimpleObjectEvent.__slots__ = [
    'time',
    'object',
]
struct_SurviveSimpleObjectEvent._fields_ = [
    ('time', c_double),
    ('object', POINTER(SurviveSimpleObject)),
]

SurviveSimpleObjectEvent = struct_SurviveSimpleObjectEvent# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 68

SurviveSimpleDeviceAddedEvent = struct_SurviveSimpleObjectEvent# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 70

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 75
if _libs["survive"].has("survive_simple_init", "cdecl"):
    survive_simple_init = _libs["survive"].get("survive_simple_init", "cdecl")
    survive_simple_init.argtypes = [c_int, POINTER(POINTER(c_char))]
    survive_simple_init.restype = POINTER(SurviveSimpleContext)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 76
if _libs["survive"].has("survive_simple_set_user", "cdecl"):
    survive_simple_set_user = _libs["survive"].get("survive_simple_set_user", "cdecl")
    survive_simple_set_user.argtypes = [POINTER(SurviveSimpleContext), POINTER(None)]
    survive_simple_set_user.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 77
if _libs["survive"].has("survive_simple_get_user", "cdecl"):
    survive_simple_get_user = _libs["survive"].get("survive_simple_get_user", "cdecl")
    survive_simple_get_user.argtypes = [POINTER(SurviveSimpleContext)]
    survive_simple_get_user.restype = POINTER(c_ubyte)
    survive_simple_get_user.errcheck = lambda v,*a : cast(v, c_void_p)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 78
if _libs["survive"].has("survive_simple_init_with_logger", "cdecl"):
    survive_simple_init_with_logger = _libs["survive"].get("survive_simple_init_with_logger", "cdecl")
    survive_simple_init_with_logger.argtypes = [c_int, POINTER(POINTER(c_char)), SurviveSimpleLogFn]
    survive_simple_init_with_logger.restype = POINTER(SurviveSimpleContext)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 84
if _libs["survive"].has("survive_simple_close", "cdecl"):
    survive_simple_close = _libs["survive"].get("survive_simple_close", "cdecl")
    survive_simple_close.argtypes = [POINTER(SurviveSimpleContext)]
    survive_simple_close.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 89
if _libs["survive"].has("survive_simple_start_thread", "cdecl"):
    survive_simple_start_thread = _libs["survive"].get("survive_simple_start_thread", "cdecl")
    survive_simple_start_thread.argtypes = [POINTER(SurviveSimpleContext)]
    survive_simple_start_thread.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 94
if _libs["survive"].has("survive_simple_is_running", "cdecl"):
    survive_simple_is_running = _libs["survive"].get("survive_simple_is_running", "cdecl")
    survive_simple_is_running.argtypes = [POINTER(SurviveSimpleContext)]
    survive_simple_is_running.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 99
if _libs["survive"].has("survive_simple_get_first_object", "cdecl"):
    survive_simple_get_first_object = _libs["survive"].get("survive_simple_get_first_object", "cdecl")
    survive_simple_get_first_object.argtypes = [POINTER(SurviveSimpleContext)]
    survive_simple_get_first_object.restype = POINTER(SurviveSimpleObject)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 103
if _libs["survive"].has("survive_simple_get_next_object", "cdecl"):
    survive_simple_get_next_object = _libs["survive"].get("survive_simple_get_next_object", "cdecl")
    survive_simple_get_next_object.argtypes = [POINTER(SurviveSimpleContext), POINTER(SurviveSimpleObject)]
    survive_simple_get_next_object.restype = POINTER(SurviveSimpleObject)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 108
if _libs["survive"].has("survive_simple_get_object", "cdecl"):
    survive_simple_get_object = _libs["survive"].get("survive_simple_get_object", "cdecl")
    survive_simple_get_object.argtypes = [POINTER(SurviveSimpleContext), String]
    survive_simple_get_object.restype = POINTER(SurviveSimpleObject)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 110
if _libs["survive"].has("survive_simple_get_object_count", "cdecl"):
    survive_simple_get_object_count = _libs["survive"].get("survive_simple_get_object_count", "cdecl")
    survive_simple_get_object_count.argtypes = [POINTER(SurviveSimpleContext)]
    survive_simple_get_object_count.restype = c_size_t

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 115
if _libs["survive"].has("survive_simple_get_next_updated", "cdecl"):
    survive_simple_get_next_updated = _libs["survive"].get("survive_simple_get_next_updated", "cdecl")
    survive_simple_get_next_updated.argtypes = [POINTER(SurviveSimpleContext)]
    survive_simple_get_next_updated.restype = POINTER(SurviveSimpleObject)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 121
if _libs["survive"].has("survive_simple_object_get_latest_pose", "cdecl"):
    survive_simple_object_get_latest_pose = _libs["survive"].get("survive_simple_object_get_latest_pose", "cdecl")
    survive_simple_object_get_latest_pose.argtypes = [POINTER(SurviveSimpleObject), POINTER(SurvivePose)]
    survive_simple_object_get_latest_pose.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 123
if _libs["survive"].has("survive_simple_object_get_transform_to_imu", "cdecl"):
    survive_simple_object_get_transform_to_imu = _libs["survive"].get("survive_simple_object_get_transform_to_imu", "cdecl")
    survive_simple_object_get_transform_to_imu.argtypes = [POINTER(SurviveSimpleObject), POINTER(SurvivePose)]
    survive_simple_object_get_transform_to_imu.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 129
if _libs["survive"].has("survive_simple_object_get_latest_velocity", "cdecl"):
    survive_simple_object_get_latest_velocity = _libs["survive"].get("survive_simple_object_get_latest_velocity", "cdecl")
    survive_simple_object_get_latest_velocity.argtypes = [POINTER(SurviveSimpleObject), POINTER(SurviveVelocity)]
    survive_simple_object_get_latest_velocity.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 134
if _libs["survive"].has("survive_simple_object_charging", "cdecl"):
    survive_simple_object_charging = _libs["survive"].get("survive_simple_object_charging", "cdecl")
    survive_simple_object_charging.argtypes = [POINTER(SurviveSimpleObject)]
    survive_simple_object_charging.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 139
if _libs["survive"].has("survive_simple_object_charge_percet", "cdecl"):
    survive_simple_object_charge_percet = _libs["survive"].get("survive_simple_object_charge_percet", "cdecl")
    survive_simple_object_charge_percet.argtypes = [POINTER(SurviveSimpleObject)]
    survive_simple_object_charge_percet.restype = c_uint8

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 144
if _libs["survive"].has("survive_simple_object_name", "cdecl"):
    survive_simple_object_name = _libs["survive"].get("survive_simple_object_name", "cdecl")
    survive_simple_object_name.argtypes = [POINTER(SurviveSimpleObject)]
    survive_simple_object_name.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 149
if _libs["survive"].has("survive_simple_serial_number", "cdecl"):
    survive_simple_serial_number = _libs["survive"].get("survive_simple_serial_number", "cdecl")
    survive_simple_serial_number.argtypes = [POINTER(SurviveSimpleObject)]
    survive_simple_serial_number.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 154
if _libs["survive"].has("survive_simple_json_config", "cdecl"):
    survive_simple_json_config = _libs["survive"].get("survive_simple_json_config", "cdecl")
    survive_simple_json_config.argtypes = [POINTER(SurviveSimpleObject)]
    survive_simple_json_config.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 160
if _libs["survive"].has("survive_simple_wait_for_update", "cdecl"):
    survive_simple_wait_for_update = _libs["survive"].get("survive_simple_wait_for_update", "cdecl")
    survive_simple_wait_for_update.argtypes = [POINTER(SurviveSimpleContext)]
    survive_simple_wait_for_update.restype = c_bool

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 164
if _libs["survive"].has("survive_simple_next_event", "cdecl"):
    survive_simple_next_event = _libs["survive"].get("survive_simple_next_event", "cdecl")
    survive_simple_next_event.argtypes = [POINTER(SurviveSimpleContext), POINTER(SurviveSimpleEvent)]
    survive_simple_next_event.restype = enum_SurviveSimpleEventType

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 171
if _libs["survive"].has("survive_simple_wait_for_event", "cdecl"):
    survive_simple_wait_for_event = _libs["survive"].get("survive_simple_wait_for_event", "cdecl")
    survive_simple_wait_for_event.argtypes = [POINTER(SurviveSimpleContext), POINTER(SurviveSimpleEvent)]
    survive_simple_wait_for_event.restype = enum_SurviveSimpleEventType

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 174
if _libs["survive"].has("survive_simple_object_haptic", "cdecl"):
    survive_simple_object_haptic = _libs["survive"].get("survive_simple_object_haptic", "cdecl")
    survive_simple_object_haptic.argtypes = [POINTER(struct_SurviveSimpleObject), c_double, c_double, c_double]
    survive_simple_object_haptic.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 176
if _libs["survive"].has("survive_simple_object_get_type", "cdecl"):
    survive_simple_object_get_type = _libs["survive"].get("survive_simple_object_get_type", "cdecl")
    survive_simple_object_get_type.argtypes = [POINTER(struct_SurviveSimpleObject)]
    survive_simple_object_get_type.restype = enum_SurviveSimpleObject_type

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 177
if _libs["survive"].has("survive_simple_object_get_input_axis", "cdecl"):
    survive_simple_object_get_input_axis = _libs["survive"].get("survive_simple_object_get_input_axis", "cdecl")
    survive_simple_object_get_input_axis.argtypes = [POINTER(struct_SurviveSimpleObject), enum_SurviveAxis]
    survive_simple_object_get_input_axis.restype = SurviveAxisVal_t

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 179
if _libs["survive"].has("survive_simple_object_get_button_mask", "cdecl"):
    survive_simple_object_get_button_mask = _libs["survive"].get("survive_simple_object_get_button_mask", "cdecl")
    survive_simple_object_get_button_mask.argtypes = [POINTER(struct_SurviveSimpleObject)]
    survive_simple_object_get_button_mask.restype = c_int32

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 180
if _libs["survive"].has("survive_simple_object_get_touch_mask", "cdecl"):
    survive_simple_object_get_touch_mask = _libs["survive"].get("survive_simple_object_get_touch_mask", "cdecl")
    survive_simple_object_get_touch_mask.argtypes = [POINTER(struct_SurviveSimpleObject)]
    survive_simple_object_get_touch_mask.restype = c_int32

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 182
if _libs["survive"].has("survive_simple_object_get_subtype", "cdecl"):
    survive_simple_object_get_subtype = _libs["survive"].get("survive_simple_object_get_subtype", "cdecl")
    survive_simple_object_get_subtype.argtypes = [POINTER(struct_SurviveSimpleObject)]
    survive_simple_object_get_subtype.restype = SurviveSimpleSubobject_type

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 187
if _libs["survive"].has("survive_simple_get_button_event", "cdecl"):
    survive_simple_get_button_event = _libs["survive"].get("survive_simple_get_button_event", "cdecl")
    survive_simple_get_button_event.argtypes = [POINTER(SurviveSimpleEvent)]
    survive_simple_get_button_event.restype = POINTER(SurviveSimpleButtonEvent)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 189
if _libs["survive"].has("survive_simple_get_object_event", "cdecl"):
    survive_simple_get_object_event = _libs["survive"].get("survive_simple_get_object_event", "cdecl")
    survive_simple_get_object_event.argtypes = [POINTER(SurviveSimpleEvent)]
    survive_simple_get_object_event.restype = POINTER(struct_SurviveSimpleObjectEvent)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 195
if _libs["survive"].has("survive_simple_get_pose_updated_event", "cdecl"):
    survive_simple_get_pose_updated_event = _libs["survive"].get("survive_simple_get_pose_updated_event", "cdecl")
    survive_simple_get_pose_updated_event.argtypes = [POINTER(SurviveSimpleEvent)]
    survive_simple_get_pose_updated_event.restype = POINTER(struct_SurviveSimplePoseUpdatedEvent)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 202
if _libs["survive"].has("survive_simple_get_config_event", "cdecl"):
    survive_simple_get_config_event = _libs["survive"].get("survive_simple_get_config_event", "cdecl")
    survive_simple_get_config_event.argtypes = [POINTER(SurviveSimpleEvent)]
    survive_simple_get_config_event.restype = POINTER(SurviveSimpleConfigEvent)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 204
if _libs["survive"].has("survive_simple_run_time", "cdecl"):
    survive_simple_run_time = _libs["survive"].get("survive_simple_run_time", "cdecl")
    survive_simple_run_time.argtypes = [POINTER(struct_SurviveSimpleContext)]
    survive_simple_run_time.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 205
if _libs["survive"].has("survive_simple_run_time_since_epoch", "cdecl"):
    survive_simple_run_time_since_epoch = _libs["survive"].get("survive_simple_run_time_since_epoch", "cdecl")
    survive_simple_run_time_since_epoch.argtypes = [POINTER(struct_SurviveSimpleContext)]
    survive_simple_run_time_since_epoch.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 240
class union_anon_53(Union):
    pass

union_anon_53.__slots__ = [
    '__private_object_event',
    '__private_button_event',
    '__private_config_event',
    '__private_pose_event',
]
union_anon_53._fields_ = [
    ('__private_object_event', SurviveSimpleObjectEvent),
    ('__private_button_event', SurviveSimpleButtonEvent),
    ('__private_config_event', SurviveSimpleConfigEvent),
    ('__private_pose_event', SurviveSimplePoseUpdatedEvent),
]

struct_SurviveSimpleEvent.__slots__ = [
    'event_type',
    'd',
]
struct_SurviveSimpleEvent._fields_ = [
    ('event_type', enum_SurviveSimpleEventType),
    ('d', union_anon_53),
]

SurviveAngleReading = c_double * int(2)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 22

survive_reproject_axis_fn_t = CFUNCTYPE(UNCHECKED(c_double), POINTER(BaseStationCal), POINTER(c_double))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 24

survive_reproject_xy_fn_t = CFUNCTYPE(UNCHECKED(None), POINTER(BaseStationCal), LinmathVec3d, POINTER(c_double))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 25

survive_reproject_full_xy_fn_t = CFUNCTYPE(UNCHECKED(c_double), POINTER(SurvivePose), LinmathVec3d, POINTER(SurvivePose), POINTER(BaseStationCal))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 27

survive_reproject_axis_jacob_sensor_pt_fn_t = CFUNCTYPE(UNCHECKED(None), POINTER(c_double), POINTER(SurvivePose), POINTER(c_double), POINTER(SurvivePose), POINTER(BaseStationCal))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 30

survive_reproject_axisangle_axis_jacob_sensor_pt_fn_t = CFUNCTYPE(UNCHECKED(None), POINTER(c_double), POINTER(LinmathAxisAnglePose), POINTER(c_double), POINTER(LinmathAxisAnglePose), POINTER(BaseStationCal))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 32

survive_reproject_axis_jacob_fn_t = CFUNCTYPE(UNCHECKED(None), POINTER(c_double), POINTER(SurvivePose), LinmathPoint3d, POINTER(SurvivePose), POINTER(BaseStationCal))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 38

survive_reproject_full_jac_obj_pose_fn_t = CFUNCTYPE(UNCHECKED(None), POINTER(c_double), POINTER(SurvivePose), LinmathVec3d, POINTER(SurvivePose), POINTER(BaseStationCal))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 40

survive_reproject_full_jac_lh_pose_fn_t = survive_reproject_full_jac_obj_pose_fn_t# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 44

survive_reproject_axis_jacob_lh_pose_fn_t = survive_reproject_axis_jacob_fn_t# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 45

survive_reproject_axisangle_axis_jacob_fn_t = CFUNCTYPE(UNCHECKED(None), POINTER(c_double), POINTER(LinmathAxisAnglePose), LinmathPoint3d, POINTER(LinmathAxisAnglePose), POINTER(BaseStationCal))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 47

survive_reproject_axisangle_full_jac_obj_pose_fn_t = CFUNCTYPE(UNCHECKED(None), POINTER(c_double), POINTER(LinmathAxisAnglePose), LinmathVec3d, POINTER(LinmathAxisAnglePose), POINTER(BaseStationCal))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 50

survive_reproject_axisangle_full_jac_lh_pose_fn_t = survive_reproject_axisangle_axis_jacob_fn_t# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 55

survive_reproject_axisangle_axis_jacob_lh_pose_fn_t = survive_reproject_axisangle_full_jac_obj_pose_fn_t# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 56

survive_reproject_axisangle_full_xy_fn_t = CFUNCTYPE(UNCHECKED(c_double), POINTER(LinmathAxisAnglePose), LinmathVec3d, POINTER(LinmathAxisAnglePose), POINTER(BaseStationCal))# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 58

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 83
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
    'reprojectAxisangleFullXyFn',
    'reprojectAxisAngleFullJacObjPose',
    'reprojectAxisAngleAxisJacobFn',
    'reprojectAxisAngleFullJacLhPose',
    'reprojectAxisAngleAxisJacobLhPoseFn',
    'reprojectAxisJacobSensorPt',
    'reprojectAxisAngleAxisJacobSensorPt',
]
struct_survive_reproject_model_t._fields_ = [
    ('reprojectXY', survive_reproject_xy_fn_t),
    ('reprojectAxisFn', survive_reproject_axis_fn_t * int(2)),
    ('reprojectAxisFullFn', survive_reproject_full_xy_fn_t * int(2)),
    ('reprojectFullJacObjPose', survive_reproject_full_jac_obj_pose_fn_t),
    ('reprojectAxisJacobFn', survive_reproject_axis_jacob_fn_t * int(2)),
    ('reprojectFullJacLhPose', survive_reproject_full_jac_lh_pose_fn_t),
    ('reprojectAxisJacobLhPoseFn', survive_reproject_axis_jacob_lh_pose_fn_t * int(2)),
    ('reprojectAxisangleFullXyFn', survive_reproject_axisangle_full_xy_fn_t * int(2)),
    ('reprojectAxisAngleFullJacObjPose', survive_reproject_axisangle_full_jac_obj_pose_fn_t),
    ('reprojectAxisAngleAxisJacobFn', survive_reproject_axisangle_axis_jacob_fn_t * int(2)),
    ('reprojectAxisAngleFullJacLhPose', survive_reproject_axisangle_full_jac_lh_pose_fn_t),
    ('reprojectAxisAngleAxisJacobLhPoseFn', survive_reproject_axisangle_axis_jacob_lh_pose_fn_t * int(2)),
    ('reprojectAxisJacobSensorPt', survive_reproject_axis_jacob_sensor_pt_fn_t * int(2)),
    ('reprojectAxisAngleAxisJacobSensorPt', survive_reproject_axisangle_axis_jacob_sensor_pt_fn_t * int(2)),
]

survive_reproject_model_t = struct_survive_reproject_model_t# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 83

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 85
if _libs["survive"].has("survive_reproject_model", "cdecl"):
    survive_reproject_model = _libs["survive"].get("survive_reproject_model", "cdecl")
    survive_reproject_model.argtypes = [POINTER(SurviveContext)]
    survive_reproject_model.restype = POINTER(survive_reproject_model_t)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 86
try:
    survive_reproject_gen1_model = (survive_reproject_model_t).in_dll(_libs["survive"], "survive_reproject_gen1_model")
except:
    pass

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 88
if _libs["survive"].has("survive_reproject_axis_x", "cdecl"):
    survive_reproject_axis_x = _libs["survive"].get("survive_reproject_axis_x", "cdecl")
    survive_reproject_axis_x.argtypes = [POINTER(BaseStationCal), LinmathVec3d]
    survive_reproject_axis_x.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 89
if _libs["survive"].has("survive_reproject_axis_y", "cdecl"):
    survive_reproject_axis_y = _libs["survive"].get("survive_reproject_axis_y", "cdecl")
    survive_reproject_axis_y.argtypes = [POINTER(BaseStationCal), LinmathVec3d]
    survive_reproject_axis_y.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 91
if _libs["survive"].has("survive_reproject_xy", "cdecl"):
    survive_reproject_xy = _libs["survive"].get("survive_reproject_xy", "cdecl")
    survive_reproject_xy.argtypes = [POINTER(BaseStationCal), LinmathVec3d, SurviveAngleReading]
    survive_reproject_xy.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 92
if _libs["survive"].has("survive_reproject_from_pose", "cdecl"):
    survive_reproject_from_pose = _libs["survive"].get("survive_reproject_from_pose", "cdecl")
    survive_reproject_from_pose.argtypes = [POINTER(SurviveContext), c_int, POINTER(SurvivePose), LinmathVec3d, SurviveAngleReading]
    survive_reproject_from_pose.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 95
for _lib in _libs.values():
    if not _lib.has("survive_reproject_full_jac_obj_pose", "cdecl"):
        continue
    survive_reproject_full_jac_obj_pose = _lib.get("survive_reproject_full_jac_obj_pose", "cdecl")
    survive_reproject_full_jac_obj_pose.argtypes = [SurviveAngleReading, POINTER(SurvivePose), LinmathVec3d, POINTER(SurvivePose), POINTER(BaseStationCal)]
    survive_reproject_full_jac_obj_pose.restype = None
    break

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 99
for _lib in _libs.values():
    if not _lib.has("survive_reproject_full_x_jac_obj_pose", "cdecl"):
        continue
    survive_reproject_full_x_jac_obj_pose = _lib.get("survive_reproject_full_x_jac_obj_pose", "cdecl")
    survive_reproject_full_x_jac_obj_pose.argtypes = [SurviveAngleReading, POINTER(SurvivePose), LinmathVec3d, POINTER(SurvivePose), POINTER(BaseStationCal)]
    survive_reproject_full_x_jac_obj_pose.restype = None
    break

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 103
for _lib in _libs.values():
    if not _lib.has("survive_reproject_full_y_jac_obj_pose", "cdecl"):
        continue
    survive_reproject_full_y_jac_obj_pose = _lib.get("survive_reproject_full_y_jac_obj_pose", "cdecl")
    survive_reproject_full_y_jac_obj_pose.argtypes = [SurviveAngleReading, POINTER(SurvivePose), LinmathVec3d, POINTER(SurvivePose), POINTER(BaseStationCal)]
    survive_reproject_full_y_jac_obj_pose.restype = None
    break

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 107
if _libs["survive"].has("survive_reproject_full", "cdecl"):
    survive_reproject_full = _libs["survive"].get("survive_reproject_full", "cdecl")
    survive_reproject_full.argtypes = [POINTER(BaseStationCal), POINTER(SurvivePose), POINTER(SurvivePose), LinmathVec3d, SurviveAngleReading]
    survive_reproject_full.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 109
if _libs["survive"].has("survive_reproject_full_axisangle", "cdecl"):
    survive_reproject_full_axisangle = _libs["survive"].get("survive_reproject_full_axisangle", "cdecl")
    survive_reproject_full_axisangle.argtypes = [POINTER(BaseStationCal), POINTER(LinmathAxisAnglePose), POINTER(LinmathAxisAnglePose), LinmathVec3d, SurviveAngleReading]
    survive_reproject_full_axisangle.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 121
if _libs["survive"].has("survive_reproject_from_pose_with_bcal", "cdecl"):
    survive_reproject_from_pose_with_bcal = _libs["survive"].get("survive_reproject_from_pose_with_bcal", "cdecl")
    survive_reproject_from_pose_with_bcal.argtypes = [POINTER(BaseStationCal), POINTER(SurvivePose), LinmathVec3d, SurviveAngleReading]
    survive_reproject_from_pose_with_bcal.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 124
if _libs["survive"].has("survive_reproject", "cdecl"):
    survive_reproject = _libs["survive"].get("survive_reproject", "cdecl")
    survive_reproject.argtypes = [POINTER(SurviveContext), c_int, LinmathVec3d, SurviveAngleReading]
    survive_reproject.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 132
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
    'covar_free',
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
    ('covar_free', POINTER(c_double)),
    ('version', c_char * int(20)),
]

mp_config = struct_mp_config_struct# /home/justin/source/oss/libsurvive/redist/mpfit/mpfit.h: 129

enum_survive_optimizer_measurement_type = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 14

survive_optimizer_measurement_type_none = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 14

survive_optimizer_measurement_type_light = (survive_optimizer_measurement_type_none + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 14

survive_optimizer_measurement_type_object_pose = (survive_optimizer_measurement_type_light + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 14

survive_optimizer_measurement_type_object_accel = (survive_optimizer_measurement_type_object_pose + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 14

survive_optimizer_measurement_type_camera_accel = (survive_optimizer_measurement_type_object_accel + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 14

enum_survive_optimizer_parameter_type = c_int# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 22

survive_optimizer_parameter_none = 0# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 22

survive_optimizer_parameter_object_pose = (survive_optimizer_parameter_none + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 22

survive_optimizer_parameter_object_velocity = (survive_optimizer_parameter_object_pose + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 22

survive_optimizer_parameter_object_scale = (survive_optimizer_parameter_object_velocity + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 22

survive_optimizer_parameter_camera = (survive_optimizer_parameter_object_scale + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 22

survive_optimizer_parameter_camera_parameters = (survive_optimizer_parameter_camera + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 22

survive_optimizer_parameter_obj_points = (survive_optimizer_parameter_camera_parameters + 1)# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 22

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 38
class struct_anon_54(Structure):
    pass

struct_anon_54.__slots__ = [
    'value',
    'lh',
    'sensor_idx',
    'axis',
    'object',
]
struct_anon_54._fields_ = [
    ('value', c_double),
    ('lh', c_uint8),
    ('sensor_idx', c_uint8),
    ('axis', c_uint8),
    ('object', c_int),
]

survive_optimizer_light_measurement = struct_anon_54# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 38

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 43
class struct_anon_55(Structure):
    pass

struct_anon_55.__slots__ = [
    'object',
    'pose',
]
struct_anon_55._fields_ = [
    ('object', c_int),
    ('pose', LinmathPose),
]

survive_optimizer_object_pose_measurement = struct_anon_55# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 43

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 48
class struct_anon_56(Structure):
    pass

struct_anon_56.__slots__ = [
    'object',
    'acc',
]
struct_anon_56._fields_ = [
    ('object', c_int),
    ('acc', LinmathVec3d),
]

survive_optimizer_object_acc_measurement = struct_anon_56# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 48

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 53
class struct_anon_57(Structure):
    pass

struct_anon_57.__slots__ = [
    'camera',
    'acc',
]
struct_anon_57._fields_ = [
    ('camera', c_int),
    ('acc', LinmathVec3d),
]

survive_optimizer_camera_acc_measurement = struct_anon_57# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 53

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 63
class union_anon_58(Union):
    pass

union_anon_58.__slots__ = [
    'light',
    'pose',
    'pose_acc',
    'camera_acc',
]
union_anon_58._fields_ = [
    ('light', survive_optimizer_light_measurement),
    ('pose', survive_optimizer_object_pose_measurement),
    ('pose_acc', survive_optimizer_object_acc_measurement),
    ('camera_acc', survive_optimizer_camera_acc_measurement),
]

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 69
class struct_anon_59(Structure):
    pass

struct_anon_59.__slots__ = [
    'time',
    'size',
    'invalid',
    'variance',
    'meas_type',
    'unnamed_1',
]
struct_anon_59._anonymous_ = [
    'unnamed_1',
]
struct_anon_59._fields_ = [
    ('time', c_double),
    ('size', c_size_t),
    ('invalid', c_bool),
    ('variance', c_double),
    ('meas_type', enum_survive_optimizer_measurement_type),
    ('unnamed_1', union_anon_58),
]

survive_optimizer_measurement = struct_anon_59# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 69

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 77
class struct_anon_60(Structure):
    pass

struct_anon_60.__slots__ = [
    'size',
    'elem_size',
    'param_type',
    'pi',
    'p',
]
struct_anon_60._fields_ = [
    ('size', c_size_t),
    ('elem_size', c_size_t),
    ('param_type', enum_survive_optimizer_parameter_type),
    ('pi', POINTER(struct_mp_par_struct)),
    ('p', POINTER(c_double)),
]

survive_optimizer_parameter = struct_anon_60# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 77

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 85
class struct_survive_optimizer_settings(Structure):
    pass

struct_survive_optimizer_settings.__slots__ = [
    'use_quat_model',
    'disallow_pair_calc',
    'optimize_scale_threshold',
    'current_pos_bias',
    'current_rot_bias',
]
struct_survive_optimizer_settings._fields_ = [
    ('use_quat_model', c_bool),
    ('disallow_pair_calc', c_bool),
    ('optimize_scale_threshold', c_double),
    ('current_pos_bias', c_double),
    ('current_rot_bias', c_double),
]

survive_optimizer_settings = struct_survive_optimizer_settings# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 85

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 117
class struct_anon_61(Structure):
    pass

struct_anon_61.__slots__ = [
    'total_meas_cnt',
    'total_lh_cnt',
    'dropped_meas_cnt',
    'dropped_lh_cnt',
    'object_up_error',
    'object_up_error_cnt',
    'sensor_error',
    'sensor_error_cnt',
    'current_error',
    'current_error_cnt',
]
struct_anon_61._fields_ = [
    ('total_meas_cnt', c_uint32),
    ('total_lh_cnt', c_uint32),
    ('dropped_meas_cnt', c_uint32),
    ('dropped_lh_cnt', c_uint32),
    ('object_up_error', c_double),
    ('object_up_error_cnt', c_int),
    ('sensor_error', c_double),
    ('sensor_error_cnt', c_int),
    ('current_error', c_double),
    ('current_error_cnt', c_int),
]

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 90
class struct_survive_optimizer(Structure):
    pass

struct_survive_optimizer.__slots__ = [
    'settings',
    'reprojectModel',
    'sos',
    'measurements',
    'measurementsCnt',
    'parametersCnt',
    'parameterBlockCnt',
    'objectUpVectorVariance',
    'timecode',
    'mp_parameters_info',
    'parameters_info',
    'parameters',
    'disableVelocity',
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
    ('settings', POINTER(survive_optimizer_settings)),
    ('reprojectModel', POINTER(survive_reproject_model_t)),
    ('sos', POINTER(POINTER(SurviveObject))),
    ('measurements', POINTER(survive_optimizer_measurement)),
    ('measurementsCnt', c_size_t),
    ('parametersCnt', c_size_t),
    ('parameterBlockCnt', c_size_t),
    ('objectUpVectorVariance', c_double),
    ('timecode', c_double),
    ('mp_parameters_info', POINTER(struct_mp_par_struct)),
    ('parameters_info', POINTER(survive_optimizer_parameter)),
    ('parameters', POINTER(c_double)),
    ('disableVelocity', c_bool),
    ('poseLength', c_int),
    ('cameraLength', c_int),
    ('ptsLength', c_int),
    ('nofilter', c_bool),
    ('cfg', POINTER(mp_config)),
    ('needsFiltering', c_bool),
    ('stats', struct_anon_61),
    ('user', POINTER(None)),
    ('iteration_cb', CFUNCTYPE(UNCHECKED(None), POINTER(struct_survive_optimizer), c_int, c_int, POINTER(c_double), POINTER(c_double), POINTER(POINTER(c_double)))),
]

survive_optimizer = struct_survive_optimizer# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 130

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 164
if _libs["survive"].has("survive_optimizer_realloc", "cdecl"):
    survive_optimizer_realloc = _libs["survive"].get("survive_optimizer_realloc", "cdecl")
    survive_optimizer_realloc.argtypes = [POINTER(None), c_size_t]
    survive_optimizer_realloc.restype = POINTER(c_ubyte)
    survive_optimizer_realloc.errcheck = lambda v,*a : cast(v, c_void_p)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 166
if _libs["survive"].has("survive_optimizer_get_max_measurements_count", "cdecl"):
    survive_optimizer_get_max_measurements_count = _libs["survive"].get("survive_optimizer_get_max_measurements_count", "cdecl")
    survive_optimizer_get_max_measurements_count.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_max_measurements_count.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 167
if _libs["survive"].has("survive_optimizer_get_max_parameters_count", "cdecl"):
    survive_optimizer_get_max_parameters_count = _libs["survive"].get("survive_optimizer_get_max_parameters_count", "cdecl")
    survive_optimizer_get_max_parameters_count.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_max_parameters_count.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 168
if _libs["survive"].has("survive_optimizer_get_parameters_count", "cdecl"):
    survive_optimizer_get_parameters_count = _libs["survive"].get("survive_optimizer_get_parameters_count", "cdecl")
    survive_optimizer_get_parameters_count.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_parameters_count.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 169
if _libs["survive"].has("survive_optimizer_get_free_parameters_count", "cdecl"):
    survive_optimizer_get_free_parameters_count = _libs["survive"].get("survive_optimizer_get_free_parameters_count", "cdecl")
    survive_optimizer_get_free_parameters_count.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_free_parameters_count.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 171
if _libs["survive"].has("survive_optimizer_get_total_buffer_size", "cdecl"):
    survive_optimizer_get_total_buffer_size = _libs["survive"].get("survive_optimizer_get_total_buffer_size", "cdecl")
    survive_optimizer_get_total_buffer_size.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_total_buffer_size.restype = c_size_t

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 173
if _libs["survive"].has("survive_optimizer_setup_buffers", "cdecl"):
    survive_optimizer_setup_buffers = _libs["survive"].get("survive_optimizer_setup_buffers", "cdecl")
    survive_optimizer_setup_buffers.argtypes = [POINTER(survive_optimizer), POINTER(c_double), POINTER(survive_optimizer_parameter), POINTER(struct_mp_par_struct), POINTER(None), POINTER(None)]
    survive_optimizer_setup_buffers.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 178
if _libs["survive"].has("survive_optimizer_get_pose", "cdecl"):
    survive_optimizer_get_pose = _libs["survive"].get("survive_optimizer_get_pose", "cdecl")
    survive_optimizer_get_pose.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_pose.restype = POINTER(SurvivePose)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 179
if _libs["survive"].has("survive_optimizer_get_velocity_index", "cdecl"):
    survive_optimizer_get_velocity_index = _libs["survive"].get("survive_optimizer_get_velocity_index", "cdecl")
    survive_optimizer_get_velocity_index.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_velocity_index.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 180
if _libs["survive"].has("survive_optimizer_get_velocity", "cdecl"):
    survive_optimizer_get_velocity = _libs["survive"].get("survive_optimizer_get_velocity", "cdecl")
    survive_optimizer_get_velocity.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_velocity.restype = POINTER(SurviveVelocity)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 182
if _libs["survive"].has("survive_optimizer_get_sensor_scale_index", "cdecl"):
    survive_optimizer_get_sensor_scale_index = _libs["survive"].get("survive_optimizer_get_sensor_scale_index", "cdecl")
    survive_optimizer_get_sensor_scale_index.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_sensor_scale_index.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 183
if _libs["survive"].has("survive_optimizer_disable_sensor_scale", "cdecl"):
    survive_optimizer_disable_sensor_scale = _libs["survive"].get("survive_optimizer_disable_sensor_scale", "cdecl")
    survive_optimizer_disable_sensor_scale.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_disable_sensor_scale.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 185
if _libs["survive"].has("survive_optimizer_get_camera_index", "cdecl"):
    survive_optimizer_get_camera_index = _libs["survive"].get("survive_optimizer_get_camera_index", "cdecl")
    survive_optimizer_get_camera_index.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_camera_index.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 187
if _libs["survive"].has("survive_optimizer_get_camera", "cdecl"):
    survive_optimizer_get_camera = _libs["survive"].get("survive_optimizer_get_camera", "cdecl")
    survive_optimizer_get_camera.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_camera.restype = POINTER(SurvivePose)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 189
if _libs["survive"].has("survive_optimizer_get_calibration_index", "cdecl"):
    survive_optimizer_get_calibration_index = _libs["survive"].get("survive_optimizer_get_calibration_index", "cdecl")
    survive_optimizer_get_calibration_index.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_calibration_index.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 191
if _libs["survive"].has("survive_optimizer_get_calibration", "cdecl"):
    survive_optimizer_get_calibration = _libs["survive"].get("survive_optimizer_get_calibration", "cdecl")
    survive_optimizer_get_calibration.argtypes = [POINTER(survive_optimizer), c_int]
    survive_optimizer_get_calibration.restype = POINTER(BaseStationCal)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 193
if _libs["survive"].has("survive_optimizer_get_sensors_index", "cdecl"):
    survive_optimizer_get_sensors_index = _libs["survive"].get("survive_optimizer_get_sensors_index", "cdecl")
    survive_optimizer_get_sensors_index.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_get_sensors_index.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 195
if _libs["survive"].has("survive_optimizer_get_sensors", "cdecl"):
    survive_optimizer_get_sensors = _libs["survive"].get("survive_optimizer_get_sensors", "cdecl")
    survive_optimizer_get_sensors.argtypes = [POINTER(survive_optimizer), c_size_t]
    survive_optimizer_get_sensors.restype = POINTER(c_double)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 197
if _libs["survive"].has("survive_optimizer_setup_pose_n", "cdecl"):
    survive_optimizer_setup_pose_n = _libs["survive"].get("survive_optimizer_setup_pose_n", "cdecl")
    survive_optimizer_setup_pose_n.argtypes = [POINTER(survive_optimizer), POINTER(SurvivePose), c_size_t, c_bool, c_int]
    survive_optimizer_setup_pose_n.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 200
if _libs["survive"].has("survive_optimizer_fix_camera", "cdecl"):
    survive_optimizer_fix_camera = _libs["survive"].get("survive_optimizer_fix_camera", "cdecl")
    survive_optimizer_fix_camera.argtypes = [POINTER(survive_optimizer), c_int]
    survive_optimizer_fix_camera.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 202
if _libs["survive"].has("survive_optimizer_setup_pose", "cdecl"):
    survive_optimizer_setup_pose = _libs["survive"].get("survive_optimizer_setup_pose", "cdecl")
    survive_optimizer_setup_pose.argtypes = [POINTER(survive_optimizer), POINTER(SurvivePose), c_bool, c_int]
    survive_optimizer_setup_pose.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 205
if _libs["survive"].has("survive_optimizer_setup_camera", "cdecl"):
    survive_optimizer_setup_camera = _libs["survive"].get("survive_optimizer_setup_camera", "cdecl")
    survive_optimizer_setup_camera.argtypes = [POINTER(survive_optimizer), c_int8, POINTER(SurvivePose), c_bool, c_int]
    survive_optimizer_setup_camera.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 208
if _libs["survive"].has("survive_optimizer_setup_cameras", "cdecl"):
    survive_optimizer_setup_cameras = _libs["survive"].get("survive_optimizer_setup_cameras", "cdecl")
    survive_optimizer_setup_cameras.argtypes = [POINTER(survive_optimizer), POINTER(SurviveContext), c_bool, c_int]
    survive_optimizer_setup_cameras.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 211
if _libs["survive"].has("survive_optimizer_error", "cdecl"):
    survive_optimizer_error = _libs["survive"].get("survive_optimizer_error", "cdecl")
    survive_optimizer_error.argtypes = [c_int]
    survive_optimizer_error.restype = c_char_p

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 213
if _libs["survive"].has("survive_optimizer_run", "cdecl"):
    survive_optimizer_run = _libs["survive"].get("survive_optimizer_run", "cdecl")
    survive_optimizer_run.argtypes = [POINTER(survive_optimizer), POINTER(struct_mp_result_struct), POINTER(struct_CnMat)]
    survive_optimizer_run.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 216
if _libs["survive"].has("survive_optimizer_set_reproject_model", "cdecl"):
    survive_optimizer_set_reproject_model = _libs["survive"].get("survive_optimizer_set_reproject_model", "cdecl")
    survive_optimizer_set_reproject_model.argtypes = [POINTER(survive_optimizer), POINTER(survive_reproject_model_t)]
    survive_optimizer_set_reproject_model.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 219
if _libs["survive"].has("survive_optimizer_serialize", "cdecl"):
    survive_optimizer_serialize = _libs["survive"].get("survive_optimizer_serialize", "cdecl")
    survive_optimizer_serialize.argtypes = [POINTER(survive_optimizer), String]
    survive_optimizer_serialize.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 221
if _libs["survive"].has("survive_optimizer_load", "cdecl"):
    survive_optimizer_load = _libs["survive"].get("survive_optimizer_load", "cdecl")
    survive_optimizer_load.argtypes = [String]
    survive_optimizer_load.restype = POINTER(survive_optimizer)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 223
if _libs["survive"].has("survive_optimizer_current_norm", "cdecl"):
    survive_optimizer_current_norm = _libs["survive"].get("survive_optimizer_current_norm", "cdecl")
    survive_optimizer_current_norm.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_current_norm.restype = c_double

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 225
if _libs["survive"].has("survive_optimizer_precise_config", "cdecl"):
    survive_optimizer_precise_config = _libs["survive"].get("survive_optimizer_precise_config", "cdecl")
    survive_optimizer_precise_config.argtypes = []
    survive_optimizer_precise_config.restype = POINTER(mp_config)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 227
if _libs["survive"].has("survive_optimizer_nonfixed_cnt", "cdecl"):
    survive_optimizer_nonfixed_cnt = _libs["survive"].get("survive_optimizer_nonfixed_cnt", "cdecl")
    survive_optimizer_nonfixed_cnt.argtypes = [POINTER(survive_optimizer)]
    survive_optimizer_nonfixed_cnt.restype = c_int

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 229
if _libs["survive"].has("survive_optimizer_get_nonfixed", "cdecl"):
    survive_optimizer_get_nonfixed = _libs["survive"].get("survive_optimizer_get_nonfixed", "cdecl")
    survive_optimizer_get_nonfixed.argtypes = [POINTER(survive_optimizer), POINTER(c_double)]
    survive_optimizer_get_nonfixed.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 230
if _libs["survive"].has("survive_optimizer_set_nonfixed", "cdecl"):
    survive_optimizer_set_nonfixed = _libs["survive"].get("survive_optimizer_set_nonfixed", "cdecl")
    survive_optimizer_set_nonfixed.argtypes = [POINTER(survive_optimizer), POINTER(c_double)]
    survive_optimizer_set_nonfixed.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 232
if _libs["survive"].has("survive_optimizer_emplace_meas", "cdecl"):
    survive_optimizer_emplace_meas = _libs["survive"].get("survive_optimizer_emplace_meas", "cdecl")
    survive_optimizer_emplace_meas.argtypes = [POINTER(survive_optimizer), enum_survive_optimizer_measurement_type]
    survive_optimizer_emplace_meas.restype = POINTER(survive_optimizer_measurement)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 234
if _libs["survive"].has("survive_optimizer_emplace_params", "cdecl"):
    survive_optimizer_emplace_params = _libs["survive"].get("survive_optimizer_emplace_params", "cdecl")
    survive_optimizer_emplace_params.argtypes = [POINTER(survive_optimizer), enum_survive_optimizer_parameter_type, c_int]
    survive_optimizer_emplace_params.restype = POINTER(survive_optimizer_parameter)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 237
if _libs["survive"].has("survive_optimizer_pop_meas", "cdecl"):
    survive_optimizer_pop_meas = _libs["survive"].get("survive_optimizer_pop_meas", "cdecl")
    survive_optimizer_pop_meas.argtypes = [POINTER(survive_optimizer), c_int]
    survive_optimizer_pop_meas.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 239
if _libs["survive"].has("survive_optimizer_obj_up_vector", "cdecl"):
    survive_optimizer_obj_up_vector = _libs["survive"].get("survive_optimizer_obj_up_vector", "cdecl")
    survive_optimizer_obj_up_vector.argtypes = [POINTER(survive_optimizer), c_int]
    survive_optimizer_obj_up_vector.restype = POINTER(c_double)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 240
if _libs["survive"].has("survive_optimizer_cam_up_vector", "cdecl"):
    survive_optimizer_cam_up_vector = _libs["survive"].get("survive_optimizer_cam_up_vector", "cdecl")
    survive_optimizer_cam_up_vector.argtypes = [POINTER(survive_optimizer), c_int]
    survive_optimizer_cam_up_vector.restype = POINTER(c_double)

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 241
if _libs["survive"].has("survive_optimizer_set_cam_up_vector", "cdecl"):
    survive_optimizer_set_cam_up_vector = _libs["survive"].get("survive_optimizer_set_cam_up_vector", "cdecl")
    survive_optimizer_set_cam_up_vector.argtypes = [POINTER(survive_optimizer), c_int, c_double, LinmathVec3d]
    survive_optimizer_set_cam_up_vector.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 243
if _libs["survive"].has("survive_optimizer_set_obj_up_vector", "cdecl"):
    survive_optimizer_set_obj_up_vector = _libs["survive"].get("survive_optimizer_set_obj_up_vector", "cdecl")
    survive_optimizer_set_obj_up_vector.argtypes = [POINTER(survive_optimizer), c_int, c_double, LinmathVec3d]
    survive_optimizer_set_obj_up_vector.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 245
if _libs["survive"].has("survive_optimizer_settings_attach_config", "cdecl"):
    survive_optimizer_settings_attach_config = _libs["survive"].get("survive_optimizer_settings_attach_config", "cdecl")
    survive_optimizer_settings_attach_config.argtypes = [POINTER(SurviveContext), POINTER(survive_optimizer_settings)]
    survive_optimizer_settings_attach_config.restype = None

# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 246
if _libs["survive"].has("survive_optimizer_settings_detach_config", "cdecl"):
    survive_optimizer_settings_detach_config = _libs["survive"].get("survive_optimizer_settings_detach_config", "cdecl")
    survive_optimizer_settings_detach_config.argtypes = [POINTER(SurviveContext), POINTER(survive_optimizer_settings)]
    survive_optimizer_settings_detach_config.restype = None

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

survive_kalman_model_t = struct_survive_kalman_model_t# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 126

SurviveObject = struct_SurviveObject# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 128

SurviveContext = struct_SurviveContext# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 321

BaseStationData = struct_BaseStationData# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 256

SurviveCalData = struct_SurviveCalData# /home/justin/source/oss/libsurvive/include/libsurvive/survive_types.h: 214

PoserDataIMU = struct_PoserDataIMU# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 97

PoserDataLight = struct_PoserDataLight# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 107

PoserDataLightGen1 = struct_PoserDataLightGen1# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 114

PoserDataLightGen2 = struct_PoserDataLightGen2# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 121

PoserDataGlobalScene = struct_PoserDataGlobalScene# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 131

PoserDataGlobalScenes = struct_PoserDataGlobalScenes# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 145

PoserDataAll = union_PoserDataAll# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 147

SurviveSensorActivations_s = struct_SurviveSensorActivations_s# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 26

survive_threaded_poser = struct_survive_threaded_poser# /home/justin/source/oss/libsurvive/include/libsurvive/poser.h: 163

SurviveSensorActivations_params = struct_SurviveSensorActivations_params# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 58

SurviveKalmanTracker = struct_SurviveKalmanTracker# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 211

BaseStationCal = struct_BaseStationCal# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 254

SurviveKalmanLighthouse = struct_SurviveKalmanLighthouse# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 277

config_group = struct_config_group# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 280

SurviveRecordingData = struct_SurviveRecordingData# /home/justin/source/oss/libsurvive/include/libsurvive/survive.h: 310

SurviveSimpleContext = struct_SurviveSimpleContext# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 11

SurviveSimpleObject = struct_SurviveSimpleObject# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 24

SurviveSimpleEvent = struct_SurviveSimpleEvent# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 238

SurviveSimpleButtonEvent = struct_SurviveSimpleButtonEvent# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 50

SurviveSimpleConfigEvent = struct_SurviveSimpleConfigEvent# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 56

SurviveSimplePoseUpdatedEvent = struct_SurviveSimplePoseUpdatedEvent# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 63

SurviveSimpleObjectEvent = struct_SurviveSimpleObjectEvent# /home/justin/source/oss/libsurvive/include/libsurvive/survive_api.h: 68

survive_reproject_model_t = struct_survive_reproject_model_t# /home/justin/source/oss/libsurvive/include/libsurvive/survive_reproject.h: 83

survive_optimizer_settings = struct_survive_optimizer_settings# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 85

survive_optimizer = struct_survive_optimizer# /home/justin/source/oss/libsurvive/include/libsurvive/survive_optimizer.h: 90

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

