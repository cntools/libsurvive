import ctypes
import types

import pysurvive.pysurvive_generated
from pysurvive.pysurvive_generated import *

LP_c_char = ctypes.POINTER(ctypes.c_char)
LP_LP_c_char = ctypes.POINTER(LP_c_char)

def add_to_lifetime(ctx, obj):
    if not hasattr(ctx, 'lifetime'):
        ctx.lifetime = []
    ctx.lifetime.append(obj)

def init(sargv = None, log_fn = None):
    if sargv == None:
        sargv = []
        
    argc = len(sargv)
    argv = (LP_c_char * (argc + 1))()
    for i, arg in enumerate(sargv):
        enc_arg = arg.encode('utf-8')
        argv[i] = ctypes.create_string_buffer(enc_arg)

    wrapped_log_fn = ctypes.cast(None, log_process_func)
    if log_fn is not None:
        wrapped_log_fn = log_process_func(log_fn)

    ctx = init_internal(argc, argv, None, wrapped_log_fn)
    add_to_lifetime(ctx, wrapped_log_fn)

    return ctx

def install_imu_fn(ctx, fn):
    def imu_func(ctx, mode, accelgyro, timecode, id):
        try:
            call_def = fn(ctx, mode, list(map(lambda x: accelgyro[x], range(9))), timecode, id)
            if call_def is None or call_def:
                default_imu_process(ctx, mode, accelgyro, timecode, id)
        except:
            ctx.contents.report_errorproc(ctx, SURVIVE_ERROR_GENERAL)
    imu_fn = imu_process_func(imu_func)

    add_to_lifetime(ctx, imu_fn)
    pysurvive_generated.install_imu_fn(ctx, imu_fn)

def install_generic_process(ctx, fn, default_fn, install_fn, fn_type):
    def wrapper_fn(*args):
        try:
            call_def = fn(*args)
            if call_def is None or call_def:
                default_fn(*args)
        except:
            ctx.contents.report_errorproc(ctx, SURVIVE_ERROR_GENERAL)

    fn_inst = fn_type(wrapper_fn)

    add_to_lifetime(ctx, fn_inst)
    install_fn(ctx, fn_inst)

def install_light_fn(ctx, fn):
    install_generic_process(ctx, fn, default_light_process, pysurvive_generated.install_light_fn, light_process_func)

def install_angle_fn(ctx, fn):
    install_generic_process(ctx, fn, survive_default_angle_process, pysurvive_generated.install_angle_fn, angle_process_func)

def configs(ctx, name, method=SC_GET, default=None):
    return pysurvive_generated.configs(ctx, name, method, default)

def configf(ctx, name, method=SC_GET, default=None):
    return pysurvive_generated.configf(ctx, name, method, default)

class SimpleObject:
    ptr = 0
    def __init__(self, ptr):
        self.ptr = ptr

    def Name(self):
        return simple_object_name(self.ptr)

    def Pose(self):
        pose = SurvivePose()
        time = simple_object_get_latest_pose(self.ptr, pose)
        return (pose, time)

class SimpleContext:
    ptr = 0
    objects = []

    def __init__(self, args):
        argc = len(args)
        argv = (LP_c_char * (argc + 1))()
        for i, arg in enumerate(args):
            enc_arg = arg.encode('utf-8')
            argv[i] = ctypes.create_string_buffer(enc_arg)
        self.ptr = simple_init(argc, argv)

        self.objects = []
        curr = simple_get_first_object(self.ptr);
        while curr:
            self.objects.append(SimpleObject(curr))
            curr = simple_get_next_object(self.ptr, curr);
        simple_start_thread(self.ptr)

    def Objects(self):
        return self.objects

    def Running(self):
        return simple_is_running(self.ptr)

    def NextUpdated(self):
        ptr = simple_get_next_updated(self.ptr)
        if ptr:
            return SimpleObject(ptr)
        return None

