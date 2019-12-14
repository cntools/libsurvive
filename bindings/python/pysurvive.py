import ctypes

import pysurvive_generated
from pysurvive_generated import *

LP_c_char = ctypes.POINTER(ctypes.c_char)
LP_LP_c_char = ctypes.POINTER(LP_c_char)

def survive_init(sargv):
    argc = len(sargv)
    argv = (LP_c_char * (argc + 1))()
    for i, arg in enumerate(sargv):
        enc_arg = arg.encode('utf-8')
        argv[i] = ctypes.create_string_buffer(enc_arg)
    return init_internal(argc, argv, None, ctypes.cast(None, log_process_func))

imu_fn = None
def install_imu_fn(ctx, fn):
    global imu_fn

    def imu_func(ctx, mode, accelgyro, timecode, id):
        fn(ctx, mode, list(map(lambda x: accelgyro[x], range(9))), timecode, id)
    imu_fn = imu_process_func(imu_func)
    pysurvive_generated.install_imu_fn(ctx, imu_fn)


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

