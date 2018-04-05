import ctypes

_libsurvive = ctypes.CDLL('./_libsurvive.so')
LP_c_char = ctypes.POINTER(ctypes.c_char)
LP_LP_c_char = ctypes.POINTER(LP_c_char)

_libsurvive.survive_async_init.argtypes = (ctypes.c_int, LP_LP_c_char) # argc, argv
_libsurvive.survive_async_init.restype = ctypes.c_void_p

_libsurvive.survive_async_get_next_object.argtypes = [ctypes.c_void_p, ctypes.c_void_p]
_libsurvive.survive_async_get_next_object.restype = ctypes.c_void_p

_libsurvive.survive_async_get_first_object.argtypes = [ctypes.c_void_p]
_libsurvive.survive_async_get_first_object.restype = ctypes.c_void_p

_libsurvive.survive_async_get_next_updated.argtypes = [ctypes.c_void_p]
_libsurvive.survive_async_get_next_updated.restype = ctypes.c_void_p

_libsurvive.survive_async_object_name.argtypes = [ ctypes.c_void_p ]
_libsurvive.survive_async_object_name.restype = ctypes.c_char_p

_libsurvive.survive_async_is_running.argtypes = [ctypes.c_void_p]
_libsurvive.survive_async_is_running.restype = ctypes.c_bool

_libsurvive.survive_async_start_thread.argtypes = [ctypes.c_void_p]
_libsurvive.survive_async_start_thread.restype = None

class SurvivePose(ctypes.Structure):
    _fields_ = [
        ('Pos', ctypes.c_double * 3),
        ('Rot', ctypes.c_double * 4)
    ]
    def __repr__(self):
        return '[{0} {1} {2}], [{3} {4} {5} {6}]'.format(self.Pos[0],self.Pos[1],self.Pos[2],self.Rot[0],self.Rot[1],self.Rot[2],self.Rot[3])

    
_libsurvive.survive_async_object_get_latest_pose.argtypes = [ctypes.c_void_p, ctypes.POINTER(SurvivePose)]
_libsurvive.survive_async_object_get_latest_pose.restype = ctypes.c_uint

class AsyncObject:
    ptr = 0
    def __init__(self, ptr):
        self.ptr = ptr
        
    def Name(self):
        return _libsurvive.survive_async_object_name(self.ptr)

    def Pose(self):
        pose = SurvivePose()
        time = _libsurvive.survive_async_object_get_latest_pose(self.ptr, pose)
        return (pose, time)
    
class AsyncContext:
    ptr = 0
    objects = []
    
    def __init__(self, args):
        argc = len(args)
        argv = (LP_c_char * (argc + 1))()
        for i, arg in enumerate(args):
            enc_arg = arg.encode('utf-8')
            argv[i] = ctypes.create_string_buffer(enc_arg)
        self.ptr = _libsurvive.survive_async_init(argc, argv)
        
        self.objects = []
        curr = _libsurvive.survive_async_get_first_object(self.ptr);
        while curr:
            self.objects.append(AsyncObject(curr))
            curr = _libsurvive.survive_async_get_next_object(self.ptr, curr);
        _libsurvive.survive_async_start_thread(self.ptr)
            
    def Objects(self):
        return self.objects

    def Running(self):
        return _libsurvive.survive_async_is_running(self.ptr)

    def NextUpdated(self):
        ptr = _libsurvive.survive_async_get_next_updated(self.ptr)
        if ptr:
            return AsyncObject(ptr)
        return None

