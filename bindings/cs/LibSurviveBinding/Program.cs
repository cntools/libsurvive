using System;
using System.Runtime.InteropServices;

namespace LibSurVive
{
    class Program
    {
        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall)]
        static extern IntPtr survive_init_internal(int argc, string[] args);

        public delegate void raw_pose_func(IntPtr so, byte lighthouse, IntPtr pose);
        public delegate void lighthouse_pose_func(IntPtr ctx, byte lighthouse, IntPtr lighthouse_pose, IntPtr object_pose);
        public delegate void light_process_func( IntPtr so, int sensor_id, int acode, int timeinsweep, 
        UInt32 timecode, UInt32 length, UInt32 lighthouse);                                                

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall)]
        static extern void survive_install_raw_pose_fn(IntPtr ctx, raw_pose_func fbp);
        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall)]
        static extern void survive_install_lighthouse_pose_fn(IntPtr ctx, lighthouse_pose_func fbp);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall)]
        static extern void survive_install_light_fn(IntPtr ctx, light_process_func fbp);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall)]
        static extern int survive_startup(IntPtr ctx);
        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall)]
        static extern void survive_cal_install(IntPtr ctx);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall)]
        static extern int survive_poll(IntPtr ctx);

        static void Main(string[] args)
        {
            IntPtr context = survive_init_internal(args.Length, args);

            survive_install_lighthouse_pose_fn(context, LighthousPos);
            survive_install_raw_pose_fn(context, PositionUpdate);
            survive_install_light_fn(context, LightUpdate); 

            survive_startup(context);
            survive_cal_install(context);

            while(survive_poll(context) == 0) {}

        }
        
        public static void LightUpdate( IntPtr so, int sensor_id, int acode, int timeinsweep, 
        UInt32 timecode, UInt32 length, UInt32 lighthouse) {
            Console.WriteLine(timeinsweep);
        }                                                

        public static void PositionUpdate(IntPtr so, byte lighthouse, IntPtr pose)
        {
            Console.WriteLine(pose);
        }

        public static void LighthousPos(IntPtr ctx, byte lighthouse, IntPtr lighthouse_pose, IntPtr object_pose)
        {

        }
    }
}
