using System;
using System.Runtime.InteropServices;

namespace LibSurVive
{
    class Program
    {
        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall)]
        static extern IntPtr survive_init_internal(int argc, char[] args);

        public delegate void raw_pose_func(IntPtr so, byte lighthouse, IntPtr pose);
        public delegate void lighthouse_pose_func(IntPtr ctx, byte lighthouse, IntPtr lighthouse_pose, IntPtr object_pose);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall)]
        static extern void survive_install_raw_pose_fn(IntPtr ctx, raw_pose_func fbp);
        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall)]
        static extern void survive_install_lighthouse_pose_fn(IntPtr ctx, lighthouse_pose_func fbp);


        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall)]
        static extern int survive_startup(IntPtr ctx);
        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall)]
        static extern void survive_cal_install(IntPtr ctx);

        public static lighthouse_pose_func lighthouse_Pose_Func { get; private set; }
        public static raw_pose_func raw_Pose_Func { get; private set; }

        static void Main(string[] args)
        {
            IntPtr context = survive_init_internal(0, null);

            lighthouse_Pose_Func = LighthousPos;
            survive_install_lighthouse_pose_fn(context, lighthouse_Pose_Func);
            raw_Pose_Func = PositionUpdate;
            survive_install_raw_pose_fn(context, raw_Pose_Func);

            try
            {
                int a = survive_startup(context);
                //survive_cal_install(context);
            }
            catch (Exception)
            {

                throw;
            }

            bool running = true;



            Console.WriteLine("Hello World!");

            while (running)
            {
                Console.ReadLine();
            }

        }

        public static void PositionUpdate(IntPtr so, byte lighthouse, IntPtr pose)
        {
            //Console.WriteLine(pose);
        }

        public static void LighthousPos(IntPtr ctx, byte lighthouse, IntPtr lighthouse_pose, IntPtr object_pose)
        {

        }
    }
}
