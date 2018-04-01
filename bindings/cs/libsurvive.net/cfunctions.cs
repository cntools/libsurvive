using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;

namespace libsurvive
{
    using SurviveContextPtr = IntPtr;
    using SurviveObjectPtr = IntPtr;
    using SurvivePosePtr = IntPtr;

    [StructLayout(LayoutKind.Sequential)]
    public class SurvivePose
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] Pos; // Position in the form xyz
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public double[] Rot; // Quaternion in the form wxyz        
    }

    class cfunctions
    {
        //#pragma warning disable IDE1006 // Naming Styles
        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_init_internal")]
        public static extern SurviveContextPtr Survive_init_internal(int argc, string[] args);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_close")]
        public static extern SurviveContextPtr Survive_close(SurviveContextPtr ctx);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_poll")]
        public static extern int Survive_poll(SurviveContextPtr ctx);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_startup")]
        public static extern int Survive_startup(SurviveContextPtr ctx);


        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_install_htc_config_fn")]
        public static extern void Survive_install_htc_config_fn(SurviveContextPtr ctx, htc_config_func fbp);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_install_info_fn")]
        public static extern void Survive_install_info_fn(SurviveContextPtr ctx, text_feedback_func fbp);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_install_error_fn")]
        public static extern void Survive_install_error_fn(SurviveContextPtr ctx, text_feedback_func fbp);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_install_imu_fn")]
        public static extern void Survive_install_imu_fn(SurviveContextPtr ctx, imu_process_func fbp);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_install_angle_fn")]
        public static extern void Survive_install_angle_fn(SurviveContextPtr ctx, angle_process_func fbp);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_install_button_fn")]
        public static extern void Survive_install_button_fn(SurviveContextPtr ctx, button_process_func fbp);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_install_raw_pose_fn")]
        public static extern void Survive_install_raw_pose_fn(SurviveContextPtr ctx, raw_pose_func fbp);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_install_lighthouse_pose_fn")]
        public static extern void Survive_install_lighthouse_pose_fn(SurviveContextPtr ctx, lighthouse_pose_func fbp);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_install_light_fn")]
        public static extern void Survive_install_light_fn(SurviveContextPtr ctx, light_process_func fbp);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_cal_install")]
        public static extern void Survive_cal_install(SurviveContextPtr ctx);


        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_default_light_process")]
        public static extern void Survive_default_light_process(SurviveObjectPtr so, int sensor_id, int acode, int timeinsweep, UInt32 timecode, UInt32 length, UInt32 lh);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_default_imu_process")]
        public static extern void Survive_default_imu_process(SurviveObjectPtr so, int mode, double[] accelgyro, UInt32 timecode, int id);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_default_angle_process")]
        public static extern void Survive_default_angle_process(SurviveObjectPtr so, int sensor_id, int acode, UInt32 timecode, double length, double angle, UInt32 lh);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_default_button_process")]
        public static extern void Survive_default_button_process(SurviveObjectPtr so, byte eventType, byte buttonId, byte axis1Id, UInt16 axis1Val, byte axis2Id, UInt16 axis2Val);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_default_raw_pose_process")]
        public static extern void Survive_default_raw_pose_process(SurviveObjectPtr so, byte lighthouse, SurvivePose pose);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_default_lighthouse_pose_process")]
        public static extern void Survive_default_lighthouse_pose_process(SurviveContextPtr ctx, byte lighthouse, SurvivePose lh_pose, SurvivePose obj_pose);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_default_htc_config_process")]
        public static extern int Survive_default_htc_config_process(SurviveObjectPtr so, string ct0conf, int len);


        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_object_codename")]
        public static extern string Survive_object_codename(SurviveObjectPtr so);


        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_object_drivername")]
        public static extern char Survive_object_drivername(SurviveObjectPtr so);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_object_charge")]
        public static extern byte Survive_object_charge(SurviveObjectPtr so);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_object_charging")]
        public static extern bool Survive_object_charging(SurviveObjectPtr so);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_object_pose")]
        public static extern SurvivePose Survive_object_pose(SurviveObjectPtr so);


        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_object_sensor_locations")]
        public static extern double[] Survive_object_sensor_locations(SurviveObjectPtr so);

        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_object_sensor_normals")]
        public static extern double[] Survive_object_sensor_normals(SurviveObjectPtr so);



        [DllImport("libsurvive", CallingConvention = CallingConvention.StdCall, EntryPoint = "survive_get_so_by_name")]
        public static extern IntPtr Survive_get_so_by_name(IntPtr ctx, string name);

        //#pragma warning restore IDE1006 // Naming Styles
    }


    [UnmanagedFunctionPointer(CallingConvention.StdCall)]
    public delegate int htc_config_func(SurviveObjectPtr so, string ct0conf, int len);

    [UnmanagedFunctionPointer(CallingConvention.StdCall)]
    public delegate void text_feedback_func(SurviveContextPtr ctx, string fault);

    [UnmanagedFunctionPointer(CallingConvention.StdCall)]
    public delegate void light_process_func(SurviveObjectPtr so, int sensor_id, int acode, int timeinsweep, UInt32 timecode, UInt32 length, UInt32 lighthouse);

    [UnmanagedFunctionPointer(CallingConvention.StdCall)]
    public delegate void imu_process_func(SurviveObjectPtr so, int mask, double[] accelgyro, UInt32 timecode, int id);

    [UnmanagedFunctionPointer(CallingConvention.StdCall)]
    public delegate void angle_process_func(SurviveObjectPtr so, int sensor_id, int acode, UInt32 timecode, double length, double angle, UInt32 lh);

    [UnmanagedFunctionPointer(CallingConvention.StdCall)]
    public delegate void button_process_func(SurviveObjectPtr so, byte eventType, byte buttonId, byte axis1Id, UInt16 axis1Val, byte axis2Id, UInt16 axis2Val);

    [UnmanagedFunctionPointer(CallingConvention.StdCall)]
    public delegate void raw_pose_func(SurviveObjectPtr so, byte lighthouse, SurvivePose pose);


    [UnmanagedFunctionPointer(CallingConvention.StdCall)]
    public delegate void lighthouse_pose_func(SurviveContextPtr ctx, byte lighthouse, SurvivePose lighthouse_pose, SurvivePose object_pose);


}
