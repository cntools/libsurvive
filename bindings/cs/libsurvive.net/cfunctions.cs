using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Text;

namespace libsurvive
{
    using SurviveContextPtr = IntPtr;
    using SurviveObjectPtr = IntPtr;
    using SurvivePosePtr = IntPtr;

	using SurviveSimpleContextPtr = IntPtr;
	using SurviveSimpleObjectPtr = IntPtr;

	[StructLayout(LayoutKind.Sequential)]
    public class SurvivePose
    {
		public SurvivePose() {
			Pos = new double[3];
			Rot = new double[4];
			Rot[0] = 1;
		}
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public double[] Pos; // Position in the form xyz
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
		public double[] Rot; // Quaternion in the form wxyz

		public override string ToString() { return string.Join(",", Pos) + ", " + string.Join(",", Rot); }
	}

	[StructLayout(LayoutKind.Sequential)]
	public class SurviveIMUData {
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
		public double[] Accel; // Position in the form xyz
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
		public double[] Gyro; // Position in the form xyz
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
		public double[] Mag; // Position in the form xyz
	}

	[StructLayout(LayoutKind.Sequential)]
	public class SurviveSimpleButtonEvent {
		public SurviveSimpleObjectPtr obj;
		public UInt32 event_type;
		public UInt32 button_id;

		public Byte axis_count;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
		public UInt32[] axis_ids;
		[MarshalAs(UnmanagedType.ByValArray, SizeConst = 8)]
		public float[] axis_values;
	}

	class Cfunctions
    {
        //#pragma warning disable IDE1006 // Naming Styles
		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_init_internal")]
		public static extern SurviveContextPtr Survive_init_internal(int argc, string[] args, IntPtr user,
																	 log_process_func fbp);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_close")]
        public static extern SurviveContextPtr Survive_close(SurviveContextPtr ctx);

        [DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_poll")]
        public static extern int Survive_poll(SurviveContextPtr ctx);

        [DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_startup")]
        public static extern int Survive_startup(SurviveContextPtr ctx);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl,
				   EntryPoint = "survive_install_config_fn")]
		public static extern void Survive_install_config_fn(SurviveContextPtr ctx, config_func fbp);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_install_log_fn")]
		public static extern void Survive_install_log_fn(SurviveContextPtr ctx, log_process_func fbp);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_install_imu_fn")]
        public static extern void Survive_install_imu_fn(SurviveContextPtr ctx, imu_process_func fbp);

        [DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_install_angle_fn")]
        public static extern void Survive_install_angle_fn(SurviveContextPtr ctx, angle_process_func fbp);

        [DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_install_button_fn")]
        public static extern void Survive_install_button_fn(SurviveContextPtr ctx, button_process_func fbp);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_install_pose_fn")]
		public static extern void Survive_install_pose_fn(SurviveContextPtr ctx, pose_process_func fbp);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl,
				   EntryPoint = "survive_install_lighthouse_pose_fn")]
		public static extern void Survive_install_lighthouse_pose_fn(SurviveContextPtr ctx,
																	 lighthouse_pose_process_func fbp);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_install_light_fn")]
        public static extern void Survive_install_light_fn(SurviveContextPtr ctx, light_process_func fbp);

        [DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_cal_install")]
        public static extern void Survive_cal_install(SurviveContextPtr ctx);

        [DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_default_light_process")]
        public static extern void Survive_default_light_process(SurviveObjectPtr so, int sensor_id, int acode, int timeinsweep, UInt32 timecode, UInt32 length, UInt32 lh);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl,
				   EntryPoint = "survive_default_imu_process")]
		public static extern void Survive_default_imu_process(SurviveObjectPtr so, int mode, SurviveIMUData accelgyro,
															  UInt32 timecode, int id);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_default_angle_process")]
        public static extern void Survive_default_angle_process(SurviveObjectPtr so, int sensor_id, int acode, UInt32 timecode, double length, double angle, UInt32 lh);

        [DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_default_button_process")]
        public static extern void Survive_default_button_process(SurviveObjectPtr so, byte eventType, byte buttonId, byte axis1Id, UInt16 axis1Val, byte axis2Id, UInt16 axis2Val);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl,
				   EntryPoint = "survive_default_pose_process")]
		public static extern void Survive_default_pose_process(SurviveObjectPtr so, UInt32 timecode, SurvivePose pose);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl,
				   EntryPoint = "survive_default_lighthouse_pose_process")]
		public static extern void Survive_default_lighthouse_pose_process(SurviveContextPtr ctx, byte lighthouse,
																		  SurvivePose lh_pose);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl,
				   EntryPoint = "survive_default_config_process")]
		public static extern int Survive_default_config_process(SurviveObjectPtr so, string ct0conf, int len);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_object_codename")]
        public static extern string Survive_object_codename(SurviveObjectPtr so);

        [DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_object_drivername")]
        public static extern char Survive_object_drivername(SurviveObjectPtr so);

        [DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_object_charge")]
        public static extern byte Survive_object_charge(SurviveObjectPtr so);

        [DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_object_charging")]
        public static extern bool Survive_object_charging(SurviveObjectPtr so);

        [DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_object_pose")]
        public static extern IntPtr Survive_object_pose(SurviveObjectPtr so);


        [DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_object_sensor_locations")]
        public static extern double[] Survive_object_sensor_locations(SurviveObjectPtr so);

        [DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_object_sensor_normals")]
        public static extern double[] Survive_object_sensor_normals(SurviveObjectPtr so);


        [DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_get_so_by_name")]
        public static extern IntPtr Survive_get_so_by_name(IntPtr ctx, string name);

        //#pragma warning restore IDE1006 // Naming Styles
    }

	class Cfunctions_api {
		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_simple_init")]
		public static extern SurviveSimpleContextPtr survive_simple_init(int argc, string[] args);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl,
				   EntryPoint = "survive_simple_init_with_logger")]
		public static extern SurviveSimpleContextPtr survive_simple_init_with_logger(int argc, string[] args,
																					 SurviveSimpleLogFn fn);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl,
				   EntryPoint = "survive_simple_start_thread")]
		public static extern void survive_simple_start_thread(SurviveSimpleContextPtr actx);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl,
				   EntryPoint = "survive_simple_get_first_object")]
		public static extern SurviveSimpleObjectPtr survive_simple_get_first_object(SurviveSimpleContextPtr actx);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl,
				   EntryPoint = "survive_simple_get_next_object")]
		public static extern SurviveSimpleObjectPtr survive_simple_get_next_object(SurviveSimpleContextPtr actx,
																				   SurviveSimpleObjectPtr aso);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_simple_object_name",
				   CharSet = CharSet.Ansi)]
		public static extern IntPtr survive_simple_object_name(SurviveSimpleObjectPtr aso);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_simple_serial_number",
				   CharSet = CharSet.Ansi)]
		public static extern IntPtr survive_simple_serial_number(SurviveSimpleObjectPtr aso);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl,
				   EntryPoint = "survive_simple_wait_for_update")]
		public static extern bool survive_simple_wait_for_update(SurviveSimpleContextPtr actx);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl,
				   EntryPoint = "survive_simple_get_next_updated")]
		public static extern SurviveSimpleObjectPtr survive_simple_get_next_updated(SurviveSimpleContextPtr actx);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl,
				   EntryPoint = "survive_simple_object_get_latest_pose")]
		public static extern double survive_simple_object_get_latest_pose(SurviveSimpleObjectPtr aso, IntPtr pose);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl,
				   EntryPoint = "survive_simple_next_event")]
		public static extern UInt32 survive_simple_next_event(SurviveSimpleObjectPtr aso, IntPtr evt);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl,
				   EntryPoint = "survive_simple_get_button_event")]
		public static extern SurviveSimpleButtonEvent survive_simple_get_button_event(IntPtr evt);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl, EntryPoint = "survive_simple_close")]
		public static extern void survive_simple_close(SurviveSimpleContextPtr actx);

		[DllImport("libsurvive", CallingConvention = CallingConvention.Cdecl,
				   EntryPoint = "survive_simple_is_running")]
		public static extern bool survive_simple_is_running(SurviveSimpleContextPtr actx);
	}
	[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
	public delegate void SurviveSimpleLogFn(SurviveSimpleContextPtr actx, UInt32 logLevel, string msg);

	[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
	public delegate int config_func(SurviveObjectPtr so, string ct0conf, int len);

	[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
	public delegate void log_process_func(SurviveContextPtr ctx, UInt32 loglevel, string fault);

	[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    public delegate void light_process_func(SurviveObjectPtr so, int sensor_id, int acode, int timeinsweep, UInt32 timecode, UInt32 length, UInt32 lighthouse);

	[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
	public delegate void imu_process_func(SurviveObjectPtr so, int mask, SurviveIMUData accelgyro, UInt32 timecode,
										  int id);

	[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    public delegate void angle_process_func(SurviveObjectPtr so, int sensor_id, int acode, UInt32 timecode, double length, double angle, UInt32 lh);

    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    public delegate void button_process_func(SurviveObjectPtr so, byte eventType, byte buttonId, byte axis1Id, UInt16 axis1Val, byte axis2Id, UInt16 axis2Val);

	[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
	public delegate void pose_process_func(SurviveObjectPtr so, UInt32 timecode, SurvivePose pose);

	[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
	public delegate void lighthouse_pose_process_func(SurviveContextPtr ctx, byte lighthouse,
													  SurvivePose lighthouse_pose);
}
