using System;

namespace libsurvive
{
    using SurviveContextPtr = IntPtr;
    using SurviveObjectPtr = IntPtr;
    using SurvivePosePtr = IntPtr;

    public class SurviveContext : IDisposable
    {
        protected IntPtr ctx;
                
        public void Dispose()
        {
            cfunctions.survive_close(ctx);
            ctx = IntPtr.Zero;
        }

        public SurviveContext() : this(null) { }

        public SurviveContext(string[] args)
        {
            string[] newArgs = new string[args.Length + 1];
            newArgs[0] = System.Reflection.Assembly.GetEntryAssembly().FullName;
            Array.Copy(args, 0, newArgs, 1, args.Length);

            ctx = cfunctions.survive_init_internal(newArgs.Length, newArgs);

            cfunctions.survive_install_raw_pose_fn(ctx, PoseEvent);
            cfunctions.survive_install_light_fn(ctx, LightEvent);
            cfunctions.survive_install_lighthouse_pose_fn(ctx, LightHouseEvent);
            cfunctions.survive_install_angle_fn(ctx, AngleEvent);
            cfunctions.survive_install_button_fn(ctx, ButtonEvent);
            cfunctions.survive_install_htc_config_fn(ctx, HTCConfigEvent);
            cfunctions.survive_install_imu_fn(ctx, IMUEvent);
            cfunctions.survive_install_error_fn(ctx, ErrorEvent);
            cfunctions.survive_install_info_fn(ctx, InfoEvent);           
        }

        protected void InfoEvent(SurviveObjectPtr ctx, string fault)
        {
            Console.Out.WriteLine(fault);
        }

        protected void ErrorEvent(SurviveObjectPtr ctx, string fault)
        {
            Console.Error.WriteLine(fault);
        }

        protected void IMUEvent(SurviveObjectPtr so, int mask, double[] accelgyro, uint timecode, int id)
        {
            cfunctions.survive_default_imu_process(so, mask, accelgyro, timecode, id);
        }

        protected int HTCConfigEvent(SurviveObjectPtr so, string ct0conf, int len)
        {
            return cfunctions.survive_default_htc_config_process(so, ct0conf, len);
        }

        protected void ButtonEvent(SurviveObjectPtr so, byte eventType, byte buttonId, byte axis1Id, ushort axis1Val, byte axis2Id, ushort axis2Val)
        {
            cfunctions.survive_default_button_process(so, eventType, buttonId, axis1Id, axis1Val, axis2Id, axis2Val);
        }

        protected void AngleEvent(SurviveObjectPtr so, int sensor_id, int acode, uint timecode, double length, double angle, uint lh)
        {
            cfunctions.survive_default_angle_process(so, sensor_id, acode, timecode, length, angle, lh);
        }

        protected void LightHouseEvent(SurviveObjectPtr ctx, byte lighthouse, ref SurvivePose lighthouse_pose, ref SurvivePose object_pose)
        {
            cfunctions.survive_default_lighthouse_pose_process(ctx, lighthouse, ref lighthouse_pose, ref object_pose);
        }

        protected void LightEvent(SurviveObjectPtr so, int sensor_id, int acode, int timeinsweep, UInt32 timecode, UInt32 length, UInt32 lighthouse)
        {
            cfunctions.survive_default_light_process(so, sensor_id, acode, timeinsweep, timecode, length, lighthouse);
        }

        protected void PoseEvent(IntPtr so, byte lighthouse, ref SurvivePose pose)
        {
            cfunctions.survive_default_raw_pose_process(so, lighthouse, ref pose);
        }
        
        public int Poll()
        {
            return cfunctions.survive_poll(ctx);
        }
    }
}