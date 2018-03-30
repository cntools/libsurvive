using System;
using System.Collections.Generic;

namespace libsurvive
{
    using SurviveContextPtr = IntPtr;
    using SurviveObjectPtr = IntPtr;
    using SurvivePosePtr = IntPtr;

    public class SurviveContext : IDisposable
    {
        public delegate void PoseUpdate(int ObjectID, Vector3 pos);


        protected IntPtr ctx;
                
        public void Dispose()
        {
            cfunctions.Survive_close(ctx);
            ctx = IntPtr.Zero;
        }

        /**
         * We need to keep these delegates around for the lifespan of this object. 
         * if we just pass the functions in, it creates temporary delegates that 
         * disappear if we don't keep managed ref around
         */
        light_process_func light_Process_Func;
        raw_pose_func raw_Pose_Func;
        lighthouse_pose_func lighthouse_Pose_Func;
        angle_process_func angle_Process_Func;
        button_process_func button_Process_Func;
        htc_config_func htc_Config_Func;
        imu_process_func imu_Process_Func;
        text_feedback_func error_func;
        text_feedback_func info_func;

        public SurviveContext(string[] args)
        {
            Init(args);
        }

        public SurviveContext(string replaydata)
        {
            string[] vs = new[] { "--playback", "lightcap-reformat.log"};
            Init(vs);
        }

        internal void Init(string[] args)
        {
            string[] newArgs = new string[args.Length + 1];
            newArgs[0] = System.Reflection.Assembly.GetEntryAssembly().FullName;
            Array.Copy(args, 0, newArgs, 1, args.Length);

            ctx = cfunctions.Survive_init_internal(newArgs.Length, newArgs);

            if (ctx == IntPtr.Zero)
            {
                throw new Exception("There was a problem initializing the lib!");
            }

            light_Process_Func = LightEvent;
            raw_Pose_Func = PoseEvent;
            lighthouse_Pose_Func = LightHouseEvent;
            angle_Process_Func = AngleEvent;
            button_Process_Func = ButtonEvent;
            htc_Config_Func = HTCConfigEvent;
            imu_Process_Func = IMUEvent;
            error_func = ErrorEvent;
            info_func = InfoEvent;

            cfunctions.Survive_install_raw_pose_fn(ctx, raw_Pose_Func);
            cfunctions.Survive_install_light_fn(ctx, light_Process_Func);
            cfunctions.Survive_install_lighthouse_pose_fn(ctx, lighthouse_Pose_Func);
            cfunctions.Survive_install_angle_fn(ctx, angle_Process_Func);
            cfunctions.Survive_install_button_fn(ctx, button_Process_Func);
            cfunctions.Survive_install_htc_config_fn(ctx, htc_Config_Func);
            cfunctions.Survive_install_imu_fn(ctx, imu_Process_Func);
            cfunctions.Survive_install_error_fn(ctx, error_func);
            cfunctions.Survive_install_info_fn(ctx, info_func);
        }


        //Dictionary<int,PoseUpdate> PoseUpdateCallbacks = new Dictionary<int, PoseUpdate>();
        PoseUpdate poseUpdate;

        public void AddPoseUpdateCallback(PoseUpdate update, int objectID)
        {
            //PoseUpdateCallbacks[objectID] += update;
            poseUpdate += update;
        }



        virtual protected void InfoEvent(SurviveObjectPtr ctx, string fault)
        {
            Console.Out.WriteLine(fault);
        }

        virtual protected void ErrorEvent(SurviveObjectPtr ctx, string fault)
        {
            Console.Error.WriteLine(fault);
        }

        virtual protected void IMUEvent(SurviveObjectPtr so, int mask, double[] accelgyro, uint timecode, int id)
        {
            cfunctions.Survive_default_imu_process(so, mask, accelgyro, timecode, id);
        }

        virtual protected int HTCConfigEvent(SurviveObjectPtr so, string ct0conf, int len)
        {
            return cfunctions.Survive_default_htc_config_process(so, ct0conf, len);
        }

        virtual protected void ButtonEvent(SurviveObjectPtr so, byte eventType, byte buttonId, byte axis1Id, ushort axis1Val, byte axis2Id, ushort axis2Val)
        {
            cfunctions.Survive_default_button_process(so, eventType, buttonId, axis1Id, axis1Val, axis2Id, axis2Val);
        }

        virtual protected void AngleEvent(SurviveObjectPtr so, int sensor_id, int acode, uint timecode, double length, double angle, uint lh)
        {
            cfunctions.Survive_default_angle_process(so, sensor_id, acode, timecode, length, angle, lh);
        }

        protected void LightHouseEvent(SurviveObjectPtr ctx, byte lighthouse, SurvivePose lighthouse_pose, SurvivePose object_pose)
        {
            cfunctions.Survive_default_lighthouse_pose_process(ctx, lighthouse, lighthouse_pose, object_pose);
        }

        virtual protected void LightEvent(SurviveObjectPtr so, int sensor_id, int acode, int timeinsweep, UInt32 timecode, UInt32 length, UInt32 lighthouse)
        {
            cfunctions.Survive_default_light_process(so, sensor_id, acode, timeinsweep, timecode, length, lighthouse);

            //Console.WriteLine("LightEvent");
        }

        virtual protected void PoseEvent(SurviveObjectPtr so, byte lighthouse, SurvivePose pose)
        {
            cfunctions.Survive_default_raw_pose_process(so, lighthouse, pose);

            Console.WriteLine("PoseEvent");
        }
        
        public int Poll()
        {
            return cfunctions.Survive_poll(ctx);
        }
    }

    public struct Vector3
    {
        public float x;
        public float y;
        public float z;
    }

    public struct Qutarnion
    {
        public float x;
        public float y;
        public float z;
        public float w;
    }
}