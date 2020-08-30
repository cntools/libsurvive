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

		public void Dispose() { Close(); }

		public void Close() {
			if (ctx != IntPtr.Zero) {
				Cfunctions.Survive_close(ctx);
			}
			ctx = IntPtr.Zero;
		}

		/**
         * We need to keep these delegates around for the lifespan of this object. 
         * if we just pass the functions in, it creates temporary delegates that 
         * disappear if we don't keep managed ref around
         */
        light_process_func light_Process_Func;
		pose_process_func raw_Pose_Func;
		lighthouse_pose_process_func lighthouse_pose_process_func;
		angle_process_func angle_Process_Func;
        button_process_func button_Process_Func;
		config_func Config_Func;
		imu_process_func imu_Process_Func;
		log_process_func log_func;

		public SurviveContext(string[] args)
        {
            Init(args);
        }

        internal void Init(string[] args)
        {
            string[] newArgs = new string[args.Length + 1];
			newArgs[0] = "program";
			if (System.Reflection.Assembly.GetEntryAssembly() != null) {
				newArgs[0] = System.Reflection.Assembly.GetEntryAssembly().FullName;
			}

			Array.Copy(args, 0, newArgs, 1, args.Length);

			ctx = Cfunctions.Survive_init_internal(newArgs.Length, newArgs, IntPtr.Zero, log_func);

			if (ctx == IntPtr.Zero)
            {
                throw new Exception("There was a problem initializing the lib!");
            }

            light_Process_Func = LightEvent;
            raw_Pose_Func = PoseEvent;
			lighthouse_pose_process_func = LightHouseEvent;
			angle_Process_Func = AngleEvent;
            button_Process_Func = ButtonEvent;
			Config_Func = ConfigEvent;
			imu_Process_Func = IMUEvent;
			log_func = InfoEvent;

			Cfunctions.Survive_install_pose_fn(ctx, raw_Pose_Func);
			Cfunctions.Survive_install_light_fn(ctx, light_Process_Func);
			Cfunctions.Survive_install_lighthouse_pose_fn(ctx, lighthouse_pose_process_func);
			Cfunctions.Survive_install_angle_fn(ctx, angle_Process_Func);
            Cfunctions.Survive_install_button_fn(ctx, button_Process_Func);
			Cfunctions.Survive_install_config_fn(ctx, Config_Func);
			Cfunctions.Survive_install_imu_fn(ctx, imu_Process_Func);
        }


        //Dictionary<int,PoseUpdate> PoseUpdateCallbacks = new Dictionary<int, PoseUpdate>();
        PoseUpdate poseUpdate;

        public void AddPoseUpdateCallback(PoseUpdate update, int objectID)
        {
            //PoseUpdateCallbacks[objectID] += update;
            poseUpdate += update;
        }

		virtual protected void InfoEvent(SurviveObjectPtr ctx, UInt32 loglevl, string fault) {
			Console.Out.WriteLine(fault);
		}

		virtual protected void ErrorEvent(SurviveObjectPtr ctx, string fault)
        {
            Console.Error.WriteLine(fault);
        }

		virtual protected void IMUEvent(SurviveObjectPtr so, int mask, SurviveIMUData accelgyro, uint timecode,
										int id) {
			Cfunctions.Survive_default_imu_process(so, mask, accelgyro, timecode, id);
		}

		virtual protected int ConfigEvent(SurviveObjectPtr so, string ct0conf, int len) {
			return Cfunctions.Survive_default_config_process(so, ct0conf, len);
		}

		virtual protected void ButtonEvent(SurviveObjectPtr so, byte eventType, byte buttonId, byte axis1Id, ushort axis1Val, byte axis2Id, ushort axis2Val)
        {
            Cfunctions.Survive_default_button_process(so, eventType, buttonId, axis1Id, axis1Val, axis2Id, axis2Val);
        }

        virtual protected void AngleEvent(SurviveObjectPtr so, int sensor_id, int acode, uint timecode, double length, double angle, uint lh)
        {
            Cfunctions.Survive_default_angle_process(so, sensor_id, acode, timecode, length, angle, lh);
        }

		protected void LightHouseEvent(SurviveObjectPtr ctx, byte lighthouse, SurvivePose lighthouse_pose) {
			Cfunctions.Survive_default_lighthouse_pose_process(ctx, lighthouse, lighthouse_pose);
		}

		virtual protected void LightEvent(SurviveObjectPtr so, int sensor_id, int acode, int timeinsweep, UInt32 timecode, UInt32 length, UInt32 lighthouse)
        {
            Cfunctions.Survive_default_light_process(so, sensor_id, acode, timeinsweep, timecode, length, lighthouse);

        }

		virtual protected void PoseEvent(SurviveObjectPtr so, UInt32 timecode, SurvivePose pose) {
			Cfunctions.Survive_default_pose_process(so, timecode, pose);
		}

		public int Poll()
        {
            return Cfunctions.Survive_poll(ctx);
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
