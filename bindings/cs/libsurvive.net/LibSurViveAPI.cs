using System.Collections;
using System.Collections.Generic;

using libsurvive;
using System;
using System.Threading;
using System.Runtime.InteropServices;

public class LibSurViveAPI
{
    private static LibSurViveAPI _instance;
    public static LibSurViveAPI Instance
    {
        get
        {
            if (_instance == null)
            {
                _instance = new LibSurViveAPI();
            }

            return _instance;
        }
    }

    bool running = true;
    Thread internalPollTread;

    public IntPtr context;

    light_process_func light_Process_Func;
    raw_pose_func raw_Pose_Func;
    lighthouse_pose_func lighthouse_Pose_Func;
    angle_process_func angle_Process_Func;
    button_process_func button_Process_Func;
    htc_config_func htc_Config_Func;
    imu_process_func imu_Process_Func;
    text_feedback_func error_func;
    text_feedback_func info_func;

    public delegate void Log(string message);
    Log LogInfo = delegate { };

    public delegate void ErrorLog(string message);
    ErrorLog LogError = delegate { };

    LibSurViveAPI()
    {
        CreateContext();

        CreateTread();
    }

    ~LibSurViveAPI()
    {
        running = false;
    }

    private void CreateTread()
    {
        internalPollTread = new Thread(InternalPoll);
        internalPollTread.Start();
    }

    void InternalPoll()
    {
        while (running)
        {
            int code = Cfunctions.Survive_poll(context);

            if (code != 0)
            {
                running = false;
            }
        }
    }

    internal void CreateContext()
    {
        LogInfo("Start Init");

        SetupConfigs configs = new SetupConfigs
        {
            configFile = "survive_conf.json"
        };

        string[] args = CreateStartParameters(configs);

        //string[] vs = new[] { "--playback", "P:/c/libsurvive-data/lightcap-reformat/lightcap-reformat.log", "--disambiguator", "StateBased", "--calibrate" };

        context = Cfunctions.Survive_init_internal(args.Length, args);

        if (context == IntPtr.Zero)
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

        Cfunctions.Survive_install_raw_pose_fn(context, raw_Pose_Func);
        Cfunctions.Survive_install_light_fn(context, light_Process_Func);
        Cfunctions.Survive_install_lighthouse_pose_fn(context, lighthouse_Pose_Func);
        Cfunctions.Survive_install_angle_fn(context, angle_Process_Func);
        Cfunctions.Survive_install_button_fn(context, button_Process_Func);
        Cfunctions.Survive_install_htc_config_fn(context, htc_Config_Func);
        Cfunctions.Survive_install_imu_fn(context, imu_Process_Func);
        Cfunctions.Survive_install_error_fn(context, error_func);
        Cfunctions.Survive_install_info_fn(context, info_func);

        LogInfo("Finished Init");

        LogInfo("Start Startup");

        int a = 0;
        try
        {
            a = Cfunctions.Survive_startup(context);
        }
        catch (Exception)
        {
            throw;
        }

        if (a != 0)
        {
            throw new Exception("Error in startup");
        }

        LogInfo("Finished Startup");

    }


    static public string[] CreateStartParameters(SetupConfigs configs)
    {
        List<string> args = new List<string>
        {
            "unity"
        };

        if (configs.playbackFile != "" && configs.playbackFile != null)
        {
            args.AddRange(new[] { "--playback", configs.playbackFile });
            args.AddRange(new[] { "--playback-factor", configs.playbackFactor.ToString() });
        }

        if(configs.disambiguator != Disambiguator.Default)
        args.AddRange(new[] { "--disambiguator", Enum.GetName(typeof(Disambiguator), configs.disambiguator) });

        if (configs.calibrate != BoolConfig.Default)
            args.Add(configs.calibrate == BoolConfig.Yes ? "--calibrate" : "--no-calibrate");

        if (configs.configFile != "")
            args.AddRange(new[] { "-c", configs.configFile });

        //args.AddRange(new[] { "--disambiguator", Enum.GetName(typeof(Poser), disambiguator) });



        return args.ToArray();
    }



    virtual protected void InfoEvent(IntPtr ctx, string fault)
    {
        LogInfo(fault);
    }

    virtual protected void ErrorEvent(IntPtr ctx, string fault)
    {
        LogError(fault);
    }

    virtual protected void IMUEvent(IntPtr so, int mask, double[] accelgyro, uint timecode, int id)
    {
        Cfunctions.Survive_default_imu_process(so, mask, accelgyro, timecode, id);
    }

    virtual protected int HTCConfigEvent(IntPtr so, string ct0conf, int len)
    {
        return Cfunctions.Survive_default_htc_config_process(so, ct0conf, len);
    }

    virtual protected void ButtonEvent(IntPtr so, byte eventType, byte buttonId, byte axis1Id, ushort axis1Val, byte axis2Id, ushort axis2Val)
    {
        Cfunctions.Survive_default_button_process(so, eventType, buttonId, axis1Id, axis1Val, axis2Id, axis2Val);
    }

    virtual protected void AngleEvent(IntPtr so, int sensor_id, int acode, uint timecode, double length, double angle, uint lh)
    {
        Cfunctions.Survive_default_angle_process(so, sensor_id, acode, timecode, length, angle, lh);

        //Debug.Log("AngleEvent");
    }

    protected void LightHouseEvent(IntPtr ctx, byte lighthouse, SurvivePose lighthouse_pose, SurvivePose object_pose)
    {
        Cfunctions.Survive_default_lighthouse_pose_process(ctx, lighthouse, lighthouse_pose, object_pose);

        //Debug.Log("LightHouseEvent");
    }

    virtual protected void LightEvent(IntPtr so, int sensor_id, int acode, int timeinsweep, UInt32 timecode, UInt32 length, UInt32 lighthouse)
    {
        Cfunctions.Survive_default_light_process(so, sensor_id, acode, timeinsweep, timecode, length, lighthouse);

        //Console.WriteLine("LightEvent");
        //Debug.Log("LightEvent");
    }

    virtual protected void PoseEvent(IntPtr so, byte lighthouse, SurvivePose pose)
    {
        Cfunctions.Survive_default_raw_pose_process(so, lighthouse, pose);

        //Debug.Log("PoseEvent");
    }

    /*
    public delegate void PoseUpdate(SurviveVector3 pos, SurviveQuaternion quat);

    public Dictionary<string, PoseUpdate> updates = new Dictionary<string, PoseUpdate>();

    public void AddCalback(string ID, PoseUpdate update)
    {
        if (!updates.ContainsKey(ID))
        {
            updates.Add(ID, update);
        }
        else
        {
            updates[ID] += update;
        }
    }
    */


    public SurviveObject GetSurviveObjectByName(string name)
    {
        if (name == "")
        {
            throw new Exception("Empty string is not accepted");
        }

        if (context == IntPtr.Zero)
            throw new Exception("The context hasn't been initialsied yet");

        return new SurviveObject( Cfunctions.Survive_get_so_by_name(context, name));
    }
}


public struct SetupConfigs
{
    public string playbackFile;
    public int playbackFactor;
    public Disambiguator disambiguator;
    public Poser poser;
    public BoolConfig calibrate;
    public string configFile;
}

public enum Disambiguator
{
    StateBased,
    Charles,
    Turvey,
    Default
}

public enum Poser
{
    CharlesSlow,
    DaveOrtho,
    Dummy,
    EPNP,
    SBA
}

public enum BoolConfig
{
    Yes,
    No,
    Default
}


public class SurviveObject
{
    private SurvivePose _pose;
    public SurvivePose Pose
    {
        get
        {
            var ptr1 = Cfunctions.Survive_object_pose(ptr);
            return (SurvivePose)Marshal.PtrToStructure(ptr1, typeof(SurvivePose));
        }
    }

    private int _charge;
    public int Charge
    {
        get
        {
            return Cfunctions.Survive_object_charge(ptr);
        }
    }
    public bool Charging
    {
        get
        {
            return Cfunctions.Survive_object_charging(ptr);
        }
    }

    IntPtr ptr;
    private bool _charging;

    public SurviveObject(IntPtr obj_ptr)
    {
        if (obj_ptr == IntPtr.Zero)
        {
            throw new Exception("Can't create SurviveObject with 0 pointer");
        }
        ptr = obj_ptr;
    }
}

public class SurviveVector3
{
    double x;
    double y;
    double z;
}

public class SurviveQuaternion
{
    double x;
    double y;
    double z;
    double w;
}
