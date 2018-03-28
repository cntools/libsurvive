using System;
using System.Collections.Generic;
using System.Text;

namespace LibSurviveBinding
{
    class Binding
    {
        /*
        typedef int (* htc_config_func) (SurviveObject* so, char* ct0conf, int len);
        typedef void (* text_feedback_func) (SurviveContext* ctx, const char* fault );
        typedef void (* light_process_func) (SurviveObject* so, int sensor_id, int acode, int timeinsweep, uint32_t timecode, uint32_t length, uint32_t lighthouse);
        typedef void (* imu_process_func) (SurviveObject* so, int mask, FLT* accelgyro, uint32_t timecode, int id);
        typedef void (* angle_process_func) (SurviveObject* so, int sensor_id, int acode, uint32_t timecode, FLT length, FLT angle, uint32_t lh);
        typedef void (* button_process_func) (SurviveObject* so, uint8_t eventType, uint8_t buttonId, uint8_t axis1Id, uint16_t axis1Val, uint8_t axis2Id, uint16_t axis2Val);
        typedef void (* raw_pose_func) (SurviveObject* so, uint8_t lighthouse, SurvivePose* pose);
        typedef void (* lighthouse_pose_func) (SurviveContext* ctx, uint8_t lighthouse, SurvivePose* lighthouse_pose,
                                       SurvivePose* object_pose);
        */
    }

    public delegate int htc_config_func(IntPtr so, char ct0conf, int len);
    public delegate void text_feedback_func(IntPtr ctx, string fault);
    public delegate void light_process_func(IntPtr so, int sensor_id, int acode, int timeinsweep, UInt32 timecode, UInt32 length, UInt32 lighthouse);
    public delegate void imu_process_func(IntPtr so, int mask, double accelgyro, UInt32 timecode, int id);
    public delegate void angle_process_func(IntPtr so, int sensor_id, int acode, UInt32 timecode, double length, double angle, UInt32 lh);
    public delegate void button_process_func(IntPtr so, byte eventType, byte buttonId, byte axis1Id, UInt16 axis1Val, byte axis2Id, UInt16 axis2Val);
    public delegate void raw_pose_func(IntPtr so, byte lighthouse, IntPtr pose);
    public delegate void lighthouse_pose_func(IntPtr ctx, byte lighthouse, IntPtr lighthouse_pose,
                                   IntPtr object_pose);
    public delegate void handle_lightcap_func (IntPtr so, IntPtr le);

}
