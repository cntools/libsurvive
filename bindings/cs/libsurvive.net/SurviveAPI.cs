using System;
using System.Runtime.InteropServices;
using SurviveContextPtr = System.IntPtr;
using SurviveObjectPtr = System.IntPtr;
using SurvivePosePtr = System.IntPtr;

using SurviveSimpleContextPtr = System.IntPtr;
using SurviveSimpleObjectPtr = System.IntPtr;

namespace libsurvive
{
public class SurviveAPIOObject : IDisposable {
	protected IntPtr Aso = IntPtr.Zero;

	protected SurviveAPIOObject(SurviveSimpleObjectPtr aso) { Aso = aso; }
	public static SurviveAPIOObject Create(SurviveSimpleObjectPtr aso) {
		if (aso == IntPtr.Zero)
			return null;
		return new SurviveAPIOObject(aso);
	}

	public IntPtr Ptr() { return Aso; }

	public float LatestPoseTime {
		get { return (float) Cfunctions_api.survive_simple_object_get_latest_pose(Aso, IntPtr.Zero); }
	}

	private IntPtr latestPosePtr = Marshal.AllocHGlobal(Marshal.SizeOf<SurvivePose>());
	public SurvivePose LatestPose {
		get {
			Cfunctions_api.survive_simple_object_get_latest_pose(Aso, latestPosePtr);
			SurvivePose pose = Marshal.PtrToStructure(latestPosePtr, typeof(SurvivePose)) as SurvivePose;
			return pose;
		}
	}

	public string Name => Marshal.PtrToStringAnsi(Cfunctions_api.survive_simple_object_name(Aso));

	public string SerialNumber => Marshal.PtrToStringAnsi(Cfunctions_api.survive_simple_serial_number(Aso));

	public void Dispose() { Marshal.FreeHGlobal(latestPosePtr); }
}

public class SurviveAPI : IDisposable {
	protected IntPtr actx;
	protected bool threadStarted = false;
	private SurviveSimpleLogFn logFunction;

	public void Dispose() { Close(); }

	public void Close() {
		if (actx != IntPtr.Zero) {
			Cfunctions_api.survive_simple_close(actx);
		}
		actx = IntPtr.Zero;
	}

	public SurviveAPI(string[] args, SurviveSimpleLogFn logFunc = null, bool dontStartYet = false) {
		Init(args, logFunc, dontStartYet);
	}

	virtual protected void InfoEvent(SurviveObjectPtr ctx, UInt32 loglevl, string fault) {
		Console.Out.WriteLine(fault);
	}

	internal void Init(string[] args, SurviveSimpleLogFn logFunc = null, bool dontStartYet = false) {
		if (logFunc == null) {
			logFunc = InfoEvent;
		}

		logFunction = logFunc;
		actx = Cfunctions_api.survive_simple_init_with_logger(args.Length, args, logFunction);

		if (actx == IntPtr.Zero) {
			throw new Exception("There was a problem initializing the lib!");
		}

		if (!dontStartYet) {
			Start();
		}
	}

	public void Start() {
		if (threadStarted)
			return;

		threadStarted = true;
		Cfunctions_api.survive_simple_start_thread(actx);
	}

	public SurviveAPIOObject GetFirstObject() {
		return SurviveAPIOObject.Create(Cfunctions_api.survive_simple_get_first_object(actx));
	}

	public SurviveAPIOObject GetNextObject(SurviveAPIOObject obj) {
		return SurviveAPIOObject.Create(Cfunctions_api.survive_simple_get_next_object(actx, obj.Ptr()));
	}

	public SurviveAPIOObject GetNextUpdated() {
		return SurviveAPIOObject.Create(Cfunctions_api.survive_simple_get_next_updated(actx));
	}

	public bool IsRunning() { return Cfunctions_api.survive_simple_is_running(actx); }

	public bool WaitForUpdate() { return Cfunctions_api.survive_simple_wait_for_update(actx); }
}
}