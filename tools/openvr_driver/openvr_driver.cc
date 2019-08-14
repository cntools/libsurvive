#include <openvr/openvr_driver.h>

#include <cstring>
#include <memory>
#include <survive_api.h>

#include <map>

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec(dllexport)
#define HMD_DLL_IMPORT extern "C" __declspec(dllimport)
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C"
#else
#error "Unsupported Platform."
#endif

struct SurviveObjectOpenVRDriver : public vr::ITrackedDeviceServerDriver {
	explicit SurviveObjectOpenVRDriver(const SurviveSimpleObject *surviveSimpleObject)
		: surviveSimpleObject(surviveSimpleObject) {}

	vr::EVRInitError Activate(uint32_t unObjectId) override {
		objectId = unObjectId;

		return vr::VRInitError_None;
	}

	void Deactivate() override { objectId = vr::k_unTrackedDeviceIndexInvalid; }

	void EnterStandby() override {}

	void *GetComponent(const char *pchComponentNameAndVersion) override { return nullptr; }

	void DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize) override {}

	vr::DriverPose_t Pose() const {
		SurvivePose sPose;
		survive_simple_object_get_latest_pose(surviveSimpleObject, &sPose);

		vr::DriverPose_t pose = {0};
		pose.poseIsValid = true;
		pose.result = vr::TrackingResult_Running_OK;
		pose.deviceIsConnected = true;

		pose.vecPosition[0] = sPose.Pos[0];
		pose.vecPosition[2] = sPose.Pos[1];
		pose.vecPosition[1] = sPose.Pos[2];
		quatcopy(&pose.qRotation.w, sPose.Rot);

		pose.qWorldFromDriverRotation = vr::HmdQuaternion_t{1, 0, 0, 0};
		pose.qDriverFromHeadRotation = vr::HmdQuaternion_t{1, 0, 0, 0};

		return pose;
	}

	vr::DriverPose_t GetPose() override { return Pose(); }

	void NotifyUpdate() const {
		if (objectId != vr::k_unTrackedDeviceIndexInvalid) {
			vr::VRServerDriverHost()->TrackedDevicePoseUpdated(objectId, Pose(), sizeof(vr::DriverPose_t));
		}
	}

	uint32_t objectId = vr::k_unTrackedDeviceIndexInvalid;
	const SurviveSimpleObject *surviveSimpleObject;
};

static void log_fn(SurviveSimpleContext *ctx, SurviveLogLevel logLevel, const char *fault) {
	vr::VRDriverLog()->Log(fault);
}

class SurviveOpenVRDriver : public vr::IServerTrackedDeviceProvider {
	vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override {
		VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

		vr::VRDriverLog()->Log("Loading up\n");
		const char *args[] = {"", "--v", "100", "--log", "/home/justin/.steam/logs/libsurvive.txt"};

		actx = survive_simple_init_with_logger(sizeof(args) / sizeof(args[0]), (char *const *)args, log_fn);

		if (actx == nullptr) {
			vr::VRDriverLog()->Log("Survive context was null");
			return vr::VRInitError_Init_InterfaceNotFound;
		}
		survive_simple_start_thread(actx);

		for (const SurviveSimpleObject *it = survive_simple_get_first_object(actx); it != 0;
			 it = survive_simple_get_next_object(actx, it)) {
			if (objects[it] == nullptr) {
				objects[it] = std::make_unique<SurviveObjectOpenVRDriver>(it);
				vr::VRServerDriverHost()->TrackedDeviceAdded(survive_simple_object_name(it),
															 survive_simple_object_get_type(it) ==
																	 SurviveSimpleObject_LIGHTHOUSE
																 ? vr::TrackedDeviceClass_TrackingReference
																 : vr::TrackedDeviceClass_GenericTracker,
															 objects[it].get());

				vr::VRDriverLog()->Log("Loaded object...");
			}
		}

		vr::VRDriverLog()->Log("Init done...");

		return vr::VRInitError_None;
	}

	void Cleanup() override {
		survive_simple_close(actx);
		actx = 0;
	}

	const char *const *GetInterfaceVersions() override { return vr::k_InterfaceVersions; }

	void RunFrame() override {
		for (const SurviveSimpleObject *it = survive_simple_get_next_updated(actx); it != 0;
			 it = survive_simple_get_next_updated(actx)) {
			if (objects[it] == nullptr) {
				objects[it] = std::make_unique<SurviveObjectOpenVRDriver>(it);
				vr::VRServerDriverHost()->TrackedDeviceAdded(survive_simple_object_name(it),
															 vr::TrackedDeviceClass_Controller, objects[it].get());
			}

			objects[it]->NotifyUpdate();
		}
	}

	bool ShouldBlockStandbyMode() override { return false; }

	void EnterStandby() override {}

	void LeaveStandby() override {}

	SurviveSimpleContext *actx = 0;
	std::map<const SurviveSimpleObject *, std::unique_ptr<SurviveObjectOpenVRDriver>> objects;
};

SurviveOpenVRDriver g_serverDriver;

HMD_DLL_EXPORT void *HmdDriverFactory(const char *pInterfaceName, int *pReturnCode) {
	if (0 == strcmp(vr::IServerTrackedDeviceProvider_Version, pInterfaceName)) {
		return &g_serverDriver;
	}

	if (pReturnCode)
		*pReturnCode = vr::VRInitError_Init_InterfaceNotFound;

	return NULL;
}