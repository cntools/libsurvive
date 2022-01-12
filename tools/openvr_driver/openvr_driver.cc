#include <openvr_driver.h>

#include <cstring>
#include <memory>
#include <survive_api.h>

#include <cstdarg>
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

static void DriverLogVarArgs(const char *pMsgFormat, va_list args) {
	char buf[1024];
	vsnprintf(buf, sizeof(buf), pMsgFormat, args);
	vr::VRDriverLog()->Log(buf);
}

void DriverLog(const char *pMsgFormat, ...) {
	va_list args;
	va_start(args, pMsgFormat);

	DriverLogVarArgs(pMsgFormat, args);

	va_end(args);
}

void DebugDriverLog(const char *pMsgFormat, ...) {
	va_list args;
	va_start(args, pMsgFormat);

	DriverLogVarArgs(pMsgFormat, args);

	va_end(args);
}

static vr::HmdQuaternion_t survive2openvr() {
	LinmathQuat q = {};

	LinmathPoint3d survivePts[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

	LinmathPoint3d openvrPts[] = {{1, 0, 0}, {0, 0, -1}, {0, 1, 0}};

	KabschCentered(q, (FLT *)survivePts, (FLT *)openvrPts, 3);

	return vr::HmdQuaternion_t{q[0], q[1], q[2], q[3]};
}
LinmathPoint3d lh0position = {};

struct SurviveObjectOpenVRDriver : public vr::ITrackedDeviceServerDriver {
	explicit SurviveObjectOpenVRDriver(const SurviveSimpleObject *surviveSimpleObject)
		: surviveSimpleObject(surviveSimpleObject) {}

	vr::EVRInitError Activate(uint32_t unObjectId) override {
		objectId = unObjectId;

		// Prop_FieldOfViewLeftDegrees_Float
		auto objType = survive_simple_object_get_type(surviveSimpleObject);
		auto propContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(unObjectId);
		switch (objType) {
		case SurviveSimpleObject_LIGHTHOUSE:
			vr::VRProperties()->SetFloatProperty(propContainer, vr::Prop_FieldOfViewLeftDegrees_Float, 60);
			vr::VRProperties()->SetFloatProperty(propContainer, vr::Prop_FieldOfViewRightDegrees_Float, 60);
			vr::VRProperties()->SetFloatProperty(propContainer, vr::Prop_FieldOfViewBottomDegrees_Float, 60);
			vr::VRProperties()->SetFloatProperty(propContainer, vr::Prop_FieldOfViewTopDegrees_Float, 60);
			vr::VRProperties()->SetFloatProperty(propContainer, vr::Prop_TrackingRangeMinimumMeters_Float, 0.1);
			vr::VRProperties()->SetFloatProperty(propContainer, vr::Prop_TrackingRangeMaximumMeters_Float, 4);

			vr::VRProperties()->SetStringProperty(propContainer, vr::Prop_RenderModelName_String,
												  "lh_basestation_vive");
			break;
		case SurviveSimpleObject_OBJECT:
			vr::VRProperties()->SetStringProperty(propContainer, vr::Prop_RenderModelName_String,
												  "{htc}vr_tracker_vive_1_0");
			break;
		case SurviveSimpleObject_EXTERNAL:
			break;
		}

		return vr::VRInitError_None;
	}

	void Deactivate() override { objectId = vr::k_unTrackedDeviceIndexInvalid; }

	void EnterStandby() override {}

	void *GetComponent(const char *pchComponentNameAndVersion) override { return nullptr; }

	void DebugRequest(const char *pchRequest, char *pchResponseBuffer, uint32_t unResponseBufferSize) override {}

	vr::DriverPose_t Pose() const {
		SurvivePose sPose;
		survive_simple_object_get_latest_pose(surviveSimpleObject, &sPose);

		const char *name = survive_simple_object_name(surviveSimpleObject);
		if (strcmp(name, "LH0") == 0) {
			DebugDriverLog("lh0position %f %f %f", lh0position[0], lh0position[1], lh0position[2]);
		}

		vr::DriverPose_t pose = {};
		pose.poseIsValid = true;
		pose.result = vr::TrackingResult_Running_OK;
		pose.deviceIsConnected = true;

		pose.vecPosition[0] = sPose.Pos[0];
		pose.vecPosition[1] = sPose.Pos[1];
		pose.vecPosition[2] = sPose.Pos[2];
		quatcopy(&pose.qRotation.w, sPose.Rot);

		pose.qWorldFromDriverRotation = survive2openvr();
		copy3d(pose.vecWorldFromDriverTranslation, lh0position);
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
		const char *args[] = {"",
							  "--v",
							  "100",
							  "--usbmon",
							  "--log",
							  "/home/justin/.steam/logs/libsurvive.txt",
							  "--center-on-lh0",
							  "--force-calibrate"};

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
