#include <iostream>
#include <libsurvive/survive.h>
#include <map>

#include <cmath>
#include <openvr.h>
#include <os_generic.h>
#include <vector>

STATIC_CONFIG_ITEM(OpenVR_DRIVER_ENABLE, "openvr", 'b', "Load openvr driver", 0)

bool operator!=(const LinmathPose &a, const LinmathPose &b) {
	for (int i = 0; i < 3; i++)
		if (fabs(a.Pos[i] - b.Pos[i]) > 0.0001)
			return true;
	for (int i = 0; i < 4; i++)
		if (fabs(a.Rot[i] - b.Rot[i]) > 0.0001)
			return true;

	return false;
}

const char *device_names[vr::TrackedDeviceClass_Max] = {"UNKNOWN", "HMD", "WM", "TR", "LH", "RD"};

static SurvivePose survivePoseFromDevicePose(const vr::TrackedDevicePose_t &dpose) {
	auto &mat = dpose.mDeviceToAbsoluteTracking.m;
	FLT matrix33[] = {mat[0][0], mat[0][1], mat[0][2], mat[1][0], mat[1][1],
					  mat[1][2], mat[2][0], mat[2][1], mat[2][2]};

	SurvivePose p = {};
	for (int i = 0; i < 3; i++) {
		p.Pos[i] = mat[i][3];
	}
	quatfrommatrix33(p.Rot, matrix33);
	return p;
}

static SurviveVelocity surviveVelocityFromDevicePose(const vr::TrackedDevicePose_t &dpose) {
	SurviveVelocity p = {};

	for (int i = 0; i < 3; i++) {
		p.Pos[i] = dpose.vVelocity.v[i];
		p.AxisAngleRot[i] = dpose.vAngularVelocity.v[i];
	}
	return p;
}

struct OpenVRDriver {
	vr::IVRSystem *vr_system = nullptr;
	SurviveContext *ctx = nullptr;
	bool *keep_running = nullptr;

	SurvivePose openvr2survive = {};

	OpenVRDriver(SurviveContext *ctx) : ctx(ctx) {}

	~OpenVRDriver() { vr::VR_Shutdown(); }

	std::map<std::string, SurvivePose> vr_poses;

	typedef struct {
		SurvivePose vr_pose;
		SurvivePose last_pose_sent;
		char name[32] = {0};

		int bsd_idx = -1;
	} tracked_device_t;

	tracked_device_t devices[vr::k_unMaxTrackedDeviceCount] = {};
	std::vector<tracked_device_t *> device_list;

	int32_t get_bsd_idx(const tracked_device_t &dev) const {
		for (int i = 0; i < ctx->activeLighthouses; i++) {
			char buffer[128] = {0};
			snprintf(buffer, 128, "LHB-%08X", ctx->bsd[i].BaseStationID);

			if (strcmp(buffer, dev.name) == 0)
				return i;
		}

		return -1;
	}

	struct Point3 {
		LinmathVec3d v = {0};
		Point3(FLT x = 0, FLT y = 0, FLT z = 0) {
			v[0] = x;
			v[1] = y;
			v[2] = z;
		}
	};

	SurvivePose getOpenVr2Survive() {
		if (quatiszero(openvr2survive.Rot)) {
			const Point3 basis[] = {
				Point3(1, 0, 0),
				Point3(0, 1, 0),
				Point3(0, 0, 1),
			};

			std::vector<Point3> openvrPts, survivePts;

			for (auto &d : devices) {
				auto bsd_idx = d.bsd_idx;
				if (bsd_idx != -1) {
					if (ctx->bsd[bsd_idx].PositionSet && !quatiszero(d.vr_pose.Rot)) {

						for (auto &x : basis) {
							Point3 openvrPt, survivePt;
							ApplyPoseToPoint(openvrPt.v, &d.vr_pose, x.v);
							ApplyPoseToPoint(survivePt.v, survive_get_lighthouse_position(ctx, bsd_idx), x.v);

							openvrPts.push_back(openvrPt);
							survivePts.push_back(survivePt);
						}
					}
				}
			}

			if (survivePts.size() >= 3) {
				Kabsch(&openvr2survive, (FLT *)&openvrPts[0].v, (FLT *)&survivePts[0].v, openvrPts.size());
				SV_VERBOSE(50, "Mapping openvr space to libsurvive space with %d lighthouse positions",
						   (uint32_t)(openvrPts.size() / 3));
			}
		}
		return openvr2survive;
	}

	SurviveVelocity velToLibsurviveWorld(const SurviveVelocity &poseInOpenVR) {
		SurvivePose OpenVr2Survive = getOpenVr2Survive();
		if (quatiszero(OpenVr2Survive.Rot))
			return {};

		SurviveVelocity rtn;
		quatrotatevector(rtn.Pos, OpenVr2Survive.Rot, poseInOpenVR.Pos);
		quatrotatevector(rtn.AxisAngleRot, OpenVr2Survive.Rot, poseInOpenVR.AxisAngleRot);

		return rtn;
	}

	void getPropString(vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, char *pchBuffer,
					   size_t availBytes) {
		size_t propLength = vr_system->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, availBytes, nullptr);
		pchBuffer[propLength] = 0;
	}

	SurvivePose txToLibsurviveWorld(const SurvivePose &poseInOpenVR) {
		SurvivePose OpenVr2Survive = getOpenVr2Survive();
		if (quatiszero(OpenVr2Survive.Rot))
			return {};

		SurvivePose rtn;
		ApplyPoseToPose(&rtn, &OpenVr2Survive, &poseInOpenVR);

		return rtn;
	}

	int init_time = 500;
	int poll() {
		if (vr_system == 0) {
			OGUSleep(10000 * 50);
			vr::EVRInitError eError = vr::VRInitError_None;
			vr_system = vr::VR_Init(&eError, vr::VRApplication_Background);
			if (eError != vr::VRInitError_None || vr_system == nullptr) {
				SV_WARN("Unable to init VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
				return 0;
			}
		}

		OGUSleep(10000);

		vr::VREvent_t event = {};
		while (vr_system->PollNextEvent(&event, sizeof(event))) {
			SV_VERBOSE(200, "(OpenVR) Event: %s (%d)",
					   vr_system->GetEventTypeNameFromEnum(static_cast<vr::EVREventType>(event.eventType)),
					   event.eventType);

			if (event.eventType == 700) {
				vr_system = 0;
				vr::VR_Shutdown();
				return 0;
			}
		}

		vr::TrackedDevicePose_t trackedDevicePoses[vr::k_unMaxTrackedDeviceCount] = {};
		vr_system->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0, trackedDevicePoses,
												   vr::k_unMaxTrackedDeviceCount);

		for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++) {
			auto &trackedDevicePose = trackedDevicePoses[unDevice];
			if (!trackedDevicePose.bDeviceIsConnected)
				continue;
			if (!trackedDevicePose.bPoseIsValid)
				continue;

			auto pose = survivePoseFromDevicePose(trackedDevicePose);
			auto velocity = surviveVelocityFromDevicePose(trackedDevicePose);

			tracked_device_t &dev = devices[unDevice];

			if (dev.name[0] == 0) {
				device_list.push_back(&dev);
				getPropString(unDevice, vr::Prop_SerialNumber_String, dev.name, sizeof(dev.name));
				dev.bsd_idx = get_bsd_idx(dev);

				SV_INFO("New OpenVR device %s", dev.name);
			}

			dev.vr_pose = pose;

			if (init_time > 0) {
				init_time--;
				continue;
			}

			auto poseInSurvive = txToLibsurviveWorld(pose);
			auto velInSurvive = velToLibsurviveWorld(velocity);

			if (quatmagnitude(poseInSurvive.Rot) > 0.0001) {
				if (poseInSurvive != dev.last_pose_sent) {
					dev.last_pose_sent = poseInSurvive;

					survive_get_ctx_lock(ctx);
					ctx->external_poseproc(ctx, dev.name, &poseInSurvive);
					ctx->external_velocityproc(ctx, dev.name, &velInSurvive);
					survive_release_ctx_lock(ctx);
				}
			}
		}

		return 0;
	}
};

static void *openvr_thread(void *d) {
	auto driver = static_cast<OpenVRDriver *>(d);
	while (driver->keep_running == nullptr || *driver->keep_running) {
		driver->poll();
	}
	return nullptr;
}

static int openvr_close(struct SurviveContext *ctx, void *_driver) {
	auto driver = static_cast<OpenVRDriver *>(_driver);
	delete (driver);
	return 0;
}

int DriverRegOpenVR(SurviveContext *ctx) {
	vr::EVRInitError eError = vr::VRInitError_None;

	auto driver = new OpenVRDriver(ctx);
	driver->keep_running = survive_add_threaded_driver(ctx, driver, "OpenVR driver", openvr_thread, openvr_close);
	return 0;
}

REGISTER_LINKTIME(DriverRegOpenVR)
