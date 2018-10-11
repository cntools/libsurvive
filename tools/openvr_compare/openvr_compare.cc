#include <iostream>
#include <libsurvive/survive.h>
#include <map>
#include <math.h>
#include <openvr.h>
#include <string.h>

#include <signal.h>

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

static SurvivePose surviveVelocityFromDevicePose(const vr::TrackedDevicePose_t &dpose) {
	SurvivePose p = {};
	FLT euler[3];

	SurvivePose pose = survivePoseFromDevicePose(dpose);

	for (int i = 0; i < 3; i++) {
		p.Pos[i] = dpose.vVelocity.v[i];
		euler[i] = dpose.vAngularVelocity.v[i];
	}
	quatfromeuler(p.Rot, euler);
	return p;
}

std::map<std::string, SurvivePose> vr_poses;
std::map<std::string, SurvivePose> vr_poses_sent;

SurviveContext *ctx = 0;

const SurvivePose *getlh02OpenVr() {
	if (ctx == 0) {
		std::cerr << "ctx null" << std::endl;
		return {};
	}
	if (vr_poses.find("openvr_LH0") == vr_poses.end()) {
		return {};
	}
	if (ctx->bsd[0].PositionSet == false) {
		return {};
	}

	return &vr_poses["openvr_LH0"];
}

SurvivePose getOpenVr2Survive() {
	// return LinmathPose_Identity;

	auto lh02OpenVr = getlh02OpenVr();
	if (!lh02OpenVr) {
		return {};
	}

	auto lh02Survive = ctx->bsd[0].Pose;

	auto OpenVr2lh0 = InvertPoseRtn(lh02OpenVr);
	SurvivePose OpenVr2Survive;
	ApplyPoseToPose(&OpenVr2Survive, &lh02Survive, &OpenVr2lh0);

	return OpenVr2Survive;
}

SurvivePose velToLibsurviveWorld(const SurvivePose &poseInOpenVR) {
	SurvivePose OpenVr2Survive = getOpenVr2Survive();
	if (quatiszero(OpenVr2Survive.Rot))
		return {};

	SurvivePose rtn;
	quatrotatevector(rtn.Pos, OpenVr2Survive.Rot, poseInOpenVR.Pos);

	LinmathQuat iRot;
	quatgetconjugate(iRot, OpenVr2Survive.Rot);
	quatconjugateby(rtn.Rot, iRot, poseInOpenVR.Rot);

	return rtn;
}
SurvivePose txToLibsurviveWorld(const SurvivePose &poseInOpenVR) {
	SurvivePose OpenVr2Survive = getOpenVr2Survive();
	if (quatiszero(OpenVr2Survive.Rot))
		return {};

	SurvivePose rtn;
	ApplyPoseToPose(&rtn, &OpenVr2Survive, &poseInOpenVR);

	return rtn;
}

bool operator!=(const LinmathPose &a, const LinmathPose &b) {
	for (int i = 0; i < 3; i++)
		if (fabs(a.Pos[i] - b.Pos[i]) > 0.0001)
			return true;
	for (int i = 0; i < 4; i++)
		if (fabs(a.Rot[i] - b.Rot[i]) > 0.0001)
			return true;

	return false;
}

bool openvr_poll(vr::IVRSystem &vr_system) {
	vr::VREvent_t event;
	while (vr_system.PollNextEvent(&event, sizeof(event))) {
		printf("(OpenVR) Event: %d\n", event.eventType);
	}

	vr::TrackedDevicePose_t trackedDevicePoses[vr::k_unMaxTrackedDeviceCount] = {};
	vr_system.GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0, trackedDevicePoses,
											  vr::k_unMaxTrackedDeviceCount);

	size_t device_cnt[vr::TrackedDeviceClass_Max] = {};

	for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++) {
		auto &trackedDevicePose = trackedDevicePoses[unDevice];
		if (!trackedDevicePose.bDeviceIsConnected)
			continue;
		if (!trackedDevicePose.bPoseIsValid)
			continue;

		vr::ETrackedDeviceClass trackedDeviceClass = vr_system.GetTrackedDeviceClass(unDevice);

		if (trackedDeviceClass >= vr::TrackedDeviceClass_Max)
			continue;

		int num = device_cnt[trackedDeviceClass]++;
		auto pose = survivePoseFromDevicePose(trackedDevicePose);
		auto velocity = surviveVelocityFromDevicePose(trackedDevicePose);

		// LH can come in any order, we order them based on which one is used as origin
		if (trackedDeviceClass == vr::TrackedDeviceClass_TrackingReference) {
			num = norm3d(pose.Pos) < .5;
		}

		char name[32] = {};
		sprintf(name, "openvr_%s%d", device_names[trackedDeviceClass], num);

		auto poseInSurvive = txToLibsurviveWorld(pose);
		auto velInSurvive = velToLibsurviveWorld(velocity);

		vr_poses[name] = pose;

		if (quatmagnitude(poseInSurvive.Rot) > 0.0001) {
			if (poseInSurvive != vr_poses_sent[name]) {
				vr_poses_sent[name] = poseInSurvive;
				ctx->externalposeproc(ctx, name, &poseInSurvive);
				ctx->externalvelocityproc(ctx, name, &velInSurvive);
			}
		}
	}

	return true;
}

// Try real hard to cleanly exit, or you'll probably crash vrserver
int caught_abort = 0;
void sighandler(int s) {
	switch (s) {
	case 2:
	case 9:
		break;
	default:
		return;
	}
	if (caught_abort == 1)
		exit(-s);
	caught_abort++;
}

static void install_sig_handler() {
	struct sigaction sigIntHandler;

	sigIntHandler.sa_handler = sighandler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;

	sigaction(SIGINT, &sigIntHandler, NULL);
}

int main(int argc, char **argv) {
	install_sig_handler();
	ctx = survive_init(argc, argv);

	if (ctx == 0) { // Implies --help or similiar
		return -1;
	}

	survive_configi(ctx, "usbmon", SC_SET | SC_OVERRIDE, 1);
	vr::EVRInitError eError = vr::VRInitError_None;
	auto vr_system = vr::VR_Init(&eError, vr::VRApplication_Utility);
	if (eError != vr::VRInitError_None) {
		std::cerr << "Unable to init VR runtime: " << vr::VR_GetVRInitErrorAsEnglishDescription(eError) << std::endl;
		return -1;
	}

	survive_startup(ctx);

	while (survive_poll(ctx) == 0 && openvr_poll(*vr_system) && caught_abort == false) {
	}

	vr::VR_Shutdown();

	return 0;
}
