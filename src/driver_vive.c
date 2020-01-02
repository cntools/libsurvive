// Unofficial driver for the official Valve/HTC Vive hardware.
//
// Based off of https://github.com/collabora/OSVR-Vive-Libre
// Originally Copyright 2016 Philipp Zabel
// Originally Copyright 2016 Lubosz Sarnecki <lubosz.sarnecki@collabora.co.uk>
// Originally Copyright (C) 2013 Fredrik Hultin
// Originally Copyright (C) 2013 Jakob Bornecrantz
//
// But, re-written as best as I can to get it put under an open souce license instead of a forced-source license.
// If there are portions of the code too similar to the original, I would like to know  so they can be re-written.
// All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.
#include <errno.h>
#include <jsmn.h>
#include <os_generic.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <survive.h>
#include <sys/stat.h>
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <assert.h>
#include <malloc.h> // for alloca

#endif

#include "json_helpers.h"
#include "survive_config.h"
#include "survive_default_devices.h"

#include "driver_vive.h"
#include "lfsr_lh2.h"
//#define DEBUG_WATCHMAN 1

struct SurviveViveData;

struct DeviceInfo {
	const char *name;
	const char *codename;

	uint16_t vid;
	uint16_t pid;
	enum USB_DEV_t type;

	struct Endpoint_t {
		uint8_t num;
		const char *name;
		enum USB_IF_t type;
	} endpoints[MAX_INTERFACES_PER_DEVICE];

	struct Magic_t {
		bool code;
		const uint8_t *magic;
		const size_t length;
	} magics[8];
};

enum vive_report_ids {
	// Possibly 0x03 - Gyro range, 0x02 - Accel range
	// Tracker/lh1/[rf,usb]: (device -> steamvr):  01 03 02
	VIVE_REPORT_INFO = 1,
	VIVE_REPORT_IMU = 0x20,
	VIVE_REPORT_USB_TRACKER_LIGHTCAP_V1 = 0x21,
	VIVE_REPORT_RF_WATCHMAN = 0x23,
	VIVE_REPORT_RF_WATCHMANx2 = 0x24,
	VIVE_REPORT_USB_LIGHTCAP_REPORT_V1 = 0x25,

	// I've seen this with a byte of data; 01. Maybe this turns on and off?
	VIVE_REPORT_RF_TURN_OFF = 0x26,
	VIVE_REPORT_USB_LIGHTCAP_REPORT_V2 = 0x27,
	VIVE_REPORT_USB_LIGHTCAP_REPORT_RAW_MODE_1 = 0x28,

	VIVE_REPORT_COMMAND = 0xff,
	// Below is from https://github.com/nairol/LighthouseRedox/blob/master/docs/USB%20Protocol.md and not necessarilly
	// up to date / used
	VIVE_REPORT_HMD_PROXIMITY = 0x3,

	VIVE_REPORT_VERSION = 0x05,

	// Tracker/lh1/[rf,usb]: (steamvr -> device): 04 00 00 00   00
	VIVE_REPORT_CHANGE_MODE = 0x04,
	// Tracker/lh1/[rf,usb]: (steamvr -> device):  07 03 00 00   00
	VIVE_REPORT_LIGHTCAP_VERBOSITY = 0x07,

	// Seems like 0x4/0x7 are used in conjunction:
	// Sync mode 0:
	// --> 04 00 00 00   00
	// --> 07 03 00 00   00
	// Sync mode 1:
	// --> 04 01 00 00   00
	// --> 07 03 00 00   00
	// Sync mode 2:
	// --> 04 03 00 00   00
	// --> 07 03 00 00   00

	// Tracker/lh1/rf: (steamvr -> device): (CLEAR_FEATURE)
	//                 (device -> steamvr): 08 03 00 00
	// If clear_feature is sent; seems to be asking how much data is in the user alloc area.
	// Seems like its maybe current offset as uint32_t, total size as uint32_t.
	// The tracker seems to not have a userdata section; and responds with 0x8 and then whatever was in the buffer.
	VIVE_REPORT_HMD_SET_READOFFSET_USERDATA = 0x08,
	VIVE_REPORT_HMD_READ_USERDATA = 0x09,

	VIVE_REPORT_CONFIG_READMODE = 0x10,
	VIVE_REPORT_CONFIG_READ = 0x11,

	// Sent and echoed by steamvr; no data
	// Directly proceeds a 0x04...
	// Possibly a different version packet. Sent when you do 'version' in lh_console.
	// knuckles -> lighthouse_console:
	// 13 b5 35 28   5d 03 00 00   00 77 61 74   63 68 6d 61   6e 00 00 00   00 00 00 00   00 56 61 6c   76 65 42 75 |
	// ..5(  ]...  .wat  chma  n...  ....  .Val  veBu
	// 69 6c 64 65   72 30 32 00   00 09 00 0e   11 00 00 bd   26 1a 02 0a   02 f1 52 2c   5c 00 00 00   00 00 00 00 |
	// ilde  r02.  ....  ....  &...  ..R,  \...  ....
	VIVE_REPORT_USB_TRACKER_UNKNOWN = 0x13,

	// Tracker/lh1/usb: (steamvr -> device): 16 01 00 00   00
	VIVE_REPORT_REBOOT = 0x16

};

enum vive_commands {
	// This command seems to dictate protocol and sync model. I think the first byte is the 'protocol',
	// the second second and fourth together dictate the sync mode.
	// Tracker/lh1/rf: (steamvr -> device):  01 00 00 02   00 00 |    ....  ..
	// Tracker/lh1/rf: (steamvr -> device):  01 00 00            |    ....  ..

	// Tracker/lh1/rf: (lhconsole -> device):  sync mode 0: 01 00 00 02   00 00
	// Tracker/lh1/rf: (lhconsole -> device):  sync mode 1: 01 01 00 02   00 00
	// Tracker/lh1/rf: (lhconsole -> device):  sync mode 2: 01 01 00 02   01 00

	// Knuckles/lh2/rf: (steamvr -> device): 01 00 00 02   00 00
	//                                       00 01 00
	//                                       01 01 00 02   00 00

	VIVE_COMMAND_CHANGE_PROTOCOL = 0x87,

	// Tracker/lh1/rf: (steamvr -> device)
	//   - Sends 2x with no data as SET_CONFIGURATION
	//   - Sends 1x as CLEAR_FEATURE
	//   - Recv back:
	//                ff 83 2d 00   b1 d7 b3 ba   01 00 23 00   00 02 0b 00   00 00 04 6a   19 e6 5b 09   06 00 02 84 0b
	//                b1 d7 b3     |    ..-.  ....  ..#.  ....  ...j  ..[.  ....  ....
	//                ba 05 09 3a   87 5a 0c 00   02 00 00 0a   cf 3b 3c 5a |    ...:  .Z..  ....  .;<Z  ....  .... ....
	//                ....
	VIVE_COMMAND_UNKNOWN3 = 0x83,
	VIVE_COMMAND_HAPTIC_PULSE = 0x8F,

	// Wants '6f 66 66 21 |    off!' as data
	VIVE_COMMAND_HAPTIC_POWER_OFF = 0x9F,

	// Knuckles/lh2/rf: (steamvr -> device):  be 5b 32 54   11 cf 83 75   53 8a 08 6a   53 58 d0 b1
	// Tracker/lh1/rf:  (steamvr -> device):  be 5b 32 54   11 cf 83 75   53 8a 08 6a   53 58 d0 b1 |    .[2T  ...u S..j
	// SX..
	VIVE_COMMAND_UNKNOWN1 = 0x96,

	// Keepalive?
	// Tracker/lh1/rf: (steamvr -> device):
	//                 (device -> steamvr): 57 16 a0 b9   32 33 46 45   41 33 44 44   44 41 00 00   00 00 00 00   00 00
	//                 00 00   00 00 00 00   00 00 00 00     |    W...  23FE  A3DD  DA..  ....  ....  ....  ....
	//                                      00 00 |    ..
	VIVE_COMMAND_UNKNOWN2 = 0xa1,

	// Knuckles/lh2/rf: (steamvr->sevice): 00
	VIVE_COMMAND_UNKNOWN4 = 0xb5,

	// Knuckles/lh2/rf: (steamvr->sevice)
	VIVE_COMMAND_UNKNOWN5 = 0x8c,
};

static uint8_t vive_magic_power_on[64] = {0x04, 0x78, 0x29, 0x38, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01};
static uint8_t vive_magic_enable_lighthouse[5] = {VIVE_REPORT_CHANGE_MODE};
static uint8_t vive_magic_enable_lighthouse_more[5] = {VIVE_REPORT_LIGHTCAP_VERBOSITY, 0x03};
static uint8_t vive_magic_power_off[] = {
	0x04, 0x78, 0x29, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x30, 0x05, 0x77,
	0x00, 0xe4, 0xf7, 0x33, 0x00, 0xe4, 0xf7, 0x33, 0x00, 0x60, 0x6e, 0x72, 0x00, 0xb4, 0xf7, 0x33,
	0x00, 0x04, 0x00, 0x00, 0x00, 0x70, 0xb0, 0x72, 0x00, 0x90, 0xf7, 0x33, 0x00, 0x7c, 0xf8, 0x33,
	0x00, 0xd0, 0xf7, 0x33, 0x00, 0x3c, 0x68, 0x29, 0x65, 0x24, 0xf9, 0x33, 0x00, 0x00, 0x00, 0x00,
};
static uint8_t vive_magic_raw_mode_1[] = {VIVE_REPORT_CHANGE_MODE, 0x01, 0x00, 0x00, 0x00};

static uint8_t vive_magic_rf_raw_mode_0[] = {
	VIVE_REPORT_COMMAND, VIVE_COMMAND_CHANGE_PROTOCOL, 0x6, 0x01, 0x00, 0x00, 0x02, 0x00, 0x00};
static uint8_t vive_magic_rf_raw_mode_1[] = {
	VIVE_REPORT_COMMAND, VIVE_COMMAND_CHANGE_PROTOCOL, 0x6, 0x01, 0x01, 0x00, 0x02, 0x00, 0x00};
static uint8_t vive_magic_protocol_switch[] = {
	VIVE_REPORT_COMMAND, VIVE_COMMAND_CHANGE_PROTOCOL, 0x3, 0x00, 0x01, 0x00};
static uint8_t vive_magic_protocol_super_magic[] = {VIVE_REPORT_COMMAND,
													VIVE_COMMAND_UNKNOWN1,
													0x10,
													0xbe,
													0x5b,
													0x32,
													0x54,
													0x11,
													0xcf,
													0x83,
													0x75,
													0x53,
													0x8a,
													0x08,
													0x6a,
													0x53,
													0x58,
													0xd0,
													0xb1};

#define MAGIC_CTOR(ison, buffer)                                                                                       \
	{ .code = ison, .magic = buffer, .length = sizeof(buffer) }
const struct DeviceInfo KnownDeviceTypes[] = {
	{.vid = 0x0bb4,
	 .pid = 0x2c87,
	 .type = USB_DEV_HMD,
	 .name = "HMD",
	 .codename = "",
	 .endpoints = {{.num = 0x81, .name = "Mainboard", .type = USB_IF_HMD_HEADSET_INFO}},
	 .magics = {MAGIC_CTOR(true, vive_magic_power_on), MAGIC_CTOR(false, vive_magic_power_off)}},
	{.vid = 0x0bb4,
	 .pid = 0x030e,
	 .type = USB_DEV_HMD,
	 .name = "HMD",
	 .codename = "",
	 .endpoints = {{.num = 0x81, .name = "Mainboard", .type = USB_IF_HMD_HEADSET_INFO}},
	 .magics = {MAGIC_CTOR(true, vive_magic_power_on), MAGIC_CTOR(false, vive_magic_power_off)}},
	{.vid = 0x28de,
	 .pid = 0x2000,
	 .type = USB_DEV_HMD_IMU_LH,
	 .name = "HMD IMU & LH",
	 .codename = "HMD",
	 .endpoints = {{.num = 0x81, .name = "IMU", .type = USB_IF_HMD_IMU},
				   {.num = 0x82, .name = "Lightcap", .type = USB_IF_HMD_LIGHTCAP}},
	 .magics = {MAGIC_CTOR(true, vive_magic_enable_lighthouse), MAGIC_CTOR(true, vive_magic_enable_lighthouse_more)}},
	{.vid = 0x28de,
	 .pid = 0x2101,
	 .type = USB_DEV_WATCHMAN1,
	 .name = "Watchman",
	 .codename = "WM0",
	 .endpoints = {{.num = 0x81, .name = "IMU/Lightcap/Buttons", .type = USB_IF_WATCHMAN1}},
	 .magics =
		 {// MAGIC_CTOR(true, vive_magic_enable_lighthouse),
		  // MAGIC_CTOR(true, vive_magic_enable_lighthouse_more),
		  MAGIC_CTOR(true, vive_magic_protocol_super_magic), MAGIC_CTOR(true, vive_magic_rf_raw_mode_0),
		  MAGIC_CTOR(true, vive_magic_protocol_switch)}},
	{.vid = 0x28de,
	 .pid = 0x2022,
	 .type = USB_DEV_TRACKER0,
	 .name = "Tracker",
	 .codename = "TR0",
	 .endpoints =
		 {
			 {.num = 0x81, .name = "IMU", .type = USB_IF_TRACKER0_IMU},
			 {.num = 0x82, .name = "Lightcap", .type = USB_IF_TRACKER0_LIGHTCAP},
			 {.num = 0x83, .name = "Buttons", .type = USB_IF_TRACKER0_BUTTONS},
		 },
	 .magics = {MAGIC_CTOR(true, vive_magic_enable_lighthouse), MAGIC_CTOR(true, vive_magic_enable_lighthouse_more)}},
	{.vid = 0x28de,
	 .pid = 0x2300,
	 .type = USB_DEV_TRACKER1,
	 .name = "Tracker (2018)",
	 .codename = "T20",
	 .endpoints =
		 {
			 {.num = 0x81, .name = "IMU", .type = USB_IF_TRACKER1_IMU},
			 {.num = 0x83, .name = "Lightcap", .type = USB_IF_TRACKER1_LIGHTCAP},
			 {.num = 0x84, .name = "Buttons", .type = USB_IF_TRACKER1_BUTTONS},
		 },
	 .magics = {MAGIC_CTOR(true, vive_magic_enable_lighthouse), MAGIC_CTOR(true, vive_magic_enable_lighthouse_more)}},
	{.vid = 0x28de,
	 .pid = 0x2012,
	 .type = USB_DEV_W_WATCHMAN1,
	 .name = "Wired Watchman",
	 .codename = "WW0",
	 .endpoints = {{.num = 0x81, .name = "IMU", .type = USB_IF_W_WATCHMAN1_IMU},
				   {.num = 0x82, .name = "Lightcap", .type = USB_IF_W_WATCHMAN1_LIGHTCAP},
				   {.num = 0x83, .name = "Buttons", .type = USB_IF_W_WATCHMAN1_BUTTONS}},
	 .magics = {MAGIC_CTOR(true, vive_magic_enable_lighthouse), MAGIC_CTOR(true, vive_magic_enable_lighthouse_more)}},
	{.vid = 0x28de,
	 .pid = 0x2102,
	 .type = USB_DEV_WATCHMAN1,
	 .name = "Knuckles",
	 .codename = "KN0",
	 .endpoints = {{.num = 0x81, .name = "IMU/Lightcap/Buttons", .type = USB_IF_WATCHMAN1}},
	 .magics = {MAGIC_CTOR(true, vive_magic_protocol_super_magic), MAGIC_CTOR(true, vive_magic_rf_raw_mode_0),
				MAGIC_CTOR(true, vive_magic_protocol_switch)}},
	{0}};

typedef struct SurviveUSBInterface SurviveUSBInterface;
typedef struct SurviveViveData SurviveViveData;

const char *survive_usb_interface_str(enum USB_IF_t interface) {
	switch (interface) {
	case USB_IF_HMD_HEADSET_INFO:
		return "USB_IF_HMD_HEADSET_INFO";
	case USB_IF_HMD_IMU:
		return "USB_IF_HMD_IMU";
	case USB_IF_WATCHMAN1:
		return "USB_IF_WATCHMAN1";
	case USB_IF_WATCHMAN2:
		return "USB_IF_WATCHMAN2";
	case USB_IF_TRACKER0_IMU:
		return "USB_IF_TRACKER0_IMU";
	case USB_IF_TRACKER_INFO:
		return "USB_IF_TRACKER_INFO";
	case USB_IF_TRACKER1_IMU:
		return "USB_IF_TRACKER1_IMU";
	case USB_IF_W_WATCHMAN1_IMU:
		return "USB_IF_W_WATCHMAN1_IMU";
	case USB_IF_HMD_LIGHTCAP:
		return "USB_IF_HMD_LIGHTCAP";
	case USB_IF_TRACKER0_LIGHTCAP:
		return "USB_IF_TRACKER0_LIGHTCAP";
	case USB_IF_TRACKER1_LIGHTCAP:
		return "USB_IF_TRACKER1_LIGHTCAP";
	case USB_IF_W_WATCHMAN1_LIGHTCAP:
		return "USB_IF_W_WATCHMAN1_LIGHTCAP";
	case USB_IF_HMD_BUTTONS:
		return "USB_IF_HMD_BUTTONS";
	case USB_IF_TRACKER0_BUTTONS:
		return "USB_IF_TRACKER0_BUTTONS";
	case USB_IF_TRACKER1_BUTTONS:
		return "USB_IF_TRACKER1_BUTTONS";
	case USB_IF_W_WATCHMAN1_BUTTONS:
		return "USB_IF_W_WATCHMAN1_BUTTONS";
	}
	return "UNKNOWN";
}

void survive_dump_buffer(SurviveContext *ctx, const uint8_t *data, size_t length) {
	int bytes_per_row = 32;
	for (size_t i = 0; i < length; i += bytes_per_row) {
		for (int j = 0; j < bytes_per_row; j++) {
			if (j > 0 && j % 4 == 0)
				ctx->printfproc(ctx, "  ");

			if (i + j < length) {
				ctx->printfproc(ctx, "%02x ", data[i + j]);
			} else {
				ctx->printfproc(ctx, "   ");
			}
		}

		ctx->printfproc(ctx, "    |    ");

		for (int j = 0; j < bytes_per_row; j++) {
			if (j > 0 && j % 4 == 0)
				ctx->printfproc(ctx, "  ");
			if (i + j < length) {
				uint8_t d = data[i + j];
				ctx->printfproc(ctx, "%c", d >= 32 && d < 127 ? d : '.');
			} else {
				ctx->printfproc(ctx, "   ");
			}
		}

		ctx->printfproc(ctx, "\n");
	}
}

struct SurviveUSBInfo {
	USBHANDLE handle;
	SurviveViveData *viveData;
	const struct DeviceInfo *device_info;
	struct SurviveObject *so;

	size_t interface_cnt;
	SurviveUSBInterface interfaces[MAX_INTERFACES_PER_DEVICE];

	enum LightcapMode {
		LightcapMode_unknown = 0,

		// This is the starting mode after an 0x40 0 0 0 0. Used for LH1
		LightcapMode_raw0,

		// This is the mode used for LH2
		LightcapMode_raw1,

		// Not sure what this is used for tbh
		LightcapMode_raw2
	} lightcapMode;

	size_t timeWithoutFlag;
	size_t packetsSeenWaitingForV2;

	bool tryConfigLoad;
};

struct SurviveViveData {
	SurviveContext *ctx;
	size_t udev_cnt;
	struct SurviveUSBInfo udev[MAX_USB_DEVS];
	struct libusb_context *usbctx;
	size_t read_count;
	int seconds_per_hz_output;

	int cnt_per_device_type[sizeof(KnownDeviceTypes) / sizeof(KnownDeviceTypes[0])];
	int hmd_mainboard_index;
	int hmd_imu_index;

	bool closing;
};

#ifdef HIDAPI
#include "driver_vive.hidapi.h"
#else
#include "driver_vive.libusb.h"
#endif

static inline int update_feature_report_async(USBHANDLE dev, uint16_t iface, uint8_t *data, int datalen);

static bool survive_device_is_rf(const struct DeviceInfo *device_info) {
	switch (device_info->type) {
	case USB_DEV_HMD:
	case USB_DEV_HMD_IMU_LH:
	case USB_DEV_W_WATCHMAN1:
	case USB_DEV_TRACKER0:
	case USB_DEV_TRACKER1:
		return false;
	}
	return true;
}

void vive_switch_mode(struct SurviveUSBInfo *driverInfo, enum LightcapMode lightcapMode) {
	SurviveContext *ctx = driverInfo->so->ctx;
	SurviveObject *w = driverInfo->so;
	if (driverInfo->timeWithoutFlag == 0) {
		driverInfo->timeWithoutFlag = 1;
		uint8_t buffer[9] = {};
		size_t buffer_length = 0;
		if (survive_device_is_rf(driverInfo->device_info)) {
			buffer[0] = VIVE_REPORT_COMMAND;
			buffer[1] = VIVE_COMMAND_CHANGE_PROTOCOL;
			buffer[2] = 6;

			buffer[3] = 1;
			buffer[4] = lightcapMode == LightcapMode_raw0 ? 0 : 1;
			buffer[5] = 0;
			buffer[6] = 2;
			buffer[7] = lightcapMode == LightcapMode_raw2 ? 1 : 0;
			buffer[8] = 0;
			buffer_length = 9;
		} else {
			buffer[0] = VIVE_REPORT_CHANGE_MODE;
			buffer[1] = (lightcapMode == LightcapMode_raw1) ? 1 : (lightcapMode == LightcapMode_raw2) ? 3 : 0;
			buffer_length = 5;
		}

		if (driverInfo->handle) {
			int r = update_feature_report_async(driverInfo->handle, 0, buffer, buffer_length);
			if (r != buffer_length) {
				SV_WARN("Could not send raw mode to %s (%d)", w->codename, r);
			}

			if (!survive_device_is_rf(driverInfo->device_info)) {
				r = update_feature_report_async(driverInfo->handle, 0, vive_magic_enable_lighthouse_more,
												sizeof(vive_magic_enable_lighthouse_more));
				if (r != buffer_length) {
					SV_WARN("Could not lighthouse more to %s (%d)", w->codename, r);
				}
			}

			SV_INFO("LightcapMode (%s) %d -> %d", w->codename, driverInfo->lightcapMode, lightcapMode);
			driverInfo->lightcapMode = lightcapMode;

		} else {
			static bool transfer_null_warning = false;
			if (!transfer_null_warning) {
				SV_WARN("Can't update the usb device %s out of raw 0 mode; dumping data", w->codename);
				transfer_null_warning = true;
			}
		}
	}
}

static bool is_mode_switch_usb(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
							   uint16_t length) {
	return bmRequestType == 0x21 && bRequest == 0x09 && wValue == 0x304 && length >= 8;
}

static bool is_mode_switch_rf(uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue, uint16_t wIndex,
							  const uint8_t *data, uint16_t length) {
	return bmRequestType == 0x21 && bRequest == 0x09 && wValue == 0x3ff && length >= 8 &&
		   data[1] == VIVE_COMMAND_CHANGE_PROTOCOL && data[2] >= 6;
}

void survive_data_on_setup_write(SurviveObject *so, uint8_t bmRequestType, uint8_t bRequest, uint16_t wValue,
								 uint16_t wIndex, const uint8_t *data, size_t length) {
	SurviveContext *ctx = so->ctx;
	struct SurviveUSBInfo *driverInfo = so->driver;
	driverInfo->timeWithoutFlag = 1;
	if (is_mode_switch_usb(bmRequestType, bRequest, wValue, wIndex, length)) {
		enum LightcapMode m = data[1] == 0 ? LightcapMode_raw0 : data[1] == 1 ? LightcapMode_raw1 : LightcapMode_raw2;
		SV_INFO("LightcapMode %d -> %d", driverInfo->lightcapMode, m);
		driverInfo->lightcapMode = m;
	}

	if (is_mode_switch_rf(bmRequestType, bRequest, wValue, wIndex, data, length)) {
		enum LightcapMode m = data[4] == 0 ? LightcapMode_raw0 : data[7] == 1 ? LightcapMode_raw2 : LightcapMode_raw1;
		SV_INFO("LightcapMode %d -> %d", driverInfo->lightcapMode, m);
		driverInfo->lightcapMode = m;
	}
}

void survive_data_cb_locked(SurviveUSBInterface *si);
void survive_data_cb(SurviveUSBInterface *si) {
	SurviveContext *ctx = si->ctx;
	survive_get_ctx_lock(ctx);
	survive_data_cb_locked(si);
	survive_release_ctx_lock(ctx);
}

// USB Subsystem
void survive_usb_close(SurviveViveData *t);
static int survive_usb_init(SurviveViveData *sv);
int survive_usb_poll(SurviveContext *ctx);
static int survive_get_config(char **config, SurviveViveData *ctx, struct SurviveUSBInfo *, int iface,
							  int send_extra_magic);
static int survive_vive_send_magic(SurviveContext *ctx, void *drv, int magic_code, void *data, int datalen);

static int AttachInterface(SurviveViveData *sv, struct SurviveUSBInfo *usbObject, const struct Endpoint_t *endpoint,
						   USBHANDLE devh, usb_callback cb) {
	SurviveContext *ctx = sv->ctx;
	size_t iface_cnt = usbObject->interface_cnt++;
	int which_interface_am_i = endpoint->type;
	const char *hname = endpoint->name;
	int endpoint_num = endpoint->num;

	SurviveUSBInterface *iface = &usbObject->interfaces[iface_cnt];
	SurviveObject *assocobj = usbObject->so;
	iface->ctx = ctx;
	iface->sv = sv;
	iface->which_interface_am_i = which_interface_am_i;
	iface->assoc_obj = usbObject->so;
	iface->usbInfo = usbObject;
	iface->hname = hname;
	iface->cb = cb;

#ifdef HIDAPI
	// What do here?
	iface->uh = usbObject->handle->interfaces[endpoint - usbObject->device_info->endpoints];
	assert(iface->uh);

#ifndef HID_NONBLOCKING
	iface->servicethread = OGCreateThread(HAPIReceiver, iface);
	OGUSleep(100000);
#else
	hid_set_nonblocking(iface->uh, 1);

	// Empty the queue out. If you don't, you might get stale data
	while (hid_read(iface->uh, iface->buffer, sizeof(iface->buffer)) > 0) {
	}
#endif
#else
	struct libusb_transfer *tx = iface->transfer = libusb_alloc_transfer(0);
	// printf( "%p %d %p %p\n", iface, which_interface_am_i, tx, devh );
	SV_VERBOSE(50, "Attaching %s(0x%x) for %s", hname, endpoint_num, assocobj ? assocobj->codename : "(unknown)");

	if (!iface->transfer) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Error: failed on libusb_alloc_transfer for %s", hname);
		return 4;
	}

	libusb_fill_interrupt_transfer(tx, devh, endpoint_num, iface->buffer, INTBUFFSIZE, handle_transfer, iface, 0);

	int rc = libusb_submit_transfer(tx);
	if (rc) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Error: Could not submit transfer for %s 0x%02x (Code %d, %s)", hname,
				 endpoint_num, rc, libusb_error_name(rc));
		return 6;
	}
#endif
	return 0;
}

static inline int hid_get_feature_report_timeout(USBHANDLE device, uint16_t iface, unsigned char *buf, size_t len) {
	int ret;
	uint8_t i = 0;
	for (i = 0; i < 50; i++) {
		ret = getupdate_feature_report(device, iface, buf, len);
		if (ret != -9 && (ret != -1 || errno != EPIPE))
			return ret;
		OGUSleep(1000);
	}

	return -1;
}

static inline int get_feature_report_timeout_locked(SurviveContext *ctx, USBHANDLE device, uint16_t iface,
													unsigned char *buf, size_t len) {
	survive_release_ctx_lock(ctx);
	int rtn = hid_get_feature_report_timeout(device, iface, buf, len);
	survive_get_ctx_lock(ctx);
	return rtn;
}

static int LoadConfig(SurviveViveData *sv, struct SurviveUSBInfo *usbInfo, int iface);
int survive_vive_add_usb_device(SurviveViveData *sv, survive_usb_device_t d) {
	SurviveContext *ctx = sv->ctx;
	const char *blacklist = survive_configs(ctx, "blacklist-devs", SC_GET, "-");
	for (const struct DeviceInfo *info = KnownDeviceTypes; info->name; info++) {
		if (info == 0 || strstr(blacklist, info->name)) {
			continue;
		}

		{
			uint16_t idVendor;
			uint16_t idProduct;
			int ret = survive_get_ids(d, &idVendor, &idProduct);

			if (ret < 0) {
				continue;
			}

			if (info->vid != idVendor || info->pid != idProduct) {
				continue;
			}

			if (info->type == USB_DEV_HMD) {
				if (sv->hmd_mainboard_index != -1) {
					continue;
				}
				sv->hmd_mainboard_index = sv->udev_cnt;
			} else if (info->type == USB_DEV_HMD_IMU_LH) {
				if (sv->hmd_imu_index != -1) {
					continue;
				}
				sv->hmd_imu_index = sv->udev_cnt;
			}

			struct SurviveUSBInfo *usbInfo = &sv->udev[sv->udev_cnt++];
			usbInfo->handle = 0;
			usbInfo->device_info = info;
			usbInfo->viveData = sv;
			ret = survive_open_usb_device(sv, d, usbInfo);

			if (ret) {
				SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT,
						 "Error: cannot open device \"%s\" with vid/pid %04x:%04x error %d (%s)", info->name, idVendor,
						 idProduct, ret, survive_usb_error_name(ret));
				sv->udev_cnt--;
				continue;
			}

			int *cnt = (sv->cnt_per_device_type + (usbInfo->device_info - KnownDeviceTypes));

			SV_VERBOSE(50, "Successfully enumerated %s %04x:%04x", info->name, idVendor, idProduct);

			if (usbInfo->device_info->codename[0] != 0) {
				char codename[4] = {0};
				strcpy(codename, usbInfo->device_info->codename);
				codename[2] += (*cnt);
				*cnt = *cnt + 1;

				SurviveObject *so = survive_create_device(ctx, "HTC", usbInfo, codename, 0);
				survive_add_object(ctx, so);
				usbInfo->so = so;
			}

			usbInfo->tryConfigLoad = 1;
			/*
			if (usbInfo->device_info->type != USB_DEV_HMD) {
				int hasError = LoadConfig(sv, usbInfo, 0);

				// Powered off devices are stripped of their SurviveObject
				if (hasError != 0 && usbInfo->so) {
					SV_INFO("%s config issue.", usbInfo->so->codename);
				}
			}
*/

			// There should only be one HMD, tie the mainboard interface to the surviveobject
			if (info->type == USB_DEV_HMD_IMU_LH || info->type == USB_DEV_HMD) {
				if (sv->hmd_imu_index != -1 && sv->hmd_mainboard_index != -1) {
					sv->udev[sv->hmd_mainboard_index].so = sv->udev[sv->hmd_imu_index].so;
				}
			}

			for (const struct Endpoint_t *endpoint = usbInfo->device_info->endpoints; endpoint->name; endpoint++) {
				int errorCode = AttachInterface(sv, usbInfo, endpoint, usbInfo->handle, survive_data_cb);
				if (errorCode < 0) {
					SV_WARN("Could not attach interface %s: %d", endpoint->name, errorCode);
				}
			}
		}
	}

	return 0;
}

int survive_usb_init(SurviveViveData *sv) {
	SurviveContext *ctx = sv->ctx;

	int r = survive_usb_subsystem_init(sv);
	if (r) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "usb fault %d (%s)\n", r, survive_usb_error_name(r));
		return r;
	}

	setup_hotplug(sv);
	survive_usb_devices_t devs;
	int ret = survive_get_usb_devices(sv, &devs);

	if (ret < 0) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Couldn't get list of USB devices %d (%s)", ret,
				 survive_usb_error_name(ret));
		return ret;
	}

	// Open all interfaces.
	survive_usb_device_enumerator e = 0;
	for (survive_usb_device_t d = 0; (d = get_next_device(&e, devs)) && sv->udev_cnt < MAX_USB_DEVS;) {
		survive_vive_add_usb_device(sv, d);
	}
	survive_free_usb_devices(devs);

	SV_INFO("All enumerated devices attached.");

	survive_vive_send_magic(ctx, sv, 1, 0, 0);

	// libUSB initialized.  Continue.
	return 0;
}

int survive_vive_send_magic(SurviveContext *ctx, void *drv, int magic_code, void *data, int datalen) {
	int r;
	SurviveViveData *sv = drv;

	for (int i = 0; i < sv->udev_cnt; i++) {
		struct SurviveUSBInfo *usbInfo = &sv->udev[i];
		usbInfo->lightcapMode = LightcapMode_raw0;
		for (const struct Magic_t *magic = usbInfo->device_info->magics; magic->magic; magic++) {
			if (magic->code == magic_code) {
				uint8_t *data = alloca(sizeof(uint8_t) * magic->length);
				memcpy(data, magic->magic, magic->length);

				survive_release_ctx_lock(ctx);
				r = update_feature_report(usbInfo->handle, 0, data, magic->length);
				survive_get_ctx_lock(ctx);

				if (r != magic->length && usbInfo->so)
					SV_WARN("Could not turn on %s(%d) (%d/%zu - %s)", usbInfo->so->codename, usbInfo->device_info->type,
							r, magic->length, survive_usb_error_name(r));
			}
		}
	}

	SV_INFO("Powered unit on.");

	return 0;
}

int survive_vive_send_haptic(SurviveObject *so, uint8_t reserved, uint16_t pulseHigh, uint16_t pulseLow,
							 uint16_t repeatCount) {
	SurviveViveData *sv = ((struct SurviveUSBInfo *)so->driver)->viveData;
	SurviveContext *ctx = so->ctx;

	if (NULL == sv) {
		return -500;
	}

	int r;
	uint8_t vive_controller_haptic_pulse[64] = {
		VIVE_REPORT_COMMAND,
		VIVE_COMMAND_HAPTIC_PULSE,
		0x07,
		0x00,
		pulseHigh & 0xff00 >> 8,
		pulseHigh & 0xff,
		pulseLow & 0xff00 >> 8,
		pulseLow & 0xff,
		repeatCount & 0xff00 >> 8,
		repeatCount & 0xff,
	};

	for (int i = 0; i < sv->udev_cnt; i++) {
		struct SurviveUSBInfo *usbInfo = &sv->udev[i];

		if (usbInfo->so == so) {
			r = update_feature_report(usbInfo->handle, 0, vive_controller_haptic_pulse,
									  sizeof(vive_controller_haptic_pulse));
			r = getupdate_feature_report(usbInfo->handle, 0, vive_controller_haptic_pulse,
										 sizeof(vive_controller_haptic_pulse));

			if (r != sizeof(vive_controller_haptic_pulse)) {
				SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "HAPTIC FAILED **************************\n");
				return -1;
			}

			return 0;
		}
	}

	return -2;
}

void survive_vive_usb_close(SurviveViveData *sv) {
	survive_release_ctx_lock(sv->ctx);
	survive_usb_close(sv);
	survive_get_ctx_lock(sv->ctx);
}

STATIC_CONFIG_ITEM(SECONDS_PER_HZ_OUTPUT, "usb-hz-output", 'i', "Seconds between outputing usb stats", -1);
int survive_vive_usb_poll(SurviveContext *ctx, void *v) {
	SurviveViveData *sv = v;
	sv->read_count++;

	static double start = 0;
	static int seconds = 0;
	if(start == 0)
		start = OGGetAbsoluteTime();

	double now = OGGetAbsoluteTime();
	int now_seconds = (int)(now - start);
	bool print = sv->seconds_per_hz_output > 0 && now_seconds > (seconds + sv->seconds_per_hz_output);
	
	if (print) {
		seconds = now_seconds;
		size_t total_packets = 0;
		for (int i = 0; i < sv->udev_cnt; i++) {
			if (sv->udev[i].so == 0)
				continue;

			for (int j = 0; j < sv->udev[i].interface_cnt; j++) {
				SurviveUSBInterface *iface = &sv->udev[i].interfaces[j];
				total_packets += iface->packet_count;
				SV_INFO("Iface %s %8s has %4zu packets (%6.2f hz)", iface->assoc_obj->codename, iface->hname,
						iface->packet_count, iface->packet_count / (now - start));
				iface->packet_count = 0;
			}
		}

		SV_INFO("Total                  %4zu packets (%6.2f hz)", total_packets, total_packets / (now - start));
		start = now;
	}

	for (int i = 0; i < sv->udev_cnt; i++) {
		struct SurviveUSBInfo *usbInfo = &sv->udev[i];

		if (usbInfo->tryConfigLoad) {
			int err = LoadConfig(sv, usbInfo, 0);
			if (err == 0) {
				for (const struct Magic_t *magic = usbInfo->device_info->magics; magic->magic; magic++) {
					if (magic->code == 1) {
						uint8_t *data = alloca(sizeof(uint8_t) * magic->length);
						memcpy(data, magic->magic, magic->length);

						survive_release_ctx_lock(ctx);
						int r = update_feature_report(usbInfo->handle, 0, data, magic->length);
						survive_get_ctx_lock(ctx);

						if (r != magic->length && usbInfo->so)
							SV_WARN("Could not turn on %s(%d) (%d/%zu - %s)", usbInfo->so->codename,
									usbInfo->device_info->type, r, magic->length, survive_usb_error_name(r));
					}
				}
			}
			usbInfo->tryConfigLoad = 0;
		}
	}

#ifdef HIDAPI
#ifdef HID_NONBLOCKING
	survive_release_ctx_lock(ctx);
	for (int i = 0; i < sv->udev_cnt; i++) {
		for (int j = 0; j < sv->udev[i].interface_cnt; j++) {
			SurviveUSBInterface* iface = &sv->udev[i].interfaces[j];
			if (iface->assoc_obj)
				HAPIReceiver(iface);
		}
	}
	survive_get_ctx_lock(ctx);
#else
	OGUSleep(1);
	return 0;
#endif
#else
	// int r = libusb_handle_events(sv->usbctx);
	struct timeval tv = {.tv_usec = 10 * 1000};
	survive_release_ctx_lock(ctx);
	int r = libusb_handle_events_timeout(sv->usbctx, &tv);
	survive_get_ctx_lock(ctx);

	if (r) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Libusb poll failed. %d (%s)", r, libusb_error_name(r));
	}
#endif
	return 0;
}

static inline survive_timecode fix_time24(survive_timecode time24, survive_timecode refTime) {
	survive_timecode upper_ref = refTime & 0xFF000000u;
	survive_timecode lower_ref = refTime & 0x00FFFFFFu;

	if (lower_ref > time24 && lower_ref - time24 > (1 << 23u)) {
		upper_ref += 0x01000000;
	} else if (lower_ref < time24 && time24 - lower_ref > (1 << 23u) && upper_ref > 0) {
		upper_ref -= 0x01000000;
	}

	return upper_ref | time24;
}

static int survive_get_config(char **config, SurviveViveData *sv, struct SurviveUSBInfo *usbInfo, int iface,
							  int send_extra_magic) {
	SurviveContext *ctx = sv->ctx;
	int ret, count = 0, size = 0;
	uint8_t cfgbuff[256] = {0};
	uint8_t compressed_data[8192] = {0};
	uint8_t uncompressed_data[65536] = {0};
	USBHANDLE dev = usbInfo->handle;

	const char *name = usbInfo->device_info->name;

	if (send_extra_magic) {
		uint8_t cfgbuffwide[257] = {0};

		memset(cfgbuffwide, 0, sizeof(cfgbuff));
		cfgbuffwide[0] = 0x01;
		ret = get_feature_report_timeout_locked(ctx, dev, iface, cfgbuffwide, sizeof(cfgbuffwide));
		OGUSleep(1000);

		int k;

		uint8_t cfgbuff_send[64] = {VIVE_REPORT_COMMAND, 0x83};

		// Switch mode to pull config?
		for (k = 0; k < 10; k++) {
			OGUSleep(1000);
		}

		cfgbuffwide[0] = VIVE_REPORT_COMMAND;
		ret = get_feature_report_timeout_locked(ctx, dev, iface, cfgbuffwide, sizeof(cfgbuffwide));
		OGUSleep(1000);
	}

	// Send Report 16 to prepare the device for reading config info
	memset(cfgbuff, 0, sizeof(cfgbuff));
	cfgbuff[0] = VIVE_REPORT_CONFIG_READMODE;
	if ((ret = get_feature_report_timeout_locked(ctx, dev, iface, cfgbuff, sizeof(cfgbuff))) < 0) {
		if (usbInfo->device_info->type == USB_DEV_WATCHMAN1) {
			SV_INFO("%s couldn't configure; probably turned off %d %s", name, ret, survive_usb_error_name(ret));
		} else {
			SV_WARN("Could not get survive config data for device %s:%d", usbInfo->device_info->name, iface);
		}
		return -1;
	}

	OGUSleep(100000);

	// Now do a bunch of Report 17 until there are no bytes left
	cfgbuff[0] = VIVE_REPORT_CONFIG_READ;
	cfgbuff[1] = 0xaa;
	do {
		if ((ret = get_feature_report_timeout_locked(ctx, dev, iface, cfgbuff, sizeof(cfgbuff))) < 0) {
			SV_INFO("Could not read config data (after first packet) on device %s:%d (count: %d)",
					usbInfo->device_info->name, iface, count);
			return -2;
		}

		size = cfgbuff[1];

		if (!size)
			break;

		if (size > (sizeof(cfgbuff) - 2)) {
			SV_INFO("Too much data (%d) on packet from config for device %s:%d (count: %d)", size, name, iface, count);
			return -3;
		}

		if (count + size >= sizeof(compressed_data)) {
			SV_INFO("Configuration length too long %s:%d (count: %d)", name, iface, count);
			return -4;
		}

		// Some (Tracker at least?) devices send a uint64_t before data; not sure what it means but skip it for now.
		if (count == 0 && size >= 2 && cfgbuff[2] != 0x78) {
			continue;
		}

		memcpy(&compressed_data[count], cfgbuff + 2, size);
		count += size;
	} while (1);

	if (count == 0) {
		SV_INFO("Empty configuration for %s:%d", name, iface);
		return -5;
	}

	SV_VERBOSE(50, "Got config data length %d", count);

	int len = survive_simple_inflate(ctx, compressed_data, count, uncompressed_data, sizeof(uncompressed_data) - 1);
	if (len <= 0) {
		SV_INFO("Error: data for config descriptor %s:%d is bad. (%d)", name, iface, len);
		return -5;
	}

	*config = SV_MALLOC(len + 1);
	memcpy(*config, uncompressed_data, len);

	return len;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define POP1 (*(readdata++))

#ifndef _MSC_VER
struct __attribute__((__packed__)) unaligned_16_t {
	int16_t v;
};
struct __attribute__((__packed__)) unaligned_32_t {
	int32_t v;
};
struct __attribute__((__packed__)) unaligned_u16_t {
	uint16_t v;
};
struct __attribute__((__packed__)) unaligned_u32_t {
	uint32_t v;
};
#else
struct unaligned_16_t {
	int16_t v;
};
struct unaligned_32_t {
	int32_t v;
};
struct unaligned_u16_t {
	uint16_t v;
};
struct unaligned_u32_t {
	uint32_t v;
};
#endif

#define POP2 (((((struct unaligned_u16_t *)((readdata += 2) - 2))))->v)
#define POP4 (((((struct unaligned_u32_t *)((readdata += 4) - 4))))->v)

typedef struct {
	// could use a bitfield here, but since this data is short-lived,
	// the space savings probably isn't worth the processing overhead.
	uint8_t pressedButtonsValid;
	uint8_t triggerOfBatteryValid;
	uint8_t batteryChargeValid;
	uint8_t hardwareIdValid;
	uint8_t touchpadHorizontalValid;
	uint8_t touchpadVerticalValid;
	uint8_t triggerHighResValid;

	uint32_t pressedButtons;
	uint16_t triggerOrBattery;
	uint8_t batteryCharge;
	uint32_t hardwareId;
	uint16_t touchpadHorizontal;
	uint16_t touchpadVertical;
	uint16_t triggerHighRes;
} buttonEvent;

void incrementAndPostButtonQueue(SurviveContext *ctx) {
	ButtonQueueEntry *entry = &(ctx->buttonQueue.entry[ctx->buttonQueue.nextWriteIndex]);

	if ((ctx->buttonQueue.nextWriteIndex + 1) % BUTTON_QUEUE_MAX_LEN == ctx->buttonQueue.nextReadIndex) {
		// There's not enough space to write this entry.  Clear it out and move along
		// printf("Button Buffer Full\n");
		memset(entry, 0, sizeof(ButtonQueueEntry));
		return;
	}
	entry->isPopulated = 1;
	ctx->buttonQueue.nextWriteIndex++;
	// if we've exceeded the size of the buffer, loop around to the beginning.
	if (ctx->buttonQueue.nextWriteIndex >= BUTTON_QUEUE_MAX_LEN) {
		ctx->buttonQueue.nextWriteIndex = 0;
	}
	OGUnlockSema(ctx->buttonQueue.buttonservicesem);

	// clear out any old data in the entry so we always start with a clean slate.
	entry = &(ctx->buttonQueue.entry[ctx->buttonQueue.nextWriteIndex]);
	memset(entry, 0, sizeof(ButtonQueueEntry));
}

static ButtonQueueEntry *prepareNextButtonEvent(SurviveObject *so) {
	ButtonQueueEntry *entry = &(so->ctx->buttonQueue.entry[so->ctx->buttonQueue.nextWriteIndex]);
	memset(entry, 0, sizeof(ButtonQueueEntry));
	assert(so);
	entry->so = so;
	return entry;
}

// important!  This must be the only place that we're posting to the buttonEntryQueue
// if that ever needs to be changed, you will have to add locking so that only one
// thread is posting at a time.
static void registerButtonEvent(SurviveObject *so, buttonEvent *event) {
	ButtonQueueEntry *entry = prepareNextButtonEvent(so);

	if (event->pressedButtonsValid) {
		// printf("trigger %8.8x\n", event->triggerHighRes);
		for (int a = 0; a < 16; a++) {
			if (((event->pressedButtons) & (1 << a)) != ((so->buttonmask) & (1 << a))) {
				// Hey, the button did something
				if (event->pressedButtons & (1 << a)) {
					// it went down
					entry->eventType = BUTTON_EVENT_BUTTON_DOWN;
				} else {
					// it went up
					entry->eventType = BUTTON_EVENT_BUTTON_UP;
				}
				entry->buttonId = a;
				if (entry->buttonId == 0) {
					// this fixes 2 issues.  First, is the a button id of 0 indicates no button pressed.
					// second is that the trigger shows up as button 0 coming from the wireless controller,
					// but we infer it from the position on the wired controller.  On the wired, we treat it
					// as buttonId 24 (look further down in this function)
					entry->buttonId = 24;
				}
				incrementAndPostButtonQueue(so->ctx);
				entry = prepareNextButtonEvent(so);
			}
		}
		// if the trigger button is depressed & it wasn't before
		if ((((event->pressedButtons) & (0xff000000)) == 0xff000000) &&
			((so->buttonmask) & (0xff000000)) != 0xff000000) {
			entry->eventType = BUTTON_EVENT_BUTTON_DOWN;
			entry->buttonId = 24;
			incrementAndPostButtonQueue(so->ctx);
			entry = prepareNextButtonEvent(so);
		}
		// if the trigger button isn't depressed but it was before
		else if ((((event->pressedButtons) & (0xff000000)) != 0xff000000) &&
				 ((so->buttonmask) & (0xff000000)) == 0xff000000) {
			entry->eventType = BUTTON_EVENT_BUTTON_UP;
			entry->buttonId = 24;
			incrementAndPostButtonQueue(so->ctx);
			entry = prepareNextButtonEvent(so);
		}
	}
	if (event->triggerHighResValid) {
		if (so->axis1 != event->triggerHighRes) {
			entry->eventType = BUTTON_EVENT_AXIS_CHANGED;
			entry->axis1Id = 1;
			entry->axis1Val = event->triggerHighRes;
			incrementAndPostButtonQueue(so->ctx);
			entry = prepareNextButtonEvent(so);
		}
	}
	if ((event->touchpadHorizontalValid) && (event->touchpadVerticalValid)) {
		if ((so->axis2 != event->touchpadHorizontal) || (so->axis3 != event->touchpadVertical)) {
			entry->eventType = BUTTON_EVENT_AXIS_CHANGED;
			entry->axis1Id = 2;
			entry->axis1Val = event->touchpadHorizontal;
			entry->axis2Id = 3;
			entry->axis2Val = event->touchpadVertical;
			incrementAndPostButtonQueue(so->ctx);
			entry = prepareNextButtonEvent(so);
		}
	}

	if (event->pressedButtonsValid) {
		so->buttonmask = event->pressedButtons;
	}
	if (event->batteryChargeValid) {
		so->charge = event->batteryCharge;
	}
	if (event->touchpadHorizontalValid) {
		so->axis2 = event->touchpadHorizontal;
	}
	if (event->touchpadVerticalValid) {
		so->axis3 = event->touchpadVertical;
	}
	if (event->triggerHighResValid) {
		so->axis1 = event->triggerHighRes;
	}
}

#define FAILURE_ON_FALSE(x)                                                                                            \
	if (!(x)) {                                                                                                        \
		SV_WARN("Assert failure for parsing watchman: " #x);                                                           \
		goto failure;                                                                                                  \
	};

//#define DEBUG_WATCHMAN
#define DEBUG_WATCHMAN_ERRORS
#ifdef DEBUG_WATCHMAN
#define DEBUG_WATCHMAN_ERRORS
#endif

#define AS_SHORT(a, b) ((uint16_t)(((uint16_t)a) << 8) | (0x00ff & b))
#define POP_BYTE(ptr) ((uint8_t) * ((ptr)++))
#define POP_SHORT(ptr) (((((struct unaligned_u16_t *)(((ptr) += 2) - 2))))->v)

#define HAS_FLAG(flags, flag) ((flags & (flag)) == (flag))

char hexstr[512];
static char *packetToHex(uint8_t *packet, uint8_t *packetEnd) {
	int count = packetEnd - packet;
	int i;
	for (i = 0; i < count; i++)
		sprintf(&hexstr[i * 3], "%02x ", packet[i]);
	hexstr[i * 3] = 0;

	return hexstr;
}

char bin[9] = {};
static char *byteToBin(uint8_t b) {
	for (int i = 0; i < 8; i++) {
		bin[i] = ((b >> (7 - i)) & 1) ? '1' : '0';
	}
	bin[8] = 0;
	return bin;
}

struct sensorData {
	uint8_t sensorId;
	uint8_t edgeCount;
};

static int32_t read_light_data(SurviveObject *w, uint16_t time, uint8_t **readPtr, uint8_t *payloadEndPtr,
							   LightcapElement *output, int output_cnt) {
	uint8_t *payloadPtr = *readPtr;
	SurviveContext *ctx = w->ctx;
	uint32_t reference_time = w->activations.last_imu;

	bool isGen2 = 0; // w->ctx->lh_version == 1;
	(void)isGen2;

	if (payloadEndPtr - payloadPtr <= 3) {
		return 0;
	}

	// DEBUG
	if ((*payloadPtr & 0xE0) == 0xE0) {
		SV_WARN("Light contains probable non-light data : 0x%02hX [Time:%04hX] [Payload: %s]", *payloadPtr, time,
				packetToHex(payloadPtr, payloadEndPtr));
	}

	/*
	 * ---=== LIGHT DATA STRUCTURE ===---
	 *
	 * | SensorData  | Time Deltas          | End Timestamp |
	 * ╔═════════════╦══════════════════════╦═══════════════╗
	 * ║ SS SS .. SS ║ DD DD DD DD DD .. DD ║ TT TT TT      ║
	 * ╚═════════════╩══════════════════════╩═══════════════╝
	 *
	 * Three parts to the packet, the sensor data which contains which sensors were triggered, and the times
	 * deltas between rising and falling of the sensor event. There are always two rising/falling events per
	 * sensor though the ordering is not simple as new sensor events may start before others are finished.
	 *
	 * The meaning and associated led with each 'event' is determined by the edge count as encoded within the
	 * sensor data (see below)
	 *
	 * The time deltas use variable length encoding, so we can't determine how many sensors are in the packet
	 * just from the packet length. However, we do know that there are always two times per sensor (rise and
	 * fall), so there are (2*Sensor)-1 deltas in the packet (-1 because the end time is 'known' yielding two
	 * times). Therefore, the general read process is thus:
	 *
	 *  1) Read off timestamp
	 *  2) Read the first byte from the start of the packet
	 *  3) Read one time delta from the end of the packet (We get two times from this since we know the 'end' time)
	 *  4) Repeatedly:
	 *     a) Read one byte from the start of the packet (Led/Flag)
	 *     b) Read two time deltas from the end of the packet (not including timestamp) [See encoding below]
	 *     c) Stop once we've read all data in the packet
	 *
	 *
	 * TT TT TT
	 * ~~~~~~~~
	 *  Timestamp of the last event [Little Endian]
	 *
	 * eg:
	 *  f6 b4 5b = 6010102
	 *
	 * DD
	 * ~~
	 *  Time deltas between events stored as variable length sequences. The lower 7 bits of each byte are
	 *  summed until a byte with the 8th bit set is encountered. [Little endian]
	 *
	 *   8 76543210
	 *  ╔═╦════════╗
	 *  ║S│Value   ║
	 *  ╚═╩════════╝
	 *    ╲  ╲_________ 7 Bits of time delta
	 *     ╲___________ Stop bit (1 = Value complete, 0 = Continue reading)
	 *
	 *  eg:
	 *      0 = 80       [(80 & 7F)                  = 0]
	 *    127 = FF       [(FF & 7F)                  = 127]
	 *    128 = 80 01    [(80 & 7F) + ((01 & 7F)<<7) = 128]
	 *    255 = FF 01    [(FF & 7F) + ((01 & 7F)<<7) = 255]
	 *    256 = 80 02    [(80 & 7F) + ((02 & 7F)<<7) = 256]
	 *  16383 = FF 7F    [(FF & 7F) + ((7F & 7F)<<7) = 16383]
	 *  16384 = 80 80 01 [(80 & 7F) + ((80 & 7F)<<7) + ((01 & 7F)<<14) = 16384]
	 *
	 *
	 *
	 * SS
	 * ~~
	 *  Packed data about which sensor was detected and how many time deltas it's associated event straddles.
	 *  1 Byte per sensor
	 *
	 *   876543 210
	 *  ╔══════╦═══╗
	 *  ║Sensor│EC ║
	 *  ╚══════╩═══╝
	 *    ╲      ╲____ Edge count
	 *     ╲__________ Sensor ID of the event
	 *
	 *  eg:
	 *    2B = Sensor 5, 3 edges [2B>>3 = 5, 2B & 03 = 3]
	 *
	 *
	 * Example full packet
	 * ===================
	 *
	 *   ┌──────┬───────┐
	 *   │Sensor│ Edges │
	 *   ├──────┼───────┤
	 *   │   5  │   3   │
	 *   │  10  │   1   │
	 *   │   9  │   2   │
	 *   │   5  │   0   │
	 *   │  12  │   0   │
	 *   └──────┴───────┘                           End Time : 6010102
	 *              ╲                               ╱
	 *               ╲                             ╱
	 *            ╔════════════════╦═┄┄┄┄┄┄┄┄┄┄┄═╦══════════╗
	 *            ║ 2b 51 4a 28 60 ║ Time Deltas ║ f6 b4 5b ║
	 *            ╚════════════════╩═┄┄┄┄┄┄┄┄┄┄┄═╩══════════╝
	 *                             ╱              ╲
	 *     _______________________╱                ╲_______________________
	 *    ╱                                                                ╲
	 *   ╱                                                                  ╲
	 *  ╔═══════╤═══════╤═══════╤══════════╤═══════╤════╤════╤═══════╤═══════╗
	 *  ║ c7 2e │ e6 66 │ 84 1f │ fb 31 0b │ d9 01 │ da │ ca │ db 02 │ e4 02 ║
	 *  ╠═══════╪═══════╪═══════╪══════════╪═══════╪════╪════╪═══════╪═══════╣
	 *  ║ 5959  | 13158 | 3972  | 186619   | 217   | 90 | 74 | 347   |356    ║
	 *  ╚═══════╧═══════╧═══════╧══════════╧═══════╧════╧════╧═══════╧═══════╝
	 *  │       │       │       │          │       │    │    │       │       │
	 *  ┕━━━━━━«E       │       │          │       │    │    │       │       │ -> Led 12 : 5799310 -> 5805269
	 *                  ┕━━━━━━«D          │       │    │    │       │       │ -> Led 5  : 5818427 -> 5822399
	 *                                     ┕━━━━━━━2━━━━2━━━«C       │       │ -> Led 9  : 6009018 -> 6009399
	 *                                             │    │    ┊       │       │
	 *                                             ┕━━━━3━━━━3━━━━━━━3━━━━━━«A -> Led 5  : 6009235 -> 6010102
	 *                                                  |    ┊       |
	 *                                                  ┕━━━━1━━━━━━«B         -> Led 10 : 6009325 -> 6009746
	 * Read order :
	 *  A : Ends at 'A' - Skip 3 edges to find start
	 *  B : Ends at 'B' - Skip 1 edge to find start
	 *  C : Ends at 'C' - Skip 2 edges to find start
	 *  D : Ends at 'D' (Since the 'end' edges from ABC have already been 'used'), ends at next edge
	 *  E : Ends at 'E' - Ends at next edge
	 */

	// Step 1 - Extract deltas between events the corresponding sensors
	size_t timeIndex = 0;
	uint32_t times[16] = {0};
	size_t maxTimeIndex = sizeof(times) / sizeof(times[0]);
	size_t maxEvents = maxTimeIndex >> 1;
	uint8_t *reportOrder = alloca(sizeof(uint8_t*) * maxTimeIndex);

	struct sensorData* sensors = alloca(sizeof(struct sensorData) * maxEvents);

	uint8_t *idsPtr = payloadPtr;
	uint8_t *eventPtr = payloadEndPtr;

	// Last three bytes of light data are the LSB of the time of the last event
	eventPtr -= 4;
	uint32_t lastEventTime =
		((uint32_t)(time >> 8) << 24) | (eventPtr[3] << 16) | (eventPtr[2] << 8) | (eventPtr[1] << 0);

	// The general issue is that the 'time1' field isn't super in sync with the last 3 bytes we use for timing
	// light events -- it can tip a smidge before those bytes see it or sometimes after. This can cause
	// wild 1<<24 tick differences which break everything.
	//
	// We base it off IMU as a reference because light events can get blocked and it's not out of the ordinary
	// to not see them for a second or two. (1 << 23) on a 48mhz clock is ~150ms; and the IMU is consistently
	// much faster than that.
	if (lastEventTime > reference_time && lastEventTime - reference_time > (1 << 23)) {
		lastEventTime -= (1 << 24);
	} else if (reference_time > lastEventTime && reference_time - lastEventTime > (1 << 23)) {
		lastEventTime += (1 << 24);
	}

	times[0] = lastEventTime;
	SV_VERBOSE(200, "Packet Start Time: %u (0x%x) (ref: %u) Payload: %s", lastEventTime, lastEventTime, reference_time,
			   packetToHex(payloadPtr, payloadEndPtr));

	while (idsPtr + (timeIndex >> 1u) < eventPtr) {
		// Obtain the timing to the previous event
		// Variable length encoding [if bit 8 is 0, continue into next byte]
		uint32_t timeDelta = 0;
		uint8_t *eventPtrStart = eventPtr;
		while (true) {
			timeDelta <<= 7;
			timeDelta |= (*eventPtr & 0x7F);
			if (((*(eventPtr--)) & 0x80) == 0x80)
				break;

			if (idsPtr + (timeIndex >> 1u) > eventPtr) {
				eventPtr = eventPtrStart;
				goto exit_while;
			}
		}
		lastEventTime -= timeDelta;
		if (timeIndex >= 16) {
			return -8;
		}
		// Store the event time
		times[++timeIndex] = lastEventTime;
		SV_VERBOSE(200, "Time: [%zd] %u (%u) %s", timeIndex, lastEventTime, timeDelta,
				   packetToHex(eventPtr + 1, eventPtrStart + 1));
	}

exit_while:
	if (timeIndex % 2 == 0 && timeIndex > 0) {
		timeIndex--;
		do {
			eventPtr++;
		} while ((*eventPtr & 0x80) == 0);
	}
	assert(timeIndex % 2 == 1);

	size_t sensor_byte_cnt = eventPtr - idsPtr + 1;
	if (sensor_byte_cnt > ((timeIndex >> 1) + 1)) {
		isGen2 = true;
		//	    SV_VERBOSE(100, "Likely gen2 data; %x %d %d %d", *idsPtr, sensor_byte_cnt, (timeIndex >> 1), idsPtr -
		//(eventPtr - (timeIndex >> 1)));
		idsPtr = eventPtr - (timeIndex >> 1);
	}

	// There are two time deltas per 'event'
	for (int i = 0; i < (timeIndex >> 1) + 1; i++) {
		sensors[i].sensorId = ((*idsPtr) >> 3) & 0x1F;
		sensors[i].edgeCount = (*idsPtr) & 0x7;
		SV_VERBOSE(200, "Sensor: %2d Edge: %d (%02x)", sensors[i].sensorId, sensors[i].edgeCount, (*idsPtr));
		idsPtr++;
	}

	// Step 2 - Convert events to pulses
	LightcapElement* les = alloca(sizeof(LightcapElement) * maxEvents);
	size_t eventCount = (timeIndex + 1) >> 1; // timeIndex>>1 = There are always twice as many time events as sensors

	memset(les, 0, maxEvents * sizeof(LightcapElement));
	memset(reportOrder, 0, maxTimeIndex * sizeof(uint8_t));
	timeIndex = -1;

	for (int i = 0; i < eventCount; i++) {
		// Get the end time (Increment and find the next 'unused' time)
		while (times[++timeIndex] == 0)
			if (timeIndex + 1 >= maxTimeIndex) {
				return -2;
			}
		if (timeIndex >= maxTimeIndex) {
			return -3;
		}

		// Get the start time
		size_t startTimeIndex = timeIndex + (sensors[i].edgeCount + 1);
		if (startTimeIndex >= maxTimeIndex) {
			return -4;
		}

		// Store the start index so we can return in ascending time order
		if (reportOrder[startTimeIndex] != 0) {
			return -5;
		}
		reportOrder[startTimeIndex] = i + 1;
		LightcapElement *le = &les[i];

		// Fill in the LightcapElement data
		assert(le->sensor_id == 0);

		le->sensor_id = sensors[i].sensorId;
		le->timestamp = times[startTimeIndex];
		le->length = times[timeIndex] - times[startTimeIndex];

		// Flag the start time as 'used'
		times[startTimeIndex] = 0;

		// SV_INFO("Light Event : [%i|%i] %li -> %li (%li) [%i-%i]", les[i].sensor_id, sensors[i].edgeCount,
		// les[i].timestamp, les[i].timestamp + les[i].length, les[i].length, startTimeIndex, timeIndex);
	}

	// Output the events in ascending time order
	uint8_t orderedIndex;
	for (int i = 0; (i < maxTimeIndex) && output_cnt > 0; i++) {
		if ((orderedIndex = reportOrder[i]) != 0) {
			LightcapElement *ol = &les[orderedIndex - 1];

			if (!(ol->length != 0 || ol->timestamp != 0)) {
				return -6;
			}

			*(output++) = *ol;
			output_cnt--;
			SV_VERBOSE(500, "Light Event [Ordered]: %i [%2i] %u -> %u (%4hu)", i, ol->sensor_id, ol->timestamp,
					   ol->timestamp + ol->length, ol->length);
		}
	}

	return eventCount;
}

static bool read_imu_data(SurviveObject *w, uint16_t time, uint8_t **readPtr, uint8_t *payloadEndPtr) {
	uint8_t *payloadPtr = *readPtr;

	SurviveContext *ctx = w->ctx;
	if (payloadEndPtr - payloadPtr < 7) {
		return false;
	}

	// First byte is higher res time, followed by 6 shorts
	uint8_t timeLSB = POP_BYTE(payloadPtr);
	int16_t aX = POP_SHORT(payloadPtr);
	int16_t aY = POP_SHORT(payloadPtr);
	int16_t aZ = POP_SHORT(payloadPtr);
	int16_t rX = POP_SHORT(payloadPtr);
	int16_t rY = POP_SHORT(payloadPtr);
	int16_t rZ = POP_SHORT(payloadPtr);

	FLT agm[9] = {aX, aY, aZ, rX, rY, rZ};

	SV_VERBOSE(200, "%s IMU: %d " Point3_format " " Point3_format " From: %s", w->codename, timeLSB,
			   LINMATH_VEC3_EXPAND(agm), LINMATH_VEC3_EXPAND(agm + 3), packetToHex(*readPtr, payloadPtr));
	w->ctx->raw_imuproc(w, 3, agm, ((uint32_t)time << 16) | (timeLSB << 8), 0);

	*readPtr = payloadPtr;

	return true;
}
#define UPDATE_PTR_AND_RETURN                                                                                          \
	*readPtr = payloadPtr;                                                                                             \
	return true;

static bool read_event(SurviveObject *w, uint16_t time, uint8_t **readPtr, uint8_t *payloadEndPtr) {
	uint8_t *payloadPtr = *readPtr;
	SurviveContext *ctx = w->ctx;

	// If we're looking at light data, return
	if (!HAS_FLAG(*payloadPtr, 0xE0))
		return true;

	/*
	 * Event Flags
	 * ===========
	 *
	 * If input (button, touch, motion) data is present in packet:
	 *   ┄╦═╤═╤═╦═╤═╤═╤═╤═╦┄
	 *    ║1│1│1│1│I│-│-│-║
	 *   ┄╩═╧═╧═╩═╧═╧═╧═╧═╩┄
	 *
	 * If battery data is present in a packet:
	 *   ┄╦═╤═╤═╦═╤═╤═╤═╤═╦═══════════════╦┄
	 *    ║1│1│1│0│?│?│?│1║ Battery       ║
	 *   ┄╩═╧═╧═╩═╧═╧═╧═╧═╩═══════════════╩┄
	 *    ▲                               │
	 *    ╰───────────────────────────────╯
	 *
	 * If battery data is not present in packet (EG IMU only packets):
	 *   ┄╦═╤═╤═╦═╤═╤═╤═╤═╦┄
	 *    ║1│1│1│0│I│?│?│0║
	 *   ┄╩═╧═╧═╩═╧═╧═╧═╧═╩┄
	 *
	 * I: IMU Data      1 = IMU Data present after event (13 Bytes)
	 *                  0 = No IMU Data present after event
	 */

	const uint8_t flags = POP_BYTE(payloadPtr);

	bool flagInput = HAS_FLAG(flags, 0x10);
	bool flagIMU = HAS_FLAG(flags, 0x08);
	if (flagInput) {
		/*
		 * Flags for input events are as follows:
		 *
		 * ┄╦═╤═╤═╦═╤═╤═╤═╤═╦┄
		 *  │1│1│1│1│-│t│m│b│
		 * ┄╩═╧═╧═╩═╧═╧═╧═╧═╩┄
		 *
		 * t: Trigger    1 = Trigger data present in event [1 Byte]  ╮
		 * m: Motion     1 = Motion data present in event [4 Byte]   ├ If all 0, this is a gen 2 event [See below]
		 * b: Button     1 = Button data present in event [1 Byte]   ╯
		 *
		 * Order of data in payload is as follows:
		 * ┄╦═══════════════╦═══════════════╦═══════════════╦═══════════════╦
		 *  ║ [Button/b]    ║ [Trigger/t]   ║ [Motion/t]    ║ [IMU Data/I]  ║
		 * ┄╩═══════════════╩═══════════════╩═══════════════╩═══════════════╩
		 */

		bool firstGen = ((flags & 0x7) != 0);

		if (firstGen) {
			bool flagTrigger = HAS_FLAG(flags, 0x4);
			bool flagMotion = HAS_FLAG(flags, 0x2);
			bool flagButton = HAS_FLAG(flags, 0x1);

			static buttonEvent bEvent;
			memset(&bEvent, 0, sizeof(bEvent));
			if (flagButton) {
				bEvent.pressedButtonsValid = 1;
				bEvent.pressedButtons = POP_BYTE(payloadPtr);
			}

			if (flagTrigger) {
				bEvent.triggerHighResValid = 1;
				bEvent.triggerHighRes = POP_BYTE(payloadPtr) * 128;
			}

			if (flagMotion) {
				bEvent.touchpadHorizontalValid = 1;
				bEvent.touchpadVerticalValid = 1;

				bEvent.touchpadHorizontal = POP_SHORT(payloadPtr);
				bEvent.touchpadVertical = POP_SHORT(payloadPtr);
			}

			registerButtonEvent(w, &bEvent);
		} else {
			// Second gen event (Eg Knuckles proximity)
			uint8_t genTwoType =
				POP_BYTE(payloadPtr); // May be flags, but currently only observed to be 'a1' when knuckles

			if (genTwoType == 0xA1) {
				// Knucles
				/*SV_INFO("GRIP 0x%02hX [Time:%04hX] [Payload: %s] <<ABORT FURTHER READ>>",
				 *(payloadPtr-1), time, packetToHex(payloadPtr, payloadEndPtr));*/

				// Resistive contact sensors in buttons?
				uint8_t touchFlags = POP_BYTE(payloadPtr);
				// 0x01 = Trigger
				// 0x08 = Menu
				// 0x10 = Button A
				// 0x20 = Button B
				// 0x40 = Thumbstick

				// Non-touching proximity to fingers
				uint8_t fingerProximity[4];
				fingerProximity[0] = POP_BYTE(payloadPtr); // Middle finger
				fingerProximity[1] = POP_BYTE(payloadPtr); // Ring finger
				fingerProximity[2] = POP_BYTE(payloadPtr); // Pinky finger
				fingerProximity[3] = POP_BYTE(payloadPtr); // Index finger (trigger)
				(void)fingerProximity;

				// Contact force (Squeeze strength)
				uint8_t gripForce = POP_BYTE(payloadPtr);
				uint8_t trackpadForce = POP_BYTE(payloadPtr);
				(void)gripForce;
				(void)trackpadForce;

				SV_VERBOSE(
					150,
					"KAS: @%04hX | Grip    [Proximity: %02X %02X %02X %02X] [Touch: %s%s%s%s%s (%02X)] [Grip: %02X "
					"%02X]",
					time, fingerProximity[3], fingerProximity[0], fingerProximity[1], fingerProximity[2],
					HAS_FLAG(touchFlags, 0x01) ? "#" : "_", HAS_FLAG(touchFlags, 0x08) ? "#" : "_",
					HAS_FLAG(touchFlags, 0x10) ? "#" : "_", HAS_FLAG(touchFlags, 0x20) ? "#" : "_",
					HAS_FLAG(touchFlags, 0x40) ? "#" : "_", touchFlags, gripForce, trackpadForce);

			} else {
				SV_WARN("Unknown gen two event 0x%02hX 0b%s [Time:%04hX] [Payload: %s] <<ABORT FURTHER READ>>",
						*(payloadPtr - 1), byteToBin(*(payloadPtr - 1)), time, packetToHex(payloadPtr, payloadEndPtr));
				// Since we don't know how much data this should consume, proceeding to IMU/Light decode is likely
				// to choke.
				return false;
			}
		}
	} else {
		/*
		 * Flags for non-input (status) events are as follows:
		 *
		 * ┄╦═╤═╤═╦═╤═╤═╤═╤═╦┄
		 *  │1│1│1│0│-│?│?│b│
		 * ┄╩═╧═╧═╩═╧═╧═╧═╧═╩┄
		 *
		 * b: Battery     1 = Battery data present in event [1 Byte] possibly followed by an another event or light data
		 */

		bool flagBatteryStatus = HAS_FLAG(flags, 0x1);
		bool flagUnknown = ((flags & 0x6) != 0);

		// flagUnknown = true;

		if (flagUnknown) {
			SV_WARN("Unknown status event 0x%02hX [Time:%04hX] [Payload: %s] <<ABORT FURTHER READ>>", *(payloadPtr - 1),
					time, packetToHex(payloadPtr, payloadEndPtr));
			// Since we don't know how much data this should consume, proceeding to IMU/Light decode is likely
			// to choke.
			*readPtr = payloadPtr;
			return false;
		}

		if (flagBatteryStatus) {
			// Battery Status
			// Happens On USB plugged in. Switch to wired mode?
			uint8_t batStatus = POP_BYTE(payloadPtr);
			int percent = (int)((((float)(batStatus & 0x7f)) / 0x7f) * 100);
			bool charging = (batStatus & 0x80) == 0x80;
#ifdef KNUCKLES_INFO
			SV_INFO("KAS: @%04hX | Status  [Battery: % 3i%%] [%s]", time, percent,
					(charging ? "CHARGING" : "ON BATTERY"));
#endif
			// Maybe read another event, IMU data or Light Data
			bool rtn = read_event(w, time, &payloadPtr, payloadEndPtr);
			*readPtr = payloadPtr;
			return rtn;
		}
	}

	// Read off any IMU data present
	if (flagIMU)
		read_imu_data(w, time, &payloadPtr, payloadEndPtr);

	UPDATE_PTR_AND_RETURN
}

static inline void parse_and_process_lightcap(SurviveObject *w, uint16_t time, uint8_t *payloadPtr,
											  uint8_t *payloadEndPtr) {
	LightcapElement les[10] = {0};
	uint8_t *payloadPtrStart = payloadPtr;
	ssize_t cnt = read_light_data(w, time, &payloadPtr, payloadEndPtr, les, 10);
	SurviveContext *ctx = w->ctx;
#ifndef NDEBUG
	for (int i = (int)cnt - 1; i >= 0; i--) {
		uint8_t sensor = survive_map_sensor_id(w, les[i].sensor_id);
		if (sensor == 255) {
			cnt = -255;
			break;
		}
	}
#endif

	if (cnt < 0) {
		SV_WARN("Read light data error %d   [Time:%04hX] [Payload: %s]", (int)cnt, time,
				packetToHex(payloadPtr, payloadEndPtr));
		SV_WARN("Full payload: %s", packetToHex(payloadPtrStart, payloadEndPtr));
	} else {

#ifdef VERIFY_LIGHTCAP
		LightcapElement les_old[10] = {0};
		int les_old_cnt = parse_watchman_lightcap(w->ctx, w->codename, time >> 8, w->activations.last_imu, payloadPtr,
												  payloadEndPtr - payloadPtr, les, 10);

		assert(cnt == les_old_cnt);
#endif
		for (int i = (int)cnt - 1; i >= 0; i--) {
#ifdef DEBUG_WATCHMAN
			printf("%d: %u [%u]\n", les[i].sensor_id, les[i].length, les[i].timestamp);
#endif
#ifdef VERIFY_LIGHTCAP
			assert(memcmp(&les[i], &les_old[i], sizeof(LightcapElement)) == 0);
#endif
			handle_lightcap(w, &les[i]);
		}
	}
}

static int parse_and_process_raw1_lightcap(SurviveObject *obj, uint16_t time, uint8_t *packet, uint8_t length) {
	bool has_errors = false;
	uint8_t idx = 0;
	uint8_t channel = 255;
	SurviveContext *ctx = obj->ctx;
	bool dump_binary = false;

	while (idx < length) {
		uint8_t data = packet[idx];

		if (data & 0x1u) {
			// Since they flag for this; I assume multiples can appear in a single packet. Need to plug in
			// second LH to find out...

			if ((data & 0x0Au) != 0) {
				// Currently I've only ever seen 0x1 if the 1 bit is set; I doubt they left 3 bits on the table
				// though....
				SV_WARN("Not entirely sure what this data is; errors may occur (%d, 0x%02x)\n", idx, data);
				dump_binary = true;
				// has_errors = true;
				goto exit_loop;
			}

			// encodes like so: 0bcccc ?F?C
			bool hasConflict = data & 0x04u;
			if (hasConflict) {
				uint8_t conflicted_channel = data >> 4u;
				SV_WARN("Two or more lighthouses are on channel %d; tracking is most likely going to fail.",
						conflicted_channel);
			}

			channel = data >> 4u;

			idx++;
		} else {
			uint32_t timecode = 0;
			memcpy(&timecode, packet + idx, sizeof(uint32_t));

			uint32_t reference_time = (obj->activations.last_imu);

			bool sync = timecode & 0x2u;
			if (!sync) {
				//                          O                               SC
				//                         GO                               YH
				//                         ET                               NA
				//                    [??] NX[    24 bit time @ 48mhz      ]CN
				// encodes like so: 0bXXXX ABTTT TTTT TTTT TTTT TTTT TTTT TTSC
				bool ootx = (timecode >> 26u) & 1u;
				bool g = (timecode >> 27u) & 1u;
				timecode = fix_time24((timecode >> 2u) & 0xFFFFFFu, reference_time);
				uint8_t unused = timecode >> 28;
				if (unused && dump_binary) {
					SV_WARN("Not sure what this is: %x", unused);
				}
				SV_VERBOSE(200, "Sync %s %02d %d %8u", obj->codename, channel, ootx, timecode);
				if (channel == 255) {
					SV_WARN("No channel specified for sync");
					dump_binary = true;
					// has_errors = true;
				} else {
					obj->ctx->syncproc(obj, channel, timecode, ootx, g);
				}
			} else {
				//                                                         SC
				//                                                         YH
				//                                                         NA
				//                    [SNSR][    25 bit time @ 96mhz      ]CN
				// encodes like so: 0bSSSS STTT TTTT TTTT TTTT TTTT TTTT TFSC

				// Since nothing in libsurvive thinks anything is 96mhz; pass in a flag
				bool half_clock_flag = timecode & 0x4u;
				uint8_t sensor = (timecode >> 27u);
				timecode = fix_time24((timecode >> 3u) & 0xFFFFFFu, reference_time);
				SV_VERBOSE(200, "Sweep %s %02d.%02d %8u", obj->codename, channel, sensor, timecode);
				if (channel == 255) {
					SV_WARN("No channel specified for sweep");
					dump_binary = true;
					// has_errors = true;
				} else {
					obj->ctx->sweepproc(obj, channel, survive_map_sensor_id(obj, sensor), timecode, half_clock_flag);
				}
			}

			if (channel == 255) {
				dump_binary = true;
				has_errors = true;
				goto exit_loop;
			}
			idx += 4;
		}
	}

exit_loop:

	if (dump_binary) {
		for (int i = 0; i < length; i++) {
			if ((i + 2) % 4 == 0)
				ctx->printfproc(ctx, "  ");
			ctx->printfproc(ctx, "%02x ", packet[i]);
		}

		ctx->printfproc(ctx, "\n");
	}

	return has_errors ? -1 : idx;
}

static bool handle_input(SurviveObject *w, uint8_t flags, uint8_t **payloadPtr, uint8_t *payloadEndPtr) {
	/*
	 * Flags for input events are as follows:
	 *
	 * ┄╦═╤═╤═╦═╤═╤═╤═╤═╦┄
	 *  │1│1│1│1│-│t│m│b│
	 * ┄╩═╧═╧═╩═╧═╧═╧═╧═╩┄
	 *
	 * t: Trigger    1 = Trigger data present in event [1 Byte]  ╮
	 * m: Motion     1 = Motion data present in event [4 Byte]   ├ If all 0, this is a gen 2 event [See below]
	 * b: Button     1 = Button data present in event [1 Byte]   ╯
	 *
	 * Order of data in payload is as follows:
	 * ┄╦═══════════════╦═══════════════╦═══════════════╦═══════════════╦
	 *  ║ [Button/b]    ║ [Trigger/t]   ║ [Motion/t]    ║ [IMU Data/I]  ║
	 * ┄╩═══════════════╩═══════════════╩═══════════════╩═══════════════╩
	 */
	struct SurviveContext *ctx = w->ctx;
	bool firstGen = ((flags & 0x7) != 0);

	if (firstGen) {
		bool flagTrigger = HAS_FLAG(flags, 0x4);
		bool flagMotion = HAS_FLAG(flags, 0x2);
		bool flagButton = HAS_FLAG(flags, 0x1);

		static buttonEvent bEvent;
		memset(&bEvent, 0, sizeof(bEvent));
		if (flagButton) {
			bEvent.pressedButtonsValid = 1;
			bEvent.pressedButtons = POP_BYTE(*payloadPtr);
		}

		if (flagTrigger) {
			bEvent.triggerHighResValid = 1;
			bEvent.triggerHighRes = POP_BYTE(*payloadPtr) * 128;
		}

		if (flagMotion) {
			bEvent.touchpadHorizontalValid = 1;
			bEvent.touchpadVerticalValid = 1;

			bEvent.touchpadHorizontal = POP_SHORT(*payloadPtr);
			bEvent.touchpadVertical = POP_SHORT(*payloadPtr);
		}
		SV_VERBOSE(150, "handle_input flags %d %d %d", flagButton, flagTrigger, flagMotion);
		registerButtonEvent(w, &bEvent);
	} else {
		// Second gen event (Eg Knuckles proximity)
		uint8_t genTwoType =
			POP_BYTE(*payloadPtr); // May be flags, but currently only observed to be 'a1' when knuckles

		if (genTwoType == 0xA1) {
			// Knucles

			// Resistive contact sensors in buttons?
			uint8_t touchFlags = POP_BYTE(*payloadPtr);
			// 0x01 = Trigger
			// 0x08 = Menu
			// 0x10 = Button A
			// 0x20 = Button B
			// 0x40 = Thumbstick

			// Non-touching proximity to fingers
			uint8_t fingerProximity[4];
			fingerProximity[0] = POP_BYTE(*payloadPtr); // Middle finger
			fingerProximity[1] = POP_BYTE(*payloadPtr); // Ring finger
			fingerProximity[2] = POP_BYTE(*payloadPtr); // Pinky finger
			fingerProximity[3] = POP_BYTE(*payloadPtr); // Index finger (trigger)
			(void)fingerProximity;

			// Contact force (Squeeze strength)
			uint8_t gripForce = POP_BYTE(*payloadPtr);
			uint8_t trackpadForce = POP_BYTE(*payloadPtr);
			(void)gripForce;
			(void)trackpadForce;

			SV_VERBOSE(
				150,
				"handle_input A1 : | Grip    [Proximity: %02X %02X %02X %02X] [Touch: %s%s%s%s%s (%02X)] [Grip: %02X "
				"%02X]",
				fingerProximity[3], fingerProximity[0], fingerProximity[1], fingerProximity[2],
				HAS_FLAG(touchFlags, 0x01) ? "#" : "_", HAS_FLAG(touchFlags, 0x08) ? "#" : "_",
				HAS_FLAG(touchFlags, 0x10) ? "#" : "_", HAS_FLAG(touchFlags, 0x20) ? "#" : "_",
				HAS_FLAG(touchFlags, 0x40) ? "#" : "_", touchFlags, gripForce, trackpadForce);
		} else {
			SV_WARN("Unknown gen two event 0x%02hX 0b%s [Payload: %s] <<ABORT FURTHER READ>>", *(*payloadPtr - 1),
					byteToBin(*(*payloadPtr - 1)), packetToHex(*payloadPtr, payloadEndPtr));
			// Since we don't know how much data this should consume, proceeding to IMU/Light decode is likely
			// to choke.
			return false;
		}
	}
	return true;
}

static void handle_watchman_v2(SurviveObject *w, uint16_t time, uint8_t *payloadPtr, uint8_t *payloadEndPtr) {
	struct SurviveContext *ctx = w->ctx;
	const uint8_t *originPayloadPtr = payloadPtr;
	struct SurviveUSBInfo *driverInfo = w->driver;
	if (driverInfo->lightcapMode != LightcapMode_raw1) {
		vive_switch_mode(driverInfo, LightcapMode_raw1);
		return;
	}

	uint8_t flags = POP_BYTE(payloadPtr);
	bool has_errors = false;
	bool flagLightcap = HAS_FLAG(flags, 0x10);

	// Info: Watchman v2(KN1): '34 f2 3f 9f a2 c9             | 61 06 8c 0a 44 '
	// Info: Watchman v2(KN1): '34 f2 7a 9e fe c8             | 61 b4 6e 27 02 7e 95 7c 7c d6 b3 7c 5c a6 da 7c bc '
	// Info: Watchman v2(KN1): '38 f0 a1 00 2a 00 00 1c 00 01 | 61 32 9d 62 15 2e d8 62 0d 62 26 63 45 '
	// Info: Watchman v2(KN1): '31 f4 ff                      | 61 de 25 d1 62 '

	// Touch (not press) A
	// Info: Watchman v2(KN1): '20 f0 a1 20 00 01 00 00 00 00 '

	// Pulling trigger:
	// Info: Watchman v2(KN1): '20 f4 39 '
	// Info: Watchman v2(KN1): '20 f0 a1 00 bb f5 ff e5 07 00 '
	// Info: Watchman v2(KN1): '20 f0 a1 00 00 03 00 00 00 00 '

	// Info: Watchman v2(KN1): '31 f1 00                      | 61 42 a4 ef 80 ee b4 ef c0 '
	// Info: Watchman v2(KN1): '31 f4 0c 61 44 56 97 09 46 88 5b b3 8a a1 5b 1b 9a e4 5b 3b 76 f0 5b bb '

	// Hitting A:
	// Info: Watchman v2(KN1): '31 f4 f8                      | 61 ca 25 9f 7f 26 49 9f 5f 06 c5 9f bf f6 db 9f 17 8a e8
	// 9f 0f '

	// Hitting B
	// Info: Watchman v2(KN1): '31 f4 12                      | 61 68 62 56 01 '

	// Info: Watchman v2(KN1): '35 f6 5b ef e5 6d 3e          | 61 cc 3d de 00 c6 51 ea 59 72 7b ea b9 9a 81 ea 11 '
	// Info: Watchman v2(KN1): '35 f6 2d 36 ec 07 1e          | 61 da 56 93 c0 02 aa 93 60 '
	// Info: Watchman v2(KN1): '35 f6 15 b6 25 98 1b          | 61 20 c4 fa 03 fa 89 23 78 f6 c9 23 58 3a 2d 24 b8 '

	// Possibly this means the protocol is done switching:
	// Info: Watchman v2(KN1): '51 20 01                      | 61 50 d2 57 01 '
	// Info: Watchman v2(KN1): '36 f7 02 18 12 e1 b8 06       | 61 74 7e bf 0b 26 b3 ac 7f 76 d2 ac 5f e2 00 ad 17 '

	// Info: Watchman v2(KN1): '80 a5 4c fb 8f f1 f2 f9 d3 ff 2c 00 df ff '
	// Info: Watchman v2(KN1): 'c0 92 52 fb 7d f1 ea f9 ce ff 2f 00 dc ff 20 01 '

	// bool has_errors = !read_event(w, time, &payloadPtr, payloadEndPtr);
	bool flagIMU = HAS_FLAG(flags, 0x80);
	bool flagUnknown01 = HAS_FLAG(flags, 0x01);
	// Haptic on the trackpad thing on the knuckles?
	bool flagUnknown04 = HAS_FLAG(flags, 0x04);
	bool flagUnknown08 = HAS_FLAG(flags, 0x08);
	bool flagInput = HAS_FLAG(flags, 0x20);
	bool flagUnknown40 = HAS_FLAG(flags, 0x40);

	if (HAS_FLAG(flags, ~0xD1)) {
		SV_VERBOSE(100, "%s Unknown flag %02x", w->codename, flags);
	}

	if (flagIMU)
		read_imu_data(w, time, &payloadPtr, payloadEndPtr);

	if (flagUnknown40) {
		uint8_t unknownByte1 = POP_BYTE(payloadPtr);
		uint8_t unknownByte2 = POP_BYTE(payloadPtr);
		SV_VERBOSE(100, "%s Unknown flag 0x40 byte %02x %02x", w->codename, unknownByte1, unknownByte2);
	}

	if (flagInput) {
		uint8_t input_flags = POP_BYTE(payloadPtr);
		has_errors = !handle_input(w, input_flags, &payloadPtr, payloadEndPtr);
	}

	/*
	if (flagUnknown01) {
		//uint8_t unknownByte1 = POP_BYTE(payloadPtr);
		//uint8_t unknownByte2 = POP_BYTE(payloadPtr);
		//SV_VERBOSE(100, "Unknown flag 0x1 %02x", unknownByte1);
		SV_VERBOSE(100, "Unknown flag 0x01");
	}

	if (flagUnknown08) {
		uint8_t unknownBytes[] = {POP_BYTE(payloadPtr), POP_BYTE(payloadPtr), POP_BYTE(payloadPtr),
								  POP_BYTE(payloadPtr), POP_BYTE(payloadPtr), POP_BYTE(payloadPtr),
								  POP_BYTE(payloadPtr), POP_BYTE(payloadPtr), POP_BYTE(payloadPtr)};

		SV_VERBOSE(100, "Unknown flag 0x8 %02x %02x %02x %02x %02x %02x ...", unknownBytes[0], unknownBytes[1],
				   unknownBytes[2], unknownBytes[3], unknownBytes[4], unknownBytes[5]);
	}

	if (flagUnknown04) {
		uint8_t unknownBytes[] = {POP_BYTE(payloadPtr), POP_BYTE(payloadPtr), POP_BYTE(payloadPtr),
								  POP_BYTE(payloadPtr), POP_BYTE(payloadPtr) };

		SV_VERBOSE(100, "Unknown flag 0x20 %02x %02x %02x %02x %02x %02x", unknownBytes[0], unknownBytes[1],
				   unknownBytes[2], unknownBytes[3], unknownBytes[4] );
	}
*/
	if (driverInfo->packetsSeenWaitingForV2 > 200) {
		driverInfo->packetsSeenWaitingForV2 = 0;
		driverInfo->timeWithoutFlag = 0;
	}

	if (driverInfo->timeWithoutFlag > 0 && driverInfo->timeWithoutFlag < 20) {
		driverInfo->packetsSeenWaitingForV2++;
		if (flagLightcap) {
			driverInfo->timeWithoutFlag = 1;
			flagLightcap = false;
			SV_VERBOSE(200, "Discard %s %lu: '%s'", w->codename, driverInfo->timeWithoutFlag,
					   packetToHex(payloadPtr, payloadEndPtr));
		} else {
			driverInfo->timeWithoutFlag++;
		}
	}

	if (flagLightcap && !has_errors) {
		if (driverInfo->lightcapMode == LightcapMode_raw0) {
			// parse_and_process_lightcap(w, time, payloadPtr, payloadEndPtr);
		} else {
			int read = parse_and_process_raw1_lightcap(w, time, payloadPtr, payloadEndPtr - payloadPtr);
			if (read == -1)
				has_errors = true;
			else
				payloadPtr += read;
		}
	}

	if (driverInfo->timeWithoutFlag == 20) {
		if (payloadPtr != payloadEndPtr) {
			has_errors = true;
			SV_WARN("Did not read full input packet; %ld bytes remain", payloadEndPtr - payloadPtr);
		}
	}
	if (has_errors) {
		survive_dump_buffer(ctx, originPayloadPtr, payloadEndPtr - originPayloadPtr);
		// assert(false);
	}
}

static void handle_watchman(SurviveObject *w, uint8_t *readdata) {
	struct SurviveUSBInfo *driverInfo = w->driver;

	// KASPER'S DECODE
	SurviveContext *ctx = w->ctx;

	/*
	 * ---=== PACKET STRUCTURE ===---
	 * Key:
	 *  [Optional] - Element may or may not be present depending on flags
	 *  1 = Bit is set
	 *  0 = Bit is not set
	 *  - = Bit is "don't care"
	 *  ? = Bit meaning is unknown
	 *  ║ = Byte boundary
	 *  │ = Bit boundary
	 *
	 *
	 *                                 ┊<------- Size is the length of this section in bytes ------->┊
	 *                                 ┊                                                             ┊
	 * ╔═══════════════╦═══════════════╬═══════════════╦┄┄┄                                          ┊
	 * ║Time MSB       ║Size           ║Time LSB       ║                                             ┊
	 * ╚═══════════════╩═══════════════╩═══════════════╩┄┄┄                                          ┊
	 *                                                 ┊                                             ┊
	 *                                              ┄┄┄╬═╤═╤═╦═╤═╤═╤═╤═╦═┄┄┄┄┄┄┄┄┄┄┄═╦═┄┄┄┄┄┄┄┄┄┄┄┄══╣
	 *  For Non-Light events (First three bits = 111)  ║1│1│1│  FLAGS  ║ [Payload]   ║ [Light Data]  ║
	 *                                              ┄┄┄╬═╧═╧═╩═╧═╧═╧═╧═╩═┄┄┄┄┄┄┄┄┄┄┄═╩═┄┄┄┄┄┄┄┄┄┄┄┄══╣
	 *                                                 ┊                                             ┊
	 *                                              ┄┄┄╬═══════════════════┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄══╣
	 *  For light-only events (First three bits != 111)║ Light Data                                  ║
	 *                                              ┄┄┄╩═══════════════════┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄══╝
	 *
	 * Since bits 7&8 are part of the input event or the sensor id, this places limits on the available id space thus :
	 *   The first byte of the light data is sensor data, of which the upper 5 bits are the sensor id, and given
	 *   bits 7&8 cannot both be 1, results in the range 00000XXX to 10111XXX yielding a maximum of 24 light
	 *   elements per device (In the current format).
	 *
	 * Packet Decode Process:
	 *   1) Read off Time (MSB and LSB) and Payload Size
	 *   2) If bits 6-8 of the byte 4 are set:
	 *       a) Read the input/status event
	 *       b) Compute the remaining bytes after the input/status event has been read (The size of the input/status
	 *          event is not explicitly provided, so must be implied by the event type.
	 *   3) Read an interpret any remaining bytes as light data
	 */

	uint16_t time = AS_SHORT(readdata[0], readdata[2]);
	uint8_t payloadSize = readdata[1] - 1;
	uint8_t *payloadPtr = &readdata[3];
	uint8_t *payloadEndPtr = payloadPtr + payloadSize;

	if (w->ctx->lh_version == 1) {
		SV_VERBOSE(150, "Watchman v2(%s): '%s'", w->codename, packetToHex(payloadPtr, payloadEndPtr));
		handle_watchman_v2(w, time, payloadPtr, payloadEndPtr);
		return;
	}

	SV_VERBOSE(150, "Watchman v1(%s): %s", w->codename, packetToHex(payloadPtr, payloadEndPtr));
	/*
	if (w->ctx->lh_version == -1) {
		attempt_lh_detection(w, payloadPtr, payloadEndPtr);
		return;
	}
	 */
	struct SurviveUSBInfo *driver = w->driver;
	if (driver->lightcapMode != LightcapMode_raw0) {
		return;
	}

	// Read any non-light events that may be in the packet
	if (!read_event(w, time, &payloadPtr, payloadEndPtr)) {
		SV_WARN("Read event failed; full payload: %s", packetToHex(&readdata[3], payloadEndPtr));
		return;
	}

	// Any remaining data after events (if any) have been read off is light data
	if (payloadPtr < payloadEndPtr) {
		LightcapElement les[10] = {0};
		int32_t cnt = read_light_data(w, time, &payloadPtr, payloadEndPtr, les, 10);

		if (cnt < 0) {
			SV_WARN("Read light data error %d   [Time:%04hX] [Payload: %s]", (int)cnt, time,
					packetToHex(payloadPtr, payloadEndPtr));

		} else {

#ifdef VERIFY_LIGHTCAP
			LightcapElement les_old[10] = {0};
			int les_old_cnt = parse_watchman_lightcap(w->ctx, w->codename, time >> 8, w->activations.last_imu,
													  payloadPtr, payloadEndPtr - payloadPtr, les, 10);

			assert(cnt == les_old_cnt);
#endif
			for (int i = (int)cnt - 1; i >= 0; i--) {
#ifdef DEBUG_WATCHMAN
				printf("%d: %u [%u]\n", les[i].sensor_id, les[i].length, les[i].timestamp);
#endif
#ifdef VERIFY_LIGHTCAP
				assert(memcmp(&les[i], &les_old[i], sizeof(LightcapElement)) == 0);
#endif
				handle_lightcap(w, &les[i]);
			}
		}
	}
}

int parse_watchman_lightcap(struct SurviveContext *ctx, const char *codename, uint8_t time1,
							survive_timecode reference_time, uint8_t *readdata, size_t qty, LightcapElement *les,
							size_t output_cnt) {

	assert(qty > 0);
	uint8_t *mptr = readdata + qty - 3 - 1; //-3 for timecode, -1 to

#ifdef DEBUG_WATCHMAN
	fprintf(stderr, "_%s lc Data: ", codename);
	for (int i = 0; i < qty; i++) {
		fprintf(stderr, "%02x ", readdata[i]);
	}
	fprintf(stderr, "\n");
#endif

	uint32_t mytime = (mptr[3] << 16) | (mptr[2] << 8) | (mptr[1] << 0);

	uint32_t times[20] = {0};
	int timecount = 0;
	int leds;
	int fault = 0;

	/// Handle uint32_tifying (making sure we keep it incrementing)
	mytime |= ((uint32_t)time1 << 24);

	// The general issue is that the 'time1' field isn't super in sync with the last 3 bytes we use for timing
	// light events -- it can tip a smidge before those bytes see it or sometimes after. This can cause
	// wild 1<<24 tick differences which break everything.
	//
	// We base it off IMU as a reference because light events can get blocked and it's not out of the ordinary
	// to not see them for a second or two. (1 << 23) on a 48mhz clock is ~150ms; and the IMU is consistently
	// much faster than that.
	if (mytime > reference_time && mytime - reference_time > (1 << 23)) {
		mytime -= (1 << 24);
	} else if (reference_time > mytime && reference_time - mytime > (1 << 23)) {
		mytime += (1 << 24);
	}

	times[timecount++] = mytime; //
#ifdef DEBUG_WATCHMAN
	fprintf(stderr, "_%s Packet Start Time: %u\n", codename, mytime);
#endif

	// First, pull off the times, starting with the current time, then all the delta times going backwards.
	{
		while (mptr - readdata > (timecount >> 1)) {
			uint32_t time_delta = 0;
#ifdef DEBUG_WATCHMAN
			fprintf(stderr, "%s\t", codename);
#endif

			// https://en.wikipedia.org/wiki/Variable-length_quantity
			uint8_t codebyte = 0;
			int codelength = 0;
			while ((codebyte & 0x80) == 0 && mptr >= readdata) {
				codebyte = *(mptr--);
				time_delta = (time_delta << 7) | (codebyte & 0x7f);
				codelength++;

				if (codelength > 5) {
					SV_WARN("Code word too long");
					fault = 7;
					goto end;
				}
#ifdef DEBUG_WATCHMAN
				fprintf(stderr, "%02x ", codebyte);
#endif
			}
			times[timecount++] = (mytime -= time_delta);
#ifdef DEBUG_WATCHMAN
			fprintf(stderr, " newtime: %u (%u)\n", mytime, time_delta);
#endif
		}

		leds = timecount >> 1;
		// Check that the # of sensors at the beginning match the # of parameters we would expect.
		if (timecount & 1) {
			SV_WARN("Uneven time count -- %d %d", leds, timecount);
			fault = 1;
			goto end;
		} // Inordinal LED count
		if (leds != mptr - readdata + 1) {
			fault = 2;
			SV_WARN("Bad LED count in packet %d; should be %d", leds, (int)(mptr - readdata + 1));
			goto end;
		} // LED Count does not line up with parameters
	}

	size_t times_size = timecount;

	int lese = 0; // les's end

	// Second, go through all LEDs and extract the lightevent from them.
	{
		uint8_t *marked = alloca(timecount);
		memset(marked, 0, timecount);

		int i, parpl = 0;
		timecount--;
		int timepl = 0;

		// This works, but usually returns the values in reverse end-time order.
		for (i = 0; i < leds; i++) {
			uint8_t led = readdata[i] >> 3;
			int adv = readdata[i] & 0x07;

			while (marked[timepl])
				timepl++;

#ifdef DEBUG_WATCHMAN
			fprintf(stderr, "TP %d   TC: %d : ", timepl, timecount);
			for (int i = 0; i < timecount; i++) {
				fprintf(stderr, "%d", marked[i]);
			}
			fprintf(stderr, "\n");
#endif

			if (timepl > timecount) {
				fault = 3;
				goto end;
			} // Ran off max of list.
			uint32_t endtime = times[timepl++];

			int end = timepl + adv;
			if (end > timecount) {
				SV_WARN("Lightfault 4: %d > %d", end, timecount);
				fault = 4;
				goto end;
			} // end referencing off list
			if (marked[end] > 0) {
				fault = 5;
				goto end;
			} // Already marked trying to be used.
			uint32_t starttime = times[end];
			marked[end] = 1;

			// Insert all lighting things into a sorted list.  This list will be
			// reverse sorted, but that is to minimize operations.  To read it
			// in sorted order simply read it back backwards.
			// Use insertion sort, since we should most of the time, be in order.
			assert(lese < output_cnt);
			LightcapElement *le = &les[lese++];
			le->sensor_id = led;

			if ((uint32_t)(endtime - starttime) > 65535) {
				fault = 6;
				goto end;
			} // Length of pulse dumb.
			le->length = endtime - starttime;
			le->timestamp = starttime;

#ifdef DEBUG_WATCHMAN
			fprintf(stderr, "_%s Event: %d %u %u-%u\n", codename, led, le->length, endtime, starttime);
#endif
			int swap = lese - 2;
			while (swap >= 0 && les[swap].timestamp < les[swap + 1].timestamp) {
				LightcapElement l;
				memcpy(&l, &les[swap], sizeof(l));
				memcpy(&les[swap], &les[swap + 1], sizeof(l));
				memcpy(&les[swap + 1], &l, sizeof(l));
				swap--;
			}
		}
	}

	return lese;

end : {
#ifdef DEBUG_WATCHMAN_ERRORS
	SV_INFO("Light decoding fault: %d", fault);
	fprintf(stderr, "Info: _%s %u %u ", codename, time1, reference_time);
	for (int i = 0; i < qty; i++) {
		fprintf(stderr, "%02x ", readdata[i]);
	}
	fprintf(stderr, "\n");
#endif
	return -fault;
	}
}

struct SurviveUSBInfo *survive_vive_register_driver(SurviveObject *so, uint16_t vid, uint16_t pid) {
	struct SurviveUSBInfo *d = calloc(1, sizeof(struct SurviveUSBInfo));
	so->driver = d;
	d->so = so;
	d->lightcapMode = LightcapMode_raw0;
	for (const struct DeviceInfo *info = KnownDeviceTypes; info->name; info++) {
		if (info == 0) {
			continue;
		}

		if (info->vid != vid || info->pid != pid) {
			continue;
		}

		d->device_info = info;
		break;
	}

	return d;
}

static inline uint16_t read_buffer16(uint8_t *readdata, int idx) {
	uint16_t rtn;
	memcpy(&rtn, readdata + idx, sizeof(uint16_t));
	return rtn;
}
static inline uint32_t read_buffer32(uint8_t *readdata, int idx) {
	uint32_t rtn;
	memcpy(&rtn, readdata + idx, sizeof(uint32_t));
	return rtn;
}

static void parse_tracker_version_info(SurviveObject *so, uint8_t *data, size_t size) {
	SurviveContext *ctx = so->ctx;

#pragma pack(push, 1)
	struct {
		uint32_t revision;
		uint32_t some_other_number;
		char fw_name[31];
		uint16_t a, b, c, d;
		uint8_t e;
		uint16_t fpga_major_version;
		uint8_t fpga_minor_version;
		uint8_t fpga_patch_version;
		uint8_t h;
		uint8_t flags1, flags2;
		uint16_t i, j;
		uint32_t k;
	} version_info;
#pragma pack(pop)
	memcpy(&version_info, data, sizeof(version_info));
	SV_INFO("Device %s has FW version %u and FPGA version %u/%u/%u; named %31s", so->codename, version_info.revision,
			version_info.fpga_major_version, version_info.fpga_minor_version, version_info.fpga_patch_version,
			version_info.fw_name);
}

static void parse_tracker_info(SurviveObject *so, uint8_t id, uint8_t *readdata, size_t size) {
	SurviveContext *ctx = so->ctx;
	switch (id) {
	case VIVE_REPORT_INFO: {
		SV_INFO("Info 1: 0x%x 0x%x", readdata[0], readdata[1]);
		goto dump_data;
		break;
	}
	case VIVE_REPORT_CHANGE_MODE: {
		SV_INFO("Info 4: ")
		goto dump_data;
	}
	case VIVE_REPORT_VERSION: {
		parse_tracker_version_info(so, readdata, size);
		break;
	}
	case VIVE_REPORT_HMD_SET_READOFFSET_USERDATA: {
		SV_INFO("Info 8: 0x%x", readdata[0]);
		break;
	}
	// Start config stream
	case VIVE_REPORT_CONFIG_READMODE: {
		break;
	}
	// Request more config stream
	case VIVE_REPORT_CONFIG_READ: {
		break;
	}
	case VIVE_REPORT_COMMAND: {
		SV_INFO("Received cmd 0x%02x with %d bytes of data", readdata[0], readdata[1]);
		survive_dump_buffer(ctx, readdata + 2, readdata[1]);
		break;
	}
	default: {
		SV_INFO("Unknown field 0x%02x for %s", id, so->codename);
		goto dump_data;
	}
	}

	return;
dump_data:
	survive_dump_buffer(ctx, readdata, size);
}

void survive_data_cb_locked(SurviveUSBInterface *si) {
	int size = si->actual_len;
	SurviveContext *ctx = si->ctx;
	int iface = si->which_interface_am_i;
	SurviveObject *obj = si->assoc_obj;
	uint8_t *readdata = si->buffer;

	if (obj->conf == 0) {
		if (si->usbInfo) {
			si->usbInfo->tryConfigLoad = 1;
		}
		return;
	}

	if (!obj->driver) {
		struct SurviveUSBInfo *d = obj->driver = calloc(1, sizeof(struct SurviveUSBInfo));
		d->so = obj;
	}

	if (iface == USB_IF_HMD_HEADSET_INFO && obj == 0)
		return;

	int id = POP1;
	size--;
	//	printf( "%16s Size: %2d ID: %d / %d\n", si->hname, size, id, iface );
//	SV_INFO("%s interface %d", obj->codename, iface);
#if 0
	if(  si->which_interface_am_i == 9 )
	{
		int i;
		printf( "%16s: %d: %d: %d: ", si->hname, id, size, sizeof(LightcapElement) );
		for( i = 0; i < size-1; i++ )
		{
			printf( "%02x ", readdata[i] );
		}
		printf( "\n" );
		
	}
#endif
	switch (si->which_interface_am_i) {
	case USB_IF_TRACKER_INFO: {
		parse_tracker_info(obj, id, readdata, size);
		break;
	}
	case USB_IF_HMD_HEADSET_INFO: {
		SurviveObject *headset = obj;
		readdata += 2;
		headset->buttonmask = POP1; // Lens
		headset->axis2 = POP2;		// Lens Separation
		readdata += 2;
		headset->buttonmask |= POP1; // Button
		readdata += 3;
		readdata++; // Proxchange, No change = 0, Decrease = 1, Increase = 2
		readdata++;
		headset->axis3 = POP2; // Proximity  	<< how close to face are you?  Less than 80 = not on face.
		headset->axis1 = POP2; // IPD   		<< what is this?
		headset->ison = 1;
		break;
	}
	case USB_IF_HMD_IMU:
	case USB_IF_W_WATCHMAN1_IMU:
	case USB_IF_TRACKER0_IMU:
	case USB_IF_TRACKER1_IMU: {
		int i;
		// printf( "%d -> ", size );
		for (i = 0; i < 3; i++) {
			struct unaligned_16_t *acceldata = (struct unaligned_16_t *)readdata;
			readdata += 12;
			uint32_t timecode = POP4;
			uint8_t code = POP1;
			// printf( "%d ", code );
			int8_t cd = code - obj->oldcode;

			if (cd > 0) {
				obj->oldcode = code;

				// XXX XXX BIG TODO!!! Actually recal gyro data.
				FLT agm[9] = {acceldata[0].v,
							  acceldata[1].v,
							  acceldata[2].v,
							  acceldata[3].v,
							  acceldata[4].v,
							  acceldata[5].v,
							  0,
							  0,
							  0};

				// assert(timecode <= obj->timebase_hz);
				ctx->raw_imuproc(obj, 3, agm, timecode, code);
			}
		}
		// DONE OK.
		break;
	}
	case USB_IF_WATCHMAN1:
	case USB_IF_WATCHMAN2: {
		SurviveObject *w = obj;
		if (id == VIVE_REPORT_RF_WATCHMAN) {
			assert(size == 29);
			handle_watchman(w, readdata);
		} else if (id == VIVE_REPORT_RF_WATCHMANx2) {
			assert(size == 29 * 2);
			handle_watchman(w, readdata);
			handle_watchman(w, readdata + 29);
		} else if (id == VIVE_REPORT_RF_TURN_OFF) {
			w->ison = 0; // turning off
		} else {
			SV_INFO("Unknown watchman code %d\n", id);
		}
		break;
	}
	case USB_IF_HMD_LIGHTCAP:
	case USB_IF_TRACKER1_LIGHTCAP: {

		bool dump_binary = false;
		if (id == VIVE_REPORT_USB_LIGHTCAP_REPORT_V1) { // LHv1
			int i;
			for (i = 0; i < 9; i++) {
				LightcapElement le;
				le.sensor_id = POP1;
				le.length = POP2;
				le.timestamp = POP4;
				if (le.sensor_id > 0xfd)
					continue;
				//SV_INFO("%d %d %d %d %d", id, le.sensor_id, le.length, le.timestamp, si->buffer + size - readdata);

				if (obj->ctx->lh_version != 1) {
					handle_lightcap(obj, &le);
				} else if (dump_binary) {
					fprintf(stderr, "sensor: %2d         time: %3.5f length: %4d end_time: %8u\n", le.sensor_id,
							le.timestamp / 48000000., le.length, le.length + le.timestamp);
				}
			}
		} else if (id == VIVE_REPORT_USB_LIGHTCAP_REPORT_V2) { // LHv2
			if (obj->ctx->lh_version == 0) {
				return;
			}
			survive_notify_gen2(obj, "Report id 39");

			// Implies that the user forced gen1
			if (obj->ctx->lh_version != 1) {
				// Shouldn't see this if the user said to use gen1 -- dump the output.
				static bool force_gen_warning = false;
				if (!force_gen_warning) {
					SV_WARN("LH Gen is %d, dumping data", obj->ctx->lh_version);
					force_gen_warning = true;
				}

				dump_binary = true;
			} else {
				vive_switch_mode(obj->driver, LightcapMode_raw1);
			}

			if (dump_binary) {
#pragma pack(push, 1)
				struct lh2_entry {
					uint8_t code; // sensor with some bit flag. Continuation flag?
					uint32_t time;
					uint32_t data;
					uint32_t mask;
				};
#pragma pack(pop)
				uint32_t samples[4] = {0}, masks[4] = {0}, times[4] = {0};

				struct lh2_entry *entries = (struct lh2_entry *)readdata;
				static uint32_t last_time = 0;

				for (int i = 0; i < 4; i++) {
					struct lh2_entry *entry = &entries[i];
					if (entry->code == 0xff)
						break;
					fprintf(stderr, "sensor: %2u flag: %u time: %8u %3.5f (%7u) ", entry->code & 0x7fu,
							(entry->code & 0x80u) > 0, entry->time, entry->time / 48000000., entry->time - last_time);

					for (int k = 0; k < 32; k++) {
						uint8_t idx = 32 - k - 1;
						bool d = ((entry->data >> (idx)) & 1u);
						bool m = ((entry->mask >> (idx)) & 1u);
						if (m)
							fprintf(stderr, "%d", d);
						else
							fprintf(stderr, "_");
					}
					samples[i] = entry->data;
					masks[i] = entry->mask;
					times[i] = entry->time;
					last_time = entry->time;
					fprintf(stderr, "\n");
				}

				uint32_t time_since_sync[4] = {0};
				survive_channel chan = survive_decipher_channel(samples, masks, times, time_since_sync, 4);
				fprintf(stderr, "Chan ootx: %d %d\n", chan / 2, chan & 1);
				for (int i = 0; i < 4; i++) {
					fprintf(stderr, "time since sync: %8u time of last sync: %8u\n", time_since_sync[i],
							entries[i].time - time_since_sync[i] * 8);
				}

				for (int i = 59 - 7; i < 59; i++) {
					fprintf(stderr, "%02x ", readdata[i]);
				}
				fprintf(stderr, "\n");
			}
		} else if (id == VIVE_REPORT_USB_LIGHTCAP_REPORT_RAW_MODE_1) {
			survive_notify_gen2(obj, "Report ID 40");

			uint8_t *packet = readdata + 1;
			uint8_t length = readdata[0];

			parse_and_process_raw1_lightcap(obj, 0, packet, length);
		} else if (id == VIVE_REPORT_USB_TRACKER_LIGHTCAP_V1) {
			SV_INFO("USB lightcap report is of an unexpected type for %s: %d (0x%02x)", obj->codename, id, id);
		} else {
			SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "USB lightcap report is of an unknown type for %s: %d (0x%02x)",
					 obj->codename, id, id);
		}

		break;
	}
	case USB_IF_W_WATCHMAN1_LIGHTCAP:
	case USB_IF_TRACKER0_LIGHTCAP: {
		if (id == VIVE_REPORT_USB_TRACKER_LIGHTCAP_V1) {
			int i = 0;
			for (i = 0; i < 7; i++) {
				uint16_t *sensorId = (uint16_t *)readdata;
				uint16_t *length = (uint16_t *)(&(readdata[2]));
				unsigned long *time = (unsigned long *)(&(readdata[4]));
				LightcapElement le;
				le.sensor_id = (uint8_t)POP2;
				le.length = POP2;
				le.timestamp = POP4;
				if (le.sensor_id > 0xfd)
					continue; //
				handle_lightcap(obj, &le);
			}
		} else {
			SV_WARN("Unrecognized report id: 0x%02x for %s", id, obj->codename);
		}
		break;
	}
	case USB_IF_TRACKER0_BUTTONS:
	case USB_IF_TRACKER1_BUTTONS:
	case USB_IF_W_WATCHMAN1_BUTTONS: {
		if (VIVE_REPORT_INFO == id) {
			// 0x00	uint8	1	reportID	HID report identifier(= 1)
			// 0x02	uint16	2	reportType(? )	0x0B04: Ping(every second) / 0x3C01 : User input
			// 0x04	uint32	4	reportCount	Counter that increases with every report
			// 0x08	uint32	4	pressedButtons	Bit field, see below for individual buttons
			// 0x0C	uint16	2	triggerOrBattery	Analog trigger value(user input) / Battery voltage ? (ping)
			// 0x0E	uint8	1	batteryCharge	Bit 7 : Charging / Bit 6..0 : Battery charge in percent
			// 0x10	uint32	4	hardwareID	Hardware ID(user input) / 0x00000000 (ping)
			// 0x14	int16	2	touchpadHorizontal	Horizontal thumb position(Left : -32768 / Right : 32767)
			// 0x16	int16	2	touchpadVertical	Vertical thumb position(Bottom : -32768 / Top : 32767)
			// 0x18 ? 2 ? unknown
			// 0x1A	uint16	2	triggerHighRes	Analog trigger value with higher resolution
			// 0x1C ? 24 ? unknown
			// 0x34	uint16	2	triggerRawMaybe	Analog trigger value, maybe raw sensor data
			// 0x36 ? 8 ? unknown
			// 0x3E	uint8	1	someBitFieldMaybe	0x00 : ping / 0x64 : user input
			// 0x3F ? 1 ? unknown

			// typedef struct
			//{
			//	//uint8_t reportId;
			//	uint16_t reportType;
			//	uint32_t reportCount;
			//	uint32_t pressedButtons;
			//	uint16_t  triggerOrBattery;
			//	uint8_t batteryCharge;
			//	uint32_t hardwareId;
			//	int16_t  touchpadHorizontal;
			//	int16_t  touchpadVertical;
			//	uint16_t unknown1;
			//	uint16_t triggerHighRes;
			//	uint8_t  unknown2;
			//	uint8_t  unknown3;
			//	uint8_t  unknown4;
			//	uint16_t triggerRaw;
			//	uint8_t  unknown5;
			//	uint8_t  unknown6; // maybe some bitfield?
			//	uint8_t  unknown7;
			//} usb_buttons_raw;

			// usb_buttons_raw *raw = (usb_buttons_raw*) readdata;
			if (read_buffer16(readdata, 0) == 0x100) {
				buttonEvent bEvent;
				memset(&bEvent, 0, sizeof(bEvent));

				bEvent.pressedButtonsValid = 1;
				bEvent.pressedButtons = read_buffer32(readdata, 0x7);
				bEvent.triggerHighResValid = 1;
				// bEvent.triggerHighRes = raw->triggerHighRes;
				// bEvent.triggerHighRes = (raw->pressedButtons & 0xff000000) >> 24; // this seems to provide the same
				// data at 2x the resolution as above bEvent.triggerHighRes = raw->triggerRaw;

				bEvent.triggerHighRes = read_buffer16(readdata, 0x19);
				bEvent.touchpadHorizontalValid = 1;
				// bEvent.touchpadHorizontal = raw->touchpadHorizontal;
				bEvent.touchpadHorizontal = read_buffer16(readdata, 0x13);
				bEvent.touchpadVerticalValid = 1;
				// bEvent.touchpadVertical = raw->touchpadVertical;
				bEvent.touchpadVertical = read_buffer16(readdata, 0x15);

				// printf("%4.4x\n", bEvent.triggerHighRes);
				registerButtonEvent(obj, &bEvent);

				// printf("Buttons: %8.8x\n", raw->pressedButtons);
			}
		} else {
			SV_WARN("Unknown report id 0x%02x for %s", id, obj->codename);
		}
	} break;
	default: { SV_WARN("Unknown interface %d for %s", si->which_interface_am_i, obj->codename); }
	}

}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static int LoadConfig(SurviveViveData *sv, struct SurviveUSBInfo *usbInfo, int iface) {
	SurviveContext *ctx = sv->ctx;
	char *ct0conf = 0;

	bool extra_magic = usbInfo->device_info->type == USB_DEV_WATCHMAN1;

	int *cnt = (sv->cnt_per_device_type + (usbInfo->device_info - KnownDeviceTypes));
	// Don't create an object for stuff without a codename; ie the HMD main board

	SurviveObject *so = usbInfo->so;

	int len = survive_get_config(&ct0conf, sv, usbInfo, iface, extra_magic);
	if (len < 0) {
		return len;
	}

	{
		char raw_fname[100];
		sprintf(raw_fname, "%s_config.json", so->codename);
		FILE *f = fopen(raw_fname, "w");
		fwrite(ct0conf, len, 1, f);
		fclose(f);
	}

	if (so)
		return sv->ctx->configproc(so, ct0conf, len);
	return -1;
}

int survive_vive_close(SurviveContext *ctx, void *driver) {
	SurviveViveData *sv = driver;
	for (int i = 0; i < sv->udev_cnt; i++) {
		survive_close_usb_device(&sv->udev[i]);
	}
	survive_vive_usb_close(sv);
	free(sv);
	return 0;
}

int DriverRegHTCVive(SurviveContext *ctx) {
	SurviveViveData *sv = SV_CALLOC(1, sizeof(SurviveViveData));
	sv->hmd_imu_index = sv->hmd_mainboard_index = -1;

	// Note: don't sleep for HTCVive, the handle_events call can block
	ctx->poll_min_time_ms = 0;
	survive_attach_configi(ctx, SECONDS_PER_HZ_OUTPUT_TAG, &sv->seconds_per_hz_output);
	if(sv->seconds_per_hz_output > 0) {
	  SV_INFO("Reporting usb hz in %d second intervals", sv->seconds_per_hz_output);
	}
	sv->ctx = ctx;

#ifdef _WIN32
	CreateDirectoryA("calinfo", NULL);
#elif defined WINDOWS
	mkdir("calinfo");
#else
	mkdir("calinfo", 0755);
#endif

	// USB must happen last.
	if (survive_usb_init(sv)) {
		// TODO: Cleanup any libUSB stuff sitting around.
		SV_WARN("USB Init failed");
		goto fail_gracefully;
	}

	if (sv->udev_cnt) {
		survive_add_driver(ctx, sv, survive_vive_usb_poll, survive_vive_close, survive_vive_send_magic);
	} else {
		SV_INFO("No USB devices detected");
		goto fail_gracefully;
	}

#ifdef HIDAPI
	/*
	// Tricky: use other interface for actual lightcap.  XXX THIS IS NOT YET RIGHT!!!
	if (sv->udev[USB_DEV_HMD_IMU_LHB] && AttachInterface(sv, hmd, USB_IF_HMD_LIGHTCAP, sv->udev[USB_DEV_HMD_IMU_LHB],
														 0x82, survive_data_cb, "Lightcap")) {
		return -12;
	}

	// This is a HACK!  But it works.  Need to investigate further
	sv->uiface[USB_DEV_TRACKER0_LIGHTCAP].actual_len = 64;
	sv->uiface[USB_DEV_TRACKER1_LIGHTCAP].actual_len = 64;
*/
#endif

	return 0;
fail_gracefully:
	survive_vive_usb_close(sv);
	free(sv);
	return -1;
}

REGISTER_LINKTIME(DriverRegHTCVive)
