#pragma once

#ifdef HIDAPI
#if defined(WINDOWS) || defined(WIN32) || defined(_WIN32)
#include <windows.h>
#undef WCHAR_MAX
#endif
#include <hidapi.h>
#define HID_NONBLOCKING
#else
#ifdef SURVIVE_LIBUSB_NO_DIR
#include <libusb.h>
#elif defined(SURVIVE_LIBUSB_UNVER_DIR)
#include <libusb/libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif
#endif

#include "os_generic.h"

#define MAX_USB_DEVS 32

enum USB_DEV_t {
	USB_DEV_HMD = 0,
	USB_DEV_HMD_IMU_LH,
	USB_DEV_WATCHMAN1,
	USB_DEV_WATCHMAN2,
	USB_DEV_TRACKER0,
	USB_DEV_TRACKER1,
	USB_DEV_W_WATCHMAN1, // Wired Watchman attached via USB
};

#define MAX_INTERFACES_PER_DEVICE 8
enum USB_IF_t {
	USB_IF_HMD_HEADSET_INFO = 1,
	USB_IF_HMD_IMU,
	USB_IF_WATCHMAN1,
	USB_IF_WATCHMAN2,
	USB_IF_TRACKER0_IMU,

	USB_IF_TRACKER_INFO,
	USB_IF_TRACKER1_IMU,
	USB_IF_W_WATCHMAN1_IMU,

	USB_IF_HMD_LIGHTCAP,
	USB_IF_TRACKER0_LIGHTCAP,
	USB_IF_TRACKER1_LIGHTCAP,
	USB_IF_W_WATCHMAN1_LIGHTCAP,

	USB_IF_HMD_BUTTONS,
	USB_IF_TRACKER0_BUTTONS,
	USB_IF_TRACKER1_BUTTONS,
	USB_IF_W_WATCHMAN1_BUTTONS,
	MAX_INTERFACES
};

SURVIVE_EXPORT const char *survive_usb_interface_str(enum USB_IF_t iface);

struct SurviveViveData;

struct SurviveUSBInterface;

typedef void (*usb_callback)(uint64_t time_recv_us, struct SurviveUSBInterface *ti);
#ifdef HIDAPI
struct HIDAPI_USB_Handle_t {
	hid_device *interfaces[8];
};
typedef struct HIDAPI_USB_Handle_t libsurvive_usb_handle;
#define USB_INTERFACE_HANDLE hid_device *
#else
typedef libusb_device_handle libsurvive_usb_handle;
#define USB_INTERFACE_HANDLE void *
#endif
#define USBHANDLE libsurvive_usb_handle *
void survive_usb_handle_close(libsurvive_usb_handle *handle);

struct SurviveUSBInfo;

typedef struct SurviveUSBInterface {
	struct SurviveViveData *sv;
	SurviveContext *ctx;

#ifdef HIDAPI
	USB_INTERFACE_HANDLE uh;
#ifndef HID_NONBLOCKING
	og_thread_t servicethread;
#endif
#else
	struct libusb_transfer *transfer;
#endif
	struct SurviveUSBInfo *usbInfo;
	SurviveObject *assoc_obj;
	int actual_len;

	uint8_t *buffer;
	uint8_t swap_buffer[2][INTBUFFSIZE];
	uint8_t swap_buffer_idx;

	usb_callback cb;
	int which_interface_am_i; // for indexing into uiface
	const char *hname;		  // human-readable names
	size_t packet_count;

	uint32_t consecutive_timeouts;
	uint32_t time_constraint;
	uint32_t error_count;
	uint64_t last_submit_time, sum_submit_cb_time, sum_cb_time;
	uint32_t max_submit_time, max_cb_time, cb_time_violation;
	bool shutdown;
} SurviveUSBInterface;

SURVIVE_EXPORT void survive_dump_buffer(SurviveContext *ctx, const uint8_t *data, size_t length);
SURVIVE_EXPORT void survive_data_cb(uint64_t time_received_us, SurviveUSBInterface *si);
SURVIVE_EXPORT void survive_data_on_setup_write(SurviveObject *so, uint8_t bmRequestType, uint8_t bRequest,
												uint16_t wValue, uint16_t wIndex, const uint8_t *data, size_t length);
SURVIVE_EXPORT void survive_usb_feature_read(SurviveObject *, const uint8_t *data, size_t length);
SURVIVE_EXPORT int parse_watchman_lightcap(struct SurviveContext *ctx, const char *codename, uint8_t time1,
										   survive_timecode reference_time, uint8_t *readdata, size_t qty,
										   LightcapElement *les, size_t output_cnt);
SURVIVE_EXPORT void survive_handle_watchman(SurviveObject *w, uint64_t time_in_us, uint8_t *readdata);
struct SurviveUSBInfo;
SURVIVE_EXPORT struct SurviveUSBInfo *survive_vive_register_driver(SurviveObject *so, uint16_t vid, uint16_t pid);
