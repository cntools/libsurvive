
#ifdef HIDAPI
#if defined(WINDOWS) || defined(WIN32) || defined(_WIN32)
#include <windows.h>
#undef WCHAR_MAX
#endif
#include <hidapi.h>
#define HID_NONBLOCKING
#else
#ifdef __FreeBSD__
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif
#endif

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

struct SurviveViveData;

struct SurviveUSBInterface;

typedef void (*usb_callback)(struct SurviveUSBInterface *ti);
#ifdef HIDAPI
struct HIDAPI_USB_Handle_t {
	hid_device *interfaces[8];
};
#define USBHANDLE struct HIDAPI_USB_Handle_t *
#define USB_INTERFACE_HANDLE hid_device *
#else
#define USBHANDLE libusb_device_handle *
#define USB_INTERFACE_HANDLE void *
#endif

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
	SurviveObject *assoc_obj;
	int actual_len;
	uint8_t buffer[INTBUFFSIZE];
	usb_callback cb;
	int which_interface_am_i; // for indexing into uiface
	const char *hname;		  // human-readable names
	size_t packet_count;
} SurviveUSBInterface;

void survive_dump_buffer(SurviveContext *ctx, uint8_t *data, size_t length);
void survive_data_cb(SurviveUSBInterface *si);
int parse_watchman_lightcap(struct SurviveContext *ctx, const char *codename, uint8_t time1,
							survive_timecode reference_time, uint8_t *readdata, size_t qty, LightcapElement *les,
							size_t output_cnt);