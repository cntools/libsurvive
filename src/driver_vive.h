
#ifdef HIDAPI
#if defined(WINDOWS) || defined(WIN32) || defined(_WIN32)
#include <windows.h>
#undef WCHAR_MAX
#endif
#include <hidapi.h>
#else
#ifdef __FreeBSD__
#include <libusb.h>
#else
#include <libusb-1.0/libusb.h>
#endif
#endif

enum {
	USB_DEV_HMD = 0,
	USB_DEV_HMD_IMU_LH,
	USB_DEV_WATCHMAN1,
	USB_DEV_WATCHMAN2,
	USB_DEV_TRACKER0,
	USB_DEV_TRACKER1,
	USB_DEV_W_WATCHMAN1, // Wired Watchman attached via USB
#ifdef HIDAPI
	USB_DEV_HMD_IMU_LHB,
	USB_DEV_TRACKER0_LIGHTCAP,
	USB_DEV_TRACKER1_LIGHTCAP,
	USB_DEV_W_WATCHMAN1_LIGHTCAP,

	USB_DEV_HMD_BUTTONS,
	USB_DEV_TRACKER0_BUTTONS,
	USB_DEV_TRACKER1_BUTTONS,
	USB_DEV_W_WATCHMAN1_BUTTONS,
#endif
	MAX_USB_DEVS
};

enum {
	USB_IF_HMD = 0,
	USB_IF_HMD_IMU_LH,
	USB_IF_WATCHMAN1,
	USB_IF_WATCHMAN2,
	USB_IF_TRACKER0,
	USB_IF_TRACKER1,
	USB_IF_W_WATCHMAN1,

	USB_IF_LIGHTCAP,
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
#define USBHANDLE hid_device *
#else
#define USBHANDLE libusb_device_handle *
#endif

typedef struct SurviveUSBInterface {
	struct SurviveViveData *sv;
	SurviveContext *ctx;

#ifdef HIDAPI
	USBHANDLE uh;
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

void survive_data_cb(SurviveUSBInterface *si);
