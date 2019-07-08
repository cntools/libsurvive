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
//#define DEBUG_WATCHMAN 1

#ifdef DEBUG_WATCHMAN
#define SV_VERBOSE SV_INFO
#else
#define SV_VERBOSE(...)
#endif

STATIC_CONFIG_ITEM(LHV2_ENABLE, "lhv2-experimental", 'i', "Allow system to work with lighthouse v2.", 0);

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

static uint8_t vive_magic_power_on[64] = {0x04, 0x78, 0x29, 0x38, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01};
static uint8_t vive_magic_enable_lighthouse[5] = {0x04};
static uint8_t vive_magic_enable_lighthouse_more[5] = {0x07, 0x02};
static uint8_t vive_magic_power_off[] = {
	0x04, 0x78, 0x29, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x30, 0x05, 0x77,
	0x00, 0xe4, 0xf7, 0x33, 0x00, 0xe4, 0xf7, 0x33, 0x00, 0x60, 0x6e, 0x72, 0x00, 0xb4, 0xf7, 0x33,
	0x00, 0x04, 0x00, 0x00, 0x00, 0x70, 0xb0, 0x72, 0x00, 0x90, 0xf7, 0x33, 0x00, 0x7c, 0xf8, 0x33,
	0x00, 0xd0, 0xf7, 0x33, 0x00, 0x3c, 0x68, 0x29, 0x65, 0x24, 0xf9, 0x33, 0x00, 0x00, 0x00, 0x00,
};
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
	 .magics = {MAGIC_CTOR(true, vive_magic_enable_lighthouse), MAGIC_CTOR(true, vive_magic_enable_lighthouse_more)}},
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
	{0}};

typedef struct SurviveUSBInterface SurviveUSBInterface;
typedef struct SurviveViveData SurviveViveData;

struct SurviveUSBInfo {
	USBHANDLE handle;
	const struct DeviceInfo *device_info;
	struct SurviveObject *so;

	size_t interface_cnt;
	SurviveUSBInterface interfaces[MAX_INTERFACES_PER_DEVICE];
};

struct SurviveViveData {
	SurviveContext *ctx;
	size_t udev_cnt;
	struct SurviveUSBInfo udev[MAX_USB_DEVS];
	struct libusb_context *usbctx;
	size_t read_count;
	int seconds_per_hz_output;
};

#ifdef HIDAPI
#ifndef HID_NONBLOCKING
og_sema_t GlobalRXUSBSem;
#endif
#endif

void survive_data_cb(SurviveUSBInterface *si);

// USB Subsystem
void survive_usb_close(SurviveContext *t);
static int survive_usb_init(SurviveViveData *sv);
int survive_usb_poll(SurviveContext *ctx);
static int survive_get_config(char **config, SurviveViveData *ctx, struct SurviveUSBInfo *, int iface,
							  int send_extra_magic);
static int survive_vive_send_magic(SurviveContext *ctx, void *drv, int magic_code, void *data, int datalen);

#ifdef HIDAPI
static void *HAPIReceiver(void *v) {

	SurviveUSBInterface *iface = v;
	USB_INTERFACE_HANDLE *hp = &iface->uh;

	if ((iface->actual_len = hid_read(*hp, iface->buffer, sizeof(iface->buffer))) > 0) {
		// if( iface->actual_len  == 52 ) continue;
		iface->packet_count++;
#ifndef HID_NONBLOCKING
		OGLockSema(GlobalRXUSBSem);
#endif
#if 0
		printf( "%d %d: ", iface->which_interface_am_i, iface->actual_len );
		int i;
		for( i = 0; i < iface->actual_len; i++ )
		{
			printf( "%02x ", iface->buffer[i] );
		}
		printf("\n" );
#endif
		survive_data_cb(iface);
#ifndef HID_NONBLOCKING
		OGUnlockSema(GlobalRXUSBSem);
#endif
	}
	if (iface->actual_len < 0) {
		SurviveContext* ctx = iface->sv->ctx;
		SV_WARN("Error in hid read: %d", iface->actual_len);
		iface->assoc_obj = 0;
	}
	// XXX TODO: Mark device as failed.
	return 0;
}
#else
static void handle_transfer(struct libusb_transfer *transfer) {
	SurviveUSBInterface *iface = transfer->user_data;
	if (iface->assoc_obj == 0) {
		return;
	}

	SurviveContext *ctx = iface->ctx;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Transfer problem %s %d with %s", libusb_error_name(transfer->status),
				 transfer->status, iface->hname);
		return;
	}

	iface->actual_len = transfer->actual_length;
	iface->cb(iface);
	iface->packet_count++;

	if (libusb_submit_transfer(transfer)) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Error resubmitting transfer for %s", iface->hname);
	}
}
#endif

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
	SV_INFO("Attaching %s(0x%x) for %s", hname, endpoint_num, assocobj ? assocobj->codename : "(unknown)");

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

/*
static void debug_cb( struct SurviveUSBInterface * si )
{
	int i;
	int len = si->actual_len;
	printf( "%16s: %d: ", si->hname, len );
	for( i = 0; i < len; i++ )
	{
		printf( "%02x ", si->buffer[i] );
	}
	printf( "\n" );
}*/

// XXX TODO: Redo this subsystem for setting/updating feature reports.

#ifdef HIDAPI

static inline int update_feature_report(USBHANDLE dev, uint16_t iface, uint8_t *data, int datalen) {
	int r = hid_send_feature_report(dev->interfaces[iface], data, datalen);
	// wprintf( L"HUR: (%p) %d (%d) [%d] %S\n", dev, r, datalen, data[0], hid_error(dev->interfaces[iface]) );

	return r;
}
static inline int getupdate_feature_report(USBHANDLE dev, uint16_t iface, uint8_t *data, size_t datalen) {
	int r = hid_get_feature_report(dev->interfaces[iface], data, datalen);
	//	printf( "HGR: (%p) %d (%d) (%d)\n", dev, r, datalen, data[0] );
	if (r == -1)
		return -9; // Pretend it's not a critical error
	return r;
}

#else

static inline int update_feature_report(libusb_device_handle *dev, uint16_t interface, uint8_t *data, int datalen) {
	//	int xfer;
	//	int r = libusb_interrupt_transfer(dev, 0x01, data, datalen, &xfer, 1000);
	//	printf( "XFER: %d / R: %d\n", xfer, r );
	//	return xfer;
	return libusb_control_transfer(dev, LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT,
								   0x09, 0x300 | data[0], interface, data, datalen, 1000);
}

static inline int getupdate_feature_report(libusb_device_handle *dev, uint16_t interface, uint8_t *data, int datalen) {

	int ret = libusb_control_transfer(dev, LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_IN,
									  0x01, 0x300 | data[0], interface, data, datalen, 1000);
	if (ret == -9)
		return -9;
	if (ret < 0)
		return -1;
	return ret;
}

#endif

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

#ifdef HIDAPI
typedef struct hid_device_info *survive_usb_device_t;
typedef struct hid_device_info *survive_usb_devices_t;

static int survive_usb_subsystem_init(SurviveViveData *sv) {
#ifndef HID_NONBLOCKING
	if (!GlobalRXUSBSem) {
		GlobalRXUSBSem = OGCreateSema();
		// OGLockSema( GlobalRXUSBSem );
	}
#endif
	return hid_init();
}
static int survive_get_usb_devices(SurviveViveData *sv, survive_usb_devices_t *devs) {
	*devs = hid_enumerate(0, 0);
	return 0;
}
static void survive_free_usb_devices(survive_usb_devices_t devs) { hid_free_enumeration(devs); }

typedef survive_usb_device_t survive_usb_device_enumerator;
static survive_usb_device_t get_next_device(survive_usb_device_enumerator *iterator, survive_usb_devices_t list) {
	if (*iterator == 0) {
		return *iterator = list;
	}

	do {
		*iterator = ((*iterator)->next);
	} while (*iterator && (*iterator)->interface_number != 0 && (*iterator)->interface_number != -1);

	return *iterator;
}

static int survive_get_ids(survive_usb_device_t d, uint16_t *idVendor, uint16_t *idProduct) {
	*idVendor = d->vendor_id;
	*idProduct = d->product_id;

	return 0;
}

static const char *survive_usb_error_name(int ret) { return ""; }

static int survive_open_usb_device(SurviveViveData *sv, survive_usb_device_t d, struct SurviveUSBInfo *usbInfo) {
	usbInfo->handle = calloc(1, sizeof(struct HIDAPI_USB_Handle_t));
	survive_usb_device_t c = d;

	struct SurviveContext *ctx = sv->ctx;
	

	survive_usb_devices_t devs;
	int ret = survive_get_usb_devices(sv, &devs);

	if (ret < 0) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Couldn't get list of USB devices %d (%s)", ret,
				 survive_usb_error_name(ret));
		return ret;
	}

	if (d->serial_number == 0) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Couldn't get serial number for device %s", usbInfo->device_info->name);
		return -1;
	}

	for (survive_usb_device_t c = devs; c; c = c->next) {
		int interface_num = c->interface_number;
		interface_num = interface_num < 0 ? 0 : interface_num;
		if (c && c->serial_number && wcscmp(c->serial_number, d->serial_number)==0) {
			
			usbInfo->handle->interfaces[interface_num] = hid_open_path(c->path);

			if (!usbInfo->handle->interfaces[interface_num]) {
				SV_INFO("Warning: Could not find vive device %04x:%04x", d->vendor_id, d->product_id);
				return -1;
			}
		}
	}

	survive_free_usb_devices(devs);

	return 0;
}
#else
typedef libusb_device *survive_usb_device_t;
typedef libusb_device **survive_usb_devices_t;

static int survive_usb_subsystem_init(SurviveViveData *sv) { return libusb_init(&sv->usbctx); }
static int survive_get_usb_devices(SurviveViveData *sv, survive_usb_devices_t *devs) {
	return libusb_get_device_list(sv->usbctx, devs);
}
static void survive_free_usb_devices(survive_usb_devices_t devs) { libusb_free_device_list(devs, 1); }

typedef int survive_usb_device_enumerator;
static survive_usb_device_t get_next_device(survive_usb_device_enumerator *iterator, survive_usb_devices_t list) {
	assert(iterator);
	return list[(*iterator)++];
}

static int survive_get_ids(survive_usb_device_t d, uint16_t *idVendor, uint16_t *idProduct) {
	struct libusb_device_descriptor desc;

	int ret = libusb_get_device_descriptor(d, &desc);
	*idVendor = 0;
	*idProduct = 0;
	if (ret)
		return ret;

	*idVendor = desc.idVendor;
	*idProduct = desc.idProduct;

	return ret;
}

static const char *survive_usb_error_name(int ret) { return libusb_error_name(ret); }

static int survive_open_usb_device(SurviveViveData *sv, survive_usb_device_t d, struct SurviveUSBInfo *usbInfo) {
	struct libusb_config_descriptor *conf;
	int ret = libusb_get_config_descriptor(d, 0, &conf);
	if (ret)
		return ret;

	const struct DeviceInfo *info = usbInfo->device_info;
	ret = libusb_open(d, &usbInfo->handle);

	uint16_t idVendor;
	uint16_t idProduct;
	survive_get_ids(d, &idVendor, &idProduct);

	SurviveContext *ctx = sv->ctx;
	if (!usbInfo->handle || ret) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Error: cannot open device \"%s\" with vid/pid %04x:%04x error %d (%s)",
				 info->name, idVendor, idProduct, ret, libusb_error_name(ret));
		return ret;
	}

	libusb_set_auto_detach_kernel_driver(usbInfo->handle, 1);
	for (int j = 0; j < conf->bNumInterfaces; j++) {
		if (libusb_claim_interface(usbInfo->handle, j)) {
			SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Could not claim interface %d of %s", j, info->name);
			return ret;
		}
	}

	SV_INFO("Successfully enumerated %s (%d) %04x:%04x", info->name, conf->bNumInterfaces, idVendor, idProduct);

	usleep(100000);

	return ret;
}
#endif

static void survive_close_usb_device(struct SurviveUSBInfo *usbInfo) {
	for (size_t j = 0; j < usbInfo->interface_cnt; j++) {
		usbInfo->interfaces[j].assoc_obj = 0;
	}
}

int survive_usb_init(SurviveViveData *sv) {
	SurviveContext *ctx = sv->ctx;
	const char *blacklist = survive_configs(ctx, "blacklist-devs", SC_GET, "-");

	int r = survive_usb_subsystem_init(sv);
	if (r) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "usb fault %d (%s)\n", r, survive_usb_error_name(r));
		return r;
	}

	survive_usb_devices_t devs;
	int ret = survive_get_usb_devices(sv, &devs);

	if (ret < 0) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Couldn't get list of USB devices %d (%s)", ret,
				 survive_usb_error_name(ret));
		return ret;
	}

	// Open all interfaces.

	bool has_hmd_mainboard = false;

	for (const struct DeviceInfo *info = KnownDeviceTypes; info->name; info++) {
		if (info == 0 || strstr(blacklist, info->name)) {
			continue;
		}

		survive_usb_device_enumerator e = 0;
		for (survive_usb_device_t d = 0; (d = get_next_device(&e, devs)) && sv->udev_cnt < MAX_USB_DEVS;) {
			uint16_t idVendor;
			uint16_t idProduct;
			int ret = survive_get_ids(d, &idVendor, &idProduct);

			if (ret < 0) {
				continue;
			}

			if (info->vid != idVendor || info->pid != idProduct) {
				continue;
			}

			if (info->type == USB_DEV_HMD && has_hmd_mainboard) {
				continue;
			}

			if (info->type == USB_DEV_HMD) {
				has_hmd_mainboard = true;
			}

			struct SurviveUSBInfo *usbInfo = &sv->udev[sv->udev_cnt++];
			usbInfo->handle = 0;
			usbInfo->device_info = info;

			ret = survive_open_usb_device(sv, d, usbInfo);

			if (ret) {
				SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT,
						 "Error: cannot open device \"%s\" with vid/pid %04x:%04x error %d (%s)", info->name, idVendor,
						 idProduct, ret, survive_usb_error_name(ret));
				sv->udev_cnt--;
				continue;
			}

			SV_INFO("Successfully enumerated %s %04x:%04x", info->name, idVendor, idProduct);
		}
	}
	survive_free_usb_devices(devs);

	SurviveObject *hmd = 0;
	int cnt_per_device_type[sizeof(KnownDeviceTypes) / sizeof(KnownDeviceTypes[0])] = {0};
	for (int i = 0; i < sv->udev_cnt; i++) {
		struct SurviveUSBInfo *usbInfo = &sv->udev[i];

		int *cnt = (cnt_per_device_type + (usbInfo->device_info - KnownDeviceTypes));

		if (usbInfo->device_info->codename[0] != 0) {
			char codename[4] = {0};
			strcpy(codename, usbInfo->device_info->codename);
			codename[2] += (*cnt);
			*cnt = *cnt + 1;

			SurviveObject *so = survive_create_device(ctx, "HTC", sv, codename, 0);
			survive_add_object(ctx, so);
			usbInfo->so = so;

			if (USB_DEV_HMD_IMU_LH == usbInfo->device_info->type) {
				hmd = so;
			}
		}
	}

	// There should only be one HMD, tie the mainboard interface to the surviveobject
	if (hmd) {
		for (int i = 0; i < sv->udev_cnt; i++) {
			struct SurviveUSBInfo *usbInfo = &sv->udev[i];
			if (USB_DEV_HMD == usbInfo->device_info->type) {
				usbInfo->so = hmd;
			}
		}
	}

	for (int i = 0; i < sv->udev_cnt; i++) {
		struct SurviveUSBInfo *usbInfo = &sv->udev[i];

		for (const struct Endpoint_t *endpoint = usbInfo->device_info->endpoints; endpoint->name; endpoint++) {
			int errorCode = AttachInterface(sv, usbInfo, endpoint, usbInfo->handle, survive_data_cb);

			if (errorCode != 0)
				return -errorCode;
		}
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

		for (const struct Magic_t *magic = usbInfo->device_info->magics; magic->magic; magic++) {
			if (magic->code == magic_code) {
				uint8_t *data = alloca(sizeof(uint8_t) * magic->length);
				memcpy(data, magic->magic, magic->length);

				r = update_feature_report(usbInfo->handle, 0, data, magic->length);
				if (r != magic->length && usbInfo->so)
					SV_WARN("Could not turn on %s(%d) (%d/%zu - %s)", usbInfo->so->codename, usbInfo->device_info->type,
							r, magic->length, survive_usb_error_name(r));
			}
		}
	}

#if 0
		//// working code to turn off a wireless controller:
		//{
		//	uint8_t vive_controller_off[64] = { 0xff, 0x9f, 0x04, 'o', 'f', 'f', '!' };
		//	//r = update_feature_report( sv->udev[USB_DEV_WATCHMAN1], 0, vive_controller_haptic_pulse, sizeof( vive_controller_haptic_pulse ) );
		//	r = update_feature_report(sv->udev[USB_DEV_WATCHMAN1], 0, vive_controller_off, sizeof(vive_controller_off));
		//	SV_INFO("UCR: %d", r);
		//	if (r != sizeof(vive_controller_off)) printf("OFF FAILED **************************\n"); // return 5;
		//	OGUSleep(1000);
		//}
#endif
		// if (sv->udev[USB_DEV_TRACKER0])
		//{
		//	static uint8_t vive_magic_power_on[64] = {  0x04, 0x78, 0x29, 0x38 };
		//	r = update_feature_report( sv->udev[USB_DEV_TRACKER0], 0, vive_magic_power_on, sizeof( vive_magic_power_on )
		//); 	if( r != sizeof( vive_magic_power_on ) ) return 5;
		//}

	SV_INFO("Powered unit on.");

	return 0;
}

int survive_vive_send_haptic(SurviveObject *so, uint8_t reserved, uint16_t pulseHigh, uint16_t pulseLow,
							 uint16_t repeatCount) {
	SurviveViveData *sv = (SurviveViveData *)so->driver;
	SurviveContext *ctx = so->ctx;

	if (NULL == sv) {
		return -500;
	}

	int r;
	uint8_t vive_controller_haptic_pulse[64] = {
		0xff,
		0x8f,
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
	int i;
#ifdef HIDAPI
	for (i = 0; i < sv->udev_cnt; i++) {
		for (int j = 0; j < 8; j++) {
			hid_close(sv->udev[i].handle->interfaces[j]);
		}

		free(sv->udev[i].handle);
#ifndef HID_NONBLOCKING
		for (int j = 0; j < MAX_INTERFACES_PER_DEVICE; j++) {
			OGJoinThread(sv->udev[i].interfaces->servicethread);
		}
#endif
	}
	// This is global, don't do it on account of other tasks.
	// hid_exit();

#else
	for (i = 0; i < sv->udev_cnt; i++) {
		libusb_close(sv->udev[i].handle);
	}
	libusb_exit(sv->usbctx);
#endif
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
		for (int i = 0; i < sv->udev_cnt; i++) {
			if (sv->udev[i].so == 0)
				continue;

			for (int j = 0; j < sv->udev[i].interface_cnt; j++) {
				SurviveUSBInterface *iface = &sv->udev[i].interfaces[j];
				SV_INFO("Iface %s %s has %zu packets (%f hz)", iface->assoc_obj->codename, iface->hname,
						iface->packet_count, iface->packet_count / (now - start));
			}
		}
	}

#ifdef HIDAPI
#ifdef HID_NONBLOCKING
	for (int i = 0; i < sv->udev_cnt; i++) {
		for (int j = 0; j < sv->udev[i].interface_cnt; j++) {
			SurviveUSBInterface* iface = &sv->udev[i].interfaces[j];
			if (iface->assoc_obj)
				HAPIReceiver(iface);
		}
	}
#else
	OGUnlockSema(GlobalRXUSBSem);
	OGUSleep(1);
	OGLockSema(GlobalRXUSBSem);
	return 0;
#endif
#else
	int r = libusb_handle_events(sv->usbctx);
	if (r) {
		SurviveContext *ctx = sv->ctx;
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Libusb poll failed. %d (%s)", r, libusb_error_name(r));
	}
#endif
	return 0;
}

static int survive_get_config(char **config, SurviveViveData *sv, struct SurviveUSBInfo *usbInfo, int iface,
							  int send_extra_magic) {
	SurviveContext *ctx = sv->ctx;
	int ret, count = 0, size = 0;
	uint8_t cfgbuff[256];
	uint8_t compressed_data[8192];
	uint8_t uncompressed_data[65536];
	USBHANDLE dev = usbInfo->handle;

	if (send_extra_magic) {
		uint8_t cfgbuffwide[257];

		memset(cfgbuffwide, 0, sizeof(cfgbuff));
		cfgbuffwide[0] = 0x01;
		ret = hid_get_feature_report_timeout(dev, iface, cfgbuffwide, sizeof(cfgbuffwide));
		OGUSleep(1000);

		int k;

		uint8_t cfgbuff_send[64] = {0xff, 0x83};

#ifdef HIDAPI
		// XXX TODO WRITEME
		for (k = 0; k < 10; k++) {
			OGUSleep(1000);
		}

#else
		// Switch mode to pull config?
		for (k = 0; k < 10; k++) {
			update_feature_report(dev, iface, cfgbuff_send, 64);
			OGUSleep(1000);
		}
#endif

		cfgbuffwide[0] = 0xff;
		ret = hid_get_feature_report_timeout(dev, iface, cfgbuffwide, sizeof(cfgbuffwide));
		OGUSleep(1000);
	}

	// Send Report 16 to prepare the device for reading config info
	memset(cfgbuff, 0, sizeof(cfgbuff));
	cfgbuff[0] = 0x10;
	if ((ret = hid_get_feature_report_timeout(dev, iface, cfgbuff, sizeof(cfgbuff))) < 0) {
		if (usbInfo->device_info->type == USB_DEV_WATCHMAN1) {
			SV_INFO("%s couldn't configure; probably turned off", usbInfo->so->codename);
		} else {
			SV_WARN("Could not get survive config data for device %s:%d", usbInfo->device_info->name, iface);
		}
		return -1;
	}

	// Now do a bunch of Report 17 until there are no bytes left
	cfgbuff[1] = 0xaa;
	cfgbuff[0] = 0x11;
	do {
		if ((ret = hid_get_feature_report_timeout(dev, iface, cfgbuff, sizeof(cfgbuff))) < 0) {
			SV_INFO("Could not read config data (after first packet) on device %s:%d (count: %d)",
					usbInfo->device_info->name, iface, count);
			return -2;
		}

		size = cfgbuff[1];

		if (!size)
			break;

		if (size > (sizeof(cfgbuff) - 2)) {
			SV_INFO("Too much data (%d) on packet from config for device %s:%d (count: %d)", size,
					usbInfo->so->codename, iface, count);
			return -3;
		}

		if (count + size >= sizeof(compressed_data)) {
			SV_INFO("Configuration length too long %s:%d (count: %d)", usbInfo->so->codename, iface, count);
			return -4;
		}

		memcpy(&compressed_data[count], cfgbuff + 2, size);
		count += size;
	} while (1);

	if (count == 0) {
		SV_INFO("Empty configuration for %s:%d", usbInfo->so->codename, iface);
		return -5;
	}

	SV_INFO("Got config data length %d", count);

	int len = survive_simple_inflate(ctx, compressed_data, count, uncompressed_data, sizeof(uncompressed_data) - 1);
	if (len <= 0) {
		SV_INFO("Error: data for config descriptor %s:%d is bad. (%d)", usbInfo->so->codename, iface, len);
		return -5;
	}

	*config = malloc(len + 1);
	memcpy(*config, uncompressed_data, len);

	char fstname[128];
	sprintf(fstname, "calinfo/%s.json", usbInfo->so->codename);
	FILE *f = fopen(fstname, "wb");
	fwrite(uncompressed_data, len, 1, f);
	fclose(f);

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

void calibrate_acc(SurviveObject *so, FLT *agm) {
	agm[0] *= so->acc_scale[0];
	agm[1] *= so->acc_scale[1];
	agm[2] *= so->acc_scale[2];

	agm[0] -= so->acc_bias[0];
	agm[1] -= so->acc_bias[1];
	agm[2] -= so->acc_bias[2];
}

void calibrate_gyro(SurviveObject *so, FLT *agm) {
	agm[0] *= so->gyro_scale[0];
	agm[1] *= so->gyro_scale[1];
	agm[2] *= so->gyro_scale[2];

	agm[0] -= so->gyro_bias[0];
	agm[1] -= so->gyro_bias[1];
	agm[2] -= so->gyro_bias[2];
}

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
#define POP_BYTE(ptr) ((uint8_t) * (ptr++))
#define POP_SHORT(ptr) (((((struct unaligned_u16_t *)((ptr += 2) - 2))))->v)

#define HAS_FLAG(flags, flag) ((flags & flag) == flag)

char hexstr[512];
static char *packetToHex(uint8_t *packet, uint8_t *packetEnd) {
	int count = packetEnd - packet;
	int i;
	for (i = 0; i < count; i++)
		sprintf(&hexstr[i * 3], "%02x ", packet[i]);
	hexstr[i * 3] = 0;

	return hexstr;
}

struct sensorData {
	uint8_t sensorId;
	uint8_t edgeCount;
};

static size_t read_light_data(SurviveObject *w, uint16_t time, uint8_t **readPtr, uint8_t *payloadEndPtr,
							  LightcapElement *output, int output_cnt) {
	uint8_t *payloadPtr = *readPtr;
	SurviveContext *ctx = w->ctx;
	uint32_t reference_time = w->activations.last_imu;

	// DEBUG
	if ((*payloadPtr & 0xE0) == 0xE0) {
		SV_INFO("Warn : Light contains probable non-light data : 0x%02hX [Time:%04hX] [Payload: %s]", *payloadPtr, time,
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
	 * The meaning and associated led with each 'event' is dermined by the edge count as encoded within the
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
	SV_VERBOSE("Packet Start Time: %u", lastEventTime);

	while (idsPtr < eventPtr) {
		// There are two time deltas per 'event'
		if (timeIndex % 2 == 0) {
			sensors[timeIndex >> 1].sensorId = ((*idsPtr) >> 3) & 0x1F;
			sensors[timeIndex >> 1].edgeCount = (*idsPtr) & 0x7;
			idsPtr++;
		}

		// Obtain the timing to the previous event
		// Variable length encoding [if bit 8 is 0, continue into next byte]
		uint32_t timeDelta = 0;
		while (true) {
			timeDelta <<= 7;
			timeDelta |= (*eventPtr & 0x7F);
			if (((*(eventPtr--)) & 0x80) == 0x80)
				break;
			if (idsPtr > eventPtr) {
				SV_WARN("Light data parse error 1");
				return -1;
			}
		}
		lastEventTime -= timeDelta;

		// Store the event time
		times[++timeIndex] = lastEventTime;
		SV_VERBOSE("Time: [%zd] %u (%u)", timeIndex, lastEventTime, timeDelta);
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
				SV_WARN("Light data parse error 2");
				return -2;
			}
		if (timeIndex >= maxTimeIndex) {
			SV_WARN("Light data parse error 3");
			return -3;
		}

		// Get the start time
		size_t startTimeIndex = timeIndex + (sensors[i].edgeCount + 1);
		if (startTimeIndex >= maxTimeIndex) {
			SV_WARN("Light data parse error 4");
			return -4;
		}

		// Store the start index so we can return in ascending time order
		assert(reportOrder[startTimeIndex] == 0);
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
			assert(ol->length != 0 || ol->timestamp != 0);

			*(output++) = *ol;
			output_cnt--;
			// SV_INFO("Light Event [Ordered]: %i [%i] %li -> %li (%li)", i, ol->sensor_id, ol->timestamp, ol->timestamp
			// + ol->length, ol->length);
		}
	}
	return eventCount;
}

static void read_imu_data(SurviveObject *w, uint16_t time, uint8_t **readPtr, uint8_t *payloadEndPtr) {
	uint8_t *payloadPtr = *readPtr;

	SurviveContext *ctx = w->ctx;

	// First byte is higher res time, followed by 6 shorts
	uint8_t timeLSB = POP_BYTE(payloadPtr);
	int16_t aX = POP_SHORT(payloadPtr);
	int16_t aY = POP_SHORT(payloadPtr);
	int16_t aZ = POP_SHORT(payloadPtr);
	int16_t rX = POP_SHORT(payloadPtr);
	int16_t rY = POP_SHORT(payloadPtr);
	int16_t rZ = POP_SHORT(payloadPtr);

	FLT agm[9] = {aX, aY, aZ, rX, rY, rZ};

	calibrate_acc(w, agm);
	calibrate_gyro(w, agm + 3);

	w->ctx->imuproc(w, 3, agm, ((uint32_t)time << 16) | (timeLSB << 8), 0);

	*readPtr = payloadPtr;
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

	uint8_t flags = POP_BYTE(payloadPtr);

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

#ifdef KNUCKLES_INFO
				SV_INFO("KAS: @%04hX | Grip    [Proximity: %02X %02X %02X %02X] [Touch: %s%s%s%s%s (%02X)] [Grip: %02X "
						"%02X]",
						time, fingerProximity[3], fingerProximity[0], fingerProximity[1], fingerProximity[2],
						HAS_FLAG(touchFlags, 0x01) ? "#" : "_", HAS_FLAG(touchFlags, 0x08) ? "#" : "_",
						HAS_FLAG(touchFlags, 0x10) ? "#" : "_", HAS_FLAG(touchFlags, 0x20) ? "#" : "_",
						HAS_FLAG(touchFlags, 0x40) ? "#" : "_", touchFlags, gripForce, trackpadForce);
#endif
			} else {
				SV_WARN("Unknown gen two event 0x%02hX [Time:%04hX] [Payload: %s] <<ABORT FURTHER READ>>",
						*(payloadPtr - 1), time, packetToHex(payloadPtr, payloadEndPtr));
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
			read_event(w, time, &payloadPtr, payloadEndPtr);

			UPDATE_PTR_AND_RETURN
		}
	}

	// Read off any IMU data present
	if (flagIMU)
		read_imu_data(w, time, &payloadPtr, payloadEndPtr);

	UPDATE_PTR_AND_RETURN
}

static void handle_watchman(SurviveObject *w, uint8_t *readdata) {
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

	// Read any non-light events that may be in the packet
	if (!read_event(w, time, &payloadPtr, payloadEndPtr))
		return;

	// Any remaining data after events (if any) have been read off is light data
	if (payloadPtr < payloadEndPtr) {
		LightcapElement les[10] = {0};
		size_t cnt = read_light_data(w, time, &payloadPtr, payloadEndPtr, les, 10);

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

void survive_data_cb(SurviveUSBInterface *si) {
	int size = si->actual_len;
	SurviveContext *ctx = si->ctx;

	int iface = si->which_interface_am_i;
	SurviveObject *obj = si->assoc_obj;
	uint8_t *readdata = si->buffer;

	if (iface == USB_IF_HMD_HEADSET_INFO && obj == 0)
		return;

	int id = POP1;
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

				calibrate_acc(obj, agm);
				calibrate_gyro(obj, agm + 3);
				ctx->imuproc(obj, 3, agm, timecode, code);
			}
		}
		if (id != 32) {
			int a = 0; // set breakpoint here
		}
		// DONE OK.
		break;
	}
	case USB_IF_WATCHMAN1:
	case USB_IF_WATCHMAN2: {
		SurviveObject *w = obj;
		if (id == 35) {
			assert(size == 30);
			handle_watchman(w, readdata);
		} else if (id == 36) {
			assert(size == 29 * 2 + 1);
			handle_watchman(w, readdata);
			handle_watchman(w, readdata + 29);
		} else if (id == 38) {
			w->ison = 0; // turning off
		} else {
			SV_INFO("Unknown watchman code %d\n", id);
		}
		break;
	}
	case USB_IF_HMD_LIGHTCAP:
	case USB_IF_TRACKER1_LIGHTCAP: {
		if (id == 37) { // LHv1
			int i;
			for (i = 0; i < 9; i++) {
				LightcapElement le;
				le.sensor_id = POP1;
				le.length = POP2;
				le.timestamp = POP4;
				if (le.sensor_id > 0xfd)
					continue;
				//SV_INFO("%d %d %d %d %d", id, le.sensor_id, le.length, le.timestamp, si->buffer + size - readdata);

				if (obj->ctx->lh_version == 0) {
					handle_lightcap(obj, &le);
				} else {
					fprintf(stderr, "sensor: %2d         time: %8u length: %4d end_time: %8u\n", le.sensor_id,
							le.timestamp, le.length, le.length + le.timestamp);
				}
			}
		} else if (id == 39) { // LHv2
			if (obj->ctx->lh_version == 0) {
				bool allowExperimental = (bool)survive_configi(ctx, "lhv2-experimental", SC_GET, 0);
				if (!allowExperimental) {
					if (obj->ctx->currentError == SURVIVE_OK) {
						SV_ERROR(SURVIVE_ERROR_INVALID_CONFIG,
								 "System detected lighthouse v2 system. Currently, libsurvive does not work with this "
								 "setup. If you want to see debug information for this system, pass in "
								 "'--lhv2-experimental'");
					}
					return;
				}

				obj->ctx->lh_version = 1;
			}

#pragma pack(push, 1)
			struct lh2_entry {
				uint8_t code; // sensor with some bit flag. Continuation flag?
				uint32_t time;
				uint8_t data[8];
			};
#pragma pack(pop)

			struct lh2_entry *entries = (struct lh2_entry *)readdata;
			static uint32_t last_time = 0;
			for (int i = 0; i < 4; i++) {
				struct lh2_entry *entry = &entries[i];
				if (entry->code == 0xff)
					break;
				fprintf(stderr, "sensor: %2u flag: %u time: %8u (%7u) %f ", entry->code & 0x7f, (entry->code & 0x80) > 0,
						entry->time, entry->time - last_time, entry->time / (48000000.));

				for (int j = 0; j < 8; j++) {
					for (int k = 0; k < 8; k++)
						fprintf(stderr, "%d", ((entry->data[j] >> (8 - k - 1)) & 1));
				}

				last_time = entry->time;
				fprintf(stderr, "\n");
			}

			for (int i = 59 - 7; i < 59; i++) {
				fprintf(stderr, "%02x ", readdata[i]);
			}
			fprintf(stderr, "\n");
		} else if (id == 40) {
			uint8_t *packet = readdata + 1;
			uint8_t length = readdata[0];
			uint8_t idx = 0;
			bool dump_binary = false;
			uint8_t channel = -1;
			while (idx < length) {
				uint8_t data = packet[idx];

				if (data & 0x1u) {
					// Since they flag for this; I assume multiples can appear in a single packet. Need to plug in
					// second LH to find out...

					if ((data & 0x0Fu) != 1) {
						// Currently I've only ever seen 0x1 if the 1 bit is set; I doubt they left 3 bits on the table
						// though....
						fprintf(stderr, "Not entirely sure what this data is; errors may occur\n");
						dump_binary = true;
					}

					// encodes like so: 0bcccc ???C
					channel = data >> 4u;
					idx++;
				} else {
					uint32_t timecode = 0;
					memcpy(&timecode, packet + idx, sizeof(uint32_t));

					uint32_t reference_time = (obj->activations.last_imu) & 0xFF000000;

					bool sync = timecode & 0x2u;
					if (!sync) {
						// encodes like so: 0bXXXX ABTTT TTTT TTTT TTTT TTTT TTTT TTSC
						bool ootx = (timecode >> 26u) & 1u;
						bool g = (timecode >> 27u) & 1u;
						timecode = reference_time | (timecode >> 2u) & 0xFFFFFF;
						fprintf(stderr, "Sync   ch%2d  %d %d %12d\n", channel, ootx, g, timecode);

						obj->last_sync_time[0] = timecode;
					} else {
						// encodes like so: 0bSSSS STTT TTTT TTTT TTTT TTTT TTTT TFSC
						bool flag = timecode & 0x4u; // ?? Seems to slightly change time? Is clock now 96mhz?
						uint8_t sensor = (timecode >> 27u);
						timecode = reference_time | (timecode >> 3u) & 0xFFFFFF;
						fprintf(stderr, "Sensor ch%2d %2d %d %12d %6d\n", channel, sensor, flag, timecode,
								timecode - obj->last_sync_time[0]);
					}

					idx += 4;
				}
			}

			if (dump_binary) {
				for (int i = 0; i < size - 1; i++) {
					if ((i + 2) % 4 == 0)
						fprintf(stderr, "  ");
					fprintf(stderr, "%02x ", readdata[i]);
				}

				fprintf(stderr, "\n");
			}
		} else {
			SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "USB lightcap report is of an unknown type for %s: %d",
					 obj->codename, id)
		}

		break;
	}
	case USB_IF_W_WATCHMAN1_LIGHTCAP:
	case USB_IF_TRACKER0_LIGHTCAP: {
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
		break;

		if (id != 33) {
			int a = 0; // breakpoint here
		}
	}
	case USB_IF_TRACKER0_BUTTONS:
	case USB_IF_TRACKER1_BUTTONS:
	case USB_IF_W_WATCHMAN1_BUTTONS: {
		if (1 == id) {
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
			int a = 0;
		} else {
			int a = 0; // breakpoint here
		}
	}
	default: {
		int a = 0; // breakpoint here
	}
	}
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

static int LoadConfig(SurviveViveData *sv, struct SurviveUSBInfo *usbInfo, int iface) {
	SurviveContext *ctx = sv->ctx;
	char *ct0conf = 0;

	bool extra_magic = usbInfo->device_info->type == USB_DEV_WATCHMAN1;

	SurviveObject *so = usbInfo->so;
	int len = survive_get_config(&ct0conf, sv, usbInfo, iface, extra_magic);
	if (len < 0) {
		survive_remove_object(ctx, so);
		usbInfo->so = 0;
		return len;
	}

	{
		char raw_fname[100];
		sprintf(raw_fname, "%s_config.json", so->codename);
		FILE *f = fopen(raw_fname, "w");
		fwrite(ct0conf, strlen(ct0conf), 1, f);
		fclose(f);
	}

	return so->ctx->configfunction(so, ct0conf, len);
}

int survive_vive_close(SurviveContext *ctx, void *driver) {
	SurviveViveData *sv = driver;

	survive_vive_usb_close(sv);
	return 0;
}

int DriverRegHTCVive(SurviveContext *ctx) {
	SurviveViveData *sv = calloc(1, sizeof(SurviveViveData));

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

	for (int i = 0; i < sv->udev_cnt; i++) {
		struct SurviveUSBInfo *usbInfo = &sv->udev[i];
		if (usbInfo->device_info->type != USB_DEV_HMD) {
			int hasError = LoadConfig(sv, usbInfo, 0);

			// Powered off devices are stripped of their SurviveObject
			if (hasError != 0 && usbInfo->so) {
				SV_INFO("%s config issue.", usbInfo->so->codename);
			}

			if (hasError != 0) {
				survive_close_usb_device(usbInfo);
			}
		}
	}
	return 0;
fail_gracefully:
	survive_vive_usb_close(sv);
	free(sv);
	return -1;
}

REGISTER_LINKTIME(DriverRegHTCVive);
