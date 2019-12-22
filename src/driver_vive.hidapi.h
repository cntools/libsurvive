#include "driver_vive.h"
#include <assert.h>
#include <errno.h>
#include <hidapi.h>
#include <survive.h>

static inline int update_feature_report_async(USBHANDLE dev, uint16_t iface, uint8_t *data, int datalen) {
	errno = 0;
	int r = hid_send_feature_report(dev->interfaces[iface], data, datalen);
	assert(errno == 0);
	// wprintf( L"HUR: (%p) %d (%d) [%d] %S\n", dev, r, datalen, data[0], hid_error(dev->interfaces[iface]) );
	return r;
}

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

static void *HAPIReceiver(void *v) {
	SurviveUSBInterface *iface = v;
	USB_INTERFACE_HANDLE *hp = &iface->uh;

	if ((iface->actual_len = hid_read(*hp, iface->buffer, sizeof(iface->buffer))) > 0) {
		// if( iface->actual_len  == 52 ) continue;
		iface->packet_count++;
		survive_data_cb(iface);
	}
	if (iface->actual_len < 0) {
		SurviveContext *ctx = iface->sv->ctx;
		SV_WARN("Error in hid read: %d", iface->actual_len);
		iface->assoc_obj = 0;
	}
	// XXX TODO: Mark device as failed.
	return 0;
}

typedef struct hid_device_info *survive_usb_device_t;
typedef struct hid_device_info *survive_usb_devices_t;

static int survive_usb_subsystem_init(SurviveViveData *sv) { return hid_init(); }
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
	usbInfo->handle = SV_CALLOC(1, sizeof(struct HIDAPI_USB_Handle_t));
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
		if (c && c->serial_number && wcscmp(c->serial_number, d->serial_number) == 0) {

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
static void setup_hotplug(SurviveViveData *sv) {}

static inline void survive_close_usb_device(struct SurviveUSBInfo *usbInfo) {
	for (int j = 0; j < 8; j++) {
		hid_close(usbInfo->handle->interfaces[j]);
	}

	free(usbInfo->handle);
#ifndef HID_NONBLOCKING
	for (int j = 0; j < MAX_INTERFACES_PER_DEVICE; j++) {
		OGJoinThread(sv->udev[i].interfaces->servicethread);
	}
#endif
}

void survive_usb_close(SurviveViveData *sv) {}
