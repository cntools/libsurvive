#include "driver_vive.h"
#include <assert.h>
#include <errno.h>
#include <hidapi.h>
#include <survive.h>
#define LIBUSB_TRANSFER_COMPLETED 0
#define LIBUSB_TRANSFER_STALL -1
#define LIBUSB_TRANSFER_TIMED_OUT -2
#define LIBUSB_TRANSFER_NOT_RECENT -3

typedef struct survive_usb_transfer_t {
	enum survive_usb_transfer_type {
		survive_usb_transfer_type_read_feature_report,
		survive_usb_transfer_type_send_feature_report,
	} type;
	struct SurviveUSBInfo *dev;
	void (*cb)(struct survive_usb_transfer_t *);
	void *user_data;
	uint8_t *buffer;
	int status;
	bool processed;
	size_t length;
	size_t actual_length;
	uint32_t timeout;
} survive_usb_transfer_t;

static inline uint8_t *survive_usb_transfer_data(survive_usb_transfer_t *tx) { return tx->buffer; }

static inline void *survive_usb_transfer_alloc() { return calloc(1, sizeof(survive_usb_transfer_t)); }
static inline void survive_usb_transfer_free(survive_usb_transfer_t *tx) { free(tx); }
static inline int survive_usb_transfer_submit(survive_usb_transfer_t *tx) {
	int rtn = 0;
	// char tmp[64];
	// while (hid_read_timeout(tx->dev->handle->interfaces[0], tmp, sizeof(tmp), 0) > 0) {}

	switch (tx->type) {
	case survive_usb_transfer_type_read_feature_report: {
		rtn = hid_get_feature_report(tx->dev->handle->interfaces[0], tx->buffer, tx->actual_length);
		break;
	}
	case survive_usb_transfer_type_send_feature_report: {
		rtn = hid_send_feature_report(tx->dev->handle->interfaces[0], tx->buffer, tx->length);
		break;
	}
	}

	tx->processed = false;
	if (rtn > 0) {
		tx->status = LIBUSB_TRANSFER_COMPLETED;
		tx->length = rtn;
	} else if (rtn == -1) {
		tx->status = -2;
	} else {
		tx->status = LIBUSB_TRANSFER_TIMED_OUT;
	}

	return 0;
}
static inline void survive_usb_setup_get_feature_report(survive_usb_transfer_t *tx, uint8_t report_id) {
	tx->type = survive_usb_transfer_type_read_feature_report;
	tx->buffer[0] = report_id;
	tx->length = tx->actual_length;
}

static inline void survive_usb_setup_update_feature_report(survive_usb_transfer_t *tx, const uint8_t *data,
														   size_t datalen) {
	tx->type = survive_usb_transfer_type_send_feature_report;
	memcpy(tx->buffer, data, datalen);
	tx->length = datalen;
}

static inline void survive_usb_setup_control(survive_usb_transfer_t *tx, struct SurviveUSBInfo *usbInfo,
											 void (*cb)(survive_usb_transfer_t *), void *user, int32_t timeout) {
	tx->cb = cb;
	tx->dev = usbInfo;
	tx->user_data = user;
	tx->timeout = timeout;
}

static inline int update_feature_report_async(USBHANDLE dev, uint16_t iface, uint8_t *data, int datalen) {
	errno = 0;
	uint8_t buffer[255];
	memcpy(buffer, data, datalen);
	errno = 0;
	int r = hid_send_feature_report(dev->interfaces[iface], buffer, sizeof(buffer));
	if (r == -1) {
		// fprintf(stderr,"async: (%p) %d (%d) [%d] %S errno %d\n", dev, r, datalen, data[0],
		// hid_error(dev->interfaces[iface]), errno);
		return -1;
	}
	return datalen;
}

static inline int update_feature_report(USBHANDLE dev, uint16_t iface, uint8_t *data, int datalen) {
	int r = update_feature_report_async(dev, iface, data, datalen);
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
		survive_data_cb(OGGetAbsoluteTimeUS(), iface);
	}
	if (iface->actual_len < 0) {
		SurviveContext *ctx = iface->sv->ctx;
		SV_WARN("Error in hid read: %d", iface->actual_len);
		iface->usbInfo->request_close = true;
		// iface->assoc_obj = 0;
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
void survive_usb_handle_close(libsurvive_usb_handle *handle) {}
static int survive_get_ids(survive_usb_device_t d, uint16_t *idVendor, uint16_t *idProduct, uint8_t *class_id) {
	*idVendor = d->vendor_id;
	*idProduct = d->product_id;
	*class_id = 1;

	return 0;
}

static const char *survive_usb_error_name(int ret) { return ""; }

static int survive_open_usb_device(SurviveViveData *sv, survive_usb_device_t d, struct SurviveUSBInfo *usbInfo) {
	usbInfo->handle = SV_CALLOC(sizeof(struct HIDAPI_USB_Handle_t));
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

	SV_VERBOSE(40, "Successfully enumerated %s at %.7f", survive_colorize(usbInfo->device_info->name),
			   survive_run_time(ctx));

	survive_free_usb_devices(devs);

	return 0;
}
typedef survive_usb_device_t survive_usb_device_enumerator;

static survive_usb_device_t get_next_device(survive_usb_device_enumerator* d, survive_usb_device_t head) {
	if (*d == 0) {
		return *d = head;
	}
	
	for (survive_usb_device_t c = (*d)->next; c; c = c->next) {
		if (c->vendor_id != (*d)->vendor_id || c->product_id != (*d)->product_id) {
			return *d = c; 
		}
	}
	return 0; 
}

int survive_vive_add_usb_device(SurviveViveData *sv, survive_usb_device_t d);
static bool setup_hotplug(SurviveViveData *sv) { 
	return true; 
}

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

static inline int hid_get_feature_report_timeout(USBHANDLE device, uint16_t iface, unsigned char *buf, size_t len) {
	int ret;
	uint8_t i = 0;
	for (i = 0; i < 50; i++) {
		ret = getupdate_feature_report(device, iface, buf, len);
		if (ret != -9 && (ret != -1 || errno != EPIPE))
			return ret;
		OGUSleep(1);
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

		uint8_t cfgbuff_send[64] = {VIVE_REPORT_COMMAND, 0x83};
		cfgbuffwide[0] = VIVE_REPORT_COMMAND;
		ret = get_feature_report_timeout_locked(ctx, dev, iface, cfgbuffwide, sizeof(cfgbuffwide));
	}

	// Send Report 16 to prepare the device for reading config info
	memset(cfgbuff, 0, sizeof(cfgbuff));
	cfgbuff[0] = VIVE_REPORT_CONFIG_READMODE;
	if ((ret = get_feature_report_timeout_locked(ctx, dev, iface, cfgbuff, sizeof(cfgbuff))) < 0) {
		if (usbInfo->device_info->type == USB_DEV_WATCHMAN1) {

		} else {
			SV_WARN("Could not get survive config data for device %s:%d", usbInfo->device_info->name, iface);
		}
		return -1;
	}

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
		/*if (count == 0 && size >= 2 && cfgbuff[2] != 0x78) {
			SV_INFO("Got preamble of %x %x %x", cfgbuff[0], cfgbuff[1], cfgbuff[2]);
			continue;
		}*/

		memcpy(&compressed_data[count], cfgbuff + 2, size);
		count += size;
	} while (1);

	if (count == 0) {
		SV_INFO("Empty configuration for %s:%d", name, iface);
		return -5;
	}

	SV_VERBOSE(50, "Got config data length %d for %s:%d", count, name, iface);

	int len = survive_simple_inflate(ctx, compressed_data, count, uncompressed_data, sizeof(uncompressed_data) - 1);
	if (len <= 0) {
		SV_INFO("Error: data for config descriptor %s:%d is bad. (%d)", name, iface, len);
		return -5;
	}

	*config = SV_MALLOC(len + 1);
	memcpy(*config, uncompressed_data, len);

	memcpy(cfgbuff, vive_request_version_info, sizeof(vive_request_version_info));
	if ((ret = get_feature_report_timeout_locked(ctx, dev, iface, cfgbuff, 0x42)) < 0) {
		SV_INFO("Could not read config data (after first packet) on device %s:%d (count: %d)",
				usbInfo->device_info->name, iface, count);
		return -2;
	} else {
		parse_tracker_version_info(usbInfo->so, cfgbuff + 1, ret - 1);
	}

	return len;
}

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

	if (so) {
		SV_VERBOSE(10, "Successfully configured %s", so->codename);
		return sv->ctx->configproc(so, ct0conf, len);
	}
	return -1;
}

static void send_devices_magics(SurviveContext *ctx, struct SurviveUSBInfo *usbInfo) {
	for (const struct Magic_t *magic = usbInfo->device_info->magics; magic->magic; magic++) {
		if (magic->code == 1) {
			usbInfo->lightcapMode = LightcapMode_raw0;
			uint8_t *data = alloca(sizeof(uint8_t) * magic->length);
			memcpy(data, magic->magic, magic->length);

			survive_release_ctx_lock(ctx);
			int r = update_feature_report(usbInfo->handle, 0, data, magic->length);
			survive_get_ctx_lock(ctx);

			if (r != magic->length && usbInfo->so)
				SV_WARN("Could not turn on %s(%d) (%d/%zu - %s)", usbInfo->so->codename, usbInfo->device_info->type, r,
						magic->length, survive_usb_error_name(r));
		}
	}
}

/*
static int survive_config_submit(struct SurviveUSBInfo *usbInfo, int iface) {
	double time = OGRelativeTime();
	SurviveViveData *sv = usbInfo->viveData;
	SurviveContext* ctx = sv->ctx;
	int err = LoadConfig(sv, usbInfo, 0);
	double diff_time = OGRelativeTime() - time;

	if (err == 0) {
		send_devices_magics(ctx, usbInfo);
		usbInfo->nextCfgSubmitTime = 0;
	} else {
		usbInfo->nextCfgSubmitTime = survive_run_time(usbInfo->so->ctx) + .1;
	}
	return err;
}
*/
/*
static int survive_start_get_config(SurviveViveData *sv, struct SurviveUSBInfo *usbInfo, int iface) {
	usbInfo->nextCfgSubmitTime = survive_run_time(usbInfo->so->ctx);
	return 0;
}
 */