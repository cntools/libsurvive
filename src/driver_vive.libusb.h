
static inline int update_feature_report(libusb_device_handle *dev, uint16_t interface, uint8_t *data, int datalen) {
	//	int xfer;
	//	int r = libusb_interrupt_transfer(dev, 0x01, data, datalen, &xfer, 1000);
	//	printf( "XFER: %d / R: %d\n", xfer, r );
	//	return xfer;
	return libusb_control_transfer(dev, LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT,
								   0x09, 0x300 | data[0], interface, data, datalen, 1000);
}

void monitor_transfer(struct libusb_transfer *transfer) {
	assert(transfer->status == 0);
	libusb_free_transfer(transfer);
}

int libusb_control_transfer_async(libusb_device_handle *dev_handle, uint8_t bmRequestType, uint8_t bRequest,
								  uint16_t wValue, uint16_t wIndex, unsigned char *data, uint16_t wLength,
								  unsigned int timeout) {
	struct libusb_transfer *transfer = libusb_alloc_transfer(0);
	unsigned char *buffer;
	int r;
	if (!transfer)
		return LIBUSB_ERROR_NO_MEM;

	buffer = SV_MALLOC(LIBUSB_CONTROL_SETUP_SIZE + wLength);
	if (!buffer) {
		libusb_free_transfer(transfer);
		return LIBUSB_ERROR_NO_MEM;
	}
	libusb_fill_control_setup(buffer, bmRequestType, bRequest, wValue, wIndex, wLength);
	if ((bmRequestType & LIBUSB_ENDPOINT_DIR_MASK) == LIBUSB_ENDPOINT_OUT)
		memcpy(buffer + LIBUSB_CONTROL_SETUP_SIZE, data, wLength);
	libusb_fill_control_transfer(transfer, dev_handle, buffer, monitor_transfer, dev_handle, timeout);
	transfer->flags = LIBUSB_TRANSFER_FREE_BUFFER;
	r = libusb_submit_transfer(transfer);
	if (r < 0) {
		libusb_free_transfer(transfer);
		return r;
	}
	return wLength;
}

static inline int update_feature_report_async(libusb_device_handle *dev, uint16_t interface, uint8_t *data,
											  int datalen) {
	return libusb_control_transfer_async(dev,
										 LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT,
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

typedef libusb_device *survive_usb_device_t;
typedef libusb_device **survive_usb_devices_t;

static int survive_usb_subsystem_init(SurviveViveData *sv) {
	int rtn = libusb_init(&sv->usbctx);
#if LIBUSB_API_VERSION < 0x01000106
	libusb_set_debug(NULL, LIBUSB_LOG_LEVEL_WARNING);
#else
	libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
#endif
	return rtn;
}
static int survive_get_usb_devices(SurviveViveData *sv, survive_usb_devices_t *devs) {
	return libusb_get_device_list(sv->usbctx, devs);
}
static void survive_free_usb_devices(survive_usb_devices_t devs) { libusb_free_device_list(devs, 1); }

typedef int survive_usb_device_enumerator;
static survive_usb_device_t get_next_device(survive_usb_device_enumerator *iterator, survive_usb_devices_t list) {
	assert(iterator);
	return list[(*iterator)++];
}

int survive_vive_add_usb_device(SurviveViveData *sv, survive_usb_device_t d);
int libusb_hotplug(libusb_context *usbctx, libusb_device *device, libusb_hotplug_event event, void *user_data) {
	SurviveViveData *sv = user_data;
	SurviveContext *ctx = sv->ctx;

	if (event == LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED) {
		survive_vive_add_usb_device(sv, device);
	}

	return 0;
}
static bool setup_hotplug(SurviveViveData *sv) {
	libusb_hotplug_register_callback(sv->usbctx, LIBUSB_HOTPLUG_EVENT_DEVICE_ARRIVED | LIBUSB_HOTPLUG_EVENT_DEVICE_LEFT,
									 LIBUSB_HOTPLUG_NO_FLAGS, LIBUSB_HOTPLUG_MATCH_ANY, LIBUSB_HOTPLUG_MATCH_ANY,
									 LIBUSB_HOTPLUG_MATCH_ANY, libusb_hotplug, sv, 0);
	return 0;
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
		goto cleanup_and_rtn;

	const struct DeviceInfo *info = usbInfo->device_info;
	ret = libusb_open(d, &usbInfo->handle);

	uint16_t idVendor;
	uint16_t idProduct;
	survive_get_ids(d, &idVendor, &idProduct);

	SurviveContext *ctx = sv->ctx;
	if (!usbInfo->handle || ret) {
		SV_WARN("Error: cannot open device \"%s\" with vid/pid %04x:%04x error %d (%s)", info->name, idVendor,
				idProduct, ret, libusb_error_name(ret));
		goto cleanup_and_rtn;
	}

	libusb_set_auto_detach_kernel_driver(usbInfo->handle, 1);
	for (int j = 0; j < conf->bNumInterfaces; j++) {
		ret = libusb_claim_interface(usbInfo->handle, j);
		if (ret != 0) {
			SV_WARN("Could not claim interface %d of %s: %d %s", j, info->name, ret, libusb_error_name(ret));
			goto cleanup_and_rtn;
		}
	}

	SV_VERBOSE(40, "Successfully enumerated %s (%d) %04x:%04x", info->name, conf->bNumInterfaces, idVendor, idProduct);

	usleep(100000);

cleanup_and_rtn:
	libusb_free_config_descriptor(conf);
	return ret;
}

static void handle_transfer(struct libusb_transfer *transfer) {
	uint64_t time = OGGetAbsoluteTimeUS();

	SurviveUSBInterface *iface = transfer->user_data;
	if (iface->shutdown) {
		SurviveContext *ctx = iface->ctx;
		SV_VERBOSE(100, "Cleaning up transfer on %d %s", iface->which_interface_am_i, iface->hname);
		iface->ctx = 0;
		return;
	}

	SurviveContext *ctx = iface->ctx;

	if (transfer->status != LIBUSB_TRANSFER_COMPLETED) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Transfer problem %s %d with %s", libusb_error_name(transfer->status),
				 transfer->status, iface->hname);
		return;
	}

	iface->actual_len = transfer->actual_length;

	iface->cb(time, iface);
	iface->packet_count++;

	if (libusb_submit_transfer(transfer)) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT, "Error resubmitting transfer for %s", iface->hname);
	}
}

static inline void survive_close_usb_device(struct SurviveUSBInfo *usbInfo) {
	for (size_t j = 0; j < usbInfo->interface_cnt; j++) {
		usbInfo->interfaces[j].shutdown = 1;
	}
	SurviveContext *ctx = usbInfo->viveData->ctx;
	for (int j = 0; j < usbInfo->interface_cnt; j++) {
		SurviveUSBInterface *iface = &usbInfo->interfaces[j];
		SV_INFO("Cleaning up interface on %d %s", iface->which_interface_am_i, iface->hname);
		libusb_cancel_transfer(usbInfo->interfaces[j].transfer);
		while (usbInfo->interfaces[j].ctx) {
			survive_release_ctx_lock(ctx);
			libusb_handle_events(usbInfo->viveData->usbctx);
			survive_get_ctx_lock(ctx);
		}
		libusb_free_transfer(usbInfo->interfaces[j].transfer);

		libusb_release_interface(usbInfo->handle, j);
	}

	libusb_close(usbInfo->handle);
}

void survive_usb_close(SurviveViveData *sv) { libusb_exit(sv->usbctx); }
