#include "errno.h"
#include "os_generic.h"
#include "survive_config.h"
#include "survive_default_devices.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <survive.h>

#include "pcap/usb.h"
#include <pcap.h>

#include "driver_vive.h"

#include <libusb-1.0/libusb.h>
typedef struct vive_device_t {
	uint16_t vid, pid;
	const char *codename;
	const char *def_config;
} vive_device_t;

typedef struct vive_device_inst_t {
	const struct vive_device_t *device;
	int bus_id;
	int dev_id;
	SurviveObject *so;
} vive_device_inst_t;
#define VIVE_DEVICE_INST_MAX 32

struct vive_device_t devices[] = {{.vid = 0x28de, .pid = 0x2000, .codename = "HMD", .def_config = "HMD_config.json"},
								  {.vid = 0x28de, .pid = 0x2101, .codename = "WM", .def_config = "WM0_config.json"},
								  {.vid = 0x28de, .pid = 0x2022, .codename = "TR", .def_config = "TR0_config.json"},
								  {.vid = 0x28de, .pid = 0x2300, .codename = "T2", .def_config = "TR1_config.json"},
								  {.vid = 0x28de, .pid = 0x2012, .codename = "WW", .def_config = "WW0_config.json"},
								  {}};

static const int DEVICES_CNT = sizeof(devices) / sizeof(vive_device_t);

typedef struct SurviveDriverUSBMon {
	SurviveContext *ctx;
	pcap_t *pcap;

	char errbuf[PCAP_ERRBUF_SIZE];
	vive_device_inst_t usb_devices[VIVE_DEVICE_INST_MAX];
	size_t usb_devices_cnt;
} SurviveDriverUSBMon;

vive_device_inst_t *find_device_inst(SurviveDriverUSBMon *d, int bus_id, int dev_id) {
	for (size_t i = 0; i < d->usb_devices_cnt; i++) {
		if (d->usb_devices[i].bus_id == bus_id && d->usb_devices[i].dev_id == dev_id)
			return &d->usb_devices[i];
	}
	return 0;
}

static int interface_lookup(const vive_device_inst_t *dev, int endpoint) {
	int32_t id = dev->device->pid + (endpoint << 24);
	switch (id) {
	case 0x81002000:
		return USB_IF_HMD;
	case 0x81002101:
		return USB_IF_WATCHMAN1;
	case 0x81002022:
		return USB_IF_TRACKER0;
	case 0x81002300:
		return USB_IF_TRACKER1;
	case 0x81002012:
		return USB_IF_W_WATCHMAN1;

	case 0x82002000:
		return USB_IF_LIGHTCAP;
	case 0x82002101:
		return USB_IF_WATCHMAN1;
	case 0x82002022:
		return USB_IF_TRACKER0_LIGHTCAP;
	case 0x82002300:
		return USB_IF_TRACKER1_LIGHTCAP;
	case 0x82002012:
		return USB_IF_W_WATCHMAN1_LIGHTCAP;

	case 0x83002000:
		return USB_IF_HMD_BUTTONS;
	case 0x83002101:
		return USB_IF_W_WATCHMAN1_BUTTONS;
	case 0x83002022:
		return USB_IF_TRACKER0_BUTTONS;
	case 0x83002300:
		return USB_IF_TRACKER1_BUTTONS;
	case 0x83002012:
		return USB_IF_W_WATCHMAN1_BUTTONS;

	default:
		return 0;
	}
}

static int usbmon_poll(struct SurviveContext *ctx, void *_driver) {
	SurviveDriverUSBMon *driver = _driver;

	typedef pcap_usb_header_mmapped usb_header_t;

	struct pcap_pkthdr pkthdr = {};
	const usb_header_t *usbp = 0;

	while (usbp = (usb_header_t *)pcap_next(driver->pcap, &pkthdr)) {
		if (!(usbp->endpoint_number & 0x80))
			continue; // Only want incoming data
		if (usbp->status != 0)
			continue; // Only want responses
		if (usbp->data_flag)
			continue; // Only want data

		vive_device_inst_t *dev = find_device_inst(driver, usbp->bus_id, usbp->device_address);
		if (dev && dev->so) {
			/*SV_INFO("Packet number [%d], length of this packet is: %d %lx %x.%x %x %x %d %s", count++,
				pkthdr.len, usbp->id, usbp->bus_id, usbp->device_address,
				usbp->endpoint_number, usbp->event_type, usbp->status, dev->so->codename);*/
			int interface = interface_lookup(dev, usbp->endpoint_number);
			if (interface == 0) {
				SV_WARN("Don't understand %s endpoint 0x%x", dev->so->codename, usbp->endpoint_number);
			} else {
				SurviveUSBInterface si = {.ctx = ctx,
										  .actual_len = pkthdr.len,
										  .assoc_obj = dev->so,
										  .which_interface_am_i = interface,
										  .hname = dev->so->codename};

				// memcpy(si.buffer, (u_char*)&usbp[1], usbp->data);
				memcpy(si.buffer, (u_char *)&usbp[1], usbp->data_len);
				survive_data_cb(&si);
			}
		}
	}
	return 0;
}

static int usbmon_close(struct SurviveContext *ctx, void *_driver) {
	SurviveDriverUSBMon *driver = _driver;
	pcap_close(driver->pcap);
	free(driver);
	return 0;
}

static size_t fill_device_inst(vive_device_inst_t *insts) {
	libusb_context *context = NULL;
	libusb_device **list = NULL;
	int rc = 0;
	ssize_t count = 0;

	size_t rtn = 0;
	rc = libusb_init(&context);
	if (rc != 0)
		return 0;

	count = libusb_get_device_list(context, &list);

	for (size_t idx = 0; idx < count; ++idx) {
		libusb_device *device = list[idx];
		struct libusb_device_descriptor desc = {0};

		rc = libusb_get_device_descriptor(device, &desc);
		if (rc != 0)
			break;

		for (vive_device_t *dev = devices; dev->vid != 0; dev++) {
			if (desc.idVendor == dev->vid && desc.idProduct == dev->pid) {
				insts->device = dev;
				insts->bus_id = libusb_get_bus_number(device);
				insts->dev_id = libusb_get_device_address(device);
				insts++;
				rtn++;
			}
		}
	}

	libusb_free_device_list(list, count);
	libusb_exit(context);

	return rtn;
};

static char *read_file(const char *fn, size_t *size) {
	char *source = 0;
	FILE *fp = fopen(fn, "r");
	if (fp != NULL) {
		if (fseek(fp, 0L, SEEK_END) == 0) {

			long bufsize = ftell(fp);
			if (bufsize == -1) {
				fprintf(stderr, "ftell file '%s' failed with %d", fn, errno);
				return 0;
			}

			source = malloc(sizeof(char) * (bufsize + 1));

			if (fseek(fp, 0L, SEEK_SET) != 0) {
				fprintf(stderr, "fseek file '%s' failed with %d", fn, errno);
				free(source);
				return 0;
			}

			/* Read the entire file into memory. */
			size_t newLen = fread(source, sizeof(char), bufsize, fp);
			if (ferror(fp) != 0) {
				fputs("Error reading file", stderr);
			} else {
				source[newLen++] = '\0'; /* Just to be safe. */
			}

			if (size)
				*size = bufsize;
		} else {
			fprintf(stderr, "Read file '%s' failed with %d", fn, errno);
		}
	}
	if (fp)
		fclose(fp);

	return source;
}

static int setup_usb_devices(SurviveDriverUSBMon *sp) {
	SurviveContext *ctx = sp->ctx;
	int rtn = 0;

	int device_cnts[DEVICES_CNT];
	memset(device_cnts, 0, sizeof(int) * DEVICES_CNT);
	sp->usb_devices_cnt = fill_device_inst(sp->usb_devices);
	for (int i = 0; i < sp->usb_devices_cnt; i++) {
		int dev_idx = sp->usb_devices[i].device - devices;

		char buff[16] = "HMD";
		if (dev_idx != 0) {
			sprintf(buff, "%s%d", sp->usb_devices[i].device->codename, device_cnts[dev_idx]++);
		}

		SurviveObject *so = survive_create_device(ctx, "UMN", sp, buff, 0);

		size_t len = 0;
		const char *config_fn = sp->usb_devices[i].device->def_config;
		char *config_file = read_file(config_fn, &len);

		if (config_file && ctx->configfunction(so, config_file, len) == 0) {
			survive_add_object(ctx, so);
			sp->usb_devices[i].so = so;
			rtn++;
		} else {
			SV_WARN("Could not read %s for %s", config_fn, buff);
		}

		free(config_file);
	}

	return rtn;
}

int DriverRegUSBMon(SurviveContext *ctx) {
	int enable = survive_configi(ctx, "usbmon", SC_GET, 0);
	if (!enable)
		return 0;

	SurviveDriverUSBMon *sp = calloc(1, sizeof(SurviveDriverUSBMon));
	sp->ctx = ctx;

	sp->pcap = pcap_open_live("usbmon0", BUFSIZ, 0, -1, sp->errbuf);
	if (sp->pcap == NULL) {
		SV_WARN("pcap_open_live() failed due to [%s]", sp->errbuf);
		SV_INFO("You probably need to call 'sudo modprobe usbmob'");
		return -1;
	}

	pcap_setnonblock(sp->pcap, 1, sp->errbuf);

	int device_count = setup_usb_devices(sp);

	if (device_count) {
		survive_add_driver(ctx, sp, usbmon_poll, usbmon_close, 0);
	} else {
		usbmon_close(ctx, sp);
		return -1;
	}
	return 0;
}

REGISTER_LINKTIME(DriverRegUSBMon);
