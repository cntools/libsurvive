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

STATIC_CONFIG_ITEM(USBMON_RECORD, "usbmon-record", 's', "File to save .pcap to.", "");
STATIC_CONFIG_ITEM(USBMON_PLAYBACK, "usbmon-playback", 's', "File to replay .pcap from.", "");

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

typedef struct usb_info_t {
	uint16_t vid, pid;
	int bus_id, dev_id;
} usb_info_t;

#define VIVE_DEVICE_INST_MAX 32

struct vive_device_t devices[] = {{.vid = 0x28de, .pid = 0x2000, .codename = "HMD", .def_config = "HMD_config.json"},
								  {.vid = 0x28de, .pid = 0x2101, .codename = "WM", .def_config = "WM%d_config.json"},
								  {.vid = 0x28de, .pid = 0x2022, .codename = "TR", .def_config = "TR%d_config.json"},
								  {.vid = 0x28de, .pid = 0x2300, .codename = "T2", .def_config = "T2%d_config.json"},
								  {.vid = 0x28de, .pid = 0x2012, .codename = "WW", .def_config = "WW%d_config.json"},
								  {}};

static const int DEVICES_CNT = sizeof(devices) / sizeof(vive_device_t);

typedef struct SurviveDriverUSBMon {
	SurviveContext *ctx;
	pcap_t *pcap;
	pcap_dumper_t *pcapDumper;

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
	int32_t id = dev->device->pid + (endpoint << 16);
	switch (id) {
	case 0x812000:
		return USB_IF_HMD_HEADSET_INFO;
	case 0x812101:
		return USB_IF_WATCHMAN1;
	case 0x812022:
		return USB_IF_TRACKER0_IMU;
	case 0x812300:
		return USB_IF_TRACKER1_IMU;
	case 0x812012:
		return USB_IF_W_WATCHMAN1_IMU;

	case 0x822000:
		return USB_IF_HMD_LIGHTCAP;
	case 0x822101:
		return USB_IF_WATCHMAN1;
	case 0x822022:
		return USB_IF_TRACKER0_LIGHTCAP;
	case 0x822300:
		return USB_IF_TRACKER1_LIGHTCAP;
	case 0x822012:
		return USB_IF_W_WATCHMAN1_LIGHTCAP;

	case 0x832000:
		return USB_IF_HMD_BUTTONS;
	case 0x832101:
		return USB_IF_W_WATCHMAN1_BUTTONS;
	case 0x832022:
		return USB_IF_TRACKER0_BUTTONS;
	case 0x832300:
		return USB_IF_TRACKER1_LIGHTCAP;
	case 0x832012:
		return USB_IF_W_WATCHMAN1_BUTTONS;
	case 0x842300:
		return USB_IF_TRACKER1_BUTTONS;
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
		vive_device_inst_t *dev = find_device_inst(driver, usbp->bus_id, usbp->device_address);
		if (dev && dev->so) {
			// Packet data is directly after the packet header
			uint8_t *pktData = (uint8_t *)&usbp[1];

			if (driver->pcapDumper) {
				pcap_dump((uint8_t *)driver->pcapDumper, &pkthdr, (uint8_t *)usbp);
			}

			if (!(usbp->endpoint_number & 0x80u)) {

				fprintf(stderr, "W: %d (0x%02x): ", usbp->endpoint_number, usbp->data_len);
				for (int i = 0; i < usbp->data_len; i++) {
					if ((i + 2) % 4 == 0)
						fprintf(stderr, "  ");
					fprintf(stderr, "%02x ", pktData[i]);
				}

				fprintf(stderr, "\n");

				continue; // Only want incoming data
			}
			if (usbp->status != 0)
				continue; // Only want responses
			if (usbp->data_flag)
				continue; // Only want data

			/*SV_INFO("Packet number [%d], length of this packet is: %d %lx %x.%x %x %x %d %s", count++,
				pkthdr.len, usbp->id, usbp->bus_id, usbp->device_address,
				usbp->endpoint_number, usbp->event_type, usbp->status, dev->so->codename);*/
			int interface = interface_lookup(dev, usbp->endpoint_number);
			if (interface == 0) {
				SV_WARN("Don't understand %s(%04x) endpoint 0x%x", dev->so->codename, dev->device->pid,
						usbp->endpoint_number);
			} else {
				SurviveUSBInterface si = {.ctx = ctx,
										  .actual_len = pkthdr.len,
										  .assoc_obj = dev->so,
										  .which_interface_am_i = interface,
										  .hname = dev->so->codename};

				// memcpy(si.buffer, (u_char*)&usbp[1], usbp->data);
				si.actual_len = usbp->data_len;
				memset(si.buffer, 0xCA, sizeof(si.buffer));
				memcpy(si.buffer, pktData, usbp->data_len);
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
static usb_info_t *get_usb_info_from_file(const char *fname) {
	usb_info_t *rtn = calloc(MAX_USB_DEVS, sizeof(usb_info_t));
	size_t count = 0;
	FILE *f = fopen(fname, "r");
	while (!feof(f)) {
		char name[128];
		if (fscanf(f, "%hd %hd %d %d %s ", &rtn[count].vid, &rtn[count].pid, &rtn[count].bus_id, &rtn[count].dev_id,
				   name) == 5) {
			count++;
		}
	}
	return rtn;
}

static usb_info_t *get_usb_info_from_libusb() {
	usb_info_t *rtn = 0;

	libusb_context *context = NULL;
	libusb_device **list = NULL;
	int rc = 0;
	ssize_t count = 0;

	rc = libusb_init(&context);
	if (rc != 0)
		return 0;

	count = libusb_get_device_list(context, &list);
	rtn = (usb_info_t *)calloc(count + 1, sizeof(usb_info_t));
	size_t fill_cnt = 0;
	for (size_t idx = 0; idx < count; ++idx) {
		libusb_device *device = list[idx];
		struct libusb_device_descriptor desc = {0};

		rc = libusb_get_device_descriptor(device, &desc);
		if (rc != 0)
			break;

		rtn[fill_cnt] = (usb_info_t){.vid = desc.idVendor,
									 .pid = desc.idProduct,
									 .bus_id = libusb_get_bus_number(device),
									 .dev_id = libusb_get_device_address(device)};

		fill_cnt++;
	}

	libusb_free_device_list(list, count);
	libusb_exit(context);

	return rtn;
}

static size_t fill_device_inst(vive_device_inst_t *insts, const usb_info_t *usb_dev, FILE *save_file) {
	size_t rtn = 0;
	while (usb_dev->vid != 0 && usb_dev->pid != 0) {
		for (vive_device_t *dev = devices; dev->vid != 0; dev++) {
			if (usb_dev->vid == dev->vid && usb_dev->pid == dev->pid) {
				insts->device = dev;
				insts->bus_id = usb_dev->bus_id;
				insts->dev_id = usb_dev->dev_id;

				if (save_file) {
					fprintf(save_file, "%d %d %d %d %s\n", usb_dev->vid, usb_dev->pid, insts->bus_id, insts->dev_id,
							dev->codename);
				}

				insts++;
				rtn++;
			}
		}

		usb_dev++;
	}

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

	const char *usbmon_record = survive_configs(ctx, "usbmon-record", SC_GET, 0);
	const char *usbmon_playback = survive_configs(ctx, "usbmon-playback", SC_GET, 0);

	FILE *listing_file = 0;

	if (usbmon_record && *usbmon_record) {
		char fname[256] = {0};
		sprintf(fname, "%s.usbdevs", usbmon_record);
		listing_file = fopen(fname, "w");
	}

	usb_info_t *usbInfo = 0;
	if (usbmon_playback && *usbmon_playback) {
		char fname[256] = {0};
		sprintf(fname, "%s.usbdevs", usbmon_playback);
		usbInfo = get_usb_info_from_file(fname);
	} else {
		usbInfo = get_usb_info_from_libusb();
	}

	sp->usb_devices_cnt = fill_device_inst(sp->usb_devices, usbInfo, listing_file);
	if (listing_file) {
		fclose(listing_file);
	}
	free(usbInfo);

	for (int i = 0; i < sp->usb_devices_cnt; i++) {
		int dev_idx = sp->usb_devices[i].device - devices;

		char buff[16] = "HMD";
		if (dev_idx != 0) {
			sprintf(buff, "%s%d", sp->usb_devices[i].device->codename, device_cnts[dev_idx]++);
		}

		SurviveObject *so = survive_create_device(ctx, "UMN", sp, buff, 0);

		size_t len = 0;
		const char *config_fn_template = sp->usb_devices[i].device->def_config;
		char config_fn[1024] = {};
		sprintf(config_fn, config_fn_template, device_cnts[dev_idx] - 1);

		char *config_file = read_file(config_fn, &len);

		if (config_file && ctx->configproc(so, config_file, len) == 0) {
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
	const char *usbmon_record = survive_configs(ctx, "usbmon-record", SC_GET, 0);
	const char *usbmon_playback = survive_configs(ctx, "usbmon-playback", SC_GET, 0);

	if (!enable && !usbmon_record && !usbmon_playback)
		return 0;

	SurviveDriverUSBMon *sp = calloc(1, sizeof(SurviveDriverUSBMon));
	sp->ctx = ctx;

	if (usbmon_playback && *usbmon_playback) {
		sp->pcap = pcap_open_offline(usbmon_playback, sp->errbuf);
	} else {
		sp->pcap = pcap_open_live("usbmon0", PCAP_ERRBUF_SIZE, 0, -1, sp->errbuf);
	}
	if (sp->pcap == NULL) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT,
				 "pcap_open_live() failed due to [%s] - You probably need to call 'sudo modprobe usbmon'", sp->errbuf);
		return -1;
	}

	if (usbmon_record && *usbmon_record) {
		sp->pcapDumper = pcap_dump_open(sp->pcap, usbmon_record);
	}

	pcap_setnonblock(sp->pcap, 1, sp->errbuf);

	int device_count = setup_usb_devices(sp);

	if (device_count) {
		survive_add_driver(ctx, sp, usbmon_poll, usbmon_close, 0);
	} else {
		usbmon_close(ctx, sp);
		SV_ERROR(SURVIVE_ERROR_NO_TRACKABLE_OBJECTS, "USBMon found no devices");
		return -1;
	}
	return 0;
}

REGISTER_LINKTIME(DriverRegUSBMon);
