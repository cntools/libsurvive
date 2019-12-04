#define _GNU_SOURCE

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
#include <pcap/usb.h>
#include <zlib.h>

STATIC_CONFIG_ITEM(USBMON_RECORD, "usbmon-record", 's', "File to save .pcap to.", 0);
STATIC_CONFIG_ITEM(USBMON_PLAYBACK, "usbmon-playback", 's', "File to replay .pcap from.", 0);
STATIC_CONFIG_ITEM(USBMON_RECORD_ALL, "usbmon-record-all", 'i', "Whether or not to record all usb traffic", 0);

typedef struct vive_device_t {
	uint16_t vid, pid;
	const char *codename;
	const char *def_config;
} vive_device_t;

typedef struct vive_device_inst_t {
	const struct vive_device_t *device;
	int bus_id;
	int dev_id;
	int devIdxForType;
	bool hasConfiged;
	SurviveObject *so;

	uint64_t last_config_id;
	uint8_t compressed_data[8192];
	uint16_t compressed_data_idx;
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
	bool record_all;

	char errbuf[PCAP_ERRBUF_SIZE];
	vive_device_inst_t usb_devices[VIVE_DEVICE_INST_MAX];
	size_t usb_devices_cnt;

	bool passiveMode;
	size_t packet_cnt;

	bool keepRunning;
	og_thread_t pcap_thread;
} SurviveDriverUSBMon;

vive_device_inst_t *find_device_inst(SurviveDriverUSBMon *d, int bus_id, int dev_id) {
	for (size_t i = 0; i < d->usb_devices_cnt; i++) {
		if (d->usb_devices[i].bus_id == bus_id && d->usb_devices[i].dev_id == dev_id)
			return &d->usb_devices[i];
	}
	return 0;
}

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

			source = SV_MALLOC(sizeof(char) * (bufsize + 1));

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

static int interface_lookup(const vive_device_inst_t *dev, int endpoint) {
	int32_t id = dev->device->pid + (endpoint << 16);
	switch (id) {
	case 0x812000:
		return USB_IF_HMD_IMU;
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

static bool is_config_request(const struct _usb_header_mmapped *usbp) {
	return usbp->event_type == 'S' && usbp->s.setup.bmRequestType == 0xa1 && usbp->s.setup.bRequest == 1 &&
		   usbp->s.setup.wValue == 0x311;
}

static bool is_config_start(const struct _usb_header_mmapped *usbp) {
	return usbp->event_type == 'S' && usbp->s.setup.bmRequestType == 0xa1 && usbp->s.setup.bRequest == 1 &&
		   usbp->s.setup.wValue == 0x310;
}

static void ingest_config_request(vive_device_inst_t *dev, const struct _usb_header_mmapped *usbp, uint8_t *pktData) {
	if (dev->so == 0) {
		return;
	}

	uint16_t cnt = pktData[1];

	if (cnt) {
		// Some (Tracker at least?) devices send a uint64_t before data; not sure what it means but skip it for now.
		if (dev->compressed_data_idx == 0 && cnt >= 2 && pktData[2] != 0x78) {

		} else {
			memcpy(&dev->compressed_data[dev->compressed_data_idx], pktData + 2, cnt);
			dev->compressed_data_idx += cnt;
		}
	} else {
		char *uncompressed_data = SV_MALLOC(65536);
		SurviveContext *ctx = dev->so->ctx;

		int len = survive_simple_inflate(dev->so->ctx, dev->compressed_data, dev->compressed_data_idx,
										 (uint8_t *)uncompressed_data, 65536 - 1);

		if (len <= 0) {
			SV_WARN("Error: data for config descriptor");
		} else {
			SV_INFO("usbmon loaded %d bytes of config data", len);
		}

		if (!dev->hasConfiged) {
			if (ctx->configproc(dev->so, uncompressed_data, len) == 0) {
				survive_add_object(ctx, dev->so);
				dev->hasConfiged = true;
				dev->last_config_id = 0;
			} else {
				SV_WARN("Could not load from config");
			}
		}
	}
}

static int usbmon_poll(struct SurviveContext *ctx, void *_driver) {
	SurviveDriverUSBMon *driver = _driver;
	if (driver->keepRunning == false)
		return -1;
	return 0;
}

static int usbmon_close(struct SurviveContext *ctx, void *_driver) {
	SurviveDriverUSBMon *driver = _driver;
	driver->keepRunning = false;
	pcap_breakloop(driver->pcap);
	SV_VERBOSE(100, "Waiting on pcap thread...");
	OGJoinThread(driver->pcap_thread);

	struct pcap_stat stats = {};
	pcap_stats(driver->pcap, &stats);

	SV_INFO("usbmon saw %u/%u packets, %u dropped, %u dropped in driver", (uint32_t)driver->packet_cnt, stats.ps_recv,
			stats.ps_drop, stats.ps_ifdrop);
	if (driver->pcapDumper) {
		pcap_dump_close(driver->pcapDumper);
	}
	pcap_close(driver->pcap);
	free(driver);
	return 0;
}
static usb_info_t *get_usb_info_from_file(const char *fname) {
	usb_info_t *rtn = SV_CALLOC(MAX_USB_DEVS, sizeof(usb_info_t));
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
	rtn = (usb_info_t *)SV_CALLOC(count + 1, sizeof(usb_info_t));
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

static size_t fill_device_inst(SurviveContext *ctx, vive_device_inst_t *insts, const usb_info_t *usb_dev,
							   FILE *save_file) {
	size_t rtn = 0;
	while (usb_dev->vid != 0 && usb_dev->pid != 0) {
		bool foundDevice = false;
		for (vive_device_t *dev = devices; dev->vid != 0; dev++) {
			if (usb_dev->vid == dev->vid && usb_dev->pid == dev->pid) {
				foundDevice = true;
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

		if (!foundDevice && usb_dev->vid == 0x28de) {
			SV_WARN("Didn't find device instance for %04x:%04x", usb_dev->vid, usb_dev->pid);
		}

		usb_dev++;
	}

	return rtn;
};

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

	sp->usb_devices_cnt = fill_device_inst(ctx, sp->usb_devices, usbInfo, listing_file);
	if (listing_file) {
		fclose(listing_file);
	}
	free(usbInfo);

	for (int i = 0; i < sp->usb_devices_cnt; i++) {
		int dev_idx = sp->usb_devices[i].device - devices;
		if (sp->passiveMode == false) {
			char buff[16] = "HMD";
			if (dev_idx != 0) {
				sprintf(buff, "%s%d", sp->usb_devices[i].device->codename, device_cnts[dev_idx]++);
			}

			SurviveObject *so = survive_create_device(ctx, "UMN", sp, buff, 0);
			sp->usb_devices[i].so = so;
		}
		char filter[256] = {};
		sprintf(filter, "(usb.bus_id = %d and usb.device_address = %d)", sp->usb_devices[i].bus_id,
				sp->usb_devices[i].dev_id);

		sp->usb_devices[i].devIdxForType = device_cnts[dev_idx];
		rtn++;
	}

	return rtn;
}

void *pcap_thread_fn(void *_driver) {
	SurviveDriverUSBMon *driver = _driver;
	struct SurviveContext *ctx = driver->ctx;
	typedef pcap_usb_header_mmapped usb_header_t;

	struct pcap_pkthdr *pkthdr = 0;
	const usb_header_t *usbp = 0;

	while (driver->keepRunning) {
		int result = pcap_next_ex(driver->pcap, &pkthdr, (const uint8_t **)&usbp);
		switch (result) {
		case 0:
			goto continue_loop;
		case 1: {
			// if (usbp = (usb_header_t *)pcap_next(driver->pcap, &pkthdr)) {
			vive_device_inst_t *dev = find_device_inst(driver, usbp->bus_id, usbp->device_address);

			// Packet data is directly after the packet header
			uint8_t *pktData = (uint8_t *)&usbp[1];
			if (driver->pcapDumper && (dev || driver->record_all)) {
				pcap_dump((uint8_t *)driver->pcapDumper, pkthdr, (uint8_t *)usbp);
			}

			if (dev) {
				driver->packet_cnt++;

				// Print setup flags, then just bail
				if (!usbp->setup_flag) {
					if (is_config_start(usbp)) {
						dev->last_config_id = 0;
						dev->compressed_data_idx = 0;
					} else if (is_config_request(usbp)) {
						dev->last_config_id = usbp->id;
					} else {
						fprintf(stderr,
								"S: 0x%016lx event_type: %c transfer_type: %d bmRequestType: 0x%02x bRequest: 0x%02x "
								"wValue: 0x%04x wIndex: 0x%04x wLength: %4d\n",
								usbp->id, usbp->event_type, usbp->transfer_type, usbp->s.setup.bmRequestType,
								usbp->s.setup.bRequest, usbp->s.setup.wValue, usbp->s.setup.wIndex,
								usbp->s.setup.wLength);
					}
					goto continue_loop;
				}

				if (!(usbp->endpoint_number & 0x80u)) {
					fprintf(stderr, "W: 0x%016lx event_type: %c transfer_type: %d 0x%02x (0x%02x): ", usbp->id,
							usbp->event_type, usbp->transfer_type, usbp->endpoint_number, usbp->data_len);
					for (int i = 0; i < usbp->data_len; i++) {
						if ((i + 2) % 4 == 0)
							fprintf(stderr, "  ");
						fprintf(stderr, "%02x ", pktData[i]);
					}

					fprintf(stderr, "\n");

					goto continue_loop; // Only want incoming data
				}

				if (usbp->status != 0) {
					// EINPROGRESS is normal
					if (usbp->status != -115)
						fprintf(stderr, "E: 0x%016lx event_type: %c transfer_type: %d status: %d\n", usbp->id,
								usbp->event_type, usbp->transfer_type, usbp->status);
					goto continue_loop; // Only want responses
				}

				if (usbp->id == dev->last_config_id && usbp->event_type == 'C' && dev->hasConfiged == false) {
					ingest_config_request(dev, usbp, pktData);
					goto continue_loop;
				}

				/*
			if (usbp->data_flag)
				continue; // Only want data
				*/
				/*SV_INFO("Packet number [%d], length of this packet is: %d %lx %x.%x %x %x %d %s", count++,
					pkthdr.len, usbp->id, usbp->bus_id, usbp->device_address,
					usbp->endpoint_number, usbp->event_type, usbp->status, dev->so->codename);*/
				int interface = interface_lookup(dev, usbp->endpoint_number);
				if (interface == 0) {
					fprintf(stderr, "R: 0x%016lx event_type: %c transfer_type: %d 0x%02x (0x%02x): ", usbp->id,
							usbp->event_type, usbp->transfer_type, usbp->endpoint_number, usbp->data_len);
					for (int i = 0; i < usbp->data_len; i++) {
						if ((i + 2) % 4 == 0)
							fprintf(stderr, "  ");
						fprintf(stderr, "%02x ", pktData[i]);
					}

					fprintf(stderr, "\n");

				} else if (dev->hasConfiged) {

					if (dev->so) {
						SurviveUSBInterface si = {.ctx = ctx,
												  .actual_len = pkthdr->len,
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

			break;
		}
		case PCAP_ERROR:
			SV_WARN("Pcap error %s", pcap_geterr(driver->pcap));
		case PCAP_ERROR_BREAK:
			driver->keepRunning = false;
			goto exit_loop;
		default:
			SV_WARN("Pcap next got %d", result);
		}

	continue_loop:
		continue;
	}

exit_loop:

	SV_VERBOSE(100, "Exiting usbmon thread");
	return 0;
}

#if defined(HAVE_FOPENCOOKIE)
static ssize_t gzip_cookie_write(void *cookie, const char *buf, size_t size) {
	return gzwrite((gzFile)cookie, (voidpc)buf, size);
}

static int gzip_cookie_close(void *cookie) { return gzclose((gzFile)cookie); }

static ssize_t gzip_cookie_read(void *cookie, char *buf, size_t nbytes) { return gzread((gzFile)cookie, buf, nbytes); }

int gzip_cookie_seek(void *cookie, off64_t *pos, int __w) { return gzseek((gzFile)cookie, *pos, __w); }

cookie_io_functions_t gzip_cookie = {
	.close = gzip_cookie_close, .write = gzip_cookie_write, .read = gzip_cookie_read, .seek = gzip_cookie_seek};
#endif

static FILE *open_playback(const char *fn, const char *mode) {
#if defined(HAVE_FOPENCOOKIE)
	gzFile z = gzopen(fn, mode);
	return fopencookie(z, mode, gzip_cookie);
#else
	return fopen(fn, mode);
#endif
}

int DriverRegUSBMon(SurviveContext *ctx) {
	int enable = survive_configi(ctx, "usbmon", SC_GET, 0);
	const char *usbmon_record = survive_configs(ctx, "usbmon-record", SC_GET, 0);
	const char *usbmon_playback = survive_configs(ctx, "usbmon-playback", SC_GET, 0);

	SurviveDriverUSBMon *sp = SV_CALLOC(1, sizeof(SurviveDriverUSBMon));
	sp->ctx = ctx;
	sp->passiveMode = !enable && usbmon_record;
	if (sp->passiveMode) {
		SV_INFO("Starting usbmon in passive (record) mode.");
	}
	if (usbmon_playback && *usbmon_playback) {
		SV_INFO("Opening '%s' for usb playback", usbmon_playback);
		FILE *pF = open_playback(usbmon_playback, "r");
		sp->pcap = pcap_fopen_offline(pF, sp->errbuf);
	} else {
		sp->pcap = pcap_open_live("usbmon0", PCAP_ERRBUF_SIZE, 0, -1, sp->errbuf);
	}

	if (sp->pcap == NULL) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT,
				 "pcap_open_live() failed due to [%s] - You probably need to call 'sudo modprobe usbmon'", sp->errbuf);
		return SURVIVE_DRIVER_ERROR;
	}

	if (usbmon_record && *usbmon_record) {
		FILE *fd = open_playback(usbmon_record, "w");
		SV_INFO("Opening %s for usb recording (%p)", usbmon_record, fd);
		sp->pcapDumper = pcap_dump_fopen(sp->pcap, fd);
		sp->record_all = survive_configi(ctx, "usbmon-record-all", SC_GET, 0);
		if (sp->record_all) {
			SV_WARN("All USB traffic is being captured. Don't use 'usbmon-record-all' if you don't want to expose "
					"things like input from keyboards.");
		}
	}

	int device_count = setup_usb_devices(sp);
	if (device_count) {
		sp->keepRunning = true;
		sp->pcap_thread = OGCreateThread(pcap_thread_fn, sp);

		survive_add_driver(ctx, sp, usbmon_poll, usbmon_close, 0);
	} else {
		usbmon_close(ctx, sp);
		SV_ERROR(SURVIVE_ERROR_NO_TRACKABLE_OBJECTS, "USBMon found no devices");
		return SURVIVE_DRIVER_ERROR;
	}
	return sp->passiveMode ? SURVIVE_DRIVER_PASSIVE : SURVIVE_DRIVER_NORMAL;
}

REGISTER_LINKTIME(DriverRegUSBMon);

int DriverRegUSBMon_Record(SurviveContext *ctx) { return DriverRegUSBMon(ctx); }
REGISTER_LINKTIME(DriverRegUSBMon_Record);

int DriverRegUSBMon_Playback(SurviveContext *ctx) { return DriverRegUSBMon(ctx); }
REGISTER_LINKTIME(DriverRegUSBMon_Playback);
