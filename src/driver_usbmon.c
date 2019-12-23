#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

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
STATIC_CONFIG_ITEM(USBMON_OUTPUT_EVERYTHING, "usbmon-output-all", 'i', "Whether or not to log all usb traffic", 0);
STATIC_CONFIG_ITEM(USBMON_OUTPUT, "usbmon-output", 'i', "Whether or not to log any generic usb traffic", 0);
STATIC_CONFIG_ITEM(USBMON_ONLY_RECORD, "usbmon-only-record", 'i', "Record only; don't forward to libsurvive", 0);

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
	struct SurviveUSBInfo *usbInfo;

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
								  {.vid = 0x28de, .pid = 0x2102, .codename = "KN", .def_config = "KN%d_config.json"},
								  {}};

static const int DEVICES_CNT = sizeof(devices) / sizeof(vive_device_t);

typedef struct SurviveDriverUSBMon {
	SurviveContext *ctx;
	pcap_t *pcap;
	double playback_factor;
	double time_now;

	pcap_dumper_t *pcapDumper;
	bool record_all;
	bool record_only;

	bool output_usb_stream;
	bool output_everything;

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
	case 0x802300:
	case 0x802101:
		return USB_IF_TRACKER_INFO;
	case 0x822000:
		return USB_IF_HMD_LIGHTCAP;
	case 0x822101:
	case 0x812102:
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

static bool is_command_setup(const struct _usb_header_mmapped *usbp) {
	return usbp->event_type == 'S' && usbp->s.setup.bmRequestType == 0x21 && usbp->s.setup.bRequest == 0x09 &&
		   usbp->s.setup.wValue == 0x3ff;
}

static void ingest_config_request(vive_device_inst_t *dev, const struct _usb_header_mmapped *usbp, uint8_t *pktData) {
	if (dev->so == 0) {
		return;
	}

	uint16_t cnt = pktData[1];
	SurviveContext *ctx = dev->so->ctx;

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
			SV_INFO("usbmon loaded %d total bytes of config data", len);
		}

		if (!dev->hasConfiged) {
			if (ctx->configproc(dev->so, uncompressed_data, len) == 0) {
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

	for (int i = 0; i < driver->usb_devices_cnt; i++) {
		vive_device_inst_t *dev = &driver->usb_devices[i];
		free(dev->usbInfo);
	}
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

			SurviveObject *so = survive_create_device(ctx, "UMN", 0, buff, 0);
			sp->usb_devices[i].so = so;
			sp->usb_devices[i].usbInfo =
				survive_vive_register_driver(so, sp->usb_devices[i].device->vid, sp->usb_devices[i].device->pid);
			survive_add_object(ctx, so);
		}
		char filter[256] = {};
		sprintf(filter, "(usb.bus_id = %d and usb.device_address = %d)", sp->usb_devices[i].bus_id,
				sp->usb_devices[i].dev_id);

		sp->usb_devices[i].devIdxForType = device_cnts[dev_idx];
		rtn++;
	}

	return rtn;
}

double make_time(double start, const struct _usb_header_mmapped *pMmapped) {
	return pMmapped->ts_sec + pMmapped->ts_usec * 1.e-6 - start;
}

const char *requestTypeToStr(uint8_t requestType) {
	switch (requestType) {
	case 0:
		return "GET_STATUS";
	case 1:
		return "CLEAR_FEATURE";
	case 3:
		return "SET_FEATURE";
	case 5:
		return "SET_ADDRESS";
	case 6:
		return "GET_DESCRIPTOR";
	case 7:
		return "SET_DESCRIPTOR";
	case 8:
		return "GET_CONFIGURATION";
	case 9:
		return "SET_CONFIGURATION";
	case 10:
		return "GET_INTERFACE";
	case 11:
		return "SET_INTERFACE";
	case 12:
		return "SYNC_FRAME";
	}
	return "<unknown>";
}

static double timestamp_in_s() {
	static double start_time_s = 0;
	if (start_time_s == 0.)
		start_time_s = OGGetAbsoluteTime();
	return OGGetAbsoluteTime() - start_time_s;
}

static double survive_usbmon_playback_run_time(const SurviveContext *ctx, void *_driver) {
	SurviveDriverUSBMon *driver = _driver;
	return driver->time_now;
}

void *pcap_thread_fn(void *_driver) {
	SurviveDriverUSBMon *driver = _driver;
	struct SurviveContext *ctx = driver->ctx;
	typedef pcap_usb_header_mmapped usb_header_t;

	struct pcap_pkthdr *pkthdr = 0;
	const usb_header_t *usbp = 0;

	SV_INFO("Pcap thread started");
	double start_time = 0;
	double real_time_start = timestamp_in_s();
	while (driver->keepRunning && ctx->currentError == SURVIVE_OK) {
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
				const char *dev_name = dev->device->codename;
				if (dev->so)
					dev_name = dev->so->codename;

				if (start_time == 0) {
					start_time = make_time(0, usbp);
				}
				double this_real_time = timestamp_in_s();
				double this_time = make_time(start_time, usbp);
				if (driver->playback_factor > 0.) {
					double next_time_s_scaled = this_time * driver->playback_factor;
					while (this_real_time < next_time_s_scaled) {
						int sleep_time_ms = 1 + (next_time_s_scaled - this_real_time) * 1000.;
						OGUSleep(sleep_time_ms * 1000);
						this_real_time = timestamp_in_s();
					}
				}
				driver->time_now = this_time;

				// Print setup flags, then just bail
				if (!usbp->setup_flag) {
					if (is_config_start(usbp)) {
						dev->last_config_id = 0;
						dev->compressed_data_idx = 0;
						SV_VERBOSE(200, "%s start of config", dev_name);
					} else if (is_config_request(usbp)) {
						dev->last_config_id = usbp->id;
					} else if (is_command_setup(usbp)) {
						SV_INFO("%s sent command 0x%02x with %u bytes:", dev_name, pktData[1], pktData[2]);
						survive_dump_buffer(ctx, pktData + 3, pktData[2]);
					}
					if (driver->output_usb_stream) {
						ctx->printfproc(
							ctx,
							"--> %10.6f S: %s 0x%016lx event_type: %c transfer_type: %d bmRequestType: 0x%02x "
							"bRequest: 0x%02x (%s) "
							"wValue: 0x%04x wIndex: 0x%04x wLength: %4d\n",
							this_time, dev_name, usbp->id, usbp->event_type, usbp->transfer_type,
							usbp->s.setup.bmRequestType, usbp->s.setup.bRequest,
							requestTypeToStr(usbp->s.setup.bRequest), usbp->s.setup.wValue, usbp->s.setup.wIndex,
							usbp->s.setup.wLength);

						survive_dump_buffer(ctx, pktData, usbp->data_len);
					}

					if (dev->so) {
						survive_data_on_setup_write(dev->so, usbp->s.setup.bmRequestType, usbp->s.setup.bRequest,
													usbp->s.setup.wValue, usbp->s.setup.wIndex, pktData,
													usbp->data_len);
					}
					goto continue_loop;
				}

				if (!(usbp->endpoint_number & 0x80u)) {

					if (driver->output_usb_stream) {
						if (usbp->event_type == 'C') {
							ctx->printfproc(
								ctx, "<-- %10.6f C: %s 0x%016lx event_type: %c transfer_type: %d 0x%02x (0x%02x):\n",
								this_time, dev_name, usbp->id, usbp->event_type, usbp->transfer_type,
								usbp->endpoint_number, usbp->data_len);
						} else {
							ctx->printfproc(
								ctx, "--> %10.6f W: %s 0x%016lx event_type: %c transfer_type: %d 0x%02x (0x%02x):\n",
								this_time, dev_name, usbp->id, usbp->event_type, usbp->transfer_type,
								usbp->endpoint_number, usbp->data_len);
						}
						survive_dump_buffer(ctx, pktData, usbp->data_len);
					}
					goto continue_loop; // Only want incoming data
				}

				if (usbp->status != 0) {
					// EINPROGRESS is normal, EPIPE means stalled
					if (driver->output_usb_stream) {
						if ((usbp->status != -115 && usbp->status != -32) || driver->output_everything)
							ctx->printfproc(
								ctx, "<-- %10.6f E: %s 0x%016lx event_type: %c transfer_type: %d status: %d\n",
								this_time, dev_name, usbp->id, usbp->event_type, usbp->transfer_type, usbp->status);
					}
					goto continue_loop; // Only want responses
				}

				if (usbp->id == dev->last_config_id && usbp->event_type == 'C' && dev->hasConfiged == false) {
					ingest_config_request(dev, usbp, pktData);
					goto continue_loop;
				}

				int interface = interface_lookup(dev, usbp->endpoint_number);

				bool output_read = driver->output_usb_stream &&
								   (interface == 0 || driver->output_everything || interface == USB_IF_TRACKER_INFO) &&
								   interface != USB_IF_W_WATCHMAN1_IMU && interface != USB_IF_TRACKER1_IMU &&
								   interface != USB_IF_TRACKER0_IMU;

				if (output_read) {
					ctx->printfproc(
						ctx,
						"<-- %10.6f R: %s 0x%016lx event_type: %c transfer_type: %d endpoint: 0x%02x (%s) (0x%02x): \n",
						this_time, dev_name, usbp->id, usbp->event_type, usbp->transfer_type, usbp->endpoint_number,
						survive_usb_interface_str(interface), usbp->data_len);

					survive_dump_buffer(ctx, pktData, usbp->data_len);
				}

				bool forward_to_data_cb = driver->record_only == false &&
										  (interface != 0 && (dev->hasConfiged || interface == USB_IF_TRACKER_INFO)) &&
										  dev->so != 0;

				if (forward_to_data_cb) {
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

static int DriverRegUSBMon_(SurviveContext *ctx, int driver_id) {
	int enable = survive_configi(ctx, "usbmon", SC_GET, 0);
	const char *usbmon_record = survive_configs(ctx, "usbmon-record", SC_GET, 0);
	const char *usbmon_playback = survive_configs(ctx, "usbmon-playback", SC_GET, 0);

	if (enable && driver_id != 0)
		return -1;

	SurviveDriverUSBMon *sp = SV_CALLOC(1, sizeof(SurviveDriverUSBMon));
	sp->playback_factor = -1;
	sp->ctx = ctx;
	sp->passiveMode = !enable && driver_id == 1;
	if (sp->passiveMode) {
		SV_INFO("Starting usbmon in passive (record) mode.");
	} else {
		SV_INFO("Starting usbmon");
	}
	if (usbmon_playback && *usbmon_playback) {
		SV_INFO("Opening '%s' for usb playback", usbmon_playback);
		FILE *pF = open_playback(usbmon_playback, "r");
		sp->pcap = pcap_fopen_offline(pF, sp->errbuf);
		sp->playback_factor = survive_configf(ctx, "playback-factor", SC_GET, 1.0);
		survive_install_run_time_fn(ctx, survive_usbmon_playback_run_time, sp);
	} else {
		sp->pcap = pcap_open_live("usbmon0", PCAP_ERRBUF_SIZE, 0, -1, sp->errbuf);
	}

	if (sp->pcap == NULL) {
		SV_ERROR(SURVIVE_ERROR_HARWARE_FAULT,
				 "pcap_open_live() failed due to [%s] - You probably need to call 'sudo modprobe usbmon'. If you want "
				 "to capture as a normal user; try 'sudo setfacl -m u:$USER:r /dev/usbmon*'",
				 sp->errbuf);
		return SURVIVE_DRIVER_ERROR;
	}

	sp->output_everything = survive_configi(ctx, "usbmon-output-all", SC_GET, 0);
	sp->output_usb_stream = sp->output_everything || survive_configi(ctx, "usbmon-output", SC_GET, 0);
	sp->record_only = survive_configi(ctx, "usbmon-only-record", SC_GET, 0);

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
	return SURVIVE_DRIVER_NORMAL;
}

int DriverRegUSBMon(SurviveContext *ctx) { return DriverRegUSBMon_(ctx, 0); }
REGISTER_LINKTIME(DriverRegUSBMon);

int DriverRegUSBMon_Record(SurviveContext *ctx) { return DriverRegUSBMon_(ctx, 1); }
REGISTER_LINKTIME(DriverRegUSBMon_Record);

int DriverRegUSBMon_Playback(SurviveContext *ctx) { return DriverRegUSBMon_(ctx, 2); }
REGISTER_LINKTIME(DriverRegUSBMon_Playback);
