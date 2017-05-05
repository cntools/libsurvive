//Unofficial driver for the official Valve/HTC Vive hardware.
//
//Based off of https://github.com/collabora/OSVR-Vive-Libre
// Originally Copyright 2016 Philipp Zabel
// Originally Copyright 2016 Lubosz Sarnecki <lubosz.sarnecki@collabora.co.uk>
// Originally Copyright (C) 2013 Fredrik Hultin
// Originally Copyright (C) 2013 Jakob Bornecrantz
//
//But, re-written as best as I can to get it put under an open souce license instead of a forced-source license.
//If there are portions of the code too similar to the original, I would like to know  so they can be re-written.
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#include <survive.h>
#include <jsmn.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>
#include <os_generic.h>
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h> // for alloca
#endif

#ifdef HIDAPI
#if defined(WINDOWS) || defined(WIN32) || defined (_WIN32)
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

struct SurviveViveData;

const short vidpids[] = {
	0x0bb4, 0x2c87, 0, //Valve HMD Button and face proximity sensor
	0x28de, 0x2000, 0, //Valve HMD IMU & Lighthouse Sensors
	0x28de, 0x2101, 0, //Valve Watchman
	0x28de, 0x2101, 1, //Valve Watchman
	0x28de, 0x2022, 0, //HTC Tracker
	0x28de, 0x2012, 0, //Valve Watchman, USB connected
#ifdef HIDAPI
	0x28de, 0x2000, 1, //Valve HMD lighthouse(B) (only used on HIDAPI, for lightcap)
	0x28de, 0x2022, 1, //HTC Tracker (only used on HIDAPI, for lightcap)
	0x28de, 0x2012, 1, //Valve Watchman, USB connected (only used on HIDAPI, for lightcap)
#endif
}; //length MAX_USB_INTERFACES*2

const char * devnames[] = {
	"HMD",
	"HMD IMU & LH",
	"Watchman 1",
	"Watchman 2",
	"Tracker 0",
	"Wired Watchman 1",
#ifdef HIDAPI
	"HMD Lightcap",
	"Tracker 0 Lightcap",
	"Wired Watchman 1 Lightcap",
#endif
}; //length MAX_USB_INTERFACES


#define USB_DEV_HMD			0
#define USB_DEV_HMD_IMU_LH	1
#define USB_DEV_WATCHMAN1	2
#define USB_DEV_WATCHMAN2	3
#define USB_DEV_TRACKER0	4
#define USB_DEV_W_WATCHMAN1	5 // Wired Watchman attached via USB

#ifdef HIDAPI
#define USB_DEV_HMD_IMU_LHB 6
#define USB_DEV_TRACKER0_LIGHTCAP 7
#define USB_DEV_W_WATCHMAN1_LIGHTCAP 8
#define MAX_USB_DEVS		9
#else
#define MAX_USB_DEVS		6
#endif

#define USB_IF_HMD			0
#define USB_IF_HMD_IMU_LH 	1
#define USB_IF_WATCHMAN1	2
#define USB_IF_WATCHMAN2	3
#define USB_IF_TRACKER0		4
#define USB_IF_W_WATCHMAN1	5
#define USB_IF_LIGHTCAP		6
#define USB_IF_TRACKER0_LIGHTCAP		7
#define USB_IF_W_WATCHMAN1_LIGHTCAP		8
#define MAX_INTERFACES		9

typedef struct SurviveUSBInterface SurviveUSBInterface;
typedef struct SurviveViveData SurviveViveData;

typedef void (*usb_callback)( SurviveUSBInterface * ti );

#ifdef HIDAPI
#define USBHANDLE hid_device *
#else
#define USBHANDLE libusb_device_handle *
#endif

struct SurviveUSBInterface
{
	SurviveViveData * sv;
	SurviveContext * ctx;

#ifdef HIDAPI
	USBHANDLE uh;
#else
	struct libusb_transfer * transfer;
#endif
	SurviveObject * assoc_obj;
	int actual_len;
	uint8_t buffer[INTBUFFSIZE];
	usb_callback cb;
	int which_interface_am_i;	//for indexing into uiface
	const char * hname;			//human-readable names
};

struct SurviveViveData
{
	SurviveContext * ctx;

	SurviveUSBInterface uiface[MAX_INTERFACES];

	USBHANDLE udev[MAX_USB_DEVS];
#ifdef HIDAPI
	og_thread_t servicethread[MAX_USB_DEVS];
#else
	struct libusb_context* usbctx;
#endif
};

#ifdef HIDAPI
og_mutex_t      GlobalRXUSBMutx;
#endif

void survive_data_cb( SurviveUSBInterface * si );

//USB Subsystem 
void survive_usb_close( SurviveContext * t );
int survive_usb_init( SurviveViveData * sv, SurviveObject * hmd, SurviveObject *wm0, SurviveObject * wm1, SurviveObject * tr0 , SurviveObject * ww0 );
int survive_usb_poll( SurviveContext * ctx );
int survive_get_config( char ** config, SurviveViveData * ctx, int devno, int iface, int send_extra_magic );
int survive_vive_send_magic(SurviveContext * ctx, void * drv, int magic_code, void * data, int datalen );

#ifdef HIDAPI
void * HAPIReceiver( void * v )
{

	SurviveUSBInterface * iface = v;
	USBHANDLE * hp = &iface->uh;
	
	while( (iface->actual_len = hid_read( *hp, iface->buffer, sizeof( iface->buffer ) )) > 0 )
	{
		//if( iface->actual_len  == 52 ) continue;
		OGLockMutex( GlobalRXUSBMutx );
#if 0
		printf( "%d %d: ", iface->which_interface_am_i, iface->actual_len );
		int i;
		for( i = 0; i < iface->actual_len; i++ )
		{
			printf( "%02x ", iface->buffer[i] );
		}
		printf("\n" );
#endif
		survive_data_cb( iface );
		OGUnlockMutex( GlobalRXUSBMutx );
	}
	//XXX TODO: Mark device as failed.
	*hp = 0;
	return 0;
}

#else
static void handle_transfer(struct libusb_transfer* transfer)
{
	SurviveUSBInterface * iface = transfer->user_data;
	SurviveContext * ctx = iface->ctx;

	if( transfer->status != LIBUSB_TRANSFER_COMPLETED )
	{
		SV_ERROR("Transfer problem %d with %s", transfer->status, iface->hname );
		SV_KILL();
		return;
	}

	iface->actual_len = transfer->actual_length;
	iface->cb( iface );

	if( libusb_submit_transfer(transfer) )
	{
		SV_ERROR( "Error resubmitting transfer for %s", iface->hname );
		SV_KILL();
	}
}
#endif


static int AttachInterface( SurviveViveData * sv, SurviveObject * assocobj, int which_interface_am_i, USBHANDLE devh, int endpoint, usb_callback cb, const char * hname )
{
	SurviveContext * ctx = sv->ctx;
	SurviveUSBInterface * iface = &sv->uiface[which_interface_am_i];
	iface->ctx = ctx;
	iface->sv = sv;
	iface->which_interface_am_i = which_interface_am_i;
	iface->assoc_obj = assocobj;
	iface->hname = hname;
	iface->cb = cb;

#ifdef HIDAPI
	//What do here?
	iface->uh = devh;
	sv->servicethread[which_interface_am_i] = OGCreateThread( HAPIReceiver, iface );
	OGUSleep(100000);
#else
	struct libusb_transfer * tx = iface->transfer = libusb_alloc_transfer(0);
	//printf( "%p %d %p %p\n", iface, which_interface_am_i, tx, devh );

	if (!iface->transfer)
	{
		SV_ERROR( "Error: failed on libusb_alloc_transfer for %s", hname );
		return 4;
	}

	libusb_fill_interrupt_transfer( tx, devh, endpoint, iface->buffer, INTBUFFSIZE, handle_transfer, iface, 0);

	int rc = libusb_submit_transfer( tx );
	if( rc )
	{
		SV_ERROR( "Error: Could not submit transfer for %s (Code %d)", hname, rc );
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

//XXX TODO: Redo this subsystem for setting/updating feature reports.


#ifdef HIDAPI

static inline int update_feature_report(USBHANDLE dev, uint16_t iface, uint8_t * data, int datalen )
{
	int r = hid_send_feature_report( dev, data, datalen );
//	printf( "HUR: (%p) %d (%d) [%d]\n", dev, r, datalen, data[0] );
	return r;
}
static inline int getupdate_feature_report(USBHANDLE dev, uint16_t iface, uint8_t * data, size_t datalen ) 
{
	int r = hid_get_feature_report( dev, data, datalen );
//	printf( "HGR: (%p) %d (%d) (%d)\n", dev, r, datalen, data[0] );
	if( r == -1 ) return -9; //Pretend it's not a critical error
	return r;
}

#else
	
static inline int update_feature_report(libusb_device_handle* dev, uint16_t interface, uint8_t * data, int datalen ) {
//	int xfer;
//	int r = libusb_interrupt_transfer(dev, 0x01, data, datalen, &xfer, 1000);
//	printf( "XFER: %d / R: %d\n", xfer, r );
//	return xfer;
	return libusb_control_transfer(dev, LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT,
		0x09, 0x300 | data[0], interface, data, datalen, 1000 );
}


static inline int getupdate_feature_report(libusb_device_handle* dev, uint16_t interface, uint8_t * data, int datalen ) {

	int ret = libusb_control_transfer(dev, LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_IN,
		0x01, 0x300 | data[0], interface, data, datalen, 1000 );
	if( ret == -9 ) return -9;
    if (ret < 0)
        return -1;
	return ret;
}

#endif

static inline int hid_get_feature_report_timeout(USBHANDLE device, uint16_t iface, unsigned char *buf, size_t len )
{
	int ret;
	uint8_t i = 0;
    for (i = 0; i < 50; i++)
	{
        ret = getupdate_feature_report(device, iface, buf, len);
		if( ret != -9 && ( ret != -1 || errno != EPIPE ) ) return ret;
		OGUSleep( 1000 );
	}

	return -1;
}

int survive_usb_init( SurviveViveData * sv, SurviveObject * hmd, SurviveObject *wm0, SurviveObject * wm1, SurviveObject * tr0 , SurviveObject * ww0 )
{
	SurviveContext * ctx = sv->ctx;
#ifdef HIDAPI
	if( !GlobalRXUSBMutx )
	{
		GlobalRXUSBMutx = OGCreateMutex();
		OGLockMutex( GlobalRXUSBMutx );
	}
	int res, i;
	res = hid_init();
	if( res )
	{
		SV_ERROR( "Could not setup hidapi." );
		return res;
	}
	
	for( i = 0; i < MAX_USB_DEVS; i++ )
	{
		int enumid = vidpids[i*3+2];
		int vendor_id = vidpids[i*3+0];
		int product_id = vidpids[i*3+1];
		struct hid_device_info * devs = hid_enumerate(vendor_id, product_id);
		struct hid_device_info * cur_dev = devs;
		const char *path_to_open = NULL;
		hid_device *handle = NULL;
		int menum = 0;
		
		cur_dev = devs;
		while (cur_dev) {
			if (cur_dev->vendor_id == vendor_id &&
				cur_dev->product_id == product_id)
			{
				if( cur_dev->interface_number == enumid ||
					cur_dev->interface_number == -1 && menum == enumid)
				{
					path_to_open = cur_dev->path;
					break;
				}
				menum++;
			}
			cur_dev = cur_dev->next;
		}

		if (path_to_open) {
			handle = hid_open_path(path_to_open);
		}

		hid_free_enumeration(devs);

		if( !handle )
		{
			SV_INFO( "Warning: Could not find vive device %04x:%04x", vendor_id, product_id );
			continue;
		}
	
		// Read the Serial Number String
		wchar_t wstr[255];

		res = hid_get_serial_number_string(handle, wstr, 255);
		printf("Found %s. ", devnames[i]);
		wprintf(L"Serial Number String: (%d) %s for %04x:%04x@%d  (Dev: %p)\n", wstr[0], wstr,vendor_id, product_id, menum, handle);
		
		sv->udev[i] = handle;

	}

#else
	int r = libusb_init( &sv->usbctx );
	if( r )
	{
		SV_ERROR( "libusb fault %d\n", r );
		return r;
	}

	int i;
	int16_t j;
	libusb_device** devs;
	int ret = libusb_get_device_list(sv->usbctx, &devs);

	if( ret < 0 )
	{
		SV_ERROR( "Couldn't get list of USB devices %d", ret );
		return ret;
	}


	//Open all interfaces.
	for( i = 0; i < MAX_USB_DEVS; i++ )
	{
		libusb_device * d;
		int vid = vidpids[i*3+0];
		int pid = vidpids[i*3+1];
		int which = vidpids[i*3+2];

		int did;
		for( did = 0; d = devs[did]; did++ )
		{
			struct libusb_device_descriptor desc;

			int ret = libusb_get_device_descriptor( d, &desc);
			if (ret < 0) {
				continue;
			}

			if( desc.idVendor == vid && desc.idProduct == pid)
			{
				if( which == 0 ) break;
				which--;
			}
		}

		if( d == 0 )
		{
			printf( "!!%p  %d %04x %04x %d\n", devnames[i], i, vid, pid, which );
			SV_INFO( "Did not find device %s (%04x:%04x.%d)", devnames[i], vid, pid, which );
			sv->udev[i] = 0;
			continue;
		}

		struct libusb_config_descriptor *conf;
		ret = libusb_get_config_descriptor(d, 0, &conf);
		if( ret )
			continue;
		ret = libusb_open(d, &sv->udev[i]);

		if( !sv->udev[i] || ret )
		{
			SV_ERROR( "Error: cannot open device \"%s\" with vid/pid %04x:%04x", devnames[i], vid, pid );
			return -5;
		}

		libusb_set_auto_detach_kernel_driver( sv->udev[i], 1 );
		for (j = 0; j < conf->bNumInterfaces; j++ )
		{
#if 0
		    if (libusb_kernel_driver_active(sv->udev[i], j) == 1) {
		        ret = libusb_detach_kernel_driver(sv->udev[i], j);
		        if (ret != LIBUSB_SUCCESS) {
		            SV_ERROR("Failed to unclaim interface %d for device %s "
		                    "from the kernel.", j, devnames[i] );
		            libusb_free_config_descriptor(conf);
		            libusb_close(sv->udev[i]);
		            continue;
		        }
		    }
#endif

			if( libusb_claim_interface(sv->udev[i], j) )
			{
				SV_ERROR( "Could not claim interface %d of %s", j, devnames[i] );
				return -9;
			}
		}

		SV_INFO( "Successfully enumerated %s (%d, %d)", devnames[i], did, conf->bNumInterfaces );
	}
	
	libusb_free_device_list( devs, 1 );
#endif

	if( sv->udev[USB_DEV_HMD] && AttachInterface( sv, hmd, USB_IF_HMD,        sv->udev[USB_DEV_HMD],        0x81, survive_data_cb, "Mainboard" ) ) { return -6; }
	if( sv->udev[USB_DEV_HMD_IMU_LH] && AttachInterface( sv, hmd, USB_IF_HMD_IMU_LH, sv->udev[USB_DEV_HMD_IMU_LH], 0x81, survive_data_cb, "Lighthouse" ) ) { return -7; }
	if( sv->udev[USB_DEV_WATCHMAN1] && AttachInterface( sv, wm0, USB_IF_WATCHMAN1,  sv->udev[USB_DEV_WATCHMAN1],  0x81, survive_data_cb, "Watchman 1" ) ) { return -8; }
	if( sv->udev[USB_DEV_WATCHMAN2] && AttachInterface( sv, wm1, USB_IF_WATCHMAN2, sv->udev[USB_DEV_WATCHMAN2], 0x81, survive_data_cb, "Watchman 2")) { return -9; }
	if( sv->udev[USB_DEV_TRACKER0] && AttachInterface( sv, tr0, USB_IF_TRACKER0, sv->udev[USB_DEV_TRACKER0], 0x81, survive_data_cb, "Tracker 1")) { return -10; }
	if( sv->udev[USB_DEV_W_WATCHMAN1] && AttachInterface( sv, ww0, USB_IF_W_WATCHMAN1, sv->udev[USB_DEV_W_WATCHMAN1], 0x81, survive_data_cb, "Wired Watchman 1")) { return -11; }
#ifdef HIDAPI
	//Tricky: use other interface for actual lightcap.  XXX THIS IS NOT YET RIGHT!!!
	if( sv->udev[USB_DEV_HMD_IMU_LHB] && AttachInterface( sv, hmd, USB_IF_LIGHTCAP, sv->udev[USB_DEV_HMD_IMU_LHB], 0x82, survive_data_cb, "Lightcap")) { return -12; }

	// This is a HACK!  But it works.  Need to investigate further
	sv->uiface[USB_DEV_TRACKER0_LIGHTCAP].actual_len = 64;
	if( sv->udev[USB_DEV_TRACKER0_LIGHTCAP] && AttachInterface( sv, tr0, USB_IF_TRACKER0_LIGHTCAP, sv->udev[USB_DEV_TRACKER0_LIGHTCAP], 0x82, survive_data_cb, "Tracker 1 Lightcap")) { return -13; }
	if( sv->udev[USB_DEV_W_WATCHMAN1_LIGHTCAP] && AttachInterface( sv, ww0, USB_IF_W_WATCHMAN1_LIGHTCAP, sv->udev[USB_DEV_W_WATCHMAN1_LIGHTCAP], 0x82, survive_data_cb, "Wired Watchman 1 Lightcap")) { return -13; }
#else
	if( sv->udev[USB_DEV_HMD_IMU_LH] && AttachInterface( sv, hmd, USB_IF_LIGHTCAP, sv->udev[USB_DEV_HMD_IMU_LH], 0x82, survive_data_cb, "Lightcap")) { return -12; }
	if( sv->udev[USB_DEV_TRACKER0] && AttachInterface( sv, ww0, USB_IF_TRACKER0_LIGHTCAP, sv->udev[USB_DEV_TRACKER0], 0x82, survive_data_cb, "Tracker 0 Lightcap")) { return -13; }
	if( sv->udev[USB_DEV_W_WATCHMAN1] && AttachInterface( sv, ww0, USB_IF_W_WATCHMAN1_LIGHTCAP, sv->udev[USB_DEV_W_WATCHMAN1], 0x82, survive_data_cb, "Wired Watchman 1 Lightcap")) { return -13; }
#endif
	SV_INFO( "All enumerated devices attached." );

	survive_vive_send_magic(ctx, sv, 1, 0, 0 );

	//libUSB initialized.  Continue.
	return 0;
}

int survive_vive_send_magic(SurviveContext * ctx, void * drv, int magic_code, void * data, int datalen )
{
	int r;
	SurviveViveData * sv = drv;
	printf( "*CALLING %p %p\n", ctx, sv );

	//XXX TODO: Handle haptics, etc.
	int turnon = magic_code;


	if( turnon )
	{
		//From actual steam.
		if (sv->udev[USB_DEV_HMD])
		{
			static uint8_t vive_magic_power_on[64] = {  0x04, 0x78, 0x29, 0x38 };
			r = update_feature_report( sv->udev[USB_DEV_HMD], 0, vive_magic_power_on, sizeof( vive_magic_power_on ) );
			if( r != sizeof( vive_magic_power_on ) ) return 5;
		}

		if (sv->udev[USB_DEV_HMD_IMU_LH])
		{
			static uint8_t vive_magic_enable_lighthouse[5] = { 0x04 };
			r = update_feature_report( sv->udev[USB_DEV_HMD_IMU_LH], 0, vive_magic_enable_lighthouse, sizeof( vive_magic_enable_lighthouse ) );
			if( r != sizeof( vive_magic_enable_lighthouse ) ) return 5;

			static uint8_t vive_magic_enable_lighthouse2[5] = { 0x07, 0x02 };  //Switch to 0x25 mode (able to get more light updates)
			r = update_feature_report( sv->udev[USB_DEV_HMD_IMU_LH], 0, vive_magic_enable_lighthouse2, sizeof( vive_magic_enable_lighthouse2 ) );
			if( r != sizeof( vive_magic_enable_lighthouse2 ) ) return 5;
		}

		if (sv->udev[USB_DEV_W_WATCHMAN1])
		{
			static uint8_t vive_magic_power_on[5] = { 0x04 };
			r = update_feature_report( sv->udev[USB_DEV_W_WATCHMAN1], 0, vive_magic_power_on, sizeof( vive_magic_power_on ) );
			if( r != sizeof( vive_magic_power_on ) ) return 5;
		}

#ifdef HIDAPI
		if (sv->udev[USB_DEV_W_WATCHMAN1_LIGHTCAP])
		{
			static uint8_t vive_magic_enable_lighthouse[5] = { 0x04 };
			r = update_feature_report( sv->udev[USB_DEV_W_WATCHMAN1_LIGHTCAP], 0, vive_magic_enable_lighthouse, sizeof( vive_magic_enable_lighthouse ) );
			if( r != sizeof( vive_magic_enable_lighthouse ) ) return 5;

			static uint8_t vive_magic_enable_lighthouse2[5] = { 0x07, 0x02 };  //Switch to 0x25 mode (able to get more light updates)
			r = update_feature_report( sv->udev[USB_DEV_W_WATCHMAN1_LIGHTCAP], 0, vive_magic_enable_lighthouse2, sizeof( vive_magic_enable_lighthouse2 ) );
			if( r != sizeof( vive_magic_enable_lighthouse2 ) ) return 5;
		}

#else
		if (sv->udev[USB_DEV_W_WATCHMAN1])
		{
			static uint8_t vive_magic_enable_lighthouse[5] = { 0x04 };
			r = update_feature_report( sv->udev[USB_DEV_W_WATCHMAN1], 0, vive_magic_enable_lighthouse, sizeof( vive_magic_enable_lighthouse ) );
			if( r != sizeof( vive_magic_enable_lighthouse ) ) return 5;

			static uint8_t vive_magic_enable_lighthouse2[5] = { 0x07, 0x02 };  //Switch to 0x25 mode (able to get more light updates)
			r = update_feature_report( sv->udev[USB_DEV_W_WATCHMAN1], 0, vive_magic_enable_lighthouse2, sizeof( vive_magic_enable_lighthouse2 ) );
			if( r != sizeof( vive_magic_enable_lighthouse2 ) ) return 5;
		}

#endif

		if (sv->udev[USB_DEV_TRACKER0])
		{
			static uint8_t vive_magic_power_on[5] = { 0x04 };
			r = update_feature_report( sv->udev[USB_DEV_TRACKER0], 0, vive_magic_power_on, sizeof( vive_magic_power_on ) );
			if( r != sizeof( vive_magic_power_on ) ) return 5;
		}
//#ifdef HIDAPI
//		if (sv->udev[USB_DEV_TRACKER0_LIGHTCAP])
//		{
//			static uint8_t vive_magic_enable_lighthouse[5] = { 0x04 };
//			r = update_feature_report( sv->udev[USB_DEV_TRACKER0_LIGHTCAP], 0, vive_magic_enable_lighthouse, sizeof( vive_magic_enable_lighthouse ) );
//			if( r != sizeof( vive_magic_enable_lighthouse ) ) return 5;
//
//			static uint8_t vive_magic_enable_lighthouse2[5] = { 0x07, 0x02 };  //Switch to 0x25 mode (able to get more light updates)
//			r = update_feature_report( sv->udev[USB_DEV_TRACKER0_LIGHTCAP], 0, vive_magic_enable_lighthouse2, sizeof( vive_magic_enable_lighthouse2 ) );
//			if( r != sizeof( vive_magic_enable_lighthouse2 ) ) return 5;
//		}
//#else		
		if (sv->udev[USB_DEV_TRACKER0])
		{
			static uint8_t vive_magic_enable_lighthouse[5] = { 0x04 };
			r = update_feature_report( sv->udev[USB_DEV_TRACKER0], 0, vive_magic_enable_lighthouse, sizeof( vive_magic_enable_lighthouse ) );
			if( r != sizeof( vive_magic_enable_lighthouse ) ) return 5;

			static uint8_t vive_magic_enable_lighthouse2[5] = { 0x07, 0x02 };  //Switch to 0x25 mode (able to get more light updates)
			r = update_feature_report( sv->udev[USB_DEV_TRACKER0], 0, vive_magic_enable_lighthouse2, sizeof( vive_magic_enable_lighthouse2 ) );
			if( r != sizeof( vive_magic_enable_lighthouse2 ) ) return 5;
		}

//#endif

#if 0
		for( int i = 0; i < 256; i++ )
		{
			static uint8_t vive_controller_haptic_pulse[64] = { 0xff, 0x8f, 0xff, 0, 0, 0, 0, 0, 0, 0 };
			//r = update_feature_report( sv->udev[USB_DEV_WATCHMAN1], 0, vive_controller_haptic_pulse, sizeof( vive_controller_haptic_pulse ) );
			r = update_feature_report( sv->udev[USB_DEV_W_WATCHMAN1_LIGHTCAP], 0, vive_controller_haptic_pulse, sizeof( vive_controller_haptic_pulse ) );
			SV_INFO( "UCR: %d", r );
			if( r != sizeof( vive_controller_haptic_pulse ) ) return 5;
			OGUSleep( 1000 );
		}
#endif

		//if (sv->udev[USB_DEV_TRACKER0])
		//{
		//	static uint8_t vive_magic_power_on[64] = {  0x04, 0x78, 0x29, 0x38 };
		//	r = update_feature_report( sv->udev[USB_DEV_TRACKER0], 0, vive_magic_power_on, sizeof( vive_magic_power_on ) );
		//	if( r != sizeof( vive_magic_power_on ) ) return 5;
		//}

		SV_INFO( "Powered unit on." );
	}
	else
	{

		static uint8_t vive_magic_power_off1[] = {
			0x04, 0x78, 0x29, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
			0x30, 0x05, 0x77, 0x00, 0x30, 0x05, 0x77, 0x00, 0x6c, 0x4d, 0x37, 0x65, 0x40,
			0xf9, 0x33, 0x00, 0x04, 0xf8, 0xa3, 0x04, 0x04, 0x00, 0x00, 0x00, 0x70, 0xb0,
			0x72, 0x00, 0xf4, 0xf7, 0xa3, 0x04, 0x7c, 0xf8, 0x33, 0x00, 0x0c, 0xf8, 0xa3,
			0x04, 0x0a, 0x6e, 0x29, 0x65, 0x24, 0xf9, 0x33, 0x00, 0x00, 0x00, 0x00,
		};

		static uint8_t vive_magic_power_off2[] = {
			0x04, 0x78, 0x29, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00,
			0x30, 0x05, 0x77, 0x00, 0xe4, 0xf7, 0x33, 0x00, 0xe4, 0xf7, 0x33, 0x00, 0x60,
			0x6e, 0x72, 0x00, 0xb4, 0xf7, 0x33, 0x00, 0x04, 0x00, 0x00, 0x00, 0x70, 0xb0,
			0x72, 0x00, 0x90, 0xf7, 0x33, 0x00, 0x7c, 0xf8, 0x33, 0x00, 0xd0, 0xf7, 0x33,
			0x00, 0x3c, 0x68, 0x29, 0x65, 0x24, 0xf9, 0x33, 0x00, 0x00, 0x00, 0x00,
		};

//		r = update_feature_report( sv->udev[USB_DEV_HMD], 0, vive_magic_power_off1, sizeof( vive_magic_power_off1 ) );
//		SV_INFO( "UCR: %d", r );
//		if( r != sizeof( vive_magic_power_off1 ) ) return 5;

		if (sv->udev[USB_DEV_HMD])
		{
			r = update_feature_report( sv->udev[USB_DEV_HMD], 0, vive_magic_power_off2, sizeof( vive_magic_power_off2 ) );
			SV_INFO( "UCR: %d", r );
			if( r != sizeof( vive_magic_power_off2 ) ) return 5;
		}
	}
	return 0;
}

void survive_vive_usb_close( SurviveViveData * sv )
{
	int i;
#ifdef HIDAPI
	for( i = 0; i < MAX_USB_DEVS; i++ )
	{
		if( sv->udev[i] )
			hid_close( sv->udev[i] );
	}
	for( i = 0; i < MAX_USB_DEVS; i++ )
	{
		OGJoinThread( sv->servicethread[i] );
	}
	//This is global, don't do it on account of other tasks.
	//hid_exit();

#else
	for( i = 0; i < MAX_USB_DEVS; i++ )
	{
	    libusb_close( sv->udev[i] );
	}
    libusb_exit(sv->usbctx);	
#endif
}

int survive_vive_usb_poll( SurviveContext * ctx, void * v )
{
#ifdef HIDAPI
	OGUnlockMutex( GlobalRXUSBMutx );
	OGUSleep( 100 );
	OGUnlockMutex( GlobalRXUSBMutx );
	return 0;
#else
	SurviveViveData * sv = v;
	int r = libusb_handle_events( sv->usbctx );
	if( r )
	{
		SurviveContext * ctx = sv->ctx;
		SV_ERROR( "Libusb poll failed." );
	}
	return r;
#endif
	return 0;
}


int survive_get_config( char ** config, SurviveViveData * sv, int devno, int iface, int send_extra_magic )
{
	SurviveContext * ctx = sv->ctx;
	int ret, count = 0, size = 0;
	uint8_t cfgbuff[64];
	uint8_t compressed_data[8192];
	uint8_t uncompressed_data[65536];
	USBHANDLE dev = sv->udev[devno];

	if( send_extra_magic )
	{
		uint8_t cfgbuffwide[65];

		memset( cfgbuffwide, 0, sizeof( cfgbuff ) );
		cfgbuffwide[0] = 0x01;
		ret = hid_get_feature_report_timeout( dev, iface, cfgbuffwide, sizeof( cfgbuffwide ) );
		OGUSleep(1000);

		int k;
		
		uint8_t cfgbuff_send[64] = { 0xff, 0x83 };
			
		#ifdef HIDAPI
		//XXX TODO WRITEME
		for( k = 0; k < 10; k++ )
		{
			OGUSleep( 1000);
		}
	
		#else
		//Switch mode to pull config?
		for( k = 0; k < 10; k++ )
		{
			update_feature_report( dev, iface, cfgbuff_send, 64 );
			OGUSleep(1000);
		}
		#endif

		cfgbuffwide[0] = 0xff;
		ret = hid_get_feature_report_timeout( dev, iface, cfgbuffwide, sizeof( cfgbuffwide ) );
		OGUSleep(1000);
	}

	// Send Report 16 to prepare the device for reading config info
	memset( cfgbuff, 0, sizeof( cfgbuff ) );
	cfgbuff[0] = 0x10;
	if( ( ret = hid_get_feature_report_timeout( dev, iface, cfgbuff, sizeof( cfgbuff ) ) ) < 0 )
	{
		SV_INFO( "Could not get survive config data for device %d:%d", devno, iface );
		return -1;
	}


	// Now do a bunch of Report 17 until there are no bytes left
	cfgbuff[1] = 0xaa;
	cfgbuff[0] = 0x11;
	do
	{
		if( (ret = hid_get_feature_report_timeout(dev, iface, cfgbuff, sizeof( cfgbuff ) ) ) < 0 )
		{
			SV_INFO( "Could not read config data (after first packet) on device %d:%d (count: %d)\n", devno, iface, count );
			return -2;
		}

		size = cfgbuff[1];

		if( !size ) break;

		if( size > 62 )
		{
			SV_INFO( "Too much data (%d) on packet from config for device %d:%d (count: %d)", size, devno, iface, count );
			return -3;
		}

		if( count + size >= sizeof( compressed_data ) )
		{
			SV_INFO( "Configuration length too long %d:%d (count: %d)", devno, iface, count );
			return -4;
		}

        memcpy( &compressed_data[count], cfgbuff + 2, size );
		count += size;
	} while( 1 );

	if( count == 0 )
	{
		SV_INFO( "Empty configuration for %d:%d", devno, iface );
		return -5;
	}

	SV_INFO( "Got config data length %d", count );

	char fstname[128];
	sprintf( fstname, "calinfo/%d.json.gz", devno );
	FILE * f = fopen( fstname, "wb" );
	fwrite( compressed_data, count, 1, f );
	fclose( f );
	
	int len = survive_simple_inflate( ctx, compressed_data, count, uncompressed_data, sizeof(uncompressed_data)-1 );
	if( len <= 0 )
	{
		SV_INFO( "Error: data for config descriptor %d:%d is bad. (%d)", devno, iface, len );
		return -5;
	}

	*config = malloc( len + 1 );
	memcpy( *config, uncompressed_data, len );
	return len;
}

///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////

#define POP1  (*(readdata++))
#define POP2  (*(((uint16_t*)((readdata+=2)-2))))
#define POP4  (*(((uint32_t*)((readdata+=4)-4))))

static void handle_watchman( SurviveObject * w, uint8_t * readdata )
{

	uint8_t startread[29];
	memcpy( startread, readdata, 29 );

#if 0
	printf( "DAT:     " );
		for(int i = 0; i < 29; i++ )
		{
			printf( "%02x ", readdata[i] );
		}
		printf("\n");
#endif

	uint8_t time1 = POP1;
	uint8_t qty = POP1;
	uint8_t time2 = POP1;
	uint8_t type = POP1;
	qty-=2;
	int propset = 0;
	int doimu = 0;

	if( (type & 0xf0) == 0xf0 )
	{
		propset |= 4;
		//printf( "%02x %02x %02x %02x\n", qty, type, time1, time2 );
		type &= ~0x10;

		if( type & 0x01 )
		{
			qty-=1;
			w->buttonmask = POP1;
			type &= ~0x01;
		}
		if( type & 0x04 ) 
		{
			qty-=1;
			w->axis1 = ( POP1 ) * 128; 
			type &= ~0x04;
		}
		if( type & 0x02 )
		{
			qty-=4;
			w->axis2 = POP2;
			w->axis3 = POP2;
			type &= ~0x02;
		}

		//XXX TODO: Is this correct?  It looks SO WACKY
		type &= 0x7f;
		if( type == 0x68 ) doimu = 1;
		type &= 0x0f;
		if( type == 0x00 && qty ) { type = POP1; qty--; }
	}

	if( type == 0xe1 )
	{
		propset |= 1;
		w->charging = readdata[0]>>7;
		w->charge = POP1&0x7f; qty--;
		w->ison = 1; 
		if( qty )
		{
			qty--;
			type = POP1; //IMU usually follows.
		}
	}

	if( ( ( type & 0xe8 ) == 0xe8 ) || doimu ) //Hmm, this looks kind of yucky... we can get e8's that are accelgyro's but, cleared by first propset.
	{
		propset |= 2;
		//XXX XXX BIG TODO!!! Actually recal gyro data.
		FLT agm[9] = { readdata[1], readdata[2], readdata[3],
						readdata[4], readdata[5], readdata[6],
						0,0,0 };

		w->ctx->imuproc( w, 3, agm, (time1<<24)|(time2<<16)|readdata[0], 0 );

		int16_t * k = (int16_t *)readdata+1;
		//printf( "Match8 %d %d %d %d %d %3d %3d\n", qty, k[0], k[1], k[2], k[3], k[4], k[5] );
		readdata += 13; qty -= 13;
		type &= ~0xe8;
		if( qty )
		{
			qty--;
			type = POP1;
		}
	}


	if( qty )
	{
		qty++;
		readdata--;
		*readdata = type; //Put 'type' back on stack.
		uint8_t * mptr = readdata + qty-3-1; //-3 for timecode, -1 to 

#ifdef DEBUG_WATCHMAN
		printf( "_%s ", w->codename);
		for(int i = 0; i < qty; i++ )
		{
			printf( "%02x ", readdata[i] );
		}
		printf("\n");
#endif


		uint32_t mytime = (mptr[3] << 16)|(mptr[2] << 8)|(mptr[1] << 0);

		uint32_t times[20];
		const int nrtime = sizeof(times)/sizeof(uint32_t);
		int timecount = 0;
		int leds;
		int fault = 0;

		///Handle uint32_tifying (making sure we keep it incrementing)
		uint32_t llt = w->last_lighttime;
		uint32_t imumsb = time1<<24;
		mytime |= imumsb;

		//Compare mytime to llt

		int diff = mytime - llt;
		if( diff < -0x1000000 )
			mytime += 0x1000000;
		else if( diff > 0x100000 )
			mytime -= 0x1000000;

		w->last_lighttime = mytime;

		times[timecount++] = mytime;
#ifdef DEBUG_WATCHMAN
		printf( "_%s Packet Start Time: %d\n", w->codename, mytime );
#endif	

		//First, pull off the times, starting with the current time, then all the delta times going backwards.
		{
			while( mptr - readdata > (timecount>>1) )
			{
				uint32_t arcane_value = 0;
				//ArcanePop (Pop off values from the back, forward, checking if the MSB is set)
				do
				{
					uint8_t ap = *(mptr--);
					arcane_value |= (ap&0x7f);
					if( ap & 0x80 )  break;
					arcane_value <<= 7;
				} while(1);
				times[timecount++] = (mytime -= arcane_value);
#ifdef DEBUG_WATCHMAN
				printf( "_%s Time: %d  newtime: %d\n", w->codename, arcane_value, mytime );
#endif
			}

			leds = timecount>>1;
			//Check that the # of sensors at the beginning match the # of parameters we would expect.
			if( timecount & 1 ) { fault = 1; goto end; }				//Inordinal LED count
			if( leds != mptr - readdata + 1 ) { fault = 2; goto end; }	//LED Count does not line up with parameters
		}


		LightcapElement les[10];
		int lese = 0; //les's end


		//Second, go through all LEDs and extract the lightevent from them. 
		{
			uint8_t *marked;
			marked = alloca(nrtime);
			memset( marked, 0, nrtime );
			int i, parpl = 0;
			timecount--;
			int timepl = 0;

			//This works, but usually returns the values in reverse end-time order.
			for( i = 0; i < leds; i++ )
			{
				int led = readdata[i];
				int adv = led & 0x07;
				led >>= 3;

				while( marked[timepl] ) timepl++;

#ifdef DEBUG_WATCHMAN
				int i;
				printf( "TP %d   TC: %d : ", timepl, timecount );
				for( i = 0; i < nrtime; i++ )
				{
					printf( "%d", marked[i] );
				}
				printf( "\n" );
#endif

				if( timepl > timecount ) { fault = 3; goto end; }         //Ran off max of list.
				uint32_t endtime = times[timepl++];

				int end = timepl + adv;
				if( end > timecount ) { fault = 4; goto end; } //end referencing off list
				if( marked[end] > 0 ) { fault = 5; goto end; } //Already marked trying to be used.
				uint32_t starttime = times[end];
				marked[end] = 1;

				//Insert all lighting things into a sorted list.  This list will be
				//reverse sorted, but that is to minimize operations.  To read it
				//in sorted order simply read it back backwards.
				//Use insertion sort, since we should most of the time, be in order.
				LightcapElement * le = &les[lese++];
				le->sensor_id = led;

				if( (uint32_t)(endtime - starttime) > 65535 ) { fault = 6; goto end; } //Length of pulse dumb.
				le->length = endtime - starttime;
				le->timestamp = starttime;

#ifdef DEBUG_WATCHMAN
				printf( "_%s Event: %d %d %d-%d\n", w->codename, led, le->length, endtime, starttime );
#endif
				int swap = lese-2;
				while( swap >= 0 && les[swap].timestamp < les[swap+1].timestamp )
				{
					LightcapElement l;
					memcpy( &l, &les[swap], sizeof( l ) );
					memcpy( &les[swap], &les[swap+1], sizeof( l ) );
					memcpy( &les[swap+1], &l, sizeof( l ) );
					swap--;
				}
			}
		}

		int i;
		for( i = lese-1; i >= 0; i-- )
		{
			//printf( "%d: %d [%d]\n", les[i].sensor_id, les[i].length, les[i].timestamp );
			handle_lightcap( w, &les[i] );
		}

		return;
	end:
		{
			SurviveContext * ctx = w->ctx;
			SV_INFO( "Light decoding fault: %d", fault );
		}
	}
}


void survive_data_cb( SurviveUSBInterface * si )
{
	int size = si->actual_len;
	SurviveContext * ctx = si->ctx;

	int iface = si->which_interface_am_i;
	SurviveObject * obj = si->assoc_obj;
	uint8_t * readdata = si->buffer;

	int id = POP1;
//	printf( "%16s Size: %2d ID: %d / %d\n", si->hname, size, id, iface );


#if 0
	if(  si->which_interface_am_i == 5 )
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

	switch( si->which_interface_am_i )
	{
	case USB_IF_HMD:
	{
		SurviveObject * headset = obj;
		readdata+=2;
		headset->buttonmask = POP1;		//Lens
		headset->axis2 = POP2;			//Lens Separation
		readdata+=2;
		headset->buttonmask |= POP1;	//Button
		readdata+=3;
		readdata++;						//Proxchange, No change = 0, Decrease = 1, Increase = 2
		readdata++;
		headset->axis3 = POP2;			//Proximity  	<< how close to face are you?  Less than 80 = not on face.
		headset->axis1 = POP2;			//IPD   		<< what is this?
		headset->ison = 1;
		break;
	}
	case USB_IF_HMD_IMU_LH:
	case USB_IF_W_WATCHMAN1:
	case USB_IF_TRACKER0:
	{
		int i;
		//printf( "%d -> ", size );
		for( i = 0; i < 3; i++ )
		{
			int16_t * acceldata = (int16_t*)readdata;
			readdata += 12;
			uint32_t timecode = POP4;
			uint8_t code = POP1;
			//printf( "%d ", code );
			int8_t cd = code - obj->oldcode;

			if( cd > 0 )
			{
				obj->oldcode = code;

				//XXX XXX BIG TODO!!! Actually recal gyro data.
				FLT agm[9] = { acceldata[0], acceldata[1], acceldata[2],
								acceldata[3], acceldata[4], acceldata[5],
								0,0,0 };

				//1G for accelerometer, from MPU6500 datasheet
				//this can change if the firmware changes the sensitivity.
				agm[0]/=8192.0f;
				agm[1]/=8192.0f;
				agm[2]/=8192.0f;

				//1000 deg/s for gyroscope, from MPU6500 datasheet
				agm[3]/=65.5f;
				agm[4]/=65.5f;
				agm[5]/=65.5f;

				ctx->imuproc( obj, 3, agm, timecode, code );
			}
		}

		//DONE OK.
		break;
	}
	case USB_IF_WATCHMAN1:
	case USB_IF_WATCHMAN2:
	{
		SurviveObject * w = obj;
		if( id == 35 )
		{
			handle_watchman( w, readdata);
		}
		else if( id == 36 )
		{
			handle_watchman( w, readdata);
			handle_watchman( w, readdata+29 );
		}
		else if( id == 38 )
		{
			w->ison = 0;
		}
		else
		{
			SV_INFO( "Unknown watchman code %d\n", id );
		}
		break;
	}
	case USB_IF_LIGHTCAP:
	{
		int i;
		for( i = 0; i < 9; i++ )
		{
			LightcapElement le;
			le.sensor_id = POP1;
			le.length = POP2;
			le.timestamp = POP4;
			if( le.sensor_id > 0xfd ) continue;
			handle_lightcap( obj, &le );
		}		
		break;
	}
	case USB_IF_W_WATCHMAN1_LIGHTCAP:
	case USB_IF_TRACKER0_LIGHTCAP:
	{
		int i=0;
		for( i = 0; i < 7; i++ )
		{
			unsigned short *sensorId = (unsigned short *)readdata;
			unsigned short *length = (unsigned short *)(&(readdata[2]));
			unsigned long *time = (unsigned long *)(&(readdata[4]));
			LightcapElement le;
			le.sensor_id = (uint8_t)POP2;
			le.length = POP2;
			le.timestamp = POP4;
			if( le.sensor_id > 0xfd ) continue;  //
			handle_lightcap( obj, &le );
		}		
		break;
	}
	}
}










///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////



static int jsoneq(const char *json, jsmntok_t *tok, const char *s) {
 if (tok->type == JSMN_STRING && (int) strlen(s) == tok->end - tok->start &&
    strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
		return 0;
	}
	return -1;
}


static int ParsePoints( SurviveContext * ctx, SurviveObject * so, char * ct0conf, FLT ** floats_out, jsmntok_t * t, int i )
{
	int k;
	int pts = t[i+1].size;
	jsmntok_t * tk;

	so->nr_locations = 0;
	*floats_out = malloc( sizeof( **floats_out ) * 32 * 3 );

	for( k = 0; k < pts; k++ )
	{
		tk = &t[i+2+k*4];

		int m;
		for( m = 0; m < 3; m++ )
		{
			char ctt[128];

			tk++;
			int elemlen = tk->end - tk->start;

			if( tk->type != 4 || elemlen > sizeof( ctt )-1 )
			{
				SV_ERROR( "Parse error in JSON\n" );
				return 1;
			}

			memcpy( ctt, ct0conf + tk->start, elemlen );
			ctt[elemlen] = 0;
			FLT f = atof( ctt );
			int id = so->nr_locations*3+m;
			(*floats_out)[id] = f;
		}
		so->nr_locations++;
	}
	return 0;
}

static int LoadConfig( SurviveViveData * sv, SurviveObject * so, int devno, int iface, int extra_magic )
{
	SurviveContext * ctx = sv->ctx;
	char * ct0conf = 0;
	int len = survive_get_config( &ct0conf, sv, devno, iface, extra_magic );
printf( "Loading config: %d\n", len );
#if 0
	char fname[100];
	sprintf( fname, "%s_config.json", so->codename );
	FILE * f = fopen( fname, "w" );
	fwrite( ct0conf, strlen(ct0conf), 1, f );
	fclose( f );
#endif

	if( len > 0 )
	{

		//From JSMN example.
		jsmn_parser p;
		jsmntok_t t[4096];
		jsmn_init(&p);
		int i;
		int r = jsmn_parse(&p, ct0conf, len, t, sizeof(t)/sizeof(t[0]));	
		if (r < 0) {
			SV_INFO("Failed to parse JSON in HMD configuration: %d\n", r);
			return -1;
		}
		if (r < 1 || t[0].type != JSMN_OBJECT) {
			SV_INFO("Object expected in HMD configuration\n");
			return -2;
		}

		for (i = 1; i < r; i++) {
			jsmntok_t * tk = &t[i];

			char ctxo[100];
			int ilen = tk->end - tk->start;
			if( ilen > 99 ) ilen = 99;
			memcpy(ctxo, ct0conf + tk->start, ilen);
			ctxo[ilen] = 0;

//				printf( "%d / %d / %d / %d %s %d\n", tk->type, tk->start, tk->end, tk->size, ctxo, jsoneq(ct0conf, &t[i], "modelPoints") );
//				printf( "%.*s\n", ilen, ct0conf + tk->start );

			if (jsoneq(ct0conf, tk, "modelPoints") == 0) {
				if( ParsePoints( ctx, so, ct0conf, &so->sensor_locations, t, i  ) )
				{
					break;
				}
			}
			if (jsoneq(ct0conf, tk, "modelNormals") == 0) {
				if( ParsePoints( ctx, so, ct0conf, &so->sensor_normals, t, i  ) )
				{
					break;
				}
			}
		}
	}
	else
	{
		//TODO: Cleanup any remaining USB stuff.
		return 1;
	}

	char fname[64];

	sprintf( fname, "calinfo/%s_points.csv", so->codename );
	FILE * f = fopen( fname, "w" );
	int j;
	for( j = 0; j < so->nr_locations; j++ )
	{
		fprintf( f, "%f %f %f\n", so->sensor_locations[j*3+0], so->sensor_locations[j*3+1], so->sensor_locations[j*3+2] );
	}
	fclose( f );

	sprintf( fname, "calinfo/%s_normals.csv", so->codename );
	f = fopen( fname, "w" );
	for( j = 0; j < so->nr_locations; j++ )
	{
		fprintf( f, "%f %f %f\n", so->sensor_normals[j*3+0], so->sensor_normals[j*3+1], so->sensor_normals[j*3+2] );
	}
	fclose( f );

	return 0;
}


int survive_vive_close( SurviveContext * ctx, void * driver )
{
	SurviveViveData * sv = driver;
 
	survive_vive_usb_close( sv );
	return 0;
}


int DriverRegHTCVive( SurviveContext * ctx )
{
	int r;
	SurviveObject * hmd = calloc( 1, sizeof( SurviveObject ) );
	SurviveObject * wm0 = calloc( 1, sizeof( SurviveObject ) );
	SurviveObject * wm1 = calloc( 1, sizeof( SurviveObject ) );
	SurviveObject * tr0 = calloc( 1, sizeof( SurviveObject ) );
	SurviveObject * ww0 = calloc( 1, sizeof( SurviveObject ) );
	SurviveViveData * sv = calloc( 1, sizeof( SurviveViveData ) );

	sv->ctx = ctx;
	
	#ifdef _WIN32
		CreateDirectoryA("calinfo", NULL);
	#elif defined WINDOWS
		mkdir( "calinfo" );
	#else
		mkdir( "calinfo", 0755 );
	#endif


	hmd->ctx = ctx;
	memcpy( hmd->codename, "HMD", 4 );
	memcpy( hmd->drivername, "HTC", 4 );
	wm0->ctx = ctx;
	memcpy( wm0->codename, "WM0", 4 );
	memcpy( wm0->drivername, "HTC", 4 );
	wm1->ctx = ctx;
	memcpy( wm1->codename, "WM1", 4 );
	memcpy( wm1->drivername, "HTC", 4 );
	tr0->ctx = ctx;
	memcpy( tr0->codename, "TR0", 4 );
	memcpy( tr0->drivername, "HTC", 4 );
	ww0->ctx = ctx;
	memcpy( ww0->codename, "WW0", 4 );
	memcpy( ww0->drivername, "HTC", 4 );

	//USB must happen last.
	if( r = survive_usb_init( sv, hmd, wm0, wm1, tr0, ww0) )
	{
		//TODO: Cleanup any libUSB stuff sitting around.
		goto fail_gracefully;
	}

	//Next, pull out the config stuff.
	if( sv->udev[USB_DEV_HMD_IMU_LH]       && LoadConfig( sv, hmd, 1, 0, 0 )) { SV_INFO( "HMD config issue." ); }
	if( sv->udev[USB_DEV_WATCHMAN1] && LoadConfig( sv, wm0, 2, 0, 1 )) { SV_INFO( "Watchman 0 config issue." ); }
	if( sv->udev[USB_DEV_WATCHMAN2] && LoadConfig( sv, wm1, 3, 0, 1 )) { SV_INFO( "Watchman 1 config issue." ); }
	if( sv->udev[USB_DEV_TRACKER0]  && LoadConfig( sv, tr0, 4, 0, 0 )) { SV_INFO( "Tracker 0 config issue." ); }
	if( sv->udev[USB_DEV_W_WATCHMAN1]  && LoadConfig( sv, ww0, 5, 0, 0 )) { SV_INFO( "Wired Watchman 0 config issue." ); }

	hmd->timebase_hz = wm0->timebase_hz = wm1->timebase_hz = 48000000;
	tr0->timebase_hz = ww0->timebase_hz = hmd->timebase_hz;

	hmd->pulsedist_max_ticks = wm0->pulsedist_max_ticks = wm1->pulsedist_max_ticks = 500000;
	tr0->pulsedist_max_ticks = ww0->pulsedist_max_ticks = hmd->pulsedist_max_ticks;

	hmd->pulselength_min_sync = wm0->pulselength_min_sync = wm1->pulselength_min_sync = 2200;
	tr0->pulselength_min_sync = ww0->pulselength_min_sync = hmd->pulselength_min_sync;

	hmd->pulse_in_clear_time = wm0->pulse_in_clear_time = wm1->pulse_in_clear_time = 35000;
	tr0->pulse_in_clear_time = ww0->pulse_in_clear_time = hmd->pulse_in_clear_time;

	hmd->pulse_max_for_sweep = wm0->pulse_max_for_sweep = wm1->pulse_max_for_sweep = 1800;
	tr0->pulse_max_for_sweep = ww0->pulse_max_for_sweep = hmd->pulse_max_for_sweep;

	hmd->pulse_synctime_offset = wm0->pulse_synctime_offset = wm1->pulse_synctime_offset = 20000;
	tr0->pulse_synctime_offset = ww0->pulse_synctime_offset = hmd->pulse_synctime_offset;

	hmd->pulse_synctime_slack = wm0->pulse_synctime_slack = wm1->pulse_synctime_slack = 5000;
	tr0->pulse_synctime_slack = ww0->pulse_synctime_slack = hmd->pulse_synctime_slack;

	hmd->timecenter_ticks = hmd->timebase_hz / 240;
	wm0->timecenter_ticks = wm0->timebase_hz / 240;
	wm1->timecenter_ticks = wm1->timebase_hz / 240;
	tr0->timecenter_ticks = tr0->timebase_hz / 240;
	ww0->timecenter_ticks = ww0->timebase_hz / 240;
/*
	int i;
	int locs = hmd->nr_locations;
	printf( "Locs: %d\n", locs );
	if (hmd->sensor_locations )
	{
		printf( "POSITIONS:\n" );
		for( i = 0; i < locs*3; i+=3 )
		{
			printf( "%f %f %f\n", hmd->sensor_locations[i+0], hmd->sensor_locations[i+1], hmd->sensor_locations[i+2] );
		}
	}
	if( hmd->sensor_normals )
	{
		printf( "NORMALS:\n" );
		for( i = 0; i < locs*3; i+=3 )
		{
			printf( "%f %f %f\n", hmd->sensor_normals[i+0], hmd->sensor_normals[i+1], hmd->sensor_normals[i+2] );
		}
	}
*/

	//Add the drivers.
	if( sv->udev[USB_DEV_HMD_IMU_LH]       ) { survive_add_object( ctx, hmd ); }
	if( sv->udev[USB_DEV_WATCHMAN1] ) { survive_add_object( ctx, wm0 ); }
	if( sv->udev[USB_DEV_WATCHMAN2] ) { survive_add_object( ctx, wm1 ); }
	if( sv->udev[USB_DEV_TRACKER0]  ) { survive_add_object( ctx, tr0 ); }
	if( sv->udev[USB_DEV_W_WATCHMAN1]  ) { survive_add_object( ctx, ww0 ); }

	survive_add_driver( ctx, sv, survive_vive_usb_poll, survive_vive_close, survive_vive_send_magic );

	return 0;
fail_gracefully:
	free( hmd );
	free( wm0 );
	free( wm1 );
	free( tr0 );
	free( ww0 );
	survive_vive_usb_close( sv );
	free( sv );
	return -1;
}

REGISTER_LINKTIME( DriverRegHTCVive );

