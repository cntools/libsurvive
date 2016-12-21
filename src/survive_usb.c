//<>< (C) 2016 C. N. Lohr, MOSTLY Under MIT/x11 License.
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


#include "survive_internal.h"
#include <libusb-1.0/libusb.h>
#include <stdio.h>
#include <unistd.h> //sleep if I ever use it.
#include <errno.h>
#include <string.h>

const short vidpids[] = {
	0x0bb4, 0x2c87, 0, //The main HTC HMD device
	0x28de, 0x2000, 0, //Valve lighthouse
	0x28de, 0x2101, 0, //Valve Watchman
	0x28de, 0x2101, 1, //Valve Watchman
}; //length MAX_USB_INTERFACES*2

const char * devnames[] = {
	"HMD",
	"Lighthouse",
	"Watchman 1",
	"Watchman 2",
}; //length MAX_USB_INTERFACES


static void handle_transfer(struct libusb_transfer* transfer)
{
	struct SurviveUSBInterface * iface = transfer->user_data;
	struct SurviveContext * ctx = iface->ctx;

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



static int AttachInterface( struct SurviveContext * ctx, int which_interface_am_i, libusb_device_handle * devh, int endpoint, usb_callback cb, const char * hname )
{
	struct SurviveUSBInterface * iface = &ctx->uiface[which_interface_am_i];
	iface->ctx = ctx;
	iface->which_interface_am_i = which_interface_am_i;
	iface->hname = hname;
	struct libusb_transfer * tx = iface->transfer = libusb_alloc_transfer(0);
	iface->cb = cb;
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



static inline int hid_get_feature_report_timeout(libusb_device_handle* device, uint16_t interface, unsigned char *buf, size_t len )
{
	int ret;
    for (unsigned i = 0; i < 100; i++)
	{
        ret = getupdate_feature_report(device, interface, buf, len);
		if( ret != -9 && ( ret != -1 || errno != EPIPE ) ) return ret;
		usleep( 1000 );
	}

	return -1;
}


int survive_usb_init( struct SurviveContext * ctx )
{
	int r = libusb_init( &ctx->usbctx );
	if( r )
	{
		SV_ERROR( "libusb fault %d\n", r );
		return r;
	}

	int i;
	libusb_device** devs;
	int ret = libusb_get_device_list(ctx->usbctx, &devs);

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
			SV_ERROR( "Couldn't find device %s (%04x:%04x.%d)", devnames[i], vid, pid, which );
			return -99;
		}

		struct libusb_config_descriptor *conf;
		ret = libusb_get_config_descriptor(d, 0, &conf);
		if( ret )
			continue;

		ret = libusb_open(d, &ctx->udev[i]);

		if( !ctx->udev[i] || ret )
		{
			SV_ERROR( "Error: cannot open device \"%s\" with vid/pid %04x:%04x", devnames[i], vid, pid );
			return -5;
		}

		libusb_set_auto_detach_kernel_driver( ctx->udev[i], 1 );
		for (int j = 0; j < conf->bNumInterfaces; j++ )
		{
#if 0
		    if (libusb_kernel_driver_active(ctx->udev[i], j) == 1) {
		        ret = libusb_detach_kernel_driver(ctx->udev[i], j);
		        if (ret != LIBUSB_SUCCESS) {
		            SV_ERROR("Failed to unclaim interface %d for device %s "
		                    "from the kernel.", j, devnames[i] );
		            libusb_free_config_descriptor(conf);
		            libusb_close(ctx->udev[i]);
		            continue;
		        }
		    }
#endif

			if( libusb_claim_interface(ctx->udev[i], j) )
			{
				SV_ERROR( "Could not claim interface %d of %s", j, devnames[i] );
				return -9;
			}
		}

		SV_INFO( "Successfully enumerated %s (%d, %d)", devnames[i], did, conf->bNumInterfaces );
	}
	libusb_free_device_list( devs, 1 );

	if( AttachInterface( ctx, USB_IF_HMD,        ctx->udev[USB_DEV_HMD],        0x81, survive_data_cb, "Mainboard" ) ) { return -6; }
	if( AttachInterface( ctx, USB_IF_LIGHTHOUSE, ctx->udev[USB_DEV_LIGHTHOUSE], 0x81, survive_data_cb, "Lighthouse" ) ) { return -7; }
	if( AttachInterface( ctx, USB_IF_WATCHMAN1,  ctx->udev[USB_DEV_WATCHMAN1],  0x81, survive_data_cb, "Watchman 1" ) ) { return -8; }
	if( AttachInterface( ctx, USB_IF_WATCHMAN2,  ctx->udev[USB_DEV_WATCHMAN2],  0x81, survive_data_cb, "Watchman 2" ) ) { return -9; }
	if( AttachInterface( ctx, USB_IF_LIGHTCAP,   ctx->udev[USB_DEV_LIGHTHOUSE], 0x82, survive_data_cb, "Lightcap" ) ) { return -10; }

	SV_INFO( "All devices attached." );


	//libUSB initialized.  Continue.
	return 0;
}

int survive_usb_send_magic(struct SurviveContext * ctx, int turnon )
{
	int r;


	if( turnon )
	{
		//Magic from vl_magic.h, originally copywritten under LGPL. 
		// * Copyright (C) 2013 Fredrik Hultin
		// * Copyright (C) 2013 Jakob Bornecrantz
#if 0
		static uint8_t vive_magic_power_on[] = {
			0x04, 0x78, 0x29, 0x38, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x01,
			0xa8, 0x0d, 0x76, 0x00, 0x40, 0xfc, 0x01, 0x05, 0xfa, 0xec, 0xd1, 0x6d, 0x00,
			0x00, 0x6c, 0x00, 0x00, 0x00, 0x00, 0x00, 0xa8, 0x0d, 0x76, 0x00, 0x68, 0xfc,
			0x01, 0x05, 0x2c, 0xb0, 0x2e, 0x65, 0x7a, 0x0d, 0x76, 0x00, 0x68, 0x54, 0x72,
			0x00, 0x18, 0x54, 0x72, 0x00, 0x00, 0x6a, 0x72, 0x00, 0x00, 0x00, 0x00,
		};
#else
		//From actual steam.
		static uint8_t vive_magic_power_on[64] = {   0x04, 0x78, 0x29, 0x38,
			0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,	 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  0x00, 0x7a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#endif
		r = update_feature_report( ctx->udev[USB_DEV_HMD], 0, vive_magic_power_on, sizeof( vive_magic_power_on ) );
		if( r != sizeof( vive_magic_power_on ) ) return 5;

		static uint8_t vive_magic_enable_lighthouse[64] = { 0x04 };  //[64] wat?  Why did that fix it?
		r = update_feature_report( ctx->udev[USB_DEV_LIGHTHOUSE], 0, vive_magic_enable_lighthouse, sizeof( vive_magic_enable_lighthouse ) );
		if( r != sizeof( vive_magic_enable_lighthouse ) ) return 5;

#if 0
		for( i = 0; i < 256; i++ )
		{
			static uint8_t vive_controller_haptic_pulse[64] = { 0xff, 0x8f, 0xff, 0, 0, 0, 0, 0, 0, 0 };
			r = update_feature_report( ctx->udev[USB_DEV_WATCHMAN1], 0, vive_controller_haptic_pulse, sizeof( vive_controller_haptic_pulse ) );
			SV_INFO( "UCR: %d", r );
			if( r != sizeof( vive_controller_haptic_pulse ) ) return 5;
			usleep( 1000 );
		}
#endif
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

//		r = update_feature_report( ctx->udev[USB_DEV_HMD], 0, vive_magic_power_off1, sizeof( vive_magic_power_off1 ) );
//		SV_INFO( "UCR: %d", r );
//		if( r != sizeof( vive_magic_power_off1 ) ) return 5;

		r = update_feature_report( ctx->udev[USB_DEV_HMD], 0, vive_magic_power_off2, sizeof( vive_magic_power_off2 ) );
		SV_INFO( "UCR: %d", r );
		if( r != sizeof( vive_magic_power_off2 ) ) return 5;

	}

}

void survive_usb_close( struct SurviveContext * t )
{
	int i;
	for( i = 0; i < MAX_USB_DEVS; i++ )
	{
	    libusb_close( t->udev[i] );
	}
    libusb_exit(t->usbctx);	
}

int survive_usb_poll( struct SurviveContext * ctx )
{
	int r = libusb_handle_events( ctx->usbctx );
	if( r )
	{
		SV_ERROR( "Libusb poll failed." );
	}
	return r;
}


int survive_get_config( char ** config, struct SurviveContext * ctx, int devno, int iface, int send_extra_magic )
{
	int i, ret, count = 0, size = 0;
	uint8_t cfgbuff[64];
	uint8_t compressed_data[8192];
	uint8_t uncompressed_data[65536];
	struct libusb_device_handle * dev = ctx->udev[devno];

	if( send_extra_magic )
	{
		uint8_t cfgbuffwide[65];

		memset( cfgbuffwide, 0, sizeof( cfgbuff ) );
		cfgbuffwide[0] = 0x01;
		ret = hid_get_feature_report_timeout( dev, iface, cfgbuffwide, sizeof( cfgbuffwide ) );
		usleep(1000);

		int k;

		//Switch mode to pull config?
		for( k = 0; k < 10; k++ )
		{
			uint8_t cfgbuff_send[64] = { 
				0xff, 0x83, 0x00, 0xb6, 0x5b, 0xb0, 0x78, 0x69,
				0x0f, 0xf8, 0x78, 0x69, 0x0f, 0xa0, 0xf3, 0x18,
				0x00, 0xe8,	0xf2, 0x18, 0x00, 0x27, 0x44, 0x5a,
				0x0f, 0xf8, 0x78, 0x69, 0x0f, 0xf0, 0x77, 0x69,
				0x0f, 0xf0, 0x77, 0x69, 0x0f, 0x50, 0xca, 0x45,
				0x77, 0xa0, 0xf3, 0x18, 0x00, 0xf8, 0x78, 0x69,
				0x0f, 0x00, 0x00, 0xa0, 0x0f, 0xa0, 0x9b, 0x0a,
				0x01, 0x00, 0x00, 0x35, 0x00, 0x34, 0x02, 0x00
			};

			int rk = libusb_control_transfer(dev, LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE | LIBUSB_ENDPOINT_OUT,
			0x09, 0x300 | cfgbuff_send[0], iface, cfgbuff_send, 64, 1000 );
			usleep(1000);
		}

		cfgbuffwide[0] = 0xff;
		ret = hid_get_feature_report_timeout( dev, iface, cfgbuffwide, sizeof( cfgbuffwide ) );
		usleep(1000);
	}

	memset( cfgbuff, 0, sizeof( cfgbuff ) );
	cfgbuff[0] = 0x10;
	if( ( ret = hid_get_feature_report_timeout( dev, iface, cfgbuff, sizeof( cfgbuff ) ) ) < 0 )
	{
		SV_INFO( "Could not get survive config data for device %d:%d", devno, iface );
		return -1;
	}


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

	int len = survive_simple_inflate( ctx, compressed_data, count, uncompressed_data, sizeof(uncompressed_data)-1 );
	if( len <= 0 )
	{
		SV_INFO( "Error: data for config descriptor %d:%d is bad.", devno, iface );
		return -5;
	}

	*config = malloc( len + 1 );
	memcpy( *config, uncompressed_data, len );
	return len;
}


