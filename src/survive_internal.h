#ifndef _SURVIVE_INTERNAL_H
#define _SURVIVE_INTERNAL_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <libusb-1.0/libusb.h>

#define SV_INFO( x... ) { char stbuff[1024]; sprintf( stbuff, x ); ctx->notefunction( ctx, stbuff ); }
#define SV_ERROR( x... ) { char stbuff[1024]; sprintf( stbuff, x ); ctx->faultfunction( ctx, stbuff ); }

//XXX TODO This one needs to be rewritten.
#define SV_KILL()		exit(0)

#define USB_DEV_HMD			0
#define USB_DEV_LIGHTHOUSE	1
#define USB_DEV_WATCHMAN1	2
#define USB_DEV_WATCHMAN2	3
#define MAX_USB_DEVS				4


#define USB_IF_HMD			0
#define USB_IF_LIGHTHOUSE 	1
#define USB_IF_WATCHMAN1	2
#define USB_IF_WATCHMAN2	3
#define USB_IF_LIGHTCAP		4
#define MAX_INTERFACES				5

#define INTBUFFSIZE			64
#define SENSORS_PER_OBJECT	32

struct SurviveContext;
struct SurviveUSBInterface;

typedef void (*usb_callback)( struct SurviveUSBInterface * ti );

struct SurviveUSBInterface
{
	struct libusb_transfer * transfer;
	struct SurviveContext * ctx;
	int actual_len;
	uint8_t buffer[INTBUFFSIZE];
	usb_callback cb;
	int which_interface_am_i;	//for indexing into uiface
	const char * hname;			//human-readable names
};

struct SurvivePhoto
{
	uint32_t last;
	uint32_t lastcode;
	uint32_t lastlength;
};

struct SurviveContext
{
	//USB Subsystem
    struct libusb_context* usbctx;
	void(*faultfunction)( struct SurviveContext * ctx, const char * fault );
	void(*notefunction)( struct SurviveContext * ctx, const char * fault );
	struct libusb_device_handle * udev[MAX_USB_DEVS];
	struct SurviveUSBInterface uiface[MAX_INTERFACES];

//	struct SurvivePhoto 
};


//USB Subsystem 
void survive_usb_close( struct SurviveContext * t );
int survive_usb_init( struct SurviveContext * t );
int survive_usb_poll( struct SurviveContext * ctx );

//Accept Data from backend.
void survive_data_cb( struct SurviveUSBInterface * si );

#endif


