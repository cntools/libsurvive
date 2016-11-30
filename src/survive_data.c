
//Based off of vl_hid_reports (HTC Vive USB HID reports).  Who would license a header un the GPL???

#include "survive_internal.h"
#include <stdint.h>

#define POP1  (*(readdata++))
#define POP2  (*(((uint16_t*)((readdata+=2)-2))))
#define POP4  (*(((uint32_t*)readdata)++))


struct LightpulseStructure
{
	uint8_t id;			//Random divisible-by-2 numbers
	uint8_t type;		//Always 0
	int16_t unknown1;	//About  7,500
	int16_t unknown2;   //About -3,600
	int16_t unknown3;	//About 89 to 107
	int16_t unknown4;	//Normally around 0.
	int16_t unknown5;	//About -45, to -50
	uint32_t timestamp;
	uint8_t  unknown6;
} __attribute__((packed));


static void handle_lightdata( struct LightpulseStructure * p )
{
	//TODO: Wat?
	//printf( "%4d %4d (%6d %6d %6d %6d %6d) %10d %6d\n", p->id, p->type, p->unknown1, p->unknown2, p->unknown3, p->unknown4, p->unknown5, p->timestamp, p->unknown6 );
}

/*
struct vive_controller_analog_trigger_message {
	__u8 squeeze;
	__u8 unknown[4];
} __attribute__((packed));

struct vive_controller_button_message {
	__u8 buttons;
} __attribute__((packed));

struct vive_controller_touch_move_message {
	__le16 pos[2];
	__u8 unknown[4];
} __attribute__((packed));

struct vive_controller_touch_press_message {
	__u8 buttons;
	__le16 pos[2];
	__u8 unknown[4];
} __attribute__((packed));

struct vive_controller_imu_message {
	__u8 time3;
	__le16 accel[3];
	__le16 gyro[3];
	__u8 unknown[4];
} __attribute__((packed));

struct vive_controller_ping_message {
	__u8 charge : 7;
	__u8 charging : 1;
	__u8 unknown1[2];
	__le16 accel[3];
	__le16 gyro[3];
	__u8 unknown2[5];
} __attribute__((packed));

struct vive_controller_message {
	__u8 time1;
	__u8 sensor_id;
	__u8 time2;
	__u8 type;
	union {
		struct vive_controller_analog_trigger_message analog_trigger;
		struct vive_controller_button_message button;
		struct vive_controller_touch_move_message touch_move;
		struct vive_controller_touch_press_message touch_press;
		struct vive_controller_imu_message imu;
		struct vive_controller_ping_message ping;
		__u8 unknown[25];
	};
} __attribute__((packed));
*/

static void handle_watchman( int whichwatch, uint8_t * readdata )
{
	uint8_t time1 = POP1;
	uint8_t sensor_id = POP1;
	uint8_t time2 = POP1;
	uint8_t type = POP1;
	int i;

	switch(type)
	{
	case 0xe1: //Ping
		break;
	case 0xe8: //IMU
		break;
	case 0xf2: //Touch
		break;
	case 0xf3: //Button?
		break; //...many more F's.
	default:
		//It's a light!
		printf( "WM: %3d %3d %3d %3d %02x: ", whichwatch, time1, sensor_id, time2, type );
		for( i = 0; i < 26; i++ )
			printf( "%02x ", readdata[i] );
		printf("\n" );

		break;

	};

/*enum class vl_controller_type : uint8_t {
    PING = 0xe1,
    IMU = 0xe8,
    TOUCH = 0xf2,
    ANALOG_TRIGGER = 0xf4,*/
}


void survive_data_cb( struct SurviveUSBInterface * si )
{
	int size = si->actual_len;

#if 0
	int i;
	printf( "%16s: %d: ", si->hname, len );
	for( i = 0; i < size; i++ )
	{
		printf( "%02x ", si->buffer[i] );
	}
	printf( "\n" );
	return;
#endif 

	int iface = si->which_interface_am_i;
	uint8_t * readdata = si->buffer;

	int id = POP1;
//	printf( "%16s Size: %2d ID: %d / %d\n", si->hname, size, id, iface );


	switch( si->which_interface_am_i )
	{
	case USB_IF_HMD:
	{
		readdata+=2;
		int lens = POP1;		//Lens
		int lens_sep = POP2;	//Lens Separation
		readdata+=2;
		int btn = POP1;			//Button
		readdata+=3;
		readdata++;					//Proxchange, No change = 0, Decrease = 1, Increase = 2
		readdata++;
		int proximity = POP2;	//Proximity  	<< how close to face are you?  Less than 80 = not on face.
		int ipd = POP2;			//IPD   		<< what is this?

		//TODO: Store in thing.
		break;
	}
	case USB_IF_LIGHTHOUSE:
	{
		int i;
		for( i = 0; i < 3; i++ )
		{
			handle_lightdata( (struct LightpulseStructure *)readdata );
			readdata+= 17;
		}
		break;
	}
	case USB_IF_WATCHMAN1:
	case USB_IF_WATCHMAN2:
	{
		if( id == 35 )
		{
			handle_watchman( si->which_interface_am_i-USB_IF_WATCHMAN1, readdata);
		}
		else if( id == 36 )
		{
			handle_watchman( si->which_interface_am_i-USB_IF_WATCHMAN1, readdata);
			handle_watchman( si->which_interface_am_i-USB_IF_WATCHMAN1, readdata+29 );
		}
		else
		{
			printf( "Unknown watchman code\n" );
		}
		break;
	}
	case USB_IF_LIGHTCAP:
	{
		int i;
		for( i = 0; i < size; i++ )
		{
			printf( "%02x ", si->buffer[i] );
		}
		printf( "\n" );

		break;
	}
	}
}


/*
 * 
 * Copyright 2016 Philipp Zabel
 * SPDX-License-Identifier:	LGPL-2.0+
 */
#if 0

struct vive_headset_power_report {
	__u8 id;
	__le16 type;
	__u8 len;
	__u8 unknown1[9];
	__u8 reserved1[32];
	__u8 unknown2;
	__u8 reserved2[18];
} __attribute__((packed));

struct vive_headset_mainboard_device_info_report {
	__u8 id;
	__le16 type;
	__u8 len;
	__be16 edid_vid;
	__le16 edid_pid;
	__u8 unknown1[4];
	__le32 display_firmware_version;
	__u8 unknown2[48];
} __attribute__((packed));

struct vive_firmware_version_report {
	__u8 id;
	__le32 firmware_version;
	__le32 unknown1;
	__u8 string1[16];
	__u8 string2[16];
	__u8 hardware_version_micro;
	__u8 hardware_version_minor;
	__u8 hardware_version_major;
	__u8 hardware_revision;
	__le32 unknown2;
	__u8 fpga_version_minor;
	__u8 fpga_version_major;
	__u8 reserved[13];
} __attribute__((packed));

struct vive_headset_imu_sample {
        __s16 acc[3];
        __s16 rot[3];
	__le32 time_ticks;
	__u8 seq;
} __attribute__((packed));

struct vive_headset_imu_report {
	__u8 report_id;
	struct vive_headset_imu_sample samples[3];
} __attribute__((packed));



struct vive_controller_analog_trigger_message {
	__u8 squeeze;
	__u8 unknown[4];
} __attribute__((packed));

struct vive_controller_button_message {
	__u8 buttons;
} __attribute__((packed));

struct vive_controller_touch_move_message {
	__le16 pos[2];
	__u8 unknown[4];
} __attribute__((packed));

struct vive_controller_touch_press_message {
	__u8 buttons;
	__le16 pos[2];
	__u8 unknown[4];
} __attribute__((packed));

struct vive_controller_imu_message {
	__u8 time3;
	__le16 accel[3];
	__le16 gyro[3];
	__u8 unknown[4];
} __attribute__((packed));

struct vive_controller_ping_message {
	__u8 charge : 7;
	__u8 charging : 1;
	__u8 unknown1[2];
	__le16 accel[3];
	__le16 gyro[3];
	__u8 unknown2[5];
} __attribute__((packed));

struct vive_controller_message {
	__u8 time1;
	__u8 sensor_id;
	__u8 time2;
	__u8 type;
	union {
		struct vive_controller_analog_trigger_message analog_trigger;
		struct vive_controller_button_message button;
		struct vive_controller_touch_move_message touch_move;
		struct vive_controller_touch_press_message touch_press;
		struct vive_controller_imu_message imu;
		struct vive_controller_ping_message ping;
		__u8 unknown[25];
	};
} __attribute__((packed));

struct vive_controller_report1 {
	__u8 report_id;
	struct vive_controller_message message;
} __attribute__((packed));

struct vive_controller_report2 {
	__u8 report_id;
	struct vive_controller_message message[2];
} __attribute__((packed));

struct vive_headset_lighthouse_pulse2 {
        uint8_t sensor_id;
        uint16_t length;
        uint32_t timestamp;
} __attribute__((packed));

struct vive_headset_lighthouse_pulse_report2 {
	__u8 report_id;
	struct vive_headset_lighthouse_pulse2 samples[9];
} __attribute__((packed));

struct vive_controller_poweroff_report {
	__u8 id;
	__u8 command;
	__u8 len;
	__u8 magic[4];
} __attribute__((packed));


#endif
