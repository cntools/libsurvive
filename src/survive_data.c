
//Based off of vl_hid_reports (HTC Vive USB HID reports).  Who would license a header un the GPL???

#include "survive_internal.h"
#include <stdint.h>
#include <string.h>

#define POP1  (*(readdata++))
#define POP2  (*(((uint16_t*)((readdata+=2)-2))))
#define POP4  (*(((uint32_t*)((readdata+=4)-4))))


struct LightcapElement
{
	uint8_t sensor_id;
	uint8_t type;
	uint16_t length;
	uint32_t timestamp;
} __attribute__((packed));



static void handle_lightcap( struct SurviveObject * so, struct LightcapElement * le )
{
	struct SurviveContext * ct = so->ctx;

	if( le->type != 0xfe || le->length < 50 ) return;
	//le->timestamp += (le->length/2);

	if( le->length > 900 ) //Pulse longer than 18us? 
	{
		int32_t deltat = (uint32_t)le->timestamp - (uint32_t)ct->last_photo_time;
		if( deltat > 2000 || deltat < -2000 )		//New pulse. (may be inverted)
		{
			ct->last_photo_time = le->timestamp;
			ct->total_photo_time = 0;
			ct->total_photos = 0;
			ct->total_pulsecode_time = 0;
			survive_light_process( so, le->sensor_id, -1, 0, le->timestamp );
		}
		else
		{
			ct->total_pulsecode_time += le->length;
			ct->total_photo_time += deltat;
			ct->total_photos++;
		}
	}
	else if( le->length < 900 && le->length > 50 && ct->total_photos )
	{
		int32_t dl = (ct->total_photo_time/ct->total_photos);
		int32_t tpco = (ct->total_pulsecode_time/ct->total_photos);
		//Adding length 
		int32_t offset_from = le->timestamp - dl - ct->last_photo_time + le->length/2;

		//Long pulse-code from IR flood.
		//Make sure it fits nicely into a divisible-by-500 time.
		int32_t acode = (tpco+125)/250;
		if( acode & 1 ) return;
		acode>>=1;

		survive_light_process( so, le->sensor_id, acode, offset_from, le->timestamp );
	}
	else
	{
		//Runt pulse.
	}
}



static void handle_watchman( struct SurviveObject * w, uint8_t * readdata )
{
	uint8_t startread[29];
	memcpy( startread, readdata, 29 );
	uint8_t time1 = POP1;
	uint8_t qty = POP1;
	uint8_t time2 = POP1;
	uint8_t type = POP1;
	int i;
	qty-=2;
	int propset = 0;

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
			w->axis1 = ( POP1 ) * 256; 
			type &= ~0x04;
		}
		if( type & 0x02 )
		{
			qty-=4;
			w->axis2 = POP2;
			w->axis3 = POP2;
			type &= ~0x02;
		}

		//XXX TODO: Maybe more data is here?
		if( type == 0xe0 )
		{
			type = 0x00;
		}
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

	if( ( type & 0xe8 ) == 0xe8 )
	{
		propset |= 2;
		survive_imu_process( w, (int16_t *)&readdata[1], (time1<<24)|(time2<<16)|readdata[0], 0 );
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
		*readdata = type;
		//Put type back on stack.  It might have changed
		//from above.


#if 0 //Doesn't work!!!!
		int reads = qty/6;

		int leds[6];
		leds[0] = type;
		for( i = 1; i < reads; i++ )
		{
			leds[i] = POP1;
		}
		for( i = 0; i < reads; i++ )
		{
			printf( "%02x: ", leds[i] );
			for( i = 0; i < 5; i++ )
			{
				printf( "%02x ", POP1 );
			}
			printf( "\n" );
		}
#endif

		//What does post data look like?

		//1) code, code, code,  PAAYYLOOAAADDD
		//"code" typically has 2 or fewer bits in word set.
		//  code = 0x00 = expect 5 bytes 
		//  code = 0x08 = expect 5 bytes
		//  code = 0x10 = expect 5 bytes
		//  code = 0x5c = ???? ... 12?
		//  code = 0x91 = ???? ... 25 bytes?


#if 1
		printf( "POST %d: %4d (%02x%02x) - ", propset, qty, time1, time2 );
		for( i = 0; i < qty; i++ )
		{
			printf( "%02x ", readdata[i] );
		}
		printf("\n");
#endif

#if 0
		printf( "  SRR: " );
		for( i = 0; i < 29; i++ )
		{
			printf( "%02x ", startread[i] );
		}
		printf( "\n" );
#endif
	}

	return;
	//NO, seriously, there is something wacky going on here.
//	else if( type == 0xe8 && sensor_id == 15 )
//	{
//		printf( "IMU\n" );
//	}

/*
	{
		printf( "PSIDPIN:%3d ", qty );
		w->charging = readdata[0]>>7;
		w->charge = readdata[0]&0x7f;
		w->ison = 1;
		printf( "%02x/%02x/%02x/%02x: ", type, qty, time1, time2 );
		for( i = 0; i < 25; i++ )
		{
			printf( "%02x ", readdata[i] );
		}
		printf("\n");

/*
		//WHAT IS THIS???
		printf( "%02x/%02x/%02x/%02x: ", type, sensor_id, time1, time2 );
		for( i = 0; i < 25; i++ )
		{
			printf( "%02x ", readdata[i] );
		}
		printf("\n");*/
}


void survive_data_cb( struct SurviveUSBInterface * si )
{
	int size = si->actual_len;
	struct SurviveContext * ctx = si->ctx;
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
		struct SurviveObject * headset = &ctx->headset;
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
	case USB_IF_LIGHTHOUSE:
	{
		int i;
		//printf( "%d -> ", size );
		for( i = 0; i < 3; i++ )
		{
			//handle_lightdata( (struct LightpulseStructure *)readdata );
			int16_t * acceldata = (int16_t*)readdata;
			readdata += 12;
			uint32_t timecode = POP4;
			uint8_t code = POP1;
			//printf( "%d ", code );
			int8_t cd = code - ctx->oldcode;

			if( cd > 0 )
			{
				ctx->oldcode = code;
				survive_imu_process( &ctx->headset, acceldata, timecode, code );
			}
		}
		//printf("\n" );
		break;
	}
	case USB_IF_WATCHMAN1:
	case USB_IF_WATCHMAN2:
	{
		struct SurviveObject * w = &ctx->watchman[si->which_interface_am_i-USB_IF_WATCHMAN1];
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
		//Done!
		int i;
		for( i = 0; i < 7; i++ )
		{
			handle_lightcap( &ctx->headset, (struct LightcapElement*)&readdata[i*8] );
		}
		break;
	}
	}
}

