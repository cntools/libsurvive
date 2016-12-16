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

	if( le->length > 2100 ) //Pulse longer indicates a sync pulse.
	{
		int32_t deltat = (uint32_t)le->timestamp - (uint32_t)ct->last_photo_time;
		if( deltat > 2000 || deltat < -2000 )		//New pulse. (may be inverted)
		{
			if( le->timestamp - ct->last_photo_time > 80000 )
			{
				ct->last_photo_time = le->timestamp;
				ct->total_photo_time = 0;
				ct->total_photos = 0;
				ct->total_pulsecode_time = 0;
				ct->lightproc( so, le->sensor_id, -1, 0, le->timestamp, deltat );
			}
		}
		else
		{
			ct->total_pulsecode_time += le->length;
			ct->total_photo_time += deltat;
			ct->total_photos++;
		}
	}
	else if( le->length < 1200 && le->length > 40 && ct->total_photos )
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
		acode -= 6;

		if( offset_from < 380000 )
		{
			ct->lightproc( so, le->sensor_id, acode, offset_from, le->timestamp, le->length );
		}
	}
	else
	{
		//Runt pulse.
	}
}



static void handle_watchman( struct SurviveObject * w, uint8_t * readdata )
{
	int i;

	uint8_t startread[29];
	memcpy( startread, readdata, 29 );

#if 0
	printf( "DAT:     " );
		for( i = 0; i < 29; i++ )
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
		w->ctx->imuproc( w, (int16_t *)&readdata[1], (time1<<24)|(time2<<16)|readdata[0], 0 );
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
			int j;
		qty++;
		readdata--;
		*readdata = type;

		uint8_t * end = &readdata[qty];
		uint32_t mytime = (end[1] << 0)|(end[2] << 8)|(end[3] << 16);

		uint32_t times[20];
		uint32_t time;
		const int nrtime = sizeof(times)/sizeof(uint32_t);
		int timecount = 0;
		int leds;
		int parameters;
		int fault = 0;

		//First, pull off the times, starting with the current time, then all the delta times going backwards.
		{
			uint8_t * mptr = readdata + qty-3-1; //-3 for timecode, -1 to advance.
			uint32_t imutime = (time1<<24) | (time2<<16);
			time = mptr[3]<<16 | mptr[2]<<8 | mptr[1];
			time |= (time1<<24);
			int32_t tdiff = time - imutime;
			if( tdiff > 0x7fffff )  { time -= 0x01000000; }
			if( tdiff < -0x7fffff ) { time += 0x01000000; }



			times[timecount++] = time;

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
				times[timecount++] = (time -= arcane_value);
			}

			leds = timecount>>1;
			//Check that the # of sensors at the beginning match the # of parameters we would expect.
			if( timecount & 1 ) { fault = 1; goto end; }				//Inordinal LED count
			if( leds != mptr - readdata + 1 ) { fault = 2; goto end; }	//LED Count does not line up with parameters
		}


		struct LightcapElement les[10];
		int lese = 0; //les's end

		//Second, go through all LEDs and extract the lightevent from them. 
		{
			uint8_t marked[nrtime];
			memset( marked, 0, sizeof( marked ) );
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
				if( timepl > timecount ) { fault = 3; goto end; }         //Ran off max of list.
				uint32_t endtime = times[timepl++];
				int end = timepl + adv;
				if( end > timecount ) { fault = 4; goto end; } //end referencing off list
				if( marked[end] > 0 ) { fault = 5; goto end; } //Already marked trying to be used.
				uint32_t starttime = times[end];
				marked[end] = 1;
				//printf( "LED: %d (adv %d) / st: %d en: %d len: %d / TC: %d\n", led, adv, starttime, endtime, endtime-starttime, end );


				//Insert all lighting things into a sorted list.  This list will be
				//reverse sorted, but that is to minimize operations.  To read it
				//in sorted order simply read it back backwards.
				//Use insertion sort, since we should most of the time, be in order.
				struct LightcapElement * le = &les[lese++];
				le->sensor_id = led;
				le->type = 0xfe;

				if( (uint32_t)(endtime - starttime) > 65535 ) { fault = 6; goto end; } //Length of pulse dumb.
				le->length = endtime - starttime;
				le->timestamp = starttime;
				int swap = lese-2;
				while( swap >= 0 && les[swap].timestamp < les[swap+1].timestamp )
				{
					struct LightcapElement l;
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
		printf( "FAULT: %d\n", fault );
	}
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
			int16_t * acceldata = (int16_t*)readdata;
			readdata += 12;
			uint32_t timecode = POP4;
			uint8_t code = POP1;
			//printf( "%d ", code );
			int8_t cd = code - ctx->oldcode;

			if( cd > 0 )
			{
				ctx->oldcode = code;
				ctx->imuproc( &ctx->headset, acceldata, timecode, code );
			}
		}

		//DONE OK.
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

