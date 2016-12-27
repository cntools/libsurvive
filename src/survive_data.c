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
#include "disambiguator.h"
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


//This is the disambiguator function, for taking light timing and figuring out place-in-sweep for a given photodiode.
static void handle_lightcap( struct SurviveObject * so, struct LightcapElement * le )
{
	struct SurviveContext * ct = so->ctx;
	int32_t deltat = (uint32_t)le->timestamp - (uint32_t)so->last_photo_time;

//	printf( "%s %d %d %d %d %d\n", so->codename, le->sensor_id, le->type, le->length, le->timestamp, le->timestamp-so->tsl );

	so->tsl = le->timestamp;
	if( le->type != 0xfe || le->length < 20 ) return;
#ifndef USE_OLD_DISAMBIGUATOR
	int32_t offset = le->timestamp - so->d->last;
	switch( disambiguator_step( so->d, le->timestamp, le->length ) ) {
	default:
	case P_SLAVE:
		// this is only interesting for the OOTX data
		break;
	case P_UNKNOWN:
		// not currently locked
		break;
	case P_MASTER:
		ct->lightproc( so, le->sensor_id, -1, 0, le->timestamp, offset );
		so->d->code = ((le->length+125)/250) - 12;
		break;
	case P_SWEEP:
		if (so->d->code & 1) return;
		ct->lightproc( so, le->sensor_id, so->d->code >> 1, offset, le->timestamp, le->length );
		break;
	}
#else
	if( le->length > 2200 ) //Pulse longer indicates a sync pulse.
	{
		int32_t deltat = (uint32_t)le->timestamp - (uint32_t)so->last_photo_time;
		if( deltat > 2000 || deltat < -2000 )		//New pulse. (may be inverted)
		{
			//See if this is a unique pulse, or another one in the same set we need to look at.
			if( le->timestamp - so->last_photo_time > 1500 + so->last_photo_length )
			{
				so->last_photo_time = le->timestamp;
				so->last_photo_length = le->length;
				ct->lightproc( so, le->sensor_id, -1, 0, le->timestamp, deltat );
				deltat = 0;
			}
		}

		//Find longest pulse-length from device in our window and use that one.
		if( le->length > so->last_photo_length )
		{
			so->last_photo_time = le->timestamp;
			so->last_photo_length = le->length;
		}
	}
	//See if this is a valid actual pulse.
	else if( le->length < 1800 && le->length > 40 && ( le->timestamp - so->last_photo_time < 380000 ) )
	{
		int32_t dl = so->last_photo_time;
		int32_t tpco = so->last_photo_length;

		//Adding length 
		int32_t offset_from = le->timestamp - dl + le->length/2;
		//Long pulse-code from IR flood.
		//Make sure it fits nicely into a divisible-by-500 time.
		int32_t acode = (tpco+125+50)/250;  //+10, seems ike that's
		if( acode & 1 ) return;

		acode >>= 1;
		acode -= 6;

		//printf( "%s / %d %d ++ %d %d\n", so->codename, dl, tpco, offset_from, acode );


		//Make sure pulse is in valid window
		if( offset_from < 380000 && offset_from > 20000 )
		{
			ct->lightproc( so, le->sensor_id, acode, offset_from, le->timestamp, le->length );
		}
	}
	else
	{
		//printf( "FAIL %d   %d - %d = %d\n", le->length, so->last_photo_time, le->timestamp, so->last_photo_time - le->timestamp );
		//Runt pulse, or no sync pulses available.
	}
#endif
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
		*readdata = type; //Put 'type' back on stack.
		uint8_t * mptr = readdata + qty-3-1; //-3 for timecode, -1 to 

//#define DEBUG_WATCHMAN
#ifdef DEBUG_WATCHMAN
		printf( "_%s ", w->codename);
		for( i = 0; i < qty; i++ )
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
		int parameters;
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

#ifdef DEBUG_WATCHMAN
				printf( "_%s Event: %d %d %d-%d\n", w->codename, led, le->length, endtime, starttime );
#endif
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
		{
			struct SurviveContext * ctx = w->ctx;
			SV_INFO( "Light decoding fault: %d\n", fault );
		}
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
			int8_t cd = code - ctx->headset.oldcode;

			if( cd > 0 )
			{
				ctx->headset.oldcode = code;
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

