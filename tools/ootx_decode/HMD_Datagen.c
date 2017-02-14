// (C) 2017 Joshua Allen, MIT/x11 License.
//
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

/* generate data to test ootx decoding */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <stdlib.h>
#include <zlib.h>

//this program is broken and does not produce useable data.

uint32_t time_stamp = -525198892;

char* fmt_str = "L Y HMD %d 5 1 206230 %d\n";

void print_bit(uint8_t data);
void print_preamble();
void print_uint16(uint16_t d);
void print_uint32(uint32_t d);
void print_payload(char* data, uint16_t length);


int main(int argc, char* argv[])
{
	char* str = "Hello World!";
//	printf("%s\n", str);

	srand(time(NULL));

	print_preamble();

	uint16_t payload_lenth = strlen(str);
	uint32_t crc = crc32( 0L, Z_NULL, 0 );
	crc = crc32( crc, (uint8_t*)str,payload_lenth);

	print_uint16(payload_lenth);
	print_payload(str,payload_lenth);
	print_uint32(crc);

	return 0;
}

void print_preamble() {
	int i;
	for (i=0;i<17;++i) print_bit(0);
	print_bit(1);
}

void print_uint16(uint16_t d) {
	int i;
	for (i=0;i<16;++i) {
		print_bit(d & 0x0001);
		d>>=1;
	}
	print_bit(1);
}

void print_uint32(uint32_t d) {
	int i = 0;
	for (;i<16;++i) {
		print_bit(d & 0x01);
		d>>=1;
	}
	print_bit(1);

	for (;i<32;++i) {
		print_bit(d & 0x01);
		d>>=1;
	}
	print_bit(1);
}

void print_payload(char* data, uint16_t length) {
	int i;
	for(i=0;i<length;i+=2) {
		uint16_t d = *((uint16_t*)(data+i));
//		printf("%d\n", d);
		print_uint16(d);
	}
}

void print_bit(uint8_t data) {
	uint32_t length = 3000 + (rand()%2)*500 + data*1000 + (rand()%2)*2000;
	length -= rand()%500;
	printf(fmt_str, time_stamp, length);

	time_stamp++;

	/*
	//to decode
	// 3000 + x*500 + dbit*1000 + y*2000
	length -= 3000;
	if (length>=2000) { length-=2000; y = 0x01; }
	if (length>=1000)  { length-=1000; dbit = 0x01; }
	if (length>=500)  { x = 0x01; }
	*/

	//fire off a callback when a full OOTX packet is received
}
