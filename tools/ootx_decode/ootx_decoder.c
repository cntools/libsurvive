// (C) 2016 Joshua Allen, MIT/x11 License.
//
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

/* ootx data decoder */

#include <stdio.h>
#include <stdlib.h>

#include <assert.h>
#include "ootx_decoder.h"

//char* fmt_str = "L Y HMD %d 5 1 206230 %d\n";

#define MAX_BUFF_SIZE 1024
uint8_t* buffer = NULL;
uint16_t buf_offset = 0;
uint8_t bits_written = 0;
uint16_t* payload_size = NULL;

void (*ootx_packet_clbk)(ootx_packet* packet) = NULL;

void ootx_init_buffer() {
	buffer = (uint8_t*)malloc(MAX_BUFF_SIZE);
	payload_size = (uint16_t*)buffer;
	*payload_size = 0;
}

uint8_t ootx_decode_bit(uint32_t length) {
	length = ((length/500)*500)+500;

	length-=3000;
	if (length>=2000) { length-=2000; }
	if (length>=1000)  { return 0x01; }

	return 0x00;
}

uint8_t ootx_detect_preamble(uint8_t dbit) {
	static uint32_t preamble = 0x00;
	preamble <<= 1;
	preamble |= dbit;
	if ((preamble & 0x0001ffff) == 0x01) return 1;
	return 0;
}

void ootx_reset_buffer() {
	buf_offset = 0;
	buffer[buf_offset] = 0;
	bits_written = 0;
	*payload_size = 0;
}

void ootx_inc_buffer_offset() {
	++buf_offset;
//	if (buf_offset>=MAX_BUFF_SIZE) buf_offset = 0;
	assert(buf_offset<MAX_BUFF_SIZE);
	buffer[buf_offset] = 0;
}

void ootx_write_to_buffer(uint8_t dbit) {
	uint8_t *current_byte = buffer+buf_offset;
//	printf("%d\n", dbit);
	*current_byte >>= 1;
	*current_byte |= (0x80 * dbit);
	++bits_written;
	if (bits_written>7) {
		bits_written=0;
//		printf("%d\n", *current_byte);
		ootx_inc_buffer_offset();
	}
}

void ootx_process_bit(uint32_t length) {
	static uint8_t bits_processed = 0;

	uint8_t dbit = ootx_decode_bit(length);
	++bits_processed;

//	printf("z %d %d\n", bits_processed,dbit);
//	printf("d %d\n", bits_processed,dbit);

	if ( ootx_detect_preamble(dbit) ) {
		/*	data stream can start over at any time so we must
			always look for preamble bits */
//		printf("Preamble found\n");
		ootx_reset_buffer();
		bits_processed = 0;
	}
	else if(bits_processed>16) {
		//every 17th bit needs to be dropped
//		printf("drop %d\n", dbit);
		bits_processed = 0;
	}
	else
	{
		ootx_write_to_buffer(dbit);

		if (buf_offset >= (*payload_size+6)) {
			/*	once we have a complete ootx packet, send it out in the callback */
			ootx_packet op;

			op.length = *(uint16_t*)buffer;
			op.data = buffer+2;
			op.crc32 = *(uint32_t*)(buffer+2+op.length);
			if (ootx_packet_clbk) ootx_packet_clbk(&op);

			ootx_reset_buffer();
		}
	}
}

