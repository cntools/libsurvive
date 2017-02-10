// (C) 2016 Joshua Allen, MIT/x11 License.
//
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#ifndef OOTX_DECODER_H
#define OOTX_DECODER_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
	uint16_t length;
	uint8_t* data;
	uint32_t crc32;
} ootx_packet;

typedef struct {
	uint8_t* buffer;
	uint16_t buf_offset;
	uint8_t bits_written;
	uint16_t* payload_size;

	uint32_t preamble;
	uint8_t bits_processed;
	uint8_t found_preamble;
} ootx_decoder_context;


//void ootx_init_buffer();
void ootx_process_bit(ootx_decoder_context *ctx, uint32_t length);
void ootx_init_decoder_context(ootx_decoder_context *ctx);

extern void (*ootx_packet_clbk)(ootx_packet* packet);

#endif