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

void ootx_init_buffer();
void ootx_process_bit(uint32_t length);

extern void (*ootx_packet_clbk)(ootx_packet* packet);

#endif