// (C) 2016 Joshua Allen, MIT/x11 License.
//
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

/* ootx data decoder */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "ootx_decoder.h"
#include <crc32.h>

//char* fmt_str = "L Y HMD %d 5 1 206230 %d\n";


void (*ootx_packet_clbk)(ootx_decoder_context *ctx, ootx_packet* packet) = NULL;

void ootx_init_decoder_context(ootx_decoder_context *ctx) {
	ctx->buf_offset = 0;
	ctx->bits_written = 0;

	ctx->preamble = 0XFFFFFFFF;
	ctx->bits_processed = 0;
	ctx->found_preamble = 0;

	memset( ctx->buffer, 0, sizeof( ctx->buffer ) );
	ctx->payload_size = (uint16_t*)ctx->buffer;
	*(ctx->payload_size) = 0;
}
/*
void ootx_init_buffer() {
	buffer = (uint8_t*)malloc(MAX_BUFF_SIZE);
	payload_size = (uint16_t*)buffer;
	*payload_size = 0;
}
*/

/*
	how to decode pulses
	ticks>2000 && delta>100000== master lighthouse
	ticks>2000 && delta>10000 == slave lighthouse
*/

int8_t ootx_decode_lighthouse_number(uint8_t last_num, uint32_t ticks, int32_t delta) {
	if (ticks<2000) return -1; //sweep
	if ((ticks > 2000) & (delta>100000)) return 0; //master
	if ((ticks > 2000) & (delta>10000)) return last_num+1; //a slave
	return -1;
}

/*
uint8_t ootx_decode_bit(uint32_t ticks) {
	ticks = ((ticks/500)*500)+500;

	ticks-=3000;
	if (ticks>=2000) { ticks-=2000; }
	if (ticks>=1000)  { return 0xFF; }

	return 0x00;
}*/

uint8_t ootx_detect_preamble(ootx_decoder_context *ctx, uint8_t dbit) {
	ctx->preamble <<= 1;
	ctx->preamble |= (0x01 & dbit);
	if ((ctx->preamble & 0x0001ffff) == 0x01) return 1;
	return 0;
}

void ootx_reset_buffer(ootx_decoder_context *ctx) {
	ctx->buf_offset = 0;
	ctx->buffer[ctx->buf_offset] = 0;
	ctx->bits_written = 0;
	*(ctx->payload_size) = 0;
}

void ootx_inc_buffer_offset(ootx_decoder_context *ctx) {
	++(ctx->buf_offset);

//	assert(ctx->buf_offset<MAX_BUFF_SIZE);

	/* the buffer is going to overflow, wrap the buffer and don't write more data until the preamble is found again */
	if(ctx->buf_offset>=MAX_OOTX_BUFF_SIZE) {
		ctx->buf_offset = 0;
		ctx->found_preamble = 0;
	}

	ctx->buffer[ctx->buf_offset] = 0;
}

void ootx_write_to_buffer(ootx_decoder_context *ctx, uint8_t dbit) {
	uint8_t *current_byte = ctx->buffer + ctx->buf_offset;
//	printf("%d\n", dbit);
	*current_byte >>= 1;
	*current_byte |= (0x80 & dbit);
	++(ctx->bits_written);
	if (ctx->bits_written>7) {
		ctx->bits_written=0;
//		printf("%d\n", *current_byte);
		ootx_inc_buffer_offset(ctx);
	}
}

void ootx_process_bit(ootx_decoder_context *ctx, uint8_t dbit) {
	//uint8_t dbit = ootx_decode_bit(length);
	++(ctx->bits_processed);

//	printf("z %d %d\n", bits_processed,dbit);
//	printf("d %d\n", bits_processed,dbit);

	if ( ootx_detect_preamble(ctx, dbit) ) {
		/*	data stream can start over at any time so we must
			always look for preamble bits */
		printf("Preamble found\n");
		ootx_reset_buffer(ctx);
		ctx->bits_processed = 0;
		ctx->found_preamble = 1;
	}
	else if(ctx->bits_processed>16) {
		//every 17th bit needs to be dropped (sync bit)
//		printf("drop %d\n", dbit);
		if( !dbit )
		{
			printf( "Sync bit missing\n" );
			ootx_reset_buffer(ctx);
		}
		ctx->bits_processed = 0;
	}
	else if (ctx->found_preamble > 0)
	{
		/*	only write to buffer if the preamble is found.
			if the buffer overflows, found_preamble will be cleared
			and writing will stop. data would be corrupted, so there is no point in continuing
		*/

		ootx_write_to_buffer(ctx, dbit);
printf( "%d / %d -> ", ctx->buf_offset, *ctx->payload_size );
int k;
for( k = 0; k < 32; k++ )
{
	printf( "%02x ", ctx->buffer[k] );
}
printf( "\n" );
		if (ctx->buf_offset >= (*(ctx->payload_size)+6)) {
			/*	once we have a complete ootx packet, send it out in the callback */
			ootx_packet op;

			op.length = *(ctx->payload_size);
			op.data = ctx->buffer+2;
			op.crc32 = *(uint32_t*)(ctx->buffer+2+op.length);

			uint32_t crc = crc32(0xffffffff,op.data,op.length);

			if (crc != op.crc32) {
				printf("CRC mismatch\n");
			}

			if ((crc == op.crc32) && ootx_packet_clbk) ootx_packet_clbk(ctx, &op);

			ootx_reset_buffer(ctx);
		}
	}
}
