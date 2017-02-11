// (C) 2016 Joshua Allen, MIT/x11 License.
//
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

/* ootx data decoder */

#include <stdio.h>
#include <stdlib.h>
#include <zlib.h>
#include <assert.h>
#include "ootx_decoder.h"
//#include "crc32.h"

//char* fmt_str = "L Y HMD %d 5 1 206230 %d\n";

#define MAX_BUFF_SIZE 1024

void (*ootx_packet_clbk)(ootx_packet* packet) = NULL;
void ootx_pump_bit(ootx_decoder_context *ctx, uint8_t dbit);

void ootx_init_decoder_context(ootx_decoder_context *ctx) {
	ctx->buf_offset = 0;
	ctx->bits_written = 0;

	ctx->preamble = 0XFFFFFFFF;
	ctx->bits_processed = 0;
	ctx->found_preamble = 0;

	ctx->buffer = (uint8_t*)malloc(MAX_BUFF_SIZE);
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
	if (delta>100000) return 0; //master
	if (delta>10000) return 1; //a slave
	return -1;
}

uint8_t decode_internal(uint32_t length) {
		uint16_t temp = length - 2880;
//		printf

#if BETTER_SAFE_THAN_FAST
	if (temp < 0 || length > 6525) {
		return -1;
	}
#endif

	if ((temp % 500) < 150) {
		return temp / 500;
	}

	return -1;

}

uint8_t ootx_decode_bit(uint32_t length) {
	length = ((length/500)*500)+500;

	length-=3000;
	if (length>=2000) { length-=2000; }
	if (length>=1000)  { return 0xFF; }

	return 0x00;
}
/*
uint8_t ootx_decode_bit(uint32_t ticks) {
	int8_t bits = decode_internal(ticks);
	return bits&0x02;
}
*/

void ootx_log_bit(ootx_decoder_context *ctx, uint32_t ticks) {
	int8_t dbit = ootx_decode_bit(ticks);
//	printf("%d\n\n", dbit);
	ctx->bit_count[(dbit&0x01)]++;
//	printf("%d %d %d\n", dbit, ctx->bit_count[0], ctx->bit_count[1]);
}

uint8_t ootx_pump_greatest_bit(ootx_decoder_context *ctx) {
	//pump the bit
	uint8_t bit = 0x00;
	if (ctx->bit_count[0] < ctx->bit_count[1]) bit = 0xFF;

	ootx_pump_bit( ctx, bit );

	ctx->bit_count[0] = 0;
	ctx->bit_count[1] = 0;

	return bit;
}

uint8_t ootx_detect_preamble(ootx_decoder_context *ctx, uint8_t dbit) {
	ctx->preamble <<= 1;
	ctx->preamble |= (0x01 & dbit);
	if ((ctx->preamble & 0x0003ffff) == 0x00000001) return 1;
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
	if(ctx->buf_offset>=MAX_BUFF_SIZE) {
		ctx->buf_offset = 0;
		ctx->found_preamble = 0;
	}

	ctx->buffer[ctx->buf_offset] = 0;
}

void ootx_write_to_buffer(ootx_decoder_context *ctx, uint8_t dbit) {
	uint8_t *current_byte = ctx->buffer + ctx->buf_offset;
//	printf("%d\n", dbit);
//	*current_byte >>= 1;
//	*current_byte |= (0x80 & dbit);
	*current_byte <<= 1;
	*current_byte |= (0x01 & dbit);
	++(ctx->bits_written);
	if (ctx->bits_written>7) {
		ctx->bits_written=0;
//		printf("%d\n", *current_byte);
		ootx_inc_buffer_offset(ctx);
	}
}

void ootx_process_bit(ootx_decoder_context *ctx, uint32_t length) {
	int8_t dbit = ootx_decode_bit(length);
	ootx_pump_bit( ctx, dbit );
}

void print_crc32(uint32_t crc) {
//	uint8_t* p = (uint32_t*)&crc;
//	uint8_t i = 0;

	printf("%X\n", crc);
}

void write_to_file(uint8_t *d, uint16_t length){
	FILE *fp = fopen("binary.data","w");
	fwrite(d, length, 1, fp);
	fclose(fp);
}

void ootx_pump_bit(ootx_decoder_context *ctx, uint8_t dbit) {
//	uint8_t dbit = ootx_decode_bit(length);
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
		ctx->bits_processed = 0;
	}
	else if (ctx->found_preamble > 0)
	{
		/*	only write to buffer if the preamble is found.
			if the buffer overflows, found_preamble will be cleared
			and writing will stop. data would be corrupted, so there is no point in continuing
		*/

		ootx_write_to_buffer(ctx, dbit);

		uint16_t padded_length = *(ctx->payload_size);
		padded_length += (padded_length&0x01); //extra null byte if odd

		if (ctx->buf_offset >= (padded_length+6)) {
			/*	once we have a complete ootx packet, send it out in the callback */
			ootx_packet op;

			op.length = *(ctx->payload_size);
			op.data = ctx->buffer+2;
			op.crc32 = *(uint32_t*)(op.data+padded_length);

			uint32_t crc = crc32( 0L, Z_NULL, 0 );
			crc = crc32( crc, op.data,op.length);
//			uint32_t crc = crc32(0xffffffff,op.data,op.length);


			if (crc != op.crc32) {
				printf("CRC mismatch\n");
/*
				printf("r:");
				print_crc32(op.crc32);

				printf("c:");
				print_crc32(crc);
//				write_to_file(op.data,op.length);
*/
			}

			if ((crc == op.crc32) && ootx_packet_clbk) ootx_packet_clbk(&op);

			ootx_reset_buffer(ctx);
		}
	}
}

