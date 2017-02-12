// (C) 2017 Joshua Allen, MIT/x11 License.
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
void (*ootx_bad_crc_clbk)(ootx_packet* packet, uint32_t crc) = NULL;

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
	how to decode pulses
	ticks>2000 && delta>100000== master lighthouse
	ticks>2000 && delta>10000 == slave lighthouse
*/

int8_t ootx_decode_lighthouse_number(uint8_t last_num, uint32_t ticks, int32_t delta) {
	if (delta<18000) return -1; //sweep
//	if (ticks<2000) return -1; //sweep
//	printf ("%d\n", delta);


	if (ticks>2000 && delta>100000) return 0; //master
	if (delta>100000) return -1; //some kind of sweep related to the master

	/*	slaves are tricky. The first few sensor readings can be confused because their tick count could be too low because of the previous master pulse?
		so we have to ignore ticks completly
	*/
	if (delta>18000) return 1; //a slave, should be at least 20000 but there are some data issues
	return -1;
}
/*
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
*/
uint8_t ootx_decode_bit(uint32_t length) {
	length = ((length/500)*500)+500;

	length-=3000;
	if (length>=2000) { length-=2000; }
	if (length>=1000)  { return 0xFF; }

	return 0x00;
}

void ootx_accumulate_bit(ootx_decoder_context *ctx, uint8_t bit) {
	ctx->bit_count[bit&0x01]++;
}

uint8_t ootx_pump_greatest_bit(ootx_decoder_context *ctx) {
	//pump the bit
	uint8_t bit = 0x00;
	if (ctx->bit_count[0] < ctx->bit_count[1]) bit = 0xFF;

//	printf("pump %d\n", bit);
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

			if (crc != op.crc32) {
				if (ootx_bad_crc_clbk != NULL) ootx_bad_crc_clbk(&op,crc);
			}
			else if (ootx_packet_clbk != NULL) {
				ootx_packet_clbk(&op);
			}

			ootx_reset_buffer(ctx);
		}
	}
}

uint8_t* get_ptr(uint8_t* data, uint8_t bytes, uint16_t* idx) {
	uint8_t* x = data + *idx;
	*idx += bytes;
	return x;
}

float _to_float(uint8_t* data) {
	uint16_t x = *(uint16_t*)data;
	return x;
}

void init_lighthouse_info_v6(lighthouse_info_v6* lhi, uint8_t* data) {
	uint16_t idx = 0;
	/*
	uint16_t fw_version;//Firmware version (bit 15..6), protocol version (bit 5..0)
	uint32_t id; //Unique identifier of the base station
	float fcal_0_phase; //"phase" for rotor 0
	float fcal_1_phase; //"phase" for rotor 1
	float fcal_0_tilt; //"tilt" for rotor 0
	float fcal_1_tilt; //"tilt" for rotor 1
	uint8_t sys_unlock_count; //Lowest 8 bits of the rotor desynchronization counter
	uint8_t hw_version; //Hardware version
	float fcal_0_curve; //"curve" for rotor 0
	float fcal_1_curve; //"curve" for rotor 1
	int8_t accel_dir_x; //"orientation vector"
	int8_t accel_dir_y; //"orientation vector"
	int8_t accel_dir_z; //"orientation vector"
	float fcal_0_gibphase; //"gibbous phase" for rotor 0 (normalized angle)
	float fcal_1_gibphase; //"gibbous phase" for rotor 1 (normalized angle)
	float fcal_0_gibmag; //"gibbous magnitude" for rotor 0
	float fcal_1_gibmag; //"gibbous magnitude" for rotor 1
	uint8_t mode_current; //Currently selected mode (default: 0=A, 1=B, 2=C)
	uint8_t sys_faults; //"fault detect flags" (should be 0)
	*/

	lhi->fw_version = *(uint16_t*)get_ptr(data,2,&idx);
	lhi->id = *(uint32_t*)get_ptr(data,4,&idx);
	lhi->fcal_0_phase = _to_float( get_ptr(data,2,&idx) );
	lhi->fcal_1_phase = _to_float( get_ptr(data,2,&idx) );
	lhi->fcal_0_tilt = _to_float( get_ptr(data,2,&idx) );
	lhi->fcal_1_tilt = _to_float( get_ptr(data,2,&idx) );
	lhi->sys_unlock_count = *get_ptr(data,1,&idx);
	lhi->hw_version = *get_ptr(data,1,&idx);
	lhi->fcal_0_curve = _to_float( get_ptr(data,2,&idx) );
	lhi->fcal_1_curve = _to_float( get_ptr(data,2,&idx) );
	lhi->accel_dir_x = *(int8_t*)get_ptr(data,1,&idx);
	lhi->accel_dir_y = *(int8_t*)get_ptr(data,1,&idx);
	lhi->accel_dir_z = *(int8_t*)get_ptr(data,1,&idx);
	lhi->fcal_0_gibphase = _to_float( get_ptr(data,2,&idx) );
	lhi->fcal_1_gibphase = _to_float( get_ptr(data,2,&idx) );
	lhi->fcal_0_gibmag = _to_float( get_ptr(data,2,&idx) );
	lhi->fcal_1_gibmag = _to_float( get_ptr(data,2,&idx) );
	lhi->mode_current = *get_ptr(data,1,&idx);
	lhi->sys_faults = *get_ptr(data,1,&idx);

}

void print_lighthouse_info_v6(lighthouse_info_v6* lhi) {

	printf("\t%X\n\t%X\n\t%f\n\t%f\n\t%f\n\t%f\n\t%d\n\t%d\n\t%f\n\t%f\n\t%d\n\t%d\n\t%d\n\t%f\n\t%f\n\t%f\n\t%f\n\t%d\n\t%d\n",
		lhi->fw_version,
		lhi->id,
		lhi->fcal_0_phase,
		lhi->fcal_1_phase,
		lhi->fcal_0_tilt,
		lhi->fcal_1_tilt,
		lhi->sys_unlock_count,
		lhi->hw_version,
		lhi->fcal_0_curve,
		lhi->fcal_1_curve,
		lhi->accel_dir_x,
		lhi->accel_dir_y,
		lhi->accel_dir_z,
		lhi->fcal_0_gibphase,
		lhi->fcal_1_gibphase,
		lhi->fcal_0_gibmag,
		lhi->fcal_1_gibmag,
		lhi->mode_current,
		lhi->sys_faults);
}