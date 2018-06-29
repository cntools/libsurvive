// (C) 2017 Joshua Allen, MIT/x11 License.
//
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

/* ootx data decoder */

#include "ootx_decoder.h"
#include "string.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef NOZLIB
#include "crc32.h"
#else
#include <zlib.h>
#endif

//char* fmt_str = "L Y HMD %d 5 1 206230 %d\n";

#define MAX_BUFF_SIZE 64

void (*ootx_error_clbk)(ootx_decoder_context *ctx, const char *msg) = NULL;
void (*ootx_packet_clbk)(ootx_decoder_context * ctx, ootx_packet* packet) = NULL;
void (*ootx_bad_crc_clbk)(ootx_decoder_context * ctx, ootx_packet* packet, uint32_t crc) = NULL;

void ootx_pump_bit(ootx_decoder_context *ctx, uint8_t dbit);

void ootx_error(ootx_decoder_context *ctx, const char *msg) {
	if (ootx_error_clbk)
		ootx_error_clbk(ctx, msg);
}

void ootx_init_decoder_context(ootx_decoder_context *ctx) {
	ctx->buf_offset = 0;
	ctx->bits_written = 0;

	ctx->preamble = 0XFFFFFFFF;
	ctx->bits_processed = 0;
	ctx->found_preamble = 0;
	ctx->ignore_sync_bit_error = 0;

	ctx->buffer = (uint8_t*)malloc(MAX_BUFF_SIZE);
	ctx->payload_size = (uint16_t*)ctx->buffer;
	*(ctx->payload_size) = 0;
}

void ootx_free_decoder_context(ootx_decoder_context *ctx) {
	free(ctx->buffer);
	ctx->buffer = NULL;
	ctx->payload_size = NULL;
}

uint8_t ootx_decode_bit(uint32_t length) {
	uint8_t t = (uint8_t)((length - 2750) / 500); //why 2750?
//	return ((t & 0x02)>0)?0xFF:0x00; //easier if we need to bitshift right
	return ((t & 0x02)>>1);
}

uint8_t ootx_detect_preamble(ootx_decoder_context *ctx, uint8_t dbit) {
	ctx->preamble <<= 1;
//	ctx->preamble |= (0x01 & dbit);
	ctx->preamble |= dbit;
	if ((ctx->preamble & 0x0003ffff) == 0x00000001) return 1;
	return 0;
}

void ootx_reset_buffer(ootx_decoder_context *ctx) {
	ctx->buf_offset = 0;
	ctx->buffer[0] = 0;
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

	*current_byte <<= 1;
//	*current_byte |= (0x01 & dbit);
	*current_byte |= dbit;

	++(ctx->bits_written);
	if (ctx->bits_written>7) {
		ctx->bits_written=0;
//		printf("%d\n", *current_byte);
		ootx_inc_buffer_offset(ctx);
	}
}

uint8_t ootx_process_bit(ootx_decoder_context *ctx, uint32_t length) {
	uint8_t dbit = ootx_decode_bit(length);
	ootx_pump_bit( ctx, dbit );
	return dbit;
}

void ootx_pump_bit(ootx_decoder_context *ctx, uint8_t dbit) {
//	uint8_t dbit = ootx_decode_bit(length);
	++(ctx->bits_processed);

	if ( ootx_detect_preamble(ctx, dbit) ) {
		/*	data stream can start over at any time so we must
			always look for preamble bits */
		ootx_error(ctx, "Preamble found");
		ootx_reset_buffer(ctx);
		ctx->bits_processed = 0;
		ctx->found_preamble = 1;
	}
	else if(ctx->bits_processed>16) {
		//every 17th bit needs to be dropped (sync bit)
//		printf("drop %d\n", dbit);
		if( !dbit )
		{
			// printf("Bad sync bit\n");
			if (ctx->ignore_sync_bit_error == 0) {
				ootx_error(ctx, "OOTX Decoder: Bad sync bit");
				ootx_reset_buffer(ctx);
			} else {
				ootx_error(ctx, "OOTX Decoder: Ignoring bad sync bit");
			}
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

		uint16_t padded_length = *(ctx->payload_size);
		padded_length += (padded_length&0x01); //extra null byte if odd

/*		int k;
		printf( ":" );
		for( k = 0; k < 36; k++ )
		{
			printf( "%02x ", ctx->buffer[k] );
		}
		printf( "\n" );*/

		if (ctx->buf_offset >= (padded_length+6)) {
			/*	once we have a complete ootx packet, send it out in the callback */
			ootx_packet op;

			op.length = *(ctx->payload_size);
			op.data = ctx->buffer+2;
			memcpy(&op.crc32, op.data + padded_length, sizeof(uint32_t));

			uint32_t crc = crc32( 0L, 0 /*Z_NULL*/, 0 );
			crc = crc32( crc, op.data,op.length);

			if (crc != op.crc32) {
				if (ootx_bad_crc_clbk != NULL) ootx_bad_crc_clbk(ctx, &op,crc);
			}
			else if (ootx_packet_clbk != NULL) {
				ootx_packet_clbk(ctx,&op);
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

/* simply doing:
float f = 0;
uint32_t *ftmp = (uint32_t*)&f; //use the allocated floating point memory
This can cause problem when strict aliasing (-O2) is used.
Reads and writes to f and ftmp would be considered independent and could be 
be reordered by the compiler. A union solves that problem.
*/
union iFloat {
	uint32_t i;
	float f;
};

#ifndef _MSC_VER
struct __attribute__((__packed__)) unaligned_u16_t {
	uint16_t v;
};
#else
struct unaligned_u16_t {
	uint16_t v;
};
#endif

float _half_to_float(uint8_t* data) {
	uint16_t x = ((struct unaligned_u16_t*)data)->v;
	union iFloat fnum;
	fnum.f = 0;

	//sign
	fnum.i = (x & 0x8000)<<16;

	if ((x & 0x7FFF) == 0) return fnum.f; //signed zero

	if ((x & 0x7c00) == 0) {
		//denormalized
		x = (x&0x3ff)<<1; //only mantissa, advance intrinsic bit forward
		uint8_t e = 0;
		//shift until intrinsic bit of mantissa overflows into exponent
		//increment exponent each time
		while ((x&0x0400) == 0) {
			x<<=1;
			e++;
		}
		fnum.i |= ((uint32_t)(112-e))<<23; //bias exponent to 127, half floats are biased 15 so only need to go 112 more.
		fnum.i |= ((uint32_t)(x&0x3ff))<<13; //insert mantissa
		return fnum.f;
	}

	if((x&0x7c00) == 0x7c00) {
		//for infinity, fraction is 0
		//for NaN, fraction is anything non zero
		//we could just copy in bits and not shift, but the mantissa of a NaN can have meaning
		fnum.i |= 0x7f800000 | ((uint32_t)(x & 0x3ff))<<13;
		return fnum.f;
	}

	fnum.i |= ((((uint32_t)(x & 0x7fff)) + 0x1c000u) << 13);

	return fnum.f;
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

	lhi->fw_version = *(uint16_t*)get_ptr(data,sizeof(uint16_t),&idx);
	lhi->id = *(uint32_t*)get_ptr(data,sizeof(uint32_t),&idx);
	lhi->fcal_0_phase = _half_to_float( get_ptr(data,sizeof(uint16_t),&idx) );
	lhi->fcal_1_phase = _half_to_float( get_ptr(data,sizeof(uint16_t),&idx) );
	lhi->fcal_0_tilt = _half_to_float( get_ptr(data,sizeof(uint16_t),&idx) );
	lhi->fcal_1_tilt = _half_to_float( get_ptr(data,sizeof(uint16_t),&idx) );
	lhi->sys_unlock_count = *get_ptr(data,sizeof(uint8_t),&idx);
	lhi->hw_version = *get_ptr(data,sizeof(uint8_t),&idx);
	lhi->fcal_0_curve = _half_to_float( get_ptr(data,sizeof(uint16_t),&idx) );
	lhi->fcal_1_curve = _half_to_float( get_ptr(data,sizeof(uint16_t),&idx) );
	lhi->accel_dir_x = *(int8_t*)get_ptr(data,sizeof(uint8_t),&idx);
	lhi->accel_dir_y = *(int8_t*)get_ptr(data,sizeof(uint8_t),&idx);
	lhi->accel_dir_z = *(int8_t*)get_ptr(data,sizeof(uint8_t),&idx);
	lhi->fcal_0_gibphase = _half_to_float( get_ptr(data,sizeof(uint16_t),&idx) );
	lhi->fcal_1_gibphase = _half_to_float( get_ptr(data,sizeof(uint16_t),&idx) );
	lhi->fcal_0_gibmag = _half_to_float( get_ptr(data,sizeof(uint16_t),&idx) );
	lhi->fcal_1_gibmag = _half_to_float( get_ptr(data,sizeof(uint16_t),&idx) );
	lhi->mode_current = *get_ptr(data,sizeof(uint8_t),&idx);
	lhi->sys_faults = *get_ptr(data,sizeof(uint8_t),&idx);

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
