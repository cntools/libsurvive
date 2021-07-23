// (C) 2017 Joshua Allen, MIT/x11 License.
//
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

/* ootx data decoder */

#include "ootx_decoder.h"
#include "string.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#include <os_generic.h>

#ifdef NOZLIB
#include "crc32.h"
#else
#include <zlib.h>
#endif

//char* fmt_str = "L Y HMD %d 5 1 206230 %d\n";

void ootx_pump_bit(ootx_decoder_context *ctx, int8_t dbit);

void ootx_error(ootx_decoder_context *ctx, const char *msg) {
	if (ctx->ootx_error_clbk)
		ctx->ootx_error_clbk(ctx, msg);
}

void ootx_init_decoder_context(ootx_decoder_context *ctx, float start) {
	ctx->buf_offset = 0;
	ctx->bits_written = 0;

	ctx->preamble = 0XFFFFFFFF;
	ctx->bits_processed = 0;
	ctx->found_preamble = 0;
	ctx->ignore_sync_bit_error = 0;

	ctx->stats.started_s = start;

	ctx->payload_size = (uint16_t*)ctx->buffer;
	*(ctx->payload_size) = 0;
}

void ootx_free_decoder_context(ootx_decoder_context *ctx) {
	ctx->payload_size = NULL;
}

static bool ootx_detect_preamble(ootx_decoder_context *ctx, int8_t dbit) {
	ctx->preamble <<= 1;
//	ctx->preamble |= (0x01 & dbit);
	if (dbit < 0) {
		// Tend to not trigger preamble if we already have one
		if (ctx->found_preamble)
			dbit = 1;
		else {
			// If we don't have a preamble; and we just saw a bunch of 0's, trigger a preamble.
			// if we dont have a bunch of 0's, inject a 0 and maybe it's part of the preamble.
			dbit = ((ctx->preamble & 0x0001ffff) == 0x00000000) ? 1 : 0;
		}
	}

	ctx->preamble |= dbit;
	if ((ctx->preamble & 0x0003ffff) == 0x00000001) return 1;
	return 0;
}

void ootx_reset_buffer(ootx_decoder_context *ctx) {
	ctx->buf_offset = 0;
	ctx->bits_written = 0;
	ctx->offset = 0;
	ctx->found_preamble = 0;
	*(ctx->payload_size) = 0;
}

void ootx_inc_buffer_offset(ootx_decoder_context *ctx) {
	++(ctx->buf_offset);

//	assert(ctx->buf_offset<MAX_BUFF_SIZE);

	/* the buffer is going to overflow, wrap the buffer and don't write more data until the preamble is found again */
	if (ctx->buf_offset >= OOTX_MAX_BUFF_SIZE) {
		ctx->buf_offset = 0;
		ctx->found_preamble = 0;
	}
}

static void ootx_write_to_buffer(ootx_decoder_context *ctx, int8_t dbit) {
	uint8_t *current_byte = ctx->buffer + ctx->buf_offset;

	// Purposefully leave the value as is if dbit is -1
	uint8_t mask = (1u << (7u - ctx->bits_written));
	if (dbit == 0) {
		*current_byte &= ~mask;
	} else if (dbit == 1) {
		*current_byte |= mask;
	} else {
		ctx->stats.guess_bits++;
	}
	ctx->stats.package_bits++;

	if (++ctx->bits_written > 7) {
		ctx->bits_written=0;
		ootx_inc_buffer_offset(ctx);
	}
}

void ootx_pump_bit(ootx_decoder_context *ctx, int8_t dbit) {
	/*if (dbit < 0) {
		if(ctx->found_preamble) {
			size_t bits_cnt = ctx->bits_written + ctx->buf_offset * 8;
			dbit = ctx->bits_processed == 16 ? 1 : ctx->bits_post_preamble[bits_cnt];
		} else {
			dbit = ((ctx->preamble & 0x0001ffff) == 0x00000000) ? 1 : ctx->bits_processed == 16;
		}
	}*/

	ctx->total_offset++;
	ctx->offset++;
	++(ctx->bits_processed);
	ctx->stats.bits_seen++;
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
		if (dbit == 0) {
			// printf("Bad sync bit\n");
			if (ctx->ignore_sync_bit_error == 0) {
				if (ctx->found_preamble) {
					ootx_error(ctx, "OOTX Decoder: Bad sync bit");
					ctx->stats.bad_sync_bits++;
				}
				ootx_reset_buffer(ctx);
			} else if (ctx->found_preamble) {
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

		if (ctx->buf_offset >= (padded_length+6)) {
			/*	once we have a complete ootx packet, send it out in the callback */
			ootx_packet op = {0};

			op.length = *(ctx->payload_size);
			op.data = ctx->buffer+2;
			memcpy(&op.crc32, op.data + padded_length, sizeof(uint32_t));

			uint32_t crc = crc32( 0L, 0 /*Z_NULL*/, 0 );
			crc = crc32( crc, op.data,op.length);

			if (crc != op.crc32) {
				if (ctx->ootx_bad_crc_clbk != NULL) {
					ctx->ootx_bad_crc_clbk(ctx, &op, crc);
				}
				ctx->stats.bad_crcs++;
			} else if (ctx->ootx_packet_clbk != NULL) {
				ctx->stats.packets_found++;
				ctx->stats.used_bytes += op.length;
				ctx->ootx_packet_clbk(ctx, &op);
			}

			ootx_reset_buffer(ctx);
		}
	}
}

static uint8_t *get_ptr(uint8_t *data, uint8_t bytes, uint16_t *idx) {
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

static float _half_to_float(uint8_t *data) {
	uint16_t x = ((struct unaligned_u16_t*)data)->v;
	union iFloat fnum;
	fnum.f = 0;

	//sign
	fnum.i = (((uint32_t)x & 0x8000) << 16);

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

void init_lighthouse_info_v15(lighthouse_info_v15 *lhi, uint8_t *data) {
#pragma pack(push, 1)
	typedef struct {
		uint16_t fw_version;		// Firmware version (bit 15..6), protocol version (bit 5..0)
		uint32_t id;				// Unique identifier of the base station
		uint16_t fcal_phase[2];		//"phase" for rotor 0
		uint16_t fcal_tilt[2];		//"tilt" for rotor 0
		uint8_t sys_unlock_count;	// Lowest 8 bits of the rotor desynchronization counter
		uint8_t ootx_model;			// 'OOTX model'?
		uint16_t fcal_curve[2];		//"curve" for rotor 0
		int8_t accel_dir[3];		//"orientation vector"
		uint16_t fcal_gibphase[2];  //"gibbous phase" for rotor 0 (normalized angle)
		uint16_t fcal_gibmag[2];	//"gibbous magnitude" for rotor 0
		uint8_t mode_current;		// Some bit flag with the mode attached?
		uint8_t sys_faults;			//
		uint16_t fcal_ogeephase[2]; //"gibbous phase" for rotor 0 (normalized angle)
		uint16_t fcal_ogeemag[2];   //"gibbous magnitude" for rotor 0
		uint16_t nonce;				//"fault detect flags" (should be 0)
	} lighthouse_info_v15_packed;
#pragma pack(pop)

	lighthouse_info_v15_packed *d = (lighthouse_info_v15_packed *)data;

	lhi->fw_version = d->fw_version;
	lhi->id = d->id;

	for (int i = 0; i < 2; i++) {
		lhi->fcal_phase[i] = _half_to_float((uint8_t *)&d->fcal_phase[i]);
		lhi->fcal_tilt[i] = _half_to_float((uint8_t *)&d->fcal_tilt[i]);
		lhi->fcal_curve[i] = _half_to_float((uint8_t *)&d->fcal_curve[i]);
		lhi->fcal_gibphase[i] = _half_to_float((uint8_t *)&d->fcal_gibphase[i]);
		lhi->fcal_gibmag[i] = _half_to_float((uint8_t *)&d->fcal_gibmag[i]);
		lhi->fcal_ogeemag[i] = _half_to_float((uint8_t *)&d->fcal_ogeemag[i]);
		lhi->fcal_ogeephase[i] = _half_to_float((uint8_t *)&d->fcal_ogeephase[i]);
	}

	lhi->sys_unlock_count = d->sys_unlock_count;
	lhi->ootx_model = d->ootx_model;

	for (int i = 0; i < 3; i++)
		lhi->accel_dir[i] = d->accel_dir[i];

	lhi->sys_faults = d->sys_faults;
	lhi->mode_current = d->mode_current;
	lhi->nonce = d->nonce;
}
