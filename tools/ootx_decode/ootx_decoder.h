// (C) 2017 Joshua Allen, MIT/x11 License.
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

	uint8_t bit_count[2];
} ootx_decoder_context;


typedef float float16;

typedef struct {
	uint16_t fw_version;//Firmware version (bit 15..6), protocol version (bit 5..0)
	uint32_t id; //Unique identifier of the base station
	float16 fcal_0_phase; //"phase" for rotor 0
	float16 fcal_1_phase; //"phase" for rotor 1
	float16 fcal_0_tilt; //"tilt" for rotor 0
	float16 fcal_1_tilt; //"tilt" for rotor 1
	uint8_t sys_unlock_count; //Lowest 8 bits of the rotor desynchronization counter
	uint8_t hw_version; //Hardware version
	float16 fcal_0_curve; //"curve" for rotor 0
	float16 fcal_1_curve; //"curve" for rotor 1
	int8_t accel_dir_x; //"orientation vector"
	int8_t accel_dir_y; //"orientation vector"
	int8_t accel_dir_z; //"orientation vector"
	float16 fcal_0_gibphase; //"gibbous phase" for rotor 0 (normalized angle)
	float16 fcal_1_gibphase; //"gibbous phase" for rotor 1 (normalized angle)
	float16 fcal_0_gibmag; //"gibbous magnitude" for rotor 0
	float16 fcal_1_gibmag; //"gibbous magnitude" for rotor 1
	uint8_t mode_current; //Currently selected mode (default: 0=A, 1=B, 2=C)
	uint8_t sys_faults; //"fault detect flags" (should be 0)
} lighthouse_info_v6;

void init_lighthouse_info_v6(lighthouse_info_v6* lhi, uint8_t* data);
void print_lighthouse_info_v6(lighthouse_info_v6* lhi);

//void ootx_init_buffer();
void ootx_process_bit(ootx_decoder_context *ctx, uint32_t length);
void ootx_init_decoder_context(ootx_decoder_context *ctx);
int8_t ootx_decode_lighthouse_number(uint8_t last_num, uint32_t ticks, int32_t delta);

void ootx_accumulate_bit(ootx_decoder_context *ctx, uint8_t bit);
uint8_t ootx_pump_greatest_bit(ootx_decoder_context *ctx);

uint8_t ootx_decode_bit(uint32_t length);

extern void (*ootx_packet_clbk)(ootx_packet* packet);
extern void (*ootx_bad_crc_clbk)(ootx_packet* packet, uint32_t crc);

#endif