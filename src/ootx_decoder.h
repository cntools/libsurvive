// (C) 2017 Joshua Allen, MIT/x11 License.
//
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

#ifndef OOTX_DECODER_H
#define OOTX_DECODER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
	uint16_t length;
	uint8_t* data;
	uint32_t crc32;
} ootx_packet;

#define OOTX_MAX_BUFF_SIZE 64

typedef struct ootx_decoder_context {
	uint8_t buffer[OOTX_MAX_BUFF_SIZE];

	bool bits_post_preamble[64 * 8];

	uint16_t buf_offset;
	uint8_t bits_written;
	uint16_t* payload_size;

	uint32_t preamble;
	uint8_t bits_processed;
	uint16_t offset;
	uint16_t total_offset;
	uint8_t found_preamble;

	int ignore_sync_bit_error;
	void * user;
	int user1;

	struct {
		uint32_t bits_seen;
		uint32_t bad_sync_bits;
		uint32_t bad_crcs;
		uint32_t packets_found;
		uint32_t used_bytes;
		uint32_t package_bits;
		uint32_t guess_bits;
		double started_s;
	} stats;

	void (*ootx_error_clbk)(struct ootx_decoder_context *ctx, const char *msg);
	void (*ootx_packet_clbk)(struct ootx_decoder_context *ctx, ootx_packet *packet);
	void (*ootx_bad_crc_clbk)(struct ootx_decoder_context *ctx, ootx_packet *packet, uint32_t crc);
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

typedef struct {
	uint16_t fw_version;	   // Firmware version (bit 15..6), protocol version (bit 5..0)
	uint32_t id;			   // Unique identifier of the base station
	float16 fcal_phase[2];	 //"phase" for rotor 0
	float16 fcal_tilt[2];	  //"tilt" for rotor 0
	uint8_t sys_unlock_count; // Think this might be FPGA code version?
	uint8_t ootx_model;		   // 'OOTX model'?
	float16 fcal_curve[2];	 //"curve" for rotor 0
	int8_t accel_dir[3];	   //"orientation vector"
	float16 fcal_gibphase[2];  //"gibbous phase" for rotor 0 (normalized angle)
	float16 fcal_gibmag[2];	//"gibbous magnitude" for rotor 0
	uint8_t mode_current;	  // Some bit flag with the mode attached?
	uint8_t sys_faults;		   //
	float16 fcal_ogeephase[2]; //"gibbous phase" for rotor 0 (normalized angle)
	float16 fcal_ogeemag[2];   //"gibbous magnitude" for rotor 0
	uint16_t nonce;			   //"fault detect flags" (should be 0)
} lighthouse_info_v15;

void init_lighthouse_info_v15(lighthouse_info_v15 *lhi, uint8_t *data);

void init_lighthouse_info_v6(lighthouse_info_v6* lhi, uint8_t* data);
void print_lighthouse_info_v6(lighthouse_info_v6* lhi);

void ootx_init_decoder_context(ootx_decoder_context *ctx, float time);
void ootx_free_decoder_context(ootx_decoder_context *ctx);

void ootx_pump_bit(ootx_decoder_context *ctx, int8_t dbit);

uint8_t ootx_decode_bit(uint32_t length);

#endif
