// (C) 2017 Joshua Allen, MIT/x11 License.
//
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.

/* ootx data decoder test*/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "ootx_decoder.h"

//char* fmt_str = "L Y HMD %d 5 1 206230 %d\n";
//extern std::istream cin;

void my_test(ootx_packet* packet) {
	packet->data[packet->length] = 0;
	printf("%d %s 0x%X\n", packet->length, packet->data, packet->crc32);
}

void my_test2(ootx_packet* packet) {
	printf("completed ootx packet\n");

	lighthouse_info_v6 lhi;
	init_lighthouse_info_v6(&lhi,packet->data);
	print_lighthouse_info_v6(&lhi);
//	packet->data[packet->length] = 0;
//	printf("%d %s 0x%X\n", packet->length, packet->data, packet->crc32);
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

void bad_crc(ootx_packet* packet, uint32_t crc) {
	printf("CRC mismatch\n");

	printf("r:");
	print_crc32(packet->crc32);

	printf("c:");
	print_crc32(crc);
	write_to_file(packet->data,packet->length);
}

ootx_decoder_context ctx[2];

void hello_world_test() {
//	ootx_init_buffer();
	ootx_packet_clbk = my_test;

	char* line = NULL;
	size_t line_len = 0;
	char trash[100] = "";
	uint32_t ticks = 0x00;

	while (getline(&line,&line_len,stdin)>0) {
//		printf("%s\n", line);
		sscanf(line,"%s %s %s %s %s %s %s %d",
			trash,
			trash,
			trash,
			trash,
			trash,
			trash,
			trash,
			&ticks);
//		printf("%d\n", ticks);

		ootx_process_bit(ctx, ticks);
	}
}

void raw_test() {
	ootx_packet_clbk = my_test2;
	ootx_bad_crc_clbk = bad_crc;

	char* line = NULL;
	size_t line_len = 0;
	char trash[100] = "";
	int32_t atime = 0x00;
	uint32_t ticks = 0x00;
	uint32_t delta = 0x00;

	int8_t current_lighthouse = 0;
	ootx_decoder_context *c_ctx = ctx;

	while (getline(&line,&line_len,stdin)>0) {
//		printf("%s\n", line);

		//HMD 20 0 5881 765645903 -5
		sscanf(line,"%s %s %s %d %d %d",
			trash,
			trash,
			trash,
			&ticks,
			&atime,
			&delta);
//		printf("%d\n", ticks);

		int8_t lh = ootx_decode_lighthouse_number(current_lighthouse, ticks, delta);
//		printf("lh:%d %s\n", lh, line);
//		if (lh>0) continue;

		if (lh > -1) {
			//pump last bit
//			printf("LH:%d ", current_lighthouse);
			uint8_t bit = 0x01; //bit for debugging purposes

//			if (current_lighthouse==1) bit &= ootx_pump_greatest_bit(c_ctx);
			bit &= ootx_pump_greatest_bit(c_ctx);

/*
			uint16_t s = *(c_ctx->payload_size);
			uint16_t fwv = *(c_ctx->buffer+2);
			uint16_t pv = 0x3f & fwv; //protocol version
			fwv>>=6; //firmware version

			//this will print after any messages from ootx_pump
			if (c_ctx->found_preamble>0) printf("LH:%d s:%d 0x%x fw:%d pv:%d bo:%d bit:%d\t%s", current_lighthouse, s, s, fwv, pv, c_ctx->buf_offset, bit, line);
*/
			//change to newly found lighthouse
			current_lighthouse = lh;
			c_ctx = ctx+current_lighthouse;
		}

//		if (ticks>2000 && current_lighthouse==1) {
		if (ticks>2000) {
				ootx_accumulate_bit(c_ctx, ootx_decode_bit(ticks) );
		}
	}	
}

void acode_test() {
	ootx_packet_clbk = my_test2;
	ootx_bad_crc_clbk = bad_crc;

	char* line = NULL;
	size_t line_len = 0;
	char trash[100] = "";
	int32_t atime = 0x00;
//	uint32_t ticks = 0x00;
//	uint32_t delta = 0x00;
	uint8_t acode = 0x00;
	char lighthouse_code = '\0';

	int8_t current_lighthouse = 0;
	ootx_decoder_context *c_ctx = ctx;

	while (getline(&line,&line_len,stdin)>0) {
		//L X HMD -1842671365 18 0 175393 222
		sscanf(line,"%c %s %s %d %s %hhu %s %s",
			&lighthouse_code,
			trash,
			trash,
			&atime,
			trash,
			&acode,
			trash,
			trash);

		int8_t lh = lighthouse_code=='R'?0:1;
//		printf("LH:%d bit:%d %s\n", lh, (acode & 0x02) >> 1,line);

		if (lh != current_lighthouse) {
			//pump last bit
			uint8_t bit = 0x01;

			if (current_lighthouse==0) bit &= ootx_pump_greatest_bit(c_ctx);
//			ootx_pump_greatest_bit(c_ctx);

			uint16_t s = *(c_ctx->payload_size);
			uint16_t fwv = *(c_ctx->buffer+2);
			uint16_t pv = 0x3f & fwv; //protocol version
			fwv>>=6; //firmware version

			//this will print after any messages from ootx_pump
			if (c_ctx->found_preamble>0) printf("LH:%d s:%d 0x%x fw:%d pv:%d bo:%d bit:%d\t%s", current_lighthouse, s, s, fwv, pv, c_ctx->buf_offset, bit, line);

			//change to newly found lighthouse
			current_lighthouse = lh;
			c_ctx = ctx+current_lighthouse;
		}

//		if (current_lighthouse==0) {
			ootx_accumulate_bit(c_ctx, (acode & 0x02) >> 1);
//		}
	}
}

void cnlohr_code_test() {
	ootx_packet_clbk = my_test2;
	ootx_bad_crc_clbk = bad_crc;

	char* line = NULL;
	size_t line_len = 0;
	char trash[100] = "";
//	int32_t atime = 0x00;
	int8_t lh_id = 0x00;
	uint32_t ticks = 0x00;
	int32_t delta = 0x00;
//	uint8_t acode = 0x00;
//	char lighthouse_code = '\0';

//	int8_t current_lighthouse = 0;
	ootx_decoder_context *c_ctx = ctx;

	while (getline(&line,&line_len,stdin)>0) {
			//R Y HMD -1575410734 -2 7 19714 6485
			sscanf(line,"%s %s %s %s %hhd %s %d %d",
			trash,
			trash,
			trash,
			trash,
			&lh_id,
			trash, //sensor id?
			&delta,
			&ticks);

//		int8_t lh = lighthouse_code=='R'?0:1;
//		printf("LH:%d %s\n", lh_id, line);
		int8_t lh = (lh_id*-1)-1;
		if (lh_id < 0) {
//			uint8_t bit = 0x01; //bit for debugging purposes

			//change to newly found lighthouse
			c_ctx = ctx+lh;

//			uint8_t dbit = ootx_decode_bit(ticks);
//			printf("LH:%d ticks:%d bit:%X %s", lh, ticks, dbit, line);

			ootx_process_bit(c_ctx, ticks);

/*
			uint16_t s = *(c_ctx->payload_size);
			uint16_t fwv = *(c_ctx->buffer+2);
			uint16_t pv = 0x3f & fwv; //protocol version
			fwv>>=6; //firmware version

			//this will print after any messages from ootx_pump
//			if (c_ctx->found_preamble>0) printf("LH:%d s:%d 0x%x fw:%d pv:%d bo:%d bit:%d\t%s", current_lighthouse, s, s, fwv, pv, c_ctx->buf_offset, bit, line);
*/	
		}
	}
}

int main(int argc, char* argv[])
{
	ootx_init_decoder_context(ctx);
	ootx_init_decoder_context(ctx+1);

	cnlohr_code_test();
//	raw_test();
//	acode_test();
//	hello_world_test();

	ootx_free_decoder_context(ctx);
	ootx_free_decoder_context(ctx+1);

	return 0;
}