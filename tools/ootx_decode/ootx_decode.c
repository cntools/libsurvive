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

void my_test(ootx_decoder_context *ctx, ootx_packet* packet) {
	packet->data[packet->length] = 0;
	printf("%d %s 0x%X\n", packet->length, packet->data, packet->crc32);
}

void my_test2(ootx_decoder_context *ctx, ootx_packet* packet) {
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

void bad_crc(ootx_decoder_context *ctx, ootx_packet* packet, uint32_t crc) {
	printf("CRC mismatch\n");

	printf("r:");
	print_crc32(packet->crc32);

	printf("c:");
	print_crc32(crc);
	write_to_file(packet->data,packet->length);
}

ootx_decoder_context ctx[2];

void cnlohr_code_test() {
	ootx_packet_clbk = my_test2;
	ootx_bad_crc_clbk = bad_crc;

	char* line = NULL;
	size_t line_len = 0;
	char trash[100] = "";
	int8_t lh_id = 0x00;
	uint32_t ticks = 0x00;
	int32_t delta = 0x00;
	uint8_t acode = 0x00;

	ootx_decoder_context *c_ctx = ctx;

	while (getline(&line,&line_len,stdin)>0) {
			//R Y HMD -1575410734 -2 7 19714 6485
			sscanf(line,"%s %s %s %s %hhd %hhd %d %d",
			trash,
			trash,
			trash,
			trash,
			&lh_id,
			&acode,
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

//			ootx_process_bit(c_ctx, ticks);
			ootx_pump_bit( c_ctx, (acode&0x02)>>1 );
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

	ootx_free_decoder_context(ctx);
	ootx_free_decoder_context(ctx+1);

	return 0;
}
