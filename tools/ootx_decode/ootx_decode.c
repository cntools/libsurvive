// (C) 2016 Joshua Allen, MIT/x11 License.
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
//	packet->data[packet->length] = 0;
//	printf("%d %s 0x%X\n", packet->length, packet->data, packet->crc32);
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
			uint8_t bit = 0x01;

			if (current_lighthouse==0) bit &= ootx_pump_greatest_bit(c_ctx);

			uint16_t s = *(c_ctx->payload_size);
			uint16_t fwv = *(c_ctx->buffer+2);
			uint16_t pv = 0x3f & fwv; //protocol version
			fwv>>=6; //firmware version
//			uint16_t ss = (s>>8) | (s<<8);

			//this will print after any messages from ootx_pump
//			if (c_ctx->found_preamble>0) printf("LH:%d s:%d 0x%x fw:%d pv:%d bo:%d bit:%d\t%s", current_lighthouse, s, s, fwv, pv, c_ctx->buf_offset, bit, line);

			//change to newly found lighthouse
			current_lighthouse = lh;
			c_ctx = ctx+current_lighthouse;
		}

		if (ticks>2000 && current_lighthouse==0) {
			//only work with master lighthouse for now
			ootx_log_bit(c_ctx, ticks);
		}

		if (lh == -1) {
//			printf("%d %d %d\n", ticks, delta, current_lighthouse);
//			ootx_process_bit(ctx+current_lighthouse, ticks);

		}
//			printf("%d %d %d\n", ticks, delta, current_lighthouse);
//			ootx_process_bit(ctx+current_lighthouse, ticks);

			//we would expect a length of 40 bytes
//			printf("%d %d %d\t%s\n", current_lighthouse, *(ctx->payload_size), ctx->found_preamble, line);
//		}
/*
		if (current_lighthouse >= 0) {
			ootx_process_bit(ctx+current_lighthouse, ticks);
		}

*/
	}	
}

int main(int argc, char* argv[])
{
	ootx_init_decoder_context(ctx);
	ootx_init_decoder_context(ctx+1);

	raw_test();
//	hello_world_test();

	return 0;
}