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

		if (lh > -1) {
			//change to newly found lighthouse
			current_lighthouse = lh;
//			printf("%d %d %d\n", ticks, delta, current_lighthouse);
			ootx_process_bit(ctx+current_lighthouse, ticks);
			printf("%d %d %d\n", current_lighthouse, *(ctx->payload_size), ctx->found_preamble);
		}
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

	return 0;
}