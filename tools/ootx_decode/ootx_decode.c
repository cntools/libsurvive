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

int main(int argc, char* argv[])
{
	ootx_decoder_context ctx;
	ootx_init_decoder_context(&ctx);
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

		ootx_process_bit(&ctx, ticks);
	}

	return 0;
}