#include "survive_reproject.h"
#include "test_case.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "../driver_vive.h"

TEST(ViveDriver, TestWatchmanParsing) {

	{
		uint8_t readdata[] = {0xff, 0x09, 0x00, 0x04, 0x00, 0x38, 0xb8, 0xec, 0xe4, 0x9f};

		LightcapElement les[10];
		int cnt = parse_watchman_lightcap(0, "WW0", 0, 0, readdata, sizeof(readdata), les, 10);
	}

	{
		uint8_t readdata[] = {0x00, 0x6f, 0xfd, 0x83, 0xff};

		LightcapElement les[10];
		int cnt = parse_watchman_lightcap(0, "WW0", 224, 3761897504, readdata, sizeof(readdata), les, 10);
	}

	{
		uint8_t readdata[] = {0x88, 0x81, 0xa1, 0x10, 0x00, 0xd3, 0x06, 0x93, 0x03, 0xa3, 0x06, 0xf3, 0x06,
							  0x83, 0x01, 0xd6, 0x06, 0xe4, 0xa8, 0x0c, 0xd9, 0x07, 0xc1, 0x92, 0xd2};
		LightcapElement les[10];
		int cnt = parse_watchman_lightcap(0, "WW0", 224, 3761897504, readdata, sizeof(readdata), les, 10);
	}
	return 0;
}