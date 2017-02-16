// (C) 2016 Joshua Allen, MIT/x11 License.
//
//All MIT/x11 Licensed Code in this file may be relicensed freely under the GPL or LGPL licenses.
//crc32.c under liberal license.
//
// You should only include this file if you are going to include the calibration subsystem of libsurvive.

#ifndef CRC32_H
#define CRC32_H

#include <stddef.h>
#include <stdint.h>

uint32_t crc32(uint32_t crc, uint8_t *buf, size_t size);

#endif

