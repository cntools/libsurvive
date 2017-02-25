// (C) 2017 <>< Joshua Allen, Under MIT/x11 License.


#ifndef _SURVIVE_CONFIG_H
#define _SURVIVE_CONFIG_H

#include "survive_internal.h"

typedef enum {
	CONFIG_UNKNOWN = 0,
	CONFIG_FLOAT = 1,
	CONFIG_UINT32 = 2,
	CONFIG_STRING = 3
} cval_type;
/*
typedef union {
		uint32_t i;
		float f;
	} Numeric;
*/
typedef struct {
	char *tag;
	cval_type type;
	union {
		uint32_t i;
		float f;
	} numeric;
	char *str;
} config_val;


void config_open(const char* path, const char* mode);
void config_close();
void config_write_lighthouse(struct BaseStationData* bsd, uint8_t length);

#endif