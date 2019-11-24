#include "lfsr.h"
#include "survive.h"

SURVIVE_EXPORT survive_channel survive_decipher_channel(const uint32_t *sample, const uint32_t *mask,
														const uint32_t *times, uint32_t *output, size_t count);