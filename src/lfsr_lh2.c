#include "lfsr_lh2.h"
#ifndef _MSC_VER
#include "alloca.h"
#define clz(x) __builtin_clz(x)
#else
#include <intrin.h>
uint32_t inline clz(uint32_t value) {
	uint32_t leading_zero = 0;

	if (_BitScanReverse(&leading_zero, value)) {
		return 31 - leading_zero;
	} else {
		// This is undefined, I better choose 32 than 0
		return 32;
	}
}
#endif
#include "stdio.h"
#include "string.h"
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h>
#endif

lfsr_poly_t poly_pairs[32] = {
	// x^17 + x^13 + x^12 + x^10 + x^7 + x^4 + x^2 + x^1 + 1
	//   0123456789ABCDEF01
	// 0b11101001001011000
	0x0001D258,
	// x^17 + x^14 +  x^7 +  x^6 + x^5 + x^4 + x^3 + x^2 + 1
	//  0123456789ABCDEF01
	// 0b10111111000000100
	0x00017E04,
	0x0001FF6B,
	0x00013F67,
	0x0001B9EE,
	0x000198D1,
	0x000178C7,
	0x00018A55,
	0x00015777,
	0x0001D911,
	0x00015769,
	0x0001991F,
	0x00012BD0,
	0x0001CF73,
	0x0001365D,
	0x000197F5,
	0x000194A0,
	0x0001B279,
	0x00013A34,
	0x0001AE41,
	0x000180D4,
	0x00017891,
	0x00012E64,
	0x00017C72,
	0x00019C6D,
	0x00013F32,
	0x0001AE14,
	0x00014E76,
	0x00013C97,
	0x000130CB,
	0x00013750,
	0x0001CB8D,
};

struct lfsr_lookup_t *poly_pair_lookups[32] = {0};
static void init_lookups() {
	if (poly_pair_lookups[0] == 0) {
		for (int i = 0; i < 32; i++)
			poly_pair_lookups[i] = lfsr_lookup_ctor(poly_pairs[i]);
	}
}

static uint32_t find_possible_polys(uint32_t sample, uint32_t mask, uint32_t *timings, uint32_t *reconstructed_sample) {
	uint8_t offset = 255;
	for (uint8_t i = 0; i < 15; i++) {
		if (((mask >> (15 - i)) & 0x1ffff) == 0x1ffff) {
			offset = i;
			break;
		}
	}

	uint32_t rtn = 0xFFFFFFFF;
	if (offset == 255) {
		return rtn;
	}

	for (int i = 0; i < 32; i++) {
		uint32_t state = (sample >> (15u - offset));

		uint32_t start_state = lsfr_iterate_rev(state, poly_pairs[i], offset);
		uint32_t final_state = lsfr_iterate(start_state, poly_pairs[i], 15);

		uint32_t error_bits = (final_state ^ sample) & mask;
		uint32_t error = popcnt(error_bits);

		if (error > 0) {
			if (((i / 2) == 6) || ((i / 2) == 15))
				fprintf(stderr, "Error for %d was %d %x %x %x\n", i, error, final_state & mask, sample & mask, mask);
			rtn ^= (1 << i);
		} else {
			timings[i] = lfsr_lookup_query(poly_pair_lookups[i], state) - offset;
			reconstructed_sample[i] = final_state;
			fprintf(stderr, "Timing for %d was %u\n", i, timings[i]);
		}
	}

	return rtn;
}

survive_channel survive_decipher_channel(const uint32_t *sample, const uint32_t *mask, const uint32_t *times,
										 uint32_t *output, size_t count) {
	init_lookups();
	uint32_t possible_polys = 0xFFFFFFFF;
	uint32_t *timings = alloca(32 * sizeof(uint32_t) * count);
	uint32_t *recon_samples = alloca(32 * sizeof(uint32_t) * count);
	size_t known_solves[32] = {0};

	memset(timings, 0, 32 * sizeof(uint32_t) * count);
	memset(recon_samples, 0, 32 * sizeof(uint32_t) * count);

	for (int i = 0; i < count; i++) {
		uint32_t new_polys = find_possible_polys(sample[i], mask[i], timings + 32 * i, recon_samples + 32 * i);
		possible_polys &= new_polys;

		uint8_t idx = 0;
		while (new_polys) {
			if (new_polys & 1 && recon_samples[idx + i * 32] != 0) {
				known_solves[idx] = i + 1;
			}
			idx++;
			new_polys = new_polys >> 1;
		}
	}

	survive_channel channel = 255;
	if (possible_polys == 0)
		return channel;

	for (int i = 0; i < count; i++) {
		for (int j = 0; j < 32; j++) {
			if ((possible_polys & (1 << j)) == 0)
				continue;
			if (recon_samples[32 * i + j] != 0)
				continue;
			size_t gi = known_solves[j];
			if (gi == 0)
				continue;
			gi--;

			survive_timecode it = times[i];
			survive_timecode igt = times[gi];
			int32_t diff = it - igt;

			for (int o = -2; o <= 2; o++) {
				int32_t o_diff = diff + o * 8;
				uint32_t predicted_sample =
					o_diff > 0 ? lsfr_iterate(recon_samples[32 * gi + j], poly_pairs[j], (o_diff + 4) / 8)
							   : lsfr_iterate_rev(recon_samples[32 * gi + j], poly_pairs[j], (-o_diff + 4) / 8);

				uint32_t error_bits = (predicted_sample ^ sample[i]) & mask[i];
				uint32_t error = popcnt(error_bits);

				if (error <= 1) {
					recon_samples[32 * i + j] = predicted_sample;
					timings[32 * i + j] = timings[32 * gi + j] + (o_diff + 4) / 8;

					if (error == 0)
						break;
				} else {
					fprintf(stderr, "Err %d for %d (%d) %08x vs %08x %08x %08x\n", error, o, j, predicted_sample,
							sample[i], mask[i], error_bits);
				}
			}

			if (recon_samples[32 * i + j] == 0) {
				possible_polys ^= (1u << j);
				fprintf(stderr, "Eliminated %d\n", j);
			}
		}
	}

	if (popcnt(possible_polys) == 1) {
		channel = 31 - clz(possible_polys);
	}

	if (channel != 255) {
		for (int i = 0; i < count; i++) {
			output[i] = (timings + (32 * i))[channel];
		}
	}

	return channel;
}
