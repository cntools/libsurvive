#include "stdint.h"

typedef uint32_t lfsr_poly_t;
typedef uint32_t lfsr_state_t;

static inline uint8_t popcnt(uint32_t x) {
	int c;
	for (c = 0; x != 0; x >>= 1u)
		if (x & 1u)
			c++;
	return c;
}

static inline uint32_t reverse32(uint32_t v) {
	uint32_t rtn = 0;
	for (int i = 0; i < 32; i++) {
		rtn = rtn << 1;
		rtn |= v & 1;
		v = v >> 1;
	}
	return rtn;
}

uint32_t lfsr_period(lfsr_poly_t p);
uint32_t lfsr_find(lfsr_poly_t p, lfsr_state_t start, lfsr_state_t end);
uint32_t lfsr_find_with_mask(lfsr_poly_t p, lfsr_state_t start, lfsr_state_t state, uint32_t mask);

lfsr_poly_t lsfr_mirror_poly(lfsr_poly_t poly);
lfsr_state_t lsfr_iterate(lfsr_state_t state, lfsr_poly_t poly, uint32_t cnt);
lfsr_state_t lsfr_iterate_rev(lfsr_state_t state, lfsr_poly_t poly, uint32_t cnt);

uint8_t lfsr_order(lfsr_poly_t v);

struct lfsr_lookup_t;
struct lfsr_lookup_t *lfsr_lookup_ctor(lfsr_poly_t p);
uint32_t lfsr_lookup_query(struct lfsr_lookup_t *lookup, uint32_t q);
