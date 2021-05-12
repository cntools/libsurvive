#include "lfsr.h"
#include <assert.h>
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h>
#endif
#include <stdbool.h>
#include <survive.h>

lfsr_state_t lsfr_iterate(lfsr_state_t state, lfsr_poly_t poly, uint32_t cnt) {
	for (int i = 0; i < cnt; i++) {
		uint16_t b = popcnt(state & poly) & 1u;
		state = (state << 1u) | b;
	}
	return state;
}

lfsr_state_t lsfr_iterate_rev(lfsr_state_t state, lfsr_poly_t poly, uint32_t cnt) {
	uint32_t orig_state = state;
	state = state & 0x1ffff;

	uint32_t order = lfsr_order(poly);
	for (int i = 0; i < cnt + (32 - order); i++) {
		uint16_t b = state & 1u;
		state = state >> 1u;
		uint16_t bn = popcnt(state & poly) & 1u;

		if (b != bn)
			state |= (1u << (order - 1));
	}

	state = lsfr_iterate(state, poly, (32 - order));

	uint32_t dorig_state = lsfr_iterate(state, poly, cnt);
	assert((dorig_state & 0x1ffff) == (orig_state & 0x1ffff));
	return state;
}

uint8_t lfsr_order(lfsr_poly_t v) {
	uint8_t rtn = 1;
	v >>= 1;
	while (v) {
		rtn++;
		v >>= 1;
	}
	return rtn;
}

uint32_t lfsr_find(lfsr_poly_t p, lfsr_state_t start, lfsr_state_t end) {
	uint32_t state = start;
	uint32_t period = lfsr_order(p);
	uint32_t mask = (1 << (period)) - 1;
	uint32_t cnt = 0;
	do {
		cnt++;
		state = lsfr_iterate(state, p, 1);
	} while ((end & mask) != (state & mask));
	return cnt;
}

uint32_t lfsr_period(lfsr_poly_t p) { return lfsr_find(p, 1, 1); }

lfsr_poly_t lsfr_mirror_poly(lfsr_poly_t p) {
	uint32_t order = lfsr_order(p);
	lfsr_poly_t rtn = 1 << (order - 1);
	for (uint8_t i = 0; i < order; i++) {
		if (p & (1u << (i - 1)))
			rtn |= 1u << (order - i - 1);
	}

	return rtn;
}

struct lfsr_lookup_t {
	uint32_t order;
	lfsr_poly_t p;
	uint32_t *table;
};

struct lfsr_lookup_t *lfsr_lookup_ctor(lfsr_poly_t p) {
	uint32_t order = lfsr_order(p);
	struct lfsr_lookup_t *lookup = SV_MALLOC(sizeof(struct lfsr_lookup_t));
	lookup->table = (uint32_t *)SV_CALLOC_N(1 << order, sizeof(uint32_t));
	lookup->order = order;
	uint32_t start = 1;
	uint32_t state = start;
	uint32_t mask = (1 << (order)) - 1;
	uint32_t cnt = 0;

	do {
		// if(cnt > order) {
		assert(lookup->table[state & mask] == 0);
		lookup->table[state & mask] = cnt;
		//}
		cnt++;
		state = lsfr_iterate(state, p, 1);
	} while ((start & mask) != (state & mask));

	return lookup;
}

uint32_t lfsr_lookup_query(struct lfsr_lookup_t *lookup, uint32_t q) {
	uint32_t mask = (1 << (lookup->order)) - 1;
	return lookup->table[q & mask];
}

uint32_t lfsr_find_with_mask(lfsr_poly_t p, lfsr_state_t start, lfsr_state_t state, uint32_t mask) {
	for (uint8_t j = 0; j < 16; j++) {
		if (((mask >> j) & 0x1FFFF) == 0x1FFFF) {
			return lfsr_find(p, start, state >> j) - j;
		}
	}
	return 0;
}
