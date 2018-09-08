// (C) 2017 <>< C. N. Lohr, Under MIT/x11 License.
//
// See notice in survive_driverman.h
//

#include "survive_internal.h"
#include <stdio.h>
#include <string.h>

static void *Drivers[MAX_DRIVERS];
static const char *DriverNames[MAX_DRIVERS];
static int NrDrivers;

void RegisterDriver(const char *element, void *data) {
	Drivers[NrDrivers] = data;
	DriverNames[NrDrivers] = element;
	NrDrivers++;
}

void *GetDriver(const char *element) {
	int i;

	if (element == 0)
		return 0;

	for (i = 0; i < NrDrivers; i++) {
		if (strcmp(element, DriverNames[i]) == 0)
			return Drivers[i];
	}
	return 0;
}

void *GetDriverWithPrefix(const char *prefix, const char *name) {
	const char *DriverName = 0;
	const char *picked = 0;
	int i = 0;
	void *func = 0;
	int prefixLen = strlen(prefix);

	while ((DriverName = GetDriverNameMatching(prefix, i++))) {
		void *p = GetDriver(DriverName);

		bool match = strcmp(DriverName, name) == 0 || (strcmp(DriverName + prefixLen, name) == 0);
		if (match) {
			return p;
		}
	}

	return 0;
}

const char *GetDriverNameMatching(const char *prefix, int place) {
	int i;
	int prefixlen = (int)strlen(prefix);

	for (i = 0; i < NrDrivers; i++) {
		if (strncmp(prefix, DriverNames[i], prefixlen) == 0)
			if (0 == (place--))
				return DriverNames[i];
	}
	return 0;
}

void ListDrivers() {
	int i;
	printf("Drivers (%d/%d):\n", NrDrivers, MAX_DRIVERS);
	for (i = 0; i < NrDrivers; i++) {
		printf(" %s\n", DriverNames[i]);
	}
}
