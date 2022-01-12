#include "../survive_internal.h"
#include "test_case.h"
#include <string.h>

cstring logs;
int main(int argc, char **argv) {
	int i = 0;
	bool failed = false;

	const char *filter = argc > 1 ? argv[1] : "";

	const char *DriverName = 0;
	while ((DriverName = GetDriverNameMatching("Test", i++))) {
		if (strstr(DriverName, filter) == NULL) {
			continue;
		}
		TestCase dd = (TestCase)GetDriver(DriverName);
		DriverName += 4; // We just want to show the test name without prefix 'Test'

		// fprintf(stderr, "Running test %s (%d)\n", DriverName, i);
		int r = dd();
		if (r != 0) {
			if (logs.d && logs.d[0] != 0) {
				fprintf(stderr, "%s output:\n%s\n", DriverName, logs.d);
			}
			fprintf(stderr, "Test %s reports status %d\n", DriverName, r);
		}
		str_clear(&logs);
		failed |= r != 0;
	}

	str_free(&logs);
	return failed ? -1 : 0;
}
