#include "../survive_internal.h"
#include "test_case.h"

int main(int argc, char **argv) {
	int i = 0;
	bool failed = false;
	const char *DriverName = 0;
	while ((DriverName = GetDriverNameMatching("Test", i++))) {
		TestCase dd = GetDriver(DriverName);
		DriverName += 4; // We just want to show the test name without prefix 'Test'

		fprintf(stderr, "Running test %s (%d)\n", DriverName, i);
		int r = dd();
		fprintf(stderr, "Test %s reports status %d\n", DriverName, r);

		failed &= r == 0;
	}

	return failed ? -1 : 0;
}
