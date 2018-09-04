#include "survive.h"

static inline int survive_test_assert() { return -1; }

#define ASSERT_DOUBLE_EQ(val1, val2)                                                                                   \
	if (fabs((val1) - (val2)) > 0.00001) {                                                                             \
		fprintf(stderr, "Assert failed: " #val1 " != " #val2 ": %f != %f (%f)\n", val1, val2, fabs((val1) - (val2)));  \
		return survive_test_assert();                                                                                  \
	}

#define ASSERT_GE(val1, val2)                                                                                          \
	if ((val1) < (val2)) {                                                                                             \
		fprintf(stderr, "Assert failed: " #val1 " < " #val2 ": %f < %f\n", val1, val2);                                \
		return survive_test_assert();                                                                                  \
	}

#define ASSERT_GT(val1, val2)                                                                                          \
	if ((val1) <= (val2)) {                                                                                            \
		fprintf(stderr, "Assert failed: " #val1 " <= " #val2 ": %f <= %f\n", val1, val2);                              \
		return survive_test_assert();                                                                                  \
	}

#define TEST(suite, test_name)                                                                                         \
	int Test##suite##test_name();                                                                                      \
	REGISTER_LINKTIME(Test##suite##test_name);                                                                         \
	int Test##suite##test_name()

typedef int (*TestCase)();
