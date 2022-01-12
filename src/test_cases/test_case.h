#include "../survive_str.h"
#include "math.h"
#include "survive.h"

static inline int survive_test_assert() { return -1; }

#define ASSERT_SUCCESS(x)                                                                                              \
	{                                                                                                                  \
		int error = (x);                                                                                               \
		if (x < 0)                                                                                                     \
			return x;                                                                                                  \
	}

#define ASSERT_DOUBLE_EQ(val1, val2)                                                                                   \
	if (fabs((val1) - (val2)) > 1e-7 && fabs((val1) - (val2)) > ((fabs(val1 + val2)) * 0.0001)) {                      \
		fprintf(stderr, "Assert failed: " #val1 " != " #val2 ": %.13f != %.13f (%.13f)\n", val1, val2,                 \
				fabs((val1) - (val2)));                                                                                \
		return survive_test_assert();                                                                                  \
	}

#define ASSERT_DOUBLE_ARRAY_EQ(n, arr1, arr2)                                                                          \
	for (int i = 0; i < n; i++) {                                                                                      \
		ASSERT_DOUBLE_EQ(arr1[i], arr2[i]);                                                                            \
	}

#define ASSERT_QUAT_EQ(q1, q2) ASSERT_DOUBLE_ARRAY_EQ(4, q1, q2);

#define ASSERT_EQ(val1, val2)                                                                                          \
	if ((val1) != (val2)) {                                                                                            \
		fprintf(stderr, "Assert failed: " #val1 " == " #val2 ": %ld != %ld\n", (long)val1, (long)val2);                \
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
	REGISTER_LINKTIME(Test##suite##test_name)                                                                          \
	int Test##suite##test_name()

typedef int (*TestCase)();

extern cstring logs;
#define TEST_PRINTF(...) str_append_printf(&logs, __VA_ARGS__)