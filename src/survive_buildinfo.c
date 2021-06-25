#include "survive.h"

#ifndef GIT_VERSION
#define GIT_VERSION "Unknown version"
#endif

SURVIVE_EXPORT const char *survive_build_tag() { return GIT_VERSION; }