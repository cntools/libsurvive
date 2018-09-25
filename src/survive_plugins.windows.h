#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "dirent.windows.h"
#include "assert.h"

EXTERN_C IMAGE_DOS_HEADER __ImageBase;
static const char *get_so_filename() {
	static char module_path[MAX_PATH] = { 0 };
	if (!module_path[0])
		GetModuleFileName((HINSTANCE)&__ImageBase, module_path, MAX_PATH);
	return module_path;
}

static const char *get_exe_filename() {
	static char module_path[MAX_PATH] = { 0 };
	if (!module_path[0])
		GetModuleFileName(0, module_path, MAX_PATH);
	return module_path;
}

static void* survive_load_plugin(const char* path) {
	return LoadLibrary(path);
}

const char* survive_load_plugin_error() {
	return "";
}

static const char* plugin_ext() { return ".dll";  }