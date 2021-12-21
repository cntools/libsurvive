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

static char error[512];

static void *survive_load_plugin(const char *path) { 
	SetLastError(0);
	error[0] = 0;
	void *rtn = LoadLibrary(path);
	if (rtn == 0) {
		FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, 
			0, GetLastError(), 0, error, 512, NULL);
	
	}
	return rtn;
}

const char* survive_load_plugin_error() {
	return error;
}

static const char* plugin_ext() { return ".dll";  }