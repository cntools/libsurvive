#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <dirent.h>
#include <dlfcn.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static const char *get_so_filename() {
	Dl_info dl_info;
	dladdr((void *)get_so_filename, &dl_info);
	return (dl_info.dli_fname);
}

static char *strip_filename(char *buffer) {
	int n = strlen(buffer);
	char *ptr = buffer + n;
	while (ptr > buffer && *ptr != '/')
		ptr--;

	if (*ptr == '/') {
		*ptr = 0;
		ptr++;
	}

	return ptr;
}

static bool has_suffix(const char *str, const char *suffix) {
	int s_len = strlen(str);
	int suffix_len = strlen(suffix);

	if (suffix_len > s_len)
		return false;

	return strcmp(str + s_len - suffix_len, suffix) == 0;
}

void survive_load_plugins() {
	const char *_filename = get_so_filename();

	char dirname[strlen(_filename) + 1];
	strcpy(dirname, _filename);
	char *filename = strip_filename(dirname);

	DIR *dir_handle = opendir(dirname);
	struct dirent *dir_entry = 0;
	while (dir_entry = readdir(dir_handle)) {
		if (strcmp(dir_entry->d_name, filename) != 0 && has_suffix(dir_entry->d_name, ".so")) {
			char full_path[1024] = {};
			snprintf(full_path, 1024, "%s/%s", dirname, dir_entry->d_name);
			void *handle = dlopen(full_path, RTLD_NOW);
			if (!handle) {
				fprintf(stderr, "Error loading %s: %s\n", full_path, dlerror());
			}
		}
	}
}
