#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include <dirent.h>
#include <dlfcn.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "assert.h"

static const char *get_so_filename() {
	static char so_path[1024] = {};
	if (so_path[0] == 0) {

		Dl_info dl_info;
		dladdr((void *)get_so_filename, &dl_info);
		ssize_t len = readlink(dl_info.dli_fname, so_path, sizeof(so_path));
		if (len == -1) {
			strncpy(so_path, dl_info.dli_fname, sizeof(so_path));
		} else {
			so_path[len] = 0;
		}
	}
	return so_path;
}

static const char *get_exe_filename() {
	static char exe_path[1024] = {};
	if (exe_path[0] == 0) {
		size_t len = readlink("/proc/self/exe", exe_path, sizeof(exe_path));
		exe_path[len] = 0;
	}
	return exe_path;
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

typedef struct list_t {
	size_t size;
	char **data;
} list_t;

static void list_add(list_t *list, const char *item) {
	list->data = realloc(list->data, sizeof(item) * ++list->size);
	list->data[list->size - 1] = malloc(strlen(item) + 1);
	strcpy(list->data[list->size - 1], item);
}

void survive_load_plugins(const char *plugin_dir) {
	// The basic strategy here is to compile a list of all possible plugins, then try to load them. Some
	// will fail if they have a dependency on other plugins which aren't loaded; and that is fine -- we
	// just keep loading the list until no new libraries are accepted.
	//
	// If there are still unresolved symbols, errors are reported.

	const char *check_from_files[] = {get_so_filename(), get_exe_filename(), getenv("SURVIVE_PLUGINS"), 0};
	const char *plugin_dirs[] = {"plugins", plugin_dir, 0};

	list_t plugin_list = {};

	for (const char **check_from_file = check_from_files; *check_from_file; check_from_file++) {
		char dirname[strlen(*check_from_file) + 1];
		strcpy(dirname, *check_from_file);
		char *filename = strip_filename(dirname);

		for (const char **plugin_dir = plugin_dirs; *plugin_dir; plugin_dir++) {
			char plugindirname[strlen(*check_from_file) + strlen(*plugin_dir) + 2];
			strcpy(plugindirname, dirname);

			strcat(plugindirname, "/");
			strcat(plugindirname, *plugin_dir);

			DIR *dir_handle = opendir(plugindirname);
			if (dir_handle == 0)
				continue;

			struct dirent *dir_entry = 0;
			while (dir_entry = readdir(dir_handle)) {
				if (strcmp(dir_entry->d_name, filename) != 0 && has_suffix(dir_entry->d_name, ".so")) {
					char full_path[1024] = {};
					snprintf(full_path, 1024, "%s/%s", plugindirname, dir_entry->d_name);
					list_add(&plugin_list, full_path);
				}
			}
		}
	}

	bool change = true;
	while (change) {
		change = false;
		for (size_t i = 0; i < plugin_list.size; i++) {
			char *plugin_path = plugin_list.data[i];
			if (plugin_path) {
				// Global is important to share symbols
				void *handle = dlopen(plugin_path, RTLD_NOW | RTLD_GLOBAL);
				if (handle) {
					// fprintf(stderr, "Loaded %s\n", plugin_path);
					change = true;
					free(plugin_path);
					plugin_list.data[i] = 0;
				} else {
					fprintf(stderr, "Error loading %s: %s\n", plugin_path, dlerror());
				}
			}
		}
	}

	for (size_t i = 0; i < plugin_list.size; i++) {
		if (plugin_list.data[i]) {
			void *handle = dlopen(plugin_list.data[i], RTLD_NOW);
			assert(handle == false);
			fprintf(stderr, "Error loading %s: %s\n", plugin_list.data[i], dlerror());
		}
	}

	free(plugin_list.data);
}
