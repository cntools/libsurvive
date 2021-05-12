
#ifdef NOZLIB
#define gzFile FILE *
#define gzopen fopen
#define gzprintf fprintf
#define gzclose fclose
#define gzvprintf vfprintf
#define gzerror_dropin ferror
#define gzwrite(file, buf, len) fwrite(buf, 1, len, file)
#define gzeof feof
#define gzseek fseek
#define gzgetc fgetc
#else
#include <zlib.h>
static inline int gzerror_dropin(gzFile f) {
	int rtn = 0;
	gzerror(f, &rtn);
	return rtn;
}

#ifdef HAVE_NO_GZVPRINTF
static int gzvprintf(gzFile file, const char *format, va_list va) {
	static char buffer[4096] = {0};
	int rtn = vsprintf(buffer, format, va);
	return gzwrite(file, buffer, rtn);
}
#endif

#endif
