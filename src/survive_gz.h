
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
	int rtn;
	gzerror(f, &rtn);
	return rtn;
}
#endif
