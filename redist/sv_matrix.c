#include "sv_matrix.h"
#include <limits.h>
#include <string.h>

#ifdef _WIN32
#define SURVIVE_LOCAL_ONLY
#include <malloc.h>
#define alloca _alloca
#else
#define SURVIVE_LOCAL_ONLY __attribute__((visibility("hidden")))
#endif

#ifdef USE_FLOAT

#define FLT float
#define FLT_SQRT sqrtf
#define FLT_TAN tanf
#define FLT_SIN sinf
#define FLT_COS cosf
#define FLT_ACOS acosf
#define FLT_ASIN asinf
#define FLT_ATAN2 atan2f
#define FLT_FABS__ fabsf
#define FLT_STRTO strtof

#define SURVIVE_SV_F SV_32F
#define SV_FLT SV_32F
#define SV_RAW_PTR(X) ((X)->data.fl)
#define FLT_POW powf

#else

#define USE_DOUBLE 1
#define FLT double
#define FLT_SQRT sqrt
#define FLT_TAN tan
#define FLT_SIN sin
#define FLT_COS cos
#define FLT_ACOS acos
#define FLT_ASIN asin
#define FLT_ATAN2 atan2
#define FLT_FABS__ fabs
#define FLT_STRTO strtod
#define SURVIVE_SV_F SV_64F
#define SV_FLT SV_64F
#define SV_RAW_PTR(X) ((X)->data.db)
#endif

#define SV_Error(code, msg) assert(0 && msg); // cv::error( code, msg, SV_Func, __FILE__, __LINE__ )
SURVIVE_LOCAL_ONLY SvMat *svCloneMat(const SvMat *mat) {
	SvMat *rtn = svCreateMat(mat->rows, mat->cols, mat->type);
	svCopy(mat, rtn, 0);
	return rtn;
}

static size_t mat_size_bytes(const SvMat *mat) { return (size_t)SV_ELEM_SIZE(mat->type) * mat->cols * mat->rows; }

void svCopy(const SvMat *src, SvMat *dest, const SvMat *mask) {
	assert(mask == 0 && "This isn't implemented yet");
	assert(src->rows == dest->rows);
	assert(src->cols == dest->cols);
	assert(dest->type == src->type);
	memcpy(SV_RAW_PTR(dest), SV_RAW_PTR(src), mat_size_bytes(src));
}

SURVIVE_LOCAL_ONLY void svSetZero(SvMat *arr) {
	for (int i = 0; i < arr->rows; i++)
		for (int j = 0; j < arr->cols; j++)
			SV_RAW_PTR(arr)[i * arr->cols + j] = 0;
}
SURVIVE_LOCAL_ONLY void cvSetIdentity(SvMat *arr) {
	for (int i = 0; i < arr->rows; i++)
		for (int j = 0; j < arr->cols; j++)
			SV_RAW_PTR(arr)[i * arr->cols + j] = i == j;
}

SURVIVE_LOCAL_ONLY void svReleaseMat(SvMat **mat) {
	assert(*(*mat)->refcount == 1);
	free((*mat)->refcount);
	free(*mat);
	*mat = 0;
}

/* the alignment of all the allocated buffers */
#define SV_MALLOC_ALIGN 16
SURVIVE_LOCAL_ONLY void *svAlloc(size_t size) { return malloc(size); }
static inline void *svAlignPtr(const void *ptr, int align) {
	assert((align & (align - 1)) == 0);
	return (void *)(((size_t)ptr + align - 1) & ~(size_t)(align - 1));
}

SURVIVE_LOCAL_ONLY void svCreateData(SvMat *arr) {
	if (SV_IS_MAT_HDR_Z(arr)) {
		size_t step, total_size;
		SvMat *mat = (SvMat *)arr;
		step = mat->step;

		if (mat->rows == 0 || mat->cols == 0)
			return;

		if (mat->data.ptr != 0)
			SV_Error(SV_StsError, "Data is already allocated");

		if (step == 0)
			step = SV_ELEM_SIZE(mat->type) * mat->cols;

		int64_t _total_size = (int64_t)step * mat->rows + sizeof(int) + SV_MALLOC_ALIGN;
		total_size = (size_t)_total_size;
		if (_total_size != (int64_t)total_size)
			SV_Error(SV_StsNoMem, "Too big buffer is allocated");
		mat->refcount = (int *)svAlloc((size_t)total_size);
		mat->data.ptr = (unsigned char *)svAlignPtr(mat->refcount + 1, SV_MALLOC_ALIGN);
		*mat->refcount = 1;
	} else if (SV_IS_MATND_HDR(arr)) {
		assert("There is no support for ND types");
	} else
		SV_Error(SV_StsBadArg, "unrecognized or unsupported array type");
}

static void icvCheckHuge(SvMat *arr) {
	if ((int64_t)arr->step * arr->rows > INT_MAX)
		arr->type &= ~SV_MAT_CONT_FLAG;
}

SvMat *svInitMatHeader(SvMat *arr, int rows, int cols, int type) {
	type = SV_MAT_TYPE(type);

	assert(!(rows < 0 || cols < 0));

	int min_step = SV_ELEM_SIZE(type);
	assert(!(min_step <= 0));
	min_step *= cols;

	arr->step = min_step;
	arr->type = SV_MAT_MAGIC_VAL | type | SV_MAT_CONT_FLAG;
	arr->rows = rows;
	arr->cols = cols;
	arr->data.ptr = 0;
	arr->refcount = 0;
	arr->hdr_refcount = 1;

	icvCheckHuge(arr);
	return arr;
}

SURVIVE_LOCAL_ONLY SvMat *svCreateMatHeader(int rows, int cols, int type) {
	return svInitMatHeader((SvMat *)svAlloc(sizeof(SvMat)), rows, cols, type);
}
SURVIVE_LOCAL_ONLY SvMat *svCreateMat(int height, int width, int type) {
	SvMat *arr = svCreateMatHeader(height, width, type);
	svCreateData(arr);

	return arr;
}