#include "minimal_opencv.h"
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

#define SURVIVE_CV_F CV_32F
#define CV_FLT CV_32F
#define CV_RAW_PTR(X) ((X)->data.fl)
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
#define SURVIVE_CV_F CV_64F
#define CV_FLT CV_64F
#define CV_RAW_PTR(X) ((X)->data.db)
#endif

#define CV_Error(code, msg) assert(0 && msg); // cv::error( code, msg, CV_Func, __FILE__, __LINE__ )
SURVIVE_LOCAL_ONLY CvMat *cvCloneMat(const CvMat *mat) {
	CvMat *rtn = cvCreateMat(mat->rows, mat->cols, mat->type);
	cvCopy(mat, rtn, 0);
	return rtn;
}

static size_t mat_size_bytes(const CvMat *mat) { return (size_t)CV_ELEM_SIZE(mat->type) * mat->cols * mat->rows; }

SURVIVE_LOCAL_ONLY void cvCopy(const CvMat *srcarr, CvMat *dstarr, const CvMat *mask) {
	assert(mask == 0 && "This isn't implemented yet");
	assert(srcarr->rows == dstarr->rows);
	assert(srcarr->cols == dstarr->cols);
	assert(dstarr->type == srcarr->type);
	memcpy(CV_RAW_PTR(dstarr), CV_RAW_PTR(srcarr), mat_size_bytes(srcarr));
}

SURVIVE_LOCAL_ONLY void cvSetZero(CvMat *arr) {
	for (int i = 0; i < arr->rows; i++)
		for (int j = 0; j < arr->cols; j++)
			CV_RAW_PTR(arr)[i * arr->cols + j] = 0;
}
SURVIVE_LOCAL_ONLY void cvSetIdentity(CvMat *arr) {
	for (int i = 0; i < arr->rows; i++)
		for (int j = 0; j < arr->cols; j++)
			CV_RAW_PTR(arr)[i * arr->cols + j] = i == j;
}

SURVIVE_LOCAL_ONLY void cvReleaseMat(CvMat **mat) {
	assert(*(*mat)->refcount == 1);
	free((*mat)->refcount);
	free(*mat);
	*mat = 0;
}

/* the alignment of all the allocated buffers */
#define CV_MALLOC_ALIGN 16
SURVIVE_LOCAL_ONLY void *cvAlloc(size_t size) { return malloc(size); }
static inline void *cvAlignPtr(const void *ptr, int align) {
	assert((align & (align - 1)) == 0);
	return (void *)(((size_t)ptr + align - 1) & ~(size_t)(align - 1));
}

SURVIVE_LOCAL_ONLY void cvCreateData(CvMat *arr) {
	if (CV_IS_MAT_HDR_Z(arr)) {
		size_t step, total_size;
		CvMat *mat = (CvMat *)arr;
		step = mat->step;

		if (mat->rows == 0 || mat->cols == 0)
			return;

		if (mat->data.ptr != 0)
			CV_Error(CV_StsError, "Data is already allocated");

		if (step == 0)
			step = CV_ELEM_SIZE(mat->type) * mat->cols;

		int64_t _total_size = (int64_t)step * mat->rows + sizeof(int) + CV_MALLOC_ALIGN;
		total_size = (size_t)_total_size;
		if (_total_size != (int64_t)total_size)
			CV_Error(CV_StsNoMem, "Too big buffer is allocated");
		mat->refcount = (int *)cvAlloc((size_t)total_size);
		mat->data.ptr = (unsigned char *)cvAlignPtr(mat->refcount + 1, CV_MALLOC_ALIGN);
		*mat->refcount = 1;
	} else if (CV_IS_MATND_HDR(arr)) {
		assert("There is no support for ND types");
	} else
		CV_Error(CV_StsBadArg, "unrecognized or unsupported array type");
}

static void icvCheckHuge(CvMat *arr) {
	if ((int64_t)arr->step * arr->rows > INT_MAX)
		arr->type &= ~CV_MAT_CONT_FLAG;
}

CvMat *cvInitMatHeader(CvMat *arr, int rows, int cols, int type) {
	type = CV_MAT_TYPE(type);

	assert(!(rows < 0 || cols < 0));

	int min_step = CV_ELEM_SIZE(type);
	assert(!(min_step <= 0));
	min_step *= cols;

	arr->step = min_step;
	arr->type = CV_MAT_MAGIC_VAL | type | CV_MAT_CONT_FLAG;
	arr->rows = rows;
	arr->cols = cols;
	arr->data.ptr = 0;
	arr->refcount = 0;
	arr->hdr_refcount = 1;

	icvCheckHuge(arr);
	return arr;
}

SURVIVE_LOCAL_ONLY CvMat *cvCreateMatHeader(int rows, int cols, int type) {
	return cvInitMatHeader((CvMat *)cvAlloc(sizeof(CvMat)), rows, cols, type);
}
SURVIVE_LOCAL_ONLY CvMat *cvCreateMat(int height, int width, int type) {
	CvMat *arr = cvCreateMatHeader(height, width, type);
	cvCreateData(arr);

	return arr;
}