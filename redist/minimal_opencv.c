#include <cblas.h>
#include <lapacke.h>

#include "math.h"
#include "minimal_opencv.h"
#include "stdbool.h"
#include "stdio.h"
#include "string.h"

#include <limits.h>
#include <varargs.h>

//#define DEBUG_PRINT

#ifdef _WIN3211
#include "cblas.h"

int CBLAS_CallFromC;
int RowMajorStrg;
void cblas_xerbla(int info, const char *rout, const char *form, ...)
{
	extern int RowMajorStrg;
	char empty[1] = "";
	va_list argptr;

	va_start(argptr, form);

	if (RowMajorStrg)
	{
		if (strstr(rout, "gemm") != 0)
		{
			if (info == 5) info = 4;
			else if (info == 4) info = 5;
			else if (info == 11) info = 9;
			else if (info == 9) info = 11;
		}
		else if (strstr(rout, "symm") != 0 || strstr(rout, "hemm") != 0)
		{
			if (info == 5) info = 4;
			else if (info == 4) info = 5;
		}
		else if (strstr(rout, "trmm") != 0 || strstr(rout, "trsm") != 0)
		{
			if (info == 7) info = 6;
			else if (info == 6) info = 7;
		}
		else if (strstr(rout, "gemv") != 0)
		{
			if (info == 4)  info = 3;
			else if (info == 3)  info = 4;
		}
		else if (strstr(rout, "gbmv") != 0)
		{
			if (info == 4)  info = 3;
			else if (info == 3)  info = 4;
			else if (info == 6)  info = 5;
			else if (info == 5)  info = 6;
		}
		else if (strstr(rout, "ger") != 0)
		{
			if (info == 3) info = 2;
			else if (info == 2) info = 3;
			else if (info == 8) info = 6;
			else if (info == 6) info = 8;
		}
		else if ((strstr(rout, "her2") != 0 || strstr(rout, "hpr2") != 0)
			&& strstr(rout, "her2k") == 0)
		{
			if (info == 8) info = 6;
			else if (info == 6) info = 8;
		}
	}
	if (info)
		fprintf(stderr, "Parameter %d to routine %s was incorrect\n", info, rout);
	vfprintf(stderr, form, argptr);
	va_end(argptr);
	if (info && !info)
		F77_xerbla(empty, &info); /* Force link of our F77 error handler */
	exit(-1);
}
void cblas_dgemm(const CBLAS_LAYOUT layout, const CBLAS_TRANSPOSE TransA,
	const CBLAS_TRANSPOSE TransB, const int M, const int N,
	const int K, const double alpha, const double  *A,
	const int lda, const double  *B, const int ldb,
	const double beta, double  *C, const int ldc)
{
	char TA, TB;
#ifdef F77_CHAR
	F77_CHAR F77_TA, F77_TB;
#else
#define F77_TA &TA
#define F77_TB &TB
#endif

#ifdef F77_INT
	F77_INT F77_M = M, F77_N = N, F77_K = K, F77_lda = lda, F77_ldb = ldb;
	F77_INT F77_ldc = ldc;
#else
#define F77_M M
#define F77_N N
#define F77_K K
#define F77_lda lda
#define F77_ldb ldb
#define F77_ldc ldc
#endif

	RowMajorStrg = 0;
	CBLAS_CallFromC = 1;

	if (layout == CblasColMajor)
	{
		if (TransA == CblasTrans) TA = 'T';
		else if (TransA == CblasConjTrans) TA = 'C';
		else if (TransA == CblasNoTrans)   TA = 'N';
		else
		{
			cblas_xerbla(2, "cblas_dgemm", "Illegal TransA setting, %d\n", TransA);
			CBLAS_CallFromC = 0;
			RowMajorStrg = 0;
			return;
		}

		if (TransB == CblasTrans) TB = 'T';
		else if (TransB == CblasConjTrans) TB = 'C';
		else if (TransB == CblasNoTrans)   TB = 'N';
		else
		{
			cblas_xerbla(3, "cblas_dgemm", "Illegal TransB setting, %d\n", TransB);
			CBLAS_CallFromC = 0;
			RowMajorStrg = 0;
			return;
		}

#ifdef F77_CHAR
		F77_TA = C2F_CHAR(&TA);
		F77_TB = C2F_CHAR(&TB);
#endif

		F77_dgemm(F77_TA, F77_TB, &F77_M, &F77_N, &F77_K, &alpha, A,
			&F77_lda, B, &F77_ldb, &beta, C, &F77_ldc);
	}
	else if (layout == CblasRowMajor)
	{
		RowMajorStrg = 1;
		if (TransA == CblasTrans) TB = 'T';
		else if (TransA == CblasConjTrans) TB = 'C';
		else if (TransA == CblasNoTrans)   TB = 'N';
		else
		{
			cblas_xerbla(2, "cblas_dgemm", "Illegal TransA setting, %d\n", TransA);
			CBLAS_CallFromC = 0;
			RowMajorStrg = 0;
			return;
		}
		if (TransB == CblasTrans) TA = 'T';
		else if (TransB == CblasConjTrans) TA = 'C';
		else if (TransB == CblasNoTrans)   TA = 'N';
		else
		{
			cblas_xerbla(2, "cblas_dgemm", "Illegal TransB setting, %d\n", TransB);
			CBLAS_CallFromC = 0;
			RowMajorStrg = 0;
			return;
		}
#ifdef F77_CHAR
		F77_TA = C2F_CHAR(&TA);
		F77_TB = C2F_CHAR(&TB);
#endif

		F77_dgemm(F77_TA, F77_TB, &F77_N, &F77_M, &F77_K, &alpha, B,
			&F77_ldb, A, &F77_lda, &beta, C, &F77_ldc);
	}
	else  cblas_xerbla(1, "cblas_dgemm", "Illegal layout setting, %d\n", layout);
	CBLAS_CallFromC = 0;
	RowMajorStrg = 0;
	return;
}
#endif


int cvRound(float f) { return roundf(f); }
#define CV_Error(code, msg) assert(0 && msg); // cv::error( code, msg, CV_Func, __FILE__, __LINE__ )

const int DECOMP_SVD = 1;
const int DECOMP_LU = 2;

void print_mat(const CvMat *M);

void cvCopyTo(const CvMat *srcarr, CvMat *dstarr) {
	memcpy(dstarr->data.db, srcarr->data.db, sizeof(double) * dstarr->rows * dstarr->cols);
}

void cvGEMM(const CvMat *src1, const CvMat *src2, double alpha, const CvMat *src3, double beta, CvMat *dst, int tABC) {
	lapack_int rows1 = src1->rows;
	lapack_int cols1 = src1->cols;
		
	lapack_int rows2 = src2->rows;
	lapack_int cols2 = src2->cols;

	lapack_int lda = cols1;
	lapack_int ldb = cols2;

	assert(src1->cols == src2->rows);
	assert(src1->rows == dst->rows);
	assert(src2->cols == dst->cols);

	if (src3)
		cvCopyTo(src3, dst);
	else
		beta = 0;

	cblas_dgemm(CblasRowMajor, (tABC & GEMM_1_T) ? CblasTrans : CblasNoTrans,
				(tABC & GEMM_2_T) ? CblasTrans : CblasNoTrans, src1->rows, dst->cols, src1->cols, alpha, src1->data.db,
				lda, src2->data.db, ldb, beta, dst->data.db, dst->cols);
}

void cvMulTransposed(const CvMat *src, CvMat *dst, int order, const CvMat *delta, double scale) {
	lapack_int rows = src->rows;
	lapack_int cols = src->cols;

	lapack_int drows = dst->rows;
	assert(drows == cols);
	assert(order == 1 ? (dst->cols == src->cols) : (dst->cols == src->rows));
	assert(delta == 0 && "This isn't implemented yet");
	double beta = 0;

	bool isAT = order == 1;
	bool isBT = !isAT;

	lapack_int dstCols = dst->cols;

	cblas_dgemm(CblasRowMajor, isAT ? CblasTrans : CblasNoTrans, isBT ? CblasTrans : CblasNoTrans, cols, dstCols, rows,
				scale, src->data.db, cols, src->data.db, cols, beta, dst->data.db, dstCols);
}

void *cvAlloc(size_t size) { return malloc(size); }

static void icvCheckHuge(CvMat *arr) {
	if ((int64_t)arr->step * arr->rows > INT_MAX)
		arr->type &= ~CV_MAT_CONT_FLAG;
}

CvMat *cvCreateMatHeader(int rows, int cols, int type) {
	type = CV_MAT_TYPE(type);

	assert(!(rows < 0 || cols < 0));

	int min_step = CV_ELEM_SIZE(type);
	assert(!(min_step <= 0));
	min_step *= cols;

	CvMat *arr = (CvMat *)cvAlloc(sizeof(*arr));

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

/* the alignment of all the allocated buffers */
#define CV_MALLOC_ALIGN 16

/* IEEE754 constants and macros */
#define CV_TOGGLE_FLT(x) ((x) ^ ((int)(x) < 0 ? 0x7fffffff : 0))
#define CV_TOGGLE_DBL(x) ((x) ^ ((int64)(x) < 0 ? CV_BIG_INT(0x7fffffffffffffff) : 0))

#define CV_DbgAssert assert

static inline void *cvAlignPtr(const void *ptr, int align) {
	CV_DbgAssert((align & (align - 1)) == 0);
	return (void *)(((size_t)ptr + align - 1) & ~(size_t)(align - 1));
}

static inline int cvAlign(int size, int align) {
	CV_DbgAssert((align & (align - 1)) == 0 && size < INT_MAX);
	return (size + align - 1) & -align;
}

void cvCreateData(CvMat *arr) {
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
		mat->data.ptr = (uchar *)cvAlignPtr(mat->refcount + 1, CV_MALLOC_ALIGN);
		*mat->refcount = 1;
	} else if (CV_IS_MATND_HDR(arr)) {
		assert("There is no support for ND types");
	} else
		CV_Error(CV_StsBadArg, "unrecognized or unsupported array type");
}

CvMat *cvCreateMat(int height, int width, int type) {
	CvMat *arr = cvCreateMatHeader(height, width, type);
	cvCreateData(arr);

	return arr;
}

double cvInvert(const CvMat *srcarr, CvMat *dstarr, int method) {
	lapack_int inf;
	lapack_int rows = srcarr->rows;
	lapack_int cols = srcarr->cols;
	lapack_int lda = srcarr->cols;

	cvCopyTo(srcarr, dstarr);
	double *a = dstarr->data.db;

#ifdef DEBUG_PRINT
	printf("a: \n");
	print_mat(srcarr);
#endif
	if (method == DECOMP_LU) {
		lapack_int *ipiv = malloc(sizeof(lapack_int) * MIN(srcarr->rows, srcarr->cols));
		inf = LAPACKE_dgetrf(LAPACK_ROW_MAJOR, rows, cols, a, lda, ipiv);
		assert(inf == 0);

		inf = LAPACKE_dgetri(LAPACK_ROW_MAJOR, rows, a, lda, ipiv);
		assert(inf >= 0);
		if (inf > 0) {
			printf("Warning: Singular matrix: \n");
			print_mat(srcarr);
		}

		free(ipiv);

	} else if (method == DECOMP_SVD) {
		// TODO: There is no way this needs this many allocations,
		// but in my defense I was very tired when I wrote this code
		CvMat *w = cvCreateMat(1, MIN(dstarr->rows, dstarr->cols), dstarr->type);
		CvMat *u = cvCreateMat(dstarr->cols, dstarr->cols, dstarr->type);
		CvMat *v = cvCreateMat(dstarr->rows, dstarr->rows, dstarr->type);
		CvMat *um = cvCreateMat(w->cols, w->cols, w->type);

		cvSVD(dstarr, w, u, v, 0);

		cvSetZero(um);
		for (int i = 0; i < w->cols; i++) {
			cvmSet(um, i, i, 1. / w->data.db[i]);
		}

		CvMat *tmp = cvCreateMat(dstarr->cols, dstarr->rows, dstarr->type);
		cvGEMM(v, um, 1, 0, 0, tmp, GEMM_1_T);
		cvGEMM(tmp, u, 1, 0, 0, dstarr, GEMM_2_T);

		cvReleaseMat(&tmp);
		cvReleaseMat(&w);
		cvReleaseMat(&u);
		cvReleaseMat(&v);
		cvReleaseMat(&um);
	}
	return 0;
}

CvMat *cvCloneMat(const CvMat *mat) {
	CvMat *rtn = cvCreateMat(mat->rows, mat->cols, mat->type);
	cvCopyTo(mat, rtn);
	return rtn;
}

int cvSolve(const CvMat *Aarr, const CvMat *xarr, CvMat *Barr, int method) {
	lapack_int inf;
	lapack_int arows = Aarr->rows;
	lapack_int acols = Aarr->cols;
	lapack_int xcols = xarr->cols;
	lapack_int xrows = xarr->rows;
	lapack_int lda = acols; // Aarr->step / sizeof(double);
	lapack_int type = CV_MAT_TYPE(Aarr->type);

	if (method == DECOMP_LU) {
		assert(Aarr->cols == Barr->rows);
		assert(xarr->rows == Aarr->rows);
		assert(Barr->cols == xarr->cols);
		assert(type == CV_MAT_TYPE(Barr->type) && (type == CV_32F || type == CV_64F));

		cvCopyTo(xarr, Barr);
		CvMat *a_ws = cvCloneMat(Aarr);

		lapack_int brows = Barr->rows;
		lapack_int bcols = Barr->cols;
		lapack_int ldb = bcols; // Barr->step / sizeof(double);

		lapack_int *ipiv = malloc(sizeof(lapack_int) * MIN(Aarr->rows, Aarr->cols));

		inf = LAPACKE_dgetrf(LAPACK_ROW_MAJOR, arows, acols, a_ws->data.db, lda, ipiv);
		assert(inf >= 0);
		if (inf > 0) {
			printf("Warning: Singular matrix: \n");
			print_mat(a_ws);
		}

#ifdef DEBUG_PRINT
		printf("Solve A * x = B:\n");
		print_mat(a_ws);
		print_mat(Barr);
#endif

		inf =
			LAPACKE_dgetrs(LAPACK_ROW_MAJOR, CblasNoTrans, arows, bcols, a_ws->data.db, lda, ipiv, Barr->data.db, ldb);
		assert(inf == 0);

		free(ipiv);
		cvReleaseMat(&a_ws);
	} else if (method == DECOMP_SVD) {

#ifdef DEBUG_PRINT
		printf("Solve |b - A * x|:\n");
		print_mat(Aarr);
		print_mat(xarr);
#endif

		CvMat *aCpy = cvCloneMat(Aarr);
		CvMat *xCpy = cvCloneMat(xarr);
		double *S = malloc(sizeof(double) * MIN(arows, acols));
		double rcond = -1;
		lapack_int *rank = malloc(sizeof(lapack_int) * MIN(arows, acols));
		lapack_int inf = LAPACKE_dgelss(LAPACK_ROW_MAJOR, arows, acols, xcols, aCpy->data.db, acols, xCpy->data.db,
										xcols, S, rcond, rank);
		free(rank);
		free(S);

		assert(Barr->rows == acols);
		assert(Barr->cols == xCpy->cols);
		xCpy->rows = acols;
		cvCopyTo(xCpy, Barr);

		cvReleaseMat(&aCpy);
		cvReleaseMat(&xCpy);
#ifdef DEBUG_PRINT
		print_mat(Barr);
#endif
		assert(inf == 0);
	}
	return 0;
}

void cvTranspose(const CvMat *M, CvMat *dst) {
	bool inPlace = M == dst || M->data.db == dst->data.db;
	double *src = M->data.db;

	CvMat *tmp = 0;
	if (inPlace) {
		tmp = cvCloneMat(dst);
		src = tmp->data.db;
	}

	assert(M->rows == dst->cols);
	assert(M->cols == dst->rows);
	for (unsigned i = 0; i < M->rows; i++) {
		for (unsigned j = 0; j < M->cols; j++) {
			dst->data.db[j * M->rows + i] = src[i * M->cols + j];
		}
	}

	if (inPlace) {
		cvReleaseMat(&tmp);
	}
}

void cvSVD(CvMat *aarr, CvMat *warr, CvMat *uarr, CvMat *varr, int flags) {
	char jobu = 'A';
	char jobvt = 'A';

	lapack_int inf;

	if ((flags & CV_SVD_MODIFY_A) == 0) {
		aarr = cvCloneMat(aarr);
	}

	if (uarr == 0)
		jobu = 'N';
	if (varr == 0)
		jobvt = 'N';

	lapack_int arows = aarr->rows, acols = aarr->cols;
	lapack_int ulda = uarr ? uarr->cols : 1;
	lapack_int plda = varr ? varr->cols : acols;

	double *superb = malloc(sizeof(double) * MIN(arows, acols));
	inf = LAPACKE_dgesvd(LAPACK_ROW_MAJOR, jobu, jobvt, arows, acols, aarr->data.db, acols, warr ? warr->data.db : 0,
						 uarr ? uarr->data.db : 0, ulda, varr ? varr->data.db : 0, plda, superb);

	free(superb);
	assert(inf == 0);
	if (uarr && (flags & CV_SVD_U_T)) {
		cvTranspose(uarr, uarr);
	}

	if (varr && (flags & CV_SVD_V_T) == 0) {
		cvTranspose(varr, varr);
	}

	if ((flags & CV_SVD_MODIFY_A) == 0) {
		cvReleaseMat(&aarr);
	}
}

void cvSetZero(CvMat *arr) {
	for (int i = 0; i < arr->rows; i++)
		for (int j = 0; j < arr->cols; j++)
			arr->data.db[i * arr->cols + j] = 0;
}

void cvReleaseMat(CvMat **mat) {
	assert(*(*mat)->refcount == 1);
	free((*mat)->refcount);
	free(*mat);
	*mat = 0;
}
