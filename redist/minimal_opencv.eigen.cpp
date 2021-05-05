#define EIGEN_RUNTIME_NO_MALLOC

#include "linmath.h"
#include "minimal_opencv.h"
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/QR>
#include <Eigen/SVD>

#include <iostream>

#ifdef USE_FLOAT
typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor, 50, 50> MatrixType;
#else
typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor, 50, 50> MatrixType;
#endif
typedef Eigen::Map<MatrixType> MapType;

#define CONVERT_TO_EIGEN(A) MapType(A ? CV_FLT_PTR(A) : 0, A ? (A)->rows : 0, A ? (A)->cols : 0)

double cvInvert(const CvMat *_srcarr, CvMat *_dstarr, int method) {
	auto src = CONVERT_TO_EIGEN(_srcarr);
	auto dst = CONVERT_TO_EIGEN(_dstarr);

	Eigen::internal::set_is_malloc_allowed(false);
	if (method == DECOMP_LU) {
		dst.noalias() = src.inverse();
	} else {
		dst.noalias() = src.completeOrthogonalDecomposition().pseudoInverse();
	}
	return 0;
}

void cvGEMM(const CvMat *_src1, const CvMat *_src2, double alpha, const CvMat *_src3, double beta, CvMat *_dst,
			int tABC) {
	auto src1 = CONVERT_TO_EIGEN(_src1);
	auto src2 = CONVERT_TO_EIGEN(_src2);

	auto dst = CONVERT_TO_EIGEN(_dst);

	Eigen::internal::set_is_malloc_allowed(false);
	if (tABC & CV_GEMM_A_T)
		if (tABC & CV_GEMM_B_T)
			dst.noalias() = alpha * src1.transpose() * src2.transpose();
		else
			dst.noalias() = alpha * src1.transpose() * src2;
	else {
		if (tABC & CV_GEMM_B_T)
			dst.noalias() = alpha * src1 * src2.transpose();
		else
			dst.noalias() = alpha * src1 * src2;
	}

	if (_src3) {
		auto src3 = CONVERT_TO_EIGEN(_src3);
		if (tABC & CV_GEMM_C_T)
			dst.noalias() += beta * src3.transpose();
		else
			dst.noalias() += beta * src3;
	}
}

const int DECOMP_SVD = 1;
const int DECOMP_LU = 2;

int cvSolve(const CvMat *_Aarr, const CvMat *_Barr, CvMat *_xarr, int method) {
	auto Aarr = CONVERT_TO_EIGEN(_Aarr);
	auto Barr = CONVERT_TO_EIGEN(_Barr);
	auto xarr = CONVERT_TO_EIGEN(_xarr);
	if (method == DECOMP_LU) {
		xarr.noalias() = Aarr.partialPivLu().solve(Barr);
	} else {
		Eigen::internal::set_is_malloc_allowed(true);
		auto svd = Aarr.bdcSvd(
			Eigen::ComputeFullU |
			Eigen::ComputeFullV); // Eigen::JacobiSVD<MatrixType>(Aarr, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::internal::set_is_malloc_allowed(false);
		xarr.noalias() = svd.solve(Barr);
	}
	return 0;
}

void cvSVD(CvMat *_aarr, CvMat *_warr, CvMat *_uarr, CvMat *_varr, int flags) {
	auto aarr = CONVERT_TO_EIGEN(_aarr);
	auto warr = CONVERT_TO_EIGEN(_warr);

	int options = 0;
	if (_uarr)
		options |= Eigen::ComputeFullU;
	if (_varr)
		options |= Eigen::ComputeFullV;
	Eigen::internal::set_is_malloc_allowed(true);
	auto svd = aarr.bdcSvd(options);
	Eigen::internal::set_is_malloc_allowed(false);

	if (warr.cols() == 1) {
		warr.noalias() = svd.singularValues();
	} else {
		warr.diagonal().noalias() = svd.singularValues();
	}

	if (_uarr) {
		auto uarr = CONVERT_TO_EIGEN(_uarr);
		if (flags & CV_SVD_U_T)
			uarr.noalias() = svd.matrixU().transpose();
		else
			uarr.noalias() = svd.matrixU();
	}

	if (_varr) {
		auto varr = CONVERT_TO_EIGEN(_varr);
		if (flags & CV_SVD_V_T)
			varr.noalias() = svd.matrixV().transpose();
		else
			varr.noalias() = svd.matrixV();
	}
}

void cvMulTransposed(const CvMat *_src, CvMat *_dst, int order, const CvMat *_delta, double scale) {
	auto src = CONVERT_TO_EIGEN(_src);
	auto dst = CONVERT_TO_EIGEN(_dst);

	if (_delta) {
		auto delta = CONVERT_TO_EIGEN(_delta);
		if (order == 0)
			dst.noalias() = scale * (src - delta) * (src - delta).transpose();
		else
			dst.noalias() = scale * (src - delta).transpose() * (src - delta);
	} else {
		if (order == 0)
			dst.noalias() = scale * src * src.transpose();
		else
			dst.noalias() = scale * src.transpose() * src;
	}
}

void cvTranspose(const CvMat *_src, CvMat *_dst) {
	auto src = CONVERT_TO_EIGEN(_src);
	auto dst = CONVERT_TO_EIGEN(_dst);
	if (CV_FLT_PTR(_src) == CV_FLT_PTR(_dst))
		src.transposeInPlace();
	else
		dst.noalias() = src.transpose();
}

void print_mat(const CvMat *M);

double cvDet(const CvMat *_M) {
	Eigen::internal::set_is_malloc_allowed(false);
	auto M = CONVERT_TO_EIGEN(_M);
	return M.determinant();
}