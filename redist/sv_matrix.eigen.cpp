#define EIGEN_RUNTIME_NO_MALLOC

#include "linmath.h"
#include "sv_matrix.h"
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

#define CONVERT_TO_EIGEN(A) MapType(A ? SV_FLT_PTR(A) : 0, A ? (A)->rows : 0, A ? (A)->cols : 0)

double svInvert(const SvMat *srcarr, SvMat *dstarr, int method) {
	auto src = CONVERT_TO_EIGEN(srcarr);
	auto dst = CONVERT_TO_EIGEN(dstarr);

	Eigen::internal::set_is_malloc_allowed(false);
	if (method == DECOMP_LU) {
		dst.noalias() = src.inverse();
	} else {
		dst.noalias() = src.completeOrthogonalDecomposition().pseudoInverse();
	}
	return 0;
}

void svGEMM(const SvMat *_src1, const SvMat *_src2, double alpha, const SvMat *_src3, double beta, SvMat *_dst,
			int tABC) {
	auto src1 = CONVERT_TO_EIGEN(_src1);
	auto src2 = CONVERT_TO_EIGEN(_src2);

	auto dst = CONVERT_TO_EIGEN(_dst);

	Eigen::internal::set_is_malloc_allowed(false);
	if (tABC & SV_GEMM_A_T)
		if (tABC & SV_GEMM_B_T)
			dst.noalias() = alpha * src1.transpose() * src2.transpose();
		else
			dst.noalias() = alpha * src1.transpose() * src2;
	else {
		if (tABC & SV_GEMM_B_T)
			dst.noalias() = alpha * src1 * src2.transpose();
		else
			dst.noalias() = alpha * src1 * src2;
	}

	if (_src3) {
		auto src3 = CONVERT_TO_EIGEN(_src3);
		if (tABC & SV_GEMM_C_T)
			dst.noalias() += beta * src3.transpose();
		else
			dst.noalias() += beta * src3;
	}
}

const int DECOMP_SVD = 1;
const int DECOMP_LU = 2;

int svSolve(const SvMat *_Aarr, const SvMat *_Barr, SvMat *_xarr, int method) {
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

void svSVD(SvMat *aarr, SvMat *warr, SvMat *uarr, SvMat *varr, int flags) {
	auto aarrEigen = CONVERT_TO_EIGEN(aarr);
	auto warrEigen = CONVERT_TO_EIGEN(warr);

	int options = 0;
	if (uarr)
		options |= Eigen::ComputeFullU;
	if (varr)
		options |= Eigen::ComputeFullV;
	Eigen::internal::set_is_malloc_allowed(true);
	auto svd = aarrEigen.bdcSvd(options);
	Eigen::internal::set_is_malloc_allowed(false);

	if (warrEigen.cols() == 1) {
		warrEigen.noalias() = svd.singularValues();
	} else if (warrEigen.rows() == 1) {
		warrEigen.noalias() = svd.singularValues().transpose();
	} else {
		warrEigen.diagonal().noalias() = svd.singularValues();
	}

	if (uarr) {
		auto uarrEigen = CONVERT_TO_EIGEN(uarr);
		if (flags & SV_SVD_U_T)
			uarrEigen.noalias() = svd.matrixU().transpose();
		else
			uarrEigen.noalias() = svd.matrixU();
	}

	if (varr) {
		auto varrEigen = CONVERT_TO_EIGEN(varr);
		if (flags & SV_SVD_V_T)
			varrEigen.noalias() = svd.matrixV().transpose();
		else
			varrEigen.noalias() = svd.matrixV();
	}
}

void svMulTransposed(const SvMat *src, SvMat *dst, int order, const SvMat *delta, double scale) {
	auto srcEigen = CONVERT_TO_EIGEN(src);
	auto dstEigen = CONVERT_TO_EIGEN(dst);

	if (delta) {
		auto deltaEigen = CONVERT_TO_EIGEN(delta);
		if (order == 0)
			dstEigen.noalias() = scale * (srcEigen - deltaEigen) * (srcEigen - deltaEigen).transpose();
		else
			dstEigen.noalias() = scale * (srcEigen - deltaEigen).transpose() * (src - delta);
	} else {
		if (order == 0)
			dstEigen.noalias() = scale * srcEigen * srcEigen.transpose();
		else
			dstEigen.noalias() = scale * srcEigen.transpose() * srcEigen;
	}
}

void svTranspose(const SvMat *M, SvMat *dst) {
	auto src = CONVERT_TO_EIGEN(M);
	auto dstEigen = CONVERT_TO_EIGEN(dst);
	if (SV_FLT_PTR(M) == SV_FLT_PTR(dst))
		src.transposeInPlace();
	else
		dstEigen.noalias() = src.transpose();
}

void print_mat(const SvMat *M);

double svDet(const SvMat *M) {
	Eigen::internal::set_is_malloc_allowed(false);
	auto MEigen = CONVERT_TO_EIGEN(M);
	return MEigen.determinant();
}