#include "minimal_opencv.h"
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>

void print_mat(const CvMat *M) {}

void test_gemm() {
  double _2x3[2*3] = {1, 2, 3, 4, 5, 6};
  CvMat m2x3 = cvMat(2, 3, CV_64F, _2x3);
  
  double _3x2[2*3] = {1, 2, 3, 4, 5, 6};
  CvMat m3x2 = cvMat(3, 2, CV_64F, _3x2);
  
  double _2x2[2*2] = {1, 2, 3, 4};
  CvMat m2x2 = cvMat(2, 2, CV_64F, _2x2);

  double _3x3[3*3] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
  CvMat m3x3 = cvMat(3, 3, CV_64F, _3x3);

  cvGEMM(&m2x3, &m3x2, 1, 0, 0, &m2x2, 0);
  cvGEMM(&m3x2, &m2x3, 1, 0, 0, &m3x3, 0);

  cvGEMM(&m2x3, &m2x3, 1, 0, 0, &m3x3, GEMM_1_T);
  cvGEMM(&m2x3, &m2x3, 1, 0, 0, &m2x2, GEMM_2_T);

  cvGEMM(&m2x3, &m3x2, 1, 0, 0, &m3x3, GEMM_1_T | GEMM_2_T);
  //  cvGEMM(&m3x2, &m2x3, 1, 0, 0, &m2x2, GEMM_1_T | GEMM_2_T);
  
}

int main()
{
  test_gemm();
  
  return 0;
}

