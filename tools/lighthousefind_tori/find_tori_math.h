#ifndef __FIND_TORI_MATH_H
#define __FIND_TORI_MATH_H

#include "tori_includes.h"

Matrix3x3 inverseM33(const Matrix3x3 mat);
double distance(Point a, Point b);

void unit_m3(double m[3][3]);
double dot_v3v3(const double a[3], const double b[3]);
double normalize_v3(double n[3]);
void cross_v3_v3v3(double r[3], const double a[3], const double b[3]);
void mul_v3_v3fl(double r[3], const double a[3], double f);
void ortho_v3_v3(double p[3], const double v[3]);
void axis_angle_normalized_to_mat3_ex(
        double mat[3][3], 
        const double axis[3],
        const double angle_sin, 
        const double angle_cos);
void rotation_between_vecs_to_mat3(double m[3][3], const double v1[3], const double v2[3]);


#endif
