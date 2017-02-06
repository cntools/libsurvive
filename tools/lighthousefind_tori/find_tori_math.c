#include <math.h>
#include <float.h>
#include "find_tori_math.h"

// TODO: optimization potential to do in-place inverse for some places where this is used.
Matrix3x3 inverseM33(const Matrix3x3 mat)
{
    Matrix3x3 newMat;
    for (int a = 0; a < 3; a++)
    {
        for (int b = 0; b < 3; b++)
        {
            newMat.val[a][b] = mat.val[a][b];
        }
    }

    for (int i = 0; i < 3; i++)
    {
        for (int j = i + 1; j < 3; j++)
        {
            double tmp = newMat.val[i][j];
            newMat.val[i][j] = newMat.val[j][i];
            newMat.val[j][i] = tmp;
        }
    }

    return newMat;
}


double distance(Point a, Point b)
{
    double x = a.x - b.x;
    double y = a.y - b.y;
    double z = a.z - b.z;
    return sqrt(x*x + y*y + z*z);
}

//###################################
// The following code originally came from 
//  http://stackoverflow.com/questions/23166898/efficient-way-to-calculate-a-3x3-rotation-matrix-from-the-rotation-defined-by-tw
// Need to check up on license terms and give proper attribution
// I think we'll be good with proper attribution, but don't want to assume without checking.



/* -------------------------------------------------------------------- */
/* Math Lib declarations */



/* -------------------------------------------------------------------- */
/* Main function */

/**
* Calculate a rotation matrix from 2 normalized vectors.
*
* v1 and v2 must be unit length.
*/
void rotation_between_vecs_to_mat3(double m[3][3], const double v1[3], const double v2[3])
{
    double axis[3];
    /* avoid calculating the angle */
    double angle_sin;
    double angle_cos;

    cross_v3_v3v3(axis, v1, v2);

    angle_sin = normalize_v3(axis);
    angle_cos = dot_v3v3(v1, v2);

    if (angle_sin > FLT_EPSILON) {
    axis_calc:
        axis_angle_normalized_to_mat3_ex(m, axis, angle_sin, angle_cos);
    }
    else {
        /* Degenerate (co-linear) vectors */
        if (angle_cos > 0.0f) {
            /* Same vectors, zero rotation... */
            unit_m3(m);
        }
        else {
            /* Colinear but opposed vectors, 180 rotation... */
            ortho_v3_v3(axis, v1);
            normalize_v3(axis);
            angle_sin = 0.0f;  /* sin(M_PI) */
            angle_cos = -1.0f;  /* cos(M_PI) */
            goto axis_calc;
        }
    }
}


/* -------------------------------------------------------------------- */
/* Math Lib */

void unit_m3(double m[3][3])
{
    m[0][0] = m[1][1] = m[2][2] = 1.0;
    m[0][1] = m[0][2] = 0.0;
    m[1][0] = m[1][2] = 0.0;
    m[2][0] = m[2][1] = 0.0;
}

double dot_v3v3(const double a[3], const double b[3])
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void cross_v3_v3v3(double r[3], const double a[3], const double b[3])
{
    r[0] = a[1] * b[2] - a[2] * b[1];
    r[1] = a[2] * b[0] - a[0] * b[2];
    r[2] = a[0] * b[1] - a[1] * b[0];
}

void mul_v3_v3fl(double r[3], const double a[3], double f)
{
    r[0] = a[0] * f;
    r[1] = a[1] * f;
    r[2] = a[2] * f;
}

double normalize_v3_v3(double r[3], const double a[3])
{
    double d = dot_v3v3(a, a);

    if (d > 1.0e-35f) {
        d = sqrtf((float)d);
        mul_v3_v3fl(r, a, 1.0f / d);
    }
    else {
        d = r[0] = r[1] = r[2] = 0.0f;
    }

    return d;
}

double normalize_v3(double n[3])
{
    return normalize_v3_v3(n, n);
}

int axis_dominant_v3_single(const double vec[3])
{
    const float x = fabsf((float)vec[0]);
    const float y = fabsf((float)vec[1]);
    const float z = fabsf((float)vec[2]);
    return ((x > y) ?
        ((x > z) ? 0 : 2) :
        ((y > z) ? 1 : 2));
}

void ortho_v3_v3(double p[3], const double v[3])
{
    const int axis = axis_dominant_v3_single(v);

    switch (axis) {
    case 0:
        p[0] = -v[1] - v[2];
        p[1] = v[0];
        p[2] = v[0];
        break;
    case 1:
        p[0] = v[1];
        p[1] = -v[0] - v[2];
        p[2] = v[1];
        break;
    case 2:
        p[0] = v[2];
        p[1] = v[2];
        p[2] = -v[0] - v[1];
        break;
    }
}

/* axis must be unit length */
void axis_angle_normalized_to_mat3_ex(
    double mat[3][3], const double axis[3],
    const double angle_sin, const double angle_cos)
{
    double nsi[3], ico;
    double n_00, n_01, n_11, n_02, n_12, n_22;

    ico = (1.0f - angle_cos);
    nsi[0] = axis[0] * angle_sin;
    nsi[1] = axis[1] * angle_sin;
    nsi[2] = axis[2] * angle_sin;

    n_00 = (axis[0] * axis[0]) * ico;
    n_01 = (axis[0] * axis[1]) * ico;
    n_11 = (axis[1] * axis[1]) * ico;
    n_02 = (axis[0] * axis[2]) * ico;
    n_12 = (axis[1] * axis[2]) * ico;
    n_22 = (axis[2] * axis[2]) * ico;

    mat[0][0] = n_00 + angle_cos;
    mat[0][1] = n_01 + nsi[2];
    mat[0][2] = n_02 - nsi[1];
    mat[1][0] = n_01 - nsi[2];
    mat[1][1] = n_11 + angle_cos;
    mat[1][2] = n_12 + nsi[0];
    mat[2][0] = n_02 + nsi[1];
    mat[2][1] = n_12 - nsi[0];
    mat[2][2] = n_22 + angle_cos;
}
