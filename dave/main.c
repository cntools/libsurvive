#include <stdio.h>
#include "kalman_filter.h"

int main()
{
    KAL_VEC(xhat_k_km1);    /* OUTPUT: (S)     Predicted state at time 'k' */
    KAL_MAT(P_k_km1);       /* OUTPUT: (S x S) Predicted covariance at time 'k' */
    KAL_MAT(P_km1_km1);     /* INPUT:  (S x S) Updated covariance from time 'k-1' */
    KAL_VEC(xhat_km1_km1);  /* INPUT:  (S)     Updated state from time 'k-1' */
    KAL_MAT(F_k);           /* INPUT:  (S x S) State transition model */
    KAL_MAT(B_k);           /* INPUT:  (S x U) Control input model */
    KAL_VEC(u_k);           /* INPUT:  (U)     Control vector */
    KAL_MAT(Q_k);           /* INPUT:  (S x S) Covariance of process noise */

    KalmanPredict(
        xhat_k_km1,    /* OUTPUT: (S)     Predicted state at time 'k' */
        P_k_km1,       /* OUTPUT: (S x S) Predicted covariance at time 'k' */
        P_km1_km1,     /* INPUT:  (S x S) Updated covariance from time 'k-1' */
        xhat_km1_km1,  /* INPUT:  (S)     Updated state from time 'k-1' */
        F_k,           /* INPUT:  (S x S) State transition model */
        B_k,           /* INPUT:  (S x U) Control input model */
        u_k,           /* INPUT:  (U)     Control vector */
        Q_k,           /* INPUT:  (S x S) Covariance of process noise */
        36,            /* INPUT:          Number of dimensions in state vector */
        36);            /* INPUT:          Size of control input vector */

    return 0;
}

