#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "kalman_filter.h"

void KalmanPredict(
    KAL_VEC(xhat_k_km1),    /* OUTPUT: (S)     Predicted state at time 'k' */
    KAL_MAT(P_k_km1),       /* OUTPUT: (S x S) Predicted covariance at time 'k' */
    KAL_MAT(P_km1_km1),     /* INPUT:  (S x S) Updated covariance from time 'k-1' */
    KAL_VEC(xhat_km1_km1),  /* INPUT:  (S)     Updated state from time 'k-1' */
    KAL_MAT(F_k),           /* INPUT:  (S x S) State transition model */
    KAL_MAT(B_k),           /* INPUT:  (S x U) Control input model */
    KAL_VEC(u_k),           /* INPUT:  (U)     Control vector */
    KAL_MAT(Q_k),           /* INPUT:  (S x S) Covariance of process noise */
    int S,                  /* INPUT:          Number of dimensions in state vector */
    int U)                  /* INPUT:          Size of control input vector */
{
    KAL_MAT(F_k_tran);
    KAL_MAT(F_k__P_km1_km1);

    //   Predicted state:     xhat_k_km1 = Fk * xhat_km1_km1    +  Bk * uk
    MUL(F_k, xhat_km1_km1, xhat_k_km1, S,S,1);

    //   Predicted covar:     P_k_km1 = Fk * P_km1_km1 * Fk' +  Qk
    MUL(F_k, P_km1_km1, F_k__P_km1_km1, S, S, S);
    TRANSP(F_k, F_k_tran, S, S);
    MULADD(F_k__P_km1_km1, F_k_tran, Q_k, P_k_km1, S, S, S);
}

void KalmanUpdate(
    KAL_VEC(xhat_k_k),      /* (S)     OUTPUT: Updated state at time 'k' */
    KAL_MAT(P_k_k),         /* (S x S) OUTPUT: Updated covariance at time 'k' */
    KAL_VEC(xhat_k_km1),    /* (S)     INPUT:  Predicted state at time 'k' */
    KAL_MAT(P_k_km1),       /* (S x S) INPUT:  Predicted covariance at time 'k' */
    KAL_VEC(z_k),           /* (B)     INPUT:  Observation vector */
    KAL_MAT(H_k),           /* (B x S) INPUT:  Observational model */
    KAL_MAT(R_k),           /* (S x S) INPUT:  Covariance of observational noise */
    int B,                  /*         INPUT:  Number of observations in observation vector */
    int S)                  /*         INPUT:  Number of measurements in the state vector */
{
    // UPDATE PHASE
    //   Measurement residual:       yhat_k = zk - Hk * xhat_k_km1
    GMULADD(H_k,xhat_k_km1,z_k,D,-1.0f,1.0f,S,S,1);

    //   Residual covariance:           S_k = H_k * P_k_km1 * H_k' + R_k


    //   Optimal Kalman gain:           K_k = P_k_km1 * H_k' * inv(S_k)
    //   Updated state esti:       xhat_k_k = xhat_k_km1 + K_k * yhat_k
    //   Updated covariance:          P_k_k = (I - K_k * H_k) * P_k_km1
    
}



