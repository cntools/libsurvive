#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "kalman_filter.h"

/*
 * Performs alpha * A * B + beta * C
 */
extern void sgemm_(
        char *TRANSA,  // INPUT: Do we transpose A? 'n' no transpose, 't' transpose
		char *TRANSB,  // INPUT: Do we transpose B? 'n' no transpose, 't' transpose
		int *M,        // INPUT: Size parameter 'M'
		int *N,        // INPUT: Size parameter 'N'
		int *K,        // INPUT: Size parameter 'K'
		float *ALPHA,  // INPUT: scaling coefficient for A * B
		void *A,       // INPUT: (float) Column array 'A' (M by K)
		int *LDA,      // INPUT: Column stride for 'A'
		void *B,       // INPUT: (float) Column array 'B' (K by N)
		int *LDB,      // INPUT: Column stride for 'B'
		float *BETA,   // INPUT: Scaling factor for 'C'
		void *C,       // INPUT/OUTPUT: (float) Column array 'C' RESULT PLACED HERE
		int *LDC); 	   // INPUT: Column stride for 'C'

/*
 * General A X = B solution
 */
extern void sgesv_(
    int *N,            // INPUT: The order of matrix A
    int *NRHS,         // INPUT: the number of columns of matrix B
    void *A,           // INPUT/OUTPUT: (float) Column array 
                       //  entry: (N by N) matrix A
                       //  exit:  (L and U) from factorization A = P*L*U
    int *LDA,          // INPUT: Column stride of A
    int *IPIV,         // OUTPUT: Ineger array (dimension N) the pivot indices of the permutation matrix
    int *B,            // INPUT/OUTPUT: (float) Column array
                       //  entry: (N by NRHS) matrix of right hand side matrix 'B'
                       //  exit:  (if INFO=0) N-NRHS solution matrix 'X'
    int *LDB,          // INPUT: Column stride of B
    int *INFO);        // OUTPUT: Did it work?
                       //   0: success
                       // < 0: if INFO==-i, the -ith argument had illegal val
                       // > 0: if INFO== i, U(i,i) is exactly zero, thus the factorization is singular (error).


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
    /* Temporary storage */
    KAL_MAT(F_k__times__P_km1_km1);
    #define B_k__times__u_k  xhat_k_km1    /* This vector overwrite the same memory  */

    /* Fortran pass by pointer */
    float zerof=0.0f;
    float onef=1.0f;
    int one=1;
    int stride=KAL_STRIDE;

    int x,y;

    /* Calculate B_k__times__u_k */
    if (U == 0) {
        /* If no control input model, then zero out */
        memset(B_k__times__u_k, 0, S*sizeof(float));
    } else {

        /* Otherwise: B_k * u_k */
        sgemm_(
            "n",  // INPUT: Do we transpose A? 'n' no transpose, 't' transpose
    		"n",  // INPUT: Do we transpose B? 'n' no transpose, 't' transpose
    		&S,               // INPUT: Size parameter 'M'
    		&one,             // INPUT: Size parameter 'N'
    		&U,               // INPUT: Size parameter 'K'
    		&onef,            // INPUT: scaling coefficient for A * B
    	    B_k,              // INPUT: (float) Column array 'A' (M by K)
    		&stride,          // INPUT: Column stride for 'A'
    		u_k,              // INPUT: (float) Column array 'B' (K by N)
    		&stride,          // INPUT: Column stride for 'B'
    		&zerof,           // INPUT: Scaling factor for 'C'
    		B_k__times__u_k,  // INPUT/OUTPUT: (float) Column array 'C' RESULT PLACED HERE
    		&stride); 	      // INPUT: Column stride for 'C'        
    }

    /* Calculate xhat_k_km1 */
    sgemm_(
        "n",  // INPUT: Do we transpose A? 'n' no transpose, 't' transpose
		"n",  // INPUT: Do we transpose B? 'n' no transpose, 't' transpose
		&S,               // INPUT: Size parameter 'M'
		&one,             // INPUT: Size parameter 'N'
		&S,               // INPUT: Size parameter 'K'
		&onef,            // INPUT: scaling coefficient for A * B
	    F_k,              // INPUT: (float) Column array 'A' (M by K)
		&stride,          // INPUT: Column stride for 'A'
		xhat_km1_km1,     // INPUT: (float) Column array 'B' (K by N)
		&stride,          // INPUT: Column stride for 'B'
		&onef,            // INPUT: Scaling factor for 'C'
		xhat_k_km1,       // INPUT/OUTPUT: (float) Column array 'C' RESULT PLACED HERE
		&stride); 	      // INPUT: Column stride for 'C'        

    /* Calculate F_k * P_km1_km1*/
    sgemm_(
        "n",  // INPUT: Do we transpose A? 'n' no transpose, 't' transpose
		"n",  // INPUT: Do we transpose B? 'n' no transpose, 't' transpose
		&S,               // INPUT: Size parameter 'M'
		&S,               // INPUT: Size parameter 'N'
		&S,               // INPUT: Size parameter 'K'
		&onef,            // INPUT: scaling coefficient for A * B
	    F_k,              // INPUT: (float) Column array 'A' (M by K)
		&stride,          // INPUT: Column stride for 'A'
		P_km1_km1,        // INPUT: (float) Column array 'B' (K by N)
		&stride,          // INPUT: Column stride for 'B'
		&zerof,           // INPUT: Scaling factor for 'C'
		F_k__times__P_km1_km1,  // INPUT/OUTPUT: (float) Column array 'C' RESULT PLACED HERE
		&stride); 	      // INPUT: Column stride for 'C'

    /* Calculate P_k_km1 */
    for (x=0; x<S; x++) {
        for (y=0; y<S; y++) {
            P_k_km1[x][y] = Q_k[x][y];
        }
    }
    sgemm_(
        "n",  // INPUT: Do we transpose A? 'n' no transpose, 't' transpose
		"t",  // INPUT: Do we transpose B? 'n' no transpose, 't' transpose
		&S,               // INPUT: Size parameter 'M'
		&S,               // INPUT: Size parameter 'N'
		&S,               // INPUT: Size parameter 'K'
		&onef,            // INPUT: scaling coefficient for A * B
	    F_k__times__P_km1_km1,  // INPUT: (float) Column array 'A' (M by K)
		&stride,          // INPUT: Column stride for 'A'
		F_k,              // INPUT: (float) Column array 'B' (K by N)
		&stride,          // INPUT: Column stride for 'B'
		&zerof,           // INPUT: Scaling factor for 'C'
		P_k_km1,          // INPUT/OUTPUT: (float) Column array 'C' RESULT PLACED HERE
		&stride); 	      // INPUT: Column stride for 'C'
}


void KalmanUpdate(
    KAL_VEC(xhat_k_k),      /* OUTPUT: Updated state at time 'k' */
    KAL_MAT(P_k_k),         /* OUTPUT: Updated covariance at time 'k' */
    KAL_VEC(xhat_k_km1),    /* INPUT:  Predicted state at time 'k' */
    KAL_MAT(P_k_km1),       /* INPUT:  Predicted covariance at time 'k' */
    KAL_MAT(H_k),           /* INPUT:  Observational model */
    KAL_MAT(R_k),           /* INPUT:  Covariance of observational noise */
    int B,                  /* INPUT:  Number of observations in observation vector */
    int S)                  /* INPUT:  Number of measurements in the state vector */
{
}


