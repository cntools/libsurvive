//
//  main.c
//  AffineSolve
//
//  Created by user on 3/2/17.
//  Copyright © 2017 user. All rights reserved.
//

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define LH_ID    0
#define NUM_HMD 32

float hmd_pos[NUM_HMD][3];
void ReadHmdPoints()
{
    int i;
    FILE *fin = fopen("HMD_points.csv","r");
    if (fin==NULL) {
        printf("ERROR: could not open HMD_points.csv for reading\n");
        exit(1);
    }
    
    for (i=0; i<NUM_HMD; i++) {
        fscanf(fin, "%f %f %f", &(hmd_pos[i][0]), &(hmd_pos[i][1]), &(hmd_pos[i][2]));
    }
    
    fclose(fin);
}

#define MAX_POINTS 128
#define _ABS(a)  ( (a)<=0 ? -(a) : (a) )
#define _SIGN(a) ( (a)<=0 ? -1.0f : 1.0f )
#define RANDF  ( (float)rand() / (float)RAND_MAX )

#define STEP_SIZE_ROT 1.0
#define STEP_SIZE_POS 1.0
#define FALLOFF   0.99999
#define NITER     2000000
#define TOO_SMALL 0.0001
#define ORTHOG_PENALTY   1.0

#define PRINT_MAT(A,M,N) { \
    int m,n; \
    printf(#A "\n"); \
    for (m=0; m<M; m++) { \
        for (n=0; n<N; n++) { \
            printf("%f\t", A[m][n]); \
        } \
        printf("\n"); \
    } \
}

void AffineSolve(
    float T[4][4],           // OUTPUT: transform
    float O[MAX_POINTS][4],  // INPUT:  points, offsets
    float N[MAX_POINTS][3],  // INPUT:  plane normals
    float D[MAX_POINTS],     // INPUT:  plane offsets
    int nPoints, int nIter,
    float stepSizeRot, float stepSizePos, float falloff, int constrain)
{
    int i,j,k,iter;
    //T[3][3] = 1.0f;
    
    printf("iter x y z error\n");
    
    float gradDot     = 1.0;
    float prevGradDot = 1.0;
    float de_dT[3][4];  // the gradient
    float conj[3][4];   // the conjugate
    float errorSq=0.0;
    for (iter=0; iter<nIter; iter++)
    {
        //----------------------------------
        // Calculate the gradient direction
        //----------------------------------
        errorSq = 0.0;
        memset(de_dT, 0, 3*4*sizeof(float));
        for (i=0; i<nPoints; i++)
        {
            // What is the plane deviation error
            float Ei = -D[i];
            for (j=0; j<3; j++) {
                float Tj_oi = 0.0f;
                for (k=0; k<4; k++) {
                    Tj_oi += T[j][k] * O[i][k];
                }
                Ei += N[i][j] * Tj_oi;
            }
//            printf("E[%d] %f\n", i, Ei);
        
            // Figure out contribution to the error
            for (j=0; j<3; j++) {
                for (k=0; k<4; k++) {
                    de_dT[j][k] += N[i][j] * O[i][k] * Ei;
                }
            }
            
            errorSq += Ei*Ei;
        }

        printf("%d %f %f %f %f\n", iter, T[0][3], T[1][3], T[2][3], sqrt(errorSq));

        // Constrain the gradient (such that dot products are zero)
        if (constrain)
        {
            float T0T1 = 0.0, T1T2 = 0.0, T2T0 = 0.0;
            for (k=0; k<3; k++) {
                T0T1 += T[0][k] * T[1][k];
                T1T2 += T[1][k] * T[2][k];
                T2T0 += T[2][k] * T[0][k];
            }
//            printf("T0T1 %f T1T2 %f T2T0 %f\n", T0T1, T1T2, T2T0);
            for (k=0; k<3; k++) {
                de_dT[0][k] += ORTHOG_PENALTY * 2.0 * T0T1 * T[1][k];
                de_dT[0][k] += ORTHOG_PENALTY * 2.0 * T2T0 * T[2][k];
                de_dT[1][k] += ORTHOG_PENALTY * 2.0 * T1T2 * T[2][k];
                de_dT[1][k] += ORTHOG_PENALTY * 2.0 * T0T1 * T[0][k];
                de_dT[2][k] += ORTHOG_PENALTY * 2.0 * T1T2 * T[1][k];
                de_dT[2][k] += ORTHOG_PENALTY * 2.0 * T2T0 * T[0][k];
            }
        }

        // Calculate the gradient dot product
        //  (used by conjugate gradient method)
        prevGradDot = gradDot;
        gradDot = 0.0;
        for (j=0; j<3; j++) {
            for (k=0; k<4; k++) {
                gradDot += de_dT[j][k] * de_dT[j][k];
            }
        }

//        printf("Iter %d error %f gradDot %f prevGradDot %f\n", iter, sqrt(errorSq), gradDot, prevGradDot);

        //----------------------------------
        // Calculate the conjugate direction
        //----------------------------------
//        if (iter==0) {
            // First iteration, just use the gradient
            for (j=0; j<3; j++) {
                for (k=0; k<4; k++) {
                    conj[j][k] = -de_dT[j][k];
                }
            }
/*        } else {
            // Calculate "beta" for Fletcher Reeves method
            float beta = gradDot / prevGradDot;
//printf("gradDot %f prevGradDot %f beta %f\n", gradDot, prevGradDot, beta);

            // Update the conjugate
            for (j=0; j<3; j++) {
                for (k=0; k<4; k++) {
                    conj[j][k] = beta*conj[j][k] - de_dT[j][k];
                }
            }            
        }
*/

//        PRINT_MAT(de_dT,4,4);
//        exit(1);

        //----------------------------------
        // How large is the gradient ?
        //----------------------------------
        
        double gradSizeRot = 0.0;
        double gradSizePos = 0.0;
        for (j=0; j<3; j++) {
            for (k=0; k<3; k++) {
                gradSizeRot += _ABS(conj[j][k]);
            }
            gradSizePos += _ABS(conj[j][k]);
        }
        if (gradSizeRot <= TOO_SMALL && gradSizePos <= TOO_SMALL) { break; }  // Quit, we've totally converged
        
        //----------------------------------
        // Descend in the gradient direction
        //----------------------------------
        if (gradSizeRot > TOO_SMALL) {
            float scaleRot = stepSizeRot / gradSizeRot;
            for (j=0; j<3; j++) {
                for (k=0; k<3; k++) {
                    T[j][k] += scaleRot * conj[j][k];
                }
            }
            stepSizeRot *= falloff;
        }
        
        if (gradSizePos > TOO_SMALL) {
            float scalePos = stepSizePos / gradSizePos;
            for (j=0; j<3; j++) {
                T[j][3] += scalePos * conj[j][3];
            }
            stepSizePos *= falloff;
        }
        
        // Constrain the gradient (such that scaling is one)
        if (constrain)
        {
            // Measure the scales
            float len[3] = {0.0, 0.0, 0.0};
            for (j=0; j<3; j++) {
                double lenSq = 0.0;
                for (k=0; k<3; k++) { lenSq += (double)T[j][k] * (double)T[j][k]; }
                len[j] = sqrt(lenSq);
            }
            
            // How far off is the scale?
            float xzLen = 0.5 * (len[0] + len[2]);
            if (xzLen > TOO_SMALL) {
                float inv_xzLen = 1.0 / xzLen;
                for (j=0; j<3; j++) {
                    T[3][j] *= inv_xzLen;
                }
            }
            
            // Rescale the thing
            for (j=0; j<3; j++)
            {
                if (len[j] > TOO_SMALL) {
                    float inv_len = 1.0 / len[j];
                    for (k=0; k<3; k++) { T[j][k] *= inv_len; }
                }
            }
        }
    }

}

int main()
{
    int i,j,k;
    float Tcalc[4][4];
    float O[MAX_POINTS][4];
    float N[MAX_POINTS][3];
    float D[MAX_POINTS];
    int nPoints = 0;
    
    // Read the hmd points
    ReadHmdPoints();
    
    //-------------------------
    // Read the lighthouse data
    //-------------------------
    FILE *fin = fopen("ptinfo.csv", "r");
    if (fin==NULL) { printf("ERROR: could not open ptinfo.csv for reading\n"); exit(1); }
    while (!feof(fin))
    {
        // Read the angle
        int sen,lh,axis,count;
        float angle, avglen, stddevang, stddevlen;
        float max_outlier_length, max_outlier_angle;
        int rt = fscanf( fin, "%d %d %d %d %f %f %f %f %f %f\n",
                        &sen, &lh, &axis, &count,
                        &angle, &avglen, &stddevang, &stddevlen,
                        &max_outlier_length, &max_outlier_angle);
        if (rt != 10) { break; }
        
        if (lh == LH_ID && sen < NUM_HMD) {
            // Set the offset
            O[nPoints][0] = hmd_pos[sen][0];
            O[nPoints][1] = hmd_pos[sen][1];
            O[nPoints][2] = hmd_pos[sen][2];
            O[nPoints][3] = 1.0;
        
            // Calculate the plane equation
            if (axis == 1) {   // Horizontal
                N[nPoints][0] = -cos(angle);
                N[nPoints][1] = -sin(angle);
                N[nPoints][2] = 0.0;
                D[nPoints]    = 0.0;
            } else {           // Vertical
                N[nPoints][0] = 0.0;
                N[nPoints][1] = -sin(angle);
                N[nPoints][2] =  cos(angle);
                D[nPoints]    = 0.0;
            }
            
            printf("pt %d O %.3f %.3f %.3f %.3f  N %.3f %.3f %.3f  D %.3f\n",
                nPoints,
                O[nPoints][0], O[nPoints][1], O[nPoints][2], O[nPoints][3], 
                N[nPoints][0], N[nPoints][1], N[nPoints][2], 
                D[nPoints]);
        
            nPoints++;
        }
    }
    fclose(fin);
    
    printf("nPoints %d\n", nPoints);
        
    // Run the calculation for Tcalc
    int run;
    //for (run=0; run<100; run++) {

        // Initialize Tcalc to the identity matrix
        //memcpy(Tcalc, Torig, 4*4*sizeof(float));
        memset(Tcalc, 0, 4*4*sizeof(float));
        for (i=0; i<4; i++) { Tcalc[i][i] = 1.0f; }

        // Solve it!
        AffineSolve(
            Tcalc,           // OUTPUT: transform
            O,  // INPUT:  points, offsets
            N,  // INPUT:  plane normals
            D,     // INPUT:  plane offsets
            nPoints, NITER,
            STEP_SIZE_ROT, STEP_SIZE_POS, FALLOFF,
            1);
    //}
    
    PRINT_MAT(Tcalc,4,4);

    
    // insert code here...
    printf("Hello, World!\n");
    return 0;
}
