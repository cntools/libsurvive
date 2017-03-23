//
//  main.c
//  Aff
//  Created by user on 3/2/17.
//  Copyright © 2017 user. All rights reserved.
//

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "dclapack.h"
#include <linmath.h>
#define RegisterDriver(a,b)
#include "poser_daveortho.c"
#define LH_ID    0
#define NUM_HMD 32
#define INDIR "full_test_triangle_on_floor/"

#define MAX_POINTS SENSORS_PER_OBJECT
//#define _ABS(a)  ( (a)<=0 ? -(a) : (a) )
#define _SIGN(a) ( (a)<=0 ? -1.0f : 1.0f )
#define RANDF  ( (float)rand() / (float)RAND_MAX )
#define PI 3.14159265358979323846264

#define STEP_SIZE_ROT 1.0
#define STEP_SIZE_POS 1.0
#define FALLOFF   0.99999
#define NITER     2000000
#define TOO_SMALL 0.0001
#define ORTHOG_PENALTY   1.0

float hmd_pos[NUM_HMD][3];
void ReadHmdPoints()
{
    int i;
    FILE *fin = fopen(INDIR "HMD_points.csv","r");
    if (fin==NULL) {
        printf("ERROR: could not open HMD_points.csv for reading\n");
        exit(1);
    }
    
    for (i=0; i<NUM_HMD; i++) {
        fscanf(fin, "%f %f %f", &(hmd_pos[i][0]), &(hmd_pos[i][1]), &(hmd_pos[i][2]));
    }
    
    fclose(fin);
}

float hmd_angle[NUM_HMD][2];
void ReadPtinfo()
{
    // Initialize to -9999
    int i;
    for (i=0; i<NUM_HMD; i++) { hmd_angle[i][0]=-9999.0; hmd_angle[i][1]=-9999.0; }

    // Read ptinfo.csv
    FILE *fin = fopen(INDIR "ptinfo.csv", "r");
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
        
        // If it's valid, store in the result
        if (lh == LH_ID && sen < NUM_HMD) {
            hmd_angle[sen][axis] = angle;
        }
    }
    fclose(fin);
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

//        printf("%d %f %f %f %f\n", iter, T[0][3], T[1][3], T[2][3], sqrt(errorSq));
//exit(1);
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
    float dist = sqrt(T[0][3]*T[0][3] + T[1][3]*T[1][3] + T[2][3]*T[2][3]);
    printf("AffineSolve: pos: %f %f %f dist: %f\n", T[0][3], T[1][3], T[2][3], dist);
}

int main()
{
    int i,j,k,sen,axis;
        
    // Read the data files
    ReadHmdPoints();
    ReadPtinfo();

    //-------------------------
    // Package the lighthouse data for "AffineSolve"
    //-------------------------

    // Data for the "iterative" affine solve formula
    float Tcalc[4][4];
    float O[MAX_POINTS][4];
    float N[MAX_POINTS][3];
    float D[MAX_POINTS];
    int nPlanes = 0;

    for (sen=0; sen<NUM_HMD; sen++)
    {
        for (axis=0; axis<2; axis++)
        {
            if (hmd_angle[sen][axis] != -9999.0)
            {
                // Set the offset
                O[nPlanes][0] = hmd_pos[sen][0];
                O[nPlanes][1] = hmd_pos[sen][1];
                O[nPlanes][2] = hmd_pos[sen][2];
                O[nPlanes][3] = 1.0;
        
                // Calculate the plane equation
                if (axis == 0) {   // Horizontal
                    N[nPlanes][0] = -cos(hmd_angle[sen][axis]);
                    N[nPlanes][1] = -sin(hmd_angle[sen][axis]);
                    N[nPlanes][2] = 0.0;
                    D[nPlanes]    = 0.0;
                } else {           // Vertical
                    N[nPlanes][0] = 0.0;
                    N[nPlanes][1] = -sin(hmd_angle[sen][axis]);
                    N[nPlanes][2] =  cos(hmd_angle[sen][axis]);
                    D[nPlanes]    = 0.0;
                }

                printf("plane %d O %.3f %.3f %.3f %.3f  N %.3f %.3f %.3f  D %.3f\n",
                    nPlanes,
                    O[nPlanes][0], O[nPlanes][1], O[nPlanes][2], O[nPlanes][3], 
                    N[nPlanes][0], N[nPlanes][1], N[nPlanes][2], 
                    D[nPlanes]);
                nPlanes++;
            }
        }
    }

    
    printf("nPlanes %d\n", nPlanes);
        
    //}
    
    PRINT_MAT(Tcalc,4,4);

    
    //--------------------------------------------------
    // Package the data for "OrthoSolve"
    //--------------------------------------------------

    // Data for the "fake" ortho solve formula
    float Tortho[4][4];         // OUTPUT: 4x4 transformation matrix
    FLOAT S_out[2][MAX_POINTS];  // INPUT:  array of screenspace points
    FLOAT S_in[2][MAX_POINTS];  // INPUT:  array of screenspace points
    FLOAT X_in[3][MAX_POINTS];  // INPUT:  array of offsets
    int nPoints=0;  

    // Transform into the "OrthoSolve" format
    for (sen=0; sen<NUM_HMD; sen++)
    {
        if (hmd_angle[sen][0] != -9999.0 && hmd_angle[sen][1] != -9999.0)
        {
            S_in[0][nPoints] = hmd_angle[sen][0];
            S_in[1][nPoints] = hmd_angle[sen][1];
            X_in[0][nPoints] = hmd_pos[sen][0];
            X_in[1][nPoints] = hmd_pos[sen][1];
            X_in[2][nPoints] = hmd_pos[sen][2];
            nPoints++;
        }
    }
    printf("OrthoSolve nPoints %d\n", nPoints);
    
    //--------------------------------------------------
    // Run the "OrthoSolve" and then the "AffineSolve"
    //--------------------------------------------------    
    
    int loop;
    for (loop=0; loop<1; loop++)
    {
        // Run OrthoSolve
        OrthoSolve(
            Tortho,     // OUTPUT: 4x4 transformation matrix
            S_out,      // OUTPUT: array of output screenspace points
            S_in,       // INPUT:  array of screenspace points
            X_in,       // INPUT:  array of offsets
            nPoints);
			printf( "POS: %f %f %f\n", Tortho[0][3], Tortho[1][3], Tortho[2][3]);

		// Come up with rotation and transposed version of Tortho
		FLT TorthoTr[4][4], Rortho[4][4], RorthoTr[4][4];
		TRANSP(Tortho,TorthoTr,4,4);
		memcpy(Rortho,Tortho,4*4*sizeof(FLT));
		Rortho[3][0]=0.0;  Rortho[3][1]=0.0;  Rortho[3][2]=0.0;
		TRANSP(Rortho,RorthoTr,4,4);
		PRINT(Tortho,4,4);
		PRINT(Rortho,4,4);

		// Print out some quaternions
		FLT Tquat[4], TquatTr[4], Rquat[4], RquatTr[4];
		quatfrommatrix(Tquat,   &Tortho[0][0] );
		quatfrommatrix(TquatTr, &TorthoTr[0][0] );
		quatfrommatrix(Rquat,   &Rortho[0][0] );
		quatfrommatrix(RquatTr, &RorthoTr[0][0] );
		printf( "Tquat  : %f %f %f %f = %f\n", Tquat  [0], Tquat  [1], Tquat  [2], Tquat  [3], quatmagnitude(Tquat));
		printf( "TquatTr: %f %f %f %f = %f\n", TquatTr[0], TquatTr[1], TquatTr[2], TquatTr[3], quatmagnitude(TquatTr));
		printf( "Rquat  : %f %f %f %f = %f\n", Rquat  [0], Rquat  [1], Rquat  [2], Rquat  [3], quatmagnitude(Rquat));
		printf( "RquatTr: %f %f %f %f = %f\n", RquatTr[0], RquatTr[1], RquatTr[2], RquatTr[3], quatmagnitude(RquatTr));

		// Flip y and z axies
		FLT T2[4][4] = {
			{ Tortho[0][0], -Tortho[0][2], -Tortho[0][1], 0.0 },
			{ Tortho[1][0], -Tortho[1][2], -Tortho[1][1], 0.0 },
			{ Tortho[2][0], -Tortho[2][2], -Tortho[2][1], 0.0 },
			{          0.0,           0.0,           0.0, 1.0 } };
PRINT(T2,4,4);

		// Print out the quaternions
		FLT T2quat[4];
		quatfrommatrix(T2quat,   &T2[0][0] );
		printf( "T2quat : %f %f %f %f = %f\n", T2quat [0], T2quat [1], T2quat [2], T2quat [3], quatmagnitude(T2quat));
    }

    // Run the calculation for Tcalc
    //int run;
    //for (run=0; run<100; run++) {
/*
    // Initialize Tcalc to the identity matrix
    memcpy(Tcalc, Tortho, 4*4*sizeof(float));
    //memset(Tcalc, 0, 4*4*sizeof(float));
    //for (i=0; i<4; i++) { Tcalc[i][i] = 1.0f; }

    // Solve it!
    AffineSolve(
        Tcalc,           // OUTPUT: transform
        O,  // INPUT:  points, offsets
        N,  // INPUT:  plane normals
        D,     // INPUT:  plane offsets
        nPlanes, NITER,
        STEP_SIZE_ROT, STEP_SIZE_POS, FALLOFF,
        1);
*/
    // insert code here...
    return 0;
}





#if 0
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

#define CrossProduct(ox,oy,oz,a,b,c,x,y,z) { \
    ox=(b)*(z)-(c)*(y); \
    oy=(c)*(x)-(a)*(z); \
    oz=(a)*(y)-(b)*(x); }

void OrthoSolve(
    float T[4][4],               // OUTPUT: 4x4 transformation matrix
    FLOAT S_out[2][MAX_POINTS],  // OUTPUT:  array of screenspace points
    FLOAT S_in[2][MAX_POINTS],   // INPUT:  array of screenspace points
    FLOAT X_in[3][MAX_POINTS],   // INPUT:  array of offsets
    int nPoints)
{
    int i,j,k;
    FLOAT R[3][3];           // OUTPUT: 3x3 rotation matrix
    FLOAT trans[3];          // INPUT:  x,y,z translation vector

    //--------------------
    // Remove the center of the HMD offsets, and the screen space
    //--------------------
    FLOAT xbar[3] = {0.0, 0.0, 0.0};
    FLOAT sbar[2] = {0.0, 0.0};
    FLOAT S[2][MAX_POINTS];
    FLOAT X[3][MAX_POINTS];
    FLOAT inv_nPoints = 1.0 / nPoints;
    for (i=0; i<nPoints; i++) {
        xbar[0] += X_in[0][i];
        xbar[1] += X_in[1][i];
        xbar[2] += X_in[2][i];
        sbar[0] += S_in[0][i];
        sbar[1] += S_in[1][i];        
    }
    for (j=0; j<3; j++) { xbar[j] *= inv_nPoints; }
    for (j=0; j<2; j++) { sbar[j] *= inv_nPoints; }
    for (i=0; i<nPoints; i++) {
        X[0][i] = X_in[0][i] - xbar[0];
        X[1][i] = X_in[1][i] - xbar[1];
        X[2][i] = X_in[2][i] - xbar[2];
        S[0][i] = S_in[0][i] - sbar[0];
        S[1][i] = S_in[1][i] - sbar[1];
    }
    
    //--------------------
    // Solve for the morph matrix
    //  S = M X
    // thus
    // (SX^t)(XX^t)^-1 = M
    //--------------------
    FLOAT Xt[MAX_POINTS][3];
    FLOAT XXt[3][3];
    FLOAT invXXt[3][3];
    FLOAT SXt[2][3];
    FLOAT M[2][3];           // Morph matrix! (2 by 3)
    TRANSP(X,Xt,3,nPoints);
    MUL(X,Xt,XXt,3,nPoints,3);
    MUL(S,Xt,SXt,2,nPoints,3);
    INV(XXt,invXXt,3);
    MUL(SXt,invXXt,M,2,3,3);
//PRINT(M,2,3);

// Double checking work
FLOAT S_morph[2][MAX_POINTS];
MUL(M,X,S_morph,2,3,nPoints);
for (i=0; i<nPoints; i++) { S_morph[0][i]+=sbar[0]; S_morph[1][i]+=sbar[1]; }

    //--------------------
    // Solve for the non-trivial vector
    //  uf -- vector that goes into the camera
    //--------------------
    FLOAT uM[3][3] = {
        { M[0][0], M[0][1], M[0][2] },
        { M[1][0], M[1][1], M[1][2] },
        { 3.14567, -1.2345, 4.32567 } };      // Morph matrix with appended row
//PRINT(uM,3,3);
// ToDo: Pick a number for the bottom that is NOT linearly separable with M[0] and M[1]
    FLOAT B[3][1] = { {0.0}, {0.0}, {1.0} };
    FLOAT inv_uM[3][3];
    FLOAT uf[3][1];
    INV(uM,inv_uM,3);
    MUL(inv_uM,B,uf,3,3,1);
    
    //--------------------
    // Solve for unit length vector
    //  f that goes into the camera
    //--------------------
    FLOAT uf_len = sqrt( uf[0][0]*uf[0][0] + uf[1][0]*uf[1][0] + uf[2][0]*uf[2][0] );
    FLOAT f[3][1] = { {uf[0][0]/uf_len}, {uf[1][0]/uf_len}, {uf[2][0]/uf_len} };
//PRINT(uf,3,1);
//PRINT(f,3,1);

//FLOAT check[3][1];
//MUL(uM,uf,check,3,3,1);
//PRINT(check,3,1);

    //--------------------
    // take cross products to get vectors u,r
    //--------------------
    FLOAT u[3][1], r[3][1];
    CrossProduct(u[0][0],u[1][0],u[2][0],f[0][0],f[1][0],f[2][0],1.0,0.0,0.0);
    FLOAT inv_ulen = 1.0 / sqrt( u[0][0]*u[0][0] + u[1][0]*u[1][0] + u[2][0]*u[2][0] );
    u[0][0]*=inv_ulen; u[1][0]*=inv_ulen; u[2][0]*=inv_ulen;
    CrossProduct(r[0][0],r[1][0],r[2][0],f[0][0],f[1][0],f[2][0],u[0][0],u[1][0],u[2][0]);
//PRINT(u,3,1);
//PRINT(r,3,1);

    //--------------------
    // Use morph matrix to get screen space
    //  uhat,rhat
    //--------------------
    FLOAT uhat[2][1], rhat[2][1], fhat[2][1];
    MUL(M,f,fhat,2,3,1);
    MUL(M,u,uhat,2,3,1);
    MUL(M,r,rhat,2,3,1);
    FLOAT fhat_len = sqrt( fhat[0][0]*fhat[0][0] + fhat[1][0]*fhat[1][0] );
    FLOAT uhat_len = sqrt( uhat[0][0]*uhat[0][0] + uhat[1][0]*uhat[1][0] );
    FLOAT rhat_len = sqrt( rhat[0][0]*rhat[0][0] + rhat[1][0]*rhat[1][0] );
    FLOAT urhat_len = 0.5 * (uhat_len + rhat_len);
/*    
printf("fhat %f %f (len %f)\n", fhat[0][0], fhat[1][0], fhat_len);
printf("uhat %f %f (len %f)\n", uhat[0][0], uhat[1][0], uhat_len);
printf("rhat %f %f (len %f)\n", rhat[0][0], rhat[1][0], rhat_len);
*/
//    FLOAT ydist1 = 1.0 /  uhat_len; //0.25*PI / uhat_len;
//    FLOAT ydist2 = 1.0 /  rhat_len; //0.25*PI / rhat_len;
    FLOAT ydist  = 1.0 / urhat_len;
    //printf("ydist1 %f ydist2 %f ydist %f\n", ydist1, ydist2, ydist);

    //--------------------
    // Rescale the axies to be of the proper length
    //--------------------
    FLOAT x[3][1] = { {M[0][0]*ydist}, {0.0}, {M[1][0]*ydist} };
    FLOAT y[3][1] = { {M[0][1]*ydist}, {0.0}, {M[1][1]*ydist} };
    FLOAT z[3][1] = { {M[0][2]*ydist}, {0.0}, {M[1][2]*ydist} };

    // we know the distance into (or out of) the camera for the z axis,
    //  but we don't know which direction . . .
    FLOAT x_y = sqrt(1.0 - x[0][0]*x[0][0] - x[2][0]*x[2][0]);
    FLOAT y_y = sqrt(1.0 - y[0][0]*y[0][0] - y[2][0]*y[2][0]);
    FLOAT z_y = sqrt(1.0 - z[0][0]*z[0][0] - z[2][0]*z[2][0]);

	if( x_y != x_y ) x_y = 0;
	if( y_y != y_y ) y_y = 0;
	if( z_y != z_y ) z_y = 0;

/*
    // Exhaustively flip the minus sign of the z axis until we find the right one . . .
    FLOAT bestErr = 9999.0;
    FLOAT xy_dot2 = x[0][0]*y[0][0] + x[2][0]*y[2][0];
    FLOAT yz_dot2 = y[0][0]*z[0][0] + y[2][0]*z[2][0];
    FLOAT zx_dot2 = z[0][0]*x[0][0] + z[2][0]*x[2][0];
    for (i=0;i<2;i++) {
        for (j=0;j<2;j++) {
            for(k=0;k<2;k++) {
            
                // Calculate the error term
                FLOAT xy_dot = xy_dot2 + x_y*y_y;
                FLOAT yz_dot = yz_dot2 + y_y*z_y;
                FLOAT zx_dot = zx_dot2 + z_y*x_y;
                FLOAT err = _ABS(xy_dot) + _ABS(yz_dot) + _ABS(zx_dot);
                
                // Calculate the handedness
                FLOAT cx,cy,cz;
                CrossProduct(cx,cy,cz,x[0][0],x_y,x[2][0],y[0][0],y_y,y[2][0]);
                FLOAT hand = cx*z[0][0] + cy*z_y + cz*z[2][0];
                printf("err %f hand %f\n", err, hand);
                
                // If we are the best right-handed frame so far
                //if (hand > 0 && err < bestErr) { x[1][0]=x_y; y[1][0]=y_y; z[1][0]=z_y; bestErr=err; }
				if ( i == 0 && j == 1 && k == 0) { x[1][0]=x_y; y[1][0]=y_y; z[1][0]=z_y; bestErr=err; }
                z_y = -z_y;
            }
            y_y = -y_y;
        }
        x_y = -x_y;
    }
    printf("bestErr %f\n", bestErr);
*/

    //-------------------------
    // A test version of the rescaling to the proper length
    //-------------------------
    FLOAT ydist2;
    FLOAT bestBestErr = 9999.0;
    FLOAT bestYdist = 0;
    for (ydist2=ydist-0.1; ydist2<ydist+0.1; ydist2+=0.0001)
    {
        FLOAT x2[3][1] = { {M[0][0]*ydist2}, {0.0}, {M[1][0]*ydist2} };
        FLOAT y2[3][1] = { {M[0][1]*ydist2}, {0.0}, {M[1][1]*ydist2} };
        FLOAT z2[3][1] = { {M[0][2]*ydist2}, {0.0}, {M[1][2]*ydist2} };

        // we know the distance into (or out of) the camera for the z axis,
        //  but we don't know which direction . . .
        FLOAT x_y = sqrt(1.0 - x2[0][0]*x2[0][0] - x2[2][0]*x2[2][0]);
        FLOAT y_y = sqrt(1.0 - y2[0][0]*y2[0][0] - y2[2][0]*y2[2][0]);
        FLOAT z_y = sqrt(1.0 - z2[0][0]*z2[0][0] - z2[2][0]*z2[2][0]);

        // Exhaustively flip the minus sign of the z axis until we find the right one . . .
        FLOAT bestErr = 9999.0;
        FLOAT xy_dot2 = x2[0][0]*y2[0][0] + x2[2][0]*y2[2][0];
        FLOAT yz_dot2 = y2[0][0]*z2[0][0] + y2[2][0]*z2[2][0];
        FLOAT zx_dot2 = z2[0][0]*x2[0][0] + z2[2][0]*x2[2][0];
        for (i=0;i<2;i++) {
            for (j=0;j<2;j++) {
                for(k=0;k<2;k++) {
            
                    // Calculate the error term
                    FLOAT xy_dot = xy_dot2 + x_y*y_y;
                    FLOAT yz_dot = yz_dot2 + y_y*z_y;
                    FLOAT zx_dot = zx_dot2 + z_y*x_y;
                    FLOAT err = _ABS(xy_dot) + _ABS(yz_dot) + _ABS(zx_dot);
                
                    // Calculate the handedness
                    FLOAT cx,cy,cz;
                    CrossProduct(cx,cy,cz,x2[0][0],x_y,x2[2][0],y2[0][0],y_y,y2[2][0]);
                    FLOAT hand = cx*z2[0][0] + cy*z_y + cz*z2[2][0];
//                  printf("err %f hand %f\n", err, hand);
                
                    // If we are the best right-handed frame so far
                    if (hand > 0 && err < bestErr) { x2[1][0]=x_y; y2[1][0]=y_y; z2[1][0]=z_y; bestErr=err; }
                    z_y = -z_y;
                }
                y_y = -y_y;
            }
            x_y = -x_y;
        }
        printf("ydist2 %f bestErr %f\n",ydist2,bestErr);
        
        if (bestErr < bestBestErr) {
            memcpy(x,x2,3*sizeof(FLOAT));
            memcpy(y,y2,3*sizeof(FLOAT));
            memcpy(z,z2,3*sizeof(FLOAT));
            bestBestErr = bestErr;
            bestYdist = ydist2;
        }
    }
    ydist = bestYdist;

/*
    for (i=0; i<nPoints; i++) {
        float x1 = x[0][0]*X[0][i] + y[0][0]*X[1][i] + z[0][0]*X[2][i];
        float y1 = x[1][0]*X[0][i] + y[1][0]*X[1][i] + z[1][0]*X[2][i];
        float z1 = x[2][0]*X[0][i] + y[2][0]*X[1][i] + z[2][0]*X[2][i];
        printf("x1z1 %f %f y1 %f\n", x1, z1, y1);
    }
*/
/*    
    //--------------------
    // Combine uhat and rhat to figure out the unit x-vector
    //--------------------
    FLOAT xhat[2][1]  = { {0.0}, {1.0} };
    FLOAT urhat[2][2] = {
        {uhat[0][0], uhat[1][0]},
        {rhat[0][0], rhat[1][0]} };
    FLOAT inv_urhat[2][2];
    FLOAT ab[2][1];
    INV(urhat,inv_urhat,2);
    MUL(inv_urhat,xhat,ab,2,2,1);
PRINT(ab,2,1);
    FLOAT a = ab[0][0], b = ab[1][0];

    //-------------------
    // calculate the xyz coordinate system
    //-------------------
    FLOAT y[3][1] = { {f[0][0]}, {f[1][0]}, {f[2][0]} };
    FLOAT x[3][1] = { {a*u[0][0] + b*r[0][0]}, {a*u[1][0] + b*r[1][0]}, {a*u[2][0] + b*r[2][0]} };
    FLOAT inv_xlen = 1.0 / sqrt( x[0][0]*x[0][0] + x[1][0]*x[1][0] + x[2][0]*x[2][0] );
    x[0][0]*=inv_xlen; x[1][0]*=inv_xlen; x[2][0]*=inv_xlen;
    FLOAT z[3][1];
    CrossProduct(z[0][0],z[1][0],z[2][0],x[0][0],x[1][0],x[2][0],y[0][0],y[1][0],y[2][0]);
*/
    // Store into the rotation matrix
    for (i=0; i<3; i++) { R[i][0] = x[i][0]; R[i][1] = y[i][0]; R[i][2] = z[i][0]; }
//PRINT(R,3,3);

    //-------------------
    // Calculate the translation of the centroid
    //-------------------
    trans[0]=tan(sbar[0]);  trans[1]=1.0;  trans[2]=tan(sbar[1]);
    FLOAT inv_translen = ydist / sqrt( trans[0]*trans[0] + trans[1]*trans[1] + trans[2]*trans[2] );
    trans[0]*=inv_translen; trans[1]*=inv_translen; trans[2]*=inv_translen;

    //-------------------
    // Add in the centroid point
    //-------------------
    trans[0] -= xbar[0]*R[0][0] + xbar[1]*R[0][1] + xbar[2]*R[0][2];
    trans[1] -= xbar[0]*R[1][0] + xbar[1]*R[1][1] + xbar[2]*R[1][2];
    trans[2] -= xbar[0]*R[2][0] + xbar[1]*R[2][1] + xbar[2]*R[2][2];
    FLOAT transdist = sqrt( trans[0]*trans[0] + trans[1]*trans[1] + trans[2]*trans[2] );

    //-------------------
    // Pack into the 4x4 transformation matrix
    //-------------------
    T[0][0]=R[0][0]; T[0][1]=R[0][1]; T[0][2]=R[0][2]; T[0][3]=trans[0];
    T[1][0]=R[1][0]; T[1][1]=R[1][1]; T[1][2]=R[1][2]; T[1][3]=trans[1];
    T[2][0]=R[2][0]; T[2][1]=R[2][1]; T[2][2]=R[2][2]; T[2][3]=trans[2];
    T[3][0]=0.0;     T[3][1]=0.0;     T[3][2]=0.0;     T[3][3]=1.0;

    PRINT_MAT(T,4,4);
    //-------------------
    // Plot the output points
    //-------------------
    for (i=0; i<nPoints; i++) {
        float Tx = T[0][0]*X_in[0][i] + T[0][1]*X_in[1][i] + T[0][2]*X_in[2][i] + T[0][3];
        float Ty = T[1][0]*X_in[0][i] + T[1][1]*X_in[1][i] + T[1][2]*X_in[2][i] + T[1][3];
        float Tz = T[2][0]*X_in[0][i] + T[2][1]*X_in[1][i] + T[2][2]*X_in[2][i] + T[2][3];
        S_out[0][i] = atan2(Tx, Ty);   // horiz
        S_out[1][i] = atan2(Tz, Ty);   // vert
        //S_out[0][i] = Tx;
        //S_out[1][i] = Tz;
        printf("point %i Txyz %f %f %f in %f %f out %f %f morph %f %f\n", i, Tx,Ty,Tz, S_in[0][i], S_in[1][i], S_out[0][i], S_out[1][i], S_morph[0][i], S_morph[1][i]);
    }




	float quat[4];
	float posoff[3] = { trans[0], trans[1], trans[2] };
	float MT[4][4];
	//matrix44transpose( MT, &T[0][0] );
	matrix44copy( &MT[0][0], &T[0][0] );

//QUAT: 0.657864 -0.305763 0.128486 0.265895 = 0.783252
//QUAT: 0.657864 0.305763 -0.128486 -0.265895 = 0.783252


	quatfrommatrix( quat, &MT[0][0] );
	printf( "QUAT: %f %f %f %f = %f\n", quat[0], quat[1], quat[2], quat[3], quatmagnitude(quat) );
	//quat[2] -= 0.005; //fixes up lh0 in test data set.
	quatnormalize( quat, quat );
	printf( "QUAT: %f %f %f %f = %f\n", quat[0], quat[1], quat[2], quat[3], quatmagnitude(quat) );


	for( i = 0; i <nPoints;i++ )
	{
		float pt[3] = { X_in[0][i], X_in[1][i], X_in[2][i] };
		quatrotatevector( pt, quat, pt );
		add3d( pt, pt, posoff );
		printf( "%f %f %f OUT %f %f %f ANGLE %f %f AOUT %f %f\n", 
			X_in[0][i], X_in[1][i], X_in[2][i], pt[0], pt[1], pt[2],
			S_in[0][i], S_in[1][i], atan2( pt[0], pt[1] ), atan2( pt[2], pt[1] ) );
	}
	quattomatrix( &MT[0][0], quat );
    PRINT_MAT(MT,4,4);



//    printf("xbar %f %f %f\n", xbar[0], xbar[1], xbar[2]);
//    printf("trans %f %f %f dist: %f\n", trans[0], trans[1], trans[2], transdist);
}
#endif
