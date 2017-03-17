#include "survive_cal.h"
#include <math.h>
#include <string.h>
#include "linmath.h"
#include <survive.h>
#include <stdio.h>
#include <stdlib.h>
#include <dclapack.h>
#include <linmath.h>

static int LH_ID;

void OrthoSolve(
    FLT T[4][4],               // OUTPUT: 4x4 transformation matrix
    FLT S_out[2][SENSORS_PER_OBJECT],  // OUTPUT:  array of screenspace points
    FLT S_in[2][SENSORS_PER_OBJECT],   // INPUT:  array of screenspace points
    FLT X_in[3][SENSORS_PER_OBJECT],   // INPUT:  array of offsets
    int nPoints);


typedef struct
{
	int something;
	//Stuff
} DummyData;

int PoserDaveOrtho( SurviveObject * so, PoserData * pd )
{
	PoserType pt = pd->pt;
	SurviveContext * ctx = so->ctx;
	DummyData * dd = so->PoserData;

	if( !dd ) so->PoserData = dd = malloc( sizeof( DummyData ) );

	switch( pt )
	{
	case POSERDATA_IMU:
	{
		PoserDataIMU * imu = (PoserDataIMU*)pd;
		//printf( "IMU:%s (%f %f %f) (%f %f %f)\n", so->codename, imu->accel[0], imu->accel[1], imu->accel[2], imu->gyro[0], imu->gyro[1], imu->gyro[2] );
		break;
	}
	case POSERDATA_LIGHT:
	{
		PoserDataLight * l = (PoserDataLight*)pd;
		//printf( "LIG:%s %d @ %f rad, %f s (AC %d) (TC %d)\n", so->codename, l->sensor_id, l->angle, l->length, l->acode, l->timecode );
		break;
	}
	case POSERDATA_FULL_SCENE:
	{
		PoserDataFullScene * fs = (PoserDataFullScene*)pd;

		for( LH_ID = 0; LH_ID < 2; LH_ID++ )
		{
			int i;
			int max_hits = 0;
			FLT S_in[2][SENSORS_PER_OBJECT];
			FLT X_in[3][SENSORS_PER_OBJECT];
			for( i = 0; i < SENSORS_PER_OBJECT; i++ )
			{
				//Load all our valid points into something the LHFinder can use.
				if( fs->lengths[i][LH_ID][0] > 0 )
				{
					S_in[0][max_hits] = fs->angles[i][LH_ID][0];
					S_in[1][max_hits] = fs->angles[i][LH_ID][1];
					X_in[0][max_hits] = so->sensor_locations[i*3+0];
					X_in[1][max_hits] = so->sensor_locations[i*3+1];
					X_in[2][max_hits] = so->sensor_locations[i*3+2];
					max_hits++;
				}

			}
			FLT tOut[4][4];
			FLT S_out[2][SENSORS_PER_OBJECT];
			OrthoSolve( tOut, S_out, S_in, X_in, max_hits );

						//Now, we need to solve where we are as a function of where
			//the lighthouses are.
			FLT quat[4];
			FLT posoff[3] = { tOut[0][3], tOut[1][3], tOut[2][3] };
			FLT MT[4][4];

			//matrix44transpose( MT, &tOut[0][0] );
			matrix44copy( &MT[0][0], &tOut[0][0] );

			quatfrommatrix( quat, &MT[0][0] );


			//printf( "QUAT: %f %f %f %f = %f\n", quat[0], quat[1], quat[2], quat[3], quatmagnitude(quat) );
			//quat[2] -= 0.005; //fixes up lh0 in test data set.
			quatnormalize( quat, quat );
			printf( "QUAT: %f %f %f %f = %f [%f %f %f]\n", quat[0], quat[1], quat[2], quat[3], quatmagnitude(quat), posoff[0], posoff[1], posoff[2] );
			
			for( i = 0; i < max_hits;i++ )
			{
				FLT pt[3] = { X_in[0][i], X_in[1][i], X_in[2][i] };
				quatrotatevector( pt, quat, pt );
				add3d( pt, pt, posoff );
				printf( "OUT %f %f %f ANGLE %f %f AOUT %f %f\n", 
					pt[0], pt[1], pt[2],
					S_in[0][i], S_in[1][i], atan2( pt[0], pt[1] ), atan2( pt[2], pt[1] ) );
			}

			so->FromLHPose[LH_ID].Pos[0] = posoff[0];
			so->FromLHPose[LH_ID].Pos[1] = posoff[1];
			so->FromLHPose[LH_ID].Pos[2] = posoff[2];
			so->FromLHPose[LH_ID].Rot[0] = quat[0];
			so->FromLHPose[LH_ID].Rot[1] = quat[1];
			so->FromLHPose[LH_ID].Rot[2] = quat[2];
			so->FromLHPose[LH_ID].Rot[3] = quat[3];
		}

		break;
	}
	case POSERDATA_DISASSOCIATE:
	{
		free( dd );
		so->PoserData = 0;
		//printf( "Need to disassociate.\n" );
		break;
	}
	}
	return 0;
}


REGISTER_LINKTIME( PoserDaveOrtho );





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
    FLT T[4][4],               // OUTPUT: 4x4 transformation matrix
    FLT S_out[2][SENSORS_PER_OBJECT],  // OUTPUT:  array of screenspace points
    FLT S_in[2][SENSORS_PER_OBJECT],   // INPUT:  array of screenspace points
    FLT X_in[3][SENSORS_PER_OBJECT],   // INPUT:  array of offsets
    int nPoints)
{
    int i,j,k;
    FLT R[3][3];           // OUTPUT: 3x3 rotation matrix
    FLT trans[3];          // INPUT:  x,y,z translation vector

    //--------------------
    // Remove the center of the HMD offsets, and the screen space
    //--------------------
    FLT xbar[3] = {0.0, 0.0, 0.0};
    FLT sbar[2] = {0.0, 0.0};
    FLT S[2][SENSORS_PER_OBJECT];
    FLT X[3][SENSORS_PER_OBJECT];
    FLT inv_nPoints = 1.0 / nPoints;
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
    FLT Xt[SENSORS_PER_OBJECT][3];
    FLT XXt[3][3];
    FLT invXXt[3][3];
    FLT SXt[2][3];
    FLT M[2][3];           // Morph matrix! (2 by 3)
    TRANSP(X,Xt,3,nPoints);
    MUL(X,Xt,XXt,3,nPoints,3);
    MUL(S,Xt,SXt,2,nPoints,3);
    INV(XXt,invXXt,3);
    MUL(SXt,invXXt,M,2,3,3);
//PRINT(M,2,3);

// Double checking work
FLT S_morph[2][SENSORS_PER_OBJECT];
MUL(M,X,S_morph,2,3,nPoints);
for (i=0; i<nPoints; i++) { S_morph[0][i]+=sbar[0]; S_morph[1][i]+=sbar[1]; }

    //--------------------
    // Solve for the non-trivial vector
    //  uf -- vector that goes into the camera
    //--------------------
    FLT uM[3][3] = {
        { M[0][0], M[0][1], M[0][2] },
        { M[1][0], M[1][1], M[1][2] },
        { 3.14567, -1.2345, 4.32567 } };      // Morph matrix with appended row
//PRINT(uM,3,3);
// ToDo: Pick a number for the bottom that is NOT linearly separable with M[0] and M[1]
    FLT B[3][1] = { {0.0}, {0.0}, {1.0} };
    FLT inv_uM[3][3];
    FLT uf[3][1];
    INV(uM,inv_uM,3);
    MUL(inv_uM,B,uf,3,3,1);
    
    //--------------------
    // Solve for unit length vector
    //  f that goes into the camera
    //--------------------
    FLT uf_len = sqrt( uf[0][0]*uf[0][0] + uf[1][0]*uf[1][0] + uf[2][0]*uf[2][0] );
    FLT f[3][1] = { {uf[0][0]/uf_len}, {uf[1][0]/uf_len}, {uf[2][0]/uf_len} };
//PRINT(uf,3,1);
//PRINT(f,3,1);

//FLT check[3][1];
//MUL(uM,uf,check,3,3,1);
//PRINT(check,3,1);

    //--------------------
    // take cross products to get vectors u,r
    //--------------------
    FLT u[3][1], r[3][1];
    CrossProduct(u[0][0],u[1][0],u[2][0],f[0][0],f[1][0],f[2][0],1.0,0.0,0.0);
    FLT inv_ulen = 1.0 / sqrt( u[0][0]*u[0][0] + u[1][0]*u[1][0] + u[2][0]*u[2][0] );
    u[0][0]*=inv_ulen; u[1][0]*=inv_ulen; u[2][0]*=inv_ulen;
    CrossProduct(r[0][0],r[1][0],r[2][0],f[0][0],f[1][0],f[2][0],u[0][0],u[1][0],u[2][0]);
//PRINT(u,3,1);
//PRINT(r,3,1);

    //--------------------
    // Use morph matrix to get screen space
    //  uhat,rhat
    //--------------------
    FLT uhat[2][1], rhat[2][1], fhat[2][1];
    MUL(M,f,fhat,2,3,1);
    MUL(M,u,uhat,2,3,1);
    MUL(M,r,rhat,2,3,1);
    FLT fhat_len = sqrt( fhat[0][0]*fhat[0][0] + fhat[1][0]*fhat[1][0] );
    FLT uhat_len = sqrt( uhat[0][0]*uhat[0][0] + uhat[1][0]*uhat[1][0] );
    FLT rhat_len = sqrt( rhat[0][0]*rhat[0][0] + rhat[1][0]*rhat[1][0] );
    FLT urhat_len = 0.5 * (uhat_len + rhat_len);
/*    
printf("fhat %f %f (len %f)\n", fhat[0][0], fhat[1][0], fhat_len);
printf("uhat %f %f (len %f)\n", uhat[0][0], uhat[1][0], uhat_len);
printf("rhat %f %f (len %f)\n", rhat[0][0], rhat[1][0], rhat_len);
*/
//    FLT ydist1 = 1.0 /  uhat_len; //0.25*PI / uhat_len;
//    FLT ydist2 = 1.0 /  rhat_len; //0.25*PI / rhat_len;
    FLT ydist  = 1.0 / urhat_len;
    //printf("ydist1 %f ydist2 %f ydist %f\n", ydist1, ydist2, ydist);

    //--------------------
    // Rescale the axies to be of the proper length
    //--------------------
    FLT x[3][1] = { {M[0][0]*ydist}, {0.0}, {M[1][0]*ydist} };
    FLT y[3][1] = { {M[0][1]*ydist}, {0.0}, {M[1][1]*ydist} };
    FLT z[3][1] = { {M[0][2]*ydist}, {0.0}, {M[1][2]*ydist} };
    // we know the distance into (or out of) the camera for the z axis,
    //  but we don't know which direction . . .
    FLT x_y = sqrt(1.0 - x[0][0]*x[0][0] - x[2][0]*x[2][0]);
    FLT y_y = sqrt(1.0 - y[0][0]*y[0][0] - y[2][0]*y[2][0]);
    FLT z_y = sqrt(1.0 - z[0][0]*z[0][0] - z[2][0]*z[2][0]);

	if( x_y != x_y ) x_y = 0;
	if( y_y != y_y ) y_y = 0;
	if( z_y != z_y ) z_y = 0;
/*
    // Exhaustively flip the minus sign of the z axis until we find the right one . . .
    FLT bestErr = 9999.0;
    FLT xy_dot2 = x[0][0]*y[0][0] + x[2][0]*y[2][0];
    FLT yz_dot2 = y[0][0]*z[0][0] + y[2][0]*z[2][0];
    FLT zx_dot2 = z[0][0]*x[0][0] + z[2][0]*x[2][0];
    for (i=0;i<2;i++) {
        for (j=0;j<2;j++) {
            for(k=0;k<2;k++) {
            
                // Calculate the error term
                FLT xy_dot = xy_dot2 + x_y*y_y;
                FLT yz_dot = yz_dot2 + y_y*z_y;
                FLT zx_dot = zx_dot2 + z_y*x_y;
                FLT err = _ABS(xy_dot) + _ABS(yz_dot) + _ABS(zx_dot);
                
                // Calculate the handedness
                FLT cx,cy,cz;
                CrossProduct(cx,cy,cz,x[0][0],x_y,x[2][0],y[0][0],y_y,y[2][0]);
                FLT hand = cx*z[0][0] + cy*z_y + cz*z[2][0];
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
    FLT ydist2 = ydist;
    FLT bestBestErr = 9999.0;
    FLT bestYdist = 0;
    for (ydist2=ydist-0.1; ydist2<ydist+0.1; ydist2+=0.0001)
    {
        FLT x2[3][1] = { {M[0][0]*ydist2}, {0.0}, {M[1][0]*ydist2} };
        FLT y2[3][1] = { {M[0][1]*ydist2}, {0.0}, {M[1][1]*ydist2} };
        FLT z2[3][1] = { {M[0][2]*ydist2}, {0.0}, {M[1][2]*ydist2} };

        // we know the distance into (or out of) the camera for the z axis,
        //  but we don't know which direction . . .
        FLT x_y = sqrt(1.0 - x2[0][0]*x2[0][0] - x2[2][0]*x2[2][0]);
        FLT y_y = sqrt(1.0 - y2[0][0]*y2[0][0] - y2[2][0]*y2[2][0]);
        FLT z_y = sqrt(1.0 - z2[0][0]*z2[0][0] - z2[2][0]*z2[2][0]);

		if( x_y != x_y ) x_y = 0;
		if( y_y != y_y ) y_y = 0;
		if( z_y != z_y ) z_y = 0;

		printf( "---> %f %f %f\n", x_y, y_y, z_y );

        // Exhaustively flip the minus sign of the z axis until we find the right one . . .
        FLT bestErr = 9999.0;
        FLT xy_dot2 = x2[0][0]*y2[0][0] + x2[2][0]*y2[2][0];
        FLT yz_dot2 = y2[0][0]*z2[0][0] + y2[2][0]*z2[2][0];
        FLT zx_dot2 = z2[0][0]*x2[0][0] + z2[2][0]*x2[2][0];
        for (i=0;i<2;i++) {
            for (j=0;j<2;j++) {
                for(k=0;k<2;k++) {
            
                    // Calculate the error term
                    FLT xy_dot = xy_dot2 + x_y*y_y;
                    FLT yz_dot = yz_dot2 + y_y*z_y;
                    FLT zx_dot = zx_dot2 + z_y*x_y;
                    FLT err = _ABS(xy_dot) + _ABS(yz_dot) + _ABS(zx_dot);
                
                    // Calculate the handedness
                    FLT cx,cy,cz;
                    CrossProduct(cx,cy,cz,x2[0][0],x_y,x2[2][0],y2[0][0],y_y,y2[2][0]);
                    FLT hand = cx*z2[0][0] + cy*z_y + cz*z2[2][0];
                  printf("err %f hand %f\n", err, hand);
                
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
            memcpy(x,x2,3*sizeof(FLT));
            memcpy(y,y2,3*sizeof(FLT));
            memcpy(z,z2,3*sizeof(FLT));
            bestBestErr = bestErr;
            bestYdist = ydist2;
        }
    }
    ydist = bestYdist;

/*
    for (i=0; i<nPoints; i++) {
        FLT x1 = x[0][0]*X[0][i] + y[0][0]*X[1][i] + z[0][0]*X[2][i];
        FLT y1 = x[1][0]*X[0][i] + y[1][0]*X[1][i] + z[1][0]*X[2][i];
        FLT z1 = x[2][0]*X[0][i] + y[2][0]*X[1][i] + z[2][0]*X[2][i];
        printf("x1z1 %f %f y1 %f\n", x1, z1, y1);
    }
*/
/*    
    //--------------------
    // Combine uhat and rhat to figure out the unit x-vector
    //--------------------
    FLT xhat[2][1]  = { {0.0}, {1.0} };
    FLT urhat[2][2] = {
        {uhat[0][0], uhat[1][0]},
        {rhat[0][0], rhat[1][0]} };
    FLT inv_urhat[2][2];
    FLT ab[2][1];
    INV(urhat,inv_urhat,2);
    MUL(inv_urhat,xhat,ab,2,2,1);
PRINT(ab,2,1);
    FLT a = ab[0][0], b = ab[1][0];

    //-------------------
    // calculate the xyz coordinate system
    //-------------------
    FLT y[3][1] = { {f[0][0]}, {f[1][0]}, {f[2][0]} };
    FLT x[3][1] = { {a*u[0][0] + b*r[0][0]}, {a*u[1][0] + b*r[1][0]}, {a*u[2][0] + b*r[2][0]} };
    FLT inv_xlen = 1.0 / sqrt( x[0][0]*x[0][0] + x[1][0]*x[1][0] + x[2][0]*x[2][0] );
    x[0][0]*=inv_xlen; x[1][0]*=inv_xlen; x[2][0]*=inv_xlen;
    FLT z[3][1];
    CrossProduct(z[0][0],z[1][0],z[2][0],x[0][0],x[1][0],x[2][0],y[0][0],y[1][0],y[2][0]);
*/
    // Store into the rotation matrix
    for (i=0; i<3; i++) { R[i][0] = x[i][0]; R[i][1] = y[i][0]; R[i][2] = z[i][0]; }
//PRINT(R,3,3);

    //-------------------
    // Calculate the translation of the centroid
    //-------------------
    trans[0]=tan(sbar[0]);  trans[1]=1.0;  trans[2]=tan(sbar[1]);
    FLT inv_translen = ydist / sqrt( trans[0]*trans[0] + trans[1]*trans[1] + trans[2]*trans[2] );
    trans[0]*=inv_translen; trans[1]*=inv_translen; trans[2]*=inv_translen;

    //-------------------
    // Add in the centroid point
    //-------------------
    trans[0] -= xbar[0]*R[0][0] + xbar[1]*R[0][1] + xbar[2]*R[0][2];
    trans[1] -= xbar[0]*R[1][0] + xbar[1]*R[1][1] + xbar[2]*R[1][2];
    trans[2] -= xbar[0]*R[2][0] + xbar[1]*R[2][1] + xbar[2]*R[2][2];
    FLT transdist = sqrt( trans[0]*trans[0] + trans[1]*trans[1] + trans[2]*trans[2] );

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
        FLT Tx = T[0][0]*X_in[0][i] + T[0][1]*X_in[1][i] + T[0][2]*X_in[2][i] + T[0][3];
        FLT Ty = T[1][0]*X_in[0][i] + T[1][1]*X_in[1][i] + T[1][2]*X_in[2][i] + T[1][3];
        FLT Tz = T[2][0]*X_in[0][i] + T[2][1]*X_in[1][i] + T[2][2]*X_in[2][i] + T[2][3];
        S_out[0][i] = atan2(Tx, Ty);   // horiz
        S_out[1][i] = atan2(Tz, Ty);   // vert
        //S_out[0][i] = Tx;
        //S_out[1][i] = Tz;
        printf("point %i Txyz %f %f %f in %f %f out %f %f morph %f %f\n", i, Tx,Ty,Tz, S_in[0][i], S_in[1][i], S_out[0][i], S_out[1][i], S_morph[0][i], S_morph[1][i]);
    }

}

