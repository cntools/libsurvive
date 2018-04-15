/*
 When creating your project, uncheck OWL,
 uncheck Class Library, select Static
 instead of Dynamic and change the target
 model to Console from GUI.
 Also link glut.lib to your project once its done.
 */

#include <stdio.h>   // Standard Header For Most Programs
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "os_generic.h"
#include "linmath.h"
#include "fileutil.h"

#ifdef __APPLE__
#include <OpenGL/gl.h>   // The GL Header File
#include <GLUT/glut.h>   // The GL Utility Toolkit (Glut) Header
#else
#include <GL/gl.h>
#include <GL/glu.h>
#endif
#ifdef __linux__
#include <GL/freeglut.h>
#endif

#define RegisterDriver(a,b)
#include "poser_daveortho.c"


// Required to set up a window
#define WIDTH  1280
#define HEIGHT 1280
#define FULLSCREEN 0
int    keys[256];               // Regular keyboard keys
int sp_keys[256];               // Special keyboard keycodes (GLUT specific)

#define LH_ID    0
#define NUM_HMD 32
#define INDIR "dave/full_test_triangle_on_floor/"
#define MAX_POINTS SENSORS_PER_OBJECT

#define PI 3.1415926535897932386264
#define MOVESPEED 1.0
#define ROTSPEED  5.0

// View space
float posx=0.0f;
float posy=0.0f;
float posz=0.0f;
float rotx=0.0f;
float roty=0.0f;
float rotz=0.0f;

// Data for the "fake" ortho solve formula
float Tortho[4][4];         // OUTPUT: 4x4 transformation matrix
FLOAT S_out[2][MAX_POINTS];  // INPUT:  array of screenspace points
FLOAT S_in[2][MAX_POINTS];  // INPUT:  array of screenspace points
FLOAT X_in[3][MAX_POINTS];  // INPUT:  array of offsets
int nPoints=0;  

//--------------------------------------------------------------------
// 
//--------------------------------------------------------------------

void DrawGrid(
	float minX, float maxX,
	float minY, float maxY,
	float minZ, float maxZ,
	float stepX, float stepY, float stepZ);

void DrawCoordinateSystem(
	float x, float y, float z,
	float qx, float qy, float qz, float qr);


float hmd_pos[NUM_HMD][3];
void ReadHmdPoints()
{
    int i;
    FILE *fin = fopen(INDIR "HMD_points.csv","r");
    if (fin==NULL) {
        printf("ERROR: could not open " INDIR "HMD_points.csv for reading\n");
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


//--------------------------------------------------------------------
// 
//--------------------------------------------------------------------

/*
 * init() is called at program startup
 */
void init()
{
    int i,j,k,sen,axis;
        
    // Read the data files
    ReadHmdPoints();
    ReadPtinfo();

    //--------------------------------------------------
    // Package the data for "OrthoSolve"
    //--------------------------------------------------

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
    
    // Run OrthoSolve
    OrthoSolve(
		Tortho,     // OUTPUT: 4x4 transformation matrix
		S_out,      // OUTPUT: array of output screenspace points
		S_in,       // INPUT:  array of screenspace points
		X_in,       // INPUT:  array of offsets
		nPoints);
		//printf( "POS: %f %f %f\n", Tortho[0][3], Tortho[1][3], Tortho[2][3]);

	//--------------------------------------------------
	// Spawn a thread to read the HMD angles
	//--------------------------------------------------
	OGCreateThread(ThreadReadHmtAngles,0);

    //--------------------------------------------------
	// Initialize OpenGL
    //--------------------------------------------------
	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);				// Black Background
	glClearDepth(1.0f);									// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);							// Enables Depth Testing
	glDepthFunc(GL_LEQUAL);								// The Type Of Depth Testing To Do
	glEnable ( GL_COLOR_MATERIAL );
	glDisable(GL_CULL_FACE);
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
}

/*
 * draw() is called once every frame
 */
void draw()
{
	int i,j;

    //------------------------
    // Check for keyboard input
    //------------------------
    if (keys['w'] || keys['W']) {
        posz += MOVESPEED;
    }
    if (keys['s'] || keys['S']) {
        posz -= MOVESPEED;
    }
    if (keys['a'] || keys['A']) {
        roty += ROTSPEED;
    }
    if (keys['d'] || keys['D']) {
        roty -= ROTSPEED;
    }
    if (keys[27]) {
        exit(0);
    }
    
    //------------------------
    // Update the scene
    //------------------------
    
    //------------------------
    // Draw using OpenGL
    //------------------------
    
    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);	// Clear Screen And Depth Buffer
 
    // Translate and rotate the camera
    glLoadIdentity();  // Reset The Current Modelview Matrix
//    glTranslatef(-posx,-posy,posz);						// Move Left 1.5 Units And Into The    

	// Bouning box around the points (in radians)
	float x0=-45.0 * (PI/180.0);
	float x1= 45.0 * (PI/180.0);
	float y0=-45.0 * (PI/180.0);
	float y1= 45.0 * (PI/180.0);


	//------------------------
	// Read the angles from stdin
	//------------------------
	FLT hmdAngles[4][NUM_HMD];

	// Read the hmd angles
	OGLockMutex(read_mutex);
	for (i=0; i<4; i++) {
		for (j=0; j<NUM_HMD; j++) {
			hmdAngles[i][j] = read_hmdAngles[i][j];
		}
	}
	OGUnlockMutex(read_mutex);

	// Draw the hmd bearing angles
	nPoints=0;
	for (i=0; i<NUM_HMD; i++) {
		const double dist=10.0;
		
		int sweepx = (LH_ID==0) ? SWEEP_LX : SWEEP_RX;
		int sweepy = (LH_ID==0) ? SWEEP_LY : SWEEP_RY;

		// If the left lighthouse sees it
		if (read_hmdAngleViewed[sweepx][i] >= read_frameno-6 &&
		    read_hmdAngleViewed[sweepy][i] >= read_frameno-6 &&
		    hmdAngles[sweepx][i]!=-9999.0 && hmdAngles[sweepy][i]!=-9999.0)
		{
			S_in[0][nPoints] = hmdAngles[sweepy][i];
			S_in[1][nPoints] = hmdAngles[sweepx][i];
            X_in[0][nPoints] = hmd_pos[i][0];
            X_in[1][nPoints] = hmd_pos[i][1];
            X_in[2][nPoints] = hmd_pos[i][2];
printf("i %d S %f %f X %f %f %f frno %d %d currfr %d\n",
	i, S_in[0][nPoints], S_in[1][nPoints],
	X_in[0][nPoints], X_in[1][nPoints], X_in[2][nPoints],
	read_hmdAngleViewed[sweepx][i], read_hmdAngleViewed[sweepy][i], read_frameno);
			nPoints++;
		}
	}

	read_frameno++;

    //--------------------------------------------------
    // Run the "OrthoSolve" and then the "AffineSolve"
    //--------------------------------------------------    
    
    // Run OrthoSolve
    OrthoSolve(
		Tortho,     // OUTPUT: 4x4 transformation matrix
		S_out,      // OUTPUT: array of output screenspace points
		S_in,       // INPUT:  array of screenspace points
		X_in,       // INPUT:  array of offsets
		nPoints);
		printf( "POS: %f %f %f\n", Tortho[0][3], Tortho[1][3], Tortho[2][3]);


	//------------------------
	// Draw the inputs
	//------------------------
	glPointSize(3.0);

	// Draw the input points
	glColor3f(1.0,0.5,0.5);
	glBegin(GL_POINTS);
	for (i=0; i<nPoints; i++) {
		glVertex2f(  (S_in[0][i]-x0)/(x1-x0),  (S_in[1][i]-y0)/(y1-y0)  );
	}
	glEnd();

	// Draw the output points
	glColor3f(0.5,0.5,1.0);
	glBegin(GL_POINTS);
	for (i=0; i<nPoints; i++) {
		glVertex2f(  (S_out[0][i]-x0)/(x1-x0),  (S_out[1][i]-y0)/(y1-y0)  );
	}
	glEnd();


    //
    // We're Done
    //
    glutSwapBuffers ( );         // Swap The Buffers To Not Be Left With A Clear Screen
}

/*
 * resize() is called when we change the screen size
 */
void resize(int width, int height)		// Resize And Initialize The GL Window
{
    glViewport(0,0,width,height);						// Reset The Current Viewport
    
    glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
    glLoadIdentity();									// Reset The Projection Matrix
    
    // Uncomment For a 3D perspective
    //gluPerspective(45.0f,(float)width/(float)height,0.1f,1000.0f);
    // Uncomment for a 2D perspective
    glOrtho(0.0, 1.0, 0.0, 1.0, -10.0, 10.0);
    
    glMatrixMode(GL_MODELVIEW);							// Select The Modelview Matrix
    glLoadIdentity();									// Reset The Modelview Matrix
}

/*
 * These functions are called whenever a key is pressed
 */
void keyboardDown ( unsigned char key, int x, int y )  // Create Keyboard Function
{
    keys[key] = 1;
}
void keyboardUp ( unsigned char key, int x, int y )
{
    keys[key] = 0;
}

void specialKeyDown ( int key, int x, int y )  // Create Special Function (required for arrow keys)
{
    if (key<256) {
        sp_keys[key] = 1;
    }
}
void specialKeyUp (int key, int x, int y)
{
    if (key<256) {
        sp_keys[key] = 0;
    }
}

int main ( int argc, char** argv )   // Create Main Function For Bringing It All Together
{
    glutInit            ( &argc, argv ); // Erm Just Write It =)
    glutInitDisplayMode ( GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE ); // Display Mode
    glutInitWindowSize  ( WIDTH, HEIGHT ); // If glutFullScreen wasn't called this is the window size
    glutCreateWindow    ( "OpenGL" ); // Window Title (argv[0] for current directory as title)
    if (FULLSCREEN) {
        glutFullScreen      ( );          // Put Into Full Screen
    }
    glutDisplayFunc     ( draw );  // Matching Earlier Functions To Their Counterparts
    glutIdleFunc        ( draw );
    glutReshapeFunc     ( resize );
    glutKeyboardFunc    ( keyboardDown );
    glutKeyboardUpFunc  ( keyboardUp );
    glutSpecialFunc     ( specialKeyDown );
    glutSpecialUpFunc   ( specialKeyUp );
    init ();
    glutMainLoop        ( );          // Initialize The Main Loop
    return 0;
}



void DrawGrid(
	float minX, float maxX,
	float minY, float maxY,
	float minZ, float maxZ,
	float stepX, float stepY, float stepZ)
{
	float x,y,z;
	
	glBegin(GL_LINES);

	// X grid stripes
	for (y=minY; y<maxY; y+=stepY) {
		for (z=minZ; z<maxZ; z+=stepZ) {
			glVertex3f(minX, y, z);
			glVertex3f(maxX, y, z);
		}
	}

	// Y grid stripes
	for (x=minX; x<maxX; x+=stepX) {
		for (z=minZ; z<maxZ; z+=stepZ) {
			glVertex3f(x, minY, z);
			glVertex3f(x, maxY, z);
		}
	}

	// Z grid stripes
	for (y=minY; y<maxY; y+=stepY) {
		for (x=minX; x<maxX; x+=stepX) {
			glVertex3f(x, y, minZ);
			glVertex3f(x, y, maxZ);
		}
	}

	glEnd();
}


void DrawCoordinateSystem(
	float x, float y, float z,
	float qx, float qy, float qz, float qr)
{
	FLT i0[3],j0[3],k0[3];
	FLT i[3],j[3],k[3];
	FLT q[4];
	
	i0[0]=1.0; i0[1]=0.0; i0[2]=0.0;
	j0[0]=0.0; j0[1]=1.0; j0[2]=0.0;
	k0[0]=0.0; k0[1]=0.0; k0[2]=1.0;
	q [0]=qr;  q [1]=qx;  q [2]=qy;  q [3]=qz;
	
	quatrotatevector(i, q, i0);
	quatrotatevector(j, q, j0);
	quatrotatevector(k, q, k0);

	glBegin(GL_LINES);
	glColor3f(1, 0, 0); glVertex3f(x,z,y); glVertex3f(x+i[0],z+i[2],y+i[1]);
	glColor3f(0, 1, 0); glVertex3f(x,z,y); glVertex3f(x+j[0],z+j[2],y+j[1]);
	glColor3f(0, 0, 1); glVertex3f(x,z,y); glVertex3f(x+k[0],z+k[2],y+k[1]);
	glEnd();
}


