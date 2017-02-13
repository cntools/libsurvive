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
#include "glutil.h"
#include "fileutil.h"

#ifdef LINUX
#include <GL/freeglut.h>
#endif


// Required to set up a window
#define WIDTH  800
#define HEIGHT 600
#define FULLSCREEN 0
int    keys[256];               // Regular keyboard keys
int sp_keys[256];               // Special keyboard keycodes (GLUT specific)

#define PI 3.1415926535897932386264
#define MOVESPEED 1.0
#define ROTSPEED  5.0

#define GRID_SIZE       6
#define GRID_MAJOR_STEP 3
#define GRID_MINOR_STEP 1

// Lighthouses
typedef struct Lhouse {
	float x,y,z,qi,qj,qk,qreal;   // position x,y,z and quaternion
} Lhouse;
Lhouse houseL, houseR;

// HmdSensor array
double hmdAngles[NUM_SWEEP][NUM_HMD];


static void HmdDir(double LX, double LY, double *pX, double *pY, double *pZ) {
	// Normals for the two planes
	double a = cos(LX);  double b = 0.0;     double c = -sin(LX);  // X,Z plane
	double A = 0.0;      double B = cos(LY); double C = -sin(LY);  // Y,Z plane
//	double A=0.0; double B=1.0; double C=0.0;
	printf("rotX %f rotY %f x,z plane normal %f %f %f  y,z plane normal %f %f %f\n",
		180/PI*LX,180/PI*LY, a,b,c, A,B,C);
	
	// calculate the direction
	double x = (b*C - B*c) / (B*a - A*b);
	double y = (a*C - A*c) / (A*b - B*a);
	double z = 1.0;
	
	// Normalize
	double len = sqrt(x*x + y*y + z*z);
	x/=len; y/=len; z/=len;
	printf("   xyz %f %f %f\n", x, y, z);
	
	*pX=x; *pY=y; *pZ=z;
}

// View space
float posx=0.0f;
float posy=0.0f;
float posz=0.0f;
float rotx=0.0f;
float roty=0.0f;
float rotz=0.0f;

/*
 * init() is called at program startup
 */
void init()
{    
	int i,j;
//void quatrotatevector( FLT * vec3out, const FLT * quat, const FLT * vec3in );

	// Load the lighthouses
	LoadLighthousePos("L.txt", &houseL.x,&houseL.y,&houseL.z,&houseL.qi,&houseL.qj,&houseL.qk,&houseL.qreal);
	LoadLighthousePos("R.txt", &houseR.x,&houseR.y,&houseR.z,&houseR.qi,&houseR.qj,&houseR.qk,&houseR.qreal);
	printf("L pos %f %f %f quat %f %f %f %f\n", houseL.x,houseL.y,houseL.z,houseL.qi,houseL.qj,houseL.qk,houseL.qreal);
	printf("R pos %f %f %f quat %f %f %f %f\n", houseR.x,houseR.y,houseR.z,houseR.qi,houseR.qj,houseR.qk,houseR.qreal);
/*
	// Load the bearing angles
	LoadHmdProcessedDataAngles("processed_data.txt", hmdAngles);
*/
	// Spawn the thread to read the hmt angles
	memset(read_hmdAngleViewed, 0, NUM_HMD*NUM_SWEEP*sizeof(int));
	read_mutex  = OGCreateMutex();
	for (i=0; i<NUM_HMD; i++) {
		for (j=0; j<4; j++) {
			read_hmdAngles[j][i] = -9999.0;
		}
	}
	read_thread = OGCreateThread(ThreadReadHmtAngles,NULL);	

	for (i=0; i<NUM_HMD; i++) {
		if (hmdAngles[0][i]!=-9999.0 && hmdAngles[1][i]!=-9999.0 && hmdAngles[2][i]!=-9999.0 && hmdAngles[3][i]!=-9999.0)
		{
			printf("hmd %d lx %f ly %f rx %f ry %f\n",
				i,
				(180.0/PI) * hmdAngles[SWEEP_LX][i],
				(180.0/PI) * hmdAngles[SWEEP_LY][i],
				(180.0/PI) * hmdAngles[SWEEP_RX][i],
				(180.0/PI) * hmdAngles[SWEEP_RY][i]);
		}
	}

	// Initialize OpenGL
	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading
	glClearColor(0.6f, 0.8f, 1.0f, 0.5f);				// Black Background
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
    glTranslatef(-posx,-posy,posz);						// Move Left 1.5 Units And Into The    

    glRotatef(-rotx, 1.0f, 0.0f, 0.0f);
    glRotatef(-roty+90.0, 0.0f, 1.0f, 0.0f);
    glRotatef(-rotz, 0.0f, 0.0f, 1.0f);

    glScalef(1.0, 1.0, -1.0);
//    printf("rot: %f %f %f  pos: %f %f %f\n", rotx, roty, rotz, posx, posy, posz);

	//-----------------    
	// Draw the grid
	//-----------------
//    glPointSize(1.0);
//    glColor3f(0.5, 0.5, 0.5);
//    DrawGrid(-GRID_SIZE, GRID_SIZE, -GRID_SIZE, GRID_SIZE, -GRID_SIZE, GRID_SIZE, 
//	GRID_MINOR_STEP, GRID_MINOR_STEP, GRID_MINOR_STEP);


	glLineWidth(1.0);
	
	glPointSize(4.0);
	glColor3f(0.0, 0.0, 0.0);
	DrawGrid(-GRID_SIZE, GRID_SIZE, -GRID_SIZE, GRID_SIZE, -GRID_SIZE, GRID_SIZE, 
		GRID_MAJOR_STEP, GRID_MAJOR_STEP, GRID_MAJOR_STEP);

	//-----------------
	// Plot the lighthouses
	//-----------------
	
	glLineWidth(2.0);
	
	// Left lighthouse coordinate system
	DrawCoordinateSystem(
		houseL.x, houseL.y, houseL.z,
		houseL.qi, houseL.qj, houseL.qk, houseL.qreal);
	
	// Rightt lighthouse coordinate system
	DrawCoordinateSystem(
		houseR.x, houseR.y, houseR.z,
		houseR.qi, houseR.qj, houseR.qk, houseR.qreal);

	glPointSize(15.0);
	glBegin(GL_POINTS);
	
	// Left house in red
	glColor3f(1,0,0);
	glVertex3f(houseL.x, houseL.z, houseL.y);
	
	// Green is the origin (hmd)
	glColor3f(0,1,0);
	glVertex3f(0,0,0);
	
	// Right house in blue
	glColor3f(0,0,1);
	glVertex3f(houseR.x, houseR.z, houseR.y);
	
	glEnd();

	//-------------------
	// Draw the bearing angles
	//-------------------
	
	// Read the hmd angles
	OGLockMutex(read_mutex);
	for (i=0; i<4; i++) {
		for (j=0; j<NUM_HMD; j++) {
			hmdAngles[i][j] = read_hmdAngles[i][j];
		}
	}
	OGUnlockMutex(read_mutex);

	// Draw the hmd bearing angles
	glBegin(GL_LINES);
	for (i=0; i<NUM_HMD; i++) {
		const double dist=10.0;
		
		// If the left lighthouse sees it
		if (read_hmdAngleViewed[SWEEP_LX][i] >= read_frameno-6 &&
		    read_hmdAngleViewed[SWEEP_LY][i] >= read_frameno-6 &&
		    hmdAngles[SWEEP_LX][i]!=-9999.0 && hmdAngles[SWEEP_LY][i]!=-9999.0)
		{
			// Get the hmd bearings
			double Ldx,Ldy,Ldz;
			HmdDir(hmdAngles[SWEEP_LX][i], hmdAngles[SWEEP_LY][i], &Ldx, &Ldy, &Ldz);

			// Rotate the bearings by the lighthouse orientation
			FLT L0[3],L[3],Lq[4];
			L0[0]=Ldx; L0[1]=Ldy; L0[2]=Ldz;
			Lq[0]=houseL.qreal; Lq[1]=houseL.qi; Lq[2]=houseL.qj; Lq[3]=houseL.qk;
			quatrotatevector(L, Lq, L0);

			// Plot the lines
			glVertex3f(houseL.x, houseL.z, houseL.y);
			glVertex3f(houseL.x+dist*L[0], houseL.z+dist*L[2], houseL.y+dist*L[1]);
		}
		
		// If the left lighthouse sees it
		if (read_hmdAngleViewed[SWEEP_RX][i] >= read_frameno-6 &&
		    read_hmdAngleViewed[SWEEP_RY][i] >= read_frameno-6 &&
		    hmdAngles[SWEEP_RX][i]!=-9999.0 && hmdAngles[SWEEP_RY][i]!=-9999.0)
		{
			// Get the hmd bearings
			double Rdx,Rdy,Rdz;
			HmdDir(hmdAngles[SWEEP_RX][i], hmdAngles[SWEEP_RY][i], &Rdx, &Rdy, &Rdz);

			// Rotate the bearings by the lighthouse orientation
			FLT R0[3],R[3],Rq[4];
			R0[0]=Rdx; R0[1]=Rdy; R0[2]=Rdz;
			Rq[0]=houseR.qreal; Rq[1]=houseR.qi; Rq[2]=houseR.qj; Rq[3]=houseR.qk;
			quatrotatevector(R, Rq, R0);

			// Plot the lines
			glVertex3f(houseR.x, houseR.z, houseR.y);
			glVertex3f(houseR.x+dist*R[0], houseR.z+dist*R[2], houseR.y+dist*R[1]);
		}
	}
	glEnd();

	read_frameno++;

    //
    // Draw the Terrain
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
    gluPerspective(45.0f,(float)width/(float)height,0.1f,1000.0f);
    // Uncomment for a 2D perspective
    //glOrtho(0.0, WIDTH, 0.0, HEIGHT, -10.0, 10.0);
    
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

