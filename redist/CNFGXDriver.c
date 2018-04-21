//Copyright (c) 2011, 2017 <>< Charles Lohr - Under the MIT/x11 or NewBSD License you choose.
//portions from 
//http://www.xmission.com/~georgeps/documentation/tutorials/Xlib_Beginner.html

//#define HAS_XINERAMA

#include "CNFGFunctions.h"

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/Xos.h>
#include <X11/Xatom.h>
#include <X11/keysym.h>
#ifdef HAS_XINERAMA
#include <X11/extensions/shape.h>
#include <X11/extensions/Xinerama.h>
#endif
#include <stdio.h>
#include <stdlib.h>

XWindowAttributes CNFGWinAtt;
XClassHint *CNFGClassHint;
Display *CNFGDisplay;
Window CNFGWindow;
Pixmap CNFGPixmap;
GC     CNFGGC;
GC     CNFGWindowGC;
Visual * CNFGVisual;


#ifdef CNFGOGL
#include <GL/glx.h>
#include <GL/glxext.h>

GLXContext CNFGCtx;
void * CNFGGetExtension( const char * extname ) { return glXGetProcAddressARB((const GLubyte *) extname); }
#endif

int FullScreen = 0;

void CNFGGetDimensions( short * x, short * y )
{
	static int lastx;
	static int lasty;

	*x = CNFGWinAtt.width;
	*y = CNFGWinAtt.height;

	if( lastx != *x || lasty != *y )
	{
		lastx = *x;
		lasty = *y;
		CNFGInternalResize( lastx, lasty );
	}
}

static void InternalLinkScreenAndGo( const char * WindowName )
{
	XGetWindowAttributes( CNFGDisplay, CNFGWindow, &CNFGWinAtt );

	XGetClassHint( CNFGDisplay, CNFGWindow, CNFGClassHint );
	if (!CNFGClassHint) {
		CNFGClassHint = XAllocClassHint();
		if (CNFGClassHint) {
			CNFGClassHint->res_name = "cnping";
			CNFGClassHint->res_class = "cnping";
			XSetClassHint( CNFGDisplay, CNFGWindow, CNFGClassHint );
		} else {
			fprintf( stderr, "Failed to allocate XClassHint!\n" );
		}
	} else {
		fprintf( stderr, "Pre-existing XClassHint\n" );
	}

	XSelectInput (CNFGDisplay, CNFGWindow, KeyPressMask | KeyReleaseMask | ButtonPressMask | ButtonReleaseMask | ExposureMask | PointerMotionMask );
	XSetStandardProperties( CNFGDisplay, CNFGWindow, WindowName, WindowName, None, NULL, 0, NULL );

	CNFGWindowGC = XCreateGC(CNFGDisplay, CNFGWindow, 0, 0);

	CNFGPixmap = XCreatePixmap( CNFGDisplay, CNFGWindow, CNFGWinAtt.width, CNFGWinAtt.height, CNFGWinAtt.depth );
	CNFGGC = XCreateGC(CNFGDisplay, CNFGPixmap, 0, 0);
}

void CNFGSetupFullscreen( const char * WindowName, int screen_no )
{
#ifdef HAS_XINERAMA
	XineramaScreenInfo *screeninfo = NULL;
	int screens;
	int event_basep, error_basep, a, b;
	CNFGDisplay = XOpenDisplay(NULL);
	int screen = XDefaultScreen(CNFGDisplay);
	int xpos, ypos;

	if (!XShapeQueryExtension(CNFGDisplay, &event_basep, &error_basep))
	{
    	fprintf( stderr, "X-Server does not support shape extension" );
		exit( 1 );
	}

 	CNFGVisual = DefaultVisual(CNFGDisplay, screen);
	CNFGWinAtt.depth = DefaultDepth(CNFGDisplay, screen);

	if (XineramaQueryExtension(CNFGDisplay, &a, &b ) &&
		(screeninfo = XineramaQueryScreens(CNFGDisplay, &screens)) &&
		XineramaIsActive(CNFGDisplay) && screen_no >= 0 &&
		screen_no < screens ) {

		CNFGWinAtt.width = screeninfo[screen_no].width;
		CNFGWinAtt.height = screeninfo[screen_no].height;
		xpos = screeninfo[screen_no].x_org;
		ypos = screeninfo[screen_no].y_org;
	} else
	{
		CNFGWinAtt.width = XDisplayWidth(CNFGDisplay, screen);
		CNFGWinAtt.height = XDisplayHeight(CNFGDisplay, screen);
		xpos = 0;
		ypos = 0;
	}
	if (screeninfo)
	XFree(screeninfo);


	XSetWindowAttributes setwinattr;
	setwinattr.override_redirect = 1;
	setwinattr.save_under = 1;
	setwinattr.event_mask = StructureNotifyMask | SubstructureNotifyMask | ExposureMask | ButtonPressMask | ButtonReleaseMask | ButtonPressMask | PointerMotionMask | ButtonMotionMask | EnterWindowMask | LeaveWindowMask |KeyPressMask |KeyReleaseMask | SubstructureNotifyMask | FocusChangeMask;
	setwinattr.border_pixel = 0;

	CNFGWindow = XCreateWindow(CNFGDisplay, XRootWindow(CNFGDisplay, screen),
		xpos, ypos, CNFGWinAtt.width, CNFGWinAtt.height,
		0, CNFGWinAtt.depth, InputOutput, CNFGVisual, 
		CWBorderPixel | CWEventMask | CWOverrideRedirect | CWSaveUnder, 
		&setwinattr);

	XMapWindow(CNFGDisplay, CNFGWindow);
	XSetInputFocus( CNFGDisplay, CNFGWindow,   RevertToParent, CurrentTime );
	XFlush(CNFGDisplay);
	FullScreen = 1;
//printf( "%d %d %d %d\n", xpos, ypos, CNFGWinAtt.width, CNFGWinAtt.height );
	InternalLinkScreenAndGo( WindowName );
/*
	setwinattr.override_redirect = 1;
	XChangeWindowAttributes(
		CNFGDisplay, CNFGWindow,
		CWBorderPixel | CWEventMask | CWOverrideRedirect, &setwinattr);
*/
#else
	CNFGSetup( WindowName, 640, 480 );
#endif
}


void CNFGTearDown()
{
	if ( CNFGClassHint ) XFree( CNFGClassHint );
	if ( CNFGGC ) XFreeGC( CNFGDisplay, CNFGGC );
	if ( CNFGWindowGC ) XFreeGC( CNFGDisplay, CNFGWindowGC );
	if ( CNFGDisplay ) XCloseDisplay( CNFGDisplay );
	CNFGDisplay = NULL;
	CNFGWindowGC = CNFGGC = NULL;
	CNFGClassHint = NULL;
}

int CNFGSetup( const char * WindowName, int w, int h )
{
	CNFGDisplay = XOpenDisplay(NULL);
	atexit( CNFGTearDown );
	XGetWindowAttributes( CNFGDisplay, RootWindow(CNFGDisplay, 0), &CNFGWinAtt );

	int depth = CNFGWinAtt.depth;
	int screen = DefaultScreen(CNFGDisplay);
 	CNFGVisual = DefaultVisual(CNFGDisplay, screen);

#ifdef CNFGOGL
	int attribs[] = { GLX_RGBA,
		GLX_DOUBLEBUFFER, 
		GLX_RED_SIZE, 1,
		GLX_GREEN_SIZE, 1,
		GLX_BLUE_SIZE, 1,
		GLX_DEPTH_SIZE, 1,
		None };
	XVisualInfo * vis = glXChooseVisual(CNFGDisplay, screen, attribs);
	CNFGVisual = vis->visual;
	depth = vis->depth;
	CNFGCtx = glXCreateContext( CNFGDisplay, vis, NULL, True );
#endif

	XSetWindowAttributes attr;
	attr.background_pixel = 0;
	attr.border_pixel = 0;
	attr.colormap = XCreateColormap( CNFGDisplay, RootWindow(CNFGDisplay, 0), CNFGVisual, AllocNone);
	attr.event_mask = StructureNotifyMask | ExposureMask | KeyPressMask;
	int mask = CWBackPixel | CWBorderPixel | CWColormap | CWEventMask;

	CNFGWindow = XCreateWindow(CNFGDisplay, RootWindow(CNFGDisplay, 0), 1, 1, w, h, 0, depth, InputOutput, CNFGVisual, mask, &attr );
	XMapWindow(CNFGDisplay, CNFGWindow);
	XFlush(CNFGDisplay);

	InternalLinkScreenAndGo( WindowName );

	Atom WM_DELETE_WINDOW = XInternAtom( CNFGDisplay, "WM_DELETE_WINDOW", False );
	XSetWMProtocols( CNFGDisplay, CNFGWindow, &WM_DELETE_WINDOW, 1 );

	XSelectInput( CNFGDisplay, CNFGWindow, KeyPressMask | KeyReleaseMask | ButtonPressMask | ButtonReleaseMask | ExposureMask | PointerMotionMask );

#ifdef CNFGOGL
	glXMakeCurrent( CNFGDisplay, CNFGWindow, CNFGCtx );
#endif
	return 0;
}

void CNFGHandleInput()
{
	static int ButtonsDown;
	XEvent report;

	int bKeyDirection = 1;

	while( XPending( CNFGDisplay ) )
	{
		XNextEvent(CNFGDisplay, &report);

		bKeyDirection = 1;
		switch  (report.type)
		{
		case NoExpose:
			break;
		case Expose:
			XGetWindowAttributes( CNFGDisplay, CNFGWindow, &CNFGWinAtt );
			if( CNFGPixmap ) XFreePixmap( CNFGDisplay, CNFGPixmap );
			CNFGPixmap = XCreatePixmap( CNFGDisplay, CNFGWindow, CNFGWinAtt.width, CNFGWinAtt.height, CNFGWinAtt.depth );
			if( CNFGGC ) XFreeGC( CNFGDisplay, CNFGGC );
			CNFGGC = XCreateGC(CNFGDisplay, CNFGPixmap, 0, 0);
			break;
		case KeyRelease:
			bKeyDirection = 0;
		case KeyPress:
			HandleKey( XLookupKeysym(&report.xkey, 0), bKeyDirection );
			break;
		case ButtonRelease:
			bKeyDirection = 0;
		case ButtonPress:
			HandleButton( report.xbutton.x, report.xbutton.y, report.xbutton.button, bKeyDirection );
			ButtonsDown = (ButtonsDown & (~(1<<report.xbutton.button))) | ( bKeyDirection << report.xbutton.button );

			//Intentionall fall through -- we want to send a motion in event of a button as well.
		case MotionNotify:
			HandleMotion( report.xmotion.x, report.xmotion.y, ButtonsDown>>1 );
			break;
		case ClientMessage:
			// Only subscribed to WM_DELETE_WINDOW, so just exit
			exit( 0 );
			break;
		default:
			printf( "Event: %d\n", report.type );
		}
	}
}


void CNFGUpdateScreenWithBitmap( unsigned long * data, int w, int h )
{
	static XImage *xi;
	static int depth;
	static int lw, lh;
	static unsigned char * lbuffer;
	int r;

	if( !xi )
	{
		int screen = DefaultScreen(CNFGDisplay);
		depth = DefaultDepth(CNFGDisplay, screen)/8;
//		xi = XCreateImage(CNFGDisplay, DefaultVisual( CNFGDisplay, DefaultScreen(CNFGDisplay) ), depth*8, ZPixmap, 0, (char*)data, w, h, 32, w*4 );
//		lw = w;
//		lh = h;
	}

	if( lw != w || lh != h )
	{
		if( xi ) free( xi );
		xi = XCreateImage(CNFGDisplay, CNFGVisual, depth*8, ZPixmap, 0, (char*)data, w, h, 32, w*4 );
		lw = w;
		lh = h;
	}

	XPutImage(CNFGDisplay, CNFGWindow, CNFGWindowGC, xi, 0, 0, 0, 0, w, h );
}


#ifdef CNFGOGL

void   CNFGSetVSync( int vson )
{
	void (*glfn)( int );
	glfn = (void (*)( int ))CNFGGetExtension( "glXSwapIntervalMESA" );	if( glfn ) glfn( vson );
	glfn = (void (*)( int ))CNFGGetExtension( "glXSwapIntervalSGI" );	if( glfn ) glfn( vson );
	glfn = (void (*)( int ))CNFGGetExtension( "glXSwapIntervalEXT" );	if( glfn ) glfn( vson );
}

void CNFGSwapBuffers()
{
	glFlush();
	glFinish();
	glXSwapBuffers( CNFGDisplay, CNFGWindow );
}
#endif

#if !defined( RASTERIZER ) && !defined( CNFGOGL)


uint32_t CNFGColor( uint32_t RGB )
{
	unsigned char red = RGB & 0xFF;
	unsigned char grn = ( RGB >> 8 ) & 0xFF;
	unsigned char blu = ( RGB >> 16 ) & 0xFF;
	CNFGLastColor = RGB;
	unsigned long color = (red<<16)|(grn<<8)|(blu);
	XSetForeground(CNFGDisplay, CNFGGC, color);
	return color;
}

void CNFGClearFrame()
{
	XGetWindowAttributes( CNFGDisplay, CNFGWindow, &CNFGWinAtt );
	XSetForeground(CNFGDisplay, CNFGGC, CNFGColor(CNFGBGColor) );	
	XFillRectangle(CNFGDisplay, CNFGPixmap, CNFGGC, 0, 0, CNFGWinAtt.width, CNFGWinAtt.height );
}

void CNFGSwapBuffers()
{
	XCopyArea(CNFGDisplay, CNFGPixmap, CNFGWindow, CNFGWindowGC, 0,0,CNFGWinAtt.width,CNFGWinAtt.height,0,0);
	XFlush(CNFGDisplay);
	if( FullScreen )
		XSetInputFocus( CNFGDisplay, CNFGWindow, RevertToParent, CurrentTime );
}

void CNFGTackSegment( short x1, short y1, short x2, short y2 )
{
	XDrawLine( CNFGDisplay, CNFGPixmap, CNFGGC, x1, y1, x2, y2 );
}

void CNFGTackPixel( short x1, short y1 )
{
	XDrawPoint( CNFGDisplay, CNFGPixmap, CNFGGC, x1, y1 );
}

void CNFGTackRectangle( short x1, short y1, short x2, short y2 )
{
	XFillRectangle(CNFGDisplay, CNFGPixmap, CNFGGC, x1, y1, x2-x1, y2-y1 );
}

void CNFGTackPoly( RDPoint * points, int verts )
{
	XFillPolygon(CNFGDisplay, CNFGPixmap, CNFGGC, (XPoint *)points, 3, Convex, CoordModeOrigin );
}

void CNFGInternalResize( short x, short y ) { }

#else
#include "CNFGRasterizer.h"
#endif


