//Copyright (c) 2017 <>< Charles Lohr - Under the MIT/x11 or NewBSD License you choose.

#include "CNFGFunctions.h"

static int w, h;
void CNFGGetDimensions( short * x, short * y )
{
	*x = w;
	*y = h;
}

static void InternalLinkScreenAndGo( const char * WindowName )
{
}

void CNFGSetupFullscreen( const char * WindowName, int screen_no )
{
	CNFGSetup( WindowName, 640, 480 );
}


void CNFGTearDown()
{
}

int CNFGSetup( const char * WindowName, int sw, int sh )
{
	w = sw;
	h = sh;
	return 0;
}

void CNFGHandleInput()
{
}


void CNFGUpdateScreenWithBitmap( unsigned long * data, int w, int h )
{
}

#ifndef RASTERIZER


uint32_t CNFGColor( uint32_t RGB )
{
}
#endif

void CNFGClearFrame()
{
}

void CNFGSwapBuffers()
{
}

void CNFGTackSegment( short x1, short y1, short x2, short y2 )
{
}

void CNFGTackPixel( short x1, short y1 )
{
}

void CNFGTackRectangle( short x1, short y1, short x2, short y2 )
{
}

void CNFGTackPoly( RDPoint * points, int verts )
{
}

