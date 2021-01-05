//Copyright (c) 2011 <>< Charles Lohr - Under the MIT/x11 or NewBSD License you choose.
//Portion from: http://en.wikibooks.org/wiki/Windows_Programming/Window_Creation


#include "CNFGFunctions.h"
#include <windows.h>
#include <stdlib.h>
#if !defined(__FreeBSD__) && !defined(__APPLE__)
#include <malloc.h> //for alloca
#endif

static HBITMAP lsBitmap;
static HINSTANCE lhInstance;
static HWND lsHWND;
static HDC lsWindowHDC;
static HDC lsHDC;

#ifdef RASTERIZER
#include "CNFGRasterizer.h"

void InternalHandleResize()
{
	if( lsBitmap ) DeleteObject( lsBitmap );

	CNFGInternalResize( bufferx, buffery );
	lsBitmap = CreateBitmap( bufferx, buffery, 1, 32, buffer );
	SelectObject( lsHDC, lsBitmap );
}
#else
static int bufferx, buffery;
static int bufferx, buffery;
static void InternalHandleResize();
#endif


#ifdef CNFGOGL
#include <GL/gl.h>
static HGLRC           hRC=NULL; 
static void InternalHandleResize() { }
void CNFGSwapBuffers()
{
	SwapBuffers(lsWindowHDC);
}
#endif

void CNFGGetDimensions( short * x, short * y )
{
	static int lastx, lasty;
	RECT window;
	GetClientRect( lsHWND, &window );
	bufferx = ( window.right - window.left);
	buffery = ( window.bottom - window.top);
	if( bufferx != lastx || buffery != lasty )
	{
		lastx = bufferx;
		lasty = buffery;
		InternalHandleResize();
	}
	*x = (short)bufferx;
	*y = (short)buffery;
}


void CNFGUpdateScreenWithBitmap( unsigned long * data, int w, int h )
{
	RECT r;

	int a = SetBitmapBits(lsBitmap,w*h*4,data);
	a = BitBlt(lsWindowHDC, 0, 0, w, h, lsHDC, 0, 0, SRCCOPY);
	UpdateWindow( lsHWND );

	int thisw, thish;

	//Check to see if the window is closed.
	if( !IsWindow( lsHWND ) )
	{
		exit( 0 );
	}

	GetClientRect( lsHWND, &r );
	thisw = r.right - r.left;
	thish = r.bottom - r.top;
	if( thisw != bufferx || thish != buffery )
	{
		bufferx = thisw;
		buffery = thish;
		InternalHandleResize();
	}
}


void CNFGTearDown()
{
	PostQuitMessage(0);
}

//This was from the article
LRESULT CALLBACK MyWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch(msg)
	{
	case WM_DESTROY:
		HandleDestroy();
		CNFGTearDown();
		return 0;
	}
	return DefWindowProc(hwnd, msg, wParam, lParam);
}

//This was from the article, too... well, mostly.
int CNFGSetup( const char * name_of_window, int width, int height )
{
	static LPSTR szClassName = "MyClass";
	RECT client, window;
	WNDCLASS wnd;
	int w, h, wd, hd;
	HINSTANCE hInstance = GetModuleHandle(NULL);

	bufferx = width;
	buffery = height;

	wnd.style = CS_HREDRAW | CS_VREDRAW; //we will explain this later
	wnd.lpfnWndProc = MyWndProc;
	wnd.cbClsExtra = 0;
	wnd.cbWndExtra = 0;
	wnd.hInstance = hInstance;
	wnd.hIcon = LoadIcon(NULL, IDI_APPLICATION); //default icon
	wnd.hCursor = LoadCursor(NULL, IDC_ARROW);   //default arrow mouse cursor
	wnd.hbrBackground = (HBRUSH)(COLOR_BACKGROUND);
	wnd.lpszMenuName = NULL;                     //no menu
	wnd.lpszClassName = szClassName;

	if(!RegisterClass(&wnd))                     //register the WNDCLASS
	{
		MessageBox(NULL, "This Program Requires Windows NT", "Error", MB_OK);
	}

	lsHWND = CreateWindow(szClassName,
		name_of_window,      //name_of_window,
		WS_OVERLAPPEDWINDOW, //basic window style
		CW_USEDEFAULT,
		CW_USEDEFAULT,       //set starting point to default value
		bufferx,
		buffery,        //set all the dimensions to default value
		NULL,                //no parent window
		NULL,                //no menu
		hInstance,
		NULL);               //no parameters to pass

	lsWindowHDC = GetDC( lsHWND );

#ifdef CNFGOGL
	//From NeHe
	static  PIXELFORMATDESCRIPTOR pfd =
	{
		sizeof(PIXELFORMATDESCRIPTOR),
		1,
		PFD_DRAW_TO_WINDOW |
		PFD_SUPPORT_OPENGL |
		PFD_DOUBLEBUFFER,
		PFD_TYPE_RGBA,
		32,
		0, 0, 0, 0, 0, 0, 
		0,
		0,
		0,
		0, 0, 0, 0,
		16,
		0,
		0,
		PFD_MAIN_PLANE,
		0,
		0, 0, 0
	};
	GLuint      PixelFormat = ChoosePixelFormat( lsWindowHDC, &pfd );
	if( !SetPixelFormat( lsWindowHDC, PixelFormat, &pfd ) )
	{
		MessageBox( 0, "Could not create PFD for OpenGL Context\n", 0, 0 );
		exit( -1 );
	}
	if (!(hRC=wglCreateContext(lsWindowHDC)))                   // Are We Able To Get A Rendering Context?
	{
		MessageBox( 0, "Could not create OpenGL Context\n", 0, 0 );
		exit( -1 );
	}
	if(!wglMakeCurrent(lsWindowHDC,hRC))                        // Try To Activate The Rendering Context
	{
		MessageBox( 0, "Could not current OpenGL Context\n", 0, 0 );
		exit( -1 );
	}
#endif

	lsHDC = CreateCompatibleDC( lsWindowHDC );
	lsBitmap = CreateCompatibleBitmap( lsWindowHDC, bufferx, buffery );
	SelectObject( lsHDC, lsBitmap );

	//lsClearBrush = CreateSolidBrush( CNFGBGColor );
	//lsHBR = CreateSolidBrush( 0xFFFFFF );
	//lsHPEN = CreatePen( PS_SOLID, 0, 0xFFFFFF );

	ShowWindow(lsHWND, 1);              //display the window on the screen

	//Once set up... we have to change the window's borders so we get the client size right.
	GetClientRect( lsHWND, &client );
	GetWindowRect( lsHWND, &window );
	w = ( window.right - window.left);
	h = ( window.bottom - window.top);
	wd = w - client.right;
	hd = h - client.bottom;
	MoveWindow( lsHWND, window.left, window.top, bufferx + wd, buffery + hd, 1 );

	InternalHandleResize();
	return 0;
}

void CNFGHandleInput()
{
	int ldown = 0;

	MSG msg;
	while( PeekMessage( &msg, lsHWND, 0, 0xFFFF, 1 ) )
	{
		TranslateMessage(&msg);

		switch( msg.message )
		{
		case WM_MOUSEMOVE:
			HandleMotion( (msg.lParam & 0xFFFF), (msg.lParam>>16) & 0xFFFF, ( (msg.wParam & 0x01)?1:0) | ((msg.wParam & 0x02)?2:0) | ((msg.wParam & 0x10)?4:0) );
			break;
		case WM_LBUTTONDOWN:	HandleButton( (msg.lParam & 0xFFFF), (msg.lParam>>16) & 0xFFFF, 1, 1 ); break;
		case WM_RBUTTONDOWN:	HandleButton( (msg.lParam & 0xFFFF), (msg.lParam>>16) & 0xFFFF, 2, 1 ); break;
		case WM_MBUTTONDOWN:	HandleButton( (msg.lParam & 0xFFFF), (msg.lParam>>16) & 0xFFFF, 3, 1 ); break;
		case WM_LBUTTONUP:		HandleButton( (msg.lParam & 0xFFFF), (msg.lParam>>16) & 0xFFFF, 1, 0 ); break;
		case WM_RBUTTONUP:		HandleButton( (msg.lParam & 0xFFFF), (msg.lParam>>16) & 0xFFFF, 2, 0 ); break;
		case WM_MBUTTONUP:		HandleButton( (msg.lParam & 0xFFFF), (msg.lParam>>16) & 0xFFFF, 3, 0 ); break;
		case WM_KEYDOWN:
		case WM_KEYUP:
			HandleKey( tolower( (int)(msg.wParam) ), (msg.message==WM_KEYDOWN) );
			break;
		default:
			DispatchMessage(&msg);
			break;
		}
	}
}

#ifndef CNFGOGL

#ifndef RASTERIZER

static HBITMAP lsBackBitmap;
static HDC lsWindowHDC;
static HBRUSH lsHBR;
static HPEN lsHPEN;
static HBRUSH lsClearBrush;

static void InternalHandleResize()
{
	DeleteObject( lsBackBitmap );
	lsBackBitmap = CreateCompatibleBitmap( lsHDC, bufferx, buffery );
	SelectObject( lsHDC, lsBackBitmap );
}

uint32_t CNFGColor( uint32_t RGB )
{
	CNFGLastColor = RGB;

	DeleteObject( lsHBR );
	lsHBR = CreateSolidBrush( RGB );
	SelectObject( lsHDC, lsHBR );

	DeleteObject( lsHPEN );
	lsHPEN = CreatePen( PS_SOLID, 0, RGB );
	SelectObject( lsHDC, lsHPEN );

	return RGB;
}

void CNFGTackSegment( short x1, short y1, short x2, short y2 )
{
	POINT pt[2] = { {x1, y1}, {x2, y2} };
	Polyline( lsHDC, pt, 2 );
	SetPixel( lsHDC, x1, y1, CNFGLastColor );
	SetPixel( lsHDC, x2, y2, CNFGLastColor );
}

void CNFGTackRectangle( short x1, short y1, short x2, short y2 )
{
	RECT r;
	if( x1 < x2 ) { r.left = x1; r.right = x2; }
	else          { r.left = x2; r.right = x1; }
	if( y1 < y2 ) { r.top = y1; r.bottom = y2; }
	else          { r.top = y2; r.bottom = y1; }
	FillRect( lsHDC, &r, lsHBR );
}

void CNFGClearFrame()
{
	RECT r = { 0, 0, bufferx, buffery };
	DeleteObject( lsClearBrush  );
	lsClearBrush = CreateSolidBrush( CNFGBGColor );
	SelectObject( lsHDC, lsClearBrush );
	FillRect( lsHDC, &r, lsClearBrush);
}

void CNFGTackPoly( RDPoint * points, int verts )
{
	int i;
	POINT * t = (POINT*)alloca( sizeof( POINT ) * verts );
	for( i = 0; i < verts; i++ )
	{
		t[i].x = points[i].x;
		t[i].y = points[i].y;
	}
	Polygon( lsHDC, t, verts );
}


void CNFGTackPixel( short x1, short y1 )
{
	SetPixel( lsHDC, x1, y1, CNFGLastColor );
}

void CNFGSwapBuffers()
{
	int thisw, thish;

	RECT r;
	BitBlt( lsWindowHDC, 0, 0, bufferx, buffery, lsHDC, 0, 0, SRCCOPY );
	UpdateWindow( lsHWND );
	//Check to see if the window is closed.
	if( !IsWindow( lsHWND ) )
	{
		exit( 0 );
	}

	GetClientRect( lsHWND, &r );
	thisw = r.right - r.left;
	thish = r.bottom - r.top;

	if( thisw != bufferx || thish != buffery )
	{
		bufferx = thisw;
		buffery = thish;
		InternalHandleResize();
	}
}

void CNFGInternalResize( short bufferx, short  buffery ) { }
#endif

#endif

