//Copyright (c) 2011 <>< Charles Lohr - Under the MIT/x11 or NewBSD License you choose.
//Portion from: http://en.wikibooks.org/wiki/Windows_Programming/Window_Creation


#include "DrawFunctions.h"
#include <windows.h>
#include <stdlib.h>
#include <malloc.h> //for alloca

static HINSTANCE lhInstance;
static HWND lsHWND;
static HDC lsHDC;
static HBITMAP lsBackBitmap;
static HDC lsWindowHDC;
static HBRUSH lsHBR;
static HPEN lsHPEN;
static HBRUSH lsClearBrush;
static unsigned int lsLastWidth;
static unsigned int lsLastHeight;

static void InternalHandleResize()
{
	DeleteObject( lsBackBitmap );
	lsBackBitmap = CreateCompatibleBitmap( lsHDC, lsLastWidth, lsLastHeight );
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
	RECT r = { 0, 0, lsLastWidth, lsLastHeight };
	DeleteObject( lsClearBrush  );
	lsClearBrush = CreateSolidBrush( CNFGBGColor );
	SelectObject( lsHDC, lsClearBrush );

	FillRect( lsHDC, &r, lsClearBrush );
}

void CNFGSwapBuffers()
{
	int thisw, thish;
	RECT r;
	BitBlt( lsWindowHDC, 0, 0, lsLastWidth, lsLastHeight, lsHDC, 0, 0, SRCCOPY );
	UpdateWindow( lsHWND );

	//Check to see if the window is closed.
	if( !IsWindow( lsHWND ) )
	{
		exit( 0 );
	}

	GetClientRect( lsHWND, &r );
	thisw = r.right - r.left;
	thish = r.bottom - r.top;
	if( thisw != lsLastWidth || thish != lsLastHeight )
	{
		lsLastWidth = thisw;
		lsLastHeight = thish;
		InternalHandleResize();
	}
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

void CNFGGetDimensions( short * x, short * y )
{
	*x = lsLastWidth;
	*y = lsLastHeight;
}

//This was from the article
LRESULT CALLBACK MyWndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam)
{
	switch(msg)
	{
	case WM_DESTROY:
		CNFGTearDown();
		return 0;
	}
	return DefWindowProc(hwnd, msg, wParam, lParam);
}

void CNFGTearDown()
{
	PostQuitMessage(0);
}

//This was from the article, too... well, mostly.
void CNFGSetup( const char * name_of_window, int width, int height )
{
	static LPSTR szClassName = "MyClass";
	RECT client, window;
	WNDCLASS wnd;
	int w, h, wd, hd;
	HINSTANCE hInstance = GetModuleHandle(NULL);

	lsLastWidth = width;
	lsLastHeight = height;

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
		lsLastWidth,
		lsLastHeight,        //set all the dimensions to default value
		NULL,                //no parent window
		NULL,                //no menu
		hInstance,
		NULL);               //no parameters to pass


	lsWindowHDC = GetDC( lsHWND );
	lsHDC = CreateCompatibleDC( lsWindowHDC );
	lsBackBitmap = CreateCompatibleBitmap( lsWindowHDC, lsLastWidth, lsLastHeight );
	SelectObject( lsHDC, lsBackBitmap );

	lsClearBrush = CreateSolidBrush( CNFGBGColor );
	lsHBR = CreateSolidBrush( 0xFFFFFF );
	lsHPEN = CreatePen( PS_SOLID, 0, 0xFFFFFF );

	ShowWindow(lsHWND, 1);              //display the window on the screen

	//Once set up... we have to change the window's borders so we get the client size right.
	GetClientRect( lsHWND, &client );
	GetWindowRect( lsHWND, &window );
	w = ( window.right - window.left);
	h = ( window.bottom - window.top);
	wd = w - client.right;
	hd = h - client.bottom;
	MoveWindow( lsHWND, window.left, window.top, lsLastWidth + wd, lsLastHeight + hd, 1 );

	InternalHandleResize();
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
			HandleKey( tolower( (int)msg.wParam ), (msg.message==WM_KEYDOWN) );
			break;
		default:
			DispatchMessage(&msg);
			break;
		}
	}
}

