//Copyright (c) 2011 <>< Charles Lohr - Under the MIT/x11 or NewBSD License you choose.

#ifndef _DRAWFUCNTIONS_H
#define _DRAWFUCNTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct {
    short x, y; 
} RDPoint; 

extern int CNFGPenX, CNFGPenY;
extern uint32_t CNFGBGColor;
extern uint32_t CNFGLastColor;
extern uint32_t CNFGDialogColor; //background for boxes

void CNFGDrawText( const char * text, int scale );
void CNFGDrawBox(  int x1, int y1, int x2, int y2 );
void CNFGGetTextExtents( const char * text, int * w, int * h, int textsize  );
void CNFGDrawTextbox( int x, int y, const char * text, int textsize ); //ignores pen.

//To be provided by driver.
uint32_t CNFGColor( uint32_t RGB );
void CNFGUpdateScreenWithBitmap( unsigned long * data, int w, int h );
void CNFGTackPixel( short x1, short y1 );
void CNFGTackSegment( short x1, short y1, short x2, short y2 );
void CNFGTackRectangle( short x1, short y1, short x2, short y2 );
void CNFGTackPoly( RDPoint * points, int verts );
void CNFGClearFrame();
void CNFGSwapBuffers();

void CNFGGetDimensions( short * x, short * y );
void CNFGTearDown();
void CNFGSetup( const char * WindowName, int w, int h );
void CNFGSetupFullscreen( const char * WindowName, int screen_number );
void CNFGHandleInput();


//You must provide:
void HandleKey( int keycode, int bDown );
void HandleButton( int x, int y, int button, int bDown );
void HandleMotion( int x, int y, int mask );


#ifdef __cplusplus
};
#endif


#endif

