//Copyright (c) 2011, 2017 <>< Charles Lohr - Under the MIT/x11 or NewBSD License you choose.

#ifndef _DRAWFUCNTIONS_H
#define _DRAWFUCNTIONS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#ifndef CN_EXPORT
#if defined(_WIN32) && !defined(TCC)
#define CN_EXPORT __declspec(dllexport)
#else
#define CN_EXPORT __attribute__((visibility("default")))
#endif
#endif

typedef struct {
    short x, y; 
} RDPoint; 

extern int CNFGPenX, CNFGPenY;
extern uint32_t CNFGBGColor;
extern uint32_t CNFGLastColor;
extern uint32_t CNFGDialogColor; //background for boxes

CN_EXPORT void CNFGDrawText(const char *text, int scale);
CN_EXPORT void CNFGDrawBox(int x1, int y1, int x2, int y2);
CN_EXPORT void CNFGGetTextExtents(const char *text, int *w, int *h, int textsize);
CN_EXPORT void CNFGDrawTextbox(int x, int y, const char *text, int textsize); // ignores pen.

//To be provided by driver.
CN_EXPORT uint32_t CNFGColor(uint32_t RGB);
CN_EXPORT void CNFGUpdateScreenWithBitmap(unsigned long *data, int w, int h);
CN_EXPORT void CNFGTackPixel(short x1, short y1);
CN_EXPORT void CNFGTackSegment(short x1, short y1, short x2, short y2);
CN_EXPORT void CNFGTackRectangle(short x1, short y1, short x2, short y2);
CN_EXPORT void CNFGTackPoly(RDPoint *points, int verts);
CN_EXPORT void CNFGClearFrame();
CN_EXPORT void CNFGSwapBuffers();

CN_EXPORT void CNFGGetDimensions(short *x, short *y);
CN_EXPORT int CNFGSetup(const char *WindowName, int w, int h); // return 0 if ok.
CN_EXPORT void CNFGSetupFullscreen(const char *WindowName, int screen_number);
CN_EXPORT void CNFGHandleInput();

//You must provide:
CN_EXPORT void HandleKey(int keycode, int bDown);
CN_EXPORT void HandleButton(int x, int y, int button, int bDown);
CN_EXPORT void HandleMotion(int x, int y, int mask);
CN_EXPORT void HandleDestroy();

//Internal function for resizing rasterizer for rasterizer-mode.
CN_EXPORT void CNFGInternalResize(short x, short y); // don't call this.

//Not available on all systems.  Use The OGL portion with care.
#ifdef CNFGOGL
CN_EXPORT void CNFGSetVSync(int vson);
CN_EXPORT void *CNFGGetExtension(const char *extname);
#endif

#ifdef __cplusplus
};
#endif


#endif

