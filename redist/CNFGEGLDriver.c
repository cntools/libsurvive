/*
 * Copyright (c) 2011-2013 Luc Verhaegen <libv@skynet.be>
 * Copyright (c) 2018 <>< Charles Lohr
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sub license,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "CNFGFunctions.h"
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>
#include <EGL/egl.h>
#include <GLES2/gl2.h>

#define EGL_ZBITS 16
#define EGL_IMMEDIATE_SIZE 2048

#ifdef USE_EGL_X
	#error This feature has never been completed or tested.
	Display *XDisplay;
	Window XWindow;
#else
	typedef enum
	{
		FBDEV_PIXMAP_DEFAULT = 0,
		FBDEV_PIXMAP_SUPPORTS_UMP = (1<<0),
		FBDEV_PIXMAP_ALPHA_FORMAT_PRE = (1<<1),
		FBDEV_PIXMAP_COLORSPACE_sRGB = (1<<2),
		FBDEV_PIXMAP_EGL_MEMORY = (1<<3)        /* EGL allocates/frees this memory */
	} fbdev_pixmap_flags;

	typedef struct fbdev_window
	{
		unsigned short width;
		unsigned short height;
	} fbdev_window;

	typedef struct fbdev_pixmap
	{
		unsigned int height;
		unsigned int width;
		unsigned int bytes_per_pixel;
		unsigned char buffer_size;
		unsigned char red_size;
		unsigned char green_size;
		unsigned char blue_size;
		unsigned char alpha_size;
		unsigned char luminance_size;
		fbdev_pixmap_flags flags;
		unsigned short *data;
		unsigned int format; /* extra format information in case rgbal is not enough, especially for YUV formats */
	} fbdev_pixmap;

struct fbdev_window native_window;
#endif


static const char *default_vertex_shader_source =
	"attribute vec4 aPosition;    \n"
	"attribute vec4 aColor;       \n"
	"uniform vec4 screenscale;    \n"
	"                             \n"
	"varying vec4 vColor;         \n"
	"                             \n"
	"void main()                  \n"
	"{                            \n"
	"    vColor = aColor;         \n"
	"    gl_Position = vec4( -1.0, 1.0, 0.0, 0.0 ) + aPosition * screenscale; \n"
	"}                            \n";
static const char *default_fragment_shader_source =
	"precision mediump float;     \n"
	"                             \n"
	"varying vec4 vColor;         \n"
	"                             \n"
	"void main()                  \n"
	"{                            \n"
	"    gl_FragColor = vColor;   \n"
	"}                            \n";
GLuint default_vertex_shader;
GLuint default_fragment_shader;
GLuint default_screenscale_offset;


static EGLint const config_attribute_list[] = {
	EGL_RED_SIZE, 8,
	EGL_GREEN_SIZE, 8,
	EGL_BLUE_SIZE, 8,
	EGL_ALPHA_SIZE, 8,
	EGL_BUFFER_SIZE, 32,
	EGL_STENCIL_SIZE, 0,
	EGL_DEPTH_SIZE, EGL_ZBITS,
	EGL_SAMPLES, 4,
	EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
	EGL_SURFACE_TYPE, EGL_WINDOW_BIT | EGL_PIXMAP_BIT,
	EGL_NONE
};


static EGLint window_attribute_list[] = {
	EGL_NONE
};

static const EGLint context_attribute_list[] = {
	EGL_CONTEXT_CLIENT_VERSION, 2,
	EGL_NONE
};

EGLDisplay egl_display;
EGLSurface egl_surface;
uint32_t egl_currentcolor;

uint32_t CNFGColor( uint32_t RGB ) { egl_currentcolor = RGB|((RGB<0x1000000)?0xff000000:0); }

void CNFGUpdateScreenWithBitmap( unsigned long * data, int w, int h )
{
	fprintf( stderr, "Screen bitmap update not permitted.\n" );
	//Not implemented
}



int egl_immediate_stride;
int egl_immediate_size;
int egl_immediate_draw_mode;
int16_t         egl_immediate_geo_buffer[EGL_IMMEDIATE_SIZE*4];
int16_t	*	 egl_immediate_geo_ptr;
uint32_t 		 egl_immediate_color_buffer[EGL_IMMEDIATE_SIZE*4];


static GLfloat vVertices[] = {  0.0f,  0.5f, 0.0f,
			       -0.5f, -0.5f, 0.0f,
				0.5f, -0.5f, 0.0f };
static GLfloat vColors[] = {1.0f, 0.0f, 0.0f, 1.0f,
			    0.0f, 1.0f, 0.0f, 1.0f,
			    0.0f, 0.0f, 1.0f, 1.0f};


void FlushRender()
{
	if( egl_immediate_size && egl_immediate_draw_mode >= 0 )
	{
		//printf( "%d %d %d %d\n",egl_immediate_geo_buffer[0],egl_immediate_geo_buffer[1], egl_immediate_geo_buffer[2], egl_immediate_geo_buffer[3]  );
		//printf( "%d*%d ", egl_immediate_size,egl_immediate_stride );
		glVertexAttribPointer(0, egl_immediate_stride, GL_SHORT, GL_FALSE, 0, egl_immediate_geo_buffer);
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(1, 4, GL_UNSIGNED_BYTE, GL_TRUE, 0, egl_immediate_color_buffer);
		glEnableVertexAttribArray(1);
		glDrawArrays(egl_immediate_draw_mode, 0, egl_immediate_size);
	
		egl_immediate_size = 0;
	}

	egl_immediate_geo_ptr = egl_immediate_geo_buffer;
	egl_immediate_draw_mode = -1;
}

void CNFGTackPixel( short x1, short y1 )
{
	if( egl_immediate_draw_mode != GL_POINTS 
		|| (egl_immediate_size+2) >= EGL_IMMEDIATE_SIZE )	FlushRender();
	egl_immediate_geo_ptr[0] = x1;
	egl_immediate_geo_ptr[1] = y1;
	egl_immediate_geo_ptr += 2;
	egl_immediate_color_buffer[egl_immediate_size] = egl_currentcolor;
	egl_immediate_size++;
	egl_immediate_draw_mode = GL_POINTS;
	egl_immediate_stride = 2;
}

void CNFGTackSegment( short x1, short y1, short x2, short y2 )
{
	if( egl_immediate_draw_mode != GL_LINES 
		|| egl_immediate_size >= EGL_IMMEDIATE_SIZE )	FlushRender();
	egl_immediate_geo_ptr[0] = x1;
	egl_immediate_geo_ptr[1] = y1;
	egl_immediate_geo_ptr[2] = x2;
	egl_immediate_geo_ptr[3] = y2;
	egl_immediate_geo_ptr += 4;
	egl_immediate_color_buffer[egl_immediate_size+0] = egl_currentcolor;
	egl_immediate_color_buffer[egl_immediate_size+1] = egl_currentcolor;
	egl_immediate_size+=2;
	egl_immediate_draw_mode = GL_LINES;
	egl_immediate_stride = 2;
}

void CNFGTackRectangle( short x1, short y1, short x2, short y2 )
{
	if( egl_immediate_draw_mode != GL_TRIANGLES
		|| (egl_immediate_size+12) >= EGL_IMMEDIATE_SIZE )	FlushRender();
	/*
			*-*
			|/|
			*-*   Don't forget to go clockwise!
	*/
	egl_immediate_geo_ptr[0] = x1;
	egl_immediate_geo_ptr[1] = y1;
	egl_immediate_geo_ptr[2] = x2;
	egl_immediate_geo_ptr[3] = y1;
	egl_immediate_geo_ptr[4] = x1;
	egl_immediate_geo_ptr[5] = y2;
	egl_immediate_geo_ptr[6] = x1;
	egl_immediate_geo_ptr[7] = y2;
	egl_immediate_geo_ptr[8] = x2;
	egl_immediate_geo_ptr[9] = y1;
	egl_immediate_geo_ptr[10] = x2;
	egl_immediate_geo_ptr[11] = y2;
	egl_immediate_geo_ptr += 12;
	egl_immediate_color_buffer[egl_immediate_size+0] = egl_currentcolor;
	egl_immediate_color_buffer[egl_immediate_size+1] = egl_currentcolor;
	egl_immediate_color_buffer[egl_immediate_size+2] = egl_currentcolor;
	egl_immediate_color_buffer[egl_immediate_size+3] = egl_currentcolor;
	egl_immediate_color_buffer[egl_immediate_size+4] = egl_currentcolor;
	egl_immediate_color_buffer[egl_immediate_size+5] = egl_currentcolor;
	egl_immediate_size+=6;
	egl_immediate_draw_mode = GL_TRIANGLES;
	egl_immediate_stride = 2;
}

void CNFGTackPoly( RDPoint * points, int verts )
{
	if( egl_immediate_draw_mode != GL_TRIANGLES
		|| egl_immediate_size+verts*2 >= EGL_IMMEDIATE_SIZE )	FlushRender();

	int i;
	for( i = 0; i < verts; i++ )
	{
		egl_immediate_geo_ptr[0] = points[i].x;
		egl_immediate_geo_ptr[1] = points[i].y;
		egl_immediate_geo_ptr += 2;
		egl_immediate_color_buffer[egl_immediate_size] = egl_currentcolor;
		egl_immediate_size++;
	}
	egl_immediate_draw_mode = GL_TRIANGLES;
	egl_immediate_stride = 2;
}

void CNFGClearFrame()
{
	glClearColor( (CNFGBGColor&0xff)/255.0, 
		(CNFGBGColor&0xff00)/65280.0, 
		(CNFGBGColor&0xff0000)/16711680.0,
		((CNFGBGColor&0xff000000)>>24)/255.0);
	glClear(GL_COLOR_BUFFER_BIT /*| GL_DEPTH_BUFFER_BIT*/);
}

void CNFGSwapBuffers()
{
	FlushRender();
	eglSwapBuffers(egl_display, egl_surface);
}

void CNFGGetDimensions( short * x, short * y )
{
	*x = native_window.width;
	*y = native_window.height;
}

int CNFGSetup( const char * WindowName, int w, int h )
{
	int ret;
	EGLint egl_major, egl_minor;
	EGLConfig config;
	EGLint num_config;
	EGLContext context;
	GLuint program;

#ifdef USE_EGL_X
	XDisplay = XOpenDisplay(NULL);
	if (!XDisplay) {
		fprintf(stderr, "Error: failed to open X display.\n");
		return -1;
	}

	Window XRoot = DefaultRootWindow(XDisplay);

	XSetWindowAttributes XWinAttr;
	XWinAttr.event_mask  =  ExposureMask | PointerMotionMask;

	XWindow = XCreateWindow(XDisplay, XRoot, 0, 0, WIDTH, HEIGHT, 0,
				CopyFromParent, InputOutput,
				CopyFromParent, CWEventMask, &XWinAttr);

	Atom XWMDeleteMessage =
		XInternAtom(XDisplay, "WM_DELETE_WINDOW", False);

	XMapWindow(XDisplay, XWindow);
	XStoreName(XDisplay, XWindow, "Mali libs test");
	XSetWMProtocols(XDisplay, XWindow, &XWMDeleteMessage, 1);

	egl_display = eglGetDisplay((EGLNativeDisplayType) XDisplay);
#else
	native_window.width = w;
	native_window.height =h;
	egl_display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
#endif
	if (egl_display == EGL_NO_DISPLAY) {
		fprintf(stderr, "Error: No display found!\n");
		return -1;
	}

	if (!eglInitialize(egl_display, &egl_major, &egl_minor)) {
		fprintf(stderr, "Error: eglInitialise failed!\n");
		return -1;
	}

	printf("EGL Version: \"%s\"\n",
	       eglQueryString(egl_display, EGL_VERSION));
	printf("EGL Vendor: \"%s\"\n",
	       eglQueryString(egl_display, EGL_VENDOR));
	printf("EGL Extensions: \"%s\"\n",
	       eglQueryString(egl_display, EGL_EXTENSIONS));

	eglChooseConfig(egl_display, config_attribute_list, &config, 1,
			&num_config);

	context = eglCreateContext(egl_display, config, EGL_NO_CONTEXT,
				   context_attribute_list);
	if (context == EGL_NO_CONTEXT) {
		fprintf(stderr, "Error: eglCreateContext failed: 0x%08X\n",
			eglGetError());
		return -1;
	}

#ifdef USE_EGL_X
	egl_surface = eglCreateWindowSurface(egl_display, config, XWindow,
					     window_attribute_list);
#else
	egl_surface = eglCreateWindowSurface(egl_display, config,
					     (EGLNativeWindowType)&native_window,
					     window_attribute_list);
#endif
	if (egl_surface == EGL_NO_SURFACE) {
		fprintf(stderr, "Error: eglCreateWindowSurface failed: "
			"0x%08X\n", eglGetError());
		return -1;
	}

	int width, height;
	if (!eglQuerySurface(egl_display, egl_surface, EGL_WIDTH, &width) ||
	    !eglQuerySurface(egl_display, egl_surface, EGL_HEIGHT, &height)) {
		fprintf(stderr, "Error: eglQuerySurface failed: 0x%08X\n",
			eglGetError());
		return -1;
	}
	printf("Surface size: %dx%d\n", width, height);
	native_window.width = width;
	native_window.height = height;


	if (!eglMakeCurrent(egl_display, egl_surface, egl_surface, context)) {
		fprintf(stderr, "Error: eglMakeCurrent() failed: 0x%08X\n",
			eglGetError());
		return -1;
	}

	printf("GL Vendor: \"%s\"\n", glGetString(GL_VENDOR));
	printf("GL Renderer: \"%s\"\n", glGetString(GL_RENDERER));
	printf("GL Version: \"%s\"\n", glGetString(GL_VERSION));
	printf("GL Extensions: \"%s\"\n", glGetString(GL_EXTENSIONS));

	default_vertex_shader = glCreateShader(GL_VERTEX_SHADER);
	if (!default_vertex_shader) {
		fprintf(stderr, "Error: glCreateShader(GL_VERTEX_SHADER) "
			"failed: 0x%08X\n", glGetError());
		return -1;
	}

	glShaderSource(default_vertex_shader, 1, &default_vertex_shader_source, NULL);
	glCompileShader(default_vertex_shader);

	glGetShaderiv(default_vertex_shader, GL_COMPILE_STATUS, &ret);
	if (!ret) {
		char *log;

		fprintf(stderr, "Error: vertex shader compilation failed!\n");
		glGetShaderiv(default_vertex_shader, GL_INFO_LOG_LENGTH, &ret);

		if (ret > 1) {
			log = malloc(ret);
			glGetShaderInfoLog(default_vertex_shader, ret, NULL, log);
			fprintf(stderr, "%s", log);
		}
		return -1;
	}

	default_fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
	if (!default_fragment_shader) {
		fprintf(stderr, "Error: glCreateShader(GL_FRAGMENT_SHADER) "
			"failed: 0x%08X\n", glGetError());
		return -1;
	}

	glShaderSource(default_fragment_shader, 1, &default_fragment_shader_source, NULL);
	glCompileShader(default_fragment_shader);

	glGetShaderiv(default_fragment_shader, GL_COMPILE_STATUS, &ret);
	if (!ret) {
		char *log;

		fprintf(stderr, "Error: fragment shader compilation failed!\n");
		glGetShaderiv(default_fragment_shader, GL_INFO_LOG_LENGTH, &ret);

		if (ret > 1) {
			log = malloc(ret);
			glGetShaderInfoLog(default_fragment_shader, ret, NULL, log);
			fprintf(stderr, "%s", log);
		}
		return -1;
	}

	program = glCreateProgram();
	if (!program) {
		fprintf(stderr, "Error: failed to create program!\n");
		return -1;
	}

	glAttachShader(program, default_vertex_shader);
	glAttachShader(program, default_fragment_shader);

	glBindAttribLocation(program, 0, "aPosition");
	glBindAttribLocation(program, 1, "aColor");

	glLinkProgram(program);

	glGetProgramiv(program, GL_LINK_STATUS, &ret);
	if (!ret) {
		char *log;

		fprintf(stderr, "Error: program linking failed!\n");
		glGetProgramiv(program, GL_INFO_LOG_LENGTH, &ret);

		if (ret > 1) {
			log = malloc(ret);
			glGetProgramInfoLog(program, ret, NULL, log);
			fprintf(stderr, "%s", log);
		}
		return -1;
	}
	glUseProgram(program);
	default_screenscale_offset = glGetUniformLocation ( program , "screenscale" );
	glUniform4f( default_screenscale_offset, 2./width, -2./height, 1.0, 1.0 );

	egl_immediate_geo_ptr = &egl_immediate_geo_buffer[0];

	glLineWidth(10.0);
	//glEnable(GL_DEPTH_TEST);

	return 0;
}

void CNFGSetupFullscreen( const char * WindowName, int screen_number )
{
	fprintf( stderr, "You had better already be in full-screen mode.\n" );
}

void CNFGHandleInput()
{

#ifdef USE_EGL_X
	while (1) {
		XEvent event;

		XNextEvent(XDisplay, &event);

		if ((event.type == MotionNotify) ||
		    (event.type == Expose))
			Redraw(width, height);
		else if (event.type == ClientMessage) {
			if (event.xclient.data.l[0] == XWMDeleteMessage)
				break;
		}
	}
	XSetWMProtocols(XDisplay, XWindow, &XWMDeleteMessage, 0);
#endif
}


