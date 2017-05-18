/*
 * This program is WAY overkill for what we need,
 *  but it works fine on Cocoa
 * It sets up a minimalist OpenGL Cocoa program with compatibility
 */

//Copyright (c) 2017 <>< David Chapman - Under the MIT/x11 or NewBSD License you choose.

#import <Cocoa/Cocoa.h>
// ToDo: This shouldn't need OpenGL, just to draw to framebuffer
//  but if you have Cocoa, you should have OpenGL, so it's okay
#import <OpenGL/OpenGL.h>
#import <OpenGL/gl3.h>
#include "CNFGFunctions.h"
#include "CNFGRasterizer.h"

















#define MultMatrix4x4_internal(A,B,C) { \
    int _y,_x,_k;\
    for (_y=0; _y<4; _y++) { \
        for (_x=0; _x<4; _x++) { \
            C[_y][_x] = 0.0f; \
            for (_k=0; _k<4; _k++) { \
                C[_y][_x] += B[_y][_k] * A[_k][_x]; \
            } \
        } \
    } \
}

#define PrintMatrix4x4(A) { \
    int _y,_x;\
    printf(#A "\n"); \
    for (_y=0; _y<4; _y++) { \
        for (_x=0; _x<4; _x++) { \
            printf("%0.6f\t", A[_y][_x]); \
        } \
        printf("\n"); \
    } \
} \

typedef struct {
    float x,y,z,    // Position
          r,g,b,a,  // Color
          u,v,w,    // Tex Coord
          nx,ny,nz; // Normals
} VertexArrayVert;
#define VertexArrayVertSet(_p,_x,_y,_z,_r,_g,_b,_a,_u,_v,_w) { _p.x=(_x); _p.y=(_y); _p.z=(_z); _p.r=(_r); _p.g=(_g); _p.b=(_b); _p.a=(_a); _p.u=(_u); _p.v=(_v); _p.w=(_w); }

typedef struct {
    VertexArrayVert *vert;   // An Interleaved Vertex Array
    int nVert;               // Numver of vertices
} VertexArray;

/*
 * VertexArrayDraw(A, mode)
 *   Draws an interleaved vertex array 'A'
 *   mode must be GL_TRIANGLES or GL_QUADS
 */

// Deprecated, OGL 1.1 code (vertex arrays)
#define VertexArrayDraw(A,mode) { \
    glEnableClientState(GL_VERTEX_ARRAY); \
    glEnableClientState(GL_COLOR_ARRAY);  \
    glEnableClientState(GL_TEXTURE_COORD_ARRAY); \
    glVertexPointer  (3, GL_FLOAT, sizeof(VertexArrayVert), &((A).vert[0].x)); \
    glColorPointer   (4, GL_FLOAT, sizeof(VertexArrayVert), &((A).vert[0].r)); \
    glTexCoordPointer(3, GL_FLOAT, sizeof(VertexArrayVert), &((A).vert[0].u)); \
    glDrawArrays     ((mode), 0, (A).nVert); \
    glDisableClientState(GL_VERTEX_ARRAY); \
    glDisableClientState(GL_COLOR_ARRAY);  \
    glDisableClientState(GL_TEXTURE_COORD_ARRAY); }

/*
 * Compatibility layer
 */
void oglCompatibilityInit(int maxVerts);

extern GLuint           ogl_compat_shader_program;
extern GLuint           ogl_compat_position_attribute;
extern GLuint           ogl_compat_colour_attribute;
extern GLuint           ogl_compat_texcoord_attribute;
extern GLuint           ogl_compat_texcoord_enabled;
extern GLuint           ogl_compat_texUnit;
extern GLuint           ogl_compat_modelview_projection;
extern GLuint           ogl_compat_vertex_array_object;
extern GLuint           ogl_compat_vertex_buffer;
extern int              ogl_compat_vertex_array_mode;
extern VertexArrayVert  ogl_compat_vertex_array_prev_vert;
extern VertexArray      ogl_compat_vertex_array;

#define oglEnableTexCoord()  { glUniform1i(ogl_compat_texcoord_enabled,1); }
#define oglDisableTexCoord() { glUniform1i(ogl_compat_texcoord_enabled,0); }
#define oglBindTexture(_tex) { glProgramUniform1i(ogl_compat_shader_program, ogl_compat_texUnit, 0); glActiveTexture(GL_TEXTURE0); glBindTexture(GL_TEXTURE_2D,_tex); }
#define oglBegin(mode) { ogl_compat_vertex_array_mode = mode; ogl_compat_vertex_array.nVert = 0; }
#define oglVertex3f(_x,_y,_z) { \
    ogl_compat_vertex_array_prev_vert.x = (_x);  ogl_compat_vertex_array_prev_vert.y = (_y);  ogl_compat_vertex_array_prev_vert.z = (_z); \
    ogl_compat_vertex_array.vert[ogl_compat_vertex_array.nVert] = ogl_compat_vertex_array_prev_vert; \
    ogl_compat_vertex_array.nVert++; }
#define oglVertex2f(_x,_y)  oglVertex3f(_x,_y,0.0f)
#define oglTexCoord3f(_u,_v,_w) { ogl_compat_vertex_array_prev_vert.u = (_u);  ogl_compat_vertex_array_prev_vert.v = (_v);  ogl_compat_vertex_array_prev_vert.w = (_w); }
#define oglTexCoord2f(_u,_v)    oglTexCoord3f(_u,_v,0.0f)
#define oglColor4f(_r,_g,_b,_a) { ogl_compat_vertex_array_prev_vert.r = (_r);  ogl_compat_vertex_array_prev_vert.g = (_g);  ogl_compat_vertex_array_prev_vert.b = (_b);  ogl_compat_vertex_array_prev_vert.a = (_a); }
#define oglColor3f(r,g,b) oglColor4f(r,g,b,1.0f)

#define oglEnd() { \
    glBindVertexArray(ogl_compat_vertex_array_object); \
    glBindBuffer(GL_ARRAY_BUFFER, ogl_compat_vertex_buffer); \
    glBufferData(GL_ARRAY_BUFFER, ogl_compat_vertex_array.nVert*sizeof(VertexArrayVert), ogl_compat_vertex_array.vert, GL_DYNAMIC_DRAW); \
    glEnableVertexAttribArray((GLuint)ogl_compat_position_attribute); \
    glEnableVertexAttribArray((GLuint)ogl_compat_colour_attribute  ); \
    glEnableVertexAttribArray((GLuint)ogl_compat_texcoord_attribute  ); \
    glVertexAttribPointer((GLuint)ogl_compat_position_attribute, 3, GL_FLOAT, GL_FALSE, sizeof(VertexArrayVert), (GLvoid *)(0)); \
    glVertexAttribPointer((GLuint)ogl_compat_colour_attribute  , 4, GL_FLOAT, GL_FALSE, sizeof(VertexArrayVert), (GLvoid *)(3*sizeof(float))); \
    glVertexAttribPointer((GLuint)ogl_compat_texcoord_attribute, 3, GL_FLOAT, GL_FALSE, sizeof(VertexArrayVert), (GLvoid *)(7*sizeof(float))); \
    glUseProgram(ogl_compat_shader_program); \
    float (*_modelview)[4]  = ogl_compat_matrix[ ogl_compat_matrix_level[OGL_MODELVIEW] ][ OGL_MODELVIEW ]; \
    float (*_projection)[4] = ogl_compat_matrix[ ogl_compat_matrix_level[OGL_PROJECTION] ][ OGL_PROJECTION ]; \
    float _MVP[4][4]; MultMatrix4x4_internal(_projection,_modelview,_MVP); \
    glUniformMatrix4fv(	ogl_compat_modelview_projection, 1, 0, (GLfloat*)(_MVP)); \
    glDrawArrays(ogl_compat_vertex_array_mode, 0, ogl_compat_vertex_array.nVert); \
    glDisableVertexAttribArray((GLuint)ogl_compat_position_attribute); \
    glDisableVertexAttribArray((GLuint)ogl_compat_colour_attribute  ); \
    glDisableVertexAttribArray((GLuint)ogl_compat_texcoord_attribute); }
//PrintMatrix4x4(_modelview); PrintMatrix4x4(_projection); PrintMatrix4x4(_MVP); \
//printf("mode %d nVerts %d\n", ogl_compat_vertex_array_mode, ogl_compat_vertex_array.nVert); PrintMatrix4x4(_MVP); \
/* Matrix math */
#define OGL_MODELVIEW  0
#define OGL_PROJECTION 1
#define OGL_MAX_MATRIX_PUSH 64

//                                  how many?      m/p dims
extern float ogl_compat_matrix[OGL_MAX_MATRIX_PUSH][2][4][4];   // the matrices
extern int   ogl_compat_matrix_level[2];                        // how many times have we pushed?
extern int   ogl_compat_matrix_mode;

#define oglMatrixMode(mode) { ogl_compat_matrix_mode=mode; }

#define oglLoadIdentity() { \
    float (*m)[4] = ogl_compat_matrix[ ogl_compat_matrix_level[ogl_compat_matrix_mode] ][ ogl_compat_matrix_mode ]; \
    m[0][0] = 1.0f; m[0][1] = 0.0f; m[0][2] = 0.0f; m[0][3] = 0.0f; \
    m[1][0] = 0.0f; m[1][1] = 1.0f; m[1][2] = 0.0f; m[1][3] = 0.0f; \
    m[2][0] = 0.0f; m[2][1] = 0.0f; m[2][2] = 1.0f; m[2][3] = 0.0f; \
    m[3][0] = 0.0f; m[3][1] = 0.0f; m[3][2] = 0.0f; m[3][3] = 1.0f; }
    
#define oglPushMatrix() { \
    if ( ogl_compat_matrix_level[ogl_compat_matrix_mode] >= OGL_MAX_MATRIX_PUSH ) {  printf("ERROR oglPushMatrix max push level (%d) exceeded\n", OGL_MAX_MATRIX_PUSH); exit(1); } \
    memcpy(ogl_compat_matrix[ogl_compat_matrix_level[0]+1], ogl_compat_matrix[ogl_compat_matrix_level[0]], 16*sizeof(float)); \
    memcpy(ogl_compat_matrix[ogl_compat_matrix_level[1]+1], ogl_compat_matrix[ogl_compat_matrix_level[1]], 16*sizeof(float)); \
    ogl_compat_matrix_level[0]++; ogl_compat_matrix_level[1]++; }
    
#define oglPopMatrix() { \
    if ( ogl_compat_matrix_level[ogl_compat_matrix_mode] < 0 ) {  printf("ERROR oglPopMatrix pop level underflow ( < 0 )\n"); exit(1); } \
    ogl_compat_matrix_level[0]--; ogl_compat_matrix_level[1]--; }

#define oglMultMatrixf(A) {\
    float (*B)[4] = ogl_compat_matrix[ ogl_compat_matrix_level[ogl_compat_matrix_mode] ][ ogl_compat_matrix_mode ]; \
    float C[4][4]; \
    MultMatrix4x4_internal(B,A,C); \
    memcpy(B, C, 16*sizeof(float)); }

#define oglRotatef(deg,ux,uy,uz) { \
    float _rad=(deg)*(PI/180.0); \
    float _inv_len=sqrt((ux)*(ux) + (uy)*(uy) + (uz)*(uz)); \
    float _ux=(ux)*_inv_len, _uy=(uy)*_inv_len, _uz=(uz)*_inv_len; \
    float _c=cos(_rad), _s=sin(_rad); \
    float m[4][4]; \
    m[0][0] = _c +_ux*_ux*(1.0f-_c);      m[0][1] = _ux*_uy*(1.0f-_c) - _uz*_s; m[0][2] = _ux*_uz*(1.0f-_c) + _uy*_s; m[0][3] = 0.0f; \
    m[1][0] = _uy*_ux*(1.0f-_c) + _uz*_s; m[1][1] = _c + _uy*_uy*(1.0f-_c);     m[1][2] = _uy*_uz*(1.0f-_c) - _ux*_s; m[1][3] = 0.0f; \
    m[2][0] = _uz*_ux*(1.0f-_c) - _uy*_s; m[2][1] = _uz*_uy*(1.0f-_c) + _ux*_s; m[2][2] = _c + _uz*_uz*(1.0f-_c);     m[2][3] = 0.0f; \
    m[3][0] = 0.0f;                       m[3][1] = 0.0f;                       m[3][2] = 0.0f;                       m[3][3] = 1.0f; \
    oglMultMatrixf(m); }

#define oglTranslatef(_x,_y,_z) { \
    float (*m)[4] = ogl_compat_matrix[ ogl_compat_matrix_level[ogl_compat_matrix_mode] ][ ogl_compat_matrix_mode ]; \
    m[3][0]+=_x; m[3][1]+=_y; m[3][2]+=_z; }

#define oglScalef(_x,_y,_z) { \
    float (*m)[4] = ogl_compat_matrix[ ogl_compat_matrix_level[ogl_compat_matrix_mode] ][ ogl_compat_matrix_mode ];\
    m[0][3]+=_x; m[1][3]+=_y; m[2][3]+=_z; }

#define ogluPerspective(_fovy,_aspect,_zNear,_zFar) { \
    float _rad=0.5f*(_fovy)*PI/180.0; \
    float _f=cos(_rad) / sin(_rad); \
    float _form1 = ((float)(_zFar) + (float)(_zNear)) / ((float)(_zNear) - (float)(_zFar)); \
    float _form2 = (2.0f * (float)(_zFar) * (float)(_zNear)) / ((float)(_zNear) - (float)(_zFar)); \
    float m[4][4]; \
    m[0][0] = (_f) / (_aspect); m[0][1] = 0.0f; m[0][2] = 0.0f;     m[0][3] =  0.0f; \
    m[1][0] = 0.0f;             m[1][1] = (_f); m[1][2] = 0.0f;     m[1][3] =  0.0f; \
    m[2][0] = 0.0f;             m[2][1] = 0.0f; m[2][2] = (_form1); m[2][3] = -1.0f; \
    m[3][0] = 0.0f;             m[3][1] = 0.0f; m[3][2] = (_form2); m[3][3] =  0.0f; \
    oglMultMatrixf(m); }

#define oglOrtho(l,r,b,t,n,f) { \
    float _l=(float)l, _r=(float)r, _b=(float)b, _t=(float)t, _n=(float)n, _f=(float)f;\
    float _trx=-(_r+_l)/(_r-_l), _try=-(_t+_b)/(_t-_b), _trz=-(_f+_n)/(_f-_n); \
    float m[4][4]; \
    m[0][0] = 2.0 / (_r-_l); m[0][1] = 0.0f;            m[0][2] = 0.0f;              m[0][3] = 0.0f; \
    m[1][0] = 0.0f;          m[1][1] = 2.0 / (_t-_b);   m[1][2] = 0.0f;              m[1][3] = 0.0f; \
    m[2][0] = 0.0f;          m[2][1] = 0.0f;            m[2][2] = -2.0f / (_f - _n); m[2][3] = 0.0f; \
    m[3][0] = _trx;          m[3][1] = _try;            m[3][2] = _trz;              m[3][3] = 1.0f; \
    oglMultMatrixf(m); }

int CompileGLSLShader(const char *vert, const char *frag)
{
    GLint rt;   // return codes
    GLsizei logLen;
    char log[4096];  // for error messages

    // Create Shader And Program Objects
    int my_program         = glCreateProgram();
    int my_vertex_shader   = glCreateShader(GL_VERTEX_SHADER);
    int my_fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
 
    // Load Shader Sources
    glShaderSource(my_vertex_shader, 1, &vert, NULL);
    glShaderSource(my_fragment_shader, 1, &frag, NULL);
 
    // Compile The Shaders
    glCompileShader(my_vertex_shader);
    glGetShaderiv(my_vertex_shader, GL_COMPILE_STATUS, &rt);
    if (!rt) {
        glGetShaderInfoLog(my_vertex_shader, 4096, &logLen, log);
        printf("vert log (code %d): %s\n", rt, log);
        exit(1);
    }

    glCompileShader(my_fragment_shader);
    glGetShaderiv(my_fragment_shader, GL_COMPILE_STATUS, &rt);
    if (!rt) {
        glGetShaderInfoLog(my_fragment_shader, 4096, &logLen, log);
        printf("frag log (code %d): %s\n", rt, log);
        exit(1);
    }
    
    
    // Attach The Shader Objects To The Program Object
    glAttachShader(my_program, my_vertex_shader);
    glAttachShader(my_program, my_fragment_shader);
    glBindFragDataLocation(my_program, 0, "fragColour");

    // Link The Program Object
    glLinkProgram(my_program);
    glGetShaderiv(my_program, GL_LINK_STATUS, &rt);
    if (!rt) {
        glGetShaderInfoLog(my_program, 4096, &logLen, log);
        printf("prog log: (code %d) %s\n", rt, log);
        exit(0);
    }

    // Use The Program Object Instead Of Fixed Function OpenGL
    glUseProgram(my_program);
    return my_program;
}


/* * * * * * * * * * * * * * * * * * * * * * *
 * Minimal Compatibility Layer
 * * * * * * * * * * * * * * * * * * * * * * */
GLuint           ogl_compat_shader_program;
GLuint           ogl_compat_position_attribute;
GLuint           ogl_compat_hasTex_attribute;
GLuint           ogl_compat_colour_attribute;
GLuint           ogl_compat_texcoord_attribute;
GLuint           ogl_compat_texcoord_enabled;
GLuint           ogl_compat_texUnit;
GLuint           ogl_compat_modelview_projection;
GLuint           ogl_compat_vertex_array_object;
GLuint           ogl_compat_vertex_buffer;
int              ogl_compat_vertex_array_mode;
VertexArrayVert  ogl_compat_vertex_array_prev_vert;
VertexArray      ogl_compat_vertex_array;
void oglCompatibilityInit(int maxVerts)
{
    // Initialize the state machine
    ogl_compat_vertex_array.nVert = 0;
    ogl_compat_vertex_array.vert  = (VertexArrayVert*)malloc( maxVerts * sizeof(VertexArrayVert) );
    VertexArrayVert v;
    v.r=1.0f; v.g=1.0f; v.b=1.0f; v.a=1.0f;
    v.x=0.0f; v.y=0.0f; v.z=0.0f;
    v.u=0.0f; v.v=0.0f; v.w=0.0f;
    ogl_compat_vertex_array_prev_vert = v;

    // Create the vertex buffers
    glGenVertexArrays(1, &ogl_compat_vertex_array_object);
    glBindVertexArray(ogl_compat_vertex_array_object);
    glGenBuffers(1, &ogl_compat_vertex_buffer);
    glBindBuffer(GL_ARRAY_BUFFER, ogl_compat_vertex_buffer);

    // Create the fixed function shader
    const char    *vss="#version 150\n\
        uniform mat4 MVP;\n\
        in vec4 position;\n\
        in vec4 colour;\n\
        in vec3 texCoord;\n\
        out vec4 colourV;\n\
        out vec3 texCoordV;\n\
        void main (void) {\n\
            vec4 pos = vec4(position.xyz, 1.0f);\n\
            colourV = colour;\n\
            texCoordV  = texCoord;\n\
            gl_Position = MVP * pos;\n\
        }";
    
    const char    *fss="#version 150\n\
        uniform int hasTex;\n\
        uniform sampler2D texUnit;\n\
        in vec4 colourV;\n\
        in vec3 texCoordV;\n\
        out vec4 fragColour;\n\
        void main(void) { \n\
            vec4 texColor = vec4(1.0f, 1.0f, 1.0f, 1.0f);\n\
            if (hasTex==1) { texColor = texture(texUnit, texCoordV.xy); } \n\
            fragColour = colourV * texColor;\n\
        }";
    
    ogl_compat_shader_program = CompileGLSLShader(vss, fss);

    // 5. Get pointers to uniforms and attributes
    ogl_compat_texcoord_enabled     = glGetUniformLocation(ogl_compat_shader_program, "hasTex");
    ogl_compat_texUnit              = glGetUniformLocation(ogl_compat_shader_program, "texUnit");
    ogl_compat_modelview_projection = glGetUniformLocation(ogl_compat_shader_program, "MVP");
    ogl_compat_colour_attribute     = glGetAttribLocation (ogl_compat_shader_program, "colour");
    ogl_compat_texcoord_attribute   = glGetAttribLocation (ogl_compat_shader_program, "texCoord");
    ogl_compat_position_attribute   = glGetAttribLocation (ogl_compat_shader_program, "position");
    printf("ogl_compat_texcoord_enabled: %d ogl_compat_texUnit %d ogl_compat_colour_attribute: %d ogl_compat_texcoord_attribute: %d ogl_compat_position_attribute: %d\n",
        ogl_compat_texcoord_enabled, ogl_compat_texUnit, ogl_compat_colour_attribute, ogl_compat_texcoord_attribute, ogl_compat_position_attribute);
}


//                           how many?      m/p dims
float ogl_compat_matrix[OGL_MAX_MATRIX_PUSH][2][4][4];   // the matrices
int   ogl_compat_matrix_level[2] = {0, 0};               // how many times have we pushed?
int   ogl_compat_matrix_mode = OGL_MODELVIEW;

















//window context functions.
id app_oglContext;
id app_menubar, app_appMenuItem, app_appMenu, app_appName, app_quitMenuItem, app_quitTitle, app_quitMenuItem, app_window;
id app_oglView;
NSAutoreleasePool *app_pool;
NSDate *app_currDate; 

int app_sw=-999, app_sh=-999;


//------------------------
// ToDo: It may be possible to programmatically use
//  NSOpenGLView directly, and extract the opengl context
//  in "main".  For the time being, this does the trick.
//------------------------
@interface MyOpenGLView : NSOpenGLView
@end
@implementation MyOpenGLView
    - (id)initWithFrame:(NSRect)frame
    {
        // 1. Create a context with opengl pixel format
        NSOpenGLPixelFormatAttribute pixelFormatAttributes[] =
        {
            NSOpenGLPFAOpenGLProfile, NSOpenGLProfileVersion3_2Core,
            NSOpenGLPFAColorSize    , 24                           ,
            NSOpenGLPFAAlphaSize    , 8                            ,
            NSOpenGLPFADepthSize    , 16                           ,
            NSOpenGLPFADoubleBuffer ,
            NSOpenGLPFAAccelerated  ,
            NSOpenGLPFANoRecovery   ,
            0
        };
        NSOpenGLPixelFormat *pixelFormat = [[NSOpenGLPixelFormat alloc] initWithAttributes:pixelFormatAttributes];
        self = [super initWithFrame:frame pixelFormat:pixelFormat];

        // 2. Make the context current
        [[self openGLContext] makeCurrentContext];
        app_oglContext = [self openGLContext];
        return self;
    }
@end

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

void CNFGSetup( const char * WindowName, int sw, int sh )
{
	w = sw;
	h = sh;
    app_sw=sw;
    app_sh=sh;
    printf("CNFGSetup\n");
        
    //----------------------------------
    // Create a programmatic Cocoa OpenGL window
    // This code is slightly modified from
    // CocoaWithLove's Minimalist Cocoa tutorial
    //----------------------------------
    [NSAutoreleasePool new];
    [NSApplication sharedApplication];
    [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];
    app_menubar = [[NSMenu new] autorelease];
    app_appMenuItem = [[NSMenuItem new] autorelease];
    [app_menubar addItem:app_appMenuItem];
    [NSApp setMainMenu:app_menubar];
    app_appMenu = [[NSMenu new] autorelease];
    app_appName = [[NSProcessInfo processInfo] processName];
    app_quitTitle = [@"Quit " stringByAppendingString:app_appName];
    app_quitMenuItem = [[[NSMenuItem alloc] initWithTitle:app_quitTitle
        action:@selector(terminate:) keyEquivalent:@"q"] autorelease];
    [app_appMenu addItem:app_quitMenuItem];
    [app_appMenuItem setSubmenu:app_appMenu];
    app_window = [[[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, sw, sh)
        styleMask:NSTitledWindowMask backing:NSBackingStoreBuffered defer:NO]
            autorelease];

    app_oglView = [[[MyOpenGLView alloc] initWithFrame:NSMakeRect(0, 0, sw, sh) ] autorelease];
    [app_window setContentView:app_oglView];
    [app_window cascadeTopLeftFromPoint:NSMakePoint(20,20)];
    [app_window setTitle:app_appName];
    [app_window makeKeyAndOrderFront:nil];
    [NSApp activateIgnoringOtherApps:YES];
    app_pool = [[NSAutoreleasePool alloc] init];
    app_currDate = [[NSDate alloc] init];
    
    oglCompatibilityInit(sw*sh);       // Initialize the OpenGL compatibility layer

        //--------------------
        // Clear the screen to black
        //--------------------

        [app_pool release];
        //[currDate release];
        app_pool = [[NSAutoreleasePool alloc] init];

        // Peek at the next event
        app_currDate = [[NSDate alloc] init];

        NSEvent *event =
            [NSApp
                nextEventMatchingMask:NSAnyEventMask
                untilDate:app_currDate
                inMode:NSEventTrackingRunLoopMode
                dequeue:YES];
        [app_currDate release];
		
        [NSApp updateWindows];

        glClearColor(0.0,0.0,0.0,0.0);
        glClear(GL_COLOR_BUFFER_BIT);

        [app_oglContext flushBuffer];

    // Set up a 2D projection
    //oglMatrixMode(OGL_PROJECTION);						// Select The Projection Matrix
    //oglLoadIdentity();									// Reset The Projection Matrix
    //oglOrtho(0.0, WIDTH, 0.0, HEIGHT, -10.0, 10.0);
    //oglMatrixMode(OGL_MODELVIEW);							// Select The Modelview Matrix
    //oglLoadIdentity();									// Reset The Modelview Matrix
    //glDisable(GL_DEPTH_TEST);
}

void CNFGHandleInput()
{
}


void CNFGUpdateScreenWithBitmap( unsigned long * data, int w, int h )
{
    unsigned char *rgba=data;

//        printf("data %p w %d h %d sw %d sh %d\n", data, w, h, app_sw, app_sh);
//    exit(1);

    [app_pool release];
    //[currDate release];
    app_pool = [[NSAutoreleasePool alloc] init];

    // Peek at the next event
    app_currDate = [[NSDate alloc] init];
    NSEvent *event =
        [NSApp
            nextEventMatchingMask:NSAnyEventMask
            untilDate:app_currDate
            inMode:NSEventTrackingRunLoopMode
            dequeue:YES];
    [app_currDate release];
/*
        // If we have an event, handle it!
        if (event)
        {
            NSEventType type = [event type];
            switch (type)
            {
                case EVENT_KEY_DOWN:
                    for (i=0; i<[event.characters length]; i++) {
                        unichar ch = [event.characters characterAtIndex: i];
                        Keyboard(keycode(ch), 1);
                    }
                    break;
                    
                case EVENT_KEY_UP:
                    for (i=0; i<[event.characters length]; i++) {
                        unichar ch = [event.characters characterAtIndex: i];
                        Keyboard(keycode(ch), 0);
                    }
                    break;
                    
                case EVENT_LEFT_MOUSE_DOWN:
                    Keyboard(KEY_LEFT_MOUSE, 1);
                    break;
                    
                case EVENT_LEFT_MOUSE_UP:
                    Keyboard(KEY_LEFT_MOUSE, 0);
                    break;

                default:
                    break;
            }
            //printf("type %d\n", (int)type);
        }

        //----------------------
        // Check for mouse motion (NOTE: the mouse move event
        //  has complex behavior after a mouse click.
        //  we can work around this by checking mouse motion explicitly)
        //----------------------
        NSPoint location = [app_window mouseLocationOutsideOfEventStream];
        if ((int)location.x != mouseX || (int)location.y != mouseY) {
            mouseX = (int)location.x;
            mouseY = (int)location.y;
            if (mouseX >= 0 && mouseX < WIDTH &&
                mouseY >= 0 && mouseY < HEIGHT)
            {
                MouseMove(mouseX, mouseY);
            }
        }
*/

    [NSApp updateWindows];

    //--------------------
    // Draw the scene
    //--------------------
    glClearColor(1.0,0.0,0.0,0.0);
    glClear(GL_COLOR_BUFFER_BIT);
    oglMatrixMode(OGL_PROJECTION);						// Select The Projection Matrix
    oglLoadIdentity();									// Reset The Projection Matrix
    oglOrtho(0.0, app_sw, app_sh-1, -1.0, -10.0, 10.0);
    oglMatrixMode(OGL_MODELVIEW);						// Select The Modelview Matrix
    oglLoadIdentity();									// Reset The Modelview Matrix
        
    int i=0,x,y;
    oglBegin(GL_POINTS);
    const float scale = 1.0 / 255;
    for (y=0; y<app_sh; y++) {
        for (x=0; x<app_sw; x++) {
            //oglColor3f(scale*(data[i]>>24), scale*((data[i]>>16)&0xff), scale*((data[i]>>8)&0xff));
            oglColor3f(scale*rgba[4*i],scale*rgba[4*i+1],scale*rgba[4*i+2]);
            oglVertex2f(x,y);
            i++;
        }
    }
    oglEnd();
//        glRasterPos2i(0,0);
//        glDrawPixels(w,h,GL_RGBA,GL_UNSIGNED_BYTE,(const GLvoid*) data);

    [app_oglContext flushBuffer];
}

#ifndef RASTERIZER


uint32_t CNFGColor( uint32_t RGB )
{
}

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

#endif

