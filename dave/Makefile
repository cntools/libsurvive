UNAME:=$(shell uname)

CFLAGS:= -lm -I../redist -I../src -I../include/libsurvive

ifeq ($(UNAME), Linux)
CFLAGS:= $(CFLAGS) -lGL -lGLU -lglut -lX11 -I../../redist -DLINUX -lm -lpthread -DLINUX
endif

# Darwin is Mac OSX !!
ifeq ($(UNAME), Darwin)
CFLAGS:= $(CFLAGS) -I../../redist -w -framework OpenGL -framework GLUT
endif


all:
#	gcc -O3 -o kalman_filter kalman_filter.c main.c
	gcc -O3 -o dclapack_test dclapack_test.c $(CFLAGS)
	gcc  -O3 -o AffineSolve   AffineSolve.c $(CFLAGS) ../redist/linmath.c #-Wall
	gcc -O3 -o OrthoPlot OrthoPlot.c fileutil.c ../redist/linmath.c ../redist/os_generic.c $(CFLAGS)
clean:
	rm -f kalman_filter dclapack_test
