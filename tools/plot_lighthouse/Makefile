UNAME := $(shell uname)

ifeq ($(UNAME), Linux)
CFLAGS:= -lGL -lGLU -lglut -I../../redist -DLINUX -lm -lpthread
endif

# Darwin is Mac OSX !!
ifeq ($(UNAME), Darwin)
CFLAGS:= -I../../redist -w -framework OpenGL -framework GLUT
endif

all:
	gcc -O3 -o plot_lighthouse main.c glutil.c fileutil.c ../../redist/os_generic.c ../../redist/linmath.c $(CFLAGS)
clean:
	rm -f plot_lighthouse
