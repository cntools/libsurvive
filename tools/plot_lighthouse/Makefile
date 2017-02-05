UNAME := $(shell uname)

ifeq ($(UNAME), Linux)
CFLAGS:= -lGL -lGLU -lglut
endif

# Darwin is Mac OSX !!
ifeq ($(UNAME), Darwin)
CFLAGS:= -w -framework OpenGL -framework GLUT
endif

all:
	gcc -O3 -o plot_lighthouse main.c glutil.c fileutil.c ../../redist/os_generic.c $(CFLAGS)
clean:
	rm -f plot_lighthouse
