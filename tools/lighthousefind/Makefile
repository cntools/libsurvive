all : camfind

CFLAGS:=-g -O4 -DFLT=double -I../../redist -flto
LDFLAGS:=$(CFLAGS) -lm 

camfind : camfind.o ../../redist/linmath.c
	gcc -o $@ $^  $(LDFLAGS)

clean :
	rm -rf *.o *~ camfind
