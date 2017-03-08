all : lighthousefind_radii

CFLAGS:=-g -O4 -DUSE_DOUBLE -I../../redist -flto
LDFLAGS:=$(CFLAGS) -lm 

lighthousefind : lighthousefind.o ../../redist/linmath.c
	gcc -o $@ $^  $(LDFLAGS)

clean :
	rm -rf *.o *~ lighthousefind_radii
