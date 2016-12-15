all : camfind

CFLAGS:=-g -O4 -DFLT=float -ffast-math
LDFLAGS:=$(CFLAGS) -lm

camfind : camfind.o linmath.o
	gcc -o $@ $^  $(LDFLAGS)

clean :
	rm -rf *.o *~ camfind
