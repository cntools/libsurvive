CFLAGS:=-g -O4 -DUSE_DOUBLE -I../../redist -flto
LDFLAGS:=$(CFLAGS) -lm

all:
	gcc -O3 -o lighthousefind-tori main.c torus_localizer.c visualization.c ../../redist/linmath.c $(LDFLAGS)
clean:
	rm -f lighthousefind-tori
