CFLAGS:=-g -O4 -DFLT=double -I../../redist -flto
LDFLAGS:=$(CFLAGS) -lm

all:
	gcc -O3 -o lighthousefind-tori main.c find_tori_math.c torus_localizer.c visualization.c ../../redist/linmath.c $(LDFLAGS)
clean:
	rm -f lighthousefind-tori
