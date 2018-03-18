all : jsmntest lintest test_dcl

jsmntest : jsmntest.c jsmn.c
	gcc -g -O0 -o $@ $^

lintest : lintest.c linmath.c linmath.h
	gcc -g -O0 -o $@ $^ -lm

test_dcl : test_dcl.c dclhelpers.c minimal_opencv.c ../src/epnp/epnp.c
	gcc -o $@ $^  os_generic.c -DFLT=double -lpthread -lcblas -lm -llapacke -O3 -msse2 -ftree-vectorize 

clean :
	rm -rf *.o *~ jsmntest lintest

