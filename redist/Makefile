all : jsmntest lintest test_dcl

jsmntest : jsmntest.c jsmn.c
	gcc -g -O0 -o $@ $^

lintest : lintest.c linmath.c linmath.h minimal_opencv.c
	gcc -DUSE_DOUBLE -DFLT=double -g -O0 -o $@ $^ -lcblas -lm -llapacke

minimal_opencvtest : minimal_opencvtest.c minimal_opencv.c minimal_opencv.h
	gcc -g -O0 -o $@ $^  -lcblas -lm -llapacke

test_dcl : test_dcl.c dclhelpers.c minimal_opencv.c ../src/epnp/epnp.c
	gcc -o $@ $^  os_generic.c -DFLT=double -lpthread -lcblas -lm -llapacke -O3 -msse2 -ftree-vectorize 

.run_tests: clean all
	./lintest
	./minimal_opencvtest
	./test_dcl
	./jsmntest

clean :
	rm -rf *.o *~ jsmntest lintest

