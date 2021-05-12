all : jsmntest lintest test_dcl

jsmntest : jsmntest.c jsmn.c
	gcc -g -O0 -o $@ $^

lintest : lintest.c linmath.c linmath.h sv_matrix.blas.c
	gcc -DUSE_DOUBLE -DFLT=double -g -O0 -o $@ $^ -lcblas -lm -llapacke

minimal_opencvtest : sv_matrixtest.c sv_matrix.blas.c sv_matrix.h
	gcc -g -O0 -o $@ $^  -lcblas -lm -llapacke

test_dcl : test_dcl.c dclhelpers.c sv_matrix.blas.c ../src/epnp/epnp.c
	gcc -o $@ $^  os_generic.c -DFLT=double -lpthread -lcblas -lm -llapacke -O3 -msse2 -ftree-vectorize 

.run_tests: clean all
	./lintest
	./minimal_opencvtest
	./test_dcl
	./jsmntest

clean :
	rm -rf *.o *~ jsmntest lintest

