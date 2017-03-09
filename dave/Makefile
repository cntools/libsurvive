all:
#	gcc -O3 -o kalman_filter kalman_filter.c main.c
	gcc -O3 -o dclapack_test dclapack_test.c -lm
	gcc  -O0 -g -o AffineSolve   AffineSolve.c -lm #-Wall

clean:
	rm -f kalman_filter dclapack_test
