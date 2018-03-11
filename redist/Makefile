all : jsmntest lintest

jsmntest : jsmntest.c jsmn.c
	gcc -g -O0 -o $@ $^

lintest : lintest.c linmath.c linmath.h
	gcc -g -O0 -o $@ $^ -lm

clean :
	rm -rf *.o *~ jsmntest lintest

