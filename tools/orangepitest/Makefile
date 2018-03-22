all : orangepitest

SRT:=../..

RAWDRAWS:=$(SRT)/redist/CNFGFunctions.c $(SRT)/redist/CNFGEGLDriver.c $(SRT)/redist/CNFG3D.c $(SRT)/redist/os_generic.c
LIBSURVIVE:=$(SRT)/lib/libsurvive.so

CFLAGS:=-I$(SRT)/redist -I$(SRT)/include -O3 -DFLT=double -DUSE_DOUBLE
LDFLAGS:=-lm -lpthread -lMali -llapacke  -lcblas 

orangepitest : orangepitest.c $(LIBSURVIVE) $(RAWDRAWS)
	gcc $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean :
	rm -rf orangepitest

