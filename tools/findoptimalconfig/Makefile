all : findoptimalconfig

SRT:=../..

LIBSURVIVE:=$(SRT)/lib/libsurvive.so

CFLAGS:=-I$(SRT)/redist -I$(SRT)/include -O0 -g #-fsanitize=address -fsanitize=undefined
LDFLAGS:=-lm -lpthread -llapacke  -lcblas 

findoptimalconfig : findoptimalconfig.cc $(LIBSURVIVE)
	cd ../..;make
	g++ $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean :
	rm -rf findoptimalconfig

