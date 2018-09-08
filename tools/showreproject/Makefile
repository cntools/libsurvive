all : showreproject

SRT:=../..

LIBSURVIVE:=$(SRT)/lib/libsurvive.so

CFLAGS:=$(CFLAGS) -I$(SRT)/redist -I$(SRT)/include -O0 -g #-fsanitize=address -fsanitize=undefined
LDFLAGS:=-lm -lpthread -llapacke  -lcblas -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs

showreproject : showreproject.cc $(LIBSURVIVE)
	cd ../..;make
	g++ $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean :
	rm -rf showreproject

