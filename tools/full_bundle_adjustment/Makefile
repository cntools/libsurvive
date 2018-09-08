all : full_bundle_adjustment

SRT:=../..

LIBSURVIVE:=$(SRT)/lib/libsurvive.so

CFLAGS:=$(CFLAGS) -I$(SRT)/redist -I$(SRT)/include -O0 -g #-fsanitize=address -fsanitize=undefined
LDFLAGS:=-lm -lpthread -llapacke  -lcblas

full_bundle_adjustment : full_bundle_adjustment.cc $(LIBSURVIVE)
	cd ../..;make
	g++ $(CFLAGS) -o $@ $^ $(LDFLAGS)

clean :
	rm -rf full_bundle_adjustment

