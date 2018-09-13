all : openvr_compare

SRT:=../..

LIBSURVIVE:=$(SRT)/lib/libsurvive.so

CFLAGS:=$(CFLAGS) -I$(SRT)/redist -I$(SRT)/include -O0 -g -I./openvr/headers
LDFLAGS:=-lm -ldl -lopenvr_api -Wl,-rpath=./openvr/bin/linux64 -L./openvr/bin/linux64

# I like nothing about this 'packaging' mechanism, but it seems like the best option
openvr:
	git clone https://github.com/ValveSoftware/openvr.git openvr
	cd openvr && git checkout v1.0.16
	mkdir -p openvr/build
	cd openvr/build && cmake .. && make

openvr_compare : openvr openvr_compare.cc $(LIBSURVIVE) Makefile
	cd ../..;USE_USBMON=1 make
	g++ $(CFLAGS) -o $@ openvr_compare.cc $(LIBSURVIVE) $(LDFLAGS)

clean :
	rm -rf openvr_compare
