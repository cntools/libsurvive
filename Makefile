all : test

CFLAGS:=-Iinclude -fPIC -g -Os
LDFLAGS:=-lpthread -lusb-1.0 -lz

test : test.c lib/libsurvive.so
	gcc -o $@ $^ $(LDFLAGS) $(CFLAGS)

lib/libsurvive.so : src/survive.o src/survive_usb.o src/survive_data.o src/survive_process.o
	gcc -o $@ $^ $(LDFLAGS) -shared

clean :
	rm -rf *.o src/*.o *~ src/*~ test libsurvive.so



