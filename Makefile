all : test

CFLAGS:=-Iinclude -fPIC -g -Os
LDFLAGS:=-lpthread -lusb-1.0

test : test.c lib/libsurvive.so
	gcc -o $@ $^ $(LDFLAGS) $(CFLAGS)

lib/libsurvive.so : src/os_generic.o src/survive.o src/survive_usb.o src/survive_data.o
	gcc -o $@ $^ $(LDFLAGS) -shared

clean :
	rm -rf *.o src/*.o *~ src/*~ test libsurvive.so



