all : test

CFLAGS:=-Iinclude -fPIC -g -Os -Iredist
LDFLAGS:=-lpthread -lusb-1.0 -lz -lX11 -lXinerama
DEBUGSTUFF:=redist/os_generic.o redist/DrawFunctions.o redist/XDriver.o

test : test.c lib/libsurvive.so
	gcc -o $@ $^ $(LDFLAGS) $(CFLAGS)

lib/libsurvive.so : src/survive.o src/survive_usb.o src/survive_data.o src/survive_process.o redist/jsmn.o $(DEBUGSTUFF)
	gcc -o $@ $^ $(LDFLAGS) -shared

clean :
	rm -rf *.o src/*.o *~ src/*~ test libsurvive.so



