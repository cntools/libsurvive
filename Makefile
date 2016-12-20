all : lib data_recorder test

CFLAGS:=-Iinclude -fPIC -g -Os -Iredist
LDFLAGS:=-lpthread -lusb-1.0 -lz -lX11

test : test.c lib/libsurvive.so redist/os_generic.o
	gcc -o $@ $^ $(LDFLAGS) $(CFLAGS)

data_recorder : data_recorder.c lib/libsurvive.so redist/os_generic.o redist/DrawFunctions.o redist/XDriver.o
	gcc -o $@ $^ $(LDFLAGS) $(CFLAGS)

lib:
	mkdir lib

lib/libsurvive.so : src/survive.o src/survive_usb.o src/survive_data.o src/survive_process.o redist/jsmn.o $(DEBUGSTUFF)
	gcc -o $@ $^ $(LDFLAGS) -shared

clean :
	rm -rf *.o src/*.o *~ src/*~ test libsurvive.so



