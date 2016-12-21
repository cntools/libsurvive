all : lib data_recorder test

CFLAGS:=-Iinclude -fPIC -g -Os -Iredist -flto -DUSE_OLD_DISAMBIGUATOR
LDFLAGS:=-lpthread -lusb-1.0 -lz -lX11 -flto

test : test.c lib/libsurvive.so redist/os_generic.o
	gcc -o $@ $^ $(LDFLAGS) $(CFLAGS)

data_recorder : data_recorder.c lib/libsurvive.so redist/os_generic.o redist/DrawFunctions.o redist/XDriver.o
	gcc -o $@ $^ $(LDFLAGS) $(CFLAGS)

lib:
	mkdir lib

lib/libsurvive.so : src/survive.o src/survive_usb.o src/survive_data.o src/survive_process.o src/disambiguator.c redist/jsmn.o $(DEBUGSTUFF)
	gcc -o $@ $^ $(LDFLAGS) -shared

clean :
	rm -rf *.o src/*.o *~ src/*~ test data_recorder lib/libsurvive.so



