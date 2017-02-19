all : lib data_recorder test calibrate

CFLAGS:=-Iinclude -fPIC -g -Os -Iredist -flto
LDFLAGS:=-lpthread -lusb-1.0 -lz -lX11 -lm -flto

# unused: redist/crc32.c

test : test.c lib/libsurvive.so redist/os_generic.o
	gcc -o $@ $^ $(LDFLAGS) $(CFLAGS)

data_recorder : data_recorder.c lib/libsurvive.so redist/os_generic.o redist/DrawFunctions.o redist/XDriver.o
	gcc -o $@ $^ $(LDFLAGS) $(CFLAGS)

calibrate :  calibrate.c lib/libsurvive.so redist/os_generic.c redist/DrawFunctions.c redist/XDriver.c
	gcc -o $@ $^ $(LDFLAGS) $(CFLAGS)

lib:
	mkdir lib

lib/libsurvive.so : src/survive.o src/survive_usb.o src/survive_data.o src/survive_process.o redist/jsmn.o src/survive_cal.o src/ootx_decoder.o $(DEBUGSTUFF)
	gcc -o $@ $^ $(LDFLAGS) -shared

clean :
	rm -rf *.o src/*.o *~ src/*~ test data_recorder lib/libsurvive.so



