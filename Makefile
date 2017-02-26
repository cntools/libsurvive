all : lib data_recorder test calibrate

CFLAGS:=-Iinclude -I. -fPIC -g -O0 -Iredist -flto -DUSE_DOUBLE
LDFLAGS:=-lpthread -lusb-1.0 -lz -lX11 -lm -flto -g


CALS:=src/survive_cal_lhfind.o src/survive_cal.o

# unused: redist/crc32.c

test : test.c lib/libsurvive.so redist/os_generic.o
	gcc -o $@ $^ $(LDFLAGS) $(CFLAGS)

data_recorder : data_recorder.c lib/libsurvive.so redist/os_generic.o redist/DrawFunctions.o redist/XDriver.o
	gcc -o $@ $^ $(LDFLAGS) $(CFLAGS)

calibrate :  calibrate.c lib/libsurvive.so redist/os_generic.c redist/DrawFunctions.c redist/XDriver.c
	gcc -o $@ $^ $(LDFLAGS) $(CFLAGS)

lib:
	mkdir lib

lib/libsurvive.so : src/survive.o src/survive_usb.o src/survive_data.o src/survive_process.o redist/jsmn.o src/ootx_decoder.o redist/linmath.o src/survive_driverman.o src/survive_vive.o $(DEBUGSTUFF) $(CALS)
	gcc -o $@ $^ $(LDFLAGS) -shared

clean :
	rm -rf *.o src/*.o *~ src/*~ test data_recorder lib/libsurvive.so



