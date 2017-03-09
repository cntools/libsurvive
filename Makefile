all : lib data_recorder test calibrate calibrate_client

CFLAGS:=-Iinclude/libsurvive -I. -fPIC -g -O0 -Iredist -flto -DUSE_DOUBLE -std=gnu99
LDFLAGS:=-lpthread -lusb-1.0 -lz -lX11 -lm -flto -g

CALS:=src/survive_cal_lhfind.o src/survive_cal.o
POSERS:=src/poser_dummy.o
REDISTS:=redist/json_helpers.o redist/linmath.o redist/jsmn.o
LIBSURVIVE_CORE:=src/survive.o src/survive_usb.o src/survive_data.o src/survive_process.o src/ootx_decoder.o src/survive_driverman.o src/survive_vive.o src/survive_config.o 
LIBSURVIVE_O:=$(CALS) $(POSERS) $(REDISTS) $(LIBSURVIVE_CORE)

GRAPHICS_LOFI:=redist/DrawFunctions.o redist/XDriver.o

# unused: redist/crc32.c

test : test.c lib/libsurvive.so redist/os_generic.o
	gcc -o $@ $^ $(LDFLAGS) $(CFLAGS)

data_recorder : data_recorder.c lib/libsurvive.so redist/os_generic.c $(GRAPHICS_LOFI)
	gcc -o $@ $^ $(LDFLAGS) $(CFLAGS)

calibrate :  calibrate.c lib/libsurvive.so redist/os_generic.c $(GRAPHICS_LOFI)
	gcc -o $@ $^ $(LDFLAGS) $(CFLAGS)

calibrate_client :  calibrate_client.c lib/libsurvive.so redist/os_generic.c $(GRAPHICS_LOFI)
	gcc -o $@ $^ $(LDFLAGS) $(CFLAGS)

lib:
	mkdir lib

lib/libsurvive.so : $(LIBSURVIVE_O)
	gcc -o $@ $^ $(LDFLAGS) -shared

clean :
	rm -rf *.o src/*.o *~ src/*~ test data_recorder lib/libsurvive.so redist/*.o redist/*~



