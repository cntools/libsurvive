all : lib data_recorder test calibrate calibrate_client

CC:=gcc

CFLAGS:=-Iinclude/libsurvive -I. -fPIC -g -O0 -Iredist -flto -DUSE_DOUBLE -std=gnu99 -rdynamic
LDFLAGS:=-lpthread -lusb-1.0 -lz -lX11 -lm -flto -g

POSERS:=src/poser_dummy.o src/poser_daveortho.o src/poser_charlesslow.o
REDISTS:=redist/json_helpers.o redist/linmath.o redist/jsmn.o
LIBSURVIVE_CORE:=src/survive.o src/survive_usb.o src/survive_data.o src/survive_process.o src/ootx_decoder.o src/survive_driverman.o src/survive_vive.o src/survive_config.o src/survive_cal.o
LIBSURVIVE_O:=$(POSERS) $(REDISTS) $(LIBSURVIVE_CORE)
LIBSURVIVE_C:=$(LIBSURVIVE_O:.o=.c)

GRAPHICS_LOFI:=redist/DrawFunctions.o redist/XDriver.o

# unused: redist/crc32.c

test : test.c ./lib/libsurvive.so redist/os_generic.o
	$(CC) -o $@ $^ $(LDFLAGS) $(CFLAGS)

data_recorder : data_recorder.c ./lib/libsurvive.so redist/os_generic.c $(GRAPHICS_LOFI)
	$(CC) -o $@ $^ $(LDFLAGS) $(CFLAGS)

calibrate :  calibrate.c ./lib/libsurvive.so redist/os_generic.c $(GRAPHICS_LOFI)
	$(CC) -o $@ $^ $(LDFLAGS) $(CFLAGS)

calibrate_client :  calibrate_client.c ./lib/libsurvive.so redist/os_generic.c $(GRAPHICS_LOFI)
	$(CC) -o $@ $^ $(LDFLAGS) $(CFLAGS)

## Still not working!!! Don't use.
static_calibrate : calibrate.c redist/os_generic.c redist/XDriver.c redist/DrawFunctions.c $(LIBSURVIVE_C)
	tcc -o $@ $^ $(CFLAGS) $(LDFLAGS) -DTCC

lib:
	mkdir lib

lib/libsurvive.so : $(LIBSURVIVE_O)
	$(CC) -o $@ $^ $(LDFLAGS) -shared


calibrate_tcc : $(LIBSURVIVE_C)
	tcc -DRUNTIME_SYMNUM $(CFLAGS) -o $@ $^ $(LDFLAGS) calibrate.c redist/XDriver.c redist/os_generic.c redist/DrawFunctions.c redist/symbol_enumerator.c

clean :
	rm -rf *.o src/*.o *~ src/*~ test data_recorder lib/libsurvive.so redist/*.o redist/*~



