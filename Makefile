all : lib data_recorder test calibrate calibrate_client simple_pose_test

CC?=gcc

CFLAGS:=-Iinclude/libsurvive -fPIC -g -O3 -Iredist -flto -DUSE_DOUBLE -std=gnu99 -rdynamic -llapacke  -lcblas -lm #-fsanitize=address -fsanitize=undefined -Wall -Wno-unused-variable -Wno-switch -Wno-unused-but-set-variable -Wno-pointer-sign -Wno-parentheses
CFLAGS_RELEASE:=-Iinclude/libsurvive -fPIC -msse2 -ftree-vectorize -O3 -Iredist -flto -DUSE_DOUBLE -std=gnu99 -rdynamic -llapacke  -lcblas -lm


#LDFLAGS:=-L/usr/local/lib -lpthread -lusb-1.0 -lz -lm -flto -g
LDFLAGS:=-L/usr/local/lib -lpthread -lz -lm -flto -g

#----------
# Platform specific changes to CFLAGS/LDFLAGS
#----------
UNAME=$(shell uname)

# Mac OSX
ifeq ($(UNAME), Darwin)

CFLAGS:=$(CFLAGS) -DRASTERIZER -DHIDAPI -I/usr/local/include -x objective-c
LDFLAGS:=$(LDFLAGS) -framework OpenGL -framework Cocoa -framework IOKit
#DRAWFUNCTIONS=redist/CNFGFunctions.c redist/CocoaDriver.m
#GRAPHICS_LOFI:=redist/CNFGFunctions.o redist/CocoaDriver.o
DRAWFUNCTIONS=redist/CNFGFunctions.c redist/CNFGCocoaNSImageDriver.m
GRAPHICS_LOFI:=redist/CNFGFunctions.o redist/CNFGCocoaNSImageDriver.o

# Linux / FreeBSD
else

LDFLAGS:=$(LDFLAGS) -lX11 -lusb-1.0
DRAWFUNCTIONS=redist/CNFGFunctions.c redist/CNFGXDriver.c redist/CNFG3D.c
GRAPHICS_LOFI:=redist/CNFGFunctions.o redist/CNFGXDriver.o

endif

SBA:=redist/sba/sba_chkjac.o  redist/sba/sba_crsm.o  redist/sba/sba_lapack.o  redist/sba/sba_levmar.o  redist/sba/sba_levmar_wrap.o
POSERS:=src/poser_dummy.o src/poser_daveortho.o src/poser_charlesslow.o src/poser_octavioradii.o src/poser_turveytori.o src/poser_epnp.o src/poser_sba.o
REDISTS:=redist/json_helpers.o redist/linmath.o redist/jsmn.o redist/minimal_opencv.o
ifeq ($(UNAME), Darwin)
REDISTS:=$(REDISTS) redist/hid-osx.c
endif
LIBSURVIVE_CORE:=src/survive.o src/survive_usb.o src/survive_charlesbiguator.o src/survive_process.o src/ootx_decoder.o src/survive_driverman.o src/survive_default_devices.o src/survive_vive.o src/survive_playback.o src/survive_config.o src/survive_cal.o src/survive_reproject.o src/poser.o src/epnp/epnp.o src/survive_sensor_activations.o src/survive_turveybiguator.o src/survive_disambiguator.o src/survive_statebased_disambiguator.o

#If you want to use HIDAPI on Linux.
#CFLAGS:=$(CFLAGS) -DHIDAPI
#REDISTS:=$(REDISTS) redist/hid-linux.o
#LDFLAGS:=$(LDFLAGS) -ludev

#Useful Preprocessor Directives:
# -DUSE_DOUBLE = use double instead of float for most operations.
# -DNOZLIB = use puff.c
# -DTCC = various things needed for TCC.
# -DWINDOWS -DWIN32 = Building for Windows
# -DHIDAPI = Build vive driver to use USBHID instead of interrupt/control messages.
# -DRUNTIME_SYMNUM = Don't assume __attribute__((constructor)) works.  Instead comb for anything starting with REGISTER.




LIBSURVIVE_CORE:=$(LIBSURVIVE_CORE)
LIBSURVIVE_O:=$(POSERS) $(REDISTS) $(LIBSURVIVE_CORE) $(SBA)
LIBSURVIVE_C:=$(LIBSURVIVE_O:.o=.c)

# unused: redist/crc32.c

testCocoa : testCocoa.c
	$(CC) -o $@ $^ $(LDFLAGS) $(CFLAGS)

test : test.c ./lib/libsurvive.so 
	$(CC) -o $@ $^ $(LDFLAGS) $(CFLAGS)

simple_pose_test : simple_pose_test.c ./lib/libsurvive.so $(DRAWFUNCTIONS)
	$(CC) -o $@ $^ $(LDFLAGS) $(CFLAGS)

data_recorder : data_recorder.c ./lib/libsurvive.so
	$(CC) -o $@ $^ $(LDFLAGS) $(CFLAGS)

calibrate :  calibrate.c ./lib/libsurvive.so $(DRAWFUNCTIONS)
	$(CC) -o $@ $^ $(LDFLAGS) $(CFLAGS)

calibrate_client :  calibrate_client.c ./lib/libsurvive.so $(GRAPHICS_LOFI)
	$(CC) -o $@ $^ $(LDFLAGS) $(CFLAGS)

## Still not working!!! Don't use.
static_calibrate : calibrate.c $(DRAWFUNCTIONS) $(LIBSURVIVE_C)
	tcc -o $@ $^ $(CFLAGS) $(LDFLAGS) -DTCC

./redist/dclhelpers_debuggable.c : ./redist/dclhelpers.c ./redist/dclhelpers.h ./redist/dclapack.h
	gcc -E ./redist/dclhelpers.c  > ./redist/dclhelpers_debuggable.c
	clang-format -i ./redist/dclhelpers_debuggable.c
	sed -i 's/#/\/\/#/g' ./redist/dclhelpers_debuggable.c


test_dcl: ./redist/test_dcl.c ./redist/dclhelpers.c ./redist/dclhelpers.h ./redist/dclapack.h ./redist/minimal_opencv.c ./src/epnp/epnp.c
	$(CC) -o $@ $^ $(LDFLAGS) $(CFLAGS_RELEASE) -DFLT=double

test_dcl_debug: ./redist/test_dcl.c ./redist/dclhelpers_debuggable.c ./redist/dclhelpers.h ./redist/dclapack.h redist/os_generic.c
	$(CC) -o $@ $^ $(LDFLAGS) $(CFLAGS) -DFLT=double -fsanitize=address -fsanitize=undefined

test_minimal_cv: ./src/epnp/test_minimal_cv.c ./lib/libsurvive.so 
	$(CC) -o $@ $^ $(LDFLAGS) $(CFLAGS)

test_epnp: ./src/epnp/test_epnp.c ./lib/libsurvive.so 
	$(CC) -o $@ $^ $(LDFLAGS) $(CFLAGS)

test_epnp_ocv: ./src/epnp/test_epnp.c ./src/epnp/epnp.c
	$(CC) -o $@ $^ -DWITH_OPENCV -lpthread -lz -lm -flto -g -lX11 -lusb-1.0 -Iinclude/libsurvive -fPIC -g -O4 -Iredist -flto -DUSE_DOUBLE -std=gnu99 -rdynamic -fsanitize=address -fsanitize=undefined   -llapack -lm -lopencv_core

lib:
	mkdir lib

lib/libsurvive.so : $(LIBSURVIVE_O)
	$(CC) -o $@ $^ $(LDFLAGS) -shared


calibrate_tcc : $(LIBSURVIVE_C)
	tcc -DRUNTIME_SYMNUM $(CFLAGS) -o $@ $^ $(LDFLAGS) calibrate.c $(DRAWFUNCTIONS) redist/symbol_enumerator.c

clean :
	rm -rf */*/*.o *.o src/*.o *~ src/*~ test simple_pose_test data_recorder calibrate testCocoa lib/libsurvive.so test_minimal_cv test_epnp test_epnp_ocv calibrate_client redist/*.o redist/*~ tools/data_server/data_server tools/lighthousefind/lighthousefind tools/lighthousefind_tori/lighthousefind-tori tools/plot_lighthouse/plot_lighthouse tools/process_rawcap/process_to_points redist/jsmntest redist/lintest

.test_redist:
	cd redist && make .run_tests;

.run_tests: .test_redist

