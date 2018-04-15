all : data_recorder test calibrate calibrate_client simple_pose_test
	@echo "Built with defaults.  Type 'make help' for more info."

.PHONY : help clean buildfolders

LIBRARY:=./lib/libsurvive.so
OBJDIR:=build
CC?=gcc

CFLAGS+=-Iinclude/libsurvive -fPIC -g -O3 -Iredist -DUSE_DOUBLE -std=gnu99 -rdynamic -MD

ifdef EXTRA_WARNINGS
	CFLAGS+=-fsanitize=address -fsanitize=undefined -Wall -Wno-unused-variable -Wno-switch -Wno-unused-but-set-variable -Wno-pointer-sign -Wno-parentheses
endif

LDFLAGS+=-L/usr/local/lib -lpthread -lz -lm -g -llapacke  -lcblas -lm 
LDFLAGS_TOOLS+=-Llib -lsurvive -Wl,-rpath,lib -lX11 $(LDFLAGS)


SBA:=redist/sba/sba_chkjac.c  redist/sba/sba_crsm.c  redist/sba/sba_lapack.c  redist/sba/sba_levmar.c  redist/sba/sba_levmar_wrap.c redist/minimal_opencv.c src/poser_epnp.c src/poser_sba.c src/epnp/epnp.c 
LIBSURVIVE_CORE:=src/survive.c src/survive_process.c src/ootx_decoder.c src/survive_driverman.c src/survive_default_devices.c src/survive_playback.c src/survive_config.c src/survive_cal.c  src/poser.c src/survive_sensor_activations.c src/survive_disambiguator.c src/survive_imu.c
MINIMAL_NEEDED:=src/survive_usb.c src/survive_charlesbiguator.c  src/survive_vive.c src/survive_reproject.c 
AUX_NEEDED:=src/survive_turveybiguator.c  src/survive_statebased_disambiguator.c
POSERS:=src/poser_dummy.c src/poser_imu.c src/poser_charlesrefine.c
EXTRA_POSERS:=src/poser_daveortho.c src/poser_charlesslow.c src/poser_octavioradii.c src/poser_turveytori.c  
REDISTS:=redist/json_helpers.c redist/linmath.c redist/jsmn.c

ifdef MINIMAL
	LIBSURVIVE_C:=$(REDISTS) $(LIBSURVIVE_CORE) $(MINIMAL_NEEDED)
else
	LIBSURVIVE_C:=$(POSERS) $(REDISTS) $(LIBSURVIVE_CORE) $(SBA) $(MINIMAL_NEEDED) $(AUX_NEEDED)
endif


LIBSURVIVE_O:=$(LIBSURVIVE_C:%.c=$(OBJDIR)/%.o)
LIBSURVIVE_D:=$(LIBSURVIVE_C:%.c=$(OBJDIR)/%.d)
-include $(LIBSURVIVE_D)

#----------
# Platform specific changes to CFLAGS/LDFLAGS
#----------
UNAME=$(shell uname)

# Mac OSX
ifeq ($(UNAME), Darwin)

CFLAGS:=$(CFLAGS) -DRASTERIZER -DHIDAPI -I/usr/local/include -x objective-c
LDFLAGS:=$(LDFLAGS) -framework OpenGL -framework Cocoa -framework IOKit
DRAWFUNCTIONS=redist/CNFGFunctions.c redist/CNFGCocoaNSImageDriver.m
GRAPHICS_LOFI:=redist/CNFGFunctions.o redist/CNFGCocoaNSImageDriver.o

# Linux / FreeBSD
else

LDFLAGS:=$(LDFLAGS) -lX11 -lusb-1.0
DRAWFUNCTIONS=redist/CNFGFunctions.c redist/CNFGXDriver.c redist/CNFG3D.c
GRAPHICS_LOFI:=redist/CNFGFunctions.o redist/CNFGXDriver.o

endif


ifeq ($(UNAME), Darwin)
REDISTS:=$(REDISTS) redist/hid-osx.c
endif


ifdef LINUX_USE_HIDAPI
	CFLAGS:=$(CFLAGS) -DHIDAPI
	REDISTS:=$(REDISTS) redist/hid-linux.o
	LDFLAGS:=$(LDFLAGS) -ludev
endif


help :
	@echo "Usage: make [flags]"
	@echo "  Build-system flags:"
	@echo "    MINIMAL=1          Buld a minimal build, geared for embedded systems."
	@echo "    LINUX_USE_HIDAPI=1 Build with HIDAPI in Linux instead of just libusb."
	@echo "    EXTRA_WARNINGS=1   Provide many mor warnings for build system."
	@echo "    CFLAGS=            Specify additional CFLAGS."
	@echo "    LDFLAGS=           Specify additional LDFLAGS."
	@echo "    CC=                Specify a different C compiler."
	@echo "  Useful Preprocessor Directives (For CFLAGS):"
	@echo "    -DUSE_DOUBLE       Use double instead of float for most operations."
	@echo "    -DNOZLIB           Use puff.c"
	@echo "    -DTCC              Various things needed for TCC."
	@echo "    -DWINDOWS -DWIN32  Building for Windows."
	@echo "    -DRUNTIME_SYMNUM   Don't assume __attribute__((constructor)) works.  Instead comb for anything starting with REGISTER."
	@echo "    -flto              Do link-time optimizations.  This significantly increases period of time to link but improves performance.."
	@echo "  Useful build targets:"
	@echo "    all                Build libsurvive.so and tools."
	@echo "    clean              Erase build and incremental files."
	@echo "    buildfolders       Produce build file structure."
	@echo "    $(LIBRARY)  Produce libsurvive.so"


	
testCocoa : testCocoa.c $(LIBRARY)
	$(CC) -o $@ $^ $(LDFLAGS_TOOLS) $(CFLAGS)

test : test.c $(LIBRARY)
	$(CC) -o $@ $^ $(LDFLAGS_TOOLS) $(CFLAGS)

simple_pose_test : simple_pose_test.c $(DRAWFUNCTIONS) $(LIBRARY)
	$(CC) -o $@ $^ $(LDFLAGS_TOOLS) $(CFLAGS)

data_recorder : data_recorder.c $(LIBRARY)
	$(CC) -o $@ $^ $(LDFLAGS_TOOLS) $(CFLAGS) 

calibrate :  calibrate.c $(DRAWFUNCTIONS) $(LIBRARY)
	$(CC) -o $@ $^ $(LDFLAGS_TOOLS) $(CFLAGS)

calibrate_client :  calibrate_client.c $(GRAPHICS_LOFI) $(LIBRARY)
	$(CC) -o $@ $^ $(LDFLAGS_TOOLS) $(CFLAGS)



## Still not working!!! Don't use.
static_calibrate : calibrate.c $(DRAWFUNCTIONS) $(LIBSURVIVE_C) $(LIBRARY)
	tcc -o $@ $^ $(LDFLAGS_TOOLS) $(LDFLAGS) -DTCC

./redist/dclhelpers_debuggable.c : ./redist/dclhelpers.c ./redist/dclhelpers.h ./redist/dclapack.h
	gcc -E ./redist/dclhelpers.c  > ./redist/dclhelpers_debuggable.c
	clang-format -i ./redist/dclhelpers_debuggable.c
	sed -i 's/#/\/\/#/g' ./redist/dclhelpers_debuggable.c


test_dcl: ./redist/test_dcl.c ./redist/dclhelpers.c ./redist/dclhelpers.h ./redist/dclapack.h ./redist/minimal_opencv.c ./src/epnp/epnp.c
	$(CC) -o $@ $^ $(LDFLAGS_TOOLS) $(CFLAGS) -DFLT=double

test_dcl_debug: ./redist/test_dcl.c ./redist/dclhelpers_debuggable.c ./redist/dclhelpers.h ./redist/dclapack.h redist/os_generic.c
	$(CC) -o $@ $^ $(LDFLAGS_TOOLS) $(CFLAGS) -DFLT=double -fsanitize=address -fsanitize=undefined

test_minimal_cv: ./src/epnp/test_minimal_cv.c $(LIBRARY)
	$(CC) -o $@ $^ $(LDFLAGS_TOOLS) $(CFLAGS)

test_epnp: ./src/epnp/test_epnp.c $(LIBRARY)
	$(CC) -o $@ $^ $(LDFLAGS_TOOLS) $(CFLAGS)

test_epnp_ocv: ./src/epnp/test_epnp.c ./src/epnp/epnp.c
	$(CC) -o $@ $^ -DWITH_OPENCV -lpthread -lz -lm -flto -g -lX11 -lusb-1.0 -Iinclude/libsurvive -fPIC -g -O4 -Iredist -flto -DUSE_DOUBLE -std=gnu99 -rdynamic -fsanitize=address -fsanitize=undefined   -llapack -lm -lopencv_core $(LDFLAGS_TOOLS)

$(OBJDIR):
	mkdir -p lib
	mkdir -p $(OBJDIR)
	mkdir -p $(OBJDIR)/src
	mkdir -p $(OBJDIR)/redist
	mkdir -p $(OBJDIR)/redist/sba
	mkdir -p $(OBJDIR)/src/epnp

$(LIBRARY): $(LIBSURVIVE_O) $(OBJDIR)
	$(CC) -o $@ $(LIBSURVIVE_O) $(CFLAGS) $(LDFLAGS) -shared

$(OBJDIR)/%.o : %.c $(OBJDIR)
	$(CC) -c -o $@ $< $(CFLAGS)

calibrate_tcc : $(LIBSURVIVE_C)
	tcc -DRUNTIME_SYMNUM $(CFLAGS) -o $@ $^ $(LDFLAGS) calibrate.c $(DRAWFUNCTIONS) redist/symbol_enumerator.c

clean :
	rm -rf */*/*.o *.o src/*.o $(OBJDIR) *~ src/*~ test simple_pose_test data_recorder calibrate testCocoa lib/libsurvive.so test_minimal_cv test_epnp test_epnp_ocv calibrate_client redist/*.o redist/*~ tools/data_server/data_server tools/lighthousefind/lighthousefind tools/lighthousefind_tori/lighthousefind-tori tools/plot_lighthouse/plot_lighthouse tools/process_rawcap/process_to_points redist/jsmntest redist/lintest

.test_redist:
	cd redist && make .run_tests;

.run_tests: .test_redist

