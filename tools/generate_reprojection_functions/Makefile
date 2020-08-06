-include ../../.options
all : check_generated

SRT:=../..

LIBSURVIVE:=$(SRT)/lib/libsurvive.a

CFLAGS:=-I$(SRT)/src/generated -I$(SRT)/redist -I$(SRT)/include/libsurvive -I$(SRT)/include -O3 -fwhole-program -g -ffast-math -pthread #  -fsanitize=address -fsanitize=undefined

ifdef USE_ASAN
	CFLAGS+=-fsanitize=address -fsanitize=undefined -DSURVIVE_ASAN_CHECKS=1
endif

all: ../../src/generated/survive_imu.generated.h check_generated

check_generated: check_generated.c survive_reproject.generated.h survive_reproject.aux.generated.h $(LIBSURVIVE)
	cd ../..;make
	gcc $(CFLAGS) -o $@ $^ $(LDFLAGS) -lm -lc -lgcc  -llapacke  -lcblas -lm  -lz -lusb-1.0 -ldl

clean :
	rm -rf check_generated

../../src/generated/survive_imu.generated.h: imu_functions.py codegen.py  common_math.py
	python imu_functions.py > ../../src/generated/survive_imu.generated.h

../../src/generated/survive_reproject.generated.h: reprojection_functions.py codegen.py  common_math.py  gen1.py  gen2.py
	python reprojection_functions.py > ../../src/generated/survive_reproject.generated.h

survive_reproject.generated.h: ../../src/generated/survive_reproject.generated.h
	cp ../../src/generated/survive_reproject.generated.h survive_reproject.generated.h

survive_reproject.aux.generated.h: reprojection_functions.py
	python reprojection_functions.py --aux > survive_reproject.aux.generated.h

survive_imu.generated.h: ../../src/generated/survive_imu.generated.h
	cp ../../src/generated/survive_imu.generated.h  .