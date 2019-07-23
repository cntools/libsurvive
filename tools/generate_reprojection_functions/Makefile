-include ../../.options
all : check_generated

SRT:=../..

LIBSURVIVE:=$(SRT)/lib/libsurvive.a

CFLAGS:=-I$(SRT)/redist -I$(SRT)/include -O3 -fwhole-program -g -ffast-math #  -fsanitize=address -fsanitize=undefined

ifdef USE_ASAN
	CFLAGS+=-fsanitize=address -fsanitize=undefined -DSURVIVE_ASAN_CHECKS=1
endif

check_generated: check_generated.c survive_reproject.generated.h survive_reproject.aux.generated.h $(LIBSURVIVE)
	cd ../..;make
	gcc $(CFLAGS) -o $@ $^ $(LDFLAGS) -lm -lc -lgcc  -llapacke  -lcblas -lm  -lusb-1.0 -ldl

clean :
	rm -rf check_generated

../../src/survive_reproject.generated.h: reprojection_functions.py
	python reprojection_functions.py > ../../src/survive_reproject.generated.h

survive_reproject.generated.h: ../../src/survive_reproject.generated.h
	cp ../../src/survive_reproject.generated.h survive_reproject.generated.h

survive_reproject.aux.generated.h: reprojection_functions.py
	python reprojection_functions.py --aux > survive_reproject.aux.generated.h
