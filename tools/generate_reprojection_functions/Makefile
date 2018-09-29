all : check_generated

SRT:=../..

LIBSURVIVE:=$(SRT)/lib/libsurvive.so

CFLAGS:=-I$(SRT)/redist -I$(SRT)/include -O3 -g -fsanitize=address -fsanitize=undefined

check_generated: check_generated.c ../../src/survive_reproject.generated.h survive_reproject.full.generated.h $(LIBSURVIVE)
	cd ../..;USE_ASAN=1 make
	gcc $(CFLAGS) -o $@ $^ $(LDFLAGS) -lm -lc -lgcc 

clean :
	rm -rf check_generated

../../src/survive_reproject.generated.h: reprojection_functions.py
	python reprojection_functions.py > ../../src/survive_reproject.generated.h

survive_reproject.full.generated.h: reprojection_functions.py
	python reprojection_functions.py --full > survive_reproject.full.generated.h
