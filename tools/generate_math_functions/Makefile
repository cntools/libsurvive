all: ../../src/generated/survive_imu.generated.h ../../src/generated/survive_reproject.generated.h ../../src/generated/survive_reproject.aux.generated.h

../../src/generated/survive_imu.generated.h: imu_functions.py codegen.py  common_math.py
	python imu_functions.py > ../../src/generated/survive_imu.generated.h

../../src/generated/survive_reproject.generated.h: reprojection_functions.py codegen.py  common_math.py  gen1.py  gen2.py
	python reprojection_functions.py > ../../src/generated/survive_reproject.generated.h

../../src/generated/survive_reproject.aux.generated.h: reprojection_functions.py codegen.py  common_math.py  gen1.py  gen2.py
	python reprojection_functions.py --aux > ../../src/generated/survive_reproject.aux.generated.h
