_libsurvive.so: ../../bin/libsurvive.so
	cp ../../bin/libsurvive.so pysurvive/libsurvive.so
	cp -r ../../bin/plugins pysurvive/plugins

../../bin/libsurvive.so: .always
	cd ../.. && make clean all 

.always:
