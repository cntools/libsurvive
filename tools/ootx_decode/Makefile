all: ootx_decode hmd_datagen

hmd_datagen: HMD_Datagen.c 
	gcc -Wall HMD_Datagen.c -lz -o hmd_datagen

ootx_decode: ootx_decode.c ../../src/ootx_decoder.c ../../src/ootx_decoder.h
	gcc -Wall ootx_decode.c ../../src/ootx_decoder.c -lz -o ootx_decode -I ../../src/
