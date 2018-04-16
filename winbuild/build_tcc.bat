@echo off
set TCC=C:\tcc\tcc.exe
echo USing %TCC%
set EXEC=calibrate.c redist\CNFGWinDriver.c redist\CNFGFunctions.c
set SOURCES=redist/json_helpers.c redist/linmath.c redist/jsmn.c redist/puff.c redist/crc32.c redist/hid-windows.c winbuild/getdelim.c src/survive.c src/survive_process.c src/ootx_decoder.c src/survive_driverman.c src/survive_default_devices.c src/survive_playback.c src/survive_config.c src/survive_cal.c  src/poser.c src/survive_sensor_activations.c src/survive_disambiguator.c src/survive_imu.c src/survive_usb.c src/survive_charlesbiguator.c  src/survive_vive.c src/survive_reproject.c 
set CFLAGS=-DTCC -Iinclude/libsurvive -g -O3 -Iredist -DUSE_DOUBLE -std=gnu99 -MD -DNOZLIB -DWINDOWS -DWIN32 -DHIDAPI
set LDFLAGS=-lkernel32 -lgdi32 -luser32
@echo on
%TCC% -v %CFLAGS% %SOURCES% %REDIST% %EXEC% %LDFLAGS% winbuild/tcc_stubs.c -o calibrate.exe
