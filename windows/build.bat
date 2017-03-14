@echo off
set SR=..\src\
set RD=..\redist\
set SOURCES=%SR%ootx_decoder.c %SR%poser_charlesslow.c %SR%poser_daveortho.c %SR%poser_dummy.c %SR%survive.c %SR%survive_cal.c %SR%survive_config.c %SR%survive_data.c %SR%survive_driverman.c %SR%survive_process.c %SR%survive_usb.c
set REDIST=%RD%crc32.c %RD%linmath.c %RD%puff.c %RD%jsmn.c %rd%json_helpers.c
set TCC=C:\tcc\tcc.exe
set CFLAGS=-DNOZLIB -DTCC -DWINDOWS -DHIDAPI -O0 -g -rdynamic -I..\redist -I..\include\libsurvive -I..\src
set LDFLAGS=
@echo on
%TCC% -v %CFLAGS% %SOURCES% %REDIST% %LDFLAGS%

rem %SR%survive_vive.c