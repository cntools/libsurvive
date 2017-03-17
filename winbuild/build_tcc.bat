@echo off

set TCC=C:\tcc\tcc.exe
echo Be sure to have TinyCC installed to %TCC% (or change the TCC parameter.)

set SR=..\src\
set RD=..\redist\
set SOURCES=%SR%ootx_decoder.c %SR%poser_charlesslow.c %SR%poser_daveortho.c %SR%poser_dummy.c %SR%survive.c %SR%survive_cal.c %SR%survive_config.c %SR%survive_data.c %SR%survive_driverman.c %SR%survive_process.c %SR%survive_vive.c
set REDIST=%RD%crc32.c %RD%linmath.c %RD%puff.c %RD%jsmn.c %RD%json_helpers.c  %RD%symbol_enumerator.c
set EXEC=..\calibrate.c %RD%CNFGWinDriver.c %RD%os_generic.c %RD%CNFGFunctions.c
set CFLAGS=-DNOZLIB -DTCC -DWINDOWS -DHIDAPI -DWIN32 -DRUNTIME_SYMNUM -O0 -g -rdynamic -I..\redist -I..\include\libsurvive -I..\src -I.
set LDFLAGS=-lkernel32 -lgdi32 -luser32 -lsetupapi -ldbghelp
@echo on
%TCC% -v %CFLAGS% %SOURCES% %REDIST% %EXEC% %LDFLAGS% tcc_stubs.c %RD%hid-windows.c -o calibrate.exe
