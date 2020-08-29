
set(CNGFX_SRCS ./CNFG3D.c ./CNFGFunctions.c)
IF(UNIX)
  check_include_file("X11/Xlib.h" HAVE_X11_H)

  IF(HAVE_X11_H) 
    list(APPEND CNGFX_SRCS ./CNFGXDriver.c)
    set(CNGFX_LIBS X11)
  else()
    SET(CNGFX_SRCS)
  endif()
elseif(WIN32)
  list(APPEND CNGFX_SRCS ./CNFGWinDriver.c)
endif()

if(CNGFX_SRCS)
  add_library(CNGFX ${CNGFX_SRCS})
  set_target_properties(CNGFX PROPERTIES FOLDER "libraries")
  target_link_libraries(CNGFX ${CNGFX_LIBS})
endif()

IF(USE_OPENCV)
	SET(MINIMAL_OPENCV_SRCS ./minimal_opencv.h)
ELSE()
	SET(MINIMAL_OPENCV_SRCS ./minimal_opencv.c ./minimal_opencv.h)
ENDIF()

IF(WIN32)
	set(packages_config "<packages><package id=\"lapacke\" version=\"0.1.0\" targetFramework=\"native\" /><package id=\"OpenBLAS\" version=\"0.2.14.1\" targetFramework=\"native\" /></packages>")
	file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/packages.config ${packages_config})
	ADD_DEFINITIONS(-DHAVE_LAPACK_CONFIG_H -DLAPACK_COMPLEX_STRUCTURE)
	set(MINIMAL_OPENCV_SRCS ${MINIMAL_OPENCV_SRCS} ${CMAKE_CURRENT_BINARY_DIR}/packages.config)
endif()

add_library(mpfit STATIC mpfit/mpfit.c)
set_target_properties(mpfit PROPERTIES LINKER_LANGUAGE C)
set_target_properties(mpfit PROPERTIES FOLDER "libraries")

add_library(minimal_opencv STATIC ${MINIMAL_OPENCV_SRCS})
set_target_properties(minimal_opencv PROPERTIES LINKER_LANGUAGE C)
set_target_properties(minimal_opencv PROPERTIES FOLDER "libraries")

if(USE_OPENBLAS OR WIN32)
  set(BLAS_BACKEND "openblas")
else()
  set(BLAS_BACKEND "cblas")  
  find_file(CBLAS_FULL_PATH cblas.h )
  IF(CBLAS_FULL_PATH)
    get_filename_component(CBLAS_PATH ${CBLAS_FULL_PATH} DIRECTORY)
    include_directories( ${CBLAS_PATH})
  ENDIF()
endif()

IF(UNIX)  
  target_link_libraries(minimal_opencv ${BLAS_BACKEND} lapacke m)
elseif(WIN32)
  if (CMAKE_SIZEOF_VOID_P MATCHES 8)
    set(WIN_PLATFORM "x64")
  else(CMAKE_SIZEOF_VOID_P MATCHES 8)
    set(WIN_PLATFORM "win32")
  endif(CMAKE_SIZEOF_VOID_P MATCHES 8)

  include_directories(${CMAKE_BINARY_DIR}/packages/OpenBLAS.0.2.14.1/lib/native/include/)
  target_link_libraries(minimal_opencv
    ${CMAKE_BINARY_DIR}/packages/OpenBLAS.0.2.14.1/lib/native/lib/${WIN_PLATFORM}/libopenblas.dll.a
    )

  find_program(NUGET nuget)
  if(NOT NUGET)
    file(DOWNLOAD https://dist.nuget.org/win-x86-commandline/v5.6.0/nuget.exe "${CMAKE_BINARY_DIR}/nuget.exe"
      EXPECTED_MD5 3f8a53a524c54cf2e4efb7aea370a977 STATUS status LOG log)
    find_program(NUGET nuget HINTS "${CMAKE_BINARY_DIR}" )
  endif()
  message("Nuget is found at ${NUGET}")
  add_custom_command(TARGET minimal_opencv PRE_BUILD COMMAND ${NUGET} restore ${CMAKE_BINARY_DIR}/${PROJECT_NAME}.sln)
  
ENDIF()

IF (ENABLE_TESTS)
	add_executable(lintest linmath.c linmath.h lintest.c)
	target_link_libraries(lintest minimal_opencv)
	set_target_properties(lintest PROPERTIES FOLDER "tests")

	add_executable(minimal_opencvtest minimal_opencvtest.c linmath.c)
	target_link_libraries(minimal_opencvtest minimal_opencv)
	set_target_properties(minimal_opencvtest PROPERTIES FOLDER "tests")

	add_test(NAME lintest COMMAND lintest)
ENDIF()

install(TARGETS minimal_opencv DESTINATION lib)
install(TARGETS mpfit DESTINATION lib)
