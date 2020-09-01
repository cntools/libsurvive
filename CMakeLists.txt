cmake_minimum_required(VERSION 3.12.4)

project(libsurvive C CXX)
include(CheckIncludeFile)
include (CheckSymbolExists)

SET(CORE_BUILD_DEFAULT OFF)
if(ANDROID)
  SET(CORE_BUILD_DEFAULT ON)
endif()

option(DO_CORE_BUILD "Only build essentials by default" ${CORE_BUILD_DEFAULT})
option(BUILD_STATIC "Build as static library" ${DO_CORE_BUILD})

option(USE_SINGLE_PRECISION "Use float instead of double" OFF)
option(ENABLE_WARNINGS_AS_ERRORS "Use to flag all warnings as errors" OFF)
option(USE_HIDAPI "Use HIDAPI instead of libusb" OFF)
option(USE_ASAN "Use address sanitizer" OFF)
option(ENABLE_TESTS "Enable build / execution of tests" OFF)
option(USE_HEX_FLOAT_PRINTF "Use hex floats when recording" OFF)
option(USE_OPENBLAS "Use OpenBLAS" OFF)

if(USE_HEX_FLOAT_PRINTF)
  add_definitions(-DSURVIVE_HEX_FLOATS)
endif()

SET(SURVIVE_LIBRARY_TYPE SHARED)
if(BUILD_STATIC)
  SET(SURVIVE_LIBRARY_TYPE STATIC)
  add_definitions(-DSURVIVE_DISABLE_PLUGINS)
endif()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads)

check_include_file("gattlib.h" HAVE_GATTLIB_H)
option(BUILD_GATT_SUPPORT "Whether or not to include gatt support for the basestations -- requires gattlib (https://github.com/labapart/gattlib)" ${HAVE_GATTLIB_H})

check_include_file("zlib.h" HAVE_ZLIB_H)
IF(NOT HAVE_ZLIB_H OR ANDROID)
  add_definitions(-DNOZLIB)
ENDIF()

SET(NOT_CORE_BUILD ON)
if(DO_CORE_BUILD)
  SET(NOT_CORE_BUILD OFF)
endif()

include_directories(${CMAKE_INSTALL_PREFIX}/include)
link_directories(${CMAKE_INSTALL_PREFIX}/lib)

IF(UNIX)
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -std=gnu99 -rdynamic -Werror=incompatible-pointer-types -Wall -Wno-unused-variable -Wno-switch -Wno-parentheses -Wno-missing-braces -Werror=implicit-function-declaration -Wpedantic -Werror=return-type -fvisibility=hidden -Werror=vla")
	if(ENABLE_WARNINGS_AS_ERRORS)
	  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror")
	endif()
	      	
	if(USE_ASAN)
	  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fsanitize=undefined")
	endif()	    
ELSEIF(WIN32)
  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /wd4244 /wd4996 /wd4018 /wd4101 /wd4477 /wd4068 /wd4217")
  set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/$<CONFIG>)
endif()

include_directories(redist include/libsurvive include)

if(USE_SINGLE_PRECISION)
    add_definitions(-DUSE_FLOAT)
endif()

IF(ENABLE_TESTS)
  enable_testing()
ENDIF()

 find_library(OPENVR_LIBRARIES
                NAMES
               openvr_api
               openvr_api64
                PATH_SUFFIXES
                osx32
               linux64
               PATHS
               "C:/Program Files (x86)/OpenVRSDK/lib")

find_path(OPENVR_INCLUDE_PATH
               NAMES openvr/openvr.h openvr/openvr_driver.h
               PATHS
               "C:/Program Files (x86)/OpenVRSDK/include"
)
find_library(OPENVR_LIBRARIES
  NAMES
  openvr_api
  PATH_SUFFIXES
  osx32
  linux64)

add_subdirectory(redist)
add_subdirectory(src)
add_subdirectory(tools)

SET(SURVIVE_EXECUTABLES survive-cli api_example sensors-readout survive-solver survive-buttons)
foreach(executable ${SURVIVE_EXECUTABLES})
  option(ENABLE_${executable} "Build ${executable}" ${NOT_CORE_BUILD})

  if(ENABLE_${executable})
    add_executable(${executable} ${executable}.c )
    target_link_libraries(${executable} survive ${${executable}_ADDITIONAL_LIBS} ${CMAKE_THREAD_LIBS_INIT})
    set_target_properties(${executable} PROPERTIES FOLDER "apps")
    foreach(plugin ${SURVIVE_BUILT_PLUGINS})
      add_dependencies(${executable} ${plugin})
    endforeach()
    install(TARGETS ${executable} DESTINATION bin)
  endif()
endforeach()

IF(WIN32)
  # This property needs to be set on a target at the root; doesn't matter which
  set_target_properties( survive-cli PROPERTIES VS_USER_PROPS "$(SolutionDir)\\packages\\OpenBLAS.0.2.14.1\\build\\native\\openblas.targets" )
endif()

SET(SURVIVE_WEBSOCKETD ${CMAKE_CURRENT_SOURCE_DIR}/useful_files/survive-websocketd)
if(WIN32)
	SET(SURVIVE_WEBSOCKETD ${CMAKE_CURRENT_SOURCE_DIR}/useful_files/survive-websocketd.ps1)
endif()

IF(TARGET survive-cli)
  add_custom_command(TARGET survive-cli POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy ${SURVIVE_WEBSOCKETD} $<TARGET_FILE_DIR:survive-cli>
    )
  install(PROGRAMS ${SURVIVE_WEBSOCKETD} DESTINATION bin)
endif()

find_program(DOTNET dotnet)
if(DOTNET)
  execute_process(
    COMMAND ${DOTNET} build -c Release
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/bindings/cs/)

  file(GLOB_RECURSE CS_BINDINGS
    "${CMAKE_CURRENT_SOURCE_DIR}/bindings/cs/libsurvive.net/bin/Release/*.dll")

  install(FILES ${CS_BINDINGS} DESTINATION "lib")
endif()

install(DIRECTORY include/libsurvive DESTINATION include)

file(GLOB REDIST_HEADERS
  "redist/*.h"
)
install(FILES ${REDIST_HEADERS} DESTINATION include/libsurvive/redist)

if(PYTHON_GENERATED_DIR)
  find_package (Python3 COMPONENTS Interpreter Development)
  find_program(CTYPESGEN ctypesgen REQUIRED)

  message("-- Building python bindings file to ${PYTHON_GENERATED_DIR}")
  message("-- ctypesgen found at ${CTYPESGEN}") 
  get_property(include_directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
  set(INCLUDE_FLAGS "")
  foreach(include_directory ${include_directories})
    list(APPEND INCLUDE_FLAGS "-I${include_directory}")
  endforeach()

  add_custom_target(pysurvive ALL COMMAND ${Python3_EXECUTABLE} ${CTYPESGEN} ${CMAKE_SOURCE_DIR}/include/libsurvive/*.h ${INCLUDE_FLAGS} --no-macros -L$<TARGET_FILE_DIR:survive> -llibsurvive.so
    --strip-prefix=survive_ -P Survive -o ${PYTHON_GENERATED_DIR}pysurvive_generated.py )
endif()

include(GNUInstallDirs)
configure_file(survive.pc.in survive.pc @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/survive.pc" DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")
