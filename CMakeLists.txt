project(libsurvive C CXX)
cmake_minimum_required(VERSION 3.5.1)

option(ENABLE_WARNINGS_AS_ERRORS "Use to flag all warnings as errors" OFF)
include(CheckIncludeFile)

IF(UNIX)
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -std=gnu99 -rdynamic -Werror=incompatible-pointer-types -Wall -Wno-unused-variable -Wno-switch -Wno-parentheses -Wno-missing-braces -Werror=implicit-function-declaration -Wpedantic -Werror=return-type -fvisibility=hidden -Werror=vla")
	if(ENABLE_WARNINGS_AS_ERRORS)
		SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Werror")
	endif()
ENDIF()

IF(WIN32)
	SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} /wd4244 /wd4996 /wd4018 /wd4101 /wd4477 /wd4068 /wd4217")
endif()

include_directories(redist include/libsurvive include)

option(USE_SINGLE_PRECISION "Use float instead of double" OFF)
if(USE_SINGLE_PRECISION)
    add_definitions(-DUSE_FLOAT)
endif()

option(USE_HIDAPI "Use HIDAPI instead of libusb" OFF)
option(USE_ASAN "Use address sanitizer" OFF)
option(ENABLE_TESTS "Enable build / execution of tests" OFF)

IF (ENABLE_TESTS)
	enable_testing()
ENDIF()

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

IF(NOT WIN32)

	check_include_file("lapacke.h" HAVE_LAPACKE_H)

	option(USE_OPENBLAS "Use OpenBLAS -- requires USE_LAPACKE ON" OFF)

	option(USE_LAPACKE "Use LAPACKE" ${HAVE_LAPACKE_H})
	option(USE_OPENBLAS "Use OpenBLAS" OFF)

	if(USE_OPENBLAS)
		set(BLAS_BACKEND "openblas")
	else()
		set(BLAS_BACKEND "cblas")
	endif()

	if(NOT USE_LAPACKE)
		find_package( OpenCV )
		option(USE_OPENCV "Use OpenCV" ${OpenCV_FOUND})
	endif()

	IF(NOT USE_OPENCV AND NOT USE_LAPACKE)
		message(FATAL_ERROR "Libsurvive requires either opencv or lapacke backend")
	ENDIF()

	IF(USE_OPENCV)
	  add_definitions(-DUSE_OPENCV)
	ENDIF()
	
	if(USE_ASAN)
	  SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fsanitize=undefined")
	endif()
	      
ELSE()
	option(USE_LAPACKE "Use LAPACKE" ON)
ENDIF()

if(USE_HIDAPI OR WIN32)
    add_definitions (-DHIDAPI)
	IF(WIN32)
		SET(SURVIVE_SRCS ${SURVIVE_SRCS} ./redist/hid-windows.c ./winbuild/getdelim.c)
	else()
		list(APPEND ADDITIONAL_LIBRARIES udev hidapi-libusb)
	endif()
endif()

check_include_file("zlib.h" HAVE_ZLIB_H)

IF(NOT HAVE_ZLIB_H)
  add_definitions(-DNOZLIB)
ENDIF()

IF(WIN32)
  set( CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/$<CONFIG>)
ENDIF()

list (APPEND CMAKE_REQUIRED_DEFINITIONS -D_GNU_SOURCE)

include (CheckSymbolExists)
check_symbol_exists (fopencookie stdio.h HAVE_FOPENCOOKIE)

if(HAVE_FOPENCOOKIE)
	add_definitions(-DHAVE_FOPENCOOKIE)
endif()

SET(SURVIVE_SRCS ${SURVIVE_SRCS}
  ./include/libsurvive/poser.h
  ./include/libsurvive/survive.h
  ./include/libsurvive/survive_api.h
  ./include/libsurvive/survive_optimizer.h
  ./include/libsurvive/survive_reproject.h
  ./include/libsurvive/survive_types.h
  ./redist/crc32.c
  ./redist/glutil.c
  ./redist/jsmn.c
  ./redist/json_helpers.c
  ./redist/linmath.c
  ./redist/mpfit/mpfit.c
  ./redist/puff.c
  ./redist/symbol_enumerator.c
  ./src/ootx_decoder.c
  ./src/poser.c
  ./src/poser_general_optimizer.c
  ./src/survive.c
  ./src/survive_api.c
  ./src/survive_cal.c
  ./src/survive_config.c
  ./src/survive_default_devices.c
  ./src/survive_disambiguator.c
  ./src/survive_driverman.c
        src/survive_kalman_tracker.c
  ./src/survive_optimizer.c
  ./src/survive_playback.c        
  ./src/survive_plugins.c
  ./src/survive_process.c
  ./src/survive_process_gen2.c
  ./src/survive_reproject.c
		src/generated/survive_reproject.generated.h
  ./src/survive_sensor_activations.c
		./src/survive_kalman.c
  ./src/barycentric_svd/barycentric_svd.c
  ./src/barycentric_svd/barycentric_svd.h
  ./src/survive_reproject_gen2.c
  src/lfsr.c
  src/lfsr.h
  src/lfsr_lh2.h
  src/lfsr_lh2.c
  src/survive_str.h src/survive_str.c src/test_cases/str.c
  src/survive_async_optimizer.c
  src/survive_async_optimizer.h
  )
	      
add_library(survive SHARED ${SURVIVE_SRCS})
set_target_properties(survive PROPERTIES FOLDER "libraries")

add_subdirectory(redist)

target_link_libraries(survive minimal_opencv ${CMAKE_THREAD_LIBS_INIT} Threads::Threads ${CMAKE_DL_LIBS} ${ADDITIONAL_LIBRARIES})
IF(HAVE_ZLIB_H)
  target_link_libraries(survive z)
ENDIF()
IF(NOT WIN32)
	target_link_libraries(survive m )
else()
	target_link_libraries(survive DbgHelp SetupAPI)
endif()

SET(PLUGINS
        driver_dummy driver_vive driver_simulator
        disambiguator_statebased
        poser_dummy poser_mpfit poser_imu
	poser_barycentric_svd
)

set(driver_vive_ADDITIONAL_LIBS usb-1.0)

check_include_file("gattlib.h" HAVE_GATTLIB_H)
option(BUILD_GATT_SUPPORT "Whether or not to include gatt support for the basestations -- requires gattlib (https://github.com/labapart/gattlib)" ${HAVE_GATTLIB_H})
if(BUILD_GATT_SUPPORT)
	LIST(APPEND PLUGINS  driver_gatt)
	set(driver_gatt_ADDITIONAL_LIBS gattlib)
endif()

IF(NOT WIN32)
  find_library(PCAP_LIBRARY pcap)
  if(PCAP_LIBRARY)
    list(APPEND PLUGINS driver_usbmon)
    set(driver_usbmon_ADDITIONAL_LIBS "${PCAP_LIBRARY}" driver_vive)
  else()
    message("Can't build usbmon plugin -- pcap library was not found")
  endif()
endif()

set(poser_barycentric_svd_ADDITIONAL_SRCS src/barycentric_svd/barycentric_svd.c )

add_custom_target(survive_plugins)

find_library(OPENVR_LIBRARIES
		NAMES
		openvr_api
		PATH_SUFFIXES
		osx32
		linux64)

if(OPENVR_LIBRARIES AND NOT USE_SINGLE_PRECISION)
	add_subdirectory(tools/openvr_driver)
	include_directories(redist include/libsurvive include)
	LIST(APPEND PLUGINS driver_openvr)
	set(driver_openvr_ADDITIONAL_LIBS ${OPENVR_LIBRARIES})
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
foreach(PLUGIN ${PLUGINS})
	SET(SRC src/${PLUGIN}.c)
	if(NOT EXISTS ${CMAKE_SOURCE_DIR}/${SRC})
		SET(SRC src/${PLUGIN}.cc)
	endif()
    add_library(${PLUGIN} SHARED ${SRC} ${${PLUGIN}_ADDITIONAL_SRCS})
    target_link_libraries(${PLUGIN} survive ${${PLUGIN}_ADDITIONAL_LIBS})

	add_dependencies(survive_plugins ${PLUGIN})
    STRING(REGEX REPLACE "(.*)\_.*" "\\1" plugin_type "${PLUGIN}")
    if(plugin_type)
      set_target_properties(${PLUGIN} PROPERTIES FOLDER "${plugin_type}")
    endif()

	set(OUTPUT_DIR "")
	if(CMAKE_LIBRARY_OUTPUT_DIRECTORY)
		set(OUTPUT_DIR "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/")
	endif()
    set_target_properties(${PLUGIN} PROPERTIES PREFIX "")
    set_target_properties(${PLUGIN} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIR}plugins")
    set_target_properties(${PLUGIN} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG "${OUTPUT_DIR}Debug/plugins")
    set_target_properties(${PLUGIN} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${OUTPUT_DIR}Release/plugins")
    set_target_properties(${PLUGIN} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${OUTPUT_DIR}RelWithDebInfo/plugins")

endforeach()

SET(SURVIVE_EXECUTABLES data_recorder survive-cli api_example sensors-readout survive-solver)
IF(TARGET CNGFX)
  list(APPEND SURVIVE_EXECUTABLES simple_pose_test)
  set(simple_pose_test_ADDITIONAL_LIBS CNGFX)
endif()

foreach(executable ${SURVIVE_EXECUTABLES})
  add_executable(${executable} ${executable}.c )
  target_link_libraries(${executable} survive ${${executable}_ADDITIONAL_LIBS})
  set_target_properties(${executable} PROPERTIES FOLDER "apps")
  foreach(PLUGIN ${PLUGINS})
    add_dependencies(${executable} ${PLUGIN})
  endforeach()
endforeach()

IF (ENABLE_TESTS)
	add_subdirectory(src/test_cases)
ENDIF()

find_package(catkin QUIET COMPONENTS
  roscpp
  geometry_msgs)

option(BUILD_ROS_PUBLISHER "Build the ros publisher" catkin_DIR)
IF(BUILD_ROS_PUBLISHER)
	add_subdirectory(tools/ros_publisher)
ENDIF()

file(COPY survive-websocketd DESTINATION ${CMAKE_BINARY_DIR})
install(TARGETS ${PLUGINS} DESTINATION lib/libsurvive/plugins)

install(TARGETS survive DESTINATION lib/libsurvive)
install(TARGETS survive DESTINATION lib)
install(TARGETS survive-cli DESTINATION bin)
install(TARGETS sensors-readout DESTINATION bin)

INSTALL(CODE "execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink survive ${CMAKE_INSTALL_PREFIX}/lib)")

install(DIRECTORY include/libsurvive DESTINATION include)

file(GLOB REDIST_HEADERS
  "redist/*.h"
)
install(FILES ${REDIST_HEADERS} DESTINATION include/libsurvive/redist)
install(PROGRAMS survive-websocketd DESTINATION bin)

find_program(CTYPESGEN ctypesgen PATHS $ENV{HOME}/anaconda3/bin)
if(CTYPESGEN)
	get_property(include_directories DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
	set(INCLUDE_FLAGS "")
	foreach(include_directory ${include_directories})
		list(APPEND INCLUDE_FLAGS "-I${include_directory}")
	endforeach()

	add_custom_target(pysurvive COMMAND ${CTYPESGEN} ${CMAKE_SOURCE_DIR}/include/libsurvive/*.h ${INCLUDE_FLAGS} --no-macros -L$<TARGET_FILE_DIR:survive> -llibsurvive.so
			--strip-prefix=survive_ -P Survive -o ${PYTHON_GENERATED_DIR}pysurvive_generated.py )
endif()

include(GNUInstallDirs)
configure_file(survive.pc.in survive.pc @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/survive.pc" DESTINATION "${CMAKE_INSTALL_LIBDIR}/pkgconfig")
