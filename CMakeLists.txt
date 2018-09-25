project(libsurvive)
cmake_minimum_required(VERSION 3.5)

enable_testing()
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -flto -std=gnu99 -rdynamic -Werror=incompatible-pointer-types -Wall -Wno-unused-variable -Wno-switch -Wno-parentheses -Wno-missing-braces")

include_directories(redist include/libsurvive include)

option(LINUX_USE_HIDAPI "Use HIDAPI instead of libusb" OFF)
option(USE_ASAN "Use address sanitizer" OFF)

if(USE_ASAN)
    SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fsanitize=address -fsanitize=undefined")
endif()

if(LINUX_USE_HIDAPI)
    ADD_COMPILE_DEFINITIONS(HIDAPI)
    list(APPEND ADDITIONAL_LIBRARIES udev hidapi-libusb)
endif()

add_library(survive SHARED
        ./include/libsurvive/poser.h
        ./include/libsurvive/survive_api.h
        ./include/libsurvive/survive_optimizer.h
        ./include/libsurvive/survive_reproject.h
        ./include/libsurvive/survive_types.h
        ./redist/crc32.c
        ./redist/glutil.c
        ./redist/jsmn.c
        ./redist/jsmntest.c
        ./redist/json_helpers.c
        ./redist/linmath.c
        ./redist/minimal_opencv.c
        ./redist/mpfit/mpfit.c
        ./redist/puff.c
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
        ./src/survive_imu.c
        ./src/survive_optimizer.c
        ./src/survive_playback.c
        ./src/survive_plugins.c
        ./src/survive_process.c
        ./src/survive_reproject.c
        ./src/survive_reproject.generated.h
        ./src/survive_sensor_activations.c
        ./src/survive_usb.c
        )

add_library(minimal_opencv ./redist/minimal_opencv.c ./redist/minimal_opencv.h)
target_link_libraries(minimal_opencv lapacke cblas)

add_library(sba ./redist/sba/sba_levmar_wrap.c
        ./redist/sba/sba_chkjac.c
        ./redist/sba/sba_lapack.c
        ./redist/sba/sba_levmar.c
        ./redist/sba/sba_crsm.c
        )

add_library(CNGFX
        ./redist/CNFG3D.c
        ./redist/CNFGXDriver.c
        ./redist/CNFGFunctions.c
        )

target_link_libraries(CNGFX X11)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

target_link_libraries(survive minimal_opencv Threads::Threads z m usb-1.0 ${CMAKE_DL_LIBS} ${ADDITIONAL_LIBRARIES})
set_target_properties(survive PROPERTIES LIBRARY_OUTPUT_DIRECTORY "lib")


foreach(executable calibrate data_recorder simple_pose_test survive-cli)
    add_executable(${executable} ${executable}.c )
    target_link_libraries(${executable} survive)
endforeach()
target_link_libraries(calibrate CNGFX)
target_link_libraries(simple_pose_test CNGFX)

SET(PLUGINS
        driver_dummy driver_udp driver_vive
        disambiguator_turvey disambiguator_statebased disambiguator_charles
        poser_dummy poser_mpfit poser_epnp poser_sba poser_imu poser_charlesrefine
)

find_library(PCAP_LIBRARY pcap)
if(PCAP_LIBRARY)
    list(APPEND PLUGINS driver_usbmon)
    set(driver_usbmon_ADDITIONAL_LIBS "${PCAP_LIBRARY}")
else()
    message("Can't build usbmon plugin -- pcap library was not found")
endif()

set(poser_sba_ADDITIONAL_LIBS sba)
set(poser_epnp_ADDITIONAL_SRCS src/epnp/epnp.c)
foreach(PLUGIN ${PLUGINS})
    add_library(${PLUGIN} SHARED src/${PLUGIN}.c ${${PLUGIN}_ADDITIONAL_SRCS})
    target_link_libraries(${PLUGIN} survive ${${PLUGIN}_ADDITIONAL_LIBS})
    set_target_properties(${PLUGIN} PROPERTIES PREFIX "")
    set_target_properties(${PLUGIN} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "lib/plugins")
endforeach()

add_subdirectory(src/test_cases)