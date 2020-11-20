SET(SURVIVE_LIBRARY_TYPE SHARED)
if(BUILD_STATIC)
  SET(SURVIVE_LIBRARY_TYPE STATIC)
  add_definitions(-DSURVIVE_DISABLE_PLUGINS)
endif()

list (APPEND CMAKE_REQUIRED_DEFINITIONS -D_GNU_SOURCE)
check_symbol_exists (fopencookie stdio.h HAVE_FOPENCOOKIE)

set(CMAKE_REQUIRED_LIBRARIES z)
check_symbol_exists (gzvprintf zlib.h HAVE_GZVPRINTF)

if(HAVE_FOPENCOOKIE)
  add_definitions(-DHAVE_FOPENCOOKIE)
endif()

if(HAVE_GZVPRINTF)
    add_definitions(-DHAVE_GZVPRINTF)
endif()


SET(SURVIVE_SRCS ${SURVIVE_SRCS}
  ootx_decoder.c
  poser.c
  poser_general_optimizer.c
  survive.c
  survive_api.c
        survive_config.c
  survive_default_devices.c
  survive_disambiguator.c
  survive_driverman.c
  survive_kalman_tracker.c
  survive_optimizer.c
  survive_recording.c        
  survive_plugins.c
        survive_process.c
  survive_process_gen2.c
  survive_sensor_activations.c
  survive_kalman.c
  barycentric_svd/barycentric_svd.c
  survive_reproject_gen2.c
        survive_process_gen1.c
        survive_reproject.c
  lfsr.c
  lfsr_lh2.c
  survive_str.h survive_str.c test_cases/str.c
  survive_async_optimizer.c
  ../redist/linmath.c ../redist/puff.c ../redist/symbol_enumerator.c
  ../redist/jsmn.c ../redist/json_helpers.c ../redist/crc32.c
  )

if(USE_HIDAPI OR WIN32)
  add_definitions (-DHIDAPI)
  IF(WIN32)
    SET(SURVIVE_SRCS ${SURVIVE_SRCS} ../redist/hid-windows.c ../winbuild/getdelim.c)
  else()
    list(APPEND ADDITIONAL_LIBRARIES udev hidapi-libusb)
  endif()
endif()

add_library(survive ${SURVIVE_LIBRARY_TYPE} ${SURVIVE_SRCS})
set_target_properties(survive PROPERTIES
	FOLDER "libraries"
	SOVERSION ${PROJECT_VERSION_MAJOR}
	VERSION ${PROJECT_VERSION})
set_target_properties(survive PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set_target_properties(survive PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")

if(WIN32)
    set_target_properties(survive PROPERTIES PREFIX "lib")
endif()

target_link_libraries(survive minimal_opencv ${ADDITIONAL_LIBRARIES} mpfit)
IF(HAVE_ZLIB_H)
  target_link_libraries(survive z)
ENDIF()

IF(Threads_FOUND)
  target_link_libraries(survive ${CMAKE_THREAD_LIBS_INIT} Threads::Threads)
endif()

if(NOT BUILD_STATIC)
  target_link_libraries(survive ${CMAKE_DL_LIBS})
endif()

IF(NOT WIN32)
	target_link_libraries(survive m )
else()
	set_target_properties( survive PROPERTIES VS_USER_PROPS "$(SolutionDir)\\packages\\OpenBLAS.0.2.14.1\\build\\native\\openblas.targets" )
	target_link_libraries(survive DbgHelp SetupAPI)
endif()


SET(PLUGINS
        driver_dummy driver_vive driver_simulator driver_playback
        disambiguator_statebased
        poser_dummy poser_mpfit poser_kalman_only
	poser_barycentric_svd driver_global_scene_solver
)

if(BUILD_GATT_SUPPORT)
	LIST(APPEND PLUGINS  driver_gatt)
	set(driver_gatt_ADDITIONAL_LIBS gattlib)
endif()

IF(NOT WIN32)
  LIST(APPEND PLUGINS driver_udp)

	set(driver_vive_ADDITIONAL_LIBS usb-1.0)

	find_library(PCAP_LIBRARY pcap)
  if(PCAP_LIBRARY)
    list(APPEND PLUGINS driver_usbmon)
    set(driver_usbmon_ADDITIONAL_LIBS "${PCAP_LIBRARY};z" driver_vive)
  else()
    message("Can't build usbmon plugin -- pcap library was not found")
  endif()
endif()

set(poser_barycentric_svd_ADDITIONAL_SRCS barycentric_svd/barycentric_svd.c )

add_custom_target(survive_plugins)

if(OPENVR_LIBRARIES AND NOT USE_SINGLE_PRECISION)
	include_directories(redist include/libsurvive include ${OPENVR_INCLUDE_PATH})
	LIST(APPEND PLUGINS driver_openvr)
	set(driver_openvr_ADDITIONAL_LIBS ${OPENVR_LIBRARIES})
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)
set(SURVIVE_BUILT_PLUGINS)

function(SURVIVE_REGISTER_PLUGIN PLUGIN)
  IF(NOT TARGET ${PLUGIN})
    SET(SRC ${PLUGIN}.c)
    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${SRC})
      SET(SRC ${PLUGIN}.cc)
    endif()
    add_library(${PLUGIN} ${SURVIVE_LIBRARY_TYPE} ${SRC} ${${PLUGIN}_ADDITIONAL_SRCS})
    target_link_libraries(${PLUGIN} survive ${${PLUGIN}_ADDITIONAL_LIBS})
  ENDIF()
  
  target_link_libraries(${PLUGIN} survive)
  list(APPEND SURVIVE_BUILT_PLUGINS ${PLUGIN})
  set(SURVIVE_BUILT_PLUGINS "${SURVIVE_BUILT_PLUGINS}" PARENT_SCOPE)
  
  add_dependencies(survive_plugins ${PLUGIN})
  STRING(REGEX REPLACE "(.*)\_.*" "\\1" plugin_type "${PLUGIN}")
  if(plugin_type)
    set_target_properties(${PLUGIN} PROPERTIES FOLDER "${plugin_type}")
  endif()

  set(OUTPUT_DIR "$<TARGET_FILE_DIR:survive>/")#set(OUTPUT_DIR "${survive_location}")

  if(NOT BUILD_STATIC)
    set_target_properties(${PLUGIN} PROPERTIES PREFIX "")
  endif()
  set_target_properties(${PLUGIN} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${OUTPUT_DIR}plugins")
  set_target_properties(${PLUGIN} PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${OUTPUT_DIR}plugins")
  #set_target_properties(${PLUGIN} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG "${OUTPUT_DIR}Debug/plugins")
  #set_target_properties(${PLUGIN} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELEASE "${OUTPUT_DIR}Release/plugins")
  #set_target_properties(${PLUGIN} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_RELWITHDEBINFO "${OUTPUT_DIR}RelWithDebInfo/plugins")

if(NOT WIN32)
  install(TARGETS ${PLUGIN} DESTINATION lib/libsurvive/plugins)
else()
  install(TARGETS ${PLUGIN} DESTINATION bin/plugins)
endif()

endfunction()

foreach(PLUGIN ${PLUGINS})
  option(ENABLE_${PLUGIN} "Build ${PLUGIN}" ${NOT_CORE_BUILD})
  if(ENABLE_${PLUGIN})
    SURVIVE_REGISTER_PLUGIN(${PLUGIN})
  endif()
endforeach()

IF(TARGET CNGFX)
  list(APPEND SURVIVE_EXECUTABLES simple_pose_test)
  set(simple_pose_test_ADDITIONAL_LIBS CNGFX)
endif()

IF (ENABLE_TESTS)
  add_subdirectory(test_cases)
ENDIF()

set(SURVIVE_BUILT_PLUGINS "${SURVIVE_BUILT_PLUGINS}" PARENT_SCOPE)

if(NOT WIN32)
install(TARGETS ${SURVIVE_BUILT_PLUGINS} DESTINATION lib/libsurvive/plugins)
install(TARGETS survive DESTINATION lib/libsurvive)
install(TARGETS survive DESTINATION lib)
else()
install(TARGETS ${SURVIVE_BUILT_PLUGINS} DESTINATION bin/plugins)
install(TARGETS survive DESTINATION bin)
endif()


if(WIN32)
  cmake_policy(SET CMP0087 NEW)
  install(CODE [[  
  file(GET_RUNTIME_DEPENDENCIES LIBRARIES $<TARGET_FILE:survive>
    RESOLVED_DEPENDENCIES_VAR _r_deps
    UNRESOLVED_DEPENDENCIES_VAR _u_deps
	POST_EXCLUDE_REGEXES ".*[Ww][Ii][Nn][Dd][Oo][Ww][Ss][/\\][Ss]ystem32.*"
  )
  foreach(_file ${_r_deps})
    file(INSTALL
      DESTINATION "${CMAKE_INSTALL_PREFIX}/bin"
      TYPE SHARED_LIBRARY     
      FILES "${_file}"
    )
  endforeach()
]])
endif()