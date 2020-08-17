SET(SURVIVE_TESTS
        reproject
        check_generated
        kalman rotate_angvel export_config)

IF(NOT WIN32)
    LIST(APPEND SURVIVE_TESTS watchman)
    set(watchman_ADDITIONAL_LIBS driver_vive)
endif()
SET(SURVIVE_TESTS_EXE)
foreach(test ${SURVIVE_TESTS})
    list(APPEND SURVIVE_TESTS_EXE test-${test})
    add_executable(test-${test} ${test}.c main.c)
    target_link_libraries(test-${test} survive ${${test}_ADDITIONAL_LIBS})
    add_test(NAME ${test} COMMAND test-${test})
    set_target_properties(test-${test} PROPERTIES FOLDER "tests")
endforeach()

add_definitions(-DDEBUG_WATCHMAN)

add_executable(test_replays test_replays.c)
set_target_properties(test_replays PROPERTIES FOLDER "tests")
target_link_libraries(test_replays survive)

IF(NOT WIN32)
    # This file doesn't have raw gyro data and it was recorded with the wrong corrective values. So we ignore the IMU input
  add_test(NAME lh1_test_cal COMMAND $<TARGET_FILE:test_replays>     ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/lh1_test_cal.rec.gz --imu-acc-variance -1 --imu-gyro-variance -1)
    
  add_test(NAME lh2_test_cal COMMAND $<TARGET_FILE:test_replays>     ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/lh2_test_cal.rec.gz)
  add_test(NAME multi_cal COMMAND $<TARGET_FILE:test_replays> ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/multi-cal.rec.gz)
ENDIF()

if(PCAP_LIBRARY)
	add_test(NAME lh2_test_cal_usb COMMAND $<TARGET_FILE:test_replays> ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/lh2_test_cal.pcap.gz)
	add_test(NAME wm1_wand_test_cal_usb COMMAND $<TARGET_FILE:test_replays> ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/WM1-wand.pcap.gz)
	add_test(NAME wireless_tracker_test_cal_usb COMMAND $<TARGET_FILE:test_replays> ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/wireless-tracker.pcap.gz)
	add_test(NAME vive_cal_usb COMMAND $<TARGET_FILE:test_replays> ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/vive_cal.pcap.gz)
    add_test(NAME vive_index_cal COMMAND $<TARGET_FILE:test_replays> ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/vive-index-cal.pcap.gz)

    # This calibrates 5 lighthouses across three objects
    add_test(NAME vive-and-controllers COMMAND $<TARGET_FILE:test_replays> ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/vive-and-controllers.pcap.gz)

    # This calibrates a wireless tracker with 4 lh. Originally OOTX didn't work.
    add_test(NAME tracker-cal-attempt COMMAND $<TARGET_FILE:test_replays> ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/tracker-cal-attempt.pcap.gz --force-ootx)

    # In this one lighthouse 2 moves by about a foot
    add_test(NAME tracker-lh-move COMMAND $<TARGET_FILE:test_replays> ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/tracker-lh-move.pcap.gz)
endif()

include(ExternalProject)

ExternalProject_Add(libsurvive-extras-data
        GIT_REPOSITORY https://github.com/jdavidberger/libsurvive-extras-data.git
        BUILD_COMMAND ""
        INSTALL_COMMAND ""
        CONFIGURE_COMMAND ""
        SOURCE_DIR ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data
        )

add_custom_target(run_all_tests COMMAND ${CMAKE_CTEST_COMMAND} -j 8 --output-on-failure DEPENDS test_replays libsurvive-extras-data survive_plugins ${SURVIVE_TESTS_EXE})
