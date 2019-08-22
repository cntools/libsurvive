
add_executable(survive_tests
        main.c
        reproject.c
        kalman.c rotate_angvel.c watchman.c ../driver_vive.c)

add_definitions(-DDEBUG_WATCHMAN)

target_link_libraries(survive_tests survive)

add_test(NAME survive_tests COMMAND survive_tests)
set_target_properties(survive_tests PROPERTIES FOLDER "tests")

add_executable(test_replays test_replays.c)
set_target_properties(test_replays PROPERTIES FOLDER "tests")
target_link_libraries(test_replays survive)

add_test(NAME lh1_test_cal COMMAND ./test_replays ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/lh1_test_cal.rec.gz)
add_test(NAME lh2_test_cal COMMAND ./test_replays ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data/tests/lh2_test_cal.rec.gz)

add_custom_command(OUTPUT libsurvive-extras-data/.git/config COMMAND git clone https://github.com/jdavidberger/libsurvive-extras-data.git)
add_custom_target(get_test_data COMMAND git pull WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/libsurvive-extras-data DEPENDS libsurvive-extras-data/.git/config)

add_custom_target(run_all_tests COMMAND ${CMAKE_CTEST_COMMAND} DEPENDS test_replays get_test_data survive_tests)

#add_dependencies(lh1_test_cal get_test_data)
#add_dependencies(lh2_test_cal get_test_data)
