
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