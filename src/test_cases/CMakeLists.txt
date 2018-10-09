
add_executable(survive_tests
        main.c
        reproject.c
        kalman.c rotate_angvel.c)

target_link_libraries(survive_tests survive)

add_test(NAME survive_tests COMMAND survive_tests)
