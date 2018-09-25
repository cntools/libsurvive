
add_executable(survive_tests
        main.c
        reproject.c
        kalman.c)

target_link_libraries(survive_tests survive)


add_executable(lintest ../../redist/linmath.c ../../redist/linmath.h ../../redist/lintest.c ../../redist/minimal_opencv.c)
target_link_libraries(lintest lapacke cblas m)

add_test(NAME survive_tests COMMAND survive_tests)
add_test(NAME lintest COMMAND lintest)