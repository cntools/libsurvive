set -o xtrace

mkdir -p bin
cd bin
cmake -DENABLE_TESTS=ON -DUSE_ASAN=${USE_ASAN} -DCMAKE_BUILD_TYPE=${CONFIG} -DCMAKE_INSTALL_PREFIX=INSTALL_ROOT -DUSE_SINGLE_PRECISION=${USE_SINGLE_PRECISION} ..
cmake --build . --config ${CONFIG}
ctest . -C ${CONFIG} --output-on-failure -j30

