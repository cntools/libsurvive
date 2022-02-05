cd cmake-build-release
make -j4 || exit 125
ctest $@ .

