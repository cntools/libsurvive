set -o xtrace

7z a libsurvive-$TRAVIS_TAG-$TRAVIS_OS_NAME-$CONFIG$RELEASE_FILE_SUFFIX.zip ./bin/INSTALL_ROOT/* 
