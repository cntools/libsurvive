set -o xtrace

7z a libsurvive-$TRAVIS_TAG-$TRAVIS_OS_NAME-$CONFIG.zip ./bin/INSTALL_ROOT/*
