./survive-cli --init-configfile /dev/null --configfile ./src/test_cases/libsurvive-extras-data/tests/$1.json --no-gss-auto-floor-height --no-gss-threaded --playback-replay-pose --playback ./src/test_cases/libsurvive-extras-data/tests/$1 --playback-factor 0 --no-threaded-posers --v 100 --force-calibrate --playback-replay-external --record rework-$1
mv rework-$1 ./src/test_cases/libsurvive-extras-data/tests/$1
mv ~/.config/libsurvive/$1.json ./src/test_cases/libsurvive-extras-data/tests
