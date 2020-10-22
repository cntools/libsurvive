set -o xtrace

#export VS160COMNTOOLS="/c/Program Files (x86)/Microsoft Visual Studio/2019/Community/Common7/Tools"
export PATH="/c/Python38/:/c/Python38/scripts:$PATH"

python setup.py install bdist_wheel
cd bindings/python
python ./example.py --simulator --simulator-time .1 --playback-factor 0
cd ../..

./useful_files/travis/shared.sh

cd bin
cmake --build . --target install --config Release
