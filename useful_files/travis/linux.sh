set -o xtrace

virtualenv --python=python3 venv
source ./venv/bin/activate
python3 setup.py install bdist_wheel
cd bindings/python
python3 ./example.py --simulator --simulator-time .1 --playback-factor 0
cd ../..
./useful_files/travis/shared.sh

cd bin
sudo make install
