set -o xtrace

#choco upgrade dotnetcore
choco install python --version=3.8.6

export PATH="/c/Python38/:$PATH"
python -m ensurepip
python -m pip install wheel setuptools ctypesgen

#choco install visualstudio2019community
#choco install visualstudio2019-workload-nativedesktop



