from setuptools import dist, find_packages
dist.Distribution().fetch_build_eggs(['wheel', 'cmake_setuptools'])
ctypesgen_install = dist.Distribution().fetch_build_egg('ctypesgen')

dist.Distribution().fetch_build_eggs(['cmake_setuptools', 'scikit-build'])

from skbuild import setup
import os

os.environ['PYTHONPATH'] = ctypesgen_install.module_path
os.environ['PATH'] += ":" + ctypesgen_install.module_path + '/ctypesgen'

os.environ['CMAKE_COMMON_VARIABLES'] = \
    '-DPYTHON_GENERATED_DIR="'+ os.path.dirname(os.path.abspath(__file__))+'/bindings/python/pysurvive/"'

setup(name='pysurvive',
      description='',
      version=os.environ.get("TRAVIS_TAG", "develop"),
      packages=['pysurvive'],
      package_dir={'pysurvive': 'bindings/python/pysurvive'},
      include_package_data=True,      
      )
