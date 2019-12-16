from setuptools import dist, find_packages
dist.Distribution().fetch_build_eggs(['wheel'])
dist.Distribution().fetch_build_eggs(['cmake_setuptools'])

from setuptools import setup
from cmake_setuptools import *
import os
os.environ['CMAKE_COMMON_VARIABLES'] = '-DPYTHON_GENERATED_DIR="'+ os.path.dirname(os.path.abspath(__file__))+'/bindings/python/pysurvive/"'

setup(name='pysurvive',
      description='',
      version='0.0.1.dev0',
      ext_modules=[CMakeExtension('all', sourcedir='.')],      
      packages=['pysurvive'],
      package_dir={'pysurvive': 'bindings/python/pysurvive'},
      cmdclass={'build_ext': CMakeBuildExt}
      )
