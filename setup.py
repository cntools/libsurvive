from setuptools import dist, find_packages
dist.Distribution().fetch_build_eggs(['wheel', 'cmake_setuptools'])
ctypesgen_install = dist.Distribution().fetch_build_egg('ctypesgen')

ctypesgen = open(ctypesgen_install.module_path + "/ctypesgen/ctypesgen", "w")
print(ctypesgen, ctypesgen_install)
ctypesgen.write('''
import re
import sys
from ctypesgen.main import main
if __name__ == '__main__':
    sys.argv[0] = re.sub(r'(-script\.pyw|\.exe)?$', '', sys.argv[0])
    sys.exit(main())
''');
ctypesgen.close()

dist.Distribution().fetch_build_eggs(['cmake_setuptools'])

from setuptools import setup
from cmake_setuptools import *
import os

os.environ['PYTHONPATH'] = ctypesgen_install.module_path
os.environ['PATH'] += ":" + ctypesgen_install.module_path + '/ctypesgen'

os.environ['CMAKE_COMMON_VARIABLES'] = \
    '-DPYTHON_GENERATED_DIR="'+ os.path.dirname(os.path.abspath(__file__))+'/bindings/python/pysurvive/" '

setup(name='pysurvive',
      description='',
      version='0.0.1.dev0',
      ext_modules=[CMakeExtension('all', sourcedir='.')],      
      packages=['pysurvive'],
      package_dir={'pysurvive': 'bindings/python/pysurvive'},
      cmdclass={'build_ext': CMakeBuildExt}
      )
