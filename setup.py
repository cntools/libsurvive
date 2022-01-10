import platform
from setuptools import dist, find_packages
dist.Distribution().fetch_build_eggs(['wheel', 'cmake_setuptools'])

dist.Distribution().fetch_build_eggs(['cmake_setuptools', 'scikit-build'])

from skbuild import setup    
import os
import subprocess

this_directory = os.path.abspath(os.path.dirname(__file__))
with open(os.path.join(this_directory, 'README.md'), encoding='utf-8') as f:
      long_description = f.read()


version = subprocess.check_output(["git", "describe", "--tags", "--long"]).strip().decode('utf-8').replace("-", ".")[1:]
version = version[:version.rfind("g")-1]

cmake_args=['-DPYTHON_GENERATED_DIR="'+ os.path.dirname(os.path.abspath(__file__))+'/bindings/python/pysurvive/"',
            "-DDOWNLOAD_EIGEN=ON",
            "-DUSE_EIGEN=ON",
	    "-DBUILD_APPLICATIONS=OFF",
            "-DLIB_INSTALL_DIR=bindings/python/pysurvive/"]

if platform.system() != 'Windows':
      cmake_args.append('-DUSE_EIGEN=ON')

setup(name='pysurvive',
      version=version,
      long_description=long_description,
      long_description_content_type='text/markdown',
      description='Libsurvive is a set of tools and libraries that enable 6 dof tracking on lighthouse and vive based systems that is completely open source and can run on any device. It currently supports both SteamVR 1.0 and SteamVR 2.0 generation of devices and should support any tracked object commercially available.',
      url='https://github.com/cntools/libsurvive',
      packages=['pysurvive'],
      package_dir={'pysurvive': 'bindings/python/pysurvive'},
      package_data={'pysurvive': ['images/*']},
      install_requires=['gooey'],
      include_package_data=False,
      license='MIT',
      cmake_args=cmake_args
      )
