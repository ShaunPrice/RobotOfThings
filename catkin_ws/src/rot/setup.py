import os
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup


# ROS PACKAGING
# using distutils : https://docs.python.org/2/distutils
# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['rot'],
    package_dir={'': '/home/rot/catkin_ws/src'},
    package_data={'': ['']}
)
setup(**setup_args)
