#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

#fetch values from package.xml
setup_args = generate_distutils_setup(
    packages = ['axis_teleop'],
    package_dir = {'':'src'},
    requires = ['std_msgs', 'rospy', 'axis_camera']
)

setup(**setup_args)
