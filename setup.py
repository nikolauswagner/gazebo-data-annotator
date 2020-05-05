#!/usr/bin/env python

#  Nikolaus Wagner Copyright (c) 2020
#  Email: nwagner@lincoln.ac.uk

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['gazebo-data-annotator'],
    package_dir={'': 'src'},
    install_requires=['cv_bridge',
                      'gazebo_msgs',
                      'geometry_msgs',
                      'rospy',
                      'sensor_msgs',
                      'tf']
)

setup(**setup_args)


