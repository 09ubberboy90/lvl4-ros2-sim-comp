"""webots_ros2 package setup file."""

import os
import fnmatch

from setuptools import setup

package_name = 'webots_simple_arm'
worlds = [
    'worlds/universal_robot_rviz.wbt',
    'worlds/universal_robot.wbt',
]

launchers = [
    'launch/universal_robot.launch.py',
    'launch/universal_robot_rviz.launch.py',
]

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, launchers))
data_files.append(('share/' + package_name + '/worlds', worlds))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='1.0.2',
    packages=[],
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    author='Cyberbotics',
    author_email='support@cyberbotics.com',
    maintainer='Cyberbotics',
    maintainer_email='support@cyberbotics.com',
    keywords=['ROS', 'Webots', 'Robot', 'Simulation', 'Universal Robots'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Universal Robot ROS2 interface for Webots.',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'launch.frontend.launch_extension': ['launch_ros = launch_ros']
    }
)
