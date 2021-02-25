"""webots_ros2 package setup file."""

import os
import fnmatch
from glob import glob

from setuptools import setup

package_name = 'webots_simple_arm'
worlds = glob('worlds/*.wbt')
worlds.extend(glob("worlds/*.wbo"))
launchers = [
    'launch/panda.launch.py',
    'launch/panda_trajectory.launch.py',
    'launch/moveit_webots.launch.py',
    'launch/collision_webots.launch.py',
    'launch/pick_place.launch.py',
    'launch/vr_webots.launch.py',
]

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name+ "/launch", launchers))
data_files.append(('share/' + package_name + '/worlds', worlds))
data_files.append(('share/' + package_name, ['package.xml']))
data_files.append(('share/' + package_name +'/protos', ['protos/panda.proto']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
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
        'console_scripts': [
            'panda_controller = webots_simple_arm.panda:main',
        ],
    }
)
