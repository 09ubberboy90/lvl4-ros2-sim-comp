# Universal Robot

This package provides an interface between ROS2 and the [UR3e, UR5e and UR10e simulation models](https://cyberbotics.com/doc/guide/ure) of the [Universal Robots](https://www.universal-robots.com) running in Webots.
It includes several simulations of these robots.

Documentation is available [here](https://github.com/cyberbotics/webots_ros2/wiki/Example-Universal-Robots).

Urdf generated from moveit package with : `python -m urdf2webots.importer --input=/home/ubb/Documents/ros/moveit2_ws/src/moveit_resources/panda_description/urdf/panda.urdf --output=./panda.proto --static-base`