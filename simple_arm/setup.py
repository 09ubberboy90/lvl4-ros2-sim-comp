from setuptools import setup
from glob import glob
package_name = 'simple_arm'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Florent Audonnet',
    maintainer_email='2330834a@student.gla.ac.uk',
    description='Run a pick and place task and a throw task on Ignition and Gazebo',
    license='BSD 3-Clause License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_publish = simple_arm.vr_publish:main',
            'gz_srv_disable = simple_arm.gz_srv_disable:main'
        ],
    },
)
