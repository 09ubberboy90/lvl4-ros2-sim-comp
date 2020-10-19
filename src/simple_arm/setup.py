from setuptools import setup
from glob import glob
package_name = 'simple_arm'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + "/models/ur10", glob("models/ur10/*.*")),
        ('share/' + package_name + "/models/ur10/meshes", glob("models/ur10/meshes/*.*")),
        ('share/' + package_name + "/models/ur10/meshes/visual", glob("models/ur10/meshes/visual/*.*")),
        ('share/' + package_name + "/models/ur10/meshes/collision", glob("models/ur10/meshes/collision/*.*")),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.*')),
        ('share/' + package_name + '/configs', glob('configs/*.config')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Florent Audonnet',
    maintainer_email='2330834a@student.gla.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_publish = simple_arm.vr_publish:main',
            'ign_publish = simple_arm.ign_publisher:main'
        ],
    },
)
