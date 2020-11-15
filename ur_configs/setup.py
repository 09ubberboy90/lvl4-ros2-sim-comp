from setuptools import setup
import os
import fnmatch

package_name = 'ur_configs'
data_files = []
for rootPath, dirNames, fileNames in os.walk('meshes'):
    for fileName in fnmatch.filter(fileNames, '*.stl'):
        filePath = os.path.relpath(os.path.join(rootPath, fileName))
        data_files.append(('share/' + package_name + '/' + os.path.dirname(filePath), [filePath]))
# Add DAE files
for rootPath, dirNames, fileNames in os.walk('meshes'):
    for fileName in fnmatch.filter(fileNames, '*.dae'):
        filePath = os.path.relpath(os.path.join(rootPath, fileName))
        data_files.append(('share/' + package_name + '/' + os.path.dirname(filePath), [filePath]))
# Add rviz files
for rootPath, dirNames, fileNames in os.walk('rviz'):
    for fileName in fnmatch.filter(fileNames, '*.rviz'):
        filePath = os.path.relpath(os.path.join(rootPath, fileName))
        data_files.append(('share/' + package_name + '/' + os.path.dirname(filePath), [filePath]))
# Add URDF files
for rootPath, dirNames, fileNames in os.walk('urdf'):
    for fileName in fnmatch.filter(fileNames, '*.urdf'):
        filePath = os.path.relpath(os.path.join(rootPath, fileName))
        data_files.append(('share/' + package_name + '/' + os.path.dirname(filePath), [filePath]))
    # Add Xacro files
    for fileName in fnmatch.filter(fileNames, '*.xacro'):
        filePath = os.path.relpath(os.path.join(rootPath, fileName))
        data_files.append(('share/' + package_name + '/' + os.path.dirname(filePath), [filePath]))
# Other files
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Florent Audonnet',
    maintainer_email='2330834a@student.gla.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
