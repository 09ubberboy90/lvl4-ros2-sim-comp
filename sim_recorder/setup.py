from setuptools import setup

package_name = 'sim_recorder'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubb',
    maintainer_email='2330834a@student.gla.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "proc_monitor = sim_recorder.proc_monitor:main",
            "proc_monitor_gui = sim_recorder.proc_monitor_gui:main",
            "webots_spawner = sim_recorder.webots_spawner:main",
            "run_recording = sim_recorder.run_recording:main"
        ],
    },
)
