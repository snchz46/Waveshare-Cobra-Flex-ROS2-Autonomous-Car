from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'cobraflex'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.xml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),          
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),         
    ],

    install_requires=[
        'setuptools',
        'pyserial',     # Needed for serial communication
    ],
    zip_safe=True,
    maintainer='samuel',
    maintainer_email='sanchezmorenosamuel23@gmail.com',
    description='ROS2 driver + avoidance logic for the CobraFlex robot chassis.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cobraflex_ros_driver = cobraflex.cobraflex_ros_driver:main',
            'lidar_avoidance_pid_node = cobraflex.lidar_avoidance_pid_node:main',
        ],
    },
)
