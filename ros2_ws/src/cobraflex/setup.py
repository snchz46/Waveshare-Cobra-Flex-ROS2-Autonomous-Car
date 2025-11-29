from setuptools import find_packages, setup

package_name = 'cobraflex'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='samuel',
    maintainer_email='sanchezmorenosamuel23@gmail.com',
    description='ROS 2 control nodes for the Cobra chassis.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cobraflex_cmdvel_driver = cobraflex.cobraflex_cmdvel_driver:main',
            'lidar_avoidance_pid_node = cobraflex.lidar_avoidance_pid_node:main',
        ],
    },
)
