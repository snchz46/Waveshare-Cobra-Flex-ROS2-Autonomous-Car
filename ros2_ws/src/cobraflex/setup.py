from setuptools import setup

package_name = 'cobraflex'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel',
    maintainer_email='sanchezmorenosamuel23@gmail.com',
    description='ROS 2 control nodes for the Cobra chassis.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cobraflex_node = cobraflex.cobraflex_node:main',
            'lidar_avoidance_node = cobraflex.lidar_avoidance_node:main',
        ],
    },
)
