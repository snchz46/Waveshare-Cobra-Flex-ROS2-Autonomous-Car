"""Package metadata for the CobraFlex ROS 2 stack."""

import os
from glob import glob

from setuptools import find_packages, setup


package_name = "cobraflex"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.xml"),
        ),
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*.launch.py"),
        ),

        (os.path.join("share", package_name, "urdf"), [f for f in glob("urdf/*") if os.path.isfile(f)]),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world") + glob("worlds/*.sdf")),
        (os.path.join("share", package_name, "worlds", "materials"), glob("worlds/materials/*")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*")),
        
        (
            os.path.join("share", package_name, "materials", "road_assets", "road_textures"),
            glob("materials/road_assets/road_textures/*.png"),
        ),
        (
            os.path.join("share", package_name, "materials", "road_assets", "road_curves"),
            glob("materials/road_assets/road_curves/*.png"),
        ),
        (
            os.path.join("share", package_name, "materials", "road_assets", "road_variants"),
            glob("materials/road_assets/road_variants/*.png"),
        ),
        (
            os.path.join("share", package_name, "materials", "road_assets", "road_variants_lot2"),
            glob("materials/road_assets/road_variants_lot2/*.png"),
        ),
        (
            os.path.join("share", package_name, "materials", "road_assets", "road_tiles"),
            glob("materials/road_assets/road_tiles/*.png"),
        ),
        (
            os.path.join("share", package_name, "materials", "road_assets", "tracks"),
            glob("materials/road_assets/tracks/*.png"),
        ),
    ],
    install_requires=[
        "setuptools",
        "pyserial",
    ],
    zip_safe=True,
    maintainer="samuel",
    maintainer_email="sanchezmorenosamuel23@gmail.com",
    description="ROS 2 driver and autonomy nodes for the CobraFlex chassis.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cobraflex_ros_driver = cobraflex.cobraflex_ros_driver:main",
            "lidar_avoidance_node = cobraflex.lidar_avoidance_node:main",
            "lane_keeper_node = cobraflex.lane_keeper_node:main",
            "lane_keeper_gazebo_node = cobraflex.lane_keeper_gazebo_node:main",
        ],
    },
)
