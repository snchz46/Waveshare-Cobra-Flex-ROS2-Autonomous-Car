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

        (
            os.path.join("share", package_name, "urdf"),
            [f for f in glob("urdf/*") if os.path.isfile(f)],
        ),
        # The Isaac USD payloads live in a subdirectory, which the file-level
        # glob above skips; without these entries they ship in git but never
        # reach the install share.
        (
            os.path.join("share", package_name, "urdf", "cobraflex_isaac"),
            [f for f in glob("urdf/cobraflex_isaac/*") if os.path.isfile(f)],
        ),
        (
            os.path.join("share", package_name, "urdf", "cobraflex_isaac", "payloads"),
            [f for f in glob("urdf/cobraflex_isaac/payloads/*") if os.path.isfile(f)],
        ),
        (
            os.path.join(
                "share", package_name, "urdf", "cobraflex_isaac", "payloads", "Physics"
            ),
            [
                f
                for f in glob("urdf/cobraflex_isaac/payloads/Physics/*")
                if os.path.isfile(f)
            ],
        ),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        # Maps are gitignored (site-specific, regenerated per environment), so
        # this glob is usually empty on a fresh clone -- navigation.launch.py
        # points at maps/cobraflex_map.yaml and maps/README.md says how to
        # produce it.
        (
            os.path.join("share", package_name, "maps"),
            [f for f in glob("maps/*") if os.path.isfile(f)],
        ),
        (
            os.path.join("share", package_name, "worlds"),
            glob("worlds/*.world") + glob("worlds/*.sdf"),
        ),
        (
            os.path.join("share", package_name, "worlds", "materials"),
            glob("worlds/materials/*"),
        ),
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
    maintainer="Samuel Sanchez",
    maintainer_email="sasamt02@hs-esslingen.de",
    description="ROS 2 driver and autonomy nodes for the CobraFlex chassis.",
    license="MIT",
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
