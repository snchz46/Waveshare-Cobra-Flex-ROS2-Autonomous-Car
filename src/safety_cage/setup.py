from glob import glob
import os

from setuptools import setup

package_name = "safety_cage"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Samuel Sanchez",
    maintainer_email="sasamt02@hs-esslingen.de",
    description="ROS2 wrapper for the safety cage.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cage_ros_node = safety_cage.cage_ros_node:main",
        ],
    },
)
