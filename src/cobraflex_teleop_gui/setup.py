"""Package metadata for the CobraFlex teleoperation GUI."""

from glob import glob
import os

from setuptools import find_packages, setup


package_name = "cobraflex_teleop_gui"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Samuel Sanchez",
    maintainer_email="sasamt02@hs-esslingen.de",
    description="Qt teleoperation window for the CobraFlex chassis.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "teleop_gui = cobraflex_teleop_gui.main:main",
        ],
    },
)
