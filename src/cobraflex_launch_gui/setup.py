"""Package metadata for the CobraFlex launch manager."""

from glob import glob
import os

from setuptools import find_packages, setup


package_name = "cobraflex_launch_gui"


setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    # The style sheet is read from next to the modules, so that the same path
    # resolves in the source tree (mock mode without ROS) and once installed.
    package_data={package_name: ["style.qss"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Samuel Sanchez",
    maintainer_email="sasamt02@hs-esslingen.de",
    description="Qt launch manager with live ROS graph and TF tree views.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "launch_gui = cobraflex_launch_gui.main:main",
        ],
    },
)
