from setuptools import setup
from glob import glob
import os

package_name = "cobraflex_rl"


setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=[
        "setuptools",
        "numpy",
        "PyYAML",
        "gymnasium",
        "stable-baselines3",
    ],
    zip_safe=True,
    maintainer="Samuel Sanchez",
    maintainer_email="sasamt02@hs-esslingen.de",
    description="ROS2 PPO lane-following package for Gazebo.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "train_ppo = cobraflex_rl.train_ppo:main",
            "eval_policy = cobraflex_rl.eval_policy:main",
            "eval_cv_controller = cobraflex_rl.eval_cv_controller:main",
            "gazebo_lane_env = cobraflex_rl.gazebo_lane_env:main",
            "lane_perception_node = cobraflex_rl.lane_perception_node:main",
            "cv_lane_estimator_node = cobraflex_rl.cv_lane_estimator_node:main",
            "csi_camera_node = cobraflex_rl.csi_camera_node:main",
            "rl_policy_node = cobraflex_rl.rl_policy_node:main",
            "vehicle_control_node = cobraflex_rl.vehicle_control_node:main",
            "pd_baseline_node = cobraflex_rl.pd_baseline_node:main",
            "cage_logger_node = cobraflex_rl.cage_logger_node:main",
        ],
    },
)
