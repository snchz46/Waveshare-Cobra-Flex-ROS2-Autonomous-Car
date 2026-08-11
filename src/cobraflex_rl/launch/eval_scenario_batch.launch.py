"""
eval_scenario_batch — headless, single-run launch for the F4 scenario-validation
campaign. Brings up Gazebo + the eval_policy node for one (scenario, mode, rep)
cell and **shuts the whole launch down** the moment the eval node exits, so the
campaign orchestrator (tools/run_campaign.py execute_run) can drive it as a
blocking subprocess without Gazebo lingering between runs.

Differs from eval_lane.launch.py (the watch/record eval) only in: gui defaults
off, run_id/output_root are exposed so the orchestrator can locate the result,
and the on-exit shutdown handler.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cobraflex_share = get_package_share_directory("cobraflex")

    args = [
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(cobraflex_share, "worlds", "lane_following_oval.world"),
            description="Path to the Gazebo .world file.",
        ),
        DeclareLaunchArgument("gui", default_value="false",
                              description="Gazebo GUI (off for batch)."),
        DeclareLaunchArgument("rviz", default_value="false",
                              description="RViz (off for batch; on to watch a run)."),
        DeclareLaunchArgument("model_path", default_value="cobraflex_ppo_lane",
                              description="Trained PPO checkpoint (.zip)."),
        DeclareLaunchArgument("scenario", default_value="",
                              description="Scenario YAML path (required for a campaign run)."),
        DeclareLaunchArgument("mode", default_value="enforcement",
                              description="Cage mode: enforcement | monitoring."),
        DeclareLaunchArgument("rep", default_value="0",
                              description="Repetition index (seeds the jitter)."),
        DeclareLaunchArgument("criterion_arm", default_value="",
                              description="Labelled multi-arm criterion arm."),
        DeclareLaunchArgument("protocol_manifest", default_value="",
                              description="Completed two-arm protocol manifest."),
        DeclareLaunchArgument("max_steps", default_value="0",
                              description="Override max_episode_steps (0 = scenario timeout)."),
        DeclareLaunchArgument("run_id", default_value="",
                              description="Output run id (empty = timestamp)."),
        DeclareLaunchArgument("output_root", default_value="",
                              description="Directory holding run subdirs."),
        # Track 'E': the camera campaign passes train_ppo_camera.yaml so
        # eval_policy builds the camera env + frame stack; empty keeps the
        # F-track default (train_ppo.yaml).
        DeclareLaunchArgument("train_config", default_value="",
                              description="Training config YAML for the eval env "
                                          "(empty = package default train_ppo.yaml)."),
        # Track geometry overrides (empty = eval_policy's oval defaults, so the
        # F-track / oval campaign cells are byte-identical). The complex_b camera
        # campaign passes all three from the scenario's track block.
        DeclareLaunchArgument("centerline", default_value="",
                              description="Lane centerline YAML (reward target / Frenet "
                                          "frame); empty = oval_right_lane_centerline.yaml."),
        DeclareLaunchArgument("road_centerline", default_value="",
                              description="Road-centre centerline YAML for off-road geometry "
                                          "(complex_b self-approach, docs/11 §3.5); empty = none."),
        DeclareLaunchArgument("world_name", default_value="",
                              description="SDF world name for the gz teleport services "
                                          "(e.g. lane_following_complex_b); empty = eval_policy default."),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cobraflex_share, "launch", "gazebo_mesh.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )

    eval_node = Node(
        package="cobraflex_rl",
        executable="eval_policy",
        output="screen",
        arguments=[
            "--model-path", LaunchConfiguration("model_path"),
            "--scenario", LaunchConfiguration("scenario"),
            "--mode", LaunchConfiguration("mode"),
            "--rep", LaunchConfiguration("rep"),
            "--criterion-arm", LaunchConfiguration("criterion_arm"),
            "--protocol-manifest", LaunchConfiguration("protocol_manifest"),
            "--max-steps", LaunchConfiguration("max_steps"),
            "--run-id", LaunchConfiguration("run_id"),
            "--output-root", LaunchConfiguration("output_root"),
            # Empty string falls back to the package default inside eval_policy.
            "--train-config", LaunchConfiguration("train_config"),
            # Track geometry: eval_policy treats an empty value as "use default"
            # (`centerline_config or <oval>`; `if road_centerline_config:`;
            # `if world_name:`), so passing "" keeps the oval behaviour.
            "--centerline-config", LaunchConfiguration("centerline"),
            "--road-centerline-config", LaunchConfiguration("road_centerline"),
            "--world-name", LaunchConfiguration("world_name"),
        ],
    )

    # Tear the launch (and Gazebo) down as soon as the eval node finishes, so a
    # blocking `ros2 launch` subprocess returns instead of hanging on Gazebo.
    shutdown_on_eval_exit = RegisterEventHandler(
        OnProcessExit(target_action=eval_node, on_exit=[EmitEvent(event=Shutdown())])
    )

    return LaunchDescription(args + [gazebo, eval_node, shutdown_on_eval_exit])
