import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cobraflex_share = get_package_share_directory("cobraflex")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(cobraflex_share, "worlds", "lane_following_oval.world"),
        description="Path to the Gazebo .world file for evaluation.",
    )
    # gui defaults to true here (vs false for training): evaluation runs are the
    # ones you watch / screen-capture for the §7.5 qualitative figures + video.
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Launch Gazebo GUI (true to record the evaluation run).",
    )
    # Episodes to evaluate (one full nominal lap-set is the §7.5 default).
    episodes_arg = DeclareLaunchArgument(
        "episodes",
        default_value="1",
        description="Number of evaluation episodes.",
    )
    # Horizon override: 0 keeps train_ppo.yaml's max_episode_steps; set e.g.
    # 4400 (~10 continuous laps) for the §7.5.1 lap-count comparison vs the PD.
    max_steps_arg = DeclareLaunchArgument(
        "max_steps",
        default_value="0",
        description="Override max_episode_steps (0 = use config default).",
    )
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="cobraflex_ppo_lane",
        description="Path to the trained PPO checkpoint (.zip).",
    )
    # F4 scenario campaign: drive the run from an SC-* YAML (initial conditions,
    # commanded speed, timeout) and score its pass_criterion. Empty = legacy
    # SC-NOM-01 §7.5 eval.
    scenario_arg = DeclareLaunchArgument(
        "scenario",
        default_value="",
        description="Path to a scenario YAML (scenarios/<cat>/*.yaml); empty = NOM-01 eval.",
    )
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="enforcement",
        description="Cage mode: enforcement | monitoring.",
    )
    rep_arg = DeclareLaunchArgument(
        "rep",
        default_value="0",
        description="Repetition index (seeds the per-run randomisation jitter).",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cobraflex_share, "launch", "gazebo_mesh.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )

    eval_node = Node(
        package="cobraflex_rl",
        executable="eval_policy",
        output="screen",
        arguments=[
            "--episodes", LaunchConfiguration("episodes"),
            "--max-steps", LaunchConfiguration("max_steps"),
            "--model-path", LaunchConfiguration("model_path"),
            "--scenario", LaunchConfiguration("scenario"),
            "--mode", LaunchConfiguration("mode"),
            "--rep", LaunchConfiguration("rep"),
        ],
    )

    return LaunchDescription([
        world_arg,
        gui_arg,
        episodes_arg,
        max_steps_arg,
        model_path_arg,
        scenario_arg,
        mode_arg,
        rep_arg,
        gazebo,
        eval_node,
    ])
