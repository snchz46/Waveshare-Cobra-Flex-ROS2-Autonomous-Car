import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cobraflex_share = get_package_share_directory("cobraflex")
    cobraflex_rl_share = get_package_share_directory("cobraflex_rl")

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=os.path.join(
            cobraflex_share, "worlds", "lane_following_oval_complex.world"
        ),
        description="Path to the Gazebo .world file for RL training "
        "(default: complex_b circuit).",
    )
    # The centerline must match the world: complex_b world -> complex_b
    # right-lane centerline (reward/cage ground-truth lane geometry). Override
    # both together to train on a different track (e.g. world:=lane_following_oval
    # centerline:=.../oval_right_lane_centerline.yaml).
    centerline_arg = DeclareLaunchArgument(
        "centerline",
        default_value=os.path.join(
            cobraflex_rl_share, "config", "complex_b_right_lane_centerline.yaml"
        ),
        description="Centerline YAML matching the world (reward/cage geometry).",
    )
    # gz set_pose/set_physics services are namespaced by the SDF <world name=...>,
    # which differs per world (lane_following_complex_b vs lane_following_oval).
    # Keep this in sync with `world` or episode-reset teleports silently no-op.
    world_name_arg = DeclareLaunchArgument(
        "world_name",
        default_value="lane_following_complex_b",
        description="SDF world name for gz service calls; must match `world`.",
    )
    # Road-centre centerline for off-road geometry: the reward `centerline` is
    # the right lane (offset), but "left the painted road" is judged against the
    # road centre (off_road = global distance > road_width/2). Robust to the
    # complex_b circuit approaching itself, unlike the stateful cross-track ey.
    road_centerline_arg = DeclareLaunchArgument(
        "road_centerline",
        default_value=os.path.join(
            cobraflex_rl_share, "config", "complex_b_centerline.yaml"
        ),
        description="Road-centre centerline YAML for off-road termination geometry.",
    )
    # gui:=false disables the Gazebo client window for headless training.
    # gazebo_mesh.launch.py exposes "gui" (not "headless").
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="false",
        description="Launch Gazebo GUI (set true for debugging).",
    )
    # Training config YAML — selects observation type, hyperparameters AND the
    # algorithm (`algorithm: ppo|sac`); the default mirrors the trainer's own
    # fallback (the F3 state-vector config) so a bare launch is unchanged.
    train_config_arg = DeclareLaunchArgument(
        "train_config",
        default_value=os.path.join(
            cobraflex_rl_share, "config", "train_ppo.yaml"
        ),
        description="Training config YAML (e.g. train_ppo_camera.yaml, "
        "train_sac_camera.yaml).",
    )
    run_id_arg = DeclareLaunchArgument(
        "run_id",
        default_value="",
        description="Run id under experiments/sim/training/ (empty -> "
        "<algorithm>_train_<timestamp>).",
    )
    model_path_arg = DeclareLaunchArgument(
        "model_path",
        default_value="",
        description="Final model .zip path (empty -> the config's model_path).",
    )
    resume_from_arg = DeclareLaunchArgument(
        "resume_from",
        default_value="",
        description="Parent SB3 checkpoint for a continuation run.",
    )
    resume_vecnormalize_arg = DeclareLaunchArgument(
        "resume_vecnormalize",
        default_value="",
        description="VecNormalize state paired with resume_from.",
    )
    resume_replay_buffer_arg = DeclareLaunchArgument(
        "resume_replay_buffer",
        default_value="",
        description="SAC replay buffer paired with resume_from.",
    )
    shutdown_on_train_exit_arg = DeclareLaunchArgument(
        "shutdown_on_train_exit",
        default_value="false",
        description="Shut down Gazebo when training exits (batch/protocol mode).",
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

    def train_node(context):
        # Optional args (run_id/model_path) are appended only when non-empty:
        # the trainer's own defaults (timestamped run id, config model_path)
        # must keep applying when they are not set.
        arguments = [
            "--centerline-config", LaunchConfiguration("centerline").perform(context),
            "--road-centerline-config", LaunchConfiguration("road_centerline").perform(context),
            "--world-name", LaunchConfiguration("world_name").perform(context),
            "--train-config", LaunchConfiguration("train_config").perform(context),
        ]
        run_id = LaunchConfiguration("run_id").perform(context).strip()
        if run_id:
            arguments += ["--run-id", run_id]
        model_path = LaunchConfiguration("model_path").perform(context).strip()
        if model_path:
            arguments += ["--model-path", model_path]
        resume_from = LaunchConfiguration("resume_from").perform(context).strip()
        if resume_from:
            arguments += ["--resume-from", resume_from]
        resume_vecnormalize = LaunchConfiguration("resume_vecnormalize").perform(context).strip()
        if resume_vecnormalize:
            arguments += ["--resume-vecnormalize", resume_vecnormalize]
        resume_replay = LaunchConfiguration("resume_replay_buffer").perform(context).strip()
        if resume_replay:
            arguments += ["--resume-replay-buffer", resume_replay]
        train = Node(
            package="cobraflex_rl",
            executable="train_ppo",
            output="screen",
            arguments=arguments,
        )
        actions = [train]
        shutdown = LaunchConfiguration("shutdown_on_train_exit").perform(context)
        if shutdown.strip().lower() in {"1", "true", "yes", "on"}:
            actions.append(RegisterEventHandler(
                OnProcessExit(
                    target_action=train,
                    on_exit=[EmitEvent(event=Shutdown())],
                )
            ))
        return actions

    return LaunchDescription([
        world_arg,
        centerline_arg,
        road_centerline_arg,
        world_name_arg,
        gui_arg,
        train_config_arg,
        run_id_arg,
        model_path_arg,
        resume_from_arg,
        resume_vecnormalize_arg,
        resume_replay_buffer_arg,
        shutdown_on_train_exit_arg,
        gazebo,
        OpaqueFunction(function=train_node),
    ])
