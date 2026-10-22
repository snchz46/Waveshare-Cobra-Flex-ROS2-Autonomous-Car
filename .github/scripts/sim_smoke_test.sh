#!/usr/bin/env bash
# Smoke test of the simulation procedures of docs/2_USAGE.md and lab/README.md
# on a headless machine (Xvfb, software OpenGL).
#
# Each check prints PASS, FAIL or WARN. FAIL marks the job as failed; WARN marks
# checks whose outcome depends on the simulation (for example whether Nav2
# reaches a goal in software rendering) and does not fail the job. Logs of
# every process are written to $LOG_DIR and uploaded as a workflow artifact.

set +u
set -o pipefail

LOG_DIR=${LOG_DIR:-$HOME/smoke_logs}
SCRIPTS=$(cd "$(dirname "$0")" && pwd)
WS=$HOME/ros2_ws
mkdir -p "$LOG_DIR"

FAILED=0
pass() { echo "PASS  $1"; }
fail() { echo "FAIL  $1"; echo "::error title=Smoke test::$1"; FAILED=1; }
warn() { echo "WARN  $1"; echo "::warning title=Smoke test::$1"; }
info() { echo "INFO  $1"; }

check() {  # check <name> <command...>: FAIL when the command fails
  local name=$1; shift
  echo "### $name" >>"$LOG_DIR/checks.log"
  if "$@" >>"$LOG_DIR/checks.log" 2>&1; then pass "$name"; else fail "$name"; fi
}
soft() {   # soft <name> <command...>: WARN when the command fails
  local name=$1; shift
  echo "### $name" >>"$LOG_DIR/checks.log"
  if "$@" >>"$LOG_DIR/checks.log" 2>&1; then pass "$name"; else warn "$name"; fi
}
probe() { python3 "$SCRIPTS/probe.py" "$@"; }

declare -A PGID=()
start() {  # start <name> <command...>: run in its own process group
  local name=$1; shift
  setsid "$@" >"$LOG_DIR/$name.log" 2>&1 &
  PGID[$name]=$!
  info "started $name: $*"
}
stop() {   # stop <name>...
  for name in "$@"; do
    [ -n "${PGID[$name]}" ] && kill -INT -- -"${PGID[$name]}" 2>/dev/null
  done
  sleep 8
  for name in "$@"; do
    [ -n "${PGID[$name]}" ] && kill -KILL -- -"${PGID[$name]}" 2>/dev/null
    unset "PGID[$name]"
  done
}
stop_all() {
  stop "${!PGID[@]}"
  pkill -KILL -f "gz sim" 2>/dev/null
  pkill -KILL -f rviz2 2>/dev/null
  ros2 daemon stop >/dev/null 2>&1
  sleep 3
}
trap stop_all EXIT

# ---- environment -------------------------------------------------------------
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash || { fail "ROS 2 Humble not installed (/opt/ros/humble)"; exit 1; }
# shellcheck disable=SC1091
source "$WS/install/setup.bash" || { fail "workspace not built ($WS/install/setup.bash)"; exit 1; }
export ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=42 GZ_IP=127.0.0.1
export LIBGL_ALWAYS_SOFTWARE=1 QT_X11_NO_MITSHM=1
Xvfb :99 -screen 0 1280x1024x24 >"$LOG_DIR/xvfb.log" 2>&1 &
export DISPLAY=:99
sleep 2

info "gz: $(command -v gz || echo missing); ros_gz_sim: $(ros2 pkg prefix ros_gz_sim 2>/dev/null || echo missing)"
check "package list contains all five packages" bash -c \
  'for p in cobraflex cobraflex_rl cobraflex_safety_msgs cobraflex_teleop_gui safety_cage; do ros2 pkg list | grep -qx "$p" || { echo "missing $p"; exit 1; }; done'

# ---- static checks (docs/2_USAGE.md §7) ---------------------------------------
for urdf in "$WS"/src/cobraflex/urdf/my_robot_*.urdf; do
  check "xacro expands $(basename "$urdf")" bash -c "xacro '$urdf' > /dev/null"
done
for world in "$WS"/src/cobraflex/worlds/*.world "$WS"/src/cobraflex/worlds/*.sdf; do
  check "gz sdf -k $(basename "$world")" gz sdf -k "$world"
done

if ! command -v gz >/dev/null || ! ros2 pkg prefix ros_gz_sim >/dev/null 2>&1; then
  fail "Gazebo Harmonic or ros_gz_sim missing: simulation checks skipped"
else
  # ---- §1 Simulation: gazebo.launch.py ----------------------------------------
  start gazebo ros2 launch cobraflex gazebo.launch.py
  check "gazebo.launch.py: /clock"                 probe --topic /clock --timeout 240
  check "gazebo.launch.py: /scan"                  probe --topic /scan --timeout 180 --expr "len(msg.ranges) > 0"
  check "gazebo.launch.py: /odom"                  probe --topic /odom --timeout 120
  check "gazebo.launch.py: /odom_truth"            probe --topic /odom_truth --timeout 120
  check "gazebo.launch.py: /imu"                   probe --topic /imu --timeout 120
  check "gazebo.launch.py: /joint_states"          probe --topic /joint_states --timeout 120
  check "gazebo.launch.py: /camera/image_raw_lane" probe --topic /camera/image_raw_lane --timeout 180 --expr "msg.width == 640 and msg.height == 360"
  check "gazebo.launch.py: /odometry/filtered"     probe --topic /odometry/filtered --timeout 120
  check "gazebo.launch.py: TF odom -> base_footprint" probe --tf odom base_footprint --timeout 60
  check "gazebo.launch.py: TF base_footprint -> lidar_link" probe --tf base_footprint lidar_link --timeout 60

  # ---- §1 SLAM: mapping.launch.py, map_saver_cli, rebuild ----------------------
  start mapping ros2 launch cobraflex mapping.launch.py
  check "mapping.launch.py: /map"                  probe --topic /map --timeout 180 --expr "msg.info.width > 0"
  check "mapping.launch.py: TF map -> odom"        probe --tf map odom --timeout 60
  rm -f "$WS"/src/cobraflex/maps/cobraflex_map.*
  check "map_saver_cli writes cobraflex_map.yaml/.pgm" bash -c \
    "timeout 90 ros2 run nav2_map_server map_saver_cli -f $WS/src/cobraflex/maps/cobraflex_map \
     && test -f $WS/src/cobraflex/maps/cobraflex_map.yaml && test -f $WS/src/cobraflex/maps/cobraflex_map.pgm"
  stop mapping
  check "rebuild installs the map into the share" bash -c \
    "cd $WS && colcon build --packages-select cobraflex --symlink-install \
     && test -e $WS/install/cobraflex/share/cobraflex/maps/cobraflex_map.yaml"

  # ---- §1 Navigation: navigation.launch.py -------------------------------------
  start navigation ros2 launch cobraflex navigation.launch.py
  check "navigation.launch.py: bt_navigator active" bash -c \
    'for i in $(seq 1 90); do ros2 lifecycle get /bt_navigator 2>/dev/null | grep -q "^active" && exit 0; sleep 2; done; exit 1'
  ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {orientation: {w: 1.0}}}}" >>"$LOG_DIR/checks.log" 2>&1
  check "navigation.launch.py: /global_costmap/costmap" probe --topic /global_costmap/costmap --timeout 120
  soft "navigation.launch.py: goal 0.4 m ahead reached" bash -c \
    "timeout 240 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
     '{pose: {header: {frame_id: map}, pose: {position: {x: 0.4, y: 0.0}, orientation: {w: 1.0}}}}' \
     | tee /dev/stderr | grep -q SUCCEEDED"
  stop navigation

  # ---- §1 Drive by /cmd_vel and obstacle avoidance ------------------------------
  x0=$(probe --topic /odom_truth --print "round(msg.pose.pose.position.x, 3)" --timeout 30 | tail -1)
  timeout 6 ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: -0.15}}" >/dev/null 2>&1
  x1=$(probe --topic /odom_truth --print "round(msg.pose.pose.position.x, 3)" --timeout 30 | tail -1)
  info "odom_truth x before/after /cmd_vel: $x0 -> $x1"
  check "robot moves on /cmd_vel" python3 -c "import sys; sys.exit(0 if abs(float('$x1') - float('$x0')) > 0.05 else 1)"
  start avoidance ros2 run cobraflex lidar_avoidance_node
  check "lidar_avoidance_node publishes /cmd_vel" probe --topic /cmd_vel --timeout 60
  stop avoidance
  stop_all

  # ---- §1 Lane keeping: lane_keeper_gazebo.launch.py ---------------------------
  start lane_keeper ros2 launch cobraflex lane_keeper_gazebo.launch.py
  check "lane_keeper_gazebo.launch.py: /lane/image_overlay" probe --topic /lane/image_overlay --timeout 240
  check "lane_keeper_gazebo.launch.py: lane detected, forward command" \
    probe --topic /cmd_vel --timeout 120 --expr "msg.linear.x > 0.1" --print "msg.angular.z"
  stop_all

  # ---- Teleoperation GUI --------------------------------------------------------
  start teleop ros2 launch cobraflex_teleop_gui teleop_gui.launch.py
  soft "teleop_gui.launch.py publishes /cmd_vel" probe --topic /cmd_vel --timeout 60
  stop_all

  # ---- Safety-cage demonstration (requires the thesis repository) -------------
  start cage_demo ros2 launch safety_cage lane_following.launch.py
  sleep 60
  if grep -q "No module named 'cage'" "$LOG_DIR/cage_demo.log"; then
    info "lane_following.launch.py stops with ModuleNotFoundError 'cage', as documented (thesis repository not installed)"
  else
    soft "safety_cage lane_following.launch.py: /safe_action" probe --topic /safe_action --timeout 120
  fi
  stop_all
fi

# ---- lab/README.md ---------------------------------------------------------------
check "lab session 2: open gap reported as NameError ___A___" bash -c \
  "cd $WS/lab/session2 && timeout 30 python3 emergency_brake_node.py 2>&1 | tee /dev/stderr | grep -q \"NameError: name '___A___' is not defined\""
check "lab session 1: cobraflex_lab package creation and build" bash -c "
  set -e
  rm -rf $HOME/lab_ws && mkdir -p $HOME/lab_ws/src && cd $HOME/lab_ws/src
  ros2 pkg create --build-type ament_python cobraflex_lab --dependencies rclpy geometry_msgs sensor_msgs std_msgs
  mkdir -p cobraflex_lab/launch cobraflex_lab/config
  cp $WS/lab/session1/circle_driver.py $WS/lab/session1/obstacle_monitor.py cobraflex_lab/cobraflex_lab/
  cp $WS/lab/session1/session1.launch.py cobraflex_lab/launch/
  cp $WS/lab/session1/session1.yaml cobraflex_lab/config/
  cd $HOME/lab_ws && colcon build --symlink-install"
check "lab session 1: open gap reported as NameError" bash -c \
  "cd $WS/lab/session1 && timeout 30 python3 circle_driver.py 2>&1 | tee /dev/stderr | grep -q \"NameError: name '___A___' is not defined\""

# ---- docs/2_USAGE.md §7: linters ---------------------------------------------------
check "colcon test --packages-select cobraflex" bash -c \
  "cd $WS && colcon test --packages-select cobraflex && colcon test-result --verbose"

echo
if [ "$FAILED" -ne 0 ]; then echo "Smoke test: FAILED"; else echo "Smoke test: all checks passed"; fi
exit "$FAILED"
