#!/usr/bin/env bash
# 启动 record_camera_rig_trajectory.py，并自动配置 Isaac Sim ROS2 bridge 所需环境变量。
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ISAAC_ROOT="$(realpath "$ROOT/app")"
ROS_DISTRO="${ROS_DISTRO:-humble}"
BRIDGE_EXT="${ISAAC_ROOT}/exts/isaacsim.ros2.bridge"
BRIDGE_LIB="${BRIDGE_EXT}/${ROS_DISTRO}/lib"
BRIDGE_RCLPY="${BRIDGE_EXT}/${ROS_DISTRO}/rclpy"

# 去掉终端里 source /opt/ros/... 注入的路径（Python 3.10 rclpy 与 Isaac 3.11 不兼容）
_strip_opt_ros_paths() {
  local var="$1" entry out=""
  IFS=':' read -ra _parts <<< "${!var:-}"
  for entry in "${_parts[@]}"; do
    [[ -z "$entry" || "$entry" == *"/opt/ros/"* ]] && continue
    out="${out:+$out:}$entry"
  done
  printf -v "$var" '%s' "$out"
}

export ROS_DISTRO
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
_strip_opt_ros_paths LD_LIBRARY_PATH
_strip_opt_ros_paths PYTHONPATH
if [[ -d "$BRIDGE_LIB" ]]; then
  export LD_LIBRARY_PATH="${BRIDGE_LIB}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
else
  echo "警告: ROS2 bridge 库目录不存在: $BRIDGE_LIB" >&2
fi
if [[ -d "$BRIDGE_RCLPY" ]]; then
  export PYTHONPATH="${BRIDGE_RCLPY}${PYTHONPATH:+:$PYTHONPATH}"
else
  echo "警告: ROS2 bridge rclpy 目录不存在: $BRIDGE_RCLPY" >&2
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

exec "$ROOT/app/python.sh" "$ROOT/tools/demo_data/record_camera_rig_trajectory.py" "$@"
