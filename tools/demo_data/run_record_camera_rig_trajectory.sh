#!/usr/bin/env bash
# 启动 record_camera_rig_trajectory.py，并自动配置 Isaac Sim ROS2 bridge 所需环境变量。
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
ISAAC_ROOT="$(realpath "$ROOT/app")"
BRIDGE_LIB="${ISAAC_ROOT}/exts/isaacsim.ros2.bridge/${ROS_DISTRO:-humble}/lib"

export ROS_DISTRO="${ROS_DISTRO:-humble}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
if [[ -d "$BRIDGE_LIB" ]]; then
  export LD_LIBRARY_PATH="${BRIDGE_LIB}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
else
  echo "警告: ROS2 bridge 库目录不存在: $BRIDGE_LIB" >&2
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

exec "$ROOT/app/python.sh" "$ROOT/tools/demo_data/record_camera_rig_trajectory.py" "$@"
