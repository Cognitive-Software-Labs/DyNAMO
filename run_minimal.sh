#!/bin/bash
set -e

# Remove Snap runtime libs that can break ROS Jazzy GUI binaries (rviz2 / gz sim).
unset LD_PRELOAD
unset SNAP SNAP_NAME SNAP_INSTANCE_NAME SNAP_ARCH SNAP_LIBRARY_PATH
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
	CLEAN_LD=""
	IFS=':' read -r -a LD_PARTS <<< "${LD_LIBRARY_PATH}"
	for p in "${LD_PARTS[@]}"; do
		[[ -z "${p}" ]] && continue
		[[ "${p}" == /snap/* ]] && continue
		CLEAN_LD="${CLEAN_LD:+${CLEAN_LD}:}${p}"
	done
	export LD_LIBRARY_PATH="${CLEAN_LD}"
fi

cd /home/pratham/Desktop/DyNAMO
source /opt/ros/jazzy/setup.bash

USE_RVIZ=${USE_RVIZ:-true}
USE_GZ_GUI=${USE_GZ_GUI:-true}
GZ_PARTITION="dynamo_${USER}_$$"

# Stop stale instances from previous runs to avoid multiple /clock publishers.
for proc in robot_state_publisher parameter_bridge odom_tf_pub async_slam_toolbox_node rviz2; do
	pkill -x "${proc}" 2>/dev/null || true
done

# Gazebo transport can leak messages from stale simulator instances.
pkill -x gz 2>/dev/null || true
pkill -f 'gz sim' 2>/dev/null || true

# Stop stale launch/create processes without matching this shell.
pkill -f 'ros2 launch dynamo_minimal_sim minimal_mapping_launch.py' 2>/dev/null || true
pkill -f '/ros_gz_sim/create' 2>/dev/null || true

colcon build --packages-select dynamo_minimal_sim --symlink-install
source install/setup.bash

# Re-clean after setup scripts in case they re-introduce Snap paths.
if [[ -n "${LD_LIBRARY_PATH:-}" ]]; then
	CLEAN_LD=""
	IFS=':' read -r -a LD_PARTS <<< "${LD_LIBRARY_PATH}"
	for p in "${LD_PARTS[@]}"; do
		[[ -z "${p}" ]] && continue
		[[ "${p}" == /snap/* ]] && continue
		CLEAN_LD="${CLEAN_LD:+${CLEAN_LD}:}${p}"
	done
	export LD_LIBRARY_PATH="${CLEAN_LD}"
fi

echo "Starting minimal mapping (use_gz_gui=${USE_GZ_GUI}, use_rviz=${USE_RVIZ})"
echo "Using isolated Gazebo partition: ${GZ_PARTITION}"

env -i \
	HOME="${HOME}" \
	USER="${USER}" \
	LOGNAME="${LOGNAME:-${USER}}" \
	SHELL="/bin/bash" \
	PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" \
	LANG="${LANG:-C.UTF-8}" \
	FASTDDS_BUILTIN_TRANSPORTS="${FASTDDS_BUILTIN_TRANSPORTS}" \
	GZ_PARTITION="${GZ_PARTITION}" \
	DISPLAY="${DISPLAY:-}" \
	WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" \
	XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" \
	XAUTHORITY="${XAUTHORITY:-${HOME}/.Xauthority}" \
	bash --noprofile --norc -lc "cd /home/pratham/Desktop/DyNAMO && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch dynamo_minimal_sim minimal_mapping_launch.py use_gz_gui:=${USE_GZ_GUI} use_rviz:=${USE_RVIZ}"
