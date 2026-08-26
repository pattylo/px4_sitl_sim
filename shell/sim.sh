#!/usr/bin/env bash

set -euo pipefail

readonly SESSION_NAME="multi_sim"
readonly FORMATION_SPACING_METERS=2
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly SCRIPT_DIR
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
readonly REPO_DIR
readonly PX4_DIR="${REPO_DIR}/PX4-Autopilot"
readonly PX4_BINARY="${PX4_DIR}/build/px4_sitl_default/bin/px4"
readonly ROS2_WS_DIR="${REPO_DIR}/ros2_ws"
readonly WORKSPACE_SETUP="${ROS2_WS_DIR}/install/setup.bash"
readonly INSTALLED_PX4_LAUNCH="${ROS2_WS_DIR}/install/px4_sitl_sim/share/px4_sitl_sim/launch/px4.launch"

usage() {
	printf 'Usage: %s <vehicle-count> <namespace-prefix>\n' "${0##*/}"
	printf 'Example: %s 2 uav    # creates uav0 and uav1\n' "${0##*/}"
}

if (( $# != 2 )); then
	usage >&2
	exit 2
fi

readonly VEHICLE_COUNT="$1"
NAMESPACE_PREFIX="${2#/}"
NAMESPACE_PREFIX="${NAMESPACE_PREFIX%/}"
readonly NAMESPACE_PREFIX

# PX4 assigns distinct offboard receive ports only to instances 0 through 9.
if [[ ! "${VEHICLE_COUNT}" =~ ^[1-9][0-9]*$ ]] || (( VEHICLE_COUNT > 10 )); then
	printf 'Error: vehicle-count must be an integer from 1 to 10.\n' >&2
	exit 2
fi

if [[ -z "${NAMESPACE_PREFIX}" ]]; then
	printf 'Error: namespace-prefix must not be empty.\n' >&2
	exit 2
fi

# Use the smallest square-enclosing grid. Every occupied neighbor is 2 m away.
GRID_COLUMNS=1
while (( GRID_COLUMNS * GRID_COLUMNS < VEHICLE_COUNT )); do
	(( GRID_COLUMNS += 1 ))
done
readonly GRID_COLUMNS

declare -a SPAWN_POSES=()
declare -a INITIAL_GAZEBO_POSITIONS=()
for (( instance = 0; instance < VEHICLE_COUNT; instance++ )); do
	grid_column=$(( instance % GRID_COLUMNS ))
	grid_row=$(( instance / GRID_COLUMNS ))
	spawn_x=$(( FORMATION_SPACING_METERS * grid_column ))
	spawn_y=$(( FORMATION_SPACING_METERS * grid_row ))
	SPAWN_POSES[instance]="${spawn_x},${spawn_y},0.3,0,0,0"
	INITIAL_GAZEBO_POSITIONS+=("${spawn_x}.0" "${spawn_y}.0" '0.3')
done

printf -v initial_positions_csv '%s,' "${INITIAL_GAZEBO_POSITIONS[@]}"
readonly INITIAL_POSITIONS_PARAMETER="[${initial_positions_csv%,}]"

if [[ ! -d "${PX4_DIR}" ]]; then
	printf 'Error: PX4 checkout not found at %s\n' "${PX4_DIR}" >&2
	exit 1
fi

if [[ ! -f "${WORKSPACE_SETUP}" || ! -f "${INSTALLED_PX4_LAUNCH}" ]]; then
	printf 'Error: px4_sitl_sim is not built in %s\n' "${ROS2_WS_DIR}" >&2
	printf 'Run: cd %s && colcon build --symlink-install\n' "${ROS2_WS_DIR}" >&2
	exit 1
fi

# Stop only processes owned by the previous simulation.
tmux kill-session -t "${SESSION_NAME}" 2>/dev/null || true
pkill -x rviz2 2>/dev/null || true
pkill -x px4 2>/dev/null || true
pkill -x mavros_node 2>/dev/null || true
pkill -f 'gz sim' 2>/dev/null || true

sleep 2

# Instance 0 starts Gazebo from the PX4 checkout in this repository.
tmux new-session -d -s "${SESSION_NAME}" -n px4_0
for display_variable in DISPLAY WAYLAND_DISPLAY XAUTHORITY XDG_RUNTIME_DIR; do
	if [[ -n "${!display_variable:-}" ]]; then
		tmux set-environment -t "${SESSION_NAME}" "${display_variable}" "${!display_variable}"
	fi
done

printf -v px4_command \
	'cd %q && PX4_GZ_MODEL_POSE=%q make px4_sitl gz_x500' \
	"${PX4_DIR}" "${SPAWN_POSES[0]}"
tmux send-keys -t "${SESSION_NAME}:px4_0" "${px4_command}" C-m

# Remaining PX4 instances wait for the first build, then join its Gazebo world.
for (( instance = 1; instance < VEHICLE_COUNT; instance++ )); do
	tmux new-window -t "${SESSION_NAME}" -n "px4_${instance}"
	printf -v px4_command \
		'cd %q && until [[ -x %q ]]; do sleep 1; done; sleep 5; PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=%q PX4_SIM_MODEL=gz_x500 %q -i %d' \
		"${PX4_DIR}" "${PX4_BINARY}" "${SPAWN_POSES[instance]}" "${PX4_BINARY}" "${instance}"
	tmux send-keys -t "${SESSION_NAME}:px4_${instance}" "${px4_command}" C-m
done

# Each MAVROS process gets a unique ROS namespace, UDP pair, and PX4 system ID.
printf '%-10s %-28s %-12s %-12s %-12s\n' \
	'INSTANCE' 'NAMESPACE' 'GAZEBO XYZ' 'FCU UDP' 'TARGET SYS'
for (( instance = 0; instance < VEHICLE_COUNT; instance++ )); do
	namespace="${NAMESPACE_PREFIX}${instance}/mavros"
	mavros_bind_port=$(( 14540 + instance ))
	px4_bind_port=$(( 14580 + instance ))
	target_system=$(( instance + 1 ))
	fcu_url="udp://:${mavros_bind_port}@127.0.0.1:${px4_bind_port}"

	tmux new-window -t "${SESSION_NAME}" -n "mavros_${instance}"
	printf -v mavros_command \
		'. %q && ros2 launch px4_sitl_sim px4.launch fcu_url:=%q tgt_system:=%d namespace:=%q' \
		"${WORKSPACE_SETUP}" "${fcu_url}" "${target_system}" "${namespace}"
	tmux send-keys -t "${SESSION_NAME}:mavros_${instance}" "${mavros_command}" C-m

	printf '%-10d %-28s %-12s %-12s %-12d\n' \
		"${instance}" "${namespace}" "${SPAWN_POSES[instance]%%,0,0,0}" \
		"${mavros_bind_port}:${px4_bind_port}" "${target_system}"
done

# Relay all MAVROS poses into the local ENU frame whose origin is agent 0.
tmux new-window -t "${SESSION_NAME}" -n frame_sync
printf -v frame_sync_command \
	'. %q && ros2 run px4_sitl_sim swarm_frame_sync --ros-args -p vehicle_count:=%d -p namespace_prefix:=%q -p initial_gazebo_positions:=%q' \
	"${WORKSPACE_SETUP}" "${VEHICLE_COUNT}" "${NAMESPACE_PREFIX}" "${INITIAL_POSITIONS_PARAMETER}"
tmux send-keys -t "${SESSION_NAME}:frame_sync" "${frame_sync_command}" C-m

tmux select-window -t "${SESSION_NAME}:px4_0"
if [[ "${NO_ATTACH:-0}" != "1" ]]; then
	tmux attach-session -t "${SESSION_NAME}"
fi
