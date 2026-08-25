#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
WORKSPACE_SETUP="${ROS2_WS_DIR}/install/setup.bash"

pkill -f rviz2 2>/dev/null || true
pkill -f px4 2>/dev/null || true
pkill -f gz 2>/dev/null || true
pkill -f mavros 2>/dev/null || true
tmux kill-session -t multi_sim 2>/dev/null || true

sleep 2

# launch gazebo
tmux new-session -d -s multi_sim -n gazebo
tmux set-environment -t multi_sim DISPLAY "${DISPLAY:-}"
tmux set-environment -t multi_sim WAYLAND_DISPLAY "${WAYLAND_DISPLAY:-}"
tmux set-environment -t multi_sim XAUTHORITY "${XAUTHORITY:-$HOME/.Xauthority}"
tmux send-keys -t multi_sim:0 'cd ~/PX4-Autopilot && PX4_GZ_MODEL_POSE="0,0,0.3,0,0,0.0" make px4_sitl gz_x500' C-m

# follower PX4 starts 4 m behind the leader, matching the real-world setup.
sleep 4
tmux split-window -h -t multi_sim:0
tmux send-keys -t multi_sim:0.1 'cd ~/PX4-Autopilot && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE="-2,0,0.3,0,0,0.0" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1' C-m

# launch mavros
sleep 4
tmux new-window -t multi_sim:1 -n mavros
tmux send-keys -t multi_sim:1 "source ${WORKSPACE_SETUP} && ros2 launch scout uav_leader_gazebo.launch" C-m
tmux split-window -h -t multi_sim:1
tmux send-keys -t multi_sim:1.1 "source ${WORKSPACE_SETUP} && ros2 launch scout uav_follower_gazebo.launch tgt_system:=2" C-m

# launch replay controller and RViz visualizer
sleep 4
tmux new-window -t multi_sim:2 -n replay_viz
tmux send-keys -t multi_sim:2 "source ${WORKSPACE_SETUP} && ros2 launch scout replay_with_viz.launch.py" C-m

# attach to it
tmux attach -t multi_sim:0
