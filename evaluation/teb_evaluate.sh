#!/bin/bash

# Workspace path
WS=~/pedsim_ws
SESSION=pedsim

# Bắt tín hiệu Ctrl+C (SIGINT) và kill toàn bộ tmux session
trap "echo 'Ctrl+C detected! Killing session $SESSION ...'; tmux kill-session -t $SESSION 2>/dev/null; exit" INT

# Kill old session if exists
tmux kill-session -t $SESSION 2>/dev/null

# Step 1: Start Gazebo immediately
echo "Step 1: Launching Gazebo simulation..."
tmux new-session -d -s $SESSION "bash -ic 'cd $WS && source install/setup.bash && ros2 launch pedsim_gazebo_plugin robot_test_launch.py'"

# Step 2: After 8s, start KF and ellipse nodes
sleep 10
echo "Step 2: Starting obs_kf and obstacle_ellipses_node..."
tmux split-window -v -t $SESSION "bash -ic 'cd $WS && source install/setup.bash && ros2 run obs_param obs_kf'"
tmux split-window -h -t $SESSION "bash -ic 'cd $WS && source install/setup.bash && ros2 run data_processor obstacle_ellipses_node'"

# Step 3: After 10s total, start Navigation2
sleep 2
echo "Step 3: Launching Navigation2..."
tmux split-window -v -t $SESSION "bash -ic 'cd $WS && source install/setup.bash && ros2 launch rur_navigation2 navigation2_teb.launch.py'"

# Step 4: Start metrics logger
sleep 7
echo "Step 4: Starting navigation_metrics_logger_node..."
tmux split-window -h -t $SESSION "bash -ic 'cd $WS && source install/setup.bash && ros2 run data_processor navigation_metrics_logger_node'"

# Arrange windows
tmux select-layout -t $SESSION tiled

# Attach session
tmux attach -t $SESSION

