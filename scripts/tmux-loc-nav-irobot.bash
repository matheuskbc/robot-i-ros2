#!/bin/bash

tmux new-session -d

# Sourcing tmux scripts
tmux send-keys -t 0 '. install/setup.bash' C-m
tmux send-keys -t 0 'clear' C-m

# Launch files
sleep 1
tmux send-keys -t 0 'ros2 launch roomba_navigation roomba_robot_localization.py' C-m

# sleep 1
# tmux send-keys -t 2 'roslaunch roomba_bringup roomba_imu.launch' C-m

# sleep 1
# tmux send-keys -t 3 'roslaunch roomba_bringup roomba_lidar.launch' C-m

# sleep 1
# tmux send-keys -t 4 'roslaunch roomba_bringup roomba_rviz.launch' C-m

# sleep 1
# tmux send-keys -t 5 'roslaunch roomba_bringup roomba_transforms.launch' C-m

sleep 1
tmux -2 attach-session 
