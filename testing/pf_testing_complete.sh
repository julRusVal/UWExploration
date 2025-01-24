#!/bin/bash

# Define Arguments
namespace="hugin_0"

# Start a new tmux session in the background
tmux new-session -d -s mysession

# Run roscore in window 1
tmux new-window -t mysession:1
tmux send-keys -t mysession:1 "roscore" C-m
tmux rename-window -t mysession:1 "roscore"

# Add a delay in seconds to allow roscore to start
sleep 2

# Run RViz in window 2
tmux new-window -t mysession:2
# need to find the rviz file
# tmux send-keys -t mysession:2 "rviz -d /home/sam/auv_ws/src/UWExploration/localization/sss_particle_filter/sss_pf.rviz" C-m
tmux send-keys -t mysession:2 "rviz -d /home/julianvaldez/catkin_ws/src/UWExploration/testing/rviz/testing.rviz" C-m
tmux rename-window -t mysession:2 "rviz"

# Run the first roslaunch in window 3
tmux new-window -t mysession:3
tmux send-keys -t mysession:3 "roslaunch auv_model auv_env_aux.launch dataset:=asko namespace:=$namespace" C-m
tmux rename-window -t mysession:3 "auv_env_aux"

# Run the second roslaunch in window 4
tmux new-window -t mysession:4
tmux send-keys -t mysession:4 "roslaunch basic_navigation basic_mission.launch manual_control:=True namespace:=$namespace" C-m
tmux rename-window -t mysession:4 "basic_mission"

# Run the third roslaunch in window 5
tmux new-window -t mysession:5
tmux send-keys -t mysession:5 "roslaunch auv_model auv_environment.launch namespace:=$namespace mode:=sim dataset:=asko" C-m
tmux rename-window -t mysession:5 "auv_environment"

# Run the fourth roslaunch in window 6
tmux new-window -t mysession:6
tmux send-keys -t mysession:6 "roslaunch sss_particle_filter sss_pf.launch namespace:=$namespace particle_count:=4 mode:=sim results_path:=/home/sam/.ros" C-m
tmux rename-window -t mysession:6 "sss_pf"

# Run the view_sidescan.py script in window 7
tmux new-window -t mysession:7
tmux send-keys -t mysession:7 "rosrun auv_model view_sidescan.py" C-m
tmux rename-window -t mysession:7 "view_sidescan"

# Attach to the tmux session
tmux attach -t mysession
