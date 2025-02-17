#!/bin/bash

# Define Arguments
namespace="hugin_0"
dataset="asko"
auv_count="2"

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
tmux send-keys -t mysession:2 "rviz -d /home/sam/auv_ws/src/UWExploration/testing/rviz/basic_config_v2.rviz" C-m
tmux rename-window -t mysession:2 "rviz"

# Run the environment in window 3
tmux new-window -t mysession:3
tmux send-keys -t mysession:3 "roslaunch auv_model auv_env_aux.launch dataset:=$dataset namespace:=$namespace" C-m
tmux rename-window -t mysession:3 "auv_env_aux"

# Run the second roslaunch in window 4
tmux new-window -t mysession:4
tmux send-keys -t mysession:4 "roslaunch multi_agent multi_agent.launch num_auvs:=$auv_count dataset:=$dataset rbpf:=false" C-m
tmux rename-window -t mysession:4 "Multi_agent_mission"

# Run the third roslaunch in window 5
#tmux new-window -t mysession:5
#tmux send-keys -t mysession:5 "roslaunch auv_model auv_environment.launch namespace:=$namespace mode:=sim dataset:=$dataset" C-m
#tmux rename-window -t mysession:5 "auv_environment"


# Attach to the tmux session
tmux attach -t mysession
