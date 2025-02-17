#!/usr/bin/env bash

# This is a tmux script that starts a multi-agent simulation
# It is based on mutli_agent.launch
# The purpose is to allow for easy debugging and monitoring of the simulation

# Note: Not sure if this is bad form

SESSION="multi_agent_session"
# Hopefully this is the correct path
ACTIVE_WS=$(echo $CMAKE_PREFIX_PATH | cut -d':' -f1 | sed 's|/devel||')
PARAM_FILE="$ACTIVE_WS/src/UWExploration/testing/multi_agent_params.yaml"
RVIZ_FILE="$ACTIVE_WS/src/UWExploration/testing/rviz/basic_config_v2.rviz"

# 1. Create a new tmux session (detached)
tmux new-session -d -s $SESSION

# -----------------------------------------------------------------------------
# Window 0: roscore
# -----------------------------------------------------------------------------
tmux rename-window -t $SESSION:0 'roscore'
tmux send-keys -t $SESSION:0 "roscore" C-m

# Give roscore a moment to initialize
sleep 2

# -----------------------------------------------------------------------------
# Window 1: Load parameters from YAML
# -----------------------------------------------------------------------------
tmux new-window -t $SESSION -n 'load_params'
tmux send-keys -t $SESSION:1 "
echo 'Clearing parameters...'
rosparam delete \/
echo 'Loading parameters from $PARAM_FILE ...'
rosparam load $PARAM_FILE
rosparam get \/
" C-m

# Optional small delay so params are definitely on the server
sleep 3

# -----------------------------------------------------------------------------
# Window 2: RVIZ
# -----------------------------------------------------------------------------
tmux new-window -t $SESSION -n 'RVIZ'
tmux send-keys -t $SESSION:2 "
echo 'Starting RVIZ using config: $RVIZ_FILE'
rviz -d $RVIZ_FILE
" C-m

# -----------------------------------------------------------------------------
# Window 2: AUV Spawner
# Spawner node is responsible for spawning AUVs in the simulation
# Parameters are loaded from the YAML file
# -----------------------------------------------------------------------------
tmux new-window -t $SESSION -n 'spawner'
tmux send-keys -t $SESSION:3 "
echo 'Starting AUV spawner...'

SPAWNER_NAME=\$(rosparam get node_name_spawner)
MODE=\$(rosparam get mode)
DATASET=\$(rosparam get dataset)
AUV_LAUNCH_FILE=\$(rospack find auv_model)/launch/auv_environment.launch
FLS_HORIZONTAL=\$(rosparam get fls_horizontal_angle)
FLS_VERTICAL=\$(rosparam get fls_vertical_angle)
FLS_RANGE=\$(rosparam get fls_max_range)
ODOM_PERIOD=\$(rosparam get odom_period)
MBES_PERIOD=\$(rosparam get mbes_meas_period)

rosrun multi_agent auv_spawner.py \\
  __name:=\$SPAWNER_NAME \\
  _mode:=\$MODE \\
  _dataset:=\$DATASET \\
  _auv_launch_file:=\$AUV_LAUNCH_FILE \\
  _fls_horizontal_angle:=\$FLS_HORIZONTAL \\
  _fls_vertical_angle:=\$FLS_VERTICAL \\
  _fls_max_range:=\$FLS_RANGE \\
  _fls_range_std:=0.0 \\
  _fls_angle_std:=0.0 \\
  _fls_meas_period:=0.1 \\
  _odom_period:=\$ODOM_PERIOD \\
  _mbes_meas_period:=\$MBES_PERIOD
" C-m

# -----------------------------------------------------------------------------
# Window 3: AUV Navigation
# -----------------------------------------------------------------------------
tmux new-window -t $SESSION -n 'navigation'
tmux send-keys -t $SESSION:4 "
echo 'Starting AUV navigation...'

NAV_NAME=\$(rosparam get node_name_navigation)
NAV_LAUNCH_FILE=\$(rospack find basic_navigation)/launch/basic_mission.launch
MANUAL_CONTROL=\$(rosparam get manual_control)
MAX_THRUST=\$(rosparam get max_thrust)
MAX_THROTTLE=\$(rosparam get max_throttle)
WP_TYPE=\$(rosparam get waypoint_follower_type)
DUBINS_STEP=\$(rosparam get dubins_step_size)
GOAL_TOL=\$(rosparam get goal_tolerance)
TIME_SYNC=\$(rosparam get time_sync)
ODOM_PERIOD=\$(rosparam get odom_period)

rosrun multi_agent auv_navigation.py \\
  __name:=\$NAV_NAME \\
  _navigation_launch_file:=\"\$NAV_LAUNCH_FILE\" \\
  _manual_control:=\$MANUAL_CONTROL \\
  _max_thrust:=\$MAX_THRUST \\
  _max_throttle:=\$MAX_THROTTLE \\
  _waypoint_follower_type:=\$WP_TYPE \\
  _dubins_step_size:=\$DUBINS_STEP \\
  _goal_tolerance:=\$GOAL_TOL \\
  _time_sync:=\$TIME_SYNC \\
  _odom_period:=\$ODOM_PERIOD
" C-m

# -----------------------------------------------------------------------------
# Window 4: Path Relay
# -----------------------------------------------------------------------------
tmux new-window -t $SESSION -n 'path_relay'
tmux send-keys -t $SESSION:5 "
echo 'Starting path_relay node...'
RELAY_NAME=\$(rosparam get node_name_relay)

rosrun multi_agent path_relay.py \\
  __name:=\$RELAY_NAME
" C-m

# -----------------------------------------------------------------------------
# Window 6: Pattern Generator
# rviz_helper and pattern_generation are mutually exclusive
# -----------------------------------------------------------------------------
RVIZ_HELPER=$(rosparam get rviz_helper)
PATTERN_GEN=$(rosparam get pattern_generation)
tmux new-window -t $SESSION -n 'pattern_generator'
if [ "$RVIZ_HELPER" = "true" ]; then
  tmux send-keys -t $SESSION:6 "
  echo 'rviz_helper is enabled.'

  HELPER_NAME=\$(rosparam get node_name_helper)

  rosrun multi_agent rviz_wp_helper.py
    __name:=\$HELPER_NAME
  " C-m

elif [ "$PATTERN_GEN" = "true" ]; then
  tmux send-keys -t $SESSION:6 "
  echo 'Starting path_pattern_generator...'

  PATTERN_NAME=\$(rosparam get node_name_pattern)
  SWATH=\$(rosparam get swath)
  SPEED=\$(rosparam get speed)
  STRAIGHT_SLACK=\$(rosparam get straight_slack)
  ROWS_OVERLAP=\$(rosparam get overlap_between_rows)
  LANES_OVERLAP=\$(rosparam get overlap_between_lanes)
  DOUBLE_SIDED=\$(rosparam get double_sided)
  CENTER_X=\$(rosparam get center_x)
  CENTER_Y=\$(rosparam get center_y)
  EXIT_LINE=\$(rosparam get exiting_line)
  SURVEY_AREA=\$(rosparam get survey_area_topic)

  rosrun multi_agent path_pattern_generator.py \\
    __name:=\$PATTERN_NAME \\
    _swath:=\$SWATH \\
    _speed:=\$SPEED \\
    _straight_slack:=\$STRAIGHT_SLACK \\
    _overlap_between_rows:=\$ROWS_OVERLAP \\
    _overlap_between_lanes:=\$LANES_OVERLAP \\
    _double_sided:=\$DOUBLE_SIDED \\
    _center_x:=\$CENTER_X \\
    _center_y:=\$CENTER_Y \\
    _exiting_line:=\$EXIT_LINE \\
    _survey_area_topic:=\$SURVEY_AREA
  " C-m
else
  tmux send-keys -t $SESSION:6 "
  echo 'Neither rviz_helper nor pattern_generation is enabled.'
  " C-m

fi

# -----------------------------------------------------------------------------
# Window 6: Rviz Display Service
# -----------------------------------------------------------------------------
tmux new-window -t $SESSION -n 'rviz_display'
tmux send-keys -t $SESSION:7 "
echo 'Starting display_message_service...'
rosrun rviz_visualization display_message_service.py
" C-m

# -----------------------------------------------------------------------------
# Window 8: Plot Generator
# -----------------------------------------------------------------------------
tmux new-window -t $SESSION -n 'plot_generator'
tmux send-keys -t $SESSION:8 "
echo 'Starting plot_generator_service...'

ANIMATE=\$(rosparam get animate_plots)
SAVE_FINAL=\$(rosparam get save_final_plots)
RESULTS_PATH=\$(rosparam get plots_results_path)
VEHICLE_MODEL=\$(rosparam get vehicle_model)
RECORD_ARGS=\$(rosparam get record_launch_parameters_and_arguments)

rosrun plot_generator plot_generator_service.py \\
  _animate_plots:=\$ANIMATE \\
  _save_final_plots:=\$SAVE_FINAL \\
  _results_path:=\"\$RESULTS_PATH\" \\
  _vehicle_model:=\"\$VEHICLE_MODEL\" \\
  _record_launch_parameters_and_arguments:=\$RECORD_ARGS
" C-m

# -----------------------------------------------------------------------------
#Window 9: Auxiliary Nodes
# Conditional: If auxiliary_enabled == true, launch auxiliary nodes
# -----------------------------------------------------------------------------
# I was having issues with the parameter being read correctly
# For now it is hardcoded to run the auv_env_aux.launch file

AUX_ENABLED_VALUE=$(rosparam get auxiliary_enabled)
tmux new-window -t $SESSION -n 'auxiliary'
if [ "$AUX_ENABLED_VALUE" = "true" ]; then
  tmux send-keys -t $SESSION:9 "
  echo \$(rosparam get auxiliary_enabled)
  echo 'auxiliary_enabled is true -- Launching auxiliary nodes...'

  AUX_LAUNCH_FILE=\$(rospack find auv_model)/launch/auv_env_aux.launch

 roslaunch auv_model auv_env_aux.launch \\
  mode:=\$(rosparam get mode) \\
  dataset:=\$(rosparam get dataset) \\
  num_auvs:=\$(rosparam get num_auvs) \\
  namespace:=\$(rosparam get namespace)
  " C-m
else

  tmux send-keys -t $SESSION:9 "
  echo \$(rosparam get auxiliary_enabled)
  echo 'auxiliary_enabled is false -- Skipping auxiliary nodes...'
  " C-m
fi

# tmux send-keys -t $SESSION:9 "
# echo 'auxiliary_enabled is true -- Launching auxiliary nodes...'

# AUX_LAUNCH_FILE=\$(rospack find auv_model)/launch/auv_env_aux.launch

# roslaunch auv_model auv_env_aux.launch \\
# mode:=\$(rosparam get mode) \\
# dataset:=\$(rosparam get dataset) \\
# num_auvs:=\$(rosparam get num_auvs) \\
# namespace:=\$(rosparam get namespace)
# " C-m

# -----------------------------------------------------------------------------
# Conditional: If rbpf == true, launch RBPF node
# -----------------------------------------------------------------------------
# RBPF_VALUE=$(rosparam get rbpf)
# tmux new-window -t $SESSION -n 'rbpf_multi'
# if [ "$RBPF_VALUE" = "true" ]; then
#   tmux send-keys -t $SESSION:10 "
#   echo 'rbpf is true -- Starting RBPF multi-agent...'

#   RBPF_NAME=\$(rosparam get node_name_rbpf_multi)
#   PARTICLE_COUNT=\$(rosparam get particle_count)
#   PARTICLE_COUNT_NEIGH=\$(rosparam get particle_count_neighbours)
#   NUM_PARTICLE_HANDLERS=\$(rosparam get num_particle_handlers)
#   RESULTS_PATH=\$(rosparam get results_path)
#   RBPF_SENSOR_FLS=\$(rosparam get rbpf_sensor_FLS)
#   RBPF_SENSOR_MBES=\$(rosparam get rbpf_sensor_MBES)
#   COMMS_TYPE=\$(rosparam get comms_type)
#   INIT_COV=\$(rosparam get init_covariance)
#   MOTION_COV=\$(rosparam get motion_covariance)
#   RESAMPLING_COV=\$(rosparam get resampling_noise_covariance)
#   FLS_RANGE_STD=\$(rosparam get fls_range_std)
#   FLS_ANGLE_STD=\$(rosparam get fls_angle_std)
#   PARTICLE_SPREAD=\$(rosparam get particle_spread_std_factor)
#   WEIGHT_SLICING=\$(rosparam get weight_slicing)
#   PMP=\$(rosparam get pmp)
#   NUM_AUVS=\$(rosparam get num_auvs)
#   MAX_THROTTLE=\$(rosparam get max_throttle)
#   SURVEY_AREA=\$(rosparam get survey_area_topic)
#   MODE=\$(rosparam get mode)

#   rosrun rbpf_multiagent rbpf_setup_4_all_agents.py \\
#     __name:=\$RBPF_NAME \\
#     _particle_count:=\$PARTICLE_COUNT \\
#     _particle_count_neighbours:=\$PARTICLE_COUNT_NEIGH \\
#     _num_particle_handlers:=\$NUM_PARTICLE_HANDLERS \\
#     _results_path:=\"\$RESULTS_PATH\" \\
#     _mode:=\$MODE \\
#     _rbpf_sensor_FLS:=\$RBPF_SENSOR_FLS \\
#     _rbpf_sensor_MBES:=\$RBPF_SENSOR_MBES \\
#     _comms_type:=\$COMMS_TYPE \\
#     _init_covariance:=\"\$INIT_COV\" \\
#     _motion_covariance:=\"\$MOTION_COV\" \\
#     _resampling_noise_covariance:=\"\$RESAMPLING_COV\" \\
#     _fls_range_std:=\$FLS_RANGE_STD \\
#     _fls_angle_std:=\$FLS_ANGLE_STD \\
#     _particle_spread_std_factor:=\$PARTICLE_SPREAD \\
#     _weight_slicing:=\"\$WEIGHT_SLICING\" \\
#     _pmp:=\"\$PMP\" \\
#     _num_auvs:=\$NUM_AUVS \\
#     _max_throttle:=\$MAX_THROTTLE \\
#     _survey_area_topic:=\$SURVEY_AREA
#   " C-m
# else
#   tmux send-keys -t $SESSION:10 "
#   echo 'rbpf is false -- Skipping RBPF multi-agent...'
#   " C-m
# fi

# -----------------------------------------------------------------------------
# Finally, attach to the tmux session
# -----------------------------------------------------------------------------
tmux attach-session -t $SESSION
