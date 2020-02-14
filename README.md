# AUV Localization

Particle filter for AUV localization based on MBES measurements.

## Dependencies (tested on Ubuntu 16.04 and 18.04)
* AUVLIB [here](https://github.com/nilsbore/auvlib) 
* Bathymetric SLAM [in the private gitr] (https://gitr.sys.kth.se/torroba/bathymetric_slam)

## Building

This is a collection of ROS packages. Just clone the repo within your catking workspace and run
```
catkin_make install
```
If you've managed to install the dependencies, this part should be pretty easy.

## ROS Packages

### auv_2_ros
It will contain all the utilities to interface the AUV with the ROS environment.
For now we'll be doing this via parsing of `.all` files from Hugin which contain, among other things, the MBES pings and AUV trajectories we need.
There are two "nodes" for now:

#### read_auv_data
This is not a node and doesn't need to be. It's just an app for you to parse `.all` files into `.cereal` files, which are lighter and faster to read, use:
```
rosrun auv_2_ros read_auv_data --folder /path/to/folder/with.allFiles --type all
```

#### auv_2_ros
It will provide the functionality to parse the cereal files and publish into the ROS environment the information we need from them (map, control input, ground truth vehicle estimate...).
```
rosrun auv_2_ros auv_2_ros --map /your/path/to/map.cereal --trajectory /your/path/to/trajectory.cereal
```
This node takes care of making available in ROS all the AUV trajectory and bathymetric data from Hugin.
Topics and such so far:
* `/gt/odom`: ground truth AUV odometry
* `/gt/mbes_pings`: ground truth MBES ping at current AUV pose
* `/map`: ground truth bathymetry from the AUV survey
* `/sim/mbes`: simulated MBES ping at current AUV pose


