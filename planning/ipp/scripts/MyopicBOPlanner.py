#!/usr/bin/env python3

# Standard tools
import numpy as np
import matplotlib.pyplot as plt

import open3d as o3d
import torch
import numpy as np

# BOtorch
# from botorch.fit import fit_gpytorch_mll
import botorch
import dubins
import pickle

# ROS imports
import rospy

# Custom imports
from gp_mapping.svgp_map import SVGP_map
import BayesianOptimizerClass
# import BayesianPlannerClass
import ipp_utils

# Python tools
import warnings
import os
import time
import tf
import pathlib
import ast
import copy
from collections import OrderedDict
from threading import Lock

import tf
import tf_conversions
import geometry_msgs
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import std_msgs.msg
import tf.transformations
from sensor_msgs.msg import PointCloud2

class MyopicBOPlanner():
    
    def __init__(self, corner_topic, path_topic, planner_req_topic, odom_topic, bounds, turning_radius, 
                 training_rate, wp_resolution, swath_width, path_nbr_samples, voxel_size, 
                 wp_sample_interval, horizon_distance, border_margin, beta):
        
        
        # Logic checks
        assert border_margin > 0,       "Planner safety/border margin must be positive"
        assert horizon_distance > 0,    "Planner safety horizon must be positive"   
        assert voxel_size > 0,          "Planner voxel size must be positive" 
        
        # Planner variables
        self.state              = []
        self.planner_initial_pose   = []
        self.wp_list                = []
        self.currently_planning     = False
        self.nbr_wp                 = 0
        self.max_travel_distance    = rospy.get_param("~max_travel_distance")

        # Path publisher - publishes waypoints for AUV to follow
        self.path_pub = rospy.Publisher(path_topic, Path, queue_size=100)
        # rospy.sleep(1)  # Give time for topic to be registered
        
        # Filelocks for file mutexes
        # self.gp_env_lock            = filelock.FileLock("GP_env.pickle.lock")
        
        # Setup GP for storage
        self.gp          = SVGP_map(int(0))
        self.beams       = np.empty((0, 3))
        
        # Parameters for optimizer
        self.wp_resolution          = wp_resolution
        self.swath_width            = swath_width
        self.path_nbr_samples       = path_nbr_samples
        self.voxel_size             = voxel_size
        self.wp_sample_interval     = wp_sample_interval
        self.horizon_distance       = horizon_distance
        self.border_margin          = border_margin
        self.beta                   = beta
        self.bounds             = bounds
        self.turning_radius     = turning_radius
        self.distance_travelled = 0

        # Publish corners locations to produce IPs 
        self.corner_pub  = rospy.Publisher(corner_topic, PointCloud2, queue_size=1, latch=True)
        corners = ipp_utils.generate_ip_corners(self.bounds)
        self.corner_pub.publish(corners)

        self.map_frame          = rospy.get_param("~map_frame")
        self.odom_frame         = rospy.get_param("~odom_frame")
        self.tf_listener        = tf.TransformListener()

        self.odom_init = False
        self.odom_topic = rospy.get_param("~odom_topic")
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_update_cb)

        # Publish an initial path
        # initial_path = self.initial_random_path(1)
        initial_path = self.initial_deterministic_path()
        self.path_pub.publish(initial_path) 

        # Setup timers for callbacks
        self.finish_imminent = False
        self.bo_planning = False
        # rospy.Timer(rospy.Duration(2), self.periodic_call)
        # self.execute_planner_pub    = rospy.Publisher("execute_planning_topic_handle", std_msgs.msg.Bool, queue_size=1)

        bo_replan_topic = rospy.get_param("~bo_replan_topic")
        self.execute_planner_sub    = rospy.Subscriber(bo_replan_topic, std_msgs.msg.Bool, self.execute_planning)
        
        # Initiate training of GP
        r = rospy.Rate(1)
        while not rospy.is_shutdown():

            # If planning requested
            if self.bo_planning:
                self.bo_plan()
                self.bo_planning = False

            # GP training 
            self.gp.train_iteration()  
            r.sleep()
    
    def odom_update_cb(self, msg):
        """ Gets our current 2D state (x,y,theta) from the tf frames.
            Also calculates the total distance travelled.

        Args:
            msg (PoseWithCovarianceStamped): A pose message (assumes msg type from AUV messages)
        """

        p = PoseStamped(header=msg.header, pose=msg.pose.pose)
        p.header.stamp = msg.header.stamp
        try:
            self.tf_listener.waitForTransform(self.map_frame, p.header.frame_id, rospy.Time(0), timeout=rospy.Duration(10.)) # 0.1s
            p_in_map = self.tf_listener.transformPose(self.map_frame, p)
            explicit_quat = [p_in_map.pose.orientation.x, p_in_map.pose.orientation.y, p_in_map.pose.orientation.z, p_in_map.pose.orientation.w]
            _, _, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
            if len(self.state) > 0:
                self.distance_travelled += np.hypot(p_in_map.pose.position.x - self.state[0], p_in_map.pose.position.y - self.state[1])
            self.state = [p_in_map.pose.position.x, p_in_map.pose.position.y, yaw]
            
            if self.odom_init == False:
                self.planner_initial_pose   = copy.deepcopy(self.state)
            
            self.odom_init = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("Couldn't get transform")



        # self.state_history.append(self.state)
        #print(self.state_history)


    def initial_deterministic_path(self):
        """ Generates a path to the center of the map, as defined by the boundaries.

        Returns:
            nav_msgs.msg.Path: ROS message with waypoint poses in list
        """
        
        while not self.odom_init and not rospy.is_shutdown():
            print("Planner is waiting for odometry before starting.")
            rospy.sleep(2)

        # Generate point in center of bounds
        x_pos = self.bounds[0] + (self.bounds[2] - self.bounds[0])/2
        y_pos = self.bounds[1] + (self.bounds[3] - self.bounds[1])/2
        samples = np.random.uniform(low=[x_pos - 1, y_pos - 1, -np.pi], high=[x_pos + 1, y_pos + 1, np.pi], size=[1, 3])
        h = std_msgs.msg.Header()
        h.frame_id = self.map_frame
        h.stamp = rospy.Time.now()
        sampling_path = Path()
        sampling_path.header = h
        for sample in samples:
            path = dubins.shortest_path(self.planner_initial_pose, [sample[0], sample[1], sample[2]], self.turning_radius)
            wp_poses, _ = path.sample_many(self.wp_resolution)

            skip = 1
            if len(wp_poses) == 1:
                skip = 0
            self.wp_list.extend(wp_poses[skip:])
            for pose in wp_poses[skip:]:
                wp = PoseStamped()
                wp.header = h
                wp.pose.position.x = pose[0]
                wp.pose.position.y = pose[1]
                wp.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, pose[2]))
                sampling_path.poses.append(wp)
                self.nbr_wp += 1
            self.planner_initial_pose = wp_poses[-1]
        return sampling_path
    
        
    def initial_random_path(self, nbr_samples = 1):
        """ Generates a set of random waypoints for initial sampling of BO

        Args:
            nbr_samples (int): number of samples. Defaults to 1.

        Returns:
            nav_msgs.msg.Path: ROS message with waypoint poses in list
        """

        while not self.odom_init and not rospy.is_shutdown():
            print("Planner is waiting for odometry before starting.")
            rospy.sleep(2)

        local_bounds = ipp_utils.generate_local_bounds(self.bounds, self.planner_initial_pose, self.horizon_distance, self.border_margin)
        
        samples = np.random.uniform(low=[local_bounds[0], local_bounds[1], -np.pi], high=[local_bounds[2], local_bounds[3], np.pi], size=[nbr_samples, 3])
        h = std_msgs.msg.Header()
        h.frame_id = self.map_frame
        h.stamp = rospy.Time.now()
        sampling_path = Path()
        sampling_path.header = h
        for sample in samples:
            path = dubins.shortest_path(self.planner_initial_pose, [sample[0], sample[1], sample[2]], self.turning_radius)
            wp_poses, _ = path.sample_many(self.wp_resolution)
            skip = 1
            if len(wp_poses) == 1:
                skip = 0
            self.wp_list.extend(wp_poses[skip:])
            for pose in wp_poses[skip:]:
                wp = PoseStamped()
                wp.header = h
                wp.pose.position.x = pose[0]
                wp.pose.position.y = pose[1]
                wp.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, pose[2]))
                sampling_path.poses.append(wp)
                self.nbr_wp += 1
            self.planner_initial_pose = wp_poses[-1]
        return sampling_path
    
    
    # def calculate_time2target(self):
    #     """ Calculates the time to target based on euclidean distance
    #         along the path. 
            
    #         NOTE: The Euclidean path is not a perfect measurement,
    #         but it is fast. This was not used in real experiments,
    #         as it was found to be unreliable (speed was not as expected).

    #     Returns:
    #         double: time to arrival at target location
    #     """
    #     speed = self.vehicle_velocity
    #     distance = 0
    #     current = copy.deepcopy(self.state)
    #     for pose in self.wp_list:
    #         distance += np.hypot(current[0] - pose[0], current[1] - pose[1])
    #         current = pose
    #     return distance/speed
             
    def execute_planning(self, msg):
        """ 
            This callback essentially runs the entire BO planning loop. To begin with,
            we freeze a copy of the current GP to use for BO. A myopic BO could potentially
            get away with using the current GP, but thread safety becomes an issue there as well.
            
            A tree search (with *sort of* MC based method) performs BO to find
            candidates (locations) in this frozen GP. For each candidate, we can then
            train a new, separate GP where we have simulated MBES measurements as
            if we actually travelled to that candidate location. The tree expands,
            and repeat until we run out of time.
            
            The location itself is the intensive part to find. After deciding that we are 
            running out of time, we take our currently best location, and decide the best
            angle of the connecting Dubins path. The optimal angle is found through
            secondary BO, with rewards from sampling MBES swaths along the candidate 
            paths (defined by the already determined location, and candidate angle).

        """
    
        if self.odom_init:

            self.bo_planning = True

        else:

            print("Odom not initialized yet")
        
    def bo_plan(self):

        # Myopic quick candidate with angle optimization 

        # rush_order_activated  = True
        local_bounds          = ipp_utils.generate_local_bounds(self.bounds, self.planner_initial_pose, self.horizon_distance, self.border_margin)
        self.bounds_XY_torch  = torch.tensor([[local_bounds[0], local_bounds[1]], [local_bounds[2], local_bounds[3]]]).to(torch.float)
        self.XY_acqf          = botorch.acquisition.UpperConfidenceBound(model=self.gp.model, beta=self.beta)
        candidates_XY, _      = botorch.optim.optimize_acqf(acq_function=self.XY_acqf, bounds=self.bounds_XY_torch, q=1, num_restarts=5, raw_samples=50)
        
        print("XY cand ", candidates_XY)
        
        print("Optimizing angle...")
        BO = BayesianOptimizerClass.BayesianOptimizer(self.state, self.gp.model, wp_resolution=self.wp_resolution,
                                                    turning_radius=self.turning_radius, swath_width=self.swath_width,
                                                    path_nbr_samples=self.path_nbr_samples, voxel_size=self.voxel_size,
                                                    wp_sample_interval=self.wp_sample_interval, device=self.gp.device)
        
        angle_optim_max_iter = 5
        # if rush_order_activated:
        #     print("Not enough time for full angle optimization, rush order requested")
        #     angle_optim_max_iter = 0

        candidates_theta, angle_gp  = BO.optimize_theta_with_grad(XY=candidates_XY, max_iter=angle_optim_max_iter, nbr_samples=15)
        candidate = torch.cat([candidates_XY.to(self.gp.device), candidates_theta.to(self.gp.device)], 1).squeeze(0)
        
        with open("angle_gp" + str(self.distance_travelled) + ".pickle", 'wb') as handle:
            pickle.dump(angle_gp, handle)
        
        print("Optimal pose: ", candidate, ", publishing trajectory.")
        
        # Publish this trajectory as a set of waypoints
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        h.frame_id = self.map_frame
        sampling_path = Path()
        sampling_path.header = h
        location = candidate.cpu().numpy()
        path = dubins.shortest_path(self.planner_initial_pose, [location[0], location[1], location[2]], self.turning_radius)
        wp_poses, _ = path.sample_many(self.wp_resolution)
        skip = 1
        if len(wp_poses) == 1:
            skip = 0
        self.wp_list.extend(wp_poses[skip:])
        for pose in wp_poses[skip:]:       #removing first as hack to ensure AUV doesnt get stuck
            wp = PoseStamped()
            wp.header = h
            wp.pose.position.x = pose[0] 
            wp.pose.position.y = pose[1]  
            wp.pose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, pose[2]))
            sampling_path.poses.append(wp)
            self.nbr_wp += 1
        self.planner_initial_pose = wp_poses[-1]
        self.path_pub.publish(sampling_path)

        self.currently_planning = False
        self.finish_imminent = False
        
        #torch.save({"model": angle_gp.state_dict()}, self.store_path  + "_GP_" + str(round(self.distance_travelled)) + "_angle.pickle")
        #torch.save({"model": self.gp.model.state_dict()}, self.store_path + "_GP_" + str(round(self.distance_travelled)) + "_env.pickle")
        #print("Decision models saved.")
        print("Current distance travelled: " + str(round(self.distance_travelled)) + " m.")
        


if __name__ == '__main__':

    rospy.init_node('myopic_planner_node' , disable_signals=False)
    node_name = rospy.get_name()
    namespace = rospy.get_namespace()

    # Get parameters from ROS
    # choice              = rospy.get_param("~planner_type")
    turn_radius         = rospy.get_param("~turning_radius")
    corner_topic        = rospy.get_param("~corner_topic")
    path_topic          = rospy.get_param("~path_topic")
    planner_req_topic   = rospy.get_param("~planner_req_topic")
    odom_topic          = rospy.get_param("~odom_topic")
    swath_width         = rospy.get_param("~swath_width")
    bound_left          = rospy.get_param("~bound_left")
    bound_right         = rospy.get_param("~bound_right")
    bound_up            = rospy.get_param("~bound_up")
    bound_down          = rospy.get_param("~bound_down")
    train_rate          = rospy.get_param("~train_rate")
    wp_resolution       = rospy.get_param("~wp_resolution")
    path_nbr_samples    = rospy.get_param("~path_nbr_samples")
    voxel_size          = rospy.get_param("~voxel_size")
    wp_sample_interval  = rospy.get_param("~wp_sample_interval")
    horizon_distance    = rospy.get_param("~horizon_distance")
    border_margin       = rospy.get_param("~border_margin")
    beta                = rospy.get_param("~beta")    

    # Make sure map bounds make sense
    low_x = min(bound_left, bound_right)
    high_x = max(bound_left, bound_right)
    low_y = min(bound_down, bound_up)
    high_y = max(bound_down, bound_up)

    bounds = [low_x, low_y, high_x, high_y]
    
    assert bounds[0] < bounds[2],       "planner_node: Given global bounds wrong in X dimension"
    assert bounds[1] < bounds[3],       "planner_node: Given global bounds wrong in Y dimension"
    assert odom_topic != "",            "planner_node: Odom topic empty"
    assert path_topic != "",            "planner_node: Path topic empty"
    assert corner_topic != "",          "planner_node: Corner topic empty"

    try:
        rospy.loginfo("Initializing planner node! Using Bayesian Optimization.")  
        planner = MyopicBOPlanner(corner_topic=corner_topic, path_topic=path_topic, 
                            planner_req_topic=planner_req_topic, odom_topic=odom_topic,bounds=bounds, 
                            turning_radius=turn_radius, training_rate=train_rate, 
                            wp_resolution=wp_resolution, swath_width=swath_width, 
                            path_nbr_samples=path_nbr_samples, 
                            voxel_size=voxel_size, wp_sample_interval=wp_sample_interval, 
                            horizon_distance=horizon_distance, border_margin=border_margin, 
                            beta=beta)
        # rospy.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch myopic planner")