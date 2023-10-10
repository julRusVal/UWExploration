#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped,Quaternion
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseResult, MoveBaseAction, MoveBaseGoal
import actionlib
import rospy
import tf
from std_msgs.msg import Float64, Header, Bool
import math
import dubins
import pdb
import copy



class W2WMissionPlanner(object):

    # create messages that are used to publish feedback/result
    _feedback = MoveBaseFeedback()
    _result = MoveBaseResult()

    def __init__(self, name):
        self._action_name = name

        #self.heading_offset = rospy.get_param('~heading_offsets', 5.)
        self.planner_as_name = rospy.get_param('~path_planner_as')
        self.path_topic = rospy.get_param('~path_topic')
        self.wp_topic = rospy.get_param('~wp_topic')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.relocalize_topic = rospy.get_param('~relocalize_topic')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')


        # The waypoints as a path
        rospy.Subscriber(self.path_topic, Path, self.path_cb, queue_size=1)
        self.latest_path = Path()

        # LC waypoints, individually
        rospy.Subscriber(self.wp_topic, PoseStamped, self.wp_cb, queue_size=1)

        # The bs driver can be fairly slow on the simulations, so it's necessary
        # to stop the vehicle until the LC area has been selected
        rospy.Subscriber(self.relocalize_topic, Bool, self.start_relocalize, queue_size=1)
        self.relocalizing = False

        # The client to send each wp to the server
        self.ac = actionlib.SimpleActionClient(self.planner_as_name, MoveBaseAction)
        while not self.ac.wait_for_server(rospy.Duration(1)) and not rospy.is_shutdown():
            rospy.loginfo("Waiting for action client: %s",
                          self.planner_as_name)
        rospy.loginfo("Action client connected: %s", self.planner_as_name)

        self.listener = tf.TransformListener()
        self.dubins_pub = rospy.Publisher('dubins_path', Path, queue_size=1)


        while not rospy.is_shutdown():
            
            if self.latest_path.poses and not self.relocalizing:
                # Get next waypoint in path
                rospy.loginfo("Sending WP")
                wp = self.latest_path.poses[0]
                del self.latest_path.poses[0]

                #Create dubins path
                #-----------------------------------------------------------

                # goal_pose = PoseStamped()
                # goal_pose.header.frame_id = wp.header.frame_id
                # goal_pose.header.stamp = rospy.Time.now()
                # goal_pose.pose.position = wp.pose.position
                # goal_pose.pose.orientation = wp.pose.orientation

                goal_pose = copy.deepcopy(wp)
                goal_pose.header.stamp = rospy.Time(0)

                # goal_pose_local = self.listener.transformPose(
                #     self.base_frame, goal_pose)
                
                # #plot goal point vs base frame
                # plt.figure()
                # plt.plot(goal_pose_local.point.x, goal_pose_local.point.y, 'ro')
                # plt.plot(0,0,'bo')
                # plt.axis('equal')
                # plt.show()

                robot_pose_local = PoseStamped()
                robot_pose_local.header.frame_id = self.base_frame
                robot_pose_local.header.stamp = rospy.Time(0)

                robot_pose = self.listener.transformPose( #we need to send all wp's in map frame in order to be able to check if wp is reached correctly. Otherwise we would never stop (due to how we check if reached). Also it's cheaper to transform once for each robot rather than trasnforming for all wps in the dubins path
                    self.map_frame, robot_pose_local)

                
                #dubins tests
                robot_heading = tf.transformations.euler_from_quaternion([robot_pose.pose.orientation.x,robot_pose.pose.orientation.y,robot_pose.pose.orientation.z,robot_pose.pose.orientation.w])[2]
                q0 = (robot_pose.pose.position.x, robot_pose.pose.position.y, robot_heading)
                goal_heading = tf.transformations.euler_from_quaternion([goal_pose.pose.orientation.x,goal_pose.pose.orientation.y,goal_pose.pose.orientation.z,goal_pose.pose.orientation.w])[2]
                q1 = (goal_pose.pose.position.x, goal_pose.pose.position.y, goal_heading)
                turning_radius = 5
                step_size = 0.5

                path = dubins.shortest_path(q0, q1, turning_radius)
                configurations, _ = path.sample_many(step_size)
                del configurations[0:3] #remove first wp since it's the robot pose
                #List all available methods for the path object
                # print(dir(path))

                # pdb.set_trace()
                #sub sample configurations
                # configurations = configurations[::10]
                # pdb.set_trace()
                # configurations = self.filter_dubins_path(configurations)
                # pdb.set_trace()
                # # Plot
                # configurations_array = np.array(configurations)
                # if len(configurations_array) > 0:
                #     plt.figure()
                #     plt.plot(configurations_array[:,0], configurations_array[:,1])
                #     plt.axis('equal')
                #     plt.show()
                dubins_path = Path()
                dubins_path.header.frame_id = self.map_frame
                dubins_path.header.stamp = rospy.Time(0)
                for sub_wp in configurations:
                    wp = PoseStamped()
                    wp.header.frame_id = self.map_frame
                    wp.header.stamp = rospy.Time(0)
                    wp.pose.position.x = sub_wp[0]
                    wp.pose.position.y = sub_wp[1]
                    wp.pose.position.z = 0
                    quaternion = tf.transformations.quaternion_from_euler(0, 0, sub_wp[2])
                    wp.pose.orientation = Quaternion(*quaternion)
                    # self.latest_path.poses.insert(0, wp)

                    dubins_path.poses.append(wp)
                
                self.dubins_pub.publish(dubins_path)

                #TODO: 
                #1. The wps sent to w2w_planner are supposed to be base_link, but they are strange. Plotting the dubins_path, seems to go from map frame
                #2. When sending the wps, it's sending too many to w2w_planner. It moves to each one that are super close, make it smoother
                
                for i,wp in enumerate(dubins_path.poses):
                    print("Sending wp %d of %d" % (i+1,len(dubins_path.poses)))
                    # print(wp)
                    goal = MoveBaseGoal(wp)
                    goal.target_pose.header.frame_id = self.map_frame
                    self.ac.send_goal(goal)
                    self.ac.wait_for_result()
                    rospy.loginfo("WP reached, moving on to next one")
                #-----------------------------------------------------------


                # TODO: normalize quaternions here according to rviz warning?
                # goal = MoveBaseGoal(wp)
                # goal.target_pose.header.frame_id = self.map_frame
                # self.ac.send_goal(goal)
                # self.ac.wait_for_result()
                # rospy.loginfo("WP reached, moving on to next one")

            elif not self.latest_path.poses:
                rospy.loginfo_once("Mission finished")

    def filter_dubins_path(self, configurations):
        """If you want to reduce the number of waypoints and have 
        waypoints only before each left turn, right turn, or going 
        straight, you'll need to post-process the generated path to 
        filter out unnecessary waypoints. """
        
        filtered_configurations = []  

        # filtered_configurations.append(configurations[0]) # Add the start point

        for i in range(1, len(configurations) - 1):
            prev_configuration = configurations[i - 1]
            current_configuration = configurations[i]
            next_configuration = configurations[i + 1]

            # Calculate the change in heading from the previous waypoint to the current one
            delta_heading = current_configuration[2] - prev_configuration[2]

            # Check if the waypoint is before a turn or on a straight segment
            if abs(delta_heading) > np.deg2rad(2):  # You can adjust this threshold
                filtered_configurations.append(current_configuration)

        filtered_configurations.append(configurations[-1])  # Add the goal point
        return filtered_configurations

    def start_relocalize(self, bool_msg):
        self.relocalizing = bool_msg.data

    def path_cb(self, path_msg):
        self.latest_path = path_msg
        rospy.loginfo("Path received with number of wp: %d",
                      len(self.latest_path.poses))

    def wp_cb(self, wp_msg):
        # Waypoints for LC from the backseat driver
        rospy.loginfo("LC wp received")
        self.latest_path.poses.insert(0, wp_msg)


if __name__ == '__main__':

    rospy.init_node('w2w_mission_planner')
    planner = W2WMissionPlanner(rospy.get_name())
