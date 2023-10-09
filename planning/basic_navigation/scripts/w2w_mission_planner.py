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

                goal_pose = PoseStamped()
                goal_pose.header.frame_id = wp.header.frame_id
                goal_pose.header.stamp = rospy.Time(0)
                goal_pose.pose.position = wp.pose.position
                goal_pose.pose.orientation = wp.pose.orientation

                goal_pose_local = self.listener.transformPose(
                    self.base_frame, goal_pose)
                
                # #plot goal point vs base frame
                # plt.figure()
                # plt.plot(goal_pose_local.point.x, goal_pose_local.point.y, 'ro')
                # plt.plot(0,0,'bo')
                # plt.axis('equal')
                # plt.show()

                
                #dubins tests
                q0 = (0,0,0)
                goal_heading = tf.transformations.euler_from_quaternion([goal_pose_local.pose.orientation.x,goal_pose_local.pose.orientation.y,goal_pose_local.pose.orientation.z,goal_pose_local.pose.orientation.w])[2]
                q1 = (goal_pose_local.pose.position.x, goal_pose_local.pose.position.y, goal_heading)
                turning_radius = 5.0
                step_size = 1.0

                path = dubins.shortest_path(q0, q1, turning_radius)
                configurations, _ = path.sample_many(step_size)

                # # Plot
                # configurations_array = np.array(configurations)
                # if len(configurations_array) > 0:
                #     plt.figure()
                #     plt.plot(configurations_array[:,0], configurations_array[:,1])
                #     plt.axis('equal')
                #     plt.show()
                dubins_path = Path()
                dubins_path.header.frame_id = self.base_frame
                dubins_path.header.stamp = rospy.Time(0)
                for sub_wp in configurations:
                    wp = PoseStamped()
                    wp.header.frame_id = self.base_frame
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
                
                for wp in dubins_path.poses:
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
