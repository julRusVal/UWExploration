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
from std_msgs.msg import Float64, Header, Bool, Time, Int32MultiArray
import math
import dubins
import pdb
import copy
import dubins_smarc
from visualization_msgs.msg import Marker, MarkerArray



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
        self.wp_follower_type = rospy.get_param('~waypoint_follower_type', 'dubins_smarc')
        self.dubins_step_size = rospy.get_param('~dubins_step_size', 0.5)
        self.dubins_turning_radius = rospy.get_param('~dubins_turning_radius', 5)
        self.namespace = rospy.get_param('~namespace', 'hugin')
        self.max_throttle = rospy.get_param('~max_throttle', 4)


        # The waypoints as a path
        rospy.Subscriber(self.path_topic, Path, self.path_cb, queue_size=1)
        self.latest_path = Path()

        # LC waypoints, individually
        rospy.Subscriber(self.wp_topic, PoseStamped, self.wp_cb, queue_size=1)

        # The bs driver can be fairly slow on the simulations, so it's necessary
        # to stop the vehicle until the LC area has been selected
        rospy.Subscriber(self.relocalize_topic, Bool, self.start_relocalize, queue_size=1)
        self.relocalizing = False

        rospy.Subscriber('/multi_agent/common_timestamps',Int32MultiArray,self.common_timestamps_cb,queue_size=1)
        self.common_timestamps = None

        # The client to send each wp to the server
        self.ac = actionlib.SimpleActionClient(self.planner_as_name, MoveBaseAction)
        while not self.ac.wait_for_server(rospy.Duration(1)) and not rospy.is_shutdown():
            rospy.loginfo("Waiting for action client: %s",
                          self.planner_as_name)
        rospy.loginfo("Action client connected: %s", self.planner_as_name)

        self.listener = tf.TransformListener()
        self.dubins_pub = rospy.Publisher('dubins_path', Path, queue_size=1)

        self.wp_old = None
        self.wp_artificial_old = None

        self.point_marker_pub = rospy.Publisher('artificial_wps', MarkerArray, queue_size=1)
        self.delta_t_pub = rospy.Publisher('delta_t', Time, queue_size=1)
        self.arrival_time_pub = rospy.Publisher('arrival_time', Time, queue_size=1)

        self.wp_counter = 0


        while not rospy.is_shutdown():
            
            if self.latest_path.poses and not self.relocalizing:
                self.wp_counter += 1
                # Get next waypoint in path
                rospy.loginfo("Sending WP")
                wp = self.latest_path.poses[0]
                del self.latest_path.poses[0]

                goal_pose = copy.deepcopy(wp)
                goal_pose.header.stamp = rospy.Time(0)

                robot_pose_local = PoseStamped()
                robot_pose_local.header.frame_id = self.base_frame
                robot_pose_local.header.stamp = rospy.Time(0)

                robot_pose = self.listener.transformPose( #we need to send all wp's in map frame in order to be able to check if wp is reached correctly. Otherwise we would never stop (due to how we check if reached). Also it's cheaper to transform once for each robot rather than trasnforming for all wps in the dubins path
                    self.map_frame, robot_pose_local)
                
                

                #Create dubins path
                if self.wp_follower_type == 'dubins' or self.wp_follower_type == 'dubins_smarc':
                    if self.wp_old is None:
                        self.wp_old = robot_pose
                        self.wp_artificial_old = robot_pose
                
                    wps = self.generate_two_artificial_wps(wp)
                    if wps is None:
                        continue
                    self.wp_old = wp
                    self.publish_points_to_rviz(wps)
                    # delta_t = self.calc_optimal_delta_t(wps[0],wps[1])
                    # if delta_t is not None:
                    #     self.delta_t_pub.publish(delta_t)

                    #-----------------------------------------------------------
                    # goal_pose = copy.deepcopy(wp)
                    # goal_pose.header.stamp = rospy.Time(0)

                    # robot_pose_local = PoseStamped()
                    # robot_pose_local.header.frame_id = self.base_frame
                    # robot_pose_local.header.stamp = rospy.Time(0)

                    # robot_pose = self.listener.transformPose( #we need to send all wp's in map frame in order to be able to check if wp is reached correctly. Otherwise we would never stop (due to how we check if reached). Also it's cheaper to transform once for each robot rather than trasnforming for all wps in the dubins path
                    #     self.map_frame, robot_pose_local)

                    #dubins tests
                    # robot_heading = tf.transformations.euler_from_quaternion([robot_pose.pose.orientation.x,robot_pose.pose.orientation.y,robot_pose.pose.orientation.z,robot_pose.pose.orientation.w])[2]
                    # q0 = (robot_pose.pose.position.x, robot_pose.pose.position.y, robot_heading)
                    # goal_heading = tf.transformations.euler_from_quaternion([goal_pose.pose.orientation.x,goal_pose.pose.orientation.y,goal_pose.pose.orientation.z,goal_pose.pose.orientation.w])[2]
                    # q1 = (goal_pose.pose.position.x, goal_pose.pose.position.y, goal_heading)
                    # turning_radius = 15
                    # step_size = 0.5

                    # path = dubins.shortest_path(q0, q1, turning_radius)
                    # configurations, _ = path.sample_many(step_size)
                    # del configurations[0:3] #remove first wp since it's the robot pose
                    for i,wp in enumerate(wps):
                        if i==1:
                            # print("sending delta t for time boost")
                            # delta_t = self.calc_optimal_delta_t(wps[0],wps[1])
                            # if delta_t is not None:
                            #     self.delta_t_pub.publish(delta_t)
                            print("sending arrival time")
                            print(self.common_timestamps)
                            arrival_time = Time()
                            arrival_time.data.secs = self.common_timestamps.pop(0)
                            self.arrival_time_pub.publish(arrival_time)

                        if self.wp_follower_type == 'dubins':
                            configurations = self.generate_dubins_path(wp,self.wp_artificial_old)
                        elif self.wp_follower_type == 'dubins_smarc':
                            configurations = self.generate_dubins_smarc_path(wp,self.wp_artificial_old)
                        self.wp_artificial_old = wp

                        configurations = self.filter_dubins_path(configurations) #filter out unnecessary wps in straight lines
                    
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
                        
                        for i,wp in enumerate(dubins_path.poses):
                            print("Sending wp %d of %d" % (i+1,len(dubins_path.poses)))
                            # print(wp)
                            goal = MoveBaseGoal(wp)
                            goal.target_pose.header.frame_id = self.map_frame
                            self.ac.send_goal(goal)
                            self.ac.wait_for_result()
                            rospy.loginfo("WP reached, moving on to next one")
                        # rospy.sleep(10)
                    #-----------------------------------------------------------

                elif self.wp_follower_type == 'simple':
                    #Publish path to rviz
                    dubins_path = Path()
                    dubins_path.header.frame_id = self.map_frame
                    dubins_path.header.stamp = rospy.Time(0)
                    dubins_path.poses.append(robot_pose)
                    dubins_path.poses.append(wp)
                    self.dubins_pub.publish(dubins_path)

                    # TODO: normalize quaternions here according to rviz warning?
                    goal = MoveBaseGoal(wp)
                    goal.target_pose.header.frame_id = self.map_frame
                    self.ac.send_goal(goal)
                    self.ac.wait_for_result()
                    rospy.loginfo("WP reached, moving on to next one")
                else:
                    rospy.logerr("Unknown waypoint follower type: %s", self.wp_follower_type)
                    raise ValueError("Unknown waypoint follower type: %s", self.wp_follower_type)

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
    
    def common_timestamps_cb(self, msg):
        rospy.loginfo("Received common timestamps from path pattern generator")
        data = np.array(msg.data[1:])
        n_turns = len(data)
        turn_duration = 10 #seconds
        data[1:] = data[1:] + np.arange(1,n_turns)*turn_duration
        self.common_timestamps = list(data) #remove the first one at 0 since it's the start time
    
    def generate_dubins_path(self,goal_pose,robot_pose):
        robot_heading = tf.transformations.euler_from_quaternion([robot_pose.pose.orientation.x,robot_pose.pose.orientation.y,robot_pose.pose.orientation.z,robot_pose.pose.orientation.w])[2]
        q0 = (robot_pose.pose.position.x, robot_pose.pose.position.y, robot_heading)
        goal_heading = tf.transformations.euler_from_quaternion([goal_pose.pose.orientation.x,goal_pose.pose.orientation.y,goal_pose.pose.orientation.z,goal_pose.pose.orientation.w])[2]
        q1 = (goal_pose.pose.position.x, goal_pose.pose.position.y, goal_heading)
        turning_radius = self.dubins_turning_radius
        step_size = self.dubins_step_size

        path = dubins.shortest_path(q0, q1, turning_radius)
        configurations, _ = path.sample_many(step_size)
        return configurations

    def generate_dubins_smarc_path(self,goal_pose,robot_pose):
        wp_start = dubins_smarc.Waypoint(robot_pose.pose.position.x, robot_pose.pose.position.y, tf.transformations.euler_from_quaternion([robot_pose.pose.orientation.x,robot_pose.pose.orientation.y,robot_pose.pose.orientation.z,robot_pose.pose.orientation.w])[2])
        wp_end = dubins_smarc.Waypoint(goal_pose.pose.position.x, goal_pose.pose.position.y, tf.transformations.euler_from_quaternion([goal_pose.pose.orientation.x,goal_pose.pose.orientation.y,goal_pose.pose.orientation.z,goal_pose.pose.orientation.w])[2])

        turning_radius = self.dubins_turning_radius
        step_size = self.dubins_step_size

        traj = dubins_smarc.sample_between_wps(wp_start, wp_end, turning_radius, step_size)
        #convert traj from 2D array with N rows and 3 columns to 1D array containing N 3D tuples
        configurations = [tuple(x) for x in traj]
        return configurations

    def generate_two_artificial_wps(self,wp):
        wp_start = np.array([self.wp_old.pose.position.x, self.wp_old.pose.position.y])
        wp_end = np.array([wp.pose.position.x, wp.pose.position.y])

        # wp1_norm = np.sqrt(self.dubins_turning_radius**2/(wp_start[0]**2+wp_start[1]**2)) + 1
        # wp2_norm = 1 - np.sqrt(self.dubins_turning_radius**2/(wp_end[0]**2+wp_end[1]**2))

        # wp1 = copy.deepcopy(self.wp_old)
        # wp1.pose.position.x *= wp1_norm
        # wp1.pose.position.y *= wp1_norm
        # wp1.pose.orientation = self.wp_old.pose.orientation
        
        # wp2 = copy.deepcopy(wp)
        # wp2.pose.position.x *= wp2_norm
        # wp2.pose.position.y *= wp2_norm
        # wp2.pose.orientation = self.wp_old.pose.orientation

        norm = np.linalg.norm(wp_end-wp_start)
        buffer = 3 #buffer in m, to avoid edge cases
        num_wps = int(norm/(self.dubins_turning_radius + buffer))
        xs=np.linspace(wp_start[0],wp_end[0],num_wps)
        ys=np.linspace(wp_start[1],wp_end[1],num_wps)
        if len(xs) == 0:
            return None
        wp1_vec = np.array([xs[1],ys[1]])
        wp2_vec = np.array([xs[-2],ys[-2]])

        if wp2_vec[0] > wp1_vec[0]:
            heading = 0
        elif wp2_vec[0] < wp1_vec[0]:
            heading = math.pi
        elif wp2_vec[1] > wp1_vec[1]:
            heading = math.pi/2
        elif wp2_vec[1] < wp1_vec[1]: #Won't really happen since we're always going forwards
            heading = -math.pi/2

        quaternion = tf.transformations.quaternion_from_euler(0, 0, heading)

        wp1 = copy.deepcopy(wp)
        wp1.pose.position.x = wp1_vec[0]
        wp1.pose.position.y = wp1_vec[1]
        wp1.pose.orientation = Quaternion(*quaternion)
        
        wp2 = copy.deepcopy(wp)
        wp2.pose.position.x = wp2_vec[0]
        wp2.pose.position.y = wp2_vec[1]
        wp2.pose.orientation = Quaternion(*quaternion)
        return [wp1,wp2]

    def publish_points_to_rviz(self,points_array):
        marker_array = MarkerArray()
        for i,point in enumerate(points_array):
            marker = Marker()
            marker.header.frame_id = self.map_frame
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.id = i
            marker.scale.x = 5
            marker.scale.y = 5
            marker.scale.z = 5
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = point.pose.position.x
            marker.pose.position.y = point.pose.position.y
            marker.pose.position.z = point.pose.position.z
            marker_array.markers.append(marker)
        self.point_marker_pub.publish(marker_array)
    
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
            if abs(delta_heading) > np.deg2rad(1) or not np.isclose(current_configuration[2]%(np.pi/2),0):  # You can adjust this threshold
                filtered_configurations.append(current_configuration)
        filtered_configurations.append(configurations[-1])  # Add the goal point
        return filtered_configurations

    def calc_optimal_delta_t(self,wp1,wp2):
        """Calculate the optimal delta_t if moving in straight line between two waypoints and using max velocity"""
        delta_s = np.linalg.norm(np.array([wp2.pose.position.x,wp2.pose.position.y])-np.array([wp1.pose.position.x,wp1.pose.position.y]))
        if wp1.pose.orientation == wp2.pose.orientation:
            #only return delta_t if we're moving in a straight line
            delta_t = Time()
            delta_t.data.secs = int(delta_s/self.max_throttle)
            return delta_t
        else:
            return None

if __name__ == '__main__':

    rospy.init_node('w2w_mission_planner')
    planner = W2WMissionPlanner(rospy.get_name())
