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
# import dubins_smarc
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
        self.wp_follower_type = rospy.get_param('~waypoint_follower_type', 'simple')
        self.dubins_step_size = rospy.get_param('~dubins_step_size', 0.5)
        self.dubins_turning_radius = rospy.get_param('~dubins_turning_radius', 5)
        self.namespace = rospy.get_param('~namespace', 'hugin')
        self.max_throttle = rospy.get_param('~max_throttle', 4)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 0.5)


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
        self.path_pub = rospy.Publisher('local_path', Path, queue_size=1)

        self.wp_old = None
        self.wp_artificial_old = None

        self.point_marker_pub = rospy.Publisher('artificial_wps', MarkerArray, queue_size=1)
        self.arrival_time_pub = rospy.Publisher('arrival_time', Time, queue_size=1)

        self.wp_counter = 0

        while not rospy.is_shutdown():
            
            if self.latest_path.poses and not self.relocalizing:
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
                
                

                if self.wp_follower_type == 'dubins' or self.wp_follower_type == 'simple_maxturn' :
                    if self.wp_old is None:
                        self.wp_old = robot_pose
                        self.wp_artificial_old = robot_pose
                
                    wps = self.generate_two_artificial_wps(wp)
                    if wps is None:
                        continue
                    self.wp_old = wp
                    if self.wp_counter == 0: #remove the very first artifialc wp since it's straight infront turninigradius distance away (will give issues with the max_turn/on_circle check)
                        wps.pop(0)
                    self.publish_points_to_rviz(wps)
                    
                    for i,wp in enumerate(wps):
                        if i==1:
                            print("sending arrival time")
                            print(self.common_timestamps)
                            arrival_time = Time()
                            arrival_time.data.secs = self.common_timestamps.pop(0)
                            self.arrival_time_pub.publish(arrival_time)
                        
                        # configurations = [(wp.pose.position.x,wp.pose.position.y,tf.transformations.euler_from_quaternion([wp.pose.orientation.x,wp.pose.orientation.y,wp.pose.orientation.z,wp.pose.orientation.w])[2])]
                        
                        if i==1 or self.wp_counter == 0 or self.wp_follower_type == 'simple_maxturn':
                            configurations = [(wp.pose.position.x,wp.pose.position.y,tf.transformations.euler_from_quaternion([wp.pose.orientation.x,wp.pose.orientation.y,wp.pose.orientation.z,wp.pose.orientation.w])[2])]


                        if self.wp_follower_type == 'dubins':
                            configurations = self.generate_dubins_path(wp,self.wp_artificial_old)
                            configurations = self.filter_dubins_path(configurations)

                        self.wp_artificial_old = wp
                    
                        path = Path()
                        path.header.frame_id = self.map_frame
                        path.header.stamp = rospy.Time(0)
                        print("len configurations: %d" % len(configurations))
                        for sub_wp in configurations:
                            wp = PoseStamped()
                            wp.header.frame_id = self.map_frame
                            wp.header.stamp = rospy.Time(0)
                            wp.pose.position.x = sub_wp[0]
                            wp.pose.position.y = sub_wp[1]
                            wp.pose.position.z = 0
                            quaternion = tf.transformations.quaternion_from_euler(0, 0, sub_wp[2])
                            wp.pose.orientation = Quaternion(*quaternion)

                            path.poses.append(wp)
                        self.path_pub.publish(path)

                        for i,wp in enumerate(path.poses):
                            print("Sending wp %d of %d" % (i+1,len(path.poses)))
                            goal = MoveBaseGoal(wp)
                            goal.target_pose.header.frame_id = self.map_frame
                            self.ac.send_goal(goal)
                            self.ac.wait_for_result()
                            rospy.loginfo("WP reached, moving on to next one")

                elif self.wp_follower_type == 'simple':
                    #Publish path to rviz
                    path = Path()
                    path.header.frame_id = self.map_frame
                    path.header.stamp = rospy.Time(0)
                    path.poses.append(robot_pose)
                    path.poses.append(wp)
                    self.path_pub.publish(path)

                    # TODO: normalize quaternions here according to rviz warning?
                    goal = MoveBaseGoal(wp)
                    goal.target_pose.header.frame_id = self.map_frame
                    self.ac.send_goal(goal)
                    self.ac.wait_for_result()
                    rospy.loginfo("WP reached, moving on to next one")
                else:
                    rospy.logerr("Unknown waypoint follower type: %s", self.wp_follower_type)
                    raise ValueError("Unknown waypoint follower type: %s", self.wp_follower_type)
                self.wp_counter += 1
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
        # print(np.array(data)-np.concatenate(([0],data[:-1])))
        # data /= self.max_throttle
        # print(data)
        # duration1 = data[0]
        # duration2 = data[1]-data[0]
        #I want to create an np array where the first element is 0, and the rest are the sum of the previous element and the duration altering between duration1 and duration2
        # dataNew = np.cumsum(np.concatenate((np.tile([duration1,duration2],int(len(data)/2)),[duration1]))) 
        # print("newdata:",dataNew)
        # assert len(dataNew) == len(data)
        # data = dataNew
        # print(np.array(data)-np.concatenate(([0],data[:-1])))

        n_turns = len(data)
        if self.wp_follower_type == 'dubins':
            turn_duration = 4 #seconds
        else:
            turn_duration = 2 #seconds
        data[1:] = data[1:] + np.arange(1,n_turns)*turn_duration #arange vector is [1 2 3 4 .... n_turns-1]
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

    def generate_two_artificial_wps(self,wp):
        wp_start = np.array([self.wp_old.pose.position.x, self.wp_old.pose.position.y])
        wp_end = np.array([wp.pose.position.x, wp.pose.position.y])

        norm = np.linalg.norm(wp_end-wp_start)
        buffer = 0 #buffer in m, to avoid edge cases
        num_wps = int(norm/(self.dubins_turning_radius + buffer))
        xs=np.linspace(wp_start[0],wp_end[0],num_wps)
        ys=np.linspace(wp_start[1],wp_end[1],num_wps)
        if len(xs) == 0:
            return None
        wp1_vec = np.array([xs[1],ys[1]])
        wp2_vec = np.array([xs[-2],ys[-2]])

        delta_x = wp2_vec[0] - wp1_vec[0]
        delta_y = wp2_vec[1] - wp1_vec[1]

        if abs(delta_x) > abs(delta_y):
            if delta_x > 0:
                heading = 0
            elif delta_x < 0:
                heading = math.pi
        else:
            if delta_y > 0:
                heading = math.pi/2
            elif delta_y < 0: #Won't really happen since we're always going forwards
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
            marker.scale.x = self.goal_tolerance#5
            marker.scale.y = self.goal_tolerance#5
            marker.scale.z = self.goal_tolerance#5
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
            if abs(delta_heading) > np.deg2rad(2):# or not np.isclose(current_configuration[2]%(np.pi/2),0):  # You can adjust this threshold
                filtered_configurations.append(current_configuration)
        filtered_configurations.append(configurations[-1])  # Add the goal point
        return filtered_configurations

if __name__ == '__main__':

    rospy.init_node('w2w_mission_planner')
    planner = W2WMissionPlanner(rospy.get_name())
