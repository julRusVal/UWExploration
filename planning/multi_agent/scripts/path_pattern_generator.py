#!/usr/bin/env python3
import rospy
from subprocess import call, Popen
import numpy as np
import rospkg
from multi_agent.msg import AgentPath, AgentPathArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point
from coop_cov import mission_plan
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
import matplotlib.pyplot as plt

import tf.transformations
from geometry_msgs.msg import Quaternion

from visualization_msgs.msg import Marker, MarkerArray

from rviz_visualization.srv import DisplayRvizMessage, DisplayRvizMessageRequest

from std_msgs.msg import String

import copy

class PatternGenerator():
    def __init__(self):
        # Initialize the TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.num_auvs = rospy.get_param('num_auvs',1)
        self.vehicle_model = rospy.get_param('vehicle_model','hugin')
        
        self.paths_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        # self.goal = None
        self.bottom_left = None
        self.top_right = None

        self.spawn_pos_path_array_topic = rospy.get_param('path_array_spawn_pos_topic', '/multi_agent/spawn_pos/path_array')
        self.spawn_pos_path_array_pub = rospy.Publisher(self.spawn_pos_path_array_topic, AgentPathArray, queue_size=1)
        self.paths = AgentPathArray()

        self.survey_marker_pub = rospy.Publisher('/multi_agent/survey_area', MarkerArray, queue_size=1)
        # self.message_pub = rospy.Publisher('/rviz_message', MarkerArray, queue_size=1)
        self.message_srv = rospy.ServiceProxy('/display_rviz_message', DisplayRvizMessage)
        rospy.wait_for_service('/display_rviz_message',timeout=5)

        self.spawn_separation = rospy.get_param('spawn_separation', 10)
        #Özer lawn mower params
        self.num_agents = self.num_auvs
        self.swath = rospy.get_param('~swath', 50)

        self.rect_width = self.num_agents*self.spawn_separation
        self.rect_height = None #Get from desired end point

        #Doesn't matter since we don't care about time domain
        self.speed = rospy.get_param('~speed', 1.0)

        self.straight_slack = rospy.get_param('~straight_slack', 1.0)
        self.overlap_between_rows = rospy.get_param('~overlap_between_rows', 0.0)
        self.overlap_between_lanes = rospy.get_param('~overlap_between_lanes', 0.0)
        self.double_sided = rospy.get_param('~double_sided', False)

        self.center_x = rospy.get_param('~center_x', False)
        self.center_y = rospy.get_param('~center_y', False)
        self.exiting_line = rospy.get_param('~exiting_line', True)

        self.message_timer = rospy.Timer(rospy.Duration(0.1), self.message_timer_cb)

        self.dubins_turning_radius = rospy.get_param('dubins_turning_radius', 5)
        
        rospy.spin()


    def message_timer_cb(self, event):
        if self.bottom_left is None:
            self.display_message_in_rviz("Choose bottom left corner using 2D Nav Goal tool...")
        elif self.top_right is None:
            self.display_message_in_rviz("Choose top right corner using 2D Nav Goal tool...")
        else:
            self.display_message_in_rviz("")
            self.message_timer.shutdown()

    def goal_cb(self, msg):
        # rospy.loginfo("Received goal")
        # self.rect_height = self.distance_hugin_0_to_goal(msg)
        # print("rect_height: ", self.rect_height)
        # self.generate_lawn_mower_pattern()
        if self.bottom_left is None:
            self.bottom_left = msg
            rospy.loginfo("Received bottom left corner")

        elif self.top_right is None:
            self.top_right = msg
            rospy.loginfo("Received top right corner")
            self.publish_survey_area()
            self.generate_lawn_mower_pattern()
        else:
            rospy.loginfo("Survey area already defined!")
    
    def publish_survey_area(self):
        """Publishes a marker representing the survey area as a rectangle"""
        # Create marker array
        marker_array = MarkerArray()

        # Create a line strip marker for the rectangle
        marker_rect = Marker()
        marker_rect.header.frame_id = "map"
        marker_rect.header.stamp = rospy.Time.now()
        marker_rect.type = Marker.LINE_STRIP
        marker_rect.id = 0
        marker_rect.pose.orientation = Quaternion(0, 0, 0, 1)  # Set the quaternion for orientation

        # Set the scale of the marker (line width)
        marker_rect.scale.x = 1.0  # You can adjust this value

        # Set the color (blue, fully opaque)
        marker_rect.color.r = 0.0
        marker_rect.color.g = 0.0
        marker_rect.color.b = 1.0
        marker_rect.color.a = 1.0

        # Add points to the line strip marker to define the rectangle
        # Assuming that bottom_left and top_right are PoseStamped objects
        points = [
                self.bottom_left.pose.position,
                Point(self.top_right.pose.position.x, self.bottom_left.pose.position.y, 0.0),
                self.top_right.pose.position,
                Point(self.bottom_left.pose.position.x, self.top_right.pose.position.y, 0.0),
                self.bottom_left.pose.position  # Closing the loop
                ]

        # Append points to the marker
        for point in points:
            marker_point = Point()
            marker_point.x = point.x
            marker_point.y = point.y
            marker_point.z = point.z
            marker_rect.points.append(marker_point)

        # Add the rectangle marker to the marker array
        marker_array.markers.append(marker_rect)

        # Publish the marker array
        self.survey_marker_pub.publish(marker_array)
    
    def display_message_in_rviz(self, message):
        self.message_srv(DisplayRvizMessageRequest(String(message)))

    # def display_message_in_rviz(self, message):
    #     """Publishes a message to rviz"""
    #     # Create marker array
    #     marker_array = MarkerArray()

    #     # Create a line strip marker for the rectangle
    #     marker_text = Marker()
    #     marker_text.header.frame_id = "map"
    #     marker_text.header.stamp = rospy.Time.now()
    #     marker_text.type = Marker.TEXT_VIEW_FACING
    #     marker_text.id = 0
    #     marker_text.pose.orientation = Quaternion(0, 0, 0, 1)
    #     marker_text.pose.position = Point(0, 0, 0)  # Set the position of the marker
    #     marker_text.scale.z = 10.0  # Set the scale of the marker (text height)
    #     marker_text.color.r = 1.0  # Set the color (blue, fully opaque)
    #     marker_text.color.g = 1.0
    #     marker_text.color.b = 1.0
    #     marker_text.color.a = 1.0
    #     marker_text.text = message  # Set the text of the marker

    #     # Add the marker to the marker array
    #     marker_array.markers.append(marker_text)

    #     # Publish the marker array
    #     self.message_pub.publish(marker_array)


    
    def distance_hugin_0_to_goal(self, goal):
        """Calculate the distance from Hugin 0 to the goal, using the tf tree"""
        try:
            # # Lookup the transform from hugin_0/base_link to map frame
            # transform = self.tf_buffer.lookup_transform("map", "hugin_0/base_link", rospy.Time.now(), rospy.Duration(1.0))

            # # Transform the goal pose into the base_link frame
            # transformed_goal = do_transform_pose(goal, transform)

            # # Calculate the absolute distance
            # distance = ((transformed_goal.pose.position.x ** 2) +
            #             (transformed_goal.pose.position.y ** 2) +
            #             (transformed_goal.pose.position.z ** 2)) ** 0.5

            distance = ((goal.pose.position.x ** 2) +
                        (goal.pose.position.y ** 2) +
                        (goal.pose.position.z ** 2)) ** 0.5
            

            return distance
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF transform error: %s", str(e))
            return None
    
    def calc_rect_height_N_width(self):
        """Calculate the height and width of the rectangle"""
        # Calculate the height of the rectangle
        self.rect_height = abs(self.bottom_left.pose.position.y - self.top_right.pose.position.y)

        # Calculate the width of the rectangle
        self.rect_width = abs(self.bottom_left.pose.position.x - self.top_right.pose.position.x)

    def generate_lawn_mower_pattern(self):
        self.calc_rect_height_N_width()
        #Generate a lawn mower pattern
        timed_paths_list = mission_plan.plan_simple_lawnmower(
        num_agents=self.num_agents,
        swath=self.swath,
        rect_width=self.rect_width,
        rect_height=self.rect_height,
        speed=self.speed,
        straight_slack=self.straight_slack,
        overlap_between_rows=self.overlap_between_rows,
        overlap_between_lanes=self.overlap_between_lanes,
        double_sided=self.double_sided,
        center_x=self.center_x,
        center_y=self.center_y,
        exiting_line=self.exiting_line
        )

        # Visualization
        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal')

        
        # Transform waypoints for agent 0
        timed_paths_list = self.transform_waypoints(timed_paths_list)
        # Publish waypoints as Path messages for each AUV
        for agent_idx, timed_path in enumerate(timed_paths_list):
            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = "map"  # Assuming the waypoints are in the map frame

            # Create PoseStamped messages for each waypoint
            for wp_id, waypoint in enumerate(timed_path.wps):
                # if wp_id == 0:
                #     continue #Don't add the first two waypoints to the path, such that the auvs start in a stright path
                pose_stamped = PoseStamped()
                pose_stamped.header = path_msg.header
                pose_stamped.pose.position.x = waypoint.pose[0]
                pose_stamped.pose.position.y = waypoint.pose[1]
                # if wp_id == 1:
                #     heading = np.pi/2 #All start looking forward
                # else:
                #     heading = waypoint.pose[2]
                heading = waypoint.pose[2]
                # Convert heading to quaternion
                quaternion = tf.transformations.quaternion_from_euler(0, 0, heading)

                # Add the quaternion to pose_stamped
                pose_stamped.pose.orientation = Quaternion(*quaternion)

                path_msg.poses.append(pose_stamped)

            # Publish the Path message for this agent
            agent_path = AgentPath()
            agent_path.agent_id = (self.num_agents-1) - agent_idx  # Reverse the order of the generated paths to match the order of the spawned AUVs
            agent_path.path = path_msg
            self.paths.path_array.append(agent_path)

            timed_path.visualize(ax, wp_labels=False, circles=True, alpha=0.1, c='k') #Uncomment to plot the paths in separate window

        # Publish AgentPathArray containing all agent paths
        self.paths.header.stamp = rospy.Time.now()
        self.paths.header.frame_id = "map"  # Assuming the paths are in the map frame
        self.spawn_pos_path_array_pub.publish(self.paths)
        print("Published paths")

        plt.show()

        # rospy.loginfo("Publishing AgentPathArray")
        # self.path_array_pub.publish(self.paths)
        #Reset
        self.paths = AgentPathArray()
        self.goal = None

    def transform_waypoints(self, timed_paths_list):
        # Check if self.bottom_left is defined
        if self.bottom_left is None:
            rospy.logwarn("Bottom left corner is not defined.")
            return timed_paths_list  # Return the original list

        # Calculate the translation vector
        if self.num_auvs % 2 == 0:
            left_most_index = 0
        else:
            left_most_index = 1
        translation_x = self.bottom_left.pose.position.x - timed_paths_list[-1].wps[left_most_index].pose[0] #the order of the agent ids is reversed in the timed_paths_list, so take the last one to get the waypoints of the first auv
        translation_y = self.bottom_left.pose.position.y - timed_paths_list[-1].wps[left_most_index].pose[1]

        # Transform waypoints for agent 0
        for timed_path in timed_paths_list:
            for wp_id,waypoint in enumerate(timed_path.wps):
                waypoint.pose[0] += translation_x
                waypoint.pose[1] += translation_y
                # #if the wp_id is 0 or 1, remove them from timed_path.wps
                # if wp_id == 0 or wp_id == 1:
                #     del timed_path.wps[wp_id]
        return timed_paths_list
    
    # def exclude_wps_within_turning_radius(self, timed_path):
    #     """This method removes all waypoints that are within the turning radius of the start and end point of the path"""
    #     def _remove_wps_close_to(wp,distance,path):
    #         """This method removes all waypoints that are within distance of wp from path"""
    #         for wp_id, waypoint in enumerate(path.wps):
    #             if np.linalg.norm(np.array(wp.pose[:2]) - np.array(waypoint.pose[:2])) < distance:
    #                 del path.wps[wp_id]
    #         #NOTE(Koray): Here you can optimize by breaking the loop once you've found the first wp that is within distance of wp, but since we also want to compare to the last wp you need more code to make it happen.
    #         #               I prefer to have short code, this doesn't run with high frequency anyway.
    #         return path

    #     wp_start = timed_path.wps[0]
    #     wp_end = timed_path.wps[-1]
    #     timed_path = _remove_wps_close_to(wp_start,self.dubins_turning_radius,timed_path)
    #     timed_path = _remove_wps_close_to(wp_end,self.dubins_turning_radius,timed_path)
    #     return timed_path
    

if __name__ == '__main__':

    rospy.init_node('pattern_generator')

    try:
        launcher = PatternGenerator()


    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch pattern_generator node")
