#!/usr/bin/env python3
"""
This module contains the PatternGenerator class, which is responsible for generating and publishing path patterns for multiple autonomous underwater vehicles (AUVs) in a survey area.


Classes:
    PatternGenerator: A class to generate and publish path patterns for multiple AUVs.
Functions:
    __init__(self): Initializes the PatternGenerator class.
    message_timer_cb(self, event): Callback for the message timer to display messages in RViz.
    safe_path_publishing_cb(self, event): Callback for publishing the path in a safe manner that waits for the spawner to be online.
    update_spawner_status(self): Updates the status of the spawner node.
    update_mission_planner_count(self): Updates the count of the mission planner node.
    goal_cb(self, msg): Callback for the goal subscriber.
    publish_survey_perimeter_path(self, event=None): Publishes a Path message representing the survey area as a rectangle.
    construct_perimeter_path_msg(self): Constructs the perimeter path message.
    publish_survey_area(self, event=None): Publishes a marker representing the survey area as a rectangle.
    display_message_in_rviz(self, message): Displays a message in RViz.
    pose_from_param(self, param_name): Gets a PoseStamped object from a parameter.
    distance_hugin_0_to_goal(self, goal): Calculates the distance from Hugin 0 to the goal using the tf tree.
    calc_rect_height_N_width(self): Calculates the height and width of the rectangle.
    generate_lawn_mower_pattern(self): Generates a lawn mower pattern for the survey area.
    transform_waypoints(self, timed_paths_list): Transforms waypoints based on the bottom left corner.
"""
# General imports
from subprocess import call, Popen
import numpy as np
import matplotlib.pyplot as plt
import copy

# ROS import
import rospy
import rospkg
import tf.transformations
import tf2_ros
from rviz_visualization.srv import DisplayRvizMessage, DisplayRvizMessageRequest

# ROS message imports
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Path
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String, Time, Int32MultiArray
from multi_agent.msg import AgentPath, AgentPathArray

# 
from coop_cov import mission_plan

class PatternGenerator():
    def __init__(self):
        # Initialize the TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.num_auvs = rospy.get_param('num_auvs',1)
        self.vehicle_model = rospy.get_param('vehicle_model','hugin')

        # Defines the frame id that the goals are given in, for the generated paths and the survey area
        self.default_frame_id = "map"

        # I want to allow for the user to define the bounding corners of the survey area from the yaml file
        # If they are not defined, the user can use the 2D Nav Goal tool in the RViz window to define them
        # Locally path_bottom_left = PoseStamped()
        self.path_bottom_left = PatternGenerator.pose_from_param(self,'path_bottom_left')
        self.path_top_right = PatternGenerator.pose_from_param(self, 'path_top_right')

        if self.path_bottom_left is None or self.path_top_right is None:
            # Subscribe to the goal topic
            self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)

        # Publishers used to send messages to the planner, w2w_planner for now
        self.spawn_pos_path_array_topic = rospy.get_param('path_array_spawn_pos_topic', '/multi_agent/spawn_pos/path_array')
        self.spawn_pos_path_array_pub = rospy.Publisher(self.spawn_pos_path_array_topic, AgentPathArray, queue_size=1)
        self.paths = AgentPathArray()
        self.valid_paths = False
        self.goal = None

        # Dictionary of paths for each AUV
        self.paths_dict = {} # Dictionary of paths for each AUV


        # Publishers for sending the outer perimeter (as Path) to each AUV
        self.pub_dict = {} # Dictionary of publishers for each AUV
        self.perimeter_path_msg = None
        self.perimeter_path_msg_logged = False
        for i in range(self.num_auvs):
            namespace = self.vehicle_model + '_' + str(i)
            self.pub_dict[i] = rospy.Publisher(namespace + '/gp/waypoints', Path, queue_size=1)

        rospy.Timer(rospy.Duration(1), self.publish_survey_perimeter_path)  # Publish the survey area as a Path message for gp mapping
        
        # Publisher for the common timestamps
        self.time_array_pub = rospy.Publisher('/multi_agent/common_timestamps', Int32MultiArray, queue_size=1)
        self.time_array = None
        self.valid_time_array = False

        self.survey_area_topic = rospy.get_param('survey_area_topic', '/multi_agent/survey_area')
        self.survey_marker_pub = rospy.Publisher(self.survey_area_topic, MarkerArray, queue_size=1)
        # self.message_pub = rospy.Publisher('/rviz_message', MarkerArray, queue_size=1)
        self.message_srv = rospy.ServiceProxy('/display_rviz_message', DisplayRvizMessage)
        rospy.wait_for_service('/display_rviz_message',timeout=5)

        self.spawn_separation = rospy.get_param('spawn_separation', 10)
        #Özer lawn mower params
        self.num_agents = self.num_auvs
        self.swath = rospy.get_param('~swath', 50)

        self.rect_width = self.num_agents*self.spawn_separation
        self.rect_height = None #Get from desired end point

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

        rospy.Timer(rospy.Duration(1), self.publish_survey_area)  # Publish the survey area as a rectangle for visualization
        rospy.Timer(rospy.Duration(1), self.publish_survey_perimeter_path)  # Publish the survey area as a Path message for gp mapping

        # The pattern generator should wait for the spawner to be online before generating the pattern
        self.spawner_status_param_name = rospy.get_param('spawner_status_param_name',
                                                         '/spawner_status')
        self.spawn_online = False  # This is the status of the spawner node, lack of parameter means it is offline
        self.spawner_status = False  # This is the status of the spawning, True indicates all AUVs have been spawned

        self.mission_planner_count_name = rospy.get_param('mission_planner_count_param_name',
                                                          '/mission_planner_count')
        self.mission_planner_count = -1
        self.valid_mission_planner_count = False

        self.safe_path_publishing_timer = rospy.Timer(rospy.Duration(1), self.safe_path_publishing_cb)

        rospy.loginfo(f"PATH GENERATOR:: Name: {rospy.get_name()} - Namespace: {rospy.get_namespace()}")

        # Status report to terminal
        # If the corner goals are set through the yaml file, self.generate_lawn_mower_pattern() will be called in the __init__ method
        self.status_reported = False
        if not self.status_reported:
            rospy.loginfo("Pattern Generator node is ready!")
            if self.path_bottom_left is None or self.path_top_right is None:
                rospy.loginfo("Choose bounding corners using 2D Nav Goal tool in the RViz window")
            else:
                rospy.loginfo("Bounding corners have been pre-defined")
                rospy.loginfo("Generating lawn mower pattern")

                self.generate_lawn_mower_pattern()
            self.status_reported = True

        rospy.spin()

    def message_timer_cb(self, event):
        if self.path_bottom_left is None:
            self.display_message_in_rviz("Choose bottom left corner using 2D Nav Goal tool...")
        elif self.path_top_right is None:
            self.display_message_in_rviz("Choose top right corner using 2D Nav Goal tool...")
        else:
            self.display_message_in_rviz("")
            self.message_timer.shutdown()

    def safe_path_publishing_cb(self, event):
        """
        Callback for publishing the path in a safe manner that waits for the spawner to be online.
        :return:
        """
        self.update_spawner_status()
        self.update_mission_planner_count()

        if self.spawn_online and self.valid_mission_planner_count and self.valid_paths and self.valid_time_array:

            self.spawn_pos_path_array_pub.publish(self.paths)
            rospy.loginfo("Safely published paths")
            self.time_array_pub.publish(Int32MultiArray(data=self.time_array))
            rospy.loginfo("Safely published time array")

            self.safe_path_publishing_timer.shutdown()

            # Reset the paths and time_array
            self.paths = AgentPathArray()
            self.valid_paths = False
            self.goal = None

            self.time_array = None
            self.valid_time_array = False

    def update_spawner_status(self):
        """Update the status of the spawner node"""
        if rospy.has_param(self.spawner_status_param_name):
            self.spawn_online = True
            self.spawner_status = rospy.get_param(self.spawner_status_param_name)
        else:
            self.spawn_online = False
            self.spawner_status = False

    def update_mission_planner_count(self):
        """Update the count of the mission planner node"""
        if rospy.has_param(self.mission_planner_count_name):
            self.mission_planner_count = rospy.get_param(self.mission_planner_count_name)
            self.valid_mission_planner_count = ( self.mission_planner_count == self.num_agents)

        else:
            self.mission_planner_count = -1
            self.valid_mission_planner = False

    def goal_cb(self, msg):
        """
        Callback for the goal subscriber
        :param msg: PoseStamped message1
        :return:
        """
        # rospy.loginfo("Received goal")
        # self.rect_height = self.distance_hugin_0_to_goal(msg)
        # print("rect_height: ", self.rect_height)
        # self.generate_lawn_mower_pattern()

        def debug_goal_cb(pose_msg : PoseStamped, label :str):
            x = pose_msg.pose.position.x
            y = pose_msg.pose.position.y
            z = pose_msg.pose.position.z

            rospy.loginfo(f"{label} -> Pose Coordinates: x: {x}, y: {y}, z: {z}")

        # Process the goal messages as they come in
        if self.path_bottom_left is None:
            self.path_bottom_left = msg
            debug_goal_cb(msg, "Bottom Left")

        elif self.path_top_right is None:
            self.path_top_right = msg
            debug_goal_cb(msg, "Top Right")
            # self.publish_survey_area()
            # self.generate_lawn_mower_pattern()
        else:
            rospy.loginfo("Survey area already defined!")

        if self.path_bottom_left is not None and self.path_top_right is not None:
            rospy.loginfo("Generating lawn mower pattern")
            self.generate_lawn_mower_pattern()
    
    def publish_survey_perimeter_path(self, event=None):
        """
        Publishes a Path message representing the survey area as a rectangle
        """
        if self.perimeter_path_msg is None:
            self.construct_complete_perimeter_path_msg()

        if self.perimeter_path_msg is not None:
            for publisher_id, publisher in self.pub_dict.items():
                if not self.perimeter_path_msg_logged:
                    rospy.loginfo(f"Publishing perimeter path for {publisher_id}")
                publisher.publish(self.perimeter_path_msg)
            self.perimeter_path_msg_logged = True

    def construct_agent_perimeter_path_msg(self):
        """
        Construct the perimeter path message as a rectangle using Path messages.
        This perimeter includes the entire survey area.
        """
        # TODO: Implement this method
        pass
    
    def construct_complete_perimeter_path_msg(self):
        """
        Construct the perimeter path message as a rectangle using Path messages.
        This perimeter includes the entire survey area.
        """
        if self.path_bottom_left and self.path_top_right:
            path_msg = Path()
            path_msg.header.frame_id = self.default_frame_id
            path_msg.header.stamp = rospy.Time.now()

            # Define the points of the rectangle
            points = [
                self.path_bottom_left.pose.position,
                Point(self.path_top_right.pose.position.x, self.path_bottom_left.pose.position.y, 0.0),
                self.path_top_right.pose.position,
                Point(self.path_bottom_left.pose.position.x, self.path_top_right.pose.position.y, 0.0),
                self.path_bottom_left.pose.position  # Closing the loop
            ]

            # Create PoseStamped messages for each point and add to the path
            for point in points:
                pose_stamped = PoseStamped()
                pose_stamped.header = path_msg.header
                pose_stamped.pose.position = point
                pose_stamped.pose.orientation.w = 1.0  # Default orientation
                path_msg.poses.append(pose_stamped)

            self.perimeter_path_msg = path_msg
    
    def publish_survey_area(self, event=None):
        """
        Publishes a marker representing the complete survey area as a rectangle
        """
        if self.path_bottom_left and self.path_top_right:
            # Create marker array
            marker_array = MarkerArray()

            # Create a line strip marker for the rectangle
            marker_rect = Marker()
            marker_rect.header.frame_id = self.default_frame_id
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
                    self.path_bottom_left.pose.position,
                    Point(self.path_top_right.pose.position.x, self.path_bottom_left.pose.position.y, 0.0),
                    self.path_top_right.pose.position,
                    Point(self.path_bottom_left.pose.position.x, self.path_top_right.pose.position.y, 0.0),
                    self.path_bottom_left.pose.position  # Closing the loop
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

    def pose_from_param(self, param_name):
        """
        Get a PoseStamped object from a parameter.
        The parameter should be a list of 3 elements [x, y, z].
        This will do some basic error checking and return None if the parameter is not set or is the wrong size.
        """

        param_value = rospy.get_param(param_name, None)
        if param_value is None:
            rospy.logerr(f"Parameter {param_name} is not set")
            return None

        if not isinstance(param_value, list):
            rospy.logerr(f"Parameter {param_name} is not a list")
            return None

        if len(param_value) != 3:
            rospy.logerr(f"Parameter {param_name} is not the correct size")
            return None

        pose = PoseStamped()

        pose.header.frame_id = self.default_frame_id
        pose.header.stamp = rospy.Time.now()
        pose.pose.orientation.w = 1.0

        pose.pose.position.x = param_value[0]
        pose.pose.position.y = param_value[1]
        pose.pose.position.z = param_value[2]
        return pose

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
        self.rect_height = abs(self.path_bottom_left.pose.position.y - self.path_top_right.pose.position.y)

        # Calculate the width of the rectangle
        self.rect_width = abs(self.path_bottom_left.pose.position.x - self.path_top_right.pose.position.x)

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

        # # Visualization
        # fig = plt.figure()
        # ax = fig.add_subplot(111, aspect='equal')

        
        # Transform waypoints for agent 0
        timed_paths_list = self.transform_waypoints(timed_paths_list)
        
        # Publish waypoints as Path messages for each AUV
        for agent_idx, timed_path in enumerate(timed_paths_list):
            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = self.default_frame_id # Assuming the waypoints are in the map frame
            times = []
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
                times.append(int(np.rint(waypoint.time))) #Round to nearest second
                path_msg.poses.append(pose_stamped)

            # Publish the Path message for this agent
            agent_path = AgentPath()
            agent_path.agent_id = (self.num_agents-1) - agent_idx  # Reverse the order of the generated paths to match the order of the spawned AUVs
            agent_path.path = path_msg
            self.paths.path_array.append(agent_path)
            if self.time_array is None:
                self.time_array = times
                self.valid_time_array = True
                # This is published by the safe_path_publishing_cb
                # self.time_array_pub.publish(Int32MultiArray(data=self.time_array))

            # timed_path.visualize(ax, wp_labels=False, circles=True, alpha=0.1, c='k') #Uncomment to plot the paths in separate window

        # Publish AgentPathArray containing all agent paths
        self.paths.header.stamp = rospy.Time.now()
        self.paths.header.frame_id = self.default_frame_id  # Assuming the paths are in the map frame
        self.valid_paths = True
        # self.spawn_pos_path_array_pub.publish(self.paths)
        # rospy.loginfo("Published paths")

        ## plt.show()

        # rospy.loginfo("Publishing AgentPathArray")
        # self.path_array_pub.publish(self.paths)
        #Reset
        # self.paths = AgentPathArray()
        # self.goal = None

    def transform_waypoints(self, timed_paths_list):
        # Check if self.bottom_left is defined
        if self.path_bottom_left is None:
            rospy.logwarn("Bottom left corner is not defined.")
            return timed_paths_list  # Return the original list

        # Calculate the translation vector
        if self.num_auvs % 2 == 0:
            left_most_index = 0
        else:
            left_most_index = 1
        translation_x = self.path_bottom_left.pose.position.x - timed_paths_list[-1].wps[left_most_index].pose[0] #the order of the agent ids is reversed in the timed_paths_list, so take the last one to get the waypoints of the first auv
        translation_y = self.path_bottom_left.pose.position.y - timed_paths_list[-1].wps[left_most_index].pose[1]

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
