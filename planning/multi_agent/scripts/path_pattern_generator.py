#!/usr/bin/env python3
import rospy
from subprocess import call, Popen
import numpy as np
import rospkg
from multi_agent.msg import AgentPath, AgentPathArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
from coop_cov import mission_plan
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class PatternGenerator():
    def __init__(self):
        # Initialize the TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.num_auvs = rospy.get_param('num_auvs',1)
        self.vehicle_model = rospy.get_param('vehicle_model','hugin')
        
        self.paths_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        self.goal = None

        self.path_array_topic = rospy.get_param('path_array_topic', '/multi_agent/path_array')
        self.path_array_pub = rospy.Publisher(self.path_array_topic, AgentPathArray, queue_size=1)
        self.paths = AgentPathArray()

        self.spawn_separation = rospy.get_param('spawn_separation', 10)
        #Özer lawn mower params
        self.num_agents = self.num_auvs
        self.swath = rospy.get_param('swath', 50)

        self.rect_width = self.num_agents*self.spawn_separation
        self.rect_height = None #Get from desired end point

        #Doesn't matter since we don't care about time domain
        self.speed = rospy.get_param('speed', 1.0)
        self.straight_slack = rospy.get_param('straight_slack', 1.0)
        self.overlap_between_rows = rospy.get_param('overlap_between_rows', 0.0)
        self.overlap_between_lanes = rospy.get_param('overlap_between_lanes', 0.0)
        self.double_sided = rospy.get_param('double_sided', False)

        self.center_x = rospy.get_param('center_x', False)
        self.center_y = rospy.get_param('center_y', False)
        self.exiting_line = rospy.get_param('exiting_line', True)
        
        rospy.spin()

    def goal_cb(self, msg):
        rospy.loginfo("Received goal")
        self.rect_height = self.distance_hugin_0_to_goal(msg)
        print("rect_height: ", self.rect_height)
        # self.generate_lawn_mower_pattern()

    
    def distance_hugin_0_to_goal(self, goal):
        """Calculate the distance from Hugin 0 to the goal, using the tf tree"""
        try:
            # Lookup the transform from hugin_0/base_link to map frame
            transform = self.tf_buffer.lookup_transform("map", "hugin_0/base_link", rospy.Time.now(), rospy.Duration(1.0))

            # Transform the goal pose into the base_link frame
            transformed_goal = do_transform_pose(goal, transform)

            # Calculate the absolute distance
            distance = ((transformed_goal.pose.position.x ** 2) +
                        (transformed_goal.pose.position.y ** 2) +
                        (transformed_goal.pose.position.z ** 2)) ** 0.5

            return distance
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF transform error: %s", str(e))
            return None

    def generate_lawn_mower_pattern(self):
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

        # rospy.loginfo("Publishing AgentPathArray")
        # self.path_array_pub.publish(self.paths)
        # #Reset
        # self.paths = AgentPathArray()
        # self.goal = None

    


if __name__ == '__main__':

    rospy.init_node('pattern_generator')

    try:
        launcher = PatternGenerator()


    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch pattern_generator node")
