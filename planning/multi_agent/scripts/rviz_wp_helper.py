#!/usr/bin/env python3
import rospy
from subprocess import call, Popen
import numpy as np
import rospkg
from multi_agent.msg import AgentPath, AgentPathArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose
class RvizHelper():
    def __init__(self):

        self.num_auvs = rospy.get_param('num_auvs',1)
        self.vehicle_model = rospy.get_param('vehicle_model','hugin')
        
        self.paths_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_cb)
        self.goal = None

        self.path_array_topic = rospy.get_param('path_array_topic', '/multi_agent/path_array')
        self.path_array_pub = rospy.Publisher(self.path_array_topic, AgentPathArray, queue_size=1)
        self.paths = AgentPathArray()


        # if self.goal:
        #     rospy.loginfo("2D goal exists")
        #     for i in range(self.num_auvs):
        #         rospy.loginfo("Artificially creating AgentPath for agent: " + str(i))
        #         agent = AgentPath()
        #         agent_path = Path()
        #         agent_path.header.frame_id = 'map'
        #         agent_path.header.stamp = rospy.Time.now()
        #         agent_path.poses.append(self.goal)
        #         agent.agent_id = i
        #         agent.path = agent_path
        #         self.paths.path_array.append(agent)
        #     rospy.loginfo("Publishing AgentPathArray")
        #     self.path_array_pub.publish(self.paths)
        #     self.goal = None

        rospy.spin()

        

    def goal_cb(self, msg):
        rospy.loginfo("Received goal")
        self.goal = msg
        self.generate_artificial_path()

    def generate_artificial_path(self):
        for i in range(self.num_auvs):
            rospy.loginfo("Artificially creating AgentPath for agent: " + str(i))
            agent = AgentPath()
            agent_path = Path()
            agent_path.header.frame_id = 'map'
            agent_path.header.stamp = rospy.Time.now()
            agent_path.poses.append(self.goal)
            agent.agent_id = i
            agent.path = agent_path
            self.paths.path_array.append(agent)
        rospy.loginfo("Publishing AgentPathArray")
        self.path_array_pub.publish(self.paths)
        self.goal = None


if __name__ == '__main__':

    rospy.init_node('rviz_helper')

    try:
        launcher = RvizHelper()


    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch rviz_helper node")
