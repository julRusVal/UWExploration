#!/usr/bin/env python3
import rospy
from subprocess import call, Popen
import numpy as np
import rospkg
from multi_agent.msg import AgentPath, AgentPathArray
from nav_msgs.msg import Path
class PathRelay():
    def __init__(self):
        self.num_auvs = rospy.get_param('num_auvs',1)
        self.vehicle_model = rospy.get_param('vehicle_model','hugin')

        self.path_array_topic = rospy.get_param('path_array_topic', '/multi_agent/path_array')
        self.paths_sub = rospy.Subscriber(self.path_array_topic, AgentPathArray, self.path_array_cb)
        self.paths = AgentPathArray()
        self.pub_dict = {}

        # rospy.loginfo("Enabling AUV navigation...")

        #Create publishers for each AUV
        for i in range(self.num_auvs):
            namespace = self.vehicle_model + '_' + str(i)
            self.pub_dict[i] = rospy.Publisher(namespace + '/waypoints', Path, queue_size=1)

        while not rospy.is_shutdown():
            rate = 10 #Hz
            rate = rospy.Rate(rate)
            while len(self.paths.path_array) > 0:
                AgentPath_instance = self.paths.path_array.pop()
                agent_path = AgentPath_instance.path
                agent_id = AgentPath_instance.agent_id
                self.pub_dict[agent_id].publish(agent_path)
                rospy.loginfo(str("Published path for agent: " + str(agent_id)))
                rate.sleep()
        

    def path_array_cb(self, msg):
        rospy.loginfo("Received AgentPathArray")
        self.paths = msg

if __name__ == '__main__':

    rospy.init_node('path_relay')

    try:
        launcher = PathRelay()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch path_relay node")
