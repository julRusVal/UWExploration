#!/usr/bin/env python3
import rospy
from subprocess import call, Popen
import numpy as np
import rospkg

class PathRelay():
    def __init__(self):
        self.num_auvs = rospy.get_param('num_auvs',1)
        self.vehicle_model = rospy.get_param('vehicle_model','hugin')

        self.path_array_topic = rospy.get_param('~path_array_topic', '/multi_agent/path_array')
        self.path_array_sub = rospy.Subscriber(self.path_array_topic, Floats, self.path_array_cb)
        

        # rospy.loginfo("Enabling AUV navigation...")

        for i in range(self.num_auvs):
            rospy.loginfo(str("Enabling navigation for auv: "+ str(i)))
            namespace = self.vehicle_model + '_' + str(i)
            proc = Popen(["roslaunch", self.launch_file, 
                          "manual_control:=" + str(self.manual_control),
                          "namespace:=" + namespace,
                          ])
            
            # rospy.sleep(3)

        rospy.spin()

    def path_array_cb(self, msg):

        data_t = msg.data.copy().reshape(self.datagram_size,1)


if __name__ == '__main__':

    rospy.init_node('path_relay')

    try:
        launcher = PathRelay()

    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch path_relay node")
