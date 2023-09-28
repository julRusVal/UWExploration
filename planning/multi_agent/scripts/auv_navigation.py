#!/usr/bin/env python3
import rospy
from subprocess import call, Popen
import numpy as np
import rospkg

class AUVNavigation():
    def __init__(self):
        self.num_auvs = rospy.get_param('num_auvs',1)
        self.manual_control = rospy.get_param('~manual_control','false')
        self.vehicle_model = rospy.get_param('vehicle_model','hugin')
        rospack = rospkg.RosPack()
        self.launch_file = rospy.get_param('~navigation_launch_file',rospack.get_path('basic_navigation') + '/launch/basic_mission.launch')
        

        rospy.loginfo("Enabling AUV navigation...")

        for i in range(self.num_auvs):
            rospy.loginfo(str("Enabling navigation for auv: "+ str(i)))
            namespace = self.vehicle_model + '_' + str(i)
            proc = Popen(["roslaunch", self.launch_file, 
                          "manual_control:=" + str(self.manual_control),
                          "namespace:=" + namespace,
                          ])
            
            # rospy.sleep(3)

        rospy.spin()

if __name__ == '__main__':

    rospy.init_node('auv_navigation')

    try:
        launcher = AUVNavigation()

    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch auv_navigation node")
