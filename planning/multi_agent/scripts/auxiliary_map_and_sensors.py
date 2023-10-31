#!/usr/bin/env python3
import rospy
from subprocess import call, Popen
import numpy as np
import rospkg

class LaunchAuxiliary():
    def __init__(self):
        self.num_auvs = rospy.get_param('num_auvs',1)
        self.vehicle_model = rospy.get_param('vehicle_model','hugin')
        rospack = rospkg.RosPack()
        self.launch_file = rospy.get_param('~auxiliary_launch_file',rospack.get_path('auv_model') + '/launch/auv_env_aux.launch')

        self.dataset = rospy.get_param('~dataset','lost_targets')

        rospy.loginfo("Loading dataset and submaps for each AUV...")

        for i in range(self.num_auvs):
            rospy.loginfo(str("Enabling auxiliary nodes for AUV: "+ str(i)))
            namespace = self.vehicle_model + '_' + str(i)
            if i==0:
                load_dataset = 'true' #Only load the ground truth dataset once
            else:
                load_dataset = 'false'
            proc = Popen(["roslaunch", self.launch_file, 
                            "namespace:=" + namespace,
                            "dataset:=" + str(self.dataset),
                            "load_dataset:=" + str(load_dataset),
                          ])
            
            # rospy.sleep(3)

        rospy.spin()

if __name__ == '__main__':

    rospy.init_node('auxiliary_map_and_sensors')

    try:
        launcher = LaunchAuxiliary()

    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch auxiliary_map_and_sensors node")
