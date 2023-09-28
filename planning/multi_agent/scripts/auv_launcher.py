#!/usr/bin/env python3
import rospy
from subprocess import call, Popen
import numpy as np
import rospkg

class auv_launcher():
    def __init__(self):
        self.num_auvs = rospy.get_param('num_auvs',1)
        self.mode = rospy.get_param('~mode','sim')
        self.dataset = rospy.get_param('~dataset','lost_targets')
        self.vehicle_model = rospy.get_param('~vehicle_model','hugin')
        self.spawn_sep = rospy.get_param('~spawn_separation',10)
        rospack = rospkg.RosPack()
        self.launch_file = rospy.get_param('~auv_launch_file',rospack.get_path('auv_model') + '/launch/auv_environment.launch')
        

        rospy.loginfo(str("Preparing to launch '%s' auvs..." % str(self.num_auvs)))

        for i in range(self.num_auvs):
            rospy.loginfo(str("Launching auv: "+ str(i)))
            namespace = self.vehicle_model + '_' + str(i)
            y = i*self.spawn_sep
            proc = Popen(["roslaunch", self.launch_file, 
                          "mode:=" + self.mode,
                          "dataset:=" + self.dataset,
                          "namespace:=" + namespace,
                          "y:=" + str(y),
                          ])
            
            # rospy.sleep(3)

        rospy.spin()

if __name__ == '__main__':

    rospy.init_node('auv_launcher')

    try:
        launcher = auv_launcher()

    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch auv_launcher node")
