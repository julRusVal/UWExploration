#!/usr/bin/env python3
import rospy
from subprocess import call, Popen
import numpy as np
import rospkg
import math
import pdb

class AUVSpawner():
    def __init__(self):
        self.num_auvs = rospy.get_param('num_auvs',1)
        self.mode = rospy.get_param('~mode','sim')
        self.dataset = rospy.get_param('~dataset','lost_targets')
        self.vehicle_model = rospy.get_param('vehicle_model','hugin')
        self.spawn_sep = rospy.get_param('spawn_separation',10)
        rospack = rospkg.RosPack()
        self.launch_file = rospy.get_param('~auv_launch_file',rospack.get_path('auv_model') + '/launch/auv_environment.launch')
        

        rospy.loginfo(str("Preparing to spawn '%s' AUVs..." % str(self.num_auvs)))

        #spawn_points should be an array consisting of evenly spaced points along the x-axis with 2*self.spawn_sep spacing
        # spacing = 2*self.spawn_sep
        # elements = self.num_auvs-2
        # spawn_points = np.linspace(0, self.num_auvs*self.spawn_sep, elements, endpoint=True)
        # print(spawn_points)
        
        for i in range(self.num_auvs):
            rospy.loginfo(str("Spawning AUV: "+ str(i)))
            namespace = self.vehicle_model + '_' + str(i)

            


            x = i*self.spawn_sep

            proc = Popen(["roslaunch", self.launch_file, 
                          "mode:=" + self.mode,
                          "dataset:=" + self.dataset,
                          "namespace:=" + namespace,
                          "x:=" + str(x), #This and yaw below are for the initial pose, such that the auvs are spawned along the x-axis heading looking along the y-axis
                          "yaw:=" + str(math.pi/2),
                          ])
            
            # rospy.sleep(3)

        rospy.spin()

if __name__ == '__main__':

    rospy.init_node('auv_spawner')

    try:
        launcher = AUVSpawner()

    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch auv_spawner node")
