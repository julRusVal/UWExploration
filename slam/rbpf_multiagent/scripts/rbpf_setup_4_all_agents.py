#!/usr/bin/env python3
import rospy
from subprocess import call, Popen
import numpy as np
import rospkg

class RbpfSetup():
    def __init__(self):
        self.num_auvs = rospy.get_param('num_auvs',1)
        self.vehicle_model = rospy.get_param('vehicle_model','hugin')
        rospack = rospkg.RosPack()
        self.launch_file = rospy.get_param('~navigation_launch_file',rospack.get_path('rbpf_multiagent') + '/launch/rbpf_multi.launch')
        self.particle_count = rospy.get_param('~particle_count',10)
        self.num_particle_handlers = rospy.get_param('~num_particle_handlers',1)
        self.results_path = rospy.get_param('~results_path','/home/kurreman/Downloads/rbpf_test"')
        self.mode = rospy.get_param('~mode','sim')
        self.i = 0
        

        rospy.loginfo("Setting up AUV RBPF SLAM...")

        self.timer = rospy.Timer(rospy.Duration(3.0), self.cb)
        
        # for i in range(self.num_auvs):
            # rospy.loginfo(str("Setting up RBPF SLAM for auv: "+ str(i)))
            # namespace = self.vehicle_model + '_' + str(i)
            # # color_r = np.random.uniform(0.5,1.0)
            # # color_g = np.random.uniform(0.5,1.0)
            # # color_b = np.random.uniform(0.5,1.0)
            # # rospy.loginfo("particle color for auv: " + str(i) + " is: " + str(particle_viz_color))
            # proc = Popen(["roslaunch", self.launch_file, 
            #                 "namespace:=" + namespace,
            #                 "particle_count:=" + str(self.particle_count),
            #                 "num_particle_handlers:=" + str(self.num_particle_handlers),
            #                 "results_path:=" + self.results_path,
            #                 "mode:=" + self.mode
            #               ])
            
            
            # rospy.sleep(3)

        rospy.spin()
    def cb(self,event):
        i = self.i
        if i<self.num_auvs:
            rospy.loginfo(str("Setting up RBPF SLAM for auv: "+ str(i)))
            namespace = self.vehicle_model + '_' + str(i)
            # color_r = np.random.uniform(0.5,1.0)
            # color_g = np.random.uniform(0.5,1.0)
            # color_b = np.random.uniform(0.5,1.0)
            # rospy.loginfo("particle color for auv: " + str(i) + " is: " + str(particle_viz_color))
            proc = Popen(["roslaunch", self.launch_file, 
                            "namespace:=" + namespace,
                            "particle_count:=" + str(self.particle_count),
                            "num_particle_handlers:=" + str(self.num_particle_handlers),
                            "results_path:=" + self.results_path,
                            "mode:=" + self.mode
                            ])
            self.i  += 1
        else:
            rospy.loginfo("Shutting down rbpf_slam_setup timer")
            self.timer.shutdown()
if __name__ == '__main__':

    rospy.init_node('rbpf_slam_setup')

    try:
        launcher = RbpfSetup()

    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch rbpf_slam_setup node")
