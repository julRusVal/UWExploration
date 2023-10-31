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
        self.max_thrust = rospy.get_param('~max_thrust', 5)
        self.max_throttle = rospy.get_param('~max_throttle', 4)
        self.wp_follower_type = rospy.get_param('~waypoint_follower_type', 'dubins_smarc')
        self.dubins_step_size = rospy.get_param('~dubins_step_size', 0.5)
        self.dubins_turning_radius = rospy.get_param('dubins_turning_radius', 5)
        self.goal_tolerance = rospy.get_param('~goal_tolerance', 1.)
        self.time_sync = rospy.get_param('~time_sync', 'false')

        

        rospy.loginfo("Enabling AUV navigation...")

        for i in range(self.num_auvs):
            rospy.loginfo(str("Enabling navigation for auv: "+ str(i)))
            namespace = self.vehicle_model + '_' + str(i)
            proc = Popen(["roslaunch", self.launch_file, 
                            "manual_control:=" + str(self.manual_control),
                            "namespace:=" + namespace,
                            "max_thrust:=" + str(self.max_thrust),
                            "max_throttle:=" + str(self.max_throttle),
                            "waypoint_follower_type:=" + str(self.wp_follower_type),
                            "dubins_step_size:=" + str(self.dubins_step_size),
                            "dubins_turning_radius:=" + str(self.dubins_turning_radius),
                            "goal_tolerance:=" + str(self.goal_tolerance),
                            "time_sync:=" + str(self.time_sync),
                          ])
            
            # rospy.sleep(3)

        rospy.spin()

if __name__ == '__main__':

    rospy.init_node('auv_navigation')

    try:
        launcher = AUVNavigation()

    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch auv_navigation node")
