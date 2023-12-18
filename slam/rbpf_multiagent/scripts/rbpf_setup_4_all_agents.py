#!/usr/bin/env python3
import rospy
from subprocess import call, Popen
import numpy as np
import rospkg
import time
import tf

class RbpfSetup():
    def __init__(self):
        self.num_auvs = rospy.get_param('num_auvs',1)
        self.vehicle_model = rospy.get_param('vehicle_model','hugin')
        rospack = rospkg.RosPack()
        self.launch_file = rospy.get_param('~navigation_launch_file',rospack.get_path('rbpf_multiagent') + '/launch/rbpf_multi.launch')
        self.particle_count = rospy.get_param('~particle_count',10)
        self.particle_count_neighbours = rospy.get_param('~particle_count_neighbours',5)
        self.num_particle_handlers = rospy.get_param('~num_particle_handlers',1)
        self.results_path = rospy.get_param('~results_path','/home/kurreman/Downloads/rbpf_test"')
        self.mode = rospy.get_param('~mode','sim')
        self.rbpf_sensor_FLS = rospy.get_param('~rbpf_sensor_FLS',True)
        self.rbpf_sensor_MBES = rospy.get_param('~rbpf_sensor_MBES',False)
        self.survey_area_topic = rospy.get_param('survey_area_topic', '/multi_agent/survey_area')
        self.max_throttle = rospy.get_param('~max_throttle')
        self.comms_type = rospy.get_param('~comms_type',"disabled")
        self.i = 0
        self.init_covariance = rospy.get_param('~init_covariance')
        self.motion_covariance = rospy.get_param('~motion_covariance')
        self.resampling_noise_covariance = rospy.get_param('~resampling_noise_covariance')
        self.fls_range_std = rospy.get_param('~fls_range_std')
        self.fls_angle_std = rospy.get_param('~fls_angle_std')
        self.particle_spread_std_factor = rospy.get_param('~particle_spread_std_factor')

        rospy.loginfo("Setting up AUV RBPF SLAM...")

        listener = tf.TransformListener()
        auvs_spawned = False
        pr = False
        while not auvs_spawned:
            if not pr:
                rospy.loginfo("Waiting for AUVS to spawn...")
                pr = True
            try:
                listener.lookupTransform(
                    "hugin_%d/base_link" % (self.num_auvs-1), "hugin_%d/base_link" % (self.num_auvs-2), rospy.Time(0))
                auvs_spawned = True
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


        # self.timer = rospy.Timer(rospy.Duration(3.0), self.cb)
        t = time.time()
        for i in range(self.num_auvs):
            rospy.loginfo(str("Setting up RBPF SLAM for auv: "+ str(i)))
            namespace = self.vehicle_model + '_' + str(i)
            # color_r = np.random.uniform(0.5,1.0)
            # color_g = np.random.uniform(0.5,1.0)
            # color_b = np.random.uniform(0.5,1.0)
            # rospy.loginfo("particle color for auv: " + str(i) + " is: " + str(particle_viz_color))
            proc = Popen(["roslaunch", self.launch_file, 
                            "namespace:=" + namespace,
                            "particle_count:=" + str(self.particle_count),
                            "particle_count_neighbours:=" + str(self.particle_count_neighbours),
                            "num_particle_handlers:=" + str(self.num_particle_handlers),
                            "results_path:=" + self.results_path,
                            "mode:=" + self.mode,
                            "rbpf_sensor_FLS:=" + str(self.rbpf_sensor_FLS),
                            "rbpf_sensor_MBES:=" + str(self.rbpf_sensor_MBES),
                            "survey_area_topic:=" + self.survey_area_topic,
                            "num_auvs:=" + str(self.num_auvs),
                            "vehicle_model:=" + self.vehicle_model,
                            "max_throttle:=" + str(self.max_throttle),
                            "comms_type:=" + str(self.comms_type),
                            "init_covariance:=" + str(self.init_covariance),
                            "motion_covariance:=" + str(self.motion_covariance),
                            "resampling_noise_covariance:=" + str(self.resampling_noise_covariance),
                            "fls_range_std:=" + str(self.fls_range_std),
                            "fls_angle_std:=" + str(self.fls_angle_std),
                            "particle_spread_std_factor:=" + str(self.particle_spread_std_factor),
                          ])
            
            while time.time() - t < 2:
                # print("waiting for %d seconds" % (2 - (time.time() - t)))
                pass
            t = time.time()
            # rospy.sleep(3)

        rospy.spin()
    # def cb(self,event):
    #     i = self.i
    #     if i<self.num_auvs:
    #         rospy.loginfo(str("Setting up RBPF SLAM for auv: "+ str(i)))
    #         namespace = self.vehicle_model + '_' + str(i)
    #         # color_r = np.random.uniform(0.5,1.0)
    #         # color_g = np.random.uniform(0.5,1.0)
    #         # color_b = np.random.uniform(0.5,1.0)
    #         # rospy.loginfo("particle color for auv: " + str(i) + " is: " + str(particle_viz_color))
    #         proc = Popen(["roslaunch", self.launch_file, 
    #                         "namespace:=" + namespace,
    #                         "particle_count:=" + str(self.particle_count),
    #                         "num_particle_handlers:=" + str(self.num_particle_handlers),
    #                         "results_path:=" + self.results_path,
    #                         "mode:=" + self.mode,
    #                         "rbpf_sensor_FLS:=" + str(self.rbpf_sensor_FLS),
    #                         "rbpf_sensor_MBES:=" + str(self.rbpf_sensor_MBES),
    #                         ])
    #         self.i  += 1
    #     else:
    #         rospy.loginfo("Shutting down rbpf_slam_setup timer")
    #         self.timer.shutdown()
if __name__ == '__main__':

    rospy.init_node('rbpf_slam_setup')

    try:
        launcher = RbpfSetup()

    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch rbpf_slam_setup node")
