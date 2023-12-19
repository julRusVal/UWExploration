#!/usr/bin/env python3

import rospy
import roslaunch
from std_msgs.msg import Bool
# import os
import numpy as np
from pathlib import Path
import time

class experiments_loop(object):

    def __init__(self):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # For future Koray: now that you have 2+ RBPFs running (congrats!) you'll have to adapt this cb 
        # to run your experiments
        # finished_top = rospy.get_param("~rbpf_saved_top", "/gt/rbpf_saved")
        self.synch_pub = rospy.Subscriber(finished_top, Bool, self.synch_cb)
        self.finished = False
        # dataset = "overnight_2020"
        # particle_count = 1
        # num_particle_handlers = 1
        # path = "/media/orin/Seagate Expansion Drive/rbpf_results/hugin/"
        path = "/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection"
        test_run_date_id = time.strftime("%Y%m%d_%H%M%S")
        nums_auvs = '2'
        time_sync = 'true'
        save_plots = 'true'
        animate_plots = 'false'
        
        motion_cov_list = [1e-5, 1e-6, 1e-7]
        resampling_cov_list = [10, 1, 0.1]
        fls_range_std_list = [1e-2, 1e-3, 1e-4]
        fls_angle_std_list = [np.deg2rad(1), np.deg2rad(0.1), np.deg2rad(0.01)]

        self.finished_flags_received = 0

        # params = np.meshgrid(motion_cov_list, resampling_cov_list, fls_range_std_list, fls_angle_std_list)
        # print(params)

        # N_params = param_list.shape[0]
        N_tests = len(motion_cov_list)*len(resampling_cov_list)*len(fls_range_std_list)*len(fls_angle_std_list)
        test_i = 0
        N_retests = 5
        for motion_cov in motion_cov_list:
            for res_cov in resampling_cov_list:
                for fls_range_std in fls_range_std_list:
                    for fls_angle_std in fls_angle_std_list:
                        test_i += 1
                        for t in range(N_retests):
                            # Path(path + str(i)).mkdir(parents=True, exist_ok=True)
                            cli_args = ['/home/kurreman/catkin_ws/src/UWExploration/planning/multi_agent/launch/multi_agent.launch', 
                                        'num_auvs:=' + nums_auvs, 
                                        'time_sync:=' + time_sync, 
                                        'save_final_plots:=' + save_plots, 
                                        'animate_plots:=' + animate_plots, 
                                        'motion_covariance:=' + '[0.0, 0.0, 0.0, 0.0, 0.0, %s]' % (str(motion_cov)),
                                        'resampling_noise_covariance:=' + '[%s, %s, 0.0, 0.0, 0.0, 0.0]' % (str(res_cov), str(res_cov)), 
                                        'fls_range_std:=' + str(fls_range_std), 
                                        'fls_angle_std:=' + str(fls_angle_std), 
                                        'plots_results_path:=' + path + "/test_run_" + test_run_date_id + "/" + "my" + str(motion_cov) + "_rxy" + str(res_cov) + "_fr" + str(fls_range_std) + "_fa" + str(fls_angle_std),
                                        ]
                            
                            roslaunch_args = cli_args[1:]
                            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], 
                                                roslaunch_args)]

                            parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                            print("Launching test ", test_i)
                            parent.start()

                            while not rospy.is_shutdown() and not self.finished:
                                rospy.sleep(1)

                            print("Shutting down test ", test_i, "of ", N_tests)
                            # rospy.sleep(particle_count*10)
                            parent.shutdown()
                            self.finished = False
                            rospy.sleep(5)

        # duration = 2  # seconds
        # freq = 340  # Hz
        # os.system('play -nq -t alsa synth {} sine {}'.format(duration, freq))

    def synch_cb(self, finished_msg):
        # self.finished = True
        if finished_msg.data:
            self.finished_flags_received += 1
            if self.finished_flags_received == self.num_auvs:
                self.finished = True
                self.finished_flags_received = 0
            


if __name__ == '__main__':

    rospy.init_node('experiments_loop_node', disable_signals=False, anonymous=True)
    try:
        experiments_loop()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not launch experiments')
