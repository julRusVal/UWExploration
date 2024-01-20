#!/usr/bin/env python3

import rospy
import roslaunch
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose
# import os
import numpy as np
from pathlib import Path
import time
from pynput.keyboard import Key, Controller
from stats_from_multiple_csvs import gen_stats
import os
import subprocess
class experiments_loop(object):

    def __init__(self):

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        # For future Koray: now that you have 2+ RBPFs running (congrats!) you'll have to adapt this cb 
        # to run your experiments
        # finished_top = rospy.get_param("~rbpf_saved_top", "/gt/rbpf_saved")
        self.synch_pub_0 = rospy.Subscriber('/finished/hugin_0', Bool, self.synch_cb)
        self.synch_pub_1 = rospy.Subscriber('/finished/hugin_1', Bool, self.synch_cb)
        self.finished = False
        # dataset = "overnight_2020"
        # particle_count = 1
        # num_particle_handlers = 1
        # path = "/media/orin/Seagate Expansion Drive/rbpf_results/hugin/"
        path = "/home/kurreman/catkin_ws/src/UWExploration/utils/plot_generator/data_collection"
        test_run_date_id = time.strftime("%Y%m%d_%H%M%S")
        num_auvs = '2'
        self.num_auvs = int(num_auvs)
        time_sync = 'true'
        save_plots = 'true'
        animate_plots = 'false'
        auxiliary_enabled = 'false' #true is standard, false to save computation power for large simulation runs
        mbes_meas_period = '100' #0.1 is standard, 100 to save computation power for large simulation runs
        rbpf_sensor_FLS = "false"
        comms_type ="disabled" # <!-- 'disabled', 'realistic', 'unlimited' -->
        weight_slicing="all" # <!-- 'all', 'top' -->
        pmp="poly" # <!-- particle marital policy: 'poly', 'mono' -->
        # motion_cov_list = [1e-5, 1e-6, 1e-7]
        # resampling_cov_list = [10, 1, 0.1]
        # fls_range_std_list = [1e-2, 1e-3, 1e-4]
        # fls_angle_std_list = [np.deg2rad(1), np.deg2rad(0.1), np.deg2rad(0.01)]

        motion_cov_list = [1e-5]
        resampling_cov_list = [0.1]
        fls_range_std_list = [0.0001]
        fls_angle_std_list = [np.deg2rad(0.01)]

        self.finished_flags_received = 0
        self.t_first_finished = None

        self.survey_area_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=1)

        left_corner_pose = PoseStamped()
        left_corner_pose.header.frame_id = "map"
        left_corner_pose.pose.position.x = -50-200
        left_corner_pose.pose.position.y = -10-80

        right_corner_pose = PoseStamped()
        right_corner_pose.header.frame_id = "map"
        right_corner_pose.pose.position.x = 154-200
        right_corner_pose.pose.position.y = 57-80

        # params = np.meshgrid(motion_cov_list, resampling_cov_list, fls_range_std_list, fls_angle_std_list)
        # print(params)

        # N_params = param_list.shape[0]
        N_tests = len(motion_cov_list)*len(resampling_cov_list)*len(fls_range_std_list)*len(fls_angle_std_list)
        test_i = 0
        # N_retests = 5
        N_retests = 5
        keyboard = Controller()

        self.timer = rospy.Timer(rospy.Duration(1.0), self.cb)
        # ###
        # motion_cov_list = motion_cov_list[:2]
        # resampling_cov_list = resampling_cov_list[:2]
        # fls_range_std_list = fls_range_std_list[:2]
        # fls_angle_std_list = fls_angle_std_list[:2]
        # N_retests = 2
        # ###
        for motion_cov in motion_cov_list:
            for res_cov in resampling_cov_list:
                for fls_range_std in fls_range_std_list:
                    for fls_angle_std in fls_angle_std_list:
                        test_i += 1
                        for t in range(N_retests):
                            # print(motion_cov, res_cov, fls_range_std, fls_angle_std)
                            # Path(path + str(i)).mkdir(parents=True, exist_ok=True)
                            cli_args = ['/home/kurreman/catkin_ws/src/UWExploration/planning/multi_agent/launch/multi_agent.launch', 
                                        'num_auvs:=' + num_auvs, 
                                        'time_sync:=' + time_sync, 
                                        'save_final_plots:=' + save_plots, 
                                        'animate_plots:=' + animate_plots, 
                                        'motion_covariance:=' + '[0.0, 0.0, 0.0, 0.0, 0.0, %s]' % (str(format(motion_cov,'.9f'))),
                                        'resampling_noise_covariance:=' + '[%s, %s, 0.0, 0.0, 0.0, 0.0]' % (str(format(res_cov,'.9f')), str(format(res_cov,'.9f'))),
                                        'fls_range_std:=' + str(format(fls_range_std,'.9f')),
                                        'fls_angle_std:=' + str(format(fls_angle_std,'.9f')),
                                        'plots_results_path:=' + path + "/test_run_" + test_run_date_id + "/" + "my" + str(motion_cov) + "_rxy" + str(res_cov) + "_fr" + str(fls_range_std) + "_fa" + str(fls_angle_std),
                                        'record_launch_parameters_and_arguments:=true',
                                        "mbes_meas_period:=" + mbes_meas_period,
                                        "auxiliary_enabled:="+auxiliary_enabled,
                                        "rbpf_sensor_FLS:="+rbpf_sensor_FLS,
                                        "comms_type:="+comms_type,
                                        "weight_slicing:="+weight_slicing,
                                        "pmp:="+pmp
                                        ]
                            
                            roslaunch_args = cli_args[1:]
                            roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], 
                                                roslaunch_args)]

                            parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_file)
                            # rospy.logfatal("Launching test ", test_i, "of ", N_tests)
                            # rospy.logfatal("Iteration ", t+1, "of ", N_retests)
                            rospy.logfatal("Launching test {} of {}".format(test_i, N_tests))
                            rospy.logfatal("Iteration {} of {}".format(t+1, N_retests))
                            parent.start()

                            rospy.sleep(1)
                            left_corner_pose.header.stamp = rospy.Time.now()
                            self.survey_area_pub.publish(left_corner_pose)
                            rospy.sleep(1)
                            right_corner_pose.header.stamp = rospy.Time.now()
                            self.survey_area_pub.publish(right_corner_pose)
                            rospy.sleep(10)

                            # # Define the command to execute rosparam dump
                            # folder_name = path + "/test_run_" + test_run_date_id
                            # filename = "rosparams.yaml"
                            # command = ["rosparam", "dump"]
                            # command.append(folder_name+"/"+filename)

                            # #if the file doesn't exists
                            # if not os.path.isfile(folder_name+"/"+filename):
                            #     try:
                            #         subprocess.run(command, check=True)
                            #         print("Parameters dumped successfully.")
                            #     except subprocess.CalledProcessError as e:
                            #         print(f"An error occurred while dumping parameters: {e}")
                            # else:
                            #     print("Parameters already dumped.")

                            #TODO. press Enter
                            # subprocess.run(["xdotool", "key", "Return"])
                            # keyboard.send('enter')
                            # keyboard.type('Enter')
                            while not rospy.is_shutdown() and not self.finished:
                                rospy.sleep(1)

                            # rospy.logfatal("Shutting down test ", test_i, "of ", N_tests)
                            # rospy.logfatal("Iteration ", t+1, "of ", N_retests)
                            rospy.logfatal("Shutting down test {} of {}".format(test_i, N_tests))
                            rospy.logfatal("Iteration {} of {}".format(t+1, N_retests))
                            # rospy.sleep(particle_count*10)
                            parent.shutdown()
                            self.finished = False
                            rospy.sleep(5)
                        gen_stats(path + "/test_run_" + test_run_date_id + "/" + "my" + str(motion_cov) + "_rxy" + str(res_cov) + "_fr" + str(fls_range_std) + "_fa" + str(fls_angle_std))
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
    
    def cb(self, event):
        if self.finished_flags_received != 0:
            if self.t_first_finished is None:
                self.t_first_finished = time.time()
            elif time.time() - self.t_first_finished < 20:
                rospy.logfatal("Waiting for all AUVs to finish...")
                rospy.logfatal("Finished flags received: {}".format(self.finished_flags_received))
                rospy.logfatal("Time left: {}".format(20 - (time.time() - self.t_first_finished)))
            else:
                self.finished = True
                self.finished_flags_received = 0
                self.t_first_finished = None
        


if __name__ == '__main__':

    rospy.init_node('experiments_loop_node', disable_signals=False, anonymous=True)
    try:
        experiments_loop()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not launch experiments')
