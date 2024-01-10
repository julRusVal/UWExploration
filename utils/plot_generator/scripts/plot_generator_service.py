#!/usr/bin/env python3

import rospy 
# from rviz_visualization.srv import DisplayRvizMessage, DisplayRvizMessageResponse
# from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from plot_generator.srv import PlotGenerator, PlotGeneratorResponse
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import tf
import io
import os
import time
from std_msgs.msg import Bool
import csv
import subprocess

class PlotGeneratorService:
    def __init__(self):
        self.service = rospy.Service('/plot_generator', PlotGenerator, self.callback)
        rospy.loginfo("Plot Generator service initialized")
        self.save_final_plots = rospy.get_param('~save_final_plots',False)
        self.results_path = rospy.get_param('~results_path','/home/kurreman/Documents/data_collection"')
        self.results_path = self.results_path.replace("[", "").replace("]", "")
        self.record_params = rospy.get_param('~record_launch_parameters_and_arguments',False)
        self.plot_instances_dict = {}

        if self.save_final_plots:
            #Create a folder in for the results with the current time as the name using os
            folder_name = time.strftime("%Y%m%d_%H%M%S")
            # Get the path of the current script
            # script_path = os.path.dirname(os.path.abspath(__file__))
            # Create a new directory path by joining the script path with the new folder name
            # new_folder_path = os.path.join(script_path, folder_name)
            new_folder_path = os.path.join(self.results_path, folder_name)

            # Check if the directory already exists
            if not os.path.exists(new_folder_path):
                # Create the directory if it doesn't exist
                os.makedirs(new_folder_path)
                self.new_folder_path = new_folder_path
        else:
            self.new_folder_path = None

        if self.record_params:
            # Define the command to execute rosparam dump
            folder_name = self.results_path
            filename = "rosparams.yaml"
            command = ["rosparam", "dump"]
            command.append(folder_name+"/"+filename)

            #if the file doesn't exists
            if not os.path.isfile(folder_name+"/"+filename):
                try:
                    subprocess.run(command, check=True)
                    rospy.logfatal("Parameters dumped successfully.")
                except subprocess.CalledProcessError as e:
                    rospy.logfatal(f"An error occurred while dumping parameters: {e}")
            else:
                rospy.logfatal("Parameters already dumped.")
        
        self.start_time = None
    
    def callback(self, req):
        ego_odom_frame = req.ego.header.frame_id
        # rospy.logerr("timestamp: %f",req.ego.header.stamp.to_sec())
        if self.start_time is None:
            self.start_time = req.ego.header.stamp.to_sec()
        if ego_odom_frame[0:5] == "hugin":
            ego_id = int(ego_odom_frame[6])
        else:
            rospy.logerr("Vehicle model not recognized!")

        try:
            plot_instance = self.plot_instances_dict[ego_id]
        except KeyError:
            plot_instance = PlotGeneratorServiceInstance(ego_id,self.new_folder_path,self.save_final_plots,self.start_time)
            self.plot_instances_dict[ego_id] = plot_instance
        plot_instance.callback(req)
    
        return PlotGeneratorResponse()



class PlotGeneratorServiceInstance:
    
    def __init__(self,ego_id,path,save_plots,start_time):
        # self.service = rospy.Service('/plot_generator', PlotGenerator, self.callback)
        # rospy.loginfo("Plot Generator service initialized")

        # self.message_pub = rospy.Publisher('/rviz_message', MarkerArray, queue_size=1)
        self.gt_distance = np.array([None,None]) #left, right
        self.gt_ego_bearing = np.array([None,None])
        self.distance = np.array([None,None]) #left, right
        self.ego_bearing = np.array([None,None])
        self.gt_ego_pose_ego_bl, self.gt_left_pose_ego_bl, self.gt_right_pose_ego_bl = None, None, None
        self.listener = tf.TransformListener()

        self.left_distance_errors = []  # Store distance errors
        self.l_d_e_t = [] #left distance error timestamps
        self.left_bearing_errors = []   # Store bearing errors
        self.l_b_e_t = [] #left bearing error timestamps
        self.right_distance_errors = [] # Store distance errors
        self.r_d_e_t = [] #right distance error timestamps
        self.right_bearing_errors = []  # Store bearing errors
        self.r_b_e_t = [] #right bearing error timestamps
        self.frame_count = 0        # Frame count for the animation

        self.time_diff = 0.0
        self.animate_plots = rospy.get_param('~animate_plots',True)
        self.vehicle_model = rospy.get_param('~vehicle_model',"hugin")
        self.ego_id = ego_id
        self.path = path
        self.save_final_plots = save_plots
        # rospy.Subscriber('/multi_agent/common_timestamps',Int32MultiArray,self.common_timestamps_cb,queue_size=1)
        rospy.Subscriber('/finished/'+str(self.vehicle_model)+'_'+str(self.ego_id),Bool,self.finished_cb,queue_size=1)

        # cv2.namedWindow("Errors Plot", cv2.WINDOW_NORMAL)
        # cv2.resizeWindow("Errors Plot", 500, 500)

        # Create a figure and axis for the animation
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(12, 10))
        self.fig.suptitle('Position error and variance over time')
        self.line_left_distance, = self.ax1.plot([], [], label='Left Error Sum')
        self.line_right_distance, = self.ax1.plot([], [], label='Right Error Sum')
        # self.line_left_bearing, = self.ax1.plot([], [], label='Left Bearing Error')
        # self.line_right_bearing, = self.ax1.plot([], [], label='Right Bearing Error')
        self.ax1.set_xlabel('Callback Iteration')
        self.ax1.set_ylabel('Error [m] or [deg]')
        self.ax1.legend()
        self.ax1.set_ylim(0, 1)
        self.ax1.set_title('Distance and Bearing Errors Over Time')

        self.ego_cov_list = []
        self.e_c_t = [] #ego covariance timestamps
        self.left_cov_list = []
        self.l_c_t = [] #left covariance timestamps
        self.right_cov_list = []
        self.r_c_t = [] #right covariance timestamps

        self.line_ego_cov, = self.ax2.plot([], [], label='Ego x & y Covariance Sum')
        self.line_left_cov, = self.ax2.plot([], [], label='Left x & y Covariance Sum')
        self.line_right_cov, = self.ax2.plot([], [], label='Right x & y Covariance Sum')
        self.ax2.set_xlabel('Callback Iteration')
        self.ax2.set_ylabel('Covariance Sum')
        self.ax2.legend()
        self.ax2.set_title('Covariance Sum Over Time')

        self.ego_abs_error = []
        self.e_a_e_t = [] #ego absolute error timestamps
        self.left_abs_error = []
        self.l_a_e_t = [] #left absolute error timestamps
        self.right_abs_error = []
        self.r_a_e_t = [] #right absolute error timestamps

        self.line_ego_abs_error, = self.ax3.plot([], [], label='Ego Absolute Positional Error')
        self.line_left_abs_error, = self.ax3.plot([], [], label='Left Absolute Positional Error')
        self.line_right_abs_error, = self.ax3.plot([], [], label='Right Absolute Position Error')
        self.ax3.set_xlabel('Callback Iteration')
        self.ax3.set_ylabel('Absolute Positional Error [m]')
        self.ax3.legend()
        self.ax3.set_title('Absolute Positional Error Over Time')

        # self.line_ego_x_var, = self.ax2.plot([], [], label='Ego Variance X')
        # self.line_ego_y_var, = self.ax2.plot([], [], label='Ego Variance Y')
        # self.ax2.set_xlabel('Callback Iteration')
        # self.ax2.set_ylabel('Ego Variance')
        # self.ax2.legend()

        # self.line_left_x_var, = self.ax3.plot([], [], label='Left Variance X')
        # self.line_left_y_var, = self.ax3.plot([], [], label='Left Variance Y')
        # self.ax3.set_xlabel('Callback Iteration')
        # self.ax3.set_ylabel('Left Variance')
        # self.ax3.legend()

        # self.line_right_x_var, = self.ax4.plot([], [], label='Right Variance X')
        # self.line_right_y_var, = self.ax4.plot([], [], label='Right Variance Y')
        # self.ax4.set_xlabel('Callback Iteration')
        # self.ax4.set_ylabel('Right Variance')
        # self.ax4.legend()

        # self.ego_x_var_list = []
        # self.ego_y_var_list = []

        # self.left_x_var_list = []
        # self.left_y_var_list = []

        # self.right_x_var_list = []
        # self.right_y_var_list = []


        # Create the animation function
        # self.ani = animation.FuncAnimation(self.fig, self.update_plot, frames=None, interval=200, blit=True)
        # plt.show()

        self.T_global_start = start_time
        self.T_local = 0
        # self.T_list = [0]

    def callback(self, req):
        """Generates plots"""
        # print("Received request")
        # print(self.ani)
        # print("left distance errors:",self.left_distance_errors)
        # print("right distance errors:",self.right_distance_errors)
        # print("left bearing errors:",self.left_bearing_errors)
        # print("right bearing errors:",self.right_bearing_errors)
        # print(req)
        # if self.T_global_start is None:
        #     # self.T_global_start = time.time()
        #     self.T_global_start = req.ego.header.stamp.to_sec()
        # else:
        #     # self.T_local = time.time()-self.T_global_start
        #     self.T_local = req.ego.header.stamp.to_sec()#-self.T_global_start
        #     # self.T_global = time.time()
        self.T_local = req.ego.header.stamp.to_sec() - self.T_global_start#-self.T_global_start
        ego_pose_with_cov = req.ego
        left_pose_with_cov = req.left
        right_pose_with_cov = req.right
        ego_odom_frame = ego_pose_with_cov.header.frame_id
        left_odom_frame = left_pose_with_cov.header.frame_id
        right_odom_frame = right_pose_with_cov.header.frame_id
        #Get the last character in the string and convert to int
        ego_id = -1
        left_id = -1
        right_id = -1
        if ego_odom_frame[0:5] == "hugin":
            ego_id = int(ego_odom_frame[6])
            # print("ego_id:",ego_id)
            #use numpy to convert covariance arrayto 6x6 matrix and calcualte determinant
            # ego_cov_matrix = np.array(ego_pose_with_cov.pose.covariance).reshape(6,6)
            # ego_cov_det = np.linalg.det(ego_cov_matrix)
            # print("ego_cov_matrix:",ego_cov_matrix)
            # print("ego_cov_det:",ego_cov_det)
            # self.ego_cov_det_list.append(ego_cov_det)
            self.ego_cov_list.append(ego_pose_with_cov.pose.covariance[0]+ego_pose_with_cov.pose.covariance[7])
            self.e_c_t.append(self.T_local)
            # self.ego_x_var_list.append(ego_pose_with_cov.pose.covariance[0]) #CONTINUE HERE 1, the variance values doesn't seem to decrease even though the partiles converge to a solution
            # self.ego_y_var_list.append(ego_pose_with_cov.pose.covariance[7])
            if left_odom_frame != str(-1):
                # print("left odom frame:",left_odom_frame)
                left_id = int(left_odom_frame[6])
                # left_cov_matrix = np.array(left_pose_with_cov.pose.covariance).reshape(6,6)
                # left_cov_det = np.linalg.det(left_cov_matrix)
                # self.left_cov_det_list.append(left_cov_det)
                self.left_cov_list.append(left_pose_with_cov.pose.covariance[0]+left_pose_with_cov.pose.covariance[7])
                self.l_c_t.append(self.T_local)
                # self.left_x_var_list.append(left_pose_with_cov.pose.covariance[0])
                # self.left_y_var_list.append(left_pose_with_cov.pose.covariance[7])
            if right_odom_frame != str(-1):
                # print("right odom frame:",right_odom_frame)
                right_id = int(right_odom_frame[6])
                # right_cov_matrix = np.array(right_pose_with_cov.pose.covariance).reshape(6,6)
                # right_cov_det = np.linalg.det(right_cov_matrix)
                # self.right_cov_det_list.append(right_cov_det)
                self.right_cov_list.append(right_pose_with_cov.pose.covariance[0]+right_pose_with_cov.pose.covariance[7])
                self.r_c_t.append(self.T_local)
                # self.right_x_var_list.append(right_pose_with_cov.pose.covariance[0])
                # self.right_y_var_list.append(right_pose_with_cov.pose.covariance[7])
        else: #Here you can handle other vehicle models
            rospy.logerr("Vehicle model not recognized!")
    
        self.update_gt(ego_id,left_id,right_id,ego_pose_with_cov.header.stamp)
        self.update_estimates(ego_pose_with_cov,left_pose_with_cov,right_pose_with_cov)
        self.update_absolute_errors(ego_pose_with_cov,left_pose_with_cov,right_pose_with_cov)
        # error_distance = np.array([abs(self.gt_distance[0]-self.distance[0]),abs(self.gt_distance[1]-self.distance[1])]) #left, right
        # error_bearing = np.array([abs(self.gt_ego_bearing[0]-self.ego_bearing[0]),abs(self.gt_ego_bearing[1]-self.ego_bearing[1])]) #left, right
        # Calculate errors

        # error_distance = np.array([None,None])
        # error_bearing = np.array([None,None])
        # print("left_id:",left_id)
        if left_id >= 0 and self.gt_distance[0] != None and self.distance[0] != None and self.gt_ego_bearing[0] != None and self.ego_bearing[0] != None:
            # print("gt_distance[0]:",self.gt_distance[0])
            # print("distance[0]:",self.distance[0])
            left_error_distance = abs(self.gt_distance[0] - self.distance[0])#/self.gt_distance[0]
            left_error_bearing = abs(self.gt_ego_bearing[0] - self.ego_bearing[0])#/self.gt_ego_bearing[0]
            # print("left_error_distance:",left_error_distance)
            # print("left_error_bearing:",left_error_bearing)
            # error_distance[0] = left_error_distance
            # error_bearing[0] = left_error_bearing
            self.left_distance_errors.append(left_error_distance)
            self.l_d_e_t.append(self.T_local)
            self.left_bearing_errors.append(np.rad2deg(left_error_bearing))
            self.l_b_e_t.append(self.T_local)
        # else:
        #     self.left_distance_errors.append(0)
        #     self.left_bearing_errors.append(0)
        # print("right_id:",right_id)
        if right_id >= 0 and self.gt_distance[1] != None and self.distance[1] != None and self.gt_ego_bearing[1] != None and self.ego_bearing[1] != None:
            # print("gt_distance[1]:",self.gt_distance[1])
            # print("distance[1]:",self.distance[1])
            right_error_distance = abs(self.gt_distance[1] - self.distance[1])#/self.gt_distance[1]
            right_error_bearing = abs(self.gt_ego_bearing[1] - self.ego_bearing[1])#/self.gt_ego_bearing[1]
            # print("right_error_distance:",right_error_distance)
            # print("right_error_bearing:",right_error_bearing)
            # error_distance[1] = right_error_distance
            # error_bearing[1] = right_error_bearing
            self.right_distance_errors.append(right_error_distance)
            self.r_d_e_t.append(self.T_local)
            self.right_bearing_errors.append(np.rad2deg(right_error_bearing))
            self.r_b_e_t.append(self.T_local)
        

        # else:
        #     self.right_distance_errors.append(0)
        #     self.right_bearing_errors.append(0)

        # error_distance = np.array([abs(self.gt_distance[0] - self.distance[0]), abs(self.gt_distance[1] - self.distance[1])])
        # error_bearing = np.array([abs(self.gt_ego_bearing[0] - self.ego_bearing[0]), abs(self.gt_ego_bearing[1] - self.ego_bearing[1])])

        # # Store errors
        # self.distance_errors.append(error_distance)
        # self.bearing_errors.append(error_bearing)

        # # Generate and display plot
        # if self.time_diff < 0.1:
        #     self.plot_errors()
        # else:
        #     rospy.logwarn("Time difference between ground truth and estimate is too large: %f",self.time_diff)

        # return PlotGeneratorResponse()

        self.update_plot(None)

        if self.animate_plots:
            plot_image = self.plot_to_image()

            window_name = "Animated Plot - AUV " + str(ego_id)
            cv2.imshow(window_name, plot_image)
            cv2.waitKey(1)  # Adjust the waitKey value as needed
    
    def plot_to_image(self):
        # Convert the plot to an image using matplotlib
        buf = io.BytesIO()
        self.fig.savefig(buf, format='png')
        buf.seek(0)
        img_array = np.frombuffer(buf.getvalue(), dtype=np.uint8)
        buf.close()
        plot_image = cv2.imdecode(img_array, 1)

        return plot_image


    def update_gt(self,ego_id,left_id,right_id,timestamp):
        """Updates the ground truth values"""
        if left_id >=0:
            self.gt_distance[0], self.gt_ego_bearing[0], self.gt_ego_pose_ego_bl, self.gt_left_pose_ego_bl = self.get_gt(ego_id,left_id,timestamp)
        if right_id >=0:
            self.gt_distance[1], self.gt_ego_bearing[1], self.gt_ego_pose_ego_bl, self.gt_right_pose_ego_bl = self.get_gt(ego_id,right_id,timestamp)

    def get_gt(self,ego_id,neighbour_id,timestamp):
        """Get the ground truth distance and bearing between two AUVs using the tf tree"""
        ego_bl_frame = "hugin_" + str(ego_id) + "/base_link"
        ego_pose = PoseStamped()
        ego_pose.header.frame_id = ego_bl_frame
        ego_pose.header.stamp = timestamp #rospy.Time(0) 

        # self.time_diff = abs(timestamp.to_sec() - ego_pose.header.stamp.to_sec())

        neighbour_bl_frame = "hugin_" + str(neighbour_id) + "/base_link"
        neighbour_pose = PoseStamped()
        neighbour_pose.header.frame_id = neighbour_bl_frame
        neighbour_pose.header.stamp = timestamp #rospy.Time(0)
        # self.time_diff = abs(timestamp.to_sec() - neighbour_pose.header.stamp.to_sec())
        try:
            neighbour_pose_ego_bl = self.listener.transformPose(ego_bl_frame,neighbour_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Error in transforming pose from %s to %s",neighbour_bl_frame,ego_bl_frame)
            rospy.logwarn(e)
            rospy.logwarn("In get_gt")
            return None, None, None, None
        
        # distance = np.sqrt(neighbour_pose_ego_bl.pose.position.x**2 + neighbour_pose_ego_bl.pose.position.y**2)
        # ego_bearing = np.arctan2(neighbour_pose_ego_bl.pose.position.y,neighbour_pose_ego_bl.pose.position.x)
        # print("in get_gt")
        # distance, ego_bearing = self.cartesian_to_polar(neighbour_pose_ego_bl.pose.position.x,neighbour_pose_ego_bl.pose.position.y)

        # self.time_diff = abs(timestamp.to_sec() - neighbour_pose.header.stamp.to_sec())
        ego_odom_frame = "hugin_" + str(ego_id) + "/odom"
        try:
            neighbour_pose_ego_odom = self.listener.transformPose(ego_odom_frame,neighbour_pose_ego_bl)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Error in transforming pose from %s to %s",neighbour_pose_ego_bl.header.frame_id,ego_odom_frame)
            rospy.logwarn(e)
            rospy.logwarn("In get_gt")
            return None, None, None, None
        
        try:
            ego_pose_ego_odom = self.listener.transformPose(ego_odom_frame,ego_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Error in transforming pose from %s to %s",ego_pose.header.frame_id,ego_odom_frame)
            rospy.logwarn(e)
            rospy.logwarn("In get_gt")
            return None, None, None, None

        distance, ego_bearing = self.cartesian_to_polar(neighbour_pose_ego_odom.pose.position.x-ego_pose_ego_odom.pose.position.x,neighbour_pose_ego_odom.pose.position.y-ego_pose_ego_odom.pose.position.y)
        
        return distance, ego_bearing, ego_pose, neighbour_pose_ego_bl

    def update_estimates(self,ego_pose_with_cov,left_pose_with_cov,right_pose_with_cov):
        """Updates the estimated values"""
        if left_pose_with_cov.header.frame_id != str(-1):
            self.distance[0], self.ego_bearing[0] = self.get_estimate(ego_pose_with_cov,left_pose_with_cov)
        if right_pose_with_cov.header.frame_id != str(-1):
            self.distance[1], self.ego_bearing[1] = self.get_estimate(ego_pose_with_cov,right_pose_with_cov)

    def get_estimate(self,ego_pose_with_cov,neighbour_pose_with_cov):
        """Get the estimated distance and bearing between two AUVs"""
        ego_odom_frame = ego_pose_with_cov.header.frame_id
        ego_pose = PoseStamped()
        ego_pose.header = ego_pose_with_cov.header
        ego_pose.pose = ego_pose_with_cov.pose.pose
        neighbour_pose = PoseStamped()
        neighbour_pose.header = neighbour_pose_with_cov.header
        neighbour_pose.pose = neighbour_pose_with_cov.pose.pose
        try:
            neighbour_pose_ego_odom = self.listener.transformPose(ego_odom_frame,neighbour_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Error in transforming pose from %s to %s",neighbour_pose.header.frame_id,ego_odom_frame)
            rospy.logwarn(e)
            rospy.logwarn("In get_estimate")
            return None, None
        # distance = np.sqrt(abs(neighbour_pose_ego_odom.pose.position.x-ego_pose.pose.position.x)**2 + abs(neighbour_pose_ego_odom.pose.position.y-ego_pose.pose.position.y)**2)
        # ego_bearing = np.arctan2(neighbour_pose_ego_odom.pose.position.y-ego_pose.pose.position.y,neighbour_pose_ego_odom.pose.position.x-ego_pose.pose.position.x) #CONTINUE HERE 2, the bearing valuues seem to be wrong. They oscillate weirdly.
        # print("ego bearing:",ego_bearing)
        # print("in get_estimate")
        distance, ego_bearing = self.cartesian_to_polar(neighbour_pose_ego_odom.pose.position.x-ego_pose.pose.position.x,neighbour_pose_ego_odom.pose.position.y-ego_pose.pose.position.y)
        return distance, ego_bearing
    
    def cartesian_to_polar(self,x,y):
        """Converts cartesian coordinates to polar coordinates"""
        # print("x:",x)
        # print("y:",y)
        if x < 1e-6:
            x = 0
        if y < 1e-6:
            y = 0
        
        if x == 0 and y == 0:
            return 0, 0
        elif x == 0 and y > 0:
            return y, np.pi/2
        elif x == 0 and y < 0:
            return y, -np.pi/2
        elif x > 0 and y == 0:
            return x, 0
        elif x < 0 and y == 0:
            return x, np.pi
            
        distance = np.sqrt(x**2 + y**2)
        bearing = np.arctan2(y,x)
        return distance, bearing
    
    def update_absolute_errors(self,ego_pose_with_cov,left_pose_with_cov,right_pose_with_cov):
        """Updates the absolute errors"""
        ego_error = self.get_absolute_error(ego_pose_with_cov,self.gt_ego_pose_ego_bl)
        if ego_error is not None:
            self.ego_abs_error.append(ego_error)
            self.e_a_e_t.append(self.T_local)
        if left_pose_with_cov.header.frame_id != str(-1):
            left_error = self.get_absolute_error(left_pose_with_cov,self.gt_left_pose_ego_bl)
            if left_error is not None:
                self.left_abs_error.append(left_error)
                self.l_a_e_t.append(self.T_local)
        if right_pose_with_cov.header.frame_id != str(-1):
            right_error = self.get_absolute_error(right_pose_with_cov,self.gt_right_pose_ego_bl)
            if right_error is not None:
                self.right_abs_error.append(right_error)
                self.r_a_e_t.append(self.T_local)

    def get_absolute_error(self,pose_with_cov,gt_pose_ego_bl):
        """Get the absolute error between the estimated and ground truth pose"""
        if gt_pose_ego_bl is None:
            return
        pose = PoseStamped()
        pose.header = pose_with_cov.header
        # pose.header.stamp = rospy.Time(0) #NOTE extrapolation issues here
        # pose.header.frame_id = pose_with_cov.header.frame_id
        pose.pose = pose_with_cov.pose.pose
        try:
            pose_ego_bl = self.listener.transformPose(gt_pose_ego_bl.header.frame_id,pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn("Error in transforming pose from %s to %s",pose.header.frame_id,gt_pose_ego_bl.header.frame_id)
            rospy.logwarn(e)
            rospy.logwarn("In get_absolute_error")
            return None
        error = np.sqrt(abs(pose_ego_bl.pose.position.x-gt_pose_ego_bl.pose.position.x)**2 + abs(pose_ego_bl.pose.position.y-gt_pose_ego_bl.pose.position.y)**2)
        return error

    # def plot_errors(self):
    #     # Create a plot of distance and bearing errors
    #     plt.figure(figsize=(8, 6))

    #     plt.subplot(2, 1, 1)
    #     # print("Distance errors:",self.distance_errors)
    #     # print("Bearing errors:",self.bearing_errors)
    #     # plt.plot([err[0] for err in self.distance_errors], label='Left Distance Error')
    #     # plt.plot([err[1] for err in self.distance_errors], label='Right Distance Error')
    #     plt.plot(self.left_distance_errors, label='Left Distance Error')
    #     plt.plot(self.right_distance_errors, label='Right Distance Error')
    #     plt.xlabel('Callback Iteration')
    #     plt.ylabel('Distance Error')
    #     plt.legend()
    #     plt.title('Distance Errors Over Time')

    #     plt.subplot(2, 1, 2)
    #     # plt.plot([err[0] for err in self.bearing_errors], label='Left Bearing Error')
    #     # plt.plot([err[1] for err in self.bearing_errors], label='Right Bearing Error')
    #     plt.plot(self.left_bearing_errors, label='Left Bearing Error')
    #     plt.plot(self.right_bearing_errors, label='Right Bearing Error')
    #     plt.xlabel('Callback Iteration')
    #     plt.ylabel('Bearing Error')
    #     plt.legend()
    #     plt.title('Bearing Errors Over Time')

    #     # plt.show()

    #     plt.tight_layout()

    #     # Convert the plot to an image
    #     buf = io.BytesIO()
    #     plt.savefig(buf, format='png')
    #     buf.seek(0)
    #     img_array = np.frombuffer(buf.getvalue(), dtype=np.uint8)
    #     buf.close()
    #     img = cv2.imdecode(img_array, 1)

    #     # Display the plot using cv2
    #     cv2.imshow('Errors Plot', img)
    #     cv2.waitKey(1)

    #     # Save the plot as an image if needed
    #     # cv2.imwrite('errors_plot.png', img)

    #     plt.close()

    #     # Increment frame count for the animation
    #     self.frame_count += 1

    def update_plot(self, frame):
        # Update the plot with new data for each frame
        # rospy.logfatal("Time: %f s",self.T_local)
        x_left = np.arange(len(self.left_distance_errors))
        x_right = np.arange(len(self.right_distance_errors))
        # print("left distance errors:",self.left_distance_errors)
        # print("right distance errors:",self.right_distance_errors)
        # print("left bearing errors:",self.left_bearing_errors)
        # print("right bearing errors:",self.right_bearing_errors)
        if len(self.left_distance_errors) != 0:
            self.line_left_distance.set_data(self.l_d_e_t, [x+y for x,y in zip(self.left_distance_errors,self.left_bearing_errors)])
        if len(self.right_distance_errors) != 0:
            self.line_right_distance.set_data(self.r_d_e_t, [x+y for x,y in zip(self.right_distance_errors,self.right_bearing_errors)])
        # if len(self.left_bearing_errors) != 0:
        #     self.line_left_bearing.set_data(self.l_b_e_t, self.left_bearing_errors)
        # if len(self.right_bearing_errors) != 0:
        #     self.line_right_bearing.set_data(self.r_b_e_t, self.right_bearing_errors)

        # Update the x-axis limits dynamically
        max_x = max(len(self.left_distance_errors), len(self.right_distance_errors))
        self.ax1.set_xlim(0, max_x)
        y_max_list = [0]
        if len(self.left_distance_errors) != 0:
            y_max_list.append(max(self.left_distance_errors))
            y_max_list.append(max(self.left_bearing_errors))
        if len(self.right_distance_errors) != 0:
            y_max_list.append(max(self.right_distance_errors))
            y_max_list.append(max(self.right_bearing_errors))
        self.ax1.set_ylim(0, max(y_max_list))

        self.line_ego_cov.set_data(self.e_c_t, self.ego_cov_list)
        if len(self.left_cov_list) != 0:
            self.line_left_cov.set_data(self.l_c_t, self.left_cov_list)
        if len(self.right_cov_list) != 0:
            self.line_right_cov.set_data(self.r_c_t, self.right_cov_list)
        self.ax2.set_xlim(0, len(self.ego_cov_list))
        y_max_list = [0] #sqrt0.0001 = 0.01. = 1cm uncertainty
        if len(self.ego_cov_list) != 0:
            y_max_list.append(max(self.ego_cov_list))
        if len(self.left_cov_list) != 0:
            y_max_list.append(max(self.left_cov_list))
        if len(self.right_cov_list) != 0:
            y_max_list.append(max(self.right_cov_list))

        self.ax2.set_ylim(0, max(y_max_list))


        self.line_ego_abs_error.set_data(self.e_a_e_t, self.ego_abs_error)
        if len(self.left_abs_error) != 0:
            self.line_left_abs_error.set_data(self.l_a_e_t, self.left_abs_error)
        if len(self.right_abs_error) != 0:
            self.line_right_abs_error.set_data(self.r_a_e_t, self.right_abs_error)
        self.ax3.set_xlim(0, len(self.ego_abs_error))
        y_max_list = [0.001]
        if len(self.ego_abs_error) != 0:
            y_max_list.append(max(self.ego_abs_error))
        if len(self.left_abs_error) != 0:
            y_max_list.append(max(self.left_abs_error))
        if len(self.right_abs_error) != 0:
            y_max_list.append(max(self.right_abs_error))
        self.ax3.set_ylim(0, max(y_max_list))
        # self.line_ego_x_var.set_data(np.arange(len(self.ego_x_var_list)), self.ego_x_var_list)
        # self.line_ego_y_var.set_data(np.arange(len(self.ego_y_var_list)), self.ego_y_var_list)
        # self.ax2.set_xlim(0, len(self.ego_x_var_list))

        # self.line_left_x_var.set_data(np.arange(len(self.left_x_var_list)), self.left_x_var_list)
        # self.line_left_y_var.set_data(np.arange(len(self.left_y_var_list)), self.left_y_var_list)
        # self.ax3.set_xlim(0, len(self.left_x_var_list))

        # self.line_right_x_var.set_data(np.arange(len(self.right_x_var_list)), self.right_x_var_list)
        # self.line_right_y_var.set_data(np.arange(len(self.right_y_var_list)), self.right_y_var_list)
        # self.ax4.set_xlim(0, len(self.right_x_var_list))

        return self.line_left_distance, self.line_right_distance, self.line_left_bearing, self.line_right_bearing, self.line_ego_cov, self.line_left_cov, self.line_right_cov, self.line_ego_abs_error, self.line_left_abs_error, self.line_right_abs_error

    def finished_cb(self,msg):
        """Called when the finished topic is published"""
        if msg.data and self.save_final_plots:
            # Define the filename for the CSV file
            #csv_filename = self.path + auv_ + self.ego_id.csv
            csv_filename = os.path.join(self.path, f"auv_{self.ego_id}.csv")

            # Gather data from the plots to save
            data_to_save = {
                'left_distance_errors': self.left_distance_errors,
                'right_distance_errors': self.right_distance_errors,
                'left_bearing_errors': self.left_bearing_errors,
                'right_bearing_errors': self.right_bearing_errors,
                'ego_cov_list': self.ego_cov_list,
                'left_cov_list': self.left_cov_list,
                'right_cov_list': self.right_cov_list,
                'ego_abs_error': self.ego_abs_error,
                'left_abs_error': self.left_abs_error,
                'right_abs_error': self.right_abs_error,
                'l_d_e_t': self.l_d_e_t,
                'r_d_e_t': self.r_d_e_t,
                'l_b_e_t': self.l_b_e_t,
                'r_b_e_t': self.r_b_e_t,
                'e_c_t': self.e_c_t,
                'l_c_t': self.l_c_t,
                'r_c_t': self.r_c_t,
                'e_a_e_t': self.e_a_e_t,
                'l_a_e_t': self.l_a_e_t,
                'r_a_e_t': self.r_a_e_t,
                # Add more data you want to save here
            }

            # Determine the length of the data
            data_length = max(len(data) for data in data_to_save.values())

            # Write the data to a CSV file
            with open(csv_filename, mode='w', newline='') as file:
                writer = csv.DictWriter(file, fieldnames=data_to_save.keys())
                writer.writeheader()

                for i in range(data_length):
                    row_data = {key: data[i] if i < len(data) else None for key, data in data_to_save.items()}
                    writer.writerow(row_data)

            rospy.loginfo(f"Saved plot data to {csv_filename}")

if __name__ == '__main__':
    rospy.init_node('plot_generator_service')
    plot_generator = PlotGeneratorService()
    # plt.show()
    rospy.spin()
    