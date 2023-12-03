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

class PlotGeneratorService:
    def __init__(self):
        self.service = rospy.Service('/plot_generator', PlotGenerator, self.callback)
        rospy.loginfo("Plot Generator service initialized")

        self.plot_instances_dict = {}
    
    def callback(self, req):
        ego_odom_frame = req.ego.header.frame_id
        if ego_odom_frame[0:5] == "hugin":
            ego_id = int(ego_odom_frame[6])
        else:
            rospy.logerr("Vehicle model not recognized!")

        try:
            plot_instance = self.plot_instances_dict[ego_id]
        except KeyError:
            plot_instance = PlotGeneratorServiceInstance()
            self.plot_instances_dict[ego_id] = plot_instance
        plot_instance.callback(req)
    
        return PlotGeneratorResponse()



class PlotGeneratorServiceInstance:
    
    def __init__(self):
        # self.service = rospy.Service('/plot_generator', PlotGenerator, self.callback)
        # rospy.loginfo("Plot Generator service initialized")

        # self.message_pub = rospy.Publisher('/rviz_message', MarkerArray, queue_size=1)
        self.gt_distance = np.array([None,None]) #left, right
        self.gt_ego_bearing = np.array([None,None])
        self.distance = np.array([None,None]) #left, right
        self.ego_bearing = np.array([None,None])

        self.listener = tf.TransformListener()

        self.left_distance_errors = []  # Store distance errors
        self.left_bearing_errors = []   # Store bearing errors
        self.right_distance_errors = [] # Store distance errors
        self.right_bearing_errors = []  # Store bearing errors
        self.frame_count = 0        # Frame count for the animation

        self.time_diff = 0.0

        # cv2.namedWindow("Errors Plot", cv2.WINDOW_NORMAL)
        # cv2.resizeWindow("Errors Plot", 500, 500)

        # Create a figure and axis for the animation
        self.fig, self.ax = plt.subplots()
        self.line_left_distance, = self.ax.plot([], [], label='Left Distance Error')
        self.line_right_distance, = self.ax.plot([], [], label='Right Distance Error')
        self.line_left_bearing, = self.ax.plot([], [], label='Left Bearing Error')
        self.line_right_bearing, = self.ax.plot([], [], label='Right Bearing Error')
        self.ax.set_xlabel('Callback Iteration')
        self.ax.set_ylabel('Error')
        self.ax.legend()

        # Create the animation function
        self.ani = animation.FuncAnimation(self.fig, self.update_plot, frames=None, interval=200, blit=True)


    def callback(self, req):
        """Generates plots"""
        print("Received request")
        # print("left distance errors:",self.left_distance_errors)
        # print("right distance errors:",self.right_distance_errors)
        # print("left bearing errors:",self.left_bearing_errors)
        # print("right bearing errors:",self.right_bearing_errors)
        # print(req)
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
            if left_odom_frame != str(-1):
                # print("left odom frame:",left_odom_frame)
                left_id = int(left_odom_frame[6])
            if right_odom_frame != str(-1):
                # print("right odom frame:",right_odom_frame)
                right_id = int(right_odom_frame[6])
        else: #Here you can handle other vehicle models
            rospy.logerr("Vehicle model not recognized!")
    
        self.update_gt(ego_id,left_id,right_id,ego_pose_with_cov.header.stamp)
        self.update_estimates(ego_pose_with_cov,left_pose_with_cov,right_pose_with_cov)
        # error_distance = np.array([abs(self.gt_distance[0]-self.distance[0]),abs(self.gt_distance[1]-self.distance[1])]) #left, right
        # error_bearing = np.array([abs(self.gt_ego_bearing[0]-self.ego_bearing[0]),abs(self.gt_ego_bearing[1]-self.ego_bearing[1])]) #left, right
        # Calculate errors

        # error_distance = np.array([None,None])
        # error_bearing = np.array([None,None])
        if left_id >= 0:
            left_error_distance = abs(self.gt_distance[0] - self.distance[0])
            left_error_bearing = abs(self.gt_ego_bearing[0] - self.ego_bearing[0])
            # error_distance[0] = left_error_distance
            # error_bearing[0] = left_error_bearing
            self.left_distance_errors.append(left_error_distance)
            self.left_bearing_errors.append(left_error_bearing)
        else:
            self.left_distance_errors.append(0)
            self.left_bearing_errors.append(0)
        if right_id >= 0:
            right_error_distance = abs(self.gt_distance[1] - self.distance[1])
            right_error_bearing = abs(self.gt_ego_bearing[1] - self.ego_bearing[1])
            # error_distance[1] = right_error_distance
            # error_bearing[1] = right_error_bearing
            self.right_distance_errors.append(right_error_distance)
            self.right_bearing_errors.append(right_error_bearing)
        else:
            self.right_distance_errors.append(0)
            self.right_bearing_errors.append(0)

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

    def update_gt(self,ego_id,left_id,right_id,timestamp):
        """Updates the ground truth values"""
        if left_id >=0:
            self.gt_distance[0], self.gt_ego_bearing[0] = self.get_gt(ego_id,left_id,timestamp)
        if right_id >=0:
            self.gt_distance[1], self.gt_ego_bearing[1] = self.get_gt(ego_id,right_id,timestamp)

    def get_gt(self,ego_id,neighbour_id,timestamp):
        """Get the ground truth distance and bearing between two AUVs using the tf tree"""
        ego_bl_frame = "hugin_" + str(ego_id) + "/base_link"
        ego_pose = PoseStamped()
        ego_pose.header.frame_id = ego_bl_frame
        ego_pose.header.stamp = rospy.Time(0) #timestamp

        # self.time_diff = abs(timestamp.to_sec() - ego_pose.header.stamp.to_sec())

        neighbour_bl_frame = "hugin_" + str(neighbour_id) + "/base_link"
        neighbour_pose = PoseStamped()
        neighbour_pose.header.frame_id = neighbour_bl_frame
        neighbour_pose.header.stamp = rospy.Time(0) #timestamp
        # self.time_diff = abs(timestamp.to_sec() - neighbour_pose.header.stamp.to_sec())
        try:
            neighbour_pose_ego_bl = self.listener.transformPose(ego_bl_frame,neighbour_pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Error in transforming pose from %s to %s",neighbour_bl_frame,ego_bl_frame)
            return 0, 0

        distance = np.sqrt(neighbour_pose_ego_bl.pose.position.x**2 + neighbour_pose_ego_bl.pose.position.y**2)
        ego_bearing = np.arctan2(neighbour_pose_ego_bl.pose.position.y,neighbour_pose_ego_bl.pose.position.x)
        return distance, ego_bearing

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
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("Error in transforming pose from %s to %s",neighbour_pose.header.frame_id,ego_odom_frame)
            return 0, 0
        distance = np.sqrt(abs(neighbour_pose_ego_odom.pose.position.x-ego_pose.pose.position.x)**2 + abs(neighbour_pose_ego_odom.pose.position.y-ego_pose.pose.position.y)**2)
        ego_bearing = np.arctan2(neighbour_pose_ego_odom.pose.position.y-ego_pose.pose.position.y,neighbour_pose_ego_odom.pose.position.x-ego_pose.pose.position.x)
        return distance, ego_bearing
    
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
        x_left = np.arange(len(self.left_distance_errors))
        x_right = np.arange(len(self.right_distance_errors))
        print("left distance errors:",self.left_distance_errors)
        print("right distance errors:",self.right_distance_errors)
        print("left bearing errors:",self.left_bearing_errors)
        print("right bearing errors:",self.right_bearing_errors)
        self.line_left_distance.set_data(x_left, self.left_distance_errors)
        self.line_right_distance.set_data(x_right, self.right_distance_errors)
        self.line_left_bearing.set_data(x_left, self.left_bearing_errors)
        self.line_right_bearing.set_data(x_right, self.right_bearing_errors)

        # Update the x-axis limits dynamically
        max_x = max(len(self.left_distance_errors), len(self.right_distance_errors))
        self.ax.set_xlim(0, max_x)

        return self.line_left_distance, self.line_right_distance, self.line_left_bearing, self.line_right_bearing

if __name__ == '__main__':
    rospy.init_node('plot_generator_service')
    plot_generator = PlotGeneratorService()
    # plt.show()
    rospy.spin()
    