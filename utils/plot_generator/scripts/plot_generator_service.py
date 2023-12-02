#!/usr/bin/env python3

import rospy 
# from rviz_visualization.srv import DisplayRvizMessage, DisplayRvizMessageResponse
# from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from plot_generator.srv import PlotGenerator, PlotGeneratorResponse
import cv2
import numpy as np
import matplotlib.pyplot as plt
import tf
import io


class PlotGeneratorService:
    
    def __init__(self):
        self.service = rospy.Service('/plot_generator', PlotGenerator, self.callback)
        rospy.loginfo("Plot Generator service initialized")
        # self.message_pub = rospy.Publisher('/rviz_message', MarkerArray, queue_size=1)
        self.gt_distance = np.array([None,None]) #left, right
        self.gt_ego_bearing = np.array([None,None])
        self.distance = np.array([None,None]) #left, right
        self.ego_bearing = np.array([None,None])

        self.listener = tf.TransformListener()

        self.distance_errors = []  # Store distance errors
        self.bearing_errors = []   # Store bearing errors
        self.frame_count = 0        # Frame count for the animation



    def callback(self, req):
        """Generates plots"""
        print("Received request")
        print(req)
        ego_pose_with_cov = req.ego
        left_pose_with_cov = req.left
        right_pose_with_cov = req.right
        #Get the last character in the string and convert to int
        ego_id = int(ego_pose_with_cov.header.frame_id[-1]) #CONTINUE HERE
        left_id = int(left_pose_with_cov.header.frame_id[-1])
        right_id = int(right_pose_with_cov.header.frame_id[-1])
        self.update_gt(ego_id,left_id,right_id,ego_pose_with_cov.header.stamp)
        self.update_estimates(ego_pose_with_cov,left_pose_with_cov,right_pose_with_cov)
        # error_distance = np.array([abs(self.gt_distance[0]-self.distance[0]),abs(self.gt_distance[1]-self.distance[1])]) #left, right
        # error_bearing = np.array([abs(self.gt_ego_bearing[0]-self.ego_bearing[0]),abs(self.gt_ego_bearing[1]-self.ego_bearing[1])]) #left, right
        # Calculate errors
        error_distance = np.array([abs(self.gt_distance[0] - self.distance[0]), abs(self.gt_distance[1] - self.distance[1])])
        error_bearing = np.array([abs(self.gt_ego_bearing[0] - self.ego_bearing[0]), abs(self.gt_ego_bearing[1] - self.ego_bearing[1])])

        # Store errors
        self.distance_errors.append(error_distance)
        self.bearing_errors.append(error_bearing)

        # Generate and display plot
        self.plot_errors()

        return PlotGeneratorResponse()

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
        ego_pose.header.stamp = timestamp

        neighbour_bl_frame = "hugin_" + str(neighbour_id) + "/base_link"
        neighbour_pose = PoseStamped()
        neighbour_pose.header.frame_id = neighbour_bl_frame
        neighbour_pose.header.stamp = timestamp

        neighbour_pose_ego_bl = self.listener.transformPose(ego_bl_frame,neighbour_pose)

        distance = np.sqrt(neighbour_pose_ego_bl.pose.position.x**2 + neighbour_pose_ego_bl.pose.position.y**2)
        ego_bearing = np.arctan2(neighbour_pose_ego_bl.pose.position.y,neighbour_pose_ego_bl.pose.position.x)
        return distance, ego_bearing

    def update_estimates(self,ego_pose_with_cov,left_pose_with_cov,right_pose_with_cov):
        """Updates the estimated values"""
        if left_pose_with_cov.header.frame_id >=0:
            self.distance[0], self.ego_bearing[0] = self.get_estimate(ego_pose_with_cov,left_pose_with_cov)
        if right_pose_with_cov.header.frame_id >=0:
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
        neighbour_pose_ego_odom = self.listener.transformPose(ego_odom_frame,neighbour_pose)

        distance = np.sqrt(abs(neighbour_pose_ego_odom.pose.position.x-ego_pose.pose.position.x)**2 + abs(neighbour_pose_ego_odom.pose.position.y-ego_pose.pose.position.y)**2)
        ego_bearing = np.arctan2(neighbour_pose_ego_odom.pose.position.y-ego_pose.pose.position.y,neighbour_pose_ego_odom.pose.position.x-ego_pose.pose.position.x)
        return distance, ego_bearing
    
    def plot_errors(self):
        # Create a plot of distance and bearing errors
        plt.figure(figsize=(8, 6))

        plt.subplot(2, 1, 1)
        plt.plot([err[0] for err in self.distance_errors], label='Left Distance Error')
        plt.plot([err[1] for err in self.distance_errors], label='Right Distance Error')
        plt.xlabel('Callback Iteration')
        plt.ylabel('Distance Error')
        plt.legend()
        plt.title('Distance Errors Over Time')

        plt.subplot(2, 1, 2)
        plt.plot([err[0] for err in self.bearing_errors], label='Left Bearing Error')
        plt.plot([err[1] for err in self.bearing_errors], label='Right Bearing Error')
        plt.xlabel('Callback Iteration')
        plt.ylabel('Bearing Error')
        plt.legend()
        plt.title('Bearing Errors Over Time')

        plt.tight_layout()

        # Convert the plot to an image
        buf = io.BytesIO()
        plt.savefig(buf, format='png')
        buf.seek(0)
        img_array = np.frombuffer(buf.getvalue(), dtype=np.uint8)
        buf.close()
        img = cv2.imdecode(img_array, 1)

        # Display the plot using cv2
        cv2.imshow('Errors Plot', img)
        cv2.waitKey(1)

        # Save the plot as an image if needed
        # cv2.imwrite('errors_plot.png', img)

        plt.close()

        # Increment frame count for the animation
        self.frame_count += 1

if __name__ == '__main__':
    rospy.init_node('plot_generator_service')
    PlotGeneratorService()
    rospy.spin()
    