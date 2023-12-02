#!/usr/bin/env python3

import rospy 
# from rviz_visualization.srv import DisplayRvizMessage, DisplayRvizMessageResponse
# from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from plot_generator.srv import PlotGenerator, PlotGeneratorResponse



class PlotGeneratorService:
    
    def __init__(self):
        self.service = rospy.Service('/plot_generator', PlotGenerator, self.callback)
        rospy.loginfo("Plot Generator service initialized")
        # self.message_pub = rospy.Publisher('/rviz_message', MarkerArray, queue_size=1)


    def callback(self, req):
        """Generates plots"""
        print("Received request")
        print(req)
        
        return PlotGeneratorResponse()

if __name__ == '__main__':
    rospy.init_node('plot_generator_service')
    PlotGeneratorService()
    rospy.spin()
    