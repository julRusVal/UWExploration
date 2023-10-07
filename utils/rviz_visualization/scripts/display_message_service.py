#!/usr/bin/env python3

import rospy 
from rviz_visualization.srv import DisplayRvizMessage, DisplayRvizMessageResponse
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion



class DisplayMessageService:
    
    def __init__(self):
        self.service = rospy.Service('/display_rviz_message', DisplayRvizMessage, self.callback)
        rospy.loginfo("Display message service initialized")
        self.message_pub = rospy.Publisher('/rviz_message', MarkerArray, queue_size=1)


    def callback(self, req):
        """Publishes a message to rviz"""
        message = req.message.data
        # rospy.loginfo("Displaying message: " + message)
        
        # Create marker array
        marker_array = MarkerArray()

        # Create a line strip marker for the rectangle
        marker_text = Marker()
        marker_text.header.frame_id = "map"
        marker_text.header.stamp = rospy.Time.now()
        marker_text.type = Marker.TEXT_VIEW_FACING
        marker_text.id = 0
        marker_text.pose.orientation = Quaternion(0, 0, 0, 1)
        marker_text.pose.position = Point(0, 0, 0)  # Set the position of the marker
        marker_text.scale.z = 10.0  # Set the scale of the marker (text height)
        marker_text.color.r = 1.0  # Set the color (blue, fully opaque)
        marker_text.color.g = 1.0
        marker_text.color.b = 1.0
        marker_text.color.a = 1.0
        marker_text.text = message  # Set the text of the marker

        # Add the marker to the marker array
        marker_array.markers.append(marker_text)

        # Publish the marker array
        self.message_pub.publish(marker_array)
        return DisplayRvizMessageResponse()

if __name__ == '__main__':
    rospy.init_node('display_rviz_message_service')
    DisplayMessageService()
    rospy.spin()
    