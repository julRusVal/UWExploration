#!/usr/bin/env python3

# Standard dependencies
import math
import rospy
import sys
import numpy as np
# from scipy.spatial.transform import Rotation as rot

from tf.transformations import quaternion_matrix
from tf.transformations import rotation_matrix

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Bool
from geometry_msgs.msg import Point
from sensor_msgs import point_cloud2
from scipy.ndimage import gaussian_filter1d

# For sim mbes action client
import actionlib
from auv_2_ros.msg import FlsSimAction, FlsSimResult
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion

#TODO:
#1. OK auv_fls_model.py: action server that gives FLS reading upon request
# - a semi-OK Make a bool such that FLS sends 	set_aborted() if it is not ready to send a ping. Ie, from mission planner we can publish ushc that when we know we wont
#       have an auv infront of us, we can set the bool to false and the FLS will not send a ping. This to save computation. But for other cases, it's just always true.
# - b CONTINUE HERE Static tf fls - base_link somewhere (see where mbes - base_link is)
#2. auv_motion_simple.cpp: a single request at a given transform and time to the action server above #1
#3. auv_motion_simple_node.cpp: a node that at a certain timer interval calls the single request #2 with this specific rate
#4. Integrate into all launch files

class FLSModel(object):

    def __init__(self):

        # # Load mesh
        # svp_path = rospy.get_param('~sound_velocity_prof')
        # mesh_path = rospy.get_param('~mesh_path')
        self.fls_horizontal_angle = rospy.get_param("~fls_horizontal_angle", np.deg2rad(135))
        self.fls_vertical_angle = rospy.get_param("~fls_vertical_angle", np.deg2rad(60))
        self.fls_max_range = rospy.get_param("~fls_max_range", 100) #meters
        self.vehicle_model = rospy.get_param("~vehicle_model", "hugin")
        self.num_auvs = rospy.get_param("~num_auvs", 1)
        self.fls_enable_topic = rospy.get_param("~fls_enable_topic", "/fls_sim_enable") #TODO: add in launch file 1a
        self.scan_area_marker_topic = rospy.get_param("~scan_area_marker_topic", "/sim/fls_scan_area") #TODO: add in launch file 1a
        # Initialize the TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.fls_enabled = True #TODO: in w2w_planner publish to this topic to enable/disable FLS

        rospy.Subscriber(self.fls_enable_topic, Bool, self.fls_enable_cb)


        # # Action server for FLS pings sim (necessary to be able to use UFO maps as well)
        sim_fls_as = rospy.get_param('~fls_sim_as', '/fls_sim_server')
        server_mode = rospy.get_param("~server_mode", False)
        self.as_ping = actionlib.SimpleActionServer(sim_fls_as, FlsSimAction,
                                                    execute_cb=self.fls_as_cb, auto_start=server_mode)
        #Publish scan area to rviz
        self.scan_area_marker_pub = rospy.Publisher(self.scan_area_marker_topic, MarkerArray, queue_size=1)
        self.goal_header = None

        rospy.spin()


    # Action server to simulate FLS for the sim AUV
    def fls_as_cb(self, goal):
        self.goal_header = goal.header
        if self.fls_enabled:
            self.publish_fls_scan_area_to_rviz()
            tf_map2fls = goal.map2fls_tf
            r,theta = None,None
            for i in range(self.num_auvs): #TODO: find a faster way to do this, if it's an issue - rewrite in C++
                hugin_frame = "hugin_" + str(i) + "/base_link"
                tf_hugin2map = self.tf_buffer.lookup_transform("map", hugin_frame, goal.header.stamp, rospy.Duration(1.0))
                tf_hugin2fls = tf_hugin2map * tf_map2fls
                point = Point() #point in origin
                point = tf_hugin2fls*point
                if self.point_in_fov(point):
                    r, theta = self.polar_coordinates(point.x,point.y)
                    break #TODO: handle the case with multiple agents in the fov. This is out of scope for the multi-agent thesis of Koray, but can be extended later here. If you do, remember to change definition of action message aswell
            
            result = FlsSimResult()
            result.header = goal.header
            result.range = r
            result.angle = theta
            self.as_ping.set_succeeded(result)
        else:
            #Don't do any calculation if FLS is disabled, save computation.
            self.as_ping.set_aborted()
    
    def point_in_fov(self,point):
        """Returns true if the point is within the field of view of the FLS. 
        Point in the FLS frame.
        """
        x = point.x
        y = point.y
        z = point.z
        R = self.fls_max_range
        alpha = self.fls_horizontal_angle
        beta = self.fls_vertical_angle
        in_fov = x >=0 and x<=R*np.cos(alpha/2) and y>=-R*np.sin(alpha/2) and y<=R*np.sin(alpha/2) and z>=-R*np.sin(beta/2) and z<=R*np.sin(beta/2)
        return in_fov
    
    def polar_coordinates(self,x,y):
        """Returns the polar coordinates of a point in the FLS frame.
        """
        r = np.sqrt(x**2 + y**2)
        theta = np.arctan2(y,x)
        return r,theta
    
    def fls_enable_cb(self, msg):
        self.fls_enabled = msg.data
    
    def publish_fls_scan_area_to_rviz(self):
        # Create marker array
        #r_list = np.linspace(0,self.fls_max_range,5)
        theta_list = np.linspace(-self.fls_horizontal_angle/2,self.fls_horizontal_angle/2,5)
        phi_list = np.linspace(-self.fls_vertical_angle/2,self.fls_vertical_angle/2,5)
        marker_array = MarkerArray()

        for theta in theta_list:
            for phi in phi_list:
                # Create a line strip marker for the rectangle
                marker = Marker()
                marker.header = self.goal_header
                marker.type = Marker.LINE_STRIP
                marker.id = 0
                marker.pose.orientation = Quaternion(0, 0, 0, 1)  # Set the quaternion for orientation

                # Set the scale of the marker (line width)
                marker.scale.x = 0.5  # You can adjust this value

                # Set the color 
                marker.color.r = 0.0
                marker.color.g = 0.0
                marker.color.b = 1.0
                marker.color.a = 0.75 #A bit see through

                # Add points to the line strip marker to define the rectangle
                x = self.fls_max_range*np.sin(theta)*np.cos(phi)
                y = self.fls_max_range*np.sin(theta)*np.sin(phi)
                z = self.fls_max_range*np.cos(phi)
                points = [
                        Point(0,0,0),
                        Point(x,y,z),
                        ]

                # Append points to the marker
                for point in points:
                    marker_point = Point()
                    marker_point.x = point.x
                    marker_point.y = point.y
                    marker_point.z = point.z
                    marker.points.append(marker_point)

                # Add the rectangle marker to the marker array
                marker_array.markers.append(marker)

        # Publish the marker array
        self.scan_area_marker_pub.publish(marker_array)
            



if __name__ == '__main__':

    rospy.init_node('auv_fls_model', disable_signals=False)
    try:
        FLSModel()
    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch auv_fls_model")
        pass

