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
from std_msgs.msg import Header, Bool, Float32
from geometry_msgs.msg import Point, Transform, PointStamped
from sensor_msgs import point_cloud2
from scipy.ndimage import gaussian_filter1d

# For sim mbes action client
import actionlib
from auv_2_ros.msg import FlsSimAction, FlsSimResult
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Quaternion
import tf
from tf2_geometry_msgs import do_transform_point


#TODO:
#1. OK auv_fls_model.py: action server that gives FLS reading upon request
# - a semi-OK Make a bool such that FLS sends 	set_aborted() if it is not ready to send a ping. Ie, from mission planner we can publish ushc that when we know we wont
#       have an auv infront of us, we can set the bool to false and the FLS will not send a ping. This to save computation. But for other cases, it's just always true.
# -OK Static tf fls - base_link somewhere (see where mbes - base_link is)
#2. -OK auv_motion_simple.cpp: a single request at a given transform and time to the action server above #1
#3. - OK auv_motion_simple_node.cpp: a node that at a certain timer interval calls the single request #2 with this specific rate
#4. - OK Integrate into all launch files
#5. OK Visualise BEAMS in rviz
#6. OK See why measurement is not published to topic
#7. Test scenarios

class FLSModel(object):

    def __init__(self):

        # # Load mesh
        # svp_path = rospy.get_param('~sound_velocity_prof')
        # mesh_path = rospy.get_param('~mesh_path')
        self.fls_horizontal_angle = rospy.get_param("~fls_horizontal_angle", 135)
        self.fls_vertical_angle = rospy.get_param("~fls_vertical_angle", 60)
        self.fls_horizontal_angle = np.deg2rad(self.fls_horizontal_angle)
        self.fls_vertical_angle = np.deg2rad(self.fls_vertical_angle)
        self.fls_max_range = rospy.get_param("~fls_max_range", 100) #meters
        self.vehicle_model = rospy.get_param("~vehicle_model", "hugin")
        self.num_auvs = rospy.get_param("~num_auvs", 1)
        self.fls_enable_topic = rospy.get_param("~fls_enable_topic", "/fls_sim_enable") #TODO: add in launch file 1a
        self.scan_area_marker_topic = rospy.get_param("~scan_area_marker_topic", "/sim/fls_scan_area") #TODO: add in launch file 1a
        self.namespace = rospy.get_param("~namespace", "hugin_0") 
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
        self.goal_header_stamp = None

        self.fls_frame_current_auv = self.namespace + "/fls_link"

        rospy.spin()


    # Action server to simulate FLS for the sim AUV
    def fls_as_cb(self, goal):
        self.goal_header_stamp = goal.map2fls_tf.header.stamp
        if self.fls_enabled:
            self.publish_fls_scan_area_to_rviz()
            tf_map2fls = goal.map2fls_tf
            r,theta = None,None
            for i in range(self.num_auvs): #TODO: find a faster way to do this, if it's an issue - rewrite in C++
                # print(self.namespace, " is looking for agent", i, " in the FLS fov")
                if self.namespace != "hugin_"+ str(i):
                    hugin_frame_i = "hugin_" + str(i) + "/base_link"
                    # tf_hugin2map = self.tf_buffer.lookup_transform("map", hugin_frame, self.goal_header.stamp, rospy.Duration(1.0))
                    try:
                        tf_hugin2fls = self.tf_buffer.lookup_transform(self.fls_frame_current_auv,hugin_frame_i, self.goal_header_stamp, rospy.Duration(1.0)) #self._multiply_transforms(tf_hugin2map,tf_map2fls)#tf_hugin2map.transform * tf_map2fls.transform #NOTE: Continue here, trasnform multiplication is issue here.
                        point = PointStamped() #point in origin
                        point.header.stamp = self.goal_header_stamp
                        point.header.frame_id = hugin_frame_i
                        # point = tf_hugin2fls*point
                        point = do_transform_point(point, tf_hugin2fls)
                        # x = point.point.x
                        # z = point.point.z
                        # point.point.x = z
                        # point.point.z = x
                        # print("FLS ping at point: ", point)
                        if self.point_in_fov(point.point):
                            # print("FLS ping at point: ", point)
                            r, theta = self.polar_coordinates(point.point.z,point.point.y)
                            # print("FLS ping at r = ", r, " theta = ", theta)
                            #break #TODO: handle the case with multiple agents in the fov. This is out of scope for the multi-agent thesis of Koray, but can be extended later here. If you do, remember to change definition of action message aswell
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                        rospy.logerr("Could not lookup transform from %s to %s" % (self.fls_frame_current_auv,hugin_frame_i))
                        rospy.logerr(e)
            if r is None or theta is None:
                self.as_ping.set_aborted()
                # print(self.namespace, " did not find any agents in the FLS fov")
            else:
                result = FlsSimResult()
                result.header.stamp = self.goal_header_stamp
                result.header.frame_id = self.fls_frame_current_auv
                result.range = Float32(r)
                result.angle = Float32(theta)
                self.as_ping.set_succeeded(result)
        else:
            #Don't do any calculation if FLS is disabled, save computation.
            self.as_ping.set_aborted()

    def _multiply_transforms(self, tf1, tf2):
        trans1 = tf1.transform.translation
        rot1 = tf1.transform.rotation
        trans1_mat = tf.transformations.translation_matrix(trans1)
        rot1_mat   = tf.transformations.quaternion_matrix(rot1)
        mat1 = np.dot(trans1_mat, rot1_mat)

        trans2 = tf2.transform.translation
        rot2 = tf2.transform.rotation
        trans2_mat = tf.transformations.translation_matrix(trans2)
        rot2_mat    = tf.transformations.quaternion_matrix(rot2)
        mat2 = np.dot(trans2_mat, rot2_mat)

        mat3 = np.dot(mat1, mat2)
        trans3 = tf.transformations.translation_from_matrix(mat3)
        rot3 = tf.transformations.quaternion_from_matrix(mat3)
        tf3 = Transform() 
        tf3.translation = trans3
        tf3.rotation = rot3
        return tf3

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
        # # in_fov = x >=0 and x<=R*np.cos(alpha/2) and y>=-R*np.sin(alpha/2) and y<=R*np.sin(alpha/2) and z>=-R*np.sin(beta/2) and z<=R*np.sin(beta/2)
        # in_fov = (
        #         z >= 0
        #         and z <= R
        #         and y >= -R * np.sin(alpha / 2)
        #         and y <= R * np.sin(alpha / 2)
        #         and x >= -R * np.sin(beta / 2)
        #         and x <= R * np.sin(beta / 2)
        #         )
        # Calculate polar coordinates (r, theta, phi) for the point (x, y, z)
        r_horizontal,theta_horizontal = self.polar_coordinates(z,y)
        r_vertical,theta_vertical = self.polar_coordinates(z,x)

        in_fov = (
                r_horizontal >= 0
                and r_horizontal <= R
                and r_vertical >= 0
                and r_vertical <= R
                and abs(theta_horizontal) <= alpha/2
                and abs(theta_vertical) <= beta/2
                )
        if in_fov:
            print("theta_horizontal: ", np.rad2deg(theta_horizontal), " theta_vertical: ", np.rad2deg(theta_vertical))
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
              
        phi_list = np.linspace(-self.fls_horizontal_angle/2,self.fls_horizontal_angle/2,5,endpoint=True)
        theta_list = np.linspace(-self.fls_vertical_angle/2,self.fls_vertical_angle/2,5,endpoint=True)
        # print("phi_list: ", np.rad2deg(phi_list))
        # print("theta_list: ", np.rad2deg(theta_list))
        marker_array = MarkerArray()
        id = 0
        for theta in theta_list:
            for phi in phi_list:
                # Create a line strip marker for the rectangle
                marker = Marker()
                marker.header.stamp = self.goal_header_stamp
                marker.header.frame_id = self.fls_frame_current_auv
                marker.type = Marker.LINE_STRIP
                marker.id = id
                id += 1
                marker.pose.orientation = Quaternion(0, 0, 0, 1)  # Set the quaternion for orientation

                # Set the scale of the marker (line width)
                marker.scale.x = 0.5  # You can adjust this value

                # Set the color 
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.3 #A bit see-through

                # Add points to the line strip marker to define the rectangle
                x = self.fls_max_range*np.sin(theta)
                y = self.fls_max_range*np.sin(phi)
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

