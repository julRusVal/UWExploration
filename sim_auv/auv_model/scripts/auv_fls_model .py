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
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from sensor_msgs import point_cloud2
from scipy.ndimage import gaussian_filter1d

# For sim mbes action client
import actionlib
from auv_2_ros.msg import FlsSimAction, FlsSimResult
import tf2_ros

#TODO:
#1. auv_fls_model.py: action server that gives FLS reading upon request
# - Make a bool such that FLS sends 	set_aborted() if it is not ready to send a ping. Ie, from mission planner we can publish ushc that when we know we wont
#       have an auv infront of us, we can set the bool to false and the FLS will not send a ping. This to save computation. But for other cases, it's just always true.
# - Static tf fls - base_link somewhere (see where mbes - base_link is)
#2. auv_motion_simple.cpp: a single request at a given transform and time to the action server above #1
#3. auv_motion_simple_node.cpp: a node that at a certain timer interval calls the single request #2 with this specific rate
#4. Integrate into all launch files

class FLSModel(object):

    def __init__(self):

        # # Load mesh
        # svp_path = rospy.get_param('~sound_velocity_prof')
        # mesh_path = rospy.get_param('~mesh_path')
        self.fls_horizontal_angle = rospy.get_param("~fls_horizonntal_angle", np.deg2rad(135))
        self.fls_vertical_angle = rospy.get_param("~fls_vertical_angle", np.deg2rad(60))
        self.fls_max_range = rospy.get_param("~fls_max_range", 100) #meters
        self.vehicle_model = rospy.get_param("~vehicle_model", "hugin")
        self.num_auvs = rospy.get_param("~num_auvs", 1)
        # Initialize the TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # self.mbes_frame = rospy.get_param(
        #     '~mbes_link', 'mbes_link')  # mbes frame_id

        # if svp_path.split('.')[1] != 'cereal':
        #     sound_speeds = csv_data.csv_asvp_sound_speed.parse_file(svp_path)
        # else:
        #     sound_speeds = csv_data.csv_asvp_sound_speed.read_data(svp_path)

        # data = np.load(mesh_path)
        # V, F, bounds = data['V'], data['F'], data['bounds']
        # print("Mesh loaded")

        # # Create draper
        # self.draper = base_draper.BaseDraper(V, F, bounds, sound_speeds)
        # self.draper.set_ray_tracing_enabled(False)
        # data = None
        # V = None
        # F = None
        # bounds = None
        # sound_speeds = None
        # print("draper created")
        # print("Size of draper: ", sys.getsizeof(self.draper))

        # # Action server for MBES pings sim (necessary to be able to use UFO maps as well)
        sim_fls_as = rospy.get_param('~fls_sim_as', '/fls_sim_server')
        server_mode = rospy.get_param("~server_mode", False)
        self.as_ping = actionlib.SimpleActionServer(sim_fls_as, FlsSimAction,
                                                    execute_cb=self.fls_as_cb, auto_start=server_mode)

        rospy.spin()


    # Action server to simulate FLS for the sim AUV
    def fls_as_cb(self, goal):

        tf_map2fls = goal.map2fls_tf

        for i in range(self.num_auvs): #TODO: find a faster way to do this, if it's an issue - rewrite in C++
            hugin_frame = "hugin_" + str(i) + "/base_link"
            tf_hugin2map = self.tf_buffer.lookup_transform("map", hugin_frame, rospy.Time.now(), rospy.Duration(1.0))
            tf_hugin2fls = tf_hugin2map * tf_map2fls
            point = Point()
            point = tf_hugin2fls*point
            if self.point_in_fov(point):
                break
        
        result = FlsSimResult()
        result.header
        result.range
        result.angle
        self.as_ping.set_succeeded(result)
    
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
        if in_fov:
            



if __name__ == '__main__':

    rospy.init_node('auv_fls_model', disable_signals=False)
    try:
        FLSModel()
    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch auv_fls_model")
        pass

