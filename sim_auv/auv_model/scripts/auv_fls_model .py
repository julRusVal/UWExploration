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
from sensor_msgs import point_cloud2
from scipy.ndimage import gaussian_filter1d

# For sim mbes action client
import actionlib
from auv_2_ros.msg import FlsSimAction, FlsSimResult

#TODO:
#1. auv_fls_model.py: action server that gives FLS reading upon request
# - Make a bool such that FLS sends 	set_aborted() if it is not ready to send a ping. Ie, from mission planner we can publish ushc that when we know we wont
#       have an auv infront of us, we can set the bool to false and the FLS will not send a ping. This to save computation. But for other cases, it's just always true.
#2. auv_motion_simple.cpp: a single request at a given transform and time to the action server above #1
#3. auv_motion_simple_node.cpp: a node that at a certain timer interval calls the single request #2 with this specific rate
#4. Integrate into all launch files

class mbes_model(object):

    def __init__(self):

        # # Load mesh
        # svp_path = rospy.get_param('~sound_velocity_prof')
        # mesh_path = rospy.get_param('~mesh_path')
        # self.mbes_angle = rospy.get_param("~mbes_open_angle", np.pi/180. * 60.)
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
        self.as_ping = actionlib.SimpleActionServer(sim_fls_as, MbesSimAction,
                                                    execute_cb=self.fls_as_cb, auto_start=server_mode)

        rospy.spin()


    # Action server to simulate MBES for the sim AUV
    def fls_as_cb(self, goal):

        # Unpack goal
        p_mbes = [goal.mbes_pose.transform.translation.x,
                  goal.mbes_pose.transform.translation.y,
                  goal.mbes_pose.transform.translation.z]
        r_mbes = quaternion_matrix([goal.mbes_pose.transform.rotation.x,
                                    goal.mbes_pose.transform.rotation.y,
                                    goal.mbes_pose.transform.rotation.z,
                                    goal.mbes_pose.transform.rotation.w])[0:3, 0:3]

        # IGL sim ping
        # The sensor frame on IGL needs to have the z axis pointing opposite from the actual sensor direction
        R_flip = rotation_matrix(np.pi, (1, 0, 0))[0:3, 0:3]
        mbes = self.draper.project_mbes(np.asarray(p_mbes), r_mbes,
                                        goal.beams_num.data, self.mbes_angle)

        mbes = mbes[::-1]  # Reverse beams for same order as real pings

        # Transform points to MBES frame (same frame than real pings)
        rot_inv = r_mbes.transpose()
        p_inv = rot_inv.dot(p_mbes)
        mbes = np.dot(rot_inv, mbes.T)
        mbes = np.subtract(mbes.T, p_inv)

        # Add noise
        # mbes = gaussian_filter1d(mbes , sigma=0.5)


        # Pack result
        mbes_cloud = self.pack_cloud(self.mbes_frame, mbes)
        result = MbesSimResult()
        result.sim_mbes = mbes_cloud
        self.as_ping.set_succeeded(result)


    # Create PointCloud2 msg out of ping
    def pack_cloud(self, frame, mbes):
        mbes_pcloud = PointCloud2()
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = frame
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)]

        mbes_pcloud = point_cloud2.create_cloud(header, fields, mbes)

        return mbes_pcloud


if __name__ == '__main__':

    rospy.init_node('auv_mbes_model', disable_signals=False)
    try:
        mbes_model()
    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch mbes_model")
        pass

