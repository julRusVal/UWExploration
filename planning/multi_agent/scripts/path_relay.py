#!/usr/bin/env python3
import rospy
from subprocess import call, Popen
import numpy as np
import rospkg
from multi_agent.msg import AgentPath, AgentPathArray
from nav_msgs.msg import Path
import tf2_ros
from geometry_msgs.msg import TransformStamped

class PathRelay():
    def __init__(self):
        self.num_auvs = rospy.get_param('num_auvs',1)
        self.vehicle_model = rospy.get_param('vehicle_model','hugin')
        # self.pattern_generator = rospy.get_param('pattern_generation','true')

        self.path_array_topic = rospy.get_param('path_array_topic', '/multi_agent/path_array')
        self.paths_sub = rospy.Subscriber(self.path_array_topic, AgentPathArray, self.path_array_cb)
        self.paths = AgentPathArray()
        self.pub_dict = {}

        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # rospy.loginfo("Enabling AUV navigation...")

        #Create publishers for each AUV
        for i in range(self.num_auvs):
            namespace = self.vehicle_model + '_' + str(i)
            self.pub_dict[i] = rospy.Publisher(namespace + '/waypoints', Path, queue_size=1)

        rate = 10 #Hz
        rate = rospy.Rate(rate)

        while not rospy.is_shutdown():
            
            while len(self.paths.path_array) > 0:
                AgentPath_instance = self.paths.path_array.pop()
                agent_path = AgentPath_instance.path
                agent_id = AgentPath_instance.agent_id
                # if self.pattern_generator:
                #     start_pose = agent_path.poses[0].pose
                #     self.teleport_agent_to_pose(agent_id, start_pose)
                self.pub_dict[agent_id].publish(agent_path)
                rospy.loginfo(str("Published lawn mover path for agent: " + str(agent_id)))
                rate.sleep()
        

    def path_array_cb(self, msg):
        # rospy.loginfo("Received AgentPathArray")
        self.paths = msg

    # def teleport_agent_to_pose(self, agent_id, pose):
    #     """This method publishes a static transform between each AUV and the map such that they teleport to the correct starting pose for their pattern, found in aelf.paths.path_array"""
    #     # Create a static transform between the AUV frame and the map frame
    #     transform_stamped = TransformStamped()
    #     transform_stamped.header.stamp = rospy.Time.now()
    #     transform_stamped.header.frame_id = "map" 
    #     transform_stamped.child_frame_id = f"{self.vehicle_model}_{agent_id}/odom"

    #     # Set the translation and rotation based on the input pose
    #     transform_stamped.transform.translation.x = pose.position.x
    #     transform_stamped.transform.translation.y = pose.position.y
    #     transform_stamped.transform.translation.z = pose.position.z
    #     transform_stamped.transform.rotation = pose.orientation

    #     # Publish the static transform
    #     t = rospy.Time.now()
    #     while rospy.Time.now() < t + rospy.Duration(5):
    #         transform_stamped.header.stamp = rospy.Time.now()
    #         self.tf_broadcaster.sendTransform(transform_stamped)
        
    #     print("TELEPORT with transform: ", transform_stamped)

if __name__ == '__main__':

    rospy.init_node('path_relay')

    try:
        launcher = PathRelay()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch path_relay node")
