#!/usr/bin/env python3

import rospy
from subprocess import Popen
import numpy as np
import rospkg
import math
import tf
from multi_agent.msg import AgentPathArray
from rviz_visualization.srv import DisplayRvizMessage, DisplayRvizMessageRequest
from std_msgs.msg import String, Time

# This is the node responsible for launching different aspects of multi agent scenarios
# Script allows for using seperate launchers for agent, maps, and comms

class AUVSpawner():
    def __init__(self):
        self.num_auvs = rospy.get_param('num_auvs', 1)
        self.mode = rospy.get_param('~mode','sim')
        # launcher mode: agent, map, comm, or all
        self.spawner_mode = rospy.get_param('~spawner_mode', 'all')
        rospy.loginfo(f"[{rospy.get_name()}] Spawner mode: {self.spawner_mode}")
        self.dataset = rospy.get_param('~dataset', 'lost_targets')
        self.vehicle_model = rospy.get_param('vehicle_model', 'hugin')
        self.spawn_sep = rospy.get_param('spawn_separation', 10)

        # Set up parameters for the spawner status
        # This is only used when agents are being spawned
        if self.spawner_mode == 'agent' or self.spawner_mode == 'all':
            self.spawner_status_param_name = rospy.get_param('~spawner_status_param_name', '/spawner_status')
            rospy.set_param(self.spawner_status_param_name, False)
        self.spawn_counter = 0

        # AUV model parameters
        self.fls_horizontal_angle = rospy.get_param("~fls_horizontal_angle", 135)
        self.fls_vertical_angle = rospy.get_param("~fls_vertical_angle", 60)
        self.fls_max_range = rospy.get_param("~fls_max_range", 50)  # meters
        self.fls_range_std = rospy.get_param("~fls_range_std")  # meters
        self.fls_angle_std = rospy.get_param("~fls_angle_std")  # radians
        self.odom_period = rospy.get_param("~odom_period")  # seconds
        self.fls_period = rospy.get_param("~fls_meas_period")  # seconds
        self.mbes_period = rospy.get_param("~mbes_meas_period")  # seconds

        # Mapping params are set mostly set in the relevant launch file for now

        # Comms parameters are mostly set in the relevant launch file for now

        # Launch files
        rospack = rospkg.RosPack()
        # Agent
        self.launch_file = rospy.get_param('~auv_launch_file', rospack.get_path('auv_model') + '/launch/auv_environment.launch')
        # Map
        self.map_launch_file = rospy.get_param('~map_launch_file', rospack.get_path('gp_mapping') + '/launch/ma_gp_mapping.launch')
        # Comm
        default_comms_launch_path = f"{rospack.get_path('multi_agent_comms')}/launch/ma_comms.launch"
        self.comms_launch_file = rospy.get_param('~comms_launch_file', default_comms_launch_path)

        
        self.pattern_generator = rospy.get_param('pattern_generation', 'true')

        # Publishers and subscribers for pattern generation
        self.path_array_topic = rospy.get_param('path_array_topic', '/multi_agent/path_array')
        self.spawn_pos_path_array_topic = rospy.get_param('path_array_spawn_pos_topic', '/multi_agent/spawn_pos/path_array')

        self.path_array_pub = rospy.Publisher(self.path_array_topic, AgentPathArray, queue_size=1)
        self.spawn_pos_paths_sub = rospy.Subscriber(self.spawn_pos_path_array_topic, AgentPathArray, self.callback)

        self.t_start_pub = rospy.Publisher('/multi_agent/t_start', Time, queue_size=1)

        self.message_srv = rospy.ServiceProxy('/display_rviz_message', DisplayRvizMessage)
        rospy.wait_for_service('/display_rviz_message', timeout=5)

        rospy.loginfo(f"Launching AUVSpawner in {self.mode} mode...")

        # if self.mode == "agent" or self.mode == 'all':
        #     self.spawn_agents()
        # if self.mode == "map" or self.mode == 'all':
        #     self.spawn_maps()
        # if self.mode == "comms" or self.mode == 'all':
        #     self.spawn_comms()
        
        if not self.spawner_mode in ['agent', 'map', 'comms', 'all'] :
            rospy.logerr(f"Unknown mode: {self.spawner_mode}. Use 'agent', 'map', 'comms' or 'all'.")

        rospy.spin()

    def spawn_agents(self):
        """ Spawns agent AUVs in the simulation """
        rospy.loginfo(f"Spawning {self.num_auvs} AUV agents...")

        if not self.pattern_generator:
            for i in range(self.num_auvs):
                namespace = self.vehicle_model + '_' + str(i)
                x = i * self.spawn_sep
                yaw = math.pi / 2
                self.spawn_auv(x, 0, 0, 0, 0, yaw, namespace)
                self.spawn_counter += 1

        if self.spawn_counter == self.num_auvs:
            rospy.set_param(self.spawner_status_param_name, True)

    def spawn_maps(self):
        """ Spawns mapping nodes for AUVs """
        rospy.loginfo(f"Spawning maps for {self.num_auvs} AUVs...")

        for i in range(self.num_auvs):
            namespace = self.vehicle_model + '_' + str(i)
            x = i * self.spawn_sep
            yaw = math.pi / 2
            self.spawn_auv_map(x, 0, 0, 0, 0, yaw, namespace)

    def spawn_comms(self):
        """ Placeholder for launching communication nodes """
        rospy.loginfo("Launching communication nodes ...")
        rospy.loginfo(f"[Format vehicle_model]_[agent_id]")
        for agent_id in range(self.num_auvs):
            namespace = self.vehicle_model + '_' + str(agent_id)
            self.spawn_auv_comm(agent_id=agent_id, agent_count=self.num_auvs, namespace=namespace)

    def callback(self, msg):
        rospy.loginfo(f"[{rospy.get_name()}] Preparing to Spawn: {self.spawner_mode}")
        
        for AgentPath_instance in msg.path_array:
            agent_path = AgentPath_instance.path
            agent_id = AgentPath_instance.agent_id
            start_pose = agent_path.poses[0].pose
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(
                [start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w])
            x, y, z = start_pose.position.x, start_pose.position.y, start_pose.position.z
            namespace = self.vehicle_model + '_' + str(agent_id)

            if self.spawner_mode == "agent" or self.spawner_mode == "all":
                self.spawn_auv(x, y, z, roll, pitch, yaw, namespace)
                self.spawn_counter += 1
            
            if self.spawner_mode == "map" or self.spawner_mode == "all":
                self.spawn_auv_map(x, y, z, roll, pitch, yaw, namespace)

            if self.spawner_mode == "comms" or self.spawner_mode == "all":
                self.spawn_auv_comm(agent_id=agent_id, agent_count=self.num_auvs, namespace=namespace)

            
        if self.spawner_mode == "agent" or self.spawner_mode == "all":
            rospy.set_param(self.spawner_status_param_name, True)

            self.t_start_pub.publish(rospy.Time.now())
            self.path_array_pub.publish(msg)
            self.display_message_in_rviz("Survey started!")

    def display_message_in_rviz(self, message):
        self.message_srv(DisplayRvizMessageRequest(String(message)))

    def spawn_auv(self, x, y, z, roll, pitch, yaw, namespace):
        """ Spawns an AUV in the simulation environment """
        rospy.loginfo(f"Spawning AUV - Namespace: {namespace} - Mode: {self.mode}")
        Popen(["roslaunch", self.launch_file,
               "mode:=" + self.mode,
               "dataset:=" + self.dataset,
               "namespace:=" + namespace,
               "x:=" + str(x),
               "y:=" + str(y),
               "z:=" + str(z),
               "roll:=" + str(roll),
               "pitch:=" + str(pitch),
               "yaw:=" + str(yaw),
               "fls_horizontal_angle:=" + str(self.fls_horizontal_angle),
               "fls_vertical_angle:=" + str(self.fls_vertical_angle),
               "fls_max_range:=" + str(self.fls_max_range),
               "fls_range_std:=" + str(self.fls_range_std),
               "fls_angle_std:=" + str(self.fls_angle_std),
               "vehicle_model:=" + self.vehicle_model,
               "num_auvs:=" + str(self.num_auvs),
               "odom_rate:=" + str(self.odom_period),
               "fls_meas_period:=" + str(self.fls_period),
               "meas_rate:=" + str(self.mbes_period)])

    def spawn_auv_map(self, x, y, z, roll, pitch, yaw, namespace):
        """ Spawns an AUV map node """
        rospy.loginfo(f"Spawning map for AUV - Namespace: {namespace} - Mode: {self.mode}")
        Popen(["roslaunch", self.map_launch_file,
               "mode:=" + self.mode,
               "dataset:=" + self.dataset,
               "namespace:=" + namespace,
               "x:=" + str(x),
               "y:=" + str(y),
               "z:=" + str(z),
               "roll:=" + str(roll),
               "pitch:=" + str(pitch),
               "yaw:=" + str(yaw)])

    def spawn_auv_comm(self, agent_id, agent_count, namespace):
        """ Spawns an AUV comms node """
        rospy.loginfo(f"Spawning comms for AUV - Namespace: {namespace} - Mode: {self.mode}")
        rospy.loginfo(f"Launch file: {self.comms_launch_file}")
        Popen(["roslaunch", self.comms_launch_file,
               "namespace:=" + namespace,
               "mode:=" + self.mode,
               "agent_id:=" + str(agent_id),
               "agent_count:=" + str(agent_count)])


if __name__ == '__main__':
    rospy.init_node('auv_spawner')
    try:
        launcher = AUVSpawner()
    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch auv_spawner node")
