#!/usr/bin/env python3
import rospy
from subprocess import call, Popen
import numpy as np
import rospkg
import math
import pdb
from multi_agent.msg import AgentPath, AgentPathArray
import tf
from rviz_visualization.srv import DisplayRvizMessage, DisplayRvizMessageRequest
from std_msgs.msg import String, Time



class AUVSpawner():
    def __init__(self):
        self.num_auvs = rospy.get_param('num_auvs',1)
        self.mode = rospy.get_param('~mode','sim')
        self.dataset = rospy.get_param('~dataset','lost_targets')
        self.vehicle_model = rospy.get_param('vehicle_model','hugin')
        self.spawn_sep = rospy.get_param('spawn_separation',10)

        
        self.fls_horizontal_angle = rospy.get_param("~fls_horizontal_angle", 135)
        self.fls_vertical_angle = rospy.get_param("~fls_vertical_angle", 60)
        self.fls_max_range = rospy.get_param("~fls_max_range", 50) #meters
        self.fls_range_std = rospy.get_param("~fls_range_std") #meters
        self.fls_angle_std = rospy.get_param("~fls_angle_std") #radians
        rospack = rospkg.RosPack()
        self.launch_file = rospy.get_param('~auv_launch_file',rospack.get_path('auv_model') + '/launch/auv_environment.launch')

        self.pattern_generator = rospy.get_param('pattern_generation','true')

        #Publishers and subscribers used for pattern generation
        self.path_array_topic = rospy.get_param('path_array_topic', '/multi_agent/path_array')
        self.spawn_pos_path_array_topic = rospy.get_param('path_array_spawn_pos_topic', '/multi_agent/spawn_pos/path_array')

        self.path_array_pub = rospy.Publisher(self.path_array_topic, AgentPathArray, queue_size=1)
        self.spawn_pos_paths_sub = rospy.Subscriber(self.spawn_pos_path_array_topic, AgentPathArray, self.callback)

        self.t_start_pub = rospy.Publisher('/multi_agent/t_start', Time, queue_size=1)


        self.message_srv = rospy.ServiceProxy('/display_rviz_message', DisplayRvizMessage)
        rospy.wait_for_service('/display_rviz_message',timeout=5)

        # self.paths = AgentPathArray()

        

        rospy.loginfo(str("Preparing to spawn '%s' AUVs..." % str(self.num_auvs)))

        #spawn_points should be an array consisting of evenly spaced points along the x-axis with 2*self.spawn_sep spacing
        # spacing = 2*self.spawn_sep
        # elements = self.num_auvs-2
        # spawn_points = np.linspace(0, self.num_auvs*self.spawn_sep, elements, endpoint=True)
        # print(spawn_points)
        
        if not self.pattern_generator: #Spawn auvs in a line only if pattern generation is disabled, eg not using lawn mower pattern
            for i in range(self.num_auvs):
                rospy.loginfo(str("Spawning AUV: "+ str(i)))
                namespace = self.vehicle_model + '_' + str(i)
                
                #TODO: 
                #1. OK - Spawn auvs in correct position and orientation for the first wps they'll be given (i.e. the first wps in the mission plan) 
                #2. OK - Make turns tighter, such that auvs don't make large loops when turning
                #3. Update documentation in README including:
                #  - How to use the pattern generator
                #  - Dependencies to install (Özer's stuff)
                #4. OK Display text in rviz
                #5. Add Integral part to w2w_planner

                        
                


                x = i*self.spawn_sep
                yaw = math.pi/2

                self.spawn_auv(x,0,0,0,0,yaw,namespace)
                
            # rospy.sleep(3)

        rospy.spin()

    def callback(self,msg):
        for AgentPath_instance in msg.path_array:
            agent_path = AgentPath_instance.path
            agent_id = AgentPath_instance.agent_id
            start_pose = agent_path.poses[0].pose
            startup_distance = 0 #meters #this allows the auvs to start up a bit before the first waypoint, which avoids uneccessary startup movements to reach first wp
            roll,pitch,yaw = tf.transformations.euler_from_quaternion([start_pose.orientation.x,start_pose.orientation.y,start_pose.orientation.z,start_pose.orientation.w])
            x,y,z = start_pose.position.x, start_pose.position.y-startup_distance, start_pose.position.z
            namespace = self.vehicle_model + '_' + str(agent_id)
            self.spawn_auv(x,y,z,roll,pitch,yaw,namespace)
        #wait for user to press enter before publishing paths
        self.display_message_in_rviz("Press 'Enter' in terminal to start survey, once all AUVs are spawned...")
        rospy.loginfo("Press Enter to publish paths...")
        input()
        self.t_start_pub.publish(rospy.Time.now())
        self.path_array_pub.publish(msg)
        self.display_message_in_rviz("Survey started!")
        self.display_message_in_rviz("")



    def display_message_in_rviz(self, message):
        self.message_srv(DisplayRvizMessageRequest(String(message)))
            
            

    def spawn_auv(self,x,y,z,roll,pitch,yaw,namespace):
        proc = Popen(["roslaunch", self.launch_file, 
                            "mode:=" + self.mode,
                            "dataset:=" + self.dataset,
                            "namespace:=" + namespace,
                            "x:=" + str(x), #This and yaw below are for the initial pose, such that the auvs are spawned along the x-axis heading looking along the y-axis
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
                            ])
        rospy.loginfo(str("Spawned AUV: "+ str(namespace)) + str(" at x: " + str(x) + " y: " + str(y) + " z: " + str(z) + " roll: " + str(roll) + " pitch: " + str(pitch) + " yaw: " + str(yaw)))
if __name__ == '__main__':

    rospy.init_node('auv_spawner')

    try:
        launcher = AUVSpawner()

    except rospy.ROSInterruptException:
        rospy.logerr("Couldn't launch auv_spawner node")
