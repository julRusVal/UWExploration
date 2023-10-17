#!/usr/bin/env python3

from __future__ import division, print_function

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped
from move_base_msgs.msg import MoveBaseFeedback, MoveBaseResult, MoveBaseAction
import actionlib
import rospy
import tf
from std_msgs.msg import Float64, Header, Bool
import math
import dubins
import pdb
from std_msgs.msg import Time
import time

class W2WPathPlanner(object):

    # create messages that are used to publish feedback/result
    _feedback = MoveBaseFeedback()
    _result = MoveBaseResult()

    def execute_cb(self, goal):

        rospy.loginfo("Goal received")

        success = True
        self.nav_goal = goal.target_pose.pose
        self.nav_goal_frame = goal.target_pose.header.frame_id
        if self.nav_goal_frame is None or self.nav_goal_frame == '':
            rospy.logwarn("Goal has no frame id! Using map by default")
            self.nav_goal_frame = self.map_frame

        r = rospy.Rate(10.)  # 10hz
        counter = 0
        while not rospy.is_shutdown() and self.nav_goal is not None:

            # Preempted
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                self.nav_goal = None

                # Stop thruster
                self.motion_command(0., 0., 0.)
                break

            # Transform goal map --> base frame
            goal_point = PointStamped()
            goal_point.header.frame_id = self.nav_goal_frame
            goal_point.header.stamp = rospy.Time(0)
            goal_point.point.x = self.nav_goal.position.x
            goal_point.point.y = self.nav_goal.position.y
            goal_point.point.z = self.nav_goal.position.z

            # goal_pose = PoseStamped()
            # goal_pose.header.frame_id = self.nav_goal_frame
            # goal_pose.header.stamp = rospy.Time(0)
            # goal_pose.pose.position = self.nav_goal.position
            # goal_pose.pose.orientation = self.nav_goal.orientation

            

            try:
                goal_point_local = self.listener.transformPoint(
                    self.base_frame, goal_point)
                

                #-----------------------------------------------------------
                
                # goal_pose_local = self.listener.transformPose(
                #     self.base_frame, goal_pose)
                
                # # #plot goal point vs base frame
                # # plt.figure()
                # # plt.plot(goal_pose_local.point.x, goal_pose_local.point.y, 'ro')
                # # plt.plot(0,0,'bo')
                # # plt.axis('equal')
                # # plt.show()

                
                # #dubins tests
                # q0 = (0,0,0)
                # goal_heading = tf.transformations.euler_from_quaternion([goal_pose_local.pose.orientation.x,goal_pose_local.pose.orientation.y,goal_pose_local.pose.orientation.z,goal_pose_local.pose.orientation.w])[2]
                # q1 = (goal_pose_local.pose.position.x, goal_pose_local.pose.position.y, goal_heading)
                # turning_radius = 1.0
                # step_size = 0.5

                # path = dubins.shortest_path(q0, q1, turning_radius)
                # configurations, _ = path.sample_many(step_size)
                # # Plot
                # configurations_array = np.array(configurations)
                # if len(configurations_array) > 0:
                #     plt.figure()
                #     plt.plot(configurations_array[:,0], configurations_array[:,1])
                #     plt.axis('equal')
                #     plt.show()
                #-----------------------------------------------------------
                
                #Compute throttle error
                # throttle_level = min(self.max_throttle, np.linalg.norm(
                #     np.array([goal_point_local.point.x + goal_point_local.point.y])))
                
                throttle_error = np.linalg.norm(np.array([goal_point_local.point.x + goal_point_local.point.y]))
                thrust_error = math.atan2(goal_point_local.point.y,goal_point_local.point.x)

                

                if self.t:
                    dt = (rospy.Time.now() - self.t).to_sec()
                    # print("dt: ", dt)
                    self.int_throttle_error += throttle_error * dt
                    self.int_thrust_error += thrust_error * dt
                    der_throttle_error = (throttle_error - self.prev_throttle_error) / dt
                    der_thrust_error = (thrust_error - self.prev_thrust_error) / dt
                else:
                    der_throttle_error = 0
                    der_thrust_error = 0
                
                # Nacho: no real need to adjust the throttle 
                # Koray: That's true for single agent missions, but for multi-agent missions it's important that an agent far away from its goal will speed up to "catch up" with it's neighbour.
                # It's also important to we don't throttle too hard when we're close to a wp, but need to yaw a lot. Throttling too much in this situation will result in large circles around wps, which can 
                # delay an agent in relation to its neighbours.

                # throttle_level = self.max_throttle
                # Compute thrust error
                # alpha = math.atan2(goal_point_local.point.y,
                #                 goal_point_local.point.x)
                sign = np.copysign(1, thrust_error)
                yaw_setpoint = (self.P_thrust * thrust_error + 
                                self.I_thrust * self.int_thrust_error +
                                self.D_thrust * der_thrust_error)
                throttle_level = (self.P_throttle * throttle_error + 
                                  self.I_throttle * self.int_throttle_error +
                                  self.D_throttle * der_throttle_error)
                # throttle_level = self.max_throttle
                yaw_setpoint = sign * min(self.max_thrust, abs(yaw_setpoint))
               
                throttle_level = min(self.max_throttle, throttle_level)

                #Time boosting
                # if len(self.delta_t_array) > 0:
                #     delta_t_ij = self.delta_t_array.pop(0)
                #     if delta_t_ij:
                #         print("time boosting")
                #         if self.t_start is None:
                #             self.t_start = time.time()
                #         boost = 1.0
                #         delta_t_ik = time.time() - self.t_start
                #         throttle_level = min(v for v in [3*self.max_throttle,(delta_t_ij/(delta_t_ij-delta_t_ik)-1)*boost] if v > 0)

                if self.t_arrival:
                    boost = 1.0
                    if self.t_start is None:
                        self.t_start = time.time()
                    t = time.time()-self.t_start
                    print("arrival time: ", self.t_arrival)
                    print("current time: ", t)
                    print("old arrival time: ", self.t_arrival_old)
                    time_boost = ((self.t_arrival-self.t_arrival_old)/(self.t_arrival-t)-1)*boost
                    time_boost = max(time_boost,self.time_boost_old)
                    throttle_level = max(throttle_level,time_boost)
                    throttle_level = min(throttle_level,4*self.max_throttle)
                    print("throttle level: ", throttle_level)
                    self.time_boost_old = time_boost
                    self.early = max(0,self.t_arrival-t)

                #TODO:
                #1. Create common time tags for all agents in pattern generator, they all should have common time tags 
                #2. Use these common time tags to boost the throttle of agents that are behind in the pattern
                #3. Edit aux launch file to generate cool lookingmaps in rviz
                #4. Start looking into PF
                    


                self.motion_command(throttle_level, yaw_setpoint, 0.)
                self.t = rospy.Time.now()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("Transform to base frame not available yet")
            pass

            # Publish feedback
            if counter % 10 == 0:
                self._as.publish_feedback(self._feedback)

            counter += 1
            r.sleep()

        # Stop thruster
        t = rospy.Time.now()
        while rospy.Time.now() - t < rospy.Duration(self.early): #catch wait for remaining auvs, you're too early
            self.motion_command(0.,0.,0.)
            rospy.loginfo("waiting for remaining auvs to catch up")

        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._reset_params()
        self._as.set_succeeded(self._result)

    def motion_command(self, throttle_level, thruster_angle, inclination_angle):
        
        incl = Float64()
        throttle = Float64()
        thrust = Float64()

        throttle.data = throttle_level
        thrust.data = thruster_angle
        incl.data = inclination_angle
        self.thruster_pub.publish(thrust) #yaw angular acceleration
        self.inclination_pub.publish(incl)
        self.throttle_pub.publish(throttle) #linear velocity
        #when pitch is zero, this is a differential drive robot

    def timer_callback(self, event):
        if self.nav_goal is None:
            #rospy.loginfo_throttle(30, "Nav goal is None!")
            return

        # Check if the goal has been reached
        try:
            (trans, rot) = self.listener.lookupTransform(
                self.nav_goal_frame, self.base_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        start_pos = np.array(trans)
        end_pos = np.array(
            [self.nav_goal.position.x, self.nav_goal.position.y, self.nav_goal.position.z])

        rospy.logdebug("diff " + str(np.linalg.norm(start_pos - end_pos)))
        if np.linalg.norm(start_pos - end_pos) < self.goal_tolerance:
            # Goal reached
            self.nav_goal = None

    def delta_t_cb(self, msg):
        self.delta_t_array.append(msg.data.secs)

    def arrival_time_cb(self, msg):
        self.t_arrival = msg.data.secs
    
    def _reset_params(self):
        self.t = None
        self.int_throttle_error = 0
        self.int_thrust_error = 0
        self.prev_throttle_error = 0
        self.prev_thrust_error = 0
        # self.t_start = None
        if self.t_arrival is not None:
            self.t_arrival_old = self.t_arrival
        self.t_arrival = None
        self.time_boost_old = 0
        self.early = 0


    def __init__(self, name):
        self._action_name = name

        self.goal_tolerance = rospy.get_param('~goal_tolerance', 1.)
        self.max_throttle = rospy.get_param('~max_throttle', 2.)
        self.max_thrust = rospy.get_param('~max_thrust', 0.5)
        self.map_frame = rospy.get_param('~map_frame', 'map') 
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.throttle_top = rospy.get_param('~throttle_cmd', '/throttle')
        self.thruster_top = rospy.get_param('~thruster_cmd', '/thruster')
        self.inclination_top = rospy.get_param('~inclination_cmd', '/inclination')
        self.as_name = rospy.get_param('~path_planner_as', 'path_planner')
        
        self.P_throttle = rospy.get_param('~P_throttle', 2.0)
        self.I_throttle = rospy.get_param('~I_throttle', 0.0)
        self.D_throttle = rospy.get_param('~D_throttle', 0.0)
        self.P_thrust = rospy.get_param('~P_thrust', 1.0)
        self.I_thrust = rospy.get_param('~I_thrust', 1.0)
        self.D_thrust = rospy.get_param('~D_thrust', 0.8)

        self.t = None
        self.int_throttle_error = 0
        self.int_thrust_error = 0
        self.prev_throttle_error = 0
        self.prev_thrust_error = 0

        self.nav_goal = None

        self.delta_t_array = []
        self.t_start = None
        self.t_arrival = None
        self.t_arrival_old = 0
        self.time_boost_old = 0
        self.early = 0

        self.listener = tf.TransformListener()
        rospy.Timer(rospy.Duration(1/20), self.timer_callback)

        self.throttle_pub = rospy.Publisher(self.throttle_top, Float64, queue_size=1)
        self.thruster_pub = rospy.Publisher(self.thruster_top, Float64, queue_size=1)
        self.inclination_pub = rospy.Publisher(self.inclination_top, Float64, queue_size=1)
        self.delta_t_sub = rospy.Subscriber('delta_t', Time, self.delta_t_cb)
        self.arrival_time_sub = rospy.Subscriber('arrival_time', Time, self.arrival_time_cb)

        self._as = actionlib.SimpleActionServer(
            self.as_name, MoveBaseAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("Announced action server with name: %s", self.as_name)

        rospy.spin()


if __name__ == '__main__':

    rospy.init_node('w2w_path_planner')
    planner = W2WPathPlanner(rospy.get_name())
