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

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = self.nav_goal_frame
            goal_pose.header.stamp = rospy.Time(0)
            goal_pose.pose.position = self.nav_goal.position
            goal_pose.pose.orientation = self.nav_goal.orientation

            

            try:
                goal_point_local = self.listener.transformPoint(
                    self.base_frame, goal_point)
                
                x = goal_point_local.point.x
                y = goal_point_local.point.y

                throttle_error = np.linalg.norm(np.array([x + y]))
                thrust_error = math.atan2(y,x)
                
                if self.do_max_turn is None:
                    radius = self.dubins_turning_radius
                    h = 0
                    if y >= 0: #= to include the case when wp is straight infront of the auv
                        self.k = radius
                    elif y < 0:
                        self.k = -radius
                    
                    self.do_max_turn = self._point_on_circle(x,y,radius,h,self.k)
                    rospy.loginfo("Do max turn: %s", self.do_max_turn)

                
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
                
                # if not self.do_max_turn or self.wp_follower_type != 'simple_artificial':
                #     self.goal_tolerance = self.goal_tolerance_original
                #     yaw_setpoint = (self.P_thrust * thrust_error + 
                #                     self.I_thrust * self.int_thrust_error +
                #                     self.D_thrust * der_thrust_error)
                #     sign = np.copysign(1, thrust_error)
                #     yaw_setpoint = sign * min(self.max_thrust, abs(yaw_setpoint))

                if self.do_max_turn and self.wp_follower_type == 'simple_artificial':
                    self.goal_tolerance = self.goal_tolerance_max_turn
                    yaw_setpoint = np.copysign(self.max_thrust,self.k) # Do a maximum turn
                else:
                    self.goal_tolerance = self.goal_tolerance_original
                    yaw_setpoint = (self.P_thrust * thrust_error + 
                                    self.I_thrust * self.int_thrust_error +
                                    self.D_thrust * der_thrust_error)
                    sign = np.copysign(1, thrust_error)
                    yaw_setpoint = sign * min(self.max_thrust, abs(yaw_setpoint))
                
                if self.wp_follower_type == 'simple_artificial':
                    throttle_level = self.max_throttle
                else:
                    throttle_level = (self.P_throttle * throttle_error + 
                                    self.I_throttle * self.int_throttle_error +
                                    self.D_throttle * der_throttle_error)
                    throttle_level = min(self.max_throttle, throttle_level)
        
                # throttle_level = self.max_throttle

                if self.time_sync:
                    if self.t_arrival: #if you have an arrival time, boost the throttle to guarantee you arrive on time
                        boost = 1.0
                        if self.t_start is None:
                            self.t_start = time.time()
                        t = time.time()-self.t_start
                        delta_t = self.t_arrival-t
                        rospy.loginfo("time left %f s", delta_t)
                        distance = np.linalg.norm(np.array([goal_point_local.point.x, goal_point_local.point.y]))
                        throttle_level = distance/delta_t
                        rospy.loginfo("throttle level %f m/s", throttle_level)

                #TODO:
                #1. OK - Create common time tags for all agents in pattern generator, they all should have common time tags 
                #2. OK - Use these common time tags to boost the throttle of agents that are behind in the pattern
                    ## - Instead of proportional appriach, calc distance and needed velocity to catch up
                #3. OK - Double check why Dubins planner sometimes generates the longer paths
                #4. OK - Add time sync as arg
                #5. OK - Edit aux launch file to generate cool lookingmaps in rviz
                #6. Look into time sync some more, the arrays are weird... Se continue comment
                #7. Ensure backwards compatibility with old launch files
                #8. Start looking into PF
                #9. OK - Add dubins back
                

                    


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
        self.motion_command(0.,0.,0.)

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
        # if np.isclose((rospy.Time.now() - self.timer_t_old).to_nsec(), 1e9/self.timer_rate, atol=1e7):
        #     self.timer_t_old = rospy.Time.now()
        # diff = time.time()-int(time.time())
        
        # while not np.isclose(diff % 1/self.timer_rate, 0,atol=1e-1):
        #     print(diff % 1/self.timer_rate)
        #     diff = time.time()-int(time.time())
        #     pass
        # print("passed")

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

    def arrival_time_cb(self, msg):
        self.t_arrival = msg.data.secs
    
    def _reset_params(self):
        self.t = None
        self.int_throttle_error = 0
        self.int_thrust_error = 0
        self.prev_throttle_error = 0
        self.prev_thrust_error = 0
        self.t_arrival = None
        self.do_max_turn = None
        self.k = None
    
    def _point_on_circle(self, x, y,radius,h,k):
        """Returns True if the point (x,y) lies on the circle centered at (h,k) with radius r"""
        return np.isclose(np.sqrt((x-h)**2 + (y-k)**2),radius,atol=self.goal_tolerance)

    def __init__(self, name):

        self.timer_rate = 20 #Hz
        t = time.time()
        while not np.isclose((t-int(t)) % (1/self.timer_rate), 0,atol=1e-4):
            t = time.time()
        #     print("diff",t-int(t))
        #     print((t-int(t)) % (1/self.timer_rate))
        # print("passed")

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
        
        self.P_throttle = rospy.get_param('~P_throttle', 1.0)
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

        #self.delta_t_array = []
        self.t_start = None
        self.t_arrival = None
        # self.t_arrival_old = 0
        self.time_sync = rospy.get_param('~time_sync', 'false')

        self.dubins_turning_radius = rospy.get_param('~dubins_turning_radius')
        self.do_max_turn = None
        self.k = None
        self.goal_tolerance_max_turn = self.goal_tolerance+0.5
        self.goal_tolerance_original = self.goal_tolerance
        self.wp_follower_type = rospy.get_param('~waypoint_follower_type', 'simple')

        self.listener = tf.TransformListener()
        # self.timer_rate = 20 #Hz
        self.timer_t_old = rospy.Time.now()
        rospy.Timer(rospy.Duration(1/(self.timer_rate)), self.timer_callback)

        self.throttle_pub = rospy.Publisher(self.throttle_top, Float64, queue_size=1)
        self.thruster_pub = rospy.Publisher(self.thruster_top, Float64, queue_size=1)
        self.inclination_pub = rospy.Publisher(self.inclination_top, Float64, queue_size=1)
        # self.delta_t_sub = rospy.Subscriber('delta_t', Time, self.delta_t_cb)
        self.arrival_time_sub = rospy.Subscriber('arrival_time', Time, self.arrival_time_cb)

        self._as = actionlib.SimpleActionServer(
            self.as_name, MoveBaseAction, execute_cb=self.execute_cb, auto_start=False)
        self._as.start()
        rospy.loginfo("Announced action server with name: %s", self.as_name)

        rospy.spin()
        # rate = 20 #Hz
        # t_old = rospy.Time.now()
        # while not rospy.is_shutdown():
        #     t = rospy.Time.now()
        #     if np.isclose((t-t_old).to_nsec(), 1e9/rate, atol=1e7):
        #         print("here")
        #         self.timer_callback(None)
        #         t_old = t

if __name__ == '__main__':

    rospy.init_node('w2w_path_planner')
    planner = W2WPathPlanner(rospy.get_name())
