#!/usr/bin/env python3

# General imports
import math
import time
import os
from typing import Tuple, Optional
import threading
import sys
# import cbor2

# ROS imports
import rospy
import tf

# ROS messages
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from multi_agent_comms.msg import BasicComms


# Constants
DEFAULT_COMM_RANGE = 100.0  # Default communication range in meters
DEFAULT_COMM_SOUND_SPEED = 1500.0  # Speed of sound in water (m/s)
DEFAULLT_COMM_RATE = 1.0  # Message send rate (Hz)
DEFAULT_COMM_DELAYED = True

class AcousticCommsNode:
    def __init__(self):

        rospy.init_node(f'acoustic_comms', anonymous=True)
        
        self.agent_id = rospy.get_param('~agent_id', 0)
        self.agent_count = rospy.get_param('~agent_count', 1)
        self.node_name = rospy.get_name()

        rospy.loginfo(f'[{self.node_name}] Starting accoustic comms node')

        # Get communication parameters from parameter server (default: 100m)
        self.comm_range = rospy.get_param('~comm_range', DEFAULT_COMM_RANGE)
        self.comm_sound_speed = rospy.get_param('~comm_sound_speed', DEFAULT_COMM_SOUND_SPEED)
        self.comm_rate = rospy.get_param('~comm_rate', DEFAULLT_COMM_RATE)
        self.comm_delayed = rospy.get_param('~comm_delayed', DEFAULT_COMM_DELAYED)

        # Testing mode for communication
        # This will bypass the range and delay calculations
        self.comm_testing = rospy.get_param('~comm_testing', False)
        self.comm_verbose = rospy.get_param('~comm_verbose', False)
        self.verbose_period = 5

        if self.comm_testing:
            rospy.loginfo(f'[{self.node_name}] Testing Mode')

        # Log file setup
        # log_dir = os.path.expanduser("~/ros_logs")
        # os.makedirs(log_dir, exist_ok=True)
        # self.log_file = os.path.join(log_dir, f"acoustic_comms_{self.agent_id}.log")

        # TF listener
        self.tf_listener = tf.TransformListener()

        # Publishers
        # Accoustic comms messages to each of the other agents
        self.comms_publishers = {}
        for i in range(self.agent_count):
            if i != self.agent_id:
                self.comms_publishers[i] = rospy.Publisher(f'/hugin_{i}/comms', BasicComms, queue_size=10)
        
        # Trigger saving in svgp
        save_svgp_topic = f'/hugin_{self.agent_id}/save_svgp'
        self.save_svgp_pub = rospy.Publisher(save_svgp_topic, String, queue_size=10)

        # Visualizations
        self.marker_pub = rospy.Publisher(f'/hugin_{self.agent_id}/comms_visualization', Marker, queue_size=10)

        # === Subscribers ===
        # Topics
        # Not used after moving to the tf for positions
        # position_topic = '/sim/odometry'  # This will need to be namespaced properly

        # = Local agent = 
        # Position
        self.own_position = None  # Update when position message is recieved
        #rospy.Subscriber(f'/hugin_{self.agent_id}{position_topic}', Odometry, self.update_position)

        # Incoming accoustic messages
        rospy.Subscriber(f'/hugin_{self.agent_id}/comms', BasicComms, self.message_callback)

        # = Remote agent(s) = 
        self.other_positions = {}
        self.other_distances = {}

        # Subscribe to other agents to determin position
        for i in range(self.agent_count):
            if i != self.agent_id:
                self.other_positions[i] = None
                self.other_distances[i] = None
                # rospy.Subscriber(f'/hugin_{i}{position_topic}', Odometry, self.position_callback, callback_args=i)

        # Start periodic message sending
        self.send_timer = rospy.Timer(rospy.Duration(1.0 / self.comm_rate), self.send_message)

        # Dynamic parameter update
        rospy.Timer(rospy.Duration(1), self.update_comm_range)

    def compute_distance(self, p1, p2):
        """Calculate Euclidean distance between two points."""
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2 + (p1.z - p2.z) ** 2)

    def encode_cbor(self, msg_data):
        """Encode message using CBOR."""
        # return cbor2.dumps(msg_data)
        # TODO fix up enncoding
        return msg_data

    def decode_cbor(self, encoded_msg):
        """Decode CBOR message."""
        # return cbor2.loads(encoded_msg)
        # TODO fix up decoding
        return encoded_msg

    def log_event(self, event_type, message):
        """Log message transmission and reception events."""
        timestamp = rospy.get_time()
        log_entry = f"{timestamp:.2f} | {event_type} | {message}\n"
        with open(self.log_file, "a") as f:
            f.write(log_entry)

    # def delayed_process(self, encoded_msg, delay, sender_id):
    #     """Wait for the delay time and then process the received message."""
    #     time.sleep(delay)
    #     msg_data = self.decode_cbor(encoded_msg)
    #     rospy.loginfo(f"[{self.node_name}] Received after {delay:.2f} sec: {msg_data}")
    #     self.log_event("RECEIVED", str(msg_data))

    # def delayed_publish(self, message, delay, target_id):
    #     time.sleep(delay)
    #     rospy.loginfo(f"[{self.node_name}] PUBLISH - Delay: {delay:.2f} sec Target: {target_id}")
    #     self.comms_publishers[target_id].publish(message)

    def delayed_publish_callback(self, event, msg, target, delay):
        """Function to handle delayed publishing."""
        if self.comm_verbose:
            rospy.loginfo_throttle(5, f"[{self.node_name}] PUBLISH - Delay: {delay:.2f} sec Target: {target}")
        self.comms_publishers[target].publish(msg)

    def message_callback(self, msg: BasicComms):
        """Handle incoming messages from other agents."""

        # TODO There will need to be some interfacing with svgp mapper here
        # Parse the message and send it to the mapper
        sender_id = msg.sender_id

        if self.comm_verbose:
            rospy.loginfo_throttle(5, f"[{self.node_name}] Received message: {sender_id}")   
        
    # def position_callback(self, pos_msg, sender_id):
    #     """Update stored positions of other agents."""
    #     self.other_positions[sender_id] = pos_msg

    # def update_position(self, pos_msg):
    #     """Update this agent's position dynamically."""
    #     # Currently subscribing to a an Odometry
    #     self.own_position = pos_msg
    #     rospy.logdebug_once(f"[{self.node_name}] Updated position \n{self.own_position}")

    def update_positions_tf(self):
        """Update the positions of all agents using TF."""

        # For testing we need to fake the positions
        if self.comm_testing:
            rospy.loginfo_throttle(5, f'[{self.node_name}] Update_position_tf() - Testing Mode')
            for i in range(self.agent_count):
                agent_xyz = self.testing_generate_points(i)
                if i == self.agent_id:
                    self.own_position = Point(*agent_xyz)
                    rospy.loginfo_throttle(5, f'[{self.node_name}] Current position: {self.own_position}')
                else:
                    self.other_positions[i] = Point(*agent_xyz)
                    
            return
        
        # Get the positions of all agents using TF
        # This will be the actual positions of the agents
        else:
            rospy.loginfo_throttle(5, f'[{self.node_name}] Update_position_tf()')
            for i in range(self.agent_count):
                try:
                    # Wait up to 1 second for the transform to become available
                    current_frame = f"hugin_{i}/base_link"
                    base_frame = 'map'

                    self.tf_listener.waitForTransform(base_frame, current_frame, rospy.Time(0), rospy.Duration(1.0))

                    # Get the transform if available
                    (pos, rot) = self.tf_listener.lookupTransform(base_frame, current_frame, rospy.Time(0))
                    
                    if i == self.agent_id:
                        self.own_position = Point(*pos)
                        rospy.loginfo_throttle(5, f'[{self.node_name}] Current position: {self.own_position}')
                    else:
                        self.other_positions[i] = Point(*pos)
                    
                # except tf.Exception as e:  # Catch all TF errors
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logwarn(f"TF Exception: {e}")
                    # rospy.logerr(f"Transform lookup failed: {e}")
                    rospy.loginfo_throttle(5, f'[{self.node_name}] Update_position_tf() - tf exception')
                    if i == self.agent_id:
                        self.own_position = None
                    else:
                        self.other_positions[i] = None                      

    def update_distances(self):
        """
        This method will update the computed distance between the local agent and remote agents.
        If self.comm_testing these distances will be hard coded to self.comm_range/2
        """
        # Clear the previoulsy computed distances
        self.other_distances = {}
        for i in range(self.agent_count):
            if i != self.agent_id:
                self.other_distances[i] = None

        # Moved to  update_position_tf()
        # Compute false distances for testing purposes
        # if self.comm_testing:
        #     for i in range(self.agent_count):
        #         if i != self.agent_id:
        #             self.other_distances[i] = self.comm_range/2
            
        #     # Verbose of testing
        #     if self.comm_verbose:
        #         rospy.loginfo_throttle(5, f"[{self.node_name}] update_distances() - testing - {self.other_distances}")

        # Compute distances using the simulated odometry 
        if self.own_position is None:
            return
        for i in range(self.agent_count):
            if i != self.agent_id:
                if self.other_positions[i] is None:
                    self.other_distances[i] = None
                else:
                    # Remember that self.own_position and the values of self.other_positions are of type Odometry
                    # Positions are now stored as Points
                    own_pos = self.own_position
                    other_pos = self.other_positions[i]
                    distance = self.compute_distance(own_pos, other_pos)
                    self.other_distances[i] = distance

        # Verbose in actual usage
        if self.comm_verbose:
            rospy.loginfo_throttle(5, f"[{self.node_name}] update_distances() - usage - {self.other_distances}")

    def form_message(self):
        """Form a message to be sent to other agents."""
        
        msg_data = BasicComms()
        msg_data.sent = rospy.Time.now()
        msg_data.sender_id = self.agent_id
        msg_data.parameter_1 = 1
        msg_data.parameter_2 = 2
        msg_data.parameter_3 = 3
        msg_data.parameter_4 = 4
        msg_data.parameter_5 = 5
        
        return msg_data

    def send_message(self, event):
        """Broadcast this agent's message  position."""
        if self.comm_verbose:
            rospy.loginfo_throttle(5, f'[{self.node_name}] send_message()')

        self.save_svgp_pub.publish("save_svgp")

        self.update_positions_tf()

        self.update_distances()

        # TODO we need to get the actual mesage to this node still...
        msg_string = f"Message from {self.agent_id}"
        comms_msg = self.form_message()

        for target_i in self.comms_publishers.keys():
            distance_i = self.other_distances[target_i]
            if distance_i is not None and distance_i < self.comm_range:
                # Delay
                delay = distance_i / self.comm_sound_speed
                rospy.loginfo_throttle(5, f"[{self.node_name}] PUBLISH - Target: {target_i} Message: {msg_string}")
                if self.comm_delayed:
                    # update the time per message
                    comms_msg.sent = rospy.Time.now()
                    rospy.Timer(rospy.Duration(delay),
                                lambda event: self.delayed_publish_callback(event, 
                                                                            msg=comms_msg, 
                                                                            target=target_i, delay=delay),
                                oneshot=True)
                # threading.Thread(target=self.delayed_publish, args=(msg_string, delay, target_i))
                else:
                    self.comms_publishers[target_i].publish(msg_string)

        # rospy.loginfo(f"[{self.node_name}] Sent message")
        # self.log_event("SENT", str(msg_data))

        # Visualize message transmission
        self.publish_marker()

    def get_color_by_id(self, agent_id):
        """Assign a unique color based on agent_id."""
        colors = [
            (1.0, 0.0, 0.0),  # Red
            (0.0, 1.0, 0.0),  # Green
            (0.0, 0.0, 1.0),  # Blue
            (1.0, 1.0, 0.0),  # Yellow
            (1.0, 0.0, 1.0),  # Magenta
            (0.0, 1.0, 1.0),  # Cyan
        ]
        return colors[agent_id % len(colors)]

    def publish_marker(self):
        """Publish a visualization marker in RViz to show message transmission."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "acoustic_comms"
        marker.id = self.agent_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.2  # Line width

        # Assign color based on agent_id
        r, g, b = self.get_color_by_id(self.agent_id)
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0  # Full opacity

        # Ensure quaternion is defined
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.lifetime = rospy.Duration(2.0)  # Marker disappears after 2 seconds

        # Generate a list of start and end points for every valid transmission
        # If self.comm_testing, points need to be c
        for agent_id, distance in self.other_distances.items():
            if distance is None:
                continue
            if distance <= self.comm_range:
                # Get the actual positions of the agents stored as Points
                start_point = self.own_position
                end_point = self.other_positions[agent_id]
                # Append the points to the marker
                marker.points.append(start_point)
                marker.points.append(end_point)

        if marker.points:
            self.marker_pub.publish(marker)

    def testing_generate_points(self, agent_id: int, radius: Optional[float] = None) -> Tuple[float, float, float]:
        """
        Computes the coordinates of the m-th point on a circle with n evenly spaced points.
        If r is None, it defaults to 1.0.

        Args:
            agent_id (int): Index of the point (0-based).
            r (Optional[float]): Radius of the circle (default is None, set to 1.0 inside).

        Returns:
            Tuple[float, float]: (x, y, 0) coordinates of the m-th point.
        """

        if agent_id < 0 or agent_id >= self.agent_count:
            raise ValueError("Index m must be between 0 and n-1.")

        if radius is None:
            radius = self.comm_range/5  # Default radius

        angle = (2 * math.pi * agent_id) / self.agent_count  # Compute angle for m-th point
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)

        return x, y, 0

    def update_comm_range(self, event):
        """Check and update communication range dynamically from parameter server."""
        new_range = rospy.get_param('~comm_range', self.comm_range)
        if new_range != self.comm_range:
            rospy.loginfo(f"[{self.node_name}] Updated comm range: {new_range} meters")
            self.comm_range = new_range

if __name__ == '__main__':
    try:
        # # Read agent ID and total agents from launch arguments
        # if len(sys.argv) < 3:
        #     print("Usage: rosrun your_package acoustic_comms.py <agent_id> <agent_count>")
        #     sys.exit(1)

        # agent_id = int(sys.argv[1])
        # agent_count = int(sys.argv[2])

        # node = AcousticCommsNode(agent_id, agent_count)
        node = AcousticCommsNode()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
