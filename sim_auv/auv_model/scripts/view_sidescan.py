#!/usr/bin/env python3
import rospy
import cv2
import argparse
# from smarc_msgs.msg import Sidescan  # TODO Check which one is the correct one
from auv_model.msg import Sidescan
from functools import partial
import numpy as np
from ctypes import cast, pointer, POINTER, c_char, c_int



def convert(c):
    return cast(pointer(c_char(c)), POINTER(c_int)).contents.value


def callback(img, msg):

    #print msg
    debug_callback = False

    #for p in msg.sidescan.sidescan.port_channel:
    #    print convert(p)

    port = np.array(bytearray(msg.port_channel), dtype=np.ubyte)
    stbd = np.array(bytearray(msg.starboard_channel), dtype=np.ubyte)

    meas = np.concatenate([np.flip(port), stbd])
    #print(meas)
    img[1:, :] = img[:-1, :]
    img[0, :] = meas

    if debug_callback:
        # Old Debugging
        # print msg

        # for p in msg.sidescan.sidescan.port_channel:
        #    print convert(p)

        # port = np.array([int(p) for p in msg.sidescan.sidescan.port_channel]) # dtype=np.ubyte)
        # stbd = np.array([int(p) for p in msg.sidescan.sidescan.starboard_channel])
        # stbd = np.array(msg.sidescan.sidescan.starboard_channel, dtype=float) #dtype=np.ubyte)
        # print port.shape, stbd.shape
        # print port, stbd

        # New Debugging
        print("view_sidescan.py callback")
        print("max port: ", np.max(port))
        print("max stbd: ", np.max(stbd))
        print("port.shape: ", port.shape)
        print("stbd.shape: ", stbd.shape)

# Initialize the node
rospy.init_node('sidescan_viewer', anonymous=True)

# Parse the arguments
# parser = argparse.ArgumentParser()
# parser.add_argument("namespace", default="hugin_0", help="namespace of the auv")
# parser.add_argument("topic", default="sim/sss_pings", help="Select which SSS topic to subscribe to")
# args = parser.parse_args()

# sss_topic = "/hugin_0/pf/sss/particle_0"
#sss_topic = f"/{args.namespace}/{args.topic}"
sss_topic = f"/hugin_0/sim/sss_pings"

img = np.zeros((1000, 1000), dtype=np.ubyte)  # dtype=float) #
cv2.namedWindow(f'Sidescan image - {sss_topic}', cv2.WINDOW_NORMAL)
cv2.resizeWindow(f'Sidescan image - {sss_topic}', 2 * 256, 1000)

rospy.Subscriber(sss_topic, Sidescan, partial(callback, img))

print(f"Subscribed to {sss_topic}")

# spin() simply keeps python from exiting until this node is stopped
r = rospy.Rate(5)  # 10hz
while not rospy.is_shutdown():
    resized = cv2.resize(img, (2 * 256, 1000), interpolation=cv2.INTER_AREA)
    cv2.imshow(f"Sidescan image - {sss_topic}", resized)
    cv2.waitKey(1)
    r.sleep()
