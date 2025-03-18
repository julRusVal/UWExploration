#!/usr/bin/env python3

import rospy
import roslaunch
import rospkg

def launch_nodes():
    rospy.init_node('multi_agent_launcher', anonymous=True)

    # Path to your launch file
    rospack = rospkg.RosPack()
    launch_file = f"{rospack.get_path('multi_agent_comms')}/launch/ma_comms.launch"

    # Define different arguments for each agent
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    launch_1 = roslaunch.parent.ROSLaunchParent(
        uuid, [(launch_file, [
            ('namespace:=hugin_0'),
            ('agent_id:=0'),
            ('agent_count:=2'),
            ('comm_delayed:=True'),
            ('comm_testing:=True'),
            ('comm_verbose:=True')
        ])]
    )

    launch_2 = roslaunch.parent.ROSLaunchParent(
        uuid, [(launch_file, [
            ('namespace:=hugin_1'),
            ('agent_id:=1'),
            ('agent_count:=2'),
            ('comm_delayed:=True'),
            ('comm_testing:=True'),
            ('comm_verbose:=True')
        ])]
    )

    try:
        # Start the launch processes
        launch_1.start()
        launch_2.start()

        rospy.loginfo("Launched two communication nodes.")
        rospy.spin()  # Keep running

    except KeyboardInterrupt:
        rospy.loginfo("Shutting down launched nodes.")
        launch_1.shutdown()
        launch_2.shutdown()

if __name__ == "__main__":
    launch_nodes()
