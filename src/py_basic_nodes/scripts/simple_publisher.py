#!/usr/bin/env python3

"""
    Main objective of this program is to do the following:
        1. Demonstrate how to create a publisher
        2. Introduces executing things at a particular frequency
"""

import rospy
import sys
# Import the library generated for the message file
"""
    Every message actually has a data structure to it (implemented as a class in Python)
    This class is stored in a file which you must import in order to use the class

    Location on local PC: /opt/ros/noetic/lib/python3/dist-packages, inside std_msgs/msg/_String.py
"""
from std_msgs.msg import String

def main():
    # Initialize node
    rospy.init_node("simple_py_publisher", argv=sys.argv, anonymous=False)

    # Create a publisher object
    """
        We will use this object to publish messages
        First argument is the topic name, second argument is the class (message type) and third is the queue size
        The first argument uses the node name for the topic name (to get it under a namespace)

        Link (for names in ROS): http://wiki.ros.org/Names
        Link (for Publisher class): http://docs.ros.org/en/melodic/api/rospy/html/rospy.topics.Publisher-class.html
        Link (for get_name()): http://docs.ros.org/en/melodic/api/rospy/html/rospy-module.html#get_name
    """
    pub_obj = rospy.Publisher("{0}/hello_str".format(rospy.get_name()), String, queue_size=1000)

    # Create a rate controller
    """
        This is used to cause a delay to suit a particular frequency or duration
        Here, we will publish once every two seconds (i.e. frequency is 0.5 Hz)

        Link: http://docs.ros.org/en/melodic/api/rospy/html/rospy.timer.Rate-class.html
    """
    rt_ctrl = rospy.Rate(0.5)

    # Some variables to keep track of things
    seq = 0     # Sequence number of the message number

    # Loop execution until node hasn't shutdown
    while not rospy.is_shutdown():
        # Create a message object
        """
            This message will be finally published in the end
        """
        msg_obj = String()
        msg_obj.data = "Hello, World! [I: {}]".format(seq)  # Fill data
        seq += 1    # Increment sequence number

        # Publish the message
        """
            Use the publish member function to publish a message

            Link: http://docs.ros.org/en/melodic/api/rospy/html/rospy.topics.Publisher-class.html#publish
        """
        pub_obj.publish(msg_obj)

        # Log the message published
        rospy.loginfo("Published '{}'".format(msg_obj.data))

        # Cause a delay to obtain the frequency
        """
            Create a delay using the sleep function. This will hold the execution according to the rate.

            Link: http://docs.ros.org/en/melodic/api/rospy/html/rospy.timer.Rate-class.html#sleep
        """
        rt_ctrl.sleep()
        pass
    pass

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException as e:
        rospy.logfatal("A ROS Exception has occurred: {0}".format(e))
