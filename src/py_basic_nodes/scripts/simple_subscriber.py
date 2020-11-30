#!/usr/bin/env python3

"""
    Main objective of this program is to do the following:
        1. Demonstrate how to create a subscriber
"""

# Import ROS Client library
import rospy
# Import system library
import sys
# Import library for the message file
from std_msgs.msg import String

# Create a callback function
"""
    This function will be called with an object to access the message contents as the argument
    every time a message is received on the subscribed topic
"""
def subscriber_callback(msg_obj: String):
    rospy.loginfo("Received a message: '{0}'".format(msg_obj.data))

def main():
    # Initialize the node
    rospy.init_node("simple_py_subscriber", argv=sys.argv)
    
    # Create a subscriber object
    """
        We will use this object to subscribe to topics
        The first argument is topic name
        The second argument is the class (message type)
        The third argument is the 

        Link (Subscriber class): http://docs.ros.org/en/melodic/api/rospy/html/rospy.topics.Subscriber-class.html
        Link (constructor): http://docs.ros.org/en/melodic/api/rospy/html/rospy.topics.Subscriber-class.html#__init__
    """
    sub_obj = rospy.Subscriber("{0}/subs_str".format(rospy.get_name()), String, subscriber_callback, queue_size=1000)
    # Set the node to spin
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException as e:
        rospy.logfatal("A fatal error has ocurred: {0}".format(e))
