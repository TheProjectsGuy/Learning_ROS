#!/usr/bin/env python3

"""
    This is a simple subscriber that is a part of `launch1`

    This node subscribes to a topic and simply prints out the message contents
    The node subscribes to the following topics
    - subs_topic [std_msgs/String]: Subscribing topic
"""

# Include basic modules
import rospy
import sys
# Modules for messages
from std_msgs.msg import String


# Subscriber callback
def subs_callback(msg: String, subs_obj: rospy.Subscriber):
    # Log
    rospy.logdebug("Received a message")
    rospy.loginfo("Received message [topic: {tname} ({nc})] '{msgdata}'".format(
        tname=subs_obj.name, nc=subs_obj.get_num_connections(),
        msgdata=msg.data
    ))
    pass


# Main function
def main():
    # Initialize node
    rospy.init_node("launch1_subscriber", argv=sys.argv)
    # Create a subscriber
    subs_obj = rospy.Subscriber("subs_topic", String, callback=lambda msg: subs_callback(msg, subs_obj))
    # Set the node to spin indefinitely
    rospy.spin()


# Entry point
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSException as e:
        rospy.logfatal("ROS Exception: {err}".format(err=e))
