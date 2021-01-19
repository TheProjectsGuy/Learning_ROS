#!/usr/bin/env python3

"""
    This is a simple publisher that is a part of `launch1`

    This node publishes a string message on a topic.
    It takes the following parameters
    - ~/l1pub_PubFreq [double]: The publishing frequency (in Hz) for the node

    The node publishes to the following topics
    - pub_topic [std_msgs/String]: Publishing topic
"""

# Import basic library
import rospy
import sys
# Import the modules for messages
from std_msgs.msg import String


# Main function
def main():
    # Initialize node
    rospy.init_node("launch1_publisher", argv=sys.argv)

    # Parameter for publishing frequency
    param_name = "{node_name}/l1pub_PubFreq".format(node_name=rospy.get_name()) # Need parameter in local scope
    pub_freq = 0.1  # Default publishing frequency
    try:
        pub_freq = float(rospy.get_param(param_name))
        rospy.loginfo("Publishing frequency set to {freq} Hz from parameter '{pname}'".format(
            freq=pub_freq, pname=param_name
        ))
    except ValueError as e1:
        rospy.logwarn("Parameter '{pname}' could not be parsed as float, value remains {val}: {err}".format(
            pname=param_name, val=pub_freq, err=e1
        ))
    except KeyError as e2:
        rospy.loginfo("Could not find the parameter '{pname}', using default value of {val}: {err}".format(
            pname=param_name, val=pub_freq, err=e2
        ))

    # Rate of publisher
    pub_hdlr = rospy.Rate(pub_freq)
    # Publisher
    pub_obj = rospy.Publisher("pub_topic", String, queue_size=100)
    seq_num = 1
    # Main publishing loop
    while not rospy.is_shutdown():
        # Create a message object
        msg_obj = String()
        msg_obj.data = str(seq_num)
        pub_obj.publish(msg_obj)
        seq_num += 1
        # Log it
        rospy.logdebug("Published message [topic: {tname}] '{msgc}'".format(
            tname=pub_obj.name, msgc=msg_obj.data
        ))
        rospy.loginfo("Published '{msgc}'".format(
            msgc=msg_obj.data
        ))
        # Delay for adjusting to the pub_freq
        pub_hdlr.sleep()


# Entry point
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSException as e:
        rospy.logfatal("ROS Exception: {0}".format(e))
