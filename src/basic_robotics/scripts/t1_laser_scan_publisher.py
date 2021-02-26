"""
    The purpose of this file is as follows:
        1. To be used as a dummy sensor for Tutorial 1
        2. Create a publisher that can publish data for `LaserScan` type
    
    This node takes the following paraemters
        - ~/pub_topic (string): Publishing topic name (default: /t1_laser_scan)
        - ~/pub_freq (double): Publishing frequency in Hz (default: 10 Hz)
        - ~/pub_frame (string): Frame for header (header -> frame_id) (default: global)
"""

# Import basic modules
from os import read
import rospy
import sys
# Import the module for messages
from sensor_msgs.msg import LaserScan
import math


# Main function
def main():
    # Initialize the node
    rospy.init_node("t1_py_laser_scan_publisher", argv=sys.argv)
    
    # Configurations
    # Configure the publishing topic
    pub_topic_name = "/t1_laser_scan"
    if rospy.has_param("{nm}/pub_topic".format(nm=rospy.get_name())):
        try:
            pub_topic_name = str(rospy.get_param("{nm}/pub_topic".format(nm=rospy.get_name())))
        except ValueError as err:
            rospy.logwarn("Publishing topic could not be parsed to string, using default (error: {})".format(err))
    rospy.loginfo("Publishing topic set to '{0}'".format(pub_topic_name))
    # Configure the publishing frequency
    pub_frequency = 10
    if rospy.has_param("{nm}/pub_freq".format(nm=rospy.get_name())):
        try:
            pub_frequency = float(rospy.get_param("{nm}/pub_freq".format(nm=rospy.get_name())))
        except ValueError as err:
            rospy.logwarn("Publishing frequency could not be parsed to float, using default (error: {})".format(err))
    rospy.loginfo("Publishing frequency set to '{0} Hz'".format(pub_frequency))
    # Configure the publishing frame
    pub_frame = "global"
    if rospy.has_param("{nm}/pub_frame".format(nm=rospy.get_name())):
        try:
            pub_frame = str(rospy.get_param("{nm}/pub_frame".format(nm=rospy.get_name())))
        except ValueError as err:
            rospy.logwarn("Publishing frame could not be parsed to string, using default (error: {})".format(err))
    rospy.loginfo("Publishing frame set to '{0}'".format(pub_frame))

    # Publisher object
    pub_obj = rospy.Publisher(pub_topic_name, LaserScan, queue_size=100)
    # Rate handler
    rate_hdlr = rospy.Rate(pub_frequency)

    # Message to publish
    pub_msg = LaserScan()
    # Configure defaults (that will remain same throughout, sensor properties: you may find these in datasheets)
    pub_msg.header.seq = 0  # Set sequence to 0
    pub_msg.header.frame_id = pub_frame # Publishing frame (for header)
    pub_msg.angle_min = math.radians(-135)  # Minimum angle of view
    pub_msg.angle_max = math.radians(135)   # Maximum angle of view
    pub_msg.angle_increment = math.radians(1)   # Increments of 1 degree
    pub_msg.scan_time = 1 / pub_frequency   # Time for the entire scan
    pub_msg.time_increment = pub_msg.scan_time / 270    # Scan / (270 measurements = ((max - min) / (increment)))
    pub_msg.range_max = 5   # 5 m maximum range
    pub_msg.range_min = 0.10    # 10 cm minimum range

    # Start the publishing loop
    while not rospy.is_shutdown():
        # Assign values to the publishing message
        pub_msg.header.stamp = rospy.Time.now()
        pub_msg.header.seq += 1
        readings = []
        for i in range(270):
            readings.append(2)  # For now, all readings are at 2 m
        pub_msg.ranges = readings

        # Publish the message
        pub_obj.publish(pub_msg)

        # Delay
        rate_hdlr.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException as exc:
        rospy.logfatal("Fatal exception occurred: {}".format(exc))
