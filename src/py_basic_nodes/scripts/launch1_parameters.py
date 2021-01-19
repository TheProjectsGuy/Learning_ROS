#!/usr/bin/env python3

"""
    This is a node that prints out all the parameters available (just names / keys)
"""

# Import basic modules
import rospy
import sys


# Main function
def main():
    # Initialize node
    rospy.init_node("launch1_params", argv=sys.argv)
    # Get a list of names on the parameter server
    names = rospy.get_param_names()
    rospy.loginfo("Found {pnum} parameters".format(pnum=len(names)))
    for i in range(len(names)):
        rospy.loginfo("Parameter {n}: {pval}".format(
            n=i+1, pval=names[i]
        ))


# Entry point
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSException as ex:
        rospy.logfatal("ROS Exception: {err}".format(err=ex))

