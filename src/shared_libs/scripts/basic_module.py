#!/usr/bin/env python3
"""
    This is a node to demonstrate accessing a local python package
"""
# The local package
"""
    Path (in this package): py_modules folder of tha package
    Path (wrapper): devel/lib/python3/dist-packages folder in workspace
"""
import shared_pylib
# Import a file in the sub-module
"""
    Import the simple_math.py file
"""
from shared_pylib.simple_math import simple_math
# Import a class from that file simple_math.py
from shared_pylib.simple_math.simple_math import RandomClass as RC
# Import a sub-module within that module
"""
    This is not the file simple_math.py, but the __init__.py file
"""
import shared_pylib.simple_math as smath
# Basic modules
import rospy
import sys

# Main function
def main():
    # Initialize the node
    rospy.init_node("py_locallib_node", argv=sys.argv)
    # Access the library, deliver output from a function call
    rospy.loginfo("Call to shared_pylib function returned: {rval}".format(
        rval=shared_pylib.func_global()
    ))
    # Access another function
    c = simple_math.add(3, 4)
    rospy.loginfo("Calling add returns: {v}".format(v=c))
    # Access a local function in a sub-module
    rospy.loginfo("Call to simple_math local function returned: {rval}".format(
        rval=smath.func_local()
    ))
    # Access a member function of a class
    rc_obj = RC()   # Same as: rc_obj = simple_math.RandomClass()
    rospy.loginfo("Call to random function of a class object: {rval}".format(
        rval=rc_obj.random_function()
    ))


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException as exc:
        rospy.logfatal("A fatal ROS Exception occurred: {msg}".format(msg=exc))
    except Exception as exc:
        rospy.logfatal("A fatal exception occurred: {msg}".format(msg=exc))
