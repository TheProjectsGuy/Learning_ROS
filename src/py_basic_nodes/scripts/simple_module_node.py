#!/usr/bin/env python3

"""
    This node is to demonstrate how you can use a Python module created in a ROS package
"""

# Import basic modules
import rospy
import sys

# Import the module
"""
    This is the module that we have created. The module must exist in `devel/lib/python3/dist-packages`
    folder of your workspace

    Executing directory: src/basic_py_libs/src/simple_module
    Import directory: devel/lib/python3/dist-packages/simple_module
"""
import simple_module


# Main function
def main():
    # Initialize node
    rospy.init_node("simple_module_node", argv=sys.argv)
    # Call functions from the library
    simple_module.say_hello("User")


# Entry point
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException as exc:
        rospy.logfatal("A fatal ROS exception has occurred (Msg: {msg})".format(msg=exc))
    except Exception as exc:
        rospy.logfatal("A non-ROS exception occurred (Msg: {msg})".format(msg=exc))
