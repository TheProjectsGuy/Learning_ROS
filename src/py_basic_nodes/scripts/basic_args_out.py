#!/usr/bin/env python3

"""
    Prints the arguments passed
"""

# Import the rospy module for ROS Python client
import rospy
# System module
import sys

if __name__ == "__main__":
    try:
        # Initialize the node
        """
            rospy.init is used to initialize the node and connect it to the ROS master
            Call it before using any part of ROS.
            More here: http://docs.ros.org/en/melodic/api/rospy/html/rospy-module.html#init_node
        """
        rospy.init_node('py_arg_out', anonymous=False)
        print("Initialized the node")
        # Print the arguments
        for i in range(len(sys.argv)):
            print("Argument %d: %s" % ((i+1), sys.argv[i]))


    except rospy.ROSInterruptException:
        print("ROS ended")


"""
    After this is over, open CMakeLists.txt and under the `Install` section, add
    ```
    catkin_install_python(PROGRAMS
        scripts/basic_args_out.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )
    ```

    Then, call catkin_make in the workspace folder

    Run this node using `rosnode` command
    ```
    rosrun py_basic_nodes basic_args_out.py arg1 arg2 arg3
    ```
"""
