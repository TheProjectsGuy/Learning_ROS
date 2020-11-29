#!/usr/bin/env python3
"""
    The first line is called a shebang line. It explains the executable to use when running this script.
    ROS Noetic uses python3, hence the python3 interpreter path is used.
"""

"""
    Main motive of this program is to do the following:
        1. Print "Hello, World!"
        2. Demonstrates different types of logging messages
        3. Print the arguments passed to the executable using an INFO message
        4. Demonstrates how to change the logger level using code
"""

# Import the main rospy client library
import rospy
# For dealing with arguments
import sys

# A proxy to the main function
def main():
    # Initialize the node
    """
        This function is used to initialize the link between this node and the ROS Master.
        The arguments passed to this node are passed to the master (for things like parameter remapping, etc.).
        By default, the name of this node is used as a base name and is suffixed with a random number (so that you can run multiple nodes)
            To undo this, set anonymous to False
        In order to print debug messages, set the log_level to DEBUG (more about logging later in this script)
        
        Link: http://docs.ros.org/en/melodic/api/rospy/html/rospy-module.html#init_node
    """
    rospy.init_node("hello_world_simple_py", argv=sys.argv, anonymous=False, log_level=rospy.DEBUG)

    # Print out "Hello, World!"
    print("Hello, World!")

    # Print different levels of logging messages
    """
        Logging is the professional method of displaying messages to the console (we rarely use print)
        There are several levels
        - DEBUG: Debugging level messages are used for primitive debugging purposes. These are hidden by default.
        - INFO: Information level messages, higher in priority than DEBUG
        - WARN: Warning level messages. Usually for handled exceptions. Higher priority than INFO
        - ERROR: Error level messages. Exception will cause some aspect of the program to fail or malfunction. But the program at large can run.
        - FATAL: Fatal level messages. Fatal error has occurred and the program cannot run.

        Macro definitions (core functions): http://docs.ros.org/en/melodic/api/rospy/html/rospy.core-module.html
        Link: http://wiki.ros.org/rospy/Overview/Logging
    """
    rospy.logdebug("This is a DEBUG message")
    rospy.loginfo("This is an INFO message")
    rospy.logwarn("This is a WARN message")
    rospy.logerr("This is an ERROR message")
    rospy.logfatal("This is a FATAL message")

    # Print out the arguments
    for i in range(len(sys.argv)):
        rospy.loginfo("Argument {n}: {m}".format(n=i+1, m=sys.argv[i]))

    # Hold the execution here till the shutdown flag is received
    """
        is_shutdown is used to cause a delay until the node is shut down
        The code below is equivalent to rospy.spin()

        Link: http://docs.ros.org/en/melodic/api/rospy/html/rospy-module.html#is_shutdown
    """
    while not rospy.is_shutdown():
        pass

# Main code
if __name__ == "__main__":
    try:
        # Call the main function
        main()
    except rospy.ROSException as e: # Handle a ROS Exception
        print("A ROS exception has occurred: {}".format(e))
        pass
    pass
