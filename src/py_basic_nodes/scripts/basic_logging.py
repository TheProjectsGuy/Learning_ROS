#!/usr/bin/env python3

# Main ros python module
import rospy

def shutdown_function():
    print("Showdown signal received, node is shutting down")

if __name__ == "__main__":
    try:
        # Initialize the node
        rospy.init_node("py_logger", anonymous=False)
        # Bind a function to be executed before shutdown
        rospy.on_shutdown(shutdown_function)
        # Handle delay based on frequency (2 Hz here)
        """
            This is used to cause a delay and maintain some activity at a desired frequency
            More here: http://wiki.ros.org/rospy/Overview/Time#Sleeping_and_Rates
        """
        rate_handler = rospy.Rate(2)
        # While ROS is not shutdown
        """
            rospy.is_shutdown() returns 1 if ROS has shutdown
            More here: http://wiki.ros.org/rospy/Overview/Initialization%20and%20Shutdown
        """
        while not rospy.is_shutdown():
            rospy.logdebug("This is a debug message")
            rospy.loginfo("This is an info message")
            rospy.logwarn("This is a warning message")
            rospy.logerr("This is an error message")
            rospy.logfatal("This is a fatal message")
            rate_handler.sleep()
    except rospy.ROSInterruptException:
        print("ROS Ended")

"""
    Add the `scripts/basic_logging.py` to the `catkin_install_python` function in CMakeLists.txt file

    Run the node using 
    ```
    rosrun py_basic_nodes basic_logging.py
    ```

    By default, you may not see the debug messages, run
    ```
    rosrun rqt_logger_level rqt_logger_level
    ```
    And change the `rosout` logger of the node to "Debug" level. You must now be able to see debug messages.

    It is good to kill a node not by using Ctrl + C but by using `rosnode`
    ```
    rosnode kill /py_logger
    ```
"""
