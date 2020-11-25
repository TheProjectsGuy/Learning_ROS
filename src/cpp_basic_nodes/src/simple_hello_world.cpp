/*
 * This program does the following:
 *      1. Prints "Hello, World!"
 *      2. Demonstrates different types of logging messages
 *      3. Print the arguments passed to the executable using an INFO message
 *      4. Demonstrates how to change logger level using code
 */ 

// Include the header file to include everything ROS
/*
 * This file can be found in /opt/ros/noetic/include, inside ros/ros.h
 * The entire API documentation can be found at https://docs.ros.org/en/api/roscpp/html/
 */ 
#include <ros/ros.h>
#include <iostream>

using namespace std;

// Main functions
int main(int argc, char **argv) {
    // Initialize the node
    /*
     * This function is used to initialize the link between this node and the ROS master.
     *      The parameters passed to the node are also passed to the master. This is useful for purposes like remapping
     * The string passed in the end is the name using which this node registers. On running `rosnode list`, this node 
     * will appear with the name given (here, "hello_world_simple")
     * 
     * Link: https://docs.ros.org/en/api/roscpp/html/namespaceros.html#a7f5c939b8a0548ca9057392cc78d7ecb
     */
    ros::init(argc, argv, "hello_world_simple");
    
    // Create a NodeHandle
    /*
     * Since this is a client library to access the ROS master, this node can communicate with the master
     * This communication is done using a unique object, called the NodeHandle
     * 
     * Link: https://docs.ros.org/en/api/roscpp/html/classros_1_1NodeHandle.html
     */
    ros::NodeHandle nh;
    
    // Print a "Hello, World!" using simple cout
    cout << "Hello, World!" << endl;
    
    // Print different levels of logging messages
    /*
     * Logging is the professional method of displaying messages to the console (using cout isn't very good)
     * There are several levels
     * - DEBUG: Debugging level messages (hidden by default)
     * - INFO: Information level messages, higher in priority than DEBUG
     * - WARN: Warning level messages. Usually for handled exceptions. Higher priority than INFO
     * - ERROR: Error level messages. Exception will cause some aspect of the program to fail or malfunction. But the program at large can run.
     * - FATAL: Fatal level messages. Fatal error has occurred and the program cannot run.
     * 
     * Macro definitions can be found in: /opt/ros/noetic/include/rosconsole/macros_generated.h
     * Link: http://wiki.ros.org/roscpp/Overview/Logging
     */
    ROS_DEBUG("This is a DEBUG message");
    ROS_INFO("This is an INFO message");
    ROS_WARN("This is a WARN message");
    ROS_ERROR("This is an ERROR message");
    ROS_FATAL("This is a FATAL message");
    
    // Display the arguments passed as INFO parameters
    ROS_INFO("The following arguments were passed: ");
    for (int i = 0; i < argc; i++) {
        ROS_INFO_STREAM("Argument " << (i+1) << ": " << argv[i]);
    }

    // Changing the verbosity level using only source code
    /*
     * Use function set_logger_level to set a logger level
     * Main source code: https://docs.ros.org/en/api/rosconsole/html/namespaceros_1_1console.html#a7510136ca626d1b27655ddabfaef685a
     * 
     * Link: http://wiki.ros.org/roscpp/Overview/Logging#Setting_Verbosity_Levels
     */
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        // Notify the ROS system that a logger level has been changed
        /*
         * Link: https://docs.ros.org/en/api/rosconsole/html/namespaceros_1_1console.html#a7d8d9c277994cb315961ddbee540f1e1
         */
        ros::console::notifyLoggerLevelsChanged();
    }
    // Print a visible debug message
    ROS_DEBUG("This debug message must be visible");

    // Hold execution and prevent the node from automatically closing
    /*
     * There are lot of background processes happening by roscpp (including function callbacks if the node is a subscriber)
     * Therefore, the spin function hands over control of the node to the roscpp library until the node is shut down.
     * Another popular single threaded option is to use ros::spinOnce() which returns control to the program from roscpp
     */
    ros::spin();
    
    return 0;
}

