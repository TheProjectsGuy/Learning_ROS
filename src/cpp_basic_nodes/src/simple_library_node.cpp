/**
 * @file simple_library_node.cpp
 * @author Avneesh Mishra (123avneesh@gmail.com)
 * @brief 
 *      A basic C++ node written to demonstrate the use of `simple_library` in this package
 * @version 0.1
 * @date 2021-04-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// Basic include files
#include <ros/ros.h>

// Include the library
#include "basic_cpp_libs/simple_library.h"

using namespace std;

int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "simple_library_node");
    // Use the library
    string ret = say_hello("User");
    ROS_INFO_STREAM("Greeting: " << ret);
    // End
    return 0;
}
