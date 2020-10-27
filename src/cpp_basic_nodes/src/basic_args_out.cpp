/*
 * Print out the arguments passed to the node
 * 
 * Created by: Avneesh Mishra
 */

// Start by including the ROS header file
/*
    This file is present in the directory /opt/ros/noetic/include
    File: ros/ros.h
    It includes other core ROS header files
*/
#include "ros/ros.h"
// Include other files
#include <iostream>

using namespace std;

// Main function
int main(int argc, char **argv)
{
    // Initialize the node
    /*
        This function tells the master (roscore) about the existence of this node
        Subsequently, this node registers with the ROS Master using the name given here
        There are some arguments that are specific to the master, they can be passed through the node call
        You must call `ros::init` before using any part of ROS 
    */
    ros::init(argc, argv, "cpp_arg_out");
    cout << "Initialized the node" << endl;
    // Create a NodeHandle object
    /*
        The NodeHandle handles all the communication with the master. It is the object used to communicate 
        with everything on ROS end. However, this program shall not use it.
    */
    ros::NodeHandle n;

    // Print the arguments
    for (int i = 0; i < argc; i++) {
        cout << "Argument " << (i+1) << ": " << argv[i] << endl;
    }

    return 0;
}

/*
    Note: After writing this code, the following has to be done in the CMakeLists.txt file in the cpp_basic_nodes
    package
    1. Go to the build section and add a line for `add_executable`. This will create an executable for the code written above
        ```
        add_executable(cpp_basic_args_out src/basic_args_out.cpp)
        ```
    2. Add the target_link_libraries for the executable. This will link the catkin libraries with the executable
        ```
        target_link_libraries(cpp_basic_args_out ${catkin_LIBRARIES})
        ```
    
    Then, go to the workspace folder and run the `catkin_make` command.

    Note: The first argument passed to the program is the full path to the executable
        - The executable can be found in devel/lib/cpp_basic_nodes/cpp_basic_args_out in the workspace directory
*/
