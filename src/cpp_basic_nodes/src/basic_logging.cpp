/*
 * Log messages of different priority levels until terminated
 * More info here: http://wiki.ros.org/roscpp/Overview/Logging
 * 
 * Created by: Avneesh Mishra
 */

// Include everything
#include "ros/ros.h"
#include <iostream>
// Associate a shutdown signal
#include <signal.h>

using namespace std;

void custom_sigint(int sig);

int main(int argc, char **argv) {
    // Initialize node (don't use inbuilt SIGINT handler)
    ros::init(argc, argv, "cpp_logger", ros::init_options::NoSigintHandler);
    // Create node handle (not used here)
    ros::NodeHandle nh;
    // Create a SIGINT handler (override the one inbuilt)
    signal(SIGINT, custom_sigint);
    // Create a rate controller
    /*
        This is used to control something at a particular frequency and cause delays accordingly. Here, the frequency is 2 Hz.
        More here: http://wiki.ros.org/roscpp/Overview/Time#Sleeping_and_Rates
    */
    ros::Rate rt_ctrl(2);
    
    // Log these messages forever
    /*
        A common way to properly initialize and shutdown includes checking if ROS end is all right
        More here: http://wiki.ros.org/roscpp/Overview/Initialization%20and%20Shutdown
    */
    while (ros::ok()) { // While the ROS node is running
        // These print out debug messages on the terminal
        /*
            There are different levels of messages. You can configure the logger level of nodes using `rqt_logger_level`
            ```
            rosrun rqt_logger_level rqt_logger_level
            ```
        */
        ROS_DEBUG("This is a debug message");
        ROS_INFO("This is an info message");
        ROS_WARN("This is a warning message");
        ROS_ERROR("This is an error message");
        ROS_FATAL("This is a fatal message");
        rt_ctrl.sleep();
    }
    // Program ended
    return 0;
}

void custom_sigint (int sig) {
    // This function is called when Ctrl + C is pressed for the node
    cout << endl << "SIGINT signal called (Value: " << sig << ")" << endl;
    // Shutdown the node
    ros::shutdown();
}


/*
    1)
    Add the following to the CMakeLists.txt
    - Add the executable
        ```
        add_executable(cpp_basic_logger src/basic_logging.cpp)
        ```
    - Add target_link_libraries
        ```
        target_link_libraries(cpp_basic_logger ${catkin_LIBRARIES})
        ```
    
    2)
    To run this node, run the command
    ```
    rosrun cpp_basic_nodes cpp_basic_logger 
    ```
    
    3)
    You might first only get messages of priority >= INFO
    To enable DEBUG, run
    ```
    rosrun rqt_logger_level rqt_logger_level
    ```
    And select Debug in the ros logger
    
    4)
    Despite Ctrl + C being a common way to kill a node, it is suggested to use `rosnode`
    ```
    rosnode kill /cpp_logger
    ```
    This won't call the SIGINT handler
*/
