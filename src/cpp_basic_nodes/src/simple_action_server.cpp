/*
 * This program demonstrates the following
 *      1. Creating an action server
 */

// Include ROS header file
#include <ros/ros.h>

// Include the header file for the action
/*
 * This header file was generated in the build process of the action.
 * Note the "Action" suffix in the end
 * 
 * Location: devel/include (inside the workspace folder)
 */
#include "cpp_basic_nodes/CountNumbers_cppAction.h"

// Include the actionlib header file for simple servers
/*
 * This includes the underlying actionlib implementation to carry out the action communications on the ROS side
 * 
 * Location: /opt/ros/noetic/include/
 * Link (actionlib doc): https://docs.ros.org/en/api/actionlib/html/
 */
#include <actionlib/server/simple_action_server.h>

// Create an alias datatype name for the actionserver type
/*
 * Instead of using the long name, create a shorter name for it
 */
using Server_t = actionlib::SimpleActionServer<cpp_basic_nodes::CountNumbers_cppAction>;

using namespace std;

// Declare a function that will handle calls to the funcion server (callback function)
/*
 * This function will handle an entire call, its return type is void and takes the "Goal" as a parameter
 * The other parameter is to handle a graceful exit of the service (see the call to bind this function)
 */
void execute_action(const cpp_basic_nodes::CountNumbers_cppGoalConstPtr& goal, Server_t* sv) {
    // Do the things needed to execute the goal
    ros::Rate del_ms(1000.0/double(goal->del_ms));  // Create a delay based on frequency from delay passed
    int target_number = goal->target_number;
    ROS_INFO_STREAM("Received a goal (" << goal->del_ms << " ms, up to " << goal->target_number << "), starting to count");
    // Create an object for packaging feedback messages in
    cpp_basic_nodes::CountNumbers_cppFeedback fb_obj;
    // Run a for loop
    for (int i = 1; i <= target_number; i++) {
        ROS_INFO_STREAM("Count at " << i);
        // Send a feedback to the client regarding the state of goal
        /*
         * Link: https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1SimpleActionServer.html#a497b5bb99c01e9424f3d8d0eada2a934
         */
        fb_obj.curr_number = i;
        sv->publishFeedback(fb_obj);    // Publish feedback
        // Wait for the next count
        del_ms.sleep();
    }
    // Create a response (result) message
    cpp_basic_nodes::CountNumbers_cppResult res_obj;
    res_obj.res_note = "Finished counting upto " + to_string(target_number);
    // Graceful exit from the call (with the result)
    sv->setSucceeded(res_obj, "Success");
}


int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "simple_cpp_action_server");
    // Create a nodehandle
    ros::NodeHandle nh("~");
    // Create an action server object
    /*
     * Just like in the case of a simple server there was an object created to deal with the subscription, there is one here too.
     * First argument is the node handle
     * Second argumet is the name under which the action is being published
     * Third argument is the callback function, but binded using the boost::bind function
     *      This function creates custom function callbacks
     * 
     * Link (boost::bind): https://www.boost.org/doc/libs/1_75_0/libs/bind/doc/html/bind.html#bind.purpose
     * Link: https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1SimpleActionServer.html
     */
    Server_t action_server_obj(nh, "count_numbers", boost::bind(execute_action, _1, &action_server_obj), false);
    // Start the action server
    action_server_obj.start();  // Now, calls to the server can be made
    // Spin the node
    ros::spin();
    return 0;
}
