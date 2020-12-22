/*
 * This program demonstrates the following
 *      1. Creating an action client
 */

// Include ROS header file
#include <ros/ros.h>
// Include the header file for the action
#include "cpp_basic_nodes/CountNumbers_cppAction.h"

// Include header file for action client
/*
 * This includes the underlying actionlib implementation to carry out the action communications on the ROS side
 * 
 * Location: /opt/ros/noetic/include/
 * Link (actionlib doc): https://docs.ros.org/en/api/actionlib/html/
 */
#include <actionlib/client/simple_action_client.h>

// Alias name for the client datatype
using Client_t = actionlib::SimpleActionClient<cpp_basic_nodes::CountNumbers_cppAction>;

using namespace std;

// Declare a function to be called when a feedback is received
/*
 * This will later be used with the action client object when sending the goal
 */
void recv_feedback(const cpp_basic_nodes::CountNumbers_cppFeedbackConstPtr& fb_obj) {
    ROS_INFO_STREAM("Feedback received: " << fb_obj->curr_number);
}

int main(int argc, char **argv) {
    // Initialize the node
    ros::init(argc, argv, "simple_cpp_action_client");
    // Create NodeHandle
    ros::NodeHandle nh("~");
    
    // Create an action client object
    /*
     * Just as a service client is used to call a service server, an action client object is used to call an action server
     * The first argument is the action name and the second argument is 
     * 
     * Link: https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1SimpleActionClient.html#a416667d5c870abc27227e45a492c1e3f
     */
    Client_t action_client_obj("/simple_cpp_action_server/count_numbers", true);
    ROS_INFO("Waiting for the /simple_cpp_action_server/count_numbers action");
    // Wait for the server with 1s timeout
    /*
     * Used to wait until a timeout occurrs. If a timeout does occur, then false is returned and if it can connect to the action server,
     * then true is returned.
     * The first argument is the duration
     * 
     * Link: https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1SimpleActionClient.html#a92acc8c2fdec699bd5070bb83263786f
     */
    if (!action_client_obj.waitForServer(ros::Duration(2, 0))) {
        ROS_FATAL_STREAM("Action server not available, timeout occurred");
        ros::shutdown();    // Shutdown the node
    } else {
        ROS_INFO("Found the acition server");
        // Create a goal object
        cpp_basic_nodes::CountNumbers_cppGoal goal_obj;
        goal_obj.target_number = (argc > 1) ? stoi(string(argv[1])) : 5; // Count upto 5 (default) or the first argument
        goal_obj.del_ms = (argc > 2) ? stoi(string(argv[2])) : 500; // Count with delay of 500 ms (default) or the second argument
        // Send the goal
        /*
         * This is a call to the server
         * 
         * The first argument is the goal object
         * The second argument is a function for the callback when the action is done
         * The third argument is a function for the callback when the action transitions to active
         * The fourth argument is a function for the feedback values received
         * 
         * Link: https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1SimpleActionClient.html#ae6a2e6904495e7c20c59e96af0d86801
         */
        action_client_obj.sendGoal(goal_obj, 
                                   Client_t::SimpleDoneCallback(),
                                   Client_t::SimpleActiveCallback(),
                                   boost::bind(recv_feedback, _1)   // Feedback
        );
        ROS_INFO("Waiting for the response");
        int wait_duration = goal_obj.del_ms * goal_obj.target_number + 1;
        // Wait for the response
        /*
         * Wait for the action server to finish the task and send a result
         * 
         * Link: https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1SimpleActionClient.html#a94b5a38fae6917f83331560b81eea341
         */
        if (!action_client_obj.waitForResult(ros::Duration(wait_duration, 0))) {
            ROS_INFO("Got a response");
            // Get the state of the result
            /*
             * The state returned is SUCCEEDED if everything went fine
             * 
             * Link: https://docs.ros.org/en/api/actionlib/html/classactionlib_1_1SimpleActionClient.html#ac02703c818902c18e72844e58b4ef285
             */
            if (action_client_obj.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                // Get the result
                cpp_basic_nodes::CountNumbers_cppResultConstPtr res_obj = action_client_obj.getResult();
                ROS_INFO_STREAM("Response received: " << res_obj->res_note);
            } else {
                ROS_ERROR_STREAM("State not a 'success':" << action_client_obj.getState().toString());
            }
        }
    }
    ROS_INFO("Program ended");
    return 0;
}
