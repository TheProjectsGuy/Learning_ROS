/*
 * This is a node that prints out all the parameters available (just names / keys)
 */

// Basic header files
#include <ros/ros.h>

using namespace std;

int main(int argc, char **argv) {
    // Initialize node
    ros::init(argc, argv, "launch1_params");
    // Nodehandler
    ros::NodeHandle nh;
    
    // Get the name of this node
    // Link: https://docs.ros.org/en/latest/api/roscpp/html/namespaceros_1_1this__node.html#a2a991f67172b59ccd7e5b4aeca9b76b7
    ROS_INFO_STREAM("Node " << ros::this_node::getName() << " has been activated");
    
    // Get the list of parameters
    vector<string> param_list;
    if (!nh.getParamNames(param_list)) {
        ROS_FATAL("Could not get a list of paramters");
        ros::shutdown();
    }
    // Print out all of them
    ROS_INFO_STREAM("Found " << param_list.size() << " parameters");
    for (int i = 0; i < param_list.size(); i++) {
        ROS_INFO_STREAM("Parameter " << i << ": `" << param_list.at(i) << "`");
    }
    
    // Set the node to spin indefinitely
    ros::spin();
    
    return 0;
}
