/*
 * This program demonstrates the following
 *      1. Creating a dynamic reconfiguration server node that can use 'FirstDR'
 */

// Needed ROS header file
#include <ros/ros.h>
// The header file for using the dynamic reconfiguration server
/*
 * This file is included for the Server class which will help create a server object
 * 
 * Location (on disk): /opt/ros/noetic/include/dynamic_reconfigure
 */
#include <dynamic_reconfigure/server.h>
// Include the header file made
/*
 * This was made after building the `.cfg` file
 * 
 * Location (on disk): devel/include/cpp_basic_nodes (in workspace)
 * Documentation (on parameters): devel/share/cpp_basic_nodes/docs/FirstDRConfig-usage.dox
 */
#include <cpp_basic_nodes/FirstDRConfig.h>

using namespace std;

// A callback function to call whenever the server receives a parameter modification
/*
 * This function is called once when setting up (default values in the config, level is 0xffffffff - all ones) and 
 * every time a client makes a parameter modification request.
 *  
 * Parameters:
 *  - config: The configuration object. Different paraemters are members of this object.
 *  - level: The level which is the OR of the levels of changed parameters
 */
void callback_function(cpp_basic_nodes::FirstDRConfig &config, uint32_t level) {
    string OP_sel_str;  // String for parameter OP_sel
    switch(config.OP_sel) {
        case cpp_basic_nodes::FirstDR_OP_1:
            OP_sel_str = "Option 1";
            break;
        case cpp_basic_nodes::FirstDR_OP_2:
            OP_sel_str = "Option 2";
            break;
        case cpp_basic_nodes::FirstDR_OP_3:
            OP_sel_str = "Option 3";
            break;
        default:
            OP_sel_str = "Invalid selection";
            break;
    }
    ROS_INFO_STREAM(
        "Reconfig request (level = " << level << ")" << endl
        << "\t" << "user_int = " << config.user_int << endl
        << "\t" << "Level testing parameters" << endl
        << "\t\t" << "usr_int1 = " << config.usr_int1 << endl
        << "\t\t" << "usr_int1 = " << config.usr_int2 << endl
        << "\t\t" << "usr_int1 = " << config.usr_int3 << endl
        << "\t" << "user_bool = " << config.user_bool << endl
        << "\t" << "user_str = " << config.user_str << endl
        << "\t" << "OP_sel = " << OP_sel_str << " (" << config.OP_sel << ")"
    );
}

// Main function
int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "simple_cpp_firstdr_server");

    // Create server
    /*
     * This is an object of the Server class in dynamic_reconfig
     * It is like a subscriber, in fact it subscribes to a topic called
     *  - NODE_NAME/parameter_descriptions (msg: dynamic_reconfigure/ConfigDescription)
     *      - Descriptions of all the parameters in the parameter server
     *  - NODE_NAME/parameter_updates (msg: dynamic_reconfigure/Config)
     *      - The updated values of the parameters
     *  
     * Client nodes are expected to publish to these topics (using client interface, like in services 
     * and actions)
     */
    dynamic_reconfigure::Server<cpp_basic_nodes::FirstDRConfig> server;
    // Create a callback handler
    /*
     * Boost is used to create a callback. The callback_handler is essentially a boost::function type
     */
    dynamic_reconfigure::Server<cpp_basic_nodes::FirstDRConfig>::CallbackType callback_handler;
    // Bind callback
    callback_handler = boost::bind(callback_function, _1, _2);
    // Set the callback for the server
    server.setCallback(callback_handler);

    // Set the node into spinning
    ROS_INFO("Spinning Node");
    ros::spin();
    return 0;
}
