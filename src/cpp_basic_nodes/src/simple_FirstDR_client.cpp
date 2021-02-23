/*
 * This program demonstrates the following
 *      1. Creating a dynamic reconfiguration client node that can use 'FirstDR'
 */

// Needed ROS header file
#include <ros/ros.h>

// The header file for using the dynamic reconfiguration client
/*
 * This file is included for the Client class which will create a client object.
 * Using this object, the server can be sent parameter updates.
 * 
 * Location (on disk): /opt/ros/noetic/include/dynamic_reconfigure
 */
#include <dynamic_reconfigure/client.h>

// Include the header file made
/*
 * This was made after building the `.cfg` file
 * 
 * Location (on disk): devel/include/cpp_basic_nodes (in workspace)
 * Documentation (on parameters): devel/share/cpp_basic_nodes/docs/FirstDRConfig-usage.dox
 */
#include <cpp_basic_nodes/FirstDRConfig.h>

using namespace std;

// Main function
int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "simple_cpp_firstdr_client");

    // Create the client object
    /*
     * This is an object of the Client class in dynamic_reconfig. It is like a publisher.
     * It in fact is more like a service client which will adjust parameters on the server.
     * Parameters are
     *  - Node name: Name of the node running the dynamic reconfiguration server.
     *      - For 'simple_cpp_firstdr_server', this will connect to service '/simple_cpp_firstdr_server/set_parameters'
     */
    dynamic_reconfigure::Client<cpp_basic_nodes::FirstDRConfig> client_obj(
        "simple_cpp_firstdr_server"
    );

    // Create the rate handler
    ros::Rate rate_hdlr = ros::Rate(1.0);
    int val = 50;   // Some value that we'll update
    // The object that we'll send (we'll send updates using this)
    /*
     * FirstDRConfig object will contain the parameters to be sent
     * To load the default values into this, use the function FirstDRConfig::__getDefault__()
     * 
     * FirstDRConfig.h file at: devel/include/cpp_basic_nodes/ (in workspace directory)
     */
    cpp_basic_nodes::FirstDRConfig send_config = cpp_basic_nodes::FirstDRConfig::__getDefault__();
    // Indefinitely publish values
    while (ros::ok()) {
        // Update the value
        val = (val + 1) % 100;
        // Update the object you're sending
        send_config.usr_int1 = val;
        send_config.usr_int2 = val;
        send_config.usr_int3 = val;

        // Send configuration
        /*
         * Use the setConfiguration function to send the configurations to the server
         * Note that if the callback function of the servermakes any changes to the parameters,
         * they will be referenced in the passed object.
         */
        client_obj.setConfiguration(send_config);
        ROS_INFO_STREAM("Reconfigure request sent for value " << val);

        // Delay
        rate_hdlr.sleep();
    }

    return 0;
}

