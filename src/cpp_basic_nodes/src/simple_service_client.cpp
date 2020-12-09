/*
 * This program does the following
 *      1. Demonstrates accessing a service client and calling services
 */

// Include the ROS header file
#include <ros/ros.h>
// Include the header file for service
#include "cpp_basic_nodes/AddAllFloat64Numbers_cpp.h"
// Include other header files too
#include <iostream>
#include <cstdlib>  // For atof function

using namespace std;

int main(int argc, char **argv) {
    // Initialize the node
    ros::init(argc, argv, "simple_cpp_service_client");
    // Create a NodeHandle
    /*
     * We need not use '~' for this one
     */
    ros::NodeHandle nh;
    
    // Create a service client
    /*
     * Just like a publisher has a publishing object, this has a service client object
     * The template is the service class (service type)
     * The first parameter is the service name (here: /simple_cpp_service_server/add_numbers)
     * 
     * Link: http://docs.ros.org/en/latest/api/roscpp/html/classros_1_1NodeHandle.html#a183d4cba0ea5c78f075304b91e07cc61
     */
    ros::ServiceClient srv_client = nh.serviceClient<cpp_basic_nodes::AddAllFloat64Numbers_cpp>("/simple_cpp_service_server/add_numbers");
    
    // Create a service object
    /*
     * You can use the service using this object. You store the request, call the service and then retrieve the response. 
     * This is like a message object for a publisher.
     */
    cpp_basic_nodes::AddAllFloat64Numbers_cpp srv_obj;
    
    // Append the argument values to the request
    ROS_INFO("Numbers passed to the node are ");
    for (int i = 0; i < argc; i++) {
        double num = atof(argv[i]); // Link (atof): http://www.cplusplus.com/reference/cstdlib/atof/
        ROS_INFO_STREAM(num);
        // Append the number to the request.data
        srv_obj.request.data.push_back(num);
    }
    
    // Call the service
    /*
     * The service handling function returns a boolean true on the successful execution of the service
     * 
     * Link: http://docs.ros.org/en/latest/api/roscpp/html/classros_1_1ServiceClient.html#a8a0c9be49046998a830df625babd396f
     */
    if (srv_client.call(srv_obj)) {
        ROS_INFO_STREAM("The result of the sum is " << srv_obj.response.sum);
    } else {
        ROS_ERROR_STREAM("Service " << srv_client.getService() << " did not respond");
    }
    
    return 0;
}
