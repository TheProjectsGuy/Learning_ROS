/*
 * This program does the following
 *      1. Demonstrates accessing messages that have array types
 *      2. Demonstrates creating a service server
 */

// Include the ROS header file
#include <ros/ros.h>

// Include the header file for service
/*
 * These header files are generated after building messages, services and actions.
 * 
 * Local location: devel/include (inside the workspace folder)
 */
#include "cpp_basic_nodes/AddAllFloat64Numbers_cpp.h"

using namespace std;

// Declare a service server function
/*
 * Everytime the binded service is called, this function would be called. It takes two objects
 * a request and a response. The request will contain data, and response will have to be filled.
 * Since the objects are call by reference, the response is sent after a successful execution.
 * Return true if execution is successful.
 */
bool add_numbers(cpp_basic_nodes::AddAllFloat64Numbers_cpp::Request &req,
                 cpp_basic_nodes::AddAllFloat64Numbers_cpp::Response &res) {
    double ans = 0;  // Holds the sum
    for (const double &d: req.data) {   // Request is actually a vector type
        ans += d;
    }
    res.sum = ans;
    ROS_INFO_STREAM("Received " << req.data.size() << " numbers to add, result is " << ans);
    return true;
}

int main(int argc, char**argv) {
    // Initialize the node
    ros::init(argc, argv, "simple_cpp_service_server");
    // Create a NodeHandle
    ros::NodeHandle nh("~");
    
    // Create a service server object
    /*
     * This tells the master that this node advertises a service (it can serve it, and therefore be a "server").
     * The first argument is the name of the service. Since the nodehandle was created with the ~, the name will
     * be /simple_cpp_service_server/add_numbers
     * The second argument is the service server function. It is a callback funciton.
     * Note that just like subscriber callbacks, even service callbacks happen
     * 
     * Link: http://docs.ros.org/en/latest/api/roscpp/html/classros_1_1NodeHandle.html#a20f789c8500181d91f8fecba4cd51cbc
     * Link (ServiceServer class): http://docs.ros.org/en/latest/api/roscpp/html/classros_1_1ServiceServer.html
     */
    ros::ServiceServer srv_server = nh.advertiseService("add_numbers", add_numbers);
    
    // Info message
    /*
     * The ros::ServiceServer object can be used to access properties of the service
     * 
     * Link (ServiceServer class): http://docs.ros.org/en/latest/api/roscpp/html/classros_1_1ServiceServer.html
     */
    ROS_INFO_STREAM("Service " << srv_server.getService() << " has been set up");
    // Set the entire thing ro run using spin
    ros::spin();
    
    return 0;
}
