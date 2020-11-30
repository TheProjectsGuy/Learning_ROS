/*
 * This program does the following
 *      1. Demonstrates how to create a publisher using the NodeHandle
 */

// Include ROS header file
#include <ros/ros.h>
// Include the header file for the message content
#include <std_msgs/String.h>
// Other header files
#include <iostream>

using namespace std;

// Create a callback function
/*
 * This function will be called whenever a message is published on a given topic (with a constant pointer
 * to the message as a parameter)
 */
void subscriber_callback(const std_msgs::String::ConstPtr& msg) {
    // Get the contents of the message
    string st = msg->data;  // Get the data into a string
    ROS_INFO_STREAM("Received a message: '" << st << "'");
}

int main(int argc, char **argv) {
    // Initialize the node
    ros::init(argc, argv, "simple_cpp_subscriber");
    // Initialize the NodeHandle
    ros::NodeHandle nh("~");
    
    // Create a subscriber request
    /*
     * Here, the nodehandle tells the master to route the messages from a topic to this node. A handling function
     * has to deal with the messages. The call returns an object of the Subscriber class
     * First parameter is the topic name (it will be scoped inside the node name)
     * Second parameter is the queue size
     * Third parameter is the callback function
     * Note that the actual callback will happen only in the spin function
     * 
     * Link: http://docs.ros.org/en/latest/api/roscpp/html/classros_1_1NodeHandle.html#a317fe4c05919e0bf3fb5162ccb2f7c28
     * Link (Subscriber class): http://docs.ros.org/en/latest/api/roscpp/html/classros_1_1Subscriber.html
     */
    ros::Subscriber msg_subs = nh.subscribe("subs_str", 1000, subscriber_callback);
    
    // Spin the node (call callbacks when messages are received, etc.)
    ros::spin();
    
    return 0;
}
