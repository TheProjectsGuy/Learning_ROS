/*
 * This program does the following
 *      1. Demonstrates how to create a publisher using the NodeHandle
 *      2. Introduces executing things at a frequency
 */

// Include ROS header file
#include <ros/ros.h>
// Include the header file for the message content
/*
 * Every message actually has a data structure to it (implemented as a class in C++)
 * This class is stored in a header file which you must include in order to use that message
 * 
 * Location on local PC: /opt/ros/noetic/include, inside std_msgs/String.h
 */
#include <std_msgs/String.h>
// Other header files
#include <iostream>

using namespace std;

int main(int argc, char **argv) {
    // Initialize the node
    ros::init(argc, argv, "simple_cpp_publisher");
    
    // Initialize the NodeHandle
    /*
     * The first parameter is the namespace parameter. The ~ character makes the names of all the topics 
     * and services prefixed with /node_name/ (which is /simple_cpp_publisher/ here)
     * 
     * More here: http://wiki.ros.org/Names
     * Constructor reference: http://docs.ros.org/en/latest/api/roscpp/html/classros_1_1NodeHandle.html#a0eae6a7fd8c216c3f0860aec44ca81ba
     */
    ros::NodeHandle nh("~");
    
    // Create a publisher object
    /*
     * We will use this object to publish messages
     * The template parameter (in <>) is the class of the message type
     * The first parameter is the topic name and the second parameter is the queue size
     * 
     * Link: http://docs.ros.org/en/latest/api/roscpp/html/classros_1_1NodeHandle.html#a6b655c04f32c4c967d49799ff9312ac6
     */
    ros::Publisher str_publisher = nh.advertise<std_msgs::String>("hello_str", 1000);
    
    // Create a rate controller
    /*
     * For the purpose of this tutorial, the messages will be published in a loop, once every two seconds (i.e. 0.5 Hz)
     * 
     * Link: http://docs.ros.org/en/latest/api/rostime/html/classros_1_1Rate.html#ad7ef59c5fb4edb69c6a9471987c3117d
     */
    ros::Rate rate_ctrl(0.5);
    
    // Some variables to keep track of things
    long int seq = 0;   // Sequence number
    
    // Loop while ROS is "ok"
    /*
     * ros::ok() is false when shutdown is called
     * 
     * Link: http://docs.ros.org/en/latest/api/roscpp/html/namespaceros.html#a276d68870be2125b1cde229fee013e45
     */
    while (ros::ok()) {
        // Construct a message
        /*
         * This object will be assigned all data and given to the ros::Publisher object created earlier.
         * The underlying ROS infrastructure will then take the message and publish it to the topic.
         */
        std_msgs::String msg;
        /* 
         * A string object which will finally be assigned to the message
         * Reference: http://www.cplusplus.com/reference/string/
         */
        std::string str_msg = "Hello, World!";
        str_msg += " [I: " + std::to_string(seq) + "]"; // Append sequence number
        seq += 1;   // Increment to the next sequence
        msg.data = str_msg;    // Assign it data
        
        // Publish it on the topic
        /*
         * Use the publish member function to publish a message
         * 
         * Link: http://docs.ros.org/en/latest/api/roscpp/html/classros_1_1Publisher.html#a0933b8fb71fa577e0eeed771d37631ed
         */
        str_publisher.publish(msg);
        
        // Print it as INFO message
        ROS_INFO_STREAM("Published " << str_msg);   // You can verify the time and sequence number for the rate (during execution)
        
        // Call the ros::spinOnce function
        /*
         * This is ideal to the ros::spin(), but this will return control to the loop and the execution will resume from after the call point
         * 
         */
        ros::spinOnce();
        
        // Cause a delay to obtain the frequency
        /*
         * Causes a delay. Since we gave a frequency of 0.5 Hz, the delay will be of approx. 2 seconds
         * 
         * Link: http://docs.ros.org/en/latest/api/rostime/html/classros_1_1Rate.html#ae5664d27cda1b17a103347560259e945
         */
        rate_ctrl.sleep();
    }
}
