/*
 * This is a simple publisher that is a part of `launch1`
 * 
 * This node publishes a string message on a topic. 
 * It takes the following parameters
 * - l1pub_PubFreq [double]: The publishing frequency (in Hz) for the node
 * 
 * The node publishes to the following topics
 * - ~/pub_topic [std_msgs/String]: Publishing topic
 */

// Include basic header files
#include <ros/ros.h>
// Message header file for publisher
#include <std_msgs/String.h>

using namespace std;

int main(int argc, char** argv) {
    // Initialize the node
    ros::init(argc, argv, "launch1_publisher");
    // Create a node handler
    ros::NodeHandle nh("~");    // "~" -> Local namespace
    
    // Set the value of publishing frequency
    string param_name = "l1pub_PubFreq";
    double pub_freq = 0.1;  // Default publishing frequency
    if (nh.hasParam(param_name)) {  // If the parameter exists, try allotting value
        if (nh.getParam(param_name, pub_freq)) {
            ROS_INFO_STREAM("Publishing frequency set to " << 
                            pub_freq << " Hz from parameter '" << param_name << "'");
        } else {
            ROS_WARN_STREAM("Parameter '" << param_name << "' could not be parsed as double");
        }
    } else {
        ROS_INFO_STREAM("Could not find the parameter '" << param_name << "', using default value of " 
                        << pub_freq << " Hz instead");
    }
    
    // Set the rate of publisher
    ros::Rate rate_handler(pub_freq);
    
    // Create a publisher object
    ros::Publisher pub_obj = nh.advertise<std_msgs::String>("pub_topic", 100);
    
    // Message object
    double seq_num = 1;
    std_msgs::String msg_obj;
    // Main publishing loop
    while (ros::ok()) {
        // Create message
        msg_obj.data = to_string(seq_num);
        pub_obj.publish(msg_obj);
        seq_num += 1;
        ROS_DEBUG_STREAM("Published message [topic: " << pub_obj.getTopic() << "] '" << msg_obj.data << "'");
        ROS_INFO_STREAM("Published " << msg_obj.data);
        // Wait for the duration
        rate_handler.sleep();
    }
    
    return 0;
}
