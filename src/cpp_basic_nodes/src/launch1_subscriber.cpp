/*
 * This is a simple subscriber that is a part of `launch1`
 * 
 * This node subscribes to a topic and simply prints out the message contents
 * The node subscribes to the following topics
 * - ~/subs_topic [std_msgs/String]: Subscribing topic
 */

// Include basic header files
#include <ros/ros.h>
// Message header file for subscriber
#include <std_msgs/String.h>

using namespace std;

// Callback (when a message is received) 
void callback_hdlr(const std_msgs::String::ConstPtr& msg, const ros::Subscriber* subs_obj) {
    // Received a message
    ROS_DEBUG_STREAM("Received a message");
    // Info
    ROS_INFO_STREAM("Received message [topic: " << subs_obj->getTopic() 
                    << " (" << subs_obj->getNumPublishers() << ")] '" << msg->data << "'");
}

int main(int argc, char **argv) {
    // Initialize the node
    ros::init(argc, argv, "launch1_subscriber");
    // Create NodeHandle
    ros::NodeHandle nh("~");
    // Create a subscriber object
    ros::Subscriber subs_obj = nh.subscribe<std_msgs::String>("subs_topic", 100, boost::bind(
        callback_hdlr, _1, &subs_obj
    ));
    // Set the node to spin
    ros::spin();
    return 0;
}

