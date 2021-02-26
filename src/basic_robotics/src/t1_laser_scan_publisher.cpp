/*
 * The purpose of this file is as follows:
 *      1. To be used as a dummy sensor for Tutorial 1
 *      2. Create a publisher that can publish data for `LaserScan` type
 * 
 * This node takes the following paraemters
 *  - ~/pub_topic (string): Publishing topic name (default: /t1_laser_scan)
 *  - ~/pub_freq (double): Publishing frequency in Hz (default: 10 Hz)
 *  - ~/pub_frame (string): Frame for header (header -> frame_id) (default: global)
 */

// Include basic libraries
#include <ros/ros.h>
// Include the header file for LaserScan message
/*
 * Reference (.msg file): https://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html
 * Location (on Disk): /opt/ros/noetic/include/sensor_msgs
 */
#include <sensor_msgs/LaserScan.h>

// Some #define macros for the program
// Convert 'x' from degrees to radians
#define DEG_TO_RAD(x) ( (M_PI/(double)180) * ((double) x) )
// Convert 'x' from radians to degrees
#define RAD_TO_DEG(x) ( ((double)180/M_PI) * ((double) x) )

using namespace std;

int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "t1_cpp_laser_scan_publisher");
    // Create the node handler
    ros::NodeHandle nh("~");
    
    // Configurations
    // Configure the publishing topic
    string pub_topic_name = "placeholder";
    if (nh.hasParam("pub_topic")) {
        if (nh.getParam("pub_topic", pub_topic_name)) {
            ROS_INFO_STREAM("Publishing topic set to '" << pub_topic_name << "'");
        } else {
            pub_topic_name = "/t1_laser_scan";  // Default topic name
            ROS_WARN_STREAM("Publishing topic could not be parsed to string, using default '" << pub_topic_name << "'");
        }
    } else {
        pub_topic_name = "/t1_laser_scan";  // Default topic name
        ROS_INFO_STREAM("Publishing topic set to '" << pub_topic_name << "'");
    }
    // Configure the publishing frequency
    double pub_frequency = -1;
    if (nh.hasParam("pub_freq")) {
        if (nh.getParam("pub_freq", pub_frequency)) {
            ROS_INFO_STREAM("Publishing frequency set to '" << pub_frequency << " Hz'");
        } else {
            pub_frequency = 10;
            ROS_WARN_STREAM("Publishing frequency could not be parsed to double, using default '" << pub_frequency << " Hz'");
        }
    } else {
        pub_frequency = 10;
        ROS_INFO_STREAM("Publishing frequency set to '" << pub_frequency << " Hz'");
    }
    // Configure the publishing frame
    string pub_frame = "placeholder";
    if (nh.hasParam("pub_frame")) {
        if (nh.getParam("pub_frame", pub_frame)) {
            ROS_INFO_STREAM("Publishing frame set to '" << pub_frame << "'");
        } else {
            pub_frame = "global";
            ROS_WARN_STREAM("Publishing frame could not be mapped to double, using default '" << pub_frame << "'");
        }
    } else {
        pub_frame = "global";
        ROS_INFO_STREAM("Publishing frame set to '" << pub_frame << "'");
    }

    // Create a publisher object
    ros::Publisher pub_obj = nh.advertise<sensor_msgs::LaserScan>(pub_topic_name, 100);
    // Create the Rate handler (for publishing frequency)
    ros::Rate rate_hdlr = ros::Rate(pub_frequency);

    // Create a message that we will publish
    sensor_msgs::LaserScan pub_msg;
    // Configure defaults (that will remain same throughout, sensor properties: you may find these in datasheets)
    pub_msg.header.seq = 0; // Set sequence to 0 (beginning)
    pub_msg.header.frame_id = pub_frame;    // Publishing frame (for header)
    pub_msg.angle_min = DEG_TO_RAD(-135);   // Minimum angle of view
    pub_msg.angle_max = DEG_TO_RAD(135);    // Maximum angle of view
    pub_msg.angle_increment = DEG_TO_RAD(1);    // Increments of 1 degree
    pub_msg.scan_time = 1.0 / pub_frequency;    // Time for the entire scan
    pub_msg.time_increment = pub_msg.scan_time / ((float)270);  // Scan / (270 measurements = ((max - min) / (increment)))
    pub_msg.range_max = 5;  // 5 m maximum range
    pub_msg.range_min = 0.10;   // 10 cm minimum range

    // Start the publishing loop
    while (ros::ok()) {
        // Assign values to the publishing message
        pub_msg.header.stamp = ros::Time::now();    // Assign correct time to the header
        pub_msg.header.seq += 1;
        vector<float> readings;
        for (int i = 0; i < 270; i++) {
            readings.push_back(3);  // For now, all readings are at 3 m
        }
        pub_msg.ranges = readings;
        
        // Publish the message
        pub_obj.publish(pub_msg);

        // Delay
        rate_hdlr.sleep();
    }

    return 0;
}
