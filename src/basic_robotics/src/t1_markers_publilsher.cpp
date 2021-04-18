/**
 * @file t1_markers_publilsher.cpp
 * @author Avneesh Mishra (123avneesh@gmail.com)
 * @brief 
 *      A node that publishes markers and a fixed transformation.
 *          1. Publish a fixed transformation (on /tf_static) for {f2} -> {f3}
 *          2. Publish a sphere marker
 * @version 0.1
 * @date 2021-04-17
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// Basic header files
#include <ros/ros.h>
// Header file for broadcasting static transformations
/*
 * Static transformations are published on /tf_static. Just like how transform_broadcaster.h
 * is for /tf, static_transform_broadcaster is for /tf_static. Static transforms are meant to
 * be published only once
 * 
 * Docs (C++ API): https://docs.ros.org/en/latest/api/tf2_ros/html/c++/
 * Docs (StaticTransformBroadcaster class): https://docs.ros.org/en/latest/api/tf2_ros/html/c++/classtf2__ros_1_1StaticTransformBroadcaster.html
 */
#include <tf2_ros/static_transform_broadcaster.h>
// Transform geometry message
#include <geometry_msgs/TransformStamped.h>
// Quaternions
#include <tf2/LinearMath/Quaternion.h>
// Include visualization message
/*
 * Markers are used to display things in RViz like spheres, arrows, etc.
 *
 * Reference (Markers): https://wiki.ros.org/rviz/DisplayTypes/Marker
 * Reference (message): https://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html
 */
#include <visualization_msgs/Marker.h>

// Some #define macros for the program
// Convert 'x' from degrees to radians
#define DEG_TO_RAD(x) ( (M_PI/(double)180) * ((double) x) )
// Convert 'x' from radians to degrees
#define RAD_TO_DEG(x) ( ((double)180/M_PI) * ((double) x) )

using namespace std;


// Main function
int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "t1_cpp_markers_publisher");
    // Create node handler
    ros::NodeHandle nh("~");

    // Create publisher to broadcast static transformation
    tf2_ros::StaticTransformBroadcaster stf_broadcaster;
    // Create a static transformation message for {f2} -> {f3} ({f3} in {f2})
    geometry_msgs::TransformStamped stf_msg;
    stf_msg.header.seq = 1;
    stf_msg.header.stamp = ros::Time::now();
    stf_msg.header.frame_id = "f2"; // From {f2}
    stf_msg.child_frame_id = "f3";  // To {f3}
    // {f3} is at [0, 0, 1] in {f2}
    stf_msg.transform.translation.x = 0;
    stf_msg.transform.translation.y = 0;
    stf_msg.transform.translation.z = 1;
    // Rotate along Z (yaw) by -45 deg
    tf2::Quaternion f2_f3_quat;
    f2_f3_quat.setRotation(tf2::Vector3(0, 0, 1), DEG_TO_RAD(-45)); // -45 deg along Z axis
    stf_msg.transform.rotation.w = f2_f3_quat.w();
    stf_msg.transform.rotation.x = f2_f3_quat.x();
    stf_msg.transform.rotation.y = f2_f3_quat.y();
    stf_msg.transform.rotation.z = f2_f3_quat.z();
    // Publish a static transformation
    stf_broadcaster.sendTransform(stf_msg);

    // Create a publisher for the markers (topic name must match that in Rviz)
    ros::Publisher marker_publisher = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 10);
    // Sphere marker
    visualization_msgs::Marker sphere_marker;
    sphere_marker.header.seq = 0;
    sphere_marker.header.frame_id = "f3";   // In {f3}
    sphere_marker.ns = "frames";    // Namespace for filtering
    sphere_marker.id = 10;  // A unique ID in the namespace
    sphere_marker.type = visualization_msgs::Marker::SPHERE;    // This is a sphere
    sphere_marker.action = visualization_msgs::Marker::ADD; // Add this object (same as modify)
    sphere_marker.color.a = 1;
    sphere_marker.color.r = 1;
    sphere_marker.color.g = 0;
    sphere_marker.color.b = 0;  // Red color marker (with alpha = 1)
    sphere_marker.pose.orientation.w = 1;
    sphere_marker.pose.orientation.x = 0;
    sphere_marker.pose.orientation.y = 0;
    sphere_marker.pose.orientation.z = 0;   // Assign orientation (no rotation)
    sphere_marker.scale.x = sphere_marker.scale.y = sphere_marker.scale.z = 0.1;  // 0.1m diameter
    // Sphere marker parameters
    double freq = 2.0 * M_PI / (5.0);   // 5 second time period
    ros::Time start_t = ros::Time::now();   // Starting time    
    ros::Rate rate_hdlr(30);    // 30 Hz for publishing rate

    // Main loop
    while (ros::ok()) {
        ros::Time curr_time = ros::Time::now(); // Current time
        // Update header
        sphere_marker.header.seq += 1;
        sphere_marker.header.stamp = curr_time;
        double dt = (curr_time - start_t).toSec();  // Time delay
        sphere_marker.pose.position.x = 1.0 * cos(dt*freq);
        sphere_marker.pose.position.y = 1.0 * sin(dt*freq);
        sphere_marker.pose.position.z = 0;
        // Publish the sphere marker
        marker_publisher.publish(sphere_marker);  
        // Sleep
        rate_hdlr.sleep();
    }
    return 0;
}
