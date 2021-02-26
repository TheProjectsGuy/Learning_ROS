/*
 * The purpose of this file is as follows:
 *      1. To be used as a dummy tf publisher (also called transform broadcaster) for Tutorial 1
 *      2. Create a transform broadcaster that can publish data for `TF` type
 */

// Include basic libraries
#include <ros/ros.h>
// Include the header file for transformations
/*
 * Include the wrapper for broadcasting transformations. This will make the direct publisher 
 * abstract and the whole process would become easier to deal with.
 * 
 * Docs (C++ API): https://docs.ros.org/en/latest/api/tf2_ros/html/c++/
 */
#include <tf2_ros/transform_broadcaster.h>
// Include the header file for creating quaternions
/*
 * The mathematical side of creating and handling quaternion transformations is handled
 * 
 * Docs (C++ API): https://docs.ros.org/en/latest/api/tf2/html/Quaternion_8h.html
 */
#include <tf2/LinearMath/Quaternion.h>
// Include the message type that will be published
/*
 * The topic '/tf' has messages of type 'tf2_msgs/TFMessage' which is basically an array of messages
 * of type 'geometry_msgs/TransformStamped'. Include that message header.
 * 
 * Ref (.msg file): https://docs.ros.org/en/api/geometry_msgs/html/msg/TransformStamped.html
 */
#include <geometry_msgs/TransformStamped.h>

// Some #define macros for the program
// Convert 'x' from degrees to radians
#define DEG_TO_RAD(x) ( (M_PI/(double)180) * ((double) x) )
// Convert 'x' from radians to degrees
#define RAD_TO_DEG(x) ( ((double)180/M_PI) * ((double) x) )

using namespace std;

int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "t1_cpp_tf_broadcaster");
    // Create the node handler
    ros::NodeHandle nh("~");
    
    // Something that will handle rate
    ros::Rate rate_hdlr = ros::Rate(1); // 1 Hz
    // An object that will broadcast transformations
    tf2_ros::TransformBroadcaster tf_broadcaster;

    // Create transformations
    /*
     * Transformations: {global} -> {f1} -> {f2}
     * {f1} is +2m in X w.r.t. {global}
     * {f2} is +1m in X, +1m in Y and 45 deg Z (yaw) rotated w.r.t. {f1}
     */
    geometry_msgs::TransformStamped trans1;    // Transform {global} -> {f1}
    trans1.header.seq = 0; // Starting sequence
    trans1.header.frame_id = "global";  // From {global}
    trans1.child_frame_id = "f1";       // To {f1}
    trans1.transform.translation.x = 1; // {global} to {f1}: +1m in X
    trans1.transform.rotation.w = 1;    // This is required to normalize the quaternion
    geometry_msgs::TransformStamped trans2;     // Transform {f1} -> {f2}
    trans2.header.seq = 0;
    trans2.header.frame_id = "f1";  // From {f1}
    trans2.child_frame_id = "f2";   // To {f2}
    trans2.transform.translation.x = 1; // +1m in X
    trans2.transform.translation.y = 1; // +1m in Y
    tf2::Quaternion quat;
    quat.setRotation(tf2::Vector3(0, 0, 1), DEG_TO_RAD(45));    // 45 deg along Z axis
    // Assign quaternion to the message
    trans2.transform.rotation.w = quat.w();
    trans2.transform.rotation.x = quat.x();
    trans2.transform.rotation.y = quat.y();
    trans2.transform.rotation.z = quat.z();

    // Publish messages (in a continuous loop)
    while (ros::ok()) {
        // Update times
        trans1.header.seq += 1;
        trans1.header.stamp = ros::Time::now();
        trans2.header.seq += 1;
        trans2.header.stamp = ros::Time::now();
        // Send the transformations
        tf_broadcaster.sendTransform(trans1);
        tf_broadcaster.sendTransform(trans2);
        // Handle rate
        rate_hdlr.sleep();
    }
    return 0;
}
