/*
 * User defined Gazebo Plugin for control of a Single Link Robot
 * This is the header file (declarations only)
 * 
 * This file is the header file for a Gazebo plugin, made for
 * controlling the single link robot in Tutorial 5. Most of the
 * inference can be derived from template [1] as well as the
 * header [2] and source code [3] of a simple but elaborate
 * plugin: skid_steer_drive. The source code of the template [4]
 * can also be referred to. See the Gazebo API reference [5] for
 * more information on header files and contents.
 * 
 * 
 * [1]: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/include/gazebo_plugins/gazebo_ros_template.h
 * [2]: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/include/gazebo_plugins/gazebo_ros_skid_steer_drive.h
 * [3]: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_skid_steer_drive.cpp
 * [4]: https://github.com/ros-simulation/gazebo_ros_pkgs/blob/kinetic-devel/gazebo_plugins/src/gazebo_ros_template.cpp
 * [5]: https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/index.html
 * 
 */


#ifndef GAZEBO_ROS_SLBOT_CONTROLLER
#define GAZEBO_ROS_SLBOT_CONTROLLER

// Main ROS header file
#include <ros/ros.h>

// Gazebo header files
/*
 * Location on disk: /usr/include/gazebo-11/gazebo/physics
 * API reference: https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/group__gazebo__physics.html
 */
#include <gazebo/physics/physics.hh>
/*
 * API reference: https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/group__gazebo__transport.html
 */
#include <gazebo/transport/TransportTypes.hh>
/*
 * API reference: https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/group__gazebo__common.html
 */
#include <gazebo/common/common.hh>
// For SDF elements
/*
 * API reference: https://osrf-distributions.s3.amazonaws.com/sdformat/api/dev/classsdf_1_1v11_1_1Element.html
 */
#include <sdf/sdf.hh>
// Header file for publishing sensor_msgs/JointState
#include <sensor_msgs/JointState.h>


// Main namespace
namespace gazebo {
    // A class for wrapping the entire plugin
    /*
     * A class wrapper for controlling the single link robot plugin.
     * As this is a model plugin (accessing joints and links), we
     * inherit the ModelPlugin [1]
     * 
     * [1]: https://osrf-distributions.s3.amazonaws.com/gazebo/api/dev/classgazebo_1_1ModelPlugin.html
     */
    class GazeboRosSlbotController : public ModelPlugin {
        private:
            physics::WorldPtr world;    // Simulation World
            physics::ModelPtr parent;   // The model XML tree
            physics::JointPtr joint;    // The joint that is to be controlled
            event::ConnectionPtr update_connection; // Update the connection
            // ROS variables
            ros::NodeHandle* _nh;   // NodeHandler
            ros::Publisher joint_angle_publisher_;
            // Variables for this node
            std::string robot_namespace; // Namespace for node handler
            struct {    // Parameters for this plugin (wrapped in one object)
                std::string joint_name; // Joint name
                std::string js_topic;   // JointState topic name
                double rate_hz; // Rate in Hz (for updating)
                double update_period_sec;   // Update period (in seconds)
                common::Time last_update_time;  // Time last update was done
                sensor_msgs::JointState js_msg; // JointState message
            } params;
        public:
            // Constructor
            GazeboRosSlbotController();
            // Destructor (has to be virtual)
            virtual ~GazeboRosSlbotController();
        protected:
            // Load function
            virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);
            // Update the child
            virtual void UpdateChild();
    };
}

#endif  // #ifndef GAZEBO_ROS_SLBOT_CONTROLLER
