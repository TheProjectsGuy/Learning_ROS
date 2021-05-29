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
// Service to set the PID gains
/*
 * The control_toolbox/SetPidGains rossrv is used to
 * set the PID gains of a controller [1]. The wiki 
 * page can be found here [2]. This plugin will host
 * a service server for setting PID gains.
 * 
 * [1]: http://docs.ros.org/en/lunar/api/control_toolbox/html/srv/SetPidGains.html
 * [2]: https://wiki.ros.org/control_toolbox
 */
#include <control_toolbox/SetPidGains.h>
// Subscribing to std_msgs/Float64
#include <std_msgs/Float64.h>
// Header file for creating a custom callback queue
/*
 * A CallbackQueue [1] is a queue used to handle ROS 
 * callbacks (for things like subscribers). It is
 * recommended for plugins to not use the main ROS
 * callback queue for their subscriber callbacks. So
 * this plugin simply makes and uses its own.
 * 
 * [1]: http://docs.ros.org/en/latest/api/roscpp/html/classros_1_1CallbackQueue.html
 */
#include <ros/callback_queue.h>


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
            bool is_alive;  // True if running, false if dead
            // Connected to the Gazebo model and world
            physics::WorldPtr world;    // Simulation World
            physics::ModelPtr parent;   // The model XML tree
            physics::JointPtr joint;    // The joint that is to be controlled
            event::ConnectionPtr update_connection; // Update the connection
            // ROS variables
            ros::NodeHandle* _nh;   // NodeHandler
            ros::Publisher joint_angle_publisher_;  // Publisher for joint angles
            ros::Subscriber new_joint_pos_subscriber_;  // Subscriber for new joint targets
            ros::ServiceServer set_pid_gains_server_;   // Service server to set PID gains
            // Variables for this node
            std::string robot_namespace; // Namespace for node handler
            struct {    // Parameters for this plugin (wrapped in one object)
                std::string joint_name; // Joint name
                std::string js_topic;   // JointState topic name (published by plugin)
                std::string jtarget_topic;  // Topic for joint target (subscribed by plugin)
                std::string cgains_srv; // Service name for setting controller gains
                double rate_hz; // Rate in Hz (for updating)
                double update_period_sec;   // Update period (in seconds)
                common::Time last_update_time;  // Time last update was done
                sensor_msgs::JointState js_msg; // JointState message
            } params;
            struct {    // Controller (for controlling torque)
                double kp;  // P part (spring)
                double kd;  // D part (damping)
                double desired_pos; // Desired position
                // A mutex to not edit this structure at the same time through multiple threads
                boost::mutex lock;
            } controller;
            void PublishJointAngle();   // Publish the Joint angle in sensor_msgs/JointState
            void ControllerLoop();  // Controller loop for controlling the joint
            // Separate callback queue for this plugin
            ros::CallbackQueue queue_;  // Queue to handle the callback
            boost::thread callback_queue_thread_;   // Thread for callbacks (just for this plugin)
            void QueueThread();
            // Subscriber: Set new target
            void SetNewTarget(const std_msgs::Float64::ConstPtr& npos); // New position callback
            bool SetController(control_toolbox::SetPidGainsRequest &req,
                control_toolbox::SetPidGainsResponse &res); // Set PID gains for the controller
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
