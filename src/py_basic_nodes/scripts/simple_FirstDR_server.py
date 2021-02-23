#!/usr/bin/env python3

"""
    Main objective of this program is to do the following:
        1. Creating a dynamic reconfiguration server node that can use 'FirstDR'
"""

# Main imports
import rospy
import sys

# Import the module for using dynamic configuration server
"""
    This file is included for the Server class which will help create a server object

    Location (on disk): /opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure
    Doc: https://docs.ros.org/en/lunar/api/dynamic_reconfigure/html/dynamic_reconfigure.server.Server-class.html
"""
from dynamic_reconfigure.server import Server
# Import the module made
"""
    This was made after building the `.cfg` file

    Location (on disk): devel/lib/python3/dist-packages/py_basic_nodes (in workspace)
    Documentation (on parameters): devel/share/py_basic_nodes/docs/FirstDRConfig-usage.dox
"""
from py_basic_nodes.cfg import FirstDRConfig


# Define the callback function
"""
    This function is called once when setting up (default values in the config, level is 0xffffffff - all ones) and 
    every time a client makes a parameter modification request.

    Parameters:
        - config: The configuration object. Different paraemters are members of this object.
        - level: The level which is the OR of the levels of changed parameters
"""
def callback_function(config, level):
    option_string = ""  # String for parameter OP_sel
    if (config["OP_sel"] == 1):
        option_string = "Option 1"
    elif (config["OP_sel"] == 2):
        option_string = "Option 2"
    elif (config["OP_sel"] == 3):
        option_string = "Option 3"
    else:
        option_string = "Invalid selection"
    rospy.loginfo(
        "Reconfig request (level = {lvl})\n\
        \tuser_int = {user_int}\n\
        \tLevel testing parameters\n\
        \t\tusr_int1 = {usr_int1}\n\
        \t\tusr_int2 = {usr_int2}\n\
        \t\tusr_int2 = {usr_int3}\n\
        \tuser_bool = {user_bool}\n\
        \tuser_str = {user_str}\n\
        \tOP_sel = {op_str} ({OP_sel})".format(
            lvl= level,
            user_int = config["user_int"],
            usr_int1 = config["usr_int1"],
            usr_int2 = config["usr_int2"],
            usr_int3 = config["usr_int3"],
            user_bool = config["user_bool"],
            user_str = config["user_str"],
            op_str = option_string, OP_sel = config["OP_sel"]
        )
    )
    # Return the configurations
    return config


# Main function
def main():
    # Initialize the node
    rospy.init_node("simple_py_firstdr_server", argv=sys.argv)
    
    # Create a server object
    """
        This is an object of the Server class in dynamic_reconfig
        It is like a subscriber, in fact it subscribes to a topic called
            - NODE_NAME/parameter_descriptions (msg: dynamic_reconfigure/ConfigDescription)
                - Descriptions of all the parameters in the parameter server
            - NODE_NAME/parameter_updates (msg: dynamic_reconfigure/Config)
                - The updated values of the parameters
        
        Client nodes are expected to publish to these topics (using client interface, like in services 
        and actions)
        The parameters are
            - Type of Dynamic Reconfiguration (pass the class)
            - Callback function to handle changes received
    """
    srv = Server(FirstDRConfig, callback=callback_function)

    # Set the node into spinning
    rospy.loginfo("Spinning Node")
    rospy.spin()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException as e:
        rospy.logfatal("Fatal ROS error occurred: ", e)
