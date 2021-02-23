#!/usr/bin/env python3

"""
    Main objective of this program is to do the following:
        1. Creating a dynamic reconfiguration client node that can use 'FirstDR'
"""

# Main imports
import rospy
import sys

# Import the module for using dynamic configuration client
"""
    This file is included for the Client class which will help create a client object.
    This client will send commands to change paramters on the server.

    Location (on disk): /opt/ros/noetic/lib/python3/dist-packages/dynamic_reconfigure
    Doc: https://docs.ros.org/en/melodic/api/dynamic_reconfigure/html/dynamic_reconfigure.client.Client-class.html
"""
from dynamic_reconfigure.client import Client
# Import the module made
"""
    This was made after building the `.cfg` file

    Location (on disk): devel/lib/python3/dist-packages/py_basic_nodes (in workspace)
    Documentation (on parameters): devel/share/py_basic_nodes/docs/FirstDRConfig-usage.dox
"""
from py_basic_nodes.cfg import FirstDRConfig


# Configuration callback
"""
    It is not mandatory to implement this, but if you want to call a function every time the client sends new values, you can use this
    The callback accepts the new configurations as an argument ('config' here)
"""
def client_callback(config):
    # Just print out the entire new packet sent
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
        "Reconfig request sent\n\
        \tuser_int = {user_int}\n\
        \tLevel testing parameters\n\
        \t\tusr_int1 = {usr_int1}\n\
        \t\tusr_int2 = {usr_int2}\n\
        \t\tusr_int2 = {usr_int3}\n\
        \tuser_bool = {user_bool}\n\
        \tuser_str = {user_str}\n\
        \tOP_sel = {op_str} ({OP_sel})".format(
            user_int = config["user_int"],
            usr_int1 = config["usr_int1"],
            usr_int2 = config["usr_int2"],
            usr_int3 = config["usr_int3"],
            user_bool = config["user_bool"],
            user_str = config["user_str"],
            op_str = option_string, OP_sel = config["OP_sel"]
        )
    )


# Main function
def main():
    # Initialize the node
    rospy.init_node("simple_py_firstdr_client", argv=sys.argv)

    # Create a client object
    """
        This is an object that will be used to send parameter updates to the server.
        The parameters are
            - Name: Name of the node that is running the dynamic reconfiguration server
                - Passing 'simple_py_firstdr_server' means that a service by the name of '/simple_py_firstdr_server/set_parameters' must exist
                    This service is to update paraemters on the server
            - Timeout: Timeout to wait for parameters to be adjusted (in ms)
            - Callback: A function called everytime a message is sent
        
        Reference (constructor): https://docs.ros.org/en/melodic/api/dynamic_reconfigure/html/dynamic_reconfigure.client.Client-class.html#__init__
    """
    client_obj = Client("simple_py_firstdr_server", timeout=30, config_callback=client_callback)

    # Create a rate at which things are updated
    rate_hdlr = rospy.Rate(1)   # 1 Hz
    val = 50    # Some value that we'll update
    # Indefinitely publish values
    while not rospy.is_shutdown():
        # Update value
        val = (val + 1) % 100
        
        # Send configuration
        """
            Use the update_configuration function for this purpose. Pass it a dictionary of values that need to be changed
            Here, we're updating only usr_int1, usr_int2, and usr_int3 parameters
            Make sure that you have the FirstDRConfig-usage.docx file open (for reference on names and types)

            Location of FirstDRConfig-usage.docx: devel/share/py_basic_nodes/docs (in workspace)
            Doc: https://docs.ros.org/en/melodic/api/dynamic_reconfigure/html/dynamic_reconfigure.client.Client-class.html#update_configuration
        """
        client_obj.update_configuration({
            "usr_int1": val,
            "usr_int2": val,
            "usr_int3": val
        })
        rospy.loginfo("Reconfigure request sent for value {v}".format(v=val))

        # Delay
        rate_hdlr.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException as exc:
        rospy.logfatal("Fatal ROS exception occurred: ", exc)
