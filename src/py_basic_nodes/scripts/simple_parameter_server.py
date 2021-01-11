#!/usr/bin/env python3

"""
    Main motive of this program is to do the following:
        1. Get the parameters on the parameter server
        2. Create parameters on the parameter server
        3. Deleting parameters on the parameter server
"""

# Import basic libraries
import rospy
import sys


# Main function
def main():
    # Initialize node
    rospy.init_node("simple_py_parameter_node", argv=sys.argv)

    # Retrieve a list of parameters running on the parameter server
    """
        This call returns the names as a list of strings
        
        Link (function): https://docs.ros.org/en/melodic/api/rospy/html/rospy-module.html#get_param_names
    """
    param_names = rospy.get_param_names()
    rospy.loginfo("Found the following parameters")
    for param_name in param_names:
        # Get the value of a parameter
        """
            The call returns the parameter value (the type is as on the parameter server)
            
            Link: https://docs.ros.org/en/melodic/api/rospy/html/rospy-module.html#get_param
        """
        param_value = rospy.get_param(param_name)
        rospy.loginfo("Parameter {pname}: '{pvalue}' (Type: {tp})".format(
            pname=param_name, pvalue=param_value, tp=type(param_value)
        ))

    # Get a specific parameter (here 'num_param')
    """
        It is better to search for a parameter (to know if it is existing) before trying to retrieve it.
        If you try to call get_param on a parameter that doesn't exist, then a 'KeyError' is thrown (you can therefore 
        alternatively use exception handling) 
        
        Link (has_param): https://docs.ros.org/en/melodic/api/rospy/html/rospy-module.html#has_param
    """
    param_name = "num_param"
    if rospy.has_param(param_name):
        param_value = rospy.get_param(param_name)
        rospy.loginfo("Paramater '{pname}' = '{pvalue}'".format(pname=param_name, pvalue=param_value))
    else:
        rospy.logwarn("Could not find parameter '{pname}'".format(pname=param_name))

    # Get a parameter that has child parameters (here 'parent_param')
    """
        Where there is a dictionary like structure (python 'dict'), a 'dict' (dictionary) is returned
    """
    param_name = "parent_param"
    try:
        param_value = rospy.get_param(param_name)
        rospy.loginfo("Parameter '{pname}' = {pvalue}".format(pname=param_name, pvalue=param_value))
    except KeyError as e:
        rospy.logwarn("Could not find parameter '{pname}'".format(pname=param_name))

    # Get parameter which is an array
    param_name = "list_param"
    try:
        param_value = rospy.get_param(param_name)
        rospy.loginfo("Parameter '{pname}' = '{pvalue}'".format(
            pname=param_name, pvalue="; ".join(param_value)
        ))
    except KeyError as e:
        rospy.logwarn("Could not find parameter '{pname}'".format(pname=param_name))

    # Create a parameter on the parameter server (custom_py_parameter)
    """
        Use the function set_param for this. The parameter value can be of any type (including `list` and `dict`).
        
        Link (function): https://docs.ros.org/en/melodic/api/rospy/html/rospy-module.html#set_param
    """
    param_name = "custom_py_parameter"          # Parameter name (key)
    param_value = "Custom Python Parameter"     # Parameter value
    rospy.set_param(param_name, param_value)
    rospy.loginfo("Created parameter '{pname}' to value '{pvalue}'".format(pname=param_name, pvalue=param_value))

    # Delete a parameter from the parameter server (junk_parameter)
    """
        Use the function delete_param for this. If the parameter being deleted does not exist, a 'KeyError' is raised
        
        Link (function): https://docs.ros.org/en/melodic/api/rospy/html/rospy-module.html#delete_param
    """
    param_name = "junk_parameter"
    try:
        rospy.delete_param(param_name)
        rospy.loginfo("Deleted parameter '{pname}'".format(pname=param_name))
    except KeyError as e:
        rospy.logwarn("Could not delete parameter '{pname}' as it doesn't exist: (Error {err})".format(
            pname=param_name, err=e
        ))


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSException as e:
        rospy.logfatal("Fatal ROS exception occurred: {0}".format(e))

"""
    Reference link: https://wiki.ros.org/rospy/Overview/Parameter%20Server
"""
