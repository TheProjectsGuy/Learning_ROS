#!/usr/bin/env python3

"""
    Main objective of this program is to do the following:
        1. Demonstrates accessing messages that have array types
        2. Demonstrates creating a service server
"""

import rospy
import sys

# Import the module for using the service
"""
    This is developed after building the service files
    
    Local location: devel/lib/python3/dist-packages 
"""
from py_basic_nodes.srv import AddAllFloat64Numbers_py
from py_basic_nodes.srv import AddAllFloat64Numbers_pyRequest, AddAllFloat64Numbers_pyResponse


def add_numbers(req: AddAllFloat64Numbers_pyRequest):
    resp = AddAllFloat64Numbers_pyResponse()
    resp.sum = 0
    # Add all numbers
    for i in req.data:
        resp.sum += i
    rospy.loginfo("Received {0} numbers to add, result is {1}".format(len(req.data), resp.sum))
    return resp


def main():
    # Initialize node
    rospy.init_node("simple_py_service_server", argv=sys.argv, anonymous=False)

    # Create a service server
    """
        This tells the master that this node advertises a service (it can serve it, and therefore be a "server").
        The first argument is the service name
        The second argument is the service type (class)
        The third argument is the service server function. It is a callback function.
        
        Link: http://docs.ros.org/en/melodic/api/rospy/html/rospy.impl.tcpros_service.Service-class.html
    """
    srv_server = rospy.Service("{0}/add_numbers".format(rospy.get_name()), AddAllFloat64Numbers_py, add_numbers)

    # Info message
    rospy.loginfo("Service {0}/add_numbers has been started".format(rospy.get_name()))
    # Set the node to spin
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSException as e:
        rospy.logfatal("ROS error occurred: {0}".format(e))
