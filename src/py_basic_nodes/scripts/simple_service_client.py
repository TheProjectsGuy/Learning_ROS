#!/usr/bin/env python3

"""
    Main objective of this program is to do the following:
        1. Demonstrates creating a service client
"""

import rospy
import sys
# Import the service module
from py_basic_nodes.srv import AddAllFloat64Numbers_py
from py_basic_nodes.srv import AddAllFloat64Numbers_pyRequest, AddAllFloat64Numbers_pyResponse

def main():
    # Initialize the node
    rospy.init_node("simple_py_service_client", argv=sys.argv)

    # Wait for the service
    """
        It is good to check for the service already set up and ready to accept calls
        
        Link: http://docs.ros.org/en/melodic/api/rospy/html/rospy-module.html#wait_for_service
    """
    service_name = "/simple_py_service_server/add_numbers"
    rospy.wait_for_service(service_name)
    rospy.loginfo("Connected to service {0}".format(service_name))

    # Create service client
    """
        The client is created using a ServiceProxy. It will be used to call the service, similar to a publisher 
        object being used to publish a message.
        First argument is the service name
        Second argument is the service type (class)
                
        Link: http://docs.ros.org/en/melodic/api/rospy/html/rospy.impl.tcpros_service.ServiceProxy-class.html
    """
    srv_proxy = rospy.ServiceProxy(service_name, AddAllFloat64Numbers_py)

    # A request object
    req_obj = AddAllFloat64Numbers_pyRequest()
    # Fill in data from the arguments passed
    for i in range(1, len(sys.argv)):
        try:
            req_obj.data.append(float(sys.argv[i]))
        except ValueError:
            rospy.logwarn("Argument '{0}' couldn't be converted to float".format(sys.argv[i]))

    # Call the service
    """
        A call is made using the request object. It returns an object of the response type. Here, it is of type
        AddAllFloat64Numbers_pyResponse
        
        Link: http://docs.ros.org/en/melodic/api/rospy/html/rospy.impl.tcpros_service.ServiceProxy-class.html#call
    """
    resp_obj = AddAllFloat64Numbers_pyResponse(srv_proxy.call(req_obj))

    # Print it out
    rospy.loginfo("Sum is {0}".format(resp_obj.sum))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSException as e:
        rospy.logfatal("Fatal ROS exception: {}".format(e))
    pass
