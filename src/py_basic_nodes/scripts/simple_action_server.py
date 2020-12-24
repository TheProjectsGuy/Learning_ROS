#!/usr/bin/env python3

"""
    Main objective of this program is to do the following:
        1. Creating an action server
"""

import rospy
import sys

# Import the module for using the action messages
"""
    This is developed after the building of the `.action` file.
    Notice the "Action" suffix
    
    Local location: devel/lib/python3/dist-packages (of the workspace)
"""
from py_basic_nodes.msg import CountNumbers_pyAction
from py_basic_nodes.msg import CountNumbers_pyGoal, CountNumbers_pyFeedback, CountNumbers_pyResult

# Import the action library
"""
    This includes the underlying actionlib implementation to carry out the action communications on the ROS side
    
    Location: /opt/ros/noetic/lib/python3/dist-packages/
    Link (actionlib doc): https://docs.ros.org/en/api/actionlib/html/
"""
import actionlib


# Create a class to enclose everything associated with this action server
class ActionServerHandle:
    def __init__(self, server_name):
        self.action_name = server_name
        # Create an object for the Action Server
        """
            Just as a Service Server had an object that was used to bind the callback server to the service name, actions
            have a similar object to handle action callbacks
            First argument is action name, second is the action class (notice "Action" suffix)
            Third argument is the callback function
            Fourth argument is false (to set the server to start manually using a call to the start function)

            Link: http://docs.ros.org/en/latest/api/actionlib/html/classactionlib_1_1simple__action__server_1_1SimpleActionServer.html
        """
        self.action_server_obj = actionlib.SimpleActionServer(self.action_name, CountNumbers_pyAction,
                                                              execute_cb=self.actionCallbackFunction,
                                                              auto_start=False)

    def startActionServer(self):
        # Start the action server (to start processing callbacks)
        """
            This is used to start the action server object. Needs to be called explicitly.

            Link: http://docs.ros.org/en/latest/api/actionlib/html/classactionlib_1_1simple__action__server_1_1SimpleActionServer.html#ab1d26031978ae7a1a5e04a79ad63d3c1
        """
        self.action_server_obj.start()

    def actionCallbackFunction(self, goal_object: CountNumbers_pyGoal):
        # Carry out the steps to attain the goal
        rate_hdlr = rospy.Rate(1000 / goal_object.del_ms)
        target_count = goal_object.target_number
        rospy.loginfo("Received a goal ({dms} ms, up to {tar}), starting count".format(
                        dms=goal_object.del_ms,
                        tar=goal_object.target_number
        ))
        # Run the loop
        for i in range(1, target_count+1):
            rospy.loginfo("Count at {ival}".format(ival=i))
            # Send a feedback
            fb_obj = CountNumbers_pyFeedback()
            fb_obj.curr_number = i
            # Send feedback using publish_feedback function
            """
                Link: http://docs.ros.org/en/latest/api/actionlib/html/classactionlib_1_1simple__action__server_1_1SimpleActionServer.html#a4c5e2e6c8c55a18ab46699f2e6e93bc2
            """
            self.action_server_obj.publish_feedback(fb_obj)
            # Delay
            rate_hdlr.sleep()
        # Create a response message
        res_obj = CountNumbers_pyResult()
        res_obj.res_note = "Finished counting upto {tar}".format(tar=target_count)
        # Send result using set_succeeded function
        """
            Link: http://docs.ros.org/en/latest/api/actionlib/html/classactionlib_1_1simple__action__server_1_1SimpleActionServer.html#a401ac707896044cc76479a06b691842e
        """
        self.action_server_obj.set_succeeded(res_obj, "Success")


# Main function
def main():
    # Initialize the node
    rospy.init_node("simple_py_action_server", argv=sys.argv)
    # Create an object of that class
    action_obj = ActionServerHandle("{node_name}/count_numbers".format(
        node_name=rospy.get_name()
    ))
    action_obj.startActionServer()  # Start the server
    rospy.spin()    # Go into spin mode
    pass


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSException as e:
        rospy.logfatal("Fatal exception occurred: %s" % e)
