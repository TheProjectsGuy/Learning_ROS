#!/usr/bin/env python3

"""
    This program demonstrates the following
        1. Creating an action client
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


# A class to handle all communications with the action server
class ActionClientHandle:
    def __init__(self, server_name):
        self.action_name = server_name
        # Create an object for the action client
        """
            Just as a service client (proxy) is used to call a service server, an action client object is used
            to call an action server
            First argument is action name, second is the action class (notice "Action" suffix)

            Link: http://docs.ros.org/en/latest/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html
        """
        self.action_client_obj = actionlib.SimpleActionClient(self.action_name, CountNumbers_pyAction)

    def send_goal(self, goal_msg: CountNumbers_pyGoal):
        # Wait for the server and then send goal
        if self.wait_for_action_server():
            # Send the goal
            """
                Used to send a goal to the action server
                First argument is the goal which will be sent, the second one is the feedback funciton that needs to be 
                called with the feedback message (when one is received)
                
                Link: http://docs.ros.org/en/latest/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html#a53ec4bfc7f01a415c6d960b47f73da4e
            """
            self.action_client_obj.send_goal(goal=goal_msg, feedback_cb=self.callback_feedback)
            rospy.loginfo("Waiting for the response")
            timeout_duration_ms = goal_msg.target_number * goal_msg.del_ms
            # Wait for result (with timeout)
            """
                It's good to keep a buffer time (for waiting). 1s is a good buffer, but it depends on the application 
                
                Link: http://docs.ros.org/en/latest/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html#aa898e3966303511fa9a322e54254ecad
            """
            wait_res = self.action_client_obj.wait_for_result(
                timeout=rospy.Duration(secs=(timeout_duration_ms/1000)+1)
            )
            if not wait_res:
                # Timeout occurred
                raise rospy.ROSException("Timeout occurred in the response from action server")
            # If a success state has been returned, the proceed further
            """
                Link: http://docs.ros.org/en/latest/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html#a1496dbc011f48451f4ea98e1ad2f8cd9
            """
            if self.action_client_obj.get_state() != actionlib.GoalStatus.SUCCEEDED:
                raise rospy.ROSException("Goal status ({stat}) not SUCCEEDED".format(
                    stat=self.action_client_obj.get_state()
                ))
            # Get the response
            """
                Get the result, not that everything is verified to have happened correctly
                Link: http://docs.ros.org/en/latest/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html#ad3813394674ee680c997fa0279aa6c34
            """
            result_obj = self.action_client_obj.get_result()    # Type is CountNumbers_pyResult
            rospy.loginfo("Response received: {res}".format(res=result_obj.res_note))
        else:
            raise rospy.ROSException("Action (name: {name}) not found".format(
                name=self.action_name
            ))
            pass
        pass

    # A function called when a feedback message is received
    def callback_feedback(self, fb_msg: CountNumbers_pyFeedback):
        rospy.loginfo("Feedback received: {fbcon}".format(
            fbcon=fb_msg.curr_number
        ))  # Print the message contents

    def wait_for_action_server(self, timeout_val=rospy.Duration(5)):
        # Wait till the server is available
        """
            Used to wait until a timeout occurs

            Link: http://docs.ros.org/en/latest/api/actionlib/html/classactionlib_1_1simple__action__client_1_1SimpleActionClient.html#afd4d2c147e2ddc59af9c6570b72151fc
        """
        ret_val = self.action_client_obj.wait_for_server(timeout=timeout_val)
        return ret_val


# Main function
def main():
    # Initialize the node
    rospy.init_node("simple_py_action_client", argv=sys.argv)
    # Create an object of the class
    action_client_obj = ActionClientHandle("simple_py_action_server/count_numbers")
    # Send a goal
    goal_msg = CountNumbers_pyGoal()
    goal_msg.target_number = int(sys.argv[1]) if len(sys.argv) > 1 else 5
    goal_msg.del_ms = int(sys.argv[2]) if len(sys.argv) > 2 else 500
    action_client_obj.send_goal(goal_msg)
    rospy.loginfo("Program ended")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSException as e:
        rospy.logfatal("Fatal exception occurred: {0}".format(e))
