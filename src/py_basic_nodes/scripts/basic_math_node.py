#!/usr/bin/env python3

"""
    Demonstrates how to use a complex module created in Python (for ROS packages)
"""

# Basic imports
import rospy
import sys

# Import the module
"""
    This essentially initializes the __init__.py file of the module
"""
import basic_math

# Main function
def main():
    # Initialize node
    rospy.init_node("basic_math_node", argv=sys.argv)

    # Using the algebra module
    num1 = basic_math.algebra.RealNumber(15)
    num2 = basic_math.algebra.real_number.RealNumber(20) # This can be done too
    rospy.loginfo("Result of the sum: {}".format(num1 + num2))
    rospy.loginfo("Result of division: {}".format(num1 / num2))
    
    # Use the imp_functions module
    n1 = basic_math.functions.fact(5)
    n2 = basic_math.functions.factorial.factorial(6)    # This can be done too
    rospy.loginfo("Factorial of 5 is: {}".format(n1))
    rospy.loginfo("Factorial of 6 is: {}".format(n2))


# Entry point
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSException as exc:
        rospy.logfatal("A fatal ROS exception has occurred (Msg: {msg})".format(msg=exc))
    except Exception as exc:
        rospy.logfatal("A non-ROS exception occurred (Msg: {msg})".format(msg=exc))
