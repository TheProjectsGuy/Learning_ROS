/**
 * @file basic_math_node.cpp
 * @author Avneesh Mishra (123avneesh@gmail.com)
 * @brief 
 *      A basic C++ node written to demonstrate the use of `basic_math` library in this package
 * @version 0.1
 * @date 2021-04-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// Basic header files
#include <ros/ros.h>

// Include the library
#include "basic_math/basic_math.hpp"

// Use the std namespace
using namespace std;


int main(int argc, char **argv)
{
    // Initialize the node
    ros::init(argc, argv, "basic_math_node");
    
    // Use the factorial function
    long int fact_val = basic_math::factorial(5);
    ROS_INFO_STREAM("Factorial of 5 is: " << fact_val);

    // Use RealNumbers
    basic_math::RealNumber<double> num1(10);
    basic_math::RealNumber<double> num2(15.5);
    ROS_INFO_STREAM("Number 1 = " << num1 << " and Number 2 = " << num2);
    ROS_INFO_STREAM("Value for sum: " << (num1 + num2).to_string());
    ROS_INFO_STREAM("Value for subtraction: " << (num1 - num2).to_string());
    ROS_INFO_STREAM("Value for multiplication: " << (num1 * num2).to_string());
    ROS_INFO_STREAM("Value for division: " << (num1 / num2).to_string());

    // Exit
    return 0;
}
