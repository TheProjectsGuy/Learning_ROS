/**
 * @file factorial.cpp
 * @author Avneesh Mishra (123avneesh@gmail.com)
 * @brief 
 *      Definition of the factorial function defined in include/basic_math/imp_functions/factorial.hpp
 *      This library is made for demonstrating a complex library called "basic_math" inside a ROS package
 * @version 0.1
 * @date 2021-04-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// Include the header file
#include "basic_math/imp_functions/factorial.hpp"

// Define the function
long int basic_math::factorial(int num) {
    // Start with 1
    long int ret_val = 1;
    for (int i = 2; i <= num; i++) {
        ret_val *= i;
    }
    return ret_val;
}
