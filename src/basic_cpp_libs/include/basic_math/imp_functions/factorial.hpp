/**
 * @file factorial.hpp
 * @author Avneesh Mishra (123avneesh@gmail.com)
 * @brief 
 *      Factorial function
 *      This library is made for demonstrating a complex library called "basic_math" inside a ROS package
 * @version 0.1
 * @date 2021-04-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// Header guard
#ifndef BASIC_MATH_IMP_FUNCTION_FACTORIAL_HPP
#define BASIC_MATH_IMP_FUNCTION_FACTORIAL_HPP


// It is a good idea to enclose user defined libraries into defined namespaces
namespace basic_math {

    /**
     * @brief Calculates the factorial of a number and returns the value
     * 
     * @param num : int
     *      Number whose factorial is to be calculated (must be positive)
     * @return long int 
     *      Factorial of `num` (= num!)
     */
    long int factorial(int num);

}

#endif
