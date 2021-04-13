/**
 * @file simple_library.h
 * @author Avneesh Mishra (123avneesh@gmail.com)
 * @brief 
 *      A simple C++ library to demonstrate how C++ libraries can be made and maintained in ROS
 * @version 0.1
 * @date 2021-04-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// Header guiards to prevent including the contents more than once
// Reference: https://en.wikipedia.org/wiki/Include_guard
#ifndef BASIC_CPP_LIBS_SIMPLE_LIBRARY_H
#define BASIC_CPP_LIBS_SIMPLE_LIBRARY_H

// Include basic header files (for this library)
#include<string>

/**
 * @brief Returns a greeting string for the passed `name`
 * 
 * @param name : std::string
 *      The name of the person to greet
 * @return std::string 
 *      A string prefixed with "Hello, " and followed by the `name`
 */
std::string say_hello(std::string name);

#endif
