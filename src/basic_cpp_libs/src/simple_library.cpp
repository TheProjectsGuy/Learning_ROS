/**
 * @file simple_library.cpp
 * @author Avneesh Mishra (123avneesh@gmail.com)
 * @brief 
 *      A simple C++ library to demonstrate how C++ libraries can be made and maintained in ROS
 * @version 0.1
 * @date 2021-04-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// Include the header file
/*
 * Include the library header files. The functions declared there will be defined in this file
 * 
 * Location: src/basic_cpp_libs/include
 */
#include "basic_cpp_libs/simple_library.h"

// Namespace
using namespace std;

// Implementation of say_hello
string say_hello(string name) {
    return "Hello, " + name;
}
