/**
 * @file real_number.tpp
 * @author Avneesh Mishra (123avneesh@gmail.com)
 * @brief 
 *      This contains definitions for the template class ReamNumber
 *      The aim is to have all template definitions at one place (with declarations)
 *      Source file at: include/basic_math/algebra/real_number.hpp
 *      This library is made for demonstrating a complex library called "basic_math" inside a ROS package
 * @version 0.1
 * @date 2021-04-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */


// Include the header file
#include "basic_math/algebra/real_number.hpp"
// Other header files for this code
#include <iostream>

// Use the namespace (to avoid using :: everywhere)
using namespace basic_math;

/*
 * ==== Constructors ====
 */
// Constructor definition
template<class T>
basic_math::RealNumber<T>::RealNumber(T value) {
    this->data = value; // Assign the data
}

/*
 * ==== Public member functions
 */

// Conversion to string
template<class T>
std::string RealNumber<T>::to_string() {
    return "Real: " + std::to_string(this->data);
}

/*
 * ==== Operators ====
 */

// Addition operator
template<class T>
RealNumber<T> RealNumber<T>::operator+(const RealNumber<T>& other) {
    return RealNumber<T>(this->data + other.data);
}

// Subtraction operator
template<class T>
RealNumber<T> RealNumber<T>::operator-(const RealNumber<T>& other) {
    return RealNumber<T>(this->data - other.data);
}

// Multiplication operator
template<class T>
RealNumber<T> RealNumber<T>::operator*(const RealNumber<T>& other) {
    return RealNumber<T>(this->data * other.data);
}

// Division operator
template<class T>
RealNumber<T> RealNumber<T>::operator/(const RealNumber<T>& other) {
    return RealNumber<T>(this->data / other.data);
}
