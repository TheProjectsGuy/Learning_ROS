/**
 * @file real_number.hpp
 * @author Avneesh Mishra (123avneesh@gmail.com)
 * @brief 
 *      Real number algebra
 *      This library is made for demonstrating a complex library called "basic_math" inside a ROS package
 * @version 0.1
 * @date 2021-04-13
 * 
 * @copyright Copyright (c) 2021
 * 
 */

// Header guard
#ifndef BASIC_MATH_ALGEBRA_REAL_NUMBER_HPP
#define BASIC_MATH_ALGEBRA_REAL_NUMBER_HPP

// For iostream
#include <iostream>
// Include strings
#include <string>


// It is a good idea to enclose user defined libraries into defined namespaces
namespace basic_math {

    // Use template
    /*
     * A template allows generic datatypes (like int, double, float, some custom datatype) to be implemented
     * Reference: https://en.cppreference.com/w/cpp/language/templates
     */
    template<class T>
    class RealNumber {
        // Data for the type
        T data;

        /**
         * @brief Output stream
         *      Reference: https://en.cppreference.com/w/cpp/language/friend#Template_friend_operators
         * 
         * @param os : std::ostream&
         *      Output stream
         * @param obj : const ReamNumber&
         *      A number
         * @return std::ostream& 
         *      Output stream
         */
        friend std::ostream& operator<<(std::ostream& sout, const RealNumber& var) {
            return sout << var.data;
        }

    public:
        /**
         * @brief Construct a new RealNumber object (empty)
         * 
         */
        RealNumber() {
            this->data = 0;
        }

        /**
         * @brief Construct a new RealNumber object
         * 
         * @param value : <T>
         *      The data is of type <T>. Type must have operators +, -, *, / defined
         */
        RealNumber(T value);

        /**
         * @brief Converts the data to string
         * 
         * @return std::string 
         */
        std::string to_string();

        // Operator overloading functions
        // Reference: https://en.cppreference.com/w/cpp/language/operators

        /**
         * @brief Addition operator for RealNumber class
         * 
         * @param other 
         * @return RealNumber<T> 
         */
        RealNumber<T> operator+(const RealNumber<T>& other);

        /**
         * @brief Subtraction operator for RealNumber class
         * 
         * @param other 
         * @return RealNumber<T> 
         */
        RealNumber<T> operator-(const RealNumber<T>& other);

        /**
         * @brief Multiplication operator for RealNumber class
         * 
         * @param other 
         * @return RealNumber<T> 
         */
        RealNumber<T> operator*(const RealNumber<T>& other);

        /**
         * @brief Division operator for RealNumber class
         * 
         * @param other 
         * @return RealNumber<T> 
         */
        RealNumber<T> operator/(const RealNumber<T>& other);
    };
}

// To have modularity but also to get around template instantiation, distribute code this way
#include "real_number.ipp"

#endif  // #ifndef BASIC_MATH_ALGEBRA_REAL_NUMBER_HPP
