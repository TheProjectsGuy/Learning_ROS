# A basic mathematics library
"""
    ## Basic Math

    A library for demonstrating some basic mathematical functionality. The main objective is to
    demonstrate constructing a complex python module in ROS.

    This library contains two modules
    1. algebra: Algebra module for real numbers
    2. functions: Basic functions used in maths
"""

# Import sub-modules in this package
"""
    This is so that the user doesn't have to import every submodule individually

    Reference: https://docs.python.org/3/tutorial/modules.html#intra-package-references
"""

# Algebra module
from . import algebra
# Basic functions
from . import imp_functions as functions
