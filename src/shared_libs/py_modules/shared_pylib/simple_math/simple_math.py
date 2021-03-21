"""
    A wrapper for simple functions to demonstrate a python module in a ROS package
    The main file is in `simple_math.py`, so import using
    
    ```py
    from shared_pylib.simple_math import simple_math
    ```
"""

# Add
def add(a, b):
    """
        Adds two numbers

        ## Parameters

        1. a
        2. b

        ## Returns

        1. a + b
    """
    return a + b


# A random class
class RandomClass:
    """
        A Random Class made to demonstrate python classes

        ## Member functions

        - random_function: Return a string
    """
    def random_function(self):
        """
            A random function
        """
        return "A random function of RandomClass"
