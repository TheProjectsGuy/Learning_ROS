# Real number algebra
"""
    Algebra with real numbers
"""

class RealNumber:
    """
        A real number that you can add, subtract, multiply and divide.
        Most of the reference from: https://docs.python.org/3/reference/datamodel.html#emulating-numeric-types

        ## Constructor:
        Parameters:
        - number: float
            A real number
    """
    # Constructor
    def __init__(self, number = 0.0):
        self.number = number
    # Add function
    def __add__(self, another):
        return RealNumber(self.number + another.number)
    # Subtract function
    def __sub__(self, another):
        return RealNumber(self.number - another.number)
    # Multiply function
    def __mul__(self, another):
        return RealNumber(self.number * another.number)
    # Divide function
    def __truediv__(self, another):
        return RealNumber(self.number / another.number)
    # To string (for printing)
    def __str__(self):
        return "Real: " + str(self.number)
