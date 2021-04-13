"""
    The factorial function
"""

def factorial(num):
    """
        Returns the factorial of a passed number

        Parameters:
        - num: int
            A number whose factorial is to be calculated
        Returns
        - res: int
            The result, which is num!
    """
    res = 1
    for n in range(2, num + 1):
        res *= n
    return res
