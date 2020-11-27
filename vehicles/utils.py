from math import (
    sin, 
    cos, 
    sqrt,
    acos,
    asin
)

def hypotenuse(*x_vec):
    acc = 0.0
    for x in x_vec:
        acc += x**2
    return sqrt(acc)