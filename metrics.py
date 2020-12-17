import numpy as np
from math import (
    sqrt
)


def summed_distance(x: np.ndarray, y: np.ndarray) -> float:
    """
    Feed points for a single loop
    """
    total = 0.0
    prev_x, prev_y = x[0], y[0]
    for i in range(len(x)):
        if i + 1 == len(x):
            i = -1
        xi, yi = x[i+1], y[i+1]
        total += sqrt(
            (xi - prev_x)**2 + (yi - prev_y)**2
        )
        prev_x, prev_y = xi, yi

    return total

