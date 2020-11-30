from functools import wraps
from math import sqrt, log


def steering_constraint(f):
    @wraps(f)
    def inner(node, x, y):
        if node.vehicle.can_reach_location((x,y)) is False:
            return None
        
        return f(node, x, y)
    
    return inner

@steering_constraint
def distance_score(node, x, y):
    dx, dy = node.vehicle.get_distance_components((x, y))
    dist = sqrt(dx**2 + dy**2)
    # score = node.val + 1.0 / (dist + 1.0)
    score = node.val + dist
    return score

@steering_constraint
def time_score(node, x, y):
    dt = node.vehicle.get_time((x,y))
    # score = node.val + 1.0 / (dt + 1.0)
    score = node.val + dt
    return score

def centerline_score(node, x, y):
    dx, dy = node.vehicle.get_distance_components((x, y))
    dist = sqrt(dx**2 + dy**2)
    score = node.val + 1.0 / (dist + 1.0)
    return score