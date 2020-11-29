from math import sqrt, log

def distance_score(node, x, y):
    if node.vehicle.can_reach_location((x,y)) is False:
        return None
    dx, dy = node.vehicle.get_distance_components((x, y))
    dist = sqrt(dx**2 + dy**2)
    score = node.val + 1.0 / (dist + 1.0)
    return score

def time_score(node, x, y):
    if node.vehicle.can_reach_location((x,y)) is False:
        return None
    dt = node.vehicle.get_time((x,y))
    score = node.val + 1 / (dt + 1.0)
    return score