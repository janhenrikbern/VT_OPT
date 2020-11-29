from math import sqrt, log

def distance_score(node, x, y):
    if node.vehicle.can_reach_location((x,y)) is False:
        return None
    dx, dy = node.vehicle.get_distance_components((x, y))
    dist = sqrt(dx**2 + dy**2)
    print(f"distance to next location: {dist}")
    score = node.val + 1 / (dist + 1)
    return score

def time_score(node, x, y):
    if node.vehicle.can_reach_location((x,y)) is False:
        return None
    dt = node.vehicle.get_time((x,y))
    print(dt)
    score = node.val + 1 / log(1.0 + dt)
    return score