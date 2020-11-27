from math import sqrt, log, acos


def dynamics(cur_coor, nxt_coor, dt=0.1):
    x1, y1 = cur_coor
    x2, y2 = nxt_coor
    dx = x1 - x2
    dy = y1 - y2
    dist = sqrt((dx)**2 + (dy)**2)
    print(f"distance to next location: {dist}")
    return dist, acos(dx/dist)

def distance_score(node, x, y):
    dx, heading = dynamics((node.x, node.y), (x, y))
    score = node.val + 1 / (dx + 1)
    return score

def time_score(node, x, y):
    dt = node.vehicle.get_time((x,y))
    if dt == None:
        return None, None, None
    print(dt)
    score = node.val + 1 / log(1.0 + dt)
    return score