from math import sqrt, log, acos
import numpy as np
import matplotlib.pyplot as plt
from track import load_track
from vehicles import PointCar
from path_finding import find_valid_trajectory
from Node import Node

def distance(x1: np.ndarray, x2: np.ndarray) -> float:
    return np.sqrt(np.sum((x1 - x2)**2))


def segment_score(prev, cur, nxt, alpha, beta):
    prev = np.array(prev)
    cur = np.array(cur)
    nxt = np.array(nxt)

    v1 = cur - prev
    v2 = nxt - cur
    v1_mag = distance(cur, prev)
    v2_mag = distance(nxt, cur)
    cos_angle  = np.dot(v1, v2) / (v1_mag * v2_mag)

    return beta * cos_angle * (v1_mag + v2_mag) - alpha * (v1_mag + v2_mag)


def time_score(node, nxt):
    dt = node.vehicle.get_time(nxt)
    score = node.val - dt
    return score


def score(nodes, state, alpha, beta):
    best_node = None
    max_val = -np.Infinity
    for prev in nodes:
        val = prev.val + segment_score(prev.prev.get_location(), prev.get_location(), state, alpha, beta)
        # val = time_score(prev, state)
        # print(val)
        if val == None:
            continue
        elif val > max_val:
            best_node = prev
            max_val = val
    if best_node == None:
        print("Couldn't find a valid node to proceed to.")
        exit()

    # set best node
    cur = Node()
    cur.inherit(best_node)
    cur.update(state, max_val)
    return cur

def init_node(state_idx, cur, nxt):
    n = Node()
    n.state_idx = state_idx
    n.x, n.y = cur
    n.vehicle = PointCar(n.x, n.y)
    n.vehicle.theta = n.vehicle.heading(nxt)
    return n



def additive_viterbi(trellis, alpha=1.0, beta=0.0):
    """
    Implementation inspired by: 

    Mazurek, P. (2014). Line estimation using the Viterbi algorithm and 
    track-before-detect approach for line following mobile robots. 
    In 2014 19th International Conference on Methods and Models 
    in Automation and Robotics (MMAR) (pp. 788–793). 
    IEEE. https://doi.org/10.1109/MMAR.2014.6957456

    Arguments:
        trellis: n x m x 2 np.array where n is the number of segments that combined are a full loop of the track
    """

    # forward pass
    nodes = []
    tmp = []
    for i, col in enumerate(trellis):
        for j, state in enumerate(col):
            if i == 0:
                prev = init_node(j, trellis[-1][j], state)
                n = init_node(j, state, trellis[i][j])
                n.prev = prev
                nodes.append(n)
                tmp.append(n)
                continue
    
            tmp[j] = score(nodes, state, alpha, beta)

        nodes, tmp = tmp, nodes
    
    # backward pass
    best_node = min(nodes, key=lambda n: n.val)
    
    path = []
    cur = best_node
    n_nodes = 0 
    while cur is not None:
        path.append(cur.vehicle.location)
        cur = cur.prev
        n_nodes += 1 
    
    return np.array(list(reversed(path)))


if __name__ == "__main__":
    IMG_PATH = "./tracks/loop.png"
    track = load_track(IMG_PATH)

    # Set to a valid point in trajectory
    starting_position = (150., 200.)
    car = PointCar(*starting_position)

    fig, ax = plt.subplots()
    ax.imshow(track)
    plt.xlabel("Meters")
    plt.ylabel("Meters")
    # baseline_trellis = find_valid_trajectory(car, track, states=1)
    # baseline = additive_viterbi(baseline_trellis, starting_position, centerline_score)
    # ax.fill(baseline[:,0], baseline[:,1], facecolor='none', edgecolor='black', linestyle="-.", label="Centerline")

    n_loops = 2
    trellis = find_valid_trajectory(car, track, loops=n_loops, states=30)
    split_idx = (len(trellis) // n_loops) + 1 if n_loops > 1 else 0

    time = additive_viterbi(trellis)
    ax.fill(time[split_idx:,0], time[split_idx:,1], facecolor='none', edgecolor='red', linestyle="-", label="Time Objective")

    # distance = additive_viterbi(trellis, starting_position, distance_score)
    # ax.fill(distance[:split_idx,0], distance[:split_idx,1], facecolor='none', edgecolor='blue', linestyle="-", label="Distance Objective")
    plt.legend(loc=4)
    plt.show()

    ## Trellis Video plot
    # fig = plt.figure()
    # for states in trellis:
    #     plt.imshow(track)
    #     plt.xlabel("Meter")
    #     plt.ylabel("Meter")
    #     for x, y in states:
    #         plt.scatter(x, y)
            
    #     plt.pause(0.1)
    #     fig.clear()
    # plt.pause(.5)
    # plt.close()

    # Trajectory Video plot
    # fig = plt.figure()
    # for x, y in time:
    #     plt.imshow(track)
    #     # plt.title("Baseline: Centerline Trajectory")
    #     plt.title("Viterbi Trajectory Optimization w/ distance specific energy function")
    #     plt.xlabel("Meters")
    #     plt.ylabel("Meters")
    #     plt.scatter(x, y)
    #     plt.pause(0.1)
    #     fig.clear()
    # plt.pause(0.5)
    # plt.close()