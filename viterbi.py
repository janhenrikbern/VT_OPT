from math import sqrt, log, acos
import numpy as np
import matplotlib.pyplot as plt
from track import load_track
from vehicles import PointCar
from path_finding import find_valid_trajectory
from Node import Node
from scoring_functions import (
    time_score,
    distance_score
)


def score(scoring_fn, nodes, state):
    x, y = state
    best_node = None
    max_val = -1
    for prev in nodes:
            val = scoring_fn(prev, x, y)
            if val == None:
                continue
            elif val > max_val:
                best_node = prev
                max_val = val
    if max_val < 0:
        print("Couldn't find a valid node to proceed to.")
        exit()
    # set best node
    cur = Node()
    cur.inherit(best_node)
    cur.update(state, max_val)
    return cur

def init_node(state_idx, starting_position, state):
    n = Node()
    n.state_idx = state_idx
    n.x, n.y = state
    x, y = starting_position
    dx = n.x - x
    dy = n.y - y
    n.vehicle = PointCar(n.x, n.y)
    n.vehicle.delta = acos(dx / sqrt(dy**2 + dx**2))
    return n



def additive_viterbi(trellis, starting_position, scoring_fn):
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
                n = init_node(j, starting_position, state)
                nodes.append(n)
                tmp.append(n)
                continue
    
            tmp[j] = score(scoring_fn, nodes, state)

        nodes, tmp = tmp, nodes
    
    # for n in nodes:
    #     print(n)
    
    # backward pass
    best_node = min(nodes, key=lambda n: n.val)
    print(best_node)
    
    path = []
    cur = best_node
    n_nodes = 0 
    while cur is not None:
        path.append((cur.x, cur.y))
        cur = cur.prev
        n_nodes += 1 
    print(f"Results of {scoring_fn.__name__} optimization: {best_node}")
    
    return get_full_path(starting_position, *list(reversed(path)))


def get_full_path(*coors):
    return np.array(coors).reshape(-1, 2)


if __name__ == "__main__":
    IMG_PATH = "./tracks/loop.png"
    track = load_track(IMG_PATH)

    # Set to a valid point in trajectory
    starting_position = (150., 200.)
    car = PointCar(*starting_position)

    fig, ax = plt.subplots()
    ax.imshow(track)
    baseline_trellis = find_valid_trajectory(car, track, states=1)
    baseline = additive_viterbi(baseline_trellis, starting_position, distance_score)
    ax.fill(baseline[:,0], baseline[:,1], facecolor='none', edgecolor='black', linestyle="-.", label="Centerline")

    trellis = find_valid_trajectory(car, track, nodes=399, states=20)
    time = additive_viterbi(trellis, starting_position, time_score)
    ax.fill(time[201:,0], time[201:,1], facecolor='none', edgecolor='red', linestyle="-", label="Time Objective")

    distance = additive_viterbi(trellis, starting_position, distance_score)
    ax.fill(distance[201:,0], distance[201:,1], facecolor='none', edgecolor='blue', linestyle="-", label="Distance Objective")
    # plt.legend()
    plt.show()

    ## Trajectory Video plot
    # fig = plt.figure()
    # for x, y in best_path:
    #     plt.imshow(track)
    #     # plt.title("Baseline: Centerline Trajectory")
    #     plt.title("Viterbi Trajectory Optimization w/ distance specific energy function")
    #     plt.xlabel("Unit distance")
    #     plt.ylabel("Unit distance")
    #     plt.scatter(x, y)
    #     plt.pause(0.1)
    #     fig.clear()
    # plt.pause(1.0)
    # plt.close()