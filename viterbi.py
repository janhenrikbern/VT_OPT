from math import sqrt, log, acos
import matplotlib.pyplot as plt
from track import load_track
from vehicles import PointCar
from path_finding import find_valid_trajectory
from Node import Node
from scoring_functions import (
    time_score,
    distance_score
)


def score(nodes, state, scoring_fn=distance_score):
    x, y = state
    best_node = None
    max_val = -1
    for prev in nodes:
            val = scoring_fn(prev, x, y)
            if val == None:
                continue
            elif val > max_val:
                best_node = prev

    # set best node
    cur = Node().inherit(best_node)
    cur.update(state, max_val)
    return cur


def additive_viterbi(trellis):
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
                n = Node()
                n.x = state[0]
                n.y = state[1]
                n.vehicle = PointCar(n.x, n.y)
                x, y = trellis[i+1][j]
                dx = x - n.x
                dy = y - n.y
                n.vehicle.delta = acos(dx / sqrt(dy**2 + dx**2))
                nodes.append(n)
                tmp.append(n)
                continue
    
            tmp[j] = score(nodes, state)

        nodes, tmp = tmp, nodes
    
    # for n in nodes:
    #     print(n)
    
    # backward pass
    best_node = max(nodes, key=lambda n: n.val)
    print(best_node)
    
    path = []
    cur = best_node
    while cur is not None:
        path.append((cur.x, cur.y))
        cur = cur.prev
    
    return path


if __name__ == "__main__":
    IMG_PATH = "./tracks/loop.png"
    track = load_track(IMG_PATH)

    # Set to a valid point in trajectory
    car = PointCar(150, 200)

    trellis = find_valid_trajectory(car, track)
    best_path = additive_viterbi(trellis)

    fig = plt.figure()
    for x, y in best_path:
        plt.imshow(track)
        plt.title("Baseline: Centerline Trajectory")
        # plt.title("Viterbi Trajectory Optimization w/ distance specific energy function")
        plt.xlabel("Unit distance")
        plt.ylabel("Unit distance")
        plt.scatter(x, y)
        plt.pause(0.1)
        fig.clear()
    plt.pause(1.0)
    plt.close()