from math import sqrt, log
import matplotlib.pyplot as plt
from track import load_track
from vehicles import PointCar
from path_finding import find_valid_trajectory


class Node:
    def __init__(self) -> None:
        self.prev = None
        self.val = 0
        self.state = -1
        self.x = None
        self.y = None
        self.vel = 0
        self.steps = 0
        self.summed_metric = 0.0
        self.heading = [0., 0.]

    def __str__(self) -> str:
        return f"Node value {self.val} with step cnt {self.steps} and metric value {self.summed_metric}"


def dynamics(cur_coor, nxt_coor, curr_vel, dt=0.1):
    x1, y1 = cur_coor
    x2, y2 = nxt_coor

    dx = sqrt( (x1 - x2)**2 + (y1 - y2)**2 )
    v2 = dx / dt

    # TODO: Add distance punishment if distance between nodes is too large 
    # (jumps more than 1 neighboring node or more then some threshold distance)


    # currently unused because distance is the better estimator
    # v1 = curr_vel
    # a = abs(v2 - v1) / dt
    return dx, v2

def distance_score(node, x, y, nxt_state):
    dx, cur_vel = dynamics((node.x, node.y), (x, y), node.vel)
    score = node.val + 1 / (dx + 1)
    return score, dx, cur_vel

def additive_viterbi(trellis):
    """
    Implementation inspired by: 

    Mazurek, P. (2014). Line estimation using the Viterbi algorithm and 
    track-before-detect approach for line following mobile robots. 
    In 2014 19th International Conference on Methods and Models 
    in Automation and Robotics (MMAR) (pp. 788–793). 
    IEEE. https://doi.org/10.1109/MMAR.2014.6957456

    Arguments:
        trellis: n x m x 2 np.array where n is the number of points along  
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
                n.state = j+1
                nodes.append(n)
                tmp.append(n)
                continue
            
            x, y = state
            # Wrap this into scoring function
            max_node = None
            max_val = -1
            best_v = None
            best_dm = None
            for prev in nodes:
                # if prev.state == 3: # Centerline trajectory for trellis of width 5
                if abs(prev.state - j) <= 1:
                    val, dm, v = distance_score(prev, x, y, j)
                    if val > max_val:
                        max_node = prev
                        max_val = val
                        best_v = v
                        best_dm = dm

            cur = Node()
            cur.val = max_val
            cur.vel = best_v
            cur.x = state[0]
            cur.y = state[1]
            cur.state = j+1
            cur.prev = max_node
            cur.summed_metric = max_node.summed_metric + best_dm
            ###### End scoring function
            tmp[j] = cur
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
    
    visible = 199
    width = 1
    trellis = find_valid_trajectory(car, track, visible, width)
    best_path = additive_viterbi(trellis)

    fig = plt.figure()
    for x, y in best_path:
        plt.imshow(track)
        plt.title("Baseline: Centerline Trajectory")
        # plt.title("Viterbi Trajectory Optimization w/ distance specific energy function")
        plt.xlabel("Unit distance")
        plt.ylabel("Unit distance")
        plt.scatter(x, y)
        plt.pause(0.016)
        fig.clear()
    plt.pause(1.0)
    plt.close()

    

    

    


