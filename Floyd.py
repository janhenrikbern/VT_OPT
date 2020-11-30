from math import sqrt, log, acos
import matplotlib.pyplot as plt
from track import load_track
from vehicles import PointCar
from path_finding import find_valid_trajectory
import numpy as np
from distance_viterbi import dynamics
import datetime


def build_matrix(trellis):
    visible = len(trellis)
    width = trellis[0].shape[0]

    trellis = np.array(trellis)
    nodes = trellis.reshape(visible*width, 2)
    matrix = np.ones([nodes.shape[0], nodes.shape[0]]) * 100000
    for i in range(visible*width):
        for j in range(visible*width):
            phase = i // width + 1
            if phase*width <= j < phase*width+width:
                dx, _ = dynamics(nodes[i], nodes[j], None)
                matrix[i][j] = dx
    return matrix


    # Floyd-Warshall算法的伪代码描述如下：
    # 1 let dist be a |V| × |V| array of minimum distances initialized to ∞ (infinity)
    # 2 for each vertex v
    # 3    dist[v][v] ← 0
    # 4 for each edge (u,v)
    # 5    dist[u][v] ← w(u,v)  // the weight of the edge (u,v)
    # 6 for k from 1 to |V|
    # 7    for i from 1 to |V|
    # 8       for j from 1 to |V|
    # 9          if dist[i][j] > dist[i][k] + dist[k][j]
    # 10             dist[i][j] ← dist[i][k] + dist[k][j]
    # 11         end if

def floyd(matrix):
    length = matrix.shape[0]
    for k in range(0, length):
        for i in range(0, length):
            for j in range(0, length):
                if matrix[i][j] > matrix[i][k] + matrix[k][j]:
                    matrix[i][j] = matrix[i][k] + matrix[k][j]

    return matrix


if __name__ == "__main__":
    IMG_PATH = "./tracks/loop.png"
    track = load_track(IMG_PATH)

    a = datetime.datetime.now()
    # Set to a valid point in trajectory
    car = PointCar(150, 200)
    trellis = find_valid_trajectory(car, track)
    matrix = build_matrix(trellis)
    floyd_m = floyd(matrix)
    for item in floyd_m[0:10, -1]:
        print(item)

    b = datetime.datetime.now()
    print(b-a)

    # best_path = []
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