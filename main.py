import matplotlib.pyplot as plt
import numpy as np

import argparse
from track import load_track
from vehicles import PointCar
import viterbi
import path_finding
from scoring_functions import (
    time_score,
    distance_score,
    centerline_score
)
import metrics
import optimal

parser = argparse.ArgumentParser()
parser.add_argument(
    "--plot", default=False, help="plot trajectories"
)
a = parser.parse_args()


def get_time(path):
    car = PointCar(*path[0])
    car.theta = car.heading(path[1])
    for coor in path[1:]:
        car.update(coor, is_coor=True)

    return car.travel_time


def get_distance(path):
    x = [i[0] for i in path]
    y = [i[1] for i in path]

    return metrics.summed_distance(x, y)


def print_stats(path):
    print(f"{get_distance(path)}, {get_time(path)}")


def min_distance():
    IMG_PATH = "./tracks/loop.png"
    track = load_track(IMG_PATH)

    # Set to a valid point in trajectory
    starting_position = (150., 200.)
    car = PointCar(*starting_position)
    n_loops = 1
    trellis = path_finding.find_valid_trajectory(car, track, loops=n_loops, states=31)
    split_idx = (len(trellis) // n_loops) + 1 if n_loops > 1 else 0

    c_idx = len(trellis[0])//2
    in_idx = 0
    centerline = [t[c_idx] for t in trellis]
    print_stats(centerline)
    inner_contour = [t[in_idx] for t in trellis]
    print_stats(inner_contour)

    path_v = viterbi.additive_viterbi(trellis, alpha=1.0, beta=0.0)
    print_stats(path_v[split_idx:2*split_idx])
    path_s = optimal.run(trellis, alpha=1.0, beta=0.0)
    print_stats(path_s[split_idx:2*split_idx])

    
    xv = [i[0] for i in path_v[split_idx:2*split_idx]]
    yv = [i[1] for i in path_v[split_idx:2*split_idx]]
    xs = [i[0] for i in path_s[split_idx:2*split_idx]]
    ys = [i[1] for i in path_s[split_idx:2*split_idx]]

    if a.plot:
        fig, ax = plt.subplots()
        ax.imshow(track)
        plt.xlabel("Meters")
        plt.ylabel("Meters")
        ax.fill(xv, yv, facecolor='none', edgecolor='green', linestyle="-", label="Distance Objective")


        fig, ax = plt.subplots()
        ax.imshow(track)
        plt.xlabel("Meters")
        plt.ylabel("Meters")
        ax.fill(xs, ys, facecolor='none', edgecolor='blue', linestyle="-", label="Time Objective")
        plt.show()
    

def min_time():
    IMG_PATH = "./tracks/loop.png"
    track = load_track(IMG_PATH)

    # Set to a valid point in trajectory
    starting_position = (150., 200.)
    car = PointCar(*starting_position)
    n_loops = 3
    trellis = path_finding.find_valid_trajectory(car, track, loops=n_loops, states=30)
    split_idx = (len(trellis) // n_loops) + 1 if n_loops > 1 else 0

    path_v = viterbi.additive_viterbi(trellis, alpha=1.0, beta=0.0)
    print_stats(path_v[split_idx:2*split_idx])
    path_s = optimal.run(trellis, alpha=0.5, beta=0.5)
    print_stats(path_s[split_idx:])

    
    xv = [i[0] for i in path_v[split_idx:2*split_idx]]
    yv = [i[1] for i in path_v[split_idx:2*split_idx]]
    xs = [i[0] for i in path_v[split_idx:2*split_idx]]
    ys = [i[1] for i in path_v[split_idx:2*split_idx]]

    if a.plot:
        fig1, ax1 = plt.subplots()
        ax1.imshow(track)
        plt.xlabel("Meters")
        plt.ylabel("Meters")
        ax1.fill(xv, yv, facecolor='none', edgecolor='green', linestyle="-")


        fig2, ax2 = plt.subplots()
        ax2.imshow(track)
        plt.xlabel("Meters")
        plt.ylabel("Meters")
        ax2.fill(xs, ys, facecolor='none', edgecolor='blue', linestyle="-")
        plt.show()

def state_tradeoff():
    IMG_PATH = "./tracks/loop.png"
    track = load_track(IMG_PATH)

    # Set to a valid point in trajectory
    starting_position = (150., 200.)
    car = PointCar(*starting_position)
    for s in [1, 5, 10, 20, 40, 80]:
        n_loops = 3
        trellis = path_finding.find_valid_trajectory(car, track, loops=n_loops, states=s)
        split_idx = (len(trellis) // n_loops) + 1 if n_loops > 1 else 0
        path_v = viterbi.additive_viterbi(trellis, alpha=1.0, beta=0.0)[split_idx:2*split_idx]
        print_stats(path_v)
        path_s = optimal.run(trellis, alpha=1.0, beta=0.0)[split_idx:2*split_idx]
        print_stats(path_s)

if __name__ == "__main__":

    # min_distance()
    # min_time()
    state_tradeoff()







    # IMG_PATH = "./tracks/loop.png"
    # track = load_track(IMG_PATH)

    # # Set to a valid point in trajectory
    # starting_position = (150., 200.)
    # car = PointCar(*starting_position)
    # baseline_trellis = path_finding.find_valid_trajectory(car, track, states=1)
    # baseline = viterbi.additive_viterbi(baseline_trellis, starting_position, centerline_score)
    # n_loops = 1
    # trellis = path_finding.find_valid_trajectory(car, track, loops=n_loops, states=10)

    # split_idx = (len(trellis) // n_loops) + 1 if n_loops > 1 else 0
    # time = viterbi.additive_viterbi(trellis, starting_position, time_score)
    # distance = viterbi.additive_viterbi(trellis, starting_position, distance_score)

    # for path in (baseline, time[split_idx:], distance[split_idx:]):
    #     print(get_stats(path, starting_position))

    # x = np.array([x[0] for x in distance])
    # y = np.array([x[1] for x in distance])
    # print(metrics.summed_distance(x, y))

    # if a.plot:
    #     fig, ax = plt.subplots()
    #     ax.imshow(track)
    #     plt.xlabel("Meters")
    #     plt.ylabel("Meters")
    #     ax.fill(baseline[:,0], baseline[:,1], facecolor='none', edgecolor='black', linestyle="-.", label="Centerline")
    #     ax.fill(time[split_idx:,0], time[split_idx:,1], facecolor='none', edgecolor='red', linestyle="-", label="Time Objective")
    #     ax.fill(distance[:split_idx,0], distance[:split_idx,1], facecolor='none', edgecolor='blue', linestyle="-", label="Distance Objective")
    #     plt.legend(loc=4)
    #     plt.show()