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

    
def run_viterbi(parameterized_track):
    return viterbi.additive_viterbi(parameterized_track)


def run_shooting(parameterized_track):
    return optimal.run(parameterized_track, 1.0, 0.0)


def min_distance():
    IMG_PATH = "./tracks/loop.png"
    track = load_track(IMG_PATH)

    # Set to a valid point in trajectory
    starting_position = (150., 200.)
    car = PointCar(*starting_position)
    n_loops = 1
    trellis = path_finding.find_valid_trajectory(car, track, loops=n_loops, states=30)
    split_idx = (len(trellis) // n_loops) + 1 if n_loops > 1 else 0

    path_v = run_viterbi(trellis)
    print_stats(path_v[split_idx:])
    path_s = run_shooting(trellis)
    print_stats(path_s[split_idx:])
    


if __name__ == "__main__":

    min_distance()







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