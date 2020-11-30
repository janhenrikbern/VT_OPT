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

parser = argparse.ArgumentParser()
parser.add_argument(
    "--plot", default=False, help="plot trajectories"
)
a = parser.parse_args()


def get_stats(path, starting_position):
    vehicle = PointCar(*starting_position)
    for coor in path:
        vehicle.update(coor, is_coor=True)
    return vehicle.travel_dist, vehicle.travel_time

if __name__ == "__main__":
    IMG_PATH = "./tracks/loop.png"
    track = load_track(IMG_PATH)

    # Set to a valid point in trajectory
    starting_position = (150., 200.)
    car = PointCar(*starting_position)
    baseline_trellis = path_finding.find_valid_trajectory(car, track, states=1)
    baseline = viterbi.additive_viterbi(baseline_trellis, starting_position, centerline_score)
    n_loops = 2
    trellis = path_finding.find_valid_trajectory(car, track, loops=n_loops, states=20)
    split_idx = (len(trellis) // n_loops) + 1 if n_loops > 1 else 0
    time = viterbi.additive_viterbi(trellis, starting_position, time_score)
    distance = viterbi.additive_viterbi(trellis, starting_position, distance_score)

    for path in (baseline, time[split_idx:], distance[split_idx:]):
        print(get_stats(path, starting_position))


    if a.plot:
        fig, ax = plt.subplots()
        ax.imshow(track)
        plt.xlabel("Meters")
        plt.ylabel("Meters")
        ax.fill(baseline[:,0], baseline[:,1], facecolor='none', edgecolor='black', linestyle="-.", label="Centerline")
        ax.fill(time[split_idx:,0], time[split_idx:,1], facecolor='none', edgecolor='red', linestyle="-", label="Time Objective")
        ax.fill(distance[:split_idx,0], distance[:split_idx,1], facecolor='none', edgecolor='blue', linestyle="-", label="Distance Objective")
        plt.legend(loc=4)
        plt.show()