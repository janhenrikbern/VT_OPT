import numpy as np
import metrics
from vehicles.point_car import PointCar
from path_finding import find_valid_trajectory
from track import load_track
import argparse 
import matplotlib.pyplot as plt


# Code in this file is based on the racing line optimization algorithm presented 
# in the post http://phungdinhthang.com/2016/12/16/calculate-racing-lines-automatically/?i=1
#

parser = argparse.ArgumentParser()
parser.add_argument(
    "--plot", default=False, help="plot trajectories"
)
a = parser.parse_args()

def distance(x1: np.ndarray, x2: np.ndarray) -> float:
    return np.sqrt(np.sum((x1 - x2)**2))

def segmentValue(prev, cur, nxt, alpha, beta):
    v1 = cur - prev
    v2 = nxt - cur
    v1_mag = distance(cur, prev)
    v2_mag = distance(nxt, cur)
    cos_angle  = np.dot(v1, v2) / (v1_mag * v2_mag)

    return beta * cos_angle * (v1_mag + v2_mag) - alpha * (v1_mag + v2_mag)

class RouteValue:
    def __init__(self, val, idx, coor):
        self.nxt = idx
        self.val = val
        self.coor = coor


    def __str__(self) -> str:
        return f"{self.val}\n"

def buildRouteSegments(parameterized_track, alpha, beta):
    n_states = len(parameterized_track[0])
    RVM = []
    # Build RVM from finish line to start line
    reversed_trellis = list(reversed(parameterized_track))
    for i, site in enumerate(reversed_trellis):
        cur_line = []
        for j, cur in enumerate(site):
            if i == 0:
                prev_line = []
                for idx in range(n_states):
                    # value beyond the finish is 0.0
                    prev_line.append(RouteValue(0.0, idx, cur))
                cur_line.append(prev_line)
            else:
                prev_line = []
                idx = i + 1 if i + 1 < len(reversed_trellis) else 0
                for prev in reversed_trellis[idx]:
                    best_val = -np.Infinity
                    best_idx = None
                    for l, nxt in enumerate(reversed_trellis[i-1]):
                        seg_val = 0.0 # set segment value to 0 if start line
                        if i + 1 < len(reversed_trellis):
                            seg_val = segmentValue(prev, cur, nxt, alpha, beta)
                        val = seg_val + RVM[i-1][l][j].val
                        if val > best_val:
                            best_val = val
                            best_idx = l
                    prev_line.append(RouteValue(best_val, best_idx, cur))
                cur_line.append(prev_line)
        RVM.append(cur_line)

    return list(reversed(RVM))

def get_best_path(RVM, states):
    dists = []
    paths  = []
    for idx in range(states):
        path = []
        nxt = idx
        cur = idx
        for line in RVM:
            tmp = line[nxt][cur]
            cur = nxt
            nxt = tmp.nxt
            path.append(tmp.coor)
        paths.append(path)
        x = [i[0] for i in path]
        y = [i[1] for i in path]
        dist = metrics.summed_distance(x, y)
        dists.append(dist)

    return paths[np.argmin(dists)]

def run(parameterized_track, alpha, beta):
    # compute all possible waypoint values
    RVM = buildRouteSegments(parameterized_track, alpha, beta)

    # collect best path
    path = get_best_path(RVM, len(parameterized_track[0]))
    return path

def run_gridsearch(parameterized_track, n_segments=10):
    for i in np.linspace(0.3, 0.7, num = n_segments+1, endpoint=True , dtype=float):
        print(i)
        path = run(parameterized_track, alpha=1.0 - i, beta=i)
        x = [i[0] for i in path]
        y = [i[1] for i in path]
        print(metrics.summed_distance(x, y))

        car = PointCar(*path[0])
        car.theta = car.heading(path[1])
        for coor in path[1:]:
            car.update(coor, is_coor=True)

        print(car.travel_time)

if __name__ == "__main__":
    IMG_PATH = "./tracks/loop.png"
    track = load_track(IMG_PATH)
    # Set to a valid point in trajectory
    car = PointCar(150, 200)
    STATES = 30 # a waypoint for every 0.5 of the track width
    trellis = find_valid_trajectory(car, track, states=STATES)

    run_gridsearch(trellis, n_segments=10)

    # path = run(trellis, alpha=.9, beta=0.1)
    # x = [i[0] for i in path]
    # y = [i[1] for i in path]
    # print(metrics.summed_distance(x, y))

    # car = PointCar(*path[0])
    # car.theta = car.heading(path[1])
    # for coor in path[1:]:
    #     car.update(coor, is_coor=True)

    # print(car.travel_time)


    if a.plot:
        fig, ax = plt.subplots()
        ax.imshow(track)
        plt.xlabel("Meters")
        plt.ylabel("Meters")
        ax.fill(x, y, facecolor='none', edgecolor='black', linestyle="-.", label="Centerline")
        plt.legend(loc=4)
        plt.show()