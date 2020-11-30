from math import sqrt, log, acos
import matplotlib.pyplot as plt
from track import load_track
from vehicles import PointCar
from path_finding import find_valid_trajectory
import numpy as np
from distance_viterbi import dynamics
import datetime
from Floyd import build_matrix


def dijstra_O2(start, mgraph):
    passed = [start]
    nopass = [x for x in range(mgraph.shape[0]) if x != start]
    dis = mgraph[start]

    while len(nopass):
        idx = nopass[0]
        for i in nopass:
            if dis[i] < dis[idx]: idx = i

        nopass.remove(idx)
        passed.append(idx)

        for i in nopass:
            if dis[idx] + mgraph[idx][i] < dis[i]: dis[i] = dis[idx] + mgraph[idx][i]
    return dis


if __name__ == "__main__":
    IMG_PATH = "./tracks/loop.png"
    track = load_track(IMG_PATH)

    a = datetime.datetime.now()
    # Set to a valid point in trajectory
    car = PointCar(150, 200)
    trellis = find_valid_trajectory(car, track)
    matrix = build_matrix(trellis)
    dijstra_m = dijstra_O2(0, matrix)
    for item in dijstra_m[-10:]:
        print(item)

    b = datetime.datetime.now()
    print(b-a)