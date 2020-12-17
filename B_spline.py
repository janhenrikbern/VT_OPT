import numpy as np
import scipy.interpolate as interpolate
import matplotlib.pyplot as plt
from track import load_track
from vehicles import PointCar
from path_finding import find_valid_trajectory
from distance_viterbi import dynamics
import datetime
import scipy.interpolate as si
import metrics


def return_better(candidates, idx, left, right, displacement):
    id_list = [i for i in range(len(candidates)) if abs(i-idx) <= 1]
    score = []
    for i in range(len(id_list)):
        v_left = np.array([left[0]-candidates[id_list[i]][0], left[1]-candidates[id_list[i]][1]])
        norm_left = v_left/np.linalg.norm(v_left)
        v_right = np.array([right[0]-candidates[id_list[i]][0], right[1]-candidates[id_list[i]][1]])
        norm_right = v_right/np.linalg.norm(v_right)
        score.append(np.dot(norm_left, norm_right))

    displacement_new = id_list[np.argmin(score)] - idx
    if not 0 <= id_list[np.argmin(score)] + displacement <= 9:
        displacement = 0
    return id_list[np.argmin(score)] + displacement, displacement_new


if __name__ == "__main__":
    IMG_PATH = "./tracks/loop.png"
    track = load_track(IMG_PATH)

    a = datetime.datetime.now()
    # Set to a valid point in trajectory
    car = PointCar(150, 200)
    trellis = find_valid_trajectory(car, track)

    visible = len(trellis)
    iter_num = 1000
    cont_p_idx = [round(trellis[1].shape[0]/2)] * len(trellis)
    displacement = [0] * len(trellis)
    for i in range(iter_num):
        for j in range(visible):
            left_idx = (j-1)%(visible-1)
            right_idx = (j+1)%(visible-1)
            left_p = trellis[left_idx][cont_p_idx[left_idx]]
            right_p = trellis[right_idx][cont_p_idx[right_idx]]
            cont_p_idx[j], displacement[j] = return_better(trellis[j], cont_p_idx[j], left_p, right_p, displacement[j])

    x = np.array([trellis[i][cont_p_idx[i]][0] for i in range(len(trellis))])
    y = np.array([trellis[i][cont_p_idx[i]][1] for i in range(len(trellis))])

    t = range(len(x))
    ipl_t = np.linspace(0.0, len(x) - 1, 100)

    x_tup = si.splrep(t, x, k=3)
    y_tup = si.splrep(t, y, k=3)

    x_list = list(x_tup)
    xl = x.tolist()
    x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

    y_list = list(y_tup)
    yl = y.tolist()
    y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

    x_i = si.splev(ipl_t, x_list)
    y_i = si.splev(ipl_t, y_list)

    print(metrics.summed_distance(x_i, y_i))

    fig = plt.figure()
    plt.imshow(track)
    # plt.title("Baseline: Centerline Trajectory")
    plt.xlabel("Unit distance")
    plt.ylabel("Unit distance")
    plt.scatter(x_i, y_i)
    plt.show()
