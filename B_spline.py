import numpy as np
import scipy.interpolate as interpolate
import matplotlib.pyplot as plt
from track import load_track
from vehicles import PointCar
from path_finding import find_valid_trajectory
from distance_viterbi import dynamics
import datetime
import scipy.interpolate as si


if __name__ == "__main__":
    IMG_PATH = "./tracks/loop.png"
    track = load_track(IMG_PATH)

    a = datetime.datetime.now()
    # Set to a valid point in trajectory
    car = PointCar(150, 200)
    trellis = find_valid_trajectory(car, track)

    x = np.array([trellis[i][int(np.random.rand(1)*10)][0] for i in range(len(trellis))])
    y = np.array([trellis[i][int(np.random.rand(1)*10)][1] for i in range(len(trellis))])

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

    fig = plt.figure()
    plt.plot(x, y, '-og')
    plt.plot(x_i, y_i, 'r')
    plt.xlim([min(x) - 0.3, max(x) + 0.3])
    plt.ylim([min(y) - 0.3, max(y) + 0.3])
    plt.title('Splined f(x(t), y(t))')
    plt.show()